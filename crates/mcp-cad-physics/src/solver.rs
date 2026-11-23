//! Multi-body constraint and dynamics solver in double precision (`f64`) (`P4-T3` / `ADR-004`).
//!
//! Implements the coupled Newton–Euler + Lagrange Multiplier (`DirectKktIndex2`) formulation:
//! 1. **Block-Diagonal Mass-Inertia Assembly**:
//!    $$M(\mathbf{q}) = \operatorname{diag}(m_1 I_3, I_{w,1}, \dots, m_n I_3, I_{w,n}), \qquad
//!    I_{w,i} = R_i I_{\text{loc},i} R_i^T.$$
//! 2. **Unconstrained Velocity Predictor**:
//!    $$\mathbf{v}^* = \mathbf{v}_k + M^{-1}\left(\mathbf{f}_{\text{ext}} + \mathbf{f}_g - \boldsymbol{\omega}_k \times (I_w \boldsymbol{\omega}_k)\right)\Delta t.$$
//! 3. **Coupled Velocity KKT + Box-LCP Solve**:
//!    Assembles bilateral holonomic rows (`Fixed`, `Spherical`, `Revolute`, `Prismatic`),
//!    direct coupled `Gear` velocity constraints ($\omega_b \cdot \hat{\mathbf{z}}_b + \text{ratio}\,(\omega_a \cdot \hat{\mathbf{z}}_a) = 0$),
//!    unilateral and zero-range joint travel `limits`, bounded velocity actuators (`MotorConfig`),
//!    and unilateral contact constraints into a regularized Schur-complement system:
//!    $$\left(J M^{-1} J^T + \Sigma_{\text{CFM}}\right)\boldsymbol{\gamma} = \mathbf{b} - J\mathbf{v}^*, \qquad
//!    \gamma_r \in [\gamma_{r,\min}, \gamma_{r,\max}].$$
//! 4. **Symplectic Midpoint / Exponential-Map Pose Integration**:
//!    Advances COM positions by $\frac{1}{2}(\mathbf{v}_k + \mathbf{v}_{k+1})\Delta t$ and orientations
//!    on $SO(3)$ via $\exp\left(\frac{1}{2}(\boldsymbol{\omega}_k + \boldsymbol{\omega}_{k+1})\Delta t\right) q_k$.
//! 5. **Gauss–Newton Holonomic Manifold Projection ($\Phi(\mathbf{q}_{k+1}) = \mathbf{0}$)**:
//!    Eliminates geometric position/orientation drift on open and closed kinematic chains to `< 1e-12`,
//!    followed by tangent-space velocity projection ($J\mathbf{v}_{k+1} = \mathbf{b}$) and exact
//!    end-of-step Lagrange multiplier reaction force/torque extraction.

use std::collections::{HashMap, HashSet};

use nalgebra::{DMatrix, DVector, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

use crate::collision::{detect_contacts, orthonormal_pair, ContactPoint};
use crate::joint::{
    is_zero_range_limits, local_orthonormal_basis, CadJointType, JointData, JointHandle,
    MotorConfig,
};
use crate::rigid_body::{BodyHandle, BodyRegistry, ColliderRegistry};
use crate::telemetry::{CollisionEventType, CollisionRecord};

/// Numerical configuration for the `f64` multi-body constraint & dynamics solver.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SolverConfig {
    /// Constraint force mixing (Tikhonov regularization) $\epsilon_{\text{CFM}}$ for bilateral rows
    /// to handle overconstrained planar/spatial closed loops smoothly (`1e-12`).
    pub cfm_regularization: f64,
    /// Maximum Gauss–Newton iterations for holonomic position projection $\Phi(\mathbf{q}) = \mathbf{0}$.
    pub position_projection_iters: usize,
    /// Convergence tolerance on $\|\Phi(\mathbf{q})\|_\infty$ during position projection (`1e-13`).
    pub position_projection_tol: f64,
    /// Maximum active-set iterations for the Box-LCP KKT solver.
    pub max_lcp_iters: usize,
    /// Maximum allowable position or velocity norm before declaring numerical divergence.
    pub max_state_norm: f64,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            cfm_regularization: 1e-12,
            position_projection_iters: 6,
            position_projection_tol: 1e-13,
            max_lcp_iters: 64,
            max_state_norm: 1.0e8,
        }
    }
}

/// Persistent state and metadata for a joint managed by the `f64` multi-body solver.
#[derive(Debug, Clone)]
pub struct SolvedJointRecord {
    /// Unique joint identifier.
    pub id: String,
    /// Internal handle of the joint.
    pub handle: JointHandle,
    /// Identifier of the first connected part (`part_a`).
    pub part_a: String,
    /// Identifier of the second connected part (`part_b`).
    pub part_b: String,
    /// Handle of Body A in [`BodyRegistry`].
    pub body_a: BodyHandle,
    /// Handle of Body B in [`BodyRegistry`].
    pub body_b: BodyHandle,
    /// High-level CAD joint specification.
    pub joint_type: CadJointType,
    /// Optional motor configuration.
    pub motor: Option<MotorConfig>,
    /// Validated `f64` joint frame and limit data.
    pub data: JointData,
    /// Initial relative orientation $q_{a,0}^{-1} q_{b,0}$ captured at joint creation.
    pub initial_rel_rot: UnitQuaternion<f64>,
    /// Initial scalar coordinate for `Revolute` ($\text{rad}$) or `Prismatic` ($\text{m}$).
    pub initial_coord: f64,
    /// Accumulated angle of Body A along `local_axis_a` for `Gear` phase tracking ($\text{rad}$).
    pub gear_theta_a: f64,
    /// Accumulated angle of Body B along `local_axis_b` for `Gear` phase tracking ($\text{rad}$).
    pub gear_theta_b: f64,
    /// Accumulated target coordinate for an active unsaturated motor.
    pub motor_accum_coord: f64,
    /// Whether the joint's motor impulse saturated (or was blocked by a limit) on the last step.
    pub motor_saturated: bool,
    /// World-space constraint reaction force $\mathbf{F}_A$ ($\text{N}$) exerted on `part_a`.
    pub last_force_on_a: [f64; 3],
    /// World-space constraint reaction torque $\boldsymbol{\tau}_A$ ($\text{N}\cdot\text{m}$) about `anchor_a` exerted on `part_a`.
    pub last_torque_on_a: [f64; 3],
    /// World-space constraint reaction force $\mathbf{F}_B$ ($\text{N}$) exerted on `part_b`.
    pub last_force_on_b: [f64; 3],
    /// World-space constraint reaction torque $\boldsymbol{\tau}_B$ ($\text{N}\cdot\text{m}$) about `anchor_b` exerted on `part_b`.
    pub last_torque_on_b: [f64; 3],
}

/// Computes the signed rotation vector $\boldsymbol{\theta} \in \mathbb{R}^3$ ($\text{rad}$)
/// from unit quaternion $q$: $\boldsymbol{\theta} = 2\,\operatorname{atan2}(\|\mathbf{v}\|, w)\,\hat{\mathbf{v}}$.
#[must_use]
pub fn rotation_vector(q: &UnitQuaternion<f64>) -> Vector3<f64> {
    let quat = q.quaternion();
    let (w, v) = if quat.w < 0.0 {
        (-quat.w, -quat.imag())
    } else {
        (quat.w, quat.imag())
    };
    let sin_half = v.norm();
    if sin_half < 1e-12 {
        2.0 * v
    } else {
        let angle = 2.0 * sin_half.atan2(w);
        v * (angle / sin_half)
    }
}

/// Role of a constraint row with respect to contact / Coulomb friction handling.
#[derive(Debug, Clone, Copy, PartialEq)]
enum ContactRowRole {
    /// Bilateral/unilateral joint or motor row.
    None,
    /// Unilateral non-penetration contact normal row ($\gamma_n \ge 0$).
    Normal { contact_idx: usize, friction: f64 },
    /// Coupled Coulomb tangential friction row constrained within $\|\boldsymbol{\gamma}_t\|_2 \le \mu \gamma_n$.
    Tangent {
        normal_row: usize,
        pair_row: usize,
        friction: f64,
    },
}

/// Single scalar constraint row in the global velocity Jacobian $J(\mathbf{q})\mathbf{v} = \mathbf{b}$.
#[derive(Debug, Clone)]
struct AssembledConstraintRow {
    /// Index of the owning joint in `joints` (`None` for contact rows).
    joint_idx: Option<usize>,
    /// Index of Body A in the solver's ordered body array.
    body_a: usize,
    /// Index of Body B in the solver's ordered body array.
    body_b: usize,
    /// Linear Jacobian block on Body A ($\mathbb{R}^3$).
    j_lin_a: Vector3<f64>,
    /// Angular Jacobian block on Body A about COM A ($\mathbb{R}^3$).
    j_ang_a: Vector3<f64>,
    /// Linear Jacobian block on Body B ($\mathbb{R}^3$).
    j_lin_b: Vector3<f64>,
    /// Angular Jacobian block on Body B about COM B ($\mathbb{R}^3$).
    j_ang_b: Vector3<f64>,
    /// Target relative velocity bias $b_r$ on the right-hand side.
    rhs_velocity_bias: f64,
    /// Lower impulse bound $\gamma_{\min} = \lambda_{\min}\Delta t$.
    lower_impulse: f64,
    /// Upper impulse bound $\gamma_{\max} = \lambda_{\max}\Delta t$.
    upper_impulse: f64,
    /// Row-specific CFM regularization $\epsilon_r$.
    cfm: f64,
    /// Whether this row represents a motor actuator row.
    is_motor_row: bool,
    /// Contact / Coulomb friction role metadata.
    contact_role: ContactRowRole,
}

/// Double-precision (`f64`) multi-body constraint and dynamics solver (`ADR-004`).
#[derive(Debug, Clone, Default)]
pub struct MultiBodySolver {
    /// Solver numerical parameters.
    pub config: SolverConfig,
    /// Set of `(part_a, part_b)` pairs (in canonical sorted order) that were in contact on the previous step.
    active_contact_pairs: HashSet<(String, String)>,
}

impl MultiBodySolver {
    /// Creates a new [`MultiBodySolver`] with default [`SolverConfig`].
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a new [`MultiBodySolver`] with custom [`SolverConfig`].
    #[must_use]
    pub fn with_config(config: SolverConfig) -> Self {
        Self {
            config,
            active_contact_pairs: HashSet::new(),
        }
    }

    /// Resets cached contact state.
    pub fn reset(&mut self) {
        self.active_contact_pairs.clear();
    }

    /// Advances the multi-body system by one time step `dt` ($\text{s}$) in `f64`.
    ///
    /// Returns `(step_succeeded, collision_events)` where `step_succeeded` is `false` if any
    /// body state becomes non-finite or exceeds `config.max_state_norm`.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        bodies: &mut BodyRegistry,
        colliders: &mut ColliderRegistry,
        joints: &mut [SolvedJointRecord],
        gravity: [f64; 3],
        dt: f64,
        step_index: usize,
        timestamp_s: f64,
    ) -> (bool, Vec<CollisionRecord>) {
        if !dt.is_finite() || dt <= 0.0 {
            return (false, Vec::new());
        }

        let handles: Vec<BodyHandle> = bodies.handles().to_vec();
        let nb = handles.len();
        if nb == 0 {
            return (true, Vec::new());
        }

        // Map BodyHandle -> dense index 0..nb
        let mut handle_to_idx = std::collections::HashMap::with_capacity(nb);
        for (idx, &h) in handles.iter().enumerate() {
            handle_to_idx.insert(h, idx);
        }

        let g_vec = Vector3::new(gravity[0], gravity[1], gravity[2]);

        // Synchronize collider world poses before contact detection
        sync_all_colliders(bodies, colliders);

        // Detect active contacts via broad-phase + f64 narrow-phase collision pipeline
        let col_slice: Vec<_> = colliders.iter().collect();
        let contacts = detect_contacts(bodies, &col_slice, joints, &handle_to_idx);

        // 1. Assemble block-diagonal inverse mass/inertia and unconstrained velocity predictor v*
        let mut v_old = DVector::<f64>::zeros(6 * nb);
        let mut v_star = DVector::<f64>::zeros(6 * nb);
        let mut inv_mass_vec = vec![0.0_f64; nb];
        let mut inv_inertia_world = vec![nalgebra::Matrix3::<f64>::zeros(); nb];

        for (i, &h) in handles.iter().enumerate() {
            let Some(b) = bodies.get(h) else {
                continue;
            };
            let v = b.linear_velocity;
            let w = b.angular_velocity;
            v_old[6 * i] = v.x;
            v_old[6 * i + 1] = v.y;
            v_old[6 * i + 2] = v.z;
            v_old[6 * i + 3] = w.x;
            v_old[6 * i + 4] = w.y;
            v_old[6 * i + 5] = w.z;

            if b.is_dynamic && b.inv_mass > 0.0 {
                inv_mass_vec[i] = b.inv_mass;
                let inv_i_w = b.world_inv_inertia();
                inv_inertia_world[i] = inv_i_w;

                let dv = (g_vec + b.external_force * b.inv_mass) * dt;
                let vp = v + dv;
                let wp = predict_angular_velocity_implicit_midpoint(b, dt);
                v_star[6 * i] = vp.x;
                v_star[6 * i + 1] = vp.y;
                v_star[6 * i + 2] = vp.z;
                v_star[6 * i + 3] = wp.x;
                v_star[6 * i + 4] = wp.y;
                v_star[6 * i + 5] = wp.z;
            }
        }

        // 2. Assemble velocity constraint rows (joints + limits + motors + contacts/friction)
        let rows = assemble_velocity_constraints(
            bodies,
            &handles,
            &handle_to_idx,
            joints,
            &contacts,
            &v_star,
            dt,
            true,
            true,
            self.config.cfm_regularization,
        );

        // 3. Solve coupled KKT / Box-LCP + Coulomb friction cone system for velocity impulses
        let mut v_new = v_star.clone();
        let m = rows.len();
        let mut step_motor_impulses = vec![0.0_f64; joints.len()];
        let mut step_limit_impulses = vec![0.0_f64; joints.len()];
        for j in joints.iter_mut() {
            j.last_force_on_a = [0.0; 3];
            j.last_torque_on_a = [0.0; 3];
            j.last_force_on_b = [0.0; 3];
            j.last_torque_on_b = [0.0; 3];
            j.motor_saturated = false;
        }

        let mut step_impulses = DVector::<f64>::zeros(m);
        if m > 0 {
            let impulses = solve_kkt_system(
                &rows,
                &v_star,
                &inv_mass_vec,
                &inv_inertia_world,
                nb,
                self.config.max_lcp_iters,
            );
            step_impulses = impulses.clone();

            // Apply impulses: v_new = v_star + M^{-1} J^T * gamma
            for (r, row) in rows.iter().enumerate() {
                let imp = impulses[r];
                let ia = row.body_a;
                let ib = row.body_b;

                if inv_mass_vec[ia] > 0.0 {
                    let dv_a = row.j_lin_a * (inv_mass_vec[ia] * imp);
                    let dw_a = inv_inertia_world[ia] * (row.j_ang_a * imp);
                    v_new[6 * ia] += dv_a.x;
                    v_new[6 * ia + 1] += dv_a.y;
                    v_new[6 * ia + 2] += dv_a.z;
                    v_new[6 * ia + 3] += dw_a.x;
                    v_new[6 * ia + 4] += dw_a.y;
                    v_new[6 * ia + 5] += dw_a.z;
                }
                if inv_mass_vec[ib] > 0.0 {
                    let dv_b = row.j_lin_b * (inv_mass_vec[ib] * imp);
                    let dw_b = inv_inertia_world[ib] * (row.j_ang_b * imp);
                    v_new[6 * ib] += dv_b.x;
                    v_new[6 * ib + 1] += dv_b.y;
                    v_new[6 * ib + 2] += dv_b.z;
                    v_new[6 * ib + 3] += dw_b.x;
                    v_new[6 * ib + 4] += dw_b.y;
                    v_new[6 * ib + 5] += dw_b.z;
                }

                // Track motor saturation, step motor/limit impulses, and accumulate reaction forces/torques
                if let Some(j_idx) = row.joint_idx {
                    let lambda = imp / dt;
                    let f_a = row.j_lin_a * lambda;
                    let t_a_com = row.j_ang_a * lambda;
                    let f_b = row.j_lin_b * lambda;
                    let t_b_com = row.j_ang_b * lambda;
                    if row.is_motor_row {
                        let bound = row.upper_impulse.abs();
                        if bound <= 1e-15 || imp.abs() >= bound * (1.0 - 1e-5) {
                            joints[j_idx].motor_saturated = true;
                        }
                        step_motor_impulses[j_idx] += imp;
                    } else if row.lower_impulse.is_finite() || row.upper_impulse.is_finite() {
                        step_limit_impulses[j_idx] += imp;
                    }
                    joints[j_idx].last_force_on_a[0] += f_a.x;
                    joints[j_idx].last_force_on_a[1] += f_a.y;
                    joints[j_idx].last_force_on_a[2] += f_a.z;
                    joints[j_idx].last_torque_on_a[0] += t_a_com.x;
                    joints[j_idx].last_torque_on_a[1] += t_a_com.y;
                    joints[j_idx].last_torque_on_a[2] += t_a_com.z;
                    joints[j_idx].last_force_on_b[0] += f_b.x;
                    joints[j_idx].last_force_on_b[1] += f_b.y;
                    joints[j_idx].last_force_on_b[2] += f_b.z;
                    joints[j_idx].last_torque_on_b[0] += t_b_com.x;
                    joints[j_idx].last_torque_on_b[1] += t_b_com.y;
                    joints[j_idx].last_torque_on_b[2] += t_b_com.z;
                }
            }
        }

        let collision_events = self.update_collision_events(
            &contacts,
            &rows,
            &step_impulses,
            step_index,
            timestamp_s,
        );

        // Convert accumulated COM torque to pure anchor couple for step-level fallback
        convert_com_torque_to_anchor_couple(bodies, joints);

        // 4. Advance Gear and Motor reference coordinates using midpoint velocities
        for j in joints.iter_mut() {
            let Some(&ia) = handle_to_idx.get(&j.body_a) else {
                continue;
            };
            let Some(&ib) = handle_to_idx.get(&j.body_b) else {
                continue;
            };
            let Some(ba) = bodies.get(j.body_a) else {
                continue;
            };
            let Some(bb) = bodies.get(j.body_b) else {
                continue;
            };

            let wa_mid = Vector3::new(
                0.5 * (v_old[6 * ia + 3] + v_new[6 * ia + 3]),
                0.5 * (v_old[6 * ia + 4] + v_new[6 * ia + 4]),
                0.5 * (v_old[6 * ia + 5] + v_new[6 * ia + 5]),
            );
            let wb_mid = Vector3::new(
                0.5 * (v_old[6 * ib + 3] + v_new[6 * ib + 3]),
                0.5 * (v_old[6 * ib + 4] + v_new[6 * ib + 4]),
                0.5 * (v_old[6 * ib + 5] + v_new[6 * ib + 5]),
            );

            if matches!(j.joint_type, CadJointType::Gear { .. }) {
                let za_w = ba.orientation * Vector3::from(j.data.local_axis_a);
                let zb_w = bb.orientation * Vector3::from(j.data.local_axis_b);
                j.gear_theta_a += wa_mid.dot(&za_w) * dt;
                j.gear_theta_b += wb_mid.dot(&zb_w) * dt;
            }

            if let Some(m) = j.motor {
                if !j.motor_saturated && m.max_force > 0.0 {
                    j.motor_accum_coord += m.target_velocity * dt;
                }
            }
        }

        // 5. Symplectic Midpoint (Trapezoidal) Position & Exponential-Map Orientation Integration
        for (i, &h) in handles.iter().enumerate() {
            let Some(b) = bodies.get_mut(h) else {
                continue;
            };
            if !b.is_dynamic {
                b.linear_velocity = Vector3::zeros();
                b.angular_velocity = Vector3::zeros();
                continue;
            }
            let v_next = Vector3::new(v_new[6 * i], v_new[6 * i + 1], v_new[6 * i + 2]);
            let w_next = Vector3::new(v_new[6 * i + 3], v_new[6 * i + 4], v_new[6 * i + 5]);

            let v_avg = 0.5 * (b.linear_velocity + v_next);
            let w_avg = 0.5 * (b.angular_velocity + w_next);

            let q_old = b.orientation;
            let w_body_next = q_old.inverse() * w_next;

            let x_com_old = b.world_com();
            let x_com_new = x_com_old + v_avg * dt;
            let dq = UnitQuaternion::from_scaled_axis(w_avg * dt);
            let q_new = UnitQuaternion::from_quaternion((dq * q_old).quaternion().normalize());

            b.orientation = q_new;
            b.position = x_com_new - q_new * b.local_com;
            b.linear_velocity = v_next;
            b.angular_velocity = q_new * w_body_next;
        }

        // Depenetrate any resting/colliding contact pairs
        resolve_contact_penetrations(bodies, &handles, &contacts);

        // 6. Gauss–Newton Holonomic Position Manifold Projection Phi(q_{k+1}) = 0
        project_holonomic_manifold(
            bodies,
            &handles,
            &handle_to_idx,
            joints,
            self.config.position_projection_iters,
            self.config.position_projection_tol,
            self.config.cfm_regularization,
        );

        // Sync motor_accum_coord with actual post-step coordinate if motor saturated or hit a limit
        for j in joints.iter_mut() {
            let (Some(ba), Some(bb)) = (bodies.get(j.body_a), bodies.get(j.body_b)) else {
                continue;
            };
            if let Some(cur_c) = j.joint_type.current_coordinate(
                ba.position,
                ba.rotation_matrix(),
                bb.position,
                bb.rotation_matrix(),
            ) {
                if let Some([min_l, max_l]) = j.data.limits {
                    if j.motor_accum_coord <= min_l || j.motor_accum_coord >= max_l {
                        j.motor_accum_coord = j.motor_accum_coord.clamp(min_l, max_l);
                        j.motor_saturated = true;
                    }
                }
                if j.motor_saturated {
                    j.motor_accum_coord = cur_c;
                }
            }
        }

        // 7. Tangent-Space Velocity Projection & Exact End-of-Step Reaction Extraction
        project_velocity_and_compute_reactions(
            bodies,
            &handles,
            &handle_to_idx,
            joints,
            &contacts,
            &step_motor_impulses,
            &step_limit_impulses,
            g_vec,
            dt,
            self.config.cfm_regularization,
            self.config.max_lcp_iters,
        );

        // Synchronize collider poses and clear per-step forces
        sync_all_colliders(bodies, colliders);

        // 8. Check numerical stability / divergence
        let max_norm = self.config.max_state_norm;
        let mut success = true;
        for &h in &handles {
            if let Some(b) = bodies.get(h) {
                let p = b.position;
                let v = b.linear_velocity;
                let w = b.angular_velocity;
                let q = b.orientation.coords;
                if !p.x.is_finite()
                    || !p.y.is_finite()
                    || !p.z.is_finite()
                    || !v.x.is_finite()
                    || !v.y.is_finite()
                    || !v.z.is_finite()
                    || !w.x.is_finite()
                    || !w.y.is_finite()
                    || !w.z.is_finite()
                    || !q.x.is_finite()
                    || !q.y.is_finite()
                    || !q.z.is_finite()
                    || !q.w.is_finite()
                    || p.norm() > max_norm
                    || v.norm() > max_norm
                    || w.norm() > max_norm
                {
                    success = false;
                    break;
                }
            }
        }

        (success, collision_events)
    }

    fn update_collision_events(
        &mut self,
        contacts: &[ContactPoint],
        rows: &[AssembledConstraintRow],
        impulses: &DVector<f64>,
        _step_index: usize,
        timestamp_s: f64,
    ) -> Vec<CollisionRecord> {
        // Gather solved normal impulse per contact index
        let mut contact_normal_impulses = vec![0.0_f64; contacts.len()];
        for (r, row) in rows.iter().enumerate() {
            if let ContactRowRole::Normal { contact_idx, .. } = row.contact_role {
                if let Some(slot) = contact_normal_impulses.get_mut(contact_idx) {
                    if r < impulses.len() {
                        *slot = impulses[r].max(0.0);
                    }
                }
            }
        }

        let mut current_pairs = HashSet::new();
        let mut pair_normals: HashMap<(String, String), ([f64; 3], f64)> = HashMap::new();
        let mut pair_impulses: HashMap<(String, String), f64> = HashMap::new();

        for (c_idx, c) in contacts.iter().enumerate() {
            let (pair, n) = if c.part_a <= c.part_b {
                (
                    (c.part_a.clone(), c.part_b.clone()),
                    [c.normal.x, c.normal.y, c.normal.z],
                )
            } else {
                (
                    (c.part_b.clone(), c.part_a.clone()),
                    [-c.normal.x, -c.normal.y, -c.normal.z],
                )
            };
            current_pairs.insert(pair.clone());
            pair_normals
                .entry(pair.clone())
                .and_modify(|e| {
                    if c.depth > e.1 {
                        *e = (n, c.depth);
                    }
                })
                .or_insert((n, c.depth));
            *pair_impulses.entry(pair).or_insert(0.0) += contact_normal_impulses[c_idx];
        }

        let mut events = Vec::new();
        for pair in &current_pairs {
            if !self.active_contact_pairs.contains(pair) {
                let normal = pair_normals.get(pair).map(|x| x.0);
                let imp = pair_impulses.get(pair).copied();
                events.push(CollisionRecord {
                    time_s: timestamp_s,
                    body_a: pair.0.clone(),
                    body_b: pair.1.clone(),
                    event_type: CollisionEventType::Started,
                    contact_normal: normal,
                    max_impulse: imp,
                });
            }
        }
        for pair in &self.active_contact_pairs {
            if !current_pairs.contains(pair) {
                events.push(CollisionRecord {
                    time_s: timestamp_s,
                    body_a: pair.0.clone(),
                    body_b: pair.1.clone(),
                    event_type: CollisionEventType::Stopped,
                    contact_normal: None,
                    max_impulse: None,
                });
            }
        }
        self.active_contact_pairs = current_pairs;
        events
    }
}

/// Integrates the unconstrained 3D Euler rotational dynamics over time step `dt` using the
/// implicit midpoint (Lie–Poisson) discretization in the body's local frame:
/// $$I_b (\boldsymbol{\Omega}^* - \boldsymbol{\Omega}_k) + \Delta t \left(\boldsymbol{\Omega}_{\text{mid}} \times (I_b \boldsymbol{\Omega}_{\text{mid}}) - \boldsymbol{\tau}_{\text{ext},b}\right) = \mathbf{0}, \qquad \boldsymbol{\Omega}_{\text{mid}} = \frac{\boldsymbol{\Omega}_k + \boldsymbol{\Omega}^*}{2}.$$
///
/// For torque-free 3D asymmetric rigid bodies ($\boldsymbol{\tau}_{\text{ext}} = \mathbf{0}$), this
/// preserves both rotational kinetic energy $T_{\text{rot}} = \frac{1}{2}\boldsymbol{\Omega}\cdot(I_b\boldsymbol{\Omega})$
/// and angular momentum magnitude $\|\mathbf{L}\| = \|I_b\boldsymbol{\Omega}\|$ to machine precision (`~1e-15`).
fn predict_angular_velocity_implicit_midpoint(
    body: &crate::rigid_body::RigidBody,
    dt: f64,
) -> Vector3<f64> {
    let q_old = body.orientation;
    let q_inv = q_old.inverse();
    let w_b0 = q_inv * body.angular_velocity;
    let tau_b = q_inv * body.external_torque;
    let i_b = body.local_inertia;
    let inv_i_b = body.local_inv_inertia;

    let gyro0 = w_b0.cross(&(i_b * w_b0));
    let mut w_b = w_b0 + inv_i_b * (tau_b - gyro0) * dt;

    for _ in 0..5 {
        let w_mid = 0.5 * (w_b0 + w_b);
        let m_mid = i_b * w_mid;
        let res = i_b * (w_b - w_b0) + (w_mid.cross(&m_mid) - tau_b) * dt;
        if res.norm_squared() <= 1e-30 {
            break;
        }
        let jac = i_b + 0.5 * dt * (w_mid.cross_matrix() * i_b - m_mid.cross_matrix());
        if let Some(delta) = jac.lu().solve(&res) {
            w_b -= delta;
            if delta.norm_squared() <= 1e-30 {
                break;
            }
        } else {
            break;
        }
    }

    q_old * w_b
}

fn sync_all_colliders(bodies: &BodyRegistry, colliders: &mut ColliderRegistry) {
    let col_handles: Vec<_> = colliders.iter().map(|(h, _)| h).collect();
    for ch in col_handles {
        let parent_pose = colliders
            .get(ch)
            .and_then(|c| c.parent)
            .and_then(|bh| bodies.get(bh))
            .map(|b| (b.position, b.orientation));
        if let (Some((pos, rot)), Some(col)) = (parent_pose, colliders.get_mut(ch)) {
            col.update_world_pose(pos, rot);
        }
    }
}

fn resolve_contact_penetrations(
    bodies: &mut BodyRegistry,
    handles: &[BodyHandle],
    contacts: &[ContactPoint],
) {
    // Group multi-point manifolds by unordered body pair so position correction is applied
    // once per pair using the deepest contact rather than multiplying by manifold point count.
    let mut deepest_by_pair: HashMap<(usize, usize), &ContactPoint> = HashMap::new();
    for c in contacts {
        let pair = if c.body_a <= c.body_b {
            (c.body_a, c.body_b)
        } else {
            (c.body_b, c.body_a)
        };
        deepest_by_pair
            .entry(pair)
            .and_modify(|prev| {
                if c.depth > prev.depth {
                    *prev = c;
                }
            })
            .or_insert(c);
    }

    let mut pairs: Vec<((usize, usize), &ContactPoint)> = deepest_by_pair.into_iter().collect();
    pairs.sort_unstable_by_key(|&(pair, _)| pair);

    for (_, c) in pairs {
        let slop = 1e-5;
        if c.depth <= slop {
            continue;
        }
        let ha = handles[c.body_a];
        let hb = handles[c.body_b];
        let w_a = bodies.get(ha).map_or(0.0, |b| b.inv_mass());
        let w_b = bodies.get(hb).map_or(0.0, |b| b.inv_mass());
        let w_sum = w_a + w_b;
        if w_sum <= 1e-15 {
            continue;
        }
        let corr = (c.depth - slop) * 0.8 / w_sum;
        if w_a > 0.0 {
            if let Some(ba) = bodies.get_mut(ha) {
                ba.position -= c.normal * (w_a * corr);
            }
        }
        if w_b > 0.0 {
            if let Some(bb) = bodies.get_mut(hb) {
                bb.position += c.normal * (w_b * corr);
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn assemble_velocity_constraints(
    bodies: &BodyRegistry,
    handles: &[BodyHandle],
    handle_to_idx: &std::collections::HashMap<BodyHandle, usize>,
    joints: &[SolvedJointRecord],
    contacts: &[ContactPoint],
    v_ref: &DVector<f64>,
    dt: f64,
    include_centripetal_bias: bool,
    include_friction: bool,
    base_cfm: f64,
) -> Vec<AssembledConstraintRow> {
    let mut rows = Vec::with_capacity(joints.len() * 6 + contacts.len() * 3);
    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];

    for (j_idx, j) in joints.iter().enumerate() {
        let (Some(&ia), Some(&ib)) = (handle_to_idx.get(&j.body_a), handle_to_idx.get(&j.body_b))
        else {
            continue;
        };
        let (Some(ba), Some(bb)) = (bodies.get(j.body_a), bodies.get(j.body_b)) else {
            continue;
        };

        let wa = Vector3::new(v_ref[6 * ia + 3], v_ref[6 * ia + 4], v_ref[6 * ia + 5]);
        let wb = Vector3::new(v_ref[6 * ib + 3], v_ref[6 * ib + 4], v_ref[6 * ib + 5]);
        let va = Vector3::new(v_ref[6 * ia], v_ref[6 * ia + 1], v_ref[6 * ia + 2]);
        let vb = Vector3::new(v_ref[6 * ib], v_ref[6 * ib + 1], v_ref[6 * ib + 2]);

        let anchor_a_loc = Vector3::from(j.data.anchor_a);
        let anchor_b_loc = Vector3::from(j.data.anchor_b);
        let r_a = ba.orientation * (anchor_a_loc - ba.local_com);
        let r_b = bb.orientation * (anchor_b_loc - bb.local_com);
        let p_a = ba.world_com() + r_a;
        let p_b = bb.world_com() + r_b;

        let centripetal_lin = if include_centripetal_bias {
            (wa.cross(&wa.cross(&r_a)) - wb.cross(&wb.cross(&r_b))) * dt
        } else {
            Vector3::zeros()
        };

        match &j.joint_type {
            CadJointType::Spherical { .. } => {
                for axis in &axes {
                    rows.push(AssembledConstraintRow {
                        joint_idx: Some(j_idx),
                        body_a: ia,
                        body_b: ib,
                        j_lin_a: -axis,
                        j_ang_a: -r_a.cross(axis),
                        j_lin_b: *axis,
                        j_ang_b: r_b.cross(axis),
                        rhs_velocity_bias: axis.dot(&centripetal_lin),
                        lower_impulse: f64::NEG_INFINITY,
                        upper_impulse: f64::INFINITY,
                        cfm: base_cfm,
                        is_motor_row: false,
                        contact_role: ContactRowRole::None,
                    });
                }
            }

            CadJointType::Fixed { .. } => {
                for axis in &axes {
                    rows.push(AssembledConstraintRow {
                        joint_idx: Some(j_idx),
                        body_a: ia,
                        body_b: ib,
                        j_lin_a: -axis,
                        j_ang_a: -r_a.cross(axis),
                        j_lin_b: *axis,
                        j_ang_b: r_b.cross(axis),
                        rhs_velocity_bias: axis.dot(&centripetal_lin),
                        lower_impulse: f64::NEG_INFINITY,
                        upper_impulse: f64::INFINITY,
                        cfm: base_cfm,
                        is_motor_row: false,
                        contact_role: ContactRowRole::None,
                    });
                }
                for axis in &axes {
                    rows.push(AssembledConstraintRow {
                        joint_idx: Some(j_idx),
                        body_a: ia,
                        body_b: ib,
                        j_lin_a: Vector3::zeros(),
                        j_ang_a: -axis,
                        j_lin_b: Vector3::zeros(),
                        j_ang_b: *axis,
                        rhs_velocity_bias: 0.0,
                        lower_impulse: f64::NEG_INFINITY,
                        upper_impulse: f64::INFINITY,
                        cfm: base_cfm,
                        is_motor_row: false,
                        contact_role: ContactRowRole::None,
                    });
                }
            }

            CadJointType::Revolute { limits, .. } => {
                // 3 translational lock rows at anchor
                for axis in &axes {
                    rows.push(AssembledConstraintRow {
                        joint_idx: Some(j_idx),
                        body_a: ia,
                        body_b: ib,
                        j_lin_a: -axis,
                        j_ang_a: -r_a.cross(axis),
                        j_lin_b: *axis,
                        j_ang_b: r_b.cross(axis),
                        rhs_velocity_bias: axis.dot(&centripetal_lin),
                        lower_impulse: f64::NEG_INFINITY,
                        upper_impulse: f64::INFINITY,
                        cfm: base_cfm,
                        is_motor_row: false,
                        contact_role: ContactRowRole::None,
                    });
                }

                // 2 out-of-axis angular lock rows (u_a . z_b = 0, v_a . z_b = 0)
                let z_a = ba.orientation * Vector3::from(j.data.local_axis_a);
                let u_a = ba.orientation * Vector3::from(j.data.tangent_u);
                let v_a = ba.orientation * Vector3::from(j.data.tangent_v);
                let z_b = bb.orientation * Vector3::from(j.data.local_axis_b);

                let t1 = z_b.cross(&u_a);
                let t2 = z_b.cross(&v_a);
                let ang_bias_1 = if include_centripetal_bias {
                    -(wa.cross(&u_a)).dot(&(wb.cross(&z_b))) * dt
                } else {
                    0.0
                };
                let ang_bias_2 = if include_centripetal_bias {
                    -(wa.cross(&v_a)).dot(&(wb.cross(&z_b))) * dt
                } else {
                    0.0
                };

                rows.push(AssembledConstraintRow {
                    joint_idx: Some(j_idx),
                    body_a: ia,
                    body_b: ib,
                    j_lin_a: Vector3::zeros(),
                    j_ang_a: -t1,
                    j_lin_b: Vector3::zeros(),
                    j_ang_b: t1,
                    rhs_velocity_bias: ang_bias_1,
                    lower_impulse: f64::NEG_INFINITY,
                    upper_impulse: f64::INFINITY,
                    cfm: base_cfm,
                    is_motor_row: false,
                    contact_role: ContactRowRole::None,
                });
                rows.push(AssembledConstraintRow {
                    joint_idx: Some(j_idx),
                    body_a: ia,
                    body_b: ib,
                    j_lin_a: Vector3::zeros(),
                    j_ang_a: -t2,
                    j_lin_b: Vector3::zeros(),
                    j_ang_b: t2,
                    rhs_velocity_bias: ang_bias_2,
                    lower_impulse: f64::NEG_INFINITY,
                    upper_impulse: f64::INFINITY,
                    cfm: base_cfm,
                    is_motor_row: false,
                    contact_role: ContactRowRole::None,
                });

                // Current hinge angle theta (rad)
                let cur_theta = j
                    .joint_type
                    .current_coordinate(
                        ba.position,
                        ba.rotation_matrix(),
                        bb.position,
                        bb.rotation_matrix(),
                    )
                    .unwrap_or(0.0);

                // Limits (zero-range bilateral lock or unilateral box bounds)
                if let Some([min_a, max_a]) = *limits {
                    if is_zero_range_limits(*limits) {
                        let target = 0.5 * (min_a + max_a);
                        rows.push(AssembledConstraintRow {
                            joint_idx: Some(j_idx),
                            body_a: ia,
                            body_b: ib,
                            j_lin_a: Vector3::zeros(),
                            j_ang_a: -z_a,
                            j_lin_b: Vector3::zeros(),
                            j_ang_b: z_a,
                            rhs_velocity_bias: (target - cur_theta) / dt,
                            lower_impulse: f64::NEG_INFINITY,
                            upper_impulse: f64::INFINITY,
                            cfm: base_cfm,
                            is_motor_row: false,
                            contact_role: ContactRowRole::None,
                        });
                    } else {
                        let rel_w = (wb - wa).dot(&z_a);
                        let pred_theta = cur_theta + rel_w * dt;
                        let motor_v = j.motor.map_or(0.0, |m| m.target_velocity);
                        let pred_motor = cur_theta + motor_v * dt;

                        if cur_theta <= min_a + 1e-2
                            || pred_theta <= min_a + 1e-3
                            || pred_motor <= min_a + 1e-3
                        {
                            rows.push(AssembledConstraintRow {
                                joint_idx: Some(j_idx),
                                body_a: ia,
                                body_b: ib,
                                j_lin_a: Vector3::zeros(),
                                j_ang_a: -z_a,
                                j_lin_b: Vector3::zeros(),
                                j_ang_b: z_a,
                                rhs_velocity_bias: (min_a - cur_theta) / dt,
                                lower_impulse: 0.0,
                                upper_impulse: f64::INFINITY,
                                cfm: base_cfm,
                                is_motor_row: false,
                                contact_role: ContactRowRole::None,
                            });
                        }
                        if cur_theta >= max_a - 1e-2
                            || pred_theta >= max_a - 1e-3
                            || pred_motor >= max_a - 1e-3
                        {
                            rows.push(AssembledConstraintRow {
                                joint_idx: Some(j_idx),
                                body_a: ia,
                                body_b: ib,
                                j_lin_a: Vector3::zeros(),
                                j_ang_a: -z_a,
                                j_lin_b: Vector3::zeros(),
                                j_ang_b: z_a,
                                rhs_velocity_bias: (max_a - cur_theta) / dt,
                                lower_impulse: f64::NEG_INFINITY,
                                upper_impulse: 0.0,
                                cfm: base_cfm,
                                is_motor_row: false,
                                contact_role: ContactRowRole::None,
                            });
                        }
                    }
                }

                // Velocity motor along z_a with exact torque saturation [-max_force * dt, +max_force * dt]
                if let Some(m) = j.motor {
                    if m.max_force > 0.0 && !is_zero_range_limits(*limits) {
                        let max_imp = m.max_force * dt;
                        rows.push(AssembledConstraintRow {
                            joint_idx: Some(j_idx),
                            body_a: ia,
                            body_b: ib,
                            j_lin_a: Vector3::zeros(),
                            j_ang_a: -z_a,
                            j_lin_b: Vector3::zeros(),
                            j_ang_b: z_a,
                            rhs_velocity_bias: m.target_velocity,
                            lower_impulse: -max_imp,
                            upper_impulse: max_imp,
                            cfm: base_cfm.max(1e-11),
                            is_motor_row: true,
                            contact_role: ContactRowRole::None,
                        });
                    }
                }
            }

            CadJointType::Prismatic { limits, .. } => {
                let d = p_b - p_a;
                let z_a = ba.orientation * Vector3::from(j.data.local_axis_a);
                let u_a = ba.orientation * Vector3::from(j.data.tangent_u);
                let v_a = ba.orientation * Vector3::from(j.data.tangent_v);

                let r_a_eff = r_a + d;
                let v_rel_ab = (vb + wb.cross(&r_b)) - (va + wa.cross(&r_a_eff));

                // 2 perpendicular translational lock rows along u_a, v_a
                for perp in [&u_a, &v_a] {
                    let coriolis_bias = if include_centripetal_bias {
                        -(2.0 * (wa.cross(perp)).dot(&v_rel_ab)
                            + perp.dot(&(wa.cross(&wa.cross(&r_a_eff)) - wb.cross(&wb.cross(&r_b)))))
                            * dt
                    } else {
                        0.0
                    };
                    rows.push(AssembledConstraintRow {
                        joint_idx: Some(j_idx),
                        body_a: ia,
                        body_b: ib,
                        j_lin_a: -perp,
                        j_ang_a: -r_a_eff.cross(perp),
                        j_lin_b: *perp,
                        j_ang_b: r_b.cross(perp),
                        rhs_velocity_bias: coriolis_bias,
                        lower_impulse: f64::NEG_INFINITY,
                        upper_impulse: f64::INFINITY,
                        cfm: base_cfm,
                        is_motor_row: false,
                        contact_role: ContactRowRole::None,
                    });
                }

                // 3 relative rotation lock rows
                for axis in &axes {
                    rows.push(AssembledConstraintRow {
                        joint_idx: Some(j_idx),
                        body_a: ia,
                        body_b: ib,
                        j_lin_a: Vector3::zeros(),
                        j_ang_a: -axis,
                        j_lin_b: Vector3::zeros(),
                        j_ang_b: *axis,
                        rhs_velocity_bias: 0.0,
                        lower_impulse: f64::NEG_INFINITY,
                        upper_impulse: f64::INFINITY,
                        cfm: base_cfm,
                        is_motor_row: false,
                        contact_role: ContactRowRole::None,
                    });
                }

                let cur_s = d.dot(&z_a);

                if let Some([min_s, max_s]) = *limits {
                    if is_zero_range_limits(*limits) {
                        let target = 0.5 * (min_s + max_s);
                        rows.push(AssembledConstraintRow {
                            joint_idx: Some(j_idx),
                            body_a: ia,
                            body_b: ib,
                            j_lin_a: -z_a,
                            j_ang_a: -r_a_eff.cross(&z_a),
                            j_lin_b: z_a,
                            j_ang_b: r_b.cross(&z_a),
                            rhs_velocity_bias: (target - cur_s) / dt,
                            lower_impulse: f64::NEG_INFINITY,
                            upper_impulse: f64::INFINITY,
                            cfm: base_cfm,
                            is_motor_row: false,
                            contact_role: ContactRowRole::None,
                        });
                    } else {
                        let rel_v = v_rel_ab.dot(&z_a);
                        let pred_s = cur_s + rel_v * dt;
                        let motor_v = j.motor.map_or(0.0, |m| m.target_velocity);
                        let pred_motor = cur_s + motor_v * dt;

                        if cur_s <= min_s + 1e-2
                            || pred_s <= min_s + 1e-3
                            || pred_motor <= min_s + 1e-3
                        {
                            rows.push(AssembledConstraintRow {
                                joint_idx: Some(j_idx),
                                body_a: ia,
                                body_b: ib,
                                j_lin_a: -z_a,
                                j_ang_a: -r_a_eff.cross(&z_a),
                                j_lin_b: z_a,
                                j_ang_b: r_b.cross(&z_a),
                                rhs_velocity_bias: (min_s - cur_s) / dt,
                                lower_impulse: 0.0,
                                upper_impulse: f64::INFINITY,
                                cfm: base_cfm,
                                is_motor_row: false,
                                contact_role: ContactRowRole::None,
                            });
                        }
                        if cur_s >= max_s - 1e-2
                            || pred_s >= max_s - 1e-3
                            || pred_motor >= max_s - 1e-3
                        {
                            rows.push(AssembledConstraintRow {
                                joint_idx: Some(j_idx),
                                body_a: ia,
                                body_b: ib,
                                j_lin_a: -z_a,
                                j_ang_a: -r_a_eff.cross(&z_a),
                                j_lin_b: z_a,
                                j_ang_b: r_b.cross(&z_a),
                                rhs_velocity_bias: (max_s - cur_s) / dt,
                                lower_impulse: f64::NEG_INFINITY,
                                upper_impulse: 0.0,
                                cfm: base_cfm,
                                is_motor_row: false,
                                contact_role: ContactRowRole::None,
                            });
                        }
                    }
                }

                // Velocity motor along z_a with exact force saturation [-max_force * dt, +max_force * dt]
                if let Some(m) = j.motor {
                    if m.max_force > 0.0 && !is_zero_range_limits(*limits) {
                        let max_imp = m.max_force * dt;
                        rows.push(AssembledConstraintRow {
                            joint_idx: Some(j_idx),
                            body_a: ia,
                            body_b: ib,
                            j_lin_a: -z_a,
                            j_ang_a: -r_a_eff.cross(&z_a),
                            j_lin_b: z_a,
                            j_ang_b: r_b.cross(&z_a),
                            rhs_velocity_bias: m.target_velocity,
                            lower_impulse: -max_imp,
                            upper_impulse: max_imp,
                            cfm: base_cfm.max(1e-11),
                            is_motor_row: true,
                            contact_role: ContactRowRole::None,
                        });
                    }
                }
            }

            CadJointType::Gear { ratio, .. } => {
                // Coupled velocity constraint: ratio * (w_a . z_a) + 1.0 * (w_b . z_b) = 0
                let z_a = ba.orientation * Vector3::from(j.data.local_axis_a);
                let z_b = bb.orientation * Vector3::from(j.data.local_axis_b);
                let phase_err = if include_centripetal_bias {
                    j.gear_theta_b + (*ratio) * j.gear_theta_a
                } else {
                    0.0
                };
                rows.push(AssembledConstraintRow {
                    joint_idx: Some(j_idx),
                    body_a: ia,
                    body_b: ib,
                    j_lin_a: Vector3::zeros(),
                    j_ang_a: z_a * (*ratio),
                    j_lin_b: Vector3::zeros(),
                    j_ang_b: z_b,
                    rhs_velocity_bias: -phase_err / dt,
                    lower_impulse: f64::NEG_INFINITY,
                    upper_impulse: f64::INFINITY,
                    cfm: base_cfm,
                    is_motor_row: false,
                    contact_role: ContactRowRole::None,
                });
            }
        }
    }

    // Unilateral contact normal + coupled Coulomb friction constraint rows
    for (c_idx, c) in contacts.iter().enumerate() {
        let ha = handles[c.body_a];
        let hb = handles[c.body_b];
        let (Some(ba), Some(bb)) = (bodies.get(ha), bodies.get(hb)) else {
            continue;
        };
        let r_a = c.point - ba.world_com();
        let r_b = c.point - bb.world_com();

        let va_pre = ba.linear_velocity + ba.angular_velocity.cross(&r_a);
        let vb_pre = bb.linear_velocity + bb.angular_velocity.cross(&r_b);
        let vn_pre = (vb_pre - va_pre).dot(&c.normal);

        let bounce = if include_centripetal_bias && c.restitution > 0.0 && vn_pre < -0.05 {
            -c.restitution * vn_pre
        } else {
            0.0
        };

        let normal_row_idx = rows.len();
        rows.push(AssembledConstraintRow {
            joint_idx: None,
            body_a: c.body_a,
            body_b: c.body_b,
            j_lin_a: -c.normal,
            j_ang_a: -r_a.cross(&c.normal),
            j_lin_b: c.normal,
            j_ang_b: r_b.cross(&c.normal),
            rhs_velocity_bias: bounce,
            lower_impulse: 0.0,
            upper_impulse: f64::INFINITY,
            cfm: base_cfm.max(1e-10),
            is_motor_row: false,
            contact_role: ContactRowRole::Normal {
                contact_idx: c_idx,
                friction: c.friction,
            },
        });

        // Two orthogonal tangential Coulomb friction rows bounded dynamically by |gamma_t|_2 <= mu * gamma_n
        if include_friction && c.friction > 0.0 {
            let ia = c.body_a;
            let ib = c.body_b;
            let va_ref = Vector3::new(v_ref[6 * ia], v_ref[6 * ia + 1], v_ref[6 * ia + 2])
                + Vector3::new(v_ref[6 * ia + 3], v_ref[6 * ia + 4], v_ref[6 * ia + 5]).cross(&r_a);
            let vb_ref = Vector3::new(v_ref[6 * ib], v_ref[6 * ib + 1], v_ref[6 * ib + 2])
                + Vector3::new(v_ref[6 * ib + 3], v_ref[6 * ib + 4], v_ref[6 * ib + 5]).cross(&r_b);
            let v_rel_ref = vb_ref - va_ref;
            let v_tan = v_rel_ref - c.normal * v_rel_ref.dot(&c.normal);
            let slip = v_tan.norm();
            let (t1, t2) = if slip > 1e-8 {
                let d1 = v_tan / slip;
                let d2 = c.normal.cross(&d1).normalize();
                (d1, d2)
            } else {
                orthonormal_pair(&c.normal)
            };

            let t1_row = rows.len();
            let t2_row = t1_row + 1;
            rows.push(AssembledConstraintRow {
                joint_idx: None,
                body_a: c.body_a,
                body_b: c.body_b,
                j_lin_a: -t1,
                j_ang_a: -r_a.cross(&t1),
                j_lin_b: t1,
                j_ang_b: r_b.cross(&t1),
                rhs_velocity_bias: 0.0,
                lower_impulse: 0.0,
                upper_impulse: 0.0,
                cfm: base_cfm.max(1e-10),
                is_motor_row: false,
                contact_role: ContactRowRole::Tangent {
                    normal_row: normal_row_idx,
                    pair_row: t2_row,
                    friction: c.friction,
                },
            });
            rows.push(AssembledConstraintRow {
                joint_idx: None,
                body_a: c.body_a,
                body_b: c.body_b,
                j_lin_a: -t2,
                j_ang_a: -r_a.cross(&t2),
                j_lin_b: t2,
                j_ang_b: r_b.cross(&t2),
                rhs_velocity_bias: 0.0,
                lower_impulse: 0.0,
                upper_impulse: 0.0,
                cfm: base_cfm.max(1e-10),
                is_motor_row: false,
                contact_role: ContactRowRole::Tangent {
                    normal_row: normal_row_idx,
                    pair_row: t1_row,
                    friction: c.friction,
                },
            });
        }
    }

    rows
}

fn solve_kkt_system(
    rows: &[AssembledConstraintRow],
    v_star: &DVector<f64>,
    inv_mass_vec: &[f64],
    inv_inertia_world: &[nalgebra::Matrix3<f64>],
    nb: usize,
    max_lcp_iters: usize,
) -> DVector<f64> {
    let m = rows.len();
    let ndof = 6 * nb;

    // Build J * M^{-1}
    let mut j_mat = DMatrix::<f64>::zeros(m, ndof);
    let mut jm_inv = DMatrix::<f64>::zeros(m, ndof);
    let mut rhs = DVector::<f64>::zeros(m);
    let mut lower = vec![f64::NEG_INFINITY; m];
    let mut upper = vec![f64::INFINITY; m];

    for (r, row) in rows.iter().enumerate() {
        let ia = row.body_a;
        let ib = row.body_b;

        for k in 0..3 {
            j_mat[(r, 6 * ia + k)] += row.j_lin_a[k];
            j_mat[(r, 6 * ia + 3 + k)] += row.j_ang_a[k];
            j_mat[(r, 6 * ib + k)] += row.j_lin_b[k];
            j_mat[(r, 6 * ib + 3 + k)] += row.j_ang_b[k];
        }

        let w_ma = inv_mass_vec[ia];
        let w_ia = &inv_inertia_world[ia];
        let w_mb = inv_mass_vec[ib];
        let w_ib = &inv_inertia_world[ib];

        let jma_lin = row.j_lin_a * w_ma;
        let jma_ang = w_ia * row.j_ang_a;
        let jmb_lin = row.j_lin_b * w_mb;
        let jmb_ang = w_ib * row.j_ang_b;

        for k in 0..3 {
            jm_inv[(r, 6 * ia + k)] += jma_lin[k];
            jm_inv[(r, 6 * ia + 3 + k)] += jma_ang[k];
            jm_inv[(r, 6 * ib + k)] += jmb_lin[k];
            jm_inv[(r, 6 * ib + 3 + k)] += jmb_ang[k];
        }

        let jv = j_mat.row(r).dot(&v_star.transpose());
        rhs[r] = row.rhs_velocity_bias - jv;
        lower[r] = row.lower_impulse;
        upper[r] = row.upper_impulse;
    }

    let mut a_mat = &jm_inv * j_mat.transpose();
    for (r, row) in rows.iter().enumerate() {
        a_mat[(r, r)] += row.cfm;
    }

    solve_kkt_with_coulomb_contacts(rows, &a_mat, &rhs, &mut lower, &mut upper, max_lcp_iters)
}

/// Solves Box-LCP KKT with coupled isotropic Coulomb friction cones
/// $\|\boldsymbol{\gamma}_t\|_2 \le \mu \gamma_n$ ($\gamma_n \ge 0$).
fn solve_kkt_with_coulomb_contacts(
    rows: &[AssembledConstraintRow],
    a_mat: &DMatrix<f64>,
    rhs: &DVector<f64>,
    lower: &mut [f64],
    upper: &mut [f64],
    max_lcp_iters: usize,
) -> DVector<f64> {
    let mut friction_triplets: Vec<(usize, usize, usize, f64)> = Vec::new();
    for (r, row) in rows.iter().enumerate() {
        if let ContactRowRole::Tangent {
            normal_row,
            pair_row,
            friction,
        } = row.contact_role
        {
            if r < pair_row {
                friction_triplets.push((normal_row, r, pair_row, friction));
            }
        }
    }

    // Fast path: preserve exact Cholesky / Box-LCP behavior when no Coulomb tangent rows exist
    if friction_triplets.is_empty() {
        return solve_box_constrained_kkt(a_mat, rhs, lower, upper, max_lcp_iters);
    }

    let m = rhs.len();
    // Pass 0: solve frictionless normal + joint KKT to obtain initial normal impulses gamma_n >= 0
    let mut x = solve_box_constrained_kkt(a_mat, rhs, lower, upper, max_lcp_iters);
    let mut prev_r_cone = vec![0.0_f64; friction_triplets.len()];

    for pass in 0..20 {
        let mut max_cone_change = 0.0_f64;
        for (idx, &(n_row, t1_row, t2_row, mu)) in friction_triplets.iter().enumerate() {
            let raw_cone = mu * x[n_row].max(0.0);
            let r_cone = if pass >= 8 {
                0.5 * prev_r_cone[idx] + 0.5 * raw_cone
            } else {
                raw_cone
            };
            max_cone_change = max_cone_change.max((r_cone - prev_r_cone[idx]).abs());
            prev_r_cone[idx] = r_cone;

            if r_cone <= 1e-15 {
                lower[t1_row] = 0.0;
                upper[t1_row] = 0.0;
                lower[t2_row] = 0.0;
                upper[t2_row] = 0.0;
                continue;
            }

            let mut b_t1 = rhs[t1_row];
            let mut b_t2 = rhs[t2_row];
            for j in 0..m {
                if j != t1_row && j != t2_row {
                    b_t1 -= a_mat[(t1_row, j)] * x[j];
                    b_t2 -= a_mat[(t2_row, j)] * x[j];
                }
            }

            let a11 = a_mat[(t1_row, t1_row)];
            let a12 = a_mat[(t1_row, t2_row)];
            let a22 = a_mat[(t2_row, t2_row)];
            let (gt1, gt2, inside_cone) =
                solve_2x2_coulomb_disk(a11, a12, a22, b_t1, b_t2, r_cone);

            if inside_cone {
                // Sticking inside Coulomb cone: allow full isotropic disk radius along either axis
                lower[t1_row] = -r_cone;
                upper[t1_row] = r_cone;
                lower[t2_row] = -r_cone;
                upper[t2_row] = r_cone;
            } else {
                // Sliding on boundary of Coulomb disk: exact KKT direction opposes post-impulse slip
                let lim1 = gt1.abs().min(r_cone);
                let lim2 = gt2.abs().min(r_cone);
                lower[t1_row] = -lim1;
                upper[t1_row] = lim1;
                lower[t2_row] = -lim2;
                upper[t2_row] = lim2;
            }
        }

        let x_new = solve_box_constrained_kkt(a_mat, rhs, lower, upper, max_lcp_iters);
        let mut max_diff = 0.0_f64;
        for i in 0..m {
            max_diff = max_diff.max((x_new[i] - x[i]).abs());
        }
        x = x_new;

        let mut max_cone_viol = 0.0_f64;
        for &(n_row, t1_row, t2_row, mu) in &friction_triplets {
            let r_cone = mu * x[n_row].max(0.0);
            let t_mag = (x[t1_row] * x[t1_row] + x[t2_row] * x[t2_row]).sqrt();
            max_cone_viol = max_cone_viol.max(t_mag - r_cone);
        }

        if max_diff < 1e-11 && max_cone_change < 1e-11 && max_cone_viol < 1e-10 {
            break;
        }
    }

    // Strict Coulomb cone invariant clamp on tangent impulses
    for &(n_row, t1_row, t2_row, mu) in &friction_triplets {
        let r_cone = mu * x[n_row].max(0.0);
        let t_mag = (x[t1_row] * x[t1_row] + x[t2_row] * x[t2_row]).sqrt();
        if t_mag > r_cone && t_mag > 1e-15 {
            let scale = r_cone / t_mag;
            x[t1_row] *= scale;
            x[t2_row] *= scale;
        }
    }

    x
}

/// Solves $\min_{\|\boldsymbol{\gamma}_t\|_2 \le r} \frac{1}{2}\boldsymbol{\gamma}_t^T A_{tt}\boldsymbol{\gamma}_t - \mathbf{b}_t^T\boldsymbol{\gamma}_t$
/// via safeguarded Moré–Sorensen secular Newton iteration on $(A_{tt} + \lambda I)\boldsymbol{\gamma}_t = \mathbf{b}_t$.
/// Returns `(gt1, gt2, inside_cone)`.
fn solve_2x2_coulomb_disk(
    a11: f64,
    a12: f64,
    a22: f64,
    b1: f64,
    b2: f64,
    r: f64,
) -> (f64, f64, bool) {
    let det = a11 * a22 - a12 * a12;
    let (g1_0, g2_0) = if det.abs() > 1e-24 {
        ((a22 * b1 - a12 * b2) / det, (-a12 * b1 + a11 * b2) / det)
    } else {
        (b1 / a11.abs().max(1e-15), b2 / a22.abs().max(1e-15))
    };
    let mag0 = g1_0.hypot(g2_0);
    if mag0 <= r * (1.0 + 1e-9) {
        return (g1_0, g2_0, true);
    }

    let b_norm = b1.hypot(b2);
    if b_norm <= 1e-20 || r <= 1e-20 {
        return (0.0, 0.0, false);
    }

    let mut lo = 0.0_f64;
    let mut hi = (b_norm / r) * 1.01 + 1e-12;
    let mut lambda = ((b_norm / r) - 0.5 * (a11 + a22)).clamp(lo + 1e-15, hi);
    let mut g1 = g1_0 * (r / mag0);
    let mut g2 = g2_0 * (r / mag0);

    for _ in 0..24 {
        let d1 = a11 + lambda;
        let d2 = a22 + lambda;
        let det_l = d1 * d2 - a12 * a12;
        if det_l.abs() <= 1e-28 {
            lambda = 0.5 * (lo + hi);
            continue;
        }
        g1 = (d2 * b1 - a12 * b2) / det_l;
        g2 = (-a12 * b1 + d1 * b2) / det_l;
        let nrm = g1.hypot(g2);
        if (nrm - r).abs() <= 1e-13 * r {
            let scale = r / nrm.max(1e-20);
            return (g1 * scale, g2 * scale, false);
        }

        if nrm > r {
            lo = lo.max(lambda);
        } else {
            hi = hi.min(lambda);
        }

        let q1 = (d2 * g1 - a12 * g2) / det_l;
        let q2 = (-a12 * g1 + d1 * g2) / det_l;
        let gq = g1 * q1 + g2 * q2;
        let next = if gq > 1e-30 {
            lambda + (nrm * nrm / gq) * ((nrm - r) / r)
        } else {
            0.5 * (lo + hi)
        };
        lambda = if next > lo && next < hi {
            next
        } else {
            0.5 * (lo + hi)
        };
    }

    let nrm = g1.hypot(g2).max(1e-20);
    (g1 * (r / nrm), g2 * (r / nrm), false)
}

/// Active-set + Projected Gauss–Seidel fallback solver for the symmetric positive-definite
/// Box-LCP KKT system $(J M^{-1} J^T + \Sigma_{\text{CFM}})\boldsymbol{\gamma} = \mathbf{r}$,
/// $\gamma_i \in [l_i, u_i]$.
fn solve_box_constrained_kkt(
    a_mat: &DMatrix<f64>,
    rhs: &DVector<f64>,
    lower: &[f64],
    upper: &[f64],
    max_iters: usize,
) -> DVector<f64> {
    let m = rhs.len();
    if m == 0 {
        return DVector::zeros(0);
    }

    // Fast path: purely bilateral unconstrained system
    let has_bounds = lower
        .iter()
        .zip(upper.iter())
        .any(|(&l, &u)| l.is_finite() || u.is_finite());
    if !has_bounds {
        return a_mat
            .clone()
            .cholesky()
            .map(|chol| chol.solve(rhs))
            .or_else(|| a_mat.clone().lu().solve(rhs))
            .unwrap_or_else(|| DVector::zeros(m));
    }

    // Active-set Box-LCP iteration:
    //  0 = free, -1 = clamped at lower, +1 = clamped at upper, 2 = permanently fixed (lower == upper)
    let mut state = vec![0i8; m];
    let mut x = DVector::<f64>::zeros(m);
    for i in 0..m {
        if lower[i].is_finite() && upper[i].is_finite() && (upper[i] - lower[i]).abs() <= 1e-15 {
            state[i] = 2;
            x[i] = 0.5 * (lower[i] + upper[i]);
        }
    }

    for iter in 0..max_iters.min(32) {
        let free_indices: Vec<usize> = (0..m).filter(|&i| state[i] == 0).collect();

        for i in 0..m {
            if state[i] == -1 {
                x[i] = lower[i];
            } else if state[i] == 1 {
                x[i] = upper[i];
            } else if state[i] == 2 {
                x[i] = 0.5 * (lower[i] + upper[i]);
            }
        }

        if !free_indices.is_empty() {
            let nf = free_indices.len();
            let mut a_ff = DMatrix::<f64>::zeros(nf, nf);
            let mut b_f = DVector::<f64>::zeros(nf);

            for (fi, &r) in free_indices.iter().enumerate() {
                let mut rhs_r = rhs[r];
                for c in 0..m {
                    if state[c] != 0 {
                        rhs_r -= a_mat[(r, c)] * x[c];
                    }
                }
                b_f[fi] = rhs_r;
                for (fj, &c) in free_indices.iter().enumerate() {
                    a_ff[(fi, fj)] = a_mat[(r, c)];
                }
            }

            let x_f = a_ff
                .clone()
                .cholesky()
                .map(|chol| chol.solve(&b_f))
                .or_else(|| a_ff.lu().solve(&b_f))
                .unwrap_or_else(|| DVector::zeros(nf));

            for (fi, &r) in free_indices.iter().enumerate() {
                x[r] = x_f[fi];
            }
        }

        let mut changed = false;
        let single_pivot = iter >= 8;
        let mut best_flip_idx: Option<usize> = None;
        let mut best_flip_state: i8 = 0;
        let mut best_flip_score = 0.0_f64;

        for i in 0..m {
            if state[i] == 2 {
                continue;
            }
            if state[i] == 0 {
                if x[i] < lower[i] - 1e-13 {
                    let viol = lower[i] - x[i];
                    if !single_pivot {
                        x[i] = lower[i];
                        state[i] = -1;
                        changed = true;
                    } else if viol > best_flip_score {
                        best_flip_score = viol;
                        best_flip_idx = Some(i);
                        best_flip_state = -1;
                    }
                } else if x[i] > upper[i] + 1e-13 {
                    let viol = x[i] - upper[i];
                    if !single_pivot {
                        x[i] = upper[i];
                        state[i] = 1;
                        changed = true;
                    } else if viol > best_flip_score {
                        best_flip_score = viol;
                        best_flip_idx = Some(i);
                        best_flip_state = 1;
                    }
                }
            } else {
                let residual = a_mat.row(i).dot(&x.transpose()) - rhs[i];
                if state[i] == -1 && residual < -1e-11 {
                    let viol = -residual;
                    if !single_pivot {
                        state[i] = 0;
                        changed = true;
                    } else if viol > best_flip_score {
                        best_flip_score = viol;
                        best_flip_idx = Some(i);
                        best_flip_state = 0;
                    }
                } else if state[i] == 1 && residual > 1e-11 {
                    let viol = residual;
                    if !single_pivot {
                        state[i] = 0;
                        changed = true;
                    } else if viol > best_flip_score {
                        best_flip_score = viol;
                        best_flip_idx = Some(i);
                        best_flip_state = 0;
                    }
                }
            }
        }

        if single_pivot {
            if let Some(idx) = best_flip_idx {
                state[idx] = best_flip_state;
                if best_flip_state == -1 {
                    x[idx] = lower[idx];
                } else if best_flip_state == 1 {
                    x[idx] = upper[idx];
                }
                changed = true;
            }
        }

        if !changed {
            return x;
        }
    }

    // Fallback: Projected Gauss–Seidel polish if active-set cycled
    for _ in 0..(max_iters * 4) {
        for i in 0..m {
            if state[i] == 2 {
                x[i] = 0.5 * (lower[i] + upper[i]);
                continue;
            }
            let diag = a_mat[(i, i)];
            if diag.abs() <= 1e-20 {
                continue;
            }
            let mut sigma = 0.0;
            for j in 0..m {
                if j != i {
                    sigma += a_mat[(i, j)] * x[j];
                }
            }
            let val = (rhs[i] - sigma) / diag;
            x[i] = val.clamp(lower[i], upper[i]);
        }
    }
    x
}

/// Projects generalized positions $(\mathbf{x}_{\text{com}}, q)$ onto the exact holonomic
/// constraint manifold $\Phi(\mathbf{q}) = \mathbf{0}$ using mass-weighted Gauss–Newton iteration.
fn project_holonomic_manifold(
    bodies: &mut BodyRegistry,
    handles: &[BodyHandle],
    handle_to_idx: &std::collections::HashMap<BodyHandle, usize>,
    joints: &[SolvedJointRecord],
    max_iters: usize,
    tol: f64,
    cfm: f64,
) {
    let nb = handles.len();
    let ndof = 6 * nb;
    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];

    let mut trans_joint_counts: std::collections::HashMap<BodyHandle, usize> =
        std::collections::HashMap::new();
    for j in joints {
        if !matches!(j.joint_type, CadJointType::Gear { .. }) {
            *trans_joint_counts.entry(j.body_a).or_insert(0) += 1;
            *trans_joint_counts.entry(j.body_b).or_insert(0) += 1;
        }
    }

    type HolonomicRowEntry = (
        usize,
        usize,
        Vector3<f64>,
        Vector3<f64>,
        Vector3<f64>,
        Vector3<f64>,
    );

    for _iter in 0..max_iters {
        let mut phi_list: Vec<f64> = Vec::with_capacity(joints.len() * 6);
        let mut row_entries: Vec<HolonomicRowEntry> =
            Vec::with_capacity(joints.len() * 6);

        for j in joints {
            let (Some(&ia), Some(&ib)) =
                (handle_to_idx.get(&j.body_a), handle_to_idx.get(&j.body_b))
            else {
                continue;
            };
            let (Some(ba), Some(bb)) = (bodies.get(j.body_a), bodies.get(j.body_b)) else {
                continue;
            };

            let multi_a = trans_joint_counts.get(&j.body_a).copied().unwrap_or(0) > 1;
            let multi_b = trans_joint_counts.get(&j.body_b).copied().unwrap_or(0) > 1;

            let r_a = ba.orientation * (Vector3::from(j.data.anchor_a) - ba.local_com);
            let r_b = bb.orientation * (Vector3::from(j.data.anchor_b) - bb.local_com);
            let r_a_trans = if multi_a { r_a } else { Vector3::zeros() };
            let r_b_trans = if multi_b { r_b } else { Vector3::zeros() };
            let p_a = ba.world_com() + r_a;
            let p_b = bb.world_com() + r_b;
            let dp = p_b - p_a;

            match &j.joint_type {
                CadJointType::Spherical { .. } => {
                    for axis in &axes {
                        phi_list.push(dp.dot(axis));
                        row_entries.push((
                            ia,
                            ib,
                            -axis,
                            -r_a_trans.cross(axis),
                            *axis,
                            r_b_trans.cross(axis),
                        ));
                    }
                }

                CadJointType::Fixed { .. } => {
                    for axis in &axes {
                        phi_list.push(dp.dot(axis));
                        row_entries.push((
                            ia,
                            ib,
                            -axis,
                            -r_a_trans.cross(axis),
                            *axis,
                            r_b_trans.cross(axis),
                        ));
                    }
                    let q_target_b = ba.orientation * j.initial_rel_rot;
                    let q_err = bb.orientation * q_target_b.inverse();
                    let rot_err = rotation_vector(&q_err);
                    for axis in &axes {
                        phi_list.push(rot_err.dot(axis));
                        row_entries.push((
                            ia,
                            ib,
                            Vector3::zeros(),
                            -axis,
                            Vector3::zeros(),
                            *axis,
                        ));
                    }
                }

                CadJointType::Revolute { limits, .. } => {
                    for axis in &axes {
                        phi_list.push(dp.dot(axis));
                        row_entries.push((
                            ia,
                            ib,
                            -axis,
                            -r_a_trans.cross(axis),
                            *axis,
                            r_b_trans.cross(axis),
                        ));
                    }
                    let z_a = ba.orientation * Vector3::from(j.data.local_axis_a);
                    let u_a = ba.orientation * Vector3::from(j.data.tangent_u);
                    let v_a = ba.orientation * Vector3::from(j.data.tangent_v);
                    let z_b = bb.orientation * Vector3::from(j.data.local_axis_b);

                    let t1 = z_b.cross(&u_a);
                    let t2 = z_b.cross(&v_a);
                    phi_list.push(u_a.dot(&z_b));
                    row_entries.push((ia, ib, Vector3::zeros(), -t1, Vector3::zeros(), t1));
                    phi_list.push(v_a.dot(&z_b));
                    row_entries.push((ia, ib, Vector3::zeros(), -t2, Vector3::zeros(), t2));

                    let cur_theta = j
                        .joint_type
                        .current_coordinate(
                            ba.position,
                            ba.rotation_matrix(),
                            bb.position,
                            bb.rotation_matrix(),
                        )
                        .unwrap_or(0.0);

                    if let Some([min_a, max_a]) = *limits {
                        if is_zero_range_limits(*limits) {
                            let target = 0.5 * (min_a + max_a);
                            phi_list.push(cur_theta - target);
                            row_entries.push((
                                ia,
                                ib,
                                Vector3::zeros(),
                                -z_a,
                                Vector3::zeros(),
                                z_a,
                            ));
                        } else if cur_theta < min_a {
                            phi_list.push(cur_theta - min_a);
                            row_entries.push((
                                ia,
                                ib,
                                Vector3::zeros(),
                                -z_a,
                                Vector3::zeros(),
                                z_a,
                            ));
                        } else if cur_theta > max_a {
                            phi_list.push(cur_theta - max_a);
                            row_entries.push((
                                ia,
                                ib,
                                Vector3::zeros(),
                                -z_a,
                                Vector3::zeros(),
                                z_a,
                            ));
                        } else if j.motor.is_some() && !j.motor_saturated {
                            let phi_m = (cur_theta - j.motor_accum_coord).rem_euclid(std::f64::consts::TAU);
                            let phi_wrapped = if phi_m > std::f64::consts::PI {
                                phi_m - std::f64::consts::TAU
                            } else {
                                phi_m
                            };
                            phi_list.push(phi_wrapped);
                            row_entries.push((
                                ia,
                                ib,
                                Vector3::zeros(),
                                -z_a,
                                Vector3::zeros(),
                                z_a,
                            ));
                        }
                    } else if j.motor.is_some() && !j.motor_saturated {
                        let phi_m = (cur_theta - j.motor_accum_coord).rem_euclid(std::f64::consts::TAU);
                        let phi_wrapped = if phi_m > std::f64::consts::PI {
                            phi_m - std::f64::consts::TAU
                        } else {
                            phi_m
                        };
                        phi_list.push(phi_wrapped);
                        row_entries.push((ia, ib, Vector3::zeros(), -z_a, Vector3::zeros(), z_a));
                    }
                }

                CadJointType::Prismatic { limits, .. } => {
                    let z_a = ba.orientation * Vector3::from(j.data.local_axis_a);
                    let u_a = ba.orientation * Vector3::from(j.data.tangent_u);
                    let v_a = ba.orientation * Vector3::from(j.data.tangent_v);
                    let r_a_eff = if multi_a { r_a + dp } else { Vector3::zeros() };
                    let r_b_eff = if multi_b { r_b } else { Vector3::zeros() };

                    for perp in [&u_a, &v_a] {
                        phi_list.push(dp.dot(perp));
                        row_entries.push((
                            ia,
                            ib,
                            -perp,
                            -r_a_eff.cross(perp),
                            *perp,
                            r_b_eff.cross(perp),
                        ));
                    }

                    let q_target_b = ba.orientation * j.initial_rel_rot;
                    let q_err = bb.orientation * q_target_b.inverse();
                    let rot_err = rotation_vector(&q_err);
                    for axis in &axes {
                        phi_list.push(rot_err.dot(axis));
                        row_entries.push((
                            ia,
                            ib,
                            Vector3::zeros(),
                            -axis,
                            Vector3::zeros(),
                            *axis,
                        ));
                    }

                    let cur_s = dp.dot(&z_a);
                    if let Some([min_s, max_s]) = *limits {
                        if is_zero_range_limits(*limits) {
                            let target = 0.5 * (min_s + max_s);
                            phi_list.push(cur_s - target);
                            row_entries.push((
                                ia,
                                ib,
                                -z_a,
                                -r_a_eff.cross(&z_a),
                                z_a,
                                r_b_eff.cross(&z_a),
                            ));
                        } else if cur_s < min_s {
                            phi_list.push(cur_s - min_s);
                            row_entries.push((
                                ia,
                                ib,
                                -z_a,
                                -r_a_eff.cross(&z_a),
                                z_a,
                                r_b_eff.cross(&z_a),
                            ));
                        } else if cur_s > max_s {
                            phi_list.push(cur_s - max_s);
                            row_entries.push((
                                ia,
                                ib,
                                -z_a,
                                -r_a_eff.cross(&z_a),
                                z_a,
                                r_b_eff.cross(&z_a),
                            ));
                        } else if j.motor.is_some() && !j.motor_saturated {
                            phi_list.push(cur_s - j.motor_accum_coord);
                            row_entries.push((
                                ia,
                                ib,
                                -z_a,
                                -r_a_eff.cross(&z_a),
                                z_a,
                                r_b_eff.cross(&z_a),
                            ));
                        }
                    } else if j.motor.is_some() && !j.motor_saturated {
                        phi_list.push(cur_s - j.motor_accum_coord);
                        row_entries.push((
                            ia,
                            ib,
                            -z_a,
                            -r_a_eff.cross(&z_a),
                            z_a,
                            r_b_eff.cross(&z_a),
                        ));
                    }
                }

                CadJointType::Gear { .. } => {}
            }
        }

        let m = phi_list.len();
        if m == 0 {
            break;
        }
        let max_err = phi_list.iter().fold(0.0_f64, |acc, &x| acc.max(x.abs()));
        if max_err <= tol {
            break;
        }

        let phi = DVector::from_vec(phi_list);
        let mut j_mat = DMatrix::<f64>::zeros(m, ndof);
        let mut jm_inv = DMatrix::<f64>::zeros(m, ndof);

        for (r, (ia, ib, jla, jaa, jlb, jab)) in row_entries.iter().enumerate() {
            let ba = bodies.get(handles[*ia]);
            let bb = bodies.get(handles[*ib]);
            let w_ma = ba.map_or(0.0, |b| b.inv_mass());
            let w_ia = ba.map_or_else(nalgebra::Matrix3::zeros, |b| b.world_inv_inertia());
            let w_mb = bb.map_or(0.0, |b| b.inv_mass());
            let w_ib = bb.map_or_else(nalgebra::Matrix3::zeros, |b| b.world_inv_inertia());

            let jma_lin = jla * w_ma;
            let jma_ang = w_ia * jaa;
            let jmb_lin = jlb * w_mb;
            let jmb_ang = w_ib * jab;

            for k in 0..3 {
                j_mat[(r, 6 * ia + k)] += jla[k];
                j_mat[(r, 6 * ia + 3 + k)] += jaa[k];
                j_mat[(r, 6 * ib + k)] += jlb[k];
                j_mat[(r, 6 * ib + 3 + k)] += jab[k];

                jm_inv[(r, 6 * ia + k)] += jma_lin[k];
                jm_inv[(r, 6 * ia + 3 + k)] += jma_ang[k];
                jm_inv[(r, 6 * ib + k)] += jmb_lin[k];
                jm_inv[(r, 6 * ib + 3 + k)] += jmb_ang[k];
            }
        }

        let mut a_mat = &jm_inv * j_mat.transpose();
        for r in 0..m {
            a_mat[(r, r)] += cfm;
        }

        let mu = a_mat
            .clone()
            .cholesky()
            .map(|chol| chol.solve(&(-&phi)))
            .or_else(|| a_mat.lu().solve(&(-&phi)))
            .unwrap_or_else(|| DVector::zeros(m));

        let dq = jm_inv.transpose() * mu;
        for (i, &h) in handles.iter().enumerate() {
            let Some(b) = bodies.get_mut(h) else {
                continue;
            };
            if !b.is_dynamic {
                continue;
            }
            let dx = Vector3::new(dq[6 * i], dq[6 * i + 1], dq[6 * i + 2]);
            let dtheta = Vector3::new(dq[6 * i + 3], dq[6 * i + 4], dq[6 * i + 5]);

            let x_com_new = b.world_com() + dx;
            let drot = UnitQuaternion::from_scaled_axis(dtheta);
            let q_new =
                UnitQuaternion::from_quaternion((drot * b.orientation).quaternion().normalize());

            b.orientation = q_new;
            b.position = x_com_new - q_new * b.local_com;
        }
    }
}

/// Projects velocities onto the tangent space $J(\mathbf{q}_{k+1})\mathbf{v}_{k+1} = \mathbf{b}$
/// and computes exact end-of-step constraint reaction forces (`last_force_on_a`) and anchor
/// couples (`last_torque_on_a`) at the final pose $\mathbf{q}_{k+1}$.
#[allow(clippy::too_many_arguments)]
fn project_velocity_and_compute_reactions(
    bodies: &mut BodyRegistry,
    handles: &[BodyHandle],
    handle_to_idx: &std::collections::HashMap<BodyHandle, usize>,
    joints: &mut [SolvedJointRecord],
    contacts: &[ContactPoint],
    step_motor_impulses: &[f64],
    step_limit_impulses: &[f64],
    g_vec: Vector3<f64>,
    dt: f64,
    cfm: f64,
    max_lcp_iters: usize,
) {
    let nb = handles.len();
    let mut v_cur = DVector::<f64>::zeros(6 * nb);
    let mut inv_mass_vec = vec![0.0_f64; nb];
    let mut inv_inertia_world = vec![nalgebra::Matrix3::<f64>::zeros(); nb];

    let mut trans_joint_counts: std::collections::HashMap<BodyHandle, usize> =
        std::collections::HashMap::new();
    for j in joints.iter() {
        if !matches!(j.joint_type, CadJointType::Gear { .. }) {
            *trans_joint_counts.entry(j.body_a).or_insert(0) += 1;
            *trans_joint_counts.entry(j.body_b).or_insert(0) += 1;
        }
    }

    for (i, &h) in handles.iter().enumerate() {
        let Some(b) = bodies.get(h) else {
            continue;
        };
        v_cur[6 * i] = b.linear_velocity.x;
        v_cur[6 * i + 1] = b.linear_velocity.y;
        v_cur[6 * i + 2] = b.linear_velocity.z;
        v_cur[6 * i + 3] = b.angular_velocity.x;
        v_cur[6 * i + 4] = b.angular_velocity.y;
        v_cur[6 * i + 5] = b.angular_velocity.z;
        if b.is_dynamic && b.inv_mass > 0.0 {
            inv_mass_vec[i] = b.inv_mass;
            inv_inertia_world[i] = b.world_inv_inertia();
        }
    }

    // 1. Project velocities onto tangent manifold (skipping motor rows that saturated during the step,
    //    and omitting tangential friction rows so sliding contacts do not incur duplicate braking impulses)
    let mut proj_rows = assemble_velocity_constraints(
        bodies,
        handles,
        handle_to_idx,
        joints,
        contacts,
        &v_cur,
        dt,
        false,
        false,
        cfm,
    );
    proj_rows.retain(|row| {
        if row.is_motor_row {
            if let Some(j_idx) = row.joint_idx {
                return !joints[j_idx].motor_saturated;
            }
        }
        true
    });

    if !proj_rows.is_empty() {
        let m_proj = proj_rows.len();
        let ndof = 6 * nb;
        let mut j_mat = DMatrix::<f64>::zeros(m_proj, ndof);
        let mut minv_jt_proj = DMatrix::<f64>::zeros(ndof, m_proj);
        let mut b_rhs = DVector::<f64>::zeros(m_proj);
        let mut lower = vec![0.0_f64; m_proj];
        let mut upper = vec![0.0_f64; m_proj];

        for (r, row) in proj_rows.iter().enumerate() {
            let ia = row.body_a;
            let ib = row.body_b;
            let is_pure_trans = row.j_lin_a.norm_squared() > 0.0;
            let multi_a = trans_joint_counts.get(&handles[ia]).copied().unwrap_or(0) > 1;
            let multi_b = trans_joint_counts.get(&handles[ib]).copied().unwrap_or(0) > 1;

            let w_ma = inv_mass_vec[ia];
            let w_ia = &inv_inertia_world[ia];
            let w_mb = inv_mass_vec[ib];
            let w_ib = &inv_inertia_world[ib];

            let jma_lin = row.j_lin_a * w_ma;
            let jma_ang = if row.joint_idx.is_none() || !is_pure_trans || multi_a {
                w_ia * row.j_ang_a
            } else {
                Vector3::zeros()
            };
            let jmb_lin = row.j_lin_b * w_mb;
            let jmb_ang = if row.joint_idx.is_none() || !is_pure_trans || multi_b {
                w_ib * row.j_ang_b
            } else {
                Vector3::zeros()
            };

            for k in 0..3 {
                j_mat[(r, 6 * ia + k)] += row.j_lin_a[k];
                j_mat[(r, 6 * ia + 3 + k)] += row.j_ang_a[k];
                j_mat[(r, 6 * ib + k)] += row.j_lin_b[k];
                j_mat[(r, 6 * ib + 3 + k)] += row.j_ang_b[k];

                minv_jt_proj[(6 * ia + k, r)] += jma_lin[k];
                minv_jt_proj[(6 * ia + 3 + k, r)] += jma_ang[k];
                minv_jt_proj[(6 * ib + k, r)] += jmb_lin[k];
                minv_jt_proj[(6 * ib + 3 + k, r)] += jmb_ang[k];
            }
            b_rhs[r] = row.rhs_velocity_bias;
            lower[r] = row.lower_impulse;
            upper[r] = row.upper_impulse;
        }

        let mut a_proj = &j_mat * &minv_jt_proj;
        for r in 0..m_proj {
            let scale = a_proj[(r, r)].abs().max(1.0);
            a_proj[(r, r)] += proj_rows[r].cfm * scale;
        }
        let vel_deficit = b_rhs - &j_mat * &v_cur;
        let gamma_proj = solve_box_constrained_kkt(&a_proj, &vel_deficit, &lower, &upper, max_lcp_iters);
        let dv_proj = &minv_jt_proj * gamma_proj;
        v_cur += dv_proj;

        for (i, &h) in handles.iter().enumerate() {
            if let Some(b) = bodies.get_mut(h) {
                if b.is_dynamic {
                    b.linear_velocity =
                        Vector3::new(v_cur[6 * i], v_cur[6 * i + 1], v_cur[6 * i + 2]);
                    b.angular_velocity =
                        Vector3::new(v_cur[6 * i + 3], v_cur[6 * i + 4], v_cur[6 * i + 5]);
                }
            }
        }
    }

    // 2. Evaluate exact end-of-step Lagrange multipliers at (q_{k+1}, v_{k+1})
    let mut v_react_star = v_cur.clone();
    for (i, &h) in handles.iter().enumerate() {
        let Some(b) = bodies.get(h) else {
            continue;
        };
        if b.is_dynamic && b.inv_mass > 0.0 {
            let i_w = b.world_inertia();
            let inv_i_w = b.world_inv_inertia();
            let w = b.angular_velocity;
            let dv = (g_vec + b.external_force * b.inv_mass) * dt;
            let dw = inv_i_w * (b.external_torque - w.cross(&(i_w * w))) * dt;
            v_react_star[6 * i] += dv.x;
            v_react_star[6 * i + 1] += dv.y;
            v_react_star[6 * i + 2] += dv.z;
            v_react_star[6 * i + 3] += dw.x;
            v_react_star[6 * i + 4] += dw.y;
            v_react_star[6 * i + 5] += dw.z;
        }
    }

    let mut react_rows = assemble_velocity_constraints(
        bodies,
        handles,
        handle_to_idx,
        joints,
        contacts,
        &v_cur,
        dt,
        true,
        true,
        cfm,
    );
    if !react_rows.is_empty() {
        let mut gamma_react = solve_kkt_system(
            &react_rows,
            &v_react_star,
            &inv_mass_vec,
            &inv_inertia_world,
            nb,
            max_lcp_iters,
        );

        // If any motor actuator or unilateral travel limit applied a larger step impulse during Step 3
        // (e.g. an unsaturated velocity motor spinning up a body in a single step, or an inelastic impact
        // arresting motion at a travel limit so that v_{k+1} already satisfies the target velocity),
        // pin those active actuator/limit impulses and re-solve the coupled KKT system at q_{k+1}
        // so that all bilateral support and gear rows balance the impulse consistently.
        let mut pinned_any = false;
        for (r, row) in react_rows.iter_mut().enumerate() {
            if let Some(j_idx) = row.joint_idx {
                if row.is_motor_row {
                    let step_imp = step_motor_impulses.get(j_idx).copied().unwrap_or(0.0);
                    if step_imp.abs() > gamma_react[r].abs() + 1e-12 {
                        row.lower_impulse = step_imp;
                        row.upper_impulse = step_imp;
                        pinned_any = true;
                    }
                } else if row.lower_impulse.is_finite() || row.upper_impulse.is_finite() {
                    let step_imp = step_limit_impulses.get(j_idx).copied().unwrap_or(0.0);
                    if step_imp.abs() > gamma_react[r].abs() + 1e-12 {
                        row.lower_impulse = step_imp;
                        row.upper_impulse = step_imp;
                        pinned_any = true;
                    }
                }
            }
        }
        if pinned_any {
            gamma_react = solve_kkt_system(
                &react_rows,
                &v_react_star,
                &inv_mass_vec,
                &inv_inertia_world,
                nb,
                max_lcp_iters,
            );
        }

        for j in joints.iter_mut() {
            j.last_force_on_a = [0.0; 3];
            j.last_torque_on_a = [0.0; 3];
            j.last_force_on_b = [0.0; 3];
            j.last_torque_on_b = [0.0; 3];
        }
        for (r, row) in react_rows.iter().enumerate() {
            if let Some(j_idx) = row.joint_idx {
                let lambda = gamma_react[r] / dt;
                let f_a = row.j_lin_a * lambda;
                let t_a_com = row.j_ang_a * lambda;
                let f_b = row.j_lin_b * lambda;
                let t_b_com = row.j_ang_b * lambda;
                joints[j_idx].last_force_on_a[0] += f_a.x;
                joints[j_idx].last_force_on_a[1] += f_a.y;
                joints[j_idx].last_force_on_a[2] += f_a.z;
                joints[j_idx].last_torque_on_a[0] += t_a_com.x;
                joints[j_idx].last_torque_on_a[1] += t_a_com.y;
                joints[j_idx].last_torque_on_a[2] += t_a_com.z;
                joints[j_idx].last_force_on_b[0] += f_b.x;
                joints[j_idx].last_force_on_b[1] += f_b.y;
                joints[j_idx].last_force_on_b[2] += f_b.z;
                joints[j_idx].last_torque_on_b[0] += t_b_com.x;
                joints[j_idx].last_torque_on_b[1] += t_b_com.y;
                joints[j_idx].last_torque_on_b[2] += t_b_com.z;
            }
        }
        convert_com_torque_to_anchor_couple(bodies, joints);
    }

    // Clear per-step external force/torque accumulators
    for &h in handles {
        if let Some(b) = bodies.get_mut(h) {
            b.clear_forces();
        }
    }
}

/// Converts `last_torque_on_a` and `last_torque_on_b` from total moments about Body A's and Body B's
/// centers of mass ($\boldsymbol{\tau}_{A,\text{com}}$, $\boldsymbol{\tau}_{B,\text{com}}$) into pure
/// joint reaction couples about `anchor_a` and `anchor_b`:
/// $\boldsymbol{\tau}_{A,\text{anchor}} = \boldsymbol{\tau}_{A,\text{com}} - (\mathbf{p}_{\text{anchor},a} - \mathbf{x}_{\text{com},a}) \times \mathbf{F}_A$,
/// $\boldsymbol{\tau}_{B,\text{anchor}} = \boldsymbol{\tau}_{B,\text{com}} - (\mathbf{p}_{\text{anchor},b} - \mathbf{x}_{\text{com},b}) \times \mathbf{F}_B$.
fn convert_com_torque_to_anchor_couple(
    bodies: &BodyRegistry,
    joints: &mut [SolvedJointRecord],
) {
    for j in joints.iter_mut() {
        if let Some(ba) = bodies.get(j.body_a) {
            let r_a = ba.orientation * (Vector3::from(j.data.anchor_a) - ba.local_com);
            let f_a = Vector3::from(j.last_force_on_a);
            let t_a_com = Vector3::from(j.last_torque_on_a);
            let t_anchor = t_a_com - r_a.cross(&f_a);
            j.last_torque_on_a = [t_anchor.x, t_anchor.y, t_anchor.z];
        }
        if let Some(bb) = bodies.get(j.body_b) {
            let r_b = bb.orientation * (Vector3::from(j.data.anchor_b) - bb.local_com);
            let f_b = Vector3::from(j.last_force_on_b);
            let t_b_com = Vector3::from(j.last_torque_on_b);
            let t_anchor = t_b_com - r_b.cross(&f_b);
            j.last_torque_on_b = [t_anchor.x, t_anchor.y, t_anchor.z];
        }
    }
}

/// Helper to initialize a [`SolvedJointRecord`] given the current poses of `body_a` and `body_b`.
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn create_solved_joint_record(
    id: String,
    handle: JointHandle,
    part_a: String,
    part_b: String,
    body_a: BodyHandle,
    body_b: BodyHandle,
    joint_type: CadJointType,
    motor: Option<MotorConfig>,
    data: JointData,
    bodies: &BodyRegistry,
) -> SolvedJointRecord {
    let (qa, pa, ra_mat) = bodies.get(body_a).map_or_else(
        || (UnitQuaternion::identity(), Vector3::zeros(), nalgebra::Matrix3::identity()),
        |b| (b.orientation, b.position, b.rotation_matrix()),
    );
    let (qb, pb, rb_mat) = bodies.get(body_b).map_or_else(
        || (UnitQuaternion::identity(), Vector3::zeros(), nalgebra::Matrix3::identity()),
        |b| (b.orientation, b.position, b.rotation_matrix()),
    );

    let initial_rel_rot = qa.inverse() * qb;
    let initial_coord = joint_type
        .current_coordinate(pa, ra_mat, pb, rb_mat)
        .unwrap_or(0.0);

    // Ensure local_orthonormal_basis is linked/used for consistent coordinate conventions
    let _ = local_orthonormal_basis([0.0, 0.0, 1.0], Vector3::z());

    SolvedJointRecord {
        id,
        handle,
        part_a,
        part_b,
        body_a,
        body_b,
        joint_type,
        motor,
        data,
        initial_rel_rot,
        initial_coord,
        gear_theta_a: 0.0,
        gear_theta_b: 0.0,
        motor_accum_coord: initial_coord,
        motor_saturated: false,
        last_force_on_a: [0.0; 3],
        last_torque_on_a: [0.0; 3],
        last_force_on_b: [0.0; 3],
        last_torque_on_b: [0.0; 3],
    }
}

// ============================================================================
// Deformable FEM Hub Coupling Bridge (P5-T4)
// ============================================================================

/// Kinematic joint family exported for coupling a deformable body's hub/bore nodes
/// to `f64` joints and actuators managed by [`MultiBodySolver`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HubJointKind {
    /// 1-DOF rotational shaft joint around `axis_world` through `pivot_world`.
    Revolute,
    /// 1-DOF translational slide joint along `axis_world`.
    Prismatic,
    /// 0-DOF rigid mounting hub constraint.
    Fixed,
}

/// Serializable `f64` kinematic hub coupling specification extracted from a [`SolvedJointRecord`],
/// linking a deformable FEA body's shaft bore / hub surface nodes to an `f64` joint & motor
/// without artificially locking free regions such as gear teeth.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointHubCouplingSpec {
    /// Joint identifier in the physics assembly.
    pub joint_id: String,
    /// Identifier of the deformable target part.
    pub part_id: String,
    /// Joint kinematic family.
    pub kind: HubJointKind,
    /// Joint pivot point in world coordinates ($\text{m}$).
    pub pivot_world: [f64; 3],
    /// Unit joint axis in world coordinates (rotation or sliding axis).
    pub axis_world: [f64; 3],
    /// Current scalar joint coordinate ($\text{rad}$ for `Revolute`, $\text{m}$ for `Prismatic`).
    pub current_coordinate: f64,
    /// Optional travel limits `[min, max]` ($\text{rad}$ or $\text{m}$).
    pub limits: Option<[f64; 2]>,
    /// Optional motor target velocity ($\text{rad/s}$ or $\text{m/s}$).
    pub motor_target_velocity: Option<f64>,
    /// Optional motor maximum actuation torque ($\text{N}\cdot\text{m}$) or force ($\text{N}$).
    pub motor_max_effort: Option<f64>,
    /// Last solved axial reaction torque ($\text{N}\cdot\text{m}$) or force ($\text{N}$) along `axis_world`.
    pub last_axial_effort: f64,
}

/// Extracts deformable hub coupling descriptors for all ground-connected or multi-body
/// `Revolute`, `Prismatic`, and `Fixed` joints acting on dynamic bodies in `bodies`.
#[must_use]
pub fn extract_joint_hub_couplings(
    bodies: &BodyRegistry,
    joints: &[SolvedJointRecord],
) -> Vec<JointHubCouplingSpec> {
    let mut specs = Vec::new();
    for j in joints {
        let (Some(ba), Some(bb)) = (bodies.get(j.body_a), bodies.get(j.body_b)) else {
            continue;
        };

        let kind = match j.joint_type {
            CadJointType::Revolute { .. } => HubJointKind::Revolute,
            CadJointType::Prismatic { .. } => HubJointKind::Prismatic,
            CadJointType::Fixed { .. } => HubJointKind::Fixed,
            CadJointType::Spherical { .. } | CadJointType::Gear { .. } => continue,
        };

        let current_coord = j
            .joint_type
            .current_coordinate(
                ba.position,
                ba.rotation_matrix(),
                bb.position,
                bb.rotation_matrix(),
            )
            .unwrap_or(0.0);

        // Emit coupling for the dynamic body (or Body B if both are dynamic).
        // In MultiBodySolver, relative joint velocity is (v_b - v_a) · axis, so if only Body A
        // is dynamic we negate coordinate/motor signs so positive velocity always moves `target_part`
        // along `+axis_world`.
        let (target_part, pivot_w, axis_w, force_w, torque_w, coord_sign) = if bb.is_dynamic {
            let mut pivot = bb.position + bb.orientation * Vector3::from(j.data.anchor_b);
            if !ba.is_dynamic {
                let pivot_a = ba.position + ba.orientation * Vector3::from(j.data.anchor_a);
                let anchor_b_zero = Vector3::from(j.data.anchor_b).norm() < 1e-9;
                if bb.position.norm() < 1e-9
                    && anchor_b_zero
                    && (pivot_a - bb.local_com).norm() + 1e-6 < (pivot - bb.local_com).norm()
                {
                    pivot = pivot_a;
                }
            }
            let axis_raw = bb.orientation * Vector3::from(j.data.local_axis_b);
            let axis = if axis_raw.norm() > 1e-12 {
                axis_raw.normalize()
            } else {
                Vector3::z()
            };
            (
                j.part_b.clone(),
                pivot,
                axis,
                Vector3::from(j.last_force_on_b),
                Vector3::from(j.last_torque_on_b),
                1.0_f64,
            )
        } else if ba.is_dynamic {
            let mut pivot = ba.position + ba.orientation * Vector3::from(j.data.anchor_a);
            if !bb.is_dynamic {
                let pivot_b = bb.position + bb.orientation * Vector3::from(j.data.anchor_b);
                let anchor_a_zero = Vector3::from(j.data.anchor_a).norm() < 1e-9;
                if ba.position.norm() < 1e-9
                    && anchor_a_zero
                    && (pivot_b - ba.local_com).norm() + 1e-6 < (pivot - ba.local_com).norm()
                {
                    pivot = pivot_b;
                }
            }
            let axis_raw = ba.orientation * Vector3::from(j.data.local_axis_a);
            let axis = if axis_raw.norm() > 1e-12 {
                axis_raw.normalize()
            } else {
                Vector3::z()
            };
            (
                j.part_a.clone(),
                pivot,
                axis,
                Vector3::from(j.last_force_on_a),
                Vector3::from(j.last_torque_on_a),
                -1.0_f64,
            )
        } else {
            continue;
        };

        let last_axial_effort = match kind {
            HubJointKind::Revolute => torque_w.dot(&axis_w),
            HubJointKind::Prismatic => force_w.dot(&axis_w),
            HubJointKind::Fixed => torque_w.norm(),
        };
        let limits = j.data.limits.map(|[lo, hi]| {
            if coord_sign < 0.0 {
                [-hi, -lo]
            } else {
                [lo, hi]
            }
        });

        specs.push(JointHubCouplingSpec {
            joint_id: j.id.clone(),
            part_id: target_part,
            kind,
            pivot_world: [pivot_w.x, pivot_w.y, pivot_w.z],
            axis_world: [axis_w.x, axis_w.y, axis_w.z],
            current_coordinate: current_coord * coord_sign,
            limits,
            motor_target_velocity: j.motor.map(|m| m.target_velocity * coord_sign),
            motor_max_effort: j.motor.map(|m| m.max_force),
            last_axial_effort,
        });
    }
    specs
}

/// Serializable `f64` gear-mesh descriptor extracted from a [`CadJointType::Gear`] joint
/// in [`SolvedJointRecord`] (`P5-T14`), used by coupled dynamic FEA to identify meshing gear pairs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GearJointCouplingSpec {
    /// Joint identifier in the physics assembly.
    pub joint_id: String,
    /// First meshing gear part identifier (typically pinion / driver).
    pub part_a: String,
    /// Second meshing gear part identifier (typically driven gear).
    pub part_b: String,
    /// Kinematic speed ratio $r = Z_a / Z_b$ (for external gears $\omega_b = -r \omega_a$).
    pub ratio: f64,
    /// Unit rotation axis of `part_a` in world coordinates.
    pub axis_a_world: [f64; 3],
    /// Unit rotation axis of `part_b` in world coordinates.
    pub axis_b_world: [f64; 3],
}

/// Extracts all [`CadJointType::Gear`] descriptors from the active solved joint records (`P5-T14`).
#[must_use]
pub fn extract_gear_joint_couplings(
    bodies: &BodyRegistry,
    joints: &[SolvedJointRecord],
) -> Vec<GearJointCouplingSpec> {
    let mut specs = Vec::new();
    for j in joints {
        let CadJointType::Gear { ratio, .. } = j.joint_type else {
            continue;
        };
        let (Some(ba), Some(bb)) = (bodies.get(j.body_a), bodies.get(j.body_b)) else {
            continue;
        };
        let ax_a_raw = ba.orientation * Vector3::from(j.data.local_axis_a);
        let ax_b_raw = bb.orientation * Vector3::from(j.data.local_axis_b);
        let ax_a = if ax_a_raw.norm() > 1e-12 {
            ax_a_raw.normalize()
        } else {
            Vector3::z()
        };
        let ax_b = if ax_b_raw.norm() > 1e-12 {
            ax_b_raw.normalize()
        } else {
            Vector3::z()
        };
        specs.push(GearJointCouplingSpec {
            joint_id: j.id.clone(),
            part_a: j.part_a.clone(),
            part_b: j.part_b.clone(),
            ratio,
            axis_a_world: [ax_a.x, ax_a.y, ax_a.z],
            axis_b_world: [ax_b.x, ax_b.y, ax_b.z],
        });
    }
    specs
}

/// Updates a `TriMesh` collider's local vertices from newly deformed FEM surface coordinates,
/// rebuilding its local AABB and cached [`crate::collision::TriMeshBvh`] so subsequent collision
/// queries see the true deformed geometry (`P5-T4`).
pub fn sync_deformed_trimesh_collider(
    colliders: &mut ColliderRegistry,
    handle: crate::rigid_body::ColliderHandle,
    deformed_local_vertices: Vec<[f64; 3]>,
) -> bool {
    let Some(col) = colliders.get_mut(handle) else {
        return false;
    };
    let indices_clone = match &mut col.shape {
        crate::rigid_body::ColliderShape::TriMesh { vertices, indices } => {
            *vertices = deformed_local_vertices;
            indices.clone()
        }
        _ => return false,
    };
    if let crate::rigid_body::ColliderShape::TriMesh { vertices, .. } = &col.shape {
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for v in vertices {
            for k in 0..3 {
                min[k] = min[k].min(v[k]);
                max[k] = max[k].max(v[k]);
            }
        }
        if vertices.is_empty() {
            min = [0.0; 3];
            max = [0.0; 3];
        }
        col.local_aabb = mcp_cad_core::topology::BoundingBox::new(min, max);
        col.tri_bvh = Some(crate::collision::TriMeshBvh::build(vertices, &indices_clone));
        col.world_aabb = col.local_aabb;
        return true;
    }
    false
}

#[cfg(test)]
mod p5_t4_coupling_tests {
    use super::*;
    use crate::rigid_body::{Collider, ColliderShape, ColliderType, RigidBody};
    use mcp_cad_core::dag::FeatureOp;
    use mcp_cad_core::material::MaterialRegistry;
    use mcp_cad_core::Part;

    fn make_box_part(id: &str) -> Part {
        let reg = MaterialRegistry::new();
        let steel = reg.get("steel_structural_aisi1020").unwrap().clone();
        let mut p = Part::new(id, id, steel);
        p.dag.add_feature(
            FeatureOp::AddBox {
                width: 0.04,
                height: 0.04,
                depth: 0.01,
                position: None,
                rotation_deg: None,
            },
            None,
        );
        p.rebuild().unwrap();
        p
    }

    #[test]
    fn test_p5_t4_extract_joint_hub_couplings_and_deformed_trimesh_sync() {
        let base_part = make_box_part("ground");
        let rotor_part = make_box_part("rotor");

        let mut bodies = BodyRegistry::new();
        let h_ground = bodies.insert(RigidBody::new_fixed(
            "ground",
            Vector3::zeros(),
            UnitQuaternion::identity(),
        ));
        let mut rb_rotor = RigidBody::from_part(&rotor_part, true, None, None);
        rb_rotor.set_pose(
            Vector3::new(0.05, 0.0, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.25),
        );
        let h_rotor = bodies.insert(rb_rotor);

        let jtype = CadJointType::Revolute {
            anchor_a: [0.05, 0.0, 0.0],
            anchor_b: [0.0, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            limits: Some([-1.5, 1.5]),
        };
        let motor = MotorConfig {
            target_velocity: 2.0,
            max_force: 15.0,
        };
        let jdata = jtype.build_generic_joint(Some(motor)).unwrap();
        let rec = SolvedJointRecord {
            id: "shaft_rev".to_string(),
            handle: JointHandle(0),
            part_a: "ground".to_string(),
            part_b: "rotor".to_string(),
            body_a: h_ground,
            body_b: h_rotor,
            joint_type: jtype,
            motor: Some(motor),
            data: jdata,
            initial_rel_rot: UnitQuaternion::identity(),
            initial_coord: 0.0,
            gear_theta_a: 0.0,
            gear_theta_b: 0.0,
            motor_accum_coord: 0.25,
            motor_saturated: false,
            last_force_on_a: [0.0; 3],
            last_torque_on_a: [0.0, 0.0, -9.5],
            last_force_on_b: [0.0; 3],
            last_torque_on_b: [0.0, 0.0, 9.5],
        };

        let couplings = extract_joint_hub_couplings(&bodies, &[rec]);
        assert_eq!(couplings.len(), 1);
        let c = &couplings[0];
        assert_eq!(c.joint_id, "shaft_rev");
        assert_eq!(c.part_id, "rotor");
        assert_eq!(c.kind, HubJointKind::Revolute);
        assert!((c.pivot_world[0] - 0.05).abs() < 1e-12);
        assert!((c.axis_world[2] - 1.0).abs() < 1e-12);
        assert!((c.current_coordinate - 0.25).abs() < 1e-9);
        assert_eq!(c.motor_target_velocity, Some(2.0));
        assert_eq!(c.motor_max_effort, Some(15.0));
        assert!((c.last_axial_effort - 9.5).abs() < 1e-12);

        // Verify dynamic deformed TriMesh collider update + BVH rebuild
        let mut colliders = ColliderRegistry::new();
        let col = Collider::from_part(&base_part, ColliderType::TriMesh).unwrap();
        let col_handle = colliders.insert_with_parent(col, h_ground, &mut bodies);

        let orig_verts = match &colliders.get(col_handle).unwrap().shape {
            ColliderShape::TriMesh { vertices, .. } => vertices.clone(),
            _ => panic!("expected trimesh"),
        };
        let deformed_verts: Vec<[f64; 3]> = orig_verts
            .iter()
            .map(|&[x, y, z]| [x, y - 0.002, z])
            .collect();
        assert!(sync_deformed_trimesh_collider(
            &mut colliders,
            col_handle,
            deformed_verts
        ));
        let updated = colliders.get(col_handle).unwrap();
        assert!((updated.local_aabb.max[1] - 0.018).abs() < 1e-9);
        assert!(updated.tri_bvh.is_some());
    }
}


