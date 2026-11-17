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

use std::collections::HashSet;

use nalgebra::{DMatrix, DVector, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

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
}

/// Detected overlap between two colliders for contact constraint resolution.
#[derive(Debug, Clone)]
struct DetectedContact {
    /// Index of Body A in the solver's body list.
    body_a: usize,
    /// Index of Body B in the solver's body list.
    body_b: usize,
    /// Part ID of Body A.
    part_a: String,
    /// Part ID of Body B.
    part_b: String,
    /// Contact point in world coordinates ($\text{m}$).
    point: Vector3<f64>,
    /// Unit contact normal pointing from Body A to Body B.
    normal: Vector3<f64>,
    /// Penetration depth $\delta > 0$ ($\text{m}$).
    depth: f64,
    /// Combined coefficient of restitution $e \in [0, 1]$.
    restitution: f64,
    /// Combined Coulomb friction coefficient $\mu \ge 0$.
    friction: f64,
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

        // Detect active contacts between non-jointed colliders
        let contacts = detect_contacts(bodies, colliders, joints, &handle_to_idx);
        let collision_events =
            self.update_collision_events(&contacts, step_index, timestamp_s);

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
                let i_w = b.world_inertia();
                let inv_i_w = b.world_inv_inertia();
                inv_inertia_world[i] = inv_i_w;

                let dv = (g_vec + b.external_force * b.inv_mass) * dt;
                let gyro = w.cross(&(i_w * w));
                let dw = inv_i_w * (b.external_torque - gyro) * dt;

                let vp = v + dv;
                let wp = w + dw;
                v_star[6 * i] = vp.x;
                v_star[6 * i + 1] = vp.y;
                v_star[6 * i + 2] = vp.z;
                v_star[6 * i + 3] = wp.x;
                v_star[6 * i + 4] = wp.y;
                v_star[6 * i + 5] = wp.z;
            }
        }

        // 2. Assemble velocity constraint rows (joints + limits + motors + contacts)
        let rows = assemble_velocity_constraints(
            bodies,
            &handles,
            &handle_to_idx,
            joints,
            &contacts,
            &v_star,
            dt,
            true,
            self.config.cfm_regularization,
        );

        // 3. Solve coupled KKT / Box-LCP system for velocity impulses
        let mut v_new = v_star.clone();
        let m = rows.len();
        let mut step_motor_force_on_a = vec![Vector3::<f64>::zeros(); joints.len()];
        let mut step_motor_torque_on_a = vec![Vector3::<f64>::zeros(); joints.len()];
        for j in joints.iter_mut() {
            j.last_force_on_a = [0.0; 3];
            j.last_torque_on_a = [0.0; 3];
            j.motor_saturated = false;
        }

        if m > 0 {
            let impulses = solve_kkt_system(
                &rows,
                &v_star,
                &inv_mass_vec,
                &inv_inertia_world,
                nb,
                self.config.max_lcp_iters,
            );

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

                // Track motor saturation and accumulate reaction forces/torques
                if let Some(j_idx) = row.joint_idx {
                    let lambda = imp / dt;
                    let f_a = row.j_lin_a * lambda;
                    let t_a_com = row.j_ang_a * lambda;
                    if row.is_motor_row {
                        let bound = row.upper_impulse.abs();
                        if bound <= 1e-15 || imp.abs() >= bound * (1.0 - 1e-5) {
                            joints[j_idx].motor_saturated = true;
                        }
                        step_motor_force_on_a[j_idx] += f_a;
                        step_motor_torque_on_a[j_idx] += t_a_com;
                    }
                    joints[j_idx].last_force_on_a[0] += f_a.x;
                    joints[j_idx].last_force_on_a[1] += f_a.y;
                    joints[j_idx].last_force_on_a[2] += f_a.z;
                    joints[j_idx].last_torque_on_a[0] += t_a_com.x;
                    joints[j_idx].last_torque_on_a[1] += t_a_com.y;
                    joints[j_idx].last_torque_on_a[2] += t_a_com.z;
                }
            }
        }

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

            let x_com_old = b.world_com();
            let x_com_new = x_com_old + v_avg * dt;
            let dq = UnitQuaternion::from_scaled_axis(w_avg * dt);
            let q_new = UnitQuaternion::from_quaternion((dq * b.orientation).quaternion().normalize());

            b.orientation = q_new;
            b.position = x_com_new - q_new * b.local_com;
            b.linear_velocity = v_next;
            b.angular_velocity = w_next;
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
            &step_motor_force_on_a,
            &step_motor_torque_on_a,
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
        contacts: &[DetectedContact],
        _step_index: usize,
        timestamp_s: f64,
    ) -> Vec<CollisionRecord> {
        let mut current_pairs = HashSet::new();
        let mut pair_normals = std::collections::HashMap::new();
        for c in contacts {
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
            pair_normals.insert(pair, n);
        }

        let mut events = Vec::new();
        for pair in &current_pairs {
            if !self.active_contact_pairs.contains(pair) {
                events.push(CollisionRecord {
                    time_s: timestamp_s,
                    body_a: pair.0.clone(),
                    body_b: pair.1.clone(),
                    event_type: CollisionEventType::Started,
                    contact_normal: pair_normals.get(pair).copied(),
                    max_impulse: None,
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

fn detect_contacts(
    bodies: &BodyRegistry,
    colliders: &ColliderRegistry,
    joints: &[SolvedJointRecord],
    handle_to_idx: &std::collections::HashMap<BodyHandle, usize>,
) -> Vec<DetectedContact> {
    let mut disabled_pairs = HashSet::new();
    for j in joints {
        if !j.data.contacts_enabled {
            let pair = if j.body_a <= j.body_b {
                (j.body_a, j.body_b)
            } else {
                (j.body_b, j.body_a)
            };
            disabled_pairs.insert(pair);
        }
    }

    let col_list: Vec<_> = colliders.iter().map(|(_, c)| c).collect();
    let mut contacts = Vec::new();

    for i in 0..col_list.len() {
        for k in (i + 1)..col_list.len() {
            let ca = col_list[i];
            let cb = col_list[k];
            let (Some(ha), Some(hb)) = (ca.parent, cb.parent) else {
                continue;
            };
            if ha == hb {
                continue;
            }
            let pair_key = if ha <= hb { (ha, hb) } else { (hb, ha) };
            if disabled_pairs.contains(&pair_key) {
                continue;
            }
            let (Some(ba), Some(bb)) = (bodies.get(ha), bodies.get(hb)) else {
                continue;
            };
            if !ba.is_dynamic && !bb.is_dynamic {
                continue;
            }
            let (Some(&ia), Some(&ib)) = (handle_to_idx.get(&ha), handle_to_idx.get(&hb)) else {
                continue;
            };

            let aabb_a = ca.world_aabb;
            let aabb_b = cb.world_aabb;
            if aabb_a.max[0] < aabb_b.min[0]
                || aabb_a.min[0] > aabb_b.max[0]
                || aabb_a.max[1] < aabb_b.min[1]
                || aabb_a.min[1] > aabb_b.max[1]
                || aabb_a.max[2] < aabb_b.min[2]
                || aabb_a.min[2] > aabb_b.max[2]
            {
                continue;
            }

            // Compute minimum penetration axis between world AABBs
            let mut min_depth = f64::INFINITY;
            let mut normal = Vector3::y();
            let mut overlap_min = [0.0_f64; 3];
            let mut overlap_max = [0.0_f64; 3];

            let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
            let mut valid = true;
            for axis_idx in 0..3 {
                let lo = aabb_a.min[axis_idx].max(aabb_b.min[axis_idx]);
                let hi = aabb_a.max[axis_idx].min(aabb_b.max[axis_idx]);
                if hi < lo {
                    valid = false;
                    break;
                }
                overlap_min[axis_idx] = lo;
                overlap_max[axis_idx] = hi;

                // Depth if B is pushed along +axis vs -axis
                let d_pos = aabb_a.max[axis_idx] - aabb_b.min[axis_idx];
                let d_neg = aabb_b.max[axis_idx] - aabb_a.min[axis_idx];
                if d_pos <= d_neg {
                    if d_pos < min_depth {
                        min_depth = d_pos;
                        normal = axes[axis_idx];
                    }
                } else if d_neg < min_depth {
                    min_depth = d_neg;
                    normal = -axes[axis_idx];
                }
            }

            if !valid || !min_depth.is_finite() {
                continue;
            }

            let contact_pt = Vector3::new(
                0.5 * (overlap_min[0] + overlap_max[0]),
                0.5 * (overlap_min[1] + overlap_max[1]),
                0.5 * (overlap_min[2] + overlap_max[2]),
            );
            let restitution = 0.5 * (ca.restitution + cb.restitution);
            let friction = (ca.friction * cb.friction).sqrt();

            contacts.push(DetectedContact {
                body_a: ia,
                body_b: ib,
                part_a: ba.part_id.clone(),
                part_b: bb.part_id.clone(),
                point: contact_pt,
                normal,
                depth: min_depth,
                restitution,
                friction,
            });
        }
    }

    contacts
}

fn resolve_contact_penetrations(
    bodies: &mut BodyRegistry,
    handles: &[BodyHandle],
    contacts: &[DetectedContact],
) {
    for c in contacts {
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
    contacts: &[DetectedContact],
    v_ref: &DVector<f64>,
    dt: f64,
    include_centripetal_bias: bool,
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
                });
            }
        }
    }

    // Unilateral contact normal + friction constraint rows
    for c in contacts {
        let ha = handles[c.body_a];
        let hb = handles[c.body_b];
        let (Some(ba), Some(bb)) = (bodies.get(ha), bodies.get(hb)) else {
            continue;
        };
        let r_a = c.point - ba.world_com();
        let r_b = c.point - bb.world_com();

        let va = ba.linear_velocity + ba.angular_velocity.cross(&r_a);
        let vb = bb.linear_velocity + bb.angular_velocity.cross(&r_b);
        let vn_pre = (vb - va).dot(&c.normal);

        let bounce = if include_centripetal_bias && vn_pre < -0.5 {
            -c.restitution * vn_pre
        } else {
            0.0
        };
        let bias = if include_centripetal_bias {
            (0.2 / dt) * (c.depth - 1e-4).max(0.0)
        } else {
            0.0
        };

        rows.push(AssembledConstraintRow {
            joint_idx: None,
            body_a: c.body_a,
            body_b: c.body_b,
            j_lin_a: -c.normal,
            j_ang_a: -r_a.cross(&c.normal),
            j_lin_b: c.normal,
            j_ang_b: r_b.cross(&c.normal),
            rhs_velocity_bias: bounce.max(bias),
            lower_impulse: 0.0,
            upper_impulse: f64::INFINITY,
            cfm: base_cfm.max(1e-10),
            is_motor_row: false,
        });

        // Tangential Coulomb friction row opposing relative slip
        if c.friction > 0.0 {
            let v_rel = vb - va;
            let v_tan = v_rel - c.normal * vn_pre;
            let slip = v_tan.norm();
            if slip > 1e-6 {
                let t_dir = v_tan / slip;
                let m_eff = (ba.inv_mass() + bb.inv_mass()).max(1e-6).recip();
                let max_f_imp = c.friction * m_eff * 9.81 * dt;
                rows.push(AssembledConstraintRow {
                    joint_idx: None,
                    body_a: c.body_a,
                    body_b: c.body_b,
                    j_lin_a: -t_dir,
                    j_ang_a: -r_a.cross(&t_dir),
                    j_lin_b: t_dir,
                    j_ang_b: r_b.cross(&t_dir),
                    rhs_velocity_bias: 0.0,
                    lower_impulse: -max_f_imp,
                    upper_impulse: max_f_imp,
                    cfm: base_cfm.max(1e-9),
                    is_motor_row: false,
                });
            }
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

    solve_box_constrained_kkt(&a_mat, &rhs, &lower, &upper, max_lcp_iters)
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

    // Active-set Box-LCP iteration: 0 = free, -1 = clamped at lower, +1 = clamped at upper
    let mut state = vec![0i8; m];
    let mut x = DVector::<f64>::zeros(m);

    for _iter in 0..max_iters.min(24) {
        let free_indices: Vec<usize> = (0..m).filter(|&i| state[i] == 0).collect();

        for i in 0..m {
            if state[i] == -1 {
                x[i] = lower[i];
            } else if state[i] == 1 {
                x[i] = upper[i];
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
        for i in 0..m {
            if state[i] == 0 {
                if x[i] < lower[i] - 1e-14 {
                    x[i] = lower[i];
                    state[i] = -1;
                    changed = true;
                } else if x[i] > upper[i] + 1e-14 {
                    x[i] = upper[i];
                    state[i] = 1;
                    changed = true;
                }
            } else {
                let residual = a_mat.row(i).dot(&x.transpose()) - rhs[i];
                if (state[i] == -1 && residual < -1e-12) || (state[i] == 1 && residual > 1e-12) {
                    state[i] = 0;
                    changed = true;
                }
            }
        }

        if !changed {
            return x;
        }
    }

    // Fallback: Projected Gauss–Seidel polish if active-set cycled
    for _ in 0..max_iters {
        for i in 0..m {
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
    contacts: &[DetectedContact],
    step_motor_force_on_a: &[Vector3<f64>],
    step_motor_torque_on_a: &[Vector3<f64>],
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

    // 1. Project velocities onto tangent manifold (skipping motor rows that saturated during the step)
    let mut proj_rows = assemble_velocity_constraints(
        bodies,
        handles,
        handle_to_idx,
        joints,
        contacts,
        &v_cur,
        dt,
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
            let jma_ang = if !is_pure_trans || multi_a {
                w_ia * row.j_ang_a
            } else {
                Vector3::zeros()
            };
            let jmb_lin = row.j_lin_b * w_mb;
            let jmb_ang = if !is_pure_trans || multi_b {
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

    let react_rows = assemble_velocity_constraints(
        bodies,
        handles,
        handle_to_idx,
        joints,
        contacts,
        &v_cur,
        dt,
        true,
        cfm,
    );
    if !react_rows.is_empty() {
        let gamma_react = solve_kkt_system(
            &react_rows,
            &v_react_star,
            &inv_mass_vec,
            &inv_inertia_world,
            nb,
            max_lcp_iters,
        );
        for j in joints.iter_mut() {
            j.last_force_on_a = [0.0; 3];
            j.last_torque_on_a = [0.0; 3];
        }
        for (r, row) in react_rows.iter().enumerate() {
            if let Some(j_idx) = row.joint_idx {
                if row.is_motor_row {
                    let rf_f = row.j_lin_a * (gamma_react[r] / dt);
                    let rf_t = row.j_ang_a * (gamma_react[r] / dt);
                    let sf_f = step_motor_force_on_a
                        .get(j_idx)
                        .copied()
                        .unwrap_or_else(Vector3::zeros);
                    let sf_t = step_motor_torque_on_a
                        .get(j_idx)
                        .copied()
                        .unwrap_or_else(Vector3::zeros);
                    let f_a = if sf_f.norm() >= rf_f.norm() { sf_f } else { rf_f };
                    let t_a_com = if sf_t.norm() >= rf_t.norm() { sf_t } else { rf_t };
                    joints[j_idx].last_force_on_a[0] += f_a.x;
                    joints[j_idx].last_force_on_a[1] += f_a.y;
                    joints[j_idx].last_force_on_a[2] += f_a.z;
                    joints[j_idx].last_torque_on_a[0] += t_a_com.x;
                    joints[j_idx].last_torque_on_a[1] += t_a_com.y;
                    joints[j_idx].last_torque_on_a[2] += t_a_com.z;
                } else {
                    let lambda = gamma_react[r] / dt;
                    let f_a = row.j_lin_a * lambda;
                    let t_a_com = row.j_ang_a * lambda;
                    joints[j_idx].last_force_on_a[0] += f_a.x;
                    joints[j_idx].last_force_on_a[1] += f_a.y;
                    joints[j_idx].last_force_on_a[2] += f_a.z;
                    joints[j_idx].last_torque_on_a[0] += t_a_com.x;
                    joints[j_idx].last_torque_on_a[1] += t_a_com.y;
                    joints[j_idx].last_torque_on_a[2] += t_a_com.z;
                }
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

/// Converts `last_torque_on_a` from total moment about Body A's COM ($\boldsymbol{\tau}_{A,\text{com}}$)
/// into the pure joint reaction couple about `anchor_a`:
/// $\boldsymbol{\tau}_{A,\text{anchor}} = \boldsymbol{\tau}_{A,\text{com}} - (\mathbf{p}_{\text{anchor},a} - \mathbf{x}_{\text{com},a}) \times \mathbf{F}_A$.
fn convert_com_torque_to_anchor_couple(
    bodies: &BodyRegistry,
    joints: &mut [SolvedJointRecord],
) {
    for j in joints.iter_mut() {
        let Some(ba) = bodies.get(j.body_a) else {
            continue;
        };
        let r_a = ba.orientation * (Vector3::from(j.data.anchor_a) - ba.local_com);
        let f_a = Vector3::from(j.last_force_on_a);
        let t_a_com = Vector3::from(j.last_torque_on_a);
        let t_anchor = t_a_com - r_a.cross(&f_a);
        j.last_torque_on_a = [t_anchor.x, t_anchor.y, t_anchor.z];
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
    }
}
