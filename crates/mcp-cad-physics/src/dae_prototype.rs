//! Prototype and benchmark suite for the `f64` multi-body DAE/LCP physics solver (`P4-T1` / `ADR-004`).
//!
//! Compares two `f64` solver formulations built on `nalgebra` against closed-form analytical
//! mechanics solutions and the legacy `f32` `TGS-Soft` impulse baseline across four
//! canonical engineering benchmarks:
//! 1. **Free fall under uniform gravity** ($y(t) = y_0 + v_0 t + \frac{1}{2} g t^2$, $v_y(t) = v_0 + g t$).
//! 2. **Rotating cantilevered rod** (centripetal reaction force $F_r = -m \omega^2 r_c$ in the body frame,
//!    angular momentum conservation, and zero pivot drift).
//! 3. **Planar slider-crank 4-bar closed kinematic loop** ($x_C(\theta) = R\cos\theta + \sqrt{L^2 - R^2\sin^2\theta}$
//!    with 3 redundant out-of-plane spatial constraints, $\text{DOF}_{\text{grubler}} = -2, \text{DOF}_{\text{numeric}} = 1$).
//! 4. **Spur gear pair over 10 full pinion revolutions** ($\theta_b + i\,\theta_a = 0$, $\omega_b + i\,\omega_a = 0$,
//!    and coupled dynamic torque transmission $\tau_a = -i\,\tau_{\text{load},b}$).

use std::collections::HashMap;
use std::time::Instant;

use nalgebra::{DMatrix, DVector, Matrix3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

use crate::joint::{
    CadJointType, MotorConfig, is_zero_range_limits, local_orthonormal_basis, perpendicular_axes,
};

/// Solver formulation candidates evaluated in `P4-T1` (`ADR-004`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DaeSolverCandidate {
    /// **Candidate A (Selected in ADR-004)**: Velocity/acceleration-level regularized KKT
    /// saddle-point solver $(J M^{-1} J^T + \epsilon I)\,\boldsymbol{\lambda} = \mathbf{b}$ with
    /// active-set / Box-LCP bounds for unilateral joint limits and motors, exact centripetal
    /// curvature bias $-\dot{J}\mathbf{v}$, trapezoidal midpoint position update, and
    /// Gauss–Newton SVD coordinate projection $\Phi(\mathbf{q}) = \mathbf{0}$.
    DirectKktIndex2Projection,
    /// **Candidate B**: Semi-implicit (symplectic) Euler integrator coupled with an iterative
    /// Projected Gauss–Seidel (PGS) impulse solver and Baumgarte drift stabilization
    /// $\dot{\Phi} + \frac{\beta}{\Delta t}\Phi = \mathbf{0}$ in `f64`.
    SemiImplicitEulerPgsBaumgarte,
}

/// Rigid body state in double precision (`f64`) used by the DAE solver prototype.
#[derive(Debug, Clone)]
pub struct DaeBody {
    /// Unique part identifier.
    pub id: String,
    /// True if movable, false if fixed in world space.
    pub is_dynamic: bool,
    /// Mass in kg (`0.0` for fixed bodies).
    pub mass: f64,
    /// Inverse mass in $\text{kg}^{-1}$ (`0.0` for fixed bodies).
    pub inv_mass: f64,
    /// Center of mass offset in body-local coordinates (meters).
    pub local_com: Vector3<f64>,
    /// $3\times 3$ inertia tensor about the center of mass in body-local axes ($\text{kg}\cdot\text{m}^2$).
    pub local_inertia: Matrix3<f64>,
    /// Inverse of `local_inertia` in body-local axes ($\text{kg}^{-1}\cdot\text{m}^{-2}$).
    pub local_inv_inertia: Matrix3<f64>,
    /// World-space position of the body origin (meters).
    pub position: Vector3<f64>,
    /// World-space orientation of the body frame.
    pub orientation: UnitQuaternion<f64>,
    /// World-space linear velocity of the center of mass (m/s).
    pub linear_velocity: Vector3<f64>,
    /// World-space angular velocity vector (rad/s).
    pub angular_velocity: Vector3<f64>,
    /// Continuous external force applied at the center of mass (N).
    pub external_force: Vector3<f64>,
    /// Continuous external torque applied about the center of mass (N·m).
    pub external_torque: Vector3<f64>,
}

impl DaeBody {
    /// Creates a fixed (ground/static) body at the given world pose.
    #[must_use]
    pub fn new_fixed(id: impl Into<String>, position: Vector3<f64>, orientation: UnitQuaternion<f64>) -> Self {
        Self {
            id: id.into(),
            is_dynamic: false,
            mass: 0.0,
            inv_mass: 0.0,
            local_com: Vector3::zeros(),
            local_inertia: Matrix3::zeros(),
            local_inv_inertia: Matrix3::zeros(),
            position,
            orientation,
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            external_force: Vector3::zeros(),
            external_torque: Vector3::zeros(),
        }
    }

    /// Creates a dynamic rigid body with explicit mass, local COM, and local inertia tensor about COM.
    #[must_use]
    pub fn new_dynamic(
        id: impl Into<String>,
        mass: f64,
        local_com: Vector3<f64>,
        local_inertia: Matrix3<f64>,
        position: Vector3<f64>,
        orientation: UnitQuaternion<f64>,
    ) -> Self {
        let inv_mass = if mass > 1e-15 { 1.0 / mass } else { 0.0 };
        let local_inv_inertia = local_inertia.try_inverse().unwrap_or_else(Matrix3::zeros);
        Self {
            id: id.into(),
            is_dynamic: inv_mass > 0.0,
            mass,
            inv_mass,
            local_com,
            local_inertia,
            local_inv_inertia,
            position,
            orientation,
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            external_force: Vector3::zeros(),
            external_torque: Vector3::zeros(),
        }
    }

    /// Helper to create a uniform rectangular box body of dimensions `(w, h, d)` and `local_com`.
    #[must_use]
    pub fn uniform_box(
        id: impl Into<String>,
        mass: f64,
        dims: [f64; 3],
        local_com: Vector3<f64>,
        position: Vector3<f64>,
    ) -> Self {
        let [w, h, d] = dims;
        let ix = (mass / 12.0) * (h * h + d * d);
        let iy = (mass / 12.0) * (w * w + d * d);
        let iz = (mass / 12.0) * (w * w + h * h);
        let inertia = Matrix3::from_diagonal(&Vector3::new(ix, iy, iz));
        Self::new_dynamic(id, mass, local_com, inertia, position, UnitQuaternion::identity())
    }

    /// Helper to create a uniform cylinder body of `radius` and `height` aligned with the local Z axis.
    #[must_use]
    pub fn uniform_cylinder_z(
        id: impl Into<String>,
        mass: f64,
        radius: f64,
        height: f64,
        position: Vector3<f64>,
    ) -> Self {
        let i_xy = (mass / 12.0) * (3.0 * radius * radius + height * height);
        let i_z = 0.5 * mass * radius * radius;
        let inertia = Matrix3::from_diagonal(&Vector3::new(i_xy, i_xy, i_z));
        Self::new_dynamic(
            id,
            mass,
            Vector3::zeros(),
            inertia,
            position,
            UnitQuaternion::identity(),
        )
    }

    /// Returns the $3\times 3$ world rotation matrix $R$.
    #[must_use]
    pub fn rotation_matrix(&self) -> Matrix3<f64> {
        *self.orientation.to_rotation_matrix().matrix()
    }

    /// Returns the world-space center of mass $\mathbf{x}_{\text{com}} = \mathbf{p} + R\,\mathbf{c}_{\text{loc}}$.
    #[must_use]
    pub fn world_com(&self) -> Vector3<f64> {
        self.position + self.rotation_matrix() * self.local_com
    }

    /// Sets the world-space position of the body origin given a desired world-space COM position.
    pub fn set_world_com(&mut self, world_com: Vector3<f64>) {
        self.position = world_com - self.rotation_matrix() * self.local_com;
    }

    /// Returns the $3\times 3$ world-space inertia tensor $I_w = R I_{\text{loc}} R^T$.
    #[must_use]
    pub fn world_inertia(&self) -> Matrix3<f64> {
        if !self.is_dynamic {
            return Matrix3::zeros();
        }
        let r = self.rotation_matrix();
        r * self.local_inertia * r.transpose()
    }

    /// Returns the $3\times 3$ world-space inverse inertia tensor $I_w^{-1} = R I_{\text{loc}}^{-1} R^T$.
    #[must_use]
    pub fn world_inv_inertia(&self) -> Matrix3<f64> {
        if !self.is_dynamic {
            return Matrix3::zeros();
        }
        let r = self.rotation_matrix();
        r * self.local_inv_inertia * r.transpose()
    }
}

/// Joint constraint record with solved Lagrange multipliers and reaction wrench in `f64`.
#[derive(Debug, Clone)]
pub struct DaeJoint {
    /// Joint identifier.
    pub id: String,
    /// First connected body ID.
    pub part_a: String,
    /// Second connected body ID.
    pub part_b: String,
    /// Kinematic joint specification.
    pub joint_type: CadJointType,
    /// Optional motor actuation.
    pub motor: Option<MotorConfig>,
    /// Reference relative rotation $R_{a,0}^T R_{b,0}$ captured at joint creation.
    pub initial_rel_rot: Matrix3<f64>,
    /// Initial scalar coordinate for 1-DOF joints.
    pub initial_coord: Option<f64>,
    /// Accumulated angle (rad) of `part_a` for `Gear` constraints.
    pub gear_theta_a: f64,
    /// Accumulated angle (rad) of `part_b` for `Gear` constraints.
    pub gear_theta_b: f64,
    /// Accumulated target motor displacement/angle $\int \omega_{\text{target}}\,dt$ from initial pose.
    pub motor_accum_coord: f64,
    /// World-space reaction force acting ON `part_a` at `anchor_a` (Newtons) from the last step.
    pub last_force_on_a: Vector3<f64>,
    /// World-space reaction torque acting ON `part_a` about `anchor_a` (N·m) from the last step.
    pub last_torque_on_a: Vector3<f64>,
}

/// Prototype multi-body DAE/LCP simulation world in `f64`.
#[derive(Debug, Clone)]
pub struct DaeWorld {
    /// Solver formulation candidate.
    pub candidate: DaeSolverCandidate,
    /// Gravity acceleration vector (m/s²).
    pub gravity: Vector3<f64>,
    /// Registered bodies in deterministic insertion order.
    pub body_order: Vec<String>,
    /// Map from part ID to `DaeBody`.
    pub bodies: HashMap<String, DaeBody>,
    /// Registered joints in deterministic insertion order.
    pub joints: Vec<DaeJoint>,
    /// Tikhonov / Constraint Force Mixing (CFM) regularization parameter $\epsilon$ for redundant constraints.
    pub cfm_regularization: f64,
    /// Baumgarte stabilization factor $\beta \in [0, 1]$ (used by `SemiImplicitEulerPgsBaumgarte`).
    pub baumgarte_beta: f64,
    /// Number of PGS iterations (used by `SemiImplicitEulerPgsBaumgarte`).
    pub pgs_iterations: usize,
    /// Current simulation time (seconds).
    pub current_time_s: f64,
}

impl DaeWorld {
    /// Creates a new `DaeWorld` with `"ground"` and `"world"` static anchors.
    #[must_use]
    pub fn new(candidate: DaeSolverCandidate, gravity: Vector3<f64>) -> Self {
        let mut bodies = HashMap::new();
        bodies.insert(
            "ground".to_string(),
            DaeBody::new_fixed("ground", Vector3::zeros(), UnitQuaternion::identity()),
        );
        bodies.insert(
            "world".to_string(),
            DaeBody::new_fixed("world", Vector3::zeros(), UnitQuaternion::identity()),
        );
        Self {
            candidate,
            gravity,
            body_order: Vec::new(),
            bodies,
            joints: Vec::new(),
            cfm_regularization: 1e-12,
            baumgarte_beta: 0.25,
            pgs_iterations: 60,
            current_time_s: 0.0,
        }
    }

    /// Registers a rigid body in the DAE world.
    pub fn add_body(&mut self, body: DaeBody) {
        let id = body.id.clone();
        if !self.bodies.contains_key(&id) {
            self.body_order.push(id.clone());
        }
        self.bodies.insert(id, body);
    }

    /// Adds a kinematic/dynamic joint between `part_a` and `part_b`.
    pub fn add_joint(
        &mut self,
        id: impl Into<String>,
        part_a: impl Into<String>,
        part_b: impl Into<String>,
        joint_type: CadJointType,
        motor: Option<MotorConfig>,
    ) {
        let pa_id = part_a.into();
        let pb_id = part_b.into();
        let (pa, ra) = self
            .bodies
            .get(&pa_id)
            .map(|b| (b.position, b.rotation_matrix()))
            .unwrap_or_else(|| (Vector3::zeros(), Matrix3::identity()));
        let (pb, rb) = self
            .bodies
            .get(&pb_id)
            .map(|b| (b.position, b.rotation_matrix()))
            .unwrap_or_else(|| (Vector3::zeros(), Matrix3::identity()));

        let initial_rel_rot = ra.transpose() * rb;
        let initial_coord = joint_type.current_coordinate(pa, ra, pb, rb);

        self.joints.push(DaeJoint {
            id: id.into(),
            part_a: pa_id,
            part_b: pb_id,
            joint_type,
            motor,
            initial_rel_rot,
            initial_coord,
            gear_theta_a: 0.0,
            gear_theta_b: 0.0,
            motor_accum_coord: 0.0,
            last_force_on_a: Vector3::zeros(),
            last_torque_on_a: Vector3::zeros(),
        });
    }

    /// Returns the list of dynamic body IDs and their index map `0..N_dyn`.
    fn dynamic_body_indices(&self) -> (Vec<String>, HashMap<String, usize>) {
        let mut dyn_ids = Vec::new();
        let mut map = HashMap::new();
        for id in &self.body_order {
            if let Some(b) = self.bodies.get(id) {
                if b.is_dynamic {
                    let idx = dyn_ids.len();
                    dyn_ids.push(id.clone());
                    map.insert(id.clone(), idx);
                }
            }
        }
        (dyn_ids, map)
    }

    /// Advances the multi-body system by one time step `dt` (seconds).
    pub fn step(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        match self.candidate {
            DaeSolverCandidate::DirectKktIndex2Projection => self.step_direct_kkt_index2(dt),
            DaeSolverCandidate::SemiImplicitEulerPgsBaumgarte => self.step_pgs_baumgarte(dt),
        }
        self.current_time_s += dt;
    }

    /// Evaluates the maximum infinity-norm holonomic position/rotation constraint residual $\|\Phi(q)\|_\infty$.
    #[must_use]
    pub fn max_constraint_residual(&self) -> f64 {
        let mut max_err = 0.0_f64;
        for j in &self.joints {
            let (pa, ra) = self.body_pose(&j.part_a);
            let (pb, rb) = self.body_pose(&j.part_b);
            match &j.joint_type {
                CadJointType::Fixed { anchor_a, anchor_b } => {
                    let dp = (pa + ra * Vector3::from(*anchor_a))
                        - (pb + rb * Vector3::from(*anchor_b));
                    let drot = rotation_vector(rb * (ra * j.initial_rel_rot).transpose());
                    max_err = max_err.max(dp.amax()).max(drot.amax());
                }
                CadJointType::Spherical { anchor_a, anchor_b } => {
                    let dp = (pa + ra * Vector3::from(*anchor_a))
                        - (pb + rb * Vector3::from(*anchor_b));
                    max_err = max_err.max(dp.amax());
                }
                CadJointType::Revolute {
                    anchor_a,
                    anchor_b,
                    axis,
                    ..
                } => {
                    let dp = (pa + ra * Vector3::from(*anchor_a))
                        - (pb + rb * Vector3::from(*anchor_b));
                    let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::z());
                    let z_a = ra * u_a;
                    let z_b = rb * u_a;
                    let axis_err = z_a.cross(&z_b).norm().atan2(z_a.dot(&z_b));
                    max_err = max_err.max(dp.amax()).max(axis_err.abs());
                }
                CadJointType::Prismatic {
                    anchor_a,
                    anchor_b,
                    axis,
                    ..
                } => {
                    let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::x());
                    let z_a = ra * u_a;
                    let (x_a, y_a) = perpendicular_axes(&z_a);
                    let drot = rotation_vector(rb * (ra * j.initial_rel_rot).transpose());
                    let delta_r = (pb + rb * Vector3::from(*anchor_b))
                        - (pa + ra * Vector3::from(*anchor_a));
                    let p_err = x_a.dot(&delta_r).abs().max(y_a.dot(&delta_r).abs());
                    max_err = max_err.max(drot.amax()).max(p_err);
                }
                CadJointType::Gear { ratio, .. } => {
                    let g_err = (ratio * j.gear_theta_a + j.gear_theta_b).abs();
                    max_err = max_err.max(g_err);
                }
            }
        }
        max_err
    }

    fn body_pose(&self, id: &str) -> (Vector3<f64>, Matrix3<f64>) {
        self.bodies
            .get(id)
            .map(|b| (b.position, b.rotation_matrix()))
            .unwrap_or_else(|| (Vector3::zeros(), Matrix3::identity()))
    }

    /// Assembles all active velocity/acceleration constraint rows across all joints.
    fn assemble_constraint_rows(
        &self,
        dyn_map: &HashMap<String, usize>,
        dt: f64,
        include_baumgarte: bool,
        exact_lie_transport: bool,
    ) -> Vec<AssembledConstraintRow> {
        let mut rows = Vec::new();
        let beta_over_dt = if include_baumgarte && dt > 0.0 {
            self.baumgarte_beta / dt
        } else {
            0.0
        };

        for (j_idx, j) in self.joints.iter().enumerate() {
            let col_a = dyn_map.get(&j.part_a).copied();
            let col_b = dyn_map.get(&j.part_b).copied();
            if col_a.is_none() && col_b.is_none() {
                continue;
            }

            let ba = &self.bodies[&j.part_a];
            let bb = &self.bodies[&j.part_b];
            let pa = ba.position;
            let ra = ba.rotation_matrix();
            let com_a = ba.world_com();
            let wa = ba.angular_velocity;

            let pb = bb.position;
            let rb = bb.rotation_matrix();
            let com_b = bb.world_com();
            let wb = bb.angular_velocity;

            let centripetal_vel_bias = |r_a: Vector3<f64>, r_b: Vector3<f64>| -> Vector3<f64> {
                if dt <= 0.0 {
                    Vector3::zeros()
                } else if exact_lie_transport {
                    let ra_next = UnitQuaternion::from_scaled_axis(wa * dt) * r_a;
                    let rb_next = UnitQuaternion::from_scaled_axis(wb * dt) * r_b;
                    wa.cross(&(ra_next - r_a)) - wb.cross(&(rb_next - r_b))
                } else {
                    (wa.cross(&wa.cross(&r_a)) - wb.cross(&wb.cross(&r_b))) * dt
                }
            };

            match &j.joint_type {
                CadJointType::Fixed { anchor_a, anchor_b } => {
                    let p_anchor_a = pa + ra * Vector3::from(*anchor_a);
                    let p_anchor_b = pb + rb * Vector3::from(*anchor_b);
                    let r_a = p_anchor_a - com_a;
                    let r_b = p_anchor_b - com_b;
                    let vel_bias = centripetal_vel_bias(r_a, r_b);
                    let pos_err = p_anchor_a - p_anchor_b;
                    let e_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
                    for k in 0..3 {
                        let ek = e_axes[k];
                        rows.push(AssembledConstraintRow {
                            joint_idx: j_idx,
                            col_a,
                            col_b,
                            lin_a: ek,
                            ang_a: r_a.cross(&ek),
                            lin_b: -ek,
                            ang_b: -r_b.cross(&ek),
                            force_dir_on_a: ek,
                            torque_dir_on_a: Vector3::zeros(),
                            target_vel_rhs: -ek.dot(&vel_bias) - beta_over_dt * pos_err[k],
                            min_force: f64::NEG_INFINITY,
                            max_force: f64::INFINITY,
                        });
                    }
                    let drot = rotation_vector(rb * (ra * j.initial_rel_rot).transpose());
                    for k in 0..3 {
                        let ek = e_axes[k];
                        rows.push(AssembledConstraintRow {
                            joint_idx: j_idx,
                            col_a,
                            col_b,
                            lin_a: Vector3::zeros(),
                            ang_a: -ek,
                            lin_b: Vector3::zeros(),
                            ang_b: ek,
                            force_dir_on_a: Vector3::zeros(),
                            torque_dir_on_a: -ek,
                            target_vel_rhs: -beta_over_dt * drot[k],
                            min_force: f64::NEG_INFINITY,
                            max_force: f64::INFINITY,
                        });
                    }
                }
                CadJointType::Spherical { anchor_a, anchor_b } => {
                    let p_anchor_a = pa + ra * Vector3::from(*anchor_a);
                    let p_anchor_b = pb + rb * Vector3::from(*anchor_b);
                    let r_a = p_anchor_a - com_a;
                    let r_b = p_anchor_b - com_b;
                    let vel_bias = centripetal_vel_bias(r_a, r_b);
                    let pos_err = p_anchor_a - p_anchor_b;
                    let e_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
                    for k in 0..3 {
                        let ek = e_axes[k];
                        rows.push(AssembledConstraintRow {
                            joint_idx: j_idx,
                            col_a,
                            col_b,
                            lin_a: ek,
                            ang_a: r_a.cross(&ek),
                            lin_b: -ek,
                            ang_b: -r_b.cross(&ek),
                            force_dir_on_a: ek,
                            torque_dir_on_a: Vector3::zeros(),
                            target_vel_rhs: -ek.dot(&vel_bias) - beta_over_dt * pos_err[k],
                            min_force: f64::NEG_INFINITY,
                            max_force: f64::INFINITY,
                        });
                    }
                }
                CadJointType::Revolute {
                    anchor_a,
                    anchor_b,
                    axis,
                    limits,
                } => {
                    let p_anchor_a = pa + ra * Vector3::from(*anchor_a);
                    let p_anchor_b = pb + rb * Vector3::from(*anchor_b);
                    let r_a = p_anchor_a - com_a;
                    let r_b = p_anchor_b - com_b;
                    let vel_bias = centripetal_vel_bias(r_a, r_b);
                    let pos_err = p_anchor_a - p_anchor_b;
                    let e_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
                    for k in 0..3 {
                        let ek = e_axes[k];
                        rows.push(AssembledConstraintRow {
                            joint_idx: j_idx,
                            col_a,
                            col_b,
                            lin_a: ek,
                            ang_a: r_a.cross(&ek),
                            lin_b: -ek,
                            ang_b: -r_b.cross(&ek),
                            force_dir_on_a: ek,
                            torque_dir_on_a: Vector3::zeros(),
                            target_vel_rhs: -ek.dot(&vel_bias) - beta_over_dt * pos_err[k],
                            min_force: f64::NEG_INFINITY,
                            max_force: f64::INFINITY,
                        });
                    }

                    let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::z());
                    let z_a = ra * u_a;
                    let (x_a, y_a) = perpendicular_axes(&z_a);
                    let z_b = rb * u_a;
                    let err_x = -y_a.dot(&z_b);
                    let err_y = x_a.dot(&z_b);
                    let ang_bias = (wa.cross(&x_a)).dot(&(wb - wa));
                    let ang_bias_y = (wa.cross(&y_a)).dot(&(wb - wa));

                    rows.push(AssembledConstraintRow {
                        joint_idx: j_idx,
                        col_a,
                        col_b,
                        lin_a: Vector3::zeros(),
                        ang_a: -x_a,
                        lin_b: Vector3::zeros(),
                        ang_b: x_a,
                        force_dir_on_a: Vector3::zeros(),
                        torque_dir_on_a: -x_a,
                        target_vel_rhs: -ang_bias * dt - beta_over_dt * err_x,
                        min_force: f64::NEG_INFINITY,
                        max_force: f64::INFINITY,
                    });
                    rows.push(AssembledConstraintRow {
                        joint_idx: j_idx,
                        col_a,
                        col_b,
                        lin_a: Vector3::zeros(),
                        ang_a: -y_a,
                        lin_b: Vector3::zeros(),
                        ang_b: y_a,
                        force_dir_on_a: Vector3::zeros(),
                        torque_dir_on_a: -y_a,
                        target_vel_rhs: -ang_bias_y * dt - beta_over_dt * err_y,
                        min_force: f64::NEG_INFINITY,
                        max_force: f64::INFINITY,
                    });

                    // Limits along hinge axis z_a
                    if let Some([min_a, max_a]) = limits {
                        let cur_theta = j
                            .joint_type
                            .current_coordinate(pa, ra, pb, rb)
                            .unwrap_or(0.0);
                        if is_zero_range_limits(Some([*min_a, *max_a])) {
                            let err = cur_theta - min_a;
                            rows.push(AssembledConstraintRow {
                                joint_idx: j_idx,
                                col_a,
                                col_b,
                                lin_a: Vector3::zeros(),
                                ang_a: -z_a,
                                lin_b: Vector3::zeros(),
                                ang_b: z_a,
                                force_dir_on_a: Vector3::zeros(),
                                torque_dir_on_a: -z_a,
                                target_vel_rhs: -err / dt,
                                min_force: f64::NEG_INFINITY,
                                max_force: f64::INFINITY,
                            });
                        } else if cur_theta <= *min_a + 1e-6 {
                            let pen = (cur_theta - *min_a).min(0.0);
                            rows.push(AssembledConstraintRow {
                                joint_idx: j_idx,
                                col_a,
                                col_b,
                                lin_a: Vector3::zeros(),
                                ang_a: -z_a,
                                lin_b: Vector3::zeros(),
                                ang_b: z_a,
                                force_dir_on_a: Vector3::zeros(),
                                torque_dir_on_a: -z_a,
                                target_vel_rhs: -pen / dt,
                                min_force: 0.0,
                                max_force: f64::INFINITY,
                            });
                        } else if cur_theta >= *max_a - 1e-6 {
                            let pen = (cur_theta - *max_a).max(0.0);
                            rows.push(AssembledConstraintRow {
                                joint_idx: j_idx,
                                col_a,
                                col_b,
                                lin_a: Vector3::zeros(),
                                ang_a: -z_a,
                                lin_b: Vector3::zeros(),
                                ang_b: z_a,
                                force_dir_on_a: Vector3::zeros(),
                                torque_dir_on_a: -z_a,
                                target_vel_rhs: -pen / dt,
                                min_force: f64::NEG_INFINITY,
                                max_force: 0.0,
                            });
                        }
                    }

                    // Motor row along z_a
                    if let Some(m) = j.motor {
                        if m.max_force > 0.0 {
                            rows.push(AssembledConstraintRow {
                                joint_idx: j_idx,
                                col_a,
                                col_b,
                                lin_a: Vector3::zeros(),
                                ang_a: -z_a,
                                lin_b: Vector3::zeros(),
                                ang_b: z_a,
                                force_dir_on_a: Vector3::zeros(),
                                torque_dir_on_a: -z_a,
                                target_vel_rhs: m.target_velocity,
                                min_force: -m.max_force,
                                max_force: m.max_force,
                            });
                        }
                    }
                }
                CadJointType::Prismatic {
                    anchor_a,
                    anchor_b,
                    axis,
                    limits,
                } => {
                    let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::x());
                    let z_a = ra * u_a;
                    let (x_a, y_a) = perpendicular_axes(&z_a);
                    let drot = rotation_vector(rb * (ra * j.initial_rel_rot).transpose());
                    let e_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
                    for k in 0..3 {
                        let ek = e_axes[k];
                        rows.push(AssembledConstraintRow {
                            joint_idx: j_idx,
                            col_a,
                            col_b,
                            lin_a: Vector3::zeros(),
                            ang_a: -ek,
                            lin_b: Vector3::zeros(),
                            ang_b: ek,
                            force_dir_on_a: Vector3::zeros(),
                            torque_dir_on_a: -ek,
                            target_vel_rhs: -beta_over_dt * drot[k],
                            min_force: f64::NEG_INFINITY,
                            max_force: f64::INFINITY,
                        });
                    }

                    let p_anchor_a = pa + ra * Vector3::from(*anchor_a);
                    let p_anchor_b = pb + rb * Vector3::from(*anchor_b);
                    let r_b_com = p_anchor_b - com_b;
                    let r_ab_from_com_a = p_anchor_b - com_a;
                    let delta_r = p_anchor_b - p_anchor_a;

                    for (perp_axis, err_val) in [(x_a, x_a.dot(&delta_r)), (y_a, y_a.dot(&delta_r))]
                    {
                        rows.push(AssembledConstraintRow {
                            joint_idx: j_idx,
                            col_a,
                            col_b,
                            lin_a: -perp_axis,
                            ang_a: perp_axis.cross(&r_ab_from_com_a),
                            lin_b: perp_axis,
                            ang_b: r_b_com.cross(&perp_axis),
                            force_dir_on_a: -perp_axis,
                            torque_dir_on_a: Vector3::zeros(),
                            target_vel_rhs: -beta_over_dt * err_val,
                            min_force: f64::NEG_INFINITY,
                            max_force: f64::INFINITY,
                        });
                    }

                    if let Some([min_s, max_s]) = limits {
                        let cur_s = delta_r.dot(&z_a);
                        if is_zero_range_limits(Some([*min_s, *max_s])) {
                            let err = cur_s - min_s;
                            rows.push(AssembledConstraintRow {
                                joint_idx: j_idx,
                                col_a,
                                col_b,
                                lin_a: -z_a,
                                ang_a: z_a.cross(&r_ab_from_com_a),
                                lin_b: z_a,
                                ang_b: r_b_com.cross(&z_a),
                                force_dir_on_a: -z_a,
                                torque_dir_on_a: Vector3::zeros(),
                                target_vel_rhs: -err / dt,
                                min_force: f64::NEG_INFINITY,
                                max_force: f64::INFINITY,
                            });
                        } else if cur_s <= *min_s + 1e-6 {
                            let pen = (cur_s - *min_s).min(0.0);
                            rows.push(AssembledConstraintRow {
                                joint_idx: j_idx,
                                col_a,
                                col_b,
                                lin_a: -z_a,
                                ang_a: z_a.cross(&r_ab_from_com_a),
                                lin_b: z_a,
                                ang_b: r_b_com.cross(&z_a),
                                force_dir_on_a: -z_a,
                                torque_dir_on_a: Vector3::zeros(),
                                target_vel_rhs: -pen / dt,
                                min_force: 0.0,
                                max_force: f64::INFINITY,
                            });
                        } else if cur_s >= *max_s - 1e-6 {
                            let pen = (cur_s - *max_s).max(0.0);
                            rows.push(AssembledConstraintRow {
                                joint_idx: j_idx,
                                col_a,
                                col_b,
                                lin_a: -z_a,
                                ang_a: z_a.cross(&r_ab_from_com_a),
                                lin_b: z_a,
                                ang_b: r_b_com.cross(&z_a),
                                force_dir_on_a: -z_a,
                                torque_dir_on_a: Vector3::zeros(),
                                target_vel_rhs: -pen / dt,
                                min_force: f64::NEG_INFINITY,
                                max_force: 0.0,
                            });
                        }
                    }

                    if let Some(m) = j.motor {
                        if m.max_force > 0.0 {
                            rows.push(AssembledConstraintRow {
                                joint_idx: j_idx,
                                col_a,
                                col_b,
                                lin_a: -z_a,
                                ang_a: z_a.cross(&r_ab_from_com_a),
                                lin_b: z_a,
                                ang_b: r_b_com.cross(&z_a),
                                force_dir_on_a: -z_a,
                                torque_dir_on_a: Vector3::zeros(),
                                target_vel_rhs: m.target_velocity,
                                min_force: -m.max_force,
                                max_force: m.max_force,
                            });
                        }
                    }
                }
                CadJointType::Gear {
                    ratio,
                    axis_a,
                    axis_b,
                } => {
                    let (_, _, u_a) = local_orthonormal_basis(*axis_a, Vector3::z());
                    let (_, _, u_b) = local_orthonormal_basis(*axis_b, Vector3::z());
                    let z_a = ra * u_a;
                    let z_b = rb * u_b;
                    let phase_err = ratio * j.gear_theta_a + j.gear_theta_b;
                    rows.push(AssembledConstraintRow {
                        joint_idx: j_idx,
                        col_a,
                        col_b,
                        lin_a: Vector3::zeros(),
                        ang_a: z_a * (*ratio),
                        lin_b: Vector3::zeros(),
                        ang_b: z_b,
                        force_dir_on_a: Vector3::zeros(),
                        torque_dir_on_a: z_a * (*ratio),
                        target_vel_rhs: -beta_over_dt * phase_err,
                        min_force: f64::NEG_INFINITY,
                        max_force: f64::INFINITY,
                    });
                }
            }
        }

        rows
    }

    /// Candidate A: Direct Regularized KKT Velocity/Acceleration DAE + Trapezoidal Midpoint
    /// Integration + Gauss–Newton SVD Manifold Projection $\Phi(q) = 0$.
    fn step_direct_kkt_index2(&mut self, dt: f64) {
        let (dyn_ids, dyn_map) = self.dynamic_body_indices();
        let n_dyn = dyn_ids.len();
        if n_dyn == 0 {
            return;
        }

        // 1. Assemble unconstrained velocity predictor v* = v_k + M^{-1} (f_ext - w x I w) * dt
        let mut v_curr = DVector::<f64>::zeros(6 * n_dyn);
        let mut v_star = DVector::<f64>::zeros(6 * n_dyn);
        let mut m_inv_blocks: Vec<(f64, Matrix3<f64>)> = Vec::with_capacity(n_dyn);

        for (idx, id) in dyn_ids.iter().enumerate() {
            let b = &self.bodies[id];
            let base = 6 * idx;
            let lv = b.linear_velocity;
            let av = b.angular_velocity;
            v_curr.fixed_rows_mut::<3>(base).copy_from(&lv);
            v_curr.fixed_rows_mut::<3>(base + 3).copy_from(&av);

            let inv_m = b.inv_mass;
            let i_w = b.world_inertia();
            let inv_i_w = b.world_inv_inertia();
            m_inv_blocks.push((inv_m, inv_i_w));

            let f_total = b.mass * self.gravity + b.external_force;
            let gyro = av.cross(&(i_w * av));
            let tau_total = b.external_torque - gyro;

            let lv_star = lv + inv_m * f_total * dt;
            let av_star = av + inv_i_w * tau_total * dt;
            v_star.fixed_rows_mut::<3>(base).copy_from(&lv_star);
            v_star.fixed_rows_mut::<3>(base + 3).copy_from(&av_star);
        }

        // 2. Assemble constraint Jacobian J (m x 6N) and velocity RHS b_rhs (m)
        let rows = self.assemble_constraint_rows(&dyn_map, dt, false, false);
        let m_rows = rows.len();

        let mut v_next = v_star.clone();
        for j in &mut self.joints {
            j.last_force_on_a = Vector3::zeros();
            j.last_torque_on_a = Vector3::zeros();
        }

        if m_rows > 0 {
            let mut j_mat = DMatrix::<f64>::zeros(m_rows, 6 * n_dyn);
            let mut rhs = DVector::<f64>::zeros(m_rows);
            let mut lambda_min = vec![f64::NEG_INFINITY; m_rows];
            let mut lambda_max = vec![f64::INFINITY; m_rows];

            for (r, row) in rows.iter().enumerate() {
                if let Some(ca) = row.col_a {
                    let b = 6 * ca;
                    j_mat.fixed_view_mut::<1, 3>(r, b).copy_from(&row.lin_a.transpose());
                    j_mat.fixed_view_mut::<1, 3>(r, b + 3).copy_from(&row.ang_a.transpose());
                }
                if let Some(cb) = row.col_b {
                    let b = 6 * cb;
                    j_mat.fixed_view_mut::<1, 3>(r, b).copy_from(&row.lin_b.transpose());
                    j_mat.fixed_view_mut::<1, 3>(r, b + 3).copy_from(&row.ang_b.transpose());
                }
                rhs[r] = row.target_vel_rhs;
                // Convert force/torque bounds [min_force, max_force] to impulse bounds [min_force*dt, max_force*dt]
                lambda_min[r] = if row.min_force.is_finite() {
                    row.min_force * dt
                } else {
                    f64::NEG_INFINITY
                };
                lambda_max[r] = if row.max_force.is_finite() {
                    row.max_force * dt
                } else {
                    f64::INFINITY
                };
            }

            // Compute M^{-1} J^T (6N x m) and regularized Delassus / Schur complement A = J M^{-1} J^T + eps * I
            let mut minv_jt = DMatrix::<f64>::zeros(6 * n_dyn, m_rows);
            for (r, row) in rows.iter().enumerate() {
                if let Some(ca) = row.col_a {
                    let (im, ii) = &m_inv_blocks[ca];
                    let base = 6 * ca;
                    minv_jt.fixed_view_mut::<3, 1>(base, r).copy_from(&(row.lin_a * *im));
                    minv_jt.fixed_view_mut::<3, 1>(base + 3, r).copy_from(&(ii * row.ang_a));
                }
                if let Some(cb) = row.col_b {
                    let (im, ii) = &m_inv_blocks[cb];
                    let base = 6 * cb;
                    minv_jt.fixed_view_mut::<3, 1>(base, r).copy_from(&(row.lin_b * *im));
                    minv_jt.fixed_view_mut::<3, 1>(base + 3, r).copy_from(&(ii * row.ang_b));
                }
            }

            let mut a_mat = &j_mat * &minv_jt;
            for r in 0..m_rows {
                a_mat[(r, r)] += self.cfm_regularization * a_mat[(r, r)].abs().max(1.0);
            }

            let b_deficit = &rhs - &j_mat * &v_star;
            let impulse = solve_box_constrained_kkt(&a_mat, &b_deficit, &lambda_min, &lambda_max);

            v_next += &minv_jt * &impulse;

            // Extract exact physical forces and torques F = impulse / dt for each joint
            let inv_dt = 1.0 / dt;
            for (r, row) in rows.iter().enumerate() {
                let lambda_force = impulse[r] * inv_dt;
                let j = &mut self.joints[row.joint_idx];
                j.last_force_on_a += row.force_dir_on_a * lambda_force;
                j.last_torque_on_a += row.torque_dir_on_a * lambda_force;
            }
        }

        // Advance target motor coordinates for unsaturated velocity motors
        for j in &mut self.joints {
            if let Some(m) = j.motor {
                if m.max_force > 0.0 {
                    j.motor_accum_coord += m.target_velocity * dt;
                }
            }
        }

        // 3. Trapezoidal / Midpoint position & exponential-map orientation integration
        let mut prev_rotations: HashMap<String, Matrix3<f64>> = HashMap::with_capacity(n_dyn);
        for (idx, id) in dyn_ids.iter().enumerate() {
            let base = 6 * idx;
            let lv_new = v_next.fixed_rows::<3>(base).into_owned();
            let av_new = v_next.fixed_rows::<3>(base + 3).into_owned();
            let lv_old = v_curr.fixed_rows::<3>(base).into_owned();
            let av_old = v_curr.fixed_rows::<3>(base + 3).into_owned();

            let lv_mid = 0.5 * (lv_old + lv_new);
            let av_mid = 0.5 * (av_old + av_new);

            if let Some(b) = self.bodies.get_mut(id) {
                prev_rotations.insert(id.clone(), b.rotation_matrix());
                let old_com = b.world_com();
                let new_com = old_com + lv_mid * dt;
                let dq = UnitQuaternion::from_scaled_axis(av_mid * dt);
                b.orientation = dq * b.orientation;
                let _ = b.orientation.renormalize();
                b.set_world_com(new_com);
                b.linear_velocity = lv_new;
                b.angular_velocity = av_new;
            }
        }

        // Update gear incremental angles before manifold projection
        self.advance_gear_angles(&prev_rotations);

        // 4. Mass-Weighted Cholesky Gauss–Newton Position Manifold Projection (\Phi(q_{k+1}) = 0)
        self.project_holonomic_manifold(&dyn_ids, &dyn_map);

        // 5. Velocity Manifold Projection (J(q_{k+1}) v_{k+1} = b_motor) and exact end-of-step KKT reactions
        self.project_velocity_and_compute_reactions(&dyn_ids, &dyn_map, dt);
    }

    /// Candidate B: Semi-Implicit Euler + Iterative Projected Gauss–Seidel (PGS) with Baumgarte Stabilization.
    fn step_pgs_baumgarte(&mut self, dt: f64) {
        let (dyn_ids, dyn_map) = self.dynamic_body_indices();
        let n_dyn = dyn_ids.len();
        if n_dyn == 0 {
            return;
        }

        let mut v_vec = DVector::<f64>::zeros(6 * n_dyn);
        let mut m_inv_blocks: Vec<(f64, Matrix3<f64>)> = Vec::with_capacity(n_dyn);

        for (idx, id) in dyn_ids.iter().enumerate() {
            let b = &self.bodies[id];
            let base = 6 * idx;
            let inv_m = b.inv_mass;
            let i_w = b.world_inertia();
            let inv_i_w = b.world_inv_inertia();
            m_inv_blocks.push((inv_m, inv_i_w));

            let f_total = b.mass * self.gravity + b.external_force;
            let gyro = b.angular_velocity.cross(&(i_w * b.angular_velocity));
            let tau_total = b.external_torque - gyro;

            let lv_star = b.linear_velocity + inv_m * f_total * dt;
            let av_star = b.angular_velocity + inv_i_w * tau_total * dt;
            v_vec.fixed_rows_mut::<3>(base).copy_from(&lv_star);
            v_vec.fixed_rows_mut::<3>(base + 3).copy_from(&av_star);
        }

        let rows = self.assemble_constraint_rows(&dyn_map, dt, true, false);
        let m_rows = rows.len();
        let mut impulses = vec![0.0_f64; m_rows];

        // Precompute effective diagonal mass 1 / (j_r M^{-1} j_r^T + cfm)
        let mut inv_diag = vec![0.0_f64; m_rows];
        for (r, row) in rows.iter().enumerate() {
            let mut d = 0.0;
            if let Some(ca) = row.col_a {
                let (im, ii) = &m_inv_blocks[ca];
                d += row.lin_a.norm_squared() * *im + row.ang_a.dot(&(ii * row.ang_a));
            }
            if let Some(cb) = row.col_b {
                let (im, ii) = &m_inv_blocks[cb];
                d += row.lin_b.norm_squared() * *im + row.ang_b.dot(&(ii * row.ang_b));
            }
            if d > 1e-15 {
                inv_diag[r] = 1.0 / (d + self.cfm_regularization);
            }
        }

        // Sequential PGS sweeps
        for _iter in 0..self.pgs_iterations {
            for (r, row) in rows.iter().enumerate() {
                if inv_diag[r] == 0.0 {
                    continue;
                }
                let mut jv = 0.0;
                if let Some(ca) = row.col_a {
                    let base = 6 * ca;
                    jv += row.lin_a.dot(&v_vec.fixed_rows::<3>(base))
                        + row.ang_a.dot(&v_vec.fixed_rows::<3>(base + 3));
                }
                if let Some(cb) = row.col_b {
                    let base = 6 * cb;
                    jv += row.lin_b.dot(&v_vec.fixed_rows::<3>(base))
                        + row.ang_b.dot(&v_vec.fixed_rows::<3>(base + 3));
                }

                let d_imp = (row.target_vel_rhs - jv) * inv_diag[r];
                let old_imp = impulses[r];
                let min_imp = if row.min_force.is_finite() {
                    row.min_force * dt
                } else {
                    f64::NEG_INFINITY
                };
                let max_imp = if row.max_force.is_finite() {
                    row.max_force * dt
                } else {
                    f64::INFINITY
                };
                let new_imp = (old_imp + d_imp).clamp(min_imp, max_imp);
                let delta = new_imp - old_imp;
                impulses[r] = new_imp;

                if let Some(ca) = row.col_a {
                    let (im, ii) = &m_inv_blocks[ca];
                    let base = 6 * ca;
                    let dv_l = row.lin_a * (*im * delta);
                    let dv_a = ii * (row.ang_a * delta);
                    let mut lv = v_vec.fixed_rows_mut::<3>(base);
                    lv += dv_l;
                    let mut av = v_vec.fixed_rows_mut::<3>(base + 3);
                    av += dv_a;
                }
                if let Some(cb) = row.col_b {
                    let (im, ii) = &m_inv_blocks[cb];
                    let base = 6 * cb;
                    let dv_l = row.lin_b * (*im * delta);
                    let dv_a = ii * (row.ang_b * delta);
                    let mut lv = v_vec.fixed_rows_mut::<3>(base);
                    lv += dv_l;
                    let mut av = v_vec.fixed_rows_mut::<3>(base + 3);
                    av += dv_a;
                }
            }
        }

        for j in &mut self.joints {
            j.last_force_on_a = Vector3::zeros();
            j.last_torque_on_a = Vector3::zeros();
        }
        let inv_dt = 1.0 / dt;
        for (r, row) in rows.iter().enumerate() {
            let f = impulses[r] * inv_dt;
            let j = &mut self.joints[row.joint_idx];
            j.last_force_on_a += row.force_dir_on_a * f;
            j.last_torque_on_a += row.torque_dir_on_a * f;
        }

        // Symplectic Euler position integration: x_{k+1} = x_k + v_{k+1} * dt
        let mut prev_rotations: HashMap<String, Matrix3<f64>> = HashMap::with_capacity(n_dyn);
        for (idx, id) in dyn_ids.iter().enumerate() {
            let base = 6 * idx;
            let lv_new = v_vec.fixed_rows::<3>(base).into_owned();
            let av_new = v_vec.fixed_rows::<3>(base + 3).into_owned();
            if let Some(b) = self.bodies.get_mut(id) {
                prev_rotations.insert(id.clone(), b.rotation_matrix());
                let new_com = b.world_com() + lv_new * dt;
                let dq = UnitQuaternion::from_scaled_axis(av_new * dt);
                b.orientation = dq * b.orientation;
                let _ = b.orientation.renormalize();
                b.set_world_com(new_com);
                b.linear_velocity = lv_new;
                b.angular_velocity = av_new;
            }
        }

        self.advance_gear_angles(&prev_rotations);
    }

    fn advance_gear_angles(&mut self, prev_rotations: &HashMap<String, Matrix3<f64>>) {
        let mut updates = Vec::new();
        for (idx, j) in self.joints.iter().enumerate() {
            if let CadJointType::Gear { axis_a, axis_b, .. } = &j.joint_type {
                let ra_cur = self.body_pose(&j.part_a).1;
                let rb_cur = self.body_pose(&j.part_b).1;
                let ra_prev = prev_rotations.get(&j.part_a).copied().unwrap_or(ra_cur);
                let rb_prev = prev_rotations.get(&j.part_b).copied().unwrap_or(rb_cur);
                let (_, _, ua) = local_orthonormal_basis(*axis_a, Vector3::z());
                let (_, _, ub) = local_orthonormal_basis(*axis_b, Vector3::z());
                let dta = rotation_vector(ra_cur * ra_prev.transpose()).dot(&(ra_prev * ua));
                let dtb = rotation_vector(rb_cur * rb_prev.transpose()).dot(&(rb_prev * ub));
                updates.push((idx, dta, dtb));
            }
        }
        for (idx, dta, dtb) in updates {
            self.joints[idx].gear_theta_a += dta;
            self.joints[idx].gear_theta_b += dtb;
        }
    }

    /// Projects body positions and orientations onto the exact holonomic constraint manifold
    /// $\Phi(\mathbf{q}) = \mathbf{0}$ using direct Cholesky Gauss–Newton iterations in COM coordinates.
    fn project_holonomic_manifold(
        &mut self,
        dyn_ids: &[String],
        dyn_map: &HashMap<String, usize>,
    ) {
        let n_dyn = dyn_ids.len();
        if n_dyn == 0 || self.joints.is_empty() {
            return;
        }

        let base_rotations: HashMap<String, Matrix3<f64>> = dyn_ids
            .iter()
            .map(|id| (id.clone(), self.body_pose(id).1))
            .collect();
        let base_gears: Vec<(f64, f64)> = self
            .joints
            .iter()
            .map(|j| (j.gear_theta_a, j.gear_theta_b))
            .collect();

        // Count how many translational joints each dynamic body participates in.
        // A body with only 1 translational joint can close its anchor position purely by COM
        // translation without perturbing any free rotational DOF.
        let mut trans_joint_counts: HashMap<String, usize> = HashMap::new();
        for j in &self.joints {
            if !matches!(j.joint_type, CadJointType::Gear { .. }) {
                *trans_joint_counts.entry(j.part_a.clone()).or_insert(0) += 1;
                *trans_joint_counts.entry(j.part_b.clone()).or_insert(0) += 1;
            }
        }

        type HolonomicRow = (
            Option<usize>,
            Option<usize>,
            Vector3<f64>,
            Vector3<f64>,
            Vector3<f64>,
            Vector3<f64>,
            f64,
        );

        for _iter in 0..8 {
            let mut hol_rows: Vec<HolonomicRow> = Vec::new();

            for (j_idx, j) in self.joints.iter().enumerate() {
                let col_a = dyn_map.get(&j.part_a).copied();
                let col_b = dyn_map.get(&j.part_b).copied();
                if col_a.is_none() && col_b.is_none() {
                    continue;
                }
                let ba = &self.bodies[&j.part_a];
                let bb = &self.bodies[&j.part_b];
                let pa = ba.position;
                let ra = ba.rotation_matrix();
                let com_a = ba.world_com();
                let pb = bb.position;
                let rb = bb.rotation_matrix();
                let com_b = bb.world_com();
                let multi_a = trans_joint_counts.get(&j.part_a).copied().unwrap_or(0) > 1;
                let multi_b = trans_joint_counts.get(&j.part_b).copied().unwrap_or(0) > 1;

                match &j.joint_type {
                    CadJointType::Fixed { anchor_a, anchor_b } => {
                        let p_anchor_a = pa + ra * Vector3::from(*anchor_a);
                        let p_anchor_b = pb + rb * Vector3::from(*anchor_b);
                        let r_a = p_anchor_a - com_a;
                        let r_b = p_anchor_b - com_b;
                        let dp = p_anchor_a - p_anchor_b;
                        let e_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
                        for k in 0..3 {
                            let ek = e_axes[k];
                            hol_rows.push((
                                col_a,
                                col_b,
                                ek,
                                r_a.cross(&ek),
                                -ek,
                                -r_b.cross(&ek),
                                dp[k],
                            ));
                        }
                        let drot = rotation_vector(rb * (ra * j.initial_rel_rot).transpose());
                        for k in 0..3 {
                            let ek = e_axes[k];
                            hol_rows.push((
                                col_a,
                                col_b,
                                Vector3::zeros(),
                                -ek,
                                Vector3::zeros(),
                                ek,
                                drot[k],
                            ));
                        }
                    }
                    CadJointType::Spherical { anchor_a, anchor_b } => {
                        let p_anchor_a = pa + ra * Vector3::from(*anchor_a);
                        let p_anchor_b = pb + rb * Vector3::from(*anchor_b);
                        let r_a = if multi_a { p_anchor_a - com_a } else { Vector3::zeros() };
                        let r_b = if multi_b { p_anchor_b - com_b } else { Vector3::zeros() };
                        let dp = p_anchor_a - p_anchor_b;
                        let e_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
                        for k in 0..3 {
                            let ek = e_axes[k];
                            hol_rows.push((
                                col_a,
                                col_b,
                                ek,
                                r_a.cross(&ek),
                                -ek,
                                -r_b.cross(&ek),
                                dp[k],
                            ));
                        }
                    }
                    CadJointType::Revolute {
                        anchor_a,
                        anchor_b,
                        axis,
                        limits,
                    } => {
                        let p_anchor_a = pa + ra * Vector3::from(*anchor_a);
                        let p_anchor_b = pb + rb * Vector3::from(*anchor_b);
                        let r_a = if multi_a { p_anchor_a - com_a } else { Vector3::zeros() };
                        let r_b = if multi_b { p_anchor_b - com_b } else { Vector3::zeros() };
                        let dp = p_anchor_a - p_anchor_b;
                        let e_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
                        for k in 0..3 {
                            let ek = e_axes[k];
                            hol_rows.push((
                                col_a,
                                col_b,
                                ek,
                                r_a.cross(&ek),
                                -ek,
                                -r_b.cross(&ek),
                                dp[k],
                            ));
                        }
                        let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::z());
                        let z_a = ra * u_a;
                        let (x_a, y_a) = perpendicular_axes(&z_a);
                        let z_b = rb * u_a;
                        hol_rows.push((
                            col_a,
                            col_b,
                            Vector3::zeros(),
                            -x_a,
                            Vector3::zeros(),
                            x_a,
                            -y_a.dot(&z_b),
                        ));
                        hol_rows.push((
                            col_a,
                            col_b,
                            Vector3::zeros(),
                            -y_a,
                            Vector3::zeros(),
                            y_a,
                            x_a.dot(&z_b),
                        ));
                        if is_zero_range_limits(*limits) {
                            let target = limits.map(|l| l[0]).or(j.initial_coord).unwrap_or(0.0);
                            let cur = j
                                .joint_type
                                .current_coordinate(pa, ra, pb, rb)
                                .unwrap_or(target);
                            hol_rows.push((
                                col_a,
                                col_b,
                                Vector3::zeros(),
                                -z_a,
                                Vector3::zeros(),
                                z_a,
                                cur - target,
                            ));
                        } else if let Some(m) = j.motor {
                            if m.max_force > 0.0 {
                                let r_rel_motor =
                                    *UnitQuaternion::from_scaled_axis(u_a * j.motor_accum_coord)
                                        .to_rotation_matrix()
                                        .matrix();
                                let r_b_desired = ra * j.initial_rel_rot * r_rel_motor;
                                let err_vec = rotation_vector(rb * r_b_desired.transpose());
                                let phi_motor = z_a.dot(&err_vec);
                                hol_rows.push((
                                    col_a,
                                    col_b,
                                    Vector3::zeros(),
                                    -z_a,
                                    Vector3::zeros(),
                                    z_a,
                                    phi_motor,
                                ));
                            }
                        }
                    }
                    CadJointType::Prismatic {
                        anchor_a,
                        anchor_b,
                        axis,
                        limits,
                    } => {
                        let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::x());
                        let z_a = ra * u_a;
                        let (x_a, y_a) = perpendicular_axes(&z_a);
                        let drot = rotation_vector(rb * (ra * j.initial_rel_rot).transpose());
                        let e_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
                        for k in 0..3 {
                            let ek = e_axes[k];
                            hol_rows.push((
                                col_a,
                                col_b,
                                Vector3::zeros(),
                                -ek,
                                Vector3::zeros(),
                                ek,
                                drot[k],
                            ));
                        }
                        let p_anchor_a = pa + ra * Vector3::from(*anchor_a);
                        let p_anchor_b = pb + rb * Vector3::from(*anchor_b);
                        let r_b_com = p_anchor_b - com_b;
                        let r_ab_from_com_a = p_anchor_b - com_a;
                        let delta_r = p_anchor_b - p_anchor_a;
                        for (perp_axis, err_val) in
                            [(x_a, x_a.dot(&delta_r)), (y_a, y_a.dot(&delta_r))]
                        {
                            hol_rows.push((
                                col_a,
                                col_b,
                                -perp_axis,
                                perp_axis.cross(&r_ab_from_com_a),
                                perp_axis,
                                r_b_com.cross(&perp_axis),
                                err_val,
                            ));
                        }
                        if is_zero_range_limits(*limits) {
                            let target = limits.map(|l| l[0]).or(j.initial_coord).unwrap_or(0.0);
                            hol_rows.push((
                                col_a,
                                col_b,
                                -z_a,
                                z_a.cross(&r_ab_from_com_a),
                                z_a,
                                r_b_com.cross(&z_a),
                                delta_r.dot(&z_a) - target,
                            ));
                        } else if let Some(m) = j.motor {
                            if m.max_force > 0.0 {
                                let target = j.initial_coord.unwrap_or(0.0) + j.motor_accum_coord;
                                hol_rows.push((
                                    col_a,
                                    col_b,
                                    -z_a,
                                    z_a.cross(&r_ab_from_com_a),
                                    z_a,
                                    r_b_com.cross(&z_a),
                                    delta_r.dot(&z_a) - target,
                                ));
                            }
                        }
                    }
                    CadJointType::Gear {
                        ratio,
                        axis_a,
                        axis_b,
                    } => {
                        let ra_base = base_rotations.get(&j.part_a).copied().unwrap_or(ra);
                        let rb_base = base_rotations.get(&j.part_b).copied().unwrap_or(rb);
                        let (_, _, ua) = local_orthonormal_basis(*axis_a, Vector3::z());
                        let (_, _, ub) = local_orthonormal_basis(*axis_b, Vector3::z());
                        let z_a = ra * ua;
                        let z_b = rb * ub;
                        let dta = rotation_vector(ra * ra_base.transpose()).dot(&(ra_base * ua));
                        let dtb = rotation_vector(rb * rb_base.transpose()).dot(&(rb_base * ub));
                        let phi_gear =
                            ratio * (base_gears[j_idx].0 + dta) + (base_gears[j_idx].1 + dtb);
                        hol_rows.push((
                            col_a,
                            col_b,
                            Vector3::zeros(),
                            z_a * (*ratio),
                            Vector3::zeros(),
                            z_b,
                            phi_gear,
                        ));
                    }
                }
            }

            let m_rows = hol_rows.len();
            if m_rows == 0 {
                break;
            }
            let mut phi = DVector::<f64>::zeros(m_rows);
            let mut j_mat = DMatrix::<f64>::zeros(m_rows, 6 * n_dyn);
            for (r, (ca, cb, la, aa, lb, ab, err)) in hol_rows.iter().enumerate() {
                phi[r] = *err;
                if let Some(c) = ca {
                    let b = 6 * c;
                    j_mat.fixed_view_mut::<1, 3>(r, b).copy_from(&la.transpose());
                    j_mat.fixed_view_mut::<1, 3>(r, b + 3).copy_from(&aa.transpose());
                }
                if let Some(c) = cb {
                    let b = 6 * c;
                    j_mat.fixed_view_mut::<1, 3>(r, b).copy_from(&lb.transpose());
                    j_mat.fixed_view_mut::<1, 3>(r, b + 3).copy_from(&ab.transpose());
                }
            }

            if phi.amax() <= 1e-14 {
                break;
            }

            let jt = j_mat.transpose();
            let mut j_jt = &j_mat * &jt;
            let diag_max = j_jt.diagonal().amax().max(1.0);
            let reg = 1e-11 * diag_max;
            for r in 0..m_rows {
                j_jt[(r, r)] += reg;
            }
            let neg_phi = -&phi;
            let y_sol = if let Some(chol) = j_jt.clone().cholesky() {
                chol.solve(&neg_phi)
            } else {
                j_jt.lu().solve(&neg_phi).unwrap_or_else(|| DVector::zeros(m_rows))
            };
            let dq = &jt * y_sol;

            for (idx, id) in dyn_ids.iter().enumerate() {
                let base = 6 * idx;
                let dx_com = Vector3::new(dq[base], dq[base + 1], dq[base + 2]);
                let dtheta = Vector3::new(dq[base + 3], dq[base + 4], dq[base + 5]);
                if let Some(b) = self.bodies.get_mut(id) {
                    let old_com = b.world_com();
                    let drot = UnitQuaternion::from_scaled_axis(dtheta);
                    b.orientation = drot * b.orientation;
                    let _ = b.orientation.renormalize();
                    b.set_world_com(old_com + dx_com);
                }
            }

            // Synchronize gear angles with projected rotations
            for (j_idx, j) in self.joints.iter_mut().enumerate() {
                if let CadJointType::Gear { axis_a, axis_b, .. } = &j.joint_type {
                    let ra = self
                        .bodies
                        .get(&j.part_a)
                        .map(DaeBody::rotation_matrix)
                        .unwrap_or_else(Matrix3::identity);
                    let rb = self
                        .bodies
                        .get(&j.part_b)
                        .map(DaeBody::rotation_matrix)
                        .unwrap_or_else(Matrix3::identity);
                    let ra_base = base_rotations.get(&j.part_a).copied().unwrap_or(ra);
                    let rb_base = base_rotations.get(&j.part_b).copied().unwrap_or(rb);
                    let (_, _, ua) = local_orthonormal_basis(*axis_a, Vector3::z());
                    let (_, _, ub) = local_orthonormal_basis(*axis_b, Vector3::z());
                    let dta = rotation_vector(ra * ra_base.transpose()).dot(&(ra_base * ua));
                    let dtb = rotation_vector(rb * rb_base.transpose()).dot(&(rb_base * ub));
                    j.gear_theta_a = base_gears[j_idx].0 + dta;
                    j.gear_theta_b = base_gears[j_idx].1 + dtb;
                }
            }
        }
    }

    /// Projects velocities $\mathbf{v}_{k+1}$ onto the velocity constraint manifold
    /// $J(\mathbf{q}_{k+1})\mathbf{v}_{k+1} = \mathbf{b}_{\text{motor}}$ and extracts the exact
    /// instantaneous acceleration-level KKT reaction forces $\boldsymbol{\lambda}_{k+1}$ at $(\mathbf{q}_{k+1}, \mathbf{v}_{k+1})$.
    fn project_velocity_and_compute_reactions(
        &mut self,
        dyn_ids: &[String],
        dyn_map: &HashMap<String, usize>,
        dt: f64,
    ) {
        let n_dyn = dyn_ids.len();
        if n_dyn == 0 || self.joints.is_empty() {
            return;
        }

        let mut v_vec = DVector::<f64>::zeros(6 * n_dyn);
        let mut m_inv_blocks: Vec<(f64, Matrix3<f64>)> = Vec::with_capacity(n_dyn);
        for (idx, id) in dyn_ids.iter().enumerate() {
            let b = &self.bodies[id];
            let base = 6 * idx;
            v_vec.fixed_rows_mut::<3>(base).copy_from(&b.linear_velocity);
            v_vec.fixed_rows_mut::<3>(base + 3).copy_from(&b.angular_velocity);
            m_inv_blocks.push((b.inv_mass, b.world_inv_inertia()));
        }

        // First assemble rows with dt -> 0 (pure velocity manifold J v = b_motor, zero centripetal bias)
        // to project v_{k+1} onto the tangent space of the projected pose q_{k+1}.
        let vel_rows = self.assemble_constraint_rows(dyn_map, 0.0, false, false);
        let m_rows = vel_rows.len();
        if m_rows == 0 {
            return;
        }

        let mut trans_joint_counts: HashMap<String, usize> = HashMap::new();
        for j in &self.joints {
            if !matches!(j.joint_type, CadJointType::Gear { .. }) {
                *trans_joint_counts.entry(j.part_a.clone()).or_insert(0) += 1;
                *trans_joint_counts.entry(j.part_b.clone()).or_insert(0) += 1;
            }
        }

        let mut j_mat = DMatrix::<f64>::zeros(m_rows, 6 * n_dyn);
        let mut b_motor = DVector::<f64>::zeros(m_rows);
        let mut minv_jt_proj = DMatrix::<f64>::zeros(6 * n_dyn, m_rows);
        let mut minv_jt_full = DMatrix::<f64>::zeros(6 * n_dyn, m_rows);

        for (r, row) in vel_rows.iter().enumerate() {
            let j_ref = &self.joints[row.joint_idx];
            let is_pure_trans_row = row.torque_dir_on_a.norm_squared() == 0.0;
            let multi_a = trans_joint_counts.get(&j_ref.part_a).copied().unwrap_or(0) > 1;
            let multi_b = trans_joint_counts.get(&j_ref.part_b).copied().unwrap_or(0) > 1;

            if let Some(ca) = row.col_a {
                let b = 6 * ca;
                let (im, ii) = &m_inv_blocks[ca];
                j_mat.fixed_view_mut::<1, 3>(r, b).copy_from(&row.lin_a.transpose());
                j_mat.fixed_view_mut::<1, 3>(r, b + 3).copy_from(&row.ang_a.transpose());
                minv_jt_full.fixed_view_mut::<3, 1>(b, r).copy_from(&(row.lin_a * *im));
                minv_jt_full.fixed_view_mut::<3, 1>(b + 3, r).copy_from(&(ii * row.ang_a));
                minv_jt_proj.fixed_view_mut::<3, 1>(b, r).copy_from(&(row.lin_a * *im));
                if !is_pure_trans_row || multi_a {
                    minv_jt_proj.fixed_view_mut::<3, 1>(b + 3, r).copy_from(&(ii * row.ang_a));
                }
            }
            if let Some(cb) = row.col_b {
                let b = 6 * cb;
                let (im, ii) = &m_inv_blocks[cb];
                j_mat.fixed_view_mut::<1, 3>(r, b).copy_from(&row.lin_b.transpose());
                j_mat.fixed_view_mut::<1, 3>(r, b + 3).copy_from(&row.ang_b.transpose());
                minv_jt_full.fixed_view_mut::<3, 1>(b, r).copy_from(&(row.lin_b * *im));
                minv_jt_full.fixed_view_mut::<3, 1>(b + 3, r).copy_from(&(ii * row.ang_b));
                minv_jt_proj.fixed_view_mut::<3, 1>(b, r).copy_from(&(row.lin_b * *im));
                if !is_pure_trans_row || multi_b {
                    minv_jt_proj.fixed_view_mut::<3, 1>(b + 3, r).copy_from(&(ii * row.ang_b));
                }
            }
            b_motor[r] = if row.target_vel_rhs.is_finite() {
                row.target_vel_rhs
            } else {
                0.0
            };
        }

        let mut a_proj = &j_mat * &minv_jt_proj;
        for r in 0..m_rows {
            a_proj[(r, r)] += self.cfm_regularization * a_proj[(r, r)].abs().max(1.0);
        }

        let vel_deficit = &b_motor - &j_mat * &v_vec;
        let mu = if let Some(chol) = a_proj.clone().cholesky() {
            chol.solve(&vel_deficit)
        } else {
            a_proj.lu().solve(&vel_deficit).unwrap_or_else(|| DVector::zeros(m_rows))
        };
        v_vec += &minv_jt_proj * &mu;

        for (idx, id) in dyn_ids.iter().enumerate() {
            let base = 6 * idx;
            if let Some(b) = self.bodies.get_mut(id) {
                b.linear_velocity = v_vec.fixed_rows::<3>(base).into_owned();
                b.angular_velocity = v_vec.fixed_rows::<3>(base + 3).into_owned();
            }
        }

        // Now evaluate instantaneous acceleration-level KKT reaction forces at (q_{k+1}, v_{k+1})
        let mut a_mat = &j_mat * &minv_jt_full;
        for r in 0..m_rows {
            a_mat[(r, r)] += self.cfm_regularization * a_mat[(r, r)].abs().max(1.0);
        }

        let mut v_star = DVector::<f64>::zeros(6 * n_dyn);
        for (idx, id) in dyn_ids.iter().enumerate() {
            let b = &self.bodies[id];
            let base = 6 * idx;
            let (inv_m, inv_i_w) = &m_inv_blocks[idx];
            let i_w = b.world_inertia();
            let f_total = b.mass * self.gravity + b.external_force;
            let gyro = b.angular_velocity.cross(&(i_w * b.angular_velocity));
            let tau_total = b.external_torque - gyro;
            v_star
                .fixed_rows_mut::<3>(base)
                .copy_from(&(b.linear_velocity + *inv_m * f_total * dt));
            v_star
                .fixed_rows_mut::<3>(base + 3)
                .copy_from(&(b.angular_velocity + inv_i_w * tau_total * dt));
        }

        let acc_rows = self.assemble_constraint_rows(dyn_map, dt, false, false);
        let mut rhs_acc = DVector::<f64>::zeros(m_rows);
        let mut lambda_min = vec![f64::NEG_INFINITY; m_rows];
        let mut lambda_max = vec![f64::INFINITY; m_rows];
        for (r, row) in acc_rows.iter().enumerate().take(m_rows) {
            rhs_acc[r] = row.target_vel_rhs;
            lambda_min[r] = if row.min_force.is_finite() {
                row.min_force * dt
            } else {
                f64::NEG_INFINITY
            };
            lambda_max[r] = if row.max_force.is_finite() {
                row.max_force * dt
            } else {
                f64::INFINITY
            };
        }

        let b_acc = &rhs_acc - &j_mat * &v_star;
        let impulse = solve_box_constrained_kkt(&a_mat, &b_acc, &lambda_min, &lambda_max);

        for j in &mut self.joints {
            j.last_force_on_a = Vector3::zeros();
            j.last_torque_on_a = Vector3::zeros();
        }
        let inv_dt = 1.0 / dt;
        for (r, row) in acc_rows.iter().enumerate().take(m_rows) {
            let f = impulse[r] * inv_dt;
            let j = &mut self.joints[row.joint_idx];
            j.last_force_on_a += row.force_dir_on_a * f;
            j.last_torque_on_a += row.torque_dir_on_a * f;
        }
    }
}

struct AssembledConstraintRow {
    joint_idx: usize,
    col_a: Option<usize>,
    col_b: Option<usize>,
    lin_a: Vector3<f64>,
    ang_a: Vector3<f64>,
    lin_b: Vector3<f64>,
    ang_b: Vector3<f64>,
    force_dir_on_a: Vector3<f64>,
    torque_dir_on_a: Vector3<f64>,
    target_vel_rhs: f64,
    min_force: f64,
    max_force: f64,
}

fn rotation_vector(r: Matrix3<f64>) -> Vector3<f64> {
    let s_vec = 0.5
        * Vector3::new(
            r[(2, 1)] - r[(1, 2)],
            r[(0, 2)] - r[(2, 0)],
            r[(1, 0)] - r[(0, 1)],
        );
    let sin_t = s_vec.norm();
    let cos_t = (0.5 * (r.trace() - 1.0)).clamp(-1.0, 1.0);
    if sin_t < 1e-12 {
        s_vec
    } else {
        let theta = sin_t.atan2(cos_t);
        s_vec * (theta / sin_t)
    }
}

/// Solves the regularized symmetric positive-definite box-constrained KKT / LCP system:
/// $$A \boldsymbol{\lambda} = \mathbf{b}, \quad \lambda_{\min,i} \le \lambda_i \le \lambda_{\max,i}.$$
///
/// Uses direct Cholesky ($LL^T$) / LU when all active rows are within their box bounds, and an
/// active-set projection when unilateral limits or motor torque saturation engage.
fn solve_box_constrained_kkt(
    a_mat: &DMatrix<f64>,
    b_vec: &DVector<f64>,
    lo: &[f64],
    hi: &[f64],
) -> DVector<f64> {
    let m = b_vec.len();
    if m == 0 {
        return DVector::zeros(0);
    }

    // 1. Solve unconstrained linear system A * lambda = b via Cholesky (with LU fallback)
    let unconstrained = if let Some(chol) = a_mat.clone().cholesky() {
        chol.solve(b_vec)
    } else {
        a_mat.clone().lu().solve(b_vec).unwrap_or_else(|| DVector::zeros(m))
    };

    let within_bounds = (0..m).all(|i| unconstrained[i] >= lo[i] && unconstrained[i] <= hi[i]);
    if within_bounds {
        return unconstrained;
    }

    // 2. Active-set projection for box-constrained rows (motor torque saturation or active joint limits)
    let mut lambda = unconstrained;
    for i in 0..m {
        lambda[i] = lambda[i].clamp(lo[i], hi[i]);
    }

    for _as_iter in 0..8 {
        let mut free_indices = Vec::new();
        let mut clamped_indices = Vec::new();
        for i in 0..m {
            let res_i = (a_mat.row(i) * &lambda)[0] - b_vec[i];
            if (lambda[i] <= lo[i] && res_i > 0.0) || (lambda[i] >= hi[i] && res_i < 0.0) {
                clamped_indices.push(i);
            } else {
                free_indices.push(i);
            }
        }
        if free_indices.is_empty() {
            break;
        }

        let nf = free_indices.len();
        let mut a_ff = DMatrix::<f64>::zeros(nf, nf);
        let mut b_f = DVector::<f64>::zeros(nf);
        for (ri, &i) in free_indices.iter().enumerate() {
            let mut rhs_i = b_vec[i];
            for &c in &clamped_indices {
                rhs_i -= a_mat[(i, c)] * lambda[c];
            }
            b_f[ri] = rhs_i;
            for (cj, &j) in free_indices.iter().enumerate() {
                a_ff[(ri, cj)] = a_mat[(i, j)];
            }
        }

        let sol_f = if let Some(chol) = a_ff.clone().cholesky() {
            chol.solve(&b_f)
        } else {
            a_ff.lu().solve(&b_f).unwrap_or(b_f)
        };

        let mut any_newly_clamped = false;
        for (ri, &i) in free_indices.iter().enumerate() {
            let v = sol_f[ri];
            if v < lo[i] {
                lambda[i] = lo[i];
                any_newly_clamped = true;
            } else if v > hi[i] {
                lambda[i] = hi[i];
                any_newly_clamped = true;
            } else {
                lambda[i] = v;
            }
        }
        if !any_newly_clamped {
            break;
        }
    }

    lambda
}

// ============================================================================
// Benchmark Suite (`P4-T1` / `ADR-004`): 4 Canonical Engineering Benchmarks
// ============================================================================

/// Accuracy metrics for Benchmark 1: Free Fall under Gravity ($T = 1.0\text{ s}, \Delta t = 5\text{ ms}$).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FreeFallBenchmarkResult {
    /// Solver name (`"DirectKktIndex2 (f64)"`, `"PgsBaumgarte (f64)"`).
    pub solver_name: String,
    /// Simulated final vertical position $y(T)$ in meters (analytical: $5.095000\text{ m}$).
    pub final_y_m: f64,
    /// Analytical vertical position $y_{\text{exact}}(T) = 5.095\text{ m}$.
    pub analytical_y_m: f64,
    /// Absolute vertical position error $|y(T) - y_{\text{exact}}(T)|$ in meters.
    pub pos_error_m: f64,
    /// Simulated final vertical velocity $v_y(T)$ in m/s (analytical: $-9.810000\text{ m/s}$).
    pub final_vy_m_s: f64,
    /// Absolute vertical velocity error $|v_y(T) - v_{y,\text{exact}}(T)|$ in m/s.
    pub vel_error_m_s: f64,
    /// Wall-clock execution time in microseconds.
    pub elapsed_us: f64,
}

/// Accuracy metrics for Benchmark 2: Rotating Cantilevered Rod ($L=0.20\text{ m}, \omega_0=10\text{ rad/s}$).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RotatingRodBenchmarkResult {
    /// Solver name.
    pub solver_name: String,
    /// Analytical inward centripetal reaction force $F_x^{\text{local}} = -m \omega_0^2 r_c$ (N).
    pub analytical_fx_local_n: f64,
    /// Simulated local reaction force $F_x^{\text{local}}$ (N) at $t = 0.20\text{ s}$.
    pub simulated_fx_local_n: f64,
    /// Relative centripetal force error $|F_x - F_{x,\text{exact}}| / |F_{x,\text{exact}}|$ in percent.
    pub force_rel_error_pct: f64,
    /// Angular velocity drift $|\omega_z(T) - \omega_0|$ in rad/s.
    pub omega_drift_rad_s: f64,
    /// Pivot position drift $\|\mathbf{p}_{\text{anchor}}(T)\|$ in meters.
    pub pivot_drift_m: f64,
    /// Wall-clock execution time in microseconds.
    pub elapsed_us: f64,
}

/// Accuracy metrics for Benchmark 3: Planar Slider-Crank 4-Bar Closed Kinematic Loop.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SliderCrankBenchmarkResult {
    /// Solver name.
    pub solver_name: String,
    /// Maximum slider position error $\max_t |x_C(t) - x_{\text{exact}}(\theta(t))|$ over 1 full revolution (meters).
    pub max_slider_pos_error_m: f64,
    /// RMS slider position error over 1 full revolution (meters).
    pub rms_slider_pos_error_m: f64,
    /// Maximum slider velocity error $\max_t |v_C(t) - v_{\text{exact}}(\theta(t))|$ (m/s).
    pub max_slider_vel_error_m_s: f64,
    /// Maximum holonomic loop-closure residual $\|\Phi(q)\|_\infty$ across all steps (meters/radians).
    pub max_closure_residual: f64,
    /// Wall-clock execution time in microseconds.
    pub elapsed_us: f64,
}

/// Accuracy metrics for Benchmark 4: Spur Gear Pair over 10 Full Pinion Revolutions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GearPairBenchmarkResult {
    /// Solver name.
    pub solver_name: String,
    /// Total pinion rotation angle $\theta_1(T)$ in radians (analytical: $20\pi \approx 62.831853\text{ rad}$).
    pub pinion_angle_rad: f64,
    /// Total driven gear rotation angle $\theta_2(T)$ in radians (analytical: $-10\pi \approx -31.415927\text{ rad}$).
    pub gear_angle_rad: f64,
    /// Kinematic phase error $|\theta_2(T) + \text{ratio}\cdot \theta_1(T)|$ after 10 revolutions (radians).
    pub phase_error_rad: f64,
    /// Relative angular velocity ratio error $|\omega_2 / \omega_1 - (-\text{ratio})| / \text{ratio}$ in percent.
    pub velocity_ratio_error_pct: f64,
    /// Transmitted motor holding torque $\tau_{\text{motor},1}$ under external load $\tau_{\text{load},2} = -4.0\text{ N}\cdot\text{m}$
    /// (analytical: $-\text{ratio}\cdot\tau_{\text{load},2} = +2.0\text{ N}\cdot\text{m}$).
    pub transmitted_motor_torque_nm: f64,
    /// Relative torque transmission error in percent.
    pub torque_error_pct: f64,
    /// Wall-clock execution time in microseconds.
    pub elapsed_us: f64,
}

/// Complete comparison report across all 4 `P4-T1` benchmarks.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaeBenchmarkComparisonReport {
    /// Benchmark 1 results (Direct KKT f64, PGS Baumgarte f64).
    pub free_fall: Vec<FreeFallBenchmarkResult>,
    /// Benchmark 2 results.
    pub rotating_rod: Vec<RotatingRodBenchmarkResult>,
    /// Benchmark 3 results.
    pub slider_crank: Vec<SliderCrankBenchmarkResult>,
    /// Benchmark 4 results.
    pub gear_pair_10_revs: Vec<GearPairBenchmarkResult>,
}

/// Runs Benchmark 1 (Free Fall under Gravity) for a `DaeSolverCandidate`.
#[must_use]
pub fn run_free_fall_dae(candidate: DaeSolverCandidate) -> FreeFallBenchmarkResult {
    let solver_name = match candidate {
        DaeSolverCandidate::DirectKktIndex2Projection => "DirectKktIndex2 (f64)",
        DaeSolverCandidate::SemiImplicitEulerPgsBaumgarte => "PgsBaumgarte (f64)",
    }
    .to_string();

    let g = -9.81_f64;
    let y0 = 10.0_f64;
    let duration_s = 1.0_f64;
    let dt = 0.005_f64;
    let steps = (duration_s / dt).round() as usize;

    let mut world = DaeWorld::new(candidate, Vector3::new(0.0, g, 0.0));
    let body = DaeBody::uniform_box(
        "box_falling",
        62.8,
        [0.2, 0.2, 0.2],
        Vector3::zeros(),
        Vector3::new(0.0, y0, 0.0),
    );
    world.add_body(body);

    let t0 = Instant::now();
    for _ in 0..steps {
        world.step(dt);
    }
    let elapsed_us = t0.elapsed().as_secs_f64() * 1e6;

    let b = &world.bodies["box_falling"];
    let final_y_m = b.position.y;
    let final_vy_m_s = b.linear_velocity.y;
    let analytical_y_m = y0 + 0.5 * g * duration_s * duration_s;
    let analytical_vy_m_s = g * duration_s;

    FreeFallBenchmarkResult {
        solver_name,
        final_y_m,
        analytical_y_m,
        pos_error_m: (final_y_m - analytical_y_m).abs(),
        final_vy_m_s,
        vel_error_m_s: (final_vy_m_s - analytical_vy_m_s).abs(),
        elapsed_us,
    }
}

/// Runs Benchmark 2 (Rotating Rod Centripetal Reaction) for a `DaeSolverCandidate`.
#[must_use]
pub fn run_rotating_rod_dae(candidate: DaeSolverCandidate) -> RotatingRodBenchmarkResult {
    let solver_name = match candidate {
        DaeSolverCandidate::DirectKktIndex2Projection => "DirectKktIndex2 (f64)",
        DaeSolverCandidate::SemiImplicitEulerPgsBaumgarte => "PgsBaumgarte (f64)",
    }
    .to_string();

    let length_m = 0.20_f64;
    let r_c = length_m * 0.5;
    let mass_kg = 7850.0 * (length_m * 0.02 * 0.02); // 0.628 kg
    let omega_0 = 10.0_f64;
    let dt = 1.0 / 240.0;
    let steps = 48usize; // t = 0.20 s (~114.6 deg rotation)

    let mut world = DaeWorld::new(candidate, Vector3::zeros());
    let mut rod = DaeBody::uniform_box(
        "rotating_rod",
        mass_kg,
        [length_m, 0.02, 0.02],
        Vector3::new(r_c, 0.0, 0.0),
        Vector3::zeros(),
    );
    rod.angular_velocity = Vector3::new(0.0, 0.0, omega_0);
    rod.linear_velocity = Vector3::new(0.0, omega_0 * r_c, 0.0);
    world.add_body(rod);

    world.add_joint(
        "pivot_z",
        "ground",
        "rotating_rod",
        CadJointType::Revolute {
            anchor_a: [0.0, 0.0, 0.0],
            anchor_b: [0.0, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            limits: None,
        },
        None,
    );

    let t0 = Instant::now();
    for _ in 0..steps {
        world.step(dt);
    }
    let elapsed_us = t0.elapsed().as_secs_f64() * 1e6;

    let b = &world.bodies["rotating_rod"];
    let j = &world.joints[0];
    // Newton's 3rd Law: force on `part_b` ("rotating_rod") is `-last_force_on_a`
    let force_on_rod_world = -j.last_force_on_a;
    let force_on_rod_local = b.rotation_matrix().transpose() * force_on_rod_world;

    let analytical_fx_local_n = -mass_kg * omega_0 * omega_0 * r_c;
    let simulated_fx_local_n = force_on_rod_local.x;
    let force_rel_error_pct =
        ((simulated_fx_local_n - analytical_fx_local_n).abs() / analytical_fx_local_n.abs()) * 100.0;
    let omega_drift_rad_s = (b.angular_velocity.z - omega_0).abs();
    let pivot_drift_m = b.position.norm();

    RotatingRodBenchmarkResult {
        solver_name,
        analytical_fx_local_n,
        simulated_fx_local_n,
        force_rel_error_pct,
        omega_drift_rad_s,
        pivot_drift_m,
        elapsed_us,
    }
}

/// Runs Benchmark 3 (Planar Slider-Crank 4-Bar Closed Kinematic Loop) for a `DaeSolverCandidate`.
#[must_use]
pub fn run_slider_crank_dae(candidate: DaeSolverCandidate) -> SliderCrankBenchmarkResult {
    let solver_name = match candidate {
        DaeSolverCandidate::DirectKktIndex2Projection => "DirectKktIndex2 (f64)",
        DaeSolverCandidate::SemiImplicitEulerPgsBaumgarte => "PgsBaumgarte (f64)",
    }
    .to_string();

    // Exact Pythagorean initial state at theta_0 = pi/2:
    // Crank R = 0.03 m from A=(0,0,0) to B=(0, 0.03, 0).
    // Conrod L = 0.05 m from B=(0, 0.03, 0) to C=(0.04, 0, 0).
    // Slider at C=(0.04, 0, 0) sliding along +X.
    let r_crank = 0.03_f64;
    let l_rod = 0.05_f64;
    let omega = std::f64::consts::TAU; // 1 rev/s (2*pi rad/s)
    let dt = 0.002_f64;
    let steps = 500usize; // 1.0 s (full 360° revolution)

    let mut world = DaeWorld::new(candidate, Vector3::zeros());

    // Initial velocities at theta = pi/2:
    // B = (0, R, 0), v_B = (-R*omega, 0, 0).
    // Since x_C(pi/2) = 0.04 and cos(pi/2) = 0, v_C(pi/2) = -R*omega*sin(pi/2)*(1 + 0) = -R*omega.
    // Because v_B == v_C == (-R*omega, 0, 0) at t=0, the connecting rod has zero initial angular velocity!
    let v0_x = -r_crank * omega;

    let mut crank = DaeBody::uniform_box(
        "sc_crank",
        0.15,
        [0.008, r_crank, 0.008],
        Vector3::zeros(),
        Vector3::new(0.0, r_crank * 0.5, 0.0),
    );
    crank.angular_velocity = Vector3::new(0.0, 0.0, omega);
    crank.linear_velocity = Vector3::new(v0_x * 0.5, 0.0, 0.0);

    let mut conrod = DaeBody::uniform_box(
        "sc_rod",
        0.20,
        [l_rod, 0.008, 0.008],
        Vector3::zeros(),
        Vector3::new(0.02, 0.015, 0.0),
    );
    conrod.linear_velocity = Vector3::new(v0_x, 0.0, 0.0);

    let mut slider = DaeBody::uniform_box(
        "sc_slider",
        0.25,
        [0.02, 0.02, 0.02],
        Vector3::zeros(),
        Vector3::new(0.04, 0.0, 0.0),
    );
    slider.linear_velocity = Vector3::new(v0_x, 0.0, 0.0);

    world.add_body(crank);
    world.add_body(conrod);
    world.add_body(slider);

    world.add_joint(
        "j_sc_crank",
        "ground",
        "sc_crank",
        CadJointType::Revolute {
            anchor_a: [0.0, 0.0, 0.0],
            anchor_b: [0.0, -0.015, 0.0],
            axis: [0.0, 0.0, 1.0],
            limits: None,
        },
        Some(MotorConfig::new(omega, 5000.0)),
    );
    world.add_joint(
        "j_sc_pin_b",
        "sc_crank",
        "sc_rod",
        CadJointType::Revolute {
            anchor_a: [0.0, 0.015, 0.0],
            anchor_b: [-0.02, 0.015, 0.0],
            axis: [0.0, 0.0, 1.0],
            limits: None,
        },
        None,
    );
    world.add_joint(
        "j_sc_pin_c",
        "sc_rod",
        "sc_slider",
        CadJointType::Revolute {
            anchor_a: [0.02, -0.015, 0.0],
            anchor_b: [0.0, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            limits: None,
        },
        None,
    );
    world.add_joint(
        "j_sc_rail",
        "sc_slider",
        "ground",
        CadJointType::Prismatic {
            anchor_a: [0.0, 0.0, 0.0],
            anchor_b: [0.0, 0.0, 0.0],
            axis: [1.0, 0.0, 0.0],
            limits: None,
        },
        None,
    );

    let mut max_pos_err = 0.0_f64;
    let mut sum_sq_pos_err = 0.0_f64;
    let mut max_vel_err = 0.0_f64;
    let mut max_closure = 0.0_f64;

    let t0 = Instant::now();
    for s in 1..=steps {
        world.step(dt);
        let t = (s as f64) * dt;
        let theta = std::f64::consts::FRAC_PI_2 + omega * t;
        let sin_t = theta.sin();
        let cos_t = theta.cos();
        let rad = (l_rod * l_rod - r_crank * r_crank * sin_t * sin_t).sqrt();
        let x_exact = r_crank * cos_t + rad;
        let v_exact = -r_crank * omega * sin_t * (1.0 + (r_crank * cos_t) / rad);

        let slider_b = &world.bodies["sc_slider"];
        let pos_err = (slider_b.position.x - x_exact).abs();
        let vel_err = (slider_b.linear_velocity.x - v_exact).abs();
        let closure = world.max_constraint_residual();

        max_pos_err = max_pos_err.max(pos_err);
        sum_sq_pos_err += pos_err * pos_err;
        max_vel_err = max_vel_err.max(vel_err);
        max_closure = max_closure.max(closure);
    }
    let elapsed_us = t0.elapsed().as_secs_f64() * 1e6;
    let rms_slider_pos_error_m = (sum_sq_pos_err / (steps as f64)).sqrt();

    SliderCrankBenchmarkResult {
        solver_name,
        max_slider_pos_error_m: max_pos_err,
        rms_slider_pos_error_m,
        max_slider_vel_error_m_s: max_vel_err,
        max_closure_residual: max_closure,
        elapsed_us,
    }
}

/// Runs Benchmark 4 (Spur Gear Pair over 10 Full Pinion Revolutions with Load Torque) for a `DaeSolverCandidate`.
#[must_use]
pub fn run_gear_pair_10_revs_dae(candidate: DaeSolverCandidate) -> GearPairBenchmarkResult {
    let solver_name = match candidate {
        DaeSolverCandidate::DirectKktIndex2Projection => "DirectKktIndex2 (f64)",
        DaeSolverCandidate::SemiImplicitEulerPgsBaumgarte => "PgsBaumgarte (f64)",
    }
    .to_string();

    let ratio = 0.5_f64; // z1 = 12, z2 = 24 -> w2 = -0.5 * w1
    let omega_1 = 20.0 * std::f64::consts::PI; // 10 rev/s -> 10 full revolutions in 1.0 s
    let load_torque_2 = -4.0_f64; // N*m resisting rotation of gear_b (which rotates at -10*pi rad/s)
    let dt = 0.002_f64;
    let steps = 500usize; // 1.0 s

    let mut world = DaeWorld::new(candidate, Vector3::zeros());

    let mut pinion = DaeBody::uniform_cylinder_z("pinion", 0.25, 0.012, 0.010, Vector3::zeros());
    pinion.angular_velocity = Vector3::new(0.0, 0.0, omega_1);

    let mut gear = DaeBody::uniform_cylinder_z(
        "gear",
        1.0,
        0.024,
        0.010,
        Vector3::new(0.036, 0.0, 0.0),
    );
    gear.angular_velocity = Vector3::new(0.0, 0.0, -ratio * omega_1);
    gear.external_torque = Vector3::new(0.0, 0.0, load_torque_2);

    world.add_body(pinion);
    world.add_body(gear);

    world.add_joint(
        "j_pinion",
        "ground",
        "pinion",
        CadJointType::Revolute {
            anchor_a: [0.0, 0.0, 0.0],
            anchor_b: [0.0, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            limits: None,
        },
        Some(MotorConfig::new(omega_1, 1000.0)),
    );
    world.add_joint(
        "j_gear",
        "ground",
        "gear",
        CadJointType::Revolute {
            anchor_a: [0.036, 0.0, 0.0],
            anchor_b: [0.0, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            limits: None,
        },
        None,
    );
    world.add_joint(
        "j_mesh",
        "pinion",
        "gear",
        CadJointType::Gear {
            ratio,
            axis_a: [0.0, 0.0, 1.0],
            axis_b: [0.0, 0.0, 1.0],
        },
        None,
    );

    let t0 = Instant::now();
    for _ in 0..steps {
        world.step(dt);
    }
    let elapsed_us = t0.elapsed().as_secs_f64() * 1e6;

    let gear_joint = &world.joints[2];
    let pinion_angle_rad = gear_joint.gear_theta_a;
    let gear_angle_rad = gear_joint.gear_theta_b;
    let phase_error_rad = (gear_angle_rad + ratio * pinion_angle_rad).abs();

    let w1 = world.bodies["pinion"].angular_velocity.z;
    let w2 = world.bodies["gear"].angular_velocity.z;
    let actual_ratio = -w2 / w1;
    let velocity_ratio_error_pct = ((actual_ratio - ratio).abs() / ratio) * 100.0;

    // Torque applied by motor on `pinion` ("part_b" of `j_pinion`) is `-last_torque_on_a.z`.
    // At steady state: tau_motor + ratio * tau_mesh = 0, and tau_mesh + load_torque_2 = 0
    // => tau_motor = ratio * load_torque_2 = -2.0 N*m (magnitude 2.0 N*m).
    let tau_on_pinion_z = -world.joints[0].last_torque_on_a.z;
    let expected_tau_mag = (ratio * load_torque_2).abs();
    let transmitted_motor_torque_nm = tau_on_pinion_z.abs();
    let torque_error_pct =
        ((transmitted_motor_torque_nm - expected_tau_mag).abs() / expected_tau_mag) * 100.0;

    GearPairBenchmarkResult {
        solver_name,
        pinion_angle_rad,
        gear_angle_rad,
        phase_error_rad,
        velocity_ratio_error_pct,
        transmitted_motor_torque_nm,
        torque_error_pct,
        elapsed_us,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_direct_kkt_free_fall_exact_machine_precision() {
        let res = run_free_fall_dae(DaeSolverCandidate::DirectKktIndex2Projection);
        assert!(
            res.pos_error_m < 1e-12,
            "Expected machine-precision free-fall position, got error {} m",
            res.pos_error_m
        );
        assert!(
            res.vel_error_m_s < 1e-12,
            "Expected machine-precision free-fall velocity, got error {} m/s",
            res.vel_error_m_s
        );
    }

    #[test]
    fn test_direct_kkt_rotating_rod_exact_centripetal_reaction() {
        let res = run_rotating_rod_dae(DaeSolverCandidate::DirectKktIndex2Projection);
        assert!(
            res.force_rel_error_pct < 1e-7,
            "Expected exact centripetal force (< 1e-7%), got {}%",
            res.force_rel_error_pct
        );
        assert!(
            res.omega_drift_rad_s < 1e-11,
            "Expected zero angular velocity drift, got {}",
            res.omega_drift_rad_s
        );
        assert!(
            res.pivot_drift_m < 1e-13,
            "Expected zero pivot drift, got {}",
            res.pivot_drift_m
        );
    }

    #[test]
    fn test_direct_kkt_slider_crank_closed_loop_accuracy() {
        let res = run_slider_crank_dae(DaeSolverCandidate::DirectKktIndex2Projection);
        assert!(
            res.max_slider_pos_error_m < 1e-9,
            "Expected slider position error < 1e-9 m, got {} m",
            res.max_slider_pos_error_m
        );
        assert!(
            res.max_closure_residual < 1e-11,
            "Expected loop closure residual < 1e-11, got {}",
            res.max_closure_residual
        );
    }

    #[test]
    fn test_direct_kkt_gear_pair_10_revolutions_and_torque_transfer() {
        let res = run_gear_pair_10_revs_dae(DaeSolverCandidate::DirectKktIndex2Projection);
        assert!(
            (res.pinion_angle_rad - 20.0 * std::f64::consts::PI).abs() < 1e-9,
            "Expected 10 revolutions (20*pi rad), got {}",
            res.pinion_angle_rad
        );
        assert!(
            res.phase_error_rad < 1e-11,
            "Expected zero phase drift over 10 revolutions, got {} rad",
            res.phase_error_rad
        );
        assert!(
            res.velocity_ratio_error_pct < 1e-9,
            "Expected exact velocity ratio, got {}% error",
            res.velocity_ratio_error_pct
        );
        assert!(
            res.torque_error_pct < 1e-7,
            "Expected exact dynamic gear torque transmission (2.0 N*m), got {} N*m ({}% error)",
            res.transmitted_motor_torque_nm,
            res.torque_error_pct
        );
    }
}
