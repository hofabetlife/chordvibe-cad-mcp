//! Physics simulation world managing `f64` rigid bodies, colliders, joints, and the multi-body constraint solver.

use std::collections::HashMap;
use std::time::Instant;

use mcp_cad_core::csg::csg_intersection;
use mcp_cad_core::topology::{BoundingBox, TriangleMesh, Vertex};
use mcp_cad_core::Part;
use nalgebra::{DMatrix, Matrix3, Quaternion, UnitQuaternion, Vector3};

use crate::error::PhysicsError;
use crate::joint::{
    CadJointType, JointHandle, JointLimitStatus, MechanismAnalysisReport, MotorConfig,
    is_zero_range_limits, local_orthonormal_basis, perpendicular_axes, quat_to_matrix,
};
use crate::rigid_body::{
    BodyHandle, BodyRegistry, ColliderHandle, ColliderRegistry, ColliderType,
    RigidBody as NativeRigidBody, create_collider, create_rigid_body,
};
use crate::solver::{MultiBodySolver, SolvedJointRecord, create_solved_joint_record};
use crate::telemetry::{
    BodyState, BodyTelemetry, CollisionRecord, InterferenceReport, JointLoadCase,
    JointReactionForce, JointSweepReport, LoadCaseSelector, PairInterferenceResult,
    PartInertialLoad, PartLoadCase, SimulationSummary, SweepStepResult,
};

/// Internal record of a kinematic joint registered in the physics world.
#[derive(Debug, Clone)]
pub struct CadJointRecord {
    /// Unique identifier of the joint.
    pub id: String,
    /// First connected part identifier.
    pub part_a: String,
    /// Second connected part identifier.
    pub part_b: String,
    /// CAD joint configuration type and anchors.
    pub joint_type: CadJointType,
    /// Optional motor configuration for actuated joints.
    pub motor: Option<MotorConfig>,
    /// Native `f64` joint handle.
    pub handle: JointHandle,
}

/// The main physics simulation world managing native `f64` [`BodyRegistry`], [`ColliderRegistry`],
/// and [`MultiBodySolver`].
#[derive(Debug, Clone)]
pub struct PhysicsWorld {
    gravity_f64: [f64; 3],

    /// Native `f64` rigid body registry (`P4-T2`).
    bodies: BodyRegistry,
    /// Native `f64` collider registry (`P4-T2`).
    colliders: ColliderRegistry,
    /// Native `f64` multi-body constraint and dynamics solver (`P4-T3`).
    solver: MultiBodySolver,

    // Mappings between CAD part IDs, joints, and native f64 handles
    part_to_body: HashMap<String, BodyHandle>,
    body_to_part: HashMap<BodyHandle, String>,
    collider_to_part: HashMap<ColliderHandle, String>,
    part_to_colliders: HashMap<String, Vec<ColliderHandle>>,

    part_meshes: HashMap<String, TriangleMesh>,
    joints: HashMap<String, CadJointRecord>,
    solved_joints: Vec<SolvedJointRecord>,
    next_joint_id: usize,
    step_counter: usize,
    last_step_succeeded: bool,

    // Current elapsed simulation time in seconds
    current_time_s: f64,
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsWorld {
    /// Creates a new physics world with standard Earth gravity $g = [0, -9.81, 0]$ m/s².
    #[must_use]
    pub fn new() -> Self {
        Self::with_gravity([0.0, -9.81, 0.0])
    }

    /// Creates a new physics world with custom gravity acceleration vector `[gx, gy, gz]`.
    #[must_use]
    pub fn with_gravity(gravity: [f64; 3]) -> Self {
        let mut bodies = BodyRegistry::new();

        // Insert a static anchor rigid body for "ground" and "world" references
        let ground_body = NativeRigidBody::new_fixed(
            "ground",
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        let ground_handle = bodies.insert(ground_body);

        let mut part_to_body = HashMap::new();
        let mut body_to_part = HashMap::new();

        part_to_body.insert("ground".to_string(), ground_handle);
        part_to_body.insert("world".to_string(), ground_handle);
        body_to_part.insert(ground_handle, "ground".to_string());

        Self {
            gravity_f64: gravity,
            bodies,
            colliders: ColliderRegistry::new(),
            solver: MultiBodySolver::new(),
            part_to_body,
            body_to_part,
            collider_to_part: HashMap::new(),
            part_to_colliders: HashMap::new(),
            part_meshes: HashMap::new(),
            joints: HashMap::new(),
            solved_joints: Vec::new(),
            next_joint_id: 0,
            step_counter: 0,
            last_step_succeeded: true,
            current_time_s: 0.0,
        }
    }

    /// Returns an immutable reference to the native `f64` [`BodyRegistry`].
    #[must_use]
    pub fn body_registry(&self) -> &BodyRegistry {
        &self.bodies
    }

    /// Returns an immutable reference to the native `f64` [`ColliderRegistry`].
    #[must_use]
    pub fn collider_registry(&self) -> &ColliderRegistry {
        &self.colliders
    }

    /// Sets the gravity acceleration vector `[gx, gy, gz]`.
    pub fn set_gravity(&mut self, gravity: [f64; 3]) {
        self.gravity_f64 = gravity;
    }

    /// Gets the current gravity acceleration vector `[gx, gy, gz]`.
    #[must_use]
    pub fn gravity(&self) -> [f64; 3] {
        self.gravity_f64
    }

    /// Current accumulated simulation time in seconds.
    #[must_use]
    pub fn current_time(&self) -> f64 {
        self.current_time_s
    }

    /// Resets accumulated simulation time to zero.
    pub fn reset_time(&mut self) {
        self.current_time_s = 0.0;
        self.step_counter = 0;
        self.last_step_succeeded = true;
        self.solver.reset();
    }

    /// Adds a CAD [`Part`] into the simulation world.
    ///
    /// For dynamic bodies, defaults to `ColliderType::ConvexHull`.
    /// For static/fixed bodies, defaults to `ColliderType::TriMesh`.
    pub fn add_part(&mut self, part: &Part, is_dynamic: bool) -> Result<BodyHandle, PhysicsError> {
        let collider_type = if is_dynamic {
            ColliderType::ConvexHull
        } else {
            ColliderType::TriMesh
        };
        self.add_part_with_options(part, is_dynamic, collider_type, None, None)
    }

    /// Adds a CAD [`Part`] with explicit collider approximation type and initial transform.
    pub fn add_part_with_options(
        &mut self,
        part: &Part,
        is_dynamic: bool,
        collider_type: ColliderType,
        initial_position: Option<[f64; 3]>,
        initial_rotation: Option<[f64; 4]>,
    ) -> Result<BodyHandle, PhysicsError> {
        if self.part_to_body.contains_key(&part.id)
            || part.id.eq_ignore_ascii_case("ground")
            || part.id.eq_ignore_ascii_case("world")
        {
            return Err(PhysicsError::PartAlreadyExists(part.id.clone()));
        }

        let f64_rb = create_rigid_body(part, is_dynamic, initial_position, initial_rotation);
        let f64_col = create_collider(part, collider_type)?;

        let rb_handle = self.bodies.insert(f64_rb);
        let col_handle = self
            .colliders
            .insert_with_parent(f64_col, rb_handle, &mut self.bodies);

        self.part_to_body.insert(part.id.clone(), rb_handle);
        self.body_to_part.insert(rb_handle, part.id.clone());
        self.collider_to_part.insert(col_handle, part.id.clone());
        self.part_to_colliders
            .entry(part.id.clone())
            .or_default()
            .push(col_handle);

        self.part_meshes.insert(part.id.clone(), part.mesh.clone());

        Ok(rb_handle)
    }

    /// Updates the collider and mass properties of a CAD [`Part`] in the physics world
    /// while preserving its [`BodyHandle`] and all attached joints.
    ///
    /// If the part is not yet registered in the physics world, this delegates to
    /// [`Self::add_part`] using `!part.is_fixed` as `is_dynamic`.
    pub fn update_part_collider(&mut self, part: &Part) -> Result<BodyHandle, PhysicsError> {
        if part.id.eq_ignore_ascii_case("ground") || part.id.eq_ignore_ascii_case("world") {
            return Err(PhysicsError::InvalidJointConfiguration(format!(
                "Cannot modify geometry of reserved static anchor '{}'",
                part.id
            )));
        }
        let is_dynamic = !part.is_fixed;
        let Some(&rb_handle) = self.part_to_body.get(&part.id) else {
            return self.add_part(part, is_dynamic);
        };

        let collider_type = if is_dynamic {
            ColliderType::ConvexHull
        } else {
            ColliderType::TriMesh
        };
        let new_col = create_collider(part, collider_type)?;

        // Remove old native colliders for this part
        if let Some(old_colliders) = self.part_to_colliders.remove(&part.id) {
            for col_h in old_colliders {
                self.collider_to_part.remove(&col_h);
                self.colliders.remove(col_h, &mut self.bodies);
            }
        }

        // Update native f64 RigidBody in place while preserving its BodyHandle and pose
        if let Some(rb) = self.bodies.get_mut(rb_handle) {
            rb.update_mass_properties_from_part(part, is_dynamic);
        }

        let col_handle = self
            .colliders
            .insert_with_parent(new_col, rb_handle, &mut self.bodies);
        self.collider_to_part.insert(col_handle, part.id.clone());
        self.part_to_colliders
            .entry(part.id.clone())
            .or_default()
            .push(col_handle);

        self.part_meshes.insert(part.id.clone(), part.mesh.clone());

        // Wake up any bodies coupled to this part via joints
        let coupled_body_handles: Vec<BodyHandle> = self
            .joints
            .values()
            .filter(|rec| rec.part_a == part.id || rec.part_b == part.id)
            .flat_map(|rec| {
                [
                    self.part_to_body.get(&rec.part_a).copied(),
                    self.part_to_body.get(&rec.part_b).copied(),
                ]
            })
            .flatten()
            .collect();
        for h in coupled_body_handles {
            if let Some(rb) = self.bodies.get_mut(h) {
                rb.wake_up();
            }
        }

        Ok(rb_handle)
    }

    /// Removes a CAD part, its colliders, and all attached joints from the world.
    pub fn remove_part(&mut self, part_id: &str) -> Result<(), PhysicsError> {
        if part_id.eq_ignore_ascii_case("ground") || part_id.eq_ignore_ascii_case("world") {
            return Err(PhysicsError::InvalidJointConfiguration(format!(
                "Cannot remove reserved static anchor '{part_id}' from physics world"
            )));
        }

        let rb_handle = self
            .part_to_body
            .remove(part_id)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))?;

        self.body_to_part.remove(&rb_handle);
        self.part_meshes.remove(part_id);

        if let Some(col_handles) = self.part_to_colliders.remove(part_id) {
            for col_h in col_handles {
                self.collider_to_part.remove(&col_h);
                self.colliders.remove(col_h, &mut self.bodies);
            }
        }
        self.bodies.remove(rb_handle);

        // Clean up attached joints in self.joints and self.solved_joints
        let attached_joint_ids: Vec<String> = self
            .joints
            .iter()
            .filter(|(_, rec)| rec.part_a == part_id || rec.part_b == part_id)
            .map(|(id, _)| id.clone())
            .collect();

        for j_id in attached_joint_ids {
            self.joints.remove(&j_id);
        }
        self.solved_joints
            .retain(|rec| rec.part_a != part_id && rec.part_b != part_id);

        Ok(())
    }

    /// Adds a kinematic/dynamic joint between two parts in `f64`.
    pub fn add_joint(
        &mut self,
        joint_id: impl Into<String>,
        part_a: &str,
        part_b: &str,
        joint_type: CadJointType,
        motor: Option<MotorConfig>,
    ) -> Result<JointHandle, PhysicsError> {
        let j_id = joint_id.into();
        if self.joints.contains_key(&j_id) {
            return Err(PhysicsError::JointAlreadyExists(j_id));
        }

        let norm_part = |id: &str| -> String {
            if id.eq_ignore_ascii_case("ground") {
                "ground".to_string()
            } else if id.eq_ignore_ascii_case("world") {
                "world".to_string()
            } else {
                id.to_string()
            }
        };
        let norm_a = norm_part(part_a);
        let norm_b = norm_part(part_b);

        let handle_a = *self
            .part_to_body
            .get(&norm_a)
            .ok_or_else(|| PhysicsError::PartNotFound(part_a.to_string()))?;

        let handle_b = *self
            .part_to_body
            .get(&norm_b)
            .ok_or_else(|| PhysicsError::PartNotFound(part_b.to_string()))?;

        let joint_data = joint_type.build_generic_joint(motor)?;
        let joint_handle = JointHandle(self.next_joint_id);
        self.next_joint_id += 1;

        let solved_rec = create_solved_joint_record(
            j_id.clone(),
            joint_handle,
            norm_a.clone(),
            norm_b.clone(),
            handle_a,
            handle_b,
            joint_type.clone(),
            motor,
            joint_data,
            &self.bodies,
        );
        self.solved_joints.push(solved_rec);

        self.joints.insert(
            j_id.clone(),
            CadJointRecord {
                id: j_id,
                part_a: norm_a,
                part_b: norm_b,
                joint_type,
                motor,
                handle: joint_handle,
            },
        );

        Ok(joint_handle)
    }

    /// Returns a reference to all registered CAD joints.
    #[must_use]
    pub fn joints(&self) -> &HashMap<String, CadJointRecord> {
        &self.joints
    }

    /// Updates motor actuation on an existing joint transactionally.
    pub fn set_joint_motor(
        &mut self,
        joint_id: &str,
        motor: MotorConfig,
    ) -> Result<(), PhysicsError> {
        let joint_type = {
            let record = self
                .joints
                .get(joint_id)
                .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;
            record.joint_type.clone()
        };

        let s_idx = self
            .solved_joints
            .iter()
            .position(|j| j.id == joint_id)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        joint_type.apply_motor(&mut self.solved_joints[s_idx].data, motor)?;

        let (body_a, body_b) = (
            self.solved_joints[s_idx].body_a,
            self.solved_joints[s_idx].body_b,
        );
        let cur_coord = match (self.bodies.get(body_a), self.bodies.get(body_b)) {
            (Some(ba), Some(bb)) => joint_type
                .current_coordinate(
                    ba.position,
                    ba.rotation_matrix(),
                    bb.position,
                    bb.rotation_matrix(),
                )
                .unwrap_or(0.0),
            _ => 0.0,
        };

        self.solved_joints[s_idx].motor = Some(motor);
        self.solved_joints[s_idx].motor_accum_coord = cur_coord;
        self.solved_joints[s_idx].motor_saturated = false;

        if let Some(record) = self.joints.get_mut(joint_id) {
            record.motor = Some(motor);
        }

        for bh in [body_a, body_b] {
            if let Some(rb) = self.bodies.get_mut(bh) {
                rb.wake_up();
            }
        }
        Ok(())
    }

    /// Disables motor actuation on an existing joint.
    pub fn disable_joint_motor(&mut self, joint_id: &str) -> Result<(), PhysicsError> {
        let joint_type = {
            let record = self
                .joints
                .get(joint_id)
                .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;
            record.joint_type.clone()
        };

        let s_idx = self
            .solved_joints
            .iter()
            .position(|j| j.id == joint_id)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        joint_type.disable_motor(&mut self.solved_joints[s_idx].data)?;
        self.solved_joints[s_idx].motor = None;
        self.solved_joints[s_idx].motor_saturated = false;

        let (body_a, body_b) = (
            self.solved_joints[s_idx].body_a,
            self.solved_joints[s_idx].body_b,
        );

        if let Some(record) = self.joints.get_mut(joint_id) {
            record.motor = None;
        }

        for bh in [body_a, body_b] {
            if let Some(rb) = self.bodies.get_mut(bh) {
                rb.wake_up();
            }
        }
        Ok(())
    }

    /// Removes a joint by identifier.
    pub fn remove_joint(&mut self, joint_id: &str) -> Result<(), PhysicsError> {
        let _record = self
            .joints
            .remove(joint_id)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        self.solved_joints.retain(|j| j.id != joint_id);
        Ok(())
    }

    /// Executes a single discrete `f64` physics integration step of size `dt` seconds.
    ///
    /// Returns any collision events that started or stopped during this step.
    pub fn step(&mut self, dt: f64) -> Vec<CollisionRecord> {
        let step_dt = if dt > 0.0 && dt.is_finite() {
            dt
        } else {
            1.0 / 60.0
        };
        let next_time = self.current_time_s + step_dt;
        let (ok, records) = self.solver.step(
            &mut self.bodies,
            &mut self.colliders,
            &mut self.solved_joints,
            self.gravity_f64,
            step_dt,
            self.step_counter,
            next_time,
        );
        self.last_step_succeeded = ok;
        self.step_counter += 1;
        self.current_time_s = next_time;
        records
    }

    /// Records an instantaneous telemetry snapshot for all bodies and joints.
    fn record_telemetry_snapshot(
        &self,
        summary: &mut SimulationSummary,
        time_s: f64,
        _dt: f64,
    ) {
        // Bodies (read directly from native f64 BodyRegistry)
        for (part_id, &rb_handle) in &self.part_to_body {
            if part_id == "ground" || part_id == "world" {
                continue;
            }
            if let Some(rb) = self.bodies.get(rb_handle) {
                let pos = rb.position;
                let q = rb.orientation.quaternion();
                let lv = rb.linear_velocity;
                let av = rb.angular_velocity;

                let state = BodyState {
                    time_s,
                    position: [pos.x, pos.y, pos.z],
                    orientation_quat: [q.coords.x, q.coords.y, q.coords.z, q.coords.w],
                    linear_velocity: [lv.x, lv.y, lv.z],
                    angular_velocity: [av.x, av.y, av.z],
                };

                if let Some(traj) = summary.trajectories.get_mut(part_id) {
                    traj.push_sample(state);
                }
            }
        }

        // Joints (read exact f64 Lagrange multiplier reactions from SolvedJointRecord)
        for s_joint in &self.solved_joints {
            let force = s_joint.last_force_on_a;
            let torque = s_joint.last_torque_on_a;
            let force_mag = (force[0].powi(2) + force[1].powi(2) + force[2].powi(2)).sqrt();
            let torque_mag = (torque[0].powi(2) + torque[1].powi(2) + torque[2].powi(2)).sqrt();

            let reaction = JointReactionForce {
                joint_id: s_joint.id.clone(),
                time_s,
                force,
                torque,
                force_magnitude: force_mag,
                torque_magnitude: torque_mag,
            };

            if let Some(list) = summary.joint_reactions.get_mut(&s_joint.id) {
                list.push(reaction);
            }

            if let Some(peak) = summary.peak_joint_forces.get_mut(&s_joint.id) {
                if force_mag > *peak {
                    *peak = force_mag;
                }
            }
        }
    }

    /// Simulates physics forward for `duration_s` with integration step `step_size_s`.
    ///
    /// Produces a comprehensive [`SimulationSummary`] containing trajectories, joint reactions,
    /// and collision events tailored for AI agent analysis.
    pub fn simulate(&mut self, duration_s: f64, step_size_s: f64) -> SimulationSummary {
        let dt = if step_size_s > 0.0 && step_size_s.is_finite() {
            step_size_s
        } else {
            1.0 / 60.0
        };
        let duration = duration_s.max(0.0);
        let steps = (duration / dt).round() as usize;

        let mut summary = SimulationSummary::new(duration, dt);

        // Pre-allocate telemetry logs for all parts
        for part_id in self.part_to_body.keys() {
            if part_id == "ground" || part_id == "world" {
                continue;
            }
            summary
                .trajectories
                .insert(part_id.clone(), BodyTelemetry::new(part_id));
        }

        // Pre-allocate joint reaction histories
        for joint_id in self.joints.keys() {
            summary.joint_reactions.insert(joint_id.clone(), Vec::new());
            summary.peak_joint_forces.insert(joint_id.clone(), 0.0);
        }

        // Record initial state at t = current_time_s
        self.record_telemetry_snapshot(&mut summary, self.current_time_s, dt);

        for _ in 0..steps {
            let collisions = self.step(dt);
            if !self.last_step_succeeded {
                summary.success = false;
            }
            summary.collisions.extend(collisions);
            self.record_telemetry_snapshot(&mut summary, self.current_time_s, dt);
        }

        summary.step_count = steps;
        summary
    }

    // --- Query and inspection methods ---

    fn get_body(&self, part_id: &str) -> Result<&NativeRigidBody, PhysicsError> {
        let norm_id = if part_id.eq_ignore_ascii_case("ground") {
            "ground"
        } else if part_id.eq_ignore_ascii_case("world") {
            "world"
        } else {
            part_id
        };
        let handle = self
            .part_to_body
            .get(norm_id)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))?;
        self.bodies
            .get(*handle)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))
    }

    fn get_body_mut(&mut self, part_id: &str) -> Result<&mut NativeRigidBody, PhysicsError> {
        let norm_id = if part_id.eq_ignore_ascii_case("ground") {
            "ground"
        } else if part_id.eq_ignore_ascii_case("world") {
            "world"
        } else {
            part_id
        };
        let handle = self
            .part_to_body
            .get(norm_id)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))?;
        self.bodies
            .get_mut(*handle)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))
    }

    /// Returns `true` if a part is registered in the physics world.
    #[must_use]
    pub fn has_part(&self, part_id: &str) -> bool {
        let norm_id = if part_id.eq_ignore_ascii_case("ground") {
            "ground"
        } else if part_id.eq_ignore_ascii_case("world") {
            "world"
        } else {
            part_id
        };
        self.part_to_body.contains_key(norm_id)
    }

    /// Returns `true` if a registered part's rigid body is dynamic (movable), or `false` if fixed.
    pub fn is_part_dynamic(&self, part_id: &str) -> Result<bool, PhysicsError> {
        let rb = self.get_body(part_id)?;
        Ok(rb.is_dynamic())
    }

    /// Returns the computed mass (in kg, `f64`) of a registered part's rigid body.
    pub fn part_mass(&self, part_id: &str) -> Result<f64, PhysicsError> {
        let rb = self.get_body(part_id)?;
        Ok(rb.mass())
    }

    /// Returns the local-frame center of mass `[cx, cy, cz]` (in meters, `f64`) of a registered part.
    pub fn part_local_com(&self, part_id: &str) -> Result<[f64; 3], PhysicsError> {
        let rb = self.get_body(part_id)?;
        Ok([rb.local_com.x, rb.local_com.y, rb.local_com.z])
    }

    /// Returns the exact $3 \times 3$ local-frame inertia tensor $I_{\text{loc}}$ ($\text{kg}\cdot\text{m}^2$, `f64`)
    /// about the part's center of mass.
    pub fn part_inertia_tensor(&self, part_id: &str) -> Result<Matrix3<f64>, PhysicsError> {
        let rb = self.get_body(part_id)?;
        Ok(rb.local_inertia)
    }

    /// Returns the world-space axis-aligned bounding box `([min_x, min_y, min_z], [max_x, max_y, max_z])`
    /// in meters of all colliders attached to a registered part.
    pub fn part_collider_aabb(&self, part_id: &str) -> Result<([f64; 3], [f64; 3]), PhysicsError> {
        let _ = self.get_body(part_id)?;
        let col_handles = self
            .part_to_colliders
            .get(part_id)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))?;
        if col_handles.is_empty() {
            return Err(PhysicsError::InvalidGeometry(
                part_id.to_string(),
                "Part has no colliders".to_string(),
            ));
        }
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for &col_h in col_handles {
            if let Some(col) = self.colliders.get(col_h) {
                let aabb = col.compute_aabb();
                min[0] = min[0].min(aabb.min[0]);
                min[1] = min[1].min(aabb.min[1]);
                min[2] = min[2].min(aabb.min[2]);
                max[0] = max[0].max(aabb.max[0]);
                max[1] = max[1].max(aabb.max[1]);
                max[2] = max[2].max(aabb.max[2]);
            }
        }
        Ok((min, max))
    }

    /// Retrieves current 3D position `[x, y, z]` of a part.
    pub fn part_position(&self, part_id: &str) -> Result<[f64; 3], PhysicsError> {
        let rb = self.get_body(part_id)?;
        Ok([rb.position.x, rb.position.y, rb.position.z])
    }

    /// Retrieves current orientation quaternion `[x, y, z, w]` of a part.
    pub fn part_orientation(&self, part_id: &str) -> Result<[f64; 4], PhysicsError> {
        let rb = self.get_body(part_id)?;
        let q = rb.orientation.quaternion();
        Ok([q.coords.x, q.coords.y, q.coords.z, q.coords.w])
    }

    /// Retrieves current linear velocity `[vx, vy, vz]` of a part.
    pub fn part_velocity(&self, part_id: &str) -> Result<[f64; 3], PhysicsError> {
        let rb = self.get_body(part_id)?;
        Ok([
            rb.linear_velocity.x,
            rb.linear_velocity.y,
            rb.linear_velocity.z,
        ])
    }

    /// Retrieves current linear velocity `[vx, vy, vz]` of a part (alias for [`Self::part_velocity`]).
    pub fn part_linear_velocity(&self, part_id: &str) -> Result<[f64; 3], PhysicsError> {
        self.part_velocity(part_id)
    }

    /// Retrieves current angular velocity `[wx, wy, wz]` of a part.
    pub fn part_angular_velocity(&self, part_id: &str) -> Result<[f64; 3], PhysicsError> {
        let rb = self.get_body(part_id)?;
        Ok([
            rb.angular_velocity.x,
            rb.angular_velocity.y,
            rb.angular_velocity.z,
        ])
    }

    /// Sets linear velocity `[vx, vy, vz]` on a part.
    pub fn set_part_velocity(
        &mut self,
        part_id: &str,
        linvel: [f64; 3],
    ) -> Result<(), PhysicsError> {
        let rb = self.get_body_mut(part_id)?;
        rb.set_linvel(Vector3::new(linvel[0], linvel[1], linvel[2]));
        Ok(())
    }

    /// Sets angular velocity `[wx, wy, wz]` on a part.
    pub fn set_part_angular_velocity(
        &mut self,
        part_id: &str,
        angvel: [f64; 3],
    ) -> Result<(), PhysicsError> {
        let rb = self.get_body_mut(part_id)?;
        rb.set_angvel(Vector3::new(angvel[0], angvel[1], angvel[2]));
        Ok(())
    }

    /// Sets the world-space position `[x, y, z]` (m) and orientation quaternion `[qx, qy, qz, qw]`
    /// of a registered part and immediately synchronizes its attached colliders.
    pub fn set_part_pose(
        &mut self,
        part_id: &str,
        position: [f64; 3],
        orientation_quat: [f64; 4],
    ) -> Result<(), PhysicsError> {
        let pos_f64 = Vector3::new(position[0], position[1], position[2]);
        let quat_f64 = Quaternion::new(
            orientation_quat[3],
            orientation_quat[0],
            orientation_quat[1],
            orientation_quat[2],
        );
        let unit_quat_f64 = if quat_f64.norm() > 1e-12 {
            UnitQuaternion::from_quaternion(quat_f64)
        } else {
            UnitQuaternion::identity()
        };

        let rb = self.get_body_mut(part_id)?;
        rb.set_pose(pos_f64, unit_quat_f64);

        if let Some(col_handles) = self.part_to_colliders.get(part_id).cloned() {
            for col_h in col_handles {
                if let Some(col) = self.colliders.get_mut(col_h) {
                    col.update_world_pose(pos_f64, unit_quat_f64);
                }
            }
        }
        Ok(())
    }

    /// Applies a continuous external force vector `[fx, fy, fz]` in Newtons to a part.
    pub fn apply_force(&mut self, part_id: &str, force: [f64; 3]) -> Result<(), PhysicsError> {
        let rb = self.get_body_mut(part_id)?;
        rb.add_force(Vector3::new(force[0], force[1], force[2]));
        Ok(())
    }

    /// Applies an instantaneous impulse vector `[ix, iy, iz]` in N*s to a part.
    pub fn apply_impulse(&mut self, part_id: &str, impulse: [f64; 3]) -> Result<(), PhysicsError> {
        let rb = self.get_body_mut(part_id)?;
        rb.apply_impulse(Vector3::new(impulse[0], impulse[1], impulse[2]));
        Ok(())
    }

    /// Computes the reaction force on a joint over the last integration step `dt`.
    pub fn joint_reaction_force(
        &self,
        joint_id: &str,
        _dt: f64,
    ) -> Result<JointReactionForce, PhysicsError> {
        let s_joint = self
            .solved_joints
            .iter()
            .find(|j| j.id == joint_id)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        let force = s_joint.last_force_on_a;
        let torque = s_joint.last_torque_on_a;
        let force_magnitude = (force[0].powi(2) + force[1].powi(2) + force[2].powi(2)).sqrt();
        let torque_magnitude = (torque[0].powi(2) + torque[1].powi(2) + torque[2].powi(2)).sqrt();

        Ok(JointReactionForce {
            joint_id: joint_id.to_string(),
            time_s: self.current_time_s,
            force,
            torque,
            force_magnitude,
            torque_magnitude,
        })
    }

    /// Returns all joint records connected to `part_id` (`part_a == part_id` or `part_b == part_id`),
    /// sorted deterministically by `joint.id`.
    #[must_use]
    pub fn joints_for_part(&self, part_id: &str) -> Vec<&CadJointRecord> {
        let norm_id = if part_id.eq_ignore_ascii_case("ground") {
            "ground"
        } else if part_id.eq_ignore_ascii_case("world") {
            "world"
        } else {
            part_id
        };
        let mut list: Vec<&CadJointRecord> = self
            .joints
            .values()
            .filter(|j| j.part_a == norm_id || j.part_b == norm_id)
            .collect();
        list.sort_by(|a, b| a.id.cmp(&b.id));
        list
    }

    /// Extracts the complete dynamic/quasi-static load case for `part_id` from a completed
    /// [`SimulationSummary`] in the **local coordinate system of `part_id`** (`P3-T6`).
    pub fn extract_part_load_case(
        &self,
        summary: &SimulationSummary,
        part_id: &str,
        selector: LoadCaseSelector,
    ) -> Result<PartLoadCase, PhysicsError> {
        if let LoadCaseSelector::AtTime(t) = selector {
            if !t.is_finite() || t < 0.0 {
                return Err(PhysicsError::SimulationError(format!(
                    "Load case timestamp must be a finite non-negative number, got {t}"
                )));
            }
        }

        let rb = self.get_body(part_id)?;
        let is_fixed = !rb.is_dynamic();
        let mass_kg = if is_fixed { 0.0 } else { rb.mass() };
        let center_of_mass_local = [rb.local_com.x, rb.local_com.y, rb.local_com.z];
        let com_local_vec = rb.local_com;

        // Exact f64 local inertia tensor I_local (3x3) in body local coordinates
        let local_inertia_mat = if is_fixed {
            nalgebra::Matrix3::<f64>::zeros()
        } else {
            rb.local_inertia
        };

        let connected_joints = self.joints_for_part(part_id);
        let norm_part_id = if part_id.eq_ignore_ascii_case("ground") {
            "ground"
        } else if part_id.eq_ignore_ascii_case("world") {
            "world"
        } else {
            part_id
        };

        // Determine sample timeline for this part (or synthesized fixed-ground samples)
        let synthesized_samples: Vec<BodyState>;
        let samples: &[BodyState] = if let Some(traj) = summary.trajectories.get(norm_part_id) {
            if traj.samples.is_empty() {
                return Err(PhysicsError::SimulationError(format!(
                    "No simulation telemetry samples recorded for part '{part_id}'"
                )));
            }
            &traj.samples
        } else {
            let n = summary
                .joint_reactions
                .values()
                .map(Vec::len)
                .max()
                .unwrap_or(summary.step_count + 1)
                .max(1);
            let pos = self.part_position(part_id).unwrap_or([0.0; 3]);
            let quat = self.part_orientation(part_id).unwrap_or([0.0, 0.0, 0.0, 1.0]);
            synthesized_samples = (0..n)
                .map(|i| BodyState {
                    time_s: (i as f64) * summary.step_size_s,
                    position: pos,
                    orientation_quat: quat,
                    linear_velocity: [0.0; 3],
                    angular_velocity: [0.0; 3],
                })
                .collect();
            &synthesized_samples
        };

        let num_samples = samples.len();
        let start_idx = usize::from(num_samples > 1);

        let quat_from_arr = |q: [f64; 4]| -> nalgebra::UnitQuaternion<f64> {
            nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                q[3], q[0], q[1], q[2],
            ))
        };

        let score_at_step = |idx: usize| -> (f64, f64, f64, f64) {
            let q_k = quat_from_arr(samples[idx].orientation_quat);
            let rot_inv = q_k.inverse();
            let mut sum_f_mag = 0.0_f64;
            let mut sum_t_mag = 0.0_f64;
            let mut net_f_local = Vector3::<f64>::zeros();
            let mut net_t_com_local = Vector3::<f64>::zeros();

            for jrec in &connected_joints {
                if let Some(reactions) = summary.joint_reactions.get(&jrec.id) {
                    if let Some(rf) = reactions.get(idx) {
                        let (sign, anchor_arr) = if jrec.part_a == norm_part_id {
                            (1.0_f64, jrec.joint_type.anchor_a())
                        } else {
                            (-1.0_f64, jrec.joint_type.anchor_b())
                        };
                        let fw = Vector3::new(
                            sign * rf.force[0],
                            sign * rf.force[1],
                            sign * rf.force[2],
                        );
                        let tw = Vector3::new(
                            sign * rf.torque[0],
                            sign * rf.torque[1],
                            sign * rf.torque[2],
                        );
                        let fl = rot_inv * fw;
                        let tl = rot_inv * tw;
                        let anchor_vec = Vector3::new(anchor_arr[0], anchor_arr[1], anchor_arr[2]);
                        let lever = anchor_vec - com_local_vec;
                        sum_f_mag += fl.norm();
                        sum_t_mag += tl.norm();
                        net_f_local += fl;
                        net_t_com_local += tl + lever.cross(&fl);
                    }
                }
            }
            (
                sum_f_mag,
                net_f_local.norm(),
                sum_t_mag,
                net_t_com_local.norm(),
            )
        };

        let selected_idx = match selector {
            LoadCaseSelector::PeakForce => {
                let mut best_idx = start_idx;
                let mut best_score = (-1.0_f64, -1.0_f64);
                for idx in start_idx..num_samples {
                    let (sum_f, net_f, _, _) = score_at_step(idx);
                    let score = (sum_f.max(net_f), net_f);
                    if score.0 > best_score.0 + 1e-12
                        || ((score.0 - best_score.0).abs() <= 1e-12 && score.1 > best_score.1 + 1e-12)
                    {
                        best_score = score;
                        best_idx = idx;
                    }
                }
                best_idx
            }
            LoadCaseSelector::PeakTorque => {
                let mut best_idx = start_idx;
                let mut best_score = (-1.0_f64, -1.0_f64);
                for idx in start_idx..num_samples {
                    let (_, _, sum_t, net_t_com) = score_at_step(idx);
                    let primary = sum_t.max(net_t_com);
                    let secondary = sum_t + net_t_com;
                    if primary > best_score.0 + 1e-12
                        || ((primary - best_score.0).abs() <= 1e-12
                            && secondary > best_score.1 + 1e-12)
                    {
                        best_score = (primary, secondary);
                        best_idx = idx;
                    }
                }
                best_idx
            }
            LoadCaseSelector::AtTime(target_t) => {
                let mut best_idx = start_idx;
                let mut best_diff = f64::INFINITY;
                for (idx, sample) in samples.iter().enumerate().take(num_samples).skip(start_idx) {
                    let diff = (sample.time_s - target_t).abs();
                    if diff < best_diff {
                        best_diff = diff;
                        best_idx = idx;
                    }
                }
                best_idx
            }
        };

        let state_k = &samples[selected_idx];
        let q_k = quat_from_arr(state_k.orientation_quat);
        let omega_k_world = Vector3::new(
            state_k.angular_velocity[0],
            state_k.angular_velocity[1],
            state_k.angular_velocity[2],
        );
        // Reactions in `MultiBodySolver` (`DirectKktIndex2`) are evaluated at the exact
        // end-of-step manifold pose `q_k` (`state_k.orientation_quat`).
        let q_solve = q_k;
        let rot_world_to_local = q_solve.inverse();

        let mut joint_cases: Vec<JointLoadCase> = Vec::with_capacity(connected_joints.len());
        let mut net_force_local = Vector3::<f64>::zeros();
        let mut net_couple_local = Vector3::<f64>::zeros();
        let mut net_torque_com_local = Vector3::<f64>::zeros();

        for jrec in &connected_joints {
            let (sign, role, connected_part_id, anchor_local) = if jrec.part_a == norm_part_id {
                (
                    1.0_f64,
                    "part_a".to_string(),
                    jrec.part_b.clone(),
                    jrec.joint_type.anchor_a(),
                )
            } else {
                (
                    -1.0_f64,
                    "part_b".to_string(),
                    jrec.part_a.clone(),
                    jrec.joint_type.anchor_b(),
                )
            };

            let (rf_force, rf_torque) = summary
                .joint_reactions
                .get(&jrec.id)
                .and_then(|list| list.get(selected_idx))
                .map_or(([0.0; 3], [0.0; 3]), |rf| (rf.force, rf.torque));

            let force_world_vec = Vector3::new(
                sign * rf_force[0],
                sign * rf_force[1],
                sign * rf_force[2],
            );
            let torque_world_vec = Vector3::new(
                sign * rf_torque[0],
                sign * rf_torque[1],
                sign * rf_torque[2],
            );

            let force_local_vec = rot_world_to_local * force_world_vec;
            let torque_local_vec = rot_world_to_local * torque_world_vec;
            let force_mag = force_local_vec.norm();
            let torque_mag = torque_local_vec.norm();

            let anchor_vec = Vector3::new(anchor_local[0], anchor_local[1], anchor_local[2]);
            let lever_from_com = anchor_vec - com_local_vec;

            net_force_local += force_local_vec;
            net_couple_local += torque_local_vec;
            net_torque_com_local += torque_local_vec + lever_from_com.cross(&force_local_vec);

            joint_cases.push(JointLoadCase {
                joint_id: jrec.id.clone(),
                joint_type: jrec.joint_type.type_name().to_string(),
                connected_part_id,
                role,
                anchor_local,
                force: [force_local_vec.x, force_local_vec.y, force_local_vec.z],
                torque: [torque_local_vec.x, torque_local_vec.y, torque_local_vec.z],
                force_world: [force_world_vec.x, force_world_vec.y, force_world_vec.z],
                torque_world: [torque_world_vec.x, torque_world_vec.y, torque_world_vec.z],
                force_magnitude: force_mag,
                torque_magnitude: torque_mag,
            });
        }

        // Compute linear and angular velocities and accelerations in the part's local frame
        let v_k_world = Vector3::new(
            state_k.linear_velocity[0],
            state_k.linear_velocity[1],
            state_k.linear_velocity[2],
        );
        let v_k_local = q_k.inverse() * v_k_world;
        let omega_k_local = q_k.inverse() * omega_k_world;

        let (a_com_local, alpha_local) = if is_fixed || num_samples <= 1 {
            (Vector3::zeros(), Vector3::zeros())
        } else {
            let (prev_idx, curr_idx) = if selected_idx > 0 {
                (selected_idx - 1, selected_idx)
            } else {
                (0, 1)
            };
            let s_prev = &samples[prev_idx];
            let s_curr = &samples[curr_idx];
            let dt_step = (s_curr.time_s - s_prev.time_s).max(1e-9);
            let q_prev = quat_from_arr(s_prev.orientation_quat);
            let q_curr = quat_from_arr(s_curr.orientation_quat);

            let v_prev_local = q_prev.inverse()
                * Vector3::new(
                    s_prev.linear_velocity[0],
                    s_prev.linear_velocity[1],
                    s_prev.linear_velocity[2],
                );
            let v_curr_local = q_curr.inverse()
                * Vector3::new(
                    s_curr.linear_velocity[0],
                    s_curr.linear_velocity[1],
                    s_curr.linear_velocity[2],
                );
            let omega_prev_local = q_prev.inverse()
                * Vector3::new(
                    s_prev.angular_velocity[0],
                    s_prev.angular_velocity[1],
                    s_prev.angular_velocity[2],
                );
            let omega_curr_local = q_curr.inverse()
                * Vector3::new(
                    s_curr.angular_velocity[0],
                    s_curr.angular_velocity[1],
                    s_curr.angular_velocity[2],
                );

            // Co-rotating transport theorem: a_local = dv_local/dt + omega_local x v_local
            let dv_local_dt = (v_curr_local - v_prev_local) / dt_step;
            let a_local = dv_local_dt + omega_curr_local.cross(&v_curr_local);
            let domega_local_dt = (omega_curr_local - omega_prev_local) / dt_step;
            (a_local, domega_local_dt)
        };

        let a_com_world = q_solve * a_com_local;
        let alpha_world = q_solve * alpha_local;

        let gravity_world = Vector3::new(
            self.gravity_f64[0],
            self.gravity_f64[1],
            self.gravity_f64[2],
        );
        let gravity_local = rot_world_to_local * gravity_world;

        // Dynamic equilibrium residuals in the part's local frame (for dynamic bodies):
        // Force equilibrium: sum(F_joints) + m * g - m * a_com = 0
        // Moment equilibrium about COM: sum(T_joint + (anchor - com) x F_joint) - (I * alpha + omega x (I * omega)) = 0
        let (res_force_vec, res_torque_vec) = if is_fixed {
            (Vector3::zeros(), Vector3::zeros())
        } else {
            let rf = net_force_local + mass_kg * gravity_local - mass_kg * a_com_local;
            let i_omega = local_inertia_mat * omega_k_local;
            let euler_inertial_torque =
                local_inertia_mat * alpha_local + omega_k_local.cross(&i_omega);
            let rt = net_torque_com_local - euler_inertial_torque;
            (rf, rt)
        };

        let res_force_norm = res_force_vec.norm();
        let res_torque_norm = res_torque_vec.norm();

        Ok(PartLoadCase {
            part_id: part_id.to_string(),
            select: selector.label(),
            time_s: state_k.time_s,
            step_index: selected_idx,
            position_world: state_k.position,
            orientation_quat: state_k.orientation_quat,
            joints: joint_cases,
            net_joint_force: [net_force_local.x, net_force_local.y, net_force_local.z],
            net_joint_torque_about_com: [
                net_torque_com_local.x,
                net_torque_com_local.y,
                net_torque_com_local.z,
            ],
            net_joint_couple: [net_couple_local.x, net_couple_local.y, net_couple_local.z],
            net_joint_force_magnitude: net_force_local.norm(),
            net_joint_torque_about_com_magnitude: net_torque_com_local.norm(),
            inertia: PartInertialLoad {
                mass_kg,
                is_fixed,
                center_of_mass_local,
                linear_acceleration: [a_com_local.x, a_com_local.y, a_com_local.z],
                angular_acceleration: [alpha_local.x, alpha_local.y, alpha_local.z],
                linear_acceleration_world: [a_com_world.x, a_com_world.y, a_com_world.z],
                angular_acceleration_world: [alpha_world.x, alpha_world.y, alpha_world.z],
                linear_velocity: [v_k_local.x, v_k_local.y, v_k_local.z],
                angular_velocity: [omega_k_local.x, omega_k_local.y, omega_k_local.z],
                gravity_local: [gravity_local.x, gravity_local.y, gravity_local.z],
            },
            equilibrium_residual_force: [res_force_vec.x, res_force_vec.y, res_force_vec.z],
            equilibrium_residual_force_norm: res_force_norm,
            equilibrium_residual_torque: [res_torque_vec.x, res_torque_vec.y, res_torque_vec.z],
            equilibrium_residual_torque_norm: res_torque_norm,
        })
    }

    /// Retrieves world position and 3x3 rotation matrix for a body or ground anchor.
    fn get_world_pose_vectors(&self, part_id: &str) -> (Vector3<f64>, Matrix3<f64>) {
        if part_id.eq_ignore_ascii_case("ground") || part_id.eq_ignore_ascii_case("world") {
            (Vector3::zeros(), Matrix3::identity())
        } else if let Ok(rb) = self.get_body(part_id) {
            let q = rb.orientation.quaternion();
            let rot_mat = quat_to_matrix([q.coords.x, q.coords.y, q.coords.z, q.coords.w]);
            (rb.position, rot_mat)
        } else {
            (Vector3::zeros(), Matrix3::identity())
        }
    }

    /// Performs kinematic degrees-of-freedom, mobility, and overconstraint analysis (`P3-T9`).
    ///
    /// Computes:
    /// 1. Spatial mobility using the Chebychev-Grübler-Kutzbach formula:
    ///    $$M = 6(N - 1) - \sum c_i$$
    /// 2. Numerical degrees of freedom via SVD rank decomposition of the linearized kinematic constraint Jacobian $J$:
    ///    $$\text{DOF} = 6 N_{\text{moving}} - \text{rank}(J)$$
    /// 3. Redundant constraints count: $\max(0, \text{DOF}_{\text{numeric}} - \text{DOF}_{\text{grubler}})$.
    /// 4. Graph connectivity: identification of grounded, floating, and unconnected bodies.
    pub fn analyze_mechanism(
        &self,
        part_ids: Option<&[String]>,
        rank_tolerance: Option<f64>,
    ) -> Result<MechanismAnalysisReport, PhysicsError> {
        self.analyze_mechanism_with_scene_parts(part_ids, rank_tolerance, None)
    }

    /// Variant of [`Self::analyze_mechanism`] that also accounts for CAD scene parts
    /// (`(part_id, is_fixed)`) not yet registered as Rapier bodies in `PhysicsWorld`.
    pub fn analyze_mechanism_with_scene_parts(
        &self,
        part_ids: Option<&[String]>,
        rank_tolerance: Option<f64>,
        scene_parts: Option<&[(String, bool)]>,
    ) -> Result<MechanismAnalysisReport, PhysicsError> {
        if let Some(tol) = rank_tolerance {
            if !tol.is_finite() || tol <= 0.0 {
                return Err(PhysicsError::InvalidJointConfiguration(format!(
                    "rank_tolerance must be a finite positive number (> 0), got {tol}"
                )));
            }
        }

        let norm_part = |id: &str| -> String {
            if id.eq_ignore_ascii_case("ground") {
                "ground".to_string()
            } else if id.eq_ignore_ascii_case("world") {
                "world".to_string()
            } else {
                id.to_string()
            }
        };

        let target_part_set: Option<std::collections::HashSet<String>> = part_ids.map(|ids| {
            ids.iter().map(|s| norm_part(s)).collect()
        });

        if let Some(ref set) = target_part_set {
            for pid in set {
                if pid == "ground" || pid == "world" {
                    continue;
                }
                let in_world = self.part_to_body.contains_key(pid);
                let in_scene = scene_parts
                    .map(|sp| sp.iter().any(|(id, _)| norm_part(id) == *pid))
                    .unwrap_or(false);
                if !in_world && !in_scene {
                    return Err(PhysicsError::PartNotFound(pid.clone()));
                }
            }
        }

        let mut grounded_set = std::collections::HashSet::new();
        let mut moving_set = std::collections::HashSet::new();

        grounded_set.insert("ground".to_string());
        grounded_set.insert("world".to_string());

        for (pid, &handle) in &self.part_to_body {
            let norm_id = norm_part(pid);
            if norm_id == "ground" || norm_id == "world" {
                continue;
            }
            if let Some(ref set) = target_part_set {
                if !set.contains(&norm_id) {
                    continue;
                }
            }
            if let Some(rb) = self.bodies.get(handle) {
                if rb.is_dynamic() {
                    moving_set.insert(norm_id);
                } else {
                    grounded_set.insert(norm_id);
                }
            }
        }

        if let Some(sp) = scene_parts {
            for (pid, is_fixed) in sp {
                let norm_id = norm_part(pid);
                if norm_id == "ground" || norm_id == "world" {
                    continue;
                }
                if let Some(ref set) = target_part_set {
                    if !set.contains(&norm_id) {
                        continue;
                    }
                }
                if !moving_set.contains(&norm_id) && !grounded_set.contains(&norm_id) {
                    if *is_fixed {
                        grounded_set.insert(norm_id);
                    } else {
                        moving_set.insert(norm_id);
                    }
                }
            }
        }

        let mut moving_parts: Vec<String> = moving_set.into_iter().collect();
        moving_parts.sort();

        let moving_index_map: HashMap<String, usize> = moving_parts
            .iter()
            .enumerate()
            .map(|(i, id)| (id.clone(), i))
            .collect();

        let mut active_joints = Vec::new();
        for joint in self.joints.values() {
            let na = norm_part(&joint.part_a);
            let nb = norm_part(&joint.part_b);

            if let Some(ref set) = target_part_set {
                let a_ok = set.contains(&na) || na == "ground" || na == "world";
                let b_ok = set.contains(&nb) || nb == "ground" || nb == "world";
                if !a_ok || !b_ok {
                    continue;
                }
            }
            active_joints.push(joint);
        }
        active_joints.sort_by_key(|j| j.id.clone());

        // Connectivity graph
        let mut adj: HashMap<String, Vec<String>> = HashMap::new();
        for j in &active_joints {
            let na = norm_part(&j.part_a);
            let nb = norm_part(&j.part_b);
            adj.entry(na.clone()).or_default().push(nb.clone());
            adj.entry(nb).or_default().push(na);
        }

        let mut unconnected_parts = Vec::new();
        for p in &moving_parts {
            if !adj.contains_key(p) || adj[p].is_empty() {
                unconnected_parts.push(p.clone());
            }
        }
        unconnected_parts.sort();

        let mut visited_from_ground = std::collections::HashSet::new();
        let mut queue = std::collections::VecDeque::new();
        for g in &grounded_set {
            visited_from_ground.insert(g.clone());
            queue.push_back(g.clone());
        }
        while let Some(curr) = queue.pop_front() {
            if let Some(neighbors) = adj.get(&curr) {
                for n in neighbors {
                    if visited_from_ground.insert(n.clone()) {
                        queue.push_back(n.clone());
                    }
                }
            }
        }

        let mut floating_parts = Vec::new();
        for p in &moving_parts {
            if !visited_from_ground.contains(p) {
                floating_parts.push(p.clone());
            }
        }
        floating_parts.sort();

        let mut reported_grounded = Vec::new();
        for g in &grounded_set {
            if g == "ground" || g == "world" {
                if active_joints.iter().any(|j| norm_part(&j.part_a) == *g || norm_part(&j.part_b) == *g) {
                    reported_grounded.push(g.clone());
                }
            } else {
                reported_grounded.push(g.clone());
            }
        }
        reported_grounded.sort();
        reported_grounded.dedup();

        let mut contributing_joints = Vec::new();
        for j in active_joints {
            let na = norm_part(&j.part_a);
            let nb = norm_part(&j.part_b);
            let is_a_moving = moving_index_map.contains_key(&na);
            let is_b_moving = moving_index_map.contains_key(&nb);
            if is_a_moving || is_b_moving {
                contributing_joints.push(j);
            }
        }

        let total_constraint_rows: usize = contributing_joints
            .iter()
            .map(|j| j.joint_type.effective_constraints_count())
            .sum();
        let num_moving = moving_parts.len();
        let total_cols = 6 * num_moving;

        let mut joints_at_limits = Vec::new();
        let mut limit_warnings = Vec::new();
        let mut joint_row_spans: Vec<(&CadJointRecord, usize, usize)> =
            Vec::with_capacity(contributing_joints.len());

        let (j_mat_opt, jacobian_rank, singular_values, effective_tol) =
            if total_constraint_rows == 0 || total_cols == 0 {
                for j in &contributing_joints {
                    let na = norm_part(&j.part_a);
                    let nb = norm_part(&j.part_b);
                    let (pos_a, rot_a) = self.get_world_pose_vectors(&na);
                    let (pos_b, rot_b) = self.get_world_pose_vectors(&nb);
                    if let Some((status, warn)) =
                        evaluate_joint_limit_status(j, pos_a, rot_a, pos_b, rot_b)
                    {
                        joints_at_limits.push(status);
                        limit_warnings.push(warn);
                    }
                }
                (None, 0, Vec::new(), rank_tolerance.unwrap_or(1e-6))
            } else {
                let mut j_mat = DMatrix::<f64>::zeros(total_constraint_rows, total_cols);
                let mut current_row = 0;

                for j in &contributing_joints {
                    let na = norm_part(&j.part_a);
                    let nb = norm_part(&j.part_b);
                    let col_a = moving_index_map.get(&na).copied();
                    let col_b = moving_index_map.get(&nb).copied();

                    let (pos_a, rot_a) = self.get_world_pose_vectors(&na);
                    let (pos_b, rot_b) = self.get_world_pose_vectors(&nb);

                    if let Some((status, warn)) =
                        evaluate_joint_limit_status(j, pos_a, rot_a, pos_b, rot_b)
                    {
                        joints_at_limits.push(status);
                        limit_warnings.push(warn);
                    }

                    let rows_added = j.joint_type.fill_jacobian_rows(
                        &mut j_mat,
                        current_row,
                        col_a,
                        col_b,
                        pos_a,
                        rot_a,
                        pos_b,
                        rot_b,
                    );
                    joint_row_spans.push((j, current_row, rows_added));
                    current_row += rows_added;
                }

                let svd = j_mat.clone().svd(false, false);
                let s_max = svd.singular_values[0];
                let tol = match rank_tolerance {
                    Some(t) => t,
                    None => ((total_constraint_rows.max(total_cols) as f64) * s_max.max(1.0) * 1e-7)
                        .max(1e-9),
                };

                let rank = svd.singular_values.iter().filter(|&&s| s > tol).count();
                let s_vals: Vec<f64> = svd.singular_values.iter().copied().collect();
                (Some(j_mat), rank, s_vals, tol)
            };

        let dof_numeric = (6 * num_moving as i32) - (jacobian_rank as i32);
        let sum_constraints: i32 = total_constraint_rows as i32;
        let dof_grubler = (6 * num_moving as i32) - sum_constraints;
        let redundant_constraints = (dof_numeric - dof_grubler).max(0);
        let redundant_joint_ids = match j_mat_opt {
            Some(ref j_mat) if redundant_constraints > 0 && total_cols > 0 => {
                find_redundant_joint_ids(j_mat, &joint_row_spans, total_cols, jacobian_rank, effective_tol)
            }
            _ => Vec::new(),
        };

        let mut warnings = Vec::new();
        if !contributing_joints.is_empty() && dof_numeric == 0 {
            warnings.push("DOF=0 при заявленном механизме".to_string());
        }
        let motors_count = contributing_joints.iter().filter(|j| j.motor.is_some()).count();
        if dof_numeric > 1 && motors_count == 0 {
            warnings.push("DOF>1 без двигателей".to_string());
        }
        if !unconnected_parts.is_empty() || !floating_parts.is_empty() {
            warnings.push("есть свободные детали".to_string());
        }
        warnings.extend(limit_warnings);

        let mut notes = Vec::new();
        notes.push("Chebychev-Grübler-Kutzbach spatial mobility formula: M = 6(N - 1) - sum(c_i), where N includes ground and c_i are joint constraint counts.".to_string());
        notes.push(format!(
            "Numerical mobility: DOF = 6 * N_moving ({}) - rank(J) ({}) = {}.",
            6 * num_moving, jacobian_rank, dof_numeric
        ));
        notes.push(format!(
            "Jacobian SVD rank tolerance: {:.2e} (maximum singular value: {:.4}).",
            effective_tol,
            singular_values.first().copied().unwrap_or(0.0)
        ));
        if redundant_constraints > 0 {
            notes.push(format!(
                "Detected {} redundant constraints involving joints [{}] (common in planar four-bar linkages, slider-cranks, and overconstrained parallel mechanisms).",
                redundant_constraints,
                redundant_joint_ids.join(", ")
            ));
        }
        if !joints_at_limits.is_empty() {
            let locked_count = joints_at_limits
                .iter()
                .filter(|s| s.boundary == "locked")
                .count();
            notes.push(format!(
                "Detected {} joint(s) at or beyond travel limits ({} bilaterally locked with zero-range limits).",
                joints_at_limits.len(),
                locked_count
            ));
        }

        Ok(MechanismAnalysisReport {
            dof_numeric,
            dof_grubler,
            redundant_constraints,
            redundant_joint_ids,
            joints_at_limits,
            grounded_parts: reported_grounded,
            moving_parts,
            floating_parts,
            unconnected_parts,
            joint_constraints_count: total_constraint_rows,
            jacobian_rows: total_constraint_rows,
            jacobian_cols: total_cols,
            jacobian_rank,
            singular_values,
            rank_tolerance: effective_tol,
            warnings,
            notes,
        })
    }

    /// Computes the world-space `TriangleMesh` for `part_id` by transforming its local mesh
    /// by the rigid body's current world pose `(translation, rotation)`.
    pub fn world_mesh_for_part(
        &self,
        part_id: &str,
        scene_parts: Option<&HashMap<String, Part>>,
    ) -> Result<TriangleMesh, PhysicsError> {
        let base_mesh = scene_parts
            .and_then(|sp| sp.get(part_id))
            .map(|p| &p.mesh)
            .or_else(|| self.part_meshes.get(part_id))
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))?;

        if base_mesh.vertices.is_empty() || base_mesh.indices.is_empty() {
            return Err(PhysicsError::InvalidGeometry(
                part_id.to_string(),
                "Part mesh is empty (0 vertices or 0 triangles)".to_string(),
            ));
        }

        let (pos, rot_mat) = self.get_world_pose_vectors(part_id);
        Ok(transform_triangle_mesh(base_mesh, pos, rot_mat))
    }

    /// Checks static multi-body geometric interference and clearance across registered parts (`P3-T10`).
    ///
    /// Performs:
    /// 1. Broad-phase world-space AABB overlap filtering across candidate part pairs.
    /// 2. Narrow-phase exact Boolean intersection (`csg_intersection`) on overlapping AABB pairs
    ///    to compute `intersection_volume` and `penetration_depth`.
    /// 3. Exact minimum Euclidean distance (`clearance`) computation on non-intersecting pairs.
    pub fn check_interference(
        &self,
        part_ids: Option<&[String]>,
        ignore_adjacent: bool,
        explicit_pairs: Option<&[(String, String)]>,
    ) -> Result<InterferenceReport, PhysicsError> {
        self.check_interference_with_parts(part_ids, ignore_adjacent, explicit_pairs, None)
    }

    /// Variant of [`Self::check_interference`] that also accepts a reference to the CAD scene's
    /// `Part` map so parts not yet inserted into `PhysicsWorld` can be evaluated in place.
    pub fn check_interference_with_parts(
        &self,
        part_ids: Option<&[String]>,
        ignore_adjacent: bool,
        explicit_pairs: Option<&[(String, String)]>,
        scene_parts: Option<&HashMap<String, Part>>,
    ) -> Result<InterferenceReport, PhysicsError> {
        let start_time = Instant::now();

        let norm_part = |id: &str| -> String {
            if id.eq_ignore_ascii_case("ground") {
                "ground".to_string()
            } else if id.eq_ignore_ascii_case("world") {
                "world".to_string()
            } else {
                id.to_string()
            }
        };

        // Build adjacency set of part pairs directly connected by a joint
        let mut adjacent_pairs: std::collections::HashSet<(String, String)> =
            std::collections::HashSet::new();
        for joint in self.joints.values() {
            let na = norm_part(&joint.part_a);
            let nb = norm_part(&joint.part_b);
            if na != "ground" && na != "world" && nb != "ground" && nb != "world" {
                adjacent_pairs.insert((na.clone(), nb.clone()));
                adjacent_pairs.insert((nb, na));
            }
        }

        // Determine candidate pairs
        let candidate_pairs: Vec<(String, String)> = if let Some(pairs) = explicit_pairs {
            let mut list = Vec::with_capacity(pairs.len());
            let mut seen = std::collections::HashSet::new();
            for (pa, pb) in pairs {
                let a = pa.trim().to_string();
                let b = pb.trim().to_string();
                if a.is_empty() || b.is_empty() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Interference pair part IDs must be non-empty strings".to_string(),
                    ));
                }
                if a == b {
                    return Err(PhysicsError::InvalidJointConfiguration(format!(
                        "Cannot check interference of part '{a}' against itself"
                    )));
                }
                let key = if a < b {
                    (a.clone(), b.clone())
                } else {
                    (b.clone(), a.clone())
                };
                if seen.insert(key) {
                    list.push((a, b));
                }
            }
            list
        } else {
            let mut parts_list: Vec<String> = if let Some(ids) = part_ids {
                let mut unique = Vec::with_capacity(ids.len());
                let mut seen = std::collections::HashSet::new();
                for raw in ids {
                    let pid = raw.trim().to_string();
                    if pid.is_empty()
                        || pid.eq_ignore_ascii_case("ground")
                        || pid.eq_ignore_ascii_case("world")
                    {
                        continue;
                    }
                    if seen.insert(pid.clone()) {
                        unique.push(pid);
                    }
                }
                unique
            } else {
                let mut all_ids = std::collections::HashSet::new();
                for pid in self.part_meshes.keys() {
                    if !pid.eq_ignore_ascii_case("ground") && !pid.eq_ignore_ascii_case("world") {
                        all_ids.insert(pid.clone());
                    }
                }
                if let Some(sp) = scene_parts {
                    for (pid, part) in sp {
                        if !part.mesh.vertices.is_empty() {
                            all_ids.insert(pid.clone());
                        }
                    }
                }
                all_ids.into_iter().collect()
            };
            parts_list.sort();

            let mut list = Vec::new();
            for i in 0..parts_list.len() {
                for j in (i + 1)..parts_list.len() {
                    list.push((parts_list[i].clone(), parts_list[j].clone()));
                }
            }
            list
        };

        // Precompute world-space meshes and AABBs for all referenced parts
        let mut needed_parts = std::collections::HashSet::new();
        for (a, b) in &candidate_pairs {
            needed_parts.insert(a.clone());
            needed_parts.insert(b.clone());
        }

        let mut world_meshes: HashMap<String, (TriangleMesh, BoundingBox)> =
            HashMap::with_capacity(needed_parts.len());
        for pid in needed_parts {
            let mesh = self.world_mesh_for_part(&pid, scene_parts)?;
            let aabb = mesh.bounding_box();
            world_meshes.insert(pid, (mesh, aabb));
        }

        let candidate_pairs_count = candidate_pairs.len();
        let mut skipped_adjacent_pairs_count = 0usize;
        let mut broadphase_overlap_pairs_count = 0usize;
        let mut interfering_pairs_count = 0usize;
        let mut min_clearance: Option<f64> = None;
        let mut max_intersection_volume = 0.0_f64;
        let mut max_penetration_depth = 0.0_f64;
        let mut results: Vec<PairInterferenceResult> = Vec::with_capacity(candidate_pairs_count);

        let aabb_eps = 1e-9_f64;
        let vol_eps = 1e-15_f64;

        for (a, b) in candidate_pairs {
            let adjacent_by_joint = adjacent_pairs.contains(&(a.clone(), b.clone()));
            if ignore_adjacent && adjacent_by_joint {
                skipped_adjacent_pairs_count += 1;
                continue;
            }

            let (mesh_a, aabb_a) = &world_meshes[&a];
            let (mesh_b, aabb_b) = &world_meshes[&b];

            let aabb_overlap = aabb_a.min[0] <= aabb_b.max[0] + aabb_eps
                && aabb_b.min[0] <= aabb_a.max[0] + aabb_eps
                && aabb_a.min[1] <= aabb_b.max[1] + aabb_eps
                && aabb_b.min[1] <= aabb_a.max[1] + aabb_eps
                && aabb_a.min[2] <= aabb_b.max[2] + aabb_eps
                && aabb_b.min[2] <= aabb_a.max[2] + aabb_eps;

            if aabb_overlap {
                broadphase_overlap_pairs_count += 1;
                let inter_mesh = csg_intersection(mesh_a, mesh_b);
                let vol = if inter_mesh.is_empty() {
                    0.0
                } else {
                    inter_mesh.volume()
                };

                if vol > vol_eps {
                    interfering_pairs_count += 1;
                    let pen_depth =
                        compute_penetration_depth_from_intersection(&inter_mesh, aabb_a, aabb_b);
                    if vol > max_intersection_volume {
                        max_intersection_volume = vol;
                    }
                    if pen_depth > max_penetration_depth {
                        max_penetration_depth = pen_depth;
                    }
                    results.push(PairInterferenceResult {
                        a,
                        b,
                        intersects: true,
                        adjacent_by_joint,
                        aabb_overlap: true,
                        penetration_depth: Some(pen_depth),
                        intersection_volume: Some(vol),
                        clearance: None,
                    });
                    continue;
                }
            }

            // Non-intersecting pair: compute minimum separation distance (clearance)
            let clearance = compute_mesh_clearance(mesh_a, aabb_a, mesh_b, aabb_b);
            min_clearance = Some(match min_clearance {
                Some(prev) => prev.min(clearance),
                None => clearance,
            });
            results.push(PairInterferenceResult {
                a,
                b,
                intersects: false,
                adjacent_by_joint,
                aabb_overlap,
                penetration_depth: None,
                intersection_volume: None,
                clearance: Some(clearance),
            });
        }

        let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
        let status = if interfering_pairs_count == 0 {
            "PASS".to_string()
        } else {
            "FAIL".to_string()
        };

        Ok(InterferenceReport {
            status,
            candidate_pairs_count,
            skipped_adjacent_pairs_count,
            broadphase_overlap_pairs_count,
            evaluated_pairs_count: results.len(),
            interfering_pairs_count,
            min_clearance,
            max_intersection_volume,
            max_penetration_depth,
            ignore_adjacent,
            pairs: results,
            elapsed_ms,
        })
    }

    /// Sweeps a `Revolute` or `Prismatic` joint across `[from, to]` in `steps` intervals (`0..=steps`),
    /// evaluating multi-body interference and clearance at each discrete configuration (`P3-T10`).
    ///
    /// * For `CadJointType::Revolute`, `from` and `to` are angles in **radians**.
    /// * For `CadJointType::Prismatic`, `from` and `to` are translations in **meters**.
    /// * Does not mutate `self` (evaluates on a cloned world state).
    pub fn sweep_joint(
        &self,
        joint_id: &str,
        from: f64,
        to: f64,
        steps: usize,
        ignore_adjacent: bool,
    ) -> Result<JointSweepReport, PhysicsError> {
        self.sweep_joint_with_parts(joint_id, from, to, steps, ignore_adjacent, None)
    }

    /// Variant of [`Self::sweep_joint`] that accepts a reference to the CAD scene's `Part` map.
    pub fn sweep_joint_with_parts(
        &self,
        joint_id: &str,
        from: f64,
        to: f64,
        steps: usize,
        ignore_adjacent: bool,
        scene_parts: Option<&HashMap<String, Part>>,
    ) -> Result<JointSweepReport, PhysicsError> {
        let start_time = Instant::now();

        if !from.is_finite() || !to.is_finite() {
            return Err(PhysicsError::InvalidJointConfiguration(format!(
                "sweep_joint range [from, to] must be finite numbers, got [{from}, {to}]"
            )));
        }
        if steps == 0 || steps > 10_000 {
            return Err(PhysicsError::InvalidJointConfiguration(format!(
                "sweep_joint 'steps' must be between 1 and 10000, got {steps}"
            )));
        }

        let record = self
            .joints
            .get(joint_id)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?
            .clone();

        let (is_revolute, anchor_a, anchor_b, axis_raw) = match &record.joint_type {
            CadJointType::Revolute {
                anchor_a,
                anchor_b,
                axis,
                ..
            } => (true, *anchor_a, *anchor_b, *axis),
            CadJointType::Prismatic {
                anchor_a,
                anchor_b,
                axis,
                ..
            } => (false, *anchor_a, *anchor_b, *axis),
            other => {
                return Err(PhysicsError::InvalidJointConfiguration(format!(
                    "sweep_joint is only supported on 'revolute' and 'prismatic' joints, but joint '{joint_id}' has type '{}'",
                    other.type_name()
                )));
            }
        };

        let axis_vec = Vector3::new(axis_raw[0], axis_raw[1], axis_raw[2]);
        if axis_vec.norm() < 1e-9 {
            return Err(PhysicsError::InvalidJointConfiguration(format!(
                "Joint '{joint_id}' has a degenerate zero axis"
            )));
        }

        let mut sim_world = self.clone();
        if let Some(sp) = scene_parts {
            for part in sp.values() {
                if !part.mesh.vertices.is_empty() && !sim_world.has_part(&part.id) {
                    let _ = sim_world.add_part(part, !part.is_fixed);
                }
            }
        }

        let is_fixed_or_ground = |pid: &str, w: &PhysicsWorld| -> bool {
            if pid.eq_ignore_ascii_case("ground") || pid.eq_ignore_ascii_case("world") {
                true
            } else if let Some(sp) = scene_parts {
                if let Some(p) = sp.get(pid) {
                    return p.is_fixed;
                }
                !w.is_part_dynamic(pid).unwrap_or(true)
            } else {
                !w.is_part_dynamic(pid).unwrap_or(true)
            }
        };

        let a_fixed = is_fixed_or_ground(&record.part_a, &sim_world);
        let b_fixed = is_fixed_or_ground(&record.part_b, &sim_world);

        let (reference_part_id, driven_part_id, anchor_ref, anchor_driven) = if a_fixed && !b_fixed
        {
            (
                record.part_a.clone(),
                record.part_b.clone(),
                Vector3::new(anchor_a[0], anchor_a[1], anchor_a[2]),
                Vector3::new(anchor_b[0], anchor_b[1], anchor_b[2]),
            )
        } else if b_fixed && !a_fixed {
            (
                record.part_b.clone(),
                record.part_a.clone(),
                Vector3::new(anchor_b[0], anchor_b[1], anchor_b[2]),
                Vector3::new(anchor_a[0], anchor_a[1], anchor_a[2]),
            )
        } else {
            // Determine which side is closer to ground via BFS excluding this joint
            let mut adj: HashMap<String, Vec<String>> = HashMap::new();
            for (jid, jrec) in &sim_world.joints {
                if jid == joint_id {
                    continue;
                }
                adj.entry(jrec.part_a.clone())
                    .or_default()
                    .push(jrec.part_b.clone());
                adj.entry(jrec.part_b.clone())
                    .or_default()
                    .push(jrec.part_a.clone());
            }

            let bfs_reaches_ground = |start: &str| -> Option<usize> {
                let mut visited = std::collections::HashSet::new();
                let mut queue = std::collections::VecDeque::new();
                visited.insert(start.to_string());
                queue.push_back((start.to_string(), 0usize));
                while let Some((curr, d)) = queue.pop_front() {
                    if is_fixed_or_ground(&curr, &sim_world) {
                        return Some(d);
                    }
                    if let Some(neighbors) = adj.get(&curr) {
                        for n in neighbors {
                            if visited.insert(n.clone()) {
                                queue.push_back((n.clone(), d + 1));
                            }
                        }
                    }
                }
                None
            };

            let da = bfs_reaches_ground(&record.part_a).unwrap_or(usize::MAX);
            let db = bfs_reaches_ground(&record.part_b).unwrap_or(usize::MAX);
            if da <= db {
                (
                    record.part_a.clone(),
                    record.part_b.clone(),
                    Vector3::new(anchor_a[0], anchor_a[1], anchor_a[2]),
                    Vector3::new(anchor_b[0], anchor_b[1], anchor_b[2]),
                )
            } else {
                (
                    record.part_b.clone(),
                    record.part_a.clone(),
                    Vector3::new(anchor_b[0], anchor_b[1], anchor_b[2]),
                    Vector3::new(anchor_a[0], anchor_a[1], anchor_a[2]),
                )
            }
        };

        // Collect downstream moving subtree attached to `driven_part_id` (without crossing `joint_id` or fixed bodies)
        let mut downstream_parts: Vec<String> = Vec::new();
        {
            let mut visited = std::collections::HashSet::new();
            let mut queue = std::collections::VecDeque::new();
            visited.insert(reference_part_id.clone());
            visited.insert(driven_part_id.clone());
            queue.push_back(driven_part_id.clone());

            while let Some(curr) = queue.pop_front() {
                for (jid, jrec) in &sim_world.joints {
                    if jid == joint_id {
                        continue;
                    }
                    let next_opt = if jrec.part_a == curr {
                        Some(&jrec.part_b)
                    } else if jrec.part_b == curr {
                        Some(&jrec.part_a)
                    } else {
                        None
                    };
                    if let Some(next_id) = next_opt {
                        if !is_fixed_or_ground(next_id, &sim_world)
                            && visited.insert(next_id.clone())
                        {
                            downstream_parts.push(next_id.clone());
                            queue.push_back(next_id.clone());
                        }
                    }
                }
            }
        }

        // Detect whether the swept joint belongs to a closed kinematic loop (`P3-T16`).
        let mut m_drv_set: std::collections::HashSet<String> = std::collections::HashSet::new();
        m_drv_set.insert(driven_part_id.clone());
        for dp in &downstream_parts {
            m_drv_set.insert(dp.clone());
        }
        let mut touches_boundary = 0usize;
        let mut touches_external_or_ground = false;
        let mut internal_joints_count = 0usize;
        for (jid, jrec) in &sim_world.joints {
            if jid == joint_id {
                continue;
            }
            let a_in = m_drv_set.contains(&jrec.part_a);
            let b_in = m_drv_set.contains(&jrec.part_b);
            if a_in && b_in {
                internal_joints_count += 1;
            } else if a_in != b_in {
                touches_boundary += 1;
                let other = if a_in { &jrec.part_b } else { &jrec.part_a };
                if other != &reference_part_id {
                    touches_external_or_ground = true;
                }
            }
        }
        let is_closed_loop =
            touches_boundary > 0 || internal_joints_count > m_drv_set.len().saturating_sub(1);

        let (pos_ref_0, rot_ref_0) = sim_world.get_world_pose_vectors(&reference_part_id);
        let (pos_drv_0, rot_drv_0) = sim_world.get_world_pose_vectors(&driven_part_id);
        let rot_drv_0_inv = rot_drv_0.transpose();

        let mut downstream_initial_poses: Vec<(String, Vector3<f64>, Matrix3<f64>)> =
            Vec::with_capacity(downstream_parts.len());
        for d_id in &downstream_parts {
            let (p0, r0) = sim_world.get_world_pose_vectors(d_id);
            downstream_initial_poses.push((d_id.clone(), p0, r0));
        }

        let pivot_world = pos_ref_0 + rot_ref_0 * anchor_ref;
        let world_axis_unnorm = rot_ref_0 * axis_vec;
        let world_axis = nalgebra::Unit::new_normalize(world_axis_unnorm);

        let mat_to_quat_arr = |m: &Matrix3<f64>| -> [f64; 4] {
            let rot3 = nalgebra::Rotation3::from_matrix_unchecked(*m);
            let uq = nalgebra::UnitQuaternion::from_rotation_matrix(&rot3);
            let q = uq.quaternion();
            [q.coords.x, q.coords.y, q.coords.z, q.coords.w]
        };

        let mut samples = Vec::with_capacity(steps + 1);
        let mut first_collision_step: Option<usize> = None;
        let mut first_collision_value: Option<f64> = None;
        let mut first_collision_pairs: Vec<PairInterferenceResult> = Vec::new();
        let mut min_clearance: Option<f64> = None;
        let mut min_clearance_step: Option<usize> = None;
        let mut min_clearance_value: Option<f64> = None;
        let mut lockup_step: Option<usize> = None;
        let mut lockup_value: Option<f64> = None;
        let mut max_closure_residual = 0.0_f64;

        if !is_closed_loop {
            for step_idx in 0..=steps {
                let alpha = (step_idx as f64) / (steps as f64);
                let q_val = from + alpha * (to - from);

                let (rot_drv_k, pos_drv_k) = if is_revolute {
                    let delta_rot =
                        nalgebra::Rotation3::from_axis_angle(&world_axis, q_val).into_inner();
                    let rk = delta_rot * rot_drv_0;
                    let pk = pivot_world - rk * anchor_driven;
                    (rk, pk)
                } else {
                    let rk = rot_drv_0;
                    let pk =
                        pivot_world - rk * anchor_driven + world_axis.into_inner() * q_val;
                    (rk, pk)
                };

                let quat_drv_k = mat_to_quat_arr(&rot_drv_k);
                let _ = sim_world.set_part_pose(
                    &driven_part_id,
                    [pos_drv_k.x, pos_drv_k.y, pos_drv_k.z],
                    quat_drv_k,
                );

                // Update any downstream parts rigidly relative to driven_part_id
                let rel_rot = rot_drv_k * rot_drv_0_inv;
                for (d_id, p0, r0) in &downstream_initial_poses {
                    let rk = rel_rot * r0;
                    let pk = pos_drv_k + rel_rot * (p0 - pos_drv_0);
                    let qk = mat_to_quat_arr(&rk);
                    let _ = sim_world.set_part_pose(d_id, [pk.x, pk.y, pk.z], qk);
                }

                let step_report = sim_world.check_interference_with_parts(
                    None,
                    ignore_adjacent,
                    None,
                    scene_parts,
                )?;

                let step_interfering: Vec<PairInterferenceResult> = step_report
                    .pairs
                    .iter()
                    .filter(|p| p.intersects)
                    .cloned()
                    .collect();
                let intersects = !step_interfering.is_empty();

                if intersects && first_collision_step.is_none() {
                    first_collision_step = Some(step_idx);
                    first_collision_value = Some(q_val);
                    first_collision_pairs = step_interfering.clone();
                }

                if !intersects {
                    if let Some(clr) = step_report.min_clearance {
                        let is_smaller = match min_clearance {
                            None => true,
                            Some(prev) => clr < prev,
                        };
                        if is_smaller {
                            min_clearance = Some(clr);
                            min_clearance_step = Some(step_idx);
                            min_clearance_value = Some(q_val);
                        }
                    }
                }

                samples.push(SweepStepResult {
                    step_index: step_idx,
                    value: q_val,
                    intersects,
                    interfering_pairs_count: step_interfering.len(),
                    min_clearance: if intersects {
                        Some(0.0)
                    } else {
                        step_report.min_clearance
                    },
                    max_penetration_depth: step_report.max_penetration_depth,
                    max_intersection_volume: step_report.max_intersection_volume,
                    interfering_pairs: step_interfering,
                    closure_residual: 0.0,
                    locked_up: false,
                });
            }
        } else {
            // Closed-loop iterative constraint solver (`P3-T16`)
            let ref_is_fixed = is_fixed_or_ground(&reference_part_id, &sim_world);
            let include_ref_as_free = !ref_is_fixed && touches_external_or_ground;

            let mut free_parts: Vec<String> = if include_ref_as_free {
                let mut visited = std::collections::HashSet::new();
                let mut queue = std::collections::VecDeque::new();
                let mut list = Vec::new();
                for start in [&reference_part_id, &driven_part_id] {
                    if !is_fixed_or_ground(start, &sim_world) && visited.insert(start.clone()) {
                        list.push(start.clone());
                        queue.push_back(start.clone());
                    }
                }
                while let Some(curr) = queue.pop_front() {
                    for jrec in sim_world.joints.values() {
                        let next_opt = if jrec.part_a == curr {
                            Some(&jrec.part_b)
                        } else if jrec.part_b == curr {
                            Some(&jrec.part_a)
                        } else {
                            None
                        };
                        if let Some(next_id) = next_opt {
                            if !is_fixed_or_ground(next_id, &sim_world)
                                && visited.insert(next_id.clone())
                            {
                                list.push(next_id.clone());
                                queue.push_back(next_id.clone());
                            }
                        }
                    }
                }
                list
            } else {
                downstream_parts.clone()
            };
            free_parts.sort();

            let mut solver_ctx = ClosedLoopSolver::new(
                &sim_world,
                joint_id,
                &reference_part_id,
                &driven_part_id,
                is_revolute,
                anchor_ref,
                anchor_driven,
                axis_vec.normalize(),
                pos_ref_0,
                rot_ref_0,
                pos_drv_0,
                rot_drv_0,
                free_parts,
                include_ref_as_free,
            );

            for step_idx in 0..=steps {
                let alpha = (step_idx as f64) / (steps as f64);
                let q_val = from + alpha * (to - from);

                if lockup_step.is_some() {
                    // Once locked up along the monotonic sweep, subsequent steps remain unreachable.
                    let residual = solver_ctx.last_failed_residual;
                    samples.push(SweepStepResult {
                        step_index: step_idx,
                        value: q_val,
                        intersects: false,
                        interfering_pairs_count: 0,
                        min_clearance: None,
                        max_penetration_depth: 0.0,
                        max_intersection_volume: 0.0,
                        interfering_pairs: Vec::new(),
                        closure_residual: residual,
                        locked_up: true,
                    });
                    continue;
                }

                match solver_ctx.advance_to(q_val) {
                    Ok(residual) => {
                        if residual > max_closure_residual {
                            max_closure_residual = residual;
                        }
                        for (pid, (pk, rk)) in &solver_ctx.poses {
                            if !is_fixed_or_ground(pid, &sim_world) {
                                let qk = mat_to_quat_arr(rk);
                                let _ = sim_world.set_part_pose(pid, [pk.x, pk.y, pk.z], qk);
                            }
                        }

                        let step_report = sim_world.check_interference_with_parts(
                            None,
                            ignore_adjacent,
                            None,
                            scene_parts,
                        )?;

                        let step_interfering: Vec<PairInterferenceResult> = step_report
                            .pairs
                            .iter()
                            .filter(|p| p.intersects)
                            .cloned()
                            .collect();
                        let intersects = !step_interfering.is_empty();

                        if intersects && first_collision_step.is_none() {
                            first_collision_step = Some(step_idx);
                            first_collision_value = Some(q_val);
                            first_collision_pairs = step_interfering.clone();
                        }

                        if !intersects {
                            if let Some(clr) = step_report.min_clearance {
                                let is_smaller = match min_clearance {
                                    None => true,
                                    Some(prev) => clr < prev,
                                };
                                if is_smaller {
                                    min_clearance = Some(clr);
                                    min_clearance_step = Some(step_idx);
                                    min_clearance_value = Some(q_val);
                                }
                            }
                        }

                        samples.push(SweepStepResult {
                            step_index: step_idx,
                            value: q_val,
                            intersects,
                            interfering_pairs_count: step_interfering.len(),
                            min_clearance: if intersects {
                                Some(0.0)
                            } else {
                                step_report.min_clearance
                            },
                            max_penetration_depth: step_report.max_penetration_depth,
                            max_intersection_volume: step_report.max_intersection_volume,
                            interfering_pairs: step_interfering,
                            closure_residual: residual,
                            locked_up: false,
                        });
                    }
                    Err(residual) => {
                        if residual > max_closure_residual {
                            max_closure_residual = residual;
                        }
                        lockup_step = Some(step_idx);
                        lockup_value = Some(q_val);
                        samples.push(SweepStepResult {
                            step_index: step_idx,
                            value: q_val,
                            intersects: false,
                            interfering_pairs_count: 0,
                            min_clearance: None,
                            max_penetration_depth: 0.0,
                            max_intersection_volume: 0.0,
                            interfering_pairs: Vec::new(),
                            closure_residual: residual,
                            locked_up: true,
                        });
                    }
                }
            }
        }

        let collision_detected = first_collision_step.is_some();
        let lockup_detected = lockup_step.is_some();
        let safe_travel_range = match (first_collision_value, lockup_value) {
            (Some(v_col), _) => (v_col - from).abs(),
            (None, Some(v_lock)) => (v_lock - from).abs(),
            (None, None) => (to - from).abs(),
        };
        let margin_to_collision = if collision_detected {
            Some(safe_travel_range)
        } else {
            min_clearance
        };

        let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
        let status = if collision_detected || lockup_detected {
            "FAIL".to_string()
        } else {
            "PASS".to_string()
        };

        Ok(JointSweepReport {
            status,
            joint_id: joint_id.to_string(),
            joint_type: record.joint_type.type_name().to_string(),
            driven_part_id,
            reference_part_id,
            from,
            to,
            steps,
            collision_detected,
            first_collision_step,
            first_collision_value,
            first_collision_pairs,
            min_clearance,
            min_clearance_step,
            min_clearance_value,
            safe_travel_range,
            margin_to_collision,
            ignore_adjacent,
            closed_loop_detected: is_closed_loop,
            lockup_detected,
            lockup_step,
            lockup_value,
            max_closure_residual,
            samples,
            elapsed_ms,
        })
    }
}

struct LoopJointEntry {
    record: CadJointRecord,
    initial_rel_rot: Matrix3<f64>,
    initial_coord: Option<f64>,
    is_driven_constraint: bool,
    gear_theta_a: f64,
    gear_theta_b: f64,
}

struct ClosedLoopSolver {
    reference_part_id: String,
    driven_part_id: String,
    is_revolute: bool,
    anchor_ref: Vector3<f64>,
    anchor_driven: Vector3<f64>,
    local_axis_ref: Vector3<f64>,
    rot_ref_0: Matrix3<f64>,
    rot_drv_0: Matrix3<f64>,
    pivot_world: Vector3<f64>,
    world_axis: nalgebra::Unit<Vector3<f64>>,
    driven_as_constraint: bool,
    free_parts: Vec<String>,
    free_index: HashMap<String, usize>,
    joints: Vec<LoopJointEntry>,
    poses: HashMap<String, (Vector3<f64>, Matrix3<f64>)>,
    char_len: f64,
    pos_tol: f64,
    rot_tol: f64,
    current_q: f64,
    last_failed_residual: f64,
}

impl ClosedLoopSolver {
    #[allow(clippy::too_many_arguments)]
    fn new(
        world: &PhysicsWorld,
        driven_joint_id: &str,
        reference_part_id: &str,
        driven_part_id: &str,
        is_revolute: bool,
        anchor_ref: Vector3<f64>,
        anchor_driven: Vector3<f64>,
        local_axis_ref: Vector3<f64>,
        pos_ref_0: Vector3<f64>,
        rot_ref_0: Matrix3<f64>,
        pos_drv_0: Vector3<f64>,
        rot_drv_0: Matrix3<f64>,
        free_parts: Vec<String>,
        driven_as_constraint: bool,
    ) -> Self {
        let free_set: std::collections::HashSet<String> = free_parts.iter().cloned().collect();
        let free_index: HashMap<String, usize> = free_parts
            .iter()
            .enumerate()
            .map(|(i, id)| (id.clone(), i))
            .collect();

        let mut poses: HashMap<String, (Vector3<f64>, Matrix3<f64>)> = HashMap::new();
        let mut joint_ids: Vec<String> = world.joints.keys().cloned().collect();
        joint_ids.sort();

        let mut joints = Vec::new();
        let mut points_for_scale: Vec<Vector3<f64>> = Vec::new();

        for jid in joint_ids {
            let Some(rec) = world.joints.get(&jid) else {
                continue;
            };
            let is_driven = jid == driven_joint_id;
            if is_driven && !driven_as_constraint {
                continue;
            }
            let touches_free = free_set.contains(&rec.part_a) || free_set.contains(&rec.part_b);
            let touches_driven_only = free_parts.is_empty()
                && !is_driven
                && (rec.part_a == driven_part_id || rec.part_b == driven_part_id);
            if !touches_free && !touches_driven_only && !is_driven {
                continue;
            }

            let (pa, ra) = *poses
                .entry(rec.part_a.clone())
                .or_insert_with(|| world.get_world_pose_vectors(&rec.part_a));
            let (pb, rb) = *poses
                .entry(rec.part_b.clone())
                .or_insert_with(|| world.get_world_pose_vectors(&rec.part_b));

            let aa = Vector3::from(rec.joint_type.anchor_a());
            let ab = Vector3::from(rec.joint_type.anchor_b());
            points_for_scale.push(pa + ra * aa);
            points_for_scale.push(pb + rb * ab);

            let initial_rel_rot = ra.transpose() * rb;
            let initial_coord = rec.joint_type.current_coordinate(pa, ra, pb, rb);

            joints.push(LoopJointEntry {
                record: rec.clone(),
                initial_rel_rot,
                initial_coord,
                is_driven_constraint: is_driven,
                gear_theta_a: 0.0,
                gear_theta_b: 0.0,
            });
        }

        poses
            .entry(reference_part_id.to_string())
            .or_insert((pos_ref_0, rot_ref_0));
        poses
            .entry(driven_part_id.to_string())
            .or_insert((pos_drv_0, rot_drv_0));
        points_for_scale.push(pos_ref_0 + rot_ref_0 * anchor_ref);
        points_for_scale.push(pos_drv_0 + rot_drv_0 * anchor_driven);

        let mut max_span = 0.0_f64;
        for i in 0..points_for_scale.len() {
            for j in (i + 1)..points_for_scale.len() {
                let d = (points_for_scale[i] - points_for_scale[j]).norm();
                if d > max_span {
                    max_span = d;
                }
            }
        }
        let char_len = if max_span > 1e-6 {
            max_span.clamp(1e-3, 100.0)
        } else {
            0.1
        };
        let pos_tol = (char_len * 1e-3).clamp(2e-5, 5e-4);
        let rot_tol = 1e-3;

        let pivot_world = pos_ref_0 + rot_ref_0 * anchor_ref;
        let world_axis = nalgebra::Unit::new_normalize(rot_ref_0 * local_axis_ref);

        Self {
            reference_part_id: reference_part_id.to_string(),
            driven_part_id: driven_part_id.to_string(),
            is_revolute,
            anchor_ref,
            anchor_driven,
            local_axis_ref,
            rot_ref_0,
            rot_drv_0,
            pivot_world,
            world_axis,
            driven_as_constraint,
            free_parts,
            free_index,
            joints,
            poses,
            char_len,
            pos_tol,
            rot_tol,
            current_q: 0.0,
            last_failed_residual: 0.0,
        }
    }

    fn driven_world_pose(&self, q_val: f64) -> (Vector3<f64>, Matrix3<f64>) {
        if self.is_revolute {
            let delta_rot =
                nalgebra::Rotation3::from_axis_angle(&self.world_axis, q_val).into_inner();
            let rk = delta_rot * self.rot_drv_0;
            let pk = self.pivot_world - rk * self.anchor_driven;
            (pk, rk)
        } else {
            let rk = self.rot_drv_0;
            let pk = self.pivot_world - rk * self.anchor_driven
                + self.world_axis.into_inner() * q_val;
            (pk, rk)
        }
    }

    fn advance_to(&mut self, q_target: f64) -> Result<f64, f64> {
        let dq_total = q_target - self.current_q;
        if dq_total.abs() <= 1e-12 && self.current_q.abs() <= 1e-12 {
            return self.solve_single_q(q_target);
        }

        let max_substep = if self.is_revolute {
            3.0_f64.to_radians()
        } else {
            (self.char_len * 0.05).max(1e-4)
        };
        let n_sub = ((dq_total.abs() / max_substep).ceil() as usize).clamp(1, 2000);
        let q_start = self.current_q;
        let mut last_res = 0.0;

        for s in 1..=n_sub {
            let q_sub = q_start + dq_total * (s as f64) / (n_sub as f64);
            match self.solve_single_q(q_sub) {
                Ok(res) => {
                    self.current_q = q_sub;
                    last_res = res;
                }
                Err(res) => {
                    self.last_failed_residual = res;
                    return Err(res);
                }
            }
        }
        Ok(last_res)
    }

    fn solve_single_q(&mut self, q_val: f64) -> Result<f64, f64> {
        let base_poses = self.poses.clone();
        let base_gears: Vec<(f64, f64)> = self
            .joints
            .iter()
            .map(|j| (j.gear_theta_a, j.gear_theta_b))
            .collect();

        if !self.driven_as_constraint {
            let drv_pose = self.driven_world_pose(q_val);
            self.poses.insert(self.driven_part_id.clone(), drv_pose);
        }

        let total_rows: usize = self
            .joints
            .iter()
            .map(|j| {
                if j.is_driven_constraint {
                    6
                } else {
                    j.record.joint_type.effective_constraints_count()
                }
            })
            .sum();

        if total_rows == 0 {
            return Ok(0.0);
        }

        let n_free = self.free_parts.len();
        if n_free == 0 {
            let eval = self.evaluate_system(q_val, &self.poses, &base_poses, &base_gears, false);
            if eval.max_pos_err <= self.pos_tol && eval.max_rot_err <= self.rot_tol {
                return Ok(eval.phys_norm);
            }
            return Err(eval.phys_norm);
        }

        let total_cols = 6 * n_free;
        let max_iters = 50usize;
        let mut lambda_factor = 1e-9_f64;

        for _iter in 0..max_iters {
            let eval = self.evaluate_system(q_val, &self.poses, &base_poses, &base_gears, true);
            if eval.max_pos_err <= self.pos_tol
                && eval.max_rot_err <= self.rot_tol
                && eval.scaled_norm <= 1e-4
            {
                self.commit_gear_angles(&base_poses, &base_gears);
                return Ok(eval.phys_norm);
            }

            let j_mat = eval.j_scaled.expect("jacobian requested");
            let svd = j_mat.svd(true, true);
            let s_vals = &svd.singular_values;
            let s_max = s_vals.as_slice().first().copied().unwrap_or(0.0).max(1e-12);
            let (Some(u_mat), Some(v_t_mat)) = (svd.u.as_ref(), svd.v_t.as_ref()) else {
                break;
            };

            let mut accepted = false;
            let mut try_lambda = (lambda_factor * s_max).max(1e-12);

            for _damp_try in 0..7 {
                let mut dx_scaled = nalgebra::DVector::<f64>::zeros(total_cols);
                let k_sv = s_vals.len();
                for i in 0..k_sv {
                    let s = s_vals[i];
                    let coeff = -s / (s * s + try_lambda * try_lambda);
                    let u_dot_phi = u_mat.column(i).dot(&eval.phi_scaled);
                    let v_col = v_t_mat.row(i).transpose();
                    dx_scaled += v_col * (coeff * u_dot_phi);
                }

                let step_norm = dx_scaled.norm();
                if step_norm > 0.5 {
                    dx_scaled *= 0.5 / step_norm;
                }

                for alpha in [1.0_f64, 0.5, 0.25] {
                    let mut trial_poses = self.poses.clone();
                    for (idx, pid) in self.free_parts.iter().enumerate() {
                        let base = 6 * idx;
                        let dp = Vector3::new(
                            dx_scaled[base] * self.char_len * alpha,
                            dx_scaled[base + 1] * self.char_len * alpha,
                            dx_scaled[base + 2] * self.char_len * alpha,
                        );
                        let dtheta = Vector3::new(
                            dx_scaled[base + 3] * alpha,
                            dx_scaled[base + 4] * alpha,
                            dx_scaled[base + 5] * alpha,
                        );
                        if let Some((p, r)) = trial_poses.get_mut(pid) {
                            *p += dp;
                            *r = nalgebra::Rotation3::from_scaled_axis(dtheta).into_inner() * (*r);
                        }
                    }

                    let trial_eval =
                        self.evaluate_system(q_val, &trial_poses, &base_poses, &base_gears, false);
                    if trial_eval.scaled_norm < eval.scaled_norm - 1e-14 {
                        self.poses = trial_poses;
                        lambda_factor = (try_lambda / s_max * 0.3).clamp(1e-11, 1e2);
                        accepted = true;
                        break;
                    }
                }

                if accepted {
                    break;
                }
                try_lambda *= 10.0;
            }

            if !accepted {
                break;
            }
        }

        let final_eval = self.evaluate_system(q_val, &self.poses, &base_poses, &base_gears, false);
        if final_eval.max_pos_err <= self.pos_tol && final_eval.max_rot_err <= self.rot_tol {
            self.commit_gear_angles(&base_poses, &base_gears);
            Ok(final_eval.phys_norm)
        } else {
            let err_res = final_eval.phys_norm;
            self.poses = base_poses;
            Err(err_res)
        }
    }

    fn commit_gear_angles(
        &mut self,
        base_poses: &HashMap<String, (Vector3<f64>, Matrix3<f64>)>,
        base_gears: &[(f64, f64)],
    ) {
        for (idx, entry) in self.joints.iter_mut().enumerate() {
            if let CadJointType::Gear { axis_a, axis_b, .. } = &entry.record.joint_type {
                let (_, ra_cur) = self.poses[&entry.record.part_a];
                let (_, rb_cur) = self.poses[&entry.record.part_b];
                let (_, ra_base) = base_poses[&entry.record.part_a];
                let (_, rb_base) = base_poses[&entry.record.part_b];
                let (_, _, ua) = local_orthonormal_basis(*axis_a, Vector3::z());
                let (_, _, ub) = local_orthonormal_basis(*axis_b, Vector3::z());
                let dta = rotation_vector(ra_cur * ra_base.transpose()).dot(&(ra_base * ua));
                let dtb = rotation_vector(rb_cur * rb_base.transpose()).dot(&(rb_base * ub));
                entry.gear_theta_a = base_gears[idx].0 + dta;
                entry.gear_theta_b = base_gears[idx].1 + dtb;
            }
        }
    }

    fn evaluate_system(
        &self,
        q_val: f64,
        poses: &HashMap<String, (Vector3<f64>, Matrix3<f64>)>,
        base_poses: &HashMap<String, (Vector3<f64>, Matrix3<f64>)>,
        base_gears: &[(f64, f64)],
        build_jacobian: bool,
    ) -> SystemEvalResult {
        let total_rows: usize = self
            .joints
            .iter()
            .map(|j| {
                if j.is_driven_constraint {
                    6
                } else {
                    j.record.joint_type.effective_constraints_count()
                }
            })
            .sum();
        let total_cols = 6 * self.free_parts.len();

        let mut phi_phys = nalgebra::DVector::<f64>::zeros(total_rows);
        let mut phi_scaled = nalgebra::DVector::<f64>::zeros(total_rows);
        let mut j_mat_opt = if build_jacobian {
            Some(DMatrix::<f64>::zeros(total_rows, total_cols))
        } else {
            None
        };

        let mut max_pos_err = 0.0_f64;
        let mut max_rot_err = 0.0_f64;
        let mut row = 0usize;
        let inv_len = 1.0 / self.char_len;

        for (j_idx, entry) in self.joints.iter().enumerate() {
            if entry.is_driven_constraint {
                let (pa, ra) = poses[&self.reference_part_id];
                let (pb, rb) = poses[&self.driven_part_id];
                let col_a = self.free_index.get(&self.reference_part_id).copied();
                let col_b = self.free_index.get(&self.driven_part_id).copied();

                let s_loc = if self.is_revolute {
                    Vector3::zeros()
                } else {
                    self.local_axis_ref * q_val
                };
                let eff_anchor_a = self.anchor_ref + s_loc;
                let r_a = pa + ra * eff_anchor_a;
                let r_b = pb + rb * self.anchor_driven;
                let dp = r_a - r_b;

                let rel_0 = self.rot_ref_0.transpose() * self.rot_drv_0;
                let target_rel = if self.is_revolute {
                    let axis_unit = nalgebra::Unit::new_normalize(self.local_axis_ref);
                    nalgebra::Rotation3::from_axis_angle(&axis_unit, q_val).into_inner() * rel_0
                } else {
                    rel_0
                };
                let r_err = rb * (ra * target_rel).transpose();
                let drot = rotation_vector(r_err);

                for k in 0..3 {
                    let pe = dp[k];
                    let re = drot[k];
                    max_pos_err = max_pos_err.max(pe.abs());
                    max_rot_err = max_rot_err.max(re.abs());
                    phi_phys[row + k] = pe;
                    phi_scaled[row + k] = pe * inv_len;
                    phi_phys[row + 3 + k] = re;
                    phi_scaled[row + 3 + k] = re;
                }

                if let Some(ref mut j_mat) = j_mat_opt {
                    let virtual_fixed = CadJointType::Fixed {
                        anchor_a: [eff_anchor_a.x, eff_anchor_a.y, eff_anchor_a.z],
                        anchor_b: [self.anchor_driven.x, self.anchor_driven.y, self.anchor_driven.z],
                    };
                    virtual_fixed.fill_jacobian_rows(j_mat, row, col_a, col_b, pa, ra, pb, rb);
                    scale_jacobian_block_rows(j_mat, row..row + 3, total_cols, inv_len);
                }
                row += 6;
                continue;
            }

            let jrec = &entry.record;
            let (pa, ra) = poses[&jrec.part_a];
            let (pb, rb) = poses[&jrec.part_b];
            let col_a = self.free_index.get(&jrec.part_a).copied();
            let col_b = self.free_index.get(&jrec.part_b).copied();

            let rows_added = match &jrec.joint_type {
                CadJointType::Fixed { anchor_a, anchor_b } => {
                    let dp = (pa + ra * Vector3::from(*anchor_a))
                        - (pb + rb * Vector3::from(*anchor_b));
                    let drot = rotation_vector(rb * (ra * entry.initial_rel_rot).transpose());
                    for k in 0..3 {
                        max_pos_err = max_pos_err.max(dp[k].abs());
                        max_rot_err = max_rot_err.max(drot[k].abs());
                        phi_phys[row + k] = dp[k];
                        phi_scaled[row + k] = dp[k] * inv_len;
                        phi_phys[row + 3 + k] = drot[k];
                        phi_scaled[row + 3 + k] = drot[k];
                    }
                    if let Some(ref mut j_mat) = j_mat_opt {
                        jrec.joint_type
                            .fill_jacobian_rows(j_mat, row, col_a, col_b, pa, ra, pb, rb);
                        scale_jacobian_block_rows(j_mat, row..row + 3, total_cols, inv_len);
                    }
                    6
                }
                CadJointType::Spherical { anchor_a, anchor_b } => {
                    let dp = (pa + ra * Vector3::from(*anchor_a))
                        - (pb + rb * Vector3::from(*anchor_b));
                    for k in 0..3 {
                        max_pos_err = max_pos_err.max(dp[k].abs());
                        phi_phys[row + k] = dp[k];
                        phi_scaled[row + k] = dp[k] * inv_len;
                    }
                    if let Some(ref mut j_mat) = j_mat_opt {
                        jrec.joint_type
                            .fill_jacobian_rows(j_mat, row, col_a, col_b, pa, ra, pb, rb);
                        scale_jacobian_block_rows(j_mat, row..row + 3, total_cols, inv_len);
                    }
                    3
                }
                CadJointType::Revolute {
                    anchor_a,
                    anchor_b,
                    axis,
                    limits,
                } => {
                    let dp = (pa + ra * Vector3::from(*anchor_a))
                        - (pb + rb * Vector3::from(*anchor_b));
                    for k in 0..3 {
                        max_pos_err = max_pos_err.max(dp[k].abs());
                        phi_phys[row + k] = dp[k];
                        phi_scaled[row + k] = dp[k] * inv_len;
                    }
                    let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::z());
                    let z_a = ra * u_a;
                    let (x_a, y_a) = perpendicular_axes(&z_a);
                    let z_b = rb * u_a;
                    let r3 = -y_a.dot(&z_b);
                    let r4 = x_a.dot(&z_b);
                    let axis_angle_err = z_a.cross(&z_b).norm().atan2(z_a.dot(&z_b));
                    max_rot_err = max_rot_err.max(axis_angle_err);
                    phi_phys[row + 3] = r3;
                    phi_scaled[row + 3] = r3;
                    phi_phys[row + 4] = r4;
                    phi_scaled[row + 4] = r4;

                    let has_zero = is_zero_range_limits(*limits);
                    if has_zero {
                        let target = limits
                            .map(|l| l[0])
                            .or(entry.initial_coord)
                            .unwrap_or(0.0);
                        let cur = jrec
                            .joint_type
                            .current_coordinate(pa, ra, pb, rb)
                            .unwrap_or(target);
                        let r5 = wrap_angle_pi(cur - target);
                        max_rot_err = max_rot_err.max(r5.abs());
                        phi_phys[row + 5] = r5;
                        phi_scaled[row + 5] = r5;
                    }
                    if let Some(ref mut j_mat) = j_mat_opt {
                        jrec.joint_type
                            .fill_jacobian_rows(j_mat, row, col_a, col_b, pa, ra, pb, rb);
                        scale_jacobian_block_rows(j_mat, row..row + 3, total_cols, inv_len);
                    }
                    if has_zero { 6 } else { 5 }
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
                    let drot = rotation_vector(rb * (ra * entry.initial_rel_rot).transpose());
                    for k in 0..3 {
                        max_rot_err = max_rot_err.max(drot[k].abs());
                        phi_phys[row + k] = drot[k];
                        phi_scaled[row + k] = drot[k];
                    }
                    let r_a = pa + ra * Vector3::from(*anchor_a);
                    let r_b = pb + rb * Vector3::from(*anchor_b);
                    let delta_r = r_b - r_a;
                    let p3 = x_a.dot(&delta_r);
                    let p4 = y_a.dot(&delta_r);
                    max_pos_err = max_pos_err.max(p3.abs()).max(p4.abs());
                    phi_phys[row + 3] = p3;
                    phi_scaled[row + 3] = p3 * inv_len;
                    phi_phys[row + 4] = p4;
                    phi_scaled[row + 4] = p4 * inv_len;

                    let has_zero = is_zero_range_limits(*limits);
                    if has_zero {
                        let target = limits
                            .map(|l| l[0])
                            .or(entry.initial_coord)
                            .unwrap_or(0.0);
                        let p5 = delta_r.dot(&z_a) - target;
                        max_pos_err = max_pos_err.max(p5.abs());
                        phi_phys[row + 5] = p5;
                        phi_scaled[row + 5] = p5 * inv_len;
                    }
                    if let Some(ref mut j_mat) = j_mat_opt {
                        jrec.joint_type
                            .fill_jacobian_rows(j_mat, row, col_a, col_b, pa, ra, pb, rb);
                        let pos_end = if has_zero { row + 6 } else { row + 5 };
                        scale_jacobian_block_rows(j_mat, row + 3..pos_end, total_cols, inv_len);
                    }
                    if has_zero { 6 } else { 5 }
                }
                CadJointType::Gear {
                    ratio,
                    axis_a,
                    axis_b,
                } => {
                    let (_, ra_base) = base_poses[&jrec.part_a];
                    let (_, rb_base) = base_poses[&jrec.part_b];
                    let (_, _, ua) = local_orthonormal_basis(*axis_a, Vector3::z());
                    let (_, _, ub) = local_orthonormal_basis(*axis_b, Vector3::z());
                    let dta = rotation_vector(ra * ra_base.transpose()).dot(&(ra_base * ua));
                    let dtb = rotation_vector(rb * rb_base.transpose()).dot(&(rb_base * ub));
                    let r0 = ratio * (base_gears[j_idx].0 + dta) + (base_gears[j_idx].1 + dtb);
                    max_rot_err = max_rot_err.max(r0.abs());
                    phi_phys[row] = r0;
                    phi_scaled[row] = r0;
                    if let Some(ref mut j_mat) = j_mat_opt {
                        jrec.joint_type
                            .fill_jacobian_rows(j_mat, row, col_a, col_b, pa, ra, pb, rb);
                    }
                    1
                }
            };
            row += rows_added;
        }

        if let Some(ref mut j_mat) = j_mat_opt {
            for idx in 0..self.free_parts.len() {
                let base_col = 6 * idx;
                for c in 0..3 {
                    for r in 0..total_rows {
                        j_mat[(r, base_col + c)] *= self.char_len;
                    }
                }
            }
        }

        let phys_norm = phi_phys.norm();
        let scaled_norm = phi_scaled.norm();

        SystemEvalResult {
            phi_scaled,
            phys_norm,
            scaled_norm,
            max_pos_err,
            max_rot_err,
            j_scaled: j_mat_opt,
        }
    }
}

struct SystemEvalResult {
    phi_scaled: nalgebra::DVector<f64>,
    phys_norm: f64,
    scaled_norm: f64,
    max_pos_err: f64,
    max_rot_err: f64,
    j_scaled: Option<DMatrix<f64>>,
}

fn scale_jacobian_block_rows(
    j_mat: &mut DMatrix<f64>,
    rows: std::ops::Range<usize>,
    total_cols: usize,
    factor: f64,
) {
    for r in rows {
        for c in 0..total_cols {
            j_mat[(r, c)] *= factor;
        }
    }
}

fn rotation_vector(r: Matrix3<f64>) -> Vector3<f64> {
    nalgebra::Rotation3::from_matrix_unchecked(r).scaled_axis()
}

fn wrap_angle_pi(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Transforms a `TriangleMesh` by a 3D translation vector and 3x3 rotation matrix.
fn transform_triangle_mesh(
    mesh: &TriangleMesh,
    translation: Vector3<f64>,
    rotation: Matrix3<f64>,
) -> TriangleMesh {
    let is_identity_pos = translation.norm_squared() < 1e-24;
    let is_identity_rot = (rotation - Matrix3::identity()).norm_squared() < 1e-24;
    if is_identity_pos && is_identity_rot {
        return mesh.clone();
    }

    let vertices: Vec<Vertex> = mesh
        .vertices
        .iter()
        .map(|v| {
            let p = Vector3::new(v.position[0], v.position[1], v.position[2]);
            let n = Vector3::new(v.normal[0], v.normal[1], v.normal[2]);
            let tp = rotation * p + translation;
            let tn = rotation * n;
            Vertex::new([tp.x, tp.y, tp.z], [tn.x, tn.y, tn.z])
        })
        .collect();

    TriangleMesh {
        vertices,
        indices: mesh.indices.clone(),
        face_ids: mesh.face_ids.clone(),
        faces: mesh.faces.clone(),
    }
}

/// Computes the minimum translational penetration depth from an intersection solid `inter_mesh`.
fn compute_penetration_depth_from_intersection(
    inter_mesh: &TriangleMesh,
    aabb_a: &BoundingBox,
    aabb_b: &BoundingBox,
) -> f64 {
    if inter_mesh.vertices.is_empty() {
        return 0.0;
    }

    let mut dirs: Vec<Vector3<f64>> = vec![
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];

    let ca = aabb_a.center();
    let cb = aabb_b.center();
    let center_diff = Vector3::new(cb[0] - ca[0], cb[1] - ca[1], cb[2] - ca[2]);
    if center_diff.norm() > 1e-9 {
        dirs.push(center_diff.normalize());
    }

    // Collect up to 48 distinct face normals from the intersection mesh
    for tri in &inter_mesh.indices {
        if dirs.len() >= 52 {
            break;
        }
        let v0 = inter_mesh.vertices[tri[0] as usize].position;
        let v1 = inter_mesh.vertices[tri[1] as usize].position;
        let v2 = inter_mesh.vertices[tri[2] as usize].position;
        let e1 = Vector3::new(v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]);
        let e2 = Vector3::new(v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]);
        let n = e1.cross(&e2);
        let norm = n.norm();
        if norm > 1e-12 {
            let unit_n = n / norm;
            let already_present = dirs.iter().any(|d| d.dot(&unit_n).abs() > 0.999);
            if !already_present {
                dirs.push(unit_n);
            }
        }
    }

    let mut min_span = f64::INFINITY;
    for d in &dirs {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for v in &inter_mesh.vertices {
            let proj = d.x * v.position[0] + d.y * v.position[1] + d.z * v.position[2];
            if proj < lo {
                lo = proj;
            }
            if proj > hi {
                hi = proj;
            }
        }
        let span = (hi - lo).max(0.0);
        if span > 1e-12 && span < min_span {
            min_span = span;
        }
    }

    if min_span.is_finite() {
        min_span
    } else {
        let dims = inter_mesh.bounding_box().dimensions();
        dims[0].min(dims[1]).min(dims[2]).max(0.0)
    }
}

/// Minimum squared Euclidean distance between two axis-aligned bounding boxes.
fn aabb_min_dist_sq(a: &BoundingBox, b: &BoundingBox) -> f64 {
    let mut dist_sq = 0.0_f64;
    for i in 0..3 {
        if a.max[i] < b.min[i] {
            let d = b.min[i] - a.max[i];
            dist_sq += d * d;
        } else if b.max[i] < a.min[i] {
            let d = a.min[i] - b.max[i];
            dist_sq += d * d;
        }
    }
    dist_sq
}

/// Computes the exact minimum Euclidean distance (clearance) between two non-intersecting triangle meshes.
fn compute_mesh_clearance(
    mesh_a: &TriangleMesh,
    aabb_a: &BoundingBox,
    mesh_b: &TriangleMesh,
    aabb_b: &BoundingBox,
) -> f64 {
    if mesh_a.indices.is_empty() || mesh_b.indices.is_empty() {
        return aabb_min_dist_sq(aabb_a, aabb_b).sqrt();
    }

    // Precompute per-triangle vertices and AABBs for mesh A and mesh B
    struct TriData {
        p0: Vector3<f64>,
        p1: Vector3<f64>,
        p2: Vector3<f64>,
        aabb: BoundingBox,
    }

    let build_tris = |mesh: &TriangleMesh| -> Vec<TriData> {
        mesh.indices
            .iter()
            .map(|tri| {
                let v0 = mesh.vertices[tri[0] as usize].position;
                let v1 = mesh.vertices[tri[1] as usize].position;
                let v2 = mesh.vertices[tri[2] as usize].position;
                let min = [
                    v0[0].min(v1[0]).min(v2[0]),
                    v0[1].min(v1[1]).min(v2[1]),
                    v0[2].min(v1[2]).min(v2[2]),
                ];
                let max = [
                    v0[0].max(v1[0]).max(v2[0]),
                    v0[1].max(v1[1]).max(v2[1]),
                    v0[2].max(v1[2]).max(v2[2]),
                ];
                TriData {
                    p0: Vector3::new(v0[0], v0[1], v0[2]),
                    p1: Vector3::new(v1[0], v1[1], v1[2]),
                    p2: Vector3::new(v2[0], v2[1], v2[2]),
                    aabb: BoundingBox::new(min, max),
                }
            })
            .collect()
    };

    let mut tris_a = build_tris(mesh_a);
    let tris_b = build_tris(mesh_b);

    // Sort triangles of A by their AABB distance to B's overall AABB so we find a tight upper bound immediately
    tris_a.sort_by(|ta, tb| {
        let da = aabb_min_dist_sq(&ta.aabb, aabb_b);
        let db = aabb_min_dist_sq(&tb.aabb, aabb_b);
        da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
    });

    let global_lower_bound_sq = aabb_min_dist_sq(aabb_a, aabb_b);
    let mut best_dist_sq = f64::INFINITY;

    for ta in &tris_a {
        let lower_a = aabb_min_dist_sq(&ta.aabb, aabb_b);
        if lower_a >= best_dist_sq {
            // Because tris_a is sorted by lower_a, no subsequent triangle in A can improve best_dist_sq
            break;
        }
        for tb in &tris_b {
            if aabb_min_dist_sq(&ta.aabb, &tb.aabb) >= best_dist_sq {
                continue;
            }
            let d_sq = tri_tri_dist_sq(ta.p0, ta.p1, ta.p2, tb.p0, tb.p1, tb.p2);
            if d_sq < best_dist_sq {
                best_dist_sq = d_sq;
                if best_dist_sq <= global_lower_bound_sq + 1e-24 {
                    return best_dist_sq.max(0.0).sqrt();
                }
            }
        }
    }

    best_dist_sq.max(0.0).sqrt()
}

/// Squared Euclidean distance between two 3D triangles `(a0, a1, a2)` and `(b0, b1, b2)`.
fn tri_tri_dist_sq(
    a0: Vector3<f64>,
    a1: Vector3<f64>,
    a2: Vector3<f64>,
    b0: Vector3<f64>,
    b1: Vector3<f64>,
    b2: Vector3<f64>,
) -> f64 {
    let mut best = point_triangle_dist_sq(a0, b0, b1, b2)
        .min(point_triangle_dist_sq(a1, b0, b1, b2))
        .min(point_triangle_dist_sq(a2, b0, b1, b2))
        .min(point_triangle_dist_sq(b0, a0, a1, a2))
        .min(point_triangle_dist_sq(b1, a0, a1, a2))
        .min(point_triangle_dist_sq(b2, a0, a1, a2));

    let edges_a = [(a0, a1), (a1, a2), (a2, a0)];
    let edges_b = [(b0, b1), (b1, b2), (b2, b0)];
    for &(pa, qa) in &edges_a {
        for &(pb, qb) in &edges_b {
            let d = segment_segment_dist_sq(pa, qa, pb, qb);
            if d < best {
                best = d;
            }
        }
    }
    best
}

/// Squared Euclidean distance from point `p` to 3D triangle `(a, b, c)` (Ericson §5.1.5).
fn point_triangle_dist_sq(
    p: Vector3<f64>,
    a: Vector3<f64>,
    b: Vector3<f64>,
    c: Vector3<f64>,
) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.norm_squared();
    }

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return bp.norm_squared();
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        let v = if denom.abs() > 1e-30 { d1 / denom } else { 0.0 };
        let closest = a + v * ab;
        return (p - closest).norm_squared();
    }

    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return cp.norm_squared();
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        let w = if denom.abs() > 1e-30 { d2 / denom } else { 0.0 };
        let closest = a + w * ac;
        return (p - closest).norm_squared();
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let bc = c - b;
        let denom = (d4 - d3) + (d5 - d6);
        let w = if denom.abs() > 1e-30 {
            (d4 - d3) / denom
        } else {
            0.0
        };
        let closest = b + w * bc;
        return (p - closest).norm_squared();
    }

    let denom = va + vb + vc;
    if denom.abs() <= 1e-30 {
        return ap.norm_squared();
    }
    let inv_denom = 1.0 / denom;
    let v = vb * inv_denom;
    let w = vc * inv_denom;
    let closest = a + ab * v + ac * w;
    (p - closest).norm_squared()
}

/// Squared Euclidean distance between two 3D line segments `[p1, q1]` and `[p2, q2]` (Ericson §5.1.9).
fn segment_segment_dist_sq(
    p1: Vector3<f64>,
    q1: Vector3<f64>,
    p2: Vector3<f64>,
    q2: Vector3<f64>,
) -> f64 {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);
    let eps = 1e-18;

    let (s, t) = if a <= eps && e <= eps {
        (0.0, 0.0)
    } else if a <= eps {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(&r);
        if e <= eps {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(&d2);
            let denom = a * e - b * b;
            let mut s = if denom.abs() > eps {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };

    let c1 = p1 + d1 * s;
    let c2 = p2 + d2 * t;
    (c1 - c2).norm_squared()
}

/// Identifies all joints whose constraint rows are linearly dependent with other constraints
/// via leave-one-joint-out SVD rank comparison at `rank_tol` (`P3-T15`).
fn find_redundant_joint_ids(
    j_mat: &DMatrix<f64>,
    joint_row_spans: &[(&CadJointRecord, usize, usize)],
    total_cols: usize,
    full_rank: usize,
    rank_tol: f64,
) -> Vec<String> {
    let total_rows = j_mat.nrows();
    let mut redundant_ids = Vec::new();

    for &(joint_rec, start_row, rows_k) in joint_row_spans {
        let remaining_rows = total_rows.saturating_sub(rows_k);
        let rank_without_k = if remaining_rows == 0 {
            0
        } else {
            let mut j_sub = DMatrix::<f64>::zeros(remaining_rows, total_cols);
            let mut dst_row = 0;
            for r in 0..total_rows {
                if r >= start_row && r < start_row + rows_k {
                    continue;
                }
                j_sub.row_mut(dst_row).copy_from(&j_mat.row(r));
                dst_row += 1;
            }
            let svd_sub = j_sub.svd(false, false);
            svd_sub
                .singular_values
                .iter()
                .filter(|&&s| s > rank_tol)
                .count()
        };

        let dep_rows = (rows_k as i32) - ((full_rank as i32) - (rank_without_k as i32));
        if dep_rows > 0 {
            redundant_ids.push(joint_rec.id.clone());
        }
    }

    redundant_ids
}

/// Evaluates whether a `Revolute` or `Prismatic` joint is currently at or beyond its travel `limits` (`P3-T15`).
fn evaluate_joint_limit_status(
    joint: &CadJointRecord,
    pos_a: Vector3<f64>,
    rot_a: Matrix3<f64>,
    pos_b: Vector3<f64>,
    rot_b: Matrix3<f64>,
) -> Option<(JointLimitStatus, String)> {
    let (is_revolute, limits) = match joint.joint_type {
        CadJointType::Revolute {
            limits: Some(lim), ..
        } => (true, lim),
        CadJointType::Prismatic {
            limits: Some(lim), ..
        } => (false, lim),
        _ => return None,
    };

    let q = joint
        .joint_type
        .current_coordinate(pos_a, rot_a, pos_b, rot_b)?;
    let [min_v, max_v] = limits;
    let span = (max_v - min_v).abs();
    let tol = (span * 1e-3).clamp(1e-6, 1e-3);
    let unit = if is_revolute { "rad" } else { "m" };

    let (boundary, motor_blocked, warning) = if is_zero_range_limits(Some(limits)) {
        let warn = if is_revolute {
            format!(
                "Достигнута граница limits для шарнира '{}': нулевой диапазон хода [{:.2}, {:.2}] deg (заблокирован)",
                joint.id,
                min_v.to_degrees(),
                max_v.to_degrees()
            )
        } else {
            format!(
                "Достигнута граница limits для шарнира '{}': нулевой диапазон хода [{:.4}, {:.4}] m (заблокирован)",
                joint.id, min_v, max_v
            )
        };
        ("locked".to_string(), true, warn)
    } else if q < min_v - tol || (q - min_v).abs() <= tol {
        let boundary = if q < min_v - tol {
            "below_min".to_string()
        } else {
            "min".to_string()
        };
        let blocked = joint.motor.is_some_and(|m| m.target_velocity < -1e-9);
        let suffix = if blocked {
            "; привод направлен в упор"
        } else {
            ""
        };
        let warn = if is_revolute {
            format!(
                "Шарнир '{}' достиг нижней границы limits ({:.2} deg <= {:.2} deg){}",
                joint.id,
                q.to_degrees(),
                min_v.to_degrees(),
                suffix
            )
        } else {
            format!(
                "Шарнир '{}' достиг нижней границы limits ({:.4} m <= {:.4} m){}",
                joint.id, q, min_v, suffix
            )
        };
        (boundary, blocked, warn)
    } else if q > max_v + tol || (q - max_v).abs() <= tol {
        let boundary = if q > max_v + tol {
            "above_max".to_string()
        } else {
            "max".to_string()
        };
        let blocked = joint.motor.is_some_and(|m| m.target_velocity > 1e-9);
        let suffix = if blocked {
            "; привод направлен в упор"
        } else {
            ""
        };
        let warn = if is_revolute {
            format!(
                "Шарнир '{}' достиг верхней границы limits ({:.2} deg >= {:.2} deg){}",
                joint.id,
                q.to_degrees(),
                max_v.to_degrees(),
                suffix
            )
        } else {
            format!(
                "Шарнир '{}' достиг верхней границы limits ({:.4} m >= {:.4} m){}",
                joint.id, q, max_v, suffix
            )
        };
        (boundary, blocked, warn)
    } else {
        return None;
    };

    Some((
        JointLimitStatus {
            joint_id: joint.id.clone(),
            joint_type: joint.joint_type.type_name().to_string(),
            boundary,
            current_value: q,
            limits,
            unit: unit.to_string(),
            motor_blocked,
        },
        warning,
    ))
}


