//! Physics simulation world managing Rapier3D pipelines, bodies, colliders, and joints.

use std::collections::HashMap;

use mcp_cad_core::Part;
use nalgebra::Vector3;
use rapier3d::crossbeam::channel::{Receiver, Sender};
use rapier3d::prelude::*;

use crate::error::PhysicsError;
use crate::joint::{CadJointType, MotorConfig};
use crate::rigid_body::{ColliderType, create_collider_builder, create_rigid_body_builder};
use crate::telemetry::{
    BodyState, BodyTelemetry, CollisionEventType, CollisionRecord, JointReactionForce,
    SimulationSummary,
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
    /// Rapier impulse joint handle.
    pub handle: ImpulseJointHandle,
}

/// Kinematic gear constraint coupling two rotating bodies.
#[derive(Debug, Clone)]
pub struct GearConstraintRecord {
    pub id: String,
    pub part_a: String,
    pub part_b: String,
    pub ratio: f64,
    pub axis_a: Vector3<f64>,
    pub axis_b: Vector3<f64>,
}

/// The main physics simulation world wrapping Rapier3D.
pub struct PhysicsWorld {
    pipeline: PhysicsPipeline,
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,

    // Channels for capturing Rapier collision and contact force events
    collision_sender: Sender<CollisionEvent>,
    collision_receiver: Receiver<CollisionEvent>,
    contact_force_sender: Sender<ContactForceEvent>,
    contact_force_receiver: Receiver<ContactForceEvent>,

    // Mappings between CAD part IDs, joints, and Rapier handles
    part_to_body: HashMap<String, RigidBodyHandle>,
    body_to_part: HashMap<RigidBodyHandle, String>,
    collider_to_part: HashMap<ColliderHandle, String>,
    part_to_colliders: HashMap<String, Vec<ColliderHandle>>,
    joints: HashMap<String, CadJointRecord>,
    gear_constraints: HashMap<String, GearConstraintRecord>,

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
    pub fn new() -> Self {
        Self::with_gravity([0.0, -9.81, 0.0])
    }

    /// Creates a new physics world with custom gravity acceleration vector [gx, gy, gz].
    pub fn with_gravity(gravity: [f64; 3]) -> Self {
        let (collision_sender, collision_receiver) = rapier3d::crossbeam::channel::unbounded();
        let (contact_force_sender, contact_force_receiver) =
            rapier3d::crossbeam::channel::unbounded();

        let integration_parameters = IntegrationParameters {
            dt: 1.0 / 60.0,
            ..Default::default()
        };

        let mut bodies = RigidBodySet::new();
        // Insert a static anchor rigid body for "ground" and "world" references
        let ground_handle = bodies.insert(RigidBodyBuilder::fixed());
        let mut part_to_body = HashMap::new();
        let mut body_to_part = HashMap::new();
        part_to_body.insert("ground".to_string(), ground_handle);
        part_to_body.insert("world".to_string(), ground_handle);
        body_to_part.insert(ground_handle, "ground".to_string());

        Self {
            pipeline: PhysicsPipeline::new(),
            gravity: vector![gravity[0] as Real, gravity[1] as Real, gravity[2] as Real],
            integration_parameters,
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies,
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),

            collision_sender,
            collision_receiver,
            contact_force_sender,
            contact_force_receiver,

            part_to_body,
            body_to_part,
            collider_to_part: HashMap::new(),
            part_to_colliders: HashMap::new(),
            joints: HashMap::new(),
            gear_constraints: HashMap::new(),

            current_time_s: 0.0,
        }
    }

    /// Sets the gravity acceleration vector [gx, gy, gz].
    pub fn set_gravity(&mut self, gravity: [f64; 3]) {
        self.gravity = vector![gravity[0] as Real, gravity[1] as Real, gravity[2] as Real];
    }

    /// Gets the current gravity acceleration vector [gx, gy, gz].
    pub fn gravity(&self) -> [f64; 3] {
        [
            self.gravity.x as f64,
            self.gravity.y as f64,
            self.gravity.z as f64,
        ]
    }

    /// Current accumulated simulation time in seconds.
    pub fn current_time(&self) -> f64 {
        self.current_time_s
    }

    /// Resets accumulated simulation time to zero.
    pub fn reset_time(&mut self) {
        self.current_time_s = 0.0;
    }

    /// Adds a CAD [`Part`] into the simulation world.
    ///
    /// For dynamic bodies, defaults to `ColliderType::ConvexHull`.
    /// For static/fixed bodies, defaults to `ColliderType::TriMesh`.
    pub fn add_part(&mut self, part: &Part, is_dynamic: bool) -> Result<RigidBodyHandle, PhysicsError> {
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
    ) -> Result<RigidBodyHandle, PhysicsError> {
        if self.part_to_body.contains_key(&part.id) {
            return Err(PhysicsError::PartAlreadyExists(part.id.clone()));
        }

        let rb_builder = create_rigid_body_builder(part, is_dynamic, initial_position, initial_rotation);
        let collider_builder = create_collider_builder(part, collider_type)?;

        let rb_handle = self.bodies.insert(rb_builder);
        let collider_handle = self.colliders.insert_with_parent(
            collider_builder,
            rb_handle,
            &mut self.bodies,
        );

        self.part_to_body.insert(part.id.clone(), rb_handle);
        self.body_to_part.insert(rb_handle, part.id.clone());
        self.collider_to_part.insert(collider_handle, part.id.clone());
        self.part_to_colliders
            .entry(part.id.clone())
            .or_default()
            .push(collider_handle);

        Ok(rb_handle)
    }

    /// Removes a CAD part, its colliders, and all attached joints from the world.
    pub fn remove_part(&mut self, part_id: &str) -> Result<(), PhysicsError> {
        let rb_handle = self
            .part_to_body
            .remove(part_id)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))?;

        self.body_to_part.remove(&rb_handle);

        if let Some(col_handles) = self.part_to_colliders.remove(part_id) {
            for col_h in col_handles {
                self.collider_to_part.remove(&col_h);
            }
        }

        // Clean up attached joints in self.joints
        let attached_joint_ids: Vec<String> = self
            .joints
            .iter()
            .filter(|(_, rec)| rec.part_a == part_id || rec.part_b == part_id)
            .map(|(id, _)| id.clone())
            .collect();

        for j_id in attached_joint_ids {
            self.joints.remove(&j_id);
            self.gear_constraints.remove(&j_id);
        }
        self.gear_constraints.retain(|_, rec| rec.part_a != part_id && rec.part_b != part_id);

        self.bodies.remove(
            rb_handle,
            &mut self.island_manager,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );

        Ok(())
    }

    /// Adds a kinematic/dynamic joint between two parts.
    pub fn add_joint(
        &mut self,
        joint_id: impl Into<String>,
        part_a: &str,
        part_b: &str,
        joint_type: CadJointType,
        motor: Option<MotorConfig>,
    ) -> Result<ImpulseJointHandle, PhysicsError> {
        let j_id = joint_id.into();
        if self.joints.contains_key(&j_id) {
            return Err(PhysicsError::JointAlreadyExists(j_id));
        }

        let handle_a = *self
            .part_to_body
            .get(part_a)
            .ok_or_else(|| PhysicsError::PartNotFound(part_a.to_string()))?;

        let handle_b = *self
            .part_to_body
            .get(part_b)
            .ok_or_else(|| PhysicsError::PartNotFound(part_b.to_string()))?;

        let generic_joint = joint_type.build_generic_joint(motor)?;
        let joint_handle = self
            .impulse_joints
            .insert(handle_a, handle_b, generic_joint, true);

        self.joints.insert(
            j_id.clone(),
            CadJointRecord {
                id: j_id.clone(),
                part_a: part_a.to_string(),
                part_b: part_b.to_string(),
                joint_type: joint_type.clone(),
                handle: joint_handle,
            },
        );

        if let CadJointType::Gear { ratio, axis_a, axis_b } = &joint_type {
            self.gear_constraints.insert(
                j_id.clone(),
                GearConstraintRecord {
                    id: j_id,
                    part_a: part_a.to_string(),
                    part_b: part_b.to_string(),
                    ratio: *ratio,
                    axis_a: Vector3::new(axis_a[0], axis_a[1], axis_a[2]),
                    axis_b: Vector3::new(axis_b[0], axis_b[1], axis_b[2]),
                },
            );
        }

        Ok(joint_handle)
    }

    /// Updates motor actuation on an existing joint.
    pub fn set_joint_motor(
        &mut self,
        joint_id: &str,
        motor: MotorConfig,
    ) -> Result<(), PhysicsError> {
        let record = self
            .joints
            .get(joint_id)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        let joint = self
            .impulse_joints
            .get_mut(record.handle)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        record.joint_type.apply_motor(&mut joint.data, motor)
    }

    /// Disables motor actuation on an existing joint.
    pub fn disable_joint_motor(&mut self, joint_id: &str) -> Result<(), PhysicsError> {
        let record = self
            .joints
            .get(joint_id)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        let joint = self
            .impulse_joints
            .get_mut(record.handle)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        record.joint_type.disable_motor(&mut joint.data)
    }

    /// Removes a joint by identifier.
    pub fn remove_joint(&mut self, joint_id: &str) -> Result<(), PhysicsError> {
        let record = self
            .joints
            .remove(joint_id)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        self.impulse_joints.remove(record.handle, true);
        self.gear_constraints.remove(joint_id);
        Ok(())
    }

    /// Executes a single discrete physics integration step of size `dt` seconds.
    ///
    /// Returns any collision events that started or stopped during this step.
    pub fn step(&mut self, dt: f64) -> Vec<CollisionRecord> {
        let step_dt = if dt > 0.0 { dt } else { 1.0 / 60.0 };
        self.integration_parameters.dt = step_dt as Real;

        let event_handler = ChannelEventCollector::new(
            self.collision_sender.clone(),
            self.contact_force_sender.clone(),
        );

        let physics_hooks = ();

        self.pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            None,
            &physics_hooks,
            &event_handler,
        );

        // Enforce kinematic gear constraints: w_b = -ratio * w_a
        for gear in self.gear_constraints.values() {
            let Some(&handle_a) = self.part_to_body.get(&gear.part_a) else { continue };
            let Some(&handle_b) = self.part_to_body.get(&gear.part_b) else { continue };

            let (Some(rb_a), Some(rb_b)) = (self.bodies.get(handle_a), self.bodies.get(handle_b)) else { continue };

            let w_a = rb_a.angvel();
            let w_b = rb_b.angvel();

            let axis_a_norm = if gear.axis_a.norm() > 1e-6 {
                gear.axis_a.normalize()
            } else {
                Vector3::z()
            };
            let axis_b_norm = if gear.axis_b.norm() > 1e-6 {
                gear.axis_b.normalize()
            } else {
                Vector3::z()
            };

            let wa_proj = w_a.x as f64 * axis_a_norm.x + w_a.y as f64 * axis_a_norm.y + w_a.z as f64 * axis_a_norm.z;
            let wb_proj = w_b.x as f64 * axis_b_norm.x + w_b.y as f64 * axis_b_norm.y + w_b.z as f64 * axis_b_norm.z;

            if wa_proj.abs() > 1e-5 || wb_proj.abs() <= 1e-5 {
                let target_wb = -gear.ratio * wa_proj;
                let delta_wb = target_wb - wb_proj;
                let delta_angvel = vector![
                    (delta_wb * axis_b_norm.x) as Real,
                    (delta_wb * axis_b_norm.y) as Real,
                    (delta_wb * axis_b_norm.z) as Real
                ];

                if let Some(rb_b_mut) = self.bodies.get_mut(handle_b) {
                    if rb_b_mut.is_dynamic() {
                        let new_w = rb_b_mut.angvel() + delta_angvel;
                        rb_b_mut.set_angvel(new_w, true);
                    }
                }
            } else {
                let target_wa = -wb_proj / gear.ratio;
                let delta_wa = target_wa - wa_proj;
                let delta_angvel = vector![
                    (delta_wa * axis_a_norm.x) as Real,
                    (delta_wa * axis_a_norm.y) as Real,
                    (delta_wa * axis_a_norm.z) as Real
                ];

                if let Some(rb_a_mut) = self.bodies.get_mut(handle_a) {
                    if rb_a_mut.is_dynamic() {
                        let new_w = rb_a_mut.angvel() + delta_angvel;
                        rb_a_mut.set_angvel(new_w, true);
                    }
                }
            }
        }

        self.current_time_s += step_dt;

        // Collect contact forces to correlate with collision records
        let mut contact_force_map: HashMap<(ColliderHandle, ColliderHandle), (Vector3<f64>, f64)> =
            HashMap::new();

        while let Ok(cf) = self.contact_force_receiver.try_recv() {
            let norm = Vector3::new(
                cf.max_force_direction.x as f64,
                cf.max_force_direction.y as f64,
                cf.max_force_direction.z as f64,
            );
            let mag = cf.max_force_magnitude as f64;
            contact_force_map.insert((cf.collider1, cf.collider2), (norm, mag));
        }

        // Drain collision events
        let mut records = Vec::new();

        while let Ok(event) = self.collision_receiver.try_recv() {
            let (c1, c2, event_type) = match event {
                CollisionEvent::Started(c1, c2, _) => (c1, c2, CollisionEventType::Started),
                CollisionEvent::Stopped(c1, c2, _) => (c1, c2, CollisionEventType::Stopped),
            };

            let body_a = self
                .collider_to_part
                .get(&c1)
                .cloned()
                .unwrap_or_else(|| format!("{c1:?}"));
            let body_b = self
                .collider_to_part
                .get(&c2)
                .cloned()
                .unwrap_or_else(|| format!("{c2:?}"));

            let (contact_normal, max_impulse) = if let Some((norm, mag)) =
                contact_force_map.get(&(c1, c2)).or_else(|| contact_force_map.get(&(c2, c1)))
            {
                (Some([norm.x, norm.y, norm.z]), Some(*mag))
            } else {
                (None, None)
            };

            records.push(CollisionRecord {
                time_s: self.current_time_s,
                body_a,
                body_b,
                event_type,
                contact_normal,
                max_impulse,
            });
        }

        records
    }

    /// Records an instantaneous telemetry snapshot for all bodies and joints.
    fn record_telemetry_snapshot(
        &self,
        summary: &mut SimulationSummary,
        time_s: f64,
        dt: f64,
    ) {
        // Bodies
        for (part_id, &rb_handle) in &self.part_to_body {
            if part_id == "ground" || part_id == "world" {
                continue;
            }
            if let Some(rb) = self.bodies.get(rb_handle) {
                let pos = rb.translation();
                let q = rb.rotation().quaternion();
                let lv = rb.linvel();
                let av = rb.angvel();

                let state = BodyState {
                    time_s,
                    position: [pos.x as f64, pos.y as f64, pos.z as f64],
                    orientation_quat: [
                        q.coords.x as f64,
                        q.coords.y as f64,
                        q.coords.z as f64,
                        q.coords.w as f64,
                    ],
                    linear_velocity: [lv.x as f64, lv.y as f64, lv.z as f64],
                    angular_velocity: [av.x as f64, av.y as f64, av.z as f64],
                };

                if let Some(traj) = summary.trajectories.get_mut(part_id) {
                    traj.push_sample(state);
                }
            }
        }

        // Joints
        let safe_dt = if dt > 0.0 { dt } else { 1.0 / 60.0 };
        for (joint_id, joint_rec) in &self.joints {
            if let Some(joint) = self.impulse_joints.get(joint_rec.handle) {
                let imp = joint.impulses;
                let force = [
                    imp[0] as f64 / safe_dt,
                    imp[1] as f64 / safe_dt,
                    imp[2] as f64 / safe_dt,
                ];
                let torque = [
                    imp[3] as f64 / safe_dt,
                    imp[4] as f64 / safe_dt,
                    imp[5] as f64 / safe_dt,
                ];
                let force_mag = (force[0].powi(2) + force[1].powi(2) + force[2].powi(2)).sqrt();
                let torque_mag =
                    (torque[0].powi(2) + torque[1].powi(2) + torque[2].powi(2)).sqrt();

                let reaction = JointReactionForce {
                    joint_id: joint_id.clone(),
                    time_s,
                    force,
                    torque,
                    force_magnitude: force_mag,
                    torque_magnitude: torque_mag,
                };

                if let Some(list) = summary.joint_reactions.get_mut(joint_id) {
                    list.push(reaction);
                }

                if let Some(peak) = summary.peak_joint_forces.get_mut(joint_id) {
                    if force_mag > *peak {
                        *peak = force_mag;
                    }
                }
            }
        }
    }

    /// Simulates physics forward for `duration_s` with integration step `step_size_s`.
    ///
    /// Produces a comprehensive [`SimulationSummary`] containing trajectories, joint reactions,
    /// and collision events tailored for AI agent analysis.
    pub fn simulate(&mut self, duration_s: f64, step_size_s: f64) -> SimulationSummary {
        let dt = if step_size_s > 0.0 {
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
            summary.collisions.extend(collisions);
            self.record_telemetry_snapshot(&mut summary, self.current_time_s, dt);
        }

        summary.step_count = steps;
        summary
    }

    // --- Query and inspection methods ---

    fn get_body(&self, part_id: &str) -> Result<&RigidBody, PhysicsError> {
        let handle = self
            .part_to_body
            .get(part_id)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))?;
        self.bodies
            .get(*handle)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))
    }

    fn get_body_mut(&mut self, part_id: &str) -> Result<&mut RigidBody, PhysicsError> {
        let handle = self
            .part_to_body
            .get(part_id)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))?;
        self.bodies
            .get_mut(*handle)
            .ok_or_else(|| PhysicsError::PartNotFound(part_id.to_string()))
    }

    /// Retrieves current 3D position [x, y, z] of a part.
    pub fn part_position(&self, part_id: &str) -> Result<[f64; 3], PhysicsError> {
        let rb = self.get_body(part_id)?;
        let pos = rb.translation();
        Ok([pos.x as f64, pos.y as f64, pos.z as f64])
    }

    /// Retrieves current orientation quaternion [x, y, z, w] of a part.
    pub fn part_orientation(&self, part_id: &str) -> Result<[f64; 4], PhysicsError> {
        let rb = self.get_body(part_id)?;
        let q = rb.rotation().quaternion();
        Ok([
            q.coords.x as f64,
            q.coords.y as f64,
            q.coords.z as f64,
            q.coords.w as f64,
        ])
    }

    /// Retrieves current linear velocity [vx, vy, vz] of a part.
    pub fn part_velocity(&self, part_id: &str) -> Result<[f64; 3], PhysicsError> {
        let rb = self.get_body(part_id)?;
        let lv = rb.linvel();
        Ok([lv.x as f64, lv.y as f64, lv.z as f64])
    }

    /// Retrieves current angular velocity [wx, wy, wz] of a part.
    pub fn part_angular_velocity(&self, part_id: &str) -> Result<[f64; 3], PhysicsError> {
        let rb = self.get_body(part_id)?;
        let av = rb.angvel();
        Ok([av.x as f64, av.y as f64, av.z as f64])
    }

    /// Sets linear velocity [vx, vy, vz] on a part.
    pub fn set_part_velocity(
        &mut self,
        part_id: &str,
        linvel: [f64; 3],
    ) -> Result<(), PhysicsError> {
        let rb = self.get_body_mut(part_id)?;
        rb.set_linvel(
            vector![linvel[0] as Real, linvel[1] as Real, linvel[2] as Real],
            true,
        );
        Ok(())
    }

    /// Sets angular velocity [wx, wy, wz] on a part.
    pub fn set_part_angular_velocity(
        &mut self,
        part_id: &str,
        angvel: [f64; 3],
    ) -> Result<(), PhysicsError> {
        let rb = self.get_body_mut(part_id)?;
        rb.set_angvel(
            vector![angvel[0] as Real, angvel[1] as Real, angvel[2] as Real],
            true,
        );
        Ok(())
    }

    /// Applies a continuous external force vector [fx, fy, fz] in Newtons to a part.
    pub fn apply_force(&mut self, part_id: &str, force: [f64; 3]) -> Result<(), PhysicsError> {
        let rb = self.get_body_mut(part_id)?;
        rb.add_force(
            vector![force[0] as Real, force[1] as Real, force[2] as Real],
            true,
        );
        Ok(())
    }

    /// Applies an instantaneous impulse vector [ix, iy, iz] in N*s to a part.
    pub fn apply_impulse(&mut self, part_id: &str, impulse: [f64; 3]) -> Result<(), PhysicsError> {
        let rb = self.get_body_mut(part_id)?;
        rb.apply_impulse(
            vector![impulse[0] as Real, impulse[1] as Real, impulse[2] as Real],
            true,
        );
        Ok(())
    }

    /// Computes the reaction force on a joint over the last integration step `dt`.
    pub fn joint_reaction_force(
        &self,
        joint_id: &str,
        dt: f64,
    ) -> Result<JointReactionForce, PhysicsError> {
        let record = self
            .joints
            .get(joint_id)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        let joint = self
            .impulse_joints
            .get(record.handle)
            .ok_or_else(|| PhysicsError::JointNotFound(joint_id.to_string()))?;

        let safe_dt = if dt > 0.0 { dt } else { 1.0 / 60.0 };
        let imp = joint.impulses;
        let force = [
            imp[0] as f64 / safe_dt,
            imp[1] as f64 / safe_dt,
            imp[2] as f64 / safe_dt,
        ];
        let torque = [
            imp[3] as f64 / safe_dt,
            imp[4] as f64 / safe_dt,
            imp[5] as f64 / safe_dt,
        ];
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
}
