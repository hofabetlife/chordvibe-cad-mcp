//! Simulation telemetry: body states, joint forces, collision records, and AI summaries.

use std::collections::HashMap;
use serde::{Deserialize, Serialize};

/// State snapshot of a rigid body at a specific point in time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyState {
    /// Timestamp in seconds from start of simulation.
    pub time_s: f64,
    /// 3D position [x, y, z] in meters.
    pub position: [f64; 3],
    /// Orientation quaternion [x, y, z, w].
    pub orientation_quat: [f64; 4],
    /// Linear velocity [vx, vy, vz] in m/s.
    pub linear_velocity: [f64; 3],
    /// Angular velocity [wx, wy, wz] in rad/s.
    pub angular_velocity: [f64; 3],
}

/// Recorded trajectory and kinematics telemetry for an individual part.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyTelemetry {
    /// Identifier of the CAD part.
    pub part_id: String,
    /// Time-series state samples.
    pub samples: Vec<BodyState>,
}

impl BodyTelemetry {
    /// Creates a new telemetry log for a part.
    pub fn new(part_id: impl Into<String>) -> Self {
        Self {
            part_id: part_id.into(),
            samples: Vec::new(),
        }
    }

    /// Appends a new time step sample.
    pub fn push_sample(&mut self, state: BodyState) {
        self.samples.push(state);
    }

    /// Final position [x, y, z] reached at the end of the simulation.
    pub fn final_position(&self) -> Option<[f64; 3]> {
        self.samples.last().map(|s| s.position)
    }

    /// Final linear velocity [vx, vy, vz] at the end of the simulation.
    pub fn final_linear_velocity(&self) -> Option<[f64; 3]> {
        self.samples.last().map(|s| s.linear_velocity)
    }

    /// Final angular velocity [wx, wy, wz] at the end of the simulation.
    pub fn final_angular_velocity(&self) -> Option<[f64; 3]> {
        self.samples.last().map(|s| s.angular_velocity)
    }

    /// Peak linear speed (magnitude of linear velocity) observed across all samples.
    pub fn peak_linear_speed(&self) -> f64 {
        self.samples
            .iter()
            .map(|s| {
                (s.linear_velocity[0].powi(2)
                    + s.linear_velocity[1].powi(2)
                    + s.linear_velocity[2].powi(2))
                .sqrt()
            })
            .fold(0.0, f64::max)
    }

    /// Peak angular speed (magnitude of angular velocity) observed across all samples.
    pub fn peak_angular_speed(&self) -> f64 {
        self.samples
            .iter()
            .map(|s| {
                (s.angular_velocity[0].powi(2)
                    + s.angular_velocity[1].powi(2)
                    + s.angular_velocity[2].powi(2))
                .sqrt()
            })
            .fold(0.0, f64::max)
    }
}

/// Reaction force and torque acting on a joint at a specific time step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointReactionForce {
    /// Identifier of the CAD joint.
    pub joint_id: String,
    /// Timestamp in seconds.
    pub time_s: f64,
    /// Linear reaction force vector [Fx, Fy, Fz] in Newtons.
    pub force: [f64; 3],
    /// Reaction torque vector [Tx, Ty, Tz] in N*m.
    pub torque: [f64; 3],
    /// Scalar magnitude of reaction force ||F|| in Newtons.
    pub force_magnitude: f64,
    /// Scalar magnitude of reaction torque ||T|| in N*m.
    pub torque_magnitude: f64,
}

/// Type of collision lifecycle event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollisionEventType {
    /// Contact between bodies has commenced.
    Started,
    /// Contact between bodies has ceased.
    Stopped,
}

/// Record of an impact or contact between two CAD parts.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CollisionRecord {
    /// Timestamp in seconds.
    pub time_s: f64,
    /// First body involved in collision.
    pub body_a: String,
    /// Second body involved in collision.
    pub body_b: String,
    /// Lifecycle phase of contact.
    pub event_type: CollisionEventType,
    /// Normal vector at contact if available.
    pub contact_normal: Option<[f64; 3]>,
    /// Estimated impact force or impulse magnitude in N or N*s.
    pub max_impulse: Option<f64>,
}

/// Comprehensive summary of a completed physics simulation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimulationSummary {
    /// True if the simulation completed without instability or divergence.
    pub success: bool,
    /// Total duration simulated in seconds.
    pub duration_s: f64,
    /// Total number of discrete integration steps taken.
    pub step_count: usize,
    /// Discrete integration time step size in seconds.
    pub step_size_s: f64,
    /// Trajectory telemetry per part id.
    pub trajectories: HashMap<String, BodyTelemetry>,
    /// Maximum reaction force (N) recorded for each joint id.
    pub peak_joint_forces: HashMap<String, f64>,
    /// History of reaction forces on joints.
    pub joint_reactions: HashMap<String, Vec<JointReactionForce>>,
    /// Chronological log of collision events.
    pub collisions: Vec<CollisionRecord>,
}

impl SimulationSummary {
    /// Creates a new empty simulation summary.
    pub fn new(duration_s: f64, step_size_s: f64) -> Self {
        Self {
            success: true,
            duration_s,
            step_count: 0,
            step_size_s,
            trajectories: HashMap::new(),
            peak_joint_forces: HashMap::new(),
            joint_reactions: HashMap::new(),
            collisions: Vec::new(),
        }
    }

    /// Retrieves telemetry for a part by id.
    pub fn body_telemetry(&self, part_id: &str) -> Option<&BodyTelemetry> {
        self.trajectories.get(part_id)
    }

    /// Retrieves the final position of a part by id.
    pub fn final_position(&self, part_id: &str) -> Option<[f64; 3]> {
        self.trajectories.get(part_id).and_then(|t| t.final_position())
    }

    /// Retrieves the final linear velocity of a part by id.
    pub fn final_linear_velocity(&self, part_id: &str) -> Option<[f64; 3]> {
        self.trajectories.get(part_id).and_then(|t| t.final_linear_velocity())
    }

    /// Returns true if any collisions were logged during the simulation.
    pub fn has_collisions(&self) -> bool {
        !self.collisions.is_empty()
    }

    /// Retrieves all collision records involving a specific part id.
    pub fn collisions_for_part<'a>(&'a self, part_id: &'a str) -> impl Iterator<Item = &'a CollisionRecord> {
        self.collisions
            .iter()
            .filter(move |c| c.body_a == part_id || c.body_b == part_id)
    }
}

/// Selector for choosing which simulation timestep to extract load cases from (`P3-T6`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum LoadCaseSelector {
    /// Timestep where the total reaction force on the part is maximal (`"peak_force"`).
    PeakForce,
    /// Timestep where the total reaction torque on the part is maximal (`"peak_torque"`).
    PeakTorque,
    /// Specific simulation timestamp `time_s` in seconds (selects closest sample).
    AtTime(f64),
}

impl LoadCaseSelector {
    /// Human-readable label for the selector mode.
    #[must_use]
    pub fn label(&self) -> String {
        match self {
            Self::PeakForce => "peak_force".to_string(),
            Self::PeakTorque => "peak_torque".to_string(),
            Self::AtTime(t) => format!("time_s:{t:.6}"),
        }
    }
}

/// Reaction load exerted by a single connected joint on the target part,
/// expressed in the target part's local coordinate system (`P3-T6`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointLoadCase {
    /// Identifier of the CAD joint.
    pub joint_id: String,
    /// Canonical joint type (`"fixed"`, `"revolute"`, `"prismatic"`, `"spherical"`, `"gear"`).
    pub joint_type: String,
    /// Identifier of the other part connected by this joint (or `"ground"`).
    pub connected_part_id: String,
    /// Role of the queried part in the joint (`"part_a"` or `"part_b"`).
    pub role: String,
    /// Joint anchor position `[x, y, z]` in the queried part's local coordinate frame (meters in `mcp-cad-physics`).
    pub anchor_local: [f64; 3],
    /// Reaction force `[Fx, Fy, Fz]` (N) acting ON this part in the part's local coordinate frame.
    pub force: [f64; 3],
    /// Reaction torque `[Tx, Ty, Tz]` (N*m) acting ON this part at `anchor_local` in the part's local coordinate frame.
    pub torque: [f64; 3],
    /// Reaction force `[Fx, Fy, Fz]` (N) acting ON this part in the world coordinate frame.
    pub force_world: [f64; 3],
    /// Reaction torque `[Tx, Ty, Tz]` (N*m) acting ON this part at the anchor in the world coordinate frame.
    pub torque_world: [f64; 3],
    /// Scalar magnitude `||force||` in Newtons.
    pub force_magnitude: f64,
    /// Scalar magnitude `||torque||` in N*m.
    pub torque_magnitude: f64,
}

/// Inertial state and accelerations of the part at the selected load-case instant (`P3-T6`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PartInertialLoad {
    /// Part mass in kg (`0.0` for fixed/ground bodies).
    pub mass_kg: f64,
    /// Whether the part is fixed in space (`is_fixed == true`).
    pub is_fixed: bool,
    /// Center of mass `[x, y, z]` in the part's local coordinate frame (meters in `mcp-cad-physics`).
    pub center_of_mass_local: [f64; 3],
    /// Linear acceleration of the center of mass `[ax, ay, az]` (m/s^2) in the part's local frame.
    pub linear_acceleration: [f64; 3],
    /// Angular acceleration `[alpha_x, alpha_y, alpha_z]` (rad/s^2) in the part's local frame.
    pub angular_acceleration: [f64; 3],
    /// Linear acceleration of the center of mass `[ax, ay, az]` (m/s^2) in the world frame.
    pub linear_acceleration_world: [f64; 3],
    /// Angular acceleration `[alpha_x, alpha_y, alpha_z]` (rad/s^2) in the world frame.
    pub angular_acceleration_world: [f64; 3],
    /// Linear velocity `[vx, vy, vz]` (m/s) in the part's local frame.
    pub linear_velocity: [f64; 3],
    /// Angular velocity `[wx, wy, wz]` (rad/s) in the part's local frame.
    pub angular_velocity: [f64; 3],
    /// Gravity vector `[gx, gy, gz]` (m/s^2) rotated into the part's local frame.
    pub gravity_local: [f64; 3],
}

/// Complete dynamic/quasi-static load case for a single part extracted from a physics simulation (`P3-T6`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PartLoadCase {
    /// Target part identifier.
    pub part_id: String,
    /// Selection mode used (`"peak_force"`, `"peak_torque"`, or `"time_s:<t>"`).
    pub select: String,
    /// Timestamp `time_s` (s) of the selected simulation sample.
    pub time_s: f64,
    /// Index of the selected sample in `BodyTelemetry.samples`.
    pub step_index: usize,
    /// Part world position `[x, y, z]` (m) at `time_s`.
    pub position_world: [f64; 3],
    /// Part world orientation quaternion `[qx, qy, qz, qw]` at `time_s`.
    pub orientation_quat: [f64; 4],
    /// Reaction loads from all joints connected to `part_id` (in `part_id`'s local coordinate frame).
    pub joints: Vec<JointLoadCase>,
    /// Resultant joint reaction force `sum(F_joint)` (N) in `part_id`'s local frame.
    pub net_joint_force: [f64; 3],
    /// Resultant joint reaction torque about the center of mass `sum(T_joint + (anchor - com) x F_joint)` (N*m) in `part_id`'s local frame.
    pub net_joint_torque_about_com: [f64; 3],
    /// Sum of pure joint reaction couples `sum(T_joint)` (N*m) in `part_id`'s local frame.
    pub net_joint_couple: [f64; 3],
    /// Scalar magnitude `||net_joint_force||` (N).
    pub net_joint_force_magnitude: f64,
    /// Scalar magnitude `||net_joint_torque_about_com||` (N*m).
    pub net_joint_torque_about_com_magnitude: f64,
    /// Inertial state (mass, COM, linear & angular acceleration) in `part_id`'s local frame.
    pub inertia: PartInertialLoad,
    /// Dynamic force equilibrium residual `sum(F_joints) + m * g - m * a_com` (N) in `part_id`'s local frame.
    pub equilibrium_residual_force: [f64; 3],
    /// Euclidean norm of `equilibrium_residual_force` (N).
    pub equilibrium_residual_force_norm: f64,
    /// Dynamic moment equilibrium residual about COM `sum(T_joint + (anchor - com) x F_joint) - (I * alpha + omega x (I * omega))` (N*m) in `part_id`'s local frame.
    pub equilibrium_residual_torque: [f64; 3],
    /// Euclidean norm of `equilibrium_residual_torque` (N*m).
    pub equilibrium_residual_torque_norm: f64,
}

/// Static interference and clearance analysis result for a single pair of CAD parts (`P3-T10`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PairInterferenceResult {
    /// First part identifier.
    pub a: String,
    /// Second part identifier.
    pub b: String,
    /// True if the two parts geometrically intersect (`intersection_volume > 0`).
    pub intersects: bool,
    /// True if `a` and `b` are directly connected by a registered kinematic joint in `PhysicsWorld`.
    pub adjacent_by_joint: bool,
    /// True if the world-space axis-aligned bounding boxes of `a` and `b` overlap (broad-phase pass).
    pub aabb_overlap: bool,
    /// Estimated penetration depth in meters when `intersects == true` (`None` when `intersects == false`).
    pub penetration_depth: Option<f64>,
    /// Exact CSG intersection volume in cubic meters (`m^3`) when `intersects == true` (`None` when `intersects == false`).
    pub intersection_volume: Option<f64>,
    /// Minimum separation distance (clearance) in meters when `intersects == false` (`None` when `intersects == true`).
    pub clearance: Option<f64>,
}

/// Summary report of multi-body static interference and clearance analysis (`P3-T10`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InterferenceReport {
    /// Overall status (`"PASS"` if `interfering_pairs_count == 0`, `"FAIL"` if any evaluated pair intersects).
    pub status: String,
    /// Total number of candidate part pairs before `ignore_adjacent` filtering.
    pub candidate_pairs_count: usize,
    /// Number of pairs skipped because `ignore_adjacent == true` and the pair is connected by a joint.
    pub skipped_adjacent_pairs_count: usize,
    /// Number of pairs whose world-space AABBs overlapped and required narrow-phase evaluation.
    pub broadphase_overlap_pairs_count: usize,
    /// Total number of pairs evaluated (returned in `pairs`).
    pub evaluated_pairs_count: usize,
    /// Number of evaluated pairs where `intersects == true`.
    pub interfering_pairs_count: usize,
    /// Minimum clearance (in meters) across all non-intersecting evaluated pairs (`None` if no non-intersecting pairs).
    pub min_clearance: Option<f64>,
    /// Maximum intersection volume (in `m^3`) across all intersecting pairs (`0.0` if none intersect).
    pub max_intersection_volume: f64,
    /// Maximum penetration depth (in meters) across all intersecting pairs (`0.0` if none intersect).
    pub max_penetration_depth: f64,
    /// Whether pairs directly connected by a joint were ignored.
    pub ignore_adjacent: bool,
    /// Per-pair interference and clearance results.
    pub pairs: Vec<PairInterferenceResult>,
    /// Wall-clock execution time of the interference check in milliseconds.
    pub elapsed_ms: f64,
}

/// Result of a single discrete configuration sample during [`crate::PhysicsWorld::sweep_joint`] (`P3-T10`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SweepStepResult {
    /// Step index `0..=steps`.
    pub step_index: usize,
    /// Joint coordinate value at this step (radians for `Revolute`, meters for `Prismatic`).
    pub value: f64,
    /// True if any evaluated pair intersects at this step.
    pub intersects: bool,
    /// Number of intersecting pairs at this step.
    pub interfering_pairs_count: usize,
    /// Minimum clearance (in meters) across non-intersecting pairs at this step (`None` if all pairs intersect).
    pub min_clearance: Option<f64>,
    /// Maximum penetration depth (in meters) at this step (`0.0` if no intersection).
    pub max_penetration_depth: f64,
    /// Maximum intersection volume (in `m^3`) at this step (`0.0` if no intersection).
    pub max_intersection_volume: f64,
    /// Intersecting pairs at this step.
    pub interfering_pairs: Vec<PairInterferenceResult>,
}

/// Complete report returned by [`crate::PhysicsWorld::sweep_joint`] (`P3-T10`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointSweepReport {
    /// Overall status (`"PASS"` if no collision occurred across the entire sweep, `"FAIL"` if collision detected).
    pub status: String,
    /// Swept joint identifier.
    pub joint_id: String,
    /// Canonical joint type (`"revolute"` or `"prismatic"`).
    pub joint_type: String,
    /// Driven (moving) part identifier.
    pub driven_part_id: String,
    /// Reference (parent/fixed) part identifier.
    pub reference_part_id: String,
    /// Sweep start coordinate (radians for `revolute`, meters for `prismatic`).
    pub from: f64,
    /// Sweep end coordinate (radians for `revolute`, meters for `prismatic`).
    pub to: f64,
    /// Number of sweep intervals (`samples.len() == steps + 1`).
    pub steps: usize,
    /// True if at least one step along the trajectory had an interference collision.
    pub collision_detected: bool,
    /// Index of the first step (`0..=steps`) where a collision occurred (`None` if collision-free).
    pub first_collision_step: Option<usize>,
    /// Joint coordinate value (radians or meters) at the first collision (`None` if collision-free).
    pub first_collision_value: Option<f64>,
    /// Intersecting part pairs at the first collision step.
    pub first_collision_pairs: Vec<PairInterferenceResult>,
    /// Minimum clearance (in meters) observed along the non-colliding steps (`None` if every step collided).
    pub min_clearance: Option<f64>,
    /// Step index where `min_clearance` was observed.
    pub min_clearance_step: Option<usize>,
    /// Joint coordinate value where `min_clearance` was observed.
    pub min_clearance_value: Option<f64>,
    /// Travel range before collision `(first_collision_value - from).abs()` if a collision occurred,
    /// or `(to - from).abs()` if the entire sweep range is collision-free.
    pub safe_travel_range: f64,
    /// Margin to collision: travel range to first collision when colliding, or `min_clearance` when collision-free.
    pub margin_to_collision: Option<f64>,
    /// Whether pairs directly connected by a joint were ignored.
    pub ignore_adjacent: bool,
    /// Per-step sweep samples (`steps + 1` entries).
    pub samples: Vec<SweepStepResult>,
    /// Wall-clock execution time in milliseconds.
    pub elapsed_ms: f64,
}


