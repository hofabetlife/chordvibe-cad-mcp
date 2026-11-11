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
