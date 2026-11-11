//! Physics simulation crate for MCP-CAD based on Rapier3D.
#![deny(unsafe_code)]

pub mod error;
pub mod joint;
pub mod rigid_body;
pub mod simulator;
pub mod telemetry;

pub use error::PhysicsError;
pub use joint::{CadJointType, MotorConfig};
pub use rigid_body::{ColliderType, create_collider_builder, create_rigid_body_builder};
pub use simulator::{CadJointRecord, PhysicsWorld};
pub use telemetry::{
    BodyState, BodyTelemetry, CollisionEventType, CollisionRecord, JointReactionForce,
    SimulationSummary,
};
