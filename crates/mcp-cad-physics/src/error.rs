//! Error definitions for the MCP-CAD physics crate.

use thiserror::Error;

/// Physics engine errors.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum PhysicsError {
    /// Part was not found in the physics world.
    #[error("Part with id '{0}' was not found in physics world")]
    PartNotFound(String),

    /// Joint was not found in the physics world.
    #[error("Joint with id '{0}' was not found in physics world")]
    JointNotFound(String),

    /// Part already exists in the physics world.
    #[error("Part with id '{0}' already exists in physics world")]
    PartAlreadyExists(String),

    /// Joint already exists in the physics world.
    #[error("Joint with id '{0}' already exists in physics world")]
    JointAlreadyExists(String),

    /// Part geometry is invalid or empty.
    #[error("Invalid geometry for part '{0}': {1}")]
    InvalidGeometry(String, String),

    /// Collider generation failed.
    #[error("Failed to create collider for part '{0}': {1}")]
    ColliderCreationFailed(String, String),

    /// Joint configuration is invalid.
    #[error("Invalid joint configuration: {0}")]
    InvalidJointConfiguration(String),

    /// Simulation step or solve failed.
    #[error("Physics simulation error: {0}")]
    SimulationError(String),

    /// Serialization error.
    #[error("Physics serialization error: {0}")]
    SerializationError(String),
}
