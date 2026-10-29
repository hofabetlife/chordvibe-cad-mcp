//! Server error definitions.

use thiserror::Error;

/// Unified error type for the MCP-CAD server.
#[derive(Debug, Error)]
pub enum ServerError {
    #[error("CAD core error: {0}")]
    Cad(#[from] mcp_cad_core::CadError),

    #[error("Physics error: {0}")]
    Physics(#[from] mcp_cad_physics::PhysicsError),

    #[error("FEA error: {0}")]
    Fea(#[from] mcp_cad_fea::FeaError),

    #[error("Viewer error: {0}")]
    Viewer(#[from] mcp_cad_viewer::ViewerError),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("JSON serialization/deserialization error: {0}")]
    Json(#[from] serde_json::Error),

    #[error("Invalid argument: {0}")]
    InvalidArgument(String),

    #[error("Part not found: {0}")]
    PartNotFound(String),

    #[error("Feature not found: {0}")]
    FeatureNotFound(String),

    #[error("Joint not found: {0}")]
    JointNotFound(String),

    #[error("Tool not found: {0}")]
    ToolNotFound(String),

    #[error("Internal engine error: {0}")]
    Internal(String),
}
