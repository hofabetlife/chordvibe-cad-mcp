//! Error types for CAD operations.

use thiserror::Error;
use uuid::Uuid;

/// Errors that can occur within the CAD core engine.
#[derive(Error, Debug)]
pub enum CadError {
    #[error("Feature not found: {0}")]
    FeatureNotFound(Uuid),

    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    #[error("Evaluation error: {0}")]
    EvaluationError(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Material not found: {0}")]
    MaterialNotFound(String),

    #[error("CSG error: {0}")]
    CsgError(String),

    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),
}
