//! Error types for the MCP-CAD Viewer crate.

use thiserror::Error;

/// Enumeration of errors that can occur during 3D rendering and viewport management.
#[derive(Error, Debug)]
pub enum ViewerError {
    /// No suitable GPU adapter was found.
    #[error("No suitable GPU adapter found")]
    NoSuitableAdapter,

    /// Requesting a logical device or queue from the adapter failed.
    #[error("Failed to request GPU device: {0}")]
    DeviceRequestFailed(#[from] wgpu::RequestDeviceError),

    /// Image rendering dimensions are invalid (width or height is zero or excessively large).
    #[error("Invalid image dimensions: {width}x{height} (must be > 0 and within GPU texture limits)")]
    InvalidDimensions {
        /// Width in pixels
        width: u32,
        /// Height in pixels
        height: u32,
    },

    /// WGPU asynchronous buffer mapping failed.
    #[error("Buffer mapping failed: {0}")]
    BufferAsyncError(#[from] wgpu::BufferAsyncError),

    /// Image encoding or decoding error.
    #[error("Image encoding error: {0}")]
    ImageError(#[from] image::ImageError),

    /// Windowing event loop creation or execution failed.
    #[error("Winit event loop error: {0}")]
    EventLoopError(#[from] winit::error::EventLoopError),

    /// Operating system window creation error.
    #[error("Winit OS error: {0}")]
    OsError(#[from] winit::error::OsError),

    /// WGPU Surface creation error.
    #[error("WGPU surface creation error: {0}")]
    CreateSurfaceError(#[from] wgpu::CreateSurfaceError),

    /// WGPU Surface rendering error.
    #[error("WGPU surface error: {0}")]
    SurfaceError(#[from] wgpu::SurfaceError),

    /// Triangle mesh does not contain any vertices or faces.
    #[error("Mesh is empty: cannot render mesh without vertices or indices")]
    EmptyMesh,

    /// Number of stress scalars does not match vertex or triangle count.
    #[error("Stress scalar count mismatch for FEA heatmap: expected {expected}, got {actual}")]
    StressScalarMismatch {
        /// Expected scalar count
        expected: usize,
        /// Actual scalar count provided
        actual: usize,
    },

    /// An internal or unexpected error occurred.
    #[error("Internal viewer error: {0}")]
    Internal(String),
}
