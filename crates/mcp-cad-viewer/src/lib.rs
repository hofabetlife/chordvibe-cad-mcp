//! MCP-CAD Viewer: WGPU-based 3D viewport and offscreen headless renderer.

pub mod camera;
pub mod error;
pub mod offscreen;
pub mod shader;
pub mod window;

pub use camera::{CameraPreset, CameraUniform, OrbitCamera};
pub use error::ViewerError;
pub use offscreen::OffscreenRenderer;
pub use shader::{CadVertex, FeaVertex, MaterialUniform};
pub use window::CadWindowViewer;
