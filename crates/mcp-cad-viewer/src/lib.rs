//! MCP-CAD Viewer: WGPU-based 3D viewport and offscreen headless renderer.

pub mod camera;
pub mod error;
pub mod offscreen;
pub mod shader;
pub mod window;

pub use camera::{CameraPreset, CameraUniform, OrbitCamera, ProjectionMode};
pub use error::ViewerError;
pub use offscreen::OffscreenRenderer;
pub use mcp_cad_core::tessellation::{TessellationQuality, TessellationSettings};
pub use shader::{
    CadVertex, FeaVertex, MaterialUniform, UiVertex, WireframeUniform, CAD_SHADER_WGSL,
    FEA_HEATMAP_SHADER_WGSL, UI_SHADER_WGSL, WIREFRAME_SHADER_WGSL,
};
pub use window::{
    create_default_demo_mesh, create_gui_event_loop, extract_wireframe_indices, run_gui_app,
    CadWindowApp, CadWindowViewer, GuiEvent, GuiProxy, RenderMode, ViewportId, ViewportLayout,
};
