//! MCP-CAD Viewer: WGPU-based 3D viewport and offscreen headless renderer.

pub mod camera;
pub mod error;
pub mod offscreen;
pub mod shader;
pub(crate) mod software;
pub mod window;

pub use camera::{CameraPreset, CameraUniform, OrbitCamera, ProjectionMode};
pub use error::ViewerError;
pub use offscreen::{extract_boundary_indices, DebugRenderMode, OffscreenRenderer};
pub use mcp_cad_core::tessellation::{TessellationQuality, TessellationSettings};
pub use shader::{
    deform_positions_with_scale, eval_field_colormap, CadVertex, FeaVertex, FieldColormap,
    FieldColormapUniform, MaterialUniform, UiVertex, WireframeUniform, CAD_SHADER_WGSL,
    FEA_HEATMAP_SHADER_WGSL, FIELD_COLORMAP_SHADER_WGSL, UI_SHADER_WGSL, WIREFRAME_SHADER_WGSL,
};
pub use window::{
    create_default_demo_mesh, create_gui_event_loop, extract_wireframe_indices, run_gui_app,
    CadWindowApp, CadWindowViewer, GuiEvent, GuiProxy, RenderMode, ViewportId, ViewportLayout,
};
