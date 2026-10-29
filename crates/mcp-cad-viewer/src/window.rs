//! Interactive native Windows GUI viewport using WGPU and Winit 0.30.

use std::sync::Arc;
use wgpu::util::DeviceExt;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowAttributes, WindowId};

use mcp_cad_core::topology::TriangleMesh;
use crate::camera::{CameraPreset, OrbitCamera, ProjectionMode};
use crate::error::ViewerError;
use crate::shader::{
    get_char_strokes, CadVertex, FeaVertex, MaterialUniform, UiVertex, WireframeUniform,
    CAD_SHADER_WGSL, FEA_HEATMAP_SHADER_WGSL, UI_SHADER_WGSL, WIREFRAME_SHADER_WGSL,
};

/// User event enum for GUI communication from background threads or MCP server.
#[derive(Debug, Clone)]
pub enum GuiEvent {
    Show,
    UpdateMesh(TriangleMesh),
    UpdateFea {
        mesh: TriangleMesh,
        stresses: Vec<f32>,
    },
    Close,
}

/// Event loop proxy type for sending GUI events to the main thread.
pub type GuiProxy = winit::event_loop::EventLoopProxy<GuiEvent>;

/// Active viewport render mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RenderMode {
    Cad,
    Fea,
}

/// Identifies one of the four viewport quadrants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ViewportId {
    /// Top-Left: General view (Perspective / Isometric 3D shaded).
    Perspective,
    /// Top-Right: Top view (Orthographic X-Z plane looking down -Y).
    Top,
    /// Bottom-Left: Right side view (Orthographic Z-Y plane looking towards -X).
    Right,
    /// Bottom-Right: Front elevation view (Orthographic X-Y plane looking towards -Z).
    Front,
}

/// Viewport layout modes: single maximized viewport or 4-view split (Cinema 4D style).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewportLayout {
    /// One single maximized viewport covering the entire window.
    Single(ViewportId),
    /// Four-view split layout (2x2 grid).
    FourView,
}

/// Extracts unique line segment indices from triangle mesh topology for wireframe rendering.
pub fn extract_wireframe_indices(indices: &[[u32; 3]]) -> Vec<u32> {
    use std::collections::HashSet;
    let mut edge_set = HashSet::with_capacity(indices.len() * 3);
    let mut line_indices = Vec::with_capacity(indices.len() * 3);

    for tri in indices {
        let e1 = if tri[0] < tri[1] { (tri[0], tri[1]) } else { (tri[1], tri[0]) };
        let e2 = if tri[1] < tri[2] { (tri[1], tri[2]) } else { (tri[2], tri[1]) };
        let e3 = if tri[2] < tri[0] { (tri[2], tri[0]) } else { (tri[0], tri[2]) };

        for e in [e1, e2, e3] {
            if edge_set.insert(e) {
                line_indices.push(e.0);
                line_indices.push(e.1);
            }
        }
    }

    line_indices
}

/// Generates a default CAD demo model (mounting bracket with center bore and bolt holes)
/// displayed when the viewer is launched manually without an initial mesh.
pub fn create_default_demo_mesh() -> TriangleMesh {
    let base = mcp_cad_core::create_box(100.0, 40.0, 60.0);
    let center_bore = mcp_cad_core::cut_hole(
        &base,
        [0.0, 20.0, 0.0],
        [0.0, 1.0, 0.0],
        14.0,
        40.0,
        Some(5.0),
        Some(3.0),
    );
    let left_hole = mcp_cad_core::cut_hole(
        &center_bore,
        [-35.0, 20.0, 0.0],
        [0.0, 1.0, 0.0],
        5.0,
        40.0,
        None,
        None,
    );
    mcp_cad_core::cut_hole(
        &left_hole,
        [35.0, 20.0, 0.0],
        [0.0, 1.0, 0.0],
        5.0,
        40.0,
        None,
        None,
    )
}

/// Interactive 3D CAD window state and renderer builder.
pub struct CadWindowViewer {
    title: String,
    mesh: Option<TriangleMesh>,
    stress_scalars: Option<Vec<f32>>,
    camera: OrbitCamera,
    layout: ViewportLayout,
}

impl CadWindowViewer {
    /// Creates a new CadWindowViewer instance.
    pub fn new(title: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            mesh: None,
            stress_scalars: None,
            camera: OrbitCamera::default(),
            layout: ViewportLayout::FourView,
        }
    }

    /// Sets the solid CAD mesh to display.
    pub fn with_mesh(mut self, mesh: TriangleMesh) -> Self {
        let bbox = mesh.bounding_box();
        let min_f32 = [bbox.min[0] as f32, bbox.min[1] as f32, bbox.min[2] as f32];
        let max_f32 = [bbox.max[0] as f32, bbox.max[1] as f32, bbox.max[2] as f32];
        self.camera.fit_bounding_box(min_f32, max_f32);
        self.mesh = Some(mesh);
        self
    }

    /// Sets FEA stress scalars for heatmap display.
    pub fn with_fea_stresses(mut self, stresses: Vec<f32>) -> Self {
        self.stress_scalars = Some(stresses);
        self
    }

    /// Sets camera preset viewpoint.
    pub fn with_camera_preset(mut self, preset: CameraPreset) -> Self {
        self.camera.apply_preset(preset);
        self
    }

    /// Sets initial viewport layout mode.
    pub fn with_layout(mut self, layout: ViewportLayout) -> Self {
        self.layout = layout;
        self
    }

    /// Returns a reference to the configured mesh, if any.
    pub fn mesh(&self) -> Option<&TriangleMesh> {
        self.mesh.as_ref()
    }

    /// Returns a reference to the configured stress scalars, if any.
    pub fn stress_scalars(&self) -> Option<&[f32]> {
        self.stress_scalars.as_deref()
    }

    /// Runs the interactive window event loop blocking the calling thread.
    pub fn run(self) -> Result<(), ViewerError> {
        let event_loop = create_gui_event_loop()?;
        let mut app = CadWindowApp::new(self.title);
        app.mesh = self.mesh;
        app.stress_scalars = self.stress_scalars;
        app.camera = self.camera;
        app.layout = self.layout;
        app.exit_on_close = true;
        if app.stress_scalars.is_some() {
            app.current_mode = RenderMode::Fea;
        }

        run_gui_app(event_loop, app)
    }
}

/// Creates a new Winit 0.30 event loop configured with user event `GuiEvent`.
pub fn create_gui_event_loop() -> Result<EventLoop<GuiEvent>, ViewerError> {
    EventLoop::<GuiEvent>::with_user_event()
        .build()
        .map_err(ViewerError::EventLoopError)
}

/// Runs the GUI application handler on the provided event loop.
pub fn run_gui_app(
    event_loop: EventLoop<GuiEvent>,
    mut app: CadWindowApp,
) -> Result<(), ViewerError> {
    event_loop.set_control_flow(ControlFlow::Wait);
    event_loop.run_app(&mut app).map_err(ViewerError::EventLoopError)
}

/// Helper structure for accumulating 2D UI batch geometry (quads & lines).
struct UiBatch {
    quads: Vec<UiVertex>,
    lines: Vec<UiVertex>,
    screen_w: f32,
    screen_h: f32,
}

impl UiBatch {
    fn new(screen_w: f32, screen_h: f32) -> Self {
        Self {
            quads: Vec::with_capacity(512),
            lines: Vec::with_capacity(1024),
            screen_w: screen_w.max(1.0),
            screen_h: screen_h.max(1.0),
        }
    }

    fn px_to_ndc(&self, px: f32, py: f32) -> [f32; 2] {
        [
            (px / self.screen_w) * 2.0 - 1.0,
            1.0 - (py / self.screen_h) * 2.0,
        ]
    }

    fn fill_rect(&mut self, x: f32, y: f32, w: f32, h: f32, color: [f32; 4]) {
        let p0 = self.px_to_ndc(x, y);
        let p1 = self.px_to_ndc(x + w, y);
        let p2 = self.px_to_ndc(x, y + h);
        let p3 = self.px_to_ndc(x + w, y + h);

        self.quads.push(UiVertex { position: p0, color });
        self.quads.push(UiVertex { position: p2, color });
        self.quads.push(UiVertex { position: p1, color });

        self.quads.push(UiVertex { position: p1, color });
        self.quads.push(UiVertex { position: p2, color });
        self.quads.push(UiVertex { position: p3, color });
    }

    fn stroke_rect(&mut self, x: f32, y: f32, w: f32, h: f32, color: [f32; 4]) {
        let p0 = self.px_to_ndc(x, y);
        let p1 = self.px_to_ndc(x + w, y);
        let p2 = self.px_to_ndc(x + w, y + h);
        let p3 = self.px_to_ndc(x, y + h);

        self.lines.push(UiVertex { position: p0, color });
        self.lines.push(UiVertex { position: p1, color });
        self.lines.push(UiVertex { position: p1, color });
        self.lines.push(UiVertex { position: p2, color });
        self.lines.push(UiVertex { position: p2, color });
        self.lines.push(UiVertex { position: p3, color });
        self.lines.push(UiVertex { position: p3, color });
        self.lines.push(UiVertex { position: p0, color });
    }

    fn line(&mut self, x0: f32, y0: f32, x1: f32, y1: f32, color: [f32; 4]) {
        let p0 = self.px_to_ndc(x0, y0);
        let p1 = self.px_to_ndc(x1, y1);
        self.lines.push(UiVertex { position: p0, color });
        self.lines.push(UiVertex { position: p1, color });
    }

    fn text(&mut self, text: &str, x: f32, y: f32, size: f32, color: [f32; 4]) {
        let char_w = size * 0.65;
        let char_h = size;
        let spacing = size * 0.22;
        let mut cur_x = x;
        for ch in text.chars() {
            let strokes = get_char_strokes(ch);
            for &(sx0, sy0, sx1, sy1) in strokes {
                let px0 = cur_x + sx0 * char_w;
                let py0 = y + (1.0 - sy0) * char_h;
                let px1 = cur_x + sx1 * char_w;
                let py1 = y + (1.0 - sy1) * char_h;
                let p0 = self.px_to_ndc(px0, py0);
                let p1 = self.px_to_ndc(px1, py1);
                self.lines.push(UiVertex { position: p0, color });
                self.lines.push(UiVertex { position: p1, color });
            }
            cur_x += char_w + spacing;
        }
    }
}

/// Application handler implementing Winit 0.30 ApplicationHandler for GuiEvent.
pub struct CadWindowApp {
    pub title: String,
    pub mesh: Option<TriangleMesh>,
    pub stress_scalars: Option<Vec<f32>>,

    // 4 Cameras for Cinema 4D multi-view layout
    pub camera: OrbitCamera,          // Top-Left: General 3D Perspective
    pub camera_top: OrbitCamera,      // Top-Right: Top Orthographic
    pub camera_right: OrbitCamera,    // Bottom-Left: Right Orthographic
    pub camera_front: OrbitCamera,    // Bottom-Right: Front Orthographic

    pub layout: ViewportLayout,
    pub active_viewport: ViewportId,
    pub hovered_viewport: Option<ViewportId>,
    pub current_mode: RenderMode,

    pub window: Option<Arc<Window>>,
    pub surface: Option<wgpu::Surface<'static>>,
    pub device: Option<wgpu::Device>,
    pub queue: Option<wgpu::Queue>,
    pub surface_config: Option<wgpu::SurfaceConfiguration>,
    pub depth_view: Option<wgpu::TextureView>,

    // CAD Solid Pipeline
    pub cad_pipeline: Option<wgpu::RenderPipeline>,
    pub cad_bind_group_layout: Option<wgpu::BindGroupLayout>,
    pub cad_bind_group: Option<wgpu::BindGroup>,

    // FEA Heatmap Pipeline
    pub fea_pipeline: Option<wgpu::RenderPipeline>,
    pub fea_bind_group_layout: Option<wgpu::BindGroupLayout>,
    pub fea_bind_group: Option<wgpu::BindGroup>,

    // Wireframe Line Pipeline
    pub wireframe_pipeline: Option<wgpu::RenderPipeline>,
    pub wireframe_bind_group_layout: Option<wgpu::BindGroupLayout>,
    pub wireframe_bind_group_top: Option<wgpu::BindGroup>,
    pub wireframe_bind_group_right: Option<wgpu::BindGroup>,
    pub wireframe_bind_group_front: Option<wgpu::BindGroup>,
    pub wireframe_bind_group_persp: Option<wgpu::BindGroup>,

    // 2D UI Pipelines
    pub ui_quad_pipeline: Option<wgpu::RenderPipeline>,
    pub ui_line_pipeline: Option<wgpu::RenderPipeline>,

    // Buffers
    pub vertex_buffer: Option<wgpu::Buffer>,
    pub index_buffer: Option<wgpu::Buffer>,
    pub wireframe_index_buffer: Option<wgpu::Buffer>,
    pub index_count: u32,
    pub wireframe_index_count: u32,

    pub camera_buffer_persp: Option<wgpu::Buffer>,
    pub camera_buffer_top: Option<wgpu::Buffer>,
    pub camera_buffer_right: Option<wgpu::Buffer>,
    pub camera_buffer_front: Option<wgpu::Buffer>,

    pub material_buffer: Option<wgpu::Buffer>,
    pub wireframe_uniform_buffer: Option<wgpu::Buffer>,

    // Mouse Interaction
    pub is_mouse_down: bool,
    pub is_rmb_down: bool,
    pub last_mouse_pos: Option<(f64, f64)>,
    pub exit_on_close: bool,
}

impl Default for CadWindowApp {
    fn default() -> Self {
        Self::new("MCP-CAD Preview")
    }
}

impl CadWindowApp {
    /// Creates a new CadWindowApp with default settings.
    pub fn new(title: impl Into<String>) -> Self {
        let mut camera_top = OrbitCamera::default();
        camera_top.apply_preset(CameraPreset::Top);
        camera_top.projection = ProjectionMode::Orthographic;

        let mut camera_right = OrbitCamera::default();
        camera_right.apply_preset(CameraPreset::Right);
        camera_right.projection = ProjectionMode::Orthographic;

        let mut camera_front = OrbitCamera::default();
        camera_front.apply_preset(CameraPreset::Front);
        camera_front.projection = ProjectionMode::Orthographic;

        Self {
            title: title.into(),
            mesh: None,
            stress_scalars: None,
            camera: OrbitCamera::default(),
            camera_top,
            camera_right,
            camera_front,
            layout: ViewportLayout::FourView,
            active_viewport: ViewportId::Perspective,
            hovered_viewport: None,
            current_mode: RenderMode::Cad,
            window: None,
            surface: None,
            device: None,
            queue: None,
            surface_config: None,
            depth_view: None,
            cad_pipeline: None,
            cad_bind_group_layout: None,
            cad_bind_group: None,
            fea_pipeline: None,
            fea_bind_group_layout: None,
            fea_bind_group: None,
            wireframe_pipeline: None,
            wireframe_bind_group_layout: None,
            wireframe_bind_group_top: None,
            wireframe_bind_group_right: None,
            wireframe_bind_group_front: None,
            wireframe_bind_group_persp: None,
            ui_quad_pipeline: None,
            ui_line_pipeline: None,
            vertex_buffer: None,
            index_buffer: None,
            wireframe_index_buffer: None,
            index_count: 0,
            wireframe_index_count: 0,
            camera_buffer_persp: None,
            camera_buffer_top: None,
            camera_buffer_right: None,
            camera_buffer_front: None,
            material_buffer: None,
            wireframe_uniform_buffer: None,
            is_mouse_down: false,
            is_rmb_down: false,
            last_mouse_pos: None,
            exit_on_close: false,
        }
    }

    /// Sets whether the application should exit when the window is closed.
    pub fn with_exit_on_close(mut self, exit_on_close: bool) -> Self {
        self.exit_on_close = exit_on_close;
        self
    }

    /// Determines which viewport quadrant contains the given pixel coordinate.
    pub fn calculate_viewport_at(&self, mx: f32, my: f32, screen_w: f32, screen_h: f32) -> Option<ViewportId> {
        let top_bar_h = 32.0;
        let bot_bar_h = 24.0;
        if my < top_bar_h || my > (screen_h - bot_bar_h) {
            return None;
        }
        match self.layout {
            ViewportLayout::Single(v) => Some(v),
            ViewportLayout::FourView => {
                let content_h = screen_h - top_bar_h - bot_bar_h;
                let w_half = (screen_w * 0.5).floor();
                let h_half = top_bar_h + (content_h * 0.5).floor();
                if mx < w_half && my < h_half {
                    Some(ViewportId::Perspective)
                } else if mx >= w_half && my < h_half {
                    Some(ViewportId::Top)
                } else if mx < w_half && my >= h_half {
                    Some(ViewportId::Right)
                } else {
                    Some(ViewportId::Front)
                }
            }
        }
    }

    /// Updates camera aspect ratios according to current window dimensions and layout mode.
    pub fn update_camera_aspects(&mut self, width: u32, height: u32) {
        let screen_w = width as f32;
        let screen_h = height as f32;
        let top_y = 32.0f32;
        let bot_h = 24.0f32;
        let content_h = (screen_h - top_y - bot_h).max(10.0);
        let w_half = (screen_w * 0.5).floor();
        let h_half = (content_h * 0.5).floor();

        match self.layout {
            ViewportLayout::Single(_) => {
                let aspect = (screen_w / content_h).max(0.01);
                self.camera.aspect = aspect;
                self.camera_top.aspect = aspect;
                self.camera_right.aspect = aspect;
                self.camera_front.aspect = aspect;
            }
            ViewportLayout::FourView => {
                self.camera.aspect = (w_half / h_half).max(0.01);
                self.camera_top.aspect = ((screen_w - w_half) / h_half).max(0.01);
                self.camera_right.aspect = (w_half / (content_h - h_half)).max(0.01);
                self.camera_front.aspect = ((screen_w - w_half) / (content_h - h_half)).max(0.01);
            }
        }
    }

    /// Ensures that the native OS window and WGPU graphics pipelines are created and initialized.
    pub fn ensure_window(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        let window_title = if self.title.is_empty() {
            "MCP-CAD Preview".to_string()
        } else {
            self.title.clone()
        };

        let attrs = WindowAttributes::default()
            .with_title(&window_title)
            .with_inner_size(winit::dpi::LogicalSize::new(1152, 864));

        let window = match event_loop.create_window(attrs) {
            Ok(w) => Arc::new(w),
            Err(e) => {
                eprintln!("Failed to create window: {e}");
                return;
            }
        };

        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let surface = match instance.create_surface(window.clone()) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to create WGPU surface: {e}");
                return;
            }
        };

        let adapter = match pollster::block_on(async {
            if let Some(a) = instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::HighPerformance,
                    force_fallback_adapter: false,
                    compatible_surface: Some(&surface),
                })
                .await
            {
                return Some(a);
            }
            if let Some(a) = instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::None,
                    force_fallback_adapter: false,
                    compatible_surface: Some(&surface),
                })
                .await
            {
                return Some(a);
            }
            instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::None,
                    force_fallback_adapter: true,
                    compatible_surface: Some(&surface),
                })
                .await
        }) {
            Some(a) => a,
            None => {
                eprintln!("Failed to acquire graphics adapter");
                return;
            }
        };

        let (device, queue) = match pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("CAD Window Device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::Performance,
            },
            None,
        )) {
            Ok(pair) => pair,
            Err(e) => {
                eprintln!("Failed to create logical device: {e}");
                return;
            }
        };

        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .find(|f| f.is_srgb())
            .copied()
            .unwrap_or(surface_caps.formats[0]);

        let width = size.width.max(1);
        let height = size.height.max(1);

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width,
            height,
            present_mode: wgpu::PresentMode::AutoVsync,
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &config);

        self.update_camera_aspects(width, height);
        let depth_view = create_depth_texture(&device, width, height);

        // 1. Uniform Buffers for 4 Cameras
        let cam_u_persp = self.camera.build_uniform();
        let cam_buf_persp = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window Camera Buffer Persp"),
            contents: bytemuck::bytes_of(&cam_u_persp),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let cam_u_top = self.camera_top.build_uniform();
        let cam_buf_top = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window Camera Buffer Top"),
            contents: bytemuck::bytes_of(&cam_u_top),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let cam_u_right = self.camera_right.build_uniform();
        let cam_buf_right = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window Camera Buffer Right"),
            contents: bytemuck::bytes_of(&cam_u_right),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let cam_u_front = self.camera_front.build_uniform();
        let cam_buf_front = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window Camera Buffer Front"),
            contents: bytemuck::bytes_of(&cam_u_front),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let mat_uniform = MaterialUniform::default();
        let material_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window Material Buffer"),
            contents: bytemuck::bytes_of(&mat_uniform),
            usage: wgpu::BufferUsages::UNIFORM,
        });

        let wireframe_uniform = WireframeUniform::default(); // Black lines
        let wireframe_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window Wireframe Uniform Buffer"),
            contents: bytemuck::bytes_of(&wireframe_uniform),
            usage: wgpu::BufferUsages::UNIFORM,
        });

        // 2. CAD Render Pipeline & Bind Group
        let cad_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Window CAD Shader"),
            source: wgpu::ShaderSource::Wgsl(CAD_SHADER_WGSL.into()),
        });

        let cad_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Window CAD Bind Group Layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
            });

        let cad_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Window CAD Pipeline Layout"),
                bind_group_layouts: &[&cad_bind_group_layout],
                push_constant_ranges: &[],
            });

        let cad_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Window CAD Render Pipeline"),
            layout: Some(&cad_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &cad_shader,
                entry_point: "vs_main",
                buffers: &[CadVertex::LAYOUT],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &cad_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let cad_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Window CAD Bind Group"),
            layout: &cad_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: cam_buf_persp.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: material_buffer.as_entire_binding(),
                },
            ],
        });

        // 3. FEA Heatmap Render Pipeline & Bind Group
        let fea_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Window FEA Shader"),
            source: wgpu::ShaderSource::Wgsl(FEA_HEATMAP_SHADER_WGSL.into()),
        });

        let fea_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Window FEA Bind Group Layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let fea_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Window FEA Pipeline Layout"),
                bind_group_layouts: &[&fea_bind_group_layout],
                push_constant_ranges: &[],
            });

        let fea_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Window FEA Render Pipeline"),
            layout: Some(&fea_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &fea_shader,
                entry_point: "vs_main",
                buffers: &[FeaVertex::LAYOUT],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &fea_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let fea_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Window FEA Bind Group"),
            layout: &fea_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: cam_buf_persp.as_entire_binding(),
            }],
        });

        // 4. Wireframe Render Pipeline & Bind Groups
        let wireframe_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Window Wireframe Shader"),
            source: wgpu::ShaderSource::Wgsl(WIREFRAME_SHADER_WGSL.into()),
        });

        let wireframe_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Window Wireframe Bind Group Layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::VERTEX,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
            });

        let wireframe_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Window Wireframe Pipeline Layout"),
                bind_group_layouts: &[&wireframe_bind_group_layout],
                push_constant_ranges: &[],
            });

        let wireframe_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Window Wireframe Render Pipeline"),
            layout: Some(&wireframe_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &wireframe_shader,
                entry_point: "vs_main",
                buffers: &[CadVertex::LAYOUT],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &wireframe_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::Always,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let wireframe_bind_group_persp = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Window Wireframe BG Persp"),
            layout: &wireframe_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: cam_buf_persp.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wireframe_uniform_buffer.as_entire_binding(),
                },
            ],
        });

        let wireframe_bind_group_top = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Window Wireframe BG Top"),
            layout: &wireframe_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: cam_buf_top.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wireframe_uniform_buffer.as_entire_binding(),
                },
            ],
        });

        let wireframe_bind_group_right = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Window Wireframe BG Right"),
            layout: &wireframe_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: cam_buf_right.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wireframe_uniform_buffer.as_entire_binding(),
                },
            ],
        });

        let wireframe_bind_group_front = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Window Wireframe BG Front"),
            layout: &wireframe_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: cam_buf_front.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wireframe_uniform_buffer.as_entire_binding(),
                },
            ],
        });

        // 5. 2D UI Pipelines (Quads & Lines)
        let ui_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Window UI Shader"),
            source: wgpu::ShaderSource::Wgsl(UI_SHADER_WGSL.into()),
        });

        let ui_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Window UI Pipeline Layout"),
            bind_group_layouts: &[],
            push_constant_ranges: &[],
        });

        let ui_quad_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Window UI Quad Pipeline"),
            layout: Some(&ui_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &ui_shader,
                entry_point: "vs_main",
                buffers: &[UiVertex::LAYOUT],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &ui_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::Always,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let ui_line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Window UI Line Pipeline"),
            layout: Some(&ui_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &ui_shader,
                entry_point: "vs_main",
                buffers: &[UiVertex::LAYOUT],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &ui_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::Always,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        self.window = Some(window);
        self.surface = Some(surface);
        self.device = Some(device);
        self.queue = Some(queue);
        self.surface_config = Some(config);
        self.depth_view = Some(depth_view);

        self.camera_buffer_persp = Some(cam_buf_persp);
        self.camera_buffer_top = Some(cam_buf_top);
        self.camera_buffer_right = Some(cam_buf_right);
        self.camera_buffer_front = Some(cam_buf_front);
        self.material_buffer = Some(material_buffer);
        self.wireframe_uniform_buffer = Some(wireframe_uniform_buffer);

        self.cad_pipeline = Some(cad_pipeline);
        self.cad_bind_group_layout = Some(cad_bind_group_layout);
        self.cad_bind_group = Some(cad_bind_group);

        self.fea_pipeline = Some(fea_pipeline);
        self.fea_bind_group_layout = Some(fea_bind_group_layout);
        self.fea_bind_group = Some(fea_bind_group);

        self.wireframe_pipeline = Some(wireframe_pipeline);
        self.wireframe_bind_group_layout = Some(wireframe_bind_group_layout);
        self.wireframe_bind_group_persp = Some(wireframe_bind_group_persp);
        self.wireframe_bind_group_top = Some(wireframe_bind_group_top);
        self.wireframe_bind_group_right = Some(wireframe_bind_group_right);
        self.wireframe_bind_group_front = Some(wireframe_bind_group_front);

        self.ui_quad_pipeline = Some(ui_quad_pipeline);
        self.ui_line_pipeline = Some(ui_line_pipeline);

        // Upload initial mesh if already staged, or generate default CAD demo model if none
        if let Some(mesh) = self.mesh.clone() {
            if self.current_mode == RenderMode::Fea {
                if let Some(stresses) = self.stress_scalars.clone() {
                    self.upload_fea_mesh(&mesh, &stresses);
                } else {
                    self.upload_cad_mesh(&mesh);
                }
            } else {
                self.upload_cad_mesh(&mesh);
            }
        } else {
            let demo_mesh = create_default_demo_mesh();
            self.upload_cad_mesh(&demo_mesh);
        }
    }

    /// Uploads standard CAD mesh geometry into GPU buffers and fits all 4 cameras.
    pub fn upload_cad_mesh(&mut self, mesh: &TriangleMesh) {
        self.mesh = Some(mesh.clone());
        self.stress_scalars = None;
        self.current_mode = RenderMode::Cad;

        let bbox = mesh.bounding_box();
        let min_f32 = [bbox.min[0] as f32, bbox.min[1] as f32, bbox.min[2] as f32];
        let max_f32 = [bbox.max[0] as f32, bbox.max[1] as f32, bbox.max[2] as f32];

        self.camera.fit_bounding_box(min_f32, max_f32);
        self.camera.projection = ProjectionMode::Perspective;

        self.camera_top.fit_bounding_box(min_f32, max_f32);
        self.camera_top.apply_preset(CameraPreset::Top);
        self.camera_top.projection = ProjectionMode::Orthographic;

        self.camera_right.fit_bounding_box(min_f32, max_f32);
        self.camera_right.apply_preset(CameraPreset::Right);
        self.camera_right.projection = ProjectionMode::Orthographic;

        self.camera_front.fit_bounding_box(min_f32, max_f32);
        self.camera_front.apply_preset(CameraPreset::Front);
        self.camera_front.projection = ProjectionMode::Orthographic;

        if let Some(cfg) = &self.surface_config {
            self.update_camera_aspects(cfg.width, cfg.height);
        }
        self.update_camera_buffer();

        let Some(device) = &self.device else { return };
        if mesh.vertices.is_empty() || mesh.indices.is_empty() {
            self.vertex_buffer = None;
            self.index_buffer = None;
            self.wireframe_index_buffer = None;
            self.index_count = 0;
            self.wireframe_index_count = 0;
            return;
        }

        let cad_verts: Vec<CadVertex> = mesh
            .vertices
            .iter()
            .map(|v| CadVertex {
                position: [v.position[0] as f32, v.position[1] as f32, v.position[2] as f32],
                normal: [v.normal[0] as f32, v.normal[1] as f32, v.normal[2] as f32],
            })
            .collect();

        let flat_indices: Vec<u32> = mesh
            .indices
            .iter()
            .flat_map(|tri| tri.iter().copied())
            .collect();

        let line_indices = extract_wireframe_indices(&mesh.indices);

        let vb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window CAD VB"),
            contents: bytemuck::cast_slice(&cad_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let ib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window CAD IB"),
            contents: bytemuck::cast_slice(&flat_indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        let wib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window Wireframe IB"),
            contents: bytemuck::cast_slice(&line_indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        self.vertex_buffer = Some(vb);
        self.index_buffer = Some(ib);
        self.wireframe_index_buffer = Some(wib);
        self.index_count = flat_indices.len() as u32;
        self.wireframe_index_count = line_indices.len() as u32;
    }

    /// Uploads FEA surface mesh geometry and scalar stress values into GPU buffers and fits all cameras.
    pub fn upload_fea_mesh(&mut self, mesh: &TriangleMesh, stresses: &[f32]) {
        self.mesh = Some(mesh.clone());
        self.stress_scalars = Some(stresses.to_vec());
        self.current_mode = RenderMode::Fea;

        let bbox = mesh.bounding_box();
        let min_f32 = [bbox.min[0] as f32, bbox.min[1] as f32, bbox.min[2] as f32];
        let max_f32 = [bbox.max[0] as f32, bbox.max[1] as f32, bbox.max[2] as f32];

        self.camera.fit_bounding_box(min_f32, max_f32);
        self.camera.projection = ProjectionMode::Perspective;

        self.camera_top.fit_bounding_box(min_f32, max_f32);
        self.camera_top.apply_preset(CameraPreset::Top);
        self.camera_top.projection = ProjectionMode::Orthographic;

        self.camera_right.fit_bounding_box(min_f32, max_f32);
        self.camera_right.apply_preset(CameraPreset::Right);
        self.camera_right.projection = ProjectionMode::Orthographic;

        self.camera_front.fit_bounding_box(min_f32, max_f32);
        self.camera_front.apply_preset(CameraPreset::Front);
        self.camera_front.projection = ProjectionMode::Orthographic;

        if let Some(cfg) = &self.surface_config {
            self.update_camera_aspects(cfg.width, cfg.height);
        }
        self.update_camera_buffer();

        let Some(device) = &self.device else { return };
        if mesh.vertices.is_empty() || mesh.indices.is_empty() {
            self.vertex_buffer = None;
            self.index_buffer = None;
            self.wireframe_index_buffer = None;
            self.index_count = 0;
            self.wireframe_index_count = 0;
            return;
        }

        let fea_verts: Vec<FeaVertex> = mesh
            .vertices
            .iter()
            .zip(stresses.iter().chain(std::iter::repeat(&0.0)))
            .map(|(v, &s)| FeaVertex {
                position: [v.position[0] as f32, v.position[1] as f32, v.position[2] as f32],
                normal: [v.normal[0] as f32, v.normal[1] as f32, v.normal[2] as f32],
                scalar: s.clamp(0.0, 1.0),
            })
            .collect();

        let flat_indices: Vec<u32> = mesh
            .indices
            .iter()
            .flat_map(|tri| tri.iter().copied())
            .collect();

        let line_indices = extract_wireframe_indices(&mesh.indices);

        let vb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window FEA VB"),
            contents: bytemuck::cast_slice(&fea_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let ib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window FEA IB"),
            contents: bytemuck::cast_slice(&flat_indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        let wib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window Wireframe IB"),
            contents: bytemuck::cast_slice(&line_indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        self.vertex_buffer = Some(vb);
        self.index_buffer = Some(ib);
        self.wireframe_index_buffer = Some(wib);
        self.index_count = flat_indices.len() as u32;
        self.wireframe_index_count = line_indices.len() as u32;
    }

    /// Writes the current camera uniform buffers to the GPU for all active viewports.
    pub fn update_camera_buffer(&self) {
        let Some(queue) = &self.queue else { return };
        if let Some(cb) = &self.camera_buffer_persp {
            let uniform = self.camera.build_uniform();
            queue.write_buffer(cb, 0, bytemuck::bytes_of(&uniform));
        }
        if let Some(cb) = &self.camera_buffer_top {
            let uniform = self.camera_top.build_uniform();
            queue.write_buffer(cb, 0, bytemuck::bytes_of(&uniform));
        }
        if let Some(cb) = &self.camera_buffer_right {
            let uniform = self.camera_right.build_uniform();
            queue.write_buffer(cb, 0, bytemuck::bytes_of(&uniform));
        }
        if let Some(cb) = &self.camera_buffer_front {
            let uniform = self.camera_front.build_uniform();
            queue.write_buffer(cb, 0, bytemuck::bytes_of(&uniform));
        }
    }

    /// Fits bounding box framing for all cameras.
    pub fn fit_all_cameras(&mut self) {
        if let Some(mesh) = &self.mesh {
            let bbox = mesh.bounding_box();
            let min_f32 = [bbox.min[0] as f32, bbox.min[1] as f32, bbox.min[2] as f32];
            let max_f32 = [bbox.max[0] as f32, bbox.max[1] as f32, bbox.max[2] as f32];

            self.camera.fit_bounding_box(min_f32, max_f32);
            self.camera_top.fit_bounding_box(min_f32, max_f32);
            self.camera_top.apply_preset(CameraPreset::Top);
            self.camera_right.fit_bounding_box(min_f32, max_f32);
            self.camera_right.apply_preset(CameraPreset::Right);
            self.camera_front.fit_bounding_box(min_f32, max_f32);
            self.camera_front.apply_preset(CameraPreset::Front);

            if let Some(cfg) = &self.surface_config {
                self.update_camera_aspects(cfg.width, cfg.height);
            }
            self.update_camera_buffer();
        }
    }

    /// Renders the current frame to the window surface.
    pub fn render_frame(&mut self) {
        let (Some(surface), Some(device), Some(queue)) =
            (&self.surface, &self.device, &self.queue)
        else {
            return;
        };

        let output = match surface.get_current_texture() {
            Ok(output) => output,
            Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                if let Some(config) = &self.surface_config {
                    surface.configure(device, config);
                }
                return;
            }
            Err(wgpu::SurfaceError::OutOfMemory) => {
                eprintln!("WGPU Out of memory during surface acquisition");
                return;
            }
            Err(e) => {
                eprintln!("Surface texture error: {e}");
                return;
            }
        };

        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let (screen_w, screen_h) = if let Some(cfg) = &self.surface_config {
            (cfg.width as f32, cfg.height as f32)
        } else {
            (1024.0, 768.0)
        };

        let top_y = 32.0f32;
        let bot_h = 24.0f32;
        let content_h = (screen_h - top_y - bot_h).max(10.0);
        let w_half = (screen_w * 0.5).floor();
        let h_half = (content_h * 0.5).floor();
        let mid_y = top_y + h_half;

        // Build UI batches
        let mut ui = UiBatch::new(screen_w, screen_h);
        let mut bg_ui = UiBatch::new(screen_w, screen_h);

        // 1. Draw technical paper background for orthographic viewports
        let ortho_bg_color = [0.82, 0.84, 0.87, 1.0];
        match self.layout {
            ViewportLayout::FourView => {
                // Top view (Quadrant 1)
                bg_ui.fill_rect(w_half, top_y, screen_w - w_half, h_half, ortho_bg_color);
                // Right view (Quadrant 2)
                bg_ui.fill_rect(0.0, mid_y, w_half, content_h - h_half, ortho_bg_color);
                // Front view (Quadrant 3)
                bg_ui.fill_rect(w_half, mid_y, screen_w - w_half, content_h - h_half, ortho_bg_color);
            }
            ViewportLayout::Single(id) => {
                if id != ViewportId::Perspective {
                    bg_ui.fill_rect(0.0, top_y, screen_w, content_h, ortho_bg_color);
                }
            }
        }

        // 2. Build Top Bar & Interactive Navigation Buttons
        ui.fill_rect(0.0, 0.0, screen_w, top_y, [0.10, 0.12, 0.15, 0.95]);
        ui.line(0.0, top_y, screen_w, top_y, [0.22, 0.25, 0.30, 1.0]);
        ui.text("MCP-CAD", 12.0, 9.0, 14.0, [0.35, 0.85, 1.0, 1.0]);

        let buttons = [
            ("4-VIEW", 95.0, 72.0, self.layout == ViewportLayout::FourView),
            ("PERSP", 173.0, 65.0, self.layout == ViewportLayout::Single(ViewportId::Perspective)),
            ("TOP", 244.0, 55.0, self.layout == ViewportLayout::Single(ViewportId::Top)),
            ("RIGHT", 305.0, 65.0, self.layout == ViewportLayout::Single(ViewportId::Right)),
            ("FRONT", 376.0, 65.0, self.layout == ViewportLayout::Single(ViewportId::Front)),
            ("FIT", 447.0, 50.0, false),
        ];

        let (mouse_x, mouse_y) = self.last_mouse_pos.unwrap_or((-1.0, -1.0));
        let mx = mouse_x as f32;
        let my = mouse_y as f32;

        for &(label, bx, bw, is_active) in &buttons {
            let by = 5.0f32;
            let bh = 22.0f32;
            let is_hovered = mx >= bx && mx <= (bx + bw) && my >= by && my <= (by + bh);

            if is_active {
                ui.fill_rect(bx, by, bw, bh, [0.18, 0.45, 0.75, 1.0]);
                ui.stroke_rect(bx, by, bw, bh, [0.45, 0.75, 1.0, 1.0]);
                let pad = (bw - (label.len() as f32 * 11.0 * 0.87)) * 0.5;
                ui.text(label, bx + pad.max(4.0), by + 5.0, 11.0, [1.0, 1.0, 1.0, 1.0]);
            } else if is_hovered {
                ui.fill_rect(bx, by, bw, bh, [0.22, 0.26, 0.33, 1.0]);
                ui.stroke_rect(bx, by, bw, bh, [0.35, 0.65, 0.95, 1.0]);
                let pad = (bw - (label.len() as f32 * 11.0 * 0.87)) * 0.5;
                ui.text(label, bx + pad.max(4.0), by + 5.0, 11.0, [0.35, 0.85, 1.0, 1.0]);
            } else {
                ui.fill_rect(bx, by, bw, bh, [0.14, 0.16, 0.20, 1.0]);
                ui.stroke_rect(bx, by, bw, bh, [0.25, 0.28, 0.33, 1.0]);
                let pad = (bw - (label.len() as f32 * 11.0 * 0.87)) * 0.5;
                ui.text(label, bx + pad.max(4.0), by + 5.0, 11.0, [0.72, 0.76, 0.80, 1.0]);
            }
        }

        // Mesh statistics badge
        if let Some(m) = &self.mesh {
            let info = format!("V:{} T:{}", m.vertices.len(), m.indices.len());
            ui.text(&info, screen_w - 150.0, 10.0, 11.0, [0.55, 0.60, 0.66, 1.0]);
        }

        // 3. Bottom Status Bar with shortcut instructions
        ui.fill_rect(0.0, screen_h - bot_h, screen_w, bot_h, [0.10, 0.12, 0.15, 0.95]);
        ui.line(0.0, screen_h - bot_h, screen_w, screen_h - bot_h, [0.22, 0.25, 0.30, 1.0]);
        ui.text(
            "[MMB] TOGGLE 4-VIEW   |   [WHEEL] ZOOM   |   [LMB] ORBIT/PAN   |   [RMB] PAN   |   [1-4] SWITCH",
            16.0,
            screen_h - 17.0,
            10.0,
            [0.65, 0.70, 0.76, 1.0],
        );

        // 4. Viewport Overlays & Dividers in 4-View Mode
        if self.layout == ViewportLayout::FourView {
            // Dividers
            ui.fill_rect(w_half - 1.0, top_y, 2.0, content_h, [0.20, 0.22, 0.26, 1.0]);
            ui.fill_rect(0.0, mid_y - 1.0, screen_w, 2.0, [0.20, 0.22, 0.26, 1.0]);

            // Active/Hovered viewport border highlight
            let active_id = self.hovered_viewport.unwrap_or(self.active_viewport);
            let (bx, by, bw, bh) = match active_id {
                ViewportId::Perspective => (0.0, top_y, w_half, h_half),
                ViewportId::Top => (w_half, top_y, screen_w - w_half, h_half),
                ViewportId::Right => (0.0, mid_y, w_half, content_h - h_half),
                ViewportId::Front => (w_half, mid_y, screen_w - w_half, content_h - h_half),
            };
            ui.stroke_rect(bx + 1.0, by + 1.0, bw - 2.0, bh - 2.0, [0.30, 0.70, 1.0, 0.85]);

            // Viewport title badges
            ui.fill_rect(8.0, top_y + 8.0, 130.0, 20.0, [0.10, 0.12, 0.16, 0.85]);
            ui.stroke_rect(8.0, top_y + 8.0, 130.0, 20.0, [0.25, 0.45, 0.75, 0.8]);
            ui.text("PERSPECTIVE 3D", 14.0, top_y + 12.0, 10.0, [0.4, 0.85, 1.0, 1.0]);

            ui.fill_rect(w_half + 8.0, top_y + 8.0, 105.0, 20.0, [0.10, 0.12, 0.16, 0.85]);
            ui.stroke_rect(w_half + 8.0, top_y + 8.0, 105.0, 20.0, [0.30, 0.35, 0.40, 0.8]);
            ui.text("TOP [ORTHO]", w_half + 14.0, top_y + 12.0, 10.0, [0.95, 0.95, 0.95, 1.0]);

            ui.fill_rect(8.0, mid_y + 8.0, 115.0, 20.0, [0.10, 0.12, 0.16, 0.85]);
            ui.stroke_rect(8.0, mid_y + 8.0, 115.0, 20.0, [0.30, 0.35, 0.40, 0.8]);
            ui.text("RIGHT [ORTHO]", 14.0, mid_y + 12.0, 10.0, [0.95, 0.95, 0.95, 1.0]);

            ui.fill_rect(w_half + 8.0, mid_y + 8.0, 115.0, 20.0, [0.10, 0.12, 0.16, 0.85]);
            ui.stroke_rect(w_half + 8.0, mid_y + 8.0, 115.0, 20.0, [0.30, 0.35, 0.40, 0.8]);
            ui.text("FRONT [ORTHO]", w_half + 14.0, mid_y + 12.0, 10.0, [0.95, 0.95, 0.95, 1.0]);

            // Coordinate axes in each viewport
            // 3D Perspective axes:
            let view_m = self.camera.view_matrix();
            let ax_ox = 35.0f32;
            let ax_oy = mid_y - 35.0f32;
            let len = 20.0f32;
            let rx = view_m[(0, 0)] * len;
            let ry = -view_m[(1, 0)] * len;
            let gx = view_m[(0, 1)] * len;
            let gy = -view_m[(1, 1)] * len;
            let bx_axis = view_m[(0, 2)] * len;
            let by_axis = -view_m[(1, 2)] * len;
            ui.line(ax_ox, ax_oy, ax_ox + rx, ax_oy + ry, [0.95, 0.25, 0.25, 1.0]);
            ui.text("X", ax_ox + rx + 3.0, ax_oy + ry - 4.0, 9.0, [0.95, 0.25, 0.25, 1.0]);
            ui.line(ax_ox, ax_oy, ax_ox + gx, ax_oy + gy, [0.25, 0.85, 0.35, 1.0]);
            ui.text("Y", ax_ox + gx + 3.0, ax_oy + gy - 4.0, 9.0, [0.25, 0.85, 0.35, 1.0]);
            ui.line(ax_ox, ax_oy, ax_ox + bx_axis, ax_oy + by_axis, [0.30, 0.55, 0.95, 1.0]);
            ui.text("Z", ax_ox + bx_axis + 3.0, ax_oy + by_axis - 4.0, 9.0, [0.30, 0.55, 0.95, 1.0]);

            // Top view axes: X right (red), Z down (blue)
            let tox = w_half + 35.0;
            let toy = mid_y - 35.0;
            ui.line(tox, toy, tox + len, toy, [0.95, 0.25, 0.25, 1.0]);
            ui.text("X", tox + len + 3.0, toy - 4.0, 9.0, [0.95, 0.25, 0.25, 1.0]);
            ui.line(tox, toy, tox, toy + len, [0.30, 0.55, 0.95, 1.0]);
            ui.text("Z", tox - 4.0, toy + len + 3.0, 9.0, [0.30, 0.55, 0.95, 1.0]);

            // Right view axes: Z right (blue), Y up (green)
            let rox = 35.0;
            let roy = screen_h - bot_h - 35.0;
            ui.line(rox, roy, rox + len, roy, [0.30, 0.55, 0.95, 1.0]);
            ui.text("Z", rox + len + 3.0, roy - 4.0, 9.0, [0.30, 0.55, 0.95, 1.0]);
            ui.line(rox, roy, rox, roy - len, [0.25, 0.85, 0.35, 1.0]);
            ui.text("Y", rox - 4.0, roy - len - 10.0, 9.0, [0.25, 0.85, 0.35, 1.0]);

            // Front view axes: X right (red), Y up (green)
            let fox = w_half + 35.0;
            let foy = screen_h - bot_h - 35.0;
            ui.line(fox, foy, fox + len, foy, [0.95, 0.25, 0.25, 1.0]);
            ui.text("X", fox + len + 3.0, foy - 4.0, 9.0, [0.95, 0.25, 0.25, 1.0]);
            ui.line(fox, foy, fox, foy - len, [0.25, 0.85, 0.35, 1.0]);
            ui.text("Y", fox - 4.0, foy - len - 10.0, 9.0, [0.25, 0.85, 0.35, 1.0]);
        } else if let ViewportLayout::Single(id) = self.layout {
            let (name, is_ortho) = match id {
                ViewportId::Perspective => ("PERSPECTIVE 3D", false),
                ViewportId::Top => ("TOP [ORTHO]", true),
                ViewportId::Right => ("RIGHT [ORTHO]", true),
                ViewportId::Front => ("FRONT [ORTHO]", true),
            };
            ui.fill_rect(8.0, top_y + 8.0, 130.0, 20.0, [0.10, 0.12, 0.16, 0.85]);
            ui.stroke_rect(
                8.0,
                top_y + 8.0,
                130.0,
                20.0,
                if is_ortho { [0.35, 0.40, 0.45, 0.8] } else { [0.25, 0.45, 0.75, 0.8] },
            );
            ui.text(
                name,
                14.0,
                top_y + 12.0,
                10.0,
                if is_ortho { [0.95, 0.95, 0.95, 1.0] } else { [0.4, 0.85, 1.0, 1.0] },
            );
        }

        // Upload UI Vertex Buffers
        let bg_quad_vb = if !bg_ui.quads.is_empty() {
            Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Window BG Quad VB"),
                contents: bytemuck::cast_slice(&bg_ui.quads),
                usage: wgpu::BufferUsages::VERTEX,
            }))
        } else {
            None
        };

        let ui_quad_vb = if !ui.quads.is_empty() {
            Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Window UI Quad VB"),
                contents: bytemuck::cast_slice(&ui.quads),
                usage: wgpu::BufferUsages::VERTEX,
            }))
        } else {
            None
        };

        let ui_line_vb = if !ui.lines.is_empty() {
            Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Window UI Line VB"),
                contents: bytemuck::cast_slice(&ui.lines),
                usage: wgpu::BufferUsages::VERTEX,
            }))
        } else {
            None
        };

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Window Redraw Encoder"),
        });

        {
            let depth_stencil = self.depth_view.as_ref().map(|dv| wgpu::RenderPassDepthStencilAttachment {
                view: dv,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            });

            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Window Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.15,
                            g: 0.17,
                            b: 0.20,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: depth_stencil,
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            // 1. Draw Ortho Backgrounds
            if let (Some(pipeline), Some(vb)) = (&self.ui_quad_pipeline, &bg_quad_vb) {
                render_pass.set_viewport(0.0, 0.0, screen_w, screen_h, 0.0, 1.0);
                render_pass.set_scissor_rect(0, 0, screen_w as u32, screen_h as u32);
                render_pass.set_pipeline(pipeline);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.draw(0..bg_ui.quads.len() as u32, 0..1);
            }

            // 2. Draw 3D solid and 2D wireframe geometry
            let (solid_pipeline, solid_bind_group) = match self.current_mode {
                RenderMode::Cad => (self.cad_pipeline.as_ref(), self.cad_bind_group.as_ref()),
                RenderMode::Fea => (self.fea_pipeline.as_ref(), self.fea_bind_group.as_ref()),
            };

            match self.layout {
                ViewportLayout::Single(id) => {
                    render_pass.set_viewport(0.0, top_y, screen_w, content_h, 0.0, 1.0);
                    render_pass.set_scissor_rect(0, top_y as u32, screen_w as u32, content_h as u32);

                    if id == ViewportId::Perspective {
                        if let (Some(pipeline), Some(bg), Some(vb), Some(ib)) = (
                            solid_pipeline,
                            solid_bind_group,
                            &self.vertex_buffer,
                            &self.index_buffer,
                        ) {
                            render_pass.set_pipeline(pipeline);
                            render_pass.set_bind_group(0, bg, &[]);
                            render_pass.set_vertex_buffer(0, vb.slice(..));
                            render_pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                            render_pass.draw_indexed(0..self.index_count, 0, 0..1);
                        }
                    } else {
                        let bg_opt = match id {
                            ViewportId::Top => self.wireframe_bind_group_top.as_ref(),
                            ViewportId::Right => self.wireframe_bind_group_right.as_ref(),
                            ViewportId::Front => self.wireframe_bind_group_front.as_ref(),
                            _ => None,
                        };
                        if let (Some(pipeline), Some(bg), Some(vb), Some(wib)) = (
                            &self.wireframe_pipeline,
                            bg_opt,
                            &self.vertex_buffer,
                            &self.wireframe_index_buffer,
                        ) {
                            render_pass.set_pipeline(pipeline);
                            render_pass.set_bind_group(0, bg, &[]);
                            render_pass.set_vertex_buffer(0, vb.slice(..));
                            render_pass.set_index_buffer(wib.slice(..), wgpu::IndexFormat::Uint32);
                            render_pass.draw_indexed(0..self.wireframe_index_count, 0, 0..1);
                        }
                    }
                }
                ViewportLayout::FourView => {
                    // Quadrant 0: Perspective 3D
                    render_pass.set_viewport(0.0, top_y, w_half, h_half, 0.0, 1.0);
                    render_pass.set_scissor_rect(0, top_y as u32, w_half as u32, h_half as u32);
                    if let (Some(pipeline), Some(bg), Some(vb), Some(ib)) = (
                        solid_pipeline,
                        solid_bind_group,
                        &self.vertex_buffer,
                        &self.index_buffer,
                    ) {
                        render_pass.set_pipeline(pipeline);
                        render_pass.set_bind_group(0, bg, &[]);
                        render_pass.set_vertex_buffer(0, vb.slice(..));
                        render_pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(0..self.index_count, 0, 0..1);
                    }

                    // Quadrants 1-3: Top, Right, Front (Ortho Wireframe)
                    if let (Some(pipeline), Some(vb), Some(wib)) = (
                        &self.wireframe_pipeline,
                        &self.vertex_buffer,
                        &self.wireframe_index_buffer,
                    ) {
                        render_pass.set_pipeline(pipeline);
                        render_pass.set_vertex_buffer(0, vb.slice(..));
                        render_pass.set_index_buffer(wib.slice(..), wgpu::IndexFormat::Uint32);

                        // Quadrant 1: Top
                        if let Some(bg) = &self.wireframe_bind_group_top {
                            render_pass.set_viewport(w_half, top_y, screen_w - w_half, h_half, 0.0, 1.0);
                            render_pass.set_scissor_rect(w_half as u32, top_y as u32, (screen_w - w_half) as u32, h_half as u32);
                            render_pass.set_bind_group(0, bg, &[]);
                            render_pass.draw_indexed(0..self.wireframe_index_count, 0, 0..1);
                        }

                        // Quadrant 2: Right
                        if let Some(bg) = &self.wireframe_bind_group_right {
                            render_pass.set_viewport(0.0, mid_y, w_half, content_h - h_half, 0.0, 1.0);
                            render_pass.set_scissor_rect(0, mid_y as u32, w_half as u32, (content_h - h_half) as u32);
                            render_pass.set_bind_group(0, bg, &[]);
                            render_pass.draw_indexed(0..self.wireframe_index_count, 0, 0..1);
                        }

                        // Quadrant 3: Front
                        if let Some(bg) = &self.wireframe_bind_group_front {
                            render_pass.set_viewport(w_half, mid_y, screen_w - w_half, content_h - h_half, 0.0, 1.0);
                            render_pass.set_scissor_rect(w_half as u32, mid_y as u32, (screen_w - w_half) as u32, (content_h - h_half) as u32);
                            render_pass.set_bind_group(0, bg, &[]);
                            render_pass.draw_indexed(0..self.wireframe_index_count, 0, 0..1);
                        }
                    }
                }
            }

            // 3. Render 2D UI Overlay (Top bar, bottom bar, badges, dividers)
            render_pass.set_viewport(0.0, 0.0, screen_w, screen_h, 0.0, 1.0);
            render_pass.set_scissor_rect(0, 0, screen_w as u32, screen_h as u32);

            if let (Some(pipeline), Some(vb)) = (&self.ui_quad_pipeline, &ui_quad_vb) {
                render_pass.set_pipeline(pipeline);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.draw(0..ui.quads.len() as u32, 0..1);
            }

            if let (Some(pipeline), Some(vb)) = (&self.ui_line_pipeline, &ui_line_vb) {
                render_pass.set_pipeline(pipeline);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.draw(0..ui.lines.len() as u32, 0..1);
            }
        }

        queue.submit(std::iter::once(encoder.finish()));
        output.present();
    }

    /// Resets all window and surface resources when the window is closed.
    fn reset_window_resources(&mut self) {
        self.window = None;
        self.surface = None;
        self.surface_config = None;
        self.depth_view = None;
        self.device = None;
        self.queue = None;
        self.cad_pipeline = None;
        self.cad_bind_group_layout = None;
        self.cad_bind_group = None;
        self.fea_pipeline = None;
        self.fea_bind_group_layout = None;
        self.fea_bind_group = None;
        self.wireframe_pipeline = None;
        self.wireframe_bind_group_layout = None;
        self.wireframe_bind_group_persp = None;
        self.wireframe_bind_group_top = None;
        self.wireframe_bind_group_right = None;
        self.wireframe_bind_group_front = None;
        self.ui_quad_pipeline = None;
        self.ui_line_pipeline = None;
        self.vertex_buffer = None;
        self.index_buffer = None;
        self.wireframe_index_buffer = None;
        self.camera_buffer_persp = None;
        self.camera_buffer_top = None;
        self.camera_buffer_right = None;
        self.camera_buffer_front = None;
        self.material_buffer = None;
        self.wireframe_uniform_buffer = None;
    }
}

impl ApplicationHandler<GuiEvent> for CadWindowApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.ensure_window(event_loop);
        if let Some(w) = &self.window {
            w.request_redraw();
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: GuiEvent) {
        match event {
            GuiEvent::Show => {
                self.ensure_window(event_loop);
                if let Some(w) = &self.window {
                    w.focus_window();
                    w.request_redraw();
                }
            }
            GuiEvent::UpdateMesh(mesh) => {
                self.ensure_window(event_loop);
                self.upload_cad_mesh(&mesh);
                if let Some(w) = &self.window {
                    w.focus_window();
                    w.request_redraw();
                }
            }
            GuiEvent::UpdateFea { mesh, stresses } => {
                self.ensure_window(event_loop);
                self.upload_fea_mesh(&mesh, &stresses);
                if let Some(w) = &self.window {
                    w.focus_window();
                    w.request_redraw();
                }
            }
            GuiEvent::Close => {
                event_loop.exit();
            }
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                if self.exit_on_close {
                    event_loop.exit();
                } else {
                    self.reset_window_resources();
                }
            }
            WindowEvent::Resized(new_size) => {
                if new_size.width > 0 && new_size.height > 0 {
                    if let (Some(surface), Some(device), Some(config)) = (
                        &self.surface,
                        &self.device,
                        &mut self.surface_config,
                    ) {
                        config.width = new_size.width;
                        config.height = new_size.height;
                        surface.configure(device, config);
                    }
                    self.update_camera_aspects(new_size.width, new_size.height);
                    if let Some(device) = &self.device {
                        self.depth_view = Some(create_depth_texture(device, new_size.width, new_size.height));
                    }
                    self.update_camera_buffer();
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                let pressed = state == ElementState::Pressed;
                match button {
                    MouseButton::Left => {
                        self.is_mouse_down = pressed;
                        if pressed {
                            let (screen_w, screen_h) = if let Some(cfg) = &self.surface_config {
                                (cfg.width as f32, cfg.height as f32)
                            } else {
                                (1024.0, 768.0)
                            };

                            if let Some((mx_f64, my_f64)) = self.last_mouse_pos {
                                let mx = mx_f64 as f32;
                                let my = my_f64 as f32;

                                // Check Top Bar buttons
                                if my <= 32.0 {
                                    if (95.0..=167.0).contains(&mx) {
                                        self.layout = ViewportLayout::FourView;
                                    } else if (173.0..=238.0).contains(&mx) {
                                        self.layout = ViewportLayout::Single(ViewportId::Perspective);
                                    } else if (244.0..=299.0).contains(&mx) {
                                        self.layout = ViewportLayout::Single(ViewportId::Top);
                                    } else if (305..=370).contains(&(mx as i32)) {
                                        self.layout = ViewportLayout::Single(ViewportId::Right);
                                    } else if (376..=441).contains(&(mx as i32)) {
                                        self.layout = ViewportLayout::Single(ViewportId::Front);
                                    } else if (447..=497).contains(&(mx as i32)) {
                                        self.fit_all_cameras();
                                    }
                                    if let Some(cfg) = &self.surface_config {
                                        self.update_camera_aspects(cfg.width, cfg.height);
                                    }
                                    self.update_camera_buffer();
                                    if let Some(w) = &self.window {
                                        w.request_redraw();
                                    }
                                } else if let Some(v) = self.calculate_viewport_at(mx, my, screen_w, screen_h) {
                                    self.active_viewport = v;
                                    if let Some(w) = &self.window {
                                        w.request_redraw();
                                    }
                                }
                            }
                        }
                    }
                    MouseButton::Right => {
                        self.is_rmb_down = pressed;
                    }
                    MouseButton::Middle => {
                        if pressed {
                            let (screen_w, screen_h) = if let Some(cfg) = &self.surface_config {
                                (cfg.width as f32, cfg.height as f32)
                            } else {
                                (1024.0, 768.0)
                            };

                            let (mx, my) = if let Some((mx_f64, my_f64)) = self.last_mouse_pos {
                                (mx_f64 as f32, my_f64 as f32)
                            } else {
                                (screen_w * 0.5, screen_h * 0.5)
                            };

                            // Cinema 4D style MMB view toggle
                            match self.layout {
                                ViewportLayout::FourView => {
                                    let target_view = self
                                        .calculate_viewport_at(mx, my, screen_w, screen_h)
                                        .unwrap_or(self.active_viewport);
                                    self.layout = ViewportLayout::Single(target_view);
                                    self.active_viewport = target_view;
                                }
                                ViewportLayout::Single(_) => {
                                    self.layout = ViewportLayout::FourView;
                                }
                            }

                            if let Some(cfg) = &self.surface_config {
                                self.update_camera_aspects(cfg.width, cfg.height);
                            }
                            self.update_camera_buffer();
                            if let Some(w) = &self.window {
                                w.request_redraw();
                            }
                        }
                    }
                    _ => {}
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                let (screen_w, screen_h) = if let Some(cfg) = &self.surface_config {
                    (cfg.width as f32, cfg.height as f32)
                } else {
                    (1024.0, 768.0)
                };

                let cur_x = position.x as f32;
                let cur_y = position.y as f32;
                let prev_hover = self.hovered_viewport;
                self.hovered_viewport = self.calculate_viewport_at(cur_x, cur_y, screen_w, screen_h);

                if let Some((last_x_f64, last_y_f64)) = self.last_mouse_pos {
                    let dx = cur_x - last_x_f64 as f32;
                    let dy = cur_y - last_y_f64 as f32;

                    if self.is_mouse_down {
                        // Left-drag: Orbit in Perspective, Pan in Orthographic
                        let target_view = match self.layout {
                            ViewportLayout::Single(v) => v,
                            ViewportLayout::FourView => self.active_viewport,
                        };

                        match target_view {
                            ViewportId::Perspective => {
                                self.camera.yaw += dx * 0.01;
                                self.camera.pitch = (self.camera.pitch - dy * 0.01)
                                    .clamp(-std::f32::consts::FRAC_PI_2 + 0.01, std::f32::consts::FRAC_PI_2 - 0.01);
                            }
                            ViewportId::Top => {
                                self.camera_top.pan(dx, dy);
                                self.camera.target = self.camera_top.target;
                                self.camera_right.target = self.camera_top.target;
                                self.camera_front.target = self.camera_top.target;
                            }
                            ViewportId::Right => {
                                self.camera_right.pan(dx, dy);
                                self.camera.target = self.camera_right.target;
                                self.camera_top.target = self.camera_right.target;
                                self.camera_front.target = self.camera_right.target;
                            }
                            ViewportId::Front => {
                                self.camera_front.pan(dx, dy);
                                self.camera.target = self.camera_front.target;
                                self.camera_top.target = self.camera_front.target;
                                self.camera_right.target = self.camera_front.target;
                            }
                        }

                        self.update_camera_buffer();
                        if let Some(w) = &self.window {
                            w.request_redraw();
                        }
                    } else if self.is_rmb_down {
                        // Right-drag: Pan target across view plane
                        let target_view = match self.layout {
                            ViewportLayout::Single(v) => v,
                            ViewportLayout::FourView => self.active_viewport,
                        };

                        match target_view {
                            ViewportId::Perspective => self.camera.pan(dx, dy),
                            ViewportId::Top => self.camera_top.pan(dx, dy),
                            ViewportId::Right => self.camera_right.pan(dx, dy),
                            ViewportId::Front => self.camera_front.pan(dx, dy),
                        }

                        let shared_target = match target_view {
                            ViewportId::Perspective => self.camera.target,
                            ViewportId::Top => self.camera_top.target,
                            ViewportId::Right => self.camera_right.target,
                            ViewportId::Front => self.camera_front.target,
                        };
                        self.camera.target = shared_target;
                        self.camera_top.target = shared_target;
                        self.camera_right.target = shared_target;
                        self.camera_front.target = shared_target;

                        self.update_camera_buffer();
                        if let Some(w) = &self.window {
                            w.request_redraw();
                        }
                    }
                }

                if prev_hover != self.hovered_viewport {
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                self.last_mouse_pos = Some((position.x, position.y));
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll_amount = match delta {
                    winit::event::MouseScrollDelta::LineDelta(_, y) => y,
                    winit::event::MouseScrollDelta::PixelDelta(p) => (p.y as f32) * 0.05,
                };
                // Proportional multiplicative zoom
                let zoom_factor = (-scroll_amount * 0.15).exp();

                let (screen_w, screen_h) = if let Some(cfg) = &self.surface_config {
                    (cfg.width as f32, cfg.height as f32)
                } else {
                    (1024.0, 768.0)
                };

                let hovered = if let Some((mx_f64, my_f64)) = self.last_mouse_pos {
                    self.calculate_viewport_at(mx_f64 as f32, my_f64 as f32, screen_w, screen_h)
                } else {
                    None
                };

                match self.layout {
                    ViewportLayout::Single(v) => match v {
                        ViewportId::Perspective => self.camera.zoom_by(zoom_factor),
                        ViewportId::Top => self.camera_top.zoom_by(zoom_factor),
                        ViewportId::Right => self.camera_right.zoom_by(zoom_factor),
                        ViewportId::Front => self.camera_front.zoom_by(zoom_factor),
                    },
                    ViewportLayout::FourView => {
                        let target_view = hovered.unwrap_or(self.active_viewport);
                        match target_view {
                            ViewportId::Perspective => {
                                self.camera.zoom_by(zoom_factor);
                            }
                            ViewportId::Top | ViewportId::Right | ViewportId::Front => {
                                // Keep ortho views at synchronized engineering drawing scale
                                self.camera_top.zoom_by(zoom_factor);
                                self.camera_right.zoom_by(zoom_factor);
                                self.camera_front.zoom_by(zoom_factor);
                            }
                        }
                    }
                }

                self.update_camera_buffer();
                if let Some(w) = &self.window {
                    w.request_redraw();
                }
            }
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        physical_key: PhysicalKey::Code(key_code),
                        state: ElementState::Pressed,
                        ..
                    },
                ..
            } => match key_code {
                KeyCode::Escape => {
                    if self.exit_on_close {
                        event_loop.exit();
                    } else {
                        self.reset_window_resources();
                    }
                }
                KeyCode::Space | KeyCode::F4 => {
                    // Cinema 4D space / F4 view toggle
                    self.layout = match self.layout {
                        ViewportLayout::FourView => ViewportLayout::Single(self.active_viewport),
                        ViewportLayout::Single(_) => ViewportLayout::FourView,
                    };
                    if let Some(cfg) = &self.surface_config {
                        self.update_camera_aspects(cfg.width, cfg.height);
                    }
                    self.update_camera_buffer();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                KeyCode::Digit1 => {
                    self.layout = ViewportLayout::Single(ViewportId::Perspective);
                    self.active_viewport = ViewportId::Perspective;
                    if let Some(cfg) = &self.surface_config {
                        self.update_camera_aspects(cfg.width, cfg.height);
                    }
                    self.update_camera_buffer();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                KeyCode::Digit2 => {
                    self.layout = ViewportLayout::Single(ViewportId::Top);
                    self.active_viewport = ViewportId::Top;
                    if let Some(cfg) = &self.surface_config {
                        self.update_camera_aspects(cfg.width, cfg.height);
                    }
                    self.update_camera_buffer();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                KeyCode::Digit3 => {
                    self.layout = ViewportLayout::Single(ViewportId::Right);
                    self.active_viewport = ViewportId::Right;
                    if let Some(cfg) = &self.surface_config {
                        self.update_camera_aspects(cfg.width, cfg.height);
                    }
                    self.update_camera_buffer();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                KeyCode::Digit4 => {
                    self.layout = ViewportLayout::Single(ViewportId::Front);
                    self.active_viewport = ViewportId::Front;
                    if let Some(cfg) = &self.surface_config {
                        self.update_camera_aspects(cfg.width, cfg.height);
                    }
                    self.update_camera_buffer();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                KeyCode::KeyI => {
                    self.camera.apply_preset(CameraPreset::Isometric);
                    self.update_camera_buffer();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                KeyCode::KeyF => {
                    self.fit_all_cameras();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                KeyCode::KeyT => {
                    self.camera.apply_preset(CameraPreset::Top);
                    self.update_camera_buffer();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                KeyCode::KeyR => {
                    self.camera.apply_preset(CameraPreset::Right);
                    self.update_camera_buffer();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                _ => {}
            },
            WindowEvent::RedrawRequested => {
                self.render_frame();
            }
            _ => {}
        }
    }
}

fn create_depth_texture(device: &wgpu::Device, width: u32, height: u32) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("Window Depth Texture"),
        size: wgpu::Extent3d {
            width: width.max(1),
            height: height.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth24Plus,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    texture.create_view(&wgpu::TextureViewDescriptor::default())
}
