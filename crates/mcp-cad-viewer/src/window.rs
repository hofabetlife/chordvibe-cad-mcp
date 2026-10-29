//! Interactive native Windows GUI viewport using WGPU and Winit 0.30.

use std::sync::Arc;
use wgpu::util::DeviceExt;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowAttributes, WindowId};

use mcp_cad_core::topology::TriangleMesh;
use crate::camera::{CameraPreset, OrbitCamera};
use crate::error::ViewerError;
use crate::shader::{CadVertex, MaterialUniform, CAD_SHADER_WGSL};

/// Interactive 3D CAD window state and renderer.
pub struct CadWindowViewer {
    title: String,
    mesh: Option<TriangleMesh>,
    stress_scalars: Option<Vec<f32>>,
    camera: OrbitCamera,
}

impl CadWindowViewer {
    /// Creates a new CadWindowViewer instance.
    pub fn new(title: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            mesh: None,
            stress_scalars: None,
            camera: OrbitCamera::default(),
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

    /// Returns a reference to the configured mesh, if any.
    pub fn mesh(&self) -> Option<&TriangleMesh> {
        self.mesh.as_ref()
    }

    /// Returns a reference to the configured stress scalars, if any.
    pub fn stress_scalars(&self) -> Option<&[f32]> {
        self.stress_scalars.as_deref()
    }

    /// Runs the interactive window event loop.
    pub fn run(self) -> Result<(), ViewerError> {
        let event_loop = EventLoop::new().map_err(ViewerError::EventLoopError)?;
        event_loop.set_control_flow(ControlFlow::Poll);

        let mut app = CadWindowApp {
            title: self.title,
            mesh: self.mesh,
            _stress_scalars: self.stress_scalars,
            camera: self.camera,
            window: None,
            surface: None,
            device: None,
            queue: None,
            surface_config: None,
            depth_view: None,
            pipeline: None,
            vertex_buffer: None,
            index_buffer: None,
            camera_buffer: None,
            bind_group: None,
            index_count: 0,
            is_mouse_down: false,
            last_mouse_pos: None,
        };

        event_loop.run_app(&mut app).map_err(ViewerError::EventLoopError)?;
        Ok(())
    }
}

/// Internal application handler implementing Winit 0.30 ApplicationHandler.
struct CadWindowApp {
    title: String,
    mesh: Option<TriangleMesh>,
    _stress_scalars: Option<Vec<f32>>,
    camera: OrbitCamera,
    window: Option<Arc<Window>>,
    surface: Option<wgpu::Surface<'static>>,
    device: Option<wgpu::Device>,
    queue: Option<wgpu::Queue>,
    surface_config: Option<wgpu::SurfaceConfiguration>,
    depth_view: Option<wgpu::TextureView>,
    pipeline: Option<wgpu::RenderPipeline>,
    vertex_buffer: Option<wgpu::Buffer>,
    index_buffer: Option<wgpu::Buffer>,
    camera_buffer: Option<wgpu::Buffer>,
    bind_group: Option<wgpu::BindGroup>,
    index_count: u32,
    is_mouse_down: bool,
    last_mouse_pos: Option<(f64, f64)>,
}

impl ApplicationHandler for CadWindowApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        let attrs = WindowAttributes::default()
            .with_title(&self.title)
            .with_inner_size(winit::dpi::LogicalSize::new(1024, 768));

        let window = match event_loop.create_window(attrs) {
            Ok(w) => Arc::new(w),
            Err(e) => {
                eprintln!("Failed to create window: {e}");
                event_loop.exit();
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
                event_loop.exit();
                return;
            }
        };

        let adapter = match pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            compatible_surface: Some(&surface),
        })) {
            Some(a) => a,
            None => {
                eprintln!("Failed to acquire graphics adapter");
                event_loop.exit();
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
                event_loop.exit();
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

        self.camera.aspect = width as f32 / height as f32;

        // Depth texture
        let depth_view = create_depth_texture(&device, width, height);

        // Pipeline and geometry
        if let Some(mesh) = &self.mesh {
            if !mesh.vertices.is_empty() && !mesh.indices.is_empty() {
                let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                    label: Some("Window CAD Shader"),
                    source: wgpu::ShaderSource::Wgsl(CAD_SHADER_WGSL.into()),
                });

                let bind_group_layout =
                    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                        label: Some("Window Bind Group Layout"),
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

                let pipeline_layout =
                    device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                        label: Some("Window Pipeline Layout"),
                        bind_group_layouts: &[&bind_group_layout],
                        push_constant_ranges: &[],
                    });

                let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("Window Render Pipeline"),
                    layout: Some(&pipeline_layout),
                    vertex: wgpu::VertexState {
                        module: &shader,
                        entry_point: "vs_main",
                        buffers: &[CadVertex::LAYOUT],
                        compilation_options: Default::default(),
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &shader,
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

                let vb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Window VB"),
                    contents: bytemuck::cast_slice(&cad_verts),
                    usage: wgpu::BufferUsages::VERTEX,
                });

                let ib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Window IB"),
                    contents: bytemuck::cast_slice(&flat_indices),
                    usage: wgpu::BufferUsages::INDEX,
                });

                let cam_uniform = self.camera.build_uniform();
                let cb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Window Camera Buffer"),
                    contents: bytemuck::bytes_of(&cam_uniform),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });

                let mat_uniform = MaterialUniform::default();
                let mb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Window Material Buffer"),
                    contents: bytemuck::bytes_of(&mat_uniform),
                    usage: wgpu::BufferUsages::UNIFORM,
                });

                let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("Window Bind Group"),
                    layout: &bind_group_layout,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: cb.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: mb.as_entire_binding(),
                        },
                    ],
                });

                self.index_count = flat_indices.len() as u32;
                self.vertex_buffer = Some(vb);
                self.index_buffer = Some(ib);
                self.camera_buffer = Some(cb);
                self.bind_group = Some(bg);
                self.pipeline = Some(pipeline);
            }
        }

        self.depth_view = Some(depth_view);
        self.window = Some(window);
        self.surface = Some(surface);
        self.device = Some(device);
        self.queue = Some(queue);
        self.surface_config = Some(config);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::Resized(new_size) => {
                if let (Some(surface), Some(device), Some(config)) = (
                    &self.surface,
                    &self.device,
                    &mut self.surface_config,
                ) {
                    if new_size.width > 0 && new_size.height > 0 {
                        config.width = new_size.width;
                        config.height = new_size.height;
                        surface.configure(device, config);
                        self.camera.aspect = config.width as f32 / config.height as f32;
                        self.depth_view = Some(create_depth_texture(device, config.width, config.height));
                        self.update_camera_buffer();
                    }
                }
            }
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Left,
                ..
            } => {
                self.is_mouse_down = state == ElementState::Pressed;
            }
            WindowEvent::CursorMoved { position, .. } => {
                if let Some((last_x, last_y)) = self.last_mouse_pos {
                    if self.is_mouse_down {
                        let dx = (position.x - last_x) as f32;
                        let dy = (position.y - last_y) as f32;

                        self.camera.yaw += dx * 0.01;
                        self.camera.pitch = (self.camera.pitch - dy * 0.01)
                            .clamp(-std::f32::consts::FRAC_PI_2 + 0.01, std::f32::consts::FRAC_PI_2 - 0.01);

                        self.update_camera_buffer();
                        if let Some(w) = &self.window {
                            w.request_redraw();
                        }
                    }
                }
                self.last_mouse_pos = Some((position.x, position.y));
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let zoom = match delta {
                    winit::event::MouseScrollDelta::LineDelta(_, y) => y * 0.5,
                    winit::event::MouseScrollDelta::PixelDelta(p) => (p.y as f32) * 0.01,
                };
                self.camera.distance = (self.camera.distance - zoom).max(0.1);
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
                KeyCode::Escape => event_loop.exit(),
                KeyCode::KeyI => {
                    self.camera.apply_preset(CameraPreset::Isometric);
                    self.update_camera_buffer();
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
                KeyCode::KeyF => {
                    self.camera.apply_preset(CameraPreset::Front);
                    self.update_camera_buffer();
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

impl CadWindowApp {
    fn update_camera_buffer(&self) {
        if let (Some(queue), Some(cb)) = (&self.queue, &self.camera_buffer) {
            let uniform = self.camera.build_uniform();
            queue.write_buffer(cb, 0, bytemuck::bytes_of(&uniform));
        }
    }

    fn render_frame(&mut self) {
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

            if let (Some(pipeline), Some(bg), Some(vb), Some(ib)) = (
                &self.pipeline,
                &self.bind_group,
                &self.vertex_buffer,
                &self.index_buffer,
            ) {
                render_pass.set_pipeline(pipeline);
                render_pass.set_bind_group(0, bg, &[]);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..self.index_count, 0, 0..1);
            }
        }

        queue.submit(std::iter::once(encoder.finish()));
        output.present();
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
