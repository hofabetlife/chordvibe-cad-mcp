//! Headless offscreen renderer using WGPU for CAD mesh visualization and FEA colormaps.

use std::sync::mpsc::channel;
use base64::Engine;
use wgpu::util::DeviceExt;
use mcp_cad_core::part::Part;
use mcp_cad_core::tessellation::{TessellationQuality, TessellationSettings};
use mcp_cad_core::topology::TriangleMesh;

use crate::camera::{CameraPreset, OrbitCamera};
use crate::error::ViewerError;
use crate::shader::{
    CadVertex, FeaVertex, MaterialUniform, WireframeUniform, CAD_SHADER_WGSL,
    FEA_HEATMAP_SHADER_WGSL, WIREFRAME_SHADER_WGSL,
};
use crate::window::extract_wireframe_indices;

/// Default width in pixels for offscreen rendering.
pub const DEFAULT_RENDER_WIDTH: u32 = 800;
/// Default height in pixels for offscreen rendering.
pub const DEFAULT_RENDER_HEIGHT: u32 = 600;

/// Debug rendering modes for geometry inspection and defect audit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DebugRenderMode {
    #[default]
    None,
    /// Triangle wireframe lines over solid fill
    Wireframe,
    /// Surface normals mapped to RGB color
    Normals,
    /// Inverted backfaces highlighted in vivid magenta
    Backfaces,
    /// Open boundary edges and non-manifold edges highlighted in vivid amber-yellow
    BoundaryEdges,
    /// Pure per-facet flat shading without normal smoothing
    FlatShading,
}

impl DebugRenderMode {
    pub fn parse(s: Option<&str>) -> Self {
        match s.map(|v| v.to_lowercase()).as_deref() {
            Some("wireframe") => Self::Wireframe,
            Some("normals") => Self::Normals,
            Some("backfaces") => Self::Backfaces,
            Some("boundary_edges") | Some("boundary") => Self::BoundaryEdges,
            Some("flat_shading") | Some("flat") => Self::FlatShading,
            _ => Self::None,
        }
    }
}

/// Extracts edge indices that do not belong to exactly 2 triangles after spatial vertex welding
/// (true open boundary edges and non-manifold edges, excluding split-normal CAD creases).
pub fn extract_boundary_indices(mesh: &TriangleMesh) -> Vec<u32> {
    use std::collections::HashMap;
    if mesh.vertices.is_empty() || mesh.indices.is_empty() {
        return Vec::new();
    }

    let aabb = mesh.bounding_box();
    let dims = aabb.dimensions();
    let diag = (dims[0] * dims[0] + dims[1] * dims[1] + dims[2] * dims[2]).sqrt();
    let weld_eps = (diag * 1e-7).max(1e-12);
    let weld_scale = 1.0 / weld_eps;
    let weld_eps_sq = weld_eps * weld_eps;

    let mut cell_to_welded: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
    let mut vertex_to_welded: Vec<u32> = Vec::with_capacity(mesh.vertices.len());
    let mut welded_positions: Vec<[f64; 3]> = Vec::new();

    for v in &mesh.vertices {
        let p = v.position;
        let base_cell = [
            (p[0] * weld_scale).round() as i64,
            (p[1] * weld_scale).round() as i64,
            (p[2] * weld_scale).round() as i64,
        ];
        let mut found_idx = None;
        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let cell = [base_cell[0] + dx, base_cell[1] + dy, base_cell[2] + dz];
                    if let Some(cands) = cell_to_welded.get(&cell) {
                        for &cand in cands {
                            let cp = welded_positions[cand as usize];
                            let d2 = (cp[0] - p[0]).powi(2)
                                + (cp[1] - p[1]).powi(2)
                                + (cp[2] - p[2]).powi(2);
                            if d2 <= weld_eps_sq {
                                found_idx = Some(cand);
                                break 'search;
                            }
                        }
                    }
                }
            }
        }
        let w_idx = match found_idx {
            Some(idx) => idx,
            None => {
                let idx = welded_positions.len() as u32;
                cell_to_welded.entry(base_cell).or_default().push(idx);
                welded_positions.push(p);
                idx
            }
        };
        vertex_to_welded.push(w_idx);
    }

    let mut edge_counts: HashMap<(u32, u32), (usize, (u32, u32))> = HashMap::new();
    for tri in &mesh.indices {
        for &(orig_u, orig_v) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let wu = vertex_to_welded[orig_u as usize];
            let wv = vertex_to_welded[orig_v as usize];
            if wu == wv {
                continue;
            }
            let key = if wu < wv { (wu, wv) } else { (wv, wu) };
            let entry = edge_counts.entry(key).or_insert((0, (orig_u, orig_v)));
            entry.0 += 1;
        }
    }

    let mut line_indices = Vec::new();
    for (_welded_edge, (count, (orig_u, orig_v))) in edge_counts {
        if count != 2 {
            line_indices.push(orig_u);
            line_indices.push(orig_v);
        }
    }
    line_indices
}

/// Headless WGPU offscreen renderer for capturing PNG snapshots and Base64 images.
pub struct OffscreenRenderer {
    /// WGPU Logical Device.
    pub device: wgpu::Device,
    /// WGPU Command Queue.
    pub queue: wgpu::Queue,
    /// Render pipeline for standard CAD solid shading.
    cad_pipeline: wgpu::RenderPipeline,
    /// Bind group layout for CAD uniforms.
    cad_bind_group_layout: wgpu::BindGroupLayout,
    /// Render pipeline for FEA stress heatmap rendering.
    fea_pipeline: wgpu::RenderPipeline,
    /// Bind group layout for FEA uniforms.
    fea_bind_group_layout: wgpu::BindGroupLayout,
    /// Render pipeline for wireframe and boundary line overlay.
    wireframe_pipeline: wgpu::RenderPipeline,
    /// Bind group layout for wireframe uniforms.
    wireframe_bind_group_layout: wgpu::BindGroupLayout,
}

impl OffscreenRenderer {
    /// Initializes a new headless OffscreenRenderer instance.
    ///
    /// Discovers an available GPU adapter (DirectX 12, Vulkan, or software WARP rasterizer)
    /// without requiring an OS display window.
    pub fn new() -> Result<Self, ViewerError> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let adapter = pollster::block_on(async {
            // First attempt: High performance discrete GPU
            if let Some(adapter) = instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::HighPerformance,
                    force_fallback_adapter: false,
                    compatible_surface: None,
                })
                .await
            {
                return Some(adapter);
            }

            // Second attempt: Default adapter
            if let Some(adapter) = instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::None,
                    force_fallback_adapter: false,
                    compatible_surface: None,
                })
                .await
            {
                return Some(adapter);
            }

            // Third attempt: Fallback software rasterizer (e.g. WARP on Windows)
            instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::None,
                    force_fallback_adapter: true,
                    compatible_surface: None,
                })
                .await
        })
        .ok_or(ViewerError::NoSuitableAdapter)?;

        let (device, queue) = pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("MCP-CAD Headless Device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::Performance,
            },
            None,
        ))?;

        // 1. Build CAD Pipeline
        let cad_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("CAD Shader Module"),
            source: wgpu::ShaderSource::Wgsl(CAD_SHADER_WGSL.into()),
        });

        let cad_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("CAD Bind Group Layout"),
                entries: &[
                    // Camera uniform
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
                    // Material uniform
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

        let cad_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("CAD Pipeline Layout"),
            bind_group_layouts: &[&cad_bind_group_layout],
            push_constant_ranges: &[],
        });

        let cad_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("CAD Render Pipeline"),
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
                    format: wgpu::TextureFormat::Rgba8Unorm,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None, // Two-sided rendering for robust CAD viewing
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

        // 2. Build FEA Heatmap Pipeline
        let fea_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("FEA Heatmap Shader Module"),
            source: wgpu::ShaderSource::Wgsl(FEA_HEATMAP_SHADER_WGSL.into()),
        });

        let fea_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("FEA Bind Group Layout"),
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

        let fea_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("FEA Pipeline Layout"),
            bind_group_layouts: &[&fea_bind_group_layout],
            push_constant_ranges: &[],
        });

        let fea_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("FEA Render Pipeline"),
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
                    format: wgpu::TextureFormat::Rgba8Unorm,
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

        // 3. Build Wireframe / Line Overlay Pipeline
        let wireframe_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Wireframe Shader Module"),
            source: wgpu::ShaderSource::Wgsl(WIREFRAME_SHADER_WGSL.into()),
        });

        let wireframe_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Wireframe Bind Group Layout"),
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
                label: Some("Wireframe Pipeline Layout"),
                bind_group_layouts: &[&wireframe_bind_group_layout],
                push_constant_ranges: &[],
            });

        let wireframe_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Wireframe Render Pipeline"),
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
                    format: wgpu::TextureFormat::Rgba8Unorm,
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
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState {
                    constant: -2,
                    slope_scale: -2.0,
                    clamp: 0.0,
                },
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Ok(Self {
            device,
            queue,
            cad_pipeline,
            cad_bind_group_layout,
            fea_pipeline,
            fea_bind_group_layout,
            wireframe_pipeline,
            wireframe_bind_group_layout,
        })
    }

    /// Renders a triangular solid CAD mesh into PNG encoded bytes.
    pub fn render_mesh_png(
        &self,
        mesh: &TriangleMesh,
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        self.render_mesh_png_with_material(mesh, camera_preset, MaterialUniform::default(), width, height)
    }

    /// Renders a triangular solid CAD mesh with a custom material uniform into PNG encoded bytes.
    pub fn render_mesh_png_with_material(
        &self,
        mesh: &TriangleMesh,
        camera_preset: CameraPreset,
        material: MaterialUniform,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        if width == 0 || height == 0 {
            return Err(ViewerError::InvalidDimensions { width, height });
        }
        if mesh.vertices.is_empty() || mesh.indices.is_empty() {
            return Err(ViewerError::EmptyMesh);
        }

        // Camera setup
        let bbox = mesh.bounding_box();
        let min_f32 = [bbox.min[0] as f32, bbox.min[1] as f32, bbox.min[2] as f32];
        let max_f32 = [bbox.max[0] as f32, bbox.max[1] as f32, bbox.max[2] as f32];
        let mut camera = OrbitCamera::from_preset(
            camera_preset,
            [0.0, 0.0, 0.0],
            5.0,
            width as f32 / height as f32,
        );
        camera.fit_bounding_box(min_f32, max_f32);
        let camera_uniform = camera.build_uniform();

        // GPU Buffers
        let cad_vertices: Vec<CadVertex> = mesh
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

        let vertex_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("CAD Vertex Buffer"),
                contents: bytemuck::cast_slice(&cad_vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });

        let index_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("CAD Index Buffer"),
                contents: bytemuck::cast_slice(&flat_indices),
                usage: wgpu::BufferUsages::INDEX,
            });

        let camera_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("CAD Camera Uniform Buffer"),
                contents: bytemuck::bytes_of(&camera_uniform),
                usage: wgpu::BufferUsages::UNIFORM,
            });

        let material_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("CAD Material Uniform Buffer"),
                contents: bytemuck::bytes_of(&material),
                usage: wgpu::BufferUsages::UNIFORM,
            });

        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("CAD Draw Bind Group"),
            layout: &self.cad_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: camera_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: material_buffer.as_entire_binding(),
                },
            ],
        });

        self.execute_render_pass(
            &self.cad_pipeline,
            &bind_group,
            &vertex_buffer,
            &index_buffer,
            flat_indices.len() as u32,
            None,
            width,
            height,
        )
    }

    /// Renders a triangular solid CAD mesh in a specified debug visualization mode.
    pub fn render_mesh_png_debug(
        &self,
        mesh: &TriangleMesh,
        camera_preset: CameraPreset,
        mode: DebugRenderMode,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        if width == 0 || height == 0 {
            return Err(ViewerError::InvalidDimensions { width, height });
        }
        if mesh.vertices.is_empty() || mesh.indices.is_empty() {
            return Err(ViewerError::EmptyMesh);
        }

        let bbox = mesh.bounding_box();
        let min_f32 = [bbox.min[0] as f32, bbox.min[1] as f32, bbox.min[2] as f32];
        let max_f32 = [bbox.max[0] as f32, bbox.max[1] as f32, bbox.max[2] as f32];
        let mut camera = OrbitCamera::from_preset(
            camera_preset,
            [0.0, 0.0, 0.0],
            5.0,
            width as f32 / height as f32,
        );
        camera.fit_bounding_box(min_f32, max_f32);
        let camera_uniform = camera.build_uniform();

        let cad_vertices: Vec<CadVertex> = mesh
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

        let vertex_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("CAD Debug Vertex Buffer"),
                contents: bytemuck::cast_slice(&cad_vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });

        let index_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("CAD Debug Index Buffer"),
                contents: bytemuck::cast_slice(&flat_indices),
                usage: wgpu::BufferUsages::INDEX,
            });

        let camera_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("CAD Debug Camera Uniform Buffer"),
                contents: bytemuck::bytes_of(&camera_uniform),
                usage: wgpu::BufferUsages::UNIFORM,
            });

        let (material, line_indices_opt, line_color) = match mode {
            DebugRenderMode::None => (
                MaterialUniform {
                    color: [0.72, 0.77, 0.84, 1.0],
                    debug_mode: 0,
                    _pad0: 0,
                    _pad1: 0,
                    _pad2: 0,
                },
                None,
                [0.0; 4],
            ),
            DebugRenderMode::Normals => (
                MaterialUniform {
                    color: [1.0, 1.0, 1.0, 1.0],
                    debug_mode: 2,
                    _pad0: 0,
                    _pad1: 0,
                    _pad2: 0,
                },
                None,
                [0.0; 4],
            ),
            DebugRenderMode::Backfaces => (
                MaterialUniform {
                    color: [0.72, 0.77, 0.84, 1.0],
                    debug_mode: 3,
                    _pad0: 0,
                    _pad1: 0,
                    _pad2: 0,
                },
                None,
                [0.0; 4],
            ),
            DebugRenderMode::FlatShading => (
                MaterialUniform {
                    color: [0.72, 0.77, 0.84, 1.0],
                    debug_mode: 4,
                    _pad0: 0,
                    _pad1: 0,
                    _pad2: 0,
                },
                None,
                [0.0; 4],
            ),
            DebugRenderMode::Wireframe => (
                MaterialUniform {
                    color: [0.65, 0.70, 0.78, 1.0],
                    debug_mode: 0,
                    _pad0: 0,
                    _pad1: 0,
                    _pad2: 0,
                },
                Some(extract_wireframe_indices(&mesh.indices)),
                [0.05, 0.08, 0.12, 1.0],
            ),
            DebugRenderMode::BoundaryEdges => (
                MaterialUniform {
                    color: [0.25, 0.28, 0.32, 1.0],
                    debug_mode: 0,
                    _pad0: 0,
                    _pad1: 0,
                    _pad2: 0,
                },
                Some(extract_boundary_indices(mesh)),
                [1.0, 0.85, 0.0, 1.0],
            ),
        };

        let material_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("CAD Debug Material Uniform Buffer"),
                contents: bytemuck::bytes_of(&material),
                usage: wgpu::BufferUsages::UNIFORM,
            });

        let cad_bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("CAD Debug Draw Bind Group"),
            layout: &self.cad_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: camera_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: material_buffer.as_entire_binding(),
                },
            ],
        });

        let line_overlay_data = if let Some(ref lines) = line_indices_opt {
            if !lines.is_empty() {
                let line_ib = self
                    .device
                    .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("CAD Debug Line Index Buffer"),
                        contents: bytemuck::cast_slice(lines),
                        usage: wgpu::BufferUsages::INDEX,
                    });
                let wire_uniform = WireframeUniform { color: line_color };
                let wire_buffer = self
                    .device
                    .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("CAD Debug Wireframe Uniform Buffer"),
                        contents: bytemuck::bytes_of(&wire_uniform),
                        usage: wgpu::BufferUsages::UNIFORM,
                    });
                let line_bg = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("CAD Debug Line Bind Group"),
                    layout: &self.wireframe_bind_group_layout,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: camera_buffer.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: wire_buffer.as_entire_binding(),
                        },
                    ],
                });
                Some((line_bg, line_ib, lines.len() as u32))
            } else {
                None
            }
        } else {
            None
        };

        let line_param = line_overlay_data.as_ref().map(|(bg, ib, count)| {
            (&self.wireframe_pipeline, bg, ib, *count)
        });

        self.execute_render_pass(
            &self.cad_pipeline,
            &cad_bind_group,
            &vertex_buffer,
            &index_buffer,
            flat_indices.len() as u32,
            line_param,
            width,
            height,
        )
    }

    /// Helper to render CAD mesh in debug mode directly to a Base64 encoded PNG string.
    pub fn render_mesh_base64_debug(
        &self,
        mesh: &TriangleMesh,
        camera_preset: CameraPreset,
        mode: DebugRenderMode,
        width: u32,
        height: u32,
    ) -> Result<String, ViewerError> {
        let png = self.render_mesh_png_debug(mesh, camera_preset, mode, width, height)?;
        self.render_to_base64(&png)
    }

    /// Renders a CAD `Part` with the specified tessellation quality into PNG encoded bytes.
    pub fn render_part_png(
        &self,
        part: &mut Part,
        quality: TessellationQuality,
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        let mesh = part.tessellate_quality(quality)?;
        self.render_mesh_png(mesh, camera_preset, width, height)
    }

    /// Renders a CAD `Part` with explicit adaptive tessellation settings into PNG encoded bytes.
    pub fn render_part_png_with_settings(
        &self,
        part: &mut Part,
        settings: &TessellationSettings,
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        let mesh = part.tessellate(settings)?;
        self.render_mesh_png(mesh, camera_preset, width, height)
    }

    /// Renders a CAD `Part` with the specified tessellation quality into a Base64 encoded PNG string.
    pub fn render_part_base64(
        &self,
        part: &mut Part,
        quality: TessellationQuality,
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<String, ViewerError> {
        let png = self.render_part_png(part, quality, camera_preset, width, height)?;
        self.render_to_base64(&png)
    }

    /// Renders a fast preview snapshot (TessellationQuality::Coarse) of a CAD `Part`.
    pub fn render_part_preview_coarse(
        &self,
        part: &mut Part,
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        self.render_part_png(part, TessellationQuality::Coarse, camera_preset, width, height)
    }

    /// Renders a high-detail snapshot (TessellationQuality::Fine) of a CAD `Part`.
    pub fn render_part_snapshot_fine(
        &self,
        part: &mut Part,
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        self.render_part_png(part, TessellationQuality::Fine, camera_preset, width, height)
    }

    /// Renders a CAD `Part` with tessellation quality without modifying the part's internal cache.
    pub fn render_part_immutable_png(
        &self,
        part: &Part,
        quality: TessellationQuality,
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        let settings = TessellationSettings::from(quality);
        let mesh = if !part.dag.nodes.is_empty() {
            part.dag.evaluate_with_settings(&settings)?
        } else {
            part.mesh().clone()
        };
        self.render_mesh_png(&mesh, camera_preset, width, height)
    }

    /// Renders a CAD `Part` evaluated at `quality` colored by FEA normalized stress scalars into PNG bytes.
    pub fn render_part_fea_heatmap_png(
        &self,
        part: &mut Part,
        quality: TessellationQuality,
        stress_scalars: &[f32],
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        let mesh = part.tessellate_quality(quality)?;
        self.render_fea_heatmap_png(mesh, stress_scalars, camera_preset, width, height)
    }

    /// Renders a triangular mesh colored by normalized FEA stress scalars [0.0, 1.0] into PNG bytes.
    pub fn render_fea_heatmap_png(
        &self,
        mesh: &TriangleMesh,
        stress_scalars: &[f32],
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        if width == 0 || height == 0 {
            return Err(ViewerError::InvalidDimensions { width, height });
        }
        if mesh.vertices.is_empty() || mesh.indices.is_empty() {
            return Err(ViewerError::EmptyMesh);
        }
        if stress_scalars.len() != mesh.vertices.len() {
            return Err(ViewerError::StressScalarMismatch {
                expected: mesh.vertices.len(),
                actual: stress_scalars.len(),
            });
        }

        // Camera setup
        let bbox = mesh.bounding_box();
        let min_f32 = [bbox.min[0] as f32, bbox.min[1] as f32, bbox.min[2] as f32];
        let max_f32 = [bbox.max[0] as f32, bbox.max[1] as f32, bbox.max[2] as f32];
        let mut camera = OrbitCamera::from_preset(
            camera_preset,
            [0.0, 0.0, 0.0],
            5.0,
            width as f32 / height as f32,
        );
        camera.fit_bounding_box(min_f32, max_f32);
        let camera_uniform = camera.build_uniform();

        // GPU Buffers
        let fea_vertices: Vec<FeaVertex> = mesh
            .vertices
            .iter()
            .zip(stress_scalars.iter())
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

        let vertex_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("FEA Vertex Buffer"),
                contents: bytemuck::cast_slice(&fea_vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });

        let index_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("FEA Index Buffer"),
                contents: bytemuck::cast_slice(&flat_indices),
                usage: wgpu::BufferUsages::INDEX,
            });

        let camera_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("FEA Camera Uniform Buffer"),
                contents: bytemuck::bytes_of(&camera_uniform),
                usage: wgpu::BufferUsages::UNIFORM,
            });

        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("FEA Draw Bind Group"),
            layout: &self.fea_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        self.execute_render_pass(
            &self.fea_pipeline,
            &bind_group,
            &vertex_buffer,
            &index_buffer,
            flat_indices.len() as u32,
            None,
            width,
            height,
        )
    }

    /// Internal execution of the render pass and GPU-to-CPU texture readback.
    #[allow(clippy::too_many_arguments)]
    fn execute_render_pass(
        &self,
        pipeline: &wgpu::RenderPipeline,
        bind_group: &wgpu::BindGroup,
        vertex_buffer: &wgpu::Buffer,
        index_buffer: &wgpu::Buffer,
        index_count: u32,
        lines_overlay: Option<(&wgpu::RenderPipeline, &wgpu::BindGroup, &wgpu::Buffer, u32)>,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, ViewerError> {
        let texture_desc = wgpu::TextureDescriptor {
            label: Some("Offscreen Color Texture"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        };
        let color_texture = self.device.create_texture(&texture_desc);
        let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());

        let depth_desc = wgpu::TextureDescriptor {
            label: Some("Offscreen Depth Texture"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        };
        let depth_texture = self.device.create_texture(&depth_desc);
        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        let unpadded_bytes_per_row = width * 4;
        let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT; // 256 bytes
        let padded_bytes_per_row = unpadded_bytes_per_row.div_ceil(align) * align;
        let buffer_size = (padded_bytes_per_row * height) as u64;

        let output_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Offscreen Readback Buffer"),
            size: buffer_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Offscreen Render Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Offscreen Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &color_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        // Sleek dark CAD studio background: [0.15, 0.17, 0.20]
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.15,
                            g: 0.17,
                            b: 0.20,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            render_pass.set_pipeline(pipeline);
            render_pass.set_bind_group(0, bind_group, &[]);
            render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            render_pass.set_index_buffer(index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            render_pass.draw_indexed(0..index_count, 0, 0..1);

            if let Some((line_pipe, line_bg, line_ib, line_count)) = lines_overlay {
                if line_count > 0 {
                    render_pass.set_pipeline(line_pipe);
                    render_pass.set_bind_group(0, line_bg, &[]);
                    render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
                    render_pass.set_index_buffer(line_ib.slice(..), wgpu::IndexFormat::Uint32);
                    render_pass.draw_indexed(0..line_count, 0, 0..1);
                }
            }
        }

        encoder.copy_texture_to_buffer(
            wgpu::ImageCopyTexture {
                texture: &color_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::ImageCopyBuffer {
                buffer: &output_buffer,
                layout: wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: Some(padded_bytes_per_row),
                    rows_per_image: Some(height),
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );

        self.queue.submit(std::iter::once(encoder.finish()));

        // Map buffer to CPU memory
        let buffer_slice = output_buffer.slice(..);
        let (sender, receiver) = channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = sender.send(result);
        });

        self.device.poll(wgpu::Maintain::Wait);

        receiver
            .recv()
            .map_err(|_| ViewerError::Internal("Receiver channel disconnected".to_string()))?
            .map_err(ViewerError::BufferAsyncError)?;

        let padded_data = buffer_slice.get_mapped_range();
        let mut unpadded = Vec::with_capacity((width * height * 4) as usize);

        for row in 0..height {
            let start = (row * padded_bytes_per_row) as usize;
            let end = start + (width * 4) as usize;
            unpadded.extend_from_slice(&padded_data[start..end]);
        }

        drop(padded_data);
        output_buffer.unmap();

        // Encode to PNG bytes
        let img = image::RgbaImage::from_raw(width, height, unpadded).ok_or_else(|| {
            ViewerError::Internal("Failed to construct RgbaImage from raw frame bytes".to_string())
        })?;

        let mut png_bytes = Vec::new();
        let mut cursor = std::io::Cursor::new(&mut png_bytes);
        img.write_to(&mut cursor, image::ImageFormat::Png)?;

        Ok(png_bytes)
    }

    /// Converts raw PNG bytes to Base64 string for direct delivery over MCP protocol.
    pub fn render_to_base64(&self, png_bytes: &[u8]) -> Result<String, ViewerError> {
        Ok(base64::prelude::BASE64_STANDARD.encode(png_bytes))
    }

    /// Helper to render CAD mesh directly to Base64 string.
    pub fn render_mesh_base64(
        &self,
        mesh: &TriangleMesh,
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<String, ViewerError> {
        let png = self.render_mesh_png(mesh, camera_preset, width, height)?;
        self.render_to_base64(&png)
    }

    /// Helper to render FEA heatmap directly to Base64 string.
    pub fn render_fea_heatmap_base64(
        &self,
        mesh: &TriangleMesh,
        stress_scalars: &[f32],
        camera_preset: CameraPreset,
        width: u32,
        height: u32,
    ) -> Result<String, ViewerError> {
        let png = self.render_fea_heatmap_png(mesh, stress_scalars, camera_preset, width, height)?;
        self.render_to_base64(&png)
    }
}
