//! Built-in WGSL shaders and vertex layouts for CAD solid shading and FEA stress heatmaps.

/// WGSL shader source code for CAD solid mesh rendering with Blinn-Phong lighting.
pub const CAD_SHADER_WGSL: &str = r#"
struct CameraUniform {
    view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
};

struct MaterialUniform {
    color: vec4<f32>,
    debug_mode: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(0) @binding(1)
var<uniform> material: MaterialUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_position: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.world_position = in.position;
    out.world_normal = in.normal;
    out.clip_position = camera.view_proj * vec4<f32>(in.position, 1.0);
    return out;
}

@fragment
fn fs_main(in: VertexOutput, @builtin(front_facing) is_front: bool) -> @location(0) vec4<f32> {
    var n = normalize(in.world_normal);
    let view_dir = normalize(camera.camera_pos.xyz - in.world_position);
    
    // Debug mode 2: Normals visualization (RGB = Normal * 0.5 + 0.5)
    if (material.debug_mode == 2u) {
        return vec4<f32>(n * 0.5 + 0.5, 1.0);
    }

    // Debug mode 3: Backfaces highlight (bright magenta for geometric back-facing triangles)
    if (material.debug_mode == 3u) {
        if (!is_front) {
            return vec4<f32>(1.0, 0.05, 0.6, 1.0);
        }
    }

    // Debug mode 4: Flat shading (pure per-polygon normal from screen-space derivatives)
    if (material.debug_mode == 4u) {
        let dpdx = dpdx(in.world_position);
        let dpdy = dpdy(in.world_position);
        let geo_n = normalize(cross(dpdx, dpdy));
        if (dot(geo_n, view_dir) < 0.0) {
            n = -geo_n;
        } else {
            n = geo_n;
        }
    } else {
        // Two-sided lighting
        if (dot(n, view_dir) < 0.0) {
            n = -n;
        }
    }

    // Directional Key Light (upper right front)
    let light_dir1 = normalize(vec3<f32>(0.5, 0.8, 0.6));
    let diff1 = max(dot(n, light_dir1), 0.0);
    let half_dir1 = normalize(light_dir1 + view_dir);
    let spec1 = pow(max(dot(n, half_dir1), 0.0), 32.0);

    // Directional Fill Light (softer from lower opposite side)
    let light_dir2 = normalize(vec3<f32>(-0.4, -0.3, -0.5));
    let diff2 = max(dot(n, light_dir2), 0.0) * 0.35;

    let base_color = material.color.rgb;
    let ambient = base_color * 0.25;
    let diffuse = base_color * (diff1 * 0.65 + diff2);
    let specular = vec3<f32>(1.0, 1.0, 1.0) * (spec1 * 0.30);

    let final_rgb = ambient + diffuse + specular;
    return vec4<f32>(final_rgb, material.color.a);
}
"#;

/// WGSL shader source code for FEA mesh rendering with dynamic Rainbow/Jet stress colormap.
pub const FEA_HEATMAP_SHADER_WGSL: &str = r#"
struct CameraUniform {
    view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct FeaVertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) scalar: f32,
};

struct FeaVertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_position: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) scalar: f32,
};

@vertex
fn vs_main(in: FeaVertexInput) -> FeaVertexOutput {
    var out: FeaVertexOutput;
    out.world_position = in.position;
    out.world_normal = in.normal;
    out.scalar = in.scalar;
    out.clip_position = camera.view_proj * vec4<f32>(in.position, 1.0);
    return out;
}

// 5-point colormap gradient: Blue (0.0) -> Cyan (0.25) -> Green (0.50) -> Yellow (0.75) -> Red (1.0)
fn colormap_jet(t: f32) -> vec3<f32> {
    let val = clamp(t, 0.0, 1.0);
    if (val <= 0.25) {
        let f = val / 0.25;
        return mix(vec3<f32>(0.0, 0.0, 1.0), vec3<f32>(0.0, 1.0, 1.0), f);
    } else if (val <= 0.50) {
        let f = (val - 0.25) / 0.25;
        return mix(vec3<f32>(0.0, 1.0, 1.0), vec3<f32>(0.0, 1.0, 0.0), f);
    } else if (val <= 0.75) {
        let f = (val - 0.50) / 0.25;
        return mix(vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(1.0, 1.0, 0.0), f);
    } else {
        let f = (val - 0.75) / 0.25;
        return mix(vec3<f32>(1.0, 1.0, 0.0), vec3<f32>(1.0, 0.0, 0.0), f);
    }
}

@fragment
fn fs_main(in: FeaVertexOutput) -> @location(0) vec4<f32> {
    var n = normalize(in.world_normal);
    let view_dir = normalize(camera.camera_pos.xyz - in.world_position);

    if (dot(n, view_dir) < 0.0) {
        n = -n;
    }

    let light_dir = normalize(vec3<f32>(0.5, 0.8, 0.6));
    let diff = max(dot(n, light_dir), 0.0);

    let heat_color = colormap_jet(in.scalar);
    // Modulate base heat color slightly with shading for 3D depth perception
    let lit_rgb = heat_color * (0.70 + 0.30 * diff);

    return vec4<f32>(lit_rgb, 1.0);
}
"#;

/// Vertex structure for standard CAD solid rendering.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CadVertex {
    /// 3D position [x, y, z].
    pub position: [f32; 3],
    /// Unit surface normal [nx, ny, nz].
    pub normal: [f32; 3],
}

impl CadVertex {
    /// Vertex buffer layout descriptor for WGPU render pipeline creation.
    pub const LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<CadVertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute {
                offset: 0,
                shader_location: 0,
                format: wgpu::VertexFormat::Float32x3,
            },
            wgpu::VertexAttribute {
                offset: 12,
                shader_location: 1,
                format: wgpu::VertexFormat::Float32x3,
            },
        ],
    };
}

/// Vertex structure for FEA stress heatmap rendering.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct FeaVertex {
    /// 3D position [x, y, z].
    pub position: [f32; 3],
    /// Unit surface normal [nx, ny, nz].
    pub normal: [f32; 3],
    /// Normalized stress scalar in range [0.0, 1.0].
    pub scalar: f32,
}

impl FeaVertex {
    /// Vertex buffer layout descriptor for FEA heatmap pipeline creation.
    pub const LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<FeaVertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute {
                offset: 0,
                shader_location: 0,
                format: wgpu::VertexFormat::Float32x3,
            },
            wgpu::VertexAttribute {
                offset: 12,
                shader_location: 1,
                format: wgpu::VertexFormat::Float32x3,
            },
            wgpu::VertexAttribute {
                offset: 24,
                shader_location: 2,
                format: wgpu::VertexFormat::Float32,
            },
        ],
    };
}

/// Uniform structure representing material properties and viewport debug mode.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MaterialUniform {
    /// RGBA diffuse color.
    pub color: [f32; 4],
    /// 0 = standard shaded, 1 = wireframe, 2 = normals, 3 = backfaces, 4 = flat_shading, 5 = boundary_edges
    pub debug_mode: u32,
    pub _pad0: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

impl Default for MaterialUniform {
    fn default() -> Self {
        Self {
            color: [0.72, 0.77, 0.84, 1.0], // Engineering CAD silver-blue
            debug_mode: 0,
            _pad0: 0,
            _pad1: 0,
            _pad2: 0,
        }
    }
}

/// WGSL shader source code for Wireframe CAD rendering with customizable line color.
pub const WIREFRAME_SHADER_WGSL: &str = r#"
struct CameraUniform {
    view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
};

struct WireframeUniform {
    color: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(0) @binding(1)
var<uniform> wireframe: WireframeUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> @builtin(position) vec4<f32> {
    return camera.view_proj * vec4<f32>(in.position, 1.0);
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
    return wireframe.color;
}
"#;

/// WGSL shader source code for 2D Viewport UI overlay, buttons, badges, and dividers.
pub const UI_SHADER_WGSL: &str = r#"
struct UiVertexInput {
    @location(0) position: vec2<f32>,
    @location(1) color: vec4<f32>,
};

struct UiVertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(in: UiVertexInput) -> UiVertexOutput {
    var out: UiVertexOutput;
    out.clip_position = vec4<f32>(in.position, 0.0, 1.0);
    out.color = in.color;
    return out;
}

@fragment
fn fs_main(in: UiVertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}
"#;

/// Uniform structure for wireframe line color.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct WireframeUniform {
    /// RGBA line color (default black [0.05, 0.05, 0.05, 1.0]).
    pub color: [f32; 4],
}

impl Default for WireframeUniform {
    fn default() -> Self {
        Self {
            color: [0.05, 0.05, 0.05, 1.0],
        }
    }
}

/// Vertex structure for 2D UI overlay elements.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct UiVertex {
    /// 2D position in Normalized Device Coordinates (NDC) [-1.0, 1.0].
    pub position: [f32; 2],
    /// RGBA vertex color.
    pub color: [f32; 4],
}

impl UiVertex {
    /// Vertex buffer layout descriptor for 2D UI pipeline creation.
    pub const LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<UiVertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute {
                offset: 0,
                shader_location: 0,
                format: wgpu::VertexFormat::Float32x2,
            },
            wgpu::VertexAttribute {
                offset: 8,
                shader_location: 1,
                format: wgpu::VertexFormat::Float32x4,
            },
        ],
    };
}

/// Returns vector strokes for rendering CAD HUD alphanumeric characters.
pub fn get_char_strokes(c: char) -> &'static [(f32, f32, f32, f32)] {
    match c.to_ascii_uppercase() {
        '-' => &[(0.2, 0.5, 0.8, 0.5)],
        ':' => &[(0.5, 0.3, 0.5, 0.35), (0.5, 0.65, 0.5, 0.7)],
        '[' => &[(0.7, 0.1, 0.3, 0.1), (0.3, 0.1, 0.3, 0.9), (0.3, 0.9, 0.7, 0.9)],
        ']' => &[(0.3, 0.1, 0.7, 0.1), (0.7, 0.1, 0.7, 0.9), (0.7, 0.9, 0.3, 0.9)],
        '|' => &[(0.5, 0.1, 0.5, 0.9)],
        '/' => &[(0.2, 0.1, 0.8, 0.9)],
        '0' => &[(0.2, 0.1, 0.8, 0.1), (0.8, 0.1, 0.8, 0.9), (0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.1), (0.8, 0.9, 0.2, 0.1)],
        '1' => &[(0.3, 0.7, 0.5, 0.9), (0.5, 0.9, 0.5, 0.1), (0.2, 0.1, 0.8, 0.1)],
        '2' => &[(0.2, 0.8, 0.8, 0.8), (0.8, 0.8, 0.8, 0.5), (0.8, 0.5, 0.2, 0.1), (0.2, 0.1, 0.8, 0.1)],
        '3' => &[(0.2, 0.9, 0.8, 0.9), (0.8, 0.9, 0.5, 0.5), (0.5, 0.5, 0.8, 0.5), (0.8, 0.5, 0.8, 0.1), (0.8, 0.1, 0.2, 0.1)],
        '4' => &[(0.8, 0.1, 0.8, 0.9), (0.8, 0.9, 0.2, 0.4), (0.2, 0.4, 0.9, 0.4)],
        '5' => &[(0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.5), (0.2, 0.5, 0.8, 0.5), (0.8, 0.5, 0.8, 0.1), (0.8, 0.1, 0.2, 0.1)],
        '6' => &[(0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.1), (0.2, 0.1, 0.8, 0.1), (0.8, 0.1, 0.8, 0.5), (0.8, 0.5, 0.2, 0.5)],
        '7' => &[(0.2, 0.9, 0.8, 0.9), (0.8, 0.9, 0.3, 0.1)],
        '8' => &[(0.2, 0.1, 0.8, 0.1), (0.8, 0.1, 0.8, 0.9), (0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.1), (0.2, 0.5, 0.8, 0.5)],
        '9' => &[(0.8, 0.1, 0.8, 0.9), (0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.5), (0.2, 0.5, 0.8, 0.5)],
        'A' => &[(0.2, 0.1, 0.5, 0.9), (0.5, 0.9, 0.8, 0.1), (0.3, 0.4, 0.7, 0.4)],
        'B' => &[(0.2, 0.1, 0.2, 0.9), (0.2, 0.9, 0.7, 0.9), (0.7, 0.9, 0.7, 0.5), (0.7, 0.5, 0.2, 0.5), (0.2, 0.5, 0.8, 0.5), (0.8, 0.5, 0.8, 0.1), (0.8, 0.1, 0.2, 0.1)],
        'C' => &[(0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.1), (0.2, 0.1, 0.8, 0.1)],
        'D' => &[(0.2, 0.1, 0.2, 0.9), (0.2, 0.9, 0.6, 0.9), (0.6, 0.9, 0.8, 0.5), (0.8, 0.5, 0.6, 0.1), (0.6, 0.1, 0.2, 0.1)],
        'E' => &[(0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.1), (0.2, 0.1, 0.8, 0.1), (0.2, 0.5, 0.6, 0.5)],
        'F' => &[(0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.1), (0.2, 0.5, 0.6, 0.5)],
        'G' => &[(0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.1), (0.2, 0.1, 0.8, 0.1), (0.8, 0.1, 0.8, 0.5), (0.8, 0.5, 0.5, 0.5)],
        'H' => &[(0.2, 0.1, 0.2, 0.9), (0.8, 0.1, 0.8, 0.9), (0.2, 0.5, 0.8, 0.5)],
        'I' => &[(0.3, 0.9, 0.7, 0.9), (0.5, 0.9, 0.5, 0.1), (0.3, 0.1, 0.7, 0.1)],
        'J' => &[(0.7, 0.9, 0.7, 0.3), (0.7, 0.3, 0.5, 0.1), (0.5, 0.1, 0.3, 0.1), (0.3, 0.1, 0.3, 0.3)],
        'K' => &[(0.2, 0.1, 0.2, 0.9), (0.8, 0.9, 0.2, 0.5), (0.2, 0.5, 0.8, 0.1)],
        'L' => &[(0.2, 0.9, 0.2, 0.1), (0.2, 0.1, 0.8, 0.1)],
        'M' => &[(0.2, 0.1, 0.2, 0.9), (0.2, 0.9, 0.5, 0.5), (0.5, 0.5, 0.8, 0.9), (0.8, 0.9, 0.8, 0.1)],
        'N' => &[(0.2, 0.1, 0.2, 0.9), (0.2, 0.9, 0.8, 0.1), (0.8, 0.1, 0.8, 0.9)],
        'O' => &[(0.2, 0.1, 0.8, 0.1), (0.8, 0.1, 0.8, 0.9), (0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.1)],
        'P' => &[(0.2, 0.1, 0.2, 0.9), (0.2, 0.9, 0.8, 0.9), (0.8, 0.9, 0.8, 0.5), (0.8, 0.5, 0.2, 0.5)],
        'Q' => &[(0.2, 0.1, 0.8, 0.1), (0.8, 0.1, 0.8, 0.9), (0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.1), (0.5, 0.3, 0.8, 0.05)],
        'R' => &[(0.2, 0.1, 0.2, 0.9), (0.2, 0.9, 0.8, 0.9), (0.8, 0.9, 0.8, 0.5), (0.8, 0.5, 0.2, 0.5), (0.4, 0.5, 0.8, 0.1)],
        'S' => &[(0.8, 0.9, 0.2, 0.9), (0.2, 0.9, 0.2, 0.5), (0.2, 0.5, 0.8, 0.5), (0.8, 0.5, 0.8, 0.1), (0.8, 0.1, 0.2, 0.1)],
        'T' => &[(0.1, 0.9, 0.9, 0.9), (0.5, 0.9, 0.5, 0.1)],
        'U' => &[(0.2, 0.9, 0.2, 0.2), (0.2, 0.2, 0.4, 0.1), (0.4, 0.1, 0.6, 0.1), (0.6, 0.1, 0.8, 0.2), (0.8, 0.2, 0.8, 0.9)],
        'V' => &[(0.2, 0.9, 0.5, 0.1), (0.5, 0.1, 0.8, 0.9)],
        'W' => &[(0.1, 0.9, 0.3, 0.1), (0.3, 0.1, 0.5, 0.6), (0.5, 0.6, 0.7, 0.1), (0.7, 0.1, 0.9, 0.9)],
        'X' => &[(0.2, 0.1, 0.8, 0.9), (0.2, 0.9, 0.8, 0.1)],
        'Y' => &[(0.2, 0.9, 0.5, 0.5), (0.8, 0.9, 0.5, 0.5), (0.5, 0.5, 0.5, 0.1)],
        'Z' => &[(0.2, 0.9, 0.8, 0.9), (0.8, 0.9, 0.2, 0.1), (0.2, 0.1, 0.8, 0.1)],
        _ => &[],
    }
}
