//! Built-in WGSL shaders and vertex layouts for CAD solid shading and FEA stress heatmaps.

/// WGSL shader source code for CAD solid mesh rendering with Blinn-Phong lighting.
pub const CAD_SHADER_WGSL: &str = r#"
struct CameraUniform {
    view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
};

struct MaterialUniform {
    color: vec4<f32>,
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
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    var n = normalize(in.world_normal);
    let view_dir = normalize(camera.camera_pos.xyz - in.world_position);
    
    // Two-sided lighting
    if (dot(n, view_dir) < 0.0) {
        n = -n;
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

/// Uniform structure representing material properties.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MaterialUniform {
    /// RGBA diffuse color.
    pub color: [f32; 4],
}

impl Default for MaterialUniform {
    fn default() -> Self {
        Self {
            color: [0.72, 0.77, 0.84, 1.0], // Engineering CAD silver-blue
        }
    }
}
