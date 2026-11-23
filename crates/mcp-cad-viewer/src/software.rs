//! Pure-Rust software CPU rasterizer fallback for headless offscreen rendering (`ADR-003`, `P5-T9`).
//!
//! Matches the WGPU offscreen shaders (`CAD_SHADER_WGSL`, `FEA_HEATMAP_SHADER_WGSL`,
//! `WIREFRAME_SHADER_WGSL`) with perspective-correct barycentric interpolation, Z-buffering,
//! Blinn-Phong two-sided lighting, debug inspection modes, and 5-stop FEA Jet colormaps.

use crate::camera::CameraUniform;
use crate::error::ViewerError;
use crate::shader::{CadVertex, FeaVertex, MaterialUniform};

/// Clear color matching the WGPU dark CAD studio background `[0.15, 0.17, 0.20, 1.0]`.
const CLEAR_RGBA: [u8; 4] = [38, 43, 51, 255];

/// Maximum render dimension allowed by the software fallback to prevent accidental OOM.
pub(crate) const MAX_SOFTWARE_DIMENSION: u32 = 8192;

#[derive(Clone, Copy)]
struct ClipVertex {
    clip: [f32; 4],
    world_pos: [f32; 3],
    world_normal: [f32; 3],
    scalar: f32,
}

#[derive(Clone, Copy)]
struct ScreenVertex {
    sx: f32,
    sy: f32,
    sz: f32,
    inv_w: f32,
    world_pos: [f32; 3],
    world_normal: [f32; 3],
    scalar: f32,
}

struct Framebuffer {
    width: u32,
    height: u32,
    color: Vec<u8>,
    depth: Vec<f32>,
}

impl Framebuffer {
    fn new(width: u32, height: u32) -> Result<Self, ViewerError> {
        if width == 0 || height == 0 || width > MAX_SOFTWARE_DIMENSION || height > MAX_SOFTWARE_DIMENSION {
            return Err(ViewerError::InvalidDimensions { width, height });
        }
        let pixels = (width as usize)
            .checked_mul(height as usize)
            .ok_or(ViewerError::InvalidDimensions { width, height })?;
        let mut color = vec![0u8; pixels * 4];
        for px in color.chunks_exact_mut(4) {
            px.copy_from_slice(&CLEAR_RGBA);
        }
        let depth = vec![1.0_f32; pixels];
        Ok(Self {
            width,
            height,
            color,
            depth,
        })
    }

    fn encode_png(self) -> Result<Vec<u8>, ViewerError> {
        let img = image::RgbaImage::from_raw(self.width, self.height, self.color).ok_or_else(|| {
            ViewerError::Internal(
                "Failed to construct RgbaImage from software framebuffer".to_string(),
            )
        })?;
        let mut png_bytes = Vec::new();
        let mut cursor = std::io::Cursor::new(&mut png_bytes);
        img.write_to(&mut cursor, image::ImageFormat::Png)?;
        Ok(png_bytes)
    }
}

#[inline]
fn mul_mat4_point(m: &[[f32; 4]; 4], p: [f32; 3]) -> [f32; 4] {
    [
        m[0][0] * p[0] + m[1][0] * p[1] + m[2][0] * p[2] + m[3][0],
        m[0][1] * p[0] + m[1][1] * p[1] + m[2][1] * p[2] + m[3][1],
        m[0][2] * p[0] + m[1][2] * p[1] + m[2][2] * p[2] + m[3][2],
        m[0][3] * p[0] + m[1][3] * p[1] + m[2][3] * p[2] + m[3][3],
    ]
}

#[inline]
fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn cross3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
fn sub3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
fn add3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
fn normalize3(v: [f32; 3]) -> [f32; 3] {
    let len_sq = dot3(v, v);
    if len_sq > 1e-20 && len_sq.is_finite() {
        let inv = len_sq.sqrt().recip();
        [v[0] * inv, v[1] * inv, v[2] * inv]
    } else {
        [0.0, 0.0, 1.0]
    }
}

#[inline]
fn lerp_clip_vertex(a: ClipVertex, b: ClipVertex, t: f32) -> ClipVertex {
    ClipVertex {
        clip: [
            a.clip[0] + (b.clip[0] - a.clip[0]) * t,
            a.clip[1] + (b.clip[1] - a.clip[1]) * t,
            a.clip[2] + (b.clip[2] - a.clip[2]) * t,
            a.clip[3] + (b.clip[3] - a.clip[3]) * t,
        ],
        world_pos: [
            a.world_pos[0] + (b.world_pos[0] - a.world_pos[0]) * t,
            a.world_pos[1] + (b.world_pos[1] - a.world_pos[1]) * t,
            a.world_pos[2] + (b.world_pos[2] - a.world_pos[2]) * t,
        ],
        world_normal: [
            a.world_normal[0] + (b.world_normal[0] - a.world_normal[0]) * t,
            a.world_normal[1] + (b.world_normal[1] - a.world_normal[1]) * t,
            a.world_normal[2] + (b.world_normal[2] - a.world_normal[2]) * t,
        ],
        scalar: a.scalar + (b.scalar - a.scalar) * t,
    }
}

/// Clips a convex polygon against a single linear half-space `dist(v) >= 0`.
fn clip_polygon_plane(
    input: &[ClipVertex],
    out: &mut Vec<ClipVertex>,
    dist: impl Fn(&ClipVertex) -> f32,
) {
    out.clear();
    if input.is_empty() {
        return;
    }
    let mut prev = input[input.len() - 1];
    let mut prev_d = dist(&prev);

    for &curr in input {
        let curr_d = dist(&curr);
        let prev_in = prev_d >= 0.0;
        let curr_in = curr_d >= 0.0;

        if curr_in {
            if !prev_in {
                let denom = prev_d - curr_d;
                let t = if denom.abs() > 1e-12 {
                    (prev_d / denom).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                out.push(lerp_clip_vertex(prev, curr, t));
            }
            out.push(curr);
        } else if prev_in {
            let denom = prev_d - curr_d;
            let t = if denom.abs() > 1e-12 {
                (prev_d / denom).clamp(0.0, 1.0)
            } else {
                0.5
            };
            out.push(lerp_clip_vertex(prev, curr, t));
        }

        prev = curr;
        prev_d = curr_d;
    }
}

#[inline]
fn to_screen(v: ClipVertex, wf: f32, hf: f32) -> ScreenVertex {
    let inv_w = v.clip[3].recip();
    let ndc_x = v.clip[0] * inv_w;
    let ndc_y = v.clip[1] * inv_w;
    let ndc_z = v.clip[2] * inv_w;
    ScreenVertex {
        sx: (ndc_x * 0.5 + 0.5) * wf,
        sy: (0.5 - ndc_y * 0.5) * hf,
        sz: ndc_z,
        inv_w,
        world_pos: v.world_pos,
        world_normal: v.world_normal,
        scalar: v.scalar,
    }
}

#[inline]
fn edge_fn(ax: f32, ay: f32, bx: f32, by: f32, px: f32, py: f32) -> f32 {
    (bx - ax) * (py - ay) - (by - ay) * (px - ax)
}

#[inline]
fn to_u8(v: f32) -> u8 {
    (v.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// Evaluates the 5-stop Jet colormap matching `FEA_HEATMAP_SHADER_WGSL::colormap_jet`.
#[inline]
fn colormap_jet_wgpu(t: f32) -> [f32; 3] {
    let val = if t.is_finite() { t.clamp(0.0, 1.0) } else { 0.0 };
    if val <= 0.25 {
        let f = val / 0.25;
        [0.0, f, 1.0]
    } else if val <= 0.50 {
        let f = (val - 0.25) / 0.25;
        [0.0, 1.0, 1.0 - f]
    } else if val <= 0.75 {
        let f = (val - 0.50) / 0.25;
        [f, 1.0, 0.0]
    } else {
        let f = (val - 0.75) / 0.25;
        [1.0, 1.0 - f, 0.0]
    }
}

struct CadShadeContext {
    camera_pos: [f32; 3],
    material: MaterialUniform,
    light_dir1: [f32; 3],
    light_dir2: [f32; 3],
}

#[inline]
fn shade_cad_fragment(
    world_pos: [f32; 3],
    world_normal: [f32; 3],
    geo_normal: [f32; 3],
    is_front: bool,
    ctx: &CadShadeContext,
) -> [u8; 4] {
    let mut n = normalize3(world_normal);
    let view_dir = normalize3(sub3(ctx.camera_pos, world_pos));

    // Debug mode 2: Normals visualization
    if ctx.material.debug_mode == 2 {
        return [
            to_u8(n[0] * 0.5 + 0.5),
            to_u8(n[1] * 0.5 + 0.5),
            to_u8(n[2] * 0.5 + 0.5),
            255,
        ];
    }

    // Debug mode 3: Backfaces highlight (bright magenta for geometric back-facing triangles)
    if ctx.material.debug_mode == 3 && !is_front {
        return [255, to_u8(0.05), to_u8(0.6), 255];
    }

    // Debug mode 4: Flat shading (pure per-polygon geometric normal)
    if ctx.material.debug_mode == 4 {
        n = if dot3(geo_normal, view_dir) < 0.0 {
            [-geo_normal[0], -geo_normal[1], -geo_normal[2]]
        } else {
            geo_normal
        };
    } else if dot3(n, view_dir) < 0.0 {
        // Two-sided lighting
        n = [-n[0], -n[1], -n[2]];
    }

    let diff1 = dot3(n, ctx.light_dir1).max(0.0);
    let half_dir1 = normalize3(add3(ctx.light_dir1, view_dir));
    let spec1 = dot3(n, half_dir1).max(0.0).powi(32);

    let diff2 = dot3(n, ctx.light_dir2).max(0.0) * 0.35;

    let base = ctx.material.color;
    let diffuse_gain = 0.25 + diff1 * 0.65 + diff2;
    let spec_add = spec1 * 0.30;

    [
        to_u8(base[0] * diffuse_gain + spec_add),
        to_u8(base[1] * diffuse_gain + spec_add),
        to_u8(base[2] * diffuse_gain + spec_add),
        to_u8(base[3]),
    ]
}

#[inline]
fn shade_fea_fragment(
    world_pos: [f32; 3],
    world_normal: [f32; 3],
    scalar: f32,
    camera_pos: [f32; 3],
    light_dir: [f32; 3],
) -> [u8; 4] {
    let mut n = normalize3(world_normal);
    let view_dir = normalize3(sub3(camera_pos, world_pos));
    if dot3(n, view_dir) < 0.0 {
        n = [-n[0], -n[1], -n[2]];
    }
    let diff = dot3(n, light_dir).max(0.0);
    let heat = colormap_jet_wgpu(scalar);
    let gain = 0.70 + 0.30 * diff;
    [
        to_u8(heat[0] * gain),
        to_u8(heat[1] * gain),
        to_u8(heat[2] * gain),
        255,
    ]
}

fn rasterize_triangle<F>(
    fb: &mut Framebuffer,
    v0: ScreenVertex,
    v1: ScreenVertex,
    v2: ScreenVertex,
    mut shade: F,
) where
    F: FnMut([f32; 3], [f32; 3], f32, bool) -> [u8; 4],
{
    let raw_area = edge_fn(v0.sx, v0.sy, v1.sx, v1.sy, v2.sx, v2.sy);
    if !raw_area.is_finite() || raw_area.abs() < 1e-6 {
        return;
    }

    // WGPU uses FrontFace::Ccw in +Y-up NDC, which maps to negative signed area in +Y-down screen space.
    let is_front = raw_area < 0.0;
    let (a, b, c, area) = if raw_area < 0.0 {
        (v0, v2, v1, -raw_area)
    } else {
        (v0, v1, v2, raw_area)
    };
    let inv_area = area.recip();

    let min_xf = a.sx.min(b.sx).min(c.sx);
    let max_xf = a.sx.max(b.sx).max(c.sx);
    let min_yf = a.sy.min(b.sy).min(c.sy);
    let max_yf = a.sy.max(b.sy).max(c.sy);

    let wf = fb.width as f32;
    let hf = fb.height as f32;
    if max_xf < 0.0 || max_yf < 0.0 || min_xf >= wf || min_yf >= hf {
        return;
    }

    let min_x = min_xf.floor().max(0.0) as u32;
    let max_x = (max_xf.ceil().min(wf - 1.0).max(0.0)) as u32;
    let min_y = min_yf.floor().max(0.0) as u32;
    let max_y = (max_yf.ceil().min(hf - 1.0).max(0.0)) as u32;

    let width = fb.width as usize;

    for py in min_y..=max_y {
        let cy = py as f32 + 0.5;
        let row_offset = py as usize * width;
        for px in min_x..=max_x {
            let cx = px as f32 + 0.5;

            let wa = edge_fn(b.sx, b.sy, c.sx, c.sy, cx, cy);
            if wa < 0.0 {
                continue;
            }
            let wb = edge_fn(c.sx, c.sy, a.sx, a.sy, cx, cy);
            if wb < 0.0 {
                continue;
            }
            let wc = edge_fn(a.sx, a.sy, b.sx, b.sy, cx, cy);
            if wc < 0.0 {
                continue;
            }

            let ba = wa * inv_area;
            let bb = wb * inv_area;
            let bc = wc * inv_area;

            let depth = ba * a.sz + bb * b.sz + bc * c.sz;
            if !(0.0..=1.0).contains(&depth) {
                continue;
            }

            let idx = row_offset + px as usize;
            if depth >= fb.depth[idx] {
                continue;
            }

            let pa = ba * a.inv_w;
            let pb = bb * b.inv_w;
            let pc = bc * c.inv_w;
            let p_sum = pa + pb + pc;
            if p_sum <= 1e-20 || !p_sum.is_finite() {
                continue;
            }
            let inv_p = p_sum.recip();

            let world_pos = [
                (pa * a.world_pos[0] + pb * b.world_pos[0] + pc * c.world_pos[0]) * inv_p,
                (pa * a.world_pos[1] + pb * b.world_pos[1] + pc * c.world_pos[1]) * inv_p,
                (pa * a.world_pos[2] + pb * b.world_pos[2] + pc * c.world_pos[2]) * inv_p,
            ];
            let world_normal = [
                (pa * a.world_normal[0] + pb * b.world_normal[0] + pc * c.world_normal[0]) * inv_p,
                (pa * a.world_normal[1] + pb * b.world_normal[1] + pc * c.world_normal[1]) * inv_p,
                (pa * a.world_normal[2] + pb * b.world_normal[2] + pc * c.world_normal[2]) * inv_p,
            ];
            let scalar = (pa * a.scalar + pb * b.scalar + pc * c.scalar) * inv_p;

            fb.depth[idx] = depth;
            let rgba = shade(world_pos, world_normal, scalar, is_front);
            let c_off = idx * 4;
            fb.color[c_off..c_off + 4].copy_from_slice(&rgba);
        }
    }
}

/// Clips a 2D line segment to `[0, wf] x [0, hf]` using Liang-Barsky, returning clipped parameter range `(t0, t1)`.
fn clip_line_2d(x0: f32, y0: f32, x1: f32, y1: f32, wf: f32, hf: f32) -> Option<(f32, f32)> {
    let dx = x1 - x0;
    let dy = y1 - y0;
    let mut t0 = 0.0_f32;
    let mut t1 = 1.0_f32;

    let checks = [
        (-dx, x0),
        (dx, wf - x0),
        (-dy, y0),
        (dy, hf - y0),
    ];

    for (p, q) in checks {
        if p.abs() < 1e-12 {
            if q < 0.0 {
                return None;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                if r > t1 {
                    return None;
                }
                if r > t0 {
                    t0 = r;
                }
            } else {
                if r < t0 {
                    return None;
                }
                if r < t1 {
                    t1 = r;
                }
            }
        }
    }

    if t0 <= t1 {
        Some((t0, t1))
    } else {
        None
    }
}

/// Rasterizes a line overlay with depth testing (depth bias matching WGPU wireframe overlay) and alpha blending.
fn rasterize_lines(
    fb: &mut Framebuffer,
    vertices: &[CadVertex],
    line_indices: &[u32],
    camera: &CameraUniform,
    line_color: [f32; 4],
) {
    let wf = fb.width as f32;
    let hf = fb.height as f32;
    let width = fb.width as i32;
    let height = fb.height as i32;
    let src_a = line_color[3].clamp(0.0, 1.0);
    let inv_a = 1.0 - src_a;
    let src_r = line_color[0].clamp(0.0, 1.0);
    let src_g = line_color[1].clamp(0.0, 1.0);
    let src_b = line_color[2].clamp(0.0, 1.0);

    for seg in line_indices.chunks_exact(2) {
        let (Some(va), Some(vb)) = (vertices.get(seg[0] as usize), vertices.get(seg[1] as usize)) else {
            continue;
        };

        let mut ca = mul_mat4_point(&camera.view_proj, va.position);
        let mut cb = mul_mat4_point(&camera.view_proj, vb.position);

        // Clip against near plane (z >= 0) and w >= 1e-5
        for plane in 0..2 {
            let (da, db) = if plane == 0 {
                (ca[2], cb[2])
            } else {
                (ca[3] - 1e-5, cb[3] - 1e-5)
            };
            if da < 0.0 && db < 0.0 {
                ca[3] = -1.0;
                break;
            }
            if da < 0.0 || db < 0.0 {
                let denom = da - db;
                let t = if denom.abs() > 1e-12 {
                    (da / denom).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                let mid = [
                    ca[0] + (cb[0] - ca[0]) * t,
                    ca[1] + (cb[1] - ca[1]) * t,
                    ca[2] + (cb[2] - ca[2]) * t,
                    ca[3] + (cb[3] - ca[3]) * t,
                ];
                if da < 0.0 {
                    ca = mid;
                } else {
                    cb = mid;
                }
            }
        }

        if ca[3] <= 0.0 || cb[3] <= 0.0 {
            continue;
        }

        let inv_wa = ca[3].recip();
        let inv_wb = cb[3].recip();
        let x0 = (ca[0] * inv_wa * 0.5 + 0.5) * wf;
        let y0 = (0.5 - ca[1] * inv_wa * 0.5) * hf;
        let z0 = ca[2] * inv_wa;

        let x1 = (cb[0] * inv_wb * 0.5 + 0.5) * wf;
        let y1 = (0.5 - cb[1] * inv_wb * 0.5) * hf;
        let z1 = cb[2] * inv_wb;

        let Some((t_start, t_end)) = clip_line_2d(x0, y0, x1, y1, wf, hf) else {
            continue;
        };

        let cx0 = x0 + (x1 - x0) * t_start;
        let cy0 = y0 + (y1 - y0) * t_start;
        let cz0 = z0 + (z1 - z0) * t_start;

        let cx1 = x0 + (x1 - x0) * t_end;
        let cy1 = y0 + (y1 - y0) * t_end;
        let cz1 = z0 + (z1 - z0) * t_end;

        let steps = (cx1 - cx0).abs().max((cy1 - cy0).abs()).ceil() as usize;
        for s in 0..=steps {
            let t = if steps == 0 {
                0.0
            } else {
                s as f32 / steps as f32
            };
            let px = (cx0 + (cx1 - cx0) * t).floor() as i32;
            let py = (cy0 + (cy1 - cy0) * t).floor() as i32;
            if px < 0 || px >= width || py < 0 || py >= height {
                continue;
            }
            let z = cz0 + (cz1 - cz0) * t;
            if !(0.0..=1.0).contains(&z) {
                continue;
            }
            let idx = (py as usize) * (fb.width as usize) + (px as usize);
            // Negative depth bias keeps wireframe edges cleanly visible over coplanar triangles.
            if (z - 5e-4) <= fb.depth[idx] {
                let off = idx * 4;
                let dst_r = fb.color[off] as f32 / 255.0;
                let dst_g = fb.color[off + 1] as f32 / 255.0;
                let dst_b = fb.color[off + 2] as f32 / 255.0;
                fb.color[off] = to_u8(src_r * src_a + dst_r * inv_a);
                fb.color[off + 1] = to_u8(src_g * src_a + dst_g * inv_a);
                fb.color[off + 2] = to_u8(src_b * src_a + dst_b * inv_a);
                fb.color[off + 3] = 255;
            }
        }
    }
}

/// Renders a CAD mesh (and optional wireframe/boundary overlay) on the CPU into PNG bytes.
pub(crate) fn render_cad_mesh_png_cpu(
    vertices: &[CadVertex],
    indices: &[[u32; 3]],
    camera: &CameraUniform,
    material: MaterialUniform,
    line_overlay: Option<(&[u32], [f32; 4])>,
    width: u32,
    height: u32,
) -> Result<Vec<u8>, ViewerError> {
    let mut fb = Framebuffer::new(width, height)?;
    let wf = width as f32;
    let hf = height as f32;
    let shade_ctx = CadShadeContext {
        camera_pos: [camera.camera_pos[0], camera.camera_pos[1], camera.camera_pos[2]],
        material,
        light_dir1: normalize3([0.5, 0.8, 0.6]),
        light_dir2: normalize3([-0.4, -0.3, -0.5]),
    };

    let clip_verts: Vec<ClipVertex> = vertices
        .iter()
        .map(|v| ClipVertex {
            clip: mul_mat4_point(&camera.view_proj, v.position),
            world_pos: v.position,
            world_normal: v.normal,
            scalar: 0.0,
        })
        .collect();

    let mut poly_a = Vec::with_capacity(6);
    let mut poly_b = Vec::with_capacity(6);

    for tri in indices {
        let (Some(&v0), Some(&v1), Some(&v2)) = (
            clip_verts.get(tri[0] as usize),
            clip_verts.get(tri[1] as usize),
            clip_verts.get(tri[2] as usize),
        ) else {
            continue;
        };

        let geo_normal = normalize3(cross3(
            sub3(v1.world_pos, v0.world_pos),
            sub3(v2.world_pos, v0.world_pos),
        ));

        // Fast path: all three vertices cleanly in front of the near plane
        if v0.clip[2] >= 0.0
            && v1.clip[2] >= 0.0
            && v2.clip[2] >= 0.0
            && v0.clip[3] >= 1e-5
            && v1.clip[3] >= 1e-5
            && v2.clip[3] >= 1e-5
        {
            let s0 = to_screen(v0, wf, hf);
            let s1 = to_screen(v1, wf, hf);
            let s2 = to_screen(v2, wf, hf);
            rasterize_triangle(&mut fb, s0, s1, s2, |wp, wn, _sc, is_front| {
                shade_cad_fragment(wp, wn, geo_normal, is_front, &shade_ctx)
            });
            continue;
        }

        // Clip against z >= 0 and w >= 1e-5
        let input = [v0, v1, v2];
        clip_polygon_plane(&input, &mut poly_a, |v| v.clip[2]);
        if poly_a.len() < 3 {
            continue;
        }
        clip_polygon_plane(&poly_a, &mut poly_b, |v| v.clip[3] - 1e-5);
        if poly_b.len() < 3 {
            continue;
        }

        let s0 = to_screen(poly_b[0], wf, hf);
        for i in 1..(poly_b.len() - 1) {
            let s1 = to_screen(poly_b[i], wf, hf);
            let s2 = to_screen(poly_b[i + 1], wf, hf);
            rasterize_triangle(&mut fb, s0, s1, s2, |wp, wn, _sc, is_front| {
                shade_cad_fragment(wp, wn, geo_normal, is_front, &shade_ctx)
            });
        }
    }

    if let Some((lines, line_color)) = line_overlay {
        if !lines.is_empty() {
            rasterize_lines(&mut fb, vertices, lines, camera, line_color);
        }
    }

    fb.encode_png()
}

/// Renders an FEA scalar heatmap mesh on the CPU into PNG bytes.
pub(crate) fn render_fea_heatmap_png_cpu(
    vertices: &[FeaVertex],
    indices: &[[u32; 3]],
    camera: &CameraUniform,
    width: u32,
    height: u32,
) -> Result<Vec<u8>, ViewerError> {
    let mut fb = Framebuffer::new(width, height)?;
    let wf = width as f32;
    let hf = height as f32;
    let camera_pos = [camera.camera_pos[0], camera.camera_pos[1], camera.camera_pos[2]];
    let light_dir = normalize3([0.5, 0.8, 0.6]);

    let clip_verts: Vec<ClipVertex> = vertices
        .iter()
        .map(|v| ClipVertex {
            clip: mul_mat4_point(&camera.view_proj, v.position),
            world_pos: v.position,
            world_normal: v.normal,
            scalar: v.scalar,
        })
        .collect();

    let mut poly_a = Vec::with_capacity(6);
    let mut poly_b = Vec::with_capacity(6);

    for tri in indices {
        let (Some(&v0), Some(&v1), Some(&v2)) = (
            clip_verts.get(tri[0] as usize),
            clip_verts.get(tri[1] as usize),
            clip_verts.get(tri[2] as usize),
        ) else {
            continue;
        };

        if v0.clip[2] >= 0.0
            && v1.clip[2] >= 0.0
            && v2.clip[2] >= 0.0
            && v0.clip[3] >= 1e-5
            && v1.clip[3] >= 1e-5
            && v2.clip[3] >= 1e-5
        {
            let s0 = to_screen(v0, wf, hf);
            let s1 = to_screen(v1, wf, hf);
            let s2 = to_screen(v2, wf, hf);
            rasterize_triangle(&mut fb, s0, s1, s2, |wp, wn, sc, _is_front| {
                shade_fea_fragment(wp, wn, sc, camera_pos, light_dir)
            });
            continue;
        }

        let input = [v0, v1, v2];
        clip_polygon_plane(&input, &mut poly_a, |v| v.clip[2]);
        if poly_a.len() < 3 {
            continue;
        }
        clip_polygon_plane(&poly_a, &mut poly_b, |v| v.clip[3] - 1e-5);
        if poly_b.len() < 3 {
            continue;
        }

        let s0 = to_screen(poly_b[0], wf, hf);
        for i in 1..(poly_b.len() - 1) {
            let s1 = to_screen(poly_b[i], wf, hf);
            let s2 = to_screen(poly_b[i + 1], wf, hf);
            rasterize_triangle(&mut fb, s0, s1, s2, |wp, wn, sc, _is_front| {
                shade_fea_fragment(wp, wn, sc, camera_pos, light_dir)
            });
        }
    }

    fb.encode_png()
}
