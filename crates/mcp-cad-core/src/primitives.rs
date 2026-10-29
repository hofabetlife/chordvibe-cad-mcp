//! 3D solid primitive generators: Box, Cylinder, Sphere, and CutHole.

use std::f64::consts::PI;
use nalgebra::{Rotation3, Vector3};
use crate::csg::csg_difference;
use crate::topology::{TriangleMesh, Vertex};

/// Generates a solid 3D box (cuboid) centered at the origin.
///
/// Dimensions: `width` along X, `height` along Y, `depth` along Z.
/// Coordinates span: `[-width/2, width/2] x [-height/2, height/2] x [-depth/2, depth/2]`.
pub fn create_box(width: f64, height: f64, depth: f64) -> TriangleMesh {
    let w2 = width * 0.5;
    let h2 = height * 0.5;
    let d2 = depth * 0.5;

    let mut vertices = Vec::with_capacity(24);
    let mut indices = Vec::with_capacity(12);

    // Helper macro to append a rectangular face (4 vertices, 2 triangles)
    let mut add_face = |p0: [f64; 3], p1: [f64; 3], p2: [f64; 3], p3: [f64; 3], normal: [f64; 3]| {
        let base = vertices.len() as u32;
        vertices.push(Vertex::new(p0, normal));
        vertices.push(Vertex::new(p1, normal));
        vertices.push(Vertex::new(p2, normal));
        vertices.push(Vertex::new(p3, normal));

        // CCW when viewed from outside
        indices.push([base, base + 1, base + 2]);
        indices.push([base, base + 2, base + 3]);
    };

    // Front face (+Z)
    add_face(
        [-w2, -h2,  d2],
        [ w2, -h2,  d2],
        [ w2,  h2,  d2],
        [-w2,  h2,  d2],
        [0.0, 0.0, 1.0],
    );

    // Back face (-Z)
    add_face(
        [ w2, -h2, -d2],
        [-w2, -h2, -d2],
        [-w2,  h2, -d2],
        [ w2,  h2, -d2],
        [0.0, 0.0, -1.0],
    );

    // Right face (+X)
    add_face(
        [ w2, -h2,  d2],
        [ w2, -h2, -d2],
        [ w2,  h2, -d2],
        [ w2,  h2,  d2],
        [1.0, 0.0, 0.0],
    );

    // Left face (-X)
    add_face(
        [-w2, -h2, -d2],
        [-w2, -h2,  d2],
        [-w2,  h2,  d2],
        [-w2,  h2, -d2],
        [-1.0, 0.0, 0.0],
    );

    // Top face (+Y)
    add_face(
        [-w2,  h2,  d2],
        [ w2,  h2,  d2],
        [ w2,  h2, -d2],
        [-w2,  h2, -d2],
        [0.0, 1.0, 0.0],
    );

    // Bottom face (-Y)
    add_face(
        [-w2, -h2, -d2],
        [ w2, -h2, -d2],
        [ w2, -h2,  d2],
        [-w2, -h2,  d2],
        [0.0, -1.0, 0.0],
    );

    TriangleMesh::new(vertices, indices)
}

/// Generates a solid 3D cylinder centered at the origin along the Z axis.
///
/// Coordinates span: `radius` in XY plane, `z in [-height/2, height/2]`.
pub fn create_cylinder(radius: f64, height: f64, segments: usize) -> TriangleMesh {
    let segs = segments.max(8);
    let h2 = height * 0.5;

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // 1. Top cap (z = +h2, normal = [0, 0, 1])
    let top_center_idx = vertices.len() as u32;
    vertices.push(Vertex::new([0.0, 0.0, h2], [0.0, 0.0, 1.0]));

    let top_rim_start = vertices.len() as u32;
    for i in 0..segs {
        let theta = (i as f64) * 2.0 * PI / (segs as f64);
        let x = radius * theta.cos();
        let y = radius * theta.sin();
        vertices.push(Vertex::new([x, y, h2], [0.0, 0.0, 1.0]));
    }

    for i in 0..segs {
        let next = (i + 1) % segs;
        indices.push([
            top_center_idx,
            top_rim_start + (i as u32),
            top_rim_start + (next as u32),
        ]);
    }

    // 2. Bottom cap (z = -h2, normal = [0, 0, -1])
    let bot_center_idx = vertices.len() as u32;
    vertices.push(Vertex::new([0.0, 0.0, -h2], [0.0, 0.0, -1.0]));

    let bot_rim_start = vertices.len() as u32;
    for i in 0..segs {
        let theta = (i as f64) * 2.0 * PI / (segs as f64);
        let x = radius * theta.cos();
        let y = radius * theta.sin();
        vertices.push(Vertex::new([x, y, -h2], [0.0, 0.0, -1.0]));
    }

    for i in 0..segs {
        let next = (i + 1) % segs;
        // Winding reversed so normal points down (-Z)
        indices.push([
            bot_center_idx,
            bot_rim_start + (next as u32),
            bot_rim_start + (i as u32),
        ]);
    }

    // 3. Side wall quads (split into 2 triangles each)
    let side_start = vertices.len() as u32;
    for i in 0..segs {
        let theta = (i as f64) * 2.0 * PI / (segs as f64);
        let cos_t = theta.cos();
        let sin_t = theta.sin();
        let x = radius * cos_t;
        let y = radius * sin_t;
        let normal = [cos_t, sin_t, 0.0];

        // Top vertex
        vertices.push(Vertex::new([x, y, h2], normal));
        // Bottom vertex
        vertices.push(Vertex::new([x, y, -h2], normal));
    }

    for i in 0..segs {
        let next = (i + 1) % segs;
        let curr_top = side_start + (i as u32) * 2;
        let curr_bot = curr_top + 1;
        let next_top = side_start + (next as u32) * 2;
        let next_bot = next_top + 1;

        // Tri 1: curr_top -> curr_bot -> next_top (CCW outward)
        indices.push([curr_top, curr_bot, next_top]);
        // Tri 2: curr_bot -> next_bot -> next_top (CCW outward)
        indices.push([curr_bot, next_bot, next_top]);
    }

    TriangleMesh::new(vertices, indices)
}

/// Generates a solid 3D UV sphere centered at the origin.
pub fn create_sphere(radius: f64, sectors: usize, stacks: usize) -> TriangleMesh {
    let sectors = sectors.max(6);
    let stacks = stacks.max(4);

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    for i in 0..=stacks {
        let phi = PI * 0.5 - (i as f64) * PI / (stacks as f64);
        let z = radius * phi.sin();
        let xy = radius * phi.cos();

        for j in 0..=sectors {
            let theta = (j as f64) * 2.0 * PI / (sectors as f64);
            let x = xy * theta.cos();
            let y = xy * theta.sin();

            let norm = [x / radius, y / radius, z / radius];
            vertices.push(Vertex::new([x, y, z], norm));
        }
    }

    for i in 0..stacks {
        for j in 0..sectors {
            let k1 = (i * (sectors + 1) + j) as u32;
            let k2 = k1 + (sectors + 1) as u32;

            if i != 0 {
                indices.push([k1, k2, k1 + 1]);
            }
            if i != (stacks - 1) {
                indices.push([k1 + 1, k2, k2 + 1]);
            }
        }
    }

    TriangleMesh::new(vertices, indices)
}

/// Generates an oriented cylinder with specified center, axis direction vector, radius, and height.
pub fn create_cylinder_oriented(
    center: [f64; 3],
    axis: [f64; 3],
    radius: f64,
    height: f64,
    segments: usize,
) -> TriangleMesh {
    let mut mesh = create_cylinder(radius, height, segments);

    let ax_vec = Vector3::new(axis[0], axis[1], axis[2]);
    let ax_len = ax_vec.norm();
    let unit_axis = if ax_len > 1e-12 {
        ax_vec / ax_len
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    };

    let z_unit = Vector3::new(0.0, 0.0, 1.0);
    let rot = if let Some(r) = Rotation3::rotation_between(&z_unit, &unit_axis) {
        r
    } else {
        // Opposite direction (180 deg rotation around X)
        Rotation3::from_axis_angle(&Vector3::x_axis(), PI)
    };

    for v in &mut mesh.vertices {
        let pos = rot * v.pos_vec();
        let n = rot * v.normal_vec();
        v.position = [
            pos.x + center[0],
            pos.y + center[1],
            pos.z + center[2],
        ];
        v.normal = [n.x, n.y, n.z];
    }

    mesh
}

/// Cuts a cylindrical hole in `mesh` at the specified `center` location,
/// oriented along `normal` with radius `radius` and depth `depth`.
///
/// Uses Boolean difference (CSG subtraction).
pub fn cut_hole(
    mesh: &TriangleMesh,
    center: [f64; 3],
    normal: [f64; 3],
    radius: f64,
    depth: f64,
) -> TriangleMesh {
    let norm_v = Vector3::new(normal[0], normal[1], normal[2]);
    let norm_len = norm_v.norm();
    let unit_norm = if norm_len > 1e-12 {
        norm_v / norm_len
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    };

    // To prevent coplanar artifact issues at the surface,
    // slightly extend the cutting tool outward along +normal,
    // and drill into -normal.
    let margin = (depth * 0.02).clamp(1e-4, 0.05);
    let tool_height = depth + margin * 2.0;

    // Drill inward in direction: -unit_norm
    let drill_dir = -unit_norm;
    let tool_center = Vector3::new(center[0], center[1], center[2])
        + drill_dir * (depth * 0.5)
        + unit_norm * margin;

    let tool = create_cylinder_oriented(
        [tool_center.x, tool_center.y, tool_center.z],
        [drill_dir.x, drill_dir.y, drill_dir.z],
        radius,
        tool_height,
        32,
    );

    csg_difference(mesh, &tool)
}

/// Generates a solid 3D cone or truncated cone (frustum) centered at the origin along the Z axis.
///
/// If `radius_top == 0.0`, a sharp cone with its apex at `z = +height/2` is created.
/// If `radius_top > 0.0`, a frustum with flat top cap at `z = +height/2` is created.
/// Coordinates span: `[-height/2, +height/2]` along Z axis.
pub fn create_cone(radius_bottom: f64, radius_top: f64, height: f64, segments: usize) -> TriangleMesh {
    if height <= 1e-12 || (radius_bottom <= 1e-12 && radius_top <= 1e-12) {
        return TriangleMesh::empty();
    }

    let segs = segments.max(8);
    let h2 = height * 0.5;

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    let has_bot_cap = radius_bottom > 1e-12;
    let has_top_cap = radius_top > 1e-12;

    // 1. Bottom cap (z = -h2, normal = [0, 0, -1])
    if has_bot_cap {
        let bot_center_idx = vertices.len() as u32;
        vertices.push(Vertex::new([0.0, 0.0, -h2], [0.0, 0.0, -1.0]));

        let bot_rim_start = vertices.len() as u32;
        for i in 0..segs {
            let theta = (i as f64) * 2.0 * PI / (segs as f64);
            let x = radius_bottom * theta.cos();
            let y = radius_bottom * theta.sin();
            vertices.push(Vertex::new([x, y, -h2], [0.0, 0.0, -1.0]));
        }

        for i in 0..segs {
            let next = (i + 1) % segs;
            // CCW when viewed from outside (-Z): [center, next, curr]
            indices.push([
                bot_center_idx,
                bot_rim_start + (next as u32),
                bot_rim_start + (i as u32),
            ]);
        }
    }

    // 2. Top cap (z = +h2, normal = [0, 0, 1]) if truncated cone
    if has_top_cap {
        let top_center_idx = vertices.len() as u32;
        vertices.push(Vertex::new([0.0, 0.0, h2], [0.0, 0.0, 1.0]));

        let top_rim_start = vertices.len() as u32;
        for i in 0..segs {
            let theta = (i as f64) * 2.0 * PI / (segs as f64);
            let x = radius_top * theta.cos();
            let y = radius_top * theta.sin();
            vertices.push(Vertex::new([x, y, h2], [0.0, 0.0, 1.0]));
        }

        for i in 0..segs {
            let next = (i + 1) % segs;
            // CCW when viewed from outside (+Z): [center, curr, next]
            indices.push([
                top_center_idx,
                top_rim_start + (i as u32),
                top_rim_start + (next as u32),
            ]);
        }
    }

    // 3. Side wall
    let dr = radius_bottom - radius_top;
    let slant = (height * height + dr * dr).sqrt();
    let (nz, n_radial) = if slant > 1e-12 {
        (dr / slant, height / slant)
    } else {
        (0.0, 1.0)
    };

    if !has_top_cap {
        // Sharp cone: apex at [0, 0, h2]
        let side_start = vertices.len() as u32;
        for i in 0..segs {
            let theta = (i as f64) * 2.0 * PI / (segs as f64);
            let cos_t = theta.cos();
            let sin_t = theta.sin();
            let x = radius_bottom * cos_t;
            let y = radius_bottom * sin_t;
            let norm = [n_radial * cos_t, n_radial * sin_t, nz];
            vertices.push(Vertex::new([x, y, -h2], norm));
        }

        let apex_idx = vertices.len() as u32;
        vertices.push(Vertex::new([0.0, 0.0, h2], [0.0, 0.0, 1.0]));

        for i in 0..segs {
            let next = (i + 1) % segs;
            let curr_bot = side_start + (i as u32);
            let next_bot = side_start + (next as u32);

            // CCW outward: [curr_bot, next_bot, apex]
            indices.push([curr_bot, next_bot, apex_idx]);
        }
    } else if !has_bot_cap {
        // Inverted sharp cone: apex at [0, 0, -h2]
        let side_start = vertices.len() as u32;
        for i in 0..segs {
            let theta = (i as f64) * 2.0 * PI / (segs as f64);
            let cos_t = theta.cos();
            let sin_t = theta.sin();
            let x = radius_top * cos_t;
            let y = radius_top * sin_t;
            let norm = [n_radial * cos_t, n_radial * sin_t, nz];
            vertices.push(Vertex::new([x, y, h2], norm));
        }

        let apex_idx = vertices.len() as u32;
        vertices.push(Vertex::new([0.0, 0.0, -h2], [0.0, 0.0, -1.0]));

        for i in 0..segs {
            let next = (i + 1) % segs;
            let curr_top = side_start + (i as u32);
            let next_top = side_start + (next as u32);

            // CCW outward: [curr_top, apex, next_top]
            indices.push([curr_top, apex_idx, next_top]);
        }
    } else {
        // Truncated cone (frustum)
        let side_start = vertices.len() as u32;
        for i in 0..segs {
            let theta = (i as f64) * 2.0 * PI / (segs as f64);
            let cos_t = theta.cos();
            let sin_t = theta.sin();
            let norm = [n_radial * cos_t, n_radial * sin_t, nz];

            // Top vertex
            let xt = radius_top * cos_t;
            let yt = radius_top * sin_t;
            vertices.push(Vertex::new([xt, yt, h2], norm));

            // Bottom vertex
            let xb = radius_bottom * cos_t;
            let yb = radius_bottom * sin_t;
            vertices.push(Vertex::new([xb, yb, -h2], norm));
        }

        for i in 0..segs {
            let next = (i + 1) % segs;
            let curr_top = side_start + (i as u32) * 2;
            let curr_bot = curr_top + 1;
            let next_top = side_start + (next as u32) * 2;
            let next_bot = next_top + 1;

            // Tri 1: curr_top -> curr_bot -> next_top (CCW outward)
            indices.push([curr_top, curr_bot, next_top]);
            // Tri 2: curr_bot -> next_bot -> next_top (CCW outward)
            indices.push([curr_bot, next_bot, next_top]);
        }
    }

    TriangleMesh::new(vertices, indices)
}

/// Generates a closed, watertight (manifold) 3D mesh for a helical cutting tool
/// (sweeping a circular cross section of radius `groove_radius` along a helix).
///
/// Supports multi-start flutes (`starts`), right/left handedness, and end caps for clean CSG.
#[allow(clippy::too_many_arguments)]
pub fn create_helical_cutter(
    helix_radius: f64,
    groove_radius: f64,
    pitch: f64,
    height: f64,
    start_z: f64,
    starts: usize,
    is_right_hand: bool,
    turns: f64,
) -> TriangleMesh {
    if pitch <= 1e-9 || groove_radius <= 1e-9 {
        return TriangleMesh::empty();
    }

    let effective_turns = if turns > 0.0 {
        turns
    } else if height > 0.0 {
        height / pitch
    } else {
        1.0
    };

    if effective_turns <= 0.0 {
        return TriangleMesh::empty();
    }

    let num_starts = starts.max(1);
    let cross_segments: usize = 16;
    let steps_per_turn: usize = 24;
    let k_steps = ((effective_turns * steps_per_turn as f64).round() as usize).max(8);

    let mut all_vertices = Vec::new();
    let mut all_indices = Vec::new();

    let s = if is_right_hand { 1.0 } else { -1.0 };
    let c = pitch / (2.0 * PI);
    let h_len = (helix_radius * helix_radius + c * c).sqrt();
    let total_sweep_angle = 2.0 * PI * effective_turns;

    for start_idx in 0..num_starts {
        let phi_0 = (start_idx as f64) * 2.0 * PI / (num_starts as f64);
        let start_vertex_offset = all_vertices.len() as u32;

        // Generate circular ring vertices at each step j along the helix
        for j in 0..=k_steps {
            let u = (j as f64) / (k_steps as f64);
            let t = u * total_sweep_angle;
            let theta = phi_0 + s * t;
            let z = start_z + (t / (2.0 * PI)) * pitch;

            let cos_t = theta.cos();
            let sin_t = theta.sin();

            let center = Vector3::new(helix_radius * cos_t, helix_radius * sin_t, z);
            let e_r = Vector3::new(cos_t, sin_t, 0.0);
            let e_b = if h_len > 1e-12 {
                Vector3::new(-c * sin_t / h_len, c * cos_t / h_len, -s * helix_radius / h_len)
            } else {
                Vector3::new(0.0, 0.0, 1.0)
            };

            for i in 0..cross_segments {
                let alpha = (i as f64) * 2.0 * PI / (cross_segments as f64);
                let cos_a = alpha.cos();
                let sin_a = alpha.sin();

                let n = e_r * cos_a + e_b * sin_a;
                let pos = center + n * groove_radius;

                all_vertices.push(Vertex::new([pos.x, pos.y, pos.z], [n.x, n.y, n.z]));
            }
        }

        // Quads along the helical tube
        let m = cross_segments;
        for j in 0..k_steps {
            for i in 0..m {
                let next_i = (i + 1) % m;
                let v00 = start_vertex_offset + (j * m + i) as u32;
                let v01 = start_vertex_offset + (j * m + next_i) as u32;
                let v10 = start_vertex_offset + ((j + 1) * m + i) as u32;
                let v11 = start_vertex_offset + ((j + 1) * m + next_i) as u32;

                all_indices.push([v00, v01, v11]);
                all_indices.push([v00, v11, v10]);
            }
        }

        // 1. Start cap at j = 0 (outward normal: -T_unit)
        let theta0 = phi_0;
        let z0 = start_z;
        let c0 = Vector3::new(helix_radius * theta0.cos(), helix_radius * theta0.sin(), z0);
        let tan0 = Vector3::new(-s * helix_radius * theta0.sin(), s * helix_radius * theta0.cos(), c);
        let n_start = if h_len > 1e-12 { -tan0 / h_len } else { Vector3::new(0.0, 0.0, -1.0) };

        let start_cap_center_idx = all_vertices.len() as u32;
        all_vertices.push(Vertex::new([c0.x, c0.y, c0.z], [n_start.x, n_start.y, n_start.z]));

        for i in 0..m {
            let next_i = (i + 1) % m;
            let vi = start_vertex_offset + i as u32;
            let v_next = start_vertex_offset + next_i as u32;
            all_indices.push([start_cap_center_idx, v_next, vi]);
        }

        // 2. End cap at j = k_steps (outward normal: +T_unit)
        let tk = total_sweep_angle;
        let thetak = phi_0 + s * tk;
        let zk = start_z + (tk / (2.0 * PI)) * pitch;
        let ck = Vector3::new(helix_radius * thetak.cos(), helix_radius * thetak.sin(), zk);
        let tank = Vector3::new(-s * helix_radius * thetak.sin(), s * helix_radius * thetak.cos(), c);
        let n_end = if h_len > 1e-12 { tank / h_len } else { Vector3::new(0.0, 0.0, 1.0) };

        let end_cap_center_idx = all_vertices.len() as u32;
        all_vertices.push(Vertex::new([ck.x, ck.y, ck.z], [n_end.x, n_end.y, n_end.z]));

        for i in 0..m {
            let next_i = (i + 1) % m;
            let vi = start_vertex_offset + (k_steps * m + i) as u32;
            let v_next = start_vertex_offset + (k_steps * m + next_i) as u32;
            all_indices.push([end_cap_center_idx, vi, v_next]);
        }
    }

    TriangleMesh::new(all_vertices, all_indices)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_box_volume() {
        let b = create_box(2.0, 3.0, 4.0);
        let vol = b.volume();
        assert!((vol - 24.0).abs() < 1e-6, "Box volume should be 24.0, got {}", vol);
        assert_eq!(b.triangle_count(), 12);
        assert_eq!(b.vertex_count(), 24);
    }

    #[test]
    fn test_sphere_volume() {
        let r = 2.0;
        let s = create_sphere(r, 48, 24);
        let expected = (4.0 / 3.0) * PI * r.powi(3);
        let actual = s.volume();
        // Discretized sphere volume is slightly less than smooth sphere (~1-2%)
        let diff_ratio = (actual - expected).abs() / expected;
        assert!(
            diff_ratio < 0.03,
            "Sphere volume {} should be close to {}, diff: {}",
            actual,
            expected,
            diff_ratio
        );
    }

    #[test]
    fn test_cylinder_volume() {
        let r = 1.5;
        let h = 5.0;
        let c = create_cylinder(r, h, 64);
        let expected = PI * r * r * h;
        let actual = c.volume();
        let diff_ratio = (actual - expected).abs() / expected;
        assert!(
            diff_ratio < 0.01,
            "Cylinder volume {} should be within 1% of {}, diff: {}",
            actual,
            expected,
            diff_ratio
        );
    }

    #[test]
    fn test_cone_sharp_volume() {
        let r = 3.0;
        let h = 6.0;
        let cone = create_cone(r, 0.0, h, 64);
        let expected = (1.0 / 3.0) * PI * r * r * h;
        let actual = cone.volume();
        let diff_ratio = (actual - expected).abs() / expected;
        assert!(
            diff_ratio < 0.01,
            "Sharp cone volume {} should be within 1% of {}, diff: {}",
            actual,
            expected,
            diff_ratio
        );
    }

    #[test]
    fn test_cone_frustum_volume() {
        let r_bot = 4.0;
        let r_top = 2.0;
        let h = 5.0;
        let frustum = create_cone(r_bot, r_top, h, 64);
        let expected = (1.0 / 3.0) * PI * h * (r_bot * r_bot + r_bot * r_top + r_top * r_top);
        let actual = frustum.volume();
        let diff_ratio = (actual - expected).abs() / expected;
        assert!(
            diff_ratio < 0.01,
            "Frustum volume {} should be within 1% of {}, diff: {}",
            actual,
            expected,
            diff_ratio
        );
    }

    #[test]
    fn test_helical_cutter_volume_and_manifold() {
        let cutter = create_helical_cutter(4.0, 1.5, 20.0, 40.0, 0.0, 2, true, 2.0);
        assert!(cutter.triangle_count() > 0);
        assert!(cutter.vertex_count() > 0);
        let vol = cutter.volume();
        assert!(vol > 0.0, "Helical cutter volume should be positive, got {}", vol);

        // Check AABB spans expected Z range ~ [0.0 - r_g, 40.0 + r_g]
        let aabb = cutter.bounding_box();
        assert!(aabb.min[2] < 1.0);
        assert!(aabb.max[2] > 39.0);
    }
}
