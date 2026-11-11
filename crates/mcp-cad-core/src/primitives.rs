//! 3D solid primitive generators: Box, Cylinder, Sphere, and CutHole.

use std::f64::consts::PI;
use nalgebra::{Rotation3, Vector3};
use crate::csg::csg_difference;
use crate::tessellation::{TessellationQuality, TessellationSettings};
use crate::topology::{FaceInfo, FaceKind, TriangleMesh, Vertex};

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
    let mut face_ids = Vec::with_capacity(12);
    let mut faces = Vec::with_capacity(6);

    // Helper closure to append a rectangular face (4 vertices, 2 triangles) with its FaceInfo descriptor
    let mut add_face = |fid: u32,
                        role: &str,
                        aliases: &[&str],
                        p0: [f64; 3],
                        p1: [f64; 3],
                        p2: [f64; 3],
                        p3: [f64; 3],
                        normal: [f64; 3]| {
        let base = vertices.len() as u32;
        vertices.push(Vertex::new(p0, normal));
        vertices.push(Vertex::new(p1, normal));
        vertices.push(Vertex::new(p2, normal));
        vertices.push(Vertex::new(p3, normal));

        // CCW when viewed from outside
        indices.push([base, base + 1, base + 2]);
        indices.push([base, base + 2, base + 3]);
        face_ids.push(fid);
        face_ids.push(fid);

        faces.push(FaceInfo::new_descriptor(
            fid,
            role,
            aliases.iter().map(|s| (*s).to_string()).collect(),
            FaceKind::Plane,
            normal,
            None,
        ));
    };

    // 1. Top face (+Z)
    add_face(
        1,
        "top",
        &["cap_top", "max_z", "+z", "front_z"],
        [-w2, -h2,  d2],
        [ w2, -h2,  d2],
        [ w2,  h2,  d2],
        [-w2,  h2,  d2],
        [0.0, 0.0, 1.0],
    );

    // 2. Bottom face (-Z)
    add_face(
        2,
        "bottom",
        &["cap_bottom", "min_z", "-z", "back_z"],
        [ w2, -h2, -d2],
        [-w2, -h2, -d2],
        [-w2,  h2, -d2],
        [ w2,  h2, -d2],
        [0.0, 0.0, -1.0],
    );

    // 3. Right face (+X)
    add_face(
        3,
        "right",
        &["max_x", "+x"],
        [ w2, -h2,  d2],
        [ w2, -h2, -d2],
        [ w2,  h2, -d2],
        [ w2,  h2,  d2],
        [1.0, 0.0, 0.0],
    );

    // 4. Left face (-X)
    add_face(
        4,
        "left",
        &["min_x", "-x"],
        [-w2, -h2, -d2],
        [-w2, -h2,  d2],
        [-w2,  h2,  d2],
        [-w2,  h2, -d2],
        [-1.0, 0.0, 0.0],
    );

    // 5. Back face (+Y)
    add_face(
        5,
        "back",
        &["max_y", "+y", "top_y"],
        [-w2,  h2,  d2],
        [ w2,  h2,  d2],
        [ w2,  h2, -d2],
        [-w2,  h2, -d2],
        [0.0, 1.0, 0.0],
    );

    // 6. Front face (-Y)
    add_face(
        6,
        "front",
        &["min_y", "-y", "bottom_y"],
        [-w2, -h2, -d2],
        [ w2, -h2, -d2],
        [ w2, -h2,  d2],
        [-w2, -h2,  d2],
        [0.0, -1.0, 0.0],
    );

    TriangleMesh::with_faces(vertices, indices, face_ids, faces)
}

/// Splits a quad `(p0, p1, p2, p3)` along its shorter diagonal to maximize minimum interior angle.
#[inline]
fn push_quad_shortest_diag(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<[u32; 3]>,
    p0: [f64; 3],
    p1: [f64; 3],
    p2: [f64; 3],
    p3: [f64; 3],
    normal: [f64; 3],
) {
    let base = vertices.len() as u32;
    vertices.push(Vertex::new(p0, normal));
    vertices.push(Vertex::new(p1, normal));
    vertices.push(Vertex::new(p2, normal));
    vertices.push(Vertex::new(p3, normal));

    let d02_sq = (p0[0] - p2[0]).powi(2) + (p0[1] - p2[1]).powi(2) + (p0[2] - p2[2]).powi(2);
    let d13_sq = (p1[0] - p3[0]).powi(2) + (p1[1] - p3[1]).powi(2) + (p1[2] - p3[2]).powi(2);
    if d02_sq <= d13_sq * 1.000001 {
        indices.push([base, base + 1, base + 2]);
        indices.push([base, base + 2, base + 3]);
    } else {
        indices.push([base, base + 1, base + 3]);
        indices.push([base + 1, base + 2, base + 3]);
    }
}

/// Connects two equal-count planar 2D rings `ring_a` and `ring_b` at height `z_val` using intermediate
/// concentric rings so radial step size matches circumferential chord length (eliminating cap slivers).
fn add_bridged_ring_strip_2d(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<[u32; 3]>,
    ring_a: &[[f64; 2]],
    ring_b: &[[f64; 2]],
    z_val: f64,
    is_top: bool,
) {
    let m = ring_a.len().min(ring_b.len());
    if m < 3 {
        return;
    }
    let nz = if is_top { 1.0 } else { -1.0 };
    let normal = [0.0, 0.0, nz];

    let mut total_radial = 0.0;
    let mut total_chord = 0.0;
    for i in 0..m {
        let next = (i + 1) % m;
        let a0 = ring_a[i];
        let a1 = ring_a[next];
        let b0 = ring_b[i];
        let b1 = ring_b[next];
        total_radial += (a0[0] - b0[0]).hypot(a0[1] - b0[1]);
        total_chord +=
            0.5 * ((a1[0] - a0[0]).hypot(a1[1] - a0[1]) + (b1[0] - b0[0]).hypot(b1[1] - b0[1]));
    }
    let avg_radial = total_radial / (m as f64);
    let avg_chord = (total_chord / (m as f64)).max(1e-6);
    let n_steps = ((avg_radial / (avg_chord * 1.15)).ceil() as usize).clamp(1, 14);

    let mut prev_ring: Vec<[f64; 2]> = ring_a[..m].to_vec();
    for s in 1..=n_steps {
        let t = (s as f64) / (n_steps as f64);
        let curr_ring: Vec<[f64; 2]> = if s == n_steps {
            ring_b[..m].to_vec()
        } else {
            (0..m)
                .map(|i| {
                    [
                        ring_a[i][0] + (ring_b[i][0] - ring_a[i][0]) * t,
                        ring_a[i][1] + (ring_b[i][1] - ring_a[i][1]) * t,
                    ]
                })
                .collect()
        };

        for i in 0..m {
            let next = (i + 1) % m;
            let p0 = [prev_ring[i][0], prev_ring[i][1], z_val];
            let p1 = [prev_ring[next][0], prev_ring[next][1], z_val];
            let p2 = [curr_ring[next][0], curr_ring[next][1], z_val];
            let p3 = [curr_ring[i][0], curr_ring[i][1], z_val];
            if is_top {
                // ring_a is outer, ring_b is inner: [a_i, a_next, b_next, b_i] is CCW in XY (+Z cross)
                push_quad_shortest_diag(vertices, indices, p0, p1, p2, p3, normal);
            } else {
                push_quad_shortest_diag(vertices, indices, p0, p3, p2, p1, normal);
            }
        }
        prev_ring = curr_ring;
    }
}

/// 2-to-1 vertex halving transition between an `outer_ring` of `2*k` vertices and an `inner_ring` of `k` vertices.
/// Emits 3 well-shaped triangles per pair of outer segments.
fn add_halving_ring_transition_2d(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<[u32; 3]>,
    outer_ring: &[[f64; 2]],
    inner_ring: &[[f64; 2]],
    z_val: f64,
    is_top: bool,
) {
    let k = inner_ring.len();
    if outer_ring.len() != 2 * k || k < 3 {
        return;
    }
    let nz = if is_top { 1.0 } else { -1.0 };
    let normal = [0.0, 0.0, nz];

    let mut push_tri = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        let base = vertices.len() as u32;
        vertices.push(Vertex::new([a[0], a[1], z_val], normal));
        vertices.push(Vertex::new([b[0], b[1], z_val], normal));
        vertices.push(Vertex::new([c[0], c[1], z_val], normal));
        if is_top {
            indices.push([base, base + 1, base + 2]);
        } else {
            indices.push([base, base + 2, base + 1]);
        }
    };

    for j in 0..k {
        let j_next = (j + 1) % k;
        let o0 = outer_ring[2 * j];
        let o1 = outer_ring[2 * j + 1];
        let o2 = outer_ring[2 * j_next];
        let i0 = inner_ring[j];
        let i1 = inner_ring[j_next];
        push_tri(o0, o1, i0);
        push_tri(o1, i1, i0);
        push_tri(o1, o2, i1);
    }
}

/// Triangulates a planar disk cap bounded by `outer_ring` around `center` at height `z_val`
/// using concentric rings and 2-to-1 vertex halving so the central fan has at most 8–16 segments.
fn add_subdivided_disk_cap_2d(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<[u32; 3]>,
    outer_ring: &[[f64; 2]],
    center: [f64; 2],
    z_val: f64,
    is_top: bool,
) {
    let mut curr_ring: Vec<[f64; 2]> = outer_ring.to_vec();
    if curr_ring.len() < 3 {
        return;
    }
    let nz = if is_top { 1.0 } else { -1.0 };
    let normal = [0.0, 0.0, nz];

    let mut init_chord_sum = 0.0;
    for i in 0..curr_ring.len() {
        let next = (i + 1) % curr_ring.len();
        init_chord_sum +=
            (curr_ring[next][0] - curr_ring[i][0]).hypot(curr_ring[next][1] - curr_ring[i][1]);
    }
    let initial_chord = (init_chord_sum / (curr_ring.len() as f64)).max(1e-6);

    for _iter in 0..24 {
        let m = curr_ring.len();
        let mut r_sum = 0.0;
        let mut c_sum = 0.0;
        for i in 0..m {
            let next = (i + 1) % m;
            r_sum += (curr_ring[i][0] - center[0]).hypot(curr_ring[i][1] - center[1]);
            c_sum += (curr_ring[next][0] - curr_ring[i][0])
                .hypot(curr_ring[next][1] - curr_ring[i][1]);
        }
        let avg_r = r_sum / (m as f64);
        let avg_c = (c_sum / (m as f64)).max(1e-6);
        let target_step = initial_chord.max(avg_c);

        if m <= 16 && avg_r <= target_step * 1.45 {
            break;
        }

        if m >= 16 && m.is_multiple_of(2) && avg_c < avg_r * 0.38 {
            let next_scale = ((avg_r - avg_c * 1.15) / avg_r).clamp(0.45, 0.78);
            let half_m = m / 2;
            let halved_ring: Vec<[f64; 2]> = (0..half_m)
                .map(|j| {
                    let p = curr_ring[2 * j];
                    [
                        center[0] + (p[0] - center[0]) * next_scale,
                        center[1] + (p[1] - center[1]) * next_scale,
                    ]
                })
                .collect();
            add_halving_ring_transition_2d(vertices, indices, &curr_ring, &halved_ring, z_val, is_top);
            curr_ring = halved_ring;
        } else if avg_r > target_step * 1.45 {
            let next_scale = ((avg_r - target_step * 0.95) / avg_r).clamp(0.38, 0.78);
            let next_ring: Vec<[f64; 2]> = curr_ring
                .iter()
                .map(|p| {
                    [
                        center[0] + (p[0] - center[0]) * next_scale,
                        center[1] + (p[1] - center[1]) * next_scale,
                    ]
                })
                .collect();
            add_bridged_ring_strip_2d(vertices, indices, &curr_ring, &next_ring, z_val, is_top);
            curr_ring = next_ring;
        } else {
            break;
        }
    }

    let m = curr_ring.len();
    let center_idx = vertices.len() as u32;
    vertices.push(Vertex::new([center[0], center[1], z_val], normal));
    let rim_start = vertices.len() as u32;
    for p in &curr_ring {
        vertices.push(Vertex::new([p[0], p[1], z_val], normal));
    }
    for i in 0..m {
        let next = (i + 1) % m;
        if is_top {
            indices.push([center_idx, rim_start + (i as u32), rim_start + (next as u32)]);
        } else {
            indices.push([center_idx, rim_start + (next as u32), rim_start + (i as u32)]);
        }
    }
}

/// Generates a solid 3D cylinder centered at the origin along the Z axis.
///
/// Coordinates span: `radius` in XY plane, `z in [-height/2, height/2]`.
pub fn create_cylinder(radius: f64, height: f64, segments: usize) -> TriangleMesh {
    let segs = if segments == 0 {
        TessellationSettings::default().calculate_segments(radius)
    } else {
        segments.max(8)
    };
    let h2 = height * 0.5;

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    let rim_ring: Vec<[f64; 2]> = (0..segs)
        .map(|i| {
            let theta = (i as f64) * 2.0 * PI / (segs as f64);
            [radius * theta.cos(), radius * theta.sin()]
        })
        .collect();

    // 1. Top and bottom caps with concentric ring subdivision + 2-to-1 halving
    add_subdivided_disk_cap_2d(&mut vertices, &mut indices, &rim_ring, [0.0, 0.0], h2, true);
    let top_cap_end = indices.len();

    add_subdivided_disk_cap_2d(&mut vertices, &mut indices, &rim_ring, [0.0, 0.0], -h2, false);
    let bot_cap_end = indices.len();

    // 2. Side wall subdivided axially so height steps match circumferential chord length
    let chord = (2.0 * radius * (PI / (segs as f64)).sin()).max(1e-6);
    let n_z = ((height / (chord * 1.25)).ceil() as usize).clamp(1, 64);

    for k in 0..n_z {
        let z_top = h2 - (k as f64) * (height / (n_z as f64));
        let z_bot = h2 - ((k + 1) as f64) * (height / (n_z as f64));
        for i in 0..segs {
            let next = (i + 1) % segs;
            let p0 = rim_ring[i];
            let p1 = rim_ring[next];
            let mid_ang = ((i as f64) + 0.5) * 2.0 * PI / (segs as f64);
            let normal = [mid_ang.cos(), mid_ang.sin(), 0.0];
            push_quad_shortest_diag(
                &mut vertices,
                &mut indices,
                [p0[0], p0[1], z_top],
                [p0[0], p0[1], z_bot],
                [p1[0], p1[1], z_bot],
                [p1[0], p1[1], z_top],
                normal,
            );
        }
    }

    let total_tris = indices.len();
    let mut face_ids = Vec::with_capacity(total_tris);
    face_ids.resize(top_cap_end, 1);
    face_ids.resize(bot_cap_end, 2);
    face_ids.resize(total_tris, 3);

    let faces = vec![
        FaceInfo::new_descriptor(
            1,
            "cap_top",
            vec!["top".to_string(), "max_z".to_string(), "+z".to_string()],
            FaceKind::Plane,
            [0.0, 0.0, 1.0],
            Some(radius),
        ),
        FaceInfo::new_descriptor(
            2,
            "cap_bottom",
            vec!["bottom".to_string(), "min_z".to_string(), "-z".to_string()],
            FaceKind::Plane,
            [0.0, 0.0, -1.0],
            Some(radius),
        ),
        FaceInfo::new_descriptor(
            3,
            "side",
            vec![
                "outer_wall".to_string(),
                "cylindrical_wall".to_string(),
                "barrel".to_string(),
                "outer".to_string(),
                "mantle".to_string(),
            ],
            FaceKind::Cylinder,
            [0.0, 0.0, 1.0],
            Some(radius),
        ),
    ];

    TriangleMesh::with_faces(vertices, indices, face_ids, faces)
}

/// Generates a solid 3D cylinder with adaptive tessellation based on `TessellationSettings`.
pub fn create_cylinder_with_settings(
    radius: f64,
    height: f64,
    settings: &TessellationSettings,
) -> TriangleMesh {
    let segments = settings.calculate_segments(radius);
    create_cylinder(radius, height, segments)
}

/// Generates a solid 3D UV sphere centered at the origin with polar ring halving to prevent pole slivers.
pub fn create_sphere(radius: f64, sectors: usize, stacks: usize) -> TriangleMesh {
    let (def_sec, def_stk) = TessellationSettings::default().calculate_sphere_resolution(radius);
    let base_sectors = if sectors == 0 { def_sec } else { sectors.max(8) };
    let stacks = if stacks == 0 { def_stk } else { stacks.max(8) };

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Build latitude rings from south pole (-h2) to north pole (+h2) with adaptive sector count
    // (halving sectors near poles so circumferential chord never drops below 1/5 of meridional step)
    let mut rings: Vec<Vec<[f64; 3]>> = Vec::with_capacity(stacks - 1);
    for i in 1..stacks {
        let phi = -PI * 0.5 + (i as f64) * PI / (stacks as f64);
        let z = radius * phi.sin();
        let r_xy = radius * phi.cos();
        let mut ring_segs = base_sectors;
        while ring_segs > 8 && (2.0 * PI * r_xy / (ring_segs as f64)) < (PI * radius / (stacks as f64)) * 0.42 && ring_segs % 2 == 0 {
            ring_segs /= 2;
        }
        let ring: Vec<[f64; 3]> = (0..ring_segs)
            .map(|j| {
                let theta = (j as f64) * 2.0 * PI / (ring_segs as f64);
                [r_xy * theta.cos(), r_xy * theta.sin(), z]
            })
            .collect();
        rings.push(ring);
    }

    // South pole fan (z = -radius)
    let south_pole = [0.0, 0.0, -radius];
    let south_ring = &rings[0];
    for j in 0..south_ring.len() {
        let next = (j + 1) % south_ring.len();
        let a = south_pole;
        let b = south_ring[next];
        let c = south_ring[j];
        let base = vertices.len() as u32;
        vertices.push(Vertex::new(a, [0.0, 0.0, -1.0]));
        vertices.push(Vertex::new(b, [b[0] / radius, b[1] / radius, b[2] / radius]));
        vertices.push(Vertex::new(c, [c[0] / radius, c[1] / radius, c[2] / radius]));
        indices.push([base, base + 1, base + 2]);
    }

    // Intermediate latitude bands
    for r_idx in 0..(rings.len() - 1) {
        let r0 = &rings[r_idx];
        let r1 = &rings[r_idx + 1];
        let n0 = r0.len();
        let n1 = r1.len();
        if n0 == n1 {
            for j in 0..n0 {
                let next = (j + 1) % n0;
                let p0 = r0[j];
                let p1 = r0[next];
                let p2 = r1[next];
                let p3 = r1[j];
                let n = [
                    (p0[0] + p2[0]) * 0.5 / radius,
                    (p0[1] + p2[1]) * 0.5 / radius,
                    (p0[2] + p2[2]) * 0.5 / radius,
                ];
                push_quad_shortest_diag(&mut vertices, &mut indices, p0, p1, p2, p3, n);
            }
        } else if n1 == 2 * n0 {
            for j in 0..n0 {
                let next = (j + 1) % n0;
                let i0 = r0[j];
                let i1 = r0[next];
                let o0 = r1[2 * j];
                let o1 = r1[2 * j + 1];
                let o2 = r1[2 * next];
                for (a, b, c) in [(i0, o1, o0), (i0, i1, o1), (i1, o2, o1)] {
                    let base = vertices.len() as u32;
                    vertices.push(Vertex::new(a, [a[0] / radius, a[1] / radius, a[2] / radius]));
                    vertices.push(Vertex::new(b, [b[0] / radius, b[1] / radius, b[2] / radius]));
                    vertices.push(Vertex::new(c, [c[0] / radius, c[1] / radius, c[2] / radius]));
                    indices.push([base, base + 1, base + 2]);
                }
            }
        } else if n0 == 2 * n1 {
            for j in 0..n1 {
                let next = (j + 1) % n1;
                let o0 = r0[2 * j];
                let o1 = r0[2 * j + 1];
                let o2 = r0[2 * next];
                let i0 = r1[j];
                let i1 = r1[next];
                for (a, b, c) in [(o0, o1, i0), (o1, i1, i0), (o1, o2, i1)] {
                    let base = vertices.len() as u32;
                    vertices.push(Vertex::new(a, [a[0] / radius, a[1] / radius, a[2] / radius]));
                    vertices.push(Vertex::new(b, [b[0] / radius, b[1] / radius, b[2] / radius]));
                    vertices.push(Vertex::new(c, [c[0] / radius, c[1] / radius, c[2] / radius]));
                    indices.push([base, base + 1, base + 2]);
                }
            }
        }
    }

    // North pole fan (z = +radius)
    let north_pole = [0.0, 0.0, radius];
    let north_ring = rings.last().unwrap();
    for j in 0..north_ring.len() {
        let next = (j + 1) % north_ring.len();
        let a = north_pole;
        let b = north_ring[j];
        let c = north_ring[next];
        let base = vertices.len() as u32;
        vertices.push(Vertex::new(a, [0.0, 0.0, 1.0]));
        vertices.push(Vertex::new(b, [b[0] / radius, b[1] / radius, b[2] / radius]));
        vertices.push(Vertex::new(c, [c[0] / radius, c[1] / radius, c[2] / radius]));
        indices.push([base, base + 1, base + 2]);
    }

    let face_ids = vec![1; indices.len()];
    let faces = vec![FaceInfo::new_descriptor(
        1,
        "surface",
        vec![
            "sphere_surface".to_string(),
            "side".to_string(),
            "outer".to_string(),
            "shell".to_string(),
        ],
        FaceKind::Sphere,
        [0.0, 0.0, 1.0],
        Some(radius),
    )];

    TriangleMesh::with_faces(vertices, indices, face_ids, faces)
}

/// Generates a solid 3D UV sphere with adaptive tessellation based on `TessellationSettings`.
pub fn create_sphere_with_settings(
    radius: f64,
    settings: &TessellationSettings,
) -> TriangleMesh {
    let (sectors, stacks) = settings.calculate_sphere_resolution(radius);
    create_sphere(radius, sectors, stacks)
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

    let t = Vector3::new(center[0], center[1], center[2]);
    for face in &mut mesh.faces {
        let c = rot * Vector3::new(face.centroid[0], face.centroid[1], face.centroid[2]) + t;
        let s = rot
            * Vector3::new(
                face.sample_point[0],
                face.sample_point[1],
                face.sample_point[2],
            )
            + t;
        let n = rot
            * Vector3::new(
                face.normal_or_axis[0],
                face.normal_or_axis[1],
                face.normal_or_axis[2],
            );
        face.centroid = [c.x, c.y, c.z];
        face.sample_point = [s.x, s.y, s.z];
        face.normal_or_axis = [n.x, n.y, n.z];
    }

    mesh
}

/// Generates an oriented cylinder with adaptive tessellation based on `TessellationSettings`.
pub fn create_cylinder_oriented_with_settings(
    center: [f64; 3],
    axis: [f64; 3],
    radius: f64,
    height: f64,
    settings: &TessellationSettings,
) -> TriangleMesh {
    let segments = settings.calculate_segments(radius);
    create_cylinder_oriented(center, axis, radius, height, segments)
}

/// Cuts a cylindrical hole in `mesh` at the specified `center` location,
/// oriented along `normal` with radius `radius` and depth `depth`.
/// Optionally cuts a DIN 6885 keyway slot of specified `keyway_width` and `keyway_depth`.
///
/// Uses Boolean difference (CSG subtraction).
pub fn cut_hole(
    mesh: &TriangleMesh,
    center: [f64; 3],
    normal: [f64; 3],
    radius: f64,
    depth: f64,
    keyway_width: Option<f64>,
    keyway_depth: Option<f64>,
) -> TriangleMesh {
    cut_hole_with_settings(
        mesh,
        center,
        normal,
        radius,
        depth,
        keyway_width,
        keyway_depth,
        &TessellationSettings::default(),
    )
}

/// Cuts a cylindrical hole in `mesh` with adaptive tessellation based on `TessellationSettings`.
#[allow(clippy::too_many_arguments)]
pub fn cut_hole_with_settings(
    mesh: &TriangleMesh,
    center: [f64; 3],
    normal: [f64; 3],
    radius: f64,
    depth: f64,
    keyway_width: Option<f64>,
    keyway_depth: Option<f64>,
    settings: &TessellationSettings,
) -> TriangleMesh {
    if radius <= 1e-12 || depth <= 1e-12 || mesh.indices.is_empty() {
        return mesh.clone();
    }

    let norm_v = Vector3::new(normal[0], normal[1], normal[2]);
    let norm_len = norm_v.norm();
    let unit_norm = if norm_len > 1e-12 {
        norm_v / norm_len
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    };

    let center_vec = Vector3::new(center[0], center[1], center[2]);
    let mut t_min = f64::INFINITY;
    let mut t_max = f64::NEG_INFINITY;
    for v in &mesh.vertices {
        let t = (v.pos_vec() - center_vec).dot(&unit_norm);
        if t < t_min {
            t_min = t;
        }
        if t > t_max {
            t_max = t;
        }
    }
    let span = (t_max - t_min).max(0.0);
    let margin = (depth * 0.05).clamp(1e-3, 0.05);

    // Determine axial start/end along +unit_norm relative to center_vec:
    // If depth covers the part span along unit_norm, treat as a through-hole spanning [t_min - margin, t_max + margin].
    // Otherwise drill from the entry side (t_entry) inward by depth along -unit_norm.
    let (t_high, t_low) = if span > 1e-9 && depth >= span - 1e-4 {
        (t_max + margin, t_min - margin)
    } else {
        let t_entry = if t_max.abs() <= 1e-4 { 0.0 } else { t_max };
        let mut t_bottom = t_entry - depth;
        if span > 1e-9 && (t_bottom - t_min).abs() < 5e-4 {
            if depth >= span * 0.95 {
                t_bottom = t_min - margin;
            } else {
                t_bottom = t_min + 1e-3;
            }
        }
        (t_entry + margin, t_bottom)
    };

    let tool_height = (t_high - t_low).max(1e-5);
    let drill_dir = -unit_norm;
    let tool_center = center_vec + unit_norm * (0.5 * (t_high + t_low));

    let mut final_tool = if let (Some(kw), Some(kd)) = (keyway_width, keyway_depth) {
        if kw > 1e-6 && kd > 1e-6 {
            build_keyway_cutter_prism(
                tool_center,
                drill_dir,
                radius,
                tool_height,
                kw,
                kd,
                settings.calculate_segments(radius).max(24),
            )
        } else {
            create_cylinder_oriented_with_settings(
                [tool_center.x, tool_center.y, tool_center.z],
                [drill_dir.x, drill_dir.y, drill_dir.z],
                radius,
                tool_height,
                settings,
            )
        }
    } else {
        create_cylinder_oriented_with_settings(
            [tool_center.x, tool_center.y, tool_center.z],
            [drill_dir.x, drill_dir.y, drill_dir.z],
            radius,
            tool_height,
            settings,
        )
    };

    // Relabel standard cylinder tool faces (cap_top -> hole_bottom, cap_bottom -> hole_top, side -> hole_wall)
    for face in &mut final_tool.faces {
        match face.role.as_str() {
            "cap_top" => {
                face.name = "hole_bottom".to_string();
                face.role = "hole_bottom".to_string();
                face.aliases = vec!["blind_bottom".to_string(), "bottom".to_string()];
            }
            "cap_bottom" => {
                face.name = "hole_top".to_string();
                face.role = "hole_top".to_string();
                face.aliases = vec![];
            }
            "side" => {
                face.name = "hole_wall".to_string();
                face.role = "hole_wall".to_string();
                face.aliases = vec![
                    "bore".to_string(),
                    "hole".to_string(),
                    "inner_wall".to_string(),
                    "side".to_string(),
                ];
                face.normal_or_axis = [unit_norm.x, unit_norm.y, unit_norm.z];
            }
            _ => {}
        }
    }

    csg_difference(mesh, &final_tool)
}

/// Builds a single watertight manifold cutter prism consisting of a cylinder of `radius`
/// with a rectangular keyway slot (`kw` wide, `kd` deep beyond `radius`) extruded along `axis`.
fn build_keyway_cutter_prism(
    center: Vector3<f64>,
    axis: Vector3<f64>,
    radius: f64,
    height: f64,
    keyway_width: f64,
    keyway_depth: f64,
    segments: usize,
) -> TriangleMesh {
    let w = axis.normalize();
    let up = if w.y.abs() < 0.9 {
        Vector3::new(0.0, 1.0, 0.0)
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    };
    let u = w.cross(&up).normalize();
    let v = w.cross(&u).normalize();

    let half_kw = (keyway_width * 0.5).clamp(radius * 0.02, radius * 0.92);
    let alpha = (half_kw / radius).clamp(0.02, 0.95).asin();
    let y_shoulder = (radius * radius - half_kw * half_kw).max(0.0).sqrt();
    let y_top = radius + keyway_depth;

    let mut profile: Vec<[f64; 2]> = Vec::with_capacity(segments + 4);
    profile.push([half_kw, y_shoulder]);
    profile.push([half_kw, y_top]);
    profile.push([-half_kw, y_top]);
    profile.push([-half_kw, y_shoulder]);

    let ang0 = std::f64::consts::FRAC_PI_2 + alpha;
    let ang1 = 1.5 * std::f64::consts::PI - alpha;
    let ang2 = 1.5 * std::f64::consts::PI + alpha;
    let ang3 = 2.5 * std::f64::consts::PI - alpha;
    let step_ang = 2.0 * std::f64::consts::PI / (segments.max(16) as f64);

    for (k, (a_start, a_end)) in [(ang0, ang1), (ang1, ang2), (ang2, ang3)]
        .into_iter()
        .enumerate()
    {
        let span_a = (a_end - a_start).max(0.0);
        let sub_steps = ((span_a / step_ang).round() as usize).max(1);
        for i in 1..=sub_steps {
            if k == 2 && i == sub_steps {
                break;
            }
            let t = i as f64 / sub_steps as f64;
            let ang = a_start + t * span_a;
            profile.push([radius * ang.cos(), radius * ang.sin()]);
        }
    }

    let n = profile.len();
    let half_h = height * 0.5;
    let to_world = |px: f64, py: f64, pz: f64| -> [f64; 3] {
        let p = center + u * px + v * py + w * pz;
        [p.x, p.y, p.z]
    };
    let norm_world = |nx: f64, ny: f64, nz: f64| -> [f64; 3] {
        let d = (u * nx + v * ny + w * nz).normalize();
        [d.x, d.y, d.z]
    };

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut face_ids = Vec::new();

    let (cap_pts, cap_tris) = triangulate_cap_polygon_2d(&profile);

    // Bottom cap at z = -half_h (normal -w, entry end -> id = 2 "hole_top")
    let n_bot = norm_world(0.0, 0.0, -1.0);
    let bot_start = vertices.len() as u32;
    for &[px, py] in &cap_pts {
        vertices.push(Vertex::new(to_world(px, py, -half_h), n_bot));
    }
    for &[a, b, c] in &cap_tris {
        indices.push([bot_start + a, bot_start + c, bot_start + b]);
        face_ids.push(2);
    }

    // Top cap at z = +half_h (normal +w, blind bottom end -> id = 1 "hole_bottom")
    let n_top = norm_world(0.0, 0.0, 1.0);
    let top_start = vertices.len() as u32;
    for &[px, py] in &cap_pts {
        vertices.push(Vertex::new(to_world(px, py, half_h), n_top));
    }
    for &[a, b, c] in &cap_tris {
        indices.push([top_start + a, top_start + b, top_start + c]);
        face_ids.push(1);
    }

    // Side walls connecting bottom ring to top ring with axial subdivision matching chord length
    let mut chord_sum = 0.0;
    for i in 0..n {
        let next = (i + 1) % n;
        chord_sum += (profile[next][0] - profile[i][0]).hypot(profile[next][1] - profile[i][1]);
    }
    let avg_chord = (chord_sum / (n as f64)).max(1e-6);
    let n_z = ((height / (avg_chord * 1.25)).ceil() as usize).clamp(1, 64);

    for kz in 0..n_z {
        let z0 = -half_h + (kz as f64) * (height / (n_z as f64));
        let z1 = -half_h + ((kz + 1) as f64) * (height / (n_z as f64));
        for i in 0..n {
            let next = (i + 1) % n;
            let p0 = profile[i];
            let p1 = profile[next];
            let dx = p1[0] - p0[0];
            let dy = p1[1] - p0[1];
            let len = (dx * dx + dy * dy).sqrt().max(1e-12);
            let n_side = norm_world(dy / len, -dx / len, 0.0);

            push_quad_shortest_diag(
                &mut vertices,
                &mut indices,
                to_world(p0[0], p0[1], z0),
                to_world(p1[0], p1[1], z0),
                to_world(p1[0], p1[1], z1),
                to_world(p0[0], p0[1], z1),
                n_side,
            );
            // Segments i in 0..3 form the 3 walls of the rectangular keyway slot (id = 4);
            // segments i >= 3 form the cylindrical bore wall (id = 3).
            let wall_fid = if i < 3 { 4 } else { 3 };
            face_ids.push(wall_fid);
            face_ids.push(wall_fid);
        }
    }

    let hole_axis = [-w.x, -w.y, -w.z];
    let faces = vec![
        FaceInfo::new_descriptor(
            1,
            "hole_bottom",
            vec!["blind_bottom".to_string(), "bottom".to_string()],
            FaceKind::Plane,
            n_top,
            Some(radius),
        ),
        FaceInfo::new_descriptor(
            2,
            "hole_top",
            vec![],
            FaceKind::Plane,
            n_bot,
            Some(radius),
        ),
        FaceInfo::new_descriptor(
            3,
            "hole_wall",
            vec![
                "bore".to_string(),
                "hole".to_string(),
                "inner_wall".to_string(),
                "side".to_string(),
            ],
            FaceKind::Cylinder,
            hole_axis,
            Some(radius),
        ),
        FaceInfo::new_descriptor(
            4,
            "keyway_wall",
            vec!["keyway".to_string(), "slot_wall".to_string()],
            FaceKind::Plane,
            norm_world(0.0, -1.0, 0.0),
            None,
        ),
    ];

    TriangleMesh::with_faces(vertices, indices, face_ids, faces)
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

    let max_r = radius_bottom.max(radius_top);
    let segs = if segments == 0 {
        TessellationSettings::default().calculate_segments(max_r)
    } else {
        segments.max(8)
    };
    let h2 = height * 0.5;

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut face_ids = Vec::new();
    let mut faces = Vec::new();
    let mut next_face_id = 1u32;

    let has_bot_cap = radius_bottom > 1e-12;
    let has_top_cap = radius_top > 1e-12;

    // 1. Bottom cap (z = -h2, normal = [0, 0, -1])
    if has_bot_cap {
        let bot_fid = next_face_id;
        next_face_id += 1;
        let bot_ring: Vec<[f64; 2]> = (0..segs)
            .map(|i| {
                let theta = (i as f64) * 2.0 * PI / (segs as f64);
                [radius_bottom * theta.cos(), radius_bottom * theta.sin()]
            })
            .collect();
        add_subdivided_disk_cap_2d(&mut vertices, &mut indices, &bot_ring, [0.0, 0.0], -h2, false);
        face_ids.resize(indices.len(), bot_fid);
        faces.push(FaceInfo::new_descriptor(
            bot_fid,
            "cap_bottom",
            vec!["bottom".to_string(), "min_z".to_string(), "-z".to_string()],
            FaceKind::Plane,
            [0.0, 0.0, -1.0],
            Some(radius_bottom),
        ));
    }

    // 2. Top cap (z = +h2, normal = [0, 0, 1]) if truncated cone
    if has_top_cap {
        let top_fid = next_face_id;
        next_face_id += 1;
        let top_ring: Vec<[f64; 2]> = (0..segs)
            .map(|i| {
                let theta = (i as f64) * 2.0 * PI / (segs as f64);
                [radius_top * theta.cos(), radius_top * theta.sin()]
            })
            .collect();
        add_subdivided_disk_cap_2d(&mut vertices, &mut indices, &top_ring, [0.0, 0.0], h2, true);
        face_ids.resize(indices.len(), top_fid);
        faces.push(FaceInfo::new_descriptor(
            top_fid,
            "cap_top",
            vec!["top".to_string(), "max_z".to_string(), "+z".to_string()],
            FaceKind::Plane,
            [0.0, 0.0, 1.0],
            Some(radius_top),
        ));
    }

    let side_fid = next_face_id;

    // 3. Side wall with axial subdivision and 2-to-1 halving near sharp cone apexes
    let dr = radius_bottom - radius_top;
    let slant = (height * height + dr * dr).sqrt();
    let (nz, n_radial) = if slant > 1e-12 {
        (dr / slant, height / slant)
    } else {
        (0.0, 1.0)
    };
    let base_chord = (2.0 * max_r * (PI / (segs as f64)).sin()).max(1e-6);
    let n_z = ((slant / (base_chord * 1.25)).ceil() as usize).clamp(4, 48);

    if !has_top_cap {
        // Sharp cone: apex at [0, 0, h2]
        let mut curr_segs = segs;
        let mut prev_ring: Vec<[f64; 3]> = (0..curr_segs)
            .map(|i| {
                let theta = (i as f64) * 2.0 * PI / (curr_segs as f64);
                [radius_bottom * theta.cos(), radius_bottom * theta.sin(), -h2]
            })
            .collect();

        for k in 1..n_z {
            let t = (k as f64) / (n_z as f64);
            let r_curr = radius_bottom * (1.0 - t);
            let z_curr = -h2 + t * height;
            let chord_curr = 2.0 * PI * r_curr / (curr_segs as f64);
            let step_len = slant / (n_z as f64);

            if curr_segs >= 12 && chord_curr < step_len * 0.55 {
                let half_segs = curr_segs / 2;
                let is_odd = curr_segs % 2 != 0;
                let next_ring: Vec<[f64; 3]> = (0..half_segs)
                    .map(|j| {
                        let theta = (j as f64) * 2.0 * PI / (half_segs as f64);
                        [r_curr * theta.cos(), r_curr * theta.sin(), z_curr]
                    })
                    .collect();
                for j in 0..half_segs {
                    let j_next = (j + 1) % half_segs;
                    let i0 = next_ring[j];
                    let i1 = next_ring[j_next];
                    let ang = ((2 * j + 1) as f64) * 2.0 * PI / (curr_segs as f64);
                    let norm = [n_radial * ang.cos(), n_radial * ang.sin(), nz];
                    if is_odd && j == half_segs - 1 {
                        let o0 = prev_ring[2 * j];
                        let o1 = prev_ring[2 * j + 1];
                        let o2 = prev_ring[2 * j + 2];
                        let o3 = prev_ring[0];
                        for (a, b, c) in [(o0, o1, i0), (o1, i1, i0), (o1, o2, i1), (o2, o3, i1)] {
                            let base = vertices.len() as u32;
                            vertices.push(Vertex::new(a, norm));
                            vertices.push(Vertex::new(b, norm));
                            vertices.push(Vertex::new(c, norm));
                            indices.push([base, base + 1, base + 2]);
                        }
                    } else {
                        let o0 = prev_ring[2 * j];
                        let o1 = prev_ring[2 * j + 1];
                        let o2 = prev_ring[(2 * j + 2) % curr_segs];
                        for (a, b, c) in [(o0, o1, i0), (o1, i1, i0), (o1, o2, i1)] {
                            let base = vertices.len() as u32;
                            vertices.push(Vertex::new(a, norm));
                            vertices.push(Vertex::new(b, norm));
                            vertices.push(Vertex::new(c, norm));
                            indices.push([base, base + 1, base + 2]);
                        }
                    }
                }
                curr_segs = half_segs;
                prev_ring = next_ring;
            } else {
                let next_ring: Vec<[f64; 3]> = (0..curr_segs)
                    .map(|i| {
                        let theta = (i as f64) * 2.0 * PI / (curr_segs as f64);
                        [r_curr * theta.cos(), r_curr * theta.sin(), z_curr]
                    })
                    .collect();
                for i in 0..curr_segs {
                    let next = (i + 1) % curr_segs;
                    let ang = ((i as f64) + 0.5) * 2.0 * PI / (curr_segs as f64);
                    let norm = [n_radial * ang.cos(), n_radial * ang.sin(), nz];
                    push_quad_shortest_diag(
                        &mut vertices,
                        &mut indices,
                        prev_ring[i],
                        prev_ring[next],
                        next_ring[next],
                        next_ring[i],
                        norm,
                    );
                }
                prev_ring = next_ring;
            }
        }

        let apex = [0.0, 0.0, h2];
        for i in 0..curr_segs {
            let next = (i + 1) % curr_segs;
            let ang = ((i as f64) + 0.5) * 2.0 * PI / (curr_segs as f64);
            let norm = [n_radial * ang.cos(), n_radial * ang.sin(), nz];
            let base = vertices.len() as u32;
            vertices.push(Vertex::new(prev_ring[i], norm));
            vertices.push(Vertex::new(prev_ring[next], norm));
            vertices.push(Vertex::new(apex, norm));
            indices.push([base, base + 1, base + 2]);
        }
    } else if !has_bot_cap {
        // Inverted sharp cone: apex at [0, 0, -h2]
        let mut curr_segs = segs;
        let mut prev_ring: Vec<[f64; 3]> = (0..curr_segs)
            .map(|i| {
                let theta = (i as f64) * 2.0 * PI / (curr_segs as f64);
                [radius_top * theta.cos(), radius_top * theta.sin(), h2]
            })
            .collect();

        for k in 1..n_z {
            let t = (k as f64) / (n_z as f64);
            let r_curr = radius_top * (1.0 - t);
            let z_curr = h2 - t * height;
            let chord_curr = 2.0 * PI * r_curr / (curr_segs as f64);
            let step_len = slant / (n_z as f64);

            if curr_segs >= 12 && chord_curr < step_len * 0.55 {
                let half_segs = curr_segs / 2;
                let is_odd = curr_segs % 2 != 0;
                let next_ring: Vec<[f64; 3]> = (0..half_segs)
                    .map(|j| {
                        let theta = (j as f64) * 2.0 * PI / (half_segs as f64);
                        [r_curr * theta.cos(), r_curr * theta.sin(), z_curr]
                    })
                    .collect();
                for j in 0..half_segs {
                    let j_next = (j + 1) % half_segs;
                    let i0 = next_ring[j];
                    let i1 = next_ring[j_next];
                    let ang = ((2 * j + 1) as f64) * 2.0 * PI / (curr_segs as f64);
                    let norm = [n_radial * ang.cos(), n_radial * ang.sin(), nz];
                    if is_odd && j == half_segs - 1 {
                        let o0 = prev_ring[2 * j];
                        let o1 = prev_ring[2 * j + 1];
                        let o2 = prev_ring[2 * j + 2];
                        let o3 = prev_ring[0];
                        for (a, b, c) in [(o0, i0, o1), (o1, i0, i1), (o1, i1, o2), (o2, i1, o3)] {
                            let base = vertices.len() as u32;
                            vertices.push(Vertex::new(a, norm));
                            vertices.push(Vertex::new(b, norm));
                            vertices.push(Vertex::new(c, norm));
                            indices.push([base, base + 1, base + 2]);
                        }
                    } else {
                        let o0 = prev_ring[2 * j];
                        let o1 = prev_ring[2 * j + 1];
                        let o2 = prev_ring[(2 * j + 2) % curr_segs];
                        for (a, b, c) in [(o0, i0, o1), (o1, i0, i1), (o1, i1, o2)] {
                            let base = vertices.len() as u32;
                            vertices.push(Vertex::new(a, norm));
                            vertices.push(Vertex::new(b, norm));
                            vertices.push(Vertex::new(c, norm));
                            indices.push([base, base + 1, base + 2]);
                        }
                    }
                }
                curr_segs = half_segs;
                prev_ring = next_ring;
            } else {
                let next_ring: Vec<[f64; 3]> = (0..curr_segs)
                    .map(|i| {
                        let theta = (i as f64) * 2.0 * PI / (curr_segs as f64);
                        [r_curr * theta.cos(), r_curr * theta.sin(), z_curr]
                    })
                    .collect();
                for i in 0..curr_segs {
                    let next = (i + 1) % curr_segs;
                    let ang = ((i as f64) + 0.5) * 2.0 * PI / (curr_segs as f64);
                    let norm = [n_radial * ang.cos(), n_radial * ang.sin(), nz];
                    push_quad_shortest_diag(
                        &mut vertices,
                        &mut indices,
                        prev_ring[i],
                        next_ring[i],
                        next_ring[next],
                        prev_ring[next],
                        norm,
                    );
                }
                prev_ring = next_ring;
            }
        }

        let apex = [0.0, 0.0, -h2];
        for i in 0..curr_segs {
            let next = (i + 1) % curr_segs;
            let ang = ((i as f64) + 0.5) * 2.0 * PI / (curr_segs as f64);
            let norm = [n_radial * ang.cos(), n_radial * ang.sin(), nz];
            let base = vertices.len() as u32;
            vertices.push(Vertex::new(prev_ring[i], norm));
            vertices.push(Vertex::new(apex, norm));
            vertices.push(Vertex::new(prev_ring[next], norm));
            indices.push([base, base + 1, base + 2]);
        }
    } else {
        // Truncated cone (frustum) subdivided into n_z axial bands
        for k in 0..n_z {
            let t0 = (k as f64) / (n_z as f64);
            let t1 = ((k + 1) as f64) / (n_z as f64);
            let r0 = radius_bottom + (radius_top - radius_bottom) * t0;
            let r1 = radius_bottom + (radius_top - radius_bottom) * t1;
            let z0 = -h2 + t0 * height;
            let z1 = -h2 + t1 * height;

            for i in 0..segs {
                let next = (i + 1) % segs;
                let a0 = (i as f64) * 2.0 * PI / (segs as f64);
                let a1 = (next as f64) * 2.0 * PI / (segs as f64);
                let mid_a = ((i as f64) + 0.5) * 2.0 * PI / (segs as f64);
                let norm = [n_radial * mid_a.cos(), n_radial * mid_a.sin(), nz];

                let p_bot_curr = [r0 * a0.cos(), r0 * a0.sin(), z0];
                let p_bot_next = [r0 * a1.cos(), r0 * a1.sin(), z0];
                let p_top_next = [r1 * a1.cos(), r1 * a1.sin(), z1];
                let p_top_curr = [r1 * a0.cos(), r1 * a0.sin(), z1];
                push_quad_shortest_diag(
                    &mut vertices,
                    &mut indices,
                    p_top_curr,
                    p_bot_curr,
                    p_bot_next,
                    p_top_next,
                    norm,
                );
            }
        }
    }

    face_ids.resize(indices.len(), side_fid);
    faces.push(FaceInfo::new_descriptor(
        side_fid,
        "side",
        vec!["conical_wall".to_string(), "outer_wall".to_string()],
        FaceKind::Cone,
        [0.0, 0.0, 1.0],
        Some(0.5 * (radius_bottom + radius_top)),
    ));

    TriangleMesh::with_faces(vertices, indices, face_ids, faces)
}

/// Generates a solid 3D cone or frustum with adaptive tessellation based on `TessellationSettings`.
pub fn create_cone_with_settings(
    radius_bottom: f64,
    radius_top: f64,
    height: f64,
    settings: &TessellationSettings,
) -> TriangleMesh {
    let max_r = radius_bottom.max(radius_top);
    let segments = settings.calculate_segments(max_r);
    create_cone(radius_bottom, radius_top, height, segments)
}

/// Generates a solid regular polygonal prism centered at origin along Z axis.
///
/// Parameters:
/// - `sides`: number of facets (minimum 3, default 6 for standard hexagon)
/// - `height`: prism height along Z axis
/// - `width_across_flats`: optional flat-to-flat distance (S, e.g. wrench size for nuts/bolts)
/// - `radius`: optional circumscribed circle radius (distance from center to vertices)
/// - `chamfer`: optional chamfer size at top and bottom faces (30-degree bevel per DIN 934)
///
/// Features sharp edges, flat facet normals, and 100% watertight manifold topology.
pub fn create_polygon_prism(
    sides: usize,
    height: f64,
    width_across_flats: Option<f64>,
    radius: Option<f64>,
    chamfer: Option<f64>,
) -> TriangleMesh {
    if sides < 3 || height <= 1e-12 {
        return TriangleMesh::empty();
    }
    let n = sides;

    let angle_step = 2.0 * PI / (n as f64);
    let half_angle = PI / (n as f64);

    let circumradius = if let Some(waf) = width_across_flats {
        if waf > 1e-12 {
            waf / (2.0 * half_angle.cos())
        } else if let Some(r) = radius {
            r
        } else {
            return TriangleMesh::empty();
        }
    } else if let Some(r) = radius {
        if r > 1e-12 {
            r
        } else {
            return TriangleMesh::empty();
        }
    } else {
        return TriangleMesh::empty();
    };

    if circumradius <= 1e-12 {
        return TriangleMesh::empty();
    }

    let h2 = height * 0.5;
    let c_val = chamfer.unwrap_or(0.0);
    let has_chamfer = c_val > 1e-12;

    let (r_cap, z_bot_mid, z_top_mid) = if has_chamfer {
        let tan_alpha = (PI / 6.0).tan(); // 30-degree DIN 934 chamfer angle
        let mut c_r = c_val.min(circumradius * 0.45);
        let mut c_z = c_r * tan_alpha;
        if c_z > height * 0.40 {
            c_z = height * 0.40;
            c_r = c_z / tan_alpha;
        }
        let r_c = (circumradius - c_r).max(circumradius * 0.20);
        (r_c, -h2 + c_z, h2 - c_z)
    } else {
        (circumradius, -h2, h2)
    };

    let sin_half = half_angle.sin();
    let cos_half = half_angle.cos();
    let tan_half = sin_half / cos_half;
    let s_mid = 2.0 * circumradius * sin_half;
    let s_cap = 2.0 * r_cap * sin_half;
    let h_mid = (z_top_mid - z_bot_mid).max(1e-12);

    let (m_edge, n_bevel_steps) = if has_chamfer {
        let c_r_eff = circumradius - r_cap;
        let c_z_eff = h2 - z_top_mid;
        let h_bevel = ((c_r_eff * cos_half).powi(2) + c_z_eff.powi(2))
            .sqrt()
            .max(1e-12);
        let min_band_h = h_bevel.min(h_mid).max(1e-12);
        let mut m = 1usize;
        while m < 512 && (s_mid / (m as f64)) > 3.0 * min_band_h {
            m *= 2;
        }
        let s_cap_seg = (s_cap / (m as f64)).max(1e-12);
        let bevel_steps = ((h_bevel / (1.5 * s_cap_seg)).ceil() as usize).clamp(1, 8);
        (m, bevel_steps)
    } else {
        let mut m = 1usize;
        while m < 512 && (s_mid / (m as f64)) > 10.0 * h_mid {
            m *= 2;
        }
        (m, 1usize)
    };

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut face_ids = Vec::new();
    let mut faces = Vec::with_capacity(if has_chamfer { 2 + 3 * n } else { 2 + n });

    // Helper to emit top or bottom planar polygonal cap with 2-to-1 inward halving from m_edge -> 1
    let mut emit_polygon_cap = |is_top: bool, cap_fid: u32| {
        let z_val = if is_top { h2 } else { -h2 };
        let nz = if is_top { 1.0 } else { -1.0 };
        let normal = [0.0, 0.0, nz];

        let mut push_cap_tri = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
            let base = vertices.len() as u32;
            vertices.push(Vertex::new([a[0], a[1], z_val], normal));
            vertices.push(Vertex::new([b[0], b[1], z_val], normal));
            vertices.push(Vertex::new([c[0], c[1], z_val], normal));
            if is_top {
                indices.push([base, base + 1, base + 2]);
            } else {
                indices.push([base, base + 2, base + 1]);
            }
            face_ids.push(cap_fid);
        };

        let mut r_curr = r_cap;
        let mut m_curr = m_edge;

        while m_curr > 1 {
            let k = m_curr / 2;
            let sigma = (0.65 * tan_half).min((0.55 * (k as f64)).min(0.75));
            let eta = (1.0 - sigma / (k as f64)).clamp(0.45, 0.92);
            let r_next = r_curr * eta;

            for i in 0..n {
                let next = (i + 1) % n;
                let t1 = (i as f64) * angle_step;
                let t2 = (next as f64) * angle_step;
                let p_out_0 = [r_curr * t1.cos(), r_curr * t1.sin()];
                let p_out_1 = [r_curr * t2.cos(), r_curr * t2.sin()];
                let p_in_0 = [r_next * t1.cos(), r_next * t1.sin()];
                let p_in_1 = [r_next * t2.cos(), r_next * t2.sin()];

                let lerp2 = |a: [f64; 2], b: [f64; 2], t: f64| -> [f64; 2] {
                    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
                };

                for l in 0..k {
                    let o0 = lerp2(p_out_0, p_out_1, (2 * l) as f64 / (m_curr as f64));
                    let o1 = lerp2(p_out_0, p_out_1, (2 * l + 1) as f64 / (m_curr as f64));
                    let o2 = lerp2(p_out_0, p_out_1, (2 * l + 2) as f64 / (m_curr as f64));
                    let i0 = lerp2(p_in_0, p_in_1, (l as f64) / (k as f64));
                    let i1 = lerp2(p_in_0, p_in_1, ((l + 1) as f64) / (k as f64));

                    push_cap_tri(o0, o1, i0);
                    push_cap_tri(o1, i1, i0);
                    push_cap_tri(o1, o2, i1);
                }
            }

            r_curr = r_next;
            m_curr = k;
        }

        // Central n-gon fan for m_curr == 1
        let center_idx = vertices.len() as u32;
        vertices.push(Vertex::new([0.0, 0.0, z_val], normal));
        let rim_start = vertices.len() as u32;
        for i in 0..n {
            let theta = (i as f64) * angle_step;
            vertices.push(Vertex::new(
                [r_curr * theta.cos(), r_curr * theta.sin(), z_val],
                normal,
            ));
        }
        for i in 0..n {
            let next = (i + 1) % n;
            if is_top {
                indices.push([
                    center_idx,
                    rim_start + (i as u32),
                    rim_start + (next as u32),
                ]);
            } else {
                indices.push([
                    center_idx,
                    rim_start + (next as u32),
                    rim_start + (i as u32),
                ]);
            }
            face_ids.push(cap_fid);
        }
    };

    // 1. Top cap (z = +h2, id = 1) & 2. Bottom cap (z = -h2, id = 2)
    emit_polygon_cap(true, 1);
    faces.push(FaceInfo::new_descriptor(
        1,
        "cap_top",
        vec!["top".to_string(), "max_z".to_string(), "+z".to_string()],
        FaceKind::Plane,
        [0.0, 0.0, 1.0],
        Some(r_cap),
    ));

    emit_polygon_cap(false, 2);
    faces.push(FaceInfo::new_descriptor(
        2,
        "cap_bottom",
        vec!["bottom".to_string(), "min_z".to_string(), "-z".to_string()],
        FaceKind::Plane,
        [0.0, 0.0, -1.0],
        Some(r_cap),
    ));

    // Helper function to add a subdivided planar quad facet (p0_top, p1_bot, p2_bot_next, p3_top_next)
    #[allow(clippy::too_many_arguments)]
    fn add_subdivided_quad_facet(
        vertices: &mut Vec<Vertex>,
        indices: &mut Vec<[u32; 3]>,
        face_ids: &mut Vec<u32>,
        facet_fid: u32,
        p0: [f64; 3],
        p1: [f64; 3],
        p2: [f64; 3],
        p3: [f64; 3],
        m_horiz: usize,
        n_vert: usize,
    ) {
        let v0 = Vector3::new(p0[0], p0[1], p0[2]);
        let v1 = Vector3::new(p1[0], p1[1], p1[2]);
        let v2 = Vector3::new(p2[0], p2[1], p2[2]);
        let v3 = Vector3::new(p3[0], p3[1], p3[2]);
        let n_vec = (v1 - v0).cross(&(v3 - v0)) + (v2 - v1).cross(&(v3 - v1));
        let n_len = n_vec.norm();
        let normal = if n_len > 1e-12 {
            [n_vec.x / n_len, n_vec.y / n_len, n_vec.z / n_len]
        } else {
            [1.0, 0.0, 0.0]
        };

        let lerp3 = |a: [f64; 3], b: [f64; 3], t: f64| -> [f64; 3] {
            [
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            ]
        };

        for rv in 0..n_vert {
            let v_top = (rv as f64) / (n_vert as f64);
            let v_bot = ((rv + 1) as f64) / (n_vert as f64);
            let row_top_l = lerp3(p0, p1, v_top);
            let row_top_r = lerp3(p3, p2, v_top);
            let row_bot_l = lerp3(p0, p1, v_bot);
            let row_bot_r = lerp3(p3, p2, v_bot);

            for cu in 0..m_horiz {
                let u_l = (cu as f64) / (m_horiz as f64);
                let u_r = ((cu + 1) as f64) / (m_horiz as f64);
                let q0 = lerp3(row_top_l, row_top_r, u_l);
                let q1 = lerp3(row_bot_l, row_bot_r, u_l);
                let q2 = lerp3(row_bot_l, row_bot_r, u_r);
                let q3 = lerp3(row_top_l, row_top_r, u_r);
                push_quad_shortest_diag(vertices, indices, q0, q1, q2, q3, normal);
                face_ids.push(facet_fid);
                face_ids.push(facet_fid);
            }
        }
    }

    // 3. Side wall quads (and optional top/bottom chamfer bevel bands)
    for i in 0..n {
        let next = (i + 1) % n;
        let t1 = (i as f64) * angle_step;
        let t2 = (next as f64) * angle_step;
        let c1 = t1.cos();
        let s1 = t1.sin();
        let c2 = t2.cos();
        let s2 = t2.sin();

        let x1_mid = circumradius * c1;
        let y1_mid = circumradius * s1;
        let x2_mid = circumradius * c2;
        let y2_mid = circumradius * s2;

        let side_fid = (3 + i) as u32;
        let mid_theta = ((i as f64) + 0.5) * angle_step;
        let side_normal = [mid_theta.cos(), mid_theta.sin(), 0.0];
        faces.push(FaceInfo::new_descriptor(
            side_fid,
            format!("side_{i}"),
            vec![format!("flat_{i}")],
            FaceKind::Plane,
            side_normal,
            None,
        ));

        if has_chamfer {
            let x1_cap = r_cap * c1;
            let y1_cap = r_cap * s1;
            let x2_cap = r_cap * c2;
            let y2_cap = r_cap * s2;

            let chamfer_top_fid = (3 + n + i) as u32;
            let chamfer_bot_fid = (3 + 2 * n + i) as u32;

            // Top bevel band: from z = h2 (r_cap) down to z = z_top_mid (circumradius)
            add_subdivided_quad_facet(
                &mut vertices,
                &mut indices,
                &mut face_ids,
                chamfer_top_fid,
                [x1_cap, y1_cap, h2],
                [x1_mid, y1_mid, z_top_mid],
                [x2_mid, y2_mid, z_top_mid],
                [x2_cap, y2_cap, h2],
                m_edge,
                n_bevel_steps,
            );
            faces.push(FaceInfo::new_descriptor(
                chamfer_top_fid,
                format!("chamfer_top_{i}"),
                vec![format!("bevel_top_{i}")],
                FaceKind::Plane,
                [0.0, 0.0, 1.0],
                None,
            ));

            // Bottom bevel band: from z = z_bot_mid (circumradius) down to z = -h2 (r_cap)
            add_subdivided_quad_facet(
                &mut vertices,
                &mut indices,
                &mut face_ids,
                chamfer_bot_fid,
                [x1_mid, y1_mid, z_bot_mid],
                [x1_cap, y1_cap, -h2],
                [x2_cap, y2_cap, -h2],
                [x2_mid, y2_mid, z_bot_mid],
                m_edge,
                n_bevel_steps,
            );
            faces.push(FaceInfo::new_descriptor(
                chamfer_bot_fid,
                format!("chamfer_bottom_{i}"),
                vec![format!("bevel_bottom_{i}")],
                FaceKind::Plane,
                [0.0, 0.0, -1.0],
                None,
            ));
        }

        // Middle vertical band: from z = z_top_mid down to z = z_bot_mid, coarsening 2-to-1 from m_edge
        let mut z_top_c = z_top_mid;
        let mut z_bot_c = z_bot_mid;
        let mut m_c = m_edge;

        while m_c > 1 {
            let ds = s_mid / (m_c as f64);
            let h_rem = z_top_c - z_bot_c;
            if h_rem < 1.3 * ds {
                break;
            }
            let dz = (0.55 * ds).min(0.35 * h_rem);
            let k = m_c / 2;
            let z_top_in = z_top_c - dz;
            let z_bot_in = z_bot_c + dz;

            let lerp_xy = |t: f64| -> [f64; 2] {
                [x1_mid + (x2_mid - x1_mid) * t, y1_mid + (y2_mid - y1_mid) * t]
            };

            for l in 0..k {
                let xy_o0 = lerp_xy((2 * l) as f64 / (m_c as f64));
                let xy_o1 = lerp_xy((2 * l + 1) as f64 / (m_c as f64));
                let xy_o2 = lerp_xy((2 * l + 2) as f64 / (m_c as f64));
                let xy_i0 = lerp_xy((l as f64) / (k as f64));
                let xy_i1 = lerp_xy(((l + 1) as f64) / (k as f64));

                // Top vertical halving strip (z_top_c -> z_top_in)
                let base_t = vertices.len() as u32;
                vertices.push(Vertex::new([xy_o0[0], xy_o0[1], z_top_c], side_normal));
                vertices.push(Vertex::new([xy_o1[0], xy_o1[1], z_top_c], side_normal));
                vertices.push(Vertex::new([xy_o2[0], xy_o2[1], z_top_c], side_normal));
                vertices.push(Vertex::new([xy_i0[0], xy_i0[1], z_top_in], side_normal));
                vertices.push(Vertex::new([xy_i1[0], xy_i1[1], z_top_in], side_normal));
                indices.push([base_t, base_t + 3, base_t + 1]);
                indices.push([base_t + 1, base_t + 3, base_t + 4]);
                indices.push([base_t + 1, base_t + 4, base_t + 2]);
                face_ids.extend_from_slice(&[side_fid, side_fid, side_fid]);

                // Bottom vertical halving strip (z_bot_c -> z_bot_in)
                let base_b = vertices.len() as u32;
                vertices.push(Vertex::new([xy_o0[0], xy_o0[1], z_bot_c], side_normal));
                vertices.push(Vertex::new([xy_o1[0], xy_o1[1], z_bot_c], side_normal));
                vertices.push(Vertex::new([xy_o2[0], xy_o2[1], z_bot_c], side_normal));
                vertices.push(Vertex::new([xy_i0[0], xy_i0[1], z_bot_in], side_normal));
                vertices.push(Vertex::new([xy_i1[0], xy_i1[1], z_bot_in], side_normal));
                indices.push([base_b, base_b + 1, base_b + 3]);
                indices.push([base_b + 1, base_b + 4, base_b + 3]);
                indices.push([base_b + 1, base_b + 2, base_b + 4]);
                face_ids.extend_from_slice(&[side_fid, side_fid, side_fid]);
            }

            z_top_c = z_top_in;
            z_bot_c = z_bot_in;
            m_c = k;
        }

        let ds_rem = (s_mid / (m_c as f64)).max(1e-12);
        let h_rem = (z_top_c - z_bot_c).max(0.0);
        let n_z_mid = if !has_chamfer && m_edge == 1 && (h_rem / ds_rem) <= 10.0 {
            1
        } else {
            ((h_rem / (1.5 * ds_rem)).ceil() as usize).max(1)
        };

        add_subdivided_quad_facet(
            &mut vertices,
            &mut indices,
            &mut face_ids,
            side_fid,
            [x1_mid, y1_mid, z_top_c],
            [x1_mid, y1_mid, z_bot_c],
            [x2_mid, y2_mid, z_bot_c],
            [x2_mid, y2_mid, z_top_c],
            m_c,
            n_z_mid,
        );
    }

    TriangleMesh::with_faces(vertices, indices, face_ids, faces)
}

/// Generates a closed, watertight (manifold) 3D mesh for a helical cutting tool
/// (sweeping a circular or 60-degree metric triangular cross section of radius/depth `groove_radius` along a helix).
///
/// Supports multi-start flutes (`starts`), right/left handedness, end caps for clean CSG,
/// and profile_type ("circular" or "metric_60" per ISO 68-1).
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
    profile_type: &str,
) -> TriangleMesh {
    create_helical_cutter_with_settings(
        helix_radius,
        groove_radius,
        pitch,
        height,
        start_z,
        starts,
        is_right_hand,
        turns,
        profile_type,
        &TessellationSettings::default(),
    )
}

/// Generates a helical cutter mesh with adaptive tessellation based on `TessellationSettings`.
#[allow(clippy::too_many_arguments)]
pub fn create_helical_cutter_with_settings(
    helix_radius: f64,
    groove_radius: f64,
    pitch: f64,
    height: f64,
    start_z: f64,
    starts: usize,
    is_right_hand: bool,
    turns: f64,
    profile_type: &str,
    settings: &TessellationSettings,
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
    let cross_segments: usize = match settings.quality {
        TessellationQuality::Coarse => 12,
        TessellationQuality::Medium => 16,
        TessellationQuality::Fine => 24,
        TessellationQuality::Custom { .. } => settings.calculate_segments(groove_radius).clamp(12, 32),
    };
    let steps_per_turn: usize = match settings.quality {
        TessellationQuality::Coarse => 16,
        TessellationQuality::Medium => 24,
        TessellationQuality::Fine => 36,
        TessellationQuality::Custom { .. } => settings.calculate_segments(helix_radius).clamp(16, 64),
    };
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

        let is_metric = profile_type.eq_ignore_ascii_case("metric_60");
        let m = if is_metric { 4 } else { cross_segments };

        // Generate ring vertices at each step j along the helix
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

            if is_metric {
                // ISO 68-1 60-degree triangular metric thread profile (symmetric rhombus cutter
                // supporting both external shaft threads and internal nut threads).
                // Flank angle is 30 deg to radial axis: tan(30) = 1 / sqrt(3).
                let tan30 = (PI / 6.0).tan();
                let w = (groove_radius * tan30).min(0.48 * pitch);
                let h_eff = w / tan30;

                let corners = [
                    (Vector3::new(h_eff, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
                    (
                        Vector3::new(1e-3 * h_eff, w, 0.0),
                        Vector3::new(0.0, 1.0, 0.0),
                    ),
                    (Vector3::new(-h_eff, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0)),
                    (
                        Vector3::new(1e-3 * h_eff, -w, 0.0),
                        Vector3::new(0.0, -1.0, 0.0),
                    ),
                ];
                for (p_c, n_c) in corners {
                    let pos = center + e_r * p_c.x + e_b * p_c.y;
                    let n = (e_r * n_c.x + e_b * n_c.y).normalize();
                    all_vertices.push(Vertex::new([pos.x, pos.y, pos.z], [n.x, n.y, n.z]));
                }
            } else {
                // Half-step angular offset ensures no longitudinal edge lies at cos(alpha) == 0 (r_coord == 0)
                for i in 0..cross_segments {
                    let alpha = ((i as f64) + 0.5) * 2.0 * PI / (cross_segments as f64);
                    let cos_a = alpha.cos();
                    let sin_a = alpha.sin();

                    let n = e_r * cos_a + e_b * sin_a;
                    let pos = center + n * groove_radius;

                    all_vertices.push(Vertex::new([pos.x, pos.y, pos.z], [n.x, n.y, n.z]));
                }
            }
        }

        // Quads along the helical tube
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

    let face_ids = vec![1; all_indices.len()];
    let faces = vec![FaceInfo::new_descriptor(
        1,
        "helical_groove",
        vec![
            "thread".to_string(),
            "flute".to_string(),
            "groove".to_string(),
        ],
        FaceKind::Helical,
        [0.0, 0.0, 1.0],
        Some(helix_radius),
    )];
    TriangleMesh::with_faces(all_vertices, all_indices, face_ids, faces)
}

/// Generates a solid 3D parametric spur gear centered at origin along the Z axis.
///
/// Features:
/// - True involute flank profile according to DIN 867 / ISO 53 standards.
/// - Module `module` ($m$), tooth count `teeth` ($z$).
/// - Pitch diameter $d = m \cdot z$, pitch radius $r = m \cdot z / 2$.
/// - Tip (addendum) radius $r_a = r + m$, root (dedendum) radius $r_f = r - 1.25 \cdot m$.
/// - Base radius $r_b = r \cdot \cos(\alpha)$ with pressure angle $\alpha$ (default 20 deg).
/// - Radial dedendum flank connecting smoothly below base circle if $r_f < r_b$.
/// - Face width `face_width` ($b$) along Z axis.
/// - Optional cylindrical mounting hub (`hub_diameter`, `hub_length`).
/// - Optional central shaft bore hole (`bore_radius`).
/// - Optional DIN 6885 parallel keyway slot (`keyway_width`, `keyway_depth`).
#[allow(clippy::too_many_arguments)]
pub fn create_spur_gear(
    module: f64,
    teeth: usize,
    pressure_angle_deg: f64,
    face_width: f64,
    bore_radius: Option<f64>,
    keyway_width: Option<f64>,
    keyway_depth: Option<f64>,
    hub_diameter: Option<f64>,
    hub_length: Option<f64>,
) -> TriangleMesh {
    create_spur_gear_with_settings(
        module,
        teeth,
        pressure_angle_deg,
        face_width,
        bore_radius,
        keyway_width,
        keyway_depth,
        hub_diameter,
        hub_length,
        &TessellationSettings::default(),
    )
}

/// Generates a solid 3D parametric spur gear with adaptive tessellation based on `TessellationSettings`.
#[allow(clippy::too_many_arguments)]
pub fn create_spur_gear_with_settings(
    module: f64,
    teeth: usize,
    pressure_angle_deg: f64,
    face_width: f64,
    bore_radius: Option<f64>,
    keyway_width: Option<f64>,
    keyway_depth: Option<f64>,
    hub_diameter: Option<f64>,
    hub_length: Option<f64>,
    settings: &TessellationSettings,
) -> TriangleMesh {
    if module <= 1e-9 || teeth < 3 || face_width <= 1e-9 {
        return TriangleMesh::empty();
    }

    let alpha_deg = if pressure_angle_deg > 0.0 && pressure_angle_deg < 89.0 {
        pressure_angle_deg
    } else {
        20.0
    };
    let alpha = alpha_deg * PI / 180.0;

    let z = teeth as f64;
    let r = module * z * 0.5;
    let r_a = r + module;
    let r_f = (r - 1.25 * module).max(0.1 * module);
    let r_b = r * alpha.cos();
    let h2 = face_width * 0.5;

    // Pitch circle tooth thickness half-angle: psi = pi / (2 * z)
    let psi = PI / (2.0 * z);
    let trough_half_angle = PI / z;
    // Involute function at pressure angle alpha: inv(alpha) = tan(alpha) - alpha
    let inv_alpha = alpha.tan() - alpha;

    // Angle theta from tooth centerline at radius R >= r_b
    let min_tip_half_angle = (trough_half_angle * 0.15).max(1e-4);
    let theta_at_r = |radius: f64| -> f64 {
        let raw = if radius < r_b {
            psi + inv_alpha
        } else {
            let cos_ar = (r_b / radius).clamp(0.0, 1.0);
            let alpha_r = cos_ar.acos();
            let inv_alpha_r = alpha_r.tan() - alpha_r;
            (psi + inv_alpha - inv_alpha_r).max(min_tip_half_angle)
        };
        raw.min(trough_half_angle * 0.70)
    };

    let theta_base = theta_at_r(r_b);
    // Root fillet angle widens slightly past theta_base when r_f < r_b so that polar angle
    // increases strictly monotonically and radial steps never create collinear cap triangles.
    let theta_root = if r_f < r_b - 1e-9 {
        (theta_base + 0.25 * (trough_half_angle - theta_base).max(1e-4)).min(trough_half_angle * 0.75)
    } else {
        theta_at_r(r_f)
    };

    // 1. Build structured per-tooth levels and root-circle ring
    let n_flank_steps = match settings.quality {
        TessellationQuality::Coarse => 2,
        TessellationQuality::Medium => 4,
        TessellationQuality::Fine => 8,
        TessellationQuality::Custom { .. } => (settings.calculate_segments(r_a) / teeth).clamp(2, 16),
    };
    let r_inv_start = r_b.max(r_f);

    // Build radial levels for each tooth from level 0 (r_f) up to level M (r_a)
    let mut radial_levels: Vec<(f64, f64)> = Vec::new(); // (radius, half_angle_from_tooth_center)
    radial_levels.push((r_f, theta_root));
    let step_r = (r_a - r_inv_start) / (n_flank_steps as f64);
    if r_f < r_b - 1e-9 && (r_b - r_f) >= 0.35 * step_r {
        radial_levels.push((r_b, theta_base));
    }
    for s_idx in 1..=n_flank_steps {
        let s = (s_idx as f64) / (n_flank_steps as f64);
        let r_curr = r_inv_start + s * (r_a - r_inv_start);
        radial_levels.push((r_curr, theta_at_r(r_curr)));
    }
    let m_levels = radial_levels.len() - 1; // index of top level at r_a

    // Root circle ring has L_{k,0} and R_{k,0} per tooth, plus trough midpoint T_k when trough arc is wide
    let include_trough_mid = (trough_half_angle - theta_root) > 0.12;
    let mut root_angles: Vec<f64> = Vec::with_capacity(teeth * 3 + 4);
    for k in 0..teeth {
        let phi_k = (k as f64) * 2.0 * PI / z;
        root_angles.push((phi_k - theta_root).rem_euclid(2.0 * PI));
        root_angles.push((phi_k + theta_root).rem_euclid(2.0 * PI));
        if include_trough_mid {
            root_angles.push((phi_k + trough_half_angle).rem_euclid(2.0 * PI));
        }
    }
    root_angles.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    // Determine optional bore, keyway, and hub parameters
    let bore_r = bore_radius.unwrap_or(0.0);
    let has_bore = bore_r > 1e-6 && bore_r < r_f * 0.96;

    let kw_w = keyway_width.unwrap_or(0.0);
    let kw_d = keyway_depth.unwrap_or(0.0);
    let has_keyway = has_bore && kw_w > 1e-6 && kw_d > 1e-6 && (bore_r + kw_d) < r_f * 0.96;

    let hub_r = hub_diameter.map(|d| d * 0.5).unwrap_or(0.0);
    let hub_l = hub_length.unwrap_or(0.0);
    let max_inner_r = if has_keyway {
        ((bore_r + kw_d).hypot(kw_w * 0.5)).max(bore_r)
    } else {
        bore_r
    };
    let has_hub = hub_r > 1e-6
        && hub_l > 1e-6
        && hub_r < r_f * 0.98
        && (!has_bore || hub_r > max_inner_r * 1.02);

    let total_hub_len = if has_hub {
        if hub_l > face_width {
            hub_l
        } else {
            face_width + hub_l
        }
    } else {
        face_width
    };
    let z_hub_top = -h2 + total_hub_len;

    // If a keyway is present, ensure exact keyway corner angles are in root_angles
    let (kw_half, kw_y_top, kw_corners) = if has_keyway {
        let w_half = (kw_w * 0.5).min(bore_r * 0.85);
        let y_shoulder = (bore_r * bore_r - w_half * w_half).max(0.0).sqrt();
        let y_top = bore_r + kw_d;
        let c1 = y_shoulder.atan2(w_half).rem_euclid(2.0 * PI);
        let c2 = y_top.atan2(w_half).rem_euclid(2.0 * PI);
        let c3 = y_top.atan2(-w_half).rem_euclid(2.0 * PI);
        let c4 = y_shoulder.atan2(-w_half).rem_euclid(2.0 * PI);
        (w_half, y_top, Some([c1, c2, c3, c4]))
    } else {
        (0.0, 0.0, None)
    };

    if let Some(corners) = kw_corners {
        for &c_ang in &corners {
            let exists = root_angles.iter().any(|&a| (a - c_ang).abs() < 1e-4);
            if !exists {
                root_angles.push(c_ang);
            }
        }
        root_angles.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    }

    // Helper to find index of angle in root_angles
    let find_root_angle_idx = |target_ang: f64| -> usize {
        let norm = target_ang.rem_euclid(2.0 * PI);
        let mut best_i = 0;
        let mut best_d = f64::INFINITY;
        for (i, &a) in root_angles.iter().enumerate() {
            let diff = (a - norm)
                .abs()
                .min((a - norm + 2.0 * PI).abs())
                .min((a - norm - 2.0 * PI).abs());
            if diff < best_d {
                best_d = diff;
                best_i = i;
            }
        }
        best_i
    };

    let root_poly: Vec<[f64; 2]> = root_angles
        .iter()
        .map(|&a| [r_f * a.cos(), r_f * a.sin()])
        .collect();

    // Build inner bore/keyway ring and hub ring matching root_angles
    let inner_poly: Vec<[f64; 2]> = if has_bore {
        root_angles
            .iter()
            .map(|&ang| {
                if let Some([c1, c2, c3, c4]) = kw_corners {
                    if ang >= c1 - 1e-4 && ang <= c2 + 1e-4 {
                        let r_in = kw_half / ang.cos();
                        [r_in * ang.cos(), r_in * ang.sin()]
                    } else if ang > c2 + 1e-4 && ang < c3 - 1e-4 {
                        let r_in = kw_y_top / ang.sin();
                        [r_in * ang.cos(), r_in * ang.sin()]
                    } else if ang >= c3 - 1e-4 && ang <= c4 + 1e-4 {
                        let r_in = -kw_half / ang.cos();
                        [r_in * ang.cos(), r_in * ang.sin()]
                    } else {
                        [bore_r * ang.cos(), bore_r * ang.sin()]
                    }
                } else {
                    [bore_r * ang.cos(), bore_r * ang.sin()]
                }
            })
            .collect()
    } else {
        Vec::new()
    };

    let hub_poly: Vec<[f64; 2]> = if has_hub {
        root_angles
            .iter()
            .map(|&ang| [hub_r * ang.cos(), hub_r * ang.sin()])
            .collect()
    } else {
        Vec::new()
    };

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Helper: add planar subdivided disk cap around [0, 0, z]
    let add_fan_cap = |verts: &mut Vec<Vertex>,
                       idxs: &mut Vec<[u32; 3]>,
                       ring: &[[f64; 2]],
                       z_val: f64,
                       is_top: bool| {
        add_subdivided_disk_cap_2d(verts, idxs, ring, [0.0, 0.0], z_val, is_top);
    };

    // Helper: add planar bridged annular cap between inner_ring and outer_ring at z_val
    let add_annulus_cap = |verts: &mut Vec<Vertex>,
                           idxs: &mut Vec<[u32; 3]>,
                           in_ring: &[[f64; 2]],
                           out_ring: &[[f64; 2]],
                           z_val: f64,
                           is_top: bool| {
        add_bridged_ring_strip_2d(verts, idxs, out_ring, in_ring, z_val, is_top);
    };

    // Helper: add tooth-ladder caps at z_val for all teeth, stitched to root_poly
    let add_teeth_caps = |verts: &mut Vec<Vertex>,
                          idxs: &mut Vec<[u32; 3]>,
                          z_val: f64,
                          is_top: bool| {
        let n_z = if is_top { 1.0 } else { -1.0 };
        let mut push_tri = |a: u32, b: u32, c: u32| {
            if is_top {
                idxs.push([a, b, c]);
            } else {
                idxs.push([a, c, b]);
            }
        };

        let n_r = root_poly.len();
        for k in 0..teeth {
            let phi_k = (k as f64) * 2.0 * PI / z;
            let idx_l = find_root_angle_idx(phi_k - theta_root);
            let idx_r = find_root_angle_idx(phi_k + theta_root);

            // Create level >= 1 flank vertices for tooth k
            let mut l_idxs: Vec<u32> = Vec::with_capacity(m_levels + 1);
            let mut r_idxs: Vec<u32> = Vec::with_capacity(m_levels + 1);
            l_idxs.push(0); // placeholder for level 0
            r_idxs.push(0);

            for &(r_lvl, th_lvl) in radial_levels.iter().skip(1) {
                let p_l = [r_lvl * (phi_k - th_lvl).cos(), r_lvl * (phi_k - th_lvl).sin()];
                let p_r = [r_lvl * (phi_k + th_lvl).cos(), r_lvl * (phi_k + th_lvl).sin()];
                let il = verts.len() as u32;
                verts.push(Vertex::new([p_l[0], p_l[1], z_val], [0.0, 0.0, n_z]));
                let ir = verts.len() as u32;
                verts.push(Vertex::new([p_r[0], p_r[1], z_val], [0.0, 0.0, n_z]));
                l_idxs.push(il);
                r_idxs.push(ir);
            }

            // Connect root vertices between idx_l and idx_r to level 1 (l_idxs[1], r_idxs[1])
            let steps_lr = (idx_r + n_r - idx_l) % n_r;
            let mut base_verts: Vec<u32> = Vec::with_capacity(steps_lr + 1);
            for s in 0..=steps_lr {
                let rp = root_poly[(idx_l + s) % n_r];
                let vi = verts.len() as u32;
                verts.push(Vertex::new([rp[0], rp[1], z_val], [0.0, 0.0, n_z]));
                base_verts.push(vi);
            }
            if steps_lr == 1 {
                push_tri(base_verts[0], l_idxs[1], r_idxs[1]);
                push_tri(base_verts[0], r_idxs[1], base_verts[1]);
            } else {
                let mid = steps_lr / 2;
                for s in 0..mid {
                    push_tri(base_verts[s], l_idxs[1], base_verts[s + 1]);
                }
                push_tri(base_verts[mid], l_idxs[1], r_idxs[1]);
                for s in mid..steps_lr {
                    push_tri(base_verts[s], r_idxs[1], base_verts[s + 1]);
                }
            }

            // Quad ladder across tooth for levels 1..(m_levels - 1)
            for j in 1..(m_levels - 1) {
                push_tri(l_idxs[j], l_idxs[j + 1], r_idxs[j + 1]);
                push_tri(l_idxs[j], r_idxs[j + 1], r_idxs[j]);
            }

            // Top band (m_levels - 1) -> m_levels including tip crest point Tip_k at r_a
            // Connecting Tip_k to level (m_levels - 1) avoids 3 nearly-collinear points on r_a!
            let p_tip = [r_a * phi_k.cos(), r_a * phi_k.sin()];
            let tip_idx = verts.len() as u32;
            verts.push(Vertex::new([p_tip[0], p_tip[1], z_val], [0.0, 0.0, n_z]));
            let prev_lvl = m_levels - 1;
            push_tri(l_idxs[prev_lvl], l_idxs[m_levels], tip_idx);
            push_tri(l_idxs[prev_lvl], tip_idx, r_idxs[prev_lvl]);
            push_tri(r_idxs[prev_lvl], tip_idx, r_idxs[m_levels]);
        }
    };

    // Build outer gear perimeter polygon matching the exact cap boundary edges
    let mut outer_perimeter: Vec<[f64; 2]> = Vec::new();
    let n_r = root_poly.len();
    for k in 0..teeth {
        let phi_k = (k as f64) * 2.0 * PI / z;
        let idx_l = find_root_angle_idx(phi_k - theta_root);
        let idx_r = find_root_angle_idx(phi_k + theta_root);
        let next_k = (k + 1) % teeth;
        let phi_next = (next_k as f64) * 2.0 * PI / z;
        let idx_next_l = find_root_angle_idx(phi_next - theta_root);

        // Leading root corner L_{k,0}
        outer_perimeter.push(root_poly[idx_l]);
        // Leading flank levels 1..=m_levels
        for &(r_lvl, th_lvl) in radial_levels.iter().skip(1) {
            outer_perimeter.push([r_lvl * (phi_k - th_lvl).cos(), r_lvl * (phi_k - th_lvl).sin()]);
        }
        // Tip crest center
        outer_perimeter.push([r_a * phi_k.cos(), r_a * phi_k.sin()]);
        // Trailing flank levels m_levels..=1
        for &(r_lvl, th_lvl) in radial_levels.iter().skip(1).rev() {
            outer_perimeter.push([r_lvl * (phi_k + th_lvl).cos(), r_lvl * (phi_k + th_lvl).sin()]);
        }
        // Trailing root corner R_{k,0} and all root trough vertices up to (excluding) L_{k+1,0}
        let trough_steps = (idx_next_l + n_r - idx_r) % n_r;
        for s in 0..trough_steps {
            outer_perimeter.push(root_poly[(idx_r + s) % n_r]);
        }
    }

    // Helper: add vertical side wall between z_bot and z_top with axial subdivision
    let add_vertical_wall = |verts: &mut Vec<Vertex>,
                             idxs: &mut Vec<[u32; 3]>,
                             ring: &[[f64; 2]],
                             z_bot: f64,
                             z_top: f64,
                             is_outer: bool| {
        let n = ring.len();
        if n < 3 {
            return;
        }
        let mut chord_sum = 0.0;
        for i in 0..n {
            let next = (i + 1) % n;
            chord_sum += (ring[next][0] - ring[i][0]).hypot(ring[next][1] - ring[i][1]);
        }
        let avg_chord = (chord_sum / (n as f64)).max(1e-6);
        let dz_span = (z_top - z_bot).abs();
        let n_z = ((dz_span / (avg_chord * 1.4)).ceil() as usize).clamp(1, 24);

        for kz in 0..n_z {
            let z0 = z_bot + (kz as f64) * (z_top - z_bot) / (n_z as f64);
            let z1 = z_bot + ((kz + 1) as f64) * (z_top - z_bot) / (n_z as f64);
            for i in 0..n {
                let next = (i + 1) % n;
                let p1 = ring[i];
                let p2 = ring[next];
                let dx = p2[0] - p1[0];
                let dy = p2[1] - p1[1];
                let edge_len = dx.hypot(dy);
                let sign = if is_outer { 1.0 } else { -1.0 };
                let normal = if edge_len > 1e-12 {
                    [sign * dy / edge_len, -sign * dx / edge_len, 0.0]
                } else {
                    [sign, 0.0, 0.0]
                };

                if is_outer {
                    push_quad_shortest_diag(
                        verts,
                        idxs,
                        [p1[0], p1[1], z1],
                        [p1[0], p1[1], z0],
                        [p2[0], p2[1], z0],
                        [p2[0], p2[1], z1],
                        normal,
                    );
                } else {
                    push_quad_shortest_diag(
                        verts,
                        idxs,
                        [p1[0], p1[1], z1],
                        [p2[0], p2[1], z1],
                        [p2[0], p2[1], z0],
                        [p1[0], p1[1], z0],
                        normal,
                    );
                }
            }
        }
    };

    // Outer gear tooth wall from -h2 to +h2
    add_vertical_wall(&mut vertices, &mut indices, &outer_perimeter, -h2, h2, true);
    // Top and bottom tooth caps (outside r_f)
    add_teeth_caps(&mut vertices, &mut indices, h2, true);
    add_teeth_caps(&mut vertices, &mut indices, -h2, false);

    // Inner disk/annulus inside r_f
    match (has_hub, has_bore) {
        (false, false) => {
            add_fan_cap(&mut vertices, &mut indices, &root_poly, h2, true);
            add_fan_cap(&mut vertices, &mut indices, &root_poly, -h2, false);
        }
        (true, false) => {
            add_annulus_cap(&mut vertices, &mut indices, &hub_poly, &root_poly, h2, true);
            add_vertical_wall(&mut vertices, &mut indices, &hub_poly, h2, z_hub_top, true);
            add_fan_cap(&mut vertices, &mut indices, &hub_poly, z_hub_top, true);
            add_fan_cap(&mut vertices, &mut indices, &root_poly, -h2, false);
        }
        (false, true) => {
            add_annulus_cap(&mut vertices, &mut indices, &inner_poly, &root_poly, h2, true);
            add_annulus_cap(&mut vertices, &mut indices, &inner_poly, &root_poly, -h2, false);
            add_vertical_wall(&mut vertices, &mut indices, &inner_poly, -h2, h2, false);
        }
        (true, true) => {
            add_annulus_cap(&mut vertices, &mut indices, &hub_poly, &root_poly, h2, true);
            add_vertical_wall(&mut vertices, &mut indices, &hub_poly, h2, z_hub_top, true);
            add_annulus_cap(&mut vertices, &mut indices, &inner_poly, &hub_poly, z_hub_top, true);
            add_annulus_cap(&mut vertices, &mut indices, &inner_poly, &root_poly, -h2, false);
            add_vertical_wall(&mut vertices, &mut indices, &inner_poly, -h2, z_hub_top, false);
        }
    }

    let mut mesh = TriangleMesh::new(vertices, indices);
    mesh.face_ids = vec![0; mesh.indices.len()];
    mesh
}

#[inline]
fn cross_2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

#[inline]
fn min_triangle_angle_2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    let ab_sq = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
    let bc_sq = (c[0] - b[0]).powi(2) + (c[1] - b[1]).powi(2);
    let ca_sq = (a[0] - c[0]).powi(2) + (a[1] - c[1]).powi(2);
    let ab = ab_sq.sqrt();
    let bc = bc_sq.sqrt();
    let ca = ca_sq.sqrt();
    if ab <= 1e-14 || bc <= 1e-14 || ca <= 1e-14 {
        return 0.0;
    }
    let cos_a = ((ab_sq + ca_sq - bc_sq) / (2.0 * ab * ca)).clamp(-1.0, 1.0);
    let cos_b = ((ab_sq + bc_sq - ca_sq) / (2.0 * ab * bc)).clamp(-1.0, 1.0);
    let cos_c = ((bc_sq + ca_sq - ab_sq) / (2.0 * bc * ca)).clamp(-1.0, 1.0);
    cos_a.max(cos_b).max(cos_c).acos()
}

/// Performs iterative 2D Lawson Delaunay diagonal flips on adjacent triangles
/// using the exact Delaunay incircle cotangent criterion (`cot(alpha_w) + cot(alpha_x) < 0`)
/// and minimum interior angle maximization.
fn optimize_2d_delaunay_edges(tris: &mut [[u32; 3]], pts: &[[f64; 2]]) {
    let n_tris = tris.len();
    if n_tris < 2 {
        return;
    }

    for _pass in 0..64 {
        let mut flipped_in_pass = 0usize;
        let mut half_edge_map: std::collections::HashMap<(u32, u32), (usize, usize)> =
            std::collections::HashMap::with_capacity(n_tris * 3);
        let mut edge_count: std::collections::HashMap<(u32, u32), u8> =
            std::collections::HashMap::with_capacity(n_tris * 2);

        for (t_idx, t) in tris.iter().enumerate() {
            for e in 0..3 {
                let u = t[e];
                let v = t[(e + 1) % 3];
                half_edge_map.insert((u, v), (t_idx, e));
                let key = if u < v { (u, v) } else { (v, u) };
                *edge_count.entry(key).or_insert(0) += 1;
            }
        }

        let mut locked = vec![false; n_tris];
        for t1_idx in 0..n_tris {
            if locked[t1_idx] {
                continue;
            }
            let t1 = tris[t1_idx];
            for e1 in 0..3 {
                let u = t1[e1];
                let v = t1[(e1 + 1) % 3];
                let w = t1[(e1 + 2) % 3];
                let key = if u < v { (u, v) } else { (v, u) };
                if edge_count.get(&key).copied().unwrap_or(0) != 2 {
                    continue;
                }
                let Some(&(t2_idx, e2)) = half_edge_map.get(&(v, u)) else {
                    continue;
                };
                if t2_idx <= t1_idx || locked[t2_idx] {
                    continue;
                }
                let t2 = tris[t2_idx];
                let x = t2[(e2 + 2) % 3];
                if x == w || x == u || x == v {
                    continue;
                }
                let new_key = if w < x { (w, x) } else { (x, w) };
                if edge_count.contains_key(&new_key) {
                    continue;
                }

                let pu = pts[u as usize];
                let pv = pts[v as usize];
                let pw = pts[w as usize];
                let px = pts[x as usize];

                let a_new1 = cross_2d(pw, px, pu);
                let a_new2 = cross_2d(px, pw, pv);
                let scale_sq = (pu[0] - pv[0]).powi(2)
                    + (pu[1] - pv[1]).powi(2)
                    + (pw[0] - px[0]).powi(2)
                    + (pw[1] - px[1]).powi(2);
                if a_new1 <= 1e-10 * scale_sq || a_new2 <= 1e-10 * scale_sq {
                    continue;
                }

                // Exact 2D Delaunay incircle condition: alpha_w + alpha_x > pi <=> cot(alpha_w) + cot(alpha_x) < 0
                let area_w = cross_2d(pw, pu, pv);
                let area_x = cross_2d(px, pv, pu);
                let dot_w = (pu[0] - pw[0]) * (pv[0] - pw[0]) + (pu[1] - pw[1]) * (pv[1] - pw[1]);
                let dot_x = (pv[0] - px[0]) * (pu[0] - px[0]) + (pv[1] - px[1]) * (pu[1] - px[1]);
                let delaunay_flip = if area_w > 1e-14 * scale_sq && area_x > 1e-14 * scale_sq {
                    (dot_w / area_w) + (dot_x / area_x) < -1e-6
                } else {
                    false
                };

                let min_before =
                    min_triangle_angle_2d(pu, pv, pw).min(min_triangle_angle_2d(pv, pu, px));
                let min_after =
                    min_triangle_angle_2d(pw, px, pu).min(min_triangle_angle_2d(px, pw, pv));

                if (delaunay_flip && min_after >= min_before - 1e-6)
                    || min_after > min_before + 1e-5
                {
                    tris[t1_idx] = [w, x, u];
                    tris[t2_idx] = [x, w, v];
                    locked[t1_idx] = true;
                    locked[t2_idx] = true;
                    flipped_in_pass += 1;
                    break;
                }
            }
        }

        if flipped_in_pass == 0 {
            break;
        }
    }
}

/// Helper: 2D polygon quality ear-clipping triangulation + Lawson Delaunay optimization for simple polygons.
/// Returns a list of triangle index triples indexing into `pts`.
fn triangulate_simple_polygon_2d(pts: &[[f64; 2]]) -> Vec<[u32; 3]> {
    let n = pts.len();
    if n < 3 {
        return Vec::new();
    }
    if n == 3 {
        return vec![[0, 1, 2]];
    }

    let mut indices: Vec<u32> = (0..n as u32).collect();
    let mut triangles = Vec::with_capacity(n - 2);

    let is_ear = |prev: usize, curr: usize, next: usize, idxs: &[u32], area_tol: f64| -> Option<f64> {
        let u = idxs[prev];
        let v = idxs[curr];
        let w = idxs[next];
        let a = pts[u as usize];
        let b = pts[v as usize];
        let c = pts[w as usize];

        let ab_sq = (b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2);
        let bc_sq = (c[0] - b[0]).powi(2) + (c[1] - b[1]).powi(2);
        let ca_sq = (a[0] - c[0]).powi(2) + (a[1] - c[1]).powi(2);
        let perim_sq = ab_sq + bc_sq + ca_sq;
        if perim_sq <= 1e-24 || ca_sq <= 1e-24 {
            return None;
        }

        let area2 = cross_2d(a, b, c);
        if area2 <= area_tol * perim_sq {
            return None;
        }

        let chord = [c[0] - a[0], c[1] - a[1]];
        for (i, &idx) in idxs.iter().enumerate() {
            if i == prev || i == curr || i == next || idx == u || idx == v || idx == w {
                continue;
            }
            let p = pts[idx as usize];
            // Scale-invariant barycentric inclusion check
            let lc = cross_2d(a, b, p) / area2;
            let la = cross_2d(b, c, p) / area2;
            let lb = cross_2d(c, a, p) / area2;
            if la >= -1e-7 && lb >= -1e-7 && lc >= -1e-7 {
                return None;
            }
            // Reject if `p` lies on the open chord `(a, c)` (prevents bypassing collinear vertices)
            let ap = [p[0] - a[0], p[1] - a[1]];
            let t = (ap[0] * chord[0] + ap[1] * chord[1]) / ca_sq;
            if t > -1e-5 && t < 1.0 + 1e-5 {
                let tc = t.clamp(0.0, 1.0);
                let dx = ap[0] - tc * chord[0];
                let dy = ap[1] - tc * chord[1];
                if dx * dx + dy * dy <= 1e-12 * ca_sq {
                    return None;
                }
            }
        }

        Some(min_triangle_angle_2d(a, b, c))
    };

    while indices.len() > 3 {
        let len = indices.len();
        let mut best_i: Option<usize> = None;
        let mut best_angle = -1.0_f64;

        for &tol in &[1e-9, 1e-13] {
            for i in 0..len {
                let prev = (i + len - 1) % len;
                let next = (i + 1) % len;
                if let Some(min_ang) = is_ear(prev, i, next, &indices, tol) {
                    if min_ang > best_angle {
                        best_angle = min_ang;
                        best_i = Some(i);
                    }
                }
            }
            if best_i.is_some() {
                break;
            }
        }

        if let Some(i) = best_i {
            let prev = (i + len - 1) % len;
            let next = (i + 1) % len;
            triangles.push([indices[prev], indices[i], indices[next]]);
            indices.remove(i);
        } else {
            // Fallback: pick strictly positive-area ear with largest normalized area
            let mut fallback_i = 0usize;
            let mut best_norm_area = f64::NEG_INFINITY;
            for i in 0..len {
                let prev = (i + len - 1) % len;
                let next = (i + 1) % len;
                let a = pts[indices[prev] as usize];
                let b = pts[indices[i] as usize];
                let c = pts[indices[next] as usize];
                let perim_sq = ((b[0] - a[0]).powi(2)
                    + (b[1] - a[1]).powi(2)
                    + (c[0] - b[0]).powi(2)
                    + (c[1] - b[1]).powi(2)
                    + (a[0] - c[0]).powi(2)
                    + (a[1] - c[1]).powi(2))
                .max(1e-24);
                let norm_area = cross_2d(a, b, c) / perim_sq;
                if norm_area > best_norm_area {
                    best_norm_area = norm_area;
                    fallback_i = i;
                }
            }
            let prev = (fallback_i + len - 1) % len;
            let next = (fallback_i + 1) % len;
            triangles.push([indices[prev], indices[fallback_i], indices[next]]);
            indices.remove(fallback_i);
        }
    }
    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }

    optimize_2d_delaunay_edges(&mut triangles, pts);
    triangles
}

/// Triangulates a planar 2D cap polygon `boundary_pts` using quality ear-clipping + Lawson Delaunay flips,
/// and if any triangle still has minimum angle < 5.5°, inserts Steiner points via Bowyer-Watson
/// star-shaped circumcircle cavities (bypassing collinear boundary locks).
fn triangulate_cap_polygon_2d(boundary_pts: &[[f64; 2]]) -> (Vec<[f64; 2]>, Vec<[u32; 3]>) {
    let mut cap_pts = boundary_pts.to_vec();
    let mut triangles = triangulate_simple_polygon_2d(&cap_pts);
    if triangles.len() < 2 {
        return (cap_pts, triangles);
    }

    let five_deg_rad = 5.0_f64.to_radians();
    let target_min_rad = 6.0_f64.to_radians();

    let eval_quality = |pts: &[[f64; 2]], tris: &[[u32; 3]]| -> (usize, f64) {
        let mut lt_5_count = 0usize;
        let mut min_ang = f64::INFINITY;
        for t in tris {
            let a = min_triangle_angle_2d(
                pts[t[0] as usize],
                pts[t[1] as usize],
                pts[t[2] as usize],
            );
            if a < five_deg_rad {
                lt_5_count += 1;
            }
            if a < min_ang {
                min_ang = a;
            }
        }
        (lt_5_count, min_ang)
    };

    let (mut best_lt5, mut best_min_ang) = eval_quality(&cap_pts, &triangles);
    if best_lt5 == 0 && best_min_ang >= target_min_rad {
        return (cap_pts, triangles);
    }

    let in_circumcircle = |p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]| -> bool {
        let ax = a[0] - p[0];
        let ay = a[1] - p[1];
        let bx = b[0] - p[0];
        let by = b[1] - p[1];
        let cx = c[0] - p[0];
        let cy = c[1] - p[1];
        let a2 = ax * ax + ay * ay;
        let b2 = bx * bx + by * by;
        let c2 = cx * cx + cy * cy;
        let det = ax * (by * c2 - cy * b2) - ay * (bx * c2 - cx * b2) + a2 * (bx * cy - by * cx);
        det > 1e-18 * (a2 + b2 + c2).powi(2)
    };

    let mut best_pts = cap_pts.clone();
    let mut best_tris = triangles.clone();

    for _iter in 0..48 {
        let (curr_lt5, curr_min_ang) = eval_quality(&cap_pts, &triangles);
        if curr_lt5 < best_lt5 || (curr_lt5 == best_lt5 && curr_min_ang > best_min_ang + 1e-5) {
            best_lt5 = curr_lt5;
            best_min_ang = curr_min_ang;
            best_pts = cap_pts.clone();
            best_tris = triangles.clone();
        }
        if curr_lt5 == 0 && curr_min_ang >= target_min_rad {
            break;
        }

        // Find the worst sliver triangle (smallest min_angle < target_min_rad)
        let mut worst_idx: Option<usize> = None;
        let mut worst_ang = target_min_rad;
        for (idx, t) in triangles.iter().enumerate() {
            let p0 = cap_pts[t[0] as usize];
            let p1 = cap_pts[t[1] as usize];
            let p2 = cap_pts[t[2] as usize];
            let ang = min_triangle_angle_2d(p0, p1, p2);
            if ang < worst_ang {
                worst_ang = ang;
                worst_idx = Some(idx);
            }
        }

        let Some(seed_idx) = worst_idx else {
            break;
        };

        // Place Steiner point halfway along the median from the shortest edge to the opposite apex
        let [u, v, w] = triangles[seed_idx];
        let pu = cap_pts[u as usize];
        let pv = cap_pts[v as usize];
        let pw = cap_pts[w as usize];
        let d_uv = (pu[0] - pv[0]).powi(2) + (pu[1] - pv[1]).powi(2);
        let d_vw = (pv[0] - pw[0]).powi(2) + (pv[1] - pw[1]).powi(2);
        let d_wu = (pw[0] - pu[0]).powi(2) + (pw[1] - pu[1]).powi(2);
        let steiner = if d_uv <= d_vw && d_uv <= d_wu {
            [0.25 * (pu[0] + pv[0]) + 0.5 * pw[0], 0.25 * (pu[1] + pv[1]) + 0.5 * pw[1]]
        } else if d_vw <= d_uv && d_vw <= d_wu {
            [0.25 * (pv[0] + pw[0]) + 0.5 * pu[0], 0.25 * (pv[1] + pw[1]) + 0.5 * pu[1]]
        } else {
            [0.25 * (pw[0] + pu[0]) + 0.5 * pv[0], 0.25 * (pw[1] + pu[1]) + 0.5 * pv[1]]
        };

        // Build connected Bowyer-Watson circumcircle cavity around `seed_idx`
        let n_tris = triangles.len();
        let mut half_edge_map: std::collections::HashMap<(u32, u32), usize> =
            std::collections::HashMap::with_capacity(n_tris * 3);
        for (t_idx, t) in triangles.iter().enumerate() {
            for e in 0..3 {
                half_edge_map.insert((t[e], t[(e + 1) % 3]), t_idx);
            }
        }

        let mut in_cavity = vec![false; n_tris];
        let mut queue = std::collections::VecDeque::new();
        in_cavity[seed_idx] = true;
        queue.push_back(seed_idx);

        while let Some(curr_t) = queue.pop_front() {
            let t = triangles[curr_t];
            for e in 0..3 {
                let a = t[e];
                let b = t[(e + 1) % 3];
                if let Some(&nbr_t) = half_edge_map.get(&(b, a)) {
                    if !in_cavity[nbr_t] {
                        let nt = triangles[nbr_t];
                        if in_circumcircle(
                            steiner,
                            cap_pts[nt[0] as usize],
                            cap_pts[nt[1] as usize],
                            cap_pts[nt[2] as usize],
                        ) {
                            in_cavity[nbr_t] = true;
                            queue.push_back(nbr_t);
                        }
                    }
                }
            }
        }

        // Prune non-star-shaped cavity triangles (except seed_idx) so `steiner` sees all cavity boundary edges
        loop {
            let mut removed_any = false;
            for t_idx in 0..n_tris {
                if !in_cavity[t_idx] || t_idx == seed_idx {
                    continue;
                }
                let t = triangles[t_idx];
                let mut bad_boundary = false;
                for e in 0..3 {
                    let a = t[e];
                    let b = t[(e + 1) % 3];
                    let is_cav_boundary = match half_edge_map.get(&(b, a)) {
                        Some(&nbr) => !in_cavity[nbr],
                        None => true,
                    };
                    if is_cav_boundary {
                        let pa = cap_pts[a as usize];
                        let pb = cap_pts[b as usize];
                        let len_sq = (pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2);
                        if cross_2d(pa, pb, steiner) <= 1e-10 * len_sq {
                            bad_boundary = true;
                            break;
                        }
                    }
                }
                if bad_boundary {
                    in_cavity[t_idx] = false;
                    removed_any = true;
                }
            }
            if !removed_any {
                break;
            }
        }

        // Collect cavity boundary edges and verify all are strictly visible from `steiner`
        let mut boundary_edges: Vec<(u32, u32)> = Vec::new();
        let mut all_visible = true;
        for t_idx in 0..n_tris {
            if !in_cavity[t_idx] {
                continue;
            }
            let t = triangles[t_idx];
            for e in 0..3 {
                let a = t[e];
                let b = t[(e + 1) % 3];
                let is_cav_boundary = match half_edge_map.get(&(b, a)) {
                    Some(&nbr) => !in_cavity[nbr],
                    None => true,
                };
                if is_cav_boundary {
                    let pa = cap_pts[a as usize];
                    let pb = cap_pts[b as usize];
                    let len_sq = (pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2);
                    if cross_2d(pa, pb, steiner) <= 1e-10 * len_sq {
                        all_visible = false;
                        break;
                    }
                    boundary_edges.push((a, b));
                }
            }
            if !all_visible {
                break;
            }
        }

        if !all_visible || boundary_edges.len() < 3 {
            break;
        }

        let s_idx = cap_pts.len() as u32;
        cap_pts.push(steiner);

        let mut next_tris = Vec::with_capacity(n_tris + boundary_edges.len());
        for (t_idx, &t) in triangles.iter().enumerate() {
            if !in_cavity[t_idx] {
                next_tris.push(t);
            }
        }
        for (a, b) in boundary_edges {
            next_tris.push([a, b, s_idx]);
        }
        triangles = next_tris;

        optimize_2d_delaunay_edges(&mut triangles, &cap_pts);
    }

    let (curr_lt5, curr_min_ang) = eval_quality(&cap_pts, &triangles);
    if curr_lt5 < best_lt5 || (curr_lt5 == best_lt5 && curr_min_ang > best_min_ang + 1e-5) {
        (cap_pts, triangles)
    } else {
        (best_pts, best_tris)
    }
}

/// Generates a solid 3D body of revolution by rotating a 2D profile `[r, z]` around the Z axis.
///
/// Features:
/// - 2D profile coordinates: `r` (distance from rotation axis Z, r >= 0) and `z` (axial position).
/// - Complete 360-degree rotation or partial angle `angle_deg` (0 < angle_deg <= 360).
/// - Radial resolution controlled by `segments` (minimum 3).
/// - Manifold watertight topology with seam stitching at 360 degrees and planar end-caps for partial angles.
/// - Surface normal computation with outward orientation.
pub fn create_revolve(profile: &[[f64; 2]], angle_deg: f64, segments: usize) -> TriangleMesh {
    if profile.len() < 3 || angle_deg <= 1e-6 {
        return TriangleMesh::empty();
    }

    let max_r = profile.iter().map(|p| p[0].abs()).fold(0.0, f64::max);
    let segs = if segments == 0 {
        TessellationSettings::default().calculate_segments_arc(max_r, angle_deg * PI / 180.0)
    } else {
        segments.max(3)
    };
    let sweep_angle_deg = angle_deg.clamp(1e-4, 360.0);
    let is_full_circle = (sweep_angle_deg - 360.0).abs() < 1e-5;
    let sweep_angle_rad = sweep_angle_deg * PI / 180.0;

    // 1. Clean profile: ensure r >= 0 and remove duplicate consecutive points
    let mut raw_pts: Vec<[f64; 2]> = Vec::with_capacity(profile.len());
    for p in profile {
        let r = p[0].max(0.0);
        let z = p[1];
        if let Some(last) = raw_pts.last() {
            let d2 = (r - last[0]).powi(2) + (z - last[1]).powi(2);
            if d2 > 1e-12 {
                raw_pts.push([r, z]);
            }
        } else {
            raw_pts.push([r, z]);
        }
    }
    if raw_pts.len() > 2 {
        let first = raw_pts[0];
        let last = *raw_pts.last().unwrap();
        let d2 = (first[0] - last[0]).powi(2) + (first[1] - last[1]).powi(2);
        if d2 <= 1e-12 {
            raw_pts.pop();
        }
    }
    if raw_pts.len() < 3 {
        return TriangleMesh::empty();
    }

    // 2. Ensure CCW orientation in the (r, z) half-plane (r as X, z as Y)
    let mut area2d = 0.0;
    for i in 0..raw_pts.len() {
        let next = (i + 1) % raw_pts.len();
        area2d += raw_pts[i][0] * raw_pts[next][1] - raw_pts[next][0] * raw_pts[i][1];
    }
    if area2d < 0.0 {
        raw_pts.reverse();
    }

    // Subdivide profile segments so revolved quads, axis transitions, and partial-angle end caps have balanced aspect ratios
    let base_target_len = ((max_r * sweep_angle_rad) / (segs as f64) * 1.35)
        .max(max_r * 0.15)
        .max(1e-5);
    let mut pts: Vec<[f64; 2]> = Vec::with_capacity(raw_pts.len() * 4);
    let mut pt_segs: Vec<usize> = Vec::with_capacity(raw_pts.len() * 4);

    for i in 0..raw_pts.len() {
        let p0 = raw_pts[i];
        let p1 = raw_pts[(i + 1) % raw_pts.len()];
        let r0 = if p0[0] < 1e-9 { 0.0 } else { p0[0] };
        let r1 = if p1[0] < 1e-9 { 0.0 } else { p1[0] };
        let seg_len = (p1[0] - p0[0]).hypot(p1[1] - p0[1]);

        if r0 < 1e-9 && r1 < 1e-9 {
            // Edge along rotation axis Z
            let n_sub = ((seg_len / base_target_len).ceil() as usize).clamp(1, 16);
            for s in 0..n_sub {
                let t = (s as f64) / (n_sub as f64);
                pts.push([0.0, p0[1] + (p1[1] - p0[1]) * t]);
                pt_segs.push(segs);
            }
        } else if r0 < 1e-9 || r1 < 1e-9 {
            // Edge touching rotation axis at one end: build stations from outer (u=0, r=R) to axis (u=1, r=0)
            // using 2-to-1 angular halving when angular step < ~10.3 deg so apex fan and inner rings have zero slivers
            let r_outer = r0.max(r1);
            let p_outer = if r0 > 0.0 { p0 } else { p1 };
            let p_axis = if r0 > 0.0 { p1 } else { p0 };
            let radial_ratio = (r_outer / seg_len.max(1e-12)).clamp(0.1, 1.0);

            let mut stations: Vec<(f64, usize)> = vec![(0.0, segs)];
            let mut rem = 1.0_f64;
            let mut curr_s = segs;

            for _step in 0..12 {
                let dtheta_eff = (sweep_angle_rad / (curr_s as f64)) * radial_ratio;
                if curr_s > 2 && dtheta_eff < 0.18 {
                    let factor = (1.0 - 1.25 * dtheta_eff).clamp(0.55, 0.85);
                    rem *= factor;
                    curr_s = curr_s.div_ceil(2);
                    stations.push((1.0 - rem, curr_s));
                } else if rem * seg_len > base_target_len * 1.45 {
                    let next_s = if curr_s >= 8 && dtheta_eff < 0.30 {
                        curr_s.div_ceil(2)
                    } else {
                        curr_s
                    };
                    rem *= 0.62;
                    curr_s = next_s;
                    stations.push((1.0 - rem, curr_s));
                } else {
                    break;
                }
            }

            if r0 > 0.0 {
                // Outer -> Axis
                for &(u, s_cnt) in &stations {
                    pts.push([
                        p_outer[0] + (p_axis[0] - p_outer[0]) * u,
                        p_outer[1] + (p_axis[1] - p_outer[1]) * u,
                    ]);
                    pt_segs.push(s_cnt);
                }
            } else {
                // Axis -> Outer
                let last_s = stations.last().map(|x| x.1).unwrap_or(segs);
                pts.push([0.0, p_axis[1]]);
                pt_segs.push(last_s);
                for &(u, s_cnt) in stations.iter().skip(1).rev() {
                    pts.push([
                        p_outer[0] + (p_axis[0] - p_outer[0]) * u,
                        p_outer[1] + (p_axis[1] - p_outer[1]) * u,
                    ]);
                    pt_segs.push(s_cnt);
                }
            }
        } else {
            // Both endpoints have r > 0: ensure sub-segment length stays within 9x of local minimum circumferential chord
            let r_min = r0.min(r1);
            let c_min = (r_min * sweep_angle_rad / (segs as f64)).max(1e-6);
            let local_target = base_target_len.min(c_min * 9.0).max(1e-5);
            let n_sub = ((seg_len / local_target).ceil() as usize).clamp(1, 32);
            for s in 0..n_sub {
                let t = (s as f64) / (n_sub as f64);
                pts.push([p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t]);
                pt_segs.push(segs);
            }
        }
    }
    let m = pts.len();

    let build_thetas = |s_cnt: usize| -> Vec<(f64, f64)> {
        let n_steps = if is_full_circle { s_cnt } else { s_cnt + 1 };
        (0..n_steps)
            .map(|j| {
                let theta = if is_full_circle {
                    (j as f64) * 2.0 * PI / (s_cnt as f64)
                } else {
                    (j as f64) * sweep_angle_rad / (s_cnt as f64)
                };
                (theta.cos(), theta.sin())
            })
            .collect()
    };

    // 3. Generate side-wall strips per profile sub-segment with exact analytical normals
    let mut vertices = Vec::with_capacity((segs + 1) * m * 2);
    let mut indices = Vec::new();

    for i in 0..m {
        let next_i = (i + 1) % m;
        let r_a = if pts[i][0] < 1e-9 { 0.0 } else { pts[i][0] };
        let z_a = pts[i][1];
        let r_b = if pts[next_i][0] < 1e-9 { 0.0 } else { pts[next_i][0] };
        let z_b = pts[next_i][1];

        // Edge lying on rotation axis has zero surface area
        if r_a < 1e-9 && r_b < 1e-9 {
            continue;
        }

        // Outward 2D normal for CCW profile in (r, z): (dz, -dr) / len
        let dr = r_b - r_a;
        let dz = z_b - z_a;
        let seg_len = (dr * dr + dz * dz).sqrt().max(1e-12);
        let nr = dz / seg_len;
        let nz = -dr / seg_len;

        let segs_a = if r_a < 1e-9 { pt_segs[next_i] } else { pt_segs[i] };
        let segs_b = if r_b < 1e-9 { pt_segs[i] } else { pt_segs[next_i] };

        let thetas_a = build_thetas(segs_a);
        let thetas_b = build_thetas(segs_b);

        let base_va = vertices.len() as u32;
        for &(cos_t, sin_t) in &thetas_a {
            let xa = if r_a == 0.0 { 0.0 } else { r_a * cos_t };
            let ya = if r_a == 0.0 { 0.0 } else { r_a * sin_t };
            let n3 = [nr * cos_t, nr * sin_t, nz];
            vertices.push(Vertex::new([xa, ya, z_a], n3));
        }
        let base_vb = vertices.len() as u32;
        for &(cos_t, sin_t) in &thetas_b {
            let xb = if r_b == 0.0 { 0.0 } else { r_b * cos_t };
            let yb = if r_b == 0.0 { 0.0 } else { r_b * sin_t };
            let n3 = [nr * cos_t, nr * sin_t, nz];
            vertices.push(Vertex::new([xb, yb, z_b], n3));
        }

        let va = |j: usize| -> u32 {
            let idx = if is_full_circle { j % segs_a } else { j };
            base_va + (idx as u32)
        };
        let vb = |j: usize| -> u32 {
            let idx = if is_full_circle { j % segs_b } else { j };
            base_vb + (idx as u32)
        };

        if r_a < 1e-9 {
            for j in 0..segs_b {
                indices.push([va(j), vb(j + 1), vb(j)]);
            }
        } else if r_b < 1e-9 {
            for j in 0..segs_a {
                indices.push([va(j), va(j + 1), vb(j)]);
            }
        } else if segs_a == segs_b {
            for j in 0..segs_a {
                let a0 = va(j);
                let a1 = va(j + 1);
                let b0 = vb(j);
                let b1 = vb(j + 1);
                indices.push([a0, a1, b1]);
                indices.push([a0, b1, b0]);
            }
        } else if segs_a > segs_b {
            // 2-to-1 reduction from ring a (segs_a) to ring b (segs_b = (segs_a + 1) / 2)
            let k = segs_a - segs_b;
            for j in 0..k {
                let a0 = va(2 * j);
                let a1 = va(2 * j + 1);
                let a2 = va(2 * j + 2);
                let b0 = vb(j);
                let b1 = vb(j + 1);
                indices.push([a0, a1, b0]);
                indices.push([a1, b1, b0]);
                indices.push([a1, a2, b1]);
            }
            if 2 * k < segs_a {
                let a0 = va(2 * k);
                let a1 = va(2 * k + 1);
                let b0 = vb(k);
                let b1 = vb(k + 1);
                indices.push([a0, a1, b1]);
                indices.push([a0, b1, b0]);
            }
        } else {
            // 1-to-2 expansion from ring a (segs_a = (segs_b + 1) / 2) to ring b (segs_b)
            let k = segs_b - segs_a;
            for j in 0..k {
                let a0 = va(j);
                let a1 = va(j + 1);
                let b0 = vb(2 * j);
                let b1 = vb(2 * j + 1);
                let b2 = vb(2 * j + 2);
                indices.push([a0, b1, b0]);
                indices.push([a0, a1, b1]);
                indices.push([a1, b2, b1]);
            }
            if 2 * k < segs_b {
                let a0 = va(k);
                let a1 = va(k + 1);
                let b0 = vb(2 * k);
                let b1 = vb(2 * k + 1);
                indices.push([a0, a1, b1]);
                indices.push([a0, b1, b0]);
            }
        }
    }

    // 4. If partial angle, cap the planar start and end faces with dedicated flat-normal vertices
    if !is_full_circle {
        let (cap_pts, cap_triangles) = triangulate_cap_polygon_2d(&pts);

        // Start cap at theta = 0: outward normal points towards -Y (e_r x e_z = [0, -1, 0])
        let start_cap_base = vertices.len() as u32;
        let start_normal = [0.0, -1.0, 0.0];
        for p in &cap_pts {
            let r = if p[0] < 1e-9 { 0.0 } else { p[0] };
            vertices.push(Vertex::new([r, 0.0, p[1]], start_normal));
        }
        for tri in &cap_triangles {
            indices.push([
                start_cap_base + tri[0],
                start_cap_base + tri[1],
                start_cap_base + tri[2],
            ]);
        }

        // End cap at theta = sweep_angle: outward normal points towards +e_theta = [-sin, cos, 0]
        let end_cap_base = vertices.len() as u32;
        let (cos_end, sin_end) = (sweep_angle_rad.cos(), sweep_angle_rad.sin());
        let end_normal = [-sin_end, cos_end, 0.0];
        for p in &cap_pts {
            let r = if p[0] < 1e-9 { 0.0 } else { p[0] };
            let x = if r == 0.0 { 0.0 } else { r * cos_end };
            let y = if r == 0.0 { 0.0 } else { r * sin_end };
            vertices.push(Vertex::new([x, y, p[1]], end_normal));
        }
        for tri in &cap_triangles {
            indices.push([
                end_cap_base + tri[0],
                end_cap_base + tri[2],
                end_cap_base + tri[1],
            ]);
        }
    }

    let mut mesh = TriangleMesh::new(vertices, indices);
    mesh.face_ids = vec![0; mesh.indices.len()];

    // 5. Enforce outward orientation
    if mesh.signed_volume() < 0.0 {
        for tri in &mut mesh.indices {
            tri.swap(1, 2);
        }
        for v in &mut mesh.vertices {
            v.normal = [-v.normal[0], -v.normal[1], -v.normal[2]];
        }
    }

    mesh
}

/// Generates a solid 3D body of revolution with adaptive tessellation based on `TessellationSettings`.
pub fn create_revolve_with_settings(
    profile: &[[f64; 2]],
    angle_deg: f64,
    settings: &TessellationSettings,
) -> TriangleMesh {
    let max_r = profile.iter().map(|p| p[0].abs()).fold(0.0, f64::max);
    let segments = settings.calculate_segments_arc(max_r, angle_deg * PI / 180.0);
    create_revolve(profile, angle_deg, segments)
}

/// Generates a solid 3D parametric automotive wheel rim according to tire & wheel engineering standards.
///
/// Features:
/// - Stepped rim barrel with inner/outer flanges, bead seat shelves, and drop center mounting well (монтажный ручей).
/// - Central mounting hub with central bore hole (`hub_bore_diameter`).
/// - Standard PCD (Pitch Circle Diameter) bolt pattern with `bolt_holes_count` holes (`bolt_hole_diameter`).
/// - Structural spokes (`spokes_count`, `spoke_width`, `spoke_thickness`) connecting hub and rim barrel.
/// - Open brake cooling windows between spokes for caliper and rotor ventilation.
/// - Wheel offset ET (`offset_et`) setting hub mounting pad position relative to rim barrel centerline.
#[allow(clippy::too_many_arguments)]
pub fn create_wheel_rim(
    rim_diameter: f64,
    rim_width: f64,
    flange_diameter: Option<f64>,
    hub_diameter: Option<f64>,
    hub_bore_diameter: Option<f64>,
    pcd_diameter: Option<f64>,
    bolt_holes_count: Option<usize>,
    bolt_hole_diameter: Option<f64>,
    spokes_count: Option<usize>,
    spoke_width: Option<f64>,
    spoke_thickness: Option<f64>,
    offset_et: Option<f64>,
) -> TriangleMesh {
    create_wheel_rim_with_settings(
        rim_diameter,
        rim_width,
        flange_diameter,
        hub_diameter,
        hub_bore_diameter,
        pcd_diameter,
        bolt_holes_count,
        bolt_hole_diameter,
        spokes_count,
        spoke_width,
        spoke_thickness,
        offset_et,
        &TessellationSettings::default(),
    )
}

/// Generates a solid 3D parametric automotive wheel rim with adaptive tessellation based on `TessellationSettings`.
#[allow(clippy::too_many_arguments)]
pub fn create_wheel_rim_with_settings(
    rim_diameter: f64,
    rim_width: f64,
    flange_diameter: Option<f64>,
    hub_diameter: Option<f64>,
    hub_bore_diameter: Option<f64>,
    pcd_diameter: Option<f64>,
    bolt_holes_count: Option<usize>,
    bolt_hole_diameter: Option<f64>,
    spokes_count: Option<usize>,
    spoke_width: Option<f64>,
    spoke_thickness: Option<f64>,
    offset_et: Option<f64>,
    settings: &TessellationSettings,
) -> TriangleMesh {
    if rim_diameter <= 1e-4 || rim_width <= 1e-4 {
        return TriangleMesh::empty();
    }

    #[inline]
    fn safe_clamp(val: f64, lo: f64, hi: f64) -> f64 {
        if lo <= hi {
            val.clamp(lo, hi)
        } else {
            (lo + hi) * 0.5
        }
    }

    let d_rim = rim_diameter;
    let w_rim = rim_width;
    let r_bead = d_rim * 0.5;

    // Proportional scale factors for small/model wheel rims (< 350 mm diameter or < 160 mm width).
    // For full-size automotive rims (d_rim >= 0.35 m, w_rim >= 0.16 m), s_rim == 1.0 and s_width == 1.0,
    // preserving exact geometry while scaling down clearances homothetically for small wheels.
    let s_rim = (d_rim / 0.35).min(1.0);
    let s_width = (w_rim / 0.16).min(1.0);
    let s_min = s_rim.min(s_width);

    let d_flange = flange_diameter.unwrap_or(d_rim * 1.08).max(d_rim * 1.02);
    let r_flange = d_flange * 0.5;

    let d_hub = safe_clamp(
        hub_diameter.unwrap_or(d_rim * 0.35),
        d_rim * 0.15,
        d_rim * 0.65,
    );
    let r_hub = d_hub * 0.5;

    let d_bore = safe_clamp(
        hub_bore_diameter.unwrap_or(d_hub * 0.45),
        d_hub * 0.15,
        d_hub * 0.70,
    );
    let r_bore = d_bore * 0.5;

    let d_pcd = safe_clamp(
        pcd_diameter.unwrap_or((d_hub + d_bore) * 0.5),
        d_bore + 0.015 * s_rim,
        d_hub - 0.012 * s_rim,
    );
    let r_pcd = d_pcd * 0.5;

    let n_bolts = bolt_holes_count.unwrap_or(5).clamp(3, 12);
    let d_bolt = safe_clamp(
        bolt_hole_diameter.unwrap_or(0.014 * s_rim),
        0.005 * s_rim,
        (r_hub - r_pcd)
            .min(r_pcd - r_bore)
            .max(0.008 * s_rim)
            * 1.6,
    );
    let r_bolt = d_bolt * 0.5;

    let n_spokes = spokes_count.unwrap_or(5).max(3);

    let max_spoke_w = (2.0 * PI * r_hub / (n_spokes as f64)) * 0.75;
    let w_spoke = safe_clamp(
        spoke_width.unwrap_or(d_rim * 0.055),
        0.010 * s_rim,
        max_spoke_w,
    );

    let t_spoke = safe_clamp(
        spoke_thickness.unwrap_or(w_spoke * 0.65),
        0.008 * s_width,
        w_rim * 0.4,
    );

    let et = safe_clamp(
        offset_et.unwrap_or(0.035 * s_width),
        -w_rim * 0.4,
        w_rim * 0.4,
    );

    // 1. Key radial & axial dimensions with safe geometric clearances
    let t_w = safe_clamp(r_bead * 0.045, 0.006 * s_rim, 0.012 * s_rim);
    let drop_depth = safe_clamp(r_bead * 0.08, 0.015 * s_rim, 0.035 * s_rim);
    let r_well = (r_bead - drop_depth).max(r_hub + t_w + 0.015 * s_rim);
    let r_inner_barrel = r_well - t_w;
    let r_hump = r_bead + safe_clamp(r_bead * 0.012, 0.002 * s_rim, 0.005 * s_rim);

    let r_hub_eff = r_hub.min(r_inner_barrel * 0.78).max(r_bore + 0.018 * s_rim);
    let r_bore_eff = r_bore.min(r_hub_eff * 0.65);
    let r_bolt_ring = (r_hub_eff - (r_hub_eff - r_bore_eff) * 0.15).max(r_bore_eff + 0.012 * s_rim);
    let radial_band = r_bolt_ring - r_bore_eff;
    let r_pcd_eff = safe_clamp(
        r_pcd,
        r_bore_eff + radial_band * 0.28,
        r_bolt_ring - radial_band * 0.28,
    );
    let max_r_bolt = (r_pcd_eff - r_bore_eff)
        .min(r_bolt_ring - r_pcd_eff)
        .min(r_pcd_eff * (PI / (n_bolts as f64)).sin())
        * 0.55;
    let r_bolt_eff = safe_clamp(r_bolt, 0.002 * s_rim, max_r_bolt.max(0.0025 * s_rim));

    let q_mult = match settings.quality {
        TessellationQuality::Coarse => 1,
        TessellationQuality::Medium => 2,
        TessellationQuality::Fine => 3,
        TessellationQuality::Custom { .. } => 2,
    };

    // 2. Angular discretization of the spoke + window sectors (N_theta stations around 360 deg)
    let k_s = 2 * q_mult; // segments across each spoke
    let k_w = 4 * q_mult; // segments across each ventilation window
    let k_sec = k_s + k_w;
    let n_theta = n_spokes * k_sec;
    let d_theta_sec = 2.0 * PI / (n_spokes as f64);

    let h_w = w_rim * 0.5;
    let z_well_bot = -h_w + 0.30 * w_rim;
    let z_well_top = h_w - 0.30 * w_rim;
    let max_disc_t = (z_well_top - z_well_bot) * 0.55;
    let t_disc = safe_clamp(t_spoke, 0.010 * s_width, max_disc_t.max(0.012 * s_width));
    let min_barrel_dz = (2.0 * PI * r_inner_barrel / (n_theta as f64) * 0.12)
        .max(0.006 * s_min)
        .min(((z_well_top - z_well_bot - t_disc) * 0.45).max(1e-5));
    let z_disc_center = safe_clamp(
        et - t_disc * 0.5,
        z_well_bot + t_disc * 0.5 + min_barrel_dz,
        z_well_top - t_disc * 0.5 - min_barrel_dz,
    );
    let z_back = z_disc_center - t_disc * 0.5;
    let z_front = z_disc_center + t_disc * 0.5;

    let delta_hub = safe_clamp(
        w_spoke / (2.0 * r_hub_eff),
        0.12 * d_theta_sec,
        0.34 * d_theta_sec,
    );
    let delta_barrel = safe_clamp(
        w_spoke / (2.0 * r_inner_barrel),
        0.08 * d_theta_sec,
        0.34 * d_theta_sec,
    );

    let mut phi_hub = Vec::with_capacity(n_theta);
    let mut phi_barrel = Vec::with_capacity(n_theta);
    for s in 0..n_spokes {
        let base_ang = (s as f64) * d_theta_sec;
        // Spoke indices: 0..k_s covers [-delta, +delta] around base_ang + 0.5 * d_theta_sec?
        // Place spoke centered at base_ang + delta_max so phi starts >= 0 and increases strictly in [0, 2*PI)
        let spoke_center = base_ang + 0.25 * d_theta_sec;
        let next_spoke_center = base_ang + 1.25 * d_theta_sec;
        for m in 0..k_s {
            let t = (m as f64) / (k_s as f64);
            phi_hub.push(spoke_center - delta_hub + t * (2.0 * delta_hub));
            phi_barrel.push(spoke_center - delta_barrel + t * (2.0 * delta_barrel));
        }
        for m in 0..k_w {
            let t = (m as f64) / (k_w as f64);
            let h_start = spoke_center + delta_hub;
            let h_end = next_spoke_center - delta_hub;
            let b_start = spoke_center + delta_barrel;
            let b_end = next_spoke_center - delta_barrel;
            phi_hub.push(h_start + t * (h_end - h_start));
            phi_barrel.push(b_start + t * (b_end - b_start));
        }
    }

    let hub_outer_ring: Vec<[f64; 2]> = phi_hub
        .iter()
        .map(|&a| [r_hub_eff * a.cos(), r_hub_eff * a.sin()])
        .collect();
    let barrel_inner_ring: Vec<[f64; 2]> = phi_barrel
        .iter()
        .map(|&a| [r_inner_barrel * a.cos(), r_inner_barrel * a.sin()])
        .collect();

    let mut vertices: Vec<Vertex> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();

    // Helper: emit a single quad (2 triangles) split along its shorter diagonal
    macro_rules! push_quad {
        ($p0:expr, $p1:expr, $p2:expr, $p3:expr, $n:expr $(,)?) => {
            push_quad_shortest_diag(&mut vertices, &mut indices, $p0, $p1, $p2, $p3, $n);
        };
    }

    // 3. Build stepped rim barrel profile in CCW (r, z) order
    // Segment 15 ([r_inner_barrel, z_front] -> [r_inner_barrel, z_back]) is the spoke/window attachment band.
    let barrel_pts: Vec<[f64; 2]> = vec![
        [r_flange, -h_w],                                  // 0
        [r_flange, -h_w + 0.05 * w_rim],                   // 1
        [r_bead + 0.004 * s_rim, -h_w + 0.08 * w_rim],     // 2
        [r_bead, -h_w + 0.12 * w_rim],                     // 3
        [r_bead, -h_w + 0.22 * w_rim],                     // 4
        [r_well, z_well_bot],                              // 5
        [r_well, z_well_top],                              // 6
        [r_hump, h_w - 0.22 * w_rim],                      // 7
        [r_bead, h_w - 0.18 * w_rim],                      // 8
        [r_bead, h_w - 0.08 * w_rim],                      // 9
        [r_flange, h_w - 0.04 * w_rim],                    // 10
        [r_flange, h_w],                                   // 11
        [r_flange - t_w, h_w],                             // 12
        [r_bead - t_w, h_w - 0.08 * w_rim],                // 13
        [r_inner_barrel, z_well_top],                      // 14
        [r_inner_barrel, z_front],                         // 15 (top of spoke/window band)
        [r_inner_barrel, z_back],                          // 16 (bottom of spoke/window band)
        [r_inner_barrel, z_well_bot],                      // 17
        [r_bead - t_w, -h_w + 0.22 * w_rim],               // 18
        [r_flange - t_w, -h_w],                            // 19
    ];

    let n_b_pts = barrel_pts.len();
    for seg_i in 0..n_b_pts {
        let next_i = (seg_i + 1) % n_b_pts;
        let ra = barrel_pts[seg_i][0];
        let za = barrel_pts[seg_i][1];
        let rb = barrel_pts[next_i][0];
        let zb = barrel_pts[next_i][1];

        let dr = rb - ra;
        let dz = zb - za;
        let len = dr.hypot(dz).max(1e-12);
        let nr = dz / len;
        let nz = -dr / len;

        let is_spoke_band = seg_i == 15; // [r_inner_barrel, z_front] -> [r_inner_barrel, z_back]

        for j in 0..n_theta {
            if is_spoke_band {
                let in_sec = j % k_sec;
                if in_sec < k_s {
                    // Spoke attaches here; no barrel wall across the spoke cross-section
                    continue;
                }
            }
            let next_j = (j + 1) % n_theta;
            let a0 = phi_barrel[j];
            let a1 = phi_barrel[next_j];
            let mid_a = 0.5 * (a0 + if next_j == 0 { a1 + 2.0 * PI } else { a1 });
            let n3 = [nr * mid_a.cos(), nr * mid_a.sin(), nz];

            let p0 = [ra * a0.cos(), ra * a0.sin(), za];
            let p1 = [ra * a1.cos(), ra * a1.sin(), za];
            let p2 = [rb * a1.cos(), rb * a1.sin(), zb];
            let p3 = [rb * a0.cos(), rb * a0.sin(), zb];
            push_quad!(p0, p1, p2, p3, n3);
        }
    }

    // 4. Spokes and Ventilation Window Walls between r_hub_eff and r_inner_barrel (z in [z_back, z_front])
    let n_r_spoke = (((r_inner_barrel - r_hub_eff) / (w_spoke / (k_s as f64) * 1.4)).ceil() as usize).clamp(3, 8);
    for s in 0..n_spokes {
        let sec_start = s * k_sec;

        // 4a. Spoke top (+Z) and bottom (-Z) faces across k_s segments and n_r_spoke radial steps
        for m in 0..k_s {
            let j0 = sec_start + m;
            let j1 = (j0 + 1) % n_theta;
            let h0 = hub_outer_ring[j0];
            let h1 = hub_outer_ring[j1];
            let b0 = barrel_inner_ring[j0];
            let b1 = barrel_inner_ring[j1];

            for kr in 0..n_r_spoke {
                let t0 = (kr as f64) / (n_r_spoke as f64);
                let t1 = ((kr + 1) as f64) / (n_r_spoke as f64);
                let r0_a = [h0[0] + (b0[0] - h0[0]) * t0, h0[1] + (b0[1] - h0[1]) * t0];
                let r1_a = [h0[0] + (b0[0] - h0[0]) * t1, h0[1] + (b0[1] - h0[1]) * t1];
                let r1_b = [h1[0] + (b1[0] - h1[0]) * t1, h1[1] + (b1[1] - h1[1]) * t1];
                let r0_b = [h1[0] + (b1[0] - h1[0]) * t0, h1[1] + (b1[1] - h1[1]) * t0];

                // Front face at z_front (normal [0, 0, 1])
                push_quad!(
                    [r0_a[0], r0_a[1], z_front],
                    [r1_a[0], r1_a[1], z_front],
                    [r1_b[0], r1_b[1], z_front],
                    [r0_b[0], r0_b[1], z_front],
                    [0.0, 0.0, 1.0],
                );
                // Back face at z_back (normal [0, 0, -1])
                push_quad!(
                    [r0_a[0], r0_a[1], z_back],
                    [r0_b[0], r0_b[1], z_back],
                    [r1_b[0], r1_b[1], z_back],
                    [r1_a[0], r1_a[1], z_back],
                    [0.0, 0.0, -1.0],
                );
            }
        }

        // 4b. Leading side wall of spoke s at j_lead = sec_start (from barrel to hub, facing -theta)
        let j_lead = sec_start;
        let hl = hub_outer_ring[j_lead];
        let bl = barrel_inner_ring[j_lead];
        let dxl = bl[0] - hl[0];
        let dyl = bl[1] - hl[1];
        let len_l = dxl.hypot(dyl).max(1e-12);
        let nl = [dyl / len_l, -dxl / len_l, 0.0];
        for kr in 0..n_r_spoke {
            let t0 = (kr as f64) / (n_r_spoke as f64);
            let t1 = ((kr + 1) as f64) / (n_r_spoke as f64);
            let p0 = [hl[0] + dxl * t0, hl[1] + dyl * t0];
            let p1 = [hl[0] + dxl * t1, hl[1] + dyl * t1];
            push_quad!(
                [p0[0], p0[1], z_front],
                [p0[0], p0[1], z_back],
                [p1[0], p1[1], z_back],
                [p1[0], p1[1], z_front],
                nl,
            );
        }

        // 4c. Trailing side wall of spoke s at j_trail = sec_start + k_s (facing +theta)
        let j_trail = sec_start + k_s;
        let ht = hub_outer_ring[j_trail];
        let bt = barrel_inner_ring[j_trail];
        let dxt = bt[0] - ht[0];
        let dyt = bt[1] - ht[1];
        let len_t = dxt.hypot(dyt).max(1e-12);
        let nt = [-dyt / len_t, dxt / len_t, 0.0];
        for kr in 0..n_r_spoke {
            let t0 = (kr as f64) / (n_r_spoke as f64);
            let t1 = ((kr + 1) as f64) / (n_r_spoke as f64);
            let p0 = [ht[0] + dxt * t0, ht[1] + dyt * t0];
            let p1 = [ht[0] + dxt * t1, ht[1] + dyt * t1];
            push_quad!(
                [p1[0], p1[1], z_front],
                [p1[0], p1[1], z_back],
                [p0[0], p0[1], z_back],
                [p0[0], p0[1], z_front],
                nt,
            );
        }

        // 4d. Hub outer cylindrical wall along the ventilation window (k_s .. k_sec)
        for m in k_s..k_sec {
            let j0 = sec_start + m;
            let j1 = (j0 + 1) % n_theta;
            let h0 = hub_outer_ring[j0];
            let h1 = hub_outer_ring[j1];
            let dxh = h1[0] - h0[0];
            let dyh = h1[1] - h0[1];
            let len_h = dxh.hypot(dyh).max(1e-12);
            let nh = [dyh / len_h, -dxh / len_h, 0.0];
            push_quad!(
                [h0[0], h0[1], z_front],
                [h0[0], h0[1], z_back],
                [h1[0], h1[1], z_back],
                [h1[0], h1[1], z_front],
                nh,
            );
        }
    }

    // 5. Bolt Sector O-Grids inside [r_bore_eff, r_bolt_ring] for each of the n_bolts PCD holes
    let k_b = 2 * q_mult; // segments per side of each bolt sector
    let d_beta = 2.0 * PI / (n_bolts as f64);
    let mut bolt_outer_ring: Vec<[f64; 2]> = Vec::with_capacity(n_bolts * k_b);
    let mut bore_ring: Vec<[f64; 2]> = Vec::with_capacity(n_bolts * k_b);

    for b in 0..n_bolts {
        let beta_min = (b as f64) * d_beta;
        let beta_max = ((b + 1) as f64) * d_beta;
        let beta_mid = 0.5 * (beta_min + beta_max);
        let bolt_center = [r_pcd_eff * beta_mid.cos(), r_pcd_eff * beta_mid.sin()];

        // Record global CCW rings at r_bore_eff and r_bolt_ring
        for m in 0..k_b {
            let t = (m as f64) / (k_b as f64);
            let ang = beta_min + t * d_beta;
            bore_ring.push([r_bore_eff * ang.cos(), r_bore_eff * ang.sin()]);
            bolt_outer_ring.push([r_bolt_ring * ang.cos(), r_bolt_ring * ang.sin()]);
        }

        // Build 4 * k_b perimeter points in CCW order around bolt_center:
        // Side 0: theta = beta_min, r from r_bore_eff to r_bolt_ring
        // Side 1: r = r_bolt_ring, theta from beta_min to beta_max
        // Side 2: theta = beta_max, r from r_bolt_ring to r_bore_eff
        // Side 3: r = r_bore_eff, theta from beta_max to beta_min
        let n_loop = 4 * k_b;
        let mut perim_loop: Vec<[f64; 2]> = Vec::with_capacity(n_loop);
        for m in 0..k_b {
            let t = (m as f64) / (k_b as f64);
            let r = r_bore_eff + t * (r_bolt_ring - r_bore_eff);
            perim_loop.push([r * beta_min.cos(), r * beta_min.sin()]);
        }
        for m in 0..k_b {
            let t = (m as f64) / (k_b as f64);
            let ang = beta_min + t * d_beta;
            perim_loop.push([r_bolt_ring * ang.cos(), r_bolt_ring * ang.sin()]);
        }
        for m in 0..k_b {
            let t = (m as f64) / (k_b as f64);
            let r = r_bolt_ring - t * (r_bolt_ring - r_bore_eff);
            perim_loop.push([r * beta_max.cos(), r * beta_max.sin()]);
        }
        for m in 0..k_b {
            let t = (m as f64) / (k_b as f64);
            let ang = beta_max - t * d_beta;
            perim_loop.push([r_bore_eff * ang.cos(), r_bore_eff * ang.sin()]);
        }

        // Build 2 * k_b bolt hole points and 2 * k_b midpoint ring points in matching CCW order around bolt_center
        let n_bolt = 2 * k_b;
        let mut hole_loop: Vec<[f64; 2]> = Vec::with_capacity(n_bolt);
        let mut mid_loop: Vec<[f64; 2]> = Vec::with_capacity(n_bolt);
        for j in 0..n_bolt {
            let ox = perim_loop[2 * j][0];
            let oy = perim_loop[2 * j][1];
            let dir_x = ox - bolt_center[0];
            let dir_y = oy - bolt_center[1];
            let d_len = dir_x.hypot(dir_y).max(1e-12);
            let u_x = dir_x / d_len;
            let u_y = dir_y / d_len;
            let ix = bolt_center[0] + r_bolt_eff * u_x;
            let iy = bolt_center[1] + r_bolt_eff * u_y;
            hole_loop.push([ix, iy]);
            mid_loop.push([0.5 * ix + 0.5 * ox, 0.5 * iy + 0.5 * oy]);
        }

        // Emit front (+Z) and back (-Z) hub annular sector caps using 2-to-1 halving transition
        // Note: perim_loop and mid_loop are CCW around bolt_center, so for +Z outward normal we pass Winding::Cw
        // to add_halving_ring_transition_2d (where Winding::Cw emits (O_{2i}, M_i, O_{2i+1}) -> CCW in XY!).
        add_halving_ring_transition_2d(&mut vertices, &mut indices, &perim_loop, &mid_loop, z_front, true);
        add_bridged_ring_strip_2d(&mut vertices, &mut indices, &mid_loop, &hole_loop, z_front, true);

        add_halving_ring_transition_2d(&mut vertices, &mut indices, &perim_loop, &mid_loop, z_back, false);
        add_bridged_ring_strip_2d(&mut vertices, &mut indices, &mid_loop, &hole_loop, z_back, false);

        // Bolt hole inner vertical wall with axial subdivision (normal pointing inward toward bolt_center)
        let bolt_chord = (2.0 * PI * r_bolt_eff / (n_bolt as f64)).max(1e-6);
        let bolt_z_steps = (((z_front - z_back) / bolt_chord).round() as usize).clamp(2, 8);
        for idx in 0..n_bolt {
            let next_idx = (idx + 1) % n_bolt;
            let h0 = hole_loop[idx];
            let h1 = hole_loop[next_idx];
            let dx = h1[0] - h0[0];
            let dy = h1[1] - h0[1];
            let len = dx.hypot(dy).max(1e-12);
            let n_in = [-dy / len, dx / len, 0.0];
            for iz in 0..bolt_z_steps {
                let tz0 = (iz as f64) / (bolt_z_steps as f64);
                let tz1 = ((iz + 1) as f64) / (bolt_z_steps as f64);
                let za = z_front + (z_back - z_front) * tz0;
                let zb = z_front + (z_back - z_front) * tz1;
                push_quad!(
                    [h1[0], h1[1], za],
                    [h1[0], h1[1], zb],
                    [h0[0], h0[1], zb],
                    [h0[0], h0[1], za],
                    n_in,
                );
            }
        }
    }

    // 6. Central bore inner cylindrical wall at r = r_bore_eff
    let n_bore = bore_ring.len();
    for i in 0..n_bore {
        let next_i = (i + 1) % n_bore;
        let b0 = bore_ring[i];
        let b1 = bore_ring[next_i];
        let dx = b1[0] - b0[0];
        let dy = b1[1] - b0[1];
        let len = dx.hypot(dy).max(1e-12);
        let n_in = [-dy / len, dx / len, 0.0];
        push_quad!(
            [b1[0], b1[1], z_front],
            [b1[0], b1[1], z_back],
            [b0[0], b0[1], z_back],
            [b0[0], b0[1], z_front],
            n_in,
        );
    }

    // 7. Zipper-stitch the hub annulus between bolt_outer_ring (at r_bolt_ring) and hub_outer_ring (at r_hub_eff)
    // Align starting index of hub_outer_ring closest to bolt_outer_ring[0] angle (0.0)
    let n1 = bolt_outer_ring.len();
    let n2 = hub_outer_ring.len();
    let ang1 = |idx: usize| -> f64 {
        let wraps = idx / n1;
        let rem = idx % n1;
        let p = bolt_outer_ring[rem];
        p[1].atan2(p[0]).rem_euclid(2.0 * PI) + (wraps as f64) * 2.0 * PI
    };
    let ang2 = |idx: usize| -> f64 {
        let wraps = idx / n2;
        let rem = idx % n2;
        phi_hub[rem] + (wraps as f64) * 2.0 * PI
    };

    let mut i1 = 0usize;
    let mut i2 = 0usize;
    while i1 < n1 || i2 < n2 {
        let advance_1 = if i1 >= n1 {
            false
        } else if i2 >= n2 {
            true
        } else {
            ang1(i1 + 1) <= ang2(i2 + 1)
        };

        let a_curr = bolt_outer_ring[i1 % n1];
        let b_curr = hub_outer_ring[i2 % n2];

        if advance_1 {
            let a_next = bolt_outer_ring[(i1 + 1) % n1];
            // Front cap (+Z): [a_curr, b_curr, a_next]
            let bf = vertices.len() as u32;
            vertices.push(Vertex::new([a_curr[0], a_curr[1], z_front], [0.0, 0.0, 1.0]));
            vertices.push(Vertex::new([b_curr[0], b_curr[1], z_front], [0.0, 0.0, 1.0]));
            vertices.push(Vertex::new([a_next[0], a_next[1], z_front], [0.0, 0.0, 1.0]));
            indices.push([bf, bf + 1, bf + 2]);

            // Back cap (-Z): [a_curr, a_next, b_curr]
            let bb = vertices.len() as u32;
            vertices.push(Vertex::new([a_curr[0], a_curr[1], z_back], [0.0, 0.0, -1.0]));
            vertices.push(Vertex::new([a_next[0], a_next[1], z_back], [0.0, 0.0, -1.0]));
            vertices.push(Vertex::new([b_curr[0], b_curr[1], z_back], [0.0, 0.0, -1.0]));
            indices.push([bb, bb + 1, bb + 2]);

            i1 += 1;
        } else {
            let b_next = hub_outer_ring[(i2 + 1) % n2];
            // Front cap (+Z): [a_curr, b_curr, b_next]
            let bf = vertices.len() as u32;
            vertices.push(Vertex::new([a_curr[0], a_curr[1], z_front], [0.0, 0.0, 1.0]));
            vertices.push(Vertex::new([b_curr[0], b_curr[1], z_front], [0.0, 0.0, 1.0]));
            vertices.push(Vertex::new([b_next[0], b_next[1], z_front], [0.0, 0.0, 1.0]));
            indices.push([bf, bf + 1, bf + 2]);

            // Back cap (-Z): [a_curr, b_next, b_curr]
            let bb = vertices.len() as u32;
            vertices.push(Vertex::new([a_curr[0], a_curr[1], z_back], [0.0, 0.0, -1.0]));
            vertices.push(Vertex::new([b_next[0], b_next[1], z_back], [0.0, 0.0, -1.0]));
            vertices.push(Vertex::new([b_curr[0], b_curr[1], z_back], [0.0, 0.0, -1.0]));
            indices.push([bb, bb + 1, bb + 2]);

            i2 += 1;
        }
    }

    let mut mesh = TriangleMesh::new(vertices, indices);
    mesh.face_ids = vec![0; mesh.indices.len()];
    mesh
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
        let cutter = create_helical_cutter(4.0, 1.5, 20.0, 40.0, 0.0, 2, true, 2.0, "circular");
        assert!(cutter.triangle_count() > 0);
        assert!(cutter.vertex_count() > 0);
        let vol = cutter.volume();
        assert!(vol > 0.0, "Helical cutter volume should be positive, got {}", vol);

        // Check AABB spans expected Z range ~ [0.0 - r_g, 40.0 + r_g]
        let aabb = cutter.bounding_box();
        assert!(aabb.min[2] < 1.0);
        assert!(aabb.max[2] > 39.0);
    }

    #[test]
    fn test_helical_cutter_metric_60() {
        let cutter = create_helical_cutter(4.0, 1.0, 2.0, 10.0, 0.0, 1, true, 5.0, "metric_60");
        assert!(cutter.triangle_count() > 0);
        let vol = cutter.volume();
        assert!(vol > 0.0, "Metric 60 cutter volume must be positive: got {}", vol);
    }

    #[test]
    fn test_polygon_prism_volume_and_flat_normals() {
        // Hexagon: width across flats S = 10.0, height H = 5.0
        // Exact volume = (sqrt(3) / 2) * S^2 * H = (sqrt(3) / 2) * 100 * 5 = 250 * sqrt(3) ~ 433.01270189
        let prism = create_polygon_prism(6, 5.0, Some(10.0), None, None);
        let vol = prism.volume();
        let expected_vol = (3.0_f64.sqrt() / 2.0) * 100.0 * 5.0;
        let diff = (vol - expected_vol).abs();
        assert!(
            diff < 1e-4,
            "Hexagonal prism volume should be {}, got {}",
            expected_vol,
            vol
        );
        assert_eq!(prism.triangle_count(), 24); // 6 top + 6 bot + 12 sides

        // Test with DIN 934 conical chamfer
        let chamfered = create_polygon_prism(6, 5.0, Some(10.0), None, Some(0.5));
        let vol_chamfered = chamfered.volume();
        assert!(
            vol_chamfered < vol,
            "Chamfered prism volume ({}) must be less than unchamfered ({})",
            vol_chamfered,
            vol
        );
        assert!(
            vol_chamfered > vol * 0.8,
            "Chamfered prism volume should retain most of original body: got {}",
            vol_chamfered
        );
    }

    #[test]
    fn test_spur_gear_solid_volume_and_dimensions() {
        let m = 0.002;
        let z = 20;
        let b = 0.015;
        let gear = create_spur_gear(m, z, 20.0, b, None, None, None, None, None);
        assert!(gear.triangle_count() > 0);
        let vol = gear.volume();
        assert!(vol > 0.0, "Spur gear volume should be positive: got {}", vol);

        // Approximate reference volume: pitch cylinder V = pi * r^2 * b
        let r_pitch = m * (z as f64) * 0.5; // 0.020 m
        let v_pitch_cyl = PI * r_pitch * r_pitch * b;
        let diff_ratio = (vol - v_pitch_cyl).abs() / v_pitch_cyl;
        assert!(
            diff_ratio < 0.12,
            "Gear volume ({}) should be close to pitch cylinder volume ({}), diff: {}",
            vol,
            v_pitch_cyl,
            diff_ratio
        );

        // AABB dimensions: Z dimension should match face width, X and Y ~ tip diameter 2 * (r + m) = 0.044 m
        let aabb = gear.bounding_box();
        let dims = aabb.dimensions();
        assert!((dims[2] - b).abs() < 1e-6, "Z dimension must match face width");
        let d_tip = 2.0 * (r_pitch + m);
        assert!(
            (dims[0] - d_tip).abs() < 0.002,
            "X diameter should be close to tip diameter {}, got {}",
            d_tip,
            dims[0]
        );
        assert!(
            (dims[1] - d_tip).abs() < 0.002,
            "Y diameter should be close to tip diameter {}, got {}",
            d_tip,
            dims[1]
        );
    }

    #[test]
    fn test_spur_gear_with_hub_bore_and_keyway() {
        let m = 0.002;
        let z = 20;
        let b = 0.015;
        let gear_plain = create_spur_gear(m, z, 20.0, b, None, None, None, None, None);
        let _vol_plain = gear_plain.volume();

        // Gear with hub (dia 25mm, length 10mm -> total length 25mm), bore 12mm (r=6mm), keyway 4x1.8mm
        let gear_with_hub_and_keyway = create_spur_gear(
            m,
            z,
            20.0,
            b,
            Some(0.006),
            Some(0.004),
            Some(0.0018),
            Some(0.025),
            Some(0.010),
        );
        assert!(gear_with_hub_and_keyway.triangle_count() > 0);
        let vol_hub_gear = gear_with_hub_and_keyway.volume();
        assert!(vol_hub_gear > 0.0, "Hub gear volume must be positive: got {}", vol_hub_gear);

        // Check AABB spans total hub length 0.025m
        let aabb = gear_with_hub_and_keyway.bounding_box();
        assert!((aabb.dimensions()[2] - 0.025).abs() < 1e-4);

        // Test without keyway vs with keyway: keyway should reduce volume
        let gear_without_keyway = create_spur_gear(
            m,
            z,
            20.0,
            b,
            Some(0.006),
            None,
            None,
            Some(0.025),
            Some(0.010),
        );
        let vol_no_key = gear_without_keyway.volume();
        assert!(
            vol_hub_gear < vol_no_key,
            "Keyway must reduce volume: with_key={}, without_key={}",
            vol_hub_gear,
            vol_no_key
        );
    }

    #[test]
    fn test_cut_hole_with_keyway_reduces_volume() {
        let block = create_box(10.0, 10.0, 10.0);
        let hole_plain = cut_hole(&block, [0.0, 5.0, 0.0], [0.0, 1.0, 0.0], 2.0, 10.0, None, None);
        let hole_keyway = cut_hole(
            &block,
            [0.0, 5.0, 0.0],
            [0.0, 1.0, 0.0],
            2.0,
            10.0,
            Some(1.0),
            Some(1.0),
        );

        let v_plain = hole_plain.volume();
        let v_keyway = hole_keyway.volume();
        assert!(
            v_keyway < v_plain,
            "Hole with keyway volume ({}) must be less than plain hole ({})",
            v_keyway,
            v_plain
        );
    }

    #[test]
    fn test_revolve_cylinder_volume_and_aabb() {
        let r = 2.0;
        let h = 2.0;
        let profile = vec![
            [0.0, -1.0],
            [r, -1.0],
            [r, 1.0],
            [0.0, 1.0],
        ];
        let cyl_revolve = create_revolve(&profile, 360.0, 32);
        assert!(cyl_revolve.triangle_count() > 0);
        let vol = cyl_revolve.volume();
        let expected_vol = PI * r * r * h;
        let diff_ratio = (vol - expected_vol).abs() / expected_vol;
        assert!(
            diff_ratio < 0.03,
            "Revolved cylinder volume {} should be close to analytical {}, diff: {}",
            vol,
            expected_vol,
            diff_ratio
        );

        let aabb = cyl_revolve.bounding_box();
        let dims = aabb.dimensions();
        assert!((dims[0] - 2.0 * r).abs() < 0.1);
        assert!((dims[1] - 2.0 * r).abs() < 0.1);
        assert!((dims[2] - h).abs() < 1e-4);
    }

    #[test]
    fn test_revolve_partial_angle_and_hollow_ring() {
        // 180-degree half cylinder
        let profile = vec![
            [0.0, -1.0],
            [2.0, -1.0],
            [2.0, 1.0],
            [0.0, 1.0],
        ];
        let half_cyl = create_revolve(&profile, 180.0, 32);
        let vol_half = half_cyl.volume();
        let expected_half = 0.5 * PI * 4.0 * 2.0;
        let diff_ratio = (vol_half - expected_half).abs() / expected_half;
        assert!(
            diff_ratio < 0.03,
            "Half revolved cylinder volume {} should be close to {}, diff: {}",
            vol_half,
            expected_half,
            diff_ratio
        );

        // Hollow ring/pipe: r in [1, 2], h = 2
        let ring_profile = vec![
            [1.0, -1.0],
            [2.0, -1.0],
            [2.0, 1.0],
            [1.0, 1.0],
        ];
        let ring = create_revolve(&ring_profile, 360.0, 32);
        let vol_ring = ring.volume();
        let expected_ring = PI * (4.0 - 1.0) * 2.0;
        let ring_diff_ratio = (vol_ring - expected_ring).abs() / expected_ring;
        assert!(
            ring_diff_ratio < 0.03,
            "Hollow ring volume {} should be close to {}, diff: {}",
            vol_ring,
            expected_ring,
            ring_diff_ratio
        );
    }

    #[test]
    fn test_wheel_rim_parametric_solid_and_dimensions() {
        let rim_d = 0.4572; // 18 inches
        let rim_w = 0.2032; // 8 inches
        let wheel = create_wheel_rim(
            rim_d,
            rim_w,
            Some(0.490),
            Some(0.160),
            Some(0.066),
            Some(0.1143),
            Some(5),
            Some(0.014),
            Some(5),
            Some(0.028),
            Some(0.018),
            Some(0.035),
        );

        assert!(wheel.triangle_count() > 0, "Wheel rim should have triangles");
        let vol = wheel.volume();
        assert!(vol > 0.001, "Wheel rim volume should be positive: got {}", vol);

        let aabb = wheel.bounding_box();
        let dims = aabb.dimensions();
        // Z dimension should match rim width
        assert!(
            (dims[2] - rim_w).abs() < 0.01,
            "Wheel Z span {} should be close to rim width {}",
            dims[2],
            rim_w
        );
        // X and Y dimensions should be close to flange diameter (0.490m)
        assert!(
            (dims[0] - 0.490).abs() < 0.02,
            "Wheel X diameter {} should be close to flange diameter 0.490",
            dims[0]
        );
        assert!(
            (dims[1] - 0.490).abs() < 0.02,
            "Wheel Y diameter {} should be close to flange diameter 0.490",
            dims[1]
        );
    }

    fn verify_cap_normals_and_validation(mesh: &TriangleMesh, z_top: Option<f64>, z_bot: Option<f64>) {
        let mut top_count = 0usize;
        let mut bot_count = 0usize;
        for tri in &mesh.indices {
            let p0 = mesh.vertices[tri[0] as usize].pos_vec();
            let p1 = mesh.vertices[tri[1] as usize].pos_vec();
            let p2 = mesh.vertices[tri[2] as usize].pos_vec();
            let n_geo = (p1 - p0).cross(&(p2 - p0));
            if let Some(zt) = z_top {
                if (p0.z - zt).abs() < 1e-6 && (p1.z - zt).abs() < 1e-6 && (p2.z - zt).abs() < 1e-6 {
                    assert!(
                        n_geo.z > 0.0,
                        "Top cap triangle at z={} has inward geometric normal {:?}",
                        zt,
                        n_geo
                    );
                    top_count += 1;
                }
            }
            if let Some(zb) = z_bot {
                if (p0.z - zb).abs() < 1e-6 && (p1.z - zb).abs() < 1e-6 && (p2.z - zb).abs() < 1e-6 {
                    assert!(
                        n_geo.z < 0.0,
                        "Bottom cap triangle at z={} has inward geometric normal {:?}",
                        zb,
                        n_geo
                    );
                    bot_count += 1;
                }
            }
        }
        if z_top.is_some() {
            assert!(top_count > 0, "Expected at least one top cap triangle");
        }
        if z_bot.is_some() {
            assert!(bot_count > 0, "Expected at least one bottom cap triangle");
        }
        let report = crate::diagnostics::validate_mesh(mesh, None);
        assert!(
            report.topology.orientation_consistent,
            "validate_mesh reported inconsistent orientation: {:?}",
            report.issues
        );
        assert_eq!(report.topology.inconsistent_edges_count, 0);
        assert_eq!(report.status, "PASS", "validate_mesh failed: {:?}", report.issues);
    }

    #[test]
    fn cap_orientation_cylinder() {
        let r = 1.5;
        let h = 5.0;
        let cyl = create_cylinder(r, h, 64);
        verify_cap_normals_and_validation(&cyl, Some(h * 0.5), Some(-h * 0.5));
        let expected_vol = PI * r * r * h;
        let signed_vol = cyl.signed_volume();
        assert!(
            signed_vol > 0.0 && (signed_vol - expected_vol).abs() / expected_vol < 0.01,
            "Cylinder signed_volume {} must be within 1% of analytical {}",
            signed_vol,
            expected_vol
        );
    }

    #[test]
    fn cap_orientation_cone_sharp() {
        let r = 3.0;
        let h = 6.0;
        let cone = create_cone(r, 0.0, h, 64);
        verify_cap_normals_and_validation(&cone, None, Some(-h * 0.5));
        let expected_vol = (1.0 / 3.0) * PI * r * r * h;
        let signed_vol = cone.signed_volume();
        assert!(
            signed_vol > 0.0 && (signed_vol - expected_vol).abs() / expected_vol < 0.01,
            "Sharp cone signed_volume {} must be within 1% of analytical {}",
            signed_vol,
            expected_vol
        );
    }

    #[test]
    fn cap_orientation_cone_frustum() {
        let r_bot = 4.0;
        let r_top = 2.0;
        let h = 5.0;
        let frustum = create_cone(r_bot, r_top, h, 64);
        verify_cap_normals_and_validation(&frustum, Some(h * 0.5), Some(-h * 0.5));
        let expected_vol = (1.0 / 3.0) * PI * h * (r_bot * r_bot + r_bot * r_top + r_top * r_top);
        let signed_vol = frustum.signed_volume();
        assert!(
            signed_vol > 0.0 && (signed_vol - expected_vol).abs() / expected_vol < 0.01,
            "Frustum cone signed_volume {} must be within 1% of analytical {}",
            signed_vol,
            expected_vol
        );
    }

    #[test]
    fn cap_orientation_spur_gear() {
        let m = 0.002;
        let teeth = 20usize;
        let pa_deg = 20.0;
        let b = 0.015;
        let gear = create_spur_gear(m, teeth, pa_deg, b, None, None, None, None, None);
        verify_cap_normals_and_validation(&gear, Some(b * 0.5), Some(-b * 0.5));

        // Analytical volume of extruded 2D gear profile via divergence theorem on top cap
        // (verified against polar integration of root disk + involute teeth)
        let z = teeth as f64;
        let r = m * z * 0.5;
        let r_a = r + m;
        let r_f = (r - 1.25 * m).max(0.1 * m);
        let alpha = pa_deg * PI / 180.0;
        let r_b = r * alpha.cos();
        let psi = PI / (2.0 * z);
        let trough_half = PI / z;
        let inv_alpha = alpha.tan() - alpha;
        let min_tip = (trough_half * 0.15).max(1e-4);
        let theta_at_r = |rad: f64| -> f64 {
            let raw = if rad < r_b {
                psi + inv_alpha
            } else {
                let alpha_r = (r_b / rad).clamp(0.0, 1.0).acos();
                (psi + inv_alpha - (alpha_r.tan() - alpha_r)).max(min_tip)
            };
            raw.min(trough_half * 0.70)
        };
        let theta_base = theta_at_r(r_b);
        let theta_root = if r_f < r_b - 1e-9 {
            (theta_base + 0.25 * (trough_half - theta_base).max(1e-4)).min(trough_half * 0.75)
        } else {
            theta_at_r(r_f)
        };
        // Exact polar integral of A = pi * r_f^2 + 2 * z * \int_{r_f}^{r_a} r * theta(r) dr
        let n_int = 1000usize;
        let mut integral = 0.0;
        for i in 0..n_int {
            let r_mid = r_f + ((i as f64) + 0.5) * (r_a - r_f) / (n_int as f64);
            let th = if r_mid < r_b && r_f < r_b - 1e-9 {
                let t = (r_mid - r_f) / (r_b - r_f);
                theta_root + t * (theta_base - theta_root)
            } else {
                theta_at_r(r_mid)
            };
            integral += r_mid * th * (r_a - r_f) / (n_int as f64);
        }
        let analytical_area = PI * r_f * r_f + 2.0 * z * integral;
        let expected_vol = analytical_area * b;
        let signed_vol = gear.signed_volume();
        assert!(
            signed_vol > 0.0 && (signed_vol - expected_vol).abs() / expected_vol < 0.01,
            "Spur gear signed_volume {} must be within 1% of analytical {}",
            signed_vol,
            expected_vol
        );
    }

    /// Computes `(area, r_centroid)` of a simple 2D polygon in the `(r, z)` plane for Pappus's theorem.
    fn compute_polygon_area_and_r_centroid(profile: &[[f64; 2]]) -> (f64, f64) {
        let n = profile.len();
        let mut signed_area2 = 0.0;
        let mut r_moment6 = 0.0;
        for i in 0..n {
            let p0 = profile[i];
            let p1 = profile[(i + 1) % n];
            let cross = p0[0] * p1[1] - p1[0] * p0[1];
            signed_area2 += cross;
            r_moment6 += (p0[0] + p1[0]) * cross;
        }
        let area = 0.5 * signed_area2.abs();
        let r_c = (r_moment6 / (3.0 * signed_area2)).abs();
        (area, r_c)
    }

    #[test]
    fn test_revolve_partial_angles_validation_and_pappus_volume() {
        let rect_hollow = vec![[0.01, -0.02], [0.03, -0.02], [0.03, 0.02], [0.01, 0.02]];
        let rect_solid = vec![[0.0, -0.02], [0.03, -0.02], [0.03, 0.02], [0.0, 0.02]];
        let l_hollow = vec![
            [0.01, -0.02],
            [0.04, -0.02],
            [0.04, -0.01],
            [0.02, -0.01],
            [0.02, 0.02],
            [0.01, 0.02],
        ];
        let l_solid = vec![
            [0.0, -0.02],
            [0.04, -0.02],
            [0.04, -0.01],
            [0.02, -0.01],
            [0.02, 0.02],
            [0.0, 0.02],
        ];

        let profiles: [(&str, &[[f64; 2]], i32); 4] = [
            ("rect_hollow", &rect_hollow, 0),
            ("rect_solid", &rect_solid, 0),
            ("l_hollow", &l_hollow, 0),
            ("l_solid", &l_solid, 0),
        ];

        let segs = 32usize;
        for &(name, profile, partial_genus) in &profiles {
            let (area, r_c) = compute_polygon_area_and_r_centroid(profile);
            assert!(area > 0.0 && r_c > 0.0);

            for &angle_deg in &[30.0_f64, 90.0, 180.0, 270.0, 360.0] {
                let expected_genus = if (angle_deg - 360.0).abs() < 1e-5 {
                    if name.contains("hollow") { 1 } else { 0 }
                } else {
                    partial_genus
                };

                let mesh = create_revolve(profile, angle_deg, segs);
                let report = crate::diagnostics::validate_mesh(&mesh, Some(expected_genus));
                assert_eq!(
                    report.status, "PASS",
                    "Revolve {name} at {angle_deg}° failed validate_mesh: issues={:?}",
                    report.issues
                );
                assert!(
                    report.angle_stats.min_angle_deg >= 1.0,
                    "Revolve {name} at {angle_deg}° has min_angle={}° < 1°",
                    report.angle_stats.min_angle_deg
                );
                assert_eq!(report.geometry.lt_1_deg_count, 0);
                assert!(
                    report.geometry.lt_5_deg_ratio < 0.01,
                    "Revolve {name} at {angle_deg}° has lt_5_deg_ratio={} >= 1%",
                    report.geometry.lt_5_deg_ratio
                );

                // Pappus's centroid theorem: V = theta * r_c * A
                let theta_rad = angle_deg * PI / 180.0;
                let v_pappus = theta_rad * r_c * area;
                let dtheta = theta_rad / (segs as f64);
                let arc_poly_factor = dtheta.sin() / dtheta;
                let v_pappus_poly = v_pappus * arc_poly_factor;

                let signed_vol = mesh.signed_volume();
                let rel_err_pappus = (signed_vol - v_pappus).abs() / v_pappus;
                let rel_err_poly = (signed_vol - v_pappus_poly).abs() / v_pappus_poly;

                assert!(
                    rel_err_pappus < 0.01,
                    "Revolve {name} at {angle_deg}° signed_volume={signed_vol} differs from Pappus volume {v_pappus} by {:.4}% (> 1%)",
                    rel_err_pappus * 100.0
                );
                assert!(
                    rel_err_poly < 1e-6,
                    "Revolve {name} at {angle_deg}° signed_volume={signed_vol} differs from polygonal Pappus volume {v_pappus_poly} by {rel_err_poly}"
                );
            }
        }
    }

    #[test]
    fn test_wheel_rim_small_diameters_and_extreme_ratios_no_panic_and_valid() {
        // Regression test for P2-T15: previously rim_diameter < 0.1403 m panicked in f64::clamp(min, max)
        let small_cases = [
            (0.005, 0.0025), // 5 mm x 2.5 mm RC micro wheel
            (0.020, 0.010),  // 20 mm x 10 mm miniature wheel
            (0.050, 0.025),  // 50 mm x 25 mm model car wheel
            (0.060, 0.030),  // 60 mm x 30 mm small wheel
            (0.100, 0.050),  // 100 mm x 50 mm kart/scooter wheel
            (0.120, 0.060),  // 120 mm x 60 mm small wheel
            (0.140, 0.065),  // 140 mm x 65 mm threshold wheel
        ];


        for &(d_rim, w_rim) in &small_cases {
            let mesh = create_wheel_rim(
                d_rim,
                w_rim,
                None,
                None,
                None,
                None,
                Some(5),
                None,
                Some(5),
                None,
                None,
                None,
            );
            assert!(
                !mesh.is_empty(),
                "Wheel rim ({d_rim} m, {w_rim} m) must not be empty"
            );
            // Expected genus = 1 (center bore) + 5 (bolt holes) + 5 (ventilation windows) = 11
            let report = crate::diagnostics::validate_mesh(&mesh, Some(11));
            assert_eq!(
                report.status, "PASS",
                "Wheel rim ({d_rim} m, {w_rim} m) failed validate_mesh: issues={:?}",
                report.issues
            );
            assert!(report.topology.is_watertight);
            assert_eq!(report.topology.non_manifold_edges_count, 0);
            assert_eq!(report.topology.non_manifold_vertices_count, 0);
            assert_eq!(report.geometry.lt_1_deg_count, 0);
            assert!(report.geometry.lt_5_deg_ratio <= 0.01);
        }
    }
}
