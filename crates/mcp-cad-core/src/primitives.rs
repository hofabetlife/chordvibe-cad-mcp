//! 3D solid primitive generators: Box, Cylinder, Sphere, and CutHole.

use std::f64::consts::PI;
use nalgebra::{Rotation3, Vector3};
use crate::csg::{csg_difference, csg_intersection};
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

/// Helper to generate a convex turned solid for DIN 934 conical chamfering.
///
/// Features conical end caps tapering at chamfer angle (~30 deg per DIN 934)
/// and a central cylindrical body enclosing the polygonal prism.
fn create_turned_chamfer_solid(
    circumradius: f64,
    height: f64,
    chamfer: f64,
    sides: usize,
) -> TriangleMesh {
    let segs = (sides * 4).max(32);
    let h2 = height * 0.5;

    // DIN 934 chamfer angle: 30 degrees to the horizontal bearing face
    let tan_alpha = (PI / 6.0).tan(); // tan(30 deg) = 1 / sqrt(3) ~ 0.57735

    let mut c_r = chamfer;
    let mut c_z = c_r * tan_alpha;

    if c_z > height * 0.45 {
        c_z = height * 0.45;
        c_r = c_z / tan_alpha;
    }

    let r_top = (circumradius - c_r).max(circumradius * 0.1);
    let r_outer = circumradius * 2.0;

    // Small axial margin to prevent coplanar surface artifacts
    let delta = (c_z * 0.05).clamp(1e-4, 0.01);
    let z_bot_cap = -h2 - delta;
    let z_bot_shoulder = -h2 + c_z;
    let z_top_shoulder = h2 - c_z;
    let z_top_cap = h2 + delta;

    let r_top_cap = (r_top - delta / tan_alpha).max(r_top * 0.5);
    let r_bot_cap = r_top_cap;

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // 1. Bottom cap at z_bot_cap
    let bot_center_idx = vertices.len() as u32;
    vertices.push(Vertex::new([0.0, 0.0, z_bot_cap], [0.0, 0.0, -1.0]));

    let bot_rim_start = vertices.len() as u32;
    for i in 0..segs {
        let theta = (i as f64) * 2.0 * PI / (segs as f64);
        vertices.push(Vertex::new(
            [r_bot_cap * theta.cos(), r_bot_cap * theta.sin(), z_bot_cap],
            [0.0, 0.0, -1.0],
        ));
    }
    for i in 0..segs {
        let next = (i + 1) % segs;
        indices.push([bot_center_idx, bot_rim_start + (next as u32), bot_rim_start + (i as u32)]);
    }

    // 2. Top cap at z_top_cap
    let top_center_idx = vertices.len() as u32;
    vertices.push(Vertex::new([0.0, 0.0, z_top_cap], [0.0, 0.0, 1.0]));

    let top_rim_start = vertices.len() as u32;
    for i in 0..segs {
        let theta = (i as f64) * 2.0 * PI / (segs as f64);
        vertices.push(Vertex::new(
            [r_top_cap * theta.cos(), r_top_cap * theta.sin(), z_top_cap],
            [0.0, 0.0, 1.0],
        ));
    }
    for i in 0..segs {
        let next = (i + 1) % segs;
        indices.push([top_center_idx, top_rim_start + (i as u32), top_rim_start + (next as u32)]);
    }

    // Helper closure to loft a frustum band between two rings
    let mut add_band = |r_lower: f64, z_lower: f64, r_upper: f64, z_upper: f64, n_lower: [f64; 2], n_upper: [f64; 2]| {
        let band_start = vertices.len() as u32;
        for i in 0..segs {
            let theta = (i as f64) * 2.0 * PI / (segs as f64);
            let cos_t = theta.cos();
            let sin_t = theta.sin();

            // Lower vertex: index band_start + 2*i
            vertices.push(Vertex::new(
                [r_lower * cos_t, r_lower * sin_t, z_lower],
                [n_lower[0] * cos_t, n_lower[0] * sin_t, n_lower[1]],
            ));
            // Upper vertex: index band_start + 2*i + 1
            vertices.push(Vertex::new(
                [r_upper * cos_t, r_upper * sin_t, z_upper],
                [n_upper[0] * cos_t, n_upper[0] * sin_t, n_upper[1]],
            ));
        }

        for i in 0..segs {
            let next = (i + 1) % segs;
            let curr_low = band_start + (i as u32) * 2;
            let curr_up = curr_low + 1;
            let next_low = band_start + (next as u32) * 2;
            let next_up = next_low + 1;

            indices.push([curr_low, next_low, next_up]);
            indices.push([curr_low, next_up, curr_up]);
        }
    };

    // Bottom cone frustum (from z_bot_cap to z_bot_shoulder)
    let dr_bot = r_outer - r_bot_cap;
    let dz_bot = z_bot_shoulder - z_bot_cap;
    let slant_bot = (dr_bot * dr_bot + dz_bot * dz_bot).sqrt();
    let norm_bot = [dz_bot / slant_bot, -dr_bot / slant_bot];
    add_band(r_bot_cap, z_bot_cap, r_outer, z_bot_shoulder, norm_bot, norm_bot);

    // Middle cylinder (from z_bot_shoulder to z_top_shoulder)
    add_band(r_outer, z_bot_shoulder, r_outer, z_top_shoulder, [1.0, 0.0], [1.0, 0.0]);

    // Top cone frustum (from z_top_shoulder to z_top_cap)
    let dr_top = r_outer - r_top_cap;
    let dz_top = z_top_cap - z_top_shoulder;
    let slant_top = (dr_top * dr_top + dz_top * dz_top).sqrt();
    let norm_top = [dz_top / slant_top, dr_top / slant_top];
    add_band(r_outer, z_top_shoulder, r_top_cap, z_top_cap, norm_top, norm_top);

    TriangleMesh::new(vertices, indices)
}

/// Generates a solid regular polygonal prism centered at origin along Z axis.
///
/// Parameters:
/// - `sides`: number of facets (minimum 3, default 6 for standard hexagon)
/// - `height`: prism height along Z axis
/// - `width_across_flats`: optional flat-to-flat distance (S, e.g. wrench size for nuts/bolts)
/// - `radius`: optional circumscribed circle radius (distance from center to vertices)
/// - `chamfer`: optional conical chamfer size at top and bottom faces (DIN 934 30-degree chamfer)
///
/// Features sharp edges and flat facet normals on all side faces and caps.
pub fn create_polygon_prism(
    sides: usize,
    height: f64,
    width_across_flats: Option<f64>,
    radius: Option<f64>,
    chamfer: Option<f64>,
) -> TriangleMesh {
    let n = sides.max(3);
    if height <= 1e-12 {
        return TriangleMesh::empty();
    }

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

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // 1. Top cap (z = +h2, flat normal = [0, 0, 1])
    let top_center_idx = vertices.len() as u32;
    vertices.push(Vertex::new([0.0, 0.0, h2], [0.0, 0.0, 1.0]));

    let top_rim_start = vertices.len() as u32;
    for i in 0..n {
        let theta = (i as f64) * angle_step;
        let x = circumradius * theta.cos();
        let y = circumradius * theta.sin();
        vertices.push(Vertex::new([x, y, h2], [0.0, 0.0, 1.0]));
    }

    for i in 0..n {
        let next = (i + 1) % n;
        indices.push([
            top_center_idx,
            top_rim_start + (i as u32),
            top_rim_start + (next as u32),
        ]);
    }

    // 2. Bottom cap (z = -h2, flat normal = [0, 0, -1])
    let bot_center_idx = vertices.len() as u32;
    vertices.push(Vertex::new([0.0, 0.0, -h2], [0.0, 0.0, -1.0]));

    let bot_rim_start = vertices.len() as u32;
    for i in 0..n {
        let theta = (i as f64) * angle_step;
        let x = circumradius * theta.cos();
        let y = circumradius * theta.sin();
        vertices.push(Vertex::new([x, y, -h2], [0.0, 0.0, -1.0]));
    }

    for i in 0..n {
        let next = (i + 1) % n;
        indices.push([
            bot_center_idx,
            bot_rim_start + (next as u32),
            bot_rim_start + (i as u32),
        ]);
    }

    // 3. Side wall quads with flat facet normals (sharp edges)
    for i in 0..n {
        let next = (i + 1) % n;
        let t1 = (i as f64) * angle_step;
        let t2 = (next as f64) * angle_step;

        let x1 = circumradius * t1.cos();
        let y1 = circumradius * t1.sin();
        let x2 = circumradius * t2.cos();
        let y2 = circumradius * t2.sin();

        let dx = x2 - x1;
        let dy = y2 - y1;
        let edge_len = (dx * dx + dy * dy).sqrt();
        let normal = if edge_len > 1e-12 {
            [dy / edge_len, -dx / edge_len, 0.0]
        } else {
            [1.0, 0.0, 0.0]
        };

        let base = vertices.len() as u32;
        vertices.push(Vertex::new([x1, y1, h2], normal));  // 0: top curr
        vertices.push(Vertex::new([x1, y1, -h2], normal)); // 1: bot curr
        vertices.push(Vertex::new([x2, y2, -h2], normal)); // 2: bot next
        vertices.push(Vertex::new([x2, y2, h2], normal));  // 3: top next

        indices.push([base, base + 1, base + 3]);
        indices.push([base + 1, base + 2, base + 3]);
    }

    let prism = TriangleMesh::new(vertices, indices);

    // Conical end chamfering per DIN 934 (30-degree chamfer)
    if let Some(c) = chamfer {
        if c > 1e-6 {
            let chamfer_body = create_turned_chamfer_solid(circumradius, height, c, n);
            return csg_intersection(&prism, &chamfer_body);
        }
    }

    prism
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

            let is_metric = profile_type.eq_ignore_ascii_case("metric_60");
            if is_metric {
                // ISO 68-1 60-degree triangular metric thread profile (rhombus cutter)
                // Flank angle is 30 deg to thread axis: tan(30) = 1 / sqrt(3)
                let tan30 = (PI / 6.0).tan();
                let w = (groove_radius * tan30).min(0.48 * pitch);
                let h_eff = w / tan30;

                // 4 corners of 60-deg diamond in (e_r, e_b) plane:
                // (+h_eff, 0) -> (0, +w) -> (-h_eff, 0) -> (0, -w)
                let segs_per_flank = cross_segments / 4;
                for flank in 0..4 {
                    let (p_start, p_end, n_flank) = match flank {
                        0 => (
                            Vector3::new(h_eff, 0.0, 0.0),
                            Vector3::new(0.0, w, 0.0),
                            Vector3::new(w, h_eff, 0.0).normalize(),
                        ),
                        1 => (
                            Vector3::new(0.0, w, 0.0),
                            Vector3::new(-h_eff, 0.0, 0.0),
                            Vector3::new(-w, h_eff, 0.0).normalize(),
                        ),
                        2 => (
                            Vector3::new(-h_eff, 0.0, 0.0),
                            Vector3::new(0.0, -w, 0.0),
                            Vector3::new(-w, -h_eff, 0.0).normalize(),
                        ),
                        _ => (
                            Vector3::new(0.0, -w, 0.0),
                            Vector3::new(h_eff, 0.0, 0.0),
                            Vector3::new(w, -h_eff, 0.0).normalize(),
                        ),
                    };

                    for k in 0..segs_per_flank {
                        let frac = (k as f64) / (segs_per_flank as f64);
                        let r_coord = p_start.x + (p_end.x - p_start.x) * frac;
                        let b_coord = p_start.y + (p_end.y - p_start.y) * frac;

                        let pos = center + e_r * r_coord + e_b * b_coord;
                        let n = (e_r * n_flank.x + e_b * n_flank.y).normalize();
                        all_vertices.push(Vertex::new([pos.x, pos.y, pos.z], [n.x, n.y, n.z]));
                    }
                }
            } else {
                for i in 0..cross_segments {
                    let alpha = (i as f64) * 2.0 * PI / (cross_segments as f64);
                    let cos_a = alpha.cos();
                    let sin_a = alpha.sin();

                    let n = e_r * cos_a + e_b * sin_a;
                    let pos = center + n * groove_radius;

                    all_vertices.push(Vertex::new([pos.x, pos.y, pos.z], [n.x, n.y, n.z]));
                }
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
}
