//! 3D solid primitive generators: Box, Cylinder, Sphere, and CutHole.

use std::f64::consts::PI;
use nalgebra::{Matrix3, Rotation3, Vector3};
use crate::csg::{csg_difference, csg_intersection, csg_union};
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

    let cyl_tool = create_cylinder_oriented(
        [tool_center.x, tool_center.y, tool_center.z],
        [drill_dir.x, drill_dir.y, drill_dir.z],
        radius,
        tool_height,
        32,
    );

    let final_tool = if let (Some(kw), Some(kd)) = (keyway_width, keyway_depth) {
        if kw > 1e-6 && kd > 1e-6 {
            // Find orthonormal basis (u, v, w) where w = drill_dir
            let w = drill_dir;
            let up = if w.y.abs() < 0.9 {
                Vector3::new(0.0, 1.0, 0.0)
            } else {
                Vector3::new(0.0, 0.0, 1.0)
            };
            let u = w.cross(&up).normalize();
            let v = w.cross(&u).normalize();

            // DIN 6885 keyway slot:
            // width kw along u, radial depth (radius + kd) along v, length tool_height along w.
            let mut box_mesh = create_box(kw, radius + kd, tool_height);
            let rot_mat = Matrix3::from_columns(&[u, v, w]);
            let box_center = tool_center + v * ((radius + kd) * 0.5);

            for vert in &mut box_mesh.vertices {
                let p = Vector3::new(vert.position[0], vert.position[1], vert.position[2]);
                let n = Vector3::new(vert.normal[0], vert.normal[1], vert.normal[2]);
                let world_p = box_center + rot_mat * p;
                let world_n = rot_mat * n;
                vert.position = [world_p.x, world_p.y, world_p.z];
                vert.normal = [world_n.x, world_n.y, world_n.z];
            }

            csg_union(&cyl_tool, &box_mesh)
        } else {
            cyl_tool
        }
    } else {
        cyl_tool
    };

    csg_difference(mesh, &final_tool)
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
    let r_f = r - 1.25 * module;
    let r_b = r * alpha.cos();
    let h2 = face_width * 0.5;

    // Pitch circle tooth thickness half-angle: psi = pi / (2 * z)
    let psi = PI / (2.0 * z);
    // Involute function at pressure angle alpha: inv(alpha) = tan(alpha) - alpha
    let inv_alpha = alpha.tan() - alpha;

    // Angle theta from tooth centerline at radius R >= r_b
    let theta_at_r = |radius: f64| -> f64 {
        if radius < r_b {
            psi + inv_alpha
        } else {
            let cos_ar = (r_b / radius).clamp(0.0, 1.0);
            let alpha_r = cos_ar.acos();
            let inv_alpha_r = alpha_r.tan() - alpha_r;
            (psi + inv_alpha - inv_alpha_r).max(1e-6)
        }
    };

    let theta_base = psi + inv_alpha;
    let theta_root = if r_f < r_b { theta_base } else { theta_at_r(r_f) };

    // 1. Construct 2D contour points in CCW order around origin
    let mut poly_2d: Vec<[f64; 2]> = Vec::new();
    let n_flank_steps = 4;
    let r_inv_start = r_b.max(r_f);

    for k in 0..teeth {
        let phi_k = (k as f64) * 2.0 * PI / z;

        // 1a. Leading flank root point
        let phi_root_lead = phi_k - theta_root;
        poly_2d.push([r_f * phi_root_lead.cos(), r_f * phi_root_lead.sin()]);

        // 1b. If r_f < r_b, radial line from r_f to r_b on leading flank
        if r_f < r_b - 1e-9 {
            let phi_base_lead = phi_k - theta_base;
            poly_2d.push([r_b * phi_base_lead.cos(), r_b * phi_base_lead.sin()]);
        }

        // 1c. Involute points on leading flank from r_inv_start up to r_a
        for s_idx in 1..=n_flank_steps {
            let s = (s_idx as f64) / (n_flank_steps as f64);
            let r_curr = r_inv_start + s * (r_a - r_inv_start);
            let th = theta_at_r(r_curr);
            let phi = phi_k - th;
            poly_2d.push([r_curr * phi.cos(), r_curr * phi.sin()]);
        }

        // 1d. Tip crest center point
        poly_2d.push([r_a * phi_k.cos(), r_a * phi_k.sin()]);

        // 1e. Involute points on trailing flank from r_a down to r_inv_start
        for s_idx in 0..=n_flank_steps {
            let s = 1.0 - (s_idx as f64) / (n_flank_steps as f64);
            let r_curr = r_inv_start + s * (r_a - r_inv_start);
            let th = theta_at_r(r_curr);
            let phi = phi_k + th;
            poly_2d.push([r_curr * phi.cos(), r_curr * phi.sin()]);
        }

        // 1f. If r_f < r_b, radial line from r_b down to r_f on trailing flank
        if r_f < r_b - 1e-9 {
            let phi_base_trail = phi_k + theta_base;
            poly_2d.push([r_f * phi_base_trail.cos(), r_f * phi_base_trail.sin()]);
        }

        // 1g. Root trough point between tooth k and tooth k+1
        let phi_trough = phi_k + PI / z;
        poly_2d.push([r_f * phi_trough.cos(), r_f * phi_trough.sin()]);
    }

    // Filter duplicate consecutive points if any
    let mut cleaned_poly: Vec<[f64; 2]> = Vec::with_capacity(poly_2d.len());
    for p in poly_2d {
        if let Some(last) = cleaned_poly.last() {
            let d2 = (p[0] - last[0]).powi(2) + (p[1] - last[1]).powi(2);
            if d2 > 1e-14 {
                cleaned_poly.push(p);
            }
        } else {
            cleaned_poly.push(p);
        }
    }
    if cleaned_poly.len() > 2 {
        let first = cleaned_poly[0];
        let last = *cleaned_poly.last().unwrap();
        let d2 = (first[0] - last[0]).powi(2) + (first[1] - last[1]).powi(2);
        if d2 <= 1e-14 {
            cleaned_poly.pop();
        }
    }

    let m_pts = cleaned_poly.len();
    if m_pts < 3 {
        return TriangleMesh::empty();
    }

    // 2. Extrude the star-shaped 2D polygon into a watertight 3D solid mesh
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // 2a. Top cap at z = +h2 (normal = [0, 0, 1])
    let top_center_idx = vertices.len() as u32;
    vertices.push(Vertex::new([0.0, 0.0, h2], [0.0, 0.0, 1.0]));

    let top_rim_start = vertices.len() as u32;
    for p in &cleaned_poly {
        vertices.push(Vertex::new([p[0], p[1], h2], [0.0, 0.0, 1.0]));
    }
    for i in 0..m_pts {
        let next = (i + 1) % m_pts;
        indices.push([
            top_center_idx,
            top_rim_start + (i as u32),
            top_rim_start + (next as u32),
        ]);
    }

    // 2b. Bottom cap at z = -h2 (normal = [0, 0, -1])
    let bot_center_idx = vertices.len() as u32;
    vertices.push(Vertex::new([0.0, 0.0, -h2], [0.0, 0.0, -1.0]));

    let bot_rim_start = vertices.len() as u32;
    for p in &cleaned_poly {
        vertices.push(Vertex::new([p[0], p[1], -h2], [0.0, 0.0, -1.0]));
    }
    for i in 0..m_pts {
        let next = (i + 1) % m_pts;
        indices.push([
            bot_center_idx,
            bot_rim_start + (next as u32),
            bot_rim_start + (i as u32),
        ]);
    }

    // 2c. Side walls quads (2 triangles each) with outward facet normals
    for i in 0..m_pts {
        let next = (i + 1) % m_pts;
        let p1 = cleaned_poly[i];
        let p2 = cleaned_poly[next];

        let dx = p2[0] - p1[0];
        let dy = p2[1] - p1[1];
        let edge_len = (dx * dx + dy * dy).sqrt();
        let normal = if edge_len > 1e-12 {
            [dy / edge_len, -dx / edge_len, 0.0]
        } else {
            [1.0, 0.0, 0.0]
        };

        let base = vertices.len() as u32;
        vertices.push(Vertex::new([p1[0], p1[1], h2], normal));  // 0: top curr
        vertices.push(Vertex::new([p1[0], p1[1], -h2], normal)); // 1: bot curr
        vertices.push(Vertex::new([p2[0], p2[1], -h2], normal)); // 2: bot next
        vertices.push(Vertex::new([p2[0], p2[1], h2], normal));  // 3: top next

        indices.push([base, base + 1, base + 3]);
        indices.push([base + 1, base + 2, base + 3]);
    }

    let mut solid = TriangleMesh::new(vertices, indices);

    // 3. Optional Hub support
    if let (Some(hub_d), Some(hub_l)) = (hub_diameter, hub_length) {
        if hub_d > 1e-6 && hub_l > 1e-6 {
            let hub_r = hub_d * 0.5;
            let total_hub_len = if hub_l > face_width {
                hub_l
            } else {
                face_width + hub_l
            };
            let hub_z_center = -h2 + total_hub_len * 0.5;
            let mut hub_mesh = create_cylinder(hub_r, total_hub_len, 64);
            hub_mesh.translate([0.0, 0.0, hub_z_center]);
            solid = csg_union(&solid, &hub_mesh);
        }
    }

    // 4. Optional Bore Hole & DIN 6885 Keyway support
    if let Some(bore_r) = bore_radius {
        if bore_r > 1e-6 {
            let aabb = solid.bounding_box();
            let z_min = aabb.min[2];
            let z_max = aabb.max[2];
            let total_span = z_max - z_min;
            let margin = (total_span * 0.05).clamp(1e-4, 0.02);
            let cutter_height = total_span + margin * 2.0;
            let z_center = (z_min + z_max) * 0.5;

            let mut bore_tool = create_cylinder(bore_r, cutter_height, 64);
            bore_tool.translate([0.0, 0.0, z_center]);

            let cutter = if let (Some(kw), Some(kd)) = (keyway_width, keyway_depth) {
                if kw > 1e-6 && kd > 1e-6 {
                    let mut keyway_box = create_box(kw, bore_r + kd, cutter_height);
                    keyway_box.translate([0.0, (bore_r + kd) * 0.5, z_center]);
                    csg_union(&bore_tool, &keyway_box)
                } else {
                    bore_tool
                }
            } else {
                bore_tool
            };

            solid = csg_difference(&solid, &cutter);
        }
    }

    solid
}

/// Helper: 2D polygon ear-clipping triangulation for simple polygons.
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

    let is_point_in_tri = |p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]| -> bool {
        let cross = |p1: [f64; 2], p2: [f64; 2], p3: [f64; 2]| {
            (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
        };
        let c1 = cross(a, b, p);
        let c2 = cross(b, c, p);
        let c3 = cross(c, a, p);
        let has_neg = (c1 < -1e-9) || (c2 < -1e-9) || (c3 < -1e-9);
        let has_pos = (c1 > 1e-9) || (c2 > 1e-9) || (c3 > 1e-9);
        !(has_neg && has_pos)
    };

    let is_ear = |prev: usize, curr: usize, next: usize, idxs: &[u32]| -> bool {
        let a = pts[idxs[prev] as usize];
        let b = pts[idxs[curr] as usize];
        let c = pts[idxs[next] as usize];
        // Convex check (CCW interior angle)
        let cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
        if cross <= 1e-12 {
            return false;
        }
        for (i, &idx) in idxs.iter().enumerate() {
            if i == prev || i == curr || i == next {
                continue;
            }
            let p = pts[idx as usize];
            if is_point_in_tri(p, a, b, c) {
                return false;
            }
        }
        true
    };

    let mut stuck_count = 0;
    while indices.len() > 3 && stuck_count < indices.len() * 2 {
        let len = indices.len();
        let mut ear_found = false;
        for i in 0..len {
            let prev = (i + len - 1) % len;
            let next = (i + 1) % len;
            if is_ear(prev, i, next, &indices) {
                triangles.push([indices[prev], indices[i], indices[next]]);
                indices.remove(i);
                ear_found = true;
                break;
            }
        }
        if !ear_found {
            stuck_count += 1;
            if stuck_count >= len {
                triangles.push([indices[0], indices[1], indices[2]]);
                indices.remove(1);
            }
        } else {
            stuck_count = 0;
        }
    }
    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }
    triangles
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

    let segs = segments.max(3);
    let sweep_angle_deg = angle_deg.clamp(1e-4, 360.0);
    let is_full_circle = (sweep_angle_deg - 360.0).abs() < 1e-5;
    let sweep_angle_rad = sweep_angle_deg * PI / 180.0;

    // 1. Clean profile: ensure r >= 0 and remove duplicate consecutive points
    let mut pts: Vec<[f64; 2]> = Vec::with_capacity(profile.len());
    for p in profile {
        let r = p[0].max(0.0);
        let z = p[1];
        if let Some(last) = pts.last() {
            let d2 = (r - last[0]).powi(2) + (z - last[1]).powi(2);
            if d2 > 1e-12 {
                pts.push([r, z]);
            }
        } else {
            pts.push([r, z]);
        }
    }
    if pts.len() > 2 {
        let first = pts[0];
        let last = *pts.last().unwrap();
        let d2 = (first[0] - last[0]).powi(2) + (first[1] - last[1]).powi(2);
        if d2 <= 1e-12 {
            pts.pop();
        }
    }
    let m = pts.len();
    if m < 3 {
        return TriangleMesh::empty();
    }

    // 2. Ensure CCW orientation in the (r, z) half-plane (r as X, z as Y)
    let mut area2d = 0.0;
    for i in 0..m {
        let next = (i + 1) % m;
        area2d += pts[i][0] * pts[next][1] - pts[next][0] * pts[i][1];
    }
    if area2d < 0.0 {
        pts.reverse();
    }

    // 3. Generate vertices in angular rings
    let n_steps = if is_full_circle { segs } else { segs + 1 };
    let mut vertices = Vec::with_capacity(n_steps * m);
    let mut indices = Vec::new();

    for j in 0..n_steps {
        let theta = if is_full_circle {
            (j as f64) * 2.0 * PI / (segs as f64)
        } else {
            (j as f64) * sweep_angle_rad / (segs as f64)
        };
        let cos_t = theta.cos();
        let sin_t = theta.sin();

        for p in &pts {
            let r = p[0];
            let z = p[1];
            let x = r * cos_t;
            let y = r * sin_t;
            vertices.push(Vertex::new([x, y, z], [cos_t, sin_t, 0.0]));
        }
    }

    // 4. Triangulate quad strips between ring j and ring next_j
    for j in 0..segs {
        let next_j = if is_full_circle {
            (j + 1) % segs
        } else {
            j + 1
        };

        for i in 0..m {
            let next_i = (i + 1) % m;

            let r_a = pts[i][0];
            let r_b = pts[next_i][0];

            // Edge lying on rotation axis has zero surface area
            if r_a < 1e-9 && r_b < 1e-9 {
                continue;
            }

            let v_curr_i = (j * m + i) as u32;
            let v_curr_next = (j * m + next_i) as u32;
            let v_next_i = (next_j * m + i) as u32;
            let v_next_next = (next_j * m + next_i) as u32;

            if r_a < 1e-9 {
                // Triangle fan around axis point at i
                indices.push([v_curr_i, v_next_next, v_curr_next]);
            } else if r_b < 1e-9 {
                // Triangle fan around axis point at next_i
                indices.push([v_curr_i, v_next_i, v_next_next]);
            } else {
                // Regular quad split into 2 triangles
                indices.push([v_curr_i, v_next_i, v_next_next]);
                indices.push([v_curr_i, v_next_next, v_curr_next]);
            }
        }
    }

    // 5. If partial angle, cap the planar start and end faces
    if !is_full_circle {
        let cap_triangles = triangulate_simple_polygon_2d(&pts);
        // Start cap at ring 0 (theta = 0): outward normal points towards -Y
        for tri in &cap_triangles {
            let i0 = tri[0];
            let i1 = tri[1];
            let i2 = tri[2];
            indices.push([i0, i2, i1]);
        }
        // End cap at ring segs (theta = sweep_angle): outward normal points towards +theta
        let end_offset = (segs * m) as u32;
        for tri in &cap_triangles {
            let i0 = end_offset + tri[0];
            let i1 = end_offset + tri[1];
            let i2 = end_offset + tri[2];
            indices.push([i0, i1, i2]);
        }
    }

    let mut mesh = TriangleMesh::new(vertices, indices);

    // 6. Enforce outward orientation and calculate accurate vertex normals
    if mesh.signed_volume() < 0.0 {
        for tri in &mut mesh.indices {
            tri.swap(1, 2);
        }
    }
    mesh.recompute_vertex_normals();

    mesh
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
    if rim_diameter <= 1e-4 || rim_width <= 1e-4 {
        return TriangleMesh::empty();
    }

    let d_rim = rim_diameter;
    let w_rim = rim_width;
    let r_bead = d_rim * 0.5;

    let d_flange = flange_diameter.unwrap_or(d_rim * 1.08).max(d_rim * 1.02);
    let r_flange = d_flange * 0.5;

    let d_hub = hub_diameter.unwrap_or(d_rim * 0.35).clamp(d_rim * 0.15, d_rim * 0.65);
    let r_hub = d_hub * 0.5;

    let d_bore = hub_bore_diameter.unwrap_or(d_hub * 0.45).clamp(d_hub * 0.15, d_hub * 0.70);
    let r_bore = d_bore * 0.5;

    let d_pcd = pcd_diameter.unwrap_or((d_hub + d_bore) * 0.5).clamp(d_bore + 0.015, d_hub - 0.012);
    let r_pcd = d_pcd * 0.5;

    let n_bolts = bolt_holes_count.unwrap_or(5).clamp(3, 12);
    let d_bolt = bolt_hole_diameter.unwrap_or(0.014).clamp(0.005, (r_hub - r_pcd).min(r_pcd - r_bore).max(0.008) * 1.6);
    let r_bolt = d_bolt * 0.5;

    let n_spokes = spokes_count.unwrap_or(5).max(3);

    let max_spoke_w = (2.0 * PI * r_hub / (n_spokes as f64)) * 0.75;
    let w_spoke = spoke_width.unwrap_or(d_rim * 0.055).clamp(0.010, max_spoke_w);

    let t_spoke = spoke_thickness.unwrap_or(w_spoke * 0.65).clamp(0.008, w_rim * 0.4);

    let et = offset_et.unwrap_or(0.035).clamp(-w_rim * 0.4, w_rim * 0.4);

    // 1. Construct stepped rim barrel profile (in mm/m): outer flanges, bead seats, drop center well
    let t_w = (r_bead * 0.045).clamp(0.006, 0.012);
    let drop_depth = (r_bead * 0.08).clamp(0.015, 0.035);
    let r_well = r_bead - drop_depth;
    let r_hump = r_bead + (r_bead * 0.012).clamp(0.002, 0.005);

    let h_w = w_rim * 0.5;
    let barrel_profile: Vec<[f64; 2]> = vec![
        // Outer surface: from inner flange (-h_w) to outer flange (+h_w)
        [r_flange, -h_w],
        [r_flange, -h_w + 0.05 * w_rim],
        [r_bead + 0.004, -h_w + 0.08 * w_rim],
        [r_bead, -h_w + 0.12 * w_rim],
        [r_bead, -h_w + 0.28 * w_rim],
        [r_well, -h_w + 0.38 * w_rim],
        [r_well, h_w - 0.35 * w_rim],
        [r_hump, h_w - 0.24 * w_rim],
        [r_bead, h_w - 0.20 * w_rim],
        [r_bead, h_w - 0.08 * w_rim],
        [r_flange, h_w - 0.04 * w_rim],
        [r_flange, h_w],
        // Outer lip thickness
        [r_flange - t_w, h_w],
        // Inner surface: from outer lip back to inner lip
        [r_bead - t_w, h_w - 0.08 * w_rim],
        [r_well - t_w, h_w - 0.35 * w_rim],
        [r_well - t_w, -h_w + 0.38 * w_rim],
        [r_bead - t_w, -h_w + 0.28 * w_rim],
        [r_flange - t_w, -h_w],
    ];

    let barrel = create_revolve(&barrel_profile, 360.0, 36);

    // 2. Central hub cylinder with bore hole and PCD bolt pattern
    let t_hub = (w_rim * 0.15).clamp(0.015, 0.035);
    let z_hub_center = et - t_hub * 0.5;

    let mut hub_solid = create_cylinder(r_hub, t_hub, 32);
    hub_solid.translate([0.0, 0.0, z_hub_center]);

    let cutter_height = t_hub + 0.010;
    let mut cutter_vertices = Vec::new();
    let mut cutter_indices = Vec::new();

    let mut add_to_cutters = |cyl: &TriangleMesh| {
        let base_idx = cutter_vertices.len() as u32;
        cutter_vertices.extend_from_slice(&cyl.vertices);
        for tri in &cyl.indices {
            cutter_indices.push([tri[0] + base_idx, tri[1] + base_idx, tri[2] + base_idx]);
        }
    };

    let mut bore_cyl = create_cylinder(r_bore, cutter_height, 32);
    bore_cyl.translate([0.0, 0.0, z_hub_center]);
    add_to_cutters(&bore_cyl);

    for b in 0..n_bolts {
        let beta = (b as f64) * 2.0 * PI / (n_bolts as f64) + PI / (n_bolts as f64);
        let bx = r_pcd * beta.cos();
        let by = r_pcd * beta.sin();
        let mut bolt_cyl = create_cylinder(r_bolt, cutter_height, 16);
        bolt_cyl.translate([bx, by, z_hub_center]);
        add_to_cutters(&bolt_cyl);
    }
    let all_cutters = TriangleMesh::new(cutter_vertices, cutter_indices);

    let hub_finished = csg_difference(&hub_solid, &all_cutters);

    // 3. Structural spokes with open cooling windows between them
    let r_spoke_inner = r_hub - 0.003;
    let r_spoke_outer = (r_bead - t_w) + 0.004;
    let l_spoke = r_spoke_outer - r_spoke_inner;
    let r_spoke_mid = (r_spoke_inner + r_spoke_outer) * 0.5;

    let mut spoke_vertices = Vec::new();
    let mut spoke_indices = Vec::new();

    for s in 0..n_spokes {
        let theta_s = (s as f64) * 2.0 * PI / (n_spokes as f64);
        let mut spoke = create_box(l_spoke, w_spoke, t_spoke);
        spoke.translate([r_spoke_mid, 0.0, z_hub_center]);
        spoke.rotate_rpy_deg(0.0, 0.0, theta_s * 180.0 / PI);

        let base_idx = spoke_vertices.len() as u32;
        spoke_vertices.extend_from_slice(&spoke.vertices);
        for tri in &spoke.indices {
            spoke_indices.push([tri[0] + base_idx, tri[1] + base_idx, tri[2] + base_idx]);
        }
    }
    let all_spokes = TriangleMesh::new(spoke_vertices, spoke_indices);

    // 4. Union hub, spokes, and barrel
    let hub_and_spokes = csg_union(&hub_finished, &all_spokes);
    let mut wheel = csg_union(&hub_and_spokes, &barrel);
    wheel.recompute_vertex_normals();

    wheel
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
}
