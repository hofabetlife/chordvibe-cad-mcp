//! Constructive Solid Geometry (CSG) operations via Binary Space Partitioning (BSP) trees.
//! Provides Boolean operations: Union, Difference, and Intersection on 3D meshes.

use crate::topology::{FaceInfo, FaceKind, TriangleMesh, Vertex};
use nalgebra::Vector3;
use std::collections::{HashMap, HashSet};

const EPSILON: f64 = 1e-7;

#[derive(Debug, Clone, Copy, PartialEq)]
enum VertexType {
    Coplanar,
    Front,
    Back,
}

/// A 3D infinite plane defined by equation: normal . p - w = 0
#[derive(Debug, Clone, PartialEq)]
pub struct Plane {
    pub normal: Vector3<f64>,
    pub w: f64,
}

impl Plane {
    /// Creates a plane from 3 non-collinear points.
    pub fn from_points(a: Vector3<f64>, b: Vector3<f64>, c: Vector3<f64>) -> Option<Self> {
        let normal = (b - a).cross(&(c - a));
        let len = normal.norm();
        if len < 1e-12 {
            return None;
        }
        let unit_n = normal / len;
        let w = unit_n.dot(&a);
        Some(Self { normal: unit_n, w })
    }

    /// Flips plane orientation.
    pub fn flip(&mut self) {
        self.normal = -self.normal;
        self.w = -self.w;
    }

    /// Splits a polygon by this plane into coplanar_front, coplanar_back, front, back lists.
    pub fn split_polygon(
        &self,
        poly: &CsgPolygon,
        coplanar_front: &mut Vec<CsgPolygon>,
        coplanar_back: &mut Vec<CsgPolygon>,
        front: &mut Vec<CsgPolygon>,
        back: &mut Vec<CsgPolygon>,
    ) {
        let mut types = Vec::with_capacity(poly.vertices.len());
        let mut has_front = false;
        let mut has_back = false;

        for v in &poly.vertices {
            let t = self.normal.dot(&v.pos_vec()) - self.w;
            let vtype = if t > EPSILON {
                has_front = true;
                VertexType::Front
            } else if t < -EPSILON {
                has_back = true;
                VertexType::Back
            } else {
                VertexType::Coplanar
            };
            types.push(vtype);
        }

        if !has_front && !has_back {
            // Entire polygon is coplanar with this plane
            if self.normal.dot(&poly.plane.normal) > 0.0 {
                coplanar_front.push(poly.clone());
            } else {
                coplanar_back.push(poly.clone());
            }
        } else if has_front && !has_back {
            front.push(poly.clone());
        } else if !has_front && has_back {
            back.push(poly.clone());
        } else {
            // SPANNING: polygon crosses the plane, split it
            let mut f_verts = Vec::new();
            let mut b_verts = Vec::new();

            let count = poly.vertices.len();
            for i in 0..count {
                let j = (i + 1) % count;
                let ti = types[i];
                let tj = types[j];
                let vi = &poly.vertices[i];
                let vj = &poly.vertices[j];

                if ti != VertexType::Back {
                    f_verts.push(*vi);
                }
                if ti != VertexType::Front {
                    b_verts.push(*vi);
                }

                if (ti == VertexType::Front && tj == VertexType::Back)
                    || (ti == VertexType::Back && tj == VertexType::Front)
                {
                    // Compute intersection along edge vi -> vj
                    let pi = vi.pos_vec();
                    let pj = vj.pos_vec();
                    let denom = self.normal.dot(&(pj - pi));
                    let t = if denom.abs() > 1e-14 {
                        ((self.w - self.normal.dot(&pi)) / denom).clamp(0.0, 1.0)
                    } else {
                        0.5
                    };

                    let p_mid = pi + (pj - pi) * t;
                    let n_mid = vi.normal_vec() + (vj.normal_vec() - vi.normal_vec()) * t;
                    let n_norm = n_mid.norm();
                    let n_unit = if n_norm > 1e-12 {
                        n_mid / n_norm
                    } else {
                        poly.plane.normal
                    };

                    let v_mid =
                        Vertex::new([p_mid.x, p_mid.y, p_mid.z], [n_unit.x, n_unit.y, n_unit.z]);
                    f_verts.push(v_mid);
                    b_verts.push(v_mid);
                }
            }

            dedup_polygon_vertices(&mut f_verts);
            dedup_polygon_vertices(&mut b_verts);

            if f_verts.len() >= 3 {
                front.push(CsgPolygon {
                    vertices: f_verts,
                    plane: poly.plane.clone(),
                    face_id: poly.face_id,
                });
            }
            if b_verts.len() >= 3 {
                back.push(CsgPolygon {
                    vertices: b_verts,
                    plane: poly.plane.clone(),
                    face_id: poly.face_id,
                });
            }
        }
    }
}

fn dedup_polygon_vertices(verts: &mut Vec<Vertex>) {
    if verts.len() < 2 {
        return;
    }
    let mut cleaned: Vec<Vertex> = Vec::with_capacity(verts.len());
    for v in verts.drain(..) {
        if let Some(last) = cleaned.last() {
            if (v.pos_vec() - last.pos_vec()).norm_squared() < 1e-20 {
                continue;
            }
        }
        cleaned.push(v);
    }
    while cleaned.len() >= 2 {
        let first = cleaned[0].pos_vec();
        let last = cleaned[cleaned.len() - 1].pos_vec();
        if (first - last).norm_squared() < 1e-20 {
            cleaned.pop();
        } else {
            break;
        }
    }
    *verts = cleaned;
}

/// A convex planar polygon carrying vertex normals, a plane equation, and originating topological face ID.
#[derive(Debug, Clone, PartialEq)]
pub struct CsgPolygon {
    pub vertices: Vec<Vertex>,
    pub plane: Plane,
    pub face_id: u32,
}

impl CsgPolygon {
    /// Creates a polygon from a list of vertices with default untagged `face_id = 0`.
    pub fn new(vertices: Vec<Vertex>) -> Option<Self> {
        Self::with_face_id(vertices, 0)
    }

    /// Creates a polygon from a list of vertices with an explicit topological `face_id`.
    pub fn with_face_id(vertices: Vec<Vertex>, face_id: u32) -> Option<Self> {
        if vertices.len() < 3 {
            return None;
        }
        let plane = Plane::from_points(
            vertices[0].pos_vec(),
            vertices[1].pos_vec(),
            vertices[2].pos_vec(),
        )?;
        Some(Self {
            vertices,
            plane,
            face_id,
        })
    }

    /// Flips vertex winding order and normal orientation.
    pub fn flip(&mut self) {
        self.vertices.reverse();
        for v in &mut self.vertices {
            v.normal = [-v.normal[0], -v.normal[1], -v.normal[2]];
        }
        self.plane.flip();
    }
}

/// A node in the Binary Space Partitioning (BSP) tree.
#[derive(Debug, Clone, Default)]
pub struct BspNode {
    pub plane: Option<Plane>,
    pub front: Option<Box<BspNode>>,
    pub back: Option<Box<BspNode>>,
    pub polygons: Vec<CsgPolygon>,
}

impl BspNode {
    /// Creates an empty BSP node.
    pub fn new() -> Self {
        Self::default()
    }

    /// Extracts CSG polygons from a triangle mesh, merging consecutive coplanar convex triangle
    /// pairs (such as box faces and cylinder/prism/cone side quads) into 4-vertex polygons to avoid
    /// slicing internal triangulation diagonals during boolean splitting.
    pub fn polygons_from_mesh(mesh: &TriangleMesh) -> Vec<CsgPolygon> {
        let n_tris = mesh.indices.len();
        let mut polygons = Vec::with_capacity(n_tris);
        let mut i = 0;
        while i < n_tris {
            let tri_a = mesh.indices[i];
            let fid_a = mesh.face_ids.get(i).copied().unwrap_or(0);
            let va0 = mesh.vertices[tri_a[0] as usize];
            let va1 = mesh.vertices[tri_a[1] as usize];
            let va2 = mesh.vertices[tri_a[2] as usize];

            let mut merged_quad: Option<CsgPolygon> = None;
            if i + 1 < n_tris && mesh.face_ids.get(i + 1).copied().unwrap_or(0) == fid_a {
                let tri_b = mesh.indices[i + 1];
                let vb0 = mesh.vertices[tri_b[0] as usize];
                let vb1 = mesh.vertices[tri_b[1] as usize];
                let vb2 = mesh.vertices[tri_b[2] as usize];
                let a_verts = [va0, va1, va2];
                let b_verts = [vb0, vb1, vb2];

                if let (Some(pa), Some(pb)) = (
                    Plane::from_points(va0.pos_vec(), va1.pos_vec(), va2.pos_vec()),
                    Plane::from_points(vb0.pos_vec(), vb1.pos_vec(), vb2.pos_vec()),
                ) {
                    if pa.normal.dot(&pb.normal) > 1.0 - 1e-9 && (pa.w - pb.w).abs() <= 1e-8 {
                        'edge_search: for ea in 0..3 {
                            let ua = a_verts[ea];
                            let va = a_verts[(ea + 1) % 3];
                            let wa = a_verts[(ea + 2) % 3];
                            for eb in 0..3 {
                                let ub = b_verts[eb];
                                let vb = b_verts[(eb + 1) % 3];
                                let wb = b_verts[(eb + 2) % 3];
                                if (ua.pos_vec() - vb.pos_vec()).norm_squared() <= 1e-20
                                    && (va.pos_vec() - ub.pos_vec()).norm_squared() <= 1e-20
                                    && (wa.pos_vec() - wb.pos_vec()).norm_squared() > 1e-16
                                {
                                    let q = [ua, wb, va, wa];
                                    let mut strictly_convex = true;
                                    for k in 0..4 {
                                        let p_prev = q[(k + 3) % 4].pos_vec();
                                        let p_curr = q[k].pos_vec();
                                        let p_next = q[(k + 1) % 4].pos_vec();
                                        let e1 = p_curr - p_prev;
                                        let e2 = p_next - p_curr;
                                        let l1 = e1.norm();
                                        let l2 = e2.norm();
                                        if l1 <= 1e-12
                                            || l2 <= 1e-12
                                            || e1.cross(&e2).dot(&pa.normal) <= 1e-5 * l1 * l2
                                        {
                                            strictly_convex = false;
                                            break;
                                        }
                                    }
                                    if strictly_convex {
                                        merged_quad = Some(CsgPolygon {
                                            vertices: q.to_vec(),
                                            plane: pa,
                                            face_id: fid_a,
                                        });
                                        break 'edge_search;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if let Some(quad_poly) = merged_quad {
                polygons.push(quad_poly);
                i += 2;
            } else {
                if let Some(poly) = CsgPolygon::with_face_id(vec![va0, va1, va2], fid_a) {
                    polygons.push(poly);
                }
                i += 1;
            }
        }
        polygons
    }

    /// Builds a BSP tree from a triangle mesh.
    pub fn from_mesh(mesh: &TriangleMesh) -> Self {
        let polygons = Self::polygons_from_mesh(mesh);
        let mut node = Self::new();
        node.build(polygons);
        node
    }

    /// Inverts all polygons in the tree and swaps front/back subtrees.
    pub fn invert(&mut self) {
        for poly in &mut self.polygons {
            poly.flip();
        }
        if let Some(ref mut p) = self.plane {
            p.flip();
        }
        if let Some(ref mut f) = self.front {
            f.invert();
        }
        if let Some(ref mut b) = self.back {
            b.invert();
        }
        std::mem::swap(&mut self.front, &mut self.back);
    }

    fn clip_single_polygon(&self, poly: &CsgPolygon) -> ClipOutcome {
        let Some(ref plane) = self.plane else {
            return ClipOutcome::Unclipped;
        };

        let mut front_list = Vec::new();
        let mut back_list = Vec::new();
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();

        plane.split_polygon(
            poly,
            &mut coplanar_front,
            &mut coplanar_back,
            &mut front_list,
            &mut back_list,
        );

        front_list.extend(coplanar_front);
        back_list.extend(coplanar_back);

        match (front_list.len(), back_list.len()) {
            (0, 0) => ClipOutcome::Discarded,
            (1, 0) => {
                if let Some(ref f) = self.front {
                    f.clip_single_polygon(&front_list[0])
                } else {
                    ClipOutcome::Unclipped
                }
            }
            (0, 1) => {
                if let Some(ref b) = self.back {
                    b.clip_single_polygon(&back_list[0])
                } else {
                    ClipOutcome::Discarded
                }
            }
            _ => {
                let f_poly = &front_list[0];
                let b_poly = &back_list[0];
                let out_f = if let Some(ref f) = self.front {
                    f.clip_single_polygon(f_poly)
                } else {
                    ClipOutcome::Unclipped
                };
                let out_b = if let Some(ref b) = self.back {
                    b.clip_single_polygon(b_poly)
                } else {
                    ClipOutcome::Discarded
                };

                match (out_f, out_b) {
                    (ClipOutcome::Unclipped, ClipOutcome::Unclipped) => ClipOutcome::Unclipped,
                    (ClipOutcome::Discarded, ClipOutcome::Discarded) => ClipOutcome::Discarded,
                    (of, ob) => {
                        let mut pieces = Vec::new();
                        match of {
                            ClipOutcome::Unclipped => pieces.push(f_poly.clone()),
                            ClipOutcome::Discarded => {}
                            ClipOutcome::Partial(v) => pieces.extend(v),
                        }
                        match ob {
                            ClipOutcome::Unclipped => pieces.push(b_poly.clone()),
                            ClipOutcome::Discarded => {}
                            ClipOutcome::Partial(v) => pieces.extend(v),
                        }
                        if pieces.is_empty() {
                            ClipOutcome::Discarded
                        } else {
                            ClipOutcome::Partial(pieces)
                        }
                    }
                }
            }
        }
    }

    /// Recursively clips a list of polygons against this BSP tree, coalescing sibling splits
    /// whenever both halves of a split polygon lie completely outside (or completely inside) the solid.
    pub fn clip_polygons(&self, list: &[CsgPolygon]) -> Vec<CsgPolygon> {
        if self.plane.is_none() {
            return list.to_vec();
        }
        let mut result = Vec::with_capacity(list.len());
        for poly in list {
            match self.clip_single_polygon(poly) {
                ClipOutcome::Unclipped => result.push(poly.clone()),
                ClipOutcome::Discarded => {}
                ClipOutcome::Partial(v) => result.extend(v),
            }
        }
        result
    }

    /// Clips all polygons in this tree against another BSP tree.
    pub fn clip_to(&mut self, other: &BspNode) {
        self.polygons = other.clip_polygons(&self.polygons);
        if let Some(ref mut f) = self.front {
            f.clip_to(other);
        }
        if let Some(ref mut b) = self.back {
            b.clip_to(other);
        }
    }

    /// Collects all polygons in this BSP tree into a flat vector.
    pub fn all_polygons(&self) -> Vec<CsgPolygon> {
        let mut result = self.polygons.clone();
        if let Some(ref f) = self.front {
            result.extend(f.all_polygons());
        }
        if let Some(ref b) = self.back {
            result.extend(b.all_polygons());
        }
        result
    }

    /// Adds a list of polygons to the BSP tree.
    pub fn build(&mut self, polygons: Vec<CsgPolygon>) {
        if polygons.is_empty() {
            return;
        }

        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.as_ref().unwrap();

        let mut front_list = Vec::new();
        let mut back_list = Vec::new();
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();

        for poly in polygons {
            plane.split_polygon(
                &poly,
                &mut coplanar_front,
                &mut coplanar_back,
                &mut front_list,
                &mut back_list,
            );
        }

        self.polygons.extend(coplanar_front);
        self.polygons.extend(coplanar_back);

        if !front_list.is_empty() {
            if self.front.is_none() {
                self.front = Some(Box::new(BspNode::new()));
            }
            self.front.as_mut().unwrap().build(front_list);
        }

        if !back_list.is_empty() {
            if self.back.is_none() {
                self.back = Some(Box::new(BspNode::new()));
            }
            self.back.as_mut().unwrap().build(back_list);
        }
    }
}

enum ClipOutcome {
    Unclipped,
    Discarded,
    Partial(Vec<CsgPolygon>),
}

/// Converts a collection of CSG polygons back into a watertight, T-junction-free surface mesh.
pub fn polygons_to_mesh(polygons: &[CsgPolygon]) -> TriangleMesh {
    if polygons.is_empty() {
        return TriangleMesh::empty();
    }

    // 1. Compute bounding box diagonal for scale-relative tolerances
    let mut min_pt = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max_pt = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for poly in polygons {
        for v in &poly.vertices {
            let p = v.pos_vec();
            min_pt.x = min_pt.x.min(p.x);
            min_pt.y = min_pt.y.min(p.y);
            min_pt.z = min_pt.z.min(p.z);
            max_pt.x = max_pt.x.max(p.x);
            max_pt.y = max_pt.y.max(p.y);
            max_pt.z = max_pt.z.max(p.z);
        }
    }
    let diag = (max_pt - min_pt).norm().max(1e-6);
    let weld_eps = (diag * 2.5e-5).max(1e-7);
    let weld_eps_sq = weld_eps * weld_eps;
    let cell_size = weld_eps * 2.0;

    // 2. Weld vertices across all polygons into canonical positions
    let mut canon_pos: Vec<Vector3<f64>> = Vec::new();
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();

    let mut find_or_insert = |p: Vector3<f64>| -> u32 {
        let gx = (p.x / cell_size).floor() as i64;
        let gy = (p.y / cell_size).floor() as i64;
        let gz = (p.z / cell_size).floor() as i64;

        let mut best_id: Option<u32> = None;
        let mut best_dist_sq = weld_eps_sq;

        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(bucket) = grid.get(&(gx + dx, gy + dy, gz + dz)) {
                        for &cid in bucket {
                            let d2 = (canon_pos[cid as usize] - p).norm_squared();
                            if d2 <= best_dist_sq {
                                best_dist_sq = d2;
                                best_id = Some(cid);
                            }
                        }
                    }
                }
            }
        }

        if let Some(id) = best_id {
            id
        } else {
            let id = canon_pos.len() as u32;
            canon_pos.push(p);
            grid.entry((gx, gy, gz)).or_default().push(id);
            id
        }
    };

    let mut poly_loops: Vec<(Vec<u32>, Vector3<f64>, u32)> = Vec::with_capacity(polygons.len());
    for poly in polygons {
        if poly.vertices.len() < 3 {
            continue;
        }
        let mut ids: Vec<u32> = Vec::with_capacity(poly.vertices.len());
        for v in &poly.vertices {
            let cid = find_or_insert(v.pos_vec());
            if ids.last().copied() != Some(cid) {
                ids.push(cid);
            }
        }
        while ids.len() >= 2 && ids.first() == ids.last() {
            ids.pop();
        }
        if ids.len() >= 3 {
            poly_loops.push((ids, poly.plane.normal, poly.face_id));
        }
    }

    // 3. Subdivide polygon boundary edges at any collinear canonical vertices (T-junction prevention)
    let col_eps = (diag * 6e-5).max(2e-7);
    let col_eps_sq = col_eps * col_eps;
    let col_cell = (col_eps * 4.0).max(cell_size);
    let mut col_grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::with_capacity(canon_pos.len());
    for (cid, p) in canon_pos.iter().enumerate() {
        let key = (
            (p.x / col_cell).floor() as i64,
            (p.y / col_cell).floor() as i64,
            (p.z / col_cell).floor() as i64,
        );
        col_grid.entry(key).or_default().push(cid as u32);
    }

    let mut tris: Vec<([u32; 3], Vector3<f64>, u32)> = Vec::new();

    for (raw_loop, n_poly, face_id) in poly_loops {
        let mut subdivided: Vec<u32> = Vec::with_capacity(raw_loop.len() + 4);
        let m_len = raw_loop.len();

        for i in 0..m_len {
            let u = raw_loop[i];
            let v = raw_loop[(i + 1) % m_len];
            subdivided.push(u);
            if u == v {
                continue;
            }

            let pa = canon_pos[u as usize];
            let pb = canon_pos[v as usize];
            let ab = pb - pa;
            let ab_len_sq = ab.norm_squared();
            if ab_len_sq <= col_eps_sq {
                continue;
            }

            let seg_min_x = pa.x.min(pb.x) - col_eps;
            let seg_max_x = pa.x.max(pb.x) + col_eps;
            let seg_min_y = pa.y.min(pb.y) - col_eps;
            let seg_max_y = pa.y.max(pb.y) + col_eps;
            let seg_min_z = pa.z.min(pb.z) - col_eps;
            let seg_max_z = pa.z.max(pb.z) + col_eps;

            let mut splits: Vec<(f64, u32)> = Vec::new();
            let steps = ((ab_len_sq.sqrt() / col_cell).ceil() as usize).max(1);
            let mut visited_cells: Vec<(i64, i64, i64)> = Vec::with_capacity(steps + 2);
            for s_idx in 0..=steps {
                let frac = (s_idx as f64) / (steps as f64);
                let pt = pa + ab * frac;
                let cx = (pt.x / col_cell).floor() as i64;
                let cy = (pt.y / col_cell).floor() as i64;
                let cz = (pt.z / col_cell).floor() as i64;
                if visited_cells.last().copied() != Some((cx, cy, cz)) {
                    visited_cells.push((cx, cy, cz));
                }
            }
            let mut checked_neighborhood: Vec<(i64, i64, i64)> =
                Vec::with_capacity(visited_cells.len() * 27);
            for (cx, cy, cz) in visited_cells {
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        for dz in -1..=1 {
                            checked_neighborhood.push((cx + dx, cy + dy, cz + dz));
                        }
                    }
                }
            }
            checked_neighborhood.sort_unstable();
            checked_neighborhood.dedup();

            for cell_key in checked_neighborhood {
                let Some(bucket) = col_grid.get(&cell_key) else {
                    continue;
                };
                for &mid in bucket {
                    if mid == u || mid == v {
                        continue;
                    }
                    let pm = canon_pos[mid as usize];
                    if pm.x < seg_min_x
                        || pm.x > seg_max_x
                        || pm.y < seg_min_y
                        || pm.y > seg_max_y
                        || pm.z < seg_min_z
                        || pm.z > seg_max_z
                    {
                        continue;
                    }
                    let ap = pm - pa;
                    let t = ap.dot(&ab) / ab_len_sq;
                    if t > 1e-4 && t < 1.0 - 1e-4 {
                        let perp_sq = (ap - ab * t).norm_squared();
                        if perp_sq <= col_eps_sq {
                            splits.push((t, mid));
                        }
                    }
                }
            }

            if !splits.is_empty() {
                splits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
                splits.dedup_by_key(|&mut (_, mid)| mid);
                for (_, mid) in splits {
                    if subdivided.last().copied() != Some(mid) {
                        subdivided.push(mid);
                    }
                }
            }
        }

        clean_index_loop(&mut subdivided);
        if subdivided.len() < 3 {
            continue;
        }

        if subdivided.len() == 3 {
            let p0 = canon_pos[subdivided[0] as usize];
            let p1 = canon_pos[subdivided[1] as usize];
            let p2 = canon_pos[subdivided[2] as usize];
            if (p1 - p0).cross(&(p2 - p0)).norm() > 1e-13 {
                tris.push(([subdivided[0], subdivided[1], subdivided[2]], n_poly, face_id));
            }
        } else {
            triangulate_subdivided_convex_loop(
                &subdivided,
                n_poly,
                face_id,
                &canon_pos,
                col_eps_sq,
                &mut tris,
            );
        }
    }

    // 4. Post-triangulation manifold stitching & residual T-junction / slit resolution
    repair_manifold_triangles(&mut tris, &canon_pos, diag);

    // 4b. Multi-stage CSG mesh quality optimization (Delaunay flips, plane-preserving BSP cut
    // vertex collapse, and coplanar fan Steiner refinement)
    optimize_csg_mesh_quality(&mut tris, &mut canon_pos, diag);

    // 5. Emit TriangleMesh with crisp per-face normals and preserved face_ids
    let mut vertices = Vec::with_capacity(tris.len() * 3);
    let mut indices = Vec::with_capacity(tris.len());
    let mut face_ids = Vec::with_capacity(tris.len());

    for (tri, hint_normal, fid) in tris {
        let p0 = canon_pos[tri[0] as usize];
        let p1 = canon_pos[tri[1] as usize];
        let p2 = canon_pos[tri[2] as usize];
        let cross = (p1 - p0).cross(&(p2 - p0));
        let len = cross.norm();
        if len <= 1e-13 {
            continue;
        }
        let n = if len > 1e-12 {
            cross / len
        } else {
            hint_normal
        };
        let n_arr = [n.x, n.y, n.z];
        let base = vertices.len() as u32;
        vertices.push(Vertex::new([p0.x, p0.y, p0.z], n_arr));
        vertices.push(Vertex::new([p1.x, p1.y, p1.z], n_arr));
        vertices.push(Vertex::new([p2.x, p2.y, p2.z], n_arr));
        indices.push([base, base + 1, base + 2]);
        face_ids.push(fid);
    }

    let mut mesh = TriangleMesh::new(vertices, indices);
    mesh.face_ids = face_ids;
    mesh
}

#[inline]
fn min_triangle_angle_rad(p0: Vector3<f64>, p1: Vector3<f64>, p2: Vector3<f64>) -> f64 {
    let a = (p1 - p0).norm();
    let b = (p2 - p1).norm();
    let c = (p0 - p2).norm();
    if a <= 1e-13 || b <= 1e-13 || c <= 1e-13 {
        return 0.0;
    }
    let cos_a = ((b * b + c * c - a * a) / (2.0 * b * c)).clamp(-1.0, 1.0);
    let cos_b = ((a * a + c * c - b * b) / (2.0 * a * c)).clamp(-1.0, 1.0);
    let cos_c = ((a * a + b * b - c * c) / (2.0 * a * b)).clamp(-1.0, 1.0);
    let max_cos = cos_a.max(cos_b).max(cos_c);
    max_cos.acos()
}

fn has_t_junction_on_segment(
    pa: Vector3<f64>,
    pb: Vector3<f64>,
    ignore: &[u32],
    active_verts: &[u32],
    canon_pos: &[Vector3<f64>],
    t_eps: f64,
    t_eps_sq: f64,
) -> bool {
    let ab = pb - pa;
    let ab_len_sq = ab.norm_squared();
    if ab_len_sq <= 1e-24 {
        return true;
    }
    let min_x = pa.x.min(pb.x) - t_eps;
    let max_x = pa.x.max(pb.x) + t_eps;
    let min_y = pa.y.min(pb.y) - t_eps;
    let max_y = pa.y.max(pb.y) + t_eps;
    let min_z = pa.z.min(pb.z) - t_eps;
    let max_z = pa.z.max(pb.z) + t_eps;

    for &v in active_verts {
        if ignore.contains(&v) {
            continue;
        }
        let p = canon_pos[v as usize];
        if p.x < min_x
            || p.x > max_x
            || p.y < min_y
            || p.y > max_y
            || p.z < min_z
            || p.z > max_z
        {
            continue;
        }
        let ap = p - pa;
        let t = ap.dot(&ab) / ab_len_sq;
        if t > 0.04 && t < 0.96 {
            let perp_sq = (ap - ab * t).norm_squared();
            if perp_sq < t_eps_sq {
                return true;
            }
        }
    }
    false
}

fn collect_active_verts(tris: &[([u32; 3], Vector3<f64>, u32)]) -> Vec<u32> {
    let mut verts: Vec<u32> = Vec::with_capacity(tris.len() * 3);
    for (t, _, _) in tris {
        verts.push(t[0]);
        verts.push(t[1]);
        verts.push(t[2]);
    }
    verts.sort_unstable();
    verts.dedup();
    verts
}

fn has_sliver_above_budget(
    tris: &[([u32; 3], Vector3<f64>, u32)],
    canon_pos: &[Vector3<f64>],
) -> bool {
    let rad_1 = 1.0_f64.to_radians();
    let rad_5 = 5.0_f64.to_radians();
    let mut lt_5 = 0usize;
    for &(t, _, _) in tris {
        let ang = min_triangle_angle_rad(
            canon_pos[t[0] as usize],
            canon_pos[t[1] as usize],
            canon_pos[t[2] as usize],
        );
        if ang < rad_1 {
            return true;
        }
        if ang < rad_5 {
            lt_5 += 1;
        }
    }
    // Use 0.85% budget (strictly tighter than the 1.00% gate)
    (lt_5 as f64) / (tris.len().max(1) as f64) > 0.0085
}

fn optimize_csg_mesh_quality(
    tris: &mut Vec<([u32; 3], Vector3<f64>, u32)>,
    canon_pos: &mut Vec<Vector3<f64>>,
    diag: f64,
) {
    if tris.len() < 2 || tris.len() > 25000 {
        return;
    }

    optimize_coplanar_delaunay_edges(tris, canon_pos, diag);
    if !has_sliver_above_budget(tris, canon_pos) {
        return;
    }

    collapse_redundant_and_sliver_vertices(tris, canon_pos, diag);
    smooth_coplanar_interior_vertices(tris, canon_pos, diag);
    optimize_coplanar_delaunay_edges(tris, canon_pos, diag);
    if !has_sliver_above_budget(tris, canon_pos) {
        return;
    }

    refine_coplanar_sliver_fans_and_edges(tris, canon_pos, diag);
    smooth_coplanar_interior_vertices(tris, canon_pos, diag);
    optimize_coplanar_delaunay_edges(tris, canon_pos, diag);
    if !has_sliver_above_budget(tris, canon_pos) {
        return;
    }

    collapse_redundant_and_sliver_vertices(tris, canon_pos, diag);
    smooth_coplanar_interior_vertices(tris, canon_pos, diag);
    optimize_coplanar_delaunay_edges(tris, canon_pos, diag);
}

/// Optimizes the in-plane 2D position of purely coplanar interior vertices that are incident to
/// small-angle (< 8°) triangles using centroid blending and 8-direction tangent pattern search.
/// Because vertex motion is strictly orthogonal to the plane normal `n0`, 3D surface geometry,
/// volume, watertightness, and face normals are 100% invariant.
fn smooth_coplanar_interior_vertices(
    tris: &mut [([u32; 3], Vector3<f64>, u32)],
    canon_pos: &mut [Vector3<f64>],
    diag: f64,
) {
    let n_tris = tris.len();
    if !(4..=25000).contains(&n_tris) {
        return;
    }
    let t_eps = (diag * 1.2e-4).max(1.2e-7);
    let t_eps_sq = t_eps * t_eps;
    let active_verts = collect_active_verts(tris);

    let mut vert_tris: HashMap<u32, Vec<usize>> = HashMap::new();
    let mut edge_count: HashMap<(u32, u32), u8> = HashMap::with_capacity(n_tris * 2);
    let mut vert_neighbors: HashMap<u32, Vec<u32>> = HashMap::new();

    for (t_idx, (t, _, _)) in tris.iter().enumerate() {
        for e in 0..3 {
            let u = t[e];
            let v = t[(e + 1) % 3];
            vert_tris.entry(u).or_default().push(t_idx);
            let key = if u < v { (u, v) } else { (v, u) };
            *edge_count.entry(key).or_insert(0) += 1;
            vert_neighbors.entry(u).or_default().push(v);
            vert_neighbors.entry(v).or_default().push(u);
        }
    }

    if edge_count.values().any(|&c| c != 2) {
        return;
    }

    for nbrs in vert_neighbors.values_mut() {
        nbrs.sort_unstable();
        nbrs.dedup();
    }

    let dirs_2d: [(f64, f64); 8] = [
        (1.0, 0.0),
        (-1.0, 0.0),
        (0.0, 1.0),
        (0.0, -1.0),
        (std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2),
        (-std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2),
        (std::f64::consts::FRAC_1_SQRT_2, -std::f64::consts::FRAC_1_SQRT_2),
        (-std::f64::consts::FRAC_1_SQRT_2, -std::f64::consts::FRAC_1_SQRT_2),
    ];

    for _pass in 0..3 {
        let mut moved_any = false;

        for &v in &active_verts {
            let Some(inc) = vert_tris.get(&v) else {
                continue;
            };
            let Some(lk_v) = vert_neighbors.get(&v) else {
                continue;
            };
            if inc.len() < 3 || lk_v.len() < 3 {
                continue;
            }
            let n0 = tris[inc[0]].1;
            let fid0 = tris[inc[0]].2;
            if !inc
                .iter()
                .all(|&ti| tris[ti].1.dot(&n0) >= 0.9998 && tris[ti].2 == fid0)
            {
                continue;
            }

            let eval_star_min_angle = |pos_v: Vector3<f64>, c_pos: &[Vector3<f64>]| -> Option<f64> {
                let mut m_ang = f64::INFINITY;
                for &ti in inc {
                    let (t, nt, _) = tris[ti];
                    let p0 = if t[0] == v { pos_v } else { c_pos[t[0] as usize] };
                    let p1 = if t[1] == v { pos_v } else { c_pos[t[1] as usize] };
                    let p2 = if t[2] == v { pos_v } else { c_pos[t[2] as usize] };
                    let cross = (p1 - p0).cross(&(p2 - p0));
                    let len = cross.norm();
                    if len <= 1e-12 || (cross / len).dot(&nt) < 0.9998 {
                        return None;
                    }
                    let ang = min_triangle_angle_rad(p0, p1, p2);
                    if ang < m_ang {
                        m_ang = ang;
                    }
                }
                Some(m_ang)
            };

            let cur_pos = canon_pos[v as usize];
            let Some(cur_min_ang) = eval_star_min_angle(cur_pos, canon_pos) else {
                continue;
            };
            if cur_min_ang >= 8.5_f64.to_radians() {
                continue;
            }

            let mut centroid = Vector3::zeros();
            let mut avg_len = 0.0;
            for &u in lk_v {
                let pu = canon_pos[u as usize];
                centroid += pu;
                avg_len += (pu - cur_pos).norm();
            }
            centroid /= lk_v.len() as f64;
            avg_len /= lk_v.len() as f64;
            centroid -= n0 * (centroid - cur_pos).dot(&n0);

            let ref_axis = if n0.x.abs() < 0.9 {
                Vector3::new(1.0, 0.0, 0.0)
            } else {
                Vector3::new(0.0, 1.0, 0.0)
            };
            let e1 = n0.cross(&ref_axis).normalize();
            let e2 = n0.cross(&e1).normalize();

            let mut candidate_pts: Vec<Vector3<f64>> = Vec::with_capacity(28);
            for alpha in [0.25, 0.5, 0.75, 1.0] {
                candidate_pts.push(cur_pos + (centroid - cur_pos) * alpha);
            }
            for step_frac in [0.32, 0.15, 0.06] {
                let step = avg_len * step_frac;
                for &(dx, dy) in &dirs_2d {
                    candidate_pts.push(cur_pos + e1 * (dx * step) + e2 * (dy * step));
                }
            }

            let mut best_pos = cur_pos;
            let mut best_ang = cur_min_ang;

            for cand in candidate_pts {
                let cand_proj = cand - n0 * (cand - cur_pos).dot(&n0);
                let Some(ang) = eval_star_min_angle(cand_proj, canon_pos) else {
                    continue;
                };
                if ang <= best_ang + 2e-4 {
                    continue;
                }
                let mut ok = true;
                for &u in lk_v {
                    let pu = canon_pos[u as usize];
                    if has_t_junction_on_segment(
                        cand_proj,
                        pu,
                        &[v, u],
                        &active_verts,
                        canon_pos,
                        t_eps,
                        t_eps_sq,
                    ) {
                        ok = false;
                        break;
                    }
                }
                if ok {
                    best_ang = ang;
                    best_pos = cand_proj;
                }
            }

            if best_ang > cur_min_ang + 2e-4 {
                canon_pos[v as usize] = best_pos;
                moved_any = true;
            }
        }

        if !moved_any {
            break;
        }
    }
}

/// Performs iterative Lawson Delaunay diagonal flips on adjacent coplanar triangles
/// using both the exact 3D coplanar cotangent incircle criterion and max-min angle improvement.
fn optimize_coplanar_delaunay_edges(
    tris: &mut [([u32; 3], Vector3<f64>, u32)],
    canon_pos: &[Vector3<f64>],
    diag: f64,
) {
    let n_tris = tris.len();
    if !(2..=25000).contains(&n_tris) {
        return;
    }
    let t_eps = (diag * 1.2e-4).max(1.2e-7);
    let t_eps_sq = t_eps * t_eps;
    let active_verts = collect_active_verts(tris);

    for _pass in 0..8 {
        let mut flipped_in_pass = 0usize;
        let mut half_edge_map: HashMap<(u32, u32), (usize, usize)> =
            HashMap::with_capacity(n_tris * 3);
        let mut edge_total_count: HashMap<(u32, u32), u8> = HashMap::with_capacity(n_tris * 2);

        for (t_idx, (t, _, _)) in tris.iter().enumerate() {
            for e in 0..3 {
                let u = t[e];
                let v = t[(e + 1) % 3];
                half_edge_map.insert((u, v), (t_idx, e));
                let key = if u < v { (u, v) } else { (v, u) };
                *edge_total_count.entry(key).or_insert(0) += 1;
            }
        }

        let mut locked_tris = vec![false; n_tris];

        for t1_idx in 0..n_tris {
            if locked_tris[t1_idx] {
                continue;
            }
            let (t1, n1, fid1) = tris[t1_idx];
            for e1 in 0..3 {
                let u = t1[e1];
                let v = t1[(e1 + 1) % 3];
                let w = t1[(e1 + 2) % 3];
                let key = if u < v { (u, v) } else { (v, u) };
                if edge_total_count.get(&key).copied().unwrap_or(0) != 2 {
                    continue;
                }
                let Some(&(t2_idx, e2)) = half_edge_map.get(&(v, u)) else {
                    continue;
                };
                if t2_idx <= t1_idx || locked_tris[t2_idx] {
                    continue;
                }
                let (t2, n2, fid2) = tris[t2_idx];
                if fid1 != fid2 || n1.dot(&n2) < 0.9998 {
                    continue;
                }
                let x = t2[(e2 + 2) % 3];
                if x == w || x == u || x == v {
                    continue;
                }
                let new_key = if w < x { (w, x) } else { (x, w) };
                if edge_total_count.contains_key(&new_key) {
                    continue;
                }

                let pu = canon_pos[u as usize];
                let pv = canon_pos[v as usize];
                let pw = canon_pos[w as usize];
                let px = canon_pos[x as usize];

                // Check strict convexity of quad (u, x, v, w) in plane n1
                let c_new1 = (px - pw).cross(&(pu - pw));
                let c_new2 = (pw - px).cross(&(pv - px));
                let a_new1 = c_new1.dot(&n1);
                let a_new2 = c_new2.dot(&n1);
                if a_new1 <= 1e-12 || a_new2 <= 1e-12 {
                    continue;
                }
                if (c_new1.normalize()).dot(&n1) < 0.999 || (c_new2.normalize()).dot(&n1) < 0.999 {
                    continue;
                }

                let min_before =
                    min_triangle_angle_rad(pu, pv, pw).min(min_triangle_angle_rad(pv, pu, px));
                let min_after =
                    min_triangle_angle_rad(pw, px, pu).min(min_triangle_angle_rad(px, pw, pv));

                let sin_w = (pu - pw).cross(&(pv - pw)).dot(&n1);
                let sin_x = (pv - px).cross(&(pu - px)).dot(&n1);
                let is_delaunay_flip = if sin_w > 1e-13 && sin_x > 1e-13 {
                    let cot_w = (pu - pw).dot(&(pv - pw)) / sin_w;
                    let cot_x = (pv - px).dot(&(pu - px)) / sin_x;
                    (cot_w + cot_x < -1e-5) && (min_after >= min_before - 1e-4)
                } else {
                    false
                };

                if (min_after > min_before + 1e-4 || is_delaunay_flip)
                    && !has_t_junction_on_segment(
                        pw,
                        px,
                        &[u, v, w, x],
                        &active_verts,
                        canon_pos,
                        t_eps,
                        t_eps_sq,
                    )
                {
                    tris[t1_idx].0 = [w, x, u];
                    tris[t2_idx].0 = [x, w, v];
                    locked_tris[t1_idx] = true;
                    locked_tris[t2_idx] = true;
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

type EarClippedTriangle = ([u32; 3], Vector3<f64>, u32);

///// Ear-clips a closed 3D planar polygon loop `ring` with unit normal `n_plane` into `ring.len() - 2`
/// triangles using greedy max-min-angle ear selection, ensuring no duplicate edges or T-junctions.
#[allow(clippy::too_many_arguments)]
fn ear_clip_planar_ring_maxmin(
    ring: &[u32],
    n_plane: Vector3<f64>,
    face_id: u32,
    canon_pos: &[Vector3<f64>],
    forbidden_undirected_edges: &HashSet<(u32, u32)>,
    active_verts: &[u32],
    ignore_verts: &[u32],
    t_eps: f64,
    t_eps_sq: f64,
) -> Option<Vec<EarClippedTriangle>> {
    if ring.len() < 3 {
        return None;
    }
    let mut cur: Vec<u32> = ring.to_vec();
    let mut out: Vec<EarClippedTriangle> = Vec::with_capacity(ring.len() - 2);

    while cur.len() > 3 {
        let m = cur.len();
        let mut best_ear: Option<(usize, f64)> = None;

        for i in 0..m {
            let prev = cur[(i + m - 1) % m];
            let curr = cur[i];
            let next = cur[(i + 1) % m];
            if prev == curr || curr == next || prev == next {
                continue;
            }
            // New interior diagonal is (prev, next); check if it is forbidden
            let diag_key = if prev < next {
                (prev, next)
            } else {
                (next, prev)
            };
            if forbidden_undirected_edges.contains(&diag_key) {
                continue;
            }

            let p_prev = canon_pos[prev as usize];
            let p_curr = canon_pos[curr as usize];
            let p_next = canon_pos[next as usize];

            let cross = (p_curr - p_prev).cross(&(p_next - p_prev));
            let len = cross.norm();
            if len <= 1e-12 || (cross / len).dot(&n_plane) < 0.9995 {
                continue;
            }

            // Ensure no other vertex of `cur` lies inside or on the boundary of ear (p_prev, p_curr, p_next)
            let mut ear_empty = true;
            for &other in cur.iter().take(m) {
                if other == prev || other == curr || other == next {
                    continue;
                }
                let p_o = canon_pos[other as usize];
                let c0 = (p_curr - p_prev).cross(&(p_o - p_prev)).dot(&n_plane);
                let c1 = (p_next - p_curr).cross(&(p_o - p_curr)).dot(&n_plane);
                let c2 = (p_prev - p_next).cross(&(p_o - p_next)).dot(&n_plane);
                if c0 >= -1e-12 && c1 >= -1e-12 && c2 >= -1e-12 {
                    ear_empty = false;
                    break;
                }
            }
            if !ear_empty {
                continue;
            }

            if has_t_junction_on_segment(
                p_prev,
                p_next,
                ignore_verts,
                active_verts,
                canon_pos,
                t_eps,
                t_eps_sq,
            ) {
                continue;
            }

            let ang = min_triangle_angle_rad(p_prev, p_curr, p_next);
            if best_ear.is_none_or(|(_, b_ang)| ang > b_ang) {
                best_ear = Some((i, ang));
            }
        }

        let (ear_idx, _) = best_ear?;
        let prev = cur[(ear_idx + m - 1) % m];
        let curr = cur[ear_idx];
        let next = cur[(ear_idx + 1) % m];
        out.push(([prev, curr, next], n_plane, face_id));
        cur.remove(ear_idx);
    }

    let p0 = canon_pos[cur[0] as usize];
    let p1 = canon_pos[cur[1] as usize];
    let p2 = canon_pos[cur[2] as usize];
    let cross = (p1 - p0).cross(&(p2 - p0));
    let len = cross.norm();
    if len <= 1e-12 || (cross / len).dot(&n_plane) < 0.9995 {
        return None;
    }
    out.push(([cur[0], cur[1], cur[2]], n_plane, face_id));
    Some(out)
}

/// Attempts to remove a redundant vertex `v` (either a purely coplanar interior vertex or a
/// 2-plane collinear crease vertex) by ear-clipping its 1-ring polygon(s).
#[allow(clippy::too_many_arguments)]
fn try_remove_vertex_ear_clip(
    v: u32,
    inc: &[usize],
    tris: &[EarClippedTriangle],
    canon_pos: &[Vector3<f64>],
    edge_tris_map: &HashMap<(u32, u32), Vec<usize>>,
    active_verts: &[u32],
    min_before: f64,
    sliver_count_before: usize,
    sliver_rad: f64,
    t_eps: f64,
    t_eps_sq: f64,
) -> Option<Vec<EarClippedTriangle>> {
    if inc.len() < 3 || inc.len() > 16 {
        return None;
    }
    let inc_set: HashSet<usize> = inc.iter().copied().collect();

    // Collect undirected edges that already exist OUTSIDE `inc` so ear-clipping never creates a non-manifold edge
    let mut ring_verts: Vec<u32> = Vec::with_capacity(inc.len() + 2);
    for &ti in inc {
        for &vid in &tris[ti].0 {
            if vid != v && !ring_verts.contains(&vid) {
                ring_verts.push(vid);
            }
        }
    }
    let mut forbidden_edges: HashSet<(u32, u32)> = HashSet::new();
    for i in 0..ring_verts.len() {
        for j in (i + 1)..ring_verts.len() {
            let a = ring_verts[i];
            let b = ring_verts[j];
            let key = if a < b { (a, b) } else { (b, a) };
            if let Some(t_list) = edge_tris_map.get(&key) {
                if t_list.iter().any(|ti| !inc_set.contains(ti)) {
                    forbidden_edges.insert(key);
                }
            }
        }
    }

    let mut ignore_verts = ring_verts.clone();
    ignore_verts.push(v);

    let n0 = tris[inc[0]].1;
    let fid0 = tris[inc[0]].2;
    let is_purely_coplanar = inc.iter().all(|&ti| tris[ti].1.dot(&n0) >= 0.9998);

    if is_purely_coplanar {
        if !inc.iter().all(|&ti| tris[ti].2 == fid0) {
            return None;
        }
        // Build single closed loop of directed edges opposite `v`
        let mut opp_edges: Vec<(u32, u32)> = Vec::with_capacity(inc.len());
        for &ti in inc {
            let t = tris[ti].0;
            if t[0] == v {
                opp_edges.push((t[1], t[2]));
            } else if t[1] == v {
                opp_edges.push((t[2], t[0]));
            } else if t[2] == v {
                opp_edges.push((t[0], t[1]));
            } else {
                return None;
            }
        }
        let mut ring: Vec<u32> = Vec::with_capacity(inc.len());
        let (start_u, mut curr_u) = opp_edges.pop()?;
        ring.push(start_u);
        while !opp_edges.is_empty() {
            if ring.contains(&curr_u) {
                return None;
            }
            ring.push(curr_u);
            let pos = opp_edges.iter().position(|&(a, _)| a == curr_u)?;
            curr_u = opp_edges.swap_remove(pos).1;
        }
        if curr_u != start_u || ring.len() != inc.len() {
            return None;
        }

        let new_tris = ear_clip_planar_ring_maxmin(
            &ring,
            n0,
            fid0,
            canon_pos,
            &forbidden_edges,
            active_verts,
            &ignore_verts,
            t_eps,
            t_eps_sq,
        )?;

        let mut min_after = f64::INFINITY;
        let mut sliver_count_after = 0usize;
        for &(t, _, _) in &new_tris {
            let ang = min_triangle_angle_rad(
                canon_pos[t[0] as usize],
                canon_pos[t[1] as usize],
                canon_pos[t[2] as usize],
            );
            if ang < min_after {
                min_after = ang;
            }
            if ang < sliver_rad {
                sliver_count_after += 1;
            }
        }

        // Accept if removing the coplanar BSP vertex keeps healthy angles, reduces slivers, or
        // simplifies a sliver fan without creating < 1.0° needles
        let accept = min_after >= 5.2_f64.to_radians()
            || (sliver_count_after < sliver_count_before && min_after >= min_before - 1e-4)
            || (min_before < 5.2_f64.to_radians() && min_after > min_before + 1e-4)
            || (min_before < 5.2_f64.to_radians()
                && sliver_count_after <= sliver_count_before
                && min_after >= 1.05_f64.to_radians());
        if accept {
            return Some(new_tris);
        }
        return None;
    }

    // Check Case 2: 2-plane collinear crease vertex
    let mut g1: Vec<usize> = Vec::new();
    let mut g2: Vec<usize> = Vec::new();
    let mut n2_opt: Option<Vector3<f64>> = None;
    let mut fid2_opt: Option<u32> = None;
    for &ti in inc {
        let (_, nt, fid_t) = tris[ti];
        if nt.dot(&n0) >= 0.9998 {
            if fid_t != fid0 {
                return None;
            }
            g1.push(ti);
        } else if let Some(n2) = n2_opt {
            if nt.dot(&n2) >= 0.9998 && Some(fid_t) == fid2_opt {
                g2.push(ti);
            } else {
                return None;
            }
        } else {
            n2_opt = Some(nt);
            fid2_opt = Some(fid_t);
            g2.push(ti);
        }
    }
    let n2 = n2_opt?;
    let fid2 = fid2_opt?;
    if g1.len() < 2 || g2.len() < 2 {
        return None;
    }

    let build_open_chain = |group: &[usize]| -> Option<Vec<u32>> {
        let mut edges: Vec<(u32, u32)> = Vec::with_capacity(group.len());
        for &ti in group {
            let t = tris[ti].0;
            if t[0] == v {
                edges.push((t[1], t[2]));
            } else if t[1] == v {
                edges.push((t[2], t[0]));
            } else if t[2] == v {
                edges.push((t[0], t[1]));
            } else {
                return None;
            }
        }
        let right_set: HashSet<u32> = edges.iter().map(|&(_, r)| r).collect();
        let start_idx = edges.iter().position(|&(l, _)| !right_set.contains(&l))?;
        let (start_u, mut curr_u) = edges.swap_remove(start_idx);
        let mut chain = vec![start_u, curr_u];
        while !edges.is_empty() {
            let pos = edges.iter().position(|&(l, _)| l == curr_u)?;
            curr_u = edges.swap_remove(pos).1;
            if chain.contains(&curr_u) {
                return None;
            }
            chain.push(curr_u);
        }
        Some(chain)
    };

    let chain1 = build_open_chain(&g1)?;
    let chain2 = build_open_chain(&g2)?;
    let u_start = *chain1.first()?;
    let u_end = *chain1.last()?;
    if *chain2.first()? != u_end || *chain2.last()? != u_start {
        return None;
    }

    // Verify `v` lies strictly on the straight segment `(u_start, u_end)`
    let pv = canon_pos[v as usize];
    let p_start = canon_pos[u_start as usize];
    let p_end = canon_pos[u_end as usize];
    let se = p_end - p_start;
    let se_len_sq = se.norm_squared();
    if se_len_sq <= 1e-20 {
        return None;
    }
    let t_proj = (pv - p_start).dot(&se) / se_len_sq;
    if t_proj <= 0.01 || t_proj >= 0.99 {
        return None;
    }
    let perp_sq = ((pv - p_start) - se * t_proj).norm_squared();
    let col_tol_sq = (1e-6 * se_len_sq).max(t_eps_sq * 0.5);
    if perp_sq > col_tol_sq {
        return None;
    }

    let crease_key = if u_start < u_end {
        (u_start, u_end)
    } else {
        (u_end, u_start)
    };
    if forbidden_edges.contains(&crease_key) {
        return None;
    }
    if has_t_junction_on_segment(
        p_start,
        p_end,
        &[v, u_start, u_end],
        active_verts,
        canon_pos,
        t_eps,
        t_eps_sq,
    ) {
        return None;
    }

    let mut tris1 = ear_clip_planar_ring_maxmin(
        &chain1,
        n0,
        fid0,
        canon_pos,
        &forbidden_edges,
        active_verts,
        &ignore_verts,
        t_eps,
        t_eps_sq,
    )?;
    let tris2 = ear_clip_planar_ring_maxmin(
        &chain2,
        n2,
        fid2,
        canon_pos,
        &forbidden_edges,
        active_verts,
        &ignore_verts,
        t_eps,
        t_eps_sq,
    )?;
    tris1.extend(tris2);

    let mut min_after = f64::INFINITY;
    let mut sliver_count_after = 0usize;
    for &(t, _, _) in &tris1 {
        let ang = min_triangle_angle_rad(
            canon_pos[t[0] as usize],
            canon_pos[t[1] as usize],
            canon_pos[t[2] as usize],
        );
        if ang < min_after {
            min_after = ang;
        }
        if ang < sliver_rad {
            sliver_count_after += 1;
        }
    }

    let accept = min_after >= 5.2_f64.to_radians()
        || (sliver_count_after < sliver_count_before && min_after >= min_before - 1e-4)
        || (min_before < 5.2_f64.to_radians() && min_after > min_before + 1e-4)
        || (min_before < 5.2_f64.to_radians()
            && sliver_count_after <= sliver_count_before
            && min_after >= 1.05_f64.to_radians());
    if accept {
        Some(tris1)
    } else {
        None
    }
}

/// Collapses or removes redundant plane-preserving BSP cut vertices (both interior coplanar
/// vertices and collinear crease vertices) and micro-edges opposite < 5° sliver angles while
/// preserving exact 2-manifold Link Condition and T-junction-free topology.
fn collapse_redundant_and_sliver_vertices(
    tris: &mut Vec<([u32; 3], Vector3<f64>, u32)>,
    canon_pos: &[Vector3<f64>],
    diag: f64,
) {
    let t_eps = (diag * 1.2e-4).max(1.2e-7);
    let t_eps_sq = t_eps * t_eps;
    let sliver_rad = 5.0_f64.to_radians();

    for outer in 0..14 {
        let n_tris = tris.len();
        if n_tris < 4 {
            break;
        }
        if outer >= 3 && !has_sliver_above_budget(tris, canon_pos) {
            break;
        }

        let mut vert_tris: HashMap<u32, Vec<usize>> = HashMap::new();
        let mut edge_tris_map: HashMap<(u32, u32), Vec<usize>> = HashMap::with_capacity(n_tris * 2);
        let mut vert_neighbors: HashMap<u32, Vec<u32>> = HashMap::new();

        for (t_idx, (t, _, _)) in tris.iter().enumerate() {
            for e in 0..3 {
                let u = t[e];
                let v = t[(e + 1) % 3];
                vert_tris.entry(u).or_default().push(t_idx);
                let key = if u < v { (u, v) } else { (v, u) };
                edge_tris_map.entry(key).or_default().push(t_idx);
                vert_neighbors.entry(u).or_default().push(v);
                vert_neighbors.entry(v).or_default().push(u);
            }
        }

        // Only operate on closed manifold meshes
        if edge_tris_map.values().any(|list| list.len() != 2) {
            break;
        }

        for nbrs in vert_neighbors.values_mut() {
            nbrs.sort_unstable();
            nbrs.dedup();
        }

        let active_verts: Vec<u32> = {
            let mut v: Vec<u32> = vert_tris.keys().copied().collect();
            v.sort_unstable();
            v
        };

        // Sort candidate vertices so those incident to the smallest angles are tried first
        let mut candidates: Vec<(f64, u32)> = Vec::with_capacity(active_verts.len());
        for &v in &active_verts {
            let inc = &vert_tris[&v];
            let mut min_ang = f64::INFINITY;
            for &ti in inc {
                let [a, b, c] = tris[ti].0;
                let ang = min_triangle_angle_rad(
                    canon_pos[a as usize],
                    canon_pos[b as usize],
                    canon_pos[c as usize],
                );
                if ang < min_ang {
                    min_ang = ang;
                }
            }
            candidates.push((min_ang, v));
        }
        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let mut locked_verts: HashSet<u32> = HashSet::new();
        let mut removed_tris: HashSet<usize> = HashSet::new();
        let mut added_tris: Vec<([u32; 3], Vector3<f64>, u32)> = Vec::new();
        let mut collapsed_in_pass = 0usize;

        for &(min_before, v) in &candidates {
            if locked_verts.contains(&v) {
                continue;
            }
            let Some(inc) = vert_tris.get(&v) else {
                continue;
            };
            let Some(lk_v) = vert_neighbors.get(&v) else {
                continue;
            };
            if inc.len() < 3 || lk_v.len() < 3 {
                continue;
            }
            if lk_v.iter().any(|w| locked_verts.contains(w))
                || inc.iter().any(|ti| removed_tris.contains(ti))
            {
                continue;
            }

            let pv = canon_pos[v as usize];
            let n0 = tris[inc[0]].1;
            let fid0 = tris[inc[0]].2;
            let is_purely_coplanar = inc.iter().all(|&ti| tris[ti].1.dot(&n0) >= 0.9998);
            if is_purely_coplanar && !inc.iter().all(|&ti| tris[ti].2 == fid0) {
                continue;
            }

            if (!is_purely_coplanar && min_before >= 15.0_f64.to_radians())
                || (is_purely_coplanar && outer >= 1 && min_before >= 10.0_f64.to_radians())
            {
                continue;
            }

            let sliver_count_before = inc
                .iter()
                .filter(|&&ti| {
                    let [a, b, c] = tris[ti].0;
                    min_triangle_angle_rad(
                        canon_pos[a as usize],
                        canon_pos[b as usize],
                        canon_pos[c as usize],
                    ) < sliver_rad
                })
                .count();

            // 1. Try 1-ring ear-clipping removal first (works even when single-edge collapse is blocked)
            if let Some(ear_tris) = try_remove_vertex_ear_clip(
                v,
                inc,
                tris,
                canon_pos,
                &edge_tris_map,
                &active_verts,
                min_before,
                sliver_count_before,
                sliver_rad,
                t_eps,
                t_eps_sq,
            ) {
                locked_verts.insert(v);
                for &w in lk_v {
                    locked_verts.insert(w);
                }
                for &ti in inc {
                    removed_tris.insert(ti);
                }
                added_tris.extend(ear_tris);
                collapsed_in_pass += 1;
                continue;
            }

            // 2. Fallback: Try manifold edge collapse v -> u
            let mut best_target: Option<(u32, f64)> = None;
            let plane_tol = (diag * 6e-5).max(2e-7);

            for &u in lk_v {
                let pu = canon_pos[u as usize];
                let delta = pu - pv;
                let dist = delta.norm();
                if dist <= 1e-13 {
                    continue;
                }

                let strictly_plane_preserving = inc
                    .iter()
                    .all(|&ti| delta.dot(&tris[ti].1).abs() <= (5e-4 * dist).max(plane_tol));
                let micro_sliver_collapse = !strictly_plane_preserving
                    && min_before < sliver_rad
                    && dist <= 0.04 * diag
                    && inc
                        .iter()
                        .all(|&ti| delta.dot(&tris[ti].1).abs() <= 5e-4 * diag);

                if !strictly_plane_preserving && !micro_sliver_collapse {
                    continue;
                }

                let Some(lk_u) = vert_neighbors.get(&u) else {
                    continue;
                };
                let mut common: Vec<u32> = Vec::new();
                for &w in lk_v {
                    if lk_u.binary_search(&w).is_ok() {
                        common.push(w);
                    }
                }
                if common.len() != 2 {
                    continue;
                }
                if common.iter().any(|w| {
                    vert_neighbors
                        .get(w)
                        .map(|n| n.len() <= 3)
                        .unwrap_or(true)
                }) {
                    continue;
                }

                let mut valid = true;
                let mut surviving_count = 0usize;
                let mut min_after = f64::INFINITY;
                let mut sliver_count_after = 0usize;
                let min_norm_dot = if strictly_plane_preserving {
                    0.9995
                } else {
                    0.990
                };

                for &ti in inc {
                    let (t, n_t, _) = tris[ti];
                    if t.contains(&u) {
                        continue;
                    }
                    surviving_count += 1;
                    let nt = [
                        if t[0] == v { u } else { t[0] },
                        if t[1] == v { u } else { t[1] },
                        if t[2] == v { u } else { t[2] },
                    ];
                    let p0 = canon_pos[nt[0] as usize];
                    let p1 = canon_pos[nt[1] as usize];
                    let p2 = canon_pos[nt[2] as usize];
                    let cross = (p1 - p0).cross(&(p2 - p0));
                    let len = cross.norm();
                    if len <= 1e-12 || (cross / len).dot(&n_t) < min_norm_dot {
                        valid = false;
                        break;
                    }
                    let ang = min_triangle_angle_rad(p0, p1, p2);
                    if ang < min_after {
                        min_after = ang;
                    }
                    if ang < sliver_rad {
                        sliver_count_after += 1;
                    }
                }

                if !valid || surviving_count == 0 {
                    continue;
                }

                let acceptable = if strictly_plane_preserving {
                    (sliver_count_after < sliver_count_before && min_after >= min_before - 1e-4)
                        || (min_before < 5.2_f64.to_radians() && min_after > min_before + 1e-4)
                        || (is_purely_coplanar && min_after >= 5.2_f64.to_radians())
                        || (!is_purely_coplanar && min_after >= 8.0_f64.to_radians())
                } else {
                    sliver_count_after < sliver_count_before && min_after > min_before + 3e-3
                };

                if !acceptable {
                    continue;
                }

                for &w in lk_v {
                    if w == u {
                        continue;
                    }
                    let pw = canon_pos[w as usize];
                    if has_t_junction_on_segment(
                        pu,
                        pw,
                        &[v, u, w],
                        &active_verts,
                        canon_pos,
                        t_eps,
                        t_eps_sq,
                    ) {
                        valid = false;
                        break;
                    }
                }
                if !valid {
                    continue;
                }

                if best_target.is_none_or(|(_, b_ang)| min_after > b_ang) {
                    best_target = Some((u, min_after));
                }
            }

            if let Some((u, _)) = best_target {
                locked_verts.insert(v);
                locked_verts.insert(u);
                for &w in lk_v {
                    locked_verts.insert(w);
                }
                if let Some(lk_u) = vert_neighbors.get(&u) {
                    for &w in lk_u {
                        locked_verts.insert(w);
                    }
                }
                for &ti in inc {
                    removed_tris.insert(ti);
                    let (t, n_t, fid_t) = tris[ti];
                    if t.contains(&u) {
                        continue;
                    }
                    let nt = [
                        if t[0] == v { u } else { t[0] },
                        if t[1] == v { u } else { t[1] },
                        if t[2] == v { u } else { t[2] },
                    ];
                    let p0 = canon_pos[nt[0] as usize];
                    let p1 = canon_pos[nt[1] as usize];
                    let p2 = canon_pos[nt[2] as usize];
                    let cross = (p1 - p0).cross(&(p2 - p0));
                    let len = cross.norm();
                    let updated_n = if len > 1e-12 { cross / len } else { n_t };
                    added_tris.push((nt, updated_n, fid_t));
                }
                collapsed_in_pass += 1;
            }
        }

        if collapsed_in_pass > 0 {
            let mut next_tris = Vec::with_capacity(tris.len());
            for (ti, item) in tris.iter().enumerate() {
                if !removed_tris.contains(&ti) {
                    next_tris.push(*item);
                }
            }
            next_tris.extend(added_tris);
            *tris = next_tris;
            optimize_coplanar_delaunay_edges(tris, canon_pos, diag);
        } else {
            break;
        }
    }
}

/// Resolves remaining < 5° coplanar fan slivers (such as hole-rim chords fanning to distant box
/// corners) by inserting a Steiner hub inside contiguous coplanar fans around the narrow apex,
/// or splitting shared edges opposite narrow/obtuse angles, followed by Delaunay edge flips.
fn refine_coplanar_sliver_fans_and_edges(
    tris: &mut Vec<([u32; 3], Vector3<f64>, u32)>,
    canon_pos: &mut Vec<Vector3<f64>>,
    diag: f64,
) {
    let t_eps = (diag * 1.2e-4).max(1.2e-7);
    let t_eps_sq = t_eps * t_eps;
    let target_rad = 5.05_f64.to_radians();

    let mut unrefinable_tris: HashSet<[u32; 3]> = HashSet::new();

    for iter in 0..96 {
        if iter >= 8 && iter % 4 == 0 && !has_sliver_above_budget(tris, canon_pos) {
            break;
        }
        let mut worst: Option<(usize, f64)> = None;
        for (idx, (t, _, _)) in tris.iter().enumerate() {
            let mut key = *t;
            key.sort_unstable();
            if unrefinable_tris.contains(&key) {
                continue;
            }
            let ang = min_triangle_angle_rad(
                canon_pos[t[0] as usize],
                canon_pos[t[1] as usize],
                canon_pos[t[2] as usize],
            );
            if ang < target_rad && worst.is_none_or(|(_, w_ang)| ang < w_ang) {
                worst = Some((idx, ang));
            }
        }
        let Some((seed_idx, _seed_ang)) = worst else {
            break;
        };

        let active_verts = collect_active_verts(tris);
        let mut modified = false;

        // 1. Try Coplanar Apex-Fan Steiner Hub Insertion around the narrowest vertex of seed_idx
        let (seed_tri, seed_n, seed_fid) = tris[seed_idx];
        let mut apex_local = 0usize;
        let mut apex_min_ang = f64::INFINITY;
        for i in 0..3 {
            let pa = canon_pos[seed_tri[i] as usize];
            let pb = canon_pos[seed_tri[(i + 1) % 3] as usize];
            let pc = canon_pos[seed_tri[(i + 2) % 3] as usize];
            let d1 = (pb - pa).normalize();
            let d2 = (pc - pa).normalize();
            let ang = d1.dot(&d2).clamp(-1.0, 1.0).acos();
            if ang < apex_min_ang {
                apex_min_ang = ang;
                apex_local = i;
            }
        }
        let c_apex = seed_tri[apex_local];
        let p_c = canon_pos[c_apex as usize];
        let plane_verts: Vec<u32> = active_verts
            .iter()
            .copied()
            .filter(|&av| (canon_pos[av as usize] - p_c).dot(&seed_n).abs() <= t_eps * 2.5)
            .collect();

        let mut fan_candidates: Vec<(usize, u32, u32)> = Vec::new();
        for (ti, &(t, nt, fid_t)) in tris.iter().enumerate() {
            if fid_t != seed_fid || nt.dot(&seed_n) < 0.9998 {
                continue;
            }
            if t[0] == c_apex {
                fan_candidates.push((ti, t[1], t[2]));
            } else if t[1] == c_apex {
                fan_candidates.push((ti, t[2], t[0]));
            } else if t[2] == c_apex {
                fan_candidates.push((ti, t[0], t[1]));
            }
        }

        if let Some(&(_, seed_l, seed_r)) = fan_candidates.iter().find(|&&(ti, _, _)| ti == seed_idx)
        {
            let mut chain_tris: Vec<usize> = vec![seed_idx];
            let mut chain_rim: Vec<u32> = vec![seed_l, seed_r];

            loop {
                let last_r = *chain_rim.last().unwrap();
                if let Some(&(next_ti, _, next_r)) = fan_candidates.iter().find(|&&(ti, l, r)| {
                    l == last_r && !chain_tris.contains(&ti) && !chain_rim.contains(&r)
                }) {
                    chain_tris.push(next_ti);
                    chain_rim.push(next_r);
                } else {
                    break;
                }
            }
            loop {
                let first_l = chain_rim[0];
                if let Some(&(prev_ti, prev_l, _)) = fan_candidates.iter().find(|&&(ti, l, r)| {
                    r == first_l && !chain_tris.contains(&ti) && !chain_rim.contains(&l)
                }) {
                    chain_tris.insert(0, prev_ti);
                    chain_rim.insert(0, prev_l);
                } else {
                    break;
                }
            }

            let m = chain_tris.len();
            if m >= 2 {
                let chain_angles: Vec<f64> = chain_tris
                    .iter()
                    .map(|&ti| {
                        let [v0, v1, v2] = tris[ti].0;
                        min_triangle_angle_rad(
                            canon_pos[v0 as usize],
                            canon_pos[v1 as usize],
                            canon_pos[v2 as usize],
                        )
                    })
                    .collect();

                let seed_pos = chain_tris.iter().position(|&ti| ti == seed_idx).unwrap();
                let mut best_hub: Option<(usize, usize, Vector3<f64>, f64)> = None;

                for a in 0..=seed_pos {
                    for b in (seed_pos + 1)..=m {
                        let span = b - a;
                        if !(2..=10).contains(&span) {
                            continue;
                        }
                        // Never strand a 1-triangle sliver tail at the start (0..a) or end (b..m)
                        if a == 1 && chain_angles[0] < target_rad {
                            continue;
                        }
                        if m - b == 1 && chain_angles[m - 1] < target_rad {
                            continue;
                        }

                        let old_subfan_min = chain_angles[a..b]
                            .iter()
                            .copied()
                            .fold(f64::INFINITY, f64::min);

                        let mid_idx = (a + b) / 2;
                        let p_mid = canon_pos[chain_rim[mid_idx] as usize];
                        let mut avg_rim = Vector3::zeros();
                        for &rv in &chain_rim[a..=b] {
                            avg_rim += canon_pos[rv as usize];
                        }
                        avg_rim /= (span + 1) as f64;

                        for target_pt in [p_mid, avg_rim] {
                            for tau in [0.48, 0.55, 0.40, 0.60, 0.32] {
                                let raw_s = p_c + (target_pt - p_c) * tau;
                                let s_pt = raw_s - seed_n * (raw_s - p_c).dot(&seed_n);

                                let mut sub_valid = true;
                                let mut new_min_ang = f64::INFINITY;

                                let mut loop_v: Vec<u32> = Vec::with_capacity(span + 2);
                                loop_v.push(c_apex);
                                loop_v.extend_from_slice(&chain_rim[a..=b]);

                                for k in 0..loop_v.len() {
                                    let va = loop_v[k];
                                    let vb = loop_v[(k + 1) % loop_v.len()];
                                    let pa = canon_pos[va as usize];
                                    let pb = canon_pos[vb as usize];
                                    let cross = (pa - s_pt).cross(&(pb - s_pt));
                                    let len = cross.norm();
                                    if len <= 1e-12 || (cross / len).dot(&seed_n) < 0.999 {
                                        sub_valid = false;
                                        break;
                                    }
                                    let ang = min_triangle_angle_rad(s_pt, pa, pb);
                                    if ang < new_min_ang {
                                        new_min_ang = ang;
                                    }
                                }
                                if !sub_valid || new_min_ang <= old_subfan_min + 2e-3 {
                                    continue;
                                }

                                for &va in &loop_v {
                                    let pa = canon_pos[va as usize];
                                    if has_t_junction_on_segment(
                                        s_pt,
                                        pa,
                                        &loop_v,
                                        &plane_verts,
                                        canon_pos,
                                        t_eps,
                                        t_eps_sq,
                                    ) {
                                        sub_valid = false;
                                        break;
                                    }
                                }
                                if !sub_valid {
                                    continue;
                                }

                                // Score prefers covering all slivers in the fan when new_min_ang >= 5.2°
                                let covers_all = (0..a).all(|i| chain_angles[i] >= target_rad)
                                    && (b..m).all(|i| chain_angles[i] >= target_rad);
                                let score = if covers_all && new_min_ang >= 5.2_f64.to_radians() {
                                    new_min_ang + 1.0
                                } else {
                                    new_min_ang
                                };

                                if best_hub.is_none_or(|(_, _, _, b_score)| score > b_score) {
                                    best_hub = Some((a, b, s_pt, score));
                                }
                            }
                        }
                    }
                }

                if let Some((a, b, s_pt, _)) = best_hub {
                    let s_idx = canon_pos.len() as u32;
                    canon_pos.push(s_pt);
                    let remove_set: HashSet<usize> = chain_tris[a..b].iter().copied().collect();
                    let mut next_tris = Vec::with_capacity(tris.len() + 2);
                    for (ti, item) in tris.iter().enumerate() {
                        if !remove_set.contains(&ti) {
                            next_tris.push(*item);
                        }
                    }
                    let mut loop_v: Vec<u32> = Vec::with_capacity(b - a + 2);
                    loop_v.push(c_apex);
                    loop_v.extend_from_slice(&chain_rim[a..=b]);
                    for k in 0..loop_v.len() {
                        let va = loop_v[k];
                        let vb = loop_v[(k + 1) % loop_v.len()];
                        next_tris.push(([s_idx, va, vb], seed_n, seed_fid));
                    }
                    *tris = next_tris;
                    optimize_coplanar_delaunay_edges(tris, canon_pos, diag);
                    modified = true;
                }
            }
        }

        if modified {
            continue;
        }

        // 2. Fast local 2-triangle / 4-triangle shared-edge split & diagonal flip fallback
        let (t1, n1, fid1) = tris[seed_idx];
        let mut edges_by_len = [
            (0usize, (canon_pos[t1[1] as usize] - canon_pos[t1[0] as usize]).norm()),
            (1usize, (canon_pos[t1[2] as usize] - canon_pos[t1[1] as usize]).norm()),
            (2usize, (canon_pos[t1[0] as usize] - canon_pos[t1[2] as usize]).norm()),
        ];
        edges_by_len.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        for &(e1, _) in &edges_by_len[..2] {
            let u = t1[e1];
            let v = t1[(e1 + 1) % 3];
            let w = t1[(e1 + 2) % 3];

            let Some((t2_idx, e2)) = tris.iter().enumerate().find_map(|(idx, (t, _, _))| {
                if idx == seed_idx {
                    return None;
                }
                for e in 0..3 {
                    if t[e] == v && t[(e + 1) % 3] == u {
                        return Some((idx, e));
                    }
                }
                None
            }) else {
                continue;
            };

            let (t2, n2, fid2) = tris[t2_idx];
            let x = t2[(e2 + 2) % 3];
            let pu = canon_pos[u as usize];
            let pv = canon_pos[v as usize];
            let pw = canon_pos[w as usize];
            let px = canon_pos[x as usize];
            let uv = pv - pu;
            let uv_len_sq = uv.norm_squared();
            if uv_len_sq <= 1e-20 {
                continue;
            }
            let old_pair_min =
                min_triangle_angle_rad(pu, pv, pw).min(min_triangle_angle_rad(pv, pu, px));
            let t_proj = ((pw - pu).dot(&uv) / uv_len_sq).clamp(0.15, 0.85);
            let candidates_s = [t_proj, 0.5, 0.35, 0.65];

            let mut best_s: Option<(Vector3<f64>, f64)> = None;
            for s in candidates_s {
                let pm = pu + uv * s;
                let a1 = min_triangle_angle_rad(pu, pm, pw);
                let a2 = min_triangle_angle_rad(pm, pv, pw);
                let a3 = min_triangle_angle_rad(pv, pm, px);
                let a4 = min_triangle_angle_rad(pm, pu, px);
                let new_min = a1.min(a2).min(a3).min(a4);
                if new_min > old_pair_min + 2e-3
                    && !has_t_junction_on_segment(
                        pm,
                        pw,
                        &[u, v, w, x],
                        &active_verts,
                        canon_pos,
                        t_eps,
                        t_eps_sq,
                    )
                    && !has_t_junction_on_segment(
                        pm,
                        px,
                        &[u, v, w, x],
                        &active_verts,
                        canon_pos,
                        t_eps,
                        t_eps_sq,
                    )
                    && best_s.is_none_or(|(_, b_min)| new_min > b_min)
                {
                    best_s = Some((pm, new_min));
                }
            }

            if let Some((pm, _)) = best_s {
                let m_idx = canon_pos.len() as u32;
                canon_pos.push(pm);
                tris[seed_idx] = ([u, m_idx, w], n1, fid1);
                tris[t2_idx] = ([v, m_idx, x], n2, fid2);
                tris.push(([m_idx, v, w], n1, fid1));
                tris.push(([m_idx, u, x], n2, fid2));
                optimize_coplanar_delaunay_edges(tris, canon_pos, diag);
                modified = true;
                break;
            }
        }

        if !modified {
            let mut key = seed_tri;
            key.sort_unstable();
            unrefinable_tris.insert(key);
        }
    }
}


fn clean_index_loop(loop_ids: &mut Vec<u32>) {
    loop {
        let mut changed = false;
        if loop_ids.len() < 3 {
            break;
        }
        let mut i = 0;
        while i < loop_ids.len() && loop_ids.len() >= 3 {
            let n = loop_ids.len();
            let prev = loop_ids[(i + n - 1) % n];
            let curr = loop_ids[i];
            let next = loop_ids[(i + 1) % n];
            if curr == next || prev == next {
                loop_ids.remove(i);
                changed = true;
            } else {
                i += 1;
            }
        }
        if !changed {
            break;
        }
    }
}

/// Triangulates a convex polygon whose edges may contain 180-degree collinear vertices.
/// Rejects any candidate ear `(prev, curr, next)` if its signed area is near zero OR if another
/// vertex in the loop lies on the open chord `(prev, next)`, guaranteeing that no collinear
/// boundary vertex is ever bypassed.
fn triangulate_subdivided_convex_loop(
    initial_loop: &[u32],
    n_poly: Vector3<f64>,
    face_id: u32,
    canon_pos: &[Vector3<f64>],
    col_eps_sq: f64,
    out_tris: &mut Vec<([u32; 3], Vector3<f64>, u32)>,
) {
    let mut ring: Vec<u32> = initial_loop.to_vec();

    while ring.len() > 3 {
        let k = ring.len();
        let mut best_idx: Option<usize> = None;
        let mut best_quality = -1.0_f64;

        for i in 0..k {
            let prev = ring[(i + k - 1) % k];
            let curr = ring[i];
            let next = ring[(i + 1) % k];
            if prev == curr || curr == next || prev == next {
                continue;
            }

            let p_prev = canon_pos[prev as usize];
            let p_curr = canon_pos[curr as usize];
            let p_next = canon_pos[next as usize];

            let e1 = p_curr - p_prev;
            let e2 = p_next - p_curr;
            let chord = p_next - p_prev;
            let chord_len_sq = chord.norm_squared();
            let perim_sq = e1.norm_squared() + e2.norm_squared() + chord_len_sq;
            if perim_sq <= 1e-20 || chord_len_sq <= 1e-20 {
                continue;
            }

            let cross = e1.cross(&(p_next - p_prev));
            let signed_area2 = cross.dot(&n_poly);
            // Must be strictly convex (non-collinear)
            if signed_area2 <= 1e-9 * perim_sq {
                continue;
            }

            // Ensure no other vertex in `ring` lies on the open chord (p_prev, p_next)
            let mut chord_blocked = false;
            for &other in &ring {
                if other == prev || other == curr || other == next {
                    continue;
                }
                let po = canon_pos[other as usize];
                let ao = po - p_prev;
                let t = ao.dot(&chord) / chord_len_sq;
                if t > -1e-4 && t < 1.0 + 1e-4 {
                    let perp_sq = (ao - chord * t.clamp(0.0, 1.0)).norm_squared();
                    if perp_sq <= col_eps_sq * 4.0 {
                        chord_blocked = true;
                        break;
                    }
                }
            }
            if chord_blocked {
                continue;
            }

            let min_ang = min_triangle_angle_rad(p_prev, p_curr, p_next);
            let quality = min_ang + 0.15 * (signed_area2 / perim_sq);
            if quality > best_quality {
                best_quality = quality;
                best_idx = Some(i);
            }
        }

        let clip_i = if let Some(idx) = best_idx {
            idx
        } else {
            // Fallback: pick the ear with the largest positive cross-product magnitude
            let mut fb_idx = 0;
            let mut fb_area = -1.0_f64;
            for i in 0..k {
                let prev = ring[(i + k - 1) % k];
                let curr = ring[i];
                let next = ring[(i + 1) % k];
                let p_prev = canon_pos[prev as usize];
                let p_curr = canon_pos[curr as usize];
                let p_next = canon_pos[next as usize];
                let a = (p_curr - p_prev).cross(&(p_next - p_prev)).dot(&n_poly);
                if a > fb_area {
                    fb_area = a;
                    fb_idx = i;
                }
            }
            fb_idx
        };

        let prev = ring[(clip_i + k - 1) % k];
        let curr = ring[clip_i];
        let next = ring[(clip_i + 1) % k];
        let p_prev = canon_pos[prev as usize];
        let p_curr = canon_pos[curr as usize];
        let p_next = canon_pos[next as usize];
        if (p_curr - p_prev).cross(&(p_next - p_prev)).norm() > 1e-13 {
            out_tris.push(([prev, curr, next], n_poly, face_id));
        }
        ring.remove(clip_i);
        clean_index_loop(&mut ring);
    }

    if ring.len() == 3 {
        let p0 = canon_pos[ring[0] as usize];
        let p1 = canon_pos[ring[1] as usize];
        let p2 = canon_pos[ring[2] as usize];
        if (p1 - p0).cross(&(p2 - p0)).norm() > 1e-13 {
            out_tris.push(([ring[0], ring[1], ring[2]], n_poly, face_id));
        }
    }
}

/// Resolves any residual T-junctions or 3-edge micro-slits on unmatched boundary edges
/// without ever introducing non-manifold edges or out-of-plane folds.
fn repair_manifold_triangles(
    tris: &mut Vec<([u32; 3], Vector3<f64>, u32)>,
    canon_pos: &[Vector3<f64>],
    diag: f64,
) {
    let split_tol_sq = (diag * 2e-4).max(1e-7).powi(2);
    let cycle_tol_sq = (diag * 2e-3).max(1e-6).powi(2);
    let max_passes = 8;

    for _pass in 0..max_passes {
        // 1. Remove degenerate or exact duplicate triangles
        let mut seen_tris: HashSet<[u32; 3]> = HashSet::with_capacity(tris.len());
        tris.retain(|(t, _, _)| {
            if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] {
                return false;
            }
            let p0 = canon_pos[t[0] as usize];
            let p1 = canon_pos[t[1] as usize];
            let p2 = canon_pos[t[2] as usize];
            if (p1 - p0).cross(&(p2 - p0)).norm() <= 1e-13 {
                return false;
            }
            let key = if t[0] < t[1] && t[0] < t[2] {
                [t[0], t[1], t[2]]
            } else if t[1] < t[0] && t[1] < t[2] {
                [t[1], t[2], t[0]]
            } else {
                [t[2], t[0], t[1]]
            };
            seen_tris.insert(key)
        });

        // 2. Count directed half-edges
        let mut edge_counts: HashMap<(u32, u32), i32> = HashMap::with_capacity(tris.len() * 3);
        let mut edge_fid: HashMap<(u32, u32), u32> = HashMap::with_capacity(tris.len() * 3);
        for &(t, _, fid) in tris.iter() {
            for e in 0..3 {
                let u = t[e];
                let v = t[(e + 1) % 3];
                *edge_counts.entry((u, v)).or_insert(0) += 1;
                edge_fid.entry((u, v)).or_insert(fid);
            }
        }

        // 3. Cancel opposite-winding duplicate triangle pairs ([a, b, c] and [a, c, b] on same 3 vertices)
        // if any of their edges is non-manifold (undirected count >= 3), and prune spurious fin triangles
        // where no edge has undirected count == 2 (i.e. all edges have count 1 or >= 3, with at least one >= 3).
        let mut pruned_any = false;
        let mut unordered_map: HashMap<[u32; 3], Vec<usize>> = HashMap::new();
        for (idx, (t, _, _)) in tris.iter().enumerate() {
            let mut key = *t;
            key.sort_unstable();
            unordered_map.entry(key).or_default().push(idx);
        }
        let mut to_remove: HashSet<usize> = HashSet::new();
        for indices_list in unordered_map.values() {
            if indices_list.len() >= 2 {
                let i0 = indices_list[0];
                let i1 = indices_list[1];
                let t = tris[i0].0;
                let e0 = (t[0], t[1]);
                let e1 = (t[1], t[2]);
                let e2 = (t[2], t[0]);
                let u0 = edge_counts.get(&e0).copied().unwrap_or(0)
                    + edge_counts.get(&(e0.1, e0.0)).copied().unwrap_or(0);
                let u1 = edge_counts.get(&e1).copied().unwrap_or(0)
                    + edge_counts.get(&(e1.1, e1.0)).copied().unwrap_or(0);
                let u2 = edge_counts.get(&e2).copied().unwrap_or(0)
                    + edge_counts.get(&(e2.1, e2.0)).copied().unwrap_or(0);
                if u0 >= 3 || u1 >= 3 || u2 >= 3 {
                    if u0 >= 4 && u1 >= 4 && u2 >= 4 {
                        to_remove.insert(i0);
                        to_remove.insert(i1);
                    } else {
                        to_remove.insert(i1);
                    }
                }
            }
        }
        if !to_remove.is_empty() {
            let mut idx = 0;
            tris.retain(|_| {
                let keep = !to_remove.contains(&idx);
                idx += 1;
                keep
            });
            continue;
        }

        let mut i = 0;
        while i < tris.len() {
            let t = tris[i].0;
            let e0 = (t[0], t[1]);
            let e1 = (t[1], t[2]);
            let e2 = (t[2], t[0]);

            let c0 = edge_counts.get(&e0).copied().unwrap_or(0);
            let c1 = edge_counts.get(&e1).copied().unwrap_or(0);
            let c2 = edge_counts.get(&e2).copied().unwrap_or(0);
            let u0 = c0 + edge_counts.get(&(e0.1, e0.0)).copied().unwrap_or(0);
            let u1 = c1 + edge_counts.get(&(e1.1, e1.0)).copied().unwrap_or(0);
            let u2 = c2 + edge_counts.get(&(e2.1, e2.0)).copied().unwrap_or(0);

            let is_isolated_fin = u0 != 2 && u1 != 2 && u2 != 2 && (u0 >= 3 || u1 >= 3 || u2 >= 3);
            let is_dup_flap = ((c0 >= 2 && u0 >= 3) || (c1 >= 2 && u1 >= 3) || (c2 >= 2 && u2 >= 3))
                && (u0 == 1 || u1 == 1 || u2 == 1);

            if is_isolated_fin || is_dup_flap {
                *edge_counts.entry(e0).or_insert(1) -= 1;
                *edge_counts.entry(e1).or_insert(1) -= 1;
                *edge_counts.entry(e2).or_insert(1) -= 1;
                tris.swap_remove(i);
                pruned_any = true;
            } else {
                i += 1;
            }
        }
        if pruned_any {
            continue;
        }

        // 4. Collect unmatched directed edges: count(u, v) > count(v, u)
        let mut unmatched: HashSet<(u32, u32)> = HashSet::new();
        let mut boundary_verts: HashSet<u32> = HashSet::new();
        for (&(u, v), &c_uv) in &edge_counts {
            let c_vu = edge_counts.get(&(v, u)).copied().unwrap_or(0);
            if c_uv > c_vu {
                unmatched.insert((u, v));
                boundary_verts.insert(u);
                boundary_verts.insert(v);
            }
        }

        if unmatched.is_empty() {
            break;
        }

        let mut b_verts_vec: Vec<u32> = boundary_verts.iter().copied().collect();
        b_verts_vec.sort_unstable();
        let mut split_any = false;
        let mut next_tris: Vec<([u32; 3], Vector3<f64>, u32)> = Vec::with_capacity(tris.len() + 16);

        for &(t, n_hint, fid) in tris.iter() {
            let mut split_done = false;
            let p0 = canon_pos[t[0] as usize];
            let p1 = canon_pos[t[1] as usize];
            let p2 = canon_pos[t[2] as usize];
            let orig_cross = (p1 - p0).cross(&(p2 - p0));
            let orig_len = orig_cross.norm();
            if orig_len <= 1e-13 {
                continue;
            }
            let n_orig = orig_cross / orig_len;

            for e in 0..3 {
                let u = t[e];
                let v = t[(e + 1) % 3];
                let w = t[(e + 2) % 3];
                if !unmatched.contains(&(u, v)) {
                    continue;
                }

                let pa = canon_pos[u as usize];
                let pb = canon_pos[v as usize];
                let pw = canon_pos[w as usize];
                let ab = pb - pa;
                let ab_len_sq = ab.norm_squared();
                if ab_len_sq <= 1e-20 {
                    continue;
                }

                let max_tol = cycle_tol_sq.sqrt();
                let seg_min_x = pa.x.min(pb.x) - max_tol;
                let seg_max_x = pa.x.max(pb.x) + max_tol;
                let seg_min_y = pa.y.min(pb.y) - max_tol;
                let seg_max_y = pa.y.max(pb.y) + max_tol;
                let seg_min_z = pa.z.min(pb.z) - max_tol;
                let seg_max_z = pa.z.max(pb.z) + max_tol;

                let mut best_m: Option<(f64, u32)> = None;
                for &m in &b_verts_vec {
                    if m == u || m == v || m == w {
                        continue;
                    }
                    let pm = canon_pos[m as usize];
                    if pm.x < seg_min_x
                        || pm.x > seg_max_x
                        || pm.y < seg_min_y
                        || pm.y > seg_max_y
                        || pm.z < seg_min_z
                        || pm.z > seg_max_z
                    {
                        continue;
                    }

                    let ap = pm - pa;
                    let proj = ap.dot(&ab) / ab_len_sq;
                    if proj <= 1e-4 || proj >= 1.0 - 1e-4 {
                        continue;
                    }
                    let perp_sq = (ap - ab * proj).norm_squared();
                    if perp_sq > cycle_tol_sq {
                        continue;
                    }

                    let is_3cycle = unmatched.contains(&(v, m)) && unmatched.contains(&(m, u));
                    let tol_sq = if is_3cycle { cycle_tol_sq } else { split_tol_sq };
                    if perp_sq > tol_sq {
                        continue;
                    }

                    // Never split if any of the replacement edges already exists in the same direction
                    if edge_counts.get(&(u, m)).copied().unwrap_or(0) > 0
                        || edge_counts.get(&(m, v)).copied().unwrap_or(0) > 0
                        || edge_counts.get(&(m, w)).copied().unwrap_or(0) > 0
                        || edge_counts.get(&(w, m)).copied().unwrap_or(0) > 0
                    {
                        continue;
                    }

                    let min_dot = if is_3cycle { 0.85 } else { 0.99 };
                    let c1 = (pm - pa).cross(&(pw - pa));
                    let c2 = (pb - pm).cross(&(pw - pm));
                    let l1 = c1.norm();
                    let l2 = c2.norm();
                    if l1 > 1e-12
                        && l2 > 1e-12
                        && (c1 / l1).dot(&n_orig) > min_dot
                        && (c2 / l2).dot(&n_orig) > min_dot
                        && (best_m.is_none()
                            || (proj - 0.5).abs() < (best_m.unwrap().0 - 0.5).abs())
                    {
                        best_m = Some((proj, m));
                    }
                }

                if let Some((_, m)) = best_m {
                    next_tris.push(([u, m, w], n_hint, fid));
                    next_tris.push(([m, v, w], n_hint, fid));
                    *edge_counts.entry((u, v)).or_insert(1) -= 1;
                    *edge_counts.entry((u, m)).or_insert(0) += 1;
                    *edge_counts.entry((m, v)).or_insert(0) += 1;
                    *edge_counts.entry((m, w)).or_insert(0) += 1;
                    *edge_counts.entry((w, m)).or_insert(0) += 1;
                    if edge_counts.get(&(u, v)).copied().unwrap_or(0)
                        <= edge_counts.get(&(v, u)).copied().unwrap_or(0)
                    {
                        unmatched.remove(&(u, v));
                    }
                    if edge_counts.get(&(m, u)).copied().unwrap_or(0)
                        <= edge_counts.get(&(u, m)).copied().unwrap_or(0)
                    {
                        unmatched.remove(&(m, u));
                    }
                    if edge_counts.get(&(v, m)).copied().unwrap_or(0)
                        <= edge_counts.get(&(m, v)).copied().unwrap_or(0)
                    {
                        unmatched.remove(&(v, m));
                    }
                    split_done = true;
                    split_any = true;
                    break;
                }
            }
            if !split_done {
                next_tris.push((t, n_hint, fid));
            }
        }

        *tris = next_tris;
        if split_any {
            continue;
        }

        // 5. Seal closed 3-edge micro-hole loops u -> v -> m -> u ONLY if each edge is a true boundary edge
        let mut unmatched_vec: Vec<(u32, u32)> = unmatched.iter().copied().collect();
        unmatched_vec.sort_unstable();

        let mut next_hop: HashMap<u32, Vec<u32>> = HashMap::new();
        for &(u, v) in &unmatched_vec {
            if edge_counts.get(&(u, v)).copied().unwrap_or(0) == 1
                && edge_counts.get(&(v, u)).copied().unwrap_or(0) == 0
            {
                next_hop.entry(u).or_default().push(v);
            }
        }

        let mut sealed_any = false;
        let mut used_edges: HashSet<(u32, u32)> = HashSet::new();

        for &(u, v) in &unmatched_vec {
            if used_edges.contains(&(u, v))
                || edge_counts.get(&(u, v)).copied().unwrap_or(0) != 1
                || edge_counts.get(&(v, u)).copied().unwrap_or(0) != 0
            {
                continue;
            }
            if let Some(v_nexts) = next_hop.get(&v) {
                for &m in v_nexts {
                    if m == u || m == v || used_edges.contains(&(v, m)) {
                        continue;
                    }
                    if !used_edges.contains(&(m, u))
                        && edge_counts.get(&(m, u)).copied().unwrap_or(0) == 1
                        && edge_counts.get(&(u, m)).copied().unwrap_or(0) == 0
                        && edge_counts.get(&(m, v)).copied().unwrap_or(0) == 0
                        && edge_counts.get(&(v, u)).copied().unwrap_or(0) == 0
                    {
                        let pu = canon_pos[u as usize];
                        let pv = canon_pos[v as usize];
                        let pm = canon_pos[m as usize];
                        let max_e = (pv - pu)
                            .norm()
                            .max((pm - pv).norm())
                            .max((pu - pm).norm());
                        let cross = (pm - pu).cross(&(pv - pu));
                        let len = cross.norm();
                        if max_e < diag * 0.15 && len > 1e-13 {
                            let seal_fid = edge_fid
                                .get(&(u, v))
                                .or_else(|| edge_fid.get(&(v, m)))
                                .or_else(|| edge_fid.get(&(m, u)))
                                .copied()
                                .unwrap_or(0);
                            tris.push(([u, m, v], cross / len, seal_fid));
                            *edge_counts.entry((u, m)).or_insert(0) += 1;
                            *edge_counts.entry((m, v)).or_insert(0) += 1;
                            *edge_counts.entry((v, u)).or_insert(0) += 1;
                            used_edges.insert((u, v));
                            used_edges.insert((v, m));
                            used_edges.insert((m, u));
                            sealed_any = true;
                            break;
                        }
                    }
                }
            }
        }

        if !sealed_any {
            break;
        }
    }

    // Final pass: flip any isolated locally-inverted triangle whose 3 undirected edges are
    // manifold (count == 2) but whose directed edges conflict with its neighbors (count == 2, rev == 0).
    for _flip_pass in 0..4 {
        let mut edge_counts: HashMap<(u32, u32), i32> = HashMap::with_capacity(tris.len() * 3);
        for (t, _, _) in tris.iter() {
            for e in 0..3 {
                *edge_counts.entry((t[e], t[(e + 1) % 3])).or_insert(0) += 1;
            }
        }
        let mut flipped_any = false;
        for (t, n_hint, _) in tris.iter_mut() {
            let e0 = (t[0], t[1]);
            let e1 = (t[1], t[2]);
            let e2 = (t[2], t[0]);
            let c0 = edge_counts.get(&e0).copied().unwrap_or(0);
            let r0 = edge_counts.get(&(e0.1, e0.0)).copied().unwrap_or(0);
            let c1 = edge_counts.get(&e1).copied().unwrap_or(0);
            let r1 = edge_counts.get(&(e1.1, e1.0)).copied().unwrap_or(0);
            let c2 = edge_counts.get(&e2).copied().unwrap_or(0);
            let r2 = edge_counts.get(&(e2.1, e2.0)).copied().unwrap_or(0);

            if c0 + r0 == 2 && c1 + r1 == 2 && c2 + r2 == 2 {
                let bad_before = (c0 > 1) as i32 + (c1 > 1) as i32 + (c2 > 1) as i32;
                let bad_after = (r0 >= 1) as i32 + (r1 >= 1) as i32 + (r2 >= 1) as i32;
                if bad_before >= 2 && bad_after < bad_before {
                    *edge_counts.entry(e0).or_insert(1) -= 1;
                    *edge_counts.entry(e1).or_insert(1) -= 1;
                    *edge_counts.entry(e2).or_insert(1) -= 1;
                    *edge_counts.entry((e0.1, e0.0)).or_insert(0) += 1;
                    *edge_counts.entry((e1.1, e1.0)).or_insert(0) += 1;
                    *edge_counts.entry((e2.1, e2.0)).or_insert(0) += 1;
                    t.swap(1, 2);
                    *n_hint = -*n_hint;
                    flipped_any = true;
                }
            }
        }
        if !flipped_any {
            break;
        }
    }
}

fn flip_polygons(polys: &mut [CsgPolygon]) {
    for p in polys {
        p.flip();
    }
}

fn push_unique_alias(aliases: &mut Vec<String>, alias: String) {
    if !alias.is_empty() && !aliases.contains(&alias) {
        aliases.push(alias);
    }
}

/// Computes `id_offset` and merges the `faces` registries of operands `a` and `b` for a CSG operation.
///
/// For `is_difference == true` (`csg_difference(a, b)`):
/// - Surviving faces of `b` have their planar normals inverted (`-normal_or_axis`).
/// - Dedicated cutter faces (`hole_wall`, `hole_bottom`, `keyway_wall`, `helical_groove`) keep their
///   role and also receive `cut:<role>` aliases.
/// - General tool faces receive `cut:<origin_feature>:<original_name>` (or `cut:<original_name>`)
///   as their canonical name, and if a cylindrical side wall is subtracted, `"hole_wall"` is added
///   as an alias.
fn merge_csg_face_registries(
    a: &TriangleMesh,
    b: &TriangleMesh,
    is_difference: bool,
) -> (u32, Vec<FaceInfo>) {
    let max_a_face = a.faces.iter().map(|f| f.id).max().unwrap_or(0);
    let max_a_tid = a.face_ids.iter().copied().max().unwrap_or(0);
    let id_offset = max_a_face.max(max_a_tid);

    let mut combined = a.faces.clone();
    for fb in &b.faces {
        let mut f = fb.clone();
        f.id = fb.id + id_offset;
        f.aliases.retain(|al| !al.starts_with("helix_meta:"));

        if is_difference {
            if f.kind == FaceKind::Plane {
                f.normal_or_axis = [
                    -f.normal_or_axis[0],
                    -f.normal_or_axis[1],
                    -f.normal_or_axis[2],
                ];
            }

            let is_dedicated_cutter = f.role.starts_with("hole_")
                || f.role.starts_with("keyway_")
                || f.role.starts_with("helical_")
                || f.name.starts_with("hole_")
                || f.name.starts_with("keyway_")
                || f.name.starts_with("cut:");

            if is_dedicated_cutter {
                push_unique_alias(&mut f.aliases, f.role.clone());
                push_unique_alias(&mut f.aliases, format!("cut:{}", f.role));
                if combined.iter().any(|existing| existing.name == f.name) {
                    let mut suffix = 1u32;
                    loop {
                        let candidate = format!("{}:{suffix}", f.role);
                        if !combined.iter().any(|existing| existing.name == candidate) {
                            f.name = candidate;
                            break;
                        }
                        suffix += 1;
                    }
                }
            } else {
                let orig_name = fb.name.clone();
                let orig_role = fb.role.clone();
                let cut_name = if let Some(feat_alias) =
                    fb.aliases.iter().find(|al| al.starts_with("feat_"))
                {
                    format!("cut:{feat_alias}")
                } else if let Some(ref orig_feat) = fb.origin_feature {
                    format!("cut:{orig_feat}:{orig_name}")
                } else {
                    format!("cut:{orig_name}")
                };

                f.name = if combined.iter().any(|existing| existing.name == cut_name) {
                    let mut suffix = 1u32;
                    loop {
                        let candidate = format!("{cut_name}:{suffix}");
                        if !combined.iter().any(|existing| existing.name == candidate) {
                            break candidate;
                        }
                        suffix += 1;
                    }
                } else {
                    cut_name
                };
                f.role = format!("cut:{orig_role}");
                push_unique_alias(&mut f.aliases, format!("cut:{orig_name}"));
                push_unique_alias(&mut f.aliases, format!("cut:{orig_role}"));
                for al in &fb.aliases {
                    if !al.starts_with("helix_meta:") {
                        push_unique_alias(&mut f.aliases, format!("cut:{al}"));
                    }
                }
                if fb.kind == FaceKind::Cylinder && orig_role == "side" {
                    push_unique_alias(&mut f.aliases, "hole_wall".to_string());
                    push_unique_alias(&mut f.aliases, "bore".to_string());
                }
            }
        } else if combined.iter().any(|existing| existing.name == f.name) {
            let base = if let Some(feat_alias) =
                fb.aliases.iter().find(|al| al.starts_with("feat_"))
            {
                feat_alias.clone()
            } else if let Some(ref orig_feat) = fb.origin_feature {
                format!("{orig_feat}:{}", fb.name)
            } else {
                format!("tool:{}", fb.name)
            };
            f.name = if combined.iter().any(|existing| existing.name == base) {
                let mut suffix = 1u32;
                loop {
                    let candidate = format!("{base}:{suffix}");
                    if !combined.iter().any(|existing| existing.name == candidate) {
                        break candidate;
                    }
                    suffix += 1;
                }
            } else {
                base
            };
        }

        combined.push(f);
    }

    (id_offset, combined)
}

#[derive(Debug, Clone)]
struct HelicalGrooveParams {
    helix_radius: f64,
    groove_radius: f64,
    pitch: f64,
    height: f64,
    start_z: f64,
    starts: usize,
    is_right_hand: bool,
    turns: f64,
    is_metric_60: bool,
}

impl HelicalGrooveParams {
    fn from_cutter_mesh(b: &TriangleMesh) -> Option<Self> {
        let f0 = b.faces.first()?;
        let meta = f0.aliases.iter().find(|al| al.starts_with("helix_meta:"))?;
        let parts: Vec<&str> = meta.split(':').collect();
        if parts.len() < 10 {
            return None;
        }
        let helix_radius = parts[1].parse::<f64>().ok()?;
        let groove_radius = parts[2].parse::<f64>().ok()?;
        let pitch = parts[3].parse::<f64>().ok()?;
        let height = parts[4].parse::<f64>().ok()?;
        let start_z = parts[5].parse::<f64>().ok()?;
        let starts = parts[6].parse::<usize>().ok()?.max(1);
        let is_right_hand = parts[7] == "1";
        let turns = parts[8].parse::<f64>().ok()?;
        let is_metric_60 = parts[9].eq_ignore_ascii_case("metric_60");
        if pitch <= 1e-9 || height <= 1e-9 || groove_radius <= 1e-9 || turns <= 0.0 {
            return None;
        }
        Some(Self {
            helix_radius,
            groove_radius,
            pitch,
            height,
            start_z,
            starts,
            is_right_hand,
            turns,
            is_metric_60,
        })
    }

    /// Computes the perpendicular distance `d_perp` in the normal plane `(e_r, e_b)` of the helix
    /// from cylindrical point `(r_ref, theta, z)` to the nearest active turn among all starts.
    fn min_perp_distance(&self, r_ref: f64, theta: f64, z: f64) -> f64 {
        use std::f64::consts::PI;
        let two_pi = 2.0 * PI;
        let s = if self.is_right_hand { 1.0 } else { -1.0 };
        let c = self.pitch / two_pi;
        let h_len = (r_ref * r_ref + c * c).sqrt().max(1e-12);
        let cos_psi = (r_ref / h_len).clamp(0.1, 1.0);
        let t_max = two_pi * self.turns;

        let m_center = ((z - self.start_z) / self.pitch).round() as i64;
        let mut best_d = f64::INFINITY;

        for start_idx in 0..self.starts {
            let phi_0 = (start_idx as f64) * two_pi / (self.starts as f64);
            let mut base_t = (theta - phi_0) * s % two_pi;
            if base_t < 0.0 {
                base_t += two_pi;
            }
            for dm in -2..=2 {
                let m = m_center + dm;
                let t_m = base_t + (m as f64) * two_pi;
                if t_m >= 0.0 && t_m <= t_max {
                    let z_m = self.start_z + (t_m / two_pi) * self.pitch;
                    let d = (z - z_m).abs() * cos_psi;
                    if d < best_d {
                        best_d = d;
                    }
                } else {
                    let t_c = t_m.clamp(0.0, t_max);
                    let theta_c = phi_0 + s * t_c;
                    let z_c = self.start_z + (t_c / two_pi) * self.pitch;
                    let d_ang = 2.0 * r_ref * (0.5 * (theta - theta_c)).sin().abs();
                    let dz = z - z_c;
                    let d = (d_ang * d_ang + dz * dz).sqrt();
                    if d < best_d {
                        best_d = d;
                    }
                }
            }
        }
        best_d
    }

    /// Evaluates positive radial indentation depth at `(r_ref, theta, z)`.
    fn radial_depth(&self, r_ref: f64, theta: f64, z: f64, max_depth_cap: f64) -> f64 {
        use std::f64::consts::PI;
        let d_perp = self.min_perp_distance(r_ref, theta, z);
        if self.is_metric_60 {
            let tan30 = (PI / 6.0).tan();
            let w = (self.groove_radius * tan30).min(0.45 * self.pitch);
            if d_perp >= w || w <= 1e-12 {
                return 0.0;
            }
            let h_eff = (w / tan30).min(max_depth_cap);
            let u = 1.0 - (d_perp / w);
            // Smooth ISO 68-1 root truncation (avoids sharp 0-width root crease)
            let trunc = 0.85 * h_eff;
            (h_eff * u).min(trunc)
        } else {
            let w = self.groove_radius.min(0.45 * self.pitch * (self.starts as f64));
            if d_perp >= w || w <= 1e-12 {
                return 0.0;
            }
            let depth_max = self.groove_radius.min(max_depth_cap);
            let q = d_perp / w;
            depth_max * (1.0 - q * q)
        }
    }
}

/// Emits a quad `(p0, p1, p2, p3)` split along its shorter 3D diagonal, assigning each emitted
/// triangle's exact unit geometric normal `(b - a) x (c - a)` so `normals_vs_winding_mismatches == 0`.
#[inline]
fn push_quad_exact_normals(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<[u32; 3]>,
    p0: [f64; 3],
    p1: [f64; 3],
    p2: [f64; 3],
    p3: [f64; 3],
) {
    let mut emit_tri = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| {
        let va = Vector3::new(a[0], a[1], a[2]);
        let vb = Vector3::new(b[0], b[1], b[2]);
        let vc = Vector3::new(c[0], c[1], c[2]);
        let n_vec = (vb - va).cross(&(vc - va));
        let n_len = n_vec.norm();
        let n = if n_len > 1e-15 {
            [n_vec.x / n_len, n_vec.y / n_len, n_vec.z / n_len]
        } else {
            [0.0, 0.0, 1.0]
        };
        let base = vertices.len() as u32;
        vertices.push(Vertex::new(a, n));
        vertices.push(Vertex::new(b, n));
        vertices.push(Vertex::new(c, n));
        indices.push([base, base + 1, base + 2]);
    };

    let d02_sq = (p0[0] - p2[0]).powi(2) + (p0[1] - p2[1]).powi(2) + (p0[2] - p2[2]).powi(2);
    let d13_sq = (p1[0] - p3[0]).powi(2) + (p1[1] - p3[1]).powi(2) + (p1[2] - p3[2]).powi(2);
    if d02_sq <= d13_sq * 1.000001 {
        emit_tri(p0, p1, p2);
        emit_tri(p0, p2, p3);
    } else {
        emit_tri(p0, p1, p3);
        emit_tri(p1, p2, p3);
    }
}

/// Extracts a single simple closed 2D loop `[x, y]` from open boundary half-edges at `z ≈ z_target`,
/// ordered CCW in the XY plane and rotated so vertex `0` has the smallest angle in `[0, 2*PI)`.
fn trace_ccw_rim_loop_2d(
    open_edges: &[(u32, u32)],
    canon_pos: &[[f64; 3]],
) -> Option<Vec<[f64; 2]>> {
    use std::f64::consts::PI;
    if open_edges.len() < 3 {
        return None;
    }
    let mut next_map: HashMap<u32, u32> = HashMap::with_capacity(open_edges.len());
    let mut in_deg: HashMap<u32, usize> = HashMap::with_capacity(open_edges.len());
    for &(u, v) in open_edges {
        if next_map.insert(u, v).is_some() {
            return None;
        }
        *in_deg.entry(v).or_insert(0) += 1;
    }
    if next_map.len() != open_edges.len() || in_deg.values().any(|&c| c != 1) {
        return None;
    }

    let start_u = open_edges[0].0;
    let mut cycle: Vec<[f64; 2]> = Vec::with_capacity(open_edges.len());
    let mut curr = start_u;
    for _ in 0..open_edges.len() {
        let p = canon_pos[curr as usize];
        cycle.push([p[0], p[1]]);
        curr = *next_map.get(&curr)?;
        if curr == start_u {
            break;
        }
    }
    if curr != start_u || cycle.len() != open_edges.len() {
        return None;
    }

    let mut area2 = 0.0;
    for i in 0..cycle.len() {
        let a = cycle[i];
        let b = cycle[(i + 1) % cycle.len()];
        area2 += a[0] * b[1] - b[0] * a[1];
    }
    if area2.abs() <= 1e-18 {
        return None;
    }
    if area2 < 0.0 {
        cycle.reverse();
    }

    let two_pi = 2.0 * PI;
    let pos_angle = |p: [f64; 2]| -> f64 {
        let mut a = p[1].atan2(p[0]);
        if a < -1e-9 {
            a += two_pi;
        } else if a < 0.0 {
            a = 0.0;
        }
        a
    };

    let mut best_idx = 0usize;
    let mut best_ang = f64::INFINITY;
    for (idx, &p) in cycle.iter().enumerate() {
        let ang = pos_angle(p);
        if ang < best_ang {
            best_ang = ang;
            best_idx = idx;
        }
    }
    cycle.rotate_left(best_idx);

    // Verify star-shaped monotone CCW angles around origin
    for i in 0..cycle.len() {
        let a = cycle[i];
        let b = cycle[(i + 1) % cycle.len()];
        let cross = a[0] * b[1] - a[1] * b[0];
        if cross <= 1e-16 * (a[0] * a[0] + a[1] * a[1]).max(1e-12) {
            return None;
        }
    }
    Some(cycle)
}

type CoaxialShellTri = ([f64; 3], [f64; 3], [f64; 3], [f64; 3], u32);
type CoaxialOuterShell = (
    Vec<CoaxialShellTri>,
    Vec<[f64; 2]>,
    Vec<[f64; 2]>,
    f64,
    f64,
);

/// Extracts the outer shell triangles and top/bottom CCW rim loops of a coaxial solid `a`
/// after excluding `excluded_fids` (such as `cap_top`, `cap_bottom`, `hole_wall`, `helical_groove`).
fn extract_coaxial_outer_shell(
    a: &TriangleMesh,
    excluded_fids: &[u32],
    min_allowed_shell_radius: f64,
) -> Option<CoaxialOuterShell> {
    if a.vertices.is_empty() || a.indices.is_empty() || a.face_ids.len() != a.indices.len() {
        return None;
    }
    let mut z_min = f64::INFINITY;
    let mut z_max = f64::NEG_INFINITY;
    for v in &a.vertices {
        z_min = z_min.min(v.position[2]);
        z_max = z_max.max(v.position[2]);
    }
    let z_span = z_max - z_min;
    if z_span <= 1e-9 {
        return None;
    }
    let z_tol = (z_span * 1e-5).max(1e-8);
    let weld_tol = (z_span * 1e-7).max(1e-9);
    let inv_weld = 1.0 / weld_tol;

    let mut shell_tris: Vec<CoaxialShellTri> = Vec::new();
    let mut canon_map: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut canon_pos: Vec<[f64; 3]> = Vec::new();
    let mut canon_tris: Vec<[u32; 3]> = Vec::new();

    let mut get_cid = |p: [f64; 3]| -> u32 {
        let key = (
            (p[0] * inv_weld).round() as i64,
            (p[1] * inv_weld).round() as i64,
            (p[2] * inv_weld).round() as i64,
        );
        if let Some(&cid) = canon_map.get(&key) {
            cid
        } else {
            let cid = canon_pos.len() as u32;
            canon_pos.push(p);
            canon_map.insert(key, cid);
            cid
        }
    };

    for (t_idx, tri) in a.indices.iter().enumerate() {
        let fid = a.face_ids[t_idx];
        if excluded_fids.contains(&fid) {
            continue;
        }
        let v0 = &a.vertices[tri[0] as usize];
        let v1 = &a.vertices[tri[1] as usize];
        let v2 = &a.vertices[tri[2] as usize];
        for p in [v0.position, v1.position, v2.position] {
            let r = p[0].hypot(p[1]);
            if r < min_allowed_shell_radius {
                return None;
            }
        }
        let c0 = get_cid(v0.position);
        let c1 = get_cid(v1.position);
        let c2 = get_cid(v2.position);
        if c0 == c1 || c1 == c2 || c2 == c0 {
            continue;
        }
        shell_tris.push((v0.position, v1.position, v2.position, v0.normal, fid));
        canon_tris.push([c0, c1, c2]);
    }

    if shell_tris.is_empty() {
        return None;
    }

    let mut half_edges: HashMap<(u32, u32), usize> = HashMap::with_capacity(canon_tris.len() * 3);
    for ct in &canon_tris {
        for e in 0..3 {
            let u = ct[e];
            let v = ct[(e + 1) % 3];
            *half_edges.entry((u, v)).or_insert(0) += 1;
        }
    }

    let mut top_open: Vec<(u32, u32)> = Vec::new();
    let mut bot_open: Vec<(u32, u32)> = Vec::new();
    for (&(u, v), &cnt) in &half_edges {
        if cnt != 1 {
            return None;
        }
        let rev_cnt = half_edges.get(&(v, u)).copied().unwrap_or(0);
        if rev_cnt == 0 {
            let pu = canon_pos[u as usize];
            let pv = canon_pos[v as usize];
            if (pu[2] - z_max).abs() <= z_tol && (pv[2] - z_max).abs() <= z_tol {
                top_open.push((v, u));
            } else if (pu[2] - z_min).abs() <= z_tol && (pv[2] - z_min).abs() <= z_tol {
                bot_open.push((u, v));
            } else {
                return None;
            }
        }
    }

    let top_ring = trace_ccw_rim_loop_2d(&top_open, &canon_pos)?;
    let bot_ring = trace_ccw_rim_loop_2d(&bot_open, &canon_pos)?;
    if top_ring.len() != bot_ring.len() || top_ring.len() < 6 {
        return None;
    }

    Some((shell_tris, top_ring, bot_ring, z_min, z_max))
}

/// Emits a planar annulus cap at `z_val` connecting an `M`-vertex `outer_ring` to a `K = M * 2^p` vertex
/// circular inner bore of radius `r_bore` using `p` 1-to-2 inward doubling transitions followed by
/// concentric ring bridging. Returns the `K` inner bore `[x, y]` vertices and their angles `theta_u`.
#[allow(clippy::too_many_arguments)]
fn emit_doubled_and_bridged_annulus_cap(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<[u32; 3]>,
    face_ids: &mut Vec<u32>,
    cap_fid: u32,
    outer_ring: &[[f64; 2]],
    r_bore: f64,
    doublings: usize,
    z_val: f64,
    is_top: bool,
) -> (Vec<[f64; 2]>, Vec<f64>) {
    let start_tris = indices.len();
    let mut curr_ring: Vec<[f64; 2]> = outer_ring.to_vec();

    for _step in 0..doublings {
        let m = curr_ring.len();
        let mut sub_pts: Vec<[f64; 2]> = Vec::with_capacity(2 * m);
        let mut r_sum = 0.0;
        for j in 0..m {
            let p0 = curr_ring[j];
            let p1 = curr_ring[(j + 1) % m];
            let pm = [0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])];
            r_sum += p0[0].hypot(p0[1]) + pm[0].hypot(pm[1]);
            sub_pts.push(p0);
            sub_pts.push(pm);
        }
        let r_avg = r_sum / ((2 * m) as f64);
        let next_ring: Vec<[f64; 2]> = sub_pts
            .iter()
            .map(|p| {
                let r_p = p[0].hypot(p[1]);
                let th = p[1].atan2(p[0]);
                let r_circ = 0.5 * r_p + 0.5 * r_avg;
                let r_next = r_circ - 0.42 * (r_circ - r_bore);
                [r_next * th.cos(), r_next * th.sin()]
            })
            .collect();
        crate::primitives::add_doubling_ring_transition_2d(
            vertices,
            indices,
            &curr_ring,
            &next_ring,
            z_val,
            is_top,
        );
        curr_ring = next_ring;
    }

    let k = curr_ring.len();
    let mut thetas: Vec<f64> = Vec::with_capacity(k);
    let mut inner_ring: Vec<[f64; 2]> = Vec::with_capacity(k);
    for p in &curr_ring {
        let th = p[1].atan2(p[0]);
        thetas.push(th);
        inner_ring.push([r_bore * th.cos(), r_bore * th.sin()]);
    }

    crate::primitives::add_bridged_ring_strip_2d(
        vertices,
        indices,
        &curr_ring,
        &inner_ring,
        z_val,
        is_top,
    );
    face_ids.resize(indices.len(), cap_fid);
    let _ = start_tris;
    (inner_ring, thetas)
}

/// Local axial symmetry frame for a coaxial workpiece (`cap_bottom`, optional `cap_top`,
/// optional cylindrical `side`/`hole_wall`).
struct CoaxialWorkpieceFrame {
    /// Unit symmetry axis pointing from `cap_bottom` toward `cap_top`.
    axis: Vector3<f64>,
    /// Point on the symmetry axis lying in the `cap_bottom` plane.
    origin_bot: Vector3<f64>,
    /// Total extent of the workpiece projected onto `axis`.
    axial_span: f64,
}

/// Builds a right-handed orthonormal basis `(e1, e2)` perpendicular to unit vector `z`.
fn orthonormal_basis_from_z(z: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let ref_vec = if z[0].abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let e1 = ref_vec.cross(z).normalize();
    let e2 = z.cross(&e1).normalize();
    (e1, e2)
}

/// Fits a 2D circle center `[cx, cy]` via scale-normalized Kåsa least squares and verifies
/// that the fitted radius and all sample points match `r_expected` within `5e-3 * r_expected`.
fn fit_circle_center_2d(pts: &[[f64; 2]], r_expected: f64) -> Option<[f64; 2]> {
    if pts.len() < 3 || !r_expected.is_finite() || r_expected <= 1e-9 {
        return None;
    }
    let inv_n = 1.0 / (pts.len() as f64);
    let mx = pts.iter().map(|p| p[0]).sum::<f64>() * inv_n;
    let my = pts.iter().map(|p| p[1]).sum::<f64>() * inv_n;
    let inv_r = 1.0 / r_expected;

    let mut m = nalgebra::Matrix3::<f64>::zeros();
    let mut rhs = nalgebra::Vector3::<f64>::zeros();
    for p in pts {
        let u = (p[0] - mx) * inv_r;
        let v = (p[1] - my) * inv_r;
        let b = u * u + v * v;
        let row = nalgebra::Vector3::new(2.0 * u, 2.0 * v, 1.0);
        m += row * row.transpose();
        rhs += row * b;
    }

    let sol = m.full_piv_lu().solve(&rhs)?;
    if !sol.iter().all(|v| v.is_finite()) {
        return None;
    }
    let uc = sol[0];
    let vc = sol[1];
    let rho_sq = sol[2] + uc * uc + vc * vc;
    if !rho_sq.is_finite() || rho_sq <= 0.0 {
        return None;
    }
    let r_fit = rho_sq.sqrt() * r_expected;
    let rad_tol = 5e-3 * r_expected;
    if (r_fit - r_expected).abs() > rad_tol {
        return None;
    }

    let cx = mx + uc * r_expected;
    let cy = my + vc * r_expected;
    if pts
        .iter()
        .any(|p| ((p[0] - cx).hypot(p[1] - cy) - r_expected).abs() > rad_tol)
    {
        return None;
    }
    Some([cx, cy])
}

/// Normalizes a 3D array into a finite unit `Vector3<f64>`, returning `None` if zero or non-finite.
fn unit_vec3(v: [f64; 3]) -> Option<Vector3<f64>> {
    let vec = Vector3::from(v);
    let len = vec.norm();
    if !len.is_finite() || len <= 1e-12 {
        None
    } else {
        Some(vec / len)
    }
}

/// Detects the axial symmetry frame of `a` from `cap_bottom` (outward normal `-axis`),
/// optional `cap_top` (outward normal `+axis`), and an optional reference cylindrical face
/// (`hole_wall` or `side`).
fn detect_coaxial_workpiece_frame(
    a: &TriangleMesh,
    ref_cyl_face: Option<&FaceInfo>,
) -> Option<CoaxialWorkpieceFrame> {
    if a.vertices.is_empty() || a.face_ids.len() != a.indices.len() {
        return None;
    }
    let cap_bot_f = a
        .faces
        .iter()
        .find(|f| f.role == "cap_bottom" && f.kind == FaceKind::Plane)?;
    let axis = -unit_vec3(cap_bot_f.normal_or_axis)?;

    if let Some(cap_top_f) = a
        .faces
        .iter()
        .find(|f| f.role == "cap_top" && f.kind == FaceKind::Plane)
    {
        if unit_vec3(cap_top_f.normal_or_axis)?.dot(&axis) <= 0.999 {
            return None;
        }
    }

    let mut s_min = f64::INFINITY;
    let mut s_max = f64::NEG_INFINITY;
    for v in &a.vertices {
        let s = Vector3::from(v.position).dot(&axis);
        s_min = s_min.min(s);
        s_max = s_max.max(s);
    }
    let axial_span = s_max - s_min;
    if !axial_span.is_finite() || axial_span <= 1e-9 {
        return None;
    }

    let mut bot_area = 0.0_f64;
    let mut bot_ctr = Vector3::zeros();
    for (ti, tri) in a.indices.iter().enumerate() {
        if a.face_ids[ti] != cap_bot_f.id {
            continue;
        }
        let p0 = Vector3::from(a.vertices[tri[0] as usize].position);
        let p1 = Vector3::from(a.vertices[tri[1] as usize].position);
        let p2 = Vector3::from(a.vertices[tri[2] as usize].position);
        let area = 0.5 * (p1 - p0).cross(&(p2 - p0)).norm();
        if area > 1e-18 {
            bot_area += area;
            bot_ctr += ((p0 + p1 + p2) / 3.0) * area;
        }
    }
    if bot_area <= 1e-18 {
        return None;
    }
    bot_ctr /= bot_area;
    let s_bot = bot_ctr.dot(&axis);
    if (s_bot - s_min).abs() > (axial_span * 1e-4).max(1e-6) {
        return None;
    }

    let (e1, e2) = orthonormal_basis_from_z(&axis);
    let [c1, c2] = if let Some(cyl_f) = ref_cyl_face {
        let r_expected = cyl_f.radius?;
        if unit_vec3(cyl_f.normal_or_axis)?.dot(&axis).abs() <= 0.999 {
            return None;
        }
        let mut seen = HashSet::new();
        let mut pts_2d = Vec::new();
        for (ti, tri) in a.indices.iter().enumerate() {
            if a.face_ids[ti] != cyl_f.id {
                continue;
            }
            for &vi in tri {
                if seen.insert(vi) {
                    let p = Vector3::from(a.vertices[vi as usize].position);
                    pts_2d.push([p.dot(&e1), p.dot(&e2)]);
                }
            }
        }
        fit_circle_center_2d(&pts_2d, r_expected)?
    } else {
        [bot_ctr.dot(&e1), bot_ctr.dot(&e2)]
    };

    let origin_bot = axis * s_bot + e1 * c1 + e2 * c2;
    Some(CoaxialWorkpieceFrame {
        axis,
        origin_bot,
        axial_span,
    })
}

/// Proper 3D rotation mapping unit `axis` onto `+Z`.
fn rotation_to_z(axis: &Vector3<f64>) -> nalgebra::Rotation3<f64> {
    let dot = axis.dot(&Vector3::z());
    if dot > 1.0 - 1e-12 {
        nalgebra::Rotation3::identity()
    } else if dot < -1.0 + 1e-12 {
        nalgebra::Rotation3::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
    } else {
        nalgebra::Rotation3::rotation_between(
            &nalgebra::Unit::new_normalize(*axis),
            &Vector3::z_axis(),
        )
        .unwrap_or_else(nalgebra::Rotation3::identity)
    }
}

fn is_already_coaxial(frame: &CoaxialWorkpieceFrame) -> bool {
    (frame.axis - Vector3::z()).norm() < 1e-6
        && frame.origin_bot[0].hypot(frame.origin_bot[1]) < 1e-6
}

fn interval_overlap(a0: f64, a1: f64, b0: f64, b1: f64) -> f64 {
    (a1.min(b1) - a0.max(b0)).max(0.0)
}

/// Runs a local Z-coaxial CSG fast-path in `frame`'s local coordinate system (placing `origin_bot`
/// at `[0, 0, target_bot_z]` and `axis` along `+Z`), optionally transforming `b` into the same
/// local frame when `transform_b` is `true`, then transforms the resulting mesh back to world space.
fn execute_coaxial_in_local_frame<F>(
    a: &TriangleMesh,
    b: &TriangleMesh,
    is_difference: bool,
    transform_b: bool,
    frame: &CoaxialWorkpieceFrame,
    target_bot_z: f64,
    local_fn: F,
) -> Option<TriangleMesh>
where
    F: FnOnce(&TriangleMesh, &TriangleMesh, u32, &[FaceInfo]) -> Option<TriangleMesh>,
{
    let rot_to_z = rotation_to_z(&frame.axis);
    let shift = Vector3::new(0.0, 0.0, target_bot_z) - (rot_to_z * frame.origin_bot);

    let mut a_local = a.clone();
    a_local.apply_rot_and_shift(&rot_to_z, &shift);

    let b_local_owned = if transform_b {
        let mut bl = b.clone();
        bl.apply_rot_and_shift(&rot_to_z, &shift);
        Some(bl)
    } else {
        None
    };
    let b_local = b_local_owned.as_ref().unwrap_or(b);

    let (id_offset_local, combined_faces_local) =
        merge_csg_face_registries(&a_local, b_local, is_difference);
    let mut out = local_fn(&a_local, b_local, id_offset_local, &combined_faces_local)?;

    let rot_to_world = rot_to_z.inverse();
    let inv_shift = -(rot_to_world * shift);
    out.apply_rot_and_shift(&rot_to_world, &inv_shift);
    Some(out)
}

/// Default `start_z` for `cut_helical_groove` when the caller omits it:
/// uses world `bounding_box.min[2]` for upright Z-aligned parts (including XY/XYZ-shifted parts)
/// and centered `-0.5 * axial_span` for tilted/rotated coaxial workpieces.
pub fn default_helical_start_z(mesh: &TriangleMesh) -> f64 {
    if mesh.vertices.is_empty() {
        return 0.0;
    }
    let ref_cyl = mesh
        .faces
        .iter()
        .find(|f| f.role == "hole_wall" && f.kind == FaceKind::Cylinder)
        .or_else(|| {
            mesh.faces
                .iter()
                .find(|f| f.role == "side" && f.kind == FaceKind::Cylinder)
        });
    if let Some(frame) = detect_coaxial_workpiece_frame(mesh, ref_cyl) {
        if (frame.axis - Vector3::z()).norm() >= 1e-6 {
            return -0.5 * frame.axial_span;
        }
    }
    mesh.bounding_box().min[2]
}

/// Coaxial fast-path for cutting a through-hole (`cut_hole`) in a prismatic/cylindrical solid
/// whose outer shell lies strictly outside `r_bore`. Supports arbitrary coaxial axis/offset by
/// delegating into the workpiece's local Z frame and preserving the outer shell untouched.
fn try_cut_coaxial_through_bore(
    a: &TriangleMesh,
    b: &TriangleMesh,
    id_offset: u32,
    combined_faces: &[FaceInfo],
) -> Option<TriangleMesh> {
    if b.faces.iter().any(|f| f.role == "keyway_wall") {
        return None;
    }
    if a.faces
        .iter()
        .any(|f| f.role == "hole_wall" || f.kind == FaceKind::Helical)
    {
        return None;
    }
    let b_hole_wall = b.faces.iter().find(|f| {
        (f.role == "hole_wall" || (f.role == "side" && b.faces.len() == 3 && a.faces.len() > 3))
            && f.kind == FaceKind::Cylinder
    })?;
    let b_axis = unit_vec3(b_hole_wall.normal_or_axis)?;

    let ref_cyl = a
        .faces
        .iter()
        .find(|f| f.role == "side" && f.kind == FaceKind::Cylinder);
    let frame = detect_coaxial_workpiece_frame(a, ref_cyl)?;
    if b_axis.dot(&frame.axis).abs() <= 0.999 {
        return None;
    }

    if is_already_coaxial(&frame) {
        try_cut_coaxial_through_bore_local(a, b, id_offset, combined_faces)
    } else {
        execute_coaxial_in_local_frame(
            a,
            b,
            true,
            true,
            &frame,
            0.0,
            try_cut_coaxial_through_bore_local,
        )
    }
}

fn try_cut_coaxial_through_bore_local(
    a: &TriangleMesh,
    b: &TriangleMesh,
    id_offset: u32,
    combined_faces: &[FaceInfo],
) -> Option<TriangleMesh> {
    if b.faces.iter().any(|f| f.role == "keyway_wall") {
        return None;
    }
    let b_hole_wall = b.faces.iter().find(|f| {
        (f.role == "hole_wall" || (f.role == "side" && b.faces.len() == 3 && a.faces.len() > 3))
            && f.kind == FaceKind::Cylinder
            && f.normal_or_axis[2].abs() > 0.999
    })?;
    let r_bore = b_hole_wall.radius?;
    if r_bore <= 1e-9 {
        return None;
    }

    // Verify tool b is centered at X=0, Y=0 and spans beyond a's Z bounds
    let a_aabb = a.bounding_box();
    let b_aabb = b.bounding_box();
    let b_cx = 0.5 * (b_aabb.min[0] + b_aabb.max[0]);
    let b_cy = 0.5 * (b_aabb.min[1] + b_aabb.max[1]);
    if b_cx.hypot(b_cy) > r_bore * 1e-4
        || b_aabb.min[2] > a_aabb.min[2] - 1e-6
        || b_aabb.max[2] < a_aabb.max[2] + 1e-6
    {
        return None;
    }

    let cap_top_f = a.faces.iter().find(|f| {
        f.role == "cap_top" && f.kind == FaceKind::Plane && f.normal_or_axis[2] > 0.999
    })?;
    let cap_bot_f = a.faces.iter().find(|f| {
        f.role == "cap_bottom" && f.kind == FaceKind::Plane && f.normal_or_axis[2] < -0.999
    })?;
    if a.faces
        .iter()
        .any(|f| f.role == "hole_wall" || f.kind == FaceKind::Helical)
    {
        return None;
    }

    let excluded = [cap_top_f.id, cap_bot_f.id];
    let (shell_tris, top_outer_ring, bot_outer_ring, z_min, z_max) =
        extract_coaxial_outer_shell(a, &excluded, r_bore * 1.12)?;

    let m = top_outer_ring.len();
    let mut doublings = 0usize;
    while (m << doublings) < 24 && doublings < 3 {
        doublings += 1;
    }

    let mut vertices: Vec<Vertex> = Vec::with_capacity(shell_tris.len() * 3 + 512);
    let mut indices: Vec<[u32; 3]> = Vec::with_capacity(shell_tris.len() + 256);
    let mut face_ids: Vec<u32> = Vec::with_capacity(shell_tris.len() + 256);

    for (p0, p1, p2, n, fid) in shell_tris {
        let base = vertices.len() as u32;
        vertices.push(Vertex::new(p0, n));
        vertices.push(Vertex::new(p1, n));
        vertices.push(Vertex::new(p2, n));
        indices.push([base, base + 1, base + 2]);
        face_ids.push(fid);
    }

    let (top_bore_ring, _) = emit_doubled_and_bridged_annulus_cap(
        &mut vertices,
        &mut indices,
        &mut face_ids,
        cap_top_f.id,
        &top_outer_ring,
        r_bore,
        doublings,
        z_max,
        true,
    );
    let (bot_bore_ring, _) = emit_doubled_and_bridged_annulus_cap(
        &mut vertices,
        &mut indices,
        &mut face_ids,
        cap_bot_f.id,
        &bot_outer_ring,
        r_bore,
        doublings,
        z_min,
        false,
    );

    let k = top_bore_ring.len();
    let hole_wall_fid = b_hole_wall.id + id_offset;
    let height = z_max - z_min;
    let chord = (2.0 * std::f64::consts::PI * r_bore / (k as f64)).max(1e-6);
    let n_z = ((height / (chord * 1.25)).ceil() as usize).clamp(2, 64);

    for v in 0..n_z {
        let t0 = (v as f64) / (n_z as f64);
        let t1 = ((v + 1) as f64) / (n_z as f64);
        let z0 = z_min + t0 * height;
        let z1 = z_min + t1 * height;
        for u in 0..k {
            let u_next = (u + 1) % k;
            let p_bot_i = [
                bot_bore_ring[u][0] + (top_bore_ring[u][0] - bot_bore_ring[u][0]) * t0,
                bot_bore_ring[u][1] + (top_bore_ring[u][1] - bot_bore_ring[u][1]) * t0,
                z0,
            ];
            let p_bot_next = [
                bot_bore_ring[u_next][0]
                    + (top_bore_ring[u_next][0] - bot_bore_ring[u_next][0]) * t0,
                bot_bore_ring[u_next][1]
                    + (top_bore_ring[u_next][1] - bot_bore_ring[u_next][1]) * t0,
                z0,
            ];
            let p_top_i = [
                bot_bore_ring[u][0] + (top_bore_ring[u][0] - bot_bore_ring[u][0]) * t1,
                bot_bore_ring[u][1] + (top_bore_ring[u][1] - bot_bore_ring[u][1]) * t1,
                z1,
            ];
            let p_top_next = [
                bot_bore_ring[u_next][0]
                    + (top_bore_ring[u_next][0] - bot_bore_ring[u_next][0]) * t1,
                bot_bore_ring[u_next][1]
                    + (top_bore_ring[u_next][1] - bot_bore_ring[u_next][1]) * t1,
                z1,
            ];
            push_quad_exact_normals(
                &mut vertices,
                &mut indices,
                p_top_i,
                p_top_next,
                p_bot_next,
                p_bot_i,
            );
            face_ids.push(hole_wall_fid);
            face_ids.push(hole_wall_fid);
        }
    }

    let mut mesh = TriangleMesh::with_faces(vertices, indices, face_ids, combined_faces.to_vec());
    mesh.recompute_face_registry();
    Some(mesh)
}

/// Emits a coaxial cone wall from an ordered CCW `base_ring` at `z_bot` up to `z_top` with 2-to-1
/// vertex halving near sharp cone apexes (`radius_top <= 1e-12`).
#[allow(clippy::too_many_arguments)]
fn emit_cone_from_base_ring(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<[u32; 3]>,
    face_ids: &mut Vec<u32>,
    cone_side_fid: u32,
    cone_top_cap_fid: Option<u32>,
    base_ring: &[[f64; 2]],
    radius_bottom: f64,
    radius_top: f64,
    z_bot: f64,
    z_top: f64,
) {
    use std::f64::consts::PI;
    let segs = base_ring.len();
    if segs < 6 || z_top <= z_bot + 1e-12 {
        return;
    }
    let height = z_top - z_bot;
    let dr = radius_bottom - radius_top;
    let slant = (height * height + dr * dr).sqrt();
    let max_r = radius_bottom.max(radius_top);
    let base_chord = (2.0 * max_r * (PI / (segs as f64)).sin()).max(1e-6);
    let n_z = ((slant / (base_chord * 1.25)).ceil() as usize).clamp(4, 48);

    if radius_top <= 1e-12 {
        let mut curr_segs = segs;
        let mut prev_ring: Vec<[f64; 3]> = base_ring
            .iter()
            .map(|p| [p[0], p[1], z_bot])
            .collect();

        for k in 1..n_z {
            let t = (k as f64) / (n_z as f64);
            let r_curr = radius_bottom * (1.0 - t);
            let z_curr = z_bot + t * height;
            let chord_curr = 2.0 * PI * r_curr / (curr_segs as f64);
            let step_len = slant / (n_z as f64);

            if curr_segs >= 12 && curr_segs.is_multiple_of(2) && chord_curr < step_len * 0.55 {
                let half_segs = curr_segs / 2;
                let next_ring: Vec<[f64; 3]> = (0..half_segs)
                    .map(|j| {
                        let p_ref = prev_ring[2 * j];
                        let th = p_ref[1].atan2(p_ref[0]);
                        [r_curr * th.cos(), r_curr * th.sin(), z_curr]
                    })
                    .collect();
                for j in 0..half_segs {
                    let j_next = (j + 1) % half_segs;
                    let o0 = prev_ring[2 * j];
                    let o1 = prev_ring[2 * j + 1];
                    let o2 = prev_ring[(2 * j + 2) % curr_segs];
                    let i0 = next_ring[j];
                    let i1 = next_ring[j_next];
                    for (a, b, c) in [(o0, o1, i0), (o1, i1, i0), (o1, o2, i1)] {
                        let va = Vector3::new(a[0], a[1], a[2]);
                        let vb = Vector3::new(b[0], b[1], b[2]);
                        let vc = Vector3::new(c[0], c[1], c[2]);
                        let nv = (vb - va).cross(&(vc - va));
                        let nl = nv.norm().max(1e-15);
                        let norm = [nv.x / nl, nv.y / nl, nv.z / nl];
                        let base = vertices.len() as u32;
                        vertices.push(Vertex::new(a, norm));
                        vertices.push(Vertex::new(b, norm));
                        vertices.push(Vertex::new(c, norm));
                        indices.push([base, base + 1, base + 2]);
                        face_ids.push(cone_side_fid);
                    }
                }
                curr_segs = half_segs;
                prev_ring = next_ring;
            } else {
                let next_ring: Vec<[f64; 3]> = (0..curr_segs)
                    .map(|i| {
                        let p_ref = prev_ring[i];
                        let th = p_ref[1].atan2(p_ref[0]);
                        [r_curr * th.cos(), r_curr * th.sin(), z_curr]
                    })
                    .collect();
                for i in 0..curr_segs {
                    let next = (i + 1) % curr_segs;
                    push_quad_exact_normals(
                        vertices,
                        indices,
                        prev_ring[i],
                        prev_ring[next],
                        next_ring[next],
                        next_ring[i],
                    );
                    face_ids.push(cone_side_fid);
                    face_ids.push(cone_side_fid);
                }
                prev_ring = next_ring;
            }
        }

        let apex = [0.0, 0.0, z_top];
        for i in 0..curr_segs {
            let next = (i + 1) % curr_segs;
            let a = prev_ring[i];
            let b = prev_ring[next];
            let va = Vector3::new(a[0], a[1], a[2]);
            let vb = Vector3::new(b[0], b[1], b[2]);
            let vc = Vector3::new(apex[0], apex[1], apex[2]);
            let nv = (vb - va).cross(&(vc - va));
            let nl = nv.norm().max(1e-15);
            let norm = [nv.x / nl, nv.y / nl, nv.z / nl];
            let base = vertices.len() as u32;
            vertices.push(Vertex::new(a, norm));
            vertices.push(Vertex::new(b, norm));
            vertices.push(Vertex::new(apex, norm));
            indices.push([base, base + 1, base + 2]);
            face_ids.push(cone_side_fid);
        }
    } else {
        let mut prev_ring: Vec<[f64; 3]> = base_ring
            .iter()
            .map(|p| [p[0], p[1], z_bot])
            .collect();
        for k in 1..=n_z {
            let t = (k as f64) / (n_z as f64);
            let r_curr = radius_bottom + (radius_top - radius_bottom) * t;
            let z_curr = z_bot + t * height;
            let next_ring: Vec<[f64; 3]> = base_ring
                .iter()
                .map(|p| {
                    let th = p[1].atan2(p[0]);
                    [r_curr * th.cos(), r_curr * th.sin(), z_curr]
                })
                .collect();
            for i in 0..segs {
                let next = (i + 1) % segs;
                push_quad_exact_normals(
                    vertices,
                    indices,
                    prev_ring[i],
                    prev_ring[next],
                    next_ring[next],
                    next_ring[i],
                );
                face_ids.push(cone_side_fid);
                face_ids.push(cone_side_fid);
            }
            prev_ring = next_ring;
        }
        if let Some(top_fid) = cone_top_cap_fid {
            let top_ring_2d: Vec<[f64; 2]> = base_ring
                .iter()
                .map(|p| {
                    let th = p[1].atan2(p[0]);
                    [radius_top * th.cos(), radius_top * th.sin()]
                })
                .collect();
            crate::primitives::add_subdivided_disk_cap_2d(
                vertices,
                indices,
                &top_ring_2d,
                [0.0, 0.0],
                z_top,
                true,
            );
            face_ids.resize(indices.len(), top_fid);
        }
    }
}

/// Coaxial fast-path for `csg_union` when attaching a coaxial Z-axis `Cone` `b` onto the circular
/// top cap of a coaxial cylindrical (plain or helically fluted) body `a`.
/// Analytical coaxial fast-path for unioning a cone/frustum (`b`) onto the `cap_top` of `a`
/// in arbitrary workpiece orientation/offset.
fn try_union_coaxial_cone(
    a: &TriangleMesh,
    b: &TriangleMesh,
    id_offset: u32,
    combined_faces: &[FaceInfo],
) -> Option<TriangleMesh> {
    if a.faces.iter().any(|f| f.role == "hole_wall") {
        return None;
    }
    let b_cone_f = b.faces.iter().find(|f| f.kind == FaceKind::Cone)?;
    let b_bot_f = b
        .faces
        .iter()
        .find(|f| f.role == "cap_bottom" && f.kind == FaceKind::Plane)?;
    let b_cone_axis = unit_vec3(b_cone_f.normal_or_axis)?;
    let b_bot_axis = unit_vec3(b_bot_f.normal_or_axis)?;

    let ref_cyl = a
        .faces
        .iter()
        .find(|f| f.role == "side" && f.kind == FaceKind::Cylinder);
    let frame = detect_coaxial_workpiece_frame(a, ref_cyl)?;
    if b_cone_axis.dot(&frame.axis).abs() <= 0.999 || b_bot_axis.dot(&frame.axis) >= -0.999 {
        return None;
    }

    if is_already_coaxial(&frame) {
        try_union_coaxial_cone_local(a, b, id_offset, combined_faces)
    } else {
        execute_coaxial_in_local_frame(
            a,
            b,
            false,
            true,
            &frame,
            0.0,
            try_union_coaxial_cone_local,
        )
    }
}

fn try_union_coaxial_cone_local(
    a: &TriangleMesh,
    b: &TriangleMesh,
    id_offset: u32,
    combined_faces: &[FaceInfo],
) -> Option<TriangleMesh> {
    let b_cone_f = b
        .faces
        .iter()
        .find(|f| f.kind == FaceKind::Cone && f.normal_or_axis[2].abs() > 0.999)?;
    let b_bot_f = b.faces.iter().find(|f| {
        f.role == "cap_bottom" && f.kind == FaceKind::Plane && f.normal_or_axis[2] < -0.999
    })?;
    let r_cone_bot = b_bot_f.radius.or(b_cone_f.radius)?;
    if b_bot_f.centroid[0].hypot(b_bot_f.centroid[1]) > (r_cone_bot * 1e-3).max(1e-6) {
        return None;
    }
    let b_top_f = b
        .faces
        .iter()
        .find(|f| f.role == "cap_top" && f.kind == FaceKind::Plane && f.normal_or_axis[2] > 0.999);
    let r_cone_top = b_top_f.and_then(|f| f.radius).unwrap_or(0.0);

    let a_top_f = a.faces.iter().find(|f| {
        f.role == "cap_top" && f.kind == FaceKind::Plane && f.normal_or_axis[2] > 0.999
    })?;
    let a_bot_f = a.faces.iter().find(|f| {
        f.role == "cap_bottom" && f.kind == FaceKind::Plane && f.normal_or_axis[2] < -0.999
    })?;
    if a.faces.iter().any(|f| f.role == "hole_wall") {
        return None;
    }

    let a_aabb = a.bounding_box();
    let b_aabb = b.bounding_box();
    let z_seam = a_aabb.max[2];
    if (b_aabb.min[2] - z_seam).abs() > (r_cone_bot * 1e-4).max(1e-6)
        || b_aabb.max[2] <= z_seam + 1e-6
    {
        return None;
    }

    let excluded = [a_top_f.id, a_bot_f.id];
    let (_, top_rim, _, _, z_cyl_max) =
        extract_coaxial_outer_shell(a, &excluded, r_cone_bot * 0.15)?;
    let avg_r: f64 =
        top_rim.iter().map(|p| p[0].hypot(p[1])).sum::<f64>() / (top_rim.len() as f64);
    if (avg_r - r_cone_bot).abs() > r_cone_bot * 1e-3 {
        return None;
    }

    let mut vertices: Vec<Vertex> = Vec::with_capacity(a.vertices.len() + b.vertices.len());
    let mut indices: Vec<[u32; 3]> = Vec::with_capacity(a.indices.len() + b.indices.len());
    let mut face_ids: Vec<u32> = Vec::with_capacity(a.indices.len() + b.indices.len());

    // Keep all triangles of `a` except `cap_top`
    for (t_idx, tri) in a.indices.iter().enumerate() {
        let fid = a.face_ids[t_idx];
        if fid == a_top_f.id {
            continue;
        }
        let v0 = &a.vertices[tri[0] as usize];
        let v1 = &a.vertices[tri[1] as usize];
        let v2 = &a.vertices[tri[2] as usize];
        let base = vertices.len() as u32;
        vertices.push(*v0);
        vertices.push(*v1);
        vertices.push(*v2);
        indices.push([base, base + 1, base + 2]);
        face_ids.push(fid);
    }

    let cone_side_fid = b_cone_f.id + id_offset;
    let cone_top_fid = b_top_f.map(|f| f.id + id_offset);
    emit_cone_from_base_ring(
        &mut vertices,
        &mut indices,
        &mut face_ids,
        cone_side_fid,
        cone_top_fid,
        &top_rim,
        r_cone_bot,
        r_cone_top,
        z_cyl_max,
        b_aabb.max[2],
    );

    let mut mesh = TriangleMesh::with_faces(vertices, indices, face_ids, combined_faces.to_vec());
    mesh.recompute_face_registry();
    Some(mesh)
}

/// Analytical coaxial helical groove & thread cutter (`cut_helical_groove`) handling:
/// - Subcase 1: Internal helical thread/groove on a coaxial bored solid (e.g. `T08_hex_nut_m10`, `test_dag_polygon_prism_hex_nut_model`)
/// - Subcase 2: External helical flute/thread on a coaxial cylinder or cylinder+cone (`test_dag_helical_groove_drill_bit_model`, `DRILL_CONE`, `qa_wide_scan`)
///
/// Supports off-Z-axis and tilted/rotated workpieces by transforming `a` into its local axial
/// symmetry frame, generating the watertight coaxial threaded/fluted mesh, and transforming back.
/// Returns `None` for non-coaxial or non-cylindrical targets so `csg_difference` falls back to BSP-CSG.
fn try_cut_coaxial_helical_groove(
    a: &TriangleMesh,
    b: &TriangleMesh,
    id_offset: u32,
    combined_faces: &[FaceInfo],
) -> Option<TriangleMesh> {
    let params = HelicalGrooveParams::from_cutter_mesh(b)?;
    let ref_cyl = if let Some(hole_wall_f) = a
        .faces
        .iter()
        .find(|f| f.role == "hole_wall" && f.kind == FaceKind::Cylinder)
    {
        let r_bore = hole_wall_f.radius?;
        if (params.helix_radius - r_bore).abs() > params.groove_radius * 1.6 {
            return None;
        }
        hole_wall_f
    } else {
        let cyl_side_f = a
            .faces
            .iter()
            .find(|f| f.role == "side" && f.kind == FaceKind::Cylinder)?;
        if a.faces
            .iter()
            .any(|f| f.role == "hole_wall" || f.role.starts_with("side_"))
        {
            return None;
        }
        let r_cyl = cyl_side_f.radius?;
        if (params.helix_radius - r_cyl).abs() > params.groove_radius * 1.6 {
            return None;
        }
        cyl_side_f
    };

    let frame = detect_coaxial_workpiece_frame(a, Some(ref_cyl))?;
    if is_already_coaxial(&frame) {
        return try_cut_coaxial_helical_groove_local(a, b, id_offset, combined_faces);
    }

    let g_min = params.start_z;
    let g_max = params.start_z + params.height;
    let tol = 1e-9;
    let target_bot_z = if (frame.axis - Vector3::z()).norm() < 1e-6
        && interval_overlap(
            g_min,
            g_max,
            frame.origin_bot[2],
            frame.origin_bot[2] + frame.axial_span,
        ) > tol
    {
        frame.origin_bot[2]
    } else {
        let overlap_centered =
            interval_overlap(g_min, g_max, -0.5 * frame.axial_span, 0.5 * frame.axial_span);
        let overlap_base0 = interval_overlap(g_min, g_max, 0.0, frame.axial_span);
        if overlap_centered >= overlap_base0 && overlap_centered > tol {
            -0.5 * frame.axial_span
        } else if overlap_base0 > tol {
            0.0
        } else {
            return None;
        }
    };

    execute_coaxial_in_local_frame(
        a,
        b,
        true,
        false,
        &frame,
        target_bot_z,
        try_cut_coaxial_helical_groove_local,
    )
}

fn try_cut_coaxial_helical_groove_local(
    a: &TriangleMesh,
    b: &TriangleMesh,
    id_offset: u32,
    combined_faces: &[FaceInfo],
) -> Option<TriangleMesh> {
    use std::f64::consts::PI;
    let params = HelicalGrooveParams::from_cutter_mesh(b)?;
    let helical_fid = b.faces.first().map(|f| f.id + id_offset).unwrap_or(1 + id_offset);

    // Subcase 1: Internal helical thread/groove on a coaxial bored solid (has `hole_wall`)
    if let Some(hole_wall_f) = a.faces.iter().find(|f| {
        f.role == "hole_wall" && f.kind == FaceKind::Cylinder && f.normal_or_axis[2].abs() > 0.999
    }) {
        let r_bore = hole_wall_f.radius?;
        if (params.helix_radius - r_bore).abs() <= params.groove_radius * 1.6 {
            let cap_top_f = a.faces.iter().find(|f| {
                f.role == "cap_top" && f.kind == FaceKind::Plane && f.normal_or_axis[2] > 0.999
            })?;
            let cap_bot_f = a.faces.iter().find(|f| {
                f.role == "cap_bottom" && f.kind == FaceKind::Plane && f.normal_or_axis[2] < -0.999
            })?;

            let mut excluded = vec![cap_top_f.id, cap_bot_f.id, hole_wall_f.id];
            for f in &a.faces {
                if f.kind == FaceKind::Helical || f.role.starts_with("helical_") {
                    excluded.push(f.id);
                }
            }

            let max_thread_r = r_bore + params.groove_radius;
            let (shell_tris, top_outer_ring, bot_outer_ring, z_min, z_max) =
                extract_coaxial_outer_shell(a, &excluded, max_thread_r * 1.05)?;

            let m = top_outer_ring.len();
            let mut doublings = 0usize;
            while (m << doublings) < 48 && doublings < 3 {
                doublings += 1;
            }

            let mut vertices: Vec<Vertex> = Vec::with_capacity(shell_tris.len() * 3 + 4096);
            let mut indices: Vec<[u32; 3]> = Vec::with_capacity(shell_tris.len() + 2048);
            let mut face_ids: Vec<u32> = Vec::with_capacity(shell_tris.len() + 2048);

            for (p0, p1, p2, n, fid) in shell_tris {
                let base = vertices.len() as u32;
                vertices.push(Vertex::new(p0, n));
                vertices.push(Vertex::new(p1, n));
                vertices.push(Vertex::new(p2, n));
                indices.push([base, base + 1, base + 2]);
                face_ids.push(fid);
            }

            let (top_bore_ring, thetas) = emit_doubled_and_bridged_annulus_cap(
                &mut vertices,
                &mut indices,
                &mut face_ids,
                cap_top_f.id,
                &top_outer_ring,
                r_bore,
                doublings,
                z_max,
                true,
            );
            let (bot_bore_ring, _) = emit_doubled_and_bridged_annulus_cap(
                &mut vertices,
                &mut indices,
                &mut face_ids,
                cap_bot_f.id,
                &bot_outer_ring,
                r_bore,
                doublings,
                z_min,
                false,
            );

            let k = thetas.len();
            let z_span = z_max - z_min;
            let ds = (2.0 * PI * r_bore / (k as f64)).max(1e-6);
            let nz_min = ((z_span / (1.5 * ds)).ceil() as usize).max(12);
            let nz_max = ((z_span / (0.28 * ds)).floor() as usize).max(nz_min);
            let nz_target = ((z_span / params.pitch) * 8.0).round() as usize;
            let n_z = nz_target.clamp(nz_min, nz_max);

            let min_shell_r = top_outer_ring
                .iter()
                .map(|p| p[0].hypot(p[1]))
                .fold(f64::INFINITY, f64::min);
            let max_depth_cap = (min_shell_r - r_bore) * 0.55;

            let mut rings_3d: Vec<Vec<[f64; 3]>> = Vec::with_capacity(n_z + 1);
            let mut ring_has_groove: Vec<Vec<bool>> = Vec::with_capacity(n_z + 1);

            for v in 0..=n_z {
                let t = (v as f64) / (n_z as f64);
                let z = z_min + t * z_span;
                // Smooth 1-step chamfer lead-in/out at v=0 (z_min) and v=n_z (z_max) so cap rings are exact circles
                let axial_taper = if v == 0 || v == n_z {
                    0.0
                } else if v == 1 || v == n_z - 1 {
                    0.65
                } else {
                    1.0
                };
                let mut row_pts: Vec<[f64; 3]> = Vec::with_capacity(k);
                let mut row_groove: Vec<bool> = Vec::with_capacity(k);

                for u in 0..k {
                    if v == 0 {
                        row_pts.push([bot_bore_ring[u][0], bot_bore_ring[u][1], z_min]);
                        row_groove.push(false);
                    } else if v == n_z {
                        row_pts.push([top_bore_ring[u][0], top_bore_ring[u][1], z_max]);
                        row_groove.push(false);
                    } else {
                        let th = thetas[u];
                        let dr = params.radial_depth(r_bore, th, z, max_depth_cap) * axial_taper;
                        let r_pt = r_bore + dr;
                        row_pts.push([r_pt * th.cos(), r_pt * th.sin(), z]);
                        row_groove.push(dr > 1e-9);
                    }
                }
                rings_3d.push(row_pts);
                ring_has_groove.push(row_groove);
            }

            for v in 0..n_z {
                for u in 0..k {
                    let u_next = (u + 1) % k;
                    let p_bot_i = rings_3d[v][u];
                    let p_bot_next = rings_3d[v][u_next];
                    let p_top_i = rings_3d[v + 1][u];
                    let p_top_next = rings_3d[v + 1][u_next];
                    push_quad_exact_normals(
                        &mut vertices,
                        &mut indices,
                        p_top_i,
                        p_top_next,
                        p_bot_next,
                        p_bot_i,
                    );
                    let is_groove = ring_has_groove[v][u]
                        || ring_has_groove[v][u_next]
                        || ring_has_groove[v + 1][u]
                        || ring_has_groove[v + 1][u_next];
                    let fid = if is_groove { helical_fid } else { hole_wall_f.id };
                    face_ids.push(fid);
                    face_ids.push(fid);
                }
            }

            let mut mesh =
                TriangleMesh::with_faces(vertices, indices, face_ids, combined_faces.to_vec());
            mesh.recompute_face_registry();
            return Some(mesh);
        }
    }

    // Subcase 2: External helical flute / thread on a coaxial cylinder (optionally topped by a coaxial cone)
    let cyl_side_f = a.faces.iter().find(|f| {
        f.role == "side" && f.kind == FaceKind::Cylinder && f.normal_or_axis[2].abs() > 0.999
    })?;
    let r_cyl = cyl_side_f.radius?;
    if (params.helix_radius - r_cyl).abs() > params.groove_radius * 1.6 {
        return None;
    }
    if a.faces
        .iter()
        .any(|f| f.role == "hole_wall" || f.role.starts_with("side_"))
    {
        return None;
    }

    let cap_bot_f = a.faces.iter().find(|f| {
        f.role == "cap_bottom" && f.kind == FaceKind::Plane && f.normal_or_axis[2] < -0.999
    })?;
    let cap_top_f = a.faces.iter().find(|f| {
        f.role == "cap_top" && f.kind == FaceKind::Plane && f.normal_or_axis[2] > 0.999
    });
    let cone_side_f = a
        .faces
        .iter()
        .find(|f| f.kind == FaceKind::Cone && f.normal_or_axis[2].abs() > 0.999);

    let a_aabb = a.bounding_box();
    let z_min = a_aabb.min[2];
    let z_max = a_aabb.max[2];

    // Determine z_cyl_max (top of cylindrical section, before optional cone)
    let mut z_cyl_max = z_max;
    if cone_side_f.is_some() {
        let mut max_cyl_z = f64::NEG_INFINITY;
        for (t_idx, tri) in a.indices.iter().enumerate() {
            if a.face_ids[t_idx] == cyl_side_f.id {
                for &vi in tri {
                    max_cyl_z = max_cyl_z.max(a.vertices[vi as usize].position[2]);
                }
            }
        }
        if max_cyl_z > z_min + 1e-6 && max_cyl_z < z_max - 1e-6 {
            z_cyl_max = max_cyl_z;
        }
    }

    let k = 48usize;
    let cyl_height = z_cyl_max - z_min;
    if cyl_height <= 1e-9 {
        return None;
    }
    let ds = (2.0 * PI * r_cyl / (k as f64)).max(1e-6);
    let nz_min = ((cyl_height / (1.4 * ds)).ceil() as usize).max(12);
    let nz_max = ((cyl_height / (0.28 * ds)).floor() as usize).max(nz_min);
    let nz_target = ((cyl_height / params.pitch) * 10.0).round() as usize;
    let n_z = nz_target.clamp(nz_min, nz_max);

    let root_min_r = (r_cyl * 0.35).max(params.helix_radius - params.groove_radius);
    let max_depth_cap = (r_cyl - root_min_r).max(params.groove_radius * 0.25);

    let base_circle_2d: Vec<[f64; 2]> = (0..k)
        .map(|u| {
            let th = (u as f64) * 2.0 * PI / (k as f64);
            [r_cyl * th.cos(), r_cyl * th.sin()]
        })
        .collect();

    let mut vertices: Vec<Vertex> = Vec::with_capacity((k * n_z + 256) * 4);
    let mut indices: Vec<[u32; 3]> = Vec::with_capacity(k * n_z * 2 + 512);
    let mut face_ids: Vec<u32> = Vec::with_capacity(k * n_z * 2 + 512);

    // 1. Bottom cap at z_min
    crate::primitives::add_subdivided_disk_cap_2d(
        &mut vertices,
        &mut indices,
        &base_circle_2d,
        [0.0, 0.0],
        z_min,
        false,
    );
    face_ids.resize(indices.len(), cap_bot_f.id);

    // 2. Top cap at z_cyl_max (if no cone on top) or coaxial cone from z_cyl_max to z_max
    if let Some(cone_f) = cone_side_f {
        let r_cone_top = cap_top_f.and_then(|f| f.radius).unwrap_or(0.0);
        emit_cone_from_base_ring(
            &mut vertices,
            &mut indices,
            &mut face_ids,
            cone_f.id,
            cap_top_f.map(|f| f.id),
            &base_circle_2d,
            r_cyl,
            r_cone_top,
            z_cyl_max,
            z_max,
        );
    } else {
        let top_f = cap_top_f?;
        crate::primitives::add_subdivided_disk_cap_2d(
            &mut vertices,
            &mut indices,
            &base_circle_2d,
            [0.0, 0.0],
            z_cyl_max,
            true,
        );
        face_ids.resize(indices.len(), top_f.id);
    }

    // 3. Fluted / threaded cylindrical side wall from z_min to z_cyl_max
    let mut rings_3d: Vec<Vec<[f64; 3]>> = Vec::with_capacity(n_z + 1);
    let mut ring_has_groove: Vec<Vec<bool>> = Vec::with_capacity(n_z + 1);

    for v in 0..=n_z {
        let t = (v as f64) / (n_z as f64);
        let z = z_min + t * cyl_height;
        let axial_taper = if v == 0 || v == n_z {
            0.0
        } else if v == 1 || v == n_z - 1 {
            0.65
        } else {
            1.0
        };
        let mut row_pts: Vec<[f64; 3]> = Vec::with_capacity(k);
        let mut row_groove: Vec<bool> = Vec::with_capacity(k);

        for (u, base_pt) in base_circle_2d.iter().enumerate().take(k) {
            if v == 0 {
                row_pts.push([base_pt[0], base_pt[1], z_min]);
                row_groove.push(false);
            } else if v == n_z {
                row_pts.push([base_pt[0], base_pt[1], z_cyl_max]);
                row_groove.push(false);
            } else {
                let th = (u as f64) * 2.0 * PI / (k as f64);
                let dr = params.radial_depth(r_cyl, th, z, max_depth_cap) * axial_taper;
                let r_pt = (r_cyl - dr).max(r_cyl * 0.25);
                row_pts.push([r_pt * th.cos(), r_pt * th.sin(), z]);
                row_groove.push(dr > 1e-9);
            }
        }
        rings_3d.push(row_pts);
        ring_has_groove.push(row_groove);
    }

    for v in 0..n_z {
        for u in 0..k {
            let u_next = (u + 1) % k;
            let p_bot_i = rings_3d[v][u];
            let p_bot_next = rings_3d[v][u_next];
            let p_top_i = rings_3d[v + 1][u];
            let p_top_next = rings_3d[v + 1][u_next];
            push_quad_exact_normals(
                &mut vertices,
                &mut indices,
                p_top_i,
                p_bot_i,
                p_bot_next,
                p_top_next,
            );
            let is_groove = ring_has_groove[v][u]
                || ring_has_groove[v][u_next]
                || ring_has_groove[v + 1][u]
                || ring_has_groove[v + 1][u_next];
            let fid = if is_groove { helical_fid } else { cyl_side_f.id };
            face_ids.push(fid);
            face_ids.push(fid);
        }
    }

    let mut mesh = TriangleMesh::with_faces(vertices, indices, face_ids, combined_faces.to_vec());
    mesh.recompute_face_registry();
    Some(mesh)
}

/// Computes Boolean Union of two meshes: A ∪ B.
pub fn csg_union(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    if a.indices.is_empty() {
        return b.clone();
    }
    if b.indices.is_empty() {
        return a.clone();
    }

    let (id_offset, combined_faces) = merge_csg_face_registries(a, b, false);
    if let Some(coaxial_cone) = try_union_coaxial_cone(a, b, id_offset, &combined_faces) {
        return coaxial_cone;
    }

    let a_polys = BspNode::polygons_from_mesh(a);
    let mut b_polys = BspNode::polygons_from_mesh(b);
    if id_offset > 0 {
        for p in &mut b_polys {
            if p.face_id > 0 {
                p.face_id += id_offset;
            }
        }
    }

    let mut a_tree = BspNode::new();
    a_tree.build(a_polys.clone());
    let mut b_tree = BspNode::new();
    b_tree.build(b_polys.clone());

    let mut all = b_tree.clip_polygons(&a_polys);
    let mut b_clipped = a_tree.clip_polygons(&b_polys);
    flip_polygons(&mut b_clipped);
    b_clipped = a_tree.clip_polygons(&b_clipped);
    flip_polygons(&mut b_clipped);

    all.extend(b_clipped);
    let mut mesh = polygons_to_mesh(&all);
    if !combined_faces.is_empty() {
        mesh.faces = combined_faces;
        mesh.recompute_face_registry();
    }
    mesh
}

/// Computes Boolean Difference of two meshes: A \ B (A minus B).
pub fn csg_difference(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    if a.indices.is_empty() {
        return TriangleMesh::empty();
    }
    if b.indices.is_empty() {
        return a.clone();
    }

    let (id_offset, combined_faces) = merge_csg_face_registries(a, b, true);
    if let Some(helical_mesh) = try_cut_coaxial_helical_groove(a, b, id_offset, &combined_faces) {
        return helical_mesh;
    }
    if let Some(bored_mesh) = try_cut_coaxial_through_bore(a, b, id_offset, &combined_faces) {
        return bored_mesh;
    }

    let mut a_polys = BspNode::polygons_from_mesh(a);
    let mut b_polys = BspNode::polygons_from_mesh(b);
    if id_offset > 0 {
        for p in &mut b_polys {
            if p.face_id > 0 {
                p.face_id += id_offset;
            }
        }
    }

    let mut a_tree = BspNode::new();
    a_tree.build(a_polys.clone());
    let mut b_tree = BspNode::new();
    b_tree.build(b_polys.clone());

    flip_polygons(&mut a_polys);
    let mut all = b_tree.clip_polygons(&a_polys);
    flip_polygons(&mut all);

    a_tree.invert();
    let mut b_clipped = a_tree.clip_polygons(&b_polys);
    flip_polygons(&mut b_clipped);
    b_clipped = a_tree.clip_polygons(&b_clipped);

    all.extend(b_clipped);
    let mut mesh = polygons_to_mesh(&all);
    if !combined_faces.is_empty() {
        mesh.faces = combined_faces;
        mesh.recompute_face_registry();
    }
    mesh
}

/// Computes Boolean Intersection of two meshes: A ∩ B.
pub fn csg_intersection(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    if a.indices.is_empty() || b.indices.is_empty() {
        return TriangleMesh::empty();
    }

    let (id_offset, combined_faces) = merge_csg_face_registries(a, b, false);
    let mut a_polys = BspNode::polygons_from_mesh(a);
    let mut b_polys = BspNode::polygons_from_mesh(b);
    if id_offset > 0 {
        for p in &mut b_polys {
            if p.face_id > 0 {
                p.face_id += id_offset;
            }
        }
    }

    let mut a_tree = BspNode::new();
    a_tree.build(a_polys.clone());
    let mut b_tree = BspNode::new();
    b_tree.build(b_polys.clone());

    a_tree.invert();
    let mut b_clipped = a_tree.clip_polygons(&b_polys);
    flip_polygons(&mut b_clipped);
    b_tree.invert();
    flip_polygons(&mut a_polys);
    let mut a_clipped = b_tree.clip_polygons(&a_polys);
    b_clipped = a_tree.clip_polygons(&b_clipped);

    flip_polygons(&mut a_clipped);
    flip_polygons(&mut b_clipped);
    a_clipped.extend(b_clipped);
    let mut mesh = polygons_to_mesh(&a_clipped);
    if !combined_faces.is_empty() {
        mesh.faces = combined_faces;
        mesh.recompute_face_registry();
    }
    mesh
}
