//! Constructive Solid Geometry (CSG) operations via Binary Space Partitioning (BSP) trees.
//! Provides Boolean operations: Union, Difference, and Intersection on 3D meshes.

use crate::topology::{TriangleMesh, Vertex};
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
                });
            }
            if b_verts.len() >= 3 {
                back.push(CsgPolygon {
                    vertices: b_verts,
                    plane: poly.plane.clone(),
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

/// A convex planar polygon carrying vertex normals and a plane equation.
#[derive(Debug, Clone, PartialEq)]
pub struct CsgPolygon {
    pub vertices: Vec<Vertex>,
    pub plane: Plane,
}

impl CsgPolygon {
    /// Creates a polygon from a list of vertices.
    pub fn new(vertices: Vec<Vertex>) -> Option<Self> {
        if vertices.len() < 3 {
            return None;
        }
        let plane = Plane::from_points(
            vertices[0].pos_vec(),
            vertices[1].pos_vec(),
            vertices[2].pos_vec(),
        )?;
        Some(Self { vertices, plane })
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

    /// Builds a BSP tree from a triangle mesh.
    pub fn from_mesh(mesh: &TriangleMesh) -> Self {
        let mut polygons = Vec::with_capacity(mesh.indices.len());
        for tri in &mesh.indices {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];
            if let Some(poly) = CsgPolygon::new(vec![v0, v1, v2]) {
                polygons.push(poly);
            }
        }
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

    /// Recursively clips a list of polygons against this BSP tree.
    pub fn clip_polygons(&self, list: &[CsgPolygon]) -> Vec<CsgPolygon> {
        if self.plane.is_none() {
            return list.to_vec();
        }
        let plane = self.plane.as_ref().unwrap();

        let mut front_list = Vec::new();
        let mut back_list = Vec::new();
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();

        for poly in list {
            plane.split_polygon(
                poly,
                &mut coplanar_front,
                &mut coplanar_back,
                &mut front_list,
                &mut back_list,
            );
        }

        front_list.extend(coplanar_front);
        back_list.extend(coplanar_back);

        let mut result_front = if let Some(ref f) = self.front {
            f.clip_polygons(&front_list)
        } else {
            front_list
        };

        let result_back = if let Some(ref b) = self.back {
            b.clip_polygons(&back_list)
        } else {
            // Inside the solid; discard back polygons
            Vec::new()
        };

        result_front.extend(result_back);
        result_front
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

    let mut poly_loops: Vec<(Vec<u32>, Vector3<f64>)> = Vec::with_capacity(polygons.len());
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
            poly_loops.push((ids, poly.plane.normal));
        }
    }

    // 3. Subdivide polygon boundary edges at any collinear canonical vertices (T-junction prevention)
    let col_eps = (diag * 6e-5).max(2e-7);
    let col_eps_sq = col_eps * col_eps;
    let num_canon = canon_pos.len();

    let mut tris: Vec<([u32; 3], Vector3<f64>)> = Vec::new();

    for (raw_loop, n_poly) in poly_loops {
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
            if num_canon <= 4000 {
                for mid in 0..(num_canon as u32) {
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
                tris.push(([subdivided[0], subdivided[1], subdivided[2]], n_poly));
            }
        } else {
            triangulate_subdivided_convex_loop(
                &subdivided,
                n_poly,
                &canon_pos,
                col_eps_sq,
                &mut tris,
            );
        }
    }

    // 4. Post-triangulation manifold stitching & residual T-junction / slit resolution
    repair_manifold_triangles(&mut tris, &canon_pos, diag);

    // 5. Emit TriangleMesh with crisp per-face normals
    let mut vertices = Vec::with_capacity(tris.len() * 3);
    let mut indices = Vec::with_capacity(tris.len());

    for (tri, hint_normal) in tris {
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
    }

    TriangleMesh::new(vertices, indices)
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
    canon_pos: &[Vector3<f64>],
    col_eps_sq: f64,
    out_tris: &mut Vec<([u32; 3], Vector3<f64>)>,
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

            let quality = signed_area2 / perim_sq;
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
            out_tris.push(([prev, curr, next], n_poly));
        }
        ring.remove(clip_i);
        clean_index_loop(&mut ring);
    }

    if ring.len() == 3 {
        let p0 = canon_pos[ring[0] as usize];
        let p1 = canon_pos[ring[1] as usize];
        let p2 = canon_pos[ring[2] as usize];
        if (p1 - p0).cross(&(p2 - p0)).norm() > 1e-13 {
            out_tris.push(([ring[0], ring[1], ring[2]], n_poly));
        }
    }
}

/// Resolves any residual T-junctions or 3-edge micro-slits on unmatched boundary edges
/// without ever introducing non-manifold edges or out-of-plane folds.
fn repair_manifold_triangles(
    tris: &mut Vec<([u32; 3], Vector3<f64>)>,
    canon_pos: &[Vector3<f64>],
    diag: f64,
) {
    let split_tol_sq = (diag * 2e-4).max(1e-7).powi(2);
    let cycle_tol_sq = (diag * 2e-3).max(1e-6).powi(2);
    let max_passes = if tris.len() > 2000 { 2 } else { 8 };

    for _pass in 0..max_passes {
        // 1. Remove degenerate or exact duplicate triangles
        let mut seen_tris: HashSet<[u32; 3]> = HashSet::with_capacity(tris.len());
        tris.retain(|(t, _)| {
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
        for (t, _) in tris.iter() {
            for e in 0..3 {
                let u = t[e];
                let v = t[(e + 1) % 3];
                *edge_counts.entry((u, v)).or_insert(0) += 1;
            }
        }

        // 3. Cancel opposite-winding duplicate triangle pairs ([a, b, c] and [a, c, b] on same 3 vertices)
        // if any of their edges is non-manifold (undirected count >= 3), and prune spurious fin triangles
        // where no edge has undirected count == 2 (i.e. all edges have count 1 or >= 3, with at least one >= 3).
        let mut pruned_any = false;
        let mut unordered_map: HashMap<[u32; 3], Vec<usize>> = HashMap::new();
        for (idx, (t, _)) in tris.iter().enumerate() {
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
                    // Remove one or both depending on whether the remaining edges need one copy
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

            let u0 = edge_counts.get(&e0).copied().unwrap_or(0)
                + edge_counts.get(&(e0.1, e0.0)).copied().unwrap_or(0);
            let u1 = edge_counts.get(&e1).copied().unwrap_or(0)
                + edge_counts.get(&(e1.1, e1.0)).copied().unwrap_or(0);
            let u2 = edge_counts.get(&e2).copied().unwrap_or(0)
                + edge_counts.get(&(e2.1, e2.0)).copied().unwrap_or(0);

            // If no edge has undirected count == 2 (so removing `t` cannot create any new boundary edge)
            // and at least one edge is non-manifold (count >= 3), `t` is a redundant internal face or fin flap.
            if u0 != 2 && u1 != 2 && u2 != 2 && (u0 >= 3 || u1 >= 3 || u2 >= 3) {
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
        let mut next_tris: Vec<([u32; 3], Vector3<f64>)> = Vec::with_capacity(tris.len() + 16);

        for &(t, n_hint) in tris.iter() {
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
                    next_tris.push(([u, m, w], n_hint));
                    next_tris.push(([m, v, w], n_hint));
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
                next_tris.push((t, n_hint));
            }
        }

        *tris = next_tris;
        if split_any {
            continue;
        }

        // 5. Seal closed 3-edge micro-hole loops u -> v -> m -> u ONLY if each edge is a true boundary edge
        // (undirected count == 1, i.e., opposite count == 0 and forward count == 1)
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
                            tris.push(([u, m, v], cross / len));
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
        for (t, _) in tris.iter() {
            for e in 0..3 {
                *edge_counts.entry((t[e], t[(e + 1) % 3])).or_insert(0) += 1;
            }
        }
        let mut flipped_any = false;
        for (t, n_hint) in tris.iter_mut() {
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

/// Computes Boolean Union of two meshes: A ∪ B.
pub fn csg_union(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    if a.indices.is_empty() {
        return b.clone();
    }
    if b.indices.is_empty() {
        return a.clone();
    }

    let mut a_tree = BspNode::from_mesh(a);
    let mut b_tree = BspNode::from_mesh(b);

    a_tree.clip_to(&b_tree);
    b_tree.clip_to(&a_tree);
    b_tree.invert();
    b_tree.clip_to(&a_tree);
    b_tree.invert();

    a_tree.build(b_tree.all_polygons());
    polygons_to_mesh(&a_tree.all_polygons())
}

/// Computes Boolean Difference of two meshes: A \ B (A minus B).
pub fn csg_difference(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    if a.indices.is_empty() {
        return TriangleMesh::empty();
    }
    if b.indices.is_empty() {
        return a.clone();
    }

    let mut a_tree = BspNode::from_mesh(a);
    let mut b_tree = BspNode::from_mesh(b);

    a_tree.invert();
    a_tree.clip_to(&b_tree);
    b_tree.clip_to(&a_tree);
    b_tree.invert();
    b_tree.clip_to(&a_tree);
    b_tree.invert();

    a_tree.build(b_tree.all_polygons());
    a_tree.invert();
    polygons_to_mesh(&a_tree.all_polygons())
}

/// Computes Boolean Intersection of two meshes: A ∩ B.
pub fn csg_intersection(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    if a.indices.is_empty() || b.indices.is_empty() {
        return TriangleMesh::empty();
    }

    let mut a_tree = BspNode::from_mesh(a);
    let mut b_tree = BspNode::from_mesh(b);

    a_tree.invert();
    b_tree.clip_to(&a_tree);
    b_tree.invert();
    a_tree.clip_to(&b_tree);
    b_tree.clip_to(&a_tree);

    a_tree.build(b_tree.all_polygons());
    a_tree.invert();
    polygons_to_mesh(&a_tree.all_polygons())
}
