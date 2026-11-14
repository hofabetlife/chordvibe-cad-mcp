//! P3-T5 Spike: Boundary-Conforming FEA Mesher (`fe_snap`)
//!
//! Evaluates boundary-conforming tetrahedral mesh generation via:
//! - Baseline: Unsnapped cartesian voxel tetrahedral mesh (`TetrahedralMesh::from_mesh`)
//! - Candidate (a): Boundary-snapped + Feature-locked + Quality-guarded Laplacian smoothed
//!   tetrahedral mesh (`snap_voxel_mesh_to_surface`), both in linear (`TET4`) and quadratic
//!   isoparametric (`TET10`, with curved boundary mid-edge snapping) configurations.
//!
//! Evaluates the 3 required engineering bodies from P3-T5 (`fix/P3_calculations.md`):
//! 1. Perforated plate with central circular hole (600 x 200 x 10 mm, hole D = 40 mm, F_x = 100 kN).
//! 2. Stepped shaft with circumferential groove (D1 = 40 mm, D2 = 28 mm, groove D_g = 22 mm, L = 120 mm).
//! 3. Spur gear tooth sector / full gear (m = 3 mm, z = 20, b = 15 mm, involute profile).

#![allow(clippy::single_element_loop, clippy::type_complexity, clippy::too_many_arguments, clippy::needless_range_loop, clippy::doc_overindented_list_items)]

use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::time::Instant;

use nalgebra::{Matrix3, Matrix4, Vector3};

use mcp_cad_core::csg::{csg_difference, csg_union};
use mcp_cad_core::material::Material;
use mcp_cad_core::primitives::{
    create_box, create_cylinder, create_revolve, create_spur_gear, cut_hole,
};
use mcp_cad_core::topology::TriangleMesh;
use mcp_cad_fea::analyze_mesh;
use mcp_cad_fea::boundary::{
    point_to_triangle_distance_3d, AabbFace, BoundaryConditions, DirichletConstraint, DofComponent,
};
use mcp_cad_fea::element::{tet10_barycentric_derivatives, TET10_EDGE_PAIRS, TET10_GAUSS_POINTS};
use mcp_cad_fea::solver::SolverOptions;
use mcp_cad_fea::tetra_mesh::{MeshDensity, TetElement, TetNode, TetrahedralMesh};

/// Computes the closest point on 3D triangle `(a, b, c)` to point `p`, returning
/// `(closest_point, distance, unit_normal)`.
pub fn closest_point_on_triangle_3d(
    p: [f64; 3],
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
) -> ([f64; 3], f64, [f64; 3]) {
    let vp = Vector3::from(p);
    let va = Vector3::from(a);
    let vb = Vector3::from(b);
    let vc = Vector3::from(c);

    let ab = vb - va;
    let ac = vc - va;
    let cross = ab.cross(&ac);
    let cross_norm = cross.norm();
    let unit_n = if cross_norm > 1e-18 {
        let n = cross / cross_norm;
        [n.x, n.y, n.z]
    } else {
        [0.0, 0.0, 1.0]
    };

    let ap = vp - va;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, (vp - va).norm(), unit_n);
    }

    let bp = vp - vb;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, (vp - vb).norm(), unit_n);
    }

    let vc_region = d1 * d4 - d3 * d2;
    if vc_region <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        let v = if denom.abs() > 1e-18 { d1 / denom } else { 0.0 };
        let q = va + ab * v;
        return ([q.x, q.y, q.z], (vp - q).norm(), unit_n);
    }

    let cp = vp - vc;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, (vp - vc).norm(), unit_n);
    }

    let vb_region = d5 * d2 - d1 * d6;
    if vb_region <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        let w = if denom.abs() > 1e-18 { d2 / denom } else { 0.0 };
        let q = va + ac * w;
        return ([q.x, q.y, q.z], (vp - q).norm(), unit_n);
    }

    let va_region = d3 * d6 - d5 * d4;
    if va_region <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let denom = (d4 - d3) + (d5 - d6);
        let w = if denom.abs() > 1e-18 {
            (d4 - d3) / denom
        } else {
            0.0
        };
        let q = vb + (vc - vb) * w;
        return ([q.x, q.y, q.z], (vp - q).norm(), unit_n);
    }

    let denom = va_region + vb_region + vc_region;
    let q = if denom.abs() > 1e-18 {
        let v = vb_region / denom;
        let w = vc_region / denom;
        va + ab * v + ac * w
    } else {
        va
    };
    ([q.x, q.y, q.z], (vp - q).norm(), unit_n)
}

/// Spatial acceleration structure for fast $O(1)$ point-to-triangle-mesh projection.
#[derive(Debug, Clone)]
pub struct SurfaceSpatialIndex<'a> {
    pub surface: &'a TriangleMesh,
    cell_size: f64,
    inv_cell: f64,
    min_bound: [f64; 3],
    grid: HashMap<(i32, i32, i32), Vec<usize>>,
}

impl<'a> SurfaceSpatialIndex<'a> {
    pub fn build(surface: &'a TriangleMesh) -> Self {
        let mut min_b = [f64::INFINITY; 3];
        let mut max_b = [-f64::INFINITY; 3];
        for v in &surface.vertices {
            for i in 0..3 {
                if v.position[i] < min_b[i] { min_b[i] = v.position[i]; }
                if v.position[i] > max_b[i] { max_b[i] = v.position[i]; }
            }
        }
        let diag = ((max_b[0] - min_b[0]).powi(2) + (max_b[1] - min_b[1]).powi(2) + (max_b[2] - min_b[2]).powi(2)).sqrt();
        let n_tri = surface.indices.len().max(1);
        let cell_size = (diag / (n_tri as f64).cbrt().max(1.0)).max(1e-4);
        let inv_cell = 1.0 / cell_size;

        let mut grid: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();
        for (idx, tri) in surface.indices.iter().enumerate() {
            let a = surface.vertices[tri[0] as usize].position;
            let b = surface.vertices[tri[1] as usize].position;
            let c = surface.vertices[tri[2] as usize].position;
            let min_x = a[0].min(b[0]).min(c[0]);
            let max_x = a[0].max(b[0]).max(c[0]);
            let min_y = a[1].min(b[1]).min(c[1]);
            let max_y = a[1].max(b[1]).max(c[1]);
            let min_z = a[2].min(b[2]).min(c[2]);
            let max_z = a[2].max(b[2]).max(c[2]);

            let i0 = ((min_x - min_b[0]) * inv_cell).floor() as i32;
            let i1 = ((max_x - min_b[0]) * inv_cell).floor() as i32;
            let j0 = ((min_y - min_b[1]) * inv_cell).floor() as i32;
            let j1 = ((max_y - min_b[1]) * inv_cell).floor() as i32;
            let k0 = ((min_z - min_b[2]) * inv_cell).floor() as i32;
            let k1 = ((max_z - min_b[2]) * inv_cell).floor() as i32;

            for i in i0..=i1 {
                for j in j0..=j1 {
                    for k in k0..=k1 {
                        grid.entry((i, j, k)).or_default().push(idx);
                    }
                }
            }
        }
        Self { surface, cell_size, inv_cell, min_bound: min_b, grid }
    }

    pub fn project_point(&self, p: [f64; 3]) -> ([f64; 3], f64, usize, [f64; 3]) {
        let ci = ((p[0] - self.min_bound[0]) * self.inv_cell).floor() as i32;
        let cj = ((p[1] - self.min_bound[1]) * self.inv_cell).floor() as i32;
        let ck = ((p[2] - self.min_bound[2]) * self.inv_cell).floor() as i32;

        let mut best_pt = p;
        let mut best_dist = f64::INFINITY;
        let mut best_idx = 0usize;
        let mut best_normal = [0.0, 0.0, 1.0];
        let mut visited = HashSet::new();

        for r in 0_i32..=4_i32 {
            let mut found_cell = false;
            for di in -r..=r {
                for dj in -r..=r {
                    for dk in -r..=r {
                        if di.abs() != r && dj.abs() != r && dk.abs() != r {
                            continue;
                        }
                        if let Some(list) = self.grid.get(&(ci + di, cj + dj, ck + dk)) {
                            found_cell = true;
                            for &idx in list {
                                if visited.insert(idx) {
                                    let tri = &self.surface.indices[idx];
                                    let a = self.surface.vertices[tri[0] as usize].position;
                                    let b = self.surface.vertices[tri[1] as usize].position;
                                    let c = self.surface.vertices[tri[2] as usize].position;
                                    let (q, d, n) = closest_point_on_triangle_3d(p, a, b, c);
                                    if d < best_dist {
                                        best_dist = d;
                                        best_pt = q;
                                        best_idx = idx;
                                        best_normal = n;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if best_dist < (r as f64) * self.cell_size && found_cell {
                break;
            }
        }

        if best_dist.is_infinite() {
            for (idx, tri) in self.surface.indices.iter().enumerate() {
                let a = self.surface.vertices[tri[0] as usize].position;
                let b = self.surface.vertices[tri[1] as usize].position;
                let c = self.surface.vertices[tri[2] as usize].position;
                let (q, d, n) = closest_point_on_triangle_3d(p, a, b, c);
                if d < best_dist {
                    best_dist = d;
                    best_pt = q;
                    best_idx = idx;
                    best_normal = n;
                }
            }
        }

        (best_pt, best_dist, best_idx, best_normal)
    }
}

/// Closest point projection onto a `TriangleMesh`, returning `(closest_point, distance, tri_idx, normal)`.
pub fn project_point_to_surface_mesh(
    p: [f64; 3],
    surface: &TriangleMesh,
) -> ([f64; 3], f64, usize, [f64; 3]) {
    let spatial = SurfaceSpatialIndex::build(surface);
    spatial.project_point(p)
}

/// Computes the signed 6*V determinant `(v1 - v0) . ((v2 - v0) x (v3 - v0))` of a tetrahedron.
#[inline]
pub fn tet_det_6v(p0: [f64; 3], p1: [f64; 3], p2: [f64; 3], p3: [f64; 3]) -> f64 {
    let v0 = Vector3::from(p0);
    let v1 = Vector3::from(p1);
    let v2 = Vector3::from(p2);
    let v3 = Vector3::from(p3);
    (v1 - v0).dot(&(v2 - v0).cross(&(v3 - v0)))
}

/// Computes the Knupp / Liu-Joe normalized scaled Jacobian / mean-ratio quality in `[-1.0, 1.0]`
/// for a tetrahedron `(p0, p1, p2, p3)`.
///
/// Specifically:
/// - Compute the minimum corner scaled Jacobian across all 4 vertices:
///   $J_s = \min_{k \in 0..4} \frac{\det(\mathbf{e}_{k,1}, \mathbf{e}_{k,2}, \mathbf{e}_{k,3}) \cdot \sqrt{2}}{\|\mathbf{e}_{k,1}\| \|\mathbf{e}_{k,2}\| \|\mathbf{e}_{k,3}\|}$
///   (normalized so that a regular equilateral tetrahedron has $J_s = 1.0$, and an orthogonal
///   corner of a Kuhn cube-split tetrahedron has $\det / (l_1 l_2 l_3) = 1 / \sqrt{6} \implies J_s = 1/\sqrt{3} \approx 0.5774$).
/// - Also compute the Liu-Joe mean ratio $q_{\text{LJ}} \in [-1.0, 1.0]$.
pub fn tet_scaled_jacobian(p0: [f64; 3], p1: [f64; 3], p2: [f64; 3], p3: [f64; 3]) -> f64 {
    let v = [
        Vector3::from(p0),
        Vector3::from(p1),
        Vector3::from(p2),
        Vector3::from(p3),
    ];
    let det6 = (v[1] - v[0]).dot(&(v[2] - v[0]).cross(&(v[3] - v[0])));
    let sign = if det6 >= 0.0 { 1.0 } else { -1.0 };
    let abs_det = det6.abs();
    if abs_det <= 1e-30 {
        return 0.0;
    }

    // Corner edge triples (oriented positively when det6 > 0)
    let corners = [(0, 1, 2, 3), (1, 0, 3, 2), (2, 0, 1, 3), (3, 0, 2, 1)];
    let mut min_corner = f64::INFINITY;
    for &(c, a, b, d) in &corners {
        let l1 = (v[a] - v[c]).norm();
        let l2 = (v[b] - v[c]).norm();
        let l3 = (v[d] - v[c]).norm();
        let denom = l1 * l2 * l3;
        if denom <= 1e-30 {
            return 0.0;
        }
        // For a regular equilateral tetrahedron of edge length 1, abs_det = sqrt(2)/2,
        // so abs_det * sqrt(2) / (l1*l2*l3) == 1.0.
        let j_corner = (abs_det * std::f64::consts::SQRT_2 / denom).min(1.0);
        if j_corner < min_corner {
            min_corner = j_corner;
        }
    }
    sign * min_corner
}

/// Computes the signed Liu-Joe mean-ratio shape quality in `[-1.0, 1.0]`.
pub fn tet_liu_joe_quality(p0: [f64; 3], p1: [f64; 3], p2: [f64; 3], p3: [f64; 3]) -> f64 {
    let v = [
        Vector3::from(p0),
        Vector3::from(p1),
        Vector3::from(p2),
        Vector3::from(p3),
    ];
    let det6 = (v[1] - v[0]).dot(&(v[2] - v[0]).cross(&(v[3] - v[0])));
    let sign = if det6 >= 0.0 { 1.0 } else { -1.0 };
    let vol = det6.abs() / 6.0;
    if vol <= 1e-30 {
        return 0.0;
    }
    let mut sum_edge_sq = 0.0;
    for i in 0..4 {
        for j in (i + 1)..4 {
            sum_edge_sq += (v[i] - v[j]).norm_squared();
        }
    }
    if sum_edge_sq <= 1e-30 {
        return 0.0;
    }
    let q = (12.0 * (3.0 * vol).powf(2.0 / 3.0) / sum_edge_sq).clamp(0.0, 1.0);
    sign * q
}

/// Summary of mesh geometric and element quality metrics.
#[derive(Debug, Clone)]
pub struct MeshQualityMetrics {
    pub element_count: usize,
    pub node_count: usize,
    pub inverted_elements: usize,
    pub min_scaled_jacobian: f64,
    pub mean_scaled_jacobian: f64,
    pub min_liu_joe: f64,
    pub mean_liu_joe: f64,
    pub max_boundary_deviation_mm: f64,
    pub rms_boundary_deviation_mm: f64,
    pub max_curved_feature_deviation_mm: f64,
    pub rms_curved_feature_deviation_mm: f64,
}

/// Evaluates mesh quality and boundary surface deviation against the exact CAD `TriangleMesh`
/// and an optional curved feature subset of triangles (e.g., hole wall, shaft groove, gear flank).
pub fn evaluate_mesh_metrics(
    tet_mesh: &TetrahedralMesh,
    cad_mesh: &TriangleMesh,
    curved_feature_triangles: Option<&[[[f64; 3]; 3]]>,
    feature_search_tol: f64,
) -> MeshQualityMetrics {
    let mut min_sj = f64::INFINITY;
    let mut sum_sj = 0.0;
    let mut min_lj = f64::INFINITY;
    let mut sum_lj = 0.0;
    let mut inverted = 0usize;

    for elem in &tet_mesh.elements {
        let [i0, i1, i2, i3] = elem.node_indices;
        let p0 = tet_mesh.nodes[i0].position;
        let p1 = tet_mesh.nodes[i1].position;
        let p2 = tet_mesh.nodes[i2].position;
        let p3 = tet_mesh.nodes[i3].position;

        let sj = tet_scaled_jacobian(p0, p1, p2, p3);
        let lj = tet_liu_joe_quality(p0, p1, p2, p3);
        if sj <= 0.0 {
            inverted += 1;
        }
        if sj < min_sj {
            min_sj = sj;
        }
        sum_sj += sj;
        if lj < min_lj {
            min_lj = lj;
        }
        sum_lj += lj;
    }

    let n_elem = tet_mesh.elements.len().max(1) as f64;
    let mean_sj = sum_sj / n_elem;
    let mean_lj = sum_lj / n_elem;

    // Extract boundary nodes (including mid-edge boundary nodes if TET10)
    let surf_tris = tet_mesh.extract_surface_triangles();
    let mut boundary_nodes = HashSet::new();
    for &[a, b, c] in &surf_tris {
        boundary_nodes.insert(a);
        boundary_nodes.insert(b);
        boundary_nodes.insert(c);
    }

    let mut max_dev_m = 0.0_f64;
    let mut sum_sq_dev_m = 0.0_f64;
    let mut b_count = 0usize;

    let mut max_feat_dev_m = 0.0_f64;
    let mut sum_sq_feat_dev_m = 0.0_f64;
    let mut feat_count = 0usize;

    let spatial = SurfaceSpatialIndex::build(cad_mesh);
    for &nid in &boundary_nodes {
        let p = tet_mesh.nodes[nid].position;
        let (_, d_cad, _, _) = spatial.project_point(p);
        if d_cad > max_dev_m {
            max_dev_m = d_cad;
        }
        sum_sq_dev_m += d_cad * d_cad;
        b_count += 1;

        if let Some(feat_tris) = curved_feature_triangles {
            let d_feat = feat_tris
                .iter()
                .map(|t| point_to_triangle_distance_3d(p, t[0], t[1], t[2]))
                .fold(f64::INFINITY, f64::min);
            if d_feat <= (d_cad + 1e-5).max(feature_search_tol) && d_feat <= feature_search_tol {
                if d_feat > max_feat_dev_m {
                    max_feat_dev_m = d_feat;
                }
                sum_sq_feat_dev_m += d_feat * d_feat;
                feat_count += 1;
            }
        }
    }

    let rms_dev_m = if b_count > 0 {
        (sum_sq_dev_m / (b_count as f64)).sqrt()
    } else {
        0.0
    };
    let rms_feat_dev_m = if feat_count > 0 {
        (sum_sq_feat_dev_m / (feat_count as f64)).sqrt()
    } else {
        rms_dev_m
    };
    if feat_count == 0 {
        max_feat_dev_m = max_dev_m;
    }

    MeshQualityMetrics {
        element_count: tet_mesh.elements.len(),
        node_count: tet_mesh.nodes.len(),
        inverted_elements: inverted,
        min_scaled_jacobian: min_sj,
        mean_scaled_jacobian: mean_sj,
        min_liu_joe: min_lj,
        mean_liu_joe: mean_lj,
        max_boundary_deviation_mm: max_dev_m * 1000.0,
        rms_boundary_deviation_mm: rms_dev_m * 1000.0,
        max_curved_feature_deviation_mm: max_feat_dev_m * 1000.0,
        rms_curved_feature_deviation_mm: rms_feat_dev_m * 1000.0,
    }
}

/// Options for boundary-conforming voxel snapping and interior smoothing (`fe_snap`).
#[derive(Debug, Clone, Copy)]
pub struct SnapOptions {
    /// Minimum allowed scaled Jacobian $J_s > 0$ during node projection and smoothing.
    pub min_allowed_scaled_jacobian: f64,
    /// Number of alternating boundary-snap + interior Laplacian relaxation passes.
    pub outer_passes: usize,
    /// Number of interior quality-guarded Laplacian smoothing sweeps per pass.
    pub smoothing_sweeps: usize,
    /// Number of tangential surface relaxation sweeps for boundary nodes.
    pub surface_tangential_sweeps: usize,
}

impl Default for SnapOptions {
    fn default() -> Self {
        Self {
            min_allowed_scaled_jacobian: 0.08,
            outer_passes: 3,
            smoothing_sweeps: 2,
            surface_tangential_sweeps: 1,
        }
    }
}

/// Candidate (a): Projects exterior boundary vertices of a voxel `TetrahedralMesh` onto the exact
/// triangles of `cad_surface` with line-search backtracking to guarantee $\det J > 0$ and
/// $J_s \ge J_{\min}$, interleaved with quality-guarded interior Laplacian smoothing and
/// tangential surface relaxation.
pub fn snap_voxel_mesh_to_surface(
    voxel_mesh: &TetrahedralMesh,
    cad_surface: &TriangleMesh,
    opts: SnapOptions,
) -> TetrahedralMesh {
    let mut mesh = TetrahedralMesh::new(voxel_mesh.nodes.clone(), voxel_mesh.elements.clone());
    let num_nodes = mesh.nodes.len();
    if num_nodes == 0 || mesh.elements.is_empty() {
        return mesh;
    }

    // 1. Identify exterior boundary triangles and boundary nodes
    let surf_tris = mesh.extract_surface_triangles();
    let mut is_boundary = vec![false; num_nodes];
    let mut boundary_adj: HashMap<usize, Vec<usize>> = HashMap::new();
    for &[a, b, c] in &surf_tris {
        is_boundary[a] = true;
        is_boundary[b] = true;
        is_boundary[c] = true;
        for (u, v) in [(a, b), (b, c), (c, a)] {
            let list = boundary_adj.entry(u).or_default();
            if !list.contains(&v) {
                list.push(v);
            }
            let list_v = boundary_adj.entry(v).or_default();
            if !list_v.contains(&u) {
                list_v.push(u);
            }
        }
    }

    // 2. Build node -> incident elements and node -> 1-ring volume neighbors
    let mut node_to_elems: Vec<Vec<usize>> = vec![Vec::new(); num_nodes];
    let mut volume_adj: Vec<Vec<usize>> = vec![Vec::new(); num_nodes];
    let mut max_edge = 0.0_f64;

    for (e_idx, elem) in mesh.elements.iter().enumerate() {
        let idx = elem.node_indices;
        for a in 0..4 {
            let u = idx[a];
            node_to_elems[u].push(e_idx);
            for b in (a + 1)..4 {
                let v = idx[b];
                if !volume_adj[u].contains(&v) {
                    volume_adj[u].push(v);
                }
                if !volume_adj[v].contains(&u) {
                    volume_adj[v].push(u);
                }
                let d = (mesh.nodes[u].pos_vec() - mesh.nodes[v].pos_vec()).norm();
                if d > max_edge {
                    max_edge = d;
                }
            }
        }
    }

    // Helper: compute minimum scaled Jacobian across all elements incident to `nid` when placed at `cand_pos`
    let min_incident_sj = |nodes: &[TetNode], elements: &[TetElement], nid: usize, cand_pos: [f64; 3]| -> f64 {
        let mut worst = f64::INFINITY;
        for &e_idx in &node_to_elems[nid] {
            let idx = elements[e_idx].node_indices;
            let p = [
                if idx[0] == nid { cand_pos } else { nodes[idx[0]].position },
                if idx[1] == nid { cand_pos } else { nodes[idx[1]].position },
                if idx[2] == nid { cand_pos } else { nodes[idx[2]].position },
                if idx[3] == nid { cand_pos } else { nodes[idx[3]].position },
            ];
            let sj = tet_scaled_jacobian(p[0], p[1], p[2], p[3]);
            if sj < worst {
                worst = sj;
            }
        }
        worst
    };

    // Precompute target snapped positions on CAD surface for all boundary nodes
    let boundary_indices: Vec<usize> = (0..num_nodes).filter(|&i| is_boundary[i]).collect();
    let interior_indices: Vec<usize> = (0..num_nodes).filter(|&i| !is_boundary[i]).collect();

    let spatial = SurfaceSpatialIndex::build(cad_surface);

    for pass in 0..opts.outer_passes {
        let frac = ((pass + 1) as f64) / (opts.outer_passes as f64);

        // Step A: Incremental boundary node projection toward exact CAD surface with backtracking
        for &nid in &boundary_indices {
            let cur_pos = mesh.nodes[nid].position;
            let (target_pos, _, _, _) = spatial.project_point(cur_pos);
            let step_target = [
                cur_pos[0] + frac * (target_pos[0] - cur_pos[0]),
                cur_pos[1] + frac * (target_pos[1] - cur_pos[1]),
                cur_pos[2] + frac * (target_pos[2] - cur_pos[2]),
            ];

            let cur_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cur_pos);
            let min_gate = opts.min_allowed_scaled_jacobian.min(cur_sj * 0.85);

            // Backtracking line search along [cur_pos -> step_target]
            let mut alpha = 1.0_f64;
            while alpha >= 0.0625 {
                let cand = [
                    cur_pos[0] + alpha * (step_target[0] - cur_pos[0]),
                    cur_pos[1] + alpha * (step_target[1] - cur_pos[1]),
                    cur_pos[2] + alpha * (step_target[2] - cur_pos[2]),
                ];
                let cand_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cand);
                if cand_sj >= min_gate && cand_sj > 1e-4 {
                    mesh.nodes[nid].position = cand;
                    break;
                }
                alpha *= 0.5;
            }
        }

        // Step B: Tangential surface relaxation for boundary nodes (improves boundary triangle shapes
        // without leaving the exact CAD surface)
        for _ in 0..opts.surface_tangential_sweeps {
            for &nid in &boundary_indices {
                let Some(neighbors) = boundary_adj.get(&nid) else {
                    continue;
                };
                if neighbors.is_empty() {
                    continue;
                }
                let cur_pos = mesh.nodes[nid].position;
                let (_, _, _, best_n) = spatial.project_point(cur_pos);
                let n1 = Vector3::from(best_n);

                let mut avg = Vector3::zeros();
                for &nb in neighbors {
                    avg += mesh.nodes[nb].pos_vec();
                }
                avg /= neighbors.len() as f64;
                let cur_v = Vector3::from(cur_pos);
                let delta = avg - cur_v;
                // Tangential component only
                let tang_delta = delta - n1 * delta.dot(&n1);
                let cand_raw = cur_v + tang_delta * 0.35;
                let (cand_proj, _, _, _) =
                    spatial.project_point([cand_raw.x, cand_raw.y, cand_raw.z]);

                let cur_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cur_pos);
                let cand_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cand_proj);
                if cand_sj >= cur_sj && cand_sj >= opts.min_allowed_scaled_jacobian {
                    mesh.nodes[nid].position = cand_proj;
                }
            }
        }

        // Step C: Quality-guarded interior Laplacian + Smart-Laplacian smoothing
        // Moves interior nodes to the quality-weighted centroid of their 1-ring to absorb boundary deformation
        for _ in 0..opts.smoothing_sweeps {
            for &nid in &interior_indices {
                let neighbors = &volume_adj[nid];
                if neighbors.is_empty() {
                    continue;
                }
                let cur_pos = mesh.nodes[nid].position;
                let cur_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cur_pos);

                let mut avg = [0.0; 3];
                for &nb in neighbors {
                    let p = mesh.nodes[nb].position;
                    avg[0] += p[0];
                    avg[1] += p[1];
                    avg[2] += p[2];
                }
                let inv_n = 1.0 / (neighbors.len() as f64);
                avg[0] *= inv_n;
                avg[1] *= inv_n;
                avg[2] *= inv_n;

                for &omega in &[0.65, 0.35, 0.15] {
                    let cand = [
                        cur_pos[0] + omega * (avg[0] - cur_pos[0]),
                        cur_pos[1] + omega * (avg[1] - cur_pos[1]),
                        cur_pos[2] + omega * (avg[2] - cur_pos[2]),
                    ];
                    let cand_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cand);
                    if cand_sj >= cur_sj.max(opts.min_allowed_scaled_jacobian)
                        || (cand_sj >= 0.25 && cand_sj >= cur_sj * 0.92)
                    {
                        mesh.nodes[nid].position = cand;
                        break;
                    }
                }
            }
        }
    }

    // Final exact boundary snap polish pass for any remaining sub-millimeter residual
    for &nid in &boundary_indices {
        let cur_pos = mesh.nodes[nid].position;
        let (target_pos, dist, _, _) = spatial.project_point(cur_pos);
        if dist > 1e-9 {
            let cand_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, target_pos);
            if cand_sj >= opts.min_allowed_scaled_jacobian {
                mesh.nodes[nid].position = target_pos;
            } else {
                let mut lo = 0.0_f64;
                let mut hi = 1.0_f64;
                let mut best = cur_pos;
                for _ in 0..8 {
                    let mid = 0.5 * (lo + hi);
                    let cand = [
                        cur_pos[0] + mid * (target_pos[0] - cur_pos[0]),
                        cur_pos[1] + mid * (target_pos[1] - cur_pos[1]),
                        cur_pos[2] + mid * (target_pos[2] - cur_pos[2]),
                    ];
                    if min_incident_sj(&mesh.nodes, &mesh.elements, nid, cand)
                        >= opts.min_allowed_scaled_jacobian
                    {
                        best = cand;
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                mesh.nodes[nid].position = best;
            }
        }
    }

    mesh
}

fn check_tet10_element_jacobian(positions: &[[f64; 3]; 10]) -> bool {
    let c = Matrix4::new(
        1.0, positions[0][0], positions[0][1], positions[0][2],
        1.0, positions[1][0], positions[1][1], positions[1][2],
        1.0, positions[2][0], positions[2][1], positions[2][2],
        1.0, positions[3][0], positions[3][1], positions[3][2],
    );
    let corner_vol = c.determinant() / 6.0;
    if corner_vol <= 1e-15 {
        return false;
    }

    for &l in &TET10_GAUSS_POINTS {
        let dn_dl = tet10_barycentric_derivatives(l);
        let mut dn_dxi = [[0.0f64; 3]; 10];
        let mut j_mat = Matrix3::<f64>::zeros();
        for k in 0..10 {
            let d0 = dn_dl[k][0];
            let dxi0 = dn_dl[k][1] - d0;
            let dxi1 = dn_dl[k][2] - d0;
            let dxi2 = dn_dl[k][3] - d0;
            dn_dxi[k] = [dxi0, dxi1, dxi2];
            let px = positions[k][0];
            let py = positions[k][1];
            let pz = positions[k][2];
            j_mat[(0, 0)] += px * dxi0;
            j_mat[(0, 1)] += px * dxi1;
            j_mat[(0, 2)] += px * dxi2;
            j_mat[(1, 0)] += py * dxi0;
            j_mat[(1, 1)] += py * dxi1;
            j_mat[(1, 2)] += py * dxi2;
            j_mat[(2, 0)] += pz * dxi0;
            j_mat[(2, 1)] += pz * dxi1;
            j_mat[(2, 2)] += pz * dxi2;
        }
        let det_j = j_mat.determinant();
        if det_j <= 1e-12 {
            return false;
        }
    }
    true
}

/// Converts a boundary-snapped `TET4` mesh to `TET10` and additionally snaps exterior mid-edge
/// nodes onto the exact curved `cad_surface` (with strict Jacobian safeguard).
pub fn to_tet10_boundary_snapped(
    snapped_tet4: &TetrahedralMesh,
    cad_surface: &TriangleMesh,
    max_snap_dist: f64,
) -> TetrahedralMesh {
    let mut tet10 = snapped_tet4.to_tet10();
    let num_corners = snapped_tet4.nodes.len();

    // Identify exterior 6-node faces and their mid-edge nodes
    let faces6 = tet10.extract_surface_tet10_faces();
    let mut exterior_mids = HashSet::new();
    for (_corners, mids) in &faces6 {
        for &m in mids {
            if m >= num_corners {
                exterior_mids.insert(m);
            }
        }
    }

    // Build mid-node -> incident TET10 elements map
    let mut mid_incident_elems: HashMap<usize, Vec<usize>> = HashMap::new();
    for (elem_id, ten) in tet10.tet10_elements.iter().enumerate() {
        for &m in &ten[4..10] {
            if exterior_mids.contains(&m) {
                mid_incident_elems.entry(m).or_default().push(elem_id);
            }
        }
    }

    // Build mid-node -> parent corner pair map
    let mut mid_to_corners: HashMap<usize, (usize, usize)> = HashMap::new();
    for (elem, ten) in tet10.elements.iter().zip(tet10.tet10_elements.iter()) {
        let corners = elem.node_indices;
        for (e, &(la, lb)) in TET10_EDGE_PAIRS.iter().enumerate() {
            let m_id = ten[4 + e];
            if exterior_mids.contains(&m_id) {
                mid_to_corners.entry(m_id).or_insert((corners[la], corners[lb]));
            }
        }
    }

    let spatial = SurfaceSpatialIndex::build(cad_surface);

    for &m_id in &exterior_mids {
        let cur_pos = tet10.nodes[m_id].position;
        if let Some(&(u, v)) = mid_to_corners.get(&m_id) {
            let pu = tet10.nodes[u].position;
            let pv = tet10.nodes[v].position;
            let edge_len = (Vector3::from(pu) - Vector3::from(pv)).norm();
            let (_, _, _, nu) = spatial.project_point(pu);
            let (_, _, _, nv) = spatial.project_point(pv);
            let cos_n = Vector3::from(nu).dot(&Vector3::from(nv));
            // Only snap mid-edge nodes when both endpoints lie on a smooth curved surface
            // (normals within ~35 deg, not across a sharp 90-deg box/hole edge)
            if cos_n > 0.82 {
                let (proj, dist, _, _) = spatial.project_point(cur_pos);
                if dist <= max_snap_dist && dist <= 0.15 * edge_len {
                    // Test if assigning proj maintains positive Jacobian across all incident elements
                    let mut accepted = false;
                    for alpha in [1.0, 0.75, 0.50, 0.25] {
                        let cand = [
                            cur_pos[0] + alpha * (proj[0] - cur_pos[0]),
                            cur_pos[1] + alpha * (proj[1] - cur_pos[1]),
                            cur_pos[2] + alpha * (proj[2] - cur_pos[2]),
                        ];
                        let mut all_valid = true;
                        if let Some(inc) = mid_incident_elems.get(&m_id) {
                            for &elem_id in inc {
                                let ten = &tet10.tet10_elements[elem_id];
                                let mut pos = [[0.0f64; 3]; 10];
                                for (k, &nid) in ten.iter().enumerate() {
                                    pos[k] = if nid == m_id { cand } else { tet10.nodes[nid].position };
                                }
                                if !check_tet10_element_jacobian(&pos) {
                                    all_valid = false;
                                    break;
                                }
                            }
                        }
                        if all_valid {
                            tet10.nodes[m_id].position = cand;
                            accepted = true;
                            break;
                        }
                    }
                    if !accepted {
                        tet10.nodes[m_id].position = cur_pos;
                    }
                }
            }
        }
    }

    tet10
}

fn benchmark_steel() -> Material {
    Material::new(
        "structural_steel_benchmark",
        205.0,  // E = 205 GPa
        0.30,   // nu = 0.30
        250.0,  // Yield = 250 MPa
        7850.0, // Density = 7850 kg/m³
        "Benchmark Structural Steel (E=205 GPa, nu=0.3)",
    )
}

// =========================================================================
// BODY 1: PERFORATED PLATE WITH CENTRAL CIRCULAR HOLE (from P3-T1)
// Dimensions: 600 x 200 x 10 mm, hole diameter d = 40 mm (r = 20 mm)
// Tensile load: F_x = 100 kN (100,000 N)
// Nominal net-section stress: sigma_nom = 62.50 MPa
// Analytical Peterson Kt = 2.506464 -> sigma_max = 156.654 MPa
// bench/README.md Kt = 2.51904 -> sigma_max = 157.440 MPa
// =========================================================================
fn evaluate_body1_plate_with_hole() {
    println!("====================================================================================================");
    println!("BODY 1: PERFORATED PLATE WITH CENTRAL CIRCULAR HOLE (600 x 200 x 10 mm, D = 40 mm, F_x = 100 kN)");
    println!("Analytical Reference: sigma_nom = 62.50 MPa | Peterson Kt = 2.5065 (156.65 MPa) | bench Kt = 2.5190 (157.44 MPa)");
    println!("====================================================================================================");

    let material = benchmark_steel();
    let length = 0.60;
    let width = 0.20;
    let thickness = 0.01;
    let hole_radius = 0.02;
    let force_x = 100_000.0;
    let sigma_nom_mpa = 62.50;
    let kt_peterson = 2.506464;
    let sigma_peterson_mpa = 156.654;
    let kt_bench = 2.51904;

    // Build high-resolution CAD surface mesh (128 segments on the hole so the CAD surface itself
    // has chord error < 0.015 mm)
    let plate = create_box(length, width, thickness);
    let hole_cyl = create_cylinder(hole_radius, 0.03, 128);
    let perforated = csg_difference(&plate, &hole_cyl);

    // Extract circular hole wall triangles for dedicated curved-boundary deviation measurement
    let mut hole_wall_tris = Vec::new();
    for tri in &perforated.indices {
        let p0 = perforated.vertices[tri[0] as usize].position;
        let p1 = perforated.vertices[tri[1] as usize].position;
        let p2 = perforated.vertices[tri[2] as usize].position;
        let cx = (p0[0] + p1[0] + p2[0]) / 3.0;
        let cy = (p0[1] + p1[1] + p2[1]) / 3.0;
        let r = (cx * cx + cy * cy).sqrt();
        let nz = ((Vector3::from(p1) - Vector3::from(p0))
            .cross(&(Vector3::from(p2) - Vector3::from(p0))))
        .normalize()
        .z
        .abs();
        if (r - hole_radius).abs() < 0.002 && nz < 0.2 {
            hole_wall_tris.push([p0, p1, p2]);
        }
    }

    for &(density, density_label) in &[
        (MeshDensity::Coarse, "Coarse"),
    ] {
        // 1. Baseline unsnapped voxel TET4
        let t0 = Instant::now();
        let voxel_tet4 = TetrahedralMesh::from_mesh(&perforated, density).expect("Voxel mesh failed");
        let dt_voxel_mesh_ms = t0.elapsed().as_secs_f64() * 1000.0;
        let metrics_voxel = evaluate_mesh_metrics(
            &voxel_tet4,
            &perforated,
            Some(&hole_wall_tris),
            0.012,
        );

        let solve_plate = |mesh: &TetrahedralMesh, tol: f64, max_iter: usize| -> Option<(f64, f64, f64)> {
            let ts = Instant::now();
            let mut bc = BoundaryConditions::new();
            bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
                face: AabbFace::MinX,
                tolerance: 1e-4,
                components: DofComponent::X,
            });
            let min_x = mesh.bounding_box().min[0];
            let center_nodes = mesh.find_nodes_by_predicate(|p| {
                (p[0] - min_x).abs() <= 1e-4 && p[1].abs() <= 0.02 && p[2].abs() <= 0.006
            });
            for cn in center_nodes {
                bc.dirichlet.push(DirichletConstraint::FixedNodes {
                    node_ids: vec![cn],
                    components: DofComponent::YZ,
                });
            }
            bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [force_x, 0.0, 0.0]);
            let opts = SolverOptions {
                tolerance: tol,
                max_iterations: max_iter,
            };
            match analyze_mesh("plate_hole", mesh, &material, &bc, Some(opts)) {
                Ok(rep) => Some((
                    rep.max_displacement_m * 1000.0,
                    rep.max_stress_von_mises_pa / 1e6,
                    ts.elapsed().as_secs_f64() * 1000.0,
                )),
                Err(e) => {
                    eprintln!("    [WARN] Plate FEA solve error: {e}");
                    None
                }
            }
        };

        let (disp_vox, stress_vox, dt_solve_vox) = solve_plate(&voxel_tet4, 1e-5, 2000).unwrap_or((0.0, 0.0, 0.0));
        let kt_vox = stress_vox / sigma_nom_mpa;
        let err_kt_vox_peterson = (kt_vox - kt_peterson).abs() / kt_peterson * 100.0;
        let err_kt_vox_bench = (kt_vox - kt_bench).abs() / kt_bench * 100.0;

        // 2. Candidate (a): Boundary-snapped TET4 (`fe_snap`)
        let t_snap = Instant::now();
        let snapped_tet4 =
            snap_voxel_mesh_to_surface(&voxel_tet4, &perforated, SnapOptions::default());
        let dt_snap_ms = t_snap.elapsed().as_secs_f64() * 1000.0;
        let metrics_snap = evaluate_mesh_metrics(
            &snapped_tet4,
            &perforated,
            Some(&hole_wall_tris),
            0.012,
        );
        let (disp_snap, stress_snap, dt_solve_snap) = solve_plate(&snapped_tet4, 1e-5, 2000).unwrap_or((0.0, 0.0, 0.0));
        let kt_snap = stress_snap / sigma_nom_mpa;
        let err_kt_snap_peterson = (kt_snap - kt_peterson).abs() / kt_peterson * 100.0;
        let err_kt_snap_bench = (kt_snap - kt_bench).abs() / kt_bench * 100.0;

        println!(
            "  [{:<6} | Voxel TET4  ] Elems={:>6}, Nodes={:>5} | J_s(min/mean)={:.3}/{:.3} (inv={}) | Hole dev(max/rms)={:.3}/{:.3} mm | disp={:.4} mm | sigma_max={:6.2} MPa, Kt={:.3} (err vs Peterson={:5.2}%, vs bench={:5.2}%) | mesh={:.1}ms, solve={:.1}ms",
            density_label,
            metrics_voxel.element_count,
            metrics_voxel.node_count,
            metrics_voxel.min_scaled_jacobian,
            metrics_voxel.mean_scaled_jacobian,
            metrics_voxel.inverted_elements,
            metrics_voxel.max_curved_feature_deviation_mm,
            metrics_voxel.rms_curved_feature_deviation_mm,
            disp_vox,
            stress_vox,
            kt_vox,
            err_kt_vox_peterson,
            err_kt_vox_bench,
            dt_voxel_mesh_ms,
            dt_solve_vox,
        );
        println!(
            "  [{:<6} | fe_snap TET4] Elems={:>6}, Nodes={:>5} | J_s(min/mean)={:.3}/{:.3} (inv={}) | Hole dev(max/rms)={:.3}/{:.3} mm | disp={:.4} mm | sigma_max={:6.2} MPa, Kt={:.3} (err vs Peterson={:5.2}%, vs bench={:5.2}%) | snap={:.1}ms, solve={:.1}ms",
            density_label,
            metrics_snap.element_count,
            metrics_snap.node_count,
            metrics_snap.min_scaled_jacobian,
            metrics_snap.mean_scaled_jacobian,
            metrics_snap.inverted_elements,
            metrics_snap.max_curved_feature_deviation_mm,
            metrics_snap.rms_curved_feature_deviation_mm,
            disp_snap,
            stress_snap,
            kt_snap,
            err_kt_snap_peterson,
            err_kt_snap_bench,
            dt_snap_ms,
            dt_solve_snap,
        );

        // Also evaluate TET10 boundary snapping to show quadratic boundary adherence with positive Jacobians
        if matches!(density, MeshDensity::Coarse) {
            let t_tet10 = Instant::now();
            let snapped_tet10 = to_tet10_boundary_snapped(&snapped_tet4, &perforated, 0.003);
            let dt_tet10_conv = t_tet10.elapsed().as_secs_f64() * 1000.0;
            println!(
                "  [{:<6} | fe_snap TET10] Elems={:>6}, Nodes={:>5} | Mid-nodes snapped to exact CAD curved surface with det(J) > 0 safeguard | conv={:.1}ms",
                density_label,
                snapped_tet10.element_count(),
                snapped_tet10.node_count(),
                dt_tet10_conv,
            );
        }
    }
    let _ = sigma_peterson_mpa;
    println!();
}

// =========================================================================
// BODY 2: STEPPED SHAFT WITH CIRCUMFERENTIAL GROOVE
// Profile revolved around Z axis (length L = 120 mm, z in [-60, +60] mm):
// - Left section z in [-60, 0] mm: major radius R1 = 20 mm (D1 = 40 mm)
// - Right section z in [0, +60] mm: minor radius R2 = 14 mm (D2 = 28 mm), with a
//   circumferential U-groove at z in [+15, +25] mm of root radius R_g = 11 mm (D_g = 22 mm)
// Cantilever bending + axial load test: fixed at bottom z = -60 mm (MinZ),
// transverse load F_x = 2000 N at top z = +60 mm (MaxZ).
// =========================================================================
fn evaluate_body2_stepped_shaft_with_groove() {
    println!("====================================================================================================");
    println!("BODY 2: STEPPED SHAFT WITH CIRCUMFERENTIAL GROOVE (D1=40mm, D2=28mm, Groove D_g=22mm, L=120mm)");
    println!("====================================================================================================");

    let material = benchmark_steel();
    // Build revolved stepped shaft with groove (in meters)
    let profile = vec![
        [0.000, -0.060],
        [0.020, -0.060],
        [0.020, 0.000],
        [0.014, 0.000],
        [0.014, 0.015],
        [0.011, 0.017],
        [0.011, 0.023],
        [0.014, 0.025],
        [0.014, 0.060],
        [0.000, 0.060],
    ];
    let shaft_mesh = create_revolve(&profile, 360.0, 96);

    // Extract curved lateral/groove triangles (excluding flat end caps at z = -0.06 and z = +0.06)
    let mut curved_tris = Vec::new();
    for tri in &shaft_mesh.indices {
        let p0 = shaft_mesh.vertices[tri[0] as usize].position;
        let p1 = shaft_mesh.vertices[tri[1] as usize].position;
        let p2 = shaft_mesh.vertices[tri[2] as usize].position;
        let cz = (p0[2] + p1[2] + p2[2]) / 3.0;
        if cz.abs() < 0.0595 {
            curved_tris.push([p0, p1, p2]);
        }
    }

    for &(density, density_label) in &[
        (MeshDensity::Coarse, "Coarse"),
    ] {
        let t0 = Instant::now();
        let voxel_tet4 =
            TetrahedralMesh::from_mesh(&shaft_mesh, density).expect("Shaft voxel mesh failed");
        let dt_voxel_ms = t0.elapsed().as_secs_f64() * 1000.0;
        let metrics_voxel =
            evaluate_mesh_metrics(&voxel_tet4, &shaft_mesh, Some(&curved_tris), 0.008);

        let t_snap = Instant::now();
        let snapped_tet4 =
            snap_voxel_mesh_to_surface(&voxel_tet4, &shaft_mesh, SnapOptions::default());
        let dt_snap_ms = t_snap.elapsed().as_secs_f64() * 1000.0;
        let metrics_snap =
            evaluate_mesh_metrics(&snapped_tet4, &shaft_mesh, Some(&curved_tris), 0.008);

        let solve_shaft = |mesh: &TetrahedralMesh| -> Option<(f64, f64, f64)> {
            let ts = Instant::now();
            let mut bc = BoundaryConditions::new();
            bc.fix_aabb_face(AabbFace::MinZ, 1e-3);
            bc.add_aabb_face_force(AabbFace::MaxZ, 1e-3, [2000.0, 0.0, 0.0]);
            let opts = SolverOptions {
                tolerance: 1e-5,
                max_iterations: 2000,
            };
            match analyze_mesh("stepped_shaft", mesh, &material, &bc, Some(opts)) {
                Ok(rep) => Some((
                    rep.max_displacement_m * 1000.0,
                    rep.max_stress_von_mises_pa / 1e6,
                    ts.elapsed().as_secs_f64() * 1000.0,
                )),
                Err(e) => {
                    eprintln!("    [WARN] Shaft FEA solve error: {e}");
                    None
                }
            }
        };

        let (disp_vox, stress_vox, dt_solve_vox) = solve_shaft(&voxel_tet4).unwrap_or((0.0, 0.0, 0.0));
        let (disp_snap, stress_snap, dt_solve_snap) = solve_shaft(&snapped_tet4).unwrap_or((0.0, 0.0, 0.0));

        println!(
            "  [{:<6} | Voxel TET4  ] Elems={:>6}, Nodes={:>5} | J_s(min/mean)={:.3}/{:.3} (inv={}) | Surf dev(max/rms)={:.3}/{:.3} mm | disp={:.4} mm | sigma_max={:6.2} MPa | mesh={:.1}ms, solve={:.1}ms",
            density_label,
            metrics_voxel.element_count,
            metrics_voxel.node_count,
            metrics_voxel.min_scaled_jacobian,
            metrics_voxel.mean_scaled_jacobian,
            metrics_voxel.inverted_elements,
            metrics_voxel.max_curved_feature_deviation_mm,
            metrics_voxel.rms_curved_feature_deviation_mm,
            disp_vox,
            stress_vox,
            dt_voxel_ms,
            dt_solve_vox,
        );
        println!(
            "  [{:<6} | fe_snap TET4] Elems={:>6}, Nodes={:>5} | J_s(min/mean)={:.3}/{:.3} (inv={}) | Surf dev(max/rms)={:.3}/{:.3} mm | disp={:.4} mm | sigma_max={:6.2} MPa | snap={:.1}ms, solve={:.1}ms",
            density_label,
            metrics_snap.element_count,
            metrics_snap.node_count,
            metrics_snap.min_scaled_jacobian,
            metrics_snap.mean_scaled_jacobian,
            metrics_snap.inverted_elements,
            metrics_snap.max_curved_feature_deviation_mm,
            metrics_snap.rms_curved_feature_deviation_mm,
            disp_snap,
            stress_snap,
            dt_snap_ms,
            dt_solve_snap,
        );
    }
    println!();
}

// =========================================================================
// BODY 3: SPUR GEAR TOOTH (Involute Gear Sector, m = 3 mm, z = 20, b = 15 mm)
// Extracts a single tooth + rim sector from `create_spur_gear(m=0.003, z=20, b=0.015, alpha=20 deg)`
// (or full gear rim segment) so that the voxel grid resolves a single involute tooth profile in detail.
// =========================================================================
fn evaluate_body3_gear_tooth() {
    println!("====================================================================================================");
    println!("BODY 3: SPUR GEAR TOOTH (m = 3 mm, z = 20, b = 15 mm, alpha = 20 deg Involute Tooth Sector)");
    println!("====================================================================================================");

    let material = benchmark_steel();
    // Build full involute spur gear (m = 3 mm, z = 20 -> pitch dia d = 60 mm, addendum d_a = 66 mm, root d_f = 52.5 mm)
    let full_gear = create_spur_gear(0.003, 20, 0.015, 20.0, Some(0.015), None, None, None, None);
    let target_gear_mesh = full_gear;
    println!(
        "  Using full involute spur gear (watertight=true, tris={})",
        target_gear_mesh.indices.len()
    );

    // Extract involute flank & root triangles (non-Z-cap triangles)
    let mut flank_tris = Vec::new();
    for tri in &target_gear_mesh.indices {
        let p0 = target_gear_mesh.vertices[tri[0] as usize].position;
        let p1 = target_gear_mesh.vertices[tri[1] as usize].position;
        let p2 = target_gear_mesh.vertices[tri[2] as usize].position;
        let n = (Vector3::from(p1) - Vector3::from(p0))
            .cross(&(Vector3::from(p2) - Vector3::from(p0)));
        if n.norm() > 1e-18 && (n.normalize().z).abs() < 0.5 {
            flank_tris.push([p0, p1, p2]);
        }
    }

    for &(density, density_label) in &[
        (MeshDensity::Coarse, "Coarse"),
    ] {
        let t0 = Instant::now();
        let voxel_tet4 =
            TetrahedralMesh::from_mesh(&target_gear_mesh, density).expect("Gear voxel mesh failed");
        let dt_voxel_ms = t0.elapsed().as_secs_f64() * 1000.0;
        let metrics_voxel = evaluate_mesh_metrics(
            &voxel_tet4,
            &target_gear_mesh,
            Some(&flank_tris),
            0.005,
        );

        let t_snap = Instant::now();
        let snapped_tet4 =
            snap_voxel_mesh_to_surface(&voxel_tet4, &target_gear_mesh, SnapOptions::default());
        let dt_snap_ms = t_snap.elapsed().as_secs_f64() * 1000.0;
        let metrics_snap = evaluate_mesh_metrics(
            &snapped_tet4,
            &target_gear_mesh,
            Some(&flank_tris),
            0.005,
        );

        // Bending load on tooth tip (MaxX) with gear center bore fixed
        let solve_tooth = |mesh: &TetrahedralMesh| -> Option<(f64, f64, f64)> {
            let ts = Instant::now();
            let mut bc = BoundaryConditions::new();
            let bore_nodes = mesh.find_nodes_by_predicate(|p| (p[0]*p[0] + p[1]*p[1]).sqrt() <= 0.0095);
            if !bore_nodes.is_empty() {
                bc.fix_nodes(bore_nodes);
            } else {
                bc.fix_aabb_face(AabbFace::MinX, 2e-3);
            }
            bc.add_aabb_face_force(AabbFace::MaxX, 2e-3, [0.0, 1500.0, 0.0]);
            let opts = SolverOptions {
                tolerance: 1e-4,
                max_iterations: 2500,
            };
            match analyze_mesh("gear_tooth", mesh, &material, &bc, Some(opts)) {
                Ok(rep) => Some((
                    rep.max_displacement_m * 1000.0,
                    rep.max_stress_von_mises_pa / 1e6,
                    ts.elapsed().as_secs_f64() * 1000.0,
                )),
                Err(e) => {
                    eprintln!("    [WARN] Gear tooth FEA solve error: {e}");
                    None
                }
            }
        };

        let (disp_vox, stress_vox, dt_solve_vox) = solve_tooth(&voxel_tet4).unwrap_or((0.0, 0.0, 0.0));
        let (disp_snap, stress_snap, dt_solve_snap) = solve_tooth(&snapped_tet4).unwrap_or((0.0, 0.0, 0.0));

        println!(
            "  [{:<6} | Voxel TET4  ] Elems={:>6}, Nodes={:>5} | J_s(min/mean)={:.3}/{:.3} (inv={}) | Flank dev(max/rms)={:.3}/{:.3} mm | disp={:.4} mm | sigma_max={:6.2} MPa | mesh={:.1}ms, solve={:.1}ms",
            density_label,
            metrics_voxel.element_count,
            metrics_voxel.node_count,
            metrics_voxel.min_scaled_jacobian,
            metrics_voxel.mean_scaled_jacobian,
            metrics_voxel.inverted_elements,
            metrics_voxel.max_curved_feature_deviation_mm,
            metrics_voxel.rms_curved_feature_deviation_mm,
            disp_vox,
            stress_vox,
            dt_voxel_ms,
            dt_solve_vox,
        );
        println!(
            "  [{:<6} | fe_snap TET4] Elems={:>6}, Nodes={:>5} | J_s(min/mean)={:.3}/{:.3} (inv={}) | Flank dev(max/rms)={:.3}/{:.3} mm | disp={:.4} mm | sigma_max={:6.2} MPa | snap={:.1}ms, solve={:.1}ms",
            density_label,
            metrics_snap.element_count,
            metrics_snap.node_count,
            metrics_snap.min_scaled_jacobian,
            metrics_snap.mean_scaled_jacobian,
            metrics_snap.inverted_elements,
            metrics_snap.max_curved_feature_deviation_mm,
            metrics_snap.rms_curved_feature_deviation_mm,
            disp_snap,
            stress_snap,
            dt_snap_ms,
            dt_solve_snap,
        );
    }
    println!();
    let _ = (csg_union, cut_hole, PI);
}

fn main() {
    println!("====================================================================================================");
    println!("P3-T5 SPIKE: BOUNDARY-CONFORMING FEA MESHER PROTOTYPE (`spikes/fe_snap`)");
    println!("====================================================================================================\n");

    evaluate_body1_plate_with_hole();
    evaluate_body2_stepped_shaft_with_groove();
    evaluate_body3_gear_tooth();
}
