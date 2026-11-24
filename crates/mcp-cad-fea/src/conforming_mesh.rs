//! Boundary-conforming tetrahedral meshing (`fe_snap`), 2:1 balanced octree local refinement,
//! and curved quadratic `TET10` boundary projection (`ADR-0002` / `P5-T2`).
//!
//! This module implements a 100% Pure-Rust (`#![deny(unsafe_code)]`) pipeline for accurate
//! engineering FEA on curved CAD geometry:
//! 1. Automatic detection of high-curvature and stress-concentration features (circular holes,
//!    fillets, grooves, gear tooth flanks/roots, and user-specified contact/load zones).
//! 2. Conforming 2:1 balanced octree tetrahedralization with zero hanging nodes (Kuhn 6-tet
//!    split for uniform cells and centroid-fan conforming transition tetrahedra across level jumps).
//! 3. Sharp-feature-aware boundary snapping (`fe_snap`) with a strict scaled Jacobian quality
//!    barrier ($J_s \ge 0.06$, $\det J > 0$) interleaved with tangential surface relaxation and
//!    quality-guarded interior Laplacian smoothing.
//! 4. Curved quadratic `TET10` conversion projecting exterior mid-edge nodes onto curved CAD
//!    faces and curved sharp feature edges while guaranteeing strictly positive $\det J > 0$ at
//!    all 4 Gauss integration points.

use std::collections::{HashMap, HashSet, VecDeque};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use mcp_cad_core::topology::{BoundingBox, FaceKind, TriangleMesh};

use crate::boundary::point_to_triangle_distance_3d;
use crate::element::{
    check_tet10_gauss_jacobians, tet10_min_gauss_jacobian_ratio, TET10_EDGE_PAIRS,
};
use crate::error::FeaError;
use crate::tetra_mesh::{ElementOrder, MeshDensity, TetElement, TetNode, TetrahedralMesh};

/// Engineering accuracy preset controlling element order, boundary conformance, and local refinement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AccuracyMode {
    /// Fast screening mode: unsnapped linear `TET4` voxel mesh, no local octree refinement.
    Fast,
    /// Balanced mode: boundary-conforming snapped `TET4` mesh without local octree subdivision.
    #[default]
    Standard,
    /// High-accuracy engineering mode (`P5-T2` standard): quadratic `TET10` (`element_order = 2`),
    /// 2:1 balanced local refinement in high-curvature/stress-concentration zones, boundary-conforming
    /// snapping (`fe_snap`, $J_s \ge 0.06$), and curved `TET10` mid-edge projection.
    High,
}

impl AccuracyMode {
    /// Parses an accuracy mode string (`"fast"`, `"standard"`, `"high"`), case-insensitively.
    pub fn from_str_ignore_case(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "fast" | "coarse_voxel" | "voxel" => Some(Self::Fast),
            "standard" | "normal" | "medium" | "balanced" => Some(Self::Standard),
            "high" | "accurate" | "engineering" | "conforming" => Some(Self::High),
            _ => None,
        }
    }

    /// Returns the canonical lowercase string label.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Fast => "fast",
            Self::Standard => "standard",
            Self::High => "high",
        }
    }

    /// Returns the default `ElementOrder` associated with this accuracy mode.
    pub fn default_element_order(self) -> ElementOrder {
        match self {
            Self::Fast | Self::Standard => ElementOrder::Linear,
            Self::High => ElementOrder::Quadratic,
        }
    }
}

/// Explicit spherical refinement zone (e.g. around a contact patch, bearing seat, or point load).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RefinementZone {
    /// Center of the refinement zone `[x, y, z]` in meters.
    pub center: [f64; 3],
    /// Radius of influence in meters.
    pub radius: f64,
    /// Target octree subdivision depth inside this zone (`1..=3`).
    pub depth: usize,
}

impl RefinementZone {
    /// Creates a new spherical local refinement zone.
    pub fn new(center: [f64; 3], radius: f64, depth: usize) -> Self {
        Self {
            center,
            radius: radius.max(0.0),
            depth: depth.clamp(1, 3),
        }
    }
}

/// Configuration for boundary-conforming snapping (`fe_snap`) and curved `TET10` mid-edge projection.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SnapOptions {
    /// Minimum allowed corner scaled Jacobian $J_s > 0$ enforced during projection and smoothing.
    /// Default: `0.06` (`P5-T2` quality barrier).
    pub min_allowed_scaled_jacobian: f64,
    /// Number of alternating boundary-projection + interior-smoothing outer passes.
    pub outer_passes: usize,
    /// Number of quality-guarded interior Laplacian smoothing sweeps per outer pass.
    pub smoothing_sweeps: usize,
    /// Number of tangential surface relaxation sweeps for boundary nodes per outer pass.
    pub surface_tangential_sweeps: usize,
    /// Dihedral angle threshold (degrees) for detecting sharp CAD feature creases (e.g. hole rims).
    pub feature_angle_deg: f64,
    /// Whether to lock sharp CAD corners and snap crease nodes onto sharp CAD feature edges.
    pub preserve_sharp_features: bool,
    /// Minimum acceptable ratio $\min_q \det J(\mathbf{L}^{(q)}) / \det J_{\text{corner}}$
    /// when projecting quadratic `TET10` mid-edge nodes onto curved surfaces.
    pub min_tet10_gauss_jacobian_ratio: f64,
}

impl Default for SnapOptions {
    fn default() -> Self {
        Self {
            min_allowed_scaled_jacobian: 0.06,
            outer_passes: 5,
            smoothing_sweeps: 3,
            surface_tangential_sweeps: 2,
            feature_angle_deg: 35.0,
            preserve_sharp_features: true,
            min_tet10_gauss_jacobian_ratio: 0.45,
        }
    }
}

/// Configuration for 2:1 balanced octree local adaptive refinement in high-curvature and
/// stress-concentration regions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LocalRefinementOptions {
    /// Enable octree local refinement around curved features and concentration zones.
    pub enabled: bool,
    /// Maximum octree subdivision depth (`0` = uniform grid, `1..=3` = local grading).
    pub max_depth: usize,
    /// Minimum dihedral angle (degrees) between adjacent triangles to flag curved/fillet surfaces.
    pub curvature_angle_deg: f64,
    /// Distance band (multiple of cell diagonal) around seed triangles within which cells refine.
    pub refine_band_factor: f64,
    /// Optional explicit refinement zones (contact patches, joint anchors, localized loads).
    #[serde(default)]
    pub zones: Vec<RefinementZone>,
    /// Safety cap on total active leaf octree cells to bound memory and solve time.
    pub max_leaf_cells: usize,
}

impl Default for LocalRefinementOptions {
    fn default() -> Self {
        Self {
            enabled: true,
            max_depth: 2,
            curvature_angle_deg: 3.0,
            refine_band_factor: 0.85,
            zones: Vec::new(),
            max_leaf_cells: 6_000,
        }
    }
}

/// Complete configuration for boundary-conforming, locally refined tetrahedral meshing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConformingMeshOptions {
    /// Base mesh density profile.
    pub density: MeshDensity,
    /// Polynomial element order (`Linear` = `TET4`, `Quadratic` = `TET10`).
    pub element_order: ElementOrder,
    /// Whether to apply boundary-conforming snapping (`fe_snap`).
    pub snap_to_boundary: bool,
    /// Snapping and smoothing parameters.
    pub snap: SnapOptions,
    /// Local adaptive octree refinement parameters.
    pub refinement: LocalRefinementOptions,
}

impl ConformingMeshOptions {
    /// Constructs options tuned for the requested `AccuracyMode` and `MeshDensity`.
    pub fn for_accuracy_mode(mode: AccuracyMode, density: MeshDensity) -> Self {
        let max_leaf_cells = match density {
            MeshDensity::Coarse => 3_600,
            MeshDensity::Medium => 4_800,
            MeshDensity::Fine => 9_000,
            MeshDensity::Custom { .. } => 10_000,
        };
        match mode {
            AccuracyMode::Fast => Self {
                density,
                element_order: ElementOrder::Linear,
                snap_to_boundary: false,
                snap: SnapOptions::default(),
                refinement: LocalRefinementOptions {
                    enabled: false,
                    max_depth: 0,
                    max_leaf_cells,
                    ..LocalRefinementOptions::default()
                },
            },
            AccuracyMode::Standard => Self {
                density,
                element_order: ElementOrder::Linear,
                snap_to_boundary: true,
                snap: SnapOptions::default(),
                refinement: LocalRefinementOptions {
                    enabled: false,
                    max_depth: 0,
                    max_leaf_cells,
                    ..LocalRefinementOptions::default()
                },
            },
            AccuracyMode::High => Self {
                density,
                element_order: ElementOrder::Quadratic,
                snap_to_boundary: true,
                snap: SnapOptions::default(),
                refinement: LocalRefinementOptions {
                    enabled: true,
                    max_depth: 2,
                    max_leaf_cells,
                    ..LocalRefinementOptions::default()
                },
            },
        }
    }

    /// Convenience constructor for `AccuracyMode::High` (`TET10` + boundary snapping + local refinement).
    pub fn high_accuracy(density: MeshDensity) -> Self {
        Self::for_accuracy_mode(AccuracyMode::High, density)
    }

    /// Adds an explicit spherical refinement zone (builder pattern).
    pub fn with_refinement_zone(mut self, zone: RefinementZone) -> Self {
        self.refinement.zones.push(zone);
        self
    }
}

/// Geometric quality and boundary-conformity diagnostics for a `TetrahedralMesh`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshQualityMetrics {
    /// Number of tetrahedral elements.
    pub element_count: usize,
    /// Number of mesh nodes.
    pub node_count: usize,
    /// Number of elements with non-positive scaled Jacobian ($J_s \le 0$).
    pub inverted_elements: usize,
    /// Worst (minimum) corner scaled Jacobian $J_s \in [-1, 1]$ across all elements.
    pub min_scaled_jacobian: f64,
    /// Mean corner scaled Jacobian across all elements.
    pub mean_scaled_jacobian: f64,
    /// Minimum Liu-Joe mean-ratio shape quality across all elements.
    pub min_liu_joe: f64,
    /// Mean Liu-Joe mean-ratio shape quality across all elements.
    pub mean_liu_joe: f64,
    /// Maximum Euclidean distance (mm) from exterior mesh nodes to the CAD surface.
    pub max_boundary_deviation_mm: f64,
    /// Root-mean-square Euclidean distance (mm) from exterior mesh nodes to the CAD surface.
    pub rms_boundary_deviation_mm: f64,
    /// Maximum deviation (mm) on the optional curved feature subset (e.g. hole wall, gear flanks).
    pub max_curved_feature_deviation_mm: f64,
    /// RMS deviation (mm) on the optional curved feature subset.
    pub rms_curved_feature_deviation_mm: f64,
    /// True if every element (and all 4 Gauss points of every `TET10` element) has strictly positive Jacobian.
    pub all_tet10_gauss_jacobians_positive: bool,
    /// Minimum Gauss-point Jacobian determinant $\det J$ across all `TET10` elements (`None` for `TET4`).
    pub min_tet10_gauss_det_j: Option<f64>,
    /// True if the mesh was generated with boundary-conforming snapping enabled.
    pub boundary_conforming: bool,
}

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

/// Computes the closest point on 3D line segment `(a, b)` to point `p`, returning
/// `(closest_point, distance, unit_tangent)`.
pub fn closest_point_on_segment_3d(
    p: [f64; 3],
    a: [f64; 3],
    b: [f64; 3],
) -> ([f64; 3], f64, [f64; 3]) {
    let vp = Vector3::from(p);
    let va = Vector3::from(a);
    let vb = Vector3::from(b);
    let ab = vb - va;
    let len_sq = ab.norm_squared();
    if len_sq <= 1e-24 {
        return (a, (vp - va).norm(), [1.0, 0.0, 0.0]);
    }
    let t = ((vp - va).dot(&ab) / len_sq).clamp(0.0, 1.0);
    let q = va + ab * t;
    let tang = ab / len_sq.sqrt();
    ([q.x, q.y, q.z], (vp - q).norm(), [tang.x, tang.y, tang.z])
}

/// Reusable O(1)-reset stamp scratch buffer to eliminate per-query `HashSet` allocations.
#[derive(Debug, Default, Clone)]
pub(crate) struct SpatialQueryScratch {
    marks: Vec<u32>,
    stamp: u32,
}

impl SpatialQueryScratch {
    #[inline]
    pub(crate) fn new(capacity: usize) -> Self {
        Self {
            marks: vec![0; capacity],
            stamp: 0,
        }
    }

    #[inline]
    fn next_stamp(&mut self, len: usize) -> u32 {
        if self.marks.len() < len {
            self.marks.resize(len, 0);
        }
        self.stamp = self.stamp.wrapping_add(1);
        if self.stamp == 0 {
            self.marks.fill(0);
            self.stamp = 1;
        }
        self.stamp
    }

    #[inline]
    fn mark(&mut self, idx: usize, stamp: u32) -> bool {
        if self.marks[idx] == stamp {
            false
        } else {
            self.marks[idx] = stamp;
            true
        }
    }
}

/// Uniform 3D spatial hash index over CAD triangles for fast closest-point projection.
#[derive(Debug, Clone)]
pub struct SurfaceSpatialIndex<'a> {
    surface: &'a TriangleMesh,
    cell_size: f64,
    inv_cell: f64,
    min_bound: [f64; 3],
    grid: HashMap<(i32, i32, i32), Vec<usize>>,
}

impl<'a> SurfaceSpatialIndex<'a> {
    /// Builds a 3D spatial grid covering all triangles of `surface`.
    pub fn build(surface: &'a TriangleMesh) -> Self {
        let all: Vec<usize> = (0..surface.indices.len()).collect();
        Self::build_subset(surface, &all)
    }

    /// Builds a 3D spatial grid covering only the specified triangle indices of `surface`.
    pub fn build_subset(surface: &'a TriangleMesh, tri_indices: &[usize]) -> Self {
        let mut min_b = [f64::INFINITY; 3];
        let mut max_b = [f64::NEG_INFINITY; 3];
        for &idx in tri_indices {
            let tri = &surface.indices[idx];
            for &vi in tri {
                let p = surface.vertices[vi as usize].position;
                for axis in 0..3 {
                    if p[axis] < min_b[axis] {
                        min_b[axis] = p[axis];
                    }
                    if p[axis] > max_b[axis] {
                        max_b[axis] = p[axis];
                    }
                }
            }
        }
        if !min_b[0].is_finite() {
            min_b = [0.0; 3];
            max_b = [0.0; 3];
        }
        let diag = ((max_b[0] - min_b[0]).powi(2)
            + (max_b[1] - min_b[1]).powi(2)
            + (max_b[2] - min_b[2]).powi(2))
        .sqrt();
        let n_tri = tri_indices.len().max(1);
        let cell_size = (diag / (n_tri as f64).cbrt().max(1.0)).max(1e-4);
        let inv_cell = 1.0 / cell_size;

        let mut grid: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::with_capacity(n_tri * 2);
        for &idx in tri_indices {
            let tri = &surface.indices[idx];
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

            // Guard against huge spanning triangles blowing up the hash map
            if (i1 - i0) <= 24 && (j1 - j0) <= 24 && (k1 - k0) <= 24 {
                for i in i0..=i1 {
                    for j in j0..=j1 {
                        for k in k0..=k1 {
                            grid.entry((i, j, k)).or_default().push(idx);
                        }
                    }
                }
            } else {
                // Coarsely clamp huge triangles to centroid + vertex cells; fallback scan handles far queries
                for pt in [
                    a,
                    b,
                    c,
                    [
                        (a[0] + b[0] + c[0]) / 3.0,
                        (a[1] + b[1] + c[1]) / 3.0,
                        (a[2] + b[2] + c[2]) / 3.0,
                    ],
                ] {
                    let ci = ((pt[0] - min_b[0]) * inv_cell).floor() as i32;
                    let cj = ((pt[1] - min_b[1]) * inv_cell).floor() as i32;
                    let ck = ((pt[2] - min_b[2]) * inv_cell).floor() as i32;
                    grid.entry((ci, cj, ck)).or_default().push(idx);
                }
            }
        }

        Self {
            surface,
            cell_size,
            inv_cell,
            min_bound: min_b,
            grid,
        }
    }

    /// Projects point `p` onto the closest point of the indexed CAD surface, returning
    /// `(closest_point, distance, triangle_index, unit_normal)`.
    pub fn project_point(&self, p: [f64; 3]) -> ([f64; 3], f64, usize, [f64; 3]) {
        let mut scratch = SpatialQueryScratch::new(self.surface.indices.len());
        self.project_point_with_scratch(p, &mut scratch)
    }

    /// Zero-allocation closest-point projection using a caller-supplied [`SpatialQueryScratch`].
    pub(crate) fn project_point_with_scratch(
        &self,
        p: [f64; 3],
        scratch: &mut SpatialQueryScratch,
    ) -> ([f64; 3], f64, usize, [f64; 3]) {
        let n_tris = self.surface.indices.len();
        if n_tris == 0 {
            return (p, 0.0, 0, [0.0, 0.0, 1.0]);
        }

        let stamp = scratch.next_stamp(n_tris);
        let ci = ((p[0] - self.min_bound[0]) * self.inv_cell).floor() as i32;
        let cj = ((p[1] - self.min_bound[1]) * self.inv_cell).floor() as i32;
        let ck = ((p[2] - self.min_bound[2]) * self.inv_cell).floor() as i32;

        let mut best_pt = p;
        let mut best_dist = f64::INFINITY;
        let mut best_idx = 0usize;
        let mut best_normal = [0.0, 0.0, 1.0];

        for r in 0_i32..=5_i32 {
            for di in -r..=r {
                for dj in -r..=r {
                    for dk in -r..=r {
                        if di.abs() != r && dj.abs() != r && dk.abs() != r {
                            continue;
                        }
                        if let Some(list) = self.grid.get(&(ci + di, cj + dj, ck + dk)) {
                            for &idx in list {
                                if scratch.mark(idx, stamp) {
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
            if r >= 1 && best_dist < ((r - 1) as f64) * self.cell_size {
                return (best_pt, best_dist, best_idx, best_normal);
            }
        }

        // Exact fallback if query point was far from indexed cells
        if !best_dist.is_finite() || best_dist >= 4.0 * self.cell_size {
            for (idx, tri) in self.surface.indices.iter().enumerate() {
                if scratch.mark(idx, stamp) {
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

        (best_pt, best_dist, best_idx, best_normal)
    }

    /// Computes distance from `p` to the indexed subset of triangles, bounded by `max_search_dist`.
    pub(crate) fn distance_within_with_scratch(
        &self,
        p: [f64; 3],
        max_search_dist: f64,
        scratch: &mut SpatialQueryScratch,
    ) -> f64 {
        if self.grid.is_empty() {
            return f64::INFINITY;
        }
        let stamp = scratch.next_stamp(self.surface.indices.len());
        let r_max = ((max_search_dist * self.inv_cell).ceil() as i32 + 1).clamp(1, 5);
        let ci = ((p[0] - self.min_bound[0]) * self.inv_cell).floor() as i32;
        let cj = ((p[1] - self.min_bound[1]) * self.inv_cell).floor() as i32;
        let ck = ((p[2] - self.min_bound[2]) * self.inv_cell).floor() as i32;

        let mut best_dist = f64::INFINITY;

        for r in 0_i32..=r_max {
            for di in -r..=r {
                for dj in -r..=r {
                    for dk in -r..=r {
                        if di.abs() != r && dj.abs() != r && dk.abs() != r {
                            continue;
                        }
                        if let Some(list) = self.grid.get(&(ci + di, cj + dj, ck + dk)) {
                            for &idx in list {
                                if scratch.mark(idx, stamp) {
                                    let tri = &self.surface.indices[idx];
                                    let a = self.surface.vertices[tri[0] as usize].position;
                                    let b = self.surface.vertices[tri[1] as usize].position;
                                    let c = self.surface.vertices[tri[2] as usize].position;
                                    let d = point_to_triangle_distance_3d(p, a, b, c);
                                    if d < best_dist {
                                        best_dist = d;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if r >= 1 && best_dist < ((r - 1) as f64) * self.cell_size {
                break;
            }
        }
        best_dist
    }
}

/// Spatial index over sharp CAD feature edges (dihedral angle > threshold) and sharp corners.
struct SharpFeatureIndex {
    edges: Vec<([f64; 3], [f64; 3])>,
    edge_face_normals: Vec<(Vector3<f64>, Vector3<f64>)>,
    corners: Vec<[f64; 3]>,
    cell_size: f64,
    inv_cell: f64,
    min_bound: [f64; 3],
    edge_grid: HashMap<(i32, i32, i32), Vec<usize>>,
}

impl SharpFeatureIndex {
    fn build(surface: &TriangleMesh, feature_angle_deg: f64) -> Self {
        let cos_thresh = feature_angle_deg.to_radians().cos();

        // Weld vertices by quantized coordinates (handles split-normal CAD vertices cleanly!)
        let aabb = surface.bounding_box();
        let dims = aabb.dimensions();
        let diag = (dims[0] * dims[0] + dims[1] * dims[1] + dims[2] * dims[2]).sqrt().max(1e-6);
        let weld_eps = (diag * 1e-6).max(1e-9);
        let inv_weld = 1.0 / weld_eps;

        let quant = |p: [f64; 3]| -> (i64, i64, i64) {
            (
                (p[0] * inv_weld).round() as i64,
                (p[1] * inv_weld).round() as i64,
                (p[2] * inv_weld).round() as i64,
            )
        };

        let mut key_to_vid: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut welded_pos: Vec<[f64; 3]> = Vec::new();
        let mut vert_map: Vec<usize> = Vec::with_capacity(surface.vertices.len());
        for v in &surface.vertices {
            let k = quant(v.position);
            let id = *key_to_vid.entry(k).or_insert_with(|| {
                let id = welded_pos.len();
                welded_pos.push(v.position);
                id
            });
            vert_map.push(id);
        }

        // Map welded undirected edge -> list of incident triangle unit normals
        let mut edge_normals: HashMap<(usize, usize), Vec<Vector3<f64>>> = HashMap::new();
        for tri in &surface.indices {
            let a = vert_map[tri[0] as usize];
            let b = vert_map[tri[1] as usize];
            let c = vert_map[tri[2] as usize];
            if a == b || b == c || c == a {
                continue;
            }
            let va = Vector3::from(welded_pos[a]);
            let vb = Vector3::from(welded_pos[b]);
            let vc = Vector3::from(welded_pos[c]);
            let cross = (vb - va).cross(&(vc - va));
            let n_len = cross.norm();
            if n_len <= 1e-18 {
                continue;
            }
            let n = cross / n_len;
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let key = if u < v { (u, v) } else { (v, u) };
                edge_normals.entry(key).or_default().push(n);
            }
        }

        let mut sharp_edge_pairs: Vec<(usize, usize)> = Vec::new();
        let mut edge_face_normals: Vec<(Vector3<f64>, Vector3<f64>)> = Vec::new();
        let mut vert_sharp_dirs: HashMap<usize, Vec<Vector3<f64>>> = HashMap::new();

        for ((u, v), normals) in edge_normals {
            if normals.len() >= 2 {
                let mut sharp_pair: Option<(Vector3<f64>, Vector3<f64>)> = None;
                let mut min_dot = cos_thresh;
                for i in 0..normals.len() {
                    for j in (i + 1)..normals.len() {
                        let d = normals[i].dot(&normals[j]);
                        if d < min_dot {
                            min_dot = d;
                            sharp_pair = Some((normals[i], normals[j]));
                        }
                    }
                }
                if let Some((na, nb)) = sharp_pair {
                    sharp_edge_pairs.push((u, v));
                    edge_face_normals.push((na, nb));
                    let d = Vector3::from(welded_pos[v]) - Vector3::from(welded_pos[u]);
                    let len = d.norm();
                    if len > 1e-12 {
                        let unit_d = d / len;
                        vert_sharp_dirs.entry(u).or_default().push(unit_d);
                        vert_sharp_dirs.entry(v).or_default().push(-unit_d);
                    }
                }
            }
        }

        // A welded vertex is a sharp corner if >= 3 sharp edges meet, OR 2 sharp edges meet at a sharp corner (> 35 deg bend)
        let mut corners = Vec::new();
        for (vid, dirs) in vert_sharp_dirs {
            if dirs.len() >= 3 {
                corners.push(welded_pos[vid]);
            } else if dirs.len() == 2 {
                // Two edges continuing smoothly along a curved rim have d0 . d1 ~= -1.0
                if dirs[0].dot(&dirs[1]) > -cos_thresh {
                    corners.push(welded_pos[vid]);
                }
            }
        }

        let edges: Vec<([f64; 3], [f64; 3])> = sharp_edge_pairs
            .into_iter()
            .map(|(u, v)| (welded_pos[u], welded_pos[v]))
            .collect();

        let n_edges = edges.len().max(1);
        let cell_size = (diag / (n_edges as f64).cbrt().max(1.0)).max(1e-4);
        let inv_cell = 1.0 / cell_size;
        let min_bound = aabb.min;

        let mut edge_grid: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();
        for (e_idx, &(pa, pb)) in edges.iter().enumerate() {
            let i0 = ((pa[0].min(pb[0]) - min_bound[0]) * inv_cell).floor() as i32;
            let i1 = ((pa[0].max(pb[0]) - min_bound[0]) * inv_cell).floor() as i32;
            let j0 = ((pa[1].min(pb[1]) - min_bound[1]) * inv_cell).floor() as i32;
            let j1 = ((pa[1].max(pb[1]) - min_bound[1]) * inv_cell).floor() as i32;
            let k0 = ((pa[2].min(pb[2]) - min_bound[2]) * inv_cell).floor() as i32;
            let k1 = ((pa[2].max(pb[2]) - min_bound[2]) * inv_cell).floor() as i32;
            if (i1 - i0) <= 24 && (j1 - j0) <= 24 && (k1 - k0) <= 24 {
                for i in i0..=i1 {
                    for j in j0..=j1 {
                        for k in k0..=k1 {
                            edge_grid.entry((i, j, k)).or_default().push(e_idx);
                        }
                    }
                }
            } else {
                for pt in [
                    pa,
                    pb,
                    [
                        0.5 * (pa[0] + pb[0]),
                        0.5 * (pa[1] + pb[1]),
                        0.5 * (pa[2] + pb[2]),
                    ],
                ] {
                    let ci = ((pt[0] - min_bound[0]) * inv_cell).floor() as i32;
                    let cj = ((pt[1] - min_bound[1]) * inv_cell).floor() as i32;
                    let ck = ((pt[2] - min_bound[2]) * inv_cell).floor() as i32;
                    edge_grid.entry((ci, cj, ck)).or_default().push(e_idx);
                }
            }
        }

        Self {
            edges,
            edge_face_normals,
            corners,
            cell_size,
            inv_cell,
            min_bound,
            edge_grid,
        }
    }

    fn nearest_corner(&self, p: [f64; 3], max_dist: f64) -> Option<([f64; 3], f64)> {
        let mut best_d_sq = max_dist * max_dist;
        let mut best_pt = None;
        for &c in &self.corners {
            let dx = p[0] - c[0];
            let dy = p[1] - c[1];
            let dz = p[2] - c[2];
            let d_sq = dx * dx + dy * dy + dz * dz;
            if d_sq <= best_d_sq {
                best_d_sq = d_sq;
                best_pt = Some((c, d_sq.sqrt()));
            }
        }
        best_pt
    }

    fn project_to_sharp_edge_with_scratch(
        &self,
        p: [f64; 3],
        max_dist: f64,
        scratch: &mut SpatialQueryScratch,
    ) -> Option<([f64; 3], f64, [f64; 3])> {
        self.project_to_sharp_edge_with_normals_scratch(p, max_dist, scratch)
            .map(|proj| (proj.point, proj.dist, proj.tangent))
    }

    fn project_to_sharp_edge_with_normals_scratch(
        &self,
        p: [f64; 3],
        max_dist: f64,
        scratch: &mut SpatialQueryScratch,
    ) -> Option<SharpEdgeProjection> {
        if self.edges.is_empty() {
            return None;
        }
        let stamp = scratch.next_stamp(self.edges.len());
        let r_max = ((max_dist * self.inv_cell).ceil() as i32 + 1).clamp(1, 4);
        let ci = ((p[0] - self.min_bound[0]) * self.inv_cell).floor() as i32;
        let cj = ((p[1] - self.min_bound[1]) * self.inv_cell).floor() as i32;
        let ck = ((p[2] - self.min_bound[2]) * self.inv_cell).floor() as i32;

        let mut best_pt = p;
        let mut best_dist = max_dist;
        let mut best_tang = [1.0, 0.0, 0.0];
        let mut best_normals = (Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        let mut found = false;

        for r in 0_i32..=r_max {
            for di in -r..=r {
                for dj in -r..=r {
                    for dk in -r..=r {
                        if di.abs() != r && dj.abs() != r && dk.abs() != r {
                            continue;
                        }
                        if let Some(list) = self.edge_grid.get(&(ci + di, cj + dj, ck + dk)) {
                            for &e_idx in list {
                                if scratch.mark(e_idx, stamp) {
                                    let (a, b) = self.edges[e_idx];
                                    let (q, d, t) = closest_point_on_segment_3d(p, a, b);
                                    if d <= best_dist {
                                        best_dist = d;
                                        best_pt = q;
                                        best_tang = t;
                                        best_normals = self.edge_face_normals[e_idx];
                                        found = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if found && r >= 1 && best_dist < ((r - 1) as f64) * self.cell_size {
                break;
            }
        }

        if found {
            Some(SharpEdgeProjection {
                point: best_pt,
                dist: best_dist,
                tangent: best_tang,
                normal_a: best_normals.0,
                normal_b: best_normals.1,
            })
        } else {
            None
        }
    }
}

struct SharpEdgeProjection {
    point: [f64; 3],
    dist: f64,
    tangent: [f64; 3],
    normal_a: Vector3<f64>,
    normal_b: Vector3<f64>,
}

/// Fast 2D `(Y, Z)` grid-accelerated ray-casting inside/outside tester for closed triangle meshes.
pub(crate) struct FastRayCaster<'a> {
    mesh: &'a TriangleMesh,
    ray_dir: Vector3<f64>,
    min_y: f64,
    min_z: f64,
    inv_dy: f64,
    inv_dz: f64,
    ny: usize,
    nz: usize,
    cells: Vec<Vec<usize>>,
}

impl<'a> FastRayCaster<'a> {
    pub(crate) fn build(mesh: &'a TriangleMesh) -> Self {
        let aabb = mesh.bounding_box();
        let dims = aabb.dimensions();
        let n_tri = mesh.indices.len().max(1);
        let grid_res = ((n_tri as f64).sqrt().round() as usize).clamp(8, 48);
        let ny = grid_res;
        let nz = grid_res;
        let pad_y = dims[1].max(1e-6) * 1e-3;
        let pad_z = dims[2].max(1e-6) * 1e-3;
        let min_y = aabb.min[1] - pad_y;
        let min_z = aabb.min[2] - pad_z;
        let span_y = (dims[1] + 2.0 * pad_y).max(1e-6);
        let span_z = (dims[2] + 2.0 * pad_z).max(1e-6);
        let inv_dy = ny as f64 / span_y;
        let inv_dz = nz as f64 / span_z;

        // Slightly tilted ray along +X so it avoids exact coplanar/edge hits while staying tight in Y,Z
        let ray_dir: Vector3<f64> = Vector3::new(1.0_f64, 1.131_73e-5_f64, 1.719_23e-5_f64).normalize();
        let max_x_span = dims[0].max(1e-6);
        let drift_y = ray_dir.y.abs() * max_x_span * 1.1;
        let drift_z = ray_dir.z.abs() * max_x_span * 1.1;

        let mut cells = vec![Vec::new(); ny * nz];
        for (tri_idx, tri) in mesh.indices.iter().enumerate() {
            let p0 = mesh.vertices[tri[0] as usize].position;
            let p1 = mesh.vertices[tri[1] as usize].position;
            let p2 = mesh.vertices[tri[2] as usize].position;

            let y0 = p0[1].min(p1[1]).min(p2[1]) - drift_y;
            let y1 = p0[1].max(p1[1]).max(p2[1]) + drift_y;
            let z0 = p0[2].min(p1[2]).min(p2[2]) - drift_z;
            let z1 = p0[2].max(p1[2]).max(p2[2]) + drift_z;

            let iy0 = (((y0 - min_y) * inv_dy).floor() as isize).clamp(0, (ny - 1) as isize) as usize;
            let iy1 = (((y1 - min_y) * inv_dy).floor() as isize).clamp(0, (ny - 1) as isize) as usize;
            let iz0 = (((z0 - min_z) * inv_dz).floor() as isize).clamp(0, (nz - 1) as isize) as usize;
            let iz1 = (((z1 - min_z) * inv_dz).floor() as isize).clamp(0, (nz - 1) as isize) as usize;

            for iy in iy0..=iy1 {
                for iz in iz0..=iz1 {
                    cells[iy * nz + iz].push(tri_idx);
                }
            }
        }

        Self {
            mesh,
            ray_dir,
            min_y,
            min_z,
            inv_dy,
            inv_dz,
            ny,
            nz,
            cells,
        }
    }

    pub(crate) fn is_inside(&self, point: [f64; 3]) -> bool {
        let iy = ((point[1] - self.min_y) * self.inv_dy).floor() as isize;
        let iz = ((point[2] - self.min_z) * self.inv_dz).floor() as isize;
        if iy < 0 || iy >= self.ny as isize || iz < 0 || iz >= self.nz as isize {
            return false;
        }
        let bucket = &self.cells[(iy as usize) * self.nz + (iz as usize)];
        let origin = Vector3::from(point);
        let mut hits = 0usize;
        for &tri_idx in bucket {
            let tri = &self.mesh.indices[tri_idx];
            let v0 = self.mesh.vertices[tri[0] as usize].pos_vec();
            let v1 = self.mesh.vertices[tri[1] as usize].pos_vec();
            let v2 = self.mesh.vertices[tri[2] as usize].pos_vec();
            if ray_triangle_hit(origin, self.ray_dir, v0, v1, v2) {
                hits += 1;
            }
        }
        hits % 2 == 1
    }
}

/// Detects whether `mesh` is an additive thin helical wire/spring solid (as opposed to a
/// helical groove/thread cut into a solid shaft) and returns its wire cross-section diameter `2 * r_wire`.
pub(crate) fn detect_helical_wire_diameter(mesh: &TriangleMesh) -> Option<f64> {
    if mesh.faces.is_empty() {
        return None;
    }
    let mut has_additive_helix = false;
    let mut helix_radius_max = 0.0_f64;
    let mut cap_wire_radius: Option<f64> = None;

    for f in &mesh.faces {
        let role_lc = f.role.to_ascii_lowercase();
        let name_lc = f.name.to_ascii_lowercase();
        if role_lc.contains("groove")
            || role_lc.contains("thread")
            || role_lc.contains("notch")
            || name_lc.contains("groove")
            || name_lc.contains("thread")
        {
            return None;
        }
        if f.aliases.iter().any(|a| {
            let al = a.to_ascii_lowercase();
            al.contains("groove") || al.contains("thread") || al.contains("flute")
        }) {
            return None;
        }
        if f.kind == FaceKind::Helical {
            has_additive_helix = true;
            if let Some(hr) = f.radius {
                if hr.is_finite() && hr > 0.0 {
                    helix_radius_max = helix_radius_max.max(hr);
                }
            }
        } else if f.kind == FaceKind::Plane
            && (name_lc.contains("cap")
                || role_lc.contains("cap")
                || f.aliases.iter().any(|a| a.contains("cap")))
        {
            if let Some(r) = f.radius {
                if r.is_finite() && r > 1e-9 {
                    cap_wire_radius = Some(match cap_wire_radius {
                        Some(prev) => prev.min(r),
                        None => r,
                    });
                }
            }
        }
    }

    if !has_additive_helix {
        return None;
    }

    if let Some(wr) = cap_wire_radius {
        if helix_radius_max <= 0.0 || wr < helix_radius_max {
            return Some(2.0 * wr);
        }
    }

    // Fallback: estimate wire radius from cap face triangles if descriptor radius is absent
    let cap_ids: Vec<u32> = mesh
        .faces
        .iter()
        .filter(|f| f.kind == FaceKind::Plane)
        .map(|f| f.id)
        .collect();
    if !cap_ids.is_empty() && mesh.face_ids.len() == mesh.indices.len() {
        let cap_id = cap_ids[0];
        let mut pts = Vec::new();
        for (ti, &fid) in mesh.face_ids.iter().enumerate() {
            if fid == cap_id {
                let tri = &mesh.indices[ti];
                for &vi in tri {
                    pts.push(mesh.vertices[vi as usize].position);
                }
            }
        }
        if !pts.is_empty() {
            let mut c = [0.0; 3];
            for &p in &pts {
                c[0] += p[0];
                c[1] += p[1];
                c[2] += p[2];
            }
            let inv = 1.0 / (pts.len() as f64);
            c[0] *= inv;
            c[1] *= inv;
            c[2] *= inv;
            let mut max_r = 0.0_f64;
            for &p in &pts {
                let r = ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2) + (p[2] - c[2]).powi(2))
                    .sqrt();
                max_r = max_r.max(r);
            }
            if max_r > 1e-7 {
                return Some(2.0 * max_r);
            }
        }
    }

    None
}

#[inline]
fn ray_triangle_hit(
    ray_origin: Vector3<f64>,
    ray_dir: Vector3<f64>,
    v0: Vector3<f64>,
    v1: Vector3<f64>,
    v2: Vector3<f64>,
) -> bool {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let pvec = ray_dir.cross(&edge2);
    let det = edge1.dot(&pvec);
    if det.abs() < 1e-12 {
        return false;
    }
    let inv_det = 1.0 / det;
    let tvec = ray_origin - v0;
    let u = tvec.dot(&pvec) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let qvec = tvec.cross(&edge1);
    let v = ray_dir.dot(&qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    let t = edge2.dot(&qvec) * inv_det;
    t > 1e-9
}

/// Computes the signed $6 V$ determinant `(v1 - v0) · ((v2 - v0) × (v3 - v0))` of a tetrahedron.
#[inline]
pub fn tet_det_6v(p0: [f64; 3], p1: [f64; 3], p2: [f64; 3], p3: [f64; 3]) -> f64 {
    let v0 = Vector3::from(p0);
    let v1 = Vector3::from(p1);
    let v2 = Vector3::from(p2);
    let v3 = Vector3::from(p3);
    (v1 - v0).dot(&(v2 - v0).cross(&(v3 - v0)))
}

/// Computes the Knupp normalized corner scaled Jacobian $J_s \in [-1.0, 1.0]$ for tetrahedron `(p0..p3)`.
///
/// A regular equilateral tetrahedron has $J_s = 1.0$; a standard orthogonal Kuhn cube-split
/// tetrahedron has $J_s = 1/\sqrt{3} \approx 0.5774$; an inverted tetrahedron has $J_s < 0$.
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

/// Evaluates element quality, `TET10` Gauss-point Jacobians, and boundary surface deviation
/// of `tet_mesh` against the exact CAD `TriangleMesh` and an optional curved feature subset.
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
    if !min_sj.is_finite() {
        min_sj = 0.0;
        min_lj = 0.0;
    }

    let (all_tet10_pos, min_tet10_det) = if tet_mesh.is_tet10() {
        let mut all_ok = true;
        let mut min_det = f64::INFINITY;
        for ten in &tet_mesh.tet10_elements {
            let mut pos = [[0.0f64; 3]; 10];
            for (k, &nid) in ten.iter().enumerate() {
                pos[k] = tet_mesh.nodes[nid].position;
            }
            match check_tet10_gauss_jacobians(&pos) {
                Ok(det_j) => {
                    if det_j < min_det {
                        min_det = det_j;
                    }
                }
                Err(_) => {
                    all_ok = false;
                    min_det = min_det.min(0.0);
                }
            }
        }
        (all_ok && inverted == 0, Some(if min_det.is_finite() { min_det } else { 0.0 }))
    } else {
        (inverted == 0, None)
    };

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
    let mut query_scratch = SpatialQueryScratch::new(cad_mesh.indices.len());
    for &nid in &boundary_nodes {
        let p = tet_mesh.nodes[nid].position;
        let (_, d_cad, _, _) = spatial.project_point_with_scratch(p, &mut query_scratch);
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
        all_tet10_gauss_jacobians_positive: all_tet10_pos,
        min_tet10_gauss_det_j: min_tet10_det,
        boundary_conforming: tet_mesh.boundary_conforming,
    }
}

/// Identifies CAD triangle indices belonging to high-curvature or stress-concentration zones:
/// curved faces (cylinders, holes, fillets, cones, tori, gears), concave grooves/roots, etc.
/// Returns `(high_priority_seed_tris, general_curved_tris)`.
fn detect_refinement_seed_triangles(
    mesh: &TriangleMesh,
    curvature_angle_deg: f64,
) -> (Vec<usize>, Vec<usize>) {
    let mut high_priority = HashSet::new();
    let mut general_curved = HashSet::new();
    let is_helical_wire = detect_helical_wire_diameter(mesh).is_some();

    // 1. Named / typed CAD face metadata
    if mesh.face_ids.len() == mesh.indices.len() && !mesh.faces.is_empty() {
        let mut face_map: HashMap<u32, (FaceKind, bool)> = HashMap::new();
        for f in &mesh.faces {
            let role_lc = f.role.to_ascii_lowercase();
            let name_lc = f.name.to_ascii_lowercase();
            let is_helical_groove = f.kind == FaceKind::Helical
                && (role_lc.contains("groove")
                    || role_lc.contains("thread")
                    || role_lc.contains("notch")
                    || name_lc.contains("groove")
                    || name_lc.contains("thread")
                    || f.aliases.iter().any(|a| {
                        let al = a.to_ascii_lowercase();
                        al.contains("groove") || al.contains("thread") || al.contains("flute")
                    }));
            let is_concentrator = role_lc.contains("hole")
                || role_lc.contains("fillet")
                || role_lc.contains("groove")
                || role_lc.contains("tooth")
                || role_lc.contains("root")
                || role_lc.contains("bore")
                || role_lc.contains("notch")
                || role_lc.contains("slot")
                || name_lc.contains("hole")
                || name_lc.contains("fillet")
                || name_lc.contains("groove")
                || name_lc.contains("root")
                || f.kind == FaceKind::Torus
                || is_helical_groove;
            face_map.insert(f.id, (f.kind, is_concentrator));
        }

        for (tri_idx, &fid) in mesh.face_ids.iter().enumerate() {
            if let Some(&(kind, is_concentrator)) = face_map.get(&fid) {
                if is_concentrator {
                    high_priority.insert(tri_idx);
                    general_curved.insert(tri_idx);
                } else if kind != FaceKind::Plane {
                    general_curved.insert(tri_idx);
                }
            }
        }
    }

    // 2. Geometric dihedral curvature & concave edge detector (works on untagged / gear / STL meshes too)
    let aabb = mesh.bounding_box();
    let dims = aabb.dimensions();
    let diag = (dims[0] * dims[0] + dims[1] * dims[1] + dims[2] * dims[2]).sqrt().max(1e-6);
    let inv_weld = 1.0 / (diag * 1e-6).max(1e-9);
    let quant = |p: [f64; 3]| -> (i64, i64, i64) {
        (
            (p[0] * inv_weld).round() as i64,
            (p[1] * inv_weld).round() as i64,
            (p[2] * inv_weld).round() as i64,
        )
    };

    let mut key_to_vid: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut vert_map = Vec::with_capacity(mesh.vertices.len());
    for v in &mesh.vertices {
        let next_id = key_to_vid.len();
        let id = *key_to_vid.entry(quant(v.position)).or_insert(next_id);
        vert_map.push(id);
    }

    let mut tri_normals = Vec::with_capacity(mesh.indices.len());
    let mut tri_centroids = Vec::with_capacity(mesh.indices.len());
    let mut edge_to_tris: HashMap<(usize, usize), Vec<usize>> = HashMap::new();

    for (t_idx, tri) in mesh.indices.iter().enumerate() {
        let p0 = Vector3::from(mesh.vertices[tri[0] as usize].position);
        let p1 = Vector3::from(mesh.vertices[tri[1] as usize].position);
        let p2 = Vector3::from(mesh.vertices[tri[2] as usize].position);
        let cross = (p1 - p0).cross(&(p2 - p0));
        let n = if cross.norm() > 1e-18 {
            cross.normalize()
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        };
        tri_normals.push(n);
        tri_centroids.push((p0 + p1 + p2) / 3.0);

        let a = vert_map[tri[0] as usize];
        let b = vert_map[tri[1] as usize];
        let c = vert_map[tri[2] as usize];
        for (u, v) in [(a, b), (b, c), (c, a)] {
            if u != v {
                let key = if u < v { (u, v) } else { (v, u) };
                edge_to_tris.entry(key).or_default().push(t_idx);
            }
        }
    }

    let cos_curved_min = curvature_angle_deg.to_radians().cos();
    let cos_sharp_max = 75.0_f64.to_radians().cos();
    let cos_concave = 8.0_f64.to_radians().cos();

    for tris in edge_to_tris.values() {
        if tris.len() == 2 {
            let t0 = tris[0];
            let t1 = tris[1];
            let n0 = tri_normals[t0];
            let n1 = tri_normals[t1];
            let dot = n0.dot(&n1).clamp(-1.0, 1.0);

            // Smooth curved surface (cylinder, fillet, involute gear flank)
            if dot < cos_curved_min && dot > cos_sharp_max {
                general_curved.insert(t0);
                general_curved.insert(t1);
            }
            // Concave feature (internal hole wall, fillet, gear tooth root, groove)
            if dot < cos_concave {
                let c_diff = tri_centroids[t1] - tri_centroids[t0];
                if c_diff.dot(&n0) > 1e-9 * diag {
                    if !is_helical_wire {
                        high_priority.insert(t0);
                        high_priority.insert(t1);
                    }
                    general_curved.insert(t0);
                    general_curved.insert(t1);
                }
            }
        }
    }

    let mut hp_vec: Vec<usize> = high_priority.into_iter().collect();
    let mut gc_vec: Vec<usize> = general_curved.into_iter().collect();
    hp_vec.sort_unstable();
    gc_vec.sort_unstable();
    (hp_vec, gc_vec)
}

/// Integer-indexed octree cell in `[0..nx0*scale] x [0..ny0*scale] x [0..nz0*scale]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct OctreeCellKey {
    /// Grid index `(ix, iy, iz)` at `level`.
    ix: usize,
    iy: usize,
    iz: usize,
    /// Subdivision depth `0..=max_depth`.
    level: usize,
}

type IntCoord = (usize, usize, usize);

/// Generates a 100% conforming (zero-hanging-node) 2:1 balanced octree `TET4` mesh
/// with automatic local refinement near high-curvature/stress-concentration features
/// and explicit `RefinementZone`s.
pub(crate) fn build_octree_conforming_tet4(
    surface: &TriangleMesh,
    density: MeshDensity,
    refinement: &LocalRefinementOptions,
) -> Result<TetrahedralMesh, FeaError> {
    if surface.vertices.is_empty() || surface.indices.is_empty() {
        return Err(FeaError::EmptyMesh);
    }

    // Pre-flight watertight/manifold validation (same safety contract as `TetrahedralMesh::from_mesh`)
    let validation = mcp_cad_core::validate_mesh(surface, None);
    let is_manifold = validation.topology.non_manifold_edges_count == 0
        && validation.topology.non_manifold_vertices_count == 0;
    if !validation.topology.is_watertight
        || !is_manifold
        || !validation.topology.orientation_consistent
        || !validation.geometry.is_positive_volume
    {
        let message = if !validation.issues.is_empty() {
            validation.issues.join("; ")
        } else {
            format!(
                "Surface mesh failed pre-flight topology check (watertight={}, manifold={}, orientation_consistent={}, positive_volume={})",
                validation.topology.is_watertight,
                is_manifold,
                validation.topology.orientation_consistent,
                validation.geometry.is_positive_volume
            )
        };
        return Err(FeaError::InvalidSurfaceMesh {
            message,
            boundary_edges_count: validation.topology.boundary_edges_count,
            non_manifold_edges_count: validation.topology.non_manifold_edges_count,
            non_manifold_vertices_count: validation.topology.non_manifold_vertices_count,
            inconsistent_edges_count: validation.topology.inconsistent_edges_count,
            is_watertight: validation.topology.is_watertight,
        });
    }

    let aabb = surface.bounding_box();
    let dims = aabb.dimensions();
    if dims[0] <= 1e-9 || dims[1] <= 1e-9 || dims[2] <= 1e-9 {
        return Err(FeaError::MeshGenerationFailed(
            "Bounding box has zero or degenerate dimensions".to_string(),
        ));
    }

    let helical_wire_diam = detect_helical_wire_diameter(surface);
    let max_depth = if !refinement.enabled {
        0
    } else if helical_wire_diam.is_some() {
        refinement.max_depth.min(1)
    } else {
        refinement.max_depth.min(3)
    };

    // Detect curved & stress-concentrator features
    let (hp_seed_tris, general_curved_tris) = if max_depth > 0 {
        detect_refinement_seed_triangles(surface, refinement.curvature_angle_deg)
    } else {
        (Vec::new(), Vec::new())
    };

    let has_refinement_targets =
        !hp_seed_tris.is_empty() || !general_curved_tris.is_empty() || !refinement.zones.is_empty();

    // If no refinement is active or there are no curved features / zones, fall back cleanly to `from_mesh`
    if max_depth == 0 || !has_refinement_targets {
        return TetrahedralMesh::from_mesh(surface, density);
    }

    // Choose base coarse grid (nx0, ny0, nz0) using volume-based universal sizing so both
    // thin plates (600x200x10) and compact 3D solids (Lamé cylinder, spur gear) get well-proportioned root cells.
    let vol_aabb = (dims[0] * dims[1] * dims[2]).max(1e-18);
    let max_dim = dims[0].max(dims[1]).max(dims[2]);
    let min_dim = dims[0].min(dims[1]).min(dims[2]);
    let target_root_cells: f64 = match density {
        MeshDensity::Coarse => 120.0,
        MeshDensity::Medium => 240.0,
        MeshDensity::Fine => 450.0,
        MeshDensity::Custom { max_element_size } => {
            let h0 = (max_element_size * (1usize << max_depth.min(2)) as f64).max(1e-5);
            (vol_aabb / (h0 * h0 * h0)).clamp(32.0, 1200.0)
        }
    };
    let h0 = if let Some(d_wire) = helical_wire_diam {
        let leaf_factor = match density {
            MeshDensity::Coarse => 0.72,
            MeshDensity::Medium => 0.50,
            MeshDensity::Fine => 0.36,
            MeshDensity::Custom { max_element_size } if max_element_size > 0.0 => {
                (max_element_size / d_wire).clamp(0.25, 0.70)
            }
            MeshDensity::Custom { .. } => 0.50,
        };
        let scale_lvl = (1usize << max_depth.min(1)) as f64;
        (d_wire * leaf_factor * scale_lvl).max(1e-5)
    } else {
        match density {
            MeshDensity::Custom { max_element_size } if max_element_size > 0.0 => {
                (max_element_size * (1usize << max_depth.min(2)) as f64).max(1e-5)
            }
            _ => (vol_aabb / target_root_cells).cbrt().max(1e-5),
        }
    };

    let max_root_axis = if helical_wire_diam.is_some() { 42 } else { 30 };
    let min_count = dims
        .iter()
        .filter(|&&d| (d - min_dim).abs() <= 0.15 * min_dim)
        .count();
    let axis_cells = |d: f64| -> usize {
        let is_sole_thin_axis =
            min_count == 1 && (d - min_dim).abs() <= 1e-9 && d < 0.12 * max_dim;
        let raw = (d / h0).round() as usize;
        if is_sole_thin_axis {
            // Thin plate thickness: 1 root cell -> 2 cells at level 1, 4 cells at level 2 right at hole!
            raw.clamp(1, 3)
        } else {
            raw.clamp(4, max_root_axis)
        }
    };

    let nx0 = axis_cells(dims[0]);
    let ny0 = axis_cells(dims[1]);
    let nz0 = axis_cells(dims[2]);
    let dx0 = dims[0] / nx0 as f64;
    let dy0 = dims[1] / ny0 as f64;
    let dz0 = dims[2] / nz0 as f64;

    let hp_index = if !hp_seed_tris.is_empty() {
        Some(SurfaceSpatialIndex::build_subset(surface, &hp_seed_tris))
    } else {
        None
    };
    let gc_index = if !general_curved_tris.is_empty() {
        Some(SurfaceSpatialIndex::build_subset(
            surface,
            &general_curved_tris,
        ))
    } else {
        None
    };

    let ray_caster = FastRayCaster::build(surface);

    // Build initial root leaf set
    let mut leaves: HashSet<OctreeCellKey> = HashSet::with_capacity(nx0 * ny0 * nz0 * 4);
    for ix in 0..nx0 {
        for iy in 0..ny0 {
            for iz in 0..nz0 {
                leaves.insert(OctreeCellKey {
                    ix,
                    iy,
                    iz,
                    level: 0,
                });
            }
        }
    }

    let cell_center_and_diag = |cell: OctreeCellKey| -> ([f64; 3], f64) {
        let inv_scale = 1.0 / ((1usize << cell.level) as f64);
        let dx = dx0 * inv_scale;
        let dy = dy0 * inv_scale;
        let dz = dz0 * inv_scale;
        let cx = aabb.min[0] + (cell.ix as f64 + 0.5) * dx;
        let cy = aabb.min[1] + (cell.iy as f64 + 0.5) * dy;
        let cz = aabb.min[2] + (cell.iz as f64 + 0.5) * dz;
        let diag = (dx * dx + dy * dy + dz * dz).sqrt();
        ([cx, cy, cz], diag)
    };

    let cell_active_inside = |cell: OctreeCellKey| -> bool {
        let inv_scale = 1.0 / ((1usize << cell.level) as f64);
        let dx = dx0 * inv_scale;
        let dy = dy0 * inv_scale;
        let dz = dz0 * inv_scale;
        let cx = aabb.min[0] + (cell.ix as f64 + 0.5) * dx;
        let cy = aabb.min[1] + (cell.iy as f64 + 0.5) * dy;
        let cz = aabb.min[2] + (cell.iz as f64 + 0.5) * dz;
        if ray_caster.is_inside([cx, cy, cz]) {
            return true;
        }
        if helical_wire_diam.is_some() {
            let mut inside_faces = 0usize;
            for p in [
                [cx - 0.44 * dx, cy, cz],
                [cx + 0.44 * dx, cy, cz],
                [cx, cy - 0.44 * dy, cz],
                [cx, cy + 0.44 * dy, cz],
                [cx, cy, cz - 0.44 * dz],
                [cx, cy, cz + 0.44 * dz],
            ] {
                if ray_caster.is_inside(p) {
                    inside_faces += 1;
                    if inside_faces >= 2 {
                        return true;
                    }
                }
            }
        }
        false
    };

    // Helper: check if cell overlaps the CAD solid (center inside, OR near surface)
    let full_surface_index = SurfaceSpatialIndex::build(surface);
    let mut surf_scratch = SpatialQueryScratch::new(surface.indices.len());
    let mut cell_overlaps_solid = |cell: OctreeCellKey| -> bool {
        let (c, diag) = cell_center_and_diag(cell);
        if ray_caster.is_inside(c) {
            return true;
        }
        let d_surf = full_surface_index.distance_within_with_scratch(
            c,
            0.55 * diag,
            &mut surf_scratch,
        );
        d_surf <= 0.55 * diag
    };

    let mut hp_scratch = SpatialQueryScratch::new(surface.indices.len());
    let mut gc_scratch = SpatialQueryScratch::new(surface.indices.len());

    // Level-by-level adaptive subdivision around high-curvature & concentration zones
    for level in 0..max_depth {
        let mut to_split: Vec<(f64, OctreeCellKey)> = Vec::new();
        for &cell in &leaves {
            if cell.level != level {
                continue;
            }
            if !cell_overlaps_solid(cell) {
                continue;
            }

            let (center, diag) = cell_center_and_diag(cell);
            let mut best_priority = f64::INFINITY;

            // Check explicit spherical refinement zones
            for z in &refinement.zones {
                if level < z.depth {
                    let dist = ((center[0] - z.center[0]).powi(2)
                        + (center[1] - z.center[1]).powi(2)
                        + (center[2] - z.center[2]).powi(2))
                    .sqrt();
                    let limit = z.radius + 0.5 * diag;
                    if dist <= limit {
                        best_priority = best_priority.min(dist / limit.max(1e-9));
                    }
                }
            }

            // Check high-priority concentrator triangles (holes, fillets, tooth roots, grooves)
            if let Some(ref idx) = hp_index {
                let band_factor = if level == 0 {
                    refinement.refine_band_factor
                } else {
                    (0.65 * refinement.refine_band_factor).max(0.75)
                };
                let band = band_factor * diag;
                let d = idx.distance_within_with_scratch(center, band, &mut hp_scratch);
                if d <= band {
                    best_priority = best_priority.min(d / band.max(1e-9));
                }
            }

            // Check general curved triangles (refine to level 1; depth 2 is reserved for high-priority concentrators and explicit zones)
            if level == 0 {
                if let Some(ref idx) = gc_index {
                    let band = refinement.refine_band_factor * diag;
                    let d = idx.distance_within_with_scratch(center, band, &mut gc_scratch);
                    if d <= band {
                        best_priority = best_priority.min(0.5 + d / band.max(1e-9));
                    }
                }
            }

            if best_priority.is_finite() {
                to_split.push((best_priority, cell));
            }
        }

        // Respect leaf cell budget, splitting cells closest to concentrators first (spatially symmetric)
        to_split.sort_by(|a, b| {
            a.0.total_cmp(&b.0)
                .then_with(|| (a.1.ix, a.1.iy, a.1.iz).cmp(&(b.1.ix, b.1.iy, b.1.iz)))
        });
        for (_, cell) in to_split {
            if leaves.len() + 7 > refinement.max_leaf_cells {
                break;
            }
            if leaves.remove(&cell) {
                for ox in 0..2 {
                    for oy in 0..2 {
                        for oz in 0..2 {
                            leaves.insert(OctreeCellKey {
                                ix: cell.ix * 2 + ox,
                                iy: cell.iy * 2 + oy,
                                iz: cell.iz * 2 + oz,
                                level: cell.level + 1,
                            });
                        }
                    }
                }
            }
        }
    }

    // Enforce strong 2:1 balance across all 26 face/edge/vertex neighbors
    enforce_strong_2_to_1_balance(&mut leaves, nx0, ny0, nz0, max_depth);

    // Filter leaves to active cells lying inside the closed CAD solid
    let mut active_leaves: Vec<OctreeCellKey> = leaves
        .into_iter()
        .filter(|&cell| cell_active_inside(cell))
        .collect();

    if active_leaves.is_empty() {
        return TetrahedralMesh::from_mesh(surface, density);
    }

    active_leaves.sort_unstable_by_key(|c| (c.level, c.ix, c.iy, c.iz));

    // Convert active 2:1 balanced octree leaves to a 100% conforming TET4 mesh
    Ok(triangulate_balanced_octree(
        &active_leaves,
        &aabb,
        nx0,
        ny0,
        nz0,
        max_depth,
    ))
}

/// Enforces strong 2:1 balance (all 26 face/edge/vertex neighbors differ by at most 1 level).
fn enforce_strong_2_to_1_balance(
    leaves: &mut HashSet<OctreeCellKey>,
    nx0: usize,
    ny0: usize,
    nz0: usize,
    max_depth: usize,
) {
    if max_depth <= 1 {
        return;
    }

    let mut queue: VecDeque<OctreeCellKey> = leaves
        .iter()
        .copied()
        .filter(|c| c.level >= 2)
        .collect();

    while let Some(cell) = queue.pop_front() {
        if cell.level < 2 || !leaves.contains(&cell) {
            continue;
        }
        // Any neighbor at level <= cell.level - 2 must be split!
        // Compute parent (level - 1) and inspect its 26 neighbors at level - 2 (and shallower).
        let p_ix = cell.ix / 2;
        let p_iy = cell.iy / 2;
        let p_iz = cell.iz / 2;
        let p_level = cell.level - 1;
        let max_px = (nx0 << p_level) as isize;
        let max_py = (ny0 << p_level) as isize;
        let max_pz = (nz0 << p_level) as isize;

        for dx in -1_isize..=1 {
            let nx = p_ix as isize + dx;
            if nx < 0 || nx >= max_px {
                continue;
            }
            for dy in -1_isize..=1 {
                let ny = p_iy as isize + dy;
                if ny < 0 || ny >= max_py {
                    continue;
                }
                for dz in -1_isize..=1 {
                    let nz = p_iz as isize + dz;
                    if nz < 0 || nz >= max_pz {
                        continue;
                    }
                    // Walk ancestors of (nx, ny, nz) at levels < p_level to see if an unsplit coarse leaf covers it
                    for anc_level in 0..p_level {
                        let shift = p_level - anc_level;
                        let anc = OctreeCellKey {
                            ix: (nx as usize) >> shift,
                            iy: (ny as usize) >> shift,
                            iz: (nz as usize) >> shift,
                            level: anc_level,
                        };
                        if leaves.remove(&anc) {
                            for ox in 0..2 {
                                for oy in 0..2 {
                                    for oz in 0..2 {
                                        let child = OctreeCellKey {
                                            ix: anc.ix * 2 + ox,
                                            iy: anc.iy * 2 + oy,
                                            iz: anc.iz * 2 + oz,
                                            level: anc.level + 1,
                                        };
                                        leaves.insert(child);
                                        if child.level >= 2 {
                                            queue.push_back(child);
                                        }
                                    }
                                }
                            }
                            // Re-check this same (nx, ny, nz) in case multiple levels of ancestors existed
                            queue.push_back(cell);
                            break;
                        }
                    }
                }
            }
        }
    }
}

#[inline]
fn midpoint_int(a: IntCoord, b: IntCoord) -> IntCoord {
    ((a.0 + b.0) / 2, (a.1 + b.1) / 2, (a.2 + b.2) / 2)
}

/// Converts a set of active 2:1-balanced octree leaf cells into a strictly conforming `TET4` mesh
/// with zero hanging nodes and positively oriented tetrahedra.
fn triangulate_balanced_octree(
    active_leaves: &[OctreeCellKey],
    aabb: &BoundingBox,
    nx0: usize,
    ny0: usize,
    nz0: usize,
    max_depth: usize,
) -> TetrahedralMesh {
    // Finest integer resolution is 2 * (1 << max_depth) so every leaf cell's center AND every
    // face/edge midpoint of the finest level still has exact integer coordinates!
    let finest_unit = 2usize << max_depth;
    let dims = aabb.dimensions();
    let step_x = dims[0] / ((nx0 * finest_unit) as f64);
    let step_y = dims[1] / ((ny0 * finest_unit) as f64);
    let step_z = dims[2] / ((nz0 * finest_unit) as f64);

    let int_to_pos = |c: IntCoord| -> [f64; 3] {
        [
            aabb.min[0] + (c.0 as f64) * step_x,
            aabb.min[1] + (c.1 as f64) * step_y,
            aabb.min[2] + (c.2 as f64) * step_z,
        ]
    };

    // Phase 1: Collect all corner vertices of all active leaf cells
    let mut active_verts: HashSet<IntCoord> = HashSet::with_capacity(active_leaves.len() * 4);
    for &cell in active_leaves {
        let span = finest_unit >> cell.level;
        let x0 = cell.ix * span;
        let y0 = cell.iy * span;
        let z0 = cell.iz * span;
        for ox in [0, span] {
            for oy in [0, span] {
                for oz in [0, span] {
                    active_verts.insert((x0 + ox, y0 + oy, z0 + oz));
                }
            }
        }
    }

    // Phase 2: Closure pass — if any face of any active leaf cell has its face-center in `active_verts`,
    // ensure all 4 edge midpoints of that face are also in `active_verts` so partially-clipped
    // octree boundaries remain 100% conforming across all adjacent cells.
    let cell_outward_faces = |cell: OctreeCellKey| -> [[IntCoord; 4]; 6] {
        let span = finest_unit >> cell.level;
        let x0 = cell.ix * span;
        let y0 = cell.iy * span;
        let z0 = cell.iz * span;
        let x1 = x0 + span;
        let y1 = y0 + span;
        let z1 = z0 + span;

        let p000 = (x0, y0, z0);
        let p100 = (x1, y0, z0);
        let p010 = (x0, y1, z0);
        let p110 = (x1, y1, z0);
        let p001 = (x0, y0, z1);
        let p101 = (x1, y0, z1);
        let p011 = (x0, y1, z1);
        let p111 = (x1, y1, z1);

        // Counter-clockwise when viewed from outside the cell
        [
            [p000, p010, p110, p100], // -Z
            [p001, p101, p111, p011], // +Z
            [p000, p100, p101, p001], // -Y
            [p010, p011, p111, p110], // +Y
            [p000, p001, p011, p010], // -X
            [p100, p110, p111, p101], // +X
        ]
    };

    for _ in 0..3 {
        let mut added = false;
        for &cell in active_leaves {
            for face in cell_outward_faces(cell) {
                let vc = midpoint_int(face[0], face[2]);
                if active_verts.contains(&vc) {
                    for e in 0..4 {
                        let m = midpoint_int(face[e], face[(e + 1) % 4]);
                        if active_verts.insert(m) {
                            added = true;
                        }
                    }
                }
            }
        }
        if !added {
            break;
        }
    }

    // Phase 3: Deterministic conforming triangulation of each cell
    let mut node_map: HashMap<IntCoord, usize> = HashMap::with_capacity(active_verts.len() * 2);
    let mut nodes: Vec<TetNode> = Vec::with_capacity(active_verts.len() * 2);
    let mut elements: Vec<TetElement> = Vec::with_capacity(active_leaves.len() * 8);

    let get_or_insert_node =
        |c: IntCoord, node_map: &mut HashMap<IntCoord, usize>, nodes: &mut Vec<TetNode>| -> usize {
            *node_map.entry(c).or_insert_with(|| {
                let id = nodes.len();
                nodes.push(TetNode::new(id, int_to_pos(c)));
                id
            })
        };

    // Recursive deterministic quad triangulator: depends ONLY on vertex integer coordinates and
    // the immutable `active_verts` set, guaranteeing identical triangles on shared faces regardless
    // of face winding or cell subdivision level.
    fn triangulate_quad(
        q: [IntCoord; 4],
        active_verts: &HashSet<IntCoord>,
        node_map: &mut HashMap<IntCoord, usize>,
        nodes: &mut Vec<TetNode>,
        int_to_pos: &impl Fn(IntCoord) -> [f64; 3],
        out_tris: &mut Vec<[usize; 3]>,
    ) -> bool {
        let get_node = |c: IntCoord, nm: &mut HashMap<IntCoord, usize>, ns: &mut Vec<TetNode>| {
            *nm.entry(c).or_insert_with(|| {
                let id = ns.len();
                ns.push(TetNode::new(id, int_to_pos(c)));
                id
            })
        };

        let vc = midpoint_int(q[0], q[2]);
        let mids = [
            midpoint_int(q[0], q[1]),
            midpoint_int(q[1], q[2]),
            midpoint_int(q[2], q[3]),
            midpoint_int(q[3], q[0]),
        ];

        // Case A: Face is refined into 4 sub-quads (neighbor cell is 1 level finer)
        if active_verts.contains(&vc) {
            let sub_quads = [
                [q[0], mids[0], vc, mids[3]],
                [mids[0], q[1], mids[1], vc],
                [vc, mids[1], q[2], mids[2]],
                [mids[3], vc, mids[2], q[3]],
            ];
            for sq in sub_quads {
                triangulate_quad(sq, active_verts, node_map, nodes, int_to_pos, out_tris);
            }
            return true;
        }

        let has_mid = [
            active_verts.contains(&mids[0]),
            active_verts.contains(&mids[1]),
            active_verts.contains(&mids[2]),
            active_verts.contains(&mids[3]),
        ];
        let split_count = has_mid.iter().filter(|&&b| b).count();

        let emit_plain_quad = |quad: [IntCoord; 4],
                               nm: &mut HashMap<IntCoord, usize>,
                               ns: &mut Vec<TetNode>,
                               tris: &mut Vec<[usize; 3]>| {
            // Canonical diagonal from lexicographically smallest corner -> matches Kuhn 6-tet split!
            let mut min_k = 0usize;
            for k in 1..4 {
                if quad[k] < quad[min_k] {
                    min_k = k;
                }
            }
            let a = get_node(quad[min_k], nm, ns);
            let b = get_node(quad[(min_k + 1) % 4], nm, ns);
            let c = get_node(quad[(min_k + 2) % 4], nm, ns);
            let d = get_node(quad[(min_k + 3) % 4], nm, ns);
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        };

        match split_count {
            0 => {
                emit_plain_quad(q, node_map, nodes, out_tris);
                false
            }
            1 => {
                // Rotate so the unique split edge is edge 0: (r0, r1) with midpoint m0
                let rot = has_mid.iter().position(|&b| b).unwrap();
                let r = [
                    q[rot],
                    q[(rot + 1) % 4],
                    q[(rot + 2) % 4],
                    q[(rot + 3) % 4],
                ];
                let m0 = get_node(mids[rot], node_map, nodes);
                let v0 = get_node(r[0], node_map, nodes);
                let v1 = get_node(r[1], node_map, nodes);
                let v2 = get_node(r[2], node_map, nodes);
                let v3 = get_node(r[3], node_map, nodes);
                out_tris.push([v0, m0, v3]);
                out_tris.push([m0, v1, v2]);
                out_tris.push([m0, v2, v3]);
                true
            }
            2 => {
                if has_mid[0] && has_mid[2] {
                    // Opposite edges 0 and 2 split -> 2 sub-quads sharing (m0, m2)
                    emit_plain_quad([q[0], mids[0], mids[2], q[3]], node_map, nodes, out_tris);
                    emit_plain_quad([mids[0], q[1], q[2], mids[2]], node_map, nodes, out_tris);
                } else if has_mid[1] && has_mid[3] {
                    // Opposite edges 1 and 3 split -> 2 sub-quads sharing (m1, m3)
                    emit_plain_quad([q[0], q[1], mids[1], mids[3]], node_map, nodes, out_tris);
                    emit_plain_quad([mids[3], mids[1], q[2], q[3]], node_map, nodes, out_tris);
                } else {
                    // Adjacent edges split: find rot such that edges rot and (rot+1)%4 are split
                    let rot = (0..4)
                        .find(|&i| has_mid[i] && has_mid[(i + 1) % 4])
                        .unwrap();
                    let r = [
                        q[rot],
                        q[(rot + 1) % 4],
                        q[(rot + 2) % 4],
                        q[(rot + 3) % 4],
                    ];
                    let ma = get_node(mids[rot], node_map, nodes);
                    let mb = get_node(mids[(rot + 1) % 4], node_map, nodes);
                    let v0 = get_node(r[0], node_map, nodes);
                    let v1 = get_node(r[1], node_map, nodes);
                    let v2 = get_node(r[2], node_map, nodes);
                    let v3 = get_node(r[3], node_map, nodes);
                    out_tris.push([v0, ma, v3]);
                    out_tris.push([ma, v1, mb]);
                    out_tris.push([mb, v2, v3]);
                    out_tris.push([v3, ma, mb]);
                }
                true
            }
            _ => {
                // 3 or 4 edges split: fan around exact canonical face center `vc`
                let c_id = get_node(vc, node_map, nodes);
                let mut ring: Vec<usize> = Vec::with_capacity(8);
                for e in 0..4 {
                    ring.push(get_node(q[e], node_map, nodes));
                    if has_mid[e] {
                        ring.push(get_node(mids[e], node_map, nodes));
                    }
                }
                let n_ring = ring.len();
                for i in 0..n_ring {
                    out_tris.push([c_id, ring[i], ring[(i + 1) % n_ring]]);
                }
                true
            }
        }
    }

    let mut face_tris = Vec::with_capacity(48);

    for &cell in active_leaves {
        face_tris.clear();
        let faces = cell_outward_faces(cell);
        let mut any_split = false;
        for f in faces {
            if triangulate_quad(
                f,
                &active_verts,
                &mut node_map,
                &mut nodes,
                &int_to_pos,
                &mut face_tris,
            ) {
                any_split = true;
            }
        }

        if !any_split {
            // Regular uniform cell: use Kuhn 6-tetrahedron decomposition (exact face diagonal match!)
            let span = finest_unit >> cell.level;
            let x0 = cell.ix * span;
            let y0 = cell.iy * span;
            let z0 = cell.iz * span;
            let x1 = x0 + span;
            let y1 = y0 + span;
            let z1 = z0 + span;

            let p000 = get_or_insert_node((x0, y0, z0), &mut node_map, &mut nodes);
            let p100 = get_or_insert_node((x1, y0, z0), &mut node_map, &mut nodes);
            let p010 = get_or_insert_node((x0, y1, z0), &mut node_map, &mut nodes);
            let p110 = get_or_insert_node((x1, y1, z0), &mut node_map, &mut nodes);
            let p001 = get_or_insert_node((x0, y0, z1), &mut node_map, &mut nodes);
            let p101 = get_or_insert_node((x1, y0, z1), &mut node_map, &mut nodes);
            let p011 = get_or_insert_node((x0, y1, z1), &mut node_map, &mut nodes);
            let p111 = get_or_insert_node((x1, y1, z1), &mut node_map, &mut nodes);

            let cell_tets = [
                [p000, p100, p110, p111],
                [p000, p100, p101, p111],
                [p000, p010, p110, p111],
                [p000, p010, p011, p111],
                [p000, p001, p101, p111],
                [p000, p001, p011, p111],
            ];
            for mut tet in cell_tets {
                let det = tet_det_6v(
                    nodes[tet[0]].position,
                    nodes[tet[1]].position,
                    nodes[tet[2]].position,
                    nodes[tet[3]].position,
                );
                if det < 0.0 {
                    tet.swap(1, 2);
                }
                let id = elements.len();
                elements.push(TetElement::new(id, tet));
            }
        } else {
            // Conforming transition cell: insert cell center and fan to all conforming face triangles
            let span = finest_unit >> cell.level;
            let center_int = (
                cell.ix * span + span / 2,
                cell.iy * span + span / 2,
                cell.iz * span + span / 2,
            );
            let c_id = get_or_insert_node(center_int, &mut node_map, &mut nodes);
            for &[a, b, c] in &face_tris {
                let mut tet = [c_id, a, b, c];
                let det = tet_det_6v(
                    nodes[tet[0]].position,
                    nodes[tet[1]].position,
                    nodes[tet[2]].position,
                    nodes[tet[3]].position,
                );
                if det < 0.0 {
                    tet.swap(1, 2);
                }
                let id = elements.len();
                elements.push(TetElement::new(id, tet));
            }
        }
    }

    TetrahedralMesh::new(nodes, elements)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BoundaryNodeClass {
    Interior,
    SmoothSurface,
    SharpCrease,
    SharpCorner,
}

/// Projects exterior boundary vertices of a linear `TET4` `TetrahedralMesh` onto the exact
/// triangles and sharp feature edges of `cad_surface` (`fe_snap`, `ADR-0002` / `P5-T2`).
///
/// Enforces a strict scaled Jacobian quality barrier ($J_s \ge \text{min\_allowed\_scaled\_jacobian}$,
/// $\det J > 0$) via backtracking line search, interleaved with tangential surface relaxation
/// and quality-guarded interior Laplacian smoothing.
pub fn snap_mesh_to_surface(
    base_mesh: &TetrahedralMesh,
    cad_surface: &TriangleMesh,
    opts: SnapOptions,
) -> TetrahedralMesh {
    let mut mesh = TetrahedralMesh::new(base_mesh.nodes.clone(), base_mesh.elements.clone());
    let num_nodes = mesh.nodes.len();
    if num_nodes == 0 || mesh.elements.is_empty() || cad_surface.indices.is_empty() {
        return mesh;
    }

    // 1. Identify exterior boundary triangles, boundary adjacency, and per-node surface normals
    let surf_tris = mesh.extract_surface_triangles();
    let mut is_boundary = vec![false; num_nodes];
    let mut boundary_adj: HashMap<usize, Vec<usize>> = HashMap::new();
    let mut node_surf_normals: HashMap<usize, Vec<Vector3<f64>>> = HashMap::new();
    let mut node_min_edge: Vec<f64> = vec![f64::INFINITY; num_nodes];

    for &[a, b, c] in &surf_tris {
        is_boundary[a] = true;
        is_boundary[b] = true;
        is_boundary[c] = true;

        let va = mesh.nodes[a].pos_vec();
        let vb = mesh.nodes[b].pos_vec();
        let vc = mesh.nodes[c].pos_vec();
        let cross = (vb - va).cross(&(vc - va));
        if cross.norm() > 1e-18 {
            let un = cross.normalize();
            node_surf_normals.entry(a).or_default().push(un);
            node_surf_normals.entry(b).or_default().push(un);
            node_surf_normals.entry(c).or_default().push(un);
        }

        for (u, v) in [(a, b), (b, c), (c, a)] {
            let list = boundary_adj.entry(u).or_default();
            if !list.contains(&v) {
                list.push(v);
            }
            let list_v = boundary_adj.entry(v).or_default();
            if !list_v.contains(&u) {
                list_v.push(u);
            }
            let len = (mesh.nodes[u].pos_vec() - mesh.nodes[v].pos_vec()).norm();
            if len > 1e-12 {
                node_min_edge[u] = node_min_edge[u].min(len);
                node_min_edge[v] = node_min_edge[v].min(len);
            }
        }
    }

    // 2. Build node -> incident elements and node -> 1-ring volume neighbors
    let mut node_to_elems: Vec<Vec<usize>> = vec![Vec::new(); num_nodes];
    let mut volume_adj: Vec<Vec<usize>> = vec![Vec::new(); num_nodes];

    for (e_idx, elem) in mesh.elements.iter().enumerate() {
        let idx = elem.node_indices;
        for a in 0..4 {
            let u = idx[a];
            node_to_elems[u].push(e_idx);
            for &v in &idx[(a + 1)..4] {
                if !volume_adj[u].contains(&v) {
                    volume_adj[u].push(v);
                }
                if !volume_adj[v].contains(&u) {
                    volume_adj[v].push(u);
                }
            }
        }
    }

    let min_incident_sj =
        |nodes: &[TetNode], elements: &[TetElement], nid: usize, cand_pos: [f64; 3]| -> f64 {
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

    let boundary_indices: Vec<usize> = (0..num_nodes).filter(|&i| is_boundary[i]).collect();
    let interior_indices: Vec<usize> = (0..num_nodes).filter(|&i| !is_boundary[i]).collect();

    let spatial = SurfaceSpatialIndex::build(cad_surface);
    let sharp_index = if opts.preserve_sharp_features {
        Some(SharpFeatureIndex::build(cad_surface, opts.feature_angle_deg))
    } else {
        None
    };

    let n_cad_tris = cad_surface.indices.len();
    let n_sharp_edges = sharp_index.as_ref().map_or(0, |s| s.edges.len());
    let mut tri_scratch = SpatialQueryScratch::new(n_cad_tris);
    let mut edge_scratch = SpatialQueryScratch::new(n_sharp_edges);

    // 3. Classify boundary nodes (sharp corner / sharp crease / smooth surface)
    let mut node_class = vec![BoundaryNodeClass::Interior; num_nodes];
    let mut corner_targets: HashMap<usize, [f64; 3]> = HashMap::new();
    let cos_feat = opts.feature_angle_deg.to_radians().cos();

    for &nid in &boundary_indices {
        let p = mesh.nodes[nid].position;
        let h_local = if node_min_edge[nid].is_finite() {
            node_min_edge[nid]
        } else {
            1e-3
        };

        if let Some(ref sidx) = sharp_index {
            // Check sharp corner first
            if let Some((c_pt, _)) = sidx.nearest_corner(p, 0.55 * h_local) {
                // Verify the mesh node itself sits near a multi-normal corner or very close to the CAD corner
                let d_corner = (Vector3::from(p) - Vector3::from(c_pt)).norm();
                if d_corner <= 0.35 * h_local {
                    node_class[nid] = BoundaryNodeClass::SharpCorner;
                    corner_targets.insert(nid, c_pt);
                    continue;
                }
            }

            // Check sharp crease: incident mesh boundary normals must align with BOTH CAD face normals
            // meeting at the sharp CAD edge (prevents interior cylinder/hole staircase corners from
            // being misclassified as cap-rim creases).
            if let Some(edge_proj) =
                sidx.project_to_sharp_edge_with_normals_scratch(p, 0.75 * h_local, &mut edge_scratch)
            {
                let n_a = edge_proj.normal_a;
                let n_b = edge_proj.normal_b;
                let matches_both_cad_faces = node_surf_normals.get(&nid).is_some_and(|normals| {
                    let has_a = normals.iter().any(|n| n.dot(&n_a).abs() > 0.45);
                    let has_b = normals.iter().any(|n| n.dot(&n_b).abs() > 0.45);
                    let has_angle = normals.iter().enumerate().any(|(i, ni)| {
                        normals[(i + 1)..]
                            .iter()
                            .any(|nj| ni.dot(nj).abs() < cos_feat)
                    });
                    has_a && has_b && has_angle
                });
                if matches_both_cad_faces {
                    node_class[nid] = BoundaryNodeClass::SharpCrease;
                    continue;
                }
            }
        }

        node_class[nid] = BoundaryNodeClass::SmoothSurface;
    }

    let target_for_node = |nid: usize,
                           cur_pos: [f64; 3],
                           ts: &mut SpatialQueryScratch,
                           es: &mut SpatialQueryScratch|
     -> ([f64; 3], [f64; 3]) {
        match node_class[nid] {
            BoundaryNodeClass::SharpCorner => {
                let c = corner_targets.get(&nid).copied().unwrap_or(cur_pos);
                (c, [0.0, 0.0, 1.0])
            }
            BoundaryNodeClass::SharpCrease => {
                let h_local = if node_min_edge[nid].is_finite() {
                    node_min_edge[nid]
                } else {
                    1e-3
                };
                if let Some(ref sidx) = sharp_index {
                    if let Some((pt, _, tang)) =
                        sidx.project_to_sharp_edge_with_scratch(cur_pos, 1.50 * h_local, es)
                    {
                        return (pt, tang);
                    }
                }
                let (pt, _, _, n) = spatial.project_point_with_scratch(cur_pos, ts);
                (pt, n)
            }
            _ => {
                let (pt, _, _, n) = spatial.project_point_with_scratch(cur_pos, ts);
                (pt, n)
            }
        }
    };

    let min_sj_barrier = opts.min_allowed_scaled_jacobian.max(1e-3);
    let total_passes = opts.outer_passes.max(1);

    // 4. Alternating boundary projection + tangential relaxation + interior Laplacian smoothing
    for pass in 0..total_passes {
        let frac = ((pass + 1) as f64) / (total_passes as f64);

        // Step A: Incremental projection toward exact CAD surface / sharp crease with backtracking.
        // Strictly preserves `Js >= min_sj_barrier` for all elements starting above the barrier,
        // and never worsens any element that started below the barrier.
        for &nid in &boundary_indices {
            let cur_pos = mesh.nodes[nid].position;
            let (target_pos, _) =
                target_for_node(nid, cur_pos, &mut tri_scratch, &mut edge_scratch);
            let step_target = [
                cur_pos[0] + frac * (target_pos[0] - cur_pos[0]),
                cur_pos[1] + frac * (target_pos[1] - cur_pos[1]),
                cur_pos[2] + frac * (target_pos[2] - cur_pos[2]),
            ];

            let cur_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cur_pos);
            let min_gate = if cur_sj >= min_sj_barrier {
                min_sj_barrier
            } else {
                cur_sj
            };

            let mut alpha = 1.0_f64;
            while alpha >= 0.03125 {
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

        // Step B: Tangential surface relaxation (corners stay locked; creases slide only along crease tangent)
        for _ in 0..opts.surface_tangential_sweeps {
            for &nid in &boundary_indices {
                let cls = node_class[nid];
                if cls == BoundaryNodeClass::SharpCorner {
                    continue;
                }
                let Some(neighbors) = boundary_adj.get(&nid) else {
                    continue;
                };
                if neighbors.is_empty() {
                    continue;
                }
                let cur_pos = mesh.nodes[nid].position;
                let cur_v = Vector3::from(cur_pos);

                let cand_proj = if cls == BoundaryNodeClass::SharpCrease {
                    // Only relax along crease neighbors to preserve sharp edges & circular rims
                    let crease_nbs: Vec<usize> = neighbors
                        .iter()
                        .copied()
                        .filter(|&nb| {
                            matches!(
                                node_class[nb],
                                BoundaryNodeClass::SharpCrease | BoundaryNodeClass::SharpCorner
                            )
                        })
                        .collect();
                    if crease_nbs.is_empty() {
                        continue;
                    }
                    let mut avg = Vector3::zeros();
                    for &nb in &crease_nbs {
                        avg += mesh.nodes[nb].pos_vec();
                    }
                    avg /= crease_nbs.len() as f64;
                    let (_, tang) =
                        target_for_node(nid, cur_pos, &mut tri_scratch, &mut edge_scratch);
                    let t_vec = Vector3::from(tang);
                    let delta = avg - cur_v;
                    let tang_delta = t_vec * delta.dot(&t_vec);
                    let cand_raw = cur_v + tang_delta * 0.30;
                    let (proj, _) = target_for_node(
                        nid,
                        [cand_raw.x, cand_raw.y, cand_raw.z],
                        &mut tri_scratch,
                        &mut edge_scratch,
                    );
                    proj
                } else {
                    let (_, _, _, best_n) =
                        spatial.project_point_with_scratch(cur_pos, &mut tri_scratch);
                    let n1 = Vector3::from(best_n);
                    let mut avg = Vector3::zeros();
                    for &nb in neighbors {
                        avg += mesh.nodes[nb].pos_vec();
                    }
                    avg /= neighbors.len() as f64;
                    let delta = avg - cur_v;
                    let tang_delta = delta - n1 * delta.dot(&n1);
                    let cand_raw = cur_v + tang_delta * 0.35;
                    let (proj, _, _, _) = spatial.project_point_with_scratch(
                        [cand_raw.x, cand_raw.y, cand_raw.z],
                        &mut tri_scratch,
                    );
                    proj
                };

                let cur_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cur_pos);
                let cand_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cand_proj);
                if (cur_sj < min_sj_barrier && cand_sj > cur_sj)
                    || (cand_sj >= min_sj_barrier && cand_sj >= (cur_sj * 0.95).min(0.25))
                {
                    mesh.nodes[nid].position = cand_proj;
                }
            }
        }

        // Step C: Quality-guarded interior Laplacian smoothing
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
                    if (cur_sj < min_sj_barrier && cand_sj > cur_sj)
                        || (cand_sj >= min_sj_barrier && cand_sj >= (cur_sj * 0.85).min(0.20))
                    {
                        mesh.nodes[nid].position = cand;
                        break;
                    }
                }
            }
        }
    }

    // 5. Final multi-cycle interior relaxation + exact boundary polish with bisection line search
    for polish_cycle in 0..3 {
        for &nid in &boundary_indices {
            let cur_pos = mesh.nodes[nid].position;
            let (target_pos, _) =
                target_for_node(nid, cur_pos, &mut tri_scratch, &mut edge_scratch);
            let dist = (Vector3::from(target_pos) - Vector3::from(cur_pos)).norm();
            if dist > 1e-9 {
                let cur_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cur_pos);
                let min_gate = if cur_sj >= min_sj_barrier {
                    min_sj_barrier
                } else {
                    cur_sj
                };
                let cand_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, target_pos);
                if cand_sj >= min_gate {
                    mesh.nodes[nid].position = target_pos;
                } else {
                    let mut lo = 0.0_f64;
                    let mut hi = 1.0_f64;
                    let mut best = cur_pos;
                    for _ in 0..10 {
                        let mid = 0.5 * (lo + hi);
                        let cand = [
                            cur_pos[0] + mid * (target_pos[0] - cur_pos[0]),
                            cur_pos[1] + mid * (target_pos[1] - cur_pos[1]),
                            cur_pos[2] + mid * (target_pos[2] - cur_pos[2]),
                        ];
                        if min_incident_sj(&mesh.nodes, &mesh.elements, nid, cand) >= min_gate {
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

        if polish_cycle < 2 {
            for _ in 0..2 {
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
                    for &omega in &[0.55, 0.25] {
                        let cand = [
                            cur_pos[0] + omega * (avg[0] - cur_pos[0]),
                            cur_pos[1] + omega * (avg[1] - cur_pos[1]),
                            cur_pos[2] + omega * (avg[2] - cur_pos[2]),
                        ];
                        let cand_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cand);
                        if (cur_sj < min_sj_barrier && cand_sj > cur_sj)
                            || (cand_sj >= min_sj_barrier && cand_sj >= (cur_sj * 0.88).min(0.20))
                        {
                            mesh.nodes[nid].position = cand;
                            break;
                        }
                    }
                }
            }
        }
    }

    // 6. Post-snap quality-only optimization pass: strictly increases worst incident Js for
    //    low-quality boundary-adjacent elements without moving any boundary node off the CAD surface.
    for _ in 0..3 {
        for &nid in &interior_indices {
            let neighbors = &volume_adj[nid];
            if neighbors.is_empty() {
                continue;
            }
            let cur_pos = mesh.nodes[nid].position;
            let cur_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cur_pos);
            if cur_sj >= 0.30 {
                continue;
            }
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
            let mut best_pos = cur_pos;
            let mut best_sj = cur_sj;
            for &omega in &[0.75, 0.50, 0.25, 0.12] {
                let cand = [
                    cur_pos[0] + omega * (avg[0] - cur_pos[0]),
                    cur_pos[1] + omega * (avg[1] - cur_pos[1]),
                    cur_pos[2] + omega * (avg[2] - cur_pos[2]),
                ];
                let cand_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cand);
                if cand_sj > best_sj {
                    best_sj = cand_sj;
                    best_pos = cand;
                }
            }
            if best_sj > cur_sj {
                mesh.nodes[nid].position = best_pos;
            }
        }

        // Exact-surface tangential quality polish for boundary nodes incident to low-Js tets
        for &nid in &boundary_indices {
            let cls = node_class[nid];
            if cls == BoundaryNodeClass::SharpCorner {
                continue;
            }
            let cur_pos = mesh.nodes[nid].position;
            let cur_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cur_pos);
            if cur_sj >= 0.28 {
                continue;
            }
            let Some(neighbors) = boundary_adj.get(&nid) else {
                continue;
            };
            if neighbors.is_empty() {
                continue;
            }
            let cur_v = Vector3::from(cur_pos);
            let mut best_pos = cur_pos;
            let mut best_sj = cur_sj;
            for &omega in &[0.35, 0.18, 0.08] {
                let cand_proj = if cls == BoundaryNodeClass::SharpCrease {
                    let crease_nbs: Vec<usize> = neighbors
                        .iter()
                        .copied()
                        .filter(|&nb| {
                            matches!(
                                node_class[nb],
                                BoundaryNodeClass::SharpCrease | BoundaryNodeClass::SharpCorner
                            )
                        })
                        .collect();
                    if crease_nbs.is_empty() {
                        break;
                    }
                    let mut avg = Vector3::zeros();
                    for &nb in &crease_nbs {
                        avg += mesh.nodes[nb].pos_vec();
                    }
                    avg /= crease_nbs.len() as f64;
                    let (_, tang) =
                        target_for_node(nid, cur_pos, &mut tri_scratch, &mut edge_scratch);
                    let t_vec = Vector3::from(tang);
                    let delta = avg - cur_v;
                    let cand_raw = cur_v + t_vec * (delta.dot(&t_vec) * omega);
                    let (proj, _) = target_for_node(
                        nid,
                        [cand_raw.x, cand_raw.y, cand_raw.z],
                        &mut tri_scratch,
                        &mut edge_scratch,
                    );
                    proj
                } else {
                    let (_, _, _, best_n) =
                        spatial.project_point_with_scratch(cur_pos, &mut tri_scratch);
                    let n1 = Vector3::from(best_n);
                    let mut avg = Vector3::zeros();
                    for &nb in neighbors {
                        avg += mesh.nodes[nb].pos_vec();
                    }
                    avg /= neighbors.len() as f64;
                    let delta = avg - cur_v;
                    let tang_delta = delta - n1 * delta.dot(&n1);
                    let cand_raw = cur_v + tang_delta * omega;
                    let (proj, _, _, _) = spatial.project_point_with_scratch(
                        [cand_raw.x, cand_raw.y, cand_raw.z],
                        &mut tri_scratch,
                    );
                    proj
                };
                let cand_sj = min_incident_sj(&mesh.nodes, &mesh.elements, nid, cand_proj);
                if cand_sj > best_sj && cand_sj >= min_sj_barrier {
                    best_sj = cand_sj;
                    best_pos = cand_proj;
                }
            }
            if best_sj > cur_sj {
                mesh.nodes[nid].position = best_pos;
            }
        }
    }

    mesh.boundary_conforming = true;
    mesh
}

/// Converts a boundary-snapped `TET4` mesh to quadratic `TET10` and projects exterior mid-edge
/// nodes onto curved CAD faces and curved sharp feature creases (e.g. circular hole rims),
/// strictly enforcing $\det J > 0$ (and Jacobian ratio barrier) at all 4 Gauss points of every
/// incident `TET10` element.
pub fn to_tet10_boundary_snapped(
    snapped_tet4: &TetrahedralMesh,
    cad_surface: &TriangleMesh,
    opts: SnapOptions,
) -> TetrahedralMesh {
    let mut tet10 = snapped_tet4.to_tet10();
    tet10.boundary_conforming = snapped_tet4.boundary_conforming;
    if tet10.is_empty() || cad_surface.indices.is_empty() {
        return tet10;
    }

    let num_corners = snapped_tet4.nodes.len();
    let faces6 = tet10.extract_surface_tet10_faces();
    let mut exterior_mids = HashSet::new();
    for (_corners, mids) in &faces6 {
        for &m in mids {
            if m >= num_corners {
                exterior_mids.insert(m);
            }
        }
    }

    let mut mid_incident_elems: HashMap<usize, Vec<usize>> = HashMap::new();
    for (elem_id, ten) in tet10.tet10_elements.iter().enumerate() {
        for &m in &ten[4..10] {
            if exterior_mids.contains(&m) {
                mid_incident_elems.entry(m).or_default().push(elem_id);
            }
        }
    }

    let mut mid_to_corners: HashMap<usize, (usize, usize)> = HashMap::new();
    for (elem, ten) in tet10.elements.iter().zip(tet10.tet10_elements.iter()) {
        let corners = elem.node_indices;
        for (e, &(la, lb)) in TET10_EDGE_PAIRS.iter().enumerate() {
            let m_id = ten[4 + e];
            if exterior_mids.contains(&m_id) {
                mid_to_corners
                    .entry(m_id)
                    .or_insert((corners[la], corners[lb]));
            }
        }
    }

    let spatial = SurfaceSpatialIndex::build(cad_surface);
    let sharp_index = if opts.preserve_sharp_features {
        Some(SharpFeatureIndex::build(cad_surface, opts.feature_angle_deg))
    } else {
        None
    };

    let mut tri_scratch = SpatialQueryScratch::new(cad_surface.indices.len());
    let mut edge_scratch =
        SpatialQueryScratch::new(sharp_index.as_ref().map_or(0, |s| s.edges.len()));
    let mut cached_corner_normals: Vec<Option<[f64; 3]>> = vec![None; num_corners];

    let min_ratio = opts.min_tet10_gauss_jacobian_ratio.clamp(0.05, 0.85);
    let mut sorted_mids: Vec<usize> = exterior_mids.into_iter().collect();
    sorted_mids.sort_unstable();

    for m_id in sorted_mids {
        let cur_pos = tet10.nodes[m_id].position;
        let Some(&(u, v)) = mid_to_corners.get(&m_id) else {
            continue;
        };
        let pu = tet10.nodes[u].position;
        let pv = tet10.nodes[v].position;
        let edge_len = (Vector3::from(pu) - Vector3::from(pv)).norm();
        if edge_len <= 1e-12 {
            continue;
        }

        let mut target_proj: Option<[f64; 3]> = None;

        // Case 1: Both endpoints lie on a curved sharp CAD feature crease (e.g. circular hole/bore rim)
        if let Some(ref sidx) = sharp_index {
            let tol_end = 0.08 * edge_len;
            if let (Some((_, du, tu)), Some((_, dv, tv))) = (
                sidx.project_to_sharp_edge_with_scratch(pu, tol_end, &mut edge_scratch),
                sidx.project_to_sharp_edge_with_scratch(pv, tol_end, &mut edge_scratch),
            ) {
                if du <= tol_end && dv <= tol_end {
                    let cos_t = Vector3::from(tu).dot(&Vector3::from(tv)).abs();
                    if cos_t > 0.70 {
                        if let Some((proj, dist, _)) = sidx.project_to_sharp_edge_with_scratch(
                            cur_pos,
                            0.20 * edge_len,
                            &mut edge_scratch,
                        ) {
                            if dist > 1e-9 && dist <= 0.18 * edge_len {
                                target_proj = Some(proj);
                            }
                        }
                    }
                }
            }
        }

        // Case 2: Both endpoints lie on a smooth curved CAD face
        if target_proj.is_none() {
            let nu = *cached_corner_normals[u].get_or_insert_with(|| {
                spatial
                    .project_point_with_scratch(pu, &mut tri_scratch)
                    .3
            });
            let nv = *cached_corner_normals[v].get_or_insert_with(|| {
                spatial
                    .project_point_with_scratch(pv, &mut tri_scratch)
                    .3
            });
            let cos_n = Vector3::from(nu).dot(&Vector3::from(nv));
            if cos_n > 0.72 {
                let (proj, dist, _, _) =
                    spatial.project_point_with_scratch(cur_pos, &mut tri_scratch);
                if dist > 1e-9 && dist <= 0.18 * edge_len {
                    target_proj = Some(proj);
                }
            }
        }

        let Some(proj) = target_proj else {
            continue;
        };

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
                        pos[k] = if nid == m_id {
                            cand
                        } else {
                            tet10.nodes[nid].position
                        };
                    }
                    let ratio = tet10_min_gauss_jacobian_ratio(&pos);
                    if ratio < min_ratio || check_tet10_gauss_jacobians(&pos).is_err() {
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

    tet10.boundary_conforming = true;
    tet10
}

/// Master boundary-conforming mesh generation pipeline (`P5-T2`).
pub(crate) fn generate_conforming_mesh(
    surface: &TriangleMesh,
    options: &ConformingMeshOptions,
) -> Result<TetrahedralMesh, FeaError> {
    // 1. Generate base linear TET4 mesh (locally refined conforming octree or uniform voxel)
    let base_tet4 = if options.refinement.enabled && options.refinement.max_depth > 0 {
        build_octree_conforming_tet4(surface, options.density, &options.refinement)?
    } else {
        TetrahedralMesh::from_mesh(surface, options.density)?
    };

    // 2. Boundary-conforming snapping (`fe_snap`) with Jacobian quality barrier
    // Thin helical wires automatically receive light boundary snapping even in Fast mode
    // so curved coils keep smooth cross-sections and accurate axial compliance.
    let auto_snap_helix =
        !options.snap_to_boundary && detect_helical_wire_diameter(surface).is_some();
    let (do_snap, snap_opts) = if auto_snap_helix {
        (
            true,
            SnapOptions {
                outer_passes: 3,
                smoothing_sweeps: 2,
                surface_tangential_sweeps: 1,
                preserve_sharp_features: true,
                ..options.snap
            },
        )
    } else {
        (options.snap_to_boundary, options.snap)
    };

    let tet4 = if do_snap {
        snap_mesh_to_surface(&base_tet4, surface, snap_opts)
    } else {
        base_tet4
    };

    // 3. Upgrade to quadratic TET10 (with curved boundary mid-edge snapping if conforming)
    match options.element_order {
        ElementOrder::Linear => Ok(tet4),
        ElementOrder::Quadratic => {
            if do_snap {
                Ok(to_tet10_boundary_snapped(&tet4, surface, snap_opts))
            } else {
                Ok(tet4.to_tet10())
            }
        }
    }
}
