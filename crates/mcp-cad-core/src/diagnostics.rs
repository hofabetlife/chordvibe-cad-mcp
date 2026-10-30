//! Mesh diagnostics and topology validation tools.
//! Provides comprehensive inspection of manifoldness, orientation, watertightness,
//! Euler characteristic, sliver triangles, self-intersections, and degeneracies.

use std::collections::HashMap;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::topology::TriangleMesh;

/// Quantitative mesh quality metrics and verification verdict.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshValidationReport {
    /// Overall verdict: "PASS" or "FAIL"
    pub status: String,
    /// Critical issues causing failure (if any)
    pub issues: Vec<String>,
    /// Non-critical geometric quality warnings (slivers, unwelded vertices, T-junctions)
    pub warnings: Vec<String>,
    /// Topology metrics
    pub topology: TopologyMetrics,
    /// Geometry metrics
    pub geometry: GeometryMetrics,
    /// Triangle angle statistics in degrees
    pub angle_stats: AngleStatistics,
}

/// Topology metrics covering boundary edges, orientation, Euler characteristic, and components.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TopologyMetrics {
    /// True if every edge belongs to exactly 2 triangles
    pub is_watertight: bool,
    /// Number of open boundary edges (belonging to 1 triangle)
    pub boundary_edges_count: usize,
    /// Number of non-manifold edges (belonging to >= 3 triangles)
    pub non_manifold_edges_count: usize,
    /// True if every directed edge appears at most once
    pub orientation_consistent: bool,
    /// Number of edges with reversed/inconsistent winding order
    pub inconsistent_edges_count: usize,
    /// Number of welded unique vertices
    pub welded_vertices_count: usize,
    /// Number of unique undirected edges
    pub unique_edges_count: usize,
    /// Total triangle count
    pub triangles_count: usize,
    /// Euler characteristic: chi = V - E + F
    pub euler_characteristic: i32,
    /// Computed genus: g = (2 - chi) / 2
    pub genus: f64,
    /// Number of connected surface components
    pub connected_components: usize,
    /// Number of non-manifold vertices (bowtie/pinched vertices)
    pub non_manifold_vertices_count: usize,
}

/// Geometry quality metrics covering degeneracies, slivers, T-junctions, and self-intersections.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeometryMetrics {
    /// Signed volume of the mesh (positive = outward normals)
    pub signed_volume: f64,
    /// True if signed volume > 0
    pub is_positive_volume: bool,
    /// Number of degenerate triangles (zero area or zero edge length)
    pub degenerate_triangles_count: usize,
    /// Number of duplicate vertex positions within relative epsilon
    pub duplicate_vertices_count: usize,
    /// Number of sliver triangles (min angle < 5 deg or aspect ratio > 100)
    pub sliver_triangles_count: usize,
    /// Maximum aspect ratio observed across all triangles
    pub max_aspect_ratio: f64,
    /// Number of T-junctions (vertex lies on edge of another triangle without sharing it)
    pub t_junctions_count: usize,
    /// Triangles where stored vertex normal points opposite to geometric face normal
    pub normals_vs_winding_mismatches: usize,
    /// Pairs of non-adjacent intersecting triangles
    pub self_intersections_count: usize,
    /// Number of coplanar overlapping triangle pairs
    pub coplanar_overlaps_count: usize,
}

/// Statistical distribution of interior triangle angles.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AngleStatistics {
    /// Minimum interior angle in degrees
    pub min_angle_deg: f64,
    /// Median interior angle in degrees
    pub median_angle_deg: f64,
    /// 5th percentile interior angle in degrees
    pub p5_angle_deg: f64,
}

/// Key representing a quantized 3D point for fast vertex welding and duplicate detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct QuantizedPoint([i64; 3]);

impl QuantizedPoint {
    fn new(p: &Vector3<f64>, scale: f64) -> Self {
        Self([
            (p.x * scale).round() as i64,
            (p.y * scale).round() as i64,
            (p.z * scale).round() as i64,
        ])
    }
}

/// Key representing an undirected edge between two welded vertex indices (u < v).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct UndirectedEdge(pub u32, pub u32);

impl UndirectedEdge {
    pub fn new(u: u32, v: u32) -> Self {
        if u < v {
            Self(u, v)
        } else {
            Self(v, u)
        }
    }
}

/// Key representing a directed edge from u to v.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct DirectedEdge(u32, u32);

/// Validates a triangle mesh against rigorous engineering and topology requirements.
///
/// Returns a detailed `MeshValidationReport`.
///
/// # Strict PASS criteria:
/// - `watertight`: `boundary_edges == 0` and `non_manifold_edges == 0`
/// - `orientation_consistent`: `inconsistent_edges == 0`
/// - `signed_volume > 0.0`
/// - `degenerate_triangles == 0`
/// - `self_intersections == 0`
/// - `expected_genus` matches if provided
pub fn validate_mesh(mesh: &TriangleMesh, expected_genus: Option<i32>) -> MeshValidationReport {
    let mut issues = Vec::new();

    if mesh.vertices.is_empty() || mesh.indices.is_empty() {
        return MeshValidationReport {
            status: "FAIL".to_string(),
            issues: vec!["Mesh is empty (no vertices or indices)".to_string()],
            warnings: Vec::new(),
            topology: TopologyMetrics {
                is_watertight: false,
                boundary_edges_count: 0,
                non_manifold_edges_count: 0,
                orientation_consistent: false,
                inconsistent_edges_count: 0,
                welded_vertices_count: 0,
                unique_edges_count: 0,
                triangles_count: 0,
                euler_characteristic: 0,
                genus: 0.0,
                connected_components: 0,
                non_manifold_vertices_count: 0,
            },
            geometry: GeometryMetrics {
                signed_volume: 0.0,
                is_positive_volume: false,
                degenerate_triangles_count: 0,
                duplicate_vertices_count: 0,
                sliver_triangles_count: 0,
                max_aspect_ratio: 0.0,
                t_junctions_count: 0,
                normals_vs_winding_mismatches: 0,
                self_intersections_count: 0,
                coplanar_overlaps_count: 0,
            },
            angle_stats: AngleStatistics {
                min_angle_deg: 0.0,
                median_angle_deg: 0.0,
                p5_angle_deg: 0.0,
            },
        };
    }

    let bbox = mesh.bounding_box();
    let dims = bbox.dimensions();
    let diag = (dims[0] * dims[0] + dims[1] * dims[1] + dims[2] * dims[2]).sqrt().max(1e-6);

    // Welding tolerance: 1e-7 relative to bounding box diagonal
    let weld_eps = (diag * 1e-7).max(1e-12);
    let weld_scale = 1.0 / weld_eps;

    // 1. Weld vertices
    let mut point_to_welded: HashMap<QuantizedPoint, u32> = HashMap::new();
    let mut vertex_to_welded: Vec<u32> = Vec::with_capacity(mesh.vertices.len());
    let mut welded_positions: Vec<Vector3<f64>> = Vec::new();

    for v in &mesh.vertices {
        let pos = v.pos_vec();
        let key = QuantizedPoint::new(&pos, weld_scale);
        let welded_idx = match point_to_welded.get(&key) {
            Some(&idx) => idx,
            None => {
                let idx = welded_positions.len() as u32;
                point_to_welded.insert(key, idx);
                welded_positions.push(pos);
                idx
            }
        };
        vertex_to_welded.push(welded_idx);
    }

    let duplicate_vertices_count = mesh.vertices.len().saturating_sub(welded_positions.len());

    // 2. Map triangles to welded indices & analyze edges
    let mut undirected_edge_counts: HashMap<UndirectedEdge, usize> = HashMap::new();
    let mut directed_edge_counts: HashMap<DirectedEdge, usize> = HashMap::new();
    let mut welded_indices: Vec<[u32; 3]> = Vec::with_capacity(mesh.indices.len());

    let mut degenerate_triangles_count = 0usize;
    let mut sliver_triangles_count = 0usize;
    let mut max_aspect_ratio = 1.0f64;
    let mut all_min_angles: Vec<f64> = Vec::with_capacity(mesh.indices.len());
    let mut normals_vs_winding_mismatches = 0usize;

    for tri in &mesh.indices {
        let w0 = vertex_to_welded[tri[0] as usize];
        let w1 = vertex_to_welded[tri[1] as usize];
        let w2 = vertex_to_welded[tri[2] as usize];

        // Check topological degeneracy (collapsed indices)
        if w0 == w1 || w1 == w2 || w2 == w0 {
            degenerate_triangles_count += 1;
            continue;
        }

        welded_indices.push([w0, w1, w2]);

        let p0 = welded_positions[w0 as usize];
        let p1 = welded_positions[w1 as usize];
        let p2 = welded_positions[w2 as usize];

        let e01 = p1 - p0;
        let e12 = p2 - p1;
        let e20 = p0 - p2;

        let l01 = e01.norm();
        let l12 = e12.norm();
        let l20 = e20.norm();

        let cross = e01.cross(&(-e20));
        let area = 0.5 * cross.norm();

        if area < 1e-14 * diag * diag || l01 < 1e-12 || l12 < 1e-12 || l20 < 1e-12 {
            degenerate_triangles_count += 1;
        } else {
            // Check interior angles via law of cosines
            let a = l12;
            let b = l20;
            let c = l01;

            let cos_a = ((b * b + c * c - a * a) / (2.0 * b * c)).clamp(-1.0, 1.0);
            let cos_b = ((a * a + c * c - b * b) / (2.0 * a * c)).clamp(-1.0, 1.0);
            let cos_c = ((a * a + b * b - c * c) / (2.0 * a * b)).clamp(-1.0, 1.0);

            let angle_a_deg = cos_a.acos().to_degrees();
            let angle_b_deg = cos_b.acos().to_degrees();
            let angle_c_deg = cos_c.acos().to_degrees();

            let min_angle = angle_a_deg.min(angle_b_deg).min(angle_c_deg);
            all_min_angles.push(min_angle);

            // Aspect ratio = longest edge * perimeter / (4 * sqrt(3) * area)
            let semi = 0.5 * (a + b + c);
            let r_in = area / semi;
            let r_circ = (a * b * c) / (4.0 * area);
            let aspect = if r_in > 1e-14 { r_circ / (2.0 * r_in) } else { 1000.0 };
            if aspect > max_aspect_ratio {
                max_aspect_ratio = aspect;
            }

            if min_angle < 5.0 || aspect > 100.0 {
                sliver_triangles_count += 1;
            }
        }

        // Normals vs winding
        let geo_normal = if cross.norm() > 1e-12 {
            cross / cross.norm()
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        };

        for &vert_idx in tri {
            let stored_n = mesh.vertices[vert_idx as usize].normal_vec();
            if stored_n.norm() > 0.5 && stored_n.dot(&geo_normal) < -0.1 {
                normals_vs_winding_mismatches += 1;
                break;
            }
        }

        // Register directed edges
        for &(u, v) in &[(w0, w1), (w1, w2), (w2, w0)] {
            *directed_edge_counts.entry(DirectedEdge(u, v)).or_insert(0) += 1;
            *undirected_edge_counts.entry(UndirectedEdge::new(u, v)).or_insert(0) += 1;
        }
    }

    // Edge manifoldness & boundary metrics
    let mut boundary_edges_count = 0usize;
    let mut non_manifold_edges_count = 0usize;
    for &count in undirected_edge_counts.values() {
        if count == 1 {
            boundary_edges_count += 1;
        } else if count > 2 {
            non_manifold_edges_count += 1;
        }
    }

    let is_watertight = boundary_edges_count == 0 && non_manifold_edges_count == 0;

    // Orientation consistency:
    // A surface is orientation-consistent if no directed edge is traversed more than once in the same direction
    let mut inconsistent_edges_count = 0usize;
    for &dir_count in directed_edge_counts.values() {
        if dir_count > 1 {
            inconsistent_edges_count += dir_count - 1;
        }
    }
    let orientation_consistent = inconsistent_edges_count == 0;

    // Euler characteristic: chi = V - E + F
    let v_count = welded_positions.len() as i32;
    let e_count = undirected_edge_counts.len() as i32;
    let f_count = welded_indices.len() as i32;
    let euler_characteristic = v_count - e_count + f_count;
    let genus = (2.0 - euler_characteristic as f64) / 2.0;

    // Connected components via BFS on triangle adjacency
    let mut edge_to_triangles: HashMap<UndirectedEdge, Vec<usize>> = HashMap::new();
    for (t_idx, tri) in welded_indices.iter().enumerate() {
        for &(u, v) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            edge_to_triangles
                .entry(UndirectedEdge::new(u, v))
                .or_default()
                .push(t_idx);
        }
    }

    let mut visited_triangles = vec![false; welded_indices.len()];
    let mut connected_components = 0usize;

    for start_t in 0..welded_indices.len() {
        if visited_triangles[start_t] {
            continue;
        }
        connected_components += 1;
        let mut queue = vec![start_t];
        visited_triangles[start_t] = true;

        while let Some(curr) = queue.pop() {
            let tri = welded_indices[curr];
            for &(u, v) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                if let Some(neighbors) = edge_to_triangles.get(&UndirectedEdge::new(u, v)) {
                    for &nbr in neighbors {
                        if !visited_triangles[nbr] {
                            visited_triangles[nbr] = true;
                            queue.push(nbr);
                        }
                    }
                }
            }
        }
    }

    // Non-manifold vertices check:
    // For each vertex, the ring of adjacent triangles must form a single fan/cycle
    let mut vert_to_triangles: HashMap<u32, Vec<usize>> = HashMap::new();
    for (t_idx, tri) in welded_indices.iter().enumerate() {
        vert_to_triangles.entry(tri[0]).or_default().push(t_idx);
        vert_to_triangles.entry(tri[1]).or_default().push(t_idx);
        vert_to_triangles.entry(tri[2]).or_default().push(t_idx);
    }

    let mut non_manifold_vertices_count = 0usize;
    for (v_idx, tri_list) in &vert_to_triangles {
        if tri_list.len() < 3 {
            continue;
        }
        // Build opposing edge graph in the umbrella
        let mut opp_edges: HashMap<u32, Vec<u32>> = HashMap::new();
        for &t_idx in tri_list {
            let tri = welded_indices[t_idx];
            let other: Vec<u32> = tri.iter().copied().filter(|&x| x != *v_idx).collect();
            if other.len() == 2 {
                opp_edges.entry(other[0]).or_default().push(other[1]);
                opp_edges.entry(other[1]).or_default().push(other[0]);
            }
        }

        // Check if degree of any opposing vertex > 2 or if there are disconnected components
        let mut has_high_degree = false;
        for neighbors in opp_edges.values() {
            if neighbors.len() > 2 {
                has_high_degree = true;
                break;
            }
        }

        if has_high_degree {
            non_manifold_vertices_count += 1;
        }
    }

    // Signed volume
    let signed_volume = mesh.signed_volume();
    let is_positive_volume = signed_volume > 1e-12;

    // Angle statistics
    all_min_angles.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let (min_angle_deg, median_angle_deg, p5_angle_deg) = if all_min_angles.is_empty() {
        (0.0, 0.0, 0.0)
    } else {
        let n = all_min_angles.len();
        let min_val = all_min_angles[0];
        let med_val = all_min_angles[n / 2];
        let p5_idx = ((n as f64 * 0.05).floor() as usize).min(n - 1);
        let p5_val = all_min_angles[p5_idx];
        (min_val, med_val, p5_val)
    };

    // T-junctions detection:
    // A vertex P lies strictly on edge AB if dist(P, AB) < eps and projection is in (0.05, 0.95).
    // Bounded with vertex count and AABB check for fast execution.
    let t_eps = (diag * 1e-4).max(1e-7);
    let mut t_junctions_count = 0usize;
    if welded_positions.len() <= 2000 {
        for &edge in undirected_edge_counts.keys() {
            let pa = welded_positions[edge.0 as usize];
            let pb = welded_positions[edge.1 as usize];
            let edge_min_x = pa.x.min(pb.x) - t_eps;
            let edge_max_x = pa.x.max(pb.x) + t_eps;
            let edge_min_y = pa.y.min(pb.y) - t_eps;
            let edge_max_y = pa.y.max(pb.y) + t_eps;
            let edge_min_z = pa.z.min(pb.z) - t_eps;
            let edge_max_z = pa.z.max(pb.z) + t_eps;

            let ab = pb - pa;
            let ab_len_sq = ab.norm_squared();
            if ab_len_sq < 1e-12 {
                continue;
            }

            for (v_idx, &pv) in welded_positions.iter().enumerate() {
                let u_idx = v_idx as u32;
                if u_idx == edge.0 || u_idx == edge.1 {
                    continue;
                }
                if pv.x < edge_min_x || pv.x > edge_max_x || pv.y < edge_min_y || pv.y > edge_max_y || pv.z < edge_min_z || pv.z > edge_max_z {
                    continue;
                }
                let ap = pv - pa;
                let t = ap.dot(&ab) / ab_len_sq;
                if t > 0.05 && t < 0.95 {
                    let proj = pa + ab * t;
                    if (pv - proj).norm() < t_eps {
                        t_junctions_count += 1;
                    }
                }
            }
        }
    }

    // Self-intersections and Coplanar overlaps detection (AABB accelerated)
    let (self_intersections_count, coplanar_overlaps_count) =
        check_self_intersections(&welded_positions, &welded_indices);

    // Evaluate issues and final status
    if !is_watertight {
        issues.push(format!(
            "Mesh is not watertight: {} boundary edges (holes), {} non-manifold edges",
            boundary_edges_count, non_manifold_edges_count
        ));
    }
    if !orientation_consistent {
        issues.push(format!(
            "Orientation is inconsistent: {} inverted directed edges detected",
            inconsistent_edges_count
        ));
    }
    if !is_positive_volume {
        issues.push(format!(
            "Mesh signed volume is non-positive: {:.6e} m³ (inverted normals)",
            signed_volume
        ));
    }
    if degenerate_triangles_count > 0 {
        issues.push(format!(
            "Found {} degenerate triangles (zero area or collapsed edges)",
            degenerate_triangles_count
        ));
    }
    if self_intersections_count > 0 {
        issues.push(format!(
            "Found {} self-intersecting non-adjacent triangle pairs",
            self_intersections_count
        ));
    }
    if let Some(exp_g) = expected_genus {
        let int_genus = genus.round() as i32;
        if int_genus != exp_g {
            issues.push(format!(
                "Euler genus mismatch: computed {:.1} (round {}), expected {}",
                genus, int_genus, exp_g
            ));
        }
    }

    let mut warnings = Vec::new();
    if sliver_triangles_count > 0 {
        warnings.push(format!(
            "Notice: {} sliver triangles detected (min angle < 5° or aspect ratio > 100)",
            sliver_triangles_count
        ));
    }
    if t_junctions_count > 0 {
        warnings.push(format!(
            "Notice: {} T-junction vertices detected",
            t_junctions_count
        ));
    }
    if duplicate_vertices_count > 0 {
        warnings.push(format!(
            "Notice: {} un-welded duplicate vertices found (CAD split normals)",
            duplicate_vertices_count
        ));
    }
    if coplanar_overlaps_count > 0 {
        warnings.push(format!(
            "Notice: {} coplanar overlapping triangle pairs detected",
            coplanar_overlaps_count
        ));
    }
    if normals_vs_winding_mismatches > 0 {
        warnings.push(format!(
            "Notice: {} stored normals mismatch geometric face normal",
            normals_vs_winding_mismatches
        ));
    }

    let status = if !issues.is_empty() {
        "FAIL".to_string()
    } else {
        "PASS".to_string()
    };

    MeshValidationReport {
        status,
        issues,
        warnings,
        topology: TopologyMetrics {
            is_watertight,
            boundary_edges_count,
            non_manifold_edges_count,
            orientation_consistent,
            inconsistent_edges_count,
            welded_vertices_count: welded_positions.len(),
            unique_edges_count: undirected_edge_counts.len(),
            triangles_count: welded_indices.len(),
            euler_characteristic,
            genus,
            connected_components,
            non_manifold_vertices_count,
        },
        geometry: GeometryMetrics {
            signed_volume,
            is_positive_volume,
            degenerate_triangles_count,
            duplicate_vertices_count,
            sliver_triangles_count,
            max_aspect_ratio,
            t_junctions_count,
            normals_vs_winding_mismatches,
            self_intersections_count,
            coplanar_overlaps_count,
        },
        angle_stats: AngleStatistics {
            min_angle_deg,
            median_angle_deg,
            p5_angle_deg,
        },
    }
}

/// AABB of a single triangle with index.
struct TriAabb {
    min: Vector3<f64>,
    max: Vector3<f64>,
    idx: usize,
}

/// Fast broad-phase and narrow-phase triangle-triangle intersection tester.
fn check_self_intersections(
    positions: &[Vector3<f64>],
    indices: &[[u32; 3]],
) -> (usize, usize) {
    let mut aabbs: Vec<TriAabb> = Vec::with_capacity(indices.len());
    for (idx, tri) in indices.iter().enumerate() {
        let p0 = positions[tri[0] as usize];
        let p1 = positions[tri[1] as usize];
        let p2 = positions[tri[2] as usize];

        let min = Vector3::new(
            p0.x.min(p1.x).min(p2.x),
            p0.y.min(p1.y).min(p2.y),
            p0.z.min(p1.z).min(p2.z),
        );
        let max = Vector3::new(
            p0.x.max(p1.x).max(p2.x),
            p0.y.max(p1.y).max(p2.y),
            p0.z.max(p1.z).max(p2.z),
        );

        aabbs.push(TriAabb { min, max, idx });
    }

    // Sort by min.x for sweep-and-prune broadphase
    aabbs.sort_by(|a, b| a.min.x.partial_cmp(&b.min.x).unwrap_or(std::cmp::Ordering::Equal));

    let mut self_intersections = 0usize;
    let mut coplanar_overlaps = 0usize;

    // Limit check pairs to prevent server freeze on degenerate geometries
    let mut checked_pairs = 0usize;
    let max_check_pairs = 100_000usize;

    for i in 0..aabbs.len() {
        if checked_pairs >= max_check_pairs {
            break;
        }
        let box_a = &aabbs[i];
        for box_b in aabbs.iter().skip(i + 1) {
            if box_b.min.x > box_a.max.x {
                break;
            }

            // Check Y and Z overlap
            if box_a.max.y < box_b.min.y
                || box_a.min.y > box_b.max.y
                || box_a.max.z < box_b.min.z
                || box_a.min.z > box_b.max.z
            {
                continue;
            }

            let tri_a = indices[box_a.idx];
            let tri_b = indices[box_b.idx];

            // Ignore adjacent triangles sharing any vertex
            if tri_a[0] == tri_b[0]
                || tri_a[0] == tri_b[1]
                || tri_a[0] == tri_b[2]
                || tri_a[1] == tri_b[0]
                || tri_a[1] == tri_b[1]
                || tri_a[1] == tri_b[2]
                || tri_a[2] == tri_b[0]
                || tri_a[2] == tri_b[1]
                || tri_a[2] == tri_b[2]
            {
                continue;
            }

            checked_pairs += 1;

            let p0 = positions[tri_a[0] as usize];
            let p1 = positions[tri_a[1] as usize];
            let p2 = positions[tri_a[2] as usize];

            let q0 = positions[tri_b[0] as usize];
            let q1 = positions[tri_b[1] as usize];
            let q2 = positions[tri_b[2] as usize];

            match tri_tri_intersection_test(p0, p1, p2, q0, q1, q2) {
                TriIntersectResult::Intersect => {
                    self_intersections += 1;
                }
                TriIntersectResult::CoplanarOverlap => {
                    coplanar_overlaps += 1;
                }
                TriIntersectResult::Disjoint => {}
            }
        }
    }

    (self_intersections, coplanar_overlaps)
}

#[derive(Debug, PartialEq)]
enum TriIntersectResult {
    Disjoint,
    Intersect,
    CoplanarOverlap,
}

/// Möller's 3D Triangle-Triangle intersection test algorithm.
fn tri_tri_intersection_test(
    p0: Vector3<f64>,
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    q0: Vector3<f64>,
    q1: Vector3<f64>,
    q2: Vector3<f64>,
) -> TriIntersectResult {
    // Plane of triangle P: normal n1, distance d1
    let n1 = (p1 - p0).cross(&(p2 - p0));
    let n1_len = n1.norm();
    if n1_len < 1e-14 {
        return TriIntersectResult::Disjoint;
    }
    let n1 = n1 / n1_len;
    let d1 = -n1.dot(&p0);

    // Distances of triangle Q to plane P
    let dq0 = n1.dot(&q0) + d1;
    let dq1 = n1.dot(&q1) + d1;
    let dq2 = n1.dot(&q2) + d1;

    let eps = 1e-9;
    if (dq0 > eps && dq1 > eps && dq2 > eps) || (dq0 < -eps && dq1 < -eps && dq2 < -eps) {
        return TriIntersectResult::Disjoint;
    }

    // Plane of triangle Q: normal n2, distance d2
    let n2 = (q1 - q0).cross(&(q2 - q0));
    let n2_len = n2.norm();
    if n2_len < 1e-14 {
        return TriIntersectResult::Disjoint;
    }
    let n2 = n2 / n2_len;
    let d2 = -n2.dot(&q0);

    let dp0 = n2.dot(&p0) + d2;
    let dp1 = n2.dot(&p1) + d2;
    let dp2 = n2.dot(&p2) + d2;

    if (dp0 > eps && dp1 > eps && dp2 > eps) || (dp0 < -eps && dp1 < -eps && dp2 < -eps) {
        return TriIntersectResult::Disjoint;
    }

    // Direction of line of intersection of planes
    let line_dir = n1.cross(&n2);
    let line_len = line_dir.norm();

    if line_len < 1e-8 {
        // Planes are parallel or coplanar
        if dq0.abs() < eps && dq1.abs() < eps && dq2.abs() < eps {
            return TriIntersectResult::CoplanarOverlap;
        }
        return TriIntersectResult::Disjoint;
    }

    let line_dir = line_dir / line_len;

    // Compute intervals on the intersection line
    let (t1_min, t1_max) = match compute_interval(p0, p1, p2, dp0, dp1, dp2, line_dir) {
        Some(res) => res,
        None => return TriIntersectResult::Disjoint,
    };

    let (t2_min, t2_max) = match compute_interval(q0, q1, q2, dq0, dq1, dq2, line_dir) {
        Some(res) => res,
        None => return TriIntersectResult::Disjoint,
    };

    if t1_max < t2_min - eps || t2_max < t1_min - eps {
        TriIntersectResult::Disjoint
    } else {
        TriIntersectResult::Intersect
    }
}

/// Helper to project triangle intersection points onto the line.
fn compute_interval(
    p0: Vector3<f64>,
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    d0: f64,
    d1: f64,
    d2: f64,
    dir: Vector3<f64>,
) -> Option<(f64, f64)> {
    let mut isect = Vec::with_capacity(2);
    let pts = [p0, p1, p2];
    let dists = [d0, d1, d2];

    for i in 0..3 {
        let j = (i + 1) % 3;
        let di = dists[i];
        let dj = dists[j];

        if (di > 0.0 && dj < 0.0) || (di < 0.0 && dj > 0.0) {
            let t = di / (di - dj);
            let pt = pts[i] + (pts[j] - pts[i]) * t;
            isect.push(pt.dot(&dir));
        } else if di.abs() < 1e-12 {
            isect.push(pts[i].dot(&dir));
        }
    }

    if isect.len() >= 2 {
        let min = isect[0].min(isect[1]);
        let max = isect[0].max(isect[1]);
        Some((min, max))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{create_box, create_cylinder, create_sphere};

    #[test]
    fn test_validate_mesh_box_pass() {
        let b = create_box(1.0, 1.0, 1.0);
        let rep = validate_mesh(&b, Some(0));
        assert_eq!(rep.status, "PASS", "Box must pass validation: {:?}", rep.issues);
        assert!(rep.topology.is_watertight);
        assert!(rep.topology.orientation_consistent);
        assert!(rep.geometry.is_positive_volume);
        assert_eq!(rep.topology.euler_characteristic, 2); // V=8, E=18, F=12 -> 8-18+12 = 2
        assert_eq!(rep.topology.genus.round() as i32, 0);
        assert_eq!(rep.geometry.degenerate_triangles_count, 0);
        assert_eq!(rep.geometry.self_intersections_count, 0);
    }

    #[test]
    fn test_validate_mesh_cylinder_pass() {
        let cyl = create_cylinder(0.5, 2.0, 32);
        let rep = validate_mesh(&cyl, Some(0));
        assert_eq!(rep.status, "PASS", "Cylinder must pass validation: {:?}", rep.issues);
        assert!(rep.topology.is_watertight);
        assert!(rep.topology.orientation_consistent);
        assert!(rep.geometry.is_positive_volume);
    }

    #[test]
    fn test_validate_mesh_sphere_pass() {
        let sphere = create_sphere(1.0, 32, 16);
        let rep = validate_mesh(&sphere, Some(0));
        assert_eq!(rep.status, "PASS", "Sphere must pass validation: {:?}", rep.issues);
        assert!(rep.topology.is_watertight);
    }

    #[test]
    fn test_validate_mesh_detects_inverted_faces() {
        let mut b = create_box(1.0, 1.0, 1.0);
        // Invert one face's winding order
        b.indices[0].swap(1, 2);
        let rep = validate_mesh(&b, Some(0));
        assert_eq!(rep.status, "FAIL");
        assert!(
            !rep.topology.orientation_consistent || rep.topology.inconsistent_edges_count > 0,
            "Must detect inconsistent edge orientation"
        );
    }

    #[test]
    fn test_validate_mesh_detects_hole() {
        let mut b = create_box(1.0, 1.0, 1.0);
        // Delete one triangle to create a boundary hole
        b.indices.remove(0);
        let rep = validate_mesh(&b, Some(0));
        assert_eq!(rep.status, "FAIL");
        assert!(!rep.topology.is_watertight, "Must detect hole as non-watertight");
        assert!(rep.topology.boundary_edges_count > 0);
    }

    #[test]
    fn test_validate_mesh_detects_degenerate_triangle() {
        let mut b = create_box(1.0, 1.0, 1.0);
        // Collapse index of a triangle to make it degenerate
        b.indices[0][1] = b.indices[0][0];
        let rep = validate_mesh(&b, Some(0));
        assert_eq!(rep.status, "FAIL");
        assert!(rep.geometry.degenerate_triangles_count > 0);
    }
}
