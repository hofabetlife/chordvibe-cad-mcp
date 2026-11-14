//! 3D Tetrahedral mesh generation, representation, and topological utilities.

use std::collections::{HashMap, HashSet};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use mcp_cad_core::part::Part;
use mcp_cad_core::tessellation::{TessellationQuality, TessellationSettings};
use mcp_cad_core::topology::{BoundingBox, TriangleMesh, Vertex};
use crate::error::FeaError;

/// Density profile for automatic tetrahedral meshing.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub enum MeshDensity {
    /// Coarse mesh: faster computation, lower element count.
    Coarse,
    /// Medium mesh: balanced engineering accuracy and speed.
    #[default]
    Medium,
    /// Fine mesh: higher resolution for localized stress gradients.
    Fine,
    /// Custom maximum element edge length in meters.
    Custom {
        /// Target element edge length (m)
        max_element_size: f64,
    },
}

impl MeshDensity {
    /// Computes the characteristic target element size $h$ (in meters) for a given bounding box.
    pub fn target_element_size(&self, aabb: &BoundingBox) -> f64 {
        let dims = aabb.dimensions();
        let max_dim = dims[0].max(dims[1]).max(dims[2]);
        let min_dim = dims[0].min(dims[1]).min(dims[2]);

        match *self {
            MeshDensity::Coarse => (max_dim / 10.0).min(min_dim / 2.0).max(1e-5),
            MeshDensity::Medium => (max_dim / 18.0).min(min_dim / 4.0).max(1e-5),
            MeshDensity::Fine => (max_dim / 28.0).min(min_dim / 6.0).max(1e-5),
            MeshDensity::Custom { max_element_size } => {
                if max_element_size > 0.0 {
                    max_element_size
                } else {
                    (max_dim / 18.0).min(min_dim / 4.0).max(1e-5)
                }
            }
        }
    }

    /// Derives appropriate adaptive `TessellationSettings` with `max_edge_length`
    /// constrained to the target element size.
    pub fn to_tessellation_settings(&self, aabb: &BoundingBox) -> TessellationSettings {
        let h = self.target_element_size(aabb);
        match *self {
            MeshDensity::Coarse => TessellationSettings::coarse().with_max_edge_length(Some(h)),
            MeshDensity::Medium => TessellationSettings::medium().with_max_edge_length(Some(h)),
            MeshDensity::Fine => TessellationSettings::fine().with_max_edge_length(Some(h)),
            MeshDensity::Custom { .. } => {
                TessellationSettings::medium().with_max_edge_length(Some(h))
            }
        }
    }

    /// Maps the mesh density to its corresponding CAD `TessellationQuality` preset.
    pub fn to_tessellation_quality(&self) -> TessellationQuality {
        match *self {
            MeshDensity::Coarse => TessellationQuality::Coarse,
            MeshDensity::Medium => TessellationQuality::Medium,
            MeshDensity::Fine => TessellationQuality::Fine,
            MeshDensity::Custom { max_element_size } => TessellationQuality::Custom {
                chord_tolerance: (max_element_size * 0.1).max(1e-5),
                angular_tolerance_deg: 10.0,
                max_edge_length: Some(max_element_size),
            },
        }
    }
}

/// A node (vertex) in the 3D tetrahedral mesh.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TetNode {
    /// Zero-based sequential node index.
    pub id: usize,
    /// 3D position [x, y, z] in meters.
    pub position: [f64; 3],
}

impl TetNode {
    /// Creates a new mesh node.
    pub fn new(id: usize, position: [f64; 3]) -> Self {
        Self { id, position }
    }

    /// Returns position as an nalgebra Vector3.
    #[inline]
    pub fn pos_vec(&self) -> Vector3<f64> {
        Vector3::new(self.position[0], self.position[1], self.position[2])
    }
}

/// A linear 4-node tetrahedron (TET4) element.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TetElement {
    /// Zero-based sequential element index.
    pub id: usize,
    /// The 4 node indices forming the tetrahedron.
    pub node_indices: [usize; 4],
}

impl TetElement {
    /// Creates a new tetrahedral element.
    pub fn new(id: usize, node_indices: [usize; 4]) -> Self {
        Self { id, node_indices }
    }
}

/// Finite element polynomial interpolation order (`1` = linear `TET4`, `2` = quadratic `TET10`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum ElementOrder {
    /// Linear 4-node tetrahedron (`TET4`, `element_order = 1`).
    #[default]
    Linear = 1,
    /// Quadratic 10-node tetrahedron (`TET10`, `element_order = 2`).
    Quadratic = 2,
}

impl ElementOrder {
    /// Parses an integer order (`1` or `2`) into `ElementOrder`.
    pub fn from_u8(order: u8) -> Option<Self> {
        match order {
            1 => Some(Self::Linear),
            2 => Some(Self::Quadratic),
            _ => None,
        }
    }

    /// Returns the numeric polynomial order (`1` or `2`).
    #[inline]
    pub fn as_u8(self) -> u8 {
        match self {
            Self::Linear => 1,
            Self::Quadratic => 2,
        }
    }
}

/// An unstructured 3D tetrahedral finite element mesh (supporting both `TET4` and `TET10`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TetrahedralMesh {
    /// Array of mesh nodes (corner nodes followed by mid-edge nodes when converted to `TET10`).
    pub nodes: Vec<TetNode>,
    /// Array of 4-node corner tetrahedral elements (`TET4` connectivity).
    pub elements: Vec<TetElement>,
    /// Optional 10-node quadratic tetrahedral connectivity (`[n0..n3, n4..n9]` per element) when `is_tet10()` is true.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tet10_elements: Vec<[usize; 10]>,
}

impl TetrahedralMesh {
    /// Creates a new linear (`TET4`) tetrahedral mesh from nodes and elements.
    pub fn new(nodes: Vec<TetNode>, elements: Vec<TetElement>) -> Self {
        Self {
            nodes,
            elements,
            tet10_elements: Vec::new(),
        }
    }

    /// Creates a new quadratic (`TET10`) tetrahedral mesh from nodes, corner elements, and 10-node connectivity.
    pub fn new_tet10(
        nodes: Vec<TetNode>,
        elements: Vec<TetElement>,
        tet10_elements: Vec<[usize; 10]>,
    ) -> Self {
        Self {
            nodes,
            elements,
            tet10_elements,
        }
    }

    /// Creates an empty mesh.
    pub fn empty() -> Self {
        Self {
            nodes: Vec::new(),
            elements: Vec::new(),
            tet10_elements: Vec::new(),
        }
    }

    /// Returns `true` if this mesh carries quadratic 10-node (`TET10`) element connectivity.
    #[inline]
    pub fn is_tet10(&self) -> bool {
        !self.tet10_elements.is_empty() && self.tet10_elements.len() == self.elements.len()
    }

    /// Returns the polynomial element order of the mesh (`1` for `TET4`, `2` for `TET10`).
    #[inline]
    pub fn element_order(&self) -> u8 {
        if self.is_tet10() {
            2
        } else {
            1
        }
    }

    /// Converts a linear `TET4` tetrahedral mesh into a quadratic `TET10` mesh by inserting
    /// a shared mid-edge node at the midpoint of every unique tetrahedron edge `(u, v)`.
    ///
    /// Node ordering per element follows the standard `TET10` convention:
    /// - `0..4`: corner vertices `[n0, n1, n2, n3]`
    /// - `4..10`: mid-edge nodes `[(0,1), (1,2), (2,0), (0,3), (1,3), (2,3)]`
    pub fn to_tet10(&self) -> Self {
        if self.is_tet10() || self.is_empty() {
            return self.clone();
        }

        let mut nodes = self.nodes.clone();
        // Map canonical undirected edge (min_node, max_node) -> mid_node_id
        let mut edge_to_mid: HashMap<(usize, usize), usize> =
            HashMap::with_capacity(self.elements.len() * 2);
        let mut tet10_elements = Vec::with_capacity(self.elements.len());

        for elem in &self.elements {
            let corners = elem.node_indices;
            let mut ten = [0usize; 10];
            ten[..4].copy_from_slice(&corners);

            for (e, &(la, lb)) in crate::element::TET10_EDGE_PAIRS.iter().enumerate() {
                let u = corners[la];
                let v = corners[lb];
                let key = if u < v { (u, v) } else { (v, u) };
                let mid_id = *edge_to_mid.entry(key).or_insert_with(|| {
                    let pu = nodes[u].position;
                    let pv = nodes[v].position;
                    let mid_pos = [
                        0.5 * (pu[0] + pv[0]),
                        0.5 * (pu[1] + pv[1]),
                        0.5 * (pu[2] + pv[2]),
                    ];
                    let id = nodes.len();
                    nodes.push(TetNode::new(id, mid_pos));
                    id
                });
                ten[4 + e] = mid_id;
            }

            tet10_elements.push(ten);
        }

        Self {
            nodes,
            elements: self.elements.clone(),
            tet10_elements,
        }
    }

    /// Returns the number of nodes in the mesh.
    #[inline]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Returns the number of tetrahedral elements in the mesh.
    #[inline]
    pub fn element_count(&self) -> usize {
        self.elements.len()
    }

    /// Checks if the mesh has no nodes or elements.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty() || self.elements.is_empty()
    }

    /// Computes the axis-aligned bounding box enclosing all mesh nodes.
    pub fn bounding_box(&self) -> BoundingBox {
        if self.nodes.is_empty() {
            return BoundingBox::new([0.0; 3], [0.0; 3]);
        }
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];

        for node in &self.nodes {
            for i in 0..3 {
                if node.position[i] < min[i] {
                    min[i] = node.position[i];
                }
                if node.position[i] > max[i] {
                    max[i] = node.position[i];
                }
            }
        }

        BoundingBox::new(min, max)
    }

    /// Finds all node IDs lying on or near a 3D plane:
    /// `|(pos - origin) · normal| <= tolerance`
    pub fn find_nodes_in_plane(&self, origin: [f64; 3], normal: [f64; 3], tolerance: f64) -> Vec<usize> {
        let n = Vector3::new(normal[0], normal[1], normal[2]);
        let n_len = n.norm();
        let unit_n = if n_len > 1e-12 { n / n_len } else { Vector3::new(0.0, 0.0, 1.0) };
        let p0 = Vector3::new(origin[0], origin[1], origin[2]);

        self.nodes
            .iter()
            .filter(|node| {
                let dist = ((node.pos_vec() - p0).dot(&unit_n)).abs();
                dist <= tolerance
            })
            .map(|node| node.id)
            .collect()
    }

    /// Finds all node IDs satisfying a coordinate predicate function.
    pub fn find_nodes_by_predicate<F: Fn([f64; 3]) -> bool>(&self, predicate: F) -> Vec<usize> {
        self.nodes
            .iter()
            .filter(|node| predicate(node.position))
            .map(|node| node.id)
            .collect()
    }

    /// Extracts the external 6-node quadratic boundary faces `([c0, c1, c2], [m01, m12, m20])`
    /// when `self.is_tet10()` is `true`. Returns an empty vector for linear `TET4` meshes.
    pub fn extract_surface_tet10_faces(&self) -> Vec<([usize; 3], [usize; 3])> {
        if !self.is_tet10() {
            return Vec::new();
        }

        let mut face_count: HashMap<[usize; 3], (usize, [usize; 3], [usize; 3])> =
            HashMap::with_capacity(self.elements.len() * 2);

        for ten in &self.tet10_elements {
            let [n0, n1, n2, n3, m01, m12, m20, m03, m13, m23] = *ten;
            let faces = [
                ([n0, n1, n2], [m01, m12, m20]),
                ([n0, n2, n3], [m20, m23, m03]),
                ([n0, n3, n1], [m03, m13, m01]),
                ([n1, n3, n2], [m13, m23, m12]),
            ];

            for (corners, mids) in faces {
                let mut sorted = corners;
                sorted.sort_unstable();
                let entry = face_count.entry(sorted).or_insert((0, corners, mids));
                entry.0 += 1;
            }
        }

        face_count
            .into_values()
            .filter(|(count, _, _)| *count == 1)
            .map(|(_, corners, mids)| (corners, mids))
            .collect()
    }

    /// Extracts the external triangular boundary surface faces of the tetrahedral mesh.
    /// Faces shared by exactly one tetrahedron form the exterior skin.
    ///
    /// For quadratic `TET10` meshes (`is_tet10() == true`), each 6-node exterior face is
    /// subdivided into 4 conforming sub-triangles `[c0, m01, m20], [m01, c1, m12], [m20, m12, c2], [m01, m12, m20]`
    /// so that all exterior corner and mid-edge nodes are represented on the boundary surface.
    pub fn extract_surface_triangles(&self) -> Vec<[usize; 3]> {
        if self.is_tet10() {
            let faces6 = self.extract_surface_tet10_faces();
            let mut tris = Vec::with_capacity(faces6.len() * 4);
            for ([c0, c1, c2], [m01, m12, m20]) in faces6 {
                tris.push([c0, m01, m20]);
                tris.push([m01, c1, m12]);
                tris.push([m20, m12, c2]);
                tris.push([m01, m12, m20]);
            }
            return tris;
        }

        // Map from canonical face (sorted node indices) to original face orientation and count
        let mut face_count: HashMap<[usize; 3], (usize, [usize; 3])> = HashMap::new();

        for elem in &self.elements {
            let [n0, n1, n2, n3] = elem.node_indices;
            let faces = [
                [n0, n1, n2],
                [n0, n2, n3],
                [n0, n3, n1],
                [n1, n3, n2],
            ];

            for face in faces {
                let mut sorted = face;
                sorted.sort_unstable();
                let entry = face_count.entry(sorted).or_insert((0, face));
                entry.0 += 1;
            }
        }

        // Exterior faces have count == 1
        face_count
            .into_values()
            .filter(|(count, _)| *count == 1)
            .map(|(_, face)| face)
            .collect()
    }

    /// Generates a structured tetrahedral mesh of a rectangular box:
    /// `[min[0], max[0]] x [min[1], max[1]] x [min[2], max[2]]`
    /// partitioned into `nx * ny * nz` hexahedral cells, each decomposed into 6 tetrahedra.
    pub fn from_box(min: [f64; 3], max: [f64; 3], nx: usize, ny: usize, nz: usize) -> Result<Self, FeaError> {
        let nx = nx.max(1);
        let ny = ny.max(1);
        let nz = nz.max(1);

        let dx = (max[0] - min[0]) / nx as f64;
        let dy = (max[1] - min[1]) / ny as f64;
        let dz = (max[2] - min[2]) / nz as f64;

        if dx <= 0.0 || dy <= 0.0 || dz <= 0.0 {
            return Err(FeaError::MeshGenerationFailed(
                "Box dimensions must be strictly positive".to_string(),
            ));
        }

        let node_idx = |i: usize, j: usize, k: usize| -> usize {
            i * (ny + 1) * (nz + 1) + j * (nz + 1) + k
        };

        let mut nodes = Vec::with_capacity((nx + 1) * (ny + 1) * (nz + 1));
        for i in 0..=nx {
            let x = min[0] + i as f64 * dx;
            for j in 0..=ny {
                let y = min[1] + j as f64 * dy;
                for k in 0..=nz {
                    let z = min[2] + k as f64 * dz;
                    let id = node_idx(i, j, k);
                    nodes.push(TetNode::new(id, [x, y, z]));
                }
            }
        }

        let mut elements = Vec::with_capacity(nx * ny * nz * 6);
        let mut elem_id = 0;

        for i in 0..nx {
            for j in 0..ny {
                for k in 0..nz {
                    let p000 = node_idx(i, j, k);
                    let p100 = node_idx(i + 1, j, k);
                    let p010 = node_idx(i, j + 1, k);
                    let p110 = node_idx(i + 1, j + 1, k);
                    let p001 = node_idx(i, j, k + 1);
                    let p101 = node_idx(i + 1, j, k + 1);
                    let p011 = node_idx(i, j + 1, k + 1);
                    let p111 = node_idx(i + 1, j + 1, k + 1);

                    // Kuhn/Freudenthal 6-tetrahedron decomposition of the unit cube
                    let tets = [
                        [p000, p100, p110, p111],
                        [p000, p100, p101, p111],
                        [p000, p010, p110, p111],
                        [p000, p010, p011, p111],
                        [p000, p001, p101, p111],
                        [p000, p001, p011, p111],
                    ];

                    for mut tet in tets {
                        // Ensure positive orientation (volume > 0)
                        let v0 = nodes[tet[0]].pos_vec();
                        let v1 = nodes[tet[1]].pos_vec();
                        let v2 = nodes[tet[2]].pos_vec();
                        let v3 = nodes[tet[3]].pos_vec();

                        let det = (v1 - v0).dot(&(v2 - v0).cross(&(v3 - v0)));
                        if det < 0.0 {
                            tet.swap(1, 2);
                        }

                        elements.push(TetElement::new(elem_id, tet));
                        elem_id += 1;
                    }
                }
            }
        }

        Ok(Self::new(nodes, elements))
    }

    /// Generates a tetrahedral mesh from a closed surface `TriangleMesh` with specified density.
    pub fn from_mesh(mesh: &TriangleMesh, density: MeshDensity) -> Result<Self, FeaError> {
        if mesh.vertices.is_empty() || mesh.indices.is_empty() {
            return Err(FeaError::EmptyMesh);
        }

        // Pre-flight mesh topology validation (P3-T2): ensure closed watertight 2-manifold before voxelization
        let validation = mcp_cad_core::validate_mesh(mesh, None);
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

        let aabb = mesh.bounding_box();
        let dims = aabb.dimensions();
        if dims[0] <= 1e-9 || dims[1] <= 1e-9 || dims[2] <= 1e-9 {
            return Err(FeaError::MeshGenerationFailed(
                "Bounding box has zero or degenerate dimensions".to_string(),
            ));
        }

        // Determine target cell size h
        if let MeshDensity::Custom { max_element_size } = density {
            if max_element_size <= 0.0 {
                return Err(FeaError::MeshGenerationFailed(
                    "Custom element size must be > 0".to_string(),
                ));
            }
        }
        let h = density.target_element_size(&aabb);

        let nx = ((dims[0] / h).round() as usize).clamp(2, 60);
        let ny = ((dims[1] / h).round() as usize).clamp(2, 60);
        let nz = ((dims[2] / h).round() as usize).clamp(2, 60);

        // Check if the input mesh is a standard axis-aligned box
        if is_axis_aligned_box(mesh, &aabb) {
            return Self::from_box(aabb.min, aabb.max, nx, ny, nz);
        }

        // For non-box (curved/perforated) shapes, ensure cross-sectional axes have at least 6 voxel cells
        // so circular shafts and holes do not degenerate into a 2x2 square grid (+27% area error).
        let min_curved_cells = match density {
            MeshDensity::Coarse => 8,
            MeshDensity::Medium => 12,
            MeshDensity::Fine => 16,
            MeshDensity::Custom { .. } => 8,
        };
        let max_dim = dims[0].max(dims[1]).max(dims[2]);
        let min_dim = dims[0].min(dims[1]).min(dims[2]);
        // An axis is a thin-plate thickness only if it is the sole minimum dimension and < 25% of max_dim.
        let min_count = dims.iter().filter(|&&d| (d - min_dim).abs() <= 0.15 * min_dim).count();
        let adjust_axis = |n: usize, d: f64| -> usize {
            let is_sole_thin_axis = min_count == 1 && (d - min_dim).abs() <= 1e-9 && d < 0.25 * max_dim;
            if is_sole_thin_axis {
                n.max(2)
            } else {
                n.max(min_curved_cells).min(60)
            }
        };
        let nx = adjust_axis(nx, dims[0]);
        let ny = adjust_axis(ny, dims[1]);
        let nz = adjust_axis(nz, dims[2]);
        let dx = dims[0] / nx as f64;
        let dy = dims[1] / ny as f64;
        let dz = dims[2] / nz as f64;

        // For general shapes: test voxel containment using ray-triangle parity check
        let node_idx = |i: usize, j: usize, k: usize| -> usize {
            i * (ny + 1) * (nz + 1) + j * (nz + 1) + k
        };

        // Precompute all grid node positions
        let total_grid_nodes = (nx + 1) * (ny + 1) * (nz + 1);
        let mut grid_positions = Vec::with_capacity(total_grid_nodes);
        for i in 0..=nx {
            let x = aabb.min[0] + i as f64 * dx;
            for j in 0..=ny {
                let y = aabb.min[1] + j as f64 * dy;
                for k in 0..=nz {
                    let z = aabb.min[2] + k as f64 * dz;
                    grid_positions.push([x, y, z]);
                }
            }
        }

        // Arbitrary ray direction (non-axial to avoid coplanar degenerate hits)
        let ray_dir = Vector3::new(0.577, 0.708, 0.408).normalize();

        // Check which voxels have centers inside the mesh
        let mut kept_tets = Vec::new();
        let mut used_node_indices = HashSet::new();

        for i in 0..nx {
            let cx = aabb.min[0] + (i as f64 + 0.5) * dx;
            for j in 0..ny {
                let cy = aabb.min[1] + (j as f64 + 0.5) * dy;
                for k in 0..nz {
                    let cz = aabb.min[2] + (k as f64 + 0.5) * dz;
                    let center = Vector3::new(cx, cy, cz);

                    if point_inside_mesh(center, ray_dir, mesh) {
                        let p000 = node_idx(i, j, k);
                        let p100 = node_idx(i + 1, j, k);
                        let p010 = node_idx(i, j + 1, k);
                        let p110 = node_idx(i + 1, j + 1, k);
                        let p001 = node_idx(i, j, k + 1);
                        let p101 = node_idx(i + 1, j, k + 1);
                        let p011 = node_idx(i, j + 1, k + 1);
                        let p111 = node_idx(i + 1, j + 1, k + 1);

                        let cell_tets = [
                            [p000, p100, p110, p111],
                            [p000, p100, p101, p111],
                            [p000, p010, p110, p111],
                            [p000, p010, p011, p111],
                            [p000, p001, p101, p111],
                            [p000, p001, p011, p111],
                        ];

                        for mut tet in cell_tets {
                            let v0 = Vector3::from(grid_positions[tet[0]]);
                            let v1 = Vector3::from(grid_positions[tet[1]]);
                            let v2 = Vector3::from(grid_positions[tet[2]]);
                            let v3 = Vector3::from(grid_positions[tet[3]]);

                            let det = (v1 - v0).dot(&(v2 - v0).cross(&(v3 - v0)));
                            if det < 0.0 {
                                tet.swap(1, 2);
                            }

                            for &n in &tet {
                                used_node_indices.insert(n);
                            }
                            kept_tets.push(tet);
                        }
                    }
                }
            }
        }

        if kept_tets.is_empty() {
            // Fallback: If no voxel centers were strictly inside, fallback to AABB mesh
            return Self::from_box(aabb.min, aabb.max, nx, ny, nz);
        }

        // Remap active nodes to compact indices 0..N-1
        let mut old_to_new = HashMap::with_capacity(used_node_indices.len());
        let mut nodes = Vec::with_capacity(used_node_indices.len());

        let mut sorted_used: Vec<usize> = used_node_indices.into_iter().collect();
        sorted_used.sort_unstable();

        for (new_id, old_id) in sorted_used.into_iter().enumerate() {
            old_to_new.insert(old_id, new_id);
            nodes.push(TetNode::new(new_id, grid_positions[old_id]));
        }

        let elements = kept_tets
            .into_iter()
            .enumerate()
            .map(|(elem_id, tet)| {
                let remapped = [
                    old_to_new[&tet[0]],
                    old_to_new[&tet[1]],
                    old_to_new[&tet[2]],
                    old_to_new[&tet[3]],
                ];
                TetElement::new(elem_id, remapped)
            })
            .collect();

        Ok(Self::new(nodes, elements))
    }

    /// Generates a tetrahedral mesh directly from a CAD `Part`.
    ///
    /// If the part's mesh has not been evaluated yet and the parametric DAG contains features,
    /// it evaluates the feature tree with standard tessellation settings.
    pub fn from_part(part: &Part, density: MeshDensity) -> Result<Self, FeaError> {
        let mesh = if part.mesh().indices.is_empty() && !part.dag.nodes.is_empty() {
            let settings = TessellationSettings::default();
            part.dag
                .evaluate_with_settings(&settings)
                .map_err(|e| FeaError::MeshGenerationFailed(e.to_string()))?
        } else {
            part.mesh().clone()
        };
        Self::from_mesh(&mesh, density)
    }

    /// Generates a tetrahedral mesh from a CAD `Part` using adaptive tessellation
    /// tuned to the target FEA mesh density and bounding box.
    ///
    /// Ensures boundary triangles have maximum edge lengths constrained to the target
    /// tetrahedral cell size $h$, producing regular and well-conditioned TET4 elements.
    pub fn from_part_adaptive(part: &mut Part, density: MeshDensity) -> Result<Self, FeaError> {
        let aabb = if !part.dag.nodes.is_empty() && part.mesh().indices.is_empty() {
            let _ = part
                .tessellate_quality(TessellationQuality::Medium)
                .map_err(|e| FeaError::MeshGenerationFailed(e.to_string()))?;
            part.bounding_box()
        } else {
            part.bounding_box()
        };

        let settings = density.to_tessellation_settings(&aabb);
        Self::from_part_with_settings(part, &settings, density)
    }

    /// Generates a tetrahedral mesh from a CAD `Part` evaluating its geometry with explicit `TessellationSettings`.
    pub fn from_part_with_settings(
        part: &mut Part,
        settings: &TessellationSettings,
        density: MeshDensity,
    ) -> Result<Self, FeaError> {
        if !part.dag.nodes.is_empty() {
            let _ = part
                .tessellate(settings)
                .map_err(|e| FeaError::MeshGenerationFailed(e.to_string()))?;
        }
        Self::from_mesh(part.mesh(), density)
    }

    /// Generates a tetrahedral mesh from a CAD `Part` evaluated at a specific `TessellationQuality`.
    pub fn from_part_quality(
        part: &mut Part,
        quality: TessellationQuality,
        density: MeshDensity,
    ) -> Result<Self, FeaError> {
        let settings = TessellationSettings::from(quality);
        Self::from_part_with_settings(part, &settings, density)
    }

    /// Generates a tetrahedral mesh from a surface `TriangleMesh` after optionally subdividing
    /// long edges to maintain tetrahedral regularity.
    pub fn from_mesh_with_edge_limit(
        mesh: &TriangleMesh,
        density: MeshDensity,
        max_edge_length: Option<f64>,
    ) -> Result<Self, FeaError> {
        if let Some(limit) = max_edge_length {
            let refined = refine_mesh_max_edge_length(mesh, limit);
            Self::from_mesh(&refined, density)
        } else {
            Self::from_mesh(mesh, density)
        }
    }
}

/// Robust Möller-Trumbore ray-triangle intersection test.
fn ray_triangle_intersect(
    ray_origin: Vector3<f64>,
    ray_dir: Vector3<f64>,
    v0: Vector3<f64>,
    v1: Vector3<f64>,
    v2: Vector3<f64>,
) -> Option<f64> {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let pvec = ray_dir.cross(&edge2);
    let det = edge1.dot(&pvec);

    if det.abs() < 1e-12 {
        return None;
    }

    let inv_det = 1.0 / det;
    let tvec = ray_origin - v0;
    let u = tvec.dot(&pvec) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let qvec = tvec.cross(&edge1);
    let v = ray_dir.dot(&qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = edge2.dot(&qvec) * inv_det;
    if t > 1e-9 {
        Some(t)
    } else {
        None
    }
}

/// Tests if a point is inside a closed triangular mesh via ray casting parity.
fn point_inside_mesh(point: Vector3<f64>, ray_dir: Vector3<f64>, mesh: &TriangleMesh) -> bool {
    let mut hit_count = 0;
    for tri in &mesh.indices {
        let v0 = mesh.vertices[tri[0] as usize].pos_vec();
        let v1 = mesh.vertices[tri[1] as usize].pos_vec();
        let v2 = mesh.vertices[tri[2] as usize].pos_vec();

        if ray_triangle_intersect(point, ray_dir, v0, v1, v2).is_some() {
            hit_count += 1;
        }
    }
    hit_count % 2 == 1
}

/// Checks whether a triangle mesh is an axis-aligned box matching `aabb`.
fn is_axis_aligned_box(mesh: &TriangleMesh, aabb: &BoundingBox) -> bool {
    if mesh.indices.len() != 12 {
        return false;
    }
    let tol = 1e-5;
    for v in &mesh.vertices {
        let on_x = (v.position[0] - aabb.min[0]).abs() <= tol || (v.position[0] - aabb.max[0]).abs() <= tol;
        let on_y = (v.position[1] - aabb.min[1]).abs() <= tol || (v.position[1] - aabb.max[1]).abs() <= tol;
        let on_z = (v.position[2] - aabb.min[2]).abs() <= tol || (v.position[2] - aabb.max[2]).abs() <= tol;
        if !on_x || !on_y || !on_z {
            return false;
        }
    }
    true
}

/// Calculates the maximum edge length across all triangles in `mesh`.
pub fn mesh_max_edge_length(mesh: &TriangleMesh) -> f64 {
    let mut max_len_sq = 0.0f64;
    for tri in &mesh.indices {
        let p0 = &mesh.vertices[tri[0] as usize].position;
        let p1 = &mesh.vertices[tri[1] as usize].position;
        let p2 = &mesh.vertices[tri[2] as usize].position;

        let d01 = (p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) + (p1[2] - p0[2]).powi(2);
        let d12 = (p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2) + (p2[2] - p1[2]).powi(2);
        let d20 = (p0[0] - p2[0]).powi(2) + (p0[1] - p2[1]).powi(2) + (p0[2] - p2[2]).powi(2);

        max_len_sq = max_len_sq.max(d01).max(d12).max(d20);
    }
    max_len_sq.sqrt()
}

/// Conforming 1-to-4 red refinement of triangular surface mesh
/// so that maximum edge length does not exceed `max_edge_len`.
/// Clamps to 3 subdivision passes to avoid polygon explosion.
pub fn refine_mesh_max_edge_length(mesh: &TriangleMesh, max_edge_len: f64) -> TriangleMesh {
    if max_edge_len <= 1e-9 || mesh.vertices.is_empty() || mesh.indices.is_empty() {
        return mesh.clone();
    }

    let mut current = mesh.clone();
    for _ in 0..3 {
        if mesh_max_edge_length(&current) <= max_edge_len {
            break;
        }

        let mut new_vertices = current.vertices.clone();
        let mut midpoint_map: HashMap<(u32, u32), u32> = HashMap::new();

        let mut get_or_create_midpoint = |a: u32, b: u32, verts: &mut Vec<Vertex>| -> u32 {
            let key = if a < b { (a, b) } else { (b, a) };
            if let Some(&idx) = midpoint_map.get(&key) {
                idx
            } else {
                let va = &verts[a as usize];
                let vb = &verts[b as usize];
                let mid_pos = [
                    (va.position[0] + vb.position[0]) * 0.5,
                    (va.position[1] + vb.position[1]) * 0.5,
                    (va.position[2] + vb.position[2]) * 0.5,
                ];
                let nx = va.normal[0] + vb.normal[0];
                let ny = va.normal[1] + vb.normal[1];
                let nz = va.normal[2] + vb.normal[2];
                let len = (nx * nx + ny * ny + nz * nz).sqrt();
                let mid_norm = if len > 1e-12 {
                    [nx / len, ny / len, nz / len]
                } else {
                    va.normal
                };
                let idx = verts.len() as u32;
                verts.push(Vertex::new(mid_pos, mid_norm));
                midpoint_map.insert(key, idx);
                idx
            }
        };

        let has_face_ids =
            current.face_ids.len() == current.indices.len() && !current.face_ids.is_empty();
        let mut new_indices = Vec::with_capacity(current.indices.len() * 4);
        let mut new_face_ids = if has_face_ids {
            Vec::with_capacity(current.indices.len() * 4)
        } else {
            Vec::new()
        };

        for (tri_idx, tri) in current.indices.iter().enumerate() {
            let [i0, i1, i2] = *tri;
            let m01 = get_or_create_midpoint(i0, i1, &mut new_vertices);
            let m12 = get_or_create_midpoint(i1, i2, &mut new_vertices);
            let m20 = get_or_create_midpoint(i2, i0, &mut new_vertices);

            new_indices.push([i0, m01, m20]);
            new_indices.push([m01, i1, m12]);
            new_indices.push([m20, m12, i2]);
            new_indices.push([m01, m12, m20]);

            if has_face_ids {
                let fid = current.face_ids[tri_idx];
                new_face_ids.extend_from_slice(&[fid, fid, fid, fid]);
            }
        }

        current = if has_face_ids {
            TriangleMesh::with_faces(new_vertices, new_indices, new_face_ids, current.faces.clone())
        } else {
            TriangleMesh::new(new_vertices, new_indices)
        };
    }

    current
}
