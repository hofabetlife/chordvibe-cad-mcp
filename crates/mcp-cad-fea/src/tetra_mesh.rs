//! 3D Tetrahedral mesh generation, representation, and topological utilities.

use std::collections::{HashMap, HashSet};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use mcp_cad_core::part::Part;
use mcp_cad_core::topology::{BoundingBox, TriangleMesh};
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

/// An unstructured 3D tetrahedral finite element mesh.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TetrahedralMesh {
    /// Array of mesh nodes.
    pub nodes: Vec<TetNode>,
    /// Array of tetrahedral elements.
    pub elements: Vec<TetElement>,
}

impl TetrahedralMesh {
    /// Creates a new tetrahedral mesh from nodes and elements.
    pub fn new(nodes: Vec<TetNode>, elements: Vec<TetElement>) -> Self {
        Self { nodes, elements }
    }

    /// Creates an empty mesh.
    pub fn empty() -> Self {
        Self {
            nodes: Vec::new(),
            elements: Vec::new(),
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

    /// Extracts the external triangular boundary surface faces of the tetrahedral mesh.
    /// Faces shared by exactly one tetrahedron form the exterior skin.
    pub fn extract_surface_triangles(&self) -> Vec<[usize; 3]> {
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

        let aabb = mesh.bounding_box();
        let dims = aabb.dimensions();
        if dims[0] <= 1e-9 || dims[1] <= 1e-9 || dims[2] <= 1e-9 {
            return Err(FeaError::MeshGenerationFailed(
                "Bounding box has zero or degenerate dimensions".to_string(),
            ));
        }

        // Determine target cell size h
        let max_dim = dims[0].max(dims[1]).max(dims[2]);
        let min_dim = dims[0].min(dims[1]).min(dims[2]);

        let h = match density {
            MeshDensity::Coarse => (max_dim / 10.0).min(min_dim / 2.0).max(1e-4),
            MeshDensity::Medium => (max_dim / 18.0).min(min_dim / 4.0).max(1e-4),
            MeshDensity::Fine => (max_dim / 28.0).min(min_dim / 6.0).max(1e-4),
            MeshDensity::Custom { max_element_size } => {
                if max_element_size <= 0.0 {
                    return Err(FeaError::MeshGenerationFailed(
                        "Custom element size must be > 0".to_string(),
                    ));
                }
                max_element_size
            }
        };

        let nx = ((dims[0] / h).round() as usize).clamp(2, 60);
        let ny = ((dims[1] / h).round() as usize).clamp(2, 60);
        let nz = ((dims[2] / h).round() as usize).clamp(2, 60);

        let dx = dims[0] / nx as f64;
        let dy = dims[1] / ny as f64;
        let dz = dims[2] / nz as f64;

        // Check if the input mesh is a standard axis-aligned box
        if is_axis_aligned_box(mesh, &aabb) {
            return Self::from_box(aabb.min, aabb.max, nx, ny, nz);
        }

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
    pub fn from_part(part: &Part, density: MeshDensity) -> Result<Self, FeaError> {
        Self::from_mesh(&part.mesh, density)
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
