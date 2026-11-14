//! Boundary conditions (Dirichlet kinematic constraints and Neumann mechanical loads).

use std::collections::{HashMap, HashSet};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use mcp_cad_core::topology::TriangleMesh;
use crate::error::FeaError;
use crate::tetra_mesh::TetrahedralMesh;

fn default_tolerance() -> f64 {
    1e-4
}

/// Degrees of freedom selection for constraints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum DofComponent {
    /// Constrain all 3 translational components: [ux, uy, uz]
    #[default]
    All,
    /// Constrain only X translation: ux
    X,
    /// Constrain only Y translation: uy
    Y,
    /// Constrain only Z translation: uz
    Z,
    /// Constrain X and Y translations: [ux, uy]
    XY,
    /// Constrain X and Z translations: [ux, uz]
    XZ,
    /// Constrain Y and Z translations: [uy, uz]
    YZ,
}

/// Axis-aligned bounding box faces for boundary selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AabbFace {
    /// Face at x = x_min
    MinX,
    /// Face at x = x_max
    MaxX,
    /// Face at y = y_min
    MinY,
    /// Face at y = y_max
    MaxY,
    /// Face at z = z_min
    MinZ,
    /// Face at z = z_max
    MaxZ,
}

/// Dirichlet boundary constraint (prescribed displacement).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DirichletConstraint {
    /// Fix nodes near a plane: |(p - origin) · normal| <= tolerance
    FixedPlane {
        /// Point on the constraint plane
        origin: [f64; 3],
        /// Normal vector of the constraint plane
        normal: [f64; 3],
        /// Search distance tolerance in meters
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        /// Constrained displacement components
        #[serde(default)]
        components: DofComponent,
    },
    /// Fix nodes lying on an AABB face
    FixedAabbFace {
        /// Target AABB face
        face: AabbFace,
        /// Distance tolerance in meters
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        /// Constrained displacement components
        #[serde(default)]
        components: DofComponent,
    },
    /// Fix surface nodes matching a named CAD surface (represented by its triangles in meters).
    FixedSurfaceTriangles {
        /// Optional logical face name for diagnostics
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face_name: Option<String>,
        /// Triangles `[v0, v1, v2]` in meters forming the CAD face
        triangles: Vec<[[f64; 3]; 3]>,
        /// Search distance tolerance in meters (auto-scaled to voxel size if <= 1e-4)
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        /// Constrained displacement components
        #[serde(default)]
        components: DofComponent,
    },
    /// Fix an explicit list of node indices
    FixedNodes {
        /// Node IDs to constrain
        node_ids: Vec<usize>,
        /// Constrained displacement components
        #[serde(default)]
        components: DofComponent,
    },
    /// Prescribe a specific DOF index with an exact displacement value
    FixedDof {
        /// Global degree of freedom index
        dof_index: usize,
        /// Prescribed displacement in meters
        prescribed_value: f64,
    },
}

/// Neumann boundary load (applied forces, pressures, and moments).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum NeumannLoad {
    /// Concentrated force applied to a specific node
    NodalForce {
        /// Target node ID
        node_id: usize,
        /// Applied force vector [Fx, Fy, Fz] in Newtons
        force: [f64; 3],
    },
    /// Total force distributed evenly over surface nodes near a plane
    AreaForce {
        /// Point on the plane
        origin: [f64; 3],
        /// Normal vector of the plane
        normal: [f64; 3],
        /// Search tolerance in meters
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        /// Total resultant force [Fx, Fy, Fz] in Newtons
        total_force: [f64; 3],
    },
    /// Total force distributed over surface nodes on an AABB face
    AabbFaceForce {
        /// Target AABB face
        face: AabbFace,
        /// Search tolerance in meters
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        /// Total resultant force [Fx, Fy, Fz] in Newtons
        total_force: [f64; 3],
    },
    /// Total force distributed proportionally to tributary areas over surface nodes matching a CAD face.
    SurfaceTrianglesForce {
        /// Optional logical face name for diagnostics
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face_name: Option<String>,
        /// Triangles `[v0, v1, v2]` in meters forming the CAD face
        triangles: Vec<[[f64; 3]; 3]>,
        /// Search distance tolerance in meters
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        /// Total resultant force [Fx, Fy, Fz] in Newtons
        total_force: [f64; 3],
    },
    /// Normal pressure (in Pascals, Pa) applied over surface nodes matching a CAD face.
    /// Positive pressure acts inward against the outward surface normal (`-n`),
    /// negative pressure acts outward along `+n`.
    SurfaceTrianglesPressure {
        /// Optional logical face name for diagnostics
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face_name: Option<String>,
        /// Triangles `[v0, v1, v2]` in meters with outward orientation forming the CAD face
        triangles: Vec<[[f64; 3]; 3]>,
        /// Search distance tolerance in meters
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        /// Normal pressure in Pascals (Pa = N/m²)
        pressure_pa: f64,
    },
    /// Resultant torque / bending moment [Mx, My, Mz] (in N·m) distributed over surface nodes matching a CAD face.
    SurfaceTrianglesMoment {
        /// Optional logical face name for diagnostics
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face_name: Option<String>,
        /// Triangles `[v0, v1, v2]` in meters forming the CAD face
        triangles: Vec<[[f64; 3]; 3]>,
        /// Search distance tolerance in meters
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        /// Resultant moment vector [Mx, My, Mz] in Newton-meters (N·m)
        moment_nm: [f64; 3],
    },
}

/// Complete collection of Dirichlet constraints and Neumann loads.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct BoundaryConditions {
    /// List of kinematic constraints
    pub dirichlet: Vec<DirichletConstraint>,
    /// List of applied mechanical loads
    pub neumann: Vec<NeumannLoad>,
}

/// Matched tetrahedral boundary node with its tributary area weight and outward CAD normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchedSurfaceNode {
    /// Node index in `TetrahedralMesh::nodes`.
    pub node_id: usize,
    /// Tributary area weight in m² on the matched boundary surface.
    pub area_weight: f64,
    /// Unit outward normal vector of the closest CAD triangle.
    pub outward_normal: [f64; 3],
}

/// Extracts all triangles `[[v0, v1, v2]]` (in meters) belonging to any of the specified `face_ids`.
pub fn extract_face_triangles(mesh: &TriangleMesh, face_ids: &[u32]) -> Vec<[[f64; 3]; 3]> {
    if mesh.face_ids.len() != mesh.indices.len() || face_ids.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::new();
    for (t_idx, &[i0, i1, i2]) in mesh.indices.iter().enumerate() {
        let fid = mesh.face_ids[t_idx];
        if face_ids.contains(&fid) {
            let p0 = mesh.vertices[i0 as usize].position;
            let p1 = mesh.vertices[i1 as usize].position;
            let p2 = mesh.vertices[i2 as usize].position;
            out.push([p0, p1, p2]);
        }
    }
    out
}

impl BoundaryConditions {
    /// Creates an empty set of boundary conditions.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a plane constraint fixing [ux, uy, uz] = 0.
    pub fn fix_plane(&mut self, origin: [f64; 3], normal: [f64; 3], tolerance: f64) -> &mut Self {
        self.dirichlet.push(DirichletConstraint::FixedPlane {
            origin,
            normal,
            tolerance,
            components: DofComponent::All,
        });
        self
    }

    /// Adds an AABB face constraint fixing [ux, uy, uz] = 0.
    pub fn fix_aabb_face(&mut self, face: AabbFace, tolerance: f64) -> &mut Self {
        self.dirichlet.push(DirichletConstraint::FixedAabbFace {
            face,
            tolerance,
            components: DofComponent::All,
        });
        self
    }

    /// Adds a surface triangles constraint fixing the specified `components` to 0.
    pub fn fix_surface_triangles(
        &mut self,
        face_name: Option<String>,
        triangles: Vec<[[f64; 3]; 3]>,
        tolerance: f64,
        components: DofComponent,
    ) -> &mut Self {
        self.dirichlet.push(DirichletConstraint::FixedSurfaceTriangles {
            face_name,
            triangles,
            tolerance,
            components,
        });
        self
    }

    /// Adds a constraint on a named face from `mesh` by `face_id`.
    pub fn fix_named_face(
        &mut self,
        mesh: &TriangleMesh,
        face_id: u32,
        face_name: Option<String>,
        tolerance: f64,
        components: DofComponent,
    ) -> &mut Self {
        let triangles = extract_face_triangles(mesh, &[face_id]);
        self.fix_surface_triangles(face_name, triangles, tolerance, components)
    }

    /// Adds a list of node IDs fixing [ux, uy, uz] = 0.
    pub fn fix_nodes(&mut self, node_ids: Vec<usize>) -> &mut Self {
        self.dirichlet.push(DirichletConstraint::FixedNodes {
            node_ids,
            components: DofComponent::All,
        });
        self
    }

    /// Adds a concentrated nodal force [Fx, Fy, Fz] in Newtons.
    pub fn add_nodal_force(&mut self, node_id: usize, force: [f64; 3]) -> &mut Self {
        self.neumann.push(NeumannLoad::NodalForce { node_id, force });
        self
    }

    /// Adds a distributed force over surface nodes near a plane.
    pub fn add_area_force(
        &mut self,
        origin: [f64; 3],
        normal: [f64; 3],
        tolerance: f64,
        total_force: [f64; 3],
    ) -> &mut Self {
        self.neumann.push(NeumannLoad::AreaForce {
            origin,
            normal,
            tolerance,
            total_force,
        });
        self
    }

    /// Adds a distributed force over nodes on an AABB face.
    pub fn add_aabb_face_force(
        &mut self,
        face: AabbFace,
        tolerance: f64,
        total_force: [f64; 3],
    ) -> &mut Self {
        self.neumann.push(NeumannLoad::AabbFaceForce {
            face,
            tolerance,
            total_force,
        });
        self
    }

    /// Adds a distributed resultant force [Fx, Fy, Fz] (in Newtons) over surface nodes matching CAD `triangles`.
    pub fn add_surface_triangles_force(
        &mut self,
        face_name: Option<String>,
        triangles: Vec<[[f64; 3]; 3]>,
        tolerance: f64,
        total_force: [f64; 3],
    ) -> &mut Self {
        self.neumann.push(NeumannLoad::SurfaceTrianglesForce {
            face_name,
            triangles,
            tolerance,
            total_force,
        });
        self
    }

    /// Adds a distributed resultant force [Fx, Fy, Fz] (in Newtons) over a named face from `mesh` by `face_id`.
    pub fn add_named_face_force(
        &mut self,
        mesh: &TriangleMesh,
        face_id: u32,
        face_name: Option<String>,
        tolerance: f64,
        total_force: [f64; 3],
    ) -> &mut Self {
        let triangles = extract_face_triangles(mesh, &[face_id]);
        self.add_surface_triangles_force(face_name, triangles, tolerance, total_force)
    }

    /// Adds a normal pressure load (in Pascals, Pa) over surface nodes matching CAD `triangles`.
    pub fn add_surface_triangles_pressure(
        &mut self,
        face_name: Option<String>,
        triangles: Vec<[[f64; 3]; 3]>,
        tolerance: f64,
        pressure_pa: f64,
    ) -> &mut Self {
        self.neumann.push(NeumannLoad::SurfaceTrianglesPressure {
            face_name,
            triangles,
            tolerance,
            pressure_pa,
        });
        self
    }

    /// Adds a resultant torque / bending moment [Mx, My, Mz] (in N·m) over surface nodes matching CAD `triangles`.
    pub fn add_surface_triangles_moment(
        &mut self,
        face_name: Option<String>,
        triangles: Vec<[[f64; 3]; 3]>,
        tolerance: f64,
        moment_nm: [f64; 3],
    ) -> &mut Self {
        self.neumann.push(NeumannLoad::SurfaceTrianglesMoment {
            face_name,
            triangles,
            tolerance,
            moment_nm,
        });
        self
    }

    /// Resolves high-level boundary conditions onto actual mesh node DOFs and nodal force vectors.
    pub fn resolve(&self, mesh: &TetrahedralMesh) -> Result<ResolvedBoundary, FeaError> {
        let total_nodes = mesh.node_count();
        if total_nodes == 0 {
            return Err(FeaError::EmptyMesh);
        }

        let aabb = mesh.bounding_box();
        let mut fixed_dofs: HashMap<usize, f64> = HashMap::new();

        let add_node_constraint = |dofs: &mut HashMap<usize, f64>,
                                   node_id: usize,
                                   comp: DofComponent,
                                   val: f64| {
            match comp {
                DofComponent::All => {
                    dofs.insert(3 * node_id, val);
                    dofs.insert(3 * node_id + 1, val);
                    dofs.insert(3 * node_id + 2, val);
                }
                DofComponent::X => {
                    dofs.insert(3 * node_id, val);
                }
                DofComponent::Y => {
                    dofs.insert(3 * node_id + 1, val);
                }
                DofComponent::Z => {
                    dofs.insert(3 * node_id + 2, val);
                }
                DofComponent::XY => {
                    dofs.insert(3 * node_id, val);
                    dofs.insert(3 * node_id + 1, val);
                }
                DofComponent::XZ => {
                    dofs.insert(3 * node_id, val);
                    dofs.insert(3 * node_id + 2, val);
                }
                DofComponent::YZ => {
                    dofs.insert(3 * node_id + 1, val);
                    dofs.insert(3 * node_id + 2, val);
                }
            }
        };

        // Resolve Dirichlet constraints
        for constraint in &self.dirichlet {
            match constraint {
                DirichletConstraint::FixedPlane {
                    origin,
                    normal,
                    tolerance,
                    components,
                } => {
                    let nodes = mesh.find_nodes_in_plane(*origin, *normal, *tolerance);
                    if nodes.is_empty() {
                        return Err(FeaError::NoBoundaryNodesMatched {
                            tolerance: *tolerance,
                        });
                    }
                    for n in nodes {
                        add_node_constraint(&mut fixed_dofs, n, *components, 0.0);
                    }
                }
                DirichletConstraint::FixedAabbFace {
                    face,
                    tolerance,
                    components,
                } => {
                    let nodes = find_aabb_face_nodes(mesh, &aabb, *face, *tolerance);
                    if nodes.is_empty() {
                        return Err(FeaError::NoBoundaryNodesMatched {
                            tolerance: *tolerance,
                        });
                    }
                    for n in nodes {
                        add_node_constraint(&mut fixed_dofs, n, *components, 0.0);
                    }
                }
                DirichletConstraint::FixedSurfaceTriangles {
                    triangles,
                    tolerance,
                    components,
                    ..
                } => {
                    let matched = match_surface_nodes_and_weights(mesh, triangles, *tolerance);
                    if matched.is_empty() {
                        return Err(FeaError::NoBoundaryNodesMatched {
                            tolerance: *tolerance,
                        });
                    }
                    for m in matched {
                        add_node_constraint(&mut fixed_dofs, m.node_id, *components, 0.0);
                    }
                }
                DirichletConstraint::FixedNodes {
                    node_ids,
                    components,
                } => {
                    for &n in node_ids {
                        if n >= total_nodes {
                            return Err(FeaError::NodeOutOfBounds {
                                node_id: n,
                                total_nodes,
                            });
                        }
                        add_node_constraint(&mut fixed_dofs, n, *components, 0.0);
                    }
                }
                DirichletConstraint::FixedDof {
                    dof_index,
                    prescribed_value,
                } => {
                    if *dof_index >= 3 * total_nodes {
                        return Err(FeaError::NodeOutOfBounds {
                            node_id: *dof_index / 3,
                            total_nodes,
                        });
                    }
                    fixed_dofs.insert(*dof_index, *prescribed_value);
                }
            }
        }

        if fixed_dofs.is_empty() {
            return Err(FeaError::UnconstrainedModel);
        }

        // Resolve Neumann forces
        let mut nodal_forces = vec![[0.0, 0.0, 0.0]; total_nodes];

        for load in &self.neumann {
            match load {
                NeumannLoad::NodalForce { node_id, force } => {
                    if *node_id >= total_nodes {
                        return Err(FeaError::NodeOutOfBounds {
                            node_id: *node_id,
                            total_nodes,
                        });
                    }
                    nodal_forces[*node_id][0] += force[0];
                    nodal_forces[*node_id][1] += force[1];
                    nodal_forces[*node_id][2] += force[2];
                }
                NeumannLoad::AreaForce {
                    origin,
                    normal,
                    tolerance,
                    total_force,
                } => {
                    let nodes = mesh.find_nodes_in_plane(*origin, *normal, *tolerance);
                    if nodes.is_empty() {
                        return Err(FeaError::NoBoundaryNodesMatched {
                            tolerance: *tolerance,
                        });
                    }
                    let count = nodes.len() as f64;
                    let f_per_node = [
                        total_force[0] / count,
                        total_force[1] / count,
                        total_force[2] / count,
                    ];
                    for n in nodes {
                        nodal_forces[n][0] += f_per_node[0];
                        nodal_forces[n][1] += f_per_node[1];
                        nodal_forces[n][2] += f_per_node[2];
                    }
                }
                NeumannLoad::AabbFaceForce {
                    face,
                    tolerance,
                    total_force,
                } => {
                    let nodes = find_aabb_face_nodes(mesh, &aabb, *face, *tolerance);
                    if nodes.is_empty() {
                        return Err(FeaError::NoBoundaryNodesMatched {
                            tolerance: *tolerance,
                        });
                    }

                    // For quadratic TET10 meshes, distribute surface traction using consistent
                    // 6-node triangle virtual-work weights (int_Delta N_corner dA = 0, int_Delta N_mid dA = A_Delta / 3).
                    if mesh.is_tet10() {
                        let node_set: HashSet<usize> = nodes.iter().copied().collect();
                        let mut mid_weights: HashMap<usize, f64> = HashMap::new();
                        let mut total_w = 0.0;
                        for (corners, mids) in mesh.extract_surface_tet10_faces() {
                            if corners.iter().all(|c| node_set.contains(c))
                                && mids.iter().all(|m| node_set.contains(m))
                            {
                                let (area, _) = triangle_area_and_normal(
                                    mesh.nodes[corners[0]].position,
                                    mesh.nodes[corners[1]].position,
                                    mesh.nodes[corners[2]].position,
                                );
                                if area > 1e-18 {
                                    let share = area / 3.0;
                                    for m in mids {
                                        *mid_weights.entry(m).or_insert(0.0) += share;
                                    }
                                    total_w += area;
                                }
                            }
                        }
                        if total_w > 1e-18 {
                            for (m_id, w) in mid_weights {
                                let frac = w / total_w;
                                nodal_forces[m_id][0] += total_force[0] * frac;
                                nodal_forces[m_id][1] += total_force[1] * frac;
                                nodal_forces[m_id][2] += total_force[2] * frac;
                            }
                            continue;
                        }
                    }

                    // Linear TET4 tributary area weights:
                    // Corner nodes = 0.25, Edge nodes = 0.5, Interior nodes = 1.0
                    let (u_min, u_max, v_min, v_max) = match face {
                        AabbFace::MinX | AabbFace::MaxX => {
                            (aabb.min[1], aabb.max[1], aabb.min[2], aabb.max[2])
                        }
                        AabbFace::MinY | AabbFace::MaxY => {
                            (aabb.min[0], aabb.max[0], aabb.min[2], aabb.max[2])
                        }
                        AabbFace::MinZ | AabbFace::MaxZ => {
                            (aabb.min[0], aabb.max[0], aabb.min[1], aabb.max[1])
                        }
                    };

                    let tol = *tolerance;
                    let mut weights = Vec::with_capacity(nodes.len());
                    let mut total_weight = 0.0;

                    for &n in &nodes {
                        let p = mesh.nodes[n].position;
                        let (u, v) = match face {
                            AabbFace::MinX | AabbFace::MaxX => (p[1], p[2]),
                            AabbFace::MinY | AabbFace::MaxY => (p[0], p[2]),
                            AabbFace::MinZ | AabbFace::MaxZ => (p[0], p[1]),
                        };
                        let wu = if (u - u_min).abs() <= tol || (u - u_max).abs() <= tol {
                            0.5
                        } else {
                            1.0
                        };
                        let wv = if (v - v_min).abs() <= tol || (v - v_max).abs() <= tol {
                            0.5
                        } else {
                            1.0
                        };
                        let w = wu * wv;
                        weights.push(w);
                        total_weight += w;
                    }

                    if total_weight > 1e-12 {
                        for (idx, &n) in nodes.iter().enumerate() {
                            let frac = weights[idx] / total_weight;
                            nodal_forces[n][0] += total_force[0] * frac;
                            nodal_forces[n][1] += total_force[1] * frac;
                            nodal_forces[n][2] += total_force[2] * frac;
                        }
                    } else {
                        let count = nodes.len() as f64;
                        for n in nodes {
                            nodal_forces[n][0] += total_force[0] / count;
                            nodal_forces[n][1] += total_force[1] / count;
                            nodal_forces[n][2] += total_force[2] / count;
                        }
                    }
                }
                NeumannLoad::SurfaceTrianglesForce {
                    triangles,
                    tolerance,
                    total_force,
                    ..
                } => {
                    let matched = match_surface_nodes_and_weights(mesh, triangles, *tolerance);
                    if matched.is_empty() {
                        return Err(FeaError::NoBoundaryNodesMatched {
                            tolerance: *tolerance,
                        });
                    }
                    let total_weight: f64 = matched.iter().map(|m| m.area_weight).sum();
                    if total_weight > 1e-18 {
                        for m in &matched {
                            let frac = m.area_weight / total_weight;
                            nodal_forces[m.node_id][0] += total_force[0] * frac;
                            nodal_forces[m.node_id][1] += total_force[1] * frac;
                            nodal_forces[m.node_id][2] += total_force[2] * frac;
                        }
                    } else {
                        let count = matched.len() as f64;
                        for m in &matched {
                            nodal_forces[m.node_id][0] += total_force[0] / count;
                            nodal_forces[m.node_id][1] += total_force[1] / count;
                            nodal_forces[m.node_id][2] += total_force[2] / count;
                        }
                    }
                }
                NeumannLoad::SurfaceTrianglesPressure {
                    triangles,
                    tolerance,
                    pressure_pa,
                    ..
                } => {
                    let matched = match_surface_nodes_and_weights(mesh, triangles, *tolerance);
                    if matched.is_empty() {
                        return Err(FeaError::NoBoundaryNodesMatched {
                            tolerance: *tolerance,
                        });
                    }
                    // Total CAD surface area of the face
                    let cad_area: f64 = triangles
                        .iter()
                        .map(|t| triangle_area_and_normal(t[0], t[1], t[2]).0)
                        .sum();
                    let total_weight: f64 = matched.iter().map(|m| m.area_weight).sum();
                    let area_scale = if total_weight > 1e-18 && cad_area > 1e-18 {
                        cad_area / total_weight
                    } else {
                        1.0
                    };

                    for m in &matched {
                        // Positive pressure acts inward against outward surface normal (-n)
                        let eff_area = m.area_weight * area_scale;
                        let mag = -(*pressure_pa) * eff_area;
                        nodal_forces[m.node_id][0] += mag * m.outward_normal[0];
                        nodal_forces[m.node_id][1] += mag * m.outward_normal[1];
                        nodal_forces[m.node_id][2] += mag * m.outward_normal[2];
                    }
                }
                NeumannLoad::SurfaceTrianglesMoment {
                    triangles,
                    tolerance,
                    moment_nm,
                    ..
                } => {
                    let matched = match_surface_nodes_and_weights(mesh, triangles, *tolerance);
                    if matched.is_empty() {
                        return Err(FeaError::NoBoundaryNodesMatched {
                            tolerance: *tolerance,
                        });
                    }
                    let m_vec = Vector3::new(moment_nm[0], moment_nm[1], moment_nm[2]);
                    let m_mag = m_vec.norm();
                    if m_mag > 1e-15 {
                        let m_axis = m_vec / m_mag;
                        let total_weight: f64 = matched.iter().map(|m| m.area_weight).sum();
                        let inv_w = if total_weight > 1e-18 {
                            1.0 / total_weight
                        } else {
                            1.0 / (matched.len() as f64)
                        };
                        let mut centroid = Vector3::zeros();
                        for m in &matched {
                            let w = if total_weight > 1e-18 { m.area_weight } else { 1.0 };
                            centroid += mesh.nodes[m.node_id].pos_vec() * (w * inv_w);
                        }

                        // Compute polar inertia sum I_m = sum(w_i * |m_axis x r_i|^2)
                        let mut inertia_sum = 0.0;
                        let mut arms = Vec::with_capacity(matched.len());
                        for m in &matched {
                            let w = if total_weight > 1e-18 { m.area_weight } else { 1.0 };
                            let r = mesh.nodes[m.node_id].pos_vec() - centroid;
                            let d = m_axis.cross(&r);
                            inertia_sum += w * d.norm_squared();
                            arms.push((m.node_id, w, d));
                        }

                        if inertia_sum > 1e-18 {
                            let scale = m_mag / inertia_sum;
                            for (node_id, w, d) in arms {
                                let f = d * (w * scale);
                                nodal_forces[node_id][0] += f.x;
                                nodal_forces[node_id][1] += f.y;
                                nodal_forces[node_id][2] += f.z;
                            }
                        }
                    }
                }
            }
        }

        Ok(ResolvedBoundary {
            fixed_dofs,
            nodal_forces,
        })
    }
}

/// Resolved boundary conditions ready for linear solver assembly.
#[derive(Debug, Clone)]
pub struct ResolvedBoundary {
    /// Map of global DOF index -> prescribed displacement value
    pub fixed_dofs: HashMap<usize, f64>,
    /// Accumulated external forces per node [Fx, Fy, Fz]
    pub nodal_forces: Vec<[f64; 3]>,
}

fn find_aabb_face_nodes(
    mesh: &TetrahedralMesh,
    aabb: &mcp_cad_core::topology::BoundingBox,
    face: AabbFace,
    tolerance: f64,
) -> Vec<usize> {
    mesh.nodes
        .iter()
        .filter(|node| {
            let p = node.position;
            match face {
                AabbFace::MinX => (p[0] - aabb.min[0]).abs() <= tolerance,
                AabbFace::MaxX => (p[0] - aabb.max[0]).abs() <= tolerance,
                AabbFace::MinY => (p[1] - aabb.min[1]).abs() <= tolerance,
                AabbFace::MaxY => (p[1] - aabb.max[1]).abs() <= tolerance,
                AabbFace::MinZ => (p[2] - aabb.min[2]).abs() <= tolerance,
                AabbFace::MaxZ => (p[2] - aabb.max[2]).abs() <= tolerance,
            }
        })
        .map(|node| node.id)
        .collect()
}

fn triangle_area_and_normal(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> (f64, [f64; 3]) {
    let va = Vector3::from(a);
    let vb = Vector3::from(b);
    let vc = Vector3::from(c);
    let cross = (vb - va).cross(&(vc - va));
    let norm = cross.norm();
    if norm > 1e-18 {
        let n = cross / norm;
        (0.5 * norm, [n.x, n.y, n.z])
    } else {
        (0.0, [0.0, 0.0, 1.0])
    }
}

/// Computes the exact 3D Euclidean distance from point `p` to triangle `(a, b, c)`.
pub fn point_to_triangle_distance_3d(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let vp = Vector3::from(p);
    let va = Vector3::from(a);
    let vb = Vector3::from(b);
    let vc = Vector3::from(c);

    let ab = vb - va;
    let ac = vc - va;
    let ap = vp - va;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (vp - va).norm();
    }

    let bp = vp - vb;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (vp - vb).norm();
    }

    let vc_region = d1 * d4 - d3 * d2;
    if vc_region <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        let v = if denom.abs() > 1e-18 { d1 / denom } else { 0.0 };
        return (vp - (va + ab * v)).norm();
    }

    let cp = vp - vc;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (vp - vc).norm();
    }

    let vb_region = d5 * d2 - d1 * d6;
    if vb_region <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        let w = if denom.abs() > 1e-18 { d2 / denom } else { 0.0 };
        return (vp - (va + ac * w)).norm();
    }

    let va_region = d3 * d6 - d5 * d4;
    if va_region <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let denom = (d4 - d3) + (d5 - d6);
        let w = if denom.abs() > 1e-18 {
            (d4 - d3) / denom
        } else {
            0.0
        };
        return (vp - (vb + (vc - vb) * w)).norm();
    }

    let denom = va_region + vb_region + vc_region;
    if denom.abs() > 1e-18 {
        let v = vb_region / denom;
        let w = vc_region / denom;
        (vp - (va + ab * v + ac * w)).norm()
    } else {
        (vp - va).norm()
    }
}

/// Estimates the voxel grid steps `[dx, dy, dz]` of `mesh` from its tetrahedral edges.
fn estimate_voxel_steps(mesh: &TetrahedralMesh) -> [f64; 3] {
    let mut min_step = [f64::INFINITY; 3];
    for elem in &mesh.elements {
        let idx = elem.node_indices;
        let edges = [
            (idx[0], idx[1]),
            (idx[0], idx[2]),
            (idx[0], idx[3]),
            (idx[1], idx[2]),
            (idx[1], idx[3]),
            (idx[2], idx[3]),
        ];
        for (u, v) in edges {
            let pu = mesh.nodes[u].position;
            let pv = mesh.nodes[v].position;
            for axis in 0..3 {
                let d = (pu[axis] - pv[axis]).abs();
                if d > 1e-7 && d < min_step[axis] {
                    min_step[axis] = d;
                }
            }
        }
    }
    let dims = mesh.bounding_box().dimensions();
    for axis in 0..3 {
        if !min_step[axis].is_finite() {
            min_step[axis] = (dims[axis] / 10.0).max(1e-4);
        }
    }
    min_step
}

/// Matches surface nodes of `mesh` lying within tolerance of `cad_triangles` and computes
/// their consistent tributary area weights on the boundary skin.
pub fn match_surface_nodes_and_weights(
    mesh: &TetrahedralMesh,
    cad_triangles: &[[[f64; 3]; 3]],
    user_tolerance: f64,
) -> Vec<MatchedSurfaceNode> {
    if cad_triangles.is_empty() || mesh.nodes.is_empty() {
        return Vec::new();
    }

    let surf_tris = mesh.extract_surface_triangles();
    let mut surf_node_set = HashSet::new();
    for &[a, b, c] in &surf_tris {
        surf_node_set.insert(a);
        surf_node_set.insert(b);
        surf_node_set.insert(c);
    }
    let mut surf_nodes: Vec<usize> = if surf_node_set.is_empty() {
        (0..mesh.nodes.len()).collect()
    } else {
        surf_node_set.into_iter().collect()
    };
    surf_nodes.sort_unstable();

    let steps = estimate_voxel_steps(mesh);
    let h_max = steps[0].max(steps[1]).max(steps[2]);
    let h_diag = (steps[0] * steps[0] + steps[1] * steps[1] + steps[2] * steps[2]).sqrt();
    // Voxel boundary nodes on curved/staircase surfaces lie up to 0.5 * h_diag (~0.866 * h_max) from the smooth CAD surface.
    let search_tol = user_tolerance
        .max(0.95 * h_max)
        .max(0.60 * h_diag)
        .max(1e-4);

    // Analyze CAD triangles: area, normals, centroid, and planarity
    let mut tri_normals = Vec::with_capacity(cad_triangles.len());
    let mut weighted_normal = Vector3::zeros();
    let mut weighted_centroid = Vector3::zeros();
    let mut total_cad_area = 0.0;

    for t in cad_triangles {
        let (area, n) = triangle_area_and_normal(t[0], t[1], t[2]);
        tri_normals.push((area, n));
        if area > 1e-16 {
            let n_vec = Vector3::from(n);
            let c_vec = (Vector3::from(t[0]) + Vector3::from(t[1]) + Vector3::from(t[2])) / 3.0;
            weighted_normal += n_vec * area;
            weighted_centroid += c_vec * area;
            total_cad_area += area;
        }
    }

    let avg_normal = if weighted_normal.norm() > 1e-12 {
        weighted_normal.normalize()
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    };
    let cad_centroid = if total_cad_area > 1e-16 {
        weighted_centroid / total_cad_area
    } else {
        Vector3::from(cad_triangles[0][0])
    };

    let plane_coplanar_tol = 1e-4 * h_max.max(1.0);
    let is_planar = total_cad_area > 1e-16
        && cad_triangles.iter().zip(tri_normals.iter()).all(|(t, &(area, n))| {
            if area <= 1e-16 {
                return true;
            }
            let n_vec = Vector3::from(n);
            if n_vec.dot(&avg_normal) < 0.998 {
                return false;
            }
            t.iter().all(|v| {
                ((Vector3::from(*v) - cad_centroid).dot(&avg_normal)).abs() <= plane_coplanar_tol
            })
        });

    // Step 1: Filter surface nodes by 3D Euclidean distance to CAD triangles
    let mut candidates: Vec<(usize, f64, [f64; 3])> = Vec::new();
    for &nid in &surf_nodes {
        let p = mesh.nodes[nid].position;
        let mut min_dist = f64::INFINITY;
        let mut best_normal = [avg_normal.x, avg_normal.y, avg_normal.z];
        for (idx, t) in cad_triangles.iter().enumerate() {
            let d = point_to_triangle_distance_3d(p, t[0], t[1], t[2]);
            if d < min_dist {
                min_dist = d;
                if tri_normals[idx].0 > 1e-16 {
                    best_normal = tri_normals[idx].1;
                }
            }
        }
        if min_dist <= search_tol {
            candidates.push((nid, min_dist, best_normal));
        }
    }

    if candidates.is_empty() {
        return Vec::new();
    }

    // Step 2: For planar CAD faces, restrict to the single closest voxel grid plane layer
    // so that adjacent interior layers at distance +dz are not erroneously clamped/loaded.
    if is_planar {
        let h_n = (avg_normal.x.abs() * steps[0]
            + avg_normal.y.abs() * steps[1]
            + avg_normal.z.abs() * steps[2])
            .max(1e-6);

        let min_normal_dist = candidates
            .iter()
            .map(|&(nid, _, _)| {
                let p = mesh.nodes[nid].pos_vec();
                ((p - cad_centroid).dot(&avg_normal)).abs()
            })
            .fold(f64::INFINITY, f64::min);

        let normal_layer_tol = min_normal_dist + (0.45 * h_n).max(1e-6);
        candidates.retain(|&(nid, _, _)| {
            let p = mesh.nodes[nid].pos_vec();
            ((p - cad_centroid).dot(&avg_normal)).abs() <= normal_layer_tol
        });
    }

    let matched_ids: HashSet<usize> = candidates.iter().map(|&(nid, _, _)| nid).collect();

    // Step 3: Compute consistent tributary area weights from exterior tetrahedral surface triangles
    let mut area_weights: HashMap<usize, f64> = HashMap::with_capacity(candidates.len());
    for &(nid, _, _) in &candidates {
        area_weights.insert(nid, 0.0);
    }

    if mesh.is_tet10() {
        let mut any_tet10_face_matched = false;
        for (corners, mids) in mesh.extract_surface_tet10_faces() {
            if corners.iter().all(|c| matched_ids.contains(c))
                && mids.iter().all(|m| matched_ids.contains(m))
            {
                let p0 = mesh.nodes[corners[0]].position;
                let p1 = mesh.nodes[corners[1]].position;
                let p2 = mesh.nodes[corners[2]].position;
                let (area, _) = triangle_area_and_normal(p0, p1, p2);
                if area > 1e-18 {
                    if !is_planar {
                        let centroid = [
                            (p0[0] + p1[0] + p2[0]) / 3.0,
                            (p0[1] + p1[1] + p2[1]) / 3.0,
                            (p0[2] + p1[2] + p2[2]) / 3.0,
                        ];
                        let tri_dist = cad_triangles
                            .iter()
                            .map(|t| point_to_triangle_distance_3d(centroid, t[0], t[1], t[2]))
                            .fold(f64::INFINITY, f64::min);
                        if tri_dist > search_tol {
                            continue;
                        }
                    }
                    let share = area / 3.0;
                    for m in mids {
                        *area_weights.entry(m).or_insert(0.0) += share;
                    }
                    any_tet10_face_matched = true;
                }
            }
        }
        if any_tet10_face_matched {
            return candidates
                .into_iter()
                .map(|(nid, _, outward_normal)| {
                    let w = area_weights.get(&nid).copied().unwrap_or(0.0);
                    MatchedSurfaceNode {
                        node_id: nid,
                        area_weight: w,
                        outward_normal,
                    }
                })
                .collect();
        }
    }

    let mut any_full_tri_matched = false;
    for &[n0, n1, n2] in &surf_tris {
        if matched_ids.contains(&n0) && matched_ids.contains(&n1) && matched_ids.contains(&n2) {
            let p0 = mesh.nodes[n0].position;
            let p1 = mesh.nodes[n1].position;
            let p2 = mesh.nodes[n2].position;
            let (area, _) = triangle_area_and_normal(p0, p1, p2);
            if area > 1e-18 {
                // For non-planar faces (e.g. cylindrical hole wall), ensure the surface triangle centroid
                // is also close to the target CAD face so flat cap triangles bridging across a thin rim are excluded.
                if !is_planar {
                    let centroid = [
                        (p0[0] + p1[0] + p2[0]) / 3.0,
                        (p0[1] + p1[1] + p2[1]) / 3.0,
                        (p0[2] + p1[2] + p2[2]) / 3.0,
                    ];
                    let tri_dist = cad_triangles
                        .iter()
                        .map(|t| point_to_triangle_distance_3d(centroid, t[0], t[1], t[2]))
                        .fold(f64::INFINITY, f64::min);
                    if tri_dist > search_tol {
                        continue;
                    }
                }
                let share = area / 3.0;
                *area_weights.entry(n0).or_insert(0.0) += share;
                *area_weights.entry(n1).or_insert(0.0) += share;
                *area_weights.entry(n2).or_insert(0.0) += share;
                any_full_tri_matched = true;
            }
        }
    }

    // Fallback: if some matched nodes did not participate in a full 3-node surface triangle,
    // assign tributary area from any incident exterior surface triangle.
    if !any_full_tri_matched || area_weights.values().any(|&w| w <= 1e-18) {
        for &[n0, n1, n2] in &surf_tris {
            let p0 = mesh.nodes[n0].position;
            let p1 = mesh.nodes[n1].position;
            let p2 = mesh.nodes[n2].position;
            let (area, _) = triangle_area_and_normal(p0, p1, p2);
            let share = (area / 3.0).max(1e-12);
            for n in [n0, n1, n2] {
                if let Some(w) = area_weights.get_mut(&n) {
                    if *w <= 1e-18 {
                        *w += share;
                    }
                }
            }
        }
    }

    candidates
        .into_iter()
        .map(|(nid, _, outward_normal)| {
            let w = area_weights.get(&nid).copied().unwrap_or(1.0).max(1e-12);
            MatchedSurfaceNode {
                node_id: nid,
                area_weight: w,
                outward_normal,
            }
        })
        .collect()
}

/// Specification of a single connected joint's reaction force and torque acting on a part (P3-T7).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimulationJointLoadSpec {
    /// Identifier of the physics joint.
    pub joint_id: String,
    /// Joint type string (e.g. `"fixed"`, `"revolute"`, `"prismatic"`, `"spherical"`, `"cylindrical"`, `"gear"`).
    pub joint_type: String,
    /// Joint anchor point in part-local coordinates (SI meters `[x, y, z]`).
    pub anchor_local_m: [f64; 3],
    /// Reaction force acting ON the part at `anchor_local_m` in Newtons `[Fx, Fy, Fz]`.
    pub force_n: [f64; 3],
    /// Reaction torque acting ON the part about `anchor_local_m` in Newton-meters `[Tx, Ty, Tz]`.
    pub torque_nm: [f64; 3],
    /// Optional matched topological CAD face name (`P2-T8`).
    pub matched_face_name: Option<String>,
    /// Optional CAD surface triangles for the matched face (in part-local SI meters).
    pub face_triangles: Option<Vec<[[f64; 3]; 3]>>,
}

/// Specification of a part's mass and linear/angular accelerations for D'Alembert inertial relief (P3-T7).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimulationInertialLoadSpec {
    /// Part mass in kilograms (kg).
    pub mass_kg: f64,
    /// Part center of mass in part-local coordinates (SI meters `[x, y, z]`).
    pub center_of_mass_local_m: [f64; 3],
    /// Linear acceleration of the center of mass in part-local coordinates (m/s²).
    pub linear_acceleration_mps2: [f64; 3],
    /// Angular acceleration vector in part-local coordinates (rad/s²).
    pub angular_acceleration_rad_s2: [f64; 3],
    /// Angular velocity vector in part-local coordinates (rad/s).
    pub angular_velocity_rad_s: [f64; 3],
    /// Gravity acceleration vector in part-local coordinates (m/s²).
    pub gravity_local_mps2: [f64; 3],
}

/// Automatically matches a joint's `anchor_local_m` (in SI meters) to a named topological CAD face
/// in `part_mesh` (`P2-T8`) when the anchor lies on a face or on the axis of a cylindrical bore/shaft.
///
/// If an explicit `load_radius_m` is provided that is smaller than half the matched face's characteristic
/// radius, returns `None` so spherical `load_radius` node selection is used instead.
pub fn match_joint_anchor_to_cad_face(
    part_mesh: &TriangleMesh,
    anchor_local_m: [f64; 3],
    load_radius_m: Option<f64>,
) -> Option<(String, Vec<[[f64; 3]; 3]>)> {
    if part_mesh.faces.is_empty() || part_mesh.indices.is_empty() {
        return None;
    }

    let aabb = part_mesh.bounding_box();
    let dims = aabb.dimensions();
    let diag = (dims[0] * dims[0] + dims[1] * dims[1] + dims[2] * dims[2])
        .sqrt()
        .max(1e-4);
    let surface_tol = (0.02 * diag).max(1e-3);
    let anchor_vec = Vector3::from(anchor_local_m);

    type FaceMatchCandidate = (f64, f64, String, Vec<[[f64; 3]; 3]>, f64);
    let mut best_match: Option<FaceMatchCandidate> = None;

    for face in &part_mesh.faces {
        let tris = extract_face_triangles(part_mesh, &[face.id]);
        if tris.is_empty() {
            continue;
        }

        let mut min_tri_dist = f64::INFINITY;
        let mut weighted_centroid = Vector3::<f64>::zeros();
        let mut total_area = 0.0;

        for t in &tris {
            let d = point_to_triangle_distance_3d(anchor_local_m, t[0], t[1], t[2]);
            if d < min_tri_dist {
                min_tri_dist = d;
            }
            let (area, _) = triangle_area_and_normal(t[0], t[1], t[2]);
            if area > 1e-18 {
                let c = (Vector3::from(t[0]) + Vector3::from(t[1]) + Vector3::from(t[2])) / 3.0;
                weighted_centroid += c * area;
                total_area += area;
            }
        }

        let live_centroid = if total_area > 1e-18 {
            weighted_centroid / total_area
        } else {
            Vector3::from(tris[0][0])
        };

        let mut face_radius = 0.0_f64;
        for t in &tris {
            for v in t {
                let r = (Vector3::from(*v) - live_centroid).norm();
                if r > face_radius {
                    face_radius = r;
                }
            }
        }

        let centroid_dist = (anchor_vec - live_centroid).norm();

        // Check if anchor lies directly on the face triangles, or at the centroid/axis of a cylindrical hole/boss
        let is_cyl_axis_match = matches!(
            face.kind,
            mcp_cad_core::topology::FaceKind::Cylinder
        ) && centroid_dist <= surface_tol * 2.0;

        if min_tri_dist <= surface_tol || is_cyl_axis_match {
            let effective_dist = if is_cyl_axis_match {
                centroid_dist * 0.5
            } else {
                min_tri_dist
            };
            let replace = match &best_match {
                None => true,
                Some((best_d, best_cd, _, _, _)) => {
                    if (effective_dist - *best_d).abs() > 1e-5 {
                        effective_dist < *best_d
                    } else {
                        centroid_dist < *best_cd
                    }
                }
            };
            if replace {
                best_match = Some((
                    effective_dist,
                    centroid_dist,
                    face.name.clone(),
                    tris,
                    face_radius,
                ));
            }
        }
    }

    if let Some((_, _, name, tris, face_radius)) = best_match {
        if let Some(req_r) = load_radius_m {
            if req_r > 0.0 && face_radius > 1e-6 && req_r < 0.5 * face_radius {
                return None;
            }
        }
        Some((name, tris))
    } else {
        None
    }
}

/// Solves a $3 \times 3$ symmetric linear system $\mathbf{J} \boldsymbol{\theta} = \boldsymbol{\tau}$
/// using spectral pseudo-inversion so rank-deficient (e.g. collinear) node sets never produce NaNs.
fn solve_symmetric_3x3_pseudoinverse(
    j_mat: &nalgebra::Matrix3<f64>,
    rhs: &Vector3<f64>,
) -> Vector3<f64> {
    let sym = (*j_mat + j_mat.transpose()) * 0.5;
    let eig = sym.symmetric_eigen();
    let max_ev = eig
        .eigenvalues
        .iter()
        .copied()
        .map(f64::abs)
        .fold(0.0_f64, f64::max);
    if max_ev <= 1e-24 {
        return Vector3::zeros();
    }
    let cutoff = max_ev * 1e-9;
    let mut result = Vector3::zeros();
    for k in 0..3 {
        let ev = eig.eigenvalues[k];
        if ev.abs() > cutoff {
            let v = eig.eigenvectors.column(k);
            result += v * (v.dot(rhs) / ev);
        }
    }
    result
}

type ConsistentFaceMoment = (usize, f64, Vector3<f64>);

/// Computes consistent zeroth-order area weights $w_i = \int_\Gamma N_i\, dA$ and first-order
/// moment vectors $\mathbf{q}_i = \int_\Gamma N_i (\mathbf{x} - \mathbf{c})\, dA$ for a matched
/// surface face on `mesh` (supporting both linear `TET4` and quadratic `TET10`).
fn compute_consistent_face_moments(
    mesh: &TetrahedralMesh,
    matched: &[MatchedSurfaceNode],
    cad_triangles: &[[[f64; 3]; 3]],
) -> (Vector3<f64>, Vec<ConsistentFaceMoment>) {
    let matched_ids: HashSet<usize> = matched.iter().map(|m| m.node_id).collect();
    let steps = estimate_voxel_steps(mesh);
    let h_max = steps[0].max(steps[1]).max(steps[2]);
    let h_diag = (steps[0] * steps[0] + steps[1] * steps[1] + steps[2] * steps[2]).sqrt();
    let search_tol = (0.95 * h_max).max(0.60 * h_diag).max(1e-4);

    if mesh.is_tet10() {
        let mut active_faces = Vec::new();
        let mut weighted_centroid = Vector3::<f64>::zeros();
        let mut total_area = 0.0;

        for (corners, mids) in mesh.extract_surface_tet10_faces() {
            if corners.iter().all(|c| matched_ids.contains(c))
                && mids.iter().all(|m| matched_ids.contains(m))
            {
                let v0 = mesh.nodes[corners[0]].pos_vec();
                let v1 = mesh.nodes[corners[1]].pos_vec();
                let v2 = mesh.nodes[corners[2]].pos_vec();
                let (area, _) = triangle_area_and_normal(
                    [v0.x, v0.y, v0.z],
                    [v1.x, v1.y, v1.z],
                    [v2.x, v2.y, v2.z],
                );
                if area > 1e-18 {
                    let c_tri = (v0 + v1 + v2) / 3.0;
                    let tri_dist = cad_triangles
                        .iter()
                        .map(|t| {
                            point_to_triangle_distance_3d(
                                [c_tri.x, c_tri.y, c_tri.z],
                                t[0],
                                t[1],
                                t[2],
                            )
                        })
                        .fold(f64::INFINITY, f64::min);
                    if tri_dist <= search_tol {
                        weighted_centroid += c_tri * area;
                        total_area += area;
                        active_faces.push((corners, mids, v0, v1, v2, area));
                    }
                }
            }
        }

        if total_area > 1e-18 && !active_faces.is_empty() {
            let centroid = weighted_centroid / total_area;
            let mut w_map: HashMap<usize, f64> = HashMap::with_capacity(matched.len());
            let mut q_map: HashMap<usize, Vector3<f64>> = HashMap::with_capacity(matched.len());
            for m in matched {
                w_map.insert(m.node_id, 0.0);
                q_map.insert(m.node_id, Vector3::zeros());
            }

            for (corners, mids, v0, v1, v2, area) in active_faces {
                // Exact 6-node quadratic triangle integrals:
                // Corner a: int N_a dA = 0, int N_a (x - c) dA = (A / 60) * (2 v_a - v_b - v_c)
                let q_c0 = (v0 * 2.0 - v1 - v2) * (area / 60.0);
                let q_c1 = (v1 * 2.0 - v2 - v0) * (area / 60.0);
                let q_c2 = (v2 * 2.0 - v0 - v1) * (area / 60.0);
                *q_map.entry(corners[0]).or_insert_with(Vector3::zeros) += q_c0;
                *q_map.entry(corners[1]).or_insert_with(Vector3::zeros) += q_c1;
                *q_map.entry(corners[2]).or_insert_with(Vector3::zeros) += q_c2;

                // Mid-edge m_ab: int N_m dA = A / 3, int N_m (x - c) dA = (A / 15) * (2 v_a + 2 v_b + v_c) - (A / 3) * centroid
                let w_mid = area / 3.0;
                let q_m0 = (v0 * 2.0 + v1 * 2.0 + v2) * (area / 15.0) - centroid * w_mid;
                let q_m1 = (v1 * 2.0 + v2 * 2.0 + v0) * (area / 15.0) - centroid * w_mid;
                let q_m2 = (v2 * 2.0 + v0 * 2.0 + v1) * (area / 15.0) - centroid * w_mid;

                *w_map.entry(mids[0]).or_insert(0.0) += w_mid;
                *w_map.entry(mids[1]).or_insert(0.0) += w_mid;
                *w_map.entry(mids[2]).or_insert(0.0) += w_mid;

                *q_map.entry(mids[0]).or_insert_with(Vector3::zeros) += q_m0;
                *q_map.entry(mids[1]).or_insert_with(Vector3::zeros) += q_m1;
                *q_map.entry(mids[2]).or_insert_with(Vector3::zeros) += q_m2;
            }

            let list = matched
                .iter()
                .map(|m| {
                    let w = w_map.get(&m.node_id).copied().unwrap_or(0.0);
                    let q = q_map.get(&m.node_id).copied().unwrap_or_else(Vector3::zeros);
                    (m.node_id, w, q)
                })
                .collect();
            return (centroid, list);
        }
    }

    // Linear TET4 surface triangles:
    let surf_tris = mesh.extract_surface_triangles();
    let mut active_tris = Vec::new();
    let mut weighted_centroid = Vector3::<f64>::zeros();
    let mut total_area = 0.0;

    for [n0, n1, n2] in surf_tris {
        if matched_ids.contains(&n0) && matched_ids.contains(&n1) && matched_ids.contains(&n2) {
            let v0 = mesh.nodes[n0].pos_vec();
            let v1 = mesh.nodes[n1].pos_vec();
            let v2 = mesh.nodes[n2].pos_vec();
            let (area, _) = triangle_area_and_normal(
                [v0.x, v0.y, v0.z],
                [v1.x, v1.y, v1.z],
                [v2.x, v2.y, v2.z],
            );
            if area > 1e-18 {
                let c_tri = (v0 + v1 + v2) / 3.0;
                let tri_dist = cad_triangles
                    .iter()
                    .map(|t| {
                        point_to_triangle_distance_3d(
                            [c_tri.x, c_tri.y, c_tri.z],
                            t[0],
                            t[1],
                            t[2],
                        )
                    })
                    .fold(f64::INFINITY, f64::min);
                if tri_dist <= search_tol {
                    weighted_centroid += c_tri * area;
                    total_area += area;
                    active_tris.push(([n0, n1, n2], v0, v1, v2, area));
                }
            }
        }
    }

    if total_area > 1e-18 && !active_tris.is_empty() {
        let centroid = weighted_centroid / total_area;
        let mut w_map: HashMap<usize, f64> = HashMap::with_capacity(matched.len());
        let mut q_map: HashMap<usize, Vector3<f64>> = HashMap::with_capacity(matched.len());
        for m in matched {
            w_map.insert(m.node_id, 0.0);
            q_map.insert(m.node_id, Vector3::zeros());
        }

        for ([n0, n1, n2], v0, v1, v2, area) in active_tris {
            let w_node = area / 3.0;
            let q0 = (v0 * 2.0 + v1 + v2) * (area / 12.0) - centroid * w_node;
            let q1 = (v1 * 2.0 + v2 + v0) * (area / 12.0) - centroid * w_node;
            let q2 = (v2 * 2.0 + v0 + v1) * (area / 12.0) - centroid * w_node;

            *w_map.entry(n0).or_insert(0.0) += w_node;
            *w_map.entry(n1).or_insert(0.0) += w_node;
            *w_map.entry(n2).or_insert(0.0) += w_node;

            *q_map.entry(n0).or_insert_with(Vector3::zeros) += q0;
            *q_map.entry(n1).or_insert_with(Vector3::zeros) += q1;
            *q_map.entry(n2).or_insert_with(Vector3::zeros) += q2;
        }

        let list = matched
            .iter()
            .map(|m| {
                let w = w_map.get(&m.node_id).copied().unwrap_or(0.0);
                let q = q_map.get(&m.node_id).copied().unwrap_or_else(Vector3::zeros);
                (m.node_id, w, q)
            })
            .collect();
        return (centroid, list);
    }

    // Fallback to discrete point-cloud weights if no full surface triangles matched:
    let total_w: f64 = matched.iter().map(|m| m.area_weight).sum();
    let inv_w = if total_w > 1e-18 {
        1.0 / total_w
    } else {
        1.0 / (matched.len().max(1) as f64)
    };
    let mut centroid = Vector3::zeros();
    for m in matched {
        let w = if total_w > 1e-18 { m.area_weight } else { 1.0 };
        centroid += mesh.nodes[m.node_id].pos_vec() * (w * inv_w);
    }
    let list = matched
        .iter()
        .map(|m| {
            let w = if total_w > 1e-18 { m.area_weight } else { 1.0 };
            let r = mesh.nodes[m.node_id].pos_vec() - centroid;
            (m.node_id, w, r * w)
        })
        .collect();
    (centroid, list)
}

/// Distributes a joint's reaction force `force_n` ($\mathbf{F}_j$) and torque `torque_nm` ($\boldsymbol{\tau}_j$)
/// applied at `anchor_local_m` ($\mathbf{p}_j$) across a set of mesh nodes with consistent weights
/// `(node_id, w_i, q_i)` and centroid `centroid` ($\mathbf{c}_j$).
///
/// Guarantees:
/// - $\sum_i \mathbf{f}_i = \mathbf{F}_j$,
/// - $\sum_i (\mathbf{x}_i - \mathbf{p}_j) \times \mathbf{f}_i = \boldsymbol{\tau}_j$ (for any non-collinear node set).
fn distribute_force_and_moment_with_moments(
    mesh: &TetrahedralMesh,
    anchor_local_m: [f64; 3],
    force_n: [f64; 3],
    torque_nm: [f64; 3],
    centroid: Vector3<f64>,
    node_moments: &[(usize, f64, Vector3<f64>)],
    nodal_forces: &mut [[f64; 3]],
) {
    if node_moments.is_empty() {
        return;
    }

    let f_vec = Vector3::from(force_n);
    let tau_vec = Vector3::from(torque_nm);
    let anchor_vec = Vector3::from(anchor_local_m);

    // Total moment about patch centroid c_j: tau_c = tau_j + (p_j - c_j) x F_j
    let tau_c = tau_vec + (anchor_vec - centroid).cross(&f_vec);

    let total_w: f64 = node_moments.iter().map(|(_, w, _)| *w).sum();
    let inv_w = if total_w > 1e-18 {
        1.0 / total_w
    } else {
        1.0 / (node_moments.len() as f64)
    };

    // Assemble 3x3 consistent inertia tensor J = sum_i ((r_i . q_i) I_3 - q_i r_i^T)
    let mut j_mat = nalgebra::Matrix3::<f64>::zeros();
    for &(nid, _, ref q_i) in node_moments {
        let r_i = mesh.nodes[nid].pos_vec() - centroid;
        let rq_dot = r_i.dot(q_i);
        j_mat += nalgebra::Matrix3::<f64>::identity() * rq_dot - q_i * r_i.transpose();
    }

    let theta = if tau_c.norm() > 1e-16 {
        solve_symmetric_3x3_pseudoinverse(&j_mat, &tau_c)
    } else {
        Vector3::zeros()
    };

    for &(nid, w_i, ref q_i) in node_moments {
        let w_frac = if total_w > 1e-18 { w_i * inv_w } else { inv_w };
        let f_node = f_vec * w_frac + theta.cross(q_i);
        nodal_forces[nid][0] += f_node.x;
        nodal_forces[nid][1] += f_node.y;
        nodal_forces[nid][2] += f_node.z;
    }
}

/// Selects a minimal 6-DOF **3-2-1 kinematic support** (`(n1, n2, n3)`) on `mesh` that eliminates
/// all 3 rigid-body translations and all 3 rigid-body rotations without over-constraining the part.
///
/// Prefers corner nodes not in `excluded_loaded_nodes` and furthest from `anchor_points` so that
/// the kinematic datum lies in an unloaded region of the part.
pub fn select_321_minimal_support_constraints(
    mesh: &TetrahedralMesh,
    excluded_loaded_nodes: &HashSet<usize>,
    anchor_points: &[[f64; 3]],
) -> (Vec<DirichletConstraint>, [usize; 3]) {
    // Restrict to corner nodes (indices appearing in `mesh.elements`) so constraints are valid on both TET4 and TET10
    let mut corner_set = HashSet::new();
    for elem in &mesh.elements {
        for &nid in &elem.node_indices {
            corner_set.insert(nid);
        }
    }
    let mut candidates: Vec<usize> = corner_set
        .iter()
        .copied()
        .filter(|nid| !excluded_loaded_nodes.contains(nid))
        .collect();
    if candidates.len() < 4 {
        candidates = corner_set.into_iter().collect();
    }
    candidates.sort_unstable();

    if candidates.is_empty() {
        return (Vec::new(), [0, 0, 0]);
    }

    // 1. Choose n1: candidate node furthest from the loaded joint anchor points
    let n1 = if !anchor_points.is_empty() {
        *candidates
            .iter()
            .max_by(|&&a, &&b| {
                let pa = mesh.nodes[a].pos_vec();
                let pb = mesh.nodes[b].pos_vec();
                let da: f64 = anchor_points
                    .iter()
                    .map(|p| (pa - Vector3::from(*p)).norm())
                    .fold(f64::INFINITY, f64::min);
                let db: f64 = anchor_points
                    .iter()
                    .map(|p| (pb - Vector3::from(*p)).norm())
                    .fold(f64::INFINITY, f64::min);
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(&candidates[0])
    } else {
        candidates[0]
    };
    let p1 = mesh.nodes[n1].pos_vec();

    // 2. Choose n2 and primary separation axis a1 in {0, 1, 2}:
    // Maximize |p2[a1] - p1[a1]| while staying away from the loaded anchors when possible.
    let mut best_n2 = candidates[0];
    let mut best_a1 = 0usize;
    let mut best_span = -1.0_f64;

    // First try candidates in the unloaded zone whose minimum distance to anchors is at least 40% of n1's distance
    let n1_anchor_dist: f64 = anchor_points
        .iter()
        .map(|p| (p1 - Vector3::from(*p)).norm())
        .fold(f64::INFINITY, f64::min);
    let safe_dist_threshold = if n1_anchor_dist.is_finite() {
        0.4 * n1_anchor_dist
    } else {
        0.0
    };

    for pass in 0..2 {
        for &cand in &candidates {
            if cand == n1 {
                continue;
            }
            let pc = mesh.nodes[cand].pos_vec();
            if pass == 0 && !anchor_points.is_empty() {
                let dc = anchor_points
                    .iter()
                    .map(|p| (pc - Vector3::from(*p)).norm())
                    .fold(f64::INFINITY, f64::min);
                if dc < safe_dist_threshold {
                    continue;
                }
            }
            let diff = pc - p1;
            for axis in 0..3 {
                let span = diff[axis].abs();
                if span > best_span {
                    best_span = span;
                    best_n2 = cand;
                    best_a1 = axis;
                }
            }
        }
        if best_span > 1e-7 {
            break;
        }
    }

    let p2 = mesh.nodes[best_n2].pos_vec();
    let d12 = p2 - p1;
    let perp_axes: [usize; 2] = match best_a1 {
        0 => [1, 2],
        1 => [0, 2],
        _ => [0, 1],
    };

    // 3. Choose n3 and locked axis a2 in perp_axes to maximize |(d12 x (p3 - p1))[a2]|
    let mut best_n3 = candidates[0];
    let mut best_a2 = perp_axes[0];
    let mut best_cross_comp = -1.0_f64;

    for pass in 0..2 {
        for &cand in &candidates {
            if cand == n1 || cand == best_n2 {
                continue;
            }
            let pc = mesh.nodes[cand].pos_vec();
            if pass == 0 && !anchor_points.is_empty() {
                let dc = anchor_points
                    .iter()
                    .map(|p| (pc - Vector3::from(*p)).norm())
                    .fold(f64::INFINITY, f64::min);
                if dc < safe_dist_threshold {
                    continue;
                }
            }
            let cross = d12.cross(&(pc - p1));
            for &axis in &perp_axes {
                let val = cross[axis].abs();
                if val > best_cross_comp {
                    best_cross_comp = val;
                    best_n3 = cand;
                    best_a2 = axis;
                }
            }
        }
        if best_cross_comp > 1e-12 {
            break;
        }
    }

    let n2_comp = match best_a1 {
        0 => DofComponent::YZ,
        1 => DofComponent::XZ,
        _ => DofComponent::XY,
    };
    let n3_comp = match best_a2 {
        0 => DofComponent::X,
        1 => DofComponent::Y,
        _ => DofComponent::Z,
    };

    let constraints = vec![
        DirichletConstraint::FixedNodes {
            node_ids: vec![n1],
            components: DofComponent::All,
        },
        DirichletConstraint::FixedNodes {
            node_ids: vec![best_n2],
            components: n2_comp,
        },
        DirichletConstraint::FixedNodes {
            node_ids: vec![best_n3],
            components: n3_comp,
        },
    ];

    (constraints, [n1, best_n2, best_n3])
}

/// Synthesizes FEA `BoundaryConditions` and a `FeaSimulationSource` provenance report for a part
/// from extracted physics simulation joint loads and D'Alembert inertial state (`P3-T7`).
///
/// - Distributes each joint's reaction force $\mathbf{F}_j$ and reaction moment $\boldsymbol{\tau}_j$
///   across the matched named CAD face (`P2-T8`) or the tetrahedral nodes within `load_radius_m`
///   of `anchor_local_m` using consistent virtual-work shape function moments so that
///   $\sum \mathbf{f}_i = \mathbf{F}_j$ and $\sum (\mathbf{x}_i - \mathbf{p}_j) \times \mathbf{f}_i = \boldsymbol{\tau}_j$.
/// - Distributes the D'Alembert + gravity volume body force $m(\mathbf{g}_{\text{local}} - \mathbf{a}(\mathbf{x}))$
///   across all unconstrained volume nodes and projects out any residual net force/moment so the
///   nodal load vector is in exact 6-DOF static equilibrium ($\sum \mathbf{f}_n = \mathbf{0}$,
///   $\sum \mathbf{r}_n \times \mathbf{f}_n = \mathbf{0}$).
/// - Constrains **only 6 DOFs** using a minimal 3-node (**3-2-1**) kinematic support so that
///   stresses are never distorted by artificial boundary clamping.
#[allow(clippy::too_many_arguments)]
pub fn build_simulation_boundary_conditions(
    mesh: &TetrahedralMesh,
    joints: &[SimulationJointLoadSpec],
    inertia: &SimulationInertialLoadSpec,
    load_radius_m: Option<f64>,
    simulation_time_s: f64,
    select: &str,
    step_index: usize,
    length_scale_from_m: f64,
) -> Result<(BoundaryConditions, crate::result::FeaSimulationSource), FeaError> {
    let total_nodes = mesh.node_count();
    if total_nodes == 0 || mesh.elements.is_empty() {
        return Err(FeaError::EmptyMesh);
    }

    let steps = estimate_voxel_steps(mesh);
    let h_diag = (steps[0] * steps[0] + steps[1] * steps[1] + steps[2] * steps[2])
        .sqrt()
        .max(1e-4);
    let default_radius_m = load_radius_m.unwrap_or_else(|| (1.75 * h_diag).max(1e-3));

    let mut nodal_forces = vec![[0.0_f64; 3]; total_nodes];
    let mut loaded_nodes = HashSet::new();
    let mut joint_sources = Vec::with_capacity(joints.len());
    let mut anchor_points = Vec::with_capacity(joints.len());

    for j in joints {
        anchor_points.push(j.anchor_local_m);

        // Try matched CAD face triangles first (unless load_radius_m explicitly overrode it)
        let mut used_face: Option<String> = None;
        let mut matched_count = 0usize;

        if let Some(ref tris) = j.face_triangles {
            if !tris.is_empty() {
                let matched = match_surface_nodes_and_weights(mesh, tris, 0.0);
                if matched.len() >= 3 {
                    let (centroid, node_moments) =
                        compute_consistent_face_moments(mesh, &matched, tris);
                    distribute_force_and_moment_with_moments(
                        mesh,
                        j.anchor_local_m,
                        j.force_n,
                        j.torque_nm,
                        centroid,
                        &node_moments,
                        &mut nodal_forces,
                    );
                    for &(nid, _, _) in &node_moments {
                        loaded_nodes.insert(nid);
                    }
                    matched_count = node_moments.len();
                    used_face = j.matched_face_name.clone();
                }
            }
        }

        // Fallback to spherical neighborhood within load_radius_m of anchor_local_m
        let used_radius_opt = if matched_count == 0 {
            let anchor_vec = Vector3::from(j.anchor_local_m);
            let mut node_dists: Vec<(usize, f64)> = mesh
                .nodes
                .iter()
                .map(|n| (n.id, (n.pos_vec() - anchor_vec).norm()))
                .collect();
            node_dists.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

            let mut eff_r = default_radius_m;
            let mut selected: Vec<(usize, f64)> = node_dists
                .iter()
                .copied()
                .filter(|&(_, d)| d <= eff_r)
                .collect();

            // Ensure at least 6 nodes (non-collinear) so 3D torque couples can always be resolved
            let min_needed = 6.min(total_nodes);
            if selected.len() < min_needed {
                selected = node_dists.into_iter().take(min_needed).collect();
                if let Some(&(_, max_d)) = selected.last() {
                    eff_r = eff_r.max(max_d);
                }
            }

            let r_scale = eff_r.max(1e-6);
            let mut weights = Vec::with_capacity(selected.len());
            let mut total_w = 0.0;
            for &(nid, d) in &selected {
                let w = 1.0 + (1.0 - d / r_scale).max(0.0);
                weights.push((nid, w));
                total_w += w;
            }
            let inv_w = if total_w > 1e-18 {
                1.0 / total_w
            } else {
                1.0 / (selected.len() as f64)
            };

            let mut centroid = Vector3::zeros();
            for &(nid, w) in &weights {
                centroid += mesh.nodes[nid].pos_vec() * (w * inv_w);
            }

            let node_moments: Vec<(usize, f64, Vector3<f64>)> = weights
                .into_iter()
                .map(|(nid, w)| {
                    let w_norm = w * inv_w;
                    let r = mesh.nodes[nid].pos_vec() - centroid;
                    (nid, w_norm, r * w_norm)
                })
                .collect();

            distribute_force_and_moment_with_moments(
                mesh,
                j.anchor_local_m,
                j.force_n,
                j.torque_nm,
                centroid,
                &node_moments,
                &mut nodal_forces,
            );

            for &(nid, _, _) in &node_moments {
                loaded_nodes.insert(nid);
            }
            matched_count = node_moments.len();
            Some(eff_r * length_scale_from_m)
        } else {
            None
        };

        joint_sources.push(crate::result::FeaJointLoadSource {
            joint_id: j.joint_id.clone(),
            joint_type: j.joint_type.clone(),
            anchor_local: [
                j.anchor_local_m[0] * length_scale_from_m,
                j.anchor_local_m[1] * length_scale_from_m,
                j.anchor_local_m[2] * length_scale_from_m,
            ],
            force: j.force_n,
            torque: j.torque_nm,
            matched_face: used_face,
            load_radius: used_radius_opt,
            matched_node_count: matched_count,
        });
    }

    // 2. Select the 6-DOF minimal 3-2-1 kinematic support nodes (outside loaded_nodes)
    let (dirichlet_constraints, support_nodes_321) =
        select_321_minimal_support_constraints(mesh, &loaded_nodes, &anchor_points);

    let support_node_set: HashSet<usize> = support_nodes_321.iter().copied().collect();

    // 3. Compute nodal volume weights v_n across elements (excluding the 3 kinematic support nodes
    //    when sufficient nodes exist so the 6 fixed DOFs carry zero external nodal force)
    let mut vol_weights = vec![0.0_f64; total_nodes];
    if mesh.is_tet10() {
        for (elem_idx, elem) in mesh.elements.iter().enumerate() {
            let p0 = mesh.nodes[elem.node_indices[0]].pos_vec();
            let p1 = mesh.nodes[elem.node_indices[1]].pos_vec();
            let p2 = mesh.nodes[elem.node_indices[2]].pos_vec();
            let p3 = mesh.nodes[elem.node_indices[3]].pos_vec();
            let vol = ((p1 - p0).cross(&(p2 - p0)).dot(&(p3 - p0))).abs() / 6.0;
            let ten = mesh.tet10_elements[elem_idx];
            let share = vol / 10.0;
            for &nid in &ten {
                vol_weights[nid] += share;
            }
        }
    } else {
        for elem in &mesh.elements {
            let p0 = mesh.nodes[elem.node_indices[0]].pos_vec();
            let p1 = mesh.nodes[elem.node_indices[1]].pos_vec();
            let p2 = mesh.nodes[elem.node_indices[2]].pos_vec();
            let p3 = mesh.nodes[elem.node_indices[3]].pos_vec();
            let vol = ((p1 - p0).cross(&(p2 - p0)).dot(&(p3 - p0))).abs() / 6.0;
            let share = vol * 0.25;
            for &nid in &elem.node_indices {
                vol_weights[nid] += share;
            }
        }
    }

    if total_nodes > 8 {
        for &sn in &support_nodes_321 {
            if !loaded_nodes.contains(&sn) {
                vol_weights[sn] = 0.0;
            }
        }
    }

    let total_vol_w: f64 = vol_weights.iter().sum();
    let inv_vol_w = if total_vol_w > 1e-18 {
        1.0 / total_vol_w
    } else {
        1.0 / (total_nodes as f64)
    };

    let mut vol_centroid = Vector3::zeros();
    for (nid, &vw) in vol_weights.iter().enumerate() {
        let wn = if total_vol_w > 1e-18 {
            vw * inv_vol_w
        } else {
            inv_vol_w
        };
        vol_centroid += mesh.nodes[nid].pos_vec() * wn;
    }

    // 4. Assemble D'Alembert + gravity body forces:
    //    b_n = w_n * m * (g_local - a_com - alpha x r_n - omega x (omega x r_n))
    let g_vec = Vector3::from(inertia.gravity_local_mps2);
    let a_vec = Vector3::from(inertia.linear_acceleration_mps2);
    let alpha_vec = Vector3::from(inertia.angular_acceleration_rad_s2);
    let omega_vec = Vector3::from(inertia.angular_velocity_rad_s);
    let mass = inertia.mass_kg.max(0.0);

    let mut body_forces = vec![Vector3::<f64>::zeros(); total_nodes];
    for (nid, &vw) in vol_weights.iter().enumerate() {
        let wn = if total_vol_w > 1e-18 {
            vw * inv_vol_w
        } else if support_node_set.contains(&nid) && total_nodes > 8 {
            0.0
        } else {
            inv_vol_w
        };
        if wn <= 0.0 {
            continue;
        }
        let r_n = mesh.nodes[nid].pos_vec() - vol_centroid;
        let a_rot = alpha_vec.cross(&r_n) + omega_vec.cross(&omega_vec.cross(&r_n));
        let specific_force = g_vec - a_vec - a_rot;
        body_forces[nid] = specific_force * (mass * wn);
    }

    // 5. Project out any residual net force Delta_F and residual net moment Delta_M across the
    //    volume mass distribution (Inertial Relief projection) so that:
    //    sum_n (f_joint,n + b_n) == 0  and  sum_n (x_n - c_vol) x (f_joint,n + b_n) == 0
    //    to machine precision, guaranteeing zero support reactions at the 3-2-1 datum nodes.
    let mut net_f = Vector3::<f64>::zeros();
    for nid in 0..total_nodes {
        net_f += Vector3::from(nodal_forces[nid]) + body_forces[nid];
    }

    for (nid, &vw) in vol_weights.iter().enumerate() {
        let wn = if total_vol_w > 1e-18 {
            vw * inv_vol_w
        } else {
            0.0
        };
        body_forces[nid] -= net_f * wn;
    }

    let mut net_m = Vector3::<f64>::zeros();
    let mut j_vol = nalgebra::Matrix3::<f64>::zeros();
    for (nid, &vw) in vol_weights.iter().enumerate() {
        let wn = if total_vol_w > 1e-18 {
            vw * inv_vol_w
        } else {
            0.0
        };
        let r_n = mesh.nodes[nid].pos_vec() - vol_centroid;
        let f_tot = Vector3::from(nodal_forces[nid]) + body_forces[nid];
        net_m += r_n.cross(&f_tot);
        if wn > 0.0 {
            j_vol +=
                (nalgebra::Matrix3::<f64>::identity() * r_n.norm_squared() - r_n * r_n.transpose())
                    * wn;
        }
    }

    if net_m.norm() > 1e-16 {
        let delta_alpha = solve_symmetric_3x3_pseudoinverse(&j_vol, &net_m);
        for (nid, &vw) in vol_weights.iter().enumerate() {
            let wn = if total_vol_w > 1e-18 {
                vw * inv_vol_w
            } else {
                0.0
            };
            if wn > 0.0 {
                let r_n = mesh.nodes[nid].pos_vec() - vol_centroid;
                body_forces[nid] -= delta_alpha.cross(&r_n) * wn;
            }
        }
    }

    let mut total_body_force = Vector3::<f64>::zeros();
    for nid in 0..total_nodes {
        total_body_force += body_forces[nid];
        nodal_forces[nid][0] += body_forces[nid].x;
        nodal_forces[nid][1] += body_forces[nid].y;
        nodal_forces[nid][2] += body_forces[nid].z;
    }

    let inertial_relief_applied = total_body_force.norm() > 1e-9
        || a_vec.norm() > 1e-6
        || alpha_vec.norm() > 1e-6
        || omega_vec.norm() > 1e-6
        || g_vec.norm() > 1e-6;

    // 6. Assemble BoundaryConditions with the 3-2-1 Dirichlet constraints and non-zero nodal forces
    let mut bc = BoundaryConditions::new();
    bc.dirichlet = dirichlet_constraints;
    for (nid, &f) in nodal_forces.iter().enumerate() {
        if f[0].abs() > 1e-18 || f[1].abs() > 1e-18 || f[2].abs() > 1e-18 {
            bc.add_nodal_force(nid, f);
        }
    }

    let source = crate::result::FeaSimulationSource {
        simulation_time_s,
        select: select.to_string(),
        step_index,
        load_radius: default_radius_m * length_scale_from_m,
        inertial_relief_applied,
        body_force_total_n: [total_body_force.x, total_body_force.y, total_body_force.z],
        support_mode: "3-2-1_inertial_relief".to_string(),
        support_nodes_321: Some(support_nodes_321),
        joints: joint_sources,
    };

    Ok((bc, source))
}


