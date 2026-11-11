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

