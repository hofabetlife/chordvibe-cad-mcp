//! Boundary conditions (Dirichlet kinematic constraints and Neumann mechanical loads).

use std::collections::HashMap;
use serde::{Deserialize, Serialize};
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

/// Neumann boundary load (applied forces).
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
    /// Total force distributed evenly over surface nodes on an AABB face
    AabbFaceForce {
        /// Target AABB face
        face: AabbFace,
        /// Search tolerance in meters
        #[serde(default = "default_tolerance")]
        tolerance: f64,
        /// Total resultant force [Fx, Fy, Fz] in Newtons
        total_force: [f64; 3],
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

                    // Compute consistent tributary area weights:
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
