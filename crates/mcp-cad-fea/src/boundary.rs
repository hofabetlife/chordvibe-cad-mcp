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
        let resolved = self.resolve_allowing_unconstrained(mesh)?;
        if resolved.fixed_dofs.is_empty() {
            return Err(FeaError::UnconstrainedModel);
        }
        Ok(resolved)
    }

    /// Resolves high-level boundary conditions without requiring at least one Dirichlet DOF.
    ///
    /// Used by the multi-body deformable contact solver where a body may be kinematically
    /// constrained through [`HubCoupling`] joints and/or unilateral contact constraints.
    pub fn resolve_allowing_unconstrained(
        &self,
        mesh: &TetrahedralMesh,
    ) -> Result<ResolvedBoundary, FeaError> {
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

                    let node_set: HashSet<usize> = nodes.iter().copied().collect();
                    let aabb_n = aabb_face_outward_normal(*face);

                    // Consistent virtual-work surface integration over exterior boundary faces:
                    // - TET10 (T6): exact 6-node quadratic shape-function integrals
                    //   (0 on corners and A_T / 3 on mid-edge nodes for straight-sided faces;
                    //    6-point Dunavant quadrature for curved/shifted-midpoint faces).
                    // - TET4 (T3): exact 3-node linear triangle integrals (A_T / 3 per vertex).
                    if mesh.is_tet10() {
                        let mut node_weights: HashMap<usize, f64> = HashMap::new();
                        let mut total_w = 0.0;
                        for (corners, mids) in mesh.extract_surface_tet10_faces() {
                            if corners.iter().all(|c| node_set.contains(c))
                                && mids.iter().all(|m| node_set.contains(m))
                            {
                                let p0 = mesh.nodes[corners[0]].position;
                                let p1 = mesh.nodes[corners[1]].position;
                                let p2 = mesh.nodes[corners[2]].position;
                                let (flat_area, face_n) = triangle_area_and_normal(p0, p1, p2);
                                if flat_area > 1e-18
                                    && Vector3::from(face_n).dot(&aabb_n).abs() > 0.25
                                {
                                    let m0 = mesh.nodes[mids[0]].position;
                                    let m1 = mesh.nodes[mids[1]].position;
                                    let m2 = mesh.nodes[mids[2]].position;
                                    let (w6, face_area) =
                                        t6_consistent_area_weights(p0, p1, p2, m0, m1, m2);
                                    if face_area > 1e-18 {
                                        let nodes6 = [
                                            corners[0], corners[1], corners[2], mids[0], mids[1],
                                            mids[2],
                                        ];
                                        for k in 0..6 {
                                            if w6[k] != 0.0 {
                                                *node_weights.entry(nodes6[k]).or_insert(0.0) +=
                                                    w6[k];
                                            }
                                        }
                                        total_w += face_area;
                                    }
                                }
                            }
                        }
                        if total_w > 1e-18 {
                            for (nid, w) in node_weights {
                                let frac = w / total_w;
                                nodal_forces[nid][0] += total_force[0] * frac;
                                nodal_forces[nid][1] += total_force[1] * frac;
                                nodal_forces[nid][2] += total_force[2] * frac;
                            }
                            continue;
                        }
                    } else {
                        let mut node_weights: HashMap<usize, f64> = HashMap::new();
                        let mut total_w = 0.0;
                        for [n0, n1, n2] in mesh.extract_surface_triangles() {
                            if node_set.contains(&n0)
                                && node_set.contains(&n1)
                                && node_set.contains(&n2)
                            {
                                let p0 = mesh.nodes[n0].position;
                                let p1 = mesh.nodes[n1].position;
                                let p2 = mesh.nodes[n2].position;
                                let (area, face_n) = triangle_area_and_normal(p0, p1, p2);
                                if area > 1e-18 && Vector3::from(face_n).dot(&aabb_n).abs() > 0.25 {
                                    let share = area / 3.0;
                                    *node_weights.entry(n0).or_insert(0.0) += share;
                                    *node_weights.entry(n1).or_insert(0.0) += share;
                                    *node_weights.entry(n2).or_insert(0.0) += share;
                                    total_w += area;
                                }
                            }
                        }
                        if total_w > 1e-18 {
                            for (nid, w) in node_weights {
                                let frac = w / total_w;
                                nodal_forces[nid][0] += total_force[0] * frac;
                                nodal_forces[nid][1] += total_force[1] * frac;
                                nodal_forces[nid][2] += total_force[2] * frac;
                            }
                            continue;
                        }
                    }

                    // Degenerate fallback (e.g., single vertex or edge touching an AABB extreme)
                    let count = nodes.len() as f64;
                    for n in nodes {
                        nodal_forces[n][0] += total_force[0] / count;
                        nodal_forces[n][1] += total_force[1] / count;
                        nodal_forces[n][2] += total_force[2] / count;
                    }
                }
                NeumannLoad::SurfaceTrianglesForce {
                    triangles,
                    tolerance,
                    total_force,
                    ..
                } => {
                    let applied_conforming = if mesh.is_boundary_conforming() {
                        integrate_conforming_surface_force(
                            mesh,
                            triangles,
                            *tolerance,
                            *total_force,
                            &mut nodal_forces,
                        )
                    } else {
                        false
                    };

                    if !applied_conforming {
                        let matched = match_surface_nodes_and_weights(mesh, triangles, *tolerance);
                        if matched.is_empty() {
                            return Err(FeaError::NoBoundaryNodesMatched {
                                tolerance: *tolerance,
                            });
                        }
                        let total_weight: f64 = matched.iter().map(|m| m.area_weight).sum();
                        if total_weight > 1e-18 {
                            for m in &matched {
                                if m.area_weight != 0.0 {
                                    let frac = m.area_weight / total_weight;
                                    nodal_forces[m.node_id][0] += total_force[0] * frac;
                                    nodal_forces[m.node_id][1] += total_force[1] * frac;
                                    nodal_forces[m.node_id][2] += total_force[2] * frac;
                                }
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
                }
                NeumannLoad::SurfaceTrianglesPressure {
                    triangles,
                    tolerance,
                    pressure_pa,
                    ..
                } => {
                    let applied_conforming = if mesh.is_boundary_conforming() {
                        integrate_conforming_surface_pressure(
                            mesh,
                            triangles,
                            *tolerance,
                            *pressure_pa,
                            &mut nodal_forces,
                        )
                    } else {
                        false
                    };

                    if !applied_conforming {
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

fn aabb_face_outward_normal(face: AabbFace) -> Vector3<f64> {
    match face {
        AabbFace::MinX => Vector3::new(-1.0, 0.0, 0.0),
        AabbFace::MaxX => Vector3::new(1.0, 0.0, 0.0),
        AabbFace::MinY => Vector3::new(0.0, -1.0, 0.0),
        AabbFace::MaxY => Vector3::new(0.0, 1.0, 0.0),
        AabbFace::MinZ => Vector3::new(0.0, 0.0, -1.0),
        AabbFace::MaxZ => Vector3::new(0.0, 0.0, 1.0),
    }
}

/// 6-point degree-4 symmetric Dunavant quadrature on the reference triangle (area = 0.5).
const T6_DUNAVANT_A1: f64 = 0.816_847_572_980_459;
const T6_DUNAVANT_B1: f64 = 0.091_576_213_509_771;
const T6_DUNAVANT_W1: f64 = 0.109_951_743_655_322 * 0.5;
const T6_DUNAVANT_A2: f64 = 0.108_103_018_168_070;
const T6_DUNAVANT_B2: f64 = 0.445_948_490_915_965;
const T6_DUNAVANT_W2: f64 = 0.223_381_589_678_011 * 0.5;
const T6_DUNAVANT_GP6: [([f64; 3], f64); 6] = [
    ([T6_DUNAVANT_A1, T6_DUNAVANT_B1, T6_DUNAVANT_B1], T6_DUNAVANT_W1),
    ([T6_DUNAVANT_B1, T6_DUNAVANT_A1, T6_DUNAVANT_B1], T6_DUNAVANT_W1),
    ([T6_DUNAVANT_B1, T6_DUNAVANT_B1, T6_DUNAVANT_A1], T6_DUNAVANT_W1),
    ([T6_DUNAVANT_A2, T6_DUNAVANT_B2, T6_DUNAVANT_B2], T6_DUNAVANT_W2),
    ([T6_DUNAVANT_B2, T6_DUNAVANT_A2, T6_DUNAVANT_B2], T6_DUNAVANT_W2),
    ([T6_DUNAVANT_B2, T6_DUNAVANT_B2, T6_DUNAVANT_A2], T6_DUNAVANT_W2),
];

/// Computes exact consistent virtual-work surface area weights
/// $w_k = \int_{\hat{T}} N_k^{(6)}(\xi, \eta) \|\mathbf{x}_\xi \times \mathbf{x}_\eta\|\,d\xi\,d\eta$
/// and total surface area $A = \sum_{k=0}^5 w_k$ for a 6-node quadratic triangle (`T6`)
/// with corners `(p0, p1, p2)` and mid-edge nodes `(m0 = m01, m1 = m12, m2 = m20)`.
///
/// - For straight-sided triangles with exact edge midpoints, returns the exact closed-form
///   result `([0, 0, 0, A/3, A/3, A/3], A)` with zero floating-point corner noise.
/// - For curved or shifted-midpoint quadratic triangles, integrates with 6-point degree-4
///   Dunavant quadrature (`T6_DUNAVANT_GP6`).
fn t6_consistent_area_weights(
    p0: [f64; 3],
    p1: [f64; 3],
    p2: [f64; 3],
    m0: [f64; 3],
    m1: [f64; 3],
    m2: [f64; 3],
) -> ([f64; 6], f64) {
    let v0 = Vector3::from(p0);
    let v1 = Vector3::from(p1);
    let v2 = Vector3::from(p2);
    let vm0 = Vector3::from(m0);
    let vm1 = Vector3::from(m1);
    let vm2 = Vector3::from(m2);

    let max_edge = (v1 - v0).norm().max((v2 - v1).norm()).max((v0 - v2).norm());
    if max_edge <= 1e-18 {
        return ([0.0; 6], 0.0);
    }

    let mid_err = (vm0 - (v0 + v1) * 0.5)
        .norm()
        .max((vm1 - (v1 + v2) * 0.5).norm())
        .max((vm2 - (v2 + v0) * 0.5).norm());

    if mid_err <= 1e-12 * max_edge {
        let area = 0.5 * (v1 - v0).cross(&(v2 - v0)).norm();
        let share = area / 3.0;
        return ([0.0, 0.0, 0.0, share, share, share], area);
    }

    let pos6 = [v0, v1, v2, vm0, vm1, vm2];
    let mut weights = [0.0_f64; 6];
    let mut total_area = 0.0_f64;

    for &([l0, l1, l2], w_q) in &T6_DUNAVANT_GP6 {
        let n6 = [
            l0 * (2.0 * l0 - 1.0),
            l1 * (2.0 * l1 - 1.0),
            l2 * (2.0 * l2 - 1.0),
            4.0 * l0 * l1,
            4.0 * l1 * l2,
            4.0 * l2 * l0,
        ];
        let dn_dxi = [
            -(4.0 * l0 - 1.0),
            4.0 * l1 - 1.0,
            0.0,
            4.0 * (l0 - l1),
            4.0 * l2,
            -4.0 * l2,
        ];
        let dn_deta = [
            -(4.0 * l0 - 1.0),
            0.0,
            4.0 * l2 - 1.0,
            -4.0 * l1,
            4.0 * l1,
            4.0 * (l0 - l2),
        ];

        let mut dx_dxi = Vector3::zeros();
        let mut dx_deta = Vector3::zeros();
        for k in 0..6 {
            dx_dxi += pos6[k] * dn_dxi[k];
            dx_deta += pos6[k] * dn_deta[k];
        }
        let da = dx_dxi.cross(&dx_deta).norm() * w_q;
        total_area += da;
        for k in 0..6 {
            weights[k] += n6[k] * da;
        }
    }

    (weights, total_area)
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

/// Estimates the characteristic element step `[dx, dy, dz]` of `mesh` from its tetrahedral edges.
///
/// For axis-aligned voxel meshes this recovers the grid cell spacing. For boundary-conforming
/// snapped/octree meshes (where curved edges can have arbitrarily small projections on individual
/// coordinate axes), returns the mean corner edge length isotropically.
fn estimate_voxel_steps(mesh: &TetrahedralMesh) -> [f64; 3] {
    if mesh.is_boundary_conforming() && !mesh.elements.is_empty() {
        let mut sum_len = 0.0;
        let mut count = 0usize;
        for elem in &mesh.elements {
            let idx = elem.node_indices;
            for (u, v) in [
                (idx[0], idx[1]),
                (idx[0], idx[2]),
                (idx[0], idx[3]),
                (idx[1], idx[2]),
                (idx[1], idx[3]),
                (idx[2], idx[3]),
            ] {
                let pu = mesh.nodes[u].position;
                let pv = mesh.nodes[v].position;
                let d = ((pu[0] - pv[0]).powi(2)
                    + (pu[1] - pv[1]).powi(2)
                    + (pu[2] - pv[2]).powi(2))
                .sqrt();
                if d > 1e-9 {
                    sum_len += d;
                    count += 1;
                }
            }
        }
        if count > 0 {
            let h = (sum_len / count as f64).max(1e-5);
            return [h, h, h];
        }
    }

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
    // Voxel boundary nodes on curved/staircase surfaces lie up to 0.5 * h_diag (~0.866 * h_max) from the smooth CAD surface;
    // on boundary-conforming snapped meshes, nodes lie tightly on the CAD surface.
    let search_tol = if mesh.is_boundary_conforming() {
        user_tolerance.max(0.35 * h_max).max(1e-4)
    } else {
        user_tolerance
            .max(0.95 * h_max)
            .max(0.60 * h_diag)
            .max(1e-4)
    };

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
                let m0 = mesh.nodes[mids[0]].position;
                let m1 = mesh.nodes[mids[1]].position;
                let m2 = mesh.nodes[mids[2]].position;
                let (flat_area, face_n) = triangle_area_and_normal(p0, p1, p2);
                if flat_area > 1e-18 {
                    // Quadratic T6 center at (1/3, 1/3, 1/3): N_corner = -1/9, N_mid = +4/9
                    let quad_centroid = [
                        -(p0[0] + p1[0] + p2[0]) / 9.0 + 4.0 * (m0[0] + m1[0] + m2[0]) / 9.0,
                        -(p0[1] + p1[1] + p2[1]) / 9.0 + 4.0 * (m0[1] + m1[1] + m2[1]) / 9.0,
                        -(p0[2] + p1[2] + p2[2]) / 9.0 + 4.0 * (m0[2] + m1[2] + m2[2]) / 9.0,
                    ];
                    if is_planar {
                        if Vector3::from(face_n).dot(&avg_normal).abs() < 0.35 {
                            continue;
                        }
                        let (tri_dist, _) = closest_cad_triangle_distance_and_normal(
                            quad_centroid,
                            cad_triangles,
                            &tri_normals,
                        );
                        if tri_dist > search_tol {
                            continue;
                        }
                    } else {
                        let (tri_dist, cad_n) = closest_cad_triangle_distance_and_normal(
                            quad_centroid,
                            cad_triangles,
                            &tri_normals,
                        );
                        if tri_dist > search_tol {
                            continue;
                        }
                        if mesh.is_boundary_conforming() {
                            let align = Vector3::from(face_n).dot(&Vector3::from(cad_n)).abs();
                            if align < 0.50 {
                                continue;
                            }
                        }
                    }
                    let (w6, face_area) = t6_consistent_area_weights(p0, p1, p2, m0, m1, m2);
                    if face_area > 1e-18 {
                        let nodes6 = [
                            corners[0], corners[1], corners[2], mids[0], mids[1], mids[2],
                        ];
                        for k in 0..6 {
                            if w6[k] != 0.0 {
                                *area_weights.entry(nodes6[k]).or_insert(0.0) += w6[k];
                            }
                        }
                        any_tet10_face_matched = true;
                    }
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
            let (area, face_n) = triangle_area_and_normal(p0, p1, p2);
            if area > 1e-18 {
                let centroid = [
                    (p0[0] + p1[0] + p2[0]) / 3.0,
                    (p0[1] + p1[1] + p2[1]) / 3.0,
                    (p0[2] + p1[2] + p2[2]) / 3.0,
                ];
                if is_planar {
                    if Vector3::from(face_n).dot(&avg_normal).abs() < 0.35 {
                        continue;
                    }
                    let (tri_dist, _) = closest_cad_triangle_distance_and_normal(
                        centroid,
                        cad_triangles,
                        &tri_normals,
                    );
                    if tri_dist > search_tol {
                        continue;
                    }
                } else {
                    // For non-planar faces (e.g. cylindrical hole wall), ensure the surface triangle centroid
                    // is also close to the target CAD face so flat cap triangles bridging across a thin rim are excluded.
                    let (tri_dist, cad_n) = closest_cad_triangle_distance_and_normal(
                        centroid,
                        cad_triangles,
                        &tri_normals,
                    );
                    if tri_dist > search_tol {
                        continue;
                    }
                    if mesh.is_boundary_conforming() {
                        let align = Vector3::from(face_n).dot(&Vector3::from(cad_n)).abs();
                        if align < 0.50 {
                            continue;
                        }
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

/// Finds the closest CAD triangle to `p` and returns `(min_distance, unit_outward_normal)`.
fn closest_cad_triangle_distance_and_normal(
    p: [f64; 3],
    cad_triangles: &[[[f64; 3]; 3]],
    tri_normals: &[(f64, [f64; 3])],
) -> (f64, [f64; 3]) {
    let mut best_dist = f64::INFINITY;
    let mut best_normal = [0.0, 0.0, 1.0];
    for (idx, t) in cad_triangles.iter().enumerate() {
        let d = point_to_triangle_distance_3d(p, t[0], t[1], t[2]);
        if d < best_dist {
            best_dist = d;
            if tri_normals[idx].0 > 1e-16 {
                best_normal = tri_normals[idx].1;
            }
        }
    }
    (best_dist, best_normal)
}

/// Integrates consistent nodal forces for a uniform resultant force `total_force` applied over
/// `cad_triangles` on a boundary-conforming `TetrahedralMesh` (`P5-T12`).
///
/// - For quadratic `TET10` meshes, uses exact `T6` virtual-work weights (`t6_consistent_area_weights`:
///   `0` on corners and `A_T / 3` on mid-edge nodes for straight faces, 6-point Dunavant quadrature
///   for curved/shifted-midpoint faces) over each matched 6-node exterior boundary face.
/// - For linear `TET4` meshes, integrates exact `T3` triangle area weights (`A_T / 3` per vertex)
///   over each matched normal-aligned exterior boundary face.
///
/// Returns `true` if at least one conforming exterior face matched `cad_triangles`.
fn integrate_conforming_surface_force(
    mesh: &TetrahedralMesh,
    cad_triangles: &[[[f64; 3]; 3]],
    user_tolerance: f64,
    total_force: [f64; 3],
    nodal_forces: &mut [[f64; 3]],
) -> bool {
    if cad_triangles.is_empty() || mesh.is_empty() {
        return false;
    }

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

    let steps = estimate_voxel_steps(mesh);
    let h_max = steps[0].max(steps[1]).max(steps[2]);
    let face_tol = user_tolerance.max(0.35 * h_max).max(1e-4);

    let plane_coplanar_tol = 1e-4 * h_max.max(1.0);
    let is_planar = total_cad_area > 1e-16
        && cad_triangles.iter().zip(tri_normals.iter()).all(|(t, &(area, n))| {
            if area <= 1e-16 {
                return true;
            }
            if Vector3::from(n).dot(&avg_normal) < 0.998 {
                return false;
            }
            t.iter().all(|v| {
                ((Vector3::from(*v) - cad_centroid).dot(&avg_normal)).abs() <= plane_coplanar_tol
            })
        });

    if mesh.is_tet10() {
        let mut matched: Vec<(f64, [usize; 6], [f64; 6], f64)> = Vec::new();
        for (corners, mids) in mesh.extract_surface_tet10_faces() {
            let p0 = mesh.nodes[corners[0]].position;
            let p1 = mesh.nodes[corners[1]].position;
            let p2 = mesh.nodes[corners[2]].position;
            let m0 = mesh.nodes[mids[0]].position;
            let m1 = mesh.nodes[mids[1]].position;
            let m2 = mesh.nodes[mids[2]].position;
            let (flat_area, face_n) = triangle_area_and_normal(p0, p1, p2);
            if flat_area <= 1e-18 {
                continue;
            }

            let e01 = ((p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) + (p1[2] - p0[2]).powi(2)).sqrt();
            let e12 = ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2) + (p2[2] - p1[2]).powi(2)).sqrt();
            let e20 = ((p0[0] - p2[0]).powi(2) + (p0[1] - p2[1]).powi(2) + (p0[2] - p2[2]).powi(2)).sqrt();
            let local_edge = e01.max(e12).max(e20);
            let local_tol = user_tolerance.max(face_tol).max(0.35 * local_edge).max(2.5e-4);

            let quad_centroid = [
                -(p0[0] + p1[0] + p2[0]) / 9.0 + 4.0 * (m0[0] + m1[0] + m2[0]) / 9.0,
                -(p0[1] + p1[1] + p2[1]) / 9.0 + 4.0 * (m0[1] + m1[1] + m2[1]) / 9.0,
                -(p0[2] + p1[2] + p2[2]) / 9.0 + 4.0 * (m0[2] + m1[2] + m2[2]) / 9.0,
            ];
            let (d_cent, cad_n) =
                closest_cad_triangle_distance_and_normal(quad_centroid, cad_triangles, &tri_normals);
            if d_cent > local_tol {
                continue;
            }
            if Vector3::from(face_n).dot(&Vector3::from(cad_n)).abs() < 0.35 {
                continue;
            }
            let corners_close = [p0, p1, p2].iter().all(|&pt| {
                cad_triangles
                    .iter()
                    .map(|t| point_to_triangle_distance_3d(pt, t[0], t[1], t[2]))
                    .fold(f64::INFINITY, f64::min)
                    <= local_tol * 1.25
            });
            if !corners_close {
                continue;
            }

            let (w6, face_area) = t6_consistent_area_weights(p0, p1, p2, m0, m1, m2);
            if face_area <= 1e-18 {
                continue;
            }
            let nd = ((Vector3::from(quad_centroid) - cad_centroid).dot(&avg_normal)).abs();
            let nodes6 = [
                corners[0], corners[1], corners[2], mids[0], mids[1], mids[2],
            ];
            matched.push((nd, nodes6, w6, face_area));
        }

        if matched.is_empty() {
            return false;
        }
        if is_planar {
            let min_nd = matched.iter().map(|m| m.0).fold(f64::INFINITY, f64::min);
            let plane_tol = min_nd + user_tolerance.max(1e-4);
            if matched.iter().any(|m| m.0 <= plane_tol) {
                matched.retain(|m| m.0 <= plane_tol);
            }
        }

        let mut node_weights: HashMap<usize, f64> = HashMap::new();
        let mut total_w = 0.0;
        for (_, nodes6, w6, face_area) in matched {
            for k in 0..6 {
                if w6[k] != 0.0 {
                    *node_weights.entry(nodes6[k]).or_insert(0.0) += w6[k];
                }
            }
            total_w += face_area;
        }
        if total_w > 1e-18 {
            for (nid, w) in node_weights {
                let frac = w / total_w;
                nodal_forces[nid][0] += total_force[0] * frac;
                nodal_forces[nid][1] += total_force[1] * frac;
                nodal_forces[nid][2] += total_force[2] * frac;
            }
            return true;
        }
        return false;
    }

    // Boundary-conforming linear TET4 surface force integration (exact T3 A_T / 3 weights)
    let mut matched: Vec<(f64, [usize; 3], f64)> = Vec::new();
    for [n0, n1, n2] in mesh.extract_surface_triangles() {
        let p0 = mesh.nodes[n0].position;
        let p1 = mesh.nodes[n1].position;
        let p2 = mesh.nodes[n2].position;
        let (area, face_n) = triangle_area_and_normal(p0, p1, p2);
        if area <= 1e-18 {
            continue;
        }
        let e01 = ((p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) + (p1[2] - p0[2]).powi(2)).sqrt();
        let e12 = ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2) + (p2[2] - p1[2]).powi(2)).sqrt();
        let e20 = ((p0[0] - p2[0]).powi(2) + (p0[1] - p2[1]).powi(2) + (p0[2] - p2[2]).powi(2)).sqrt();
        let local_edge = e01.max(e12).max(e20);
        let local_tol = user_tolerance.max(face_tol).max(0.35 * local_edge).max(2.5e-4);

        let centroid = [
            (p0[0] + p1[0] + p2[0]) / 3.0,
            (p0[1] + p1[1] + p2[1]) / 3.0,
            (p0[2] + p1[2] + p2[2]) / 3.0,
        ];
        let (d_cent, cad_n) =
            closest_cad_triangle_distance_and_normal(centroid, cad_triangles, &tri_normals);
        if d_cent > local_tol {
            continue;
        }
        if Vector3::from(face_n).dot(&Vector3::from(cad_n)).abs() < 0.35 {
            continue;
        }
        let corners_close = [p0, p1, p2].iter().all(|&pt| {
            cad_triangles
                .iter()
                .map(|t| point_to_triangle_distance_3d(pt, t[0], t[1], t[2]))
                .fold(f64::INFINITY, f64::min)
                <= local_tol * 1.25
        });
        if !corners_close {
            continue;
        }

        let nd = ((Vector3::from(centroid) - cad_centroid).dot(&avg_normal)).abs();
        matched.push((nd, [n0, n1, n2], area));
    }

    if matched.is_empty() {
        return false;
    }
    if is_planar {
        let min_nd = matched.iter().map(|m| m.0).fold(f64::INFINITY, f64::min);
        let plane_tol = min_nd + user_tolerance.max(1e-4);
        if matched.iter().any(|m| m.0 <= plane_tol) {
            matched.retain(|m| m.0 <= plane_tol);
        }
    }

    let mut node_weights: HashMap<usize, f64> = HashMap::new();
    let mut total_w = 0.0;
    for (_, [n0, n1, n2], area) in matched {
        let share = area / 3.0;
        *node_weights.entry(n0).or_insert(0.0) += share;
        *node_weights.entry(n1).or_insert(0.0) += share;
        *node_weights.entry(n2).or_insert(0.0) += share;
        total_w += area;
    }
    if total_w > 1e-18 {
        for (nid, w) in node_weights {
            let frac = w / total_w;
            nodal_forces[nid][0] += total_force[0] * frac;
            nodal_forces[nid][1] += total_force[1] * frac;
            nodal_forces[nid][2] += total_force[2] * frac;
        }
        true
    } else {
        false
    }
}

/// Integrates consistent nodal forces for a uniform normal pressure `pressure_pa` applied over
/// `cad_triangles` on a boundary-conforming `TetrahedralMesh`.
///
/// - For curved quadratic `TET10` meshes, uses exact degree-4 (6-point symmetric Dunavant)
///   Gauss quadrature over each matched 6-node exterior boundary face (`T6`), computing
///   $\mathbf{f}_k = \int_{\hat{T}} N_k^{(6)}(\xi, \eta) (-p) (\mathbf{x}_\xi \times \mathbf{x}_\eta) d\xi d\eta$.
/// - For boundary-conforming linear `TET4` meshes, integrates exact triangular face forces
///   using normal-aligned exterior faces.
///
/// Returns `true` if at least one conforming exterior face matched `cad_triangles`.
fn integrate_conforming_surface_pressure(
    mesh: &TetrahedralMesh,
    cad_triangles: &[[[f64; 3]; 3]],
    user_tolerance: f64,
    pressure_pa: f64,
    nodal_forces: &mut [[f64; 3]],
) -> bool {
    if cad_triangles.is_empty() || mesh.is_empty() {
        return false;
    }

    let tri_normals: Vec<(f64, [f64; 3])> = cad_triangles
        .iter()
        .map(|t| triangle_area_and_normal(t[0], t[1], t[2]))
        .collect();

    let steps = estimate_voxel_steps(mesh);
    let h_max = steps[0].max(steps[1]).max(steps[2]);
    let face_tol = user_tolerance.max(0.35 * h_max).max(1e-4);

    if mesh.is_tet10() {
        let faces6 = mesh.extract_surface_tet10_faces();
        let mut matched_faces = 0usize;

        for (corners, mids) in faces6 {
            let p0 = mesh.nodes[corners[0]].position;
            let p1 = mesh.nodes[corners[1]].position;
            let p2 = mesh.nodes[corners[2]].position;
            let m0 = mesh.nodes[mids[0]].position;
            let m1 = mesh.nodes[mids[1]].position;
            let m2 = mesh.nodes[mids[2]].position;
            let (area, face_n) = triangle_area_and_normal(p0, p1, p2);
            if area <= 1e-18 {
                continue;
            }

            let e01 = ((p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) + (p1[2] - p0[2]).powi(2)).sqrt();
            let e12 = ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2) + (p2[2] - p1[2]).powi(2)).sqrt();
            let e20 = ((p0[0] - p2[0]).powi(2) + (p0[1] - p2[1]).powi(2) + (p0[2] - p2[2]).powi(2)).sqrt();
            let local_edge = e01.max(e12).max(e20);
            let local_tol = user_tolerance.max(face_tol).max(0.35 * local_edge).max(2.5e-4);

            // Evaluate the curved quadratic T6 face center at (1/3, 1/3, 1/3):
            // N_corner = -1/9, N_mid = +4/9 -> sits accurately on curved CAD faces without chord sagitta.
            let quad_centroid = [
                -(p0[0] + p1[0] + p2[0]) / 9.0 + 4.0 * (m0[0] + m1[0] + m2[0]) / 9.0,
                -(p0[1] + p1[1] + p2[1]) / 9.0 + 4.0 * (m0[1] + m1[1] + m2[1]) / 9.0,
                -(p0[2] + p1[2] + p2[2]) / 9.0 + 4.0 * (m0[2] + m1[2] + m2[2]) / 9.0,
            ];
            let (d_cent, cad_n) =
                closest_cad_triangle_distance_and_normal(quad_centroid, cad_triangles, &tri_normals);
            if d_cent > local_tol {
                continue;
            }

            let face_n_vec = Vector3::from(face_n);
            let cad_n_vec = Vector3::from(cad_n);
            if face_n_vec.dot(&cad_n_vec).abs() < 0.35 {
                continue;
            }

            // Also verify all 3 corners lie close to the target CAD surface
            let corners_close = [p0, p1, p2].iter().all(|&pt| {
                cad_triangles
                    .iter()
                    .map(|t| point_to_triangle_distance_3d(pt, t[0], t[1], t[2]))
                    .fold(f64::INFINITY, f64::min)
                    <= local_tol * 1.25
            });
            if !corners_close {
                continue;
            }

            let nodes6 = [
                corners[0], corners[1], corners[2], mids[0], mids[1], mids[2],
            ];
            let pos6 = [
                Vector3::from(p0),
                Vector3::from(p1),
                Vector3::from(p2),
                Vector3::from(mesh.nodes[mids[0]].position),
                Vector3::from(mesh.nodes[mids[1]].position),
                Vector3::from(mesh.nodes[mids[2]].position),
            ];

            // Determine consistent outward sign relative to CAD outward normal
            let orient_sign = if face_n_vec.dot(&cad_n_vec) >= 0.0 {
                1.0
            } else {
                -1.0
            };

            for &([l0, l1, l2], w_q) in &T6_DUNAVANT_GP6 {
                let n6 = [
                    l0 * (2.0 * l0 - 1.0),
                    l1 * (2.0 * l1 - 1.0),
                    l2 * (2.0 * l2 - 1.0),
                    4.0 * l0 * l1,
                    4.0 * l1 * l2,
                    4.0 * l2 * l0,
                ];
                let dn_dxi = [
                    -(4.0 * l0 - 1.0),
                    4.0 * l1 - 1.0,
                    0.0,
                    4.0 * (l0 - l1),
                    4.0 * l2,
                    -4.0 * l2,
                ];
                let dn_deta = [
                    -(4.0 * l0 - 1.0),
                    0.0,
                    4.0 * l2 - 1.0,
                    -4.0 * l1,
                    4.0 * l1,
                    4.0 * (l0 - l2),
                ];

                let mut dx_dxi = Vector3::zeros();
                let mut dx_deta = Vector3::zeros();
                for k in 0..6 {
                    dx_dxi += pos6[k] * dn_dxi[k];
                    dx_deta += pos6[k] * dn_deta[k];
                }
                let j_vec = dx_dxi.cross(&dx_deta) * orient_sign;
                // Positive pressure acts inward against outward surface normal (-n)
                let traction_da = j_vec * (-pressure_pa * w_q);

                for k in 0..6 {
                    let f_k = traction_da * n6[k];
                    let nid = nodes6[k];
                    nodal_forces[nid][0] += f_k.x;
                    nodal_forces[nid][1] += f_k.y;
                    nodal_forces[nid][2] += f_k.z;
                }
            }

            matched_faces += 1;
        }

        return matched_faces > 0;
    }

    // Boundary-conforming linear TET4 surface pressure integration
    let surf_tris = mesh.extract_surface_triangles();
    let mut matched_faces = 0usize;
    for &[n0, n1, n2] in &surf_tris {
        let p0 = mesh.nodes[n0].position;
        let p1 = mesh.nodes[n1].position;
        let p2 = mesh.nodes[n2].position;
        let (area, face_n) = triangle_area_and_normal(p0, p1, p2);
        if area <= 1e-18 {
            continue;
        }
        let e01 = ((p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) + (p1[2] - p0[2]).powi(2)).sqrt();
        let e12 = ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2) + (p2[2] - p1[2]).powi(2)).sqrt();
        let e20 = ((p0[0] - p2[0]).powi(2) + (p0[1] - p2[1]).powi(2) + (p0[2] - p2[2]).powi(2)).sqrt();
        let local_edge = e01.max(e12).max(e20);
        let local_tol = user_tolerance.max(face_tol).max(0.35 * local_edge).max(2.5e-4);

        let centroid = [
            (p0[0] + p1[0] + p2[0]) / 3.0,
            (p0[1] + p1[1] + p2[1]) / 3.0,
            (p0[2] + p1[2] + p2[2]) / 3.0,
        ];
        let (d_cent, cad_n) =
            closest_cad_triangle_distance_and_normal(centroid, cad_triangles, &tri_normals);
        if d_cent > local_tol {
            continue;
        }
        let face_n_vec = Vector3::from(face_n);
        let cad_n_vec = Vector3::from(cad_n);
        if face_n_vec.dot(&cad_n_vec).abs() < 0.35 {
            continue;
        }
        let corners_close = [p0, p1, p2].iter().all(|&pt| {
            cad_triangles
                .iter()
                .map(|t| point_to_triangle_distance_3d(pt, t[0], t[1], t[2]))
                .fold(f64::INFINITY, f64::min)
                <= local_tol * 1.25
        });
        if !corners_close {
            continue;
        }

        let out_n = if face_n_vec.dot(&cad_n_vec) >= 0.0 {
            face_n_vec
        } else {
            -face_n_vec
        };
        let f_per_node = out_n * (-pressure_pa * area / 3.0);
        for nid in [n0, n1, n2] {
            nodal_forces[nid][0] += f_per_node.x;
            nodal_forces[nid][1] += f_per_node.y;
            nodal_forces[nid][2] += f_per_node.z;
        }
        matched_faces += 1;
    }

    matched_faces > 0
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

// ============================================================================
// P5-T4: Kinematic Hub Couplings (RBE2 coupling of deformable boundary nodes
// to physics joints & actuators without artificially gluing free regions)
// ============================================================================

/// Selector identifying which nodes of a deformable body belong to a rigid-link hub/bore/face
/// coupled to a mechanical joint or actuator (`P5-T4`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum HubNodeSelector {
    /// Selects all mesh nodes within perpendicular distance `max_radius` of the axis line
    /// passing through `pivot` along `axis` (ideal for gear shaft bores and bearing inner rings).
    BoreCylinder {
        /// Point on the cylinder axis in meters.
        pivot: [f64; 3],
        /// Axis direction vector (automatically normalized).
        axis: [f64; 3],
        /// Maximum radial distance from the axis in meters.
        max_radius: f64,
    },
    /// Selects all mesh nodes at perpendicular distance `>= min_radius` from the axis line.
    OuterCylinder {
        /// Point on the cylinder axis in meters.
        pivot: [f64; 3],
        /// Axis direction vector (automatically normalized).
        axis: [f64; 3],
        /// Minimum radial distance from the axis in meters.
        min_radius: f64,
    },
    /// Selects exterior surface nodes matching the given CAD surface triangles.
    SurfaceTriangles {
        /// Triangles `[v0, v1, v2]` in meters forming the coupled CAD face.
        triangles: Vec<[[f64; 3]; 3]>,
        /// Search distance tolerance in meters.
        #[serde(default = "default_tolerance")]
        tolerance: f64,
    },
    /// Selects mesh nodes lying on an axis-aligned bounding box face.
    AabbFace {
        /// Target AABB face.
        face: AabbFace,
        /// Search distance tolerance in meters.
        #[serde(default = "default_tolerance")]
        tolerance: f64,
    },
    /// Explicit list of mesh node indices.
    Nodes(Vec<usize>),
}

/// Actuation / resistance mode for a revolute shaft/hub coupling (`P5-T4`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum RevoluteHubDrive {
    /// Prescribed finite shaft rotation angle around the joint axis (radians).
    PrescribedAngle {
        /// Target rotation angle in radians relative to the reference configuration.
        angle_rad: f64,
    },
    /// Free scalar rotational pilot DOF driven by an applied shaft torque and optional
    /// torsional spring stiffness ($T_{\text{net}}(\theta) = T_{\text{app}} - k_\theta (\theta - \theta_{\text{ref}})$).
    Torque {
        /// Applied axial torque about the joint axis in N·m.
        torque_nm: f64,
        /// Optional torsional stiffness $k_\theta \ge 0$ in N·m/rad (e.g., driven load / shaft compliance).
        #[serde(default)]
        torsional_stiffness_nm_per_rad: f64,
        /// Reference angle $\theta_{\text{ref}}$ for the torsional spring in radians.
        #[serde(default)]
        reference_angle_rad: f64,
    },
}

/// Actuation / resistance mode for a prismatic slider/platen hub coupling (`P5-T4`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PrismaticHubDrive {
    /// Prescribed rigid translation along the prismatic axis (meters).
    PrescribedDisplacement {
        /// Target displacement along the joint axis in meters.
        displacement_m: f64,
    },
    /// Free scalar translational pilot DOF driven by an applied axial force and optional
    /// axial spring stiffness ($F_{\text{net}}(s) = F_{\text{app}} - k_s (s - s_{\text{ref}})$).
    Force {
        /// Applied axial force along the joint axis in Newtons.
        force_n: f64,
        /// Optional axial stiffness $k_s \ge 0$ in N/m.
        #[serde(default)]
        axial_stiffness_n_per_m: f64,
        /// Reference displacement $s_{\text{ref}}$ for the axial spring in meters.
        #[serde(default)]
        reference_disp_m: f64,
    },
}

/// Kinematic RBE2 coupling between a subset of deformable-body boundary nodes and a
/// 1-DOF or 0-DOF mechanical joint / actuator (`P5-T4`).
///
/// Only the selected hub/bore nodes are kinematically coupled; all remaining nodes (e.g. gear
/// teeth, elastic rims, contact patches) remain completely free to deform.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum HubCoupling {
    /// Revolute shaft/bore coupling with exact finite-rotation kinematics on $SO(3)$:
    /// $\mathbf{x}_i(\theta) = \mathbf{p} + \mathbf{R}(\hat{\mathbf{a}}, \theta)(\mathbf{X}_i - \mathbf{p})$.
    Revolute {
        /// Optional human-readable identifier (e.g. matching `JointHubCouplingSpec::joint_id`).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        name: Option<String>,
        /// Point on the rotation axis in meters.
        pivot: [f64; 3],
        /// Rotation axis direction (automatically normalized).
        axis: [f64; 3],
        /// Node selector identifying the hub/bore boundary nodes.
        selector: HubNodeSelector,
        /// Prescribed rotation or torque-driven pilot DOF.
        drive: RevoluteHubDrive,
    },
    /// Prismatic slider / rigid-platen coupling along `axis`.
    Prismatic {
        /// Optional human-readable identifier.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        name: Option<String>,
        /// Translation axis direction (automatically normalized).
        axis: [f64; 3],
        /// Node selector identifying the coupled boundary nodes.
        selector: HubNodeSelector,
        /// Prescribed displacement or force-driven pilot DOF.
        drive: PrismaticHubDrive,
        /// If `true`, transverse DOFs perpendicular to `axis` are locked to 0 (rigid slider).
        /// If `false` and `axis` is coordinate-aligned, only the axial DOF is coupled while
        /// transverse Poisson expansion remains free (frictionless rigid platen).
        #[serde(default = "default_lock_transverse")]
        lock_transverse: bool,
    },
    /// Rigidly fixed hub/bore (0 DOF, all 3 translational components fixed to 0).
    Fixed {
        /// Optional human-readable identifier.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        name: Option<String>,
        /// Node selector identifying the fixed boundary nodes.
        selector: HubNodeSelector,
    },
}

fn default_lock_transverse() -> bool {
    true
}

/// Resolved kinematic hub coupling with deduplicated mesh node indices and normalized geometry.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedHubCoupling {
    /// Optional human-readable name.
    pub name: Option<String>,
    /// Sorted unique mesh node indices participating in this coupling.
    pub node_ids: Vec<usize>,
    /// Normalized joint kind and parameters.
    pub kind: ResolvedHubCouplingKind,
}

/// Resolved geometry and drive details for [`ResolvedHubCoupling`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ResolvedHubCouplingKind {
    /// Revolute hub coupling with normalized unit `axis`.
    Revolute {
        /// Point on the rotation axis in meters.
        pivot: [f64; 3],
        /// Unit rotation axis vector.
        axis: [f64; 3],
        /// Revolute drive mode.
        drive: RevoluteHubDrive,
    },
    /// Prismatic hub coupling with normalized unit `axis`.
    Prismatic {
        /// Unit translation axis vector.
        axis: [f64; 3],
        /// Prismatic drive mode.
        drive: PrismaticHubDrive,
        /// Whether transverse DOFs are locked.
        lock_transverse: bool,
    },
    /// Fixed hub coupling.
    Fixed,
}

impl HubNodeSelector {
    /// Resolves the selector against `mesh`, returning sorted unique node IDs.
    pub fn resolve_nodes(&self, mesh: &TetrahedralMesh) -> Result<Vec<usize>, FeaError> {
        let total_nodes = mesh.node_count();
        if total_nodes == 0 {
            return Err(FeaError::EmptyMesh);
        }
        let mut nodes: Vec<usize> = match self {
            Self::BoreCylinder {
                pivot,
                axis,
                max_radius,
            } => {
                let p0 = Vector3::from(*pivot);
                let a_raw = Vector3::from(*axis);
                let a = if a_raw.norm() > 1e-12 {
                    a_raw.normalize()
                } else {
                    Vector3::z()
                };
                let r_lim = max_radius.max(0.0);
                mesh.nodes
                    .iter()
                    .filter(|n| {
                        let d = n.pos_vec() - p0;
                        let axial = d.dot(&a);
                        let radial = (d - a * axial).norm();
                        radial <= r_lim
                    })
                    .map(|n| n.id)
                    .collect()
            }
            Self::OuterCylinder {
                pivot,
                axis,
                min_radius,
            } => {
                let p0 = Vector3::from(*pivot);
                let a_raw = Vector3::from(*axis);
                let a = if a_raw.norm() > 1e-12 {
                    a_raw.normalize()
                } else {
                    Vector3::z()
                };
                let r_lim = min_radius.max(0.0);
                mesh.nodes
                    .iter()
                    .filter(|n| {
                        let d = n.pos_vec() - p0;
                        let axial = d.dot(&a);
                        let radial = (d - a * axial).norm();
                        radial >= r_lim
                    })
                    .map(|n| n.id)
                    .collect()
            }
            Self::SurfaceTriangles {
                triangles,
                tolerance,
            } => match_surface_nodes_and_weights(mesh, triangles, *tolerance)
                .into_iter()
                .map(|m| m.node_id)
                .collect(),
            Self::AabbFace { face, tolerance } => {
                let aabb = mesh.bounding_box();
                find_aabb_face_nodes(mesh, &aabb, *face, *tolerance)
            }
            Self::Nodes(ids) => {
                for &nid in ids {
                    if nid >= total_nodes {
                        return Err(FeaError::NodeOutOfBounds {
                            node_id: nid,
                            total_nodes,
                        });
                    }
                }
                ids.clone()
            }
        };

        nodes.sort_unstable();
        nodes.dedup();
        if nodes.is_empty() {
            return Err(FeaError::NoBoundaryNodesMatched { tolerance: 0.0 });
        }
        Ok(nodes)
    }
}

impl HubCoupling {
    /// Resolves this hub coupling's node set and normalizes its joint axis against `mesh`.
    pub fn resolve(&self, mesh: &TetrahedralMesh) -> Result<ResolvedHubCoupling, FeaError> {
        match self {
            Self::Revolute {
                name,
                pivot,
                axis,
                selector,
                drive,
            } => {
                let node_ids = selector.resolve_nodes(mesh)?;
                let a_raw = Vector3::from(*axis);
                let a_norm = a_raw.norm();
                if a_norm <= 1e-12 {
                    return Err(FeaError::General(
                        "Revolute HubCoupling axis must have non-zero length".to_string(),
                    ));
                }
                let unit_a = a_raw / a_norm;
                Ok(ResolvedHubCoupling {
                    name: name.clone(),
                    node_ids,
                    kind: ResolvedHubCouplingKind::Revolute {
                        pivot: *pivot,
                        axis: [unit_a.x, unit_a.y, unit_a.z],
                        drive: *drive,
                    },
                })
            }
            Self::Prismatic {
                name,
                axis,
                selector,
                drive,
                lock_transverse,
            } => {
                let node_ids = selector.resolve_nodes(mesh)?;
                let a_raw = Vector3::from(*axis);
                let a_norm = a_raw.norm();
                if a_norm <= 1e-12 {
                    return Err(FeaError::General(
                        "Prismatic HubCoupling axis must have non-zero length".to_string(),
                    ));
                }
                let unit_a = a_raw / a_norm;
                Ok(ResolvedHubCoupling {
                    name: name.clone(),
                    node_ids,
                    kind: ResolvedHubCouplingKind::Prismatic {
                        axis: [unit_a.x, unit_a.y, unit_a.z],
                        drive: *drive,
                        lock_transverse: *lock_transverse,
                    },
                })
            }
            Self::Fixed { name, selector } => {
                let node_ids = selector.resolve_nodes(mesh)?;
                Ok(ResolvedHubCoupling {
                    name: name.clone(),
                    node_ids,
                    kind: ResolvedHubCouplingKind::Fixed,
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conforming_mesh::ConformingMeshOptions;
    use crate::elasticity::J2MaterialParams;
    use crate::solver::{NonlinearSolverOptions, SolverOptions};
    use crate::tetra_mesh::{ElementOrder, MeshDensity};
    use crate::{analyze_mesh, analyze_mesh_nonlinear_history, NonlinearLoadStage};
    use mcp_cad_core::material::Material;
    use mcp_cad_core::primitives::create_box;

    #[test]
    fn test_p5_t12_t6_consistent_area_weights_flat_and_curved() {
        // 1. Flat straight-sided T6 triangle of area 0.5 * 0.04 * 0.03 = 6e-4 m²
        let p0 = [0.0, 0.0, 0.0];
        let p1 = [0.04, 0.0, 0.0];
        let p2 = [0.0, 0.03, 0.0];
        let m0 = [0.02, 0.0, 0.0];
        let m1 = [0.02, 0.015, 0.0];
        let m2 = [0.0, 0.015, 0.0];
        let (w_flat, area_flat) = t6_consistent_area_weights(p0, p1, p2, m0, m1, m2);
        let expected_area = 0.5 * 0.04 * 0.03;
        assert!((area_flat - expected_area).abs() < 1e-15);
        for (k, &w) in w_flat.iter().enumerate().take(3) {
            assert_eq!(w, 0.0, "Flat T6 corner weight {k} must be exact 0.0");
        }
        for (k, &w) in w_flat.iter().enumerate().skip(3) {
            assert!(
                (w - expected_area / 3.0).abs() < 1e-15,
                "Flat T6 mid-edge weight {k} must equal A/3"
            );
        }

        // 2. Curved T6 triangle lifted parabolically along Z: partition of unity sum(w_k) == A_curved > A_flat
        let m0_curved = [0.02, 0.0, 0.004];
        let m1_curved = [0.02, 0.015, 0.004];
        let m2_curved = [0.0, 0.015, 0.004];
        let (w_curved, area_curved) =
            t6_consistent_area_weights(p0, p1, p2, m0_curved, m1_curved, m2_curved);
        assert!(
            area_curved > area_flat,
            "Curved T6 patch must have larger area ({area_curved}) than flat chord ({area_flat})"
        );
        let w_sum: f64 = w_curved.iter().sum();
        assert!(
            (w_sum - area_curved).abs() / area_curved < 1e-12,
            "Curved T6 weights must satisfy partition of unity: sum={w_sum} vs area={area_curved}"
        );
    }

    #[test]
    fn test_p5_t12_uniaxial_tension_uniform_stress_aabb_and_named_face_tet4_and_tet10() {
        let length = 0.12_f64;
        let width = 0.04_f64;
        let height = 0.03_f64;
        let area = width * height;
        let force_x = 36_000.0_f64; // sigma_xx = 30.0 MPa
        let expected_sigma_pa = force_x / area;
        let material = Material::new(
            "steel_p5t12",
            205.0,
            0.30,
            250.0,
            7850.0,
            "Test structural steel",
        );
        let expected_ux_m = expected_sigma_pa * length / material.youngs_modulus_pa();

        // CAD box centered at origin: X in [-L/2, +L/2], Y in [-W/2, +W/2], Z in [-H/2, +H/2]
        let box_cad = create_box(length, width, height);
        let right_face = box_cad
            .faces
            .iter()
            .find(|f| f.role == "right")
            .expect("Box must have 'right' named face");

        let min = [-length * 0.5, -width * 0.5, -height * 0.5];
        let max = [length * 0.5, width * 0.5, height * 0.5];
        let tet4_structured = TetrahedralMesh::from_box(min, max, 4, 2, 2).unwrap();
        let tet10_structured = tet4_structured.to_tet10();

        let mut conf_tet4_opts = ConformingMeshOptions::high_accuracy(MeshDensity::Coarse);
        conf_tet4_opts.element_order = ElementOrder::Linear;
        let tet4_conforming =
            TetrahedralMesh::from_mesh_conforming(&box_cad, &conf_tet4_opts).unwrap();
        let conf_tet10_opts = ConformingMeshOptions::high_accuracy(MeshDensity::Coarse);
        let tet10_conforming =
            TetrahedralMesh::from_mesh_conforming(&box_cad, &conf_tet10_opts).unwrap();

        let solver_opts = SolverOptions {
            tolerance: 1e-12,
            max_iterations: 8000,
        };

        let cases: [(&str, &TetrahedralMesh); 4] = [
            ("structured_TET4", &tet4_structured),
            ("structured_TET10", &tet10_structured),
            ("conforming_TET4", &tet4_conforming),
            ("conforming_TET10", &tet10_conforming),
        ];

        for (mesh_label, mesh) in cases {
            for load_kind in ["aabb_face_force", "named_face_force"] {
                let mut bc = BoundaryConditions::new();
                // Roller BCs allowing exact uniform Poisson contraction without clamping distortion
                bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
                    face: AabbFace::MinX,
                    tolerance: 1e-6,
                    components: DofComponent::X,
                });
                bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
                    face: AabbFace::MinY,
                    tolerance: 1e-6,
                    components: DofComponent::Y,
                });
                bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
                    face: AabbFace::MinZ,
                    tolerance: 1e-6,
                    components: DofComponent::Z,
                });

                if load_kind == "aabb_face_force" {
                    bc.add_aabb_face_force(AabbFace::MaxX, 1e-6, [force_x, 0.0, 0.0]);
                } else {
                    bc.add_named_face_force(
                        &box_cad,
                        right_face.id,
                        Some(right_face.name.clone()),
                        1e-6,
                        [force_x, 0.0, 0.0],
                    );
                }

                // Verify exact resultant force conservation and T6 zero-corner property on planar face
                let resolved = bc.resolve(mesh).expect("Boundary resolution must succeed");
                let sum_fx: f64 = resolved.nodal_forces.iter().map(|f| f[0]).sum();
                let sum_fy: f64 = resolved.nodal_forces.iter().map(|f| f[1]).sum();
                let sum_fz: f64 = resolved.nodal_forces.iter().map(|f| f[2]).sum();
                assert!(
                    (sum_fx - force_x).abs() / force_x < 1e-12
                        && sum_fy.abs() < 1e-12
                        && sum_fz.abs() < 1e-12,
                    "{mesh_label}/{load_kind}: resultant force mismatch [{sum_fx}, {sum_fy}, {sum_fz}]"
                );
                if mesh.is_tet10() {
                    let mut corner_ids = HashSet::new();
                    for elem in &mesh.elements {
                        for &cid in &elem.node_indices {
                            corner_ids.insert(cid);
                        }
                    }
                    for cid in corner_ids {
                        assert!(
                            resolved.nodal_forces[cid][0].abs() < 1e-12,
                            "{mesh_label}/{load_kind}: flat T6 corner node {cid} must carry zero force, got {}",
                            resolved.nodal_forces[cid][0]
                        );
                    }
                }

                let report = analyze_mesh(
                    format!("p5t12_{mesh_label}_{load_kind}"),
                    mesh,
                    &material,
                    &bc,
                    Some(solver_opts),
                )
                .expect("Uniaxial FEA solve must succeed");

                let max_nodal_err = report
                    .nodal_von_mises_stresses
                    .iter()
                    .map(|&s| (s - expected_sigma_pa).abs() / expected_sigma_pa)
                    .fold(0.0_f64, f64::max);
                let max_s = report
                    .nodal_von_mises_stresses
                    .iter()
                    .copied()
                    .fold(0.0_f64, f64::max);
                let min_s = report
                    .nodal_von_mises_stresses
                    .iter()
                    .copied()
                    .fold(f64::INFINITY, f64::min);

                assert!(
                    max_nodal_err < 1e-6,
                    "{mesh_label}/{load_kind}: nodal stress non-uniformity {max_nodal_err:.3e} >= 1e-6 (max={:.4} MPa, min={:.4} MPa, expected={:.4} MPa)",
                    max_s / 1e6,
                    min_s / 1e6,
                    expected_sigma_pa / 1e6
                );
                assert!(
                    (report.max_stress_von_mises_pa - expected_sigma_pa).abs() / expected_sigma_pa
                        < 1e-6,
                    "{mesh_label}/{load_kind}: max_stress_von_mises_pa {:.4} MPa != expected {:.4} MPa",
                    report.max_stress_von_mises_pa / 1e6,
                    expected_sigma_pa / 1e6
                );

                // Verify exact analytical 3D uniaxial displacement field on every node:
                // ux = (sigma/E)*(x - x_min), uy = -nu*(sigma/E)*(y - y_min), uz = -nu*(sigma/E)*(z - z_min)
                let eps_x = expected_sigma_pa / material.youngs_modulus_pa();
                let eps_t = -material.poissons_ratio * eps_x;
                let max_disp_err = mesh
                    .nodes
                    .iter()
                    .enumerate()
                    .map(|(nid, node)| {
                        let u = report.nodal_displacements[nid];
                        let ex = eps_x * (node.position[0] - min[0]);
                        let ey = eps_t * (node.position[1] - min[1]);
                        let ez = eps_t * (node.position[2] - min[2]);
                        let err = ((u[0] - ex).powi(2) + (u[1] - ey).powi(2) + (u[2] - ez).powi(2))
                            .sqrt();
                        err / expected_ux_m
                    })
                    .fold(0.0_f64, f64::max);
                assert!(
                    max_disp_err < 1e-6,
                    "{mesh_label}/{load_kind}: 3D displacement field error {max_disp_err:.3e} >= 1e-6"
                );
            }
        }
    }

    #[test]
    fn test_p5_t12_elastoplastic_uniaxial_springback_zero_spurious_corner_residual_stress() {
        // Verify that loading past sigma_y and unloading with add_aabb_face_force and
        // add_named_face_force produces zero spurious residual corner stress on both TET4 and TET10.
        let length = 0.08_f64;
        let width = 0.02_f64;
        let height = 0.02_f64;
        let area = width * height;
        let sigma_target_pa = 300.0e6; // > sigma_y = 250 MPa
        let force_x = sigma_target_pa * area;
        let h_pa = 5.0e9;

        let mut mat = Material::new("steel_p5t12_ep", 200.0, 0.30, 250.0, 7850.0, "J2 steel");
        mat.hardening_modulus_pa = h_pa;
        mat.ultimate_strength_pa = 600.0e6;
        let params = J2MaterialParams::from_material(&mat).expect("valid J2 params");

        let box_cad = create_box(length, width, height);
        let right_face = box_cad
            .faces
            .iter()
            .find(|f| f.role == "right")
            .expect("Box must have 'right' face");

        let min = [-length * 0.5, -width * 0.5, -height * 0.5];
        let max = [length * 0.5, width * 0.5, height * 0.5];
        let tet4 = TetrahedralMesh::from_box(min, max, 2, 2, 2).unwrap();
        let tet10 = tet4.to_tet10();

        let expected_eps_p = (sigma_target_pa - params.yield_strength_pa) / h_pa;
        let opts = NonlinearSolverOptions {
            initial_load_steps: 4,
            corotated: false,
            force_tolerance: 1e-8,
            displacement_tolerance: 1e-9,
            ..Default::default()
        };

        for (mesh_label, mesh) in [("TET4", &tet4), ("TET10", &tet10)] {
            for load_kind in ["aabb_face_force", "named_face_force"] {
                let mut base_support = BoundaryConditions::new();
                base_support.dirichlet.push(DirichletConstraint::FixedAabbFace {
                    face: AabbFace::MinX,
                    tolerance: 1e-6,
                    components: DofComponent::X,
                });
                base_support.dirichlet.push(DirichletConstraint::FixedAabbFace {
                    face: AabbFace::MinY,
                    tolerance: 1e-6,
                    components: DofComponent::Y,
                });
                base_support.dirichlet.push(DirichletConstraint::FixedAabbFace {
                    face: AabbFace::MinZ,
                    tolerance: 1e-6,
                    components: DofComponent::Z,
                });

                let mut bc_load = base_support.clone();
                if load_kind == "aabb_face_force" {
                    bc_load.add_aabb_face_force(AabbFace::MaxX, 1e-6, [force_x, 0.0, 0.0]);
                } else {
                    bc_load.add_named_face_force(
                        &box_cad,
                        right_face.id,
                        Some(right_face.name.clone()),
                        1e-6,
                        [force_x, 0.0, 0.0],
                    );
                }

                let stages = [
                    NonlinearLoadStage::new(bc_load).with_steps(4).with_label("load"),
                    NonlinearLoadStage::new(base_support).with_steps(2).with_label("unload"),
                ];

                let res = analyze_mesh_nonlinear_history(
                    format!("p5t12_ep_{mesh_label}_{load_kind}"),
                    mesh,
                    &params,
                    &stages,
                    Some(opts),
                    None,
                )
                .expect("Elastoplastic uniaxial cycle must converge");

                let nl = res.report.nonlinear.as_ref().expect("nonlinear summary");
                assert!(
                    nl.converged_steps >= 6,
                    "{mesh_label}/{load_kind}: expected >= 6 converged steps, got {}",
                    nl.converged_steps
                );
                assert!(
                    nl.max_residual_ratio < 1e-6,
                    "{mesh_label}/{load_kind}: max residual ratio {:.3e} >= 1e-6",
                    nl.max_residual_ratio
                );
                assert!(
                    res.report.max_stress_von_mises_pa < 1e3,
                    "{mesh_label}/{load_kind}: spurious residual stress {:.3} MPa after full unload (expected ~0 Pa)",
                    res.report.max_stress_von_mises_pa / 1e6
                );
                let eps_p_err =
                    (nl.max_equivalent_plastic_strain - expected_eps_p).abs() / expected_eps_p;
                assert!(
                    eps_p_err < 1e-4,
                    "{mesh_label}/{load_kind}: plastic strain error {eps_p_err:.3e} >= 1e-4"
                );
            }
        }
    }
}




