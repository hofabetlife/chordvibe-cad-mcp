//! Multi-body deformable contact mechanics solver (`P5-T4`).
//!
//! Implements pure-Rust `f64` deformable-to-deformable and deformable-to-rigid contact with:
//! - **Augmented Lagrangian (Uzawa)** normal non-penetration enforcement ($g_n \ge 0$, $\lambda_n \ge 0$)
//! - **Isotropic dry Coulomb friction** ($\|\mathbf{t}_T\| \le \mu p_n$) with exact stick/slip return mapping
//! - **Node-to-Surface (`TET4`)** and **Mortar Gauss-Point-to-Segment (`TET10` / `TET4`)** discretizations
//! - **Dynamic deformed surface mesh + AABB-BVH update on every Newton–Raphson iteration** using
//!   outward-oriented exterior faces and Bærentzen–Aanæs angle-weighted pseudonormals
//! - **Exact finite-rotation RBE2 [`HubCoupling`]** connecting shaft bores / sliders / platens to
//!   1-DOF actuators or torque/force loads while keeping contact teeth and rims completely free
//! - Full coupling with Co-Rotated FEM ($\mathbf{F} = \mathbf{R}\mathbf{U}$) + $J_2$ elastoplasticity (`P5-T3`).

use std::collections::{BTreeMap, HashMap, HashSet};
use std::time::Instant;

use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::boundary::{
    BoundaryConditions, HubCoupling, PrismaticHubDrive, ResolvedBoundary, ResolvedHubCoupling,
    ResolvedHubCouplingKind, RevoluteHubDrive,
};
use crate::elasticity::{GaussPointState, J2MaterialParams};
use crate::element::{Tet10Element, Tet4Element, Vector12, Vector30};
use crate::error::FeaError;
use crate::solver::{CsrMatrix, NonlinearState, SolverOptions};
use crate::sparse_solver::{solve_spd_newton_robust, LinearSolverKind};
use crate::tetra_mesh::TetrahedralMesh;

const CONTACT_PAR_ELEMENT_THRESHOLD: usize = 64;

// ============================================================================
// Public Configuration & Result Data Types
// ============================================================================

/// Contact surface discretization scheme.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum ContactDiscretization {
    /// Automatically selects [`Self::NodeToSurface`] for linear `TET4` bodies and
    /// [`Self::MortarGauss`] for quadratic `TET10` bodies.
    #[default]
    Auto,
    /// Node-to-Surface (NTS) collocation weighted by deformed nodal tributary area $A_i$.
    NodeToSurface,
    /// 3-point Hammer Gauss-point-to-segment (Mortar GPTS) integration over exterior faces.
    /// Eliminates quadratic `TET10` corner-node pressure oscillation while preserving exact
    /// area integration and positive definite stiffness on all face nodes.
    MortarGauss,
}

/// Analytical rigid surface for deformable-to-rigid contact benchmarks and platens.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum AnalyticalRigidSurface {
    /// Infinite half-space bounded by the plane `(x - origin) · normal >= 0` (outward normal).
    Plane {
        /// Point on the rigid plane in meters.
        origin: [f64; 3],
        /// Outward unit normal pointing into the allowable (free) half-space.
        normal: [f64; 3],
    },
    /// Infinite rigid cylinder of radius `radius` centered along `(origin, axis)`.
    Cylinder {
        /// Point on cylinder axis in meters.
        origin: [f64; 3],
        /// Cylinder axis direction.
        axis: [f64; 3],
        /// Cylinder radius in meters.
        radius: f64,
    },
}

/// Specification of a single contact interface in a multi-body deformable contact simulation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ContactPairSpec {
    /// Deformable-to-deformable contact between `body_a` (slave) and `body_b` (master).
    DeformablePair {
        /// Index of the slave body in `bodies`.
        body_a: usize,
        /// Index of the master body in `bodies`.
        body_b: usize,
        /// Dry Coulomb friction coefficient $\mu \ge 0$.
        friction_coeff: f64,
        /// If `true`, activates symmetric two-pass contact (`A -> B` and `B -> A` with half
        /// tributary weights on each pass) so sharp corners/tooth tips on either body cannot
        /// penetrate the mating surface.
        #[serde(default)]
        symmetric: bool,
        /// Optional subset of exterior node indices on `body_a` eligible for slave contact.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        slave_a_nodes: Option<Vec<usize>>,
        /// Optional subset of exterior node indices on `body_b` eligible for slave contact
        /// (used when `symmetric == true`).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        slave_b_nodes: Option<Vec<usize>>,
        /// Optional search distance cutoff in meters (defaults to automatic scale from mesh size).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        search_distance: Option<f64>,
        /// Maximum allowed dot product `n_slave · n_master` for opposing surface candidates
        /// (default `0.35`, rejecting back-facing flanks of thin features such as gear teeth).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        max_normal_dot: Option<f64>,
    },
    /// Deformable-to-rigid contact between `body_idx` and an analytical rigid surface.
    DeformableToRigid {
        /// Index of the deformable body in `bodies`.
        body_idx: usize,
        /// Rigid counter-surface.
        surface: AnalyticalRigidSurface,
        /// Dry Coulomb friction coefficient $\mu \ge 0$.
        friction_coeff: f64,
        /// Optional subset of exterior node indices eligible for slave contact.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        slave_nodes: Option<Vec<usize>>,
        /// Optional search distance cutoff in meters.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        search_distance: Option<f64>,
    },
}

/// A deformable 3D body participating in a multi-body contact solve.
#[derive(Debug, Clone)]
pub struct ContactBody {
    /// Logical body / part identifier.
    pub id: String,
    /// Volumetric tetrahedral mesh (`TET4` or `TET10`).
    pub mesh: TetrahedralMesh,
    /// Elastoplastic $J_2$ constitutive material parameters.
    pub material: J2MaterialParams,
    /// Base Dirichlet and Neumann boundary conditions (may be overridden per step).
    pub boundary: BoundaryConditions,
    /// Base kinematic hub couplings (e.g. gear bore revolute joint or platen prismatic joint).
    pub hub_couplings: Vec<HubCoupling>,
}

impl ContactBody {
    /// Creates a new deformable contact body with empty base boundary conditions and couplings.
    pub fn new(id: impl Into<String>, mesh: TetrahedralMesh, material: J2MaterialParams) -> Self {
        Self {
            id: id.into(),
            mesh,
            material,
            boundary: BoundaryConditions::new(),
            hub_couplings: Vec::new(),
        }
    }

    /// Attaches base boundary conditions.
    pub fn with_boundary(mut self, boundary: BoundaryConditions) -> Self {
        self.boundary = boundary;
        self
    }

    /// Appends a base kinematic hub coupling.
    pub fn with_hub_coupling(mut self, coupling: HubCoupling) -> Self {
        self.hub_couplings.push(coupling);
        self
    }
}

/// Per-step boundary/actuator override for one or more bodies in a multi-step contact history.
#[derive(Debug, Clone, Default)]
pub struct ContactStepSpec {
    /// Optional descriptive label for this step (e.g. `"engage_step_1"`).
    pub label: Option<String>,
    /// Optional per-body replacement boundary conditions (`body_idx -> BoundaryConditions`).
    pub body_boundaries: HashMap<usize, BoundaryConditions>,
    /// Optional per-body replacement hub couplings (`body_idx -> Vec<HubCoupling>`).
    pub body_hub_couplings: HashMap<usize, Vec<HubCoupling>>,
}

impl ContactStepSpec {
    /// Creates a new step specification with the given label.
    pub fn new(label: impl Into<String>) -> Self {
        Self {
            label: Some(label.into()),
            body_boundaries: HashMap::new(),
            body_hub_couplings: HashMap::new(),
        }
    }

    /// Overrides boundary conditions for `body_idx` on this step.
    pub fn with_body_boundary(mut self, body_idx: usize, bc: BoundaryConditions) -> Self {
        self.body_boundaries.insert(body_idx, bc);
        self
    }

    /// Overrides kinematic hub couplings for `body_idx` on this step.
    pub fn with_body_hub_couplings(mut self, body_idx: usize, couplings: Vec<HubCoupling>) -> Self {
        self.body_hub_couplings.insert(body_idx, couplings);
        self
    }
}

fn default_max_bisections() -> usize {
    8
}

fn default_min_substep_fraction() -> f64 {
    1e-3
}

fn default_one_usize() -> usize {
    1
}

/// Configuration options for [`solve_deformable_contact`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ContactSolverOptions {
    /// Contact surface discretization (`Auto`, `NodeToSurface`, `MortarGauss`).
    pub discretization: ContactDiscretization,
    /// Enable Co-Rotated FEM finite-rotation kinematics ($\mathbf{F} = \mathbf{R}\mathbf{U}$).
    pub corotated: bool,
    /// Include element geometric stiffness $\mathbf{K}_{\text{geo}}$ in tangent assembly.
    pub include_geometric_stiffness: bool,
    /// Maximum allowable nodal interpenetration tolerance $g_{\text{tol}}$ in meters (default: `1e-6` m).
    pub max_penetration_tol: f64,
    /// Relative Augmented Lagrangian multiplier convergence tolerance (default: `2e-3`).
    pub al_multiplier_tol: f64,
    /// Maximum outer Augmented Lagrangian (Uzawa) iterations per step (default: `25`).
    pub max_al_iterations: usize,
    /// Relative force residual tolerance for inner Newton–Raphson equilibrium (default: `1e-5`).
    pub newton_rel_tol: f64,
    /// Absolute force residual tolerance in Newtons (default: `1e-5` N).
    pub newton_abs_tol: f64,
    /// Maximum inner Newton–Raphson iterations per AL iteration (default: `30`).
    pub max_newton_iterations: usize,
    /// Dimensionless initial normal penalty scale $\alpha_n$ in $\epsilon_n = \alpha_n E^* / h$ (default: `4.0`).
    pub penalty_scale: f64,
    /// Tangential-to-normal penalty ratio $\epsilon_T / \epsilon_n$ (default: `0.5`).
    pub tangential_penalty_ratio: f64,
    /// Maximum adaptive penalty multiplier relative to initial $\epsilon_n$ (default: `80.0`).
    pub max_penalty_escalation: f64,
    /// Maximum adaptive step-bisection cutbacks per step when Newton–Raphson or Augmented
    /// Lagrangian equilibrium fails to converge (default: `8`). Set to `0` to disable bisection.
    #[serde(default = "default_max_bisections")]
    pub max_bisections: usize,
    /// Minimum substep fraction $\Delta\lambda_{\min}$ allowed before aborting bisection (default: `1e-3`).
    #[serde(default = "default_min_substep_fraction")]
    pub min_substep_fraction: f64,
    /// Linear PCG solver options.
    pub linear_solver: SolverOptions,
}

impl Default for ContactSolverOptions {
    fn default() -> Self {
        Self {
            discretization: ContactDiscretization::Auto,
            corotated: true,
            include_geometric_stiffness: true,
            max_penetration_tol: 1e-6,
            al_multiplier_tol: 2e-3,
            max_al_iterations: 25,
            newton_rel_tol: 1e-5,
            newton_abs_tol: 1e-5,
            max_newton_iterations: 30,
            penalty_scale: 4.0,
            tangential_penalty_ratio: 0.5,
            max_penalty_escalation: 80.0,
            max_bisections: default_max_bisections(),
            min_substep_fraction: default_min_substep_fraction(),
            linear_solver: SolverOptions {
                tolerance: 1e-8,
                max_iterations: 4000,
            },
        }
    }
}

/// Converged contact traction and geometry state at an active contact node or Mortar Gauss point.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ContactPointResult {
    /// Slave body index.
    pub slave_body: usize,
    /// Representative slave node ID (primary node for NTS, first corner/mid node for Mortar).
    pub slave_node_id: usize,
    /// Current deformed 3D position of the contact point in meters.
    pub position: [f64; 3],
    /// Unit outward contact normal on the master surface (pointing from master toward slave).
    pub normal: [f64; 3],
    /// Signed normal gap $g_n$ in meters ($g_n < 0$ indicates penetration).
    pub signed_gap_m: f64,
    /// Tributary contact area weight $A_c$ in m².
    pub area_m2: f64,
    /// Normal contact pressure $p_n \ge 0$ in Pascals (Pa).
    pub normal_pressure_pa: f64,
    /// Tangential friction traction vector $\mathbf{t}_T$ in Pascals (Pa).
    pub tangential_traction_pa: [f64; 3],
    /// Tangential friction stress magnitude $\|\mathbf{t}_T\|$ in Pascals (Pa).
    pub tangential_stress_pa: f64,
    /// True if Coulomb slip is active ($\|\mathbf{t}_T\| \approx \mu p_n$ with $\mu > 0$).
    pub is_slipping: bool,
}

/// Aggregate contact mechanics metrics for a single [`ContactPairSpec`] on a step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactPatchSummary {
    /// Index of the pair in the input `pairs` slice.
    pub pair_index: usize,
    /// Number of active contact points with $p_n > 0$.
    pub active_point_count: usize,
    /// Total active contact patch area in m².
    pub contact_area_m2: f64,
    /// Maximum normal contact pressure $p_{\max}$ across active points in Pa.
    pub peak_normal_pressure_pa: f64,
    /// Maximum tangential friction traction magnitude in Pa.
    pub peak_tangential_stress_pa: f64,
    /// Total integrated normal contact force scalar $\sum A_c p_n$ in Newtons.
    pub total_normal_force_n: f64,
    /// Resultant normal contact force vector acting on the primary slave body (`body_a`) in Newtons.
    pub resultant_normal_force_vec_n: [f64; 3],
    /// Resultant tangential friction force vector acting on the primary slave body in Newtons.
    pub resultant_tangential_force_vec_n: [f64; 3],
    /// Total integrated tangential friction force magnitude in Newtons.
    pub total_tangential_force_n: f64,
    /// Maximum interpenetration measured across this pair (in both directions for deformable pairs) in meters.
    pub max_penetration_m: f64,
    /// Count of active points currently in Coulomb slip.
    pub slipping_point_count: usize,
    /// Detailed per-point results for all active contact points on this pair.
    pub active_points: Vec<ContactPointResult>,
}

/// Solved state and reaction effort of a single [`HubCoupling`] on a step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HubCouplingResult {
    /// Body index owning this hub coupling.
    pub body_index: usize,
    /// Hub coupling index on that body.
    pub hub_index: usize,
    /// Optional human-readable coupling name.
    pub name: Option<String>,
    /// Final generalized coordinate (angle in radians for `Revolute`, displacement in meters for `Prismatic`, `0.0` for `Fixed`).
    pub coordinate: f64,
    /// Axial reaction torque (N·m for `Revolute`) or axial reaction force (N for `Prismatic`) exerted by the joint on the body.
    pub reaction_effort: f64,
    /// Net 3D reaction force vector exerted by the coupled hub nodes in Newtons.
    pub reaction_force_n: [f64; 3],
    /// Net 3D reaction torque vector about the hub pivot (or origin) in N·m.
    pub reaction_torque_nm: [f64; 3],
}

/// Complete telemetry and field summary for one converged step of [`solve_deformable_contact`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactStepReport {
    /// Zero-based step index.
    pub step_index: usize,
    /// Optional descriptive label.
    pub label: Option<String>,
    /// Whether both equilibrium and penetration tolerances converged.
    pub converged: bool,
    /// Number of outer Augmented Lagrangian iterations executed.
    pub al_iterations: usize,
    /// Total inner Newton–Raphson iterations executed across all AL iterations.
    pub total_newton_iterations: usize,
    /// Number of dynamic deformed-surface + BVH rebuilds executed during this step
    /// (always equals or exceeds `total_newton_iterations`).
    pub deformed_surface_updates: usize,
    /// Final reduced equilibrium residual norm in Newtons (or N·m for rotational pilot DOFs).
    pub final_residual_norm: f64,
    /// Worst interpenetration across ALL pairs (measured on both sides for deformable pairs) in meters.
    pub max_penetration_m: f64,
    /// Per-pair contact patch summaries.
    pub pairs: Vec<ContactPatchSummary>,
    /// Per-hub kinematic coordinates and reaction efforts.
    pub hubs: Vec<HubCouplingResult>,
    /// Number of adaptive step-bisection cutbacks performed during this step.
    #[serde(default)]
    pub bisections: usize,
    /// Number of converged substeps completed to reach the end of this step (`1` if no bisection occurred).
    #[serde(default = "default_one_usize")]
    pub substeps_completed: usize,
    /// Peak element/Gauss-point von Mises stress per body in Pascals (Pa).
    pub body_max_von_mises_pa: Vec<f64>,
    /// Peak equivalent plastic strain $\bar{\varepsilon}^p$ per body (dimensionless).
    pub body_max_equiv_plastic_strain: Vec<f64>,
    /// Peak nodal displacement magnitude per body in meters.
    pub body_max_displacement_m: Vec<f64>,
}

/// Complete report returned by [`solve_deformable_contact`].
#[derive(Debug, Clone)]
pub struct DeformableContactReport {
    /// Step-by-step history reports.
    pub steps: Vec<ContactStepReport>,
    /// Final converged nonlinear elastoplastic state (`u_full`, `gp_states`, `element_rotations`) for each body.
    pub body_states: Vec<NonlinearState>,
    /// Final deformed exterior surface positions for each body (indexed by node ID).
    pub body_deformed_positions: Vec<Vec<[f64; 3]>>,
    /// Total wall-clock elapsed time in milliseconds.
    pub elapsed_ms: f64,
}

impl DeformableContactReport {
    /// Returns the final step report, if at least one step was executed.
    pub fn final_step(&self) -> Option<&ContactStepReport> {
        self.steps.last()
    }

    /// Worst interpenetration observed across ALL steps in meters.
    pub fn worst_penetration_all_steps_m(&self) -> f64 {
        self.steps
            .iter()
            .map(|s| s.max_penetration_m)
            .fold(0.0_f64, f64::max)
    }
}

// ============================================================================
// Dynamic Deformed Exterior Surface Mesh & Compact Flat AABB-BVH
// ============================================================================

#[derive(Debug, Clone, Copy)]
enum ClosestFeature {
    Vertex(usize),
    Edge(usize, usize),
    Interior,
}

/// Result of projecting a 3D point onto a deformed master surface mesh.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceContactProjection {
    /// Triangle index in `DeformedSurfaceMesh::triangles`.
    pub tri_index: usize,
    /// Master mesh node indices `[n0, n1, n2]` of the projected triangle.
    pub master_nodes: [usize; 3],
    /// Barycentric coordinates `[xi0, xi1, xi2]` summing to `1.0`.
    pub barycentric: [f64; 3],
    /// Closest point $\mathbf{q}$ on the deformed master surface.
    pub closest_point: [f64; 3],
    /// Smooth outward unit normal $\mathbf{n}_c$ on the master surface.
    pub normal: [f64; 3],
    /// Exact signed normal gap $g_n$ in meters ($g_n < 0$ indicates penetration).
    pub signed_gap: f64,
}

#[derive(Debug, Clone, Copy)]
struct BvhNode {
    min: [f64; 3],
    max: [f64; 3],
    /// For internal nodes: left child index; for leaf nodes: start index in `tri_order`.
    left_or_start: usize,
    /// If `0`, internal node (right child is `left_or_start + 1` or stored in `right_child`);
    /// if `> 0`, leaf triangle count.
    count: usize,
    right_child: usize,
}

#[derive(Debug, Clone)]
struct SurfaceBvh {
    nodes: Vec<BvhNode>,
    tri_order: Vec<usize>,
}

impl SurfaceBvh {
    fn empty() -> Self {
        Self {
            nodes: Vec::new(),
            tri_order: Vec::new(),
        }
    }

    fn build(positions: &[[f64; 3]], triangles: &[[usize; 3]]) -> Self {
        let n = triangles.len();
        if n == 0 {
            return Self::empty();
        }
        let mut tri_bounds = Vec::with_capacity(n);
        let mut tri_centroids = Vec::with_capacity(n);
        for &[i0, i1, i2] in triangles {
            let p0 = positions[i0];
            let p1 = positions[i1];
            let p2 = positions[i2];
            let min = [
                p0[0].min(p1[0]).min(p2[0]),
                p0[1].min(p1[1]).min(p2[1]),
                p0[2].min(p1[2]).min(p2[2]),
            ];
            let max = [
                p0[0].max(p1[0]).max(p2[0]),
                p0[1].max(p1[1]).max(p2[1]),
                p0[2].max(p1[2]).max(p2[2]),
            ];
            let c = [
                (p0[0] + p1[0] + p2[0]) / 3.0,
                (p0[1] + p1[1] + p2[1]) / 3.0,
                (p0[2] + p1[2] + p2[2]) / 3.0,
            ];
            tri_bounds.push((min, max));
            tri_centroids.push(c);
        }

        let mut tri_order: Vec<usize> = (0..n).collect();
        let mut nodes = Vec::with_capacity(n * 2);
        Self::build_recursive(
            &tri_bounds,
            &tri_centroids,
            &mut tri_order,
            &mut nodes,
            0,
            n,
        );
        Self { nodes, tri_order }
    }

    fn build_recursive(
        tri_bounds: &[([f64; 3], [f64; 3])],
        tri_centroids: &[[f64; 3]],
        tri_order: &mut [usize],
        nodes: &mut Vec<BvhNode>,
        start: usize,
        end: usize,
    ) -> usize {
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        let mut cmin = [f64::INFINITY; 3];
        let mut cmax = [f64::NEG_INFINITY; 3];

        for &ti in &tri_order[start..end] {
            let (tmin, tmax) = tri_bounds[ti];
            let c = tri_centroids[ti];
            for k in 0..3 {
                min[k] = min[k].min(tmin[k]);
                max[k] = max[k].max(tmax[k]);
                cmin[k] = cmin[k].min(c[k]);
                cmax[k] = cmax[k].max(c[k]);
            }
        }

        let count = end - start;
        let node_idx = nodes.len();
        if count <= 6 {
            nodes.push(BvhNode {
                min,
                max,
                left_or_start: start,
                count,
                right_child: 0,
            });
            return node_idx;
        }

        let ext = [cmax[0] - cmin[0], cmax[1] - cmin[1], cmax[2] - cmin[2]];
        let axis = if ext[0] >= ext[1] && ext[0] >= ext[2] {
            0
        } else if ext[1] >= ext[2] {
            1
        } else {
            2
        };

        let mid = start + count / 2;
        if ext[axis] > 1e-15 {
            tri_order[start..end].select_nth_unstable_by(count / 2, |&a, &b| {
                tri_centroids[a][axis]
                    .partial_cmp(&tri_centroids[b][axis])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }

        nodes.push(BvhNode {
            min,
            max,
            left_or_start: 0,
            count: 0,
            right_child: 0,
        });

        let left = Self::build_recursive(tri_bounds, tri_centroids, tri_order, nodes, start, mid);
        let right = Self::build_recursive(tri_bounds, tri_centroids, tri_order, nodes, mid, end);
        nodes[node_idx].left_or_start = left;
        nodes[node_idx].right_child = right;
        node_idx
    }

    #[inline]
    fn aabb_dist_sq(p: [f64; 3], min: [f64; 3], max: [f64; 3]) -> f64 {
        let mut d2 = 0.0;
        for k in 0..3 {
            let v = if p[k] < min[k] {
                min[k] - p[k]
            } else if p[k] > max[k] {
                p[k] - max[k]
            } else {
                0.0
            };
            d2 += v * v;
        }
        d2
    }
}

/// Outward-oriented deformed exterior surface skin of a tetrahedral body, supporting
/// dynamic per-Newton-iteration vertex & BVH updates and exact Bærentzen–Aanæs signed gaps.
#[derive(Debug, Clone)]
pub struct DeformedSurfaceMesh {
    /// Outward-oriented exterior triangles `[n0, n1, n2]` (for `TET10`, 4 conforming sub-tris per $T_6$ face).
    pub triangles: Vec<[usize; 3]>,
    /// Outward-oriented 6-node exterior faces `([c0, c1, c2], [m01, m12, m20])` when `TET10`.
    pub quad_faces: Vec<([usize; 3], [usize; 3])>,
    /// Sorted unique exterior surface node IDs.
    pub surface_nodes: Vec<usize>,
    /// Current deformed 3D coordinates of every node in the body.
    pub current_positions: Vec<[f64; 3]>,
    /// Current outward unit normals per exterior triangle.
    pub tri_normals: Vec<[f64; 3]>,
    /// Current area per exterior triangle in m².
    pub tri_areas: Vec<f64>,
    /// Angle-weighted outward vertex pseudonormals indexed by node ID.
    pub vertex_normals: Vec<[f64; 3]>,
    /// Edge pseudonormals keyed by canonical undirected edge `(min(a,b), max(a,b))`.
    edge_normals: HashMap<(usize, usize), [f64; 3]>,
    /// Deformed nodal tributary area $A_i = \sum_{T \ni i} A_T / 3$ indexed by node ID.
    pub nodal_areas: Vec<f64>,
    /// Exterior triangle indices incident to each node (for directional contact tributary area).
    node_incident_tris: Vec<Vec<usize>>,
    /// Characteristic surface edge length in reference configuration.
    pub char_edge_len: f64,
    bvh: SurfaceBvh,
}

impl DeformedSurfaceMesh {
    /// Extracts all strictly outward-oriented exterior faces from `mesh` and builds the initial BVH.
    pub fn from_tetra_mesh(mesh: &TetrahedralMesh) -> Self {
        Self::from_tetra_mesh_filtered(mesh, None)
    }

    /// Extracts strictly outward-oriented exterior faces from the active elements of `mesh`
    /// (`active_elements[e] == true`, or all elements when `None`) and builds the BVH.
    ///
    /// When elements are eroded or detached in `P5-T5`, newly exposed interior faces automatically
    /// become exterior contact surfaces with outward-pointing normals.
    pub fn from_tetra_mesh_filtered(
        mesh: &TetrahedralMesh,
        active_elements: Option<&[bool]>,
    ) -> Self {
        let num_nodes = mesh.node_count();
        let initial_positions: Vec<[f64; 3]> = mesh.nodes.iter().map(|n| n.position).collect();

        let (triangles, quad_faces) = if mesh.is_tet10() {
            // Track canonical sorted corner triple -> (count, corners, mids, opp_corner)
            type Tet10FaceEntry = (usize, [usize; 3], [usize; 3], usize);
            let mut face_map: BTreeMap<[usize; 3], Tet10FaceEntry> = BTreeMap::new();
            for (e_idx, ten) in mesh.tet10_elements.iter().enumerate() {
                if active_elements.is_some_and(|m| !m.get(e_idx).copied().unwrap_or(false)) {
                    continue;
                }
                let [n0, n1, n2, n3, m01, m12, m20, m03, m13, m23] = *ten;
                let faces = [
                    ([n0, n1, n2], [m01, m12, m20], n3),
                    ([n0, n2, n3], [m20, m23, m03], n1),
                    ([n0, n3, n1], [m03, m13, m01], n2),
                    ([n1, n3, n2], [m13, m23, m12], n0),
                ];
                for (corners, mids, opp) in faces {
                    let mut key = corners;
                    key.sort_unstable();
                    let entry = face_map.entry(key).or_insert((0, corners, mids, opp));
                    entry.0 += 1;
                }
            }

            let mut qfaces = Vec::new();
            let mut tris = Vec::new();
            for (count, mut corners, mut mids, opp) in face_map.into_values() {
                if count != 1 {
                    continue;
                }
                let p0 = Vector3::from(initial_positions[corners[0]]);
                let p1 = Vector3::from(initial_positions[corners[1]]);
                let p2 = Vector3::from(initial_positions[corners[2]]);
                let popp = Vector3::from(initial_positions[opp]);
                if (p1 - p0).cross(&(p2 - p0)).dot(&(popp - p0)) > 0.0 {
                    corners.swap(1, 2);
                    // Edge (0,1) <-> Edge (2,0); Edge (1,2) reverses in place
                    mids.swap(0, 2);
                }
                let [c0, c1, c2] = corners;
                let [m01, m12, m20] = mids;
                qfaces.push((corners, mids));
                tris.push([c0, m01, m20]);
                tris.push([m01, c1, m12]);
                tris.push([m20, m12, c2]);
                tris.push([m01, m12, m20]);
            }
            (tris, qfaces)
        } else {
            let mut face_map: BTreeMap<[usize; 3], (usize, [usize; 3], usize)> = BTreeMap::new();
            for (e_idx, elem) in mesh.elements.iter().enumerate() {
                if active_elements.is_some_and(|m| !m.get(e_idx).copied().unwrap_or(false)) {
                    continue;
                }
                let [n0, n1, n2, n3] = elem.node_indices;
                let faces = [
                    ([n0, n1, n2], n3),
                    ([n0, n2, n3], n1),
                    ([n0, n3, n1], n2),
                    ([n1, n3, n2], n0),
                ];
                for (corners, opp) in faces {
                    let mut key = corners;
                    key.sort_unstable();
                    let entry = face_map.entry(key).or_insert((0, corners, opp));
                    entry.0 += 1;
                }
            }

            let mut tris = Vec::new();
            for (count, mut corners, opp) in face_map.into_values() {
                if count != 1 {
                    continue;
                }
                let p0 = Vector3::from(initial_positions[corners[0]]);
                let p1 = Vector3::from(initial_positions[corners[1]]);
                let p2 = Vector3::from(initial_positions[corners[2]]);
                let popp = Vector3::from(initial_positions[opp]);
                if (p1 - p0).cross(&(p2 - p0)).dot(&(popp - p0)) > 0.0 {
                    corners.swap(1, 2);
                }
                tris.push(corners);
            }
            (tris, Vec::new())
        };

        let mut node_set = HashSet::new();
        let mut edge_len_sum = 0.0;
        let mut edge_count = 0usize;
        for &[a, b, c] in &triangles {
            node_set.insert(a);
            node_set.insert(b);
            node_set.insert(c);
            let pa = Vector3::from(initial_positions[a]);
            let pb = Vector3::from(initial_positions[b]);
            let pc = Vector3::from(initial_positions[c]);
            edge_len_sum += (pb - pa).norm() + (pc - pb).norm() + (pa - pc).norm();
            edge_count += 3;
        }
        let mut surface_nodes: Vec<usize> = node_set.into_iter().collect();
        surface_nodes.sort_unstable();

        let char_edge_len = if edge_count > 0 {
            (edge_len_sum / edge_count as f64).max(1e-6)
        } else {
            1e-3
        };

        let tri_count = triangles.len();
        let mut node_incident_tris = vec![Vec::new(); num_nodes];
        for (t_idx, &[a, b, c]) in triangles.iter().enumerate() {
            node_incident_tris[a].push(t_idx);
            node_incident_tris[b].push(t_idx);
            node_incident_tris[c].push(t_idx);
        }
        let mut surf = Self {
            triangles,
            quad_faces,
            surface_nodes,
            current_positions: initial_positions,
            tri_normals: vec![[0.0, 0.0, 1.0]; tri_count],
            tri_areas: vec![0.0; tri_count],
            vertex_normals: vec![[0.0, 0.0, 1.0]; num_nodes],
            edge_normals: HashMap::with_capacity(tri_count * 2),
            nodal_areas: vec![0.0; num_nodes],
            node_incident_tris,
            char_edge_len,
            bvh: SurfaceBvh::empty(),
        };
        surf.rebuild_geometry();
        surf
    }

    /// Computes the deformed tributary contact area at `nid` considering ONLY incident exterior
    /// triangles whose outward normal opposes `master_outward_normal` (`n_tri · n_master < -0.25`).
    ///
    /// This excludes perpendicular side walls (such as plane-strain Z-faces or gear flat side
    /// faces) from inflating contact area weights and biasing contact pressure recovery.
    #[inline]
    pub fn contact_nodal_area(&self, nid: usize, master_outward_normal: Vector3<f64>) -> f64 {
        let mut area = 0.0;
        for &t_idx in &self.node_incident_tris[nid] {
            let tn = Vector3::from(self.tri_normals[t_idx]);
            if tn.dot(&master_outward_normal) < -0.25 {
                area += self.tri_areas[t_idx] / 3.0;
            }
        }
        if area > 1e-22 {
            area
        } else {
            self.nodal_areas[nid]
        }
    }

    /// Computes an area-weighted smooth vertex normal at `nid` considering ONLY incident
    /// triangles within the smooth crease threshold of `ref_normal` (`n_tri · ref_normal > 0.80`,
    /// ~36.9°). This prevents sharp 90° edges (plane-strain Z-walls, box corners, gear flat side
    /// faces, tooth tips) from contaminating contact traction normals while preserving C1 smooth
    /// normals across curved cylinder arcs and involute gear tooth flanks.
    #[inline]
    pub fn crease_filtered_vertex_normal(
        &self,
        nid: usize,
        ref_normal: Vector3<f64>,
    ) -> Vector3<f64> {
        let mut accum = Vector3::<f64>::zeros();
        for &t_idx in &self.node_incident_tris[nid] {
            let tn = Vector3::from(self.tri_normals[t_idx]);
            if tn.dot(&ref_normal) > 0.80 {
                accum += tn * self.tri_areas[t_idx];
            }
        }
        let len = accum.norm();
        if len > 1e-22 {
            accum / len
        } else {
            ref_normal
        }
    }

    /// Updates deformed vertex positions from `u_body`, recomputes outward triangle normals,
    /// Bærentzen–Aanæs angle-weighted vertex/edge pseudonormals, nodal tributary areas, and
    /// rebuilds the spatial BVH. Called on **every Newton–Raphson iteration**.
    pub fn update_from_displacements(&mut self, mesh: &TetrahedralMesh, u_body: &[f64]) {
        for (nid, node) in mesh.nodes.iter().enumerate() {
            let d = 3 * nid;
            self.current_positions[nid] = [
                node.position[0] + u_body[d],
                node.position[1] + u_body[d + 1],
                node.position[2] + u_body[d + 2],
            ];
        }
        self.rebuild_geometry();
    }

    fn rebuild_geometry(&mut self) {
        let num_nodes = self.current_positions.len();
        let mut v_accum = vec![Vector3::<f64>::zeros(); num_nodes];
        let mut e_accum: HashMap<(usize, usize), Vector3<f64>> =
            HashMap::with_capacity(self.triangles.len() * 2);
        self.nodal_areas.fill(0.0);

        for (t_idx, &[i0, i1, i2]) in self.triangles.iter().enumerate() {
            let p0 = Vector3::from(self.current_positions[i0]);
            let p1 = Vector3::from(self.current_positions[i1]);
            let p2 = Vector3::from(self.current_positions[i2]);
            let e01 = p1 - p0;
            let e02 = p2 - p0;
            let cross = e01.cross(&e02);
            let norm = cross.norm();
            if norm > 1e-24 {
                let n = cross / norm;
                let area = 0.5 * norm;
                self.tri_normals[t_idx] = [n.x, n.y, n.z];
                self.tri_areas[t_idx] = area;

                let share = area / 3.0;
                self.nodal_areas[i0] += share;
                self.nodal_areas[i1] += share;
                self.nodal_areas[i2] += share;

                // Bærentzen–Aanæs angle weights at each vertex of the triangle
                let u01 = e01.normalize();
                let u02 = e02.normalize();
                let u12 = (p2 - p1).normalize();
                let a0 = u01.dot(&u02).clamp(-1.0, 1.0).acos();
                let a1 = (-u01).dot(&u12).clamp(-1.0, 1.0).acos();
                let a2 = (-u02).dot(&(-u12)).clamp(-1.0, 1.0).acos();

                v_accum[i0] += n * a0;
                v_accum[i1] += n * a1;
                v_accum[i2] += n * a2;

                for (ea, eb) in [(i0, i1), (i1, i2), (i2, i0)] {
                    let key = if ea < eb { (ea, eb) } else { (eb, ea) };
                    *e_accum.entry(key).or_insert_with(Vector3::zeros) += n;
                }
            } else {
                self.tri_normals[t_idx] = [0.0, 0.0, 1.0];
                self.tri_areas[t_idx] = 0.0;
            }
        }

        for (nid, &v) in v_accum.iter().enumerate() {
            let len = v.norm();
            if len > 1e-18 {
                let vn = v / len;
                self.vertex_normals[nid] = [vn.x, vn.y, vn.z];
            }
        }

        self.edge_normals.clear();
        for (k, v) in e_accum {
            let len = v.norm();
            if len > 1e-18 {
                let en = v / len;
                self.edge_normals.insert(k, [en.x, en.y, en.z]);
            }
        }

        self.bvh = SurfaceBvh::build(&self.current_positions, &self.triangles);
    }

    /// Computes exact closest-point projection and Bærentzen–Aanæs signed gap of point `p`
    /// onto this deformed surface within `max_search_dist`.
    ///
    /// If `slave_outward_normal` is provided, triangles with `n_slave · n_master > max_normal_dot`
    /// are ignored so nodes cannot snap onto back-facing flanks of thin features.
    pub fn closest_point_signed(
        &self,
        p: [f64; 3],
        slave_outward_normal: Option<[f64; 3]>,
        max_search_dist: f64,
        max_normal_dot: f64,
    ) -> Option<SurfaceContactProjection> {
        if self.bvh.nodes.is_empty() {
            return None;
        }
        let vp = Vector3::from(p);
        let slave_n = slave_outward_normal.map(Vector3::from);

        let mut best_dist_sq = max_search_dist * max_search_dist;
        let mut best_align_dot = f64::INFINITY;
        let mut best_hit: Option<(usize, [f64; 3], [f64; 3], ClosestFeature)> = None;

        let mut stack = [0usize; 64];
        let mut sp = 0usize;
        if SurfaceBvh::aabb_dist_sq(p, self.bvh.nodes[0].min, self.bvh.nodes[0].max) <= best_dist_sq
        {
            stack[sp] = 0;
            sp += 1;
        }

        while sp > 0 {
            sp -= 1;
            let node = self.bvh.nodes[stack[sp]];
            if SurfaceBvh::aabb_dist_sq(p, node.min, node.max) > best_dist_sq + 1e-22 {
                continue;
            }
            if node.count > 0 {
                let start = node.left_or_start;
                for &t_idx in &self.bvh.tri_order[start..start + node.count] {
                    if self.tri_areas[t_idx] <= 1e-22 {
                        continue;
                    }
                    let align_dot = if let Some(sn) = slave_n {
                        let mn = Vector3::from(self.tri_normals[t_idx]);
                        let d = sn.dot(&mn);
                        if d > max_normal_dot {
                            continue;
                        }
                        d
                    } else {
                        0.0
                    };
                    let [i0, i1, i2] = self.triangles[t_idx];
                    let (q, bary, feature) = closest_point_on_triangle_with_feature(
                        vp,
                        Vector3::from(self.current_positions[i0]),
                        Vector3::from(self.current_positions[i1]),
                        Vector3::from(self.current_positions[i2]),
                    );
                    let d2 = (vp - q).norm_squared();
                    let tie_tol = 1e-22 * (1.0 + best_dist_sq);
                    let is_better = d2 + tie_tol < best_dist_sq
                        || ((d2 - best_dist_sq).abs() <= tie_tol && align_dot < best_align_dot);
                    if is_better {
                        best_dist_sq = d2.min(best_dist_sq);
                        best_align_dot = align_dot;
                        best_hit = Some((t_idx, [q.x, q.y, q.z], bary, feature));
                    }
                }
            } else {
                let left = node.left_or_start;
                let right = node.right_child;
                let dl = SurfaceBvh::aabb_dist_sq(p, self.bvh.nodes[left].min, self.bvh.nodes[left].max);
                let dr = SurfaceBvh::aabb_dist_sq(p, self.bvh.nodes[right].min, self.bvh.nodes[right].max);
                // Push farther child first so nearer child is popped first
                if dl < dr {
                    if dr <= best_dist_sq && sp < stack.len() {
                        stack[sp] = right;
                        sp += 1;
                    }
                    if dl <= best_dist_sq && sp < stack.len() {
                        stack[sp] = left;
                        sp += 1;
                    }
                } else {
                    if dl <= best_dist_sq && sp < stack.len() {
                        stack[sp] = left;
                        sp += 1;
                    }
                    if dr <= best_dist_sq && sp < stack.len() {
                        stack[sp] = right;
                        sp += 1;
                    }
                }
            }
        }

        let (t_idx, q_arr, bary, feature) = best_hit?;
        let master_nodes = self.triangles[t_idx];
        let n_face = Vector3::from(self.tri_normals[t_idx]);
        let n_pseudo = match feature {
            ClosestFeature::Interior => n_face,
            ClosestFeature::Vertex(local_v) => {
                Vector3::from(self.vertex_normals[master_nodes[local_v]])
            }
            ClosestFeature::Edge(la, lb) => {
                let ga = master_nodes[la];
                let gb = master_nodes[lb];
                let key = if ga < gb { (ga, gb) } else { (gb, ga) };
                Vector3::from(
                    self.edge_normals
                        .get(&key)
                        .copied()
                        .unwrap_or(self.tri_normals[t_idx]),
                )
            }
        };

        // Smooth C1 traction normal across co-smooth curved patches (cylinder arcs, involute
        // gear flanks) while strictly excluding sharp 90-degree walls (plane-strain Z-faces,
        // box corners, gear flat side faces, tooth tips).
        let nv0 = self.crease_filtered_vertex_normal(master_nodes[0], n_face);
        let nv1 = self.crease_filtered_vertex_normal(master_nodes[1], n_face);
        let nv2 = self.crease_filtered_vertex_normal(master_nodes[2], n_face);
        let n_interp_raw = nv0 * bary[0] + nv1 * bary[1] + nv2 * bary[2];
        let n_smooth = if n_interp_raw.norm() > 1e-12 {
            n_interp_raw.normalize()
        } else {
            n_face
        };

        let diff = vp - Vector3::from(q_arr);
        let dist = diff.norm();
        let sign_dot = diff.dot(&n_pseudo);
        let signed_gap = if dist <= 1e-15 {
            0.0
        } else if sign_dot < 0.0 {
            -dist
        } else {
            dist
        };

        Some(SurfaceContactProjection {
            tri_index: t_idx,
            master_nodes,
            barycentric: bary,
            closest_point: q_arr,
            normal: [n_smooth.x, n_smooth.y, n_smooth.z],
            signed_gap,
        })
    }
}

fn closest_point_on_triangle_with_feature(
    p: Vector3<f64>,
    a: Vector3<f64>,
    b: Vector3<f64>,
    c: Vector3<f64>,
) -> (Vector3<f64>, [f64; 3], ClosestFeature) {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, [1.0, 0.0, 0.0], ClosestFeature::Vertex(0));
    }

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, [0.0, 1.0, 0.0], ClosestFeature::Vertex(1));
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        let v = if denom.abs() > 1e-18 { d1 / denom } else { 0.0 };
        return (a + ab * v, [1.0 - v, v, 0.0], ClosestFeature::Edge(0, 1));
    }

    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, [0.0, 0.0, 1.0], ClosestFeature::Vertex(2));
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        let w = if denom.abs() > 1e-18 { d2 / denom } else { 0.0 };
        return (a + ac * w, [1.0 - w, 0.0, w], ClosestFeature::Edge(2, 0));
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let denom = (d4 - d3) + (d5 - d6);
        let w = if denom.abs() > 1e-18 {
            (d4 - d3) / denom
        } else {
            0.0
        };
        return (
            b + (c - b) * w,
            [0.0, 1.0 - w, w],
            ClosestFeature::Edge(1, 2),
        );
    }

    let denom = va + vb + vc;
    if denom.abs() > 1e-18 {
        let v = vb / denom;
        let w = vc / denom;
        let u = (1.0 - v - w).max(0.0);
        (a + ab * v + ac * w, [u, v, w], ClosestFeature::Interior)
    } else {
        (a, [1.0, 0.0, 0.0], ClosestFeature::Vertex(0))
    }
}

fn project_onto_analytical_rigid(
    p: [f64; 3],
    surface: AnalyticalRigidSurface,
) -> (f64, [f64; 3], [f64; 3]) {
    let vp = Vector3::from(p);
    match surface {
        AnalyticalRigidSurface::Plane { origin, normal } => {
            let n_raw = Vector3::from(normal);
            let n = if n_raw.norm() > 1e-12 {
                n_raw.normalize()
            } else {
                Vector3::y()
            };
            let o = Vector3::from(origin);
            let gap = (vp - o).dot(&n);
            let q = vp - n * gap;
            (gap, [n.x, n.y, n.z], [q.x, q.y, q.z])
        }
        AnalyticalRigidSurface::Cylinder {
            origin,
            axis,
            radius,
        } => {
            let a_raw = Vector3::from(axis);
            let a = if a_raw.norm() > 1e-12 {
                a_raw.normalize()
            } else {
                Vector3::z()
            };
            let o = Vector3::from(origin);
            let d = vp - o;
            let axial = d.dot(&a);
            let radial_vec = d - a * axial;
            let r = radial_vec.norm();
            let n = if r > 1e-15 {
                radial_vec / r
            } else {
                Vector3::x()
            };
            let gap = r - radius;
            let q = o + a * axial + n * radius;
            (gap, [n.x, n.y, n.z], [q.x, q.y, q.z])
        }
    }
}

// ============================================================================
// Internal Multi-Body Element Model, DOF Map, and Contact Stencil Structures
// ============================================================================

enum BodyElements {
    Tet4(Vec<Tet4Element>),
    Tet10(Vec<Tet10Element>),
}

impl BodyElements {
    fn from_mesh(mesh: &TetrahedralMesh) -> Result<Self, FeaError> {
        if mesh.is_empty() {
            return Err(FeaError::EmptyMesh);
        }
        if mesh.is_tet10() {
            let mut elems = Vec::with_capacity(mesh.tet10_elements.len());
            for (id, &node_indices) in mesh.tet10_elements.iter().enumerate() {
                let mut pos = [[0.0_f64; 3]; 10];
                for (k, &nid) in node_indices.iter().enumerate() {
                    pos[k] = mesh.nodes[nid].position;
                }
                elems.push(Tet10Element::new(id, node_indices, pos)?);
            }
            Ok(Self::Tet10(elems))
        } else {
            let mut elems = Vec::with_capacity(mesh.elements.len());
            for elem in &mesh.elements {
                let p0 = mesh.nodes[elem.node_indices[0]].position;
                let p1 = mesh.nodes[elem.node_indices[1]].position;
                let p2 = mesh.nodes[elem.node_indices[2]].position;
                let p3 = mesh.nodes[elem.node_indices[3]].position;
                elems.push(Tet4Element::new(elem.id, elem.node_indices, [p0, p1, p2, p3])?);
            }
            Ok(Self::Tet4(elems))
        }
    }

    fn gp_count(&self) -> usize {
        match self {
            Self::Tet4(v) => v.len(),
            Self::Tet10(v) => v.len() * 4,
        }
    }
}

#[inline]
fn rodrigues_rotate(v: Vector3<f64>, unit_axis: Vector3<f64>, angle: f64) -> Vector3<f64> {
    let c = angle.cos();
    let s = angle.sin();
    v * c + unit_axis.cross(&v) * s + unit_axis * (unit_axis.dot(&v) * (1.0 - c))
}

#[inline]
fn matrix3_to_rows(m: &Matrix3<f64>) -> [[f64; 3]; 3] {
    [
        [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
        [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
        [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
    ]
}

#[derive(Debug, Clone, Copy)]
enum DofMapping {
    /// Independent free translational nodal DOF at `reduced_idx`.
    Free(usize),
    /// Fixed/prescribed nodal DOF (not part of the reduced linear system).
    Fixed,
    /// Kinematically coupled to a scalar pilot DOF at `reduced_idx` with linear weight `weight`.
    Coupled { reduced_idx: usize, weight: f64 },
}

#[derive(Debug, Clone)]
struct PilotDofInfo {
    body_idx: usize,
    hub_idx: usize,
    reduced_idx: usize,
    kind: PilotKind,
    node_ids: Vec<usize>,
    /// Reference offsets `X_i - pivot` for `Revolute` hubs.
    ref_offsets: Vec<Vector3<f64>>,
    /// Unit axis vector.
    axis: Vector3<f64>,
    /// If `Prismatic` with `lock_transverse == false` on an axis-aligned axis, stores `Some(comp)`.
    free_transverse_axis: Option<usize>,
    /// External applied effort (torque in N·m or force in N).
    applied_effort: f64,
    /// Optional spring stiffness (N·m/rad or N/m).
    spring_stiffness: f64,
    /// Spring reference coordinate.
    spring_ref: f64,
    /// Current pilot generalized coordinate ($\theta$ or $s$).
    value: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PilotKind {
    Revolute,
    Prismatic,
}

#[derive(Debug, Clone)]
struct ContactSlaveStencil {
    /// Index of the owning `ContactPairSpec`.
    pair_index: usize,
    /// Index of slave body.
    slave_body: usize,
    /// Optional master body index (`None` for analytical rigid surface).
    master_body: Option<usize>,
    /// Analytical rigid surface when `master_body.is_none()`.
    rigid_surface: Option<AnalyticalRigidSurface>,
    /// Representative slave node ID for reporting.
    rep_node_id: usize,
    /// Slave nodes and shape-function weights `(node_id, N_a)` summing to `1.0`.
    /// For NTS: 1 entry `(node_id, 1.0)`. For Mortar `TET10`: 6 entries; for Mortar `TET4`: 3 entries.
    slave_nodes: Vec<(usize, f64)>,
    /// Optional exterior triangle/face index for computing deformed Mortar Gauss weight.
    mortar_face_corners: Option<[usize; 3]>,
    /// Area scale factor (`1.0` for single-pass, `0.5` for symmetric two-pass).
    area_factor: f64,
    /// Coulomb friction coefficient $\mu$.
    friction_coeff: f64,
    /// Maximum search distance in meters.
    search_distance: f64,
    /// Maximum allowed `n_slave · n_master` dot product.
    max_normal_dot: f64,
    /// Augmented Lagrangian normal multiplier $\lambda_n \ge 0$ (Pa).
    lambda_n: f64,
    /// Augmented Lagrangian tangential multiplier $\boldsymbol{\lambda}_T$ (Pa).
    lambda_t: Vector3<f64>,
    /// Current normal penalty stiffness $\epsilon_n$ (Pa/m).
    eps_n: f64,
    /// Current tangential penalty stiffness $\epsilon_T$ (Pa/m).
    eps_t: f64,
}

#[derive(Debug, Clone)]
struct EvaluatedContactPoint {
    stencil_idx: usize,
    active: bool,
    position: [f64; 3],
    normal: Vector3<f64>,
    signed_gap: f64,
    area: f64,
    t_n: f64,
    t_t: Vector3<f64>,
    delta_g_t: Vector3<f64>,
    is_slipping: bool,
    /// Unified force/stiffness stencil entries `(body_idx, node_id, signed_weight)`.
    /// Slave nodes carry `+N_a`, master triangle nodes carry `-xi_k`.
    dof_weights: Vec<(usize, usize, f64)>,
    /// Effective symmetric tangential stiffness tensor factor $\epsilon_T^{\text{eff}} \mathbf{P}_T$ (Pa/m).
    k_t_mat: Matrix3<f64>,
    /// Whether normal penalty stiffness is active (true when active OR zero-gap touching on iter 0).
    normal_stiffness_active: bool,
}

struct StepDofSystem {
    /// Per body: `3 * num_nodes` mapping to reduced DOF index + weight.
    body_dof_map: Vec<Vec<DofMapping>>,
    /// Per body: prescribed fixed DOF values (`dof -> val`).
    body_fixed_values: Vec<HashMap<usize, f64>>,
    /// Per body: resolved external nodal forces.
    body_ext_forces: Vec<Vec<[f64; 3]>>,
    /// Free pilot DOFs (Revolute::Torque and Prismatic::Force hubs).
    pilots: Vec<PilotDofInfo>,
    /// Total number of reduced DOFs (free nodal DOFs + pilot DOFs).
    num_reduced: usize,
}

impl StepDofSystem {
    fn build(
        bodies: &[ContactBody],
        resolved_bcs: &[ResolvedBoundary],
        resolved_hubs: &[Vec<ResolvedHubCoupling>],
        prev_pilot_values: &HashMap<(usize, usize), f64>,
        active_nodes: Option<&[Vec<bool>]>,
        u_frozen: Option<&[Vec<f64>]>,
    ) -> Self {
        let nb = bodies.len();
        let mut body_dof_map = Vec::with_capacity(nb);
        let mut body_fixed_values = Vec::with_capacity(nb);
        let mut body_ext_forces = Vec::with_capacity(nb);
        let mut pilots = Vec::new();

        for b in 0..nb {
            let n_dofs = bodies[b].mesh.node_count() * 3;
            let mut fixed = resolved_bcs[b].fixed_dofs.clone();
            let mut coupled_tag: Vec<Option<(usize, f64)>> = vec![None; n_dofs];
            let mut ext_forces = resolved_bcs[b].nodal_forces.clone();

            // Freeze orphaned nodes (nodes with zero active incident elements after erosion/detachment)
            // at their last known displacement so the reduced tangent matrix remains strictly SPD.
            if let Some(an_all) = active_nodes {
                let an_b = &an_all[b];
                for (nid, &is_active) in an_b.iter().enumerate() {
                    if !is_active {
                        let d0 = 3 * nid;
                        for c in 0..3 {
                            let val = u_frozen
                                .and_then(|uf| uf.get(b))
                                .and_then(|v| v.get(d0 + c))
                                .copied()
                                .unwrap_or(0.0);
                            fixed.insert(d0 + c, val);
                        }
                        if let Some(f) = ext_forces.get_mut(nid) {
                            *f = [0.0; 3];
                        }
                    }
                }
            }

            // Process hub couplings (prescribed angles/displacements become fixed DOFs;
            // torque/force hubs allocate a pilot DOF). Inactive/detached fragment nodes are skipped.
            let is_node_active = |nid: usize| -> bool {
                active_nodes
                    .and_then(|an| an.get(b))
                    .and_then(|mask| mask.get(nid))
                    .copied()
                    .unwrap_or(true)
            };
            for (h_idx, hub) in resolved_hubs[b].iter().enumerate() {
                match hub.kind {
                    ResolvedHubCouplingKind::Fixed => {
                        for &nid in &hub.node_ids {
                            if !is_node_active(nid) {
                                continue;
                            }
                            fixed.insert(3 * nid, 0.0);
                            fixed.insert(3 * nid + 1, 0.0);
                            fixed.insert(3 * nid + 2, 0.0);
                        }
                    }
                    ResolvedHubCouplingKind::Revolute { pivot, axis, drive } => {
                        let p0 = Vector3::from(pivot);
                        let a = Vector3::from(axis);
                        match drive {
                            RevoluteHubDrive::PrescribedAngle { angle_rad } => {
                                for &nid in &hub.node_ids {
                                    if !is_node_active(nid) {
                                        continue;
                                    }
                                    let r0 = bodies[b].mesh.nodes[nid].pos_vec() - p0;
                                    let u = rodrigues_rotate(r0, a, angle_rad) - r0;
                                    fixed.insert(3 * nid, u.x);
                                    fixed.insert(3 * nid + 1, u.y);
                                    fixed.insert(3 * nid + 2, u.z);
                                }
                            }
                            RevoluteHubDrive::Torque {
                                torque_nm,
                                torsional_stiffness_nm_per_rad,
                                reference_angle_rad,
                            } => {
                                let active_hub_nodes: Vec<usize> = hub
                                    .node_ids
                                    .iter()
                                    .copied()
                                    .filter(|&nid| is_node_active(nid))
                                    .collect();
                                if active_hub_nodes.is_empty() {
                                    continue;
                                }
                                let pilot_idx = pilots.len();
                                let init_theta = prev_pilot_values
                                    .get(&(b, h_idx))
                                    .copied()
                                    .unwrap_or(reference_angle_rad);
                                let mut ref_offsets = Vec::with_capacity(active_hub_nodes.len());
                                for &nid in &active_hub_nodes {
                                    let r0 = bodies[b].mesh.nodes[nid].pos_vec() - p0;
                                    ref_offsets.push(r0);
                                    let r_cur = rodrigues_rotate(r0, a, init_theta);
                                    let t_vec = a.cross(&r_cur);
                                    // Coupled revolute nodes override any conflicting zero-fix on those DOFs
                                    // except explicitly fixed axial component if user locked Z (e.g. plane strain).
                                    for c in 0..3 {
                                        let d = 3 * nid + c;
                                        if !fixed.contains_key(&d) {
                                            coupled_tag[d] = Some((pilot_idx, t_vec[c]));
                                        }
                                    }
                                }
                                pilots.push(PilotDofInfo {
                                    body_idx: b,
                                    hub_idx: h_idx,
                                    reduced_idx: 0,
                                    kind: PilotKind::Revolute,
                                    node_ids: active_hub_nodes,
                                    ref_offsets,
                                    axis: a,
                                    free_transverse_axis: None,
                                    applied_effort: torque_nm,
                                    spring_stiffness: torsional_stiffness_nm_per_rad.max(0.0),
                                    spring_ref: reference_angle_rad,
                                    value: init_theta,
                                });
                            }
                        }
                    }
                    ResolvedHubCouplingKind::Prismatic {
                        axis,
                        drive,
                        lock_transverse,
                    } => {
                        let a = Vector3::from(axis);
                        let aligned_axis = if !lock_transverse {
                            if a.x.abs() > 1.0 - 1e-9 {
                                Some(0)
                            } else if a.y.abs() > 1.0 - 1e-9 {
                                Some(1)
                            } else if a.z.abs() > 1.0 - 1e-9 {
                                Some(2)
                            } else {
                                None
                            }
                        } else {
                            None
                        };

                        match drive {
                            PrismaticHubDrive::PrescribedDisplacement { displacement_m } => {
                                for &nid in &hub.node_ids {
                                    if !is_node_active(nid) {
                                        continue;
                                    }
                                    if let Some(comp) = aligned_axis {
                                        let sign = if a[comp] >= 0.0 { 1.0 } else { -1.0 };
                                        fixed.insert(3 * nid + comp, sign * displacement_m);
                                    } else {
                                        let u = a * displacement_m;
                                        fixed.insert(3 * nid, u.x);
                                        fixed.insert(3 * nid + 1, u.y);
                                        fixed.insert(3 * nid + 2, u.z);
                                    }
                                }
                            }
                            PrismaticHubDrive::Force {
                                force_n,
                                axial_stiffness_n_per_m,
                                reference_disp_m,
                            } => {
                                let active_hub_nodes: Vec<usize> = hub
                                    .node_ids
                                    .iter()
                                    .copied()
                                    .filter(|&nid| is_node_active(nid))
                                    .collect();
                                if active_hub_nodes.is_empty() {
                                    continue;
                                }
                                let pilot_idx = pilots.len();
                                let init_s = prev_pilot_values
                                    .get(&(b, h_idx))
                                    .copied()
                                    .unwrap_or(reference_disp_m);
                                for &nid in &active_hub_nodes {
                                    if let Some(comp) = aligned_axis {
                                        let d = 3 * nid + comp;
                                        if !fixed.contains_key(&d) {
                                            let sign = if a[comp] >= 0.0 { 1.0 } else { -1.0 };
                                            coupled_tag[d] = Some((pilot_idx, sign));
                                        }
                                    } else {
                                        for c in 0..3 {
                                            let d = 3 * nid + c;
                                            if !fixed.contains_key(&d) {
                                                coupled_tag[d] = Some((pilot_idx, a[c]));
                                            }
                                        }
                                    }
                                }
                                pilots.push(PilotDofInfo {
                                    body_idx: b,
                                    hub_idx: h_idx,
                                    reduced_idx: 0,
                                    kind: PilotKind::Prismatic,
                                    node_ids: active_hub_nodes,
                                    ref_offsets: Vec::new(),
                                    axis: a,
                                    free_transverse_axis: aligned_axis,
                                    applied_effort: force_n,
                                    spring_stiffness: axial_stiffness_n_per_m.max(0.0),
                                    spring_ref: reference_disp_m,
                                    value: init_s,
                                });
                            }
                        }
                    }
                }
            }

            let mut map = vec![DofMapping::Fixed; n_dofs];
            // Temporary placeholder: mark Free vs Coupled vs Fixed; reduced indices assigned next
            for d in 0..n_dofs {
                if fixed.contains_key(&d) {
                    map[d] = DofMapping::Fixed;
                } else if let Some((pilot_idx, weight)) = coupled_tag[d] {
                    map[d] = DofMapping::Coupled {
                        reduced_idx: pilot_idx, // remapped below once num_free is known
                        weight,
                    };
                } else {
                    map[d] = DofMapping::Free(0);
                }
            }

            body_dof_map.push(map);
            body_fixed_values.push(fixed);
            body_ext_forces.push(ext_forces);
        }

        // Assign contiguous reduced indices: first all Free nodal DOFs, then Pilot DOFs.
        let mut next_reduced = 0usize;
        for map in &mut body_dof_map {
            for entry in map.iter_mut() {
                if let DofMapping::Free(_) = entry {
                    *entry = DofMapping::Free(next_reduced);
                    next_reduced += 1;
                }
            }
        }
        let pilot_base = next_reduced;
        for (p_idx, pilot) in pilots.iter_mut().enumerate() {
            pilot.reduced_idx = pilot_base + p_idx;
            next_reduced += 1;
        }
        for map in &mut body_dof_map {
            for entry in map.iter_mut() {
                if let DofMapping::Coupled {
                    reduced_idx,
                    weight,
                } = *entry
                {
                    *entry = DofMapping::Coupled {
                        reduced_idx: pilot_base + reduced_idx,
                        weight,
                    };
                }
            }
        }

        Self {
            body_dof_map,
            body_fixed_values,
            body_ext_forces,
            pilots,
            num_reduced: next_reduced,
        }
    }

    /// Synchronizes all fixed DOFs and exact finite-rotation/translation hub node displacements
    /// in `u_bodies` and refreshes revolute tangent weights `t_i(theta)` in `body_dof_map`.
    fn enforce_kinematics_and_refresh_weights(&mut self, u_bodies: &mut [Vec<f64>]) {
        for (b, fixed) in self.body_fixed_values.iter().enumerate() {
            for (&dof, &val) in fixed {
                u_bodies[b][dof] = val;
            }
        }

        for pilot in &self.pilots {
            let b = pilot.body_idx;
            let fixed = &self.body_fixed_values[b];
            match pilot.kind {
                PilotKind::Revolute => {
                    for (local_i, &nid) in pilot.node_ids.iter().enumerate() {
                        let r0 = pilot.ref_offsets[local_i];
                        let r_cur = rodrigues_rotate(r0, pilot.axis, pilot.value);
                        let u_exact = r_cur - r0;
                        let t_vec = pilot.axis.cross(&r_cur);
                        for c in 0..3 {
                            let d = 3 * nid + c;
                            if !fixed.contains_key(&d) {
                                u_bodies[b][d] = u_exact[c];
                                self.body_dof_map[b][d] = DofMapping::Coupled {
                                    reduced_idx: pilot.reduced_idx,
                                    weight: t_vec[c],
                                };
                            }
                        }
                    }
                }
                PilotKind::Prismatic => {
                    for &nid in &pilot.node_ids {
                        if let Some(comp) = pilot.free_transverse_axis {
                            let d = 3 * nid + comp;
                            if !fixed.contains_key(&d) {
                                let sign = if pilot.axis[comp] >= 0.0 { 1.0 } else { -1.0 };
                                u_bodies[b][d] = sign * pilot.value;
                            }
                        } else {
                            let u_exact = pilot.axis * pilot.value;
                            for c in 0..3 {
                                let d = 3 * nid + c;
                                if !fixed.contains_key(&d) {
                                    u_bodies[b][d] = u_exact[c];
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Applies reduced step `alpha * dq` to free nodal DOFs and pilot coordinates, then enforces
    /// exact finite-rotation/translation kinematics on all coupled hub nodes.
    fn apply_reduced_increment(
        &mut self,
        u_base: &[Vec<f64>],
        pilots_base: &[f64],
        dq: &[f64],
        alpha: f64,
        u_out: &mut [Vec<f64>],
    ) {
        for b in 0..u_base.len() {
            u_out[b].copy_from_slice(&u_base[b]);
            for (d, mapping) in self.body_dof_map[b].iter().enumerate() {
                if let DofMapping::Free(r_idx) = *mapping {
                    u_out[b][d] += alpha * dq[r_idx];
                }
            }
        }
        for (p_idx, pilot) in self.pilots.iter_mut().enumerate() {
            pilot.value = pilots_base[p_idx] + alpha * dq[pilot.reduced_idx];
        }
        self.enforce_kinematics_and_refresh_weights(u_out);
    }

    #[inline]
    fn reduced_slot(&self, body_idx: usize, dof: usize) -> Option<(usize, f64)> {
        match self.body_dof_map[body_idx][dof] {
            DofMapping::Free(r) => Some((r, 1.0)),
            DofMapping::Coupled {
                reduced_idx,
                weight,
            } => Some((reduced_idx, weight)),
            DofMapping::Fixed => None,
        }
    }
}

// ============================================================================
// Stencil Initialization & Contact Evaluation
// ============================================================================

fn effective_modulus_pair(m1: &J2MaterialParams, m2: Option<&J2MaterialParams>) -> f64 {
    let c1 = (1.0 - m1.poissons_ratio * m1.poissons_ratio) / m1.youngs_modulus_pa;
    let c2 = match m2 {
        Some(m) => (1.0 - m.poissons_ratio * m.poissons_ratio) / m.youngs_modulus_pa,
        None => 0.0,
    };
    1.0 / (c1 + c2).max(1e-30)
}

fn build_contact_stencils(
    bodies: &[ContactBody],
    surfaces: &[DeformedSurfaceMesh],
    pairs: &[ContactPairSpec],
    options: ContactSolverOptions,
) -> Result<Vec<ContactSlaveStencil>, FeaError> {
    let mut stencils = Vec::new();
    let nb = bodies.len();

    let mut add_pass = |pair_index: usize,
                        slave_body: usize,
                        master_body: Option<usize>,
                        rigid_surface: Option<AnalyticalRigidSurface>,
                        slave_filter: Option<&[usize]>,
                        area_factor: f64,
                        friction_coeff: f64,
                        search_distance: Option<f64>,
                        max_normal_dot: Option<f64>|
     -> Result<(), FeaError> {
        if slave_body >= nb {
            return Err(FeaError::General(format!(
                "ContactPairSpec slave body index {slave_body} out of bounds (bodies: {nb})"
            )));
        }
        if let Some(mb) = master_body {
            if mb >= nb || mb == slave_body {
                return Err(FeaError::General(format!(
                    "ContactPairSpec invalid master body index {mb} for slave {slave_body}"
                )));
            }
        }

        let slave_mesh = &bodies[slave_body].mesh;
        let slave_surf = &surfaces[slave_body];
        let e_star = effective_modulus_pair(
            &bodies[slave_body].material,
            master_body.map(|mb| &bodies[mb].material),
        );
        let h_char = match master_body {
            Some(mb) => slave_surf.char_edge_len.min(surfaces[mb].char_edge_len),
            None => slave_surf.char_edge_len,
        }
        .max(1e-6);

        let eps_n = (options.penalty_scale.max(0.5) * e_star / h_char).max(1e6);
        let eps_t = (eps_n * options.tangential_penalty_ratio.max(0.05)).max(1e5);
        let search_dist = search_distance.unwrap_or_else(|| (8.0 * h_char).max(1e-3));
        let normal_dot_lim = max_normal_dot.unwrap_or(0.35);
        let mu = friction_coeff.max(0.0);

        let filter_set: Option<HashSet<usize>> =
            slave_filter.map(|ids| ids.iter().copied().collect());

        let disc = match options.discretization {
            ContactDiscretization::Auto => {
                if slave_mesh.is_tet10() {
                    ContactDiscretization::MortarGauss
                } else {
                    ContactDiscretization::NodeToSurface
                }
            }
            other => other,
        };

        match disc {
            ContactDiscretization::NodeToSurface => {
                for &nid in &slave_surf.surface_nodes {
                    if let Some(ref fs) = filter_set {
                        if !fs.contains(&nid) {
                            continue;
                        }
                    }
                    if slave_surf.nodal_areas[nid] <= 1e-20 {
                        continue;
                    }
                    stencils.push(ContactSlaveStencil {
                        pair_index,
                        slave_body,
                        master_body,
                        rigid_surface,
                        rep_node_id: nid,
                        slave_nodes: vec![(nid, 1.0)],
                        mortar_face_corners: None,
                        area_factor,
                        friction_coeff: mu,
                        search_distance: search_dist,
                        max_normal_dot: normal_dot_lim,
                        lambda_n: 0.0,
                        lambda_t: Vector3::zeros(),
                        eps_n,
                        eps_t,
                    });
                }
            }
            ContactDiscretization::MortarGauss | ContactDiscretization::Auto => {
                // 3-point Hammer quadrature on triangle faces: (2/3, 1/6, 1/6) permutations
                const HAMMER3: [[f64; 3]; 3] = [
                    [2.0 / 3.0, 1.0 / 6.0, 1.0 / 6.0],
                    [1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0],
                    [1.0 / 6.0, 1.0 / 6.0, 2.0 / 3.0],
                ];
                if slave_mesh.is_tet10() {
                    for &(corners, mids) in &slave_surf.quad_faces {
                        if let Some(ref fs) = filter_set {
                            if !corners.iter().all(|n| fs.contains(n)) {
                                continue;
                            }
                        }
                        for (q_idx, &[l0, l1, l2]) in HAMMER3.iter().enumerate() {
                            let n0 = l0 * (2.0 * l0 - 1.0);
                            let n1 = l1 * (2.0 * l1 - 1.0);
                            let n2 = l2 * (2.0 * l2 - 1.0);
                            let n01 = 4.0 * l0 * l1;
                            let n12 = 4.0 * l1 * l2;
                            let n20 = 4.0 * l2 * l0;
                            let slave_nodes = vec![
                                (corners[0], n0),
                                (corners[1], n1),
                                (corners[2], n2),
                                (mids[0], n01),
                                (mids[1], n12),
                                (mids[2], n20),
                            ];
                            stencils.push(ContactSlaveStencil {
                                pair_index,
                                slave_body,
                                master_body,
                                rigid_surface,
                                rep_node_id: corners[q_idx],
                                slave_nodes,
                                mortar_face_corners: Some(corners),
                                area_factor: area_factor / 3.0,
                                friction_coeff: mu,
                                search_distance: search_dist,
                                max_normal_dot: normal_dot_lim,
                                lambda_n: 0.0,
                                lambda_t: Vector3::zeros(),
                                eps_n,
                                eps_t,
                            });
                        }
                    }
                } else {
                    for &tri in &slave_surf.triangles {
                        if let Some(ref fs) = filter_set {
                            if !tri.iter().all(|n| fs.contains(n)) {
                                continue;
                            }
                        }
                        for (q_idx, &[l0, l1, l2]) in HAMMER3.iter().enumerate() {
                            stencils.push(ContactSlaveStencil {
                                pair_index,
                                slave_body,
                                master_body,
                                rigid_surface,
                                rep_node_id: tri[q_idx],
                                slave_nodes: vec![(tri[0], l0), (tri[1], l1), (tri[2], l2)],
                                mortar_face_corners: Some(tri),
                                area_factor: area_factor / 3.0,
                                friction_coeff: mu,
                                search_distance: search_dist,
                                max_normal_dot: normal_dot_lim,
                                lambda_n: 0.0,
                                lambda_t: Vector3::zeros(),
                                eps_n,
                                eps_t,
                            });
                        }
                    }
                }
            }
        }
        Ok(())
    };

    for (pair_index, pair) in pairs.iter().enumerate() {
        match pair {
            ContactPairSpec::DeformablePair {
                body_a,
                body_b,
                friction_coeff,
                symmetric,
                slave_a_nodes,
                slave_b_nodes,
                search_distance,
                max_normal_dot,
            } => {
                let factor = if *symmetric { 0.5 } else { 1.0 };
                add_pass(
                    pair_index,
                    *body_a,
                    Some(*body_b),
                    None,
                    slave_a_nodes.as_deref(),
                    factor,
                    *friction_coeff,
                    *search_distance,
                    *max_normal_dot,
                )?;
                if *symmetric {
                    add_pass(
                        pair_index,
                        *body_b,
                        Some(*body_a),
                        None,
                        slave_b_nodes.as_deref(),
                        factor,
                        *friction_coeff,
                        *search_distance,
                        *max_normal_dot,
                    )?;
                }
            }
            ContactPairSpec::DeformableToRigid {
                body_idx,
                surface,
                friction_coeff,
                slave_nodes,
                search_distance,
            } => {
                add_pass(
                    pair_index,
                    *body_idx,
                    None,
                    Some(*surface),
                    slave_nodes.as_deref(),
                    1.0,
                    *friction_coeff,
                    *search_distance,
                    None,
                )?;
            }
        }
    }

    Ok(stencils)
}

fn evaluate_contact_points(
    stencils: &[ContactSlaveStencil],
    surfaces: &[DeformedSurfaceMesh],
    u_bodies: &[Vec<f64>],
    u_step_start: &[Vec<f64>],
    activate_zero_gap_stiffness: bool,
) -> Vec<EvaluatedContactPoint> {
    let mut evaluated = Vec::with_capacity(stencils.len());

    for (s_idx, st) in stencils.iter().enumerate() {
        let slave_surf = &surfaces[st.slave_body];

        // Compute deformed slave point position, area weight, slave normal, and step displacement
        let mut p_s = Vector3::zeros();
        let mut du_s = Vector3::zeros();
        let mut n_s_acc = Vector3::zeros();
        for &(nid, w) in &st.slave_nodes {
            p_s += Vector3::from(slave_surf.current_positions[nid]) * w;
            let d0 = 3 * nid;
            du_s += Vector3::new(
                u_bodies[st.slave_body][d0] - u_step_start[st.slave_body][d0],
                u_bodies[st.slave_body][d0 + 1] - u_step_start[st.slave_body][d0 + 1],
                u_bodies[st.slave_body][d0 + 2] - u_step_start[st.slave_body][d0 + 2],
            ) * w;
            n_s_acc += Vector3::from(slave_surf.vertex_normals[nid]) * w;
        }
        let slave_n = if n_s_acc.norm() > 1e-12 {
            let ns = n_s_acc.normalize();
            Some([ns.x, ns.y, ns.z])
        } else {
            None
        };

        type MasterProjectionEval = (f64, Vector3<f64>, Vector3<f64>, Vec<(usize, usize, f64)>);
        let (gap, normal_vec, du_m, master_entries): MasterProjectionEval =
            if let Some(mb) = st.master_body {
            let Some(proj) = surfaces[mb].closest_point_signed(
                [p_s.x, p_s.y, p_s.z],
                slave_n,
                st.search_distance,
                st.max_normal_dot,
            ) else {
                continue;
            };
            let mut du_master = Vector3::zeros();
            let mut m_ent = Vec::with_capacity(3);
            for k in 0..3 {
                let mnid = proj.master_nodes[k];
                let xi = proj.barycentric[k];
                if xi.abs() > 1e-14 {
                    let d0 = 3 * mnid;
                    du_master += Vector3::new(
                        u_bodies[mb][d0] - u_step_start[mb][d0],
                        u_bodies[mb][d0 + 1] - u_step_start[mb][d0 + 1],
                        u_bodies[mb][d0 + 2] - u_step_start[mb][d0 + 2],
                    ) * xi;
                    m_ent.push((mb, mnid, -xi));
                }
            }
            (proj.signed_gap, Vector3::from(proj.normal), du_master, m_ent)
        } else if let Some(rigid) = st.rigid_surface {
            let (gap, n_arr, _) = project_onto_analytical_rigid([p_s.x, p_s.y, p_s.z], rigid);
            if gap > st.search_distance {
                continue;
            }
            (gap, Vector3::from(n_arr), Vector3::zeros(), Vec::new())
        } else {
            continue;
        };

        let area = if let Some([c0, c1, c2]) = st.mortar_face_corners {
            let p0 = Vector3::from(slave_surf.current_positions[c0]);
            let p1 = Vector3::from(slave_surf.current_positions[c1]);
            let p2 = Vector3::from(slave_surf.current_positions[c2]);
            0.5 * (p1 - p0).cross(&(p2 - p0)).norm() * st.area_factor
        } else {
            slave_surf.contact_nodal_area(st.rep_node_id, normal_vec) * st.area_factor
        };
        if area <= 1e-22 {
            continue;
        }

        let t_n_trial = st.lambda_n - st.eps_n * gap;
        let is_active = t_n_trial > 0.0;
        let zero_gap_touch = !is_active && activate_zero_gap_stiffness && gap <= 1e-10;

        if !is_active && !zero_gap_touch && gap > 0.0 {
            continue;
        }

        let t_n = if is_active { t_n_trial } else { 0.0 };

        // Incremental tangential slip projected onto current tangent plane
        let du_rel = du_s - du_m;
        let delta_g_t = du_rel - normal_vec * du_rel.dot(&normal_vec);

        let mut t_t = Vector3::zeros();
        let mut is_slipping = false;
        let mut k_t_mat = Matrix3::zeros();

        if is_active && st.friction_coeff > 1e-12 {
            let lam_t_proj = st.lambda_t - normal_vec * st.lambda_t.dot(&normal_vec);
            let t_t_trial = lam_t_proj - delta_g_t * st.eps_t;
            let tau_trial = t_t_trial.norm();
            let tau_max = st.friction_coeff * t_n;
            let p_tang = Matrix3::identity() - normal_vec * normal_vec.transpose();

            if tau_trial <= tau_max {
                t_t = t_t_trial;
                k_t_mat = p_tang * st.eps_t;
            } else if tau_trial > 1e-18 {
                let d_hat = t_t_trial / tau_trial;
                t_t = d_hat * tau_max;
                is_slipping = true;
                // Symmetric positive semi-definite projection of slip tangent
                let p_ortho = p_tang - d_hat * d_hat.transpose();
                k_t_mat = p_ortho * (st.eps_t * (tau_max / tau_trial));
            }
        }

        let mut dof_weights = Vec::with_capacity(st.slave_nodes.len() + master_entries.len());
        for &(snid, sw) in &st.slave_nodes {
            if sw.abs() > 1e-14 {
                dof_weights.push((st.slave_body, snid, sw));
            }
        }
        dof_weights.extend(master_entries);

        evaluated.push(EvaluatedContactPoint {
            stencil_idx: s_idx,
            active: is_active,
            position: [p_s.x, p_s.y, p_s.z],
            normal: normal_vec,
            signed_gap: gap,
            area,
            t_n,
            t_t,
            delta_g_t,
            is_slipping,
            dof_weights,
            k_t_mat,
            normal_stiffness_active: is_active || zero_gap_touch,
        });
    }

    evaluated
}

// ============================================================================
// Global Multi-Body Tangent + Residual Assembly
// ============================================================================

struct GlobalEval {
    residual_red: Vec<f64>,
    k_red: Option<CsrMatrix>,
    f_int_bodies: Vec<Vec<f64>>,
    f_contact_bodies: Vec<Vec<f64>>,
    new_gp_bodies: Vec<Vec<GaussPointState>>,
    new_rot_bodies: Vec<Vec<[[f64; 3]; 3]>>,
}

#[allow(clippy::too_many_arguments)]
fn evaluate_multibody_system(
    bodies: &[ContactBody],
    body_elems: &[BodyElements],
    dof_sys: &StepDofSystem,
    u_bodies: &[Vec<f64>],
    prev_gp_bodies: &[Vec<GaussPointState>],
    stencils: &[ContactSlaveStencil],
    contact_points: &[EvaluatedContactPoint],
    options: ContactSolverOptions,
    assemble_tangent: bool,
    active_elements: Option<&[Vec<bool>]>,
    element_stiffness_scale: Option<&[Vec<f64>]>,
) -> GlobalEval {
    let nb = bodies.len();
    let n_red = dof_sys.num_reduced;

    let mut f_int_bodies = Vec::with_capacity(nb);
    let mut f_contact_bodies = Vec::with_capacity(nb);
    let mut new_gp_bodies = Vec::with_capacity(nb);
    let mut new_rot_bodies = Vec::with_capacity(nb);

    let mut row_entries: Vec<Vec<(usize, f64)>> = if assemble_tangent {
        vec![Vec::with_capacity(64); n_red]
    } else {
        Vec::new()
    };

    // 1. Continuum Co-Rotated + J2 Elastoplastic elements for every body
    for b in 0..nb {
        let n_dofs = bodies[b].mesh.node_count() * 3;
        let mut f_int = vec![0.0; n_dofs];
        let params = &bodies[b].material;
        let u_b = &u_bodies[b];
        let prev_gp = &prev_gp_bodies[b];
        let active_b = active_elements.map(|m| m[b].as_slice());
        let scale_b = element_stiffness_scale.map(|s| s[b].as_slice());

        match &body_elems[b] {
            BodyElements::Tet4(elems) => {
                let mut new_gp = Vec::with_capacity(elems.len());
                let mut new_rots = Vec::with_capacity(elems.len());

                let eval_tet4 = |(e_idx, elem): (usize, &Tet4Element)| {
                    if active_b.is_some_and(|m| !m.get(e_idx).copied().unwrap_or(true)) {
                        return (
                            false,
                            1.0_f64,
                            [0usize; 12],
                            Vector12::zeros(),
                            None,
                            prev_gp[e_idx],
                            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                        );
                    }
                    let d_scale = scale_b
                        .and_then(|s| s.get(e_idx).copied())
                        .unwrap_or(1.0);
                    let mut u_elem = Vector12::zeros();
                    let mut elem_dofs = [0usize; 12];
                    for (li, &nid) in elem.node_indices.iter().enumerate() {
                        let d0 = 3 * nid;
                        elem_dofs[3 * li] = d0;
                        elem_dofs[3 * li + 1] = d0 + 1;
                        elem_dofs[3 * li + 2] = d0 + 2;
                        u_elem[3 * li] = u_b[d0];
                        u_elem[3 * li + 1] = u_b[d0 + 1];
                        u_elem[3 * li + 2] = u_b[d0 + 2];
                    }
                    if assemble_tangent {
                        let (f_e, k_e, rm, r_mat) = elem.evaluate_nonlinear(
                            &u_elem,
                            &prev_gp[e_idx],
                            params,
                            options.corotated,
                            options.include_geometric_stiffness && d_scale >= 0.99,
                        );
                        (
                            true,
                            d_scale,
                            elem_dofs,
                            f_e,
                            Some(k_e),
                            rm.state,
                            matrix3_to_rows(&r_mat),
                        )
                    } else {
                        let (f_e, rm, r_mat) = elem.evaluate_nonlinear_forces_only(
                            &u_elem,
                            &prev_gp[e_idx],
                            params,
                            options.corotated,
                        );
                        (
                            true,
                            d_scale,
                            elem_dofs,
                            f_e,
                            None,
                            rm.state,
                            matrix3_to_rows(&r_mat),
                        )
                    }
                };

                let results: Vec<_> = if elems.len() >= CONTACT_PAR_ELEMENT_THRESHOLD {
                    elems.par_iter().enumerate().map(eval_tet4).collect()
                } else {
                    elems.iter().enumerate().map(eval_tet4).collect()
                };

                for (is_active, d_scale, elem_dofs, f_e, k_opt, gp_state, rot_rows) in results {
                    new_gp.push(gp_state);
                    new_rots.push(rot_rows);
                    if !is_active {
                        continue;
                    }
                    if let Some(k_e) = k_opt {
                        for a in 0..12 {
                            let da = elem_dofs[a];
                            f_int[da] += d_scale * f_e[a];
                            if let Some((ra, wa)) = dof_sys.reduced_slot(b, da) {
                                if wa.abs() <= 1e-22 {
                                    continue;
                                }
                                let row_vec = &mut row_entries[ra];
                                for col_b in 0..12 {
                                    let val = d_scale * k_e[(a, col_b)];
                                    if val.abs() <= 1e-25 {
                                        continue;
                                    }
                                    if let Some((rb, wb)) = dof_sys.reduced_slot(b, elem_dofs[col_b])
                                    {
                                        let prod = wa * wb * val;
                                        if prod.abs() > 1e-25 {
                                            row_vec.push((rb, prod));
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        for a in 0..12 {
                            f_int[elem_dofs[a]] += d_scale * f_e[a];
                        }
                    }
                }
                new_gp_bodies.push(new_gp);
                new_rot_bodies.push(new_rots);
            }
            BodyElements::Tet10(elems) => {
                let mut new_gp = Vec::with_capacity(elems.len() * 4);
                let mut new_rots = Vec::with_capacity(elems.len() * 4);

                let eval_tet10 = |(e_idx, elem): (usize, &Tet10Element)| {
                    let base = e_idx * 4;
                    let eye = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
                    if active_b.is_some_and(|m| !m.get(e_idx).copied().unwrap_or(true)) {
                        return (
                            false,
                            1.0_f64,
                            [0usize; 30],
                            Vector30::zeros(),
                            None,
                            [
                                prev_gp[base],
                                prev_gp[base + 1],
                                prev_gp[base + 2],
                                prev_gp[base + 3],
                            ],
                            [eye; 4],
                        );
                    }
                    let d_scale = scale_b
                        .and_then(|s| s.get(e_idx).copied())
                        .unwrap_or(1.0);
                    let mut u_elem = Vector30::zeros();
                    let mut elem_dofs = [0usize; 30];
                    for (li, &nid) in elem.node_indices.iter().enumerate() {
                        let d0 = 3 * nid;
                        elem_dofs[3 * li] = d0;
                        elem_dofs[3 * li + 1] = d0 + 1;
                        elem_dofs[3 * li + 2] = d0 + 2;
                        u_elem[3 * li] = u_b[d0];
                        u_elem[3 * li + 1] = u_b[d0 + 1];
                        u_elem[3 * li + 2] = u_b[d0 + 2];
                    }
                    let prev4 = [
                        prev_gp[base],
                        prev_gp[base + 1],
                        prev_gp[base + 2],
                        prev_gp[base + 3],
                    ];
                    if assemble_tangent {
                        let (f_e, k_e, rms, rots) = elem.evaluate_nonlinear(
                            &u_elem,
                            &prev4,
                            params,
                            options.corotated,
                            options.include_geometric_stiffness,
                        );
                        (
                            true,
                            d_scale,
                            elem_dofs,
                            f_e,
                            Some(k_e),
                            [rms[0].state, rms[1].state, rms[2].state, rms[3].state],
                            [
                                matrix3_to_rows(&rots[0]),
                                matrix3_to_rows(&rots[1]),
                                matrix3_to_rows(&rots[2]),
                                matrix3_to_rows(&rots[3]),
                            ],
                        )
                    } else {
                        let (f_e, rms, rots) = elem.evaluate_nonlinear_forces_only(
                            &u_elem,
                            &prev4,
                            params,
                            options.corotated,
                        );
                        (
                            true,
                            d_scale,
                            elem_dofs,
                            f_e,
                            None,
                            [rms[0].state, rms[1].state, rms[2].state, rms[3].state],
                            [
                                matrix3_to_rows(&rots[0]),
                                matrix3_to_rows(&rots[1]),
                                matrix3_to_rows(&rots[2]),
                                matrix3_to_rows(&rots[3]),
                            ],
                        )
                    }
                };

                let results: Vec<_> = if elems.len() >= CONTACT_PAR_ELEMENT_THRESHOLD {
                    elems.par_iter().enumerate().map(eval_tet10).collect()
                } else {
                    elems.iter().enumerate().map(eval_tet10).collect()
                };

                for (is_active, d_scale, elem_dofs, f_e, k_opt, gp4, rots4) in results {
                    for q in 0..4 {
                        new_gp.push(gp4[q]);
                        new_rots.push(rots4[q]);
                    }
                    if !is_active {
                        continue;
                    }
                    if let Some(k_e) = k_opt {
                        for a in 0..30 {
                            let da = elem_dofs[a];
                            f_int[da] += d_scale * f_e[a];
                            if let Some((ra, wa)) = dof_sys.reduced_slot(b, da) {
                                if wa.abs() <= 1e-22 {
                                    continue;
                                }
                                let row_vec = &mut row_entries[ra];
                                for col_b in 0..30 {
                                    let val = d_scale * k_e[(a, col_b)];
                                    if val.abs() <= 1e-25 {
                                        continue;
                                    }
                                    if let Some((rb, wb)) = dof_sys.reduced_slot(b, elem_dofs[col_b])
                                    {
                                        let prod = wa * wb * val;
                                        if prod.abs() > 1e-25 {
                                            row_vec.push((rb, prod));
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        for a in 0..30 {
                            f_int[elem_dofs[a]] += d_scale * f_e[a];
                        }
                    }
                }
                new_gp_bodies.push(new_gp);
                new_rot_bodies.push(new_rots);
            }
        }

        f_int_bodies.push(f_int);
        f_contact_bodies.push(vec![0.0; n_dofs]);
    }

    // 2. Contact nodal forces & symmetric positive semi-definite contact tangent stiffness
    for cp in contact_points {
        let st = &stencils[cp.stencil_idx];
        if cp.active {
            let f_c = (cp.normal * cp.t_n + cp.t_t) * cp.area;
            for &(b_idx, nid, w_node) in &cp.dof_weights {
                let d0 = 3 * nid;
                f_contact_bodies[b_idx][d0] += w_node * f_c.x;
                f_contact_bodies[b_idx][d0 + 1] += w_node * f_c.y;
                f_contact_bodies[b_idx][d0 + 2] += w_node * f_c.z;
            }
        }

        if assemble_tangent && cp.normal_stiffness_active {
            let k_n = (cp.normal * cp.normal.transpose()) * (cp.area * st.eps_n);
            let k_loc = k_n + cp.k_t_mat * cp.area;

            for &(ba, na, wa_node) in &cp.dof_weights {
                for ca in 0..3 {
                    let da = 3 * na + ca;
                    let Some((ra, wa_red)) = dof_sys.reduced_slot(ba, da) else {
                        continue;
                    };
                    let wa = wa_node * wa_red;
                    if wa.abs() <= 1e-22 {
                        continue;
                    }
                    let row_vec = &mut row_entries[ra];
                    for &(bb, nb_id, wb_node) in &cp.dof_weights {
                        for cb in 0..3 {
                            let val = k_loc[(ca, cb)];
                            if val.abs() <= 1e-25 {
                                continue;
                            }
                            let db = 3 * nb_id + cb;
                            if let Some((rb, wb_red)) = dof_sys.reduced_slot(bb, db) {
                                let prod = wa * (wb_node * wb_red) * val;
                                if prod.abs() > 1e-25 {
                                    row_vec.push((rb, prod));
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // 3. Assemble reduced residual: R_red = B^T (f_ext + f_contact - f_int) + f_pilot_ext
    let mut residual_red = vec![0.0; n_red];
    for b in 0..nb {
        let n_nodes = bodies[b].mesh.node_count();
        for nid in 0..n_nodes {
            let f_ext = dof_sys.body_ext_forces[b][nid];
            for (c, &f_ext_c) in f_ext.iter().enumerate() {
                let d = 3 * nid + c;
                if let Some((r_idx, w)) = dof_sys.reduced_slot(b, d) {
                    let net = f_ext_c + f_contact_bodies[b][d] - f_int_bodies[b][d];
                    residual_red[r_idx] += w * net;
                }
            }
        }
    }

    // Add pilot external effort and optional torsional/axial spring stiffness
    for pilot in &dof_sys.pilots {
        let spring_force = pilot.spring_stiffness * (pilot.value - pilot.spring_ref);
        residual_red[pilot.reduced_idx] += pilot.applied_effort - spring_force;
        if assemble_tangent && pilot.spring_stiffness > 0.0 {
            row_entries[pilot.reduced_idx].push((pilot.reduced_idx, pilot.spring_stiffness));
        }
    }

    let k_red = if assemble_tangent {
        Some(CsrMatrix::from_row_entries(n_red, n_red, row_entries))
    } else {
        None
    };

    GlobalEval {
        residual_red,
        k_red,
        f_int_bodies,
        f_contact_bodies,
        new_gp_bodies,
        new_rot_bodies,
    }
}

fn vec_norm(v: &[f64]) -> f64 {
    v.iter().map(|&x| x * x).sum::<f64>().sqrt()
}

/// Solves `K_red * dq = rhs` using RCM sparse direct $LDL^T$ / `IC(0)-PCG` / `SSOR-PCG`
/// with automatic Levenberg–Marquardt tangent regularization ONLY when `K_red` is nearly
/// singular (e.g., floating body before contact closes).
fn solve_contact_tangent(
    k_red: &CsrMatrix,
    rhs: &[f64],
    pilots: &[PilotDofInfo],
    lin_opts: SolverOptions,
) -> Result<Vec<f64>, FeaError> {
    let _ = pilots;
    solve_spd_newton_robust(k_red, rhs, LinearSolverKind::Auto, lin_opts)
}

/// Computes worst geometric interpenetration across ALL pairs (explicitly checking both
/// `A -> B` and `B -> A` for every deformable pair so no unmonitored corner can penetrate).
fn measure_pair_penetrations(
    pairs: &[ContactPairSpec],
    surfaces: &[DeformedSurfaceMesh],
) -> Vec<f64> {
    let mut worst_per_pair = vec![0.0_f64; pairs.len()];

    for (p_idx, pair) in pairs.iter().enumerate() {
        match pair {
            ContactPairSpec::DeformablePair {
                body_a,
                body_b,
                slave_a_nodes,
                slave_b_nodes,
                search_distance,
                max_normal_dot,
                ..
            } => {
                let h = surfaces[*body_a]
                    .char_edge_len
                    .min(surfaces[*body_b].char_edge_len)
                    .max(1e-6);
                let sdist = search_distance.unwrap_or_else(|| (6.0 * h).max(1e-3));
                let ndot = max_normal_dot.unwrap_or(0.35);

                // Check A -> B
                let nodes_a: &[usize] = slave_a_nodes
                    .as_deref()
                    .unwrap_or(&surfaces[*body_a].surface_nodes);
                for &nid in nodes_a {
                    let p = surfaces[*body_a].current_positions[nid];
                    let sn = surfaces[*body_a].vertex_normals[nid];
                    if let Some(proj) =
                        surfaces[*body_b].closest_point_signed(p, Some(sn), sdist, ndot)
                    {
                        if proj.signed_gap < 0.0 {
                            worst_per_pair[p_idx] = worst_per_pair[p_idx].max(-proj.signed_gap);
                        }
                    }
                }
                // Check B -> A
                let nodes_b: &[usize] = slave_b_nodes
                    .as_deref()
                    .unwrap_or(&surfaces[*body_b].surface_nodes);
                for &nid in nodes_b {
                    let p = surfaces[*body_b].current_positions[nid];
                    let sn = surfaces[*body_b].vertex_normals[nid];
                    if let Some(proj) =
                        surfaces[*body_a].closest_point_signed(p, Some(sn), sdist, ndot)
                    {
                        if proj.signed_gap < 0.0 {
                            worst_per_pair[p_idx] = worst_per_pair[p_idx].max(-proj.signed_gap);
                        }
                    }
                }
            }
            ContactPairSpec::DeformableToRigid {
                body_idx,
                surface,
                slave_nodes,
                ..
            } => {
                let nodes: &[usize] = slave_nodes
                    .as_deref()
                    .unwrap_or(&surfaces[*body_idx].surface_nodes);
                for &nid in nodes {
                    let p = surfaces[*body_idx].current_positions[nid];
                    let (gap, _, _) = project_onto_analytical_rigid(p, *surface);
                    if gap < 0.0 {
                        worst_per_pair[p_idx] = worst_per_pair[p_idx].max(-gap);
                    }
                }
            }
        }
    }

    worst_per_pair
}

// ============================================================================
// Main Multi-Body Deformable Contact Solver Entry Point
// ============================================================================

/// Solves a multi-body deformable contact mechanics history using Co-Rotated FEM + $J_2$
/// elastoplasticity, exact RBE2 kinematic [`HubCoupling`]s, dynamic per-Newton-iteration deformed
/// surface contact search, Augmented Lagrangian normal + Coulomb friction enforcement (`P5-T4`),
/// and adaptive step bisection on Newton/AL non-convergence (`P5-T13`).
pub fn solve_deformable_contact(
    bodies: &[ContactBody],
    pairs: &[ContactPairSpec],
    steps: &[ContactStepSpec],
    options: ContactSolverOptions,
) -> Result<DeformableContactReport, FeaError> {
    let start_time = Instant::now();
    let mut session = MaskedContactSession::new(bodies, pairs, options)?;

    let effective_steps: Vec<ContactStepSpec> = if steps.is_empty() {
        vec![ContactStepSpec::new("step_0")]
    } else {
        steps.to_vec()
    };

    let mut step_reports = Vec::with_capacity(effective_steps.len());
    for (step_idx, step_spec) in effective_steps.iter().enumerate() {
        let report = session.solve_step_internal(
            step_idx, bodies, pairs, step_spec, options, None, None, true, false,
        )?;
        step_reports.push(report);
    }

    let (body_states, body_deformed_positions) = session.into_states_and_positions();

    Ok(DeformableContactReport {
        steps: step_reports,
        body_states,
        body_deformed_positions,
        elapsed_ms: start_time.elapsed().as_secs_f64() * 1000.0,
    })
}

/// Reusable SSOR-PCG + Levenberg–Marquardt fallback linear solver exposed to `crate::fracture`
/// for softened / near-fracture tangent matrices.
pub(crate) fn solve_spd_with_fallback(
    k_red: &CsrMatrix,
    rhs: &[f64],
    lin_opts: SolverOptions,
) -> Result<Vec<f64>, FeaError> {
    solve_contact_tangent(k_red, rhs, &[], lin_opts)
}

/// Computes per-body active node mask: a node is active iff at least one active element
/// (in `active_elements[b]`) references it.
fn compute_active_nodes_from_elements(
    bodies: &[ContactBody],
    active_elements: &[Vec<bool>],
) -> Vec<Vec<bool>> {
    let mut active_nodes = Vec::with_capacity(bodies.len());
    for (b, body) in bodies.iter().enumerate() {
        let mut mask = vec![false; body.mesh.node_count()];
        if body.mesh.is_tet10() {
            for (e_idx, nodes) in body.mesh.tet10_elements.iter().enumerate() {
                if active_elements[b].get(e_idx).copied().unwrap_or(true) {
                    for &nid in nodes {
                        if nid < mask.len() {
                            mask[nid] = true;
                        }
                    }
                }
            }
        } else {
            for (e_idx, elem) in body.mesh.elements.iter().enumerate() {
                if active_elements[b].get(e_idx).copied().unwrap_or(true) {
                    for &nid in &elem.node_indices {
                        if nid < mask.len() {
                            mask[nid] = true;
                        }
                    }
                }
            }
        }
        active_nodes.push(mask);
    }
    active_nodes
}

/// Interpolates resolved Dirichlet and Neumann boundary conditions between the previous committed
/// step (`prev_bcs`, or the undeformed zero state at step 0) and `target_bcs` at `lambda in [0, 1]`.
fn interpolate_resolved_boundaries(
    prev_bcs: Option<&[ResolvedBoundary]>,
    target_bcs: &[ResolvedBoundary],
    u_step_entry: &[Vec<f64>],
    lambda: f64,
) -> Vec<ResolvedBoundary> {
    let lam = lambda.clamp(0.0, 1.0);
    target_bcs
        .iter()
        .enumerate()
        .map(|(b, target)| {
            let prev_opt = prev_bcs.and_then(|v| v.get(b));
            let mut fixed_dofs = HashMap::with_capacity(target.fixed_dofs.len());
            for (&dof, &target_val) in &target.fixed_dofs {
                let prev_val = prev_opt
                    .and_then(|p| p.fixed_dofs.get(&dof).copied())
                    .unwrap_or_else(|| u_step_entry[b].get(dof).copied().unwrap_or(0.0));
                fixed_dofs.insert(dof, prev_val + lam * (target_val - prev_val));
            }
            let nodal_forces = target
                .nodal_forces
                .iter()
                .enumerate()
                .map(|(nid, &tf)| {
                    let pf = prev_opt
                        .and_then(|p| p.nodal_forces.get(nid).copied())
                        .unwrap_or([0.0; 3]);
                    [
                        pf[0] + lam * (tf[0] - pf[0]),
                        pf[1] + lam * (tf[1] - pf[1]),
                        pf[2] + lam * (tf[2] - pf[2]),
                    ]
                })
                .collect();
            ResolvedBoundary {
                fixed_dofs,
                nodal_forces,
            }
        })
        .collect()
}

/// Interpolates resolved kinematic hub drives (prescribed angles/displacements and applied
/// torques/forces) between the previous committed step and `target_hubs` at `lambda in [0, 1]`.
fn interpolate_resolved_hubs(
    prev_hubs: Option<&[Vec<ResolvedHubCoupling>]>,
    target_hubs: &[Vec<ResolvedHubCoupling>],
    start_prescribed_angles: &HashMap<(usize, usize), f64>,
    lambda: f64,
) -> Vec<Vec<ResolvedHubCoupling>> {
    let lam = lambda.clamp(0.0, 1.0);
    target_hubs
        .iter()
        .enumerate()
        .map(|(b, hubs_b)| {
            hubs_b
                .iter()
                .enumerate()
                .map(|(h_idx, target)| {
                    let prev_kind = prev_hubs
                        .and_then(|v| v.get(b))
                        .and_then(|vh| vh.get(h_idx))
                        .map(|h| h.kind);
                    let kind = match target.kind {
                        ResolvedHubCouplingKind::Fixed => ResolvedHubCouplingKind::Fixed,
                        ResolvedHubCouplingKind::Revolute { pivot, axis, drive } => {
                            let new_drive = match drive {
                                RevoluteHubDrive::PrescribedAngle { angle_rad } => {
                                    let prev_ang = match prev_kind {
                                        Some(ResolvedHubCouplingKind::Revolute {
                                            drive:
                                                RevoluteHubDrive::PrescribedAngle {
                                                    angle_rad: p,
                                                },
                                            ..
                                        }) => p,
                                        _ => start_prescribed_angles
                                            .get(&(b, h_idx))
                                            .copied()
                                            .unwrap_or(0.0),
                                    };
                                    RevoluteHubDrive::PrescribedAngle {
                                        angle_rad: prev_ang + lam * (angle_rad - prev_ang),
                                    }
                                }
                                RevoluteHubDrive::Torque {
                                    torque_nm,
                                    torsional_stiffness_nm_per_rad,
                                    reference_angle_rad,
                                } => {
                                    let prev_t = match prev_kind {
                                        Some(ResolvedHubCouplingKind::Revolute {
                                            drive: RevoluteHubDrive::Torque { torque_nm: p, .. },
                                            ..
                                        }) => p,
                                        _ => 0.0,
                                    };
                                    RevoluteHubDrive::Torque {
                                        torque_nm: prev_t + lam * (torque_nm - prev_t),
                                        torsional_stiffness_nm_per_rad,
                                        reference_angle_rad,
                                    }
                                }
                            };
                            ResolvedHubCouplingKind::Revolute {
                                pivot,
                                axis,
                                drive: new_drive,
                            }
                        }
                        ResolvedHubCouplingKind::Prismatic {
                            axis,
                            drive,
                            lock_transverse,
                        } => {
                            let new_drive = match drive {
                                PrismaticHubDrive::PrescribedDisplacement { displacement_m } => {
                                    let prev_d = match prev_kind {
                                        Some(ResolvedHubCouplingKind::Prismatic {
                                            drive:
                                                PrismaticHubDrive::PrescribedDisplacement {
                                                    displacement_m: p,
                                                },
                                            ..
                                        }) => p,
                                        _ => 0.0,
                                    };
                                    PrismaticHubDrive::PrescribedDisplacement {
                                        displacement_m: prev_d + lam * (displacement_m - prev_d),
                                    }
                                }
                                PrismaticHubDrive::Force {
                                    force_n,
                                    axial_stiffness_n_per_m,
                                    reference_disp_m,
                                } => {
                                    let prev_f = match prev_kind {
                                        Some(ResolvedHubCouplingKind::Prismatic {
                                            drive: PrismaticHubDrive::Force { force_n: p, .. },
                                            ..
                                        }) => p,
                                        _ => 0.0,
                                    };
                                    PrismaticHubDrive::Force {
                                        force_n: prev_f + lam * (force_n - prev_f),
                                        axial_stiffness_n_per_m,
                                        reference_disp_m,
                                    }
                                }
                            };
                            ResolvedHubCouplingKind::Prismatic {
                                axis,
                                drive: new_drive,
                                lock_transverse,
                            }
                        }
                    };
                    ResolvedHubCoupling {
                        name: target.name.clone(),
                        node_ids: target.node_ids.clone(),
                        kind,
                    }
                })
                .collect()
        })
        .collect()
}

/// Returns `true` if `target_bcs` or `target_hubs` introduce a non-zero load, prescribed
/// displacement, prescribed rotation, or applied effort increment relative to the start of the step.
fn has_step_increment(
    prev_bcs: Option<&[ResolvedBoundary]>,
    target_bcs: &[ResolvedBoundary],
    prev_hubs: Option<&[Vec<ResolvedHubCoupling>]>,
    target_hubs: &[Vec<ResolvedHubCoupling>],
    u_step_entry: &[Vec<f64>],
    start_prescribed_angles: &HashMap<(usize, usize), f64>,
) -> bool {
    const EPS: f64 = 1e-12;
    for (b, target) in target_bcs.iter().enumerate() {
        let prev_opt = prev_bcs.and_then(|v| v.get(b));
        for (&dof, &tv) in &target.fixed_dofs {
            let pv = prev_opt
                .and_then(|p| p.fixed_dofs.get(&dof).copied())
                .unwrap_or_else(|| u_step_entry[b].get(dof).copied().unwrap_or(0.0));
            if (tv - pv).abs() > EPS {
                return true;
            }
        }
        for (nid, &tf) in target.nodal_forces.iter().enumerate() {
            let pf = prev_opt
                .and_then(|p| p.nodal_forces.get(nid).copied())
                .unwrap_or([0.0; 3]);
            if (tf[0] - pf[0]).abs() > EPS
                || (tf[1] - pf[1]).abs() > EPS
                || (tf[2] - pf[2]).abs() > EPS
            {
                return true;
            }
        }
    }
    for (b, hubs_b) in target_hubs.iter().enumerate() {
        for (h_idx, target) in hubs_b.iter().enumerate() {
            let prev_kind = prev_hubs
                .and_then(|v| v.get(b))
                .and_then(|vh| vh.get(h_idx))
                .map(|h| h.kind);
            let changed = match target.kind {
                ResolvedHubCouplingKind::Fixed => false,
                ResolvedHubCouplingKind::Revolute { drive, .. } => match drive {
                    RevoluteHubDrive::PrescribedAngle { angle_rad } => {
                        let prev_ang = match prev_kind {
                            Some(ResolvedHubCouplingKind::Revolute {
                                drive: RevoluteHubDrive::PrescribedAngle { angle_rad: p },
                                ..
                            }) => p,
                            _ => start_prescribed_angles
                                .get(&(b, h_idx))
                                .copied()
                                .unwrap_or(0.0),
                        };
                        (angle_rad - prev_ang).abs() > EPS
                    }
                    RevoluteHubDrive::Torque { torque_nm, .. } => {
                        let prev_t = match prev_kind {
                            Some(ResolvedHubCouplingKind::Revolute {
                                drive: RevoluteHubDrive::Torque { torque_nm: p, .. },
                                ..
                            }) => p,
                            _ => 0.0,
                        };
                        (torque_nm - prev_t).abs() > EPS
                    }
                },
                ResolvedHubCouplingKind::Prismatic { drive, .. } => match drive {
                    PrismaticHubDrive::PrescribedDisplacement { displacement_m } => {
                        let prev_d = match prev_kind {
                            Some(ResolvedHubCouplingKind::Prismatic {
                                drive:
                                    PrismaticHubDrive::PrescribedDisplacement { displacement_m: p },
                                ..
                            }) => p,
                            _ => 0.0,
                        };
                        (displacement_m - prev_d).abs() > EPS
                    }
                    PrismaticHubDrive::Force { force_n, .. } => {
                        let prev_f = match prev_kind {
                            Some(ResolvedHubCouplingKind::Prismatic {
                                drive: PrismaticHubDrive::Force { force_n: p, .. },
                                ..
                            }) => p,
                            _ => 0.0,
                        };
                        (force_n - prev_f).abs() > EPS
                    }
                },
            };
            if changed {
                return true;
            }
        }
    }
    false
}

/// Complete rollback snapshot of a [`MaskedContactSession`] before attempting a trial substep.
struct SessionSnapshot {
    body_states: Vec<NonlinearState>,
    stencil_state: Vec<(f64, Vector3<f64>, f64, f64)>,
    prev_pilot_values: HashMap<(usize, usize), f64>,
    prev_prescribed_angles: HashMap<(usize, usize), f64>,
}

/// Finite state returned by a single AL + Newton trial substep.
struct SubstepEvalResult {
    u_cur: Vec<Vec<f64>>,
    final_eval: GlobalEval,
    contact_points: Vec<EvaluatedContactPoint>,
    dof_sys: StepDofSystem,
    al_iters: usize,
    newton_iters: usize,
    surface_updates: usize,
    final_res_norm: f64,
    newton_converged: bool,
    pen_ok: bool,
}

enum SubstepOutcome {
    Converged(SubstepEvalResult),
    NotConverged(SubstepEvalResult),
    Failed {
        err: Option<FeaError>,
        al_iters: usize,
        newton_iters: usize,
        surface_updates: usize,
        last_res: f64,
    },
}

/// Persistent multi-body deformable contact session supporting dynamic element erosion,
/// face-splitting mesh duplication, detached-fragment masking, Kachanov $(1 - D)$ element
/// stiffness degradation (`P5-T5`), and adaptive step bisection with exact rollback (`P5-T13`).
pub(crate) struct MaskedContactSession {
    body_elems: Vec<BodyElements>,
    surfaces: Vec<DeformedSurfaceMesh>,
    body_states: Vec<NonlinearState>,
    stencils: Vec<ContactSlaveStencil>,
    initial_eps_n: Vec<f64>,
    initial_eps_t: Vec<f64>,
    char_length: f64,
    prev_pilot_values: HashMap<(usize, usize), f64>,
    prev_prescribed_angles: HashMap<(usize, usize), f64>,
    prev_resolved_bcs: Option<Vec<ResolvedBoundary>>,
    prev_resolved_hubs: Option<Vec<Vec<ResolvedHubCoupling>>>,
}

impl MaskedContactSession {
    pub(crate) fn new(
        bodies: &[ContactBody],
        pairs: &[ContactPairSpec],
        options: ContactSolverOptions,
    ) -> Result<Self, FeaError> {
        let nb = bodies.len();
        if nb == 0 {
            return Err(FeaError::EmptyMesh);
        }
        let mut body_elems = Vec::with_capacity(nb);
        let mut surfaces = Vec::with_capacity(nb);
        let mut body_states = Vec::with_capacity(nb);
        let mut char_length = f64::INFINITY;

        for body in bodies {
            let elems = BodyElements::from_mesh(&body.mesh)?;
            let gp_count = elems.gp_count();
            let surf = DeformedSurfaceMesh::from_tetra_mesh(&body.mesh);
            char_length = char_length.min(surf.char_edge_len);
            body_elems.push(elems);
            surfaces.push(surf);
            body_states.push(NonlinearState::initial(body.mesh.node_count(), gp_count));
        }
        if !char_length.is_finite() {
            char_length = 1e-3;
        }

        let stencils = build_contact_stencils(bodies, &surfaces, pairs, options)?;
        let initial_eps_n: Vec<f64> = stencils.iter().map(|s| s.eps_n).collect();
        let initial_eps_t: Vec<f64> = stencils.iter().map(|s| s.eps_t).collect();

        Ok(Self {
            body_elems,
            surfaces,
            body_states,
            stencils,
            initial_eps_n,
            initial_eps_t,
            char_length,
            prev_pilot_values: HashMap::new(),
            prev_prescribed_angles: HashMap::new(),
            prev_resolved_bcs: None,
            prev_resolved_hubs: None,
        })
    }

    pub(crate) fn body_states(&self) -> &[NonlinearState] {
        &self.body_states
    }

    pub(crate) fn surfaces(&self) -> &[DeformedSurfaceMesh] {
        &self.surfaces
    }

    /// Provides mutable access to a body's displacement vector and current deformed nodal
    /// positions so `FaceSplitting` can append duplicated crack-face nodes in lockstep.
    pub(crate) fn body_displacements_and_positions_mut(
        &mut self,
        body_idx: usize,
    ) -> (&mut Vec<f64>, &mut Vec<[f64; 3]>) {
        (
            &mut self.body_states[body_idx].displacements,
            &mut self.surfaces[body_idx].current_positions,
        )
    }

    pub(crate) fn element_volumes(&self, body_idx: usize) -> Vec<f64> {
        match &self.body_elems[body_idx] {
            BodyElements::Tet4(v) => v.iter().map(|e| e.volume).collect(),
            BodyElements::Tet10(v) => v.iter().map(|e| e.volume).collect(),
        }
    }

    pub(crate) fn into_states_and_positions(self) -> (Vec<NonlinearState>, Vec<Vec<[f64; 3]>>) {
        let positions = self
            .surfaces
            .into_iter()
            .map(|s| s.current_positions)
            .collect();
        (self.body_states, positions)
    }

    fn take_snapshot(&self) -> SessionSnapshot {
        SessionSnapshot {
            body_states: self.body_states.clone(),
            stencil_state: self
                .stencils
                .iter()
                .map(|s| (s.lambda_n, s.lambda_t, s.eps_n, s.eps_t))
                .collect(),
            prev_pilot_values: self.prev_pilot_values.clone(),
            prev_prescribed_angles: self.prev_prescribed_angles.clone(),
        }
    }

    fn restore_snapshot(&mut self, snapshot: SessionSnapshot, bodies: &[ContactBody]) {
        self.body_states = snapshot.body_states;
        for (st, (ln, lt, en, et)) in self.stencils.iter_mut().zip(snapshot.stencil_state) {
            st.lambda_n = ln;
            st.lambda_t = lt;
            st.eps_n = en;
            st.eps_t = et;
        }
        self.prev_pilot_values = snapshot.prev_pilot_values;
        self.prev_prescribed_angles = snapshot.prev_prescribed_angles;
        for (b, body) in bodies.iter().enumerate() {
            self.surfaces[b]
                .update_from_displacements(&body.mesh, &self.body_states[b].displacements);
        }
    }

    /// Rebuilds the exposed exterior contact surfaces, slave contact stencils, and (when
    /// `rebuild_body_elements[b]` is true, e.g. after `FaceSplitting` node duplication) element
    /// connectivity models, warm-starting Augmented Lagrangian multipliers on surviving surface nodes.
    pub(crate) fn rebuild_topology_from_masks(
        &mut self,
        bodies: &[ContactBody],
        pairs: &[ContactPairSpec],
        options: ContactSolverOptions,
        active_elements: &[Vec<bool>],
        rebuild_body_elements: &[bool],
    ) -> Result<(), FeaError> {
        let mut old_lambdas: HashMap<(usize, usize, usize), (f64, Vector3<f64>)> =
            HashMap::with_capacity(self.stencils.len());
        for st in &self.stencils {
            if st.lambda_n > 0.0 || st.lambda_t.norm() > 0.0 {
                old_lambdas.insert(
                    (st.pair_index, st.slave_body, st.rep_node_id),
                    (st.lambda_n, st.lambda_t),
                );
            }
        }

        let mut char_length = f64::INFINITY;
        for (b, body) in bodies.iter().enumerate() {
            if rebuild_body_elements.get(b).copied().unwrap_or(false) {
                self.body_elems[b] = BodyElements::from_mesh(&body.mesh)?;
                if let Some(ref mut prev_bcs) = self.prev_resolved_bcs {
                    if let Some(bc_b) = prev_bcs.get_mut(b) {
                        if bc_b.nodal_forces.len() < body.mesh.node_count() {
                            bc_b.nodal_forces.resize(body.mesh.node_count(), [0.0; 3]);
                        }
                    }
                }
            }
            let mut surf =
                DeformedSurfaceMesh::from_tetra_mesh_filtered(&body.mesh, Some(&active_elements[b]));
            surf.update_from_displacements(&body.mesh, &self.body_states[b].displacements);
            if surf.char_edge_len.is_finite() && surf.char_edge_len > 1e-9 {
                char_length = char_length.min(surf.char_edge_len);
            }
            self.surfaces[b] = surf;
        }
        if char_length.is_finite() {
            self.char_length = char_length;
        }

        let mut new_stencils = build_contact_stencils(bodies, &self.surfaces, pairs, options)?;
        for st in &mut new_stencils {
            if let Some(&(ln, lt)) = old_lambdas.get(&(st.pair_index, st.slave_body, st.rep_node_id))
            {
                st.lambda_n = ln;
                st.lambda_t = lt;
            }
        }
        self.initial_eps_n = new_stencils.iter().map(|s| s.eps_n).collect();
        self.initial_eps_t = new_stencils.iter().map(|s| s.eps_t).collect();
        self.stencils = new_stencils;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn solve_single_substep(
        &mut self,
        bodies: &[ContactBody],
        pairs: &[ContactPairSpec],
        resolved_bcs: &[ResolvedBoundary],
        resolved_hubs: &[Vec<ResolvedHubCoupling>],
        options: ContactSolverOptions,
        active_elements: Option<&[Vec<bool>]>,
        active_nodes: Option<&[Vec<bool>]>,
        element_stiffness_scale: Option<&[Vec<f64>]>,
        apply_rotation_predictor: bool,
        cap_nodal_step: bool,
    ) -> SubstepOutcome {
        let nb = bodies.len();
        let u_step_start: Vec<Vec<f64>> = self
            .body_states
            .iter()
            .map(|s| s.displacements.clone())
            .collect();
        let prev_gp_bodies: Vec<Vec<GaussPointState>> = self
            .body_states
            .iter()
            .map(|s| s.gauss_states.clone())
            .collect();

        let mut dof_sys = StepDofSystem::build(
            bodies,
            resolved_bcs,
            resolved_hubs,
            &self.prev_pilot_values,
            active_nodes,
            active_nodes.map(|_| u_step_start.as_slice()),
        );

        let mut u_cur = u_step_start.clone();

        if apply_rotation_predictor {
            for b in 0..nb {
                if resolved_bcs[b].fixed_dofs.is_empty() && resolved_hubs[b].len() == 1 {
                    if let ResolvedHubCouplingKind::Revolute {
                        pivot,
                        axis,
                        drive: RevoluteHubDrive::PrescribedAngle { angle_rad },
                    } = resolved_hubs[b][0].kind
                    {
                        let prev_theta = self
                            .prev_prescribed_angles
                            .get(&(b, 0))
                            .copied()
                            .unwrap_or(0.0);
                        let dtheta = angle_rad - prev_theta;
                        if dtheta.abs() > 1e-12 {
                            let piv = Vector3::from(pivot);
                            let ax = Vector3::from(axis);
                            for node in &bodies[b].mesh.nodes {
                                if active_nodes.is_some_and(|an| !an[b][node.id]) {
                                    continue;
                                }
                                let d0 = 3 * node.id;
                                let x0 = Vector3::from(node.position);
                                let x_prev = x0
                                    + Vector3::new(
                                        u_step_start[b][d0],
                                        u_step_start[b][d0 + 1],
                                        u_step_start[b][d0 + 2],
                                    );
                                let x_rot = piv + rodrigues_rotate(x_prev - piv, ax, dtheta);
                                let u_pred = x_rot - x0;
                                u_cur[b][d0] = u_pred.x;
                                u_cur[b][d0 + 1] = u_pred.y;
                                u_cur[b][d0 + 2] = u_pred.z;
                            }
                        }
                    }
                }
            }
        }

        dof_sys.enforce_kinematics_and_refresh_weights(&mut u_cur);

        for (idx, st) in self.stencils.iter_mut().enumerate() {
            st.eps_n = self.initial_eps_n[idx];
            st.eps_t = self.initial_eps_t[idx];
        }

        let mut total_newton_iters = 0usize;
        let mut surface_updates = 0usize;
        let mut al_iters = 0usize;
        let mut substep_converged = false;
        let mut last_newton_converged = false;
        let mut last_pen_ok = true;
        let mut final_res_norm = 0.0;
        let mut prev_al_pen = f64::INFINITY;
        let mut prev_rel_lam_change = f64::INFINITY;

        let mut ref_force_scale = 0.0_f64;
        for b in 0..nb {
            for f in &dof_sys.body_ext_forces[b] {
                ref_force_scale += f[0] * f[0] + f[1] * f[1] + f[2] * f[2];
            }
        }
        for p in &dof_sys.pilots {
            ref_force_scale += p.applied_effort * p.applied_effort;
        }
        ref_force_scale = ref_force_scale.sqrt();

        let mut last_eval_opt: Option<GlobalEval> = None;
        let mut last_contact_points: Vec<EvaluatedContactPoint> = Vec::new();
        let pilot_set: HashSet<usize> = if cap_nodal_step {
            dof_sys.pilots.iter().map(|p| p.reduced_idx).collect()
        } else {
            HashSet::new()
        };
        let line_search_alphas: &[f64] = if cap_nodal_step {
            &[1.0_f64, 0.5, 0.25, 0.125, 0.05, 0.01, 0.002]
        } else {
            &[1.0_f64, 0.5, 0.25, 0.125, 0.05]
        };

        for al_iter in 0..options.max_al_iterations.max(1) {
            al_iters = al_iter + 1;
            let mut newton_converged = false;

            for n_iter in 0..options.max_newton_iterations.max(1) {
                total_newton_iters += 1;

                for b in 0..nb {
                    self.surfaces[b].update_from_displacements(&bodies[b].mesh, &u_cur[b]);
                }
                surface_updates += 1;

                let zero_gap_touch = al_iter == 0 && n_iter == 0;
                let contact_pts = evaluate_contact_points(
                    &self.stencils,
                    &self.surfaces,
                    &u_cur,
                    &u_step_start,
                    zero_gap_touch,
                );

                let eval = evaluate_multibody_system(
                    bodies,
                    &self.body_elems,
                    &dof_sys,
                    &u_cur,
                    &prev_gp_bodies,
                    &self.stencils,
                    &contact_pts,
                    options,
                    true,
                    active_elements,
                    element_stiffness_scale,
                );

                let res_norm = vec_norm(&eval.residual_red);
                if !res_norm.is_finite() {
                    return SubstepOutcome::Failed {
                        err: None,
                        al_iters,
                        newton_iters: total_newton_iters,
                        surface_updates,
                        last_res: res_norm,
                    };
                }
                let mut internal_scale = 0.0_f64;
                for f_int in &eval.f_int_bodies {
                    for &v in f_int {
                        internal_scale += v * v;
                    }
                }
                let force_scale = ref_force_scale.max(internal_scale.sqrt()).max(1.0);
                let tol = (options.newton_rel_tol * force_scale).max(options.newton_abs_tol);
                final_res_norm = res_norm;

                if res_norm <= tol {
                    newton_converged = true;
                    break;
                }

                let k_red = eval.k_red.as_ref().expect("tangent assembled");
                let mut dq = match solve_contact_tangent(
                    k_red,
                    &eval.residual_red,
                    &dof_sys.pilots,
                    options.linear_solver,
                ) {
                    Ok(sol) => sol,
                    Err(err) => {
                        return SubstepOutcome::Failed {
                            err: Some(err),
                            al_iters,
                            newton_iters: total_newton_iters,
                            surface_updates,
                            last_res: final_res_norm,
                        };
                    }
                };
                if dq.iter().any(|v| !v.is_finite()) {
                    return SubstepOutcome::Failed {
                        err: None,
                        al_iters,
                        newton_iters: total_newton_iters,
                        surface_updates,
                        last_res: final_res_norm,
                    };
                }

                // Cap large pilot excursions (and free nodal steps in fracture mode)
                let mut dq_scale = 1.0_f64;
                for pilot in &dof_sys.pilots {
                    let step_limit = match pilot.kind {
                        PilotKind::Revolute => 0.05, // ~2.9 deg max per Newton iteration
                        PilotKind::Prismatic => (2.0 * self.char_length).max(1e-3),
                    };
                    let dval = dq[pilot.reduced_idx].abs();
                    if dval > step_limit {
                        dq_scale = dq_scale.min(step_limit / dval);
                    }
                }
                if cap_nodal_step {
                    let nodal_step_limit = (0.35 * self.char_length).max(2e-4);
                    for (ri, &dv) in dq.iter().enumerate() {
                        if !pilot_set.contains(&ri) && dv.abs() > nodal_step_limit {
                            dq_scale = dq_scale.min(nodal_step_limit / dv.abs());
                        }
                    }
                }
                if dq_scale < 1.0 {
                    for v in &mut dq {
                        *v *= dq_scale;
                    }
                }

                let u_base = u_cur.clone();
                let pilots_base: Vec<f64> = dof_sys.pilots.iter().map(|p| p.value).collect();
                let mut best_alpha = if cap_nodal_step { 0.0_f64 } else { 1.0_f64 };
                let mut best_res = if cap_nodal_step {
                    res_norm
                } else {
                    f64::INFINITY
                };
                let mut accepted = false;

                for &alpha in line_search_alphas {
                    dof_sys.apply_reduced_increment(&u_base, &pilots_base, &dq, alpha, &mut u_cur);
                    for b in 0..nb {
                        self.surfaces[b].update_from_displacements(&bodies[b].mesh, &u_cur[b]);
                    }
                    let trial_pts = evaluate_contact_points(
                        &self.stencils,
                        &self.surfaces,
                        &u_cur,
                        &u_step_start,
                        false,
                    );
                    let trial_eval = evaluate_multibody_system(
                        bodies,
                        &self.body_elems,
                        &dof_sys,
                        &u_cur,
                        &prev_gp_bodies,
                        &self.stencils,
                        &trial_pts,
                        options,
                        false,
                        active_elements,
                        element_stiffness_scale,
                    );
                    let trial_res = vec_norm(&trial_eval.residual_red);
                    if trial_res.is_finite() && trial_res < best_res {
                        best_res = trial_res;
                        best_alpha = alpha;
                    }
                    if trial_res.is_finite()
                        && (trial_res <= (1.0 - 1e-4 * alpha) * res_norm || trial_res <= tol)
                    {
                        accepted = true;
                        final_res_norm = trial_res;
                        break;
                    }
                }

                if !accepted {
                    if !best_res.is_finite() {
                        return SubstepOutcome::Failed {
                            err: None,
                            al_iters,
                            newton_iters: total_newton_iters,
                            surface_updates,
                            last_res: final_res_norm,
                        };
                    }
                    if best_alpha > 0.0 {
                        dof_sys.apply_reduced_increment(
                            &u_base,
                            &pilots_base,
                            &dq,
                            best_alpha,
                            &mut u_cur,
                        );
                        final_res_norm = best_res;
                    } else {
                        u_cur = u_base;
                        for (p, &v) in dof_sys.pilots.iter_mut().zip(pilots_base.iter()) {
                            p.value = v;
                        }
                        break;
                    }
                }
            }

            last_newton_converged = newton_converged;

            for b in 0..nb {
                self.surfaces[b].update_from_displacements(&bodies[b].mesh, &u_cur[b]);
            }
            surface_updates += 1;
            last_contact_points = evaluate_contact_points(
                &self.stencils,
                &self.surfaces,
                &u_cur,
                &u_step_start,
                false,
            );
            last_eval_opt = Some(evaluate_multibody_system(
                bodies,
                &self.body_elems,
                &dof_sys,
                &u_cur,
                &prev_gp_bodies,
                &self.stencils,
                &last_contact_points,
                options,
                false,
                active_elements,
                element_stiffness_scale,
            ));

            let pair_pens = measure_pair_penetrations(pairs, &self.surfaces);
            let worst_pen = pair_pens.iter().copied().fold(0.0_f64, f64::max);

            let mut active_set_visited = vec![false; self.stencils.len()];
            let mut lam_diff_sq = 0.0_f64;
            let mut lam_norm_sq = 0.0_f64;
            let mut any_active = false;

            for cp in &last_contact_points {
                let st = &mut self.stencils[cp.stencil_idx];
                active_set_visited[cp.stencil_idx] = true;

                let old_ln = st.lambda_n;
                let old_lt = st.lambda_t;

                let new_ln = (old_ln - st.eps_n * cp.signed_gap).max(0.0);
                let new_lt = if new_ln > 0.0 && st.friction_coeff > 1e-12 {
                    let lam_t_proj = old_lt - cp.normal * old_lt.dot(&cp.normal);
                    let trial = lam_t_proj - cp.delta_g_t * st.eps_t;
                    let tau_tr = trial.norm();
                    let tau_max = st.friction_coeff * new_ln;
                    if tau_tr <= tau_max {
                        trial
                    } else if tau_tr > 1e-18 {
                        trial * (tau_max / tau_tr)
                    } else {
                        Vector3::zeros()
                    }
                } else {
                    Vector3::zeros()
                };

                if new_ln > 0.0 {
                    any_active = true;
                }
                let d_ln = new_ln - old_ln;
                let d_lt = new_lt - old_lt;
                lam_diff_sq += d_ln * d_ln + d_lt.norm_squared();
                lam_norm_sq += new_ln * new_ln + new_lt.norm_squared();

                st.lambda_n = new_ln;
                st.lambda_t = new_lt;
            }

            for (s_idx, st) in self.stencils.iter_mut().enumerate() {
                if !active_set_visited[s_idx] && (st.lambda_n > 0.0 || st.lambda_t.norm() > 0.0) {
                    lam_diff_sq += st.lambda_n * st.lambda_n + st.lambda_t.norm_squared();
                    st.lambda_n = 0.0;
                    st.lambda_t = Vector3::zeros();
                }
            }

            let rel_lam_change = if lam_norm_sq > 1e-12 {
                (lam_diff_sq / lam_norm_sq).sqrt()
            } else {
                lam_diff_sq.sqrt()
            };

            let pen_ok = worst_pen <= options.max_penetration_tol;
            let lam_ok = !any_active || rel_lam_change <= options.al_multiplier_tol;
            last_pen_ok = pen_ok;
            if newton_converged && pen_ok && lam_ok && (al_iter >= 1 || !any_active) {
                substep_converged = true;
                break;
            }

            let pen_stalled =
                worst_pen > options.max_penetration_tol && worst_pen > 0.4 * prev_al_pen;
            let lam_stalled = any_active
                && al_iter >= 2
                && rel_lam_change > options.al_multiplier_tol
                && rel_lam_change > 0.65 * prev_rel_lam_change;
            if pen_stalled || lam_stalled {
                let mult = if pen_stalled { 3.0 } else { 2.0 };
                for (idx, st) in self.stencils.iter_mut().enumerate() {
                    let cap_n = self.initial_eps_n[idx] * options.max_penalty_escalation;
                    let cap_t = self.initial_eps_t[idx] * options.max_penalty_escalation;
                    st.eps_n = (st.eps_n * mult).min(cap_n);
                    st.eps_t = (st.eps_t * mult).min(cap_t);
                }
            }
            prev_al_pen = worst_pen;
            prev_rel_lam_change = rel_lam_change;
        }

        let eval_res = SubstepEvalResult {
            u_cur,
            final_eval: last_eval_opt.expect("at least one evaluation executed"),
            contact_points: last_contact_points,
            dof_sys,
            al_iters,
            newton_iters: total_newton_iters,
            surface_updates,
            final_res_norm,
            newton_converged: last_newton_converged,
            pen_ok: last_pen_ok,
        };

        if substep_converged {
            SubstepOutcome::Converged(eval_res)
        } else {
            SubstepOutcome::NotConverged(eval_res)
        }
    }

    fn commit_substep(
        &mut self,
        bodies: &[ContactBody],
        resolved_hubs: &[Vec<ResolvedHubCoupling>],
        res: &SubstepEvalResult,
    ) {
        for (b, body) in bodies.iter().enumerate() {
            self.body_states[b].displacements = res.u_cur[b].clone();
            self.body_states[b].gauss_states = res.final_eval.new_gp_bodies[b].clone();
            self.body_states[b].gauss_rotations = res.final_eval.new_rot_bodies[b].clone();
            self.surfaces[b]
                .update_from_displacements(&body.mesh, &self.body_states[b].displacements);
        }
        for pilot in &res.dof_sys.pilots {
            self.prev_pilot_values
                .insert((pilot.body_idx, pilot.hub_idx), pilot.value);
        }
        for (b, hubs_b) in resolved_hubs.iter().enumerate() {
            for (h_idx, hub) in hubs_b.iter().enumerate() {
                if let ResolvedHubCouplingKind::Revolute {
                    drive: RevoluteHubDrive::PrescribedAngle { angle_rad },
                    ..
                } = hub.kind
                {
                    self.prev_prescribed_angles.insert((b, h_idx), angle_rad);
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn solve_step(
        &mut self,
        step_idx: usize,
        bodies: &[ContactBody],
        pairs: &[ContactPairSpec],
        step_spec: &ContactStepSpec,
        options: ContactSolverOptions,
        active_elements: &[Vec<bool>],
        element_stiffness_scale: &[Vec<f64>],
        apply_rotation_predictor: bool,
    ) -> Result<ContactStepReport, FeaError> {
        self.solve_step_internal(
            step_idx,
            bodies,
            pairs,
            step_spec,
            options,
            Some(active_elements),
            Some(element_stiffness_scale),
            apply_rotation_predictor,
            true,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn solve_step_internal(
        &mut self,
        step_idx: usize,
        bodies: &[ContactBody],
        pairs: &[ContactPairSpec],
        step_spec: &ContactStepSpec,
        options: ContactSolverOptions,
        active_elements: Option<&[Vec<bool>]>,
        element_stiffness_scale: Option<&[Vec<f64>]>,
        apply_rotation_predictor: bool,
        cap_nodal_step: bool,
    ) -> Result<ContactStepReport, FeaError> {
        let nb = bodies.len();
        let mut target_bcs = Vec::with_capacity(nb);
        let mut target_hubs = Vec::with_capacity(nb);
        for (b, body) in bodies.iter().enumerate() {
            let bc = step_spec
                .body_boundaries
                .get(&b)
                .unwrap_or(&body.boundary);
            target_bcs.push(bc.resolve_allowing_unconstrained(&body.mesh)?);

            let hubs = step_spec
                .body_hub_couplings
                .get(&b)
                .unwrap_or(&body.hub_couplings);
            let mut rh = Vec::with_capacity(hubs.len());
            for h in hubs {
                rh.push(h.resolve(&body.mesh)?);
            }
            target_hubs.push(rh);
        }

        let active_nodes_opt =
            active_elements.map(|ae| compute_active_nodes_from_elements(bodies, ae));
        let u_step_entry: Vec<Vec<f64>> = self
            .body_states
            .iter()
            .map(|s| s.displacements.clone())
            .collect();
        let start_prescribed_angles = self.prev_prescribed_angles.clone();
        let prev_bcs_owned = self.prev_resolved_bcs.clone();
        let prev_hubs_owned = self.prev_resolved_hubs.clone();

        let has_increment = has_step_increment(
            prev_bcs_owned.as_deref(),
            &target_bcs,
            prev_hubs_owned.as_deref(),
            &target_hubs,
            &u_step_entry,
            &start_prescribed_angles,
        );

        let mut total_al_iters = 0usize;
        let mut total_newton_iters = 0usize;
        let mut total_surface_updates = 0usize;
        let mut bisections = 0usize;
        let mut substeps_completed = 0usize;
        let mut step_converged = false;
        let mut final_res_opt: Option<SubstepEvalResult> = None;

        if !has_increment {
            // Re-equilibration at fixed load (e.g. after element erosion or face splitting):
            // preserve finite warm-started contact equilibria immediately; only roll back and
            // retry with damped/zeroed AL multipliers on hard numerical failures (linear solver
            // error or NaN/Inf).
            let damping_schedule: &[f64] = if options.max_bisections > 0 {
                &[1.0, 0.25, 0.0]
            } else {
                &[1.0]
            };
            let base_snapshot = self.take_snapshot();
            let mut last_err: Option<(Option<FeaError>, f64)> = None;

            for (attempt, &lam_scale) in damping_schedule.iter().enumerate() {
                if attempt > 0 {
                    self.restore_snapshot(
                        SessionSnapshot {
                            body_states: base_snapshot.body_states.clone(),
                            stencil_state: base_snapshot
                                .stencil_state
                                .iter()
                                .map(|&(ln, lt, en, et)| (ln * lam_scale, lt * lam_scale, en, et))
                                .collect(),
                            prev_pilot_values: base_snapshot.prev_pilot_values.clone(),
                            prev_prescribed_angles: base_snapshot.prev_prescribed_angles.clone(),
                        },
                        bodies,
                    );
                    bisections += 1;
                }
                match self.solve_single_substep(
                    bodies,
                    pairs,
                    &target_bcs,
                    &target_hubs,
                    options,
                    active_elements,
                    active_nodes_opt.as_deref(),
                    element_stiffness_scale,
                    apply_rotation_predictor,
                    cap_nodal_step,
                ) {
                    SubstepOutcome::Converged(res) => {
                        total_al_iters += res.al_iters;
                        total_newton_iters += res.newton_iters;
                        total_surface_updates += res.surface_updates;
                        self.commit_substep(bodies, &target_hubs, &res);
                        substeps_completed = 1;
                        step_converged = true;
                        final_res_opt = Some(res);
                        break;
                    }
                    SubstepOutcome::NotConverged(res) => {
                        total_al_iters += res.al_iters;
                        total_newton_iters += res.newton_iters;
                        total_surface_updates += res.surface_updates;
                        self.commit_substep(bodies, &target_hubs, &res);
                        substeps_completed = 1;
                        step_converged = false;
                        final_res_opt = Some(res);
                        break;
                    }
                    SubstepOutcome::Failed {
                        err,
                        al_iters,
                        newton_iters,
                        surface_updates,
                        last_res,
                    } => {
                        total_al_iters += al_iters;
                        total_newton_iters += newton_iters;
                        total_surface_updates += surface_updates;
                        last_err = Some((err, last_res));
                    }
                }
            }

            if final_res_opt.is_none() {
                self.restore_snapshot(base_snapshot, bodies);
                let (err, last_res) = last_err.unwrap_or((None, f64::NAN));
                return Err(err.unwrap_or(FeaError::SolverConvergenceFailed {
                    iterations: total_newton_iters,
                    residual: last_res,
                    tolerance: options.newton_rel_tol,
                }));
            }
        } else {
            let mut lambda_curr = 0.0_f64;
            let mut d_lambda = 1.0_f64;
            let min_dl = options.min_substep_fraction.clamp(1e-6, 0.5);
            type AlMultiplierSnapshot = Vec<(f64, Vector3<f64>, f64, f64)>;
            let mut fallback_full_step: Option<(SubstepEvalResult, AlMultiplierSnapshot)> = None;
            let mut prev_stalled_res: Option<f64> = None;

            while lambda_curr < 1.0 - 1e-12 {
                let lambda_next = (lambda_curr + d_lambda).min(1.0);
                let is_final_target = lambda_next >= 1.0 - 1e-12;
                let can_bisect_more = options.max_bisections > 0
                    && bisections < options.max_bisections
                    && (d_lambda * 0.5) >= min_dl;

                let sub_bcs = if is_final_target {
                    target_bcs.clone()
                } else {
                    interpolate_resolved_boundaries(
                        prev_bcs_owned.as_deref(),
                        &target_bcs,
                        &u_step_entry,
                        lambda_next,
                    )
                };
                let sub_hubs = if is_final_target {
                    target_hubs.clone()
                } else {
                    interpolate_resolved_hubs(
                        prev_hubs_owned.as_deref(),
                        &target_hubs,
                        &start_prescribed_angles,
                        lambda_next,
                    )
                };

                let snapshot = self.take_snapshot();
                match self.solve_single_substep(
                    bodies,
                    pairs,
                    &sub_bcs,
                    &sub_hubs,
                    options,
                    active_elements,
                    active_nodes_opt.as_deref(),
                    element_stiffness_scale,
                    apply_rotation_predictor,
                    cap_nodal_step,
                ) {
                    SubstepOutcome::Converged(res) => {
                        total_al_iters += res.al_iters;
                        total_newton_iters += res.newton_iters;
                        total_surface_updates += res.surface_updates;
                        let al_used = res.al_iters;
                        let newton_used = res.newton_iters;
                        self.commit_substep(bodies, &sub_hubs, &res);
                        substeps_completed += 1;
                        lambda_curr = lambda_next;
                        step_converged = true;
                        final_res_opt = Some(res);

                        if lambda_curr < 1.0 - 1e-12 {
                            let rem = 1.0 - lambda_curr;
                            if al_used <= (options.max_al_iterations / 2).max(2)
                                && newton_used <= options.max_newton_iterations.max(1) * 2
                            {
                                d_lambda = (d_lambda * 1.5).min(rem);
                            } else {
                                d_lambda = d_lambda.min(rem);
                            }
                        }
                    }
                    SubstepOutcome::NotConverged(res) => {
                        total_al_iters += res.al_iters;
                        total_newton_iters += res.newton_iters;
                        total_surface_updates += res.surface_updates;

                        // In staggered fracture mode with tight outer AL budgets:
                        // - Accept without wasteful bisection when inner Newton equilibrium and
                        //   contact penetration are already satisfied (only AL multiplier delta
                        //   remained), or when actively softened crack-tip elements already
                        //   satisfy the contact penetration constraint (`pen_ok`).
                        // - Also stop bisecting if halving d_lambda stopped reducing the residual.
                        let has_softened_elements = element_stiffness_scale.is_some_and(|scales| {
                            scales
                                .iter()
                                .any(|body_s| body_s.iter().any(|&k| k < 0.99))
                        });
                        let accept_soft_al = cap_nodal_step
                            && is_final_target
                            && res.pen_ok
                            && (res.newton_converged || has_softened_elements);
                        let fracture_bisection_stalled = cap_nodal_step
                            && lambda_curr <= 1e-12
                            && bisections >= 1
                            && res.pen_ok
                            && prev_stalled_res
                                .is_some_and(|prev_r| res.final_res_norm >= 0.7 * prev_r);

                        if accept_soft_al {
                            self.commit_substep(bodies, &sub_hubs, &res);
                            substeps_completed += 1;
                            lambda_curr = 1.0;
                            step_converged = false;
                            final_res_opt = Some(res);
                        } else if fracture_bisection_stalled {
                            if let Some((fb_res, fb_stencils)) = fallback_full_step.take() {
                                self.restore_snapshot(snapshot, bodies);
                                for (st, (ln, lt, en, et)) in
                                    self.stencils.iter_mut().zip(fb_stencils)
                                {
                                    st.lambda_n = ln;
                                    st.lambda_t = lt;
                                    st.eps_n = en;
                                    st.eps_t = et;
                                }
                                self.commit_substep(bodies, &target_hubs, &fb_res);
                                substeps_completed = 1;
                                lambda_curr = 1.0;
                                step_converged = false;
                                final_res_opt = Some(fb_res);
                            } else {
                                self.commit_substep(bodies, &sub_hubs, &res);
                                substeps_completed += 1;
                                lambda_curr = 1.0;
                                step_converged = false;
                                final_res_opt = Some(res);
                            }
                        } else if can_bisect_more {
                            if lambda_curr <= 1e-12 {
                                prev_stalled_res = Some(res.final_res_norm);
                            }
                            if is_final_target
                                && lambda_curr <= 1e-12
                                && fallback_full_step.is_none()
                            {
                                let st_state = self
                                    .stencils
                                    .iter()
                                    .map(|s| (s.lambda_n, s.lambda_t, s.eps_n, s.eps_t))
                                    .collect();
                                fallback_full_step = Some((res, st_state));
                            }
                            self.restore_snapshot(snapshot, bodies);
                            bisections += 1;
                            d_lambda *= 0.5;
                        } else if is_final_target {
                            // Bisection budget exhausted at target load: commit best finite state
                            // with `converged = false` (preserves compatibility with tight budgets).
                            self.commit_substep(bodies, &sub_hubs, &res);
                            substeps_completed += 1;
                            lambda_curr = 1.0;
                            step_converged = false;
                            final_res_opt = Some(res);
                        } else if lambda_curr > 1e-12 {
                            // Partial substeps succeeded; advance remaining increment directly to
                            // target load as final best-effort attempt (no further bisection).
                            self.restore_snapshot(snapshot, bodies);
                            d_lambda = 1.0 - lambda_curr;
                        } else if let Some((fb_res, fb_stencils)) = fallback_full_step.take() {
                            // Even smallest bisection did not satisfy tight AL tolerance: restore
                            // finite full-step state with `converged = false`.
                            self.restore_snapshot(snapshot, bodies);
                            for (st, (ln, lt, en, et)) in
                                self.stencils.iter_mut().zip(fb_stencils)
                            {
                                st.lambda_n = ln;
                                st.lambda_t = lt;
                                st.eps_n = en;
                                st.eps_t = et;
                            }
                            self.commit_substep(bodies, &target_hubs, &fb_res);
                            substeps_completed = 1;
                            lambda_curr = 1.0;
                            step_converged = false;
                            final_res_opt = Some(fb_res);
                        } else {
                            let last_res = res.final_res_norm;
                            self.restore_snapshot(snapshot, bodies);
                            return Err(FeaError::SolverConvergenceFailed {
                                iterations: total_newton_iters,
                                residual: last_res,
                                tolerance: options.newton_rel_tol,
                            });
                        }
                    }
                    SubstepOutcome::Failed {
                        err,
                        al_iters,
                        newton_iters,
                        surface_updates,
                        last_res,
                    } => {
                        total_al_iters += al_iters;
                        total_newton_iters += newton_iters;
                        total_surface_updates += surface_updates;
                        self.restore_snapshot(snapshot, bodies);

                        if can_bisect_more {
                            bisections += 1;
                            d_lambda *= 0.5;
                        } else if let Some((fb_res, fb_stencils)) = fallback_full_step.take() {
                            for (st, (ln, lt, en, et)) in
                                self.stencils.iter_mut().zip(fb_stencils)
                            {
                                st.lambda_n = ln;
                                st.lambda_t = lt;
                                st.eps_n = en;
                                st.eps_t = et;
                            }
                            self.commit_substep(bodies, &target_hubs, &fb_res);
                            substeps_completed = 1;
                            lambda_curr = 1.0;
                            step_converged = false;
                            final_res_opt = Some(fb_res);
                        } else {
                            return Err(err.unwrap_or(FeaError::SolverConvergenceFailed {
                                iterations: total_newton_iters,
                                residual: last_res,
                                tolerance: options.newton_rel_tol,
                            }));
                        }
                    }
                }
            }
        }

        self.prev_resolved_bcs = Some(target_bcs);
        self.prev_resolved_hubs = Some(target_hubs.clone());

        let final_substep = final_res_opt.expect("at least one substep executed");
        let final_eval = &final_substep.final_eval;
        let last_contact_points = &final_substep.contact_points;
        let dof_sys = &final_substep.dof_sys;
        let final_res_norm = final_substep.final_res_norm;

        let pair_pens = measure_pair_penetrations(pairs, &self.surfaces);
        let worst_pen = pair_pens.iter().copied().fold(0.0_f64, f64::max);

        let mut pair_summaries = Vec::with_capacity(pairs.len());
        for (p_idx, pair) in pairs.iter().enumerate() {
            let primary_slave = match pair {
                ContactPairSpec::DeformablePair { body_a, .. } => *body_a,
                ContactPairSpec::DeformableToRigid { body_idx, .. } => *body_idx,
            };
            let mut active_points = Vec::new();
            let mut contact_area = 0.0;
            let mut peak_pn = 0.0_f64;
            let mut peak_tt = 0.0_f64;
            let mut total_fn_scalar = 0.0;
            let mut res_fn_vec = Vector3::zeros();
            let mut res_ft_vec = Vector3::zeros();
            let mut slipping_count = 0usize;

            for cp in last_contact_points {
                let st = &self.stencils[cp.stencil_idx];
                if st.pair_index != p_idx || !cp.active || cp.t_n <= 0.0 {
                    continue;
                }
                let tt_mag = cp.t_t.norm();
                let sign_for_primary = if st.slave_body == primary_slave {
                    1.0
                } else {
                    -1.0
                };
                contact_area += cp.area;
                peak_pn = peak_pn.max(cp.t_n);
                peak_tt = peak_tt.max(tt_mag);
                total_fn_scalar += cp.t_n * cp.area;
                res_fn_vec += cp.normal * (sign_for_primary * cp.t_n * cp.area);
                res_ft_vec += cp.t_t * (sign_for_primary * cp.area);
                if cp.is_slipping {
                    slipping_count += 1;
                }
                active_points.push(ContactPointResult {
                    slave_body: st.slave_body,
                    slave_node_id: st.rep_node_id,
                    position: cp.position,
                    normal: [cp.normal.x, cp.normal.y, cp.normal.z],
                    signed_gap_m: cp.signed_gap,
                    area_m2: cp.area,
                    normal_pressure_pa: cp.t_n,
                    tangential_traction_pa: [cp.t_t.x, cp.t_t.y, cp.t_t.z],
                    tangential_stress_pa: tt_mag,
                    is_slipping: cp.is_slipping,
                });
            }

            pair_summaries.push(ContactPatchSummary {
                pair_index: p_idx,
                active_point_count: active_points.len(),
                contact_area_m2: contact_area,
                peak_normal_pressure_pa: peak_pn,
                peak_tangential_stress_pa: peak_tt,
                total_normal_force_n: total_fn_scalar,
                resultant_normal_force_vec_n: [res_fn_vec.x, res_fn_vec.y, res_fn_vec.z],
                resultant_tangential_force_vec_n: [res_ft_vec.x, res_ft_vec.y, res_ft_vec.z],
                total_tangential_force_n: res_ft_vec.norm(),
                max_penetration_m: pair_pens[p_idx],
                slipping_point_count: slipping_count,
                active_points,
            });
        }

        let mut hub_results = Vec::new();
        for (b, hubs_b) in target_hubs.iter().enumerate() {
            for (h_idx, hub) in hubs_b.iter().enumerate() {
                let mut f_react_sum = Vector3::zeros();
                let mut t_react_sum = Vector3::zeros();
                let pivot = match hub.kind {
                    ResolvedHubCouplingKind::Revolute { pivot, .. } => Vector3::from(pivot),
                    _ => Vector3::zeros(),
                };
                for &nid in &hub.node_ids {
                    if active_nodes_opt
                        .as_ref()
                        .is_some_and(|an| !an[b].get(nid).copied().unwrap_or(true))
                    {
                        continue;
                    }
                    let d0 = 3 * nid;
                    let f_ext = dof_sys.body_ext_forces[b][nid];
                    let fr = Vector3::new(
                        final_eval.f_int_bodies[b][d0]
                            - f_ext[0]
                            - final_eval.f_contact_bodies[b][d0],
                        final_eval.f_int_bodies[b][d0 + 1]
                            - f_ext[1]
                            - final_eval.f_contact_bodies[b][d0 + 1],
                        final_eval.f_int_bodies[b][d0 + 2]
                            - f_ext[2]
                            - final_eval.f_contact_bodies[b][d0 + 2],
                    );
                    let pos_cur = Vector3::from(self.surfaces[b].current_positions[nid]);
                    f_react_sum += fr;
                    t_react_sum += (pos_cur - pivot).cross(&fr);
                }
                let (coordinate, reaction_effort) = match hub.kind {
                    ResolvedHubCouplingKind::Revolute { axis, drive, .. } => {
                        let a = Vector3::from(axis);
                        let coord = match drive {
                            RevoluteHubDrive::PrescribedAngle { angle_rad } => angle_rad,
                            RevoluteHubDrive::Torque { .. } => self
                                .prev_pilot_values
                                .get(&(b, h_idx))
                                .copied()
                                .unwrap_or(0.0),
                        };
                        (coord, t_react_sum.dot(&a))
                    }
                    ResolvedHubCouplingKind::Prismatic { axis, drive, .. } => {
                        let a = Vector3::from(axis);
                        let coord = match drive {
                            PrismaticHubDrive::PrescribedDisplacement { displacement_m } => {
                                displacement_m
                            }
                            PrismaticHubDrive::Force { .. } => self
                                .prev_pilot_values
                                .get(&(b, h_idx))
                                .copied()
                                .unwrap_or(0.0),
                        };
                        (coord, f_react_sum.dot(&a))
                    }
                    ResolvedHubCouplingKind::Fixed => (0.0, f_react_sum.norm()),
                };
                hub_results.push(HubCouplingResult {
                    body_index: b,
                    hub_index: h_idx,
                    name: hub.name.clone(),
                    coordinate,
                    reaction_effort,
                    reaction_force_n: [f_react_sum.x, f_react_sum.y, f_react_sum.z],
                    reaction_torque_nm: [t_react_sum.x, t_react_sum.y, t_react_sum.z],
                });
            }
        }

        let mut body_max_vm = Vec::with_capacity(nb);
        let mut body_max_ep = Vec::with_capacity(nb);
        let mut body_max_disp = Vec::with_capacity(nb);
        for b in 0..nb {
            let mut max_vm = 0.0_f64;
            let mut max_ep = 0.0_f64;
            let gp_per_elem = if bodies[b].mesh.is_tet10() { 4 } else { 1 };
            let n_elems = if bodies[b].mesh.is_tet10() {
                bodies[b].mesh.tet10_elements.len()
            } else {
                bodies[b].mesh.elements.len()
            };
            for e_idx in 0..n_elems {
                if active_elements.is_some_and(|ae| !ae[b][e_idx]) {
                    continue;
                }
                let base = e_idx * gp_per_elem;
                for q in 0..gp_per_elem {
                    let gp = &self.body_states[b].gauss_states[base + q];
                    let s = gp.stress;
                    let vm = crate::elasticity::von_mises_from_voigt(&nalgebra::Vector6::new(
                        s[0], s[1], s[2], s[3], s[4], s[5],
                    ));
                    max_vm = max_vm.max(vm);
                    max_ep = max_ep.max(gp.equivalent_plastic_strain);
                }
            }
            let mut max_u = 0.0_f64;
            for nid in 0..bodies[b].mesh.node_count() {
                if active_nodes_opt
                    .as_ref()
                    .is_some_and(|an| !an[b].get(nid).copied().unwrap_or(true))
                {
                    continue;
                }
                let d0 = 3 * nid;
                let u = &self.body_states[b].displacements;
                let mag = (u[d0] * u[d0] + u[d0 + 1] * u[d0 + 1] + u[d0 + 2] * u[d0 + 2]).sqrt();
                max_u = max_u.max(mag);
            }
            body_max_vm.push(max_vm);
            body_max_ep.push(max_ep);
            body_max_disp.push(max_u);
        }

        Ok(ContactStepReport {
            step_index: step_idx,
            label: step_spec.label.clone(),
            converged: step_converged,
            al_iterations: total_al_iters,
            total_newton_iterations: total_newton_iters,
            deformed_surface_updates: total_surface_updates,
            final_residual_norm: final_res_norm,
            max_penetration_m: worst_pen,
            pairs: pair_summaries,
            hubs: hub_results,
            bisections,
            substeps_completed,
            body_max_von_mises_pa: body_max_vm,
            body_max_equiv_plastic_strain: body_max_ep,
            body_max_displacement_m: body_max_disp,
        })
    }
}

