//! Local Fracture Mechanics, Topology Evolution (`ElementErosion` & `FaceSplitting`),
//! Connected-Component Fragment Extraction, and Exact Momentum/Inertia Conservation (`P5-T5`).
//!
//! Couples Co-Rotated FEM + $J_2$ elastoplasticity (`P5-T3`) and multi-body deformable contact
//! (`P5-T4`) with local continuum damage $D \in [0, 1]$ (`crate::damage`):
//! - **Staggered crack-propagation substeps**: within each load step, converged stress/plastic
//!   states drive irreversible damage evolution; newly failed elements/faces update topology and
//!   re-equilibrate until the crack arrests or severs a connected fragment.
//! - **Two topology evolution modes**:
//!   - [`FractureTopologyMode::ElementErosion`]: removes elements reaching $D \ge D_{\text{erode}}$
//!     from stiffness assembly and contact surfaces.
//!   - [`FractureTopologyMode::FaceSplitting`]: severs interior element faces aligned with the
//!     Mode-I principal crack normal $\mathbf{n}_1$ and duplicates shared nodes across the crack
//!     front with **zero deleted elements and zero mass loss**.
//! - **Automatic fragment detachment & exact conservation**: detects face-connected components
//!   detached from anchored supports/hubs, builds watertight outward-oriented surface meshes
//!   (`world_surface_mesh` & COM-centered `local_surface_mesh`), and integrates exact simplex
//!   mass, COM, $3\times 3$ inertia tensor, linear velocity $\mathbf{v}$, and angular velocity
//!   $\boldsymbol{\omega}$ conserving total mass, linear momentum, and angular momentum to
//!   machine precision ($\le 10^{-12} \ll 10^{-6}$).

use std::collections::{HashMap, HashSet, VecDeque};
use std::time::Instant;

use mcp_cad_core::topology::{TriangleMesh, Vertex};
use nalgebra::{Matrix3, Vector3, Vector6};
use serde::{Deserialize, Serialize};

use crate::boundary::{
    BoundaryConditions, DirichletConstraint, HubCoupling, HubNodeSelector, ResolvedBoundary,
};
use crate::contact::{
    solve_spd_with_fallback, ContactBody, ContactPairSpec, ContactSolverOptions, ContactStepReport,
    ContactStepSpec, MaskedContactSession,
};
use crate::damage::{
    effective_stiffness_factor, evaluate_element_damage, DamageMaterialParams, ElementDamageState,
};
use crate::elasticity::{von_mises_from_voigt, GaussPointState, J2MaterialParams};
use crate::element::{
    rotate_voigt_stress, Tet10Element, Tet4Element, Vector12, Vector30,
};
use crate::error::FeaError;
use crate::solver::{CsrMatrix, NonlinearSolverOptions, NonlinearState, SolverOptions};
use crate::tetra_mesh::{TetNode, TetrahedralMesh};
use crate::NonlinearLoadStage;

// ============================================================================
// Public Configuration & Result Data Structures
// ============================================================================

/// Topological fracture propagation strategy for local FEA failure (`P5-T5`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum FractureTopologyMode {
    /// Erodes elements whose scalar damage reaches $D \ge D_{\text{erode}}$ from stiffness
    /// assembly and contact surfaces. Robust for arbitrary 3D crack paths and contact meshes.
    #[default]
    ElementErosion,
    /// Severs interior tetrahedral faces aligned with the Mode-I crack plane normal $\mathbf{n}_1$
    /// and duplicates nodes across open crack fronts without deleting any elements (exact zero
    /// eroded mass).
    FaceSplitting,
}

/// Configuration options for local fracture FEA solvers (`P5-T5`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FractureSolverOptions {
    /// Topological crack-evolution mode (`ElementErosion` or `FaceSplitting`).
    pub topology_mode: FractureTopologyMode,
    /// Maximum staggered crack-propagation substeps per load step (default `24`).
    pub max_propagation_substeps: usize,
    /// Minimum element count for a detached connected component to be exported as an independent
    /// rigid [`DetachedFragment`] (default `1`).
    pub min_fragment_elements: usize,
    /// Physical time increment $\Delta t$ (seconds) per load step used to estimate instantaneous
    /// nodal velocity $\mathbf{v}_i = (\mathbf{u}_i^{(n)} - \mathbf{u}_i^{(n-1)}) / \Delta t$
    /// when initializing detached rigid-body linear and angular velocities (default `1e-3` s).
    pub step_dt_s: f64,
    /// Minimum alignment $|\mathbf{n}_{\text{face}} \cdot \mathbf{n}_1|$ for Mode-I crack face
    /// selection in [`FractureTopologyMode::FaceSplitting`] (default `0.35`).
    pub face_split_normal_alignment: f64,
    /// Single-body nonlinear Co-Rotated $J_2$ solver options.
    pub nonlinear: NonlinearSolverOptions,
    /// Multi-body deformable contact solver options.
    pub contact: ContactSolverOptions,
}

impl Default for FractureSolverOptions {
    fn default() -> Self {
        Self {
            topology_mode: FractureTopologyMode::ElementErosion,
            max_propagation_substeps: 24,
            min_fragment_elements: 1,
            step_dt_s: 1e-3,
            face_split_normal_alignment: 0.35,
            nonlinear: NonlinearSolverOptions::default(),
            contact: ContactSolverOptions::default(),
        }
    }
}

/// Exact rigid-body mass, inertia, and momentum kinematics integrated over a subset of
/// deformed finite elements (parent surviving solid or detached fragment).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FragmentKinematics {
    /// Total mass $M = \sum_e \rho V_{e,0}$ in kilograms.
    pub mass_kg: f64,
    /// Total undeformed reference volume $V_0 = \sum_e V_{e,0}$ in m³.
    pub volume_m3: f64,
    /// World-space center of mass $\mathbf{X}_C$ in meters.
    pub center_of_mass: [f64; 3],
    /// Symmetric $3\times 3$ inertia tensor $\mathbf{I}_C$ about `center_of_mass` in world axes (kg·m²).
    pub inertia_tensor_com: [[f64; 3]; 3],
    /// Rigid-body COM linear velocity $\mathbf{v}_C = \mathbf{P} / M$ in m/s.
    pub linear_velocity: [f64; 3],
    /// Rigid-body angular velocity $\boldsymbol{\omega}_C = \mathbf{I}_C^{-1} \mathbf{L}_C$ in rad/s.
    pub angular_velocity: [f64; 3],
    /// Exact total linear momentum $\mathbf{P} = \int \rho \mathbf{v} \, dV$ in kg·m/s.
    pub linear_momentum: [f64; 3],
    /// Exact total angular momentum about the world origin $\mathbf{L}_O = \int \rho (\mathbf{x} \times \mathbf{v}) \, dV$ in kg·m²/s.
    pub angular_momentum_origin: [f64; 3],
    /// Exact intrinsic angular momentum about the COM $\mathbf{L}_C = \mathbf{L}_O - \mathbf{X}_C \times \mathbf{P}$ in kg·m²/s.
    pub angular_momentum_com: [f64; 3],
}

impl Default for FragmentKinematics {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            volume_m3: 0.0,
            center_of_mass: [0.0; 3],
            inertia_tensor_com: [[0.0; 3]; 3],
            linear_velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            linear_momentum: [0.0; 3],
            angular_momentum_origin: [0.0; 3],
            angular_momentum_com: [0.0; 3],
        }
    }
}

/// An independent fractured fragment (e.g. broken gear tooth) extracted from a deformed FEA body,
/// ready to spawn directly into `mcp-cad-physics::PhysicsWorld` for free flight and collisions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetachedFragment {
    /// Deterministic unique identifier (e.g. `"brass_gear_frag_0"`).
    pub fragment_id: String,
    /// Index of the parent body (`0` for single-body solves).
    pub source_body_index: usize,
    /// Identifier of the parent part.
    pub source_part_id: String,
    /// Load step index at which detachment occurred.
    pub detachment_step: usize,
    /// Indices of the parent mesh tetrahedra belonging to this fragment.
    pub element_indices: Vec<usize>,
    /// Material density $\rho$ in kg/m³.
    pub density_kg_m3: f64,
    /// Exact mass, COM, $3\times 3$ inertia tensor, linear & angular velocities, and momenta.
    pub kinematics: FragmentKinematics,
    /// Watertight outward-oriented boundary mesh in world deformed coordinates (m).
    pub world_surface_mesh: TriangleMesh,
    /// Watertight outward-oriented boundary mesh centered at `kinematics.center_of_mass` (local COM = `[0,0,0]`).
    pub local_surface_mesh: TriangleMesh,
}

/// Exact mass, linear momentum, and angular momentum conservation audit across a fragment
/// detachment event (`before` vs `surviving parent + detached fragments + newly eroded`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConservationCheck {
    /// Total active mass immediately before the detachment partition (kg).
    pub mass_before_kg: f64,
    /// Combined mass of surviving parent body and newly detached fragments (kg).
    pub mass_after_surviving_and_fragments_kg: f64,
    /// Mass of micro-elements eroded during this fracture event (`0.0` in `FaceSplitting` mode).
    pub mass_eroded_kg: f64,
    /// Relative mass conservation error $|M_{\text{before}} - (M_{\text{surv+frag}} + M_{\text{eroded}})| / M_{\text{before}}$.
    pub mass_rel_error: f64,
    /// Total linear momentum immediately before detachment (kg·m/s).
    pub linear_momentum_before: [f64; 3],
    /// Combined linear momentum after detachment including eroded micro-elements (kg·m/s).
    pub linear_momentum_after: [f64; 3],
    /// Linear momentum carried by eroded micro-elements (`[0,0,0]` in `FaceSplitting` mode).
    pub linear_momentum_eroded: [f64; 3],
    /// Relative linear momentum conservation error $\|\mathbf{P}_{\text{before}} - \mathbf{P}_{\text{after}}\| / \max(\|\mathbf{P}_{\text{before}}\|, 10^{-12})$.
    pub linear_momentum_rel_error: f64,
    /// Total angular momentum about the world origin immediately before detachment (kg·m²/s).
    pub angular_momentum_before: [f64; 3],
    /// Combined angular momentum after detachment including eroded micro-elements (kg·m²/s).
    pub angular_momentum_after: [f64; 3],
    /// Angular momentum carried by eroded micro-elements (`[0,0,0]` in `FaceSplitting` mode).
    pub angular_momentum_eroded: [f64; 3],
    /// Relative angular momentum conservation error $\|\mathbf{L}_{\text{before}} - \mathbf{L}_{\text{after}}\| / \max(\|\mathbf{L}_{\text{before}}\|, 10^{-12})$.
    pub angular_momentum_rel_error: f64,
}

/// Telemetry record for a single fracture topology update (erosion, face split, or fragment detachment).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FractureEvent {
    /// Load step index.
    pub step_index: usize,
    /// Staggered crack-propagation substep index within `step_index`.
    pub substep_index: usize,
    /// Body index (`0` for single-body solves).
    pub body_index: usize,
    /// Elements newly eroded during this substep (empty in `FaceSplitting` mode).
    pub newly_eroded_elements: Vec<usize>,
    /// Number of interior faces newly split during this substep (`0` in `ElementErosion` mode).
    pub newly_split_faces: usize,
    /// Identifiers of any fragments detached during this substep.
    pub detached_fragment_ids: Vec<String>,
    /// Exact conservation check when fragment detachment occurred.
    pub conservation: Option<ConservationCheck>,
}

/// Per-load-step summary for single-body fracture FEA.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FractureStepReport {
    /// Zero-based load step index.
    pub step_index: usize,
    /// Descriptive label of the stage/step.
    pub label: String,
    /// Number of staggered crack-propagation substeps executed in this step.
    pub propagation_substeps: usize,
    /// Peak von Mises effective stress among active elements (Pa).
    pub max_von_mises_pa: f64,
    /// Peak maximum principal tensile stress $\sigma_1$ among active elements (Pa).
    pub max_principal_stress_pa: f64,
    /// Peak equivalent plastic strain among active elements.
    pub max_equiv_plastic_strain: f64,
    /// Peak scalar damage $D \in [0, 1]$ across all elements.
    pub max_damage: f64,
    /// Cumulative eroded element count.
    pub eroded_element_count: usize,
    /// Cumulative split interior face count.
    pub split_face_count: usize,
    /// Remaining active parent element count.
    pub active_element_count: usize,
    /// Cumulative Bažant crack-band fracture energy dissipated (Joules).
    pub total_fracture_energy_j: f64,
    /// Number of fragments detached during this step.
    pub newly_detached_fragments: usize,
}

/// Complete report of a single-body local fracture FEA simulation (`P5-T5`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FractureReport {
    /// Part identifier.
    pub part_id: String,
    /// Topology mode used (`ElementErosion` or `FaceSplitting`).
    pub topology_mode: FractureTopologyMode,
    /// Per-step telemetry summaries.
    pub steps: Vec<FractureStepReport>,
    /// Detailed chronological log of all erosion, face-split, and fragment-detachment events.
    pub events: Vec<FractureEvent>,
    /// Per-element final continuum damage states (aligned with `mesh.elements` / `mesh.tet10_elements`).
    pub element_damage: Vec<ElementDamageState>,
    /// Per-element active mask (`true` = part of surviving parent body, `false` = eroded or detached).
    pub active_elements: Vec<bool>,
    /// Watertight deformed exterior surface mesh of the surviving parent body.
    pub surviving_surface_mesh: TriangleMesh,
    /// Exact rigid-body kinematics of the surviving parent body at the final state.
    pub surviving_kinematics: FragmentKinematics,
    /// All fragments detached over the course of the simulation.
    pub detached_fragments: Vec<DetachedFragment>,
    /// Exact conservation audits for every detachment event.
    pub conservation_checks: Vec<ConservationCheck>,
    /// Final converged nonlinear elastoplastic state.
    pub final_state: NonlinearState,
    /// Final deformed nodal positions `[x, y, z]` (m) of the (possibly node-split) mesh.
    pub final_deformed_positions: Vec<[f64; 3]>,
    /// Total eroded elements across the simulation (`0` in `FaceSplitting` mode).
    pub total_eroded_elements: usize,
    /// Total split interior faces (`0` in `ElementErosion` mode).
    pub total_split_faces: usize,
    /// Total Bažant crack-band fracture energy dissipated (Joules).
    pub total_fracture_energy_j: f64,
    /// Wall-clock computation time in milliseconds.
    pub elapsed_ms: f64,
}

/// Per-load-step summary for multi-body deformable contact with local fracture.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FractureContactStepReport {
    /// Underlying converged multi-body contact step report.
    pub contact: ContactStepReport,
    /// Number of staggered crack-propagation substeps executed in this step.
    pub propagation_substeps: usize,
    /// Per-body maximum scalar damage $D \in [0, 1]$.
    pub body_max_damage: Vec<f64>,
    /// Per-body cumulative eroded element counts.
    pub body_eroded_count: Vec<usize>,
    /// Per-body cumulative split interior face counts (`FaceSplitting` mode).
    #[serde(default)]
    pub body_split_face_count: Vec<usize>,
    /// Per-body remaining active parent element counts.
    pub body_active_count: Vec<usize>,
    /// Per-body cumulative Bažant fracture energy dissipated (Joules).
    pub body_fracture_energy_j: Vec<f64>,
    /// Number of fragments detached across all bodies during this step.
    pub newly_detached_fragments: usize,
}

/// Complete report of a multi-body deformable contact simulation with local fracture (`P5-T5`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FractureContactReport {
    /// Topology mode used.
    pub topology_mode: FractureTopologyMode,
    /// Per-step contact + fracture telemetry summaries.
    pub steps: Vec<FractureContactStepReport>,
    /// Chronological fracture events across all bodies.
    pub events: Vec<FractureEvent>,
    /// Per-body per-element final damage states.
    pub body_element_damage: Vec<Vec<ElementDamageState>>,
    /// Per-body final active parent element masks.
    pub body_active_elements: Vec<Vec<bool>>,
    /// Per-body cumulative split interior face counts (`FaceSplitting` mode).
    #[serde(default)]
    pub body_total_split_faces: Vec<usize>,
    /// Per-body watertight deformed exterior surface meshes of the surviving parent solids.
    pub body_surviving_meshes: Vec<TriangleMesh>,
    /// Per-body exact rigid-body kinematics of the surviving parent solids.
    pub body_surviving_kinematics: Vec<FragmentKinematics>,
    /// Final converged nonlinear elastoplastic states per body.
    pub body_states: Vec<NonlinearState>,
    /// Final deformed nodal coordinates per body.
    pub body_deformed_positions: Vec<Vec<[f64; 3]>>,
    /// All detached fragments extracted across all bodies.
    pub detached_fragments: Vec<DetachedFragment>,
    /// Exact conservation audits for every detachment event.
    pub conservation_checks: Vec<ConservationCheck>,
    /// Wall-clock computation time in milliseconds.
    pub elapsed_ms: f64,
}

// ============================================================================
// Pure Topology, Kinematics, Watertight Surface & Conservation Functions
// ============================================================================

/// Returns the 4 corner vertex indices of element `e_idx` for either `TET4` or `TET10` meshes.
#[inline]
fn element_corners(mesh: &TetrahedralMesh, e_idx: usize) -> [usize; 4] {
    if mesh.is_tet10() {
        let n = mesh.tet10_elements[e_idx];
        [n[0], n[1], n[2], n[3]]
    } else {
        mesh.elements[e_idx].node_indices
    }
}

/// Returns total number of tetrahedral elements in `mesh`.
#[inline]
fn mesh_element_count(mesh: &TetrahedralMesh) -> usize {
    if mesh.is_tet10() {
        mesh.tet10_elements.len()
    } else {
        mesh.elements.len()
    }
}

/// Returns sorted canonical triangle key for face adjacency hashing.
#[inline]
fn sorted_face_key(a: usize, b: usize, c: usize) -> [usize; 3] {
    let mut k = [a, b, c];
    k.sort_unstable();
    k
}

/// Returns sorted canonical element-pair key for `split_faces`.
#[inline]
fn sorted_elem_pair(a: usize, b: usize) -> (usize, usize) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// Computes exact undeformed volume $V_{e,0} = \frac{1}{6}|\det([\mathbf{p}_1-\mathbf{p}_0, \mathbf{p}_2-\mathbf{p}_0, \mathbf{p}_3-\mathbf{p}_0])|$.
#[inline]
fn tet_reference_volume(mesh: &TetrahedralMesh, corners: [usize; 4]) -> f64 {
    let p0 = Vector3::from(mesh.nodes[corners[0]].position);
    let p1 = Vector3::from(mesh.nodes[corners[1]].position);
    let p2 = Vector3::from(mesh.nodes[corners[2]].position);
    let p3 = Vector3::from(mesh.nodes[corners[3]].position);
    ((p1 - p0).cross(&(p2 - p0)).dot(&(p3 - p0))).abs() / 6.0
}

/// Integrates exact mass, COM, $3\times 3$ inertia tensor about COM, linear momentum,
/// angular momentum about the world origin, and rigid-body $(\mathbf{v}_C, \boldsymbol{\omega}_C)$
/// over a subset of deformed tetrahedral elements `elem_indices`.
///
/// ### Exact Linear-Simplex Identities
/// For each tetrahedron $e = (n_0, n_1, n_2, n_3)$ with Lagrangian reference mass $m_e = \rho V_{e,0}$,
/// deformed vertex positions $\mathbf{x}_a$, and nodal velocities $\mathbf{v}_a$:
/// - $\mathbf{c}_e = \frac{1}{4}\sum_a \mathbf{x}_a$, $\mathbf{p}_e = \frac{m_e}{4}\sum_a \mathbf{v}_a$,
/// - $\mathbf{l}_{O,e} = \frac{m_e}{20}\left(\sum_a \mathbf{x}_a \times \mathbf{v}_a + \left(\sum_a \mathbf{x}_a\right)\times\left(\sum_b \mathbf{v}_b\right)\right)$,
/// - About component COM $\mathbf{X}_C$ with $\mathbf{r}_a = \mathbf{x}_a - \mathbf{X}_C$ and $\mathbf{s}_r = \sum_a \mathbf{r}_a$:
///   $$\mathbf{C}_C = \sum_e \frac{m_e}{20}\left(\sum_a \mathbf{r}_a \mathbf{r}_a^T + \mathbf{s}_r \mathbf{s}_r^T\right), \qquad \mathbf{I}_C = \operatorname{tr}(\mathbf{C}_C)\mathbf{I}_3 - \mathbf{C}_C.$$
///
/// Because $m_e$, $\mathbf{p}_e$, and $\mathbf{l}_{O,e}$ are strictly additive over elements,
/// partitioning any active mesh into surviving parent + detached fragments (+ eroded) conserves
/// total mass, linear momentum, and angular momentum to $\sim 10^{-15}$.
pub fn compute_subset_kinematics(
    mesh: &TetrahedralMesh,
    deformed_positions: &[[f64; 3]],
    nodal_velocities: &[[f64; 3]],
    elem_indices: &[usize],
    density_kg_m3: f64,
    element_volumes_override: Option<&[f64]>,
) -> FragmentKinematics {
    if elem_indices.is_empty() || density_kg_m3 <= 0.0 {
        return FragmentKinematics::default();
    }

    let mut total_vol = 0.0_f64;
    let mut total_mass = 0.0_f64;
    let mut weighted_com = Vector3::<f64>::zeros();
    let mut total_p = Vector3::<f64>::zeros();
    let mut total_l_origin = Vector3::<f64>::zeros();

    for &e_idx in elem_indices {
        let corners = element_corners(mesh, e_idx);
        let vol = element_volumes_override
            .and_then(|v| v.get(e_idx).copied())
            .unwrap_or_else(|| tet_reference_volume(mesh, corners));
        if vol <= 0.0 {
            continue;
        }
        let m_e = density_kg_m3 * vol;
        total_vol += vol;
        total_mass += m_e;

        let x = [
            Vector3::from(deformed_positions[corners[0]]),
            Vector3::from(deformed_positions[corners[1]]),
            Vector3::from(deformed_positions[corners[2]]),
            Vector3::from(deformed_positions[corners[3]]),
        ];
        let v = [
            Vector3::from(nodal_velocities[corners[0]]),
            Vector3::from(nodal_velocities[corners[1]]),
            Vector3::from(nodal_velocities[corners[2]]),
            Vector3::from(nodal_velocities[corners[3]]),
        ];

        let sum_x = x[0] + x[1] + x[2] + x[3];
        let sum_v = v[0] + v[1] + v[2] + v[3];
        weighted_com += sum_x * (0.25 * m_e);
        total_p += sum_v * (0.25 * m_e);

        let mut diag_cross = Vector3::<f64>::zeros();
        for a in 0..4 {
            diag_cross += x[a].cross(&v[a]);
        }
        total_l_origin += (diag_cross + sum_x.cross(&sum_v)) * (m_e / 20.0);
    }

    if total_mass <= 1e-30 {
        return FragmentKinematics::default();
    }

    let com = weighted_com / total_mass;
    let lin_vel = total_p / total_mass;
    let l_com = total_l_origin - com.cross(&total_p);

    // Exact second-order covariance matrix C_C = ∫ ρ (x - X_C)(x - X_C)^T dV
    let mut cov = Matrix3::<f64>::zeros();
    for &e_idx in elem_indices {
        let corners = element_corners(mesh, e_idx);
        let vol = element_volumes_override
            .and_then(|v| v.get(e_idx).copied())
            .unwrap_or_else(|| tet_reference_volume(mesh, corners));
        if vol <= 0.0 {
            continue;
        }
        let m_e = density_kg_m3 * vol;
        let r = [
            Vector3::from(deformed_positions[corners[0]]) - com,
            Vector3::from(deformed_positions[corners[1]]) - com,
            Vector3::from(deformed_positions[corners[2]]) - com,
            Vector3::from(deformed_positions[corners[3]]) - com,
        ];
        let s = r[0] + r[1] + r[2] + r[3];
        let w = m_e / 20.0;
        for row in 0..3 {
            for col in row..3 {
                let val = w
                    * (r[0][row] * r[0][col]
                        + r[1][row] * r[1][col]
                        + r[2][row] * r[2][col]
                        + r[3][row] * r[3][col]
                        + s[row] * s[col]);
                cov[(row, col)] += val;
            }
        }
    }
    cov[(1, 0)] = cov[(0, 1)];
    cov[(2, 0)] = cov[(0, 2)];
    cov[(2, 1)] = cov[(1, 2)];

    let trace_cov = cov.trace();
    let mut inertia = Matrix3::from_diagonal_element(trace_cov) - cov;
    // Symmetrize
    for row in 0..3 {
        inertia[(row, row)] = inertia[(row, row)].max(0.0);
        for col in (row + 1)..3 {
            let sym = 0.5 * (inertia[(row, col)] + inertia[(col, row)]);
            inertia[(row, col)] = sym;
            inertia[(col, row)] = sym;
        }
    }

    let ang_vel = inertia
        .try_inverse()
        .map(|inv_i| inv_i * l_com)
        .unwrap_or_else(Vector3::zeros);

    FragmentKinematics {
        mass_kg: total_mass,
        volume_m3: total_vol,
        center_of_mass: [com.x, com.y, com.z],
        inertia_tensor_com: [
            [inertia[(0, 0)], inertia[(0, 1)], inertia[(0, 2)]],
            [inertia[(1, 0)], inertia[(1, 1)], inertia[(1, 2)]],
            [inertia[(2, 0)], inertia[(2, 1)], inertia[(2, 2)]],
        ],
        linear_velocity: [lin_vel.x, lin_vel.y, lin_vel.z],
        angular_velocity: [ang_vel.x, ang_vel.y, ang_vel.z],
        linear_momentum: [total_p.x, total_p.y, total_p.z],
        angular_momentum_origin: [total_l_origin.x, total_l_origin.y, total_l_origin.z],
        angular_momentum_com: [l_com.x, l_com.y, l_com.z],
    }
}

/// Finds all face-connected components among active elements in `mesh`, treating any pair in
/// `split_faces` as severed (non-adjacent).
///
/// Returns components deterministically sorted by descending element count, then lowest element index.
pub fn find_face_connected_components(
    mesh: &TetrahedralMesh,
    active_elements: &[bool],
    split_faces: &HashSet<(usize, usize)>,
) -> Vec<Vec<usize>> {
    let n_elems = mesh_element_count(mesh);
    let mut face_owners: HashMap<[usize; 3], Vec<usize>> = HashMap::with_capacity(n_elems * 2);

    for (e_idx, &is_active) in active_elements.iter().enumerate().take(n_elems) {
        if !is_active {
            continue;
        }
        let [n0, n1, n2, n3] = element_corners(mesh, e_idx);
        let faces = [
            sorted_face_key(n0, n1, n2),
            sorted_face_key(n0, n1, n3),
            sorted_face_key(n0, n2, n3),
            sorted_face_key(n1, n2, n3),
        ];
        for fk in faces {
            face_owners.entry(fk).or_default().push(e_idx);
        }
    }

    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n_elems];
    for owners in face_owners.values() {
        if owners.len() == 2 {
            let ea = owners[0];
            let eb = owners[1];
            if !split_faces.contains(&sorted_elem_pair(ea, eb)) {
                adj[ea].push(eb);
                adj[eb].push(ea);
            }
        }
    }

    let mut visited = vec![false; n_elems];
    let mut components = Vec::new();
    let mut queue = VecDeque::new();

    for start in 0..n_elems {
        if !active_elements.get(start).copied().unwrap_or(false) || visited[start] {
            continue;
        }
        visited[start] = true;
        queue.push_back(start);
        let mut comp = Vec::new();
        while let Some(u) = queue.pop_front() {
            comp.push(u);
            for &v in &adj[u] {
                if !visited[v] {
                    visited[v] = true;
                    queue.push_back(v);
                }
            }
        }
        comp.sort_unstable();
        components.push(comp);
    }

    components.sort_by(|a, b| b.len().cmp(&a.len()).then_with(|| a[0].cmp(&b[0])));
    components
}

/// Extracts an exact watertight outward-oriented [`TriangleMesh`] for the boundary of the
/// tetrahedral subset `elem_indices` using `deformed_positions`.
///
/// If `origin_shift` is `Some(shift)`, every output vertex position is translated by `-shift`
/// (used to construct COM-centered `local_surface_mesh` for rigid-body spawning).
pub fn extract_component_surface_mesh(
    mesh: &TetrahedralMesh,
    deformed_positions: &[[f64; 3]],
    elem_indices: &[usize],
    origin_shift: Option<[f64; 3]>,
) -> TriangleMesh {
    if elem_indices.is_empty() {
        return TriangleMesh::empty();
    }

    // Weld duplicated FaceSplitting nodes within this single connected component (nodes that share
    // the exact same reference position `mesh.nodes[nid].position`) as well as geometrically
    // coincident deformed vertices so back-to-back internal crack-slit faces cancel cleanly and
    // both surviving parent and detached fragment meshes are strict manifold solids with zero
    // self-intersecting internal slits.
    let mut min_pt = [f64::INFINITY; 3];
    let mut max_pt = [f64::NEG_INFINITY; 3];
    let mut ref_min = [f64::INFINITY; 3];
    let mut ref_max = [f64::NEG_INFINITY; 3];
    for &e_idx in elem_indices {
        for nid in element_corners(mesh, e_idx) {
            let p = deformed_positions[nid];
            let r = mesh.nodes[nid].position;
            for k in 0..3 {
                min_pt[k] = min_pt[k].min(p[k]);
                max_pt[k] = max_pt[k].max(p[k]);
                ref_min[k] = ref_min[k].min(r[k]);
                ref_max[k] = ref_max[k].max(r[k]);
            }
        }
    }
    let diag = ((max_pt[0] - min_pt[0]).powi(2)
        + (max_pt[1] - min_pt[1]).powi(2)
        + (max_pt[2] - min_pt[2]).powi(2))
    .sqrt()
    .max(1e-9);
    let ref_diag = ((ref_max[0] - ref_min[0]).powi(2)
        + (ref_max[1] - ref_min[1]).powi(2)
        + (ref_max[2] - ref_min[2]).powi(2))
    .sqrt()
    .max(1e-9);
    let ref_weld_scale = 1.0 / (ref_diag * 1e-10);
    let def_weld_scale = 1.0 / (diag * 1e-7);

    let shift = origin_shift.unwrap_or([0.0; 3]);
    let mut ref_quant_to_local: HashMap<[i64; 3], u32> = HashMap::new();
    let mut def_quant_to_local: HashMap<[i64; 3], u32> = HashMap::new();
    let mut pos_sums: Vec<[f64; 3]> = Vec::new();
    let mut pos_counts: Vec<usize> = Vec::new();
    let mut nid_to_local: HashMap<usize, u32> = HashMap::new();

    for &e_idx in elem_indices {
        for nid in element_corners(mesh, e_idx) {
            if nid_to_local.contains_key(&nid) {
                continue;
            }
            let r = mesh.nodes[nid].position;
            let p = deformed_positions[nid];
            let rq = [
                (r[0] * ref_weld_scale).round() as i64,
                (r[1] * ref_weld_scale).round() as i64,
                (r[2] * ref_weld_scale).round() as i64,
            ];
            let dq = [
                (p[0] * def_weld_scale).round() as i64,
                (p[1] * def_weld_scale).round() as i64,
                (p[2] * def_weld_scale).round() as i64,
            ];
            let local_id = if let Some(&id) = ref_quant_to_local.get(&rq) {
                id
            } else if let Some(&id) = def_quant_to_local.get(&dq) {
                id
            } else {
                let next_id = pos_sums.len() as u32;
                pos_sums.push([0.0; 3]);
                pos_counts.push(0);
                ref_quant_to_local.insert(rq, next_id);
                def_quant_to_local.insert(dq, next_id);
                next_id
            };
            let idx = local_id as usize;
            pos_sums[idx][0] += p[0] - shift[0];
            pos_sums[idx][1] += p[1] - shift[1];
            pos_sums[idx][2] += p[2] - shift[2];
            pos_counts[idx] += 1;
            nid_to_local.insert(nid, local_id);
        }
    }

    let mut positions_local: Vec<[f64; 3]> = pos_sums
        .into_iter()
        .zip(pos_counts)
        .map(|(s, c)| {
            let inv = 1.0 / (c.max(1) as f64);
            [s[0] * inv, s[1] * inv, s[2] * inv]
        })
        .collect();

    // Collect active welded tets in this component
    let mut welded_tets: Vec<[u32; 4]> = Vec::with_capacity(elem_indices.len());
    for &e_idx in elem_indices {
        let [g0, g1, g2, g3] = element_corners(mesh, e_idx);
        let [n0, n1, n2, n3] = [
            nid_to_local[&g0],
            nid_to_local[&g1],
            nid_to_local[&g2],
            nid_to_local[&g3],
        ];
        if n0 == n1 || n0 == n2 || n0 == n3 || n1 == n2 || n1 == n3 || n2 == n3 {
            continue;
        }
        welded_tets.push([n0, n1, n2, n3]);
    }

    // Build face-adjacency among welded_tets so we can detect pinched non-manifold vertices/edges
    // (where >= 2 disconnected tet fans touch at a single shared vertex or edge on a jagged
    // fracture surface) and separate secondary fans infinitesimally (2e-5 * diag) along their
    // inward tet centroids to guarantee a strictly manifold watertight solid surface.
    let mut face_owners: HashMap<[u32; 3], Vec<usize>> =
        HashMap::with_capacity(welded_tets.len() * 2);
    for (ti, &[n0, n1, n2, n3]) in welded_tets.iter().enumerate() {
        for mut fk in [[n0, n1, n2], [n0, n1, n3], [n0, n2, n3], [n1, n2, n3]] {
            fk.sort_unstable();
            face_owners.entry(fk).or_default().push(ti);
        }
    }

    let mut tet_adj: Vec<Vec<usize>> = vec![Vec::new(); welded_tets.len()];
    for owners in face_owners.values() {
        if owners.len() == 2 {
            tet_adj[owners[0]].push(owners[1]);
            tet_adj[owners[1]].push(owners[0]);
        }
    }

    let n_init_verts = positions_local.len();
    let mut vert_tets: Vec<Vec<usize>> = vec![Vec::new(); n_init_verts];
    for (ti, &tet) in welded_tets.iter().enumerate() {
        for v in tet {
            vert_tets[v as usize].push(ti);
        }
    }

    let sep_eps = diag * 2e-5;
    for v in 0..n_init_verts {
        let star = &vert_tets[v];
        if star.len() <= 1 {
            continue;
        }
        let star_set: HashSet<usize> = star.iter().copied().collect();
        let mut visited: HashSet<usize> = HashSet::with_capacity(star.len());
        let mut components: Vec<Vec<usize>> = Vec::new();
        for &seed in star {
            if !visited.insert(seed) {
                continue;
            }
            let mut comp = vec![seed];
            let mut q = VecDeque::new();
            q.push_back(seed);
            while let Some(cur) = q.pop_front() {
                for &nb in &tet_adj[cur] {
                    if star_set.contains(&nb) && visited.insert(nb) {
                        comp.push(nb);
                        q.push_back(nb);
                    }
                }
            }
            components.push(comp);
        }
        if components.len() <= 1 {
            continue;
        }
        // Keep the largest tet fan at the original vertex position (nudged slightly inward too
        // if multiple fans exist) and give each secondary fan a dedicated vertex copy nudged
        // inward toward its own tet centroids by `sep_eps` (> weld tolerance, < 1e-4 relative).
        components.sort_by(|a, b| b.len().cmp(&a.len()).then_with(|| a[0].cmp(&b[0])));
        let orig_p = Vector3::from(positions_local[v]);
        for (c_idx, comp) in components.iter().enumerate() {
            if c_idx == 0 {
                continue;
            }
            let mut cent = Vector3::<f64>::zeros();
            for &ti in comp {
                let tet = welded_tets[ti];
                for nv in tet {
                    cent += Vector3::from(positions_local[nv as usize]);
                }
            }
            cent /= (comp.len() * 4) as f64;
            let dir = cent - orig_p;
            let len = dir.norm();
            let offset = if len > 1e-18 {
                dir * (sep_eps / len)
            } else {
                Vector3::new(sep_eps, 0.0, 0.0)
            };
            let new_vid = positions_local.len() as u32;
            positions_local.push([
                orig_p.x + offset.x,
                orig_p.y + offset.y,
                orig_p.z + offset.z,
            ]);
            for &ti in comp {
                for slot in &mut welded_tets[ti] {
                    if *slot == v as u32 {
                        *slot = new_vid;
                    }
                }
            }
        }
    }

    let mut face_counts: HashMap<[u32; 3], (usize, [u32; 3])> =
        HashMap::with_capacity(welded_tets.len() * 2);

    for &[n0, n1, n2, n3] in &welded_tets {
        let faces = [
            ([n0, n1, n2], n3),
            ([n0, n1, n3], n2),
            ([n0, n2, n3], n1),
            ([n1, n2, n3], n0),
        ];
        for ([a, b, c], opp) in faces {
            let mut key = [a, b, c];
            key.sort_unstable();
            let pa = Vector3::from(positions_local[a as usize]);
            let pb = Vector3::from(positions_local[b as usize]);
            let pc = Vector3::from(positions_local[c as usize]);
            let popp = Vector3::from(positions_local[opp as usize]);
            let cross = (pb - pa).cross(&(pc - pa));
            let oriented = if cross.dot(&(popp - pa)) > 0.0 {
                [a, c, b]
            } else {
                [a, b, c]
            };
            face_counts
                .entry(key)
                .and_modify(|entry| entry.0 += 1)
                .or_insert((1, oriented));
        }
    }

    // Collect boundary triangles (count == 1) in deterministic order
    let mut indices: Vec<[u32; 3]> = face_counts
        .into_iter()
        .filter_map(|(_, (cnt, tri))| if cnt == 1 { Some(tri) } else { None })
        .collect();
    indices.sort_unstable();

    // Compact vertex buffer so only vertices referenced by surviving boundary triangles remain
    let mut old_to_compact: HashMap<u32, u32> = HashMap::new();
    let mut compact_positions: Vec<[f64; 3]> = Vec::new();
    for tri in &mut indices {
        for v in tri.iter_mut() {
            let next_id = compact_positions.len() as u32;
            let mapped = *old_to_compact.entry(*v).or_insert_with(|| {
                compact_positions.push(positions_local[*v as usize]);
                next_id
            });
            *v = mapped;
        }
    }
    let positions_local = compact_positions;

    // Compute smooth area-weighted outward vertex normals
    let mut normal_accum = vec![Vector3::<f64>::zeros(); positions_local.len()];
    for &[i0, i1, i2] in &indices {
        let p0 = Vector3::from(positions_local[i0 as usize]);
        let p1 = Vector3::from(positions_local[i1 as usize]);
        let p2 = Vector3::from(positions_local[i2 as usize]);
        let area_n = (p1 - p0).cross(&(p2 - p0));
        normal_accum[i0 as usize] += area_n;
        normal_accum[i1 as usize] += area_n;
        normal_accum[i2 as usize] += area_n;
    }

    let vertices: Vec<Vertex> = positions_local
        .into_iter()
        .zip(normal_accum)
        .map(|(pos, n_acc)| {
            let len = n_acc.norm();
            let normal = if len > 1e-24 {
                [n_acc.x / len, n_acc.y / len, n_acc.z / len]
            } else {
                [0.0, 0.0, 1.0]
            };
            Vertex { position: pos, normal }
        })
        .collect();

    TriangleMesh::new(vertices, indices)
}

/// Evaluates exact mass, linear momentum, and angular momentum conservation across a
/// fracture/detachment partition.
#[allow(clippy::too_many_arguments)]
pub fn verify_detachment_conservation(
    mesh: &TetrahedralMesh,
    deformed_positions: &[[f64; 3]],
    nodal_velocities: &[[f64; 3]],
    density_kg_m3: f64,
    pre_event_active_elems: &[usize],
    surviving_elems: &[usize],
    detached_fragments: &[DetachedFragment],
    newly_eroded_elems: &[usize],
    element_volumes_override: Option<&[f64]>,
) -> ConservationCheck {
    let before = compute_subset_kinematics(
        mesh,
        deformed_positions,
        nodal_velocities,
        pre_event_active_elems,
        density_kg_m3,
        element_volumes_override,
    );
    let surviving = compute_subset_kinematics(
        mesh,
        deformed_positions,
        nodal_velocities,
        surviving_elems,
        density_kg_m3,
        element_volumes_override,
    );
    let eroded = compute_subset_kinematics(
        mesh,
        deformed_positions,
        nodal_velocities,
        newly_eroded_elems,
        density_kg_m3,
        element_volumes_override,
    );

    let mut mass_surv_and_frags = surviving.mass_kg;
    let mut p_after = Vector3::from(surviving.linear_momentum) + Vector3::from(eroded.linear_momentum);
    let mut l_after =
        Vector3::from(surviving.angular_momentum_origin) + Vector3::from(eroded.angular_momentum_origin);

    for frag in detached_fragments {
        mass_surv_and_frags += frag.kinematics.mass_kg;
        // Reconstruct fragment momentum strictly from its rigid-body (M, X_C, I_C, v_C, ω_C)
        // to also verify that the exported rigid-body properties match the continuum integral!
        let m_f = frag.kinematics.mass_kg;
        let xc = Vector3::from(frag.kinematics.center_of_mass);
        let vc = Vector3::from(frag.kinematics.linear_velocity);
        let wc = Vector3::from(frag.kinematics.angular_velocity);
        let ic_arr = frag.kinematics.inertia_tensor_com;
        let ic = Matrix3::new(
            ic_arr[0][0], ic_arr[0][1], ic_arr[0][2],
            ic_arr[1][0], ic_arr[1][1], ic_arr[1][2],
            ic_arr[2][0], ic_arr[2][1], ic_arr[2][2],
        );
        let p_f = vc * m_f;
        let l_f = xc.cross(&p_f) + ic * wc;
        p_after += p_f;
        l_after += l_f;
    }

    let total_mass_after = mass_surv_and_frags + eroded.mass_kg;
    let mass_rel_error =
        (before.mass_kg - total_mass_after).abs() / before.mass_kg.abs().max(1e-30);

    let p_before = Vector3::from(before.linear_momentum);
    let p_err = (p_before - p_after).norm();
    let p_scale = p_before.norm().max(1e-12);
    let linear_momentum_rel_error = if p_before.norm() <= 1e-15 && p_after.norm() <= 1e-15 {
        0.0
    } else {
        p_err / p_scale
    };

    let l_before = Vector3::from(before.angular_momentum_origin);
    let l_err = (l_before - l_after).norm();
    let l_scale = l_before.norm().max(1e-12);
    let angular_momentum_rel_error = if l_before.norm() <= 1e-15 && l_after.norm() <= 1e-15 {
        0.0
    } else {
        l_err / l_scale
    };

    ConservationCheck {
        mass_before_kg: before.mass_kg,
        mass_after_surviving_and_fragments_kg: mass_surv_and_frags,
        mass_eroded_kg: eroded.mass_kg,
        mass_rel_error,
        linear_momentum_before: before.linear_momentum,
        linear_momentum_after: [p_after.x, p_after.y, p_after.z],
        linear_momentum_eroded: eroded.linear_momentum,
        linear_momentum_rel_error,
        angular_momentum_before: before.angular_momentum_origin,
        angular_momentum_after: [l_after.x, l_after.y, l_after.z],
        angular_momentum_eroded: eroded.angular_momentum_origin,
        angular_momentum_rel_error,
    }
}

/// Partitions a body into anchored/surviving parent elements and detached rigid [`DetachedFragment`]s.
///
/// Any face-connected component containing at least one node in `anchored_nodes` remains part
/// of the parent body. If `anchored_nodes` is empty (unconstrained body), the largest-mass
/// component is kept as the parent. All other components with `len >= min_fragment_elements`
/// are detached, marked inactive in `active_elements`, and returned along with an exact
/// [`ConservationCheck`].
#[allow(clippy::too_many_arguments)]
pub fn partition_fractured_body(
    body_index: usize,
    source_part_id: &str,
    step_index: usize,
    mesh: &TetrahedralMesh,
    deformed_positions: &[[f64; 3]],
    nodal_velocities: &[[f64; 3]],
    density_kg_m3: f64,
    active_elements: &mut [bool],
    split_faces: &HashSet<(usize, usize)>,
    anchored_nodes: &HashSet<usize>,
    pre_event_active_elems: &[usize],
    newly_eroded_elems: &[usize],
    min_fragment_elements: usize,
    next_fragment_counter: &mut usize,
    element_volumes_override: Option<&[f64]>,
) -> (Vec<DetachedFragment>, Option<ConservationCheck>) {
    let components = find_face_connected_components(mesh, active_elements, split_faces);
    if components.len() <= 1 {
        return (Vec::new(), None);
    }

    // Identify which components are anchored to supports / hub shafts
    let mut is_parent_comp = vec![false; components.len()];
    let mut any_anchored = false;
    if !anchored_nodes.is_empty() {
        for (c_idx, comp) in components.iter().enumerate() {
            let has_anchor = comp.iter().any(|&e_idx| {
                element_corners(mesh, e_idx)
                    .iter()
                    .any(|nid| anchored_nodes.contains(nid))
            });
            if has_anchor {
                is_parent_comp[c_idx] = true;
                any_anchored = true;
            }
        }
    }
    if !any_anchored {
        // Keep largest component (components are already sorted descending by size)
        is_parent_comp[0] = true;
    }

    let mut detached = Vec::new();
    let mut surviving_elems = Vec::new();
    let mut discarded_micro_elems: Vec<usize> = newly_eroded_elems.to_vec();

    for (c_idx, comp) in components.into_iter().enumerate() {
        if is_parent_comp[c_idx] {
            surviving_elems.extend(comp);
            continue;
        }
        // Mark all elements of detached component inactive in the parent FEA solve
        for &e_idx in &comp {
            active_elements[e_idx] = false;
        }
        if comp.len() < min_fragment_elements.max(1) {
            discarded_micro_elems.extend(comp);
            continue;
        }

        let kin = compute_subset_kinematics(
            mesh,
            deformed_positions,
            nodal_velocities,
            &comp,
            density_kg_m3,
            element_volumes_override,
        );
        let world_surface_mesh =
            extract_component_surface_mesh(mesh, deformed_positions, &comp, None);
        let local_surface_mesh = extract_component_surface_mesh(
            mesh,
            deformed_positions,
            &comp,
            Some(kin.center_of_mass),
        );

        let frag_id = format!("{source_part_id}_frag_{}", *next_fragment_counter);
        *next_fragment_counter += 1;

        detached.push(DetachedFragment {
            fragment_id: frag_id,
            source_body_index: body_index,
            source_part_id: source_part_id.to_string(),
            detachment_step: step_index,
            element_indices: comp,
            density_kg_m3,
            kinematics: kin,
            world_surface_mesh,
            local_surface_mesh,
        });
    }

    if detached.is_empty() {
        return (Vec::new(), None);
    }

    let check = verify_detachment_conservation(
        mesh,
        deformed_positions,
        nodal_velocities,
        density_kg_m3,
        pre_event_active_elems,
        &surviving_elems,
        &detached,
        &discarded_micro_elems,
        element_volumes_override,
    );

    (detached, Some(check))
}

// ============================================================================
// Face-Splitting Topology Helper (Zero-Mass-Loss Crack Face Separation)
// ============================================================================

/// Identifies interior faces adjacent to newly failed elements that align with their Mode-I
/// principal crack normal $\mathbf{n}_1$, adds them to `split_faces`, and duplicates shared
/// free nodes whose local incident element fan separates across the crack.
///
/// Returns `(newly_split_faces, duplicated_nodes)` where each entry in `duplicated_nodes`
/// is `(original_node_id, new_node_id)`.
#[allow(clippy::too_many_arguments)]
fn apply_face_splitting_step_with_map(
    mesh: &mut TetrahedralMesh,
    u_cur: &mut Vec<f64>,
    u_prev: &mut Vec<f64>,
    deformed_positions: &mut Vec<[f64; 3]>,
    active_elements: &[bool],
    damage_states: &[ElementDamageState],
    failed_elements: &[usize],
    split_faces: &mut HashSet<(usize, usize)>,
    anchored_nodes: &HashSet<usize>,
    min_normal_alignment: f64,
) -> (usize, Vec<(usize, usize)>) {
    if failed_elements.is_empty() || mesh.is_tet10() {
        // FaceSplitting operates on linear tetrahedral corner connectivity; TET10 uses ElementErosion.
        return (0, Vec::new());
    }

    let n_elems = mesh.elements.len();
    // Build interior face map: face_key -> (e1, e2, [a, b, c])
    let mut face_to_elems: HashMap<[usize; 3], Vec<usize>> = HashMap::with_capacity(n_elems * 2);
    for (e_idx, &is_act) in active_elements.iter().enumerate().take(n_elems) {
        if !is_act {
            continue;
        }
        let [n0, n1, n2, n3] = mesh.elements[e_idx].node_indices;
        for fk in [
            sorted_face_key(n0, n1, n2),
            sorted_face_key(n0, n1, n3),
            sorted_face_key(n0, n2, n3),
            sorted_face_key(n1, n2, n3),
        ] {
            face_to_elems.entry(fk).or_default().push(e_idx);
        }
    }

    let mut newly_split = 0usize;
    let mut touched_nodes: HashSet<usize> = HashSet::new();

    for &e_idx in failed_elements {
        if !active_elements.get(e_idx).copied().unwrap_or(false) {
            continue;
        }
        let n1_vec = Vector3::from(damage_states[e_idx].crack_normal);
        let n1_unit = if n1_vec.norm() > 1e-12 {
            n1_vec.normalize()
        } else {
            Vector3::x()
        };

        let [v0, v1, v2, v3] = mesh.elements[e_idx].node_indices;
        let elem_faces = [[v0, v1, v2], [v0, v1, v3], [v0, v2, v3], [v1, v2, v3]];

        let mut candidates: Vec<(f64, usize, [usize; 3])> = Vec::new();
        for f in elem_faces {
            let fk = sorted_face_key(f[0], f[1], f[2]);
            if let Some(owners) = face_to_elems.get(&fk) {
                if owners.len() == 2 {
                    let nb = if owners[0] == e_idx { owners[1] } else { owners[0] };
                    let pair = sorted_elem_pair(e_idx, nb);
                    if split_faces.contains(&pair) {
                        continue;
                    }
                    let pa = Vector3::from(deformed_positions[f[0]]);
                    let pb = Vector3::from(deformed_positions[f[1]]);
                    let pc = Vector3::from(deformed_positions[f[2]]);
                    let fn_raw = (pb - pa).cross(&(pc - pa));
                    let align = if fn_raw.norm() > 1e-24 {
                        fn_raw.normalize().dot(&n1_unit).abs()
                    } else {
                        0.0
                    };
                    candidates.push((align, nb, f));
                }
            }
        }
        candidates.sort_by(|a, b| b.0.total_cmp(&a.0));

        let mut split_any_for_e = false;
        for &(align, nb, f) in &candidates {
            if align >= min_normal_alignment && split_faces.insert(sorted_elem_pair(e_idx, nb)) {
                newly_split += 1;
                split_any_for_e = true;
                touched_nodes.extend(f);
            }
        }
        // Fallback: if no face met the alignment threshold, split the best-aligned unsplit face
        if !split_any_for_e {
            if let Some(&(_, nb, f)) = candidates.first() {
                if split_faces.insert(sorted_elem_pair(e_idx, nb)) {
                    newly_split += 1;
                    touched_nodes.extend(f);
                }
            }
        }
    }

    if newly_split == 0 {
        return (0, Vec::new());
    }

    // Duplicate free nodes whose local active element fan splits into >= 2 unsplit-face sectors
    let mut node_to_elems: HashMap<usize, Vec<usize>> = HashMap::new();
    for (e_idx, &is_act) in active_elements.iter().enumerate().take(n_elems) {
        if !is_act {
            continue;
        }
        for &nid in &mesh.elements[e_idx].node_indices {
            if touched_nodes.contains(&nid) {
                node_to_elems.entry(nid).or_default().push(e_idx);
            }
        }
    }

    // Build unsplit face adjacency among active elements
    let mut elem_adj: HashMap<usize, Vec<usize>> = HashMap::new();
    for owners in face_to_elems.values() {
        if owners.len() == 2 {
            let ea = owners[0];
            let eb = owners[1];
            if !split_faces.contains(&sorted_elem_pair(ea, eb)) {
                elem_adj.entry(ea).or_default().push(eb);
                elem_adj.entry(eb).or_default().push(ea);
            }
        }
    }

    let mut touched_sorted: Vec<usize> = touched_nodes.into_iter().collect();
    touched_sorted.sort_unstable();

    let mut duplicated_nodes: Vec<(usize, usize)> = Vec::new();

    for nid in touched_sorted {
        // Never duplicate anchored Dirichlet/Hub nodes so support constraints stay attached to parent
        if anchored_nodes.contains(&nid) {
            continue;
        }
        let Some(inc_elems) = node_to_elems.get(&nid) else {
            continue;
        };
        if inc_elems.len() <= 1 {
            continue;
        }
        let inc_set: HashSet<usize> = inc_elems.iter().copied().collect();
        let mut local_visited: HashSet<usize> = HashSet::new();
        let mut sectors: Vec<Vec<usize>> = Vec::new();

        for &start_e in inc_elems {
            if local_visited.contains(&start_e) {
                continue;
            }
            let mut sector = Vec::new();
            let mut q = VecDeque::new();
            local_visited.insert(start_e);
            q.push_back(start_e);
            while let Some(u) = q.pop_front() {
                sector.push(u);
                if let Some(nbs) = elem_adj.get(&u) {
                    for &v in nbs {
                        if inc_set.contains(&v) && !local_visited.contains(&v) {
                            local_visited.insert(v);
                            q.push_back(v);
                        }
                    }
                }
            }
            sectors.push(sector);
        }

        if sectors.len() <= 1 {
            continue;
        }

        // Keep original `nid` for sector 0; create duplicate nodes for sectors 1..
        let orig_pos = mesh.nodes[nid].position;
        let orig_def = deformed_positions[nid];
        let u_c = [u_cur[3 * nid], u_cur[3 * nid + 1], u_cur[3 * nid + 2]];
        let u_p = [u_prev[3 * nid], u_prev[3 * nid + 1], u_prev[3 * nid + 2]];

        for sector in sectors.into_iter().skip(1) {
            let new_nid = mesh.nodes.len();
            mesh.nodes.push(TetNode {
                id: new_nid,
                position: orig_pos,
            });
            deformed_positions.push(orig_def);
            u_cur.extend_from_slice(&u_c);
            u_prev.extend_from_slice(&u_p);
            duplicated_nodes.push((nid, new_nid));

            for e_idx in sector {
                for slot in &mut mesh.elements[e_idx].node_indices {
                    if *slot == nid {
                        *slot = new_nid;
                    }
                }
            }
        }
    }

    (newly_split, duplicated_nodes)
}

/// Returns the number of newly split interior faces (convenience wrapper over
/// [`apply_face_splitting_step_with_map`]).
#[allow(clippy::too_many_arguments)]
fn apply_face_splitting_step(
    mesh: &mut TetrahedralMesh,
    u_cur: &mut Vec<f64>,
    u_prev: &mut Vec<f64>,
    deformed_positions: &mut Vec<[f64; 3]>,
    active_elements: &[bool],
    damage_states: &[ElementDamageState],
    failed_elements: &[usize],
    split_faces: &mut HashSet<(usize, usize)>,
    anchored_nodes: &HashSet<usize>,
    min_normal_alignment: f64,
) -> usize {
    apply_face_splitting_step_with_map(
        mesh,
        u_cur,
        u_prev,
        deformed_positions,
        active_elements,
        damage_states,
        failed_elements,
        split_faces,
        anchored_nodes,
        min_normal_alignment,
    )
    .0
}

// ============================================================================
// Internal Helpers for Stress Extraction & Single-Body Masked Nonlinear Solve
// ============================================================================

/// Extracts effective world-frame Voigt Cauchy stress and equivalent plastic strain for element `e_idx`.
fn extract_element_world_stress_and_ep(
    state: &NonlinearState,
    e_idx: usize,
    is_tet10: bool,
) -> (Vector6<f64>, f64) {
    if is_tet10 {
        let base = e_idx * 4;
        // Pick the most critical Gauss point (highest max(σ_1, σ_vM)) for crack initiation
        let mut best_stress = Vector6::zeros();
        let mut best_score = -1.0_f64;
        let mut max_ep = 0.0_f64;
        for q in 0..4 {
            let gp = &state.gauss_states[base + q];
            let s_loc = Vector6::new(
                gp.stress[0],
                gp.stress[1],
                gp.stress[2],
                gp.stress[3],
                gp.stress[4],
                gp.stress[5],
            );
            let r_arr = state
                .gauss_rotations
                .get(base + q)
                .copied()
                .unwrap_or([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
            let r_mat = Matrix3::new(
                r_arr[0][0], r_arr[0][1], r_arr[0][2],
                r_arr[1][0], r_arr[1][1], r_arr[1][2],
                r_arr[2][0], r_arr[2][1], r_arr[2][2],
            );
            let s_world = rotate_voigt_stress(&r_mat, &s_loc);
            let vm = von_mises_from_voigt(&s_world);
            let (p, _) = crate::damage::principal_stresses_and_directions(&s_world);
            let score = vm.max(p[0].max(0.0));
            if score > best_score {
                best_score = score;
                best_stress = s_world;
            }
            max_ep = max_ep.max(gp.equivalent_plastic_strain);
        }
        (best_stress, max_ep)
    } else {
        let gp = &state.gauss_states[e_idx];
        let s_loc = Vector6::new(
            gp.stress[0],
            gp.stress[1],
            gp.stress[2],
            gp.stress[3],
            gp.stress[4],
            gp.stress[5],
        );
        let r_arr = state
            .gauss_rotations
            .get(e_idx)
            .copied()
            .unwrap_or([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        let r_mat = Matrix3::new(
            r_arr[0][0], r_arr[0][1], r_arr[0][2],
            r_arr[1][0], r_arr[1][1], r_arr[1][2],
            r_arr[2][0], r_arr[2][1], r_arr[2][2],
        );
        let s_world = rotate_voigt_stress(&r_mat, &s_loc);
        (s_world, gp.equivalent_plastic_strain)
    }
}

#[inline]
fn matrix3_to_rows_f64(m: &Matrix3<f64>) -> [[f64; 3]; 3] {
    [
        [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
        [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
        [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
    ]
}

enum SingleBodyElems {
    Tet4(Vec<Tet4Element>),
    Tet10(Vec<Tet10Element>),
}

impl SingleBodyElems {
    fn from_mesh(mesh: &TetrahedralMesh) -> Result<Self, FeaError> {
        if mesh.is_tet10() {
            let mut elems = Vec::with_capacity(mesh.tet10_elements.len());
            for (id, &node_indices) in mesh.tet10_elements.iter().enumerate() {
                let mut pos = [[0.0; 3]; 10];
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

    fn volumes(&self) -> Vec<f64> {
        match self {
            Self::Tet4(v) => v.iter().map(|e| e.volume).collect(),
            Self::Tet10(v) => v.iter().map(|e| e.volume).collect(),
        }
    }

    fn gp_count(&self) -> usize {
        match self {
            Self::Tet4(v) => v.len(),
            Self::Tet10(v) => v.len() * 4,
        }
    }
}

/// Solves a single equilibrium increment of a single-body Co-Rotated $J_2$ mesh with active
/// element masking and Kachanov $(1 - D_{\text{eff}})$ stiffness scaling.
#[allow(clippy::too_many_arguments)]
fn solve_single_body_masked_equilibrium(
    mesh: &TetrahedralMesh,
    elems: &SingleBodyElems,
    params: &J2MaterialParams,
    target_fixed: &HashMap<usize, f64>,
    target_forces: &[[f64; 3]],
    state: &mut NonlinearState,
    active_elements: &[bool],
    stiffness_scale: &[f64],
    opts: NonlinearSolverOptions,
) -> Result<(), FeaError> {
    let n_nodes = mesh.node_count();
    let n_dofs = n_nodes * 3;
    if state.displacements.len() < n_dofs {
        state.displacements.resize(n_dofs, 0.0);
    }

    // Active nodes: referenced by >= 1 active element
    let mut active_nodes = vec![false; n_nodes];
    if mesh.is_tet10() {
        for (e_idx, nodes) in mesh.tet10_elements.iter().enumerate() {
            if active_elements[e_idx] {
                for &nid in nodes {
                    active_nodes[nid] = true;
                }
            }
        }
    } else {
        for (e_idx, elem) in mesh.elements.iter().enumerate() {
            if active_elements[e_idx] {
                for &nid in &elem.node_indices {
                    active_nodes[nid] = true;
                }
            }
        }
    }

    let mut fixed = target_fixed.clone();
    let mut ext_forces = vec![[0.0; 3]; n_nodes];
    for nid in 0..n_nodes {
        if active_nodes[nid] {
            if let Some(&f) = target_forces.get(nid) {
                ext_forces[nid] = f;
            }
        } else {
            for c in 0..3 {
                let d = 3 * nid + c;
                fixed.insert(d, state.displacements[d]);
            }
        }
    }

    let mut global_to_free = vec![None; n_dofs];
    let mut free_to_global = Vec::with_capacity(n_dofs);
    for (d, slot) in global_to_free.iter_mut().enumerate() {
        if !fixed.contains_key(&d) {
            *slot = Some(free_to_global.len());
            free_to_global.push(d);
        }
    }
    let n_free = free_to_global.len();

    let prev_gp = state.gauss_states.clone();
    let mut u_cur = state.displacements.clone();
    for (&d, &val) in &fixed {
        if d < n_dofs {
            u_cur[d] = val;
        }
    }

    type SingleBodyEvalResult = (
        Vec<f64>,
        Option<CsrMatrix>,
        Vec<GaussPointState>,
        Vec<[[f64; 3]; 3]>,
        f64,
    );

    let evaluate = |u: &[f64], assemble_k: bool| -> SingleBodyEvalResult {
        let mut f_int = vec![0.0; n_dofs];
        let mut row_entries: Vec<Vec<(usize, f64)>> = if assemble_k {
            vec![Vec::with_capacity(48); n_free]
        } else {
            Vec::new()
        };
        let mut new_gp = Vec::with_capacity(prev_gp.len());
        let mut new_rots = Vec::with_capacity(prev_gp.len());

        match elems {
            SingleBodyElems::Tet4(list) => {
                for (e_idx, elem) in list.iter().enumerate() {
                    if !active_elements[e_idx] {
                        new_gp.push(prev_gp[e_idx]);
                        new_rots.push([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
                        continue;
                    }
                    let sc = stiffness_scale[e_idx];
                    let mut u_e = Vector12::zeros();
                    let mut dofs = [0usize; 12];
                    for (li, &nid) in elem.node_indices.iter().enumerate() {
                        let d0 = 3 * nid;
                        dofs[3 * li] = d0;
                        dofs[3 * li + 1] = d0 + 1;
                        dofs[3 * li + 2] = d0 + 2;
                        u_e[3 * li] = u[d0];
                        u_e[3 * li + 1] = u[d0 + 1];
                        u_e[3 * li + 2] = u[d0 + 2];
                    }
                    if assemble_k {
                        let (fe, ke, rm, rm_rot) = elem.evaluate_nonlinear(
                            &u_e,
                            &prev_gp[e_idx],
                            params,
                            opts.corotated,
                            opts.include_geometric_stiffness,
                        );
                        new_gp.push(rm.state);
                        new_rots.push(matrix3_to_rows_f64(&rm_rot));
                        for a in 0..12 {
                            f_int[dofs[a]] += sc * fe[a];
                            if let Some(ra) = global_to_free[dofs[a]] {
                                let row = &mut row_entries[ra];
                                for b in 0..12 {
                                    let val = sc * ke[(a, b)];
                                    if val.abs() > 1e-25 {
                                        if let Some(rb) = global_to_free[dofs[b]] {
                                            row.push((rb, val));
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        let (fe, rm, rm_rot) = elem.evaluate_nonlinear_forces_only(
                            &u_e,
                            &prev_gp[e_idx],
                            params,
                            opts.corotated,
                        );
                        new_gp.push(rm.state);
                        new_rots.push(matrix3_to_rows_f64(&rm_rot));
                        for a in 0..12 {
                            f_int[dofs[a]] += sc * fe[a];
                        }
                    }
                }
            }
            SingleBodyElems::Tet10(list) => {
                for (e_idx, elem) in list.iter().enumerate() {
                    let base = e_idx * 4;
                    if !active_elements[e_idx] {
                        for q in 0..4 {
                            new_gp.push(prev_gp[base + q]);
                            new_rots.push([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
                        }
                        continue;
                    }
                    let sc = stiffness_scale[e_idx];
                    let mut u_e = Vector30::zeros();
                    let mut dofs = [0usize; 30];
                    for (li, &nid) in elem.node_indices.iter().enumerate() {
                        let d0 = 3 * nid;
                        dofs[3 * li] = d0;
                        dofs[3 * li + 1] = d0 + 1;
                        dofs[3 * li + 2] = d0 + 2;
                        u_e[3 * li] = u[d0];
                        u_e[3 * li + 1] = u[d0 + 1];
                        u_e[3 * li + 2] = u[d0 + 2];
                    }
                    let prev4 = [
                        prev_gp[base],
                        prev_gp[base + 1],
                        prev_gp[base + 2],
                        prev_gp[base + 3],
                    ];
                    if assemble_k {
                        let (fe, ke, rms, rots) = elem.evaluate_nonlinear(
                            &u_e,
                            &prev4,
                            params,
                            opts.corotated,
                            opts.include_geometric_stiffness,
                        );
                        for q in 0..4 {
                            new_gp.push(rms[q].state);
                            new_rots.push(matrix3_to_rows_f64(&rots[q]));
                        }
                        for a in 0..30 {
                            f_int[dofs[a]] += sc * fe[a];
                            if let Some(ra) = global_to_free[dofs[a]] {
                                let row = &mut row_entries[ra];
                                for b in 0..30 {
                                    let val = sc * ke[(a, b)];
                                    if val.abs() > 1e-25 {
                                        if let Some(rb) = global_to_free[dofs[b]] {
                                            row.push((rb, val));
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        let (fe, rms, rots) = elem.evaluate_nonlinear_forces_only(
                            &u_e,
                            &prev4,
                            params,
                            opts.corotated,
                        );
                        for q in 0..4 {
                            new_gp.push(rms[q].state);
                            new_rots.push(matrix3_to_rows_f64(&rots[q]));
                        }
                        for a in 0..30 {
                            f_int[dofs[a]] += sc * fe[a];
                        }
                    }
                }
            }
        }

        let mut res = vec![0.0; n_free];
        let mut int_norm_sq = 0.0_f64;
        for (nid, f_ext_node) in ext_forces.iter().enumerate().take(n_nodes) {
            for (c, &f_ext_c) in f_ext_node.iter().enumerate() {
                let d = 3 * nid + c;
                int_norm_sq += f_int[d] * f_int[d];
                if let Some(fi) = global_to_free[d] {
                    res[fi] = f_ext_c - f_int[d];
                }
            }
        }
        let k_mat = if assemble_k {
            Some(CsrMatrix::from_row_entries(n_free, n_free, row_entries))
        } else {
            None
        };
        (res, k_mat, new_gp, new_rots, int_norm_sq.sqrt())
    };

    if n_free == 0 {
        let (_, _, new_gp, new_rots, _) = evaluate(&u_cur, false);
        state.displacements = u_cur;
        state.gauss_states = new_gp;
        state.gauss_rotations = new_rots;
        return Ok(());
    }

    let ext_norm = ext_forces
        .iter()
        .map(|f| f[0] * f[0] + f[1] * f[1] + f[2] * f[2])
        .sum::<f64>()
        .sqrt();

    let lin_opts = SolverOptions {
        tolerance: opts.linear_solver.tolerance.min(1e-7),
        max_iterations: opts.linear_solver.max_iterations.max(4000),
    };

    for _iter in 0..opts.max_newton_iterations.max(1) {
        let (res, k_opt, new_gp, new_rots, int_norm) = evaluate(&u_cur, true);
        let res_norm = res.iter().map(|v| v * v).sum::<f64>().sqrt();
        let scale = ext_norm.max(int_norm).max(1.0);
        let tol = (opts.force_tolerance * scale).max(1e-8);

        if res_norm <= tol {
            state.displacements = u_cur;
            state.gauss_states = new_gp;
            state.gauss_rotations = new_rots;
            return Ok(());
        }

        let k_free = k_opt.as_ref().expect("tangent assembled");
        let du_free = solve_spd_with_fallback(k_free, &res, lin_opts)?;

        // Backtracking line search
        let u_base = u_cur.clone();
        let mut best_alpha = 1.0_f64;
        let mut best_res = f64::INFINITY;
        let mut accepted = false;
        for &alpha in &[1.0_f64, 0.5, 0.25, 0.125, 0.05] {
            u_cur.copy_from_slice(&u_base);
            for (fi, &gd) in free_to_global.iter().enumerate() {
                u_cur[gd] += alpha * du_free[fi];
            }
            let (tr_res, _, _, _, _) = evaluate(&u_cur, false);
            let tr_norm = tr_res.iter().map(|v| v * v).sum::<f64>().sqrt();
            if tr_norm < best_res {
                best_res = tr_norm;
                best_alpha = alpha;
            }
            if tr_norm <= (1.0 - 1e-4 * alpha) * res_norm || tr_norm <= tol {
                accepted = true;
                break;
            }
        }
        if !accepted {
            u_cur.copy_from_slice(&u_base);
            for (fi, &gd) in free_to_global.iter().enumerate() {
                u_cur[gd] += best_alpha * du_free[fi];
            }
        }
    }

    let (_, _, new_gp, new_rots, _) = evaluate(&u_cur, false);
    state.displacements = u_cur;
    state.gauss_states = new_gp;
    state.gauss_rotations = new_rots;
    Ok(())
}

// ============================================================================
// Single-Body Local Fracture Solver Entry Point
// ============================================================================

/// Executes a single-body nonlinear Co-Rotated $J_2$ FEA history with local continuum damage
/// ($D \in [0, 1]$), Bažant crack-band energy regularization, `ElementErosion` or `FaceSplitting`
/// crack propagation, and automatic detached-fragment extraction with exact mass & momentum
/// conservation (`P5-T5`).
pub fn solve_nonlinear_with_fracture(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    j2_params: &J2MaterialParams,
    damage_params: &DamageMaterialParams,
    stages: &[NonlinearLoadStage],
    options: FractureSolverOptions,
) -> Result<FractureReport, FeaError> {
    let start_time = Instant::now();
    let part_id_str = part_id.into();
    if mesh.is_empty() {
        return Err(FeaError::EmptyMesh);
    }

    let mut work_mesh = mesh.clone();
    let is_tet10 = work_mesh.is_tet10();
    let mut elems = SingleBodyElems::from_mesh(&work_mesh)?;
    let n_elems = mesh_element_count(&work_mesh);
    let elem_volumes = elems.volumes();

    let mut damage_states: Vec<ElementDamageState> = elem_volumes
        .iter()
        .map(|&v| ElementDamageState::intact(v))
        .collect();
    let mut active_elements = vec![true; n_elems];
    let mut split_faces: HashSet<(usize, usize)> = HashSet::new();
    let mut prev_elem_ep = vec![0.0_f64; n_elems];

    let mut state = NonlinearState::initial(work_mesh.node_count(), elems.gp_count());
    let mut prev_u = vec![0.0_f64; work_mesh.node_count() * 3];
    let mut deformed_positions: Vec<[f64; 3]> =
        work_mesh.nodes.iter().map(|n| n.position).collect();

    let effective_stages: Vec<NonlinearLoadStage> = if stages.is_empty() {
        vec![NonlinearLoadStage::new(BoundaryConditions::new()).with_label("stage_0")]
    } else {
        stages.to_vec()
    };

    // Resolve stage BCs on the initial mesh (before any node duplication)
    let mut resolved_stages: Vec<(ResolvedBoundary, usize, String)> =
        Vec::with_capacity(effective_stages.len());
    for (s_idx, stage) in effective_stages.iter().enumerate() {
        let rb = stage.boundary.resolve(&work_mesh)?;
        let n_sub = stage
            .load_steps
            .unwrap_or(options.nonlinear.initial_load_steps.max(1))
            .max(1);
        let lbl = stage
            .label
            .clone()
            .unwrap_or_else(|| format!("stage_{s_idx}"));
        resolved_stages.push((rb, n_sub, lbl));
    }

    let mut step_reports = Vec::new();
    let mut events = Vec::new();
    let mut detached_fragments = Vec::new();
    let mut conservation_checks = Vec::new();
    let mut next_frag_counter = 0usize;
    let mut total_fracture_energy_j = 0.0_f64;
    let mut total_eroded = 0usize;
    let mut total_split_faces = 0usize;
    let mut global_step_idx = 0usize;

    let mut prev_fixed: HashMap<usize, f64> = HashMap::new();
    let mut prev_forces: Vec<[f64; 3]> = vec![[0.0; 3]; work_mesh.node_count()];

    let dt = options.step_dt_s.max(1e-9);

    for (rb, n_steps, stage_label) in &resolved_stages {
        // Anchored nodes for this stage are nodes with Dirichlet constraints
        let anchored_nodes: HashSet<usize> = rb.fixed_dofs.keys().map(|&d| d / 3).collect();

        for sub in 1..=*n_steps {
            let frac = (sub as f64) / (*n_steps as f64);
            let step_label = format!("{stage_label}_{sub}");

            // Linearly interpolate fixed DOFs and nodal forces from previous stage end
            let mut step_fixed = HashMap::with_capacity(rb.fixed_dofs.len());
            for (&dof, &val_end) in &rb.fixed_dofs {
                let val_start = prev_fixed.get(&dof).copied().unwrap_or(0.0);
                step_fixed.insert(dof, val_start + frac * (val_end - val_start));
            }
            let base_node_count = rb.nodal_forces.len();
            let mut step_forces = vec![[0.0; 3]; work_mesh.node_count()];
            for (nid, sf) in step_forces.iter_mut().enumerate().take(base_node_count) {
                let f0 = prev_forces.get(nid).copied().unwrap_or([0.0; 3]);
                let f1 = rb.nodal_forces[nid];
                *sf = [
                    f0[0] + frac * (f1[0] - f0[0]),
                    f0[1] + frac * (f1[1] - f0[1]),
                    f0[2] + frac * (f1[2] - f0[2]),
                ];
            }

            // Snapshot displacements at step start for velocity estimation
            prev_u.resize(work_mesh.node_count() * 3, 0.0);
            prev_u.copy_from_slice(&state.displacements);

            let mut prop_substeps = 0usize;
            let mut step_detached_count = 0usize;

            for prop_iter in 0..options.max_propagation_substeps.max(1) {
                prop_substeps = prop_iter + 1;

                let stiffness_scale: Vec<f64> = damage_states
                    .iter()
                    .map(|ds| match options.topology_mode {
                        FractureTopologyMode::ElementErosion => {
                            ds.stiffness_scale(damage_params.min_residual_stiffness)
                        }
                        FractureTopologyMode::FaceSplitting => effective_stiffness_factor(
                            ds.damage,
                            damage_params.min_residual_stiffness,
                        ),
                    })
                    .collect();

                solve_single_body_masked_equilibrium(
                    &work_mesh,
                    &elems,
                    j2_params,
                    &step_fixed,
                    &step_forces,
                    &mut state,
                    &active_elements,
                    &stiffness_scale,
                    options.nonlinear,
                )?;

                // Update deformed positions and nodal velocities
                deformed_positions.resize(work_mesh.node_count(), [0.0; 3]);
                for (nid, node) in work_mesh.nodes.iter().enumerate() {
                    let d0 = 3 * nid;
                    deformed_positions[nid] = [
                        node.position[0] + state.displacements[d0],
                        node.position[1] + state.displacements[d0 + 1],
                        node.position[2] + state.displacements[d0 + 2],
                    ];
                }
                let nodal_velocities: Vec<[f64; 3]> = (0..work_mesh.node_count())
                    .map(|nid| {
                        let d0 = 3 * nid;
                        [
                            (state.displacements[d0] - prev_u[d0]) / dt,
                            (state.displacements[d0 + 1] - prev_u[d0 + 1]) / dt,
                            (state.displacements[d0 + 2] - prev_u[d0 + 2]) / dt,
                        ]
                    })
                    .collect();

                let pre_event_active: Vec<usize> = active_elements
                    .iter()
                    .enumerate()
                    .filter_map(|(i, &a)| if a { Some(i) } else { None })
                    .collect();

                // Evaluate continuum damage on all active elements
                let mut newly_failed = Vec::new();
                let mut significant_softening = false;
                for e_idx in 0..n_elems {
                    if !active_elements[e_idx] {
                        continue;
                    }
                    let (s_world, ep) =
                        extract_element_world_stress_and_ep(&state, e_idx, is_tet10);
                    let dep = (ep - prev_elem_ep[e_idx]).max(0.0);
                    prev_elem_ep[e_idx] = ep;

                    let upd = evaluate_element_damage(
                        &damage_states[e_idx],
                        &s_world,
                        ep,
                        dep,
                        elem_volumes[e_idx],
                        damage_params,
                    );
                    damage_states[e_idx] = upd.state;
                    total_fracture_energy_j += upd.delta_fracture_energy_j;
                    if upd.delta_damage > 0.05 {
                        significant_softening = true;
                    }
                    if upd.newly_eroded {
                        newly_failed.push(e_idx);
                    }
                }

                // Topology evolution according to selected mode
                let mut newly_eroded_elems = Vec::new();
                let mut newly_split_count = 0usize;

                match options.topology_mode {
                    FractureTopologyMode::ElementErosion => {
                        for &e_idx in &newly_failed {
                            active_elements[e_idx] = false;
                            newly_eroded_elems.push(e_idx);
                            total_eroded += 1;
                        }
                    }
                    FractureTopologyMode::FaceSplitting => {
                        // In FaceSplitting mode, also allow already-damaged elements (D >= 1)
                        // whose crack plane hasn't finished severing to split their next face.
                        let mut split_candidates = newly_failed.clone();
                        if split_candidates.is_empty() && prop_iter > 0 {
                            for (e_idx, &is_act) in active_elements.iter().enumerate() {
                                if is_act && damage_states[e_idx].eroded {
                                    split_candidates.push(e_idx);
                                }
                            }
                        }
                        // Keep elements active (zero eroded mass!)
                        for &e_idx in &newly_failed {
                            damage_states[e_idx].eroded = true;
                            // Soften element stiffness only along split, keeping mass 100% intact
                        }
                        if !split_candidates.is_empty() {
                            let old_node_count = work_mesh.node_count();
                            newly_split_count = apply_face_splitting_step(
                                &mut work_mesh,
                                &mut state.displacements,
                                &mut prev_u,
                                &mut deformed_positions,
                                &active_elements,
                                &damage_states,
                                &split_candidates,
                                &mut split_faces,
                                &anchored_nodes,
                                options.face_split_normal_alignment,
                            );
                            total_split_faces += newly_split_count;
                            if work_mesh.node_count() != old_node_count {
                                step_forces.resize(work_mesh.node_count(), [0.0; 3]);
                                elems = SingleBodyElems::from_mesh(&work_mesh)?;
                            }
                        }
                    }
                }

                // Detect detached connected components & verify conservation
                let (new_frags, cons_opt) = if !newly_eroded_elems.is_empty()
                    || newly_split_count > 0
                {
                    let vels_now: Vec<[f64; 3]> = (0..work_mesh.node_count())
                        .map(|nid| {
                            let d0 = 3 * nid;
                            [
                                (state.displacements[d0] - prev_u[d0]) / dt,
                                (state.displacements[d0 + 1] - prev_u[d0 + 1]) / dt,
                                (state.displacements[d0 + 2] - prev_u[d0 + 2]) / dt,
                            ]
                        })
                        .collect();
                    partition_fractured_body(
                        0,
                        &part_id_str,
                        global_step_idx,
                        &work_mesh,
                        &deformed_positions,
                        &vels_now,
                        damage_params.density_kg_m3,
                        &mut active_elements,
                        &split_faces,
                        &anchored_nodes,
                        &pre_event_active,
                        &newly_eroded_elems,
                        options.min_fragment_elements,
                        &mut next_frag_counter,
                        Some(&elem_volumes),
                    )
                } else {
                    let _ = nodal_velocities;
                    (Vec::new(), None)
                };

                if !newly_eroded_elems.is_empty() || newly_split_count > 0 || !new_frags.is_empty()
                {
                    let frag_ids: Vec<String> =
                        new_frags.iter().map(|f| f.fragment_id.clone()).collect();
                    step_detached_count += new_frags.len();
                    detached_fragments.extend(new_frags);
                    if let Some(ref c) = cons_opt {
                        conservation_checks.push(c.clone());
                    }
                    events.push(FractureEvent {
                        step_index: global_step_idx,
                        substep_index: prop_iter,
                        body_index: 0,
                        newly_eroded_elements: newly_eroded_elems,
                        newly_split_faces: newly_split_count,
                        detached_fragment_ids: frag_ids,
                        conservation: cons_opt,
                    });
                    // Continue staggered propagation to re-equilibrate the cracked topology!
                    continue;
                }

                if !significant_softening {
                    break;
                }
            }

            // Compute step telemetry over active elements
            let mut max_vm = 0.0_f64;
            let mut max_s1 = 0.0_f64;
            let mut max_ep = 0.0_f64;
            let mut max_d = 0.0_f64;
            let mut active_cnt = 0usize;
            for e_idx in 0..n_elems {
                max_d = max_d.max(damage_states[e_idx].damage);
                max_s1 = max_s1.max(damage_states[e_idx].max_principal_stress_pa);
                if active_elements[e_idx] {
                    active_cnt += 1;
                    let (sw, ep) = extract_element_world_stress_and_ep(&state, e_idx, is_tet10);
                    max_vm = max_vm.max(von_mises_from_voigt(&sw));
                    max_ep = max_ep.max(ep);
                }
            }

            step_reports.push(FractureStepReport {
                step_index: global_step_idx,
                label: step_label,
                propagation_substeps: prop_substeps,
                max_von_mises_pa: max_vm,
                max_principal_stress_pa: max_s1,
                max_equiv_plastic_strain: max_ep,
                max_damage: max_d,
                eroded_element_count: total_eroded,
                split_face_count: total_split_faces,
                active_element_count: active_cnt,
                total_fracture_energy_j,
                newly_detached_fragments: step_detached_count,
            });
            global_step_idx += 1;
        }

        prev_fixed = rb.fixed_dofs.clone();
        prev_forces = rb.nodal_forces.clone();
    }

    let surviving_elems: Vec<usize> = active_elements
        .iter()
        .enumerate()
        .filter_map(|(i, &a)| if a { Some(i) } else { None })
        .collect();
    let final_vels: Vec<[f64; 3]> = (0..work_mesh.node_count())
        .map(|nid| {
            let d0 = 3 * nid;
            [
                (state.displacements[d0] - prev_u[d0]) / dt,
                (state.displacements[d0 + 1] - prev_u[d0 + 1]) / dt,
                (state.displacements[d0 + 2] - prev_u[d0 + 2]) / dt,
            ]
        })
        .collect();

    let surviving_surface_mesh =
        extract_component_surface_mesh(&work_mesh, &deformed_positions, &surviving_elems, None);
    let surviving_kinematics = compute_subset_kinematics(
        &work_mesh,
        &deformed_positions,
        &final_vels,
        &surviving_elems,
        damage_params.density_kg_m3,
        Some(&elem_volumes),
    );

    Ok(FractureReport {
        part_id: part_id_str,
        topology_mode: options.topology_mode,
        steps: step_reports,
        events,
        element_damage: damage_states,
        active_elements,
        surviving_surface_mesh,
        surviving_kinematics,
        detached_fragments,
        conservation_checks,
        final_state: state,
        final_deformed_positions: deformed_positions,
        total_eroded_elements: total_eroded,
        total_split_faces,
        total_fracture_energy_j,
        elapsed_ms: start_time.elapsed().as_secs_f64() * 1000.0,
    })
}

// ============================================================================
// Multi-Body Deformable Contact + Local Fracture Solver Entry Point
// ============================================================================

/// Collects all anchored node indices for body `b` in a contact step (both Dirichlet fixed DOFs
/// and RBE2 [`HubCoupling`] bore/shaft nodes).
fn collect_body_anchored_nodes(
    body: &ContactBody,
    step_spec: &ContactStepSpec,
    body_idx: usize,
) -> Result<HashSet<usize>, FeaError> {
    let mut anchored = HashSet::new();
    let bc = step_spec
        .body_boundaries
        .get(&body_idx)
        .unwrap_or(&body.boundary);
    let resolved_bc = bc.resolve_allowing_unconstrained(&body.mesh)?;
    for &dof in resolved_bc.fixed_dofs.keys() {
        anchored.insert(dof / 3);
    }
    let hubs: &[HubCoupling] = step_spec
        .body_hub_couplings
        .get(&body_idx)
        .map(Vec::as_slice)
        .unwrap_or(&body.hub_couplings);
    for hub in hubs {
        let rh = hub.resolve(&body.mesh)?;
        anchored.extend(rh.node_ids);
    }
    Ok(anchored)
}

/// Propagates duplicated node IDs `(orig_nid, new_nid)` created by `FaceSplitting` on `body_idx`
/// into explicit node lists across contact pair filters, hub couplings, and Dirichlet constraints.
fn propagate_duplicated_nodes_to_contact_specs(
    body_idx: usize,
    duplicated_nodes: &[(usize, usize)],
    pairs: &mut [ContactPairSpec],
    body: &mut ContactBody,
    steps: &mut [ContactStepSpec],
) {
    if duplicated_nodes.is_empty() {
        return;
    }

    let extend_node_list = |nodes: &mut Vec<usize>| {
        let mut existing: HashSet<usize> = nodes.iter().copied().collect();
        for &(orig_nid, new_nid) in duplicated_nodes {
            if existing.contains(&orig_nid) && existing.insert(new_nid) {
                nodes.push(new_nid);
            }
        }
    };

    for pair in pairs.iter_mut() {
        match pair {
            ContactPairSpec::DeformablePair {
                body_a,
                body_b,
                slave_a_nodes,
                slave_b_nodes,
                ..
            } => {
                if *body_a == body_idx {
                    if let Some(ref mut list) = slave_a_nodes {
                        extend_node_list(list);
                    }
                }
                if *body_b == body_idx {
                    if let Some(ref mut list) = slave_b_nodes {
                        extend_node_list(list);
                    }
                }
            }
            ContactPairSpec::DeformableToRigid {
                body_idx: pair_body,
                slave_nodes,
                ..
            } => {
                if *pair_body == body_idx {
                    if let Some(ref mut list) = slave_nodes {
                        extend_node_list(list);
                    }
                }
            }
        }
    }

    let extend_hubs = |hubs: &mut [HubCoupling]| {
        for hub in hubs.iter_mut() {
            let selector = match hub {
                HubCoupling::Revolute { selector, .. }
                | HubCoupling::Prismatic { selector, .. }
                | HubCoupling::Fixed { selector, .. } => selector,
            };
            if let HubNodeSelector::Nodes(ref mut list) = selector {
                extend_node_list(list);
            }
        }
    };

    let extend_bc = |bc: &mut BoundaryConditions| {
        for constraint in &mut bc.dirichlet {
            if let DirichletConstraint::FixedNodes {
                ref mut node_ids, ..
            } = constraint
            {
                extend_node_list(node_ids);
            }
        }
    };

    extend_hubs(&mut body.hub_couplings);
    extend_bc(&mut body.boundary);
    for step in steps.iter_mut() {
        if let Some(hubs) = step.body_hub_couplings.get_mut(&body_idx) {
            extend_hubs(hubs);
        }
        if let Some(bc) = step.body_boundaries.get_mut(&body_idx) {
            extend_bc(bc);
        }
    }
}

/// Solves a multi-body deformable contact mechanics history (`P5-T4`) coupled with local
/// continuum damage ($D \in [0, 1]$), Bažant crack-band energy regularization, element erosion
/// or zero-mass-loss [`FractureTopologyMode::FaceSplitting`] (`P5-T13`), dynamic deformed-surface
/// rebuild on crack opening, and automatic detached-fragment extraction with exact mass and
/// momentum conservation (`P5-T5`).
///
/// `body_damage_params[b]` enables fracture evaluation on body `b` when `Some(...)`, or keeps
/// body `b` undamaged elastoplastic when `None` (e.g., hard steel pinion vs frangible brass/glass gear).
pub fn solve_deformable_contact_with_fracture(
    bodies: &[ContactBody],
    body_damage_params: &[Option<DamageMaterialParams>],
    pairs: &[ContactPairSpec],
    steps: &[ContactStepSpec],
    options: FractureSolverOptions,
) -> Result<FractureContactReport, FeaError> {
    let start_time = Instant::now();
    let nb = bodies.len();
    if nb == 0 {
        return Err(FeaError::EmptyMesh);
    }
    if body_damage_params.len() != nb {
        return Err(FeaError::General(format!(
            "body_damage_params length ({}) must match bodies length ({nb})",
            body_damage_params.len()
        )));
    }

    let mut work_bodies: Vec<ContactBody> = bodies.to_vec();
    let mut work_pairs: Vec<ContactPairSpec> = pairs.to_vec();
    let mut work_steps: Vec<ContactStepSpec> = if steps.is_empty() {
        vec![ContactStepSpec::new("step_0")]
    } else {
        steps.to_vec()
    };

    let mut session = MaskedContactSession::new(&work_bodies, &work_pairs, options.contact)?;
    let elem_volumes: Vec<Vec<f64>> = (0..nb).map(|b| session.element_volumes(b)).collect();

    let mut body_damage: Vec<Vec<ElementDamageState>> = (0..nb)
        .map(|b| {
            elem_volumes[b]
                .iter()
                .map(|&v| ElementDamageState::intact(v))
                .collect()
        })
        .collect();
    let mut body_active: Vec<Vec<bool>> = (0..nb)
        .map(|b| vec![true; mesh_element_count(&work_bodies[b].mesh)])
        .collect();
    let mut body_prev_ep: Vec<Vec<f64>> = (0..nb)
        .map(|b| vec![0.0; mesh_element_count(&work_bodies[b].mesh)])
        .collect();
    let mut body_eroded_counts = vec![0usize; nb];
    let mut body_split_counts = vec![0usize; nb];
    let mut body_split_faces: Vec<HashSet<(usize, usize)>> = (0..nb)
        .map(|_| HashSet::new())
        .collect();
    let mut body_fracture_energy = vec![0.0_f64; nb];

    let mut step_reports = Vec::with_capacity(work_steps.len());
    let mut events = Vec::new();
    let mut detached_fragments = Vec::new();
    let mut conservation_checks = Vec::new();
    let mut next_frag_counter = 0usize;
    let dt = options.step_dt_s.max(1e-9);

    let mut prev_step_u: Vec<Vec<f64>> = (0..nb)
        .map(|b| vec![0.0; work_bodies[b].mesh.node_count() * 3])
        .collect();

    let n_steps = work_steps.len();
    for step_idx in 0..n_steps {
        for (b, prev_u_b) in prev_step_u.iter_mut().enumerate() {
            let u_b = &session.body_states()[b].displacements;
            prev_u_b.resize(u_b.len(), 0.0);
            prev_u_b.copy_from_slice(u_b);
        }

        let mut anchored_per_body = Vec::with_capacity(nb);
        for (b, body) in work_bodies.iter().enumerate() {
            anchored_per_body.push(collect_body_anchored_nodes(body, &work_steps[step_idx], b)?);
        }

        let mut prop_substeps = 0usize;
        let mut step_detached_count = 0usize;
        let mut last_contact_report: Option<ContactStepReport> = None;

        for prop_iter in 0..options.max_propagation_substeps.max(1) {
            prop_substeps = prop_iter + 1;

            let stiffness_scales: Vec<Vec<f64>> = (0..nb)
                .map(|b| {
                    let min_k = body_damage_params[b]
                        .map(|p| p.min_residual_stiffness)
                        .unwrap_or(1e-3);
                    body_damage[b]
                        .iter()
                        .map(|ds| match options.topology_mode {
                            FractureTopologyMode::ElementErosion => ds.stiffness_scale(min_k),
                            FractureTopologyMode::FaceSplitting => {
                                effective_stiffness_factor(ds.damage, min_k)
                            }
                        })
                        .collect()
                })
                .collect();

            let contact_rep = session.solve_step(
                step_idx,
                &work_bodies,
                &work_pairs,
                &work_steps[step_idx],
                options.contact,
                &body_active,
                &stiffness_scales,
                prop_iter == 0,
            )?;
            last_contact_report = Some(contact_rep);

            let mut any_topology_change = false;
            let mut any_significant_softening = false;
            let mut rebuild_body_elements = vec![false; nb];

            for b in 0..nb {
                let Some(dparams) = body_damage_params[b] else {
                    continue;
                };
                let is_tet10 = work_bodies[b].mesh.is_tet10();
                let n_elems = mesh_element_count(&work_bodies[b].mesh);
                let pre_event_active: Vec<usize> = body_active[b]
                    .iter()
                    .enumerate()
                    .filter_map(|(i, &a)| if a { Some(i) } else { None })
                    .collect();

                let mut newly_failed = Vec::new();
                for e_idx in 0..n_elems {
                    if !body_active[b][e_idx] {
                        continue;
                    }
                    let (s_world, ep) = extract_element_world_stress_and_ep(
                        &session.body_states()[b],
                        e_idx,
                        is_tet10,
                    );
                    let dep = (ep - body_prev_ep[b][e_idx]).max(0.0);
                    body_prev_ep[b][e_idx] = ep;

                    let upd = evaluate_element_damage(
                        &body_damage[b][e_idx],
                        &s_world,
                        ep,
                        dep,
                        elem_volumes[b][e_idx],
                        &dparams,
                    );
                    body_damage[b][e_idx] = upd.state;
                    body_fracture_energy[b] += upd.delta_fracture_energy_j;
                    if upd.delta_damage > 0.05 {
                        any_significant_softening = true;
                    }
                    if upd.newly_eroded {
                        newly_failed.push(e_idx);
                    }
                }

                let mut newly_eroded = Vec::new();
                let mut newly_split_count = 0usize;

                match options.topology_mode {
                    FractureTopologyMode::ElementErosion => {
                        for &e_idx in &newly_failed {
                            body_active[b][e_idx] = false;
                            newly_eroded.push(e_idx);
                            body_eroded_counts[b] += 1;
                        }
                    }
                    FractureTopologyMode::FaceSplitting => {
                        let mut split_candidates = newly_failed.clone();
                        if split_candidates.is_empty() && prop_iter > 0 {
                            for (e_idx, &is_act) in body_active[b].iter().enumerate() {
                                if is_act && body_damage[b][e_idx].eroded {
                                    split_candidates.push(e_idx);
                                }
                            }
                        }
                        for &e_idx in &newly_failed {
                            body_damage[b][e_idx].eroded = true;
                        }
                        if !split_candidates.is_empty() {
                            let (split_cnt, duplicated_nodes) = {
                                let (u_cur_b, def_pos_b) =
                                    session.body_displacements_and_positions_mut(b);
                                apply_face_splitting_step_with_map(
                                    &mut work_bodies[b].mesh,
                                    u_cur_b,
                                    &mut prev_step_u[b],
                                    def_pos_b,
                                    &body_active[b],
                                    &body_damage[b],
                                    &split_candidates,
                                    &mut body_split_faces[b],
                                    &anchored_per_body[b],
                                    options.face_split_normal_alignment,
                                )
                            };
                            newly_split_count = split_cnt;
                            body_split_counts[b] += split_cnt;
                            if !duplicated_nodes.is_empty() {
                                rebuild_body_elements[b] = true;
                                propagate_duplicated_nodes_to_contact_specs(
                                    b,
                                    &duplicated_nodes,
                                    &mut work_pairs,
                                    &mut work_bodies[b],
                                    &mut work_steps,
                                );
                            }
                        }
                    }
                }

                if !newly_eroded.is_empty() || newly_split_count > 0 {
                    any_topology_change = true;
                    let u_now = &session.body_states()[b].displacements;
                    let n_nodes = work_bodies[b].mesh.node_count();
                    let nodal_vels: Vec<[f64; 3]> = (0..n_nodes)
                        .map(|nid| {
                            let d0 = 3 * nid;
                            [
                                (u_now[d0] - prev_step_u[b][d0]) / dt,
                                (u_now[d0 + 1] - prev_step_u[b][d0 + 1]) / dt,
                                (u_now[d0 + 2] - prev_step_u[b][d0 + 2]) / dt,
                            ]
                        })
                        .collect();

                    let (new_frags, cons_opt) = partition_fractured_body(
                        b,
                        &work_bodies[b].id,
                        step_idx,
                        &work_bodies[b].mesh,
                        &session.surfaces()[b].current_positions,
                        &nodal_vels,
                        dparams.density_kg_m3,
                        &mut body_active[b],
                        &body_split_faces[b],
                        &anchored_per_body[b],
                        &pre_event_active,
                        &newly_eroded,
                        options.min_fragment_elements,
                        &mut next_frag_counter,
                        Some(&elem_volumes[b]),
                    );

                    let frag_ids: Vec<String> =
                        new_frags.iter().map(|f| f.fragment_id.clone()).collect();
                    step_detached_count += new_frags.len();
                    detached_fragments.extend(new_frags);
                    if let Some(ref c) = cons_opt {
                        conservation_checks.push(c.clone());
                    }
                    events.push(FractureEvent {
                        step_index: step_idx,
                        substep_index: prop_iter,
                        body_index: b,
                        newly_eroded_elements: newly_eroded,
                        newly_split_faces: newly_split_count,
                        detached_fragment_ids: frag_ids,
                        conservation: cons_opt,
                    });
                }
            }

            if any_topology_change {
                session.rebuild_topology_from_masks(
                    &work_bodies,
                    &work_pairs,
                    options.contact,
                    &body_active,
                    &rebuild_body_elements,
                )?;
                continue;
            }
            if !any_significant_softening {
                break;
            }
        }

        let body_max_damage: Vec<f64> = (0..nb)
            .map(|b| {
                body_damage[b]
                    .iter()
                    .map(|ds| ds.damage)
                    .fold(0.0_f64, f64::max)
            })
            .collect();
        let body_active_count: Vec<usize> = (0..nb)
            .map(|b| body_active[b].iter().filter(|&&a| a).count())
            .collect();

        step_reports.push(FractureContactStepReport {
            contact: last_contact_report.expect("at least one contact solve executed"),
            propagation_substeps: prop_substeps,
            body_max_damage,
            body_eroded_count: body_eroded_counts.clone(),
            body_split_face_count: body_split_counts.clone(),
            body_active_count,
            body_fracture_energy_j: body_fracture_energy.clone(),
            newly_detached_fragments: step_detached_count,
        });
    }

    let (body_states, body_deformed_positions) = session.into_states_and_positions();

    let mut body_surviving_meshes = Vec::with_capacity(nb);
    let mut body_surviving_kinematics = Vec::with_capacity(nb);
    for b in 0..nb {
        let surv_elems: Vec<usize> = body_active[b]
            .iter()
            .enumerate()
            .filter_map(|(i, &a)| if a { Some(i) } else { None })
            .collect();
        let density = body_damage_params[b]
            .map(|p| p.density_kg_m3)
            .unwrap_or(7850.0);
        let n_nodes = work_bodies[b].mesh.node_count();
        let u_now = &body_states[b].displacements;
        let vels: Vec<[f64; 3]> = (0..n_nodes)
            .map(|nid| {
                let d0 = 3 * nid;
                [
                    (u_now[d0] - prev_step_u[b][d0]) / dt,
                    (u_now[d0 + 1] - prev_step_u[b][d0 + 1]) / dt,
                    (u_now[d0 + 2] - prev_step_u[b][d0 + 2]) / dt,
                ]
            })
            .collect();
        body_surviving_meshes.push(extract_component_surface_mesh(
            &work_bodies[b].mesh,
            &body_deformed_positions[b],
            &surv_elems,
            None,
        ));
        body_surviving_kinematics.push(compute_subset_kinematics(
            &work_bodies[b].mesh,
            &body_deformed_positions[b],
            &vels,
            &surv_elems,
            density,
            Some(&elem_volumes[b]),
        ));
    }

    Ok(FractureContactReport {
        topology_mode: options.topology_mode,
        steps: step_reports,
        events,
        body_element_damage: body_damage,
        body_active_elements: body_active,
        body_total_split_faces: body_split_counts,
        body_surviving_meshes,
        body_surviving_kinematics,
        body_states,
        body_deformed_positions,
        detached_fragments,
        conservation_checks,
        elapsed_ms: start_time.elapsed().as_secs_f64() * 1000.0,
    })
}

/// Builds a simple structured tetrahedral box mesh `[0, lx] x [0, ly] x [0, lz]` with `(nx, ny, nz)`
/// hexahedral cells subdivided into 5 conforming tetrahedra per cell (shared test/benchmark utility).
#[cfg(test)]
fn build_structured_box_tet4(
    lx: f64,
    ly: f64,
    lz: f64,
    nx: usize,
    ny: usize,
    nz: usize,
) -> TetrahedralMesh {
    use crate::tetra_mesh::TetElement;
    let mut nodes = Vec::with_capacity((nx + 1) * (ny + 1) * (nz + 1));
    let idx = |i: usize, j: usize, k: usize| (k * (ny + 1) + j) * (nx + 1) + i;
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                nodes.push(TetNode {
                    id: idx(i, j, k),
                    position: [
                        lx * (i as f64) / (nx as f64),
                        ly * (j as f64) / (ny as f64),
                        lz * (k as f64) / (nz as f64),
                    ],
                });
            }
        }
    }

    let mut elements = Vec::with_capacity(nx * ny * nz * 5);
    let mut eid = 0usize;
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let n000 = idx(i, j, k);
                let n100 = idx(i + 1, j, k);
                let n010 = idx(i, j + 1, k);
                let n110 = idx(i + 1, j + 1, k);
                let n001 = idx(i, j, k + 1);
                let n101 = idx(i + 1, j, k + 1);
                let n011 = idx(i, j + 1, k + 1);
                let n111 = idx(i + 1, j + 1, k + 1);

                let tets = if (i + j + k) % 2 == 0 {
                    [
                        [n000, n100, n110, n101],
                        [n000, n110, n010, n011],
                        [n000, n001, n101, n011],
                        [n111, n110, n101, n011],
                        [n000, n110, n101, n011],
                    ]
                } else {
                    [
                        [n100, n110, n010, n111],
                        [n000, n100, n010, n001],
                        [n100, n101, n111, n001],
                        [n010, n011, n111, n001],
                        [n100, n010, n111, n001],
                    ]
                };
                for mut tet in tets {
                    let p0 = Vector3::from(nodes[tet[0]].position);
                    let p1 = Vector3::from(nodes[tet[1]].position);
                    let p2 = Vector3::from(nodes[tet[2]].position);
                    let p3 = Vector3::from(nodes[tet[3]].position);
                    if (p1 - p0).cross(&(p2 - p0)).dot(&(p3 - p0)) < 0.0 {
                        tet.swap(2, 3);
                    }
                    elements.push(TetElement {
                        id: eid,
                        node_indices: tet,
                    });
                    eid += 1;
                }
            }
        }
    }

    TetrahedralMesh::new(nodes, elements)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mcp_cad_core::diagnostics::validate_mesh;

    #[test]
    fn test_exact_simplex_kinematics_and_conservation_both_modes() {
        // 6x2x2 bar (lx = 0.06, ly = 0.02, lz = 0.02), density = 8500 kg/m^3
        let mesh = build_structured_box_tet4(0.06, 0.02, 0.02, 6, 2, 2);
        let rho = 8500.0_f64;
        let def_pos: Vec<[f64; 3]> = mesh.nodes.iter().map(|n| n.position).collect();

        // Impose combined rigid-body translation + rotation + shear velocity field
        let v0 = Vector3::new(1.5, -0.8, 0.4);
        let omega = Vector3::new(12.0, -7.0, 25.0);
        let nodal_vels: Vec<[f64; 3]> = def_pos
            .iter()
            .map(|&p| {
                let x = Vector3::from(p);
                let v = v0 + omega.cross(&x) + Vector3::new(0.2 * x.y, -0.1 * x.x, 0.0);
                [v.x, v.y, v.z]
            })
            .collect();

        // Analytical check on intact box mass and inertia
        let all_elems: Vec<usize> = (0..mesh.elements.len()).collect();
        let kin_all = compute_subset_kinematics(&mesh, &def_pos, &nodal_vels, &all_elems, rho, None);
        let expected_mass = 0.06 * 0.02 * 0.02 * rho;
        let expected_izz = (expected_mass / 12.0) * (0.06 * 0.06 + 0.02 * 0.02);
        assert!((kin_all.mass_kg - expected_mass).abs() / expected_mass < 1e-13);
        assert!(
            (kin_all.inertia_tensor_com[2][2] - expected_izz).abs() / expected_izz < 1e-12,
            "Izz = {}, expected {expected_izz}",
            kin_all.inertia_tensor_com[2][2]
        );

        // Mode A: ElementErosion (erode slice x in [0.02, 0.03] -> splits into anchored left + detached right)
        let mut active_erode = vec![true; mesh.elements.len()];
        let mut newly_eroded = Vec::new();
        for (e_idx, elem) in mesh.elements.iter().enumerate() {
            let cx: f64 = elem
                .node_indices
                .iter()
                .map(|&nid| def_pos[nid][0])
                .sum::<f64>()
                / 4.0;
            if cx > 0.02 && cx < 0.03 {
                active_erode[e_idx] = false;
                newly_eroded.push(e_idx);
            }
        }
        let anchored: HashSet<usize> = mesh
            .nodes
            .iter()
            .filter(|n| n.position[0] <= 1e-9)
            .map(|n| n.id)
            .collect();
        let empty_splits = HashSet::new();
        let mut frag_cnt = 0usize;
        let (frags_e, check_e_opt) = partition_fractured_body(
            0,
            "bar",
            1,
            &mesh,
            &def_pos,
            &nodal_vels,
            rho,
            &mut active_erode,
            &empty_splits,
            &anchored,
            &all_elems,
            &newly_eroded,
            1,
            &mut frag_cnt,
            None,
        );
        assert_eq!(frags_e.len(), 1);
        let check_e = check_e_opt.expect("conservation check present");
        assert!(check_e.mass_eroded_kg > 0.0);
        assert!(check_e.mass_rel_error < 1e-13, "err = {}", check_e.mass_rel_error);
        assert!(
            check_e.linear_momentum_rel_error < 1e-13,
            "p_err = {}",
            check_e.linear_momentum_rel_error
        );
        assert!(
            check_e.angular_momentum_rel_error < 1e-13,
            "l_err = {}",
            check_e.angular_momentum_rel_error
        );
        // Watertight mesh check on detached fragment
        let v_report = validate_mesh(&frags_e[0].local_surface_mesh, None);
        assert_eq!(
            v_report.topology.boundary_edges_count, 0,
            "Detached fragment mesh must be watertight"
        );
        assert!(
            v_report.is_valid_solid,
            "Fragment mesh must pass validate_mesh: {v_report:?}"
        );

        // Mode B: FaceSplitting (sever all faces crossing x = 0.03 -> ZERO eroded mass!)
        let mut active_split = vec![true; mesh.elements.len()];
        let mut split_faces = HashSet::new();
        let mut face_owners: HashMap<[usize; 3], Vec<usize>> = HashMap::new();
        for (e_idx, elem) in mesh.elements.iter().enumerate() {
            let [a, b, c, d] = elem.node_indices;
            for fk in [
                sorted_face_key(a, b, c),
                sorted_face_key(a, b, d),
                sorted_face_key(a, c, d),
                sorted_face_key(b, c, d),
            ] {
                face_owners.entry(fk).or_default().push(e_idx);
            }
        }
        for (fk, owners) in &face_owners {
            if owners.len() == 2 {
                let fx = (def_pos[fk[0]][0] + def_pos[fk[1]][0] + def_pos[fk[2]][0]) / 3.0;
                if (fx - 0.03).abs() < 1e-9 {
                    split_faces.insert(sorted_elem_pair(owners[0], owners[1]));
                }
            }
        }
        let (frags_s, check_s_opt) = partition_fractured_body(
            0,
            "bar_split",
            1,
            &mesh,
            &def_pos,
            &nodal_vels,
            rho,
            &mut active_split,
            &split_faces,
            &anchored,
            &all_elems,
            &[],
            1,
            &mut frag_cnt,
            None,
        );
        assert_eq!(frags_s.len(), 1);
        let check_s = check_s_opt.expect("conservation check present");
        assert_eq!(check_s.mass_eroded_kg, 0.0, "FaceSplitting must have ZERO eroded mass");
        assert!(check_s.mass_rel_error < 1e-13);
        assert!(check_s.linear_momentum_rel_error < 1e-13);
        assert!(check_s.angular_momentum_rel_error < 1e-13);
        let v_split = validate_mesh(&frags_s[0].world_surface_mesh, None);
        assert_eq!(v_split.topology.boundary_edges_count, 0);
        assert!(v_split.is_valid_solid);
    }
}
