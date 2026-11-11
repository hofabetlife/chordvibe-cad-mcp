//! Post-processing of FEA results: stress recovery, von Mises equivalent stress,
//! safety factor (FoS), equilibrium diagnostics, and report generation.

use std::collections::HashSet;
use nalgebra::{Matrix6, Vector3, Vector6};
use serde::{Deserialize, Serialize};
use crate::boundary::ResolvedBoundary;
use crate::element::{Tet10Element, Tet4Element, Vector12, Vector30};
use crate::solver::{compute_nodal_reactions, compute_nodal_reactions_tet10};
use crate::tetra_mesh::TetrahedralMesh;

fn default_confidence() -> String {
    "high".to_string()
}

fn default_true() -> bool {
    true
}

fn default_element_order() -> u8 {
    1
}

/// Tetrahedral element shape quality statistics (normalized Liu-Joe mean ratio in `[0.0, 1.0]`,
/// where `1.0` is an equilateral regular tetrahedron and `0.0` is degenerate).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ElementQualityStats {
    /// Minimum element shape quality across all tetrahedra in `[0.0, 1.0]`
    pub min: f64,
    /// Mean element shape quality across all tetrahedra in `[0.0, 1.0]`
    pub mean: f64,
}

impl Default for ElementQualityStats {
    fn default() -> Self {
        Self { min: 1.0, mean: 1.0 }
    }
}

/// Pre-flight and post-solve FEA verification diagnostics (P3-T2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeaDiagnostics {
    /// True if the input boundary mesh is a closed watertight 2-manifold (`boundary_edges == 0`)
    #[serde(default = "default_true")]
    pub mesh_closed: bool,
    /// Relative force equilibrium error in percent: `||ΣR + ΣF|| / max(||ΣF||, Σ||f_i||, ε) * 100`
    pub equilibrium_error_pct: f64,
    /// Dimensionless relative force equilibrium error: `||ΣR + ΣF|| / max(||ΣF||, Σ||f_i||, ε)`
    #[serde(default)]
    pub equilibrium_relative_error: f64,
    /// Relative moment equilibrium error about the center of mass in percent
    #[serde(default)]
    pub moment_equilibrium_error_pct: f64,
    /// Total applied external force vector `[Fx, Fy, Fz]` in Newtons (N)
    pub applied_total: [f64; 3],
    /// Total support reaction force vector `[Rx, Ry, Rz]` in Newtons (N)
    pub reaction_total: [f64; 3],
    /// Total applied external moment vector `[Mx, My, Mz]` about the center of mass in N·m
    #[serde(default)]
    pub applied_moment_total: [f64; 3],
    /// Total support reaction moment vector `[Mx, My, Mz]` about the center of mass in N·m
    #[serde(default)]
    pub reaction_moment_total: [f64; 3],
    /// True if all 6 3D rigid-body modes (3 translations + 3 rotations) are kinematically constrained
    #[serde(default = "default_true")]
    pub constrained_rigid_body_modes_ok: bool,
    /// True if `critical_node_stress` lies at or within 1 element of a constrained support node
    pub critical_node_near_constraint: bool,
    /// Maximum von Mises stress (in Pa) outside the 1-element constraint singularity zone
    pub max_stress_excluding_constraint_zone: f64,
    /// Maximum von Mises stress (in MPa) outside the 1-element constraint singularity zone
    #[serde(default)]
    pub max_stress_excluding_constraint_zone_mpa: f64,
    /// Tetrahedral element shape quality metrics (`{ min, mean }` in `[0.0, 1.0]`)
    pub element_quality: ElementQualityStats,
    /// Engineering diagnostic notes and warnings (e.g. constraint singularity warnings)
    pub notes: Vec<String>,
}

impl Default for FeaDiagnostics {
    fn default() -> Self {
        Self {
            mesh_closed: true,
            equilibrium_error_pct: 0.0,
            equilibrium_relative_error: 0.0,
            moment_equilibrium_error_pct: 0.0,
            applied_total: [0.0; 3],
            reaction_total: [0.0; 3],
            applied_moment_total: [0.0; 3],
            reaction_moment_total: [0.0; 3],
            constrained_rigid_body_modes_ok: true,
            critical_node_near_constraint: false,
            max_stress_excluding_constraint_zone: 0.0,
            max_stress_excluding_constraint_zone_mpa: 0.0,
            element_quality: ElementQualityStats::default(),
            notes: Vec::new(),
        }
    }
}

/// Comprehensive FEA analysis report, serializable to JSON for AI agent introspection
/// and 3D viewer heatmaps.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeaReport {
    /// Target Part ID
    pub part_id: String,
    /// Finite element polynomial order (`1` = linear `TET4`, `2` = quadratic `TET10`)
    #[serde(default = "default_element_order")]
    pub element_order: u8,
    /// Number of nodes in the tetrahedral mesh
    pub node_count: usize,
    /// Number of elements in the tetrahedral mesh
    pub element_count: usize,
    /// Maximum displacement magnitude across all nodes in meters
    pub max_displacement_m: f64,
    /// Maximum equivalent von Mises stress in Pascals
    pub max_stress_von_mises_pa: f64,
    /// Material yield strength in Pascals
    pub yield_strength_pa: f64,
    /// Minimum Factor of Safety: FoS = yield_strength / max_stress
    pub factor_of_safety: f64,
    /// Safety verdict: "PASS" if FoS >= 1.0, otherwise "FAIL"
    pub verdict: String,
    /// Confidence level in the FEA result: "high", "medium", or "low"
    #[serde(default = "default_confidence")]
    pub confidence: String,
    /// Pre-flight and post-solve equilibrium, singularity, and mesh quality diagnostics
    #[serde(default)]
    pub diagnostics: FeaDiagnostics,
    /// Optional mesh convergence and numerical accuracy assessment (populated when `convergence_study: true`)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub accuracy: Option<crate::convergence::FeaAccuracy>,
    /// Node index exhibiting maximum displacement
    pub critical_node_displacement: usize,
    /// Node index exhibiting maximum von Mises stress
    pub critical_node_stress: usize,
    /// Nodal displacement vectors [ux, uy, uz] in meters
    pub nodal_displacements: Vec<[f64; 3]>,
    /// Nodal smoothed equivalent von Mises stresses in Pascals
    pub nodal_von_mises_stresses: Vec<f64>,
    /// Normalized nodal stress scalars in range [0.0, 1.0] for WGPU shader heatmaps
    pub normalized_stresses: Vec<f32>,
    /// Total solver and recovery wall-clock time in milliseconds
    pub computation_time_ms: f64,
}

impl FeaReport {
    /// Attaches a mesh convergence study accuracy assessment (`P3-T3`) to this `FeaReport`.
    /// If the study verdict is `NOT_CONVERGED` (`!accuracy.converged`), downgrades `confidence`
    /// to `"low"` and appends a diagnostic note to `diagnostics.notes`.
    pub fn with_accuracy(mut self, accuracy: crate::convergence::FeaAccuracy) -> Self {
        if !accuracy.converged {
            self.confidence = "low".to_string();
            let note = format!(
                "Warning [MESH_CONVERGENCE]: Mesh convergence study verdict is {} (Δstress = {:.2}%, Δdisp = {:.2}%). {}",
                accuracy.verdict,
                accuracy.stress_change_pct,
                accuracy.displacement_change_pct,
                accuracy.recommendation
            );
            if !self.diagnostics.notes.contains(&note) {
                self.diagnostics.notes.push(note);
            }
        }
        self.accuracy = Some(accuracy);
        self
    }

    /// Serializes the report into a pretty-formatted JSON string.
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Formats a concise human-readable engineering summary.
    pub fn summary(&self) -> String {
        let elem_tag = if self.element_order == 2 { "TET10" } else { "TET4" };
        let mut s = format!(
            "FEA Report for Part '{}' ({}):\n\
             - Verdict: {} (Factor of Safety: {:.2}, Confidence: {})\n\
             - Max von Mises Stress: {:.2} MPa (Excl. Constraint Zone: {:.2} MPa, Yield: {:.2} MPa)\n\
             - Max Displacement: {:.4} mm (Critical Node: #{})\n\
             - Equilibrium Error: {:.6}%\n\
             - Mesh: {} nodes, {} elements (solved in {:.1} ms)",
            self.part_id,
            elem_tag,
            self.verdict,
            self.factor_of_safety,
            self.confidence,
            self.max_stress_von_mises_pa / 1.0e6,
            self.diagnostics.max_stress_excluding_constraint_zone / 1.0e6,
            self.yield_strength_pa / 1.0e6,
            self.max_displacement_m * 1000.0,
            self.critical_node_displacement,
            self.diagnostics.equilibrium_error_pct,
            self.node_count,
            self.element_count,
            self.computation_time_ms,
        );
        if let Some(ref acc) = self.accuracy {
            s.push_str(&format!(
                "\n- Convergence: {} (Δstress: {:.2}%, Δdisp: {:.2}%)",
                acc.verdict, acc.stress_change_pct, acc.displacement_change_pct
            ));
        }
        s
    }
}

/// Computes the normalized Liu-Joe mean ratio shape quality $q_e \in [0.0, 1.0]$ of a tetrahedron:
/// $q_e = \frac{12 (3 V_e)^{2/3}}{\sum_{0 \le i < j \le 3} \|\mathbf{p}_i - \mathbf{p}_j\|^2}$.
/// Regular equilateral tetrahedron has $q_e = 1.0$; degenerate zero-volume sliver has $q_e = 0.0$.
pub fn compute_tet_element_quality(mesh: &TetrahedralMesh, elem: &Tet4Element) -> f64 {
    if elem.volume <= 0.0 {
        return 0.0;
    }
    let idx = elem.node_indices;
    let p = [
        mesh.nodes[idx[0]].pos_vec(),
        mesh.nodes[idx[1]].pos_vec(),
        mesh.nodes[idx[2]].pos_vec(),
        mesh.nodes[idx[3]].pos_vec(),
    ];
    let mut sum_edge_sq = 0.0;
    for a in 0..4 {
        for b in (a + 1)..4 {
            sum_edge_sq += (p[a] - p[b]).norm_squared();
        }
    }
    if sum_edge_sq <= 1e-30 {
        return 0.0;
    }
    let num = 12.0 * (3.0 * elem.volume).powf(2.0 / 3.0);
    (num / sum_edge_sq).clamp(0.0, 1.0)
}

/// Computes the normalized Liu-Joe mean ratio shape quality $q_e \in [0.0, 1.0]$ of a `Tet10Element`.
pub fn compute_tet10_element_quality(mesh: &TetrahedralMesh, elem: &Tet10Element) -> f64 {
    if elem.volume <= 0.0 {
        return 0.0;
    }
    let idx = elem.node_indices;
    let p = [
        mesh.nodes[idx[0]].pos_vec(),
        mesh.nodes[idx[1]].pos_vec(),
        mesh.nodes[idx[2]].pos_vec(),
        mesh.nodes[idx[3]].pos_vec(),
    ];
    let mut sum_edge_sq = 0.0;
    for a in 0..4 {
        for b in (a + 1)..4 {
            sum_edge_sq += (p[a] - p[b]).norm_squared();
        }
    }
    if sum_edge_sq <= 1e-30 {
        return 0.0;
    }
    let num = 12.0 * (3.0 * elem.volume).powf(2.0 / 3.0);
    (num / sum_edge_sq).clamp(0.0, 1.0)
}

/// Checks whether the Dirichlet constraints in `boundary` eliminate all 6 3D rigid-body modes
/// (3 translations + 3 rotations) by evaluating the $6 \times 6$ kinematic Gram matrix $G^T G$.
fn check_rigid_body_modes_constrained(
    mesh: &TetrahedralMesh,
    boundary: &ResolvedBoundary,
    com: Vector3<f64>,
    l_char: f64,
) -> bool {
    if boundary.fixed_dofs.len() < 6 {
        return false;
    }
    let inv_l = 1.0 / l_char.max(1e-6);
    let mut gtg = Matrix6::<f64>::zeros();

    for &dof in boundary.fixed_dofs.keys() {
        let node_id = dof / 3;
        let axis = dof % 3;
        if node_id >= mesh.nodes.len() {
            continue;
        }
        let mut e = Vector3::<f64>::zeros();
        e[axis] = 1.0;
        let r = (mesh.nodes[node_id].pos_vec() - com) * inv_l;
        let rot = r.cross(&e);
        let row = Vector6::new(e.x, e.y, e.z, rot.x, rot.y, rot.z);
        gtg += row * row.transpose();
    }

    let eigs = gtg.symmetric_eigenvalues();
    let max_eig = eigs.iter().copied().fold(0.0_f64, f64::max);
    let min_eig = eigs.iter().copied().fold(f64::INFINITY, f64::min);
    max_eig > 1e-12 && min_eig > max_eig * 1e-9
}

/// Checks if the exterior boundary triangles of `mesh` form a closed watertight surface (`boundary_edges == 0`).
fn is_tet_mesh_surface_closed(mesh: &TetrahedralMesh) -> bool {
    if mesh.elements.is_empty() {
        return false;
    }
    let surf_tris = mesh.extract_surface_triangles();
    if surf_tris.is_empty() {
        return false;
    }
    let mut edge_counts: std::collections::HashMap<(usize, usize), usize> =
        std::collections::HashMap::with_capacity(surf_tris.len() * 3 / 2);
    for [a, b, c] in surf_tris {
        for (u, v) in [(a, b), (b, c), (c, a)] {
            let key = if u < v { (u, v) } else { (v, u) };
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }
    edge_counts.values().all(|&count| count == 2)
}

/// Recovers strains, Cauchy stress tensors, and computes smoothed nodal von Mises stresses
/// and safety factors (without boundary reaction diagnostics) for linear `TET4` elements.
pub fn compute_fea_results(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    elements: &[Tet4Element],
    d_matrix: &Matrix6<f64>,
    u_full: &[f64],
    yield_strength_pa: f64,
    computation_time_ms: f64,
) -> FeaReport {
    compute_fea_results_with_boundary(
        part_id,
        mesh,
        elements,
        d_matrix,
        None,
        u_full,
        yield_strength_pa,
        computation_time_ms,
    )
}

/// Recovers strains, Cauchy stress tensors, and computes smoothed nodal von Mises stresses
/// and safety factors (without boundary reaction diagnostics) for quadratic `TET10` elements.
pub fn compute_fea_results_tet10(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    elements: &[Tet10Element],
    d_matrix: &Matrix6<f64>,
    u_full: &[f64],
    yield_strength_pa: f64,
    computation_time_ms: f64,
) -> FeaReport {
    compute_fea_results_with_boundary_tet10(
        part_id,
        mesh,
        elements,
        d_matrix,
        None,
        u_full,
        yield_strength_pa,
        computation_time_ms,
    )
}

/// Recovers strains, Cauchy stress tensors, smoothed nodal von Mises stresses,
/// safety factors, support reaction equilibrium, constraint singularity zone metrics,
/// and confidence assessment for linear `TET4` elements.
#[allow(clippy::too_many_arguments)]
pub fn compute_fea_results_with_boundary(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    elements: &[Tet4Element],
    d_matrix: &Matrix6<f64>,
    boundary: Option<&ResolvedBoundary>,
    u_full: &[f64],
    yield_strength_pa: f64,
    computation_time_ms: f64,
) -> FeaReport {
    let num_nodes = mesh.node_count();

    // 1. Extract nodal displacements and find maximum displacement
    let mut nodal_displacements = Vec::with_capacity(num_nodes);
    let mut max_disp = 0.0;
    let mut critical_node_disp = 0;

    for i in 0..num_nodes {
        let ux = u_full[3 * i];
        let uy = u_full[3 * i + 1];
        let uz = u_full[3 * i + 2];
        let mag = (ux * ux + uy * uy + uz * uz).sqrt();

        if mag > max_disp {
            max_disp = mag;
            critical_node_disp = i;
        }
        nodal_displacements.push([ux, uy, uz]);
    }

    // Identify constrained nodes and 1-element constraint singularity zone
    let mut constrained_nodes = HashSet::new();
    if let Some(bc) = boundary {
        for &dof in bc.fixed_dofs.keys() {
            constrained_nodes.insert(dof / 3);
        }
    }
    let mut constraint_zone_nodes = constrained_nodes.clone();
    if !constrained_nodes.is_empty() {
        for elem in elements {
            if elem
                .node_indices
                .iter()
                .any(|n| constrained_nodes.contains(n))
            {
                for &n in &elem.node_indices {
                    constraint_zone_nodes.insert(n);
                }
            }
        }
    }

    // 2. Compute element stresses, element quality, center of mass, and accumulate volume-weighted stresses to nodes
    let mut accum_stresses = vec![Vector6::zeros(); num_nodes];
    let mut accum_volume = vec![0.0; num_nodes];
    let mut elem_vm_stresses = Vec::with_capacity(elements.len());
    let mut max_elem_stress = 0.0;
    let mut max_elem_idx = 0;

    let mut min_quality = if elements.is_empty() { 1.0 } else { f64::INFINITY };
    let mut sum_quality = 0.0;
    let mut weighted_com = Vector3::<f64>::zeros();
    let mut total_vol = 0.0;

    for (idx, elem) in elements.iter().enumerate() {
        let mut u_elem = Vector12::zeros();
        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            u_elem[3 * local_i] = u_full[3 * global_node];
            u_elem[3 * local_i + 1] = u_full[3 * global_node + 1];
            u_elem[3 * local_i + 2] = u_full[3 * global_node + 2];
        }

        let stress = elem.compute_stress(d_matrix, &u_elem);
        let vm_elem = Tet4Element::von_mises_stress(&stress);
        elem_vm_stresses.push(vm_elem);
        if vm_elem > max_elem_stress {
            max_elem_stress = vm_elem;
            max_elem_idx = idx;
        }
        let vol = elem.volume;

        let q = compute_tet_element_quality(mesh, elem);
        if q < min_quality {
            min_quality = q;
        }
        sum_quality += q;

        let c_elem = (mesh.nodes[elem.node_indices[0]].pos_vec()
            + mesh.nodes[elem.node_indices[1]].pos_vec()
            + mesh.nodes[elem.node_indices[2]].pos_vec()
            + mesh.nodes[elem.node_indices[3]].pos_vec())
            * 0.25;
        weighted_com += c_elem * vol;
        total_vol += vol;

        for &n in &elem.node_indices {
            accum_stresses[n] += stress * vol;
            accum_volume[n] += vol;
        }
    }

    let mean_quality = if !elements.is_empty() {
        sum_quality / (elements.len() as f64)
    } else {
        1.0
    };
    if !min_quality.is_finite() {
        min_quality = 1.0;
    }

    let com = if total_vol > 1e-18 {
        weighted_com / total_vol
    } else {
        let aabb = mesh.bounding_box();
        Vector3::from(aabb.center())
    };

    // 3. Compute smoothed nodal stresses, von Mises equivalent, and maximum stress
    let mut nodal_von_mises = Vec::with_capacity(num_nodes);
    let mut max_stress = 0.0;
    let mut critical_node_stress = 0;

    for i in 0..num_nodes {
        let vol = accum_volume[i];
        let smoothed_stress = if vol > 1e-15 {
            accum_stresses[i] / vol
        } else {
            Vector6::zeros()
        };

        let vm = Tet4Element::von_mises_stress(&smoothed_stress);
        if vm > max_stress {
            max_stress = vm;
            critical_node_stress = i;
        }
        nodal_von_mises.push(vm);
    }

    // Ensure peak stress is not underestimated by nodal averaging (conservative engineering)
    if max_elem_stress > max_stress && !elements.is_empty() {
        max_stress = max_elem_stress;
        // Pick the node of the peak element with the highest nodal stress (preferring constrained nodes on ties)
        let peak_nodes = elements[max_elem_idx].node_indices;
        critical_node_stress = *peak_nodes
            .iter()
            .max_by(|&&a, &&b| {
                let va = nodal_von_mises[a];
                let vb = nodal_von_mises[b];
                va.partial_cmp(&vb)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| constrained_nodes.contains(&a).cmp(&constrained_nodes.contains(&b)))
            })
            .unwrap_or(&peak_nodes[0]);
    }

    let critical_node_near_constraint = constraint_zone_nodes.contains(&critical_node_stress);

    // Compute max_stress_excluding_constraint_zone:
    // Maximum stress across all nodes and elements outside the 1-element constraint zone
    let mut max_stress_excl = 0.0_f64;
    let mut found_outside_zone = false;
    for (nid, &vm) in nodal_von_mises.iter().enumerate() {
        if !constraint_zone_nodes.contains(&nid) {
            found_outside_zone = true;
            if vm > max_stress_excl {
                max_stress_excl = vm;
            }
        }
    }
    for (elem_idx, elem) in elements.iter().enumerate() {
        if elem
            .node_indices
            .iter()
            .all(|n| !constraint_zone_nodes.contains(n))
        {
            found_outside_zone = true;
            if elem_vm_stresses[elem_idx] > max_stress_excl {
                max_stress_excl = elem_vm_stresses[elem_idx];
            }
        }
    }
    if !found_outside_zone {
        // Fallback if mesh is only 1 element thick: exclude directly constrained nodes
        for (nid, &vm) in nodal_von_mises.iter().enumerate() {
            if !constrained_nodes.contains(&nid) && vm > max_stress_excl {
                max_stress_excl = vm;
                found_outside_zone = true;
            }
        }
        if !found_outside_zone {
            max_stress_excl = max_stress;
        }
    }

    // 4. Compute static equilibrium diagnostics (reactions R = K*u - f vs applied F)
    let mesh_closed = is_tet_mesh_surface_closed(mesh);
    let dims = mesh.bounding_box().dimensions();
    let l_char = dims[0].max(dims[1]).max(dims[2]).max(1e-6);

    let (
        applied_total,
        reaction_total,
        applied_moment_total,
        reaction_moment_total,
        equilibrium_relative_error,
        equilibrium_error_pct,
        moment_equilibrium_error_pct,
        constrained_rigid_body_modes_ok,
    ) = if let Some(bc) = boundary {
        let reactions = compute_nodal_reactions(mesh, elements, d_matrix, bc, u_full);
        let mut f_sum = Vector3::<f64>::zeros();
        let mut r_sum = Vector3::<f64>::zeros();
        let mut m_f_sum = Vector3::<f64>::zeros();
        let mut m_r_sum = Vector3::<f64>::zeros();
        let mut f_abs_sum = 0.0_f64;

        for (n, &rn_arr) in reactions.iter().enumerate().take(num_nodes) {
            let fn_vec = Vector3::from(bc.nodal_forces[n]);
            let rn_vec = Vector3::from(rn_arr);
            f_sum += fn_vec;
            r_sum += rn_vec;
            f_abs_sum += fn_vec.norm();

            let arm = mesh.nodes[n].pos_vec() - com;
            m_f_sum += arm.cross(&fn_vec);
            m_r_sum += arm.cross(&rn_vec);
        }

        let f_ref = f_sum.norm().max(f_abs_sum).max(1e-12);
        let force_imb = (r_sum + f_sum).norm();
        let eq_rel = force_imb / f_ref;
        let eq_pct = eq_rel * 100.0;

        let m_ref = m_f_sum.norm().max(f_ref * l_char).max(1e-12);
        let mom_imb = (m_r_sum + m_f_sum).norm();
        let mom_pct = (mom_imb / m_ref) * 100.0;

        let rbm_ok = check_rigid_body_modes_constrained(mesh, bc, com, l_char);

        (
            [f_sum.x, f_sum.y, f_sum.z],
            [r_sum.x, r_sum.y, r_sum.z],
            [m_f_sum.x, m_f_sum.y, m_f_sum.z],
            [m_r_sum.x, m_r_sum.y, m_r_sum.z],
            eq_rel,
            eq_pct,
            mom_pct,
            rbm_ok,
        )
    } else {
        (
            [0.0; 3],
            [0.0; 3],
            [0.0; 3],
            [0.0; 3],
            0.0,
            0.0,
            0.0,
            true,
        )
    };

    let mut notes = Vec::new();
    if critical_node_near_constraint {
        notes.push(format!(
            "Warning [CONSTRAINT_SINGULARITY]: Peak von Mises stress ({:.2} MPa at node #{}) occurs within 1 element of constrained support nodes and may be a boundary clamping singularity artifact (max_stress_excluding_constraint_zone = {:.2} MPa).",
            max_stress / 1.0e6,
            critical_node_stress,
            max_stress_excl / 1.0e6
        ));
    }
    if equilibrium_error_pct > 1.0 {
        notes.push(format!(
            "Warning [EQUILIBRIUM_RESIDUAL]: Static force equilibrium error ({:.4}%) exceeds 1.0% threshold.",
            equilibrium_error_pct
        ));
    }
    if !constrained_rigid_body_modes_ok {
        notes.push(
            "Warning [RIGID_BODY_MODES]: Dirichlet boundary conditions do not kinematically constrain all 6 3D rigid-body modes."
                .to_string(),
        );
    }
    if min_quality < 0.10 {
        notes.push(format!(
            "Warning [ELEMENT_QUALITY]: Minimum tetrahedral element quality ({:.3}) is below 0.10; consider refining mesh density.",
            min_quality
        ));
    }

    let confidence = if !mesh_closed
        || !constrained_rigid_body_modes_ok
        || equilibrium_error_pct > 1.0
        || critical_node_near_constraint
        || min_quality < 0.05
    {
        "low".to_string()
    } else if equilibrium_error_pct > 0.01 || min_quality < 0.25 || elements.len() < 200 {
        "medium".to_string()
    } else {
        "high".to_string()
    };

    let diagnostics = FeaDiagnostics {
        mesh_closed,
        equilibrium_error_pct,
        equilibrium_relative_error,
        moment_equilibrium_error_pct,
        applied_total,
        reaction_total,
        applied_moment_total,
        reaction_moment_total,
        constrained_rigid_body_modes_ok,
        critical_node_near_constraint,
        max_stress_excluding_constraint_zone: max_stress_excl,
        max_stress_excluding_constraint_zone_mpa: max_stress_excl / 1.0e6,
        element_quality: ElementQualityStats {
            min: min_quality,
            mean: mean_quality,
        },
        notes,
    };

    // 5. Compute Factor of Safety (FoS) and verdict
    let factor_of_safety = if max_stress > 1e-6 {
        yield_strength_pa / max_stress
    } else {
        999.0
    };

    let verdict = if factor_of_safety >= 1.0 {
        "PASS".to_string()
    } else {
        "FAIL".to_string()
    };

    // 6. Generate normalized stress scalars for WGPU shaders [0.0..1.0]
    let normalized_stresses = if max_stress > 1e-6 {
        nodal_von_mises
            .iter()
            .map(|&s| (s / max_stress).clamp(0.0, 1.0) as f32)
            .collect()
    } else {
        vec![0.0f32; num_nodes]
    };

    FeaReport {
        part_id: part_id.into(),
        element_order: 1,
        node_count: num_nodes,
        element_count: elements.len(),
        max_displacement_m: max_disp,
        max_stress_von_mises_pa: max_stress,
        yield_strength_pa,
        factor_of_safety,
        verdict,
        confidence,
        diagnostics,
        accuracy: None,
        critical_node_displacement: critical_node_disp,
        critical_node_stress,
        nodal_displacements,
        nodal_von_mises_stresses: nodal_von_mises,
        normalized_stresses,
        computation_time_ms,
    }
}

/// Recovers strains, Cauchy stress tensors (at the 4 Gauss integration points and extrapolated
/// to all 10 element nodes), volume-weighted smoothed nodal von Mises stresses, safety factors,
/// support reaction equilibrium, constraint singularity zone metrics, and confidence assessment
/// for quadratic `TET10` elements.
#[allow(clippy::too_many_arguments)]
pub fn compute_fea_results_with_boundary_tet10(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    elements: &[Tet10Element],
    d_matrix: &Matrix6<f64>,
    boundary: Option<&ResolvedBoundary>,
    u_full: &[f64],
    yield_strength_pa: f64,
    computation_time_ms: f64,
) -> FeaReport {
    let num_nodes = mesh.node_count();

    // 1. Extract nodal displacements and find maximum displacement
    let mut nodal_displacements = Vec::with_capacity(num_nodes);
    let mut max_disp = 0.0;
    let mut critical_node_disp = 0;

    for i in 0..num_nodes {
        let ux = u_full[3 * i];
        let uy = u_full[3 * i + 1];
        let uz = u_full[3 * i + 2];
        let mag = (ux * ux + uy * uy + uz * uz).sqrt();

        if mag > max_disp {
            max_disp = mag;
            critical_node_disp = i;
        }
        nodal_displacements.push([ux, uy, uz]);
    }

    // Identify constrained nodes and 1-element constraint singularity zone
    let mut constrained_nodes = HashSet::new();
    if let Some(bc) = boundary {
        for &dof in bc.fixed_dofs.keys() {
            constrained_nodes.insert(dof / 3);
        }
    }
    let mut constraint_zone_nodes = constrained_nodes.clone();
    if !constrained_nodes.is_empty() {
        for elem in elements {
            if elem
                .node_indices
                .iter()
                .any(|n| constrained_nodes.contains(n))
            {
                for &n in &elem.node_indices {
                    constraint_zone_nodes.insert(n);
                }
            }
        }
    }

    // 2. Extrapolate Gauss-point stresses to the 10 element nodes and accumulate volume-weighted nodal stress tensors
    let mut accum_stresses = vec![Vector6::zeros(); num_nodes];
    let mut accum_volume = vec![0.0; num_nodes];
    let mut elem_vm_stresses = Vec::with_capacity(elements.len());
    let mut max_elem_stress = 0.0;
    let mut max_elem_idx = 0;

    let mut min_quality = if elements.is_empty() { 1.0 } else { f64::INFINITY };
    let mut sum_quality = 0.0;
    let mut weighted_com = Vector3::<f64>::zeros();
    let mut total_vol = 0.0;

    for (idx, elem) in elements.iter().enumerate() {
        let mut u_elem = Vector30::zeros();
        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            u_elem[3 * local_i] = u_full[3 * global_node];
            u_elem[3 * local_i + 1] = u_full[3 * global_node + 1];
            u_elem[3 * local_i + 2] = u_full[3 * global_node + 2];
        }

        let gp_stresses = elem.compute_gauss_stresses(d_matrix, &u_elem);
        let vm_elem = gp_stresses
            .iter()
            .map(Tet4Element::von_mises_stress)
            .fold(0.0_f64, f64::max);
        elem_vm_stresses.push(vm_elem);
        if vm_elem > max_elem_stress {
            max_elem_stress = vm_elem;
            max_elem_idx = idx;
        }

        let vol = elem.volume;
        let q = compute_tet10_element_quality(mesh, elem);
        if q < min_quality {
            min_quality = q;
        }
        sum_quality += q;

        let c_elem = (mesh.nodes[elem.node_indices[0]].pos_vec()
            + mesh.nodes[elem.node_indices[1]].pos_vec()
            + mesh.nodes[elem.node_indices[2]].pos_vec()
            + mesh.nodes[elem.node_indices[3]].pos_vec())
            * 0.25;
        weighted_com += c_elem * vol;
        total_vol += vol;

        let nodal_stresses = elem.extrapolate_nodal_stresses(d_matrix, &u_elem);
        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            accum_stresses[global_node] += nodal_stresses[local_i] * vol;
            accum_volume[global_node] += vol;
        }
    }

    let mean_quality = if !elements.is_empty() {
        sum_quality / (elements.len() as f64)
    } else {
        1.0
    };
    if !min_quality.is_finite() {
        min_quality = 1.0;
    }

    let com = if total_vol > 1e-18 {
        weighted_com / total_vol
    } else {
        let aabb = mesh.bounding_box();
        Vector3::from(aabb.center())
    };

    // 3. Compute smoothed nodal stresses, von Mises equivalent, and maximum stress
    let mut nodal_von_mises = Vec::with_capacity(num_nodes);
    let mut max_stress = 0.0;
    let mut critical_node_stress = 0;

    for i in 0..num_nodes {
        let vol = accum_volume[i];
        let smoothed_stress = if vol > 1e-15 {
            accum_stresses[i] / vol
        } else {
            Vector6::zeros()
        };

        let vm = Tet4Element::von_mises_stress(&smoothed_stress);
        if vm > max_stress {
            max_stress = vm;
            critical_node_stress = i;
        }
        nodal_von_mises.push(vm);
    }

    if max_elem_stress > max_stress && !elements.is_empty() {
        max_stress = max_elem_stress;
        let peak_nodes = elements[max_elem_idx].node_indices;
        critical_node_stress = *peak_nodes
            .iter()
            .max_by(|&&a, &&b| {
                let va = nodal_von_mises[a];
                let vb = nodal_von_mises[b];
                va.partial_cmp(&vb)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| constrained_nodes.contains(&a).cmp(&constrained_nodes.contains(&b)))
            })
            .unwrap_or(&peak_nodes[0]);
    }

    let critical_node_near_constraint = constraint_zone_nodes.contains(&critical_node_stress);

    // Compute max_stress_excluding_constraint_zone
    let mut max_stress_excl = 0.0_f64;
    let mut found_outside_zone = false;
    for (nid, &vm) in nodal_von_mises.iter().enumerate() {
        if !constraint_zone_nodes.contains(&nid) {
            found_outside_zone = true;
            if vm > max_stress_excl {
                max_stress_excl = vm;
            }
        }
    }
    for (elem_idx, elem) in elements.iter().enumerate() {
        if elem
            .node_indices
            .iter()
            .all(|n| !constraint_zone_nodes.contains(n))
        {
            found_outside_zone = true;
            if elem_vm_stresses[elem_idx] > max_stress_excl {
                max_stress_excl = elem_vm_stresses[elem_idx];
            }
        }
    }
    if !found_outside_zone {
        for (nid, &vm) in nodal_von_mises.iter().enumerate() {
            if !constrained_nodes.contains(&nid) && vm > max_stress_excl {
                max_stress_excl = vm;
                found_outside_zone = true;
            }
        }
        if !found_outside_zone {
            max_stress_excl = max_stress;
        }
    }

    // 4. Compute static equilibrium diagnostics (reactions R = K*u - f vs applied F)
    let mesh_closed = is_tet_mesh_surface_closed(mesh);
    let dims = mesh.bounding_box().dimensions();
    let l_char = dims[0].max(dims[1]).max(dims[2]).max(1e-6);

    let (
        applied_total,
        reaction_total,
        applied_moment_total,
        reaction_moment_total,
        equilibrium_relative_error,
        equilibrium_error_pct,
        moment_equilibrium_error_pct,
        constrained_rigid_body_modes_ok,
    ) = if let Some(bc) = boundary {
        let reactions = compute_nodal_reactions_tet10(mesh, elements, d_matrix, bc, u_full);
        let mut f_sum = Vector3::<f64>::zeros();
        let mut r_sum = Vector3::<f64>::zeros();
        let mut m_f_sum = Vector3::<f64>::zeros();
        let mut m_r_sum = Vector3::<f64>::zeros();
        let mut f_abs_sum = 0.0_f64;

        for (n, &rn_arr) in reactions.iter().enumerate().take(num_nodes) {
            let fn_vec = Vector3::from(bc.nodal_forces[n]);
            let rn_vec = Vector3::from(rn_arr);
            f_sum += fn_vec;
            r_sum += rn_vec;
            f_abs_sum += fn_vec.norm();

            let arm = mesh.nodes[n].pos_vec() - com;
            m_f_sum += arm.cross(&fn_vec);
            m_r_sum += arm.cross(&rn_vec);
        }

        let f_ref = f_sum.norm().max(f_abs_sum).max(1e-12);
        let force_imb = (r_sum + f_sum).norm();
        let eq_rel = force_imb / f_ref;
        let eq_pct = eq_rel * 100.0;

        let m_ref = m_f_sum.norm().max(f_ref * l_char).max(1e-12);
        let mom_imb = (m_r_sum + m_f_sum).norm();
        let mom_pct = (mom_imb / m_ref) * 100.0;

        let rbm_ok = check_rigid_body_modes_constrained(mesh, bc, com, l_char);

        (
            [f_sum.x, f_sum.y, f_sum.z],
            [r_sum.x, r_sum.y, r_sum.z],
            [m_f_sum.x, m_f_sum.y, m_f_sum.z],
            [m_r_sum.x, m_r_sum.y, m_r_sum.z],
            eq_rel,
            eq_pct,
            mom_pct,
            rbm_ok,
        )
    } else {
        (
            [0.0; 3],
            [0.0; 3],
            [0.0; 3],
            [0.0; 3],
            0.0,
            0.0,
            0.0,
            true,
        )
    };

    let mut notes = Vec::new();
    if critical_node_near_constraint {
        notes.push(format!(
            "Warning [CONSTRAINT_SINGULARITY]: Peak von Mises stress ({:.2} MPa at node #{}) occurs within 1 element of constrained support nodes and may be a boundary clamping singularity artifact (max_stress_excluding_constraint_zone = {:.2} MPa).",
            max_stress / 1.0e6,
            critical_node_stress,
            max_stress_excl / 1.0e6
        ));
    }
    if equilibrium_error_pct > 1.0 {
        notes.push(format!(
            "Warning [EQUILIBRIUM_RESIDUAL]: Static force equilibrium error ({:.4}%) exceeds 1.0% threshold.",
            equilibrium_error_pct
        ));
    }
    if !constrained_rigid_body_modes_ok {
        notes.push(
            "Warning [RIGID_BODY_MODES]: Dirichlet boundary conditions do not kinematically constrain all 6 3D rigid-body modes."
                .to_string(),
        );
    }
    if min_quality < 0.10 {
        notes.push(format!(
            "Warning [ELEMENT_QUALITY]: Minimum tetrahedral element quality ({:.3}) is below 0.10; consider refining mesh density.",
            min_quality
        ));
    }

    let confidence = if !mesh_closed
        || !constrained_rigid_body_modes_ok
        || equilibrium_error_pct > 1.0
        || critical_node_near_constraint
        || min_quality < 0.05
    {
        "low".to_string()
    } else if equilibrium_error_pct > 0.01 || min_quality < 0.25 || elements.len() < 200 {
        "medium".to_string()
    } else {
        "high".to_string()
    };

    let diagnostics = FeaDiagnostics {
        mesh_closed,
        equilibrium_error_pct,
        equilibrium_relative_error,
        moment_equilibrium_error_pct,
        applied_total,
        reaction_total,
        applied_moment_total,
        reaction_moment_total,
        constrained_rigid_body_modes_ok,
        critical_node_near_constraint,
        max_stress_excluding_constraint_zone: max_stress_excl,
        max_stress_excluding_constraint_zone_mpa: max_stress_excl / 1.0e6,
        element_quality: ElementQualityStats {
            min: min_quality,
            mean: mean_quality,
        },
        notes,
    };

    // 5. Compute Factor of Safety (FoS) and verdict
    let factor_of_safety = if max_stress > 1e-6 {
        yield_strength_pa / max_stress
    } else {
        999.0
    };

    let verdict = if factor_of_safety >= 1.0 {
        "PASS".to_string()
    } else {
        "FAIL".to_string()
    };

    // 6. Generate normalized stress scalars for WGPU shaders [0.0..1.0]
    let normalized_stresses = if max_stress > 1e-6 {
        nodal_von_mises
            .iter()
            .map(|&s| (s / max_stress).clamp(0.0, 1.0) as f32)
            .collect()
    } else {
        vec![0.0f32; num_nodes]
    };

    FeaReport {
        part_id: part_id.into(),
        element_order: 2,
        node_count: num_nodes,
        element_count: elements.len(),
        max_displacement_m: max_disp,
        max_stress_von_mises_pa: max_stress,
        yield_strength_pa,
        factor_of_safety,
        verdict,
        confidence,
        diagnostics,
        accuracy: None,
        critical_node_displacement: critical_node_disp,
        critical_node_stress,
        nodal_displacements,
        nodal_von_mises_stresses: nodal_von_mises,
        normalized_stresses,
        computation_time_ms,
    }
}

