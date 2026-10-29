//! Post-processing of FEA results: stress recovery, von Mises equivalent stress,
//! safety factor (FoS), and report generation.

use nalgebra::{Matrix6, Vector6};
use serde::{Deserialize, Serialize};
use crate::element::{Tet4Element, Vector12};
use crate::tetra_mesh::TetrahedralMesh;

/// Comprehensive FEA analysis report, serializable to JSON for AI agent introspection
/// and 3D viewer heatmaps.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeaReport {
    /// Target Part ID
    pub part_id: String,
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
    /// Serializes the report into a pretty-formatted JSON string.
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Formats a concise human-readable engineering summary.
    pub fn summary(&self) -> String {
        format!(
            "FEA Report for Part '{}':\n\
             - Verdict: {} (Factor of Safety: {:.2})\n\
             - Max von Mises Stress: {:.2} MPa (Yield: {:.2} MPa)\n\
             - Max Displacement: {:.4} mm (Critical Node: #{})\n\
             - Mesh: {} nodes, {} elements (solved in {:.1} ms)",
            self.part_id,
            self.verdict,
            self.factor_of_safety,
            self.max_stress_von_mises_pa / 1.0e6,
            self.yield_strength_pa / 1.0e6,
            self.max_displacement_m * 1000.0,
            self.critical_node_displacement,
            self.node_count,
            self.element_count,
            self.computation_time_ms,
        )
    }
}

/// Recovers strains, Cauchy stress tensors, and computes smoothed nodal von Mises stresses
/// and safety factors.
pub fn compute_fea_results(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    elements: &[Tet4Element],
    d_matrix: &Matrix6<f64>,
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

    // 2. Compute element stresses and accumulate volume-weighted stresses to nodes
    let mut accum_stresses = vec![Vector6::zeros(); num_nodes];
    let mut accum_volume = vec![0.0; num_nodes];
    let mut max_elem_stress = 0.0;
    let mut critical_elem_node = 0;

    for elem in elements {
        let mut u_elem = Vector12::zeros();
        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            u_elem[3 * local_i] = u_full[3 * global_node];
            u_elem[3 * local_i + 1] = u_full[3 * global_node + 1];
            u_elem[3 * local_i + 2] = u_full[3 * global_node + 2];
        }

        let stress = elem.compute_stress(d_matrix, &u_elem);
        let vm_elem = Tet4Element::von_mises_stress(&stress);
        if vm_elem > max_elem_stress {
            max_elem_stress = vm_elem;
            critical_elem_node = elem.node_indices[0];
        }
        let vol = elem.volume;

        for &n in &elem.node_indices {
            accum_stresses[n] += stress * vol;
            accum_volume[n] += vol;
        }
    }

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
    if max_elem_stress > max_stress {
        max_stress = max_elem_stress;
        critical_node_stress = critical_elem_node;
    }

    // 4. Compute Factor of Safety (FoS) and verdict
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

    // 5. Generate normalized stress scalars for WGPU shaders [0.0..1.0]
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
        node_count: num_nodes,
        element_count: elements.len(),
        max_displacement_m: max_disp,
        max_stress_von_mises_pa: max_stress,
        yield_strength_pa,
        factor_of_safety,
        verdict,
        critical_node_displacement: critical_node_disp,
        critical_node_stress,
        nodal_displacements,
        nodal_von_mises_stresses: nodal_von_mises,
        normalized_stresses,
        computation_time_ms,
    }
}
