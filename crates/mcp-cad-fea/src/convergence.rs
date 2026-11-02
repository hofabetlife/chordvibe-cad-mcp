//! Mesh convergence study utilities for Finite Element Analysis (FEA).
//!
//! Evaluates displacement and stress convergence across multiple mesh refinement steps,
//! computes asymptotic stress limits via Richardson extrapolation, and flags potential
//! non-converging stress singularities.

use serde::{Deserialize, Serialize};
use crate::boundary::BoundaryConditions;
use crate::error::FeaError;
use crate::solver::SolverOptions;
use crate::tetra_mesh::{MeshDensity, TetrahedralMesh};
use mcp_cad_core::material::Material;
use mcp_cad_core::part::Part;

/// A single evaluation step in a mesh convergence study.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeshConvergenceStep {
    /// Zero-based step index.
    pub step_index: usize,
    /// Mesh density profile name or custom label.
    pub density_label: String,
    /// Target maximum edge length in meters.
    pub max_edge_length: f64,
    /// Total node count in mesh.
    pub node_count: usize,
    /// Total element count in mesh.
    pub element_count: usize,
    /// Maximum displacement magnitude in meters.
    pub max_displacement_m: f64,
    /// Peak equivalent von Mises stress in Pascals.
    pub max_stress_von_mises_pa: f64,
    /// Relative change in peak stress from previous step (percentage).
    pub stress_change_pct: f64,
    /// Relative change in max displacement from previous step (percentage).
    pub displacement_change_pct: f64,
}

/// Comprehensive report resulting from a mesh convergence study.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeshConvergenceStudyReport {
    /// Benchmark or study title.
    pub title: String,
    /// Sequential refinement steps.
    pub steps: Vec<MeshConvergenceStep>,
    /// Whether the solution has achieved numerical convergence (stress change < tolerance).
    pub is_converged: bool,
    /// Whether an unconverged stress singularity (unbounded stress growth) was detected.
    pub singularity_detected: bool,
    /// Extrapolated asymptotic stress limit in Pascals (via Richardson extrapolation).
    pub asymptotic_stress_pa: f64,
    /// Human-readable engineering summary.
    pub summary: String,
    /// Diagnostic warnings regarding singularities or convergence issues.
    pub warnings: Vec<String>,
}

impl MeshConvergenceStudyReport {
    /// Formats a multi-line engineering summary of the convergence study.
    pub fn summary_text(&self) -> String {
        let mut text = format!(
            "Mesh Convergence Study: {}\nStatus: {} | Singularity Detected: {}\nSteps:\n",
            self.title,
            if self.is_converged { "CONVERGED" } else { "NOT CONVERGED" },
            self.singularity_detected
        );

        for s in &self.steps {
            text.push_str(&format!(
                "  Step #{}: {} ({} nodes, {} elems, h={:.4} m) -> Max Stress: {:.2} MPa (Δ {:.1}%), Max Disp: {:.4} mm (Δ {:.1}%)\n",
                s.step_index,
                s.density_label,
                s.node_count,
                s.element_count,
                s.max_edge_length,
                s.max_stress_von_mises_pa / 1.0e6,
                s.stress_change_pct,
                s.max_displacement_m * 1000.0,
                s.displacement_change_pct,
            ));
        }

        if self.asymptotic_stress_pa > 0.0 {
            text.push_str(&format!(
                "Extrapolated Asymptotic Stress (h -> 0): {:.2} MPa\n",
                self.asymptotic_stress_pa / 1.0e6
            ));
        }

        for w in &self.warnings {
            text.push_str(&format!("WARNING: {}\n", w));
        }

        text
    }
}

/// Runs a mesh convergence study on a CAD `Part` across multiple mesh densities.
pub fn run_mesh_convergence_study_part(
    title: impl Into<String>,
    part: &mut Part,
    boundary: &BoundaryConditions,
    densities: &[MeshDensity],
    tolerance_pct: f64,
    options: Option<SolverOptions>,
) -> Result<MeshConvergenceStudyReport, FeaError> {
    let title_str = title.into();
    let mut steps = Vec::with_capacity(densities.len());
    let mut warnings = Vec::new();

    let mut prev_stress = 0.0;
    let mut prev_disp = 0.0;

    for (idx, &density) in densities.iter().enumerate() {
        let report = crate::analyze_part_adaptive(part, boundary, density, options)?;

        let aabb = part.bounding_box();
        let h = density.target_element_size(&aabb);

        let stress_change = if idx > 0 && prev_stress > 1e-6 {
            ((report.max_stress_von_mises_pa - prev_stress) / prev_stress) * 100.0
        } else {
            0.0
        };

        let disp_change = if idx > 0 && prev_disp > 1e-9 {
            ((report.max_displacement_m - prev_disp) / prev_disp) * 100.0
        } else {
            0.0
        };

        let label = match density {
            MeshDensity::Coarse => "Coarse",
            MeshDensity::Medium => "Medium",
            MeshDensity::Fine => "Fine",
            MeshDensity::Custom { .. } => "Custom",
        }
        .to_string();

        steps.push(MeshConvergenceStep {
            step_index: idx,
            density_label: label,
            max_edge_length: h,
            node_count: report.node_count,
            element_count: report.element_count,
            max_displacement_m: report.max_displacement_m,
            max_stress_von_mises_pa: report.max_stress_von_mises_pa,
            stress_change_pct: stress_change,
            displacement_change_pct: disp_change,
        });

        prev_stress = report.max_stress_von_mises_pa;
        prev_disp = report.max_displacement_m;
    }

    let is_converged = if steps.len() >= 2 {
        let last_change = steps.last().unwrap().stress_change_pct.abs();
        last_change <= tolerance_pct
    } else {
        false
    };

    // Singularity check: if displacements have converged (< 3% change) but stress keeps growing rapidly (> 20% increase)
    let singularity_detected = if steps.len() >= 3 {
        let s1 = steps[1].stress_change_pct;
        let s2 = steps[2].stress_change_pct;
        let d2 = steps[2].displacement_change_pct.abs();

        s1 > 15.0 && s2 > 15.0 && d2 < 5.0
    } else {
        false
    };

    if singularity_detected {
        warnings.push(
            "Unconverged stress singularity detected during mesh refinement: peak stress increases unbounded while displacement has converged. Consider rounding sharp corners, fillet radii, or spreading concentrated boundary forces.".to_string(),
        );
    }

    warnings.push("TET10 quadratic tetrahedra recommended for accurate stress concentration modeling around holes and notches.".to_string());

    // Richardson extrapolation for asymptotic stress: sigma_0 = s3 + (s3 - s2) / (r^p - 1)
    let asymptotic_stress_pa = if steps.len() >= 3 {
        let s2 = steps[steps.len() - 2].max_stress_von_mises_pa;
        let s3 = steps[steps.len() - 1].max_stress_von_mises_pa;
        let diff = s3 - s2;
        if diff.abs() > 1e-6 {
            s3 + diff * 0.5
        } else {
            s3
        }
    } else if !steps.is_empty() {
        steps.last().unwrap().max_stress_von_mises_pa
    } else {
        0.0
    };

    let summary = format!(
        "Convergence study for '{}': {} steps evaluated, status = {}, asymptotic stress = {:.2} MPa",
        title_str,
        steps.len(),
        if is_converged { "CONVERGED" } else { "IN PROGRESS / REFINING" },
        asymptotic_stress_pa / 1.0e6
    );

    Ok(MeshConvergenceStudyReport {
        title: title_str,
        steps,
        is_converged,
        singularity_detected,
        asymptotic_stress_pa,
        summary,
        warnings,
    })
}

/// Runs a mesh convergence study on a `TetrahedralMesh` builder closure across custom edge sizes.
pub fn run_mesh_convergence_study_mesh<F>(
    title: impl Into<String>,
    material: &Material,
    boundary: &BoundaryConditions,
    mesh_generator: F,
    edge_sizes: &[f64],
    tolerance_pct: f64,
    options: Option<SolverOptions>,
) -> Result<MeshConvergenceStudyReport, FeaError>
where
    F: Fn(f64) -> Result<TetrahedralMesh, FeaError>,
{
    let title_str = title.into();
    let mut steps = Vec::with_capacity(edge_sizes.len());
    let mut warnings = Vec::new();

    let mut prev_stress = 0.0;
    let mut prev_disp = 0.0;

    for (idx, &h) in edge_sizes.iter().enumerate() {
        let mesh = mesh_generator(h)?;
        let report = crate::analyze_mesh(&title_str, &mesh, material, boundary, options)?;

        let stress_change = if idx > 0 && prev_stress > 1e-6 {
            ((report.max_stress_von_mises_pa - prev_stress) / prev_stress) * 100.0
        } else {
            0.0
        };

        let disp_change = if idx > 0 && prev_disp > 1e-9 {
            ((report.max_displacement_m - prev_disp) / prev_disp) * 100.0
        } else {
            0.0
        };

        steps.push(MeshConvergenceStep {
            step_index: idx,
            density_label: format!("h={:.4}m", h),
            max_edge_length: h,
            node_count: report.node_count,
            element_count: report.element_count,
            max_displacement_m: report.max_displacement_m,
            max_stress_von_mises_pa: report.max_stress_von_mises_pa,
            stress_change_pct: stress_change,
            displacement_change_pct: disp_change,
        });

        prev_stress = report.max_stress_von_mises_pa;
        prev_disp = report.max_displacement_m;
    }

    let is_converged = if steps.len() >= 2 {
        let last_change = steps.last().unwrap().stress_change_pct.abs();
        last_change <= tolerance_pct
    } else {
        false
    };

    let singularity_detected = if steps.len() >= 3 {
        let s1 = steps[1].stress_change_pct;
        let s2 = steps[2].stress_change_pct;
        let d2 = steps[2].displacement_change_pct.abs();
        s1 > 15.0 && s2 > 15.0 && d2 < 5.0
    } else {
        false
    };

    if singularity_detected {
        warnings.push(
            "Unconverged stress singularity detected during mesh refinement: peak stress increases unbounded while displacement has converged.".to_string(),
        );
    }
    warnings.push("TET10 quadratic tetrahedra recommended for accurate stress concentration modeling around holes and notches.".to_string());

    let asymptotic_stress_pa = if steps.len() >= 3 {
        let s2 = steps[steps.len() - 2].max_stress_von_mises_pa;
        let s3 = steps[steps.len() - 1].max_stress_von_mises_pa;
        let diff = s3 - s2;
        if diff.abs() > 1e-6 {
            s3 + diff * 0.5
        } else {
            s3
        }
    } else if !steps.is_empty() {
        steps.last().unwrap().max_stress_von_mises_pa
    } else {
        0.0
    };

    let summary = format!(
        "Convergence study for '{}': {} steps evaluated, status = {}, asymptotic stress = {:.2} MPa",
        title_str,
        steps.len(),
        if is_converged { "CONVERGED" } else { "IN PROGRESS / REFINING" },
        asymptotic_stress_pa / 1.0e6
    );

    Ok(MeshConvergenceStudyReport {
        title: title_str,
        steps,
        is_converged,
        singularity_detected,
        asymptotic_stress_pa,
        summary,
        warnings,
    })
}
