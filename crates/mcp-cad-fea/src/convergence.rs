//! Mesh convergence study utilities for Finite Element Analysis (FEA).
//!
//! Evaluates displacement and stress convergence across multiple mesh refinement steps,
//! computes asymptotic stress and displacement limits via Richardson extrapolation when
//! mathematical applicability conditions hold, and flags non-converging stress singularities,
//! grid clamping saturation, and Cartesian voxel staircase artifacts on curved boundaries.

use std::time::Instant;
use serde::{Deserialize, Serialize};
use mcp_cad_core::material::Material;
use mcp_cad_core::part::Part;
use mcp_cad_core::topology::{FaceKind, TriangleMesh};
use crate::boundary::BoundaryConditions;
use crate::error::FeaError;
use crate::result::FeaReport;
use crate::solver::SolverOptions;
use crate::tetra_mesh::{MeshDensity, TetrahedralMesh};

/// Maximum relative change in peak displacement (%) between the last two mesh levels
/// to consider the displacement field converged (`P3-T3`: `< 5.0%`).
pub const CONVERGENCE_DISP_TOLERANCE_PCT: f64 = 5.0;

/// Maximum relative change in peak von Mises stress (%) between the last two mesh levels
/// to consider the stress field converged (`P3-T3`: `< 10.0%`).
pub const CONVERGENCE_STRESS_TOLERANCE_PCT: f64 = 10.0;

/// Minimum effective 3D linear mesh refinement ratio `r_eff = (N_last / N_prev)^(1/3)`
/// required between the last two levels (`>= 1.25`).
///
/// Prevents false `CONVERGED` verdicts when the Cartesian voxel mesher (`TetrahedralMesh::from_mesh`)
/// hits the 60-cell axis ceiling on one or two dominant dimensions (for example, on the
/// `600 x 200 x 10 mm` perforated plate where both `Medium` and `Fine` clamp `(nx, ny)` to `(60, 60)`
/// and only refine thickness `nz: 4 -> 6`, yielding `r_eff = 1.145 < 1.25`).
pub const MIN_EFFECTIVE_REFINEMENT_RATIO: f64 = 1.25;

/// Peak-to-mean von Mises stress concentration ratio threshold (`max_stress / mean_stress > 1.50`)
/// above which an un-snapped Cartesian voxel mesh on a curved or non-axis-aligned surface is
/// flagged as suffering from staircase boundary errors (`docs/FEA_ACCURACY.md` §2.3–2.4).
pub const CURVED_STAIRCASE_STRESS_RATIO_THRESHOLD: f64 = 1.50;

/// A single evaluation step in a mesh convergence study.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshConvergenceStep {
    /// Zero-based step index.
    pub step_index: usize,
    /// Canonical lowercase density identifier (`"coarse"`, `"medium"`, `"fine"`, `"custom"`).
    #[serde(default)]
    pub density: String,
    /// Mesh density profile name or custom label (`"Coarse"`, `"Medium"`, `"Fine"`, etc.).
    pub density_label: String,
    /// Target maximum edge length in meters.
    pub max_edge_length: f64,
    /// Total node count in mesh.
    pub node_count: usize,
    /// Total element count in mesh.
    pub element_count: usize,
    /// Maximum displacement magnitude in meters.
    pub max_displacement_m: f64,
    /// Maximum displacement magnitude in millimeters.
    #[serde(default)]
    pub max_displacement_mm: f64,
    /// Peak equivalent von Mises stress in Pascals.
    pub max_stress_von_mises_pa: f64,
    /// Peak equivalent von Mises stress in Megapascals.
    #[serde(default)]
    pub max_von_mises_mpa: f64,
    /// Relative change in peak stress from previous step (percentage).
    pub stress_change_pct: f64,
    /// Relative change in max displacement from previous step (percentage).
    pub displacement_change_pct: f64,
    /// Wall-clock computation time (meshing + solve + post-processing) for this level in milliseconds.
    #[serde(default)]
    pub computation_time_ms: f64,
}

/// Mesh convergence and numerical accuracy assessment block attached to `FeaReport`
/// and returned by `run_fea_analysis` when `convergence_study: true` is set (`P3-T3`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeaAccuracy {
    /// True if all convergence criteria are satisfied (`verdict == "CONVERGED"`).
    pub converged: bool,
    /// Convergence verdict: `"CONVERGED"` or `"NOT_CONVERGED"`.
    pub verdict: String,
    /// Relative change in peak von Mises stress between the last two evaluated levels (%).
    pub stress_change_pct: f64,
    /// Relative change in maximum displacement between the last two evaluated levels (%).
    pub displacement_change_pct: f64,
    /// Effective 3D linear refinement ratio `r_eff = (N_last / N_prev)^(1/3)` between the last two levels.
    #[serde(default)]
    pub effective_refinement_ratio: f64,
    /// Whether an unconverged stress singularity (or peak stress at a constrained boundary) was detected.
    #[serde(default)]
    pub singularity_detected: bool,
    /// Whether a curved/non-axial boundary with stress concentration on an un-snapped Cartesian voxel staircase was detected.
    #[serde(default)]
    pub voxel_staircase_detected: bool,
    /// Whether 60-cell voxel axis clamping prevented adequate 3D mesh refinement (`r_eff < 1.25`).
    #[serde(default)]
    pub grid_clamped_detected: bool,
    /// True if asymptotic Richardson extrapolation is mathematically applicable on the evaluated levels.
    #[serde(default)]
    pub richardson_applicable: bool,
    /// Extrapolated asymptotic von Mises stress in MPa (`h -> 0`) via Richardson extrapolation, if applicable.
    #[serde(default)]
    pub richardson_stress_mpa: Option<f64>,
    /// Extrapolated asymptotic maximum displacement in mm (`h -> 0`) via Richardson extrapolation, if applicable.
    #[serde(default)]
    pub richardson_displacement_mm: Option<f64>,
    /// True if the convergence study was stopped early because `max_seconds` was exceeded.
    #[serde(default)]
    pub partial_result: bool,
    /// Optional time budget in seconds configured for the convergence study.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_seconds: Option<f64>,
    /// Total wall-clock time spent across all evaluated convergence levels in milliseconds.
    #[serde(default)]
    pub total_study_time_ms: f64,
    /// Per-level metrics (element count, max stress, max displacement, computation time).
    #[serde(default)]
    pub levels: Vec<MeshConvergenceStep>,
    /// Actionable engineering recommendation based on convergence behavior and mesh diagnostics.
    pub recommendation: String,
}

/// Comprehensive report resulting from a mesh convergence study.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshConvergenceStudyReport {
    /// Benchmark or study title.
    pub title: String,
    /// Sequential refinement steps.
    pub steps: Vec<MeshConvergenceStep>,
    /// Whether the solution has achieved numerical convergence (`verdict == "CONVERGED"`).
    pub is_converged: bool,
    /// Convergence verdict string: `"CONVERGED"` or `"NOT_CONVERGED"`.
    pub verdict: String,
    /// Whether an unconverged stress singularity (unbounded stress growth) was detected.
    pub singularity_detected: bool,
    /// Whether a curved/non-axial boundary with stress concentration on an un-snapped Cartesian voxel staircase was detected.
    #[serde(default)]
    pub voxel_staircase_detected: bool,
    /// Whether 60-cell voxel axis clamping prevented adequate 3D mesh refinement (`r_eff < 1.25`).
    #[serde(default)]
    pub grid_clamped_detected: bool,
    /// Effective 3D linear refinement ratio `r_eff = (N_last / N_prev)^(1/3)` between the last two levels.
    #[serde(default)]
    pub effective_refinement_ratio: f64,
    /// True if Richardson extrapolation applicability conditions hold.
    #[serde(default)]
    pub richardson_applicable: bool,
    /// Extrapolated asymptotic stress limit in Pascals (via Richardson extrapolation, or last step if inapplicable).
    pub asymptotic_stress_pa: f64,
    /// Extrapolated asymptotic von Mises stress in MPa (`Some` only when `richardson_applicable == true`).
    #[serde(default)]
    pub richardson_stress_mpa: Option<f64>,
    /// Extrapolated asymptotic displacement in mm (`Some` only when `richardson_applicable == true`).
    #[serde(default)]
    pub richardson_displacement_mm: Option<f64>,
    /// True if the study terminated early due to `max_seconds` timeout.
    #[serde(default)]
    pub partial_result: bool,
    /// Configured time limit in seconds, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_seconds: Option<f64>,
    /// Total wall-clock time of the study in milliseconds.
    #[serde(default)]
    pub total_study_time_ms: f64,
    /// Actionable engineering recommendation.
    pub recommendation: String,
    /// Human-readable engineering summary.
    pub summary: String,
    /// Diagnostic warnings regarding singularities, staircase boundaries, or convergence issues.
    pub warnings: Vec<String>,
}

impl MeshConvergenceStudyReport {
    /// Converts this study report into the compact `FeaAccuracy` block embedded in `FeaReport`.
    pub fn to_fea_accuracy(&self) -> FeaAccuracy {
        let (stress_change_pct, displacement_change_pct) = if self.steps.len() >= 2 {
            let last = self.steps.last().unwrap();
            (last.stress_change_pct, last.displacement_change_pct)
        } else {
            (0.0, 0.0)
        };

        FeaAccuracy {
            converged: self.is_converged,
            verdict: self.verdict.clone(),
            stress_change_pct,
            displacement_change_pct,
            effective_refinement_ratio: self.effective_refinement_ratio,
            singularity_detected: self.singularity_detected,
            voxel_staircase_detected: self.voxel_staircase_detected,
            grid_clamped_detected: self.grid_clamped_detected,
            richardson_applicable: self.richardson_applicable,
            richardson_stress_mpa: self.richardson_stress_mpa,
            richardson_displacement_mm: self.richardson_displacement_mm,
            partial_result: self.partial_result,
            max_seconds: self.max_seconds,
            total_study_time_ms: self.total_study_time_ms,
            levels: self.steps.clone(),
            recommendation: self.recommendation.clone(),
        }
    }

    /// Formats a multi-line engineering summary of the convergence study.
    pub fn summary_text(&self) -> String {
        let mut text = format!(
            "Mesh Convergence Study: {}\nStatus: {} | Partial: {} | Singularity Detected: {}\nSteps:\n",
            self.title,
            self.verdict,
            self.partial_result,
            self.singularity_detected
        );

        for s in &self.steps {
            text.push_str(&format!(
                "  Step #{}: {} ({} nodes, {} elems, h={:.4} m, {:.1} ms) -> Max Stress: {:.2} MPa (Δ {:.2}%), Max Disp: {:.4} mm (Δ {:.2}%)\n",
                s.step_index,
                s.density_label,
                s.node_count,
                s.element_count,
                s.max_edge_length,
                s.computation_time_ms,
                s.max_stress_von_mises_pa / 1.0e6,
                s.stress_change_pct,
                s.max_displacement_m * 1000.0,
                s.displacement_change_pct,
            ));
        }

        if let Some(r_stress) = self.richardson_stress_mpa {
            text.push_str(&format!(
                "Extrapolated Asymptotic Stress (h -> 0): {:.2} MPa\n",
                r_stress
            ));
        }
        if let Some(r_disp) = self.richardson_displacement_mm {
            text.push_str(&format!(
                "Extrapolated Asymptotic Displacement (h -> 0): {:.4} mm\n",
                r_disp
            ));
        }

        text.push_str(&format!("Recommendation: {}\n", self.recommendation));

        for w in &self.warnings {
            text.push_str(&format!("WARNING: {}\n", w));
        }

        text
    }
}

/// Checks whether a surface `TriangleMesh` contains curved or non-axis-aligned faces
/// where a Cartesian voxel grid produces a stair-stepped (`staircase`) boundary.
pub fn has_curved_or_non_axial_surface(mesh: &TriangleMesh) -> bool {
    if mesh.faces.iter().any(|f| f.kind != FaceKind::Plane) {
        return true;
    }
    for tri in &mesh.indices {
        let v0 = mesh.vertices[tri[0] as usize].pos_vec();
        let v1 = mesh.vertices[tri[1] as usize].pos_vec();
        let v2 = mesh.vertices[tri[2] as usize].pos_vec();
        let n = (v1 - v0).cross(&(v2 - v0));
        let len = n.norm();
        if len > 1e-15 {
            let un = n / len;
            let max_axis_comp = un.x.abs().max(un.y.abs()).max(un.z.abs());
            if max_axis_comp < 0.999 {
                return true;
            }
        }
    }
    false
}

/// Computes Richardson extrapolation for a sequence of scalar quantities `vals` evaluated on
/// 3D meshes with `elem_counts` elements, accounting for non-uniform refinement ratios
/// $r_{12} = (N_2 / N_1)^{1/3}$ and $r_{23} = (N_3 / N_2)^{1/3}$.
///
/// ### Mathematical Applicability Conditions (`P3-T3`)
/// 1. At least 2 levels completed without timeout, and the surface boundary is not a
///    curved staircase stress concentrator (`allow_extrapolation == true`).
/// 2. Effective 3D linear refinement ratio on the last step satisfies
///    $r_{23} = (N_{\text{last}} / N_{\text{prev}})^{1/3} \ge \text{MIN\_EFFECTIVE\_REFINEMENT\_RATIO}$ (`1.25`).
/// 3. For $\ge 3$ levels, differences $\Delta_{12} = f_2 - f_1$ and $\Delta_{23} = f_3 - f_2$ must be
///    **strictly monotonic of the same sign** ($\Delta_{12} \cdot \Delta_{23} > 0$) and **decreasing in
///    magnitude** ($0 < |\Delta_{23}| < |\Delta_{12}|$, confirming asymptotic range and absence of
///    unbounded singularity).
/// 4. The observed convergence order $p \in [1.0, 2.5]$ is solved from the exact two-ratio relation
///    $r_{23}^p \frac{r_{12}^p - 1}{r_{23}^p - 1} = \frac{|\Delta_{12}|}{|\Delta_{23}|}$, and
///    $f_{\text{extrap}} = f_{\text{last}} + \frac{\Delta_{23}}{r_{23}^p - 1}$.
fn compute_richardson_value(
    vals: &[f64],
    elem_counts: &[usize],
    allow_extrapolation: bool,
) -> Option<f64> {
    if !allow_extrapolation || vals.len() < 2 || elem_counts.len() != vals.len() {
        return None;
    }

    let n = vals.len();
    let f_prev = vals[n - 2];
    let f_last = vals[n - 1];
    let e_prev = elem_counts[n - 2].max(1) as f64;
    let e_last = elem_counts[n - 1].max(1) as f64;
    let r_last = (e_last / e_prev).cbrt();

    if r_last < MIN_EFFECTIVE_REFINEMENT_RATIO {
        return None;
    }

    let delta_last = f_last - f_prev;
    // If already converged within 0.2%, return f_last directly as the asymptotic value
    if delta_last.abs() <= 0.002 * f_prev.abs().max(1e-12) {
        return Some(f_last);
    }

    if n >= 3 {
        let f_first = vals[n - 3];
        let e_first = elem_counts[n - 3].max(1) as f64;
        let r_prev = (e_prev / e_first).cbrt();
        let delta_prev = f_prev - f_first;

        // Require same sign (monotonicity) and shrinking step increments (asymptotic convergence)
        if delta_prev * delta_last <= 0.0 || delta_last.abs() >= delta_prev.abs() || r_prev <= 1.05 {
            return None;
        }

        let target_ratio = delta_prev.abs() / delta_last.abs();
        let phi = |p: f64| -> f64 {
            let num = r_last.powf(p) * (r_prev.powf(p) - 1.0);
            let den = (r_last.powf(p) - 1.0).max(1e-12);
            num / den
        };

        // Bisection for observed order p in [1.0, 2.5] (linear TET4 theoretical order p >= 1.0)
        let mut lo = 1.0_f64;
        let mut hi = 2.5_f64;
        let p = if target_ratio <= phi(lo) {
            lo
        } else if target_ratio >= phi(hi) {
            hi
        } else {
            for _ in 0..32 {
                let mid = 0.5 * (lo + hi);
                if phi(mid) < target_ratio {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            0.5 * (lo + hi)
        };

        let denom = (r_last.powf(p) - 1.0).max(1e-9);
        Some(f_last + delta_last / denom)
    } else {
        // 2-level fallback using theoretical linear TET4 convergence order p = 1.0
        let denom = (r_last - 1.0).max(1e-9);
        Some(f_last + delta_last / denom)
    }
}

/// Evaluates convergence criteria, Richardson extrapolation, and engineering recommendations
/// from the collected `steps`.
#[allow(clippy::too_many_arguments)]
fn finalize_convergence_report(
    title_str: String,
    steps: Vec<MeshConvergenceStep>,
    disp_tol_pct: f64,
    stress_tol_pct: f64,
    has_non_axial_boundary: bool,
    last_report: Option<&FeaReport>,
    partial_result: bool,
    max_seconds: Option<f64>,
    total_study_time_ms: f64,
) -> MeshConvergenceStudyReport {
    let mut warnings = Vec::new();

    let (last_stress_change, last_disp_change, effective_refinement_ratio) = if steps.len() >= 2 {
        let prev = &steps[steps.len() - 2];
        let last = &steps[steps.len() - 1];
        let r_eff = (last.element_count.max(1) as f64 / prev.element_count.max(1) as f64).cbrt();
        (last.stress_change_pct, last.displacement_change_pct, r_eff)
    } else {
        (0.0, 0.0, 1.0)
    };

    let grid_clamped_detected =
        steps.len() >= 2 && effective_refinement_ratio < MIN_EFFECTIVE_REFINEMENT_RATIO;

    // Singularity check: displacement has stabilized (< 5% change) while peak stress grows rapidly (> 15% per step)
    let singularity_detected = if steps.len() >= 3 {
        let s1 = steps[steps.len() - 2].stress_change_pct;
        let s2 = steps[steps.len() - 1].stress_change_pct;
        let d2 = steps[steps.len() - 1].displacement_change_pct.abs();
        s1 > 15.0 && s2 > 15.0 && d2 < CONVERGENCE_DISP_TOLERANCE_PCT
    } else {
        false
    };

    // Detect Cartesian voxel staircase stress concentration on curved/non-axial boundaries:
    // When the surface mesh has non-axial/curved faces (e.g. cylindrical hole wall or pressurized bore)
    // and exhibits localized stress concentration (max_stress / mean_stress > 1.50), the un-snapped
    // Cartesian voxel grid systematically misestimates Kt by 16-39% (`docs/FEA_ACCURACY.md` §2.3-2.4).
    let voxel_staircase_detected = if has_non_axial_boundary {
        if let Some(rep) = last_report {
            let n = rep.nodal_von_mises_stresses.len().max(1) as f64;
            let mean_stress = rep.nodal_von_mises_stresses.iter().sum::<f64>() / n;
            let stress_ratio = rep.max_stress_von_mises_pa / mean_stress.max(1.0);
            stress_ratio > CURVED_STAIRCASE_STRESS_RATIO_THRESHOLD
        } else {
            true
        }
    } else {
        false
    };

    let changes_within_tol = steps.len() >= 2
        && last_disp_change.abs() < disp_tol_pct
        && last_stress_change.abs() < stress_tol_pct;

    let is_converged = !partial_result
        && changes_within_tol
        && !singularity_detected
        && !grid_clamped_detected
        && !voxel_staircase_detected;

    let verdict = if is_converged {
        "CONVERGED".to_string()
    } else {
        "NOT_CONVERGED".to_string()
    };

    let elem_counts: Vec<usize> = steps.iter().map(|s| s.element_count).collect();
    let stress_vals_mpa: Vec<f64> = steps.iter().map(|s| s.max_stress_von_mises_pa / 1.0e6).collect();
    let disp_vals_mm: Vec<f64> = steps.iter().map(|s| s.max_displacement_m * 1000.0).collect();

    let allow_extrap = !partial_result
        && !singularity_detected
        && !grid_clamped_detected
        && !voxel_staircase_detected;

    let richardson_stress_mpa = compute_richardson_value(&stress_vals_mpa, &elem_counts, allow_extrap);
    let richardson_displacement_mm = compute_richardson_value(&disp_vals_mm, &elem_counts, allow_extrap);
    let richardson_applicable = richardson_stress_mpa.is_some() || richardson_displacement_mm.is_some();

    let asymptotic_stress_pa = if let Some(s_mpa) = richardson_stress_mpa {
        s_mpa * 1.0e6
    } else if let Some(last) = steps.last() {
        last.max_stress_von_mises_pa
    } else {
        0.0
    };

    if partial_result {
        warnings.push(format!(
            "[PARTIAL_CONVERGENCE_TIMEOUT] Convergence study stopped early after {} level(s) ({:.1} ms elapsed, max_seconds = {:.3} s); returning partial result from highest completed mesh.",
            steps.len(),
            total_study_time_ms,
            max_seconds.unwrap_or(0.0)
        ));
    }

    if singularity_detected {
        warnings.push(
            "Unconverged stress singularity detected during mesh refinement: peak stress increases unbounded while displacement has converged. Consider rounding sharp corners, adding fillets, or using max_stress_excluding_constraint_zone_mpa.".to_string(),
        );
    }

    if grid_clamped_detected {
        warnings.push(format!(
            "Voxel grid axis saturation detected: effective 3D refinement ratio r_eff = {:.3} is below minimum {:.2} (the 60-cell axis cap prevented in-plane refinement between the last two levels).",
            effective_refinement_ratio, MIN_EFFECTIVE_REFINEMENT_RATIO
        ));
    }

    if voxel_staircase_detected {
        warnings.push(
            "Curved/non-axial boundary with stress concentration on Cartesian voxel staircase mesh: peak stress at curved features (holes, fillets, bores) has systematic staircase error (16-39%); boundary-fitted meshing (P3-T5) and TET10 elements (P3-T4) are required for accurate Kt.".to_string(),
        );
    }

    let used_tet10 = last_report.map(|r| r.element_order == 2).unwrap_or(false);
    if !used_tet10 {
        warnings.push(
            "TET10 quadratic tetrahedra recommended for accurate bending stiffness and stress concentration modeling around holes and notches.".to_string(),
        );
    }

    let recommendation = if partial_result {
        format!(
            "NOT_CONVERGED (partial_result=true): convergence study timed out after {} level(s) ({:.1} ms >= max_seconds {:.3} s). Increase 'max_seconds' to complete all refinement levels.",
            steps.len(),
            total_study_time_ms,
            max_seconds.unwrap_or(0.0)
        )
    } else if is_converged {
        format!(
            "Solution CONVERGED across {} mesh levels (|Δdisp| = {:.2}% < {:.1}%, |Δstress| = {:.2}% < {:.1}%, r_eff = {:.2}). Displacements and stresses are mesh-independent.",
            steps.len(),
            last_disp_change.abs(),
            disp_tol_pct,
            last_stress_change.abs(),
            stress_tol_pct,
            effective_refinement_ratio
        )
    } else if voxel_staircase_detected && grid_clamped_detected {
        format!(
            "NOT_CONVERGED: Curved stress concentrator (e.g. hole/bore) on Cartesian voxel staircase mesh with in-plane 60-cell grid saturation (r_eff = {:.2} < {:.2}). Apparent small step change (Δstress = {:.2}%, Δdisp = {:.2}%) is an artifact of voxel clamping and staircase approximation (~16-39% Kt error); apply safety factor margin >= 1.5 or use boundary-fitted TET10 meshing.",
            effective_refinement_ratio,
            MIN_EFFECTIVE_REFINEMENT_RATIO,
            last_stress_change,
            last_disp_change
        )
    } else if voxel_staircase_detected {
        format!(
            "NOT_CONVERGED: Curved boundary stress concentrator on Cartesian voxel staircase mesh (Δstress = {:.2}%, Δdisp = {:.2}%). Voxel stair-stepping systematically under/over-estimates curved boundary stress concentrations by 16-39%; apply safety factor margin >= 1.5 or use boundary-fitted TET10 meshing.",
            last_stress_change,
            last_disp_change
        )
    } else if grid_clamped_detected {
        format!(
            "NOT_CONVERGED: Voxel grid reached the 60-cell axis limit (effective 3D refinement ratio r_eff = {:.2} < {:.2}), preventing true mesh refinement between the last two levels.",
            effective_refinement_ratio,
            MIN_EFFECTIVE_REFINEMENT_RATIO
        )
    } else if singularity_detected {
        "NOT_CONVERGED: Stress singularity detected (displacement converged while peak stress continues to grow). Evaluate structural safety using max_stress_excluding_constraint_zone_mpa or add fillets.".to_string()
    } else {
        let extrap_hint = match (richardson_stress_mpa, richardson_displacement_mm) {
            (Some(s), Some(d)) => format!(
                " Monotonic asymptotic convergence observed; Richardson extrapolation estimates h->0 limit at {:.2} MPa and {:.4} mm.",
                s, d
            ),
            (Some(s), None) => format!(
                " Richardson extrapolation estimates asymptotic stress at {:.2} MPa.",
                s
            ),
            _ => String::new(),
        };
        format!(
            "NOT_CONVERGED: Relative change between last two mesh levels (|Δdisp| = {:.2}% vs {:.1}% limit, |Δstress| = {:.2}% vs {:.1}% limit) exceeds convergence thresholds due to discretization error.{} Use finer mesh or TET10 elements.",
            last_disp_change.abs(),
            disp_tol_pct,
            last_stress_change.abs(),
            stress_tol_pct,
            extrap_hint
        )
    };

    let summary = format!(
        "Convergence study for '{}': {} steps evaluated, status = {}, Δstress = {:.2}%, Δdisp = {:.2}%, asymptotic stress = {:.2} MPa",
        title_str,
        steps.len(),
        verdict,
        last_stress_change,
        last_disp_change,
        asymptotic_stress_pa / 1.0e6
    );

    MeshConvergenceStudyReport {
        title: title_str,
        steps,
        is_converged,
        verdict,
        singularity_detected,
        voxel_staircase_detected,
        grid_clamped_detected,
        effective_refinement_ratio,
        richardson_applicable,
        asymptotic_stress_pa,
        richardson_stress_mpa,
        richardson_displacement_mm,
        partial_result,
        max_seconds,
        total_study_time_ms,
        recommendation,
        summary,
        warnings,
    }
}

/// Formats a canonical label for a `MeshDensity` level.
pub fn density_label_str(density: MeshDensity) -> String {
    match density {
        MeshDensity::Coarse => "Coarse".to_string(),
        MeshDensity::Medium => "Medium".to_string(),
        MeshDensity::Fine => "Fine".to_string(),
        MeshDensity::Custom { max_element_size } => format!("Custom(h={:.4}m)", max_element_size),
    }
}

/// Formats a canonical lowercase key for a `MeshDensity` level.
pub fn density_key_str(density: MeshDensity) -> String {
    match density {
        MeshDensity::Coarse => "coarse".to_string(),
        MeshDensity::Medium => "medium".to_string(),
        MeshDensity::Fine => "fine".to_string(),
        MeshDensity::Custom { max_element_size } => format!("custom({:.4})", max_element_size),
    }
}

/// Runs a multi-level FEA convergence study on a surface `TriangleMesh` with a per-mesh
/// boundary condition builder closure and an optional `max_seconds` wall-clock timeout (default `TET4`).
///
/// Returns the highest completed level's `(FeaReport, TetrahedralMesh)` (with `report.accuracy`
/// populated and `report.confidence` downgraded to `"low"` if `NOT_CONVERGED`) along with the
/// full `MeshConvergenceStudyReport`.
pub fn run_fea_with_convergence<F>(
    part_id: impl Into<String>,
    surface_mesh: &TriangleMesh,
    material: &Material,
    densities: &[MeshDensity],
    max_seconds: Option<f64>,
    options: Option<SolverOptions>,
    build_boundary_for_mesh: F,
) -> Result<(FeaReport, TetrahedralMesh, MeshConvergenceStudyReport), FeaError>
where
    F: Fn(&TetrahedralMesh) -> Result<BoundaryConditions, FeaError>,
{
    run_fea_with_convergence_with_order(
        part_id,
        surface_mesh,
        material,
        densities,
        crate::tetra_mesh::ElementOrder::Linear,
        max_seconds,
        options,
        build_boundary_for_mesh,
    )
}

/// Runs a multi-level FEA convergence study on a surface `TriangleMesh` with explicit
/// `element_order` (`ElementOrder::Linear` for `TET4` or `ElementOrder::Quadratic` for `TET10`).
#[allow(clippy::too_many_arguments)]
pub fn run_fea_with_convergence_with_order<F>(
    part_id: impl Into<String>,
    surface_mesh: &TriangleMesh,
    material: &Material,
    densities: &[MeshDensity],
    element_order: crate::tetra_mesh::ElementOrder,
    max_seconds: Option<f64>,
    options: Option<SolverOptions>,
    build_boundary_for_mesh: F,
) -> Result<(FeaReport, TetrahedralMesh, MeshConvergenceStudyReport), FeaError>
where
    F: Fn(&TetrahedralMesh) -> Result<BoundaryConditions, FeaError>,
{
    let part_id_str = part_id.into();
    let default_levels = [MeshDensity::Coarse, MeshDensity::Medium, MeshDensity::Fine];
    let levels = if densities.is_empty() {
        &default_levels[..]
    } else {
        densities
    };

    let study_start = Instant::now();
    let aabb = surface_mesh.bounding_box();
    let has_non_axial_boundary = has_curved_or_non_axial_surface(surface_mesh);

    let mut steps = Vec::with_capacity(levels.len());
    let mut prev_stress = 0.0_f64;
    let mut prev_disp = 0.0_f64;
    let mut last_report: Option<FeaReport> = None;
    let mut last_mesh: Option<TetrahedralMesh> = None;
    let mut partial_result = false;

    for (idx, &density) in levels.iter().enumerate() {
        if idx > 0 {
            if let Some(limit_s) = max_seconds {
                if study_start.elapsed().as_secs_f64() >= limit_s {
                    partial_result = true;
                    break;
                }
            }
        }

        let step_start = Instant::now();
        let base_tet_mesh = TetrahedralMesh::from_mesh(surface_mesh, density)?;
        let tet_mesh = match element_order {
            crate::tetra_mesh::ElementOrder::Linear => base_tet_mesh,
            crate::tetra_mesh::ElementOrder::Quadratic => base_tet_mesh.to_tet10(),
        };
        let boundary = build_boundary_for_mesh(&tet_mesh)?;
        let report = crate::analyze_mesh(&part_id_str, &tet_mesh, material, &boundary, options)?;
        let step_ms = step_start.elapsed().as_secs_f64() * 1000.0;

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

        steps.push(MeshConvergenceStep {
            step_index: idx,
            density: density_key_str(density),
            density_label: density_label_str(density),
            max_edge_length: h,
            node_count: report.node_count,
            element_count: report.element_count,
            max_displacement_m: report.max_displacement_m,
            max_displacement_mm: report.max_displacement_m * 1000.0,
            max_stress_von_mises_pa: report.max_stress_von_mises_pa,
            max_von_mises_mpa: report.max_stress_von_mises_pa / 1.0e6,
            stress_change_pct: stress_change,
            displacement_change_pct: disp_change,
            computation_time_ms: step_ms,
        });

        prev_stress = report.max_stress_von_mises_pa;
        prev_disp = report.max_displacement_m;
        last_report = Some(report);
        last_mesh = Some(tet_mesh);

        if idx + 1 < levels.len() {
            if let Some(limit_s) = max_seconds {
                if study_start.elapsed().as_secs_f64() >= limit_s {
                    partial_result = true;
                    break;
                }
            }
        }
    }

    let total_study_time_ms = study_start.elapsed().as_secs_f64() * 1000.0;
    let study_report = finalize_convergence_report(
        part_id_str,
        steps,
        CONVERGENCE_DISP_TOLERANCE_PCT,
        CONVERGENCE_STRESS_TOLERANCE_PCT,
        has_non_axial_boundary,
        last_report.as_ref(),
        partial_result,
        max_seconds,
        total_study_time_ms,
    );

    let accuracy = study_report.to_fea_accuracy();
    let final_report = last_report
        .ok_or(FeaError::EmptyMesh)?
        .with_accuracy(accuracy);
    let final_mesh = last_mesh.ok_or(FeaError::EmptyMesh)?;

    Ok((final_report, final_mesh, study_report))
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
    let study_start = Instant::now();
    let mut steps = Vec::with_capacity(densities.len());
    let mut prev_stress = 0.0;
    let mut prev_disp = 0.0;
    let mut last_report: Option<FeaReport> = None;

    let surface_mesh = if part.mesh.indices.is_empty() && !part.dag.nodes.is_empty() {
        let settings = mcp_cad_core::tessellation::TessellationSettings::default();
        part.dag
            .evaluate_with_settings(&settings)
            .map_err(|e| FeaError::MeshGenerationFailed(e.to_string()))?
    } else {
        part.mesh.clone()
    };
    let aabb = surface_mesh.bounding_box();
    let has_non_axial_boundary = has_curved_or_non_axial_surface(&surface_mesh);

    for (idx, &density) in densities.iter().enumerate() {
        let step_start = Instant::now();
        let tet_mesh = TetrahedralMesh::from_mesh(&surface_mesh, density)?;
        let report = crate::analyze_mesh(&part.id, &tet_mesh, &part.material, boundary, options)?;
        let step_ms = step_start.elapsed().as_secs_f64() * 1000.0;

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

        steps.push(MeshConvergenceStep {
            step_index: idx,
            density: density_key_str(density),
            density_label: density_label_str(density),
            max_edge_length: h,
            node_count: report.node_count,
            element_count: report.element_count,
            max_displacement_m: report.max_displacement_m,
            max_displacement_mm: report.max_displacement_m * 1000.0,
            max_stress_von_mises_pa: report.max_stress_von_mises_pa,
            max_von_mises_mpa: report.max_stress_von_mises_pa / 1.0e6,
            stress_change_pct: stress_change,
            displacement_change_pct: disp_change,
            computation_time_ms: step_ms,
        });

        prev_stress = report.max_stress_von_mises_pa;
        prev_disp = report.max_displacement_m;
        last_report = Some(report);
    }

    let stress_tol = if tolerance_pct > 0.0 {
        tolerance_pct
    } else {
        CONVERGENCE_STRESS_TOLERANCE_PCT
    };

    Ok(finalize_convergence_report(
        title_str,
        steps,
        CONVERGENCE_DISP_TOLERANCE_PCT,
        stress_tol,
        has_non_axial_boundary,
        last_report.as_ref(),
        false,
        None,
        study_start.elapsed().as_secs_f64() * 1000.0,
    ))
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
    let study_start = Instant::now();
    let mut steps = Vec::with_capacity(edge_sizes.len());

    let mut prev_stress = 0.0;
    let mut prev_disp = 0.0;
    let mut last_report: Option<FeaReport> = None;

    for (idx, &h) in edge_sizes.iter().enumerate() {
        let step_start = Instant::now();
        let mesh = mesh_generator(h)?;
        let report = crate::analyze_mesh(&title_str, &mesh, material, boundary, options)?;
        let step_ms = step_start.elapsed().as_secs_f64() * 1000.0;

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
            density: format!("h={:.4}m", h),
            density_label: format!("h={:.4}m", h),
            max_edge_length: h,
            node_count: report.node_count,
            element_count: report.element_count,
            max_displacement_m: report.max_displacement_m,
            max_displacement_mm: report.max_displacement_m * 1000.0,
            max_stress_von_mises_pa: report.max_stress_von_mises_pa,
            max_von_mises_mpa: report.max_stress_von_mises_pa / 1.0e6,
            stress_change_pct: stress_change,
            displacement_change_pct: disp_change,
            computation_time_ms: step_ms,
        });

        prev_stress = report.max_stress_von_mises_pa;
        prev_disp = report.max_displacement_m;
        last_report = Some(report);
    }

    let stress_tol = if tolerance_pct > 0.0 {
        tolerance_pct
    } else {
        CONVERGENCE_STRESS_TOLERANCE_PCT
    };

    Ok(finalize_convergence_report(
        title_str,
        steps,
        CONVERGENCE_DISP_TOLERANCE_PCT,
        stress_tol,
        false,
        last_report.as_ref(),
        false,
        None,
        study_start.elapsed().as_secs_f64() * 1000.0,
    ))
}
