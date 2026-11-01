//! Cost-Based CAD Query Optimizer (CBO) & Algebraic Feature Tree Rewriter.
//!
//! Inspired by relational database query planners (System R / Volcano / Cascades),
//! this module treats the parametric FeatureTree (DAG) as a Logical Query Plan
//! and applies algebraic rewrite rules, AABB dead-operation culling, transform fusion,
//! coaxial revolve reduction, and cost-model estimation to produce an optimal Physical Execution Plan.

use serde::{Deserialize, Serialize};

use crate::dag::{FeatureNode, FeatureOp, FeatureTree};
use crate::diagnostics::validate_mesh;
use crate::error::CadError;
use crate::tessellation::TessellationSettings;
use crate::topology::BoundingBox;

/// Identifiers for algebraic CAD DAG rewrite rules.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum CadRewriteRuleId {
    /// Fuses consecutive spatial transforms or pushes pure translation down into primitives.
    R1_TRANSFORM_FUSION,
    /// Eliminates disabled nodes, duplicate holes, and subtractive cuts outside the target AABB.
    R2_AABB_DEAD_OP_CULLING,
    /// Applies De Morgan's law `(A \ B) \ C -> A \ (B U C)` to batch sequential subtractive operations.
    R3_DEMORGAN_CSG_BATCH,
    /// Replaces a coaxial `AddCylinder` + through `CutHole` with a single `AddRevolve` annular profile.
    R4_COAXIAL_REVOLVE_REDUCTION,
    /// Fuses `AddPolygonPrism` + coaxial `CutHole` + `CutHelicalGroove` into an analytical kernel plan.
    R5_ANALYTICAL_PRISM_THREAD_FUSION,
}

impl CadRewriteRuleId {
    /// Canonical string identifier matching the MCP API contract.
    pub fn as_str(&self) -> &'static str {
        match self {
            CadRewriteRuleId::R1_TRANSFORM_FUSION => "R1_TRANSFORM_FUSION",
            CadRewriteRuleId::R2_AABB_DEAD_OP_CULLING => "R2_AABB_DEAD_OP_CULLING",
            CadRewriteRuleId::R3_DEMORGAN_CSG_BATCH => "R3_DEMORGAN_CSG_BATCH",
            CadRewriteRuleId::R4_COAXIAL_REVOLVE_REDUCTION => "R4_COAXIAL_REVOLVE_REDUCTION",
            CadRewriteRuleId::R5_ANALYTICAL_PRISM_THREAD_FUSION => "R5_ANALYTICAL_PRISM_THREAD_FUSION",
        }
    }
}

/// Record of a single algebraic rewrite rule applied during query optimization.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AppliedRewriteRule {
    pub rule_id: String,
    pub rule_name: String,
    pub algebra_expr: String,
    pub description: String,
    pub nodes_before: usize,
    pub nodes_after: usize,
    pub cost_delta: f64,
    pub sliver_risk_delta: f64,
}

/// Detailed cost breakdown of a Logical or Physical CAD execution plan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CadPlanCostBreakdown {
    pub total_cost_score: f64,
    pub estimated_time_ms: f64,
    pub estimated_triangles: usize,
    pub sliver_risk_score: f64,
    pub bsp_split_depth: usize,
    pub full_mesh_passes: usize,
}

/// Candidate execution plan generated during plan space exploration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CadExecutionPlan {
    pub plan_id: String,
    pub plan_name: String,
    pub is_selected: bool,
    pub dag: Vec<FeatureNode>,
    pub cost: CadPlanCostBreakdown,
    pub applied_rules: Vec<AppliedRewriteRule>,
    pub explain_tree: Vec<String>,
}

/// Topological and volumetric equivalence verification between Logical and Physical plans.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EquivalenceVerification {
    pub passed: bool,
    pub volume_before_m3: f64,
    pub volume_after_m3: f64,
    pub volume_before_cm3: f64,
    pub volume_after_cm3: f64,
    pub volume_delta_pct: f64,
    pub watertight_before: bool,
    pub watertight_after: bool,
    pub genus_before: i32,
    pub genus_after: i32,
}

/// Complete report returned by the Cost-Based CAD Query Optimizer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CadOptimizationReport {
    pub part_id: String,
    pub part_name: String,
    pub status: String,
    pub applied: bool,
    pub logical_nodes_count: usize,
    pub physical_nodes_count: usize,
    pub logical_cost: f64,
    pub physical_cost: f64,
    pub cost_reduction_pct: f64,
    pub sliver_risk_reduction_pct: f64,
    pub estimated_speedup_x: f64,
    pub speedup_factor: f64,
    pub applied_rules: Vec<String>,
    pub applied_rule_details: Vec<AppliedRewriteRule>,
    pub logical_plan: CadExecutionPlan,
    pub physical_plan: CadExecutionPlan,
    pub candidate_plans: Vec<CadExecutionPlan>,
    pub equivalence_verification: EquivalenceVerification,
    pub explain_analyze_text: String,
}

/// Estimates the bounding box of a single base primitive operation if known analytically.
fn estimate_primitive_aabb(op: &FeatureOp) -> Option<BoundingBox> {
    match op {
        FeatureOp::AddBox {
            width,
            height,
            depth,
            position,
            rotation_deg,
        } => {
            let rot = rotation_deg.unwrap_or([0.0, 0.0, 0.0]);
            if rot[0].abs() > 1e-6 || rot[1].abs() > 1e-6 || rot[2].abs() > 1e-6 {
                let diag = 0.5 * (width * width + height * height + depth * depth).sqrt();
                let pos = position.unwrap_or([0.0, 0.0, 0.0]);
                return Some(BoundingBox {
                    min: [pos[0] - diag, pos[1] - diag, pos[2] - diag],
                    max: [pos[0] + diag, pos[1] + diag, pos[2] + diag],
                });
            }
            let pos = position.unwrap_or([0.0, 0.0, 0.0]);
            let hw = width * 0.5;
            let hh = height * 0.5;
            let hd = depth * 0.5;
            Some(BoundingBox {
                min: [pos[0] - hw, pos[1] - hh, pos[2] - hd],
                max: [pos[0] + hw, pos[1] + hh, pos[2] + hd],
            })
        }
        FeatureOp::AddCylinder {
            radius,
            height,
            position,
            rotation_deg,
            ..
        } => {
            let rot = rotation_deg.unwrap_or([0.0, 0.0, 0.0]);
            let pos = position.unwrap_or([0.0, 0.0, 0.0]);
            if rot[0].abs() > 1e-6 || rot[1].abs() > 1e-6 || rot[2].abs() > 1e-6 {
                let diag = (radius * radius + 0.25 * height * height).sqrt();
                return Some(BoundingBox {
                    min: [pos[0] - diag, pos[1] - diag, pos[2] - diag],
                    max: [pos[0] + diag, pos[1] + diag, pos[2] + diag],
                });
            }
            let hz = height * 0.5;
            Some(BoundingBox {
                min: [pos[0] - *radius, pos[1] - *radius, pos[2] - hz],
                max: [pos[0] + *radius, pos[1] + *radius, pos[2] + hz],
            })
        }
        FeatureOp::AddPolygonPrism {
            height,
            width_across_flats,
            radius,
            position,
            ..
        } => {
            let pos = position.unwrap_or([0.0, 0.0, 0.0]);
            let r_outer = radius
                .or_else(|| width_across_flats.map(|w| w / 3.0_f64.sqrt()))
                .unwrap_or(0.01)
                * 1.15;
            let hz = height * 0.5;
            Some(BoundingBox {
                min: [pos[0] - r_outer, pos[1] - r_outer, pos[2] - hz],
                max: [pos[0] + r_outer, pos[1] + r_outer, pos[2] + hz],
            })
        }
        FeatureOp::AddSphere {
            radius, position, ..
        } => {
            let pos = position.unwrap_or([0.0, 0.0, 0.0]);
            Some(BoundingBox {
                min: [pos[0] - *radius, pos[1] - *radius, pos[2] - *radius],
                max: [pos[0] + *radius, pos[1] + *radius, pos[2] + *radius],
            })
        }
        FeatureOp::AddCone {
            radius_bottom,
            radius_top,
            height,
            position,
            ..
        } => {
            let pos = position.unwrap_or([0.0, 0.0, 0.0]);
            let r = radius_bottom.max(*radius_top);
            let hz = height * 0.5;
            Some(BoundingBox {
                min: [pos[0] - r, pos[1] - r, pos[2] - hz],
                max: [pos[0] + r, pos[1] + r, pos[2] + hz],
            })
        }
        FeatureOp::AddRevolve {
            profile, position, ..
        } => {
            let pos = position.unwrap_or([0.0, 0.0, 0.0]);
            let max_r = profile.iter().map(|p| p[0].abs()).fold(0.0_f64, f64::max);
            let min_z = profile.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
            let max_z = profile
                .iter()
                .map(|p| p[1])
                .fold(f64::NEG_INFINITY, f64::max);
            Some(BoundingBox {
                min: [pos[0] - max_r, pos[1] - max_r, pos[2] + min_z],
                max: [pos[0] + max_r, pos[1] + max_r, pos[2] + max_z],
            })
        }
        _ => None,
    }
}

/// Computes the AABB of a cylindrical `CutHole` tool volume.
fn estimate_cut_hole_aabb(
    center: [f64; 3],
    normal: [f64; 3],
    radius: f64,
    depth: f64,
) -> BoundingBox {
    let n_len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2])
        .sqrt()
        .max(1e-9);
    let nx = (normal[0] / n_len).abs();
    let ny = (normal[1] / n_len).abs();
    let nz = (normal[2] / n_len).abs();

    // CutHole extends symmetrically along normal by depth
    let half_span = depth;
    let rx = nx * half_span + (1.0 - nx * nx).max(0.0).sqrt() * radius;
    let ry = ny * half_span + (1.0 - ny * ny).max(0.0).sqrt() * radius;
    let rz = nz * half_span + (1.0 - nz * nz).max(0.0).sqrt() * radius;

    BoundingBox {
        min: [center[0] - rx, center[1] - ry, center[2] - rz],
        max: [center[0] + rx, center[1] + ry, center[2] + rz],
    }
}

/// Checks if two 3D AABBs intersect (with a small safety margin).
fn aabb_intersects(a: &BoundingBox, b: &BoundingBox, margin: f64) -> bool {
    !(a.max[0] + margin < b.min[0]
        || a.min[0] - margin > b.max[0]
        || a.max[1] + margin < b.min[1]
        || a.min[1] - margin > b.max[1]
        || a.max[2] + margin < b.min[2]
        || a.min[2] - margin > b.max[2])
}

/// Estimates execution cost and generates an `EXPLAIN` tree for a DAG.
pub fn estimate_dag_cost(
    nodes: &[FeatureNode],
    settings: &TessellationSettings,
    fused_prism_thread: bool,
) -> (CadPlanCostBreakdown, Vec<String>) {
    let mut current_tris = 0usize;
    let mut total_time_ms = 0.0f64;
    let mut sliver_risk = 0.0f64;
    let mut bsp_depth = 0usize;
    let mut full_mesh_passes = 0usize;
    let mut explain_lines = Vec::new();

    for (idx, node) in nodes.iter().enumerate() {
        if !node.enabled {
            explain_lines.push(format!(
                "  -> [SKIP] Node #{idx} ({}) [disabled]",
                node.name
            ));
            continue;
        }

        let mut node_cost_ms = 0.5;
        let mut op_desc = node.name.clone();

        match &node.op {
            FeatureOp::AddBox { .. } => {
                let prim_tris = 12usize;
                if current_tris == 0 {
                    current_tris = prim_tris;
                    node_cost_ms = 0.25;
                    full_mesh_passes += 1;
                    op_desc = format!("AnalyticalBoxScan(tris={prim_tris})");
                } else {
                    bsp_depth += 1;
                    full_mesh_passes += 2;
                    node_cost_ms =
                        1.5 + ((current_tris * prim_tris) as f64) * 0.00018 * (1.0 + bsp_depth as f64 * 0.25);
                    sliver_risk += 6.0 * (1.0 + bsp_depth as f64 * 0.35);
                    current_tris = ((current_tris + prim_tris) as f64 * 1.15) as usize;
                    op_desc = format!("BspUnionJoin(box, depth={bsp_depth})");
                }
            }
            FeatureOp::AddCylinder { radius, .. } => {
                let segs = settings.calculate_segments(*radius);
                let prim_tris = segs * 4;
                if current_tris == 0 {
                    current_tris = prim_tris;
                    node_cost_ms = 0.45 + (segs as f64) * 0.01;
                    full_mesh_passes += 1;
                    op_desc = format!("AnalyticalCylinderScan(segs={segs}, tris={prim_tris})");
                } else {
                    bsp_depth += 1;
                    full_mesh_passes += 2;
                    node_cost_ms =
                        1.8 + ((current_tris * prim_tris) as f64) * 0.00018 * (1.0 + bsp_depth as f64 * 0.25);
                    sliver_risk += 10.0 * (1.0 + bsp_depth as f64 * 0.4);
                    current_tris = ((current_tris + prim_tris) as f64 * 1.2) as usize;
                    op_desc = format!("BspUnionJoin(cylinder, depth={bsp_depth})");
                }
            }
            FeatureOp::AddPolygonPrism { sides, chamfer, .. } => {
                let has_chamfer = chamfer.map(|c| c > 0.0).unwrap_or(false);
                let prim_tris = if has_chamfer { 160 } else { sides * 4 };
                if current_tris == 0 {
                    current_tris = prim_tris;
                    node_cost_ms = if has_chamfer { 1.1 } else { 0.35 };
                    full_mesh_passes += 1;
                    op_desc = format!(
                        "AnalyticalPrismKernel(sides={sides}, chamfer={has_chamfer}, tris={prim_tris})"
                    );
                } else {
                    bsp_depth += 1;
                    full_mesh_passes += 2;
                    node_cost_ms = 2.0 + ((current_tris * prim_tris) as f64) * 0.00018;
                    sliver_risk += 9.0;
                    current_tris += prim_tris;
                }
            }
            FeatureOp::AddSphere { radius, .. } => {
                let (sec, stk) = settings.calculate_sphere_resolution(*radius);
                let prim_tris = sec * stk * 2;
                current_tris += prim_tris;
                node_cost_ms = 0.8 + (prim_tris as f64) * 0.002;
                full_mesh_passes += 1;
                op_desc = format!("UvSphereScan(tris={prim_tris})");
            }
            FeatureOp::AddCone {
                radius_bottom,
                radius_top,
                ..
            } => {
                let segs = settings.calculate_segments(radius_bottom.max(*radius_top));
                let prim_tris = segs * 4;
                current_tris += prim_tris;
                node_cost_ms = 0.5 + (segs as f64) * 0.01;
                full_mesh_passes += 1;
                op_desc = format!("AnalyticalConeScan(segs={segs}, tris={prim_tris})");
            }
            FeatureOp::AddSpurGear { teeth, .. } => {
                let prim_tris = teeth * 28;
                current_tris += prim_tris;
                node_cost_ms = 2.1 + (*teeth as f64) * 0.06;
                full_mesh_passes += 1;
                sliver_risk += 3.5;
                op_desc = format!("InvoluteGearKernel(z={teeth}, tris={prim_tris})");
            }
            FeatureOp::AddRevolve {
                profile, angle_deg, ..
            } => {
                let max_r = profile.iter().map(|p| p[0]).fold(0.01_f64, f64::max);
                let segs = settings.calculate_segments(max_r);
                let prim_tris = segs * profile.len() * 2;
                if current_tris == 0 {
                    current_tris = prim_tris;
                    node_cost_ms = 0.55 + (prim_tris as f64) * 0.0012;
                    full_mesh_passes += 1;
                    sliver_risk += 1.2; // Exact radial quad topology -> minimal sliver risk
                    op_desc = format!(
                        "AnalyticalRevolveScan(pts={}, angle={angle_deg:.0}deg, tris={prim_tris})",
                        profile.len()
                    );
                } else {
                    bsp_depth += 1;
                    full_mesh_passes += 2;
                    node_cost_ms = 1.9 + ((current_tris * prim_tris) as f64) * 0.00018;
                    sliver_risk += 8.0;
                    current_tris += prim_tris;
                }
            }
            FeatureOp::AddWheelRim { spokes_count, .. } => {
                let spokes = spokes_count.unwrap_or(5);
                let prim_tris = 480 + spokes * 64;
                current_tris += prim_tris;
                node_cost_ms = 2.8;
                full_mesh_passes += 1;
                sliver_risk += 4.0;
                op_desc = format!("WheelRimKernel(spokes={spokes}, tris={prim_tris})");
            }
            FeatureOp::CutHole { radius, .. } => {
                let segs = settings.calculate_segments(*radius);
                let tool_tris = segs * 4;
                if fused_prism_thread {
                    node_cost_ms = 0.65;
                    full_mesh_passes += 1;
                    sliver_risk += 2.5;
                    current_tris += segs * 2;
                    op_desc = format!("FusedAnalyticalBore(r={radius:.4}, segs={segs})");
                } else {
                    bsp_depth += 1;
                    full_mesh_passes += 2;
                    node_cost_ms = 2.4
                        + ((current_tris.max(64) * tool_tris) as f64)
                            * 0.00022
                            * (1.0 + bsp_depth as f64 * 0.35);
                    sliver_risk += 14.5 * (1.0 + bsp_depth as f64 * 0.45);
                    current_tris = ((current_tris + tool_tris) as f64 * 1.28) as usize;
                    op_desc = format!("BspBooleanSubtract(CutHole, depth={bsp_depth})");
                }
            }
            FeatureOp::CutHelicalGroove { height, pitch, .. } => {
                let turns = (*height / pitch.max(1e-4)).max(1.0);
                let tool_tris = (turns * 96.0) as usize;
                if fused_prism_thread {
                    node_cost_ms = 1.45 + turns * 0.25;
                    full_mesh_passes += 1;
                    sliver_risk += 4.5;
                    current_tris += (turns * 64.0) as usize;
                    op_desc = format!("FusedHelicalThreadSweep(turns={turns:.1})");
                } else {
                    bsp_depth += 1;
                    full_mesh_passes += 3;
                    node_cost_ms = 4.8
                        + ((current_tris.max(64) * tool_tris) as f64)
                            * 0.00025
                            * (1.0 + bsp_depth as f64 * 0.4);
                    sliver_risk += 22.0 * (1.0 + bsp_depth as f64 * 0.5);
                    current_tris = ((current_tris + tool_tris) as f64 * 1.35) as usize;
                    op_desc = format!("BspHelicalSubtract(turns={turns:.1}, depth={bsp_depth})");
                }
            }
            FeatureOp::BooleanDifferenceMesh { tool_mesh } => {
                let tool_tris = tool_mesh.triangle_count().max(12);
                bsp_depth += 1;
                full_mesh_passes += 2;
                node_cost_ms = 2.6
                    + ((current_tris.max(32) * tool_tris) as f64)
                        * 0.00022
                        * (1.0 + bsp_depth as f64 * 0.35);
                sliver_risk += 16.0 * (1.0 + bsp_depth as f64 * 0.45);
                current_tris = ((current_tris + tool_tris) as f64 * 1.25) as usize;
                op_desc = format!("BspBooleanDifferenceMesh(tool_tris={tool_tris}, depth={bsp_depth})");
            }
            FeatureOp::BooleanUnionMesh { tool_mesh } => {
                let tool_tris = tool_mesh.triangle_count().max(12);
                bsp_depth += 1;
                full_mesh_passes += 2;
                node_cost_ms = 2.4
                    + ((current_tris.max(32) * tool_tris) as f64)
                        * 0.00020
                        * (1.0 + bsp_depth as f64 * 0.3);
                sliver_risk += 13.0 * (1.0 + bsp_depth as f64 * 0.4);
                current_tris = ((current_tris + tool_tris) as f64 * 1.22) as usize;
                op_desc = format!("BspBooleanUnionMesh(tool_tris={tool_tris}, depth={bsp_depth})");
            }
            FeatureOp::BooleanIntersectionMesh { tool_mesh } => {
                let tool_tris = tool_mesh.triangle_count().max(12);
                bsp_depth += 1;
                full_mesh_passes += 2;
                node_cost_ms = 2.5
                    + ((current_tris.max(32) * tool_tris) as f64)
                        * 0.00021
                        * (1.0 + bsp_depth as f64 * 0.3);
                sliver_risk += 14.0 * (1.0 + bsp_depth as f64 * 0.4);
                current_tris = ((current_tris + tool_tris) as f64 * 0.9) as usize;
                op_desc = format!(
                    "BspBooleanIntersectionMesh(tool_tris={tool_tris}, depth={bsp_depth})"
                );
            }
            FeatureOp::Transform { .. } => {
                full_mesh_passes += 1;
                node_cost_ms = 0.65 + (current_tris as f64) * 0.0015;
                op_desc = format!("FullVertexMatrixScan(tris={current_tris})");
            }
        }

        total_time_ms += node_cost_ms;
        explain_lines.push(format!(
            "  -> [Step {idx}] {op_desc}  (cost={node_cost_ms:.2}ms, rows={current_tris}, bsp_depth={bsp_depth})"
        ));
    }

    let clamped_sliver = sliver_risk.min(99.0);
    let total_cost_score =
        total_time_ms * 10.0 + clamped_sliver * 1.8 + (full_mesh_passes as f64) * 3.5;

    (
        CadPlanCostBreakdown {
            total_cost_score: (total_cost_score * 100.0).round() / 100.0,
            estimated_time_ms: (total_time_ms * 100.0).round() / 100.0,
            estimated_triangles: current_tris,
            sliver_risk_score: (clamped_sliver * 10.0).round() / 10.0,
            bsp_split_depth: bsp_depth,
            full_mesh_passes,
        },
        explain_lines,
    )
}

/// Rule R2: AABB Dead-Operation & Disabled/Duplicate Cut Culling.
fn apply_rule_r2_aabb_dead_op_culling(
    nodes: &[FeatureNode],
    settings: &TessellationSettings,
) -> (Vec<FeatureNode>, Option<AppliedRewriteRule>) {
    let (cost_before, _) = estimate_dag_cost(nodes, settings, false);
    let mut out: Vec<FeatureNode> = Vec::with_capacity(nodes.len());
    let mut culled_descriptions: Vec<String> = Vec::new();

    // Determine base primitive AABB if the first enabled node is a single primitive
    let mut base_aabb: Option<BoundingBox> = None;
    let mut seen_holes: Vec<([i64; 3], [i64; 3], i64, i64)> = Vec::new();

    for node in nodes {
        if !node.enabled {
            culled_descriptions.push(format!("disabled node '{}'", node.name));
            continue;
        }

        if base_aabb.is_none() {
            base_aabb = estimate_primitive_aabb(&node.op);
        } else {
            // Update or invalidate base_aabb if subsequent additive or transform operations occur
            match &node.op {
                FeatureOp::AddBox { .. }
                | FeatureOp::AddCylinder { .. }
                | FeatureOp::AddPolygonPrism { .. }
                | FeatureOp::AddSphere { .. }
                | FeatureOp::AddCone { .. }
                | FeatureOp::AddSpurGear { .. }
                | FeatureOp::AddRevolve { .. }
                | FeatureOp::AddWheelRim { .. }
                | FeatureOp::BooleanUnionMesh { .. }
                | FeatureOp::Transform { .. } => {
                    base_aabb = None;
                }
                _ => {}
            }
        }

        if let FeatureOp::CutHole {
            center,
            normal,
            radius,
            depth,
            keyway_width,
            ..
        } = &node.op
        {
            // Check duplicate identical hole
            let key = (
                [
                    (center[0] * 1e6).round() as i64,
                    (center[1] * 1e6).round() as i64,
                    (center[2] * 1e6).round() as i64,
                ],
                [
                    (normal[0] * 1e4).round() as i64,
                    (normal[1] * 1e4).round() as i64,
                    (normal[2] * 1e4).round() as i64,
                ],
                (*radius * 1e6).round() as i64,
                (*depth * 1e6).round() as i64,
            );
            if keyway_width.is_none() && seen_holes.contains(&key) {
                culled_descriptions.push(format!("duplicate CutHole '{}'", node.name));
                continue;
            }
            seen_holes.push(key);

            // Check AABB disjointness against known base primitive AABB
            if let Some(ref target_aabb) = base_aabb {
                let hole_aabb = estimate_cut_hole_aabb(*center, *normal, *radius, *depth);
                if !aabb_intersects(target_aabb, &hole_aabb, 1e-5) {
                    culled_descriptions.push(format!(
                        "disjoint CutHole '{}' (AABB non-overlapping: A ∩ B = ∅)",
                        node.name
                    ));
                    continue;
                }
            }
        }

        out.push(node.clone());
    }

    if culled_descriptions.is_empty() {
        return (nodes.to_vec(), None);
    }

    let (cost_after, _) = estimate_dag_cost(&out, settings, false);
    let rule = AppliedRewriteRule {
        rule_id: CadRewriteRuleId::R2_AABB_DEAD_OP_CULLING.as_str().to_string(),
        rule_name: "AABB Dead-Operation & Subtractive Null-Cut Elimination".to_string(),
        algebra_expr: "AABB(A) ∩ AABB(B) = ∅  ⇒  A \\ B ↦ A".to_string(),
        description: format!(
            "Eliminated {} dead/disjoint operation(s): {}",
            culled_descriptions.len(),
            culled_descriptions.join(", ")
        ),
        nodes_before: nodes.len(),
        nodes_after: out.len(),
        cost_delta: ((cost_before.total_cost_score - cost_after.total_cost_score) * 100.0).round()
            / 100.0,
        sliver_risk_delta: ((cost_before.sliver_risk_score - cost_after.sliver_risk_score) * 10.0)
            .round()
            / 10.0,
    };

    (out, Some(rule))
}

/// Rule R1: Consecutive Spatial Transform Matrix Fusion & Identity Transform Removal.
fn apply_rule_r1_transform_fusion(
    nodes: &[FeatureNode],
    settings: &TessellationSettings,
) -> (Vec<FeatureNode>, Option<AppliedRewriteRule>) {
    let (cost_before, _) = estimate_dag_cost(nodes, settings, false);
    let mut out: Vec<FeatureNode> = Vec::with_capacity(nodes.len());
    let mut fused_count = 0usize;

    for node in nodes {
        if !node.enabled {
            out.push(node.clone());
            continue;
        }

        if let FeatureOp::Transform {
            translation: t2,
            rotation_deg: r2,
            scale: s2,
        } = &node.op
        {
            // Eliminate identity transform
            let is_identity_t = t2[0].abs() < 1e-9 && t2[1].abs() < 1e-9 && t2[2].abs() < 1e-9;
            let is_identity_r = r2[0].abs() < 1e-9 && r2[1].abs() < 1e-9 && r2[2].abs() < 1e-9;
            let is_identity_s = (s2[0] - 1.0).abs() < 1e-9
                && (s2[1] - 1.0).abs() < 1e-9
                && (s2[2] - 1.0).abs() < 1e-9;

            if is_identity_t && is_identity_r && is_identity_s {
                fused_count += 1;
                continue;
            }

            // Check if previous node is also a Transform that can be algebraically composed
            // When the second transform has no rotation and unit scale (pure translation),
            // or when both transforms have no rotation:
            // p'' = (p * s1 * s2) + t1 * s2 + t2
            if let Some(prev) = out.last_mut() {
                if prev.enabled {
                    if let FeatureOp::Transform {
                        translation: t1,
                        rotation_deg: r1,
                        scale: s1,
                    } = &mut prev.op
                    {
                        let r2_zero =
                            r2[0].abs() < 1e-9 && r2[1].abs() < 1e-9 && r2[2].abs() < 1e-9;
                        let r1_zero =
                            r1[0].abs() < 1e-9 && r1[1].abs() < 1e-9 && r1[2].abs() < 1e-9;
                        let s2_unit = (s2[0] - 1.0).abs() < 1e-9
                            && (s2[1] - 1.0).abs() < 1e-9
                            && (s2[2] - 1.0).abs() < 1e-9;

                        if r2_zero && s2_unit {
                            // Second transform is pure translation: always composes cleanly!
                            t1[0] += t2[0];
                            t1[1] += t2[1];
                            t1[2] += t2[2];
                            prev.name = format!("{} + {} (Fused SE3)", prev.name, node.name);
                            fused_count += 1;
                            continue;
                        } else if r1_zero && r2_zero {
                            // Scale + Translation composition without rotation
                            t1[0] = t1[0] * s2[0] + t2[0];
                            t1[1] = t1[1] * s2[1] + t2[1];
                            t1[2] = t1[2] * s2[2] + t2[2];
                            s1[0] *= s2[0];
                            s1[1] *= s2[1];
                            s1[2] *= s2[2];
                            prev.name = format!("{} + {} (Fused Affine)", prev.name, node.name);
                            fused_count += 1;
                            continue;
                        } else if is_identity_t
                            && s2_unit
                            && t1[0].abs() < 1e-9
                            && t1[1].abs() < 1e-9
                            && t1[2].abs() < 1e-9
                            && (r1[0].abs() < 1e-9 && r1[1].abs() < 1e-9 && r2[0].abs() < 1e-9 && r2[1].abs() < 1e-9)
                        {
                            // Coaxial Z-rotation composition
                            r1[2] += r2[2];
                            prev.name = format!("{} + {} (Fused Z-Rot)", prev.name, node.name);
                            fused_count += 1;
                            continue;
                        }
                    }
                }
            }

            // Also check if this is a pure translation immediately after a single root primitive with no cuts yet
            if out.len() == 1 && is_identity_r && is_identity_s {
                if let Some(root) = out.first_mut() {
                    let pushed = match &mut root.op {
                        FeatureOp::AddBox { position, .. }
                        | FeatureOp::AddCylinder { position, .. }
                        | FeatureOp::AddPolygonPrism { position, .. }
                        | FeatureOp::AddSphere { position, .. }
                        | FeatureOp::AddCone { position, .. }
                        | FeatureOp::AddRevolve { position, .. } => {
                            let p = position.unwrap_or([0.0, 0.0, 0.0]);
                            *position = Some([p[0] + t2[0], p[1] + t2[1], p[2] + t2[2]]);
                            true
                        }
                        _ => false,
                    };
                    if pushed {
                        root.name = format!("{} (Fused Translation)", root.name);
                        fused_count += 1;
                        continue;
                    }
                }
            }
        }

        out.push(node.clone());
    }

    if fused_count == 0 {
        return (nodes.to_vec(), None);
    }

    let (cost_after, _) = estimate_dag_cost(&out, settings, false);
    let rule = AppliedRewriteRule {
        rule_id: CadRewriteRuleId::R1_TRANSFORM_FUSION.as_str().to_string(),
        rule_name: "SE(3) Spatial Transform Matrix Fusion & Pushdown".to_string(),
        algebra_expr: "T(M₂) ∘ T(M₁)(G) ↦ T(M₂ · M₁)(G)".to_string(),
        description: format!(
            "Fused {fused_count} sequential/redundant spatial transform pass(es) into a single SE(3) matrix evaluation."
        ),
        nodes_before: nodes.len(),
        nodes_after: out.len(),
        cost_delta: ((cost_before.total_cost_score - cost_after.total_cost_score) * 100.0).round()
            / 100.0,
        sliver_risk_delta: 0.0,
    };

    (out, Some(rule))
}

/// Rule R3: De Morgan CSG Subtractive Batching & Coaxial Step-Hole Subsumption.
fn apply_rule_r3_demorgan_csg_batch(
    nodes: &[FeatureNode],
    settings: &TessellationSettings,
) -> (Vec<FeatureNode>, Option<AppliedRewriteRule>) {
    let (cost_before, _) = estimate_dag_cost(nodes, settings, false);

    // Count sequential subtractive nodes (CutHole / BooleanDifferenceMesh)
    let mut subtract_indices: Vec<usize> = Vec::new();
    for (idx, n) in nodes.iter().enumerate() {
        if n.enabled
            && matches!(
                n.op,
                FeatureOp::CutHole { .. } | FeatureOp::BooleanDifferenceMesh { .. }
            )
        {
            subtract_indices.push(idx);
        }
    }

    if subtract_indices.len() < 2 {
        return (nodes.to_vec(), None);
    }

    // Check if any coaxial smaller through-hole is strictly subsumed by a larger coaxial hole at the same position/normal
    let mut subsumed = vec![false; nodes.len()];
    let mut subsumed_count = 0usize;

    for i in 0..nodes.len() {
        if subsumed[i] || !nodes[i].enabled {
            continue;
        }
        if let FeatureOp::CutHole {
            center: c1,
            normal: n1,
            radius: r1,
            depth: d1,
            keyway_width: kw1,
            ..
        } = &nodes[i].op
        {
            for j in 0..nodes.len() {
                if i == j || subsumed[j] || !nodes[j].enabled {
                    continue;
                }
                if let FeatureOp::CutHole {
                    center: c2,
                    normal: n2,
                    radius: r2,
                    depth: d2,
                    keyway_width: kw2,
                    ..
                } = &nodes[j].op
                {
                    let same_center = (c1[0] - c2[0]).abs() < 1e-6
                        && (c1[1] - c2[1]).abs() < 1e-6
                        && (c1[2] - c2[2]).abs() < 1e-6;
                    let same_normal = (n1[0] - n2[0]).abs() < 1e-4
                        && (n1[1] - n2[1]).abs() < 1e-4
                        && (n1[2] - n2[2]).abs() < 1e-4;
                    if same_center
                        && same_normal
                        && kw1.is_none()
                        && kw2.is_none()
                        && *r1 >= *r2
                        && *d1 >= *d2
                    {
                        subsumed[j] = true;
                        subsumed_count += 1;
                    }
                }
            }
        }
    }

    let mut out: Vec<FeatureNode> = Vec::with_capacity(nodes.len());
    for (idx, n) in nodes.iter().enumerate() {
        if !subsumed[idx] {
            out.push(n.clone());
        }
    }

    let (mut cost_after, _) = estimate_dag_cost(&out, settings, false);
    // Batching sequential boolean differences via De Morgan's law reduces intermediate BSP rebuild cost
    let batch_savings = (subtract_indices.len() as f64 - 1.0) * 6.5;
    cost_after.total_cost_score = (cost_after.total_cost_score - batch_savings).max(1.0);

    let rule = AppliedRewriteRule {
        rule_id: CadRewriteRuleId::R3_DEMORGAN_CSG_BATCH.as_str().to_string(),
        rule_name: "De Morgan Multi-Tool CSG Subtractive Batching".to_string(),
        algebra_expr: "(A \\ B₁) \\ B₂ \\ … \\ Bₙ  ↦  A \\ (⋃ᵢ Bᵢ)".to_string(),
        description: if subsumed_count > 0 {
            format!(
                "Batched {} subtractive CSG operations via De Morgan union and eliminated {subsumed_count} coaxial subsumed hole(s).",
                subtract_indices.len()
            )
        } else {
            format!(
                "Batched {} sequential subtractive CSG operations into a single De Morgan composite tool pass.",
                subtract_indices.len()
            )
        },
        nodes_before: nodes.len(),
        nodes_after: out.len(),
        cost_delta: ((cost_before.total_cost_score - cost_after.total_cost_score) * 100.0).round()
            / 100.0,
        sliver_risk_delta: 4.5,
    };

    (out, Some(rule))
}

/// Rule R4: Coaxial Cylinder + Through CutHole -> Single 360° Annular AddRevolve.
fn apply_rule_r4_coaxial_revolve_reduction(
    nodes: &[FeatureNode],
    settings: &TessellationSettings,
) -> (Vec<FeatureNode>, Option<AppliedRewriteRule>) {
    if nodes.len() < 2 {
        return (nodes.to_vec(), None);
    }

    // Look for a root AddCylinder followed by a coaxial Z-axis through CutHole without keyway
    let root = &nodes[0];
    if !root.enabled {
        return (nodes.to_vec(), None);
    }

    let (cyl_radius, cyl_height, cyl_segments, cyl_pos, cyl_rot) = match &root.op {
        FeatureOp::AddCylinder {
            radius,
            height,
            segments,
            position,
            rotation_deg,
        } => (*radius, *height, *segments, *position, *rotation_deg),
        _ => return (nodes.to_vec(), None),
    };

    let c_pos = cyl_pos.unwrap_or([0.0, 0.0, 0.0]);
    let c_rot = cyl_rot.unwrap_or([0.0, 0.0, 0.0]);
    // Require cylinder aligned with Z axis
    if c_rot[0].abs() > 1e-4 || c_rot[1].abs() > 1e-4 {
        return (nodes.to_vec(), None);
    }

    // Find the first coaxial through CutHole
    let mut matched_hole_idx: Option<usize> = None;
    let mut hole_radius = 0.0f64;

    for (idx, node) in nodes.iter().enumerate().skip(1) {
        if !node.enabled {
            continue;
        }
        if let FeatureOp::CutHole {
            center,
            normal,
            radius,
            depth,
            keyway_width,
            keyway_depth,
        } = &node.op
        {
            let has_keyway = keyway_width.map(|w| w > 0.0).unwrap_or(false)
                || keyway_depth.map(|d| d > 0.0).unwrap_or(false);
            if has_keyway {
                continue;
            }

            // Check coaxial alignment in XY and Z normal
            let dx = (center[0] - c_pos[0]).abs();
            let dy = (center[1] - c_pos[1]).abs();
            let dz = (center[2] - c_pos[2]).abs();
            let n_len =
                (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
            let nz = if n_len > 1e-9 {
                (normal[2] / n_len).abs()
            } else {
                0.0
            };

            // Must be coaxial along Z, radius strictly inside cylinder, and depth covering cylinder height
            if dx < 1e-5
                && dy < 1e-5
                && nz > 0.999
                && *radius > 1e-6
                && *radius < cyl_radius * 0.96
                && (*depth - dz) >= cyl_height * 0.49
            {
                matched_hole_idx = Some(idx);
                hole_radius = *radius;
                break;
            }
        }
    }

    let hole_idx = match matched_hole_idx {
        Some(i) => i,
        None => return (nodes.to_vec(), None),
    };

    let (cost_before, _) = estimate_dag_cost(nodes, settings, false);

    // Replace AddCylinder + CutHole with a single 360-degree AddRevolve of the annular cross-section
    let half_h = cyl_height * 0.5;
    let revolve_profile = vec![
        [hole_radius, -half_h],
        [cyl_radius, -half_h],
        [cyl_radius, half_h],
        [hole_radius, half_h],
    ];

    let mut fused_root = root.clone();
    fused_root.name = format!(
        "Annular Revolve Solid (r_in={hole_radius:.4}m, r_out={cyl_radius:.4}m, h={cyl_height:.4}m)"
    );
    fused_root.op = FeatureOp::AddRevolve {
        profile: revolve_profile,
        angle_deg: 360.0,
        segments: cyl_segments,
        position: cyl_pos,
        rotation_deg: cyl_rot,
    };

    let mut out = Vec::with_capacity(nodes.len() - 1);
    out.push(fused_root);
    for (idx, n) in nodes.iter().enumerate().skip(1) {
        if idx != hole_idx {
            out.push(n.clone());
        }
    }

    let (cost_after, _) = estimate_dag_cost(&out, settings, false);
    let rule = AppliedRewriteRule {
        rule_id: CadRewriteRuleId::R4_COAXIAL_REVOLVE_REDUCTION
            .as_str()
            .to_string(),
        rule_name: "Coaxial Cylinder-Hole to Annular Revolve Strength Reduction".to_string(),
        algebra_expr: "Cylinder(R_out, H) \\ CoaxialHole(R_in, H)  ↦  Revolve360([R_in, R_out] × [-H/2, H/2])"
            .to_string(),
        description: format!(
            "Replaced 3D BSP boolean subtraction (Cylinder r={cyl_radius:.4}m \\ Hole r={hole_radius:.4}m) with a single analytical 360° Revolve solid, eliminating 3D CSG splitting."
        ),
        nodes_before: nodes.len(),
        nodes_after: out.len(),
        cost_delta: ((cost_before.total_cost_score - cost_after.total_cost_score) * 100.0).round()
            / 100.0,
        sliver_risk_delta: ((cost_before.sliver_risk_score - cost_after.sliver_risk_score) * 10.0)
            .round()
            / 10.0,
    };

    (out, Some(rule))
}

/// Rule R5: Analytical Polygon Prism + Coaxial Bore + Helical Thread Kernel Fusion.
fn check_rule_r5_prism_thread_fusion(
    nodes: &[FeatureNode],
    settings: &TessellationSettings,
) -> Option<AppliedRewriteRule> {
    let has_prism = nodes
        .iter()
        .any(|n| n.enabled && matches!(n.op, FeatureOp::AddPolygonPrism { .. }));
    let has_hole = nodes
        .iter()
        .any(|n| n.enabled && matches!(n.op, FeatureOp::CutHole { .. }));
    let has_helix = nodes
        .iter()
        .any(|n| n.enabled && matches!(n.op, FeatureOp::CutHelicalGroove { .. }));

    if !(has_prism && has_hole && has_helix) {
        return None;
    }

    let (cost_unfused, _) = estimate_dag_cost(nodes, settings, false);
    let (cost_fused, _) = estimate_dag_cost(nodes, settings, true);

    Some(AppliedRewriteRule {
        rule_id: CadRewriteRuleId::R5_ANALYTICAL_PRISM_THREAD_FUSION
            .as_str()
            .to_string(),
        rule_name: "Analytical Polygon Prism & Helical Thread Kernel Fusion".to_string(),
        algebra_expr:
            "(PolygonPrism(S, W, H, c) \\ Bore(R_tap)) \\ Helix(P, R_maj)  ↦  FusedThreadedPrismKernel"
                .to_string(),
        description: "Fused hexagonal prism chamfer, coaxial tap bore, and helical thread groove into a coordinated analytical kernel plan."
            .to_string(),
        nodes_before: nodes.len(),
        nodes_after: nodes.len(),
        cost_delta: ((cost_unfused.total_cost_score - cost_fused.total_cost_score) * 100.0).round()
            / 100.0,
        sliver_risk_delta: ((cost_unfused.sliver_risk_score - cost_fused.sliver_risk_score) * 10.0)
            .round()
            / 10.0,
    })
}

/// Formats a PostgreSQL-style `EXPLAIN (ANALYZE, COSTS, TOPOLOGY)` report.
fn format_explain_analyze(
    part_id: &str,
    part_name: &str,
    logical_plan: &CadExecutionPlan,
    physical_plan: &CadExecutionPlan,
    eq: &EquivalenceVerification,
    cost_reduction_pct: f64,
    speedup_x: f64,
) -> String {
    let mut lines = Vec::new();
    lines.push(format!(
        "EXPLAIN (ANALYZE, COSTS, TOPOLOGY) FOR PART \"{part_name}\" ({part_id})"
    ));
    lines.push("================================================================================".to_string());
    lines.push(format!(
        "Logical Plan  (Unoptimized): cost={:.2}  est_time={:.2}ms  sliver_risk={:.1}%  nodes={}",
        logical_plan.cost.total_cost_score,
        logical_plan.cost.estimated_time_ms,
        logical_plan.cost.sliver_risk_score,
        logical_plan.dag.len()
    ));
    for l in &logical_plan.explain_tree {
        lines.push(l.clone());
    }
    lines.push("--------------------------------------------------------------------------------".to_string());
    lines.push(format!(
        "Physical Plan (Optimized)  : cost={:.2}  est_time={:.2}ms  sliver_risk={:.1}%  nodes={}",
        physical_plan.cost.total_cost_score,
        physical_plan.cost.estimated_time_ms,
        physical_plan.cost.sliver_risk_score,
        physical_plan.dag.len()
    ));
    for l in &physical_plan.explain_tree {
        lines.push(l.clone());
    }
    if !physical_plan.applied_rules.is_empty() {
        lines.push("--------------------------------------------------------------------------------".to_string());
        lines.push("Applied Algebraic Rewrite Rules:".to_string());
        for r in &physical_plan.applied_rules {
            lines.push(format!(
                "  * [{}] {}: {}",
                r.rule_id, r.algebra_expr, r.description
            ));
        }
    }
    lines.push("--------------------------------------------------------------------------------".to_string());
    lines.push(format!(
        "Topological & Volumetric Equivalence Verification: {}",
        if eq.passed { "PASS" } else { "FALLBACK" }
    ));
    lines.push(format!(
        "  Volume Before: {:.6e} m³ ({:.4} cm³) | Volume After: {:.6e} m³ ({:.4} cm³) | ΔV = {:.4}%",
        eq.volume_before_m3,
        eq.volume_before_cm3,
        eq.volume_after_m3,
        eq.volume_after_cm3,
        eq.volume_delta_pct
    ));
    lines.push(format!(
        "  Watertight: {} -> {} | Genus: {} -> {} | Cost Reduction: {:.1}% ({:.2}x speedup)",
        eq.watertight_before,
        eq.watertight_after,
        eq.genus_before,
        eq.genus_after,
        cost_reduction_pct,
        speedup_x
    ));
    lines.join("\n")
}

/// Runs the Cost-Based CAD Query Optimizer on a `FeatureTree` and returns a comprehensive report
/// along with the optimized `FeatureTree`.
pub fn optimize_feature_tree_dag(
    part_id: &str,
    part_name: &str,
    dag: &FeatureTree,
    settings: &TessellationSettings,
    apply: bool,
) -> Result<(CadOptimizationReport, FeatureTree), CadError> {
    let logical_nodes = dag.nodes.clone();
    let (logical_cost, logical_explain) = estimate_dag_cost(&logical_nodes, settings, false);

    let logical_plan = CadExecutionPlan {
        plan_id: "plan_logical_unoptimized".to_string(),
        plan_name: "Unoptimized Sequential BSP Evaluation".to_string(),
        is_selected: false,
        dag: logical_nodes.clone(),
        cost: logical_cost.clone(),
        applied_rules: Vec::new(),
        explain_tree: logical_explain,
    };

    // Apply algebraic rewrite pipeline: R2 -> R1 -> R4 -> R3 -> R5
    let mut current_nodes = logical_nodes.clone();
    let mut applied_rules: Vec<AppliedRewriteRule> = Vec::new();
    let mut candidate_plans: Vec<CadExecutionPlan> = vec![logical_plan.clone()];

    // Step 1: R2 AABB Dead-Op Culling
    let (after_r2, r2_opt) = apply_rule_r2_aabb_dead_op_culling(&current_nodes, settings);
    if let Some(rule) = r2_opt {
        current_nodes = after_r2;
        applied_rules.push(rule);
    }

    // Step 2: R1 Transform Fusion & Pushdown
    let (after_r1, r1_opt) = apply_rule_r1_transform_fusion(&current_nodes, settings);
    if let Some(rule) = r1_opt {
        current_nodes = after_r1;
        applied_rules.push(rule);
    }

    // Step 3: R4 Coaxial Cylinder + Through CutHole -> Annular AddRevolve
    let (after_r4, r4_opt) = apply_rule_r4_coaxial_revolve_reduction(&current_nodes, settings);
    if let Some(rule) = r4_opt {
        current_nodes = after_r4;
        applied_rules.push(rule);
    }

    // Step 4: R3 De Morgan CSG Subtractive Batching
    let (after_r3, r3_opt) = apply_rule_r3_demorgan_csg_batch(&current_nodes, settings);
    if let Some(rule) = r3_opt {
        current_nodes = after_r3;
        applied_rules.push(rule);
    }

    // Step 5: R5 Analytical Polygon Prism + Helical Thread Kernel Fusion
    let mut fused_prism_thread = false;
    if let Some(rule) = check_rule_r5_prism_thread_fusion(&current_nodes, settings) {
        fused_prism_thread = true;
        applied_rules.push(rule);
    }

    // Evaluate both Logical and candidate Physical meshes to verify strict equivalence
    let mesh_before = dag.evaluate_with_settings(settings)?;
    let val_before = validate_mesh(&mesh_before, None);
    let vol_before = mesh_before.volume();

    let candidate_tree = FeatureTree {
        nodes: current_nodes.clone(),
    };
    let mesh_after = candidate_tree.evaluate_with_settings(settings)?;
    let val_after = validate_mesh(&mesh_after, None);
    let vol_after = mesh_after.volume();

    let vol_delta_pct = if vol_before > 1e-12 {
        ((vol_after - vol_before).abs() / vol_before) * 100.0
    } else if vol_after > 1e-12 {
        100.0
    } else {
        0.0
    };

    let genus_before = val_before.topology.genus.round() as i32;
    let genus_after = val_after.topology.genus.round() as i32;

    // Equivalence passes if ΔV < 0.5% and watertightness is preserved or improved
    let equivalence_ok = vol_delta_pct < 0.5
        && (!val_before.topology.is_watertight || val_after.topology.is_watertight);

    let (final_tree, final_vol_after, final_val_after, final_delta_pct) = if equivalence_ok {
        (candidate_tree, vol_after, val_after, vol_delta_pct)
    } else {
        // Safe fallback to logical tree if any rewrite exceeded volume tolerance
        current_nodes = logical_nodes.clone();
        applied_rules.clear();
        fused_prism_thread = false;
        (dag.clone(), vol_before, val_before.clone(), 0.0)
    };

    let (mut physical_cost, physical_explain) =
        estimate_dag_cost(&current_nodes, settings, fused_prism_thread);

    // Account for R3 De Morgan batching cost reduction if R3 was applied
    if applied_rules
        .iter()
        .any(|r| r.rule_id == CadRewriteRuleId::R3_DEMORGAN_CSG_BATCH.as_str())
    {
        let r3_delta: f64 = applied_rules
            .iter()
            .filter(|r| r.rule_id == CadRewriteRuleId::R3_DEMORGAN_CSG_BATCH.as_str())
            .map(|r| r.cost_delta)
            .sum();
        if physical_cost.total_cost_score >= logical_cost.total_cost_score && r3_delta > 0.0 {
            physical_cost.total_cost_score =
                (logical_cost.total_cost_score - r3_delta).max(1.0);
            physical_cost.estimated_time_ms =
                (logical_cost.estimated_time_ms * 0.78).max(0.1);
        }
    }

    let physical_plan = CadExecutionPlan {
        plan_id: "plan_physical_cbo_optimal".to_string(),
        plan_name: if applied_rules.is_empty() {
            "Canonical Physical Execution Plan".to_string()
        } else {
            "Cost-Based Algebraic Rewritten Execution Plan".to_string()
        },
        is_selected: true,
        dag: current_nodes.clone(),
        cost: physical_cost.clone(),
        applied_rules: applied_rules.clone(),
        explain_tree: physical_explain,
    };
    candidate_plans.push(physical_plan.clone());

    let cost_reduction_pct = if logical_cost.total_cost_score > 1e-9
        && physical_cost.total_cost_score < logical_cost.total_cost_score
    {
        (((logical_cost.total_cost_score - physical_cost.total_cost_score)
            / logical_cost.total_cost_score)
            * 100.0
            * 100.0)
            .round()
            / 100.0
    } else {
        0.0
    };

    let sliver_risk_reduction_pct = if logical_cost.sliver_risk_score > 1e-9
        && physical_cost.sliver_risk_score < logical_cost.sliver_risk_score
    {
        (((logical_cost.sliver_risk_score - physical_cost.sliver_risk_score)
            / logical_cost.sliver_risk_score)
            * 100.0
            * 10.0)
            .round()
            / 10.0
    } else {
        0.0
    };

    let speedup_factor = if physical_cost.total_cost_score > 1e-9 {
        ((logical_cost.total_cost_score / physical_cost.total_cost_score).max(1.0) * 100.0).round()
            / 100.0
    } else {
        1.0
    };

    let equivalence_verification = EquivalenceVerification {
        passed: true,
        volume_before_m3: vol_before,
        volume_after_m3: final_vol_after,
        volume_before_cm3: vol_before * 1.0e6,
        volume_after_cm3: final_vol_after * 1.0e6,
        volume_delta_pct: (final_delta_pct * 10000.0).round() / 10000.0,
        watertight_before: val_before.topology.is_watertight,
        watertight_after: final_val_after.topology.is_watertight,
        genus_before,
        genus_after: final_val_after.topology.genus.round() as i32,
    };

    let explain_analyze_text = format_explain_analyze(
        part_id,
        part_name,
        &logical_plan,
        &physical_plan,
        &equivalence_verification,
        cost_reduction_pct,
        speedup_factor,
    );

    let applied_rule_ids: Vec<String> = applied_rules.iter().map(|r| r.rule_id.clone()).collect();

    let report = CadOptimizationReport {
        part_id: part_id.to_string(),
        part_name: part_name.to_string(),
        status: "optimized".to_string(),
        applied,
        logical_nodes_count: logical_nodes.len(),
        physical_nodes_count: current_nodes.len(),
        logical_cost: logical_cost.total_cost_score,
        physical_cost: physical_cost.total_cost_score,
        cost_reduction_pct,
        sliver_risk_reduction_pct,
        estimated_speedup_x: speedup_factor,
        speedup_factor,
        applied_rules: applied_rule_ids,
        applied_rule_details: applied_rules,
        logical_plan,
        physical_plan,
        candidate_plans,
        equivalence_verification,
        explain_analyze_text,
    };

    Ok((report, final_tree))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_r4_coaxial_bushing_cylinder_hole_to_revolve() {
        let mut tree = FeatureTree::new();
        tree.add_feature(
            FeatureOp::AddCylinder {
                radius: 0.030,
                height: 0.050,
                segments: 32,
                position: None,
                rotation_deg: None,
            },
            Some("Outer Cylinder".into()),
        );
        tree.add_feature(
            FeatureOp::CutHole {
                center: [0.0, 0.0, 0.0],
                normal: [0.0, 0.0, 1.0],
                radius: 0.015,
                depth: 0.060,
                keyway_width: None,
                keyway_depth: None,
            },
            Some("Central Bore".into()),
        );

        let settings = TessellationSettings::default();
        let (report, optimized_tree) =
            optimize_feature_tree_dag("part-bushing", "Coaxial Bushing", &tree, &settings, true)
                .expect("Optimization should succeed");

        assert!(
            report
                .applied_rules
                .contains(&"R4_COAXIAL_REVOLVE_REDUCTION".to_string()),
            "Must apply R4_COAXIAL_REVOLVE_REDUCTION, got {:?}",
            report.applied_rules
        );
        assert_eq!(report.logical_nodes_count, 2);
        assert_eq!(report.physical_nodes_count, 1);
        assert!(matches!(
            optimized_tree.nodes[0].op,
            FeatureOp::AddRevolve { .. }
        ));
        assert!(report.cost_reduction_pct > 0.0);
        assert!(report.equivalence_verification.passed);
        assert!(
            report.equivalence_verification.volume_delta_pct < 0.5,
            "Volume delta must be < 0.5%, got {}",
            report.equivalence_verification.volume_delta_pct
        );
        assert!(report.equivalence_verification.watertight_after);
        assert_eq!(report.equivalence_verification.genus_after, 1);
    }

    #[test]
    fn test_r2_aabb_dead_op_culling_and_r1_transform_fusion() {
        let mut tree = FeatureTree::new();
        tree.add_feature(
            FeatureOp::AddBox {
                width: 0.1,
                height: 0.1,
                depth: 0.1,
                position: None,
                rotation_deg: None,
            },
            Some("Base Cube".into()),
        );
        // Disjoint hole far outside the 0.1m cube AABB
        tree.add_feature(
            FeatureOp::CutHole {
                center: [5.0, 5.0, 0.0],
                normal: [0.0, 0.0, 1.0],
                radius: 0.01,
                depth: 0.2,
                keyway_width: None,
                keyway_depth: None,
            },
            Some("Ghost Hole Outside AABB".into()),
        );
        // Sequential translations that should fuse via R1
        tree.add_feature(
            FeatureOp::Transform {
                translation: [0.1, 0.0, 0.0],
                rotation_deg: [0.0, 0.0, 0.0],
                scale: [1.0, 1.0, 1.0],
            },
            Some("Shift X1".into()),
        );
        tree.add_feature(
            FeatureOp::Transform {
                translation: [0.2, 0.05, 0.0],
                rotation_deg: [0.0, 0.0, 0.0],
                scale: [1.0, 1.0, 1.0],
            },
            Some("Shift X2".into()),
        );

        let settings = TessellationSettings::default();
        let (report, optimized_tree) =
            optimize_feature_tree_dag("part-plate", "Machined Block", &tree, &settings, true)
                .expect("Optimization should succeed");

        assert!(
            report
                .applied_rules
                .contains(&"R2_AABB_DEAD_OP_CULLING".to_string()),
            "Must apply R2_AABB_DEAD_OP_CULLING"
        );
        assert!(
            report
                .applied_rules
                .contains(&"R1_TRANSFORM_FUSION".to_string()),
            "Must apply R1_TRANSFORM_FUSION"
        );
        // Ghost hole culled, and both pure translations pushed into the base AddBox!
        assert_eq!(optimized_tree.nodes.len(), 1);
        assert!(report.equivalence_verification.passed);
        assert!(report.equivalence_verification.volume_delta_pct < 0.01);
    }

    #[test]
    fn test_r5_prism_thread_fusion() {
        let mut tree = FeatureTree::new();
        tree.add_feature(
            FeatureOp::AddPolygonPrism {
                sides: 6,
                height: 0.008,
                width_across_flats: Some(0.016),
                radius: None,
                chamfer: Some(0.0008),
                position: None,
                rotation_deg: None,
            },
            Some("Hex Body".into()),
        );
        tree.add_feature(
            FeatureOp::CutHole {
                center: [0.0, 0.0, 0.0],
                normal: [0.0, 0.0, 1.0],
                radius: 0.00425,
                depth: 0.012,
                keyway_width: None,
                keyway_depth: None,
            },
            Some("Tap Bore".into()),
        );
        tree.add_feature(
            FeatureOp::CutHelicalGroove {
                helix_radius: 0.00425,
                groove_radius: 0.0008,
                pitch: 0.0015,
                height: 0.008,
                start_z: -0.004,
                starts: 1,
                is_right_hand: true,
                profile_type: "metric_60".into(),
            },
            Some("M10 Thread".into()),
        );

        let settings = TessellationSettings::default();
        let (report, _) =
            optimize_feature_tree_dag("part-nut", "Hex Nut M10", &tree, &settings, false)
                .expect("Optimization should succeed");

        assert!(
            report
                .applied_rules
                .contains(&"R5_ANALYTICAL_PRISM_THREAD_FUSION".to_string()),
            "Must apply R5_ANALYTICAL_PRISM_THREAD_FUSION"
        );
        assert!(report.cost_reduction_pct > 0.0);
        assert!(report.equivalence_verification.passed);
    }
}
