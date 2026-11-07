//! Comprehensive telemetry, simulation, analysis metrics, and compact scene summary reporting.

use serde_json::{json, Value};

use mcp_cad_core::part::Part;
use mcp_cad_physics::joint::CadJointType;
use mcp_cad_physics::simulator::CadJointRecord;

use crate::error::ServerError;
use crate::state::EngineState;

/// 17. `get_simulation_metrics_json`: Exports the complete engineering status and metrics report.
pub fn get_simulation_metrics_json(_args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let unit = state.length_unit;
    let k_from_m = unit.from_meters_factor();

    let part_summaries: Vec<Value> = state
        .parts
        .values()
        .map(|p| super::cad::part_summary_in_unit(p, unit))
        .collect();

    let fea_summaries: Vec<_> = state
        .last_fea_reports
        .values()
        .map(|r| {
            json!({
                "part_id": r.part_id,
                "verdict": r.verdict,
                "factor_of_safety": r.factor_of_safety,
                "max_von_mises_mpa": r.max_stress_von_mises_pa / 1.0e6,
                "yield_strength_mpa": r.yield_strength_pa / 1.0e6,
                "max_displacement_mm": r.max_displacement_m * 1000.0,
                "nodes": r.node_count,
                "elements": r.element_count,
                "time_ms": r.computation_time_ms
            })
        })
        .collect();

    let scaled_last_sim = state.last_simulation.as_ref().map(|sim| {
        let mut s = sim.clone();
        s.trajectories = super::physics::scale_trajectories_from_meters(&sim.trajectories, k_from_m);
        s
    });

    let metrics = json!({
        "parts_count": state.parts.len(),
        "parts": part_summaries,
        "physics": {
            "accumulated_time_s": state.physics_world.current_time(),
            "gravity": state.physics_world.gravity(),
            "last_simulation": scaled_last_sim
        },
        "fea_reports": fea_summaries,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    });

    Ok(json!({
        "current_metrics": metrics,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// 34. `get_scene_summary`: Returns a compact, one-call scene overview for AI agents entering a new context:
/// project units, parts (id, name, bounding box, volume, mass, mesh validity, latest FEA FoS/verdict/staleness),
/// kinematic joints, and declared engineering requirements with their verification status.
pub fn get_scene_summary(args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();
    let u_str = unit.as_str();
    let vol_u_str = unit.volume_str();

    let mut sorted_parts: Vec<&Part> = state.parts.values().collect();
    sorted_parts.sort_by(|a, b| a.id.cmp(&b.id));

    let mut parts_json = Vec::with_capacity(sorted_parts.len());
    let mut text_lines = Vec::new();
    text_lines.push(format!(
        "=== MCP-CAD SCENE SUMMARY (length_unit: {u_str}, volume: {vol_u_str}, mass: kg, force: N, stress: MPa) ==="
    ));
    text_lines.push(format!("Parts ({}):", sorted_parts.len()));

    if sorted_parts.is_empty() {
        text_lines.push("  (none)".to_string());
    }

    for part in sorted_parts {
        let aabb = part.bounding_box();
        let dims_m = if part.mesh().vertices.is_empty() {
            [0.0, 0.0, 0.0]
        } else {
            let d = aabb.dimensions();
            [d[0].max(0.0), d[1].max(0.0), d[2].max(0.0)]
        };
        let dims_u = [
            dims_m[0] * k_from_m,
            dims_m[1] * k_from_m,
            dims_m[2] * k_from_m,
        ];
        let min_u = if part.mesh().vertices.is_empty() {
            [0.0, 0.0, 0.0]
        } else {
            [
                aabb.min[0] * k_from_m,
                aabb.min[1] * k_from_m,
                aabb.min[2] * k_from_m,
            ]
        };
        let max_u = if part.mesh().vertices.is_empty() {
            [0.0, 0.0, 0.0]
        } else {
            [
                aabb.max[0] * k_from_m,
                aabb.max[1] * k_from_m,
                aabb.max[2] * k_from_m,
            ]
        };

        let vol_m3 = part.volume();
        let vol_u = vol_m3 * k_from_m.powi(3);
        let mass_kg = part.mass();
        let tris = part.mesh().indices.len();

        let (mesh_valid, mesh_status, is_watertight, is_manifold, open_edges, genus) =
            if part.mesh().vertices.is_empty() {
                (false, "EMPTY".to_string(), false, false, 0usize, 0.0f64)
            } else {
                let val = mcp_cad_core::validate_mesh(part.mesh(), None);
                let manifold = val.topology.non_manifold_edges_count == 0
                    && val.topology.non_manifold_vertices_count == 0;
                (
                    val.status == "PASS",
                    val.status,
                    val.topology.is_watertight,
                    manifold,
                    val.topology.boundary_edges_count,
                    val.topology.genus,
                )
            };

        let (last_fea_json, fea_text) = if let Some(rep) = state.last_fea_reports.get(&part.id) {
            let fea_rev = state.last_fea_revisions.get(&part.id).copied().unwrap_or(0);
            let stale = fea_rev != part.revision;
            let verdict_str = rep.verdict.as_str();
            let freshness = if stale { "STALE" } else { "FRESH" };
            let stress_mpa = rep.max_stress_von_mises_pa / 1.0e6;
            let disp_mm = rep.max_displacement_m * 1000.0;
            (
                json!({
                    "verdict": verdict_str,
                    "factor_of_safety": rep.factor_of_safety,
                    "safety_factor": rep.factor_of_safety,
                    "max_von_mises_mpa": stress_mpa,
                    "yield_strength_mpa": rep.yield_strength_pa / 1.0e6,
                    "max_displacement_mm": disp_mm,
                    "fea_revision": fea_rev,
                    "part_revision": part.revision,
                    "stale": stale,
                    "is_stale": stale,
                    "status": freshness,
                }),
                format!(
                    "FEA(verdict={verdict_str}, FoS={:.3}, max_stress={stress_mpa:.2} MPa, status={freshness}, fea_rev={fea_rev}/part_rev={})",
                    rep.factor_of_safety, part.revision
                ),
            )
        } else {
            (Value::Null, "FEA(none)".to_string())
        };

        text_lines.push(format!(
            "  - {} ('{}', mat={}, rev={}, fixed={}): bbox=[{:.2}, {:.2}, {:.2}] {u_str}, vol={:.2} {vol_u_str}, mass={:.4} kg, mesh={} (valid={}, tris={}), {}",
            part.id,
            part.name,
            part.material.name,
            part.revision,
            part.is_fixed,
            dims_u[0],
            dims_u[1],
            dims_u[2],
            vol_u,
            mass_kg,
            mesh_status,
            mesh_valid,
            tris,
            fea_text
        ));

        parts_json.push(json!({
            "part_id": part.id,
            "name": part.name,
            "material": part.material.name,
            "is_fixed": part.is_fixed,
            "revision": part.revision,
            "features_count": part.dag.nodes.len(),
            "bounding_box": {
                "min": min_u,
                "max": max_u,
                "dimensions": dims_u,
            },
            "dimensions": dims_u,
            "volume": vol_u,
            "volume_m3": vol_m3,
            "mass_kg": mass_kg,
            "triangles_count": tris,
            "mesh_valid": mesh_valid,
            "mesh_status": mesh_status,
            "is_watertight": is_watertight,
            "is_manifold": is_manifold,
            "open_edges_count": open_edges,
            "genus": genus,
            "last_fea": last_fea_json,
        }));
    }

    let mut sorted_joints: Vec<&CadJointRecord> = state.physics_world.joints().values().collect();
    sorted_joints.sort_by(|a, b| a.id.cmp(&b.id));

    let mut joints_json = Vec::with_capacity(sorted_joints.len());
    text_lines.push(format!("Joints ({}):", sorted_joints.len()));
    if sorted_joints.is_empty() {
        text_lines.push("  (none)".to_string());
    }

    for j in sorted_joints {
        let (j_type_str, anchor_a_u, anchor_b_u, extra_info) = match &j.joint_type {
            CadJointType::Revolute {
                anchor_a,
                anchor_b,
                axis,
                ..
            } => (
                "revolute",
                Some([
                    anchor_a[0] * k_from_m,
                    anchor_a[1] * k_from_m,
                    anchor_a[2] * k_from_m,
                ]),
                Some([
                    anchor_b[0] * k_from_m,
                    anchor_b[1] * k_from_m,
                    anchor_b[2] * k_from_m,
                ]),
                format!("axis=[{:.2}, {:.2}, {:.2}]", axis[0], axis[1], axis[2]),
            ),
            CadJointType::Prismatic {
                anchor_a,
                anchor_b,
                axis,
                ..
            } => (
                "prismatic",
                Some([
                    anchor_a[0] * k_from_m,
                    anchor_a[1] * k_from_m,
                    anchor_a[2] * k_from_m,
                ]),
                Some([
                    anchor_b[0] * k_from_m,
                    anchor_b[1] * k_from_m,
                    anchor_b[2] * k_from_m,
                ]),
                format!("axis=[{:.2}, {:.2}, {:.2}]", axis[0], axis[1], axis[2]),
            ),
            CadJointType::Spherical { anchor_a, anchor_b } => (
                "spherical",
                Some([
                    anchor_a[0] * k_from_m,
                    anchor_a[1] * k_from_m,
                    anchor_a[2] * k_from_m,
                ]),
                Some([
                    anchor_b[0] * k_from_m,
                    anchor_b[1] * k_from_m,
                    anchor_b[2] * k_from_m,
                ]),
                "3-DOF".to_string(),
            ),
            CadJointType::Fixed { anchor_a, anchor_b } => (
                "fixed",
                Some([
                    anchor_a[0] * k_from_m,
                    anchor_a[1] * k_from_m,
                    anchor_a[2] * k_from_m,
                ]),
                Some([
                    anchor_b[0] * k_from_m,
                    anchor_b[1] * k_from_m,
                    anchor_b[2] * k_from_m,
                ]),
                "0-DOF".to_string(),
            ),
            CadJointType::Gear { ratio, .. } => ("gear", None, None, format!("ratio={ratio:.3}")),
        };

        let motor_str = if let Some(m) = j.motor {
            format!(", motor(vel={:.2}, max_force={:.2})", m.target_velocity, m.max_force)
        } else {
            String::new()
        };

        text_lines.push(format!(
            "  - {} ({j_type_str}): {} <-> {} ({extra_info}{motor_str})",
            j.id, j.part_a, j.part_b
        ));

        joints_json.push(json!({
            "joint_id": j.id,
            "joint_type": j_type_str,
            "part_a": j.part_a,
            "part_b": j.part_b,
            "anchor_a": anchor_a_u,
            "anchor_b": anchor_b_u,
            "has_motor": j.motor.is_some(),
            "motor": j.motor.map(|m| json!({
                "target_velocity": m.target_velocity,
                "max_force": m.max_force,
            })),
        }));
    }

    let req_report = super::requirements::evaluate_requirements_report(state, None, unit);
    let req_total = req_report["total"].as_u64().unwrap_or(0);
    let req_passed = req_report["passed"].as_u64().unwrap_or(0);
    let req_failed = req_report["failed"].as_u64().unwrap_or(0);
    let req_stale = req_report["stale"].as_u64().unwrap_or(0);
    let req_na = req_report["not_available"].as_u64().unwrap_or(0);
    let req_status = req_report["status"].as_str().unwrap_or("PASS");

    text_lines.push(format!(
        "Requirements ({req_total}, auto_check={}, status={req_status}: {req_passed} PASS, {req_failed} FAIL, {req_stale} STALE, {req_na} NOT_AVAILABLE):",
        state.requirements.auto_check
    ));
    if let Some(items) = req_report["requirements"].as_array() {
        if items.is_empty() {
            text_lines.push("  (none)".to_string());
        } else {
            for item in items {
                let id = item["id"].as_str().unwrap_or("?");
                let rtype = item["type"].as_str().unwrap_or("?");
                let scope = item["scope"].as_str().unwrap_or("scene");
                let st = item["status"].as_str().unwrap_or("?");
                text_lines.push(format!(
                    "  - {id} ({rtype} on {scope}): {st} (measured={}, target={})",
                    item["measured"], item["target"]
                ));
            }
        }
    }

    let summary_text = text_lines.join("\n");

    Ok(json!({
        "summary_text": summary_text,
        "length_unit": u_str,
        "units": unit.units_metadata(),
        "parts_count": parts_json.len(),
        "parts": parts_json,
        "joints_count": joints_json.len(),
        "joints": joints_json,
        "requirements": {
            "status": req_status,
            "auto_check": state.requirements.auto_check,
            "total": req_total,
            "passed": req_passed,
            "failed": req_failed,
            "stale": req_stale,
            "not_available": req_na,
            "items": req_report["requirements"],
            "failed_requirements": req_report["failed_requirements"],
        }
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mcp_cad_core::dag::FeatureOp;
    use mcp_cad_physics::joint::MotorConfig;
    use mcp_cad_viewer::offscreen::OffscreenRenderer;
    use std::sync::Arc;

    #[test]
    fn test_get_scene_summary_unit_conversion_and_joints() {
        let renderer = Arc::new(OffscreenRenderer::new().expect("OffscreenRenderer init"));
        let mut state = EngineState::with_renderer(renderer);

        let mat = state.materials.get("aluminum_6061_t6").unwrap().clone();
        let mut p1 = Part::new("part_a", "Plate A", mat.clone());
        p1.dag.add_feature(
            FeatureOp::AddBox {
                width: 0.1,
                height: 0.02,
                depth: 0.05,
                position: None,
                rotation_deg: None,
            },
            None,
        );
        p1.is_fixed = true;
        p1.rebuild().unwrap();
        state.physics_world.add_part(&p1, false).unwrap();
        state.parts.insert(p1.id.clone(), p1);

        let mut p2 = Part::new("part_b", "Pin B", mat);
        p2.dag.add_feature(
            FeatureOp::AddCylinder {
                radius: 0.005,
                height: 0.04,
                position: None,
                rotation_deg: None,
                segments: 32,
            },
            None,
        );
        p2.rebuild().unwrap();
        state.physics_world.add_part(&p2, true).unwrap();
        state.parts.insert(p2.id.clone(), p2);

        state
            .physics_world
            .add_joint(
                "j_rev_1",
                "part_a",
                "part_b",
                CadJointType::Revolute {
                    anchor_a: [0.05, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                Some(MotorConfig::new(2.0, 10.0)),
            )
            .unwrap();

        let res_mm = get_scene_summary(&json!({}), &state).unwrap();
        assert_eq!(res_mm["parts_count"], 2);
        assert_eq!(res_mm["joints_count"], 1);
        assert_eq!(res_mm["length_unit"], "mm");
        assert!((res_mm["parts"][0]["dimensions"][0].as_f64().unwrap() - 100.0).abs() < 1e-6);
        assert!((res_mm["joints"][0]["anchor_a"][0].as_f64().unwrap() - 50.0).abs() < 1e-6);

        let res_m = get_scene_summary(&json!({ "length_unit": "m" }), &state).unwrap();
        assert_eq!(res_m["length_unit"], "m");
        assert!((res_m["parts"][0]["dimensions"][0].as_f64().unwrap() - 0.1).abs() < 1e-6);
        assert!((res_m["joints"][0]["anchor_a"][0].as_f64().unwrap() - 0.05).abs() < 1e-6);
    }
}

