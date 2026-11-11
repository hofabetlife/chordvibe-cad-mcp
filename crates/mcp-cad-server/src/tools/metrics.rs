//! Comprehensive telemetry, simulation, analysis metrics, and compact scene summary reporting.

use serde_json::{json, Value};

use mcp_cad_core::part::Part;
use mcp_cad_physics::joint::CadJointType;
use mcp_cad_physics::simulator::CadJointRecord;

use crate::error::ServerError;
use crate::state::EngineState;

/// 17. `get_simulation_metrics_json`: Exports the complete engineering status and metrics report.
pub fn get_simulation_metrics_json(args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let mut sorted_parts: Vec<&Part> = state.parts.values().collect();
    sorted_parts.sort_by(|a, b| a.id.cmp(&b.id));
    let part_summaries: Vec<Value> = sorted_parts
        .into_iter()
        .map(|p| super::cad::part_summary_in_unit(p, unit))
        .collect();

    let mut sorted_fea: Vec<_> = state.last_fea_reports.values().collect();
    sorted_fea.sort_by(|a, b| a.part_id.cmp(&b.part_id));
    let fea_summaries: Vec<_> = sorted_fea
        .into_iter()
        .map(|r| {
            let fea_rev = state.last_fea_revisions.get(&r.part_id).copied().unwrap_or(0);
            let part_rev = state.parts.get(&r.part_id).map(|p| p.revision).unwrap_or(0);
            let stale = fea_rev != part_rev;
            let freshness = if stale { "STALE" } else { "FRESH" };
            json!({
                "part_id": r.part_id,
                "verdict": r.verdict,
                "factor_of_safety": r.factor_of_safety,
                "safety_factor": r.factor_of_safety,
                "max_von_mises_mpa": r.max_stress_von_mises_pa / 1.0e6,
                "yield_strength_mpa": r.yield_strength_pa / 1.0e6,
                "max_displacement_mm": r.max_displacement_m * 1000.0,
                "nodes": r.node_count,
                "elements": r.element_count,
                "time_ms": r.computation_time_ms,
                "fea_revision": fea_rev,
                "part_revision": part_rev,
                "stale": stale,
                "is_stale": stale,
                "status": freshness
            })
        })
        .collect();

    let scaled_last_sim = state.last_simulation.as_ref().map(|sim| {
        let mut s = sim.clone();
        s.trajectories = super::physics::scale_trajectories_from_meters(&sim.trajectories, k_from_m);
        s
    });

    let raw_gravity = state.physics_world.gravity();
    let scaled_gravity = [
        raw_gravity[0] * k_from_m,
        raw_gravity[1] * k_from_m,
        raw_gravity[2] * k_from_m,
    ];
    let joints_count = state.physics_world.joints().len();

    let metrics = json!({
        "parts_count": state.parts.len(),
        "parts": part_summaries,
        "joints_count": joints_count,
        "physics": {
            "accumulated_time_s": state.physics_world.current_time(),
            "gravity": scaled_gravity,
            "gravity_m_s2": raw_gravity,
            "joints_count": joints_count,
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
        let (
            j_type_str,
            anchor_a_u,
            anchor_b_u,
            axis_opt,
            axis_a_opt,
            axis_b_opt,
            limits_u,
            limits_unit_opt,
            ratio_opt,
            extra_info,
        ) = match &j.joint_type {
            CadJointType::Revolute {
                anchor_a,
                anchor_b,
                axis,
                limits,
            } => {
                let lim_deg =
                    limits.map(|[min_rad, max_rad]| [min_rad.to_degrees(), max_rad.to_degrees()]);
                let lim_str = if let Some([l0, l1]) = lim_deg {
                    format!(", limits=[{l0:.2}, {l1:.2}] deg")
                } else {
                    String::new()
                };
                (
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
                    Some(*axis),
                    None,
                    None,
                    lim_deg,
                    lim_deg.map(|_| "deg"),
                    None,
                    format!(
                        "axis=[{:.2}, {:.2}, {:.2}]{lim_str}",
                        axis[0], axis[1], axis[2]
                    ),
                )
            }
            CadJointType::Prismatic {
                anchor_a,
                anchor_b,
                axis,
                limits,
            } => {
                let lim_len = limits.map(|[min_m, max_m]| [min_m * k_from_m, max_m * k_from_m]);
                let lim_str = if let Some([l0, l1]) = lim_len {
                    if u_str == "m" {
                        format!(", limits=[{l0:.4}, {l1:.4}] {u_str}")
                    } else {
                        format!(", limits=[{l0:.2}, {l1:.2}] {u_str}")
                    }
                } else {
                    String::new()
                };
                (
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
                    Some(*axis),
                    None,
                    None,
                    lim_len,
                    lim_len.map(|_| u_str),
                    None,
                    format!(
                        "axis=[{:.2}, {:.2}, {:.2}]{lim_str}",
                        axis[0], axis[1], axis[2]
                    ),
                )
            }
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
                None,
                None,
                None,
                None,
                None,
                None,
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
                None,
                None,
                None,
                None,
                None,
                None,
                "0-DOF".to_string(),
            ),
            CadJointType::Gear {
                ratio,
                axis_a,
                axis_b,
            } => (
                "gear",
                None,
                None,
                Some(*axis_a),
                Some(*axis_a),
                Some(*axis_b),
                None,
                None,
                Some(*ratio),
                format!(
                    "ratio={ratio:.3}, axis_a=[{:.2}, {:.2}, {:.2}], axis_b=[{:.2}, {:.2}, {:.2}]",
                    axis_a[0], axis_a[1], axis_a[2], axis_b[0], axis_b[1], axis_b[2]
                ),
            ),
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

        let mut joint_obj = json!({
            "joint_id": j.id,
            "joint_type": j_type_str,
            "part_a": j.part_a,
            "part_b": j.part_b,
            "anchor_a": anchor_a_u,
            "anchor_b": anchor_b_u,
            "axis": axis_opt,
            "limits": limits_u,
            "limits_unit": limits_unit_opt,
            "has_motor": j.motor.is_some(),
            "motor": j.motor.map(|m| json!({
                "target_velocity": m.target_velocity,
                "max_force": m.max_force,
            })),
        });
        if let Some(r) = ratio_opt {
            if let Some(map) = joint_obj.as_object_mut() {
                map.insert("ratio".to_string(), json!(r));
            }
        }
        if let (Some(aa), Some(ab)) = (axis_a_opt, axis_b_opt) {
            if let Some(map) = joint_obj.as_object_mut() {
                map.insert("axis_a".to_string(), json!(aa));
                map.insert("axis_b".to_string(), json!(ab));
            }
        }
        joints_json.push(joint_obj);
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
                    limits: Some([(-45.0_f64).to_radians(), (90.0_f64).to_radians()]),
                },
                Some(MotorConfig::new(2.0, 10.0)),
            )
            .unwrap();

        state
            .physics_world
            .add_joint(
                "j_prism_2",
                "part_a",
                "part_b",
                CadJointType::Prismatic {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [1.0, 0.0, 0.0],
                    limits: Some([-0.02, 0.08]),
                },
                None,
            )
            .unwrap();

        state
            .physics_world
            .add_joint(
                "j_gear_0",
                "part_a",
                "part_b",
                CadJointType::Gear {
                    ratio: 2.5,
                    axis_a: [0.0, 0.0, 1.0],
                    axis_b: [0.0, 1.0, 0.0],
                },
                None,
            )
            .unwrap();

        let res_mm = get_scene_summary(&json!({}), &state).unwrap();
        assert_eq!(res_mm["parts_count"], 2);
        assert_eq!(res_mm["joints_count"], 3);
        assert_eq!(res_mm["length_unit"], "mm");
        assert!((res_mm["parts"][0]["dimensions"][0].as_f64().unwrap() - 100.0).abs() < 1e-6);

        // Sorted by joint_id: j_gear_0 is index 0, j_prism_2 is index 1, j_rev_1 is index 2
        assert_eq!(res_mm["joints"][0]["joint_id"], "j_gear_0");
        assert_eq!(res_mm["joints"][0]["joint_type"], "gear");
        assert_eq!(res_mm["joints"][0]["axis"], json!([0.0, 0.0, 1.0]));
        assert_eq!(res_mm["joints"][0]["axis_a"], json!([0.0, 0.0, 1.0]));
        assert_eq!(res_mm["joints"][0]["axis_b"], json!([0.0, 1.0, 0.0]));
        assert!((res_mm["joints"][0]["ratio"].as_f64().unwrap() - 2.5).abs() < 1e-6);
        assert!(res_mm["joints"][0]["limits"].is_null());

        assert_eq!(res_mm["joints"][1]["joint_id"], "j_prism_2");
        assert_eq!(res_mm["joints"][1]["limits_unit"], "mm");
        assert!((res_mm["joints"][1]["limits"][0].as_f64().unwrap() - (-20.0)).abs() < 1e-6);
        assert!((res_mm["joints"][1]["limits"][1].as_f64().unwrap() - 80.0).abs() < 1e-6);

        assert_eq!(res_mm["joints"][2]["joint_id"], "j_rev_1");
        assert!((res_mm["joints"][2]["anchor_a"][0].as_f64().unwrap() - 50.0).abs() < 1e-6);
        assert_eq!(res_mm["joints"][2]["limits_unit"], "deg");
        assert!((res_mm["joints"][2]["limits"][0].as_f64().unwrap() - (-45.0)).abs() < 1e-6);
        assert!((res_mm["joints"][2]["limits"][1].as_f64().unwrap() - 90.0).abs() < 1e-6);
        assert!(res_mm["summary_text"]
            .as_str()
            .unwrap()
            .contains("ratio=2.500, axis_a=[0.00, 0.00, 1.00], axis_b=[0.00, 1.00, 0.00]"));
        assert!(res_mm["summary_text"]
            .as_str()
            .unwrap()
            .contains("limits=[-45.00, 90.00] deg"));
        assert!(res_mm["summary_text"]
            .as_str()
            .unwrap()
            .contains("limits=[-20.00, 80.00] mm"));

        let res_m = get_scene_summary(&json!({ "length_unit": "m" }), &state).unwrap();
        assert_eq!(res_m["length_unit"], "m");
        assert!((res_m["parts"][0]["dimensions"][0].as_f64().unwrap() - 0.1).abs() < 1e-6);
        assert_eq!(res_m["joints"][0]["axis"], json!([0.0, 0.0, 1.0]));
        assert_eq!(res_m["joints"][0]["axis_a"], json!([0.0, 0.0, 1.0]));
        assert_eq!(res_m["joints"][0]["axis_b"], json!([0.0, 1.0, 0.0]));
        assert_eq!(res_m["joints"][1]["limits_unit"], "m");
        assert!((res_m["joints"][1]["limits"][0].as_f64().unwrap() - (-0.02)).abs() < 1e-6);
        assert!((res_m["joints"][1]["limits"][1].as_f64().unwrap() - 0.08).abs() < 1e-6);
        assert!((res_m["joints"][2]["anchor_a"][0].as_f64().unwrap() - 0.05).abs() < 1e-6);
        assert_eq!(res_m["joints"][2]["limits_unit"], "deg");
        assert!((res_m["joints"][2]["limits"][0].as_f64().unwrap() - (-45.0)).abs() < 1e-6);
        assert!((res_m["joints"][2]["limits"][1].as_f64().unwrap() - 90.0).abs() < 1e-6);
    }

    #[test]
    fn test_p2_t27_get_simulation_metrics_json_length_unit_and_fea_staleness() {
        let renderer = Arc::new(OffscreenRenderer::new().expect("OffscreenRenderer init"));
        let mut state = EngineState::with_renderer(renderer);

        crate::tools::cad::create_part(
            &json!({
                "part_id": "beam_p2t27",
                "name": "Cantilever Beam",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        crate::tools::cad::add_feature_primitive(
            &json!({
                "part_id": "beam_p2t27",
                "primitive_type": "box",
                "params": { "width": 100.0, "height": 20.0, "depth": 20.0 }
            }),
            &mut state,
        )
        .unwrap();

        crate::tools::fea::run_fea_analysis(
            &json!({
                "part_id": "beam_p2t27",
                "fixed_faces": ["min_x"],
                "applied_forces": [
                    { "face": "max_x", "force_vector": [0.0, -500.0, 0.0] }
                ],
                "mesh_density": "coarse"
            }),
            &mut state,
        )
        .unwrap();

        // 1. Default session unit ("mm") -> fresh FEA (fea_revision == 1, part_revision == 1, stale == false)
        let metrics_mm = get_simulation_metrics_json(&json!({}), &state).unwrap();
        assert_eq!(metrics_mm["length_unit"], "mm");
        assert_eq!(metrics_mm["current_metrics"]["length_unit"], "mm");
        let part_mm = &metrics_mm["current_metrics"]["parts"][0];
        assert!((part_mm["bounding_box_dimensions"][0].as_f64().unwrap() - 100.0).abs() < 1e-6);
        assert!((part_mm["volume"].as_f64().unwrap() - 40_000.0).abs() < 1e-3);

        let fea_mm = &metrics_mm["current_metrics"]["fea_reports"][0];
        assert_eq!(fea_mm["part_id"], "beam_p2t27");
        assert_eq!(fea_mm["fea_revision"], 1);
        assert_eq!(fea_mm["part_revision"], 1);
        assert_eq!(fea_mm["stale"], false);
        assert_eq!(fea_mm["is_stale"], false);
        assert_eq!(fea_mm["status"], "FRESH");

        // 2. Per-call override length_unit = "m"
        let metrics_m = get_simulation_metrics_json(&json!({ "length_unit": "m" }), &state).unwrap();
        assert_eq!(metrics_m["length_unit"], "m");
        assert_eq!(metrics_m["units"]["volume"], "m³");
        let part_m = &metrics_m["current_metrics"]["parts"][0];
        assert!((part_m["bounding_box_dimensions"][0].as_f64().unwrap() - 0.1).abs() < 1e-6);
        assert!((part_m["volume"].as_f64().unwrap() - 4.0e-5).abs() < 1e-9);

        // 3. Calling optimize_feature_tree when no rules apply must NOT mark FEA as stale
        let opt_res = crate::tools::cad::optimize_feature_tree(
            &json!({
                "part_id": "beam_p2t27",
                "apply": true
            }),
            &mut state,
        )
        .unwrap();
        assert!(opt_res["applied_rules"].as_array().unwrap().is_empty());
        assert_eq!(opt_res["revision"], 1);

        let metrics_after_noop_opt = get_simulation_metrics_json(&json!({}), &state).unwrap();
        let fea_after_noop = &metrics_after_noop_opt["current_metrics"]["fea_reports"][0];
        assert_eq!(fea_after_noop["fea_revision"], 1);
        assert_eq!(fea_after_noop["part_revision"], 1);
        assert_eq!(fea_after_noop["stale"], false);
        assert_eq!(fea_after_noop["status"], "FRESH");

        // 4. Modifying geometry (cut_hole) bumps revision to 2 -> get_simulation_metrics_json reports stale == true
        crate::tools::cad::cut_hole(
            &json!({
                "part_id": "beam_p2t27",
                "position": [0.0, 0.0, 0.0],
                "normal": [0.0, 0.0, 1.0],
                "radius": 3.0,
                "through": true
            }),
            &mut state,
        )
        .unwrap();

        let metrics_stale = get_simulation_metrics_json(&json!({}), &state).unwrap();
        let fea_stale = &metrics_stale["current_metrics"]["fea_reports"][0];
        assert_eq!(fea_stale["fea_revision"], 1);
        assert_eq!(fea_stale["part_revision"], 2);
        assert_eq!(fea_stale["stale"], true);
        assert_eq!(fea_stale["is_stale"], true);
        assert_eq!(fea_stale["status"], "STALE");
    }

    #[test]
    fn test_p2_t32_get_simulation_metrics_json_gravity_scaling_and_joints_count() {
        let renderer = Arc::new(OffscreenRenderer::new().expect("OffscreenRenderer init"));
        let mut state = EngineState::with_renderer(renderer);

        crate::tools::cad::create_part(
            &json!({
                "part_id": "part_p2t32",
                "name": "Base Part",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        crate::tools::cad::add_feature_primitive(
            &json!({
                "part_id": "part_p2t32",
                "primitive_type": "cylinder",
                "params": { "radius": 10.0, "height": 30.0 }
            }),
            &mut state,
        )
        .unwrap();

        // 1. Initially 0 joints
        let m_mm = get_simulation_metrics_json(&json!({}), &state).unwrap();
        assert_eq!(m_mm["current_metrics"]["joints_count"], 0);
        assert_eq!(m_mm["current_metrics"]["physics"]["joints_count"], 0);
        assert_eq!(
            m_mm["current_metrics"]["physics"]["gravity"][1].as_f64().unwrap(),
            -9810.0
        );
        assert_eq!(
            m_mm["current_metrics"]["physics"]["gravity_m_s2"][1].as_f64().unwrap(),
            -9.81
        );

        // 2. Override unit: 'm' -> gravity in m/s²
        let m_m = get_simulation_metrics_json(&json!({ "length_unit": "m" }), &state).unwrap();
        assert_eq!(
            m_m["current_metrics"]["physics"]["gravity"][1].as_f64().unwrap(),
            -9.81
        );
        assert_eq!(
            m_m["current_metrics"]["physics"]["gravity_m_s2"][1].as_f64().unwrap(),
            -9.81
        );

        // 3. Create a joint with ground -> joints_count becomes 1
        crate::tools::physics::create_joint(
            &json!({
                "joint_id": "joint_base",
                "joint_type": "fixed",
                "part_a": "ground",
                "part_b": "part_p2t32",
                "anchor_a": [0.0, 0.0, 0.0],
                "anchor_b": [0.0, 0.0, 0.0]
            }),
            &mut state,
        )
        .unwrap();

        let m_with_joint = get_simulation_metrics_json(&json!({}), &state).unwrap();
        assert_eq!(m_with_joint["current_metrics"]["joints_count"], 1);
        assert_eq!(m_with_joint["current_metrics"]["physics"]["joints_count"], 1);
    }
}

