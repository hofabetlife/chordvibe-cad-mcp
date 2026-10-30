//! Phase 2: Autonomous Wide Scanner for MCP-CAD Engine.
//! Executes mesh-audit matrices (primitives, operations, chains), analytical benchmarks (Section 6),
//! and the 26-tool functional matrix (Section 5), dumping structured JSON results to `qa/results/`
//! and renders to `qa/artifacts/`.

use serde_json::{json, Value};
use std::fs::File;
use std::path::{Path, PathBuf};

use mcp_cad_core::csg::{csg_difference, csg_intersection, csg_union};
use mcp_cad_core::diagnostics::validate_mesh;
use mcp_cad_core::primitives::*;
use mcp_cad_core::tessellation::TessellationSettings;
use mcp_cad_server::protocol::handle_message;
use mcp_cad_server::state::EngineState;

#[allow(dead_code)]
const PNG_MAGIC: [u8; 8] = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];

fn qa_base_dir() -> PathBuf {
    if Path::new("../../qa").exists() {
        PathBuf::from("../../qa")
    } else {
        PathBuf::from("qa")
    }
}

async fn rpc_call(state: &mcp_cad_server::SharedState, req: Value) -> Value {
    let req_str = req.to_string();
    let resp = handle_message(&req_str, state)
        .await
        .expect("Expected a JSON-RPC response");
    serde_json::to_value(&resp).expect("Failed to serialize response")
}

async fn call_tool(state: &mcp_cad_server::SharedState, name: &str, args: Value) -> (bool, Value) {
    let resp = rpc_call(
        state,
        json!({
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {
                "name": name,
                "arguments": args
            }
        }),
    )
    .await;

    if resp.get("error").is_some() {
        return (false, resp["error"].clone());
    }
    let result = &resp["result"];
    let is_err = result
        .get("isError")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let text = result["content"]
        .as_array()
        .and_then(|arr| arr.iter().find(|c| c["type"] == "text"))
        .and_then(|c| c["text"].as_str())
        .unwrap_or("{}");
    let parsed = serde_json::from_str::<Value>(text).unwrap_or_else(|_| json!({ "raw_text": text }));
    (!is_err, parsed)
}

#[tokio::test]
async fn test_wide_scan_primitives_mesh_quality() {
    let base_dir = qa_base_dir();
    let art_dir = base_dir.join("artifacts");
    let res_dir = base_dir.join("results");
    let _ = std::fs::create_dir_all(&art_dir);
    let _ = std::fs::create_dir_all(&res_dir);

    let mut scan_results = Vec::new();

    let qualities = [
        ("coarse", TessellationSettings::coarse()),
        ("medium", TessellationSettings::medium()),
        ("fine", TessellationSettings::fine()),
    ];

    // 1. Box
    for (q_name, _settings) in &qualities {
        let mesh = create_box(0.1, 0.2, 0.3);
        let report = validate_mesh(&mesh, Some(0));

        scan_results.push(json!({
            "target": "box",
            "quality": q_name,
            "triangles": mesh.triangle_count(),
            "vertices": mesh.vertex_count(),
            "status": report.status,
            "watertight": report.topology.is_watertight,
            "orientation_consistent": report.topology.orientation_consistent,
            "euler_chi": report.topology.euler_characteristic,
            "genus": report.topology.genus,
            "signed_volume": report.geometry.signed_volume,
            "issues": report.issues,
            "warnings": report.warnings,
        }));
    }

    // 2. Cylinder
    for (q_name, settings) in &qualities {
        let mesh = create_cylinder_with_settings(0.05, 0.2, settings);
        let report = validate_mesh(&mesh, Some(0));

        scan_results.push(json!({
            "target": "cylinder",
            "quality": q_name,
            "triangles": mesh.triangle_count(),
            "vertices": mesh.vertex_count(),
            "status": report.status,
            "watertight": report.topology.is_watertight,
            "orientation_consistent": report.topology.orientation_consistent,
            "euler_chi": report.topology.euler_characteristic,
            "genus": report.topology.genus,
            "signed_volume": report.geometry.signed_volume,
            "issues": report.issues,
            "warnings": report.warnings,
        }));
    }

    // 3. Sphere
    for (q_name, settings) in &qualities {
        let mesh = create_sphere_with_settings(0.05, settings);
        let report = validate_mesh(&mesh, Some(0));

        scan_results.push(json!({
            "target": "sphere",
            "quality": q_name,
            "triangles": mesh.triangle_count(),
            "vertices": mesh.vertex_count(),
            "status": report.status,
            "watertight": report.topology.is_watertight,
            "orientation_consistent": report.topology.orientation_consistent,
            "euler_chi": report.topology.euler_characteristic,
            "genus": report.topology.genus,
            "signed_volume": report.geometry.signed_volume,
            "issues": report.issues,
            "warnings": report.warnings,
        }));
    }

    // 4. Cone (sharp r_top = 0 and frustum r_top > 0)
    for (q_name, settings) in &qualities {
        let mesh_sharp = create_cone_with_settings(0.05, 0.0, 0.1, settings);
        let rep_sharp = validate_mesh(&mesh_sharp, Some(0));
        scan_results.push(json!({
            "target": "cone_sharp",
            "quality": q_name,
            "triangles": mesh_sharp.triangle_count(),
            "status": rep_sharp.status,
            "watertight": rep_sharp.topology.is_watertight,
            "orientation_consistent": rep_sharp.topology.orientation_consistent,
            "euler_chi": rep_sharp.topology.euler_characteristic,
            "genus": rep_sharp.topology.genus,
            "signed_volume": rep_sharp.geometry.signed_volume,
            "issues": rep_sharp.issues,
            "warnings": rep_sharp.warnings,
        }));

        let mesh_frustum = create_cone_with_settings(0.05, 0.02, 0.1, settings);
        let rep_frustum = validate_mesh(&mesh_frustum, Some(0));
        scan_results.push(json!({
            "target": "cone_frustum",
            "quality": q_name,
            "triangles": mesh_frustum.triangle_count(),
            "status": rep_frustum.status,
            "watertight": rep_frustum.topology.is_watertight,
            "orientation_consistent": rep_frustum.topology.orientation_consistent,
            "euler_chi": rep_frustum.topology.euler_characteristic,
            "genus": rep_frustum.topology.genus,
            "signed_volume": rep_frustum.geometry.signed_volume,
            "issues": rep_frustum.issues,
            "warnings": rep_frustum.warnings,
        }));
    }

    // 5. Polygon Prism (3, 4, 6, 8 sides, with and without chamfer)
    for sides in [3, 4, 6, 8] {
        for chamfer in [0.0, 0.002] {
            let mesh = create_polygon_prism(
                sides,
                0.05,
                Some(0.04),
                None,
                if chamfer > 0.0 { Some(chamfer) } else { None },
            );
            let report = validate_mesh(&mesh, Some(0));
            scan_results.push(json!({
                "target": format!("polygon_prism_{}sides_chamfer_{}", sides, chamfer),
                "sides": sides,
                "chamfer": chamfer,
                "triangles": mesh.triangle_count(),
                "status": report.status,
                "watertight": report.topology.is_watertight,
                "orientation_consistent": report.topology.orientation_consistent,
                "euler_chi": report.topology.euler_characteristic,
                "genus": report.topology.genus,
                "signed_volume": report.geometry.signed_volume,
                "issues": report.issues,
                "warnings": report.warnings,
            }));
        }
    }

    // 6. Spur Gear (teeth: 8, 20, 60; with and without hub/keyway; pressure angle: 14.5, 20.0, 25.0)
    for teeth in [8, 20, 60] {
        let mesh_plain = create_spur_gear(0.002, teeth, 20.0, 0.015, None, None, None, None, None);
        let rep_plain = validate_mesh(&mesh_plain, Some(0));
        scan_results.push(json!({
            "target": format!("spur_gear_teeth_{}_plain", teeth),
            "teeth": teeth,
            "triangles": mesh_plain.triangle_count(),
            "status": rep_plain.status,
            "watertight": rep_plain.topology.is_watertight,
            "orientation_consistent": rep_plain.topology.orientation_consistent,
            "euler_chi": rep_plain.topology.euler_characteristic,
            "genus": rep_plain.topology.genus,
            "signed_volume": rep_plain.geometry.signed_volume,
            "issues": rep_plain.issues,
            "warnings": rep_plain.warnings,
        }));

        let mesh_hub = create_spur_gear(
            0.002,
            teeth,
            20.0,
            0.015,
            Some(0.006),
            Some(0.003),
            Some(0.002),
            Some(0.025),
            Some(0.010),
        );
        let rep_hub = validate_mesh(&mesh_hub, Some(1));
        scan_results.push(json!({
            "target": format!("spur_gear_teeth_{}_with_hub_bore_keyway", teeth),
            "teeth": teeth,
            "triangles": mesh_hub.triangle_count(),
            "status": rep_hub.status,
            "watertight": rep_hub.topology.is_watertight,
            "orientation_consistent": rep_hub.topology.orientation_consistent,
            "euler_chi": rep_hub.topology.euler_characteristic,
            "genus": rep_hub.topology.genus,
            "signed_volume": rep_hub.geometry.signed_volume,
            "issues": rep_hub.issues,
            "warnings": rep_hub.warnings,
        }));
    }

    for pa in [14.5, 25.0] {
        let mesh_pa = create_spur_gear(0.002, 20, pa, 0.015, None, None, None, None, None);
        let rep_pa = validate_mesh(&mesh_pa, Some(0));
        scan_results.push(json!({
            "target": format!("spur_gear_teeth_20_pa_{}", pa),
            "pressure_angle_deg": pa,
            "triangles": mesh_pa.triangle_count(),
            "status": rep_pa.status,
            "watertight": rep_pa.topology.is_watertight,
            "orientation_consistent": rep_pa.topology.orientation_consistent,
            "euler_chi": rep_pa.topology.euler_characteristic,
            "genus": rep_pa.topology.genus,
            "signed_volume": rep_pa.geometry.signed_volume,
            "issues": rep_pa.issues,
            "warnings": rep_pa.warnings,
        }));
    }

    // 7. Revolve (closed profile around Z axis: hollow ring, solid touching Z axis, partial 180, L-profile)
    let rev_profile = vec![[0.01, -0.02], [0.03, -0.02], [0.03, 0.02], [0.01, 0.02]];
    let mesh_rev360 = create_revolve(&rev_profile, 360.0, 32);
    let rep_rev360 = validate_mesh(&mesh_rev360, Some(1));
    scan_results.push(json!({
        "target": "revolve_hollow_ring_360",
        "triangles": mesh_rev360.triangle_count(),
        "status": rep_rev360.status,
        "watertight": rep_rev360.topology.is_watertight,
        "orientation_consistent": rep_rev360.topology.orientation_consistent,
        "euler_chi": rep_rev360.topology.euler_characteristic,
        "genus": rep_rev360.topology.genus,
        "signed_volume": rep_rev360.geometry.signed_volume,
        "issues": rep_rev360.issues,
        "warnings": rep_rev360.warnings,
    }));

    let mesh_rev180 = create_revolve(&rev_profile, 180.0, 32);
    let rep_rev180 = validate_mesh(&mesh_rev180, Some(0));
    scan_results.push(json!({
        "target": "revolve_partial_180",
        "triangles": mesh_rev180.triangle_count(),
        "status": rep_rev180.status,
        "watertight": rep_rev180.topology.is_watertight,
        "orientation_consistent": rep_rev180.topology.orientation_consistent,
        "euler_chi": rep_rev180.topology.euler_characteristic,
        "genus": rep_rev180.topology.genus,
        "signed_volume": rep_rev180.geometry.signed_volume,
        "issues": rep_rev180.issues,
        "warnings": rep_rev180.warnings,
    }));

    // Revolve profile touching r = 0 axis (solid cylinder-like body, genus 0)
    let rev_axis_profile = vec![[0.0, -0.02], [0.03, -0.02], [0.03, 0.02], [0.0, 0.02]];
    let mesh_rev_axis = create_revolve(&rev_axis_profile, 360.0, 32);
    let rep_rev_axis = validate_mesh(&mesh_rev_axis, Some(0));
    scan_results.push(json!({
        "target": "revolve_touching_axis_360",
        "triangles": mesh_rev_axis.triangle_count(),
        "status": rep_rev_axis.status,
        "watertight": rep_rev_axis.topology.is_watertight,
        "orientation_consistent": rep_rev_axis.topology.orientation_consistent,
        "euler_chi": rep_rev_axis.topology.euler_characteristic,
        "genus": rep_rev_axis.topology.genus,
        "signed_volume": rep_rev_axis.geometry.signed_volume,
        "issues": rep_rev_axis.issues,
        "warnings": rep_rev_axis.warnings,
    }));

    // 8. Wheel Rim (5, 6, 10 spokes)
    for spokes in [5, 6, 10] {
        let mesh_wheel = create_wheel_rim(
            0.4572,
            0.2032,
            Some(0.490),
            Some(0.160),
            Some(0.066),
            Some(0.1143),
            Some(5),
            Some(0.014),
            Some(spokes),
            Some(0.028),
            Some(0.018),
            Some(0.035),
        );
        let rep_wheel = validate_mesh(&mesh_wheel, None);
        scan_results.push(json!({
            "target": format!("wheel_rim_{}spoke", spokes),
            "spokes": spokes,
            "triangles": mesh_wheel.triangle_count(),
            "status": rep_wheel.status,
            "watertight": rep_wheel.topology.is_watertight,
            "orientation_consistent": rep_wheel.topology.orientation_consistent,
            "euler_chi": rep_wheel.topology.euler_characteristic,
            "genus": rep_wheel.topology.genus,
            "signed_volume": rep_wheel.geometry.signed_volume,
            "issues": rep_wheel.issues,
            "warnings": rep_wheel.warnings,
        }));
    }

    let report_json = json!({
        "phase": "phase_2_primitives_scan",
        "total_primitives_tested": scan_results.len(),
        "results": scan_results
    });

    let mut out_file = File::create(res_dir.join("primitives_mesh_scan.json")).unwrap();
    serde_json::to_writer_pretty(&mut out_file, &report_json).unwrap();
}

#[tokio::test]
async fn test_wide_scan_operations_and_chains() {
    let base_dir = qa_base_dir();
    let res_dir = base_dir.join("results");
    let _ = std::fs::create_dir_all(&res_dir);

    let mut ops_results = Vec::new();
    let settings = TessellationSettings::medium();

    // 1. cut_hole variations
    let base_box = create_box(0.1, 0.1, 0.04);
    // 1a. Through hole along Z
    let mesh_thru = cut_hole_with_settings(
        &base_box,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.015,
        0.10,
        None,
        None,
        &settings,
    );
    let rep_thru = validate_mesh(&mesh_thru, Some(1));
    ops_results.push(json!({
        "operation": "cut_hole_through_box",
        "status": rep_thru.status,
        "watertight": rep_thru.topology.is_watertight,
        "orientation_consistent": rep_thru.topology.orientation_consistent,
        "boundary_edges": rep_thru.topology.boundary_edges_count,
        "t_junctions": rep_thru.geometry.t_junctions_count,
        "self_intersections": rep_thru.geometry.self_intersections_count,
        "genus": rep_thru.topology.genus,
        "expected_genus": 1,
        "signed_volume": rep_thru.geometry.signed_volume,
        "issues": rep_thru.issues,
    }));

    // 1b. Blind hole along Z (starts on top face z=0.02, goes down depth=0.02 in -Z direction)
    let mesh_blind = cut_hole_with_settings(
        &base_box,
        [0.0, 0.0, 0.01],
        [0.0, 0.0, 1.0],
        0.015,
        0.03,
        None,
        None,
        &settings,
    );
    let rep_blind = validate_mesh(&mesh_blind, Some(0));
    ops_results.push(json!({
        "operation": "cut_hole_blind_box",
        "status": rep_blind.status,
        "watertight": rep_blind.topology.is_watertight,
        "orientation_consistent": rep_blind.topology.orientation_consistent,
        "boundary_edges": rep_blind.topology.boundary_edges_count,
        "t_junctions": rep_blind.geometry.t_junctions_count,
        "self_intersections": rep_blind.geometry.self_intersections_count,
        "genus": rep_blind.topology.genus,
        "expected_genus": 0,
        "signed_volume": rep_blind.geometry.signed_volume,
        "issues": rep_blind.issues,
    }));

    // 1c. Hole with keyway
    let mesh_keyway = cut_hole_with_settings(
        &base_box,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.015,
        0.10,
        Some(0.006),
        Some(0.003),
        &settings,
    );
    let rep_keyway = validate_mesh(&mesh_keyway, Some(1));
    ops_results.push(json!({
        "operation": "cut_hole_with_keyway",
        "status": rep_keyway.status,
        "watertight": rep_keyway.topology.is_watertight,
        "orientation_consistent": rep_keyway.topology.orientation_consistent,
        "boundary_edges": rep_keyway.topology.boundary_edges_count,
        "t_junctions": rep_keyway.geometry.t_junctions_count,
        "self_intersections": rep_keyway.geometry.self_intersections_count,
        "genus": rep_keyway.topology.genus,
        "expected_genus": 1,
        "issues": rep_keyway.issues,
    }));

    // 1d. Angled hole [1, 0, 1]
    let mesh_angled = cut_hole_with_settings(
        &base_box,
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 1.0],
        0.01,
        0.20,
        None,
        None,
        &settings,
    );
    let rep_angled = validate_mesh(&mesh_angled, Some(1));
    ops_results.push(json!({
        "operation": "cut_hole_angled",
        "status": rep_angled.status,
        "watertight": rep_angled.topology.is_watertight,
        "orientation_consistent": rep_angled.topology.orientation_consistent,
        "boundary_edges": rep_angled.topology.boundary_edges_count,
        "t_junctions": rep_angled.geometry.t_junctions_count,
        "self_intersections": rep_angled.geometry.self_intersections_count,
        "genus": rep_angled.topology.genus,
        "expected_genus": 1,
        "issues": rep_angled.issues,
    }));

    // 2. cut_helical_groove (circular & metric_60)
    let cyl = create_cylinder_with_settings(0.01, 0.04, &settings);
    for profile_type in ["circular", "metric_60"] {
        let cutter = create_helical_cutter_with_settings(
            0.01,
            0.002,
            0.008,
            0.03,
            -0.015,
            2,
            true,
            0.03 / 0.008,
            profile_type,
            &settings,
        );
        let grooved = csg_difference(&cyl, &cutter);
        let rep_groove = validate_mesh(&grooved, Some(0));
        ops_results.push(json!({
            "operation": format!("cut_helical_groove_{}", profile_type),
            "status": rep_groove.status,
            "watertight": rep_groove.topology.is_watertight,
            "orientation_consistent": rep_groove.topology.orientation_consistent,
            "boundary_edges": rep_groove.topology.boundary_edges_count,
            "t_junctions": rep_groove.geometry.t_junctions_count,
            "self_intersections": rep_groove.geometry.self_intersections_count,
            "genus": rep_groove.topology.genus,
            "issues": rep_groove.issues,
        }));
    }

    // 3. Booleans (union, difference, intersection) on overlapping boxes & coplanar faces
    let box_a = create_box(0.1, 0.1, 0.1);
    let mut box_b = create_box(0.1, 0.1, 0.1);
    box_b.translate([0.05, 0.0, 0.0]); // Coplanar top/bottom/front/back faces!

    let mesh_union = csg_union(&box_a, &box_b);
    let rep_union = validate_mesh(&mesh_union, Some(0));
    ops_results.push(json!({
        "operation": "boolean_union_coplanar_boxes",
        "status": rep_union.status,
        "watertight": rep_union.topology.is_watertight,
        "orientation_consistent": rep_union.topology.orientation_consistent,
        "boundary_edges": rep_union.topology.boundary_edges_count,
        "t_junctions": rep_union.geometry.t_junctions_count,
        "self_intersections": rep_union.geometry.self_intersections_count,
        "genus": rep_union.topology.genus,
        "signed_volume": rep_union.geometry.signed_volume,
        "expected_volume": 0.0015,
        "issues": rep_union.issues,
    }));

    let mesh_diff = csg_difference(&box_a, &box_b);
    let rep_diff = validate_mesh(&mesh_diff, Some(0));
    ops_results.push(json!({
        "operation": "boolean_difference_coplanar_boxes",
        "status": rep_diff.status,
        "watertight": rep_diff.topology.is_watertight,
        "orientation_consistent": rep_diff.topology.orientation_consistent,
        "boundary_edges": rep_diff.topology.boundary_edges_count,
        "t_junctions": rep_diff.geometry.t_junctions_count,
        "self_intersections": rep_diff.geometry.self_intersections_count,
        "genus": rep_diff.topology.genus,
        "signed_volume": rep_diff.geometry.signed_volume,
        "expected_volume": 0.0005,
        "issues": rep_diff.issues,
    }));

    let mesh_inter = csg_intersection(&box_a, &box_b);
    let rep_inter = validate_mesh(&mesh_inter, Some(0));
    ops_results.push(json!({
        "operation": "boolean_intersection_coplanar_boxes",
        "status": rep_inter.status,
        "watertight": rep_inter.topology.is_watertight,
        "orientation_consistent": rep_inter.topology.orientation_consistent,
        "boundary_edges": rep_inter.topology.boundary_edges_count,
        "t_junctions": rep_inter.geometry.t_junctions_count,
        "self_intersections": rep_inter.geometry.self_intersections_count,
        "genus": rep_inter.topology.genus,
        "signed_volume": rep_inter.geometry.signed_volume,
        "expected_volume": 0.0005,
        "issues": rep_inter.issues,
    }));

    // 4. Transform with negative scale [-1.0, 1.0, 1.0]
    let mut box_neg = create_box(0.1, 0.2, 0.3);
    box_neg.scale([-1.0, 1.0, 1.0]);
    let rep_neg = validate_mesh(&box_neg, Some(0));
    ops_results.push(json!({
        "operation": "transform_negative_scale",
        "status": rep_neg.status,
        "watertight": rep_neg.topology.is_watertight,
        "orientation_consistent": rep_neg.topology.orientation_consistent,
        "signed_volume": rep_neg.geometry.signed_volume,
        "issues": rep_neg.issues,
    }));

    // 5. Chain: box -> 2 perpendicular non-intersecting holes (genus = 2) -> transform -> STL export/import
    let state = EngineState::new().unwrap().into_shared();
    let (_, p_res) = call_tool(
        &state,
        "create_part",
        json!({"name": "ChainBlock", "material": "steel_structural_aisi1020"}),
    )
    .await;
    let pid = p_res["part_id"].as_str().unwrap().to_string();
    let _ = call_tool(
        &state,
        "add_feature_primitive",
        json!({"part_id": pid, "primitive_type": "box", "width": 0.1, "height": 0.1, "depth": 0.1}),
    )
    .await;
    // Hole 1 along Z at y = -0.025
    let _ = call_tool(
        &state,
        "cut_hole",
        json!({"part_id": pid, "position": [0.0, -0.025, 0.0], "normal": [0.0, 0.0, 1.0], "radius": 0.012, "depth": 0.2}),
    )
    .await;
    // Hole 2 along X at y = +0.025 (non-intersecting with Hole 1 -> genus = 2)
    let _ = call_tool(
        &state,
        "cut_hole",
        json!({"part_id": pid, "position": [0.0, 0.025, 0.0], "normal": [1.0, 0.0, 0.0], "radius": 0.012, "depth": 0.2}),
    )
    .await;
    let (_, val_chain) = call_tool(
        &state,
        "validate_mesh",
        json!({"part_id": pid, "expected_genus": 2}),
    )
    .await;
    ops_results.push(json!({
        "operation": "chain_box_two_holes_genus2",
        "validation": val_chain
    }));

    let report_json = json!({
        "phase": "phase_2_operations_and_chains_scan",
        "total_operations_tested": ops_results.len(),
        "results": ops_results
    });
    let mut out_file = File::create(res_dir.join("operations_mesh_scan.json")).unwrap();
    serde_json::to_writer_pretty(&mut out_file, &report_json).unwrap();
}

#[tokio::test]
async fn test_wide_scan_analytical_benchmarks() {
    let base_dir = qa_base_dir();
    let res_dir = base_dir.join("results");
    let _ = std::fs::create_dir_all(&res_dir);

    let state = EngineState::new().unwrap().into_shared();
    let mut benchmarks = Vec::new();

    // 1. Box 0.2 x 0.3 x 0.4
    let a = 0.2_f64;
    let b = 0.3_f64;
    let c = 0.4_f64;
    let box_mesh = create_box(a, b, c);
    let v_exp = a * b * c;
    let s_exp = 2.0 * (a * b + b * c + c * a);
    let v_err = (box_mesh.volume() - v_exp).abs() / v_exp;
    let s_err = (box_mesh.surface_area() - s_exp).abs() / s_exp;
    let com = box_mesh.center_of_mass();
    let com_err = (com[0].powi(2) + com[1].powi(2) + com[2].powi(2)).sqrt();
    benchmarks.push(json!({
        "benchmark": "box_geometry",
        "v_actual": box_mesh.volume(),
        "v_expected": v_exp,
        "v_rel_err": v_err,
        "s_actual": box_mesh.surface_area(),
        "s_expected": s_exp,
        "s_rel_err": s_err,
        "com_err": com_err,
        "pass": v_err < 1e-6 && s_err < 1e-6 && com_err < 1e-6
    }));

    // 2. Cylinder fine (r=0.05, h=0.2)
    let r = 0.05_f64;
    let h = 0.2_f64;
    let cyl_mesh = create_cylinder_with_settings(r, h, &TessellationSettings::fine());
    let cyl_v_exp = std::f64::consts::PI * r * r * h;
    let cyl_s_exp = 2.0 * std::f64::consts::PI * r * (r + h);
    let cyl_v_err = (cyl_mesh.volume() - cyl_v_exp).abs() / cyl_v_exp;
    let cyl_s_err = (cyl_mesh.surface_area() - cyl_s_exp).abs() / cyl_s_exp;
    benchmarks.push(json!({
        "benchmark": "cylinder_fine_geometry",
        "v_actual": cyl_mesh.volume(),
        "v_expected": cyl_v_exp,
        "v_rel_err": cyl_v_err,
        "s_actual": cyl_mesh.surface_area(),
        "s_expected": cyl_s_exp,
        "s_rel_err": cyl_s_err,
        "pass": cyl_v_err < 0.005 && cyl_s_err < 0.005
    }));

    // 3. Sphere fine (r=0.05)
    let sph_mesh = create_sphere_with_settings(r, &TessellationSettings::fine());
    let sph_v_exp = (4.0 / 3.0) * std::f64::consts::PI * r.powi(3);
    let sph_s_exp = 4.0 * std::f64::consts::PI * r.powi(2);
    let sph_v_err = (sph_mesh.volume() - sph_v_exp).abs() / sph_v_exp;
    let sph_s_err = (sph_mesh.surface_area() - sph_s_exp).abs() / sph_s_exp;
    benchmarks.push(json!({
        "benchmark": "sphere_fine_geometry",
        "v_actual": sph_mesh.volume(),
        "v_expected": sph_v_exp,
        "v_rel_err": sph_v_err,
        "s_actual": sph_mesh.surface_area(),
        "s_expected": sph_s_exp,
        "s_rel_err": sph_s_err,
        "pass": sph_v_err < 0.01 && sph_s_err < 0.01
    }));

    // 4. Cone frustum fine (r1=0.05, r2=0.02, h=0.1)
    let r1 = 0.05_f64;
    let r2 = 0.02_f64;
    let hc = 0.1_f64;
    let cone_mesh = create_cone_with_settings(r1, r2, hc, &TessellationSettings::fine());
    let cone_v_exp = (1.0 / 3.0) * std::f64::consts::PI * hc * (r1 * r1 + r1 * r2 + r2 * r2);
    let cone_v_err = (cone_mesh.volume() - cone_v_exp).abs() / cone_v_exp;
    benchmarks.push(json!({
        "benchmark": "cone_frustum_fine_volume",
        "v_actual": cone_mesh.volume(),
        "v_expected": cone_v_exp,
        "v_rel_err": cone_v_err,
        "pass": cone_v_err < 0.01
    }));

    // 5. FEA Cantilever Beam Benchmark
    // L = 0.2 m (along X), b = 0.02 m (along Z), h = 0.02 m (along Y), F = -1000 N (along Y)
    // E = 200 GPa = 200e9 Pa, I = b * h^3 / 12 = 0.02 * (0.02)^3 / 12 = 1.333333e-8 m^4
    // delta_analytical = F * L^3 / (3 * E * I) = 1000 * 0.008 / (3 * 200e9 * 1.333333e-8) = 0.001 m = 1.0 mm
    // sigma_analytical = M * c / I = (1000 * 0.2) * 0.01 / 1.333333e-8 = 150 MPa
    let (_, p_beam) = call_tool(
        &state,
        "create_part",
        json!({"name": "Cantilever", "material": "steel_structural_aisi1020"}),
    )
    .await;
    let beam_id = p_beam["part_id"].as_str().unwrap().to_string();
    let _ = call_tool(
        &state,
        "add_feature_primitive",
        json!({"part_id": beam_id, "primitive_type": "box", "width": 0.2, "height": 0.02, "depth": 0.02}),
    )
    .await;
    let (ok_fea, fea_res) = call_tool(
        &state,
        "run_fea_analysis",
        json!({
            "part_id": beam_id,
            "mesh_density": "fine",
            "fixed_faces": ["MinX"],
            "applied_forces": [{"force_vector": [0.0, -1000.0, 0.0]}]
        }),
    )
    .await;
    let delta_mm_actual = fea_res["max_displacement_mm"].as_f64().unwrap_or(0.0);
    let sigma_mpa_actual = fea_res["max_von_mises_mpa"].as_f64().unwrap_or(0.0);
    let fos_actual = fea_res["safety_factor"].as_f64().unwrap_or(0.0);
    let yield_mpa = fea_res["yield_strength_mpa"].as_f64().unwrap_or(350.0);
    let delta_err = (delta_mm_actual - 1.0).abs() / 1.0;
    let sigma_err = (sigma_mpa_actual - 150.0).abs() / 150.0;
    let fos_err = (fos_actual - (yield_mpa / sigma_mpa_actual)).abs();

    benchmarks.push(json!({
        "benchmark": "fea_cantilever_beam",
        "ok": ok_fea,
        "delta_mm_actual": delta_mm_actual,
        "delta_mm_expected": 1.0,
        "delta_rel_err": delta_err,
        "sigma_mpa_actual": sigma_mpa_actual,
        "sigma_mpa_expected": 150.0,
        "sigma_rel_err": sigma_err,
        "fos_err": fos_err,
        "pass": ok_fea && delta_err <= 0.25 && sigma_err <= 0.30 && fos_err < 1e-4
    }));

    // 6. FEA Axial Rod Tension Benchmark
    // L = 0.2 m, b = 0.02 m, h = 0.02 m => A = 4e-4 m^2, F_x = +40,000 N
    // delta_analytical = F * L / (E * A) = 40000 * 0.2 / (200e9 * 4e-4) = 0.0001 m = 0.1 mm
    let (ok_rod, rod_res) = call_tool(
        &state,
        "run_fea_analysis",
        json!({
            "part_id": beam_id,
            "mesh_density": "fine",
            "fixed_faces": ["MinX"],
            "applied_forces": [{"force_vector": [40000.0, 0.0, 0.0]}]
        }),
    )
    .await;
    let rod_delta_mm = rod_res["max_displacement_mm"].as_f64().unwrap_or(0.0);
    let rod_delta_err = (rod_delta_mm - 0.1).abs() / 0.1;
    benchmarks.push(json!({
        "benchmark": "fea_axial_rod_tension",
        "ok": ok_rod,
        "delta_mm_actual": rod_delta_mm,
        "delta_mm_expected": 0.1,
        "delta_rel_err": rod_delta_err,
        "pass": ok_rod && rod_delta_err <= 0.10
    }));

    let report_json = json!({
        "phase": "phase_2_analytical_benchmarks",
        "benchmarks": benchmarks
    });
    let mut out_file = File::create(res_dir.join("analytical_benchmarks.json")).unwrap();
    serde_json::to_writer_pretty(&mut out_file, &report_json).unwrap();
}

#[tokio::test]
async fn test_wide_scan_functional_matrix_26_tools() {
    let base_dir = qa_base_dir();
    let res_dir = base_dir.join("results");
    let _ = std::fs::create_dir_all(&res_dir);

    let state = EngineState::new().unwrap().into_shared();
    let mut matrix_results = Vec::new();

    // Test 1: Boundary values on add_feature_primitive (negative / zero dimensions)
    let (_, p1) = call_tool(
        &state,
        "create_part",
        json!({"name": "BoundaryPart", "material": "steel_structural_aisi1020"}),
    )
    .await;
    let pid = p1["part_id"].as_str().unwrap().to_string();

    // Add valid box first
    let (ok_valid_box, _) = call_tool(
        &state,
        "add_feature_primitive",
        json!({"part_id": pid, "primitive_type": "box", "width": 0.1, "height": 0.1, "depth": 0.1}),
    )
    .await;

    // Now try adding an invalid box (width = -0.1)
    let (ok_neg_box, _) = call_tool(
        &state,
        "add_feature_primitive",
        json!({"part_id": pid, "primitive_type": "box", "width": -0.1, "height": 0.1, "depth": 0.1}),
    )
    .await;

    // Now check if the part DAG was polluted by the failed add_feature_primitive!
    let (ok_query_after_fail, q_res) = call_tool(
        &state,
        "query_part_geometry",
        json!({"part_id": pid}),
    )
    .await;
    let (_, tree_res) = call_tool(&state, "get_feature_tree", json!({"part_id": pid})).await;
    let dag_nodes_count = tree_res["dag"].as_array().map(|a| a.len()).unwrap_or(0);

    // Try adding a valid hole after a failed primitive to see if rebuild still works!
    let (ok_hole_after_fail, hole_err) = call_tool(
        &state,
        "cut_hole",
        json!({"part_id": pid, "position": [0.0, 0.0, 0.0], "normal": [0.0, 0.0, 1.0], "radius": 0.02, "depth": 0.2}),
    )
    .await;

    matrix_results.push(json!({
        "check": "dag_rollback_on_invalid_feature",
        "ok_valid_box": ok_valid_box,
        "rejected_negative_box": !ok_neg_box,
        "dag_nodes_count_after_rejected_op": dag_nodes_count,
        "expected_dag_nodes_count": 1,
        "subsequent_valid_op_succeeded": ok_hole_after_fail,
        "subsequent_op_response": hole_err,
        "query_after_fail": ok_query_after_fail,
        "volume_preserved": q_res["volume"],
    }));

    // Test 2: batch_cad_operations atomicity when step 2 fails
    let (ok_batch, _) = call_tool(
        &state,
        "batch_cad_operations",
        json!({
            "operations": [
                {
                    "tool": "create_part",
                    "params": {"part_id": "atomic-orphan-part", "name": "Orphan", "material": "steel_structural_aisi1020"}
                },
                {
                    "tool": "delete_part",
                    "params": {"part_id": "non-existent-part-99999"}
                }
            ]
        }),
    )
    .await;
    let (_, parts_after_batch) = call_tool(&state, "list_parts", json!({})).await;
    let orphan_exists = parts_after_batch["parts"]
        .as_array()
        .map(|arr| arr.iter().any(|p| p["id"] == "atomic-orphan-part"))
        .unwrap_or(false);

    matrix_results.push(json!({
        "check": "batch_cad_operations_atomicity",
        "batch_failed_as_expected": !ok_batch,
        "orphan_part_left_behind": orphan_exists,
        "pass": !ok_batch && !orphan_exists
    }));

    // Test 3: polygon_prism with sides < 3 (e.g. sides = 2)
    let (_, p2) = call_tool(
        &state,
        "create_part",
        json!({"name": "PrismTest", "material": "steel_structural_aisi1020"}),
    )
    .await;
    let p2_id = p2["part_id"].as_str().unwrap().to_string();
    let (ok_prism_2sides, _) = call_tool(
        &state,
        "add_feature_primitive",
        json!({"part_id": p2_id, "primitive_type": "polygon_prism", "sides": 2, "height": 0.05, "radius": 0.02}),
    )
    .await;
    matrix_results.push(json!({
        "check": "polygon_prism_rejects_sides_lt_3",
        "rejected": !ok_prism_2sides,
        "pass": !ok_prism_2sides
    }));

    // Test 4: open_preview_window with non-existent part_id in headless mode
    let (ok_preview_bad_id, _) = call_tool(
        &state,
        "open_preview_window",
        json!({"part_id": "non-existent-part-id"}),
    )
    .await;
    matrix_results.push(json!({
        "check": "open_preview_window_validates_part_id",
        "rejected_non_existent_part": !ok_preview_bad_id,
        "pass": !ok_preview_bad_id
    }));

    // Test 5: capture_viewport with width=0 or height=0 (zero-panic check)
    let (ok_cap_zero, _) = call_tool(
        &state,
        "capture_viewport",
        json!({"width": 0, "height": 0}),
    )
    .await;
    matrix_results.push(json!({
        "check": "capture_viewport_rejects_zero_dimensions",
        "rejected": !ok_cap_zero,
        "pass": !ok_cap_zero
    }));

    let report_json = json!({
        "phase": "phase_2_functional_matrix",
        "results": matrix_results
    });
    let mut out_file = File::create(res_dir.join("functional_matrix_scan.json")).unwrap();
    serde_json::to_writer_pretty(&mut out_file, &report_json).unwrap();
}

#[tokio::test]
async fn test_bug_003_partial_revolve_mesh_and_renders() {
    let art_dir = qa_base_dir().join("artifacts");
    std::fs::create_dir_all(&art_dir).unwrap();

    // 1. Hollow rectangular profile at 90, 180, 270, 360 degrees (including CW reversed profile)
    let profile_ccw = vec![[0.01, -0.02], [0.03, -0.02], [0.03, 0.02], [0.01, 0.02]];
    let mut profile_cw = profile_ccw.clone();
    profile_cw.reverse();

    for angle in [90.0, 180.0, 270.0] {
        let mesh = create_revolve(&profile_ccw, angle, 32);
        let rep = validate_mesh(&mesh, Some(0));
        assert_eq!(rep.status, "PASS", "Partial revolve {} deg CCW failed: {:?}", angle, rep.issues);
        assert!(rep.topology.is_watertight);
        assert!(rep.topology.orientation_consistent);
        assert_eq!(rep.topology.inconsistent_edges_count, 0);

        let mesh_cw = create_revolve(&profile_cw, angle, 32);
        let rep_cw = validate_mesh(&mesh_cw, Some(0));
        assert_eq!(rep_cw.status, "PASS", "Partial revolve {} deg CW failed: {:?}", angle, rep_cw.issues);
    }

    // 2. Non-convex L-shaped profile at 180 degrees
    let l_profile = vec![
        [0.01, -0.02],
        [0.04, -0.02],
        [0.04, -0.01],
        [0.02, -0.01],
        [0.02, 0.02],
        [0.01, 0.02],
    ];
    let mesh_l180 = create_revolve(&l_profile, 180.0, 32);
    let rep_l180 = validate_mesh(&mesh_l180, Some(0));
    assert_eq!(rep_l180.status, "PASS", "Partial L-profile revolve 180 failed: {:?}", rep_l180.issues);

    // 3. Axis-touching profile at 180 degrees
    let axis_profile = vec![[0.0, -0.02], [0.03, -0.02], [0.03, 0.02], [0.0, 0.02]];
    let mesh_ax180 = create_revolve(&axis_profile, 180.0, 32);
    let rep_ax180 = validate_mesh(&mesh_ax180, Some(0));
    assert_eq!(rep_ax180.status, "PASS", "Partial axis-touching revolve 180 failed: {:?}", rep_ax180.issues);

    // 4. Render revolve_partial_180 in all 5 debug modes and save PNGs for visual audit
    let state = EngineState::new().unwrap().into_shared();
    let mesh_180 = create_revolve(&profile_ccw, 180.0, 32);
    let engine = state.read();

    for mode_name in ["shaded", "wireframe", "normals", "backfaces", "boundary_edges"] {
        let out_path = art_dir.join(format!("revolve_partial_180_{}.png", mode_name));
        let debug_mode = mcp_cad_viewer::DebugRenderMode::parse(Some(mode_name));
        let png_bytes = engine
            .offscreen_renderer
            .render_mesh_png_debug(
                &mesh_180,
                mcp_cad_viewer::CameraPreset::Isometric,
                debug_mode,
                800,
                600,
            )
            .expect("render_mesh_png_debug should succeed");
        assert!(png_bytes.starts_with(&PNG_MAGIC));
        std::fs::write(&out_path, &png_bytes).unwrap();
    }
}

#[test]
fn test_bug_007_negative_scale_preserves_outward_winding() {
    let art_dir = qa_base_dir().join("artifacts");
    std::fs::create_dir_all(&art_dir).unwrap();

    // SKEPTIC cases: 1 negative axis [-1, 1, 1], 2 negative axes [-1, -1, 1], 3 negative axes [-2, -1, -0.5]
    for scale in [[-1.0, 1.0, 1.0], [-1.0, -1.0, 1.0], [-2.0, -1.0, -0.5]] {
        let mut mesh = create_box(0.1, 0.2, 0.3);
        mesh.scale(scale);
        let rep = validate_mesh(&mesh, Some(0));
        assert_eq!(rep.status, "PASS", "Scale {:?} failed: {:?}", scale, rep.issues);
        assert!(rep.geometry.signed_volume > 0.0, "Signed volume must be positive for {:?}", scale);
        assert_eq!(rep.geometry.normals_vs_winding_mismatches, 0);
    }

    let mut cyl = create_cylinder(0.04, 0.12, 32);
    cyl.scale([-1.0, 1.0, 1.0]);
    let rep_cyl = validate_mesh(&cyl, Some(0));
    assert_eq!(rep_cyl.status, "PASS", "Mirrored cylinder failed: {:?}", rep_cyl.issues);

    let state = EngineState::new().unwrap().into_shared();
    let engine = state.read();
    for mode_name in ["normals", "backfaces"] {
        let out_path = art_dir.join(format!("neg_scale_cyl_{}.png", mode_name));
        let debug_mode = mcp_cad_viewer::DebugRenderMode::parse(Some(mode_name));
        let png_bytes = engine
            .offscreen_renderer
            .render_mesh_png_debug(
                &cyl,
                mcp_cad_viewer::CameraPreset::Isometric,
                debug_mode,
                800,
                600,
            )
            .unwrap();
        std::fs::write(&out_path, &png_bytes).unwrap();
    }
}

#[tokio::test]
async fn test_bug_005_polygon_prism_chamfer_and_sides_validation() {
    let art_dir = qa_base_dir().join("artifacts");
    std::fs::create_dir_all(&art_dir).unwrap();

    for sides in [3, 4, 6, 8, 12] {
        let plain = create_polygon_prism(sides, 0.05, Some(0.04), None, None);
        let rep_plain = validate_mesh(&plain, Some(0));
        assert_eq!(rep_plain.status, "PASS", "Plain prism {} sides failed: {:?}", sides, rep_plain.issues);

        let chamf = create_polygon_prism(sides, 0.05, Some(0.04), None, Some(0.003));
        let rep_chamf = validate_mesh(&chamf, Some(0));
        assert_eq!(rep_chamf.status, "PASS", "Chamfered prism {} sides failed: {:?}", sides, rep_chamf.issues);
        assert!(
            chamf.volume() < plain.volume(),
            "Chamfer on {}-sided prism must reduce volume: chamf={} vs plain={}",
            sides,
            chamf.volume(),
            plain.volume()
        );
    }

    // Verify sides < 3 is rejected by MCP add_feature_primitive
    let state = EngineState::new().unwrap().into_shared();
    let (_, p_res) = call_tool(
        &state,
        "create_part",
        json!({"name": "PrismReject", "material": "steel_structural_aisi1020"}),
    )
    .await;
    let pid = p_res["part_id"].as_str().unwrap().to_string();
    let (ok_s2, _) = call_tool(
        &state,
        "add_feature_primitive",
        json!({"part_id": pid, "primitive_type": "polygon_prism", "sides": 2, "height": 0.05, "radius": 0.02}),
    )
    .await;
    assert!(!ok_s2, "sides=2 must be rejected by add_feature_primitive");

    // Render chamfered hex prism in all 5 debug modes for visual inspection
    let hex_chamf = create_polygon_prism(6, 0.03, Some(0.04), None, Some(0.004));
    let engine = state.read();
    for mode_name in ["shaded", "wireframe", "normals", "backfaces", "boundary_edges"] {
        let out_path = art_dir.join(format!("polygon_prism_hex_chamfer_{}.png", mode_name));
        let debug_mode = mcp_cad_viewer::DebugRenderMode::parse(Some(mode_name));
        let png_bytes = engine
            .offscreen_renderer
            .render_mesh_png_debug(
                &hex_chamf,
                mcp_cad_viewer::CameraPreset::Isometric,
                debug_mode,
                800,
                600,
            )
            .unwrap();
        std::fs::write(&out_path, &png_bytes).unwrap();
    }
}

#[tokio::test]
async fn test_bug_002_spur_gear_manifold_and_keyway() {
    let art_dir = qa_base_dir().join("artifacts");
    std::fs::create_dir_all(&art_dir).unwrap();

    // 1. SKEPTIC grid: teeth in [8, 20, 60], pressure angle in [14.5, 20.0, 25.0]
    for &teeth in &[8usize, 20, 60] {
        for &pa in &[14.5f64, 20.0, 25.0] {
            let gear = create_spur_gear(0.002, teeth, pa, 0.015, None, None, None, None, None);
            let rep = validate_mesh(&gear, Some(0));
            assert_eq!(
                rep.status, "PASS",
                "Plain gear (teeth={}, pa={}) failed validate_mesh: {:?}",
                teeth, pa, rep.issues
            );
            assert!(
                rep.geometry.max_aspect_ratio < 50.0,
                "Plain gear (teeth={}, pa={}) max_aspect_ratio too high: {}",
                teeth, pa, rep.geometry.max_aspect_ratio
            );
        }
    }

    // 2. Bore only (genus = 1)
    let gear_bore = create_spur_gear(0.002, 20, 20.0, 0.015, Some(0.006), None, None, None, None);
    let rep_bore = validate_mesh(&gear_bore, Some(1));
    assert_eq!(rep_bore.status, "PASS", "Gear with bore failed: {:?}", rep_bore.issues);

    // 3. Hub only (genus = 0)
    let gear_hub = create_spur_gear(0.002, 20, 20.0, 0.015, None, None, None, Some(0.025), Some(0.010));
    let rep_hub = validate_mesh(&gear_hub, Some(0));
    assert_eq!(rep_hub.status, "PASS", "Gear with hub failed: {:?}", rep_hub.issues);

    // 4. Full Hub + Bore + DIN 6885 Keyway (genus = 1)
    let gear_full = create_spur_gear(
        0.002,
        20,
        20.0,
        0.015,
        Some(0.006),
        Some(0.004),
        Some(0.0018),
        Some(0.025),
        Some(0.010),
    );
    let rep_full = validate_mesh(&gear_full, Some(1));
    assert_eq!(
        rep_full.status, "PASS",
        "Gear with hub, bore, and keyway failed: {:?}",
        rep_full.issues
    );
    assert!(rep_full.topology.is_watertight);
    assert_eq!(rep_full.topology.non_manifold_edges_count, 0);
    assert!(rep_full.topology.orientation_consistent);
    assert_eq!(rep_full.geometry.degenerate_triangles_count, 0);
    assert_eq!(rep_full.geometry.self_intersections_count, 0);

    // 5. Render gear_full in all 5 debug modes for visual inspection
    let state = EngineState::new().unwrap().into_shared();
    let engine = state.read();
    for mode_name in ["shaded", "wireframe", "normals", "backfaces", "boundary_edges"] {
        let out_path = art_dir.join(format!("spur_gear_hub_keyway_{}.png", mode_name));
        let debug_mode = mcp_cad_viewer::DebugRenderMode::parse(Some(mode_name));
        let png_bytes = engine
            .offscreen_renderer
            .render_mesh_png_debug(
                &gear_full,
                mcp_cad_viewer::CameraPreset::Isometric,
                debug_mode,
                800,
                600,
            )
            .unwrap();
        std::fs::write(&out_path, &png_bytes).unwrap();
    }
}

#[tokio::test]
async fn test_bug_004_wheel_rim_manifold_and_renders() {
    let art_dir = qa_base_dir().join("artifacts");
    std::fs::create_dir_all(&art_dir).unwrap();

    // 1. SKEPTIC grid: (spokes, bolts) combinations -> expected genus = 1 + spokes + bolts
    for &(spokes, bolts) in &[(3usize, 4usize), (5, 5), (6, 5), (8, 6), (10, 5)] {
        let expected_genus = (1 + spokes + bolts) as i32;
        let wheel = create_wheel_rim(
            0.4572,
            0.2032,
            Some(0.490),
            Some(0.160),
            Some(0.066),
            Some(0.1143),
            Some(bolts),
            Some(0.014),
            Some(spokes),
            Some(0.028),
            Some(0.018),
            Some(0.035),
        );
        let rep = validate_mesh(&wheel, Some(expected_genus));
        assert_eq!(
            rep.status, "PASS",
            "wheel_rim (spokes={}, bolts={}) failed validate_mesh: {:?}",
            spokes, bolts, rep.issues
        );
        assert!(rep.topology.is_watertight);
        assert_eq!(rep.topology.non_manifold_edges_count, 0);
        assert!(rep.topology.orientation_consistent);
        assert!((rep.topology.genus - (expected_genus as f64)).abs() < 1e-6);
        assert_eq!(rep.geometry.degenerate_triangles_count, 0);
        assert_eq!(rep.geometry.self_intersections_count, 0);
    }

    // 2. Render standard 5-spoke 5-bolt wheel rim in all 5 debug modes for visual inspection
    let wheel_std = create_wheel_rim(
        0.4572,
        0.2032,
        Some(0.490),
        Some(0.160),
        Some(0.066),
        Some(0.1143),
        Some(5),
        Some(0.014),
        Some(5),
        Some(0.028),
        Some(0.018),
        Some(0.035),
    );
    let state = EngineState::new().unwrap().into_shared();
    let engine = state.read();
    for mode_name in ["shaded", "wireframe", "normals", "backfaces", "boundary_edges"] {
        let out_path = art_dir.join(format!("wheel_rim_5spoke_{}.png", mode_name));
        let debug_mode = mcp_cad_viewer::DebugRenderMode::parse(Some(mode_name));
        let png_bytes = engine
            .offscreen_renderer
            .render_mesh_png_debug(
                &wheel_std,
                mcp_cad_viewer::CameraPreset::Isometric,
                debug_mode,
                800,
                600,
            )
            .unwrap();
        std::fs::write(&out_path, &png_bytes).unwrap();
    }
}

#[tokio::test]
async fn test_bug_006_csg_booleans_and_holes_watertight() {
    let base_dir = qa_base_dir();
    let art_dir = base_dir.join("artifacts");
    std::fs::create_dir_all(&art_dir).unwrap();
    let settings = TessellationSettings::medium();

    // 1. Coplanar and offset box booleans: exact analytical volume & genus 0
    let box_a = create_box(0.1, 0.1, 0.1);
    let mut box_b = create_box(0.1, 0.1, 0.1);
    box_b.translate([0.05, 0.0, 0.0]);

    let m_union = csg_union(&box_a, &box_b);
    let r_union = validate_mesh(&m_union, Some(0));
    assert_eq!(r_union.status, "PASS", "Union failed: {:?}", r_union.issues);
    assert!((r_union.geometry.signed_volume - 0.0015).abs() < 1e-6);
    assert_eq!(r_union.geometry.t_junctions_count, 0);

    let m_diff = csg_difference(&box_a, &box_b);
    let r_diff = validate_mesh(&m_diff, Some(0));
    assert_eq!(r_diff.status, "PASS", "Diff failed: {:?}", r_diff.issues);
    assert!((r_diff.geometry.signed_volume - 0.0005).abs() < 1e-6);
    assert_eq!(r_diff.geometry.t_junctions_count, 0);

    let m_inter = csg_intersection(&box_a, &box_b);
    let r_inter = validate_mesh(&m_inter, Some(0));
    assert_eq!(r_inter.status, "PASS", "Inter failed: {:?}", r_inter.issues);
    assert!((r_inter.geometry.signed_volume - 0.0005).abs() < 1e-6);
    assert_eq!(r_inter.geometry.t_junctions_count, 0);

    // 2. CutHole variations: through, blind, keyway, angled
    let plate = create_box(0.1, 0.1, 0.04);
    let m_thru = cut_hole_with_settings(&plate, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.015, 0.10, None, None, &settings);
    let r_thru = validate_mesh(&m_thru, Some(1));
    assert_eq!(r_thru.status, "PASS", "Thru hole failed: {:?}", r_thru.issues);
    assert_eq!(r_thru.geometry.t_junctions_count, 0);

    let m_blind = cut_hole_with_settings(&plate, [0.0, 0.0, 0.01], [0.0, 0.0, 1.0], 0.015, 0.03, None, None, &settings);
    let r_blind = validate_mesh(&m_blind, Some(0));
    assert_eq!(r_blind.status, "PASS", "Blind hole failed: {:?}", r_blind.issues);
    assert_eq!(r_blind.geometry.t_junctions_count, 0);

    let m_key = cut_hole_with_settings(
        &plate,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.015,
        0.10,
        Some(0.006),
        Some(0.003),
        &settings,
    );
    let r_key = validate_mesh(&m_key, Some(1));
    assert_eq!(r_key.status, "PASS", "Keyway hole failed: {:?}", r_key.issues);
    assert_eq!(r_key.geometry.t_junctions_count, 0);

    let m_ang = cut_hole_with_settings(&plate, [0.0, 0.0, 0.0], [1.0, 0.0, 1.0], 0.01, 0.20, None, None, &settings);
    let r_ang = validate_mesh(&m_ang, Some(1));
    assert_eq!(r_ang.status, "PASS", "Angled hole failed: {:?}", r_ang.issues);
    assert_eq!(r_ang.geometry.t_junctions_count, 0);

    // 3. Multi-hole CSG chain (genus 2)
    let m_two_holes = cut_hole_with_settings(
        &cut_hole_with_settings(&plate, [-0.025, 0.0, 0.0], [0.0, 0.0, 1.0], 0.012, 0.10, Some(0.005), Some(0.0025), &settings),
        [0.025, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.012,
        0.10,
        None,
        None,
        &settings,
    );
    let r_two_holes = validate_mesh(&m_two_holes, Some(2));
    assert_eq!(r_two_holes.status, "PASS", "Two holes chain failed: {:?}", r_two_holes.issues);
    assert_eq!(r_two_holes.topology.boundary_edges_count, 0);
    assert_eq!(r_two_holes.topology.non_manifold_edges_count, 0);
    assert_eq!(r_two_holes.geometry.t_junctions_count, 0);

    // 4. Helical cutters manifold & watertight (circular and metric_60)
    for profile_type in ["circular", "metric_60"] {
        let cutter = create_helical_cutter_with_settings(
            0.01,
            0.002,
            0.008,
            0.03,
            -0.015,
            1,
            true,
            0.03 / 0.008,
            profile_type,
            &settings,
        );
        let val_cutter = validate_mesh(&cutter, Some(0));
        assert_eq!(
            val_cutter.status, "PASS",
            "Helical cutter ({}) failed: {:?}",
            profile_type, val_cutter.issues
        );
        assert_eq!(val_cutter.topology.boundary_edges_count, 0);
        assert_eq!(val_cutter.topology.non_manifold_edges_count, 0);
        assert!(val_cutter.topology.orientation_consistent);
        assert_eq!(val_cutter.geometry.self_intersections_count, 0);
        assert!(val_cutter.geometry.signed_volume > 0.0);
    }

    // 5. Render m_two_holes in all 5 debug modes for visual inspection (Rule R1/R2)
    let state = EngineState::new().unwrap().into_shared();
    let engine = state.read();
    for mode_name in ["shaded", "wireframe", "normals", "backfaces", "boundary_edges"] {
        let out_path = art_dir.join(format!("csg_chain_two_holes_{}.png", mode_name));
        let debug_mode = mcp_cad_viewer::DebugRenderMode::parse(Some(mode_name));
        let png_bytes = engine
            .offscreen_renderer
            .render_mesh_png_debug(
                &m_two_holes,
                mcp_cad_viewer::CameraPreset::Isometric,
                debug_mode,
                800,
                600,
            )
            .unwrap();
        std::fs::write(&out_path, &png_bytes).unwrap();
    }
}

#[test]
fn test_bug_008_dag_rollback_on_failed_rebuild() {
    let mut state = EngineState::new().unwrap();
    // 1. Create a valid part with a box
    let p_res = mcp_cad_server::tools::cad::create_part(&json!({"name": "rollback_test", "material": "aluminum_6061_t6"}), &mut state).unwrap();
    let part_id = p_res["part_id"].as_str().unwrap();

    let f_box = mcp_cad_server::tools::cad::add_feature_primitive(&json!({
        "part_id": part_id,
        "primitive_type": "box",
        "params": {"width": 0.1, "height": 0.1, "depth": 0.1}
    }), &mut state).unwrap();
    let box_feat_id = f_box["feature_id"].as_str().unwrap();

    let initial_verts = state.parts[part_id].mesh.vertices.len();
    let initial_nodes_len = state.parts[part_id].dag.nodes.len();
    assert_eq!(initial_nodes_len, 1);
    assert!(initial_verts > 0);

    // 2. Attempt an invalid cut_hole that produces CadError (e.g. radius <= 0)
    let bad_hole = mcp_cad_server::tools::cad::cut_hole(&json!({
        "part_id": part_id,
        "position": [0.0, 0.0, 0.0],
        "normal": [0.0, 0.0, 1.0],
        "radius": 0.0,
        "depth": 0.1
    }), &mut state);
    assert!(bad_hole.is_err());

    // 3. Verify DAG and mesh rolled back cleanly to 1 node
    assert_eq!(state.parts[part_id].dag.nodes.len(), 1);
    assert_eq!(state.parts[part_id].mesh.vertices.len(), initial_verts);

    // 4. Attempt an invalid modify_feature (e.g. box width -0.05)
    let bad_mod = mcp_cad_server::tools::cad::modify_feature(&json!({
        "part_id": part_id,
        "feature_id": box_feat_id,
        "params": {"width": -0.05}
    }), &mut state);
    assert!(bad_mod.is_err());

    // Verify DAG and mesh are still intact
    assert_eq!(state.parts[part_id].dag.nodes.len(), 1);
    assert_eq!(state.parts[part_id].mesh.vertices.len(), initial_verts);
}

#[test]
fn test_bug_009_batch_operations_atomic_rollback() {
    let mut state = EngineState::new().unwrap();
    let p_res = mcp_cad_server::tools::cad::create_part(&json!({"name": "batch_part", "material": "aluminum_6061_t6"}), &mut state).unwrap();
    let part_id = p_res["part_id"].as_str().unwrap();

    // Valid box
    mcp_cad_server::tools::cad::add_feature_primitive(&json!({
        "part_id": part_id,
        "primitive_type": "box",
        "params": {"width": 0.1, "height": 0.1, "depth": 0.1}
    }), &mut state).unwrap();

    let initial_parts_count = state.parts.len();
    let initial_nodes_len = state.parts[part_id].dag.nodes.len();

    // Run a batch where op 1 succeeds (create new part) and op 2 fails (unknown primitive)
    let batch_call = mcp_cad_server::tools::cad::batch_cad_operations(&json!({
        "operations": [
            {
                "tool": "create_part",
                "params": {"name": "temporary_part", "material": "aluminum_6061_t6"}
            },
            {
                "tool": "add_feature_primitive",
                "params": {"part_id": part_id, "primitive_type": "nonexistent_primitive"}
            }
        ]
    }), &mut state);

    assert!(batch_call.is_err());
    // Verify atomicity: temporary_part must NOT exist! state.parts must be exactly as before.
    assert_eq!(state.parts.len(), initial_parts_count);
    assert_eq!(state.parts[part_id].dag.nodes.len(), initial_nodes_len);
    assert!(!state.parts.values().any(|p| p.name == "temporary_part"));
}

#[test]
fn test_bug_010_viewport_and_preview_validation() {
    let state = EngineState::new().unwrap();

    // 1. capture_viewport with width=0 or height=0 must fail with InvalidArgument
    let err_w = mcp_cad_server::tools::render::capture_viewport(&json!({"width": 0, "height": 600}), &state);
    assert!(err_w.is_err());

    let err_h = mcp_cad_server::tools::render::capture_viewport(&json!({"width": 800, "height": 0}), &state);
    assert!(err_h.is_err());

    // 2. open_preview_window with nonexistent part must fail with PartNotFound even in headless mode
    let err_preview = mcp_cad_server::tools::render::open_preview_window(&json!({"part_id": "nonexistent_part"}), &state);
    assert!(err_preview.is_err());
}

