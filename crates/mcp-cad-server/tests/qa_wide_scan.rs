//! Phase 2: Autonomous Wide Scanner for MCP-CAD Engine.
//! Executes mesh-audit matrices, analytical benchmarks, and functional tool matrices,
//! dumping structured JSON results to `qa/results/` and renders to `qa/artifacts/`.

use serde_json::{json, Value};
use std::fs::File;
use std::io::Write;
use std::path::Path;

use mcp_cad_core::diagnostics::validate_mesh;
use mcp_cad_core::primitives::*;
use mcp_cad_core::tessellation::TessellationSettings;
use mcp_cad_server::protocol::handle_message;
use mcp_cad_server::state::EngineState;
use mcp_cad_viewer::CameraPreset;

#[allow(dead_code)]
const PNG_MAGIC: [u8; 8] = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];

#[allow(dead_code)]
async fn rpc_call(state: &mcp_cad_server::SharedState, req: Value) -> Value {
    let req_str = req.to_string();
    let resp = handle_message(&req_str, state)
        .await
        .expect("Expected a JSON-RPC response");
    serde_json::to_value(&resp).expect("Failed to serialize response")
}

#[tokio::test]
async fn test_wide_scan_primitives_mesh_quality() {
    let state = EngineState::new().unwrap().into_shared();

    // Directory for artifacts
    let base_dir = if Path::new("../../qa").exists() {
        std::path::PathBuf::from("../../qa")
    } else {
        std::path::PathBuf::from("qa")
    };
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
        // Sharp cone
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

        // Frustum cone
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
            let mesh = create_polygon_prism(sides, 0.05, Some(0.04), None, if chamfer > 0.0 { Some(chamfer) } else { None });
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
        // plain gear
        let mesh_plain = create_spur_gear(0.002, teeth, 0.015, 20.0, None, None, None, None, None);
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

        // gear with hub, bore hole, and keyway
        let mesh_hub = create_spur_gear(
            0.002,
            teeth,
            0.015,
            20.0,
            Some(0.006),
            Some(0.003),
            Some(0.002),
            Some(0.025),
            Some(0.010),
        );
        // A gear with a thru bore hole has 1 through-hole -> genus should be 1!
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

    // 7. Revolve (closed profile around Z axis)
    let rev_profile = vec![[0.01, -0.02], [0.03, -0.02], [0.03, 0.02], [0.01, 0.02]];
    let mesh_rev360 = create_revolve(&rev_profile, 360.0, 32);
    // Hollow ring revolve (has 1 central through-hole) -> genus = 1
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

    // Partial revolve with end-caps
    let mesh_rev180 = create_revolve(&rev_profile, 180.0, 32);
    // Partial ring with end caps is a topological sphere (genus = 0)
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

    // 8. Wheel Rim
    let mesh_wheel = create_wheel_rim(
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
    let rep_wheel = validate_mesh(&mesh_wheel, None);
    scan_results.push(json!({
        "target": "wheel_rim_5spoke",
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

    // Render diagnostic images for spur gear with hub and wheel rim
    let engine = state.read();
    let gear_png = engine
        .offscreen_renderer
        .render_mesh_png_debug(
            &mesh_rev360,
            CameraPreset::Isometric,
            mcp_cad_viewer::DebugRenderMode::Wireframe,
            800,
            600,
        )
        .expect("Render revolve wireframe");
    let mut f = File::create(art_dir.join("revolve_ring_wireframe.png")).unwrap();
    f.write_all(&gear_png).unwrap();

    let wheel_normals_png = engine
        .offscreen_renderer
        .render_mesh_png_debug(
            &mesh_wheel,
            CameraPreset::Isometric,
            mcp_cad_viewer::DebugRenderMode::Normals,
            800,
            600,
        )
        .expect("Render wheel rim normals");
    let mut f = File::create(art_dir.join("wheel_rim_normals.png")).unwrap();
    f.write_all(&wheel_normals_png).unwrap();

    let wheel_wire_png = engine
        .offscreen_renderer
        .render_mesh_png_debug(
            &mesh_wheel,
            CameraPreset::Isometric,
            mcp_cad_viewer::DebugRenderMode::Wireframe,
            800,
            600,
        )
        .expect("Render wheel rim wireframe");
    let mut f = File::create(art_dir.join("wheel_rim_wireframe.png")).unwrap();
    f.write_all(&wheel_wire_png).unwrap();

    // Write primitive scan report
    let report_json = json!({
        "phase": "phase_2_primitives_scan",
        "timestamp": format!("{:?}", std::time::SystemTime::now()),
        "total_primitives_tested": scan_results.len(),
        "results": scan_results
    });

    let mut out_file = File::create(res_dir.join("primitives_mesh_scan.json")).unwrap();
    serde_json::to_writer_pretty(&mut out_file, &report_json).unwrap();
    println!("Primitives scan complete: {} models evaluated", report_json["total_primitives_tested"]);
}
