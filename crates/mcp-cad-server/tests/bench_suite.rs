//! Benchmark suite runner executing the 15 engineering tasks from `bench/tasks.json`
//! against the native Rust `mcp-cad-server` via JSON-RPC (`handle_message`).
//!
//! Run with:
//!   cargo test -p mcp-cad-server --test bench_suite -- --nocapture

use std::collections::HashMap;
use std::path::PathBuf;

use serde::Deserialize;
use serde_json::{json, Value};

use mcp_cad_server::protocol::handle_message;
use mcp_cad_server::state::EngineState;

#[derive(Debug, Clone, Deserialize)]
struct BenchmarkSpec {
    tasks: Vec<BenchmarkTask>,
}

#[derive(Debug, Clone, Deserialize)]
struct BenchmarkTask {
    id: String,
    #[allow(dead_code)]
    prompt_ru: String,
    target_part_id: String,
    expected_dimensions_m: [f64; 3],
    expected_genus: i32,
    #[serde(default)]
    expected_fea: Option<ExpectedFea>,
    #[serde(default)]
    expected_kinematics: Option<ExpectedKinematics>,
    steps: Vec<BenchmarkStep>,
}

#[derive(Debug, Clone, Deserialize)]
struct ExpectedFea {
    verdict: String,
    stress_mpa: f64,
    #[serde(default)]
    displacement_mm: Option<f64>,
    #[serde(default)]
    tolerance_pct: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
struct ExpectedKinematics {
    #[serde(rename = "type")]
    kin_type: String,
    #[serde(default)]
    driver_id: Option<String>,
    #[serde(default)]
    driven_id: Option<String>,
    #[serde(default)]
    expected_velocity_ratio: Option<f64>,
    #[serde(default)]
    max_drift_m: Option<f64>,
    #[serde(default)]
    slider_id: Option<String>,
    #[serde(default)]
    r_m: Option<f64>,
    #[serde(default)]
    l_m: Option<f64>,
    #[serde(default)]
    omega_rad_s: Option<f64>,
    #[serde(default)]
    max_position_error_pct: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
struct BenchmarkStep {
    tool: String,
    params: Value,
}

#[derive(Debug, Clone)]
struct TaskRunMetrics {
    id: String,
    calls_succeeded: bool,
    is_watertight: bool,
    max_dim_err_pct: f64,
    domain_fact_vs_ref: String,
    passed: bool,
    failure_reason: String,
}

async fn rpc_call(state: &mcp_cad_server::SharedState, id: u64, tool: &str, args: Value) -> Value {
    let req = json!({
        "jsonrpc": "2.0",
        "id": id,
        "method": "tools/call",
        "params": {
            "name": tool,
            "arguments": args
        }
    });
    let req_str = req.to_string();
    let resp = handle_message(&req_str, state)
        .await
        .expect("Expected JSON-RPC response");
    serde_json::to_value(&resp).expect("Failed to serialize response")
}

fn map_ts_material_to_rust(ts_mat: Option<&str>) -> &'static str {
    match ts_mat.unwrap_or("steel_1045") {
        "steel_1045" | "steel_structural_aisi1020" => "steel_structural_aisi1020",
        "steel_4140" | "steel_high_strength_4140" => "steel_high_strength_4140",
        "alu_6061" | "aluminum_6061_t6" => "aluminum_6061_t6",
        "ti_6al_4v" | "titanium_grade5_ti6al4v" => "titanium_grade5_ti6al4v",
        "brass_c360" => "brass_c360",
        "nylon_pa12" => "nylon_pa12",
        "pla_3d" | "pla_filament_3dprint" => "pla_filament_3dprint",
        "petg_3d" | "petg_filament_3dprint" => "petg_filament_3dprint",
        _ => "steel_structural_aisi1020",
    }
}

fn extract_tool_response_json(resp: &Value) -> Result<Value, String> {
    if let Some(err) = resp.get("error") {
        if !err.is_null() {
            return Err(format!("JSON-RPC error: {err}"));
        }
    }
    let is_error = resp["result"]["isError"].as_bool().unwrap_or(false);
    let text = resp["result"]["content"]
        .as_array()
        .and_then(|arr| arr.first())
        .and_then(|c| c.get("text"))
        .and_then(|v| v.as_str())
        .unwrap_or("");
    if is_error {
        return Err(text.to_string());
    }
    serde_json::from_str::<Value>(text).map_err(|e| format!("Invalid JSON in tool output: {e} ({text})"))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ExecutionMode {
    /// Mode 1: Send steps from `bench/tasks.json` strictly as-is without any pre-calls or edits.
    StrictAsIs,
    /// Mode 2: Pre-create referenced parts via `create_part` (with Rust material ID mapping),
    /// then execute every step and parameter from `bench/tasks.json` 100% verbatim ("as-is").
    PreCreatePartsVerbatimSteps,
    /// Mode 3: Bridge schema/parameter-name differences (`cut_hole` depth, `revolve` 4-point contour,
    /// `polygon_prism` radius/chamfer, `create_joint` generated `joint_id`, fixed base part) while
    /// preserving all numerical dimensions, modules, loads, and tolerances from `bench/tasks.json`.
    SchemaBridgedDiagnostic,
}

async fn run_single_task(
    task: &BenchmarkTask,
    mode: ExecutionMode,
    renderer: std::sync::Arc<mcp_cad_viewer::offscreen::OffscreenRenderer>,
) -> TaskRunMetrics {
    let mut engine = EngineState::with_renderer(renderer);
    engine.length_unit = mcp_cad_server::state::LengthUnit::M;
    let state = engine.into_shared();

    let mut req_id = 1u64;
    let mut calls_succeeded = true;
    let mut failed_step_info: Option<String> = None;
    let mut last_fea_json: Option<Value> = None;
    let mut last_sim_json: Option<Value> = None;
    let mut joint_id_map: HashMap<String, String> = HashMap::new();
    let mut part_z_span: HashMap<String, f64> = HashMap::new();

    if mode != ExecutionMode::StrictAsIs {
        // Pre-create parts referenced in `add_feature_primitive` steps
        let mut created_parts = std::collections::HashSet::new();
        for step in &task.steps {
            if step.tool == "add_feature_primitive" {
                if let Some(part_id) = step.params.get("part_id").and_then(|v| v.as_str()) {
                    if created_parts.insert(part_id.to_string()) {
                        let mat = map_ts_material_to_rust(
                            step.params.get("material").and_then(|v| v.as_str()),
                        );
                        let is_fixed = mode == ExecutionMode::SchemaBridgedDiagnostic
                            && (part_id.contains("housing") || part_id.contains("base"));
                        let create_resp = rpc_call(
                            &state,
                            req_id,
                            "create_part",
                            json!({
                                "part_id": part_id,
                                "name": part_id,
                                "material": mat,
                                "is_fixed": is_fixed
                            }),
                        )
                        .await;
                        req_id += 1;
                        if let Err(e) = extract_tool_response_json(&create_resp) {
                            calls_succeeded = false;
                            failed_step_info = Some(format!("pre-create_part({part_id}): {e}"));
                            break;
                        }
                    }
                }
            }
        }
    }

    if calls_succeeded {
        for (idx, step) in task.steps.iter().enumerate() {
            let mut args = step.params.clone();

            if mode == ExecutionMode::SchemaBridgedDiagnostic {
                match step.tool.as_str() {
                    "add_feature_primitive" => {
                        let part_id = args
                            .get("part_id")
                            .and_then(|v| v.as_str())
                            .unwrap_or("")
                            .to_string();
                        let prim_type = args
                            .get("primitive_type")
                            .and_then(|v| v.as_str())
                            .unwrap_or("");
                        if prim_type == "box" {
                            if let Some(d) = args
                                .get("params")
                                .and_then(|p| p.get("depth"))
                                .and_then(|v| v.as_f64())
                            {
                                part_z_span.insert(part_id, d);
                            }
                        } else if prim_type == "revolve" {
                            // Bridge 2-point profile + inner_radius into 4-point closed contour
                            if let Some(p_obj) = args.get_mut("params").and_then(|v| v.as_object_mut()) {
                                if let (Some(prof), Some(inner_r)) = (
                                    p_obj.get("profile").and_then(|v| v.as_array()).cloned(),
                                    p_obj.get("inner_radius").and_then(|v| v.as_f64()),
                                ) {
                                    if prof.len() == 2 {
                                        let r_out = prof[0][0].as_f64().unwrap_or(0.035);
                                        let z_min = prof[0][1].as_f64().unwrap_or(-0.04);
                                        let z_max = prof[1][1].as_f64().unwrap_or(0.04);
                                        p_obj.insert(
                                            "profile".to_string(),
                                            json!([
                                                [inner_r, z_min],
                                                [r_out, z_min],
                                                [r_out, z_max],
                                                [inner_r, z_max]
                                            ]),
                                        );
                                    }
                                }
                            }
                        } else if prim_type == "polygon_prism" {
                            if let Some(p_obj) = args.get_mut("params").and_then(|v| v.as_object_mut()) {
                                if let Some(h) = p_obj.get("height").and_then(|v| v.as_f64()) {
                                    part_z_span.insert(part_id, h);
                                }
                                if let Some(or_val) = p_obj.get("outer_radius").cloned() {
                                    p_obj.entry("radius".to_string()).or_insert(or_val);
                                }
                                if let Some(cr_val) = p_obj.get("chamfer_radius").cloned() {
                                    p_obj.entry("chamfer".to_string()).or_insert(cr_val);
                                }
                            }
                        }
                    }
                    "cut_hole" => {
                        if let Some(obj) = args.as_object_mut() {
                            let part_id = obj
                                .get("part_id")
                                .and_then(|v| v.as_str())
                                .unwrap_or("")
                                .to_string();
                            let through = obj.get("through").and_then(|v| v.as_bool()).unwrap_or(false);
                            if through && !obj.contains_key("depth") {
                                let z_thickness = part_z_span.get(&part_id).copied().unwrap_or(0.04);
                                obj.insert("depth".to_string(), json!(z_thickness));
                                // Shift entry position to top face (+z_thickness/2) when normal is [0,0,-1]
                                if let (Some(pos), Some(norm)) = (
                                    obj.get("position").and_then(|v| v.as_array()).cloned(),
                                    obj.get("normal").and_then(|v| v.as_array()).cloned(),
                                ) {
                                    if pos.len() == 3
                                        && norm.len() == 3
                                        && pos[2].as_f64().unwrap_or(0.0).abs() < 1e-9
                                        && (norm[2].as_f64().unwrap_or(0.0) + 1.0).abs() < 1e-6
                                    {
                                        obj.insert(
                                            "position".to_string(),
                                            json!([pos[0], pos[1], z_thickness * 0.5]),
                                        );
                                    }
                                }
                            }
                        }
                    }
                    "cut_helical_groove" => {
                        if let Some(obj) = args.as_object_mut() {
                            let part_id = obj
                                .get("part_id")
                                .and_then(|v| v.as_str())
                                .unwrap_or("")
                                .to_string();
                            if !obj.contains_key("start_z") {
                                if let Some(&h) = part_z_span.get(&part_id) {
                                    obj.insert("start_z".to_string(), json!(-h * 0.5));
                                }
                            }
                            if !obj.contains_key("starts") {
                                obj.insert("starts".to_string(), json!(1));
                            }
                        }
                    }
                    "set_joint_motor" => {
                        if let Some(obj) = args.as_object_mut() {
                            if let Some(orig_id) = obj.get("joint_id").and_then(|v| v.as_str()) {
                                if let Some(mapped_id) = joint_id_map.get(orig_id) {
                                    obj.insert("joint_id".to_string(), json!(mapped_id));
                                }
                            }
                        }
                    }
                    "run_physics_simulation" => {
                        if let Some(obj) = args.as_object_mut() {
                            if !obj.contains_key("step_size_ms") {
                                if let Some(dt) = obj.get("dt").and_then(|v| v.as_f64()) {
                                    obj.insert("step_size_ms".to_string(), json!(dt * 1000.0));
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }

            let resp = rpc_call(&state, req_id, &step.tool, args).await;
            req_id += 1;

            match extract_tool_response_json(&resp) {
                Ok(val) => {
                    if step.tool == "create_joint" {
                        if let (Some(req_jid), Some(actual_jid)) = (
                            step.params.get("joint_id").and_then(|v| v.as_str()),
                            val.get("joint_id").and_then(|v| v.as_str()),
                        ) {
                            joint_id_map.insert(req_jid.to_string(), actual_jid.to_string());
                        }
                    } else if step.tool == "run_fea_analysis" {
                        last_fea_json = Some(val);
                    } else if step.tool == "run_physics_simulation" {
                        last_sim_json = Some(val);
                    }
                }
                Err(err_msg) => {
                    calls_succeeded = false;
                    failed_step_info = Some(format!("step #{} `{}`: {}", idx + 1, step.tool, err_msg));
                    break;
                }
            }
        }
    }

    // Inspect target part geometry & mesh diagnostics directly from EngineState
    let mut is_watertight = false;
    let mut actual_genus: Option<i32> = None;
    let mut genus_matched = false;
    let mut actual_dims_m: Option<[f64; 3]> = None;
    let mut max_dim_err_pct = 100.0;
    let mut dims_within_1pct = false;

    {
        let guard = state.read();
        if let Some(part) = guard.parts.get(&task.target_part_id) {
            if !part.mesh.vertices.is_empty() {
                let report = mcp_cad_core::validate_mesh(&part.mesh, Some(task.expected_genus));
                is_watertight = report.topology.is_watertight;
                actual_genus = Some(report.topology.genus.round() as i32);
                genus_matched = (report.topology.genus - task.expected_genus as f64).abs() < 1e-6;

                let aabb = part.bounding_box();
                let dims = aabb.dimensions();
                actual_dims_m = Some(dims);
                max_dim_err_pct = dims
                    .iter()
                    .zip(task.expected_dimensions_m.iter())
                    .map(|(&act, &exp)| ((act - exp).abs() / exp.max(1e-9)) * 100.0)
                    .fold(0.0f64, f64::max);
                dims_within_1pct = max_dim_err_pct <= 1.0;
            }
        }
    }

    // Evaluate domain criteria (geometry/topology or FEA or Kinematics)
    let mut domain_matched = calls_succeeded && is_watertight && genus_matched;
    let mut domain_fact_vs_ref = if let Some(dims) = actual_dims_m {
        format!(
            "dims=[{:.4},{:.4},{:.4}] vs [{:.4},{:.4},{:.4}] m, g={:?} (exp {})",
            dims[0],
            dims[1],
            dims[2],
            task.expected_dimensions_m[0],
            task.expected_dimensions_m[1],
            task.expected_dimensions_m[2],
            actual_genus,
            task.expected_genus
        )
    } else {
        "no target part mesh".to_string()
    };

    if let Some(exp_fea) = &task.expected_fea {
        if let Some(fea) = &last_fea_json {
            let status_str = fea["status"].as_str().unwrap_or("UNKNOWN");
            // Note: Rust returns "PASS" / "FAIL (YIELD EXCEEDED)" in `status`, while tasks.json expects `verdict: "SAFE"`
            let verdict_strict_match = status_str == exp_fea.verdict;
            let verdict_semantic_match =
                (exp_fea.verdict == "SAFE" && status_str == "PASS") || verdict_strict_match;

            let act_stress = fea["max_von_mises_mpa"].as_f64().unwrap_or(f64::NAN);
            let act_disp = fea["max_displacement_mm"].as_f64().unwrap_or(f64::NAN);
            let stress_err_pct =
                ((act_stress - exp_fea.stress_mpa).abs() / exp_fea.stress_mpa) * 100.0;
            let disp_err_pct = exp_fea
                .displacement_mm
                .map(|exp_d| ((act_disp - exp_d).abs() / exp_d) * 100.0);
            let tol = exp_fea.tolerance_pct.unwrap_or(5.0);

            let disp_ok = disp_err_pct.is_none_or(|e| e <= tol);
            domain_matched = verdict_semantic_match && stress_err_pct <= tol && disp_ok;

            domain_fact_vs_ref = if let (Some(exp_d), Some(d_err)) =
                (exp_fea.displacement_mm, disp_err_pct)
            {
                format!(
                    "status={} (exp {}), σ={:.2} vs {:.2} MPa (err {:.2}%), δ={:.4} vs {:.4} mm (err {:.2}%), tol={:.1}%",
                    status_str, exp_fea.verdict, act_stress, exp_fea.stress_mpa, stress_err_pct, act_disp, exp_d, d_err, tol
                )
            } else {
                format!(
                    "status={} (exp {}), σ={:.2} vs {:.2} MPa (err {:.2}%), δ={:.4} mm, tol={:.1}%",
                    status_str, exp_fea.verdict, act_stress, exp_fea.stress_mpa, stress_err_pct, act_disp, tol
                )
            };
        } else {
            domain_matched = false;
            domain_fact_vs_ref = format!(
                "missing FEA result (exp σ={:.2} MPa, verdict={})",
                exp_fea.stress_mpa, exp_fea.verdict
            );
        }
    } else if let Some(exp_kin) = &task.expected_kinematics {
        if let Some(sim) = &last_sim_json {
            let traj = &sim["trajectory_summary"];
            if exp_kin.kin_type == "gear_pair" {
                let driver_id = exp_kin.driver_id.as_deref().unwrap_or("t14_gear_z20");
                let driven_id = exp_kin.driven_id.as_deref().unwrap_or("t14_gear_z40");
                let exp_ratio = exp_kin.expected_velocity_ratio.unwrap_or(-0.5);
                let max_drift_limit = exp_kin.max_drift_m.unwrap_or(0.001);

                let s1 = traj[driver_id]["samples"].as_array();
                let s2 = traj[driven_id]["samples"].as_array();
                if let (Some(arr1), Some(arr2)) = (s1, s2) {
                    let w1 = arr1
                        .last()
                        .and_then(|s| s["angular_velocity"][2].as_f64())
                        .unwrap_or(0.0);
                    let w2 = arr2
                        .last()
                        .and_then(|s| s["angular_velocity"][2].as_f64())
                        .unwrap_or(0.0);
                    let ratio = if w1.abs() > 1e-6 { w2 / w1 } else { 0.0 };

                    let p0 = arr2
                        .first()
                        .and_then(|s| s["position"].as_array())
                        .cloned()
                        .unwrap_or_default();
                    let p_last = arr2
                        .last()
                        .and_then(|s| s["position"].as_array())
                        .cloned()
                        .unwrap_or_default();
                    let drift = if p0.len() == 3 && p_last.len() == 3 {
                        let dx = p_last[0].as_f64().unwrap_or(0.0) - p0[0].as_f64().unwrap_or(0.0);
                        let dy = p_last[1].as_f64().unwrap_or(0.0) - p0[1].as_f64().unwrap_or(0.0);
                        let dz = p_last[2].as_f64().unwrap_or(0.0) - p0[2].as_f64().unwrap_or(0.0);
                        (dx * dx + dy * dy + dz * dz).sqrt()
                    } else {
                        0.0
                    };

                    domain_matched =
                        (ratio - exp_ratio).abs() < 1e-3 && drift <= max_drift_limit;
                    domain_fact_vs_ref = format!(
                        "ω1={:.3}, ω2={:.3} rad/s, ω2/ω1={:.4} vs {:.4}, drift={:.4} m (max {:.4} m)",
                        w1, w2, ratio, exp_ratio, drift, max_drift_limit
                    );
                } else {
                    domain_matched = false;
                    domain_fact_vs_ref = "missing gear trajectory samples".to_string();
                }
            } else if exp_kin.kin_type == "slider_crank" {
                let slider_id = exp_kin.slider_id.as_deref().unwrap_or("t15_piston_slider");
                let r = exp_kin.r_m.unwrap_or(0.05);
                let l = exp_kin.l_m.unwrap_or(0.2);
                let omega = exp_kin.omega_rad_s.unwrap_or(std::f64::consts::TAU);
                let tol_pct = exp_kin.max_position_error_pct.unwrap_or(2.0);

                if let Some(samples) = traj[slider_id]["samples"].as_array() {
                    let mut max_err_pct = 0.0f64;
                    let mut sample_last_x = 0.0f64;
                    let mut sample_last_exp_x = 0.0f64;
                    for s in samples {
                        let t = s["time_s"].as_f64().unwrap_or(0.0);
                        let theta = omega * t;
                        let exp_x = r * theta.cos() + (l * l - r * r * theta.sin().powi(2)).sqrt();
                        let act_x = s["position"][0].as_f64().unwrap_or(0.0);
                        let err_pct = ((act_x - exp_x).abs() / exp_x.abs().max(1e-9)) * 100.0;
                        if err_pct > max_err_pct {
                            max_err_pct = err_pct;
                        }
                        sample_last_x = act_x;
                        sample_last_exp_x = exp_x;
                    }
                    domain_matched = samples.len() > 10 && max_err_pct <= tol_pct;
                    domain_fact_vs_ref = format!(
                        "slider x_end={:.4} vs {:.4} m, max_x_err={:.2}% (tol {:.1}%)",
                        sample_last_x, sample_last_exp_x, max_err_pct, tol_pct
                    );
                } else {
                    domain_matched = false;
                    domain_fact_vs_ref = "missing slider trajectory samples".to_string();
                }
            }
        } else {
            domain_matched = false;
            domain_fact_vs_ref = format!("missing simulation result ({})", exp_kin.kin_type);
        }
    }

    let passed =
        calls_succeeded && is_watertight && genus_matched && dims_within_1pct && domain_matched;

    let mut reasons = Vec::new();
    if let Some(step_err) = &failed_step_info {
        reasons.push(step_err.clone());
    }
    if calls_succeeded && !is_watertight {
        reasons.push("mesh not watertight".to_string());
    }
    if calls_succeeded && !genus_matched {
        reasons.push(format!(
            "genus {:?} != expected {}",
            actual_genus, task.expected_genus
        ));
    }
    if calls_succeeded && !dims_within_1pct {
        reasons.push(format!("dim error {:.2}% > 1%", max_dim_err_pct));
    }
    if calls_succeeded && !domain_matched {
        reasons.push(format!("domain mismatch ({domain_fact_vs_ref})"));
    }
    let failure_reason = if passed {
        "OK".to_string()
    } else {
        reasons.join("; ")
    };

    TaskRunMetrics {
        id: task.id.clone(),
        calls_succeeded,
        is_watertight,
        max_dim_err_pct,
        domain_fact_vs_ref,
        passed,
        failure_reason,
    }
}

fn print_results_table(title: &str, results: &[TaskRunMetrics]) {
    let passed_count = results.iter().filter(|r| r.passed).count();
    println!("\n========================================================================================================================");
    println!("{title}");
    println!(
        "Summary: {}/{} PASSED ({:.1}%)",
        passed_count,
        results.len(),
        (passed_count as f64 / results.len() as f64) * 100.0
    );
    println!("========================================================================================================================");
    println!(
        "{:<32} | {:<6} | {:<5} | {:<10} | {:<12} | Fact vs Reference / Failure Reason",
        "Task ID", "Status", "Calls", "Watertight", "MaxDimErr(%)"
    );
    println!("------------------------------------------------------------------------------------------------------------------------");
    for r in results {
        println!(
            "{:<32} | {:<6} | {:<5} | {:<10} | {:>11.4}% | {} [{}]",
            r.id,
            if r.passed { "PASS" } else { "FAIL" },
            if r.calls_succeeded { "OK" } else { "ERR" },
            r.is_watertight,
            r.max_dim_err_pct,
            r.domain_fact_vs_ref,
            r.failure_reason
        );
    }
    println!("========================================================================================================================\n");
}

#[tokio::test(flavor = "multi_thread")]
async fn test_rust_benchmark_15_tasks_baseline() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let tasks_path = manifest_dir.join("../../bench/tasks.json");
    let raw = std::fs::read_to_string(&tasks_path)
        .unwrap_or_else(|e| panic!("Failed to read {}: {e}", tasks_path.display()));
    let spec: BenchmarkSpec =
        serde_json::from_str(&raw).expect("Failed to deserialize bench/tasks.json");

    assert_eq!(spec.tasks.len(), 15, "bench/tasks.json must contain 15 tasks");

    let renderer = std::sync::Arc::new(
        mcp_cad_viewer::offscreen::OffscreenRenderer::new()
            .expect("Failed to initialize OffscreenRenderer for benchmark suite"),
    );

    let mut strict_handles = Vec::with_capacity(spec.tasks.len());
    let mut precreate_handles = Vec::with_capacity(spec.tasks.len());
    let mut bridged_handles = Vec::with_capacity(spec.tasks.len());

    for task in &spec.tasks {
        let t1 = task.clone();
        let r1 = renderer.clone();
        strict_handles.push(tokio::spawn(async move {
            run_single_task(&t1, ExecutionMode::StrictAsIs, r1).await
        }));

        let t2 = task.clone();
        let r2 = renderer.clone();
        precreate_handles.push(tokio::spawn(async move {
            run_single_task(&t2, ExecutionMode::PreCreatePartsVerbatimSteps, r2).await
        }));

        let t3 = task.clone();
        let r3 = renderer.clone();
        bridged_handles.push(tokio::spawn(async move {
            run_single_task(&t3, ExecutionMode::SchemaBridgedDiagnostic, r3).await
        }));
    }

    let mut strict_results = Vec::with_capacity(spec.tasks.len());
    for h in strict_handles {
        strict_results.push(h.await.expect("strict task panicked"));
    }

    let mut precreate_results = Vec::with_capacity(spec.tasks.len());
    for h in precreate_handles {
        precreate_results.push(h.await.expect("precreate task panicked"));
    }

    let mut bridged_results = Vec::with_capacity(spec.tasks.len());
    for h in bridged_handles {
        bridged_results.push(h.await.expect("bridged task panicked"));
    }

    print_results_table(
        "MODE 1: STRICT AS-IS (`bench/tasks.json` steps sent verbatim to Rust `mcp-cad-server`)",
        &strict_results,
    );
    print_results_table(
        "MODE 2: WITH `create_part` PRE-INITIALIZATION (`task.steps` and parameters verbatim from `bench/tasks.json`)",
        &precreate_results,
    );
    print_results_table(
        "MODE 3: SCHEMA-BRIDGED DIAGNOSTIC PROBE (schema differences adapted, all numbers/units from `bench/tasks.json` kept as-is)",
        &bridged_results,
    );
}
