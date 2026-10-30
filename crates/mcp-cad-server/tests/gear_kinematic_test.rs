//! Test verifying kinematic gear-to-gear transmission and animated preview generation.

use serde_json::{json, Value};
use mcp_cad_server::protocol::handle_message;
use mcp_cad_server::state::EngineState;

async fn rpc_call(state: &mcp_cad_server::SharedState, req: Value) -> Value {
    let req_str = req.to_string();
    let resp = handle_message(&req_str, state)
        .await
        .expect("Expected a JSON-RPC response");
    serde_json::to_value(&resp).expect("Failed to serialize response")
}

#[tokio::test]
async fn test_gear_kinematic_simulation_and_preview() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create a fixed mounting base (is_fixed: true)
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "gearbox_housing",
                    "material": "steel_structural_aisi1020",
                    "is_fixed": true
                }
            }
        }),
    ).await;
    let base_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let base_val: Value = serde_json::from_str(base_text).unwrap();
    let base_id = base_val["part_id"].as_str().unwrap().to_string();
    assert_eq!(base_val["is_fixed"], true);

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": base_id,
                    "primitive_type": "box",
                    "params": { "width": 0.20, "height": 0.10, "depth": 0.02 }
                }
            }
        }),
    ).await;

    // 2. Create Drive Pinion: m = 2mm, z = 20 -> pitch radius = 20mm = 0.02m
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "pinion_z20",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    ).await;
    let gear1_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let gear1_val: Value = serde_json::from_str(gear1_text).unwrap();
    let gear1_id = gear1_val["part_id"].as_str().unwrap().to_string();

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 4,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": gear1_id,
                    "primitive_type": "spur_gear",
                    "params": {
                        "module": 0.002,
                        "teeth": 20,
                        "pressure_angle_deg": 20.0,
                        "face_width": 0.010
                    }
                }
            }
        }),
    ).await;

    // 3. Create Driven Gear: m = 2mm, z = 40 -> pitch radius = 40mm = 0.04m
    // Center distance = 20mm + 40mm = 60mm = 0.06m
    // Theoretical ratio: -20/40 = -0.5000
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 5,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "gear_z40",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    ).await;
    let gear2_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let gear2_val: Value = serde_json::from_str(gear2_text).unwrap();
    let gear2_id = gear2_val["part_id"].as_str().unwrap().to_string();

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 6,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": gear2_id,
                    "primitive_type": "spur_gear",
                    "params": {
                        "module": 0.002,
                        "teeth": 40,
                        "pressure_angle_deg": 20.0,
                        "face_width": 0.010
                    }
                }
            }
        }),
    ).await;

    // Translate driven gear by 60mm along X, rotate by 4.5 deg (half tooth pitch)
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 7,
            "method": "tools/call",
            "params": {
                "name": "transform_part",
                "arguments": {
                    "part_id": gear2_id,
                    "translation": [0.06, 0.0, 0.0],
                    "rotation_deg": [0.0, 0.0, 4.5]
                }
            }
        }),
    ).await;

    // 4. Mount Gear 1 on fixed base with Revolute joint
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 8,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_type": "revolute",
                    "part_a": base_id,
                    "part_b": gear1_id,
                    "anchor_a": [0.0, 0.0, 0.02],
                    "anchor_b": [0.0, 0.0, 0.0],
                    "axis": [0.0, 0.0, 1.0]
                }
            }
        }),
    ).await;
    let j1_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let j1_val: Value = serde_json::from_str(j1_text).unwrap();
    let joint1_id = j1_val["joint_id"].as_str().unwrap().to_string();

    // Mount Gear 2 on fixed base with Revolute joint
    let _resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 9,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_type": "revolute",
                    "part_a": base_id,
                    "part_b": gear2_id,
                    "anchor_a": [0.06, 0.0, 0.02],
                    "anchor_b": [0.06, 0.0, 0.0],
                    "axis": [0.0, 0.0, 1.0]
                }
            }
        }),
    ).await;

    // 5. Connect Gear 1 and Gear 2 with Gear Constraint (ratio = 20 / 40 = 0.5)
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 10,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_type": "gear",
                    "part_a": gear1_id,
                    "part_b": gear2_id,
                    "teeth_a": 20,
                    "teeth_b": 40,
                    "axis": [0.0, 0.0, 1.0]
                }
            }
        }),
    ).await;
    let j_gear_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let j_gear_val: Value = serde_json::from_str(j_gear_text).unwrap();
    assert!(j_gear_val["joint_id"].as_str().unwrap().starts_with("joint-"));

    // 6. Set motor on Pinion to target velocity 6.0 rad/s
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 11,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": joint1_id,
                    "target_velocity": 6.0,
                    "max_force": 100.0
                }
            }
        }),
    ).await;
    let m_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let m_val: Value = serde_json::from_str(m_text).unwrap();
    assert_eq!(m_val["status"], "motor_active");

    // 7. Run simulation for 0.5 seconds
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 12,
            "method": "tools/call",
            "params": {
                "name": "run_physics_simulation",
                "arguments": {
                    "duration_seconds": 0.5,
                    "step_size_ms": 10.0
                }
            }
        }),
    ).await;

    let sim_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let sim: Value = serde_json::from_str(sim_text).unwrap();
    assert_eq!(sim["success"], true);

    let traj = &sim["trajectory_summary"];
    let s1 = traj[&gear1_id]["samples"].as_array().unwrap();
    let s2 = traj[&gear2_id]["samples"].as_array().unwrap();
    let s_base = traj[&base_id]["samples"].as_array().unwrap();

    // Base position must remain strictly anchored at [0, 0, 0] (no falling under gravity)
    let final_base_pos = s_base.last().unwrap()["position"].as_array().unwrap();
    assert!(final_base_pos[0].as_f64().unwrap().abs() < 1e-5);
    assert!(final_base_pos[1].as_f64().unwrap().abs() < 1e-5, "Fixed base must not fall: got y={}", final_base_pos[1]);
    assert!(final_base_pos[2].as_f64().unwrap().abs() < 1e-5);

    println!("\n=== Verification of Kinematic Gear Constraint (Ratio = 0.5) ===");
    println!("Step | Time (s) | Gear 1 w_z | Gear 2 w_z | Actual Ratio (w2/w1)");
    for i in (0..s1.len()).step_by(10) {
        let t = s1[i]["time_s"].as_f64().unwrap();
        let w1 = s1[i]["angular_velocity"][2].as_f64().unwrap();
        let w2 = s2[i]["angular_velocity"][2].as_f64().unwrap();
        let ratio = if w1.abs() > 0.01 { w2 / w1 } else { 0.0 };
        println!("{:4} | {:8.2} | {:10.4} | {:10.4} | {:12.4}", i, t, w1, w2, ratio);

        if i > 0 {
            // Must match exact theoretical ratio: w2 = -0.5 * w1
            assert!((ratio - (-0.5)).abs() < 1e-4, "Gear ratio deviation at step {i}: got {ratio}, expected -0.5");
        }
    }

    let final_w1 = s1.last().unwrap()["angular_velocity"][2].as_f64().unwrap();
    let final_w2 = s2.last().unwrap()["angular_velocity"][2].as_f64().unwrap();
    println!("\nFinal Gear 1 w_z: {:.4} rad/s", final_w1);
    println!("Final Gear 2 w_z: {:.4} rad/s", final_w2);
    assert!((final_w1 - 6.0).abs() < 0.2, "Gear 1 should reach target velocity 6.0 rad/s, got {final_w1}");
    assert!((final_w2 - (-3.0)).abs() < 0.1, "Gear 2 should reach target velocity -3.0 rad/s, got {final_w2}");

    // 8. Verify Animation Frames for 3D Preview Window
    let read_state = state.read();
    let frames = mcp_cad_server::tools::physics::generate_animation_frames(
        &read_state.parts,
        &read_state.last_simulation.as_ref().unwrap().trajectories,
        60,
    );
    assert!(!frames.is_empty(), "Animation frames must be generated for the preview window");
    println!("Generated {} animation frames for 3D Viewport playback", frames.len());
    assert!(frames.len() >= 30);
    // Verify that geometry is changing between frames (motion detected)
    let max_disp = frames
        .last()
        .unwrap()
        .vertices
        .iter()
        .zip(&frames[0].vertices)
        .map(|(v1, v0)| {
            ((v1.position[0] - v0.position[0]).powi(2)
                + (v1.position[1] - v0.position[1]).powi(2)
                + (v1.position[2] - v0.position[2]).powi(2))
            .sqrt()
        })
        .fold(0.0f64, f64::max);
    println!("Max vertex displacement across assembly: {:.4} m", max_disp);
    assert!(max_disp > 0.005, "Moving parts must displace/rotate across animation frames");
}
