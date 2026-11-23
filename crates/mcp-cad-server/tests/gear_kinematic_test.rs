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
                    "params": { "width": 200.0, "height": 100.0, "depth": 20.0 }
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
                        "module": 2.0,
                        "teeth": 20,
                        "pressure_angle_deg": 20.0,
                        "face_width": 10.0
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
                        "module": 2.0,
                        "teeth": 40,
                        "pressure_angle_deg": 20.0,
                        "face_width": 10.0
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
                    "translation": [60.0, 0.0, 0.0],
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
                    "anchor_a": [0.0, 0.0, 20.0],
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
                    "anchor_a": [60.0, 0.0, 20.0],
                    "anchor_b": [60.0, 0.0, 0.0],
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

#[tokio::test]
async fn test_gear_kinematic_10_revolutions_and_load_cases() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // Apply gear_transmission recipe with motor velocity 20.0 rad/s (10 revolutions in pi seconds)
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 100,
            "method": "tools/call",
            "params": {
                "name": "apply_recipe",
                "arguments": {
                    "recipe": "gear_transmission",
                    "params": {
                        "id_prefix": "g10",
                        "module": 2.0,
                        "teeth_pinion": 20,
                        "teeth_gear": 40,
                        "motor_velocity_rad_s": 20.0
                    }
                }
            }
        }),
    )
    .await;
    let rec_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let rec: Value = serde_json::from_str(rec_text).unwrap();
    assert_eq!(rec["status"], "recipe_applied");

    // Simulate for pi seconds (3.141592653589793 s) at 10 ms step size => 10 revolutions of pinion
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 101,
            "method": "tools/call",
            "params": {
                "name": "run_physics_simulation",
                "arguments": {
                    "duration_seconds": std::f64::consts::PI,
                    "step_size_ms": 10.0
                }
            }
        }),
    )
    .await;
    let sim_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let sim: Value = serde_json::from_str(sim_text).unwrap();
    assert_eq!(sim["success"], true);

    let traj = &sim["trajectory_summary"];
    let s_pinion = traj["g10_pinion"]["samples"].as_array().unwrap();
    let s_gear = traj["g10_gear"]["samples"].as_array().unwrap();
    assert!(s_pinion.len() >= 314);

    for i in 1..s_pinion.len() {
        let w1 = s_pinion[i]["angular_velocity"][2].as_f64().unwrap();
        let w2 = s_gear[i]["angular_velocity"][2].as_f64().unwrap();
        assert!(
            (w1 - 20.0).abs() < 1e-9,
            "Pinion w_z at sample {i} must be 20.0 rad/s, got {w1}"
        );
        assert!(
            (w2 - (-10.0)).abs() < 1e-9,
            "Gear w_z at sample {i} must be -10.0 rad/s, got {w2}"
        );
    }

    // Verify load case extraction from the 10-revolution simulation
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 102,
            "method": "tools/call",
            "params": {
                "name": "get_part_load_cases",
                "arguments": {
                    "part_id": "g10_pinion",
                    "select": "peak_force"
                }
            }
        }),
    )
    .await;
    let lc_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let lc: Value = serde_json::from_str(lc_text).unwrap();
    assert_eq!(lc["part_id"], "g10_pinion");
    assert!(
        lc["equilibrium_residual_force_norm"].as_f64().unwrap() < 1e-7,
        "10-rev pinion force equilibrium residual must be < 1e-7 N"
    );
    assert!(
        lc["equilibrium_residual_torque_norm"].as_f64().unwrap() < 1e-7,
        "10-rev pinion torque equilibrium residual must be < 1e-7 N*m"
    );
}

#[tokio::test]
async fn test_coupled_fem_gear_clash_stays_on_shafts_and_deforms_teeth() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // Switch EngineState to meters (matching Web Studio rust_scene_sync)
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 200,
            "method": "tools/call",
            "params": {
                "name": "set_units",
                "arguments": { "length_unit": "m" }
            }
        }),
    )
    .await;

    // 1. Left gear: Z=18, m=2.2mm (steel_high_strength_4140) at x = -0.022275m
    let x_left = -0.022275_f64;
    let x_right = 0.022275_f64;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 201,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "gear_clash_left",
                    "name": "Left Gear Z18",
                    "material": "steel_high_strength_4140",
                    "is_fixed": false
                }
            }
        }),
    )
    .await;
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 202,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "gear_clash_left",
                    "primitive_type": "spur_gear",
                    "length_unit": "m",
                    "params": {
                        "module": 0.0022,
                        "teeth": 18,
                        "face_width": 0.016,
                        "pressure_angle_deg": 20.0,
                        "bore_diameter": 0.012,
                        "hub_diameter": 0.024,
                        "hub_length": 0.006,
                        "keyway_width": 0.004,
                        "keyway_depth": 0.0018
                    }
                }
            }
        }),
    )
    .await;
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 203,
            "method": "tools/call",
            "params": {
                "name": "transform_part",
                "arguments": {
                    "part_id": "gear_clash_left",
                    "length_unit": "m",
                    "translation": [x_left, 0.0, 0.0]
                }
            }
        }),
    )
    .await;

    // 2. Right gear: Z=22, m=2.2mm (brass_c360) at x = +0.022275m, rot_z = 180/22 deg
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 204,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "gear_clash_right",
                    "name": "Right Gear Z22",
                    "material": "brass_c360",
                    "is_fixed": false
                }
            }
        }),
    )
    .await;
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 205,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "gear_clash_right",
                    "primitive_type": "spur_gear",
                    "length_unit": "m",
                    "params": {
                        "module": 0.0022,
                        "teeth": 22,
                        "face_width": 0.016,
                        "pressure_angle_deg": 20.0,
                        "bore_diameter": 0.014,
                        "hub_diameter": 0.028,
                        "hub_length": 0.006,
                        "keyway_width": 0.004,
                        "keyway_depth": 0.0018
                    }
                }
            }
        }),
    )
    .await;
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 206,
            "method": "tools/call",
            "params": {
                "name": "transform_part",
                "arguments": {
                    "part_id": "gear_clash_right",
                    "length_unit": "m",
                    "translation": [x_right, 0.0, 0.0],
                    "rotation_deg": [0.0, 0.0, 180.0 / 22.0]
                }
            }
        }),
    )
    .await;

    // 3. Revolute shaft bearings with counter-clashing +120 rpm motors (185 N*m) + gear metadata
    let omega = 120.0 * 2.0 * std::f64::consts::PI / 60.0;
    for (idx, (jid, pid, x_pos)) in [
        ("joint_drive_left", "gear_clash_left", x_left),
        ("joint_drive_right", "gear_clash_right", x_right),
    ]
    .iter()
    .enumerate()
    {
        rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 210 + idx * 2,
                "method": "tools/call",
                "params": {
                    "name": "create_joint",
                    "arguments": {
                        "joint_id": jid,
                        "joint_type": "revolute",
                        "part_a": "ground",
                        "part_b": pid,
                        "length_unit": "m",
                        "anchor_a": [x_pos, 0.0, 0.0],
                        "anchor_b": [x_pos, 0.0, 0.0],
                        "axis": [0.0, 0.0, 1.0]
                    }
                }
            }),
        )
        .await;
        rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 211 + idx * 2,
                "method": "tools/call",
                "params": {
                    "name": "set_joint_motor",
                    "arguments": {
                        "joint_id": jid,
                        "target_velocity": omega,
                        "max_force": 185.0
                    }
                }
            }),
        )
        .await;
    }

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 220,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "auto_gear_clash",
                    "joint_type": "gear",
                    "part_a": "gear_clash_left",
                    "part_b": "gear_clash_right",
                    "axis_a": [0.0, 0.0, 1.0],
                    "axis_b": [0.0, 0.0, 1.0],
                    "teeth_a": 18,
                    "teeth_b": 22
                }
            }
        }),
    )
    .await;

    // 4. Bake coupled HHT-alpha dynamic FEM simulation (synchronous for test)
    let bake_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 221,
            "method": "tools/call",
            "params": {
                "name": "bake_coupled_simulation",
                "arguments": {
                    "part_ids": ["gear_clash_left", "gear_clash_right"],
                    "accuracy_level": "standard",
                    "duration_s": 0.05,
                    "output_fps": 600,
                    "max_frames": 60,
                    "gravity": [0.0, -9.81, 0.0],
                    "enable_plasticity": true,
                    "enable_fracture": true,
                    "enable_contact": true,
                    "use_physics_joints": true,
                    "contact_friction": 0.2,
                    "async_mode": false
                }
            }
        }),
    )
    .await;
    assert_ne!(
        bake_resp["result"]["isError"],
        json!(true),
        "bake_coupled_simulation failed: {bake_resp}"
    );
    let bake_text = bake_resp["result"]["content"][0]["text"].as_str().unwrap();
    let bake_json: Value = serde_json::from_str(bake_text).unwrap();
    let sim_id = bake_json["simulation_id"].as_str().unwrap().to_string();
    let frame_count = bake_json["frame_count"].as_u64().unwrap() as usize;
    let elapsed_ms = bake_json["elapsed_ms"].as_f64().unwrap_or(0.0);

    // Verify shaft hub pivots are at true gear centers (NOT [0, 0, 0]!)
    let applied_joints = bake_json["applied_joint_couplings"].as_array().unwrap();
    assert_eq!(applied_joints.len(), 2);
    for jc in applied_joints {
        let pid = jc["part_id"].as_str().unwrap();
        let px = jc["pivot_world"][0].as_f64().unwrap();
        let expected_x = if pid == "gear_clash_left" {
            x_left
        } else {
            x_right
        };
        assert!(
            (px - expected_x).abs() < 1e-4,
            "Hub pivot X for {pid} must be {expected_x}, got {px}"
        );
    }

    // Query Frame 0 and final frame
    let f0_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 222,
            "method": "tools/call",
            "params": {
                "name": "query_simulation_frame",
                "arguments": {
                    "simulation_id": sim_id,
                    "frame_index": 0,
                    "include_mesh_fields": true
                }
            }
        }),
    )
    .await;
    let f0: Value = serde_json::from_str(f0_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let ee0 = f0["energy_balance"]["elastic_j"].as_f64().unwrap();

    let fl_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 223,
            "method": "tools/call",
            "params": {
                "name": "query_simulation_frame",
                "arguments": {
                    "simulation_id": sim_id,
                    "frame_index": frame_count - 1,
                    "include_mesh_fields": true
                }
            }
        }),
    )
    .await;
    let fl: Value = serde_json::from_str(fl_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();

    let mut peak_vm_all = 0.0_f64;
    let mut peak_ep_all = 0.0_f64;
    let mut peak_dmg_all = 0.0_f64;
    let mut total_eroded = 0u64;
    for part in fl["parts"].as_array().unwrap() {
        let pid = part["part_id"].as_str().unwrap();
        let vm = part["max_von_mises_mpa"].as_f64().unwrap_or(0.0);
        let ep = part["max_equiv_plastic_strain"].as_f64().unwrap_or(0.0);
        let dm = part["max_damage"].as_f64().unwrap_or(0.0);
        let eroded = part["eroded_element_count"].as_u64().unwrap_or(0);
        peak_vm_all = peak_vm_all.max(vm);
        peak_ep_all = peak_ep_all.max(ep);
        peak_dmg_all = peak_dmg_all.max(dm);
        total_eroded += eroded;

        // Verify gear centroid stays anchored at its own shaft center (NOT orbiting (0,0,0)!)
        let positions = part["surface_positions_m"].as_array().unwrap();
        let mut cx = 0.0_f64;
        let mut cy = 0.0_f64;
        for pt in positions {
            cx += pt[0].as_f64().unwrap();
            cy += pt[1].as_f64().unwrap();
        }
        cx /= positions.len().max(1) as f64;
        cy /= positions.len().max(1) as f64;
        let expected_x = if pid == "gear_clash_left" {
            x_left
        } else {
            x_right
        };
        let drift_m = ((cx - expected_x).powi(2) + cy.powi(2)).sqrt();
        eprintln!(
            "[gear_clash] {pid}: nodes={}, vm={vm:.1} MPa, ep={:.3}%, D={dm:.3}, eroded={eroded}, centroid=({:.4}, {:.4}), drift={:.2}mm",
            part["node_count"],
            ep * 100.0,
            cx,
            cy,
            drift_m * 1000.0
        );
        assert!(
            drift_m < 0.004,
            "Gear {pid} drifted {:.2} mm away from its shaft center ({expected_x}, 0)!",
            drift_m * 1000.0
        );
    }
    eprintln!(
        "[gear_clash] frames={frame_count}, elapsed={elapsed_ms:.0}ms, E_e(0)={ee0:.6} J, peak_vm={peak_vm_all:.1} MPa, peak_ep={:.3}%, peak_D={peak_dmg_all:.3}, eroded={total_eroded}",
        peak_ep_all * 100.0
    );

    // Frame 0 must have near-zero elastic energy (no bogus 983 J initial interpenetration!)
    assert!(
        ee0 < 0.05,
        "Frame 0 elastic energy must be near zero (< 0.05 J), got {ee0:.4} J"
    );
    // Counter-clashing teeth must produce significant real contact stress (> 80 MPa)
    assert!(
        peak_vm_all > 80.0,
        "Counter-clashing gears must develop substantial tooth contact stress, got {peak_vm_all:.2} MPa"
    );
}


