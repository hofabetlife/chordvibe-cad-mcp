//! End-to-End and contract verification tests for the MCP-CAD server.

use base64::Engine;
use serde_json::{json, Value};
use std::collections::HashSet;

use mcp_cad_server::protocol::handle_message;
use mcp_cad_server::state::EngineState;

const PNG_MAGIC: [u8; 8] = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];

/// Helper to send a JSON-RPC message to the engine state and parse response.
async fn rpc_call(state: &mcp_cad_server::SharedState, req: Value) -> Value {
    let req_str = req.to_string();
    let resp = handle_message(&req_str, state)
        .await
        .expect("Expected a JSON-RPC response");
    serde_json::to_value(&resp).expect("Failed to serialize response")
}

#[tokio::test]
async fn test_mcp_initialize() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let init_req = json!({
        "jsonrpc": "2.0",
        "id": 1,
        "method": "initialize",
        "params": {
            "protocolVersion": "2024-11-05",
            "capabilities": {},
            "clientInfo": {
                "name": "antigravity-test-client",
                "version": "1.0.0"
            }
        }
    });

    let resp = rpc_call(&state, init_req).await;
    assert_eq!(resp["jsonrpc"], "2.0");
    assert_eq!(resp["id"], 1);

    let result = &resp["result"];
    assert_eq!(result["protocolVersion"], "2024-11-05");
    assert_eq!(result["serverInfo"]["name"], "mcp-cad-engine");
    assert_eq!(result["serverInfo"]["version"], "0.1.0");
    assert!(result["capabilities"]["tools"].is_object());
}

#[tokio::test]
async fn test_mcp_tools_list() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let list_req = json!({
        "jsonrpc": "2.0",
        "id": 2,
        "method": "tools/list"
    });

    let resp = rpc_call(&state, list_req).await;
    assert_eq!(resp["id"], 2);

    let tools = resp["result"]["tools"]
        .as_array()
        .expect("tools must be an array");

    // Must declare exactly 18 tools
    assert_eq!(tools.len(), 18, "Server must expose exactly 18 MCP tools");

    let expected_tools = [
        "create_part",
        "list_parts",
        "delete_part",
        "add_feature_primitive",
        "cut_hole",
        "modify_feature",
        "get_feature_tree",
        "cut_helical_groove",
        "boolean_difference",
        "create_joint",
        "set_joint_motor",
        "run_physics_simulation",
        "run_fea_analysis",
        "get_fea_heatmap_image",
        "capture_viewport",
        "export_stl",
        "get_simulation_metrics_json",
        "open_preview_window",
    ];

    let mut found_names = HashSet::new();

    for tool in tools {
        let name = tool["name"].as_str().expect("Tool must have a name");
        found_names.insert(name.to_string());

        let desc = tool["description"].as_str().unwrap_or("");
        assert!(!desc.is_empty(), "Tool '{name}' must have a non-empty description");

        let schema = &tool["inputSchema"];
        assert_eq!(
            schema["type"], "object",
            "Tool '{name}' inputSchema must be of type 'object'"
        );
        assert!(
            schema["properties"].is_object(),
            "Tool '{name}' inputSchema must have 'properties'"
        );
    }

    for expected in expected_tools {
        assert!(
            found_names.contains(expected),
            "Expected tool '{expected}' was not found in tools/list"
        );
    }
}

#[tokio::test]
async fn test_mcp_e2e_engineering_workflow() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create a structural aluminum part
    let create_req = json!({
        "jsonrpc": "2.0",
        "id": 10,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Cantilever Arm",
                "material": "aluminum_6061_t6"
            }
        }
    });

    let resp = rpc_call(&state, create_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let data: Value = serde_json::from_str(text).unwrap();
    let part_id = data["part_id"].as_str().unwrap().to_string();
    assert!(part_id.starts_with("part-"));

    // 2. Add box primitive (1.0m x 0.1m x 0.1m beam)
    let add_prim_req = json!({
        "jsonrpc": "2.0",
        "id": 11,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_id,
                "primitive_type": "box",
                "params": {
                    "width": 1.0,
                    "height": 0.1,
                    "depth": 0.1
                }
            }
        }
    });

    let resp = rpc_call(&state, add_prim_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let data: Value = serde_json::from_str(text).unwrap();
    let feature_id = data["feature_id"].as_str().unwrap().to_string();
    assert!(!feature_id.is_empty());

    // 3. Verify DAG history
    let tree_req = json!({
        "jsonrpc": "2.0",
        "id": 12,
        "method": "tools/call",
        "params": {
            "name": "get_feature_tree",
            "arguments": {
                "part_id": part_id
            }
        }
    });
    let resp = rpc_call(&state, tree_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let tree: Value = serde_json::from_str(text).unwrap();
    assert_eq!(tree["dag"].as_array().unwrap().len(), 1);

    // 4. Run FEA Analysis on the beam
    let fea_req = json!({
        "jsonrpc": "2.0",
        "id": 13,
        "method": "tools/call",
        "params": {
            "name": "run_fea_analysis",
            "arguments": {
                "part_id": part_id,
                "mesh_density": "coarse",
                "fixed_faces": ["min_x"],
                "applied_force": {
                    "position": [1.0, 0.05, 0.05],
                    "force_vector": [0.0, -1000.0, 0.0]
                }
            }
        }
    });
    let resp = rpc_call(&state, fea_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let fea: Value = serde_json::from_str(text).unwrap();

    let von_mises = fea["max_von_mises_mpa"].as_f64().unwrap();
    let fos = fea["safety_factor"].as_f64().unwrap();
    let disp_mm = fea["max_displacement_mm"].as_f64().unwrap();
    let status = fea["status"].as_str().unwrap();

    assert!(von_mises > 0.0, "Stress must be positive");
    assert!(fos > 0.0, "Factor of safety must be positive");
    assert!(disp_mm > 0.0, "Displacement must be positive");
    assert!(status == "PASS" || status == "FAIL");

    // 5. Capture 3D viewport
    let capture_req = json!({
        "jsonrpc": "2.0",
        "id": 14,
        "method": "tools/call",
        "params": {
            "name": "capture_viewport",
            "arguments": {
                "camera_preset": "isometric",
                "width": 640,
                "height": 480
            }
        }
    });
    let resp = rpc_call(&state, capture_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let view: Value = serde_json::from_str(text).unwrap();
    let b64 = view["image_base64"].as_str().unwrap();
    assert!(!b64.is_empty(), "Viewport base64 image must not be empty");

    let png_bytes = base64::prelude::BASE64_STANDARD.decode(b64).unwrap();
    assert_eq!(&png_bytes[0..8], &PNG_MAGIC, "Must be valid PNG signature");

    // 6. Capture FEA heatmap image
    let heatmap_req = json!({
        "jsonrpc": "2.0",
        "id": 15,
        "method": "tools/call",
        "params": {
            "name": "get_fea_heatmap_image",
            "arguments": {
                "part_id": part_id,
                "camera_preset": "front",
                "width": 640,
                "height": 480
            }
        }
    });
    let resp = rpc_call(&state, heatmap_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let heatmap: Value = serde_json::from_str(text).unwrap();
    let heat_b64 = heatmap["image_base64"].as_str().unwrap();
    assert!(!heat_b64.is_empty());
    let heat_bytes = base64::prelude::BASE64_STANDARD.decode(heat_b64).unwrap();
    assert_eq!(&heat_bytes[0..8], &PNG_MAGIC);
    assert!(heatmap["colormap_legend"].is_object());

    // 7. Export to binary STL file
    let tmp_dir = std::env::temp_dir();
    let stl_path = tmp_dir.join(format!("test_beam_{}.stl", uuid::Uuid::new_v4()));
    let stl_path_str = stl_path.to_str().unwrap().to_string();

    let export_req = json!({
        "jsonrpc": "2.0",
        "id": 16,
        "method": "tools/call",
        "params": {
            "name": "export_stl",
            "arguments": {
                "part_id": part_id,
                "filepath": stl_path_str,
                "binary": true
            }
        }
    });
    let resp = rpc_call(&state, export_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let export_data: Value = serde_json::from_str(text).unwrap();
    assert!(export_data["triangles_count"].as_u64().unwrap() > 0);

    // Verify STL file exists on disk
    assert!(stl_path.exists(), "STL file must be created on disk");
    let file_len = std::fs::metadata(&stl_path).unwrap().len();
    assert!(file_len > 84, "Binary STL file must exceed header size");
    let _ = std::fs::remove_file(&stl_path);

    // 8. Modify feature dimensions
    let mod_req = json!({
        "jsonrpc": "2.0",
        "id": 17,
        "method": "tools/call",
        "params": {
            "name": "modify_feature",
            "arguments": {
                "part_id": part_id,
                "feature_id": feature_id,
                "params": {
                    "width": 1.5,
                    "height": 0.1,
                    "depth": 0.1
                }
            }
        }
    });
    let resp = rpc_call(&state, mod_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let mod_res: Value = serde_json::from_str(text).unwrap();
    assert_eq!(mod_res["status"], "updated");

    // 9. Cut hole
    let cut_req = json!({
        "jsonrpc": "2.0",
        "id": 18,
        "method": "tools/call",
        "params": {
            "name": "cut_hole",
            "arguments": {
                "part_id": part_id,
                "position": [0.5, 0.0, 0.0],
                "normal": [0.0, 1.0, 0.0],
                "radius": 0.02,
                "depth": 0.2
            }
        }
    });
    let resp = rpc_call(&state, cut_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let cut_res: Value = serde_json::from_str(text).unwrap();
    assert!(!cut_res["feature_id"].as_str().unwrap().is_empty());

    // 10. List parts
    let list_req = json!({
        "jsonrpc": "2.0",
        "id": 19,
        "method": "tools/call",
        "params": {
            "name": "list_parts",
            "arguments": {}
        }
    });
    let resp = rpc_call(&state, list_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let parts_list: Value = serde_json::from_str(text).unwrap();
    assert_eq!(parts_list["parts"].as_array().unwrap().len(), 1);

    // 11. Simulation metrics JSON
    let metrics_req = json!({
        "jsonrpc": "2.0",
        "id": 20,
        "method": "tools/call",
        "params": {
            "name": "get_simulation_metrics_json",
            "arguments": {}
        }
    });
    let resp = rpc_call(&state, metrics_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let metrics: Value = serde_json::from_str(text).unwrap();
    assert!(metrics["current_metrics"]["parts"].is_array());

    // 12. Delete part
    let del_req = json!({
        "jsonrpc": "2.0",
        "id": 21,
        "method": "tools/call",
        "params": {
            "name": "delete_part",
            "arguments": {
                "part_id": part_id
            }
        }
    });
    let resp = rpc_call(&state, del_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let del_res: Value = serde_json::from_str(text).unwrap();
    assert_eq!(del_res["status"], "deleted");
}

#[tokio::test]
async fn test_mcp_physics_and_joints() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // Create Base Part
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 30,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Base Anchor",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let p_base: Value = serde_json::from_str(text).unwrap();
    let base_id = p_base["part_id"].as_str().unwrap().to_string();

    // Add box to base
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 31,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": base_id,
                    "primitive_type": "box",
                    "params": { "width": 0.5, "height": 0.5, "depth": 0.5 }
                }
            }
        }),
    )
    .await;

    // Create Rotor Part
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 32,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Rotor Link",
                    "material": "aluminum_6061_t6"
                }
            }
        }),
    )
    .await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let p_rotor: Value = serde_json::from_str(text).unwrap();
    let rotor_id = p_rotor["part_id"].as_str().unwrap().to_string();

    // Add cylinder to rotor
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 33,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": rotor_id,
                    "primitive_type": "cylinder",
                    "params": { "radius": 0.1, "height": 0.8 }
                }
            }
        }),
    )
    .await;

    // Create revolute joint between base and rotor
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 34,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_type": "revolute",
                    "part_a": base_id,
                    "part_b": rotor_id,
                    "anchor_a": [0.0, 0.25, 0.0],
                    "anchor_b": [0.0, -0.4, 0.0],
                    "axis": [0.0, 1.0, 0.0]
                }
            }
        }),
    )
    .await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let j_data: Value = serde_json::from_str(text).unwrap();
    let joint_id = j_data["joint_id"].as_str().unwrap().to_string();
    assert!(joint_id.starts_with("joint-"));

    // Set motor on revolute joint
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 35,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": joint_id,
                    "target_velocity": 5.0,
                    "max_force": 100.0
                }
            }
        }),
    )
    .await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let motor_res: Value = serde_json::from_str(text).unwrap();
    assert_eq!(motor_res["status"], "motor_active");

    // Run physics simulation for 0.5 seconds
    let resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 36,
            "method": "tools/call",
            "params": {
                "name": "run_physics_simulation",
                "arguments": {
                    "duration_seconds": 0.5,
                    "step_size_ms": 16.6667
                }
            }
        }),
    )
    .await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let sim: Value = serde_json::from_str(text).unwrap();
    assert_eq!(sim["success"], true);
    assert!(sim["steps_simulated"].as_u64().unwrap() > 0);
    assert!(sim["trajectory_summary"].is_object());
}

#[tokio::test]
async fn test_mcp_sse_endpoints() {
    use axum::body::Body;
    use axum::http::{Request, StatusCode};
    use tower::ServiceExt;

    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let (app, _) = mcp_cad_server::sse::build_router(state);

    // Test POST /message
    let req_body = json!({
        "jsonrpc": "2.0",
        "id": 42,
        "method": "ping"
    });

    let request = Request::builder()
        .method("POST")
        .uri("/message")
        .header("content-type", "application/json")
        .body(Body::from(req_body.to_string()))
        .unwrap();

    let response = app.clone().oneshot(request).await.unwrap();
    assert_eq!(response.status(), StatusCode::OK);

    let body_bytes = axum::body::to_bytes(response.into_body(), usize::MAX)
        .await
        .unwrap();
    let body_json: Value = serde_json::from_slice(&body_bytes).unwrap();
    assert_eq!(body_json["jsonrpc"], "2.0");
    assert_eq!(body_json["id"], 42);
    assert_eq!(body_json["result"], json!({}));

    // Test GET /sse endpoint connects
    let sse_request = Request::builder()
        .method("GET")
        .uri("/sse")
        .body(Body::empty())
        .unwrap();

    let sse_response = app.oneshot(sse_request).await.unwrap();
    assert_eq!(sse_response.status(), StatusCode::OK);
    let content_type = sse_response.headers().get("content-type").unwrap();
    assert!(content_type.to_str().unwrap().contains("text/event-stream"));
}

#[tokio::test]
async fn test_mcp_open_preview_window() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // Call open_preview_window in headless mode
    let req = json!({
        "jsonrpc": "2.0",
        "id": 100,
        "method": "tools/call",
        "params": {
            "name": "open_preview_window",
            "arguments": {}
        }
    });

    let resp = rpc_call(&state, req).await;
    assert_eq!(resp["id"], 100);
    assert!(resp["result"]["content"].is_array());
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let val: Value = serde_json::from_str(text).unwrap();
    assert_eq!(val["status"], "headless");
}

#[tokio::test]
async fn test_mcp_cone_and_helical_groove_and_boolean_difference() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create a tool / drill part
    let create_drill_req = json!({
        "jsonrpc": "2.0",
        "id": 201,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Twist Drill",
                "material": "steel_structural_aisi1020"
            }
        }
    });
    let resp = rpc_call(&state, create_drill_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let data: Value = serde_json::from_str(text).unwrap();
    let drill_id = data["part_id"].as_str().unwrap().to_string();

    // 2. Add cylinder base (shank + body)
    let add_cyl_req = json!({
        "jsonrpc": "2.0",
        "id": 202,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": drill_id,
                "primitive_type": "cylinder",
                "params": {
                    "radius": 0.01,
                    "height": 0.1,
                    "segments": 32,
                    "position": [0.0, 0.0, 0.05]
                }
            }
        }
    });
    let resp = rpc_call(&state, add_cyl_req).await;
    assert_eq!(resp["id"], 202);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let cyl_data: Value = serde_json::from_str(text).unwrap();
    assert!(cyl_data["feature_id"].as_str().is_some());

    // 3. Add cone tip with position
    let add_cone_req = json!({
        "jsonrpc": "2.0",
        "id": 203,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": drill_id,
                "primitive_type": "cone",
                "params": {
                    "radius_bottom": 0.01,
                    "radius_top": 0.0,
                    "height": 0.02,
                    "segments": 32,
                    "position": [0.0, 0.0, 0.11]
                }
            }
        }
    });
    let resp = rpc_call(&state, add_cone_req).await;
    assert_eq!(resp["id"], 203);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let cone_data: Value = serde_json::from_str(text).unwrap();
    assert!(cone_data["feature_id"].as_str().is_some());

    // 4. Cut helical groove (flutes)
    let cut_helix_req = json!({
        "jsonrpc": "2.0",
        "id": 204,
        "method": "tools/call",
        "params": {
            "name": "cut_helical_groove",
            "arguments": {
                "part_id": drill_id,
                "helix_radius": 0.01,
                "groove_radius": 0.004,
                "pitch": 0.04,
                "height": 0.08,
                "start_z": 0.02,
                "starts": 2,
                "is_right_hand": true
            }
        }
    });
    let resp = rpc_call(&state, cut_helix_req).await;
    assert_eq!(resp["id"], 204);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let helix_data: Value = serde_json::from_str(text).unwrap();
    assert_eq!(helix_data["status"], "helical_groove_cut");
    assert!(helix_data["feature_id"].as_str().is_some());

    // 5. Test boolean difference: create a block and subtract the drill
    let create_block_req = json!({
        "jsonrpc": "2.0",
        "id": 205,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Machining Block",
                "material": "aluminum_6061_t6"
            }
        }
    });
    let resp = rpc_call(&state, create_block_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let block_data: Value = serde_json::from_str(text).unwrap();
    let block_id = block_data["part_id"].as_str().unwrap().to_string();

    let add_box_req = json!({
        "jsonrpc": "2.0",
        "id": 206,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": block_id,
                "primitive_type": "box",
                "params": {
                    "width": 0.1,
                    "height": 0.1,
                    "depth": 0.15,
                    "position": [0.0, 0.0, 0.05]
                }
            }
        }
    });
    let _ = rpc_call(&state, add_box_req).await;

    let bool_diff_req = json!({
        "jsonrpc": "2.0",
        "id": 207,
        "method": "tools/call",
        "params": {
            "name": "boolean_difference",
            "arguments": {
                "target_part_id": block_id,
                "tool_part_id": drill_id
            }
        }
    });
    let resp = rpc_call(&state, bool_diff_req).await;
    assert_eq!(resp["id"], 207);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let diff_data: Value = serde_json::from_str(text).unwrap();
    assert_eq!(diff_data["status"], "boolean_difference_applied");
    assert!(diff_data["feature_id"].as_str().is_some());

    // Check DAG feature tree of block has 2 features (box and boolean diff)
    let tree_req = json!({
        "jsonrpc": "2.0",
        "id": 208,
        "method": "tools/call",
        "params": {
            "name": "get_feature_tree",
            "arguments": {
                "part_id": block_id
            }
        }
    });
    let resp = rpc_call(&state, tree_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let tree: Value = serde_json::from_str(text).unwrap();
    assert_eq!(tree["dag"].as_array().unwrap().len(), 2);
}
