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

    let resp = rpc_call(&state, list_req.clone()).await;
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
    let resp = rpc_call(&state, list_req.clone()).await;
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

#[tokio::test]
async fn test_mcp_polygon_prism_and_metric_thread() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create part for hex nut
    let create_req = json!({
        "jsonrpc": "2.0",
        "id": 301,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Hex Nut M10 DIN934",
                "material": "steel_structural_aisi1020"
            }
        }
    });
    let resp = rpc_call(&state, create_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let data: Value = serde_json::from_str(text).unwrap();
    let nut_id = data["part_id"].as_str().unwrap().to_string();

    // 2. Add polygon_prism primitive with DIN 934 chamfer
    let add_prism_req = json!({
        "jsonrpc": "2.0",
        "id": 302,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": nut_id,
                "primitive_type": "polygon_prism",
                "params": {
                    "sides": 6,
                    "width_across_flats": 0.016,
                    "height": 0.008,
                    "chamfer": 0.0008
                }
            }
        }
    });
    let resp = rpc_call(&state, add_prism_req).await;
    assert_eq!(resp["id"], 302);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let prism_data: Value = serde_json::from_str(text).unwrap();
    let prism_feat_id = prism_data["feature_id"].as_str().unwrap().to_string();

    // 3. Cut tap hole through center
    let cut_hole_req = json!({
        "jsonrpc": "2.0",
        "id": 303,
        "method": "tools/call",
        "params": {
            "name": "cut_hole",
            "arguments": {
                "part_id": nut_id,
                "position": [0.0, 0.0, 0.0],
                "normal": [0.0, 0.0, 1.0],
                "radius": 0.00425,
                "depth": 0.010
            }
        }
    });
    let resp = rpc_call(&state, cut_hole_req).await;
    assert_eq!(resp["id"], 303);

    // 4. Cut internal metric 60-degree thread
    let cut_thread_req = json!({
        "jsonrpc": "2.0",
        "id": 304,
        "method": "tools/call",
        "params": {
            "name": "cut_helical_groove",
            "arguments": {
                "part_id": nut_id,
                "helix_radius": 0.00425,
                "groove_radius": 0.00085,
                "pitch": 0.0015,
                "height": 0.008,
                "start_z": -0.004,
                "starts": 1,
                "is_right_hand": true,
                "profile_type": "metric_60"
            }
        }
    });
    let resp = rpc_call(&state, cut_thread_req).await;
    assert_eq!(resp["id"], 304);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let thread_data: Value = serde_json::from_str(text).unwrap();
    assert_eq!(thread_data["status"], "helical_groove_cut");

    // 5. Test modify_feature on polygon_prism
    let mod_req = json!({
        "jsonrpc": "2.0",
        "id": 305,
        "method": "tools/call",
        "params": {
            "name": "modify_feature",
            "arguments": {
                "part_id": nut_id,
                "feature_id": prism_feat_id,
                "params": {
                    "chamfer": 0.001
                }
            }
        }
    });
    let resp = rpc_call(&state, mod_req).await;
    assert_eq!(resp["id"], 305);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let mod_data: Value = serde_json::from_str(text).unwrap();
    assert_eq!(mod_data["status"], "updated");
}

#[tokio::test]
async fn test_mcp_spur_gear_and_keyway() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create part for transmission drive gear
    let create_req = json!({
        "jsonrpc": "2.0",
        "id": 401,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Transmission Drive Pinion",
                "material": "steel_structural_aisi1020"
            }
        }
    });
    let resp = rpc_call(&state, create_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let data: Value = serde_json::from_str(text).unwrap();
    let gear_part_id = data["part_id"].as_str().unwrap().to_string();

    // 2. Add spur_gear primitive with involute teeth, mounting hub, bore and DIN 6885 keyway
    let add_gear_req = json!({
        "jsonrpc": "2.0",
        "id": 402,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": gear_part_id,
                "primitive_type": "spur_gear",
                "params": {
                    "module": 0.002,
                    "teeth": 20,
                    "pressure_angle_deg": 20.0,
                    "face_width": 0.015,
                    "hub_diameter": 0.025,
                    "hub_length": 0.010,
                    "bore_diameter": 0.012,
                    "keyway_width": 0.004,
                    "keyway_depth": 0.0018
                }
            }
        }
    });
    let resp = rpc_call(&state, add_gear_req).await;
    assert_eq!(resp["id"], 402);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let gear_data: Value = serde_json::from_str(text).unwrap();
    let gear_feat_id = gear_data["feature_id"].as_str().unwrap().to_string();

    // 3. Inspect part summary via list_parts
    let list_req = json!({
        "jsonrpc": "2.0",
        "id": 403,
        "method": "tools/call",
        "params": {
            "name": "list_parts",
            "arguments": {}
        }
    });
    let resp = rpc_call(&state, list_req.clone()).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let list_data: Value = serde_json::from_str(text).unwrap();
    let parts_arr = list_data["parts"].as_array().unwrap();
    let gear_summary = parts_arr.iter().find(|p| p["id"] == gear_part_id).unwrap();
    let vol1 = gear_summary["volume"].as_f64().unwrap();
    assert!(vol1 > 0.0, "Spur gear volume must be positive: got {}", vol1);
    let dims = gear_summary["bounding_box_dimensions"].as_array().unwrap();
    let z_span = dims[2].as_f64().unwrap();
    // 0.015 face_width + 0.010 hub_length = 0.025 m total axial length
    assert!((z_span - 0.025).abs() < 1e-4, "Z dimension should be 0.025, got {}", z_span);

    // 4. Modify spur_gear: increase teeth from 20 to 24
    let mod_req = json!({
        "jsonrpc": "2.0",
        "id": 404,
        "method": "tools/call",
        "params": {
            "name": "modify_feature",
            "arguments": {
                "part_id": gear_part_id,
                "feature_id": gear_feat_id,
                "params": {
                    "teeth": 24
                }
            }
        }
    });
    let resp = rpc_call(&state, mod_req).await;
    assert_eq!(resp["id"], 404);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let mod_data: Value = serde_json::from_str(text).unwrap();
    assert_eq!(mod_data["status"], "updated");

    // Check that volume increased with more teeth
    let resp = rpc_call(&state, list_req.clone()).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let list_data2: Value = serde_json::from_str(text).unwrap();
    let gear_summary2 = list_data2["parts"].as_array().unwrap().iter().find(|p| p["id"] == gear_part_id).unwrap();
    let vol2 = gear_summary2["volume"].as_f64().unwrap();
    assert!(
        vol2 > vol1,
        "Volume after increasing teeth to 24 ({}) must exceed 20 teeth ({})",
        vol2,
        vol1
    );

    // 5. Test cut_hole with DIN 6885 keyway on a separate cylindrical hub part
    let create_hub_req = json!({
        "jsonrpc": "2.0",
        "id": 405,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Coupling Sleeve",
                "material": "aluminum_6061_t6"
            }
        }
    });
    let resp = rpc_call(&state, create_hub_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let hub_part_id = serde_json::from_str::<Value>(text).unwrap()["part_id"].as_str().unwrap().to_string();

    let add_cyl_req = json!({
        "jsonrpc": "2.0",
        "id": 406,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": hub_part_id,
                "primitive_type": "cylinder",
                "params": {
                    "radius": 0.025,
                    "height": 0.040
                }
            }
        }
    });
    let resp = rpc_call(&state, add_cyl_req).await;
    assert_eq!(resp["id"], 406);

    let cut_keyed_hole_req = json!({
        "jsonrpc": "2.0",
        "id": 407,
        "method": "tools/call",
        "params": {
            "name": "cut_hole",
            "arguments": {
                "part_id": hub_part_id,
                "position": [0.0, 0.0, 0.020],
                "normal": [0.0, 0.0, 1.0],
                "radius": 0.010,
                "depth": 0.045,
                "keyway_width": 0.006,
                "keyway_depth": 0.0028
            }
        }
    });
    let resp = rpc_call(&state, cut_keyed_hole_req).await;
    assert_eq!(resp["id"], 407);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let cut_data: Value = serde_json::from_str(text).unwrap();
    assert!(!cut_data["feature_id"].as_str().unwrap().is_empty());
}

#[tokio::test]
async fn test_mcp_revolve_and_wheel_rim() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Verify schema in tools/list
    let list_tools_req = json!({
        "jsonrpc": "2.0",
        "id": 501,
        "method": "tools/list",
        "params": {}
    });
    let resp = rpc_call(&state, list_tools_req).await;
    let tools = resp["result"]["tools"].as_array().unwrap();
    let add_prim = tools.iter().find(|t| t["name"] == "add_feature_primitive").unwrap();
    let prim_enum = add_prim["inputSchema"]["properties"]["primitive_type"]["enum"].as_array().unwrap();
    assert!(prim_enum.iter().any(|v| v.as_str() == Some("revolve")));
    assert!(prim_enum.iter().any(|v| v.as_str() == Some("wheel_rim")));

    // 2. Test revolve primitive
    let create_part_req = json!({
        "jsonrpc": "2.0",
        "id": 502,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Shaft Collar",
                "material": "steel_structural_aisi1020"
            }
        }
    });
    let resp = rpc_call(&state, create_part_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let collar_part_id = serde_json::from_str::<Value>(text).unwrap()["part_id"].as_str().unwrap().to_string();

    let add_rev_req = json!({
        "jsonrpc": "2.0",
        "id": 503,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": collar_part_id,
                "primitive_type": "revolve",
                "params": {
                    "profile": [
                        [0.0, -0.05],
                        [0.03, -0.05],
                        [0.03, 0.05],
                        [0.0, 0.05]
                    ],
                    "angle_deg": 360.0,
                    "segments": 32
                }
            }
        }
    });
    let resp = rpc_call(&state, add_rev_req).await;
    assert_eq!(resp["id"], 503);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let rev_feat_id = serde_json::from_str::<Value>(text).unwrap()["feature_id"].as_str().unwrap().to_string();

    let list_req = json!({
        "jsonrpc": "2.0",
        "id": 504,
        "method": "tools/call",
        "params": {
            "name": "list_parts",
            "arguments": {}
        }
    });
    let resp = rpc_call(&state, list_req.clone()).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let list_data: Value = serde_json::from_str(text).unwrap();
    let collar_summary = list_data["parts"].as_array().unwrap().iter().find(|p| p["id"] == collar_part_id).unwrap();
    let vol_full = collar_summary["volume"].as_f64().unwrap();
    assert!(vol_full > 0.0, "Revolve volume should be positive: got {}", vol_full);

    // Modify revolve to 180 degrees
    let mod_rev_req = json!({
        "jsonrpc": "2.0",
        "id": 505,
        "method": "tools/call",
        "params": {
            "name": "modify_feature",
            "arguments": {
                "part_id": collar_part_id,
                "feature_id": rev_feat_id,
                "params": {
                    "angle_deg": 180.0
                }
            }
        }
    });
    let resp = rpc_call(&state, mod_rev_req).await;
    assert_eq!(resp["id"], 505);

    let resp = rpc_call(&state, list_req.clone()).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let list_data2: Value = serde_json::from_str(text).unwrap();
    let collar_summary2 = list_data2["parts"].as_array().unwrap().iter().find(|p| p["id"] == collar_part_id).unwrap();
    let vol_half = collar_summary2["volume"].as_f64().unwrap();
    assert!(
        (vol_half - vol_full * 0.5).abs() / (vol_full * 0.5) < 0.05,
        "180-deg revolve volume ({}) should be ~half of 360-deg ({})",
        vol_half,
        vol_full
    );

    // 3. Test wheel_rim primitive
    let create_wheel_req = json!({
        "jsonrpc": "2.0",
        "id": 506,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Forged Alloy Wheel",
                "material": "aluminum_6061_t6"
            }
        }
    });
    let resp = rpc_call(&state, create_wheel_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let wheel_part_id = serde_json::from_str::<Value>(text).unwrap()["part_id"].as_str().unwrap().to_string();

    let add_wheel_req = json!({
        "jsonrpc": "2.0",
        "id": 507,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": wheel_part_id,
                "primitive_type": "wheel_rim",
                "params": {
                    "rim_diameter": 0.4572,
                    "rim_width": 0.2032,
                    "flange_diameter": 0.490,
                    "hub_diameter": 0.160,
                    "hub_bore_diameter": 0.066,
                    "pcd_diameter": 0.1143,
                    "bolt_holes_count": 5,
                    "bolt_hole_diameter": 0.014,
                    "spokes_count": 5,
                    "spoke_width": 0.028,
                    "spoke_thickness": 0.018,
                    "offset_et": 0.035
                }
            }
        }
    });
    let resp = rpc_call(&state, add_wheel_req).await;
    assert_eq!(resp["id"], 507);
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let wheel_feat_id = serde_json::from_str::<Value>(text).unwrap()["feature_id"].as_str().unwrap().to_string();

    let resp = rpc_call(&state, list_req.clone()).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let list_data3: Value = serde_json::from_str(text).unwrap();
    let wheel_summary = list_data3["parts"].as_array().unwrap().iter().find(|p| p["id"] == wheel_part_id).unwrap();
    let vol_wheel = wheel_summary["volume"].as_f64().unwrap();
    assert!(vol_wheel > 0.001, "Wheel volume should be positive: got {}", vol_wheel);

    let dims = wheel_summary["bounding_box_dimensions"].as_array().unwrap();
    assert!((dims[2].as_f64().unwrap() - 0.2032).abs() < 0.02);

    // Modify wheel rim: increase rim_width to 0.254
    let mod_wheel_req = json!({
        "jsonrpc": "2.0",
        "id": 508,
        "method": "tools/call",
        "params": {
            "name": "modify_feature",
            "arguments": {
                "part_id": wheel_part_id,
                "feature_id": wheel_feat_id,
                "params": {
                    "rim_width": 0.254
                }
            }
        }
    });
    let resp = rpc_call(&state, mod_wheel_req).await;
    assert_eq!(resp["id"], 508);

    let resp = rpc_call(&state, list_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let list_data4: Value = serde_json::from_str(text).unwrap();
    let wheel_summary2 = list_data4["parts"].as_array().unwrap().iter().find(|p| p["id"] == wheel_part_id).unwrap();
    let vol_wheel2 = wheel_summary2["volume"].as_f64().unwrap();
    assert!(
        vol_wheel2 > vol_wheel,
        "Wider wheel volume ({}) must exceed narrower wheel ({})",
        vol_wheel2,
        vol_wheel
    );
}
