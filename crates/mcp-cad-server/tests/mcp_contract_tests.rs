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

/// Asserts that a `tools/call` response conforms to the MCP tool error contract:
/// top-level JSON-RPC `error` is absent/null, `result.isError == true`, and
/// `result.content[0].text` contains a non-empty error message.
fn assert_tool_error(resp: &Value) -> &str {
    assert!(
        resp.get("error").is_none() || resp["error"].is_null(),
        "Tool error must not use top-level JSON-RPC 'error' field, got: {resp}"
    );
    assert_eq!(
        resp["result"]["isError"],
        json!(true),
        "Expected result.isError == true for tool execution error, got: {resp}"
    );
    let text = resp["result"]["content"]
        .as_array()
        .and_then(|arr| arr.first())
        .and_then(|c| c.get("text"))
        .and_then(|v| v.as_str())
        .unwrap_or("");
    assert!(
        !text.trim().is_empty(),
        "Expected non-empty error message in result.content[0].text, got: {resp}"
    );
    let parsed: Value = serde_json::from_str(text)
        .unwrap_or_else(|e| panic!("Error message in content[0].text must be valid JSON: {e}, text: {text}"));
    assert!(
        parsed.get("error").is_some(),
        "Structured error must contain 'error' object: {text}"
    );
    assert!(
        parsed["error"].get("code").and_then(|c| c.as_str()).is_some(),
        "Structured error must contain non-empty 'error.code': {text}"
    );
    assert!(
        parsed["error"].get("message").and_then(|m| m.as_str()).is_some(),
        "Structured error must contain non-empty 'error.message': {text}"
    );
    text
}

/// Parses `result.content[0].text` of a successful MCP tool call response into JSON `Value`.
fn parse_content(resp: &Value) -> Value {
    assert!(
        resp.get("error").is_none() || resp["error"].is_null(),
        "Expected successful tool response without JSON-RPC error, got: {resp}"
    );
    assert!(
        resp["result"].get("isError").is_none() || resp["result"]["isError"] == json!(false),
        "Expected successful tool response (isError != true), got: {resp}"
    );
    let text = resp["result"]["content"][0]["text"]
        .as_str()
        .unwrap_or_else(|| panic!("Expected string in result.content[0].text, got: {resp}"));
    serde_json::from_str(text)
        .unwrap_or_else(|e| panic!("Failed to parse JSON in result.content[0].text: {e}, text: {text}"))
}

/// Asserts a tool error and deserializes the structured `ErrorEnvelope`.
fn parse_error_envelope(resp: &Value) -> mcp_cad_server::ErrorEnvelope {
    let text = assert_tool_error(resp);
    serde_json::from_str(text).expect("Failed to deserialize ErrorEnvelope")
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
    assert_eq!(result["length_unit"], "mm");
    assert_eq!(result["units"]["length"], "mm");
    assert_eq!(result["units"]["stress_output"], "MPa");
    assert!(result["capabilities"]["tools"].is_object());
    let instructions = result["instructions"]
        .as_str()
        .expect("initialize response must include 'instructions' string");
    assert!(
        instructions.contains("docs/AGENT_GUIDE.md") && instructions.contains("get_scene_summary"),
        "initialize.instructions must reference docs/AGENT_GUIDE.md and get_scene_summary"
    );
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

    // Must declare exactly 34 tools
    assert_eq!(tools.len(), 34, "Server must expose exactly 34 MCP tools");

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
        "import_stl",
        "tessellate_part",
        "query_part_geometry",
        "boolean_union",
        "boolean_intersection",
        "transform_part",
        "define_material",
        "batch_cad_operations",
        "validate_mesh",
        "optimize_feature_tree",
        "set_units",
        "save_project",
        "load_project",
        "set_requirements",
        "check_requirements",
        "get_scene_summary",
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

    // 2. Add box primitive (1000mm x 100mm x 100mm beam)
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
                    "width": 1000.0,
                    "height": 100.0,
                    "depth": 100.0
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
                    "position": [1000.0, 50.0, 50.0],
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
                    "width": 1500.0,
                    "height": 100.0,
                    "depth": 100.0
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
                "position": [500.0, 0.0, 0.0],
                "normal": [0.0, 1.0, 0.0],
                "radius": 20.0,
                "depth": 200.0
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
                    "params": { "width": 500.0, "height": 500.0, "depth": 500.0 }
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
                    "params": { "radius": 100.0, "height": 800.0 }
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
                    "anchor_a": [0.0, 250.0, 0.0],
                    "anchor_b": [0.0, -400.0, 0.0],
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
                    "radius": 10.0,
                    "height": 100.0,
                    "segments": 32,
                    "position": [0.0, 0.0, 50.0]
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
                    "radius_bottom": 10.0,
                    "radius_top": 0.0,
                    "height": 20.0,
                    "segments": 32,
                    "position": [0.0, 0.0, 110.0]
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
                "helix_radius": 10.0,
                "groove_radius": 4.0,
                "pitch": 40.0,
                "height": 80.0,
                "start_z": 20.0,
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
                    "width": 100.0,
                    "height": 100.0,
                    "depth": 150.0,
                    "position": [0.0, 0.0, 50.0]
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
                    "width_across_flats": 16.0,
                    "height": 8.0,
                    "chamfer": 0.8
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
                "radius": 4.25,
                "depth": 10.0
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
                "helix_radius": 4.25,
                "groove_radius": 0.85,
                "pitch": 1.5,
                "height": 8.0,
                "start_z": -4.0,
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
                    "chamfer": 1.0
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
                    "module": 2.0,
                    "teeth": 20,
                    "pressure_angle_deg": 20.0,
                    "face_width": 15.0,
                    "hub_diameter": 25.0,
                    "hub_length": 10.0,
                    "bore_diameter": 12.0,
                    "keyway_width": 4.0,
                    "keyway_depth": 1.8
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
    // 15.0 mm face_width + 10.0 mm hub_length = 25.0 mm total axial length
    assert!((z_span - 25.0).abs() < 0.1, "Z dimension should be 25.0 mm, got {}", z_span);

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
                    "radius": 25.0,
                    "height": 40.0
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
                "position": [0.0, 0.0, 20.0],
                "normal": [0.0, 0.0, 1.0],
                "radius": 10.0,
                "depth": 45.0,
                "keyway_width": 6.0,
                "keyway_depth": 2.8
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
                        [0.0, -50.0],
                        [30.0, -50.0],
                        [30.0, 50.0],
                        [0.0, 50.0]
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
                    "rim_diameter": 457.2,
                    "rim_width": 203.2,
                    "flange_diameter": 490.0,
                    "hub_diameter": 160.0,
                    "hub_bore_diameter": 66.0,
                    "pcd_diameter": 114.3,
                    "bolt_holes_count": 5,
                    "bolt_hole_diameter": 14.0,
                    "spokes_count": 5,
                    "spoke_width": 28.0,
                    "spoke_thickness": 18.0,
                    "offset_et": 35.0
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
    let vol_wheel_m3 = wheel_summary["volume_m3"].as_f64().unwrap();
    assert!(vol_wheel_m3 > 0.001, "Wheel volume in m³ should be > 0.001: got {}", vol_wheel_m3);

    let dims = wheel_summary["bounding_box_dimensions"].as_array().unwrap();
    assert!((dims[2].as_f64().unwrap() - 203.2).abs() < 20.0);

    // Modify wheel rim: increase rim_width to 254.0 mm
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
                    "rim_width": 254.0
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

#[tokio::test]
async fn test_mcp_import_stl() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create a part and export it to an STL file first
    let part_req = json!({
        "jsonrpc": "2.0",
        "id": 601,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "stl_source_part",
                "material": "aluminum_6061_t6"
            }
        }
    });
    let resp = rpc_call(&state, part_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let src_part_id = serde_json::from_str::<Value>(text).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    let add_prim_req = json!({
        "jsonrpc": "2.0",
        "id": 602,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": src_part_id,
                "primitive_type": "box",
                "params": {
                    "width": 50.0,
                    "height": 30.0,
                    "depth": 20.0
                }
            }
        }
    });
    let _ = rpc_call(&state, add_prim_req).await;

    let tmp_dir = std::env::temp_dir();
    let stl_path = tmp_dir.join(format!("test_import_{}.stl", uuid::Uuid::new_v4().simple()));
    let stl_path_str = stl_path.to_str().unwrap().to_string();

    let export_req = json!({
        "jsonrpc": "2.0",
        "id": 603,
        "method": "tools/call",
        "params": {
            "name": "export_stl",
            "arguments": {
                "part_id": src_part_id,
                "filepath": stl_path_str.clone(),
                "binary": true
            }
        }
    });
    let _ = rpc_call(&state, export_req).await;
    assert!(stl_path.exists(), "Source STL must exist on disk");

    // 2. Import the STL via MCP import_stl tool
    let import_req = json!({
        "jsonrpc": "2.0",
        "id": 604,
        "method": "tools/call",
        "params": {
            "name": "import_stl",
            "arguments": {
                "filepath": stl_path_str,
                "name": "custom_imported_box",
                "material": "steel_structural_aisi1020"
            }
        }
    });
    let resp = rpc_call(&state, import_req).await;
    assert_eq!(resp["id"], 604);
    assert!(resp["error"].is_null());

    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let import_data: Value = serde_json::from_str(text).unwrap();

    let imported_part_id = import_data["part_id"].as_str().unwrap().to_string();
    assert_eq!(import_data["name"], "custom_imported_box");
    assert_eq!(import_data["status"], "stl_imported");
    assert_eq!(import_data["triangles_count"], 12);
    assert_eq!(import_data["vertices_count"], 36);
    assert!(import_data["bounding_box"].is_object());

    // 3. Verify the imported part appears in list_parts
    let list_req = json!({
        "jsonrpc": "2.0",
        "id": 605,
        "method": "tools/call",
        "params": {
            "name": "list_parts",
            "arguments": {}
        }
    });
    let resp = rpc_call(&state, list_req).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let list_data: Value = serde_json::from_str(text).unwrap();
    let found = list_data["parts"]
        .as_array()
        .unwrap()
        .iter()
        .any(|p| p["id"] == imported_part_id && p["name"] == "custom_imported_box");
    assert!(found, "Imported part must appear in list_parts");

    // Clean up
    let _ = std::fs::remove_file(&stl_path);
}

#[tokio::test]
async fn test_mcp_tessellation_and_stl_quality() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create a part (cylinder bracket)
    let create_req = json!({
        "jsonrpc": "2.0",
        "id": 701,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "adaptive_cylinder",
                "material": "aluminum_6061_t6"
            }
        }
    });
    let resp = rpc_call(&state, create_req).await;
    let part_id = resp["result"]["content"][0]["text"]
        .as_str()
        .and_then(|s| serde_json::from_str::<Value>(s).ok())
        .and_then(|v| v["part_id"].as_str().map(|s| s.to_string()))
        .expect("part_id expected");

    // 2. Add a cylinder primitive (curved geometry benefits from adaptive tessellation)
    let add_cyl_req = json!({
        "jsonrpc": "2.0",
        "id": 702,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_id,
                "primitive_type": "cylinder",
                "params": {
                    "radius": 50.0,
                    "height": 200.0
                }
            }
        }
    });
    let _ = rpc_call(&state, add_cyl_req).await;

    // 3. Test tessellate_part with coarse quality
    let tess_coarse_req = json!({
        "jsonrpc": "2.0",
        "id": 703,
        "method": "tools/call",
        "params": {
            "name": "tessellate_part",
            "arguments": {
                "part_id": part_id,
                "quality": "coarse"
            }
        }
    });
    let resp = rpc_call(&state, tess_coarse_req).await;
    assert_eq!(resp["id"], 703);
    assert!(resp["error"].is_null());
    let coarse_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let coarse_tris = coarse_data["triangle_count"].as_u64().unwrap();
    let coarse_verts = coarse_data["vertex_count"].as_u64().unwrap();
    assert_eq!(coarse_data["quality_applied"], "coarse");
    assert!(coarse_tris > 0);
    assert!(coarse_verts > 0);
    assert!(coarse_data["volume"].as_f64().unwrap() > 0.0);

    // 4. Test tessellate_part with fine quality
    let tess_fine_req = json!({
        "jsonrpc": "2.0",
        "id": 704,
        "method": "tools/call",
        "params": {
            "name": "tessellate_part",
            "arguments": {
                "part_id": part_id,
                "quality": "fine"
            }
        }
    });
    let resp = rpc_call(&state, tess_fine_req).await;
    assert_eq!(resp["id"], 704);
    assert!(resp["error"].is_null());
    let fine_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let fine_tris = fine_data["triangle_count"].as_u64().unwrap();
    assert_eq!(fine_data["quality_applied"], "fine");
    assert!(
        fine_tris > coarse_tris,
        "Fine tessellation triangles ({fine_tris}) must exceed Coarse triangles ({coarse_tris})"
    );

    // 5. Test tessellate_part with custom chord tolerance
    let tess_custom_req = json!({
        "jsonrpc": "2.0",
        "id": 705,
        "method": "tools/call",
        "params": {
            "name": "tessellate_part",
            "arguments": {
                "part_id": part_id,
                "chord_tolerance": 0.005,
                "angular_tolerance_deg": 2.0
            }
        }
    });
    let resp = rpc_call(&state, tess_custom_req).await;
    assert_eq!(resp["id"], 705);
    assert!(resp["error"].is_null());
    let custom_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(custom_data["quality_applied"], "custom");
    let custom_tris = custom_data["triangle_count"].as_u64().unwrap();
    assert!(custom_tris >= fine_tris);

    // 6. Test export_stl with quality "coarse"
    let tmp_dir = std::env::temp_dir();
    let coarse_path = tmp_dir.join(format!("test_coarse_{}.stl", uuid::Uuid::new_v4().simple()));
    let coarse_path_str = coarse_path.to_str().unwrap().to_string();

    let export_coarse_req = json!({
        "jsonrpc": "2.0",
        "id": 706,
        "method": "tools/call",
        "params": {
            "name": "export_stl",
            "arguments": {
                "part_id": part_id,
                "filepath": coarse_path_str,
                "binary": true,
                "quality": "coarse"
            }
        }
    });
    let resp = rpc_call(&state, export_coarse_req).await;
    assert_eq!(resp["id"], 706);
    assert!(resp["error"].is_null());
    let exp_coarse_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let exp_coarse_tris = exp_coarse_data["triangles_count"].as_u64().unwrap();
    assert_eq!(exp_coarse_data["quality_applied"], "coarse");
    assert!(coarse_path.exists());
    let coarse_file_size = std::fs::metadata(&coarse_path).unwrap().len();

    // 7. Test export_stl with quality "fine"
    let fine_path = tmp_dir.join(format!("test_fine_{}.stl", uuid::Uuid::new_v4().simple()));
    let fine_path_str = fine_path.to_str().unwrap().to_string();

    let export_fine_req = json!({
        "jsonrpc": "2.0",
        "id": 707,
        "method": "tools/call",
        "params": {
            "name": "export_stl",
            "arguments": {
                "part_id": part_id,
                "filepath": fine_path_str,
                "binary": true,
                "quality": "fine"
            }
        }
    });
    let resp = rpc_call(&state, export_fine_req).await;
    assert_eq!(resp["id"], 707);
    assert!(resp["error"].is_null());
    let exp_fine_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let exp_fine_tris = exp_fine_data["triangles_count"].as_u64().unwrap();
    assert_eq!(exp_fine_data["quality_applied"], "fine");
    assert!(fine_path.exists());
    let fine_file_size = std::fs::metadata(&fine_path).unwrap().len();

    // Assert: polygon count and file size for Fine > Coarse
    assert!(
        exp_fine_tris > exp_coarse_tris,
        "Fine STL triangles ({exp_fine_tris}) must exceed Coarse STL triangles ({exp_coarse_tris})"
    );
    assert!(
        fine_file_size > coarse_file_size,
        "Fine STL file size ({fine_file_size} bytes) must exceed Coarse STL file size ({coarse_file_size} bytes)"
    );

    // Clean up temporary files
    let _ = std::fs::remove_file(&coarse_path);
    let _ = std::fs::remove_file(&fine_path);
}

#[tokio::test]
async fn test_mcp_query_part_geometry() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create part
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 801,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "box_inspect",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    assert_eq!(resp["id"], 801);
    let part_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let part_id = part_data["part_id"].as_str().unwrap();

    // 2. Add box: 10 x 20 x 30
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 802,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_id,
                "primitive_type": "box",
                "width": 10.0,
                "height": 20.0,
                "depth": 30.0
            }
        }
    })).await;
    assert_eq!(resp["id"], 802);

    // 3. Query geometry
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 803,
        "method": "tools/call",
        "params": {
            "name": "query_part_geometry",
            "arguments": {
                "part_id": part_id
            }
        }
    })).await;
    assert_eq!(resp["id"], 803);
    assert!(resp["error"].is_null());
    let geom: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();

    assert_eq!(geom["part_id"], part_id);
    assert!((geom["volume"].as_f64().unwrap() - 6000.0).abs() < 1e-3);
    assert!((geom["surface_area"].as_f64().unwrap() - 2200.0).abs() < 1e-3);
    assert_eq!(geom["faces_count"], 12);
    assert_eq!(geom["vertices_count"], 24);

    let dims = geom["bounding_box"]["dimensions"].as_array().unwrap();
    assert!((dims[0].as_f64().unwrap() - 10.0).abs() < 1e-4);
    assert!((dims[1].as_f64().unwrap() - 20.0).abs() < 1e-4);
    assert!((dims[2].as_f64().unwrap() - 30.0).abs() < 1e-4);

    let planar_faces = geom["detected_planar_faces"].as_array().unwrap();
    assert_eq!(planar_faces.len(), 6, "Box should have 6 detected planar faces");
}

#[tokio::test]
async fn test_mcp_boolean_union_and_intersection() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create target and tool parts
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 810,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "box_a",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let part_a = serde_json::from_str::<Value>(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"].as_str().unwrap().to_string();

    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 811,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "box_b",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let part_b = serde_json::from_str::<Value>(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"].as_str().unwrap().to_string();

    // Add box to Part A: 10 x 10 x 10 at [0, 0, 0]
    rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 812,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_a,
                "primitive_type": "box",
                "width": 10.0,
                "height": 10.0,
                "depth": 10.0,
                "position": [0.0, 0.0, 0.0]
            }
        }
    })).await;

    // Add box to Part B: 10 x 10 x 10 at [5, 0, 0]
    rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 813,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_b,
                "primitive_type": "box",
                "width": 10.0,
                "height": 10.0,
                "depth": 10.0,
                "position": [5.0, 0.0, 0.0]
            }
        }
    })).await;

    // Test boolean_union
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 814,
        "method": "tools/call",
        "params": {
            "name": "boolean_union",
            "arguments": {
                "target_part_id": part_a,
                "tool_part_id": part_b
            }
        }
    })).await;
    assert_eq!(resp["id"], 814);
    assert!(resp["error"].is_null());
    let union_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(union_data["status"], "union_applied");
    assert!(union_data["triangles_count"].as_u64().unwrap() > 0);

    // Test boolean_intersection
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 815,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "box_c",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let part_c = serde_json::from_str::<Value>(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"].as_str().unwrap().to_string();

    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 816,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "box_d",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let part_d = serde_json::from_str::<Value>(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"].as_str().unwrap().to_string();

    rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 817,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_c,
                "primitive_type": "box",
                "width": 10.0,
                "height": 10.0,
                "depth": 10.0,
                "position": [0.0, 0.0, 0.0]
            }
        }
    })).await;

    rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 818,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_d,
                "primitive_type": "box",
                "width": 10.0,
                "height": 10.0,
                "depth": 10.0,
                "position": [5.0, 0.0, 0.0]
            }
        }
    })).await;

    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 819,
        "method": "tools/call",
        "params": {
            "name": "boolean_intersection",
            "arguments": {
                "target_part_id": part_c,
                "tool_part_id": part_d
            }
        }
    })).await;
    assert_eq!(resp["id"], 819);
    assert!(resp["error"].is_null());
    let inter_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(inter_data["status"], "intersection_applied");
    assert!(inter_data["triangles_count"].as_u64().unwrap() > 0);

    // Test error: target == tool
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 820,
        "method": "tools/call",
        "params": {
            "name": "boolean_union",
            "arguments": {
                "target_part_id": part_c,
                "tool_part_id": part_c
            }
        }
    })).await;
    assert_tool_error(&resp);
}

#[tokio::test]
async fn test_mcp_transform_part() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create part & box
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 830,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "transform_target",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let part_id = serde_json::from_str::<Value>(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"].as_str().unwrap().to_string();

    rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 831,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_id,
                "primitive_type": "box",
                "width": 2.0,
                "height": 4.0,
                "depth": 6.0
            }
        }
    })).await;

    // Original volume = 48
    // Transform: scale 2x on X, rotate 90 deg around Z, translate [10, 20, 30]
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 832,
        "method": "tools/call",
        "params": {
            "name": "transform_part",
            "arguments": {
                "part_id": part_id,
                "scale": [2.0, 1.0, 1.0],
                "rotation_deg": [0.0, 0.0, 90.0],
                "translation": [10.0, 20.0, 30.0]
            }
        }
    })).await;
    assert_eq!(resp["id"], 832);
    assert!(resp["error"].is_null());
    let trans_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(trans_data["status"], "transformed");
    // Volume after 2x scale: 48 * 2 = 96
    assert!((trans_data["volume"].as_f64().unwrap() - 96.0).abs() < 1e-3);

    let min = trans_data["bounding_box"]["min"].as_array().unwrap();
    let max = trans_data["bounding_box"]["max"].as_array().unwrap();
    let center_x = (min[0].as_f64().unwrap() + max[0].as_f64().unwrap()) / 2.0;
    let center_y = (min[1].as_f64().unwrap() + max[1].as_f64().unwrap()) / 2.0;
    let center_z = (min[2].as_f64().unwrap() + max[2].as_f64().unwrap()) / 2.0;
    assert!((center_x - 10.0).abs() < 1e-3);
    assert!((center_y - 20.0).abs() < 1e-3);
    assert!((center_z - 30.0).abs() < 1e-3);
}

#[tokio::test]
async fn test_mcp_define_material() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Register new custom material
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 840,
        "method": "tools/call",
        "params": {
            "name": "define_material",
            "arguments": {
                "id": "carbon_fiber_t700",
                "name": "Toray T700 Carbon Fiber",
                "young_modulus_gpa": 230.0,
                "poisson_ratio": 0.30,
                "yield_strength_mpa": 2500.0,
                "density_kg_m3": 1780.0
            }
        }
    })).await;
    assert_eq!(resp["id"], 840);
    assert!(resp["error"].is_null());
    let mat_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(mat_data["status"], "material_registered");
    assert_eq!(mat_data["material_id"], "carbon_fiber_t700");
    assert!((mat_data["properties"]["young_modulus_pa"].as_f64().unwrap() - 230e9).abs() < 1.0);
    assert!((mat_data["properties"]["yield_strength_pa"].as_f64().unwrap() - 2500e6).abs() < 1.0);

    // 2. Create part using newly defined material
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 841,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "cf_bracket",
                "material": "carbon_fiber_t700"
            }
        }
    })).await;
    assert_eq!(resp["id"], 841);
    assert!(resp["error"].is_null());

    // 3. Validation: negative young's modulus
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 842,
        "method": "tools/call",
        "params": {
            "name": "define_material",
            "arguments": {
                "id": "bad_mat",
                "young_modulus_gpa": -50.0,
                "poisson_ratio": 0.3,
                "yield_strength_mpa": 100.0,
                "density_kg_m3": 1000.0
            }
        }
    })).await;
    assert_tool_error(&resp);

    // 4. Validation: invalid Poisson ratio (>= 0.5)
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 843,
        "method": "tools/call",
        "params": {
            "name": "define_material",
            "arguments": {
                "id": "bad_nu",
                "young_modulus_gpa": 100.0,
                "poisson_ratio": 0.55,
                "yield_strength_mpa": 100.0,
                "density_kg_m3": 1000.0
            }
        }
    })).await;
    assert_tool_error(&resp);
}

#[tokio::test]
async fn test_mcp_batch_cad_operations() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Successful batch of 3 operations
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 850,
        "method": "tools/call",
        "params": {
            "name": "batch_cad_operations",
            "arguments": {
                "operations": [
                    {
                        "tool": "create_part",
                        "params": {
                            "part_id": "batch_part_1",
                            "name": "batch_part_1",
                            "material": "aluminum_6061_t6"
                        }
                    },
                    {
                        "tool": "add_feature_primitive",
                        "params": {
                            "part_id": "batch_part_1",
                            "primitive_type": "box",
                            "width": 5.0,
                            "height": 5.0,
                            "depth": 5.0
                        }
                    },
                    {
                        "tool": "query_part_geometry",
                        "params": {
                            "part_id": "batch_part_1"
                        }
                    }
                ]
            }
        }
    })).await;
    assert_eq!(resp["id"], 850);
    assert!(resp["error"].is_null());
    let batch_data: Value = serde_json::from_str(resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(batch_data["success"], true);
    assert_eq!(batch_data["count"], 3);
    let results = batch_data["results"].as_array().unwrap();
    assert_eq!(results.len(), 3);
    assert!(results[1]["feature_id"].is_string());
    assert!((results[2]["volume"].as_f64().unwrap() - 125.0).abs() < 1e-3);

    // 2. Nested batch recursion prevention
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 851,
        "method": "tools/call",
        "params": {
            "name": "batch_cad_operations",
            "arguments": {
                "operations": [
                    {
                        "tool": "batch_cad_operations",
                        "params": { "operations": [] }
                    }
                ]
            }
        }
    })).await;
    let err_msg = assert_tool_error(&resp);
    assert!(err_msg.contains("Nested 'batch_cad_operations' is not permitted"));

    // 3. Operation failure reports step index
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 852,
        "method": "tools/call",
        "params": {
            "name": "batch_cad_operations",
            "arguments": {
                "operations": [
                    {
                        "tool": "create_part",
                        "params": {
                            "name": "valid_part",
                            "material": "steel_structural_aisi1020"
                        }
                    },
                    {
                        "tool": "query_part_geometry",
                        "params": { "part_id": "non_existent_part" }
                    }
                ]
            }
        }
    })).await;
    let err_msg = assert_tool_error(&resp);
    assert!(err_msg.contains("Batch operation at index 1"));
}

#[tokio::test]
async fn test_mcp_validate_mesh_and_debug_viewport_modes() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create part
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 901,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Diagnostic Box",
                "material": "aluminum_6061_t6"
            }
        }
    })).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let data: Value = serde_json::from_str(text).unwrap();
    let part_id = data["part_id"].as_str().unwrap().to_string();

    // 2. Add box primitive
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 902,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_id,
                "primitive_type": "box",
                "params": {
                    "width": 200.0,
                    "height": 200.0,
                    "depth": 200.0
                }
            }
        }
    })).await;
    assert_eq!(resp["id"], 902);

    // 3. Test validate_mesh tool call
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 903,
        "method": "tools/call",
        "params": {
            "name": "validate_mesh",
            "arguments": {
                "part_id": part_id,
                "expected_genus": 0
            }
        }
    })).await;
    assert_eq!(resp["id"], 903);
    let val_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let val: Value = serde_json::from_str(val_text).unwrap();
    assert_eq!(val["status"], "PASS");
    assert_eq!(val["topology"]["is_watertight"], true);
    assert_eq!(val["topology"]["orientation_consistent"], true);
    assert_eq!(val["geometry"]["is_positive_volume"], true);
    assert_eq!(val["geometry"]["degenerate_triangles_count"], 0);

    // 4. Test query_part_geometry contains mesh_validation
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 904,
        "method": "tools/call",
        "params": {
            "name": "query_part_geometry",
            "arguments": {
                "part_id": part_id
            }
        }
    })).await;
    let q_text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let q_data: Value = serde_json::from_str(q_text).unwrap();
    assert!(q_data["mesh_validation"].is_object());
    assert_eq!(q_data["mesh_validation"]["status"], "PASS");

    // 5. Test capture_viewport with all debug_mode options
    let modes = ["wireframe", "normals", "backfaces", "boundary_edges", "flat_shading"];
    for (idx, mode) in modes.iter().enumerate() {
        let resp = rpc_call(&state, json!({
            "jsonrpc": "2.0",
            "id": 910 + idx,
            "method": "tools/call",
            "params": {
                "name": "capture_viewport",
                "arguments": {
                    "part_id": part_id,
                    "camera_preset": "isometric",
                    "debug_mode": mode,
                    "width": 320,
                    "height": 240
                }
            }
        })).await;

        let v_text = resp["result"]["content"][0]["text"].as_str().unwrap();
        let v_data: Value = serde_json::from_str(v_text).unwrap();
        let b64 = v_data["image_base64"].as_str().unwrap();
        assert!(!b64.is_empty(), "Base64 image for mode {} must not be empty", mode);
        let bytes = base64::prelude::BASE64_STANDARD.decode(b64).unwrap();
        assert_eq!(&bytes[0..8], &PNG_MAGIC, "PNG signature mismatch for mode {}", mode);
    }
}

#[tokio::test]
async fn test_mcp_optimize_feature_tree_cbo() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create a coaxial bushing part (Cylinder + coaxial through CutHole + disjoint CutHole + sequential Transforms)
    let resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 950,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Precision Coaxial Bushing",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let text = resp["result"]["content"][0]["text"].as_str().unwrap();
    let part_id = serde_json::from_str::<Value>(text).unwrap()["part_id"].as_str().unwrap().to_string();

    // 2. Add cylinder primitive
    rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 951,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_id,
                "primitive_type": "cylinder",
                "params": {
                    "radius": 40.0,
                    "height": 60.0,
                    "segments": 32
                }
            }
        }
    })).await;

    // 3. Cut coaxial central bore
    rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 952,
        "method": "tools/call",
        "params": {
            "name": "cut_hole",
            "arguments": {
                "part_id": part_id,
                "position": [0.0, 0.0, 0.0],
                "normal": [0.0, 0.0, 1.0],
                "radius": 20.0,
                "depth": 80.0
            }
        }
    })).await;

    // 4. Cut disjoint hole outside AABB (should be culled by R2)
    rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 953,
        "method": "tools/call",
        "params": {
            "name": "cut_hole",
            "arguments": {
                "part_id": part_id,
                "position": [2000.0, 2000.0, 0.0],
                "normal": [0.0, 0.0, 1.0],
                "radius": 10.0,
                "depth": 50.0
            }
        }
    })).await;

    // 5. Call optimize_feature_tree with apply: true
    let opt_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 954,
        "method": "tools/call",
        "params": {
            "name": "optimize_feature_tree",
            "arguments": {
                "part_id": part_id,
                "apply": true
            }
        }
    })).await;
    assert_eq!(opt_resp["id"], 954);
    let opt_text = opt_resp["result"]["content"][0]["text"].as_str().unwrap();
    let opt_data: Value = serde_json::from_str(opt_text).unwrap();

    assert_eq!(opt_data["status"], "optimized");
    assert_eq!(opt_data["applied"], true);
    assert_eq!(opt_data["logical_nodes_count"], 3);
    assert_eq!(opt_data["physical_nodes_count"], 1);
    assert!(opt_data["cost_reduction_pct"].as_f64().unwrap() > 0.0);
    assert!(opt_data["explain_analyze_text"].as_str().unwrap().contains("EXPLAIN (ANALYZE, COSTS, TOPOLOGY)"));
    assert_eq!(opt_data["equivalence_verification"]["passed"], true);
    assert!(opt_data["equivalence_verification"]["volume_delta_pct"].as_f64().unwrap() < 0.5);

    let rules = opt_data["applied_rules"].as_array().unwrap();
    assert!(rules.iter().any(|r| r.as_str() == Some("R2_AABB_DEAD_OP_CULLING")));
    assert!(rules.iter().any(|r| r.as_str() == Some("R4_COAXIAL_REVOLVE_REDUCTION")));
}

#[tokio::test]
async fn test_bug015_create_joint_and_create_part_schema_contract() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();
    let list_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 960,
        "method": "tools/list",
        "params": {}
    })).await;

    let tools = list_resp["result"]["tools"].as_array().unwrap();
    assert_eq!(tools.len(), 34, "tools/list must expose all 34 MCP tools");

    let create_part_tool = tools.iter().find(|t| t["name"] == "create_part").unwrap();
    assert!(
        create_part_tool["inputSchema"]["properties"]["is_fixed"].is_object(),
        "create_part inputSchema must document optional 'is_fixed' boolean property"
    );

    let create_joint_tool = tools.iter().find(|t| t["name"] == "create_joint").unwrap();
    let joint_enums = create_joint_tool["inputSchema"]["properties"]["joint_type"]["enum"]
        .as_array()
        .unwrap();
    assert!(
        joint_enums.iter().any(|e| e.as_str() == Some("gear")),
        "create_joint joint_type enum must include 'gear'"
    );
    assert!(create_joint_tool["inputSchema"]["properties"]["ratio"].is_object());
    assert!(create_joint_tool["inputSchema"]["properties"]["teeth_a"].is_object());
    assert!(create_joint_tool["inputSchema"]["properties"]["teeth_b"].is_object());

    // Verify revolute joint works with default anchor_a / anchor_b when omitted
    let p_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 961,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Rotor",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let p_id = serde_json::from_str::<Value>(p_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    // Add a cylinder primitive to give Rotor valid physical geometry
    let prim_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 9611,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": p_id,
                "primitive_type": "cylinder",
                "params": {
                    "radius": 50.0,
                    "height": 200.0
                }
            }
        }
    })).await;
    assert_ne!(
        prim_resp["result"]["isError"], true,
        "Adding primitive to Rotor must succeed: {:?}",
        prim_resp["result"]["content"]
    );

    let j_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 962,
        "method": "tools/call",
        "params": {
            "name": "create_joint",
            "arguments": {
                "joint_type": "revolute",
                "part_a": "ground",
                "part_b": p_id,
                "axis": [0.0, 0.0, 1.0]
            }
        }
    })).await;
    assert!(j_resp.get("error").is_none(), "create_joint should not produce JSON-RPC error");
    assert_ne!(
        j_resp["result"]["isError"], true,
        "create_joint should succeed and default anchor_a/anchor_b to [0,0,0]: {:?}",
        j_resp["result"]["content"]
    );
    let j_text = j_resp["result"]["content"][0]["text"].as_str().unwrap();
    let j_val: Value = serde_json::from_str(j_text).unwrap();
    let joint_id = j_val["joint_id"].as_str().expect("create_joint must return joint_id");
    assert!(joint_id.starts_with("joint-"), "joint_id must start with 'joint-' prefix");

    // Verify creating a joint on a part without geometry returns a clear tool execution error
    let empty_p_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 963,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "EmptyRotor",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let empty_p_id = serde_json::from_str::<Value>(empty_p_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    let invalid_j_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 964,
        "method": "tools/call",
        "params": {
            "name": "create_joint",
            "arguments": {
                "joint_type": "revolute",
                "part_a": "ground",
                "part_b": empty_p_id,
                "axis": [0.0, 0.0, 1.0]
            }
        }
    })).await;
    let err_msg = assert_tool_error(&invalid_j_resp);
    assert!(
        err_msg.contains("no geometry") || err_msg.contains("empty"),
        "Error message must indicate that part has no geometry: {err_msg}"
    );

    // Verify creating a joint connecting part to itself returns tool execution error
    let self_j_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 965,
        "method": "tools/call",
        "params": {
            "name": "create_joint",
            "arguments": {
                "joint_type": "revolute",
                "part_a": p_id,
                "part_b": p_id,
                "axis": [0.0, 0.0, 1.0]
            }
        }
    })).await;
    assert_tool_error(&self_j_resp);
}

#[tokio::test]
async fn test_bug016_revolve_boundary_and_self_intersection_validation() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();
    let p_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 970,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "RevolveValidationPart",
                "material": "aluminum_6061_t6"
            }
        }
    })).await;
    let part_id = serde_json::from_str::<Value>(p_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    // 1. Negative r coordinate (crossing rotation axis) must be rejected
    let neg_r_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 971,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_id,
                "primitive_type": "revolve",
                "params": {
                    "profile": [[-10.0, -20.0], [30.0, -20.0], [30.0, 20.0], [-10.0, 20.0]],
                    "angle_deg": 360.0
                }
            }
        }
    })).await;
    let neg_r_err = assert_tool_error(&neg_r_resp);
    assert!(
        neg_r_err.contains("cannot be negative"),
        "Revolve with r < 0 must explain negative r violation, got: {neg_r_err}"
    );

    // 2. Out-of-range angle_deg = 720 must be rejected
    let bad_angle_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 972,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_id,
                "primitive_type": "revolve",
                "params": {
                    "profile": [[10.0, -20.0], [30.0, -20.0], [30.0, 20.0], [10.0, 20.0]],
                    "angle_deg": 720.0
                }
            }
        }
    })).await;
    let bad_angle_err = assert_tool_error(&bad_angle_resp);
    assert!(
        bad_angle_err.contains("angle_deg"),
        "Revolve with angle_deg > 360 must explain angle_deg violation, got: {bad_angle_err}"
    );

    // 3. Self-intersecting figure-8 profile must be rejected
    let self_int_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 973,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": part_id,
                "primitive_type": "revolve",
                "params": {
                    "profile": [[10.0, -20.0], [40.0, 20.0], [10.0, 20.0], [40.0, -20.0]],
                    "angle_deg": 360.0
                }
            }
        }
    })).await;
    let self_int_err = assert_tool_error(&self_int_resp);
    assert!(
        self_int_err.contains("self-intersecting"),
        "Self-intersecting revolve profile must explain self-intersection, got: {self_int_err}"
    );
}

#[tokio::test]
async fn test_units_equivalence_mm_vs_m() {
    // Part 1: Created in default millimeters (200 x 20 x 20 mm cantilever beam with R=5 mm hole)
    let state_mm = EngineState::new().unwrap().into_shared();

    let p_mm_resp = rpc_call(&state_mm, json!({
        "jsonrpc": "2.0",
        "id": 1001,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Beam_MM",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let p_mm_data: Value = serde_json::from_str(p_mm_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(p_mm_data["length_unit"], "mm");
    assert_eq!(p_mm_data["units"]["length"], "mm");
    let pid_mm = p_mm_data["part_id"].as_str().unwrap().to_string();

    let add_mm_resp = rpc_call(&state_mm, json!({
        "jsonrpc": "2.0",
        "id": 1002,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": pid_mm,
                "primitive_type": "box",
                "width": 200.0,
                "height": 20.0,
                "depth": 20.0
            }
        }
    })).await;
    let add_mm_data: Value = serde_json::from_str(add_mm_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(add_mm_data["warnings"].as_array().unwrap().len(), 0);

    rpc_call(&state_mm, json!({
        "jsonrpc": "2.0",
        "id": 1003,
        "method": "tools/call",
        "params": {
            "name": "cut_hole",
            "arguments": {
                "part_id": pid_mm,
                "position": [0.0, 0.0, 0.0],
                "normal": [0.0, 0.0, 1.0],
                "radius": 5.0,
                "depth": 30.0
            }
        }
    })).await;

    let q_mm_resp = rpc_call(&state_mm, json!({
        "jsonrpc": "2.0",
        "id": 1004,
        "method": "tools/call",
        "params": {
            "name": "query_part_geometry",
            "arguments": { "part_id": pid_mm }
        }
    })).await;
    let q_mm: Value = serde_json::from_str(q_mm_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(q_mm["length_unit"], "mm");
    assert_eq!(q_mm["units"]["volume"], "mm³");

    let fea_mm_resp = rpc_call(&state_mm, json!({
        "jsonrpc": "2.0",
        "id": 1005,
        "method": "tools/call",
        "params": {
            "name": "run_fea_analysis",
            "arguments": {
                "part_id": pid_mm,
                "mesh_density": "medium",
                "fixed_faces": ["min_x"],
                "applied_forces": [{
                    "position": [100.0, 0.0, 0.0],
                    "force_vector": [0.0, -500.0, 0.0]
                }]
            }
        }
    })).await;
    let fea_mm: Value = serde_json::from_str(fea_mm_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();

    // Part 2: Created in meters via set_units ("m") (0.2 x 0.02 x 0.02 m cantilever beam with R=0.005 m hole)
    let state_m = EngineState::new().unwrap().into_shared();
    let set_u_resp = rpc_call(&state_m, json!({
        "jsonrpc": "2.0",
        "id": 2000,
        "method": "tools/call",
        "params": {
            "name": "set_units",
            "arguments": { "length_unit": "m" }
        }
    })).await;
    let set_u_data: Value = serde_json::from_str(set_u_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(set_u_data["status"], "units_updated");
    assert_eq!(set_u_data["previous_length_unit"], "mm");
    assert_eq!(set_u_data["length_unit"], "m");
    assert_eq!(set_u_data["units"]["length"], "m");

    let p_m_resp = rpc_call(&state_m, json!({
        "jsonrpc": "2.0",
        "id": 2001,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": {
                "name": "Beam_M",
                "material": "steel_structural_aisi1020"
            }
        }
    })).await;
    let p_m_data: Value = serde_json::from_str(p_m_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(p_m_data["length_unit"], "m");
    let pid_m = p_m_data["part_id"].as_str().unwrap().to_string();

    rpc_call(&state_m, json!({
        "jsonrpc": "2.0",
        "id": 2002,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": pid_m,
                "primitive_type": "box",
                "width": 0.2,
                "height": 0.02,
                "depth": 0.02
            }
        }
    })).await;

    rpc_call(&state_m, json!({
        "jsonrpc": "2.0",
        "id": 2003,
        "method": "tools/call",
        "params": {
            "name": "cut_hole",
            "arguments": {
                "part_id": pid_m,
                "position": [0.0, 0.0, 0.0],
                "normal": [0.0, 0.0, 1.0],
                "radius": 0.005,
                "depth": 0.03
            }
        }
    })).await;

    let q_m_resp = rpc_call(&state_m, json!({
        "jsonrpc": "2.0",
        "id": 2004,
        "method": "tools/call",
        "params": {
            "name": "query_part_geometry",
            "arguments": { "part_id": pid_m }
        }
    })).await;
    let q_m: Value = serde_json::from_str(q_m_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(q_m["length_unit"], "m");
    assert_eq!(q_m["units"]["volume"], "m³");

    let fea_m_resp = rpc_call(&state_m, json!({
        "jsonrpc": "2.0",
        "id": 2005,
        "method": "tools/call",
        "params": {
            "name": "run_fea_analysis",
            "arguments": {
                "part_id": pid_m,
                "mesh_density": "medium",
                "fixed_faces": ["min_x"],
                "applied_forces": [{
                    "position": [0.1, 0.0, 0.0],
                    "force_vector": [0.0, -500.0, 0.0]
                }]
            }
        }
    })).await;
    let fea_m: Value = serde_json::from_str(fea_m_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();

    // Verify physical invariants match within 1e-6 relative error
    let vol_m3_from_mm = q_mm["volume_m3"].as_f64().unwrap();
    let vol_m3_from_m = q_m["volume_m3"].as_f64().unwrap();
    assert!(
        (vol_m3_from_mm - vol_m3_from_m).abs() / vol_m3_from_m < 1e-6,
        "SI volume_m3 must match: {} vs {}",
        vol_m3_from_mm,
        vol_m3_from_m
    );

    let mass_mm = q_mm["mass_kg"].as_f64().unwrap();
    let mass_m = q_m["mass_kg"].as_f64().unwrap();
    assert!(
        (mass_mm - mass_m).abs() / mass_m < 1e-6,
        "Mass in kg must match: {} vs {}",
        mass_mm,
        mass_m
    );

    let vm_mm = fea_mm["max_von_mises_mpa"].as_f64().unwrap();
    let vm_m = fea_m["max_von_mises_mpa"].as_f64().unwrap();
    assert!(
        (vm_mm - vm_m).abs() / vm_m < 1e-6,
        "FEA max_von_mises_mpa must match: {} vs {}",
        vm_mm,
        vm_m
    );

    let disp_mm = fea_mm["max_displacement_mm"].as_f64().unwrap();
    let disp_m = fea_m["max_displacement_mm"].as_f64().unwrap();
    assert!(
        (disp_mm - disp_m).abs() / disp_m < 1e-6,
        "FEA max_displacement_mm must match: {} vs {}",
        disp_mm,
        disp_m
    );

    let sf_mm = fea_mm["safety_factor"].as_f64().unwrap();
    let sf_m = fea_m["safety_factor"].as_f64().unwrap();
    assert!(
        (sf_mm - sf_m).abs() / sf_m < 1e-6,
        "FEA safety_factor must match: {} vs {}",
        sf_mm,
        sf_m
    );
}

#[tokio::test]
async fn test_units_scale_sanity_warnings_20m_and_micro() {
    let state = EngineState::new().unwrap().into_shared();

    // 1. Create a part with a 20-meter dimension (20_000 mm in default "mm" mode) -> SCALE_TOO_LARGE
    let p1_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 3001,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": { "name": "GiantPart", "material": "aluminum_6061_t6" }
        }
    })).await;
    let p1_id = serde_json::from_str::<Value>(p1_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    let giant_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 3002,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": p1_id,
                "primitive_type": "box",
                "width": 20000.0,
                "height": 100.0,
                "depth": 100.0
            }
        }
    })).await;
    let giant_data: Value = serde_json::from_str(giant_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let giant_warnings = giant_data["warnings"].as_array().expect("warnings array must be present");
    assert!(
        giant_warnings.iter().any(|w| w.as_str().unwrap_or("").contains("SCALE_TOO_LARGE")),
        "Expected SCALE_TOO_LARGE warning for 20-meter box, got: {:?}",
        giant_warnings
    );

    // 2. Create a normal 100 mm part and scale it by 200x via transform_part -> 20 meters -> SCALE_TOO_LARGE
    let p2_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 3003,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": { "name": "ScaledGiantPart", "material": "aluminum_6061_t6" }
        }
    })).await;
    let p2_id = serde_json::from_str::<Value>(p2_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 3004,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": p2_id,
                "primitive_type": "box",
                "width": 100.0,
                "height": 100.0,
                "depth": 100.0
            }
        }
    })).await;

    let trans_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 3005,
        "method": "tools/call",
        "params": {
            "name": "transform_part",
            "arguments": {
                "part_id": p2_id,
                "scale": [200.0, 1.0, 1.0]
            }
        }
    })).await;
    let trans_data: Value = serde_json::from_str(trans_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let trans_warnings = trans_data["warnings"].as_array().expect("warnings array must be present on transform_part");
    assert!(
        trans_warnings.iter().any(|w| w.as_str().unwrap_or("").contains("SCALE_TOO_LARGE")),
        "Expected SCALE_TOO_LARGE warning after scaling to 20 meters, got: {:?}",
        trans_warnings
    );

    // 3. Create a part with accidental meter values (0.05 mm < 0.1 mm) in "mm" mode -> SCALE_TOO_SMALL
    let p3_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 3006,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": { "name": "MicroPart", "material": "aluminum_6061_t6" }
        }
    })).await;
    let p3_id = serde_json::from_str::<Value>(p3_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    let micro_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 3007,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": p3_id,
                "primitive_type": "box",
                "width": 0.05,
                "height": 0.05,
                "depth": 0.05
            }
        }
    })).await;
    let micro_data: Value = serde_json::from_str(micro_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let micro_warnings = micro_data["warnings"].as_array().expect("warnings array must be present");
    assert!(
        micro_warnings.iter().any(|w| w.as_str().unwrap_or("").contains("SCALE_TOO_SMALL")),
        "Expected SCALE_TOO_SMALL warning for 0.05 mm box, got: {:?}",
        micro_warnings
    );

    // 4. Create a wheel_rim with rim_diameter < 150 mm (e.g. 145 mm = 0.145 m < 0.15 m) -> WHEEL_RIM_SCALE
    let p4_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 3008,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": { "name": "TinyWheel", "material": "aluminum_6061_t6" }
        }
    })).await;
    let p4_id = serde_json::from_str::<Value>(p4_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    let wheel_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 3009,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": p4_id,
                "primitive_type": "wheel_rim",
                "params": {
                    "rim_diameter": 145.0,
                    "rim_width": 60.0
                }
            }
        }
    })).await;
    let wheel_data: Value = serde_json::from_str(wheel_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
    let wheel_warnings = wheel_data["warnings"].as_array().expect("warnings array must be present");
    assert!(
        wheel_warnings.iter().any(|w| w.as_str().unwrap_or("").contains("WHEEL_RIM_SCALE")),
        "Expected WHEEL_RIM_SCALE warning for 145 mm wheel rim, got: {:?}",
        wheel_warnings
    );
}

#[tokio::test]
async fn test_structured_error_contract_and_part_not_found_hint() {
    let state = EngineState::new().unwrap().into_shared();

    // 1. Initially scene has no parts: PART_NOT_FOUND hint indicates no parts exist
    let resp_empty = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 1001,
        "method": "tools/call",
        "params": {
            "name": "query_part_geometry",
            "arguments": { "part_id": "non_existent_part" }
        }
    })).await;
    let err_text = assert_tool_error(&resp_empty);
    let err_json: Value = serde_json::from_str(err_text).unwrap();
    assert_eq!(err_json["error"]["code"], "PART_NOT_FOUND");
    assert_eq!(err_json["error"]["param"], "part_id");
    assert_eq!(err_json["error"]["retryable"], false);
    assert_eq!(err_json["error"]["details"]["part_id"], "non_existent_part");
    assert_eq!(err_json["error"]["details"]["available_parts"], json!([]));
    assert!(err_json["error"]["hint"].as_str().unwrap().contains("No parts currently exist"));

    // 2. Create parts: 'bracket' and 'motor_mount'
    let p1_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 1002,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": { "name": "bracket", "material": "aluminum_6061_t6" }
        }
    })).await;
    let p1_id = serde_json::from_str::<Value>(p1_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    let p2_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 1003,
        "method": "tools/call",
        "params": {
            "name": "create_part",
            "arguments": { "name": "motor_mount", "material": "steel_structural_aisi1020" }
        }
    })).await;
    let p2_id = serde_json::from_str::<Value>(p2_resp["result"]["content"][0]["text"].as_str().unwrap()).unwrap()["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    // 3. Now PART_NOT_FOUND lists existing parts in hint and details.available_parts
    let resp_with_parts = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 1004,
        "method": "tools/call",
        "params": {
            "name": "query_part_geometry",
            "arguments": { "part_id": "shaft_rod" }
        }
    })).await;
    let err_text = assert_tool_error(&resp_with_parts);
    let err_json: Value = serde_json::from_str(err_text).unwrap();
    assert_eq!(err_json["error"]["code"], "PART_NOT_FOUND");
    assert_eq!(err_json["error"]["param"], "part_id");
    let hint = err_json["error"]["hint"].as_str().unwrap();
    assert!(hint.contains(&p1_id), "Hint must list available parts: {hint}");
    assert!(hint.contains(&p2_id), "Hint must list available parts: {hint}");
    let avail = err_json["error"]["details"]["available_parts"].as_array().unwrap();
    assert!(avail.iter().any(|p| p == &p1_id));
    assert!(avail.iter().any(|p| p == &p2_id));

    // 4. Invalid primitive_type provides supported_primitives list
    let bad_prim_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 1005,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": "bracket",
                "primitive_type": "hyperboloid",
                "params": {}
            }
        }
    })).await;
    let bad_prim_text = assert_tool_error(&bad_prim_resp);
    let bad_prim_json: Value = serde_json::from_str(bad_prim_text).unwrap();
    assert_eq!(bad_prim_json["error"]["code"], "INVALID_PRIMITIVE_TYPE");
    assert_eq!(bad_prim_json["error"]["param"], "primitive_type");
    let supp = bad_prim_json["error"]["details"]["supported_primitives"].as_array().unwrap();
    assert!(supp.iter().any(|p| p == "box"));
    assert!(supp.iter().any(|p| p == "cylinder"));

    // 5. Missing required argument identifies param
    let missing_arg_resp = rpc_call(&state, json!({
        "jsonrpc": "2.0",
        "id": 1006,
        "method": "tools/call",
        "params": {
            "name": "add_feature_primitive",
            "arguments": {
                "part_id": "bracket",
                "primitive_type": "box",
                "params": { "height": 10.0, "depth": 10.0 }
            }
        }
    })).await;
    let missing_text = assert_tool_error(&missing_arg_resp);
    let missing_json: Value = serde_json::from_str(missing_text).unwrap();
    assert_eq!(missing_json["error"]["code"], "INVALID_ARGUMENT");
    assert_eq!(missing_json["error"]["param"], "width");
}

#[tokio::test]
async fn test_project_save_and_load_mcp_contract() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Verify save_project and load_project are in tools/list
    let list_tools_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1100,
            "method": "tools/list",
            "params": {}
        }),
    )
    .await;
    let tools = list_tools_resp["result"]["tools"].as_array().unwrap();
    assert!(tools.iter().any(|t| t["name"] == "save_project"));
    assert!(tools.iter().any(|t| t["name"] == "load_project"));

    let parse_content = |resp: &Value| -> Value {
        let text = resp["result"]["content"][0]["text"].as_str().expect("content[0].text must be a string");
        serde_json::from_str(text).expect("content[0].text must be valid JSON")
    };

    // 2. Define custom material
    let def_mat_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1101,
            "method": "tools/call",
            "params": {
                "name": "define_material",
                "arguments": {
                    "id": "inconel_718",
                    "name": "Inconel 718 Superalloy",
                    "young_modulus_gpa": 211.0,
                    "poisson_ratio": 0.29,
                    "yield_strength_mpa": 1100.0,
                    "density_kg_m3": 8190.0
                }
            }
        }),
    )
    .await;
    let def_mat_data = parse_content(&def_mat_resp);
    assert_eq!(def_mat_data["status"], "material_registered");

    // 3. Create part A (Base in inconel_718)
    let p_a_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1102,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Base Mount",
                    "material": "inconel_718",
                    "part_id": "mount_base",
                    "is_fixed": true
                }
            }
        }),
    )
    .await;
    assert!(p_a_resp.get("error").is_none());

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1103,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "mount_base",
                    "primitive_type": "box",
                    "params": {
                        "width": 80.0,
                        "height": 15.0,
                        "depth": 40.0
                    }
                }
            }
        }),
    )
    .await;

    // 4. Create part B (Link in aluminum_6061_t6)
    let p_b_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1104,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Link Arm",
                    "material": "aluminum_6061_t6",
                    "part_id": "arm_link",
                    "is_fixed": false
                }
            }
        }),
    )
    .await;
    assert!(p_b_resp.get("error").is_none());

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1105,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "arm_link",
                    "primitive_type": "cylinder",
                    "params": {
                        "radius": 8.0,
                        "height": 50.0
                    }
                }
            }
        }),
    )
    .await;

    // 5. Connect with revolute joint and configure motor
    let joint_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1106,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "part_a": "mount_base",
                    "part_b": "arm_link",
                    "joint_type": "revolute",
                    "anchor_a": [0.0, 15.0, 0.0],
                    "anchor_b": [0.0, 0.0, 0.0],
                    "axis": [0.0, 1.0, 0.0]
                }
            }
        }),
    )
    .await;
    let joint_data = parse_content(&joint_resp);
    let joint_id = joint_data["joint_id"].as_str().unwrap().to_string();

    let motor_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1107,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": joint_id,
                    "target_velocity": 12.5,
                    "max_force": 120.0
                }
            }
        }),
    )
    .await;
    let motor_data = parse_content(&motor_resp);
    assert_eq!(motor_data["status"], "motor_active");

    let temp_dir = std::env::temp_dir().join(format!("mcpcad_mcp_test_{}", uuid::Uuid::new_v4()));
    let project_path = temp_dir.join("mcp_project.json");
    let path_str = project_path.to_str().unwrap();

    // 6. Save project via MCP
    let save_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1108,
            "method": "tools/call",
            "params": {
                "name": "save_project",
                "arguments": {
                    "path": path_str,
                    "project_name": "MCP Assembly Test"
                }
            }
        }),
    )
    .await;
    let save_data = parse_content(&save_resp);
    assert_eq!(save_data["status"], "saved");
    assert_eq!(save_data["parts_count"], 2);
    assert_eq!(save_data["joints_count"], 1);
    assert!(save_data["materials_count"].as_u64().unwrap() >= 1);

    // 7. Verify fresh session loads cleanly
    let fresh_state = EngineState::new()
        .expect("Fresh state init")
        .into_shared();

    let load_resp = rpc_call(
        &fresh_state,
        json!({
            "jsonrpc": "2.0",
            "id": 1109,
            "method": "tools/call",
            "params": {
                "name": "load_project",
                "arguments": {
                    "path": path_str
                }
            }
        }),
    )
    .await;
    let load_data = parse_content(&load_resp);
    assert_eq!(load_data["status"], "loaded");
    assert_eq!(load_data["parts_count"], 2);
    assert_eq!(load_data["joints_count"], 1);

    // 8. Call list_parts on restored state to verify volumes and masses
    let list_parts_resp = rpc_call(
        &fresh_state,
        json!({
            "jsonrpc": "2.0",
            "id": 1110,
            "method": "tools/call",
            "params": {
                "name": "list_parts",
                "arguments": {}
            }
        }),
    )
    .await;
    let list_parts_data = parse_content(&list_parts_resp);
    let loaded_parts = list_parts_data["parts"].as_array().unwrap();
    assert_eq!(loaded_parts.len(), 2);

    let mount = loaded_parts.iter().find(|p| p["name"] == "Base Mount").unwrap();
    assert_eq!(mount["material"], "inconel_718");
    let arm = loaded_parts.iter().find(|p| p["name"] == "Link Arm").unwrap();
    assert_eq!(arm["material"], "aluminum_6061_t6");

    // 9. Negative test: Corrupted file
    let corrupt_path = temp_dir.join("corrupt.json");
    std::fs::write(&corrupt_path, "not a valid json").unwrap();

    let corrupt_resp = rpc_call(
        &fresh_state,
        json!({
            "jsonrpc": "2.0",
            "id": 1111,
            "method": "tools/call",
            "params": {
                "name": "load_project",
                "arguments": {
                    "path": corrupt_path.to_str().unwrap()
                }
            }
        }),
    )
    .await;
    let corrupt_text = assert_tool_error(&corrupt_resp);
    let corrupt_err: Value = serde_json::from_str(corrupt_text).unwrap();
    assert_eq!(corrupt_err["error"]["code"], "JSON_ERROR");

    // Clean up
    let _ = std::fs::remove_dir_all(temp_dir);
}

#[tokio::test]
async fn test_mcp_p2_t6_named_faces_query_and_selector() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create a cylinder part in default mm units (radius = 15 mm, height = 40 mm)
    let create_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1201,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Cyl_Pin",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;
    let part_id = parse_content(&create_resp)["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    let add_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1202,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": part_id,
                    "primitive_type": "cylinder",
                    "params": {
                        "radius": 15.0,
                        "height": 40.0,
                        "segments": 64
                    }
                }
            }
        }),
    )
    .await;
    let feat_id = parse_content(&add_resp)["feature_id"]
        .as_str()
        .unwrap()
        .to_string();

    // 2. Query part geometry in mm
    let query_mm_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1203,
            "method": "tools/call",
            "params": {
                "name": "query_part_geometry",
                "arguments": {
                    "part_id": part_id
                }
            }
        }),
    )
    .await;
    let q_mm = parse_content(&query_mm_resp);
    let faces_mm = q_mm["faces"].as_array().expect("faces array must be present");
    assert_eq!(faces_mm.len(), 3, "Cylinder must have 3 topological faces");

    let side_mm = faces_mm
        .iter()
        .find(|f| f["role"] == "side")
        .expect("side face must exist");
    assert_eq!(side_mm["kind"], "cylinder");
    assert_eq!(side_mm["origin_feature"], feat_id);
    assert!((side_mm["radius"].as_f64().unwrap() - 15.0).abs() < 1e-6);
    assert!(
        side_mm["area"].as_f64().unwrap() > side_mm["area_m2"].as_f64().unwrap() * 1e5,
        "area in mm^2 must be 1e6 * area_m2"
    );

    let top_mm = faces_mm
        .iter()
        .find(|f| f["role"] == "cap_top")
        .expect("cap_top face must exist");
    assert_eq!(top_mm["kind"], "plane");
    assert!((top_mm["centroid"][2].as_f64().unwrap() - 20.0).abs() < 1e-6);

    // 3. Query with length_unit = "m" override
    let query_m_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1204,
            "method": "tools/call",
            "params": {
                "name": "query_part_geometry",
                "arguments": {
                    "part_id": part_id,
                    "length_unit": "m",
                    "face_selector": {
                        "where": {
                            "kind": "cylindrical",
                            "radius": 0.015,
                            "tolerance": 0.001
                        }
                    }
                }
            }
        }),
    )
    .await;
    let q_m = parse_content(&query_m_resp);
    let faces_m = q_m["faces"].as_array().unwrap();
    assert_eq!(faces_m.len(), 1);
    assert_eq!(faces_m[0]["role"], "side");
    assert!((faces_m[0]["radius"].as_f64().unwrap() - 0.015).abs() < 1e-9);

    // 4. Query with unknown face name returns structured FACE_NOT_FOUND
    let err_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1205,
            "method": "tools/call",
            "params": {
                "name": "query_part_geometry",
                "arguments": {
                    "part_id": part_id,
                    "face_selector": "non_existent_face"
                }
            }
        }),
    )
    .await;
    let err_text = assert_tool_error(&err_resp);
    let err_json: Value = serde_json::from_str(err_text).unwrap();
    assert_eq!(err_json["error"]["code"], "FACE_NOT_FOUND");
}

#[tokio::test]
async fn test_mcp_p2_t7_csg_and_cut_hole_named_faces() {
    let state = EngineState::new().unwrap().into_shared();

    // 1. Create a plate box (100x80x20 mm) and cut a cylindrical through-hole (r=15 mm)
    let create_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1301,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "PerfPlate",
                    "material": "aluminum_6061_t6"
                }
            }
        }),
    )
    .await;
    let part_id = parse_content(&create_resp)["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1302,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": part_id,
                    "primitive_type": "box",
                    "params": {
                        "width": 100.0,
                        "height": 80.0,
                        "depth": 20.0
                    }
                }
            }
        }),
    )
    .await;

    let hole_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1303,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": part_id,
                    "position": [0.0, 0.0, -15.0],
                    "normal": [0.0, 0.0, 1.0],
                    "radius": 15.0,
                    "depth": 30.0
                }
            }
        }),
    )
    .await;
    let hole_feat_id = parse_content(&hole_resp)["feature_id"]
        .as_str()
        .unwrap()
        .to_string();

    // 2. Query hole_wall:* via wildcard selector
    let query_hole_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1304,
            "method": "tools/call",
            "params": {
                "name": "query_part_geometry",
                "arguments": {
                    "part_id": part_id,
                    "face_selector": "hole_wall:*"
                }
            }
        }),
    )
    .await;
    let q_hole = parse_content(&query_hole_resp);
    let hole_faces = q_hole["faces"].as_array().unwrap();
    assert_eq!(hole_faces.len(), 1, "Must find exactly 1 hole_wall face");
    assert_eq!(hole_faces[0]["name"], "hole_wall:feat_1");
    assert_eq!(hole_faces[0]["kind"], "cylinder");
    assert_eq!(hole_faces[0]["origin_feature"], hole_feat_id);
    assert!((hole_faces[0]["radius"].as_f64().unwrap() - 15.0).abs() < 1e-6);

    let expected_wall_area_mm2 = 2.0 * std::f64::consts::PI * 15.0 * 20.0;
    let actual_wall_area_mm2 = hole_faces[0]["area"].as_f64().unwrap();
    assert!(
        ((actual_wall_area_mm2 - expected_wall_area_mm2) / expected_wall_area_mm2).abs() < 0.01,
        "Hole wall area {actual_wall_area_mm2} mm^2 must match 2*pi*r*h = {expected_wall_area_mm2} mm^2 within 1%"
    );

    // 3. Verify all 6 original box faces survive and top/bottom lost pi*r^2
    let query_all_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1305,
            "method": "tools/call",
            "params": {
                "name": "query_part_geometry",
                "arguments": {
                    "part_id": part_id
                }
            }
        }),
    )
    .await;
    let all_faces = parse_content(&query_all_resp)["faces"]
        .as_array()
        .unwrap()
        .clone();
    assert_eq!(
        all_faces.len(),
        7,
        "Box minus through-hole must have 6 box faces + 1 hole_wall face"
    );
    let top_face = all_faces
        .iter()
        .find(|f| f["role"] == "top")
        .expect("top face must survive");
    let expected_top_area_mm2 = 100.0 * 80.0 - std::f64::consts::PI * 15.0 * 15.0;
    let actual_top_area_mm2 = top_face["area"].as_f64().unwrap();
    assert!(
        ((actual_top_area_mm2 - expected_top_area_mm2) / expected_top_area_mm2).abs() < 0.01,
        "Top face area {actual_top_area_mm2} mm^2 must match W*H - pi*r^2 = {expected_top_area_mm2} mm^2 within 1%"
    );

    // 4. Test transform_part with align_face (shifts bottom face to z = 0)
    let transform_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1306,
            "method": "tools/call",
            "params": {
                "name": "transform_part",
                "arguments": {
                    "part_id": part_id,
                    "align_face": {
                        "face": "bottom",
                        "target_point": [0.0, 0.0, 0.0]
                    }
                }
            }
        }),
    )
    .await;
    let t_json = parse_content(&transform_resp);
    assert_eq!(t_json["status"], "transformed");
    let bbox_min_z = t_json["bounding_box"]["min"][2].as_f64().unwrap();
    assert!(
        bbox_min_z.abs() < 1e-4,
        "Bottom face aligned to z=0 should make bbox.min.z = 0 (got {bbox_min_z})"
    );
}

#[tokio::test]
async fn test_mcp_p2_t8_fea_named_faces_fixed_and_loads() {
    let state = EngineState::new().unwrap().into_shared();

    // 1. Cylindrical shaft: fix cap_bottom, apply axial force on cap_top
    let create_cyl = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1401,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "P2T8_Shaft",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;
    let cyl_id = parse_content(&create_cyl)["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1402,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": cyl_id,
                    "primitive_type": "cylinder",
                    "params": {
                        "radius": 25.0,
                        "height": 250.0,
                        "segments": 48
                    }
                }
            }
        }),
    )
    .await;

    let fea_cyl_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1403,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": cyl_id,
                    "mesh_density": "medium",
                    "fixed": [{"face": "cap_bottom"}],
                    "loads": [{"face": "cap_top", "force": [0.0, 0.0, 50000.0]}]
                }
            }
        }),
    )
    .await;
    let fea_cyl = parse_content(&fea_cyl_resp);
    assert_eq!(fea_cyl["status"], "PASS");
    let disp_mm = fea_cyl["max_displacement_mm"].as_f64().unwrap();
    // Analytical delta = F * L / (E * A) = 50000 * 0.25 / (205e9 * pi * 0.025^2) = 0.031055 mm
    let expected_disp_mm = 0.031055;
    let disp_rel_err = (disp_mm - expected_disp_mm).abs() / expected_disp_mm;
    assert!(
        disp_rel_err < 0.05,
        "Cylinder axial displacement {disp_mm} mm must match analytical {expected_disp_mm} mm within 5%"
    );

    // 2. Perforated bracket: fix 'left', apply bearing force and pressure on 'hole_wall'
    let create_plate = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1404,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "P2T8_Bracket",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;
    let plate_id = parse_content(&create_plate)["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1405,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": plate_id,
                    "primitive_type": "box",
                    "params": {
                        "width": 100.0,
                        "height": 80.0,
                        "depth": 20.0
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
            "id": 1406,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": plate_id,
                    "position": [0.0, 0.0, -15.0],
                    "normal": [0.0, 0.0, 1.0],
                    "radius": 15.0,
                    "depth": 30.0
                }
            }
        }),
    )
    .await;

    let fea_hole_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1407,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": plate_id,
                    "mesh_density": "coarse",
                    "fixed": [{"face": "left"}],
                    "loads": [
                        {"face": "hole_wall", "force": [10000.0, 0.0, 0.0]},
                        {"where": {"role": "hole_wall"}, "pressure": 15.0}
                    ]
                }
            }
        }),
    )
    .await;
    let fea_hole = parse_content(&fea_hole_resp);
    assert!(fea_hole["max_von_mises_mpa"].as_f64().unwrap() > 0.0);
    assert!(fea_hole["max_displacement_mm"].as_f64().unwrap() > 0.0);

    // 3. Structured error FACE_NOT_FOUND when referencing a non-existent face
    let not_found_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1408,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": plate_id,
                    "fixed": [{"face": "nonexistent_flange_face"}],
                    "loads": [{"face": "hole_wall", "force": [1000.0, 0.0, 0.0]}]
                }
            }
        }),
    )
    .await;
    let nf_env = parse_error_envelope(&not_found_resp);
    assert_eq!(nf_env.error.code, "FACE_NOT_FOUND");
    assert!(
        nf_env.error.details["available_faces"].is_array(),
        "FACE_NOT_FOUND error details must list available_faces"
    );

    // 4. Structured error AMBIGUOUS_FACE_SELECTOR when two holes exist and unqualified 'hole_wall' is used
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1409,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": plate_id,
                    "position": [30.0, 0.0, -15.0],
                    "normal": [0.0, 0.0, 1.0],
                    "radius": 8.0,
                    "depth": 30.0
                }
            }
        }),
    )
    .await;

    let ambig_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1410,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": plate_id,
                    "fixed": [{"face": "left"}],
                    "loads": [{"face": "hole_wall", "force": [1000.0, 0.0, 0.0]}]
                }
            }
        }),
    )
    .await;
    let ambig_env = parse_error_envelope(&ambig_resp);
    assert_eq!(ambig_env.error.code, "AMBIGUOUS_FACE_SELECTOR");
}

#[tokio::test]
async fn test_mcp_p2_t9_requirements_and_auto_check() {
    let state = EngineState::new().unwrap().into_shared();

    // 1. Create a part (aluminum_6061_t6, density = 2700 kg/m^3)
    let create_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1501,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "P2T9_Bracket",
                    "material": "aluminum_6061_t6"
                }
            }
        }),
    )
    .await;
    let create_json = parse_content(&create_resp);
    let part_id = create_json["part_id"].as_str().unwrap().to_string();
    assert_eq!(create_json["revision"], 0);

    // 2. Add a 100 x 50 x 20 mm box -> volume = 1e-4 m^3, mass = 0.27 kg, revision = 1
    let add_box_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1502,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": part_id,
                    "primitive_type": "box",
                    "params": {
                        "width": 100.0,
                        "height": 50.0,
                        "depth": 20.0
                    }
                }
            }
        }),
    )
    .await;
    let add_box_json = parse_content(&add_box_resp);
    let box_feat_id = add_box_json["feature_id"].as_str().unwrap().to_string();
    assert_eq!(add_box_json["revision"], 1);
    assert!(
        add_box_json.get("requirements_status").is_none(),
        "auto_check is false by default so requirements_status should not be present yet"
    );

    // 3. Set part requirements with auto_check = true
    let set_req_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1503,
            "method": "tools/call",
            "params": {
                "name": "set_requirements",
                "arguments": {
                    "part_id": part_id,
                    "auto_check": true,
                    "requirements": [
                        {
                            "id": "req_bbox_max",
                            "type": "bbox_max",
                            "dims": [120.0, 60.0, 25.0]
                        },
                        {
                            "id": "req_bbox_min",
                            "type": "bbox_min",
                            "dims": [80.0, 40.0, 15.0]
                        },
                        {
                            "id": "req_mass",
                            "type": "mass_max",
                            "kg": 0.50
                        },
                        {
                            "id": "req_mesh",
                            "type": "mesh_valid",
                            "expected_genus": 0
                        },
                        {
                            "id": "req_fos",
                            "type": "min_safety_factor",
                            "value": 2.0,
                            "fea_case": "cantilever_tip"
                        }
                    ]
                }
            }
        }),
    )
    .await;
    let set_req_json = parse_content(&set_req_resp);
    assert_eq!(set_req_json["status"], "requirements_set");
    assert_eq!(set_req_json["auto_check"], true);
    assert_eq!(set_req_json["requirements_count"], 5);

    // Also set scene-level requirements (no_interference, mechanism_dof) -> NOT_AVAILABLE
    let set_scene_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1504,
            "method": "tools/call",
            "params": {
                "name": "set_requirements",
                "arguments": {
                    "scope": "scene",
                    "requirements": [
                        {
                            "id": "req_interf",
                            "type": "no_interference",
                            "pairs": [[part_id, "other_part"]]
                        },
                        {
                            "id": "req_dof",
                            "type": "mechanism_dof",
                            "expected": 1
                        }
                    ]
                }
            }
        }),
    )
    .await;
    assert_eq!(parse_content(&set_scene_resp)["status"], "requirements_set");

    // 4. Check all requirements (before FEA is run -> min_safety_factor is STALE)
    let check_1_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1505,
            "method": "tools/call",
            "params": {
                "name": "check_requirements",
                "arguments": {}
            }
        }),
    )
    .await;
    let check_1 = parse_content(&check_1_resp);
    assert_eq!(check_1["summary"]["passed"], 4);
    assert_eq!(check_1["summary"]["failed"], 0);
    assert_eq!(check_1["summary"]["stale"], 1);
    assert_eq!(check_1["summary"]["not_available"], 2);
    assert_eq!(check_1["status"], "STALE");

    let reqs_1 = check_1["requirements"].as_array().unwrap();
    let fos_1 = reqs_1.iter().find(|r| r["id"] == "req_fos").unwrap();
    assert_eq!(fos_1["status"], "STALE");
    assert!(fos_1["hint"].as_str().unwrap().contains("run_fea_analysis"));

    let interf_1 = reqs_1.iter().find(|r| r["id"] == "req_interf").unwrap();
    assert_eq!(interf_1["status"], "NOT_AVAILABLE");
    let dof_1 = reqs_1.iter().find(|r| r["id"] == "req_dof").unwrap();
    assert_eq!(dof_1["status"], "NOT_AVAILABLE");

    // 5. Run FEA analysis with fea_case = "cantilever_tip"
    let fea_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1506,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": part_id,
                    "fea_case": "cantilever_tip",
                    "mesh_density": "coarse",
                    "fixed": [{"face": "left"}],
                    "loads": [{"face": "right", "force": [0.0, -200.0, 0.0]}]
                }
            }
        }),
    )
    .await;
    let fea_json = parse_content(&fea_resp);
    assert_eq!(fea_json["part_revision"], 1);
    assert_eq!(fea_json["fea_case"], "cantilever_tip");
    assert!(fea_json["safety_factor"].as_f64().unwrap() > 2.0);

    // 6. Now check_requirements for part_id -> all 5 part requirements PASS!
    let check_2_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1507,
            "method": "tools/call",
            "params": {
                "name": "check_requirements",
                "arguments": {
                    "part_id": part_id
                }
            }
        }),
    )
    .await;
    let check_2 = parse_content(&check_2_resp);
    assert_eq!(check_2["status"], "PASS");
    assert_eq!(check_2["summary"]["passed"], 5);
    assert_eq!(check_2["summary"]["failed"], 0);
    assert_eq!(check_2["summary"]["stale"], 0);

    // Also verify check_requirements with length_unit = "m" reports bbox in meters
    let check_m_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1508,
            "method": "tools/call",
            "params": {
                "name": "check_requirements",
                "arguments": {
                    "part_id": part_id,
                    "length_unit": "m"
                }
            }
        }),
    )
    .await;
    let check_m = parse_content(&check_m_resp);
    let bbox_max_m = check_m["requirements"]
        .as_array()
        .unwrap()
        .iter()
        .find(|r| r["id"] == "req_bbox_max")
        .unwrap();
    assert!((bbox_max_m["measured"][0].as_f64().unwrap() - 0.100).abs() < 1e-6);
    assert!((bbox_max_m["target"][0].as_f64().unwrap() - 0.120).abs() < 1e-6);

    // 7. Modify feature to 200 x 100 x 40 mm -> violates bbox_max AND mass_max (mass = 2.16 kg > 0.50 kg),
    // and increments revision to 2 so FEA requirement becomes STALE!
    // Because auto_check = true, modify_feature response must include requirements_status and failed_requirements!
    let mod_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1509,
            "method": "tools/call",
            "params": {
                "name": "modify_feature",
                "arguments": {
                    "part_id": part_id,
                    "feature_id": box_feat_id,
                    "params": {
                        "width": 200.0,
                        "height": 100.0,
                        "depth": 40.0
                    }
                }
            }
        }),
    )
    .await;
    let mod_json = parse_content(&mod_resp);
    assert_eq!(mod_json["revision"], 2);
    assert!(
        mod_json["requirements_status"].is_object(),
        "modify_feature must include requirements_status when auto_check=true"
    );
    assert_eq!(mod_json["requirements_status"]["passed"], 2); // bbox_min, mesh_valid
    assert_eq!(mod_json["requirements_status"]["failed"], 2); // bbox_max, mass_max
    assert_eq!(mod_json["requirements_status"]["stale"], 1);  // min_safety_factor (part revision 2 != fea revision 1)
    let failed_reqs = mod_json["failed_requirements"]
        .as_array()
        .expect("failed_requirements must be present");
    assert_eq!(failed_reqs.len(), 2);
    assert!(failed_reqs.iter().any(|r| r["id"] == "req_bbox_max" && r["status"] == "FAIL"));
    assert!(failed_reqs.iter().any(|r| r["id"] == "req_mass" && r["status"] == "FAIL"));

    // 8. Cut a through-hole -> genus becomes 1, so mesh_valid {expected_genus: 0} also FAILS!
    let hole_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1510,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": part_id,
                    "position": [0.0, 0.0, -30.0],
                    "normal": [0.0, 0.0, 1.0],
                    "radius": 10.0,
                    "depth": 60.0
                }
            }
        }),
    )
    .await;
    let hole_json = parse_content(&hole_resp);
    assert_eq!(hole_json["revision"], 3);
    assert_eq!(hole_json["requirements_status"]["failed"], 3); // bbox_max, mass_max, mesh_valid (genus=1 != 0)
    let hole_failed = hole_json["failed_requirements"].as_array().unwrap();
    assert!(hole_failed.iter().any(|r| r["id"] == "req_mesh" && r["status"] == "FAIL"));

    // 9. Save and load project -> verify requirements and part revision persist across sessions
    let temp_dir = std::env::temp_dir().join(format!("mcpcad_p2t9_{}", uuid::Uuid::new_v4()));
    let proj_path = temp_dir.join("p2t9_project.json");
    let proj_path_str = proj_path.to_str().unwrap();

    let save_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1511,
            "method": "tools/call",
            "params": {
                "name": "save_project",
                "arguments": {
                    "path": proj_path_str
                }
            }
        }),
    )
    .await;
    assert_eq!(parse_content(&save_resp)["status"], "saved");

    let fresh_state = EngineState::new().unwrap().into_shared();
    let load_resp = rpc_call(
        &fresh_state,
        json!({
            "jsonrpc": "2.0",
            "id": 1512,
            "method": "tools/call",
            "params": {
                "name": "load_project",
                "arguments": {
                    "path": proj_path_str
                }
            }
        }),
    )
    .await;
    assert_eq!(parse_content(&load_resp)["status"], "loaded");

    let check_loaded_resp = rpc_call(
        &fresh_state,
        json!({
            "jsonrpc": "2.0",
            "id": 1513,
            "method": "tools/call",
            "params": {
                "name": "check_requirements",
                "arguments": {}
            }
        }),
    )
    .await;
    let check_loaded = parse_content(&check_loaded_resp);
    assert_eq!(check_loaded["auto_check"], true);
    assert_eq!(check_loaded["summary"]["total"], 7);
    assert_eq!(check_loaded["summary"]["failed"], 3);
    assert_eq!(check_loaded["summary"]["stale"], 1);
    assert_eq!(check_loaded["summary"]["not_available"], 2);

    // 10. Negative test: invalid requirement type returns structured INVALID_ARGUMENT
    let bad_req_resp = rpc_call(
        &fresh_state,
        json!({
            "jsonrpc": "2.0",
            "id": 1514,
            "method": "tools/call",
            "params": {
                "name": "set_requirements",
                "arguments": {
                    "part_id": part_id,
                    "requirements": [
                        { "type": "unknown_thermal_limit", "max_temp_c": 100.0 }
                    ]
                }
            }
        }),
    )
    .await;
    let bad_env = parse_error_envelope(&bad_req_resp);
    assert_eq!(bad_env.error.code, "INVALID_ARGUMENT");
    assert_eq!(bad_env.error.param.as_deref(), Some("requirements"));
    assert!(bad_env.error.details["supported_requirement_types"].is_array());

    let _ = std::fs::remove_dir_all(temp_dir);
}

/// Recursively verifies that every numeric/integer property or numeric array in a JSON Schema
/// has a non-empty `description` explicitly specifying a unit (or `dimensionless`).
fn assert_numeric_schema_fields_have_units(tool_name: &str, path: &str, schema: &Value) {
    let unit_keywords = [
        "length_unit",
        "mm",
        "m/s",
        "rad/s",
        "deg",
        "degree",
        "MPa",
        "GPa",
        "N*m",
        "N·m",
        "Newton",
        "(N)",
        "kg",
        "px",
        "pixel",
        "second",
        "(s)",
        "(ms)",
        "millisecond",
        "dimensionless",
    ];

    if let Some(props) = schema.get("properties").and_then(|v| v.as_object()) {
        for (prop_name, prop_schema) in props {
            let full_path = if path.is_empty() {
                prop_name.clone()
            } else {
                format!("{path}.{prop_name}")
            };

            let prop_type = prop_schema.get("type").and_then(|v| v.as_str()).unwrap_or("");
            let is_numeric_array = prop_type == "array"
                && prop_schema
                    .get("items")
                    .map(|items| {
                        items.get("type").and_then(|t| t.as_str()) == Some("number")
                            || items.get("type").and_then(|t| t.as_str()) == Some("integer")
                            || items
                                .get("items")
                                .and_then(|inner| inner.get("type"))
                                .and_then(|t| t.as_str())
                                == Some("number")
                    })
                    .unwrap_or(false);

            if prop_type == "number" || prop_type == "integer" || is_numeric_array {
                let desc = prop_schema
                    .get("description")
                    .and_then(|v| v.as_str())
                    .unwrap_or("");
                assert!(
                    !desc.is_empty(),
                    "Tool '{tool_name}' numeric field '{full_path}' is missing a description"
                );
                let desc_lower = desc.to_ascii_lowercase();
                let has_unit = unit_keywords
                    .iter()
                    .any(|kw| desc_lower.contains(&kw.to_ascii_lowercase()));
                assert!(
                    has_unit,
                    "Tool '{tool_name}' numeric field '{full_path}' description does not specify a unit (or 'dimensionless'): '{desc}'"
                );
            }

            // Recurse into nested object properties or array item object properties
            if prop_schema.get("properties").is_some() {
                assert_numeric_schema_fields_have_units(tool_name, &full_path, prop_schema);
            }
            if let Some(items) = prop_schema.get("items") {
                if items.get("properties").is_some() {
                    assert_numeric_schema_fields_have_units(
                        tool_name,
                        &format!("{full_path}[]"),
                        items,
                    );
                }
            }
        }
    }
}

#[tokio::test]
async fn test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Verify initialize.instructions contract
    let init_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1601,
            "method": "initialize",
            "params": {
                "protocolVersion": "2024-11-05",
                "capabilities": {},
                "clientInfo": { "name": "p2-t10-test", "version": "1.0.0" }
            }
        }),
    )
    .await;

    let instructions = init_resp["result"]["instructions"]
        .as_str()
        .expect("initialize result must contain 'instructions'");
    let char_count = instructions.chars().count();
    assert!(
        (800..=1650).contains(&char_count),
        "initialize.instructions should be ~1500 chars (got {char_count} chars)"
    );
    for required_topic in [
        "docs/AGENT_GUIDE.md",
        "get_scene_summary",
        "length_unit",
        "set_requirements",
        "auto_check",
        "batch_cad_operations",
        "validate_mesh",
        "run_fea_analysis",
        "query_part_geometry",
        "STALE",
        "isError",
    ] {
        assert!(
            instructions.contains(required_topic),
            "initialize.instructions must mention '{required_topic}'"
        );
    }

    // Verify docs/AGENT_GUIDE.md exists and is concise (<= 2 pages / <= 160 lines)
    let guide_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("docs/AGENT_GUIDE.md");
    let guide_text = std::fs::read_to_string(&guide_path)
        .expect("docs/AGENT_GUIDE.md must exist in repository");
    let guide_lines = guide_text.lines().count();
    assert!(
        (40..=160).contains(&guide_lines),
        "docs/AGENT_GUIDE.md must be concise (<= 2 pages, got {guide_lines} lines)"
    );
    assert!(guide_text.contains("batch_cad_operations"));
    assert!(guide_text.contains("validate_mesh"));
    assert!(guide_text.contains("set_requirements"));
    assert!(guide_text.contains("run_fea_analysis"));
    assert!(guide_text.contains("get_scene_summary"));

    // 2. Verify all 34 tools (including all 28 original tools) have 'Example', units, common errors,
    //    next step in description, and every numeric schema field specifies its unit.
    let list_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1602,
            "method": "tools/list"
        }),
    )
    .await;
    let tools = list_resp["result"]["tools"]
        .as_array()
        .expect("tools must be an array");
    assert_eq!(tools.len(), 34);

    for tool in tools {
        let name = tool["name"].as_str().unwrap();
        let desc = tool["description"].as_str().unwrap();

        assert!(
            desc.contains("Example"),
            "Tool '{name}' description must contain substring 'Example': '{desc}'"
        );
        assert!(
            desc.contains("Units:"),
            "Tool '{name}' description must document 'Units:': '{desc}'"
        );
        assert!(
            desc.contains("Common errors:"),
            "Tool '{name}' description must document 'Common errors:': '{desc}'"
        );
        assert!(
            desc.contains("Next step:"),
            "Tool '{name}' description must document 'Next step:': '{desc}'"
        );

        assert_numeric_schema_fields_have_units(name, "", &tool["inputSchema"]);
    }
}

#[tokio::test]
async fn test_mcp_p2_t10_get_scene_summary_two_parts() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 0. Empty scene summary check
    let empty_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1610,
            "method": "tools/call",
            "params": {
                "name": "get_scene_summary",
                "arguments": {}
            }
        }),
    )
    .await;
    let empty_summary = parse_content(&empty_resp);
    assert_eq!(empty_summary["parts_count"], 0);
    assert_eq!(empty_summary["joints_count"], 0);
    assert_eq!(empty_summary["length_unit"], "mm");
    assert!(empty_summary["summary_text"]
        .as_str()
        .unwrap()
        .contains("Parts (0):"));

    // 1. Create two parts and build geometry via batch_cad_operations
    let batch_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1611,
            "method": "tools/call",
            "params": {
                "name": "batch_cad_operations",
                "arguments": {
                    "operations": [
                        {
                            "tool": "create_part",
                            "params": {
                                "part_id": "base_bracket",
                                "name": "Base Bracket",
                                "material": "aluminum_6061_t6",
                                "is_fixed": true
                            }
                        },
                        {
                            "tool": "add_feature_primitive",
                            "params": {
                                "part_id": "base_bracket",
                                "primitive_type": "box",
                                "params": {
                                    "width": 100.0,
                                    "height": 20.0,
                                    "depth": 20.0
                                }
                            }
                        },
                        {
                            "tool": "create_part",
                            "params": {
                                "part_id": "rotor_arm",
                                "name": "Rotor Arm",
                                "material": "steel_structural_aisi1020",
                                "is_fixed": false
                            }
                        },
                        {
                            "tool": "add_feature_primitive",
                            "params": {
                                "part_id": "rotor_arm",
                                "primitive_type": "cylinder",
                                "params": {
                                    "radius": 10.0,
                                    "height": 50.0,
                                    "position": [50.0, 0.0, 25.0]
                                }
                            }
                        }
                    ]
                }
            }
        }),
    )
    .await;
    let batch_json = parse_content(&batch_resp);
    assert_eq!(batch_json["completed_steps"], 4);

    // Get box feature_id on base_bracket for later modification
    let tree_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1612,
            "method": "tools/call",
            "params": {
                "name": "get_feature_tree",
                "arguments": { "part_id": "base_bracket" }
            }
        }),
    )
    .await;
    let box_feat_id = parse_content(&tree_resp)["dag"][0]["id"]
        .as_str()
        .unwrap()
        .to_string();

    // 2. Declare engineering requirements on base_bracket with auto_check: true
    let req_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1613,
            "method": "tools/call",
            "params": {
                "name": "set_requirements",
                "arguments": {
                    "part_id": "base_bracket",
                    "auto_check": true,
                    "requirements": [
                        { "id": "req_bbox", "type": "bbox_max", "dims": [120.0, 30.0, 30.0] },
                        { "id": "req_mass", "type": "mass_max", "kg": 0.25 },
                        { "id": "req_mesh", "type": "mesh_valid" },
                        { "id": "req_fos", "type": "min_safety_factor", "value": 2.0, "fea_case": "tip_load" }
                    ]
                }
            }
        }),
    )
    .await;
    assert_eq!(parse_content(&req_resp)["status"], "requirements_set");

    // 3. Create a revolute joint connecting base_bracket and rotor_arm, and set a motor
    let joint_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1614,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_type": "revolute",
                    "part_a": "base_bracket",
                    "part_b": "rotor_arm",
                    "anchor_a": [50.0, 0.0, 10.0],
                    "anchor_b": [0.0, 0.0, -25.0],
                    "axis": [0.0, 0.0, 1.0]
                }
            }
        }),
    )
    .await;
    let joint_id = parse_content(&joint_resp)["joint_id"]
        .as_str()
        .unwrap()
        .to_string();

    let motor_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1615,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": joint_id,
                    "target_velocity": 5.0,
                    "max_force": 25.0
                }
            }
        }),
    )
    .await;
    assert_eq!(parse_content(&motor_resp)["status"], "motor_active");

    // 4. Run FEA analysis on base_bracket
    let fea_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1616,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": "base_bracket",
                    "fea_case": "tip_load",
                    "mesh_density": "coarse",
                    "fixed": [{ "face": "left" }],
                    "loads": [{ "face": "right", "force": [0.0, -200.0, 0.0] }]
                }
            }
        }),
    )
    .await;
    assert_eq!(parse_content(&fea_resp)["verdict"], "PASS");

    // 5. Call get_scene_summary and verify all fields and compact text for the 2-part scene
    let sum_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1617,
            "method": "tools/call",
            "params": {
                "name": "get_scene_summary",
                "arguments": {}
            }
        }),
    )
    .await;
    let summary = parse_content(&sum_resp);

    assert_eq!(summary["length_unit"], "mm");
    assert_eq!(summary["units"]["length"], "mm");
    assert_eq!(summary["parts_count"], 2);
    assert_eq!(summary["joints_count"], 1);

    let parts = summary["parts"].as_array().unwrap();
    assert_eq!(parts.len(), 2);
    assert_eq!(parts[0]["part_id"], "base_bracket");
    assert_eq!(parts[0]["name"], "Base Bracket");
    assert_eq!(parts[0]["is_fixed"], true);
    assert_eq!(parts[0]["mesh_valid"], true);
    assert_eq!(parts[0]["mesh_status"], "PASS");
    assert!((parts[0]["dimensions"][0].as_f64().unwrap() - 100.0).abs() < 1e-4);
    assert!((parts[0]["volume"].as_f64().unwrap() - 40_000.0).abs() < 1.0);
    assert!(parts[0]["mass_kg"].as_f64().unwrap() > 0.10);
    assert_eq!(parts[0]["last_fea"]["verdict"], "PASS");
    assert_eq!(parts[0]["last_fea"]["stale"], false);
    assert_eq!(parts[0]["last_fea"]["status"], "FRESH");
    assert!(parts[0]["last_fea"]["factor_of_safety"].as_f64().unwrap() > 2.0);

    assert_eq!(parts[1]["part_id"], "rotor_arm");
    assert_eq!(parts[1]["name"], "Rotor Arm");
    assert_eq!(parts[1]["is_fixed"], false);
    assert_eq!(parts[1]["mesh_valid"], true);
    assert!(parts[1]["last_fea"].is_null());

    let joints = summary["joints"].as_array().unwrap();
    assert_eq!(joints.len(), 1);
    assert_eq!(joints[0]["joint_id"], joint_id);
    assert_eq!(joints[0]["joint_type"], "revolute");
    assert_eq!(joints[0]["part_a"], "base_bracket");
    assert_eq!(joints[0]["part_b"], "rotor_arm");
    assert_eq!(joints[0]["has_motor"], true);
    assert_eq!(joints[0]["motor"]["target_velocity"], 5.0);

    assert_eq!(summary["requirements"]["status"], "PASS");
    assert_eq!(summary["requirements"]["auto_check"], true);
    assert_eq!(summary["requirements"]["total"], 4);
    assert_eq!(summary["requirements"]["passed"], 4);
    assert_eq!(summary["requirements"]["failed"], 0);
    assert_eq!(summary["requirements"]["stale"], 0);

    let text = summary["summary_text"].as_str().unwrap();
    assert!(text.contains("base_bracket"));
    assert!(text.contains("rotor_arm"));
    assert!(text.contains("FEA(verdict=PASS"));
    assert!(text.contains("status=FRESH"));
    assert!(text.contains(&joint_id));
    assert!(text.contains("Requirements (4, auto_check=true, status=PASS"));

    // 6. Modify base_bracket geometry -> bumps revision -> verify get_scene_summary reports FEA as STALE
    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1618,
            "method": "tools/call",
            "params": {
                "name": "modify_feature",
                "arguments": {
                    "part_id": "base_bracket",
                    "feature_id": box_feat_id,
                    "params": {
                        "width": 110.0,
                        "height": 20.0,
                        "depth": 20.0
                    }
                }
            }
        }),
    )
    .await;

    let sum_after_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1619,
            "method": "tools/call",
            "params": {
                "name": "get_scene_summary",
                "arguments": { "length_unit": "m" }
            }
        }),
    )
    .await;
    let sum_after = parse_content(&sum_after_resp);
    assert_eq!(sum_after["length_unit"], "m");
    let p0_after = &sum_after["parts"][0];
    assert!((p0_after["dimensions"][0].as_f64().unwrap() - 0.110).abs() < 1e-6);
    assert_eq!(p0_after["last_fea"]["stale"], true);
    assert_eq!(p0_after["last_fea"]["status"], "STALE");
    assert_eq!(sum_after["requirements"]["status"], "STALE");
    assert_eq!(sum_after["requirements"]["stale"], 1);
    assert!(sum_after["summary_text"]
        .as_str()
        .unwrap()
        .contains("status=STALE"));
}

#[tokio::test]
async fn test_mcp_p2_t11_set_joint_motor_numeric_validation() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let parse_content = |resp: &Value| -> Value {
        let text = resp["result"]["content"][0]["text"]
            .as_str()
            .expect("content[0].text must be a string");
        serde_json::from_str(text).expect("content[0].text must be valid JSON")
    };

    // 1. Create a part with cylinder geometry and a revolute joint to ground
    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1701,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Drive Shaft",
                    "part_id": "drive_shaft",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1702,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "drive_shaft",
                    "primitive_type": "cylinder",
                    "params": { "radius": 12.0, "height": 40.0 }
                }
            }
        }),
    )
    .await;

    let joint_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1703,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_type": "revolute",
                    "part_a": "ground",
                    "part_b": "drive_shaft",
                    "axis": [0.0, 0.0, 1.0]
                }
            }
        }),
    )
    .await;
    let joint_id = parse_content(&joint_resp)["joint_id"]
        .as_str()
        .unwrap()
        .to_string();

    // 2. Negative max_force (-10.0) -> must return structured INVALID_ARGUMENT on param 'max_force'
    let neg_force_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1704,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": joint_id,
                    "target_velocity": 10.0,
                    "max_force": -10.0
                }
            }
        }),
    )
    .await;
    let neg_err_str = assert_tool_error(&neg_force_resp);
    let neg_err_json: Value = serde_json::from_str(neg_err_str).unwrap();
    assert_eq!(neg_err_json["error"]["code"], "INVALID_ARGUMENT");
    assert_eq!(neg_err_json["error"]["param"], "max_force");
    assert!(neg_err_json["error"]["message"]
        .as_str()
        .unwrap()
        .contains("non-negative"));

    // 3. Non-finite / overflowing target_velocity (1e100 -> f32::INFINITY) -> INVALID_ARGUMENT on 'target_velocity'
    let inf_vel_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1705,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": joint_id,
                    "target_velocity": 1.0e100,
                    "max_force": 50.0
                }
            }
        }),
    )
    .await;
    let inf_vel_str = assert_tool_error(&inf_vel_resp);
    let inf_vel_json: Value = serde_json::from_str(inf_vel_str).unwrap();
    assert_eq!(inf_vel_json["error"]["code"], "INVALID_ARGUMENT");
    assert_eq!(inf_vel_json["error"]["param"], "target_velocity");
    assert!(inf_vel_json["error"]["message"]
        .as_str()
        .unwrap()
        .contains("finite"));

    // 4. Non-finite / overflowing max_force (1e100 -> f32::INFINITY) -> INVALID_ARGUMENT on 'max_force'
    let inf_force_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1706,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": joint_id,
                    "target_velocity": 10.0,
                    "max_force": 1.0e100
                }
            }
        }),
    )
    .await;
    let inf_force_str = assert_tool_error(&inf_force_resp);
    let inf_force_json: Value = serde_json::from_str(inf_force_str).unwrap();
    assert_eq!(inf_force_json["error"]["code"], "INVALID_ARGUMENT");
    assert_eq!(inf_force_json["error"]["param"], "max_force");
    assert!(inf_force_json["error"]["message"]
        .as_str()
        .unwrap()
        .contains("finite"));

    // 5. Non-numeric string "NaN" for target_velocity and max_force -> INVALID_ARGUMENT
    let nan_vel_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1707,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": joint_id,
                    "target_velocity": "NaN",
                    "max_force": 50.0
                }
            }
        }),
    )
    .await;
    let nan_vel_str = assert_tool_error(&nan_vel_resp);
    let nan_vel_json: Value = serde_json::from_str(nan_vel_str).unwrap();
    assert_eq!(nan_vel_json["error"]["code"], "INVALID_ARGUMENT");
    assert_eq!(nan_vel_json["error"]["param"], "target_velocity");

    // 6. Valid boundary values: max_force = 0.0 and negative target_velocity = -8.0 -> succeeds
    let valid_zero_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1708,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": joint_id,
                    "target_velocity": -8.0,
                    "max_force": 0.0
                }
            }
        }),
    )
    .await;
    assert_ne!(valid_zero_resp["result"]["isError"], json!(true));
    assert_eq!(parse_content(&valid_zero_resp)["status"], "motor_active");
}

#[tokio::test]
async fn test_mcp_p2_t12_batch_physics_rollback_and_fixed_part_preservation() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let parse_content = |resp: &Value| -> Value {
        let text = resp["result"]["content"][0]["text"]
            .as_str()
            .expect("content[0].text must be a string");
        serde_json::from_str(text).expect("content[0].text must be valid JSON")
    };

    // 1. Create a fixed base part (`is_fixed: true`) and a movable cutter/weight part (`is_fixed: false`)
    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1801,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Fixed Anchor Block",
                    "part_id": "fixed_anchor",
                    "material": "steel_structural_aisi1020",
                    "is_fixed": true
                }
            }
        }),
    )
    .await;

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1802,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "fixed_anchor",
                    "primitive_type": "box",
                    "params": { "width": 80.0, "height": 40.0, "depth": 40.0 }
                }
            }
        }),
    )
    .await;

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1803,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Cutter & Dynamic Weight",
                    "part_id": "dyn_weight",
                    "material": "aluminum_6061_t6",
                    "is_fixed": false
                }
            }
        }),
    )
    .await;

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1804,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "dyn_weight",
                    "primitive_type": "box",
                    "params": { "width": 20.0, "height": 20.0, "depth": 20.0, "position": [10.0, 0.0, 0.0] }
                }
            }
        }),
    )
    .await;

    // 2. Apply boolean_difference, boolean_union, boolean_intersection, transform_part, and optimize_feature_tree
    //    on `fixed_anchor` (`is_fixed: true`) and verify it stays fixed in `physics_world`.
    for (req_id, tool_name, args) in [
        (
            1805,
            "boolean_difference",
            json!({ "target_part_id": "fixed_anchor", "tool_part_id": "dyn_weight" }),
        ),
        (
            1806,
            "boolean_union",
            json!({ "target_part_id": "fixed_anchor", "tool_part_id": "dyn_weight" }),
        ),
        (
            1807,
            "boolean_intersection",
            json!({ "target_part_id": "fixed_anchor", "tool_part_id": "dyn_weight" }),
        ),
        (
            1808,
            "transform_part",
            json!({ "part_id": "fixed_anchor", "translation": [0.0, 10.0, 0.0] }),
        ),
        (
            1809,
            "optimize_feature_tree",
            json!({ "part_id": "fixed_anchor", "apply": true }),
        ),
    ] {
        let resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": req_id,
                "method": "tools/call",
                "params": {
                    "name": tool_name,
                    "arguments": args
                }
            }),
        )
        .await;
        assert_ne!(
            resp["result"]["isError"],
            json!(true),
            "{tool_name} failed unexpectedly: {resp}"
        );
        let guard = state.read();
        assert!(
            guard.physics_world.has_part("fixed_anchor"),
            "{tool_name} must register 'fixed_anchor' in physics_world"
        );
        assert_eq!(
            guard.physics_world.is_part_dynamic("fixed_anchor").unwrap(),
            false,
            "{tool_name} must preserve !part.is_fixed (is_dynamic == false) for fixed_anchor"
        );
    }

    // 3. Create a revolute joint before running a failing `batch_cad_operations`
    let joint_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1810,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_type": "revolute",
                    "part_a": "fixed_anchor",
                    "part_b": "dyn_weight",
                    "axis": [0.0, 0.0, 1.0]
                }
            }
        }),
    )
    .await;
    assert_ne!(joint_resp["result"]["isError"], json!(true));
    let saved_joint_id = parse_content(&joint_resp)["joint_id"]
        .as_str()
        .unwrap()
        .to_string();

    // 4. Execute `batch_cad_operations` that modifies `physics_world` (deletes `dyn_weight`, creates
    //    `batch_temp` + joint) and then fails on the last step -> verify `physics_world` is rolled back!
    let batch_fail_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1811,
            "method": "tools/call",
            "params": {
                "name": "batch_cad_operations",
                "arguments": {
                    "operations": [
                        {
                            "tool": "delete_part",
                            "params": { "part_id": "dyn_weight" }
                        },
                        {
                            "tool": "create_part",
                            "params": {
                                "name": "Batch Temp Part",
                                "part_id": "batch_temp",
                                "material": "aluminum_6061_t6"
                            }
                        },
                        {
                            "tool": "add_feature_primitive",
                            "params": {
                                "part_id": "batch_temp",
                                "primitive_type": "cylinder",
                                "params": { "radius": 8.0, "height": 20.0 }
                            }
                        },
                        {
                            "tool": "create_joint",
                            "params": {
                                "joint_type": "revolute",
                                "part_a": "ground",
                                "part_b": "batch_temp",
                                "axis": [0.0, 0.0, 1.0]
                            }
                        },
                        {
                            "tool": "delete_part",
                            "params": { "part_id": "missing_part_causes_rollback" }
                        }
                    ]
                }
            }
        }),
    )
    .await;
    let _ = assert_tool_error(&batch_fail_resp);

    // Check `get_scene_summary` and `physics_world`: `dyn_weight` and `saved_joint_id` must still exist,
    // and `batch_temp` must not exist in either `parts` or `physics_world`.
    {
        let guard = state.read();
        assert!(guard.parts.contains_key("dyn_weight"));
        assert!(!guard.parts.contains_key("batch_temp"));
        assert!(guard.physics_world.has_part("dyn_weight"));
        assert!(!guard.physics_world.has_part("batch_temp"));
        assert_eq!(guard.physics_world.joints().len(), 1);
        assert!(guard.physics_world.joints().contains_key(&saved_joint_id));
    }

    // 5. Run physics simulation and verify `fixed_anchor` stays completely stationary under gravity
    let sim_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1812,
            "method": "tools/call",
            "params": {
                "name": "run_physics_simulation",
                "arguments": {
                    "duration_seconds": 0.25,
                    "step_size_ms": 16.6667
                }
            }
        }),
    )
    .await;
    assert_ne!(sim_resp["result"]["isError"], json!(true));
    let sim_json = parse_content(&sim_resp);
    let fixed_samples = sim_json["trajectory_summary"]["fixed_anchor"]["samples"]
        .as_array()
        .expect("fixed_anchor trajectory samples");
    let first_pos = &fixed_samples.first().unwrap()["position"];
    let last_pos = &fixed_samples.last().unwrap()["position"];
    for axis in 0..3 {
        let delta = (last_pos[axis].as_f64().unwrap() - first_pos[axis].as_f64().unwrap()).abs();
        assert!(
            delta < 1e-9,
            "Fixed part 'fixed_anchor' must not move under gravity (axis {axis} delta = {delta})"
        );
    }
}

#[tokio::test]
async fn test_mcp_p2_t13_custom_joint_id_through_hole_and_aliases() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let parse_content = |resp: &Value| -> Value {
        let text = resp["result"]["content"][0]["text"]
            .as_str()
            .expect("Expected content[0].text string");
        serde_json::from_str(text).expect("Expected valid JSON in content[0].text")
    };

    // 1. Create part with material alias 'steel_4140' and add polygon_prism with 'outer_radius' + 'chamfer_radius'
    let cp_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1901,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "M10 Hex Nut",
                    "part_id": "nut_p2_t13",
                    "material": "steel_4140"
                }
            }
        }),
    )
    .await;
    assert_ne!(cp_resp["result"]["isError"], json!(true));

    let prism_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1902,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "nut_p2_t13",
                    "primitive_type": "polygon_prism",
                    "params": {
                        "sides": 6,
                        "outer_radius": 9.8,
                        "height": 8.0,
                        "chamfer_radius": 0.8
                    }
                }
            }
        }),
    )
    .await;
    assert_ne!(prism_resp["result"]["isError"], json!(true));

    // 2. Cut through-hole from z = 0 with 'through': true (no 'depth')
    let hole_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1903,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": "nut_p2_t13",
                    "position": [0.0, 0.0, 0.0],
                    "normal": [0.0, 0.0, -1.0],
                    "radius": 4.25,
                    "through": true
                }
            }
        }),
    )
    .await;
    assert_ne!(hole_resp["result"]["isError"], json!(true));

    let val_nut_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1904,
            "method": "tools/call",
            "params": {
                "name": "validate_mesh",
                "arguments": {
                    "part_id": "nut_p2_t13",
                    "expected_genus": 1
                }
            }
        }),
    )
    .await;
    assert_ne!(val_nut_resp["result"]["isError"], json!(true));
    let val_nut = parse_content(&val_nut_resp);
    assert_eq!(val_nut["status"], "PASS");
    assert_eq!(val_nut["topology"]["is_watertight"], true);
    assert_eq!(val_nut["topology"]["genus"].as_f64().unwrap().round() as i32, 1);

    // 3. Create annular bushing via revolve with 2-point profile + inner_radius and 'brass_c36000' alias
    let cp_bushing = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1905,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Annular Bushing",
                    "part_id": "bushing_p2_t13",
                    "material": "brass_c36000"
                }
            }
        }),
    )
    .await;
    assert_ne!(cp_bushing["result"]["isError"], json!(true));

    let rev_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1906,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "bushing_p2_t13",
                    "primitive_type": "revolve",
                    "params": {
                        "profile": [[35.0, -40.0], [35.0, 40.0]],
                        "inner_radius": 15.0,
                        "angle_deg": 360.0
                    }
                }
            }
        }),
    )
    .await;
    assert_ne!(rev_resp["result"]["isError"], json!(true));

    let val_bushing_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1907,
            "method": "tools/call",
            "params": {
                "name": "validate_mesh",
                "arguments": {
                    "part_id": "bushing_p2_t13",
                    "expected_genus": 1
                }
            }
        }),
    )
    .await;
    assert_ne!(val_bushing_resp["result"]["isError"], json!(true));
    let val_bushing = parse_content(&val_bushing_resp);
    assert_eq!(val_bushing["status"], "PASS");
    assert_eq!(val_bushing["topology"]["genus"].as_f64().unwrap().round() as i32, 1);

    // 4. Create joint with custom 'joint_id' and actuate it via set_joint_motor
    let j_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1908,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_bushing_drive",
                    "joint_type": "revolute",
                    "part_a": "ground",
                    "part_b": "bushing_p2_t13",
                    "axis": [0.0, 0.0, 1.0]
                }
            }
        }),
    )
    .await;
    assert_ne!(j_resp["result"]["isError"], json!(true));
    let j_json = parse_content(&j_resp);
    assert_eq!(j_json["joint_id"], "j_bushing_drive");

    let motor_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1909,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": "j_bushing_drive",
                    "target_velocity": 6.283185,
                    "max_force": 200.0
                }
            }
        }),
    )
    .await;
    assert_ne!(motor_resp["result"]["isError"], json!(true));
    let motor_json = parse_content(&motor_resp);
    assert_eq!(motor_json["status"], "motor_active");
}

#[tokio::test]
async fn test_mcp_p2_t15_small_wheel_rim_and_prismatic_motor_and_revolute_limits() {
    let state = EngineState::new().unwrap().into_shared();

    // 1. Small wheel_rim (rim_diameter = 100 mm = 0.10 m < 0.14 m) must not panic and must pass validate_mesh
    let p_wheel = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1950,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "rc_wheel_100mm",
                    "name": "RC Wheel 100mm",
                    "material": "aluminum_6061_t6"
                }
            }
        }),
    )
    .await;
    assert_ne!(p_wheel["result"]["isError"], json!(true));

    let add_wheel = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1951,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "rc_wheel_100mm",
                    "primitive_type": "wheel_rim",
                    "params": {
                        "rim_diameter": 100.0,
                        "rim_width": 45.0,
                        "bolt_holes_count": 5,
                        "spokes_count": 5
                    }
                }
            }
        }),
    )
    .await;
    assert_ne!(add_wheel["result"]["isError"], json!(true));
    let wheel_json = parse_content(&add_wheel);
    assert!(
        wheel_json["warnings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|w| w.as_str().unwrap_or("").contains("WHEEL_RIM_SCALE")),
        "Expected WHEEL_RIM_SCALE warning on 100 mm wheel rim"
    );

    let val_wheel = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1952,
            "method": "tools/call",
            "params": {
                "name": "validate_mesh",
                "arguments": {
                    "part_id": "rc_wheel_100mm",
                    "expected_genus": 11
                }
            }
        }),
    )
    .await;
    assert_ne!(val_wheel["result"]["isError"], json!(true));
    let val_wheel_json = parse_content(&val_wheel);
    assert_eq!(val_wheel_json["status"], "PASS");

    // 2. Prismatic joint + set_joint_motor + step_simulation + revolute limits validation
    for (id, pid, is_fixed, pos_z) in [
        (1953, "rail_base", true, 0.0),
        (1954, "slider_block", false, 50.0),
    ] {
        rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": id,
                "method": "tools/call",
                "params": {
                    "name": "create_part",
                    "arguments": {
                        "part_id": pid,
                        "name": pid,
                        "material": "structural_steel",
                        "is_fixed": is_fixed
                    }
                }
            }),
        )
        .await;
        rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": id + 10,
                "method": "tools/call",
                "params": {
                    "name": "add_feature_primitive",
                    "arguments": {
                        "part_id": pid,
                        "primitive_type": "box",
                        "params": {
                            "width": 40.0,
                            "height": 40.0,
                            "depth": 40.0,
                            "position": [0.0, 0.0, pos_z]
                        }
                    }
                }
            }),
        )
        .await;
    }

    // Malformed limits (wrong length) must return INVALID_ARGUMENT
    let bad_limits = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1955,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_type": "revolute",
                    "part_a": "rail_base",
                    "part_b": "slider_block",
                    "limits": [-45.0]
                }
            }
        }),
    )
    .await;
    let bad_limits_err: Value = serde_json::from_str(assert_tool_error(&bad_limits)).unwrap();
    assert_eq!(bad_limits_err["error"]["code"], "INVALID_ARGUMENT");

    // Valid prismatic joint with limits [-500, 500] mm
    let pris_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1956,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_linear_actuator",
                    "joint_type": "prismatic",
                    "part_a": "rail_base",
                    "part_b": "slider_block",
                    "anchor_a": [0.0, 0.0, 50.0],
                    "anchor_b": [0.0, 0.0, 0.0],
                    "axis": [1.0, 0.0, 0.0],
                    "limits": [-500.0, 500.0]
                }
            }
        }),
    )
    .await;
    assert_ne!(pris_resp["result"]["isError"], json!(true));

    let motor_pris = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1957,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": "j_linear_actuator",
                    "target_velocity": 200.0,
                    "max_force": 500.0
                }
            }
        }),
    )
    .await;
    assert_ne!(motor_pris["result"]["isError"], json!(true));

    let sim_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1958,
            "method": "tools/call",
            "params": {
                "name": "run_physics_simulation",
                "arguments": {
                    "duration_seconds": 0.5,
                    "step_size_ms": 10.0
                }
            }
        }),
    )
    .await;
    assert_ne!(sim_resp["result"]["isError"], json!(true));
    let sim_json = parse_content(&sim_resp);
    let slider_samples = sim_json["trajectory_summary"]["slider_block"]["samples"]
        .as_array()
        .unwrap();
    let final_x = slider_samples.last().unwrap()["position"][0]
        .as_f64()
        .unwrap();
    assert!(
        final_x > 50.0,
        "Expected slider_block to translate > 50 mm along +X under prismatic motor, got {final_x} mm"
    );

    // Also verify revolute joint with degree limits [-45, 45]
    let rev_lim_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 1959,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_rev_limited",
                    "joint_type": "revolute",
                    "part_a": "rail_base",
                    "part_b": "rc_wheel_100mm",
                    "axis": [0.0, 0.0, 1.0],
                    "limits": [-45.0, 45.0]
                }
            }
        }),
    )
    .await;
    assert_ne!(rev_lim_resp["result"]["isError"], json!(true));
}

#[tokio::test]
async fn test_mcp_p2_t16_fea_strict_fixed_faces_and_applied_forces_validation() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();


    let create_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2001,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "CantileverBeam",
                    "material": "steel_1045"
                }
            }
        }),
    )
    .await;
    assert_ne!(create_resp["result"]["isError"], json!(true));
    let part_id = parse_content(&create_resp)["part_id"]
        .as_str()
        .unwrap()
        .to_string();

    let add_prim_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2002,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": &part_id,
                    "primitive_type": "box",
                    "params": {"width": 100.0, "height": 20.0, "depth": 20.0}
                }
            }
        }),
    )
    .await;
    assert_ne!(add_prim_resp["result"]["isError"], json!(true));


    // 1. Typo in fixed_faces ("min_z ") must return FACE_NOT_FOUND
    let bad_fixed = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2002,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": part_id,
                    "fixed_faces": ["min_z "],
                    "applied_forces": [{"position": [50.0, 0.0, 0.0], "force_vector": [0.0, -500.0, 0.0]}]
                }
            }
        }),
    )
    .await;
    let bad_fixed_err: Value = serde_json::from_str(assert_tool_error(&bad_fixed)).unwrap();
    assert_eq!(bad_fixed_err["error"]["code"], "FACE_NOT_FOUND");

    // 2. Empty fixed_faces ([]) must return FEA_UNCONSTRAINED
    let empty_fixed = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2003,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": part_id,
                    "fixed_faces": [],
                    "applied_forces": [{"position": [50.0, 0.0, 0.0], "force_vector": [0.0, -500.0, 0.0]}]
                }
            }
        }),
    )
    .await;
    let empty_fixed_err: Value = serde_json::from_str(assert_tool_error(&empty_fixed)).unwrap();
    assert_eq!(empty_fixed_err["error"]["code"], "FEA_UNCONSTRAINED");
    assert_eq!(empty_fixed_err["error"]["param"], "fixed_faces");

    // 3. Empty applied_forces ([]) must return FEA_NO_LOADS
    let empty_forces = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2004,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": part_id,
                    "fixed_faces": ["min_x"],
                    "applied_forces": []
                }
            }
        }),
    )
    .await;
    let empty_forces_err: Value = serde_json::from_str(assert_tool_error(&empty_forces)).unwrap();
    assert_eq!(empty_forces_err["error"]["code"], "FEA_NO_LOADS");
    assert_eq!(empty_forces_err["error"]["param"], "applied_forces");

    // 4. applied_forces entry missing force / force_vector must return FEA_NO_LOADS
    let missing_force = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2005,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": part_id,
                    "fixed_faces": ["min_x"],
                    "applied_forces": [{"position": [50.0, 0.0, 0.0], "wrong_key": [0.0, -500.0, 0.0]}]
                }
            }
        }),
    )
    .await;
    let missing_force_err: Value = serde_json::from_str(assert_tool_error(&missing_force)).unwrap();
    assert_eq!(missing_force_err["error"]["code"], "FEA_NO_LOADS");

    // 5. applied_forces entry with 2-element position must return INVALID_ARGUMENT
    let bad_pos = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2006,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": part_id,
                    "fixed_faces": ["min_x"],
                    "applied_forces": [{"position": [50.0, 0.0], "force_vector": [0.0, -500.0, 0.0]}]
                }
            }
        }),
    )
    .await;
    let bad_pos_err: Value = serde_json::from_str(assert_tool_error(&bad_pos)).unwrap();
    assert_eq!(bad_pos_err["error"]["code"], "INVALID_ARGUMENT");
    assert_eq!(bad_pos_err["error"]["param"], "position");


    // 6. Valid fixed_faces + applied_forces with face_id string name ("max_x") succeeds
    let valid_fea = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2007,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": part_id,
                    "fixed_faces": ["min_x"],
                    "applied_forces": [{"face_id": "max_x", "force": [0.0, -500.0, 0.0]}]
                }
            }
        }),
    )
    .await;
    assert_ne!(valid_fea["result"]["isError"], json!(true));
    let fea_json = parse_content(&valid_fea);
    assert!(
        fea_json["max_von_mises_mpa"].as_f64().unwrap() > 0.0,
        "Expected positive von Mises stress for valid cantilever load"
    );
}

#[tokio::test]
async fn test_mcp_p2_t17_joint_motor_transactionality_and_gear_motor_rejection() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create two parts with cylinder geometry
    for (idx, p_id) in [(2101, "shaft_a"), (2102, "shaft_b")] {
        let c_resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": idx,
                "method": "tools/call",
                "params": {
                    "name": "create_part",
                    "arguments": {
                        "part_id": p_id,
                        "name": p_id,
                        "material": "steel_1045"
                    }
                }
            }),
        )
        .await;
        assert_ne!(c_resp["result"]["isError"], json!(true));

        let p_resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": idx + 10,
                "method": "tools/call",
                "params": {
                    "name": "add_feature_primitive",
                    "arguments": {
                        "part_id": p_id,
                        "primitive_type": "cylinder",
                        "params": { "radius": 15.0, "height": 40.0 }
                    }
                }
            }),
        )
        .await;
        assert_ne!(p_resp["result"]["isError"], json!(true));
    }

    // 2. Create spherical, fixed, and gear joints and attempt set_joint_motor on each
    for (idx, j_id, j_type) in [
        (2120, "j_sph_1", "spherical"),
        (2121, "j_fix_1", "fixed"),
        (2122, "j_gear_1", "gear"),
    ] {
        let j_resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": idx,
                "method": "tools/call",
                "params": {
                    "name": "create_joint",
                    "arguments": {
                        "joint_id": j_id,
                        "joint_type": j_type,
                        "part_a": "shaft_a",
                        "part_b": "shaft_b",
                        "ratio": 0.5
                    }
                }
            }),
        )
        .await;
        assert_ne!(j_resp["result"]["isError"], json!(true));

        let m_resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": idx + 10,
                "method": "tools/call",
                "params": {
                    "name": "set_joint_motor",
                    "arguments": {
                        "joint_id": j_id,
                        "target_velocity": 8.0,
                        "max_force": 40.0
                    }
                }
            }),
        )
        .await;
        let err_env = parse_error_envelope(&m_resp);
        assert_eq!(err_env.error.code, "INVALID_JOINT_CONFIGURATION");
        assert_eq!(err_env.error.param.as_deref(), Some("joint_id"));
    }

    // 3. Verify get_scene_summary shows has_motor == false for all 3 joints
    let sum_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2140,
            "method": "tools/call",
            "params": {
                "name": "get_scene_summary",
                "arguments": {}
            }
        }),
    )
    .await;
    let sum_json = parse_content(&sum_resp);
    let joints = sum_json["joints"].as_array().unwrap();
    assert_eq!(joints.len(), 3);
    for j in joints {
        assert_eq!(
            j["has_motor"], false,
            "Failed set_joint_motor must not leave has_motor=true on {}",
            j["joint_id"]
        );
        assert!(j["motor"].is_null());
    }

    // 4. Verify save_project and load_project roundtrip succeeds cleanly
    let temp_dir = std::env::temp_dir().join(format!("mcpcad_contract_p2t17_{}", uuid::Uuid::new_v4()));
    let proj_path = temp_dir.join("scene.json");
    let proj_path_str = proj_path.to_str().unwrap();

    let save_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2141,
            "method": "tools/call",
            "params": {
                "name": "save_project",
                "arguments": { "path": proj_path_str }
            }
        }),
    )
    .await;
    assert_ne!(save_resp["result"]["isError"], json!(true));

    let load_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2142,
            "method": "tools/call",
            "params": {
                "name": "load_project",
                "arguments": { "path": proj_path_str }
            }
        }),
    )
    .await;
    assert_ne!(
        load_resp["result"]["isError"],
        json!(true),
        "load_project must succeed after failed set_joint_motor calls"
    );
    assert_eq!(parse_content(&load_resp)["joints_count"], 3);

    // 5. Verify that a hand-crafted project file with a motor on a Gear joint is rejected by load_project
    //    and leaves the existing session state untouched
    let mut raw_proj: Value =
        serde_json::from_str(&std::fs::read_to_string(&proj_path).unwrap()).unwrap();
    for j in raw_proj["joints"].as_array_mut().unwrap() {
        if j["id"] == "j_gear_1" {
            j["motor"] = json!({
                "target_velocity": 5.0,
                "max_force": 20.0
            });
        }
    }
    let bad_gear_path = temp_dir.join("bad_gear_motor.json");
    std::fs::write(&bad_gear_path, serde_json::to_string_pretty(&raw_proj).unwrap()).unwrap();

    let bad_load_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2143,
            "method": "tools/call",
            "params": {
                "name": "load_project",
                "arguments": { "path": bad_gear_path.to_str().unwrap() }
            }
        }),
    )
    .await;
    let bad_load_err = parse_error_envelope(&bad_load_resp);
    assert_eq!(bad_load_err.error.code, "INVALID_JOINT_CONFIGURATION");
    assert_eq!(bad_load_err.error.param.as_deref(), Some("joint_id"));

    let _ = std::fs::remove_dir_all(temp_dir);
}

#[tokio::test]
async fn test_mcp_p2_t18_preserve_joints_on_csg_transform_and_update_collider_on_feature_edits() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create parts: drive_wheel, follower_wheel, and csg_tool
    for (req_id, p_id) in [
        (2201, "drive_wheel"),
        (2202, "follower_wheel"),
        (2203, "csg_tool"),
    ] {
        let resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": req_id,
                "method": "tools/call",
                "params": {
                    "name": "create_part",
                    "arguments": {
                        "part_id": p_id,
                        "name": p_id,
                        "material": "steel_1045"
                    }
                }
            }),
        )
        .await;
        assert_ne!(resp["result"]["isError"], json!(true));
    }

    let fw_prim = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2204,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "follower_wheel",
                    "primitive_type": "cylinder",
                    "params": { "radius": 20.0, "height": 30.0 }
                }
            }
        }),
    )
    .await;
    assert_ne!(fw_prim["result"]["isError"], json!(true));
    let fw_feat_id = parse_content(&fw_prim)["feature_id"]
        .as_str()
        .unwrap()
        .to_string();

    for (req_id, p_id, r, h) in [(2205, "drive_wheel", 30.0, 30.0), (2206, "csg_tool", 10.0, 40.0)]
    {
        let resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": req_id,
                "method": "tools/call",
                "params": {
                    "name": "add_feature_primitive",
                    "arguments": {
                        "part_id": p_id,
                        "primitive_type": "cylinder",
                        "params": { "radius": r, "height": h }
                    }
                }
            }),
        )
        .await;
        assert_ne!(resp["result"]["isError"], json!(true));
    }

    // 2. Create revolute joint (with active motor) and gear joint BEFORE modifying geometry
    let j1 = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2210,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_drive_rev",
                    "joint_type": "revolute",
                    "part_a": "ground",
                    "part_b": "drive_wheel",
                    "axis": [0.0, 0.0, 1.0]
                }
            }
        }),
    )
    .await;
    assert_ne!(j1["result"]["isError"], json!(true));

    let m1 = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2211,
            "method": "tools/call",
            "params": {
                "name": "set_joint_motor",
                "arguments": {
                    "joint_id": "j_drive_rev",
                    "target_velocity": 8.0,
                    "max_force": 500.0
                }
            }
        }),
    )
    .await;
    assert_ne!(m1["result"]["isError"], json!(true));

    let j2 = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2212,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_gear_couple",
                    "joint_type": "gear",
                    "part_a": "drive_wheel",
                    "part_b": "follower_wheel",
                    "ratio": 0.5
                }
            }
        }),
    )
    .await;
    assert_ne!(j2["result"]["isError"], json!(true));

    // 3. Apply CSG operations, transform_part, and optimize_feature_tree to drive_wheel
    for (req_id, tool_name, args) in [
        (
            2220,
            "boolean_difference",
            json!({ "target_part_id": "drive_wheel", "tool_part_id": "csg_tool" }),
        ),
        (
            2221,
            "boolean_union",
            json!({ "target_part_id": "drive_wheel", "tool_part_id": "csg_tool" }),
        ),
        (
            2222,
            "transform_part",
            json!({ "part_id": "drive_wheel", "translation": [0.0, 0.0, 5.0] }),
        ),
        (
            2223,
            "optimize_feature_tree",
            json!({ "part_id": "drive_wheel", "apply": true }),
        ),
    ] {
        let resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": req_id,
                "method": "tools/call",
                "params": {
                    "name": tool_name,
                    "arguments": args
                }
            }),
        )
        .await;
        assert_ne!(
            resp["result"]["isError"],
            json!(true),
            "{tool_name} failed: {resp}"
        );
    }

    // 4. Apply modify_feature, add_feature_primitive, cut_hole, and tessellate_part to follower_wheel
    let mod_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2230,
            "method": "tools/call",
            "params": {
                "name": "modify_feature",
                "arguments": {
                    "part_id": "follower_wheel",
                    "feature_id": fw_feat_id,
                    "params": { "radius": 45.0, "height": 30.0 }
                }
            }
        }),
    )
    .await;
    assert_ne!(mod_resp["result"]["isError"], json!(true));

    let hole_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2231,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": "follower_wheel",
                    "position": [0.0, 0.0, 0.0],
                    "normal": [0.0, 0.0, 1.0],
                    "radius": 8.0,
                    "through": true
                }
            }
        }),
    )
    .await;
    assert_ne!(hole_resp["result"]["isError"], json!(true));

    let tess_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2232,
            "method": "tools/call",
            "params": {
                "name": "tessellate_part",
                "arguments": {
                    "part_id": "follower_wheel",
                    "quality": "fine"
                }
            }
        }),
    )
    .await;
    assert_ne!(tess_resp["result"]["isError"], json!(true));

    // Verify collider AABB for follower_wheel in physics_world was updated to R=45 mm (span 0.09 m)
    {
        let guard = state.read();
        let (f_min, f_max) = guard
            .physics_world
            .part_collider_aabb("follower_wheel")
            .unwrap();
        assert!(
            (f_max[0] - f_min[0] - 0.09).abs() < 0.005,
            "follower_wheel collider X span must be ~0.09 m after modify_feature, got [{}, {}]",
            f_min[0],
            f_max[0]
        );
        assert_eq!(guard.physics_world.joints().len(), 2);
    }

    // 5. Verify get_scene_summary still lists both joints with j_drive_rev motor active
    let sum_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2240,
            "method": "tools/call",
            "params": {
                "name": "get_scene_summary",
                "arguments": {}
            }
        }),
    )
    .await;
    let sum_json = parse_content(&sum_resp);
    assert_eq!(sum_json["joints_count"], 2);

    // 6. Run physics simulation and verify both drive_wheel and follower_wheel rotate via the preserved joints
    let sim_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2241,
            "method": "tools/call",
            "params": {
                "name": "run_physics_simulation",
                "arguments": {
                    "duration_seconds": 0.2,
                    "step_size_ms": 10.0
                }
            }
        }),
    )
    .await;
    assert_ne!(sim_resp["result"]["isError"], json!(true));
    let sim_json = parse_content(&sim_resp);
    assert!(sim_json["peak_joint_forces"]["j_drive_rev"].is_number());
    let drive_samples = sim_json["trajectory_summary"]["drive_wheel"]["samples"]
        .as_array()
        .unwrap();
    let follower_samples = sim_json["trajectory_summary"]["follower_wheel"]["samples"]
        .as_array()
        .unwrap();
    let drive_wz = drive_samples.last().unwrap()["angular_velocity"][2]
        .as_f64()
        .unwrap();
    let follower_wz = follower_samples.last().unwrap()["angular_velocity"][2]
        .as_f64()
        .unwrap();
    assert!(
        drive_wz.abs() > 1.0,
        "drive_wheel should rotate via preserved revolute motor, got wz={drive_wz}"
    );
    assert!(
        (follower_wz + 0.5 * drive_wz).abs() < 0.2,
        "follower_wheel wz ({follower_wz}) should equal -0.5 * drive_wheel wz ({drive_wz}) via preserved gear constraint"
    );
}

#[tokio::test]
async fn test_mcp_p2_t19_modify_feature_transactional_rollback_and_helical_groove_start_z() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Verify tools/list schema for `cut_helical_groove` does not require `start_z`
    let list_tools_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2300,
            "method": "tools/list",
            "params": {}
        }),
    )
    .await;
    let tools = list_tools_resp["result"]["tools"].as_array().unwrap();
    let groove_tool = tools
        .iter()
        .find(|t| t["name"] == "cut_helical_groove")
        .expect("cut_helical_groove tool must exist");
    let req_fields = groove_tool["inputSchema"]["required"].as_array().unwrap();
    assert!(
        !req_fields.contains(&json!("start_z")),
        "cut_helical_groove inputSchema.required must not include 'start_z': {req_fields:?}"
    );

    // 2. Run T08 M10x1.5 hex nut workflow in SI meters (`set_units` = "m") WITHOUT passing `start_z`
    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2301,
            "method": "tools/call",
            "params": {
                "name": "set_units",
                "arguments": { "length_unit": "m" }
            }
        }),
    )
    .await;

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2302,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "nut_m10",
                    "name": "M10x1.5 Hex Nut",
                    "material": "steel_4140"
                }
            }
        }),
    )
    .await;

    let prism_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2303,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "nut_m10",
                    "primitive_type": "polygon_prism",
                    "params": {
                        "sides": 6,
                        "outer_radius": 0.0098,
                        "height": 0.008,
                        "chamfer_radius": 0.0008
                    }
                }
            }
        }),
    )
    .await;
    assert_ne!(prism_resp["result"]["isError"], json!(true));
    let prism_feat_id = parse_content(&prism_resp)["feature_id"]
        .as_str()
        .unwrap()
        .to_string();

    let hole_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2304,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": "nut_m10",
                    "position": [0.0, 0.0, 0.0],
                    "normal": [0.0, 0.0, -1.0],
                    "radius": 0.00425,
                    "through": true
                }
            }
        }),
    )
    .await;
    assert_ne!(hole_resp["result"]["isError"], json!(true));

    let groove_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2305,
            "method": "tools/call",
            "params": {
                "name": "cut_helical_groove",
                "arguments": {
                    "part_id": "nut_m10",
                    "helix_radius": 0.005,
                    "groove_radius": 0.00075,
                    "pitch": 0.0015,
                    "height": 0.008,
                    "starts": 1,
                    "profile_type": "metric_60"
                }
            }
        }),
    )
    .await;
    assert_ne!(
        groove_resp["result"]["isError"],
        json!(true),
        "cut_helical_groove failed: {groove_resp}"
    );
    let groove_json = parse_content(&groove_resp);
    let start_z = groove_json["start_z"].as_f64().unwrap();
    assert!(
        (start_z - (-0.004)).abs() < 1e-6,
        "Expected auto-aligned start_z == -0.004 m, got {start_z}"
    );

    // 3. Snapshot mesh volume and feature tree before attempting invalid modify_feature calls
    let val_before = parse_content(
        &rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2306,
                "method": "tools/call",
                "params": {
                    "name": "validate_mesh",
                    "arguments": { "part_id": "nut_m10" }
                }
            }),
        )
        .await,
    );
    let vol_before = val_before["volume_m3"].as_f64().unwrap();
    assert!(vol_before > 0.0);
    let tree_before = parse_content(
        &rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2307,
                "method": "tools/call",
                "params": {
                    "name": "get_feature_tree",
                    "arguments": { "part_id": "nut_m10" }
                }
            }),
        )
        .await,
    );

    // 4. Attempt invalid modify_feature (negative height) -> must return isError: true and roll back
    let bad_mod_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2308,
            "method": "tools/call",
            "params": {
                "name": "modify_feature",
                "arguments": {
                    "part_id": "nut_m10",
                    "feature_id": prism_feat_id,
                    "params": {
                        "height": -0.01
                    }
                }
            }
        }),
    )
    .await;
    assert_eq!(bad_mod_resp["result"]["isError"], json!(true));

    // 5. Verify mesh volume and feature tree are completely unchanged after rollback
    let val_after = parse_content(
        &rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2309,
                "method": "tools/call",
                "params": {
                    "name": "validate_mesh",
                    "arguments": { "part_id": "nut_m10" }
                }
            }),
        )
        .await,
    );
    let vol_after = val_after["volume_m3"].as_f64().unwrap();
    assert!((vol_after - vol_before).abs() < 1e-12);

    let tree_after = parse_content(
        &rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2310,
                "method": "tools/call",
                "params": {
                    "name": "get_feature_tree",
                    "arguments": { "part_id": "nut_m10" }
                }
            }),
        )
        .await,
    );
    assert_eq!(tree_after["dag"], tree_before["dag"]);
}

/// P2-T21: `modify_feature` on `AddWheelRim` forwards `wheel_rim_diameter_m` to `check_scale_sanity`
/// (`WHEEL_RIM_SCALE` warning when `rim_diameter < 150 mm`), and `get_scene_summary` outputs joint
/// `limits` (in `deg` for `revolute` and in active `length_unit` for `prismatic`), `limits_unit`,
/// `axis`, and `ratio` in both JSON `joints` and `summary_text`.
#[tokio::test]
async fn test_mcp_p2_t21_modify_feature_wheel_rim_scale_warning_and_scene_summary_limits() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create wheel rim part (400 mm diameter -> no WHEEL_RIM_SCALE warning)
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2401,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "rim_a",
                    "name": "Wheel Rim A",
                    "material": "aluminum_6061_t6"
                }
            }
        }),
    )
    .await;

    let add_rim = parse_content(
        &rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2402,
                "method": "tools/call",
                "params": {
                    "name": "add_feature_primitive",
                    "arguments": {
                        "part_id": "rim_a",
                        "primitive_type": "wheel_rim",
                        "rim_diameter": 400.0,
                        "rim_width": 200.0,
                        "hub_diameter": 120.0,
                        "spokes": 5
                    }
                }
            }),
        )
        .await,
    );
    let rim_feat_id = add_rim["feature_id"].as_str().unwrap().to_string();
    assert!(
        !add_rim["warnings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|w| w.as_str().is_some_and(|s| s.contains("WHEEL_RIM_SCALE")))
    );

    // 2. Modify wheel rim diameter down to 120 mm (< 150 mm) -> must trigger WHEEL_RIM_SCALE
    let mod_rim = parse_content(
        &rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2403,
                "method": "tools/call",
                "params": {
                    "name": "modify_feature",
                    "arguments": {
                        "part_id": "rim_a",
                        "feature_id": rim_feat_id,
                        "parameters": {
                            "rim_diameter": 120.0,
                            "rim_width": 60.0,
                            "hub_diameter": 40.0
                        }
                    }
                }
            }),
        )
        .await,
    );
    let warnings = mod_rim["warnings"].as_array().unwrap();
    assert!(
        warnings
            .iter()
            .any(|w| w.as_str().is_some_and(|s| s.contains("WHEEL_RIM_SCALE"))),
        "Expected WHEEL_RIM_SCALE warning when modifying rim_diameter to 120 mm, got: {warnings:?}"
    );

    // 3. Create second part and attach Revolute + Prismatic joints with limits
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2404,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "hub_b",
                    "name": "Hub Carrier B",
                    "material": "steel_4140",
                    "is_fixed": true
                }
            }
        }),
    )
    .await;
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2405,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "hub_b",
                    "primitive_type": "box",
                    "width": 100.0,
                    "height": 100.0,
                    "depth": 100.0
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2406,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_rev",
                    "part_a": "hub_b",
                    "part_b": "rim_a",
                    "joint_type": "revolute",
                    "anchor_a": [0.0, 0.0, 50.0],
                    "anchor_b": [0.0, 0.0, 0.0],
                    "axis": [0.0, 0.0, 1.0],
                    "limits": [-30.0, 60.0]
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2407,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_prism",
                    "part_a": "hub_b",
                    "part_b": "rim_a",
                    "joint_type": "prismatic",
                    "anchor_a": [10.0, 0.0, 0.0],
                    "anchor_b": [0.0, 0.0, 0.0],
                    "axis": [1.0, 0.0, 0.0],
                    "limits": [-15.0, 45.0]
                }
            }
        }),
    )
    .await;

    // 4. Inspect get_scene_summary in mm (default) and m (per-call override)
    let summary_mm = parse_content(
        &rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2408,
                "method": "tools/call",
                "params": {
                    "name": "get_scene_summary",
                    "arguments": {}
                }
            }),
        )
        .await,
    );
    let joints_mm = summary_mm["joints"].as_array().unwrap();
    let j_rev_mm = joints_mm.iter().find(|j| j["joint_id"] == "j_rev").unwrap();
    assert_eq!(j_rev_mm["limits_unit"], "deg");
    let rev_lim = j_rev_mm["limits"].as_array().unwrap();
    assert!((rev_lim[0].as_f64().unwrap() - (-30.0)).abs() < 1e-6);
    assert!((rev_lim[1].as_f64().unwrap() - 60.0).abs() < 1e-6);

    let j_prism_mm = joints_mm
        .iter()
        .find(|j| j["joint_id"] == "j_prism")
        .unwrap();
    assert_eq!(j_prism_mm["limits_unit"], "mm");
    let prism_lim_mm = j_prism_mm["limits"].as_array().unwrap();
    assert!((prism_lim_mm[0].as_f64().unwrap() - (-15.0)).abs() < 1e-6);
    assert!((prism_lim_mm[1].as_f64().unwrap() - 45.0).abs() < 1e-6);

    let text_mm = summary_mm["summary_text"].as_str().unwrap();
    assert!(
        text_mm.contains("limits=[-30.00, 60.00] deg"),
        "summary_text in mm should include revolute limits in deg, got:\n{text_mm}"
    );
    assert!(
        text_mm.contains("limits=[-15.00, 45.00] mm"),
        "summary_text in mm should include prismatic limits in mm, got:\n{text_mm}"
    );

    let summary_m = parse_content(
        &rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2409,
                "method": "tools/call",
                "params": {
                    "name": "get_scene_summary",
                    "arguments": { "length_unit": "m" }
                }
            }),
        )
        .await,
    );
    let joints_m = summary_m["joints"].as_array().unwrap();
    let j_prism_m = joints_m
        .iter()
        .find(|j| j["joint_id"] == "j_prism")
        .unwrap();
    assert_eq!(j_prism_m["limits_unit"], "m");
    let prism_lim_m = j_prism_m["limits"].as_array().unwrap();
    assert!((prism_lim_m[0].as_f64().unwrap() - (-0.015)).abs() < 1e-8);
    assert!((prism_lim_m[1].as_f64().unwrap() - 0.045).abs() < 1e-8);
    let text_m = summary_m["summary_text"].as_str().unwrap();
    assert!(
        text_m.contains("limits=[-0.0150, 0.0450] m"),
        "summary_text in m should include prismatic limits in m, got:\n{text_m}"
    );
}

#[tokio::test]
async fn test_mcp_p2_t22_gear_joint_and_simulation_step_validation() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create two test parts
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2501,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Gear Driver",
                    "part_id": "g_driver",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2502,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "g_driver",
                    "primitive_type": "cylinder",
                    "params": { "radius": 25.0, "height": 10.0 }
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2503,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Gear Driven",
                    "part_id": "g_driven",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2504,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "g_driven",
                    "primitive_type": "cylinder",
                    "params": { "radius": 50.0, "height": 10.0 }
                }
            }
        }),
    )
    .await;

    // 2. Validate gear ratio == 0.0 returns INVALID_ARGUMENT with param "ratio"
    let resp_zero_ratio = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2505,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_gear_bad_ratio",
                    "joint_type": "gear",
                    "part_a": "g_driver",
                    "part_b": "g_driven",
                    "ratio": 0.0
                }
            }
        }),
    )
    .await;
    let err_ratio = parse_error_envelope(&resp_zero_ratio);
    assert_eq!(err_ratio.error.code, "INVALID_ARGUMENT");
    assert_eq!(err_ratio.error.param.as_deref(), Some("ratio"));

    // 3. Validate teeth_a <= 0 returns INVALID_ARGUMENT with param "teeth_a"
    let resp_teeth_a = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2506,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_gear_bad_teeth_a",
                    "joint_type": "gear",
                    "part_a": "g_driver",
                    "part_b": "g_driven",
                    "teeth_a": 0,
                    "teeth_b": 40
                }
            }
        }),
    )
    .await;
    let err_teeth_a = parse_error_envelope(&resp_teeth_a);
    assert_eq!(err_teeth_a.error.code, "INVALID_ARGUMENT");
    assert_eq!(err_teeth_a.error.param.as_deref(), Some("teeth_a"));

    // 4. Validate teeth_b <= 0 returns INVALID_ARGUMENT with param "teeth_b"
    let resp_teeth_b = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2507,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_gear_bad_teeth_b",
                    "joint_type": "gear",
                    "part_a": "g_driver",
                    "part_b": "g_driven",
                    "teeth_a": 20,
                    "teeth_b": -5
                }
            }
        }),
    )
    .await;
    let err_teeth_b = parse_error_envelope(&resp_teeth_b);
    assert_eq!(err_teeth_b.error.code, "INVALID_ARGUMENT");
    assert_eq!(err_teeth_b.error.param.as_deref(), Some("teeth_b"));

    // 5. Validate zero vector axis_a returns INVALID_ARGUMENT with param "axis_a"
    let resp_axis_a = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2508,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_gear_bad_axis_a",
                    "joint_type": "gear",
                    "part_a": "g_driver",
                    "part_b": "g_driven",
                    "ratio": 2.0,
                    "axis_a": [0.0, 0.0, 0.0]
                }
            }
        }),
    )
    .await;
    let err_axis_a = parse_error_envelope(&resp_axis_a);
    assert_eq!(err_axis_a.error.code, "INVALID_ARGUMENT");
    assert_eq!(err_axis_a.error.param.as_deref(), Some("axis_a"));

    // 6. Validate zero vector axis_b returns INVALID_ARGUMENT with param "axis_b"
    let resp_axis_b = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2509,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_gear_bad_axis_b",
                    "joint_type": "gear",
                    "part_a": "g_driver",
                    "part_b": "g_driven",
                    "ratio": 2.0,
                    "axis_b": [0.0, 0.0, 0.0]
                }
            }
        }),
    )
    .await;
    let err_axis_b = parse_error_envelope(&resp_axis_b);
    assert_eq!(err_axis_b.error.code, "INVALID_ARGUMENT");
    assert_eq!(err_axis_b.error.param.as_deref(), Some("axis_b"));

    // 7. Valid gear joint creation with teeth_a and teeth_b
    let resp_valid_gear = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2510,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_gear_valid",
                    "joint_type": "gear",
                    "part_a": "g_driver",
                    "part_b": "g_driven",
                    "teeth_a": 20,
                    "teeth_b": 40
                }
            }
        }),
    )
    .await;
    assert_ne!(resp_valid_gear["result"]["isError"], json!(true));
    let valid_joint_json = parse_content(&resp_valid_gear);
    assert_eq!(valid_joint_json["joint_id"], "j_gear_valid");

    // 8. Validate step_size_ms <= 0 in run_physics_simulation
    let resp_sim_bad_step = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2511,
            "method": "tools/call",
            "params": {
                "name": "run_physics_simulation",
                "arguments": {
                    "duration_seconds": 0.1,
                    "step_size_ms": 0.0
                }
            }
        }),
    )
    .await;
    let err_sim_step = parse_error_envelope(&resp_sim_bad_step);
    assert_eq!(err_sim_step.error.code, "INVALID_ARGUMENT");
    assert_eq!(err_sim_step.error.param.as_deref(), Some("step_size_ms"));

    // 9. Valid simulation with valid step_size_ms
    let resp_sim_ok = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2512,
            "method": "tools/call",
            "params": {
                "name": "run_physics_simulation",
                "arguments": {
                    "duration_seconds": 0.05,
                    "step_size_ms": 10.0
                }
            }
        }),
    )
    .await;
    assert_ne!(resp_sim_ok["result"]["isError"], json!(true));
    let sim_ok_json = parse_content(&resp_sim_ok);
    assert!(sim_ok_json["steps_simulated"].as_u64().unwrap() > 0);
}

#[tokio::test]
async fn test_mcp_p2_t23_reserved_ids_and_ground_normalization() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Rejection of create_part with reserved IDs (case-insensitive)
    for res_id in &["ground", "world", "GROUND", "World"] {
        let resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2601,
                "method": "tools/call",
                "params": {
                    "name": "create_part",
                    "arguments": {
                        "part_id": res_id,
                        "name": "My Anchor",
                        "material": "steel_structural_aisi1020"
                    }
                }
            }),
        )
        .await;
        let err = parse_error_envelope(&resp);
        assert_eq!(err.error.code, "INVALID_ARGUMENT");
        assert_eq!(err.error.param.as_deref(), Some("part_id"));
        assert!(err.error.hint.as_deref().unwrap().contains("reserved static anchors"));
    }

    // 2. Rejection of delete_part with reserved IDs
    for res_id in &["ground", "world", "GROUND", "World"] {
        let resp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2602,
                "method": "tools/call",
                "params": {
                    "name": "delete_part",
                    "arguments": {
                        "part_id": res_id
                    }
                }
            }),
        )
        .await;
        let err = parse_error_envelope(&resp);
        assert_eq!(err.error.code, "INVALID_ARGUMENT");
        assert_eq!(err.error.param.as_deref(), Some("part_id"));
        assert!(err.error.hint.as_deref().unwrap().contains("cannot be deleted"));
    }

    // 3. Create a valid part and add geometry
    let resp_create = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2603,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "pendulum_arm",
                    "name": "Pendulum Arm",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;
    assert_ne!(resp_create["result"]["isError"], json!(true));

    let resp_geom = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2604,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "pendulum_arm",
                    "primitive_type": "box",
                    "params": {
                        "width": 10.0,
                        "height": 50.0,
                        "depth": 10.0
                    }
                }
            }
        }),
    )
    .await;
    assert_ne!(resp_geom["result"]["isError"], json!(true));

    // 4. create_joint with uppercase "GROUND" as part_a
    let resp_joint_ground = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2605,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_revolute_ground",
                    "joint_type": "revolute",
                    "part_a": "GROUND",
                    "part_b": "pendulum_arm",
                    "axis": [0.0, 0.0, 1.0]
                }
            }
        }),
    )
    .await;
    assert_ne!(resp_joint_ground["result"]["isError"], json!(true), "Connecting to 'GROUND' must succeed");
    let j_ground_res = parse_content(&resp_joint_ground);
    assert_eq!(j_ground_res["joint_id"], "j_revolute_ground");

    // 5. create_joint with mixed-case "World" as part_b
    let resp_create2 = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2606,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "slider_block",
                    "name": "Slider Block",
                    "material": "aluminum_6061_t6"
                }
            }
        }),
    )
    .await;
    assert_ne!(resp_create2["result"]["isError"], json!(true));

    let resp_geom2 = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2607,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "slider_block",
                    "primitive_type": "box",
                    "params": {
                        "width": 20.0,
                        "height": 20.0,
                        "depth": 20.0
                    }
                }
            }
        }),
    )
    .await;
    assert_ne!(resp_geom2["result"]["isError"], json!(true));

    let resp_joint_world = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2608,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_prismatic_world",
                    "joint_type": "prismatic",
                    "part_a": "slider_block",
                    "part_b": "World",
                    "axis": [1.0, 0.0, 0.0]
                }
            }
        }),
    )
    .await;
    assert_ne!(resp_joint_world["result"]["isError"], json!(true), "Connecting to 'World' must succeed");
    let j_world_res = parse_content(&resp_joint_world);
    assert_eq!(j_world_res["joint_id"], "j_prismatic_world");

    // 6. Verify get_scene_summary shows normalized joints and correct joint limits/types
    let resp_summary = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2609,
            "method": "tools/call",
            "params": {
                "name": "get_scene_summary",
                "arguments": {}
            }
        }),
    )
    .await;
    assert_ne!(resp_summary["result"]["isError"], json!(true));
    let summary_json = parse_content(&resp_summary);
    let joints_arr = summary_json["joints"].as_array().unwrap();
    assert_eq!(joints_arr.len(), 2);
    let j0 = joints_arr.iter().find(|j| j["joint_id"] == "j_revolute_ground").unwrap();
    assert_eq!(j0["part_a"], "ground");
    let j1 = joints_arr.iter().find(|j| j["joint_id"] == "j_prismatic_world").unwrap();
    assert_eq!(j1["part_b"], "world");
}

#[tokio::test]
async fn test_mcp_p2_t24_export_stl_and_tessellate_part_do_not_stale_fea() {
    let state = EngineState::new().unwrap().into_shared();

    // 1. Create part and add a cylinder feature (evaluated at default "medium" tessellation -> revision = 1)
    let create_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2701,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "pin_p2t24",
                    "name": "Load Pin",
                    "material": "steel_4140"
                }
            }
        }),
    )
    .await;
    assert_ne!(create_resp["result"]["isError"], json!(true));

    let add_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2702,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "pin_p2t24",
                    "primitive_type": "cylinder",
                    "params": {
                        "radius": 15.0,
                        "height": 80.0
                    }
                }
            }
        }),
    )
    .await;
    assert_ne!(add_resp["result"]["isError"], json!(true));
    let add_json = parse_content(&add_resp);
    let feat_id = add_json["feature_id"].as_str().unwrap().to_string();
    assert_eq!(add_json["revision"], 1);

    // 2. Declare min_safety_factor requirement with auto_check: true
    let req_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2703,
            "method": "tools/call",
            "params": {
                "name": "set_requirements",
                "arguments": {
                    "part_id": "pin_p2t24",
                    "auto_check": true,
                    "requirements": [
                        {
                            "id": "req_fos",
                            "type": "min_safety_factor",
                            "value": 1.5
                        },
                        {
                            "id": "req_mesh",
                            "type": "mesh_valid"
                        }
                    ]
                }
            }
        }),
    )
    .await;
    assert_ne!(req_resp["result"]["isError"], json!(true));

    // 3. Run FEA analysis -> records fea_revision = 1
    let fea_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2704,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": "pin_p2t24",
                    "mesh_density": "coarse",
                    "fixed": [{"face": "cap_bottom"}],
                    "loads": [{"face": "cap_top", "force": [0.0, 0.0, 500.0]}]
                }
            }
        }),
    )
    .await;
    assert_ne!(fea_resp["result"]["isError"], json!(true));
    let fea_json = parse_content(&fea_resp);
    assert_eq!(fea_json["part_revision"], 1);

    // 4. Export STL (default quality = "fine", which re-tessellates the cached "medium" mesh)
    //    and call tessellate_part ("coarse", then "fine").
    let tmp_dir = std::env::temp_dir().join(format!("mcpcad_p2t24_contract_{}", uuid::Uuid::new_v4().simple()));
    let stl_path = tmp_dir.join("pin_p2t24.stl");
    let stl_path_str = stl_path.to_str().unwrap().to_string();

    let exp_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2705,
            "method": "tools/call",
            "params": {
                "name": "export_stl",
                "arguments": {
                    "part_id": "pin_p2t24",
                    "filepath": stl_path_str
                }
            }
        }),
    )
    .await;
    assert_ne!(exp_resp["result"]["isError"], json!(true));
    let exp_json = parse_content(&exp_resp);
    assert_eq!(exp_json["quality_applied"], "fine");
    assert_eq!(exp_json["revision"], 1, "export_stl must not increment part.revision");
    let _ = std::fs::remove_dir_all(&tmp_dir);

    let tess_coarse_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2706,
            "method": "tools/call",
            "params": {
                "name": "tessellate_part",
                "arguments": {
                    "part_id": "pin_p2t24",
                    "quality": "coarse"
                }
            }
        }),
    )
    .await;
    assert_ne!(tess_coarse_resp["result"]["isError"], json!(true));
    assert_eq!(parse_content(&tess_coarse_resp)["revision"], 1);

    let tess_fine_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2707,
            "method": "tools/call",
            "params": {
                "name": "tessellate_part",
                "arguments": {
                    "part_id": "pin_p2t24",
                    "quality": "fine"
                }
            }
        }),
    )
    .await;
    assert_ne!(tess_fine_resp["result"]["isError"], json!(true));
    assert_eq!(parse_content(&tess_fine_resp)["revision"], 1);

    // 5. Verify check_requirements and get_scene_summary STILL report PASS / FRESH (not STALE!)
    let check_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2708,
            "method": "tools/call",
            "params": {
                "name": "check_requirements",
                "arguments": {
                    "part_id": "pin_p2t24"
                }
            }
        }),
    )
    .await;
    assert_ne!(check_resp["result"]["isError"], json!(true));
    let check_json = parse_content(&check_resp);
    assert_eq!(
        check_json["status"], "PASS",
        "min_safety_factor must remain PASS after export_stl and tessellate_part, got: {check_json}"
    );
    assert_eq!(check_json["stale"], 0);
    assert_eq!(check_json["passed"], 2);

    let sum_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2709,
            "method": "tools/call",
            "params": {
                "name": "get_scene_summary",
                "arguments": {}
            }
        }),
    )
    .await;
    assert_ne!(sum_resp["result"]["isError"], json!(true));
    let sum_json = parse_content(&sum_resp);
    assert_eq!(sum_json["parts"][0]["revision"], 1);
    assert_eq!(sum_json["parts"][0]["last_fea"]["stale"], false);
    assert_eq!(sum_json["parts"][0]["last_fea"]["status"], "FRESH");
    assert_eq!(sum_json["requirements"]["status"], "PASS");
    assert_eq!(sum_json["requirements"]["stale"], 0);

    // 6. Now modify parametric geometry -> revision increments to 2 and FEA becomes STALE
    let mod_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2710,
            "method": "tools/call",
            "params": {
                "name": "modify_feature",
                "arguments": {
                    "part_id": "pin_p2t24",
                    "feature_id": feat_id,
                    "params": {
                        "radius": 18.0,
                        "height": 80.0
                    }
                }
            }
        }),
    )
    .await;
    assert_ne!(mod_resp["result"]["isError"], json!(true));
    let mod_json = parse_content(&mod_resp);
    assert_eq!(mod_json["revision"], 2);
    assert_eq!(mod_json["requirements_status"]["status"], "STALE");
    assert_eq!(mod_json["requirements_status"]["stale"], 1);
}

#[tokio::test]
async fn test_mcp_p2_t26_boolean_ops_metadata_and_load_project_joint_details() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create target and tool parts
    for (part_id, width, pos_x) in [("base_p2t26", 20.0, 0.0), ("cutter_p2t26", 10.0, 10.0)] {
        rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2801,
                "method": "tools/call",
                "params": {
                    "name": "create_part",
                    "arguments": {
                        "part_id": part_id,
                        "name": part_id,
                        "material": "steel_structural_aisi1020"
                    }
                }
            }),
        )
        .await;

        rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": 2802,
                "method": "tools/call",
                "params": {
                    "name": "add_feature_primitive",
                    "arguments": {
                        "part_id": part_id,
                        "primitive_type": "box",
                        "params": {
                            "width": width,
                            "height": 10.0,
                            "depth": 10.0,
                            "position": [pos_x, 0.0, 0.0]
                        }
                    }
                }
            }),
        )
        .await;
    }

    // 2. Call boolean_difference with per-call length_unit="m"
    // base_p2t26 spans [-10..10] x [-5..5] x [-5..5] (2000 mm^3), cutter_p2t26 spans [5..15] x [-5..5] x [-5..5] (1000 mm^3)
    // Overlap is [5..10] x [-5..5] x [-5..5] = 500 mm^3 -> difference is 1500 mm^3 = 1.5e-6 m^3
    let diff_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2803,
            "method": "tools/call",
            "params": {
                "name": "boolean_difference",
                "arguments": {
                    "target_part_id": "base_p2t26",
                    "tool_part_id": "cutter_p2t26",
                    "length_unit": "m"
                }
            }
        }),
    )
    .await;
    assert_ne!(diff_resp["result"]["isError"], json!(true));
    let diff_json = parse_content(&diff_resp);
    assert_eq!(diff_json["part_id"], "base_p2t26");
    assert_eq!(diff_json["target_part_id"], "base_p2t26");
    assert_eq!(diff_json["revision"], 2);
    assert_eq!(diff_json["length_unit"], "m");
    assert_eq!(diff_json["units"]["volume"], "m³");
    assert!(diff_json["vertices_count"].as_u64().unwrap() > 0);
    assert!(diff_json["triangles_count"].as_u64().unwrap() > 0);
    let diff_vol_m3 = diff_json["volume"].as_f64().unwrap();
    assert!((diff_vol_m3 - 1.5e-6).abs() < 1e-7, "Expected 1.5e-6 m^3, got {diff_vol_m3}");
    assert!((diff_json["volume_m3"].as_f64().unwrap() - 1.5e-6).abs() < 1e-7);
    let diff_dims_m = diff_json["bounding_box"]["dimensions"].as_array().unwrap();
    assert!((diff_dims_m[0].as_f64().unwrap() - 0.015).abs() < 1e-4);
    assert!(diff_json["warnings"].as_array().unwrap().is_empty());

    // 3. Call boolean_union in default session unit ("mm") -> [-10..15] x [-5..5] x [-5..5] = 2500 mm^3
    let union_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2804,
            "method": "tools/call",
            "params": {
                "name": "boolean_union",
                "arguments": {
                    "target_part_id": "base_p2t26",
                    "tool_part_id": "cutter_p2t26"
                }
            }
        }),
    )
    .await;
    assert_ne!(union_resp["result"]["isError"], json!(true));
    let union_json = parse_content(&union_resp);
    assert_eq!(union_json["part_id"], "base_p2t26");
    assert_eq!(union_json["target_part_id"], "base_p2t26");
    assert_eq!(union_json["revision"], 3);
    assert_eq!(union_json["length_unit"], "mm");
    assert_eq!(union_json["units"]["volume"], "mm³");
    let union_vol_mm3 = union_json["volume"].as_f64().unwrap();
    assert!((union_vol_mm3 - 2500.0).abs() < 5.0, "Expected 2500 mm^3, got {union_vol_mm3}");
    let union_dims_mm = union_json["bounding_box"]["dimensions"].as_array().unwrap();
    assert!((union_dims_mm[0].as_f64().unwrap() - 25.0).abs() < 0.1);
    assert!(union_json["warnings"].as_array().unwrap().is_empty());

    // 4. Call boolean_intersection -> overlap with cutter_p2t26 is [5..15] x [-5..5] x [-5..5] = 1000 mm^3
    let inter_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2805,
            "method": "tools/call",
            "params": {
                "name": "boolean_intersection",
                "arguments": {
                    "target_part_id": "base_p2t26",
                    "tool_part_id": "cutter_p2t26"
                }
            }
        }),
    )
    .await;
    assert_ne!(inter_resp["result"]["isError"], json!(true));
    let inter_json = parse_content(&inter_resp);
    assert_eq!(inter_json["part_id"], "base_p2t26");
    assert_eq!(inter_json["target_part_id"], "base_p2t26");
    assert_eq!(inter_json["revision"], 4);
    assert_eq!(inter_json["length_unit"], "mm");
    let inter_vol_mm3 = inter_json["volume"].as_f64().unwrap();
    assert!((inter_vol_mm3 - 1000.0).abs() < 5.0, "Expected 1000 mm^3, got {inter_vol_mm3}");
    assert!(inter_json["warnings"].as_array().unwrap().is_empty());

    // 5. Create revolute joint with limits [-30, 60] deg, prismatic joint with limits [-5, 15] mm, and gear joint with ratio -2.0
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2806,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_rev_p2t26",
                    "joint_type": "revolute",
                    "part_a": "ground",
                    "part_b": "base_p2t26",
                    "anchor_a": [0.0, 0.0, 0.0],
                    "anchor_b": [5.0, 0.0, 0.0],
                    "axis": [0.0, 0.0, 1.0],
                    "limits": [-30.0, 60.0]
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2807,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_prism_p2t26",
                    "joint_type": "prismatic",
                    "part_a": "base_p2t26",
                    "part_b": "cutter_p2t26",
                    "anchor_a": [10.0, 0.0, 0.0],
                    "anchor_b": [0.0, 0.0, 0.0],
                    "axis": [1.0, 0.0, 0.0],
                    "limits": [-5.0, 15.0]
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2808,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_gear_p2t26",
                    "joint_type": "gear",
                    "part_a": "base_p2t26",
                    "part_b": "cutter_p2t26",
                    "axis": [0.0, 0.0, 1.0],
                    "ratio": -2.0
                }
            }
        }),
    )
    .await;

    // 6. Save and reload project, then verify load_project returns limits, limits_unit, axis, and ratio
    let tmp_dir = std::env::temp_dir().join(format!("mcpcad_p2t26_contract_{}", uuid::Uuid::new_v4()));
    let proj_path = tmp_dir.join("p2_t26_project.json");
    let proj_path_str = proj_path.to_str().unwrap();

    let save_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2809,
            "method": "tools/call",
            "params": {
                "name": "save_project",
                "arguments": {
                    "path": proj_path_str,
                    "project_name": "P2-T26 Contract Test"
                }
            }
        }),
    )
    .await;
    assert_ne!(save_resp["result"]["isError"], json!(true));

    let load_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2810,
            "method": "tools/call",
            "params": {
                "name": "load_project",
                "arguments": {
                    "path": proj_path_str
                }
            }
        }),
    )
    .await;
    let _ = std::fs::remove_dir_all(&tmp_dir);
    assert_ne!(load_resp["result"]["isError"], json!(true));
    let load_json = parse_content(&load_resp);
    let joints = load_json["joints"].as_array().expect("joints array in load_project");
    assert_eq!(joints.len(), 3);

    // Sorted by joint_id: j_gear_p2t26, j_prism_p2t26, j_rev_p2t26
    assert_eq!(joints[0]["joint_id"], "j_gear_p2t26");
    assert_eq!(joints[0]["joint_type"], "gear");
    assert_eq!(joints[0]["axis"], json!([0.0, 0.0, 1.0]));
    assert_eq!(joints[0]["ratio"], -2.0);
    assert!(joints[0]["limits"].is_null());

    assert_eq!(joints[1]["joint_id"], "j_prism_p2t26");
    assert_eq!(joints[1]["joint_type"], "prismatic");
    assert_eq!(joints[1]["axis"], json!([1.0, 0.0, 0.0]));
    assert_eq!(joints[1]["limits_unit"], "mm");
    let prism_lim = joints[1]["limits"].as_array().unwrap();
    assert!((prism_lim[0].as_f64().unwrap() - (-5.0)).abs() < 1e-6);
    assert!((prism_lim[1].as_f64().unwrap() - 15.0).abs() < 1e-6);

    assert_eq!(joints[2]["joint_id"], "j_rev_p2t26");
    assert_eq!(joints[2]["joint_type"], "revolute");
    assert_eq!(joints[2]["axis"], json!([0.0, 0.0, 1.0]));
    assert_eq!(joints[2]["limits_unit"], "deg");
    let rev_lim = joints[2]["limits"].as_array().unwrap();
    assert!((rev_lim[0].as_f64().unwrap() - (-30.0)).abs() < 1e-6);
    assert!((rev_lim[1].as_f64().unwrap() - 60.0).abs() < 1e-6);
}

#[tokio::test]
async fn test_mcp_p2_t27_simulation_metrics_fea_staleness_and_optimize_feature_tree_revision() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let parse_content = |resp: &Value| -> Value {
        let text = resp["result"]["content"][0]["text"]
            .as_str()
            .expect("Expected content[0].text string");
        serde_json::from_str(text).expect("Expected valid JSON in content[0].text")
    };

    // 1. Create a part and add a 100 x 20 x 20 mm box (revision = 1)
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2901,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "beam_p2t27",
                    "name": "Beam P2-T27",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2902,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "beam_p2t27",
                    "primitive_type": "box",
                    "params": { "width": 100.0, "height": 20.0, "depth": 20.0 }
                }
            }
        }),
    )
    .await;

    // 2. Declare a min_safety_factor requirement and run FEA analysis
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2903,
            "method": "tools/call",
            "params": {
                "name": "set_requirements",
                "arguments": {
                    "part_id": "beam_p2t27",
                    "auto_check": true,
                    "requirements": [
                        { "id": "req_fos", "type": "min_safety_factor", "value": 1.2 }
                    ]
                }
            }
        }),
    )
    .await;

    let fea_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2904,
            "method": "tools/call",
            "params": {
                "name": "run_fea_analysis",
                "arguments": {
                    "part_id": "beam_p2t27",
                    "fixed_faces": ["min_x"],
                    "applied_forces": [
                        { "face": "max_x", "force_vector": [0.0, -200.0, 0.0] }
                    ],
                    "mesh_density": "coarse"
                }
            }
        }),
    )
    .await;
    assert_ne!(fea_resp["result"]["isError"], json!(true));

    // 3. Call optimize_feature_tree(apply: true, length_unit: "m") when no rules apply:
    //    must keep revision == 1 and keep FEA fresh (requirements_status.status == "PASS", stale == 0)
    let opt_noop_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2905,
            "method": "tools/call",
            "params": {
                "name": "optimize_feature_tree",
                "arguments": {
                    "part_id": "beam_p2t27",
                    "apply": true,
                    "length_unit": "m"
                }
            }
        }),
    )
    .await;
    assert_ne!(opt_noop_resp["result"]["isError"], json!(true));
    let opt_noop = parse_content(&opt_noop_resp);
    assert!(opt_noop["applied_rules"].as_array().unwrap().is_empty());
    assert_eq!(opt_noop["revision"], 1);
    assert_eq!(opt_noop["length_unit"], "m");
    assert_eq!(opt_noop["units"]["volume"], "m³");
    assert!((opt_noop["volume"].as_f64().unwrap() - 4.0e-5).abs() < 1e-8);
    assert_eq!(opt_noop["requirements_status"]["status"], "PASS");
    assert_eq!(opt_noop["requirements_status"]["stale"], 0);

    // 4. Verify get_simulation_metrics_json with length_unit override ("m") and fresh FEA fields
    let metrics_m_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2906,
            "method": "tools/call",
            "params": {
                "name": "get_simulation_metrics_json",
                "arguments": {
                    "length_unit": "m"
                }
            }
        }),
    )
    .await;
    assert_ne!(metrics_m_resp["result"]["isError"], json!(true));
    let metrics_m = parse_content(&metrics_m_resp);
    assert_eq!(metrics_m["length_unit"], "m");
    assert_eq!(metrics_m["current_metrics"]["length_unit"], "m");
    let part_m = &metrics_m["current_metrics"]["parts"][0];
    assert!((part_m["bounding_box_dimensions"][0].as_f64().unwrap() - 0.1).abs() < 1e-6);
    assert!((part_m["volume"].as_f64().unwrap() - 4.0e-5).abs() < 1e-8);
    let fea_m = &metrics_m["current_metrics"]["fea_reports"][0];
    assert_eq!(fea_m["part_id"], "beam_p2t27");
    assert_eq!(fea_m["fea_revision"], 1);
    assert_eq!(fea_m["part_revision"], 1);
    assert_eq!(fea_m["stale"], false);
    assert_eq!(fea_m["is_stale"], false);
    assert_eq!(fea_m["status"], "FRESH");

    // 5. Modify geometry (transform_part) -> revision bumps to 2, FEA becomes STALE in get_simulation_metrics_json
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2907,
            "method": "tools/call",
            "params": {
                "name": "transform_part",
                "arguments": {
                    "part_id": "beam_p2t27",
                    "translation": [5.0, 0.0, 0.0]
                }
            }
        }),
    )
    .await;

    let metrics_mm_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 2908,
            "method": "tools/call",
            "params": {
                "name": "get_simulation_metrics_json",
                "arguments": {}
            }
        }),
    )
    .await;
    assert_ne!(metrics_mm_resp["result"]["isError"], json!(true));
    let metrics_mm = parse_content(&metrics_mm_resp);
    assert_eq!(metrics_mm["length_unit"], "mm");
    let fea_mm = &metrics_mm["current_metrics"]["fea_reports"][0];
    assert_eq!(fea_mm["fea_revision"], 1);
    assert_eq!(fea_mm["part_revision"], 2);
    assert_eq!(fea_mm["stale"], true);
    assert_eq!(fea_mm["is_stale"], true);
    assert_eq!(fea_mm["status"], "STALE");
}

#[tokio::test]
async fn test_mcp_p2_t29_list_parts_length_unit_override_and_scene_summary_gear_axes() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let parse_content = |resp: &Value| -> Value {
        let text = resp["result"]["content"][0]["text"]
            .as_str()
            .expect("Expected content[0].text string");
        serde_json::from_str(text).expect("Expected valid JSON in content[0].text")
    };

    // 1. Create two gear parts (`gear_pinion` and `gear_wheel`) in default session unit ("mm")
    for (req_id, pid, name, pos_x, r_mm) in [
        (3001, "gear_pinion", "Pinion Gear", 0.0, 20.0),
        (3002, "gear_wheel", "Wheel Gear", 60.0, 40.0),
    ] {
        let cp = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": req_id,
                "method": "tools/call",
                "params": {
                    "name": "create_part",
                    "arguments": {
                        "part_id": pid,
                        "name": name,
                        "material": "steel_structural_aisi1020"
                    }
                }
            }),
        )
        .await;
        assert_ne!(cp["result"]["isError"], json!(true));

        let add = rpc_call(
            &state,
            json!({
                "jsonrpc": "2.0",
                "id": req_id + 10,
                "method": "tools/call",
                "params": {
                    "name": "add_feature_primitive",
                    "arguments": {
                        "part_id": pid,
                        "primitive_type": "cylinder",
                        "params": {
                            "radius": r_mm,
                            "height": 15.0,
                            "position": [pos_x, 0.0, 0.0]
                        }
                    }
                }
            }),
        )
        .await;
        assert_ne!(add["result"]["isError"], json!(true));
    }

    // 2. Call list_parts in default session unit ("mm") and with per-call override ("m")
    let list_mm_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3020,
            "method": "tools/call",
            "params": {
                "name": "list_parts",
                "arguments": {}
            }
        }),
    )
    .await;
    assert_ne!(list_mm_resp["result"]["isError"], json!(true));
    let list_mm = parse_content(&list_mm_resp);
    assert_eq!(list_mm["length_unit"], "mm");
    assert_eq!(list_mm["units"]["length"], "mm");
    assert_eq!(list_mm["units"]["volume"], "mm³");
    assert_eq!(list_mm["parts_count"], 2);
    let parts_mm = list_mm["parts"].as_array().unwrap();
    assert_eq!(parts_mm[0]["part_id"], "gear_pinion");
    assert_eq!(parts_mm[1]["part_id"], "gear_wheel");
    assert!((parts_mm[0]["bounding_box_dimensions"][0].as_f64().unwrap() - 40.0).abs() < 1e-4);
    assert!((parts_mm[1]["bounding_box_dimensions"][0].as_f64().unwrap() - 80.0).abs() < 1e-4);

    let list_m_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3021,
            "method": "tools/call",
            "params": {
                "name": "list_parts",
                "arguments": { "length_unit": "m" }
            }
        }),
    )
    .await;
    assert_ne!(list_m_resp["result"]["isError"], json!(true));
    let list_m = parse_content(&list_m_resp);
    assert_eq!(list_m["length_unit"], "m");
    assert_eq!(list_m["units"]["length"], "m");
    assert_eq!(list_m["units"]["volume"], "m³");
    assert_eq!(list_m["parts_count"], 2);
    let parts_m = list_m["parts"].as_array().unwrap();
    assert_eq!(parts_m[0]["part_id"], "gear_pinion");
    assert_eq!(parts_m[0]["length_unit"], "m");
    assert!((parts_m[0]["bounding_box_dimensions"][0].as_f64().unwrap() - 0.04).abs() < 1e-6);
    assert!((parts_m[1]["bounding_box_dimensions"][0].as_f64().unwrap() - 0.08).abs() < 1e-6);
    assert!(
        (parts_m[0]["volume"].as_f64().unwrap() - parts_m[0]["volume_m3"].as_f64().unwrap()).abs()
            < 1e-12
    );

    // Invalid length_unit in list_parts must return structured INVALID_ARGUMENT on 'length_unit'
    let list_bad_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3022,
            "method": "tools/call",
            "params": {
                "name": "list_parts",
                "arguments": { "length_unit": "ft" }
            }
        }),
    )
    .await;
    let bad_err: Value = serde_json::from_str(assert_tool_error(&list_bad_resp)).unwrap();
    assert_eq!(bad_err["error"]["code"], "INVALID_ARGUMENT");
    assert_eq!(bad_err["error"]["param"], "length_unit");

    // 3. Create a gear joint between gear_pinion and gear_wheel with distinct axis_a and axis_b
    let gear_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3023,
            "method": "tools/call",
            "params": {
                "name": "create_joint",
                "arguments": {
                    "joint_id": "j_gear_p2t29",
                    "joint_type": "gear",
                    "part_a": "gear_pinion",
                    "part_b": "gear_wheel",
                    "ratio": -0.5,
                    "axis_a": [0.0, 0.0, 1.0],
                    "axis_b": [0.0, 1.0, 0.0]
                }
            }
        }),
    )
    .await;
    assert_ne!(gear_resp["result"]["isError"], json!(true));

    // 4. Call get_scene_summary and verify gear joint exposes axis, axis_a, axis_b, and ratio
    let sum_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3024,
            "method": "tools/call",
            "params": {
                "name": "get_scene_summary",
                "arguments": {}
            }
        }),
    )
    .await;
    assert_ne!(sum_resp["result"]["isError"], json!(true));
    let sum = parse_content(&sum_resp);
    assert_eq!(sum["joints_count"], 1);
    let gj = &sum["joints"][0];
    assert_eq!(gj["joint_id"], "j_gear_p2t29");
    assert_eq!(gj["joint_type"], "gear");
    assert_eq!(gj["part_a"], "gear_pinion");
    assert_eq!(gj["part_b"], "gear_wheel");
    assert_eq!(gj["axis"], json!([0.0, 0.0, 1.0]));
    assert_eq!(gj["axis_a"], json!([0.0, 0.0, 1.0]));
    assert_eq!(gj["axis_b"], json!([0.0, 1.0, 0.0]));
    assert!((gj["ratio"].as_f64().unwrap() - (-0.5)).abs() < 1e-6);
    assert!(sum["summary_text"]
        .as_str()
        .unwrap()
        .contains("ratio=-0.500, axis_a=[0.00, 0.00, 1.00], axis_b=[0.00, 1.00, 0.00]"));
}

#[tokio::test]
async fn test_mcp_p2_t30_import_stl_contract_and_auto_check_length_unit() {
    let state = EngineState::new()
        .expect("EngineState init")
        .into_shared();

    let parse_content = |resp: &Value| -> Value {
        let text = resp["result"]["content"][0]["text"].as_str().expect("content[0].text must be string");
        serde_json::from_str(text).expect("content[0].text must be valid JSON")
    };

    // 1. Create a box part and export it to an STL file
    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3030,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "name": "Source Export Box",
                    "part_id": "box_src_p2t30",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3031,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "box_src_p2t30",
                    "primitive_type": "box",
                    "params": { "width": 80.0, "height": 40.0, "depth": 20.0 }
                }
            }
        }),
    )
    .await;

    let tmp_dir = std::env::temp_dir().join(format!("mcpcad_p2t30_contract_{}", uuid::Uuid::new_v4().simple()));
    let stl_path = tmp_dir.join("contract_box.stl");
    let exp_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3032,
            "method": "tools/call",
            "params": {
                "name": "export_stl",
                "arguments": {
                    "part_id": "box_src_p2t30",
                    "file_path": stl_path.to_str().unwrap(),
                    "binary": true
                }
            }
        }),
    )
    .await;
    assert_ne!(exp_resp["result"]["isError"], json!(true));
    assert!(stl_path.exists());

    // 2. Call import_stl via JSON-RPC using `file_path`, `part_id`, `is_fixed: true`
    let imp_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3033,
            "method": "tools/call",
            "params": {
                "name": "import_stl",
                "arguments": {
                    "file_path": stl_path.to_str().unwrap(),
                    "part_id": "imported_box_fixed",
                    "name": "Imported Fixed Plate",
                    "material": "aluminum_6061_t6",
                    "is_fixed": true
                }
            }
        }),
    )
    .await;
    assert_ne!(imp_resp["result"]["isError"], json!(true));
    let imp_data = parse_content(&imp_resp);
    assert_eq!(imp_data["part_id"], "imported_box_fixed");
    assert_eq!(imp_data["name"], "Imported Fixed Plate");
    assert_eq!(imp_data["is_fixed"], true);
    assert_eq!(imp_data["status"], "stl_imported");
    assert_eq!(imp_data["revision"], 1);
    assert_eq!(imp_data["triangles_count"], 12);
    assert_eq!(imp_data["vertices_count"], 36);
    assert_eq!(imp_data["length_unit"], "mm");
    assert_eq!(imp_data["units"]["length"], "mm");

    // Check bounding_box.dimensions, volume, volume_m3, mass_kg
    let dims = imp_data["bounding_box"]["dimensions"].as_array().unwrap();
    assert!((dims[0].as_f64().unwrap() - 80.0).abs() < 1e-4);
    assert!((dims[1].as_f64().unwrap() - 40.0).abs() < 1e-4);
    assert!((dims[2].as_f64().unwrap() - 20.0).abs() < 1e-4);
    assert!((imp_data["volume"].as_f64().unwrap() - 64_000.0).abs() < 1.0);
    assert!((imp_data["volume_m3"].as_f64().unwrap() - 6.4e-5).abs() < 1e-9);
    assert!(imp_data["mass_kg"].as_f64().unwrap() > 0.15);

    // 3. Declare requirements on box_src_p2t30 with auto_check: true
    let set_req_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3034,
            "method": "tools/call",
            "params": {
                "name": "set_requirements",
                "arguments": {
                    "part_id": "box_src_p2t30",
                    "auto_check": true,
                    "requirements": [
                        { "id": "req_box_max", "type": "bbox_max", "dims": [100.0, 50.0, 30.0] }
                    ]
                }
            }
        }),
    )
    .await;
    assert_ne!(set_req_resp["result"]["isError"], json!(true));

    // 4. Transform box_src_p2t30 with length_unit = "m" override; verify auto-checked requirements report measured in meters
    let xform_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3035,
            "method": "tools/call",
            "params": {
                "name": "transform_part",
                "arguments": {
                    "part_id": "box_src_p2t30",
                    "translation": [0.01, 0.0, 0.0],
                    "length_unit": "m"
                }
            }
        }),
    )
    .await;
    assert_ne!(xform_resp["result"]["isError"], json!(true));
    let xform_data = parse_content(&xform_resp);
    assert_eq!(xform_data["length_unit"], "m");
    assert!(xform_data["requirements_status"].is_object());
    let req0 = &xform_data["requirements_status"]["requirements"][0];
    assert_eq!(req0["id"], "req_box_max");
    assert_eq!(req0["status"], "PASS");
    let meas = req0["measured"].as_array().unwrap();
    // In meters: 0.08, 0.04, 0.02
    assert!((meas[0].as_f64().unwrap() - 0.08).abs() < 1e-6);
    assert!((meas[1].as_f64().unwrap() - 0.04).abs() < 1e-6);
    assert!((meas[2].as_f64().unwrap() - 0.02).abs() < 1e-6);

    // 5. Test duplicate part_id returns PART_ALREADY_EXISTS with param="part_id"
    let dup_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3036,
            "method": "tools/call",
            "params": {
                "name": "import_stl",
                "arguments": {
                    "file_path": stl_path.to_str().unwrap(),
                    "part_id": "imported_box_fixed"
                }
            }
        }),
    )
    .await;
    assert_eq!(dup_resp["result"]["isError"], json!(true));
    let dup_err_str = dup_resp["result"]["content"][0]["text"].as_str().unwrap();
    let dup_err: Value = serde_json::from_str(dup_err_str).unwrap();
    assert_eq!(dup_err["error"]["code"], "PART_ALREADY_EXISTS");
    assert_eq!(dup_err["error"]["param"], "part_id");

    // 6. Test reserved ID "ground" returns INVALID_ARGUMENT with param="part_id"
    let res_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3037,
            "method": "tools/call",
            "params": {
                "name": "import_stl",
                "arguments": {
                    "file_path": stl_path.to_str().unwrap(),
                    "part_id": "world"
                }
            }
        }),
    )
    .await;
    assert_eq!(res_resp["result"]["isError"], json!(true));
    let res_err_str = res_resp["result"]["content"][0]["text"].as_str().unwrap();
    let res_err: Value = serde_json::from_str(res_err_str).unwrap();
    assert_eq!(res_err["error"]["code"], "INVALID_ARGUMENT");
    assert_eq!(res_err["error"]["param"], "part_id");

    let _ = std::fs::remove_dir_all(tmp_dir);
}

#[tokio::test]
async fn test_mcp_p2_t32_feature_tree_validate_mesh_and_metrics_gravity_scaling() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let parse_content = |resp: &Value| -> Value {
        let text = resp["result"]["content"][0]["text"]
            .as_str()
            .expect("Expected content[0].text string");
        serde_json::from_str(text).expect("Expected valid JSON in content[0].text")
    };

    // 1. Verify schema for get_feature_tree and validate_mesh contains length_unit
    let tools_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3200,
            "method": "tools/list",
            "params": {}
        }),
    )
    .await;
    assert_ne!(tools_resp["result"]["isError"], json!(true));
    let tools = tools_resp["result"]["tools"].as_array().unwrap();

    let tree_tool = tools
        .iter()
        .find(|t| t["name"] == "get_feature_tree")
        .expect("get_feature_tree tool definition not found");
    assert!(
        tree_tool["inputSchema"]["properties"]["length_unit"].is_object(),
        "get_feature_tree inputSchema must have length_unit property"
    );

    let val_tool = tools
        .iter()
        .find(|t| t["name"] == "validate_mesh")
        .expect("validate_mesh tool definition not found");
    assert!(
        val_tool["inputSchema"]["properties"]["length_unit"].is_object(),
        "validate_mesh inputSchema must have length_unit property"
    );

    // 2. Create a test part
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3201,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "bracket_p2t32",
                    "name": "Bracket P2T32",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3202,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "bracket_p2t32",
                    "primitive_type": "box",
                    "params": { "width": 50.0, "height": 30.0, "depth": 20.0 }
                }
            }
        }),
    )
    .await;

    // 3. Test get_feature_tree metadata (part_id, revision, features_count)
    let tree_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3203,
            "method": "tools/call",
            "params": {
                "name": "get_feature_tree",
                "arguments": {
                    "part_id": "bracket_p2t32"
                }
            }
        }),
    )
    .await;
    assert_ne!(tree_resp["result"]["isError"], json!(true));
    let tree_data = parse_content(&tree_resp);
    assert_eq!(tree_data["part_id"], "bracket_p2t32");
    assert_eq!(tree_data["revision"], 1);
    assert_eq!(tree_data["features_count"], 1);
    assert_eq!(tree_data["length_unit"], "mm");

    // 4. Test validate_mesh metadata (part_id, revision, volume, volume_m3)
    let val_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3204,
            "method": "tools/call",
            "params": {
                "name": "validate_mesh",
                "arguments": {
                    "part_id": "bracket_p2t32"
                }
            }
        }),
    )
    .await;
    assert_ne!(val_resp["result"]["isError"], json!(true));
    let val_data = parse_content(&val_resp);
    assert_eq!(val_data["part_id"], "bracket_p2t32");
    assert_eq!(val_data["revision"], 1);
    assert_eq!(val_data["length_unit"], "mm");
    assert_eq!(val_data["status"], "PASS");
    assert!((val_data["volume"].as_f64().unwrap() - 30_000.0).abs() < 1e-3);
    assert!((val_data["volume_m3"].as_f64().unwrap() - 3.0e-5).abs() < 1e-9);

    // 5. Test get_simulation_metrics_json with default mm -> gravity in mm/s² and joints_count
    let metrics_mm_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3205,
            "method": "tools/call",
            "params": {
                "name": "get_simulation_metrics_json",
                "arguments": {}
            }
        }),
    )
    .await;
    assert_ne!(metrics_mm_resp["result"]["isError"], json!(true));
    let metrics_mm = parse_content(&metrics_mm_resp);
    assert_eq!(metrics_mm["length_unit"], "mm");
    assert_eq!(metrics_mm["current_metrics"]["joints_count"], 0);
    assert_eq!(metrics_mm["current_metrics"]["physics"]["joints_count"], 0);
    assert_eq!(
        metrics_mm["current_metrics"]["physics"]["gravity"][1].as_f64().unwrap(),
        -9810.0
    );
    assert_eq!(
        metrics_mm["current_metrics"]["physics"]["gravity_m_s2"][1].as_f64().unwrap(),
        -9.81
    );

    // 6. Test get_simulation_metrics_json with length_unit = "m" -> gravity in m/s²
    let metrics_m_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3206,
            "method": "tools/call",
            "params": {
                "name": "get_simulation_metrics_json",
                "arguments": {
                    "length_unit": "m"
                }
            }
        }),
    )
    .await;
    assert_ne!(metrics_m_resp["result"]["isError"], json!(true));
    let metrics_m = parse_content(&metrics_m_resp);
    assert_eq!(metrics_m["length_unit"], "m");
    assert_eq!(
        metrics_m["current_metrics"]["physics"]["gravity"][1].as_f64().unwrap(),
        -9.81
    );
    assert_eq!(
        metrics_m["current_metrics"]["physics"]["gravity_m_s2"][1].as_f64().unwrap(),
        -9.81
    );
}

#[tokio::test]
async fn test_mcp_p2_t33_transform_and_csg_on_imported_stl_part() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    let parse_content = |resp: &Value| -> Value {
        let text = resp["result"]["content"][0]["text"]
            .as_str()
            .expect("Expected content[0].text string");
        serde_json::from_str(text).expect("Expected valid JSON in content[0].text")
    };

    // 1. Create a base box part (60 x 40 x 20 mm) and export to STL
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3301,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "base_for_stl",
                    "name": "Base For STL",
                    "material": "aluminum_6061_t6"
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3302,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "base_for_stl",
                    "primitive_type": "box",
                    "params": { "width": 60.0, "height": 40.0, "depth": 20.0 }
                }
            }
        }),
    )
    .await;

    let tmp_dir = std::env::temp_dir().join(format!("mcpcad_contract_p2t33_{}", uuid::Uuid::new_v4()));
    let stl_path = tmp_dir.join("p2t33_contract.stl");

    let exp_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3303,
            "method": "tools/call",
            "params": {
                "name": "export_stl",
                "arguments": {
                    "part_id": "base_for_stl",
                    "filepath": stl_path.to_str().unwrap(),
                    "binary": true
                }
            }
        }),
    )
    .await;
    assert_ne!(exp_resp["result"]["isError"], json!(true));

    // 2. Import the STL as `imported_block`
    let imp_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3304,
            "method": "tools/call",
            "params": {
                "name": "import_stl",
                "arguments": {
                    "filepath": stl_path.to_str().unwrap(),
                    "part_id": "imported_block",
                    "name": "Imported Block"
                }
            }
        }),
    )
    .await;
    assert_ne!(imp_resp["result"]["isError"], json!(true));
    let imp_data = parse_content(&imp_resp);
    assert_eq!(imp_data["part_id"], "imported_block");
    assert_eq!(imp_data["revision"], 1);
    assert!((imp_data["volume"].as_f64().unwrap() - 48_000.0).abs() < 1e-1);

    // 3. Call transform_part on `imported_block` (must succeed without CAD_EVALUATION_ERROR)
    let tr_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3305,
            "method": "tools/call",
            "params": {
                "name": "transform_part",
                "arguments": {
                    "part_id": "imported_block",
                    "translation": [50.0, -30.0, 10.0],
                    "scale": [1.5, 1.5, 1.5]
                }
            }
        }),
    )
    .await;
    assert_ne!(
        tr_resp["result"]["isError"],
        json!(true),
        "transform_part on imported_block failed: {:?}",
        tr_resp
    );
    let tr_data = parse_content(&tr_resp);
    assert_eq!(tr_data["part_id"], "imported_block");
    assert_eq!(tr_data["status"], "transformed");
    assert_eq!(tr_data["revision"], 2);
    // 48,000 * 1.5^3 = 162,000 mm^3
    assert!((tr_data["volume"].as_f64().unwrap() - 162_000.0).abs() < 1.0);
    let dims = tr_data["bounding_box"]["dimensions"].as_array().unwrap();
    assert!((dims[0].as_f64().unwrap() - 90.0).abs() < 1e-3);
    assert!((dims[1].as_f64().unwrap() - 60.0).abs() < 1e-3);
    assert!((dims[2].as_f64().unwrap() - 30.0).abs() < 1e-3);

    // 4. Cut a hole through the transformed imported STL part to verify subsequent DAG ops work
    let hole_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3306,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": "imported_block",
                    "position": [50.0, -30.0, 10.0],
                    "normal": [0.0, 0.0, 1.0],
                    "radius": 5.0,
                    "through": true
                }
            }
        }),
    )
    .await;
    assert_ne!(
        hole_resp["result"]["isError"],
        json!(true),
        "cut_hole on transformed imported_block failed: {:?}",
        hole_resp
    );
    let hole_data = parse_content(&hole_resp);
    assert_eq!(hole_data["revision"], 3);

    // 5. Verify get_feature_tree has 3 nodes (ImportedMesh -> Transform -> CutHole)
    let tree_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3307,
            "method": "tools/call",
            "params": {
                "name": "get_feature_tree",
                "arguments": {
                    "part_id": "imported_block"
                }
            }
        }),
    )
    .await;
    assert_ne!(tree_resp["result"]["isError"], json!(true));
    let tree_data = parse_content(&tree_resp);
    assert_eq!(tree_data["features_count"], 3);
    assert_eq!(tree_data["revision"], 3);

    let _ = std::fs::remove_dir_all(tmp_dir);
}

#[tokio::test]
async fn test_mcp_p2_t34_get_feature_tree_mesh_operands_scaling() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create target box (40 x 40 x 20 mm) and tool cylinder (r=10 mm, h=30 mm)
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3401,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "target_b",
                    "name": "Target Box",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3402,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "target_b",
                    "primitive_type": "box",
                    "params": { "width": 40.0, "height": 40.0, "depth": 20.0 }
                }
            }
        }),
    )
    .await;

    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3403,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "tool_c",
                    "name": "Tool Cylinder",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3404,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "tool_c",
                    "primitive_type": "cylinder",
                    "params": { "radius": 10.0, "height": 30.0, "segments": 24 }
                }
            }
        }),
    )
    .await;

    // 2. Perform boolean_difference (adds BooleanDifferenceMesh node)
    let diff_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3405,
            "method": "tools/call",
            "params": {
                "name": "boolean_difference",
                "arguments": {
                    "target_part_id": "target_b",
                    "tool_part_id": "tool_c"
                }
            }
        }),
    )
    .await;
    assert_ne!(diff_resp["result"]["isError"], json!(true));

    // 3. Call get_feature_tree in "mm" and in "m"
    let tree_mm_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3406,
            "method": "tools/call",
            "params": {
                "name": "get_feature_tree",
                "arguments": {
                    "part_id": "target_b",
                    "length_unit": "mm"
                }
            }
        }),
    )
    .await;
    let tree_m_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3407,
            "method": "tools/call",
            "params": {
                "name": "get_feature_tree",
                "arguments": {
                    "part_id": "target_b",
                    "length_unit": "m"
                }
            }
        }),
    )
    .await;
    let tree_mm = parse_content(&tree_mm_resp);
    let tree_m = parse_content(&tree_m_resp);

    let tool_mesh_mm = &tree_mm["dag"][1]["op"]["BooleanDifferenceMesh"]["tool_mesh"];
    let tool_mesh_m = &tree_m["dag"][1]["op"]["BooleanDifferenceMesh"]["tool_mesh"];

    let max_z_mm = tool_mesh_mm["vertices"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v["position"][2].as_f64().unwrap())
        .fold(f64::NEG_INFINITY, f64::max);
    let max_z_m = tool_mesh_m["vertices"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v["position"][2].as_f64().unwrap())
        .fold(f64::NEG_INFINITY, f64::max);
    assert!((max_z_mm - 15.0).abs() < 1e-6);
    assert!((max_z_m - 0.015).abs() < 1e-9);

    let faces_mm = tool_mesh_mm["faces"].as_array().unwrap();
    let faces_m = tool_mesh_m["faces"].as_array().unwrap();
    assert!(!faces_mm.is_empty());
    for (f_mm, f_m) in faces_mm.iter().zip(faces_m.iter()) {
        let area_mm = f_mm["area"].as_f64().unwrap();
        let area_m = f_m["area"].as_f64().unwrap();
        assert!((area_mm - area_m * 1e6).abs() < 1e-3);
        if let (Some(r_mm), Some(r_m)) = (f_mm["radius"].as_f64(), f_m["radius"].as_f64()) {
            assert!((r_mm - 10.0).abs() < 1e-6);
            assert!((r_m - 0.01).abs() < 1e-9);
        }
    }
}

#[tokio::test]
async fn test_mcp_p2_t35_feature_mutation_tools_part_id_depth_and_feature_id() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create part
    rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3501,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "bushing_p2t35",
                    "name": "Bushing P2T35",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;

    // 2. add_feature_primitive -> verify part_id, primitive_type, feature_id, revision, bounding_box, volume
    let add_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3502,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "bushing_p2t35",
                    "primitive_type": "cylinder",
                    "params": { "radius": 25.0, "height": 50.0, "segments": 32 }
                }
            }
        }),
    )
    .await;
    assert_ne!(add_resp["result"]["isError"], json!(true));
    let add_data = parse_content(&add_resp);
    assert_eq!(add_data["part_id"], "bushing_p2t35");
    assert_eq!(add_data["primitive_type"], "cylinder");
    assert_eq!(add_data["revision"], 1);
    let cyl_feat_id = add_data["feature_id"].as_str().unwrap().to_string();
    assert!((add_data["bounding_box"]["dimensions"][2].as_f64().unwrap() - 50.0).abs() < 1e-6);
    assert!(add_data["volume"].as_f64().unwrap() > 0.0);

    // 3. modify_feature -> verify part_id, feature_id, revision, bounding_box, volume
    let mod_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3503,
            "method": "tools/call",
            "params": {
                "name": "modify_feature",
                "arguments": {
                    "part_id": "bushing_p2t35",
                    "feature_id": cyl_feat_id,
                    "params": { "height": 60.0 }
                }
            }
        }),
    )
    .await;
    assert_ne!(mod_resp["result"]["isError"], json!(true));
    let mod_data = parse_content(&mod_resp);
    assert_eq!(mod_data["part_id"], "bushing_p2t35");
    assert_eq!(mod_data["feature_id"], cyl_feat_id);
    assert_eq!(mod_data["revision"], 2);
    assert!((mod_data["bounding_box"]["dimensions"][2].as_f64().unwrap() - 60.0).abs() < 1e-6);

    // 4. cut_hole (through: true) -> verify part_id, depth, depth_m, through, bounding_box, volume
    let hole_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3504,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": "bushing_p2t35",
                    "position": [0.0, 0.0, 30.0],
                    "normal": [0.0, 0.0, -1.0],
                    "radius": 10.0,
                    "through": true
                }
            }
        }),
    )
    .await;
    assert_ne!(hole_resp["result"]["isError"], json!(true));
    let hole_data = parse_content(&hole_resp);
    assert_eq!(hole_data["part_id"], "bushing_p2t35");
    assert_eq!(hole_data["revision"], 3);
    assert_eq!(hole_data["through"], true);
    assert!((hole_data["depth"].as_f64().unwrap() - 60.0).abs() < 1e-6);
    assert!((hole_data["depth_m"].as_f64().unwrap() - 0.06).abs() < 1e-9);
    assert!(hole_data["bounding_box"]["dimensions"].is_array());

    // 5. cut_helical_groove -> verify part_id, feature_id, revision, start_z, bounding_box, volume
    let groove_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3505,
            "method": "tools/call",
            "params": {
                "name": "cut_helical_groove",
                "arguments": {
                    "part_id": "bushing_p2t35",
                    "helix_radius": 25.0,
                    "groove_radius": 1.0,
                    "pitch": 12.0,
                    "height": 30.0
                }
            }
        }),
    )
    .await;
    assert_ne!(groove_resp["result"]["isError"], json!(true));
    let groove_data = parse_content(&groove_resp);
    assert_eq!(groove_data["part_id"], "bushing_p2t35");
    assert!(groove_data["feature_id"].as_str().is_some());
    assert_eq!(groove_data["revision"], 4);
    assert!((groove_data["start_z"].as_f64().unwrap() - (-30.0)).abs() < 1e-6);
    assert!(groove_data["bounding_box"]["dimensions"].is_array());
    assert!(groove_data["volume"].as_f64().unwrap() > 0.0);
}

#[tokio::test]
async fn test_mcp_p2_t36_optimize_feature_tree_dag_scaling_delete_part_id_and_gravity_f64() {
    let state = EngineState::new()
        .expect("EngineState initialization failed")
        .into_shared();

    // 1. Create a part and add coaxial cylinder + hole features in default "mm"
    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3601,
            "method": "tools/call",
            "params": {
                "name": "create_part",
                "arguments": {
                    "part_id": "sleeve_p2t36",
                    "name": "Sleeve P2-T36",
                    "material": "steel_structural_aisi1020"
                }
            }
        }),
    )
    .await;

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3602,
            "method": "tools/call",
            "params": {
                "name": "add_feature_primitive",
                "arguments": {
                    "part_id": "sleeve_p2t36",
                    "primitive_type": "cylinder",
                    "params": {
                        "radius": 28.0,
                        "height": 60.0
                    }
                }
            }
        }),
    )
    .await;

    let _ = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3603,
            "method": "tools/call",
            "params": {
                "name": "cut_hole",
                "arguments": {
                    "part_id": "sleeve_p2t36",
                    "position": [0.0, 0.0, 30.0],
                    "normal": [0.0, 0.0, -1.0],
                    "radius": 14.0,
                    "depth": 60.0
                }
            }
        }),
    )
    .await;

    // 2. Call optimize_feature_tree with apply: false (default length_unit: "mm")
    let opt_mm_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3604,
            "method": "tools/call",
            "params": {
                "name": "optimize_feature_tree",
                "arguments": {
                    "part_id": "sleeve_p2t36",
                    "apply": false
                }
            }
        }),
    )
    .await;
    assert_ne!(opt_mm_resp["result"]["isError"], json!(true));
    let opt_mm = parse_content(&opt_mm_resp);
    assert_eq!(opt_mm["length_unit"], "mm");
    assert!((opt_mm["logical_plan"]["dag"][0]["op"]["AddCylinder"]["radius"].as_f64().unwrap() - 28.0).abs() < 1e-6);
    assert!((opt_mm["logical_plan"]["dag"][0]["op"]["AddCylinder"]["height"].as_f64().unwrap() - 60.0).abs() < 1e-6);
    assert!((opt_mm["logical_plan"]["dag"][1]["op"]["CutHole"]["radius"].as_f64().unwrap() - 14.0).abs() < 1e-6);
    assert!((opt_mm["logical_plan"]["dag"][1]["op"]["CutHole"]["depth"].as_f64().unwrap() - 60.0).abs() < 1e-6);
    assert!((opt_mm["physical_plan"]["dag"][0]["op"]["AddRevolve"]["profile"][0][0].as_f64().unwrap() - 14.0).abs() < 1e-6);
    assert!((opt_mm["candidate_plans"][0]["dag"][0]["op"]["AddCylinder"]["radius"].as_f64().unwrap() - 28.0).abs() < 1e-6);

    // 3. Call optimize_feature_tree with length_unit: "m" and apply: true
    let opt_m_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3605,
            "method": "tools/call",
            "params": {
                "name": "optimize_feature_tree",
                "arguments": {
                    "part_id": "sleeve_p2t36",
                    "apply": true,
                    "length_unit": "m"
                }
            }
        }),
    )
    .await;
    assert_ne!(opt_m_resp["result"]["isError"], json!(true));
    let opt_m = parse_content(&opt_m_resp);
    assert_eq!(opt_m["length_unit"], "m");
    assert!((opt_m["logical_plan"]["dag"][0]["op"]["AddCylinder"]["radius"].as_f64().unwrap() - 0.028).abs() < 1e-9);
    assert!((opt_m["physical_plan"]["dag"][0]["op"]["AddRevolve"]["profile"][0][0].as_f64().unwrap() - 0.014).abs() < 1e-9);

    // 4. Verify get_simulation_metrics_json returns exact f64 gravity (-9810.0 in mm/s^2, -9.81 in m/s^2)
    let sim_mm_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3606,
            "method": "tools/call",
            "params": {
                "name": "get_simulation_metrics_json",
                "arguments": {
                    "length_unit": "mm"
                }
            }
        }),
    )
    .await;
    assert_ne!(sim_mm_resp["result"]["isError"], json!(true));
    let sim_mm = parse_content(&sim_mm_resp);
    assert_eq!(sim_mm["current_metrics"]["physics"]["gravity"][1].as_f64().unwrap(), -9810.0);
    assert_eq!(sim_mm["current_metrics"]["physics"]["gravity_m_s2"][1].as_f64().unwrap(), -9.81);

    // 5. Verify delete_part returns status, part_id, and deleted_part_id
    let del_resp = rpc_call(
        &state,
        json!({
            "jsonrpc": "2.0",
            "id": 3607,
            "method": "tools/call",
            "params": {
                "name": "delete_part",
                "arguments": {
                    "part_id": "sleeve_p2t36"
                }
            }
        }),
    )
    .await;
    assert_ne!(del_resp["result"]["isError"], json!(true));
    let del_data = parse_content(&del_resp);
    assert_eq!(del_data["status"], "deleted");
    assert_eq!(del_data["part_id"], "sleeve_p2t36");
    assert_eq!(del_data["deleted_part_id"], "sleeve_p2t36");
}

