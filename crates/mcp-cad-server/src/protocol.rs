//! MCP JSON-RPC 2.0 protocol structures and request dispatching.

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::state::SharedState;
use crate::tools::{dispatch_tool, list_tools};

/// MCP JSON-RPC 2.0 Request envelope.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonRpcRequest {
    pub jsonrpc: String,
    #[serde(default)]
    pub id: Option<Value>,
    pub method: String,
    #[serde(default)]
    pub params: Option<Value>,
}

/// MCP JSON-RPC 2.0 Response envelope.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonRpcResponse {
    pub jsonrpc: String,
    pub id: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<JsonRpcError>,
}

impl JsonRpcResponse {
    pub fn success(id: Option<Value>, result: Value) -> Self {
        Self {
            jsonrpc: "2.0".to_string(),
            id,
            result: Some(result),
            error: None,
        }
    }

    pub fn error(id: Option<Value>, error: JsonRpcError) -> Self {
        Self {
            jsonrpc: "2.0".to_string(),
            id,
            result: None,
            error: Some(error),
        }
    }
}

/// JSON-RPC 2.0 standard error payload.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonRpcError {
    pub code: i64,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<Value>,
}

impl JsonRpcError {
    pub fn parse_error(msg: impl Into<String>) -> Self {
        Self {
            code: -32700,
            message: msg.into(),
            data: None,
        }
    }

    pub fn invalid_request(msg: impl Into<String>) -> Self {
        Self {
            code: -32600,
            message: msg.into(),
            data: None,
        }
    }

    pub fn method_not_found(msg: impl Into<String>) -> Self {
        Self {
            code: -32601,
            message: msg.into(),
            data: None,
        }
    }

    pub fn invalid_params(msg: impl Into<String>) -> Self {
        Self {
            code: -32602,
            message: msg.into(),
            data: None,
        }
    }

    pub fn internal_error(msg: impl Into<String>) -> Self {
        Self {
            code: -32603,
            message: msg.into(),
            data: None,
        }
    }
}

/// Tool definition matching the Model Context Protocol specification.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolDefinition {
    pub name: String,
    pub description: String,
    #[serde(rename = "inputSchema")]
    pub input_schema: Value,
}

/// Content element returned by an MCP tool invocation.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ToolContent {
    #[serde(rename = "text")]
    Text { text: String },
    #[serde(rename = "image")]
    Image {
        data: String,
        #[serde(rename = "mimeType")]
        mime_type: String,
    },
}

/// Full result payload of an MCP `tools/call` response.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolCallResult {
    pub content: Vec<ToolContent>,
    #[serde(rename = "isError", skip_serializing_if = "Option::is_none")]
    pub is_error: Option<bool>,
}

impl ToolCallResult {
    pub fn text(text: impl Into<String>) -> Self {
        Self {
            content: vec![ToolContent::Text { text: text.into() }],
            is_error: None,
        }
    }

    pub fn json<T: Serialize>(value: &T) -> Result<Self, serde_json::Error> {
        let text = serde_json::to_string_pretty(value)?;
        Ok(Self::text(text))
    }

    pub fn image_and_json<T: Serialize>(value: &T, base64_png: String) -> Result<Self, serde_json::Error> {
        let text = serde_json::to_string_pretty(value)?;
        let mut content = vec![ToolContent::Text { text }];
        if !base64_png.is_empty() {
            content.push(ToolContent::Image {
                data: base64_png,
                mime_type: "image/png".to_string(),
            });
        }
        Ok(Self {
            content,
            is_error: None,
        })
    }

    pub fn error(message: impl Into<String>) -> Self {
        Self {
            content: vec![ToolContent::Text {
                text: message.into(),
            }],
            is_error: Some(true),
        }
    }
}

/// Concise agent operating instructions returned in the MCP `initialize` response (`instructions` field).
/// Summarizes the full guide in `docs/AGENT_GUIDE.md` (~1480 characters).
pub const AGENT_INSTRUCTIONS: &str = "\
MCP-CAD Agent Guide (full text: SKILL.md, docs/AGENT_GUIDE.md, docs/MCP_API_REFERENCE.md, docs/UNITS.md):
1. COORDINATE CONVENTIONS & PRIMITIVE ORIGINS:
   - Origin is [0, 0, 0]. Z is the primary axis of revolution/height.
   - All 3D primitives are CENTERED along Z in [-height/2, +height/2] (box, cylinder, cone, polygon_prism, spur_gear, wheel_rim).
   - 'box': X in [-width/2, +width/2], Y in [-height/2, +height/2], Z in [-depth/2, +depth/2].
   - 'cylinder': XY radius x^2+y^2 <= radius^2, Z in [-height/2, +height/2]. Caps at Z = -height/2 ('cap_bottom') and +height/2 ('cap_top').
   - 'cone': Z in [-height/2, +height/2]. Base radius_bottom is at Z = -height/2, apex radius_top is at Z = +height/2.
   - 'polygon_prism': regular N-gon in XY with outer_radius (or width_across_flats), Z in [-height/2, +height/2], optional chamfer_radius.
   - 'spur_gear': centered along Z in [-face_width/2, +face_width/2] with central bore and keyway.
   - 'revolve': 2D profile [r, z] revolved around Z-axis.
2. DAG SEMANTICS (FeatureTree):
   - First 'add_feature_primitive' creates the root base body.
   - Subsequent 'add_feature_primitive' calls on the same part_id AUTOMATICALLY perform 'csg_union' (they do NOT overwrite the part).
   - 'cut_hole' and 'cut_helical_groove' AUTOMATICALLY perform 'csg_difference'.
3. SPECIALIZED ENGINEERING TOOLS:
   - 'cut_hole': cylindrical drilling with optional DIN 6885 keyway ('keyway_width', 'keyway_depth') and 'through: true'.
   - 'cut_helical_groove': flutes and threads with 'profile_type': 'circular' (twist drills) or 'metric_60' (ISO 68-1 threads); auto-aligns start_z with bounding_box.min[2].
   - 'spur_gear': standard DIN 867 involute spur gear with hub and keyway.
   - 'wheel_rim': 5-spoke automotive wheel rim with central hub bore and PCD bolt circle.
4. GUI VS HEADLESS:
   - 'capture_viewport': renders offscreen Base64 PNG for AI agent perception (fast, works in headless CLI/container).
   - 'open_preview_window': interactive 3D WGPU desktop viewer for user. When called in stdio session without --gui, automatically exports live STL and spawns 'mcp-cad-server --stl <path>'.
   - DO NOT read Rust source code files in crates/ — all coordinate systems and API behaviors are fully defined here.
5. UNITS & WORKFLOW:
   - Default length_unit is 'mm' (force: N, moment: N*m, stress: MPa, Young's modulus: GPa, mass: kg, angles: deg). STL on disk uses meters (m).
   - Declare requirements via 'set_requirements' (auto_check: true) -> Build ('add_feature_primitive', 'cut_hole') -> 'validate_mesh' -> FEA ('run_fea_analysis') -> 'save_project'.";

/// Dispatches an MCP request and returns the optional JSON-RPC response.
pub async fn handle_request(req: JsonRpcRequest, state: &SharedState) -> Option<JsonRpcResponse> {
    let id = req.id.clone();
    let is_notification = id.is_none();

    match req.method.as_str() {
        "initialize" => {
            let (length_unit, units) = {
                let engine = state.read();
                (
                    engine.length_unit.as_str(),
                    engine.length_unit.units_metadata(),
                )
            };
            let result = json!({
                "protocolVersion": "2024-11-05",
                "capabilities": {
                    "tools": {}
                },
                "serverInfo": {
                    "name": "mcp-cad-engine",
                    "version": "0.1.0"
                },
                "instructions": AGENT_INSTRUCTIONS,
                "length_unit": length_unit,
                "units": units
            });
            Some(JsonRpcResponse::success(id, result))
        }
        "notifications/initialized" => {
            if is_notification {
                None
            } else {
                Some(JsonRpcResponse::success(id, json!({})))
            }
        }
        "ping" => Some(JsonRpcResponse::success(id, json!({}))),
        "tools/list" => {
            let tools = list_tools();
            Some(JsonRpcResponse::success(id, json!({ "tools": tools })))
        }
        "tools/call" => {
            let params = req.params.unwrap_or(Value::Null);
            let tool_name = params.get("name").and_then(|v| v.as_str());
            let arguments = params.get("arguments").cloned().unwrap_or(json!({}));

            let tool_name = match tool_name {
                Some(name) => name,
                None => {
                    return Some(JsonRpcResponse::error(
                        id,
                        JsonRpcError::invalid_params("Missing tool 'name' in arguments"),
                    ));
                }
            };

            let tool_result = {
                let mut engine = state.write();
                let call_res = dispatch_tool(tool_name, &arguments, &mut engine);
                match call_res {
                    Ok(res) => res,
                    Err(err) => {
                        let mut existing_parts: Vec<String> = engine.parts.keys().cloned().collect();
                        existing_parts.sort();
                        let error_body = err.to_error_body_with_parts(Some(&existing_parts));
                        ToolCallResult::error(error_body.to_json_string())
                    }
                }
            };

            let res_value = match serde_json::to_value(&tool_result) {
                Ok(v) => v,
                Err(e) => {
                    return Some(JsonRpcResponse::error(
                        id,
                        JsonRpcError::internal_error(format!("Failed to serialize tool result: {e}")),
                    ));
                }
            };

            Some(JsonRpcResponse::success(id, res_value))
        }
        other => {
            if is_notification {
                None
            } else {
                Some(JsonRpcResponse::error(
                    id,
                    JsonRpcError::method_not_found(format!("Unknown MCP method: '{other}'")),
                ))
            }
        }
    }
}

/// Parses a raw JSON-RPC string, dispatches the request, and returns an optional JSON-RPC response.
pub async fn handle_message(raw_json: &str, state: &SharedState) -> Option<JsonRpcResponse> {
    let req: JsonRpcRequest = match serde_json::from_str(raw_json) {
        Ok(r) => r,
        Err(e) => {
            return Some(JsonRpcResponse::error(
                None,
                JsonRpcError::parse_error(format!("JSON parse error: {e}")),
            ));
        }
    };

    handle_request(req, state).await
}
