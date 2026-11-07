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
MCP-CAD Agent Guide (full text: docs/AGENT_GUIDE.md, docs/MCP_API_REFERENCE.md, docs/UNITS.md):
1. SCENE SUMMARY: Call 'get_scene_summary' first in a new context to inspect active units, parts (id, bbox, volume, mass, mesh validity, FEA FoS/verdict/stale), joints, and requirements.
2. UNITS: Default length_unit is 'mm' (switch to 'm' via 'set_units' or per-call 'length_unit'). Force: N, moment: N*m, stress/pressure: MPa, Young's modulus: GPa, displacement: mm, mass: kg, density: kg/m^3, angles: deg, angular velocity: rad/s, time: s (step_size_ms: ms). STL files on disk use meters (m).
3. REQUIREMENTS: Declare constraints ('bbox_max', 'bbox_min', 'mass_max', 'mesh_valid', 'min_safety_factor') via 'set_requirements' with 'auto_check': true so mutating tools append 'requirements_status' and 'failed_requirements'.
4. TRANSACTIONS & VALIDATION: Use 'batch_cad_operations' for atomic multi-step edits (rolls back on error). Always call 'validate_mesh' after 'boolean_*', 'cut_hole', or 'cut_helical_groove' to confirm status=='PASS' before FEA.
5. FEA & NAMED FACES: Run 'run_fea_analysis' only after 'validate_mesh' PASS. Reference named faces from 'query_part_geometry' ('cap_bottom', 'cap_top', 'left', 'right', 'hole_wall:*') in 'fixed' and 'loads'. Edits bump part.revision and mark FEA 'STALE'.
6. ERRORS & LOOP: When isError=true, parse JSON {'error':{'code','param','hint','details'}} and follow 'hint'. Workflow: Build -> Validate ('validate_mesh') -> Analyze ('run_fea_analysis') -> Iterate ('modify_feature' + 'check_requirements') -> 'save_project'.";

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
