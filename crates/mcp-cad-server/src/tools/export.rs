//! Geometry export tools (STL).

use std::path::Path;
use serde_json::{json, Value};

use mcp_cad_core::export_stl::{write_stl_ascii_file, write_stl_binary_file};

use crate::error::ServerError;
use crate::state::EngineState;

/// 14. `export_stl`: Exports a part's boundary surface mesh to an STL file (ASCII or Binary).
pub fn export_stl(args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let filepath = args
        .get("filepath")
        .or_else(|| args.get("file_path"))
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'filepath'".to_string()))?;

    let binary = args
        .get("binary")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);

    let part = state
        .parts
        .get(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    if part.mesh.vertices.is_empty() {
        return Err(ServerError::InvalidArgument(
            "Cannot export empty part mesh to STL. Add features first.".to_string(),
        ));
    }

    let path = Path::new(filepath);
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }

    if binary {
        write_stl_binary_file(&part.mesh, path)?;
    } else {
        write_stl_ascii_file(&part.mesh, &part.name, path)?;
    }

    Ok(json!({
        "file_path": filepath,
        "triangles_count": part.mesh.triangle_count() as u32
    }))
}
