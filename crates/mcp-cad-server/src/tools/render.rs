//! Viewport capture and rendering tools.

use serde_json::{json, Value};

use mcp_cad_core::primitives::create_box;
use mcp_cad_core::topology::TriangleMesh;

use crate::error::ServerError;
use crate::state::EngineState;
use crate::tools::fea::parse_camera_preset;

/// 13. `capture_viewport`: Renders the CAD scene to PNG and returns it as a Base64 string.
pub fn capture_viewport(args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let preset = parse_camera_preset(args.get("camera_preset").and_then(|v| v.as_str()));
    let width = args.get("width").and_then(|v| v.as_u64()).unwrap_or(800) as u32;
    let height = args.get("height").and_then(|v| v.as_u64()).unwrap_or(600) as u32;

    let target_mesh = if let Some(part_id) = args.get("part_id").and_then(|v| v.as_str()) {
        let part = state
            .parts
            .get(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;
        if part.mesh.vertices.is_empty() {
            create_box(0.1, 0.1, 0.1)
        } else {
            part.mesh.clone()
        }
    } else {
        // Combine meshes of all parts
        let mut combined_verts = Vec::new();
        let mut combined_indices = Vec::new();

        for part in state.parts.values() {
            if !part.mesh.vertices.is_empty() {
                let offset = combined_verts.len() as u32;
                combined_verts.extend_from_slice(&part.mesh.vertices);
                for tri in &part.mesh.indices {
                    combined_indices.push([tri[0] + offset, tri[1] + offset, tri[2] + offset]);
                }
            }
        }

        if combined_verts.is_empty() {
            create_box(0.1, 0.1, 0.1)
        } else {
            TriangleMesh::new(combined_verts, combined_indices)
        }
    };

    let image_base64 = state
        .offscreen_renderer
        .render_mesh_base64(&target_mesh, preset, width, height)?;

    Ok(json!({
        "image_base64": image_base64,
        "format": "image/png",
        "camera_preset": format!("{preset:?}"),
        "width": width,
        "height": height
    }))
}
