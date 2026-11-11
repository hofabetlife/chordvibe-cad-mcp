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
    if width == 0 || height == 0 {
        return Err(ServerError::InvalidArgument(
            "Viewport width and height must be greater than zero".to_string(),
        ));
    }
    let debug_mode_str = args.get("debug_mode").and_then(|v| v.as_str());
    let debug_mode = mcp_cad_viewer::DebugRenderMode::parse(debug_mode_str);

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
        .render_mesh_base64_debug(&target_mesh, preset, debug_mode, width, height)?;

    Ok(json!({
        "image_base64": image_base64,
        "format": "image/png",
        "camera_preset": format!("{preset:?}"),
        "debug_mode": format!("{debug_mode:?}"),
        "width": width,
        "height": height
    }))
}

/// 16. `open_preview_window`: Opens or focuses the interactive native 3D CAD preview window on Windows.
pub fn open_preview_window(args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let part_id_opt = args.get("part_id").and_then(|v| v.as_str());
    if let Some(part_id) = part_id_opt {
        if !state.parts.contains_key(part_id) {
            return Err(ServerError::PartNotFound(part_id.to_string()));
        }
    }

    if let Some(proxy) = &state.gui_proxy {
        let part_id_opt = args.get("part_id").and_then(|v| v.as_str());
        if let Some(part_id) = part_id_opt {
            let part = state
                .parts
                .get(part_id)
                .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;
            state.send_gui_mesh(&part.mesh);
            Ok(json!({
                "status": "opened",
                "part_id": part_id,
                "message": format!("Preview window opened showing part '{}'", part.name)
            }))
        } else if let Some(sim) = &state.last_simulation {
            let frames = crate::tools::physics::generate_animation_frames(&state.parts, &sim.trajectories, 120);
            if !frames.is_empty() {
                state.send_gui_animation(frames, 30.0);
                Ok(json!({
                    "status": "opened",
                    "mode": "animation_playback",
                    "message": "Preview window opened playing mechanism simulation animation"
                }))
            } else {
                let _ = proxy.send_event(mcp_cad_viewer::window::GuiEvent::Show);
                Ok(json!({
                    "status": "opened",
                    "message": "Preview window opened"
                }))
            }
        } else {
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

            if !combined_verts.is_empty() {
                let mesh = TriangleMesh::new(combined_verts, combined_indices);
                state.send_gui_mesh(&mesh);
            } else {
                let _ = proxy.send_event(mcp_cad_viewer::window::GuiEvent::Show);
            }

            Ok(json!({
                "status": "opened",
                "message": "Preview window opened"
            }))
        }
    } else {
        let part_id_opt = args.get("part_id").and_then(|v| v.as_str());
        let target_mesh = if let Some(part_id) = part_id_opt {
            let part = state
                .parts
                .get(part_id)
                .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;
            part.mesh.clone()
        } else {
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
            TriangleMesh::new(combined_verts, combined_indices)
        };

        if target_mesh.vertices.is_empty() {
            return Ok(json!({
                "status": "empty",
                "message": "No geometry in scene to preview. Create parts or add features first."
            }));
        }

        // Export target mesh to temporary binary STL
        let out_dir = std::path::Path::new("./output");
        let preview_stl_path = if out_dir.exists() || std::fs::create_dir_all(out_dir).is_ok() {
            out_dir.join("_mcp_preview_live.stl")
        } else {
            std::env::temp_dir().join("mcp_cad_preview_live.stl")
        };

        if let Ok(mut f) = std::fs::File::create(&preview_stl_path) {
            let _ = mcp_cad_core::export_stl::export_stl_binary(&target_mesh, &mut f);
        }

        let disable_spawn = std::env::var("MCP_CAD_DISABLE_GUI_SPAWN")
            .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
            .unwrap_or(false);

        if disable_spawn {
            return Ok(json!({
                "status": "headless",
                "stl_path": preview_stl_path.to_string_lossy(),
                "message": "Server is in headless mode (MCP_CAD_DISABLE_GUI_SPAWN=1). Preview STL exported; use capture_viewport for headless perception."
            }));
        }

        let exe_path = match std::env::current_exe() {
            Ok(p) => p,
            Err(e) => {
                return Ok(json!({
                    "status": "headless",
                    "stl_path": preview_stl_path.to_string_lossy(),
                    "message": format!("Could not resolve current executable ({e}). Preview STL exported.")
                }));
            }
        };

        match std::process::Command::new(exe_path)
            .arg("--stl")
            .arg(&preview_stl_path)
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
        {
            Ok(child) => Ok(json!({
                "status": "opened",
                "mode": "spawned_gui_process",
                "pid": child.id(),
                "stl_path": preview_stl_path.to_string_lossy(),
                "part_id": part_id_opt,
                "message": "Interactive 3D preview window spawned in background"
            })),
            Err(err) => Ok(json!({
                "status": "headless",
                "stl_path": preview_stl_path.to_string_lossy(),
                "message": format!("Server failed to spawn GUI process: {err}. Preview STL exported.")
            })),
        }
    }
}
