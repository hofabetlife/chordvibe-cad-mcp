//! CAD part and parametric DAG modeling tools.

use std::path::Path;
use serde_json::{json, Value};
use uuid::Uuid;

use mcp_cad_core::dag::FeatureOp;
use mcp_cad_core::export_stl::{write_stl_ascii_file, write_stl_binary_file};
use mcp_cad_core::material::Material;
use mcp_cad_core::part::Part;
use mcp_cad_core::tessellation::TessellationSettings;

use crate::error::ServerError;
use crate::state::EngineState;

/// Helper function to parse 3D vector coordinates [x, y, z] from a JSON object.
fn parse_vec3(val: &Value, key: &str) -> Option<[f64; 3]> {
    let arr = val.get(key)?.as_array()?;
    if arr.len() == 3 {
        Some([
            arr[0].as_f64()?,
            arr[1].as_f64()?,
            arr[2].as_f64()?,
        ])
    } else {
        None
    }
}

/// 1. `create_part`: Creates a new empty part with assigned material.
pub fn create_part(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let name = args
        .get("name")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'name'".to_string()))?;

    let material_name = args
        .get("material")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'material'".to_string()))?;

    let material = state
        .materials
        .get(material_name)
        .cloned()
        .ok_or_else(|| {
            let available: Vec<_> = state.materials.list().into_iter().map(|m| m.name.as_str()).collect();
            ServerError::InvalidArgument(format!(
                "Material '{}' not found in registry. Available presets: {}",
                material_name,
                available.join(", ")
            ))
        })?;

    let part_id = args
        .get("part_id")
        .or_else(|| args.get("id"))
        .and_then(|v| v.as_str())
        .map(|s| s.to_string())
        .unwrap_or_else(|| format!("part-{}", Uuid::new_v4().simple()));

    if state.parts.contains_key(&part_id) {
        return Err(ServerError::InvalidArgument(format!("Part with id '{}' already exists", part_id)));
    }

    let is_fixed = args.get("is_fixed").and_then(|v| v.as_bool()).unwrap_or(false);
    let mut part = Part::new(&part_id, name, material);
    part.is_fixed = is_fixed;

    state.parts.insert(part_id.clone(), part);

    Ok(json!({
        "part_id": part_id,
        "is_fixed": is_fixed
    }))
}

/// 2. `list_parts`: Returns summary metadata of all active parts in the engine.
pub fn list_parts(_args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let summaries: Vec<_> = state.parts.values().map(|p| p.summary()).collect();
    Ok(json!({
        "parts": summaries
    }))
}

/// 3. `delete_part`: Deletes a part from state, physics world, and analysis caches.
pub fn delete_part(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    if state.parts.remove(part_id).is_none() {
        return Err(ServerError::PartNotFound(part_id.to_string()));
    }

    let _ = state.physics_world.remove_part(part_id);
    state.last_fea_reports.remove(part_id);
    state.last_fea_meshes.remove(part_id);

    Ok(json!({
        "status": "deleted"
    }))
}

/// 4. `add_feature_primitive`: Adds a 3D solid primitive to a part's DAG.
pub fn add_feature_primitive(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let prim_type = args
        .get("primitive_type")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'primitive_type'".to_string()))?;

    let params = args.get("params").unwrap_or(args);

    let position = parse_vec3(params, "position").or_else(|| parse_vec3(args, "position"));
    let rotation_deg = parse_vec3(params, "rotation_deg")
        .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
        .or_else(|| parse_vec3(args, "rotation_deg"))
        .or_else(|| parse_vec3(args, "rotation_rpy_deg"));

    let op = match prim_type.to_lowercase().as_str() {
        "box" => {
            let width = params
                .get("width")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Box requires 'width'".to_string()))?;
            let height = params
                .get("height")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Box requires 'height'".to_string()))?;
            let depth = params
                .get("depth")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Box requires 'depth'".to_string()))?;
            FeatureOp::AddBox {
                width,
                height,
                depth,
                position,
                rotation_deg,
            }
        }
        "cylinder" => {
            let radius = params
                .get("radius")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Cylinder requires 'radius'".to_string()))?;
            let height = params
                .get("height")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Cylinder requires 'height'".to_string()))?;
            let segments = params
                .get("segments")
                .and_then(|v| v.as_u64())
                .unwrap_or(32) as usize;
            FeatureOp::AddCylinder {
                radius,
                height,
                segments,
                position,
                rotation_deg,
            }
        }
        "sphere" => {
            let radius = params
                .get("radius")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Sphere requires 'radius'".to_string()))?;
            let sectors = params
                .get("sectors")
                .and_then(|v| v.as_u64())
                .unwrap_or(32) as usize;
            let stacks = params
                .get("stacks")
                .and_then(|v| v.as_u64())
                .unwrap_or(16) as usize;
            FeatureOp::AddSphere {
                radius,
                sectors,
                stacks,
                position,
                rotation_deg,
            }
        }
        "cone" => {
            let radius_bottom = params
                .get("radius_bottom")
                .or_else(|| params.get("radius"))
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Cone requires 'radius_bottom'".to_string()))?;
            let radius_top = params
                .get("radius_top")
                .and_then(|v| v.as_f64())
                .unwrap_or(0.0);
            let height = params
                .get("height")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Cone requires 'height'".to_string()))?;
            let segments = params
                .get("segments")
                .and_then(|v| v.as_u64())
                .unwrap_or(32) as usize;
            FeatureOp::AddCone {
                radius_bottom,
                radius_top,
                height,
                segments,
                position,
                rotation_deg,
            }
        }
        "polygon_prism" => {
            let height = params
                .get("height")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Polygon prism requires 'height'".to_string()))?;
            let sides = params
                .get("sides")
                .and_then(|v| v.as_u64())
                .unwrap_or(6) as usize;
            let width_across_flats = params
                .get("width_across_flats")
                .or_else(|| params.get("s"))
                .and_then(|v| v.as_f64());
            let radius = params
                .get("radius")
                .or_else(|| params.get("r"))
                .and_then(|v| v.as_f64());
            if width_across_flats.is_none() && radius.is_none() {
                return Err(ServerError::InvalidArgument(
                    "Polygon prism requires either 'width_across_flats' or 'radius'".to_string(),
                ));
            }
            let chamfer = params
                .get("chamfer")
                .and_then(|v| v.as_f64());
            FeatureOp::AddPolygonPrism {
                sides,
                height,
                width_across_flats,
                radius,
                chamfer,
                position,
                rotation_deg,
            }
        }
        "spur_gear" => {
            let module = params
                .get("module")
                .or_else(|| params.get("m"))
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Spur gear requires 'module'".to_string()))?;
            let teeth = params
                .get("teeth")
                .or_else(|| params.get("z"))
                .and_then(|v| v.as_u64())
                .ok_or_else(|| ServerError::InvalidArgument("Spur gear requires 'teeth'".to_string()))? as usize;
            let pressure_angle_deg = params
                .get("pressure_angle_deg")
                .or_else(|| params.get("pressure_angle"))
                .and_then(|v| v.as_f64())
                .unwrap_or(20.0);
            let face_width = params
                .get("face_width")
                .or_else(|| params.get("width"))
                .or_else(|| params.get("b"))
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Spur gear requires 'face_width'".to_string()))?;
            let bore_radius = params
                .get("bore_radius")
                .and_then(|v| v.as_f64())
                .or_else(|| params.get("bore_diameter").and_then(|v| v.as_f64()).map(|d| d * 0.5));
            let keyway_width = params
                .get("keyway_width")
                .and_then(|v| v.as_f64());
            let keyway_depth = params
                .get("keyway_depth")
                .and_then(|v| v.as_f64());
            let hub_diameter = params
                .get("hub_diameter")
                .and_then(|v| v.as_f64());
            let hub_length = params
                .get("hub_length")
                .and_then(|v| v.as_f64());

            FeatureOp::AddSpurGear {
                module,
                teeth,
                pressure_angle_deg,
                face_width,
                bore_radius,
                keyway_width,
                keyway_depth,
                hub_diameter,
                hub_length,
                position,
                rotation_deg,
            }
        }
        "revolve" => {
            let profile_val = params
                .get("profile")
                .and_then(|v| v.as_array())
                .ok_or_else(|| ServerError::InvalidArgument("Revolve requires 'profile' array of [r, z] points".to_string()))?;

            let mut profile = Vec::with_capacity(profile_val.len());
            for item in profile_val {
                let arr = item.as_array().ok_or_else(|| {
                    ServerError::InvalidArgument("Each profile element must be [r, z]".to_string())
                })?;
                if arr.len() != 2 {
                    return Err(ServerError::InvalidArgument("Profile points must have 2 coordinates: [r, z]".to_string()));
                }
                let r = arr[0].as_f64().ok_or_else(|| ServerError::InvalidArgument("Invalid r coordinate in profile".to_string()))?;
                let z = arr[1].as_f64().ok_or_else(|| ServerError::InvalidArgument("Invalid z coordinate in profile".to_string()))?;
                profile.push([r, z]);
            }

            let angle_deg = params
                .get("angle_deg")
                .or_else(|| params.get("angle"))
                .and_then(|v| v.as_f64())
                .unwrap_or(360.0);

            let segments = params
                .get("segments")
                .and_then(|v| v.as_u64())
                .unwrap_or(32) as usize;

            FeatureOp::AddRevolve {
                profile,
                angle_deg,
                segments,
                position,
                rotation_deg,
            }
        }
        "wheel_rim" => {
            let rim_diameter = params
                .get("rim_diameter")
                .or_else(|| params.get("diameter"))
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Wheel rim requires 'rim_diameter'".to_string()))?;

            let rim_width = params
                .get("rim_width")
                .or_else(|| params.get("width"))
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Wheel rim requires 'rim_width'".to_string()))?;

            let flange_diameter = params
                .get("flange_diameter")
                .and_then(|v| v.as_f64());

            let hub_diameter = params
                .get("hub_diameter")
                .and_then(|v| v.as_f64());

            let hub_bore_diameter = params
                .get("hub_bore_diameter")
                .or_else(|| params.get("bore_diameter"))
                .and_then(|v| v.as_f64());

            let pcd_diameter = params
                .get("pcd_diameter")
                .or_else(|| params.get("pcd"))
                .and_then(|v| v.as_f64());

            let bolt_holes_count = params
                .get("bolt_holes_count")
                .or_else(|| params.get("bolts_count"))
                .or_else(|| params.get("bolts"))
                .and_then(|v| v.as_u64())
                .map(|b| b as usize);

            let bolt_hole_diameter = params
                .get("bolt_hole_diameter")
                .and_then(|v| v.as_f64());

            let spokes_count = params
                .get("spokes_count")
                .or_else(|| params.get("spokes"))
                .and_then(|v| v.as_u64())
                .map(|s| s as usize);

            let spoke_width = params
                .get("spoke_width")
                .and_then(|v| v.as_f64());

            let spoke_thickness = params
                .get("spoke_thickness")
                .and_then(|v| v.as_f64());

            let offset_et = params
                .get("offset_et")
                .or_else(|| params.get("et"))
                .and_then(|v| v.as_f64());

            FeatureOp::AddWheelRim {
                rim_diameter,
                rim_width,
                flange_diameter,
                hub_diameter,
                hub_bore_diameter,
                pcd_diameter,
                bolt_holes_count,
                bolt_hole_diameter,
                spokes_count,
                spoke_width,
                spoke_thickness,
                offset_et,
                position,
                rotation_deg,
            }
        }
        other => {
            return Err(ServerError::InvalidArgument(format!(
                "Unknown primitive_type '{}'. Supported: box, cylinder, sphere, cone, polygon_prism, spur_gear, revolve, wheel_rim",
                other
            )));
        }
    };

    let part = state
        .parts
        .get_mut(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    let prev_dag = part.dag.clone();
    let prev_mesh = part.mesh.clone();
    let feature_id = part.dag.add_feature(op, None);
    if let Err(e) = part.rebuild() {
        part.dag = prev_dag;
        part.mesh = prev_mesh;
        return Err(ServerError::Cad(e));
    }

    let mesh_clone = part.mesh.clone();
    state.send_gui_mesh(&mesh_clone);

    Ok(json!({
        "feature_id": feature_id.to_string()
    }))
}

/// 5. `cut_hole`: Cuts a cylindrical hole through the solid.
pub fn cut_hole(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let pos_val = args
        .get("position")
        .or_else(|| args.get("center"))
        .and_then(|v| v.as_array())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'position' [x, y, z]".to_string()))?;

    if pos_val.len() != 3 {
        return Err(ServerError::InvalidArgument("'position' must have 3 elements".to_string()));
    }
    let center = [
        pos_val[0].as_f64().unwrap_or(0.0),
        pos_val[1].as_f64().unwrap_or(0.0),
        pos_val[2].as_f64().unwrap_or(0.0),
    ];

    let norm_val = args
        .get("normal")
        .and_then(|v| v.as_array())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'normal' [nx, ny, nz]".to_string()))?;

    if norm_val.len() != 3 {
        return Err(ServerError::InvalidArgument("'normal' must have 3 elements".to_string()));
    }
    let normal = [
        norm_val[0].as_f64().unwrap_or(0.0),
        norm_val[1].as_f64().unwrap_or(0.0),
        norm_val[2].as_f64().unwrap_or(1.0),
    ];

    let radius = args
        .get("radius")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'radius'".to_string()))?;

    let depth = args
        .get("depth")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'depth'".to_string()))?;

    let keyway_width = args
        .get("keyway_width")
        .and_then(|v| v.as_f64());
    let keyway_depth = args
        .get("keyway_depth")
        .and_then(|v| v.as_f64());

    let part = state
        .parts
        .get_mut(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    let prev_dag = part.dag.clone();
    let prev_mesh = part.mesh.clone();
    let feature_id = part.dag.add_feature(
        FeatureOp::CutHole {
            center,
            normal,
            radius,
            depth,
            keyway_width,
            keyway_depth,
        },
        None,
    );
    if let Err(e) = part.rebuild() {
        part.dag = prev_dag;
        part.mesh = prev_mesh;
        return Err(ServerError::Cad(e));
    }

    let mesh_clone = part.mesh.clone();
    state.send_gui_mesh(&mesh_clone);

    Ok(json!({
        "feature_id": feature_id.to_string()
    }))
}

/// 6. `modify_feature`: Edits parameters of an existing step in the history DAG.
pub fn modify_feature(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let feat_str = args
        .get("feature_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'feature_id'".to_string()))?;

    let feature_id = Uuid::parse_str(feat_str)
        .map_err(|e| ServerError::InvalidArgument(format!("Invalid UUID 'feature_id': {e}")))?;

    let params = args
        .get("params")
        .or_else(|| args.get("new_params"))
        .unwrap_or(args);

    let part = state
        .parts
        .get_mut(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    let current_node = part
        .dag
        .get_feature(feature_id)
        .ok_or_else(|| ServerError::FeatureNotFound(feat_str.to_string()))?;

    let updated_op = match &current_node.op {
        FeatureOp::AddBox { width, height, depth, position, rotation_deg } => {
            let new_w = params.get("width").and_then(|v| v.as_f64()).unwrap_or(*width);
            let new_h = params.get("height").and_then(|v| v.as_f64()).unwrap_or(*height);
            let new_d = params.get("depth").and_then(|v| v.as_f64()).unwrap_or(*depth);
            let pos = parse_vec3(params, "position").or(*position);
            let rot = parse_vec3(params, "rotation_deg")
                .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                .or(*rotation_deg);
            FeatureOp::AddBox {
                width: new_w,
                height: new_h,
                depth: new_d,
                position: pos,
                rotation_deg: rot,
            }
        }
        FeatureOp::AddCylinder { radius, height, segments, position, rotation_deg } => {
            let new_r = params.get("radius").and_then(|v| v.as_f64()).unwrap_or(*radius);
            let new_h = params.get("height").and_then(|v| v.as_f64()).unwrap_or(*height);
            let new_s = params.get("segments").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*segments);
            let pos = parse_vec3(params, "position").or(*position);
            let rot = parse_vec3(params, "rotation_deg")
                .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                .or(*rotation_deg);
            FeatureOp::AddCylinder {
                radius: new_r,
                height: new_h,
                segments: new_s,
                position: pos,
                rotation_deg: rot,
            }
        }
        FeatureOp::AddSphere { radius, sectors, stacks, position, rotation_deg } => {
            let new_r = params.get("radius").and_then(|v| v.as_f64()).unwrap_or(*radius);
            let new_sec = params.get("sectors").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*sectors);
            let new_stk = params.get("stacks").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*stacks);
            let pos = parse_vec3(params, "position").or(*position);
            let rot = parse_vec3(params, "rotation_deg")
                .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                .or(*rotation_deg);
            FeatureOp::AddSphere {
                radius: new_r,
                sectors: new_sec,
                stacks: new_stk,
                position: pos,
                rotation_deg: rot,
            }
        }
        FeatureOp::AddCone { radius_bottom, radius_top, height, segments, position, rotation_deg } => {
            let new_rb = params.get("radius_bottom").or_else(|| params.get("radius")).and_then(|v| v.as_f64()).unwrap_or(*radius_bottom);
            let new_rt = params.get("radius_top").and_then(|v| v.as_f64()).unwrap_or(*radius_top);
            let new_h = params.get("height").and_then(|v| v.as_f64()).unwrap_or(*height);
            let new_s = params.get("segments").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*segments);
            let pos = parse_vec3(params, "position").or(*position);
            let rot = parse_vec3(params, "rotation_deg")
                .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                .or(*rotation_deg);
            FeatureOp::AddCone {
                radius_bottom: new_rb,
                radius_top: new_rt,
                height: new_h,
                segments: new_s,
                position: pos,
                rotation_deg: rot,
            }
        }
        FeatureOp::AddPolygonPrism {
            sides,
            height,
            width_across_flats,
            radius,
            chamfer,
            position,
            rotation_deg,
        } => {
            let new_sides = params.get("sides").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*sides);
            let new_h = params.get("height").and_then(|v| v.as_f64()).unwrap_or(*height);
            let new_waf = params.get("width_across_flats").or_else(|| params.get("s")).and_then(|v| v.as_f64()).or(*width_across_flats);
            let new_r = params.get("radius").or_else(|| params.get("r")).and_then(|v| v.as_f64()).or(*radius);
            let new_chamfer = params.get("chamfer").and_then(|v| v.as_f64()).or(*chamfer);
            let pos = parse_vec3(params, "position").or(*position);
            let rot = parse_vec3(params, "rotation_deg")
                .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                .or(*rotation_deg);
            FeatureOp::AddPolygonPrism {
                sides: new_sides,
                height: new_h,
                width_across_flats: new_waf,
                radius: new_r,
                chamfer: new_chamfer,
                position: pos,
                rotation_deg: rot,
            }
        }
        FeatureOp::AddSpurGear {
            module,
            teeth,
            pressure_angle_deg,
            face_width,
            bore_radius,
            keyway_width,
            keyway_depth,
            hub_diameter,
            hub_length,
            position,
            rotation_deg,
        } => {
            let new_m = params.get("module").or_else(|| params.get("m")).and_then(|v| v.as_f64()).unwrap_or(*module);
            let new_z = params.get("teeth").or_else(|| params.get("z")).and_then(|v| v.as_u64()).map(|z| z as usize).unwrap_or(*teeth);
            let new_pa = params.get("pressure_angle_deg").or_else(|| params.get("pressure_angle")).and_then(|v| v.as_f64()).unwrap_or(*pressure_angle_deg);
            let new_fw = params.get("face_width").or_else(|| params.get("width")).or_else(|| params.get("b")).and_then(|v| v.as_f64()).unwrap_or(*face_width);
            let new_bore = params.get("bore_radius").and_then(|v| v.as_f64())
                .or_else(|| params.get("bore_diameter").and_then(|v| v.as_f64()).map(|d| d * 0.5))
                .or(*bore_radius);
            let new_kw = params.get("keyway_width").and_then(|v| v.as_f64()).or(*keyway_width);
            let new_kd = params.get("keyway_depth").and_then(|v| v.as_f64()).or(*keyway_depth);
            let new_hd = params.get("hub_diameter").and_then(|v| v.as_f64()).or(*hub_diameter);
            let new_hl = params.get("hub_length").and_then(|v| v.as_f64()).or(*hub_length);
            let pos = parse_vec3(params, "position").or(*position);
            let rot = parse_vec3(params, "rotation_deg")
                .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                .or(*rotation_deg);

            FeatureOp::AddSpurGear {
                module: new_m,
                teeth: new_z,
                pressure_angle_deg: new_pa,
                face_width: new_fw,
                bore_radius: new_bore,
                keyway_width: new_kw,
                keyway_depth: new_kd,
                hub_diameter: new_hd,
                hub_length: new_hl,
                position: pos,
                rotation_deg: rot,
            }
        }
        FeatureOp::AddRevolve { profile, angle_deg, segments, position, rotation_deg } => {
            let new_profile = if let Some(arr) = params.get("profile").and_then(|v| v.as_array()) {
                let mut p = Vec::with_capacity(arr.len());
                for item in arr {
                    if let Some(coords) = item.as_array() {
                        if coords.len() == 2 {
                            if let (Some(r), Some(z)) = (coords[0].as_f64(), coords[1].as_f64()) {
                                p.push([r, z]);
                            }
                        }
                    }
                }
                if !p.is_empty() { p } else { profile.clone() }
            } else {
                profile.clone()
            };
            let new_angle = params.get("angle_deg").or_else(|| params.get("angle")).and_then(|v| v.as_f64()).unwrap_or(*angle_deg);
            let new_segs = params.get("segments").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*segments);
            let pos = parse_vec3(params, "position").or(*position);
            let rot = parse_vec3(params, "rotation_deg")
                .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                .or(*rotation_deg);
            FeatureOp::AddRevolve {
                profile: new_profile,
                angle_deg: new_angle,
                segments: new_segs,
                position: pos,
                rotation_deg: rot,
            }
        }
        FeatureOp::AddWheelRim {
            rim_diameter,
            rim_width,
            flange_diameter,
            hub_diameter,
            hub_bore_diameter,
            pcd_diameter,
            bolt_holes_count,
            bolt_hole_diameter,
            spokes_count,
            spoke_width,
            spoke_thickness,
            offset_et,
            position,
            rotation_deg,
        } => {
            let new_dia = params.get("rim_diameter").or_else(|| params.get("diameter")).and_then(|v| v.as_f64()).unwrap_or(*rim_diameter);
            let new_w = params.get("rim_width").or_else(|| params.get("width")).and_then(|v| v.as_f64()).unwrap_or(*rim_width);
            let new_flange = params.get("flange_diameter").and_then(|v| v.as_f64()).or(*flange_diameter);
            let new_hub = params.get("hub_diameter").and_then(|v| v.as_f64()).or(*hub_diameter);
            let new_bore = params.get("hub_bore_diameter").or_else(|| params.get("bore_diameter")).and_then(|v| v.as_f64()).or(*hub_bore_diameter);
            let new_pcd = params.get("pcd_diameter").or_else(|| params.get("pcd")).and_then(|v| v.as_f64()).or(*pcd_diameter);
            let new_bolts = params.get("bolt_holes_count").or_else(|| params.get("bolts_count")).or_else(|| params.get("bolts")).and_then(|v| v.as_u64()).map(|b| b as usize).or(*bolt_holes_count);
            let new_bolt_d = params.get("bolt_hole_diameter").and_then(|v| v.as_f64()).or(*bolt_hole_diameter);
            let new_spokes = params.get("spokes_count").or_else(|| params.get("spokes")).and_then(|v| v.as_u64()).map(|s| s as usize).or(*spokes_count);
            let new_spoke_w = params.get("spoke_width").and_then(|v| v.as_f64()).or(*spoke_width);
            let new_spoke_t = params.get("spoke_thickness").and_then(|v| v.as_f64()).or(*spoke_thickness);
            let new_et = params.get("offset_et").or_else(|| params.get("et")).and_then(|v| v.as_f64()).or(*offset_et);
            let pos = parse_vec3(params, "position").or(*position);
            let rot = parse_vec3(params, "rotation_deg")
                .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                .or(*rotation_deg);
            FeatureOp::AddWheelRim {
                rim_diameter: new_dia,
                rim_width: new_w,
                flange_diameter: new_flange,
                hub_diameter: new_hub,
                hub_bore_diameter: new_bore,
                pcd_diameter: new_pcd,
                bolt_holes_count: new_bolts,
                bolt_hole_diameter: new_bolt_d,
                spokes_count: new_spokes,
                spoke_width: new_spoke_w,
                spoke_thickness: new_spoke_t,
                offset_et: new_et,
                position: pos,
                rotation_deg: rot,
            }
        }
        FeatureOp::CutHelicalGroove { helix_radius, groove_radius, pitch, height, start_z, starts, is_right_hand, profile_type } => {
            let hr = params.get("helix_radius").and_then(|v| v.as_f64()).unwrap_or(*helix_radius);
            let gr = params.get("groove_radius").and_then(|v| v.as_f64()).unwrap_or(*groove_radius);
            let p = params.get("pitch").and_then(|v| v.as_f64()).unwrap_or(*pitch);
            let h = params.get("height").and_then(|v| v.as_f64()).unwrap_or(*height);
            let sz = params.get("start_z").and_then(|v| v.as_f64()).unwrap_or(*start_z);
            let st = params.get("starts").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*starts);
            let rh = params.get("is_right_hand").and_then(|v| v.as_bool()).unwrap_or(*is_right_hand);
            let pt = params.get("profile_type").and_then(|v| v.as_str()).map(|s| s.to_string()).unwrap_or_else(|| profile_type.clone());
            FeatureOp::CutHelicalGroove {
                helix_radius: hr,
                groove_radius: gr,
                pitch: p,
                height: h,
                start_z: sz,
                starts: st,
                is_right_hand: rh,
                profile_type: pt,
            }
        }
        FeatureOp::BooleanDifferenceMesh { tool_mesh } => {
            FeatureOp::BooleanDifferenceMesh {
                tool_mesh: tool_mesh.clone(),
            }
        }
        FeatureOp::BooleanUnionMesh { tool_mesh } => {
            FeatureOp::BooleanUnionMesh {
                tool_mesh: tool_mesh.clone(),
            }
        }
        FeatureOp::BooleanIntersectionMesh { tool_mesh } => {
            FeatureOp::BooleanIntersectionMesh {
                tool_mesh: tool_mesh.clone(),
            }
        }
        FeatureOp::CutHole { center, normal, radius, depth, keyway_width, keyway_depth } => {
            let new_center = if let Some(arr) = params.get("position").or_else(|| params.get("center")).and_then(|v| v.as_array()) {
                if arr.len() == 3 {
                    [arr[0].as_f64().unwrap_or(center[0]), arr[1].as_f64().unwrap_or(center[1]), arr[2].as_f64().unwrap_or(center[2])]
                } else {
                    *center
                }
            } else {
                *center
            };
            let new_normal = if let Some(arr) = params.get("normal").and_then(|v| v.as_array()) {
                if arr.len() == 3 {
                    [arr[0].as_f64().unwrap_or(normal[0]), arr[1].as_f64().unwrap_or(normal[1]), arr[2].as_f64().unwrap_or(normal[2])]
                } else {
                    *normal
                }
            } else {
                *normal
            };
            let new_r = params.get("radius").and_then(|v| v.as_f64()).unwrap_or(*radius);
            let new_d = params.get("depth").and_then(|v| v.as_f64()).unwrap_or(*depth);
            let new_kw = params.get("keyway_width").and_then(|v| v.as_f64()).or(*keyway_width);
            let new_kd = params.get("keyway_depth").and_then(|v| v.as_f64()).or(*keyway_depth);
            FeatureOp::CutHole {
                center: new_center,
                normal: new_normal,
                radius: new_r,
                depth: new_d,
                keyway_width: new_kw,
                keyway_depth: new_kd,
            }
        }
        FeatureOp::Transform { translation, rotation_deg, scale } => {
            let t = if let Some(arr) = params.get("translation").and_then(|v| v.as_array()) {
                [arr[0].as_f64().unwrap_or(translation[0]), arr[1].as_f64().unwrap_or(translation[1]), arr[2].as_f64().unwrap_or(translation[2])]
            } else {
                *translation
            };
            let r = if let Some(arr) = params.get("rotation_deg").and_then(|v| v.as_array()) {
                [arr[0].as_f64().unwrap_or(rotation_deg[0]), arr[1].as_f64().unwrap_or(rotation_deg[1]), arr[2].as_f64().unwrap_or(rotation_deg[2])]
            } else {
                *rotation_deg
            };
            let s = if let Some(arr) = params.get("scale").and_then(|v| v.as_array()) {
                [arr[0].as_f64().unwrap_or(scale[0]), arr[1].as_f64().unwrap_or(scale[1]), arr[2].as_f64().unwrap_or(scale[2])]
            } else {
                *scale
            };
            FeatureOp::Transform {
                translation: t,
                rotation_deg: r,
                scale: s,
            }
        }
    };

    let prev_dag = part.dag.clone();
    let prev_mesh = part.mesh.clone();
    part.dag.modify_feature(feature_id, updated_op)?;
    if let Err(e) = part.rebuild() {
        part.dag = prev_dag;
        part.mesh = prev_mesh;
        return Err(ServerError::Cad(e));
    }

    let aabb = part.bounding_box();
    let mesh_clone = part.mesh.clone();
    state.send_gui_mesh(&mesh_clone);

    Ok(json!({
        "status": "updated",
        "updated_bounds": [aabb.min[0], aabb.min[1], aabb.min[2], aabb.max[0], aabb.max[1], aabb.max[2]],
        "dimensions": aabb.dimensions()
    }))
}

/// 7. `get_feature_tree`: Returns the complete DAG history nodes of a part.
pub fn get_feature_tree(args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let part = state
        .parts
        .get(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    Ok(json!({
        "dag": part.dag.nodes
    }))
}

/// 8. `cut_helical_groove`: Cuts one or more helical grooves (flutes/threads) into the solid.
pub fn cut_helical_groove(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let params = args.get("params").unwrap_or(args);

    let helix_radius = params
        .get("helix_radius")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'helix_radius'".to_string()))?;

    let groove_radius = params
        .get("groove_radius")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'groove_radius'".to_string()))?;

    let pitch = params
        .get("pitch")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'pitch'".to_string()))?;

    let height = params
        .get("height")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'height'".to_string()))?;

    let start_z = params
        .get("start_z")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);

    let starts = params
        .get("starts")
        .and_then(|v| v.as_u64())
        .map(|s| s as usize)
        .unwrap_or(2);

    let is_right_hand = params
        .get("is_right_hand")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);

    let profile_type = params
        .get("profile_type")
        .and_then(|v| v.as_str())
        .unwrap_or("circular")
        .to_string();

    let part = state
        .parts
        .get_mut(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    let prev_dag = part.dag.clone();
    let prev_mesh = part.mesh.clone();
    let feature_id = part.dag.add_feature(
        FeatureOp::CutHelicalGroove {
            helix_radius,
            groove_radius,
            pitch,
            height,
            start_z,
            starts,
            is_right_hand,
            profile_type,
        },
        None,
    );
    if let Err(e) = part.rebuild() {
        part.dag = prev_dag;
        part.mesh = prev_mesh;
        return Err(ServerError::Cad(e));
    }

    let mesh_clone = part.mesh.clone();
    state.send_gui_mesh(&mesh_clone);

    Ok(json!({
        "feature_id": feature_id.to_string(),
        "status": "helical_groove_cut"
    }))
}

/// 9. `boolean_difference`: Subtracts tool_part from target_part.
pub fn boolean_difference(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let target_part_id = args
        .get("target_part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'target_part_id'".to_string()))?;

    let tool_part_id = args
        .get("tool_part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'tool_part_id'".to_string()))?;

    if target_part_id == tool_part_id {
        return Err(ServerError::InvalidArgument(
            "target_part_id and tool_part_id must be different parts".to_string(),
        ));
    }

    let tool_mesh = {
        let tool_part = state
            .parts
            .get(tool_part_id)
            .ok_or_else(|| ServerError::PartNotFound(tool_part_id.to_string()))?;
        tool_part.mesh.clone()
    };

    let target_part = state
        .parts
        .get_mut(target_part_id)
        .ok_or_else(|| ServerError::PartNotFound(target_part_id.to_string()))?;

    let prev_dag = target_part.dag.clone();
    let prev_mesh = target_part.mesh.clone();
    let feature_id = target_part.dag.add_feature(
        FeatureOp::BooleanDifferenceMesh { tool_mesh },
        None,
    );
    if let Err(e) = target_part.rebuild() {
        target_part.dag = prev_dag;
        target_part.mesh = prev_mesh;
        return Err(ServerError::Cad(e));
    }

    let mesh_clone = target_part.mesh.clone();
    state.send_gui_mesh(&mesh_clone);

    Ok(json!({
        "feature_id": feature_id.to_string(),
        "status": "boolean_difference_applied"
    }))
}

/// `import_stl`: Imports an external 3D STL file (ASCII or binary) as a new Part and displays it in the viewport.
pub fn import_stl(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let filepath = args
        .get("filepath")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'filepath'".to_string()))?;

    let path = std::path::Path::new(filepath);
    let default_name = path
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "imported_stl".to_string());

    let part_name = args
        .get("name")
        .and_then(|v| v.as_str())
        .map(|s| s.to_string())
        .unwrap_or(default_name);

    let material_name = args
        .get("material")
        .and_then(|v| v.as_str())
        .unwrap_or("steel_structural_aisi1020");

    let material = state
        .materials
        .get(material_name)
        .cloned()
        .ok_or_else(|| {
            let available: Vec<_> = state.materials.list().into_iter().map(|m| m.name.as_str()).collect();
            ServerError::InvalidArgument(format!(
                "Material '{}' not found in registry. Available presets: {}",
                material_name,
                available.join(", ")
            ))
        })?;

    let mesh = mcp_cad_core::import_stl_file(path)?;

    let part_id = format!("part-{}", Uuid::new_v4().simple());
    let mut part = Part::new(&part_id, &part_name, material);
    part.mesh = mesh.clone();

    state.send_gui_mesh(&mesh);
    state.parts.insert(part_id.clone(), part);

    Ok(json!({
        "part_id": part_id,
        "name": part_name,
        "vertices_count": mesh.vertices.len(),
        "triangles_count": mesh.indices.len(),
        "bounding_box": mesh.bounding_box(),
        "status": "stl_imported"
    }))
}

/// Helper function to parse tessellation quality preset and tolerances.
fn resolve_tessellation_settings(
    quality_opt: Option<&str>,
    chord_tolerance: Option<f64>,
    angular_tolerance_deg: Option<f64>,
    max_edge_length: Option<f64>,
    default_quality: &str,
) -> Result<(TessellationSettings, String), ServerError> {
    if let Some(c) = chord_tolerance {
        if c <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "chord_tolerance must be greater than 0".to_string(),
            ));
        }
    }
    if let Some(a) = angular_tolerance_deg {
        if a <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "angular_tolerance_deg must be greater than 0".to_string(),
            ));
        }
    }
    if let Some(m) = max_edge_length {
        if m <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "max_edge_length must be greater than 0".to_string(),
            ));
        }
    }

    let q_str = quality_opt.unwrap_or(default_quality).to_lowercase();
    let is_custom = chord_tolerance.is_some() || angular_tolerance_deg.is_some() || max_edge_length.is_some();

    if is_custom {
        let base_settings = match q_str.as_str() {
            "coarse" => TessellationSettings::coarse(),
            "medium" => TessellationSettings::medium(),
            "fine" => TessellationSettings::fine(),
            other if quality_opt.is_some() => {
                return Err(ServerError::InvalidArgument(format!(
                    "Unknown quality preset '{other}'. Supported: coarse, medium, fine"
                )));
            }
            _ => TessellationSettings::fine(),
        };

        let chord_tol = chord_tolerance.unwrap_or(base_settings.chord_tolerance);
        let ang_tol = angular_tolerance_deg.unwrap_or(base_settings.angular_tolerance_deg);
        let max_edge = max_edge_length.or(base_settings.max_edge_length);

        let settings = TessellationSettings::custom(chord_tol, ang_tol, max_edge);
        Ok((settings, "custom".to_string()))
    } else {
        match q_str.as_str() {
            "coarse" => Ok((TessellationSettings::coarse(), "coarse".to_string())),
            "medium" => Ok((TessellationSettings::medium(), "medium".to_string())),
            "fine" => Ok((TessellationSettings::fine(), "fine".to_string())),
            other => Err(ServerError::InvalidArgument(format!(
                "Unknown quality preset '{other}'. Supported: coarse, medium, fine"
            ))),
        }
    }
}

/// `export_stl`: Exports a part's boundary surface mesh to an STL file (ASCII or Binary) with adaptive tessellation.
pub fn export_stl(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
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

    let quality = args
        .get("quality")
        .and_then(|v| v.as_str());

    let chord_tolerance = args
        .get("chord_tolerance")
        .and_then(|v| v.as_f64());

    let (settings, quality_applied) = resolve_tessellation_settings(
        quality,
        chord_tolerance,
        None,
        None,
        "fine",
    )?;

    let part = state
        .parts
        .get_mut(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    if !part.dag.nodes.is_empty() {
        part.tessellate(&settings)?;
    }

    if part.mesh().vertices.is_empty() {
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

    let mesh = part.mesh();
    if binary {
        write_stl_binary_file(mesh, path)?;
    } else {
        write_stl_ascii_file(mesh, &part.name, path)?;
    }

    let triangles_count = mesh.triangle_count();

    Ok(json!({
        "filepath": filepath,
        "file_path": filepath,
        "triangles_count": triangles_count,
        "triangle_count": triangles_count,
        "quality": quality_applied,
        "quality_applied": quality_applied
    }))
}

/// `tessellate_part`: Evaluates lazy adaptive tessellation on a CAD part with specified quality preset or custom tolerances.
pub fn tessellate_part(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let quality = args
        .get("quality")
        .and_then(|v| v.as_str());

    let chord_tolerance = args
        .get("chord_tolerance")
        .and_then(|v| v.as_f64());

    let angular_tolerance_deg = args
        .get("angular_tolerance_deg")
        .and_then(|v| v.as_f64());

    let max_edge_length = args
        .get("max_edge_length")
        .and_then(|v| v.as_f64());

    let (settings, quality_applied) = resolve_tessellation_settings(
        quality,
        chord_tolerance,
        angular_tolerance_deg,
        max_edge_length,
        "medium",
    )?;

    let (mesh_clone, volume) = {
        let part = state
            .parts
            .get_mut(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

        if !part.dag.nodes.is_empty() {
            part.tessellate(&settings)?;
        }

        (part.mesh().clone(), part.volume())
    };

    state.send_gui_mesh(&mesh_clone);

    let vertex_count = mesh_clone.vertices.len();
    let triangle_count = mesh_clone.triangle_count();

    Ok(json!({
        "part_id": part_id,
        "quality_applied": quality_applied,
        "vertex_count": vertex_count,
        "vertices_count": vertex_count,
        "triangle_count": triangle_count,
        "triangles_count": triangle_count,
        "volume": volume
    }))
}

/// 21. `query_part_geometry`: Detailed geometry inspection of a part.
pub fn query_part_geometry(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let part = state
        .parts
        .get(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    let inspection = part.inspect_geometry();
    let mesh_validation = mcp_cad_core::validate_mesh(&part.mesh, None);

    Ok(json!({
        "part_id": part_id,
        "bounding_box": {
            "min": inspection.bounding_box.min,
            "max": inspection.bounding_box.max,
            "dimensions": inspection.bounding_box.dimensions(),
        },
        "volume": inspection.volume,
        "surface_area": inspection.surface_area,
        "center_of_mass": inspection.center_of_mass,
        "faces_count": inspection.faces_count,
        "vertices_count": inspection.vertices_count,
        "detected_planar_faces": inspection.detected_planar_faces,
        "mesh_validation": mesh_validation,
    }))
}

/// Dev/diagnostic tool: `validate_mesh`: Performs thorough topology, manifoldness, and geometry validation.
pub fn validate_mesh_tool(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let part = state
        .parts
        .get(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    let expected_genus = args
        .get("expected_genus")
        .and_then(|v| v.as_i64())
        .map(|v| v as i32);

    let report = mcp_cad_core::validate_mesh(&part.mesh, expected_genus);
    serde_json::to_value(&report).map_err(ServerError::Json)
}

/// 22. `boolean_union`: Combines tool_part into target_part using CSG union.
pub fn boolean_union(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let target_part_id = args
        .get("target_part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'target_part_id'".to_string()))?;

    let tool_part_id = args
        .get("tool_part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'tool_part_id'".to_string()))?;

    if target_part_id == tool_part_id {
        return Err(ServerError::InvalidArgument(
            "target_part_id and tool_part_id must be different parts".to_string(),
        ));
    }

    let tool_mesh = {
        let tool_part = state
            .parts
            .get(tool_part_id)
            .ok_or_else(|| ServerError::PartNotFound(tool_part_id.to_string()))?;
        tool_part.mesh().clone()
    };

    let (feature_id, vertices_count, triangles_count, mesh_clone) = {
        let target_part = state
            .parts
            .get_mut(target_part_id)
            .ok_or_else(|| ServerError::PartNotFound(target_part_id.to_string()))?;

        let prev_dag = target_part.dag.clone();
        let prev_mesh = target_part.mesh.clone();
        let feature_id = target_part.dag.add_feature(
            FeatureOp::BooleanUnionMesh { tool_mesh },
            None,
        );
        if let Err(e) = target_part.rebuild() {
            target_part.dag = prev_dag;
            target_part.mesh = prev_mesh;
            return Err(ServerError::Cad(e));
        }

        let mesh = target_part.mesh().clone();
        (feature_id, mesh.vertices.len(), mesh.triangle_count(), mesh)
    };

    state.send_gui_mesh(&mesh_clone);

    let _ = state.physics_world.remove_part(target_part_id);
    if let Some(target_part) = state.parts.get(target_part_id) {
        if !target_part.mesh().vertices.is_empty() {
            let _ = state.physics_world.add_part(target_part, true);
        }
    }

    Ok(json!({
        "target_part_id": target_part_id,
        "feature_id": feature_id.to_string(),
        "status": "union_applied",
        "vertices_count": vertices_count,
        "triangles_count": triangles_count,
    }))
}

/// 23. `boolean_intersection`: Intersects tool_part with target_part using CSG intersection.
pub fn boolean_intersection(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let target_part_id = args
        .get("target_part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'target_part_id'".to_string()))?;

    let tool_part_id = args
        .get("tool_part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'tool_part_id'".to_string()))?;

    if target_part_id == tool_part_id {
        return Err(ServerError::InvalidArgument(
            "target_part_id and tool_part_id must be different parts".to_string(),
        ));
    }

    let tool_mesh = {
        let tool_part = state
            .parts
            .get(tool_part_id)
            .ok_or_else(|| ServerError::PartNotFound(tool_part_id.to_string()))?;
        tool_part.mesh().clone()
    };

    let (feature_id, vertices_count, triangles_count, mesh_clone) = {
        let target_part = state
            .parts
            .get_mut(target_part_id)
            .ok_or_else(|| ServerError::PartNotFound(target_part_id.to_string()))?;

        let prev_dag = target_part.dag.clone();
        let prev_mesh = target_part.mesh.clone();
        let feature_id = target_part.dag.add_feature(
            FeatureOp::BooleanIntersectionMesh { tool_mesh },
            None,
        );
        if let Err(e) = target_part.rebuild() {
            target_part.dag = prev_dag;
            target_part.mesh = prev_mesh;
            return Err(ServerError::Cad(e));
        }

        let mesh = target_part.mesh().clone();
        (feature_id, mesh.vertices.len(), mesh.triangle_count(), mesh)
    };

    state.send_gui_mesh(&mesh_clone);

    let _ = state.physics_world.remove_part(target_part_id);
    if let Some(target_part) = state.parts.get(target_part_id) {
        if !target_part.mesh().vertices.is_empty() {
            let _ = state.physics_world.add_part(target_part, true);
        }
    }

    Ok(json!({
        "target_part_id": target_part_id,
        "feature_id": feature_id.to_string(),
        "status": "intersection_applied",
        "vertices_count": vertices_count,
        "triangles_count": triangles_count,
    }))
}

/// 24. `transform_part`: Applies translation, rotation (RPY deg), and scale to a part's DAG.
pub fn transform_part(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let translation = parse_vec3(args, "translation").unwrap_or([0.0, 0.0, 0.0]);
    let rotation_deg = parse_vec3(args, "rotation_deg")
        .or_else(|| parse_vec3(args, "rotation_rpy_deg"))
        .unwrap_or([0.0, 0.0, 0.0]);
    let scale = parse_vec3(args, "scale").unwrap_or([1.0, 1.0, 1.0]);

    if scale[0] == 0.0 || scale[1] == 0.0 || scale[2] == 0.0 {
        return Err(ServerError::InvalidArgument(
            "Scale components cannot be zero".to_string(),
        ));
    }

    let (feature_id, aabb, volume, mesh_clone) = {
        let part = state
            .parts
            .get_mut(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

        let prev_dag = part.dag.clone();
        let prev_mesh = part.mesh.clone();
        let feature_id = part.dag.add_feature(
            FeatureOp::Transform {
                translation,
                rotation_deg,
                scale,
            },
            None,
        );
        if let Err(e) = part.rebuild() {
            part.dag = prev_dag;
            part.mesh = prev_mesh;
            return Err(ServerError::Cad(e));
        }

        let aabb = part.bounding_box();
        let volume = part.volume();
        let mesh = part.mesh().clone();
        (feature_id, aabb, volume, mesh)
    };

    state.send_gui_mesh(&mesh_clone);

    let _ = state.physics_world.remove_part(part_id);
    if let Some(part) = state.parts.get(part_id) {
        if !part.mesh().vertices.is_empty() {
            let _ = state.physics_world.add_part(part, true);
        }
    }

    Ok(json!({
        "part_id": part_id,
        "feature_id": feature_id.to_string(),
        "status": "transformed",
        "bounding_box": {
            "min": aabb.min,
            "max": aabb.max,
            "dimensions": aabb.dimensions(),
        },
        "volume": volume,
    }))
}

/// 25. `define_material`: Registers a new engineering material with physical and mechanical properties.
pub fn define_material(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let id = args
        .get("id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'id'".to_string()))?;

    if id.trim().is_empty() {
        return Err(ServerError::InvalidArgument("Material 'id' cannot be empty".to_string()));
    }

    let young_modulus_gpa = args
        .get("young_modulus_gpa")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'young_modulus_gpa'".to_string()))?;

    if young_modulus_gpa <= 0.0 || !young_modulus_gpa.is_finite() {
        return Err(ServerError::InvalidArgument("Young's modulus must be greater than zero".to_string()));
    }

    let poisson_ratio = args
        .get("poisson_ratio")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'poisson_ratio'".to_string()))?;

    if poisson_ratio <= 0.0 || poisson_ratio >= 0.5 || !poisson_ratio.is_finite() {
        return Err(ServerError::InvalidArgument("Poisson's ratio must be in range (0.0, 0.5)".to_string()));
    }

    let yield_strength_mpa = args
        .get("yield_strength_mpa")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'yield_strength_mpa'".to_string()))?;

    if yield_strength_mpa <= 0.0 || !yield_strength_mpa.is_finite() {
        return Err(ServerError::InvalidArgument("Yield strength must be greater than zero".to_string()));
    }

    let density_kg_m3 = args
        .get("density_kg_m3")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'density_kg_m3'".to_string()))?;

    if density_kg_m3 <= 0.0 || !density_kg_m3.is_finite() {
        return Err(ServerError::InvalidArgument("Density must be greater than zero".to_string()));
    }

    let name = args.get("name").and_then(|v| v.as_str()).map(|s| s.to_string());
    let desc = name.clone().unwrap_or_else(|| id.to_string());

    let material = Material::new(
        id,
        young_modulus_gpa,
        poisson_ratio,
        yield_strength_mpa,
        density_kg_m3,
        &desc,
    );

    state.materials.register(material);

    if let Some(ref custom_name) = name {
        if custom_name != id {
            let alias_mat = Material::new(
                custom_name.clone(),
                young_modulus_gpa,
                poisson_ratio,
                yield_strength_mpa,
                density_kg_m3,
                format!("Alias for {}", id),
            );
            state.materials.register(alias_mat);
        }
    }

    Ok(json!({
        "status": "material_registered",
        "material_id": id,
        "properties": {
            "name": desc,
            "young_modulus_gpa": young_modulus_gpa,
            "young_modulus_pa": young_modulus_gpa * 1.0e9,
            "poisson_ratio": poisson_ratio,
            "yield_strength_mpa": yield_strength_mpa,
            "yield_strength_pa": yield_strength_mpa * 1.0e6,
            "density_kg_m3": density_kg_m3,
        }
    }))
}

/// 26. `batch_cad_operations`: Sequentially executes a batch of MCP CAD operations.
pub fn batch_cad_operations(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let operations = args
        .get("operations")
        .and_then(|v| v.as_array())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required array 'operations'".to_string()))?;

    let prev_parts = state.parts.clone();
    let prev_materials = state.materials.clone();
    let prev_last_fea_reports = state.last_fea_reports.clone();
    let prev_last_fea_meshes = state.last_fea_meshes.clone();
    let prev_last_simulation = state.last_simulation.clone();

    let mut results = Vec::with_capacity(operations.len());

    for (idx, op) in operations.iter().enumerate() {
        let tool_name = match op.get("tool").or_else(|| op.get("name")).and_then(|v| v.as_str()) {
            Some(name) => name,
            None => {
                state.parts = prev_parts;
                state.materials = prev_materials;
                state.last_fea_reports = prev_last_fea_reports;
                state.last_fea_meshes = prev_last_fea_meshes;
                state.last_simulation = prev_last_simulation;
                return Err(ServerError::InvalidArgument(format!("Operation at index {idx} missing 'tool' field")));
            }
        };

        if tool_name == "batch_cad_operations" {
            state.parts = prev_parts;
            state.materials = prev_materials;
            state.last_fea_reports = prev_last_fea_reports;
            state.last_fea_meshes = prev_last_fea_meshes;
            state.last_simulation = prev_last_simulation;
            return Err(ServerError::InvalidParameter(format!(
                "Nested 'batch_cad_operations' is not permitted (at index {idx})"
            )));
        }

        let empty_params = json!({});
        let params = op
            .get("params")
            .or_else(|| op.get("args"))
            .unwrap_or(&empty_params);

        match super::dispatch_tool(tool_name, params, state) {
            Ok(call_result) => {
                let val = if let Some(crate::protocol::ToolContent::Text { text }) = call_result.content.first() {
                    serde_json::from_str::<Value>(text).unwrap_or_else(|_| json!(text))
                } else {
                    serde_json::to_value(&call_result).unwrap_or(json!({}))
                };
                results.push(val);
            }
            Err(err) => {
                state.parts = prev_parts;
                state.materials = prev_materials;
                state.last_fea_reports = prev_last_fea_reports;
                state.last_fea_meshes = prev_last_fea_meshes;
                state.last_simulation = prev_last_simulation;
                return Err(ServerError::InvalidArgument(format!(
                    "Batch operation at index {idx} ('{tool_name}') failed: {err}"
                )));
            }
        }
    }

    Ok(json!({
        "success": true,
        "count": operations.len(),
        "results": results,
    }))
}
