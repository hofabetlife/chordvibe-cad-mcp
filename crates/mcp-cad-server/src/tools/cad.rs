//! CAD part and parametric DAG modeling tools.

use serde_json::{json, Value};
use uuid::Uuid;

use mcp_cad_core::dag::FeatureOp;
use mcp_cad_core::part::Part;

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

    let part_id = format!("part-{}", Uuid::new_v4().simple());
    let part = Part::new(&part_id, name, material);

    state.parts.insert(part_id.clone(), part);

    Ok(json!({
        "part_id": part_id
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

    let feature_id = part.dag.add_feature(op, None);
    part.rebuild()?;

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
    part.rebuild()?;

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

    part.dag.modify_feature(feature_id, updated_op)?;
    part.rebuild()?;

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
    part.rebuild()?;

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

    let feature_id = target_part.dag.add_feature(
        FeatureOp::BooleanDifferenceMesh { tool_mesh },
        None,
    );
    target_part.rebuild()?;

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
