//! CAD part and parametric DAG modeling tools.

use std::path::Path;
use serde_json::{json, Value};
use uuid::Uuid;

use mcp_cad_core::dag::{FeatureNode, FeatureOp};
use mcp_cad_core::export_stl::{write_stl_ascii_file, write_stl_binary_file};
use mcp_cad_core::material::Material;
use mcp_cad_core::part::Part;
use mcp_cad_core::tessellation::TessellationSettings;

use crate::error::ServerError;
use crate::state::{EngineState, LengthUnit};

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

/// Scales a 3D vector by `factor`.
fn scale_vec3(v: [f64; 3], factor: f64) -> [f64; 3] {
    [v[0] * factor, v[1] * factor, v[2] * factor]
}

/// Scales a `TriangleMesh` stored in internal SI meters (`m`) to the requested output `LengthUnit` (`k_from_m`).
fn scale_mesh_from_meters(mesh: &mcp_cad_core::TriangleMesh, k_from_m: f64) -> mcp_cad_core::TriangleMesh {
    let mut scaled = mesh.clone();
    if (k_from_m - 1.0).abs() <= 1e-12 {
        return scaled;
    }
    let k_area = k_from_m.powi(2);
    for v in &mut scaled.vertices {
        v.position = scale_vec3(v.position, k_from_m);
    }
    for face in &mut scaled.faces {
        face.area *= k_area;
        face.centroid = scale_vec3(face.centroid, k_from_m);
        face.sample_point = scale_vec3(face.sample_point, k_from_m);
        if let Some(ref mut r) = face.radius {
            *r *= k_from_m;
        }
    }
    scaled
}

/// Converts a `FeatureOp` stored in internal SI meters (`m`) to the requested output `LengthUnit` (`k_from_m`).
fn scale_feature_op_from_meters(op: &FeatureOp, k_from_m: f64) -> FeatureOp {
    match op {
        FeatureOp::AddBox {
            width,
            height,
            depth,
            position,
            rotation_deg,
        } => FeatureOp::AddBox {
            width: width * k_from_m,
            height: height * k_from_m,
            depth: depth * k_from_m,
            position: position.map(|p| scale_vec3(p, k_from_m)),
            rotation_deg: *rotation_deg,
        },
        FeatureOp::AddCylinder {
            radius,
            height,
            segments,
            position,
            rotation_deg,
        } => FeatureOp::AddCylinder {
            radius: radius * k_from_m,
            height: height * k_from_m,
            segments: *segments,
            position: position.map(|p| scale_vec3(p, k_from_m)),
            rotation_deg: *rotation_deg,
        },
        FeatureOp::AddSphere {
            radius,
            sectors,
            stacks,
            position,
            rotation_deg,
        } => FeatureOp::AddSphere {
            radius: radius * k_from_m,
            sectors: *sectors,
            stacks: *stacks,
            position: position.map(|p| scale_vec3(p, k_from_m)),
            rotation_deg: *rotation_deg,
        },
        FeatureOp::AddCone {
            radius_bottom,
            radius_top,
            height,
            segments,
            position,
            rotation_deg,
        } => FeatureOp::AddCone {
            radius_bottom: radius_bottom * k_from_m,
            radius_top: radius_top * k_from_m,
            height: height * k_from_m,
            segments: *segments,
            position: position.map(|p| scale_vec3(p, k_from_m)),
            rotation_deg: *rotation_deg,
        },
        FeatureOp::AddPolygonPrism {
            sides,
            height,
            width_across_flats,
            radius,
            chamfer,
            position,
            rotation_deg,
        } => FeatureOp::AddPolygonPrism {
            sides: *sides,
            height: height * k_from_m,
            width_across_flats: width_across_flats.map(|w| w * k_from_m),
            radius: radius.map(|r| r * k_from_m),
            chamfer: chamfer.map(|c| c * k_from_m),
            position: position.map(|p| scale_vec3(p, k_from_m)),
            rotation_deg: *rotation_deg,
        },
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
        } => FeatureOp::AddSpurGear {
            module: module * k_from_m,
            teeth: *teeth,
            pressure_angle_deg: *pressure_angle_deg,
            face_width: face_width * k_from_m,
            bore_radius: bore_radius.map(|r| r * k_from_m),
            keyway_width: keyway_width.map(|w| w * k_from_m),
            keyway_depth: keyway_depth.map(|d| d * k_from_m),
            hub_diameter: hub_diameter.map(|d| d * k_from_m),
            hub_length: hub_length.map(|l| l * k_from_m),
            position: position.map(|p| scale_vec3(p, k_from_m)),
            rotation_deg: *rotation_deg,
        },
        FeatureOp::AddRevolve {
            profile,
            angle_deg,
            segments,
            position,
            rotation_deg,
        } => FeatureOp::AddRevolve {
            profile: profile
                .iter()
                .map(|pt| [pt[0] * k_from_m, pt[1] * k_from_m])
                .collect(),
            angle_deg: *angle_deg,
            segments: *segments,
            position: position.map(|p| scale_vec3(p, k_from_m)),
            rotation_deg: *rotation_deg,
        },
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
        } => FeatureOp::AddWheelRim {
            rim_diameter: rim_diameter * k_from_m,
            rim_width: rim_width * k_from_m,
            flange_diameter: flange_diameter.map(|v| v * k_from_m),
            hub_diameter: hub_diameter.map(|v| v * k_from_m),
            hub_bore_diameter: hub_bore_diameter.map(|v| v * k_from_m),
            pcd_diameter: pcd_diameter.map(|v| v * k_from_m),
            bolt_holes_count: *bolt_holes_count,
            bolt_hole_diameter: bolt_hole_diameter.map(|v| v * k_from_m),
            spokes_count: *spokes_count,
            spoke_width: spoke_width.map(|v| v * k_from_m),
            spoke_thickness: spoke_thickness.map(|v| v * k_from_m),
            offset_et: offset_et.map(|v| v * k_from_m),
            position: position.map(|p| scale_vec3(p, k_from_m)),
            rotation_deg: *rotation_deg,
        },
        FeatureOp::CutHelicalGroove {
            helix_radius,
            groove_radius,
            pitch,
            height,
            start_z,
            starts,
            is_right_hand,
            profile_type,
        } => FeatureOp::CutHelicalGroove {
            helix_radius: helix_radius * k_from_m,
            groove_radius: groove_radius * k_from_m,
            pitch: pitch * k_from_m,
            height: height * k_from_m,
            start_z: start_z * k_from_m,
            starts: *starts,
            is_right_hand: *is_right_hand,
            profile_type: profile_type.clone(),
        },
        FeatureOp::BooleanDifferenceMesh { tool_mesh } => FeatureOp::BooleanDifferenceMesh {
            tool_mesh: scale_mesh_from_meters(tool_mesh, k_from_m),
        },
        FeatureOp::BooleanUnionMesh { tool_mesh } => FeatureOp::BooleanUnionMesh {
            tool_mesh: scale_mesh_from_meters(tool_mesh, k_from_m),
        },
        FeatureOp::BooleanIntersectionMesh { tool_mesh } => FeatureOp::BooleanIntersectionMesh {
            tool_mesh: scale_mesh_from_meters(tool_mesh, k_from_m),
        },
        FeatureOp::ImportedMesh { mesh } => FeatureOp::ImportedMesh {
            mesh: scale_mesh_from_meters(mesh, k_from_m),
        },
        FeatureOp::CutHole {
            center,
            normal,
            radius,
            depth,
            keyway_width,
            keyway_depth,
        } => FeatureOp::CutHole {
            center: scale_vec3(*center, k_from_m),
            normal: *normal,
            radius: radius * k_from_m,
            depth: depth * k_from_m,
            keyway_width: keyway_width.map(|w| w * k_from_m),
            keyway_depth: keyway_depth.map(|d| d * k_from_m),
        },
        FeatureOp::Transform {
            translation,
            rotation_deg,
            scale,
        } => FeatureOp::Transform {
            translation: scale_vec3(*translation, k_from_m),
            rotation_deg: *rotation_deg,
            scale: *scale,
        },
    }
}

/// Formats a `Part` summary in the active `LengthUnit` while preserving SI `volume_m3` and `mass_kg`.
pub(crate) fn part_summary_in_unit(part: &Part, unit: LengthUnit) -> Value {
    let s = part.summary();
    let aabb = part.bounding_box();
    let k_from_m = unit.from_meters_factor();
    let k_vol = k_from_m.powi(3);
    json!({
        "id": s.id,
        "part_id": s.id,
        "name": s.name,
        "material_name": s.material_name,
        "material": s.material_name,
        "bounding_box": {
            "min": scale_vec3(aabb.min, k_from_m),
            "max": scale_vec3(aabb.max, k_from_m),
        },
        "bounding_box_dimensions": scale_vec3(s.bounding_box_dimensions, k_from_m),
        "bounding_box_dimensions_m": s.bounding_box_dimensions,
        "volume_m3": s.volume,
        "volume": s.volume * k_vol,
        "mass": s.mass,
        "mass_kg": s.mass,
        "triangle_count": s.triangle_count,
        "triangles_count": s.triangle_count,
        "features_count": part.dag.nodes.len(),
        "is_fixed": s.is_fixed,
        "revision": part.revision,
        "length_unit": unit.as_str(),
    })
}

/// 29. `set_units`: Sets the active session linear unit (`"mm"` or `"m"`).
pub fn set_units(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let unit_str = args
        .get("length_unit")
        .or_else(|| args.get("unit"))
        .and_then(|v| v.as_str())
        .ok_or_else(|| {
            ServerError::InvalidArgument(
                "Missing required argument 'length_unit' ('mm' or 'm')".to_string(),
            )
        })?;

    let prev_unit = state.length_unit;
    let unit = LengthUnit::parse(unit_str)?;
    state.length_unit = unit;

    Ok(json!({
        "status": "units_updated",
        "previous_length_unit": prev_unit.as_str(),
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
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

    if part_id.eq_ignore_ascii_case("ground") || part_id.eq_ignore_ascii_case("world") {
        return Err(ServerError::InvalidArgument(format!(
            "Part id '{part_id}' is reserved ('ground' and 'world' are reserved static physics anchors)"
        )));
    }

    if state.parts.contains_key(&part_id) {
        return Err(ServerError::InvalidArgument(format!("Part with id '{}' already exists", part_id)));
    }

    let is_fixed = args.get("is_fixed").and_then(|v| v.as_bool()).unwrap_or(false);
    let mut part = Part::new(&part_id, name, material);
    part.is_fixed = is_fixed;

    state.parts.insert(part_id.clone(), part);
    let unit = state.length_unit;

    Ok(json!({
        "part_id": part_id,
        "is_fixed": is_fixed,
        "revision": 0,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// 2. `list_parts`: Returns summary metadata of all active parts in the engine.
pub fn list_parts(args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;
    let mut sorted_parts: Vec<&Part> = state.parts.values().collect();
    sorted_parts.sort_by(|a, b| a.id.cmp(&b.id));
    let summaries: Vec<Value> = sorted_parts
        .into_iter()
        .map(|p| part_summary_in_unit(p, unit))
        .collect();
    Ok(json!({
        "parts_count": summaries.len(),
        "parts": summaries,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// 3. `delete_part`: Deletes a part from state, physics world, and analysis caches.
pub fn delete_part(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    if part_id.eq_ignore_ascii_case("ground") || part_id.eq_ignore_ascii_case("world") {
        return Err(ServerError::InvalidArgument(format!(
            "Cannot delete reserved static anchor '{part_id}'"
        )));
    }

    if state.parts.remove(part_id).is_none() {
        return Err(ServerError::PartNotFound(part_id.to_string()));
    }

    let _ = state.physics_world.remove_part(part_id);
    state.last_fea_reports.remove(part_id);
    state.last_fea_revisions.remove(part_id);
    state.fea_case_reports.retain(|(pid, _), _| pid != part_id);
    state.last_fea_meshes.remove(part_id);
    state.requirements.part_requirements.remove(part_id);
    state.requirements.part_auto_check.remove(part_id);

    let mut resp = json!({
        "status": "deleted",
        "part_id": part_id,
        "deleted_part_id": part_id
    });
    super::requirements::attach_auto_check_if_enabled(state, None, &mut resp);
    Ok(resp)
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

    let params = args
        .get("params")
        .or_else(|| args.get("parameters"))
        .unwrap_or(args);
    let unit = if params.get("length_unit").is_some() {
        state.resolve_length_unit(params)?
    } else {
        state.resolve_length_unit(args)?
    };
    let k_to_m = unit.to_meters_factor();

    let position = parse_vec3(params, "position")
        .or_else(|| parse_vec3(args, "position"))
        .map(|p| scale_vec3(p, k_to_m));
    let rotation_deg = parse_vec3(params, "rotation_deg")
        .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
        .or_else(|| parse_vec3(args, "rotation_deg"))
        .or_else(|| parse_vec3(args, "rotation_rpy_deg"));

    let mut extra_dims_m: Vec<f64> = Vec::new();
    let mut wheel_rim_diameter_m: Option<f64> = None;

    let op = match prim_type.to_lowercase().as_str() {
        "box" => {
            let width = params
                .get("width")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Box requires 'width'".to_string()))?
                * k_to_m;
            let height = params
                .get("height")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Box requires 'height'".to_string()))?
                * k_to_m;
            let depth = params
                .get("depth")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Box requires 'depth'".to_string()))?
                * k_to_m;
            extra_dims_m.extend_from_slice(&[width, height, depth]);
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
                .ok_or_else(|| ServerError::InvalidArgument("Cylinder requires 'radius'".to_string()))?
                * k_to_m;
            let height = params
                .get("height")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Cylinder requires 'height'".to_string()))?
                * k_to_m;
            let segments = params
                .get("segments")
                .and_then(|v| v.as_u64())
                .unwrap_or(32) as usize;
            extra_dims_m.extend_from_slice(&[radius * 2.0, height]);
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
                .ok_or_else(|| ServerError::InvalidArgument("Sphere requires 'radius'".to_string()))?
                * k_to_m;
            let sectors = params
                .get("sectors")
                .and_then(|v| v.as_u64())
                .unwrap_or(32) as usize;
            let stacks = params
                .get("stacks")
                .and_then(|v| v.as_u64())
                .unwrap_or(16) as usize;
            extra_dims_m.push(radius * 2.0);
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
                .ok_or_else(|| ServerError::InvalidArgument("Cone requires 'radius_bottom'".to_string()))?
                * k_to_m;
            let radius_top = params
                .get("radius_top")
                .and_then(|v| v.as_f64())
                .unwrap_or(0.0)
                * k_to_m;
            let height = params
                .get("height")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Cone requires 'height'".to_string()))?
                * k_to_m;
            let segments = params
                .get("segments")
                .and_then(|v| v.as_u64())
                .unwrap_or(32) as usize;
            extra_dims_m.extend_from_slice(&[radius_bottom * 2.0, height]);
            if radius_top > 0.0 {
                extra_dims_m.push(radius_top * 2.0);
            }
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
                .ok_or_else(|| ServerError::InvalidArgument("Polygon prism requires 'height'".to_string()))?
                * k_to_m;
            let sides = params
                .get("sides")
                .and_then(|v| v.as_u64())
                .unwrap_or(6) as usize;
            let width_across_flats = params
                .get("width_across_flats")
                .or_else(|| params.get("s"))
                .and_then(|v| v.as_f64())
                .map(|w| w * k_to_m);
            let radius = params
                .get("radius")
                .or_else(|| params.get("outer_radius"))
                .or_else(|| params.get("r"))
                .and_then(|v| v.as_f64())
                .map(|r| r * k_to_m);
            if width_across_flats.is_none() && radius.is_none() {
                return Err(ServerError::InvalidArgument(
                    "Polygon prism requires either 'width_across_flats' or 'radius' (or 'outer_radius')".to_string(),
                ));
            }
            let chamfer = params
                .get("chamfer")
                .or_else(|| params.get("chamfer_radius"))
                .and_then(|v| v.as_f64())
                .map(|c| c * k_to_m);
            extra_dims_m.push(height);
            if let Some(w) = width_across_flats {
                extra_dims_m.push(w);
            }
            if let Some(r) = radius {
                extra_dims_m.push(r * 2.0);
            }
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
                .ok_or_else(|| ServerError::InvalidArgument("Spur gear requires 'module'".to_string()))?
                * k_to_m;
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
                .ok_or_else(|| ServerError::InvalidArgument("Spur gear requires 'face_width'".to_string()))?
                * k_to_m;
            let bore_radius = params
                .get("bore_radius")
                .and_then(|v| v.as_f64())
                .or_else(|| params.get("bore_diameter").and_then(|v| v.as_f64()).map(|d| d * 0.5))
                .map(|r| r * k_to_m);
            let keyway_width = params
                .get("keyway_width")
                .and_then(|v| v.as_f64())
                .map(|w| w * k_to_m);
            let keyway_depth = params
                .get("keyway_depth")
                .and_then(|v| v.as_f64())
                .map(|d| d * k_to_m);
            let hub_diameter = params
                .get("hub_diameter")
                .and_then(|v| v.as_f64())
                .map(|d| d * k_to_m);
            let hub_length = params
                .get("hub_length")
                .and_then(|v| v.as_f64())
                .map(|l| l * k_to_m);

            extra_dims_m.extend_from_slice(&[module * (teeth as f64), face_width]);

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

            let inner_radius_opt = params.get("inner_radius").and_then(|v| v.as_f64());

            if profile_val.len() < 3 && !(profile_val.len() == 2 && inner_radius_opt.is_some()) {
                return Err(ServerError::InvalidArgument(
                    "Revolve 'profile' must contain at least 3 [r, z] points (or 2 outer points when 'inner_radius' is specified)".to_string(),
                ));
            }

            let mut parsed_points = Vec::with_capacity(profile_val.len());
            for item in profile_val {
                let arr = item.as_array().ok_or_else(|| {
                    ServerError::InvalidArgument("Each profile element must be [r, z]".to_string())
                })?;
                if arr.len() != 2 {
                    return Err(ServerError::InvalidArgument("Profile points must have 2 coordinates: [r, z]".to_string()));
                }
                let r = arr[0].as_f64().ok_or_else(|| ServerError::InvalidArgument("Invalid r coordinate in profile".to_string()))?;
                let z = arr[1].as_f64().ok_or_else(|| ServerError::InvalidArgument("Invalid z coordinate in profile".to_string()))?;
                if !r.is_finite() || !z.is_finite() {
                    return Err(ServerError::InvalidArgument("Profile coordinates [r, z] must be finite numbers".to_string()));
                }
                if r < -1e-9 {
                    return Err(ServerError::InvalidArgument(format!(
                        "Profile coordinate r ({r}) cannot be negative (crosses rotation axis)"
                    )));
                }
                parsed_points.push([r.max(0.0), z]);
            }

            let raw_profile = if parsed_points.len() == 2 {
                let inner_r = inner_radius_opt.unwrap();
                if !inner_r.is_finite() || inner_r < 0.0 {
                    return Err(ServerError::InvalidArgument(
                        "Revolve 'inner_radius' must be a finite non-negative number (>= 0)".to_string(),
                    ));
                }
                let [r0, z0] = parsed_points[0];
                let [r1, z1] = parsed_points[1];
                if r0 <= inner_r + 1e-9 || r1 <= inner_r + 1e-9 {
                    return Err(ServerError::InvalidArgument(format!(
                        "Revolve outer profile radii ({r0}, {r1}) must be strictly greater than 'inner_radius' ({inner_r})"
                    )));
                }
                if (z1 - z0).abs() <= 1e-9 {
                    return Err(ServerError::InvalidArgument(
                        "Revolve 2-point profile with 'inner_radius' requires distinct z coordinates (z0 != z1)".to_string(),
                    ));
                }
                vec![
                    [inner_r, z0],
                    [r0, z0],
                    [r1, z1],
                    [inner_r, z1],
                ]
            } else {
                parsed_points
            };

            // Reject self-intersecting 2D contour segments in the unscaled (r, z) plane
            let n_pts = raw_profile.len();
            let segments_intersect_2d = |a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]| -> bool {
                let ccw = |p1: [f64; 2], p2: [f64; 2], p3: [f64; 2]| -> f64 {
                    (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
                };
                let d1 = ccw(a, b, c);
                let d2 = ccw(a, b, d);
                let d3 = ccw(c, d, a);
                let d4 = ccw(c, d, b);
                ((d1 > 1e-9 && d2 < -1e-9) || (d1 < -1e-9 && d2 > 1e-9))
                    && ((d3 > 1e-9 && d4 < -1e-9) || (d3 < -1e-9 && d4 > 1e-9))
            };
            for i in 0..n_pts {
                let i_next = (i + 1) % n_pts;
                for j in (i + 2)..n_pts {
                    let j_next = (j + 1) % n_pts;
                    if i == 0 && j_next == 0 {
                        continue;
                    }
                    if segments_intersect_2d(raw_profile[i], raw_profile[i_next], raw_profile[j], raw_profile[j_next]) {
                        return Err(ServerError::InvalidArgument(
                            "Revolve 2D profile is self-intersecting in the (r, z) plane".to_string(),
                        ));
                    }
                }
            }

            let profile: Vec<[f64; 2]> = raw_profile
                .into_iter()
                .map(|[r, z]| [r * k_to_m, z * k_to_m])
                .collect();

            if let (Some(max_r), Some(min_z), Some(max_z)) = (
                profile.iter().map(|p| p[0]).reduce(f64::max),
                profile.iter().map(|p| p[1]).reduce(f64::min),
                profile.iter().map(|p| p[1]).reduce(f64::max),
            ) {
                extra_dims_m.push(max_r * 2.0);
                extra_dims_m.push((max_z - min_z).abs());
            }

            let angle_deg = params
                .get("angle_deg")
                .or_else(|| params.get("angle"))
                .and_then(|v| v.as_f64())
                .unwrap_or(360.0);

            if !angle_deg.is_finite() || angle_deg <= 0.0 || angle_deg > 360.0 {
                return Err(ServerError::InvalidArgument(format!(
                    "Revolve 'angle_deg' must be in (0, 360], got {angle_deg}"
                )));
            }

            let segments = params
                .get("segments")
                .and_then(|v| v.as_u64())
                .unwrap_or(32) as usize;

            if segments < 3 {
                return Err(ServerError::InvalidArgument(
                    "Revolve 'segments' must be >= 3".to_string(),
                ));
            }

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
                .ok_or_else(|| ServerError::InvalidArgument("Wheel rim requires 'rim_diameter'".to_string()))?
                * k_to_m;

            let rim_width = params
                .get("rim_width")
                .or_else(|| params.get("width"))
                .and_then(|v| v.as_f64())
                .ok_or_else(|| ServerError::InvalidArgument("Wheel rim requires 'rim_width'".to_string()))?
                * k_to_m;

            let flange_diameter = params
                .get("flange_diameter")
                .and_then(|v| v.as_f64())
                .map(|v| v * k_to_m);

            let hub_diameter = params
                .get("hub_diameter")
                .and_then(|v| v.as_f64())
                .map(|v| v * k_to_m);

            let hub_bore_diameter = params
                .get("hub_bore_diameter")
                .or_else(|| params.get("bore_diameter"))
                .and_then(|v| v.as_f64())
                .map(|v| v * k_to_m);

            let pcd_diameter = params
                .get("pcd_diameter")
                .or_else(|| params.get("pcd"))
                .and_then(|v| v.as_f64())
                .map(|v| v * k_to_m);

            let bolt_holes_count = params
                .get("bolt_holes_count")
                .or_else(|| params.get("bolts_count"))
                .or_else(|| params.get("bolts"))
                .and_then(|v| v.as_u64())
                .map(|b| b as usize);

            let bolt_hole_diameter = params
                .get("bolt_hole_diameter")
                .and_then(|v| v.as_f64())
                .map(|v| v * k_to_m);

            let spokes_count = params
                .get("spokes_count")
                .or_else(|| params.get("spokes"))
                .and_then(|v| v.as_u64())
                .map(|s| s as usize);

            let spoke_width = params
                .get("spoke_width")
                .and_then(|v| v.as_f64())
                .map(|v| v * k_to_m);

            let spoke_thickness = params
                .get("spoke_thickness")
                .and_then(|v| v.as_f64())
                .map(|v| v * k_to_m);

            let offset_et = params
                .get("offset_et")
                .or_else(|| params.get("et"))
                .and_then(|v| v.as_f64())
                .map(|v| v * k_to_m);

            wheel_rim_diameter_m = Some(rim_diameter);
            extra_dims_m.extend_from_slice(&[rim_diameter, rim_width]);

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

    let k_from_m = unit.from_meters_factor();
    let (feature_id, aabb, volume_m3, mesh_clone, part_rev) = {
        let part = state
            .parts
            .get_mut(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

        let prev_dag = part.dag.clone();
        let prev_mesh = part.mesh.clone();
        let prev_cached_mesh = part.cached_mesh.clone();
        let prev_cached_settings = part.cached_settings.clone();
        let prev_revision = part.revision;
        part.ensure_imported_mesh_in_dag();
        let feature_id = part.dag.add_feature(op, None);
        if let Err(e) = part.rebuild() {
            part.dag = prev_dag;
            part.mesh = prev_mesh;
            part.cached_mesh = prev_cached_mesh;
            part.cached_settings = prev_cached_settings;
            part.revision = prev_revision;
            return Err(ServerError::Cad(e));
        }

        (
            feature_id,
            part.bounding_box(),
            part.volume(),
            part.mesh.clone(),
            part.revision,
        )
    };

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &extra_dims_m, wheel_rim_diameter_m, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    if state.physics_world.has_part(part_id) {
        if let Some(part) = state.parts.get(part_id) {
            if !part.mesh().vertices.is_empty() {
                let _ = state.physics_world.update_part_collider(part);
            }
        }
    }

    let mut resp = json!({
        "part_id": part_id,
        "feature_id": feature_id.to_string(),
        "primitive_type": prim_type,
        "revision": part_rev,
        "bounding_box": {
            "min": scale_vec3(aabb.min, k_from_m),
            "max": scale_vec3(aabb.max, k_from_m),
            "dimensions": scale_vec3(aabb.dimensions(), k_from_m),
        },
        "volume": volume_m3 * k_from_m.powi(3),
        "volume_m3": volume_m3,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(part_id), &mut resp);
    Ok(resp)
}

/// Computes the part's span in meters along `normal` (or fallback AABB diagonal) for `through: true` holes.
fn compute_through_hole_depth_m(
    mesh: &mcp_cad_core::TriangleMesh,
    center: [f64; 3],
    normal: [f64; 3],
    explicit_depth_m: Option<f64>,
) -> f64 {
    let norm_len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    if mesh.vertices.is_empty() || norm_len <= 1e-12 {
        return explicit_depth_m.unwrap_or(0.1);
    }
    let ux = normal[0] / norm_len;
    let uy = normal[1] / norm_len;
    let uz = normal[2] / norm_len;

    let mut t_min = f64::INFINITY;
    let mut t_max = f64::NEG_INFINITY;
    for v in &mesh.vertices {
        let t = (v.position[0] - center[0]) * ux
            + (v.position[1] - center[1]) * uy
            + (v.position[2] - center[2]) * uz;
        if t < t_min {
            t_min = t;
        }
        if t > t_max {
            t_max = t;
        }
    }
    let span = (t_max - t_min).max(0.0);
    if span > 1e-9 {
        explicit_depth_m.map_or(span, |d| d.max(span))
    } else {
        explicit_depth_m.unwrap_or(0.1)
    }
}

/// 5. `cut_hole`: Cuts a cylindrical hole through the solid.
pub fn cut_hole(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let params = args.get("params").unwrap_or(args);
    let unit = if params.get("length_unit").is_some() {
        state.resolve_length_unit(params)?
    } else {
        state.resolve_length_unit(args)?
    };
    let k_to_m = unit.to_meters_factor();
    let k_from_m = unit.from_meters_factor();

    let pos_val = params
        .get("position")
        .or_else(|| params.get("center"))
        .or_else(|| args.get("position"))
        .or_else(|| args.get("center"))
        .and_then(|v| v.as_array())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'position' [x, y, z]".to_string()))?;

    if pos_val.len() != 3 {
        return Err(ServerError::InvalidArgument("'position' must have 3 elements".to_string()));
    }
    let center = [
        pos_val[0].as_f64().unwrap_or(0.0) * k_to_m,
        pos_val[1].as_f64().unwrap_or(0.0) * k_to_m,
        pos_val[2].as_f64().unwrap_or(0.0) * k_to_m,
    ];

    let norm_val = params
        .get("normal")
        .or_else(|| params.get("direction"))
        .or_else(|| args.get("normal"))
        .or_else(|| args.get("direction"))
        .and_then(|v| v.as_array())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'normal' [nx, ny, nz]".to_string()))?;

    if norm_val.len() != 3 {
        return Err(ServerError::InvalidArgument("'normal' must have 3 elements".to_string()));
    }
    let normal = [
        norm_val[0].as_f64().unwrap_or(0.0),
        norm_val[1].as_f64().unwrap_or(0.0),
        norm_val[2].as_f64().unwrap_or(0.0),
    ];
    let norm_len_sq = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
    if !norm_len_sq.is_finite() || norm_len_sq <= 1e-18 {
        return Err(ServerError::InvalidArgument(
            "Argument 'normal' must be a non-zero 3D vector".to_string(),
        ));
    }

    let radius_raw = params
        .get("radius")
        .or_else(|| args.get("radius"))
        .and_then(|v| v.as_f64())
        .or_else(|| {
            params
                .get("diameter")
                .or_else(|| args.get("diameter"))
                .and_then(|v| v.as_f64())
                .map(|d| d * 0.5)
        })
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'radius'".to_string()))?;

    if !radius_raw.is_finite() || radius_raw <= 0.0 {
        return Err(ServerError::InvalidArgument(
            "Argument 'radius' must be a positive number (> 0)".to_string(),
        ));
    }
    let radius = radius_raw * k_to_m;

    let through = params
        .get("through")
        .or_else(|| params.get("through_all"))
        .or_else(|| args.get("through"))
        .or_else(|| args.get("through_all"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

    let explicit_depth_raw = params
        .get("depth")
        .or_else(|| args.get("depth"))
        .and_then(|v| v.as_f64());

    if let Some(d_raw) = explicit_depth_raw {
        if !d_raw.is_finite() || d_raw <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "Argument 'depth' must be a positive number (> 0)".to_string(),
            ));
        }
    } else if !through {
        return Err(ServerError::InvalidArgument(
            "Missing required argument 'depth' (or set 'through': true)".to_string(),
        ));
    }
    let explicit_depth_m = explicit_depth_raw.map(|d| d * k_to_m);

    let keyway_width = params
        .get("keyway_width")
        .or_else(|| args.get("keyway_width"))
        .and_then(|v| v.as_f64())
        .map(|w| w * k_to_m);
    let keyway_depth = params
        .get("keyway_depth")
        .or_else(|| args.get("keyway_depth"))
        .and_then(|v| v.as_f64())
        .map(|d| d * k_to_m);

    let (feature_id, aabb, volume_m3, mesh_clone, part_rev, depth) = {
        let part = state
            .parts
            .get_mut(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

        let depth = if through {
            compute_through_hole_depth_m(&part.mesh, center, normal, explicit_depth_m)
        } else {
            explicit_depth_m.unwrap()
        };

        let prev_dag = part.dag.clone();
        let prev_mesh = part.mesh.clone();
        let prev_cached_mesh = part.cached_mesh.clone();
        let prev_cached_settings = part.cached_settings.clone();
        let prev_revision = part.revision;
        part.ensure_imported_mesh_in_dag();
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
            part.cached_mesh = prev_cached_mesh;
            part.cached_settings = prev_cached_settings;
            part.revision = prev_revision;
            return Err(ServerError::Cad(e));
        }

        (
            feature_id,
            part.bounding_box(),
            part.volume(),
            part.mesh.clone(),
            part.revision,
            depth,
        )
    };

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &[radius * 2.0, depth], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    if state.physics_world.has_part(part_id) {
        if let Some(part) = state.parts.get(part_id) {
            if !part.mesh().vertices.is_empty() {
                let _ = state.physics_world.update_part_collider(part);
            }
        }
    }

    let mut resp = json!({
        "part_id": part_id,
        "feature_id": feature_id.to_string(),
        "revision": part_rev,
        "depth": depth * k_from_m,
        "depth_m": depth,
        "through": through,
        "bounding_box": {
            "min": scale_vec3(aabb.min, k_from_m),
            "max": scale_vec3(aabb.max, k_from_m),
            "dimensions": scale_vec3(aabb.dimensions(), k_from_m),
        },
        "volume": volume_m3 * k_from_m.powi(3),
        "volume_m3": volume_m3,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(part_id), &mut resp);
    Ok(resp)
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
        .or_else(|| args.get("parameters"))
        .unwrap_or(args);

    let unit = if params.get("length_unit").is_some() {
        state.resolve_length_unit(params)?
    } else {
        state.resolve_length_unit(args)?
    };
    let k_to_m = unit.to_meters_factor();
    let k_from_m = unit.from_meters_factor();

    let mut extra_dims_m: Vec<f64> = Vec::new();
    let mut wheel_rim_diameter_m: Option<f64> = None;

    let (aabb, volume_m3, mesh_clone, part_rev) = {
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
                let new_w = params.get("width").and_then(|v| v.as_f64()).map(|w| w * k_to_m).unwrap_or(*width);
                let new_h = params.get("height").and_then(|v| v.as_f64()).map(|h| h * k_to_m).unwrap_or(*height);
                let new_d = params.get("depth").and_then(|v| v.as_f64()).map(|d| d * k_to_m).unwrap_or(*depth);
                let pos = parse_vec3(params, "position").map(|p| scale_vec3(p, k_to_m)).or(*position);
                let rot = parse_vec3(params, "rotation_deg")
                    .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                    .or(*rotation_deg);
                extra_dims_m.extend_from_slice(&[new_w, new_h, new_d]);
                FeatureOp::AddBox {
                    width: new_w,
                    height: new_h,
                    depth: new_d,
                    position: pos,
                    rotation_deg: rot,
                }
            }
            FeatureOp::AddCylinder { radius, height, segments, position, rotation_deg } => {
                let new_r = params.get("radius").and_then(|v| v.as_f64()).map(|r| r * k_to_m).unwrap_or(*radius);
                let new_h = params.get("height").and_then(|v| v.as_f64()).map(|h| h * k_to_m).unwrap_or(*height);
                let new_s = params.get("segments").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*segments);
                let pos = parse_vec3(params, "position").map(|p| scale_vec3(p, k_to_m)).or(*position);
                let rot = parse_vec3(params, "rotation_deg")
                    .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                    .or(*rotation_deg);
                extra_dims_m.extend_from_slice(&[new_r * 2.0, new_h]);
                FeatureOp::AddCylinder {
                    radius: new_r,
                    height: new_h,
                    segments: new_s,
                    position: pos,
                    rotation_deg: rot,
                }
            }
            FeatureOp::AddSphere { radius, sectors, stacks, position, rotation_deg } => {
                let new_r = params.get("radius").and_then(|v| v.as_f64()).map(|r| r * k_to_m).unwrap_or(*radius);
                let new_sec = params.get("sectors").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*sectors);
                let new_stk = params.get("stacks").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*stacks);
                let pos = parse_vec3(params, "position").map(|p| scale_vec3(p, k_to_m)).or(*position);
                let rot = parse_vec3(params, "rotation_deg")
                    .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                    .or(*rotation_deg);
                extra_dims_m.push(new_r * 2.0);
                FeatureOp::AddSphere {
                    radius: new_r,
                    sectors: new_sec,
                    stacks: new_stk,
                    position: pos,
                    rotation_deg: rot,
                }
            }
            FeatureOp::AddCone { radius_bottom, radius_top, height, segments, position, rotation_deg } => {
                let new_rb = params.get("radius_bottom").or_else(|| params.get("radius")).and_then(|v| v.as_f64()).map(|r| r * k_to_m).unwrap_or(*radius_bottom);
                let new_rt = params.get("radius_top").and_then(|v| v.as_f64()).map(|r| r * k_to_m).unwrap_or(*radius_top);
                let new_h = params.get("height").and_then(|v| v.as_f64()).map(|h| h * k_to_m).unwrap_or(*height);
                let new_s = params.get("segments").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*segments);
                let pos = parse_vec3(params, "position").map(|p| scale_vec3(p, k_to_m)).or(*position);
                let rot = parse_vec3(params, "rotation_deg")
                    .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                    .or(*rotation_deg);
                extra_dims_m.extend_from_slice(&[new_rb * 2.0, new_h]);
                if new_rt > 0.0 {
                    extra_dims_m.push(new_rt * 2.0);
                }
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
                let new_h = params.get("height").and_then(|v| v.as_f64()).map(|h| h * k_to_m).unwrap_or(*height);
                let new_waf = params.get("width_across_flats").or_else(|| params.get("s")).and_then(|v| v.as_f64()).map(|w| w * k_to_m).or(*width_across_flats);
                let new_r = params.get("radius").or_else(|| params.get("outer_radius")).or_else(|| params.get("r")).and_then(|v| v.as_f64()).map(|r| r * k_to_m).or(*radius);
                let new_chamfer = params.get("chamfer").or_else(|| params.get("chamfer_radius")).and_then(|v| v.as_f64()).map(|c| c * k_to_m).or(*chamfer);
                let pos = parse_vec3(params, "position").map(|p| scale_vec3(p, k_to_m)).or(*position);
                let rot = parse_vec3(params, "rotation_deg")
                    .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                    .or(*rotation_deg);
                extra_dims_m.push(new_h);
                if let Some(w) = new_waf {
                    extra_dims_m.push(w);
                }
                if let Some(r) = new_r {
                    extra_dims_m.push(r * 2.0);
                }
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
                let new_m = params.get("module").or_else(|| params.get("m")).and_then(|v| v.as_f64()).map(|m| m * k_to_m).unwrap_or(*module);
                let new_z = params.get("teeth").or_else(|| params.get("z")).and_then(|v| v.as_u64()).map(|z| z as usize).unwrap_or(*teeth);
                let new_pa = params.get("pressure_angle_deg").or_else(|| params.get("pressure_angle")).and_then(|v| v.as_f64()).unwrap_or(*pressure_angle_deg);
                let new_fw = params.get("face_width").or_else(|| params.get("width")).or_else(|| params.get("b")).and_then(|v| v.as_f64()).map(|w| w * k_to_m).unwrap_or(*face_width);
                let new_bore = params.get("bore_radius").and_then(|v| v.as_f64())
                    .or_else(|| params.get("bore_diameter").and_then(|v| v.as_f64()).map(|d| d * 0.5))
                    .map(|r| r * k_to_m)
                    .or(*bore_radius);
                let new_kw = params.get("keyway_width").and_then(|v| v.as_f64()).map(|w| w * k_to_m).or(*keyway_width);
                let new_kd = params.get("keyway_depth").and_then(|v| v.as_f64()).map(|d| d * k_to_m).or(*keyway_depth);
                let new_hd = params.get("hub_diameter").and_then(|v| v.as_f64()).map(|d| d * k_to_m).or(*hub_diameter);
                let new_hl = params.get("hub_length").and_then(|v| v.as_f64()).map(|l| l * k_to_m).or(*hub_length);
                let pos = parse_vec3(params, "position").map(|p| scale_vec3(p, k_to_m)).or(*position);
                let rot = parse_vec3(params, "rotation_deg")
                    .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                    .or(*rotation_deg);

                extra_dims_m.extend_from_slice(&[new_m * (new_z as f64), new_fw]);

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
                let inner_r_opt = params.get("inner_radius").and_then(|v| v.as_f64());
                let new_profile = if let Some(arr) = params.get("profile").and_then(|v| v.as_array()) {
                    let mut p = Vec::with_capacity(arr.len());
                    for item in arr {
                        let coords = item.as_array().ok_or_else(|| {
                            ServerError::InvalidArgument(
                                "Each profile point must be a [r, z] array".to_string(),
                            )
                        })?;
                        if coords.len() != 2 {
                            return Err(ServerError::InvalidArgument(
                                "Each profile point must have 2 elements [r, z]".to_string(),
                            ));
                        }
                        let r = coords[0].as_f64().ok_or_else(|| {
                            ServerError::InvalidArgument("Profile 'r' must be a number".to_string())
                        })?;
                        let z = coords[1].as_f64().ok_or_else(|| {
                            ServerError::InvalidArgument("Profile 'z' must be a number".to_string())
                        })?;
                        if !r.is_finite() || !z.is_finite() || r < 0.0 {
                            return Err(ServerError::InvalidArgument(
                                "Profile 'r' must be finite and >= 0, and 'z' must be finite".to_string(),
                            ));
                        }
                        p.push([r * k_to_m, z * k_to_m]);
                    }
                    if p.len() == 2 {
                        if let Some(inner_r) = inner_r_opt {
                            let ir_m = inner_r * k_to_m;
                            if !ir_m.is_finite() || ir_m < 0.0 {
                                return Err(ServerError::InvalidArgument(
                                    "Revolve 'inner_radius' must be >= 0".to_string(),
                                ));
                            }
                            if ir_m >= p[0][0] || ir_m >= p[1][0] {
                                return Err(ServerError::InvalidArgument(
                                    "Revolve 'inner_radius' must be strictly less than outer profile radius".to_string(),
                                ));
                            }
                            if (p[1][1] - p[0][1]).abs() <= 1e-12 {
                                return Err(ServerError::InvalidArgument(
                                    "Revolve 2-point profile with 'inner_radius' must span non-zero axial height".to_string(),
                                ));
                            }
                            vec![
                                [ir_m, p[0][1]],
                                p[0],
                                p[1],
                                [ir_m, p[1][1]],
                            ]
                        } else {
                            return Err(ServerError::InvalidArgument(
                                "Revolve 'profile' must contain at least 3 [r, z] points (or 2 outer points with 'inner_radius')".to_string(),
                            ));
                        }
                    } else if p.len() >= 3 {
                        p
                    } else {
                        return Err(ServerError::InvalidArgument(
                            "Revolve 'profile' must contain at least 3 [r, z] points".to_string(),
                        ));
                    }
                } else if params.get("profile").is_some() {
                    return Err(ServerError::InvalidArgument(
                        "Revolve 'profile' must be an array of [r, z] points".to_string(),
                    ));
                } else {
                    profile.clone()
                };
                if let (Some(max_r), Some(min_z), Some(max_z)) = (
                    new_profile.iter().map(|p| p[0]).reduce(f64::max),
                    new_profile.iter().map(|p| p[1]).reduce(f64::min),
                    new_profile.iter().map(|p| p[1]).reduce(f64::max),
                ) {
                    extra_dims_m.push(max_r * 2.0);
                    extra_dims_m.push((max_z - min_z).abs());
                }
                let new_angle = params.get("angle_deg").or_else(|| params.get("angle")).and_then(|v| v.as_f64()).unwrap_or(*angle_deg);
                let new_segs = params.get("segments").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*segments);
                let pos = parse_vec3(params, "position").map(|p| scale_vec3(p, k_to_m)).or(*position);
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
                let new_dia = params.get("rim_diameter").or_else(|| params.get("diameter")).and_then(|v| v.as_f64()).map(|v| v * k_to_m).unwrap_or(*rim_diameter);
                let new_w = params.get("rim_width").or_else(|| params.get("width")).and_then(|v| v.as_f64()).map(|v| v * k_to_m).unwrap_or(*rim_width);
                let new_flange = params.get("flange_diameter").and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*flange_diameter);
                let new_hub = params.get("hub_diameter").and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*hub_diameter);
                let new_bore = params.get("hub_bore_diameter").or_else(|| params.get("bore_diameter")).and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*hub_bore_diameter);
                let new_pcd = params.get("pcd_diameter").or_else(|| params.get("pcd")).and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*pcd_diameter);
                let new_bolts = params.get("bolt_holes_count").or_else(|| params.get("bolts_count")).or_else(|| params.get("bolts")).and_then(|v| v.as_u64()).map(|b| b as usize).or(*bolt_holes_count);
                let new_bolt_d = params.get("bolt_hole_diameter").and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*bolt_hole_diameter);
                let new_spokes = params.get("spokes_count").or_else(|| params.get("spokes")).and_then(|v| v.as_u64()).map(|s| s as usize).or(*spokes_count);
                let new_spoke_w = params.get("spoke_width").and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*spoke_width);
                let new_spoke_t = params.get("spoke_thickness").and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*spoke_thickness);
                let new_et = params.get("offset_et").or_else(|| params.get("et")).and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*offset_et);
                let pos = parse_vec3(params, "position").map(|p| scale_vec3(p, k_to_m)).or(*position);
                let rot = parse_vec3(params, "rotation_deg")
                    .or_else(|| parse_vec3(params, "rotation_rpy_deg"))
                    .or(*rotation_deg);

                wheel_rim_diameter_m = Some(new_dia);
                extra_dims_m.extend_from_slice(&[new_dia, new_w]);

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
                let hr = params.get("helix_radius").and_then(|v| v.as_f64()).map(|v| v * k_to_m).unwrap_or(*helix_radius);
                let gr = params.get("groove_radius").and_then(|v| v.as_f64()).map(|v| v * k_to_m).unwrap_or(*groove_radius);
                let p = params.get("pitch").and_then(|v| v.as_f64()).map(|v| v * k_to_m).unwrap_or(*pitch);
                let h = params.get("height").and_then(|v| v.as_f64()).map(|v| v * k_to_m).unwrap_or(*height);
                let sz = params.get("start_z").and_then(|v| v.as_f64()).map(|v| v * k_to_m).unwrap_or(*start_z);
                let st = params.get("starts").and_then(|v| v.as_u64()).map(|s| s as usize).unwrap_or(*starts);
                let rh = params.get("is_right_hand").and_then(|v| v.as_bool()).unwrap_or(*is_right_hand);
                let pt = params.get("profile_type").and_then(|v| v.as_str()).map(|s| s.to_string()).unwrap_or_else(|| profile_type.clone());
                extra_dims_m.extend_from_slice(&[hr * 2.0, gr, p, h]);
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
            FeatureOp::ImportedMesh { mesh } => {
                FeatureOp::ImportedMesh {
                    mesh: mesh.clone(),
                }
            }
            FeatureOp::CutHole { center, normal, radius, depth, keyway_width, keyway_depth } => {
                let new_center = if let Some(arr) = params.get("position").or_else(|| params.get("center")).and_then(|v| v.as_array()) {
                    if arr.len() != 3 {
                        return Err(ServerError::InvalidArgument(
                            "'position'/'center' must have 3 elements".to_string(),
                        ));
                    }
                    [
                        arr[0].as_f64().map(|v| v * k_to_m).unwrap_or(center[0]),
                        arr[1].as_f64().map(|v| v * k_to_m).unwrap_or(center[1]),
                        arr[2].as_f64().map(|v| v * k_to_m).unwrap_or(center[2]),
                    ]
                } else {
                    *center
                };
                let new_normal = if let Some(arr) = params.get("normal").or_else(|| params.get("direction")).and_then(|v| v.as_array()) {
                    if arr.len() != 3 {
                        return Err(ServerError::InvalidArgument(
                            "'normal'/'direction' must have 3 elements".to_string(),
                        ));
                    }
                    [arr[0].as_f64().unwrap_or(normal[0]), arr[1].as_f64().unwrap_or(normal[1]), arr[2].as_f64().unwrap_or(normal[2])]
                } else {
                    *normal
                };
                let new_r = params.get("radius").and_then(|v| v.as_f64())
                    .or_else(|| params.get("diameter").and_then(|v| v.as_f64()).map(|d| d * 0.5))
                    .map(|v| v * k_to_m)
                    .unwrap_or(*radius);
                let through = params.get("through").or_else(|| params.get("through_all")).and_then(|v| v.as_bool()).unwrap_or(false);
                let explicit_d = params.get("depth").and_then(|v| v.as_f64()).map(|v| v * k_to_m);
                let new_d = if through {
                    compute_through_hole_depth_m(&part.mesh, new_center, new_normal, explicit_d.or(Some(*depth)))
                } else {
                    explicit_d.unwrap_or(*depth)
                };
                let new_kw = params.get("keyway_width").and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*keyway_width);
                let new_kd = params.get("keyway_depth").and_then(|v| v.as_f64()).map(|v| v * k_to_m).or(*keyway_depth);
                extra_dims_m.extend_from_slice(&[new_r * 2.0, new_d]);
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
                    if arr.len() != 3 {
                        return Err(ServerError::InvalidArgument(
                            "'translation' must have 3 elements".to_string(),
                        ));
                    }
                    [
                        arr[0].as_f64().map(|v| v * k_to_m).unwrap_or(translation[0]),
                        arr[1].as_f64().map(|v| v * k_to_m).unwrap_or(translation[1]),
                        arr[2].as_f64().map(|v| v * k_to_m).unwrap_or(translation[2]),
                    ]
                } else {
                    *translation
                };
                let r = if let Some(arr) = params.get("rotation_deg").or_else(|| params.get("rotation_rpy_deg")).and_then(|v| v.as_array()) {
                    if arr.len() != 3 {
                        return Err(ServerError::InvalidArgument(
                            "'rotation_deg' must have 3 elements".to_string(),
                        ));
                    }
                    [arr[0].as_f64().unwrap_or(rotation_deg[0]), arr[1].as_f64().unwrap_or(rotation_deg[1]), arr[2].as_f64().unwrap_or(rotation_deg[2])]
                } else {
                    *rotation_deg
                };
                let s = if let Some(arr) = params.get("scale").and_then(|v| v.as_array()) {
                    if arr.len() != 3 {
                        return Err(ServerError::InvalidArgument(
                            "'scale' must have 3 elements".to_string(),
                        ));
                    }
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
        let prev_cached_mesh = part.cached_mesh.clone();
        let prev_cached_settings = part.cached_settings.clone();
        let prev_revision = part.revision;
        if let Err(e) = part.dag.modify_feature(feature_id, updated_op) {
            part.dag = prev_dag;
            part.mesh = prev_mesh;
            part.cached_mesh = prev_cached_mesh;
            part.cached_settings = prev_cached_settings;
            part.revision = prev_revision;
            return Err(ServerError::Cad(e));
        }
        if let Err(e) = part.rebuild() {
            part.dag = prev_dag;
            part.mesh = prev_mesh;
            part.cached_mesh = prev_cached_mesh;
            part.cached_settings = prev_cached_settings;
            part.revision = prev_revision;
            return Err(ServerError::Cad(e));
        }

        // Also check if any node in the updated DAG is an AddWheelRim (so even if a non-wheel-rim node
        // on a wheel-rim part wasn't the target, or if AddWheelRim was modified without changing rim_diameter,
        // wheel_rim_diameter_m is populated when modifying AddWheelRim).
        let aabb = part.bounding_box();
        let volume_m3 = part.volume();
        let mesh_clone = part.mesh.clone();
        (aabb, volume_m3, mesh_clone, part.revision)
    };

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &extra_dims_m, wheel_rim_diameter_m, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    if state.physics_world.has_part(part_id) {
        if let Some(part) = state.parts.get(part_id) {
            if !part.mesh().vertices.is_empty() {
                let _ = state.physics_world.update_part_collider(part);
            }
        }
    }

    let dims = aabb.dimensions();
    let mut resp = json!({
        "part_id": part_id,
        "feature_id": feature_id.to_string(),
        "status": "updated",
        "revision": part_rev,
        "updated_bounds": [
            aabb.min[0] * k_from_m,
            aabb.min[1] * k_from_m,
            aabb.min[2] * k_from_m,
            aabb.max[0] * k_from_m,
            aabb.max[1] * k_from_m,
            aabb.max[2] * k_from_m
        ],
        "dimensions": scale_vec3(dims, k_from_m),
        "bounding_box": {
            "min": scale_vec3(aabb.min, k_from_m),
            "max": scale_vec3(aabb.max, k_from_m),
            "dimensions": scale_vec3(dims, k_from_m),
        },
        "volume": volume_m3 * k_from_m.powi(3),
        "volume_m3": volume_m3,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(part_id), &mut resp);
    Ok(resp)
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

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let scaled_nodes: Vec<FeatureNode> = part
        .dag
        .nodes
        .iter()
        .map(|node| FeatureNode {
            id: node.id,
            name: node.name.clone(),
            op: scale_feature_op_from_meters(&node.op, k_from_m),
            created_at: node.created_at,
            enabled: node.enabled,
        })
        .collect();

    Ok(json!({
        "part_id": part.id,
        "revision": part.revision,
        "features_count": scaled_nodes.len(),
        "dag": scaled_nodes,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// 8. `cut_helical_groove`: Cuts one or more helical grooves (flutes/threads) into the solid.
pub fn cut_helical_groove(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let params = args.get("params").unwrap_or(args);
    let unit = if params.get("length_unit").is_some() {
        state.resolve_length_unit(params)?
    } else {
        state.resolve_length_unit(args)?
    };
    let k_to_m = unit.to_meters_factor();
    let k_from_m = unit.from_meters_factor();

    let helix_radius_raw = params
        .get("helix_radius")
        .or_else(|| args.get("helix_radius"))
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'helix_radius'".to_string()))?;
    if !helix_radius_raw.is_finite() || helix_radius_raw <= 0.0 {
        return Err(ServerError::InvalidArgument(
            "Argument 'helix_radius' must be > 0".to_string(),
        ));
    }
    let helix_radius = helix_radius_raw * k_to_m;

    let groove_radius_raw = params
        .get("groove_radius")
        .or_else(|| args.get("groove_radius"))
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'groove_radius'".to_string()))?;
    if !groove_radius_raw.is_finite() || groove_radius_raw <= 0.0 {
        return Err(ServerError::InvalidArgument(
            "Argument 'groove_radius' must be > 0".to_string(),
        ));
    }
    let groove_radius = groove_radius_raw * k_to_m;

    let pitch_raw = params
        .get("pitch")
        .or_else(|| args.get("pitch"))
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'pitch'".to_string()))?;
    if !pitch_raw.is_finite() || pitch_raw <= 0.0 {
        return Err(ServerError::InvalidArgument(
            "Argument 'pitch' must be > 0".to_string(),
        ));
    }
    let pitch = pitch_raw * k_to_m;

    let height_raw = params
        .get("height")
        .or_else(|| args.get("height"))
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'height'".to_string()))?;
    if !height_raw.is_finite() || height_raw <= 0.0 {
        return Err(ServerError::InvalidArgument(
            "Argument 'height' must be > 0".to_string(),
        ));
    }
    let height = height_raw * k_to_m;

    let explicit_start_z = params
        .get("start_z")
        .or_else(|| args.get("start_z"))
        .and_then(|v| v.as_f64())
        .map(|z| z * k_to_m);
    if let Some(sz) = explicit_start_z {
        if !sz.is_finite() {
            return Err(ServerError::InvalidArgument(
                "Argument 'start_z' must be a finite number".to_string(),
            ));
        }
    }

    let profile_type = params
        .get("profile_type")
        .or_else(|| args.get("profile_type"))
        .and_then(|v| v.as_str())
        .unwrap_or("circular")
        .to_string();

    let default_starts = if profile_type == "metric_60" { 1 } else { 2 };
    let starts = params
        .get("starts")
        .or_else(|| args.get("starts"))
        .and_then(|v| v.as_u64())
        .map(|s| s as usize)
        .unwrap_or(default_starts);
    if starts == 0 {
        return Err(ServerError::InvalidArgument(
            "Argument 'starts' must be >= 1".to_string(),
        ));
    }

    let is_right_hand = params
        .get("is_right_hand")
        .or_else(|| args.get("is_right_hand"))
        .and_then(|v| v.as_bool())
        .unwrap_or(true);

    let (feature_id, aabb, volume_m3, mesh_clone, part_rev, start_z) = {
        let part = state
            .parts
            .get_mut(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

        // When `start_z` is omitted, align the helical sweep start with the bottom face of the part
        // (`bounding_box.min[2]`) or `-height / 2` if the part is empty (P2-T19).
        let start_z = explicit_start_z.unwrap_or_else(|| {
            if !part.mesh().vertices.is_empty() {
                part.bounding_box().min[2]
            } else {
                -height * 0.5
            }
        });

        let prev_dag = part.dag.clone();
        let prev_mesh = part.mesh.clone();
        let prev_cached_mesh = part.cached_mesh.clone();
        let prev_cached_settings = part.cached_settings.clone();
        let prev_revision = part.revision;
        part.ensure_imported_mesh_in_dag();
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
            part.cached_mesh = prev_cached_mesh;
            part.cached_settings = prev_cached_settings;
            part.revision = prev_revision;
            return Err(ServerError::Cad(e));
        }

        (
            feature_id,
            part.bounding_box(),
            part.volume(),
            part.mesh.clone(),
            part.revision,
            start_z,
        )
    };

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &[helix_radius * 2.0, groove_radius, pitch, height], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    if state.physics_world.has_part(part_id) {
        if let Some(part) = state.parts.get(part_id) {
            if !part.mesh().vertices.is_empty() {
                let _ = state.physics_world.update_part_collider(part);
            }
        }
    }

    let mut resp = json!({
        "part_id": part_id,
        "feature_id": feature_id.to_string(),
        "revision": part_rev,
        "start_z": start_z * k_from_m,
        "status": "helical_groove_cut",
        "bounding_box": {
            "min": scale_vec3(aabb.min, k_from_m),
            "max": scale_vec3(aabb.max, k_from_m),
            "dimensions": scale_vec3(aabb.dimensions(), k_from_m),
        },
        "volume": volume_m3 * k_from_m.powi(3),
        "volume_m3": volume_m3,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(part_id), &mut resp);
    Ok(resp)
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

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let tool_mesh = {
        let tool_part = state
            .parts
            .get(tool_part_id)
            .ok_or_else(|| ServerError::PartNotFound(tool_part_id.to_string()))?;
        tool_part.mesh.clone()
    };

    let (feature_id, vertices_count, triangles_count, aabb, volume_m3, mesh_clone, part_rev) = {
        let target_part = state
            .parts
            .get_mut(target_part_id)
            .ok_or_else(|| ServerError::PartNotFound(target_part_id.to_string()))?;

        let prev_dag = target_part.dag.clone();
        let prev_mesh = target_part.mesh.clone();
        let prev_cached_mesh = target_part.cached_mesh.clone();
        let prev_cached_settings = target_part.cached_settings.clone();
        let prev_revision = target_part.revision;
        target_part.ensure_imported_mesh_in_dag();
        let feature_id = target_part.dag.add_feature(
            FeatureOp::BooleanDifferenceMesh { tool_mesh },
            None,
        );
        if let Err(e) = target_part.rebuild() {
            target_part.dag = prev_dag;
            target_part.mesh = prev_mesh;
            target_part.cached_mesh = prev_cached_mesh;
            target_part.cached_settings = prev_cached_settings;
            target_part.revision = prev_revision;
            return Err(ServerError::Cad(e));
        }

        let aabb = target_part.bounding_box();
        let volume_m3 = target_part.volume();
        let mesh = target_part.mesh().clone();
        let part_rev = target_part.revision;
        (
            feature_id,
            mesh.vertices.len(),
            mesh.triangle_count(),
            aabb,
            volume_m3,
            mesh,
            part_rev,
        )
    };

    let warnings = if let Some(target_part) = state.parts.get(target_part_id) {
        state.check_scale_sanity(target_part, &[], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    if let Some(target_part) = state.parts.get(target_part_id) {
        if !target_part.mesh().vertices.is_empty() {
            let _ = state.physics_world.update_part_collider(target_part);
        }
    }

    let mut resp = json!({
        "part_id": target_part_id,
        "target_part_id": target_part_id,
        "feature_id": feature_id.to_string(),
        "revision": part_rev,
        "status": "boolean_difference_applied",
        "vertices_count": vertices_count,
        "triangles_count": triangles_count,
        "bounding_box": {
            "min": scale_vec3(aabb.min, k_from_m),
            "max": scale_vec3(aabb.max, k_from_m),
            "dimensions": scale_vec3(aabb.dimensions(), k_from_m),
        },
        "volume": volume_m3 * k_from_m.powi(3),
        "volume_m3": volume_m3,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(target_part_id), &mut resp);
    Ok(resp)
}

/// `import_stl`: Imports an external 3D STL file (ASCII or binary) as a new Part and displays it in the viewport.
pub fn import_stl(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let filepath = args
        .get("filepath")
        .or_else(|| args.get("file_path"))
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

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let mesh = mcp_cad_core::import_stl_file(path)?;

    let part_id = args
        .get("part_id")
        .or_else(|| args.get("id"))
        .and_then(|v| v.as_str())
        .map(|s| s.to_string())
        .unwrap_or_else(|| format!("part-{}", Uuid::new_v4().simple()));

    if part_id.eq_ignore_ascii_case("ground") || part_id.eq_ignore_ascii_case("world") {
        return Err(ServerError::InvalidArgument(format!(
            "Part id '{part_id}' is reserved ('ground' and 'world' are reserved static physics anchors)"
        )));
    }

    if state.parts.contains_key(&part_id) {
        return Err(ServerError::InvalidArgument(format!("Part with id '{}' already exists", part_id)));
    }

    let is_fixed = args.get("is_fixed").and_then(|v| v.as_bool()).unwrap_or(false);

    let mut part = Part::new(&part_id, &part_name, material);
    part.is_fixed = is_fixed;
    part.mesh = mesh.clone();
    part.ensure_imported_mesh_in_dag();
    part.bump_revision();

    if !part.mesh.vertices.is_empty() {
        let _ = state.physics_world.add_part(&part, !part.is_fixed);
    }

    let warnings = state.check_scale_sanity(&part, &[], None, unit);
    let bbox = mesh.bounding_box();
    let summary = part.summary();
    let volume_m3 = summary.volume;
    let mass_kg = summary.mass;
    let part_revision = part.revision;

    state.send_gui_mesh(&mesh);
    state.parts.insert(part_id.clone(), part);

    let mut resp = json!({
        "part_id": part_id,
        "name": part_name,
        "is_fixed": is_fixed,
        "revision": part_revision,
        "vertices_count": mesh.vertices.len(),
        "triangles_count": mesh.indices.len(),
        "bounding_box": {
            "min": scale_vec3(bbox.min, k_from_m),
            "max": scale_vec3(bbox.max, k_from_m),
            "dimensions": scale_vec3(bbox.dimensions(), k_from_m),
        },
        "volume": volume_m3 * k_from_m.powi(3),
        "volume_m3": volume_m3,
        "mass_kg": mass_kg,
        "mass": mass_kg,
        "status": "stl_imported",
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(&part_id), &mut resp);
    Ok(resp)
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
        if !c.is_finite() || c <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "chord_tolerance must be greater than 0".to_string(),
            ));
        }
    }
    if let Some(a) = angular_tolerance_deg {
        if !a.is_finite() || a <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "angular_tolerance_deg must be greater than 0".to_string(),
            ));
        }
    }
    if let Some(m) = max_edge_length {
        if !m.is_finite() || m <= 0.0 {
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

    let unit = state.resolve_length_unit(args)?;
    let k_to_m = unit.to_meters_factor();

    let chord_tolerance = args
        .get("chord_tolerance")
        .and_then(|v| v.as_f64())
        .map(|c| c * k_to_m);

    let angular_tolerance_deg = args
        .get("angular_tolerance_deg")
        .and_then(|v| v.as_f64());

    let max_edge_length = args
        .get("max_edge_length")
        .and_then(|v| v.as_f64())
        .map(|m| m * k_to_m);

    let (settings, quality_applied) = resolve_tessellation_settings(
        quality,
        chord_tolerance,
        angular_tolerance_deg,
        max_edge_length,
        "fine",
    )?;

    let (triangles_count, part_revision) = {
        let part = state
            .parts
            .get_mut(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

        if part.dag.nodes.is_empty() && part.mesh().vertices.is_empty() {
            return Err(ServerError::InvalidArgument(
                "Cannot export empty part mesh to STL. Add features first.".to_string(),
            ));
        }

        let prev_mesh = part.mesh.clone();
        let prev_cached_mesh = part.cached_mesh.clone();
        let prev_cached_settings = part.cached_settings.clone();
        let prev_revision = part.revision;

        if !part.dag.nodes.is_empty() {
            if let Err(e) = part.tessellate(&settings) {
                part.mesh = prev_mesh;
                part.cached_mesh = prev_cached_mesh;
                part.cached_settings = prev_cached_settings;
                part.revision = prev_revision;
                return Err(ServerError::Cad(e));
            }
        }
        part.revision = prev_revision;

        if part.mesh().vertices.is_empty() {
            part.mesh = prev_mesh;
            part.cached_mesh = prev_cached_mesh;
            part.cached_settings = prev_cached_settings;
            return Err(ServerError::InvalidArgument(
                "Cannot export empty part mesh to STL. Add features first.".to_string(),
            ));
        }

        let path = Path::new(filepath);
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                if let Err(e) = std::fs::create_dir_all(parent) {
                    part.mesh = prev_mesh;
                    part.cached_mesh = prev_cached_mesh;
                    part.cached_settings = prev_cached_settings;
                    return Err(ServerError::Io(e));
                }
            }
        }

        let mesh = part.mesh();
        let write_res = if binary {
            write_stl_binary_file(mesh, path)
        } else {
            write_stl_ascii_file(mesh, &part.name, path)
        };
        if let Err(e) = write_res {
            part.mesh = prev_mesh;
            part.cached_mesh = prev_cached_mesh;
            part.cached_settings = prev_cached_settings;
            return Err(ServerError::Cad(e));
        }

        (part.mesh().triangle_count(), part.revision)
    };

    if state.physics_world.has_part(part_id) {
        if let Some(part) = state.parts.get(part_id) {
            if !part.mesh().vertices.is_empty() {
                let _ = state.physics_world.update_part_collider(part);
            }
        }
    }

    Ok(json!({
        "part_id": part_id,
        "revision": part_revision,
        "filepath": filepath,
        "file_path": filepath,
        "triangles_count": triangles_count,
        "triangle_count": triangles_count,
        "quality": quality_applied,
        "quality_applied": quality_applied,
        "stl_vertex_unit": "m",
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
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

    let unit = state.resolve_length_unit(args)?;
    let k_to_m = unit.to_meters_factor();
    let k_from_m = unit.from_meters_factor();

    let chord_tolerance = args
        .get("chord_tolerance")
        .and_then(|v| v.as_f64())
        .map(|c| c * k_to_m);

    let angular_tolerance_deg = args
        .get("angular_tolerance_deg")
        .and_then(|v| v.as_f64());

    let max_edge_length = args
        .get("max_edge_length")
        .and_then(|v| v.as_f64())
        .map(|m| m * k_to_m);

    let (settings, quality_applied) = resolve_tessellation_settings(
        quality,
        chord_tolerance,
        angular_tolerance_deg,
        max_edge_length,
        "medium",
    )?;

    let (mesh_clone, volume_m3, part_revision) = {
        let part = state
            .parts
            .get_mut(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

        let prev_revision = part.revision;
        if !part.dag.nodes.is_empty() {
            part.tessellate(&settings)?;
        }
        part.revision = prev_revision;

        (part.mesh().clone(), part.volume(), part.revision)
    };

    state.send_gui_mesh(&mesh_clone);

    if state.physics_world.has_part(part_id) {
        if let Some(part) = state.parts.get(part_id) {
            if !part.mesh().vertices.is_empty() {
                let _ = state.physics_world.update_part_collider(part);
            }
        }
    }

    let vertex_count = mesh_clone.vertices.len();
    let triangle_count = mesh_clone.triangle_count();
    let volume_unit = volume_m3 * k_from_m.powi(3);

    Ok(json!({
        "part_id": part_id,
        "revision": part_revision,
        "quality_applied": quality_applied,
        "vertex_count": vertex_count,
        "vertices_count": vertex_count,
        "triangle_count": triangle_count,
        "triangles_count": triangle_count,
        "volume": volume_unit,
        "volume_m3": volume_m3,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// Formats a single `FaceInfo` (stored in SI meters) into the requested output `LengthUnit`.
fn format_face_info(f: &mcp_cad_core::FaceInfo, k_from_m: f64, k_area: f64) -> Value {
    json!({
        "id": f.id,
        "name": f.name,
        "aliases": f.aliases,
        "role": f.role,
        "kind": f.kind,
        "origin_feature": f.origin_feature,
        "area": f.area * k_area,
        "area_m2": f.area,
        "centroid": scale_vec3(f.centroid, k_from_m),
        "sample_point": scale_vec3(f.sample_point, k_from_m),
        "normal_or_axis": f.normal_or_axis,
        "radius": f.radius.map(|r| r * k_from_m),
        "triangles_count": f.triangles_count,
    })
}

fn vec3_dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn vec3_cosine_similarity(a: [f64; 3], b: [f64; 3]) -> f64 {
    let na = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    let nb = (b[0] * b[0] + b[1] * b[1] + b[2] * b[2]).sqrt();
    if na < 1e-12 || nb < 1e-12 {
        return 0.0;
    }
    (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb)
}

fn parse_face_kind(s: &str) -> Result<mcp_cad_core::FaceKind, ServerError> {
    mcp_cad_core::FaceKind::from_str_ignore_case(s).ok_or_else(|| {
        ServerError::InvalidArgument(format!(
            "Invalid face kind '{s}' in 'where' filter. Supported: plane, planar, cylinder, cylindrical, cone, conical, sphere, spherical, torus, toroidal, helical, other, freeform"
        ))
    })
}

fn resolve_face_by_name<'a>(
    faces: &'a [mcp_cad_core::FaceInfo],
    target: &str,
) -> Result<&'a mcp_cad_core::FaceInfo, ServerError> {
    // 0. Wildcard prefix match (e.g. "hole_wall:*" or "cut:*")
    if let Some(prefix) = target.strip_suffix('*') {
        let matched: Vec<&mcp_cad_core::FaceInfo> = faces
            .iter()
            .filter(|f| {
                f.name.starts_with(prefix)
                    || f.role.starts_with(prefix)
                    || f.aliases.iter().any(|a| a.starts_with(prefix))
            })
            .collect();
        if matched.len() == 1 {
            return Ok(matched[0]);
        }
        if matched.len() > 1 {
            let candidates: Vec<String> = matched
                .iter()
                .map(|f| format!("id={} ({})", f.id, f.name))
                .collect();
            return Err(ServerError::InvalidArgument(format!(
                "AMBIGUOUS_FACE_SELECTOR: Ambiguous face selector '{target}' matched {} faces: [{}]",
                matched.len(),
                candidates.join(", ")
            )));
        }
        let available: Vec<&str> = faces.iter().map(|f| f.name.as_str()).collect();
        return Err(ServerError::InvalidArgument(format!(
            "FACE_NOT_FOUND: Face not found for selector '{target}'. Available faces: [{}]",
            available.join(", ")
        )));
    }

    // 1. Exact canonical name match
    let exact: Vec<&mcp_cad_core::FaceInfo> = faces.iter().filter(|f| f.name == target).collect();
    if exact.len() == 1 {
        return Ok(exact[0]);
    }
    if exact.len() > 1 {
        let candidates: Vec<String> = exact
            .iter()
            .map(|f| format!("id={} ({})", f.id, f.name))
            .collect();
        return Err(ServerError::InvalidArgument(format!(
            "AMBIGUOUS_FACE_SELECTOR: Ambiguous face selector '{target}' matched {} faces: [{}]",
            exact.len(),
            candidates.join(", ")
        )));
    }

    // 2. Alias or role match
    let matched: Vec<&mcp_cad_core::FaceInfo> = faces
        .iter()
        .filter(|f| f.role == target || f.aliases.iter().any(|a| a == target))
        .collect();
    if matched.len() == 1 {
        return Ok(matched[0]);
    }
    if matched.len() > 1 {
        let candidates: Vec<String> = matched
            .iter()
            .map(|f| format!("id={} ({})", f.id, f.name))
            .collect();
        return Err(ServerError::InvalidArgument(format!(
            "AMBIGUOUS_FACE_SELECTOR: Ambiguous face selector '{target}' matched {} faces: [{}]",
            matched.len(),
            candidates.join(", ")
        )));
    }

    let available: Vec<&str> = faces.iter().map(|f| f.name.as_str()).collect();
    Err(ServerError::InvalidArgument(format!(
        "FACE_NOT_FOUND: Face not found for selector '{target}'. Available faces: [{}]",
        available.join(", ")
    )))
}

/// Resolves a `FaceSelector` (string name/alias, `{"face": ...}`, `{"face_id": ...}`,
/// `{"faces": [...]}`, or `{"where": {...}}`) against `mesh.faces`.
pub fn resolve_face_selector<'a>(
    mesh: &'a mcp_cad_core::TriangleMesh,
    selector: &Value,
    unit: LengthUnit,
) -> Result<Vec<&'a mcp_cad_core::FaceInfo>, ServerError> {
    let faces = &mesh.faces;
    if let Some(name) = selector.as_str() {
        return Ok(vec![resolve_face_by_name(faces, name)?]);
    }
    let obj = selector.as_object().ok_or_else(|| {
        ServerError::InvalidArgument(
            "Invalid 'face_selector': expected string or object with 'face', 'face_id', 'faces', or 'where'"
                .to_string(),
        )
    })?;

    if let Some(name_val) = obj.get("face") {
        let name = name_val.as_str().ok_or_else(|| {
            ServerError::InvalidArgument("Field 'face' in face_selector must be a string".to_string())
        })?;
        return Ok(vec![resolve_face_by_name(faces, name)?]);
    }

    if let Some(id_val) = obj.get("face_id") {
        if let Some(id) = id_val.as_u64() {
            let id = id as u32;
            let found = faces.iter().find(|f| f.id == id).ok_or_else(|| {
                let available: Vec<u32> = faces.iter().map(|f| f.id).collect();
                ServerError::InvalidArgument(format!(
                    "FACE_NOT_FOUND: Face not found with face_id={id}. Available face_ids: {available:?}"
                ))
            })?;
            return Ok(vec![found]);
        } else if let Some(name) = id_val.as_str() {
            return Ok(vec![resolve_face_by_name(faces, name)?]);
        } else {
            return Err(ServerError::InvalidArgument(
                "Field 'face_id' in face_selector must be a non-negative integer or face name string".to_string(),
            ));
        }
    }

    if let Some(arr_val) = obj.get("faces") {
        let arr = arr_val.as_array().ok_or_else(|| {
            ServerError::InvalidArgument("Field 'faces' in face_selector must be an array".to_string())
        })?;
        let mut out = Vec::new();
        for item in arr {
            for f in resolve_face_selector(mesh, item, unit)? {
                if !out.iter().any(|existing: &&mcp_cad_core::FaceInfo| existing.id == f.id) {
                    out.push(f);
                }
            }
        }
        return Ok(out);
    }

    let where_val = obj.get("where").unwrap_or(selector);
    let where_obj = where_val.as_object().ok_or_else(|| {
        ServerError::InvalidArgument("Field 'where' in face_selector must be an object".to_string())
    })?;

    let k_to_m = unit.to_meters_factor();
    let k_area_to_m2 = k_to_m * k_to_m;
    let tol_m = where_obj
        .get("tolerance")
        .and_then(|v| v.as_f64())
        .map(|t| t * k_to_m)
        .unwrap_or(1e-4); // default 0.1 mm = 1e-4 m

    let kind_filter = if let Some(kv) = where_obj.get("kind").and_then(|v| v.as_str()) {
        Some(parse_face_kind(kv)?)
    } else {
        None
    };
    let radius_filter = where_obj
        .get("radius")
        .and_then(|v| v.as_f64())
        .map(|r| r * k_to_m);
    let normal_filter = parse_vec3(where_val, "normal");
    let axis_filter = parse_vec3(where_val, "axis");
    let near_filter = parse_vec3(where_val, "near").map(|p| scale_vec3(p, k_to_m));
    let closest_filter = parse_vec3(where_val, "closest").map(|p| scale_vec3(p, k_to_m));
    let origin_filter = where_obj.get("origin_feature").and_then(|v| v.as_str());
    let role_filter = where_obj.get("role").and_then(|v| v.as_str());
    let min_area_filter = where_obj
        .get("min_area")
        .and_then(|v| v.as_f64())
        .map(|a| a * k_area_to_m2);
    let max_area_filter = where_obj
        .get("max_area")
        .and_then(|v| v.as_f64())
        .map(|a| a * k_area_to_m2);

    let mut candidates: Vec<&mcp_cad_core::FaceInfo> = faces
        .iter()
        .filter(|f| {
            if let Some(k) = kind_filter {
                if f.kind != k {
                    return false;
                }
            }
            if let Some(target_r) = radius_filter {
                match f.radius {
                    Some(r) if (r - target_r).abs() <= tol_m => {}
                    _ => return false,
                }
            }
            if let Some(n) = normal_filter {
                if vec3_cosine_similarity(f.normal_or_axis, n) < 0.99 {
                    return false;
                }
            }
            if let Some(ax) = axis_filter {
                if vec3_cosine_similarity(f.normal_or_axis, ax).abs() < 0.99 {
                    return false;
                }
            }
            if let Some(origin) = origin_filter {
                let matches_origin = f.origin_feature.as_deref() == Some(origin)
                    || f.aliases.iter().any(|a| a.starts_with(&format!("{origin}:")));
                if !matches_origin {
                    return false;
                }
            }
            if let Some(role) = role_filter {
                if f.role != role && !f.aliases.iter().any(|a| a == role || a.ends_with(&format!(":{role}"))) {
                    return false;
                }
            }
            if let Some(min_a) = min_area_filter {
                if f.area < min_a {
                    return false;
                }
            }
            if let Some(max_a) = max_area_filter {
                if f.area > max_a {
                    return false;
                }
            }
            if let Some(near_pt) = near_filter {
                let d = vec3_dist(f.centroid, near_pt).min(vec3_dist(f.sample_point, near_pt));
                if d > tol_m {
                    return false;
                }
            }
            true
        })
        .collect();

    if let Some(closest_pt) = closest_filter {
        candidates.sort_by(|a, b| {
            let da = vec3_dist(a.centroid, closest_pt).min(vec3_dist(a.sample_point, closest_pt));
            let db = vec3_dist(b.centroid, closest_pt).min(vec3_dist(b.sample_point, closest_pt));
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        });
        candidates.truncate(1);
    }

    if candidates.is_empty() {
        let available: Vec<&str> = faces.iter().map(|f| f.name.as_str()).collect();
        return Err(ServerError::InvalidArgument(format!(
            "FACE_NOT_FOUND: Face not found matching 'where' filter. Available faces: [{}]",
            available.join(", ")
        )));
    }

    Ok(candidates)
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

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();
    let k_area = k_from_m.powi(2);
    let k_vol = k_from_m.powi(3);

    let inspection = part.inspect_geometry();
    let mesh_validation = mcp_cad_core::validate_mesh(&part.mesh, None);

    let planar_faces: Vec<Value> = inspection
        .detected_planar_faces
        .iter()
        .map(|f| {
            json!({
                "normal": f.normal,
                "center": scale_vec3(f.center, k_from_m),
                "area": f.area * k_area,
                "area_m2": f.area,
            })
        })
        .collect();

    let selected_faces: Vec<&mcp_cad_core::FaceInfo> = if let Some(sel) = args.get("face_selector") {
        resolve_face_selector(&part.mesh, sel, unit)?
    } else if let Some(where_val) = args.get("where") {
        let wrapper = json!({ "where": where_val });
        resolve_face_selector(&part.mesh, &wrapper, unit)?
    } else {
        part.mesh.faces.iter().collect()
    };

    let faces_json: Vec<Value> = selected_faces
        .into_iter()
        .map(|f| format_face_info(f, k_from_m, k_area))
        .collect();

    Ok(json!({
        "part_id": part_id,
        "bounding_box": {
            "min": scale_vec3(inspection.bounding_box.min, k_from_m),
            "max": scale_vec3(inspection.bounding_box.max, k_from_m),
            "dimensions": scale_vec3(inspection.bounding_box.dimensions(), k_from_m),
        },
        "volume": inspection.volume * k_vol,
        "volume_m3": inspection.volume,
        "mass": part.mass(),
        "mass_kg": part.mass(),
        "surface_area": inspection.surface_area * k_area,
        "surface_area_m2": inspection.surface_area,
        "center_of_mass": scale_vec3(inspection.center_of_mass, k_from_m),
        "faces_count": inspection.faces_count,
        "vertices_count": inspection.vertices_count,
        "faces": faces_json,
        "detected_planar_faces": planar_faces,
        "mesh_validation": mesh_validation,
        "revision": part.revision,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
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

    let strict_slivers = args
        .get("strict_slivers")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let report = mcp_cad_core::diagnostics::validate_mesh_options(&part.mesh, expected_genus, strict_slivers);
    let vol_m3 = report.geometry.signed_volume;
    let area_m2 = part.surface_area();
    let mut val = serde_json::to_value(&report).map_err(ServerError::Json)?;
    if let Some(obj) = val.as_object_mut() {
        obj.insert("part_id".to_string(), json!(part.id));
        obj.insert("revision".to_string(), json!(part.revision));
        obj.insert("volume_m3".to_string(), json!(vol_m3));
        obj.insert("surface_area_m2".to_string(), json!(area_m2));
        obj.insert("volume".to_string(), json!(vol_m3 * k_from_m.powi(3)));
        obj.insert(
            "surface_area".to_string(),
            json!(area_m2 * k_from_m.powi(2)),
        );
        obj.insert("length_unit".to_string(), json!(unit.as_str()));
        obj.insert("units".to_string(), unit.units_metadata());
    }
    Ok(val)
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

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let tool_mesh = {
        let tool_part = state
            .parts
            .get(tool_part_id)
            .ok_or_else(|| ServerError::PartNotFound(tool_part_id.to_string()))?;
        tool_part.mesh().clone()
    };

    let (feature_id, vertices_count, triangles_count, aabb, volume_m3, mesh_clone, part_rev) = {
        let target_part = state
            .parts
            .get_mut(target_part_id)
            .ok_or_else(|| ServerError::PartNotFound(target_part_id.to_string()))?;

        let prev_dag = target_part.dag.clone();
        let prev_mesh = target_part.mesh.clone();
        let prev_cached_mesh = target_part.cached_mesh.clone();
        let prev_cached_settings = target_part.cached_settings.clone();
        let prev_revision = target_part.revision;
        target_part.ensure_imported_mesh_in_dag();
        let feature_id = target_part.dag.add_feature(
            FeatureOp::BooleanUnionMesh { tool_mesh },
            None,
        );
        if let Err(e) = target_part.rebuild() {
            target_part.dag = prev_dag;
            target_part.mesh = prev_mesh;
            target_part.cached_mesh = prev_cached_mesh;
            target_part.cached_settings = prev_cached_settings;
            target_part.revision = prev_revision;
            return Err(ServerError::Cad(e));
        }

        let aabb = target_part.bounding_box();
        let volume_m3 = target_part.volume();
        let mesh = target_part.mesh().clone();
        let part_rev = target_part.revision;
        (
            feature_id,
            mesh.vertices.len(),
            mesh.triangle_count(),
            aabb,
            volume_m3,
            mesh,
            part_rev,
        )
    };

    let warnings = if let Some(target_part) = state.parts.get(target_part_id) {
        state.check_scale_sanity(target_part, &[], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    if let Some(target_part) = state.parts.get(target_part_id) {
        if !target_part.mesh().vertices.is_empty() {
            let _ = state.physics_world.update_part_collider(target_part);
        }
    }

    let mut resp = json!({
        "part_id": target_part_id,
        "target_part_id": target_part_id,
        "feature_id": feature_id.to_string(),
        "revision": part_rev,
        "status": "union_applied",
        "vertices_count": vertices_count,
        "triangles_count": triangles_count,
        "bounding_box": {
            "min": scale_vec3(aabb.min, k_from_m),
            "max": scale_vec3(aabb.max, k_from_m),
            "dimensions": scale_vec3(aabb.dimensions(), k_from_m),
        },
        "volume": volume_m3 * k_from_m.powi(3),
        "volume_m3": volume_m3,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(target_part_id), &mut resp);
    Ok(resp)
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

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let tool_mesh = {
        let tool_part = state
            .parts
            .get(tool_part_id)
            .ok_or_else(|| ServerError::PartNotFound(tool_part_id.to_string()))?;
        tool_part.mesh().clone()
    };

    let (feature_id, vertices_count, triangles_count, aabb, volume_m3, mesh_clone, part_rev) = {
        let target_part = state
            .parts
            .get_mut(target_part_id)
            .ok_or_else(|| ServerError::PartNotFound(target_part_id.to_string()))?;

        let prev_dag = target_part.dag.clone();
        let prev_mesh = target_part.mesh.clone();
        let prev_cached_mesh = target_part.cached_mesh.clone();
        let prev_cached_settings = target_part.cached_settings.clone();
        let prev_revision = target_part.revision;
        target_part.ensure_imported_mesh_in_dag();
        let feature_id = target_part.dag.add_feature(
            FeatureOp::BooleanIntersectionMesh { tool_mesh },
            None,
        );
        if let Err(e) = target_part.rebuild() {
            target_part.dag = prev_dag;
            target_part.mesh = prev_mesh;
            target_part.cached_mesh = prev_cached_mesh;
            target_part.cached_settings = prev_cached_settings;
            target_part.revision = prev_revision;
            return Err(ServerError::Cad(e));
        }

        let aabb = target_part.bounding_box();
        let volume_m3 = target_part.volume();
        let mesh = target_part.mesh().clone();
        let part_rev = target_part.revision;
        (
            feature_id,
            mesh.vertices.len(),
            mesh.triangle_count(),
            aabb,
            volume_m3,
            mesh,
            part_rev,
        )
    };

    let warnings = if let Some(target_part) = state.parts.get(target_part_id) {
        state.check_scale_sanity(target_part, &[], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    if let Some(target_part) = state.parts.get(target_part_id) {
        if !target_part.mesh().vertices.is_empty() {
            let _ = state.physics_world.update_part_collider(target_part);
        }
    }

    let mut resp = json!({
        "part_id": target_part_id,
        "target_part_id": target_part_id,
        "feature_id": feature_id.to_string(),
        "revision": part_rev,
        "status": "intersection_applied",
        "vertices_count": vertices_count,
        "triangles_count": triangles_count,
        "bounding_box": {
            "min": scale_vec3(aabb.min, k_from_m),
            "max": scale_vec3(aabb.max, k_from_m),
            "dimensions": scale_vec3(aabb.dimensions(), k_from_m),
        },
        "volume": volume_m3 * k_from_m.powi(3),
        "volume_m3": volume_m3,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(target_part_id), &mut resp);
    Ok(resp)
}

/// 24. `transform_part`: Applies translation, rotation (RPY deg), and scale to a part's DAG.
pub fn transform_part(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let unit = state.resolve_length_unit(args)?;
    let k_to_m = unit.to_meters_factor();
    let k_from_m = unit.from_meters_factor();

    let mut translation =
        scale_vec3(parse_vec3(args, "translation").unwrap_or([0.0, 0.0, 0.0]), k_to_m);
    let rotation_deg = parse_vec3(args, "rotation_deg")
        .or_else(|| parse_vec3(args, "rotation_rpy_deg"))
        .unwrap_or([0.0, 0.0, 0.0]);
    let scale = parse_vec3(args, "scale").unwrap_or([1.0, 1.0, 1.0]);

    if scale[0] == 0.0 || scale[1] == 0.0 || scale[2] == 0.0 {
        return Err(ServerError::InvalidArgument(
            "Scale components cannot be zero".to_string(),
        ));
    }

    // Optional `align_face`:
    //   - {"face": <selector>, "target_point"|"to_point": [x, y, z], "offset": 0.0}
    //   - {"source_face": <selector>, "target_part_id": "...", "target_face": <selector>, "offset": 0.0}
    if let Some(align_val) = args.get("align_face") {
        let align_obj = align_val.as_object().ok_or_else(|| {
            ServerError::InvalidArgument("Field 'align_face' must be an object".to_string())
        })?;
        let src_sel = align_obj
            .get("source_face")
            .or_else(|| align_obj.get("face"))
            .ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Missing 'source_face' or 'face' in 'align_face'".to_string(),
                )
            })?;
        let offset_m = align_obj
            .get("offset")
            .and_then(|v| v.as_f64())
            .unwrap_or(0.0)
            * k_to_m;

        let src_part = state
            .parts
            .get(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;
        let src_faces = resolve_face_selector(src_part.mesh(), src_sel, unit)?;
        let src_f = src_faces[0];

        let target_pt = if let Some(pt) = parse_vec3(align_val, "target_point")
            .or_else(|| parse_vec3(align_val, "to_point"))
        {
            let pt_m = scale_vec3(pt, k_to_m);
            [
                pt_m[0] + src_f.normal_or_axis[0] * offset_m,
                pt_m[1] + src_f.normal_or_axis[1] * offset_m,
                pt_m[2] + src_f.normal_or_axis[2] * offset_m,
            ]
        } else {
            let tgt_part_id = align_obj
                .get("target_part_id")
                .and_then(|v| v.as_str())
                .ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Missing 'target_part_id' or 'target_point' in 'align_face'".to_string(),
                    )
                })?;
            let tgt_sel = align_obj.get("target_face").ok_or_else(|| {
                ServerError::InvalidArgument("Missing 'target_face' in 'align_face'".to_string())
            })?;
            let tgt_part = state
                .parts
                .get(tgt_part_id)
                .ok_or_else(|| ServerError::PartNotFound(tgt_part_id.to_string()))?;
            let tgt_faces = resolve_face_selector(tgt_part.mesh(), tgt_sel, unit)?;
            let tgt_f = tgt_faces[0];
            [
                tgt_f.centroid[0] + tgt_f.normal_or_axis[0] * offset_m,
                tgt_f.centroid[1] + tgt_f.normal_or_axis[1] * offset_m,
                tgt_f.centroid[2] + tgt_f.normal_or_axis[2] * offset_m,
            ]
        };

        translation[0] += target_pt[0] - src_f.centroid[0];
        translation[1] += target_pt[1] - src_f.centroid[1];
        translation[2] += target_pt[2] - src_f.centroid[2];
    }

    // Optional `pivot_face`: rotate and scale about the centroid of the selected face
    if let Some(pivot_sel) = args.get("pivot_face") {
        let src_part = state
            .parts
            .get(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;
        let pivot_faces = resolve_face_selector(src_part.mesh(), pivot_sel, unit)?;
        let p = pivot_faces[0].centroid;

        // FeatureOp::Transform applies scale, then rotate_rpy_deg, then translate:
        // x' = R * (S * x) + t_final. To keep pivot `p` at `p + translation`, t_final = translation + p - R * (S * p).
        let mut dummy = mcp_cad_core::TriangleMesh::new(
            vec![mcp_cad_core::Vertex::new(p, [0.0, 0.0, 1.0])],
            vec![],
        );
        dummy.scale(scale);
        dummy.rotate_rpy_deg(rotation_deg[0], rotation_deg[1], rotation_deg[2]);
        let rsp = dummy.vertices[0].position;
        translation[0] += p[0] - rsp[0];
        translation[1] += p[1] - rsp[1];
        translation[2] += p[2] - rsp[2];
    }

    let (feature_id, aabb, volume_m3, mesh_clone, part_rev) = {
        let part = state
            .parts
            .get_mut(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

        let prev_dag = part.dag.clone();
        let prev_mesh = part.mesh.clone();
        let prev_cached_mesh = part.cached_mesh.clone();
        let prev_cached_settings = part.cached_settings.clone();
        let prev_revision = part.revision;
        part.ensure_imported_mesh_in_dag();
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
            part.cached_mesh = prev_cached_mesh;
            part.cached_settings = prev_cached_settings;
            part.revision = prev_revision;
            return Err(ServerError::Cad(e));
        }

        let aabb = part.bounding_box();
        let volume = part.volume();
        let mesh = part.mesh().clone();
        let part_rev = part.revision;
        (feature_id, aabb, volume, mesh, part_rev)
    };

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &[], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    if let Some(part) = state.parts.get(part_id) {
        if !part.mesh().vertices.is_empty() {
            let _ = state.physics_world.update_part_collider(part);
        }
    }

    let mut resp = json!({
        "part_id": part_id,
        "feature_id": feature_id.to_string(),
        "revision": part_rev,
        "status": "transformed",
        "bounding_box": {
            "min": scale_vec3(aabb.min, k_from_m),
            "max": scale_vec3(aabb.max, k_from_m),
            "dimensions": scale_vec3(aabb.dimensions(), k_from_m),
        },
        "volume": volume_m3 * k_from_m.powi(3),
        "volume_m3": volume_m3,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(part_id), &mut resp);
    Ok(resp)
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

    let unit = state.length_unit;

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
        },
        "units": unit.units_metadata(),
    }))
}

/// 26. `batch_cad_operations`: Sequentially executes a batch of MCP CAD operations.
pub fn batch_cad_operations(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let operations = args
        .get("operations")
        .and_then(|v| v.as_array())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required array 'operations'".to_string()))?;

     let prev_length_unit = state.length_unit;
    let prev_parts = state.parts.clone();
    let prev_materials = state.materials.clone();
    let prev_physics_world = state.physics_world.clone();
    let prev_last_fea_reports = state.last_fea_reports.clone();
    let prev_last_fea_revisions = state.last_fea_revisions.clone();
    let prev_fea_case_reports = state.fea_case_reports.clone();
    let prev_last_fea_meshes = state.last_fea_meshes.clone();
    let prev_last_simulation = state.last_simulation.clone();
    let prev_requirements = state.requirements.clone();

    let mut results = Vec::with_capacity(operations.len());

    for (idx, op) in operations.iter().enumerate() {
        let tool_name = match op.get("tool").or_else(|| op.get("name")).and_then(|v| v.as_str()) {
            Some(name) => name,
            None => {
                state.length_unit = prev_length_unit;
                state.parts = prev_parts;
                state.materials = prev_materials;
                state.physics_world = prev_physics_world;
                state.last_fea_reports = prev_last_fea_reports;
                state.last_fea_revisions = prev_last_fea_revisions;
                state.fea_case_reports = prev_fea_case_reports;
                state.last_fea_meshes = prev_last_fea_meshes;
                state.last_simulation = prev_last_simulation;
                state.requirements = prev_requirements;
                return Err(ServerError::InvalidArgument(format!("Operation at index {idx} missing 'tool' field")));
            }
        };

        if tool_name == "batch_cad_operations" {
            state.length_unit = prev_length_unit;
            state.parts = prev_parts;
            state.materials = prev_materials;
            state.physics_world = prev_physics_world;
            state.last_fea_reports = prev_last_fea_reports;
            state.last_fea_revisions = prev_last_fea_revisions;
            state.fea_case_reports = prev_fea_case_reports;
            state.last_fea_meshes = prev_last_fea_meshes;
            state.last_simulation = prev_last_simulation;
            state.requirements = prev_requirements;
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
                state.length_unit = prev_length_unit;
                state.parts = prev_parts;
                state.materials = prev_materials;
                state.physics_world = prev_physics_world;
                state.last_fea_reports = prev_last_fea_reports;
                state.last_fea_revisions = prev_last_fea_revisions;
                state.fea_case_reports = prev_fea_case_reports;
                state.last_fea_meshes = prev_last_fea_meshes;
                state.last_simulation = prev_last_simulation;
                state.requirements = prev_requirements;
                return Err(ServerError::InvalidArgument(format!(
                    "Batch operation at index {idx} ('{tool_name}') failed: {err}"
                )));
            }
        }
    }

    let unit = state.length_unit;

    let mut resp = json!({
        "success": true,
        "count": operations.len(),
        "completed_steps": operations.len(),
        "results": results,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    });
    super::requirements::attach_auto_check_if_enabled(state, None, &mut resp);
    Ok(resp)
}

/// 28. `optimize_feature_tree`: Runs the Cost-Based CAD Query Optimizer (CBO) on a part's parametric FeatureTree (DAG).
pub fn optimize_feature_tree(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let apply = args
        .get("apply")
        .or_else(|| args.get("apply_best_plan"))
        .and_then(|v| v.as_bool())
        .unwrap_or(true);

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();
    let k_vol = k_from_m.powi(3);

    let (mut report, mesh_opt, part_rev, vol_m3) = {
        let part = state
            .parts
            .get_mut(part_id)
            .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

        let report = part.optimize_dag(apply)?;
        let mesh = if apply {
            Some(part.mesh().clone())
        } else {
            None
        };
        let rev = part.revision;
        let v_m3 = part.volume();
        (report, mesh, rev, v_m3)
    };

    if let Some(mesh_clone) = mesh_opt {
        state.send_gui_mesh(&mesh_clone);
        if let Some(part) = state.parts.get(part_id) {
            if !part.mesh().vertices.is_empty() {
                let _ = state.physics_world.update_part_collider(part);
            }
        }
    }

    for node in &mut report.logical_plan.dag {
        node.op = scale_feature_op_from_meters(&node.op, k_from_m);
    }
    for node in &mut report.physical_plan.dag {
        node.op = scale_feature_op_from_meters(&node.op, k_from_m);
    }
    for plan in &mut report.candidate_plans {
        for node in &mut plan.dag {
            node.op = scale_feature_op_from_meters(&node.op, k_from_m);
        }
    }

    let mut val = serde_json::to_value(&report).map_err(ServerError::Json)?;
    if let Some(obj) = val.as_object_mut() {
        obj.insert("revision".to_string(), json!(part_rev));
        obj.insert("volume".to_string(), json!(vol_m3 * k_vol));
        obj.insert("volume_m3".to_string(), json!(vol_m3));
        obj.insert("length_unit".to_string(), json!(unit.as_str()));
        obj.insert("units".to_string(), unit.units_metadata());
        if let Some(eq_obj) = obj
            .get_mut("equivalence_verification")
            .and_then(|v| v.as_object_mut())
        {
            eq_obj.insert(
                "volume_before".to_string(),
                json!(report.equivalence_verification.volume_before_m3 * k_vol),
            );
            eq_obj.insert(
                "volume_after".to_string(),
                json!(report.equivalence_verification.volume_after_m3 * k_vol),
            );
        }
    }
    if apply {
        super::requirements::attach_auto_check_if_enabled(state, Some(part_id), &mut val);
    }
    Ok(val)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::physics;

    #[test]
    fn test_p2_t12_fixed_part_preserved_and_batch_physics_rollback() {
        let mut state = EngineState::new().expect("EngineState init");

        // 1. Create a fixed base part (`is_fixed: true`) and a tool part (`is_fixed: false`)
        create_part(
            &json!({
                "name": "Fixed Base",
                "part_id": "fixed_base",
                "material": "steel_structural_aisi1020",
                "is_fixed": true
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "fixed_base",
                "primitive_type": "box",
                "params": { "width": 100.0, "height": 40.0, "depth": 40.0 }
            }),
            &mut state,
        )
        .unwrap();

        create_part(
            &json!({
                "name": "Tool Block",
                "part_id": "tool_block",
                "material": "aluminum_6061_t6",
                "is_fixed": false
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "tool_block",
                "primitive_type": "box",
                "params": { "width": 30.0, "height": 30.0, "depth": 30.0, "position": [10.0, 0.0, 0.0] }
            }),
            &mut state,
        )
        .unwrap();

        // 2. Verify boolean_difference, boolean_union, boolean_intersection, transform_part,
        //    and optimize_feature_tree register `fixed_base` with `is_dynamic == false` (`!part.is_fixed`).
        boolean_difference(
            &json!({
                "target_part_id": "fixed_base",
                "tool_part_id": "tool_block"
            }),
            &mut state,
        )
        .unwrap();
        assert!(state.physics_world.has_part("fixed_base"));
        assert_eq!(
            state.physics_world.is_part_dynamic("fixed_base").unwrap(),
            false,
            "boolean_difference must register fixed part as non-dynamic (!part.is_fixed)"
        );

        boolean_union(
            &json!({
                "target_part_id": "fixed_base",
                "tool_part_id": "tool_block"
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(
            state.physics_world.is_part_dynamic("fixed_base").unwrap(),
            false,
            "boolean_union must preserve !part.is_fixed"
        );

        boolean_intersection(
            &json!({
                "target_part_id": "fixed_base",
                "tool_part_id": "tool_block"
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(
            state.physics_world.is_part_dynamic("fixed_base").unwrap(),
            false,
            "boolean_intersection must preserve !part.is_fixed"
        );

        transform_part(
            &json!({
                "part_id": "fixed_base",
                "translation": [0.0, 5.0, 0.0]
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(
            state.physics_world.is_part_dynamic("fixed_base").unwrap(),
            false,
            "transform_part must preserve !part.is_fixed"
        );

        optimize_feature_tree(
            &json!({
                "part_id": "fixed_base",
                "apply": true
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(
            state.physics_world.is_part_dynamic("fixed_base").unwrap(),
            false,
            "optimize_feature_tree must preserve !part.is_fixed"
        );

        // Also check that a dynamic part (`is_fixed: false`) is registered as dynamic (`true`)
        transform_part(
            &json!({
                "part_id": "tool_block",
                "translation": [0.0, 0.0, 2.0]
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(
            state.physics_world.is_part_dynamic("tool_block").unwrap(),
            true,
            "transform_part must register dynamic part with is_dynamic == true"
        );

        // 3. Create an initial joint between fixed_base and tool_block before batch operation
        let j_init = physics::create_joint(
            &json!({
                "joint_type": "revolute",
                "part_a": "fixed_base",
                "part_b": "tool_block",
                "axis": [0.0, 0.0, 1.0]
            }),
            &mut state,
        )
        .unwrap();
        let initial_joint_id = j_init["joint_id"].as_str().unwrap().to_string();
        assert_eq!(state.physics_world.joints().len(), 1);

        // 4. Execute a batch_cad_operations that mutates physics_world (deletes tool_block,
        //    creates temp_part + joint) and then fails on the final step -> physics_world must roll back!
        let batch_err = batch_cad_operations(
            &json!({
                "operations": [
                    {
                        "tool": "delete_part",
                        "params": { "part_id": "tool_block" }
                    },
                    {
                        "tool": "create_part",
                        "params": {
                            "name": "Temp Rotor",
                            "part_id": "temp_rotor",
                            "material": "aluminum_6061_t6"
                        }
                    },
                    {
                        "tool": "add_feature_primitive",
                        "params": {
                            "part_id": "temp_rotor",
                            "primitive_type": "cylinder",
                            "params": { "radius": 10.0, "height": 20.0 }
                        }
                    },
                    {
                        "tool": "create_joint",
                        "params": {
                            "joint_type": "revolute",
                            "part_a": "ground",
                            "part_b": "temp_rotor",
                            "axis": [0.0, 0.0, 1.0]
                        }
                    },
                    {
                        "tool": "delete_part",
                        "params": { "part_id": "non_existent_part_triggering_rollback" }
                    }
                ]
            }),
            &mut state,
        );
        assert!(batch_err.is_err(), "Batch must fail on the invalid final step");

        // Verify full rollback of both `state.parts` and `state.physics_world`
        assert!(state.parts.contains_key("tool_block"));
        assert!(!state.parts.contains_key("temp_rotor"));
        assert!(
            state.physics_world.has_part("tool_block"),
            "physics_world must restore deleted part 'tool_block' on batch rollback"
        );
        assert!(
            !state.physics_world.has_part("temp_rotor"),
            "physics_world must not retain 'temp_rotor' added during failed batch"
        );
        assert_eq!(
            state.physics_world.joints().len(),
            1,
            "physics_world joints count must roll back to 1"
        );
        assert!(
            state.physics_world.joints().contains_key(&initial_joint_id),
            "Original joint must be restored in physics_world after batch rollback"
        );
    }

    #[test]
    fn test_p2_t13_through_hole_revolve_inner_radius_and_polygon_prism_aliases() {
        let mut state = EngineState::new().expect("EngineState init");
        state.length_unit = LengthUnit::M;

        // 1. Revolve with 2-point profile + inner_radius (T07 annular bushing)
        create_part(
            &json!({ "name": "Bushing", "part_id": "t07_bushing", "material": "brass_c36000" }),
            &mut state,
        )
        .expect("brass_c36000 alias should resolve");
        add_feature_primitive(
            &json!({
                "part_id": "t07_bushing",
                "primitive_type": "revolve",
                "params": {
                    "profile": [[0.035, -0.04], [0.035, 0.04]],
                    "inner_radius": 0.015,
                    "angle_deg": 360
                }
            }),
            &mut state,
        )
        .expect("2-point profile with inner_radius should succeed");
        let bushing_part = state.parts.get("t07_bushing").unwrap();
        let bushing_report = mcp_cad_core::validate_mesh(&bushing_part.mesh, Some(1));
        assert!(bushing_report.topology.is_watertight);
        assert_eq!(bushing_report.topology.genus.round() as i32, 1);

        // 2. Polygon prism with outer_radius and chamfer_radius + cut_hole with through: true (T08 nut)
        create_part(
            &json!({ "name": "Nut M10", "part_id": "t08_nut", "material": "steel_4140" }),
            &mut state,
        )
        .expect("steel_4140 alias should resolve");
        add_feature_primitive(
            &json!({
                "part_id": "t08_nut",
                "primitive_type": "polygon_prism",
                "params": {
                    "sides": 6,
                    "outer_radius": 0.0098,
                    "height": 0.008,
                    "chamfer_radius": 0.0008
                }
            }),
            &mut state,
        )
        .expect("polygon_prism with outer_radius and chamfer_radius should succeed");
        cut_hole(
            &json!({
                "part_id": "t08_nut",
                "position": [0.0, 0.0, 0.0],
                "normal": [0.0, 0.0, -1.0],
                "radius": 0.00425,
                "through": true
            }),
            &mut state,
        )
        .expect("cut_hole with through: true and no depth should succeed");
        let nut_part = state.parts.get("t08_nut").unwrap();
        let nut_report = mcp_cad_core::validate_mesh(&nut_part.mesh, Some(1));
        assert!(nut_report.topology.is_watertight);
        assert_eq!(nut_report.topology.genus.round() as i32, 1);

        // 3. cut_hole with 'direction' and 'diameter' aliases + 'through': true
        create_part(
            &json!({ "name": "Plate", "part_id": "plate_dir", "material": "Al6061-T6" }),
            &mut state,
        )
        .expect("Al6061-T6 alias should resolve");
        add_feature_primitive(
            &json!({
                "part_id": "plate_dir",
                "primitive_type": "box",
                "params": { "width": 0.1, "height": 0.06, "depth": 0.01 }
            }),
            &mut state,
        )
        .unwrap();
        cut_hole(
            &json!({
                "part_id": "plate_dir",
                "position": [0.0, 0.0, 0.0],
                "direction": [0.0, 0.0, -1.0],
                "diameter": 0.016,
                "through": true
            }),
            &mut state,
        )
        .expect("cut_hole with direction + diameter + through: true should succeed");
        let plate_part = state.parts.get("plate_dir").unwrap();
        let plate_report = mcp_cad_core::validate_mesh(&plate_part.mesh, Some(1));
        assert!(plate_report.topology.is_watertight);
        assert_eq!(plate_report.topology.genus.round() as i32, 1);
    }

    #[test]
    fn test_p2_t18_joints_preserved_on_csg_transform_and_collider_updated_on_feature_ops() {
        let mut state = EngineState::new().expect("EngineState init");

        // 1. Create rotor, follower, and cutter parts
        create_part(
            &json!({
                "name": "Rotor",
                "part_id": "rotor",
                "material": "steel_structural_aisi1020",
                "is_fixed": false
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "rotor",
                "primitive_type": "box",
                "params": { "width": 100.0, "height": 100.0, "depth": 40.0 }
            }),
            &mut state,
        )
        .unwrap();

        create_part(
            &json!({
                "name": "Follower Shaft",
                "part_id": "follower",
                "material": "aluminum_6061_t6",
                "is_fixed": false
            }),
            &mut state,
        )
        .unwrap();
        let follower_feat = add_feature_primitive(
            &json!({
                "part_id": "follower",
                "primitive_type": "cylinder",
                "params": { "radius": 20.0, "height": 60.0 }
            }),
            &mut state,
        )
        .unwrap();
        let follower_cyl_id = follower_feat["feature_id"].as_str().unwrap().to_string();

        create_part(
            &json!({
                "name": "Cutter Tool",
                "part_id": "cutter",
                "material": "steel_structural_aisi1020",
                "is_fixed": false
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "cutter",
                "primitive_type": "box",
                "params": { "width": 30.0, "height": 30.0, "depth": 30.0, "position": [20.0, 0.0, 0.0] }
            }),
            &mut state,
        )
        .unwrap();

        // 2. Register revolute joint (with motor) and gear joint BEFORE running CSG / transform / optimize
        physics::create_joint(
            &json!({
                "joint_id": "j_rev_rotor",
                "joint_type": "revolute",
                "part_a": "ground",
                "part_b": "rotor",
                "axis": [0.0, 0.0, 1.0]
            }),
            &mut state,
        )
        .unwrap();
        physics::set_joint_motor(
            &json!({
                "joint_id": "j_rev_rotor",
                "target_velocity": 6.0,
                "max_force": 200.0
            }),
            &mut state,
        )
        .unwrap();
        physics::create_joint(
            &json!({
                "joint_id": "j_gear_pair",
                "joint_type": "gear",
                "part_a": "rotor",
                "part_b": "follower",
                "ratio": 0.5
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(state.physics_world.joints().len(), 2);

        let rotor_mass_init = state.physics_world.part_mass("rotor").unwrap();

        // 3. Run boolean_difference, boolean_union, boolean_intersection, transform_part, and optimize_feature_tree
        //    on `rotor` and verify joints and motor remain intact at every step!
        boolean_difference(
            &json!({ "target_part_id": "rotor", "tool_part_id": "cutter" }),
            &mut state,
        )
        .unwrap();
        let rotor_mass_after_diff = state.physics_world.part_mass("rotor").unwrap();
        assert!(
            rotor_mass_after_diff < rotor_mass_init,
            "boolean_difference must reduce rotor mass in physics_world ({rotor_mass_after_diff} vs {rotor_mass_init})"
        );
        assert_eq!(state.physics_world.joints().len(), 2);
        assert!(state.physics_world.joints()["j_rev_rotor"].motor.is_some());

        boolean_union(
            &json!({ "target_part_id": "rotor", "tool_part_id": "cutter" }),
            &mut state,
        )
        .unwrap();
        assert_eq!(state.physics_world.joints().len(), 2);

        transform_part(
            &json!({ "part_id": "rotor", "scale": [2.0, 1.0, 1.0] }),
            &mut state,
        )
        .unwrap();
        let (_, aabb_max_scaled) = state.physics_world.part_collider_aabb("rotor").unwrap();
        assert!(
            aabb_max_scaled[0] > 0.09,
            "transform_part scale [2,1,1] must update rotor collider X max in physics_world, got {}",
            aabb_max_scaled[0]
        );
        assert_eq!(state.physics_world.joints().len(), 2);

        boolean_intersection(
            &json!({ "target_part_id": "rotor", "tool_part_id": "cutter" }),
            &mut state,
        )
        .unwrap();
        assert_eq!(state.physics_world.joints().len(), 2);

        optimize_feature_tree(
            &json!({ "part_id": "rotor", "apply": true }),
            &mut state,
        )
        .unwrap();
        assert_eq!(
            state.physics_world.joints().len(),
            2,
            "optimize_feature_tree must preserve all attached joints"
        );
        assert!(state.physics_world.joints().contains_key("j_rev_rotor"));
        assert!(state.physics_world.joints().contains_key("j_gear_pair"));

        // 4. Verify geometry-modifying feature tools on `follower` (already in physics_world via j_gear_pair)
        //    update its collider AABB and mass in `physics_world` while preserving joints.
        let follower_mass_0 = state.physics_world.part_mass("follower").unwrap();

        // 4a. modify_feature: double radius from 20 mm to 40 mm
        modify_feature(
            &json!({
                "part_id": "follower",
                "feature_id": follower_cyl_id,
                "params": { "radius": 40.0, "height": 60.0 }
            }),
            &mut state,
        )
        .unwrap();
        let follower_mass_1 = state.physics_world.part_mass("follower").unwrap();
        let (f_min_1, f_max_1) = state.physics_world.part_collider_aabb("follower").unwrap();
        assert!(
            follower_mass_1 > follower_mass_0 * 3.5,
            "modify_feature must update follower mass in physics_world ({follower_mass_1} vs {follower_mass_0})"
        );
        assert!((f_max_1[0] - f_min_1[0] - 0.08).abs() < 0.005);

        // 4b. cut_hole: cut central bore R=20 mm through follower -> mass decreases
        cut_hole(
            &json!({
                "part_id": "follower",
                "position": [0.0, 0.0, 0.0],
                "normal": [0.0, 0.0, 1.0],
                "radius": 20.0,
                "through": true
            }),
            &mut state,
        )
        .unwrap();
        // Note: ConvexHull of hollow cylinder has same outer hull; switch follower to fixed (`is_fixed: true`)
        // so it uses exact TriMesh collider whose volume/AABB reflects internal/cut features, or add outer boss:
        add_feature_primitive(
            &json!({
                "part_id": "follower",
                "primitive_type": "box",
                "params": { "width": 120.0, "height": 20.0, "depth": 20.0 }
            }),
            &mut state,
        )
        .unwrap();
        let (f_min_2, f_max_2) = state.physics_world.part_collider_aabb("follower").unwrap();
        assert!(
            (f_max_2[0] - f_min_2[0] - 0.12).abs() < 0.005,
            "add_feature_primitive must update follower collider X span to ~0.12 m, got [{}, {}]",
            f_min_2[0],
            f_max_2[0]
        );

        // 4c. cut_helical_groove and tessellate_part preserve joints and keep collider valid
        cut_helical_groove(
            &json!({
                "part_id": "follower",
                "helix_radius": 40.0,
                "groove_radius": 3.0,
                "pitch": 20.0,
                "height": 40.0,
                "start_z": -20.0,
                "starts": 1
            }),
            &mut state,
        )
        .unwrap();
        tessellate_part(
            &json!({ "part_id": "follower", "quality": "fine" }),
            &mut state,
        )
        .unwrap();

        assert_eq!(state.physics_world.joints().len(), 2);
        assert!(state.physics_world.joints()["j_rev_rotor"].motor.is_some());
    }

    #[test]
    fn test_p2_t19_modify_feature_rollback_and_helical_groove_auto_start_z() {
        let mut state = EngineState::new().expect("EngineState init");
        state.length_unit = LengthUnit::M;

        // 1. Transactional rollback in modify_feature: create a box + revolve + transform part,
        //    tessellate with "fine" so cached_mesh is populated, then attempt invalid modifications
        //    and verify dag, mesh, cached_mesh, and revision remain 100% intact.
        create_part(
            &json!({
                "name": "Rollback Target",
                "part_id": "p_rollback",
                "material": "steel_4140"
            }),
            &mut state,
        )
        .unwrap();

        let box_feat = add_feature_primitive(
            &json!({
                "part_id": "p_rollback",
                "primitive_type": "box",
                "params": { "width": 0.08, "height": 0.06, "depth": 0.04 }
            }),
            &mut state,
        )
        .unwrap();
        let box_feat_id = box_feat["feature_id"].as_str().unwrap().to_string();

        let rev_feat = add_feature_primitive(
            &json!({
                "part_id": "p_rollback",
                "primitive_type": "revolve",
                "params": {
                    "profile": [[0.01, -0.02], [0.02, -0.02], [0.02, 0.02], [0.01, 0.02]],
                    "angle_deg": 360.0
                }
            }),
            &mut state,
        )
        .unwrap();
        let rev_feat_id = rev_feat["feature_id"].as_str().unwrap().to_string();

        let tf_feat = transform_part(
            &json!({
                "part_id": "p_rollback",
                "translation": [0.0, 0.0, 0.01],
                "scale": [1.0, 1.0, 1.0]
            }),
            &mut state,
        )
        .unwrap();
        let tf_feat_id = tf_feat["feature_id"].as_str().unwrap().to_string();

        tessellate_part(
            &json!({ "part_id": "p_rollback", "quality": "fine" }),
            &mut state,
        )
        .unwrap();

        let (orig_vol, orig_tri_count, orig_rev, orig_dag_json) = {
            let p = state.parts.get("p_rollback").unwrap();
            (
                p.volume(),
                p.mesh().triangle_count(),
                p.revision,
                serde_json::to_string(&p.dag.nodes).unwrap(),
            )
        };

        // 1a. Negative box width -> must fail and roll back
        assert!(modify_feature(
            &json!({
                "part_id": "p_rollback",
                "feature_id": box_feat_id,
                "params": { "width": -0.05 }
            }),
            &mut state,
        )
        .is_err());

        // 1b. Empty revolve profile -> must fail and roll back (not silently keep old profile)
        assert!(modify_feature(
            &json!({
                "part_id": "p_rollback",
                "feature_id": rev_feat_id,
                "params": { "profile": [] }
            }),
            &mut state,
        )
        .is_err());

        // 1c. Self-intersecting (bowtie) revolve profile -> must fail in rebuild() and roll back
        assert!(modify_feature(
            &json!({
                "part_id": "p_rollback",
                "feature_id": rev_feat_id,
                "params": {
                    "profile": [[0.01, 0.0], [0.03, 0.02], [0.03, 0.0], [0.01, 0.02]]
                }
            }),
            &mut state,
        )
        .is_err());

        // 1d. Malformed 2-element scale array on Transform feature -> must return Err (no panic!) and roll back
        assert!(modify_feature(
            &json!({
                "part_id": "p_rollback",
                "feature_id": tf_feat_id,
                "params": { "scale": [1.0, 2.0] }
            }),
            &mut state,
        )
        .is_err());

        // 1e. Zero scale component on Transform feature -> must fail in rebuild() and roll back
        assert!(modify_feature(
            &json!({
                "part_id": "p_rollback",
                "feature_id": tf_feat_id,
                "params": { "scale": [1.0, 0.0, 1.0] }
            }),
            &mut state,
        )
        .is_err());

        {
            let p = state.parts.get("p_rollback").unwrap();
            assert!((p.volume() - orig_vol).abs() < 1e-12);
            assert_eq!(p.mesh().triangle_count(), orig_tri_count);
            assert_eq!(p.revision, orig_rev);
            assert_eq!(serde_json::to_string(&p.dag.nodes).unwrap(), orig_dag_json);
        }

        // 2. Auto-alignment of omitted `start_z` to bottom of part (`bounding_box.min[2]`) in `cut_helical_groove`
        //    (Benchmark T08 M10x1.5 hex nut workflow: height = 0.008 m centered at z=0 -> z in [-0.004, +0.004]).
        create_part(
            &json!({
                "name": "M10 Hex Nut",
                "part_id": "t08_nut_auto_z",
                "material": "steel_4140"
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "t08_nut_auto_z",
                "primitive_type": "polygon_prism",
                "params": {
                    "sides": 6,
                    "outer_radius": 0.0098,
                    "height": 0.008,
                    "chamfer_radius": 0.0008
                }
            }),
            &mut state,
        )
        .unwrap();
        cut_hole(
            &json!({
                "part_id": "t08_nut_auto_z",
                "position": [0.0, 0.0, 0.0],
                "normal": [0.0, 0.0, -1.0],
                "radius": 0.00425,
                "through": true
            }),
            &mut state,
        )
        .unwrap();

        let groove_resp = cut_helical_groove(
            &json!({
                "part_id": "t08_nut_auto_z",
                "helix_radius": 0.005,
                "groove_radius": 0.00075,
                "pitch": 0.0015,
                "height": 0.008,
                "starts": 1,
                "profile_type": "metric_60"
            }),
            &mut state,
        )
        .expect("cut_helical_groove without start_z should succeed");

        let auto_start_z = groove_resp["start_z"].as_f64().unwrap();
        assert!(
            (auto_start_z - (-0.004)).abs() < 1e-6,
            "Omitted start_z must default to bottom of part (-0.004 m), got {auto_start_z}"
        );

        let nut_part = state.parts.get("t08_nut_auto_z").unwrap();
        if let FeatureOp::CutHelicalGroove { start_z, .. } = &nut_part.dag.nodes[2].op {
            assert!(
                (*start_z - (-0.004)).abs() < 1e-6,
                "DAG node start_z must be -0.004 m, got {start_z}"
            );
        } else {
            panic!("Expected CutHelicalGroove at DAG node index 2");
        }
        assert!(!nut_part.mesh.vertices.is_empty());
        let nut_aabb = nut_part.bounding_box();
        assert!((nut_aabb.min[2] - (-0.004)).abs() < 1e-4);
        assert!((nut_aabb.max[2] - 0.004).abs() < 1e-4);
    }

    #[test]
    fn test_p2_t21_modify_feature_wheel_rim_scale_warning() {
        let mut state = EngineState::new().expect("EngineState init");
        // Default length_unit is "mm"
        create_part(
            &json!({
                "part_id": "rim_part",
                "name": "Car Rim",
                "material": "aluminum_6061_t6"
            }),
            &mut state,
        )
        .unwrap();

        // 1. Add WheelRim with valid diameter 400 mm (0.4 m >= 0.15 m) -> no WHEEL_RIM_SCALE warning
        let add_res = add_feature_primitive(
            &json!({
                "part_id": "rim_part",
                "primitive_type": "wheel_rim",
                "rim_diameter": 400.0,
                "rim_width": 200.0,
                "hub_diameter": 120.0,
                "spokes": 5
            }),
            &mut state,
        )
        .unwrap();
        let feat_id = add_res["feature_id"].as_str().unwrap().to_string();
        let add_warnings = add_res["warnings"].as_array().unwrap();
        assert!(
            !add_warnings
                .iter()
                .any(|w| w.as_str().is_some_and(|s| s.contains("WHEEL_RIM_SCALE"))),
            "400 mm rim must not trigger WHEEL_RIM_SCALE warning, got {add_warnings:?}"
        );

        // 2. Modify rim_diameter down to 100 mm (0.10 m < 0.15 m) -> must emit WHEEL_RIM_SCALE warning
        let mod_small = modify_feature(
            &json!({
                "part_id": "rim_part",
                "feature_id": feat_id,
                "parameters": {
                    "rim_diameter": 100.0,
                    "rim_width": 50.0,
                    "hub_diameter": 35.0
                }
            }),
            &mut state,
        )
        .unwrap();
        let small_warnings = mod_small["warnings"].as_array().unwrap();
        assert!(
            small_warnings
                .iter()
                .any(|w| w.as_str().is_some_and(|s| s.contains("WHEEL_RIM_SCALE"))),
            "modify_feature reducing rim_diameter to 100 mm (< 150 mm) must emit WHEEL_RIM_SCALE warning, got {small_warnings:?}"
        );

        // 3. Modify rim_diameter back up to 450 mm (0.45 m >= 0.15 m) -> WHEEL_RIM_SCALE warning must clear
        let mod_normal = modify_feature(
            &json!({
                "part_id": "rim_part",
                "feature_id": feat_id,
                "parameters": {
                    "rim_diameter": 450.0,
                    "rim_width": 210.0,
                    "hub_diameter": 130.0
                }
            }),
            &mut state,
        )
        .unwrap();
        let normal_warnings = mod_normal["warnings"].as_array().unwrap();
        assert!(
            !normal_warnings
                .iter()
                .any(|w| w.as_str().is_some_and(|s| s.contains("WHEEL_RIM_SCALE"))),
            "modify_feature restoring rim_diameter to 450 mm must not emit WHEEL_RIM_SCALE warning, got {normal_warnings:?}"
        );
    }

    #[test]
    fn test_p2_t24_export_stl_and_tessellate_part_preserve_fea_freshness() {
        let mut state = EngineState::new().expect("EngineState init");

        // 1. Create a part and add a cylinder feature (default medium tessellation -> revision = 1)
        create_part(
            &json!({
                "part_id": "shaft_p2t24",
                "name": "Drive Shaft",
                "material": "steel_4140"
            }),
            &mut state,
        )
        .unwrap();

        let add_res = add_feature_primitive(
            &json!({
                "part_id": "shaft_p2t24",
                "primitive_type": "cylinder",
                "params": {
                    "radius": 20.0,
                    "height": 100.0
                }
            }),
            &mut state,
        )
        .unwrap();
        let feat_id = add_res["feature_id"].as_str().unwrap().to_string();
        assert_eq!(add_res["revision"], 1);

        // 2. Declare engineering requirements including min_safety_factor
        crate::tools::requirements::set_requirements(
            &json!({
                "part_id": "shaft_p2t24",
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
            }),
            &mut state,
        )
        .unwrap();

        // 3. Run FEA analysis -> records fea_revision = 1
        let fea_res = crate::tools::fea::run_fea_analysis(
            &json!({
                "part_id": "shaft_p2t24",
                "mesh_density": "coarse",
                "fixed": [{"face": "cap_bottom"}],
                "loads": [{"face": "cap_top", "force": [0.0, 0.0, 1000.0]}]
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(fea_res["part_revision"], 1);

        // Verify requirements PASS and get_scene_summary reports last_fea.stale == false
        let check_before = crate::tools::requirements::check_requirements(
            &json!({ "part_id": "shaft_p2t24" }),
            &state,
        )
        .unwrap();
        assert_eq!(check_before["status"], "PASS");
        assert_eq!(check_before["stale"], 0);

        // 4. Call tessellate_part with "coarse", "fine", and custom chord_tolerance
        let tess_coarse = tessellate_part(
            &json!({ "part_id": "shaft_p2t24", "quality": "coarse" }),
            &mut state,
        )
        .unwrap();
        assert_eq!(tess_coarse["revision"], 1);

        let tess_fine = tessellate_part(
            &json!({ "part_id": "shaft_p2t24", "quality": "fine" }),
            &mut state,
        )
        .unwrap();
        assert_eq!(tess_fine["revision"], 1);
        assert!(
            tess_fine["triangles_count"].as_u64().unwrap()
                > tess_coarse["triangles_count"].as_u64().unwrap()
        );

        // 5. Call export_stl (defaults to "fine" quality, then "coarse")
        let tmp_dir = std::env::temp_dir().join(format!("mcpcad_p2t24_{}", Uuid::new_v4().simple()));
        let stl_fine_path = tmp_dir.join("shaft_fine.stl");
        let exp_fine = export_stl(
            &json!({
                "part_id": "shaft_p2t24",
                "filepath": stl_fine_path.to_str().unwrap(),
                "binary": true
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(exp_fine["revision"], 1);
        assert_eq!(exp_fine["quality_applied"], "fine");

        let stl_coarse_path = tmp_dir.join("shaft_coarse.stl");
        let exp_coarse = export_stl(
            &json!({
                "part_id": "shaft_p2t24",
                "filepath": stl_coarse_path.to_str().unwrap(),
                "quality": "coarse",
                "binary": false
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(exp_coarse["revision"], 1);
        assert_eq!(exp_coarse["quality_applied"], "coarse");
        let _ = std::fs::remove_dir_all(&tmp_dir);

        // 6. Verify part.revision is STILL 1, check_requirements is STILL PASS (0 stale),
        //    and get_scene_summary reports last_fea.stale == false ("FRESH")
        assert_eq!(state.parts.get("shaft_p2t24").unwrap().revision, 1);

        let check_after_export = crate::tools::requirements::check_requirements(
            &json!({ "part_id": "shaft_p2t24" }),
            &state,
        )
        .unwrap();
        assert_eq!(
            check_after_export["status"], "PASS",
            "export_stl and tessellate_part must not mark FEA requirements STALE: {check_after_export}"
        );
        assert_eq!(check_after_export["stale"], 0);
        assert_eq!(check_after_export["passed"], 2);

        let summary = crate::tools::metrics::get_scene_summary(&json!({}), &state).unwrap();
        assert_eq!(summary["parts"][0]["revision"], 1);
        assert_eq!(summary["parts"][0]["last_fea"]["stale"], false);
        assert_eq!(summary["parts"][0]["last_fea"]["status"], "FRESH");

        // 7. Modifying actual geometry via modify_feature MUST increment revision to 2 and mark FEA as STALE
        let mod_res = modify_feature(
            &json!({
                "part_id": "shaft_p2t24",
                "feature_id": feat_id,
                "params": { "radius": 25.0, "height": 100.0 }
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(mod_res["revision"], 2);
        assert_eq!(mod_res["requirements_status"]["status"], "STALE");
        assert_eq!(mod_res["requirements_status"]["stale"], 1);

        let summary_after_mod = crate::tools::metrics::get_scene_summary(&json!({}), &state).unwrap();
        assert_eq!(summary_after_mod["parts"][0]["revision"], 2);
        assert_eq!(summary_after_mod["parts"][0]["last_fea"]["stale"], true);
        assert_eq!(summary_after_mod["parts"][0]["last_fea"]["status"], "STALE");
    }

    #[test]
    fn test_p2_t26_boolean_ops_metadata_unit_override_and_warnings() {
        let mut state = EngineState::new().unwrap();

        // Create target part (20 x 20 x 20 mm cube = 8000 mm^3 = 8e-6 m^3)
        create_part(
            &json!({
                "part_id": "target_b",
                "name": "Target",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "target_b",
                "primitive_type": "box",
                "params": {
                    "width": 20.0,
                    "height": 20.0,
                    "depth": 20.0
                }
            }),
            &mut state,
        )
        .unwrap();

        // Create tool part (10 x 10 x 10 mm cube = 1000 mm^3, shifted to [10, 0, 0] mm so it overlaps [5..10] in X -> 5 x 10 x 10 = 500 mm^3 overlap)
        create_part(
            &json!({
                "part_id": "tool_b",
                "name": "Tool",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "tool_b",
                "primitive_type": "box",
                "params": {
                    "width": 10.0,
                    "height": 10.0,
                    "depth": 10.0,
                    "position": [10.0, 0.0, 0.0]
                }
            }),
            &mut state,
        )
        .unwrap();

        // 1. boolean_difference with per-call length_unit = "m"
        let diff_res = boolean_difference(
            &json!({
                "target_part_id": "target_b",
                "tool_part_id": "tool_b",
                "length_unit": "m"
            }),
            &mut state,
        )
        .unwrap();

        assert_eq!(diff_res["part_id"], "target_b");
        assert_eq!(diff_res["target_part_id"], "target_b");
        assert_eq!(diff_res["revision"], 2);
        assert_eq!(diff_res["status"], "boolean_difference_applied");
        assert_eq!(diff_res["length_unit"], "m");
        assert_eq!(diff_res["units"]["volume"], "m³");
        assert!(diff_res["vertices_count"].as_u64().unwrap() > 0);
        assert!(diff_res["triangles_count"].as_u64().unwrap() > 0);
        // 8000 mm^3 - 500 mm^3 = 7500 mm^3 = 7.5e-6 m^3
        let vol_m = diff_res["volume"].as_f64().unwrap();
        let vol_m3 = diff_res["volume_m3"].as_f64().unwrap();
        assert!((vol_m - 7.5e-6).abs() < 1e-7, "Expected ~7.5e-6 m^3, got {vol_m}");
        assert!((vol_m3 - 7.5e-6).abs() < 1e-7);
        let dims_m = diff_res["bounding_box"]["dimensions"].as_array().unwrap();
        assert!((dims_m[0].as_f64().unwrap() - 0.02).abs() < 1e-4);
        assert!(diff_res["warnings"].as_array().unwrap().is_empty());

        // 2. boolean_union in default session unit ("mm") -> restores cut + adds protruding [10..15] mm half (8500 mm^3)
        let union_res = boolean_union(
            &json!({
                "target_part_id": "target_b",
                "tool_part_id": "tool_b"
            }),
            &mut state,
        )
        .unwrap();

        assert_eq!(union_res["part_id"], "target_b");
        assert_eq!(union_res["target_part_id"], "target_b");
        assert_eq!(union_res["revision"], 3);
        assert_eq!(union_res["status"], "union_applied");
        assert_eq!(union_res["length_unit"], "mm");
        assert_eq!(union_res["units"]["volume"], "mm³");
        let vol_mm3 = union_res["volume"].as_f64().unwrap();
        assert!((vol_mm3 - 8500.0).abs() < 10.0, "Expected ~8500 mm^3, got {vol_mm3}");
        let dims_mm = union_res["bounding_box"]["dimensions"].as_array().unwrap();
        assert!((dims_mm[0].as_f64().unwrap() - 25.0).abs() < 0.1);
        assert!(union_res["warnings"].as_array().unwrap().is_empty());

        // 3. boolean_intersection with tool_b -> leaves only tool_b volume (1000 mm^3)
        let inter_res = boolean_intersection(
            &json!({
                "target_part_id": "target_b",
                "tool_part_id": "tool_b"
            }),
            &mut state,
        )
        .unwrap();

        assert_eq!(inter_res["part_id"], "target_b");
        assert_eq!(inter_res["target_part_id"], "target_b");
        assert_eq!(inter_res["revision"], 4);
        assert_eq!(inter_res["status"], "intersection_applied");
        assert_eq!(inter_res["length_unit"], "mm");
        let inter_vol_mm3 = inter_res["volume"].as_f64().unwrap();
        assert!((inter_vol_mm3 - 1000.0).abs() < 5.0, "Expected ~1000 mm^3, got {inter_vol_mm3}");
        let inter_dims = inter_res["bounding_box"]["dimensions"].as_array().unwrap();
        assert!((inter_dims[0].as_f64().unwrap() - 10.0).abs() < 0.1);
        assert!(inter_res["warnings"].is_array());
    }

    #[test]
    fn test_p2_t27_optimize_feature_tree_revision_and_length_unit_override() {
        let mut state = EngineState::new().expect("EngineState init");

        create_part(
            &json!({
                "part_id": "bushing_p2t27",
                "name": "Coaxial Bushing",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();

        add_feature_primitive(
            &json!({
                "part_id": "bushing_p2t27",
                "primitive_type": "cylinder",
                "params": { "radius": 30.0, "height": 50.0, "segments": 32 }
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(state.parts["bushing_p2t27"].revision, 1);

        // 1. Calling optimize_feature_tree on single cylinder (no rules apply) must keep revision == 1
        let noop_res = optimize_feature_tree(
            &json!({
                "part_id": "bushing_p2t27",
                "apply": true,
                "length_unit": "m"
            }),
            &mut state,
        )
        .unwrap();
        assert!(noop_res["applied_rules"].as_array().unwrap().is_empty());
        assert_eq!(noop_res["revision"], 1);
        assert_eq!(noop_res["length_unit"], "m");
        assert_eq!(noop_res["units"]["volume"], "m³");
        assert!((noop_res["volume"].as_f64().unwrap() - noop_res["volume_m3"].as_f64().unwrap()).abs() < 1e-12);
        assert_eq!(state.parts["bushing_p2t27"].revision, 1);

        // 2. Add coaxial through hole -> revision becomes 2
        cut_hole(
            &json!({
                "part_id": "bushing_p2t27",
                "position": [0.0, 0.0, 0.0],
                "normal": [0.0, 0.0, 1.0],
                "radius": 15.0,
                "depth": 60.0
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(state.parts["bushing_p2t27"].revision, 2);

        // 3. Calling optimize_feature_tree(apply: true) applies R4_COAXIAL_REVOLVE_REDUCTION -> revision bumps to 3
        let opt_res = optimize_feature_tree(
            &json!({
                "part_id": "bushing_p2t27",
                "apply": true,
                "length_unit": "mm"
            }),
            &mut state,
        )
        .unwrap();
        assert!(!opt_res["applied_rules"].as_array().unwrap().is_empty());
        assert_eq!(opt_res["revision"], 3);
        assert_eq!(opt_res["length_unit"], "mm");
        assert_eq!(opt_res["units"]["volume"], "mm³");
        assert!(opt_res["volume"].as_f64().unwrap() > 100_000.0);
        assert!(opt_res["equivalence_verification"]["volume_after"].as_f64().unwrap() > 100_000.0);
        assert_eq!(state.parts["bushing_p2t27"].revision, 3);

        // 4. Calling optimize_feature_tree(apply: true) a second time on already-optimized DAG keeps revision == 3
        let opt_again = optimize_feature_tree(
            &json!({
                "part_id": "bushing_p2t27",
                "apply": true
            }),
            &mut state,
        )
        .unwrap();
        assert!(opt_again["applied_rules"].as_array().unwrap().is_empty());
        assert_eq!(opt_again["revision"], 3);
        assert_eq!(state.parts["bushing_p2t27"].revision, 3);
    }

    #[test]
    fn test_p2_t29_list_parts_respects_length_unit_override_and_rejects_invalid_unit() {
        let mut state = EngineState::new().expect("EngineState init");

        create_part(
            &json!({
                "part_id": "block_p2t29",
                "name": "Precision Block",
                "material": "aluminum_6061_t6"
            }),
            &mut state,
        )
        .unwrap();

        add_feature_primitive(
            &json!({
                "part_id": "block_p2t29",
                "primitive_type": "box",
                "params": { "width": 100.0, "height": 40.0, "depth": 20.0 }
            }),
            &mut state,
        )
        .unwrap();

        // 1. Default session unit ("mm")
        let res_mm = list_parts(&json!({}), &state).unwrap();
        assert_eq!(res_mm["length_unit"], "mm");
        assert_eq!(res_mm["units"]["length"], "mm");
        assert_eq!(res_mm["units"]["volume"], "mm³");
        assert_eq!(res_mm["parts_count"], 1);
        let p_mm = &res_mm["parts"][0];
        assert_eq!(p_mm["part_id"], "block_p2t29");
        assert_eq!(p_mm["length_unit"], "mm");
        assert!((p_mm["bounding_box_dimensions"][0].as_f64().unwrap() - 100.0).abs() < 1e-6);
        assert!((p_mm["bounding_box"]["min"][0].as_f64().unwrap() - (-50.0)).abs() < 1e-6);
        assert!((p_mm["bounding_box"]["max"][0].as_f64().unwrap() - 50.0).abs() < 1e-6);
        assert!((p_mm["volume"].as_f64().unwrap() - 80_000.0).abs() < 1e-3);
        assert!((p_mm["volume_m3"].as_f64().unwrap() - 8.0e-5).abs() < 1e-10);

        // 2. Per-call override length_unit = "m" without mutating session length_unit
        let res_m = list_parts(&json!({ "length_unit": "m" }), &state).unwrap();
        assert_eq!(res_m["length_unit"], "m");
        assert_eq!(res_m["units"]["length"], "m");
        assert_eq!(res_m["units"]["volume"], "m³");
        assert_eq!(res_m["parts_count"], 1);
        let p_m = &res_m["parts"][0];
        assert_eq!(p_m["part_id"], "block_p2t29");
        assert_eq!(p_m["length_unit"], "m");
        assert!((p_m["bounding_box_dimensions"][0].as_f64().unwrap() - 0.1).abs() < 1e-8);
        assert!((p_m["bounding_box"]["min"][0].as_f64().unwrap() - (-0.05)).abs() < 1e-8);
        assert!((p_m["bounding_box"]["max"][0].as_f64().unwrap() - 0.05).abs() < 1e-8);
        assert!((p_m["volume"].as_f64().unwrap() - 8.0e-5).abs() < 1e-10);
        assert!((p_m["volume_m3"].as_f64().unwrap() - 8.0e-5).abs() < 1e-10);
        assert_eq!(state.length_unit, LengthUnit::Mm);

        // 3. Invalid length_unit override must return InvalidArgument
        let err = list_parts(&json!({ "length_unit": "inch" }), &state).unwrap_err();
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("length_unit"));
    }

    #[test]
    fn test_p2_t30_import_stl_parameters_physics_registration_and_metadata() {
        let mut state = EngineState::new().expect("EngineState init");

        // 1. Create a box part and export it to STL
        create_part(
            &json!({
                "part_id": "source_box",
                "name": "Source Box",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();

        add_feature_primitive(
            &json!({
                "part_id": "source_box",
                "primitive_type": "box",
                "params": { "width": 50.0, "height": 30.0, "depth": 20.0 }
            }),
            &mut state,
        )
        .unwrap();

        let tmp_dir = std::env::temp_dir().join(format!("mcpcad_p2t30_{}", Uuid::new_v4().simple()));
        let stl_path = tmp_dir.join("box.stl");
        export_stl(
            &json!({
                "part_id": "source_box",
                "file_path": stl_path.to_str().unwrap(),
                "binary": true
            }),
            &mut state,
        )
        .unwrap();
        assert!(stl_path.exists());

        // 2. Import with alias `file_path`, explicit `part_id`, `is_fixed: true`, and custom name
        let import_res = import_stl(
            &json!({
                "file_path": stl_path.to_str().unwrap(),
                "part_id": "imported_fixed_box",
                "name": "Imported Fixed Box",
                "is_fixed": true,
                "material": "aluminum_6061_t6"
            }),
            &mut state,
        )
        .unwrap();

        assert_eq!(import_res["part_id"], "imported_fixed_box");
        assert_eq!(import_res["name"], "Imported Fixed Box");
        assert_eq!(import_res["is_fixed"], true);
        assert_eq!(import_res["status"], "stl_imported");
        assert_eq!(import_res["revision"], 1);
        assert_eq!(import_res["triangles_count"], 12);
        assert_eq!(import_res["vertices_count"], 36);
        assert_eq!(import_res["length_unit"], "mm");
        assert_eq!(import_res["units"]["length"], "mm");
        assert_eq!(import_res["units"]["volume"], "mm³");

        // Bounding box dimensions [50, 30, 20] in mm
        let dims = import_res["bounding_box"]["dimensions"].as_array().unwrap();
        assert!((dims[0].as_f64().unwrap() - 50.0).abs() < 1e-4);
        assert!((dims[1].as_f64().unwrap() - 30.0).abs() < 1e-4);
        assert!((dims[2].as_f64().unwrap() - 20.0).abs() < 1e-4);

        // Volume: 50 * 30 * 20 = 30000 mm³ = 3e-5 m³
        let vol_mm = import_res["volume"].as_f64().unwrap();
        let vol_m3 = import_res["volume_m3"].as_f64().unwrap();
        assert!((vol_mm - 30_000.0).abs() < 1.0);
        assert!((vol_m3 - 3.0e-5).abs() < 1e-9);

        // Mass: Al6061 ~ 2700 kg/m3 * 3e-5 m3 ~ 0.081 kg
        let mass_kg = import_res["mass_kg"].as_f64().unwrap();
        assert!((mass_kg - 0.081).abs() < 1e-3);

        // 3. Verify registration in state.physics_world as fixed body
        assert!(state.physics_world.has_part("imported_fixed_box"));
        assert_eq!(
            state.physics_world.is_part_dynamic("imported_fixed_box").unwrap(),
            false
        );

        // 4. Import another part with `is_fixed: false` (dynamic)
        let dynamic_res = import_stl(
            &json!({
                "filepath": stl_path.to_str().unwrap(),
                "part_id": "imported_dyn_box",
                "is_fixed": false,
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(dynamic_res["is_fixed"], false);
        assert!(state.physics_world.has_part("imported_dyn_box"));
        assert_eq!(
            state.physics_world.is_part_dynamic("imported_dyn_box").unwrap(),
            true
        );

        // 5. Duplicate part_id must fail with PART_ALREADY_EXISTS
        let dup_err = import_stl(
            &json!({
                "file_path": stl_path.to_str().unwrap(),
                "part_id": "imported_fixed_box",
            }),
            &mut state,
        )
        .unwrap_err();
        let dup_body = dup_err.to_error_body();
        assert_eq!(dup_body.code, "PART_ALREADY_EXISTS");
        assert_eq!(dup_body.param.as_deref(), Some("part_id"));

        // 6. Reserved ID "ground" or "world" must fail with INVALID_ARGUMENT
        let reserved_err = import_stl(
            &json!({
                "file_path": stl_path.to_str().unwrap(),
                "part_id": "Ground",
            }),
            &mut state,
        )
        .unwrap_err();
        let res_body = reserved_err.to_error_body();
        assert_eq!(res_body.code, "INVALID_ARGUMENT");
        assert_eq!(res_body.param.as_deref(), Some("part_id"));

        let _ = std::fs::remove_dir_all(tmp_dir);
    }

    #[test]
    fn test_p2_t32_get_feature_tree_and_validate_mesh_metadata_and_length_unit() {
        let mut state = EngineState::new().expect("EngineState init");

        create_part(
            &json!({
                "part_id": "test_p2t32_part",
                "name": "Metadata Test Part",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();

        add_feature_primitive(
            &json!({
                "part_id": "test_p2t32_part",
                "primitive_type": "box",
                "params": { "width": 40.0, "height": 20.0, "depth": 10.0 }
            }),
            &mut state,
        )
        .unwrap();

        // 1. get_feature_tree in mm (default)
        let tree_mm = get_feature_tree(
            &json!({ "part_id": "test_p2t32_part" }),
            &state,
        )
        .unwrap();
        assert_eq!(tree_mm["part_id"], "test_p2t32_part");
        assert_eq!(tree_mm["revision"], 1);
        assert_eq!(tree_mm["features_count"], 1);
        assert_eq!(tree_mm["length_unit"], "mm");
        assert_eq!(tree_mm["dag"].as_array().unwrap().len(), 1);

        // 2. get_feature_tree with length_unit override: "m"
        let tree_m = get_feature_tree(
            &json!({ "part_id": "test_p2t32_part", "length_unit": "m" }),
            &state,
        )
        .unwrap();
        assert_eq!(tree_m["part_id"], "test_p2t32_part");
        assert_eq!(tree_m["revision"], 1);
        assert_eq!(tree_m["features_count"], 1);
        assert_eq!(tree_m["length_unit"], "m");

        // 3. validate_mesh_tool in mm (default)
        let val_mm = validate_mesh_tool(
            &json!({ "part_id": "test_p2t32_part" }),
            &mut state,
        )
        .unwrap();
        assert_eq!(val_mm["part_id"], "test_p2t32_part");
        assert_eq!(val_mm["revision"], 1);
        assert_eq!(val_mm["length_unit"], "mm");
        assert_eq!(val_mm["status"], "PASS");
        assert!((val_mm["volume"].as_f64().unwrap() - 8000.0).abs() < 1e-3);
        assert!((val_mm["volume_m3"].as_f64().unwrap() - 8.0e-6).abs() < 1e-9);

        // 4. validate_mesh_tool with length_unit override: "m"
        let val_m = validate_mesh_tool(
            &json!({ "part_id": "test_p2t32_part", "length_unit": "m" }),
            &mut state,
        )
        .unwrap();
        assert_eq!(val_m["part_id"], "test_p2t32_part");
        assert_eq!(val_m["revision"], 1);
        assert_eq!(val_m["length_unit"], "m");
        assert!((val_m["volume"].as_f64().unwrap() - 8.0e-6).abs() < 1e-9);
        assert!((val_m["volume_m3"].as_f64().unwrap() - 8.0e-6).abs() < 1e-9);

        // 5. Invalid length_unit returns error
        assert!(get_feature_tree(
            &json!({ "part_id": "test_p2t32_part", "length_unit": "invalid" }),
            &state
        )
        .is_err());
        assert!(validate_mesh_tool(
            &json!({ "part_id": "test_p2t32_part", "length_unit": "invalid" }),
            &mut state
        )
        .is_err());
    }

    #[test]
    fn test_p2_t33_transform_imported_stl_part_and_empty_dag_fallback() {
        let mut state = EngineState::new().expect("EngineState init");

        // 1. Create a reference box part (40x20x10 mm) and export it to a temporary binary STL
        create_part(
            &json!({
                "part_id": "src_box",
                "name": "Source Box",
                "material": "aluminum_6061_t6"
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "src_box",
                "primitive_type": "box",
                "params": { "width": 40.0, "height": 20.0, "depth": 10.0 }
            }),
            &mut state,
        )
        .unwrap();

        let temp_dir = std::env::temp_dir().join(format!("mcpcad_p2t33_{}", Uuid::new_v4()));
        let stl_path = temp_dir.join("p2t33_box.stl");
        let stl_path_str = stl_path.to_str().unwrap();

        export_stl(
            &json!({
                "part_id": "src_box",
                "filepath": stl_path_str,
                "binary": true
            }),
            &mut state,
        )
        .unwrap();

        // 2. Import the STL as a new part `imp_box`
        let imp_resp = import_stl(
            &json!({
                "filepath": stl_path_str,
                "part_id": "imp_box",
                "name": "Imported Box"
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(imp_resp["part_id"], "imp_box");
        assert_eq!(imp_resp["revision"], 1);
        assert!((imp_resp["volume"].as_f64().unwrap() - 8000.0).abs() < 1e-2);

        // 3. Apply transform_part on the imported STL part (translation + scale)
        let tr_resp = transform_part(
            &json!({
                "part_id": "imp_box",
                "translation": [100.0, -50.0, 25.0],
                "scale": [2.0, 2.0, 2.0]
            }),
            &mut state,
        )
        .expect("transform_part on imported STL part must succeed without 'Cannot transform empty solid'");
        assert_eq!(tr_resp["part_id"], "imp_box");
        assert_eq!(tr_resp["status"], "transformed");
        assert_eq!(tr_resp["revision"], 2);
        // Scaled by 2x along all 3 axes -> volume increases by 8x (8000 * 8 = 64000 mm^3)
        assert!((tr_resp["volume"].as_f64().unwrap() - 64000.0).abs() < 1e-1);
        let dims = tr_resp["bounding_box"]["dimensions"].as_array().unwrap();
        assert!((dims[0].as_f64().unwrap() - 80.0).abs() < 1e-3);
        assert!((dims[1].as_f64().unwrap() - 40.0).abs() < 1e-3);
        assert!((dims[2].as_f64().unwrap() - 20.0).abs() < 1e-3);
        let min_pt = tr_resp["bounding_box"]["min"].as_array().unwrap();
        let max_pt = tr_resp["bounding_box"]["max"].as_array().unwrap();
        let cx = (min_pt[0].as_f64().unwrap() + max_pt[0].as_f64().unwrap()) * 0.5;
        let cy = (min_pt[1].as_f64().unwrap() + max_pt[1].as_f64().unwrap()) * 0.5;
        let cz = (min_pt[2].as_f64().unwrap() + max_pt[2].as_f64().unwrap()) * 0.5;
        assert!((cx - 100.0).abs() < 1e-3);
        assert!((cy - (-50.0)).abs() < 1e-3);
        assert!((cz - 25.0).abs() < 1e-3);

        // 4. Also verify legacy/manual Part with empty DAG (`dag.nodes.is_empty()`) and non-empty mesh
        if let Some(p) = state.parts.get_mut("imp_box") {
            p.dag.nodes.clear();
            assert!(p.dag.nodes.is_empty());
        }
        let tr2_resp = transform_part(
            &json!({
                "part_id": "imp_box",
                "translation": [10.0, 20.0, 5.0]
            }),
            &mut state,
        )
        .expect("transform_part on part with cleared DAG but non-empty mesh must auto-seed ImportedMesh");
        assert_eq!(tr2_resp["revision"], 3);
        let tree_resp = get_feature_tree(&json!({ "part_id": "imp_box" }), &state).unwrap();
        assert_eq!(tree_resp["features_count"], 2);

        // 5. Verify truly empty part (`dag.nodes.is_empty()` and `mesh.vertices.is_empty()`) still returns error
        create_part(
            &json!({
                "part_id": "empty_part",
                "name": "Empty Part",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        let empty_err = transform_part(
            &json!({
                "part_id": "empty_part",
                "translation": [10.0, 0.0, 0.0]
            }),
            &mut state,
        )
        .unwrap_err();
        assert!(
            empty_err.to_string().contains("Cannot transform empty solid"),
            "Expected 'Cannot transform empty solid' for truly empty part, got: {empty_err}"
        );

        let _ = std::fs::remove_dir_all(temp_dir);
    }

    #[test]
    fn test_p2_t34_get_feature_tree_scales_mesh_operands_and_face_metrics() {
        let mut state = EngineState::new().expect("EngineState init");

        // 1. Create a base cylinder (r=20 mm, h=50 mm) and a tool cylinder (r=8 mm, h=60 mm)
        create_part(
            &json!({
                "part_id": "base_cyl",
                "name": "Base Cylinder",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "base_cyl",
                "primitive_type": "cylinder",
                "params": { "radius": 20.0, "height": 50.0, "segments": 24 }
            }),
            &mut state,
        )
        .unwrap();

        create_part(
            &json!({
                "part_id": "tool_cyl",
                "name": "Tool Cylinder",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "tool_cyl",
                "primitive_type": "cylinder",
                "params": { "radius": 8.0, "height": 60.0, "segments": 24 }
            }),
            &mut state,
        )
        .unwrap();

        // 2. Apply boolean_difference, boolean_union, boolean_intersection on base_cyl
        boolean_difference(
            &json!({
                "target_part_id": "base_cyl",
                "tool_part_id": "tool_cyl"
            }),
            &mut state,
        )
        .unwrap();

        create_part(
            &json!({
                "part_id": "boss_box",
                "name": "Boss Box",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "boss_box",
                "primitive_type": "box",
                "params": { "width": 50.0, "height": 10.0, "depth": 10.0 }
            }),
            &mut state,
        )
        .unwrap();
        boolean_union(
            &json!({
                "target_part_id": "base_cyl",
                "tool_part_id": "boss_box"
            }),
            &mut state,
        )
        .unwrap();

        create_part(
            &json!({
                "part_id": "clip_box",
                "name": "Clip Box",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        add_feature_primitive(
            &json!({
                "part_id": "clip_box",
                "primitive_type": "box",
                "params": { "width": 60.0, "height": 60.0, "depth": 60.0 }
            }),
            &mut state,
        )
        .unwrap();
        boolean_intersection(
            &json!({
                "target_part_id": "base_cyl",
                "tool_part_id": "clip_box"
            }),
            &mut state,
        )
        .unwrap();

        // 3. Query get_feature_tree in "mm" vs "m"
        let tree_mm = get_feature_tree(
            &json!({ "part_id": "base_cyl", "length_unit": "mm" }),
            &state,
        )
        .unwrap();
        let tree_m = get_feature_tree(
            &json!({ "part_id": "base_cyl", "length_unit": "m" }),
            &state,
        )
        .unwrap();

        let dag_mm = tree_mm["dag"].as_array().unwrap();
        let dag_m = tree_m["dag"].as_array().unwrap();
        assert_eq!(dag_mm.len(), 4);
        assert_eq!(dag_m.len(), 4);

        // Node 1: BooleanDifferenceMesh (tool_cyl: r=8 mm / 0.008 m, h=60 mm / 0.06 m)
        let diff_mesh_mm = &dag_mm[1]["op"]["BooleanDifferenceMesh"]["tool_mesh"];
        let diff_mesh_m = &dag_m[1]["op"]["BooleanDifferenceMesh"]["tool_mesh"];
        let max_z_mm = diff_mesh_mm["vertices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["position"][2].as_f64().unwrap())
            .fold(f64::NEG_INFINITY, f64::max);
        let max_z_m = diff_mesh_m["vertices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["position"][2].as_f64().unwrap())
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((max_z_mm - 30.0).abs() < 1e-6);
        assert!((max_z_m - 0.03).abs() < 1e-9);

        // Check face metrics scaling on tool_mesh (area scales by 1e6, centroid/sample_point/radius scale by 1e3)
        let faces_mm = diff_mesh_mm["faces"].as_array().unwrap();
        let faces_m = diff_mesh_m["faces"].as_array().unwrap();
        assert!(!faces_mm.is_empty());
        assert_eq!(faces_mm.len(), faces_m.len());
        for (f_mm, f_m) in faces_mm.iter().zip(faces_m.iter()) {
            let area_mm = f_mm["area"].as_f64().unwrap();
            let area_m = f_m["area"].as_f64().unwrap();
            assert!((area_mm - area_m * 1e6).abs() < 1e-4);

            let c_mm = f_mm["centroid"].as_array().unwrap();
            let c_m = f_m["centroid"].as_array().unwrap();
            for axis in 0..3 {
                assert!(
                    (c_mm[axis].as_f64().unwrap() - c_m[axis].as_f64().unwrap() * 1000.0).abs()
                        < 1e-6
                );
            }

            let s_mm = f_mm["sample_point"].as_array().unwrap();
            let s_m = f_m["sample_point"].as_array().unwrap();
            for axis in 0..3 {
                assert!(
                    (s_mm[axis].as_f64().unwrap() - s_m[axis].as_f64().unwrap() * 1000.0).abs()
                        < 1e-6
                );
            }

            if let (Some(r_mm), Some(r_m)) = (f_mm["radius"].as_f64(), f_m["radius"].as_f64()) {
                assert!((r_mm - r_m * 1000.0).abs() < 1e-6);
            }
        }

        // Node 2: BooleanUnionMesh (boss_box: width=50 mm -> max_x = 25 mm vs 0.025 m)
        let union_max_x_mm = dag_mm[2]["op"]["BooleanUnionMesh"]["tool_mesh"]["vertices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["position"][0].as_f64().unwrap())
            .fold(f64::NEG_INFINITY, f64::max);
        let union_max_x_m = dag_m[2]["op"]["BooleanUnionMesh"]["tool_mesh"]["vertices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["position"][0].as_f64().unwrap())
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((union_max_x_mm - 25.0).abs() < 1e-6);
        assert!((union_max_x_m - 0.025).abs() < 1e-9);

        // Node 3: BooleanIntersectionMesh (clip_box: width=60 mm -> max_x = 30 mm vs 0.03 m)
        let inter_max_x_mm = dag_mm[3]["op"]["BooleanIntersectionMesh"]["tool_mesh"]["vertices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["position"][0].as_f64().unwrap())
            .fold(f64::NEG_INFINITY, f64::max);
        let inter_max_x_m = dag_m[3]["op"]["BooleanIntersectionMesh"]["tool_mesh"]["vertices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["position"][0].as_f64().unwrap())
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((inter_max_x_mm - 30.0).abs() < 1e-6);
        assert!((inter_max_x_m - 0.03).abs() < 1e-9);

        // 4. Also verify ImportedMesh scaling on an imported STL part
        let temp_dir = std::env::temp_dir().join(format!("mcpcad_p2t34_{}", Uuid::new_v4()));
        let stl_path = temp_dir.join("p2t34_cyl.stl");
        let stl_path_str = stl_path.to_str().unwrap();

        export_stl(
            &json!({
                "part_id": "tool_cyl",
                "filepath": stl_path_str,
                "binary": true
            }),
            &mut state,
        )
        .unwrap();

        import_stl(
            &json!({
                "filepath": stl_path_str,
                "part_id": "imp_cyl",
                "name": "Imported Cylinder"
            }),
            &mut state,
        )
        .unwrap();

        let imp_tree_mm = get_feature_tree(
            &json!({ "part_id": "imp_cyl", "length_unit": "mm" }),
            &state,
        )
        .unwrap();
        let imp_tree_m = get_feature_tree(
            &json!({ "part_id": "imp_cyl", "length_unit": "m" }),
            &state,
        )
        .unwrap();
        let imp_mesh_mm = &imp_tree_mm["dag"][0]["op"]["ImportedMesh"]["mesh"];
        let imp_mesh_m = &imp_tree_m["dag"][0]["op"]["ImportedMesh"]["mesh"];
        let imp_max_z_mm = imp_mesh_mm["vertices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["position"][2].as_f64().unwrap())
            .fold(f64::NEG_INFINITY, f64::max);
        let imp_max_z_m = imp_mesh_m["vertices"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["position"][2].as_f64().unwrap())
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((imp_max_z_mm - 30.0).abs() < 1e-4);
        assert!((imp_max_z_m - 0.03).abs() < 1e-7);

        let _ = std::fs::remove_dir_all(temp_dir);
    }

    #[test]
    fn test_p2_t35_feature_mutation_tools_return_part_id_and_geometric_metadata() {
        let mut state = EngineState::new().expect("EngineState init");

        create_part(
            &json!({
                "part_id": "shaft_p2t35",
                "name": "Shaft P2T35",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();

        // 1. add_feature_primitive: returns part_id, feature_id, primitive_type, revision, bounding_box, volume, volume_m3
        let prim_resp = add_feature_primitive(
            &json!({
                "part_id": "shaft_p2t35",
                "primitive_type": "cylinder",
                "params": { "radius": 20.0, "height": 80.0, "segments": 32 }
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(prim_resp["part_id"], "shaft_p2t35");
        assert_eq!(prim_resp["primitive_type"], "cylinder");
        assert_eq!(prim_resp["revision"], 1);
        let feat_id = prim_resp["feature_id"].as_str().unwrap().to_string();
        assert!(!feat_id.is_empty());
        assert!((prim_resp["bounding_box"]["dimensions"][2].as_f64().unwrap() - 80.0).abs() < 1e-6);
        assert!(prim_resp["volume"].as_f64().unwrap() > 0.0);
        assert!(prim_resp["volume_m3"].as_f64().unwrap() > 0.0);

        // 2. modify_feature: returns part_id, feature_id, status, revision, bounding_box, volume, volume_m3
        let mod_resp = modify_feature(
            &json!({
                "part_id": "shaft_p2t35",
                "feature_id": feat_id,
                "params": { "height": 100.0 }
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(mod_resp["part_id"], "shaft_p2t35");
        assert_eq!(mod_resp["feature_id"], feat_id);
        assert_eq!(mod_resp["status"], "updated");
        assert_eq!(mod_resp["revision"], 2);
        assert!((mod_resp["dimensions"][2].as_f64().unwrap() - 100.0).abs() < 1e-6);
        assert!((mod_resp["bounding_box"]["dimensions"][2].as_f64().unwrap() - 100.0).abs() < 1e-6);
        assert!(mod_resp["volume"].as_f64().unwrap() > prim_resp["volume"].as_f64().unwrap());

        // 3. cut_hole (with through: true): returns part_id, feature_id, revision, depth (in length_unit), depth_m, through, bounding_box, volume, volume_m3
        let hole_resp = cut_hole(
            &json!({
                "part_id": "shaft_p2t35",
                "position": [0.0, 0.0, 50.0],
                "normal": [0.0, 0.0, -1.0],
                "radius": 6.0,
                "through": true
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(hole_resp["part_id"], "shaft_p2t35");
        assert!(hole_resp["feature_id"].as_str().is_some());
        assert_eq!(hole_resp["revision"], 3);
        assert_eq!(hole_resp["through"], true);
        assert!((hole_resp["depth"].as_f64().unwrap() - 100.0).abs() < 1e-6);
        assert!((hole_resp["depth_m"].as_f64().unwrap() - 0.1).abs() < 1e-9);
        assert!(hole_resp["volume"].as_f64().unwrap() < mod_resp["volume"].as_f64().unwrap());

        // Also verify cut_hole with length_unit: "m" scales returned `depth` in meters
        let hole_m_resp = cut_hole(
            &json!({
                "part_id": "shaft_p2t35",
                "position": [0.012, 0.0, 0.05],
                "normal": [0.0, 0.0, -1.0],
                "radius": 0.002,
                "depth": 0.025,
                "length_unit": "m"
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(hole_m_resp["part_id"], "shaft_p2t35");
        assert_eq!(hole_m_resp["through"], false);
        assert_eq!(hole_m_resp["length_unit"], "m");
        assert!((hole_m_resp["depth"].as_f64().unwrap() - 0.025).abs() < 1e-9);
        assert!((hole_m_resp["depth_m"].as_f64().unwrap() - 0.025).abs() < 1e-9);

        // 4. cut_helical_groove: returns part_id, feature_id, revision, start_z, bounding_box, volume, volume_m3
        let groove_resp = cut_helical_groove(
            &json!({
                "part_id": "shaft_p2t35",
                "helix_radius": 20.0,
                "groove_radius": 1.2,
                "pitch": 10.0,
                "height": 40.0
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(groove_resp["part_id"], "shaft_p2t35");
        assert!(groove_resp["feature_id"].as_str().is_some());
        assert_eq!(groove_resp["revision"], 5);
        assert_eq!(groove_resp["status"], "helical_groove_cut");
        assert!((groove_resp["start_z"].as_f64().unwrap() - (-50.0)).abs() < 1e-6);
        assert!(groove_resp["bounding_box"]["dimensions"].is_array());
        assert!(groove_resp["volume"].as_f64().unwrap() > 0.0);
        assert!(groove_resp["volume_m3"].as_f64().unwrap() > 0.0);
    }

    #[test]
    fn test_p2_t36_optimize_feature_tree_dag_scaling_and_delete_part_id() {
        let mut state = EngineState::new().unwrap();
        create_part(
            &json!({
                "part_id": "bushing_p2t36",
                "name": "Bushing P2-T36",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();

        add_feature_primitive(
            &json!({
                "part_id": "bushing_p2t36",
                "primitive_type": "cylinder",
                "params": { "radius": 28.0, "height": 60.0 }
            }),
            &mut state,
        )
        .unwrap();

        cut_hole(
            &json!({
                "part_id": "bushing_p2t36",
                "position": [0.0, 0.0, 30.0],
                "normal": [0.0, 0.0, -1.0],
                "radius": 14.0,
                "depth": 60.0
            }),
            &mut state,
        )
        .unwrap();

        // 1. Explain-only in default "mm": verify logical_plan.dag, physical_plan.dag, and candidate_plans[i].dag are scaled in mm
        let opt_mm = optimize_feature_tree(
            &json!({
                "part_id": "bushing_p2t36",
                "apply": false
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(opt_mm["length_unit"], "mm");

        // logical_plan.dag[0] is AddCylinder with radius=28.0 mm, height=60.0 mm
        let log_cyl = &opt_mm["logical_plan"]["dag"][0]["op"]["AddCylinder"];
        assert!((log_cyl["radius"].as_f64().unwrap() - 28.0).abs() < 1e-6);
        assert!((log_cyl["height"].as_f64().unwrap() - 60.0).abs() < 1e-6);

        // logical_plan.dag[1] is CutHole with radius=14.0 mm, depth=60.0 mm, center[2]=30.0 mm
        let log_hole = &opt_mm["logical_plan"]["dag"][1]["op"]["CutHole"];
        assert!((log_hole["radius"].as_f64().unwrap() - 14.0).abs() < 1e-6);
        assert!((log_hole["depth"].as_f64().unwrap() - 60.0).abs() < 1e-6);
        assert!((log_hole["center"][2].as_f64().unwrap() - 30.0).abs() < 1e-6);

        // physical_plan.dag[0] is AddRevolve (R4 Coaxial Revolve Reduction) with profile points in mm (14.0..28.0, -30.0..30.0)
        let phys_rev = &opt_mm["physical_plan"]["dag"][0]["op"]["AddRevolve"];
        let prof_pt0 = &phys_rev["profile"][0];
        assert!((prof_pt0[0].as_f64().unwrap() - 14.0).abs() < 1e-6);
        assert!((prof_pt0[1].as_f64().unwrap() - (-30.0)).abs() < 1e-6);

        // candidate_plans[0].dag[0] (naive plan) and candidate_plans[last].dag[0] (CBO plan) must also be scaled in mm
        let cand0_cyl = &opt_mm["candidate_plans"][0]["dag"][0]["op"]["AddCylinder"];
        assert!((cand0_cyl["radius"].as_f64().unwrap() - 28.0).abs() < 1e-6);
        let cands = opt_mm["candidate_plans"].as_array().unwrap();
        let cand_last_rev = &cands.last().unwrap()["dag"][0]["op"]["AddRevolve"];
        assert!((cand_last_rev["profile"][1][0].as_f64().unwrap() - 28.0).abs() < 1e-6);

        // 2. Call optimize_feature_tree with length_unit: "m" and apply: true -> verify SI meters in all plan DAGs
        let opt_m = optimize_feature_tree(
            &json!({
                "part_id": "bushing_p2t36",
                "apply": true,
                "length_unit": "m"
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(opt_m["length_unit"], "m");
        let log_cyl_m = &opt_m["logical_plan"]["dag"][0]["op"]["AddCylinder"];
        assert!((log_cyl_m["radius"].as_f64().unwrap() - 0.028).abs() < 1e-9);
        assert!((log_cyl_m["height"].as_f64().unwrap() - 0.06).abs() < 1e-9);
        let phys_rev_m = &opt_m["physical_plan"]["dag"][0]["op"]["AddRevolve"];
        assert!((phys_rev_m["profile"][0][0].as_f64().unwrap() - 0.014).abs() < 1e-9);
        assert!((phys_rev_m["profile"][1][0].as_f64().unwrap() - 0.028).abs() < 1e-9);

        // Verify internal part DAG was preserved in meters and get_feature_tree in "mm" scales it back to 14.0 / 28.0 mm
        let tree_mm = get_feature_tree(
            &json!({
                "part_id": "bushing_p2t36",
                "length_unit": "mm"
            }),
            &state,
        )
        .unwrap();
        let tree_rev = &tree_mm["dag"][0]["op"]["AddRevolve"];
        assert!((tree_rev["profile"][0][0].as_f64().unwrap() - 14.0).abs() < 1e-6);
        assert!((tree_rev["profile"][1][0].as_f64().unwrap() - 28.0).abs() < 1e-6);

        // 3. Verify delete_part returns status, part_id, and deleted_part_id
        let del_resp = delete_part(
            &json!({
                "part_id": "bushing_p2t36"
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(del_resp["status"], "deleted");
        assert_eq!(del_resp["part_id"], "bushing_p2t36");
        assert_eq!(del_resp["deleted_part_id"], "bushing_p2t36");
    }
}
