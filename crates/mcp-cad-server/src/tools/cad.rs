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
            tool_mesh: tool_mesh.clone(),
        },
        FeatureOp::BooleanUnionMesh { tool_mesh } => FeatureOp::BooleanUnionMesh {
            tool_mesh: tool_mesh.clone(),
        },
        FeatureOp::BooleanIntersectionMesh { tool_mesh } => FeatureOp::BooleanIntersectionMesh {
            tool_mesh: tool_mesh.clone(),
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
pub fn list_parts(_args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let unit = state.length_unit;
    let summaries: Vec<Value> = state
        .parts
        .values()
        .map(|p| part_summary_in_unit(p, unit))
        .collect();
    Ok(json!({
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

    let (feature_id, mesh_clone, part_rev) = {
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

        (feature_id, part.mesh.clone(), part.revision)
    };

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &extra_dims_m, wheel_rim_diameter_m, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    let mut resp = json!({
        "feature_id": feature_id.to_string(),
        "revision": part_rev,
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

    let (feature_id, mesh_clone, part_rev, depth) = {
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

        (feature_id, part.mesh.clone(), part.revision, depth)
    };

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &[radius * 2.0, depth], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    let mut resp = json!({
        "feature_id": feature_id.to_string(),
        "revision": part_rev,
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
        .unwrap_or(args);

    let unit = if params.get("length_unit").is_some() {
        state.resolve_length_unit(params)?
    } else {
        state.resolve_length_unit(args)?
    };
    let k_to_m = unit.to_meters_factor();
    let k_from_m = unit.from_meters_factor();

    let (aabb, mesh_clone, part_rev) = {
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
                        if let Some(coords) = item.as_array() {
                            if coords.len() == 2 {
                                if let (Some(r), Some(z)) = (coords[0].as_f64(), coords[1].as_f64()) {
                                    p.push([r * k_to_m, z * k_to_m]);
                                }
                            }
                        }
                    }
                    if p.len() == 2 {
                        if let Some(inner_r) = inner_r_opt {
                            let ir_m = inner_r * k_to_m;
                            vec![
                                [ir_m, p[0][1]],
                                p[0],
                                p[1],
                                [ir_m, p[1][1]],
                            ]
                        } else {
                            p
                        }
                    } else if !p.is_empty() {
                        p
                    } else {
                        profile.clone()
                    }
                } else {
                    profile.clone()
                };
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
                        [
                            arr[0].as_f64().map(|v| v * k_to_m).unwrap_or(center[0]),
                            arr[1].as_f64().map(|v| v * k_to_m).unwrap_or(center[1]),
                            arr[2].as_f64().map(|v| v * k_to_m).unwrap_or(center[2]),
                        ]
                    } else {
                        *center
                    }
                } else {
                    *center
                };
                let new_normal = if let Some(arr) = params.get("normal").or_else(|| params.get("direction")).and_then(|v| v.as_array()) {
                    if arr.len() == 3 {
                        [arr[0].as_f64().unwrap_or(normal[0]), arr[1].as_f64().unwrap_or(normal[1]), arr[2].as_f64().unwrap_or(normal[2])]
                    } else {
                        *normal
                    }
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
                    [
                        arr[0].as_f64().map(|v| v * k_to_m).unwrap_or(translation[0]),
                        arr[1].as_f64().map(|v| v * k_to_m).unwrap_or(translation[1]),
                        arr[2].as_f64().map(|v| v * k_to_m).unwrap_or(translation[2]),
                    ]
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
        (aabb, mesh_clone, part.revision)
    };

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &[], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    let dims = aabb.dimensions();
    let mut resp = json!({
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

    let helix_radius = params
        .get("helix_radius")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'helix_radius'".to_string()))?
        * k_to_m;

    let groove_radius = params
        .get("groove_radius")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'groove_radius'".to_string()))?
        * k_to_m;

    let pitch = params
        .get("pitch")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'pitch'".to_string()))?
        * k_to_m;

    let height = params
        .get("height")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'height'".to_string()))?
        * k_to_m;

    let start_z = params
        .get("start_z")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0)
        * k_to_m;

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

    let (feature_id, mesh_clone) = {
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

        (feature_id, part.mesh.clone())
    };

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &[helix_radius * 2.0, groove_radius, pitch, height], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    let mut resp = json!({
        "feature_id": feature_id.to_string(),
        "status": "helical_groove_cut",
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
    let unit = state.length_unit;
    state.send_gui_mesh(&mesh_clone);

    let _ = state.physics_world.remove_part(target_part_id);
    if let Some(target_part) = state.parts.get(target_part_id) {
        if !target_part.mesh().vertices.is_empty() {
            let _ = state
                .physics_world
                .add_part(target_part, !target_part.is_fixed);
        }
    }

    let mut resp = json!({
        "feature_id": feature_id.to_string(),
        "status": "boolean_difference_applied",
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    });
    super::requirements::attach_auto_check_if_enabled(state, Some(target_part_id), &mut resp);
    Ok(resp)
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

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let mesh = mcp_cad_core::import_stl_file(path)?;

    let part_id = format!("part-{}", Uuid::new_v4().simple());
    let mut part = Part::new(&part_id, &part_name, material);
    part.mesh = mesh.clone();
    part.bump_revision();

    let warnings = state.check_scale_sanity(&part, &[], None, unit);
    let bbox = mesh.bounding_box();

    state.send_gui_mesh(&mesh);
    state.parts.insert(part_id.clone(), part);

    let mut resp = json!({
        "part_id": part_id,
        "name": part_name,
        "vertices_count": mesh.vertices.len(),
        "triangles_count": mesh.indices.len(),
        "bounding_box": {
            "min": scale_vec3(bbox.min, k_from_m),
            "max": scale_vec3(bbox.max, k_from_m),
        },
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

    let unit = state.resolve_length_unit(args)?;
    let k_to_m = unit.to_meters_factor();

    let chord_tolerance = args
        .get("chord_tolerance")
        .and_then(|v| v.as_f64())
        .map(|c| c * k_to_m);

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

    let (mesh_clone, volume_m3) = {
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
    let volume_unit = volume_m3 * k_from_m.powi(3);

    Ok(json!({
        "part_id": part_id,
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
        let id = id_val.as_u64().ok_or_else(|| {
            ServerError::InvalidArgument(
                "Field 'face_id' in face_selector must be a non-negative integer".to_string(),
            )
        })? as u32;
        let found = faces.iter().find(|f| f.id == id).ok_or_else(|| {
            let available: Vec<u32> = faces.iter().map(|f| f.id).collect();
            ServerError::InvalidArgument(format!(
                "FACE_NOT_FOUND: Face not found with face_id={id}. Available face_ids: {available:?}"
            ))
        })?;
        return Ok(vec![found]);
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

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let report = mcp_cad_core::validate_mesh(&part.mesh, expected_genus);
    let vol_m3 = report.geometry.signed_volume;
    let area_m2 = part.surface_area();
    let mut val = serde_json::to_value(&report).map_err(ServerError::Json)?;
    if let Some(obj) = val.as_object_mut() {
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
            let _ = state
                .physics_world
                .add_part(target_part, !target_part.is_fixed);
        }
    }

    let unit = state.length_unit;

    let mut resp = json!({
        "target_part_id": target_part_id,
        "feature_id": feature_id.to_string(),
        "status": "union_applied",
        "vertices_count": vertices_count,
        "triangles_count": triangles_count,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
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
            let _ = state
                .physics_world
                .add_part(target_part, !target_part.is_fixed);
        }
    }

    let unit = state.length_unit;

    let mut resp = json!({
        "target_part_id": target_part_id,
        "feature_id": feature_id.to_string(),
        "status": "intersection_applied",
        "vertices_count": vertices_count,
        "triangles_count": triangles_count,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
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

    let (feature_id, aabb, volume_m3, mesh_clone) = {
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

    let warnings = if let Some(part) = state.parts.get(part_id) {
        state.check_scale_sanity(part, &[], None, unit)
    } else {
        Vec::new()
    };

    state.send_gui_mesh(&mesh_clone);

    let _ = state.physics_world.remove_part(part_id);
    if let Some(part) = state.parts.get(part_id) {
        if !part.mesh().vertices.is_empty() {
            let _ = state.physics_world.add_part(part, !part.is_fixed);
        }
    }

    let mut resp = json!({
        "part_id": part_id,
        "feature_id": feature_id.to_string(),
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

    let (report, mesh_opt) = {
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
        (report, mesh)
    };

    if let Some(mesh_clone) = mesh_opt {
        state.send_gui_mesh(&mesh_clone);
        let _ = state.physics_world.remove_part(part_id);
        if let Some(part) = state.parts.get(part_id) {
            if !part.mesh().vertices.is_empty() {
                let _ = state.physics_world.add_part(part, !part.is_fixed);
            }
        }
    }

    let unit = state.length_unit;
    let mut val = serde_json::to_value(&report).map_err(ServerError::Json)?;
    if let Some(obj) = val.as_object_mut() {
        obj.insert("length_unit".to_string(), json!(unit.as_str()));
        obj.insert("units".to_string(), unit.units_metadata());
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
}

