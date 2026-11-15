//! Kinematics, joints, and physics simulation tools powered by Rapier3D.

use std::collections::HashMap;
use serde_json::{json, Value};
use uuid::Uuid;

use mcp_cad_core::part::Part;
use mcp_cad_core::topology::{TriangleMesh, Vertex};
use mcp_cad_physics::joint::{CadJointType, MotorConfig};
use mcp_cad_physics::telemetry::{BodyTelemetry, LoadCaseSelector};
use mcp_cad_physics::PhysicsError;

use crate::error::ServerError;
use crate::state::EngineState;

/// Transforms a 3D vertex position by an orientation quaternion [x, y, z, w] and translation offset.
fn transform_vertex(pos: [f64; 3], p_offset: [f64; 3], quat: [f64; 4]) -> [f64; 3] {
    let (qx, qy, qz, qw) = (quat[0], quat[1], quat[2], quat[3]);
    let rx = qy * pos[2] - qz * pos[1] + qw * pos[0];
    let ry = qz * pos[0] - qx * pos[2] + qw * pos[1];
    let rz = qx * pos[1] - qy * pos[0] + qw * pos[2];

    let cx = qy * rz - qz * ry;
    let cy = qz * rx - qx * rz;
    let cz = qx * ry - qy * rx;

    [
        pos[0] + 2.0 * cx + p_offset[0],
        pos[1] + 2.0 * cy + p_offset[1],
        pos[2] + 2.0 * cz + p_offset[2],
    ]
}

/// Rotates a 3D normal vector by an orientation quaternion [x, y, z, w].
fn rotate_normal(norm: [f64; 3], quat: [f64; 4]) -> [f64; 3] {
    let (qx, qy, qz, qw) = (quat[0], quat[1], quat[2], quat[3]);
    let rx = qy * norm[2] - qz * norm[1] + qw * norm[0];
    let ry = qz * norm[0] - qx * norm[2] + qw * norm[1];
    let rz = qx * norm[1] - qy * norm[0] + qw * norm[2];

    let cx = qy * rz - qz * ry;
    let cy = qz * rx - qx * rz;
    let cz = qx * ry - qy * rx;

    [
        norm[0] + 2.0 * cx,
        norm[1] + 2.0 * cy,
        norm[2] + 2.0 * cz,
    ]
}

/// Generates composite TriangleMesh animation frames from multi-body simulation trajectories.
pub fn generate_animation_frames(
    parts: &HashMap<String, Part>,
    trajectories: &HashMap<String, BodyTelemetry>,
    max_frames: usize,
) -> Vec<TriangleMesh> {
    let min_samples = trajectories
        .values()
        .map(|t| t.samples.len())
        .min()
        .unwrap_or(0);
    if min_samples == 0 {
        return Vec::new();
    }

    let stride = (min_samples / max_frames).max(1);
    let mut frames = Vec::new();

    let mut sorted_parts: Vec<_> = parts.iter().collect();
    sorted_parts.sort_by_key(|(id, _)| (*id).clone());

    for idx in (0..min_samples).step_by(stride) {
        let mut combined_verts = Vec::new();
        let mut combined_indices = Vec::new();

        for (part_id, part) in &sorted_parts {
            if part.mesh.vertices.is_empty() {
                continue;
            }

            let (pos, rot) = if let Some(traj) = trajectories.get(*part_id) {
                if let Some(sample) = traj.samples.get(idx) {
                    (sample.position, sample.orientation_quat)
                } else {
                    ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
                }
            } else {
                ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
            };

            let offset = combined_verts.len() as u32;
            for v in &part.mesh.vertices {
                let p = transform_vertex(v.position, pos, rot);
                let n = rotate_normal(v.normal, rot);
                combined_verts.push(Vertex {
                    position: p,
                    normal: n,
                });
            }

            for tri in &part.mesh.indices {
                combined_indices.push([tri[0] + offset, tri[1] + offset, tri[2] + offset]);
            }
        }

        if !combined_verts.is_empty() {
            frames.push(TriangleMesh::new(combined_verts, combined_indices));
        }
    }

    frames
}

/// 8. `create_joint`: Creates a kinematic or dynamic constraint between two parts.
pub fn create_joint(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let joint_type_str = args
        .get("joint_type")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'joint_type'".to_string()))?;

    let raw_part_a = args
        .get("part_a")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_a'".to_string()))?;

    let raw_part_b = args
        .get("part_b")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_b'".to_string()))?;

    let unit = state.resolve_length_unit(args)?;
    let k_to_m = unit.to_meters_factor();

    let is_world_or_ground = |id: &str| id.eq_ignore_ascii_case("ground") || id.eq_ignore_ascii_case("world");
    let norm_part_id = |id: &str| -> String {
        if id.eq_ignore_ascii_case("ground") {
            "ground".to_string()
        } else if id.eq_ignore_ascii_case("world") {
            "world".to_string()
        } else {
            id.to_string()
        }
    };
    let part_a = norm_part_id(raw_part_a);
    let part_b = norm_part_id(raw_part_b);

    if part_a.eq_ignore_ascii_case(&part_b) || (is_world_or_ground(&part_a) && is_world_or_ground(&part_b)) {
        return Err(ServerError::InvalidArgument(
            "Cannot create a joint connecting a part or ground to itself".to_string(),
        ));
    }

    // Verify parts exist
    if !is_world_or_ground(&part_a) && !state.parts.contains_key(&part_a) {
        return Err(ServerError::PartNotFound(raw_part_a.to_string()));
    }
    if !is_world_or_ground(&part_b) && !state.parts.contains_key(&part_b) {
        return Err(ServerError::PartNotFound(raw_part_b.to_string()));
    }

    // Ensure non-ground parts are registered into the Rapier physics world if they have geometry
    for p_id in [&part_a, &part_b] {
        if !is_world_or_ground(p_id) {
            let part = &state.parts[p_id.as_str()];
            if part.mesh.vertices.is_empty() {
                return Err(ServerError::InvalidArgument(format!(
                    "Part '{p_id}' has no geometry (empty mesh). Add geometric features or primitives to the part before creating joints."
                )));
            }
            if let Err(err) = state.physics_world.add_part(part, !part.is_fixed) {
                match err {
                    PhysicsError::PartAlreadyExists(_) => {}
                    other => return Err(ServerError::Physics(other)),
                }
            }
        }
    }

    let parse_opt_vec3 = |key: &str, default: [f64; 3]| -> Result<[f64; 3], ServerError> {
        let Some(val) = args.get(key) else {
            return Ok(default);
        };
        if val.is_null() {
            return Ok(default);
        }
        let arr = val
            .as_array()
            .ok_or_else(|| ServerError::InvalidArgument(format!("'{key}' must be an array [x, y, z]")))?;
        if arr.len() != 3 {
            return Err(ServerError::InvalidArgument(format!("'{key}' must contain 3 elements")));
        }
        let x = arr[0].as_f64().ok_or_else(|| ServerError::InvalidArgument(format!("'{key}[0]' must be a finite number")))?;
        let y = arr[1].as_f64().ok_or_else(|| ServerError::InvalidArgument(format!("'{key}[1]' must be a finite number")))?;
        let z = arr[2].as_f64().ok_or_else(|| ServerError::InvalidArgument(format!("'{key}[2]' must be a finite number")))?;
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return Err(ServerError::InvalidArgument(format!("'{key}' components must be finite numbers")));
        }
        Ok([x, y, z])
    };

    let scale_anchor = |v: [f64; 3]| -> [f64; 3] {
        [v[0] * k_to_m, v[1] * k_to_m, v[2] * k_to_m]
    };

    let raw_limits: Option<[f64; 2]> = match args.get("limits") {
        None => None,
        Some(v) if v.is_null() => None,
        Some(v) => {
            let arr = v.as_array().ok_or_else(|| {
                ServerError::InvalidArgument("'limits' must be an array [min, max]".into())
            })?;
            if arr.len() != 2 {
                return Err(ServerError::InvalidArgument(
                    "'limits' must contain 2 elements [min, max]".into(),
                ));
            }
            let l0 = arr[0].as_f64().ok_or_else(|| {
                ServerError::InvalidArgument("'limits[0]' must be a finite number".into())
            })?;
            let l1 = arr[1].as_f64().ok_or_else(|| {
                ServerError::InvalidArgument("'limits[1]' must be a finite number".into())
            })?;
            if !l0.is_finite() || !l1.is_finite() {
                return Err(ServerError::InvalidArgument(
                    "'limits' components must be finite numbers".into(),
                ));
            }
            Some([l0, l1])
        }
    };
    let limits_unit = args.get("limits_unit").and_then(|v| v.as_str()).unwrap_or("deg");

    let cad_joint = match joint_type_str.to_lowercase().as_str() {
        "revolute" => {
            let anchor_a = scale_anchor(parse_opt_vec3("anchor_a", [0.0, 0.0, 0.0])?);
            let anchor_b = scale_anchor(parse_opt_vec3("anchor_b", [0.0, 0.0, 0.0])?);
            let axis = parse_opt_vec3("axis", [0.0, 0.0, 1.0])?;
            let limits = raw_limits.map(|[l0, l1]| {
                if limits_unit.eq_ignore_ascii_case("rad") {
                    [l0, l1]
                } else {
                    [l0.to_radians(), l1.to_radians()]
                }
            });
            CadJointType::Revolute {
                anchor_a,
                anchor_b,
                axis,
                limits,
            }
        }
        "prismatic" => {
            let anchor_a = scale_anchor(parse_opt_vec3("anchor_a", [0.0, 0.0, 0.0])?);
            let anchor_b = scale_anchor(parse_opt_vec3("anchor_b", [0.0, 0.0, 0.0])?);
            let axis = parse_opt_vec3("axis", [1.0, 0.0, 0.0])?;
            let limits = raw_limits.map(|[l0, l1]| [l0 * k_to_m, l1 * k_to_m]);
            CadJointType::Prismatic {
                anchor_a,
                anchor_b,
                axis,
                limits,
            }
        }
        "spherical" => {
            let anchor_a = scale_anchor(parse_opt_vec3("anchor_a", [0.0, 0.0, 0.0])?);
            let anchor_b = scale_anchor(parse_opt_vec3("anchor_b", [0.0, 0.0, 0.0])?);
            CadJointType::Spherical { anchor_a, anchor_b }
        }
        "fixed" => {
            let anchor_a = scale_anchor(parse_opt_vec3("anchor_a", [0.0, 0.0, 0.0])?);
            let anchor_b = scale_anchor(parse_opt_vec3("anchor_b", [0.0, 0.0, 0.0])?);
            CadJointType::Fixed { anchor_a, anchor_b }
        }
        "gear" => {
            let ratio = if let Some(r_val) = args.get("ratio") {
                let r = r_val.as_f64().ok_or_else(|| {
                    ServerError::InvalidArgument("Argument 'ratio' must be a finite number".into())
                })?;
                if !r.is_finite() || r.abs() < 1e-9 {
                    return Err(ServerError::InvalidArgument(
                        "Gear 'ratio' must be a non-zero finite number".into(),
                    ));
                }
                r
            } else if args.get("teeth_a").is_some() || args.get("teeth_b").is_some() {
                let ta = args
                    .get("teeth_a")
                    .and_then(|v| v.as_f64())
                    .ok_or_else(|| {
                        ServerError::InvalidArgument(
                            "Argument 'teeth_a' must be a positive finite number (> 0)".into(),
                        )
                    })?;
                if !ta.is_finite() || ta <= 0.0 {
                    return Err(ServerError::InvalidArgument(
                        "Argument 'teeth_a' must be a positive finite number (> 0)".into(),
                    ));
                }

                let tb = args
                    .get("teeth_b")
                    .and_then(|v| v.as_f64())
                    .ok_or_else(|| {
                        ServerError::InvalidArgument(
                            "Argument 'teeth_b' must be a positive finite number (> 0)".into(),
                        )
                    })?;
                if !tb.is_finite() || tb <= 0.0 {
                    return Err(ServerError::InvalidArgument(
                        "Argument 'teeth_b' must be a positive finite number (> 0)".into(),
                    ));
                }

                let r = ta / tb;
                if !r.is_finite() || r.abs() < 1e-9 {
                    return Err(ServerError::InvalidArgument(
                        "Calculated gear ratio (teeth_a / teeth_b) must be a non-zero finite number".into(),
                    ));
                }
                r
            } else {
                1.0
            };

            let default_axis = parse_opt_vec3("axis", [0.0, 0.0, 1.0])?;
            let axis_a = parse_opt_vec3("axis_a", default_axis)?;
            let axis_b = parse_opt_vec3("axis_b", default_axis)?;

            let norm_a = (axis_a[0] * axis_a[0] + axis_a[1] * axis_a[1] + axis_a[2] * axis_a[2]).sqrt();
            if norm_a < 1e-7 || !norm_a.is_finite() {
                return Err(ServerError::InvalidArgument(
                    "Gear joint 'axis_a' cannot be a zero vector".into(),
                ));
            }

            let norm_b = (axis_b[0] * axis_b[0] + axis_b[1] * axis_b[1] + axis_b[2] * axis_b[2]).sqrt();
            if norm_b < 1e-7 || !norm_b.is_finite() {
                return Err(ServerError::InvalidArgument(
                    "Gear joint 'axis_b' cannot be a zero vector".into(),
                ));
            }

            CadJointType::Gear {
                ratio,
                axis_a,
                axis_b,
            }
        }
        other => {
            return Err(ServerError::InvalidArgument(format!(
                "Unknown joint_type '{other}'. Supported: revolute, prismatic, spherical, fixed, gear"
            )));
        }
    };

    let joint_id = match args.get("joint_id").or_else(|| args.get("id")) {
        None | Some(Value::Null) => format!("joint-{}", Uuid::new_v4().simple()),
        Some(val) => {
            let s = val.as_str().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Argument 'joint_id' must be a non-empty string".to_string(),
                )
            })?;
            let trimmed = s.trim();
            if trimmed.is_empty() {
                return Err(ServerError::InvalidArgument(
                    "Argument 'joint_id' must be a non-empty string".to_string(),
                ));
            }
            trimmed.to_string()
        }
    };

    state
        .physics_world
        .add_joint(joint_id.clone(), &part_a, &part_b, cad_joint, None)?;

    let mut resp = json!({
        "joint_id": joint_id,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    });
    super::requirements::attach_auto_check_if_enabled(state, None, &mut resp);

    Ok(resp)
}

/// 9. `set_joint_motor`: Configures and activates a velocity motor on a joint.
pub fn set_joint_motor(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let joint_id = args
        .get("joint_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'joint_id'".to_string()))?;

    let target_velocity = match args.get("target_velocity") {
        None | Some(Value::Null) => {
            return Err(ServerError::InvalidArgument(
                "Missing required argument 'target_velocity'".to_string(),
            ));
        }
        Some(v) => v.as_f64().ok_or_else(|| {
            ServerError::InvalidArgument(
                "Argument 'target_velocity' must be a finite number".to_string(),
            )
        })?,
    };

    if !target_velocity.is_finite() || !(target_velocity as f32).is_finite() {
        return Err(ServerError::InvalidArgument(format!(
            "Argument 'target_velocity' must be a finite number, got {target_velocity}"
        )));
    }

    let max_force = match args.get("max_force") {
        None | Some(Value::Null) => {
            return Err(ServerError::InvalidArgument(
                "Missing required argument 'max_force'".to_string(),
            ));
        }
        Some(v) => v.as_f64().ok_or_else(|| {
            ServerError::InvalidArgument(
                "Argument 'max_force' must be a finite non-negative number (>= 0.0)".to_string(),
            )
        })?,
    };

    if !max_force.is_finite() || !(max_force as f32).is_finite() {
        return Err(ServerError::InvalidArgument(format!(
            "Argument 'max_force' must be a finite number, got {max_force}"
        )));
    }

    if max_force < 0.0 {
        return Err(ServerError::InvalidArgument(format!(
            "Argument 'max_force' must be non-negative (>= 0.0), got {max_force}"
        )));
    }

    state
        .physics_world
        .set_joint_motor(joint_id, MotorConfig::new(target_velocity, max_force))?;

    let unit = state.length_unit;

    let mut resp = json!({
        "status": "motor_active",
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    });
    super::requirements::attach_auto_check_if_enabled(state, None, &mut resp);

    Ok(resp)
}

/// Converts a `BodyTelemetry` map's sample positions from internal SI meters to `k_from_m`.
pub(crate) fn scale_trajectories_from_meters(
    trajectories: &HashMap<String, BodyTelemetry>,
    k_from_m: f64,
) -> HashMap<String, BodyTelemetry> {
    trajectories
        .iter()
        .map(|(part_id, bt)| {
            let mut scaled = bt.clone();
            for s in &mut scaled.samples {
                s.position = [
                    s.position[0] * k_from_m,
                    s.position[1] * k_from_m,
                    s.position[2] * k_from_m,
                ];
            }
            (part_id.clone(), scaled)
        })
        .collect()
}

/// 10. `run_physics_simulation`: Runs dynamic physics stepping with collision and joint force capture.
pub fn run_physics_simulation(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let duration_seconds = args
        .get("duration_seconds")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'duration_seconds'".to_string()))?;

    if !duration_seconds.is_finite() || duration_seconds <= 0.0 {
        return Err(ServerError::InvalidArgument(
            "duration_seconds must be a positive finite number (> 0)".to_string(),
        ));
    }

    let step_size_ms = match args.get("step_size_ms") {
        None | Some(Value::Null) => 16.6667,
        Some(v) => {
            let s = v.as_f64().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Argument 'step_size_ms' must be a positive finite number (> 0)".to_string(),
                )
            })?;
            if !s.is_finite() || s <= 0.0 {
                return Err(ServerError::InvalidArgument(format!(
                    "Argument 'step_size_ms' must be a positive finite number (> 0), got {s}"
                )));
            }
            s
        }
    };

    let step_size_s = (step_size_ms / 1000.0).max(0.0001);
    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    // Synchronize any parts with valid mesh not yet in physics world
    for part in state.parts.values() {
        if !part.mesh.vertices.is_empty() {
            let _ = state.physics_world.add_part(part, !part.is_fixed);
        }
    }

    let summary = state.physics_world.simulate(duration_seconds, step_size_s);
    state.last_simulation = Some(summary.clone());

    // Generate animation frames in internal SI meters and send to interactive preview window
    let animation_frames = generate_animation_frames(&state.parts, &summary.trajectories, 120);
    if !animation_frames.is_empty() {
        state.send_gui_animation(animation_frames, 30.0);
    }

    let scaled_trajectories = scale_trajectories_from_meters(&summary.trajectories, k_from_m);

    Ok(json!({
        "success": true,
        "trajectory_summary": scaled_trajectories,
        "peak_joint_forces": summary.peak_joint_forces,
        "collisions": summary.collisions,
        "steps_simulated": summary.step_count,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// 11. `get_part_load_cases`: Extracts dynamic/quasi-static joint reaction forces, torques,
///     and inertial accelerations acting ON a part in its local coordinate system (`P3-T6`).
pub fn get_part_load_cases(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let raw_part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;
    let part_id = raw_part_id.trim();
    if part_id.is_empty() {
        return Err(ServerError::InvalidArgument(
            "Argument 'part_id' must be a non-empty string".to_string(),
        ));
    }

    let is_ground = part_id.eq_ignore_ascii_case("ground") || part_id.eq_ignore_ascii_case("world");
    if !is_ground && !state.parts.contains_key(part_id) {
        return Err(ServerError::PartNotFound(part_id.to_string()));
    }

    let selector = match args.get("select") {
        None | Some(Value::Null) => LoadCaseSelector::PeakForce,
        Some(Value::String(s)) => match s.trim().to_ascii_lowercase().as_str() {
            "peak_force" => LoadCaseSelector::PeakForce,
            "peak_torque" => LoadCaseSelector::PeakTorque,
            other => {
                if let Some(rest) = other.strip_prefix("time_s:") {
                    let t = rest.trim().parse::<f64>().map_err(|_| {
                        ServerError::InvalidArgument(format!(
                            "Argument 'select' has invalid timestamp '{s}'. Supported: 'peak_force', 'peak_torque', or a non-negative finite number (seconds)"
                        ))
                    })?;
                    if !t.is_finite() || t < 0.0 {
                        return Err(ServerError::InvalidArgument(format!(
                            "Argument 'select' timestamp must be a finite non-negative number (>= 0.0), got {t}"
                        )));
                    }
                    LoadCaseSelector::AtTime(t)
                } else if let Ok(t) = other.parse::<f64>() {
                    if !t.is_finite() || t < 0.0 {
                        return Err(ServerError::InvalidArgument(format!(
                            "Argument 'select' timestamp must be a finite non-negative number (>= 0.0), got {t}"
                        )));
                    }
                    LoadCaseSelector::AtTime(t)
                } else {
                    return Err(ServerError::InvalidArgument(format!(
                        "Argument 'select' has unknown mode '{s}'. Supported: 'peak_force', 'peak_torque', or a non-negative finite number (seconds)"
                    )));
                }
            }
        },
        Some(Value::Number(n)) => {
            let t = n.as_f64().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Argument 'select' timestamp must be a finite non-negative number (>= 0.0)"
                        .to_string(),
                )
            })?;
            if !t.is_finite() || t < 0.0 {
                return Err(ServerError::InvalidArgument(format!(
                    "Argument 'select' timestamp must be a finite non-negative number (>= 0.0), got {t}"
                )));
            }
            LoadCaseSelector::AtTime(t)
        }
        Some(Value::Object(map)) => {
            let t = map
                .get("time_s")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Argument 'select' object must contain a finite non-negative 'time_s' number"
                            .to_string(),
                    )
                })?;
            if !t.is_finite() || t < 0.0 {
                return Err(ServerError::InvalidArgument(format!(
                    "Argument 'select' timestamp must be a finite non-negative number (>= 0.0), got {t}"
                )));
            }
            LoadCaseSelector::AtTime(t)
        }
        Some(_) => {
            return Err(ServerError::InvalidArgument(
                "Argument 'select' must be 'peak_force', 'peak_torque', or a finite non-negative number (seconds)"
                    .to_string(),
            ));
        }
    };

    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();

    let summary = state.last_simulation.as_ref().ok_or_else(|| {
        ServerError::InvalidArgument(
            "No physics simulation has been run yet; execute run_physics_simulation before calling get_part_load_cases"
                .to_string(),
        )
    })?;

    let load_case = state
        .physics_world
        .extract_part_load_case(summary, part_id, selector)?;

    let scale_vec3 = |v: [f64; 3]| -> [f64; 3] { [v[0] * k_from_m, v[1] * k_from_m, v[2] * k_from_m] };

    let joints_json: Vec<Value> = load_case
        .joints
        .iter()
        .map(|j| {
            json!({
                "joint_id": j.joint_id,
                "joint_type": j.joint_type,
                "connected_part_id": j.connected_part_id,
                "role": j.role,
                "anchor_local": scale_vec3(j.anchor_local),
                "force": j.force,
                "torque": j.torque,
                "force_world": j.force_world,
                "torque_world": j.torque_world,
                "force_magnitude": j.force_magnitude,
                "torque_magnitude": j.torque_magnitude,
            })
        })
        .collect();

    let com_scaled = scale_vec3(load_case.inertia.center_of_mass_local);
    let pos_scaled = scale_vec3(load_case.position_world);

    Ok(json!({
        "part_id": load_case.part_id,
        "select": load_case.select,
        "time_s": load_case.time_s,
        "step_index": load_case.step_index,
        "position_world": pos_scaled,
        "orientation_quat": load_case.orientation_quat,
        "joints": joints_json,
        "net_joint_force": load_case.net_joint_force,
        "net_joint_torque_about_com": load_case.net_joint_torque_about_com,
        "net_joint_couple": load_case.net_joint_couple,
        "net_joint_force_magnitude": load_case.net_joint_force_magnitude,
        "net_joint_torque_about_com_magnitude": load_case.net_joint_torque_about_com_magnitude,
        "mass": load_case.inertia.mass_kg,
        "linear_acceleration": load_case.inertia.linear_acceleration,
        "angular_acceleration": load_case.inertia.angular_acceleration,
        "inertia": {
            "mass": load_case.inertia.mass_kg,
            "mass_kg": load_case.inertia.mass_kg,
            "is_fixed": load_case.inertia.is_fixed,
            "center_of_mass_local": com_scaled,
            "linear_acceleration": load_case.inertia.linear_acceleration,
            "angular_acceleration": load_case.inertia.angular_acceleration,
            "linear_acceleration_world": load_case.inertia.linear_acceleration_world,
            "angular_acceleration_world": load_case.inertia.angular_acceleration_world,
            "linear_velocity": load_case.inertia.linear_velocity,
            "angular_velocity": load_case.inertia.angular_velocity,
            "gravity_local": load_case.inertia.gravity_local,
        },
        "equilibrium_residual": {
            "force": load_case.equilibrium_residual_force,
            "force_norm": load_case.equilibrium_residual_force_norm,
            "torque": load_case.equilibrium_residual_torque,
            "torque_norm": load_case.equilibrium_residual_torque_norm,
        },
        "equilibrium_residual_force": load_case.equilibrium_residual_force,
        "equilibrium_residual_force_norm": load_case.equilibrium_residual_force_norm,
        "equilibrium_residual_torque": load_case.equilibrium_residual_torque,
        "equilibrium_residual_torque_norm": load_case.equilibrium_residual_torque_norm,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// Tool 12 (`analyze_mechanism`): Analyzes mechanism degrees of freedom, mobility, redundant constraints,
/// and floating/unconnected parts using both Chebychev-Grübler-Kutzbach and SVD rank of the
/// constraint Jacobian (`P3-T9`).
pub fn analyze_mechanism(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;

    let part_ids: Option<Vec<String>> = match args.get("part_ids") {
        None | Some(Value::Null) => None,
        Some(Value::Array(arr)) => {
            let mut ids = Vec::with_capacity(arr.len());
            for (idx, item) in arr.iter().enumerate() {
                let s = item.as_str().ok_or_else(|| {
                    ServerError::InvalidArgument(format!(
                        "Argument 'part_ids[{idx}]' must be a non-empty string"
                    ))
                })?;
                let trimmed = s.trim();
                if trimmed.is_empty() {
                    return Err(ServerError::InvalidArgument(format!(
                        "Argument 'part_ids[{idx}]' must not be empty"
                    )));
                }
                ids.push(trimmed.to_string());
            }
            Some(ids)
        }
        Some(_) => {
            return Err(ServerError::InvalidArgument(
                "Argument 'part_ids' must be an array of part ID strings".to_string(),
            ));
        }
    };

    let rank_tolerance: Option<f64> = match args.get("rank_tolerance") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let tol = v.as_f64().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Argument 'rank_tolerance' must be a positive number".to_string(),
                )
            })?;
            if !tol.is_finite() || tol <= 0.0 {
                return Err(ServerError::InvalidArgument(format!(
                    "Argument 'rank_tolerance' must be finite and > 0.0, got {tol}"
                )));
            }
            Some(tol)
        }
    };

    // Synchronize any parts with valid mesh not yet in physics world
    for part in state.parts.values() {
        if !part.mesh.vertices.is_empty() {
            let _ = state.physics_world.add_part(part, !part.is_fixed);
        }
    }

    let scene_parts: Vec<(String, bool)> = state
        .parts
        .iter()
        .map(|(id, part)| (id.clone(), part.is_fixed))
        .collect();

    let report = state.physics_world.analyze_mechanism_with_scene_parts(
        part_ids.as_deref(),
        rank_tolerance,
        Some(&scene_parts),
    )?;

    let k_from_m = unit.from_meters_factor();
    let joints_at_limits_json: Vec<Value> = report
        .joints_at_limits
        .iter()
        .map(|jl| {
            let (curr, lim, u_str) = if jl.joint_type == "revolute" {
                (
                    jl.current_value.to_degrees(),
                    [jl.limits[0].to_degrees(), jl.limits[1].to_degrees()],
                    "deg",
                )
            } else {
                (
                    jl.current_value * k_from_m,
                    [jl.limits[0] * k_from_m, jl.limits[1] * k_from_m],
                    unit.as_str(),
                )
            };
            json!({
                "joint_id": jl.joint_id,
                "joint_type": jl.joint_type,
                "boundary": jl.boundary,
                "current_value": curr,
                "limits": lim,
                "unit": u_str,
                "motor_blocked": jl.motor_blocked,
            })
        })
        .collect();

    let status = if report.warnings.is_empty() {
        "PASS"
    } else {
        "WARN"
    };

    Ok(json!({
        "status": status,
        "dof_numeric": report.dof_numeric,
        "dof_grubler": report.dof_grubler,
        "redundant_constraints": report.redundant_constraints,
        "redundant_joint_ids": report.redundant_joint_ids,
        "joints_at_limits": joints_at_limits_json,
        "grounded_parts": report.grounded_parts,
        "moving_parts": report.moving_parts,
        "floating_parts": report.floating_parts,
        "unconnected_parts": report.unconnected_parts,
        "joint_constraints_count": report.joint_constraints_count,
        "jacobian_rows": report.jacobian_rows,
        "jacobian_cols": report.jacobian_cols,
        "jacobian_rank": report.jacobian_rank,
        "singular_values": report.singular_values,
        "rank_tolerance": report.rank_tolerance,
        "warnings": report.warnings,
        "notes": report.notes,
        "joints_count": state.physics_world.joints().len(),
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// Tool 13 (`check_interference`): Performs static multi-body interference and clearance analysis
/// across all or selected part pairs using broad-phase AABB filtering and narrow-phase CSG Boolean
/// intersection (`P3-T10`).
pub fn check_interference(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;
    let k_from_m = unit.from_meters_factor();
    let k_vol = k_from_m.powi(3);

    let ignore_adjacent = match args.get("ignore_adjacent") {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(_) => {
            return Err(ServerError::InvalidArgument(
                "Argument 'ignore_adjacent' must be a boolean".to_string(),
            ));
        }
    };

    let part_ids: Option<Vec<String>> = match args.get("part_ids") {
        None | Some(Value::Null) => None,
        Some(Value::Array(arr)) => {
            let mut ids = Vec::with_capacity(arr.len());
            for (idx, item) in arr.iter().enumerate() {
                let s = item.as_str().ok_or_else(|| {
                    ServerError::InvalidArgument(format!(
                        "Argument 'part_ids[{idx}]' must be a non-empty string"
                    ))
                })?;
                let trimmed = s.trim();
                if trimmed.is_empty() {
                    return Err(ServerError::InvalidArgument(format!(
                        "Argument 'part_ids[{idx}]' must not be empty"
                    )));
                }
                if !trimmed.eq_ignore_ascii_case("ground")
                    && !trimmed.eq_ignore_ascii_case("world")
                    && !state.parts.contains_key(trimmed)
                {
                    return Err(ServerError::PartNotFound(trimmed.to_string()));
                }
                ids.push(trimmed.to_string());
            }
            Some(ids)
        }
        Some(_) => {
            return Err(ServerError::InvalidArgument(
                "Argument 'part_ids' must be an array of part ID strings".to_string(),
            ));
        }
    };

    let explicit_pairs: Option<Vec<(String, String)>> = match args.get("pairs") {
        None | Some(Value::Null) => None,
        Some(Value::Array(arr)) => {
            let mut list = Vec::with_capacity(arr.len());
            for (idx, pair_val) in arr.iter().enumerate() {
                let pair_arr = pair_val.as_array().ok_or_else(|| {
                    ServerError::InvalidArgument(format!(
                        "Argument 'pairs[{idx}]' must be a 2-element array [part_a, part_b]"
                    ))
                })?;
                if pair_arr.len() != 2 {
                    return Err(ServerError::InvalidArgument(format!(
                        "Argument 'pairs[{idx}]' must contain exactly 2 part IDs"
                    )));
                }
                let a = pair_arr[0]
                    .as_str()
                    .map(str::trim)
                    .filter(|s| !s.is_empty())
                    .ok_or_else(|| {
                        ServerError::InvalidArgument(format!(
                            "Argument 'pairs[{idx}][0]' must be a non-empty string"
                        ))
                    })?;
                let b = pair_arr[1]
                    .as_str()
                    .map(str::trim)
                    .filter(|s| !s.is_empty())
                    .ok_or_else(|| {
                        ServerError::InvalidArgument(format!(
                            "Argument 'pairs[{idx}][1]' must be a non-empty string"
                        ))
                    })?;
                if a == b {
                    return Err(ServerError::InvalidArgument(format!(
                        "Argument 'pairs[{idx}]' cannot pair part '{a}' with itself"
                    )));
                }
                if !state.parts.contains_key(a) {
                    return Err(ServerError::PartNotFound(a.to_string()));
                }
                if !state.parts.contains_key(b) {
                    return Err(ServerError::PartNotFound(b.to_string()));
                }
                list.push((a.to_string(), b.to_string()));
            }
            Some(list)
        }
        Some(_) => {
            return Err(ServerError::InvalidArgument(
                "Argument 'pairs' must be an array of [part_a, part_b] pairs".to_string(),
            ));
        }
    };

    // Synchronize any parts with valid mesh into the physics world
    for part in state.parts.values() {
        if !part.mesh.vertices.is_empty() {
            let _ = state.physics_world.add_part(part, !part.is_fixed);
        }
    }

    let report = state.physics_world.check_interference_with_parts(
        part_ids.as_deref(),
        ignore_adjacent,
        explicit_pairs.as_deref(),
        Some(&state.parts),
    )?;

    let mut pairs_json = Vec::with_capacity(report.pairs.len());
    let mut interfering_pairs_json = Vec::new();

    for p in &report.pairs {
        let pen_u = p.penetration_depth.map(|d| d * k_from_m);
        let vol_u = p.intersection_volume.map(|v| v * k_vol);
        let clr_u = p.clearance.map(|c| c * k_from_m);
        let item = json!({
            "a": p.a,
            "b": p.b,
            "intersects": p.intersects,
            "adjacent_by_joint": p.adjacent_by_joint,
            "aabb_overlap": p.aabb_overlap,
            "penetration_depth": pen_u,
            "penetration_depth_m": p.penetration_depth,
            "intersection_volume": vol_u,
            "intersection_volume_m3": p.intersection_volume,
            "clearance": clr_u,
            "clearance_m": p.clearance,
        });
        if p.intersects {
            interfering_pairs_json.push(item.clone());
        }
        pairs_json.push(item);
    }

    Ok(json!({
        "status": report.status,
        "intersects": report.interfering_pairs_count > 0,
        "candidate_pairs_count": report.candidate_pairs_count,
        "skipped_adjacent_pairs_count": report.skipped_adjacent_pairs_count,
        "broadphase_overlap_pairs_count": report.broadphase_overlap_pairs_count,
        "evaluated_pairs_count": report.evaluated_pairs_count,
        "interfering_pairs_count": report.interfering_pairs_count,
        "min_clearance": report.min_clearance.map(|c| c * k_from_m),
        "min_clearance_m": report.min_clearance,
        "max_intersection_volume": report.max_intersection_volume * k_vol,
        "max_intersection_volume_m3": report.max_intersection_volume,
        "max_penetration_depth": report.max_penetration_depth * k_from_m,
        "max_penetration_depth_m": report.max_penetration_depth,
        "ignore_adjacent": report.ignore_adjacent,
        "pairs": pairs_json,
        "interfering_pairs": interfering_pairs_json,
        "elapsed_ms": report.elapsed_ms,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// Tool 14 (`sweep_joint`): Sweeps a revolute or prismatic joint across `[from, to]` in `steps`
/// discrete intervals, evaluating interference and minimum clearance along the trajectory (`P3-T10`).
pub fn sweep_joint(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;
    let k_to_m = unit.to_meters_factor();
    let k_from_m = unit.from_meters_factor();
    let k_vol = k_from_m.powi(3);

    let raw_joint_id = args
        .get("joint_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| {
            ServerError::InvalidArgument("Missing required argument 'joint_id'".to_string())
        })?;
    let joint_id = raw_joint_id.trim();
    if joint_id.is_empty() {
        return Err(ServerError::InvalidArgument(
            "Argument 'joint_id' must be a non-empty string".to_string(),
        ));
    }

    let from_raw = match args.get("from") {
        None | Some(Value::Null) => {
            return Err(ServerError::InvalidArgument(
                "Missing required argument 'from'".to_string(),
            ));
        }
        Some(v) => v.as_f64().ok_or_else(|| {
            ServerError::InvalidArgument("Argument 'from' must be a finite number".to_string())
        })?,
    };
    if !from_raw.is_finite() {
        return Err(ServerError::InvalidArgument(format!(
            "Argument 'from' must be a finite number, got {from_raw}"
        )));
    }

    let to_raw = match args.get("to") {
        None | Some(Value::Null) => {
            return Err(ServerError::InvalidArgument(
                "Missing required argument 'to'".to_string(),
            ));
        }
        Some(v) => v.as_f64().ok_or_else(|| {
            ServerError::InvalidArgument("Argument 'to' must be a finite number".to_string())
        })?,
    };
    if !to_raw.is_finite() {
        return Err(ServerError::InvalidArgument(format!(
            "Argument 'to' must be a finite number, got {to_raw}"
        )));
    }

    let steps: usize = match args.get("steps") {
        None | Some(Value::Null) => {
            return Err(ServerError::InvalidArgument(
                "Missing required argument 'steps'".to_string(),
            ));
        }
        Some(v) => {
            let n = v.as_i64().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Argument 'steps' must be a positive integer (>= 1)".to_string(),
                )
            })?;
            if !(1..=10_000).contains(&n) {
                return Err(ServerError::InvalidArgument(format!(
                    "Argument 'steps' must be between 1 and 10000, got {n}"
                )));
            }
            n as usize
        }
    };

    let ignore_adjacent = match args.get("ignore_adjacent") {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(_) => {
            return Err(ServerError::InvalidArgument(
                "Argument 'ignore_adjacent' must be a boolean".to_string(),
            ));
        }
    };

    let angle_unit = match args.get("angle_unit") {
        None | Some(Value::Null) => "deg",
        Some(Value::String(s)) => match s.trim().to_ascii_lowercase().as_str() {
            "deg" | "degree" | "degrees" => "deg",
            "rad" | "radian" | "radians" => "rad",
            other => {
                return Err(ServerError::InvalidArgument(format!(
                    "Argument 'angle_unit' must be 'deg' or 'rad', got '{other}'"
                )));
            }
        },
        Some(_) => {
            return Err(ServerError::InvalidArgument(
                "Argument 'angle_unit' must be 'deg' or 'rad'".to_string(),
            ));
        }
    };

    // Synchronize any parts with valid mesh into the physics world
    for part in state.parts.values() {
        if !part.mesh.vertices.is_empty() {
            let _ = state.physics_world.add_part(part, !part.is_fixed);
        }
    }

    let joint_rec = state
        .physics_world
        .joints()
        .get(joint_id)
        .ok_or_else(|| ServerError::Physics(PhysicsError::JointNotFound(joint_id.to_string())))?;

    let (is_revolute, k_coord_to_si, k_coord_from_si, sweep_unit) = match &joint_rec.joint_type {
        CadJointType::Revolute { .. } => {
            if angle_unit == "rad" {
                (true, 1.0, 1.0, "rad".to_string())
            } else {
                (
                    true,
                    std::f64::consts::PI / 180.0,
                    180.0 / std::f64::consts::PI,
                    "deg".to_string(),
                )
            }
        }
        CadJointType::Prismatic { .. } => (false, k_to_m, k_from_m, unit.as_str().to_string()),
        other => {
            return Err(ServerError::Physics(
                PhysicsError::InvalidJointConfiguration(format!(
                    "sweep_joint is only supported on 'revolute' and 'prismatic' joints, but joint '{joint_id}' has type '{}'",
                    other.type_name()
                )),
            ));
        }
    };

    let from_si = from_raw * k_coord_to_si;
    let to_si = to_raw * k_coord_to_si;

    let report = state.physics_world.sweep_joint_with_parts(
        joint_id,
        from_si,
        to_si,
        steps,
        ignore_adjacent,
        Some(&state.parts),
    )?;

    let format_pair = |p: &mcp_cad_physics::telemetry::PairInterferenceResult| -> Value {
        json!({
            "a": p.a,
            "b": p.b,
            "intersects": p.intersects,
            "adjacent_by_joint": p.adjacent_by_joint,
            "aabb_overlap": p.aabb_overlap,
            "penetration_depth": p.penetration_depth.map(|d| d * k_from_m),
            "penetration_depth_m": p.penetration_depth,
            "intersection_volume": p.intersection_volume.map(|v| v * k_vol),
            "intersection_volume_m3": p.intersection_volume,
            "clearance": p.clearance.map(|c| c * k_from_m),
            "clearance_m": p.clearance,
        })
    };

    let first_collision_pairs_json: Vec<Value> = report
        .first_collision_pairs
        .iter()
        .map(format_pair)
        .collect();

    let samples_json: Vec<Value> = report
        .samples
        .iter()
        .map(|s| {
            let pairs_s: Vec<Value> = s.interfering_pairs.iter().map(format_pair).collect();
            json!({
                "step_index": s.step_index,
                "value": s.value * k_coord_from_si,
                "intersects": s.intersects,
                "interfering_pairs_count": s.interfering_pairs_count,
                "min_clearance": s.min_clearance.map(|c| c * k_from_m),
                "min_clearance_m": s.min_clearance,
                "max_penetration_depth": s.max_penetration_depth * k_from_m,
                "max_intersection_volume": s.max_intersection_volume * k_vol,
                "interfering_pairs": pairs_s,
            })
        })
        .collect();

    let first_col_val_u = report.first_collision_value.map(|v| v * k_coord_from_si);
    let min_clr_u = report.min_clearance.map(|c| c * k_from_m);
    let min_clr_val_u = report.min_clearance_value.map(|v| v * k_coord_from_si);
    let safe_range_u = report.safe_travel_range * k_coord_from_si;
    let margin_u = if report.collision_detected {
        Some(safe_range_u)
    } else {
        min_clr_u
    };

    let mut resp = json!({
        "status": report.status,
        "joint_id": report.joint_id,
        "joint_type": report.joint_type,
        "driven_part_id": report.driven_part_id,
        "reference_part_id": report.reference_part_id,
        "from": from_raw,
        "to": to_raw,
        "steps": report.steps,
        "sweep_unit": sweep_unit,
        "collision_detected": report.collision_detected,
        "first_collision_step": report.first_collision_step,
        "first_collision_value": first_col_val_u,
        "first_collision_pairs": first_collision_pairs_json,
        "min_clearance": min_clr_u,
        "min_clearance_m": report.min_clearance,
        "min_clearance_step": report.min_clearance_step,
        "min_clearance_value": min_clr_val_u,
        "safe_travel_range": safe_range_u,
        "margin_to_collision": margin_u,
        "ignore_adjacent": report.ignore_adjacent,
        "samples": samples_json,
        "elapsed_ms": report.elapsed_ms,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    });

    if is_revolute {
        if let Some(obj) = resp.as_object_mut() {
            obj.insert(
                "first_collision_angle_deg".to_string(),
                json!(report.first_collision_value.map(|v| v.to_degrees())),
            );
            obj.insert(
                "first_collision_angle_rad".to_string(),
                json!(report.first_collision_value),
            );
        }
    }

    Ok(resp)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::cad;

    fn setup_state_with_revolute_joint() -> (EngineState, String) {
        let mut state = EngineState::new().expect("EngineState init");
        cad::create_part(
            &json!({ "name": "Rotor", "part_id": "rotor_1", "material": "steel_structural_aisi1020" }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({
                "part_id": "rotor_1",
                "primitive_type": "cylinder",
                "params": { "radius": 10.0, "height": 30.0 }
            }),
            &mut state,
        )
        .unwrap();

        let j_res = create_joint(
            &json!({
                "joint_type": "revolute",
                "part_a": "ground",
                "part_b": "rotor_1",
                "axis": [0.0, 0.0, 1.0]
            }),
            &mut state,
        )
        .unwrap();
        let joint_id = j_res["joint_id"].as_str().unwrap().to_string();
        (state, joint_id)
    }

    #[test]
    fn test_set_joint_motor_rejects_negative_and_non_finite_parameters() {
        let (mut state, joint_id) = setup_state_with_revolute_joint();

        // 1. Negative max_force -> rejected with InvalidArgument on param 'max_force'
        let err_neg = set_joint_motor(
            &json!({
                "joint_id": joint_id,
                "target_velocity": 10.0,
                "max_force": -5.0
            }),
            &mut state,
        )
        .unwrap_err();
        let body_neg = err_neg.to_error_body();
        assert_eq!(body_neg.code, "INVALID_ARGUMENT");
        assert_eq!(body_neg.param.as_deref(), Some("max_force"));
        assert!(body_neg.message.contains("non-negative"));

        // 2. Overflowing f64 that becomes f32::INFINITY in Rapier -> rejected
        let err_inf_vel = set_joint_motor(
            &json!({
                "joint_id": joint_id,
                "target_velocity": 1.0e100,
                "max_force": 50.0
            }),
            &mut state,
        )
        .unwrap_err();
        let body_inf_vel = err_inf_vel.to_error_body();
        assert_eq!(body_inf_vel.code, "INVALID_ARGUMENT");
        assert_eq!(body_inf_vel.param.as_deref(), Some("target_velocity"));

        let err_inf_force = set_joint_motor(
            &json!({
                "joint_id": joint_id,
                "target_velocity": 10.0,
                "max_force": 1.0e100
            }),
            &mut state,
        )
        .unwrap_err();
        let body_inf_force = err_inf_force.to_error_body();
        assert_eq!(body_inf_force.code, "INVALID_ARGUMENT");
        assert_eq!(body_inf_force.param.as_deref(), Some("max_force"));

        // 3. String "NaN" or non-numeric type -> rejected with param 'target_velocity' / 'max_force'
        let err_str_vel = set_joint_motor(
            &json!({
                "joint_id": joint_id,
                "target_velocity": "NaN",
                "max_force": 50.0
            }),
            &mut state,
        )
        .unwrap_err();
        assert_eq!(err_str_vel.to_error_body().param.as_deref(), Some("target_velocity"));

        // 4. Boundary case max_force == 0.0 (free-wheeling motor) and negative target_velocity (reverse) -> allowed
        let ok_zero = set_joint_motor(
            &json!({
                "joint_id": joint_id,
                "target_velocity": -15.0,
                "max_force": 0.0
            }),
            &mut state,
        )
        .expect("max_force = 0.0 and negative target_velocity must be valid");
        assert_eq!(ok_zero["status"], "motor_active");
    }

    #[test]
    fn test_create_joint_custom_joint_id_and_duplicate_rejection() {
        let mut state = EngineState::new().expect("EngineState init");
        cad::create_part(
            &json!({ "name": "Rotor", "part_id": "rotor_custom", "material": "steel_1045" }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({
                "part_id": "rotor_custom",
                "primitive_type": "cylinder",
                "params": { "radius": 10.0, "height": 30.0 }
            }),
            &mut state,
        )
        .unwrap();

        // 1. Custom joint_id is preserved and usable in set_joint_motor
        let j_res = create_joint(
            &json!({
                "joint_id": "j_driver_custom",
                "joint_type": "revolute",
                "part_a": "ground",
                "part_b": "rotor_custom",
                "axis": [0.0, 0.0, 1.0]
            }),
            &mut state,
        )
        .expect("create_joint with custom joint_id should succeed");
        assert_eq!(j_res["joint_id"], "j_driver_custom");

        let m_res = set_joint_motor(
            &json!({
                "joint_id": "j_driver_custom",
                "target_velocity": std::f64::consts::TAU,
                "max_force": 100.0
            }),
            &mut state,
        )
        .expect("set_joint_motor with custom joint_id should succeed");
        assert_eq!(m_res["status"], "motor_active");

        // 2. Duplicate custom joint_id is rejected with JOINT_ALREADY_EXISTS
        let dup_err = create_joint(
            &json!({
                "joint_id": "j_driver_custom",
                "joint_type": "revolute",
                "part_a": "ground",
                "part_b": "rotor_custom",
                "axis": [0.0, 0.0, 1.0]
            }),
            &mut state,
        )
        .unwrap_err();
        let dup_body = dup_err.to_error_body();
        assert_eq!(dup_body.code, "JOINT_ALREADY_EXISTS");
        assert_eq!(dup_body.param.as_deref(), Some("joint_id"));

        // 3. Empty string joint_id is rejected with INVALID_ARGUMENT on 'joint_id'
        let empty_err = create_joint(
            &json!({
                "joint_id": "   ",
                "joint_type": "revolute",
                "part_a": "ground",
                "part_b": "rotor_custom",
                "axis": [0.0, 0.0, 1.0]
            }),
            &mut state,
        )
        .unwrap_err();
        let empty_body = empty_err.to_error_body();
        assert_eq!(empty_body.code, "INVALID_ARGUMENT");
        assert_eq!(empty_body.param.as_deref(), Some("joint_id"));
    }

    #[test]
    fn test_p2_t17_set_joint_motor_on_unsupported_joints_preserves_state_and_save_load() {
        let mut state = EngineState::new().expect("EngineState init");
        cad::create_part(
            &json!({ "name": "GearA", "part_id": "gear_a", "material": "steel_1045" }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({
                "part_id": "gear_a",
                "primitive_type": "cylinder",
                "params": { "radius": 10.0, "height": 20.0 }
            }),
            &mut state,
        )
        .unwrap();

        cad::create_part(
            &json!({ "name": "GearB", "part_id": "gear_b", "material": "steel_1045" }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({
                "part_id": "gear_b",
                "primitive_type": "cylinder",
                "params": { "radius": 20.0, "height": 20.0 }
            }),
            &mut state,
        )
        .unwrap();

        for (j_id, j_type) in [
            ("j_spherical", "spherical"),
            ("j_fixed", "fixed"),
            ("j_gear", "gear"),
        ] {
            create_joint(
                &json!({
                    "joint_id": j_id,
                    "joint_type": j_type,
                    "part_a": "gear_a",
                    "part_b": "gear_b",
                    "ratio": 0.5
                }),
                &mut state,
            )
            .expect("create_joint should succeed");

            let err = set_joint_motor(
                &json!({
                    "joint_id": j_id,
                    "target_velocity": 10.0,
                    "max_force": 50.0
                }),
                &mut state,
            )
            .expect_err("set_joint_motor must fail on spherical/fixed/gear");

            let body = err.to_error_body();
            assert_eq!(body.code, "INVALID_JOINT_CONFIGURATION");
            assert_eq!(body.param.as_deref(), Some("joint_id"));
            assert_eq!(
                state.physics_world.joints()[j_id].motor,
                None,
                "record.motor must remain None after failed set_joint_motor on {j_id}"
            );
        }

        // Verify get_scene_summary reports has_motor == false for all three joints
        let summary = crate::tools::metrics::get_scene_summary(&json!({}), &state).unwrap();
        for j_val in summary["joints"].as_array().unwrap() {
            assert_eq!(j_val["has_motor"], false);
            assert!(j_val["motor"].is_null());
        }

        // Verify save_project + load_project roundtrip succeeds cleanly
        let temp_dir = std::env::temp_dir().join(format!("mcpcad_p2t17_{}", Uuid::new_v4()));
        let proj_path = temp_dir.join("p2t17_test.json");
        let path_str = proj_path.to_str().unwrap();

        crate::tools::project::save_project(&json!({ "path": path_str }), &mut state)
            .expect("save_project must succeed");
        let mut loaded_state = EngineState::new().expect("EngineState init");
        let load_res = crate::tools::project::load_project(&json!({ "path": path_str }), &mut loaded_state)
            .expect("load_project must succeed when failed motor was not persisted");
        assert_eq!(load_res["joints_count"], 3);

        let _ = std::fs::remove_dir_all(temp_dir);
    }

    #[test]
    fn test_p2_t22_gear_validation_and_step_size() {
        let mut state = EngineState::new().expect("EngineState init");
        cad::create_part(
            &json!({ "name": "Gear1", "part_id": "gear_1", "material": "steel_1045" }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({
                "part_id": "gear_1",
                "primitive_type": "cylinder",
                "params": { "radius": 0.05, "height": 0.02 }
            }),
            &mut state,
        )
        .unwrap();

        cad::create_part(
            &json!({ "name": "Gear2", "part_id": "gear_2", "material": "steel_1045" }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({
                "part_id": "gear_2",
                "primitive_type": "cylinder",
                "params": { "radius": 0.1, "height": 0.02 }
            }),
            &mut state,
        )
        .unwrap();

        // 1. Ratio = 0.0
        let err = create_joint(
            &json!({
                "joint_id": "j_g_zero",
                "joint_type": "gear",
                "part_a": "gear_1",
                "part_b": "gear_2",
                "ratio": 0.0
            }),
            &mut state,
        )
        .unwrap_err();
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("ratio"));

        // 2. teeth_a <= 0
        let err = create_joint(
            &json!({
                "joint_id": "j_g_teeth_zero_a",
                "joint_type": "gear",
                "part_a": "gear_1",
                "part_b": "gear_2",
                "teeth_a": 0,
                "teeth_b": 40
            }),
            &mut state,
        )
        .unwrap_err();
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("teeth_a"));

        // 3. teeth_b <= 0
        let err = create_joint(
            &json!({
                "joint_id": "j_g_teeth_zero_b",
                "joint_type": "gear",
                "part_a": "gear_1",
                "part_b": "gear_2",
                "teeth_a": 20,
                "teeth_b": -5
            }),
            &mut state,
        )
        .unwrap_err();
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("teeth_b"));

        // 4. Non-numeric teeth
        let err = create_joint(
            &json!({
                "joint_id": "j_g_teeth_nan",
                "joint_type": "gear",
                "part_a": "gear_1",
                "part_b": "gear_2",
                "teeth_a": "twenty",
                "teeth_b": 40
            }),
            &mut state,
        )
        .unwrap_err();
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("teeth_a"));

        // 5. Zero vector axis_a
        let err = create_joint(
            &json!({
                "joint_id": "j_g_axis_a_zero",
                "joint_type": "gear",
                "part_a": "gear_1",
                "part_b": "gear_2",
                "ratio": 2.0,
                "axis_a": [0.0, 0.0, 0.0]
            }),
            &mut state,
        )
        .unwrap_err();
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("axis_a"));

        // 6. Zero vector axis_b
        let err = create_joint(
            &json!({
                "joint_id": "j_g_axis_b_zero",
                "joint_type": "gear",
                "part_a": "gear_1",
                "part_b": "gear_2",
                "ratio": 2.0,
                "axis_b": [0.0, 0.0, 0.0]
            }),
            &mut state,
        )
        .unwrap_err();
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("axis_b"));

        // 7. Valid teeth_a / teeth_b -> creates gear successfully
        let res_teeth = create_joint(
            &json!({
                "joint_id": "j_g_teeth_ok",
                "joint_type": "gear",
                "part_a": "gear_1",
                "part_b": "gear_2",
                "teeth_a": 20,
                "teeth_b": 40
            }),
            &mut state,
        )
        .expect("valid teeth should create gear joint");
        assert_eq!(res_teeth["joint_id"], "j_g_teeth_ok");

        // 8. step_size_ms validation in run_physics_simulation
        let err_step_neg = run_physics_simulation(
            &json!({
                "duration_seconds": 0.1,
                "step_size_ms": -1.0
            }),
            &mut state,
        )
        .unwrap_err();
        let body = err_step_neg.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("step_size_ms"));

        let err_step_zero = run_physics_simulation(
            &json!({
                "duration_seconds": 0.1,
                "step_size_ms": 0.0
            }),
            &mut state,
        )
        .unwrap_err();
        let body = err_step_zero.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("step_size_ms"));

        let err_step_str = run_physics_simulation(
            &json!({
                "duration_seconds": 0.1,
                "step_size_ms": "fast"
            }),
            &mut state,
        )
        .unwrap_err();
        let body = err_step_str.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("step_size_ms"));

        // 9. Valid run_physics_simulation with valid step_size_ms
        let sim_res = run_physics_simulation(
            &json!({
                "duration_seconds": 0.05,
                "step_size_ms": 10.0
            }),
            &mut state,
        )
        .expect("valid simulation should succeed");
        assert!(sim_res["steps_simulated"].as_u64().unwrap() > 0);
    }

    #[test]
    fn test_p3_t6_get_part_load_cases_unit() {
        let mut state = EngineState::new().expect("EngineState initialization failed");
        cad::create_part(
            &json!({ "name": "Wall", "part_id": "wall_u", "material": "steel_1045", "is_fixed": true }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({
                "part_id": "wall_u",
                "primitive_type": "box",
                "params": { "width": 20.0, "height": 60.0, "depth": 60.0 },
                "position": [-20.0, 0.0, 0.0]
            }),
            &mut state,
        )
        .unwrap();

        cad::create_part(
            &json!({ "name": "Beam", "part_id": "beam_u", "material": "steel_1045" }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({
                "part_id": "beam_u",
                "primitive_type": "box",
                "params": { "width": 200.0, "height": 20.0, "depth": 20.0 },
                "position": [100.0, 0.0, 0.0]
            }),
            &mut state,
        )
        .unwrap();

        // Before simulation -> INVALID_ARGUMENT
        let pre_err = get_part_load_cases(&json!({ "part_id": "beam_u" }), &mut state).unwrap_err();
        assert_eq!(pre_err.to_error_body().code, "INVALID_ARGUMENT");

        create_joint(
            &json!({
                "joint_id": "j_fixed_u",
                "joint_type": "fixed",
                "part_a": "wall_u",
                "part_b": "beam_u",
                "anchor_a": [0.0, 0.0, 0.0],
                "anchor_b": [0.0, 0.0, 0.0]
            }),
            &mut state,
        )
        .expect("create fixed joint should succeed");

        run_physics_simulation(
            &json!({
                "duration_seconds": 0.2,
                "step_size_ms": 5.0
            }),
            &mut state,
        )
        .expect("simulation should succeed");

        let lc = get_part_load_cases(
            &json!({
                "part_id": "beam_u",
                "select": "peak_force"
            }),
            &mut state,
        )
        .expect("get_part_load_cases should succeed");
        assert_eq!(lc["part_id"], "beam_u");
        assert_eq!(lc["joints"][0]["joint_id"], "j_fixed_u");
        assert!(lc["joints"][0]["force_magnitude"].as_f64().unwrap() > 0.0);
        assert!(lc["joints"][0]["torque_magnitude"].as_f64().unwrap() > 0.0);
    }

    #[test]
    fn test_p3_t9_analyze_mechanism_unit() {
        let mut state = EngineState::new().expect("EngineState init");

        // 1. Create a fixed base and a free moving arm (no joints yet)
        cad::create_part(
            &json!({ "name": "Base", "part_id": "base_u", "material": "steel_structural_aisi1020", "is_fixed": true }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({ "part_id": "base_u", "primitive_type": "box", "params": { "width": 100.0, "height": 20.0, "depth": 20.0 } }),
            &mut state,
        )
        .unwrap();

        cad::create_part(
            &json!({ "name": "Arm", "part_id": "arm_u", "material": "aluminum_6061_t6", "is_fixed": false }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({ "part_id": "arm_u", "primitive_type": "box", "params": { "width": 20.0, "height": 100.0, "depth": 20.0 } }),
            &mut state,
        )
        .unwrap();

        let free_rep = analyze_mechanism(&json!({}), &mut state).expect("analyze free");
        assert_eq!(free_rep["dof_numeric"], 6);
        assert_eq!(free_rep["dof_grubler"], 6);
        assert_eq!(free_rep["unconnected_parts"], json!(["arm_u"]));
        assert_eq!(free_rep["floating_parts"], json!(["arm_u"]));
        assert_eq!(free_rep["status"], "WARN");

        // 2. Connect with a Revolute joint -> DOF = 1
        create_joint(
            &json!({
                "joint_id": "j_rev_u",
                "joint_type": "revolute",
                "part_a": "base_u",
                "part_b": "arm_u",
                "anchor_a": [0.0, 0.0, 0.0],
                "anchor_b": [0.0, -50.0, 0.0],
                "axis": [0.0, 0.0, 1.0]
            }),
            &mut state,
        )
        .unwrap();

        let rev_rep = analyze_mechanism(&json!({}), &mut state).expect("analyze revolute");
        assert_eq!(rev_rep["dof_numeric"], 1);
        assert_eq!(rev_rep["dof_grubler"], 1);
        assert_eq!(rev_rep["redundant_constraints"], 0);
        assert_eq!(rev_rep["floating_parts"], json!([]));
        assert_eq!(rev_rep["unconnected_parts"], json!([]));
        assert_eq!(rev_rep["status"], "PASS");

        // 3. Invalid inputs: unknown part_id and non-positive rank_tolerance
        let err_part = analyze_mechanism(&json!({ "part_ids": ["missing_part"] }), &mut state).unwrap_err();
        assert_eq!(err_part.to_error_body().code, "PART_NOT_FOUND");

        let err_tol = analyze_mechanism(&json!({ "rank_tolerance": 0.0 }), &mut state).unwrap_err();
        assert_eq!(err_tol.to_error_body().code, "INVALID_ARGUMENT");
    }

    #[test]
    fn test_p3_t15_analyze_mechanism_limits_and_redundant_joints_unit() {
        let mut state = EngineState::new().expect("EngineState init");

        cad::create_part(
            &json!({ "name": "Slider", "part_id": "slider_u", "material": "steel_structural_aisi1020", "is_fixed": false }),
            &mut state,
        )
        .unwrap();
        cad::add_feature_primitive(
            &json!({ "part_id": "slider_u", "primitive_type": "box", "params": { "width": 20.0, "height": 20.0, "depth": 20.0 } }),
            &mut state,
        )
        .unwrap();

        // Prismatic joint at x = 0 with limits [0.0, 50.0] mm -> at lower limit boundary (0 mm)
        create_joint(
            &json!({
                "joint_id": "j_slide_lim",
                "joint_type": "prismatic",
                "part_a": "ground",
                "part_b": "slider_u",
                "anchor_a": [0.0, 0.0, 0.0],
                "anchor_b": [0.0, 0.0, 0.0],
                "axis": [1.0, 0.0, 0.0],
                "limits": [0.0, 50.0]
            }),
            &mut state,
        )
        .unwrap();

        let rep_mm = analyze_mechanism(&json!({}), &mut state).expect("analyze mm");
        assert_eq!(rep_mm["status"], "WARN");
        assert_eq!(rep_mm["dof_numeric"], 1);
        assert_eq!(rep_mm["redundant_joint_ids"], json!([]));
        let lims_mm = rep_mm["joints_at_limits"].as_array().unwrap();
        assert_eq!(lims_mm.len(), 1);
        assert_eq!(lims_mm[0]["joint_id"], "j_slide_lim");
        assert_eq!(lims_mm[0]["boundary"], "min");
        assert_eq!(lims_mm[0]["unit"], "mm");
        assert!((lims_mm[0]["limits"][1].as_f64().unwrap() - 50.0).abs() < 1e-6);

        let rep_m = analyze_mechanism(&json!({ "length_unit": "m" }), &mut state).expect("analyze m");
        let lims_m = rep_m["joints_at_limits"].as_array().unwrap();
        assert_eq!(lims_m[0]["unit"], "m");
        assert!((lims_m[0]["limits"][1].as_f64().unwrap() - 0.05).abs() < 1e-9);
    }
}


