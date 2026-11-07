//! Kinematics, joints, and physics simulation tools powered by Rapier3D.

use std::collections::HashMap;
use serde_json::{json, Value};
use uuid::Uuid;

use mcp_cad_core::part::Part;
use mcp_cad_core::topology::{TriangleMesh, Vertex};
use mcp_cad_physics::joint::{CadJointType, MotorConfig};
use mcp_cad_physics::telemetry::BodyTelemetry;
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

    let part_a = args
        .get("part_a")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_a'".to_string()))?;

    let part_b = args
        .get("part_b")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_b'".to_string()))?;

    let unit = state.resolve_length_unit(args)?;
    let k_to_m = unit.to_meters_factor();

    let is_world_or_ground = |id: &str| id.eq_ignore_ascii_case("ground") || id.eq_ignore_ascii_case("world");

    if part_a.eq_ignore_ascii_case(part_b) || (is_world_or_ground(part_a) && is_world_or_ground(part_b)) {
        return Err(ServerError::InvalidArgument(
            "Cannot create a joint connecting a part or ground to itself".to_string(),
        ));
    }

    // Verify parts exist
    if !is_world_or_ground(part_a) && !state.parts.contains_key(part_a) {
        return Err(ServerError::PartNotFound(part_a.to_string()));
    }
    if !is_world_or_ground(part_b) && !state.parts.contains_key(part_b) {
        return Err(ServerError::PartNotFound(part_b.to_string()));
    }

    // Ensure non-ground parts are registered into the Rapier physics world if they have geometry
    for p_id in [part_a, part_b] {
        if !is_world_or_ground(p_id) {
            let part = &state.parts[p_id];
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

    let raw_limits = args.get("limits").and_then(|v| v.as_array()).and_then(|arr| {
        if arr.len() == 2 {
            Some([arr[0].as_f64().unwrap_or(0.0), arr[1].as_f64().unwrap_or(0.0)])
        } else {
            None
        }
    });
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
            let ratio = if let Some(r) = args.get("ratio").and_then(|v| v.as_f64()) {
                if !r.is_finite() || r.abs() < 1e-9 {
                    return Err(ServerError::InvalidArgument("Gear ratio must be a non-zero finite number".into()));
                }
                r
            } else if let (Some(ta), Some(tb)) = (
                args.get("teeth_a").and_then(|v| v.as_f64()),
                args.get("teeth_b").and_then(|v| v.as_f64()),
            ) {
                if tb.abs() < 1e-6 {
                    return Err(ServerError::InvalidArgument("teeth_b cannot be zero".into()));
                }
                ta / tb
            } else {
                1.0
            };

            let default_axis = parse_opt_vec3("axis", [0.0, 0.0, 1.0])?;
            let axis_a = parse_opt_vec3("axis_a", default_axis)?;
            let axis_b = parse_opt_vec3("axis_b", default_axis)?;

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
        .add_joint(joint_id.clone(), part_a, part_b, cad_joint, None)?;

    Ok(json!({
        "joint_id": joint_id,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
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

    Ok(json!({
        "status": "motor_active",
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
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

    let step_size_ms = args
        .get("step_size_ms")
        .and_then(|v| v.as_f64())
        .unwrap_or(16.6667);

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
                "target_velocity": 6.28,
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
}
