//! Kinematics, joints, and physics simulation tools powered by Rapier3D.

use serde_json::{json, Value};
use uuid::Uuid;

use mcp_cad_physics::joint::{CadJointType, MotorConfig};

use crate::error::ServerError;
use crate::state::EngineState;

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

    // Verify parts exist
    if !state.parts.contains_key(part_a) {
        return Err(ServerError::PartNotFound(part_a.to_string()));
    }
    if !state.parts.contains_key(part_b) {
        return Err(ServerError::PartNotFound(part_b.to_string()));
    }

    // Ensure parts are registered into the Rapier physics world if they have geometry
    for p_id in [part_a, part_b] {
        let part = &state.parts[p_id];
        if !part.mesh.vertices.is_empty() {
            // Only add if not already in physics world
            let _ = state.physics_world.add_part(part, true);
        }
    }

    let parse_vec3 = |key: &str| -> Result<[f64; 3], ServerError> {
        let arr = args
            .get(key)
            .and_then(|v| v.as_array())
            .ok_or_else(|| ServerError::InvalidArgument(format!("Missing required array '{key}' [x, y, z]")))?;
        if arr.len() != 3 {
            return Err(ServerError::InvalidArgument(format!("'{key}' must contain 3 elements")));
        }
        Ok([
            arr[0].as_f64().unwrap_or(0.0),
            arr[1].as_f64().unwrap_or(0.0),
            arr[2].as_f64().unwrap_or(0.0),
        ])
    };

    let anchor_a = parse_vec3("anchor_a")?;
    let anchor_b = parse_vec3("anchor_b")?;

    let limits = args.get("limits").and_then(|v| v.as_array()).and_then(|arr| {
        if arr.len() == 2 {
            Some([arr[0].as_f64().unwrap_or(0.0), arr[1].as_f64().unwrap_or(0.0)])
        } else {
            None
        }
    });

    let cad_joint = match joint_type_str.to_lowercase().as_str() {
        "revolute" => {
            let axis = parse_vec3("axis").unwrap_or([0.0, 0.0, 1.0]);
            CadJointType::Revolute {
                anchor_a,
                anchor_b,
                axis,
                limits,
            }
        }
        "prismatic" => {
            let axis = parse_vec3("axis").unwrap_or([1.0, 0.0, 0.0]);
            CadJointType::Prismatic {
                anchor_a,
                anchor_b,
                axis,
                limits,
            }
        }
        "spherical" => CadJointType::Spherical { anchor_a, anchor_b },
        "fixed" => CadJointType::Fixed { anchor_a, anchor_b },
        other => {
            return Err(ServerError::InvalidArgument(format!(
                "Unknown joint_type '{other}'. Supported: revolute, prismatic, spherical, fixed"
            )));
        }
    };

    let joint_id = format!("joint-{}", Uuid::new_v4().simple());
    state
        .physics_world
        .add_joint(joint_id.clone(), part_a, part_b, cad_joint, None)?;

    Ok(json!({
        "joint_id": joint_id
    }))
}

/// 9. `set_joint_motor`: Configures and activates a velocity motor on a joint.
pub fn set_joint_motor(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let joint_id = args
        .get("joint_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'joint_id'".to_string()))?;

    let target_velocity = args
        .get("target_velocity")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'target_velocity'".to_string()))?;

    let max_force = args
        .get("max_force")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'max_force'".to_string()))?;

    state
        .physics_world
        .set_joint_motor(joint_id, MotorConfig::new(target_velocity, max_force))?;

    Ok(json!({
        "status": "motor_active"
    }))
}

/// 10. `run_physics_simulation`: Runs dynamic physics stepping with collision and joint force capture.
pub fn run_physics_simulation(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let duration_seconds = args
        .get("duration_seconds")
        .and_then(|v| v.as_f64())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'duration_seconds'".to_string()))?;

    let step_size_ms = args
        .get("step_size_ms")
        .and_then(|v| v.as_f64())
        .unwrap_or(16.6667);

    let step_size_s = (step_size_ms / 1000.0).max(0.0001);

    // Synchronize any parts with valid mesh not yet in physics world
    for part in state.parts.values() {
        if !part.mesh.vertices.is_empty() {
            let _ = state.physics_world.add_part(part, true);
        }
    }

    let summary = state.physics_world.simulate(duration_seconds, step_size_s);
    state.last_simulation = Some(summary.clone());

    Ok(json!({
        "success": true,
        "trajectory_summary": summary.trajectories,
        "peak_joint_forces": summary.peak_joint_forces,
        "collisions": summary.collisions,
        "steps_simulated": summary.step_count
    }))
}
