//! Kinematics, joints, and physics simulation tools powered by Rapier3D.

use std::collections::HashMap;
use serde_json::{json, Value};
use uuid::Uuid;

use mcp_cad_core::part::Part;
use mcp_cad_core::topology::{TriangleMesh, Vertex};
use mcp_cad_physics::joint::{CadJointType, MotorConfig};
use mcp_cad_physics::telemetry::BodyTelemetry;

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

    let is_world_or_ground = |id: &str| id.eq_ignore_ascii_case("ground") || id.eq_ignore_ascii_case("world");

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
            if !part.mesh.vertices.is_empty() {
                let _ = state.physics_world.add_part(part, !part.is_fixed);
            }
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

    let limits = args.get("limits").and_then(|v| v.as_array()).and_then(|arr| {
        if arr.len() == 2 {
            Some([arr[0].as_f64().unwrap_or(0.0), arr[1].as_f64().unwrap_or(0.0)])
        } else {
            None
        }
    });

    let cad_joint = match joint_type_str.to_lowercase().as_str() {
        "revolute" => {
            let anchor_a = parse_vec3("anchor_a")?;
            let anchor_b = parse_vec3("anchor_b")?;
            let axis = parse_vec3("axis").unwrap_or([0.0, 0.0, 1.0]);
            CadJointType::Revolute {
                anchor_a,
                anchor_b,
                axis,
                limits,
            }
        }
        "prismatic" => {
            let anchor_a = parse_vec3("anchor_a")?;
            let anchor_b = parse_vec3("anchor_b")?;
            let axis = parse_vec3("axis").unwrap_or([1.0, 0.0, 0.0]);
            CadJointType::Prismatic {
                anchor_a,
                anchor_b,
                axis,
                limits,
            }
        }
        "spherical" => {
            let anchor_a = parse_vec3("anchor_a")?;
            let anchor_b = parse_vec3("anchor_b")?;
            CadJointType::Spherical { anchor_a, anchor_b }
        }
        "fixed" => {
            let anchor_a = parse_vec3("anchor_a")?;
            let anchor_b = parse_vec3("anchor_b")?;
            CadJointType::Fixed { anchor_a, anchor_b }
        }
        "gear" => {
            let ratio = if let Some(r) = args.get("ratio").and_then(|v| v.as_f64()) {
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

            let axis_a = parse_vec3("axis_a").or_else(|_| parse_vec3("axis")).unwrap_or([0.0, 0.0, 1.0]);
            let axis_b = parse_vec3("axis_b").or_else(|_| parse_vec3("axis")).unwrap_or([0.0, 0.0, 1.0]);

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
            let _ = state.physics_world.add_part(part, !part.is_fixed);
        }
    }

    let summary = state.physics_world.simulate(duration_seconds, step_size_s);
    state.last_simulation = Some(summary.clone());

    // Generate animation frames and send to interactive preview window
    let animation_frames = generate_animation_frames(&state.parts, &summary.trajectories, 120);
    if !animation_frames.is_empty() {
        state.send_gui_animation(animation_frames, 30.0);
    }

    Ok(json!({
        "success": true,
        "trajectory_summary": summary.trajectories,
        "peak_joint_forces": summary.peak_joint_forces,
        "collisions": summary.collisions,
        "steps_simulated": summary.step_count
    }))
}
