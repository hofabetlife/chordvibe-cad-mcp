//! Cross-process Live Sync publisher for real-time Web CAD Studio visualization.
//!
//! Bridges isolated MCP agent processes (`--mode stdio` in Antigravity / Claude Desktop / Cursor
//! as well as `--mode sse`) with the full Web CAD Studio (`http://localhost:3000`) by writing
//! atomic JSON scene manifests (`scene.json`) and binary STL meshes (`parts/<id>.stl`) to a
//! shared live-sync workspace directory.

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use mcp_cad_core::dag::{FeatureNode, FeatureOp};
use mcp_cad_core::export_stl::export_stl_binary;
use mcp_cad_core::part::Part;
use mcp_cad_physics::joint::CadJointType;
use mcp_cad_physics::simulator::CadJointRecord;

use crate::state::EngineState;

/// Live scene manifest schema version.
pub const LIVE_SYNC_SCHEMA_VERSION: &str = "1.0.0";

/// Heartbeat freshness threshold (in milliseconds) for detecting an active Web Studio client.
const STUDIO_HEARTBEAT_MAX_AGE_MS: u64 = 15_000;

/// Maximum number of recent MCP tool calls retained in the live manifest.
const MAX_RECENT_TOOLS: usize = 25;

static LIVE_SNAPSHOT_SEQ: AtomicU64 = AtomicU64::new(0);

/// Compact record of an MCP tool execution shown in Web Studio's live agent feed.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LiveToolEvent {
    pub tool_name: String,
    pub status: String,
    pub summary: String,
    pub timestamp_ms: u64,
    pub duration_ms: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub params_preview: Option<Value>,
}

/// Compact serialized representation of a parametric DAG node (strips raw embedded meshes).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LiveDagNode {
    pub id: String,
    pub name: String,
    pub op_type: String,
    pub enabled: bool,
    pub params: Value,
}

/// Compact FEA analysis telemetry attached to a live part.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LivePartFeaSummary {
    pub max_von_mises_mpa: f64,
    pub yield_strength_mpa: f64,
    pub safety_factor: f64,
    pub max_displacement_mm: f64,
    pub verdict: String,
    pub node_count: usize,
    pub element_count: usize,
    pub element_order: u8,
    pub is_stale: bool,
}

/// Serialized material snapshot for a live part.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LiveMaterialInfo {
    pub name: String,
    pub youngs_modulus_gpa: f64,
    pub poissons_ratio: f64,
    pub yield_strength_mpa: f64,
    pub density_kg_m3: f64,
    pub failure_mode: String,
}

/// Serialized CAD part entry inside `scene.json`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LiveScenePart {
    pub id: String,
    pub safe_id: String,
    pub name: String,
    pub revision: u64,
    pub is_fixed: bool,
    pub material: LiveMaterialInfo,
    pub triangle_count: usize,
    pub vertex_count: usize,
    pub volume_m3: f64,
    pub mass_kg: f64,
    pub bounding_box_m: [[f64; 3]; 2],
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stl_file: Option<String>,
    pub dag: Vec<LiveDagNode>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fea: Option<LivePartFeaSummary>,
}

/// Serialized kinematic joint entry inside `scene.json`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LiveSceneJoint {
    pub id: String,
    pub part_a: String,
    pub part_b: String,
    pub joint_type: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub anchor_a_m: Option<[f64; 3]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub anchor_b_m: Option<[f64; 3]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub axis: Option<[f64; 3]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<[f64; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ratio: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motor_target_velocity: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motor_max_force: Option<f64>,
}

/// Top-level live scene snapshot manifest written atomically to `scene.json`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LiveSceneSnapshot {
    pub schema_version: String,
    pub revision: u64,
    pub timestamp_ms: u64,
    pub pid: u32,
    pub source: String,
    pub length_unit: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_tool: Option<LiveToolEvent>,
    #[serde(default)]
    pub recent_tools: Vec<LiveToolEvent>,
    pub parts: Vec<LiveScenePart>,
    pub joints: Vec<LiveSceneJoint>,
}

/// Per-EngineState tracker keeping exported STL revisions and recent MCP tool feed.
#[derive(Debug, Default)]
pub struct LiveSyncTracker {
    pub recent_tools: Vec<LiveToolEvent>,
    pub exported_part_revisions: HashMap<String, (u64, usize)>,
}

impl LiveSyncTracker {
    pub fn record_tool(&mut self, event: LiveToolEvent) {
        self.recent_tools.insert(0, event);
        if self.recent_tools.len() > MAX_RECENT_TOOLS {
            self.recent_tools.truncate(MAX_RECENT_TOOLS);
        }
    }
}

/// Returns current Unix timestamp in milliseconds.
pub fn now_unix_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// Sanitizes a part ID into a filesystem-safe stem for `parts/<safe_id>.stl`.
pub fn sanitize_part_id(part_id: &str) -> String {
    let mut out = String::with_capacity(part_id.len());
    for ch in part_id.chars() {
        if ch.is_ascii_alphanumeric() || ch == '_' || ch == '-' {
            out.push(ch);
        } else {
            out.push('_');
        }
    }
    if out.is_empty() {
        "part".to_string()
    } else {
        out
    }
}

/// Returns true if the current process is executing inside a Cargo test binary.
pub fn is_running_in_test_binary() -> bool {
    cfg!(test)
        || std::env::current_exe()
            .ok()
            .and_then(|exe| {
                exe.parent()
                    .and_then(|p| p.file_name())
                    .map(|n| n == std::ffi::OsStr::new("deps"))
            })
            .unwrap_or(false)
}

/// Returns true if an MCP tool can mutate the visual scene, geometry, physics, or FEA state.
pub fn should_publish_for_tool(tool_name: &str) -> bool {
    matches!(
        tool_name,
        "create_part"
            | "delete_part"
            | "add_feature_primitive"
            | "cut_hole"
            | "cut_helical_groove"
            | "modify_feature"
            | "boolean_difference"
            | "boolean_union"
            | "boolean_intersection"
            | "transform_part"
            | "tessellate_part"
            | "import_stl"
            | "optimize_feature_tree"
            | "batch_cad_operations"
            | "apply_recipe"
            | "create_joint"
            | "set_joint_motor"
            | "run_physics_simulation"
            | "run_fea_analysis"
            | "run_fea_from_simulation"
            | "bake_coupled_simulation"
            | "define_material"
            | "set_units"
            | "load_project"
            | "open_preview_window"
    )
}

/// Resolves the live-sync directories where `scene.json` and `parts/*.stl` should be written.
///
/// Priority:
/// 1. Explicit `MCP_CAD_LIVE_DIR` env var (returns only that directory; also enables sync in tests).
/// 2. If `MCP_CAD_DISABLE_LIVE_SYNC=1` or running in a test binary without `MCP_CAD_LIVE_DIR` -> empty.
/// 3. Workspace root `.mcp_live` (inferred from `current_exe()` walking up from `target/{release,debug}`,
///    or from `current_dir()` if `package.json`/`Cargo.toml` exists there) PLUS the OS temp fallback
///    `<temp_dir>/mcp-cad-live` so `server.ts` always finds the snapshot regardless of CWD!
pub fn resolve_live_sync_dirs() -> Vec<PathBuf> {
    if let Ok(explicit) = std::env::var("MCP_CAD_LIVE_DIR") {
        let trimmed = explicit.trim();
        if !trimmed.is_empty() {
            return vec![PathBuf::from(trimmed)];
        }
    }

    if let Ok(disabled) = std::env::var("MCP_CAD_DISABLE_LIVE_SYNC") {
        if disabled == "1" || disabled.eq_ignore_ascii_case("true") {
            return Vec::new();
        }
    }

    if is_running_in_test_binary() {
        return Vec::new();
    }

    let mut dirs: Vec<PathBuf> = Vec::new();

    // 1. Try deriving repository root from executable path (`.../target/{release,debug}/mcp-cad-server[.exe]`)
    if let Ok(exe) = std::env::current_exe() {
        let mut cursor = exe.parent();
        for _ in 0..4 {
            if let Some(dir) = cursor {
                if dir.join("Cargo.toml").exists() && dir.join("server.ts").exists() {
                    dirs.push(dir.join(".mcp_live"));
                    break;
                }
                cursor = dir.parent();
            } else {
                break;
            }
        }
    }

    // 2. Also check current working directory if it looks like the repo root
    if let Ok(cwd) = std::env::current_dir() {
        if cwd.join("Cargo.toml").exists() && cwd.join("server.ts").exists() {
            let candidate = cwd.join(".mcp_live");
            if !dirs.iter().any(|d| d == &candidate) {
                dirs.push(candidate);
            }
        }
    }

    // 3. Always include deterministic OS temp directory fallback (cross-CWD safe on Windows/Linux/macOS)
    let temp_dir = std::env::temp_dir().join("mcp-cad-live");
    if !dirs.iter().any(|d| d == &temp_dir) {
        dirs.push(temp_dir);
    }

    dirs
}

/// Checks whether a Web CAD Studio browser tab is currently connected and heartbeating.
pub fn is_web_studio_active() -> bool {
    let now = now_unix_ms();
    for dir in resolve_live_sync_dirs() {
        let hb_path = dir.join("studio_heartbeat.json");
        if let Ok(raw) = fs::read_to_string(&hb_path) {
            if let Ok(val) = serde_json::from_str::<Value>(&raw) {
                let clients = val
                    .get("connected_clients")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(0);
                let ts = val
                    .get("timestamp_ms")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(0);
                if clients > 0 && now.saturating_sub(ts) <= STUDIO_HEARTBEAT_MAX_AGE_MS {
                    return true;
                }
            }
        }
    }
    false
}

/// Converts a single Rust `FeatureNode` into a lightweight `LiveDagNode` for Web Studio.
fn serialize_live_dag_node(node: &FeatureNode) -> LiveDagNode {
    let (op_type, params) = match &node.op {
        FeatureOp::AddBox {
            width,
            height,
            depth,
            position,
            rotation_deg,
        } => (
            "box",
            json!({
                "width": width,
                "height": height,
                "depth": depth,
                "position": position,
                "rotation_deg": rotation_deg,
            }),
        ),
        FeatureOp::AddCylinder {
            radius,
            height,
            segments,
            position,
            rotation_deg,
        } => (
            "cylinder",
            json!({
                "radius": radius,
                "height": height,
                "segments": segments,
                "position": position,
                "rotation_deg": rotation_deg,
            }),
        ),
        FeatureOp::AddPolygonPrism {
            sides,
            height,
            width_across_flats,
            radius,
            chamfer,
            position,
            rotation_deg,
        } => (
            "polygon_prism",
            json!({
                "sides": sides,
                "height": height,
                "width_across_flats": width_across_flats,
                "radius": radius,
                "chamfer": chamfer,
                "position": position,
                "rotation_deg": rotation_deg,
            }),
        ),
        FeatureOp::AddSphere {
            radius,
            sectors,
            stacks,
            position,
            rotation_deg,
        } => (
            "sphere",
            json!({
                "radius": radius,
                "sectors": sectors,
                "stacks": stacks,
                "position": position,
                "rotation_deg": rotation_deg,
            }),
        ),
        FeatureOp::AddCone {
            radius_bottom,
            radius_top,
            height,
            segments,
            position,
            rotation_deg,
        } => (
            "cone",
            json!({
                "radius_bottom": radius_bottom,
                "radius_top": radius_top,
                "height": height,
                "segments": segments,
                "position": position,
                "rotation_deg": rotation_deg,
            }),
        ),
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
        } => (
            "spur_gear",
            json!({
                "module": module * 1000.0, // Web Studio displays gear module in mm
                "teeth": teeth,
                "pressure_angle_deg": pressure_angle_deg,
                "face_width": face_width,
                "bore_diameter": bore_radius.map(|r| r * 2.0),
                "keyway_width": keyway_width,
                "keyway_depth": keyway_depth,
                "hub_diameter": hub_diameter,
                "hub_length": hub_length,
                "position": position,
                "rotation_deg": rotation_deg,
            }),
        ),
        FeatureOp::AddRevolve {
            profile,
            angle_deg,
            segments,
            position,
            rotation_deg,
        } => (
            "revolve",
            json!({
                "profile": profile,
                "angle_deg": angle_deg,
                "segments": segments,
                "position": position,
                "rotation_deg": rotation_deg,
            }),
        ),
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
        } => (
            "wheel_rim",
            json!({
                "rim_diameter": rim_diameter,
                "rim_width": rim_width,
                "flange_diameter": flange_diameter,
                "hub_diameter": hub_diameter,
                "hub_bore_diameter": hub_bore_diameter,
                "pcd_diameter": pcd_diameter,
                "bolt_holes_count": bolt_holes_count,
                "bolt_hole_diameter": bolt_hole_diameter,
                "spokes_count": spokes_count,
                "spoke_width": spoke_width,
                "spoke_thickness": spoke_thickness,
                "offset_et": offset_et,
                "position": position,
                "rotation_deg": rotation_deg,
            }),
        ),
        FeatureOp::AddHelix {
            helix_radius,
            wire_radius,
            pitch,
            height,
            starts,
            is_right_hand,
            position,
            rotation_deg,
        } => (
            "helix",
            json!({
                "helix_radius": helix_radius,
                "wire_radius": wire_radius,
                "pitch": pitch,
                "height": height,
                "turns": if *pitch > 1e-9 { height / pitch } else { 1.0 },
                "starts": starts,
                "is_right_hand": is_right_hand,
                "position": position,
                "rotation_deg": rotation_deg,
            }),
        ),
        FeatureOp::CutHole {
            center,
            normal,
            radius,
            depth,
            keyway_width,
            keyway_depth,
        } => (
            "cut_hole",
            json!({
                "position": center,
                "normal": normal,
                "radius": radius,
                "depth": depth,
                "keyway_width": keyway_width,
                "keyway_depth": keyway_depth,
            }),
        ),
        FeatureOp::CutHelicalGroove {
            helix_radius,
            groove_radius,
            pitch,
            height,
            start_z,
            starts,
            is_right_hand,
            profile_type,
        } => (
            "cut_helical_groove",
            json!({
                "helix_radius": helix_radius,
                "groove_radius": groove_radius,
                "pitch": pitch,
                "height": height,
                "start_z": start_z,
                "starts": starts,
                "is_right_hand": is_right_hand,
                "profile_type": profile_type,
            }),
        ),
        FeatureOp::BooleanDifferenceMesh { tool_mesh } => (
            "boolean_difference",
            json!({ "tool_triangles": tool_mesh.indices.len() }),
        ),
        FeatureOp::BooleanUnionMesh { tool_mesh } => (
            "boolean_union",
            json!({ "tool_triangles": tool_mesh.indices.len() }),
        ),
        FeatureOp::BooleanIntersectionMesh { tool_mesh } => (
            "boolean_intersection",
            json!({ "tool_triangles": tool_mesh.indices.len() }),
        ),
        FeatureOp::ImportedMesh { mesh } => (
            "stl_import",
            json!({ "triangles": mesh.indices.len() }),
        ),
        FeatureOp::Transform {
            translation,
            rotation_deg,
            scale,
        } => (
            "transform",
            json!({
                "translation": translation,
                "rotation_deg": rotation_deg,
                "scale": scale,
            }),
        ),
    };

    LiveDagNode {
        id: node.id.to_string(),
        name: node.name.clone(),
        op_type: op_type.to_string(),
        enabled: node.enabled,
        params,
    }
}

fn serialize_live_joint(j: &CadJointRecord) -> LiveSceneJoint {
    let (joint_type, anchor_a_m, anchor_b_m, axis, limits, ratio) = match &j.joint_type {
        CadJointType::Revolute {
            anchor_a,
            anchor_b,
            axis,
            limits,
        } => (
            "revolute",
            Some(*anchor_a),
            Some(*anchor_b),
            Some(*axis),
            *limits,
            None,
        ),
        CadJointType::Prismatic {
            anchor_a,
            anchor_b,
            axis,
            limits,
        } => (
            "prismatic",
            Some(*anchor_a),
            Some(*anchor_b),
            Some(*axis),
            *limits,
            None,
        ),
        CadJointType::Spherical { anchor_a, anchor_b } => {
            ("spherical", Some(*anchor_a), Some(*anchor_b), None, None, None)
        }
        CadJointType::Fixed { anchor_a, anchor_b } => {
            ("fixed", Some(*anchor_a), Some(*anchor_b), None, None, None)
        }
        CadJointType::Gear {
            ratio,
            axis_a,
            axis_b: _,
        } => ("gear", None, None, Some(*axis_a), None, Some(*ratio)),
    };

    LiveSceneJoint {
        id: j.id.clone(),
        part_a: j.part_a.clone(),
        part_b: j.part_b.clone(),
        joint_type: joint_type.to_string(),
        anchor_a_m,
        anchor_b_m,
        axis,
        limits,
        ratio,
        motor_target_velocity: j.motor.map(|m| m.target_velocity),
        motor_max_force: j.motor.map(|m| m.max_force),
    }
}

/// Builds an in-memory `LiveSceneSnapshot` from the current `EngineState`.
pub fn build_live_snapshot(state: &EngineState, source: &str) -> LiveSceneSnapshot {
    let seq = LIVE_SNAPSHOT_SEQ.fetch_add(1, Ordering::SeqCst) + 1;

    let mut sorted_parts: Vec<&Part> = state.parts.values().collect();
    sorted_parts.sort_by(|a, b| a.id.cmp(&b.id));

    let parts: Vec<LiveScenePart> = sorted_parts
        .into_iter()
        .map(|p| {
            let safe_id = sanitize_part_id(&p.id);
            let aabb = p.bounding_box();
            let has_mesh = !p.mesh.indices.is_empty();
            let fea = state.last_fea_reports.get(&p.id).map(|rep| {
                let fea_rev = state.last_fea_revisions.get(&p.id).copied().unwrap_or(0);
                LivePartFeaSummary {
                    max_von_mises_mpa: rep.max_stress_von_mises_pa / 1.0e6,
                    yield_strength_mpa: rep.yield_strength_pa / 1.0e6,
                    safety_factor: rep.factor_of_safety,
                    max_displacement_mm: rep.max_displacement_m * 1000.0,
                    verdict: rep.verdict.clone(),
                    node_count: rep.node_count,
                    element_count: rep.element_count,
                    element_order: rep.element_order,
                    is_stale: fea_rev != p.revision,
                }
            });

            LiveScenePart {
                id: p.id.clone(),
                safe_id: safe_id.clone(),
                name: p.name.clone(),
                revision: p.revision,
                is_fixed: p.is_fixed,
                material: LiveMaterialInfo {
                    name: p.material.name.clone(),
                    youngs_modulus_gpa: p.material.youngs_modulus_gpa,
                    poissons_ratio: p.material.poissons_ratio,
                    yield_strength_mpa: p.material.yield_strength_mpa,
                    density_kg_m3: p.material.density_kg_m3,
                    failure_mode: p.material.failure_mode.as_str().to_string(),
                },
                triangle_count: p.mesh.indices.len(),
                vertex_count: p.mesh.vertices.len(),
                volume_m3: p.volume(),
                mass_kg: p.mass(),
                bounding_box_m: [aabb.min, aabb.max],
                stl_file: if has_mesh {
                    Some(format!("parts/{safe_id}.stl"))
                } else {
                    None
                },
                dag: p.dag.nodes.iter().map(serialize_live_dag_node).collect(),
                fea,
            }
        })
        .collect();

    let mut sorted_joints: Vec<&CadJointRecord> = state.physics_world.joints().values().collect();
    sorted_joints.sort_by(|a, b| a.id.cmp(&b.id));
    let joints: Vec<LiveSceneJoint> = sorted_joints
        .into_iter()
        .map(serialize_live_joint)
        .collect();

    LiveSceneSnapshot {
        schema_version: LIVE_SYNC_SCHEMA_VERSION.to_string(),
        revision: seq,
        timestamp_ms: now_unix_ms(),
        pid: std::process::id(),
        source: source.to_string(),
        length_unit: state.length_unit.as_str().to_string(),
        last_tool: state.live_sync.recent_tools.first().cloned(),
        recent_tools: state.live_sync.recent_tools.clone(),
        parts,
        joints,
    }
}

/// Atomically writes bytes to `target_path` by writing to a sibling `.tmp` file first and renaming.
fn atomic_write_bytes(target_path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    if let Some(parent) = target_path.parent() {
        fs::create_dir_all(parent)?;
    }
    let tmp_name = format!(
        ".{}.{}.{}.tmp",
        target_path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("live"),
        std::process::id(),
        LIVE_SNAPSHOT_SEQ.load(Ordering::Relaxed)
    );
    let tmp_path = target_path.with_file_name(tmp_name);
    fs::write(&tmp_path, bytes)?;
    // On Windows, fs::rename fails if target exists in some older environments; remove fallback ensures reliability.
    if fs::rename(&tmp_path, target_path).is_err() {
        let _ = fs::remove_file(target_path);
        fs::rename(&tmp_path, target_path)?;
    }
    Ok(())
}

/// Summarizes a tool call compactly for the live status banner in Web Studio.
fn summarize_tool_call(tool_name: &str, args: &Value, is_error: bool) -> String {
    if is_error {
        return format!("{tool_name}: ошибка");
    }
    let part_id = args
        .get("part_id")
        .or_else(|| args.get("target_part_id"))
        .and_then(|v| v.as_str());
    let prim = args
        .get("primitive_type")
        .or_else(|| args.get("recipe"))
        .and_then(|v| v.as_str());
    match (part_id, prim) {
        (Some(pid), Some(pt)) => format!("{tool_name}({pt} → {pid})"),
        (Some(pid), None) => format!("{tool_name}({pid})"),
        (None, Some(pt)) => format!("{tool_name}({pt})"),
        (None, None) => tool_name.to_string(),
    }
}

/// Publishes the current `EngineState` snapshot and any changed part STL meshes to all resolved
/// live-sync directories, and emits a notification on `state.live_scene_events_tx`.
pub fn publish_live_snapshot(
    state: &mut EngineState,
    tool_name: &str,
    args: &Value,
    is_error: bool,
    duration_ms: f64,
    source: &str,
) {
    let event = LiveToolEvent {
        tool_name: tool_name.to_string(),
        status: if is_error { "error" } else { "success" }.to_string(),
        summary: summarize_tool_call(tool_name, args, is_error),
        timestamp_ms: now_unix_ms(),
        duration_ms: (duration_ms * 10.0).round() / 10.0,
        params_preview: Some(args.clone()),
    };
    state.live_sync.record_tool(event);

    let snapshot = build_live_snapshot(state, source);
    let _ = state.live_scene_events_tx.send(snapshot.revision);

    let dirs = resolve_live_sync_dirs();
    if dirs.is_empty() {
        return;
    }

    // Export binary STL bytes only for parts whose revision or triangle count changed since last publish
    let mut updated_stls: Vec<(String, Vec<u8>)> = Vec::new();
    let mut active_safe_ids = std::collections::HashSet::new();

    for part in state.parts.values() {
        let safe_id = sanitize_part_id(&part.id);
        active_safe_ids.insert(safe_id.clone());
        let tri_count = part.mesh.indices.len();
        if tri_count == 0 {
            continue;
        }

        let prev = state.live_sync.exported_part_revisions.get(&safe_id).copied();
        let needs_disk_write = prev != Some((part.revision, tri_count))
            || dirs
                .iter()
                .any(|d| !d.join("parts").join(format!("{safe_id}.stl")).exists());

        if needs_disk_write {
            let mut buf = Vec::with_capacity(84 + tri_count * 50);
            if export_stl_binary(&part.mesh, &mut buf).is_ok() {
                updated_stls.push((safe_id.clone(), buf));
                state
                    .live_sync
                    .exported_part_revisions
                    .insert(safe_id, (part.revision, tri_count));
            }
        }
    }

    // Prune deleted parts from tracker
    state
        .live_sync
        .exported_part_revisions
        .retain(|safe_id, _| active_safe_ids.contains(safe_id));

    let Ok(scene_bytes) = serde_json::to_vec_pretty(&snapshot) else {
        return;
    };

    for dir in &dirs {
        let parts_dir = dir.join("parts");
        for (safe_id, stl_bytes) in &updated_stls {
            let target = parts_dir.join(format!("{safe_id}.stl"));
            if let Err(e) = atomic_write_bytes(&target, stl_bytes) {
                tracing::debug!("Live sync STL write skipped for {:?}: {e}", target);
            }
        }
        let manifest_path = dir.join("scene.json");
        if let Err(e) = atomic_write_bytes(&manifest_path, &scene_bytes) {
            tracing::debug!("Live sync manifest write skipped for {:?}: {e}", manifest_path);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mcp_cad_core::dag::FeatureOp;
    use mcp_cad_viewer::offscreen::OffscreenRenderer;

    #[test]
    fn test_live_sync_publishes_atomic_scene_and_stl() {
        let temp_dir =
            std::env::temp_dir().join(format!("mcp_cad_live_sync_test_{}", uuid::Uuid::new_v4()));
        std::env::set_var("MCP_CAD_LIVE_DIR", &temp_dir);

        let renderer = OffscreenRenderer::shared().expect("renderer");
        let mut state = EngineState::with_renderer(renderer);

        let steel = state
            .materials
            .get("steel_carbon_aisi1045")
            .expect("steel")
            .clone();
        let mut spring = Part::new("spring_1", "Coil Spring", steel);
        spring.dag.add_feature(
            FeatureOp::AddHelix {
                helix_radius: 0.0175,
                wire_radius: 0.0025,
                pitch: 0.012,
                height: 0.06,
                starts: 1,
                is_right_hand: true,
                position: None,
                rotation_deg: None,
            },
            Some("Helix Coil".to_string()),
        );
        spring.rebuild().expect("rebuild spring");
        state.parts.insert(spring.id.clone(), spring);

        publish_live_snapshot(
            &mut state,
            "add_feature_primitive",
            &json!({ "part_id": "spring_1", "primitive_type": "helix" }),
            false,
            4.2,
            "stdio",
        );

        let manifest_path = temp_dir.join("scene.json");
        let stl_path = temp_dir.join("parts").join("spring_1.stl");
        assert!(manifest_path.exists(), "scene.json must exist");
        assert!(stl_path.exists(), "parts/spring_1.stl must exist");

        let parsed: LiveSceneSnapshot =
            serde_json::from_str(&fs::read_to_string(&manifest_path).unwrap()).unwrap();
        assert_eq!(parsed.parts.len(), 1);
        assert_eq!(parsed.parts[0].id, "spring_1");
        assert_eq!(parsed.parts[0].dag.len(), 1);
        assert_eq!(parsed.parts[0].dag[0].op_type, "helix");
        assert!(parsed.parts[0].triangle_count > 100);
        assert_eq!(
            parsed.last_tool.as_ref().map(|t| t.tool_name.as_str()),
            Some("add_feature_primitive")
        );

        std::env::remove_var("MCP_CAD_LIVE_DIR");
        let _ = fs::remove_dir_all(&temp_dir);
    }
}
