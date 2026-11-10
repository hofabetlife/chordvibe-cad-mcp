//! Project file persistence (`save_project` and `load_project`) with schema versioning.
//!
//! Encapsulates full assembly serialization including parametric DAG histories,
//! user-defined materials, kinematic joints with motor actuations, and active unit settings.
//! Boundary surface meshes are re-evaluated from the DAG on load to keep files compact.

use std::collections::HashMap;
use std::path::Path;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use mcp_cad_core::dag::FeatureTree;
use mcp_cad_core::material::{Material, MaterialRegistry};
use mcp_cad_core::part::Part;
use mcp_cad_core::topology::TriangleMesh;
use mcp_cad_physics::joint::{CadJointType, MotorConfig};
use mcp_cad_physics::simulator::{CadJointRecord, PhysicsWorld};

use crate::error::ServerError;
use crate::state::{EngineState, LengthUnit};

/// Canonical schema version supported by this server.
pub const PROJECT_SCHEMA_VERSION: &str = "1.0.0";

fn default_project_name() -> String {
    "MCP-CAD Project".to_string()
}

fn default_length_unit() -> String {
    "mm".to_string()
}

/// Flexible material representation supporting either a full Material object or a preset/registry name.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum PartMaterialRef {
    /// Fully specified material with mechanical and physical properties.
    Full(Material),
    /// Identifier or preset name from the material registry.
    Name(String),
}

/// Serialized representation of a CAD part in a saved project file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProjectPart {
    /// Unique part identifier.
    pub id: String,
    /// Human-readable label for the part.
    pub name: String,
    /// Material properties or registry identifier.
    pub material: PartMaterialRef,
    /// Parametric DAG history.
    pub dag: FeatureTree,
    /// Explicit mesh only retained for non-parametric imported geometries (e.g. raw STL).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mesh: Option<TriangleMesh>,
    /// Whether the part is fixed in 3D space during physics simulations.
    #[serde(default)]
    pub is_fixed: bool,
    /// Monotonically increasing geometry/DAG revision counter.
    #[serde(default)]
    pub revision: u64,
}

/// Serialized representation of a kinematic/dynamic joint in a saved project file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SavedJoint {
    /// Unique joint identifier.
    pub id: String,
    /// First connected part identifier (or "ground" / "world").
    pub part_a: String,
    /// Second connected part identifier (or "ground" / "world").
    pub part_b: String,
    /// Joint type and geometric anchors.
    pub joint_type: CadJointType,
    /// Optional motor actuation configuration.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motor: Option<MotorConfig>,
}

impl From<&CadJointRecord> for SavedJoint {
    fn from(r: &CadJointRecord) -> Self {
        Self {
            id: r.id.clone(),
            part_a: r.part_a.clone(),
            part_b: r.part_b.clone(),
            joint_type: r.joint_type.clone(),
            motor: r.motor,
        }
    }
}

/// Complete serialized document schema for a CAD project file.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectFile {
    /// JSON Schema identifier.
    #[serde(rename = "$schema", default, skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,
    /// Schema version for format stability and migration checks.
    pub schema_version: String,
    /// Human-readable project title or description.
    #[serde(default = "default_project_name")]
    pub project: String,
    /// Active linear length unit ("mm" or "m").
    #[serde(default = "default_length_unit")]
    pub length_unit: String,
    /// Full unit metadata dictionary.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub units: Option<Value>,
    /// Custom materials defined in this session beyond standard built-in presets.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub materials: Vec<Material>,
    /// CAD parts comprising the assembly.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub parts: Vec<ProjectPart>,
    /// Kinematic and dynamic joints connecting parts in the assembly.
    #[serde(default, alias = "kinematic_joints", skip_serializing_if = "Vec::is_empty")]
    pub joints: Vec<SavedJoint>,
    /// Engineering requirements and auto-check configuration.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub requirements: Option<crate::tools::requirements::RequirementsState>,
}

/// 30. `save_project`: Persists active assembly state to a versioned JSON file on disk.
pub fn save_project(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let path_str = args
        .get("path")
        .or_else(|| args.get("filepath"))
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'path'".to_string()))?;

    if path_str.trim().is_empty() {
        return Err(ServerError::InvalidArgument("Project path cannot be empty".to_string()));
    }

    let project_name = args
        .get("project_name")
        .or_else(|| args.get("project"))
        .and_then(|v| v.as_str())
        .unwrap_or("MCP-CAD Project")
        .to_string();

    // 1. Gather custom materials (those not matching defaults)
    let default_reg = MaterialRegistry::new();
    let mut custom_materials: Vec<Material> = state
        .materials
        .list()
        .into_iter()
        .filter(|m| default_reg.get(&m.name) != Some(m))
        .cloned()
        .collect();
    custom_materials.sort_by(|a, b| a.name.cmp(&b.name));

    // 2. Gather parts deterministically sorted by ID
    let mut sorted_parts: Vec<&Part> = state.parts.values().collect();
    sorted_parts.sort_by(|a, b| a.id.cmp(&b.id));

    let project_parts: Vec<ProjectPart> = sorted_parts
        .into_iter()
        .map(|p| {
            // Re-evaluate from DAG on load; only store raw mesh if DAG has no procedural nodes
            let mesh = if p.dag.nodes.is_empty() && !p.mesh.vertices.is_empty() {
                Some(p.mesh.clone())
            } else {
                None
            };
            ProjectPart {
                id: p.id.clone(),
                name: p.name.clone(),
                material: PartMaterialRef::Full(p.material.clone()),
                dag: p.dag.clone(),
                mesh,
                is_fixed: p.is_fixed,
                revision: p.revision,
            }
        })
        .collect();

    // 3. Gather joints deterministically sorted by ID
    let mut sorted_joints: Vec<&CadJointRecord> = state.physics_world.joints().values().collect();
    sorted_joints.sort_by(|a, b| a.id.cmp(&b.id));
    let saved_joints: Vec<SavedJoint> = sorted_joints.into_iter().map(SavedJoint::from).collect();

    let requirements = if state.requirements.is_empty() {
        None
    } else {
        Some(state.requirements.clone())
    };

    let project_file = ProjectFile {
        schema: Some("https://json-schema.org/draft/2020-12/schema".to_string()),
        schema_version: PROJECT_SCHEMA_VERSION.to_string(),
        project: project_name,
        length_unit: state.length_unit.as_str().to_string(),
        units: Some(state.length_unit.units_metadata()),
        materials: custom_materials,
        parts: project_parts,
        joints: saved_joints,
        requirements,
    };

    let json_str = serde_json::to_string_pretty(&project_file)?;

    let target_path = Path::new(path_str);
    if let Some(parent) = target_path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }

    std::fs::write(target_path, &json_str)?;

    Ok(json!({
        "status": "saved",
        "path": path_str,
        "schema_version": PROJECT_SCHEMA_VERSION,
        "project": project_file.project,
        "parts_count": project_file.parts.len(),
        "joints_count": project_file.joints.len(),
        "materials_count": project_file.materials.len(),
        "file_size_bytes": json_str.len(),
        "length_unit": state.length_unit.as_str(),
        "units": state.length_unit.units_metadata(),
    }))
}

/// 31. `load_project`: Atomically replaces session state from a versioned JSON project file.
pub fn load_project(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let path_str = args
        .get("path")
        .or_else(|| args.get("filepath"))
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'path'".to_string()))?;

    if path_str.trim().is_empty() {
        return Err(ServerError::InvalidArgument("Project path cannot be empty".to_string()));
    }

    let target_path = Path::new(path_str);
    let content = std::fs::read_to_string(target_path)?;
    let project_file: ProjectFile = serde_json::from_str(&content)?;

    // Validate schema version
    if project_file.schema_version != PROJECT_SCHEMA_VERSION {
        return Err(ServerError::InvalidArgument(format!(
            "Unsupported project schema_version '{}'. Current supported version is '{}'.",
            project_file.schema_version, PROJECT_SCHEMA_VERSION
        )));
    }

    let new_unit = LengthUnit::parse(&project_file.length_unit)?;

    // Transactional candidate stage: any failure before state mutation aborts without altering session
    let mut new_materials = MaterialRegistry::new();
    for mat in project_file.materials {
        new_materials.register(mat);
    }

    let mut new_parts = HashMap::new();
    for p_proj in project_file.parts {
        let p_id = p_proj.id.clone();
        if p_id.eq_ignore_ascii_case("ground") || p_id.eq_ignore_ascii_case("world") {
            return Err(ServerError::InvalidArgument(format!(
                "Part id '{p_id}' is reserved ('ground' and 'world' are reserved static physics anchors)"
            )));
        }
        let saved_revision = p_proj.revision;
        let material = match p_proj.material {
            PartMaterialRef::Full(m) => m,
            PartMaterialRef::Name(ref name) => new_materials.get(name).cloned().ok_or_else(|| {
                ServerError::InvalidArgument(format!(
                    "Part '{}' references material '{}' which was not found in registry",
                    p_id, name
                ))
            })?,
        };

        let mut part = Part {
            id: p_id.clone(),
            name: p_proj.name,
            material,
            dag: p_proj.dag,
            mesh: p_proj.mesh.unwrap_or_else(TriangleMesh::empty),
            cached_mesh: None,
            cached_settings: None,
            is_fixed: p_proj.is_fixed,
            revision: saved_revision,
        };

        if !part.dag.nodes.is_empty() {
            part.rebuild().map_err(ServerError::Cad)?;
            if saved_revision > 0 {
                part.revision = saved_revision;
            }
        } else if !part.mesh.vertices.is_empty() {
            part.ensure_imported_mesh_in_dag();
            if saved_revision > 0 {
                part.revision = saved_revision;
            }
        }

        new_parts.insert(p_id, part);
    }

    let mut new_physics_world = PhysicsWorld::new();
    for part in new_parts.values() {
        if !part.mesh.vertices.is_empty() {
            new_physics_world
                .add_part(part, !part.is_fixed)
                .map_err(ServerError::Physics)?;
        }
    }

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
    for joint in project_file.joints {
        let part_a = norm_part_id(&joint.part_a);
        let part_b = norm_part_id(&joint.part_b);

        if !is_world_or_ground(&part_a) && !new_parts.contains_key(&part_a) {
            return Err(ServerError::InvalidArgument(format!(
                "Joint '{}' references non-existent part_a '{}'",
                joint.id, joint.part_a
            )));
        }
        if !is_world_or_ground(&part_b) && !new_parts.contains_key(&part_b) {
            return Err(ServerError::InvalidArgument(format!(
                "Joint '{}' references non-existent part_b '{}'",
                joint.id, joint.part_b
            )));
        }

        new_physics_world
            .add_joint(
                joint.id,
                &part_a,
                &part_b,
                joint.joint_type,
                joint.motor,
            )
            .map_err(ServerError::Physics)?;
    }

    // All validation and geometry rebuilds succeeded: perform atomic state swap
    state.length_unit = new_unit;
    state.materials = new_materials;
    state.parts = new_parts;
    state.physics_world = new_physics_world;
    state.last_fea_reports.clear();
    state.last_fea_meshes.clear();
    state.last_fea_revisions.clear();
    state.fea_case_reports.clear();
    state.last_simulation = None;
    state.requirements = project_file.requirements.unwrap_or_default();

    if let Some(first_mesh) = state.parts.values().find(|p| !p.mesh.vertices.is_empty()).map(|p| &p.mesh) {
        state.send_gui_mesh(first_mesh);
    }

    let u_str = state.length_unit.as_str();
    let k_from_m = state.length_unit.from_meters_factor();
    let mut parts_summary = Vec::new();
    let mut sorted_loaded_parts: Vec<&Part> = state.parts.values().collect();
    sorted_loaded_parts.sort_by(|a, b| a.id.cmp(&b.id));

    for p in sorted_loaded_parts {
        let vol_m3 = p.volume();
        let vol_display = vol_m3 * k_from_m.powi(3);
        let mass_kg = vol_m3 * p.material.density_kg_m3;
        parts_summary.push(json!({
            "part_id": p.id,
            "name": p.name,
            "revision": p.revision,
            "material": p.material.name,
            "volume": vol_display,
            "volume_m3": vol_m3,
            "mass_kg": mass_kg,
            "triangles_count": p.mesh.indices.len(),
            "is_fixed": p.is_fixed,
        }));
    }

    let mut joints_summary = Vec::new();
    let mut sorted_loaded_joints: Vec<&CadJointRecord> = state.physics_world.joints().values().collect();
    sorted_loaded_joints.sort_by(|a, b| a.id.cmp(&b.id));

    for j in sorted_loaded_joints {
        let (type_name, anchor_a_opt, anchor_b_opt, axis_opt, axis_a_opt, axis_b_opt, limits_opt, limits_unit_opt, ratio_opt) =
            match &j.joint_type {
                CadJointType::Revolute {
                    anchor_a,
                    anchor_b,
                    axis,
                    limits,
                } => (
                    "revolute",
                    Some([
                        anchor_a[0] * k_from_m,
                        anchor_a[1] * k_from_m,
                        anchor_a[2] * k_from_m,
                    ]),
                    Some([
                        anchor_b[0] * k_from_m,
                        anchor_b[1] * k_from_m,
                        anchor_b[2] * k_from_m,
                    ]),
                    Some(*axis),
                    None,
                    None,
                    limits.map(|[min_rad, max_rad]| [min_rad.to_degrees(), max_rad.to_degrees()]),
                    limits.map(|_| "deg"),
                    None,
                ),
                CadJointType::Prismatic {
                    anchor_a,
                    anchor_b,
                    axis,
                    limits,
                } => (
                    "prismatic",
                    Some([
                        anchor_a[0] * k_from_m,
                        anchor_a[1] * k_from_m,
                        anchor_a[2] * k_from_m,
                    ]),
                    Some([
                        anchor_b[0] * k_from_m,
                        anchor_b[1] * k_from_m,
                        anchor_b[2] * k_from_m,
                    ]),
                    Some(*axis),
                    None,
                    None,
                    limits.map(|[min_m, max_m]| [min_m * k_from_m, max_m * k_from_m]),
                    limits.map(|_| u_str),
                    None,
                ),
                CadJointType::Spherical { anchor_a, anchor_b } => (
                    "spherical",
                    Some([
                        anchor_a[0] * k_from_m,
                        anchor_a[1] * k_from_m,
                        anchor_a[2] * k_from_m,
                    ]),
                    Some([
                        anchor_b[0] * k_from_m,
                        anchor_b[1] * k_from_m,
                        anchor_b[2] * k_from_m,
                    ]),
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                ),
                CadJointType::Fixed { anchor_a, anchor_b } => (
                    "fixed",
                    Some([
                        anchor_a[0] * k_from_m,
                        anchor_a[1] * k_from_m,
                        anchor_a[2] * k_from_m,
                    ]),
                    Some([
                        anchor_b[0] * k_from_m,
                        anchor_b[1] * k_from_m,
                        anchor_b[2] * k_from_m,
                    ]),
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                ),
                CadJointType::Gear {
                    ratio,
                    axis_a,
                    axis_b,
                } => (
                    "gear",
                    None,
                    None,
                    Some(*axis_a),
                    Some(*axis_a),
                    Some(*axis_b),
                    None,
                    None,
                    Some(*ratio),
                ),
            };
        let motor_json = j.motor.map(|m| {
            json!({
                "target_velocity": m.target_velocity,
                "max_force": m.max_force,
            })
        });
        let mut joint_obj = json!({
            "joint_id": j.id,
            "part_a": j.part_a,
            "part_b": j.part_b,
            "joint_type": type_name,
            "anchor_a": anchor_a_opt,
            "anchor_b": anchor_b_opt,
            "axis": axis_opt,
            "limits": limits_opt,
            "limits_unit": limits_unit_opt,
            "ratio": ratio_opt,
            "has_motor": j.motor.is_some(),
            "motor": motor_json,
        });
        if let (Some(aa), Some(ab)) = (axis_a_opt, axis_b_opt) {
            if let Some(map) = joint_obj.as_object_mut() {
                map.insert("axis_a".to_string(), json!(aa));
                map.insert("axis_b".to_string(), json!(ab));
            }
        }
        joints_summary.push(joint_obj);
    }

    Ok(json!({
        "status": "loaded",
        "path": path_str,
        "schema_version": project_file.schema_version,
        "project": project_file.project,
        "parts_count": state.parts.len(),
        "joints_count": state.physics_world.joints().len(),
        "materials_count": state.materials.len(),
        "length_unit": state.length_unit.as_str(),
        "units": state.length_unit.units_metadata(),
        "parts": parts_summary,
        "joints": joints_summary,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mcp_cad_core::dag::FeatureOp;
    use mcp_cad_viewer::offscreen::OffscreenRenderer;
    use std::sync::Arc;

    fn test_engine_state() -> EngineState {
        let renderer = Arc::new(OffscreenRenderer::new().expect("OffscreenRenderer init"));
        EngineState::with_renderer(renderer)
    }

    #[test]
    fn test_project_schema_version_constant() {
        assert_eq!(PROJECT_SCHEMA_VERSION, "1.0.0");
    }

    #[test]
    fn test_part_material_ref_deserialize() {
        let json_name = json!("aluminum_6061_t6");
        let ref_name: PartMaterialRef = serde_json::from_value(json_name).unwrap();
        assert_eq!(ref_name, PartMaterialRef::Name("aluminum_6061_t6".to_string()));

        let json_full = json!({
            "name": "custom_alloy",
            "youngs_modulus_gpa": 70.0,
            "poissons_ratio": 0.33,
            "yield_strength_mpa": 300.0,
            "density_kg_m3": 2800.0,
            "description": "Custom alloy"
        });
        let ref_full: PartMaterialRef = serde_json::from_value(json_full).unwrap();
        match ref_full {
            PartMaterialRef::Full(m) => {
                assert_eq!(m.name, "custom_alloy");
                assert_eq!(m.youngs_modulus_gpa, 70.0);
            }
            _ => panic!("Expected PartMaterialRef::Full"),
        }
    }

    #[test]
    fn test_save_and_load_roundtrip() {
        let mut state = test_engine_state();

        // 1. Define custom material
        let custom_mat = Material::new(
            "custom_carbon_fiber",
            150.0,
            0.28,
            600.0,
            1550.0,
            "Carbon fiber test material",
        );
        state.materials.register(custom_mat.clone());

        // 2. Create part A (Box)
        let mut part_a = Part::new("part_base", "Base Plate", custom_mat);
        part_a.dag.add_feature(
            FeatureOp::AddBox {
                width: 0.1, // 100 mm in SI
                height: 0.02,
                depth: 0.05,
                position: Some([0.0, 0.0, 0.0]),
                rotation_deg: Some([0.0, 0.0, 0.0]),
            },
            None,
        );
        part_a.is_fixed = true;
        part_a.rebuild().expect("rebuild part_a");
        state.physics_world.add_part(&part_a, false).expect("add part_a to physics");
        state.parts.insert(part_a.id.clone(), part_a);

        // 3. Create part B (Cylinder)
        let al_mat = state.materials.get("aluminum_6061_t6").unwrap().clone();
        let mut part_b = Part::new("part_arm", "Rotor Arm", al_mat);
        part_b.dag.add_feature(
            FeatureOp::AddCylinder {
                radius: 0.01,
                height: 0.06,
                position: Some([0.0, 0.02, 0.0]),
                rotation_deg: Some([0.0, 0.0, 0.0]),
                segments: 32,
            },
            None,
        );
        part_b.rebuild().expect("rebuild part_b");
        state.physics_world.add_part(&part_b, true).expect("add part_b to physics");
        state.parts.insert(part_b.id.clone(), part_b);

        // 4. Create joint with motor
        let joint_type = CadJointType::Revolute {
            anchor_a: [0.0, 0.02, 0.0],
            anchor_b: [0.0, 0.0, 0.0],
            axis: [0.0, 1.0, 0.0],
            limits: None,
        };
        let motor = MotorConfig::new(5.0, 50.0);
        state
            .physics_world
            .add_joint("joint-rev-1", "part_base", "part_arm", joint_type, Some(motor))
            .expect("add joint");

        let temp_dir = std::env::temp_dir().join(format!("mcpcad_test_{}", uuid::Uuid::new_v4()));
        let file_path = temp_dir.join("test_assembly.json");
        let path_str = file_path.to_str().unwrap();

        // Save
        let save_res = save_project(
            &json!({
                "path": path_str,
                "project_name": "Test Assembly"
            }),
            &mut state,
        )
        .expect("save_project failed");
        assert_eq!(save_res["status"], "saved");
        assert_eq!(save_res["parts_count"], 2);
        assert_eq!(save_res["joints_count"], 1);
        assert_eq!(save_res["materials_count"], 1);

        // Clear state
        let mut fresh_state = test_engine_state();
        assert_eq!(fresh_state.parts.len(), 0);
        assert_eq!(fresh_state.physics_world.joints().len(), 0);
        assert!(!fresh_state.materials.contains("custom_carbon_fiber"));

        // Load into fresh state
        let load_res = load_project(
            &json!({
                "path": path_str
            }),
            &mut fresh_state,
        )
        .expect("load_project failed");

        assert_eq!(load_res["status"], "loaded");
        assert_eq!(load_res["parts_count"], 2);
        assert_eq!(load_res["joints_count"], 1);
        assert_eq!(load_res["materials_count"], 9); // 8 builtins + 1 custom

        // Verify parts
        assert!(fresh_state.parts.contains_key("part_base"));
        assert!(fresh_state.parts.contains_key("part_arm"));
        let loaded_a = &fresh_state.parts["part_base"];
        let loaded_b = &fresh_state.parts["part_arm"];

        assert!(loaded_a.is_fixed);
        assert!(!loaded_b.is_fixed);
        assert_eq!(loaded_a.material.name, "custom_carbon_fiber");
        assert_eq!(loaded_b.material.name, "aluminum_6061_t6");

        // Verify geometric meshes were re-evaluated from DAG
        assert!(!loaded_a.mesh.vertices.is_empty());
        assert!(!loaded_b.mesh.vertices.is_empty());
        assert!((loaded_a.volume() - (0.1 * 0.02 * 0.05)).abs() < 1e-6);

        // Verify joint and motor in physics world
        let loaded_joints = fresh_state.physics_world.joints();
        assert_eq!(loaded_joints.len(), 1);
        let j = &loaded_joints["joint-rev-1"];
        assert_eq!(j.part_a, "part_base");
        assert_eq!(j.part_b, "part_arm");
        assert_eq!(j.motor, Some(MotorConfig::new(5.0, 50.0)));

        // Clean up
        let _ = std::fs::remove_dir_all(temp_dir);
    }

    #[test]
    fn test_load_corrupted_file_preserves_state() {
        let mut state = test_engine_state();
        let part = Part::new("p1", "P1", state.materials.get("aluminum_6061_t6").unwrap().clone());
        state.parts.insert(part.id.clone(), part);

        let temp_dir = std::env::temp_dir().join(format!("mcpcad_test_{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&temp_dir).unwrap();
        let file_path = temp_dir.join("corrupted.json");
        std::fs::write(&file_path, "{ broken json ...").unwrap();

        let res = load_project(&json!({ "path": file_path.to_str().unwrap() }), &mut state);
        assert!(res.is_err());

        // State remains intact!
        assert_eq!(state.parts.len(), 1);
        assert!(state.parts.contains_key("p1"));

        let _ = std::fs::remove_dir_all(temp_dir);
    }

    #[test]
    fn test_load_incompatible_schema_version() {
        let mut state = test_engine_state();
        let temp_dir = std::env::temp_dir().join(format!("mcpcad_test_{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&temp_dir).unwrap();
        let file_path = temp_dir.join("v2_project.json");
        std::fs::write(
            &file_path,
            json!({
                "schema_version": "2.0.0",
                "project": "Future Project",
                "length_unit": "mm",
                "parts": [],
                "joints": []
            })
            .to_string(),
        )
        .unwrap();

        let err = load_project(&json!({ "path": file_path.to_str().unwrap() }), &mut state).unwrap_err();
        let body = err.to_error_body();
        assert_eq!(body.code, "PROJECT_SCHEMA_MISMATCH");
        assert_eq!(body.param.as_deref(), Some("schema_version"));

        let _ = std::fs::remove_dir_all(temp_dir);
    }

    #[test]
    fn test_p2_t26_load_project_joints_limits_axis_and_ratio() {
        let mut state = test_engine_state();
        let al_mat = state.materials.get("aluminum_6061_t6").unwrap().clone();

        for (id, x_m) in [("p_a", 0.0), ("p_b", 0.05), ("p_c", 0.10)] {
            let mut p = Part::new(id, id, al_mat.clone());
            p.dag.add_feature(
                FeatureOp::AddBox {
                    width: 0.02,
                    height: 0.02,
                    depth: 0.02,
                    position: Some([x_m, 0.0, 0.0]),
                    rotation_deg: Some([0.0, 0.0, 0.0]),
                },
                None,
            );
            p.rebuild().expect("rebuild part");
            state.physics_world.add_part(&p, true).expect("add part");
            state.parts.insert(p.id.clone(), p);
        }

        // 1. Revolute joint with limits [-45 deg, +90 deg] (stored in radians) and motor
        state
            .physics_world
            .add_joint(
                "j_1_rev",
                "ground",
                "p_a",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.01, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: Some([(-45.0_f64).to_radians(), 90.0_f64.to_radians()]),
                },
                Some(MotorConfig::new(3.5, 25.0)),
            )
            .unwrap();

        // 2. Prismatic joint with limits [-0.01 m, +0.025 m] -> [-10 mm, +25 mm] in default "mm"
        state
            .physics_world
            .add_joint(
                "j_2_prism",
                "p_a",
                "p_b",
                CadJointType::Prismatic {
                    anchor_a: [0.02, 0.0, 0.0],
                    anchor_b: [-0.02, 0.0, 0.0],
                    axis: [1.0, 0.0, 0.0],
                    limits: Some([-0.01, 0.025]),
                },
                None,
            )
            .unwrap();

        // 3. Gear joint with ratio = -2.5
        state
            .physics_world
            .add_joint(
                "j_3_gear",
                "p_a",
                "p_c",
                CadJointType::Gear {
                    ratio: -2.5,
                    axis_a: [0.0, 0.0, 1.0],
                    axis_b: [0.0, 0.0, 1.0],
                },
                None,
            )
            .unwrap();

        let temp_dir = std::env::temp_dir().join(format!("mcpcad_p2t26_{}", uuid::Uuid::new_v4()));
        let file_path = temp_dir.join("joints_project.json");
        let path_str = file_path.to_str().unwrap();

        save_project(&json!({ "path": path_str, "project_name": "Joints P2-T26" }), &mut state)
            .expect("save_project");

        let mut loaded_state = test_engine_state();
        let load_res = load_project(&json!({ "path": path_str }), &mut loaded_state)
            .expect("load_project");

        let joints = load_res["joints"].as_array().expect("joints array");
        assert_eq!(joints.len(), 3);

        // j_1_rev
        let j_rev = &joints[0];
        assert_eq!(j_rev["joint_id"], "j_1_rev");
        assert_eq!(j_rev["joint_type"], "revolute");
        assert_eq!(j_rev["axis"], json!([0.0, 0.0, 1.0]));
        assert_eq!(j_rev["anchor_a"], json!([0.0, 10.0, 0.0]));
        assert_eq!(j_rev["limits_unit"], "deg");
        let rev_limits = j_rev["limits"].as_array().unwrap();
        assert!((rev_limits[0].as_f64().unwrap() - (-45.0)).abs() < 1e-6);
        assert!((rev_limits[1].as_f64().unwrap() - 90.0).abs() < 1e-6);
        assert!(j_rev["ratio"].is_null());
        assert_eq!(j_rev["has_motor"], true);
        assert_eq!(j_rev["motor"]["target_velocity"], 3.5);
        assert_eq!(j_rev["motor"]["max_force"], 25.0);

        // j_2_prism
        let j_prism = &joints[1];
        assert_eq!(j_prism["joint_id"], "j_2_prism");
        assert_eq!(j_prism["joint_type"], "prismatic");
        assert_eq!(j_prism["axis"], json!([1.0, 0.0, 0.0]));
        assert_eq!(j_prism["anchor_a"], json!([20.0, 0.0, 0.0]));
        assert_eq!(j_prism["anchor_b"], json!([-20.0, 0.0, 0.0]));
        assert_eq!(j_prism["limits_unit"], "mm");
        let prism_limits = j_prism["limits"].as_array().unwrap();
        assert!((prism_limits[0].as_f64().unwrap() - (-10.0)).abs() < 1e-6);
        assert!((prism_limits[1].as_f64().unwrap() - 25.0).abs() < 1e-6);
        assert!(j_prism["ratio"].is_null());
        assert_eq!(j_prism["has_motor"], false);

        // j_3_gear
        let j_gear = &joints[2];
        assert_eq!(j_gear["joint_id"], "j_3_gear");
        assert_eq!(j_gear["joint_type"], "gear");
        assert_eq!(j_gear["axis"], json!([0.0, 0.0, 1.0]));
        assert_eq!(j_gear["axis_a"], json!([0.0, 0.0, 1.0]));
        assert_eq!(j_gear["axis_b"], json!([0.0, 0.0, 1.0]));
        assert_eq!(j_gear["ratio"], -2.5);
        assert!(j_gear["limits"].is_null());
        assert!(j_gear["limits_unit"].is_null());

        let _ = std::fs::remove_dir_all(temp_dir);
    }
}

