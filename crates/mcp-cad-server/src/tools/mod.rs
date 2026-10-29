//! MCP tool registry, JSON schemas, and execution dispatcher.

pub mod cad;
pub mod export;
pub mod fea;
pub mod metrics;
pub mod physics;
pub mod render;

use serde_json::{json, Value};

use crate::error::ServerError;
use crate::protocol::{ToolCallResult, ToolDefinition};
use crate::state::EngineState;

/// Returns the complete registry of all 18 MCP engineering tools with strict input schemas.
pub fn list_tools() -> Vec<ToolDefinition> {
    vec![
        // 1. create_part
        ToolDefinition {
            name: "create_part".to_string(),
            description: "Creates a new CAD part assigned with an engineering material (e.g. aluminum_6061_t6, steel_structural_aisi1020, titanium_grade5_ti6al4v, pla_filament_3dprint)".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "name": {
                        "type": "string",
                        "description": "Human-readable label for the part"
                    },
                    "material": {
                        "type": "string",
                        "description": "Identifier of material from registry (e.g. aluminum_6061_t6, steel_structural_aisi1020)"
                    }
                },
                "required": ["name", "material"]
            }),
        },
        // 2. list_parts
        ToolDefinition {
            name: "list_parts".to_string(),
            description: "Lists all existing CAD parts in the engine scene with bounding box dimensions, volume, mass, and triangle count".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {}
            }),
        },
        // 3. delete_part
        ToolDefinition {
            name: "delete_part".to_string(),
            description: "Deletes a part from the scene, physics simulation world, and analysis cache".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Unique identifier of the part to remove"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 4. add_feature_primitive
        ToolDefinition {
            name: "add_feature_primitive".to_string(),
            description: "Appends a 3D solid primitive (box, cylinder, sphere, or cone) into the parametric history DAG of a part, with optional 3D position and rotation".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "primitive_type": {
                        "type": "string",
                        "enum": ["box", "cylinder", "sphere", "cone"],
                        "description": "Type of 3D primitive"
                    },
                    "params": {
                        "type": "object",
                        "description": "Geometric parameters (box: width, height, depth; cylinder: radius, height; sphere: radius; cone: radius_bottom, radius_top, height; optional position [x,y,z], rotation_deg [rx,ry,rz])",
                        "properties": {
                            "width": { "type": "number", "description": "Box width in meters" },
                            "height": { "type": "number", "description": "Height in meters (box, cylinder, cone)" },
                            "depth": { "type": "number", "description": "Box depth in meters" },
                            "radius": { "type": "number", "description": "Radius in meters (cylinder, sphere)" },
                            "radius_bottom": { "type": "number", "description": "Bottom radius in meters (cone)" },
                            "radius_top": { "type": "number", "description": "Top radius in meters (cone, 0 for sharp point)" },
                            "segments": { "type": "integer", "description": "Radial segments (default 32)" },
                            "sectors": { "type": "integer", "description": "Sphere sectors (default 32)" },
                            "stacks": { "type": "integer", "description": "Sphere stacks (default 16)" },
                            "position": {
                                "type": "array",
                                "items": { "type": "number" },
                                "minItems": 3,
                                "maxItems": 3,
                                "description": "Optional center position [x, y, z] in meters"
                            },
                            "rotation_deg": {
                                "type": "array",
                                "items": { "type": "number" },
                                "minItems": 3,
                                "maxItems": 3,
                                "description": "Optional rotation angles [rx, ry, rz] in degrees"
                            }
                        }
                    },
                    "position": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Optional position [x, y, z] in meters"
                    },
                    "rotation_deg": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Optional rotation angles [rx, ry, rz] in degrees"
                    }
                },
                "required": ["part_id", "primitive_type", "params"]
            }),
        },
        // 5. cut_hole
        ToolDefinition {
            name: "cut_hole".to_string(),
            description: "Cuts a cylindrical hole through a CAD part along a specified direction normal vector".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "position": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Hole entry center coordinates [x, y, z] in meters"
                    },
                    "normal": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Unit normal vector [nx, ny, nz] defining the hole axis"
                    },
                    "radius": {
                        "type": "number",
                        "description": "Hole radius in meters"
                    },
                    "depth": {
                        "type": "number",
                        "description": "Hole depth in meters"
                    }
                },
                "required": ["part_id", "position", "normal", "radius", "depth"]
            }),
        },
        // 6. modify_feature
        ToolDefinition {
            name: "modify_feature".to_string(),
            description: "Edits parameters of an existing step in the part's parametric history DAG without losing topology".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "feature_id": {
                        "type": "string",
                        "description": "UUID of the feature node to modify"
                    },
                    "params": {
                        "type": "object",
                        "description": "Updated geometric parameters"
                    }
                },
                "required": ["part_id", "feature_id", "params"]
            }),
        },
        // 7. get_feature_tree
        ToolDefinition {
            name: "get_feature_tree".to_string(),
            description: "Retrieves the full ordered parametric DAG history operations of a part".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 8. cut_helical_groove
        ToolDefinition {
            name: "cut_helical_groove".to_string(),
            description: "Cuts one or more helical grooves (drill flutes, thread channels, or cooling spirals) into a CAD part".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "helix_radius": {
                        "type": "number",
                        "description": "Radius of the helix center-line path in meters"
                    },
                    "groove_radius": {
                        "type": "number",
                        "description": "Radius of the cutter bead (groove cross-section radius) in meters"
                    },
                    "pitch": {
                        "type": "number",
                        "description": "Axial pitch (distance along Z per 360-degree revolution) in meters"
                    },
                    "height": {
                        "type": "number",
                        "description": "Total axial length/height of the groove sweep in meters"
                    },
                    "start_z": {
                        "type": "number",
                        "description": "Z coordinate where the helix starts in meters"
                    },
                    "starts": {
                        "type": "integer",
                        "description": "Number of interleaved helical flutes (e.g. 2 for standard twist drill, default 2)"
                    },
                    "is_right_hand": {
                        "type": "boolean",
                        "description": "True for right-hand helix (standard), false for left-hand (default true)"
                    }
                },
                "required": ["part_id", "helix_radius", "groove_radius", "pitch", "height", "start_z"]
            }),
        },
        // 9. boolean_difference
        ToolDefinition {
            name: "boolean_difference".to_string(),
            description: "Subtracts the geometry of a tool part from a target part (CSG Boolean Difference)".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "target_part_id": {
                        "type": "string",
                        "description": "ID of the part to be cut/modified"
                    },
                    "tool_part_id": {
                        "type": "string",
                        "description": "ID of the part whose geometry acts as the cutting tool"
                    }
                },
                "required": ["target_part_id", "tool_part_id"]
            }),
        },
        // 8. create_joint
        ToolDefinition {
            name: "create_joint".to_string(),
            description: "Establishes a kinematic or dynamic constraint between two parts in Rapier3D (revolute, prismatic, spherical, fixed)".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "joint_type": {
                        "type": "string",
                        "enum": ["revolute", "prismatic", "spherical", "fixed"],
                        "description": "Kinematic joint type"
                    },
                    "part_a": {
                        "type": "string",
                        "description": "ID of first connected part"
                    },
                    "part_b": {
                        "type": "string",
                        "description": "ID of second connected part"
                    },
                    "anchor_a": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Anchor point [x, y, z] on part A"
                    },
                    "anchor_b": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Anchor point [x, y, z] on part B"
                    },
                    "axis": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Joint rotation or sliding axis [x, y, z] (required for revolute/prismatic)"
                    },
                    "limits": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 2,
                        "maxItems": 2,
                        "description": "Lower and upper motion limits [min, max]"
                    }
                },
                "required": ["joint_type", "part_a", "part_b", "anchor_a", "anchor_b"]
            }),
        },
        // 9. set_joint_motor
        ToolDefinition {
            name: "set_joint_motor".to_string(),
            description: "Activates motor velocity control with a maximum force/torque limit on an existing kinematic joint".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "joint_id": {
                        "type": "string",
                        "description": "Identifier of the joint"
                    },
                    "target_velocity": {
                        "type": "number",
                        "description": "Desired angular (rad/s) or linear (m/s) speed"
                    },
                    "max_force": {
                        "type": "number",
                        "description": "Maximum motor torque (N*m) or force (N)"
                    }
                },
                "required": ["joint_id", "target_velocity", "max_force"]
            }),
        },
        // 10. run_physics_simulation
        ToolDefinition {
            name: "run_physics_simulation".to_string(),
            description: "Simulates mechanism dynamics forward in time, recording part trajectories, joint reactions, and collision events".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "duration_seconds": {
                        "type": "number",
                        "description": "Duration of physical simulation in seconds"
                    },
                    "step_size_ms": {
                        "type": "number",
                        "description": "Discrete integration step size in milliseconds (default 16.67 ms)"
                    }
                },
                "required": ["duration_seconds"]
            }),
        },
        // 11. run_fea_analysis
        ToolDefinition {
            name: "run_fea_analysis".to_string(),
            description: "Generates a 3D tetrahedral mesh (TET4), solves linear elasticity (K*u = F), and computes von Mises stresses and Factor of Safety (FoS)".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "ID of part to analyze"
                    },
                    "mesh_density": {
                        "type": "string",
                        "enum": ["coarse", "medium", "fine"],
                        "description": "Tetrahedral mesh resolution (default: coarse)"
                    },
                    "fixed_plane": {
                        "type": "object",
                        "properties": {
                            "origin": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 },
                            "normal": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 },
                            "tolerance": { "type": "number" }
                        },
                        "description": "Fixed boundary constraint plane"
                    },
                    "fixed_faces": {
                        "type": "array",
                        "items": { "type": "string" },
                        "description": "AABB faces to constrain (e.g. min_x, max_x, min_y, max_y, min_z, max_z)"
                    },
                    "applied_force": {
                        "type": "object",
                        "properties": {
                            "position": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 },
                            "force_vector": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 }
                        },
                        "description": "Single mechanical force load [Fx, Fy, Fz] in Newtons"
                    },
                    "applied_forces": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "position": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 },
                                "force_vector": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 }
                            }
                        },
                        "description": "Array of mechanical force loads"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 12. get_fea_heatmap_image
        ToolDefinition {
            name: "get_fea_heatmap_image".to_string(),
            description: "Renders an offscreen 3D view of the part colored with the FEA equivalent von Mises stress heatmap into PNG/Base64".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Part ID"
                    },
                    "camera_preset": {
                        "type": "string",
                        "enum": ["isometric", "iso", "front", "top", "right"],
                        "description": "Camera viewpoint preset"
                    },
                    "width": {
                        "type": "integer",
                        "description": "Image width in pixels (default 800)"
                    },
                    "height": {
                        "type": "integer",
                        "description": "Image height in pixels (default 600)"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 13. capture_viewport
        ToolDefinition {
            name: "capture_viewport".to_string(),
            description: "Captures a photorealistic offscreen 3D render of the current CAD assembly from a specified camera angle into PNG/Base64".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "camera_preset": {
                        "type": "string",
                        "enum": ["isometric", "iso", "front", "top", "right"],
                        "description": "Camera viewpoint preset"
                    },
                    "width": {
                        "type": "integer",
                        "description": "Image width in pixels (default 800)"
                    },
                    "height": {
                        "type": "integer",
                        "description": "Image height in pixels (default 600)"
                    },
                    "part_id": {
                        "type": "string",
                        "description": "Optional specific part ID to render"
                    }
                }
            }),
        },
        // 14. export_stl
        ToolDefinition {
            name: "export_stl".to_string(),
            description: "Exports a CAD part's surface mesh to an STL file (ASCII or compact Binary) for 3D printing and manufacturing".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "filepath": {
                        "type": "string",
                        "description": "Destination file path on disk (e.g. ./output/bracket.stl)"
                    },
                    "binary": {
                        "type": "boolean",
                        "description": "Export in compact Binary STL format if true; ASCII if false (default: true)"
                    }
                },
                "required": ["part_id", "filepath"]
            }),
        },
        // 15. get_simulation_metrics_json
        ToolDefinition {
            name: "get_simulation_metrics_json".to_string(),
            description: "Returns the comprehensive engineering metrics summary including mass, bounding boxes, FEA stresses, and physics telemetry".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {}
            }),
        },
        // 16. open_preview_window
        ToolDefinition {
            name: "open_preview_window".to_string(),
            description: "Opens or focuses the interactive native 3D CAD preview window on Windows, optionally displaying a specified part".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Optional ID of the part to focus and render in the preview window"
                    }
                }
            }),
        },
    ]
}

/// Dispatches an MCP tool call by name to the appropriate handler.
pub fn dispatch_tool(
    name: &str,
    args: &Value,
    state: &mut EngineState,
) -> Result<ToolCallResult, ServerError> {
    match name {
        "create_part" => {
            let res = cad::create_part(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "list_parts" => {
            let res = cad::list_parts(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "delete_part" => {
            let res = cad::delete_part(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "add_feature_primitive" => {
            let res = cad::add_feature_primitive(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "cut_hole" => {
            let res = cad::cut_hole(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "modify_feature" => {
            let res = cad::modify_feature(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "get_feature_tree" => {
            let res = cad::get_feature_tree(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "cut_helical_groove" => {
            let res = cad::cut_helical_groove(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "boolean_difference" => {
            let res = cad::boolean_difference(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "create_joint" => {
            let res = physics::create_joint(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "set_joint_motor" => {
            let res = physics::set_joint_motor(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "run_physics_simulation" => {
            let res = physics::run_physics_simulation(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "run_fea_analysis" => {
            let res = fea::run_fea_analysis(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "get_fea_heatmap_image" => {
            let res = fea::get_fea_heatmap_image(args, state)?;
            let b64 = res.get("image_base64").and_then(|v| v.as_str()).unwrap_or("").to_string();
            ToolCallResult::image_and_json(&res, b64).map_err(ServerError::Json)
        }
        "capture_viewport" => {
            let res = render::capture_viewport(args, state)?;
            let b64 = res.get("image_base64").and_then(|v| v.as_str()).unwrap_or("").to_string();
            ToolCallResult::image_and_json(&res, b64).map_err(ServerError::Json)
        }
        "export_stl" => {
            let res = export::export_stl(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "get_simulation_metrics_json" => {
            let res = metrics::get_simulation_metrics_json(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "open_preview_window" => {
            let res = render::open_preview_window(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        other => Err(ServerError::ToolNotFound(other.to_string())),
    }
}
