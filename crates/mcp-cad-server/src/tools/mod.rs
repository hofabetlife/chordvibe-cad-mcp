//! MCP tool registry, JSON schemas, and execution dispatcher.

pub mod cad;
pub mod fea;
pub mod metrics;
pub mod physics;
pub mod project;
pub mod render;
pub mod requirements;

use serde_json::{json, Value};

use crate::error::ServerError;
use crate::protocol::{ToolCallResult, ToolDefinition};
use crate::state::EngineState;

/// Returns the complete registry of all 34 MCP engineering tools with strict input schemas,
/// explicit unit contracts, usage examples, common errors, and recommended next steps.
pub fn list_tools() -> Vec<ToolDefinition> {
    vec![
        // 1. create_part
        ToolDefinition {
            name: "create_part".to_string(),
            description: "Creates a new CAD part assigned with an engineering material (e.g. aluminum_6061_t6 / alu_6061 / Al6061-T6, steel_structural_aisi1020 / steel_1045, steel_high_strength_4140 / steel_4140, titanium_grade5_ti6al4v / ti_6al_4v / Ti-6Al-4V, brass_c360, pla_filament_3dprint / pla_3d, petg_filament_3dprint / petg_3d, nylon_pa12 / nylon_pa6). Units: subsequent linear feature dimensions use active length_unit ('mm' by default, or 'm'); material density is in kg/m^3, yield strength in MPa, Young's modulus in GPa. Example: {\"name\": \"Support Bracket\", \"material\": \"aluminum_6061_t6\", \"part_id\": \"bracket_1\", \"is_fixed\": false}. Common errors: UNKNOWN_MATERIAL (material not in registry), PART_ALREADY_EXISTS (duplicate part_id), INVALID_ARGUMENT (empty name). Next step: call 'set_requirements' to declare design constraints or 'add_feature_primitive' to add 3D geometry.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "name": {
                        "type": "string",
                        "description": "Human-readable label for the part"
                    },
                    "material": {
                        "type": "string",
                        "description": "Identifier of material from registry or common alias (e.g. aluminum_6061_t6, alu_6061, Al6061-T6, steel_structural_aisi1020, steel_1045, steel_high_strength_4140, steel_4140, titanium_grade5_ti6al4v, ti_6al_4v, brass_c360, pla_filament_3dprint, petg_filament_3dprint, nylon_pa12)"
                    },
                    "part_id": {
                        "type": "string",
                        "description": "Optional custom part ID (default: auto-generated UUID)"
                    },
                    "is_fixed": {
                        "type": "boolean",
                        "description": "If true, fixes the part in 3D space during physics simulations (default: false)"
                    }
                },
                "required": ["name", "material"]
            }),
        },
        // 2. list_parts
        ToolDefinition {
            name: "list_parts".to_string(),
            description: "Lists all existing CAD parts in the engine scene. Units: bounding box dimensions in active length_unit ('mm' by default, or 'm'), volume in mm^3 or m^3 (plus volume_m3 in m^3), mass in kilograms (kg), and triangle count (dimensionless integer). Example: {}. Common errors: INVALID_ARGUMENT (invalid length_unit override; returns empty list if no parts exist). Next step: call 'query_part_geometry' to inspect named faces of a part or 'create_part' if the scene is empty.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    }
                }
            }),
        },
        // 3. delete_part
        ToolDefinition {
            name: "delete_part".to_string(),
            description: "Deletes a part from the scene, physics simulation world, requirements registry, and FEA analysis cache. Units: none (identifies part by string ID). Example: {\"part_id\": \"bracket_1\"}. Common errors: PART_NOT_FOUND (part_id does not exist in session). Next step: call 'list_parts' or 'get_scene_summary' to confirm remaining assembly parts.".to_string(),
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
            description: "Appends a 3D solid primitive (box, cylinder, sphere, cone, polygon_prism, spur_gear, revolve, or wheel_rim) into the parametric history DAG of a part. Coordinate conventions: all primitives are centered along Z in [-height/2, +height/2] with origin [0,0,0] (box: X in [-width/2, +width/2], Y in [-height/2, +height/2], Z in [-depth/2, +depth/2]; cone: base radius_bottom at Z=-height/2, apex radius_top at Z=+height/2). FeatureTree DAG semantics: subsequent add_feature_primitive calls on the same part_id AUTOMATICALLY perform csg_union with the existing solid body (they do not overwrite the part). Units: all linear dimensions, radii, module, and positions [x, y, z] are in active length_unit ('mm' by default, or 'm'); angles are in degrees (deg). Example: {\"part_id\": \"bracket_1\", \"primitive_type\": \"box\", \"params\": {\"width\": 100.0, \"height\": 20.0, \"depth\": 20.0, \"position\": [0.0, 0.0, 0.0]}}.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "primitive_type": {
                        "type": "string",
                        "enum": ["box", "cylinder", "sphere", "cone", "polygon_prism", "spur_gear", "revolve", "wheel_rim"],
                        "description": "Type of 3D primitive"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit, which is 'mm')"
                    },
                    "params": {
                        "type": "object",
                        "description": "Geometric parameters in active length_unit (mm by default, or m) and degrees (deg) (box: width, height, depth; cylinder: radius, height; sphere: radius; cone: radius_bottom, radius_top, height; polygon_prism: height, sides, width_across_flats, radius/outer_radius, chamfer/chamfer_radius; spur_gear: module, teeth, face_width, pressure_angle_deg, bore_radius, bore_diameter, keyway_width, keyway_depth, hub_diameter, hub_length; revolve: profile [[r,z],...], optional inner_radius for 2-point profile, angle_deg; wheel_rim: rim_diameter, rim_width, flange_diameter, hub_diameter, hub_bore_diameter, pcd_diameter, bolt_holes_count, bolt_hole_diameter, spokes_count, spoke_width, spoke_thickness, offset_et; optional position [x,y,z], rotation_deg [rx,ry,rz])",
                        "properties": {
                            "width": { "type": "number", "description": "Box width in active length_unit (mm by default, or m)" },
                            "height": { "type": "number", "description": "Height in active length_unit (mm by default, or m) (box, cylinder, cone, polygon_prism)" },
                            "depth": { "type": "number", "description": "Box depth in active length_unit (mm by default, or m)" },
                            "radius": { "type": "number", "description": "Radius in active length_unit (mm by default, or m) (cylinder, sphere, polygon_prism circumradius)" },
                            "outer_radius": { "type": "number", "description": "Outer circumradius alias for polygon_prism in active length_unit (mm by default, or m)" },
                            "radius_bottom": { "type": "number", "description": "Bottom radius in active length_unit (mm by default, or m) (cone)" },
                            "radius_top": { "type": "number", "description": "Top radius in active length_unit (mm by default, or m) (cone, 0 for sharp point)" },
                            "sides": { "type": "integer", "description": "Number of polygon facets (dimensionless integer, minimum 3, default 6 for hexagon)" },
                            "width_across_flats": { "type": "number", "description": "Width across flats (S, e.g. wrench size for nuts/bolts) in active length_unit (mm by default, or m)" },
                            "chamfer": { "type": "number", "description": "Optional conical end chamfer size (30-degree DIN 934 chamfer) in active length_unit (mm by default, or m)" },
                            "chamfer_radius": { "type": "number", "description": "Alias for chamfer size on polygon_prism in active length_unit (mm by default, or m)" },
                            "module": { "type": "number", "description": "Spur gear module (m) in active length_unit (mm by default, e.g. 2.0 for 2 mm module, or 0.002 when length_unit='m')" },
                            "teeth": { "type": "integer", "description": "Number of gear teeth (dimensionless integer z, e.g. 20)" },
                            "pressure_angle_deg": { "type": "number", "description": "Pressure angle in degrees (deg, default 20.0)" },
                            "face_width": { "type": "number", "description": "Gear face width (thickness along gear axis) in active length_unit (mm by default, or m)" },
                            "bore_radius": { "type": "number", "description": "Optional shaft bore hole radius in active length_unit (mm by default, or m)" },
                            "bore_diameter": { "type": "number", "description": "Optional shaft bore hole diameter in active length_unit (mm by default, or m)" },
                            "keyway_width": { "type": "number", "description": "Optional DIN 6885 keyway slot width in active length_unit (mm by default, or m)" },
                            "keyway_depth": { "type": "number", "description": "Optional DIN 6885 keyway slot depth into hub/bore in active length_unit (mm by default, or m)" },
                            "hub_diameter": { "type": "number", "description": "Optional hub diameter in active length_unit (mm by default, or m)" },
                            "hub_length": { "type": "number", "description": "Optional hub axial length in active length_unit (mm by default, or m)" },
                            "profile": {
                                "type": "array",
                                "items": {
                                    "type": "array",
                                    "items": { "type": "number" },
                                    "minItems": 2,
                                    "maxItems": 2
                                },
                                "description": "2D cross-section contour points [[r, z], ...] for revolve in active length_unit (mm by default, or m)"
                            },
                            "inner_radius": { "type": "number", "description": "Optional inner bore radius for revolve when profile specifies a 2-point outer generating line [[r0, z0], [r1, z1]] in active length_unit (mm by default, or m)" },
                            "angle_deg": { "type": "number", "description": "Revolution sweep angle in degrees (deg, default 360.0)" },
                            "rim_diameter": { "type": "number", "description": "Nominal wheel rim bead seat diameter in active length_unit (mm by default, e.g. 457.2 for 18-inch rim, or m)" },
                            "rim_width": { "type": "number", "description": "Wheel rim width in active length_unit (mm by default, e.g. 203.2 for 8-inch rim, or m)" },
                            "flange_diameter": { "type": "number", "description": "Optional outer rim flange diameter in active length_unit (mm by default, or m)" },
                            "hub_bore_diameter": { "type": "number", "description": "Optional central hub bore hole diameter in active length_unit (mm by default, or m)" },
                            "pcd_diameter": { "type": "number", "description": "Optional PCD (Pitch Circle Diameter) for bolt holes in active length_unit (mm by default, e.g. 114.3, or m)" },
                            "bolt_holes_count": { "type": "integer", "description": "Number of bolt holes on PCD pattern (dimensionless integer, default 5)" },
                            "bolt_hole_diameter": { "type": "number", "description": "Diameter of each bolt hole in active length_unit (mm by default, e.g. 14.0 for M14, or m)" },
                            "spokes_count": { "type": "integer", "description": "Number of wheel spokes (dimensionless integer, default 5)" },
                            "spoke_width": { "type": "number", "description": "Circumferential width of each spoke in active length_unit (mm by default, or m)" },
                            "spoke_thickness": { "type": "number", "description": "Axial thickness of each spoke in active length_unit (mm by default, or m)" },
                            "offset_et": { "type": "number", "description": "Wheel offset ET in active length_unit (mm by default, e.g. 35.0 for +35 mm, or m)" },
                            "segments": { "type": "integer", "description": "Radial segments (dimensionless integer, default 32)" },
                            "sectors": { "type": "integer", "description": "Sphere sectors (dimensionless integer, default 32)" },
                            "stacks": { "type": "integer", "description": "Sphere stacks (dimensionless integer, default 16)" },
                            "position": {
                                "type": "array",
                                "items": { "type": "number" },
                                "minItems": 3,
                                "maxItems": 3,
                                "description": "Optional center position [x, y, z] in active length_unit (mm by default, or m)"
                            },
                            "rotation_deg": {
                                "type": "array",
                                "items": { "type": "number" },
                                "minItems": 3,
                                "maxItems": 3,
                                "description": "Optional Euler rotation angles [rx, ry, rz] in degrees (deg)"
                            }
                        }
                    },
                    "position": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Optional position [x, y, z] in active length_unit (mm by default, or m)"
                    },
                    "rotation_deg": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Optional Euler rotation angles [rx, ry, rz] in degrees (deg)"
                    }
                },
                "required": ["part_id", "primitive_type", "params"]
            }),
        },
        // 5. cut_hole
        ToolDefinition {
            name: "cut_hole".to_string(),
            description: "Cuts a cylindrical hole through or into a CAD part along a specified direction normal vector, with optional DIN 6885 keyway slot, tagging cylindrical wall faces as 'hole_wall:feat_N'. Supports 'through': true to automatically drill completely through the part along the hole axis without requiring an explicit 'depth'. Units: position [x, y, z], radius (or diameter), depth, keyway_width, and keyway_depth are in active length_unit ('mm' by default, or 'm'); normal (or direction) [nx, ny, nz] is a dimensionless unit vector. Example: {\"part_id\": \"bracket_1\", \"position\": [0.0, 0.0, 0.0], \"normal\": [0.0, 0.0, -1.0], \"radius\": 5.0, \"through\": true}. Common errors: PART_NOT_FOUND, INVALID_ARGUMENT (radius <= 0, missing both depth and through: true, or zero normal vector), CSG_BOOLEAN_ERROR. Next step: call 'validate_mesh' to verify watertightness and 'query_part_geometry' to inspect the new 'hole_wall' face.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "position": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Hole center coordinates [x, y, z] in active length_unit (mm by default, or m)"
                    },
                    "normal": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Unit normal direction vector [nx, ny, nz] defining the hole axis (dimensionless)"
                    },
                    "direction": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Alias for 'normal': unit direction vector [dx, dy, dz] defining the hole axis (dimensionless)"
                    },
                    "radius": {
                        "type": "number",
                        "description": "Hole radius in active length_unit (mm by default, or m)"
                    },
                    "diameter": {
                        "type": "number",
                        "description": "Optional hole diameter alias (2 * radius) in active length_unit (mm by default, or m)"
                    },
                    "depth": {
                        "type": "number",
                        "description": "Hole depth in active length_unit (mm by default, or m; optional when 'through' is true)"
                    },
                    "through": {
                        "type": "boolean",
                        "description": "If true, cuts a full through-hole spanning the entire part thickness along normal/direction without requiring 'depth'"
                    },
                    "keyway_width": {
                        "type": "number",
                        "description": "Optional DIN 6885 keyway slot width in active length_unit (mm by default, or m)"
                    },
                    "keyway_depth": {
                        "type": "number",
                        "description": "Optional DIN 6885 keyway slot depth into wall in active length_unit (mm by default, or m)"
                    }
                },
                "required": ["part_id", "position"]
            }),
        },
        // 6. modify_feature
        ToolDefinition {
            name: "modify_feature".to_string(),
            description: "Edits parameters of an existing step in the part's parametric history DAG and rebuilds the 3D solid mesh, incrementing part.revision (which marks prior FEA results 'STALE'). Units: linear dimensions and positions in active length_unit ('mm' by default, or 'm'); angles in degrees (deg). Example: {\"part_id\": \"bracket_1\", \"feature_id\": \"00000000-0000-0000-0000-000000000001\", \"params\": {\"width\": 120.0, \"height\": 25.0, \"depth\": 20.0}}. Common errors: PART_NOT_FOUND, FEATURE_NOT_FOUND (unknown feature_id), INVALID_ARGUMENT (invalid parameter values). Next step: call 'validate_mesh', then re-run 'run_fea_analysis' if structural requirements are active, and 'check_requirements'.".to_string(),
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
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "params": {
                        "type": "object",
                        "description": "Updated geometric parameters in active length_unit (mm by default, or m) and degrees (deg)"
                    }
                },
                "required": ["part_id", "feature_id", "params"]
            }),
        },
        // 7. get_feature_tree
        ToolDefinition {
            name: "get_feature_tree".to_string(),
            description: "Retrieves the full ordered parametric DAG history of a part. Units: linear feature parameters are reported in active length_unit ('mm' by default, or 'm'), angles in degrees (deg). Example: {\"part_id\": \"bracket_1\"}. Common errors: PART_NOT_FOUND. Next step: call 'modify_feature' with a 'feature_id' from the returned nodes list or 'optimize_feature_tree'.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 8. cut_helical_groove
        ToolDefinition {
            name: "cut_helical_groove".to_string(),
            description: "Cuts one or more helical grooves (drill flutes, ISO metric thread channels, or cooling spirals) into a CAD part. Units: helix_radius, groove_radius, pitch, height, and start_z are in active length_unit ('mm' by default, or 'm'); starts is a dimensionless integer. Example: {\"part_id\": \"drill_1\", \"helix_radius\": 5.0, \"groove_radius\": 2.0, \"pitch\": 30.0, \"height\": 50.0, \"start_z\": -25.0, \"starts\": 2, \"profile_type\": \"circular\"}. Common errors: PART_NOT_FOUND, INVALID_ARGUMENT (non-positive radii, pitch, or height), CSG_BOOLEAN_ERROR. Next step: always call 'validate_mesh' to confirm the resulting mesh is watertight and manifold.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "helix_radius": {
                        "type": "number",
                        "description": "Radius of the helix center-line path in active length_unit (mm by default, or m)"
                    },
                    "groove_radius": {
                        "type": "number",
                        "description": "Radius of the cutter bead (groove cross-section radius) in active length_unit (mm by default, or m)"
                    },
                    "pitch": {
                        "type": "number",
                        "description": "Axial pitch (distance along Z per 360-degree revolution) in active length_unit (mm by default, or m)"
                    },
                    "height": {
                        "type": "number",
                        "description": "Total axial length/height of the groove sweep in active length_unit (mm by default, or m)"
                    },
                    "start_z": {
                        "type": "number",
                        "description": "Optional Z coordinate where the helix starts in active length_unit (mm by default, or m). If omitted, defaults to the bottom of the part (bounding_box.min[2], or -height/2 if empty)."
                    },
                    "starts": {
                        "type": "integer",
                        "description": "Number of interleaved helical flutes (dimensionless integer, e.g. 2 for standard twist drill, 1 for single-start metric_60 thread)"
                    },
                    "is_right_hand": {
                        "type": "boolean",
                        "description": "True for right-hand helix (standard), false for left-hand (default true)"
                    },
                    "profile_type": {
                        "type": "string",
                        "enum": ["circular", "metric_60"],
                        "description": "Cross-sectional cutter profile: 'circular' (default bead) or 'metric_60' (60-degree triangular metric thread profile per ISO 68-1)"
                    }
                },
                "required": ["part_id", "helix_radius", "groove_radius", "pitch", "height"]
            }),
        },
        // 9. boolean_difference
        ToolDefinition {
            name: "boolean_difference".to_string(),
            description: "Subtracts the 3D geometry of 'tool_part_id' from 'target_part_id' (CSG Boolean Difference), preserving named faces with 'cut:' prefix on cut surfaces. Units: output volume in active length_unit^3 (mm^3 or m^3) and mass in kg. Example: {\"target_part_id\": \"housing\", \"tool_part_id\": \"bore_cutter\"}. Common errors: PART_NOT_FOUND, EMPTY_GEOMETRY, CSG_BOOLEAN_ERROR. Next step: call 'validate_mesh' on 'target_part_id' to verify watertightness before FEA or export.".to_string(),
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
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    }
                },
                "required": ["target_part_id", "tool_part_id"]
            }),
        },
        // 10. create_joint
        ToolDefinition {
            name: "create_joint".to_string(),
            description: "Establishes a kinematic or dynamic constraint between two parts in Rapier3D (revolute, prismatic, spherical, fixed, gear). Accepts an optional custom 'joint_id' (or auto-generates 'joint-<uuid>'). Units: anchor_a and anchor_b are in active length_unit ('mm' by default, or 'm'); axis vectors, ratio, teeth_a, and teeth_b are dimensionless; limits [min, max] are in active length_unit (mm or m) for prismatic joints or degrees (deg) for revolute joints. Example: {\"joint_id\": \"j_crank\", \"joint_type\": \"revolute\", \"part_a\": \"ground\", \"part_b\": \"crank_arm\", \"anchor_a\": [0.0, 0.0, 0.0], \"anchor_b\": [0.0, 0.0, 0.0], \"axis\": [0.0, 0.0, 1.0]}. Common errors: PART_NOT_FOUND, JOINT_ALREADY_EXISTS (duplicate joint_id), EMPTY_GEOMETRY (part has 0 vertices), INVALID_ARGUMENT (connecting a part to itself or empty joint_id), INVALID_JOINT_CONFIGURATION (zero axis vector or min > max limits). Next step: call 'set_joint_motor' to actuate the joint or 'run_physics_simulation'.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "joint_id": {
                        "type": "string",
                        "description": "Optional custom joint identifier (default: auto-generated 'joint-<uuid>')"
                    },
                    "joint_type": {
                        "type": "string",
                        "enum": ["revolute", "prismatic", "spherical", "fixed", "gear"],
                        "description": "Kinematic joint type"
                    },
                    "part_a": {
                        "type": "string",
                        "description": "ID of first connected part (or 'ground' / 'world')"
                    },
                    "part_b": {
                        "type": "string",
                        "description": "ID of second connected part (or 'ground' / 'world')"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "anchor_a": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Anchor point [x, y, z] on part A in active length_unit (mm by default, or m; default: [0, 0, 0])"
                    },
                    "anchor_b": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Anchor point [x, y, z] on part B in active length_unit (mm by default, or m; default: [0, 0, 0])"
                    },
                    "axis": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Unit direction vector [x, y, z] for joint rotation or sliding axis (dimensionless, for revolute/prismatic/gear)"
                    },
                    "axis_a": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Unit rotation axis [x, y, z] for gear A in a gear joint (dimensionless)"
                    },
                    "axis_b": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Unit rotation axis [x, y, z] for gear B in a gear joint (dimensionless)"
                    },
                    "ratio": {
                        "type": "number",
                        "description": "Gear transmission ratio (omega_a / omega_b, dimensionless) for gear joints"
                    },
                    "teeth_a": {
                        "type": "number",
                        "exclusiveMinimum": 0.0,
                        "description": "Positive tooth count (> 0) of gear A (dimensionless, used to compute ratio = teeth_a / teeth_b if ratio is omitted)"
                    },
                    "teeth_b": {
                        "type": "number",
                        "exclusiveMinimum": 0.0,
                        "description": "Positive tooth count (> 0) of gear B (dimensionless, used to compute ratio = teeth_a / teeth_b if ratio is omitted)"
                    },
                    "limits": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 2,
                        "maxItems": 2,
                        "description": "Lower and upper motion limits [min, max]: in active length_unit (mm or m) for prismatic joints, or degrees (deg) for revolute joints"
                    }
                },
                "required": ["joint_type", "part_a", "part_b"]
            }),
        },
        // 11. set_joint_motor
        ToolDefinition {
            name: "set_joint_motor".to_string(),
            description: "Activates velocity motor control with a maximum force/torque limit on an existing revolute or prismatic joint. Units: target_velocity is in radians per second (rad/s) for revolute joints or meters per second (m/s) for prismatic joints; max_force (>= 0) is torque in Newton-meters (N*m) for revolute joints or force in Newtons (N) for prismatic joints. Example: {\"joint_id\": \"joint-123\", \"target_velocity\": 10.0, \"max_force\": 50.0}. Common errors: JOINT_NOT_FOUND, INVALID_ARGUMENT (non-finite target_velocity or max_force < 0), INVALID_JOINT_CONFIGURATION (motors are not supported on spherical, fixed, or gear joints). Next step: call 'run_physics_simulation' to simulate mechanism motion.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "joint_id": {
                        "type": "string",
                        "description": "Identifier of the joint"
                    },
                    "target_velocity": {
                        "type": "number",
                        "description": "Desired finite angular velocity in radians per second (rad/s) for revolute joints, or linear velocity in meters per second (m/s) for prismatic joints"
                    },
                    "max_force": {
                        "type": "number",
                        "minimum": 0.0,
                        "description": "Maximum motor torque (>= 0) in Newton-meters (N*m) for revolute joints, or force in Newtons (N) for prismatic joints"
                    }
                },
                "required": ["joint_id", "target_velocity", "max_force"]
            }),
        },
        // 12. run_physics_simulation
        ToolDefinition {
            name: "run_physics_simulation".to_string(),
            description: "Steps the Rapier3D multi-body physics simulation forward in time, recording trajectories, joint reactions, and collisions. Units: duration_seconds in seconds (s), step_size_ms in milliseconds (ms); output positions in active length_unit (mm or m), angles in degrees (deg), linear velocities in m/s, angular velocities in rad/s, forces in N, torques in N*m. Example: {\"duration_seconds\": 1.0, \"step_size_ms\": 16.67}. Common errors: INVALID_ARGUMENT (duration_seconds <= 0 or step_size_ms <= 0), PHYSICS_SIMULATION_ERROR. Next step: call 'get_simulation_metrics_json' or 'get_scene_summary' to inspect telemetry.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "duration_seconds": {
                        "type": "number",
                        "exclusiveMinimum": 0.0,
                        "description": "Duration of physical simulation in seconds (s, must be > 0)"
                    },
                    "step_size_ms": {
                        "type": "number",
                        "exclusiveMinimum": 0.0,
                        "description": "Discrete integration step size in milliseconds (ms, must be > 0, default 16.67 ms)"
                    }
                },
                "required": ["duration_seconds"]
            }),
        },
        // 13. run_fea_analysis
        ToolDefinition {
            name: "run_fea_analysis".to_string(),
            description: "Generates a 3D tetrahedral mesh (TET4), solves linear elasticity (K*u = F), and computes von Mises stresses, displacements, support reaction equilibrium, and Factor of Safety (FoS). Supports optional multi-level mesh convergence study ('convergence_study': true across Coarse->Medium->Fine with Richardson extrapolation and 'accuracy' report). Units: boundary plane/load coordinates and tolerances use active length_unit ('mm' by default, or 'm'); forces use Newtons (N), pressures use Megapascals (MPa), moments use Newton-meters (N*m); output max_von_mises_mpa and yield_strength_mpa in MPa, max_displacement_mm in mm, safety_factor dimensionless, max_seconds in seconds (s). Example: {\"part_id\": \"bracket_1\", \"fea_case\": \"tip_load\", \"mesh_density\": \"coarse\", \"convergence_study\": true, \"fixed\": [{\"face\": \"left\"}], \"loads\": [{\"face\": \"right\", \"force\": [0.0, -500.0, 0.0]}]}. Common errors: PART_NOT_FOUND, EMPTY_GEOMETRY, MESH_INVALID, FACE_NOT_FOUND, AMBIGUOUS_FACE_SELECTOR, FEA_UNCONSTRAINED, FEA_NO_LOADS, FEA_NO_BOUNDARY_NODES. Next step: call 'check_requirements' to verify min_safety_factor or 'get_fea_heatmap_image' to visualize stress distribution.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "ID of part to analyze"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "mesh_density": {
                        "type": "string",
                        "enum": ["coarse", "medium", "fine"],
                        "description": "Tetrahedral mesh resolution when convergence_study is false (default: coarse)"
                    },
                    "element_order": {
                        "type": "integer",
                        "enum": [1, 2],
                        "description": "Finite element polynomial order: 1 for linear 4-node tetrahedra (TET4, default) or 2 for quadratic 10-node tetrahedra (TET10, eliminates shear locking in bending)"
                    },
                    "convergence_study": {
                        "type": "boolean",
                        "description": "When true, runs a multi-level mesh convergence study (default levels: ['coarse', 'medium', 'fine']), computes relative stress/displacement changes and Richardson extrapolation (where applicable), and populates the 'accuracy' block with verdict 'CONVERGED' or 'NOT_CONVERGED'"
                    },
                    "levels": {
                        "type": "array",
                        "items": {
                            "type": "string",
                            "enum": ["coarse", "medium", "fine"]
                        },
                        "minItems": 2,
                        "description": "Optional ordered list of mesh density levels for convergence_study (default: ['coarse', 'medium', 'fine'])"
                    },
                    "max_seconds": {
                        "type": "number",
                        "minimum": 0.0,
                        "description": "Optional wall-clock time budget in seconds for convergence_study; if exceeded, stops remaining levels and returns a partial result with accuracy.partial_result = true"
                    },
                    "fea_case": {
                        "type": "string",
                        "description": "Optional load-case identifier (e.g. 'bending_load_1') stored alongside the FEA report for requirement verification via min_safety_factor"
                    },
                    "fixed": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "face": { "type": "string", "description": "Named face, alias, role, or wildcard pattern (e.g. 'cap_bottom', 'hole_wall:*')" },
                                "face_id": { "type": "integer", "description": "Numeric face ID (dimensionless integer identifier) from query_part_geometry" },
                                "faces": { "type": "array", "items": { "type": "string" }, "description": "Multiple face names/aliases to constrain" },
                                "where": { "type": "object", "description": "Semantic/geometric face filter (kind, role, normal, axis, radius, near, closest)" },
                                "components": { "type": "string", "enum": ["all", "x", "y", "z", "xy", "xz", "yz"], "description": "Constrained displacement DOF components (default: 'all')" },
                                "tolerance": { "type": "number", "description": "Optional node capture tolerance in active length_unit (mm or m)" }
                            }
                        },
                        "description": "Dirichlet boundary constraints by named CAD faces or semantic face selectors"
                    },
                    "fixed_plane": {
                        "type": "object",
                        "properties": {
                            "origin": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3, "description": "Plane origin [x, y, z] in active length_unit (mm by default, or m)" },
                            "normal": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3, "description": "Unit normal vector [nx, ny, nz] (dimensionless)" },
                            "tolerance": { "type": "number", "description": "Plane capture distance tolerance in active length_unit (mm by default, or m)" }
                        },
                        "description": "Fixed boundary constraint plane (origin and tolerance in active length_unit: mm or m)"
                    },
                    "fixed_faces": {
                        "type": "array",
                        "items": { "type": "string" },
                        "description": "Faces to constrain: AABB face names (min_x, max_x, min_y, max_y, min_z, max_z) or named CAD faces (e.g. 'cap_bottom', 'left')"
                    },
                    "loads": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "face": { "type": "string", "description": "Target face name, alias, role, or wildcard pattern (e.g. 'cap_top', 'hole_wall')" },
                                "face_id": { "type": "integer", "description": "Numeric face ID (dimensionless integer identifier) from query_part_geometry" },
                                "faces": { "type": "array", "items": { "type": "string" }, "description": "Multiple target face names/aliases" },
                                "where": { "type": "object", "description": "Semantic/geometric face filter (kind, role, normal, axis, radius, near, closest)" },
                                "force": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3, "description": "Total force vector [Fx, Fy, Fz] in Newtons (N), distributed by tributary area" },
                                "pressure": { "type": "number", "description": "Normal pressure in Megapascals (MPa; positive pushes into the surface along -normal)" },
                                "moment": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3, "description": "Moment/torque vector [Mx, My, Mz] in N*m about the face centroid" },
                                "tolerance": { "type": "number", "description": "Optional node capture tolerance in active length_unit (mm or m)" }
                            }
                        },
                        "description": "Mechanical loads (force in N, pressure in MPa, or moment in N*m) applied to named CAD faces or face selectors"
                    },
                    "applied_force": {
                        "type": "object",
                        "properties": {
                            "face": { "type": "string", "description": "Optional named face to apply force to" },
                            "position": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3, "description": "Force application point [x, y, z] in active length_unit (mm by default, or m)" },
                            "force_vector": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3, "description": "Force vector [Fx, Fy, Fz] in Newtons (N)" }
                        },
                        "description": "Single mechanical force load: position in active length_unit (mm or m) or named face, force_vector [Fx, Fy, Fz] in Newtons (N)"
                    },
                    "applied_forces": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "face": { "type": "string", "description": "Optional named face to apply force to" },
                                "position": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3, "description": "Force application point [x, y, z] in active length_unit (mm by default, or m)" },
                                "force_vector": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3, "description": "Force vector [Fx, Fy, Fz] in Newtons (N)" }
                            }
                        },
                        "description": "Array of mechanical force loads (positions in active length_unit: mm or m, or named faces, force vectors in Newtons: N)"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 14. get_fea_heatmap_image
        ToolDefinition {
            name: "get_fea_heatmap_image".to_string(),
            description: "Renders an offscreen 3D view of the part colored with the latest FEA von Mises stress heatmap into PNG/Base64. Units: image width and height in pixels (px, default 800x600); stress scale in Megapascals (MPa). Example: {\"part_id\": \"bracket_1\", \"camera_preset\": \"isometric\", \"width\": 800, \"height\": 600}. Common errors: PART_NOT_FOUND, INVALID_ARGUMENT (run_fea_analysis has not been executed for this part yet), INVALID_DIMENSIONS. Next step: call 'modify_feature' to reinforce high-stress regions or 'check_requirements'.".to_string(),
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
                        "description": "Image width in pixels (px, default 800)"
                    },
                    "height": {
                        "type": "integer",
                        "description": "Image height in pixels (px, default 600)"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 15. capture_viewport
        ToolDefinition {
            name: "capture_viewport".to_string(),
            description: "Captures an offscreen 3D render of the current CAD assembly or a specific part into PNG/Base64. Units: width and height in pixels (px, default 800x600). Example: {\"part_id\": \"bracket_1\", \"camera_preset\": \"isometric\", \"width\": 800, \"height\": 600, \"debug_mode\": \"none\"}. Common errors: PART_NOT_FOUND, VIEWER_EMPTY_MESH (no non-empty geometry to render), INVALID_DIMENSIONS. Next step: inspect visual output or call 'export_stl' / 'save_project'.".to_string(),
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
                        "description": "Image width in pixels (px, default 800)"
                    },
                    "height": {
                        "type": "integer",
                        "description": "Image height in pixels (px, default 600)"
                    },
                    "part_id": {
                        "type": "string",
                        "description": "Optional specific part ID to render"
                    },
                    "debug_mode": {
                        "type": "string",
                        "enum": ["none", "wireframe", "normals", "backfaces", "boundary_edges", "flat_shading"],
                        "description": "Optional debug rendering mode for visual inspection and defect audit"
                    }
                }
            }),
        },
        // 16. export_stl
        ToolDefinition {
            name: "export_stl".to_string(),
            description: "Exports a CAD part's surface mesh to an STL file (ASCII or compact Binary) after verifying mesh validity. Units: chord_tolerance input is in active length_unit ('mm' by default, or 'm'); STL vertex coordinates written to disk are in internal SI meters (m, stl_vertex_unit='m'). Example: {\"part_id\": \"bracket_1\", \"filepath\": \"./output/bracket.stl\", \"binary\": true, \"quality\": \"fine\"}. Common errors: PART_NOT_FOUND, EMPTY_GEOMETRY, MESH_INVALID (export is blocked if validate_mesh fails), IO_ERROR. Next step: call 'save_project' to persist the parametric DAG alongside the STL.".to_string(),
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
                    },
                    "quality": {
                        "type": "string",
                        "enum": ["coarse", "medium", "fine"],
                        "description": "Tessellation quality preset ('coarse', 'medium', 'fine', default: 'fine')"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "chord_tolerance": {
                        "type": "number",
                        "description": "Optional maximum chordal deviation (sagitta tolerance) in active length_unit (mm by default, e.g. 0.05 for 0.05 mm, or m)"
                    }
                },
                "required": ["part_id", "filepath"]
            }),
        },
        // 17. get_simulation_metrics_json
        ToolDefinition {
            name: "get_simulation_metrics_json".to_string(),
            description: "Returns the comprehensive engineering metrics report across all parts, FEA reports, and physics simulations. Units: bounding boxes and trajectories in active length_unit (mm or m), volume in mm^3 or m^3, mass in kg, FEA stresses in MPa, displacements in mm, time in s/ms. Example: {}. Common errors: none. Next step: call 'get_scene_summary' for a compact overview or 'check_requirements' to verify design targets.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    }
                }
            }),
        },
        // 18. open_preview_window
        ToolDefinition {
            name: "open_preview_window".to_string(),
            description: "Opens or focuses the interactive native 3D CAD preview window, optionally displaying a specified part. When running in a standard headless stdio session (without --gui), automatically exports a live binary STL file and spawns a background interactive 3D window (mcp-cad-server --stl <path>). Units: none (identifies optional part by string ID). Example: {\"part_id\": \"bracket_1\"}.".to_string(),
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
        // 19. import_stl
        ToolDefinition {
            name: "import_stl".to_string(),
            description: "Imports an external 3D STL file (ASCII or binary) from disk into the CAD scene as a new Part. Units: STL file vertex coordinates on disk are interpreted in SI meters (m); returned bounding_box and volume are converted to active length_unit (mm or m), mass in kg. Example: {\"filepath\": \"./output/bracket.stl\", \"part_id\": \"bracket_1\", \"is_fixed\": false, \"material\": \"steel_structural_aisi1020\"}. Common errors: IO_ERROR (file not found), UNKNOWN_MATERIAL, CAD_EVALUATION_ERROR (malformed STL), PART_ALREADY_EXISTS. Next step: call 'validate_mesh' and 'query_part_geometry' on the imported part_id.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "filepath": {
                        "type": "string",
                        "description": "Path to the .stl file on disk (or alias 'file_path')"
                    },
                    "file_path": {
                        "type": "string",
                        "description": "Path to the .stl file on disk (alias for 'filepath')"
                    },
                    "part_id": {
                        "type": "string",
                        "description": "Optional custom part identifier (default: auto-generated UUID)"
                    },
                    "name": {
                        "type": "string",
                        "description": "Optional custom name for the part (defaults to file name without extension)"
                    },
                    "material": {
                        "type": "string",
                        "description": "Optional material identifier from registry (defaults to 'steel_structural_aisi1020')"
                    },
                    "is_fixed": {
                        "type": "boolean",
                        "description": "Whether the part is anchored/fixed in physics simulation (default: false)"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    }
                }
            }),
        },
        // 20. tessellate_part
        ToolDefinition {
            name: "tessellate_part".to_string(),
            description: "Evaluates adaptive tessellation on a CAD part with a quality preset or custom tolerances, updating the cached polygon mesh. Units: chord_tolerance and max_edge_length in active length_unit ('mm' by default, or 'm'); angular_tolerance_deg in degrees (deg). Example: {\"part_id\": \"bracket_1\", \"quality\": \"fine\", \"chord_tolerance\": 0.05, \"angular_tolerance_deg\": 5.0}. Common errors: PART_NOT_FOUND, INVALID_ARGUMENT (non-positive tolerances). Next step: call 'validate_mesh' or 'export_stl'.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID"
                    },
                    "quality": {
                        "type": "string",
                        "enum": ["coarse", "medium", "fine"],
                        "description": "Optional tessellation quality preset ('coarse', 'medium', 'fine', default: 'medium')"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "chord_tolerance": {
                        "type": "number",
                        "description": "Optional maximum chordal deviation (sagitta error) in active length_unit (mm by default, or m)"
                    },
                    "angular_tolerance_deg": {
                        "type": "number",
                        "description": "Optional maximum angular deflection between adjacent segments in degrees (deg)"
                    },
                    "max_edge_length": {
                        "type": "number",
                        "description": "Optional maximum polygon edge length in active length_unit (mm by default, or m)"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 21. query_part_geometry
        ToolDefinition {
            name: "query_part_geometry".to_string(),
            description: "Inspects detailed 3D geometry and topological named faces ('faces' registry) of a part, with optional semantic filtering via 'face_selector' or 'where'. Units: bounding box, dimensions, center_of_mass, face centroids, sample_points, and radii in active length_unit ('mm' by default, or 'm'); surface_area and face areas in mm^2 or m^2; volume in mm^3 or m^3; mass in kg. Example: {\"part_id\": \"bracket_1\", \"where\": {\"kind\": \"plane\"}}. Common errors: PART_NOT_FOUND, FACE_NOT_FOUND (when face_selector matches no faces; returns available_faces in error.details), AMBIGUOUS_FACE_SELECTOR. Next step: use the returned face names (e.g. 'cap_bottom', 'left', 'hole_wall:feat_1') in 'run_fea_analysis' or 'transform_part'.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Identifier of the CAD part to inspect"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "face_selector": {
                        "description": "Optional face selector to filter the returned 'faces' list (string face name/alias, {'face': name}, {'face_id': id}, {'faces': [...]}, or {'where': {'kind', 'radius', 'tolerance', 'normal', 'axis', 'near', 'closest', 'origin_feature', 'role', 'min_area', 'max_area'}})"
                    },
                    "where": {
                        "type": "object",
                        "description": "Optional geometric/topological filter shorthand for 'faces' (same as face_selector.where)"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 22. boolean_union
        ToolDefinition {
            name: "boolean_union".to_string(),
            description: "Performs a CSG boolean union combining the 3D solid geometry of 'tool_part_id' into 'target_part_id'. Units: resulting volume in active length_unit^3 (mm^3 or m^3) and mass in kg. Example: {\"target_part_id\": \"flange_base\", \"tool_part_id\": \"boss_cyl\"}. Common errors: PART_NOT_FOUND, EMPTY_GEOMETRY, CSG_BOOLEAN_ERROR. Next step: always call 'validate_mesh' on 'target_part_id' to verify watertightness.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "target_part_id": {
                        "type": "string",
                        "description": "ID of the target part that receives the union operation"
                    },
                    "tool_part_id": {
                        "type": "string",
                        "description": "ID of the tool part whose geometry will be united with the target part"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    }
                },
                "required": ["target_part_id", "tool_part_id"]
            }),
        },
        // 23. boolean_intersection
        ToolDefinition {
            name: "boolean_intersection".to_string(),
            description: "Performs a CSG boolean intersection keeping only the overlapping 3D solid volume between 'target_part_id' and 'tool_part_id'. Units: resulting volume in active length_unit^3 (mm^3 or m^3) and mass in kg. Example: {\"target_part_id\": \"stock_block\", \"tool_part_id\": \"envelope_sphere\"}. Common errors: PART_NOT_FOUND, EMPTY_GEOMETRY, CSG_BOOLEAN_ERROR. Next step: always call 'validate_mesh' on 'target_part_id' to verify the resulting mesh.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "target_part_id": {
                        "type": "string",
                        "description": "ID of the target part that receives the intersection operation"
                    },
                    "tool_part_id": {
                        "type": "string",
                        "description": "ID of the tool part whose geometry will be intersected with the target part"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    }
                },
                "required": ["target_part_id", "tool_part_id"]
            }),
        },
        // 24. transform_part
        ToolDefinition {
            name: "transform_part".to_string(),
            description: "Applies translation, Euler RPY rotation, dimensionless scaling, or named-face alignment ('pivot_face', 'align_face') to a part's parametric DAG. Units: translation [x, y, z] and align_face.target_point [x, y, z] in active length_unit ('mm' by default, or 'm'); rotation_deg [rx, ry, rz] in degrees (deg); scale [sx, sy, sz] dimensionless. Example: {\"part_id\": \"bracket_1\", \"translation\": [10.0, 0.0, 5.0], \"rotation_deg\": [0.0, 0.0, 90.0], \"scale\": [1.0, 1.0, 1.0]}. Common errors: PART_NOT_FOUND, FACE_NOT_FOUND, INVALID_ARGUMENT (zero scale factor). Next step: call 'query_part_geometry' to verify updated bounding box and face centroids.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Identifier of the part to transform"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "translation": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Translation vector [x, y, z] in active length_unit (mm by default, or m; default: [0, 0, 0])"
                    },
                    "rotation_deg": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Euler rotation angles [roll_x, pitch_y, yaw_z] in degrees (deg, default: [0, 0, 0])"
                    },
                    "scale": {
                        "type": "array",
                        "items": { "type": "number" },
                        "minItems": 3,
                        "maxItems": 3,
                        "description": "Dimensionless scaling factors [sx, sy, sz] along primary axes (default: [1, 1, 1])"
                    },
                    "pivot_face": {
                        "description": "Optional face selector (name/alias or selector object) whose centroid is used as the rotation/scaling pivot point"
                    },
                    "align_face": {
                        "type": "object",
                        "properties": {
                            "face": {
                                "description": "Face selector (name/alias or selector object) to align"
                            },
                            "target_point": {
                                "type": "array",
                                "items": { "type": "number" },
                                "minItems": 3,
                                "maxItems": 3,
                                "description": "Target point [x, y, z] in active length_unit (mm by default, or m) where the selected face's centroid should be placed"
                            }
                        },
                        "description": "Optional face alignment directive that translates the part so the selected face's centroid lands on target_point"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 25. define_material
        ToolDefinition {
            name: "define_material".to_string(),
            description: "Registers a custom engineering material into the session material registry for mass and FEA calculations. Units: young_modulus_gpa in Gigapascals (GPa, > 0), poisson_ratio dimensionless in (0.0, 0.5), yield_strength_mpa in Megapascals (MPa, > 0), density_kg_m3 in kilograms per cubic meter (kg/m^3, > 0). Example: {\"id\": \"carbon_fiber_t700\", \"name\": \"Carbon Fiber T700\", \"young_modulus_gpa\": 150.0, \"poisson_ratio\": 0.28, \"yield_strength_mpa\": 600.0, \"density_kg_m3\": 1550.0}. Common errors: INVALID_ARGUMENT (E <= 0, nu outside (0, 0.5), yield <= 0, or density <= 0). Next step: call 'create_part' referencing the new material 'id'.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "id": {
                        "type": "string",
                        "description": "Unique identifier for the engineering material (e.g. 'carbon_fiber_t700')"
                    },
                    "name": {
                        "type": "string",
                        "description": "Optional human-readable display name or description of the material"
                    },
                    "young_modulus_gpa": {
                        "type": "number",
                        "description": "Young's modulus of elasticity in Gigapascals (GPa, must be > 0)"
                    },
                    "poisson_ratio": {
                        "type": "number",
                        "description": "Poisson's ratio (dimensionless, must be between 0.0 and 0.5)"
                    },
                    "yield_strength_mpa": {
                        "type": "number",
                        "description": "Yield strength in Megapascals (MPa, must be > 0)"
                    },
                    "density_kg_m3": {
                        "type": "number",
                        "description": "Mass density in kilograms per cubic meter (kg/m³, must be > 0)"
                    }
                },
                "required": [
                    "id",
                    "young_modulus_gpa",
                    "poisson_ratio",
                    "yield_strength_mpa",
                    "density_kg_m3"
                ]
            }),
        },
        // 26. batch_cad_operations
        ToolDefinition {
            name: "batch_cad_operations".to_string(),
            description: "Executes a sequence of MCP CAD operations atomically in a single transaction; if any step fails, rolls back the scene state. Units: each sub-operation uses the session length_unit ('mm' by default, or 'm') and standard physical units (N, MPa, GPa, kg, deg). Example: {\"operations\": [{\"tool\": \"create_part\", \"params\": {\"part_id\": \"p1\", \"name\": \"Pin\", \"material\": \"steel_structural_aisi1020\"}}, {\"tool\": \"add_feature_primitive\", \"params\": {\"part_id\": \"p1\", \"primitive_type\": \"cylinder\", \"params\": {\"radius\": 5.0, \"height\": 40.0}}}]}. Common errors: INVALID_ARGUMENT (nested batch_cad_operations or failure in step i). Next step: call 'validate_mesh' and 'get_scene_summary' to inspect the resulting assembly.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "operations": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "tool": {
                                    "type": "string",
                                    "description": "Name of the MCP tool to execute"
                                },
                                "params": {
                                    "type": "object",
                                    "description": "Parameters payload passed to the tool"
                                }
                            },
                            "required": ["tool", "params"]
                        },
                        "description": "List of MCP CAD tool operations to execute sequentially"
                    }
                },
                "required": ["operations"]
            }),
        },
        // 27. validate_mesh
        ToolDefinition {
            name: "validate_mesh".to_string(),
            description: "Performs topological and geometric diagnostics on a part's mesh (watertightness, manifoldness, outward winding, Euler characteristic, genus, sliver triangles, T-junctions, and 3D self-intersections). In default mode (strict_slivers: false), micro-slivers on complex flutes/threads report as non-fatal warnings while solid status remains PASS (is_valid_solid: true, fea_ready: true). Set strict_slivers: true for strict primitive verification. Units: expected_genus is a dimensionless integer. Example: {\"part_id\": \"bracket_1\", \"expected_genus\": 1}.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Identifier of the part to validate"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    },
                    "expected_genus": {
                        "type": "integer",
                        "description": "Optional expected topological genus g = (2 - chi) / 2 (dimensionless integer, e.g. 0 for sphere/box, 1 for torus/through-hole)"
                    },
                    "strict_slivers": {
                        "type": "boolean",
                        "description": "Optional strict sliver quality gate enforcement (default: false). When false, sliver micro-triangles report as warnings with status 'PASS'. When true, slivers cause status 'FAIL'."
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 28. optimize_feature_tree
        ToolDefinition {
            name: "optimize_feature_tree".to_string(),
            description: "Runs the Cost-Based CAD Query Optimizer (CBO) on a part's FeatureTree (DAG), applying algebraic CSG rewrite rules (R1-R5), AABB dead-operation culling, and coaxial revolve reduction (R4) while verifying volumetric (<= 1%) and topological equivalence. Units: reported volume in active length_unit^3 (mm^3 or m^3) and execution cost in milliseconds (ms). Example: {\"part_id\": \"bushing_1\", \"apply\": true}. Common errors: PART_NOT_FOUND, CAD_EVALUATION_ERROR. Next step: call 'validate_mesh' or 'get_feature_tree' to inspect the simplified DAG.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Identifier of the part whose FeatureTree (DAG) should be optimized"
                    },
                    "apply": {
                        "type": "boolean",
                        "description": "Whether to apply the optimal physical execution plan to the part's DAG and rebuild its mesh (default: true)"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
                    }
                },
                "required": ["part_id"]
            }),
        },
        // 29. set_units
        ToolDefinition {
            name: "set_units".to_string(),
            description: "Sets the active linear length unit ('mm' or 'm') for the MCP session. Units: length_unit selects millimeters ('mm', default) or meters ('m') for all linear inputs/outputs, while force (N), moment (N*m), stress (MPa), Young's modulus (GPa), mass (kg), and angles (deg) remain invariant. Example: {\"length_unit\": \"mm\"}. Common errors: INVALID_ARGUMENT (unsupported unit string outside 'mm' or 'm'). Next step: call 'create_part' or 'get_scene_summary'.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Linear dimension unit for the session: 'mm' (millimeters, default) or 'm' (meters)"
                    }
                },
                "required": ["length_unit"]
            }),
        },
        // 30. save_project
        ToolDefinition {
            name: "save_project".to_string(),
            description: "Saves the complete CAD engine state (parts with parametric DAG histories, custom materials, physics joints with motors, engineering requirements, and active length_unit) to a versioned JSON project file (schema_version='1.0.0'). Units: file_size_bytes is reported in bytes (dimensionless integer); internal DAG parameters are stored in SI meters (m). Example: {\"path\": \"./output/assembly.json\", \"project_name\": \"Gearbox Assembly\"}. Common errors: INVALID_ARGUMENT (empty path), IO_ERROR (filesystem write failure). Next step: call 'load_project' to restore the project in a future session.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "path": {
                        "type": "string",
                        "description": "Filesystem destination path for the project JSON file (e.g. './output/my_assembly.json')"
                    },
                    "project_name": {
                        "type": "string",
                        "description": "Optional human-readable project title or description (default: 'MCP-CAD Project')"
                    }
                },
                "required": ["path"]
            }),
        },
        // 31. load_project
        ToolDefinition {
            name: "load_project".to_string(),
            description: "Atomically loads a CAD project from a versioned JSON file (schema_version='1.0.0'), rebuilding 3D meshes from parametric DAGs and restoring materials, joints, requirements, and length_unit. Units: returned part volumes use the project's length_unit^3 (mm^3 or m^3) and masses use kilograms (kg). Example: {\"path\": \"./output/assembly.json\"}. Common errors: PROJECT_SCHEMA_MISMATCH (unsupported schema_version), IO_ERROR (missing file), JSON_ERROR (corrupted JSON; preserves existing session state). Next step: call 'get_scene_summary' or 'check_requirements' to inspect the loaded assembly.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "path": {
                        "type": "string",
                        "description": "Filesystem path to the project JSON file to load"
                    }
                },
                "required": ["path"]
            }),
        },
        // 32. set_requirements
        ToolDefinition {
            name: "set_requirements".to_string(),
            description: "Declares engineering design requirements for a specific CAD part ('part_id') or the entire scene ('scope': 'scene'), optionally enabling 'auto_check': true so mutating tools automatically append 'requirements_status'. Units: bbox_max and bbox_min 'dims' [dx, dy, dz] are in active length_unit ('mm' by default, or 'm'); mass_max 'kg' in kilograms (kg); min_safety_factor 'value' is dimensionless (> 0); expected_genus and mechanism_dof 'expected' are dimensionless integers. Example: {\"part_id\": \"bracket_1\", \"auto_check\": true, \"requirements\": [{\"id\": \"req_bbox\", \"type\": \"bbox_max\", \"dims\": [120.0, 40.0, 30.0]}, {\"id\": \"req_mass\", \"type\": \"mass_max\", \"kg\": 0.5}, {\"id\": \"req_mesh\", \"type\": \"mesh_valid\"}, {\"id\": \"req_fos\", \"type\": \"min_safety_factor\", \"value\": 2.0}]}. Common errors: INVALID_ARGUMENT (unknown requirement type, negative dims/kg/value, or missing required fields). Next step: build or modify geometry and call 'check_requirements'.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Target part ID to attach requirements to (omit when scope='scene')"
                    },
                    "scope": {
                        "type": "string",
                        "enum": ["part", "scene"],
                        "description": "Requirement target scope: 'part' (requires part_id) or 'scene' (assembly-wide)"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit) for bbox_max and bbox_min dimensions"
                    },
                    "auto_check": {
                        "type": "boolean",
                        "description": "If true, mutating CAD operations automatically append requirements_status and failed_requirements to their JSON output"
                    },
                    "requirements": {
                        "type": "array",
                        "description": "Array of engineering requirement specifications",
                        "items": {
                            "type": "object",
                            "properties": {
                                "id": {
                                    "type": "string",
                                    "description": "Optional unique requirement identifier (e.g. 'req_bbox_1'; auto-generated if omitted)"
                                },
                                "type": {
                                    "type": "string",
                                    "enum": [
                                        "bbox_max",
                                        "bbox_min",
                                        "mass_max",
                                        "mesh_valid",
                                        "min_safety_factor",
                                        "no_interference",
                                        "mechanism_dof"
                                    ],
                                    "description": "Requirement category"
                                },
                                "dims": {
                                    "type": "array",
                                    "items": { "type": "number" },
                                    "minItems": 3,
                                    "maxItems": 3,
                                    "description": "Bounding box dimensions [dx, dy, dz] in active length_unit (mm by default, or m) for bbox_max / bbox_min"
                                },
                                "kg": {
                                    "type": "number",
                                    "description": "Maximum allowed mass in kilograms (kg) for mass_max"
                                },
                                "expected_genus": {
                                    "type": "integer",
                                    "description": "Optional expected topological genus (dimensionless integer) for mesh_valid"
                                },
                                "value": {
                                    "type": "number",
                                    "description": "Minimum required Factor of Safety (dimensionless > 0) for min_safety_factor"
                                },
                                "fea_case": {
                                    "type": "string",
                                    "description": "Optional FEA load case identifier for min_safety_factor"
                                },
                                "pairs": {
                                    "type": "array",
                                    "items": {
                                        "type": "array",
                                        "items": { "type": "string" },
                                        "minItems": 2,
                                        "maxItems": 2
                                    },
                                    "description": "Optional pairs of part IDs [[part_a, part_b], ...] for no_interference"
                                },
                                "expected": {
                                    "type": "integer",
                                    "description": "Expected mechanism degrees of freedom (dimensionless integer DOF count) for mechanism_dof"
                                }
                            },
                            "required": ["type"]
                        }
                    }
                },
                "required": ["requirements"]
            }),
        },
        // 33. check_requirements
        ToolDefinition {
            name: "check_requirements".to_string(),
            description: "Evaluates registered engineering requirements for a part ('part_id') or the entire scene, returning status ('PASS', 'FAIL', 'STALE', 'NOT_AVAILABLE'), measured values, targets, margins, and remediation hints. Units: bounding box dimensions and margins in active length_unit ('mm' by default, or 'm'), mass in kilograms (kg), Factor of Safety dimensionless, stresses in MPa. Example: {\"part_id\": \"bracket_1\"}. Common errors: INVALID_ARGUMENT (invalid length_unit override). Next step: if any requirement is 'STALE', call 'run_fea_analysis'; if 'FAIL', adjust geometry via 'modify_feature'.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "part_id": {
                        "type": "string",
                        "description": "Optional part ID to filter requirement checks to a single part (if omitted, evaluates all part and scene requirements)"
                    },
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit) for reported bounding box dimensions and margins"
                    }
                }
            }),
        },
        // 34. get_scene_summary
        ToolDefinition {
            name: "get_scene_summary".to_string(),
            description: "Returns a compact text and JSON overview of the entire CAD/CAE scene for an agent entering a new context: project units, all parts (id, name, material, bounding box, volume, mass, mesh validity status, latest FEA FoS/verdict/staleness), kinematic joints, and declared requirements with their status. Units: bounding boxes and joint anchors in active length_unit ('mm' by default, or 'm'), volume in mm^3 or m^3, mass in kg, FEA stress in MPa, displacement in mm, FoS dimensionless. Example: {}. Common errors: INVALID_ARGUMENT (invalid length_unit override). Next step: call 'query_part_geometry' on a specific part, 'run_fea_analysis' if FEA is STALE, or 'create_part' if the scene is empty.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "length_unit": {
                        "type": "string",
                        "enum": ["mm", "m"],
                        "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
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
        "set_units" => {
            let res = cad::set_units(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
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
            let res = cad::export_stl(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "tessellate_part" => {
            let res = cad::tessellate_part(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "import_stl" => {
            let res = cad::import_stl(args, state)?;
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
        "query_part_geometry" => {
            let res = cad::query_part_geometry(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "boolean_union" => {
            let res = cad::boolean_union(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "boolean_intersection" => {
            let res = cad::boolean_intersection(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "transform_part" => {
            let res = cad::transform_part(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "define_material" => {
            let res = cad::define_material(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "batch_cad_operations" => {
            let res = cad::batch_cad_operations(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "validate_mesh" => {
            let res = cad::validate_mesh_tool(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "optimize_feature_tree" => {
            let res = cad::optimize_feature_tree(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "save_project" => {
            let res = project::save_project(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "load_project" => {
            let res = project::load_project(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "set_requirements" => {
            let res = requirements::set_requirements(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "check_requirements" => {
            let res = requirements::check_requirements(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        "get_scene_summary" => {
            let res = metrics::get_scene_summary(args, state)?;
            ToolCallResult::json(&res).map_err(ServerError::Json)
        }
        other => Err(ServerError::ToolNotFound(other.to_string())),
    }
}
