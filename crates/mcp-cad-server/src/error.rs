//! Server error definitions and structured error representations.

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use thiserror::Error;

/// MCP tool execution error envelope containing the structured error payload.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ErrorEnvelope {
    pub error: ErrorBody,
}

/// Structured error payload providing standardized machine-readable codes,
/// human-readable messages, parameter identifiers, actionable hints,
/// retryability status, and context details for AI agents and automated clients.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ErrorBody {
    /// Stable machine-readable error code (e.g., "PART_NOT_FOUND", "INVALID_ARGUMENT", "FEA_UNCONSTRAINED").
    pub code: String,
    /// Human-readable error message.
    pub message: String,
    /// Name of the invalid or missing parameter, if applicable.
    pub param: Option<String>,
    /// Actionable recommendation or next step for the client/agent.
    pub hint: Option<String>,
    /// Indicates whether repeating the operation after a delay or minor adjustment could succeed.
    pub retryable: bool,
    /// Contextual structured details (e.g. available part IDs, supported primitive types, bounds).
    pub details: Value,
}

impl ErrorBody {
    pub fn new(code: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            code: code.into(),
            message: message.into(),
            param: None,
            hint: None,
            retryable: false,
            details: json!({}),
        }
    }

    pub fn with_param(mut self, param: impl Into<String>) -> Self {
        self.param = Some(param.into());
        self
    }

    pub fn with_hint(mut self, hint: impl Into<String>) -> Self {
        self.hint = Some(hint.into());
        self
    }

    pub fn with_retryable(mut self, retryable: bool) -> Self {
        self.retryable = retryable;
        self
    }

    pub fn with_details(mut self, details: Value) -> Self {
        self.details = details;
        self
    }

    pub fn to_envelope(&self) -> ErrorEnvelope {
        ErrorEnvelope {
            error: self.clone(),
        }
    }

    pub fn to_json_string(&self) -> String {
        serde_json::to_string(&self.to_envelope()).unwrap_or_else(|_| {
            format!(
                r#"{{"error":{{"code":"{}","message":"{}","param":null,"hint":null,"retryable":false,"details":{{}}}}}}"#,
                self.code, self.message
            )
        })
    }
}

/// Unified error type for the MCP-CAD server.
#[derive(Debug, Error)]
pub enum ServerError {
    #[error("CAD core error: {0}")]
    Cad(#[from] mcp_cad_core::CadError),

    #[error("Physics error: {0}")]
    Physics(#[from] mcp_cad_physics::PhysicsError),

    #[error("FEA error: {0}")]
    Fea(#[from] mcp_cad_fea::FeaError),

    #[error("Viewer error: {0}")]
    Viewer(#[from] mcp_cad_viewer::ViewerError),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("JSON serialization/deserialization error: {0}")]
    Json(#[from] serde_json::Error),

    #[error("Invalid argument: {0}")]
    InvalidArgument(String),

    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    #[error("Part not found: {0}")]
    PartNotFound(String),

    #[error("Feature not found: {0}")]
    FeatureNotFound(String),

    #[error("Joint not found: {0}")]
    JointNotFound(String),

    #[error("Tool not found: {0}")]
    ToolNotFound(String),

    #[error("Internal engine error: {0}")]
    Internal(String),
}

impl ServerError {
    /// Converts this `ServerError` into a standardized `ErrorBody` without external session context.
    pub fn to_error_body(&self) -> ErrorBody {
        self.to_error_body_with_parts(None)
    }

    /// Converts this `ServerError` into a standardized `ErrorBody`, incorporating
    /// the optional list of existing part IDs in the session to construct actionable hints.
    pub fn to_error_body_with_parts(&self, existing_parts: Option<&[String]>) -> ErrorBody {
        match self {
            ServerError::PartNotFound(part_id) => {
                let mut body = ErrorBody::new("PART_NOT_FOUND", self.to_string())
                    .with_param("part_id");

                if let Some(parts) = existing_parts {
                    if parts.is_empty() {
                        body = body
                            .with_hint(format!(
                                "Part '{part_id}' was not found. No parts currently exist in the session. Call 'create_part' first to create a part."
                            ))
                            .with_details(json!({
                                "part_id": part_id,
                                "available_parts": []
                            }));
                    } else {
                        body = body
                            .with_hint(format!(
                                "Part '{part_id}' was not found. Available parts: [{}]. Inspect parts via 'list_parts' or create it with 'create_part'.",
                                parts.join(", ")
                            ))
                            .with_details(json!({
                                "part_id": part_id,
                                "available_parts": parts
                            }));
                    }
                } else {
                    body = body
                        .with_hint(format!(
                            "Part '{part_id}' was not found. Call 'list_parts' to inspect available part IDs or create one with 'create_part'."
                        ))
                        .with_details(json!({ "part_id": part_id }));
                }

                body
            }

            ServerError::FeatureNotFound(feat_id) => ErrorBody::new("FEATURE_NOT_FOUND", self.to_string())
                .with_param("feature_id")
                .with_hint(format!(
                    "Feature '{feat_id}' does not exist in the part's feature tree. Inspect part features via 'query_part_geometry'."
                ))
                .with_details(json!({ "feature_id": feat_id })),

            ServerError::JointNotFound(joint_id) => ErrorBody::new("JOINT_NOT_FOUND", self.to_string())
                .with_param("joint_id")
                .with_hint(format!(
                    "Joint '{joint_id}' was not found in physics world. Create it via 'create_joint' or verify joint_id."
                ))
                .with_details(json!({ "joint_id": joint_id })),

            ServerError::ToolNotFound(tool_name) => ErrorBody::new("TOOL_NOT_FOUND", self.to_string())
                .with_param("name")
                .with_hint(format!(
                    "Tool '{tool_name}' is not recognized. Call 'tools/list' to view all available tools and their schemas."
                ))
                .with_details(json!({ "tool_name": tool_name })),

            ServerError::InvalidArgument(msg) | ServerError::InvalidParameter(msg) => {
                map_invalid_argument_error(self.to_string(), msg)
            }

            ServerError::Cad(cad_err) => map_cad_error(self.to_string(), cad_err),

            ServerError::Fea(fea_err) => map_fea_error(self.to_string(), fea_err),

            ServerError::Physics(phys_err) => map_physics_error(self.to_string(), phys_err, existing_parts),

            ServerError::Viewer(viewer_err) => map_viewer_error(self.to_string(), viewer_err),

            ServerError::Io(io_err) => ErrorBody::new("IO_ERROR", self.to_string())
                .with_hint("Check file path, filesystem permissions, and disk space.")
                .with_details(json!({ "io_error": io_err.to_string() })),

            ServerError::Json(json_err) => ErrorBody::new("JSON_ERROR", self.to_string())
                .with_hint("Verify JSON payload syntax and data types against schema.")
                .with_details(json!({ "json_error": json_err.to_string() })),

            ServerError::Internal(msg) => {
                if msg.contains("Tetrahedral meshing produced an empty mesh") {
                    ErrorBody::new("FEA_EMPTY_MESH", self.to_string())
                        .with_param("part_id")
                        .with_hint("Tetrahedral meshing produced an empty mesh. Ensure CAD part geometry has non-zero solid volume.")
                } else if msg.contains("mesh") || msg.contains("manifold") || msg.contains("sliver") {
                    ErrorBody::new("MESH_INVALID", self.to_string())
                        .with_hint("Mesh integrity check failed. Run 'validate_mesh' to diagnose topological and geometric defects.")
                } else {
                    ErrorBody::new("INTERNAL_ERROR", self.to_string())
                        .with_hint("An unexpected internal engine error occurred.")
                }
            }
        }
    }
}

impl From<&ServerError> for ErrorBody {
    fn from(err: &ServerError) -> Self {
        err.to_error_body()
    }
}

/// Helper that inspects `InvalidArgument` / `InvalidParameter` messages
/// and derives high-level structured codes, identified parameter names, hints, and details.
fn map_invalid_argument_error(full_message: String, msg: &str) -> ErrorBody {
    // 1. Unknown primitive_type
    if msg.contains("Unknown primitive_type") {
        return ErrorBody::new("INVALID_PRIMITIVE_TYPE", full_message)
            .with_param("primitive_type")
            .with_hint("Supported primitive types: box, cylinder, sphere, cone, polygon_prism, spur_gear, revolve, wheel_rim. Check parameter requirements for each in 'tools/list'.")
            .with_details(json!({
                "supported_primitives": [
                    "box", "cylinder", "sphere", "cone", "polygon_prism", "spur_gear", "revolve", "wheel_rim"
                ]
            }));
    }

    // 2. Unknown material
    if msg.contains("Unknown material") {
        return ErrorBody::new("UNKNOWN_MATERIAL", full_message)
            .with_param("material")
            .with_hint("Use a builtin material (e.g. steel_structural_aisi1020, aluminum_6061_t6, titanium_ti6al4v, copper_c10100, pla_plastic, abs_plastic) or define a custom material with 'define_material'.")
            .with_details(json!({
                "builtin_materials": [
                    "steel_structural_aisi1020",
                    "steel_stainless_304",
                    "aluminum_6061_t6",
                    "titanium_ti6al4v",
                    "copper_c10100",
                    "brass_c36000",
                    "pla_plastic",
                    "abs_plastic",
                    "polycarbonate",
                    "nylon_pa6"
                ]
            }));
    }

    // 3. Part with id already exists
    if msg.contains("already exists") && msg.contains("Part") {
        let param = if msg.contains("Part with id") { "part_id" } else { "name" };
        return ErrorBody::new("PART_ALREADY_EXISTS", full_message)
            .with_param(param)
            .with_hint("Part ID or name must be unique. Choose another name or inspect existing parts with 'list_parts'.");
    }

    // 3b. Reserved part id or cannot delete reserved static anchor
    if msg.contains("is reserved") && (msg.contains("ground") || msg.contains("world")) {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("part_id")
            .with_hint("Part IDs 'ground' and 'world' (case-insensitive) are reserved static anchors for kinematic joints. Choose a different part ID.");
    }
    if msg.contains("Cannot delete reserved static anchor") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("part_id")
            .with_hint("Reserved static anchors 'ground' and 'world' cannot be deleted.");
    }

    // 4. Part has no geometry / empty mesh
    if msg.contains("has no geometry") || msg.contains("empty mesh") {
        return ErrorBody::new("EMPTY_GEOMETRY", full_message)
            .with_param("part_id")
            .with_hint("The part contains no 3D vertices or mesh geometry. Add primitives with 'add_feature_primitive' before creating joints, exporting STL, or running FEA.");
    }

    // 5. Revolve profile negative r-coordinate
    if msg.contains("cannot be negative") && msg.contains("r-coordinate") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("profile")
            .with_hint("All radial coordinates r in revolve profile must be >= 0 because the axis of rotation is at r=0.");
    }

    // 6. Revolve angle_deg out of bounds
    if msg.contains("angle_deg") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("angle_deg")
            .with_hint("Angle 'angle_deg' must be strictly positive and at most 360.0 degrees.");
    }

    // 7. Revolve profile self-intersecting
    if msg.contains("self-intersecting") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("profile")
            .with_hint("Revolve profile polygon edges must not cross each other and points must form an ordered simple closed loop.");
    }

    // 8. FEA boundary / unconstrained
    if msg.contains("FEA requires at least one constraint")
        || msg.contains("unconstrained")
        || msg.contains("Unconstrained FEA model")
    {
        return ErrorBody::new("FEA_UNCONSTRAINED", full_message)
            .with_param("fixed_faces")
            .with_hint("Specify fixed boundary faces (e.g. ['min_x', 'max_z'] or 'fixed': [{'face': 'cap_bottom'}]) or 'fixed_plane' to prevent rigid body motion in FEA.");
    }

    // 9. FEA no loads
    if msg.contains("At least one applied force or pressure must be specified") {
        return ErrorBody::new("FEA_NO_LOADS", full_message)
            .with_param("applied_forces")
            .with_hint("Specify at least one force via 'applied_forces' (e.g. [{'face':'max_z', 'force':[0,0,-1000]}]) or 'loads'.");
    }

    // 10. Cannot connect part to itself
    if msg.contains("Cannot connect part") && msg.contains("to itself") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("part_b")
            .with_hint("A joint must connect two distinct parts (or a part to 'ground'). Choose different IDs for part_a and part_b.");
    }

    // 11. Nested batch_cad_operations
    if msg.contains("Nested 'batch_cad_operations'") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("operations")
            .with_hint("Nested batch_cad_operations is not permitted. Flatten operations into a single sequential array.");
    }

    // 12. Batch operation step failure
    if msg.contains("Batch operation at index") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("operations")
            .with_hint("Inspect the failed step in batch operations and ensure all previous referenced parts exist.");
    }

    // 13. Material property validations
    if msg.contains("Young's modulus must be greater than zero") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("young_modulus_gpa")
            .with_hint("Young's modulus must be strictly positive.");
    }
    if msg.contains("Poisson's ratio must be in range") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("poisson_ratio")
            .with_hint("Poisson's ratio must be in the range 0.0 < nu < 0.5.");
    }
    if msg.contains("Yield strength must be greater than zero") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("yield_strength_mpa")
            .with_hint("Yield strength must be strictly positive.");
    }
    if msg.contains("Density must be greater than zero") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("density_kg_m3")
            .with_hint("Density must be strictly positive in kg/m³.");
    }

    // 14. Project schema version mismatch
    if msg.contains("schema_version") {
        return ErrorBody::new("PROJECT_SCHEMA_MISMATCH", full_message)
            .with_param("schema_version")
            .with_hint("The project file uses an incompatible schema_version. Current supported version is '1.0.0'.");
    }

    // 15. Named face not found or ambiguous face selector
    if msg.contains("FACE_NOT_FOUND") || msg.contains("Face not found") {
        let mut details = json!({});
        if let Some(idx) = msg.find("Available faces: [") {
            let start = idx + "Available faces: [".len();
            if let Some(end_rel) = msg[start..].find(']') {
                let list_str = &msg[start..start + end_rel];
                let available_faces: Vec<String> = list_str
                    .split(',')
                    .map(|s| s.trim().to_string())
                    .filter(|s| !s.is_empty())
                    .collect();
                details = json!({ "available_faces": available_faces });
            }
        }
        return ErrorBody::new("FACE_NOT_FOUND", full_message)
            .with_param("face_selector")
            .with_hint("Call 'query_part_geometry' to inspect the 'faces' array for valid face names, aliases, and IDs on this part.")
            .with_details(details);
    }
    if msg.contains("AMBIGUOUS_FACE_SELECTOR") || msg.contains("Ambiguous face selector") {
        return ErrorBody::new("AMBIGUOUS_FACE_SELECTOR", full_message)
            .with_param("face_selector")
            .with_hint("Multiple faces matched the unqualified selector. Qualify by feature (e.g. 'feat_0:top'), use wildcard/array in 'faces', or select by explicit 'face_id' / 'where' filter from 'query_part_geometry'.");
    }

    // 16. Unknown requirement type
    if msg.contains("Unknown requirement type") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("requirements")
            .with_hint("Supported requirement types: bbox_max, bbox_min, mass_max, mesh_valid, min_safety_factor, no_interference, mechanism_dof.")
            .with_details(json!({
                "supported_requirement_types": [
                    "bbox_max",
                    "bbox_min",
                    "mass_max",
                    "mesh_valid",
                    "min_safety_factor",
                    "no_interference",
                    "mechanism_dof"
                ]
            }));
    }

    // 17. Joint motor parameter validations (target_velocity and max_force)
    if msg.contains("'target_velocity'") && !msg.contains("Missing required argument") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("target_velocity")
            .with_hint("Provide a finite number for 'target_velocity' (angular velocity in rad/s for revolute joints, or linear velocity in m/s for prismatic joints).");
    }
    if msg.contains("'max_force'") && !msg.contains("Missing required argument") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("max_force")
            .with_hint("Provide a finite non-negative number (>= 0.0) for 'max_force' (maximum torque in N*m for revolute joints, or maximum force in N for prismatic joints).");
    }

    // 18. Gear joint ratio, teeth_a, teeth_b, and physics step_size_ms validations
    if msg.contains("'teeth_a'") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("teeth_a")
            .with_hint("Provide a positive finite number (> 0) for tooth count 'teeth_a' in gear joint configuration.");
    }
    if msg.contains("'teeth_b'") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("teeth_b")
            .with_hint("Provide a positive finite number (> 0) for tooth count 'teeth_b' in gear joint configuration.");
    }
    if msg.contains("Gear 'ratio'") || msg.contains("gear ratio") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("ratio")
            .with_hint("Provide a non-zero finite number for gear transmission 'ratio' (omega_a / omega_b).");
    }
    if msg.contains("'axis_a'") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("axis_a")
            .with_hint("Provide a non-zero 3D direction vector [x, y, z] for gear rotation 'axis_a'.");
    }
    if msg.contains("'axis_b'") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("axis_b")
            .with_hint("Provide a non-zero 3D direction vector [x, y, z] for gear rotation 'axis_b'.");
    }
    if msg.contains("'step_size_ms'") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("step_size_ms")
            .with_hint("Provide a positive finite number (> 0) for physics simulation discrete step 'step_size_ms' (e.g. 10.0 or 16.6667 ms).");
    }
    if msg.contains("Unsupported length_unit") || msg.contains("'length_unit'") {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param("length_unit")
            .with_hint("Supported values for 'length_unit' are 'mm' and 'm'.");
    }

    // 18. Missing required argument or requires '<name>'
    if let Some(param) = extract_quoted_param(msg) {
        return ErrorBody::new("INVALID_ARGUMENT", full_message)
            .with_param(param.clone())
            .with_hint(format!("Provide or correct parameter '{param}'. Check parameter requirements in 'tools/list'."));
    }

    // General fallback
    ErrorBody::new("INVALID_ARGUMENT", full_message)
        .with_hint("Check parameter values and types against tool schema in 'tools/list'.")
}

/// Helper that inspects CAD core errors.
fn map_cad_error(full_message: String, err: &mcp_cad_core::CadError) -> ErrorBody {
    match err {
        mcp_cad_core::CadError::FeatureNotFound(uuid) => {
            ErrorBody::new("FEATURE_NOT_FOUND", full_message)
                .with_param("feature_id")
                .with_hint("Feature ID does not exist in the part's DAG. Check existing features via 'query_part_geometry'.")
                .with_details(json!({ "feature_id": uuid.to_string() }))
        }
        mcp_cad_core::CadError::MaterialNotFound(mat) => {
            ErrorBody::new("MATERIAL_NOT_FOUND", full_message)
                .with_param("material")
                .with_hint("Material not found. Use a standard builtin material or define one with 'define_material'.")
                .with_details(json!({ "material": mat }))
        }
        mcp_cad_core::CadError::CsgError(csg_msg) => {
            ErrorBody::new("CSG_BOOLEAN_ERROR", full_message)
                .with_hint("CSG boolean operation failed. Ensure operand meshes are closed watertight manifolds without self-intersections.")
                .with_details(json!({ "csg_error": csg_msg }))
        }
        mcp_cad_core::CadError::EvaluationError(eval_msg) => {
            let code = if eval_msg.contains("mesh") || eval_msg.contains("manifold") || eval_msg.contains("sliver") {
                "MESH_INVALID"
            } else {
                "CAD_EVALUATION_ERROR"
            };
            ErrorBody::new(code, full_message)
                .with_hint("CAD model evaluation failed. Check parameters and validate mesh via 'validate_mesh'.")
                .with_details(json!({ "evaluation_error": eval_msg }))
        }
        mcp_cad_core::CadError::InvalidParameter(p_msg) => {
            let mut body = ErrorBody::new("INVALID_PARAMETER", full_message)
                .with_hint("Parameter value is invalid for this CAD feature. Verify numeric ranges.");
            if let Some(param) = extract_quoted_param(p_msg) {
                body = body.with_param(param);
            }
            body
        }
        mcp_cad_core::CadError::IoError(io_err) => {
            ErrorBody::new("IO_ERROR", full_message)
                .with_hint("CAD filesystem IO operation failed.")
                .with_details(json!({ "io_error": io_err.to_string() }))
        }
        mcp_cad_core::CadError::SerializationError(ser_err) => {
            ErrorBody::new("SERIALIZATION_ERROR", full_message)
                .with_hint("CAD data serialization error.")
                .with_details(json!({ "serialization_error": ser_err.to_string() }))
        }
    }
}

/// Helper that inspects FEA solver and mesh errors.
fn map_fea_error(full_message: String, err: &mcp_cad_fea::FeaError) -> ErrorBody {
    match err {
        mcp_cad_fea::FeaError::EmptyMesh => {
            ErrorBody::new("FEA_EMPTY_MESH", full_message)
                .with_param("part_id")
                .with_hint("The tetrahedral mesh generator produced 0 elements. Ensure the CAD part has solid volume (not a degenerate surface) and passes 'validate_mesh'.")
        }
        mcp_cad_fea::FeaError::DegenerateElement { id, volume } => {
            ErrorBody::new("FEA_DEGENERATE_ELEMENT", full_message)
                .with_hint(format!(
                    "Tetrahedral element #{id} has non-positive volume ({volume:e} m³). Adjust mesh density or eliminate sliver faces in CAD geometry."
                ))
                .with_details(json!({ "element_id": id, "volume": volume }))
        }
        mcp_cad_fea::FeaError::InvalidMaterial { youngs_modulus_pa, poissons_ratio } => {
            ErrorBody::new("FEA_INVALID_MATERIAL", full_message)
                .with_param("material")
                .with_hint("Ensure Young's modulus > 0 and Poisson's ratio is in the range 0.0 < nu < 0.5.")
                .with_details(json!({
                    "youngs_modulus_pa": youngs_modulus_pa,
                    "poissons_ratio": poissons_ratio
                }))
        }
        mcp_cad_fea::FeaError::UnconstrainedModel => {
            ErrorBody::new("FEA_UNCONSTRAINED", full_message)
                .with_param("fixed_faces")
                .with_hint("Model has unconstrained rigid body modes. Add Dirichlet boundary conditions using 'fixed' (e.g. [{'face': 'cap_bottom'}]), 'fixed_faces' (e.g. ['min_z']), or 'fixed_plane'.")
        }
        mcp_cad_fea::FeaError::SolverConvergenceFailed { iterations, residual, tolerance } => {
            ErrorBody::new("FEA_SOLVER_CONVERGENCE_FAILED", full_message)
                .with_retryable(true)
                .with_hint(format!(
                    "Linear solver did not converge in {iterations} iterations (residual: {residual:e}, tol: {tolerance:e}). Try refining constraints, checking material parameters, or using a coarser mesh."
                ))
                .with_details(json!({
                    "iterations": iterations,
                    "residual": residual,
                    "tolerance": tolerance
                }))
        }
        mcp_cad_fea::FeaError::NoBoundaryNodesMatched { tolerance } => {
            ErrorBody::new("FEA_NO_BOUNDARY_NODES", full_message)
                .with_param("fixed_faces")
                .with_hint(format!(
                    "Boundary selection matched 0 mesh nodes with search tolerance {tolerance:.6} m. Try increasing 'mesh_density' to 'medium' or 'fine', or increasing 'tolerance'."
                ))
                .with_details(json!({ "tolerance": tolerance }))
        }
        mcp_cad_fea::FeaError::NodeOutOfBounds { node_id, total_nodes } => {
            ErrorBody::new("FEA_NODE_OUT_OF_BOUNDS", full_message)
                .with_details(json!({ "node_id": node_id, "total_nodes": total_nodes }))
        }
        mcp_cad_fea::FeaError::InvalidSurfaceMesh {
            message,
            boundary_edges_count,
            non_manifold_edges_count,
            non_manifold_vertices_count,
            inconsistent_edges_count,
            is_watertight,
        } => {
            ErrorBody::new("MESH_INVALID", full_message)
                .with_param("part_id")
                .with_hint(format!(
                    "Surface mesh is not a closed watertight 2-manifold (boundary_edges: {boundary_edges_count}, non_manifold_edges: {non_manifold_edges_count}, non_manifold_vertices: {non_manifold_vertices_count}). Run 'validate_mesh' to inspect topological defects and repair the CAD features before running FEA."
                ))
                .with_details(json!({
                    "is_watertight": is_watertight,
                    "boundary_edges_count": boundary_edges_count,
                    "non_manifold_edges_count": non_manifold_edges_count,
                    "non_manifold_vertices_count": non_manifold_vertices_count,
                    "inconsistent_edges_count": inconsistent_edges_count,
                    "reason": message,
                }))
        }
        mcp_cad_fea::FeaError::MeshGenerationFailed(msg) => {
            ErrorBody::new("FEA_MESH_GENERATION_FAILED", full_message)
                .with_hint("Voxel tetrahedral meshing failed. Inspect the part geometry with 'validate_mesh' to ensure a watertight 2-manifold surface.")
                .with_details(json!({ "reason": msg }))
        }
        mcp_cad_fea::FeaError::Cad(msg) => {
            ErrorBody::new("CAD_ERROR", full_message)
                .with_hint("FEA encountered a CAD core error during evaluation.")
                .with_details(json!({ "cad_error": msg }))
        }
        mcp_cad_fea::FeaError::General(msg) => {
            ErrorBody::new("FEA_ERROR", full_message)
                .with_hint("FEA analysis failed.")
                .with_details(json!({ "reason": msg }))
        }
    }
}

/// Helper that inspects physics simulation errors.
fn map_physics_error(
    full_message: String,
    err: &mcp_cad_physics::PhysicsError,
    existing_parts: Option<&[String]>,
) -> ErrorBody {
    match err {
        mcp_cad_physics::PhysicsError::PartNotFound(id) => {
            let mut body = ErrorBody::new("PART_NOT_FOUND", full_message)
                .with_param("part_id");
            if let Some(parts) = existing_parts {
                if parts.is_empty() {
                    body = body
                        .with_hint(format!(
                            "Part '{id}' was not found in physics world. No parts currently exist. Call 'create_part' first."
                        ))
                        .with_details(json!({ "part_id": id, "available_parts": [] }));
                } else {
                    body = body
                        .with_hint(format!(
                            "Part '{id}' was not found in physics world. Available parts: [{}].",
                            parts.join(", ")
                        ))
                        .with_details(json!({ "part_id": id, "available_parts": parts }));
                }
            } else {
                body = body
                    .with_hint(format!("Part '{id}' is not in the physics simulation. Ensure the part exists and has non-empty geometry."))
                    .with_details(json!({ "part_id": id }));
            }
            body
        }
        mcp_cad_physics::PhysicsError::JointNotFound(id) => {
            ErrorBody::new("JOINT_NOT_FOUND", full_message)
                .with_param("joint_id")
                .with_hint(format!("Joint '{id}' does not exist in the physics world. Create it via 'create_joint'."))
                .with_details(json!({ "joint_id": id }))
        }
        mcp_cad_physics::PhysicsError::PartAlreadyExists(id) => {
            ErrorBody::new("PART_ALREADY_EXISTS", full_message)
                .with_param("part_id")
                .with_details(json!({ "part_id": id }))
        }
        mcp_cad_physics::PhysicsError::JointAlreadyExists(id) => {
            ErrorBody::new("JOINT_ALREADY_EXISTS", full_message)
                .with_param("joint_id")
                .with_details(json!({ "joint_id": id }))
        }
        mcp_cad_physics::PhysicsError::InvalidGeometry(id, msg) => {
            ErrorBody::new("PHYSICS_INVALID_GEOMETRY", full_message)
                .with_param("part_id")
                .with_hint("Ensure part mesh is non-empty, watertight, and has positive volume before physics simulation.")
                .with_details(json!({ "part_id": id, "reason": msg }))
        }
        mcp_cad_physics::PhysicsError::ColliderCreationFailed(id, msg) => {
            ErrorBody::new("PHYSICS_COLLIDER_FAILED", full_message)
                .with_param("part_id")
                .with_hint("Failed to build collider. Run 'validate_mesh' to check part mesh validity.")
                .with_details(json!({ "part_id": id, "reason": msg }))
        }
        mcp_cad_physics::PhysicsError::InvalidJointConfiguration(msg) => {
            let mut body = ErrorBody::new("INVALID_JOINT_CONFIGURATION", full_message)
                .with_hint("Check joint anchor coordinates, rotation axis normalization, joint limits [min <= max], and part connectivity.")
                .with_details(json!({ "reason": msg }));
            if msg.contains("limits") {
                body = body.with_param("limits");
            } else if msg.contains("axis_a") {
                body = body.with_param("axis_a");
            } else if msg.contains("axis_b") {
                body = body.with_param("axis_b");
            } else if msg.contains("axis") {
                body = body.with_param("axis");
            } else if msg.contains("ratio") {
                body = body.with_param("ratio");
            } else if msg.contains("motor") || msg.contains("Motor") {
                body = body
                    .with_param("joint_id")
                    .with_hint("Motors are only supported on 'revolute' and 'prismatic' joints. For 'gear' constraints, apply the motor to one of the connected revolute joints.");
            }
            body
        }
        mcp_cad_physics::PhysicsError::SimulationError(msg) => {
            ErrorBody::new("PHYSICS_SIMULATION_ERROR", full_message)
                .with_retryable(true)
                .with_hint("Physics step failed. Check applied forces, damping, joint limits, or reduce simulation step_size.")
                .with_details(json!({ "reason": msg }))
        }
        mcp_cad_physics::PhysicsError::SerializationError(msg) => {
            ErrorBody::new("SERIALIZATION_ERROR", full_message)
                .with_hint("Physics serialization failed.")
                .with_details(json!({ "reason": msg }))
        }
    }
}

/// Helper that inspects viewer errors.
fn map_viewer_error(full_message: String, err: &mcp_cad_viewer::ViewerError) -> ErrorBody {
    match err {
        mcp_cad_viewer::ViewerError::EmptyMesh => {
            ErrorBody::new("VIEWER_EMPTY_MESH", full_message)
                .with_param("part_id")
                .with_hint("Cannot render empty mesh. Add geometric features to the part before opening preview or capturing render.")
        }
        mcp_cad_viewer::ViewerError::InvalidDimensions { width, height } => {
            ErrorBody::new("INVALID_DIMENSIONS", full_message)
                .with_param("dimensions")
                .with_hint("Width and height must be positive integers within GPU texture limits (typically <= 4096).")
                .with_details(json!({ "width": width, "height": height }))
        }
        mcp_cad_viewer::ViewerError::StressScalarMismatch { expected, actual } => {
            ErrorBody::new("STRESS_SCALAR_MISMATCH", full_message)
                .with_hint("Number of FEA stress scalars does not match vertex or triangle count of the mesh.")
                .with_details(json!({ "expected": expected, "actual": actual }))
        }
        mcp_cad_viewer::ViewerError::NoSuitableAdapter => {
            ErrorBody::new("GPU_ADAPTER_UNAVAILABLE", full_message)
                .with_hint("No suitable GPU adapter found for headless rendering.")
        }
        mcp_cad_viewer::ViewerError::DeviceRequestFailed(e) => {
            ErrorBody::new("GPU_DEVICE_ERROR", full_message)
                .with_hint("Failed to request GPU device.")
                .with_details(json!({ "reason": e.to_string() }))
        }
        other => {
            ErrorBody::new("VIEWER_ERROR", full_message)
                .with_hint("3D viewport or offscreen renderer error.")
                .with_details(json!({ "viewer_error": other.to_string() }))
        }
    }
}

/// Extracts a quoted parameter name candidate from an error message (e.g. `'param_name'`).
fn extract_quoted_param(msg: &str) -> Option<String> {
    let mut search_from = 0;
    while let Some(start_rel) = msg[search_from..].find('\'') {
        let start = search_from + start_rel + 1;
        if let Some(end_rel) = msg[start..].find('\'') {
            let end = start + end_rel;
            let candidate = &msg[start..end];
            if !candidate.is_empty() && !candidate.contains(' ') && candidate.len() <= 40 {
                return Some(candidate.to_string());
            }
            search_from = end + 1;
        } else {
            break;
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_error_body_part_not_found_without_parts() {
        let err = ServerError::PartNotFound("gear_shaft".to_string());
        let body = err.to_error_body();
        assert_eq!(body.code, "PART_NOT_FOUND");
        assert_eq!(body.param.as_deref(), Some("part_id"));
        assert!(body.message.contains("Part not found: gear_shaft"));
        assert!(body.hint.unwrap().contains("list_parts"));
        assert!(!body.retryable);
        assert_eq!(body.details["part_id"], "gear_shaft");
    }

    #[test]
    fn test_to_error_body_part_not_found_with_parts() {
        let err = ServerError::PartNotFound("missing_bracket".to_string());
        let parts = vec!["gear_1".to_string(), "motor_mount".to_string()];
        let body = err.to_error_body_with_parts(Some(&parts));
        assert_eq!(body.code, "PART_NOT_FOUND");
        assert_eq!(body.param.as_deref(), Some("part_id"));
        let hint = body.hint.expect("Hint must be present");
        assert!(hint.contains("gear_1"));
        assert!(hint.contains("motor_mount"));
        assert_eq!(body.details["available_parts"], json!(["gear_1", "motor_mount"]));
    }

    #[test]
    fn test_to_error_body_feature_not_found() {
        let err = ServerError::FeatureNotFound("feat-123".to_string());
        let body = err.to_error_body();
        assert_eq!(body.code, "FEATURE_NOT_FOUND");
        assert_eq!(body.param.as_deref(), Some("feature_id"));
        assert!(body.hint.unwrap().contains("query_part_geometry"));
    }

    #[test]
    fn test_to_error_body_joint_not_found() {
        let err = ServerError::JointNotFound("joint-99".to_string());
        let body = err.to_error_body();
        assert_eq!(body.code, "JOINT_NOT_FOUND");
        assert_eq!(body.param.as_deref(), Some("joint_id"));
    }

    #[test]
    fn test_to_error_body_tool_not_found() {
        let err = ServerError::ToolNotFound("extrude_surface".to_string());
        let body = err.to_error_body();
        assert_eq!(body.code, "TOOL_NOT_FOUND");
        assert_eq!(body.param.as_deref(), Some("name"));
        assert!(body.hint.unwrap().contains("tools/list"));
    }

    #[test]
    fn test_to_error_body_invalid_primitive_type() {
        let err = ServerError::InvalidArgument(
            "Unknown primitive_type 'pyramid'. Supported: box, cylinder, sphere, cone, polygon_prism, spur_gear, revolve, wheel_rim".to_string()
        );
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_PRIMITIVE_TYPE");
        assert_eq!(body.param.as_deref(), Some("primitive_type"));
        assert!(body.details["supported_primitives"].is_array());
    }

    #[test]
    fn test_to_error_body_unknown_material() {
        let err = ServerError::InvalidArgument(
            "Unknown material 'vibranium'. Builtin materials: steel_structural_aisi1020, aluminum_6061_t6...".to_string()
        );
        let body = err.to_error_body();
        assert_eq!(body.code, "UNKNOWN_MATERIAL");
        assert_eq!(body.param.as_deref(), Some("material"));
        assert!(body.details["builtin_materials"].is_array());
    }

    #[test]
    fn test_to_error_body_missing_required_argument() {
        let err = ServerError::InvalidArgument("Missing required argument 'part_id'".to_string());
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("part_id"));
    }

    #[test]
    fn test_to_error_body_revolve_negative_r() {
        let err = ServerError::InvalidArgument("Revolve profile r-coordinate cannot be negative: -10".to_string());
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("profile"));
    }

    #[test]
    fn test_to_error_body_revolve_angle_deg() {
        let err = ServerError::InvalidArgument("Revolve angle_deg must be in range (0.0, 360.0], got 720".to_string());
        let body = err.to_error_body();
        assert_eq!(body.code, "INVALID_ARGUMENT");
        assert_eq!(body.param.as_deref(), Some("angle_deg"));
    }

    #[test]
    fn test_to_error_body_empty_geometry() {
        let err = ServerError::InvalidArgument("Part 'Rotor' has no geometry (empty mesh)".to_string());
        let body = err.to_error_body();
        assert_eq!(body.code, "EMPTY_GEOMETRY");
        assert_eq!(body.param.as_deref(), Some("part_id"));
    }

    #[test]
    fn test_to_error_body_fea_unconstrained() {
        let err = ServerError::Fea(mcp_cad_fea::FeaError::UnconstrainedModel);
        let body = err.to_error_body();
        assert_eq!(body.code, "FEA_UNCONSTRAINED");
        assert_eq!(body.param.as_deref(), Some("fixed_faces"));
    }

    #[test]
    fn test_to_error_body_fea_solver_convergence_failed() {
        let err = ServerError::Fea(mcp_cad_fea::FeaError::SolverConvergenceFailed {
            iterations: 500,
            residual: 0.05,
            tolerance: 1e-6,
        });
        let body = err.to_error_body();
        assert_eq!(body.code, "FEA_SOLVER_CONVERGENCE_FAILED");
        assert!(body.retryable);
        assert_eq!(body.details["iterations"], 500);
    }

    #[test]
    fn test_to_error_body_physics_simulation_error() {
        let err = ServerError::Physics(mcp_cad_physics::PhysicsError::SimulationError(
            "Solver divergence".to_string(),
        ));
        let body = err.to_error_body();
        assert_eq!(body.code, "PHYSICS_SIMULATION_ERROR");
        assert!(body.retryable);
    }

    #[test]
    fn test_to_error_body_cad_csg_error() {
        let err = ServerError::Cad(mcp_cad_core::CadError::CsgError("Non-manifold mesh".to_string()));
        let body = err.to_error_body();
        assert_eq!(body.code, "CSG_BOOLEAN_ERROR");
    }

    #[test]
    fn test_to_error_body_viewer_empty_mesh() {
        let err = ServerError::Viewer(mcp_cad_viewer::ViewerError::EmptyMesh);
        let body = err.to_error_body();
        assert_eq!(body.code, "VIEWER_EMPTY_MESH");
    }

    #[test]
    fn test_to_error_body_io_and_json() {
        let io_err = ServerError::Io(std::io::Error::new(std::io::ErrorKind::NotFound, "file not found"));
        assert_eq!(io_err.to_error_body().code, "IO_ERROR");

        let json_err = ServerError::Json(serde_json::from_str::<Value>("invalid json").unwrap_err());
        assert_eq!(json_err.to_error_body().code, "JSON_ERROR");
    }

    #[test]
    fn test_error_envelope_json_roundtrip() {
        let err = ServerError::PartNotFound("bracket".to_string());
        let body = err.to_error_body_with_parts(Some(&["base".to_string()]));
        let json_str = body.to_json_string();

        let parsed: ErrorEnvelope = serde_json::from_str(&json_str)
            .expect("Serialized JSON must parse into ErrorEnvelope");
        assert_eq!(parsed.error.code, "PART_NOT_FOUND");
        assert_eq!(parsed.error.param.as_deref(), Some("part_id"));
        assert!(!parsed.error.retryable);
        assert_eq!(parsed.error.details["available_parts"], json!(["base"]));
    }

    #[test]
    fn test_to_error_body_project_schema_mismatch() {
        let err = ServerError::InvalidArgument("Unsupported project schema_version '2.0.0'. Expected '1.0.0'.".into());
        let body = err.to_error_body();
        assert_eq!(body.code, "PROJECT_SCHEMA_MISMATCH");
        assert_eq!(body.param.as_deref(), Some("schema_version"));
        assert!(body.hint.unwrap().contains("schema_version"));
    }
}
