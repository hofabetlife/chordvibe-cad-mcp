//! Finite Element Analysis (FEA) and stress heatmap visualization tools.

use serde_json::{json, Value};

use mcp_cad_core::topology::{TriangleMesh, Vertex};
use mcp_cad_fea::boundary::{AabbFace, BoundaryConditions, DofComponent, NeumannLoad};
use mcp_cad_fea::tetra_mesh::{MeshDensity, TetrahedralMesh};
use mcp_cad_viewer::camera::CameraPreset;

use crate::error::ServerError;
use crate::state::EngineState;

/// Helper to parse camera presets from string.
pub fn parse_camera_preset(s: Option<&str>) -> CameraPreset {
    match s.map(|v| v.to_lowercase()).as_deref() {
        Some("front") => CameraPreset::Front,
        Some("top") => CameraPreset::Top,
        Some("right") | Some("side") => CameraPreset::Right,
        _ => CameraPreset::Isometric,
    }
}

/// Helper to parse AABB face names.
fn parse_aabb_face(s: &str) -> Option<AabbFace> {
    match s.to_lowercase().replace('-', "_").as_str() {
        "min_x" | "minx" | "-x" => Some(AabbFace::MinX),
        "max_x" | "maxx" | "+x" => Some(AabbFace::MaxX),
        "min_y" | "miny" | "-y" => Some(AabbFace::MinY),
        "max_y" | "maxy" | "+y" => Some(AabbFace::MaxY),
        "min_z" | "minz" | "-z" => Some(AabbFace::MinZ),
        "max_z" | "maxz" | "+z" => Some(AabbFace::MaxZ),
        _ => None,
    }
}

/// Helper to parse DOF constraint components from JSON (`"all"`, `"x"`, `"y"`, `"z"`, `"xy"`, `"xz"`, `"yz"`, or array).
fn parse_dof_components(val: Option<&Value>) -> Result<DofComponent, ServerError> {
    let Some(v) = val else {
        return Ok(DofComponent::All);
    };
    if let Some(s) = v.as_str() {
        return match s.trim().to_ascii_lowercase().as_str() {
            "all" | "xyz" => Ok(DofComponent::All),
            "x" => Ok(DofComponent::X),
            "y" => Ok(DofComponent::Y),
            "z" => Ok(DofComponent::Z),
            "xy" | "yx" => Ok(DofComponent::XY),
            "xz" | "zx" => Ok(DofComponent::XZ),
            "yz" | "zy" => Ok(DofComponent::YZ),
            other => Err(ServerError::InvalidArgument(format!(
                "Invalid 'components' value '{other}': expected 'all', 'x', 'y', 'z', 'xy', 'xz', or 'yz'"
            ))),
        };
    }
    if let Some(arr) = v.as_array() {
        let mut mask = [false, false, false];
        if arr.len() == 3 && arr.iter().all(|item| item.is_boolean()) {
            mask = [
                arr[0].as_bool().unwrap_or(true),
                arr[1].as_bool().unwrap_or(true),
                arr[2].as_bool().unwrap_or(true),
            ];
        } else {
            for item in arr {
                if let Some(axis) = item.as_str() {
                    match axis.trim().to_ascii_lowercase().as_str() {
                        "x" => mask[0] = true,
                        "y" => mask[1] = true,
                        "z" => mask[2] = true,
                        "all" => mask = [true, true, true],
                        other => {
                            return Err(ServerError::InvalidArgument(format!(
                                "Invalid axis '{other}' in 'components': expected 'x', 'y', or 'z'"
                            )))
                        }
                    }
                }
            }
        }
        return match mask {
            [true, true, true] => Ok(DofComponent::All),
            [true, false, false] => Ok(DofComponent::X),
            [false, true, false] => Ok(DofComponent::Y),
            [false, false, true] => Ok(DofComponent::Z),
            [true, true, false] => Ok(DofComponent::XY),
            [true, false, true] => Ok(DofComponent::XZ),
            [false, true, true] => Ok(DofComponent::YZ),
            [false, false, false] => Err(ServerError::InvalidArgument(
                "Invalid 'components' in fixed constraint: at least one axis must be constrained".to_string(),
            )),
        };
    }
    Err(ServerError::InvalidArgument(
        "Invalid 'components' in fixed constraint: expected 'all', 'x', 'y', 'z', or boolean/axis array".to_string(),
    ))
}

/// Resolves a face selector entry against `part.mesh` and extracts its 3D triangles and metadata.
fn resolve_face_triangles_and_info<'a>(
    mesh: &'a TriangleMesh,
    selector: &Value,
    unit: crate::state::LengthUnit,
) -> Result<(String, Vec<&'a mcp_cad_core::FaceInfo>, Vec<[[f64; 3]; 3]>), ServerError> {
    let matched_faces = super::cad::resolve_face_selector(mesh, selector, unit)?;
    let face_ids: Vec<u32> = matched_faces.iter().map(|f| f.id).collect();
    let label = matched_faces
        .iter()
        .map(|f| f.name.as_str())
        .collect::<Vec<_>>()
        .join("+");
    let triangles = mcp_cad_fea::extract_face_triangles(mesh, &face_ids);
    if triangles.is_empty() {
        return Err(ServerError::InvalidArgument(format!(
            "FACE_NOT_FOUND: Resolved face '{label}' has 0 triangles on the current part mesh"
        )));
    }
    Ok((label, matched_faces, triangles))
}

/// 11. `run_fea_analysis`: Tetrahedralizes a part, solves 3D linear elasticity, and computes FoS.
pub fn run_fea_analysis(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    let unit = state.resolve_length_unit(args)?;
    let k_to_m = unit.to_meters_factor();

    let part = state
        .parts
        .get(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    if part.mesh.vertices.is_empty() {
        return Err(ServerError::InvalidArgument(
            "Cannot run FEA on empty part. Add geometric features first.".to_string(),
        ));
    }

    let density = match args.get("mesh_density").and_then(|v| v.as_str()).map(|s| s.to_lowercase()).as_deref() {
        Some("fine") => MeshDensity::Fine,
        Some("medium") => MeshDensity::Medium,
        _ => MeshDensity::Coarse,
    };

    // 1. Generate 3D tetrahedral mesh
    let tet_mesh = TetrahedralMesh::from_part(part, density)?;
    if tet_mesh.is_empty() {
        return Err(ServerError::Internal("Tetrahedral meshing produced an empty mesh".to_string()));
    }

    // 2. Setup boundary conditions
    let mut boundary = BoundaryConditions::new();
    let mut dirichlet_added = false;

    // Check fixed_plane (origin and tolerance in active length_unit)
    if let Some(fp) = args.get("fixed_plane").and_then(|v| v.as_object()) {
        if let (Some(orig_val), Some(norm_val)) = (fp.get("origin").and_then(|v| v.as_array()), fp.get("normal").and_then(|v| v.as_array())) {
            if orig_val.len() == 3 && norm_val.len() == 3 {
                let origin = [
                    orig_val[0].as_f64().unwrap_or(0.0) * k_to_m,
                    orig_val[1].as_f64().unwrap_or(0.0) * k_to_m,
                    orig_val[2].as_f64().unwrap_or(0.0) * k_to_m,
                ];
                let normal = [
                    norm_val[0].as_f64().unwrap_or(1.0),
                    norm_val[1].as_f64().unwrap_or(0.0),
                    norm_val[2].as_f64().unwrap_or(0.0),
                ];
                let tolerance = fp
                    .get("tolerance")
                    .and_then(|v| v.as_f64())
                    .map(|t| t * k_to_m)
                    .unwrap_or(1e-3);
                boundary.fix_plane(origin, normal, tolerance);
                dirichlet_added = true;
            }
        }
    }

    // Check fixed_faces (supports both legacy AABB face strings and named CAD faces)
    if let Some(faces) = args.get("fixed_faces").and_then(|v| v.as_array()) {
        for f in faces {
            if let Some(f_str) = f.as_str() {
                if let Some(face) = parse_aabb_face(f_str) {
                    boundary.fix_aabb_face(face, 1e-4);
                    dirichlet_added = true;
                } else {
                    let (label, _, triangles) = resolve_face_triangles_and_info(&part.mesh, f, unit)?;
                    boundary.fix_surface_triangles(Some(label), triangles, 0.0, DofComponent::All);
                    dirichlet_added = true;
                }
            } else {
                let (label, _, triangles) = resolve_face_triangles_and_info(&part.mesh, f, unit)?;
                boundary.fix_surface_triangles(Some(label), triangles, 0.0, DofComponent::All);
                dirichlet_added = true;
            }
        }
    }

    // Check 'fixed' parameter (array or single FaceSelector object/string)
    if let Some(fixed_val) = args.get("fixed") {
        let fixed_items: Vec<&Value> = if let Some(arr) = fixed_val.as_array() {
            arr.iter().collect()
        } else {
            vec![fixed_val]
        };

        for item in fixed_items {
            let components = parse_dof_components(item.get("components"))?;
            let tol_m = item
                .get("tolerance")
                .and_then(|v| v.as_f64())
                .map(|t| t * k_to_m)
                .unwrap_or(0.0);

            let (label, _, triangles) = resolve_face_triangles_and_info(&part.mesh, item, unit)?;
            boundary.fix_surface_triangles(Some(label), triangles, tol_m, components);
            dirichlet_added = true;
        }
    }

    // Reject unconstrained FEA models instead of silently injecting MinX (BUG-017 / BUG-019)
    if !dirichlet_added {
        return Err(ServerError::InvalidArgument(
            "Unconstrained FEA model: at least one fixed boundary constraint ('fixed', 'fixed_faces', or 'fixed_plane') is required to prevent rigid body motion".to_string(),
        ));
    }

    // Applied loads
    let mut loads_added = false;
    let explicit_loads_key = args.get("applied_forces").is_some()
        || args.get("applied_force").is_some()
        || args.get("loads").is_some();

    // Helper closure to parse and register a single load object (from 'loads' or 'applied_forces')
    let mut process_load_obj = |load_obj: &Value| -> Result<(), ServerError> {
        let has_face_selector = load_obj.get("face").is_some()
            || load_obj.get("faces").is_some()
            || load_obj.get("face_id").is_some()
            || load_obj.get("where").is_some();

        let tol_m = load_obj
            .get("tolerance")
            .and_then(|v| v.as_f64())
            .map(|t| t * k_to_m)
            .unwrap_or(0.0);

        let f_vec_val = load_obj
            .get("force")
            .or_else(|| load_obj.get("force_vector"))
            .or_else(|| load_obj.get("total_force"))
            .and_then(|v| v.as_array());

        let pressure_pa_opt = if let Some(p_mpa) = load_obj
            .get("pressure")
            .or_else(|| load_obj.get("pressure_mpa"))
            .and_then(|v| v.as_f64())
        {
            Some(p_mpa * 1.0e6)
        } else {
            load_obj.get("pressure_pa").and_then(|v| v.as_f64())
        };

        let moment_val_opt = load_obj
            .get("moment")
            .or_else(|| load_obj.get("torque"))
            .or_else(|| load_obj.get("moment_nm"));

        if has_face_selector {
            let (label, matched_faces, triangles) =
                resolve_face_triangles_and_info(&part.mesh, load_obj, unit)?;

            let mut applied_any_on_face = false;

            if let Some(f_vec) = f_vec_val {
                if f_vec.len() != 3 {
                    return Err(ServerError::InvalidArgument(
                        "Load 'force' must be a 3-element array [Fx, Fy, Fz] in Newtons (N)".to_string(),
                    ));
                }
                let force = [
                    f_vec[0].as_f64().unwrap_or(0.0),
                    f_vec[1].as_f64().unwrap_or(0.0),
                    f_vec[2].as_f64().unwrap_or(0.0),
                ];
                boundary.add_surface_triangles_force(Some(label.clone()), triangles.clone(), tol_m, force);
                loads_added = true;
                applied_any_on_face = true;
            }

            if let Some(pressure_pa) = pressure_pa_opt {
                boundary.add_surface_triangles_pressure(Some(label.clone()), triangles.clone(), tol_m, pressure_pa);
                loads_added = true;
                applied_any_on_face = true;
            }

            if let Some(m_val) = moment_val_opt {
                let moment_nm = if let Some(m_arr) = m_val.as_array() {
                    if m_arr.len() != 3 {
                        return Err(ServerError::InvalidArgument(
                            "Load 'moment' must be a 3-element array [Mx, My, Mz] in N*m or a scalar".to_string(),
                        ));
                    }
                    [
                        m_arr[0].as_f64().unwrap_or(0.0),
                        m_arr[1].as_f64().unwrap_or(0.0),
                        m_arr[2].as_f64().unwrap_or(0.0),
                    ]
                } else if let Some(m_scalar) = m_val.as_f64() {
                    let axis = matched_faces
                        .first()
                        .map(|f| f.normal_or_axis)
                        .unwrap_or([0.0, 0.0, 1.0]);
                    [axis[0] * m_scalar, axis[1] * m_scalar, axis[2] * m_scalar]
                } else {
                    return Err(ServerError::InvalidArgument(
                        "Load 'moment' must be a 3-element array [Mx, My, Mz] in N*m or a scalar".to_string(),
                    ));
                };

                boundary.add_surface_triangles_moment(Some(label), triangles, tol_m, moment_nm);
                loads_added = true;
                applied_any_on_face = true;
            }

            if !applied_any_on_face {
                return Err(ServerError::InvalidArgument(
                    "Each entry in 'loads' must specify 'force' ([Fx, Fy, Fz] in N), 'pressure' (in MPa), or 'moment' ([Mx, My, Mz] in N*m)".to_string(),
                ));
            }
            return Ok(());
        }

        if let Some(f_vec) = f_vec_val {
            if f_vec.len() == 3 {
                let force = [
                    f_vec[0].as_f64().unwrap_or(0.0),
                    f_vec[1].as_f64().unwrap_or(0.0),
                    f_vec[2].as_f64().unwrap_or(0.0),
                ];

                if let Some(pos_val) = load_obj.get("position").and_then(|v| v.as_array()) {
                    if pos_val.len() == 3 {
                        let target_pos = [
                            pos_val[0].as_f64().unwrap_or(0.0) * k_to_m,
                            pos_val[1].as_f64().unwrap_or(0.0) * k_to_m,
                            pos_val[2].as_f64().unwrap_or(0.0) * k_to_m,
                        ];

                        // Find closest node to target position
                        let mut closest_node = 0;
                        let mut min_dist_sq = f64::INFINITY;
                        for node in &tet_mesh.nodes {
                            let dx = node.position[0] - target_pos[0];
                            let dy = node.position[1] - target_pos[1];
                            let dz = node.position[2] - target_pos[2];
                            let dist_sq = dx * dx + dy * dy + dz * dz;
                            if dist_sq < min_dist_sq {
                                min_dist_sq = dist_sq;
                                closest_node = node.id;
                            }
                        }

                        boundary.neumann.push(NeumannLoad::NodalForce {
                            node_id: closest_node,
                            force,
                        });
                        loads_added = true;
                    }
                } else {
                    // No explicit position or face selector: apply to MaxX face
                    boundary.add_aabb_face_force(AabbFace::MaxX, 1e-4, force);
                    loads_added = true;
                }
            }
        }
        Ok(())
    };

    // Check 'loads' parameter (array or single object)
    if let Some(loads_val) = args.get("loads") {
        let loads_list: Vec<&Value> = if let Some(arr) = loads_val.as_array() {
            arr.iter().collect()
        } else {
            vec![loads_val]
        };
        for load_obj in loads_list {
            process_load_obj(load_obj)?;
        }
    }

    // Check applied_forces array or applied_force object
    let forces_list: Vec<&Value> = if let Some(arr) = args.get("applied_forces").and_then(|v| v.as_array()) {
        arr.iter().collect()
    } else if let Some(single) = args.get("applied_force") {
        vec![single]
    } else {
        Vec::new()
    };

    for force_obj in forces_list {
        process_load_obj(force_obj)?;
    }

    // Only apply default load if neither loads, applied_forces, nor applied_force was passed in args
    if !loads_added && !explicit_loads_key {
        boundary.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, -1000.0, 0.0]);
    }

    // 3. Execute analysis
    let part_revision = part.revision;
    let fea_case_opt = args
        .get("fea_case")
        .or_else(|| args.get("case"))
        .and_then(|v| v.as_str())
        .map(|s| s.to_string());
    let report = mcp_cad_fea::analyze_mesh(part_id, &tet_mesh, &part.material, &boundary, None)?;

    // Send FEA heatmap mesh and normalized stresses to interactive GUI
    let surface_tris = tet_mesh.extract_surface_triangles();
    let mut vertices = Vec::with_capacity(tet_mesh.nodes.len());
    for node in &tet_mesh.nodes {
        vertices.push(Vertex::new(node.position, [0.0, 1.0, 0.0]));
    }
    let indices: Vec<[u32; 3]> = surface_tris
        .into_iter()
        .map(|[a, b, c]| [a as u32, b as u32, c as u32])
        .collect();
    let tet_surface_mesh = TriangleMesh::new(vertices, indices);
    state.send_gui_fea(&tet_surface_mesh, &report.normalized_stresses);

    state.last_fea_reports.insert(part_id.to_string(), report.clone());
    state.last_fea_revisions.insert(part_id.to_string(), part_revision);
    if let Some(ref case_name) = fea_case_opt {
        state
            .fea_case_reports
            .insert((part_id.to_string(), case_name.clone()), (report.clone(), part_revision));
    }
    state.last_fea_meshes.insert(part_id.to_string(), tet_mesh);

    Ok(json!({
        "max_von_mises_mpa": report.max_stress_von_mises_pa / 1.0e6,
        "yield_strength_mpa": report.yield_strength_pa / 1.0e6,
        "safety_factor": report.factor_of_safety,
        "max_displacement_mm": report.max_displacement_m * 1000.0,
        "status": report.verdict,
        "verdict": report.verdict,
        "node_count": report.node_count,
        "element_count": report.element_count,
        "computation_time_ms": report.computation_time_ms,
        "part_revision": part_revision,
        "fea_case": fea_case_opt,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// 12. `get_fea_heatmap_image`: Captures an offscreen 3D render of the von Mises stress heatmap.
pub fn get_fea_heatmap_image(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

    // Check if report exists; if not, run default FEA analysis first
    if !state.last_fea_reports.contains_key(part_id) || !state.last_fea_meshes.contains_key(part_id) {
        run_fea_analysis(args, state)?;
    }

    let report = state
        .last_fea_reports
        .get(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    let tet_mesh = state
        .last_fea_meshes
        .get(part_id)
        .ok_or_else(|| ServerError::PartNotFound(part_id.to_string()))?;

    let preset = parse_camera_preset(args.get("camera_preset").and_then(|v| v.as_str()));
    let width = args.get("width").and_then(|v| v.as_u64()).unwrap_or(800) as u32;
    let height = args.get("height").and_then(|v| v.as_u64()).unwrap_or(600) as u32;

    // Construct external boundary surface mesh corresponding 1:1 to tet_mesh nodes
    let surface_tris = tet_mesh.extract_surface_triangles();
    let mut vertices = Vec::with_capacity(tet_mesh.nodes.len());
    for node in &tet_mesh.nodes {
        vertices.push(Vertex::new(node.position, [0.0, 1.0, 0.0]));
    }
    let indices: Vec<[u32; 3]> = surface_tris
        .into_iter()
        .map(|[a, b, c]| [a as u32, b as u32, c as u32])
        .collect();

    let tet_surface_mesh = TriangleMesh::new(vertices, indices);

    let image_base64 = state
        .offscreen_renderer
        .render_fea_heatmap_base64(
            &tet_surface_mesh,
            &report.normalized_stresses,
            preset,
            width,
            height,
        )?;

    let legend = json!({
        "min_mpa": 0.0,
        "max_mpa": report.max_stress_von_mises_pa / 1.0e6,
        "yield_strength_mpa": report.yield_strength_pa / 1.0e6,
        "factor_of_safety": report.factor_of_safety,
        "verdict": report.verdict,
        "colormap_scale": [
            "blue (0.0 MPa - lowest stress)",
            "cyan",
            "green (safe)",
            "yellow (moderate)",
            "red (peak stress)"
        ]
    });

    let unit = state.length_unit;

    Ok(json!({
        "image_base64": image_base64,
        "colormap_legend": legend,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}
