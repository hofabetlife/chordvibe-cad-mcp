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

/// Helper to validate and extract a 3-element array of finite `f64` numbers.
fn parse_finite_vec3(arr: &[Value], field_name: &str) -> Result<[f64; 3], ServerError> {
    if arr.len() != 3 {
        return Err(ServerError::InvalidArgument(format!(
            "Invalid '{field_name}': must be a 3-element array of finite numbers"
        )));
    }
    let mut out = [0.0; 3];
    for (i, item) in arr.iter().enumerate() {
        let val = item.as_f64().filter(|v| v.is_finite()).ok_or_else(|| {
            ServerError::InvalidArgument(format!(
                "Invalid '{field_name}': element at index {i} must be a finite number"
            ))
        })?;
        out[i] = val;
    }
    Ok(out)
}

type ResolvedFaceSelection<'a> = (String, Vec<&'a mcp_cad_core::FaceInfo>, Vec<[[f64; 3]; 3]>);

/// Resolves a face selector entry against `part.mesh` and extracts its 3D triangles and metadata.
fn resolve_face_triangles_and_info<'a>(
    mesh: &'a TriangleMesh,
    selector: &Value,
    unit: crate::state::LengthUnit,
) -> Result<ResolvedFaceSelection<'a>, ServerError> {
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

/// Helper to parse a single `MeshDensity` value from a JSON string or number/object.
fn parse_mesh_density_level(val: &Value, k_to_m: f64) -> Result<MeshDensity, ServerError> {
    if let Some(s) = val.as_str() {
        match s.trim().to_ascii_lowercase().as_str() {
            "coarse" => Ok(MeshDensity::Coarse),
            "medium" => Ok(MeshDensity::Medium),
            "fine" => Ok(MeshDensity::Fine),
            other => Err(ServerError::InvalidArgument(format!(
                "Invalid mesh density level '{other}': expected 'coarse', 'medium', or 'fine'"
            ))),
        }
    } else if let Some(h_raw) = val.as_f64().filter(|v| v.is_finite() && *v > 0.0) {
        Ok(MeshDensity::Custom {
            max_element_size: h_raw * k_to_m,
        })
    } else if let Some(h_raw) = val
        .get("max_element_size")
        .and_then(|v| v.as_f64())
        .filter(|v| v.is_finite() && *v > 0.0)
    {
        Ok(MeshDensity::Custom {
            max_element_size: h_raw * k_to_m,
        })
    } else {
        Err(ServerError::InvalidArgument(
            "Invalid entry in 'levels': expected 'coarse', 'medium', 'fine', or a positive element size".to_string(),
        ))
    }
}

/// Binds any positional point loads `(target_pos_m, force_n)` to the closest node in `tet_mesh`.
fn bind_positional_loads_to_mesh(
    base_boundary: &BoundaryConditions,
    positional_loads: &[([f64; 3], [f64; 3])],
    tet_mesh: &TetrahedralMesh,
) -> BoundaryConditions {
    let mut bc = base_boundary.clone();
    for &(target_pos, force) in positional_loads {
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
        bc.neumann.push(NeumannLoad::NodalForce {
            node_id: closest_node,
            force,
        });
    }
    bc
}

/// 11. `run_fea_analysis`: Tetrahedralizes a part, solves 3D linear elasticity, and computes FoS
///     (and optionally runs a multi-level mesh convergence study when `convergence_study: true`).
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

    let convergence_study = match args.get("convergence_study") {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(_) => {
            return Err(ServerError::InvalidArgument(
                "Invalid 'convergence_study': expected a boolean (true or false)".to_string(),
            ))
        }
    };

    let study_levels: Vec<MeshDensity> = if let Some(levels_val) = args.get("levels") {
        if !levels_val.is_null() {
            let arr = levels_val.as_array().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Invalid 'levels': expected an array of at least 2 mesh density levels (e.g. ['coarse', 'medium', 'fine'])"
                        .to_string(),
                )
            })?;
            if arr.len() < 2 {
                return Err(ServerError::InvalidArgument(
                    "Invalid 'levels': at least 2 mesh density levels are required for a convergence study".to_string(),
                ));
            }
            let mut parsed = Vec::with_capacity(arr.len());
            for item in arr {
                parsed.push(parse_mesh_density_level(item, k_to_m)?);
            }
            parsed
        } else {
            vec![MeshDensity::Coarse, MeshDensity::Medium, MeshDensity::Fine]
        }
    } else {
        vec![MeshDensity::Coarse, MeshDensity::Medium, MeshDensity::Fine]
    };

    let max_seconds: Option<f64> = if let Some(ms_val) = args.get("max_seconds") {
        if !ms_val.is_null() {
            let s = ms_val.as_f64().filter(|v| v.is_finite() && *v >= 0.0).ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Invalid 'max_seconds': must be a non-negative finite number in seconds".to_string(),
                )
            })?;
            Some(s)
        } else {
            None
        }
    } else {
        None
    };

    let element_order = match args.get("element_order") {
        None | Some(Value::Null) => mcp_cad_fea::ElementOrder::Linear,
        Some(v) => {
            if let Some(n) = v.as_u64() {
                match n {
                    1 => mcp_cad_fea::ElementOrder::Linear,
                    2 => mcp_cad_fea::ElementOrder::Quadratic,
                    other => {
                        return Err(ServerError::InvalidArgument(format!(
                            "Invalid 'element_order' value {other}: expected 1 (linear TET4) or 2 (quadratic TET10)"
                        )))
                    }
                }
            } else if let Some(f) = v.as_f64() {
                if (f - 1.0).abs() < 1e-9 {
                    mcp_cad_fea::ElementOrder::Linear
                } else if (f - 2.0).abs() < 1e-9 {
                    mcp_cad_fea::ElementOrder::Quadratic
                } else {
                    return Err(ServerError::InvalidArgument(format!(
                        "Invalid 'element_order' value {f}: expected 1 (linear TET4) or 2 (quadratic TET10)"
                    )));
                }
            } else if let Some(s) = v.as_str() {
                match s.trim().to_ascii_lowercase().as_str() {
                    "1" | "tet4" | "linear" => mcp_cad_fea::ElementOrder::Linear,
                    "2" | "tet10" | "quadratic" => mcp_cad_fea::ElementOrder::Quadratic,
                    other => {
                        return Err(ServerError::InvalidArgument(format!(
                            "Invalid 'element_order' value '{other}': expected 1 (linear TET4) or 2 (quadratic TET10)"
                        )))
                    }
                }
            } else {
                return Err(ServerError::InvalidArgument(
                    "Invalid 'element_order': expected 1 (linear TET4) or 2 (quadratic TET10)".to_string(),
                ));
            }
        }
    };

    let density = match args.get("mesh_density").and_then(|v| v.as_str()).map(|s| s.to_lowercase()).as_deref() {
        Some("fine") => MeshDensity::Fine,
        Some("medium") => MeshDensity::Medium,
        _ => MeshDensity::Coarse,
    };

    // 1. Validate surface mesh topology / generate initial tetrahedral mesh
    let base_tet_mesh = if convergence_study {
        TetrahedralMesh::from_mesh(&part.mesh, study_levels[0])?
    } else {
        TetrahedralMesh::from_mesh(&part.mesh, density)?
    };
    if base_tet_mesh.is_empty() {
        return Err(ServerError::Internal("Tetrahedral meshing produced an empty mesh".to_string()));
    }
    let initial_tet_mesh = match element_order {
        mcp_cad_fea::ElementOrder::Linear => base_tet_mesh,
        mcp_cad_fea::ElementOrder::Quadratic => base_tet_mesh.to_tet10(),
    };

    // 2. Setup boundary conditions (mesh-independent base constraints + optional positional point loads)
    let mut base_boundary = BoundaryConditions::new();
    let mut positional_loads: Vec<([f64; 3], [f64; 3])> = Vec::new();
    let mut dirichlet_added = false;

    // Check fixed_plane (origin and tolerance in active length_unit)
    if let Some(fp_val) = args.get("fixed_plane") {
        if !fp_val.is_null() {
            let fp = fp_val.as_object().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Invalid 'fixed_plane': expected an object with 'origin' and 'normal' arrays".to_string(),
                )
            })?;
            let orig_val = fp.get("origin").and_then(|v| v.as_array()).ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Invalid 'fixed_plane': missing or non-array 'origin' [x, y, z]".to_string(),
                )
            })?;
            let norm_val = fp.get("normal").and_then(|v| v.as_array()).ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Invalid 'fixed_plane': missing or non-array 'normal' [nx, ny, nz]".to_string(),
                )
            })?;
            let orig_raw = parse_finite_vec3(orig_val, "fixed_plane.origin")?;
            let normal = parse_finite_vec3(norm_val, "fixed_plane.normal")?;
            let norm_len_sq = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
            if norm_len_sq <= 1e-18 {
                return Err(ServerError::InvalidArgument(
                    "Invalid 'fixed_plane': 'normal' vector must be non-zero".to_string(),
                ));
            }
            let origin = [
                orig_raw[0] * k_to_m,
                orig_raw[1] * k_to_m,
                orig_raw[2] * k_to_m,
            ];
            let tolerance = if let Some(tol_val) = fp.get("tolerance") {
                if !tol_val.is_null() {
                    let t = tol_val.as_f64().filter(|v| v.is_finite() && *v > 0.0).ok_or_else(|| {
                        ServerError::InvalidArgument(
                            "Invalid 'fixed_plane': 'tolerance' must be a positive finite number".to_string(),
                        )
                    })?;
                    t * k_to_m
                } else {
                    1e-3
                }
            } else {
                1e-3
            };
            base_boundary.fix_plane(origin, normal, tolerance);
            dirichlet_added = true;
        }
    }

    // Check fixed_faces (supports both legacy AABB face strings and named CAD faces; rejects unrecognized/empty entries)
    if let Some(ff_val) = args.get("fixed_faces") {
        if !ff_val.is_null() {
            let faces = ff_val.as_array().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Invalid 'fixed_faces': expected a non-empty array of face names (e.g. ['min_x', 'cap_bottom'])"
                        .to_string(),
                )
            })?;
            if faces.is_empty() {
                return Err(ServerError::InvalidArgument(
                    "Unconstrained FEA model: 'fixed_faces' array cannot be empty; specify at least one fixed boundary face"
                        .to_string(),
                ));
            }
            for f in faces {
                if let Some(f_str) = f.as_str() {
                    if f_str.trim().is_empty() {
                        return Err(ServerError::InvalidArgument(
                            "Invalid 'fixed_faces': face name cannot be empty".to_string(),
                        ));
                    }
                    if let Some(face) = parse_aabb_face(f_str) {
                        base_boundary.fix_aabb_face(face, 1e-4);
                        dirichlet_added = true;
                    } else {
                        let (label, _, triangles) = resolve_face_triangles_and_info(&part.mesh, f, unit)?;
                        base_boundary.fix_surface_triangles(Some(label), triangles, 0.0, DofComponent::All);
                        dirichlet_added = true;
                    }
                } else {
                    let (label, _, triangles) = resolve_face_triangles_and_info(&part.mesh, f, unit)?;
                    base_boundary.fix_surface_triangles(Some(label), triangles, 0.0, DofComponent::All);
                    dirichlet_added = true;
                }
            }
        }
    }

    // Check 'fixed' parameter (array or single FaceSelector object/string)
    if let Some(fixed_val) = args.get("fixed") {
        if !fixed_val.is_null() {
            let fixed_items: Vec<&Value> = if let Some(arr) = fixed_val.as_array() {
                if arr.is_empty() {
                    return Err(ServerError::InvalidArgument(
                        "Unconstrained FEA model: 'fixed' array cannot be empty; specify at least one fixed boundary constraint"
                            .to_string(),
                    ));
                }
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

                let aabb_opt = item
                    .as_str()
                    .or_else(|| item.get("face").and_then(|v| v.as_str()))
                    .or_else(|| item.get("face_id").and_then(|v| v.as_str()))
                    .and_then(parse_aabb_face);

                if let Some(aabb_face) = aabb_opt {
                    base_boundary.dirichlet.push(
                        mcp_cad_fea::boundary::DirichletConstraint::FixedAabbFace {
                            face: aabb_face,
                            tolerance: if tol_m > 0.0 { tol_m } else { 1e-4 },
                            components,
                        },
                    );
                    dirichlet_added = true;
                } else {
                    let (label, _, triangles) = resolve_face_triangles_and_info(&part.mesh, item, unit)?;
                    base_boundary.fix_surface_triangles(Some(label), triangles, tol_m, components);
                    dirichlet_added = true;
                }
            }
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
    let explicit_loads_key = args.get("applied_forces").is_some_and(|v| !v.is_null())
        || args.get("applied_force").is_some_and(|v| !v.is_null())
        || args.get("loads").is_some_and(|v| !v.is_null());

    // Helper closure to parse and register a single load object (from 'loads' or 'applied_forces')
    let mut process_load_obj = |load_obj: &Value| -> Result<(), ServerError> {
        if !load_obj.is_object() {
            return Err(ServerError::InvalidArgument(
                "Invalid load entry in 'applied_forces' / 'loads': expected a JSON object".to_string(),
            ));
        }

        let has_face_selector = load_obj.get("face").is_some()
            || load_obj.get("faces").is_some()
            || load_obj.get("face_id").is_some()
            || load_obj.get("where").is_some();

        let tol_m = load_obj
            .get("tolerance")
            .and_then(|v| v.as_f64())
            .map(|t| t * k_to_m)
            .unwrap_or(0.0);

        let f_raw_opt = load_obj
            .get("force")
            .or_else(|| load_obj.get("force_vector"))
            .or_else(|| load_obj.get("total_force"));

        let pressure_pa_opt = if let Some(p_val) = load_obj
            .get("pressure")
            .or_else(|| load_obj.get("pressure_mpa"))
        {
            if !p_val.is_null() {
                let p_mpa = p_val.as_f64().filter(|v| v.is_finite()).ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Load 'pressure' must be a finite number in MPa".to_string(),
                    )
                })?;
                Some(p_mpa * 1.0e6)
            } else {
                None
            }
        } else if let Some(p_val) = load_obj.get("pressure_pa") {
            if !p_val.is_null() {
                let p_pa = p_val.as_f64().filter(|v| v.is_finite()).ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Load 'pressure_pa' must be a finite number in Pa".to_string(),
                    )
                })?;
                Some(p_pa)
            } else {
                None
            }
        } else {
            None
        };

        let moment_val_opt = load_obj
            .get("moment")
            .or_else(|| load_obj.get("torque"))
            .or_else(|| load_obj.get("moment_nm"))
            .filter(|v| !v.is_null());

        if has_face_selector {
            let aabb_opt = load_obj
                .get("face")
                .and_then(|v| v.as_str())
                .or_else(|| load_obj.get("face_id").and_then(|v| v.as_str()))
                .and_then(parse_aabb_face);

            if let Some(aabb_face) = aabb_opt {
                if pressure_pa_opt.is_none() && moment_val_opt.is_none() {
                    let f_raw = f_raw_opt.ok_or_else(|| {
                        ServerError::InvalidArgument(
                            "Each entry in 'loads' / 'applied_forces' must specify 'force' / 'force_vector' ([Fx, Fy, Fz] in N)".to_string(),
                        )
                    })?;
                    let f_vec = f_raw.as_array().ok_or_else(|| {
                        ServerError::InvalidArgument(
                            "Load 'force' must be a 3-element array [Fx, Fy, Fz] in Newtons (N)".to_string(),
                        )
                    })?;
                    let force = parse_finite_vec3(f_vec, "force")?;
                    base_boundary.add_aabb_face_force(
                        aabb_face,
                        if tol_m > 0.0 { tol_m } else { 1e-4 },
                        force,
                    );
                    loads_added = true;
                    return Ok(());
                }
            }

            let (label, matched_faces, triangles) =
                resolve_face_triangles_and_info(&part.mesh, load_obj, unit)?;

            let mut applied_any_on_face = false;

            if let Some(f_raw) = f_raw_opt {
                if !f_raw.is_null() {
                    let f_vec = f_raw.as_array().ok_or_else(|| {
                        ServerError::InvalidArgument(
                            "Load 'force' must be a 3-element array [Fx, Fy, Fz] in Newtons (N)".to_string(),
                        )
                    })?;
                    let force = parse_finite_vec3(f_vec, "force")?;
                    base_boundary.add_surface_triangles_force(Some(label.clone()), triangles.clone(), tol_m, force);
                    loads_added = true;
                    applied_any_on_face = true;
                }
            }

            if let Some(pressure_pa) = pressure_pa_opt {
                base_boundary.add_surface_triangles_pressure(Some(label.clone()), triangles.clone(), tol_m, pressure_pa);
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
                    parse_finite_vec3(m_arr, "moment")?
                } else if let Some(m_scalar) = m_val.as_f64().filter(|v| v.is_finite()) {
                    let axis = matched_faces
                        .first()
                        .map(|f| f.normal_or_axis)
                        .unwrap_or([0.0, 0.0, 1.0]);
                    [axis[0] * m_scalar, axis[1] * m_scalar, axis[2] * m_scalar]
                } else {
                    return Err(ServerError::InvalidArgument(
                        "Load 'moment' must be a 3-element array [Mx, My, Mz] in N*m or a finite scalar".to_string(),
                    ));
                };

                base_boundary.add_surface_triangles_moment(Some(label), triangles, tol_m, moment_nm);
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

        if pressure_pa_opt.is_some() || moment_val_opt.is_some() {
            return Err(ServerError::InvalidArgument(
                "Applying 'pressure' or 'moment' in 'loads' / 'applied_forces' requires a target face selector ('face', 'faces', 'face_id', or 'where')".to_string(),
            ));
        }

        let f_raw = f_raw_opt.filter(|v| !v.is_null()).ok_or_else(|| {
            ServerError::InvalidArgument(
                "At least one applied force or pressure must be specified: each entry in 'applied_forces' must include 'force_vector' (or 'force') as [Fx, Fy, Fz] in Newtons (N)".to_string(),
            )
        })?;
        let f_vec = f_raw.as_array().ok_or_else(|| {
            ServerError::InvalidArgument(
                "Load 'force_vector' / 'force' in 'applied_forces' must be a 3-element array [Fx, Fy, Fz] in Newtons (N)".to_string(),
            )
        })?;
        let force = parse_finite_vec3(f_vec, "force_vector")?;

        if let Some(pos_raw) = load_obj.get("position") {
            if !pos_raw.is_null() {
                let pos_val = pos_raw.as_array().ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Invalid 'position' in 'applied_forces': expected a 3-element coordinate array [x, y, z]".to_string(),
                    )
                })?;
                let pos_coords = parse_finite_vec3(pos_val, "position")?;
                let target_pos = [
                    pos_coords[0] * k_to_m,
                    pos_coords[1] * k_to_m,
                    pos_coords[2] * k_to_m,
                ];
                positional_loads.push((target_pos, force));
                loads_added = true;
            } else {
                base_boundary.add_aabb_face_force(AabbFace::MaxX, 1e-4, force);
                loads_added = true;
            }
        } else {
            // No explicit position or face selector: apply to MaxX face
            base_boundary.add_aabb_face_force(AabbFace::MaxX, 1e-4, force);
            loads_added = true;
        }
        Ok(())
    };

    // Check 'loads' parameter (array or single object)
    if let Some(loads_val) = args.get("loads") {
        if !loads_val.is_null() {
            let loads_list: Vec<&Value> = if let Some(arr) = loads_val.as_array() {
                arr.iter().collect()
            } else if loads_val.is_object() {
                vec![loads_val]
            } else {
                return Err(ServerError::InvalidArgument(
                    "Invalid 'loads': expected a non-empty array of load objects or a single load object".to_string(),
                ));
            };
            for load_obj in loads_list {
                process_load_obj(load_obj)?;
            }
        }
    }

    // Check applied_forces array or applied_force object
    if let Some(af_val) = args.get("applied_forces") {
        if !af_val.is_null() {
            let arr = af_val.as_array().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Invalid 'applied_forces': expected a non-empty array of load objects".to_string(),
                )
            })?;
            for force_obj in arr {
                process_load_obj(force_obj)?;
            }
        }
    }
    if let Some(single) = args.get("applied_force") {
        if !single.is_null() {
            process_load_obj(single)?;
        }
    }

    // Reject explicit empty loads; only apply default load when neither loads, applied_forces, nor applied_force was passed
    if !loads_added {
        if explicit_loads_key {
            return Err(ServerError::InvalidArgument(
                "At least one applied force or pressure must be specified in 'applied_forces' or 'loads'".to_string(),
            ));
        }
        base_boundary.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, -1000.0, 0.0]);
    }

    // 3. Execute single-level FEA or multi-level convergence study
    let part_revision = part.revision;
    let fea_case_opt = args
        .get("fea_case")
        .or_else(|| args.get("case"))
        .and_then(|v| v.as_str())
        .map(|s| s.to_string());

    let (report, tet_mesh, study_report_opt) = if convergence_study {
        let surface_mesh = if part.mesh.indices.is_empty() && !part.dag.nodes.is_empty() {
            let settings = mcp_cad_core::tessellation::TessellationSettings::default();
            part.dag
                .evaluate_with_settings(&settings)
                .map_err(|e| ServerError::Internal(e.to_string()))?
        } else {
            part.mesh.clone()
        };
        let (rep, mesh, study_rep) = mcp_cad_fea::run_fea_with_convergence_with_order(
            part_id,
            &surface_mesh,
            &part.material,
            &study_levels,
            element_order,
            max_seconds,
            None,
            |m| Ok(bind_positional_loads_to_mesh(&base_boundary, &positional_loads, m)),
        )?;
        (rep, mesh, Some(study_rep))
    } else {
        let boundary = bind_positional_loads_to_mesh(&base_boundary, &positional_loads, &initial_tet_mesh);
        let rep = mcp_cad_fea::analyze_mesh(part_id, &initial_tet_mesh, &part.material, &boundary, None)?;
        (rep, initial_tet_mesh, None)
    };

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

    let mut out = json!({
        "max_von_mises_mpa": report.max_stress_von_mises_pa / 1.0e6,
        "max_stress_excluding_constraint_zone_mpa": report.diagnostics.max_stress_excluding_constraint_zone / 1.0e6,
        "yield_strength_mpa": report.yield_strength_pa / 1.0e6,
        "safety_factor": report.factor_of_safety,
        "max_displacement_mm": report.max_displacement_m * 1000.0,
        "status": report.verdict,
        "verdict": report.verdict,
        "confidence": report.confidence,
        "diagnostics": report.diagnostics,
        "warnings": report.diagnostics.notes,
        "element_order": report.element_order,
        "node_count": report.node_count,
        "element_count": report.element_count,
        "computation_time_ms": report.computation_time_ms,
        "part_revision": part_revision,
        "fea_case": fea_case_opt,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    });

    if let Some(ref acc) = report.accuracy {
        if let Some(obj) = out.as_object_mut() {
            obj.insert("accuracy".to_string(), serde_json::to_value(acc).unwrap_or(Value::Null));
        }
    }
    if let Some(ref study) = study_report_opt {
        if let Some(obj) = out.as_object_mut() {
            obj.insert(
                "convergence_study".to_string(),
                serde_json::to_value(study).unwrap_or(Value::Null),
            );
        }
    }

    Ok(out)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::cad::{add_feature_primitive, create_part};

    #[test]
    fn test_p2_t16_fea_rejects_unrecognized_fixed_faces_and_empty_or_invalid_applied_forces() {
        let mut state = EngineState::new().expect("EngineState init");
        let p_res = create_part(
            &json!({
                "part_id": "beam_t16",
                "name": "BeamT16",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();
        assert_eq!(p_res["part_id"], "beam_t16");

        add_feature_primitive(
            &json!({
                "part_id": "beam_t16",
                "primitive_type": "box",
                "width": 100.0,
                "height": 20.0,
                "depth": 20.0
            }),
            &mut state,
        )
        .unwrap();

        // 1. Typo in fixed_faces with trailing whitespace ("min_z ") must return InvalidArgument (FACE_NOT_FOUND)
        let err_typo = run_fea_analysis(
            &json!({
                "part_id": "beam_t16",
                "fixed_faces": ["min_z "],
                "applied_forces": [{"force_vector": [0.0, -500.0, 0.0]}]
            }),
            &mut state,
        )
        .expect_err("Unrecognized fixed_faces entry 'min_z ' must fail");
        let body_typo = err_typo.to_error_body();
        assert_eq!(body_typo.code, "FACE_NOT_FOUND");

        // 2. Empty fixed_faces array must fail with FEA_UNCONSTRAINED
        let err_empty_ff = run_fea_analysis(
            &json!({
                "part_id": "beam_t16",
                "fixed_faces": [],
                "applied_forces": [{"force_vector": [0.0, -500.0, 0.0]}]
            }),
            &mut state,
        )
        .expect_err("Empty fixed_faces array must fail");
        assert_eq!(err_empty_ff.to_error_body().code, "FEA_UNCONSTRAINED");

        // 3. Empty applied_forces: [] must fail with FEA_NO_LOADS instead of returning 0.0 MPa PASS
        let err_empty_loads = run_fea_analysis(
            &json!({
                "part_id": "beam_t16",
                "fixed_faces": ["min_x"],
                "applied_forces": []
            }),
            &mut state,
        )
        .expect_err("Empty applied_forces array must fail");
        let body_empty_loads = err_empty_loads.to_error_body();
        assert_eq!(body_empty_loads.code, "FEA_NO_LOADS");
        assert_eq!(body_empty_loads.param.as_deref(), Some("applied_forces"));

        // 4. Non-empty applied_forces with unrecognized force key (no force/force_vector) must fail with FEA_NO_LOADS
        let err_bad_key = run_fea_analysis(
            &json!({
                "part_id": "beam_t16",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"wrong_key": [0.0, -500.0, 0.0]}]
            }),
            &mut state,
        )
        .expect_err("applied_forces entry without force/force_vector must fail");
        let body_bad_key = err_bad_key.to_error_body();
        assert_eq!(body_bad_key.code, "FEA_NO_LOADS");
        assert_eq!(body_bad_key.param.as_deref(), Some("applied_forces"));

        // 5. Malformed position (2 elements instead of 3) in applied_forces must fail with INVALID_ARGUMENT
        let err_bad_pos = run_fea_analysis(
            &json!({
                "part_id": "beam_t16",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"position": [50.0, 0.0], "force_vector": [0.0, -500.0, 0.0]}]
            }),
            &mut state,
        )
        .expect_err("2-element position in applied_forces must fail");
        let body_bad_pos = err_bad_pos.to_error_body();
        assert_eq!(body_bad_pos.code, "INVALID_ARGUMENT");
        assert_eq!(body_bad_pos.param.as_deref(), Some("position"));

        // 6. Valid applied_forces with 'force' alias or 'face_id': 'max_x' succeeds with positive stress
        let ok_res = run_fea_analysis(
            &json!({
                "part_id": "beam_t16",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force": [0.0, -500.0, 0.0]}]
            }),
            &mut state,
        )
        .expect("Valid applied_forces with face_id='max_x' and force must succeed");
        assert!(ok_res["max_von_mises_mpa"].as_f64().unwrap() > 0.0);
    }

    #[test]
    fn test_p3_t2_run_fea_analysis_diagnostics_confidence_and_mesh_invalid() {
        let mut state = EngineState::new().expect("EngineState init");
        create_part(
            &json!({
                "part_id": "beam_p3t2",
                "name": "BeamP3T2",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();

        add_feature_primitive(
            &json!({
                "part_id": "beam_p3t2",
                "primitive_type": "box",
                "width": 100.0,
                "height": 20.0,
                "depth": 20.0
            }),
            &mut state,
        )
        .unwrap();

        // 1. Valid cantilever beam returns diagnostics, confidence, and max_stress_excluding_constraint_zone_mpa
        let res = run_fea_analysis(
            &json!({
                "part_id": "beam_p3t2",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force_vector": [0.0, -1000.0, 0.0]}],
                "mesh_density": "medium"
            }),
            &mut state,
        )
        .expect("Cantilever run_fea_analysis must succeed");

        assert_eq!(res["verdict"], "PASS");
        assert_eq!(res["confidence"], "low");
        let diag = &res["diagnostics"];
        assert_eq!(diag["mesh_closed"], true);
        let eq_err = diag["equilibrium_error_pct"].as_f64().unwrap();
        assert!(
            eq_err < 1e-4,
            "equilibrium_error_pct on cantilever must be < 1e-4, got {eq_err}"
        );
        assert_eq!(diag["constrained_rigid_body_modes_ok"], true);
        assert_eq!(diag["critical_node_near_constraint"], true);
        let excl_mpa = res["max_stress_excluding_constraint_zone_mpa"]
            .as_f64()
            .unwrap();
        assert!(excl_mpa > 0.0);
        assert!(diag["element_quality"]["min"].as_f64().unwrap() > 0.0);
        assert!(diag["element_quality"]["mean"].as_f64().unwrap() > 0.0);
        let notes = diag["notes"].as_array().unwrap();
        assert!(
            notes
                .iter()
                .any(|n| n.as_str().unwrap_or("").contains("CONSTRAINT_SINGULARITY")),
            "notes must contain CONSTRAINT_SINGULARITY warning: {notes:?}"
        );

        // 2. Corrupt the part mesh by dropping one triangle -> non-watertight mesh -> MESH_INVALID
        {
            let part = state.parts.get_mut("beam_p3t2").unwrap();
            part.mesh.indices.pop();
        }

        let err_open = run_fea_analysis(
            &json!({
                "part_id": "beam_p3t2",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force_vector": [0.0, -1000.0, 0.0]}]
            }),
            &mut state,
        )
        .expect_err("Non-watertight mesh must be rejected with MESH_INVALID");
        let err_body = err_open.to_error_body();
        assert_eq!(err_body.code, "MESH_INVALID");
        assert_eq!(err_body.param.as_deref(), Some("part_id"));
        let details = &err_body.details;
        assert!(
            details["boundary_edges_count"].as_u64().unwrap_or(0) >= 3,
            "details must report boundary_edges_count >= 3, got {details}"
        );
        assert_eq!(details["is_watertight"], false);
        assert!(err_body.hint.is_some());
    }

    #[test]
    fn test_p3_t3_run_fea_analysis_convergence_study_and_timeout() {
        let mut state = EngineState::new().expect("EngineState init");
        create_part(
            &json!({
                "part_id": "beam_p3t3",
                "name": "BeamP3T3",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();

        add_feature_primitive(
            &json!({
                "part_id": "beam_p3t3",
                "primitive_type": "box",
                "width": 200.0,
                "height": 20.0,
                "depth": 20.0
            }),
            &mut state,
        )
        .unwrap();

        // 1. Default convergence_study: true runs ["coarse", "medium", "fine"] and populates accuracy + convergence_study
        let res = run_fea_analysis(
            &json!({
                "part_id": "beam_p3t3",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force_vector": [0.0, -1000.0, 0.0]}],
                "convergence_study": true
            }),
            &mut state,
        )
        .expect("run_fea_analysis with convergence_study=true must succeed");

        let acc = &res["accuracy"];
        assert!(acc.is_object(), "Response must include accuracy object: {res}");
        assert_eq!(acc["converged"], false);
        assert_eq!(acc["verdict"], "NOT_CONVERGED");
        assert_eq!(acc["partial_result"], false);
        assert_eq!(res["confidence"], "low");

        let levels = acc["levels"].as_array().expect("accuracy.levels must be an array");
        assert_eq!(levels.len(), 3);
        assert_eq!(levels[0]["density"], "coarse");
        assert_eq!(levels[1]["density"], "medium");
        assert_eq!(levels[2]["density"], "fine");
        assert!(levels[0]["element_count"].as_u64().unwrap() < levels[1]["element_count"].as_u64().unwrap());
        assert!(levels[1]["element_count"].as_u64().unwrap() < levels[2]["element_count"].as_u64().unwrap());
        assert!(levels[2]["max_von_mises_mpa"].as_f64().unwrap() > 0.0);
        assert!(levels[2]["max_displacement_mm"].as_f64().unwrap() > 0.0);
        assert!(acc["stress_change_pct"].as_f64().is_some());
        assert!(acc["displacement_change_pct"].as_f64().is_some());
        assert!(acc["recommendation"].as_str().unwrap().contains("NOT_CONVERGED"));

        // 2. Custom levels ["coarse", "medium"] with max_seconds = 0.0 triggers partial_result = true
        let res_partial = run_fea_analysis(
            &json!({
                "part_id": "beam_p3t3",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force_vector": [0.0, -1000.0, 0.0]}],
                "convergence_study": true,
                "levels": ["coarse", "medium"],
                "max_seconds": 0.0
            }),
            &mut state,
        )
        .expect("run_fea_analysis with max_seconds=0.0 must return partial result");

        let acc_p = &res_partial["accuracy"];
        assert_eq!(acc_p["partial_result"], true);
        assert_eq!(acc_p["converged"], false);
        assert_eq!(acc_p["verdict"], "NOT_CONVERGED");
        assert_eq!(res_partial["confidence"], "low");
        assert_eq!(acc_p["levels"].as_array().unwrap().len(), 1);

        // 3. Invalid levels (< 2 entries or unknown level name) must return INVALID_ARGUMENT
        let err_levels = run_fea_analysis(
            &json!({
                "part_id": "beam_p3t3",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force_vector": [0.0, -1000.0, 0.0]}],
                "convergence_study": true,
                "levels": ["coarse"]
            }),
            &mut state,
        )
        .expect_err("Single level in levels must fail");
        assert_eq!(err_levels.to_error_body().code, "INVALID_ARGUMENT");

        let err_bad_level = run_fea_analysis(
            &json!({
                "part_id": "beam_p3t3",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force_vector": [0.0, -1000.0, 0.0]}],
                "convergence_study": true,
                "levels": ["coarse", "ultra_fine"]
            }),
            &mut state,
        )
        .expect_err("Unknown level name in levels must fail");
        assert_eq!(err_bad_level.to_error_body().code, "INVALID_ARGUMENT");
    }

    #[test]
    fn test_p3_t4_run_fea_analysis_tet10_element_order() {
        let mut state = EngineState::new().expect("EngineState init");
        create_part(
            &json!({
                "part_id": "beam_p3t4",
                "name": "BeamP3T4",
                "material": "steel_structural_aisi1020"
            }),
            &mut state,
        )
        .unwrap();

        add_feature_primitive(
            &json!({
                "part_id": "beam_p3t4",
                "primitive_type": "box",
                "width": 200.0,
                "height": 20.0,
                "depth": 20.0
            }),
            &mut state,
        )
        .unwrap();

        // 1. Default element_order = 1 (TET4) on Medium mesh
        let res_tet4 = run_fea_analysis(
            &json!({
                "part_id": "beam_p3t4",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force_vector": [0.0, -1000.0, 0.0]}],
                "mesh_density": "medium"
            }),
            &mut state,
        )
        .expect("TET4 run_fea_analysis must succeed");
        assert_eq!(res_tet4["element_order"], 1);

        // 2. Quadratic element_order = 2 (TET10) on Medium mesh
        let res_tet10 = run_fea_analysis(
            &json!({
                "part_id": "beam_p3t4",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force_vector": [0.0, -1000.0, 0.0]}],
                "mesh_density": "medium",
                "element_order": 2
            }),
            &mut state,
        )
        .expect("TET10 run_fea_analysis must succeed");
        assert_eq!(res_tet10["element_order"], 2);
        assert!(
            res_tet10["node_count"].as_u64().unwrap() > res_tet4["node_count"].as_u64().unwrap(),
            "TET10 mesh must have mid-edge nodes (> TET4 node_count)"
        );
        assert_eq!(res_tet10["element_count"], res_tet4["element_count"]);

        // Analytical deflection for 200x20x20mm steel cantilever (E = 200 GPa, F = -1000 N):
        // delta_eb = F * L^3 / (3 * E * I) = 1.0000 mm (Timoshenko = 1.0078 mm).
        let disp_tet4 = res_tet4["max_displacement_mm"].as_f64().unwrap();
        let disp_tet10 = res_tet10["max_displacement_mm"].as_f64().unwrap();
        let err_tet4_pct = ((disp_tet4 - 1.0).abs() / 1.0) * 100.0;
        let err_tet10_pct = ((disp_tet10 - 1.0).abs() / 1.0) * 100.0;
        assert!(
            err_tet10_pct < 5.0 && err_tet10_pct < err_tet4_pct,
            "TET10 deflection error ({err_tet10_pct:.2}%) must be < 5% and lower than TET4 ({err_tet4_pct:.2}%)"
        );

        // 3. Invalid element_order = 3 must return INVALID_ARGUMENT
        let err_order = run_fea_analysis(
            &json!({
                "part_id": "beam_p3t4",
                "fixed_faces": ["min_x"],
                "applied_forces": [{"face_id": "max_x", "force_vector": [0.0, -1000.0, 0.0]}],
                "element_order": 3
            }),
            &mut state,
        )
        .expect_err("element_order = 3 must be rejected");
        assert_eq!(err_order.to_error_body().code, "INVALID_ARGUMENT");
    }
}

