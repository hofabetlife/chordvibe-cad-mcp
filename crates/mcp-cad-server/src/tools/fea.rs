//! Finite Element Analysis (FEA) and stress heatmap visualization tools.

use serde_json::{json, Value};

use mcp_cad_core::topology::{TriangleMesh, Vertex};
use mcp_cad_fea::boundary::{AabbFace, BoundaryConditions, NeumannLoad};
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

/// 11. `run_fea_analysis`: Tetrahedralizes a part, solves 3D linear elasticity, and computes FoS.
pub fn run_fea_analysis(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let part_id = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .ok_or_else(|| ServerError::InvalidArgument("Missing required argument 'part_id'".to_string()))?;

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

    // Check fixed_plane
    if let Some(fp) = args.get("fixed_plane").and_then(|v| v.as_object()) {
        if let (Some(orig_val), Some(norm_val)) = (fp.get("origin").and_then(|v| v.as_array()), fp.get("normal").and_then(|v| v.as_array())) {
            if orig_val.len() == 3 && norm_val.len() == 3 {
                let origin = [
                    orig_val[0].as_f64().unwrap_or(0.0),
                    orig_val[1].as_f64().unwrap_or(0.0),
                    orig_val[2].as_f64().unwrap_or(0.0),
                ];
                let normal = [
                    norm_val[0].as_f64().unwrap_or(1.0),
                    norm_val[1].as_f64().unwrap_or(0.0),
                    norm_val[2].as_f64().unwrap_or(0.0),
                ];
                let tolerance = fp.get("tolerance").and_then(|v| v.as_f64()).unwrap_or(1e-3);
                boundary.fix_plane(origin, normal, tolerance);
                dirichlet_added = true;
            }
        }
    }

    // Check fixed_faces
    if let Some(faces) = args.get("fixed_faces").and_then(|v| v.as_array()) {
        for f in faces {
            if let Some(f_str) = f.as_str() {
                if let Some(face) = parse_aabb_face(f_str) {
                    boundary.fix_aabb_face(face, 1e-4);
                    dirichlet_added = true;
                }
            }
        }
    }

    // Default constraint: fix MinX boundary if nothing specified
    if !dirichlet_added {
        boundary.fix_aabb_face(AabbFace::MinX, 1e-4);
    }

    // Applied loads
    let mut loads_added = false;

    // Check applied_forces array
    let forces_list: Vec<&Value> = if let Some(arr) = args.get("applied_forces").and_then(|v| v.as_array()) {
        arr.iter().collect()
    } else if let Some(single) = args.get("applied_force") {
        vec![single]
    } else {
        Vec::new()
    };

    for force_obj in forces_list {
        let f_vec_val = force_obj.get("force_vector").and_then(|v| v.as_array());
        if let Some(f_vec) = f_vec_val {
            if f_vec.len() == 3 {
                let force = [
                    f_vec[0].as_f64().unwrap_or(0.0),
                    f_vec[1].as_f64().unwrap_or(0.0),
                    f_vec[2].as_f64().unwrap_or(0.0),
                ];

                if let Some(pos_val) = force_obj.get("position").and_then(|v| v.as_array()) {
                    if pos_val.len() == 3 {
                        let target_pos = [
                            pos_val[0].as_f64().unwrap_or(0.0),
                            pos_val[1].as_f64().unwrap_or(0.0),
                            pos_val[2].as_f64().unwrap_or(0.0),
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
                    // No explicit position: apply to MaxX face
                    boundary.add_aabb_face_force(AabbFace::MaxX, 1e-4, force);
                    loads_added = true;
                }
            }
        }
    }

    // Default load: 1000 N downward load (-Y) on MaxX face if none specified
    if !loads_added {
        boundary.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, -1000.0, 0.0]);
    }

    // 3. Execute analysis
    let report = mcp_cad_fea::analyze_mesh(part_id, &tet_mesh, &part.material, &boundary, None)?;

    state.last_fea_reports.insert(part_id.to_string(), report.clone());
    state.last_fea_meshes.insert(part_id.to_string(), tet_mesh);

    Ok(json!({
        "max_von_mises_mpa": report.max_stress_von_mises_pa / 1.0e6,
        "yield_strength_mpa": report.yield_strength_pa / 1.0e6,
        "safety_factor": report.factor_of_safety,
        "max_displacement_mm": report.max_displacement_m * 1000.0,
        "status": report.verdict,
        "node_count": report.node_count,
        "element_count": report.element_count,
        "computation_time_ms": report.computation_time_ms
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

    Ok(json!({
        "image_base64": image_base64,
        "colormap_legend": legend
    }))
}
