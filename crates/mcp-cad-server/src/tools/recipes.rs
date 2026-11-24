//! Engineering assembly and part recipe tools (`apply_recipe`).
//!
//! Provides high-level parametric recipes built cleanly atop existing DAG primitives,
//! CSG booleans, mesh validation, and `f64` DAE kinematic joints while preserving
//! active `length_unit` semantics, transactional state rollback, and `ErrorEnvelope` contracts.

use std::f64::consts::PI;

use serde_json::{Value, json};

use crate::error::ServerError;
use crate::state::{EngineState, LengthUnit};

use super::{cad, physics, requirements};

/// Snapshot of mutable engine state for transactional rollback on recipe failure.
struct StateSnapshot {
    length_unit: LengthUnit,
    parts: std::collections::HashMap<String, mcp_cad_core::Part>,
    materials: mcp_cad_core::MaterialRegistry,
    physics_world: mcp_cad_physics::PhysicsWorld,
    last_fea_reports: std::collections::HashMap<String, mcp_cad_fea::FeaReport>,
    last_fea_revisions: std::collections::HashMap<String, u64>,
    fea_case_reports: std::collections::HashMap<(String, String), (mcp_cad_fea::FeaReport, u64)>,
    last_fea_meshes: std::collections::HashMap<String, mcp_cad_fea::TetrahedralMesh>,
    last_simulation: Option<mcp_cad_physics::SimulationSummary>,
    requirements: crate::tools::requirements::RequirementsState,
}

impl StateSnapshot {
    fn capture(state: &EngineState) -> Self {
        Self {
            length_unit: state.length_unit,
            parts: state.parts.clone(),
            materials: state.materials.clone(),
            physics_world: state.physics_world.clone(),
            last_fea_reports: state.last_fea_reports.clone(),
            last_fea_revisions: state.last_fea_revisions.clone(),
            fea_case_reports: state.fea_case_reports.clone(),
            last_fea_meshes: state.last_fea_meshes.clone(),
            last_simulation: state.last_simulation.clone(),
            requirements: state.requirements.clone(),
        }
    }

    fn restore(self, state: &mut EngineState) {
        state.length_unit = self.length_unit;
        state.parts = self.parts;
        state.materials = self.materials;
        state.physics_world = self.physics_world;
        state.last_fea_reports = self.last_fea_reports;
        state.last_fea_revisions = self.last_fea_revisions;
        state.fea_case_reports = self.fea_case_reports;
        state.last_fea_meshes = self.last_fea_meshes;
        state.last_simulation = self.last_simulation;
        state.requirements = self.requirements;
    }
}

/// Scales a millimeter default value into the active `LengthUnit`.
#[inline]
fn mm_to_unit(mm_val: f64, unit: LengthUnit) -> f64 {
    mm_val * unit.from_meters_factor() * 1e-3
}

/// Parses an optional positive finite scalar from `params`, defaulting from millimeters.
fn get_positive_dim(
    params: &Value,
    key: &str,
    default_mm: f64,
    unit: LengthUnit,
) -> Result<f64, ServerError> {
    match params.get(key) {
        None | Some(Value::Null) => Ok(mm_to_unit(default_mm, unit)),
        Some(v) => {
            let val = v.as_f64().ok_or_else(|| {
                ServerError::InvalidArgument(format!("Parameter '{key}' must be a finite number"))
            })?;
            if !val.is_finite() || val <= 0.0 {
                return Err(ServerError::InvalidArgument(format!(
                    "Parameter '{key}' must be finite and strictly positive (> 0), got {val}"
                )));
            }
            Ok(val)
        }
    }
}

/// Parses an optional finite scalar (may be zero/negative) from `params`.
fn get_finite_f64(params: &Value, key: &str, default_val: f64) -> Result<f64, ServerError> {
    match params.get(key) {
        None | Some(Value::Null) => Ok(default_val),
        Some(v) => {
            let val = v.as_f64().ok_or_else(|| {
                ServerError::InvalidArgument(format!("Parameter '{key}' must be a finite number"))
            })?;
            if !val.is_finite() {
                return Err(ServerError::InvalidArgument(format!(
                    "Parameter '{key}' must be finite, got {val}"
                )));
            }
            Ok(val)
        }
    }
}

/// Parses an optional non-empty string parameter from `params`.
fn get_nonempty_str(params: &Value, key: &str, default: &str) -> Result<String, ServerError> {
    match params.get(key) {
        None | Some(Value::Null) => Ok(default.to_string()),
        Some(v) => {
            let s = v.as_str().ok_or_else(|| {
                ServerError::InvalidArgument(format!("Parameter '{key}' must be a string"))
            })?;
            if s.trim().is_empty() {
                return Err(ServerError::InvalidArgument(format!(
                    "Parameter '{key}' must not be empty"
                )));
            }
            Ok(s.to_string())
        }
    }
}

/// Allocates a collision-free temporary part identifier inside `state.parts`.
fn unique_temp_part_id(state: &EngineState, base: &str) -> String {
    let mut candidate = format!("__recipe_tmp_{base}");
    let mut idx = 1usize;
    while state.parts.contains_key(&candidate) {
        candidate = format!("__recipe_tmp_{base}_{idx}");
        idx += 1;
    }
    candidate
}

/// Validates a generated part mesh via `cad::validate_mesh_tool` and rejects fatal defects.
fn validate_created_part(part_id: &str, state: &mut EngineState) -> Result<Value, ServerError> {
    let report = cad::validate_mesh_tool(&json!({ "part_id": part_id }), state)?;
    let is_watertight = report["topology"]["is_watertight"].as_bool().unwrap_or(false);
    let boundary_edges = report["topology"]["boundary_edges_count"]
        .as_u64()
        .unwrap_or(0);
    let non_manifold_edges = report["topology"]["non_manifold_edges_count"]
        .as_u64()
        .unwrap_or(0);
    let degenerate_triangles = report["geometry"]["degenerate_triangles_count"]
        .as_u64()
        .unwrap_or(0);
    let triangles_count = report["topology"]["triangles_count"]
        .as_u64()
        .unwrap_or(0);

    if !is_watertight || boundary_edges > 0 || non_manifold_edges > 0 {
        return Err(ServerError::Internal(format!(
            "Recipe geometry validation failed for part '{part_id}': is_watertight={is_watertight}, boundary_edges={boundary_edges}, non_manifold_edges={non_manifold_edges}, degenerate_triangles={degenerate_triangles}"
        )));
    }
    Ok(json!({
        "status": report["status"],
        "is_watertight": is_watertight,
        "boundary_edges_count": boundary_edges,
        "non_manifold_edges_count": non_manifold_edges,
        "degenerate_triangles_count": degenerate_triangles,
        "triangles_count": triangles_count
    }))
}

/// Result payload produced by an individual parametric recipe builder.
struct RecipeOutcome {
    created_parts: Vec<String>,
    created_joints: Vec<String>,
    details: Value,
    warnings: Vec<String>,
}

/// Extracts string warning messages from a nested CAD tool response.
fn extract_warnings(resp: &Value) -> Vec<String> {
    resp.get("warnings")
        .and_then(|v| v.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|v| v.as_str().map(|s| s.to_string()))
                .collect::<Vec<_>>()
        })
        .unwrap_or_default()
}

/// Recipe 1: Flanged bushing with central through-bore and flange bolt circle (`flanged_bushing`).
fn build_flanged_bushing(
    params: &Value,
    unit: LengthUnit,
    state: &mut EngineState,
) -> Result<RecipeOutcome, ServerError> {
    let part_id = get_nonempty_str(params, "part_id", "flanged_bushing")?;
    let name = get_nonempty_str(params, "name", "Flanged Bushing")?;
    let material = get_nonempty_str(params, "material", "steel_structural_aisi1020")?;
    let is_fixed = params
        .get("is_fixed")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

    let sleeve_od = get_positive_dim(params, "sleeve_outer_diameter", 30.0, unit)?;
    let sleeve_len = get_positive_dim(params, "sleeve_length", 50.0, unit)?;
    let bore_d = get_positive_dim(params, "bore_diameter", 16.0, unit)?;
    let flange_d = get_positive_dim(params, "flange_diameter", 64.0, unit)?;
    let flange_t = get_positive_dim(params, "flange_thickness", 10.0, unit)?;
    let pcd_d = get_positive_dim(params, "pcd_diameter", 48.0, unit)?;
    let bolt_d = get_positive_dim(params, "bolt_hole_diameter", 6.6, unit)?;

    let flange_pos = get_nonempty_str(params, "flange_position", "bottom")?;
    if !matches!(flange_pos.as_str(), "bottom" | "top" | "center") {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'flange_position' must be one of 'bottom', 'top', or 'center', got '{flange_pos}'"
        )));
    }

    let bolt_count = match params.get("bolt_holes_count") {
        None | Some(Value::Null) => 4usize,
        Some(v) => {
            let n = v.as_u64().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Parameter 'bolt_holes_count' must be a non-negative integer".to_string(),
                )
            })?;
            if n > 24 {
                return Err(ServerError::InvalidArgument(format!(
                    "Parameter 'bolt_holes_count' must be in 0..=24, got {n}"
                )));
            }
            n as usize
        }
    };

    // Geometric invariants
    let eps = 1e-6 * unit.from_meters_factor() * 1e-3;
    if bore_d >= sleeve_od - eps {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'bore_diameter' ({bore_d}) must be strictly smaller than 'sleeve_outer_diameter' ({sleeve_od})"
        )));
    }
    if flange_d <= sleeve_od + eps {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'flange_diameter' ({flange_d}) must be strictly greater than 'sleeve_outer_diameter' ({sleeve_od})"
        )));
    }
    if flange_t > sleeve_len + eps {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'flange_thickness' ({flange_t}) must not exceed 'sleeve_length' ({sleeve_len})"
        )));
    }
    if bolt_count > 0 {
        let r_sleeve = sleeve_od * 0.5;
        let r_flange = flange_d * 0.5;
        let r_pcd = pcd_d * 0.5;
        let r_bolt = bolt_d * 0.5;
        if r_pcd - r_bolt <= r_sleeve + eps {
            return Err(ServerError::InvalidArgument(format!(
                "Bolt holes intersect sleeve outer wall: ('pcd_diameter' - 'bolt_hole_diameter') / 2 ({:.4}) must exceed 'sleeve_outer_diameter' / 2 ({r_sleeve:.4})",
                r_pcd - r_bolt
            )));
        }
        if r_pcd + r_bolt >= r_flange - eps {
            return Err(ServerError::InvalidArgument(format!(
                "Bolt holes break out of flange rim: ('pcd_diameter' + 'bolt_hole_diameter') / 2 ({:.4}) must be smaller than 'flange_diameter' / 2 ({r_flange:.4})",
                r_pcd + r_bolt
            )));
        }
        if bolt_count >= 2 {
            let chord = 2.0 * r_pcd * (PI / (bolt_count as f64)).sin();
            if chord <= bolt_d + eps {
                return Err(ServerError::InvalidArgument(format!(
                    "Adjacent bolt holes overlap at 'bolt_holes_count'={bolt_count}: center-to-center spacing ({chord:.4}) must exceed 'bolt_hole_diameter' ({bolt_d:.4})"
                )));
            }
        }
    }

    let z_flange = match flange_pos.as_str() {
        "bottom" => -0.5 * sleeve_len + 0.5 * flange_t,
        "top" => 0.5 * sleeve_len - 0.5 * flange_t,
        _ => 0.0,
    };
    let r_bore = bore_d * 0.5;
    let r_sleeve = sleeve_od * 0.5;
    let r_flange = flange_d * 0.5;
    let z_min = -0.5 * sleeve_len;
    let z_max = 0.5 * sleeve_len;

    // Construct the coaxial sleeve + flange + through-bore as one clean annular revolve
    // profile to avoid unnecessary intermediate CSG seams and keep tessellation light.
    let profile: Vec<[f64; 2]> = if (flange_t - sleeve_len).abs() <= eps {
        vec![
            [r_bore, z_min],
            [r_flange, z_min],
            [r_flange, z_max],
            [r_bore, z_max],
        ]
    } else {
        match flange_pos.as_str() {
            "bottom" => vec![
                [r_bore, z_min],
                [r_flange, z_min],
                [r_flange, z_min + flange_t],
                [r_sleeve, z_min + flange_t],
                [r_sleeve, z_max],
                [r_bore, z_max],
            ],
            "top" => vec![
                [r_bore, z_min],
                [r_sleeve, z_min],
                [r_sleeve, z_max - flange_t],
                [r_flange, z_max - flange_t],
                [r_flange, z_max],
                [r_bore, z_max],
            ],
            _ => vec![
                [r_bore, z_min],
                [r_sleeve, z_min],
                [r_sleeve, -0.5 * flange_t],
                [r_flange, -0.5 * flange_t],
                [r_flange, 0.5 * flange_t],
                [r_sleeve, 0.5 * flange_t],
                [r_sleeve, z_max],
                [r_bore, z_max],
            ],
        }
    };
    let u_str = unit.as_str();

    // 1. Create part and build sleeve + flange + central bore in one revolve primitive
    cad::create_part(
        &json!({
            "part_id": part_id,
            "name": name,
            "material": material,
            "is_fixed": is_fixed
        }),
        state,
    )?;

    let body_resp = cad::add_feature_primitive(
        &json!({
            "part_id": part_id,
            "primitive_type": "revolve",
            "length_unit": u_str,
            "params": {
                "profile": profile,
                "angle_deg": 360.0
            }
        }),
        state,
    )?;

    // 2. Cut bolt circle holes cleanly through flange annulus
    let r_pcd = pcd_d * 0.5;
    for i in 0..bolt_count {
        let theta = 2.0 * PI * (i as f64) / (bolt_count as f64);
        let x = r_pcd * theta.cos();
        let y = r_pcd * theta.sin();
        cad::cut_hole(
            &json!({
                "part_id": part_id,
                "length_unit": u_str,
                "position": [x, y, z_flange],
                "normal": [0.0, 0.0, 1.0],
                "diameter": bolt_d,
                "through": true
            }),
            state,
        )?;
    }

    let mesh_report = validate_created_part(&part_id, state)?;
    let warnings = extract_warnings(&body_resp);

    let summary = json!({
        "part_id": part_id,
        "sleeve_outer_diameter": sleeve_od,
        "sleeve_length": sleeve_len,
        "bore_diameter": bore_d,
        "flange_diameter": flange_d,
        "flange_thickness": flange_t,
        "flange_position": flange_pos,
        "pcd_diameter": pcd_d,
        "bolt_holes_count": bolt_count,
        "bolt_hole_diameter": bolt_d,
        "mesh_validation": mesh_report
    });

    Ok(RecipeOutcome {
        created_parts: vec![part_id],
        created_joints: Vec::new(),
        details: summary,
        warnings,
    })
}

/// Parses shaft step list `[ { diameter, length }, ... ]` or returns defaults.
fn parse_shaft_steps(params: &Value, unit: LengthUnit) -> Result<Vec<(f64, f64)>, ServerError> {
    let raw_steps = match params.get("steps") {
        None | Some(Value::Null) => {
            return Ok(vec![
                (mm_to_unit(24.0, unit), mm_to_unit(30.0, unit)),
                (mm_to_unit(32.0, unit), mm_to_unit(40.0, unit)),
                (mm_to_unit(20.0, unit), mm_to_unit(30.0, unit)),
            ]);
        }
        Some(v) => v.as_array().ok_or_else(|| {
            ServerError::InvalidArgument(
                "Parameter 'steps' must be an array of {diameter, length} objects".to_string(),
            )
        })?,
    };

    if raw_steps.is_empty() || raw_steps.len() > 16 {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'steps' must contain between 1 and 16 steps, got {}",
            raw_steps.len()
        )));
    }

    let mut steps = Vec::with_capacity(raw_steps.len());
    for (idx, item) in raw_steps.iter().enumerate() {
        let d = item
            .get("diameter")
            .and_then(|v| v.as_f64())
            .ok_or_else(|| {
                ServerError::InvalidArgument(format!(
                    "Step {idx} in 'steps' requires a finite numeric 'diameter'"
                ))
            })?;
        let l = item
            .get("length")
            .and_then(|v| v.as_f64())
            .ok_or_else(|| {
                ServerError::InvalidArgument(format!(
                    "Step {idx} in 'steps' requires a finite numeric 'length'"
                ))
            })?;
        if !d.is_finite() || d <= 0.0 {
            return Err(ServerError::InvalidArgument(format!(
                "Step {idx} 'diameter' in 'steps' must be > 0, got {d}"
            )));
        }
        if !l.is_finite() || l <= 0.0 {
            return Err(ServerError::InvalidArgument(format!(
                "Step {idx} 'length' in 'steps' must be > 0, got {l}"
            )));
        }
        steps.push((d, l));
    }

    Ok(steps)
}

/// Recipe 2: Stepped shaft built via single-manifold `revolve` profile + external keyway slot.
fn build_stepped_shaft(
    params: &Value,
    unit: LengthUnit,
    state: &mut EngineState,
) -> Result<RecipeOutcome, ServerError> {
    let part_id = get_nonempty_str(params, "part_id", "stepped_shaft")?;
    let name = get_nonempty_str(params, "name", "Stepped Shaft")?;
    let material = get_nonempty_str(params, "material", "steel_high_strength_4140")?;
    let is_fixed = params
        .get("is_fixed")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

    let steps = parse_shaft_steps(params, unit)?;

    let keyway_step_index = match params.get("keyway_step_index") {
        None | Some(Value::Null) => 0usize,
        Some(v) => {
            let idx = v.as_u64().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Parameter 'keyway_step_index' must be a non-negative integer".to_string(),
                )
            })? as usize;
            if idx >= steps.len() {
                return Err(ServerError::InvalidArgument(format!(
                    "Parameter 'keyway_step_index' ({idx}) is out of range for {} steps",
                    steps.len()
                )));
            }
            idx
        }
    };

    let (target_d, target_l) = steps[keyway_step_index];
    let target_r = target_d * 0.5;
    let eps = 1e-6 * unit.from_meters_factor() * 1e-3;

    let kw_width = match params.get("keyway_width") {
        None | Some(Value::Null) => (target_d * 0.25).clamp(target_d * 0.15, target_d * 0.45),
        Some(_) => get_positive_dim(params, "keyway_width", 6.0, unit)?,
    };
    let kw_depth = match params.get("keyway_depth") {
        None | Some(Value::Null) => (target_d * 0.145).clamp(target_r * 0.1, target_r * 0.4),
        Some(_) => get_positive_dim(params, "keyway_depth", 3.5, unit)?,
    };
    let kw_length = match params.get("keyway_length") {
        None | Some(Value::Null) => target_l * 0.7,
        Some(_) => get_positive_dim(params, "keyway_length", 20.0, unit)?,
    };

    if kw_length >= target_l - eps {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'keyway_length' ({kw_length}) must be strictly shorter than step length ({target_l})"
        )));
    }
    if kw_width >= 1.6 * target_r {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'keyway_width' ({kw_width}) is too wide for step diameter ({target_d}); must be < 0.8 * diameter"
        )));
    }
    if kw_depth >= 0.8 * target_r {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'keyway_depth' ({kw_depth}) is too deep for step diameter ({target_d}); must be < 0.4 * diameter"
        )));
    }
    let chord_drop = target_r - (target_r * target_r - (kw_width * 0.5).powi(2)).sqrt();
    if kw_depth <= chord_drop + eps {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'keyway_depth' ({kw_depth:.4}) must exceed cylindrical chord drop ({chord_drop:.4}) for 'keyway_width'={kw_width:.4}"
        )));
    }

    // Build monotonic non-self-intersecting revolve profile centered along Z in [-L/2, +L/2]
    let total_length: f64 = steps.iter().map(|(_, l)| *l).sum();
    let mut z_cursor = -0.5 * total_length;
    let mut step_mid_z = 0.0;
    let mut profile: Vec<[f64; 2]> = Vec::with_capacity(steps.len() * 2 + 2);
    profile.push([0.0, z_cursor]);

    for (idx, &(d, l)) in steps.iter().enumerate() {
        let r = d * 0.5;
        let z_next = z_cursor + l;
        if idx == keyway_step_index {
            step_mid_z = 0.5 * (z_cursor + z_next);
        }
        // Skip duplicate radial transition when consecutive steps share diameter
        let push_start = profile
            .last()
            .map(|p| (p[0] - r).abs() > eps || (p[1] - z_cursor).abs() > eps)
            .unwrap_or(true);
        if push_start {
            profile.push([r, z_cursor]);
        }
        profile.push([r, z_next]);
        z_cursor = z_next;
    }
    profile.push([0.0, z_cursor]);

    let u_str = unit.as_str();
    cad::create_part(
        &json!({
            "part_id": part_id,
            "name": name,
            "material": material,
            "is_fixed": is_fixed
        }),
        state,
    )?;

    let rev_resp = cad::add_feature_primitive(
        &json!({
            "part_id": part_id,
            "primitive_type": "revolve",
            "length_unit": u_str,
            "params": {
                "profile": profile,
                "angle_deg": 360.0
            }
        }),
        state,
    )?;

    // Subtract external rectangular keyway cutter along +X on target step
    let tmp_cutter_id = unique_temp_part_id(state, &part_id);
    let overhang = target_r * 0.5 + mm_to_unit(2.0, unit);
    let cutter_width = kw_depth + overhang;
    let cutter_center_x = target_r + 0.5 * (overhang - kw_depth);

    cad::create_part(
        &json!({
            "part_id": tmp_cutter_id,
            "name": "Keyway Cutter",
            "material": material
        }),
        state,
    )?;
    cad::add_feature_primitive(
        &json!({
            "part_id": tmp_cutter_id,
            "primitive_type": "box",
            "length_unit": u_str,
            "params": {
                "width": cutter_width,
                "depth": kw_width,
                "height": kw_length,
                "position": [cutter_center_x, 0.0, step_mid_z]
            }
        }),
        state,
    )?;
    cad::boolean_difference(
        &json!({
            "target_part_id": part_id,
            "tool_part_id": tmp_cutter_id
        }),
        state,
    )?;
    cad::delete_part(&json!({ "part_id": tmp_cutter_id }), state)?;

    let mesh_report = validate_created_part(&part_id, state)?;
    let warnings = extract_warnings(&rev_resp);

    let steps_json: Vec<Value> = steps
        .iter()
        .map(|(d, l)| json!({ "diameter": d, "length": l }))
        .collect();
    let summary = json!({
        "part_id": part_id,
        "total_length": total_length,
        "steps": steps_json,
        "keyway": {
            "step_index": keyway_step_index,
            "width": kw_width,
            "depth": kw_depth,
            "length": kw_length,
            "center_z": step_mid_z
        },
        "mesh_validation": mesh_report
    });

    Ok(RecipeOutcome {
        created_parts: vec![part_id],
        created_joints: Vec::new(),
        details: summary,
        warnings,
    })
}

/// Parses integer tooth count with minimum bound.
fn get_teeth_count(params: &Value, key: &str, default_val: u64) -> Result<u64, ServerError> {
    match params.get(key) {
        None | Some(Value::Null) => Ok(default_val),
        Some(v) => {
            let n = v.as_u64().ok_or_else(|| {
                ServerError::InvalidArgument(format!(
                    "Parameter '{key}' must be a positive integer"
                ))
            })?;
            if !(8..=200).contains(&n) {
                return Err(ServerError::InvalidArgument(format!(
                    "Parameter '{key}' must be in 8..=200, got {n}"
                )));
            }
            Ok(n)
        }
    }
}

/// Recipe 3: Meshing spur gear pair mounted on a shared fixed frame with revolute + gear joints.
fn build_gear_transmission(
    params: &Value,
    unit: LengthUnit,
    state: &mut EngineState,
) -> Result<RecipeOutcome, ServerError> {
    let prefix = get_nonempty_str(params, "id_prefix", "gear_tx")?;
    let frame_id = get_nonempty_str(params, "frame_id", &format!("{prefix}_frame"))?;
    let pinion_id = get_nonempty_str(params, "pinion_id", &format!("{prefix}_pinion"))?;
    let gear_id = get_nonempty_str(params, "gear_id", &format!("{prefix}_gear"))?;

    if frame_id == pinion_id || frame_id == gear_id || pinion_id == gear_id {
        return Err(ServerError::InvalidArgument(
            "Parameters 'frame_id', 'pinion_id', and 'gear_id' must all be distinct".to_string(),
        ));
    }

    let material = get_nonempty_str(params, "material", "steel_structural_aisi1020")?;
    let frame_material = get_nonempty_str(params, "frame_material", &material)?;

    let module = get_positive_dim(params, "module", 2.0, unit)?;
    let teeth_pinion = get_teeth_count(params, "teeth_pinion", 20)?;
    let teeth_gear = get_teeth_count(params, "teeth_gear", 40)?;
    let face_width = get_positive_dim(params, "face_width", 15.0, unit)?;
    let frame_thickness = get_positive_dim(params, "frame_thickness", 12.0, unit)?;
    let frame_clearance = get_positive_dim(params, "frame_clearance", 4.0, unit)?;
    let pressure_angle_deg = get_finite_f64(params, "pressure_angle_deg", 20.0)?;
    if !(10.0..=35.0).contains(&pressure_angle_deg) {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'pressure_angle_deg' must be in [10.0, 35.0], got {pressure_angle_deg}"
        )));
    }

    let min_teeth = teeth_pinion.min(teeth_gear) as f64;
    let min_root_radius = (0.5 * module * min_teeth - 1.25 * module).max(0.1 * module);
    let bore_d = match params.get("bore_diameter") {
        None | Some(Value::Null) => (min_root_radius * 0.6).min(mm_to_unit(10.0, unit)),
        Some(_) => get_positive_dim(params, "bore_diameter", 10.0, unit)?,
    };
    if bore_d * 0.5 >= min_root_radius * 0.85 {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'bore_diameter' ({bore_d:.4}) is too large for smallest gear root radius ({min_root_radius:.4}); bore radius must be < {:.4}",
            min_root_radius * 0.85
        )));
    }

    let motor_vel_opt = match params.get("motor_velocity_rad_s") {
        None | Some(Value::Null) => None,
        Some(_) => Some(get_finite_f64(params, "motor_velocity_rad_s", 0.0)?),
    };
    let motor_max_torque = get_finite_f64(params, "motor_max_torque_nm", 100.0)?;
    if motor_max_torque < 0.0 {
        return Err(ServerError::InvalidArgument(format!(
            "Parameter 'motor_max_torque_nm' must be non-negative (>= 0), got {motor_max_torque}"
        )));
    }

    let u_str = unit.as_str();
    let r_pitch_pinion = 0.5 * module * (teeth_pinion as f64);
    let r_pitch_gear = 0.5 * module * (teeth_gear as f64);
    let center_dist = r_pitch_pinion + r_pitch_gear;
    let r_outer_pinion = r_pitch_pinion + module;
    let r_outer_gear = r_pitch_gear + module;

    // 1. Fixed shared baseplate frame beneath both gears
    let margin = mm_to_unit(20.0, unit);
    let frame_width = r_outer_pinion + center_dist + r_outer_gear + margin;
    let frame_depth = 2.0 * r_outer_pinion.max(r_outer_gear) + margin;
    let frame_center_x = 0.5 * (center_dist + r_outer_gear - r_outer_pinion);
    let frame_center_z = -(0.5 * face_width + frame_clearance + 0.5 * frame_thickness);

    cad::create_part(
        &json!({
            "part_id": frame_id,
            "name": "Transmission Frame",
            "material": frame_material,
            "is_fixed": true
        }),
        state,
    )?;
    let frame_resp = cad::add_feature_primitive(
        &json!({
            "part_id": frame_id,
            "primitive_type": "box",
            "length_unit": u_str,
            "params": {
                "width": frame_width,
                "depth": frame_depth,
                "height": frame_thickness,
                "position": [frame_center_x, 0.0, frame_center_z]
            }
        }),
        state,
    )?;

    // 2. Pinion centered at [0, 0, 0]
    cad::create_part(
        &json!({
            "part_id": pinion_id,
            "name": "Pinion Gear",
            "material": material,
            "is_fixed": false
        }),
        state,
    )?;
    cad::add_feature_primitive(
        &json!({
            "part_id": pinion_id,
            "primitive_type": "spur_gear",
            "length_unit": u_str,
            "params": {
                "module": module,
                "teeth": teeth_pinion,
                "pressure_angle_deg": pressure_angle_deg,
                "face_width": face_width,
                "bore_diameter": bore_d,
                "position": [0.0, 0.0, 0.0]
            }
        }),
        state,
    )?;

    // 3. Driven gear centered at [center_dist, 0, 0], rotated by half-tooth pitch when even teeth
    // so pinion tooth 0 (+X) aligns with a trough on the driven gear (-X).
    let gear_phase_deg = if teeth_gear % 2 == 0 {
        180.0 / (teeth_gear as f64)
    } else {
        0.0
    };
    cad::create_part(
        &json!({
            "part_id": gear_id,
            "name": "Driven Gear",
            "material": material,
            "is_fixed": false
        }),
        state,
    )?;
    cad::add_feature_primitive(
        &json!({
            "part_id": gear_id,
            "primitive_type": "spur_gear",
            "length_unit": u_str,
            "params": {
                "module": module,
                "teeth": teeth_gear,
                "pressure_angle_deg": pressure_angle_deg,
                "face_width": face_width,
                "bore_diameter": bore_d,
                "position": [center_dist, 0.0, 0.0],
                "rotation_deg": [0.0, 0.0, gear_phase_deg]
            }
        }),
        state,
    )?;

    // Validate all three meshes before wiring physics joints
    let frame_mesh = validate_created_part(&frame_id, state)?;
    let pinion_mesh = validate_created_part(&pinion_id, state)?;
    let gear_mesh = validate_created_part(&gear_id, state)?;

    // 4. Kinematic joints: two revolute bearings on the shared frame + gear coupling
    let j_pinion = format!("{prefix}_pinion_revolute");
    let j_gear = format!("{prefix}_gear_revolute");
    let j_mesh = format!("{prefix}_gear_mesh");

    physics::create_joint(
        &json!({
            "joint_id": j_pinion,
            "joint_type": "revolute",
            "part_a": frame_id,
            "part_b": pinion_id,
            "length_unit": u_str,
            "anchor_a": [0.0, 0.0, 0.0],
            "anchor_b": [0.0, 0.0, 0.0],
            "axis": [0.0, 0.0, 1.0]
        }),
        state,
    )?;
    physics::create_joint(
        &json!({
            "joint_id": j_gear,
            "joint_type": "revolute",
            "part_a": frame_id,
            "part_b": gear_id,
            "length_unit": u_str,
            "anchor_a": [center_dist, 0.0, 0.0],
            "anchor_b": [center_dist, 0.0, 0.0],
            "axis": [0.0, 0.0, 1.0]
        }),
        state,
    )?;
    physics::create_joint(
        &json!({
            "joint_id": j_mesh,
            "joint_type": "gear",
            "part_a": pinion_id,
            "part_b": gear_id,
            "teeth_a": teeth_pinion,
            "teeth_b": teeth_gear,
            "axis": [0.0, 0.0, 1.0]
        }),
        state,
    )?;

    let mut motor_active = false;
    if let Some(vel) = motor_vel_opt {
        physics::set_joint_motor(
            &json!({
                "joint_id": j_pinion,
                "target_velocity": vel,
                "max_force": motor_max_torque
            }),
            state,
        )?;
        motor_active = true;
    }

    let warnings = extract_warnings(&frame_resp);

    let gear_ratio = (teeth_pinion as f64) / (teeth_gear as f64);
    let summary = json!({
        "frame_id": frame_id,
        "pinion_id": pinion_id,
        "gear_id": gear_id,
        "module": module,
        "teeth_pinion": teeth_pinion,
        "teeth_gear": teeth_gear,
        "gear_ratio": gear_ratio,
        "center_distance": center_dist,
        "face_width": face_width,
        "bore_diameter": bore_d,
        "gear_phase_deg": gear_phase_deg,
        "motor_active": motor_active,
        "joints": {
            "pinion_revolute": j_pinion,
            "gear_revolute": j_gear,
            "gear_mesh": j_mesh
        },
        "mesh_validation": {
            "frame_watertight": frame_mesh["is_watertight"],
            "pinion_watertight": pinion_mesh["is_watertight"],
            "gear_watertight": gear_mesh["is_watertight"]
        }
    });

    Ok(RecipeOutcome {
        created_parts: vec![frame_id, pinion_id, gear_id],
        created_joints: vec![j_pinion, j_gear, j_mesh],
        details: summary,
        warnings,
    })
}

/// Recipe 4: Helical compression/extension coil spring (`coil_spring`) built natively via `AddHelix`.
fn build_coil_spring(
    params: &Value,
    unit: LengthUnit,
    state: &mut EngineState,
) -> Result<RecipeOutcome, ServerError> {
    let part_id = get_nonempty_str(params, "part_id", "coil_spring")?;
    let name = get_nonempty_str(params, "name", "Helical Coil Spring")?;
    let material = get_nonempty_str(params, "material", "steel_high_strength_4140")?;
    let is_fixed = params
        .get("is_fixed")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let is_right_hand = params
        .get("is_right_hand")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);

    let helix_radius = if params.get("helix_radius").is_some() {
        get_positive_dim(params, "helix_radius", 17.5, unit)?
    } else if params.get("coil_radius").is_some() {
        get_positive_dim(params, "coil_radius", 17.5, unit)?
    } else if params.get("mean_diameter").is_some() {
        get_positive_dim(params, "mean_diameter", 35.0, unit)? * 0.5
    } else if params.get("coil_diameter").is_some() {
        get_positive_dim(params, "coil_diameter", 35.0, unit)? * 0.5
    } else {
        mm_to_unit(17.5, unit)
    };

    let wire_radius = if params.get("wire_radius").is_some() {
        get_positive_dim(params, "wire_radius", 2.5, unit)?
    } else if params.get("wire_diameter").is_some() {
        get_positive_dim(params, "wire_diameter", 5.0, unit)? * 0.5
    } else {
        mm_to_unit(2.5, unit)
    };

    let active_coils_opt = if params.get("active_coils").is_some() {
        let v = get_finite_f64(params, "active_coils", 5.0)?;
        if v <= 0.0 {
            return Err(ServerError::InvalidArgument(format!(
                "Parameter 'active_coils' must be > 0, got {v}"
            )));
        }
        Some(v)
    } else if params.get("turns").is_some() {
        let v = get_finite_f64(params, "turns", 5.0)?;
        if v <= 0.0 {
            return Err(ServerError::InvalidArgument(format!(
                "Parameter 'turns' must be > 0, got {v}"
            )));
        }
        Some(v)
    } else {
        None
    };

    let pitch_opt = if params.get("pitch").is_some() {
        Some(get_positive_dim(params, "pitch", 12.0, unit)?)
    } else {
        None
    };
    let free_len_opt = if params.get("free_length").is_some() {
        Some(get_positive_dim(params, "free_length", 60.0, unit)?)
    } else if params.get("height").is_some() {
        Some(get_positive_dim(params, "height", 60.0, unit)?)
    } else {
        None
    };

    let (pitch, free_length, active_coils) = match (pitch_opt, free_len_opt, active_coils_opt) {
        (Some(p), Some(l0), _) => (p, l0, l0 / p),
        (Some(p), None, Some(n)) => (p, p * n, n),
        (None, Some(l0), Some(n)) => (l0 / n, l0, n),
        (Some(p), None, None) => (p, p * 5.0, 5.0),
        (None, Some(l0), None) => (l0 / 5.0, l0, 5.0),
        (None, None, Some(n)) => {
            let p = mm_to_unit(12.0, unit);
            (p, p * n, n)
        }
        (None, None, None) => {
            let p = mm_to_unit(12.0, unit);
            (p, p * 5.0, 5.0)
        }
    };

    let mean_diameter = 2.0 * helix_radius;
    let wire_diameter = 2.0 * wire_radius;
    if wire_radius >= helix_radius {
        return Err(ServerError::InvalidArgument(format!(
            "coil_spring requires wire_radius ({wire_radius}) < helix_radius ({helix_radius})"
        )));
    }
    if pitch <= wire_diameter {
        return Err(ServerError::InvalidArgument(format!(
            "coil_spring requires pitch ({pitch}) > wire_diameter ({wire_diameter}) so adjacent coils do not collide"
        )));
    }

    let u_str = unit.as_str();
    let mut warnings = Vec::new();

    cad::create_part(
        &json!({
            "part_id": part_id,
            "name": name,
            "material": material,
            "is_fixed": is_fixed
        }),
        state,
    )?;

    let add_resp = cad::add_feature_primitive(
        &json!({
            "part_id": part_id,
            "primitive_type": "helix",
            "length_unit": u_str,
            "params": {
                "helix_radius": helix_radius,
                "wire_radius": wire_radius,
                "pitch": pitch,
                "height": free_length,
                "starts": 1,
                "is_right_hand": is_right_hand
            }
        }),
        state,
    )?;
    warnings.extend(extract_warnings(&add_resp));

    let mesh_report = validate_created_part(&part_id, state)?;

    // Analytical helical compression spring engineering metrics (SI)
    let k_to_m = unit.to_meters_factor();
    let d_mean_m = mean_diameter * k_to_m;
    let d_wire_m = wire_diameter * k_to_m;
    let pitch_m = pitch * k_to_m;
    let spring_index = d_mean_m / d_wire_m;
    let wahl_factor = if spring_index > 1.05 {
        (4.0 * spring_index - 1.0) / (4.0 * spring_index - 4.0) + 0.615 / spring_index
    } else {
        1.0
    };
    let shear_modulus_gpa = state
        .parts
        .get(&part_id)
        .map(|p| {
            let mat = &p.material;
            mat.youngs_modulus_gpa / (2.0 * (1.0 + mat.poissons_ratio))
        })
        .unwrap_or(79.3);
    let g_pa = shear_modulus_gpa * 1.0e9;
    let spring_rate_n_per_m =
        (g_pa * d_wire_m.powi(4)) / (8.0 * d_mean_m.powi(3) * active_coils);
    let spring_rate_n_per_mm = spring_rate_n_per_m * 1.0e-3;
    let solid_height = active_coils * wire_diameter;
    let max_travel = (free_length - solid_height).max(0.0);
    let wire_length_m =
        active_coils * ((PI * d_mean_m).powi(2) + pitch_m * pitch_m).sqrt();
    let wire_length = wire_length_m * unit.from_meters_factor();

    let summary = json!({
        "part_id": part_id,
        "material": material,
        "mean_diameter": mean_diameter,
        "outer_diameter": mean_diameter + wire_diameter,
        "inner_diameter": mean_diameter - wire_diameter,
        "helix_radius": helix_radius,
        "wire_diameter": wire_diameter,
        "wire_radius": wire_radius,
        "pitch": pitch,
        "free_length": free_length,
        "active_coils": active_coils,
        "is_right_hand": is_right_hand,
        "solid_height": solid_height,
        "max_travel": max_travel,
        "wire_length": wire_length,
        "spring_index_c": spring_index,
        "wahl_stress_factor_kw": wahl_factor,
        "shear_modulus_gpa": shear_modulus_gpa,
        "spring_rate_n_per_m": spring_rate_n_per_m,
        "spring_rate_n_per_mm": spring_rate_n_per_mm,
        "mesh_validation": mesh_report
    });

    Ok(RecipeOutcome {
        created_parts: vec![part_id],
        created_joints: Vec::new(),
        details: summary,
        warnings,
    })
}

/// Applies a named parametric engineering recipe (`flanged_bushing`, `stepped_shaft`, `gear_transmission`, `coil_spring`).
pub fn apply_recipe(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let recipe_name = args
        .get("recipe")
        .or_else(|| args.get("recipe_name"))
        .and_then(|v| v.as_str())
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .ok_or_else(|| {
            ServerError::InvalidArgument("Missing required argument 'recipe'".to_string())
        })?;

    let params = match args.get("params") {
        None | Some(Value::Null) => json!({}),
        Some(v) if v.is_object() => v.clone(),
        Some(_) => {
            return Err(ServerError::InvalidArgument(
                "Argument 'params' must be a JSON object".to_string(),
            ));
        }
    };

    let unit = if params.get("length_unit").is_some() {
        state.resolve_length_unit(&params)?
    } else {
        state.resolve_length_unit(args)?
    };

    let snapshot = StateSnapshot::capture(state);

    let outcome = match recipe_name {
        "flanged_bushing" => build_flanged_bushing(&params, unit, state),
        "stepped_shaft" => build_stepped_shaft(&params, unit, state),
        "gear_transmission" => build_gear_transmission(&params, unit, state),
        "coil_spring" => build_coil_spring(&params, unit, state),
        other => Err(ServerError::InvalidArgument(format!(
            "Unknown 'recipe' '{other}'. Supported recipes: flanged_bushing, stepped_shaft, gear_transmission, coil_spring"
        ))),
    };

    let RecipeOutcome {
        created_parts,
        created_joints,
        details,
        mut warnings,
    } = match outcome {
        Ok(res) => res,
        Err(err) => {
            snapshot.restore(state);
            return Err(err);
        }
    };

    // Merge scale-sanity warnings across all created parts
    for pid in &created_parts {
        if let Some(part) = state.parts.get(pid) {
            let part_warnings = state.check_scale_sanity(part, &[], None, unit);
            for w in part_warnings {
                if !warnings.contains(&w) {
                    warnings.push(w);
                }
            }
        }
    }

    let mut resp = json!({
        "status": "recipe_applied",
        "recipe": recipe_name,
        "created_parts": created_parts,
        "created_joints": created_joints,
        "details": details,
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
        "warnings": warnings,
    });

    let primary_part_id = created_parts.first().map(|s| s.as_str());
    requirements::attach_auto_check_if_enabled(state, primary_part_id, &mut resp);

    Ok(resp)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recipe_flanged_bushing_default_is_watertight_and_respects_units() {
        let mut state = EngineState::new().unwrap();
        let res = apply_recipe(
            &json!({
                "recipe": "flanged_bushing"
            }),
            &mut state,
        )
        .unwrap();

        assert_eq!(res["status"], "recipe_applied");
        assert_eq!(res["recipe"], "flanged_bushing");
        assert_eq!(res["length_unit"], "mm");
        assert_eq!(res["created_parts"], json!(["flanged_bushing"]));
        assert_eq!(res["created_joints"], json!([]));
        assert_eq!(res["details"]["mesh_validation"]["is_watertight"], true);
        assert_eq!(res["details"]["mesh_validation"]["boundary_edges_count"], 0);

        // Also test top and center flange positions in meters
        let res_m = apply_recipe(
            &json!({
                "recipe": "flanged_bushing",
                "length_unit": "m",
                "params": {
                    "part_id": "bushing_top_m",
                    "flange_position": "top",
                    "sleeve_outer_diameter": 0.030,
                    "sleeve_length": 0.050,
                    "bore_diameter": 0.016,
                    "flange_diameter": 0.064,
                    "flange_thickness": 0.010,
                    "pcd_diameter": 0.048,
                    "bolt_holes_count": 6,
                    "bolt_hole_diameter": 0.0055
                }
            }),
            &mut state,
        )
        .unwrap();

        assert_eq!(res_m["length_unit"], "m");
        assert_eq!(res_m["details"]["mesh_validation"]["is_watertight"], true);
        assert_eq!(res_m["details"]["bolt_holes_count"], 6);
        let part_m = &state.parts["bushing_top_m"];
        let dims = part_m.bounding_box().dimensions();
        assert!((dims[2] - 0.050).abs() < 1e-4, "Height in SI meters should be 0.050, got {}", dims[2]);
    }

    #[test]
    fn recipe_stepped_shaft_default_is_watertight_with_keyway() {
        let mut state = EngineState::new().unwrap();
        let res = apply_recipe(
            &json!({
                "recipe": "stepped_shaft",
                "params": {
                    "keyway_step_index": 1
                }
            }),
            &mut state,
        )
        .unwrap();

        assert_eq!(res["status"], "recipe_applied");
        assert_eq!(res["recipe"], "stepped_shaft");
        assert_eq!(res["created_parts"], json!(["stepped_shaft"]));
        assert_eq!(res["details"]["mesh_validation"]["is_watertight"], true);
        assert_eq!(res["details"]["mesh_validation"]["boundary_edges_count"], 0);

        let shaft = &state.parts["stepped_shaft"];
        let dims = shaft.bounding_box().dimensions();
        assert!((dims[2] - 0.100).abs() < 1e-4, "Total length 30+40+30=100mm -> 0.100m, got {}", dims[2]);
        assert!((dims[1] - 0.032).abs() < 5e-4, "Max diameter 32mm -> 0.032m, got {}", dims[1]);
    }

    #[test]
    fn recipe_gear_transmission_builds_kinematic_assembly_and_clean_meshes() {
        let mut state = EngineState::new().unwrap();
        let res = apply_recipe(
            &json!({
                "recipe": "gear_transmission",
                "params": {
                    "motor_velocity_rad_s": 6.0
                }
            }),
            &mut state,
        )
        .unwrap();

        assert_eq!(res["status"], "recipe_applied");
        assert_eq!(res["recipe"], "gear_transmission");
        assert_eq!(
            res["created_parts"],
            json!(["gear_tx_frame", "gear_tx_pinion", "gear_tx_gear"])
        );
        assert_eq!(
            res["created_joints"],
            json!([
                "gear_tx_pinion_revolute",
                "gear_tx_gear_revolute",
                "gear_tx_gear_mesh"
            ])
        );
        assert_eq!(res["details"]["motor_active"], true);
        assert_eq!(res["details"]["mesh_validation"]["frame_watertight"], true);
        assert_eq!(res["details"]["mesh_validation"]["pinion_watertight"], true);
        assert_eq!(res["details"]["mesh_validation"]["gear_watertight"], true);

        // Verify mechanism analysis reports clean 1-DOF coupled transmission
        let mech = physics::analyze_mechanism(&json!({}), &mut state).unwrap();
        assert_eq!(mech["dof_numeric"], 1);
        assert_eq!(mech["redundant_constraints"], 0);

        // Verify simulation advances smoothly without exploding and obeys gear ratio (-0.5)
        let sim = physics::run_physics_simulation(
            &json!({ "duration_seconds": 0.25, "step_size_ms": 10.0 }),
            &mut state,
        )
        .unwrap();
        assert!(sim["steps_simulated"].as_u64().unwrap() >= 25);
        let trajectories = &sim["trajectory_summary"];
        let pinion_samples = trajectories["gear_tx_pinion"]["samples"].as_array().unwrap();
        let gear_samples = trajectories["gear_tx_gear"]["samples"].as_array().unwrap();
        let last_wp = pinion_samples.last().unwrap()["angular_velocity"][2]
            .as_f64()
            .unwrap();
        let last_wg = gear_samples.last().unwrap()["angular_velocity"][2]
            .as_f64()
            .unwrap();
        assert!(last_wp.abs() > 1.0, "Pinion should spin, got {last_wp}");
        assert!(
            (last_wg + 0.5 * last_wp).abs() < 0.2,
            "Gear angular velocity ({last_wg}) should satisfy -0.5 * pinion ({last_wp})"
        );
    }

    #[test]
    fn recipe_validation_error_triggers_transactional_rollback() {
        let mut state = EngineState::new().unwrap();
        cad::create_part(
            &json!({
                "part_id": "gear_tx_gear",
                "name": "Existing Gear",
                "material": "aluminum_6061_t6"
            }),
            &mut state,
        )
        .unwrap();
        let initial_part_count = state.parts.len();

        // gear_tx_frame and gear_tx_pinion don't exist, gear_tx_gear DOES -> fails mid-recipe, must rollback frame & pinion!
        let err = apply_recipe(
            &json!({
                "recipe": "gear_transmission"
            }),
            &mut state,
        )
        .unwrap_err();
        let err_msg = err.to_string();
        assert!(
            err_msg.contains("gear_tx_gear"),
            "Expected duplicate part error for gear_tx_gear, got: {err_msg}"
        );
        assert_eq!(state.parts.len(), initial_part_count);
        assert!(!state.parts.contains_key("gear_tx_frame"));
        assert!(!state.parts.contains_key("gear_tx_pinion"));
    }

    #[test]
    fn recipe_coil_spring_builds_watertight_spring_with_metrics() {
        let mut state = EngineState::new().unwrap();
        let res = apply_recipe(
            &json!({
                "recipe": "coil_spring",
                "params": {
                    "part_id": "main_spring",
                    "helix_radius": 17.5,
                    "wire_radius": 2.5,
                    "pitch": 12.0,
                    "active_coils": 5.0
                }
            }),
            &mut state,
        )
        .unwrap();

        assert_eq!(res["status"], "recipe_applied");
        assert_eq!(res["recipe"], "coil_spring");
        assert_eq!(res["created_parts"], json!(["main_spring"]));
        assert_eq!(res["details"]["mesh_validation"]["is_watertight"], true);
        assert!((res["details"]["spring_index_c"].as_f64().unwrap() - 7.0).abs() < 1e-6);
        assert!(res["details"]["spring_rate_n_per_mm"].as_f64().unwrap() > 10.0);
        assert!((res["details"]["free_length"].as_f64().unwrap() - 60.0).abs() < 1e-6);
    }
}
