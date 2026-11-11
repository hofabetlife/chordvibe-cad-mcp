//! Engineering requirements declaration and automated verification (`set_requirements` / `check_requirements`).
//!
//! Enables AI agents to declare quantitative engineering constraints once (`bbox_max`, `bbox_min`,
//! `mass_max`, `mesh_valid`, `min_safety_factor`, `no_interference`, `mechanism_dof`) and have
//! the CAD/CAE engine automatically verify them after every geometry mutation, returning
//! measured values, margins, staleness tracking via `Part.revision`, and actionable hints.

use std::collections::HashMap;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use mcp_cad_core::part::Part;

use crate::error::ServerError;
use crate::state::{EngineState, LengthUnit};

/// Verification status of an individual engineering requirement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RequirementStatus {
    #[serde(rename = "PASS")]
    Pass,
    #[serde(rename = "FAIL")]
    Fail,
    #[serde(rename = "STALE")]
    Stale,
    #[serde(rename = "NOT_AVAILABLE")]
    NotAvailable,
}

impl RequirementStatus {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Fail => "FAIL",
            Self::Stale => "STALE",
            Self::NotAvailable => "NOT_AVAILABLE",
        }
    }
}

/// Canonical requirement specification kind stored in internal SI units (`m`, `kg`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum RequirementKind {
    /// Maximum allowable bounding box dimensions `[dx, dy, dz]` (stored in SI meters).
    #[serde(rename = "bbox_max")]
    BboxMax { dims_m: [f64; 3] },

    /// Minimum allowable bounding box dimensions `[dx, dy, dz]` (stored in SI meters).
    #[serde(rename = "bbox_min")]
    BboxMin { dims_m: [f64; 3] },

    /// Maximum allowable mass in kilograms (`kg`).
    #[serde(rename = "mass_max")]
    MassMax { kg: f64 },

    /// Mesh topological and geometric validity gate (`validate_mesh` == `"PASS"`).
    #[serde(rename = "mesh_valid")]
    MeshValid {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        expected_genus: Option<i32>,
    },

    /// Minimum required structural Factor of Safety (`yield_strength / max_von_mises_stress`) from FEA.
    #[serde(rename = "min_safety_factor")]
    MinSafetyFactor {
        value: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        fea_case: Option<String>,
    },

    /// Static multi-body interference check (returns `NOT_AVAILABLE` until P3-T10).
    #[serde(rename = "no_interference")]
    NoInterference {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        pairs: Option<Vec<[String; 2]>>,
    },

    /// Mechanism mobility / degrees of freedom check (returns `NOT_AVAILABLE` until P3-T9).
    #[serde(rename = "mechanism_dof")]
    MechanismDof { expected: i32 },
}

impl RequirementKind {
    pub fn type_name(&self) -> &'static str {
        match self {
            Self::BboxMax { .. } => "bbox_max",
            Self::BboxMin { .. } => "bbox_min",
            Self::MassMax { .. } => "mass_max",
            Self::MeshValid { .. } => "mesh_valid",
            Self::MinSafetyFactor { .. } => "min_safety_factor",
            Self::NoInterference { .. } => "no_interference",
            Self::MechanismDof { .. } => "mechanism_dof",
        }
    }
}

/// Stored engineering requirement definition.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RequirementSpec {
    /// Requirement identifier (user-supplied or auto-generated e.g. `"req_1"`).
    pub id: String,
    /// Optional target part ID when declared inside a scene-scoped requirement list.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub part_id: Option<String>,
    /// Typed requirement parameters in SI units.
    #[serde(flatten)]
    pub kind: RequirementKind,
}

/// Session-wide storage for declared engineering requirements and `auto_check` preferences.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
pub struct RequirementsState {
    /// Global `auto_check` flag: if true, mutating CAD tools append `requirements_status`.
    #[serde(default)]
    pub auto_check: bool,
    /// Per-part requirements keyed by `part_id`.
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub part_requirements: HashMap<String, Vec<RequirementSpec>>,
    /// Per-part `auto_check` overrides keyed by `part_id`.
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub part_auto_check: HashMap<String, bool>,
    /// Scene-level requirements (`scope: "scene"`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub scene_requirements: Vec<RequirementSpec>,
}

impl RequirementsState {
    /// Returns true if no requirements or auto_check flags are active.
    pub fn is_empty(&self) -> bool {
        !self.auto_check
            && self.part_requirements.values().all(|v| v.is_empty())
            && self.scene_requirements.is_empty()
    }

    /// Returns true if `auto_check` should run when `part_id_opt` (or the whole scene if `None`) is mutated.
    pub fn should_auto_check(&self, part_id_opt: Option<&str>) -> bool {
        if let Some(pid) = part_id_opt {
            if let Some(&flag) = self.part_auto_check.get(pid) {
                if flag {
                    return true;
                }
            }
        }
        self.auto_check || self.part_auto_check.values().any(|&f| f)
    }
}

/// Helper to parse a 3-element dimension vector `[dx, dy, dz]` from a requirement JSON object.
fn parse_dims_vec3(obj: &Value, k_to_m: f64, allow_zero: bool) -> Result<[f64; 3], ServerError> {
    let arr_opt = obj
        .get("dims")
        .or_else(|| obj.get("dimensions"))
        .or_else(|| obj.get("max_dims"))
        .or_else(|| obj.get("min_dims"))
        .or_else(|| obj.get("value"))
        .or_else(|| obj.get("target"))
        .and_then(|v| v.as_array());

    let raw = if let Some(arr) = arr_opt {
        if arr.len() != 3 {
            return Err(ServerError::InvalidArgument(
                "Requirement 'dims' must be a 3-element array [dx, dy, dz]".to_string(),
            ));
        }
        [
            arr[0].as_f64().ok_or_else(|| {
                ServerError::InvalidArgument("Requirement 'dims[0]' must be a number".to_string())
            })?,
            arr[1].as_f64().ok_or_else(|| {
                ServerError::InvalidArgument("Requirement 'dims[1]' must be a number".to_string())
            })?,
            arr[2].as_f64().ok_or_else(|| {
                ServerError::InvalidArgument("Requirement 'dims[2]' must be a number".to_string())
            })?,
        ]
    } else if let (Some(w), Some(h), Some(d)) = (
        obj.get("width").or_else(|| obj.get("x")).and_then(|v| v.as_f64()),
        obj.get("height").or_else(|| obj.get("y")).and_then(|v| v.as_f64()),
        obj.get("depth").or_else(|| obj.get("z")).and_then(|v| v.as_f64()),
    ) {
        [w, h, d]
    } else {
        return Err(ServerError::InvalidArgument(
            "Bounding box requirement requires 'dims': [dx, dy, dz] in active length_unit".to_string(),
        ));
    };

    for (i, &val) in raw.iter().enumerate() {
        if !val.is_finite() || (!allow_zero && val <= 0.0) || (allow_zero && val < 0.0) {
            return Err(ServerError::InvalidArgument(format!(
                "Invalid dimension dims[{i}] = {val} in bounding box requirement: must be finite and {}",
                if allow_zero { ">= 0" } else { "> 0" }
            )));
        }
    }

    Ok([raw[0] * k_to_m, raw[1] * k_to_m, raw[2] * k_to_m])
}

/// Parses a single JSON requirement item into a validated `RequirementSpec`.
fn parse_requirement_spec(
    item: &Value,
    idx: usize,
    default_part_id: Option<&str>,
    unit: LengthUnit,
) -> Result<RequirementSpec, ServerError> {
    let obj = item.as_object().ok_or_else(|| {
        ServerError::InvalidArgument(format!(
            "Requirement at index {idx} must be a JSON object"
        ))
    })?;

    let req_type = obj
        .get("type")
        .or_else(|| obj.get("kind"))
        .or_else(|| obj.get("requirement_type"))
        .and_then(|v| v.as_str())
        .ok_or_else(|| {
            ServerError::InvalidArgument(format!(
                "Requirement at index {idx} is missing required string field 'type'"
            ))
        })?;

    let id = obj
        .get("id")
        .and_then(|v| v.as_str())
        .filter(|s| !s.trim().is_empty())
        .map(|s| s.to_string())
        .unwrap_or_else(|| format!("req_{}", idx + 1));

    let item_part_id = obj
        .get("part_id")
        .and_then(|v| v.as_str())
        .map(|s| s.to_string())
        .or_else(|| default_part_id.map(|s| s.to_string()));

    let k_to_m = unit.to_meters_factor();

    let kind = match req_type.trim().to_ascii_lowercase().as_str() {
        "bbox_max" | "max_bbox" | "bounding_box_max" => {
            let dims_m = parse_dims_vec3(item, k_to_m, false)?;
            RequirementKind::BboxMax { dims_m }
        }
        "bbox_min" | "min_bbox" | "bounding_box_min" => {
            let dims_m = parse_dims_vec3(item, k_to_m, true)?;
            RequirementKind::BboxMin { dims_m }
        }
        "mass_max" | "max_mass" => {
            let kg = obj
                .get("kg")
                .or_else(|| obj.get("mass_kg"))
                .or_else(|| obj.get("max_kg"))
                .or_else(|| obj.get("value"))
                .or_else(|| obj.get("target"))
                .and_then(|v| v.as_f64())
                .ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Requirement 'mass_max' requires numeric field 'kg' (maximum mass in kilograms)"
                            .to_string(),
                    )
                })?;
            if !kg.is_finite() || kg <= 0.0 {
                return Err(ServerError::InvalidArgument(format!(
                    "Requirement 'mass_max' field 'kg' must be a positive finite number, got {kg}"
                )));
            }
            RequirementKind::MassMax { kg }
        }
        "mesh_valid" | "valid_mesh" | "watertight" => {
            let expected_genus = obj
                .get("expected_genus")
                .or_else(|| obj.get("genus"))
                .and_then(|v| v.as_i64())
                .map(|g| g as i32);
            RequirementKind::MeshValid { expected_genus }
        }
        "min_safety_factor" | "safety_factor" | "fos_min" | "min_fos" => {
            let value = obj
                .get("value")
                .or_else(|| obj.get("min_safety_factor"))
                .or_else(|| obj.get("fos"))
                .or_else(|| obj.get("target"))
                .and_then(|v| v.as_f64())
                .ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Requirement 'min_safety_factor' requires numeric field 'value' (minimum Factor of Safety)"
                            .to_string(),
                    )
                })?;
            if !value.is_finite() || value <= 0.0 {
                return Err(ServerError::InvalidArgument(format!(
                    "Requirement 'min_safety_factor' field 'value' must be > 0, got {value}"
                )));
            }
            let fea_case = obj
                .get("fea_case")
                .or_else(|| obj.get("case"))
                .and_then(|v| v.as_str())
                .map(|s| s.to_string());
            RequirementKind::MinSafetyFactor { value, fea_case }
        }
        "no_interference" | "interference" => {
            let pairs = if let Some(arr) = obj.get("pairs").and_then(|v| v.as_array()) {
                let mut parsed_pairs = Vec::with_capacity(arr.len());
                for p in arr {
                    let pair_arr = p.as_array().ok_or_else(|| {
                        ServerError::InvalidArgument(
                            "Each element in 'pairs' for 'no_interference' must be a 2-element array [part_a, part_b]"
                                .to_string(),
                        )
                    })?;
                    if pair_arr.len() != 2 {
                        return Err(ServerError::InvalidArgument(
                            "Each element in 'pairs' for 'no_interference' must have exactly 2 part IDs"
                                .to_string(),
                        ));
                    }
                    let a = pair_arr[0].as_str().ok_or_else(|| {
                        ServerError::InvalidArgument("Part ID in 'pairs' must be a string".to_string())
                    })?;
                    let b = pair_arr[1].as_str().ok_or_else(|| {
                        ServerError::InvalidArgument("Part ID in 'pairs' must be a string".to_string())
                    })?;
                    parsed_pairs.push([a.to_string(), b.to_string()]);
                }
                Some(parsed_pairs)
            } else {
                None
            };
            RequirementKind::NoInterference { pairs }
        }
        "mechanism_dof" | "dof" => {
            let expected = obj
                .get("expected")
                .or_else(|| obj.get("dof"))
                .or_else(|| obj.get("value"))
                .or_else(|| obj.get("target"))
                .and_then(|v| v.as_i64())
                .ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Requirement 'mechanism_dof' requires integer field 'expected'".to_string(),
                    )
                })? as i32;
            RequirementKind::MechanismDof { expected }
        }
        other => {
            return Err(ServerError::InvalidArgument(format!(
                "Unknown requirement type '{other}'. Supported types: bbox_max, bbox_min, mass_max, mesh_valid, min_safety_factor, no_interference, mechanism_dof"
            )));
        }
    };

    Ok(RequirementSpec {
        id,
        part_id: item_part_id,
        kind,
    })
}

/// Evaluates a single `RequirementSpec` against the current `EngineState`.
fn evaluate_single_requirement(
    spec: &RequirementSpec,
    scope_part_id: Option<&str>,
    state: &EngineState,
    unit: LengthUnit,
) -> Value {
    let effective_part_id = spec.part_id.as_deref().or(scope_part_id);
    let k_from_m = unit.from_meters_factor();
    let u_str = unit.as_str();

    let scope_label = effective_part_id.unwrap_or("scene");

    match &spec.kind {
        RequirementKind::NoInterference { pairs } => json!({
            "id": spec.id,
            "part_id": effective_part_id,
            "scope": scope_label,
            "type": "no_interference",
            "status": RequirementStatus::NotAvailable.as_str(),
            "measured": Value::Null,
            "target": {
                "interfering_pairs": 0,
                "pairs": pairs,
            },
            "margin": Value::Null,
            "hint": "Static multi-body interference analysis ('check_interference') is not yet available (scheduled for P3-T10).",
        }),

        RequirementKind::MechanismDof { expected } => json!({
            "id": spec.id,
            "part_id": effective_part_id,
            "scope": scope_label,
            "type": "mechanism_dof",
            "status": RequirementStatus::NotAvailable.as_str(),
            "measured": Value::Null,
            "target": *expected,
            "margin": Value::Null,
            "hint": "Mechanism mobility / degrees-of-freedom analysis ('analyze_mechanism') is not yet available (scheduled for P3-T9).",
        }),

        RequirementKind::BboxMax { dims_m } => {
            let target_u = [
                dims_m[0] * k_from_m,
                dims_m[1] * k_from_m,
                dims_m[2] * k_from_m,
            ];

            // Scene-level bbox_max if no single part is specified
            if effective_part_id.is_none() && !state.parts.is_empty() {
                return evaluate_scene_bbox_requirement(spec, *dims_m, true, state, unit);
            }

            let Some(part) = resolve_target_part(effective_part_id, state) else {
                let missing = effective_part_id.unwrap_or("scene");
                return json!({
                    "id": spec.id,
                    "part_id": effective_part_id,
                    "scope": scope_label,
                    "type": "bbox_max",
                    "status": RequirementStatus::Fail.as_str(),
                    "measured": Value::Null,
                    "target": target_u,
                    "margin": Value::Null,
                    "hint": format!("Target part '{missing}' was not found or scene has no parts. Create part and add features first."),
                });
            };

            if part.mesh().vertices.is_empty() {
                return json!({
                    "id": spec.id,
                    "part_id": part.id,
                    "scope": part.id,
                    "type": "bbox_max",
                    "status": RequirementStatus::Fail.as_str(),
                    "measured": [0.0, 0.0, 0.0],
                    "target": target_u,
                    "margin": Value::Null,
                    "hint": format!("Part '{}' has an empty mesh. Add 3D primitive features via 'add_feature_primitive'.", part.id),
                });
            }

            let aabb = part.bounding_box();
            let meas_m = aabb.dimensions();
            let meas_u = [
                meas_m[0] * k_from_m,
                meas_m[1] * k_from_m,
                meas_m[2] * k_from_m,
            ];
            let margins_u = [
                target_u[0] - meas_u[0],
                target_u[1] - meas_u[1],
                target_u[2] - meas_u[2],
            ];
            let min_margin_u = margins_u[0].min(margins_u[1]).min(margins_u[2]);
            let tol_m = 1e-7;
            let passed = meas_m[0] <= dims_m[0] + tol_m
                && meas_m[1] <= dims_m[1] + tol_m
                && meas_m[2] <= dims_m[2] + tol_m;

            let hint = if passed {
                None
            } else {
                let mut exceeded = Vec::new();
                for (axis, i) in [("X", 0), ("Y", 1), ("Z", 2)] {
                    if meas_m[i] > dims_m[i] + tol_m {
                        exceeded.push(format!(
                            "{axis}: {:.3} {u_str} > {:.3} {u_str} (excess {:.3} {u_str})",
                            meas_u[i],
                            target_u[i],
                            meas_u[i] - target_u[i]
                        ));
                    }
                }
                Some(format!(
                    "Part '{}' exceeds bbox_max along [{}]. Use 'modify_feature' or 'transform_part' to reduce dimensions.",
                    part.id,
                    exceeded.join(", ")
                ))
            };

            json!({
                "id": spec.id,
                "part_id": part.id,
                "scope": part.id,
                "type": "bbox_max",
                "status": if passed { RequirementStatus::Pass.as_str() } else { RequirementStatus::Fail.as_str() },
                "measured": meas_u,
                "target": target_u,
                "margin": min_margin_u,
                "margin_per_axis": margins_u,
                "hint": hint,
            })
        }

        RequirementKind::BboxMin { dims_m } => {
            let target_u = [
                dims_m[0] * k_from_m,
                dims_m[1] * k_from_m,
                dims_m[2] * k_from_m,
            ];

            // Scene-level bbox_min if no single part is specified
            if effective_part_id.is_none() && !state.parts.is_empty() {
                return evaluate_scene_bbox_requirement(spec, *dims_m, false, state, unit);
            }

            let Some(part) = resolve_target_part(effective_part_id, state) else {
                let missing = effective_part_id.unwrap_or("scene");
                return json!({
                    "id": spec.id,
                    "part_id": effective_part_id,
                    "scope": scope_label,
                    "type": "bbox_min",
                    "status": RequirementStatus::Fail.as_str(),
                    "measured": Value::Null,
                    "target": target_u,
                    "margin": Value::Null,
                    "hint": format!("Target part '{missing}' was not found or scene has no parts."),
                });
            };

            if part.mesh().vertices.is_empty() {
                return json!({
                    "id": spec.id,
                    "part_id": part.id,
                    "scope": part.id,
                    "type": "bbox_min",
                    "status": RequirementStatus::Fail.as_str(),
                    "measured": [0.0, 0.0, 0.0],
                    "target": target_u,
                    "margin": -target_u[0].max(target_u[1]).max(target_u[2]),
                    "hint": format!("Part '{}' has an empty mesh. Add 3D primitive features via 'add_feature_primitive'.", part.id),
                });
            }

            let aabb = part.bounding_box();
            let meas_m = aabb.dimensions();
            let meas_u = [
                meas_m[0] * k_from_m,
                meas_m[1] * k_from_m,
                meas_m[2] * k_from_m,
            ];
            let margins_u = [
                meas_u[0] - target_u[0],
                meas_u[1] - target_u[1],
                meas_u[2] - target_u[2],
            ];
            let min_margin_u = margins_u[0].min(margins_u[1]).min(margins_u[2]);
            let tol_m = 1e-7;
            let passed = meas_m[0] + tol_m >= dims_m[0]
                && meas_m[1] + tol_m >= dims_m[1]
                && meas_m[2] + tol_m >= dims_m[2];

            let hint = if passed {
                None
            } else {
                let mut undersized = Vec::new();
                for (axis, i) in [("X", 0), ("Y", 1), ("Z", 2)] {
                    if meas_m[i] + tol_m < dims_m[i] {
                        undersized.push(format!(
                            "{axis}: {:.3} {u_str} < {:.3} {u_str} (deficit {:.3} {u_str})",
                            meas_u[i],
                            target_u[i],
                            target_u[i] - meas_u[i]
                        ));
                    }
                }
                Some(format!(
                    "Part '{}' is smaller than bbox_min along [{}]. Use 'modify_feature' or 'transform_part' to increase dimensions.",
                    part.id,
                    undersized.join(", ")
                ))
            };

            json!({
                "id": spec.id,
                "part_id": part.id,
                "scope": part.id,
                "type": "bbox_min",
                "status": if passed { RequirementStatus::Pass.as_str() } else { RequirementStatus::Fail.as_str() },
                "measured": meas_u,
                "target": target_u,
                "margin": min_margin_u,
                "margin_per_axis": margins_u,
                "hint": hint,
            })
        }

        RequirementKind::MassMax { kg: max_kg } => {
            if effective_part_id.is_none() && !state.parts.is_empty() {
                let total_mass_kg: f64 = state.parts.values().map(|p| p.mass()).sum();
                let any_non_empty = state.parts.values().any(|p| !p.mesh().vertices.is_empty());
                let margin = max_kg - total_mass_kg;
                let passed = any_non_empty && total_mass_kg <= max_kg + 1e-12;
                let hint = if !any_non_empty {
                    Some("Scene parts have empty geometry (0 kg mass). Add features first.".to_string())
                } else if passed {
                    None
                } else {
                    Some(format!(
                        "Total scene mass ({total_mass_kg:.4} kg) exceeds mass_max ({max_kg:.4} kg) by {:.4} kg. Reduce part dimensions, add relief holes, or choose lighter materials.",
                        total_mass_kg - max_kg
                    ))
                };
                return json!({
                    "id": spec.id,
                    "part_id": Value::Null,
                    "scope": "scene",
                    "type": "mass_max",
                    "status": if passed { RequirementStatus::Pass.as_str() } else { RequirementStatus::Fail.as_str() },
                    "measured": total_mass_kg,
                    "target": *max_kg,
                    "margin": margin,
                    "hint": hint,
                });
            }

            let Some(part) = resolve_target_part(effective_part_id, state) else {
                let missing = effective_part_id.unwrap_or("scene");
                return json!({
                    "id": spec.id,
                    "part_id": effective_part_id,
                    "scope": scope_label,
                    "type": "mass_max",
                    "status": RequirementStatus::Fail.as_str(),
                    "measured": Value::Null,
                    "target": *max_kg,
                    "margin": Value::Null,
                    "hint": format!("Target part '{missing}' was not found."),
                });
            };

            if part.mesh().vertices.is_empty() {
                return json!({
                    "id": spec.id,
                    "part_id": part.id,
                    "scope": part.id,
                    "type": "mass_max",
                    "status": RequirementStatus::Fail.as_str(),
                    "measured": 0.0,
                    "target": *max_kg,
                    "margin": *max_kg,
                    "hint": format!("Part '{}' has an empty mesh (mass = 0 kg). Add 3D features first.", part.id),
                });
            }

            let measured_kg = part.mass();
            let margin = max_kg - measured_kg;
            let passed = measured_kg <= max_kg + 1e-12;
            let hint = if passed {
                None
            } else {
                Some(format!(
                    "Part '{}' mass ({measured_kg:.4} kg) exceeds mass_max ({max_kg:.4} kg) by {:.4} kg. Consider cutting relief holes ('cut_hole'), reducing dimensions ('modify_feature'), or switching to a lower-density material.",
                    part.id,
                    measured_kg - max_kg
                ))
            };

            json!({
                "id": spec.id,
                "part_id": part.id,
                "scope": part.id,
                "type": "mass_max",
                "status": if passed { RequirementStatus::Pass.as_str() } else { RequirementStatus::Fail.as_str() },
                "measured": measured_kg,
                "target": *max_kg,
                "margin": margin,
                "hint": hint,
            })
        }

        RequirementKind::MeshValid { expected_genus } => {
            if effective_part_id.is_none() && !state.parts.is_empty() {
                let mut all_pass = !state.parts.is_empty();
                let mut failed_parts = Vec::new();
                for p in state.parts.values() {
                    if p.mesh().vertices.is_empty() {
                        all_pass = false;
                        failed_parts.push(format!("{} (empty mesh)", p.id));
                    } else {
                        let rep = mcp_cad_core::validate_mesh(p.mesh(), *expected_genus);
                        if rep.status != "PASS" {
                            all_pass = false;
                            failed_parts.push(format!("{}: {}", p.id, rep.issues.join("; ")));
                        }
                    }
                }
                return json!({
                    "id": spec.id,
                    "part_id": Value::Null,
                    "scope": "scene",
                    "type": "mesh_valid",
                    "status": if all_pass { RequirementStatus::Pass.as_str() } else { RequirementStatus::Fail.as_str() },
                    "measured": {
                        "status": if all_pass { "PASS" } else { "FAIL" },
                        "failed_parts": failed_parts,
                    },
                    "target": { "status": "PASS", "is_watertight": true, "is_manifold": true },
                    "margin": if all_pass { 1.0 } else { -(failed_parts.len() as f64) },
                    "hint": if all_pass { None } else { Some(format!("Mesh validation failed on: [{}]. Call 'validate_mesh' to inspect defects.", failed_parts.join(", "))) },
                });
            }

            let Some(part) = resolve_target_part(effective_part_id, state) else {
                let missing = effective_part_id.unwrap_or("scene");
                return json!({
                    "id": spec.id,
                    "part_id": effective_part_id,
                    "scope": scope_label,
                    "type": "mesh_valid",
                    "status": RequirementStatus::Fail.as_str(),
                    "measured": Value::Null,
                    "target": { "status": "PASS", "is_watertight": true, "is_manifold": true },
                    "margin": -1.0,
                    "hint": format!("Target part '{missing}' was not found."),
                });
            };

            if part.mesh().vertices.is_empty() {
                return json!({
                    "id": spec.id,
                    "part_id": part.id,
                    "scope": part.id,
                    "type": "mesh_valid",
                    "status": RequirementStatus::Fail.as_str(),
                    "measured": {
                        "status": "FAIL",
                        "is_watertight": false,
                        "is_manifold": false,
                        "triangles_count": 0,
                    },
                    "target": {
                        "status": "PASS",
                        "is_watertight": true,
                        "is_manifold": true,
                    },
                    "margin": -1.0,
                    "hint": format!("Part '{}' has an empty mesh (0 triangles). Add features with 'add_feature_primitive'.", part.id),
                });
            }

            let report = mcp_cad_core::validate_mesh(part.mesh(), *expected_genus);
            let passed = report.status == "PASS";
            let is_manifold = report.topology.non_manifold_edges_count == 0
                && report.topology.non_manifold_vertices_count == 0;
            let hint = if passed {
                None
            } else {
                Some(format!(
                    "Mesh validation failed for part '{}': {}. Call 'validate_mesh' for full diagnostics or adjust feature parameters.",
                    part.id,
                    report.issues.join("; ")
                ))
            };

            json!({
                "id": spec.id,
                "part_id": part.id,
                "scope": part.id,
                "type": "mesh_valid",
                "status": if passed { RequirementStatus::Pass.as_str() } else { RequirementStatus::Fail.as_str() },
                "measured": {
                    "status": report.status,
                    "is_watertight": report.topology.is_watertight,
                    "is_manifold": is_manifold,
                    "open_edges_count": report.topology.boundary_edges_count,
                    "non_manifold_edges_count": report.topology.non_manifold_edges_count,
                    "self_intersections_count": report.geometry.self_intersections_count,
                    "euler_characteristic": report.topology.euler_characteristic,
                    "genus": report.topology.genus,
                    "min_angle_deg": report.angle_stats.min_angle_deg,
                },
                "target": {
                    "status": "PASS",
                    "is_watertight": true,
                    "is_manifold": true,
                    "open_edges_count": 0,
                    "expected_genus": expected_genus,
                },
                "margin": if passed { 1.0 } else { -(report.issues.len().max(1) as f64) },
                "hint": hint,
            })
        }

        RequirementKind::MinSafetyFactor { value: target_fos, fea_case } => {
            let Some(part) = resolve_target_part(effective_part_id, state) else {
                let missing = effective_part_id.unwrap_or("scene");
                return json!({
                    "id": spec.id,
                    "part_id": effective_part_id,
                    "scope": scope_label,
                    "type": "min_safety_factor",
                    "status": RequirementStatus::Stale.as_str(),
                    "measured": Value::Null,
                    "target": *target_fos,
                    "margin": Value::Null,
                    "hint": format!("Target part '{missing}' was not found. Create part and run 'run_fea_analysis'."),
                });
            };

            // Lookup FEA report and the revision at which it was computed
            let fea_entry = if let Some(case_name) = fea_case {
                state
                    .fea_case_reports
                    .get(&(part.id.clone(), case_name.clone()))
                    .map(|(rep, rev)| (rep, *rev))
                    .or_else(|| {
                        // Fallback to latest FEA report on part if no named case matched
                        state.last_fea_reports.get(&part.id).map(|rep| {
                            let rev = state.last_fea_revisions.get(&part.id).copied().unwrap_or(0);
                            (rep, rev)
                        })
                    })
            } else {
                state.last_fea_reports.get(&part.id).map(|rep| {
                    let rev = state.last_fea_revisions.get(&part.id).copied().unwrap_or(0);
                    (rep, rev)
                })
            };

            let Some((report, fea_rev)) = fea_entry else {
                return json!({
                    "id": spec.id,
                    "part_id": part.id,
                    "scope": part.id,
                    "type": "min_safety_factor",
                    "fea_case": fea_case,
                    "status": RequirementStatus::Stale.as_str(),
                    "measured": Value::Null,
                    "target": *target_fos,
                    "margin": Value::Null,
                    "hint": format!(
                        "No FEA report available for part '{}' (revision {}). Call 'run_fea_analysis' to evaluate Factor of Safety.",
                        part.id, part.revision
                    ),
                });
            };

            if fea_rev != part.revision {
                let stale_fos = report.factor_of_safety;
                return json!({
                    "id": spec.id,
                    "part_id": part.id,
                    "scope": part.id,
                    "type": "min_safety_factor",
                    "fea_case": fea_case,
                    "status": RequirementStatus::Stale.as_str(),
                    "measured": stale_fos,
                    "target": *target_fos,
                    "margin": stale_fos - target_fos,
                    "fea_revision": fea_rev,
                    "part_revision": part.revision,
                    "hint": format!(
                        "FEA result for part '{}' is STALE: computed at part revision {}, but current part revision is {}. Call 'run_fea_analysis' to recompute Factor of Safety.",
                        part.id, fea_rev, part.revision
                    ),
                });
            }

            let fos = report.factor_of_safety;
            let margin = fos - target_fos;
            let passed = fos + 1e-9 >= *target_fos;
            let hint = if passed {
                None
            } else {
                Some(format!(
                    "Part '{}' Factor of Safety ({fos:.3}) is below target ({target_fos:.3}); max von Mises stress is {:.2} MPa vs yield {:.2} MPa. Increase cross-section thickness ('modify_feature') or select a higher-strength material.",
                    part.id,
                    report.max_stress_von_mises_pa / 1.0e6,
                    report.yield_strength_pa / 1.0e6
                ))
            };

            json!({
                "id": spec.id,
                "part_id": part.id,
                "scope": part.id,
                "type": "min_safety_factor",
                "fea_case": fea_case,
                "status": if passed { RequirementStatus::Pass.as_str() } else { RequirementStatus::Fail.as_str() },
                "measured": fos,
                "target": *target_fos,
                "margin": margin,
                "max_von_mises_mpa": report.max_stress_von_mises_pa / 1.0e6,
                "yield_strength_mpa": report.yield_strength_pa / 1.0e6,
                "fea_revision": fea_rev,
                "part_revision": part.revision,
                "hint": hint,
            })
        }
    }
}

/// Resolves target `Part` from `effective_part_id`, or returns the sole part in the scene if `effective_part_id` is `None` and `state.parts.len() == 1`.
fn resolve_target_part<'a>(effective_part_id: Option<&str>, state: &'a EngineState) -> Option<&'a Part> {
    if let Some(pid) = effective_part_id {
        state.parts.get(pid)
    } else if state.parts.len() == 1 {
        state.parts.values().next()
    } else {
        None
    }
}

/// Evaluates `bbox_max` or `bbox_min` across the combined scene AABB when multiple parts exist.
fn evaluate_scene_bbox_requirement(
    spec: &RequirementSpec,
    dims_m: [f64; 3],
    is_max: bool,
    state: &EngineState,
    unit: LengthUnit,
) -> Value {
    let k_from_m = unit.from_meters_factor();
    let u_str = unit.as_str();
    let target_u = [
        dims_m[0] * k_from_m,
        dims_m[1] * k_from_m,
        dims_m[2] * k_from_m,
    ];
    let req_type = if is_max { "bbox_max" } else { "bbox_min" };

    let mut min_pt = [f64::INFINITY; 3];
    let mut max_pt = [f64::NEG_INFINITY; 3];
    let mut any_non_empty = false;

    for p in state.parts.values() {
        if !p.mesh().vertices.is_empty() {
            any_non_empty = true;
            let b = p.bounding_box();
            for i in 0..3 {
                min_pt[i] = min_pt[i].min(b.min[i]);
                max_pt[i] = max_pt[i].max(b.max[i]);
            }
        }
    }

    if !any_non_empty {
        return json!({
            "id": spec.id,
            "part_id": Value::Null,
            "scope": "scene",
            "type": req_type,
            "status": RequirementStatus::Fail.as_str(),
            "measured": [0.0, 0.0, 0.0],
            "target": target_u,
            "margin": Value::Null,
            "hint": "All parts in the scene have empty meshes. Add primitive features first.",
        });
    }

    let meas_m = [
        (max_pt[0] - min_pt[0]).max(0.0),
        (max_pt[1] - min_pt[1]).max(0.0),
        (max_pt[2] - min_pt[2]).max(0.0),
    ];
    let meas_u = [
        meas_m[0] * k_from_m,
        meas_m[1] * k_from_m,
        meas_m[2] * k_from_m,
    ];
    let margins_u = if is_max {
        [
            target_u[0] - meas_u[0],
            target_u[1] - meas_u[1],
            target_u[2] - meas_u[2],
        ]
    } else {
        [
            meas_u[0] - target_u[0],
            meas_u[1] - target_u[1],
            meas_u[2] - target_u[2],
        ]
    };
    let min_margin_u = margins_u[0].min(margins_u[1]).min(margins_u[2]);
    let tol_m = 1e-7;
    let passed = if is_max {
        meas_m[0] <= dims_m[0] + tol_m
            && meas_m[1] <= dims_m[1] + tol_m
            && meas_m[2] <= dims_m[2] + tol_m
    } else {
        meas_m[0] + tol_m >= dims_m[0]
            && meas_m[1] + tol_m >= dims_m[1]
            && meas_m[2] + tol_m >= dims_m[2]
    };

    let hint = if passed {
        None
    } else {
        Some(format!(
            "Scene combined bounding box [{:.3}, {:.3}, {:.3}] {u_str} violates {req_type} [{:.3}, {:.3}, {:.3}] {u_str}.",
            meas_u[0], meas_u[1], meas_u[2], target_u[0], target_u[1], target_u[2]
        ))
    };

    json!({
        "id": spec.id,
        "part_id": Value::Null,
        "scope": "scene",
        "type": req_type,
        "status": if passed { RequirementStatus::Pass.as_str() } else { RequirementStatus::Fail.as_str() },
        "measured": meas_u,
        "target": target_u,
        "margin": min_margin_u,
        "margin_per_axis": margins_u,
        "hint": hint,
    })
}

/// Evaluates declared requirements for `part_id_filter` (or all declared requirements if `None`).
pub fn evaluate_requirements_report(
    state: &EngineState,
    part_id_filter: Option<&str>,
    unit: LengthUnit,
) -> Value {
    let mut evaluated = Vec::new();

    if let Some(pid) = part_id_filter {
        if let Some(specs) = state.requirements.part_requirements.get(pid) {
            for spec in specs {
                evaluated.push(evaluate_single_requirement(spec, Some(pid), state, unit));
            }
        }
        // Also include any scene-level requirements that explicitly target this part_id
        // or apply globally when auto-checking a mutated part
        for spec in &state.requirements.scene_requirements {
            if spec.part_id.is_none() || spec.part_id.as_deref() == Some(pid) {
                evaluated.push(evaluate_single_requirement(spec, spec.part_id.as_deref(), state, unit));
            }
        }
    } else {
        let mut part_ids: Vec<&String> = state.requirements.part_requirements.keys().collect();
        part_ids.sort();
        for pid in part_ids {
            if let Some(specs) = state.requirements.part_requirements.get(pid) {
                for spec in specs {
                    evaluated.push(evaluate_single_requirement(spec, Some(pid.as_str()), state, unit));
                }
            }
        }
        for spec in &state.requirements.scene_requirements {
            evaluated.push(evaluate_single_requirement(spec, spec.part_id.as_deref(), state, unit));
        }
    }

    let mut passed = 0usize;
    let mut failed = 0usize;
    let mut stale = 0usize;
    let mut not_available = 0usize;
    let mut failed_requirements = Vec::new();

    for item in &evaluated {
        match item["status"].as_str().unwrap_or("") {
            "PASS" => passed += 1,
            "FAIL" => {
                failed += 1;
                failed_requirements.push(item.clone());
            }
            "STALE" => stale += 1,
            "NOT_AVAILABLE" => not_available += 1,
            _ => {}
        }
    }

    let overall_status = if failed > 0 {
        "FAIL"
    } else if stale > 0 {
        "STALE"
    } else if passed > 0 {
        "PASS"
    } else if not_available > 0 {
        "NOT_AVAILABLE"
    } else {
        "PASS"
    };

    json!({
        "status": overall_status,
        "overall_status": overall_status,
        "auto_check": state.requirements.auto_check,
        "total": evaluated.len(),
        "passed": passed,
        "failed": failed,
        "stale": stale,
        "not_available": not_available,
        "requirements": evaluated,
        "failed_requirements": failed_requirements,
        "failures": failed_requirements,
        "summary": {
            "status": overall_status,
            "total": evaluated.len(),
            "passed": passed,
            "failed": failed,
            "stale": stale,
            "not_available": not_available,
        },
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    })
}

/// If `auto_check` is enabled in `state.requirements`, evaluates active requirements and injects
/// `requirements_status: {passed, failed, stale, ...}` and `failed_requirements` into `resp`.
pub fn attach_auto_check_if_enabled(
    state: &EngineState,
    part_id_opt: Option<&str>,
    resp: &mut Value,
) {
    if !state.requirements.should_auto_check(part_id_opt) {
        return;
    }

    // Determine whether we have requirements for `part_id_opt`; if none are registered on that specific
    // part but requirements exist elsewhere in the scene, evaluate all scene/part requirements.
    let filter = match part_id_opt {
        Some(pid) if state.requirements.part_requirements.contains_key(pid) => Some(pid),
        _ => None,
    };

    let active_unit = resp
        .get("length_unit")
        .and_then(|v| v.as_str())
        .and_then(|s| LengthUnit::parse(s).ok())
        .unwrap_or(state.length_unit);

    let report = evaluate_requirements_report(state, filter, active_unit);
    let failed_list = report["failed_requirements"].clone();
    let status_summary = json!({
        "status": report["status"],
        "passed": report["passed"],
        "failed": report["failed"],
        "stale": report["stale"],
        "not_available": report["not_available"],
        "failed_requirements": failed_list,
        "failures": report["failed_requirements"],
        "requirements": report["requirements"],
    });

    if let Some(obj) = resp.as_object_mut() {
        obj.insert("requirements_status".to_string(), status_summary);
        obj.insert("failed_requirements".to_string(), failed_list);
    }
}

/// 32. `set_requirements`: Declares engineering requirements for a part or the whole scene.
pub fn set_requirements(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;

    let reqs_arr = args
        .get("requirements")
        .and_then(|v| v.as_array())
        .ok_or_else(|| {
            ServerError::InvalidArgument(
                "Missing required array 'requirements' in 'set_requirements'".to_string(),
            )
        })?;

    let part_id_arg = args.get("part_id").and_then(|v| v.as_str());
    let scope_arg = args.get("scope").and_then(|v| v.as_str());

    enum TargetScope {
        Part(String),
        Scene,
    }

    let target_scope = if let Some(pid) = part_id_arg {
        if pid.eq_ignore_ascii_case("scene") || pid.eq_ignore_ascii_case("global") {
            TargetScope::Scene
        } else {
            if !state.parts.contains_key(pid) {
                return Err(ServerError::PartNotFound(pid.to_string()));
            }
            TargetScope::Part(pid.to_string())
        }
    } else if let Some(sc) = scope_arg {
        if sc.eq_ignore_ascii_case("scene")
            || sc.eq_ignore_ascii_case("global")
            || sc.eq_ignore_ascii_case("assembly")
        {
            TargetScope::Scene
        } else if state.parts.contains_key(sc) {
            TargetScope::Part(sc.to_string())
        } else {
            return Err(ServerError::InvalidArgument(format!(
                "Invalid 'scope' value '{sc}': expected 'scene' or an existing part_id"
            )));
        }
    } else {
        return Err(ServerError::InvalidArgument(
            "Missing required 'part_id' or 'scope': \"scene\" in 'set_requirements'".to_string(),
        ));
    };

    let default_pid = match &target_scope {
        TargetScope::Part(pid) => Some(pid.as_str()),
        TargetScope::Scene => None,
    };

    let mut parsed_specs = Vec::with_capacity(reqs_arr.len());
    for (idx, item) in reqs_arr.iter().enumerate() {
        let spec = parse_requirement_spec(item, idx, default_pid, unit)?;
        if let Some(ref pid) = spec.part_id {
            if !state.parts.contains_key(pid) {
                return Err(ServerError::PartNotFound(pid.clone()));
            }
        }
        parsed_specs.push(spec);
    }

    if let Some(ac) = args.get("auto_check").and_then(|v| v.as_bool()) {
        state.requirements.auto_check = ac;
        if let TargetScope::Part(ref pid) = target_scope {
            state.requirements.part_auto_check.insert(pid.clone(), ac);
        }
    }

    let (scope_str, filter_pid) = match target_scope {
        TargetScope::Part(pid) => {
            state
                .requirements
                .part_requirements
                .insert(pid.clone(), parsed_specs.clone());
            (pid.clone(), Some(pid))
        }
        TargetScope::Scene => {
            state.requirements.scene_requirements = parsed_specs.clone();
            ("scene".to_string(), None)
        }
    };

    let eval_report = evaluate_requirements_report(state, filter_pid.as_deref(), unit);
    let auto_check_active = state.requirements.should_auto_check(filter_pid.as_deref());

    Ok(json!({
        "status": "requirements_set",
        "scope": scope_str,
        "part_id": filter_pid,
        "count": parsed_specs.len(),
        "requirements_count": parsed_specs.len(),
        "auto_check": auto_check_active,
        "requirements_status": {
            "status": eval_report["status"],
            "passed": eval_report["passed"],
            "failed": eval_report["failed"],
            "stale": eval_report["stale"],
            "not_available": eval_report["not_available"],
            "failed_requirements": eval_report["failed_requirements"],
            "failures": eval_report["failed_requirements"],
        },
        "requirements": eval_report["requirements"],
        "length_unit": unit.as_str(),
        "units": unit.units_metadata(),
    }))
}

/// 33. `check_requirements`: Evaluates declared engineering requirements for a part or the whole scene.
pub fn check_requirements(args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;

    let part_id_opt = args
        .get("part_id")
        .and_then(|v| v.as_str())
        .filter(|s| !s.eq_ignore_ascii_case("scene") && !s.eq_ignore_ascii_case("all"));

    if let Some(pid) = part_id_opt {
        if !state.parts.contains_key(pid) {
            return Err(ServerError::PartNotFound(pid.to_string()));
        }
    }

    Ok(evaluate_requirements_report(state, part_id_opt, unit))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tools::cad::{add_feature_primitive, create_part};

    #[test]
    fn test_p2_t30_attach_auto_check_respects_response_length_unit_override() {
        let mut state = EngineState::new().expect("EngineState init");
        assert_eq!(state.length_unit, LengthUnit::Mm);

        create_part(
            &json!({
                "part_id": "bracket_auto",
                "name": "Auto Bracket",
                "material": "aluminum_6061_t6"
            }),
            &mut state,
        )
        .unwrap();

        add_feature_primitive(
            &json!({
                "part_id": "bracket_auto",
                "primitive_type": "box",
                "params": { "width": 100.0, "height": 50.0, "depth": 20.0 }
            }),
            &mut state,
        )
        .unwrap();

        // Set a requirement with auto_check: true in mm
        set_requirements(
            &json!({
                "part_id": "bracket_auto",
                "auto_check": true,
                "requirements": [
                    { "id": "req_bbox", "type": "bbox_max", "dims": [120.0, 60.0, 30.0] }
                ]
            }),
            &mut state,
        )
        .unwrap();

        // 1. Mutation response with length_unit = "m"
        let mut resp_m = json!({
            "part_id": "bracket_auto",
            "length_unit": "m",
        });
        attach_auto_check_if_enabled(&state, Some("bracket_auto"), &mut resp_m);

        assert!(resp_m["requirements_status"].is_object());
        let req0 = &resp_m["requirements_status"]["requirements"][0];
        assert_eq!(req0["id"], "req_bbox");
        assert_eq!(req0["status"], "PASS");
        // Measured in meters: [0.1, 0.05, 0.02]
        let meas = req0["measured"].as_array().unwrap();
        assert!((meas[0].as_f64().unwrap() - 0.1).abs() < 1e-6);
        assert!((meas[1].as_f64().unwrap() - 0.05).abs() < 1e-6);
        assert!((meas[2].as_f64().unwrap() - 0.02).abs() < 1e-6);

        // 2. Mutation response with length_unit = "mm"
        let mut resp_mm = json!({
            "part_id": "bracket_auto",
            "length_unit": "mm",
        });
        attach_auto_check_if_enabled(&state, Some("bracket_auto"), &mut resp_mm);
        let req_mm = &resp_mm["requirements_status"]["requirements"][0];
        let meas_mm = req_mm["measured"].as_array().unwrap();
        assert!((meas_mm[0].as_f64().unwrap() - 100.0).abs() < 1e-3);
    }
}

