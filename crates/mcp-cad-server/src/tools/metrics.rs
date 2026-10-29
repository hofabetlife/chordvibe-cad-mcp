//! Comprehensive telemetry, simulation, and analysis metrics reporting.

use serde_json::{json, Value};

use crate::error::ServerError;
use crate::state::EngineState;

/// 15. `get_simulation_metrics_json`: Exports the complete engineering status and metrics report.
pub fn get_simulation_metrics_json(_args: &Value, state: &EngineState) -> Result<Value, ServerError> {
    let part_summaries: Vec<_> = state.parts.values().map(|p| p.summary()).collect();

    let fea_summaries: Vec<_> = state
        .last_fea_reports
        .values()
        .map(|r| {
            json!({
                "part_id": r.part_id,
                "verdict": r.verdict,
                "factor_of_safety": r.factor_of_safety,
                "max_von_mises_mpa": r.max_stress_von_mises_pa / 1.0e6,
                "yield_strength_mpa": r.yield_strength_pa / 1.0e6,
                "max_displacement_mm": r.max_displacement_m * 1000.0,
                "nodes": r.node_count,
                "elements": r.element_count,
                "time_ms": r.computation_time_ms
            })
        })
        .collect();

    let metrics = json!({
        "parts_count": state.parts.len(),
        "parts": part_summaries,
        "physics": {
            "accumulated_time_s": state.physics_world.current_time(),
            "gravity": state.physics_world.gravity(),
            "last_simulation": state.last_simulation
        },
        "fea_reports": fea_summaries,
    });

    Ok(json!({
        "current_metrics": metrics
    }))
}
