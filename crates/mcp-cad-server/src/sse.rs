//! HTTP and Server-Sent Events (SSE) server using Axum, including real-time
//! coupled dynamic `HHT-α` simulation baking, SSE progress streaming, job cancellation,
//! and compact per-frame cache endpoints (`P5-T6`).

use std::collections::HashMap;
use std::convert::Infallible;
use std::net::SocketAddr;
use std::sync::Arc;

use axum::{
    extract::{Path, Query, State},
    http::StatusCode,
    response::sse::{Event, KeepAlive, Sse},
    routing::{get, post},
    Json, Router,
};
use futures_util::stream::Stream;
use parking_lot::RwLock;
use serde::Deserialize;
use serde_json::{json, Value};
use tokio::sync::broadcast;
use tokio_stream::wrappers::BroadcastStream;
use tokio_stream::StreamExt;
use tower_http::cors::CorsLayer;

use mcp_cad_core::part::Part;
use mcp_cad_physics::PhysicsWorld;

use crate::error::ServerError;
use crate::protocol::{handle_request, JsonRpcRequest, JsonRpcResponse};
use crate::state::{LengthUnit, SharedState};
use crate::tools::coupled_sim::{
    launch_prepared_bake, prepare_coupled_bake_from_parts_with_physics,
    query_cached_fragment_meshes_from_manager, query_cached_frame_from_manager,
    CoupledSimSseEvent, SharedCoupledSimManager,
};

/// Internal state passed to Axum route handlers.
#[derive(Clone)]
pub struct AppState {
    pub engine: SharedState,
    pub tx: broadcast::Sender<String>,
    pub coupled_sim: SharedCoupledSimManager,
    pub coupled_sim_events_tx: broadcast::Sender<CoupledSimSseEvent>,
}

/// Lightweight Vulkan-independent state for the standalone coupled simulation router
/// (usable both standalone and in headless tests without a GPU/Vulkan ICD).
#[derive(Clone)]
pub struct CoupledSimHttpState {
    pub parts: Arc<RwLock<HashMap<String, Part>>>,
    pub physics_world: Arc<RwLock<PhysicsWorld>>,
    pub length_unit: Arc<RwLock<LengthUnit>>,
    pub manager: SharedCoupledSimManager,
    pub events_tx: broadcast::Sender<CoupledSimSseEvent>,
}

/// Optional query parameters for `GET /api/coupled-sim/frames/*`.
#[derive(Debug, Default, Deserialize)]
pub struct FrameQueryParams {
    pub part_id: Option<String>,
    pub include_mesh_fields: Option<bool>,
    pub include_fragment_meshes: Option<bool>,
    pub length_unit: Option<String>,
}

/// Optional query parameters for `GET /api/coupled-sim/fragments/*`.
#[derive(Debug, Default, Deserialize)]
pub struct FragmentQueryParams {
    pub length_unit: Option<String>,
}

fn error_response(err: ServerError) -> (StatusCode, Json<Value>) {
    let status = match &err {
        ServerError::InvalidArgument(_) | ServerError::PartNotFound(_) => StatusCode::BAD_REQUEST,
        _ => StatusCode::INTERNAL_SERVER_ERROR,
    };
    (status, Json(json!({ "error": err.to_string() })))
}

/// Handler for the `/sse` endpoint.
///
/// Sends an initial `endpoint` event referencing `/message`, followed by JSON-RPC responses
/// (`message` events) and real-time coupled simulation progress (`coupled_sim` events).
async fn sse_handler(
    State(app_state): State<AppState>,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    let rx_rpc = app_state.tx.subscribe();
    let rx_sim = app_state.coupled_sim_events_tx.subscribe();

    let initial = tokio_stream::once(Ok::<_, Infallible>(
        Event::default().event("endpoint").data("/message"),
    ));

    let rpc_stream = BroadcastStream::new(rx_rpc).filter_map(|res| match res {
        Ok(msg) => Some(Ok(Event::default().event("message").data(msg))),
        Err(_) => None,
    });

    let sim_stream = BroadcastStream::new(rx_sim).filter_map(|res| match res {
        Ok(ev) => serde_json::to_string(&ev)
            .ok()
            .map(|payload| Ok(Event::default().event("coupled_sim").data(payload))),
        Err(_) => None,
    });

    let merged = rpc_stream.merge(sim_stream);
    let combined = initial.chain(merged);

    Sse::new(combined).keep_alive(KeepAlive::default())
}

/// Handler for POST `/message` JSON-RPC calls.
async fn message_handler(
    State(app_state): State<AppState>,
    Json(req): Json<JsonRpcRequest>,
) -> Json<Option<JsonRpcResponse>> {
    let resp = handle_request(req, &app_state.engine).await;

    if let Some(ref r) = resp {
        if let Ok(serialized) = serde_json::to_string(r) {
            let _ = app_state.tx.send(serialized);
        }
    }

    Json(resp)
}

/// SSE endpoint streaming real-time [`CoupledSimSseEvent`] updates (`/api/coupled-sim/events`).
async fn coupled_sim_events_sse(
    tx: broadcast::Sender<CoupledSimSseEvent>,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    let rx = tx.subscribe();
    let initial = tokio_stream::once(Ok::<_, Infallible>(
        Event::default()
            .event("ready")
            .data(json!({ "channel": "coupled_sim" }).to_string()),
    ));
    let stream = BroadcastStream::new(rx).filter_map(|res| match res {
        Ok(ev) => serde_json::to_string(&ev)
            .ok()
            .map(|payload| Ok(Event::default().event("coupled_sim").data(payload))),
        Err(_) => None,
    });
    Sse::new(initial.chain(stream)).keep_alive(KeepAlive::default())
}

async fn app_coupled_events_handler(
    State(app_state): State<AppState>,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    coupled_sim_events_sse(app_state.coupled_sim_events_tx.clone()).await
}

async fn app_coupled_bake_handler(
    State(app_state): State<AppState>,
    Json(args): Json<Value>,
) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
    let (prepared, unit) = {
        let state = app_state.engine.read();
        let unit = state.resolve_length_unit(&args).map_err(error_response)?;
        let prepared = prepare_coupled_bake_from_parts_with_physics(
            &args,
            &state.parts,
            Some(&state.physics_world),
            unit,
        )
        .map_err(error_response)?;
        (prepared, unit)
    };
    let res = launch_prepared_bake(
        prepared,
        Arc::clone(&app_state.coupled_sim),
        app_state.coupled_sim_events_tx.clone(),
        unit,
    )
    .map_err(error_response)?;
    Ok(Json(res))
}

async fn app_coupled_status_latest_handler(
    State(app_state): State<AppState>,
) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
    let m = app_state.coupled_sim.read();
    Ok(Json(json!({
        "latest_job": m.job_status(None),
        "jobs": m.list_jobs(),
        "cached_simulations": m.list_cached_simulation_ids()
    })))
}

async fn app_coupled_status_by_id_handler(
    State(app_state): State<AppState>,
    Path(job_id): Path<String>,
) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
    let m = app_state.coupled_sim.read();
    let job = m.job_status(Some(&job_id)).ok_or_else(|| {
        error_response(ServerError::InvalidArgument(format!(
            "JOB_NOT_FOUND: No bake job with job_id '{job_id}'"
        )))
    })?;
    Ok(Json(json!({ "job": job })))
}

async fn app_coupled_cancel_handler(
    State(app_state): State<AppState>,
    Path(job_id): Path<String>,
) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
    let cancelled = {
        let mut m = app_state.coupled_sim.write();
        m.cancel_job(&job_id)
    };
    if !cancelled {
        return Err(error_response(ServerError::InvalidArgument(format!(
            "JOB_NOT_FOUND: No bake job with job_id '{job_id}'"
        ))));
    }
    Ok(Json(json!({
        "status": "cancel_requested",
        "job_id": job_id
    })))
}

async fn app_coupled_frame_latest_handler(
    State(app_state): State<AppState>,
    Path(frame_index): Path<usize>,
    Query(q): Query<FrameQueryParams>,
) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
    let default_unit = app_state.engine.read().length_unit;
    let unit = match q.length_unit.as_deref() {
        Some(s) => LengthUnit::parse(s).map_err(error_response)?,
        None => default_unit,
    };
    let args = json!({
        "frame_index": frame_index,
        "part_id": q.part_id,
        "include_mesh_fields": q.include_mesh_fields.unwrap_or(true),
        "include_fragment_meshes": q.include_fragment_meshes
    });
    let res =
        query_cached_frame_from_manager(&args, &app_state.coupled_sim, unit).map_err(error_response)?;
    Ok(Json(res))
}

async fn app_coupled_frame_by_sim_handler(
    State(app_state): State<AppState>,
    Path((sim_id, frame_index)): Path<(String, usize)>,
    Query(q): Query<FrameQueryParams>,
) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
    let default_unit = app_state.engine.read().length_unit;
    let unit = match q.length_unit.as_deref() {
        Some(s) => LengthUnit::parse(s).map_err(error_response)?,
        None => default_unit,
    };
    let args = json!({
        "simulation_id": sim_id,
        "frame_index": frame_index,
        "part_id": q.part_id,
        "include_mesh_fields": q.include_mesh_fields.unwrap_or(true),
        "include_fragment_meshes": q.include_fragment_meshes
    });
    let res =
        query_cached_frame_from_manager(&args, &app_state.coupled_sim, unit).map_err(error_response)?;
    Ok(Json(res))
}

async fn app_coupled_fragments_latest_handler(
    State(app_state): State<AppState>,
    Query(q): Query<FragmentQueryParams>,
) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
    let default_unit = app_state.engine.read().length_unit;
    let unit = match q.length_unit.as_deref() {
        Some(s) => LengthUnit::parse(s).map_err(error_response)?,
        None => default_unit,
    };
    let res = query_cached_fragment_meshes_from_manager(None, &app_state.coupled_sim, unit)
        .map_err(error_response)?;
    Ok(Json(res))
}

async fn app_coupled_fragments_by_sim_handler(
    State(app_state): State<AppState>,
    Path(sim_id): Path<String>,
    Query(q): Query<FragmentQueryParams>,
) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
    let default_unit = app_state.engine.read().length_unit;
    let unit = match q.length_unit.as_deref() {
        Some(s) => LengthUnit::parse(s).map_err(error_response)?,
        None => default_unit,
    };
    let res = query_cached_fragment_meshes_from_manager(Some(&sim_id), &app_state.coupled_sim, unit)
        .map_err(error_response)?;
    Ok(Json(res))
}

/// Creates the Axum router for the MCP HTTP/SSE server including coupled simulation baking,
/// real-time SSE progress streaming, job cancellation, and per-frame cache endpoints (`P5-T6`).
pub fn build_router(engine: SharedState) -> (Router, broadcast::Sender<String>) {
    let (tx, _) = broadcast::channel(128);
    let (coupled_sim, coupled_sim_events_tx) = {
        let guard = engine.read();
        (
            Arc::clone(&guard.coupled_sim),
            guard.coupled_sim_events_tx.clone(),
        )
    };
    let app_state = AppState {
        engine,
        tx: tx.clone(),
        coupled_sim,
        coupled_sim_events_tx,
    };

    let router = Router::new()
        .route("/sse", get(sse_handler))
        .route("/message", post(message_handler))
        .route("/api/coupled-sim/events", get(app_coupled_events_handler))
        .route("/api/coupled-sim/bake", post(app_coupled_bake_handler))
        .route("/api/coupled-sim/status", get(app_coupled_status_latest_handler))
        .route(
            "/api/coupled-sim/status/:job_id",
            get(app_coupled_status_by_id_handler),
        )
        .route(
            "/api/coupled-sim/cancel/:job_id",
            post(app_coupled_cancel_handler),
        )
        .route(
            "/api/coupled-sim/frames/:frame_index",
            get(app_coupled_frame_latest_handler),
        )
        .route(
            "/api/coupled-sim/frames/:sim_id/:frame_index",
            get(app_coupled_frame_by_sim_handler),
        )
        .route(
            "/api/coupled-sim/fragments",
            get(app_coupled_fragments_latest_handler),
        )
        .route(
            "/api/coupled-sim/fragments/:sim_id",
            get(app_coupled_fragments_by_sim_handler),
        )
        .layer(CorsLayer::permissive())
        .with_state(app_state);

    (router, tx)
}

/// Creates a standalone Vulkan-independent Axum router exposing only `/api/coupled-sim/*`
/// endpoints (for embedded deployments and headless integration testing).
pub fn build_coupled_sim_router(state: CoupledSimHttpState) -> Router {
    async fn events_h(
        State(st): State<CoupledSimHttpState>,
    ) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
        coupled_sim_events_sse(st.events_tx.clone()).await
    }

    async fn bake_h(
        State(st): State<CoupledSimHttpState>,
        Json(args): Json<Value>,
    ) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
        let unit = match args.get("length_unit").and_then(|v| v.as_str()) {
            Some(s) => LengthUnit::parse(s).map_err(error_response)?,
            None => *st.length_unit.read(),
        };
        let prepared = {
            let parts = st.parts.read();
            let physics_world = st.physics_world.read();
            prepare_coupled_bake_from_parts_with_physics(
                &args,
                &parts,
                Some(&physics_world),
                unit,
            )
            .map_err(error_response)?
        };
        let res = launch_prepared_bake(
            prepared,
            Arc::clone(&st.manager),
            st.events_tx.clone(),
            unit,
        )
        .map_err(error_response)?;
        Ok(Json(res))
    }

    async fn status_latest_h(State(st): State<CoupledSimHttpState>) -> Json<Value> {
        let m = st.manager.read();
        Json(json!({
            "latest_job": m.job_status(None),
            "jobs": m.list_jobs(),
            "cached_simulations": m.list_cached_simulation_ids()
        }))
    }

    async fn status_by_id_h(
        State(st): State<CoupledSimHttpState>,
        Path(job_id): Path<String>,
    ) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
        let m = st.manager.read();
        let job = m.job_status(Some(&job_id)).ok_or_else(|| {
            error_response(ServerError::InvalidArgument(format!(
                "JOB_NOT_FOUND: No bake job with job_id '{job_id}'"
            )))
        })?;
        Ok(Json(json!({ "job": job })))
    }

    async fn cancel_h(
        State(st): State<CoupledSimHttpState>,
        Path(job_id): Path<String>,
    ) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
        let cancelled = {
            let mut m = st.manager.write();
            m.cancel_job(&job_id)
        };
        if !cancelled {
            return Err(error_response(ServerError::InvalidArgument(format!(
                "JOB_NOT_FOUND: No bake job with job_id '{job_id}'"
            ))));
        }
        Ok(Json(json!({
            "status": "cancel_requested",
            "job_id": job_id
        })))
    }

    async fn frame_latest_h(
        State(st): State<CoupledSimHttpState>,
        Path(frame_index): Path<usize>,
        Query(q): Query<FrameQueryParams>,
    ) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
        let default_unit = *st.length_unit.read();
        let unit = match q.length_unit.as_deref() {
            Some(s) => LengthUnit::parse(s).map_err(error_response)?,
            None => default_unit,
        };
        let args = json!({
            "frame_index": frame_index,
            "part_id": q.part_id,
            "include_mesh_fields": q.include_mesh_fields.unwrap_or(true),
            "include_fragment_meshes": q.include_fragment_meshes
        });
        let res = query_cached_frame_from_manager(&args, &st.manager, unit).map_err(error_response)?;
        Ok(Json(res))
    }

    async fn frame_by_sim_h(
        State(st): State<CoupledSimHttpState>,
        Path((sim_id, frame_index)): Path<(String, usize)>,
        Query(q): Query<FrameQueryParams>,
    ) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
        let default_unit = *st.length_unit.read();
        let unit = match q.length_unit.as_deref() {
            Some(s) => LengthUnit::parse(s).map_err(error_response)?,
            None => default_unit,
        };
        let args = json!({
            "simulation_id": sim_id,
            "frame_index": frame_index,
            "part_id": q.part_id,
            "include_mesh_fields": q.include_mesh_fields.unwrap_or(true),
            "include_fragment_meshes": q.include_fragment_meshes
        });
        let res = query_cached_frame_from_manager(&args, &st.manager, unit).map_err(error_response)?;
        Ok(Json(res))
    }

    async fn fragments_latest_h(
        State(st): State<CoupledSimHttpState>,
        Query(q): Query<FragmentQueryParams>,
    ) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
        let default_unit = *st.length_unit.read();
        let unit = match q.length_unit.as_deref() {
            Some(s) => LengthUnit::parse(s).map_err(error_response)?,
            None => default_unit,
        };
        let res = query_cached_fragment_meshes_from_manager(None, &st.manager, unit)
            .map_err(error_response)?;
        Ok(Json(res))
    }

    async fn fragments_by_sim_h(
        State(st): State<CoupledSimHttpState>,
        Path(sim_id): Path<String>,
        Query(q): Query<FragmentQueryParams>,
    ) -> Result<Json<Value>, (StatusCode, Json<Value>)> {
        let default_unit = *st.length_unit.read();
        let unit = match q.length_unit.as_deref() {
            Some(s) => LengthUnit::parse(s).map_err(error_response)?,
            None => default_unit,
        };
        let res = query_cached_fragment_meshes_from_manager(Some(&sim_id), &st.manager, unit)
            .map_err(error_response)?;
        Ok(Json(res))
    }

    Router::new()
        .route("/api/coupled-sim/events", get(events_h))
        .route("/api/coupled-sim/bake", post(bake_h))
        .route("/api/coupled-sim/status", get(status_latest_h))
        .route("/api/coupled-sim/status/:job_id", get(status_by_id_h))
        .route("/api/coupled-sim/cancel/:job_id", post(cancel_h))
        .route("/api/coupled-sim/frames/:frame_index", get(frame_latest_h))
        .route(
            "/api/coupled-sim/frames/:sim_id/:frame_index",
            get(frame_by_sim_h),
        )
        .route("/api/coupled-sim/fragments", get(fragments_latest_h))
        .route("/api/coupled-sim/fragments/:sim_id", get(fragments_by_sim_h))
        .layer(CorsLayer::permissive())
        .with_state(state)
}

/// Runs the HTTP/SSE server on the specified port.
pub async fn run_sse(
    engine: SharedState,
    port: u16,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let addr = SocketAddr::from(([127, 0, 0, 1], port));
    let (app, _) = build_router(engine);

    tracing::info!("MCP-CAD SSE Server listening on http://{}", addr);

    let listener = tokio::net::TcpListener::bind(addr).await?;
    axum::serve(listener, app).await?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::Request;
    use mcp_cad_core::material::MaterialRegistry;
    use mcp_cad_core::primitives::create_box;
    use tower::ServiceExt;

    use crate::tools::coupled_sim::CoupledSimulationManager;

    fn make_test_state_with_beam() -> CoupledSimHttpState {
        let reg = MaterialRegistry::new();
        let steel = reg.get("steel_structural_aisi1020").expect("steel").clone();
        let mut beam = Part::new("beam_1", "Cantilever Beam", steel);
        beam.mesh = create_box(0.10, 0.02, 0.02);

        let mut parts = HashMap::new();
        parts.insert("beam_1".to_string(), beam);

        CoupledSimHttpState {
            parts: Arc::new(RwLock::new(parts)),
            physics_world: Arc::new(RwLock::new(PhysicsWorld::new())),
            length_unit: Arc::new(RwLock::new(LengthUnit::Mm)),
            manager: Arc::new(RwLock::new(CoupledSimulationManager::new())),
            events_tx: broadcast::channel(256).0,
        }
    }

    async fn read_json_body(resp: axum::response::Response) -> Value {
        let bytes = axum::body::to_bytes(resp.into_body(), 8 * 1024 * 1024)
            .await
            .expect("read body");
        serde_json::from_slice(&bytes).expect("valid json")
    }

    #[tokio::test]
    async fn test_p5_t6_http_bake_sse_progress_and_frame_cache_query() {
        let state = make_test_state_with_beam();
        let mut sse_rx = state.events_tx.subscribe();
        let app = build_coupled_sim_router(state);

        // 1. Trigger coupled HHT-alpha bake over HTTP API
        let req_body = json!({
            "accuracy_level": "standard",
            "part_ids": ["beam_1"],
            "duration_s": 0.008,
            "dt_ms": 1.0,
            "hht_alpha": -0.05,
            "mass_matrix": "consistent",
            "linear_solver": "auto",
            "enable_plasticity": true,
            "enable_fracture": false,
            "fixed_faces": [{"part_id": "beam_1", "face": "min_x"}],
            "loads": [{"part_id": "beam_1", "face": "max_x", "force": [0.0, -500.0, 0.0]}],
            "async_mode": false
        });

        let bake_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/coupled-sim/bake")
                    .header("content-type", "application/json")
                    .body(Body::from(req_body.to_string()))
                    .unwrap(),
            )
            .await
            .expect("bake response");
        assert_eq!(bake_resp.status(), StatusCode::OK);
        let bake_json = read_json_body(bake_resp).await;
        assert_eq!(bake_json["status"], "completed");
        let sim_id = bake_json["simulation_id"].as_str().expect("simulation_id");
        let frame_count = bake_json["frame_count"].as_u64().expect("frame_count") as usize;
        assert!(frame_count >= 5, "Expected >= 5 cached frames, got {frame_count}");
        assert!(
            bake_json["cfl_ratio_max"].as_f64().unwrap() > 10.0,
            "Implicit HHT-alpha step must exceed explicit CFL limit"
        );

        // 2. Verify SSE channel delivered Started -> Progress -> Completed events
        let mut saw_started = false;
        let mut progress_count = 0usize;
        let mut saw_completed = false;
        while let Ok(ev) = sse_rx.try_recv() {
            match ev {
                CoupledSimSseEvent::Started { .. } => saw_started = true,
                CoupledSimSseEvent::Progress { .. } => progress_count += 1,
                CoupledSimSseEvent::Completed { .. } => saw_completed = true,
                _ => {}
            }
        }
        assert!(saw_started, "SSE stream must emit Started event");
        assert!(progress_count >= 1, "SSE stream must emit Progress events");
        assert!(saw_completed, "SSE stream must emit Completed event");

        // 3. Query compact cached frame via HTTP GET /api/coupled-sim/frames/:sim_id/:frame_index
        let last_idx = frame_count - 1;
        let frame_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri(format!("/api/coupled-sim/frames/{sim_id}/{last_idx}"))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .expect("frame response");
        assert_eq!(frame_resp.status(), StatusCode::OK);
        let frame_json = read_json_body(frame_resp).await;
        assert_eq!(frame_json["simulation_id"], sim_id);
        assert_eq!(frame_json["frame_index"], last_idx);
        let parts_arr = frame_json["parts"].as_array().expect("parts array");
        assert_eq!(parts_arr.len(), 1);
        let part0 = &parts_arr[0];
        let n_nodes = part0["node_count"].as_u64().unwrap() as usize;
        assert_eq!(part0["deformed_positions"].as_array().unwrap().len(), n_nodes);
        assert_eq!(part0["nodal_von_mises_mpa"].as_array().unwrap().len(), n_nodes);
        assert_eq!(part0["nodal_equiv_plastic_strain"].as_array().unwrap().len(), n_nodes);
        assert_eq!(part0["nodal_damage"].as_array().unwrap().len(), n_nodes);
        assert!(part0["max_von_mises_mpa"].as_f64().unwrap() > 0.0);
        assert!(frame_json["energy_balance"]["total_mechanical_j"].as_f64().unwrap() >= 0.0);

        // 4. Also verify GET /api/coupled-sim/events returns text/event-stream
        let sse_resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri("/api/coupled-sim/events")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .expect("sse response");
        assert_eq!(sse_resp.status(), StatusCode::OK);
        let ct = sse_resp
            .headers()
            .get("content-type")
            .and_then(|v| v.to_str().ok())
            .unwrap_or("");
        assert!(ct.contains("text/event-stream"), "Unexpected SSE content-type: {ct}");
    }

    #[tokio::test]
    async fn test_p5_t6_async_bake_and_cancellation_endpoint() {
        let state = make_test_state_with_beam();
        let mut sse_rx = state.events_tx.subscribe();
        let app = build_coupled_sim_router(state.clone());

        // Start a long-running async bake (2000 steps) so cancellation interrupts it cleanly
        let req_body = json!({
            "accuracy_level": "standard",
            "part_ids": ["beam_1"],
            "duration_s": 2.0,
            "dt_ms": 0.5,
            "dt_max_ms": 0.5,
            "hht_alpha": -0.05,
            "enable_plasticity": true,
            "enable_fracture": false,
            "fixed_faces": [{"part_id": "beam_1", "face": "min_x"}],
            "loads": [{"part_id": "beam_1", "face": "max_x", "force": [0.0, -500.0, 0.0]}],
            "async_mode": true
        });

        let start_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/coupled-sim/bake")
                    .header("content-type", "application/json")
                    .body(Body::from(req_body.to_string()))
                    .unwrap(),
            )
            .await
            .expect("async bake start");
        assert_eq!(start_resp.status(), StatusCode::OK);
        let start_json = read_json_body(start_resp).await;
        assert_eq!(start_json["status"], "running");
        let job_id = start_json["job_id"].as_str().expect("job_id").to_string();

        // Wait briefly for worker to start, then cancel via POST /api/coupled-sim/cancel/:job_id
        tokio::time::sleep(std::time::Duration::from_millis(15)).await;
        let cancel_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri(format!("/api/coupled-sim/cancel/{job_id}"))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .expect("cancel response");
        assert_eq!(cancel_resp.status(), StatusCode::OK);
        let cancel_json = read_json_body(cancel_resp).await;
        assert_eq!(cancel_json["status"], "cancel_requested");

        // Poll status until worker settles into cancelled (or completed if machine was extraordinarily fast)
        let mut final_state = String::new();
        for _ in 0..80 {
            let st_resp = app
                .clone()
                .oneshot(
                    Request::builder()
                        .method("GET")
                        .uri(format!("/api/coupled-sim/status/{job_id}"))
                        .body(Body::empty())
                        .unwrap(),
                )
                .await
                .expect("status response");
            let st_json = read_json_body(st_resp).await;
            final_state = st_json["job"]["state"].as_str().unwrap_or("").to_string();
            if final_state == "cancelled" || final_state == "completed" {
                break;
            }
            tokio::time::sleep(std::time::Duration::from_millis(15)).await;
        }
        assert_eq!(
            final_state, "cancelled",
            "Active bake job should transition cleanly to 'cancelled'"
        );

        // Drain SSE events to confirm lifecycle event was emitted
        let mut saw_terminal = false;
        while let Ok(ev) = sse_rx.try_recv() {
            if matches!(ev, CoupledSimSseEvent::Cancelled { .. }) {
                saw_terminal = true;
            }
        }
        // Even if Cancelled fired before subscribe drain check, confirm partial frames cached
        let cached = state.manager.read().get_simulation(None).expect("cached partial sim");
        assert!(cached.cancelled || saw_terminal);
    }

    #[test]
    fn test_p5_t6_mcp_tool_registry_contract_43_tools() {
        let tools = crate::tools::list_tools();
        assert_eq!(tools.len(), 43, "Expected 43 MCP tools after P5-T6");

        let unit_keywords = [
            "length_unit",
            "mm",
            "m/s",
            "rad/s",
            "deg",
            "degree",
            "mpa",
            "gpa",
            "n*m",
            "n·m",
            "newton",
            "(n)",
            "kg",
            "px",
            "pixel",
            "second",
            "(s)",
            "(ms)",
            "millisecond",
            "watt",
            "kw",
            "rpm",
            "dimensionless",
        ];

        for name in ["bake_coupled_simulation", "query_simulation_frame"] {
            let tool = tools
                .iter()
                .find(|t| t.name == name)
                .unwrap_or_else(|| panic!("Missing MCP tool '{name}'"));
            assert!(tool.description.contains("Example"));
            assert!(tool.description.contains("Units:"));
            assert!(tool.description.contains("Common errors:"));
            assert!(tool.description.contains("Next step:"));

            let props = tool.input_schema["properties"].as_object().unwrap();
            for (k, v) in props {
                let ty = v.get("type").and_then(|t| t.as_str()).unwrap_or("");
                let is_num_arr = ty == "array"
                    && v.get("items")
                        .and_then(|i| i.get("type"))
                        .and_then(|t| t.as_str())
                        == Some("number");
                if ty == "number" || ty == "integer" || is_num_arr {
                    let desc = v
                        .get("description")
                        .and_then(|d| d.as_str())
                        .unwrap_or("")
                        .to_ascii_lowercase();
                    assert!(
                        unit_keywords.iter().any(|kw| desc.contains(kw)),
                        "Tool '{name}' numeric property '{k}' missing unit keyword in description: '{desc}'"
                    );
                }
            }
        }
    }

    #[tokio::test]
    async fn test_p5_t14_bake_coupled_simulation_auto_extracts_revolute_joints_and_motor() {
        use mcp_cad_core::primitives::create_spur_gear;
        use mcp_cad_physics::{CadJointType, MotorConfig};

        let reg = MaterialRegistry::new();
        let steel = reg.get("steel_structural_aisi1020").expect("steel").clone();

        // Build a meshing spur-gear pair (8T + 8T, module 4 mm, pitch diameter 32 mm,
        // realistic 0.4 mm tooth backlash so coarse tet meshes start stress-free at t = 0
        // and cleanly engage under the Revolute motor drive).
        let module_m = 0.004;
        let teeth = 8usize;
        let face_width = 0.008;
        let bore_r = 0.005;
        let backlash_m = 0.0004;
        let center_dist = module_m * teeth as f64 + backlash_m;

        let mut pinion = Part::new("pinion", "Driver Pinion", steel.clone());
        pinion.mesh = create_spur_gear(
            module_m,
            teeth,
            20.0,
            face_width,
            Some(bore_r),
            None,
            None,
            None,
            None,
        );

        let mut gear = Part::new("gear", "Driven Gear", steel);
        let mut gear_mesh = create_spur_gear(
            module_m,
            teeth,
            20.0,
            face_width,
            Some(bore_r),
            None,
            None,
            None,
            None,
        );
        // Rotate driven gear by half-tooth pitch (180/8 = 22.5 deg) around Z and translate to +center_dist
        let phase_rad = std::f64::consts::PI / (teeth as f64);
        let (s, c) = phase_rad.sin_cos();
        for v in &mut gear_mesh.vertices {
            let x = v.position[0];
            let y = v.position[1];
            v.position[0] = c * x - s * y + center_dist;
            v.position[1] = s * x + c * y;
            let nx = v.normal[0];
            let ny = v.normal[1];
            v.normal[0] = c * nx - s * ny;
            v.normal[1] = s * nx + c * ny;
        }
        gear.mesh = gear_mesh;

        // Register both parts and Revolute + Motor + Gear joints in PhysicsWorld
        let mut pw = PhysicsWorld::new();
        pw.add_part(&pinion, true).expect("add pinion rigid body");
        pw.add_part(&gear, true).expect("add gear rigid body");

        pw.add_joint(
            "pinion_revolute",
            "ground",
            "pinion",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig::new(20.0, 80.0)),
        )
        .expect("add pinion revolute motor");

        pw.add_joint(
            "gear_revolute",
            "ground",
            "gear",
            CadJointType::Revolute {
                anchor_a: [center_dist, 0.0, 0.0],
                anchor_b: [center_dist, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("add gear revolute");

        pw.add_joint(
            "gear_mesh",
            "pinion",
            "gear",
            CadJointType::Gear {
                ratio: 1.0,
                axis_a: [0.0, 0.0, 1.0],
                axis_b: [0.0, 0.0, 1.0],
            },
            None,
        )
        .expect("add gear mesh joint");

        let mut parts = HashMap::new();
        parts.insert("pinion".to_string(), pinion);
        parts.insert("gear".to_string(), gear);

        let state = CoupledSimHttpState {
            parts: Arc::new(RwLock::new(parts)),
            physics_world: Arc::new(RwLock::new(pw)),
            length_unit: Arc::new(RwLock::new(LengthUnit::Mm)),
            manager: Arc::new(RwLock::new(CoupledSimulationManager::new())),
            events_tx: broadcast::channel(256).0,
        };
        let app = build_coupled_sim_router(state);

        // Bake WITHOUT manual initial_velocities, fixed_faces, or loads: PhysicsWorld joints drive the scene!
        let req_body = json!({
            "accuracy_level": "standard",
            "part_ids": ["pinion", "gear"],
            "duration_s": 0.008,
            "dt_ms": 0.4,
            "hht_alpha": -0.05,
            "mass_matrix": "lumped",
            "enable_plasticity": false,
            "enable_fracture": false,
            "enable_contact": true,
            "async_mode": false
        });

        let bake_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/coupled-sim/bake")
                    .header("content-type", "application/json")
                    .body(Body::from(req_body.to_string()))
                    .unwrap(),
            )
            .await
            .expect("bake response");
        let status = bake_resp.status();
        let bake_json = read_json_body(bake_resp).await;
        assert_eq!(
            status,
            StatusCode::OK,
            "Expected HTTP 200 from /api/coupled-sim/bake, got {status}: {bake_json}"
        );
        assert_eq!(bake_json["status"], "completed");
        assert_eq!(
            bake_json["applied_joint_couplings_count"], 2,
            "Expected 2 extracted Revolute hub couplings from PhysicsWorld"
        );
        assert_eq!(
            bake_json["applied_gear_couplings"].as_array().unwrap().len(),
            1,
            "Expected 1 extracted Gear coupling from PhysicsWorld"
        );

        let sim_id = bake_json["simulation_id"].as_str().expect("simulation_id");
        let frame_count = bake_json["frame_count"].as_u64().expect("frame_count") as usize;
        assert!(frame_count >= 4, "Expected >= 4 cached frames, got {frame_count}");

        // Scan cached frames for contact engagement, hub kinematics, and torque transmission
        let mut max_contact_n = 0.0_f64;
        let mut peak_pinion_omega = 0.0_f64;
        let mut min_gear_omega = 0.0_f64;
        let mut peak_hub_torque_nm = 0.0_f64;
        let mut last_frame = Value::Null;
        for fidx in 0..frame_count {
            let f_resp = app
                .clone()
                .oneshot(
                    Request::builder()
                        .method("GET")
                        .uri(format!(
                            "/api/coupled-sim/frames/{sim_id}/{fidx}?include_mesh_fields=false"
                        ))
                        .body(Body::empty())
                        .unwrap(),
                )
                .await
                .expect("frame query");
            assert_eq!(f_resp.status(), StatusCode::OK);
            let fj = read_json_body(f_resp).await;
            if let Some(cfs) = fj["contact_forces"].as_array() {
                for cf in cfs {
                    if let Some(fn_val) = cf["normal_force_n"].as_f64() {
                        max_contact_n = max_contact_n.max(fn_val);
                    }
                }
            }
            if let Some(hubs) = fj["hub_reactions"].as_array() {
                for h in hubs {
                    let vel = h["velocity"].as_f64().unwrap_or(0.0);
                    let eff = h["reaction_effort"].as_f64().unwrap_or(0.0).abs();
                    peak_hub_torque_nm = peak_hub_torque_nm.max(eff);
                    if h["part_id"] == "pinion" {
                        peak_pinion_omega = peak_pinion_omega.max(vel);
                    } else if h["part_id"] == "gear" {
                        min_gear_omega = min_gear_omega.min(vel);
                    }
                }
            }
            if fidx + 1 == frame_count {
                last_frame = fj;
            }
        }

        let hubs = last_frame["hub_reactions"]
            .as_array()
            .expect("hub_reactions array in queried frame");
        assert_eq!(hubs.len(), 2);
        let pinion_hub = hubs
            .iter()
            .find(|h| h["part_id"] == "pinion")
            .expect("pinion hub");
        let gear_hub = hubs
            .iter()
            .find(|h| h["part_id"] == "gear")
            .expect("gear hub");

        let pinion_theta = pinion_hub["coordinate"].as_f64().unwrap();
        let gear_theta = gear_hub["coordinate"].as_f64().unwrap();

        assert!(
            pinion_theta > 0.02 && peak_pinion_omega > 5.0,
            "Pinion must rotate CCW under extracted PhysicsWorld motor: theta={pinion_theta:.5} rad, peak_omega={peak_pinion_omega:.3} rad/s"
        );
        assert!(
            max_contact_n > 1.0,
            "Expected dynamic tooth-meshing contact force > 1.0 N, got {max_contact_n:.4} N"
        );
        assert!(
            gear_theta < -0.002 && min_gear_omega < -0.5,
            "Driven gear must rotate CW via tooth contact without manual initial_velocities: theta={gear_theta:.5} rad, min_omega={min_gear_omega:.3} rad/s"
        );
        assert!(
            peak_hub_torque_nm > 0.01,
            "Expected non-zero hub reaction torque during gear meshing, got {peak_hub_torque_nm:.6} N*m"
        );
    }

    #[test]
    fn test_p5_t15_active_surface_topology_outward_winding_and_erosion_cavity() {
        use crate::tools::coupled_sim::extract_active_surface_topology;
        use mcp_cad_fea::tetra_mesh::TetrahedralMesh;

        let signed_volume = |mesh: &TetrahedralMesh, topo: &crate::tools::coupled_sim::ActiveSurfaceTopology| -> f64 {
            let mut vol6 = 0.0_f64;
            for tri in topo.surface_indices.chunks_exact(3) {
                let a = mesh.nodes[topo.surface_node_indices[tri[0] as usize] as usize].position;
                let b = mesh.nodes[topo.surface_node_indices[tri[1] as usize] as usize].position;
                let c = mesh.nodes[topo.surface_node_indices[tri[2] as usize] as usize].position;
                let cr = [
                    b[1] * c[2] - b[2] * c[1],
                    b[2] * c[0] - b[0] * c[2],
                    b[0] * c[1] - b[1] * c[0],
                ];
                vol6 += a[0] * cr[0] + a[1] * cr[1] + a[2] * cr[2];
            }
            vol6 / 6.0
        };

        let expected_vol = 0.04 * 0.02 * 0.02;
        let tet4 = TetrahedralMesh::from_box([0.0, 0.0, 0.0], [0.04, 0.02, 0.02], 4, 2, 2)
            .expect("tet4 box");
        let all_active = vec![true; tet4.elements.len()];
        let topo4 = extract_active_surface_topology(&tet4, &all_active);
        assert!(!topo4.surface_node_indices.is_empty());
        assert!(topo4.surface_indices.len() >= 12 * 3);
        assert_eq!(topo4.surface_indices.len() % 3, 0);
        for &idx in &topo4.surface_indices {
            assert!((idx as usize) < topo4.surface_node_indices.len());
        }
        let vol4 = signed_volume(&tet4, &topo4);
        assert!(
            (vol4 - expected_vol).abs() < 1e-9,
            "TET4 active surface must have strict outward CCW winding: vol={vol4:.9e}, expected={expected_vol:.9e}"
        );

        // Erode 2 interior/boundary elements -> new cavity faces are exposed and signed volume drops strictly
        let mut eroded_mask = all_active.clone();
        eroded_mask[0] = false;
        eroded_mask[1] = false;
        let topo4_eroded = extract_active_surface_topology(&tet4, &eroded_mask);
        let vol4_eroded = signed_volume(&tet4, &topo4_eroded);
        assert!(
            vol4_eroded > 0.0 && vol4_eroded < vol4,
            "Eroded TET4 surface volume must drop strictly while remaining outward oriented: {vol4_eroded:.9e} vs {vol4:.9e}"
        );
        assert_ne!(topo4.surface_indices, topo4_eroded.surface_indices);

        // Verify TET10 quadratic element 4-sub-triangle outward subdivision as well
        let tet10 = tet4.to_tet10();
        let topo10 = extract_active_surface_topology(&tet10, &all_active);
        assert_eq!(
            topo10.surface_indices.len(),
            topo4.surface_indices.len() * 4,
            "Each 6-node TET10 exterior face must split into 4 conforming outward sub-triangles"
        );
        let vol10 = signed_volume(&tet10, &topo10);
        assert!(
            (vol10 - expected_vol).abs() < 1e-9,
            "TET10 active surface must have strict outward CCW winding: vol={vol10:.9e}, expected={expected_vol:.9e}"
        );
    }

    #[tokio::test]
    async fn test_p5_t15_query_simulation_frame_returns_active_surface_mesh_and_fragment_geometry() {
        let reg = MaterialRegistry::new();
        let glass = reg.get("borosilicate_glass").expect("borosilicate_glass").clone();
        let mut bar = Part::new("glass_bar", "Brittle Glass Bar", glass);
        // Cantilever bar under pure transverse tip bending: bending moment M(x) = F*(L - x)
        // peaks at the clamped root (min_x) and vanishes at the free tip (max_x), so the root
        // cross-section erodes completely and detaches the unstressed distal half as a real FEM fragment.
        bar.mesh = create_box(0.10, 0.016, 0.016);

        let mut parts = HashMap::new();
        parts.insert("glass_bar".to_string(), bar);

        let state = CoupledSimHttpState {
            parts: Arc::new(RwLock::new(parts)),
            physics_world: Arc::new(RwLock::new(PhysicsWorld::new())),
            length_unit: Arc::new(RwLock::new(LengthUnit::M)),
            manager: Arc::new(RwLock::new(CoupledSimulationManager::new())),
            events_tx: broadcast::channel(256).0,
        };
        let app = build_coupled_sim_router(state);

        let req_body = json!({
            "accuracy_level": "standard",
            "mesh_density": "medium",
            "part_ids": ["glass_bar"],
            "duration_s": 0.005,
            "dt_ms": 0.5,
            "hht_alpha": -0.05,
            "mass_matrix": "lumped",
            "enable_plasticity": true,
            "enable_fracture": true,
            "enable_contact": false,
            "fixed_faces": [{"part_id": "glass_bar", "face": "min_x"}],
            "loads": [{"part_id": "glass_bar", "face": "max_x", "force": [0.0, -4000.0, 0.0]}],
            "async_mode": false
        });

        let bake_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/coupled-sim/bake")
                    .header("content-type", "application/json")
                    .body(Body::from(req_body.to_string()))
                    .unwrap(),
            )
            .await
            .expect("bake response");
        assert_eq!(bake_resp.status(), StatusCode::OK);
        let bake_json = read_json_body(bake_resp).await;
        assert_eq!(bake_json["status"], "completed");

        let sim_id = bake_json["simulation_id"].as_str().expect("simulation_id");
        let frame_count = bake_json["frame_count"].as_u64().expect("frame_count") as usize;
        assert!(frame_count >= 3, "Expected >= 3 frames, got {frame_count}");
        assert_eq!(
            bake_json["fragment_meshes_endpoint"].as_str(),
            Some(format!("/api/coupled-sim/fragments/{sim_id}").as_str())
        );

        // 1. Verify frame 0 active FEM surface mesh payload
        let f0_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri(format!(
                        "/api/coupled-sim/frames/{sim_id}/0?include_mesh_fields=true&length_unit=m"
                    ))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .expect("frame 0 response");
        assert_eq!(f0_resp.status(), StatusCode::OK);
        let f0 = read_json_body(f0_resp).await;
        let p0 = &f0["parts"][0];
        let s0_verts = p0["surface_positions_m"].as_array().expect("surface_positions_m");
        let s0_ref = p0["surface_reference_positions_m"]
            .as_array()
            .expect("surface_reference_positions_m");
        let s0_idx = p0["surface_indices"].as_array().expect("surface_indices");
        let s0_nidx = p0["surface_node_indices"]
            .as_array()
            .expect("surface_node_indices");
        assert!(!s0_verts.is_empty());
        assert_eq!(s0_verts.len(), s0_ref.len());
        assert_eq!(s0_verts.len(), s0_nidx.len());
        assert_eq!(s0_verts.len() as u64, p0["surface_vertex_count"].as_u64().unwrap());
        assert_eq!(
            (s0_idx.len() / 3) as u64,
            p0["surface_triangle_count"].as_u64().unwrap()
        );
        assert!(s0_idx.len() >= 12 * 3 && s0_idx.len() % 3 == 0);
        for v in s0_idx {
            assert!((v.as_u64().unwrap() as usize) < s0_verts.len());
        }

        // 2. Verify last frame reflects erosion/fracture topology change and carries real fragment meshes
        let last_idx = frame_count - 1;
        let fl_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri(format!(
                        "/api/coupled-sim/frames/{sim_id}/{last_idx}?include_mesh_fields=true&length_unit=m"
                    ))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .expect("last frame response");
        assert_eq!(fl_resp.status(), StatusCode::OK);
        let fl = read_json_body(fl_resp).await;
        let pl = &fl["parts"][0];
        let eroded_count = pl["eroded_element_count"].as_u64().unwrap();
        assert!(
            eroded_count > 0,
            "Expected brittle glass bar under 45 kN tension to erode elements, got {eroded_count}"
        );
        assert_ne!(
            pl["surface_indices"], p0["surface_indices"],
            "Active FEM exterior surface topology must update when elements erode or detach"
        );

        // 3. Verify GET /api/coupled-sim/fragments/:sim_id endpoint transfers cached fragment meshes once
        let frags_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri(format!("/api/coupled-sim/fragments/{sim_id}?length_unit=m"))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .expect("fragments endpoint");
        assert_eq!(frags_resp.status(), StatusCode::OK);
        let frags_json = read_json_body(frags_resp).await;
        assert_eq!(frags_json["simulation_id"], sim_id);
        let cached_frags = frags_json["fragments"].as_array().expect("fragments array");
        assert_eq!(
            cached_frags.len() as u64,
            bake_json["fragment_meshes_count"].as_u64().unwrap()
        );

        let frame_frags = fl["detached_fragments"].as_array().expect("detached_fragments");
        assert!(
            !frame_frags.is_empty(),
            "Expected brittle glass bar under overload to detach at least 1 real FEM fragment (eroded_count={eroded_count}, cached_frags={})",
            cached_frags.len()
        );
        assert!(!cached_frags.is_empty());
        let fm = &frame_frags[0]["mesh"];
        assert!(
            fm.is_object(),
            "Inline fragment mesh must be present by default when include_mesh_fields=true"
        );
        let fv = fm["vertices_m"].as_array().expect("fragment vertices_m");
        let fnm = fm["normals"].as_array().expect("fragment normals");
        let fi = fm["indices"].as_array().expect("fragment indices");
        assert!(fv.len() >= 4, "Fragment must have >= 4 real FEM vertices");
        assert_eq!(fv.len(), fnm.len());
        assert!(
            fi.len() >= 12 && fi.len() % 3 == 0,
            "Fragment must have >= 4 real triangles"
        );
        assert_eq!(
            fm["fragment_id"],
            cached_frags[0]["fragment_id"],
            "Inline fragment mesh must match /api/coupled-sim/fragments/:sim_id"
        );

        // Verify include_fragment_meshes=false strips inline mesh while keeping active surface
        let no_fm_resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri(format!(
                        "/api/coupled-sim/frames/{sim_id}/{last_idx}?include_mesh_fields=true&include_fragment_meshes=false"
                    ))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .expect("frame without inline fragment meshes");
        assert_eq!(no_fm_resp.status(), StatusCode::OK);
        let no_fm = read_json_body(no_fm_resp).await;
        assert!(no_fm["parts"][0]["surface_positions_m"].is_array());
        assert!(no_fm["detached_fragments"][0].get("mesh").is_none());

        // 4. Verify include_mesh_fields=false omits heavy surface arrays but retains scalar counts
        let light_resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri(format!(
                        "/api/coupled-sim/frames/{sim_id}/{last_idx}?include_mesh_fields=false"
                    ))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .expect("lightweight frame");
        assert_eq!(light_resp.status(), StatusCode::OK);
        let light = read_json_body(light_resp).await;
        assert!(light["parts"][0].get("surface_positions_m").is_none());
        assert!(light["parts"][0].get("surface_indices").is_none());
        assert!(light["parts"][0]["surface_vertex_count"].is_u64());
        assert!(light["parts"][0]["surface_triangle_count"].is_u64());
    }
}
