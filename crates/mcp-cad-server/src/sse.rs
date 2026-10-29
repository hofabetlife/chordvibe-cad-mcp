//! HTTP and Server-Sent Events (SSE) server using Axum.

use std::convert::Infallible;
use std::net::SocketAddr;

use axum::{
    extract::State,
    response::sse::{Event, KeepAlive, Sse},
    routing::{get, post},
    Json, Router,
};
use futures_util::stream::Stream;
use tokio::sync::broadcast;
use tokio_stream::wrappers::BroadcastStream;
use tokio_stream::StreamExt;
use tower_http::cors::CorsLayer;

use crate::protocol::{handle_request, JsonRpcRequest, JsonRpcResponse};
use crate::state::SharedState;

/// Internal state passed to Axum route handlers.
#[derive(Clone)]
pub struct AppState {
    pub engine: SharedState,
    pub tx: broadcast::Sender<String>,
}

/// Handler for the `/sse` endpoint.
///
/// Sends an initial `endpoint` event referencing `/message`, followed by any
/// JSON-RPC responses broadcast across the server.
async fn sse_handler(
    State(app_state): State<AppState>,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    let rx = app_state.tx.subscribe();

    let initial = tokio_stream::once(Ok::<_, Infallible>(
        Event::default().event("endpoint").data("/message"),
    ));

    let stream = BroadcastStream::new(rx).filter_map(|res| match res {
        Ok(msg) => Some(Ok(Event::default().event("message").data(msg))),
        Err(_) => None,
    });

    let combined = initial.chain(stream);

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

/// Creates the Axum router for the MCP HTTP/SSE server.
pub fn build_router(engine: SharedState) -> (Router, broadcast::Sender<String>) {
    let (tx, _) = broadcast::channel(128);
    let app_state = AppState {
        engine,
        tx: tx.clone(),
    };

    let router = Router::new()
        .route("/sse", get(sse_handler))
        .route("/message", post(message_handler))
        .layer(CorsLayer::permissive())
        .with_state(app_state);

    (router, tx)
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
