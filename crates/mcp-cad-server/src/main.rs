//! MCP-CAD Server CLI and entry point.

use clap::Parser;
use mcp_cad_server::sse::run_sse;
use mcp_cad_server::state::EngineState;
use mcp_cad_server::stdio::run_stdio;

/// Command-line arguments for the MCP-CAD server.
#[derive(Parser, Debug)]
#[command(
    name = "mcp-cad-server",
    version = "0.1.0",
    about = "MCP-CAD Engine: Autonomous 3D CAD/CAE Model Context Protocol Server"
)]
struct Args {
    /// Port for HTTP/SSE server mode. If set, runs SSE server instead of stdio.
    #[arg(short, long)]
    port: Option<u16>,

    /// Run in headless mode (no GUI window, offscreen rendering only).
    #[arg(long, default_value_t = true)]
    headless: bool,

    /// Open native Windows 3D viewer GUI window.
    #[arg(long, default_value_t = false)]
    gui: bool,

    /// Transport mode: "stdio" or "sse".
    #[arg(short, long, default_value = "stdio")]
    mode: String,
}

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // CRITICAL MCP stdio rule:
    // All logging must route strictly to stderr! stdout is reserved exclusively for JSON-RPC messages.
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    let args = Args::parse();

    tracing::info!("Initializing MCP-CAD Engine...");

    if args.gui {
        tracing::info!("Starting MCP-CAD in GUI mode with native 3D viewport...");
        let event_loop = mcp_cad_viewer::create_gui_event_loop()?;
        let proxy = event_loop.create_proxy();

        let engine = EngineState::new()?
            .with_gui_proxy(proxy.clone())
            .into_shared();

        let proxy_clone = proxy.clone();
        let port = args.port;
        let mode = args.mode;

        std::thread::spawn(move || {
            let rt = match tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
            {
                Ok(r) => r,
                Err(e) => {
                    tracing::error!("Failed to create Tokio runtime: {e}");
                    let _ = proxy_clone.send_event(mcp_cad_viewer::GuiEvent::Close);
                    return;
                }
            };

            rt.block_on(async move {
                let res = if let Some(p) = port {
                    run_sse(engine, p).await
                } else if mode.to_lowercase() == "sse" {
                    run_sse(engine, 8080).await
                } else {
                    run_stdio(engine).await
                };

                if let Err(e) = res {
                    tracing::error!("MCP server terminated with error: {e}");
                }
                let _ = proxy_clone.send_event(mcp_cad_viewer::GuiEvent::Close);
            });
        });

        let app = mcp_cad_viewer::CadWindowApp::default();
        mcp_cad_viewer::run_gui_app(event_loop, app)?;
    } else {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()?;

        rt.block_on(async {
            let engine = EngineState::new()?.into_shared();
            if let Some(port) = args.port {
                run_sse(engine, port).await?;
            } else if args.mode.to_lowercase() == "sse" {
                run_sse(engine, 8080).await?;
            } else {
                run_stdio(engine).await?;
            }
            Ok::<(), Box<dyn std::error::Error + Send + Sync>>(())
        })?;
    }

    Ok(())
}
