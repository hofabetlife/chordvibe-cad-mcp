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

    /// Run in standalone GUI mode with local SSE server and interactive 3D viewport.
    #[arg(long, default_value_t = false)]
    standalone: bool,

    /// Transport mode: "stdio" or "sse".
    #[arg(short, long, default_value = "stdio")]
    mode: String,

    /// Path to an STL file to open and display upon launch.
    #[arg(long)]
    stl: Option<std::path::PathBuf>,

    /// Positional path to an STL file to open and display upon launch.
    #[arg(value_name = "STL_FILE")]
    file: Option<std::path::PathBuf>,
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

    let is_standalone = args.standalone;
    let is_gui = args.gui || args.standalone || args.stl.is_some() || args.file.is_some();

    if is_gui {
        tracing::info!("Starting MCP-CAD in GUI mode with native 3D viewport...");
        let event_loop = mcp_cad_viewer::create_gui_event_loop()?;
        let proxy = event_loop.create_proxy();

        let engine = EngineState::new()?
            .with_gui_proxy(proxy.clone())
            .into_shared();

        let proxy_clone = proxy.clone();
        let sse_port = args.port.unwrap_or(8080);
        let mode_lower = args.mode.to_lowercase();
        let is_sse_mode = mode_lower == "sse" || args.port.is_some() || is_standalone;
        let run_stdio_transport = !is_sse_mode;

        let engine_for_bg = engine.clone();
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
                // In standalone / GUI mode, always run the background SSE server
                // so agents or tools can connect to http://127.0.0.1:{sse_port}
                let engine_sse = engine_for_bg.clone();
                let sse_task = tokio::spawn(async move {
                    if let Err(e) = run_sse(engine_sse, sse_port).await {
                        tracing::error!("MCP-CAD SSE server terminated with error: {e}");
                    }
                });

                // If stdio transport is requested concurrently:
                if run_stdio_transport {
                    let engine_stdio = engine_for_bg.clone();
                    if let Err(e) = run_stdio(engine_stdio).await {
                        tracing::debug!("MCP stdio transport error: {e}");
                    } else {
                        tracing::debug!("MCP stdio stream reached EOF");
                    }
                    // In standalone, SSE, or manual GUI mode, DO NOT close on stdin EOF!
                }

                // Keep SSE server running in background for the lifetime of GUI window
                let _ = sse_task.await;
            });
        });

        let mut app = mcp_cad_viewer::CadWindowApp {
            exit_on_close: true,
            ..Default::default()
        };

        let stl_to_load = args.stl.as_ref().or(args.file.as_ref());
        if let Some(stl_path) = stl_to_load {
            match mcp_cad_core::import_stl_file(stl_path) {
                Ok(mesh) => {
                    if let Some(fname) = stl_path.file_name() {
                        app.title = format!("MCP-CAD Preview - {}", fname.to_string_lossy());
                    }
                    app.mesh = Some(mesh);
                    tracing::info!("Loaded startup STL file: {:?}", stl_path);
                }
                Err(e) => {
                    tracing::error!("Failed to load startup STL {:?}: {e}", stl_path);
                }
            }
        }

        let _ = mcp_cad_viewer::run_gui_app(event_loop, app);
        std::process::exit(0);
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
