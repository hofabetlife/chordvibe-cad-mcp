//! Stdio transport loop for local MCP communication (Claude Desktop, Antigravity, Cursor).

use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

use crate::protocol::handle_message;
use crate::state::SharedState;

/// Runs the standard input/output (stdio) MCP JSON-RPC loop.
///
/// Reads newline-delimited JSON-RPC requests from `stdin` and writes JSON-RPC
/// responses to `stdout` with an immediate flush.
///
/// Note: All tracing and diagnostics must route strictly to `stderr`.
pub async fn run_stdio(state: SharedState) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let stdin = tokio::io::stdin();
    let stdout = tokio::io::stdout();

    let mut reader = BufReader::new(stdin);
    let mut writer = stdout;

    let mut line = String::new();

    while reader.read_line(&mut line).await? > 0 {
        let trimmed = line.trim();
        if !trimmed.is_empty() {
            tracing::debug!("stdio received: {}", trimmed);

            if let Some(resp) = handle_message(trimmed, &state).await {
                let serialized = serde_json::to_string(&resp)?;
                writer.write_all(serialized.as_bytes()).await?;
                writer.write_all(b"\n").await?;
                writer.flush().await?;
            }
        }
        line.clear();
    }

    Ok(())
}
