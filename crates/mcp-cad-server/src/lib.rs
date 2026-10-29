#![recursion_limit = "256"]
//! MCP-CAD Server library: JSON-RPC MCP dispatcher, stdio/SSE transports, and 15 CAD/CAE tools.

pub mod error;
pub mod protocol;
pub mod sse;
pub mod state;
pub mod stdio;
pub mod tools;

pub use error::ServerError;
pub use protocol::{
    handle_message, handle_request, JsonRpcError, JsonRpcRequest, JsonRpcResponse, ToolCallResult,
    ToolContent, ToolDefinition,
};
pub use state::{EngineState, SharedState};
pub use tools::list_tools;
