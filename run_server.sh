#!/usr/bin/env bash
# Cross-platform launcher script for MCP-CAD Server on Linux / macOS
# Configures headless software rendering fallback (Lavapipe / llvmpipe) when no hardware GPU is present.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export RUST_LOG="${RUST_LOG:-info}"
export WGPU_POWER_PREF="${WGPU_POWER_PREF:-low}"

if [[ "$(uname -s)" == "Linux" ]]; then
    export WGPU_BACKEND="${WGPU_BACKEND:-vulkan,gl}"
fi

if [[ -x "./target/release/mcp-cad-server" ]]; then
    exec ./target/release/mcp-cad-server "$@"
else
    exec cargo run --release -p mcp-cad-server -- "$@"
fi
