# Multi-stage Dockerfile for headless MCP-CAD Server on Linux with Software GPU (Mesa Lavapipe / llvmpipe)
FROM rust:1.82-bookworm AS builder

WORKDIR /workspace
COPY Cargo.toml Cargo.lock ./
COPY crates ./crates

RUN cargo build --release -p mcp-cad-server

FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    libvulkan1 \
    mesa-vulkan-drivers \
    libegl1 \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY --from=builder /workspace/target/release/mcp-cad-server /usr/local/bin/mcp-cad-server
COPY run_server.sh /app/run_server.sh
RUN chmod +x /app/run_server.sh

ENV RUST_LOG=info
ENV WGPU_BACKEND=vulkan
ENV LIBGL_ALWAYS_SOFTWARE=1

EXPOSE 8080

ENTRYPOINT ["/usr/local/bin/mcp-cad-server"]
CMD ["--mode", "sse", "--port", "8080"]
