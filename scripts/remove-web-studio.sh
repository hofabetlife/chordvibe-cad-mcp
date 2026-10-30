#!/usr/bin/env bash
# ==============================================================================
# MCP-CAD Web Studio — Zero-Trace Uninstaller
# Удаляет только файлы веб-студии, оставляя чистый Rust-репозиторий
# chordvibe-cad-mcp без единого следа.
# ==============================================================================
set -euo pipefail

echo "Удаление MCP-CAD Web Studio из корня проекта..."

rm -rf \
  src \
  dist \
  node_modules \
  .playwright-cli \
  server.ts \
  index.html \
  vite.config.ts \
  tsconfig.json \
  package.json \
  package-lock.json \
  bun.lock \
  metadata.json \
  .env.example \
  scripts/remove-web-studio.sh

rmdir scripts 2>/dev/null || true

echo "Готово! В проекте остались исключительно файлы Rust-репозитория (Cargo.toml, crates/, examples/, docs/, output/)."
