import express from 'express';
import { createServer as createViteServer } from 'vite';
import { spawn, execFile, type ChildProcess } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { promisify } from 'node:util';
import {
  LiveSyncStore,
  isSafeLivePartId,
  type LiveCoupledSimProgress,
  type PushLiveSnapshotPayload,
} from './src/server/live_sync_store.ts';
import type { LiveSyncApiResponse } from './src/cad/live_sync_adapter.ts';

const execFileAsync = promisify(execFile);

const PORT = 3000;
const RUST_MCP_PORT = 8080;
const RUST_MCP_URL = `http://127.0.0.1:${RUST_MCP_PORT}/message`;
const WORKSPACE_ROOT = process.cwd();
const CARGO_TARGET_DIR = path.join(WORKSPACE_ROOT, 'target');
const RELEASE_BINARY_PATH = path.join(CARGO_TARGET_DIR, 'release', 'mcp-cad-server');
const DEBUG_BINARY_PATH = path.join(CARGO_TARGET_DIR, 'debug', 'mcp-cad-server');
const CARGO_BIN = process.env.CARGO_BIN || '/root/.cargo/bin/cargo';

function resolveRustBinaryPath(): string {
  if (fs.existsSync(RELEASE_BINARY_PATH)) return RELEASE_BINARY_PATH;
  if (fs.existsSync(DEBUG_BINARY_PATH)) return DEBUG_BINARY_PATH;
  return RELEASE_BINARY_PATH;
}

let rustProcess: ChildProcess | null = null;
let rustReady = false;
let rustLogs: string[] = [];
let bootPromise: Promise<boolean> | null = null;

function pushLog(line: string) {
  const trimmed = line.trim();
  if (!trimmed) return;
  rustLogs.push(`[${new Date().toISOString().slice(11, 19)}] ${trimmed}`);
  if (rustLogs.length > 150) {
    rustLogs = rustLogs.slice(-150);
  }
}

async function isRustServerHealthy(): Promise<boolean> {
  try {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 2500);
    const res = await fetch(RUST_MCP_URL, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ jsonrpc: '2.0', id: 1, method: 'ping' }),
      signal: controller.signal,
    });
    clearTimeout(timeout);
    if (res.ok) {
      rustReady = true;
    }
    return res.ok;
  } catch {
    return false;
  }
}

async function ensureRustMcpServer(): Promise<boolean> {
  if (rustReady && rustProcess && !rustProcess.killed && rustProcess.exitCode === null) {
    return true;
  }

  if (await isRustServerHealthy()) {
    return true;
  }

  if (bootPromise) {
    return bootPromise;
  }

  bootPromise = (async () => {
    try {
      let binPath = resolveRustBinaryPath();
      if (!fs.existsSync(binPath)) {
        if (!fs.existsSync(CARGO_BIN)) {
          pushLog(`Rust toolchain not yet installed at ${CARGO_BIN}`);
          return false;
        }
        pushLog(`Building release mcp-cad-server into ${CARGO_TARGET_DIR}...`);
        try {
          await execFileAsync(CARGO_BIN, ['build', '--release', '--bin', 'mcp-cad-server'], {
            cwd: WORKSPACE_ROOT,
            env: {
              ...process.env,
              CARGO_TARGET_DIR,
              XDG_RUNTIME_DIR: '/tmp',
              PATH: `/root/.cargo/bin:${process.env.PATH || ''}`,
            },
          });
          binPath = resolveRustBinaryPath();
        } catch (err) {
          pushLog(`Cargo release build failed: ${String(err)}`);
          return false;
        }
      }

      if (!rustProcess || rustProcess.killed || rustProcess.exitCode !== null) {
        rustReady = false;
        pushLog(`Starting ${binPath} --mode sse --port ${RUST_MCP_PORT}`);
        rustProcess = spawn(binPath, ['--mode', 'sse', '--port', String(RUST_MCP_PORT)], {
          cwd: WORKSPACE_ROOT,
          env: {
            ...process.env,
            CARGO_TARGET_DIR,
            XDG_RUNTIME_DIR: '/tmp',
            RUST_LOG: 'info',
          },
          stdio: ['ignore', 'pipe', 'pipe'],
        });

        rustProcess.stdout?.on('data', (chunk: Buffer) => pushLog(chunk.toString()));
        rustProcess.stderr?.on('data', (chunk: Buffer) => pushLog(chunk.toString()));
        rustProcess.on('exit', (code) => {
          pushLog(`mcp-cad-server exited with code ${code}`);
          rustProcess = null;
          rustReady = false;
        });
      }

      for (let i = 0; i < 80; i++) {
        if (await isRustServerHealthy()) {
          rustReady = true;
          return true;
        }
        await new Promise((r) => setTimeout(r, 200));
      }

      return isRustServerHealthy();
    } finally {
      bootPromise = null;
    }
  })();

  return bootPromise;
}

async function callRustMcp(
  method: string,
  params?: Record<string, unknown>,
  id: number | string = Date.now()
) {
  const healthy = await ensureRustMcpServer();
  if (!healthy) {
    throw new Error('Rust mcp-cad-server is not responding on port 8080');
  }
  const response = await fetch(RUST_MCP_URL, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      jsonrpc: '2.0',
      id,
      method,
      params,
    }),
  });
  return (await response.json()) as Record<string, unknown>;
}

async function startServer() {
  const app = express();
  app.use(express.json({ limit: '50mb' }));

  // 0. Cross-Process Live MCP Sync Store (bridges stdio/sse MCP agents <-> Web CAD Studio)
  const liveSyncStore = new LiveSyncStore({
    workspaceRoot: WORKSPACE_ROOT,
  });
  liveSyncStore.startWatching();

  const liveSseClients = new Set<express.Response>();
  const writeSseEvent = (
    res: express.Response,
    eventName: string,
    payload: unknown
  ) => {
    if (res.writableEnded) return;
    try {
      res.write(`event: ${eventName}\ndata: ${JSON.stringify(payload)}\n\n`);
    } catch {
      liveSseClients.delete(res);
    }
  };

  const broadcastLiveEvent = (eventName: string, payload: unknown) => {
    for (const client of Array.from(liveSseClients)) {
      writeSseEvent(client, eventName, payload);
    }
  };

  liveSyncStore.on('update', (payload: LiveSyncApiResponse) => {
    broadcastLiveEvent('scene_updated', payload);
  });
  liveSyncStore.on('clear', (payload: LiveSyncApiResponse) => {
    broadcastLiveEvent('scene_cleared', payload);
  });
  liveSyncStore.on('coupled_progress', (progress: LiveCoupledSimProgress) => {
    broadcastLiveEvent('coupled_sim_progress', progress);
  });
  liveSyncStore.on('coupled_frames', (payload: unknown) => {
    broadcastLiveEvent('coupled_sim_frames', payload);
  });

  const heartbeatTimer = setInterval(() => {
    if (liveSseClients.size > 0) {
      liveSyncStore.writeStudioHeartbeat(liveSseClients.size);
    }
  }, 5000);
  heartbeatTimer.unref?.();

  // 0a. Query current live MCP agent scene (disk bus -> legacy _mcp_preview_live.stl -> local Rust SSE fallback)
  app.get('/api/live-sync/scene', async (_req, res) => {
    const localScene = liveSyncStore.readCurrentScene();
    if (localScene.active && localScene.snapshot) {
      res.json(localScene);
      return;
    }

    // Optional fallback: if local Rust SSE daemon is running on :8080 and has in-memory parts
    if (await isRustServerHealthy()) {
      try {
        const upstream = await fetch(`http://127.0.0.1:${RUST_MCP_PORT}/api/live/scene`);
        if (upstream.ok) {
          const snapshot = await upstream.json();
          if (snapshot && Array.isArray(snapshot.parts) && snapshot.parts.length > 0) {
            res.json({
              active: true,
              sourceKind: 'rust_http',
              snapshot,
            } satisfies LiveSyncApiResponse);
            return;
          }
        }
      } catch {
        // Ignore upstream errors
      }
    }

    res.json(localScene);
  });

  // 0b. Stream binary STL for a part in the active live MCP agent scene
  app.get('/api/live-sync/stl/:partId', async (req, res) => {
    const { partId } = req.params;
    if (!isSafeLivePartId(partId)) {
      res.status(400).json({ error: 'Invalid part_id format' });
      return;
    }

    const diskStl = liveSyncStore.readPartStlBuffer(partId);
    if (diskStl) {
      res.setHeader('Content-Type', 'application/octet-stream');
      res.setHeader('Cache-Control', 'no-store');
      res.send(diskStl);
      return;
    }

    // Optional fallback to local Rust SSE daemon live STL endpoint
    if (await isRustServerHealthy()) {
      try {
        const upstream = await fetch(
          `http://127.0.0.1:${RUST_MCP_PORT}/api/live/part/${encodeURIComponent(partId)}`
        );
        if (upstream.ok) {
          const buf = Buffer.from(await upstream.arrayBuffer());
          res.setHeader('Content-Type', 'application/octet-stream');
          res.setHeader('Cache-Control', 'no-store');
          res.send(buf);
          return;
        }
      } catch {
        // Ignore upstream errors
      }
    }

    res.status(404).json({ error: `Live STL not found for part '${partId}'` });
  });

  // 0b2. Serve live coupled HHT-α simulation frames exported alongside scene.json
  app.get('/api/live-sync/coupled-sim', (_req, res) => {
    const coupled = liveSyncStore.readCoupledSimJson();
    if (!coupled) {
      res.status(404).json({ error: 'No live coupled simulation payload available' });
      return;
    }
    res.setHeader('Cache-Control', 'no-store');
    res.json(coupled);
  });

  // 0b3. Serve & push live coupled FEM solve progress + incremental frames
  app.get('/api/live-sync/coupled-progress', (_req, res) => {
    const progress = liveSyncStore.readCoupledSimProgress();
    res.setHeader('Cache-Control', 'no-store');
    res.json({ active: Boolean(progress), progress });
  });

  app.post('/api/live-sync/coupled-progress', (req, res) => {
    const body = req.body as
      | { progress?: LiveCoupledSimProgress; coupled_sim_payload?: unknown }
      | undefined;
    if (!body?.progress || typeof body.progress.job_id !== 'string') {
      res.status(400).json({ error: 'Payload must contain progress with job_id' });
      return;
    }
    try {
      const saved = liveSyncStore.pushCoupledSimProgress(
        body.progress,
        body.coupled_sim_payload
      );
      res.json({ ok: true, progress: saved });
    } catch (err) {
      res.status(500).json({ error: String(err) });
    }
  });

  // 0c. Server-Sent Events stream pushing real-time MCP agent scene updates to Web Studio
  app.get('/api/live-sync/events', (req, res) => {
    res.setHeader('Content-Type', 'text/event-stream');
    res.setHeader('Cache-Control', 'no-cache, no-transform');
    res.setHeader('Connection', 'keep-alive');
    res.flushHeaders?.();

    liveSseClients.add(res);
    liveSyncStore.writeStudioHeartbeat(liveSseClients.size);

    const initial = liveSyncStore.readCurrentScene();
    writeSseEvent(res, 'ready', initial);
    const initialProgress = liveSyncStore.readCoupledSimProgress();
    if (initialProgress) {
      writeSseEvent(res, 'coupled_sim_progress', initialProgress);
    }

    const keepalive = setInterval(() => {
      if (!res.writableEnded) {
        res.write(': keepalive\n\n');
      }
    }, 15_000);

    req.on('close', () => {
      clearInterval(keepalive);
      liveSseClients.delete(res);
      liveSyncStore.writeStudioHeartbeat(liveSseClients.size);
    });
  });

  // 0d. Push a live snapshot over HTTP (useful for external bridges, scripts, and E2E verification)
  app.post('/api/live-sync/push', (req, res) => {
    const body = req.body as PushLiveSnapshotPayload | undefined;
    if (!body || !Array.isArray(body.parts)) {
      res.status(400).json({ error: 'Payload must contain a parts array' });
      return;
    }
    try {
      const snapshot = liveSyncStore.pushSnapshot(body);
      res.json({ ok: true, snapshot });
    } catch (err) {
      res.status(500).json({ error: String(err) });
    }
  });

  // 0e. Clear active live agent snapshot from disk bus
  app.post('/api/live-sync/clear', (req, res) => {
    try {
      liveSyncStore.clearSnapshot(Boolean(req.body?.clearLegacyStl));
      res.json({ ok: true });
    } catch (err) {
      res.status(500).json({ error: String(err) });
    }
  });

  // 1. Health & Rust Engine Status
  app.get('/api/rust/status', async (_req, res) => {
    const healthy = await ensureRustMcpServer();
    res.json({
      healthy,
      rustPort: RUST_MCP_PORT,
      binaryExists: fs.existsSync(resolveRustBinaryPath()),
      cargoAvailable: fs.existsSync(CARGO_BIN),
      coupledSimProxy: true,
      logs: rustLogs.slice(-40),
    });
  });

  // 2. Forward JSON-RPC 2.0 requests directly to Rust mcp-cad-server
  app.post('/api/mcp', async (req, res) => {
    try {
      const healthy = await ensureRustMcpServer();
      if (!healthy) {
        res.status(503).json({
          jsonrpc: '2.0',
          id: req.body?.id ?? null,
          error: { code: -32000, message: 'Rust mcp-cad-server is unavailable' },
        });
        return;
      }

      const upstream = await fetch(RUST_MCP_URL, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(req.body),
      });
      const json = await upstream.json();
      res.status(upstream.status).json(json);
    } catch (err) {
      res.status(500).json({
        jsonrpc: '2.0',
        id: req.body?.id ?? null,
        error: { code: -32603, message: String(err) },
      });
    }
  });

  // 2b. Coupled Implicit HHT-α Dynamic FEM Simulation Proxy (/api/coupled-sim/*)
  // All upstream requests strictly target the local hardcoded Rust daemon on 127.0.0.1:8080.
  const RUST_COUPLED_BASE = `http://127.0.0.1:${RUST_MCP_PORT}/api/coupled-sim`;
  const SAFE_ID_RE = /^[a-zA-Z0-9][a-zA-Z0-9._-]{0,127}$/;
  const isSafeId = (val: string): boolean => SAFE_ID_RE.test(val) && !val.includes('..');
  const SAFE_FRAME_RE = /^\d{1,6}$/;
  const ALLOWED_UNITS = new Set(['m', 'mm']);

  const buildAllowlistedFrameQuery = (query: Record<string, unknown>): string => {
    const sp = new URLSearchParams();
    if (typeof query.part_id === 'string' && isSafeId(query.part_id)) {
      sp.set('part_id', query.part_id);
    }
    if (query.include_mesh_fields === 'true' || query.include_mesh_fields === 'false') {
      sp.set('include_mesh_fields', query.include_mesh_fields);
    }
    if (
      query.include_fragment_meshes === 'true' ||
      query.include_fragment_meshes === 'false'
    ) {
      sp.set('include_fragment_meshes', query.include_fragment_meshes);
    }
    if (typeof query.length_unit === 'string' && ALLOWED_UNITS.has(query.length_unit)) {
      sp.set('length_unit', query.length_unit);
    }
    const qs = sp.toString();
    return qs ? `?${qs}` : '';
  };

  const forwardCoupledJson = async (
    upstreamPath: string,
    method: 'GET' | 'POST',
    body: unknown,
    res: express.Response
  ) => {
    try {
      const healthy = await ensureRustMcpServer();
      if (!healthy) {
        res.status(503).json({
          error: 'Rust mcp-cad-server is unavailable on port 8080',
        });
        return;
      }
      const upstream = await fetch(`${RUST_COUPLED_BASE}${upstreamPath}`, {
        method,
        headers: method === 'POST' ? { 'Content-Type': 'application/json' } : undefined,
        body: method === 'POST' ? JSON.stringify(body ?? {}) : undefined,
      });
      const text = await upstream.text();
      res.status(upstream.status);
      res.setHeader(
        'Content-Type',
        upstream.headers.get('content-type') || 'application/json'
      );
      res.send(text);
    } catch (err) {
      res.status(502).json({ error: `Rust coupled-sim proxy error: ${String(err)}` });
    }
  };

  app.post('/api/coupled-sim/bake', async (req, res) => {
    await forwardCoupledJson('/bake', 'POST', req.body, res);
  });

  app.get('/api/coupled-sim/status', async (_req, res) => {
    await forwardCoupledJson('/status', 'GET', undefined, res);
  });

  app.get('/api/coupled-sim/status/:jobId', async (req, res) => {
    const { jobId } = req.params;
    if (!isSafeId(jobId)) {
      res.status(400).json({ error: 'Invalid job_id format' });
      return;
    }
    await forwardCoupledJson(`/status/${encodeURIComponent(jobId)}`, 'GET', undefined, res);
  });

  app.post('/api/coupled-sim/cancel/:jobId', async (req, res) => {
    const { jobId } = req.params;
    if (!isSafeId(jobId)) {
      res.status(400).json({ error: 'Invalid job_id format' });
      return;
    }
    await forwardCoupledJson(`/cancel/${encodeURIComponent(jobId)}`, 'POST', {}, res);
  });

  app.get('/api/coupled-sim/fragments', async (req, res) => {
    const unit =
      typeof req.query.length_unit === 'string' && ALLOWED_UNITS.has(req.query.length_unit)
        ? `?length_unit=${req.query.length_unit}`
        : '';
    await forwardCoupledJson(`/fragments${unit}`, 'GET', undefined, res);
  });

  app.get('/api/coupled-sim/fragments/:simId', async (req, res) => {
    const { simId } = req.params;
    if (!isSafeId(simId)) {
      res.status(400).json({ error: 'Invalid sim_id format' });
      return;
    }
    const unit =
      typeof req.query.length_unit === 'string' && ALLOWED_UNITS.has(req.query.length_unit)
        ? `?length_unit=${req.query.length_unit}`
        : '';
    await forwardCoupledJson(
      `/fragments/${encodeURIComponent(simId)}${unit}`,
      'GET',
      undefined,
      res
    );
  });

  app.get('/api/coupled-sim/frames/:frameIndex', async (req, res) => {
    const { frameIndex } = req.params;
    if (!SAFE_FRAME_RE.test(frameIndex)) {
      res.status(400).json({ error: 'Invalid frame_index format' });
      return;
    }
    const qs = buildAllowlistedFrameQuery(req.query as Record<string, unknown>);
    await forwardCoupledJson(`/frames/${frameIndex}${qs}`, 'GET', undefined, res);
  });

  app.get('/api/coupled-sim/frames/:simId/:frameIndex', async (req, res) => {
    const { simId, frameIndex } = req.params;
    if (!isSafeId(simId) || !SAFE_FRAME_RE.test(frameIndex)) {
      res.status(400).json({ error: 'Invalid sim_id or frame_index format' });
      return;
    }
    const qs = buildAllowlistedFrameQuery(req.query as Record<string, unknown>);
    await forwardCoupledJson(
      `/frames/${encodeURIComponent(simId)}/${frameIndex}${qs}`,
      'GET',
      undefined,
      res
    );
  });

  app.get('/api/coupled-sim/events', async (req, res) => {
    const healthy = await ensureRustMcpServer();
    if (!healthy) {
      res.status(503).json({
        error: 'Rust mcp-cad-server is unavailable on port 8080',
      });
      return;
    }

    const abortController = new AbortController();
    req.on('close', () => {
      abortController.abort();
    });

    try {
      const upstream = await fetch(`${RUST_COUPLED_BASE}/events`, {
        method: 'GET',
        headers: { Accept: 'text/event-stream' },
        signal: abortController.signal,
      });

      if (!upstream.ok || !upstream.body) {
        res.status(upstream.status || 502).json({
          error: `Upstream SSE error: HTTP ${upstream.status}`,
        });
        return;
      }

      res.setHeader('Content-Type', 'text/event-stream');
      res.setHeader('Cache-Control', 'no-cache, no-transform');
      res.setHeader('Connection', 'keep-alive');
      res.flushHeaders?.();

      const reader = upstream.body.getReader();
      try {
        while (true) {
          const { done, value } = await reader.read();
          if (done) break;
          if (value && value.byteLength > 0) {
            res.write(Buffer.from(value));
          }
        }
      } finally {
        reader.releaseLock();
        if (!res.writableEnded) {
          res.end();
        }
      }
    } catch (err) {
      if (!abortController.signal.aborted && !res.headersSent) {
        res.status(502).json({ error: `SSE proxy error: ${String(err)}` });
      } else if (!res.writableEnded) {
        res.end();
      }
    }
  });

  // 3. Export & stream binary STL buffer for any part directly from Rust engine
  app.get('/api/stl/:partId', async (req, res) => {
    const { partId } = req.params;
    const safeId = partId.replace(/[^a-zA-Z0-9_-]/g, '_');
    const tmpStlPath = `/tmp/web_studio_${safeId}.stl`;

    try {
      const rpcRes = await callRustMcp('tools/call', {
        name: 'export_stl',
        arguments: {
          part_id: partId,
          filepath: tmpStlPath,
          binary: true,
        },
      });

      const result = rpcRes.result as
        | { isError?: boolean; content?: Array<{ text?: string }> }
        | undefined;
      if (result?.isError || !fs.existsSync(tmpStlPath)) {
        const msg = result?.content?.[0]?.text || 'Failed to export STL from Rust engine';
        res.status(400).json({ error: msg });
        return;
      }

      const stlBytes = fs.readFileSync(tmpStlPath);
      res.setHeader('Content-Type', 'application/octet-stream');
      res.setHeader('Content-Disposition', `attachment; filename="${safeId}.stl"`);
      res.send(stlBytes);
    } catch (err) {
      res.status(500).json({ error: String(err) });
    }
  });

  // 4. List pre-built STL files in /output directory
  app.get('/api/output-stls', (_req, res) => {
    const outputDir = path.join(WORKSPACE_ROOT, 'output');
    if (!fs.existsSync(outputDir)) {
      res.json({ files: [] });
      return;
    }
    const files = fs
      .readdirSync(outputDir)
      .filter((f) => f.toLowerCase().endsWith('.stl'))
      .map((name) => {
        const full = path.join(outputDir, name);
        const stat = fs.statSync(full);
        return {
          name,
          filepath: `./output/${name}`,
          sizeBytes: stat.size,
        };
      });
    res.json({ files });
  });

  // 5. Run cargo test on workspace or specific crate
  app.post('/api/rust/test', async (req, res) => {
    const crate = typeof req.body?.crate === 'string' ? req.body.crate : 'workspace';
    const args = crate === 'workspace' ? ['test', '--workspace'] : ['test', '-p', crate];

    const start = Date.now();
    try {
      const { stdout, stderr } = await execFileAsync(CARGO_BIN, args, {
        cwd: WORKSPACE_ROOT,
        env: {
          ...process.env,
          CARGO_TARGET_DIR,
          XDG_RUNTIME_DIR: '/tmp',
          PATH: `/root/.cargo/bin:${process.env.PATH || ''}`,
        },
        maxBuffer: 10 * 1024 * 1024,
      });
      res.json({
        success: true,
        durationMs: Date.now() - start,
        stdout,
        stderr,
      });
    } catch (err: unknown) {
      const e = err as { stdout?: string; stderr?: string; message?: string };
      res.json({
        success: false,
        durationMs: Date.now() - start,
        stdout: e.stdout || '',
        stderr: e.stderr || e.message || String(err),
      });
    }
  });

  // 6. Git repository info & GitHub Push endpoint
  app.get('/api/git/info', async (_req, res) => {
    const hasEnvToken = Boolean(
      process.env.GITHUB_TOKEN &&
        process.env.GITHUB_TOKEN.trim() !== '' &&
        process.env.GITHUB_TOKEN !== 'MY_GITHUB_TOKEN'
    );
    try {
      const branchOut = await execFileAsync('git', ['rev-parse', '--abbrev-ref', 'HEAD'], {
        cwd: WORKSPACE_ROOT,
      });
      const commitOut = await execFileAsync('git', ['log', '-1', '--oneline'], {
        cwd: WORKSPACE_ROOT,
      });
      const statusOut = await execFileAsync('git', ['status', '--short'], {
        cwd: WORKSPACE_ROOT,
      });
      const remoteOut = await execFileAsync('git', ['remote', 'get-url', 'origin'], {
        cwd: WORKSPACE_ROOT,
      });
      res.json({
        branch: branchOut.stdout.trim(),
        lastCommit: commitOut.stdout.trim(),
        status: statusOut.stdout.trim(),
        remote: remoteOut.stdout.trim(),
        hasEnvToken,
      });
    } catch (err) {
      res.json({
        branch: 'master',
        lastCommit: '2fe786b feat(physics,viewer): add gear transmission kinematics',
        status: String(err),
        remote: 'https://github.com/hofabetlife/chordvibe-cad-mcp.git',
        hasEnvToken,
      });
    }
  });

  app.post('/api/git/push', async (req, res) => {
    const {
      token,
      branch = 'master',
      commitMessage = 'chore: update from MCP-CAD Web Studio',
      includeWebStudio = false,
    } = req.body || {};

    const effectiveToken =
      (typeof token === 'string' && token.trim()) ||
      (process.env.GITHUB_TOKEN && process.env.GITHUB_TOKEN !== 'MY_GITHUB_TOKEN'
        ? process.env.GITHUB_TOKEN.trim()
        : '');

    if (!effectiveToken) {
      res.status(400).json({
        ok: false,
        error:
          'Укажите GitHub Personal Access Token (PAT) в поле ниже или добавьте GITHUB_TOKEN в Secrets панели AI Studio.',
      });
      return;
    }

    try {
      await execFileAsync('git', ['config', 'user.name', 'hofabetlife'], { cwd: WORKSPACE_ROOT });
      await execFileAsync('git', ['config', 'user.email', 'hofabetlife@users.noreply.github.com'], {
        cwd: WORKSPACE_ROOT,
      });

      if (includeWebStudio) {
        await execFileAsync('git', ['add', '-A'], { cwd: WORKSPACE_ROOT });
      } else {
        await execFileAsync(
          'git',
          ['add', 'Cargo.toml', 'Cargo.lock', 'crates', 'examples', 'docs', 'README.md', 'PRD.md'],
          { cwd: WORKSPACE_ROOT }
        );
      }

      const status = await execFileAsync('git', ['status', '--porcelain'], { cwd: WORKSPACE_ROOT });
      if (status.stdout.trim().length > 0) {
        await execFileAsync('git', ['commit', '-m', commitMessage], { cwd: WORKSPACE_ROOT });
      }

      const authRemote = `https://x-access-token:${encodeURIComponent(effectiveToken)}@github.com/hofabetlife/chordvibe-cad-mcp.git`;
      const pushOut = await execFileAsync('git', ['push', authRemote, `HEAD:${branch}`], {
        cwd: WORKSPACE_ROOT,
      });

      res.json({
        ok: true,
        pushOut: `${pushOut.stdout}\n${pushOut.stderr}`.trim() || `Отправлено в origin/${branch}`,
      });
    } catch (err: unknown) {
      const e = err as { stderr?: string; message?: string };
      const sanitizedError = (e.stderr || e.message || String(err)).replace(
        /x-access-token:[^@]+@/g,
        'x-access-token:***@'
      );
      res.status(500).json({ ok: false, error: sanitizedError });
    }
  });

  if (process.env.NODE_ENV !== 'production') {
    const vite = await createViteServer({
      server: { middlewareMode: true },
      appType: 'spa',
    });
    app.use(vite.middlewares);
  } else {
    const distPath = path.join(WORKSPACE_ROOT, 'dist');
    app.use(express.static(distPath));
    app.get('*all', (_req, res) => {
      res.sendFile(path.join(distPath, 'index.html'));
    });
  }

  app.listen(PORT, '0.0.0.0', () => {
    const msg = `MCP-CAD Web Studio listening on http://localhost:${PORT}`;
    pushLog(msg);
    console.log(msg);
    if (fs.existsSync(resolveRustBinaryPath())) {
      void ensureRustMcpServer().catch((err) => {
        pushLog(`Startup error: ${String(err)}`);
      });
    }
  });
}

startServer();
