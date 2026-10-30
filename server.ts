import express from 'express';
import { createServer as createViteServer } from 'vite';
import { spawn, execFile, type ChildProcess } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { promisify } from 'node:util';

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
  return DEBUG_BINARY_PATH;
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
        pushLog(`Building mcp-cad-server into ${CARGO_TARGET_DIR}...`);
        try {
          await execFileAsync(CARGO_BIN, ['build', '--bin', 'mcp-cad-server'], {
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
          pushLog(`Cargo build failed: ${String(err)}`);
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

  // 1. Health & Rust Engine Status
  app.get('/api/rust/status', async (_req, res) => {
    const healthy = await ensureRustMcpServer();
    res.json({
      healthy,
      rustPort: RUST_MCP_PORT,
      binaryExists: fs.existsSync(resolveRustBinaryPath()),
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
      res.json(json);
    } catch (err) {
      res.status(500).json({
        jsonrpc: '2.0',
        id: req.body?.id ?? null,
        error: { code: -32603, message: String(err) },
      });
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
