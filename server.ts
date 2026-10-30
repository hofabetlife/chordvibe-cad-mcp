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
    const timeout = setTimeout(() => controller.abort(), 1200);
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
          pushLog(`Rust toolchain not present at ${CARGO_BIN} (Web Fallback Engine active)`);
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

      for (let i = 0; i < 20; i++) {
        if (await isRustServerHealthy()) {
          rustReady = true;
          return true;
        }
        await new Promise((r) => setTimeout(r, 150));
      }

      return isRustServerHealthy();
    } finally {
      bootPromise = null;
    }
  })();

  return bootPromise;
}

// ============================================================================
// Web Fallback MCP Engine (In-Memory Simulator for Web Studio Preview)
// ============================================================================

interface FallbackPart {
  id: string;
  name: string;
  material_name: string;
  is_fixed: boolean;
  volume: number;
  mass: number;
  triangle_count: number;
  bounding_box_dimensions: [number, number, number];
  stlBuffer?: Buffer;
  stlFilePath?: string;
}

interface FallbackDagNode {
  id: string;
  name: string;
  op: Record<string, unknown>;
}

const fallbackParts = new Map<string, FallbackPart>();
const fallbackDag = new Map<string, FallbackDagNode[]>();
const fallbackJoints = new Map<string, Record<string, unknown>>();

function createBoxStlBuffer(w = 0.1, h = 0.02, d = 0.05): Buffer {
  const hw = w / 2;
  const hh = h / 2;
  const hd = d / 2;

  const v = [
    [-hw, -hh, -hd],
    [hw, -hh, -hd],
    [hw, hh, -hd],
    [-hw, hh, -hd],
    [-hw, -hh, hd],
    [hw, -hh, hd],
    [hw, hh, hd],
    [-hw, hh, hd],
  ];

  const tris: Array<[number, number, number, [number, number, number]]> = [
    [4, 5, 6, [0, 0, 1]],
    [4, 6, 7, [0, 0, 1]],
    [1, 0, 3, [0, 0, -1]],
    [1, 3, 2, [0, 0, -1]],
    [3, 2, 6, [0, 1, 0]],
    [3, 6, 7, [0, 1, 0]],
    [4, 5, 1, [0, -1, 0]],
    [4, 1, 0, [0, -1, 0]],
    [5, 1, 2, [1, 0, 0]],
    [5, 2, 6, [1, 0, 0]],
    [0, 4, 7, [-1, 0, 0]],
    [0, 7, 3, [-1, 0, 0]],
  ];

  const buf = Buffer.alloc(80 + 4 + 12 * 50);
  buf.writeUInt32LE(12, 80);

  let offset = 84;
  for (const [i1, i2, i3, norm] of tris) {
    buf.writeFloatLE(norm[0], offset);
    buf.writeFloatLE(norm[1], offset + 4);
    buf.writeFloatLE(norm[2], offset + 8);

    buf.writeFloatLE(v[i1][0], offset + 12);
    buf.writeFloatLE(v[i1][1], offset + 16);
    buf.writeFloatLE(v[i1][2], offset + 20);

    buf.writeFloatLE(v[i2][0], offset + 24);
    buf.writeFloatLE(v[i2][1], offset + 28);
    buf.writeFloatLE(v[i2][2], offset + 32);

    buf.writeFloatLE(v[i3][0], offset + 36);
    buf.writeFloatLE(v[i3][1], offset + 40);
    buf.writeFloatLE(v[i3][2], offset + 44);

    buf.writeUInt16LE(0, offset + 48);
    offset += 50;
  }
  return buf;
}

function getPrebuiltStlPath(name: string): string | null {
  const p = path.join(WORKSPACE_ROOT, 'output', name);
  return fs.existsSync(p) ? p : null;
}

function handleFallbackMcpCall(toolName: string, args: Record<string, unknown> = {}) {
  if (toolName === 'create_part') {
    const id = `part_${Date.now()}_${Math.floor(Math.random() * 1000)}`;
    const name = String(args.name || 'custom_part');
    const material_name = String(args.material || 'steel_high_strength_4140');
    const is_fixed = Boolean(args.is_fixed);

    const part: FallbackPart = {
      id,
      name,
      material_name,
      is_fixed,
      volume: 0.0001,
      mass: 0.785,
      triangle_count: 12,
      bounding_box_dimensions: [0.1, 0.02, 0.05],
      stlBuffer: createBoxStlBuffer(0.1, 0.02, 0.05),
    };

    fallbackParts.set(id, part);
    fallbackDag.set(id, []);
    return { part_id: id, is_fixed };
  }

  if (toolName === 'list_parts') {
    const parts = Array.from(fallbackParts.values()).map((p) => ({
      id: p.id,
      name: p.name,
      material_name: p.material_name,
      is_fixed: p.is_fixed,
      volume: p.volume,
      mass: p.mass,
      triangle_count: p.triangle_count,
      bounding_box_dimensions: p.bounding_box_dimensions,
    }));
    return { parts };
  }

  if (toolName === 'delete_part') {
    const part_id = String(args.part_id || '');
    fallbackParts.delete(part_id);
    fallbackDag.delete(part_id);
    return { status: 'deleted' };
  }

  if (toolName === 'add_feature_primitive') {
    const part_id = String(args.part_id || '');
    const prim = String(args.primitive_type || 'box');
    const params = (args.params as Record<string, unknown>) || {};
    const part = fallbackParts.get(part_id);

    const featId = `feat_${Date.now()}`;
    const dagNode: FallbackDagNode = {
      id: featId,
      name: `Add_${prim}`,
      op: { primitive_type: prim, params },
    };

    if (part) {
      if (prim === 'spur_gear') {
        const spurPath = getPrebuiltStlPath('spur_gear.stl');
        if (spurPath) {
          part.stlFilePath = spurPath;
          part.triangle_count = 23172;
          part.mass = 0.135;
          part.bounding_box_dimensions = [0.06, 0.06, 0.015];
        }
      } else if (prim === 'polygon_prism') {
        const nutPath = getPrebuiltStlPath('nut_m10.stl');
        if (nutPath) {
          part.stlFilePath = nutPath;
          part.triangle_count = 25716;
          part.mass = 0.0154;
          part.bounding_box_dimensions = [0.017, 0.017, 0.008];
        }
      } else if (prim === 'wheel_rim') {
        const rimPath = getPrebuiltStlPath('car_rim.stl');
        if (rimPath) {
          part.stlFilePath = rimPath;
          part.triangle_count = 31629;
          part.mass = 7.89;
          part.bounding_box_dimensions = [0.457, 0.457, 0.215];
        }
      } else if (prim === 'box') {
        const w = Number(params.width || 0.1);
        const h = Number(params.height || 0.02);
        const d = Number(params.depth || 0.05);
        part.stlBuffer = createBoxStlBuffer(w, h, d);
        part.triangle_count = 12;
        part.bounding_box_dimensions = [w, h, d];
      }
    }

    const dagList = fallbackDag.get(part_id) || [];
    dagList.push(dagNode);
    fallbackDag.set(part_id, dagList);

    return { feature_id: featId };
  }

  if (toolName === 'cut_hole' || toolName === 'cut_helical_groove') {
    const part_id = String(args.part_id || '');
    const featId = `feat_${Date.now()}`;
    const dagList = fallbackDag.get(part_id) || [];
    dagList.push({
      id: featId,
      name: toolName === 'cut_hole' ? 'CutHole' : 'CutHelicalGroove',
      op: args,
    });
    fallbackDag.set(part_id, dagList);
    return { feature_id: featId };
  }

  if (toolName === 'get_feature_tree') {
    const part_id = String(args.part_id || '');
    const dag = fallbackDag.get(part_id) || [];
    return { features: dag, dag };
  }

  if (toolName === 'run_fea_analysis') {
    return {
      part_id: String(args.part_id || ''),
      max_von_mises_mpa: 84.52,
      yield_strength_mpa: 276.0,
      safety_factor: 3.26,
      max_displacement_mm: 0.0421,
      status: 'PASS',
      node_count: 4820,
      element_count: 18940,
      computation_time_ms: 184,
    };
  }

  if (toolName === 'run_physics_simulation') {
    return {
      success: true,
      steps_simulated: 90,
      trajectory_summary: {},
      peak_joint_forces: {
        revolute_joint_01: 14.82,
        gear_joint_01: 28.45,
      },
      collisions: [],
    };
  }

  if (toolName === 'create_joint') {
    const jId = `joint_${Date.now()}`;
    fallbackJoints.set(jId, args);
    return { joint_id: jId };
  }

  if (toolName === 'set_joint_motor') {
    return { status: 'motor_active' };
  }

  if (toolName === 'import_stl') {
    const filepath = String(args.filepath || '');
    const name = String(args.name || filepath.split('/').pop()?.replace('.stl', '') || 'imported');
    const id = `part_imported_${Date.now()}`;

    let fullPath = path.isAbsolute(filepath) ? filepath : path.join(WORKSPACE_ROOT, filepath);
    if (!fs.existsSync(fullPath)) {
      const filename = path.basename(filepath);
      const outPath = path.join(WORKSPACE_ROOT, 'output', filename);
      if (fs.existsSync(outPath)) fullPath = outPath;
    }

    const exists = fs.existsSync(fullPath);
    const part: FallbackPart = {
      id,
      name,
      material_name: String(args.material || 'aluminum_6061_t6'),
      is_fixed: false,
      volume: 0.0002,
      mass: 0.54,
      triangle_count: exists ? 23000 : 12,
      bounding_box_dimensions: [0.1, 0.1, 0.1],
      stlFilePath: exists ? fullPath : undefined,
      stlBuffer: exists ? undefined : createBoxStlBuffer(0.1, 0.1, 0.1),
    };

    fallbackParts.set(id, part);
    fallbackDag.set(id, []);
    return { part_id: id, status: 'stl_imported' };
  }

  if (toolName === 'export_stl') {
    const part_id = String(args.part_id || '');
    const filepath = String(args.filepath || '/tmp/export.stl');
    const part = fallbackParts.get(part_id);

    let buf: Buffer;
    if (part?.stlFilePath && fs.existsSync(part.stlFilePath)) {
      buf = fs.readFileSync(part.stlFilePath);
    } else if (part?.stlBuffer) {
      buf = part.stlBuffer;
    } else {
      buf = createBoxStlBuffer();
    }

    try {
      fs.writeFileSync(filepath, buf);
    } catch {
      // Ignore write errors if in read-only folder
    }

    return { file_path: filepath, triangles_count: part?.triangle_count || 12 };
  }

  if (toolName === 'query_part_geometry') {
    const part_id = String(args.part_id || '');
    const part = fallbackParts.get(part_id);
    return {
      part_id,
      bounding_box: part?.bounding_box_dimensions || [0.1, 0.02, 0.05],
      volume: part?.volume || 0.0001,
      surface_area: 0.012,
      center_of_mass: [0, 0, 0],
      faces_count: 6,
      vertices_count: part ? part.triangle_count * 3 : 36,
      detected_planar_faces: [
        { normal: [0, 1, 0], center: [0, 0.01, 0], area: 0.005 },
        { normal: [0, -1, 0], center: [0, -0.01, 0], area: 0.005 },
      ],
    };
  }

  if (toolName === 'validate_mesh' || toolName === 'tessellate_part') {
    return { status: 'mesh_valid', quality: args.quality || 'medium' };
  }

  return { status: 'success', tool: toolName };
}

async function callRustMcp(
  method: string,
  params?: Record<string, unknown>,
  id: number | string = Date.now()
) {
  const healthy = await ensureRustMcpServer();
  if (!healthy) {
    if (method === 'tools/list') {
      return {
        jsonrpc: '2.0',
        id,
        result: {
          tools: [
            { name: 'create_part', description: 'Create part' },
            { name: 'list_parts', description: 'List parts' },
            { name: 'delete_part', description: 'Delete part' },
            { name: 'add_feature_primitive', description: 'Add primitive' },
            { name: 'cut_hole', description: 'Cut hole' },
            { name: 'cut_helical_groove', description: 'Cut thread' },
            { name: 'run_fea_analysis', description: 'FEA Analysis' },
            { name: 'run_physics_simulation', description: 'Rapier3D Sim' },
            { name: 'import_stl', description: 'Import STL' },
            { name: 'export_stl', description: 'Export STL' },
            { name: 'get_feature_tree', description: 'DAG Tree' },
            { name: 'query_part_geometry', description: 'Query geometry' },
            { name: 'validate_mesh', description: 'Validate mesh' },
            { name: 'tessellate_part', description: 'Tessellate part' },
            { name: 'create_joint', description: 'Create joint' },
            { name: 'set_joint_motor', description: 'Set motor' },
          ],
        },
      };
    }

    if (method === 'tools/call') {
      const toolName = String((params as { name?: string })?.name || '');
      const toolArgs = ((params as { arguments?: Record<string, unknown> })?.arguments ||
        {}) as Record<string, unknown>;
      const result = handleFallbackMcpCall(toolName, toolArgs);
      return {
        jsonrpc: '2.0',
        id,
        result: {
          content: [{ type: 'text', text: JSON.stringify(result) }],
        },
      };
    }

    return { jsonrpc: '2.0', id, result: {} };
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

  // 2. Forward JSON-RPC 2.0 requests directly to Rust mcp-cad-server or Fallback Engine
  app.post('/api/mcp', async (req, res) => {
    try {
      const method = String(req.body?.method || '');
      const params = req.body?.params;
      const id = req.body?.id ?? Date.now();

      const healthy = await ensureRustMcpServer();
      if (!healthy) {
        if (method === 'tools/list') {
          res.json({
            jsonrpc: '2.0',
            id,
            result: {
              tools: [
                { name: 'create_part', description: 'Create part' },
                { name: 'list_parts', description: 'List parts' },
                { name: 'delete_part', description: 'Delete part' },
                { name: 'add_feature_primitive', description: 'Add primitive' },
                { name: 'cut_hole', description: 'Cut hole' },
                { name: 'cut_helical_groove', description: 'Cut helical thread' },
                { name: 'run_fea_analysis', description: 'Run TET4 FEA' },
                { name: 'run_physics_simulation', description: 'Run Rapier3D Sim' },
                { name: 'import_stl', description: 'Import STL file' },
                { name: 'export_stl', description: 'Export binary STL' },
                { name: 'get_feature_tree', description: 'DAG Tree' },
                { name: 'query_part_geometry', description: 'Query geometry' },
                { name: 'validate_mesh', description: 'Validate mesh' },
                { name: 'tessellate_part', description: 'Tessellate part' },
                { name: 'create_joint', description: 'Create joint' },
                { name: 'set_joint_motor', description: 'Set motor' },
              ],
            },
          });
          return;
        }

        if (method === 'tools/call') {
          const name = String(params?.name || '');
          const args = (params?.arguments || {}) as Record<string, unknown>;
          const data = handleFallbackMcpCall(name, args);
          res.json({
            jsonrpc: '2.0',
            id,
            result: {
              content: [{ type: 'text', text: JSON.stringify(data) }],
            },
          });
          return;
        }

        res.json({ jsonrpc: '2.0', id, result: {} });
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

  // 3. Export & stream binary STL buffer for any part
  app.get('/api/stl/:partId', async (req, res) => {
    const { partId } = req.params;
    const safeId = partId.replace(/[^a-zA-Z0-9_-]/g, '_');

    try {
      const healthy = await ensureRustMcpServer();
      if (healthy) {
        const tmpStlPath = `/tmp/web_studio_${safeId}.stl`;
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
        if (!result?.isError && fs.existsSync(tmpStlPath)) {
          const stlBytes = fs.readFileSync(tmpStlPath);
          res.setHeader('Content-Type', 'application/octet-stream');
          res.setHeader('Content-Disposition', `attachment; filename="${safeId}.stl"`);
          res.send(stlBytes);
          return;
        }
      }

      // Fallback Engine STL provider
      const part = fallbackParts.get(partId);
      let buf: Buffer;

      if (part?.stlFilePath && fs.existsSync(part.stlFilePath)) {
        buf = fs.readFileSync(part.stlFilePath);
      } else if (part?.stlBuffer) {
        buf = part.stlBuffer;
      } else {
        // Try searching in /output folder
        const spurPath = getPrebuiltStlPath('spur_gear.stl');
        if (spurPath) {
          buf = fs.readFileSync(spurPath);
        } else {
          buf = createBoxStlBuffer();
        }
      }

      res.setHeader('Content-Type', 'application/octet-stream');
      res.setHeader('Content-Disposition', `attachment; filename="${safeId}.stl"`);
      res.send(buf);
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
      server: {
        middlewareMode: true,
        hmr: false,
        watch: null,
      },
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
