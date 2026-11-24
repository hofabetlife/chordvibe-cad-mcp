import { EventEmitter } from 'node:events';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import type {
  LiveScenePart,
  LiveSceneSnapshot,
  LiveSyncApiResponse,
  LiveSyncSourceKind,
} from '../cad/live_sync_adapter';
import { sanitizeLivePartId } from '../cad/live_sync_adapter';

const SAFE_PART_ID_RE = /^[a-zA-Z0-9][a-zA-Z0-9._-]{0,127}$/;

export function isSafeLivePartId(partId: string): boolean {
  return SAFE_PART_ID_RE.test(partId) && !partId.includes('..');
}

export interface LiveSyncStoreOptions {
  workspaceRoot: string;
  tempLiveDir?: string;
  legacyStlPath?: string;
  pollIntervalMs?: number;
}

export interface PushLivePartInput extends Partial<LiveScenePart> {
  id: string;
  name?: string;
  stl_base64?: string;
}

export interface LiveCoupledSimProgress {
  job_id: string;
  simulation_id: string;
  state: 'running' | 'cancelling' | 'completed' | 'cancelled' | 'failed' | string;
  accuracy_level: string;
  part_ids: string[];
  total_time_s: number;
  current_time_s: number;
  progress_pct: number;
  step_count: number;
  frame_count: number;
  frames_cached: number;
  latest_frame_index?: number | null;
  current_dt_s: number;
  bisections: number;
  eroded_elements: number;
  detached_fragments: number;
  max_von_mises_mpa: number;
  elapsed_ms: number;
  estimated_remaining_ms?: number | null;
  stage_label: string;
  updated_at_ms: number;
}

export interface PushLiveSnapshotPayload {
  revision?: number;
  timestamp_ms?: number;
  pid?: number;
  source?: string;
  length_unit?: string;
  last_tool?: LiveSceneSnapshot['last_tool'];
  recent_tools?: LiveSceneSnapshot['recent_tools'];
  parts: PushLivePartInput[];
  joints?: LiveSceneSnapshot['joints'];
  physics_simulation?: LiveSceneSnapshot['physics_simulation'];
  coupled_simulation?: LiveSceneSnapshot['coupled_simulation'];
  coupled_sim_payload?: unknown;
}

interface ResolvedSnapshotCandidate {
  sourceKind: LiveSyncSourceKind;
  sourceDir: string;
  sourcePath: string;
  freshnessMs: number;
  signature: string;
  snapshot: LiveSceneSnapshot;
}

function writeAtomicFile(targetPath: string, data: string | Buffer): void {
  const dir = path.dirname(targetPath);
  fs.mkdirSync(dir, { recursive: true });
  const tmpPath = `${targetPath}.${process.pid}.${Date.now()}.tmp`;
  fs.writeFileSync(tmpPath, data);
  fs.renameSync(tmpPath, targetPath);
}

function readJsonSafely<T>(filePath: string): { value: T; mtimeMs: number } | null {
  try {
    if (!fs.existsSync(filePath)) return null;
    const stat = fs.statSync(filePath);
    if (!stat.isFile() || stat.size === 0) return null;
    const raw = fs.readFileSync(filePath, 'utf8');
    const parsed = JSON.parse(raw) as T;
    return { value: parsed, mtimeMs: stat.mtimeMs };
  } catch {
    return null;
  }
}

function isValidSnapshot(value: unknown): value is LiveSceneSnapshot {
  if (!value || typeof value !== 'object') return false;
  const v = value as Record<string, unknown>;
  return Array.isArray(v.parts) && typeof v.timestamp_ms === 'number';
}

function parseBinaryStlTriangleCount(filePath: string): number {
  try {
    const stat = fs.statSync(filePath);
    if (stat.size >= 84) {
      const fd = fs.openSync(filePath, 'r');
      try {
        const header = Buffer.alloc(84);
        fs.readSync(fd, header, 0, 84, 0);
        const triCount = header.readUInt32LE(80);
        if (84 + triCount * 50 === stat.size) {
          return triCount;
        }
      } finally {
        fs.closeSync(fd);
      }
    }
    return 0;
  } catch {
    return 0;
  }
}

export class LiveSyncStore extends EventEmitter {
  private readonly workspaceLiveDir: string;
  private readonly tempLiveDir: string;
  private readonly legacyStlPath: string;
  private readonly pollIntervalMs: number;
  private readonly watchers: fs.FSWatcher[] = [];
  private pollTimer: ReturnType<typeof setInterval> | null = null;
  private lastSignature = '';
  private lastCoupledProgressSignature = '';
  private lastCoupledFramesSignature = '';
  private pushSequence = 1;

  constructor(options: LiveSyncStoreOptions) {
    super();
    this.workspaceLiveDir = path.join(options.workspaceRoot, '.mcp_live');
    this.tempLiveDir =
      options.tempLiveDir ?? path.join(os.tmpdir(), 'mcp-cad-live');
    this.legacyStlPath =
      options.legacyStlPath ??
      path.join(options.workspaceRoot, 'output', '_mcp_preview_live.stl');
    this.pollIntervalMs = options.pollIntervalMs ?? 400;
  }

  public getWorkspaceLiveDir(): string {
    return this.workspaceLiveDir;
  }

  public getTempLiveDir(): string {
    return this.tempLiveDir;
  }

  private getManifestDirs(): string[] {
    const dirs = [this.workspaceLiveDir];
    if (path.resolve(this.tempLiveDir) !== path.resolve(this.workspaceLiveDir)) {
      dirs.push(this.tempLiveDir);
    }
    return dirs;
  }

  private resolveFreshestCandidate(): ResolvedSnapshotCandidate | null {
    const candidates: ResolvedSnapshotCandidate[] = [];

    for (const dir of this.getManifestDirs()) {
      const manifestPath = path.join(dir, 'scene.json');
      const loaded = readJsonSafely<LiveSceneSnapshot>(manifestPath);
      if (loaded && isValidSnapshot(loaded.value)) {
        const snap = loaded.value;
        const freshnessMs = Math.max(snap.timestamp_ms || 0, Math.floor(loaded.mtimeMs));
        candidates.push({
          sourceKind: 'manifest',
          sourceDir: dir,
          sourcePath: manifestPath,
          freshnessMs,
          signature: `manifest:${manifestPath}:${snap.revision}:${snap.timestamp_ms}:${Math.floor(loaded.mtimeMs)}`,
          snapshot: snap,
        });
      }
    }

    // Legacy fallback: works immediately even with an un-rebuilt mcp-cad-server.exe
    // whenever the agent calls open_preview_window (which writes ./output/_mcp_preview_live.stl).
    try {
      if (fs.existsSync(this.legacyStlPath)) {
        const stat = fs.statSync(this.legacyStlPath);
        if (stat.isFile() && stat.size > 0) {
          const mtimeMs = Math.floor(stat.mtimeMs);
          const triCount = parseBinaryStlTriangleCount(this.legacyStlPath);
          const legacySnapshot: LiveSceneSnapshot = {
            schema_version: '1.0.0',
            revision: mtimeMs,
            timestamp_ms: mtimeMs,
            pid: 0,
            source: 'legacy_preview_stl',
            length_unit: 'm',
            last_tool: {
              tool_name: 'open_preview_window',
              status: 'success',
              summary: 'Синхронизировано из output/_mcp_preview_live.stl',
              timestamp_ms: mtimeMs,
              duration_ms: 0,
            },
            recent_tools: [
              {
                tool_name: 'open_preview_window',
                status: 'success',
                summary: 'Синхронизировано из output/_mcp_preview_live.stl',
                timestamp_ms: mtimeMs,
                duration_ms: 0,
              },
            ],
            parts: [
              {
                id: 'mcp_preview_live',
                safe_id: 'mcp_preview_live',
                name: 'Модель агента (_mcp_preview_live.stl)',
                revision: mtimeMs,
                is_fixed: false,
                material: {
                  name: 'steel_carbon_aisi1045',
                  youngs_modulus_gpa: 205,
                  poissons_ratio: 0.29,
                  yield_strength_mpa: 350,
                  density_kg_m3: 7850,
                  failure_mode: 'ductile',
                },
                triangle_count: triCount,
                vertex_count: triCount * 3,
                volume_m3: 0,
                mass_kg: 0,
                bounding_box_m: [
                  [0, 0, 0],
                  [0, 0, 0],
                ],
                stl_file: 'output/_mcp_preview_live.stl',
                dag: [
                  {
                    id: 'legacy_stl_node',
                    name: `Legacy Preview STL (${triCount.toLocaleString()} Δ)`,
                    op_type: 'stl_import',
                    enabled: true,
                    params: { file_path: 'output/_mcp_preview_live.stl' },
                  },
                ],
              },
            ],
            joints: [],
          };

          candidates.push({
            sourceKind: 'legacy_stl',
            sourceDir: path.dirname(this.legacyStlPath),
            sourcePath: this.legacyStlPath,
            freshnessMs: mtimeMs,
            signature: `legacy:${this.legacyStlPath}:${mtimeMs}:${stat.size}`,
            snapshot: legacySnapshot,
          });
        }
      }
    } catch {
      // Ignore unreadable legacy file
    }

    if (candidates.length === 0) return null;

    candidates.sort((a, b) => {
      if (b.freshnessMs !== a.freshnessMs) {
        return b.freshnessMs - a.freshnessMs;
      }
      // Prefer full manifest over legacy STL on equal millisecond timestamps
      if (a.sourceKind === 'manifest' && b.sourceKind !== 'manifest') return -1;
      if (b.sourceKind === 'manifest' && a.sourceKind !== 'manifest') return 1;
      return 0;
    });

    return candidates[0];
  }

  public readCurrentScene(): LiveSyncApiResponse {
    const candidate = this.resolveFreshestCandidate();
    if (!candidate) {
      return { active: false, sourceKind: 'none', snapshot: null };
    }
    return {
      active: true,
      sourceKind: candidate.sourceKind,
      snapshot: candidate.snapshot,
    };
  }

  public readPartStlBuffer(partId: string): Buffer | null {
    if (!isSafeLivePartId(partId)) {
      return null;
    }

    const candidate = this.resolveFreshestCandidate();
    if (!candidate) return null;

    if (candidate.sourceKind === 'legacy_stl') {
      if (partId === 'mcp_preview_live' && fs.existsSync(this.legacyStlPath)) {
        return fs.readFileSync(this.legacyStlPath);
      }
      return null;
    }

    const safeStem = sanitizeLivePartId(partId);
    const orderedDirs = [
      candidate.sourceDir,
      ...this.getManifestDirs().filter((d) => d !== candidate.sourceDir),
    ];

    for (const dir of orderedDirs) {
      const stlPath = path.join(dir, 'parts', `${safeStem}.stl`);
      const resolvedPartsRoot = path.resolve(dir, 'parts');
      const resolvedTarget = path.resolve(stlPath);
      if (
        !resolvedTarget.startsWith(resolvedPartsRoot + path.sep) &&
        resolvedTarget !== resolvedPartsRoot
      ) {
        continue;
      }
      if (fs.existsSync(resolvedTarget)) {
        return fs.readFileSync(resolvedTarget);
      }
    }

    return null;
  }

  public readCoupledSimJson(): unknown | null {
    const candidate = this.resolveFreshestCandidate();
    const orderedDirs = candidate && candidate.sourceKind === 'manifest'
      ? [
          candidate.sourceDir,
          ...this.getManifestDirs().filter((d) => d !== candidate.sourceDir),
        ]
      : this.getManifestDirs();
    for (const dir of orderedDirs) {
      const filePath = path.join(dir, 'coupled_sim.json');
      const parsed = readJsonSafely<unknown>(filePath);
      if (parsed) {
        return parsed.value;
      }
    }
    return null;
  }

  private resolveFreshestCoupledProgress(): {
    signature: string;
    progress: LiveCoupledSimProgress;
  } | null {
    const candidates: Array<{
      freshnessMs: number;
      signature: string;
      progress: LiveCoupledSimProgress;
    }> = [];
    for (const dir of this.getManifestDirs()) {
      const filePath = path.join(dir, 'coupled_sim_progress.json');
      const loaded = readJsonSafely<LiveCoupledSimProgress>(filePath);
      if (loaded && loaded.value && typeof loaded.value.job_id === 'string') {
        const p = loaded.value;
        const freshnessMs = Math.max(p.updated_at_ms || 0, Math.floor(loaded.mtimeMs));
        candidates.push({
          freshnessMs,
          signature: `${filePath}:${p.job_id}:${p.state}:${p.progress_pct}:${p.step_count}:${p.frames_cached}:${p.updated_at_ms}:${Math.floor(loaded.mtimeMs)}`,
          progress: p,
        });
      }
    }
    if (candidates.length === 0) return null;
    candidates.sort((a, b) => b.freshnessMs - a.freshnessMs);
    return {
      signature: candidates[0].signature,
      progress: candidates[0].progress,
    };
  }

  private resolveFreshestCoupledFramesSignature(): {
    signature: string;
    payload: unknown;
  } | null {
    const candidates: Array<{
      mtimeMs: number;
      signature: string;
      payload: unknown;
    }> = [];
    for (const dir of this.getManifestDirs()) {
      const filePath = path.join(dir, 'coupled_sim.json');
      try {
        if (!fs.existsSync(filePath)) continue;
        const stat = fs.statSync(filePath);
        if (!stat.isFile() || stat.size === 0) continue;
        const raw = fs.readFileSync(filePath, 'utf8');
        const parsed = JSON.parse(raw) as {
          summary?: { simulation_id?: string; frame_count?: number };
          frames?: unknown[];
        };
        const simId = parsed?.summary?.simulation_id ?? '';
        const frameCount = Array.isArray(parsed?.frames) ? parsed.frames.length : 0;
        candidates.push({
          mtimeMs: stat.mtimeMs,
          signature: `${filePath}:${simId}:${frameCount}:${stat.size}:${Math.floor(stat.mtimeMs)}`,
          payload: parsed,
        });
      } catch {
        // Ignore transient write
      }
    }
    if (candidates.length === 0) return null;
    candidates.sort((a, b) => b.mtimeMs - a.mtimeMs);
    return {
      signature: candidates[0].signature,
      payload: candidates[0].payload,
    };
  }

  public readCoupledSimProgress(): LiveCoupledSimProgress | null {
    return this.resolveFreshestCoupledProgress()?.progress ?? null;
  }

  public pushCoupledSimProgress(
    progress: LiveCoupledSimProgress,
    coupledSimPayload?: unknown
  ): LiveCoupledSimProgress {
    const normalized: LiveCoupledSimProgress = {
      ...progress,
      updated_at_ms:
        typeof progress.updated_at_ms === 'number' && progress.updated_at_ms > 0
          ? progress.updated_at_ms
          : Date.now(),
    };
    const progBytes = JSON.stringify(normalized);
    const coupledBytes =
      coupledSimPayload !== undefined && coupledSimPayload !== null
        ? JSON.stringify(coupledSimPayload)
        : null;

    for (const dir of this.getManifestDirs()) {
      if (coupledBytes) {
        writeAtomicFile(path.join(dir, 'coupled_sim.json'), coupledBytes);
      }
      writeAtomicFile(path.join(dir, 'coupled_sim_progress.json'), progBytes);
    }

    this.checkForUpdates();
    return normalized;
  }

  public pushSnapshot(payload: PushLiveSnapshotPayload): LiveSceneSnapshot {
    const now =
      typeof payload.timestamp_ms === 'number' && Number.isFinite(payload.timestamp_ms)
        ? Math.floor(payload.timestamp_ms)
        : Date.now();
    const revision =
      typeof payload.revision === 'number' && Number.isFinite(payload.revision)
        ? Math.floor(payload.revision)
        : this.pushSequence++;

    const normalizedParts: LiveScenePart[] = [];
    const stlPayloads: Array<{ safeId: string; buffer: Buffer }> = [];

    for (const rawPart of payload.parts || []) {
      const id = String(rawPart.id || '').trim();
      if (!id) continue;
      const safeId = sanitizeLivePartId(rawPart.safe_id || id);

      if (typeof rawPart.stl_base64 === 'string' && rawPart.stl_base64.length > 0) {
        stlPayloads.push({
          safeId,
          buffer: Buffer.from(rawPart.stl_base64, 'base64'),
        });
      }

      normalizedParts.push({
        id,
        safe_id: safeId,
        name: rawPart.name || id,
        revision:
          typeof rawPart.revision === 'number' ? rawPart.revision : revision,
        is_fixed: Boolean(rawPart.is_fixed),
        material: rawPart.material ?? {
          name: 'steel_carbon_aisi1045',
          youngs_modulus_gpa: 205,
          poissons_ratio: 0.29,
          yield_strength_mpa: 350,
          density_kg_m3: 7850,
          failure_mode: 'ductile',
        },
        triangle_count: rawPart.triangle_count ?? 0,
        vertex_count: rawPart.vertex_count ?? 0,
        volume_m3: rawPart.volume_m3 ?? 0,
        mass_kg: rawPart.mass_kg ?? 0,
        bounding_box_m: rawPart.bounding_box_m ?? [
          [0, 0, 0],
          [0, 0, 0],
        ],
        stl_file:
          rawPart.stl_file !== undefined
            ? rawPart.stl_file
            : stlPayloads.some((s) => s.safeId === safeId)
              ? `parts/${safeId}.stl`
              : null,
        dag: Array.isArray(rawPart.dag) ? rawPart.dag : [],
        fea: rawPart.fea ?? null,
      });
    }

    const snapshot: LiveSceneSnapshot = {
      schema_version: '1.0.0',
      revision,
      timestamp_ms: now,
      pid: payload.pid ?? process.pid,
      source: payload.source ?? 'http_push',
      length_unit: payload.length_unit ?? 'm',
      last_tool: payload.last_tool ?? null,
      recent_tools: Array.isArray(payload.recent_tools)
        ? payload.recent_tools
        : payload.last_tool
          ? [payload.last_tool]
          : [],
      parts: normalizedParts,
      joints: Array.isArray(payload.joints) ? payload.joints : [],
      physics_simulation: payload.physics_simulation ?? null,
      coupled_simulation: payload.coupled_simulation ?? null,
    };

    const jsonBytes = JSON.stringify(snapshot, null, 2);
    const coupledBytes =
      payload.coupled_sim_payload !== undefined && payload.coupled_sim_payload !== null
        ? JSON.stringify(payload.coupled_sim_payload)
        : null;
    for (const dir of this.getManifestDirs()) {
      for (const stl of stlPayloads) {
        writeAtomicFile(path.join(dir, 'parts', `${stl.safeId}.stl`), stl.buffer);
      }
      if (coupledBytes) {
        writeAtomicFile(path.join(dir, 'coupled_sim.json'), coupledBytes);
      }
      writeAtomicFile(path.join(dir, 'scene.json'), jsonBytes);
    }

    this.checkForUpdates();
    return snapshot;
  }

  public clearSnapshot(clearLegacyStl = false): void {
    for (const dir of this.getManifestDirs()) {
      for (const fileName of ['scene.json', 'coupled_sim.json', 'coupled_sim_progress.json']) {
        const filePath = path.join(dir, fileName);
        try {
          if (fs.existsSync(filePath)) {
            fs.unlinkSync(filePath);
          }
        } catch {
          // Ignore
        }
      }
      const partsDir = path.join(dir, 'parts');
      try {
        if (fs.existsSync(partsDir)) {
          for (const entry of fs.readdirSync(partsDir)) {
            if (entry.endsWith('.stl') || entry.endsWith('.tmp')) {
              fs.unlinkSync(path.join(partsDir, entry));
            }
          }
        }
      } catch {
        // Ignore
      }
    }

    if (clearLegacyStl) {
      try {
        if (fs.existsSync(this.legacyStlPath)) {
          fs.unlinkSync(this.legacyStlPath);
        }
      } catch {
        // Ignore
      }
    }

    this.checkForUpdates();
  }

  public writeStudioHeartbeat(
    connectedClients: number,
    studioUrl = 'http://localhost:3000/?bridge=live&view=studio'
  ): void {
    const payload = JSON.stringify({
      connected_clients: connectedClients,
      timestamp_ms: Date.now(),
      studio_url: studioUrl,
    });

    for (const dir of this.getManifestDirs()) {
      try {
        writeAtomicFile(path.join(dir, 'studio_heartbeat.json'), payload);
      } catch {
        // Ignore transient write failures
      }
    }
  }

  public checkForUpdates(): void {
    const candidate = this.resolveFreshestCandidate();
    const nextSignature = candidate ? candidate.signature : '';

    if (nextSignature !== this.lastSignature) {
      this.lastSignature = nextSignature;
      if (candidate) {
        this.emit('update', {
          active: true,
          sourceKind: candidate.sourceKind,
          snapshot: candidate.snapshot,
        } satisfies LiveSyncApiResponse);
      } else {
        this.emit('clear', {
          active: false,
          sourceKind: 'none',
          snapshot: null,
        } satisfies LiveSyncApiResponse);
      }
    }

    const coupledFrames = this.resolveFreshestCoupledFramesSignature();
    const nextFramesSig = coupledFrames ? coupledFrames.signature : '';
    if (nextFramesSig !== this.lastCoupledFramesSignature) {
      this.lastCoupledFramesSignature = nextFramesSig;
      if (coupledFrames) {
        const payload = coupledFrames.payload as {
          summary?: { simulation_id?: string; frame_count?: number };
          frames?: unknown[];
        };
        this.emit('coupled_frames', {
          simulation_id: payload?.summary?.simulation_id ?? '',
          frame_count: Array.isArray(payload?.frames)
            ? payload.frames.length
            : (payload?.summary?.frame_count ?? 0),
          payload: coupledFrames.payload,
        });
      }
    }

    const coupledProgress = this.resolveFreshestCoupledProgress();
    const nextProgSig = coupledProgress ? coupledProgress.signature : '';
    if (nextProgSig !== this.lastCoupledProgressSignature) {
      this.lastCoupledProgressSignature = nextProgSig;
      if (coupledProgress) {
        this.emit('coupled_progress', coupledProgress.progress);
      }
    }
  }

  public startWatching(): void {
    this.stopWatching();

    const initial = this.resolveFreshestCandidate();
    this.lastSignature = initial ? initial.signature : '';
    this.lastCoupledProgressSignature =
      this.resolveFreshestCoupledProgress()?.signature ?? '';
    this.lastCoupledFramesSignature =
      this.resolveFreshestCoupledFramesSignature()?.signature ?? '';

    const dirsToWatch = [
      ...this.getManifestDirs(),
      path.dirname(this.legacyStlPath),
    ];

    for (const dir of dirsToWatch) {
      try {
        if (fs.existsSync(dir)) {
          const watcher = fs.watch(dir, { persistent: false }, () => {
            this.checkForUpdates();
          });
          watcher.on('error', () => {
            // Polling fallback covers watcher errors on Windows/NFS
          });
          this.watchers.push(watcher);
        }
      } catch {
        // Directory may not exist yet; poll timer picks it up once created
      }
    }

    this.pollTimer = setInterval(() => {
      this.checkForUpdates();
    }, this.pollIntervalMs);
    this.pollTimer.unref?.();
  }

  public stopWatching(): void {
    for (const watcher of this.watchers) {
      try {
        watcher.close();
      } catch {
        // Ignore
      }
    }
    this.watchers.length = 0;

    if (this.pollTimer) {
      clearInterval(this.pollTimer);
      this.pollTimer = null;
    }
  }
}
