import {
  CadPart,
  DestructionCache,
  DestructionSimConfig,
  KinematicJoint,
  MaterialSpec,
  MeshGeometryData,
  RenderMode,
} from '../types/cad';
import { parseStlBuffer } from './primitives';
import { getMaterialById } from './materials';
import { MCP_TOOLS_CATALOG } from './mcp_dispatcher';
import {
  type RustCoupledSimBakeSummaryJson,
  type RustCoupledSimFrameJson,
} from './field_colormaps';
import {
  estimateRemainingMs,
  IncrementalRustSimAssembler,
  type LiveFemSolveStatus,
} from './coupled_sim_stream';
import {
  buildRustSceneSyncPlan,
  resolveRustEndpoint,
  type RustAccuracyLevel,
  type RustSceneSyncPlan,
} from './rust_scene_sync';

export const RUST_MCP_TOOLS_COUNT = 43;
export const TS_MCP_TOOLS_COUNT = MCP_TOOLS_CATALOG.length;

export type EngineRuntimeMode =
  | 'windows_exe_connected'
  | 'unified_hybrid_kernel';

export interface WindowsBridgeStatus {
  mode: EngineRuntimeMode;
  endpointUrl: string;
  connectedToWindowsExe: boolean;
  isSameOriginProxy?: boolean;
  cargoAvailable?: boolean;
  binaryExists?: boolean;
  diagnosticMessage?: string;
  serverVersion: string;
  activeToolsCount: number;
  lastSyncTimestamp: string;
  latencyMs: number;
  rustStlCacheCount: number;
}

export type NativeBridgeStatus = WindowsBridgeStatus;

export const DEFAULT_BRIDGE_STATUS: WindowsBridgeStatus = {
  mode: 'unified_hybrid_kernel',
  endpointUrl: '',
  connectedToWindowsExe: false,
  isSameOriginProxy: true,
  serverVersion: '0.1.0-hybrid-wgpu-sync',
  activeToolsCount: TS_MCP_TOOLS_COUNT,
  lastSyncTimestamp: '00:00:00',
  latencyMs: 0.4,
  rustStlCacheCount: 5,
};

export class RustBakeCancelledError extends Error {
  constructor(message: string = 'Расчёт Rust FEM отменён пользователем') {
    super(message);
    this.name = 'RustBakeCancelledError';
  }
}

export interface GoldenArtifactMapping {
  presetId: string;
  artifactPrefix: string;
  stlUrl?: string;
  compiledStlUrl?: string;
  expectedTriangles?: number;
  expectedGenus: number;
  title: string;
  titleRu?: string;
}

/**
 * Canonical mapping between preset IDs, Windows WGPU golden screenshots (`/qa/artifacts/*.png`),
 * and pre-compiled Windows Rust binary STL files (`/output/*.stl`).
 */
export const GOLDEN_WINDOWS_MAPPINGS: Record<string, GoldenArtifactMapping> = {
  spur_gear: {
    presetId: 'spur_gear',
    artifactPrefix: 'spur_gear_hub_keyway',
    stlUrl: '/output/spur_gear.stl',
    compiledStlUrl: '/output/spur_gear.stl',
    expectedTriangles: 23172,
    expectedGenus: 1,
    title: 'Involute Spur Gear Z=20 (DIN 6885 Keyway)',
  },
  hex_nut_m10: {
    presetId: 'hex_nut_m10',
    artifactPrefix: 'polygon_prism_hex_chamfer',
    stlUrl: '/output/nut_m10.stl',
    compiledStlUrl: '/output/nut_m10.stl',
    expectedTriangles: 25716,
    expectedGenus: 1,
    title: 'M10 Hex Nut (DIN 934 / ISO 68-1)',
  },
  v8_piston: {
    presetId: 'v8_piston',
    artifactPrefix: 'v8_racing_piston',
    stlUrl: '/output/v8_racing_piston.stl',
    compiledStlUrl: '/output/v8_racing_piston.stl',
    expectedTriangles: 62676,
    expectedGenus: 0,
    title: 'V8 Racing Piston (Combustion Bowl & Ring Belt)',
  },
  twist_drill: {
    presetId: 'twist_drill',
    artifactPrefix: 'spur_gear_hub_keyway',
    stlUrl: '/output/drill_bit.stl',
    compiledStlUrl: '/output/drill_bit.stl',
    expectedTriangles: 19991,
    expectedGenus: 0,
    title: 'HSS Twist Drill Ø10mm (DIN 338)',
  },
  wheel_rim: {
    presetId: 'wheel_rim',
    artifactPrefix: 'wheel_rim_5spoke',
    stlUrl: '/output/car_rim.stl',
    compiledStlUrl: '/output/car_rim.stl',
    expectedTriangles: 31629,
    expectedGenus: 11,
    title: '5-Spoke Alloy Wheel Rim 18" (5×114.3 PCD)',
  },
  lightbulb_e27: {
    presetId: 'lightbulb_e27',
    artifactPrefix: 'revolve_partial_180',
    stlUrl: '/output/lightbulb_e27.stl',
    compiledStlUrl: '/output/lightbulb_e27.stl',
    expectedTriangles: 72028,
    expectedGenus: 0,
    title: 'E27 A60 Lightbulb (IEC 60061-1)',
  },
  qa_hex_chamfer: {
    presetId: 'qa_hex_chamfer',
    artifactPrefix: 'polygon_prism_hex_chamfer',
    expectedGenus: 0,
    title: 'QA: DIN 934 Chamfered Hex Prism (BUG-005)',
  },
  qa_revolve_180: {
    presetId: 'qa_revolve_180',
    artifactPrefix: 'revolve_partial_180',
    expectedGenus: 0,
    title: 'QA: Partial Revolve 180° Annular Arch (BUG-003)',
  },
  qa_csg_two_holes: {
    presetId: 'qa_csg_two_holes',
    artifactPrefix: 'csg_chain_two_holes',
    expectedGenus: 2,
    title: 'QA: CSG Chain Two Holes + Keyway (BUG-006)',
  },
};

const stlMeshCache = new Map<string, MeshGeometryData>();

/**
 * Fetches and caches a pre-compiled Windows Rust binary STL from `/output/*.stl`.
 */
export async function loadCachedRustStlMesh(
  stlUrl: string,
  density: number = 7850
): Promise<MeshGeometryData> {
  const cacheKey = `${stlUrl}_${density}`;
  const existing = stlMeshCache.get(cacheKey);
  if (existing) {
    return existing;
  }
  const res = await fetch(stlUrl);
  if (!res.ok) {
    throw new Error(`HTTP ${res.status} fetching ${stlUrl}`);
  }
  const buffer = await res.arrayBuffer();
  const mesh = parseStlBuffer(buffer, density, 1.0);
  stlMeshCache.set(cacheKey, mesh);
  return mesh;
}

export function getCachedRustStlCount(): number {
  return stlMeshCache.size;
}

export interface NativeStlLoadOptions {
  /** Optional explicit material ID, canonical Rust material ID, or alias used to resolve density via getMaterialById */
  materialId?: string;
  /** Optional materials registry snapshot (e.g. nextState.materials right after executeMcpToolOnEngineState) */
  materials?: MaterialSpec[];
  /**
   * When set (e.g. during async hydration after MCP `import_stl`), updates only the matching part
   * in-place without wiping other parts, joints, part_id, materialId, name, or isFixed.
   */
  hydratePartId?: string;
}

export interface ResolvedNativeStlMaterial {
  partId: string;
  materialId: string;
  material: MaterialSpec;
  density: number;
}

/**
 * Resolves the target part ID, material ID, and physical density (kg/m³) for a native STL URL
 * using `getMaterialById` so `rim` parts and explicit material overrides never fall back to a
 * hardcoded 7850 kg/m³ steel density.
 */
export function resolveNativeStlMaterialSpec(
  stlUrl: string,
  materials: MaterialSpec[],
  explicitMaterialId?: string
): ResolvedNativeStlMaterial {
  const partId =
    stlUrl
      .split('/')
      .pop()
      ?.replace(/\.stl$/i, '') || 'native_stl';
  const trimmedExplicit = explicitMaterialId?.trim();
  const materialId =
    trimmedExplicit ||
    (partId.toLowerCase().includes('rim') ? 'alu_6061' : 'steel_1045');
  const material = getMaterialById(materialId, materials);
  return {
    partId,
    materialId,
    material,
    density: material.density,
  };
}

/**
 * Rescales a parsed mesh's mass and principal inertia tensor to match `targetDensity` (kg/m³)
 * without mutating the input mesh object.
 */
export function rescaleMeshForDensity(
  mesh: MeshGeometryData,
  targetDensity: number
): MeshGeometryData {
  if (
    !Number.isFinite(targetDensity) ||
    targetDensity <= 0 ||
    !Number.isFinite(mesh.volume) ||
    mesh.volume <= 0
  ) {
    return mesh;
  }
  const targetMass = mesh.volume * targetDensity;
  if (
    mesh.mass > 0 &&
    Math.abs(targetMass - mesh.mass) / Math.max(mesh.mass, 1e-12) <= 1e-9
  ) {
    return mesh;
  }
  const ratio = mesh.mass > 0 ? targetMass / mesh.mass : 1;
  return {
    ...mesh,
    mass: targetMass,
    inertiaTensor: [
      mesh.inertiaTensor[0] * ratio,
      mesh.inertiaTensor[1] * ratio,
      mesh.inertiaTensor[2] * ratio,
    ],
  };
}

/**
 * Builds a standalone `CadPart` for direct native STL inspection (`handleLoadNativeStlUrl` without `hydratePartId`).
 */
export function buildStandaloneNativeStlPart(
  stlUrl: string,
  label: string,
  parsedMesh: MeshGeometryData,
  materialId: string,
  partId?: string
): CadPart {
  const resolvedPartId =
    partId ||
    stlUrl
      .split('/')
      .pop()
      ?.replace(/\.stl$/i, '') ||
    'native_stl';
  return {
    id: resolvedPartId,
    name: `Rust STL: ${label.split(' ')[0]}`,
    materialId,
    dag: [
      {
        id: 'node_stl',
        name: `Compiled Windows STL (${parsedMesh.triangleCount.toLocaleString()} tris)`,
        type: 'stl_import',
        params: {
          file_path: stlUrl,
          triangles: parsedMesh.triangleCount,
          mesh: parsedMesh,
        },
        enabled: true,
      },
    ],
    mesh: parsedMesh,
    visible: true,
    position: [0, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
    nativeStlUrl: stlUrl,
  };
}

/**
 * Hydrates the parsed STL mesh onto a specific imported part in `prevParts`, preserving all other
 * parts in the scene as well as the target part's `id`, `name`, `materialId`, `isFixed`, transform,
 * and non-STL DAG nodes.
 */
export function hydrateImportedStlPartInScene(
  prevParts: CadPart[],
  targetPartId: string,
  stlUrl: string,
  parsedMesh: MeshGeometryData,
  materials?: MaterialSpec[]
): { parts: CadPart[]; hydratedPart: CadPart | null } {
  let hydratedPart: CadPart | null = null;
  const parts = prevParts.map((part) => {
    if (part.id !== targetPartId) {
      return part;
    }
    const targetDensity = materials
      ? getMaterialById(part.materialId, materials).density
      : undefined;
    const finalMesh =
      targetDensity !== undefined
        ? rescaleMeshForDensity(parsedMesh, targetDensity)
        : parsedMesh;
    const nextDag = part.dag.map((node) =>
      node.type === 'stl_import'
        ? {
            ...node,
            params: {
              ...node.params,
              file_path: node.params?.file_path ?? stlUrl,
              triangles: finalMesh.triangleCount,
              mesh: finalMesh,
            },
          }
        : node
    );
    hydratedPart = {
      ...part,
      mesh: finalMesh,
      dag: nextDag,
      nativeStlUrl: part.nativeStlUrl || stlUrl,
    };
    return hydratedPart;
  });

  return { parts, hydratedPart };
}

/**
 * Returns the corresponding `/qa/artifacts/<model>_<mode>.png` URL generated by the
 * Windows WGPU `OffscreenRenderer` if a golden artifact exists for the given preset & shader mode.
 */
export function getWindowsWgpuArtifactUrl(
  presetId: string,
  renderMode: RenderMode
): string | null {
  const mapping = GOLDEN_WINDOWS_MAPPINGS[presetId];
  if (!mapping) return null;

  const supportedModes = new Set([
    'shaded',
    'wireframe',
    'normals',
    'backfaces',
    'boundary_edges',
  ]);
  const modeKey =
    renderMode === 'flat_shading'
      ? 'shaded'
      : supportedModes.has(renderMode)
        ? renderMode
        : 'shaded';

  return `/qa/artifacts/${mapping.artifactPrefix}_${modeKey}.png`;
}

/**
 * Probes whether the native Rust `mcp-cad-server` HTTP/SSE daemon is reachable.
 *
 * - Default (`endpointUrl = ""` / `"auto"`): probes same-origin Express proxy `GET /api/rust/status`
 *   (safe in every environment, zero cross-origin localhost noise).
 * - If `?bridge=live` is present in the browser URL and same-origin proxy reports unhealthy, also
 *   probes direct `http://127.0.0.1:8080`.
 * - Explicit `http(s)://...` URL (when user connects manually or passes `forceProbe = true`):
 *   probes `${base}/message` via JSON-RPC `initialize`.
 */
export async function probeWindowsMcpServer(
  endpointUrl: string = '',
  forceProbe: boolean = false
): Promise<WindowsBridgeStatus> {
  const t0 = performance.now();
  const resolved = resolveRustEndpoint(endpointUrl);

  if (!resolved.ok) {
    return {
      mode: 'unified_hybrid_kernel',
      endpointUrl,
      connectedToWindowsExe: false,
      isSameOriginProxy: false,
      diagnosticMessage: resolved.error,
      serverVersion: '0.1.0-hybrid-wgpu-sync',
      activeToolsCount: TS_MCP_TOOLS_COUNT,
      lastSyncTimestamp: new Date().toTimeString().slice(0, 8),
      latencyMs: 0,
      rustStlCacheCount: stlMeshCache.size,
    };
  }

  let cargoAvailable: boolean | undefined;
  let binaryExists: boolean | undefined;
  let diagnosticMessage: string | undefined;

  const probeDirectMcp = async (mcpUrl: string, displayEndpoint: string, isProxy: boolean) => {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 1500);
    try {
      const res = await fetch(mcpUrl, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          jsonrpc: '2.0',
          id: 1,
          method: 'initialize',
          params: {
            protocolVersion: '2024-11-05',
            capabilities: {},
            clientInfo: { name: 'mcp-cad-unified-studio', version: '1.0.0' },
          },
        }),
        signal: controller.signal,
      });
      clearTimeout(timeout);
      if (res.ok) {
        const data = await res.json();
        if (!data?.error) {
          const latencyMs = Number((performance.now() - t0).toFixed(1));
          return {
            mode: 'windows_exe_connected' as const,
            endpointUrl: displayEndpoint,
            connectedToWindowsExe: true,
            isSameOriginProxy: isProxy,
            cargoAvailable,
            binaryExists,
            diagnosticMessage: isProxy
              ? 'Подключено через локальный прокси /api -> Rust mcp-cad-server (:8080)'
              : `Прямое подключение к ${displayEndpoint}`,
            serverVersion:
              data?.result?.serverInfo?.version || '0.1.0-rust-fem-f64',
            activeToolsCount: RUST_MCP_TOOLS_COUNT,
            lastSyncTimestamp: new Date().toTimeString().slice(0, 8),
            latencyMs,
            rustStlCacheCount: stlMeshCache.size,
          };
        }
      }
    } catch {
      clearTimeout(timeout);
    }
    return null;
  };

  if (typeof fetch === 'function') {
    if (resolved.isSameOriginProxy) {
      // 1. Check same-origin Express proxy health (/api/rust/status)
      try {
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 1800);
        const statusRes = await fetch(resolved.statusUrl, {
          method: 'GET',
          signal: controller.signal,
        });
        clearTimeout(timeout);

        if (statusRes.ok) {
          const st = await statusRes.json();
          cargoAvailable = Boolean(st?.cargoAvailable);
          binaryExists = Boolean(st?.binaryExists);
          if (st?.healthy) {
            const initHit = await probeDirectMcp(resolved.mcpUrl, '', true);
            if (initHit) return initHit;
            return {
              mode: 'windows_exe_connected',
              endpointUrl: '',
              connectedToWindowsExe: true,
              isSameOriginProxy: true,
              cargoAvailable,
              binaryExists,
              diagnosticMessage:
                'Подключено через прокси /api -> Rust mcp-cad-server (:8080)',
              serverVersion: '0.1.0-rust-fem-f64',
              activeToolsCount: RUST_MCP_TOOLS_COUNT,
              lastSyncTimestamp: new Date().toTimeString().slice(0, 8),
              latencyMs: Number((performance.now() - t0).toFixed(1)),
              rustStlCacheCount: stlMeshCache.size,
            };
          } else {
            diagnosticMessage = !binaryExists && !cargoAvailable
              ? 'На сервере не установлен Rust/Cargo. Запустите mcp-cad-server локально на :8080 и укажите его URL.'
              : 'Локальный mcp-cad-server (:8080) не запущен. Запустите ./run_server.sh или cargo run --release --bin mcp-cad-server -- --mode sse --port 8080.';
          }
        }
      } catch {
        diagnosticMessage =
          'Прокси /api/rust/status недоступен. Укажите прямой URL Rust-сервера (например http://127.0.0.1:8080).';
      }

      // 2. Optional direct localhost probe when explicitly enabled via `?bridge=live`
      const allowLiveLocalhost =
        typeof window !== 'undefined' &&
        window.location.search.includes('bridge=live');
      if (allowLiveLocalhost) {
        const direct = await probeDirectMcp(
          'http://127.0.0.1:8080/message',
          'http://127.0.0.1:8080',
          false
        );
        if (direct) return direct;
      }
    } else if (
      forceProbe ||
      (typeof window !== 'undefined' &&
        window.location.search.includes('bridge=live'))
    ) {
      const direct = await probeDirectMcp(
        resolved.mcpUrl,
        resolved.endpointUrl,
        false
      );
      if (direct) return direct;
      diagnosticMessage = `Не удалось подключиться к ${resolved.endpointUrl}. Убедитесь, что mcp-cad-server запущен с флагом --mode sse.`;
    }
  }

  return {
    mode: 'unified_hybrid_kernel',
    endpointUrl: resolved.endpointUrl,
    connectedToWindowsExe: false,
    isSameOriginProxy: resolved.isSameOriginProxy,
    cargoAvailable,
    binaryExists,
    diagnosticMessage,
    serverVersion: '0.1.0-hybrid-wgpu-sync',
    activeToolsCount: TS_MCP_TOOLS_COUNT,
    lastSyncTimestamp: new Date().toTimeString().slice(0, 8),
    latencyMs: Number((performance.now() - t0).toFixed(1)),
    rustStlCacheCount: stlMeshCache.size,
  };
}

/**
 * Executes an MCP tool call over JSON-RPC 2.0 against either the same-origin Express proxy (`/api/mcp`)
 * or a direct Rust `mcp-cad-server` HTTP/SSE instance (`${base}/message`).
 */
export async function executeRemoteWindowsMcpTool(
  endpointUrl: string,
  toolName: string,
  args: Record<string, unknown>,
  signal?: AbortSignal
): Promise<{ ok: boolean; payload: any; errorMessage?: string }> {
  const resolved = resolveRustEndpoint(endpointUrl);
  if (!resolved.ok) {
    throw new Error(resolved.error);
  }

  const res = await fetch(resolved.mcpUrl, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      jsonrpc: '2.0',
      id: Date.now() + Math.floor(Math.random() * 1000),
      method: 'tools/call',
      params: {
        name: toolName,
        arguments: args,
      },
    }),
    signal,
  });

  if (!res.ok) {
    let errDetail = `HTTP ${res.status}`;
    try {
      const errJson = await res.json();
      errDetail = errJson?.error?.message || errJson?.error || errDetail;
    } catch {
      // Keep status text
    }
    throw new Error(`Rust MCP (${toolName}): ${errDetail}`);
  }

  const rpc = await res.json();
  if (rpc.error) {
    throw new Error(rpc.error.message || `MCP JSON-RPC Error (${toolName})`);
  }

  const textContent = rpc.result?.content?.find(
    (c: any) => c.type === 'text'
  )?.text;
  let parsed: any = rpc.result;
  if (typeof textContent === 'string') {
    try {
      parsed = JSON.parse(textContent);
    } catch {
      parsed = { rawText: textContent };
    }
  }

  const isErr = Boolean(rpc.result?.isError);
  const errorMessage = isErr
    ? parsed?.error?.message ||
      parsed?.message ||
      (typeof parsed?.error === 'string' ? parsed.error : undefined) ||
      textContent ||
      `MCP tool ${toolName} returned error`
    : undefined;

  return {
    ok: !isErr,
    payload: parsed,
    errorMessage,
  };
}

/**
 * Synchronizes the active Web Studio scene (`parts`, `joints`, `materials`, `config`)
 * into the native Rust `mcp-cad-server` `EngineState` and `PhysicsWorld` before baking.
 */
export async function syncSceneToRustServer(
  endpointUrl: string,
  parts: readonly CadPart[],
  joints: readonly KinematicJoint[],
  materials: readonly MaterialSpec[],
  config: DestructionSimConfig,
  accuracyLevel: RustAccuracyLevel = 'standard',
  onProgress?: (pct: number, stageLabel: string) => void,
  signal?: AbortSignal
): Promise<RustSceneSyncPlan> {
  const plan = buildRustSceneSyncPlan(
    parts,
    joints,
    materials,
    config,
    accuracyLevel
  );

  if (plan.activePartIds.length === 0) {
    throw new Error('В сцене нет видимых деталей для синхронизации с Rust FEM.');
  }

  onProgress?.(4, 'Rust FEM f64: очистка предыдущей сцены сервера...');

  // 1. Clear stale parts from previous Web Studio presets / runs
  const listRes = await executeRemoteWindowsMcpTool(
    endpointUrl,
    'list_parts',
    { length_unit: 'm' },
    signal
  );
  if (listRes.ok && Array.isArray(listRes.payload?.parts)) {
    for (const existing of listRes.payload.parts) {
      const existingId = existing?.id ?? existing?.part_id;
      if (typeof existingId === 'string' && existingId.length > 0) {
        if (signal?.aborted) throw new RustBakeCancelledError();
        await executeRemoteWindowsMcpTool(
          endpointUrl,
          'delete_part',
          { part_id: existingId },
          signal
        );
      }
    }
  }

  // 2. Replay ordered scene-sync operations (units -> custom materials -> parts -> DAG -> transforms -> joints -> motors)
  const totalOps = Math.max(1, plan.operations.length);
  for (let i = 0; i < plan.operations.length; i++) {
    if (signal?.aborted) throw new RustBakeCancelledError();
    const op = plan.operations[i];
    const pct = 5 + Math.round(((i + 1) / totalOps) * 13); // 5% .. 18%
    onProgress?.(
      pct,
      `Rust FEM f64: синхронизация (${op.tool}, ${i + 1}/${totalOps})...`
    );

    const res = await executeRemoteWindowsMcpTool(
      endpointUrl,
      op.tool,
      op.args,
      signal
    );
    if (!res.ok) {
      throw new Error(
        `Сбой синхронизации сцены с Rust (${op.tool}): ${res.errorMessage || 'неизвестная ошибка'}`
      );
    }
  }

  return plan;
}

/**
 * Cancels an active background Rust FEM coupled simulation bake job via `POST /api/coupled-sim/cancel/:job_id`.
 */
export async function cancelNativeCoupledSimulation(
  endpointUrl: string,
  jobId: string
): Promise<boolean> {
  const resolved = resolveRustEndpoint(endpointUrl);
  if (!resolved.ok) return false;
  try {
    const res = await fetch(
      `${resolved.coupledSimBaseUrl}/cancel/${encodeURIComponent(jobId)}`,
      { method: 'POST' }
    );
    return res.ok;
  } catch {
    return false;
  }
}

export interface NativeCoupledBakeOptions {
  joints?: readonly KinematicJoint[];
  accuracyLevel?: RustAccuracyLevel;
  onJobId?: (jobId: string) => void;
  onStatusUpdate?: (status: LiveFemSolveStatus) => void;
  onPartialCache?: (
    cache: DestructionCache,
    latestFrameIndex: number,
    status: LiveFemSolveStatus
  ) => void;
  signal?: AbortSignal;
  /** Internal test flag to bypass MCP scene sync when parts already live on server. */
  skipSceneSync?: boolean;
}

/**
 * Synchronizes the active Web Studio scene into the native Rust `mcp-cad-server`,
 * bakes a real f64 coupled implicit HHT-α dynamic FEM simulation (`/api/coupled-sim/*`),
 * streams SSE/poll progress AND incremental frames live as they are computed,
 * and adapts all frames into a `DestructionCache` tagged with `solverBackend: 'rust_fem_f64'`.
 */
export async function bakeNativeCoupledSimulation(
  endpointUrl: string,
  parts: readonly CadPart[],
  materials: readonly MaterialSpec[],
  config: DestructionSimConfig,
  onProgress?: (pct: number, stageLabel: string) => void,
  accuracyOrOptions: RustAccuracyLevel | NativeCoupledBakeOptions = 'standard'
): Promise<DestructionCache> {
  const opts: NativeCoupledBakeOptions =
    typeof accuracyOrOptions === 'string'
      ? { accuracyLevel: accuracyOrOptions }
      : accuracyOrOptions;

  const accuracyLevel: RustAccuracyLevel = opts.accuracyLevel ?? 'standard';
  const joints: readonly KinematicJoint[] = opts.joints ?? [];
  const signal = opts.signal;

  const resolved = resolveRustEndpoint(endpointUrl);
  if (!resolved.ok) {
    throw new Error(resolved.error);
  }
  const coupledBase = resolved.coupledSimBaseUrl;

  if (signal?.aborted) {
    throw new RustBakeCancelledError();
  }

  let bakePayload: Record<string, unknown>;
  if (opts.skipSceneSync) {
    bakePayload = buildRustSceneSyncPlan(
      parts,
      joints,
      materials,
      config,
      accuracyLevel
    ).bakePayload;
  } else {
    const plan = await syncSceneToRustServer(
      endpointUrl,
      parts,
      joints,
      materials,
      config,
      accuracyLevel,
      onProgress,
      signal
    );
    bakePayload = plan.bakePayload;
  }

  if (signal?.aborted) {
    throw new RustBakeCancelledError();
  }

  onProgress?.(20, `Rust FEM f64 (${accuracyLevel}): запуск HHT-α расчёта...`);

  const bakeRes = await fetch(`${coupledBase}/bake`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(bakePayload),
    signal,
  });

  if (!bakeRes.ok) {
    let errText = `HTTP ${bakeRes.status}`;
    try {
      const errJson = await bakeRes.json();
      errText = errJson?.error || errText;
    } catch {
      // Ignore parse failure
    }
    throw new Error(`Rust FEM bake отклонён: ${errText}`);
  }

  const bakeAck = await bakeRes.json();
  const jobId: string | undefined = bakeAck?.job_id;
  if (jobId) {
    opts.onJobId?.(jobId);
  }

  const expectedDurationS = Number(
    bakeAck?.duration_s ?? bakePayload.duration_s ?? config.durationSec
  );
  let simulationId: string | undefined = bakeAck?.simulation_id;
  let totalFrames: number = bakeAck?.frame_count ?? bakeAck?.total_frames ?? 0;
  let bakeSummary: RustCoupledSimBakeSummaryJson = {
    simulation_id: simulationId || 'latest',
    accuracy_level: accuracyLevel,
    duration_s: expectedDurationS,
    total_frames: totalFrames,
    frame_count: totalFrames,
    bake_time_ms: bakeAck?.elapsed_ms ?? bakeAck?.bake_time_ms,
    elapsed_ms: bakeAck?.elapsed_ms ?? bakeAck?.bake_time_ms,
    events: bakeAck?.events,
  };

  const assembler = new IncrementalRustSimAssembler(
    bakeSummary,
    parts,
    materials,
    config
  );
  const solveStartMs = Date.now();
  let availableFrames = Math.max(
    Number(bakeAck?.frames_cached ?? 0),
    Number(bakeAck?.frame_count ?? 0)
  );
  let fetchedUpTo = -1;
  let fetchedFragCount = 0;
  let fetchedFragmentMeshesOnce = false;
  let pumpPromise: Promise<void> | null = null;

  let latestStatus: LiveFemSolveStatus = {
    active: bakeAck?.status !== 'completed',
    source: 'rust_fem',
    state: bakeAck?.status === 'completed' ? 'completed' : 'running',
    jobId: jobId ?? null,
    simulationId: simulationId ?? null,
    progressPct: bakeAck?.status === 'completed' ? 100 : 20,
    currentTimeSec: 0,
    totalTimeSec: expectedDurationS,
    stepCount: 0,
    frameCount: totalFrames,
    framesCached: availableFrames,
    latestFrameIndex: Math.max(0, availableFrames - 1),
    currentDtSec: 0,
    bisections: 0,
    erodedElements: 0,
    detachedFragments: 0,
    maxVonMisesMPa: 0,
    elapsedMs: Number(bakeAck?.elapsed_ms ?? 0),
    estimatedRemainingMs: null,
    stageLabel: `Rust FEM f64 (${accuracyLevel}): запуск HHT-α расчёта...`,
    cancellable: Boolean(jobId),
  };

  const refreshFragments = async (force = false): Promise<void> => {
    const simKey = simulationId || 'latest';
    if (!force && fetchedFragmentMeshesOnce && latestStatus.detachedFragments <= fetchedFragCount) {
      return;
    }
    try {
      const fragsRes = await fetch(
        `${coupledBase}/fragments/${encodeURIComponent(simKey)}?length_unit=m`,
        { signal }
      );
      if (fragsRes.ok) {
        const fragsJson = await fragsRes.json();
        if (Array.isArray(fragsJson?.fragments)) {
          fetchedFragmentMeshesOnce = true;
          fetchedFragCount = Math.max(fetchedFragCount, fragsJson.fragments.length);
          bakeSummary = {
            ...bakeSummary,
            fragment_meshes: fragsJson.fragments,
          };
          assembler.updateSummary({ fragment_meshes: fragsJson.fragments });
        }
      }
    } catch {
      if (signal?.aborted) throw new RustBakeCancelledError();
    }
  };

  const fetchMissingFramesUpTo = async (
    targetFrameCount: number,
    emitLiveCallbacks: boolean
  ): Promise<void> => {
    const simKey = simulationId || 'latest';
    if (latestStatus.detachedFragments > fetchedFragCount || !fetchedFragmentMeshesOnce) {
      await refreshFragments(false);
    }
    for (let i = fetchedUpTo + 1; i < targetFrameCount; i++) {
      if (signal?.aborted) throw new RustBakeCancelledError();
      const fragInlineParam = fetchedFragmentMeshesOnce ? 'false' : 'true';
      const fRes = await fetch(
        `${coupledBase}/frames/${encodeURIComponent(simKey)}/${i}?include_mesh_fields=true&include_fragment_meshes=${fragInlineParam}&length_unit=m`,
        { signal }
      );
      if (!fRes.ok) {
        if (emitLiveCallbacks && (fRes.status === 404 || fRes.status === 409)) {
          break;
        }
        throw new Error(`Failed loading Rust FEM frame ${i}: HTTP ${fRes.status}`);
      }
      const fJson: RustCoupledSimFrameJson = await fRes.json();
      assembler.appendFrame(fJson);
      fetchedUpTo = i;
      if (opts.onPartialCache) {
        const partialCache = assembler.buildCache();
        opts.onPartialCache(partialCache, i, latestStatus);
      }
      if (!emitLiveCallbacks && targetFrameCount > 0) {
        const tailPct = 86 + Math.round(((i + 1) / targetFrameCount) * 13);
        onProgress?.(
          Math.min(99, tailPct),
          `Rust FEM f64: загрузка поверхностных сеток и полей кадра ${i + 1}/${targetFrameCount}...`
        );
      }
    }
  };

  const schedulePump = () => {
    if (!simulationId || pumpPromise) return;
    pumpPromise = (async () => {
      try {
        while (fetchedUpTo + 1 < availableFrames && !signal?.aborted) {
          const target = availableFrames;
          await fetchMissingFramesUpTo(target, true);
          if (fetchedUpTo + 1 >= availableFrames) break;
        }
      } finally {
        pumpPromise = null;
      }
    })();
  };

  if (jobId && bakeAck?.status !== 'completed') {
    await new Promise<void>((resolve, reject) => {
      let settled = false;
      let es: EventSource | null = null;
      let pollTimer: ReturnType<typeof setInterval> | null = null;

      const onAbort = () => {
        void cancelNativeCoupledSimulation(endpointUrl, jobId);
        finish(new RustBakeCancelledError());
      };

      const finish = (err?: Error) => {
        if (settled) return;
        settled = true;
        if (es) es.close();
        if (pollTimer) clearInterval(pollTimer);
        signal?.removeEventListener('abort', onAbort);
        if (err) reject(err);
        else resolve();
      };

      const handleProgressSample = (data: Record<string, any>) => {
        if (typeof data.simulation_id === 'string' && data.simulation_id) {
          simulationId = data.simulation_id;
          assembler.updateSummary({ simulation_id: simulationId });
        }
        const rawPct = Math.max(0, Math.min(100, Number(data.progress_pct ?? latestStatus.progressPct)));
        const elapsedMs = Number(data.elapsed_ms ?? Date.now() - solveStartMs);
        const framesCached = Number(
          data.frames_cached ?? data.frame_count ?? availableFrames
        );
        const frameCount = Number(data.frame_count ?? data.total_frames ?? totalFrames);
        const latestIdx =
          typeof data.latest_frame_index === 'number'
            ? Math.max(0, data.latest_frame_index)
            : Math.max(0, framesCached - 1);
        const etaMs =
          typeof data.estimated_remaining_ms === 'number' && data.estimated_remaining_ms >= 0
            ? data.estimated_remaining_ms
            : estimateRemainingMs(elapsedMs, rawPct);

        const stageLabel =
          data.stage_label ||
          `Rust FEM f64 (HHT-α): шаг ${data.step_count ?? 0}, t=${Number(data.time_s ?? 0).toFixed(4)}с, σ_max=${Number(data.max_von_mises_mpa ?? 0).toFixed(0)} МПа`;

        latestStatus = {
          active: true,
          source: 'rust_fem',
          state: 'running',
          jobId,
          simulationId: simulationId ?? null,
          progressPct: Math.round(rawPct),
          currentTimeSec: Number(data.time_s ?? latestStatus.currentTimeSec),
          totalTimeSec: Number(data.duration_s ?? expectedDurationS),
          stepCount: Number(data.step_count ?? latestStatus.stepCount),
          frameCount,
          framesCached,
          latestFrameIndex: latestIdx,
          currentDtSec: Number(data.current_dt_s ?? latestStatus.currentDtSec),
          bisections: Number(data.bisections ?? latestStatus.bisections),
          erodedElements: Number(data.eroded_elements ?? latestStatus.erodedElements),
          detachedFragments: Number(data.detached_fragments ?? latestStatus.detachedFragments),
          maxVonMisesMPa: Number(data.max_von_mises_mpa ?? latestStatus.maxVonMisesMPa),
          elapsedMs,
          estimatedRemainingMs: etaMs,
          stageLabel,
          cancellable: true,
        };

        opts.onStatusUpdate?.(latestStatus);
        const scaledPct = Math.min(84, Math.max(20, 20 + Math.round(rawPct * 0.64)));
        onProgress?.(scaledPct, stageLabel);

        if (framesCached > availableFrames) {
          availableFrames = framesCached;
        }
        if (simulationId && availableFrames > fetchedUpTo + 1) {
          schedulePump();
        }
      };

      if (signal) {
        if (signal.aborted) {
          onAbort();
          return;
        }
        signal.addEventListener('abort', onAbort, { once: true });
      }

      if (typeof EventSource === 'function') {
        try {
          es = new EventSource(`${coupledBase}/events`);
          es.addEventListener('coupled_sim', (evt) => {
            try {
              const data = JSON.parse((evt as MessageEvent).data);
              if (data.job_id && data.job_id !== jobId) return;
              if (data.event === 'progress') {
                handleProgressSample(data);
              } else if (data.event === 'completed') {
                simulationId = data.simulation_id || simulationId;
                totalFrames = data.frame_count ?? data.total_frames ?? totalFrames;
                availableFrames = Math.max(availableFrames, totalFrames);
                bakeSummary = {
                  ...bakeSummary,
                  simulation_id: simulationId || 'latest',
                  total_frames: totalFrames,
                  frame_count: totalFrames,
                  bake_time_ms: data.elapsed_ms ?? data.bake_time_ms,
                  elapsed_ms: data.elapsed_ms ?? data.bake_time_ms,
                  events: data.events ?? bakeSummary.events,
                };
                assembler.updateSummary(bakeSummary);
                finish();
              } else if (data.event === 'cancelled') {
                finish(new RustBakeCancelledError());
              } else if (data.event === 'failed') {
                finish(new Error(data.error || 'Rust FEM job failed'));
              }
            } catch {
              // Ignore malformed SSE keepalive payloads
            }
          });
        } catch {
          // Fall back to HTTP status polling below
        }
      }

      // Polling safety net (200ms) for environments without SSE or buffered SSE
      pollTimer = setInterval(async () => {
        try {
          const stRes = await fetch(
            `${coupledBase}/status/${encodeURIComponent(jobId)}`
          );
          if (!stRes.ok) return;
          const st = await stRes.json();
          const job = st?.job ?? st;
          const jobState = job?.state ?? job?.status;
          if (jobState === 'completed') {
            simulationId = job.simulation_id || simulationId;
            totalFrames = job.frame_count ?? job.total_frames ?? totalFrames;
            availableFrames = Math.max(availableFrames, totalFrames);
            bakeSummary = {
              ...bakeSummary,
              simulation_id: simulationId || 'latest',
              total_frames: totalFrames,
              frame_count: totalFrames,
              bake_time_ms: job.elapsed_ms ?? job.bake_time_ms,
              elapsed_ms: job.elapsed_ms ?? job.bake_time_ms,
              events: job.events ?? st.events ?? bakeSummary.events,
            };
            assembler.updateSummary(bakeSummary);
            finish();
          } else if (jobState === 'cancelled') {
            finish(new RustBakeCancelledError());
          } else if (jobState === 'failed') {
            finish(new Error(job.error || 'Rust FEM job failed'));
          } else if (typeof job?.progress_pct === 'number') {
            handleProgressSample(job);
          }
        } catch {
          // Retry next poll tick
        }
      }, 200);
    });
  }

  if (signal?.aborted) {
    throw new RustBakeCancelledError();
  }

  if (pumpPromise) {
    try {
      await pumpPromise;
    } catch {
      if (signal?.aborted) throw new RustBakeCancelledError();
    }
  }

  assembler.updateSummary(bakeSummary);
  await refreshFragments(true);

  const frameCount = Math.max(1, totalFrames, availableFrames);
  latestStatus = {
    ...latestStatus,
    active: false,
    state: 'completed',
    progressPct: 100,
    frameCount,
    framesCached: frameCount,
    estimatedRemainingMs: null,
    cancellable: false,
    stageLabel: 'Rust FEM f64: финализация кэша...',
  };
  await fetchMissingFramesUpTo(frameCount, false);

  onProgress?.(100, 'Rust FEM f64 кэш готов');
  return assembler.buildCache();
}


