import type {
  CachedFemSurfaceMesh,
  CachedPartFrameState,
  CadPart,
  DestructionCache,
  DestructionCacheFrame,
  DestructionSimConfig,
  DestructionTimelineEvent,
  FractureShardGeometry,
  MaterialSpec,
} from '../types/cad';
import { getMaterialById } from './materials';
import {
  buildFragmentGeometryFromRustMesh,
  buildSmallFragmentBox,
  flattenVec3List,
  gatherSurfaceScalars,
  mapRustEventType,
  type RustCoupledSimBakeSummaryJson,
  type RustCoupledSimFragmentMeshJson,
  type RustCoupledSimFrameJson,
} from './field_colormaps';

export type LiveFemSolveSource = 'rust_fem' | 'live_sync' | 'web_local';

export type LiveFemSolveState =
  | 'running'
  | 'cancelling'
  | 'completed'
  | 'cancelled'
  | 'failed';

export interface LiveFemSolveStatus {
  active: boolean;
  source: LiveFemSolveSource;
  state: LiveFemSolveState;
  jobId?: string | null;
  simulationId?: string | null;
  progressPct: number;
  currentTimeSec: number;
  totalTimeSec: number;
  stepCount: number;
  frameCount: number;
  framesCached: number;
  latestFrameIndex: number;
  currentDtSec: number;
  bisections: number;
  erodedElements: number;
  detachedFragments: number;
  maxVonMisesMPa: number;
  elapsedMs: number;
  estimatedRemainingMs: number | null;
  stageLabel: string;
  cancellable: boolean;
}

export interface LiveCoupledProgressPayload {
  job_id?: string | null;
  simulation_id?: string | null;
  state: 'running' | 'completed' | 'cancelled' | 'failed';
  progress_pct: number;
  time_s?: number;
  current_time_s?: number;
  duration_s?: number;
  total_time_s?: number;
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
  stage_label?: string;
  error?: string | null;
  updated_at_ms?: number;
}

/**
 * Estimates remaining solve time in milliseconds from elapsed wall time and completion percentage.
 * Returns `null` when progress is too small (< 2%), invalid, or already finished.
 */
export function estimateRemainingMs(
  elapsedMs: number,
  progressPct: number
): number | null {
  if (
    !Number.isFinite(elapsedMs) ||
    !Number.isFinite(progressPct) ||
    elapsedMs < 200 ||
    progressPct < 2 ||
    progressPct >= 100
  ) {
    return null;
  }
  const remaining = elapsedMs * ((100 - progressPct) / progressPct);
  if (!Number.isFinite(remaining) || remaining < 0) {
    return null;
  }
  return Math.round(remaining);
}

/**
 * Formats millisecond durations in concise Russian units without overflowing seconds to 60.
 */
export function formatDurationMsRu(ms: number | null | undefined): string {
  if (ms === null || ms === undefined || !Number.isFinite(ms) || ms < 0) {
    return '—';
  }
  if (ms < 1000) {
    return `${Math.max(1, Math.round(ms))} мс`;
  }
  if (ms < 59_950) {
    return `${(ms / 1000).toFixed(1)} с`;
  }
  const totalSec = Math.round(ms / 1000);
  if (totalSec < 3600) {
    const mins = Math.floor(totalSec / 60);
    const secs = totalSec % 60;
    return `${mins}м ${String(secs).padStart(2, '0')}с`;
  }
  const hours = Math.floor(totalSec / 3600);
  const mins = Math.floor((totalSec % 3600) / 60);
  return `${hours}ч ${String(mins).padStart(2, '0')}м`;
}

/**
 * Converts a `coupled_sim_progress.json` payload (from Rust live-sync or SSE)
 * into a UI-ready `LiveFemSolveStatus`.
 */
export function liveProgressToSolveStatus(
  p: LiveCoupledProgressPayload,
  source: LiveFemSolveSource = 'live_sync'
): LiveFemSolveStatus {
  const pct = Math.max(0, Math.min(100, Number(p.progress_pct ?? 0)));
  const elapsedMs = Math.max(0, Number(p.elapsed_ms ?? 0));
  const currentTimeSec = Number(p.current_time_s ?? p.time_s ?? 0);
  const totalTimeSec = Number(p.total_time_s ?? p.duration_s ?? 0);
  const explicitEta =
    typeof p.estimated_remaining_ms === 'number' &&
    Number.isFinite(p.estimated_remaining_ms) &&
    p.estimated_remaining_ms >= 0
      ? Math.round(p.estimated_remaining_ms)
      : estimateRemainingMs(elapsedMs, pct);
  const framesCached = Math.max(0, Math.round(p.frames_cached ?? p.frame_count ?? 0));
  const latestFrameIndex =
    typeof p.latest_frame_index === 'number' && Number.isFinite(p.latest_frame_index)
      ? Math.max(0, Math.round(p.latest_frame_index))
      : Math.max(0, framesCached - 1);

  const stageLabel =
    p.stage_label ||
    (p.state === 'completed'
      ? 'Расчёт HHT-α завершён'
      : p.state === 'cancelled'
        ? 'Расчёт отменён'
        : p.state === 'failed'
          ? p.error || 'Ошибка расчёта FEM'
          : `HHT-α FEM · шаг ${p.step_count ?? 0} · t=${currentTimeSec.toFixed(4)}с`);

  return {
    active: p.state === 'running',
    source,
    state: p.state,
    jobId: p.job_id ?? null,
    simulationId: p.simulation_id ?? null,
    progressPct: Math.round(pct),
    currentTimeSec,
    totalTimeSec,
    stepCount: Math.max(0, Math.round(p.step_count ?? 0)),
    frameCount: Math.max(0, Math.round(p.frame_count ?? 0)),
    framesCached,
    latestFrameIndex,
    currentDtSec: Number(p.current_dt_s ?? 0),
    bisections: Math.max(0, Math.round(p.bisections ?? 0)),
    erodedElements: Math.max(0, Math.round(p.eroded_elements ?? 0)),
    detachedFragments: Math.max(0, Math.round(p.detached_fragments ?? 0)),
    maxVonMisesMPa: Number(p.max_von_mises_mpa ?? 0),
    elapsedMs,
    estimatedRemainingMs: p.state === 'running' ? explicitEta : null,
    stageLabel,
    cancellable: p.state === 'running' && Boolean(p.job_id),
  };
}

function resizeFloat32(src: Float32Array | undefined, nextLen: number): Float32Array {
  if (src && src.length === nextLen) return src;
  const out = new Float32Array(nextLen);
  if (src) {
    out.set(src.subarray(0, Math.min(src.length, nextLen)));
  }
  return out;
}

function resizeUint8(src: Uint8Array, nextLen: number): Uint8Array {
  if (src.length === nextLen) return src;
  const out = new Uint8Array(nextLen);
  out.set(src.subarray(0, Math.min(src.length, nextLen)));
  return out;
}

/**
 * Incrementally converts incoming Rust HHT-α simulation frames into a live `DestructionCache`.
 * Preserves a stable `cache.id` across frame arrivals so `CadViewport3D` appends only newly
 * detached shards / FEM surfaces without disposing or resetting existing Three.js GPU buffers.
 */
export class IncrementalRustSimAssembler {
  private summary: RustCoupledSimBakeSummaryJson;
  private readonly parts: readonly CadPart[];
  private readonly materials: readonly MaterialSpec[];
  private readonly config: DestructionSimConfig;
  private readonly partById = new Map<string, CadPart>();
  private readonly cachedFragMeshById = new Map<
    string,
    RustCoupledSimFragmentMeshJson
  >();
  private readonly fragIndexById = new Map<string, number>();
  private readonly shards: FractureShardGeometry[] = [];
  private readonly frames: DestructionCacheFrame[] = [];
  private readonly frameIdxSet = new Set<number>();

  private readonly yieldRefMPa: number;
  private readonly floorY: number;
  private maxStressOverallMPa = 0;
  private maxPlasticOverallPct = 0;
  private maxDamageOverall = 0;
  private maxEnergyOverallJ = 1e-4;
  private contactPoint: [number, number, number] = [0, 0, 0];

  constructor(
    summary: RustCoupledSimBakeSummaryJson,
    parts: readonly CadPart[],
    materials: readonly MaterialSpec[],
    config: DestructionSimConfig
  ) {
    this.summary = { ...summary };
    this.parts = parts;
    this.materials = materials;
    this.config = { ...config };

    let yieldRef = Infinity;
    let floorY = -0.05;
    for (const p of parts) {
      this.partById.set(p.id, p);
      if (!p.visible) continue;
      const minY =
        p.position[1] + p.mesh.boundingBox.min[1] * Math.abs(p.scale[1] || 1);
      if (minY < floorY) floorY = minY;
      if (!p.isFixed) {
        const mat = getMaterialById(p.materialId, materials as MaterialSpec[]);
        const yMPa = mat.yieldStrength / 1e6;
        if (yMPa > 0 && yMPa < yieldRef) yieldRef = yMPa;
      }
    }
    this.yieldRefMPa = Number.isFinite(yieldRef) ? yieldRef : 350;
    this.floorY = floorY;

    if (summary.fragment_meshes) {
      this.setFragmentMeshes(summary.fragment_meshes);
    }
  }

  getSimulationId(): string {
    return this.summary.simulation_id || 'latest';
  }

  getFrameCount(): number {
    return this.frames.length;
  }

  getKnownFragmentMeshCount(): number {
    return this.cachedFragMeshById.size;
  }

  hasFrameIndex(frameIndex: number): boolean {
    return this.frameIdxSet.has(frameIndex);
  }

  updateSummary(partial: Partial<RustCoupledSimBakeSummaryJson>): void {
    this.summary = {
      ...this.summary,
      ...partial,
      events: partial.events ?? this.summary.events,
    };
    if (partial.fragment_meshes) {
      this.setFragmentMeshes(partial.fragment_meshes);
    }
  }

  setFragmentMeshes(meshes: readonly RustCoupledSimFragmentMeshJson[]): void {
    for (const fm of meshes) {
      if (!fm?.fragment_id) continue;
      this.cachedFragMeshById.set(fm.fragment_id, fm);
      // If a fallback box was created before its mesh arrived, upgrade geometry in-place
      const existingIdx = this.fragIndexById.get(fm.fragment_id);
      if (existingIdx !== undefined) {
        const currentShard = this.shards[existingIdx];
        const parent = this.partById.get(currentShard.parentPartId);
        const mat = getMaterialById(
          parent?.materialId ?? currentShard.materialId ?? 'steel_4140',
          this.materials as MaterialSpec[]
        );
        const realBuilt = buildFragmentGeometryFromRustMesh(
          fm,
          currentShard.parentPartId,
          mat,
          currentShard.mass
        );
        if (realBuilt && realBuilt.indices.length > currentShard.indices.length) {
          this.shards[existingIdx] = realBuilt;
        }
      }
    }
  }

  appendFrame(rf: RustCoupledSimFrameJson): {
    frame: DestructionCacheFrame;
    shardsAdded: boolean;
  } {
    const prevShardCount = this.shards.length;

    // 1. Ingest any newly detached fragment meshes / metadata
    for (const frag of rf.detached_fragments ?? []) {
      if (frag.mesh && !this.cachedFragMeshById.has(frag.fragment_id)) {
        this.cachedFragMeshById.set(frag.fragment_id, frag.mesh);
      }
      if (!this.fragIndexById.has(frag.fragment_id)) {
        const parentPartId =
          frag.source_part_id || frag.parent_part_id || 'unknown_part';
        const parent = this.partById.get(parentPartId);
        const mat = getMaterialById(
          parent?.materialId ?? 'steel_4140',
          this.materials as MaterialSpec[]
        );
        this.fragIndexById.set(frag.fragment_id, this.shards.length);
        const realMesh = frag.mesh ?? this.cachedFragMeshById.get(frag.fragment_id);
        const builtReal = realMesh
          ? buildFragmentGeometryFromRustMesh(
              realMesh,
              parentPartId,
              mat,
              frag.mass_kg ?? 0.004
            )
          : null;
        this.shards.push(
          builtReal ??
            buildSmallFragmentBox(
              frag.fragment_id,
              parentPartId,
              mat,
              frag.bounding_radius_m ?? frag.bounding_radius ?? 0.0025,
              frag.mass_kg ?? 0.004
            )
        );
      }
    }

    const totalShardCount = this.shards.length;
    const shardsAdded = totalShardCount > prevShardCount;

    // If new shards were discovered on this frame, pad earlier cached frames to match
    if (shardsAdded) {
      for (const prevFrame of this.frames) {
        prevFrame.shardPos = resizeFloat32(prevFrame.shardPos, totalShardCount * 3);
        prevFrame.shardQuat = resizeFloat32(prevFrame.shardQuat, totalShardCount * 4);
        prevFrame.shardActive = resizeUint8(prevFrame.shardActive, totalShardCount);
        prevFrame.shardStress = resizeFloat32(prevFrame.shardStress, totalShardCount);
        prevFrame.shardStressMPa = resizeFloat32(
          prevFrame.shardStressMPa,
          totalShardCount
        );
        prevFrame.shardPlasticStrain = resizeFloat32(
          prevFrame.shardPlasticStrain,
          totalShardCount
        );
        prevFrame.shardDamage = resizeFloat32(prevFrame.shardDamage, totalShardCount);
        prevFrame.metrics.totalShardsCount = totalShardCount;
      }
    }

    const targetIdx = rf.frame_index ?? this.frames.length;
    const partStates: Record<string, CachedPartFrameState> = {};
    let framePeakStress = 0;
    let frameMaxPlasticPct = 0;
    let frameMaxDamage = 0;
    let frameMaxDispMm = 0;
    let fracturedPartsCount = 0;
    let intactPartsCount = 0;
    const isMmFrame =
      (rf.length_unit ?? rf.units?.length ?? 'm').toLowerCase() === 'mm';
    const posToMeters = isMmFrame ? 1e-3 : 1.0;

    for (const p of this.parts) {
      if (!p.visible) continue;
      partStates[p.id] = {
        position: [...p.position],
        rotation: [...p.rotation],
        stressRatio: 0,
        intact: true,
        partiallyFractured: false,
      };
    }

    for (const rp of rf.parts) {
      const part = this.partById.get(rp.part_id);
      const mat = getMaterialById(
        part?.materialId ?? 'steel_4140',
        this.materials as MaterialSpec[]
      );
      const partYieldMPa = Math.max(1, mat.yieldStrength / 1e6);

      const maxStress = rp.max_von_mises_mpa ?? 0;
      const maxEpPct = (rp.max_equiv_plastic_strain ?? 0) * 100;
      const maxDmg = rp.max_damage ?? 0;
      const maxDispMm = rp.max_displacement_mm ?? 0;

      if (maxStress > framePeakStress) framePeakStress = maxStress;
      if (maxEpPct > frameMaxPlasticPct) frameMaxPlasticPct = maxEpPct;
      if (maxDmg > frameMaxDamage) frameMaxDamage = maxDmg;
      if (maxDispMm > frameMaxDispMm) frameMaxDispMm = maxDispMm;

      const erodedCount =
        rp.eroded_element_count ?? rp.eroded_element_indices?.length ?? 0;
      const activeCount = rp.active_element_count ?? 1;
      const isFullyBroken = activeCount === 0 && erodedCount > 0;
      const isPartiallyBroken = erodedCount > 0 && activeCount > 0;

      if (isFullyBroken || isPartiallyBroken) fracturedPartsCount++;
      else intactPartsCount++;

      let femSurfaceMesh: CachedFemSurfaceMesh | undefined;
      const surfDeformed = flattenVec3List(rp.surface_positions_m);
      if (
        surfDeformed &&
        surfDeformed.length >= 9 &&
        rp.surface_indices &&
        rp.surface_indices.length >= 3
      ) {
        const surfVertCount = Math.floor(surfDeformed.length / 3);
        const surfIndices = new Uint32Array(rp.surface_indices);
        const surfRefRaw = flattenVec3List(rp.surface_reference_positions_m);
        let surfRef: Float32Array;
        const surfDisp = new Float32Array(surfDeformed.length);

        if (surfRefRaw && surfRefRaw.length === surfDeformed.length) {
          surfRef = surfRefRaw;
          for (let i = 0; i < surfDeformed.length; i++) {
            surfDisp[i] = surfDeformed[i] - surfRef[i];
          }
        } else {
          const flatDispMm = flattenVec3List(rp.displacements_mm);
          surfRef = new Float32Array(surfDeformed.length);
          if (
            flatDispMm &&
            rp.surface_node_indices &&
            rp.surface_node_indices.length === surfVertCount
          ) {
            for (let vi = 0; vi < surfVertCount; vi++) {
              const nid = rp.surface_node_indices[vi];
              const dx =
                nid * 3 + 2 < flatDispMm.length ? flatDispMm[nid * 3] * 1e-3 : 0;
              const dy =
                nid * 3 + 2 < flatDispMm.length
                  ? flatDispMm[nid * 3 + 1] * 1e-3
                  : 0;
              const dz =
                nid * 3 + 2 < flatDispMm.length
                  ? flatDispMm[nid * 3 + 2] * 1e-3
                  : 0;
              surfDisp[vi * 3] = dx;
              surfDisp[vi * 3 + 1] = dy;
              surfDisp[vi * 3 + 2] = dz;
              surfRef[vi * 3] = surfDeformed[vi * 3] - dx;
              surfRef[vi * 3 + 1] = surfDeformed[vi * 3 + 1] - dy;
              surfRef[vi * 3 + 2] = surfDeformed[vi * 3 + 2] - dz;
            }
          } else {
            surfRef.set(surfDeformed);
          }
        }

        femSurfaceMesh = {
          referenceVertices: surfRef,
          deformedVertices: surfDeformed,
          displacements: surfDisp,
          indices: surfIndices,
          vertexCount: surfVertCount,
          triangleCount: Math.floor(surfIndices.length / 3),
          nodalStressMPa: gatherSurfaceScalars(
            rp.nodal_von_mises_mpa,
            rp.surface_node_indices,
            surfVertCount
          ),
          nodalPlasticStrain: gatherSurfaceScalars(
            rp.nodal_equiv_plastic_strain,
            rp.surface_node_indices,
            surfVertCount
          ),
          nodalDamage: gatherSurfaceScalars(
            rp.nodal_damage,
            rp.surface_node_indices,
            surfVertCount
          ),
        };
      }

      let dispM: Float32Array | undefined;
      const flatDispMm = flattenVec3List(rp.displacements_mm);
      if (
        flatDispMm &&
        part &&
        flatDispMm.length === part.mesh.vertices.length
      ) {
        dispM = new Float32Array(flatDispMm.length);
        for (let i = 0; i < flatDispMm.length; i++) {
          dispM[i] = flatDispMm[i] * 1e-3;
        }
      }

      partStates[rp.part_id] = {
        position: part ? [...part.position] : [0, 0, 0],
        rotation: part ? [...part.rotation] : [0, 0, 0],
        stressRatio: maxStress / partYieldMPa,
        intact: !isFullyBroken,
        partiallyFractured: isPartiallyBroken,
        deformedDisplacements: dispM,
        nodalStressMPa: rp.nodal_von_mises_mpa
          ? new Float32Array(rp.nodal_von_mises_mpa)
          : undefined,
        nodalPlasticStrain: rp.nodal_equiv_plastic_strain
          ? new Float32Array(rp.nodal_equiv_plastic_strain)
          : undefined,
        nodalDamage: rp.nodal_damage
          ? new Float32Array(rp.nodal_damage)
          : undefined,
        femSurfaceMesh,
        maxDisplacementMm: maxDispMm,
        maxPlasticStrainPct: maxEpPct,
        maxDamage: maxDmg,
      };
    }

    const shardPos = new Float32Array(totalShardCount * 3);
    const shardQuat = new Float32Array(totalShardCount * 4);
    const shardActive = new Uint8Array(totalShardCount);
    const shardStress = new Float32Array(totalShardCount);
    const shardStressMPa = new Float32Array(totalShardCount);
    const shardPlasticStrain = new Float32Array(totalShardCount);
    const shardDamage = new Float32Array(totalShardCount);
    let activeShardsCount = 0;

    for (const frag of rf.detached_fragments ?? []) {
      const sIdx = this.fragIndexById.get(frag.fragment_id);
      if (sIdx === undefined) continue;
      shardActive[sIdx] = 1;
      activeShardsCount++;
      const posM: [number, number, number] = frag.position_m
        ? frag.position_m
        : frag.position
          ? [
              frag.position[0] * posToMeters,
              frag.position[1] * posToMeters,
              frag.position[2] * posToMeters,
            ]
          : [0, 0, 0];
      const quat = frag.orientation_quat ?? frag.quaternion ?? [0, 0, 0, 1];
      shardPos[sIdx * 3] = posM[0];
      shardPos[sIdx * 3 + 1] = posM[1];
      shardPos[sIdx * 3 + 2] = posM[2];
      shardQuat[sIdx * 4] = quat[0];
      shardQuat[sIdx * 4 + 1] = quat[1];
      shardQuat[sIdx * 4 + 2] = quat[2];
      shardQuat[sIdx * 4 + 3] = quat[3];
      shardStress[sIdx] = 0.85;
      shardStressMPa[sIdx] = framePeakStress;
      shardPlasticStrain[sIdx] = frameMaxPlasticPct / 100;
      shardDamage[sIdx] = 1.0;
    }

    let contactForceN = 0;
    for (const cf of rf.contact_forces ?? []) {
      contactForceN += Math.abs(cf.normal_force_n || 0);
      const pt = cf.point_m ?? cf.contact_point;
      if (pt) this.contactPoint = [...pt];
    }

    const eb = rf.energy_balance;
    const kineticJ = eb?.kinetic_j ?? 0;
    const elasticJ = eb?.elastic_j ?? eb?.elastic_strain_j ?? 0;
    const plasticJ = eb?.plastic_j ?? eb?.plastic_dissipation_j ?? 0;
    const fractureJ = eb?.fracture_j ?? eb?.fracture_energy_j ?? 0;

    if (framePeakStress > this.maxStressOverallMPa) {
      this.maxStressOverallMPa = framePeakStress;
    }
    if (frameMaxPlasticPct > this.maxPlasticOverallPct) {
      this.maxPlasticOverallPct = frameMaxPlasticPct;
    }
    if (frameMaxDamage > this.maxDamageOverall) {
      this.maxDamageOverall = frameMaxDamage;
    }
    this.maxEnergyOverallJ = Math.max(
      this.maxEnergyOverallJ,
      kineticJ,
      elasticJ,
      plasticJ,
      fractureJ
    );

    const statusLabel =
      activeShardsCount > 0
        ? 'Отделение фрагментов (FEM)'
        : frameMaxDamage >= 0.35
          ? 'Рост повреждаемости D'
          : framePeakStress >= this.yieldRefMPa
            ? 'Пластическая текучесть'
            : contactForceN > 10
              ? 'Упругий контакт'
              : 'Динамический расчёт HHT-α';

    const frame: DestructionCacheFrame = {
      frameIndex: targetIdx,
      timeSec: rf.time_s,
      partStates,
      shardPos,
      shardQuat,
      shardActive,
      shardStress,
      shardStressMPa,
      shardPlasticStrain,
      shardDamage,
      sparks: new Float32Array(0),
      metrics: {
        peakStressMPa: Number(framePeakStress.toFixed(1)),
        yieldLimitMPa: Number(this.yieldRefMPa.toFixed(0)),
        contactForceN: Number(contactForceN.toFixed(0)),
        kineticEnergyJ: Number(kineticJ.toFixed(4)),
        elasticEnergyJ: Number(elasticJ.toFixed(4)),
        plasticEnergyJ: Number(plasticJ.toFixed(4)),
        fractureEnergyJ: Number(fractureJ.toFixed(4)),
        maxPlasticStrainPct: Number(frameMaxPlasticPct.toFixed(2)),
        maxDamage: Number(frameMaxDamage.toFixed(3)),
        maxDisplacementMm: Number(frameMaxDispMm.toFixed(3)),
        activeShardsCount,
        totalShardsCount: totalShardCount,
        fracturedPartsCount,
        intactPartsCount,
        statusLabel,
      },
    };

    if (this.frameIdxSet.has(targetIdx)) {
      const existingPos = this.frames.findIndex((f) => f.frameIndex === targetIdx);
      if (existingPos >= 0) {
        this.frames[existingPos] = frame;
      }
    } else {
      this.frameIdxSet.add(targetIdx);
      if (
        this.frames.length === 0 ||
        targetIdx > this.frames[this.frames.length - 1].frameIndex
      ) {
        this.frames.push(frame);
      } else {
        this.frames.push(frame);
        this.frames.sort((a, b) => a.frameIndex - b.frameIndex);
      }
    }

    return { frame, shardsAdded };
  }

  buildCache(): DestructionCache {
    const durationSec =
      this.summary.duration_s ??
      (this.frames.length > 0
        ? this.frames[this.frames.length - 1].timeSec
        : this.config.durationSec);
    const totalFrames = Math.max(0, this.frames.length - 1);
    const fps =
      durationSec > 1e-6 && totalFrames > 0
        ? Math.max(10, Math.round(totalFrames / durationSec))
        : this.config.fps;

    const events: DestructionTimelineEvent[] = (this.summary.events ?? []).map(
      (e) => {
        const rawType = e.kind ?? e.event_type ?? 'contact';
        return {
          frameIndex:
            e.frame_index ??
            Math.max(
              0,
              Math.min(
                totalFrames,
                Math.round(
                  (e.time_s / Math.max(1e-6, durationSec)) * totalFrames
                )
              )
            ),
          timeSec: e.time_s,
          type: mapRustEventType(rawType),
          label: e.detail || e.description || rawType,
          partId: e.part_id,
          stressMPa: e.value,
        };
      }
    );

    let cacheBytes = 0;
    for (const s of this.shards) {
      cacheBytes +=
        s.vertices.byteLength + s.normals.byteLength + s.indices.byteLength;
    }
    for (const f of this.frames) {
      cacheBytes +=
        f.shardPos.byteLength +
        f.shardQuat.byteLength +
        f.shardActive.byteLength +
        f.shardStress.byteLength;
      for (const ps of Object.values(f.partStates)) {
        cacheBytes +=
          (ps.deformedDisplacements?.byteLength ?? 0) +
          (ps.nodalStressMPa?.byteLength ?? 0) +
          (ps.nodalPlasticStrain?.byteLength ?? 0) +
          (ps.nodalDamage?.byteLength ?? 0);
        if (ps.femSurfaceMesh) {
          cacheBytes +=
            ps.femSurfaceMesh.referenceVertices.byteLength +
            ps.femSurfaceMesh.deformedVertices.byteLength +
            ps.femSurfaceMesh.displacements.byteLength +
            ps.femSurfaceMesh.indices.byteLength +
            (ps.femSurfaceMesh.nodalStressMPa?.byteLength ?? 0) +
            (ps.femSurfaceMesh.nodalPlasticStrain?.byteLength ?? 0) +
            (ps.femSurfaceMesh.nodalDamage?.byteLength ?? 0);
        }
      }
    }

    return {
      id: `rust_${this.summary.simulation_id || 'latest'}`,
      createdAt: new Date().toISOString(),
      solverBackend: 'rust_fem_f64',
      solverLabel: 'Rust FEM f64',
      rustSimulationId: this.summary.simulation_id,
      config: { ...this.config, durationSec, fps },
      fps,
      totalFrames,
      durationSec,
      floorY: this.floorY,
      contactPoint: [...this.contactPoint],
      bakeTimeMs: this.summary.elapsed_ms ?? this.summary.bake_time_ms ?? 1,
      cacheBytes: Math.max(256, cacheBytes),
      shards: [...this.shards],
      frames: [...this.frames],
      events,
      maxStressOverallMPa: Math.max(
        1,
        Number(this.maxStressOverallMPa.toFixed(1))
      ),
      yieldStrengthRefMPa: Number(this.yieldRefMPa.toFixed(0)),
      maxPlasticStrainOverallPct: Math.max(
        0.1,
        Number(this.maxPlasticOverallPct.toFixed(2))
      ),
      maxDamageOverall: Number(Math.max(0.01, this.maxDamageOverall).toFixed(3)),
      maxEnergyOverallJ: Number(this.maxEnergyOverallJ.toFixed(4)),
    };
  }
}
