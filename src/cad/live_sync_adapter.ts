import {
  DagNode,
  DagNodeType,
  FeaSimulationResult,
  KinematicJoint,
  MaterialSpec,
  McpToolExecution,
} from '../types/cad';
import {
  DEFAULT_MATERIALS,
  normalizeMaterialSpec,
  resolveMaterialAlias,
} from './materials';

export const MCP_LIVE_PRESET_ID = '__mcp_live__';

export interface LiveToolEvent {
  tool_name: string;
  status: string;
  summary: string;
  timestamp_ms: number;
  duration_ms: number;
  params_preview?: Record<string, unknown> | null;
}

export interface LiveDagNode {
  id: string;
  name: string;
  op_type: string;
  enabled: boolean;
  params: Record<string, unknown>;
}

export interface LivePartFeaSummary {
  max_von_mises_mpa: number;
  yield_strength_mpa: number;
  safety_factor: number;
  max_displacement_mm: number;
  verdict: string;
  node_count: number;
  element_count: number;
  element_order: number;
  is_stale: boolean;
}

export interface LiveMaterialInfo {
  name: string;
  youngs_modulus_gpa: number;
  poissons_ratio: number;
  yield_strength_mpa: number;
  density_kg_m3: number;
  failure_mode: string;
}

export interface LiveScenePart {
  id: string;
  safe_id: string;
  name: string;
  revision: number;
  is_fixed: boolean;
  material: LiveMaterialInfo;
  triangle_count: number;
  vertex_count: number;
  volume_m3: number;
  mass_kg: number;
  bounding_box_m: [[number, number, number], [number, number, number]];
  stl_file?: string | null;
  dag: LiveDagNode[];
  fea?: LivePartFeaSummary | null;
}

export interface LiveSceneJoint {
  id: string;
  part_a: string;
  part_b: string;
  joint_type: string;
  anchor_a_m?: [number, number, number] | null;
  anchor_b_m?: [number, number, number] | null;
  axis?: [number, number, number] | null;
  limits?: [number, number] | null;
  ratio?: number | null;
  motor_target_velocity?: number | null;
  motor_max_force?: number | null;
}

export interface LivePhysicsSample {
  time_s: number;
  position_m: [number, number, number];
  orientation_quat: [number, number, number, number];
  linear_velocity_m_s: [number, number, number];
  angular_velocity_rad_s: [number, number, number];
}

export interface LivePhysicsPartTrajectory {
  part_id: string;
  samples: LivePhysicsSample[];
}

export interface LivePhysicsCollision {
  time_s: number;
  body_a: string;
  body_b: string;
  event_type: string;
  contact_normal?: [number, number, number] | null;
  max_impulse?: number | null;
}

export interface LivePhysicsSimulation {
  success: boolean;
  duration_s: number;
  step_count: number;
  step_size_s: number;
  peak_joint_forces: Record<string, number>;
  collisions: LivePhysicsCollision[];
  trajectories: LivePhysicsPartTrajectory[];
}

export interface LiveCoupledSimulationSummary {
  simulation_id: string;
  accuracy_level: string;
  duration_s: number;
  frame_count: number;
  peak_von_mises_mpa: number;
  peak_damage: number;
  detached_fragments_count: number;
  events_count: number;
  data_file: string;
}

export interface LiveSceneSnapshot {
  schema_version: string;
  revision: number;
  timestamp_ms: number;
  pid: number;
  source: string;
  length_unit: string;
  last_tool?: LiveToolEvent | null;
  recent_tools: LiveToolEvent[];
  parts: LiveScenePart[];
  joints: LiveSceneJoint[];
  physics_simulation?: LivePhysicsSimulation | null;
  coupled_simulation?: LiveCoupledSimulationSummary | null;
}

export type LiveSyncSourceKind = 'manifest' | 'legacy_stl' | 'rust_http' | 'none';

export interface LiveSyncApiResponse {
  active: boolean;
  sourceKind: LiveSyncSourceKind;
  snapshot: LiveSceneSnapshot | null;
}

const VALID_DAG_NODE_TYPES: ReadonlySet<DagNodeType> = new Set<DagNodeType>([
  'box',
  'cylinder',
  'sphere',
  'cone',
  'polygon_prism',
  'spur_gear',
  'revolve',
  'wheel_rim',
  'helix',
  'cut_hole',
  'cut_helical_groove',
  'boolean_union',
  'boolean_difference',
  'boolean_intersection',
  'transform',
  'stl_import',
]);

export function sanitizeLivePartId(partId: string): string {
  const sanitized = String(partId || '').replace(/[^a-zA-Z0-9_-]/g, '_');
  return sanitized.length > 0 ? sanitized : 'part';
}

export function mapRustMaterialToWebStudio(
  info: LiveMaterialInfo | undefined | null,
  existingMaterials: readonly MaterialSpec[] = DEFAULT_MATERIALS
): { materialId: string; customMaterial?: MaterialSpec } {
  if (!info || !info.name) {
    return { materialId: 'steel_1045' };
  }

  const resolvedAlias = resolveMaterialAlias(info.name);
  const exactExisting = existingMaterials.find(
    (m) =>
      m.id === info.name ||
      m.id === resolvedAlias ||
      m.name.toLowerCase() === info.name.toLowerCase()
  );
  if (exactExisting) {
    return { materialId: exactExisting.id };
  }

  const normalizedId = sanitizeLivePartId(info.name.toLowerCase());
  const byNormalizedId = existingMaterials.find((m) => m.id === normalizedId);
  if (byNormalizedId) {
    return { materialId: byNormalizedId.id };
  }

  const customMaterial = normalizeMaterialSpec({
    id: normalizedId,
    name: info.name,
    category: 'Custom',
    density_kg_m3: info.density_kg_m3,
    youngs_modulus_gpa: info.youngs_modulus_gpa,
    poissons_ratio: info.poissons_ratio,
    yield_strength_mpa: info.yield_strength_mpa,
    failure_mode: info.failure_mode,
  });

  return {
    materialId: customMaterial.id,
    customMaterial,
  };
}

export function mapRustDagNodesToWebStudio(
  nodes: readonly LiveDagNode[] | undefined | null,
  fallbackLabel = 'MCP Live Geometry'
): DagNode[] {
  if (!nodes || nodes.length === 0) {
    return [
      {
        id: 'live_stl_0',
        name: fallbackLabel,
        type: 'stl_import',
        params: { source: 'mcp_live_sync' },
        enabled: true,
      },
    ];
  }

  return nodes.map((node, index) => {
    const rawType = String(node.op_type || 'stl_import') as DagNodeType;
    const mappedType: DagNodeType = VALID_DAG_NODE_TYPES.has(rawType)
      ? rawType
      : 'stl_import';

    const params =
      node.params && typeof node.params === 'object' && !Array.isArray(node.params)
        ? { ...node.params }
        : {};

    return {
      id: node.id || `live_node_${index}`,
      name: node.name || `${mappedType} #${index + 1}`,
      type: mappedType,
      params,
      enabled: node.enabled !== false,
    };
  });
}

export function mapRustJointsToWebStudio(
  joints: readonly LiveSceneJoint[] | undefined | null
): KinematicJoint[] {
  if (!joints || joints.length === 0) return [];

  return joints.map((joint) => {
    const rawType = String(joint.joint_type || 'revolute').toLowerCase();
    const type: KinematicJoint['type'] =
      rawType === 'prismatic' ||
      rawType === 'fixed' ||
      rawType === 'spherical' ||
      rawType === 'gear'
        ? rawType
        : 'revolute';

    const rawTargetVel =
      typeof joint.motor_target_velocity === 'number' &&
      Number.isFinite(joint.motor_target_velocity)
        ? joint.motor_target_velocity
        : 0;

    const motorSpeedRpm =
      type === 'prismatic'
        ? 0
        : Number(((rawTargetVel * 60) / (2 * Math.PI)).toFixed(2));
    const motorLinearSpeedMs = type === 'prismatic' ? rawTargetVel : undefined;
    const motorTorqueNm =
      typeof joint.motor_max_force === 'number' &&
      Number.isFinite(joint.motor_max_force)
        ? joint.motor_max_force
        : 0;

    return {
      id: joint.id,
      name: `${joint.id} (${type})`,
      type,
      partA: joint.part_a || 'world',
      partB: joint.part_b,
      anchor: joint.anchor_a_m ? [...joint.anchor_a_m] : [0, 0, 0],
      anchorB: joint.anchor_b_m ? [...joint.anchor_b_m] : undefined,
      axis: joint.axis ? [...joint.axis] : [0, 0, 1],
      limits: joint.limits ? [...joint.limits] : undefined,
      motorSpeedRpm,
      motorLinearSpeedMs,
      motorTorqueNm,
      motorEnabled:
        Math.abs(rawTargetVel) > 1e-6 ||
        (type === 'prismatic' && motorTorqueNm > 0),
      currentAngleRad: 0,
      gearRatio: typeof joint.ratio === 'number' ? joint.ratio : undefined,
    };
  });
}

export function mapRustFeaSummaryToWebStudio(
  partId: string,
  fea: LivePartFeaSummary | undefined | null
): FeaSimulationResult | null {
  if (!fea) return null;

  const upperVerdict = String(fea.verdict || '').toUpperCase();
  let status: FeaSimulationResult['status'] = 'SAFE';
  if (
    upperVerdict.includes('FAIL') ||
    upperVerdict.includes('YIELD') ||
    fea.safety_factor < 1.0
  ) {
    status = 'FAILURE_YIELD_EXCEEDED';
  } else if (upperVerdict.includes('WARN') || fea.safety_factor < 1.5) {
    status = 'WARNING_HIGH_STRESS';
  }

  return {
    partId,
    nodeCount: fea.node_count,
    tetrahedraCount: fea.element_count,
    maxDisplacementMm: fea.max_displacement_mm,
    maxVonMisesMPa: fea.max_von_mises_mpa,
    minVonMisesMPa: 0,
    yieldStrengthMPa: fea.yield_strength_mpa,
    safetyFactor: fea.safety_factor,
    status,
    solveTimeMs: 0,
    conditions: [],
    elementOrder: fea.element_order === 2 ? 'TET10' : 'TET4',
    warnings: fea.is_stale
      ? ['Результаты МКЭ устарели после изменения геометрии агентом']
      : undefined,
  };
}

export function formatTimestampMsToClock(timestampMs: number): string {
  if (!Number.isFinite(timestampMs) || timestampMs <= 0) {
    return '00:00:00';
  }
  const d = new Date(timestampMs);
  if (Number.isNaN(d.getTime())) return '00:00:00';
  const hh = String(d.getHours()).padStart(2, '0');
  const mm = String(d.getMinutes()).padStart(2, '0');
  const ss = String(d.getSeconds()).padStart(2, '0');
  return `${hh}:${mm}:${ss}`;
}

export function mapLiveToolsToMcpHistory(
  tools: readonly LiveToolEvent[] | undefined | null
): McpToolExecution[] {
  if (!tools || tools.length === 0) return [];

  return tools.map((event, idx) => ({
    id: `live_mcp_${event.timestamp_ms}_${idx}_${event.tool_name}`,
    timestamp: formatTimestampMsToClock(event.timestamp_ms),
    toolName: event.tool_name,
    params:
      event.params_preview && typeof event.params_preview === 'object'
        ? { ...event.params_preview }
        : {},
    response: {
      status: event.status,
      summary: event.summary,
      source: 'mcp_live_agent',
    },
    durationMs: Number.isFinite(event.duration_ms)
      ? Number(event.duration_ms.toFixed(1))
      : 0,
    status: event.status === 'error' ? 'error' : 'success',
  }));
}

/**
 * Decides whether applying a live snapshot should trigger camera auto-fit.
 * Auto-fits when entering live mode or when the set of part IDs changes,
 * while preserving the user's manual orbit camera during incremental tweaks.
 */
export function shouldAutoFitCamera(
  prevPartIds: readonly string[],
  nextPartIds: readonly string[],
  wasLiveMode: boolean
): boolean {
  if (!wasLiveMode) return true;
  if (prevPartIds.length !== nextPartIds.length) return true;
  const prevSorted = [...prevPartIds].sort();
  const nextSorted = [...nextPartIds].sort();
  for (let i = 0; i < prevSorted.length; i++) {
    if (prevSorted[i] !== nextSorted[i]) return true;
  }
  return false;
}

const SIM_TRIGGER_TOOLS = new Set([
  'run_physics_simulation',
  'bake_coupled_simulation',
  'set_joint_motor',
  'sweep_joint',
]);

/**
 * Decides whether a live MCP agent snapshot should automatically hydrate and play
 * an interactive simulation/destruction timeline rather than freezing as a static image.
 */
export function shouldAutoSimulateLiveSnapshot(
  snapshot: LiveSceneSnapshot,
  joints: readonly KinematicJoint[]
): boolean {
  if (snapshot.coupled_simulation && snapshot.coupled_simulation.frame_count > 1) {
    return true;
  }
  if (
    snapshot.physics_simulation &&
    snapshot.physics_simulation.trajectories.some((t) => t.samples.length > 1)
  ) {
    return true;
  }
  if (
    snapshot.last_tool &&
    SIM_TRIGGER_TOOLS.has(snapshot.last_tool.tool_name)
  ) {
    return true;
  }
  if (
    snapshot.recent_tools?.some((t) => SIM_TRIGGER_TOOLS.has(t.tool_name))
  ) {
    return true;
  }
  if (
    joints.some(
      (j) =>
        j.motorEnabled ||
        Math.abs(j.motorTorqueNm) > 0 ||
        (j.type === 'prismatic' && Boolean(j.limits))
    )
  ) {
    return true;
  }
  return false;
}
