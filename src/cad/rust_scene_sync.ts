import type {
  CadPart,
  DagNode,
  DestructionSimConfig,
  KinematicJoint,
  MaterialSpec,
} from '../types/cad';
import { getMaterialById } from './materials';

export type RustAccuracyLevel = 'standard' | 'high' | 'ultra';

export interface RustMcpOperation {
  tool: string;
  args: Record<string, unknown>;
}

export interface RustResolvedEndpoint {
  ok: true;
  /** Empty string '' represents same-origin Express proxy; otherwise normalized origin/base URL. */
  endpointUrl: string;
  isSameOriginProxy: boolean;
  mcpUrl: string;
  coupledSimBaseUrl: string;
  statusUrl: string;
}

export interface RustEndpointError {
  ok: false;
  error: string;
}

export interface RustSceneSyncPlan {
  /** MCP operations executed sequentially after clearing stale parts on the Rust server. */
  operations: RustMcpOperation[];
  /** Active part IDs included in the coupled dynamic FEM bake. */
  activePartIds: string[];
  /** Part IDs synced to the Rust EngineState (including any fixed parts referenced by joints). */
  syncedPartIds: string[];
  /** Warnings about skipped or approximated DAG features (if any). */
  warnings: string[];
  /** JSON body for POST /api/coupled-sim/bake. */
  bakePayload: Record<string, unknown>;
}

/**
 * Canonical mapping from TypeScript built-in material IDs to Rust `MaterialRegistry` canonical IDs.
 */
const TS_TO_RUST_CANONICAL_MATERIAL: Record<string, string> = {
  alu_6061: 'aluminum_6061_t6',
  aluminum_6061_t6: 'aluminum_6061_t6',
  steel_1020: 'steel_structural_aisi1020',
  steel_structural_aisi1020: 'steel_structural_aisi1020',
  steel_1045: 'steel_carbon_aisi1045',
  steel_carbon_aisi1045: 'steel_carbon_aisi1045',
  steel_4140: 'steel_high_strength_4140',
  steel_high_strength_4140: 'steel_high_strength_4140',
  ti_6al_4v: 'titanium_grade5_ti6al4v',
  titanium_grade5_ti6al4v: 'titanium_grade5_ti6al4v',
  brass_c360: 'brass_c360',
  pla_3d: 'pla_filament_3dprint',
  pla_filament_3dprint: 'pla_filament_3dprint',
  petg_3d: 'petg_filament_3dprint',
  petg_filament_3dprint: 'petg_filament_3dprint',
  nylon_pa6: 'nylon_pa12',
  nylon_pa12: 'nylon_pa12',
  borosilicate_glass: 'borosilicate_glass',
};

const STATIC_BENCH_TOKENS = new Set([
  'bench',
  'bed',
  'rig',
  'floor',
  'stand',
  'backplate',
]);

function isStaticSupportBench(part: CadPart): boolean {
  if (part.isFixed) return true;
  const lower = `${part.id} ${part.name}`.toLowerCase();
  if (lower.includes('plate_mount')) return true;
  const tokens = lower.split(/[^a-z0-9а-яё]+/);
  return tokens.some((tok) => STATIC_BENCH_TOKENS.has(tok));
}

/**
 * Validates and resolves a Rust server endpoint URL.
 * - `""`, `"auto"`, `"same-origin"`, or `"/api"` resolves to the same-origin Express proxy (`/api/mcp`, `/api/coupled-sim`).
 * - Direct `http://` or `https://` URLs resolve to `${base}/message` and `${base}/api/coupled-sim`.
 * - Rejects non-HTTP(S) schemes (e.g. `javascript:`, `file:`) and malformed URLs.
 */
export function resolveRustEndpoint(
  rawUrl: string
): RustResolvedEndpoint | RustEndpointError {
  const trimmed = (rawUrl || '').trim();
  if (
    trimmed === '' ||
    trimmed === 'auto' ||
    trimmed === 'same-origin' ||
    trimmed === '/api'
  ) {
    return {
      ok: true,
      endpointUrl: '',
      isSameOriginProxy: true,
      mcpUrl: '/api/mcp',
      coupledSimBaseUrl: '/api/coupled-sim',
      statusUrl: '/api/rust/status',
    };
  }

  let parsed: URL;
  try {
    parsed = new URL(trimmed);
  } catch {
    return {
      ok: false,
      error: `Некорректный URL сервера: "${trimmed}". Укажите http://127.0.0.1:8080 или оставьте пустым для /api.`,
    };
  }

  if (parsed.protocol !== 'http:' && parsed.protocol !== 'https:') {
    return {
      ok: false,
      error: `Неподдерживаемая схема "${parsed.protocol}". Разрешены только http:// и https://.`,
    };
  }

  const base = `${parsed.origin}${parsed.pathname}`.replace(/\/+$/, '');
  return {
    ok: true,
    endpointUrl: base,
    isSameOriginProxy: false,
    mcpUrl: `${base}/message`,
    coupledSimBaseUrl: `${base}/api/coupled-sim`,
    statusUrl: `${base}/message`,
  };
}

/**
 * Maps a Web Studio material ID to its canonical Rust registry ID, plus an optional
 * `define_material` MCP operation payload if the material is custom (not one of the 10 built-ins).
 */
export function resolveRustMaterialForPart(
  materialId: string,
  materials: readonly MaterialSpec[]
): {
  rustMaterialId: string;
  defineMaterialOp?: RustMcpOperation;
} {
  const resolved = getMaterialById(materialId, materials as MaterialSpec[]);
  const key = resolved.id.trim().toLowerCase();
  const canonical = TS_TO_RUST_CANONICAL_MATERIAL[key];
  if (canonical) {
    return { rustMaterialId: canonical };
  }

  const customId = resolved.id.replace(/[^a-zA-Z0-9_-]/g, '_') || 'custom_material';
  return {
    rustMaterialId: customId,
    defineMaterialOp: {
      tool: 'define_material',
      args: {
        id: customId,
        name: resolved.name,
        young_modulus_gpa: Math.max(0.01, resolved.youngsModulus / 1e9),
        poisson_ratio: Math.min(0.49, Math.max(0.01, resolved.poissonsRatio)),
        yield_strength_mpa: Math.max(0.1, resolved.yieldStrength / 1e6),
        density_kg_m3: Math.max(10, resolved.density),
        ultimate_strength_pa: Math.max(
          resolved.yieldStrength,
          resolved.ultimateStrengthPa || resolved.yieldStrength * 1.2
        ),
        hardening_modulus_pa: Math.max(0, resolved.hardeningModulusPa || 0),
        fracture_strain: Math.max(0.001, resolved.fractureStrain || 0.12),
        fracture_energy_j_m2: Math.max(1, resolved.fractureEnergyJM2 || 15000),
        friction_coeff: Math.max(0, resolved.frictionCoeff ?? 0.2),
        failure_mode: resolved.failureMode === 'brittle' ? 'brittle' : 'ductile',
      },
    },
  };
}

function radToDeg(rad: number): number {
  return (rad * 180) / Math.PI;
}

function vecRadToDeg(rot: readonly [number, number, number]): [number, number, number] {
  return [radToDeg(rot[0]), radToDeg(rot[1]), radToDeg(rot[2])];
}

function isNonZeroVec3(v: readonly [number, number, number], eps = 1e-9): boolean {
  return Math.abs(v[0]) > eps || Math.abs(v[1]) > eps || Math.abs(v[2]) > eps;
}

function isNonUnitScale(s: readonly [number, number, number], eps = 1e-6): boolean {
  return (
    Math.abs(s[0] - 1) > eps ||
    Math.abs(s[1] - 1) > eps ||
    Math.abs(s[2] - 1) > eps
  );
}

/**
 * Extracts spur gear pitch/outer radii (in meters) from a part's active `spur_gear` DAG node, if any.
 */
function extractSpurGearRadiiM(part: CadPart): {
  teeth: number;
  moduleM: number;
  pitchRadiusM: number;
  outerRadiusM: number;
} | null {
  const gearNode = part.dag.find((n) => n.enabled && n.type === 'spur_gear');
  if (!gearNode) return null;
  const p = gearNode.params || {};
  const teeth = Math.max(6, Math.round(Number(p.teeth ?? 20)));
  const rawModule = Number(p.module ?? 2.0);
  // Web Studio stores spur_gear.module in millimeters (e.g. 2.2), whereas length_unit='m' expects meters (0.0022).
  const moduleM = rawModule > 0.1 ? rawModule * 0.001 : Math.max(1e-4, rawModule);
  const pitchRadiusM = (teeth * moduleM) / 2;
  const outerRadiusM = pitchRadiusM + moduleM;
  return { teeth, moduleM, pitchRadiusM, outerRadiusM };
}

/**
 * Translates a single Web Studio `DagNode` into a native Rust MCP tool call
 * using `length_unit: "m"` (meters and degrees).
 */
export function mapDagNodeToRustOperation(
  partId: string,
  node: DagNode
): RustMcpOperation | null {
  if (!node.enabled) return null;
  const p = node.params || {};

  switch (node.type) {
    case 'box': {
      const params: Record<string, unknown> = {
        width: Number(p.width ?? 0.04),
        height: Number(p.height ?? 0.04),
        depth: Number(p.depth ?? 0.04),
      };
      if (Array.isArray(p.position)) params.position = p.position;
      if (Array.isArray(p.rotation_deg)) params.rotation_deg = p.rotation_deg;
      return {
        tool: 'add_feature_primitive',
        args: { part_id: partId, primitive_type: 'box', length_unit: 'm', params },
      };
    }

    case 'cylinder': {
      const params: Record<string, unknown> = {
        radius: Number(p.radius ?? 0.015),
        height: Number(p.height ?? 0.04),
      };
      if (p.segments !== undefined) params.segments = Math.round(Number(p.segments));
      if (Array.isArray(p.position)) params.position = p.position;
      if (Array.isArray(p.rotation_deg)) params.rotation_deg = p.rotation_deg;
      return {
        tool: 'add_feature_primitive',
        args: {
          part_id: partId,
          primitive_type: 'cylinder',
          length_unit: 'm',
          params,
        },
      };
    }

    case 'sphere': {
      const params: Record<string, unknown> = {
        radius: Number(p.radius ?? 0.02),
      };
      if (p.sectors !== undefined) params.sectors = Math.round(Number(p.sectors));
      if (p.stacks !== undefined) params.stacks = Math.round(Number(p.stacks));
      if (Array.isArray(p.position)) params.position = p.position;
      return {
        tool: 'add_feature_primitive',
        args: { part_id: partId, primitive_type: 'sphere', length_unit: 'm', params },
      };
    }

    case 'cone': {
      const params: Record<string, unknown> = {
        radius_bottom: Number(p.radius_bottom ?? p.radiusBottom ?? 0.02),
        radius_top: Number(p.radius_top ?? p.radiusTop ?? 0.0),
        height: Number(p.height ?? 0.04),
      };
      if (p.segments !== undefined) params.segments = Math.round(Number(p.segments));
      if (Array.isArray(p.position)) params.position = p.position;
      if (Array.isArray(p.rotation_deg)) params.rotation_deg = p.rotation_deg;
      return {
        tool: 'add_feature_primitive',
        args: { part_id: partId, primitive_type: 'cone', length_unit: 'm', params },
      };
    }

    case 'polygon_prism': {
      const params: Record<string, unknown> = {
        height: Number(p.height ?? 0.02),
        sides: Math.max(3, Math.round(Number(p.sides ?? 6))),
      };
      if (p.width_across_flats !== undefined) {
        params.width_across_flats = Number(p.width_across_flats);
      } else if (p.outer_radius !== undefined || p.radius !== undefined) {
        params.radius = Number(p.outer_radius ?? p.radius);
      }
      if (p.chamfer !== undefined || p.chamfer_radius !== undefined) {
        params.chamfer = Number(p.chamfer ?? p.chamfer_radius);
      }
      if (Array.isArray(p.position)) params.position = p.position;
      if (Array.isArray(p.rotation_deg)) params.rotation_deg = p.rotation_deg;
      return {
        tool: 'add_feature_primitive',
        args: {
          part_id: partId,
          primitive_type: 'polygon_prism',
          length_unit: 'm',
          params,
        },
      };
    }

    case 'spur_gear': {
      const rawModule = Number(p.module ?? 2.0);
      // Web Studio specifies module in mm (e.g. 2.0 or 2.2) while all other dimensions are in meters.
      const moduleM = rawModule > 0.1 ? rawModule * 0.001 : Math.max(1e-4, rawModule);
      const params: Record<string, unknown> = {
        module: Number(moduleM.toFixed(6)),
        teeth: Math.max(6, Math.round(Number(p.teeth ?? 20))),
        face_width: Number(p.face_width ?? 0.015),
        pressure_angle_deg: Number(p.pressure_angle_deg ?? 20),
      };
      if (p.bore_diameter !== undefined) params.bore_diameter = Number(p.bore_diameter);
      if (p.bore_radius !== undefined) params.bore_radius = Number(p.bore_radius);
      if (p.hub_diameter !== undefined) params.hub_diameter = Number(p.hub_diameter);
      if (p.hub_length !== undefined) params.hub_length = Number(p.hub_length);
      if (p.keyway_width !== undefined) params.keyway_width = Number(p.keyway_width);
      if (p.keyway_depth !== undefined) params.keyway_depth = Number(p.keyway_depth);
      if (Array.isArray(p.position)) params.position = p.position;
      if (Array.isArray(p.rotation_deg)) params.rotation_deg = p.rotation_deg;
      return {
        tool: 'add_feature_primitive',
        args: {
          part_id: partId,
          primitive_type: 'spur_gear',
          length_unit: 'm',
          params,
        },
      };
    }

    case 'revolve': {
      const params: Record<string, unknown> = {
        profile: p.profile ?? [
          [0.01, -0.02],
          [0.02, -0.02],
          [0.02, 0.02],
          [0.01, 0.02],
        ],
        angle_deg: Number(p.angle_deg ?? 360),
      };
      if (p.inner_radius !== undefined) params.inner_radius = Number(p.inner_radius);
      if (p.segments !== undefined) params.segments = Math.round(Number(p.segments));
      if (Array.isArray(p.position)) params.position = p.position;
      if (Array.isArray(p.rotation_deg)) params.rotation_deg = p.rotation_deg;
      return {
        tool: 'add_feature_primitive',
        args: {
          part_id: partId,
          primitive_type: 'revolve',
          length_unit: 'm',
          params,
        },
      };
    }

    case 'wheel_rim': {
      const params: Record<string, unknown> = { ...p };
      return {
        tool: 'add_feature_primitive',
        args: {
          part_id: partId,
          primitive_type: 'wheel_rim',
          length_unit: 'm',
          params,
        },
      };
    }

    case 'cut_hole': {
      const args: Record<string, unknown> = {
        part_id: partId,
        length_unit: 'm',
        position: Array.isArray(p.position) ? p.position : [0, 0, 0],
        normal: Array.isArray(p.normal)
          ? p.normal
          : Array.isArray(p.direction)
            ? p.direction
            : [0, 0, -1],
        radius: Number(p.radius ?? (p.diameter ? Number(p.diameter) * 0.5 : 0.005)),
      };
      if (p.through === true || p.depth === undefined) {
        args.through = true;
      } else {
        args.depth = Number(p.depth);
      }
      if (p.keyway_width !== undefined) args.keyway_width = Number(p.keyway_width);
      if (p.keyway_depth !== undefined) args.keyway_depth = Number(p.keyway_depth);
      return { tool: 'cut_hole', args };
    }

    case 'cut_helical_groove': {
      return {
        tool: 'cut_helical_groove',
        args: {
          part_id: partId,
          length_unit: 'm',
          ...p,
        },
      };
    }

    case 'transform': {
      const args: Record<string, unknown> = {
        part_id: partId,
        length_unit: 'm',
      };
      if (Array.isArray(p.translation)) args.translation = p.translation;
      if (Array.isArray(p.rotation_deg)) {
        args.rotation_deg = p.rotation_deg;
      } else if (Array.isArray(p.rotation)) {
        args.rotation_deg = vecRadToDeg(
          p.rotation as [number, number, number]
        );
      }
      if (Array.isArray(p.scale)) args.scale = p.scale;
      return { tool: 'transform_part', args };
    }

    case 'boolean_union':
    case 'boolean_difference':
    case 'boolean_intersection': {
      if (typeof p.tool_part_id === 'string' && p.tool_part_id) {
        return {
          tool: node.type,
          args: {
            target_part_id: partId,
            tool_part_id: p.tool_part_id,
          },
        };
      }
      return null;
    }

    case 'stl_import':
      return null;
  }
}

/**
 * Pure planner that translates the active Web Studio scene (`parts`, `joints`, `materials`)
 * and `DestructionSimConfig` into:
 * 1. An exact ordered sequence of Rust MCP operations (`set_units`, `define_material`,
 *    `create_part`, `add_feature_primitive`/`cut_hole`, `transform_part`, `create_joint`,
 *    `set_joint_motor`), and
 * 2. The `/api/coupled-sim/bake` JSON payload ready for the Rust `HHT-α` dynamic FEM solver.
 */
export function buildRustSceneSyncPlan(
  parts: readonly CadPart[],
  joints: readonly KinematicJoint[],
  materials: readonly MaterialSpec[],
  config: DestructionSimConfig,
  accuracyLevel: RustAccuracyLevel = 'standard'
): RustSceneSyncPlan {
  const operations: RustMcpOperation[] = [];
  const warnings: string[] = [];

  // Always switch Rust EngineState to meters so coordinates, radii, and anchors match 1:1.
  operations.push({
    tool: 'set_units',
    args: { length_unit: 'm' },
  });

  const visibleParts = parts.filter((p) => p.visible);
  const destructibleParts = visibleParts.filter((p) => !isStaticSupportBench(p));
  const activeParts =
    destructibleParts.length > 0 ? destructibleParts : visibleParts.slice(0, 2);
  const activePartIds = activeParts.map((p) => p.id);
  const activePartIdSet = new Set(activePartIds);

  // Compute effective world positions for synced parts.
  // In counter_clash, the web preset `gears_clash_destruction` starts with a 2.4 mm air gap
  // (relying on the TS preview's sliding carriage). For fixed-shaft HHT-α FEM, shift the
  // two gear axes to their true involute meshing pitch distance a = R1 + R2 so teeth engage.
  const syncedPositions = new Map<string, [number, number, number]>();
  for (const p of visibleParts) {
    syncedPositions.set(p.id, [...p.position] as [number, number, number]);
  }

  const gearParts = activeParts
    .map((p) => ({ part: p, gear: extractSpurGearRadiiM(p) }))
    .filter((entry): entry is { part: CadPart; gear: NonNullable<ReturnType<typeof extractSpurGearRadiiM>> } => entry.gear !== null);

  if (gearParts.length >= 2 && config.scenarioMode === 'counter_clash') {
    const gA = gearParts[0];
    const gB = gearParts[1];
    const pA = syncedPositions.get(gA.part.id)!;
    const pB = syncedPositions.get(gB.part.id)!;
    const dx = pB[0] - pA[0];
    const dy = pB[1] - pA[1];
    const dist = Math.hypot(dx, dy);
    const outerSum = gA.gear.outerRadiusM + gB.gear.outerRadiusM;
    const pitchSum = gA.gear.pitchRadiusM + gB.gear.pitchRadiusM;
    if (dist > outerSum + 1e-4) {
      const ux = dist > 1e-9 ? dx / dist : 1;
      const uy = dist > 1e-9 ? dy / dist : 0;
      const midX = 0.5 * (pA[0] + pB[0]);
      const midY = 0.5 * (pA[1] + pB[1]);
      // Slight 0.15 mm clearance above nominal pitchSum prevents initial coarse-mesh overlap at t=0
      const targetHalf = 0.5 * (pitchSum + 0.00015);
      syncedPositions.set(gA.part.id, [midX - ux * targetHalf, midY - uy * targetHalf, pA[2]]);
      syncedPositions.set(gB.part.id, [midX + ux * targetHalf, midY + uy * targetHalf, pB[2]]);
    }
  }

  // 1. Register any custom materials once
  const definedCustomMats = new Set<string>();
  const rustMatByPart = new Map<string, string>();
  for (const part of activeParts) {
    const { rustMaterialId, defineMaterialOp } = resolveRustMaterialForPart(
      part.materialId,
      materials
    );
    rustMatByPart.set(part.id, rustMaterialId);
    if (defineMaterialOp && !definedCustomMats.has(rustMaterialId)) {
      definedCustomMats.add(rustMaterialId);
      operations.push(defineMaterialOp);
    }
  }

  // 2. Create every active part, replay its DAG features, and apply world transform
  const syncedPartIds: string[] = [];
  for (const part of activeParts) {
    const rustMatId = rustMatByPart.get(part.id) ?? 'steel_carbon_aisi1045';
    operations.push({
      tool: 'create_part',
      args: {
        part_id: part.id,
        name: part.name || part.id,
        material: rustMatId,
        is_fixed: Boolean(part.isFixed),
      },
    });
    syncedPartIds.push(part.id);

    let addedGeometry = false;
    for (const node of part.dag) {
      const op = mapDagNodeToRustOperation(part.id, node);
      if (op) {
        operations.push(op);
        if (op.tool === 'add_feature_primitive') {
          addedGeometry = true;
        }
      } else if (node.enabled && node.type === 'stl_import') {
        warnings.push(
          `Деталь "${part.name}" загружена из STL — для расчёта FEM заменена эквивалентным габаритным телом.`
        );
      }
    }

    // Fallback: if part had only an STL import or unsupported DAG, build an AABB box primitive
    // so Rust FEM never rejects the part for having an empty mesh.
    if (!addedGeometry) {
      const bb = part.mesh.boundingBox;
      const w = Math.max(0.005, bb.max[0] - bb.min[0]);
      const h = Math.max(0.005, bb.max[1] - bb.min[1]);
      const d = Math.max(0.005, bb.max[2] - bb.min[2]);
      const cx = 0.5 * (bb.min[0] + bb.max[0]);
      const cy = 0.5 * (bb.min[1] + bb.max[1]);
      const cz = 0.5 * (bb.min[2] + bb.max[2]);
      operations.push({
        tool: 'add_feature_primitive',
        args: {
          part_id: part.id,
          primitive_type: 'box',
          length_unit: 'm',
          params: {
            width: w,
            height: h,
            depth: d,
            position: [cx, cy, cz],
          },
        },
      });
    }

    const pos = syncedPositions.get(part.id) ?? part.position;
    const rot = part.rotation;
    const scale = part.scale;
    if (isNonZeroVec3(pos) || isNonZeroVec3(rot) || isNonUnitScale(scale)) {
      const transformArgs: Record<string, unknown> = {
        part_id: part.id,
        length_unit: 'm',
      };
      if (isNonZeroVec3(pos)) transformArgs.translation = pos;
      if (isNonZeroVec3(rot)) transformArgs.rotation_deg = vecRadToDeg(rot);
      if (isNonUnitScale(scale)) transformArgs.scale = scale;
      operations.push({
        tool: 'transform_part',
        args: transformArgs,
      });
    }
  }

  // 3. Sync kinematic joints and motors (mapping static bench references to "ground")
  const resolveJointPart = (pid: string): string => {
    if (!pid || pid === 'ground' || pid === 'world') return 'ground';
    if (activePartIdSet.has(pid)) return pid;
    return 'ground';
  };

  const partsWithJoints = new Set<string>();
  let hasGearJoint = false;
  let syncedJointCounter = 0;

  for (const joint of joints) {
    const partA = resolveJointPart(joint.partA);
    const partB = resolveJointPart(joint.partB);
    if (partA === partB) continue;

    const jointId = joint.id || `joint_sync_${++syncedJointCounter}`;
    const targetPartId = partB !== 'ground' ? partB : partA;
    const targetPos = syncedPositions.get(targetPartId) ?? joint.anchor;

    const createArgs: Record<string, unknown> = {
      joint_id: jointId,
      joint_type: joint.type,
      part_a: partA,
      part_b: partB,
      length_unit: 'm',
      anchor_a: partA === 'ground' ? targetPos : [0, 0, 0],
      anchor_b: partB === 'ground' ? targetPos : [0, 0, 0],
      axis: joint.axis,
    };

    if (joint.type === 'gear') {
      hasGearJoint = true;
      if (joint.gearRatio !== undefined && Number.isFinite(joint.gearRatio)) {
        createArgs.ratio = joint.gearRatio;
      }
      createArgs.axis_a = joint.axis;
      createArgs.axis_b = joint.axis;
    }
    if (joint.limits && joint.limits.length === 2) {
      createArgs.limits = joint.limits;
    }

    operations.push({ tool: 'create_joint', args: createArgs });
    if (partA !== 'ground') partsWithJoints.add(partA);
    if (partB !== 'ground') partsWithJoints.add(partB);

    // Override motor speeds & max torque with current DestructionSimConfig when driving revolute/prismatic joints
    if (joint.type === 'revolute' || joint.type === 'prismatic') {
      const destrIdx = activePartIds.indexOf(targetPartId);
      let targetVelRadS = (joint.motorSpeedRpm * 2 * Math.PI) / 60;
      let maxTorqueNm = Math.max(0.5, joint.motorTorqueNm || config.impactTorqueNm);
      let shouldDrive = joint.motorEnabled;

      if (destrIdx === 0 && config.scenarioMode !== 'impact_crush') {
        targetVelRadS = (config.motorRpmLeft * 2 * Math.PI) / 60;
        maxTorqueNm = Math.max(1, config.impactTorqueNm);
        shouldDrive = true;
      } else if (destrIdx === 1 && config.scenarioMode !== 'impact_crush') {
        if (config.scenarioMode === 'torque_overload') {
          // Driven wheel resists with heavy braking torque (zero target velocity, capped braking torque)
          targetVelRadS = 0;
          maxTorqueNm = Math.max(1, config.impactTorqueNm * 0.85);
          shouldDrive = true;
        } else {
          targetVelRadS = (config.motorRpmRight * 2 * Math.PI) / 60;
          maxTorqueNm = Math.max(1, config.impactTorqueNm);
          shouldDrive = true;
        }
      }

      if (shouldDrive) {
        operations.push({
          tool: 'set_joint_motor',
          args: {
            joint_id: jointId,
            target_velocity: Number(targetVelRadS.toFixed(6)),
            max_force: Number(maxTorqueNm.toFixed(4)),
          },
        });
      }
    }
  }

  // Synthesize revolute shaft bearings + gear coupling metadata if a gear pair had no explicit joints
  if (gearParts.length >= 2 && config.scenarioMode !== 'impact_crush') {
    const gA = gearParts[0];
    const gB = gearParts[1];
    for (const [idx, gEntry] of [gA, gB].entries()) {
      if (!partsWithJoints.has(gEntry.part.id)) {
        const jId = `auto_rev_${gEntry.part.id}`;
        const pos = syncedPositions.get(gEntry.part.id)!;
        operations.push({
          tool: 'create_joint',
          args: {
            joint_id: jId,
            joint_type: 'revolute',
            part_a: 'ground',
            part_b: gEntry.part.id,
            length_unit: 'm',
            anchor_a: pos,
            anchor_b: [0, 0, 0],
            axis: [0, 0, 1],
          },
        });
        partsWithJoints.add(gEntry.part.id);
        const rpm = idx === 0 ? config.motorRpmLeft : config.motorRpmRight;
        const targetVel =
          idx === 1 && config.scenarioMode === 'torque_overload'
            ? 0
            : (rpm * 2 * Math.PI) / 60;
        const maxEffort =
          idx === 1 && config.scenarioMode === 'torque_overload'
            ? config.impactTorqueNm * 0.85
            : config.impactTorqueNm;
        operations.push({
          tool: 'set_joint_motor',
          args: {
            joint_id: jId,
            target_velocity: Number(targetVel.toFixed(6)),
            max_force: Number(Math.max(1, maxEffort).toFixed(4)),
          },
        });
      }
    }

    // Add gear metadata joint so Rust `coupled_sim.rs` enables boundary-snapped teeth (`AccuracyMode::Standard`)
    // and tightened involute contact search radius.
    if (!hasGearJoint) {
      operations.push({
        tool: 'create_joint',
        args: {
          joint_id: `auto_gear_${gA.part.id}_${gB.part.id}`,
          joint_type: 'gear',
          part_a: gA.part.id,
          part_b: gB.part.id,
          axis_a: [0, 0, 1],
          axis_b: [0, 0, 1],
          teeth_a: gA.gear.teeth,
          teeth_b: gB.gear.teeth,
        },
      });
    }
  }

  // 4. Build /api/coupled-sim/bake payload
  // Cap physical FEM bake duration reasonably for interactive HHT-α solves (10 ms .. 250 ms)
  const durationS = Number(
    Math.min(0.25, Math.max(0.015, config.durationSec * 0.04)).toFixed(4)
  );
  const outputFps = Math.max(60, Math.min(2400, Math.round(60 / Math.max(0.02, durationS))));

  const bakePayload: Record<string, unknown> = {
    part_ids: activePartIds,
    accuracy_level: accuracyLevel,
    duration_s: durationS,
    output_fps: outputFps,
    max_frames: 120,
    gravity: [0, config.gravityY, 0],
    enable_plasticity: true,
    enable_fracture: true,
    enable_contact: activePartIds.length > 1,
    use_physics_joints: true,
    corotated: true,
    async_mode: true,
    length_unit: 'm',
  };

  // For single-part / impact_crush without joints, attach physical fixed support + ramp impact force
  if (partsWithJoints.size === 0 && activeParts.length >= 1) {
    const primary = activeParts[0];
    const bb = primary.mesh.boundingBox;
    const leverR = Math.max(
      0.01,
      0.25 * ((bb.max[0] - bb.min[0]) + (bb.max[1] - bb.min[1]))
    );
    const impactForceN = Math.max(500, config.impactTorqueNm / leverR);
    bakePayload.fixed_faces = [{ part_id: primary.id, face: 'min_y' }];
    bakePayload.loads = [
      {
        part_id: primary.id,
        face: 'max_y',
        force: [0, -Number(impactForceN.toFixed(1)), 0],
      },
    ];
    bakePayload.force_curve = {
      type: 'ramp',
      amplitude: 1.0,
      duration_s: Number((durationS * 0.65).toFixed(4)),
    };
  }

  return {
    operations,
    activePartIds,
    syncedPartIds,
    warnings,
    bakePayload,
  };
}
