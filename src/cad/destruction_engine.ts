import type {
  CachedPartFrameState,
  CadPart,
  DestructionCache,
  DestructionCacheFrame,
  DestructionFrameMetrics,
  DestructionScenarioMode,
  DestructionSimConfig,
  DestructionTimelineEvent,
  FractureShardGeometry,
  KinematicJoint,
  MaterialSpec,
} from '../types/cad';
import { getMaterialById } from './materials';
import {
  bakePressDestructionCache,
  detectPressAssembly,
} from './press_simulation';
import { estimateRemainingMs } from './coupled_sim_stream';

type Vec2 = [number, number];
type Vec3 = [number, number, number];
type Quat = [number, number, number, number]; // [x, y, z, w]

export const DEFAULT_DESTRUCTION_CONFIG: DestructionSimConfig = {
  scenarioMode: 'counter_clash',
  durationSec: 2.5,
  fps: 60,
  motorRpmLeft: 120,
  motorRpmRight: 120, // Same sign -> opposing linear tooth velocity at mesh point (head-on clash)
  impactTorqueNm: 185,
  brittleness: 0.68,
  shardCountPerPart: 24,
  gravityY: -9.81,
  restitution: 0.42,
  seed: 4217,
};

export function getPresetDestructionConfig(
  mode: DestructionScenarioMode
): DestructionSimConfig {
  switch (mode) {
    case 'counter_clash':
      return {
        ...DEFAULT_DESTRUCTION_CONFIG,
        scenarioMode: 'counter_clash',
        motorRpmLeft: 120,
        motorRpmRight: 120,
        impactTorqueNm: 185,
        brittleness: 0.68,
        shardCountPerPart: 24,
        durationSec: 2.5,
      };
    case 'torque_overload':
      return {
        ...DEFAULT_DESTRUCTION_CONFIG,
        scenarioMode: 'torque_overload',
        motorRpmLeft: 180,
        motorRpmRight: -120, // Exact kinematic mesh ratio for Z1=20 / Z2=30 under heavy braking overload
        impactTorqueNm: 260,
        brittleness: 0.58,
        shardCountPerPart: 28,
        durationSec: 2.8,
      };
    case 'impact_crush':
      return {
        ...DEFAULT_DESTRUCTION_CONFIG,
        scenarioMode: 'impact_crush',
        motorRpmLeft: 35,
        motorRpmRight: 0,
        impactTorqueNm: 310,
        brittleness: 0.78,
        shardCountPerPart: 30,
        durationSec: 2.4,
      };
  }
}

/** Deterministic PRNG (Mulberry32) */
function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function add3(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function sub3(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function scale3(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

function cross3(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function dot3(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function len3(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

function normalize3(a: Vec3): Vec3 {
  const l = len3(a);
  if (l < 1e-12) return [0, 1, 0];
  return [a[0] / l, a[1] / l, a[2] / l];
}

function wrapAnglePi(a: number): number {
  let r = (a + Math.PI) % (Math.PI * 2);
  if (r < 0) r += Math.PI * 2;
  return r - Math.PI;
}

function quatNormalize(q: Quat): Quat {
  const l = Math.hypot(q[0], q[1], q[2], q[3]);
  if (l < 1e-12) return [0, 0, 0, 1];
  return [q[0] / l, q[1] / l, q[2] / l, q[3] / l];
}

function quatMul(a: Quat, b: Quat): Quat {
  return [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ];
}

/** Matches Three.js default Euler XYZ order */
export function quatFromEulerXYZ(rot: Vec3): Quat {
  const c1 = Math.cos(rot[0] * 0.5);
  const c2 = Math.cos(rot[1] * 0.5);
  const c3 = Math.cos(rot[2] * 0.5);
  const s1 = Math.sin(rot[0] * 0.5);
  const s2 = Math.sin(rot[1] * 0.5);
  const s3 = Math.sin(rot[2] * 0.5);
  return quatNormalize([
    s1 * c2 * c3 + c1 * s2 * s3,
    c1 * s2 * c3 - s1 * c2 * s3,
    c1 * c2 * s3 + s1 * s2 * c3,
    c1 * c2 * c3 - s1 * s2 * s3,
  ]);
}

export function rotateVecByQuat(v: Vec3, q: Quat): Vec3 {
  const [qx, qy, qz, qw] = q;
  const [vx, vy, vz] = v;
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  return [
    vx + qw * tx + (qy * tz - qz * ty),
    vy + qw * ty + (qz * tx - qx * tz),
    vz + qw * tz + (qx * ty - qy * tx),
  ];
}

function integrateQuat(q: Quat, omega: Vec3, dt: number): Quat {
  const [wx, wy, wz] = omega;
  const halfDt = 0.5 * dt;
  const dq: Quat = [wx * halfDt, wy * halfDt, wz * halfDt, 0];
  const prod = quatMul(dq, q);
  return quatNormalize([
    q[0] + prod[0],
    q[1] + prod[1],
    q[2] + prod[2],
    q[3] + prod[3],
  ]);
}

const STATIC_SUPPORT_TOKENS = new Set([
  'base',
  'frame',
  'rig',
  'bench',
  'bed',
  'shaft',
  'stand',
  'backplate',
]);

function isStaticSupportPart(part: CadPart): boolean {
  if (part.isFixed) return true;
  const lower = `${part.id} ${part.name}`.toLowerCase();
  if (lower.includes('plate_mount')) return true;
  const tokens = lower.split(/[^a-z0-9а-яё]+/);
  return tokens.some((tok) => STATIC_SUPPORT_TOKENS.has(tok));
}

// ============================================================================
// EXACT 2D CONVEX-TOOTH SAT COLLISION SOLVER FOR SPUR GEARS
// ============================================================================

export interface GearToothPoly {
  toothIndex: number;
  centerAngle: number;
  /** Strictly CCW convex 2D polygon in local XY plane (meters) */
  localPoly: Vec2[];
}

export interface GearCollider2D {
  teeth: number;
  moduleM: number;
  faceWidthM: number;
  pitchRadius: number;
  outerRadius: number;
  rootRadius: number;
  lewisY: number;
  toothPolys: GearToothPoly[];
}

export interface GearContact2D {
  depth: number;
  /** Unit normal pointing from Gear A toward Gear B */
  normal: Vec2;
  /** World 2D contact point */
  point: Vec2;
  toothIdxA: number;
  toothIdxB: number;
}

/** Monotone chain 2D convex hull returning strictly CCW vertices */
function convexHull2D(points: Vec2[]): Vec2[] {
  if (points.length <= 3) return points.slice();
  const sorted = points
    .slice()
    .sort((a, b) => (a[0] !== b[0] ? a[0] - b[0] : a[1] - b[1]));
  const cross = (o: Vec2, a: Vec2, b: Vec2) =>
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);

  const lower: Vec2[] = [];
  for (const p of sorted) {
    while (
      lower.length >= 2 &&
      cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 1e-12
    ) {
      lower.pop();
    }
    lower.push(p);
  }
  const upper: Vec2[] = [];
  for (let i = sorted.length - 1; i >= 0; i--) {
    const p = sorted[i];
    while (
      upper.length >= 2 &&
      cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 1e-12
    ) {
      upper.pop();
    }
    upper.push(p);
  }
  lower.pop();
  upper.pop();
  return lower.concat(upper);
}

/**
 * Extracts exact 2D involute tooth polygons matching `createSpurGearMesh` in `primitives.ts`.
 * Returns null for non-gear parts.
 */
export function buildGearCollider(part: CadPart): GearCollider2D | null {
  const gearNode = part.dag.find((n) => n.enabled && n.type === 'spur_gear');
  if (!gearNode) return null;

  const p = gearNode.params || {};
  const teeth = Math.max(6, Math.round(Number(p.teeth ?? 20)));
  const moduleMm = Math.max(0.2, Number(p.module ?? 2.0));
  const pressureAngleDeg = Number(p.pressure_angle_deg ?? 20);
  const faceWidthM = Math.max(
    0.004,
    Number(
      p.face_width ??
        Math.max(0.008, part.mesh.boundingBox.max[2] - part.mesh.boundingBox.min[2])
    )
  );
  const boreDiameter = Number(p.bore_diameter ?? 0.01);

  const m = moduleMm * 0.001;
  const pitchRadius = (teeth * m) / 2;
  const alpha = (pressureAngleDeg * Math.PI) / 180;
  const baseRadius = pitchRadius * Math.cos(alpha);
  const outerRadius = pitchRadius + 1.0 * m;
  const rootRadius = Math.max(
    pitchRadius - 1.25 * m,
    boreDiameter * 0.5 + 0.002
  );

  const invAlpha = Math.tan(alpha) - alpha;
  const phiTip =
    outerRadius > baseRadius ? Math.acos(baseRadius / outerRadius) : alpha;
  const invTip = Math.tan(phiTip) - phiTip;

  // Standard 0.8% mechanical backlash clearance (~0.014mm) ensures smooth rolling mesh without false binding
  const backlashScale = 0.992;
  const halfPitch = (Math.PI / (2 * teeth)) * backlashScale;
  const halfBase = (Math.PI / (2 * teeth) + invAlpha) * backlashScale;
  const halfTip =
    Math.max(0.05 / teeth, Math.PI / (2 * teeth) + invAlpha - invTip) *
    backlashScale;

  // Lewis form factor Y (AGMA 20° full-depth involute approximation)
  const lewisY = Math.max(0.18, 0.484 - 2.87 / teeth);

  const toothPolys: GearToothPoly[] = [];
  for (let t = 0; t < teeth; t++) {
    const centerAngle = (t / teeth) * Math.PI * 2;
    const rawPts: Vec2[] = [
      [
        Math.cos(centerAngle - halfBase) * rootRadius,
        Math.sin(centerAngle - halfBase) * rootRadius,
      ],
      [
        Math.cos(centerAngle - halfPitch) * pitchRadius,
        Math.sin(centerAngle - halfPitch) * pitchRadius,
      ],
      [
        Math.cos(centerAngle - halfTip) * outerRadius,
        Math.sin(centerAngle - halfTip) * outerRadius,
      ],
      [
        Math.cos(centerAngle + halfTip) * outerRadius,
        Math.sin(centerAngle + halfTip) * outerRadius,
      ],
      [
        Math.cos(centerAngle + halfPitch) * pitchRadius,
        Math.sin(centerAngle + halfPitch) * pitchRadius,
      ],
      [
        Math.cos(centerAngle + halfBase) * rootRadius,
        Math.sin(centerAngle + halfBase) * rootRadius,
      ],
    ];
    toothPolys.push({
      toothIndex: t,
      centerAngle,
      localPoly: convexHull2D(rawPts),
    });
  }

  return {
    teeth,
    moduleM: m,
    faceWidthM,
    pitchRadius,
    outerRadius,
    rootRadius,
    lewisY,
    toothPolys,
  };
}

function transformPoly2D(
  localPoly: Vec2[],
  cx: number,
  cy: number,
  angle: number
): Vec2[] {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const out: Vec2[] = new Array(localPoly.length);
  for (let i = 0; i < localPoly.length; i++) {
    const x = localPoly[i][0];
    const y = localPoly[i][1];
    out[i] = [cx + x * c - y * s, cy + x * s + y * c];
  }
  return out;
}

function polyCentroid2D(poly: Vec2[]): Vec2 {
  let sx = 0;
  let sy = 0;
  for (let i = 0; i < poly.length; i++) {
    sx += poly[i][0];
    sy += poly[i][1];
  }
  const inv = 1 / Math.max(1, poly.length);
  return [sx * inv, sy * inv];
}

/**
 * Exact 2D Separating Axis Theorem (SAT) collision test between two CCW convex polygons.
 * Returns null if separated; otherwise returns minimum penetration depth, unit normal (A -> B),
 * and world contact point.
 */
function satConvexPolys2D(
  polyA: Vec2[],
  polyB: Vec2[]
): { depth: number; normal: Vec2; point: Vec2 } | null {
  let minOverlap = Infinity;
  let bestNx = 0;
  let bestNy = 0;
  let normalFromA = true;

  const testAxes = (poly: Vec2[], isA: boolean): boolean => {
    const n = poly.length;
    for (let i = 0; i < n; i++) {
      const p0 = poly[i];
      const p1 = poly[(i + 1) % n];
      const ex = p1[0] - p0[0];
      const ey = p1[1] - p0[1];
      const len = Math.hypot(ex, ey);
      if (len < 1e-12) continue;
      // Outward normal for CCW polygon is (ey, -ex)
      const nx = ey / len;
      const ny = -ex / len;

      let minA = Infinity;
      let maxA = -Infinity;
      for (let k = 0; k < polyA.length; k++) {
        const proj = polyA[k][0] * nx + polyA[k][1] * ny;
        if (proj < minA) minA = proj;
        if (proj > maxA) maxA = proj;
      }

      let minB = Infinity;
      let maxB = -Infinity;
      for (let k = 0; k < polyB.length; k++) {
        const proj = polyB[k][0] * nx + polyB[k][1] * ny;
        if (proj < minB) minB = proj;
        if (proj > maxB) maxB = proj;
      }

      const overlap = Math.min(maxA, maxB) - Math.max(minA, minB);
      if (overlap <= 0) return false; // Separating axis found

      if (overlap < minOverlap) {
        minOverlap = overlap;
        bestNx = nx;
        bestNy = ny;
        normalFromA = isA;
      }
    }
    return true;
  };

  if (!testAxes(polyA, true)) return null;
  if (!testAxes(polyB, false)) return null;

  // Orient normal consistently from A -> B
  const cA = polyCentroid2D(polyA);
  const cB = polyCentroid2D(polyB);
  const abx = cB[0] - cA[0];
  const aby = cB[1] - cA[1];
  if (abx * bestNx + aby * bestNy < 0) {
    bestNx = -bestNx;
    bestNy = -bestNy;
  }

  // Contact point: average of deepest penetrating vertices along normal
  let contactPt: Vec2;
  if (normalFromA) {
    // Deepest vertex of B into A (minimum projection along normal A->B)
    let minProj = Infinity;
    let bestV = polyB[0];
    let secondV: Vec2 | null = null;
    for (let i = 0; i < polyB.length; i++) {
      const proj = polyB[i][0] * bestNx + polyB[i][1] * bestNy;
      if (proj < minProj - 1e-6) {
        minProj = proj;
        bestV = polyB[i];
        secondV = null;
      } else if (Math.abs(proj - minProj) <= 1e-5) {
        secondV = polyB[i];
      }
    }
    contactPt = secondV
      ? [(bestV[0] + secondV[0]) * 0.5, (bestV[1] + secondV[1]) * 0.5]
      : [bestV[0], bestV[1]];
  } else {
    // Deepest vertex of A into B (maximum projection along normal A->B)
    let maxProj = -Infinity;
    let bestV = polyA[0];
    let secondV: Vec2 | null = null;
    for (let i = 0; i < polyA.length; i++) {
      const proj = polyA[i][0] * bestNx + polyA[i][1] * bestNy;
      if (proj > maxProj + 1e-6) {
        maxProj = proj;
        bestV = polyA[i];
        secondV = null;
      } else if (Math.abs(proj - maxProj) <= 1e-5) {
        secondV = polyA[i];
      }
    }
    contactPt = secondV
      ? [(bestV[0] + secondV[0]) * 0.5, (bestV[1] + secondV[1]) * 0.5]
      : [bestV[0], bestV[1]];
  }

  return {
    depth: minOverlap,
    normal: [bestNx, bestNy],
    point: contactPt,
  };
}

/**
 * Detects ALL active 2D tooth-to-tooth and tooth-to-root contacts between two spur gears.
 * Fast narrowphase checks only teeth facing the mesh zone (typically 3x3 = 9 convex pairs).
 */
export function detectGearPairContacts(
  gearA: GearCollider2D,
  posA: Vec2,
  angleA: number,
  gearB: GearCollider2D,
  posB: Vec2,
  angleB: number,
  brokenTeethA?: Set<number>,
  brokenTeethB?: Set<number>
): GearContact2D[] {
  const dx = posB[0] - posA[0];
  const dy = posB[1] - posA[1];
  const centerDist = Math.hypot(dx, dy);
  if (centerDist >= gearA.outerRadius + gearB.outerRadius - 1e-7) {
    return [];
  }

  const dirAB = Math.atan2(dy, dx);
  const dirBA = dirAB + Math.PI;
  const angWindowA = Math.max(0.52, (2.6 * Math.PI) / gearA.teeth);
  const angWindowB = Math.max(0.52, (2.6 * Math.PI) / gearB.teeth);

  const candA: { toothIndex: number; poly: Vec2[] }[] = [];
  for (let i = 0; i < gearA.toothPolys.length; i++) {
    const tp = gearA.toothPolys[i];
    if (brokenTeethA?.has(tp.toothIndex)) continue;
    const diff = Math.abs(wrapAnglePi(tp.centerAngle + angleA - dirAB));
    if (diff <= angWindowA) {
      candA.push({
        toothIndex: tp.toothIndex,
        poly: transformPoly2D(tp.localPoly, posA[0], posA[1], angleA),
      });
    }
  }

  const candB: { toothIndex: number; poly: Vec2[] }[] = [];
  for (let i = 0; i < gearB.toothPolys.length; i++) {
    const tp = gearB.toothPolys[i];
    if (brokenTeethB?.has(tp.toothIndex)) continue;
    const diff = Math.abs(wrapAnglePi(tp.centerAngle + angleB - dirBA));
    if (diff <= angWindowB) {
      candB.push({
        toothIndex: tp.toothIndex,
        poly: transformPoly2D(tp.localPoly, posB[0], posB[1], angleB),
      });
    }
  }

  const contacts: GearContact2D[] = [];

  for (let i = 0; i < candA.length; i++) {
    const a = candA[i];
    for (let j = 0; j < candB.length; j++) {
      const b = candB[j];
      const hit = satConvexPolys2D(a.poly, b.poly);
      if (hit && hit.depth > 1e-7) {
        contacts.push({
          depth: hit.depth,
          normal: hit.normal,
          point: hit.point,
          toothIdxA: a.toothIndex,
          toothIdxB: b.toothIndex,
        });
      }
    }
  }

  // Root-circle guard: prevent opposing tooth tip from piercing the other gear's root body
  const rootSqB = gearB.rootRadius * gearB.rootRadius;
  for (let i = 0; i < candA.length; i++) {
    const a = candA[i];
    for (let k = 0; k < a.poly.length; k++) {
      const vx = a.poly[k][0] - posB[0];
      const vy = a.poly[k][1] - posB[1];
      const dSq = vx * vx + vy * vy;
      if (dSq < rootSqB && dSq > 1e-10) {
        const d = Math.sqrt(dSq);
        const depth = gearB.rootRadius - d;
        // Normal points from A to B (inward toward B center)
        contacts.push({
          depth,
          normal: [-vx / d, -vy / d],
          point: [a.poly[k][0], a.poly[k][1]],
          toothIdxA: a.toothIndex,
          toothIdxB: -1,
        });
      }
    }
  }

  const rootSqA = gearA.rootRadius * gearA.rootRadius;
  for (let j = 0; j < candB.length; j++) {
    const b = candB[j];
    for (let k = 0; k < b.poly.length; k++) {
      const vx = b.poly[k][0] - posA[0];
      const vy = b.poly[k][1] - posA[1];
      const dSq = vx * vx + vy * vy;
      if (dSq < rootSqA && dSq > 1e-10) {
        const d = Math.sqrt(dSq);
        const depth = gearA.rootRadius - d;
        // Normal points from A to B (outward from A center)
        contacts.push({
          depth,
          normal: [vx / d, vy / d],
          point: [b.poly[k][0], b.poly[k][1]],
          toothIdxA: -1,
          toothIdxB: b.toothIndex,
        });
      }
    }
  }

  if (contacts.length > 1) {
    contacts.sort((u, v) => v.depth - u.depth);
  }
  return contacts;
}

/**
 * Diagnostic/test helper: returns maximum tooth penetration depth (in meters)
 * between two spur gears at given world positions and Z angles.
 */
export function measureGearPairMaxPenetration(
  gearA: GearCollider2D,
  posA: Vec2,
  angleA: number,
  gearB: GearCollider2D,
  posB: Vec2,
  angleB: number,
  brokenTeethA?: Set<number>,
  brokenTeethB?: Set<number>
): number {
  const hits = detectGearPairContacts(
    gearA,
    posA,
    angleA,
    gearB,
    posB,
    angleB,
    brokenTeethA,
    brokenTeethB
  );
  return hits.length > 0 ? hits[0].depth : 0;
}

// ============================================================================
// DETERMINISTIC 3D VORONOI FRACTURE MESH GENERATOR
// ============================================================================

/**
 * Deterministically fractures a real CAD part triangle mesh into 3D solid Voronoi/wedge shards.
 * Preserves exact original CAD outer triangles (gear teeth, hubs, chamfers) and stitches
 * newly exposed faceted crystalline interior fracture faces along every crack seam.
 */
export function fracturePartIntoShards(
  part: CadPart,
  material: MaterialSpec,
  targetShardCount: number,
  contactPointLocal: Vec3,
  seed: number
): FractureShardGeometry[] {
  const rng = mulberry32(seed);
  const verts = part.mesh.vertices;
  const norms = part.mesh.normals;
  const indices = part.mesh.indices;
  const triCount = Math.floor(indices.length / 3);
  if (triCount === 0) return [];

  const bbox = part.mesh.boundingBox;
  const minX = bbox.min[0];
  const minY = bbox.min[1];
  const minZ = bbox.min[2];
  const maxX = bbox.max[0];
  const maxY = bbox.max[1];
  const maxZ = bbox.max[2];
  const sizeX = Math.max(1e-4, maxX - minX);
  const sizeY = Math.max(1e-4, maxY - minY);
  const sizeZ = Math.max(1e-4, maxZ - minZ);
  const center: Vec3 = [
    0.5 * (minX + maxX),
    0.5 * (minY + maxY),
    0.5 * (minZ + maxZ),
  ];
  const maxDim = Math.max(sizeX, sizeY, sizeZ);
  const isGear = part.dag.some((d) => d.type === 'spur_gear');

  const safeTarget = Math.max(4, Math.min(48, Math.round(targetShardCount)));
  const seeds: Vec3[] = [];

  if (isGear) {
    // Distribute ~65% rim/tooth sectors uniformly around the full 360° circumference
    // so whichever teeth physically collide at ANY rotation angle cleanly shear off in Stage 1,
    // plus ~35% inner structural web/hub sectors for Stage 2 radial body rupture.
    const rimSeeds = Math.max(4, Math.round(safeTarget * 0.65));
    const hubSeeds = Math.max(2, safeTarget - rimSeeds);
    const radiusXY = 0.5 * Math.max(sizeX, sizeY);

    for (let i = 0; i < rimSeeds; i++) {
      const a = (i / rimSeeds) * Math.PI * 2 + (rng() - 0.5) * 0.16;
      const rFrac = 0.76 + rng() * 0.22;
      const r = radiusXY * rFrac;
      const zOff = (rng() - 0.5) * sizeZ * 0.45;
      seeds.push([
        center[0] + Math.cos(a) * r,
        center[1] + Math.sin(a) * r,
        center[2] + zOff,
      ]);
    }

    for (let i = 0; i < hubSeeds; i++) {
      const a =
        ((i + 0.5) / hubSeeds) * Math.PI * 2 + (rng() - 0.5) * 0.3;
      const rFrac = 0.28 + rng() * 0.28;
      const r = radiusXY * rFrac;
      const zOff = (rng() - 0.5) * sizeZ * 0.4;
      seeds.push([
        center[0] + Math.cos(a) * r,
        center[1] + Math.sin(a) * r,
        center[2] + zOff,
      ]);
    }
  } else {
    // General 3D part: cluster ~40% around impact/contact zone + stratified volumetric grid
    const impactSeeds = Math.max(2, Math.round(safeTarget * 0.4));
    for (let i = 0; i < impactSeeds; i++) {
      seeds.push([
        contactPointLocal[0] + (rng() - 0.5) * sizeX * 0.55,
        contactPointLocal[1] + (rng() - 0.5) * sizeY * 0.55,
        contactPointLocal[2] + (rng() - 0.5) * sizeZ * 0.55,
      ]);
    }
    for (let i = impactSeeds; i < safeTarget; i++) {
      seeds.push([
        minX + rng() * sizeX,
        minY + rng() * sizeY,
        minZ + rng() * sizeZ,
      ]);
    }
  }

  // Assign each original triangle to its nearest Voronoi seed (with slight anisotropic jitter for jagged crack lines)
  const cellTris: number[][] = Array.from({ length: seeds.length }, () => []);
  const jitterFreq = 18.0 / Math.max(0.005, maxDim);

  for (let t = 0; t < triCount; t++) {
    const i0 = indices[t * 3] * 3;
    const i1 = indices[t * 3 + 1] * 3;
    const i2 = indices[t * 3 + 2] * 3;
    const cx = (verts[i0] + verts[i1] + verts[i2]) / 3;
    const cy = (verts[i0 + 1] + verts[i1 + 1] + verts[i2 + 1]) / 3;
    const cz = (verts[i0 + 2] + verts[i1 + 2] + verts[i2 + 2]) / 3;

    const warpX =
      cx + Math.sin(cy * jitterFreq + cz * jitterFreq * 0.7) * maxDim * 0.018;
    const warpY =
      cy + Math.cos(cx * jitterFreq - cz * jitterFreq * 0.9) * maxDim * 0.018;
    const warpZ = cz;

    let bestSeed = 0;
    let bestDistSq = Infinity;
    for (let s = 0; s < seeds.length; s++) {
      const dx = warpX - seeds[s][0];
      const dy = warpY - seeds[s][1];
      const dz = warpZ - seeds[s][2];
      const dSq = dx * dx + dy * dy + dz * dz;
      if (dSq < bestDistSq) {
        bestDistSq = dSq;
        bestSeed = s;
      }
    }
    cellTris[bestSeed].push(t);
  }

  const totalMass = Math.max(0.005, part.mesh.mass || 0.08);
  const invWeldTol = 1 / Math.max(1e-6, maxDim * 1e-4);
  const shardList: FractureShardGeometry[] = [];

  for (let c = 0; c < cellTris.length; c++) {
    const tris = cellTris[c];
    if (tris.length < 2) continue;

    const outerVerts: number[] = [];
    const outerNorms: number[] = [];
    const outerIndices: number[] = [];

    let sumX = 0;
    let sumY = 0;
    let sumZ = 0;
    let sumNx = 0;
    let sumNy = 0;
    let sumNz = 0;
    let vertCounter = 0;

    const weldKey = (x: number, y: number, z: number): string =>
      `${Math.round(x * invWeldTol)}_${Math.round(y * invWeldTol)}_${Math.round(z * invWeldTol)}`;

    const edgeCounts = new Map<string, { count: number; a: Vec3; b: Vec3 }>();

    for (let k = 0; k < tris.length; k++) {
      const t = tris[k];
      const triPts: Vec3[] = [];
      const triKeys: string[] = [];

      for (let corner = 0; corner < 3; corner++) {
        const vi = indices[t * 3 + corner] * 3;
        const vx = verts[vi];
        const vy = verts[vi + 1];
        const vz = verts[vi + 2];
        const nx = norms[vi] ?? 0;
        const ny = norms[vi + 1] ?? 1;
        const nz = norms[vi + 2] ?? 0;

        outerVerts.push(vx, vy, vz);
        outerNorms.push(nx, ny, nz);
        outerIndices.push(vertCounter++);

        sumX += vx;
        sumY += vy;
        sumZ += vz;
        sumNx += nx;
        sumNy += ny;
        sumNz += nz;

        triPts.push([vx, vy, vz]);
        triKeys.push(weldKey(vx, vy, vz));
      }

      for (let e = 0; e < 3; e++) {
        const next = (e + 1) % 3;
        const kA = triKeys[e];
        const kB = triKeys[next];
        if (kA === kB) continue;
        const uKey = kA < kB ? `${kA}|${kB}` : `${kB}|${kA}`;
        const existing = edgeCounts.get(uKey);
        if (existing) {
          existing.count += 1;
        } else {
          edgeCounts.set(uKey, {
            count: 1,
            a: triPts[e],
            b: triPts[next],
          });
        }
      }
    }

    const invV = 1 / Math.max(1, vertCounter);
    const localCom: Vec3 = [sumX * invV, sumY * invV, sumZ * invV];

    const avgN = normalize3([sumNx, sumNy, sumNz]);
    const toPartCenter = sub3(center, localCom);
    const coreOffset = add3(
      scale3(toPartCenter, 0.22),
      scale3(avgN, -maxDim * 0.035)
    );
    const innerApexWorld = add3(localCom, coreOffset);

    let maxRadSq = 0;
    for (let i = 0; i < outerVerts.length; i += 3) {
      outerVerts[i] -= localCom[0];
      outerVerts[i + 1] -= localCom[1];
      outerVerts[i + 2] -= localCom[2];
      const rSq =
        outerVerts[i] * outerVerts[i] +
        outerVerts[i + 1] * outerVerts[i + 1] +
        outerVerts[i + 2] * outerVerts[i + 2];
      if (rSq > maxRadSq) maxRadSq = rSq;
    }

    const innerVerts: number[] = [];
    const innerNorms: number[] = [];
    const innerIndices: number[] = [];
    let innerVertCounter = 0;

    const apexLocal = sub3(innerApexWorld, localCom);
    for (const edge of edgeCounts.values()) {
      if (edge.count !== 1) continue;
      const aL = sub3(edge.a, localCom);
      const bL = sub3(edge.b, localCom);
      const edgeMid = scale3(add3(aL, bL), 0.5);
      const rough =
        Math.sin((edgeMid[0] + edgeMid[1] + edgeMid[2]) * jitterFreq * 1.3) *
        maxDim *
        0.008;
      const apexJ: Vec3 = [
        apexLocal[0] + rough,
        apexLocal[1] - rough * 0.7,
        apexLocal[2] + rough * 0.5,
      ];

      const e1 = sub3(apexJ, bL);
      const e2 = sub3(aL, bL);
      const faceN = normalize3(cross3(e1, e2));

      innerVerts.push(
        bL[0], bL[1], bL[2],
        aL[0], aL[1], aL[2],
        apexJ[0], apexJ[1], apexJ[2]
      );
      innerNorms.push(
        faceN[0], faceN[1], faceN[2],
        faceN[0], faceN[1], faceN[2],
        faceN[0], faceN[1], faceN[2]
      );
      innerIndices.push(
        innerVertCounter,
        innerVertCounter + 1,
        innerVertCounter + 2
      );
      innerVertCounter += 3;
    }

    const massFrac = tris.length / triCount;
    const shardMass = Math.max(0.0005, totalMass * massFrac);
    const boundingRadius = Math.max(0.0018, Math.sqrt(maxRadSq) * 0.78);
    const distToContact = len3(sub3(localCom, contactPointLocal));

    shardList.push({
      id: `${part.id}_shard_${shardList.length}`,
      parentPartId: part.id,
      materialId: part.materialId,
      colorHex: material.colorHex,
      roughness: material.roughness,
      metalness: material.metalness,
      localCom,
      boundingRadius,
      mass: shardMass,
      distToContact,
      vertices: new Float32Array(outerVerts),
      normals: new Float32Array(outerNorms),
      indices: new Uint32Array(outerIndices),
      innerVertices: new Float32Array(innerVerts),
      innerNormals: new Float32Array(innerNorms),
      innerIndices: new Uint32Array(innerIndices),
    });
  }

  const rawMassSum = shardList.reduce((acc, s) => acc + s.mass, 0);
  if (rawMassSum > 1e-9) {
    const k = totalMass / rawMassSum;
    for (const s of shardList) {
      s.mass *= k;
    }
  }

  return shardList;
}

// ============================================================================
// SUBSTEPPED RIGID-BODY + SAT GEAR-TOOTH COLLISION CACHE BAKER
// ============================================================================

interface InternalShardRuntime {
  shardIdx: number;
  parentPartId: string;
  localCom: Vec3;
  localAngle: number;
  localRadiusXY: number;
  boundingRadius: number;
  mass: number;
  active: boolean;
  pos: Vec3;
  quat: Quat;
  vel: Vec3;
  omega: Vec3;
  stressGlow: number;
  stressMPa: number;
  plasticStrain: number;
  damage: number;
}

interface InternalSpark {
  pos: Vec3;
  vel: Vec3;
  life: number;
  maxLife: number;
}

interface InternalPartRuntime {
  part: CadPart;
  material: MaterialSpec;
  destructible: boolean;
  collider: GearCollider2D | null;
  brokenTeeth: Set<number>;
  pos: Vec3;
  rot: Vec3;
  omegaZ: number;
  targetOmegaZ: number;
  inertiaZ: number;
  gearDiskI: number;
  baseRpm: number;
  stressMPa: number;
  yieldMPa: number;
  damage: number;
  yieldLogged: boolean;
  stage1Broken: boolean;
  stage2Broken: boolean;
  fractureThresholdScale: number;
  // Precomputed per-vertex spatial coordinates for fast O(N) field synthesis
  vertRadiusXY: Float32Array;
  vertAngle: Float32Array;
  // Irreversible per-vertex elastoplastic & continuum damage history
  plasticStrainVert: Float32Array;
  damageVert: Float32Array;
  plasticDispVert: Float32Array;
  plasticEnergyJ: number;
  elasticEnergyJ: number;
}

function computeFloorY(parts: CadPart[]): number {
  let minY = Infinity;
  for (const p of parts) {
    if (!p.visible) continue;
    const py = p.position[1];
    const sy = Math.abs(p.scale[1] || 1);
    const bMinY = py + p.mesh.boundingBox.min[1] * sy;
    if (bMinY < minY) minY = bMinY;
  }
  if (!Number.isFinite(minY)) return -0.05;
  return minY - 0.012;
}

function computePrimaryContactPoint(
  destructibles: CadPart[],
  config: DestructionSimConfig
): Vec3 {
  if (destructibles.length >= 2) {
    const a = destructibles[0];
    const b = destructibles[1];
    const ra = Math.max(
      0.005,
      0.5 * (a.mesh.boundingBox.max[0] - a.mesh.boundingBox.min[0])
    );
    const rb = Math.max(
      0.005,
      0.5 * (b.mesh.boundingBox.max[0] - b.mesh.boundingBox.min[0])
    );
    const t = ra / (ra + rb);
    return [
      a.position[0] + (b.position[0] - a.position[0]) * t,
      a.position[1] + (b.position[1] - a.position[1]) * t,
      a.position[2] + (b.position[2] - a.position[2]) * t,
    ];
  }
  if (destructibles.length === 1) {
    const p = destructibles[0];
    const maxX = p.mesh.boundingBox.max[0];
    if (config.scenarioMode === 'impact_crush') {
      return [p.position[0] + maxX * 0.85, p.position[1], p.position[2]];
    }
    return [...p.position] as Vec3;
  }
  return [0, 0, 0];
}

/**
 * Ensures two already-meshed gears start with zero initial tooth interpenetration at frame 0.
 * Leaves separated gears untouched.
 */
function alignInitialGearPhaseIfOverlapping(
  a: InternalPartRuntime,
  b: InternalPartRuntime
): void {
  if (!a.collider || !b.collider) return;
  const initialPen = measureGearPairMaxPenetration(
    a.collider,
    [a.pos[0], a.pos[1]],
    a.rot[2],
    b.collider,
    [b.pos[0], b.pos[1]],
    b.rot[2]
  );
  if (initialPen <= 1e-5) return;

  const pitchAngB = (Math.PI * 2) / b.collider.teeth;
  let bestAngle = b.rot[2];
  let bestPen = initialPen;
  const samples = 48;
  for (let i = 0; i < samples; i++) {
    const offset = ((i / samples) - 0.5) * pitchAngB;
    const cand = b.rot[2] + offset;
    const pen = measureGearPairMaxPenetration(
      a.collider,
      [a.pos[0], a.pos[1]],
      a.rot[2],
      b.collider,
      [b.pos[0], b.pos[1]],
      cand
    );
    if (pen < bestPen) {
      bestPen = pen;
      bestAngle = cand;
      if (pen <= 1e-7) break;
    }
  }
  b.rot[2] = bestAngle;
}

export interface DestructionStreamMeta {
  progressPct: number;
  stepCount: number;
  totalSteps: number;
  framesCached: number;
  latestFrameIndex: number;
  timeSec: number;
  totalDurationSec: number;
  maxVonMisesMPa: number;
  activeShards: number;
  elapsedMs: number;
  estimatedRemainingMs: number | null;
  stageLabel: string;
}

export interface DestructionAsyncBakeOptions {
  signal?: AbortSignal;
  onPartialCache?: (
    cache: DestructionCache,
    latestFrameIndex: number,
    meta: DestructionStreamMeta
  ) => void;
  onMeta?: (meta: DestructionStreamMeta) => void;
  yieldEveryFrames?: number;
}

function computeSingleFrameBytes(f: DestructionCacheFrame): number {
  let bytes =
    f.shardPos.byteLength +
    f.shardQuat.byteLength +
    f.shardActive.byteLength +
    f.shardStress.byteLength +
    (f.shardStressMPa?.byteLength ?? 0) +
    (f.shardPlasticStrain?.byteLength ?? 0) +
    (f.shardDamage?.byteLength ?? 0) +
    f.sparks.byteLength +
    160;
  for (const ps of Object.values(f.partStates)) {
    bytes +=
      (ps.deformedDisplacements?.byteLength ?? 0) +
      (ps.nodalStressMPa?.byteLength ?? 0) +
      (ps.nodalPlasticStrain?.byteLength ?? 0) +
      (ps.nodalDamage?.byteLength ?? 0);
  }
  return bytes;
}

function createGearDestructionSession(
  parts: CadPart[],
  joints: KinematicJoint[],
  materials: MaterialSpec[],
  config: DestructionSimConfig
) {
  const startMs =
    typeof performance !== 'undefined' ? performance.now() : Date.now();
  const cacheId = `dcache_${Date.now().toString(36)}_${config.seed}`;
  const safeFps = Math.max(24, Math.min(120, Math.round(config.fps || 60)));
  const safeDuration = Math.max(0.5, Math.min(6.0, config.durationSec || 2.5));
  const totalFrames = Math.round(safeDuration * safeFps);
  const dt = 1 / safeFps;
  const substeps = 8; // 480 Hz physics solver at 60 FPS -> crisp zero-tunneling tooth collision
  const subDt = dt / substeps;
  const rng = mulberry32(config.seed + 7919);

  const visibleParts = parts.filter((p) => p.visible);
  const destructibleParts = visibleParts.filter((p) => !isStaticSupportPart(p));
  const effectiveDestructibles =
    destructibleParts.length > 0 ? destructibleParts : visibleParts.slice(0, 2);
  const destructibleIds = new Set(effectiveDestructibles.map((p) => p.id));

  let floorY = computeFloorY(visibleParts);
  const fixedBench = visibleParts.find(
    (p) =>
      isStaticSupportPart(p) &&
      (p.id.includes('bench') || p.id.includes('rig_floor'))
  );
  if (fixedBench) {
    floorY =
      fixedBench.position[1] +
      fixedBench.mesh.boundingBox.max[1] * Math.abs(fixedBench.scale[1] || 1);
  }

  let latestContactWorld: Vec3 = computePrimaryContactPoint(
    effectiveDestructibles,
    config
  );

  // Build runtime state per part
  const partRuntimes: InternalPartRuntime[] = visibleParts.map((p) => {
    const mat = getMaterialById(p.materialId, materials);
    const isDestr = destructibleIds.has(p.id);
    const destrIdx = effectiveDestructibles.findIndex((d) => d.id === p.id);
    const collider = isDestr ? buildGearCollider(p) : null;

    let rpm = 0;
    if (isDestr) {
      if (destrIdx === 0) {
        rpm = config.motorRpmLeft;
      } else if (destrIdx === 1) {
        rpm = config.motorRpmRight;
      } else {
        const j = joints.find((jt) => jt.partB === p.id);
        rpm = j?.motorSpeedRpm ?? 40;
      }
    }

    const yieldMPa = Math.max(30, mat.yieldStrength / 1e6);
    const thresholdScale =
      1.05 - 0.35 * Math.max(0, Math.min(1, config.brittleness));
    const targetOmegaZ = (rpm * 2 * Math.PI) / 60;

    // Effective shaft+motor rotor inertia (kg*m^2) coupled to gear disk
    const gearMass = Math.max(0.02, p.mesh.mass || 0.12);
    const pitchR = collider?.pitchRadius ?? 0.022;
    const gearDiskI = 0.5 * gearMass * pitchR * pitchR;
    const motorRotorI = 0.018; // Realistic industrial drive shaft + rotor flywheel inertia

    const vCount = p.mesh.vertexCount;
    const vertRadiusXY = new Float32Array(isDestr ? vCount : 0);
    const vertAngle = new Float32Array(isDestr ? vCount : 0);
    if (isDestr) {
      const verts = p.mesh.vertices;
      for (let vi = 0; vi < vCount; vi++) {
        const vx = verts[vi * 3];
        const vy = verts[vi * 3 + 1];
        vertRadiusXY[vi] = Math.hypot(vx, vy);
        vertAngle[vi] = Math.atan2(vy, vx);
      }
    }

    return {
      part: p,
      material: mat,
      destructible: isDestr,
      collider,
      brokenTeeth: new Set<number>(),
      pos: [...p.position] as Vec3,
      rot: [...p.rotation] as Vec3,
      omegaZ: targetOmegaZ,
      targetOmegaZ,
      inertiaZ: gearDiskI + motorRotorI,
      gearDiskI,
      baseRpm: rpm,
      stressMPa: 0,
      yieldMPa,
      damage: 0,
      yieldLogged: false,
      stage1Broken: false,
      stage2Broken: false,
      fractureThresholdScale: thresholdScale,
      vertRadiusXY,
      vertAngle,
      plasticStrainVert: new Float32Array(isDestr ? vCount : 0),
      damageVert: new Float32Array(isDestr ? vCount : 0),
      plasticDispVert: new Float32Array(isDestr ? vCount * 3 : 0),
      plasticEnergyJ: 0,
      elasticEnergyJ: 0,
    };
  });

  const gearPair = partRuntimes.filter((pr) => pr.destructible && pr.collider);
  const hasGearPair = gearPair.length >= 2;
  const gearA = hasGearPair ? gearPair[0] : null;
  const gearB = hasGearPair ? gearPair[1] : null;

  // Ensure zero initial interpenetration if two gears start already meshed
  if (gearA && gearB) {
    alignInitialGearPhaseIfOverlapping(gearA, gearB);
  }

  // Determine whether gear pair starts with an approach air gap (clash engagement test)
  let approachActive = false;
  let targetCenterDist = 0;
  let approachDir: Vec2 = [1, 0];
  let approachSpeed = 0;
  if (gearA && gearB && gearA.collider && gearB.collider) {
    const dx0 = gearB.pos[0] - gearA.pos[0];
    const dy0 = gearB.pos[1] - gearA.pos[1];
    const d0 = Math.hypot(dx0, dy0);
    const sumOuter = gearA.collider.outerRadius + gearB.collider.outerRadius;
    targetCenterDist = gearA.collider.pitchRadius + gearB.collider.pitchRadius;
    if (d0 > sumOuter + 1e-4 && config.scenarioMode === 'counter_clash') {
      approachActive = true;
      approachDir = d0 > 1e-9 ? [dx0 / d0, dy0 / d0] : [1, 0];
      // Close the initial air gap smoothly over ~0.22s so user clearly sees opposing spin before impact
      const gap = d0 - sumOuter;
      approachSpeed = Math.max(0.012, (gap + 0.0012) / 0.22);
    }
  }

  // Pre-fracture all destructible parts into 3D solid shards
  const allShards: FractureShardGeometry[] = [];
  const shardRuntimes: InternalShardRuntime[] = [];

  for (let pIdx = 0; pIdx < partRuntimes.length; pIdx++) {
    const pr = partRuntimes[pIdx];
    if (!pr.destructible) continue;

    const invQ = quatFromEulerXYZ([-pr.rot[0], -pr.rot[1], -pr.rot[2]]);
    const contactLocal = rotateVecByQuat(sub3(latestContactWorld, pr.pos), invQ);

    const partShards = fracturePartIntoShards(
      pr.part,
      pr.material,
      config.shardCountPerPart,
      contactLocal,
      config.seed + pIdx * 1013
    );

    const q0 = quatFromEulerXYZ(pr.rot);
    for (const s of partShards) {
      const idx = allShards.length;
      allShards.push(s);
      const wPos = add3(pr.pos, rotateVecByQuat(s.localCom, q0));
      shardRuntimes.push({
        shardIdx: idx,
        parentPartId: pr.part.id,
        localCom: s.localCom,
        localAngle: Math.atan2(s.localCom[1], s.localCom[0]),
        localRadiusXY: Math.hypot(s.localCom[0], s.localCom[1]),
        boundingRadius: s.boundingRadius,
        mass: s.mass,
        active: false,
        pos: wPos,
        quat: q0,
        vel: [0, 0, 0],
        omega: [0, 0, 0],
        stressGlow: 0,
        stressMPa: 0,
        plasticStrain: 0,
        damage: 0,
      });
    }
  }

  const events: DestructionTimelineEvent[] = [];
  const frames: DestructionCacheFrame[] = [];
  const sparks: InternalSpark[] = [];
  let maxStressOverallMPa = 0;
  let maxPlasticStrainOverallPct = 0;
  let maxDamageOverall = 0;
  let maxEnergyOverallJ = 1e-4;
  let totalFractureEnergyJ = 0;
  let contactLogged = false;
  let floorBounceLogged = false;
  let firstContactFrame = -1;
  let hasHadRealContact = false;

  const minYieldMPa = Math.min(
    ...partRuntimes.filter((p) => p.destructible).map((p) => p.yieldMPa),
    350
  );

  const spawnSparks = (
    origin: Vec3,
    baseDir: Vec3,
    count: number,
    speedScale: number
  ) => {
    for (let i = 0; i < count; i++) {
      if (sparks.length >= 160) break;
      const spread: Vec3 = [
        (rng() - 0.5) * 2.2,
        (rng() - 0.25) * 2.4,
        (rng() - 0.5) * 2.0,
      ];
      const v = scale3(
        add3(baseDir, spread),
        speedScale * (0.45 + rng() * 0.9)
      );
      sparks.push({
        pos: [
          origin[0] + (rng() - 0.5) * 0.004,
          origin[1] + (rng() - 0.5) * 0.004,
          origin[2] + (rng() - 0.5) * 0.004,
        ],
        vel: v,
        life: 1.0,
        maxLife: 0.2 + rng() * 0.3,
      });
    }
  };

  /**
   * Detaches the exact shards closest to the current world contact point (Stage 1)
   * and marks their corresponding gear teeth as broken in SAT so they no longer block.
   */
  const triggerStage1Fracture = (
    pr: InternalPartRuntime,
    frameIdx: number,
    timeSec: number,
    contactPt: Vec3,
    hitToothIdx: number
  ) => {
    pr.stage1Broken = true;
    const toothLabel = hitToothIdx >= 0 ? ` #${hitToothIdx + 1}` : '';
    events.push({
      frameIndex: frameIdx,
      timeSec,
      type: 'tooth_shear',
      label: `Скол зуба${toothLabel}: ${pr.part.name}`,
      partId: pr.part.id,
      stressMPa: Number(pr.stressMPa.toFixed(1)),
    });

    const partQ = quatFromEulerXYZ(pr.rot);
    const candidates = shardRuntimes
      .filter((sr) => sr.parentPartId === pr.part.id && !sr.active)
      .map((sr) => {
        const wPos = add3(pr.pos, rotateVecByQuat(sr.localCom, partQ));
        return { sr, wPos, dist: len3(sub3(wPos, contactPt)) };
      })
      .sort((u, v) => u.dist - v.dist);

    const detachCount = Math.max(
      2,
      Math.min(candidates.length, Math.round(candidates.length * 0.36))
    );

    if (detachCount > 0) {
      events.push({
        frameIndex: frameIdx,
        timeSec,
        type: 'fragment_detachment',
        label: `Отделение фрагментов (${detachCount} шт., ступица цела): ${pr.part.name}`,
        partId: pr.part.id,
        stressMPa: Number(pr.stressMPa.toFixed(1)),
      });
    }

    for (let i = 0; i < detachCount; i++) {
      const { sr, wPos } = candidates[i];
      sr.active = true;
      sr.pos = wPos;
      sr.quat = partQ;
      sr.stressGlow = 1.0;
      sr.stressMPa = pr.stressMPa;
      sr.plasticStrain = Math.max(
        sr.plasticStrain,
        Math.min(0.18, pr.material.fractureStrain * 0.65)
      );
      sr.damage = 1.0;

      // Mark tooth sectors belonging to this sheared rim shard as broken in SAT
      if (pr.collider && sr.localRadiusXY > pr.collider.rootRadius * 0.65) {
        const teeth = pr.collider.teeth;
        const normA = ((sr.localAngle % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2);
        const nearestTooth = Math.round((normA / (Math.PI * 2)) * teeth) % teeth;
        pr.brokenTeeth.add(nearestTooth);
        pr.brokenTeeth.add((nearestTooth + 1) % teeth);
        pr.brokenTeeth.add((nearestTooth - 1 + teeth) % teeth);
      }

      const relFromAxis = sub3(sr.pos, pr.pos);
      const vTangential: Vec3 = [
        -pr.omegaZ * relFromAxis[1],
        pr.omegaZ * relFromAxis[0],
        0,
      ];
      const outFromContact = normalize3(
        add3(sub3(sr.pos, contactPt), [
          (rng() - 0.5) * 0.014,
          0.014 + rng() * 0.025,
          (rng() - 0.5) * 0.024,
        ])
      );
      const blastSpeed =
        (0.45 + config.impactTorqueNm * 0.0028) *
        (0.75 + 0.55 * config.brittleness) *
        (0.8 + rng() * 0.45);
      sr.vel = add3(vTangential, scale3(outFromContact, blastSpeed));
      sr.omega = [
        (rng() - 0.5) * 28,
        (rng() - 0.5) * 28,
        pr.omegaZ * 1.3 + (rng() - 0.5) * 22,
      ];
      const crackAreaM2 = Math.PI * sr.boundingRadius * sr.boundingRadius * 0.35;
      const gcJm2 = Math.max(100, pr.material.fractureEnergyJM2 || 15000);
      totalFractureEnergyJ +=
        0.5 * sr.mass * dot3(sr.vel, sr.vel) + gcJm2 * crackAreaM2 * 0.08;
    }

    if (pr.collider && hitToothIdx >= 0) {
      const teeth = pr.collider.teeth;
      pr.brokenTeeth.add(hitToothIdx);
      pr.brokenTeeth.add((hitToothIdx + 1) % teeth);
      pr.brokenTeeth.add((hitToothIdx - 1 + teeth) % teeth);
    }

    spawnSparks(
      contactPt,
      [0, 0.9, (rng() - 0.5) * 0.8],
      28,
      0.8 + config.impactTorqueNm * 0.002
    );
  };

  /** Detaches all remaining shards of a part (Stage 2 full structural rupture) */
  const triggerStage2Fracture = (
    pr: InternalPartRuntime,
    frameIdx: number,
    timeSec: number
  ) => {
    pr.stage2Broken = true;
    events.push({
      frameIndex: frameIdx,
      timeSec,
      type: 'body_fracture',
      label: `Раскол корпуса: ${pr.part.name}`,
      partId: pr.part.id,
      stressMPa: Number(pr.stressMPa.toFixed(1)),
    });

    const partQ = quatFromEulerXYZ(pr.rot);
    for (const sr of shardRuntimes) {
      if (sr.parentPartId !== pr.part.id || sr.active) continue;
      sr.active = true;
      sr.pos = add3(pr.pos, rotateVecByQuat(sr.localCom, partQ));
      sr.quat = partQ;
      sr.stressGlow = 0.92;
      sr.stressMPa = pr.stressMPa;
      sr.plasticStrain = Math.max(
        sr.plasticStrain,
        Math.min(0.22, pr.material.fractureStrain * 0.85)
      );
      sr.damage = 1.0;

      const relFromAxis = sub3(sr.pos, pr.pos);
      const vTangential: Vec3 = [
        -pr.omegaZ * relFromAxis[1],
        pr.omegaZ * relFromAxis[0],
        0,
      ];
      const radialDir = normalize3(
        add3(relFromAxis, [
          (rng() - 0.5) * 0.012,
          0.008 + rng() * 0.016,
          (rng() - 0.5) * 0.022,
        ])
      );
      const shatterSpeed =
        (0.28 + config.impactTorqueNm * 0.0018) *
        (0.65 + 0.55 * config.brittleness) *
        (0.75 + rng() * 0.5);
      sr.vel = add3(
        scale3(vTangential, 0.92),
        scale3(radialDir, shatterSpeed)
      );
      sr.omega = [
        (rng() - 0.5) * 18,
        (rng() - 0.5) * 18,
        pr.omegaZ + (rng() - 0.5) * 16,
      ];
      const crackAreaM2 = Math.PI * sr.boundingRadius * sr.boundingRadius * 0.35;
      const gcJm2 = Math.max(100, pr.material.fractureEnergyJM2 || 15000);
      totalFractureEnergyJ +=
        0.5 * sr.mass * dot3(sr.vel, sr.vel) + gcJm2 * crackAreaM2 * 0.08;
    }

    spawnSparks(
      pr.pos,
      [0, 0.65, 0],
      22,
      0.6 + config.impactTorqueNm * 0.0016
    );
  };

  // Single-part crush timing (only used when there is no colliding gear pair)
  const impactCrushTimeSec = Math.min(0.28, safeDuration * 0.14);

  const partRtById = new Map<string, InternalPartRuntime>();
  for (const pr of partRuntimes) partRtById.set(pr.part.id, pr);

  let shardGeometryBytes = 0;
  for (const s of allShards) {
    shardGeometryBytes +=
      s.vertices.byteLength +
      s.normals.byteLength +
      s.indices.byteLength +
      s.innerVertices.byteLength +
      s.innerNormals.byteLength +
      s.innerIndices.byteLength;
  }
  let frameCacheBytes = 0;

  const stepFrame = (frameIdx: number): DestructionCacheFrame => {
    const timeSec = Number((frameIdx * dt).toFixed(4));

    let framePeakStress = 0;
    let frameContactForceN = 0;
    let frameHadGearContact = false;
    let lastHitToothA = -1;
    let lastHitToothB = -1;

    if (frameIdx > 0) {
      if (hasGearPair && gearA && gearB && gearA.collider && gearB.collider) {
        // ====================================================================
        // 480 Hz SUBSTEPPED RIGID-BODY + EXACT 2D SAT GEAR-TOOTH COLLISION LOOP
        // ====================================================================
        let substepImpulseSum = 0;
        let maxImpactRelSpeed = 0;
        const bothBodiesExist = !gearA.stage2Broken && !gearB.stage2Broken;

        // Check if gears are commanded in proper meshing directions (opposite signs) vs opposing clash
        const sameDirectionClash =
          gearA.targetOmegaZ * gearB.targetOmegaZ > 0 ||
          config.scenarioMode === 'counter_clash';

        // In torque_overload, resistive brake torque on driven Gear B ramps up smoothly while meshing cleanly
        const overloadRamp =
          config.scenarioMode === 'torque_overload'
            ? Math.min(1.0, Math.max(0.15, timeSec / Math.max(0.32, safeDuration * 0.24)))
            : 1.0;

        for (let s = 0; s < substeps; s++) {
          // 1. Motor & brake torque integration
          const maxMotorTorque = Math.max(1, config.impactTorqueNm);
          const kp = 45.0; // Proportional servo governor gain

          if (!gearA.stage2Broken) {
            const errA = gearA.targetOmegaZ - gearA.omegaZ;
            let tauA = errA * kp * gearA.inertiaZ;
            if (
              gearA.targetOmegaZ !== 0 &&
              errA * Math.sign(gearA.targetOmegaZ) > 0.25
            ) {
              tauA =
                Math.sign(gearA.targetOmegaZ) * maxMotorTorque * overloadRamp;
            }
            tauA = Math.max(-maxMotorTorque, Math.min(maxMotorTorque, tauA));
            gearA.omegaZ += (tauA / gearA.inertiaZ) * subDt;
            if (
              gearA.targetOmegaZ > 0 &&
              gearA.omegaZ > gearA.targetOmegaZ &&
              tauA > 0
            ) {
              gearA.omegaZ = gearA.targetOmegaZ;
            } else if (
              gearA.targetOmegaZ < 0 &&
              gearA.omegaZ < gearA.targetOmegaZ &&
              tauA < 0
            ) {
              gearA.omegaZ = gearA.targetOmegaZ;
            }
          }

          if (!gearB.stage2Broken) {
            if (config.scenarioMode === 'torque_overload' && !sameDirectionClash) {
              // Driven wheel under progressive braking torque opposing its kinematic rotation
              const brakeDir =
                gearB.omegaZ !== 0
                  ? -Math.sign(gearB.omegaZ)
                  : -Math.sign(gearB.targetOmegaZ || -1);
              const tauBrake = brakeDir * maxMotorTorque * overloadRamp;
              gearB.omegaZ += (tauBrake / gearB.inertiaZ) * subDt;
              if (gearB.targetOmegaZ < 0 && gearB.omegaZ > 0) {
                gearB.omegaZ = 0;
              } else if (gearB.targetOmegaZ > 0 && gearB.omegaZ < 0) {
                gearB.omegaZ = 0;
              }
            } else {
              // Independent opposing drive motor on Gear B with synchronous speed governor
              const errB = gearB.targetOmegaZ - gearB.omegaZ;
              let tauB = errB * kp * gearB.inertiaZ;
              if (
                gearB.targetOmegaZ !== 0 &&
                errB * Math.sign(gearB.targetOmegaZ) > 0.25
              ) {
                tauB = Math.sign(gearB.targetOmegaZ) * maxMotorTorque;
              }
              tauB = Math.max(-maxMotorTorque, Math.min(maxMotorTorque, tauB));
              gearB.omegaZ += (tauB / gearB.inertiaZ) * subDt;
              if (
                gearB.targetOmegaZ > 0 &&
                gearB.omegaZ > gearB.targetOmegaZ &&
                tauB > 0
              ) {
                gearB.omegaZ = gearB.targetOmegaZ;
              } else if (
                gearB.targetOmegaZ < 0 &&
                gearB.omegaZ < gearB.targetOmegaZ &&
                tauB < 0
              ) {
                gearB.omegaZ = gearB.targetOmegaZ;
              }
            }
          }

          // 2. Smooth test-bench carriage engagement ONLY while air gap remains (stops on tooth contact!)
          const prevPosA0 = gearA.pos[0];
          const prevPosA1 = gearA.pos[1];
          const prevPosB0 = gearB.pos[0];
          const prevPosB1 = gearB.pos[1];

          if (approachActive && bothBodiesExist) {
            const curDist = Math.hypot(
              gearB.pos[0] - gearA.pos[0],
              gearB.pos[1] - gearA.pos[1]
            );
            if (curDist > targetCenterDist + 0.0004) {
              const step = Math.min(
                approachSpeed * subDt,
                curDist - targetCenterDist
              );
              gearA.pos[0] += approachDir[0] * step * 0.5;
              gearA.pos[1] += approachDir[1] * step * 0.5;
              gearB.pos[0] -= approachDir[0] * step * 0.5;
              gearB.pos[1] -= approachDir[1] * step * 0.5;
            } else {
              approachActive = false;
            }
          }

          // 3. Advance angular positions
          if (!gearA.stage2Broken) gearA.rot[2] += gearA.omegaZ * subDt;
          if (!gearB.stage2Broken) gearB.rot[2] += gearB.omegaZ * subDt;

          // 4. Exact 2D SAT tooth-polygon collision detection & hard non-penetration solver
          if (bothBodiesExist) {
            for (let iter = 0; iter < 5; iter++) {
              const hits = detectGearPairContacts(
                gearA.collider,
                [gearA.pos[0], gearA.pos[1]],
                gearA.rot[2],
                gearB.collider,
                [gearB.pos[0], gearB.pos[1]],
                gearB.rot[2],
                gearA.brokenTeeth,
                gearB.brokenTeeth
              );
              if (hits.length === 0) break;

              frameHadGearContact = true;
              hasHadRealContact = true;

              // If carriage feed caused tooth-tip contact, immediately arrest radial feed so shafts never crush teeth together
              if (approachActive) {
                gearA.pos[0] = prevPosA0;
                gearA.pos[1] = prevPosA1;
                gearB.pos[0] = prevPosB0;
                gearB.pos[1] = prevPosB1;
                // Keep carriage engaged once tooth contact starts; allow small re-approach only after Stage 1 teeth shear off
                if (gearA.stage1Broken || gearB.stage1Broken) {
                  // Allow brief approach to bring next intact teeth into clash after Stage 1 shear
                } else {
                  approachActive = false;
                }
              }

              // Re-evaluate contact at exact non-fed position if feed was reverted on iter 0
              const activeHits =
                iter === 0
                  ? detectGearPairContacts(
                      gearA.collider,
                      [gearA.pos[0], gearA.pos[1]],
                      gearA.rot[2],
                      gearB.collider,
                      [gearB.pos[0], gearB.pos[1]],
                      gearB.rot[2],
                      gearA.brokenTeeth,
                      gearB.brokenTeeth
                    )
                  : hits;
              if (activeHits.length === 0) break;

              for (let h = 0; h < Math.min(3, activeHits.length); h++) {
                const c = activeHits[h];
                const nx = c.normal[0];
                const ny = c.normal[1];
                const rAx = c.point[0] - gearA.pos[0];
                const rAy = c.point[1] - gearA.pos[1];
                const rBx = c.point[0] - gearB.pos[0];
                const rBy = c.point[1] - gearB.pos[1];

                // 2D cross product (r x n)_z = tangential lever arm along contact normal
                const jA = rAx * ny - rAy * nx;
                const jB = rBx * ny - rBy * nx;
                const K =
                  (jA * jA) / gearA.inertiaZ + (jB * jB) / gearB.inertiaZ;

                const vRelN = gearB.omegaZ * jB - gearA.omegaZ * jA;
                if (vRelN < 0) {
                  if (-vRelN > maxImpactRelSpeed) maxImpactRelSpeed = -vRelN;
                }

                if (K > 1e-10) {
                  // Hard rotational non-penetration velocity impulse
                  if (vRelN < 0) {
                    const e =
                      config.scenarioMode === 'torque_overload' ? 0.04 : 0.14;
                    const imp = (-(1 + e) * vRelN) / K;
                    gearA.omegaZ -= (jA / gearA.inertiaZ) * imp;
                    gearB.omegaZ += (jB / gearB.inertiaZ) * imp;
                    substepImpulseSum += imp;
                  }

                  // Exact positional projection -> drives penetration to ~0.00 mm
                  if (c.depth > 1e-6) {
                    const dLambda = c.depth / K;
                    gearA.rot[2] -= (jA / gearA.inertiaZ) * dLambda;
                    gearB.rot[2] += (jB / gearB.inertiaZ) * dLambda;
                  }
                } else if (c.depth > 1e-6) {
                  // Pure radial tip-to-tip singularity (jA ~ 0 & jB ~ 0): separate shafts along normal
                  const sep = c.depth * 0.55;
                  gearA.pos[0] -= nx * sep;
                  gearA.pos[1] -= ny * sep;
                  gearB.pos[0] += nx * sep;
                  gearB.pos[1] += ny * sep;
                }

                if (h === 0) {
                  latestContactWorld = [
                    c.point[0],
                    c.point[1],
                    0.5 * (gearA.pos[2] + gearB.pos[2]),
                  ];
                  if (c.toothIdxA >= 0) lastHitToothA = c.toothIdxA;
                  if (c.toothIdxB >= 0) lastHitToothB = c.toothIdxB;
                }
              }
            }
          }
        }

        // Re-enable gentle carriage feed briefly if Stage 1 sheared the tip teeth and gears still have gap
        if (
          config.scenarioMode === 'counter_clash' &&
          bothBodiesExist &&
          (gearA.stage1Broken || gearB.stage1Broken) &&
          !frameHadGearContact
        ) {
          const curDist = Math.hypot(
            gearB.pos[0] - gearA.pos[0],
            gearB.pos[1] - gearA.pos[1]
          );
          if (curDist > targetCenterDist + 0.0005) {
            approachActive = true;
          }
        }

        // Log first physical contact event on exact SAT collision frame
        if (frameHadGearContact && !contactLogged) {
          contactLogged = true;
          firstContactFrame = frameIdx;
          events.push({
            frameIndex: frameIdx,
            timeSec,
            type: 'contact',
            label:
              config.scenarioMode === 'counter_clash'
                ? 'Упругий контакт зубьев (встречный удар)'
                : 'Упругий контакт и рост нагрузки в зацеплении',
            stressMPa: 0,
          });
          spawnSparks(
            latestContactWorld,
            [0, 0.95, (rng() - 0.5) * 0.8],
            22,
            0.75 + Math.min(1.5, maxImpactRelSpeed * 2.2)
          );
        }

        // Compute real contact normal force F_n (Newtons) from exact SAT impulses
        const measuredImpulseForceN = substepImpulseSum / dt;
        // Static jam clamping floor when teeth are locked against opposing motors
        const staticStallForceN =
          frameHadGearContact && sameDirectionClash
            ? config.impactTorqueNm / Math.max(0.01, gearA.collider.pitchRadius)
            : 0;
        frameContactForceN = Math.max(measuredImpulseForceN, staticStallForceN);

        // Emit continuous grinding sparks while teeth are jammed under high contact force
        if (frameHadGearContact && frameContactForceN > 1500 && bothBodiesExist) {
          spawnSparks(
            latestContactWorld,
            [0, 0.75, (rng() - 0.5) * 0.7],
            6,
            0.55 + Math.min(1.2, frameContactForceN / 9000)
          );
        }

        // Evaluate ISO/Lewis tooth root bending stress (MPa) & progressive fracture damage per gear
        const impactShockFactor = 1.0 + Math.min(1.6, maxImpactRelSpeed * 2.4);
        for (const pr of [gearA, gearB]) {
          if (pr.stage2Broken) {
            pr.stressMPa *= 0.84;
            continue;
          }

          if (frameHadGearContact && pr.collider) {
            // Real Lewis tooth bending stress: sigma = (K_v * F_n) / (b * m * Y) in Pascals -> MPa
            const denomM2 =
              pr.collider.faceWidthM * pr.collider.moduleM * pr.collider.lewisY;
            const lewisMPa =
              (impactShockFactor * frameContactForceN) /
              Math.max(1e-7, denomM2) /
              1e6;
            pr.stressMPa = Math.max(pr.stressMPa * 0.65, lewisMPa);
          } else {
            // Free-spinning / post-separation elastic stress relaxation
            pr.stressMPa = Math.max(
              pr.stressMPa * 0.78,
              Math.min(pr.yieldMPa * 0.08, config.impactTorqueNm * 0.06)
            );
          }

          if (pr.stressMPa > framePeakStress) framePeakStress = pr.stressMPa;
          if (pr.stressMPa > maxStressOverallMPa) {
            maxStressOverallMPa = pr.stressMPa;
          }

          if (!pr.yieldLogged && pr.stressMPa >= pr.yieldMPa) {
            pr.yieldLogged = true;
            events.push({
              frameIndex: frameIdx,
              timeSec,
              type: 'yield_onset',
              label: `Начало текучести σ_vM > σ_y: ${pr.part.name}`,
              partId: pr.part.id,
              stressMPa: Number(pr.stressMPa.toFixed(1)),
            });
          }

          const stage1Limit = pr.yieldMPa * pr.fractureThresholdScale * 0.82;
          const stage2Limit = pr.yieldMPa * pr.fractureThresholdScale * 1.1;

          // Accumulate fracture damage ONLY when real stress exceeds material tooth-root yield limit
          if (pr.stressMPa >= stage1Limit) {
            const overloadRatio = pr.stressMPa / Math.max(1, stage1Limit);
            const damageRate =
              (overloadRatio - 0.72) * (7.5 + 14.0 * config.brittleness);
            pr.damage += damageRate * dt;
          }

          const hitTooth = pr === gearA ? lastHitToothA : lastHitToothB;

          if (!pr.stage1Broken && pr.damage >= 1.0) {
            triggerStage1Fracture(
              pr,
              frameIdx,
              timeSec,
              latestContactWorld,
              hitTooth
            );
          }

          if (
            !pr.stage2Broken &&
            pr.stage1Broken &&
            (pr.damage >= 1.75 ||
              (pr.stressMPa >= stage2Limit && pr.damage >= 1.45))
          ) {
            triggerStage2Fracture(pr, frameIdx, timeSec);
          }
        }

        // Spin any extra non-pair destructible parts if present
        for (const pr of partRuntimes) {
          if (!pr.destructible || pr === gearA || pr === gearB) continue;
          if (!pr.stage2Broken) {
            pr.rot[2] += pr.omegaZ * dt;
          }
        }
      } else {
        // ====================================================================
        // SINGLE PART / ARBITRARY CAD MODEL (impact_crush)
        // ====================================================================
        const isCrushActive = timeSec >= impactCrushTimeSec;
        if (isCrushActive && !contactLogged) {
          contactLogged = true;
          firstContactFrame = frameIdx;
          events.push({
            frameIndex: frameIdx,
            timeSec,
            type: 'contact',
            label: 'Ударное нагружение детали',
            stressMPa: 0,
          });
        }

        for (const pr of partRuntimes) {
          if (!pr.destructible) continue;
          if (!pr.stage2Broken) {
            pr.rot[2] += pr.omegaZ * dt;
          }

          if (isCrushActive && !pr.stage2Broken) {
            const elapsed = timeSec - impactCrushTimeSec;
            const pitchR = Math.max(
              0.01,
              0.25 *
                ((pr.part.mesh.boundingBox.max[0] -
                  pr.part.mesh.boundingBox.min[0]) +
                  (pr.part.mesh.boundingBox.max[1] -
                    pr.part.mesh.boundingBox.min[1]))
            );
            const ft = config.impactTorqueNm / pitchR;
            frameContactForceN = Math.max(frameContactForceN, ft);
            const rampRate = 5.0 + config.brittleness * 3.0;
            pr.stressMPa = ft * 0.042 * (0.55 + elapsed * rampRate);
          } else if (pr.stage2Broken) {
            pr.stressMPa *= 0.86;
          } else {
            pr.stressMPa = Math.min(
              pr.yieldMPa * 0.15,
              config.impactTorqueNm * 0.12
            );
          }

          if (pr.stressMPa > framePeakStress) framePeakStress = pr.stressMPa;
          if (pr.stressMPa > maxStressOverallMPa) {
            maxStressOverallMPa = pr.stressMPa;
          }

          if (!pr.yieldLogged && pr.stressMPa >= pr.yieldMPa) {
            pr.yieldLogged = true;
            events.push({
              frameIndex: frameIdx,
              timeSec,
              type: 'yield_onset',
              label: `Начало текучести σ_vM > σ_y: ${pr.part.name}`,
              partId: pr.part.id,
              stressMPa: Number(pr.stressMPa.toFixed(1)),
            });
          }

          const stage1Limit = pr.yieldMPa * pr.fractureThresholdScale * 0.82;
          const stage2Limit = pr.yieldMPa * pr.fractureThresholdScale * 1.08;

          if (!pr.stage1Broken && pr.stressMPa >= stage1Limit) {
            triggerStage1Fracture(
              pr,
              frameIdx,
              timeSec,
              latestContactWorld,
              -1
            );
          }
          if (!pr.stage2Broken && pr.stressMPa >= stage2Limit) {
            triggerStage2Fracture(pr, frameIdx, timeSec);
          }
        }
      }

      // ====================================================================
      // INTEGRATE SHARDS (RIGID-BODY + BENCH COLLISION + PAIRWISE SHARD COLLISION)
      // ====================================================================
      for (const sr of shardRuntimes) {
        if (!sr.active) {
          const parent = partRtById.get(sr.parentPartId);
          if (parent) {
            const q = quatFromEulerXYZ(parent.rot);
            sr.pos = add3(parent.pos, rotateVecByQuat(sr.localCom, q));
            sr.quat = q;
            const ratio = parent.stressMPa / Math.max(1, parent.yieldMPa);
            sr.stressGlow = Math.max(0, Math.min(1, ratio * 0.85));
            // Localize stress/plasticity/damage on rim shards closest to contact point
            const distToC = len3(sub3(sr.pos, latestContactWorld));
            const locality = Math.exp(-Math.pow(distToC / 0.016, 2));
            const localStress = parent.stressMPa * (0.18 + 0.82 * locality);
            sr.stressMPa = localStress;
            if (localStress > parent.yieldMPa) {
              const epInc =
                ((localStress - parent.yieldMPa) /
                  Math.max(120, parent.material.hardeningModulusPa / 1e6 + 900)) *
                dt *
                0.8;
              sr.plasticStrain = Math.min(0.25, sr.plasticStrain + epInc);
              sr.damage = Math.min(
                1.0,
                sr.damage +
                  (epInc / Math.max(0.01, parent.material.fractureStrain)) * 1.4
              );
            }
          }
          continue;
        }

        sr.vel[1] += config.gravityY * dt;
        sr.vel[0] *= 0.996;
        sr.vel[1] *= 0.997;
        sr.vel[2] *= 0.996;

        sr.pos[0] += sr.vel[0] * dt;
        sr.pos[1] += sr.vel[1] * dt;
        sr.pos[2] += sr.vel[2] * dt;

        sr.quat = integrateQuat(sr.quat, sr.omega, dt);
        sr.omega[0] *= 0.992;
        sr.omega[1] *= 0.992;
        sr.omega[2] *= 0.992;

        sr.stressGlow = Math.max(0, sr.stressGlow - dt * 1.15);

        const minShardY = floorY + sr.boundingRadius * 0.55;
        if (sr.pos[1] < minShardY) {
          sr.pos[1] = minShardY;
          if (sr.vel[1] < -0.06) {
            if (!floorBounceLogged) {
              floorBounceLogged = true;
              events.push({
                frameIndex: frameIdx,
                timeSec,
                type: 'floor_bounce',
                label: 'Удар осколков о стенд',
              });
            }
            sr.vel[1] =
              -sr.vel[1] * Math.max(0.08, Math.min(0.85, config.restitution));
            sr.vel[0] *= 0.78;
            sr.vel[2] *= 0.78;
            sr.omega[0] *= 0.72;
            sr.omega[1] *= 0.72;
            sr.omega[2] *= 0.72;
          } else {
            sr.vel[1] = 0;
            sr.vel[0] *= 0.84;
            sr.vel[2] *= 0.84;
            sr.omega[0] *= 0.76;
            sr.omega[1] *= 0.76;
            sr.omega[2] *= 0.76;
          }
        }
      }

      // Pairwise active shard collision impulse
      const activeIndices: number[] = [];
      for (let i = 0; i < shardRuntimes.length; i++) {
        if (shardRuntimes[i].active) activeIndices.push(i);
      }
      for (let a = 0; a < activeIndices.length; a++) {
        const sa = shardRuntimes[activeIndices[a]];
        for (let b = a + 1; b < activeIndices.length; b++) {
          const sb = shardRuntimes[activeIndices[b]];
          const dx = sb.pos[0] - sa.pos[0];
          const dy = sb.pos[1] - sa.pos[1];
          const dz = sb.pos[2] - sa.pos[2];
          const minDist = (sa.boundingRadius + sb.boundingRadius) * 0.62;
          const distSq = dx * dx + dy * dy + dz * dz;
          if (distSq > 1e-10 && distSq < minDist * minDist) {
            const dist = Math.sqrt(distSq);
            const nx = dx / dist;
            const ny = dy / dist;
            const nz = dz / dist;
            const overlap = (minDist - dist) * 0.5;
            sa.pos[0] -= nx * overlap;
            sa.pos[1] -= ny * overlap;
            sa.pos[2] -= nz * overlap;
            sb.pos[0] += nx * overlap;
            sb.pos[1] += ny * overlap;
            sb.pos[2] += nz * overlap;

            const rvx = sb.vel[0] - sa.vel[0];
            const rvy = sb.vel[1] - sa.vel[1];
            const rvz = sb.vel[2] - sa.vel[2];
            const velAlongN = rvx * nx + rvy * ny + rvz * nz;
            if (velAlongN < 0) {
              const e = Math.max(0.15, config.restitution * 0.85);
              const invMassA = 1 / sa.mass;
              const invMassB = 1 / sb.mass;
              const jImp = (-(1 + e) * velAlongN) / (invMassA + invMassB);
              sa.vel[0] -= jImp * invMassA * nx;
              sa.vel[1] -= jImp * invMassA * ny;
              sa.vel[2] -= jImp * invMassA * nz;
              sb.vel[0] += jImp * invMassB * nx;
              sb.vel[1] += jImp * invMassB * ny;
              sb.vel[2] += jImp * invMassB * nz;
            }
          }
        }
      }

      // Integrate sparks
      for (let i = sparks.length - 1; i >= 0; i--) {
        const sp = sparks[i];
        sp.life -= dt / sp.maxLife;
        if (sp.life <= 0) {
          sparks.splice(i, 1);
          continue;
        }
        sp.vel[1] += config.gravityY * 0.7 * dt;
        sp.pos[0] += sp.vel[0] * dt;
        sp.pos[1] += sp.vel[1] * dt;
        sp.pos[2] += sp.vel[2] * dt;
        if (sp.pos[1] < floorY + 0.001) {
          sp.pos[1] = floorY + 0.001;
          sp.vel[1] = Math.abs(sp.vel[1]) * 0.45;
          sp.vel[0] *= 0.8;
          sp.vel[2] *= 0.8;
        }
      }
    }

    // Snapshot frame into compact typed arrays
    const shardPos = new Float32Array(shardRuntimes.length * 3);
    const shardQuat = new Float32Array(shardRuntimes.length * 4);
    const shardActive = new Uint8Array(shardRuntimes.length);
    const shardStress = new Float32Array(shardRuntimes.length);
    const shardStressMPa = new Float32Array(shardRuntimes.length);
    const shardPlasticStrain = new Float32Array(shardRuntimes.length);
    const shardDamage = new Float32Array(shardRuntimes.length);

    let activeShardsCount = 0;
    let kineticEnergyJ = 0;
    let frameMaxPlasticPct = 0;
    let frameMaxDamage = 0;
    let frameMaxDispMm = 0;

    for (let i = 0; i < shardRuntimes.length; i++) {
      const sr = shardRuntimes[i];
      shardPos[i * 3] = sr.pos[0];
      shardPos[i * 3 + 1] = sr.pos[1];
      shardPos[i * 3 + 2] = sr.pos[2];
      shardQuat[i * 4] = sr.quat[0];
      shardQuat[i * 4 + 1] = sr.quat[1];
      shardQuat[i * 4 + 2] = sr.quat[2];
      shardQuat[i * 4 + 3] = sr.quat[3];
      shardActive[i] = sr.active ? 1 : 0;
      shardStress[i] = sr.stressGlow;
      shardStressMPa[i] = sr.stressMPa;
      shardPlasticStrain[i] = sr.plasticStrain;
      shardDamage[i] = sr.damage;
      if (sr.active) {
        activeShardsCount++;
        kineticEnergyJ += 0.5 * sr.mass * dot3(sr.vel, sr.vel);
        if (sr.plasticStrain * 100 > frameMaxPlasticPct) {
          frameMaxPlasticPct = sr.plasticStrain * 100;
        }
        if (sr.damage > frameMaxDamage) frameMaxDamage = sr.damage;
      }
    }

    const packedSparks = new Float32Array(sparks.length * 7);
    for (let i = 0; i < sparks.length; i++) {
      const sp = sparks[i];
      const o = i * 7;
      packedSparks[o] = sp.pos[0];
      packedSparks[o + 1] = sp.pos[1];
      packedSparks[o + 2] = sp.pos[2];
      packedSparks[o + 3] = sp.vel[0];
      packedSparks[o + 4] = sp.vel[1];
      packedSparks[o + 5] = sp.vel[2];
      packedSparks[o + 6] = Math.max(0, Math.min(1, sp.life));
    }

    const partStates: Record<string, CachedPartFrameState> = {};
    let fracturedPartsCount = 0;
    let intactPartsCount = 0;
    let totalElasticEnergyJ = 0;
    let totalPlasticEnergyJ = 0;

    for (const pr of partRuntimes) {
      const intact = !pr.stage1Broken && !pr.stage2Broken;
      if (pr.destructible) {
        if (intact) intactPartsCount++;
        else fracturedPartsCount++;
        if (!pr.stage2Broken) {
          // Include coupled drive-shaft + spinning gear rotational kinetic energy
          const intactMassFrac = pr.stage1Broken ? 0.64 : 1.0;
          const effInertia = pr.gearDiskI + pr.inertiaZ * 0.14;
          kineticEnergyJ += 0.5 * effInertia * intactMassFrac * pr.omegaZ * pr.omegaZ;
        }
      }

      let deformedDisplacements: Float32Array | undefined;
      let nodalStressMPa: Float32Array | undefined;
      let nodalPlasticStrain: Float32Array | undefined;
      let nodalDamage: Float32Array | undefined;
      let partMaxDispMm = 0;
      let partMaxPlasticPct = 0;
      let partMaxDamage = 0;

      if (pr.destructible && !pr.stage2Broken && pr.part.mesh.vertexCount > 0) {
        const vCount = pr.part.mesh.vertexCount;
        const disp = new Float32Array(vCount * 3);
        const stressArr = new Float32Array(vCount);
        const epArr = new Float32Array(vCount);
        const dmgArr = new Float32Array(vCount);

        // Transform world contact point into part-local XY coordinates
        const dxW = latestContactWorld[0] - pr.pos[0];
        const dyW = latestContactWorld[1] - pr.pos[1];
        const cosR = Math.cos(-pr.rot[2]);
        const sinR = Math.sin(-pr.rot[2]);
        const contactLocalX = dxW * cosR - dyW * sinR;
        const contactLocalY = dxW * sinR + dyW * cosR;
        const contactAngle = Math.atan2(contactLocalY, contactLocalX);

        const outerR =
          pr.collider?.outerRadius ??
          Math.max(
            0.012,
            0.5 * (pr.part.mesh.boundingBox.max[0] - pr.part.mesh.boundingBox.min[0])
          );
        const rootR = pr.collider?.rootRadius ?? outerR * 0.78;
        const hubR = rootR * 0.52;
        const teeth = pr.collider?.teeth ?? 18;
        const angularSigma = Math.max(0.22, (Math.PI * 2) / Math.max(8, teeth) * 1.15);
        const invAngSigmaSq = 1 / (angularSigma * angularSigma);

        const youngsMPa = Math.max(1000, pr.material.youngsModulus / 1e6);
        const hardeningMPa = Math.max(
          150,
          pr.material.hardeningModulusPa / 1e6 || youngsMPa * 0.02
        );
        const epsFrac = Math.max(0.015, pr.material.fractureStrain || 0.12);
        // Physical elastic cantilever tooth deflection amplitude (meters): u ~ (σ / E) * toothHeight * 2.2
        const toothHeightM = Math.max(0.0025, outerR - rootR);
        const elasticDeflectM =
          (pr.stressMPa / youngsMPa) * toothHeightM * 2.4;
        // Tangential push direction opposes gear spin into the clash contact
        const tangSign = pr.omegaZ >= 0 ? -1 : 1;

        const verts = pr.part.mesh.vertices;
        const vRadii = pr.vertRadiusXY;
        const vAngles = pr.vertAngle;
        const pEp = pr.plasticStrainVert;
        const pDmg = pr.damageVert;
        const pDisp = pr.plasticDispVert;

        let partPlasticIncSum = 0;
        let stressSqSumMPa2 = 0;

        for (let vi = 0; vi < vCount; vi++) {
          const r = vRadii[vi];
          const ang = vAngles[vi];
          const dAng = wrapAnglePi(ang - contactAngle);
          const angWeight = Math.exp(-dAng * dAng * invAngSigmaSq);

          // Radial profile:
          // - Stress peaks sharply in tooth root fillet (r ≈ rootR) with Kt gradient decaying into intact hub
          const filletDist = (r - rootR) / Math.max(0.0015, toothHeightM * 0.55);
          const filletWeight = Math.exp(-filletDist * filletDist * 0.75);
          const rimWeight = Math.max(
            0,
            Math.min(1, (r - hubR) / Math.max(0.002, outerR - hubR))
          );
          const tipBendWeight = Math.pow(
            Math.max(0, Math.min(1, (r - rootR * 0.88) / Math.max(0.002, outerR - rootR * 0.88))),
            1.35
          );

          // Local von Mises stress (MPa): high in contact tooth root fillet, low & cool in hub
          const localStress =
            pr.stressMPa *
            (0.05 + 0.22 * rimWeight + 0.78 * angWeight * (0.45 * rimWeight + 0.55 * filletWeight));
          stressArr[vi] = localStress;
          stressSqSumMPa2 += localStress * localStress;

          // Local tangential + slight radial-inward tooth denting vector
          const vx = verts[vi * 3];
          const vy = verts[vi * 3 + 1];
          const invR = r > 1e-6 ? 1 / r : 0;
          const radX = vx * invR;
          const radY = vy * invR;
          const tanX = -radY * tangSign;
          const tanY = radX * tangSign;

          const localBend = elasticDeflectM * angWeight * (0.25 * rimWeight + 0.75 * tipBendWeight);
          const elDx = (tanX * 0.86 - radX * 0.34) * localBend;
          const elDy = (tanY * 0.86 - radY * 0.34) * localBend;
          const elDz = 0;

          // Irreversible J2 plasticity when local σ_vM > σ_y
          if (localStress > pr.yieldMPa && frameIdx > 0) {
            const overstress = localStress - pr.yieldMPa;
            const dEp =
              (overstress / (hardeningMPa + youngsMPa * 0.04)) * dt * 2.2;
            const nextEp = Math.min(epsFrac * 1.25, pEp[vi] + dEp);
            const actualInc = nextEp - pEp[vi];
            pEp[vi] = nextEp;
            partPlasticIncSum += actualInc;

            // Permanent plastic denting/bending stored in vertex displacement history
            const plScale = actualInc * toothHeightM * 2.8 * angWeight * (0.2 + 0.8 * tipBendWeight);
            pDisp[vi * 3] += (tanX * 0.82 - radX * 0.42) * plScale;
            pDisp[vi * 3 + 1] += (tanY * 0.82 - radY * 0.42) * plScale;

            // Continuum damage D ∈ [0, 1] nucleating preferentially in the root fillet
            const dmgInc =
              (actualInc / epsFrac) *
              (0.7 + 0.9 * filletWeight) *
              (0.65 + 0.7 * config.brittleness);
            pDmg[vi] = Math.min(1.0, pDmg[vi] + dmgInc);
          }

          epArr[vi] = pEp[vi];
          dmgArr[vi] = pDmg[vi];

          const dx = elDx + pDisp[vi * 3];
          const dy = elDy + pDisp[vi * 3 + 1];
          const dz = elDz + pDisp[vi * 3 + 2];
          disp[vi * 3] = dx;
          disp[vi * 3 + 1] = dy;
          disp[vi * 3 + 2] = dz;

          const dMm = Math.hypot(dx, dy, dz) * 1000;
          if (dMm > partMaxDispMm) partMaxDispMm = dMm;
          const epPct = pEp[vi] * 100;
          if (epPct > partMaxPlasticPct) partMaxPlasticPct = epPct;
          if (pDmg[vi] > partMaxDamage) partMaxDamage = pDmg[vi];
        }

        // Volume-integrated elastic strain energy: E_e = ∫ (σ_vM^2 / (2 E)) dV
        const partVolM3 = Math.max(1e-7, pr.part.mesh.volume || 1.2e-5);
        const avgStressSqPa2 = (stressSqSumMPa2 / Math.max(1, vCount)) * 1e12;
        pr.elasticEnergyJ =
          (avgStressSqPa2 / Math.max(1e6, 2 * pr.material.youngsModulus)) *
          partVolM3 *
          0.35;

        // Cumulative plastic dissipation work: ΔE_p = σ_y * ∫ Δε_p dV
        if (partPlasticIncSum > 0) {
          const avgInc = partPlasticIncSum / Math.max(1, vCount);
          pr.plasticEnergyJ += pr.material.yieldStrength * avgInc * partVolM3 * 1.1;
        }

        totalElasticEnergyJ += pr.elasticEnergyJ;
        totalPlasticEnergyJ += pr.plasticEnergyJ;

        deformedDisplacements = disp;
        nodalStressMPa = stressArr;
        nodalPlasticStrain = epArr;
        nodalDamage = dmgArr;

        if (partMaxDispMm > frameMaxDispMm) frameMaxDispMm = partMaxDispMm;
        if (partMaxPlasticPct > frameMaxPlasticPct) {
          frameMaxPlasticPct = partMaxPlasticPct;
        }
        if (partMaxDamage > frameMaxDamage) frameMaxDamage = partMaxDamage;
      } else if (pr.destructible) {
        totalPlasticEnergyJ += pr.plasticEnergyJ;
      }

      partStates[pr.part.id] = {
        position: [...pr.pos] as Vec3,
        rotation: [...pr.rot] as Vec3,
        stressRatio: pr.stressMPa / Math.max(1, pr.yieldMPa),
        intact,
        partiallyFractured: pr.stage1Broken && !pr.stage2Broken,
        deformedDisplacements,
        nodalStressMPa,
        nodalPlasticStrain,
        nodalDamage,
        maxDisplacementMm: Number(partMaxDispMm.toFixed(3)),
        maxPlasticStrainPct: Number(partMaxPlasticPct.toFixed(2)),
        maxDamage: Number(partMaxDamage.toFixed(3)),
      };
    }

    if (frameMaxPlasticPct > maxPlasticStrainOverallPct) {
      maxPlasticStrainOverallPct = frameMaxPlasticPct;
    }
    if (frameMaxDamage > maxDamageOverall) {
      maxDamageOverall = frameMaxDamage;
    }
    maxEnergyOverallJ = Math.max(
      maxEnergyOverallJ,
      kineticEnergyJ,
      totalElasticEnergyJ,
      totalPlasticEnergyJ,
      totalFractureEnergyJ
    );

    const statusLabel =
      fracturedPartsCount > 0
        ? activeShardsCount === shardRuntimes.length
          ? 'Полное разрушение'
          : 'Скол зубьев (ступица цела)'
        : framePeakStress >= minYieldMPa
          ? 'Пластическая текучесть (σ_vM > σ_y)'
          : hasHadRealContact && frameHadGearContact
            ? 'Упругий контакт и изгиб зуба'
            : firstContactFrame >= 0
              ? 'Контакт зубьев'
              : 'Сведение и разгон приводов';

    const metrics: DestructionFrameMetrics = {
      peakStressMPa: Number(framePeakStress.toFixed(1)),
      yieldLimitMPa: Number(minYieldMPa.toFixed(0)),
      contactForceN: Number(frameContactForceN.toFixed(0)),
      kineticEnergyJ: Number(kineticEnergyJ.toFixed(4)),
      elasticEnergyJ: Number(totalElasticEnergyJ.toFixed(4)),
      plasticEnergyJ: Number(totalPlasticEnergyJ.toFixed(4)),
      fractureEnergyJ: Number(totalFractureEnergyJ.toFixed(4)),
      maxPlasticStrainPct: Number(frameMaxPlasticPct.toFixed(2)),
      maxDamage: Number(frameMaxDamage.toFixed(3)),
      maxDisplacementMm: Number(frameMaxDispMm.toFixed(3)),
      activeShardsCount,
      totalShardsCount: shardRuntimes.length,
      fracturedPartsCount,
      intactPartsCount,
      statusLabel,
    };

    const frameObj: DestructionCacheFrame = {
      frameIndex: frameIdx,
      timeSec,
      partStates,
      shardPos,
      shardQuat,
      shardActive,
      shardStress,
      shardStressMPa,
      shardPlasticStrain,
      shardDamage,
      sparks: packedSparks,
      metrics,
    };
    frames.push(frameObj);
    frameCacheBytes += computeSingleFrameBytes(frameObj);
    return frameObj;
  };

  const buildCacheSnapshot = (): DestructionCache => {
    const nowMs =
      typeof performance !== 'undefined' ? performance.now() : Date.now();
    return {
      id: cacheId,
      createdAt: new Date().toISOString(),
      solverBackend: 'web_local_preview',
      solverLabel: 'Web Local Preview',
      config: { ...config },
      fps: safeFps,
      totalFrames: Math.max(0, frames.length - 1),
      durationSec: safeDuration,
      floorY,
      contactPoint: [...latestContactWorld] as Vec3,
      bakeTimeMs: Math.max(1, Number((nowMs - startMs).toFixed(1))),
      cacheBytes: shardGeometryBytes + frameCacheBytes,
      shards: allShards,
      frames: [...frames],
      events: [...events],
      maxStressOverallMPa: Math.max(1, Number(maxStressOverallMPa.toFixed(1))),
      yieldStrengthRefMPa: Number(minYieldMPa.toFixed(0)),
      maxPlasticStrainOverallPct: Math.max(
        0.5,
        Number(maxPlasticStrainOverallPct.toFixed(2))
      ),
      maxDamageOverall: Number(Math.max(0.01, maxDamageOverall).toFixed(3)),
      maxEnergyOverallJ: Number(maxEnergyOverallJ.toFixed(4)),
    };
  };

  return {
    totalFrames,
    safeFps,
    safeDuration,
    substeps,
    stepFrame,
    buildCacheSnapshot,
  };
}

/**
 * Synchronous deterministic destruction cache baker with real 480 Hz 2D SAT
 * gear-tooth rigid-body collision detection, hard non-penetration projection,
 * ISO/Lewis tooth bending stress from actual contact impulses, and 2-stage fracture.
 */
export function bakeDestructionCacheSync(
  parts: CadPart[],
  joints: KinematicJoint[],
  materials: MaterialSpec[],
  config: DestructionSimConfig
): DestructionCache {
  const detectedPress = detectPressAssembly(
    parts,
    joints,
    materials,
    config.impactTorqueNm > 0 ? Math.max(500, config.impactTorqueNm * 11) : undefined
  );
  if (detectedPress) {
    return bakePressDestructionCache(parts, joints, materials, config, detectedPress);
  }

  const session = createGearDestructionSession(parts, joints, materials, config);
  for (let frameIdx = 0; frameIdx <= session.totalFrames; frameIdx++) {
    session.stepFrame(frameIdx);
  }
  return session.buildCacheSnapshot();
}

/**
 * Chunked asynchronous baker that streams live frames to the UI as they are solved,
 * supports instant `AbortSignal` cancellation, and computes wall-clock ETA telemetry.
 */
export async function bakeDestructionCacheAsync(
  parts: CadPart[],
  joints: KinematicJoint[],
  materials: MaterialSpec[],
  config: DestructionSimConfig,
  onProgress?: (pct: number, stageLabel: string) => void,
  options?: DestructionAsyncBakeOptions
): Promise<DestructionCache> {
  const signal = options?.signal;
  const throwIfAborted = () => {
    if (signal?.aborted) {
      throw new DOMException('Aborted', 'AbortError');
    }
  };
  throwIfAborted();

  const wallStartMs =
    typeof performance !== 'undefined' ? performance.now() : Date.now();
  const yieldStep = Math.max(1, options?.yieldEveryFrames ?? 5);
  const chunkDelayMs = options?.onPartialCache ? 12 : 0;

  onProgress?.(4, 'Генерация 3D осколков (Voronoi)...');
  await new Promise((r) => setTimeout(r, chunkDelayMs || 8));
  throwIfAborted();

  const detectedPress = detectPressAssembly(
    parts,
    joints,
    materials,
    config.impactTorqueNm > 0 ? Math.max(500, config.impactTorqueNm * 11) : undefined
  );

  if (detectedPress) {
    const fullPress = bakePressDestructionCache(
      parts,
      joints,
      materials,
      config,
      detectedPress
    );
    const totalCount = fullPress.frames.length;
    let shardBytes = 0;
    for (const s of fullPress.shards) {
      shardBytes += s.vertices.byteLength + s.normals.byteLength + s.indices.byteLength;
    }
    let accFrameBytes = 0;
    let runningStress = 1;
    let runningPlastic = 0.5;
    let runningDmg = 0.01;
    let runningEnergy = 1e-4;

    for (let i = 0; i < totalCount; i++) {
      throwIfAborted();
      const f = fullPress.frames[i];
      accFrameBytes += computeSingleFrameBytes(f);
      if (f.metrics.peakStressMPa > runningStress) runningStress = f.metrics.peakStressMPa;
      if (f.metrics.maxPlasticStrainPct > runningPlastic) runningPlastic = f.metrics.maxPlasticStrainPct;
      if (f.metrics.maxDamage > runningDmg) runningDmg = f.metrics.maxDamage;
      runningEnergy = Math.max(
        runningEnergy,
        f.metrics.kineticEnergyJ,
        f.metrics.elasticEnergyJ,
        f.metrics.plasticEnergyJ,
        f.metrics.fractureEnergyJ
      );

      if (i === 0 || (i + 1) % yieldStep === 0 || i === totalCount - 1) {
        const pct = Math.min(100, Math.max(5, Math.round(((i + 1) / totalCount) * 100)));
        const nowMs =
          typeof performance !== 'undefined' ? performance.now() : Date.now();
        const elapsedMs = Math.max(1, nowMs - wallStartMs);
        const stageLabel = `${f.metrics.statusLabel} · кадр ${i + 1}/${totalCount} (${pct}%)`;
        const meta: DestructionStreamMeta = {
          progressPct: pct,
          stepCount: (i + 1) * 4,
          totalSteps: totalCount * 4,
          framesCached: i + 1,
          latestFrameIndex: i,
          timeSec: f.timeSec,
          totalDurationSec: fullPress.durationSec,
          maxVonMisesMPa: runningStress,
          activeShards: f.metrics.activeShardsCount,
          elapsedMs,
          estimatedRemainingMs: estimateRemainingMs(elapsedMs, pct),
          stageLabel,
        };
        onProgress?.(pct, stageLabel);
        options?.onMeta?.(meta);
        if (options?.onPartialCache) {
          const partialCache: DestructionCache = {
            ...fullPress,
            totalFrames: i,
            bakeTimeMs: Number(elapsedMs.toFixed(1)),
            cacheBytes: shardBytes + accFrameBytes,
            frames: fullPress.frames.slice(0, i + 1),
            events: fullPress.events.filter((e) => e.frameIndex <= i),
            maxStressOverallMPa: Number(runningStress.toFixed(1)),
            maxPlasticStrainOverallPct: Number(runningPlastic.toFixed(2)),
            maxDamageOverall: Number(runningDmg.toFixed(3)),
            maxEnergyOverallJ: Number(runningEnergy.toFixed(4)),
          };
          options.onPartialCache(partialCache, i, meta);
        }
        if (i < totalCount - 1) {
          await new Promise((r) => setTimeout(r, chunkDelayMs));
        }
      }
    }
    onProgress?.(100, 'Кэш готов');
    return fullPress;
  }

  const session = createGearDestructionSession(parts, joints, materials, config);
  const totalCount = session.totalFrames + 1;

  for (let frameIdx = 0; frameIdx <= session.totalFrames; frameIdx++) {
    throwIfAborted();
    const frameObj = session.stepFrame(frameIdx);

    if (
      frameIdx === 0 ||
      (frameIdx + 1) % yieldStep === 0 ||
      frameIdx === session.totalFrames
    ) {
      const pct = Math.min(
        100,
        Math.max(5, Math.round(((frameIdx + 1) / totalCount) * 100))
      );
      const nowMs =
        typeof performance !== 'undefined' ? performance.now() : Date.now();
      const elapsedMs = Math.max(1, nowMs - wallStartMs);
      const snapshot = session.buildCacheSnapshot();
      const stageLabel = `${frameObj.metrics.statusLabel} · кадр ${frameIdx + 1}/${totalCount} (${pct}%)`;
      const meta: DestructionStreamMeta = {
        progressPct: pct,
        stepCount: (frameIdx + 1) * session.substeps,
        totalSteps: totalCount * session.substeps,
        framesCached: frameIdx + 1,
        latestFrameIndex: frameIdx,
        timeSec: frameObj.timeSec,
        totalDurationSec: session.safeDuration,
        maxVonMisesMPa: snapshot.maxStressOverallMPa,
        activeShards: frameObj.metrics.activeShardsCount,
        elapsedMs,
        estimatedRemainingMs: estimateRemainingMs(elapsedMs, pct),
        stageLabel,
      };
      onProgress?.(pct, stageLabel);
      options?.onMeta?.(meta);
      options?.onPartialCache?.(snapshot, frameIdx, meta);

      if (frameIdx < session.totalFrames) {
        await new Promise((r) => setTimeout(r, chunkDelayMs));
      }
    }
  }

  onProgress?.(100, 'Кэш готов');
  return session.buildCacheSnapshot();
}

export function formatCacheBytes(bytes: number): string {
  if (bytes < 1024) return `${bytes} Б`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} КБ`;
  return `${(bytes / (1024 * 1024)).toFixed(2)} МБ`;
}

export function exportDestructionCacheSummaryJson(
  cache: DestructionCache
): string {
  return JSON.stringify(
    {
      id: cache.id,
      createdAt: cache.createdAt,
      config: cache.config,
      fps: cache.fps,
      totalFrames: cache.totalFrames,
      durationSec: cache.durationSec,
      bakeTimeMs: cache.bakeTimeMs,
      cacheBytes: cache.cacheBytes,
      shardsCount: cache.shards.length,
      maxStressOverallMPa: cache.maxStressOverallMPa,
      events: cache.events,
      frameTelemetry: cache.frames.map((f) => ({
        frameIndex: f.frameIndex,
        timeSec: f.timeSec,
        metrics: f.metrics,
      })),
    },
    null,
    2
  );
}
