import type {
  CachedPartFrameState,
  CadPart,
  DestructionCache,
  DestructionCacheFrame,
  DestructionFrameMetrics,
  DestructionSimConfig,
  DestructionTimelineEvent,
  FractureShardGeometry,
  KinematicJoint,
  MaterialSpec,
} from '../types/cad';
import type { LivePhysicsSimulation } from './live_sync_adapter';
import { getMaterialById } from './materials';

type Vec3 = [number, number, number];
type Quat = [number, number, number, number];

export interface DetectedPressAssembly {
  joint: KinematicJoint | null;
  basePart: CadPart | null;
  pressPart: CadPart;
  specimenParts: CadPart[];
  /** Unit axis pointing in the direction the press plate pushes toward the specimen/base */
  pushDir: Vec3;
  /** World axial coordinate along pushDir of the base support face (far end of specimen) */
  baseFaceS: number;
  /** World axial coordinate along pushDir of the specimen's near face (touched by press) */
  specimenNearS: number;
  /** World axial coordinate along pushDir of the specimen's far face (resting on base) */
  specimenFarS: number;
  /** World axial coordinate along pushDir of the press plate's leading face */
  pressFaceS: number;
  /** Initial air gap along pushDir before press contacts specimen (>= 0) */
  initialGapM: number;
  /** Free axial length of primary specimen (meters) */
  freeLengthM: number;
  /** Solid bottomed height of primary specimen (meters) */
  solidHeightM: number;
  /** Maximum compressive stroke available (meters) */
  maxStrokeM: number;
  /** Maximum press force in Newtons */
  maxForceN: number;
  /** True if primary specimen is a helical coil spring */
  isHelicalSpring: boolean;
  /** Estimated linear axial stiffness k in N/m */
  stiffnessNPerM: number;
  /** Wahl curvature factor K_W (1.0 for non-springs) */
  wahlFactor: number;
  /** Wire diameter in meters (for springs) */
  wireDiameterM: number;
  /** Mean coil diameter in meters (for springs) */
  meanCoilDiameterM: number;
}

function dot3(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function sub3(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function add3(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scale3(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

function len3(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

function normalize3(a: Vec3): Vec3 {
  const l = len3(a);
  if (l < 1e-12) return [0, 0, -1];
  return [a[0] / l, a[1] / l, a[2] / l];
}

function mulberry32(seed: number): () => number {
  let t = seed >>> 0;
  return () => {
    t += 0x6d2b79f5;
    let r = Math.imul(t ^ (t >>> 15), 1 | t);
    r ^= r + Math.imul(r ^ (r >>> 7), 61 | r);
    return ((r ^ (r >>> 14)) >>> 0) / 4294967296;
  };
}

function quatNormalize(q: Quat): Quat {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n < 1e-12) return [0, 0, 0, 1];
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

function quatConjugate(q: Quat): Quat {
  return [-q[0], -q[1], -q[2], q[3]];
}

function quatMultiply(a: Quat, b: Quat): Quat {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

function rotateVecByQuat(v: Vec3, q: Quat): Vec3 {
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
  const [qx, qy, qz, qw] = q;
  const [wx, wy, wz] = omega;
  const half = 0.5 * dt;
  const dq: Quat = [
    half * (wx * qw + wy * qz - wz * qy),
    half * (wy * qw + wz * qx - wx * qz),
    half * (wz * qw + wx * qy - wy * qx),
    half * (-wx * qx - wy * qy - wz * qz),
  ];
  return quatNormalize([
    qx + dq[0],
    qy + dq[1],
    qz + dq[2],
    qw + dq[3],
  ]);
}

function eulerFromQuatXYZ(q: Quat): Vec3 {
  const [x, y, z, w] = quatNormalize(q);
  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const rx = Math.atan2(sinrCosp, cosrCosp);

  const sinp = 2 * (w * y - z * x);
  const ry = Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp);

  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  const rz = Math.atan2(sinyCosp, cosyCosp);

  return [rx, ry, rz];
}

function cross3(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function wrapAnglePi(a: number): number {
  let r = a % (2 * Math.PI);
  if (r > Math.PI) r -= 2 * Math.PI;
  if (r < -Math.PI) r += 2 * Math.PI;
  return r;
}

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

function quatFromEulerXYZ(rot: Vec3): Quat {
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

/**
 * Computes world-space axis-aligned bounding box of a part.
 */
function getWorldAabb(part: CadPart): { min: Vec3; max: Vec3; center: Vec3; size: Vec3 } {
  const bb = part.mesh.boundingBox;
  const min: Vec3 = [
    part.position[0] + bb.min[0] * (part.scale[0] || 1),
    part.position[1] + bb.min[1] * (part.scale[1] || 1),
    part.position[2] + bb.min[2] * (part.scale[2] || 1),
  ];
  const max: Vec3 = [
    part.position[0] + bb.max[0] * (part.scale[0] || 1),
    part.position[1] + bb.max[1] * (part.scale[1] || 1),
    part.position[2] + bb.max[2] * (part.scale[2] || 1),
  ];
  return {
    min,
    max,
    center: [
      0.5 * (min[0] + max[0]),
      0.5 * (min[1] + max[1]),
      0.5 * (min[2] + max[2]),
    ],
    size: [
      Math.max(1e-6, max[0] - min[0]),
      Math.max(1e-6, max[1] - min[1]),
      Math.max(1e-6, max[2] - min[2]),
    ],
  };
}

/**
 * Projects an AABB onto an arbitrary unit axis vector, returning [minS, maxS].
 */
function projectAabbOnAxis(aabb: { min: Vec3; max: Vec3 }, axis: Vec3): [number, number] {
  let minS = Infinity;
  let maxS = -Infinity;
  for (let ix = 0; ix < 2; ix++) {
    const x = ix === 0 ? aabb.min[0] : aabb.max[0];
    for (let iy = 0; iy < 2; iy++) {
      const y = iy === 0 ? aabb.min[1] : aabb.max[1];
      for (let iz = 0; iz < 2; iz++) {
        const z = iz === 0 ? aabb.min[2] : aabb.max[2];
        const s = x * axis[0] + y * axis[1] + z * axis[2];
        if (s < minS) minS = s;
        if (s > maxS) maxS = s;
      }
    }
  }
  return [minS, maxS];
}

function isHelixPart(part: CadPart): boolean {
  if (part.dag.some((n) => n.type === 'helix')) return true;
  const hay = `${part.id} ${part.name}`.toLowerCase();
  return /spring|coil|helix|пружин/.test(hay);
}

function isPlateOrBaseName(part: CadPart): boolean {
  const hay = `${part.id} ${part.name}`.toLowerCase();
  return /base|plate|anvil|ram|press|plunger|piston|плит|пресс|опор|основан/.test(hay);
}

function extractHelixParams(part: CadPart, aabb: { size: Vec3 }, freeLengthM: number): {
  meanCoilDiameterM: number;
  wireDiameterM: number;
  activeCoils: number;
} {
  const helixNode = part.dag.find((n) => n.type === 'helix');
  const p = (helixNode?.params ?? {}) as Record<string, unknown>;
  const rawHelixR =
    typeof p.helix_radius === 'number'
      ? p.helix_radius
      : typeof p.helixRadius === 'number'
        ? p.helixRadius
        : 0;
  const rawWireR =
    typeof p.wire_radius === 'number'
      ? p.wire_radius
      : typeof p.wireRadius === 'number'
        ? p.wireRadius
        : 0;
  const rawPitch = typeof p.pitch === 'number' ? p.pitch : 0;
  const rawHeight = typeof p.height === 'number' ? p.height : 0;

  // Rust DAG params are serialized in meters (or occasionally mm if > 1.0)
  const toMeters = (v: number) => (v > 0.5 ? v * 1e-3 : v);

  const transverseSpan = Math.max(
    1e-3,
    Math.min(aabb.size[0], aabb.size[1]) > freeLengthM * 0.25
      ? 0.5 * (aabb.size[0] + aabb.size[1])
      : Math.max(aabb.size[0], aabb.size[1], aabb.size[2]) * 0.55
  );

  const wireDiameterM =
    rawWireR > 0 ? 2 * toMeters(rawWireR) : Math.max(0.002, Math.min(0.008, transverseSpan * 0.115));
  const meanCoilDiameterM =
    rawHelixR > 0
      ? 2 * toMeters(rawHelixR)
      : Math.max(wireDiameterM * 4, transverseSpan - wireDiameterM);
  const h = rawHeight > 0 ? toMeters(rawHeight) : freeLengthM;
  const pitch = rawPitch > 0 ? toMeters(rawPitch) : Math.max(wireDiameterM * 2.2, h / 5.5);
  const activeCoils = Math.max(2, Math.min(18, h / Math.max(1e-4, pitch)));

  return { meanCoilDiameterM, wireDiameterM, activeCoils };
}

/**
 * Detects a prismatic press / axial compression assembly from the current CAD parts and joints.
 * Handles assemblies built by MCP agents (e.g., `base_plate` + `spring_1` + `press_plate` + `press_slider`)
 * along any axis (X, Y, or Z).
 */
export function detectPressAssembly(
  parts: readonly CadPart[],
  joints: readonly KinematicJoint[],
  materials: readonly MaterialSpec[],
  fallbackForceN?: number
): DetectedPressAssembly | null {
  const visible = parts.filter((p) => p.visible);
  if (visible.length < 2) return null;

  // Don't hijack 2-gear clash assemblies
  const gearCount = visible.filter((p) => p.dag.some((n) => n.type === 'spur_gear')).length;
  const prismaticJoint = joints.find((j) => j.type === 'prismatic') ?? null;
  if (gearCount >= 2 && !prismaticJoint) return null;

  let basePart: CadPart | null = null;
  let pressPart: CadPart | null = null;
  let axisHint: Vec3 = [0, 0, 1];

  if (prismaticJoint) {
    const partA = visible.find((p) => p.id === prismaticJoint.partA) ?? null;
    const partB = visible.find((p) => p.id === prismaticJoint.partB) ?? null;
    axisHint = normalize3(prismaticJoint.axis);

    if (partA && partB) {
      if (partA.isFixed && !partB.isFixed) {
        basePart = partA;
        pressPart = partB;
      } else if (partB.isFixed && !partA.isFixed) {
        basePart = partB;
        pressPart = partA;
      } else {
        basePart = partA;
        pressPart = partB;
      }
    } else if (partB && !partB.isFixed) {
      pressPart = partB;
      basePart = visible.find((p) => p.isFixed && p.id !== pressPart?.id) ?? null;
    } else if (partA && !partA.isFixed) {
      pressPart = partA;
      basePart = visible.find((p) => p.isFixed && p.id !== pressPart?.id) ?? null;
    }
  } else {
    // Fallback heuristic: fixed base + helical spring + movable press plate
    const helix = visible.find((p) => isHelixPart(p));
    const fixedParts = visible.filter((p) => p.isFixed);
    const movablePlates = visible.filter(
      (p) => !p.isFixed && p !== helix && isPlateOrBaseName(p)
    );
    if (helix && fixedParts.length >= 1 && movablePlates.length >= 1) {
      basePart = fixedParts[0];
      pressPart = movablePlates[0];
      const delta = sub3(getWorldAabb(basePart).center, getWorldAabb(pressPart).center);
      axisHint = normalize3(delta);
    }
  }

  if (!pressPart) return null;

  // Specimen parts are all other movable parts (preferring springs / parts between press and base)
  const candidates = visible.filter(
    (p) => p.id !== pressPart!.id && p.id !== basePart?.id && !p.isFixed
  );
  if (candidates.length === 0) return null;

  const specimenParts = [...candidates].sort((a, b) => {
    const ah = isHelixPart(a) ? 0 : 1;
    const bh = isHelixPart(b) ? 0 : 1;
    return ah - bh;
  });
  const primarySpecimen = specimenParts[0];

  const pressAabb = getWorldAabb(pressPart);
  const specAabb = getWorldAabb(primarySpecimen);
  const baseAabb = basePart ? getWorldAabb(basePart) : null;

  // Orient pushDir so it points FROM pressPart TOWARD primarySpecimen / basePart
  const pressToSpec = sub3(specAabb.center, pressAabb.center);
  let pushDir = normalize3(axisHint);
  const dotSpec = dot3(pressToSpec, pushDir);
  if (Math.abs(dotSpec) > 1e-5) {
    if (dotSpec < 0) pushDir = scale3(pushDir, -1);
  } else if (baseAabb) {
    const pressToBase = sub3(baseAabb.center, pressAabb.center);
    if (dot3(pressToBase, pushDir) < 0) pushDir = scale3(pushDir, -1);
  } else if (prismaticJoint?.limits) {
    const [limA, limB] = prismaticJoint.limits;
    const minL = Math.min(limA, limB);
    const maxL = Math.max(limA, limB);
    if (Math.abs(minL) > Math.abs(maxL)) {
      pushDir = scale3(pushDir, -1);
    }
  }

  const [, pressMaxS] = projectAabbOnAxis(pressAabb, pushDir);
  const [specMinS, specMaxS] = projectAabbOnAxis(specAabb, pushDir);
  const baseFaceS = baseAabb ? projectAabbOnAxis(baseAabb, pushDir)[0] : specMaxS;

  const freeLengthM = Math.max(0.004, specMaxS - specMinS);
  const initialGapM = Math.max(0, specMinS - pressMaxS);
  const isHelicalSpring = isHelixPart(primarySpecimen);

  const specMat = getMaterialById(primarySpecimen.materialId, materials as MaterialSpec[]);
  const youngsPa = Math.max(1e9, specMat.youngsModulus || 68.9e9);
  const nu = Math.max(0.15, Math.min(0.45, specMat.poissonsRatio || 0.33));
  const shearModulusPa = youngsPa / (2 * (1 + nu));

  let wireDiameterM = 0.004;
  let meanCoilDiameterM = 0.032;
  let wahlFactor = 1.0;
  let stiffnessNPerM = 5000;
  let solidHeightM = freeLengthM * 0.4;

  if (isHelicalSpring) {
    const hp = extractHelixParams(primarySpecimen, specAabb, freeLengthM);
    wireDiameterM = hp.wireDiameterM;
    meanCoilDiameterM = hp.meanCoilDiameterM;
    const springIndexC = Math.max(3.2, meanCoilDiameterM / Math.max(1e-4, wireDiameterM));
    wahlFactor =
      (4 * springIndexC - 1) / (4 * springIndexC - 4) + 0.615 / springIndexC;
    stiffnessNPerM =
      (shearModulusPa * Math.pow(wireDiameterM, 4)) /
      Math.max(1e-12, 8 * Math.pow(meanCoilDiameterM, 3) * hp.activeCoils);
    solidHeightM = Math.max(
      freeLengthM * 0.22,
      Math.min(freeLengthM * 0.62, hp.activeCoils * wireDiameterM)
    );
  } else {
    const areaM2 = Math.max(1e-5, (primarySpecimen.mesh.volume || 1e-5) / freeLengthM);
    stiffnessNPerM = (youngsPa * areaM2) / freeLengthM;
    solidHeightM = freeLengthM * 0.45;
  }

  // Respect prismatic joint travel limit if specified, otherwise compress to solid height
  let maxStrokeM = Math.max(0.002, freeLengthM - solidHeightM);
  if (prismaticJoint?.limits) {
    const [l0, l1] = prismaticJoint.limits;
    const jointSpan = Math.abs(l1 - l0);
    if (jointSpan > 1e-4) {
      // Joint limits are always stored in SI meters (e.g. [0, -0.036] -> 0.036 m, or [-1.5, 1.5] -> 3.0 m)
      const limitM = jointSpan;
      // Never let press plate penetrate past the base plate!
      const clearanceToBase = Math.max(0.002, baseFaceS - pressMaxS - solidHeightM * 0.32);
      maxStrokeM = Math.min(limitM, clearanceToBase);
    }
  }

  const jointForceN =
    prismaticJoint && prismaticJoint.motorTorqueNm > 0
      ? prismaticJoint.motorTorqueNm
      : 0;
  const maxForceN = Math.max(
    50,
    jointForceN > 0 ? jointForceN : fallbackForceN ?? 2000
  );

  return {
    joint: prismaticJoint,
    basePart,
    pressPart,
    specimenParts,
    pushDir,
    baseFaceS,
    specimenNearS: specMinS,
    specimenFarS: specMaxS,
    pressFaceS: pressMaxS,
    initialGapM,
    freeLengthM,
    solidHeightM,
    maxStrokeM,
    maxForceN,
    isHelicalSpring,
    stiffnessNPerM,
    wahlFactor,
    wireDiameterM,
    meanCoilDiameterM,
  };
}

/**
 * Builds deterministic fractured coil / specimen fragments from the specimen's triangle mesh
 * so rupture shards match the exact geometry & material of the pressed part.
 */
function buildSpecimenShards(
  part: CadPart,
  mat: MaterialSpec,
  targetCount: number,
  seed: number,
  helixU?: Float32Array | null
): { shards: FractureShardGeometry[]; localComs: Vec3[] } {
  const rng = mulberry32(seed + 4099);
  const verts = part.mesh.vertices;
  const indices = part.mesh.indices;
  const triCount = Math.floor(indices.length / 3);
  if (triCount < 6) return { shards: [], localComs: [] };

  const count = Math.max(6, Math.min(24, targetCount));
  const buckets: number[][] = Array.from({ length: count }, () => []);

  if (helixU && helixU.length >= part.mesh.vertexCount) {
    // Partition helical spring strictly along its wire arc parameter u in [0, 1]
    // so broken fragments become clean curved coil-arc pieces instead of fusing touching coils
    const weights: number[] = [];
    let weightSum = 0;
    for (let i = 0; i < count; i++) {
      const w = 0.65 + rng() * 0.7;
      weights.push(w);
      weightSum += w;
    }
    const cuts: number[] = [];
    let acc = 0;
    for (let i = 0; i < count - 1; i++) {
      acc += weights[i] / weightSum;
      cuts.push(acc);
    }

    for (let t = 0; t < triCount; t++) {
      const i0 = indices[t * 3];
      const i1 = indices[t * 3 + 1];
      const i2 = indices[t * 3 + 2];
      const uTri = (helixU[i0] + helixU[i1] + helixU[i2]) / 3;
      let b = 0;
      while (b < cuts.length && uTri > cuts[b]) b++;
      buckets[b].push(t);
    }
  } else {
    const bb = part.mesh.boundingBox;
    const cx = 0.5 * (bb.min[0] + bb.max[0]);
    const cy = 0.5 * (bb.min[1] + bb.max[1]);
    const cz = 0.5 * (bb.min[2] + bb.max[2]);
    const sx = Math.max(1e-4, bb.max[0] - bb.min[0]);
    const sy = Math.max(1e-4, bb.max[1] - bb.min[1]);
    const sz = Math.max(1e-4, bb.max[2] - bb.min[2]);

    // Seed shard centroids inside the part's bounding volume
    const seeds: Vec3[] = [];
    for (let i = 0; i < count; i++) {
      seeds.push([
        cx + (rng() - 0.5) * sx * 0.85,
        cy + (rng() - 0.5) * sy * 0.85,
        cz + (rng() - 0.5) * sz * 0.85,
      ]);
    }

    // Assign each triangle to its nearest Voronoi seed
    for (let t = 0; t < triCount; t++) {
      const i0 = indices[t * 3];
      const i1 = indices[t * 3 + 1];
      const i2 = indices[t * 3 + 2];
      const mx = (verts[i0 * 3] + verts[i1 * 3] + verts[i2 * 3]) / 3;
      const my = (verts[i0 * 3 + 1] + verts[i1 * 3 + 1] + verts[i2 * 3 + 1]) / 3;
      const mz = (verts[i0 * 3 + 2] + verts[i1 * 3 + 2] + verts[i2 * 3 + 2]) / 3;

      let best = 0;
      let bestD = Infinity;
      for (let s = 0; s < count; s++) {
        const dx = mx - seeds[s][0];
        const dy = my - seeds[s][1];
        const dz = mz - seeds[s][2];
        const d = dx * dx + dy * dy + dz * dz;
        if (d < bestD) {
          bestD = d;
          best = s;
        }
      }
      buckets[best].push(t);
    }
  }

  const shards: FractureShardGeometry[] = [];
  const localComs: Vec3[] = [];
  const totalMass = Math.max(0.005, part.mesh.mass || 0.02);

  for (let b = 0; b < buckets.length; b++) {
    const tris = buckets[b];
    if (tris.length < 2) continue;

    // Compute centroid
    let sxSum = 0;
    let sySum = 0;
    let szSum = 0;
    let vCnt = 0;
    for (const t of tris) {
      for (let k = 0; k < 3; k++) {
        const vi = indices[t * 3 + k];
        sxSum += verts[vi * 3];
        sySum += verts[vi * 3 + 1];
        szSum += verts[vi * 3 + 2];
        vCnt++;
      }
    }
    const com: Vec3 = [sxSum / vCnt, sySum / vCnt, szSum / vCnt];

    const outV = new Float32Array(tris.length * 9);
    const outN = new Float32Array(tris.length * 9);
    const outI = new Uint32Array(tris.length * 3);
    let maxR = 0.0015;

    for (let ti = 0; ti < tris.length; ti++) {
      const t = tris[ti];
      const i0 = indices[t * 3];
      const i1 = indices[t * 3 + 1];
      const i2 = indices[t * 3 + 2];
      const p0: Vec3 = [verts[i0 * 3] - com[0], verts[i0 * 3 + 1] - com[1], verts[i0 * 3 + 2] - com[2]];
      const p1: Vec3 = [verts[i1 * 3] - com[0], verts[i1 * 3 + 1] - com[1], verts[i1 * 3 + 2] - com[2]];
      const p2: Vec3 = [verts[i2 * 3] - com[0], verts[i2 * 3 + 1] - com[1], verts[i2 * 3 + 2] - com[2]];

      const e1 = sub3(p1, p0);
      const e2 = sub3(p2, p0);
      const fn = normalize3([
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
      ]);

      const pts = [p0, p1, p2];
      for (let k = 0; k < 3; k++) {
        const vo = (ti * 3 + k) * 3;
        outV[vo] = pts[k][0];
        outV[vo + 1] = pts[k][1];
        outV[vo + 2] = pts[k][2];
        outN[vo] = fn[0];
        outN[vo + 1] = fn[1];
        outN[vo + 2] = fn[2];
        outI[ti * 3 + k] = ti * 3 + k;
        const r = len3(pts[k]);
        if (r > maxR) maxR = r;
      }
    }

    shards.push({
      id: `${part.id}_shard_${shards.length}`,
      parentPartId: part.id,
      materialId: part.materialId,
      colorHex: mat.colorHex,
      roughness: mat.roughness,
      metalness: mat.metalness,
      localCom: com,
      boundingRadius: maxR,
      mass: Math.max(0.0005, (totalMass * tris.length) / triCount),
      distToContact: 0,
      vertices: outV,
      normals: outN,
      indices: outI,
      innerVertices: new Float32Array(0),
      innerNormals: new Float32Array(0),
      innerIndices: new Uint32Array(0),
    });
    localComs.push(com);
  }

  return { shards, localComs };
}

interface HelixKinematicsData {
  /** Local orthonormal basis in part-local coordinates: eU x eV = eAx (eAx = pushDir) */
  eU: Vec3;
  eV: Vec3;
  eAx: Vec3;
  /** Center of helix axis in (eU, eV) coordinates */
  cU: number;
  cV: number;
  /** Axial bounds in part-local space along eAx */
  sNear: number;
  sFar: number;
  /** Centerline axial coordinates at u=0 (base end) and u=1 (press end) */
  sBaseCenter: number;
  sPressCenter: number;
  freeCenterlineH: number;
  meanCoilRadius: number;
  wireRadius: number;
  pitch0: number;
  turns: number;
  totalSweep: number;
  phiBase: number;
  handedness: 1 | -1;
  /** Per-vertex centerline parameter u in [0, 1] (0 = base end, 1 = press end) */
  vertU: Float32Array;
  /** Per-vertex local cross-section components in reference Frenet-radial frame */
  vertRad: Float32Array;
  vertBin: Float32Array;
  vertTan: Float32Array;
}

/**
 * Extracts exact Kirchhoff helical rod coordinates `(u, u_rad, u_bin, u_tan)` for every vertex
 * of a helical coil spring mesh so compression deforms the 3D centerline without squashing the
 * circular wire cross-section.
 */
function buildHelixKinematicsData(
  part: CadPart,
  press: DetectedPressAssembly
): HelixKinematicsData | null {
  const verts = part.mesh.vertices;
  const vCount = part.mesh.vertexCount;
  if (vCount < 16) return null;

  const invPartQ = quatConjugate(quatFromEulerXYZ(part.rotation));
  const eAx = normalize3(rotateVecByQuat(press.pushDir, invPartQ));
  const refVec: Vec3 = Math.abs(eAx[0]) < 0.8 ? [1, 0, 0] : [0, 1, 0];
  const eU = normalize3(cross3(refVec, eAx));
  const eV = normalize3(cross3(eAx, eU));

  const sArr = new Float32Array(vCount);
  const uRaw = new Float32Array(vCount);
  const vRaw = new Float32Array(vCount);

  let uMin = Infinity, uMax = -Infinity;
  let vMin = Infinity, vMax = -Infinity;
  let sMin = Infinity, sMax = -Infinity;

  for (let i = 0; i < vCount; i++) {
    const o = i * 3;
    const x = verts[o];
    const y = verts[o + 1];
    const z = verts[o + 2];
    const pu = x * eU[0] + y * eU[1] + z * eU[2];
    const pv = x * eV[0] + y * eV[1] + z * eV[2];
    const ps = x * eAx[0] + y * eAx[1] + z * eAx[2];
    uRaw[i] = pu;
    vRaw[i] = pv;
    sArr[i] = ps;
    if (pu < uMin) uMin = pu;
    if (pu > uMax) uMax = pu;
    if (pv < vMin) vMin = pv;
    if (pv > vMax) vMax = pv;
    if (ps < sMin) sMin = ps;
    if (ps > sMax) sMax = ps;
  }

  const L0 = sMax - sMin;
  if (!(L0 > 1e-4)) return null;

  const cU = 0.5 * (uMin + uMax);
  const cV = 0.5 * (vMin + vMax);

  const rArr = new Float32Array(vCount);
  const phiArr = new Float32Array(vCount);
  let rMin = Infinity, rMax = -Infinity;

  for (let i = 0; i < vCount; i++) {
    const du = uRaw[i] - cU;
    const dv = vRaw[i] - cV;
    const r = Math.hypot(du, dv);
    rArr[i] = r;
    phiArr[i] = Math.atan2(dv, du);
    if (r < rMin) rMin = r;
    if (r > rMax) rMax = r;
  }

  const R = 0.5 * (rMin + rMax);
  const rW = 0.5 * (rMax - rMin);
  if (!(R > 1e-4) || !(rW > 5e-5) || rW >= R * 0.85) return null;

  // Estimate pitch P by clustering outer-ridge vertices in the densest azimuthal sector
  const numSectors = 8;
  const sectorWidth = 0.28;
  let bestSectorIdx = 0;
  let bestSectorCount = 0;
  const sectorBuckets: number[][] = Array.from({ length: numSectors }, () => []);
  const rThresh = R + 0.45 * rW;

  for (let i = 0; i < vCount; i++) {
    if (rArr[i] < rThresh) continue;
    const p = phiArr[i];
    for (let sIdx = 0; sIdx < numSectors; sIdx++) {
      const targetPhi = -Math.PI + ((sIdx + 0.5) / numSectors) * (2 * Math.PI);
      if (Math.abs(wrapAnglePi(p - targetPhi)) <= sectorWidth) {
        sectorBuckets[sIdx].push(sArr[i]);
        if (sectorBuckets[sIdx].length > bestSectorCount) {
          bestSectorCount = sectorBuckets[sIdx].length;
          bestSectorIdx = sIdx;
        }
      }
    }
  }

  let pitchEst = press.freeLengthM / 5.5;
  const helixNode = part.dag.find((n) => n.type === 'helix');
  const hpRaw = (helixNode?.params ?? {}) as Record<string, unknown>;
  if (typeof hpRaw.pitch === 'number' && hpRaw.pitch > 0) {
    const dagP = hpRaw.pitch > 0.5 ? hpRaw.pitch * 1e-3 : hpRaw.pitch;
    if (dagP > 2 * rW && dagP < L0 * 0.95) {
      pitchEst = dagP;
    }
  }

  const secVals = sectorBuckets[bestSectorIdx];
  if (secVals.length >= 2) {
    secVals.sort((a, b) => a - b);
    const clusterMeans: number[] = [];
    let sum = secVals[0];
    let cnt = 1;
    const gapTol = Math.max(1.5 * rW, pitchEst * 0.35);
    for (let i = 1; i < secVals.length; i++) {
      if (secVals[i] - secVals[i - 1] <= gapTol) {
        sum += secVals[i];
        cnt++;
      } else {
        clusterMeans.push(sum / cnt);
        sum = secVals[i];
        cnt = 1;
      }
    }
    clusterMeans.push(sum / cnt);
    if (clusterMeans.length >= 2) {
      const diffs: number[] = [];
      for (let k = 1; k < clusterMeans.length; k++) {
        const d = clusterMeans[k] - clusterMeans[k - 1];
        if (d > 1.8 * rW) diffs.push(d);
      }
      if (diffs.length > 0) {
        diffs.sort((a, b) => a - b);
        pitchEst = diffs[Math.floor(diffs.length / 2)];
      }
    }
  }

  if (!(pitchEst > 1.5 * rW)) return null;

  // Determine handedness (+1 or -1) and reference phase via circular coherence on outer vertices
  const sMid = 0.5 * (sMin + sMax);
  const sampleStride = Math.max(1, Math.floor(vCount / 600));
  const sampleIdx: number[] = [];
  for (let i = 0; i < vCount; i += sampleStride) {
    if (rArr[i] > R + 0.3 * rW) sampleIdx.push(i);
  }
  if (sampleIdx.length < 8) {
    for (let i = 0; i < vCount; i += sampleStride) sampleIdx.push(i);
  }

  let bestH: 1 | -1 = 1;
  let bestScore = -1;
  let phiMid = 0;
  for (const candH of [1, -1] as const) {
    const kSlope = (candH * 2 * Math.PI) / pitchEst;
    let sumC = 0;
    let sumS = 0;
    for (const idx of sampleIdx) {
      const err = phiArr[idx] - kSlope * (sArr[idx] - sMid);
      sumC += Math.cos(err);
      sumS += Math.sin(err);
    }
    const score = Math.hypot(sumC, sumS) / Math.max(1, sampleIdx.length);
    if (score > bestScore) {
      bestScore = score;
      bestH = candH;
      phiMid = Math.atan2(sumS, sumC);
    }
  }

  // Refine pitch & phiMid via 2 linear-regression steps on wrapped phase residuals
  let kSlope = (bestH * 2 * Math.PI) / pitchEst;
  for (let iter = 0; iter < 2; iter++) {
    let sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
    const n = sampleIdx.length;
    for (const idx of sampleIdx) {
      const ds = sArr[idx] - sMid;
      const res = wrapAnglePi(phiArr[idx] - (phiMid + kSlope * ds));
      sumX += ds;
      sumY += res;
      sumXX += ds * ds;
      sumXY += ds * res;
    }
    const denom = n * sumXX - sumX * sumX;
    if (Math.abs(denom) > 1e-12) {
      const dSlope = (n * sumXY - sumX * sumY) / denom;
      const dPhi0 = (sumY - dSlope * sumX) / n;
      kSlope += dSlope;
      phiMid = wrapAnglePi(phiMid + dPhi0);
    }
  }

  const pitch0 = (2 * Math.PI) / Math.max(1e-6, Math.abs(kSlope));
  const cPitch = pitch0 / (2 * Math.PI);

  // Closest point parameter t_i on centerline:
  // s_c(t) = sMid - cPitch * t, phi_c(t) = phiMid - bestH * t
  const rawT = new Float32Array(vCount);
  let tMin = Infinity;
  let tMax = -Infinity;

  for (let i = 0; i < vCount; i++) {
    const px = uRaw[i] - cU;
    const py = vRaw[i] - cV;
    const ps = sArr[i];

    const tApprox = (sMid - ps) / cPitch;
    const phiApprox = phiMid - bestH * tApprox;
    const dPhi = wrapAnglePi(phiArr[i] - phiApprox);
    let t = tApprox - bestH * dPhi;

    for (let it = 0; it < 3; it++) {
      const phi = phiMid - bestH * t;
      const cosP = Math.cos(phi);
      const sinP = Math.sin(phi);
      const cx = R * cosP;
      const cy = R * sinP;
      const cs = sMid - cPitch * t;

      const dpx = cx - px;
      const dpy = cy - py;
      const dps = cs - ps;

      const tx = bestH * R * sinP;
      const ty = -bestH * R * cosP;
      const ts = -cPitch;

      const g = tx * dpx + ty * dpy + ts * dps;
      const hVal =
        tx * tx +
        ty * ty +
        ts * ts +
        (-cx * dpx - cy * dpy);
      if (Math.abs(hVal) > 1e-12) {
        t -= clamp(g / hVal, -0.35, 0.35);
      }
    }

    rawT[i] = t;
    if (t < tMin) tMin = t;
    if (t > tMax) tMax = t;
  }

  const totalSweep = tMax - tMin;
  if (!(totalSweep > Math.PI)) return null;

  const turns = totalSweep / (2 * Math.PI);
  const sBaseCenter = sMid - cPitch * tMin;
  const sPressCenter = sMid - cPitch * tMax;
  const freeCenterlineH = Math.max(1e-4, sBaseCenter - sPressCenter);
  const phiBase = phiMid - bestH * tMin;

  const vertU = new Float32Array(vCount);
  const vertRad = new Float32Array(vCount);
  const vertBin = new Float32Array(vCount);
  const vertTan = new Float32Array(vCount);

  for (let i = 0; i < vCount; i++) {
    const u = clamp((rawT[i] - tMin) / totalSweep, 0, 1);
    vertU[i] = u;

    const tRel = tMin + u * totalSweep;
    const phi = phiMid - bestH * tRel;
    const cosP = Math.cos(phi);
    const sinP = Math.sin(phi);

    const cx = R * cosP;
    const cy = R * sinP;
    const cs = sMid - cPitch * tRel;

    const tangent = normalize3([bestH * R * sinP, -bestH * R * cosP, -cPitch]);
    const radDotT = cosP * tangent[0] + sinP * tangent[1];
    const er = normalize3([
      cosP - radDotT * tangent[0],
      sinP - radDotT * tangent[1],
      -radDotT * tangent[2],
    ]);
    const eb = cross3(tangent, er);

    const dx = uRaw[i] - cU - cx;
    const dy = vRaw[i] - cV - cy;
    const ds = sArr[i] - cs;

    vertRad[i] = dx * er[0] + dy * er[1] + ds * er[2];
    vertBin[i] = dx * eb[0] + dy * eb[1] + ds * eb[2];
    vertTan[i] = dx * tangent[0] + dy * tangent[1] + ds * tangent[2];
  }

  return {
    eU,
    eV,
    eAx,
    cU,
    cV,
    sNear: sMin,
    sFar: sMax,
    sBaseCenter,
    sPressCenter,
    freeCenterlineH,
    meanCoilRadius: R,
    wireRadius: rW,
    pitch0,
    turns,
    totalSweep,
    phiBase,
    handedness: bestH,
    vertU,
    vertRad,
    vertBin,
    vertTan,
  };
}

/**
 * Evaluates true Kirchhoff helical rod compression, progressive coil-to-coil bind contact,
 * arc-length radial barreling, torsional wind-up, and plastic lateral buckling without squashing
 * the wire cross-section.
 */
function evaluateHelixDeformationField(
  part: CadPart,
  mat: MaterialSpec,
  press: DetectedPressAssembly,
  helix: HelixKinematicsData,
  compressionM: number,
  appliedForceN: number,
  accumPlastic: Float32Array,
  accumDamage: Float32Array,
  dt: number,
  brittleness: number
): {
  displacements: Float32Array;
  nodalStressMPa: Float32Array;
  nodalPlasticStrain: Float32Array;
  nodalDamage: Float32Array;
  peakStressMPa: number;
  maxDispMm: number;
  maxPlasticPct: number;
  maxDamage: number;
  elasticEnergyJ: number;
  plasticEnergyDeltaJ: number;
} {
  const verts = part.mesh.vertices;
  const vCount = part.mesh.vertexCount;
  const disp = new Float32Array(vCount * 3);
  const stressArr = new Float32Array(vCount);
  const epArr = new Float32Array(vCount);
  const dmgArr = new Float32Array(vCount);

  const yieldMPa = Math.max(30, mat.yieldStrength / 1e6);
  const youngsMPa = Math.max(1000, mat.youngsModulus / 1e6);
  const hardeningMPa = Math.max(120, mat.hardeningModulusPa / 1e6 || youngsMPa * 0.02);
  const epsFrac = Math.max(0.02, mat.fractureStrain || 0.12);

  const tauWahlPa =
    (press.wahlFactor * 8 * appliedForceN * press.meanCoilDiameterM) /
    Math.max(1e-12, Math.PI * Math.pow(press.wireDiameterM, 3));
  const nominalPeakMPa = Math.min(4500, (tauWahlPa * 1.15) / 1e6);

  // Measure previous accumulated mean plastic strain to drive continuous irreversible plastic buckling
  let meanPlasticPrev = 0;
  for (let i = 0; i < vCount; i++) meanPlasticPrev += accumPlastic[i];
  meanPlasticPrev /= Math.max(1, vCount);

  const R0 = helix.meanCoilRadius;
  const rW = helix.wireRadius;
  const wireDiam = 2 * rW;
  const H0 = helix.freeCenterlineH;
  const P0 = helix.pitch0;
  const N = helix.turns;

  // Target compressed centerline span H_curr (anchored at u=0 base end, compressed at u=1 press end)
  const Hcurr = Math.max(wireDiam * 1.15, H0 - Math.max(0, compressionM));
  const Pavg = Hcurr / Math.max(0.5, N);
  const Pbind = wireDiam * 0.992;
  const Pfloor =
    Pavg >= Pbind
      ? Pbind
      : Math.max(Pavg * 0.9, Pbind - (Pbind - Pavg) * 1.15);

  const bindStrokeRef = Math.max(1e-4, H0 - N * Pbind);
  const bindRatio = clamp(compressionM / bindStrokeRef, 0, 1.25);
  const overCrushM = Math.max(0, N * Pbind - Hcurr);
  const overCrushRatio = clamp(overCrushM / Math.max(1e-4, N * Pbind * 0.28), 0, 1);
  const plasticNorm = clamp(meanPlasticPrev / 0.045, 0, 1);

  // Solve progressive non-uniform coil pitch p[k] >= Pfloor with mean(p) = Pavg
  const M = 128;
  const pitchGrid = new Float64Array(M);
  const weightGrid = new Float64Array(M);
  for (let k = 0; k < M; k++) {
    const uMid = (k + 0.5) / M;
    // Active mid-coils are ~45% more compliant than plate-seated end turns -> center coils bind first
    weightGrid[k] = 0.8 + 0.38 * Math.sin(Math.PI * uMid);
  }

  let lamLo = 0;
  let lamHi = Math.max(0, P0 - Pfloor) * 4.5 + 1e-4;
  for (let iter = 0; iter < 24; iter++) {
    const lamMid = 0.5 * (lamLo + lamHi);
    let sumP = 0;
    for (let k = 0; k < M; k++) {
      sumP += Math.max(Pfloor, P0 - lamMid * weightGrid[k]);
    }
    if (sumP / M > Pavg) lamLo = lamMid;
    else lamHi = lamMid;
  }
  const lamSol = 0.5 * (lamLo + lamHi);
  let sumP = 0;
  for (let k = 0; k < M; k++) {
    const pk = Math.max(Pfloor, P0 - lamSol * weightGrid[k]);
    pitchGrid[k] = pk;
    sumP += pk;
  }
  const avgSolved = sumP / M;
  const corrScale = avgSolved > 1e-9 ? Pavg / avgSolved : 1;

  // Build deformed centerline nodes C_def[0..M] and orthonormal frames (T, e_r, e_b)
  const cX = new Float64Array(M + 1);
  const cY = new Float64Array(M + 1);
  const cS = new Float64Array(M + 1);
  const thGrid = new Float64Array(M + 1);
  const nodePitch = new Float64Array(M + 1);

  const sGrid = new Float64Array(M + 1);
  sGrid[0] = helix.sBaseCenter;
  const dTurns = N / M;
  for (let k = 0; k < M; k++) {
    const pk = pitchGrid[k] * corrScale;
    pitchGrid[k] = pk;
    sGrid[k + 1] = sGrid[k] - pk * dTurns;
  }

  for (let k = 0; k <= M; k++) {
    const u = k / M;
    const pk = k === 0 ? pitchGrid[0] : k === M ? pitchGrid[M - 1] : 0.5 * (pitchGrid[k - 1] + pitchGrid[k]);
    nodePitch[k] = pk;

    // Wire arc-length conservation as local pitch drops from P0 to pk
    const rArc = Math.sqrt(
      R0 * R0 + Math.max(0, P0 * P0 - pk * pk) / (4 * Math.PI * Math.PI)
    );
    const endClamp = Math.sin(Math.PI * u);
    const barrelBoost =
      1 +
      (0.035 * Math.min(1, bindRatio) + 0.085 * plasticNorm + 0.075 * overCrushRatio) *
        Math.pow(endClamp, 0.85);
    const coilSlipStagger =
      overCrushM * 0.44 * endClamp * Math.cos(Math.PI * N * u);
    const rDef = (R0 + (rArc - R0) * endClamp) * barrelBoost + coilSlipStagger;

    // Clamped torsional wind-up & plastic lateral buckling bow
    const deltaTheta =
      -helix.handedness *
      (0.12 * Math.min(1, bindRatio) + 0.16 * plasticNorm) *
      Math.sin(Math.PI * u);
    const theta = helix.phiBase - helix.handedness * u * helix.totalSweep + deltaTheta;
    thGrid[k] = theta;

    const buckleAmp = R0 * (0.11 * plasticNorm + 0.17 * overCrushRatio);
    const buckleEnv = Math.pow(endClamp, 1.35);

    cX[k] = rDef * Math.cos(theta) + buckleAmp * buckleEnv * 0.86;
    cY[k] = rDef * Math.sin(theta) + buckleAmp * buckleEnv * 0.51;
    cS[k] = sGrid[k];
  }

  const tX = new Float64Array(M + 1);
  const tY = new Float64Array(M + 1);
  const tS = new Float64Array(M + 1);
  const erX = new Float64Array(M + 1);
  const erY = new Float64Array(M + 1);
  const erS = new Float64Array(M + 1);
  const ebX = new Float64Array(M + 1);
  const ebY = new Float64Array(M + 1);
  const ebS = new Float64Array(M + 1);

  for (let k = 0; k <= M; k++) {
    const k0 = Math.max(0, k - 1);
    const k1 = Math.min(M, k + 1);
    const T = normalize3([cX[k1] - cX[k0], cY[k1] - cY[k0], cS[k1] - cS[k0]]);
    const cosP = Math.cos(thGrid[k]);
    const sinP = Math.sin(thGrid[k]);
    const rDotT = cosP * T[0] + sinP * T[1];
    const er = normalize3([
      cosP - rDotT * T[0],
      sinP - rDotT * T[1],
      -rDotT * T[2],
    ]);
    const eb = cross3(T, er);

    tX[k] = T[0]; tY[k] = T[1]; tS[k] = T[2];
    erX[k] = er[0]; erY[k] = er[1]; erS[k] = er[2];
    ebX[k] = eb[0]; ebY[k] = eb[1]; ebS[k] = eb[2];
  }

  // Keep cross-section circular through bind; mild Poisson contact bulge only under over-crush
  const scaleRad = 1 + 0.07 * overCrushRatio;
  const scaleBin = 1 - 0.05 * overCrushRatio;
  const sPlatePress = helix.sNear + Math.max(0, compressionM);
  const sPlateBase = helix.sFar;

  let peakStressMPa = 0;
  let maxDispMm = 0;
  let maxPlasticPct = 0;
  let maxDamage = 0;
  let stressSqSum = 0;
  let plasticIncSum = 0;

  const { eU, eV, eAx, cU, cV } = helix;

  for (let vi = 0; vi < vCount; vi++) {
    const u = helix.vertU[vi];
    const fIdx = u * M;
    const k0 = Math.min(M - 1, Math.floor(fIdx));
    const frac = fIdx - k0;
    const k1 = k0 + 1;

    const cx = cX[k0] + (cX[k1] - cX[k0]) * frac;
    const cy = cY[k0] + (cY[k1] - cY[k0]) * frac;
    const cs = cS[k0] + (cS[k1] - cS[k0]) * frac;

    const tx = tX[k0] + (tX[k1] - tX[k0]) * frac;
    const ty = tY[k0] + (tY[k1] - tY[k0]) * frac;
    const ts = tS[k0] + (tS[k1] - tS[k0]) * frac;
    const tLen = Math.hypot(tx, ty, ts) || 1;
    const uxT = tx / tLen, uyT = ty / tLen, usT = ts / tLen;

    const rxRaw = erX[k0] + (erX[k1] - erX[k0]) * frac;
    const ryRaw = erY[k0] + (erY[k1] - erY[k0]) * frac;
    const rsRaw = erS[k0] + (erS[k1] - erS[k0]) * frac;
    const rLen = Math.hypot(rxRaw, ryRaw, rsRaw) || 1;
    const uxR = rxRaw / rLen, uyR = ryRaw / rLen, usR = rsRaw / rLen;

    const bxRaw = ebX[k0] + (ebX[k1] - ebX[k0]) * frac;
    const byRaw = ebY[k0] + (ebY[k1] - ebY[k0]) * frac;
    const bsRaw = ebS[k0] + (ebS[k1] - ebS[k0]) * frac;
    const bLen = Math.hypot(bxRaw, byRaw, bsRaw) || 1;
    const uxB = bxRaw / bLen, uyB = byRaw / bLen, usB = bsRaw / bLen;

    const vR = helix.vertRad[vi] * scaleRad;
    const vB = helix.vertBin[vi] * scaleBin;
    const vT = helix.vertTan[vi];

    const defU = cU + cx + vR * uxR + vB * uxB + vT * uxT;
    const defV = cV + cy + vR * uyR + vB * uyB + vT * uyT;
    let defS = cs + vR * usR + vB * usB + vT * usT;

    // Prevent end-turn skin from poking past the moving press plate or fixed base plate
    if (defS < sPlatePress) defS = sPlatePress;
    else if (defS > sPlateBase) defS = sPlateBase;

    const defX = defU * eU[0] + defV * eV[0] + defS * eAx[0];
    const defY = defU * eU[1] + defV * eV[1] + defS * eAx[1];
    const defZ = defU * eU[2] + defV * eV[2] + defS * eAx[2];

    const o = vi * 3;
    const dx = defX - verts[o];
    const dy = defY - verts[o + 1];
    const dz = defZ - verts[o + 2];

    disp[o] = dx;
    disp[o + 1] = dy;
    disp[o + 2] = dz;

    const dMm = Math.hypot(dx, dy, dz) * 1000;
    if (dMm > maxDispMm) maxDispMm = dMm;

    // Physical Wahl inner-coil curvature stress concentration + local coil-bind contact loading
    const uRadNorm = clamp(helix.vertRad[vi] / Math.max(1e-5, rW), -1, 1);
    const innerCoilFactor = 0.5 - 0.5 * uRadNorm; // 1.0 on inner coil face, 0.0 on outer face
    const innerCoilBoost = 0.64 + 0.36 * innerCoilFactor;

    const pLocal = nodePitch[k0] + (nodePitch[k1] - nodePitch[k0]) * frac;
    const localClosure = clamp((P0 - pLocal) / Math.max(1e-5, P0 - Pfloor), 0, 1);
    const midEnv = Math.sin(Math.PI * u);
    const axialProfile = 0.58 + 0.42 * (0.3 + 0.45 * midEnv + 0.25 * localClosure);

    const localStress = nominalPeakMPa * innerCoilBoost * axialProfile;
    stressArr[vi] = localStress;
    stressSqSum += localStress * localStress;
    if (localStress > peakStressMPa) peakStressMPa = localStress;

    if (localStress > yieldMPa && dt > 0) {
      const overstress = localStress - yieldMPa;
      const dEp =
        (overstress / (hardeningMPa + youngsMPa * 0.045)) * dt * 1.65;
      const nextEp = Math.min(epsFrac * 1.6, accumPlastic[vi] + dEp);
      const inc = nextEp - accumPlastic[vi];
      accumPlastic[vi] = nextEp;
      plasticIncSum += inc;

      const dmgInc =
        (inc / epsFrac) *
        (0.75 + 0.85 * innerCoilBoost) *
        (0.75 + 0.65 * brittleness);
      accumDamage[vi] = Math.min(1.0, accumDamage[vi] + dmgInc);
    }

    epArr[vi] = accumPlastic[vi];
    dmgArr[vi] = accumDamage[vi];
    const epPct = accumPlastic[vi] * 100;
    if (epPct > maxPlasticPct) maxPlasticPct = epPct;
    if (accumDamage[vi] > maxDamage) maxDamage = accumDamage[vi];
  }

  const volM3 = Math.max(1e-7, part.mesh.volume || 6.9e-6);
  const avgStressSqPa2 = (stressSqSum / Math.max(1, vCount)) * 1e12;
  const elasticEnergyJ =
    (avgStressSqPa2 / Math.max(1e6, 2 * mat.youngsModulus)) * volM3 * 0.5 +
    0.5 * press.stiffnessNPerM * compressionM * compressionM;
  const plasticEnergyDeltaJ =
    plasticIncSum > 0
      ? mat.yieldStrength * (plasticIncSum / Math.max(1, vCount)) * volM3 * 1.15
      : 0;

  return {
    displacements: disp,
    nodalStressMPa: stressArr,
    nodalPlasticStrain: epArr,
    nodalDamage: dmgArr,
    peakStressMPa,
    maxDispMm,
    maxPlasticPct,
    maxDamage,
    elasticEnergyJ,
    plasticEnergyDeltaJ,
  };
}

/**
 * Evaluates elastoplastic compression, stress, plastic strain, and damage fields for a single
 * specimen vertex mesh at a given compressive stroke and applied load.
 */
function evaluateSpecimenDeformationField(
  part: CadPart,
  mat: MaterialSpec,
  press: DetectedPressAssembly,
  compressionM: number,
  appliedForceN: number,
  accumPlastic: Float32Array,
  accumDamage: Float32Array,
  dt: number,
  brittleness: number,
  helixKinematics?: HelixKinematicsData | null
): {
  displacements: Float32Array;
  nodalStressMPa: Float32Array;
  nodalPlasticStrain: Float32Array;
  nodalDamage: Float32Array;
  peakStressMPa: number;
  maxDispMm: number;
  maxPlasticPct: number;
  maxDamage: number;
  elasticEnergyJ: number;
  plasticEnergyDeltaJ: number;
} {
  if (helixKinematics) {
    return evaluateHelixDeformationField(
      part,
      mat,
      press,
      helixKinematics,
      compressionM,
      appliedForceN,
      accumPlastic,
      accumDamage,
      dt,
      brittleness
    );
  }

  const verts = part.mesh.vertices;
  const vCount = part.mesh.vertexCount;
  const disp = new Float32Array(vCount * 3);
  const stressArr = new Float32Array(vCount);
  const epArr = new Float32Array(vCount);
  const dmgArr = new Float32Array(vCount);

  const pushDir = press.pushDir;
  const aabb = getWorldAabb(part);
  const [sNear, sFar] = projectAabbOnAxis(aabb, pushDir);
  const L0 = Math.max(0.004, sFar - sNear);
  const compRatio = Math.max(0, Math.min(0.85, compressionM / L0));

  const yieldMPa = Math.max(30, mat.yieldStrength / 1e6);
  const youngsMPa = Math.max(1000, mat.youngsModulus / 1e6);
  const hardeningMPa = Math.max(120, mat.hardeningModulusPa / 1e6 || youngsMPa * 0.02);
  const epsFrac = Math.max(0.02, mat.fractureStrain || 0.12);

  // Analytical peak stress from applied load F(t):
  // - Helical spring: Wahl torsional shear stress τ_W = K_W * 8 * F * D / (π * d^3), σ_vM = √3 * τ_W
  // - General solid: axial compression σ = F / A * Kt
  let nominalPeakMPa = 0;
  if (press.isHelicalSpring) {
    const tauWahlPa =
      (press.wahlFactor * 8 * appliedForceN * press.meanCoilDiameterM) /
      Math.max(1e-12, Math.PI * Math.pow(press.wireDiameterM, 3));
    nominalPeakMPa = Math.min(4500, (tauWahlPa * 1.15) / 1e6);
  } else {
    const areaM2 = Math.max(1e-5, (part.mesh.volume || 1e-5) / L0);
    nominalPeakMPa = Math.min(4500, (appliedForceN / areaM2 / 1e6) * 1.35);
  }

  // Radial bulge & buckling parameters
  const radialScale = press.isHelicalSpring
    ? 0.11 * compRatio + 0.16 * Math.max(0, compRatio - 0.35)
    : 0.24 * compRatio;

  const center = aabb.center;
  const maxRadialSpan = Math.max(
    0.004,
    0.5 *
      Math.max(
        Math.abs(aabb.size[0] * (1 - Math.abs(pushDir[0]))),
        Math.abs(aabb.size[1] * (1 - Math.abs(pushDir[1]))),
        Math.abs(aabb.size[2] * (1 - Math.abs(pushDir[2])))
      )
  );

  let peakStressMPa = 0;
  let maxDispMm = 0;
  let maxPlasticPct = 0;
  let maxDamage = 0;
  let stressSqSum = 0;
  let plasticIncSum = 0;

  for (let vi = 0; vi < vCount; vi++) {
    const o = vi * 3;
    const wx = part.position[0] + verts[o];
    const wy = part.position[1] + verts[o + 1];
    const wz = part.position[2] + verts[o + 2];

    const s = wx * pushDir[0] + wy * pushDir[1] + wz * pushDir[2];
    // eta = 1 at press-contact face (sNear), eta = 0 at fixed base face (sFar)
    const eta = Math.max(0, Math.min(1, (sFar - s) / L0));

    // Radial vector perpendicular to pushDir from specimen axis
    const rel: Vec3 = [wx - center[0], wy - center[1], wz - center[2]];
    const relAxial = dot3(rel, pushDir);
    const radX = rel[0] - relAxial * pushDir[0];
    const radY = rel[1] - relAxial * pushDir[1];
    const radZ = rel[2] - relAxial * pushDir[2];
    const r = Math.hypot(radX, radY, radZ);
    const rNorm = Math.min(1.2, r / maxRadialSpan);

    // Axial displacement tracks press plate linearly from 0 at base to +compressionM along pushDir at top
    const axialShift = compressionM * eta;

    // Mid-height barreling / coil radial expansion + slight asymmetric plastic buckle when yielding
    const midEnvelope = Math.sin(Math.PI * eta);
    const buckleFactor =
      accumPlastic[vi] > 0.005 ? Math.min(0.28, accumPlastic[vi] * 1.4) : 0;
    const radExpand = (radialScale + buckleFactor * 0.14) * midEnvelope;

    const dx = pushDir[0] * axialShift + radX * radExpand;
    const dy = pushDir[1] * axialShift + radY * radExpand;
    const dz = pushDir[2] * axialShift + radZ * radExpand;

    disp[o] = dx;
    disp[o + 1] = dy;
    disp[o + 2] = dz;

    const dMm = Math.hypot(dx, dy, dz) * 1000;
    if (dMm > maxDispMm) maxDispMm = dMm;

    // Spatial stress concentration:
    // - On a helical spring, Wahl shear peaks on the INNER coil radius (small rNorm) and active mid-coils!
    // - On a solid block/cylinder, stress concentrates at mid-height & contact ends.
    const innerCoilBoost = press.isHelicalSpring
      ? 0.62 + 0.38 * (1 - Math.min(1, Math.max(0, (rNorm - 0.55) / 0.45)))
      : 0.72 + 0.28 * midEnvelope;
    const axialProfile = 0.55 + 0.45 * (0.35 + 0.65 * midEnvelope);
    const localStress = nominalPeakMPa * innerCoilBoost * axialProfile;
    stressArr[vi] = localStress;
    stressSqSum += localStress * localStress;
    if (localStress > peakStressMPa) peakStressMPa = localStress;

    // Irreversible J2 plastic strain & continuum damage
    if (localStress > yieldMPa && dt > 0) {
      const overstress = localStress - yieldMPa;
      const dEp =
        (overstress / (hardeningMPa + youngsMPa * 0.045)) * dt * 1.65;
      const nextEp = Math.min(epsFrac * 1.6, accumPlastic[vi] + dEp);
      const inc = nextEp - accumPlastic[vi];
      accumPlastic[vi] = nextEp;
      plasticIncSum += inc;

      const dmgInc =
        (inc / epsFrac) *
        (0.75 + 0.85 * innerCoilBoost) *
        (0.75 + 0.65 * brittleness);
      accumDamage[vi] = Math.min(1.0, accumDamage[vi] + dmgInc);
    }

    epArr[vi] = accumPlastic[vi];
    dmgArr[vi] = accumDamage[vi];
    const epPct = accumPlastic[vi] * 100;
    if (epPct > maxPlasticPct) maxPlasticPct = epPct;
    if (accumDamage[vi] > maxDamage) maxDamage = accumDamage[vi];
  }

  const volM3 = Math.max(1e-7, part.mesh.volume || 6.9e-6);
  const avgStressSqPa2 = (stressSqSum / Math.max(1, vCount)) * 1e12;
  const elasticEnergyJ =
    (avgStressSqPa2 / Math.max(1e6, 2 * mat.youngsModulus)) * volM3 * 0.5 +
    0.5 * press.stiffnessNPerM * compressionM * compressionM;
  const plasticEnergyDeltaJ =
    plasticIncSum > 0
      ? mat.yieldStrength * (plasticIncSum / Math.max(1, vCount)) * volM3 * 1.15
      : 0;

  return {
    displacements: disp,
    nodalStressMPa: stressArr,
    nodalPlasticStrain: epArr,
    nodalDamage: dmgArr,
    peakStressMPa,
    maxDispMm,
    maxPlasticPct,
    maxDamage,
    elasticEnergyJ,
    plasticEnergyDeltaJ,
  };
}

/**
 * Bakes a physically grounded axial pressing, elastoplastic compression, coil-bind,
 * and overload destruction simulation cache for any detected prismatic press assembly.
 */
export function bakePressDestructionCache(
  parts: readonly CadPart[],
  joints: readonly KinematicJoint[],
  materials: readonly MaterialSpec[],
  config: DestructionSimConfig,
  press: DetectedPressAssembly,
  solverLabel = 'Пресс-FEM'
): DestructionCache {
  const startMs = typeof performance !== 'undefined' ? performance.now() : Date.now();
  const safeFps = Math.max(24, Math.min(120, Math.round(config.fps || 60)));
  const safeDuration = Math.max(1.2, Math.min(6.0, config.durationSec || 2.5));
  const totalFrames = Math.round(safeDuration * safeFps);
  const dt = 1 / safeFps;
  const rng = mulberry32((config.seed || 42) + 1337);

  const visibleParts = parts.filter((p) => p.visible);
  const primarySpec = press.specimenParts[0];
  const primaryMat = getMaterialById(primarySpec.materialId, materials as MaterialSpec[]);
  const yieldRefMPa = Math.max(30, primaryMat.yieldStrength / 1e6);

  // Floor plane sits right below the lowest Y vertex of the assembly
  let floorY = Infinity;
  for (const p of visibleParts) {
    const aabb = getWorldAabb(p);
    if (aabb.min[1] < floorY) floorY = aabb.min[1];
  }
  if (!Number.isFinite(floorY)) floorY = -0.05;

  // Precompute helical rod kinematics and plastic & damage accumulators for specimen parts
  const helixByPart = new Map<string, HelixKinematicsData | null>();
  const plasticByPart = new Map<string, Float32Array>();
  const damageByPart = new Map<string, Float32Array>();
  const plasticEnergyByPart = new Map<string, number>();
  for (const sp of press.specimenParts) {
    helixByPart.set(
      sp.id,
      isHelixPart(sp) ? buildHelixKinematicsData(sp, press) : null
    );
    plasticByPart.set(sp.id, new Float32Array(sp.mesh.vertexCount));
    damageByPart.set(sp.id, new Float32Array(sp.mesh.vertexCount));
    plasticEnergyByPart.set(sp.id, 0);
  }

  // Compute physical coil-bind force and whether overload destroys the specimen
  const bindForceN = press.stiffnessNPerM * press.maxStrokeM;
  const effectiveMaxForceN = press.maxForceN;
  const reachesBind = effectiveMaxForceN >= bindForceN * 0.85;
  const maxAchievableCompressionM = reachesBind
    ? press.maxStrokeM
    : Math.min(press.maxStrokeM, effectiveMaxForceN / Math.max(1, press.stiffnessNPerM));

  // Build fracture shards from the pre-fracture compressed/buckled specimen geometry
  // so broken coil arcs seamlessly continue from the compressed spring state
  const allShards: FractureShardGeometry[] = [];
  interface PressShardRt {
    parentPartId: string;
    localCom: Vec3;
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
  const shardRuntimes: PressShardRt[] = [];

  for (let si = 0; si < press.specimenParts.length; si++) {
    const sp = press.specimenParts[si];
    const mat = getMaterialById(sp.materialId, materials as MaterialSpec[]);
    const hData = helixByPart.get(sp.id) ?? null;

    let shardSourcePart = sp;
    if (hData) {
      const preCompM =
        maxAchievableCompressionM + (reachesBind ? press.solidHeightM * 0.22 : 0);
      const dummyPlastic = new Float32Array(sp.mesh.vertexCount).fill(0.045);
      const dummyDamage = new Float32Array(sp.mesh.vertexCount);
      const preEval = evaluateHelixDeformationField(
        sp,
        mat,
        press,
        hData,
        preCompM,
        effectiveMaxForceN,
        dummyPlastic,
        dummyDamage,
        0,
        config.brittleness ?? 0.55
      );
      const defVerts = new Float32Array(sp.mesh.vertices.length);
      let minX = Infinity, minY = Infinity, minZ = Infinity;
      let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
      for (let vi = 0; vi < sp.mesh.vertexCount; vi++) {
        const o = vi * 3;
        const x = sp.mesh.vertices[o] + preEval.displacements[o];
        const y = sp.mesh.vertices[o + 1] + preEval.displacements[o + 1];
        const z = sp.mesh.vertices[o + 2] + preEval.displacements[o + 2];
        defVerts[o] = x;
        defVerts[o + 1] = y;
        defVerts[o + 2] = z;
        if (x < minX) minX = x;
        if (y < minY) minY = y;
        if (z < minZ) minZ = z;
        if (x > maxX) maxX = x;
        if (y > maxY) maxY = y;
        if (z > maxZ) maxZ = z;
      }
      shardSourcePart = {
        ...sp,
        mesh: {
          ...sp.mesh,
          vertices: defVerts,
          boundingBox: { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] },
        },
      };
    }

    const built = buildSpecimenShards(
      shardSourcePart,
      mat,
      Math.min(18, Math.max(8, config.shardCountPerPart || 14)),
      (config.seed || 42) + si * 97,
      hData?.vertU ?? null
    );
    for (const s of built.shards) {
      allShards.push(s);
      shardRuntimes.push({
        parentPartId: sp.id,
        localCom: s.localCom,
        boundingRadius: s.boundingRadius,
        mass: s.mass,
        active: false,
        pos: add3(sp.position, s.localCom),
        quat: [0, 0, 0, 1],
        vel: [0, 0, 0],
        omega: [0, 0, 0],
        stressGlow: 0,
        stressMPa: 0,
        plasticStrain: 0,
        damage: 0,
      });
    }
  }

  interface SparkRt {
    pos: Vec3;
    vel: Vec3;
    life: number;
    maxLife: number;
  }
  const sparks: SparkRt[] = [];

  const spawnSparks = (origin: Vec3, count: number, speed: number) => {
    for (let i = 0; i < count; i++) {
      if (sparks.length >= 160) break;
      sparks.push({
        pos: [
          origin[0] + (rng() - 0.5) * 0.012,
          origin[1] + (rng() - 0.5) * 0.012,
          origin[2] + (rng() - 0.5) * 0.012,
        ],
        vel: [
          (rng() - 0.5) * speed * 1.8,
          (rng() * 0.7 + 0.2) * speed,
          (rng() - 0.5) * speed * 1.8,
        ],
        life: 1.0,
        maxLife: 0.22 + rng() * 0.34,
      });
    }
  };

  // Contact point is at the middle of the specimen's contact interface
  const specAabb = getWorldAabb(primarySpec);
  const contactPoint: Vec3 = [...specAabb.center];

  // Phase timings (fractions of safeDuration):
  // - 0.00 -> 0.54: smooth press compression stroke to maxAchievableCompressionM (coil bind at ~0.50)
  // - 0.54 -> 0.72: solid-bind overload crush & plastic flattening (force spikes from F_solid to F_max=2000 N!)
  // - 0.72 -> 1.00: overload fracture / shatter if damage reaches critical threshold, else hold & elastic rebound
  const tCompressEnd = safeDuration * 0.54;
  const tOverloadEnd = safeDuration * 0.72;

  const events: DestructionTimelineEvent[] = [];
  const frames: DestructionCacheFrame[] = [];

  let contactLogged = false;
  let yieldLogged = false;
  let coilBindLogged = false;
  let fractured = false;
  let floorBounceLogged = false;

  let maxStressOverallMPa = 0;
  let maxPlasticOverallPct = 0;
  let maxDamageOverall = 0;
  let maxEnergyOverallJ = 1e-4;
  let totalFractureEnergyJ = 0;

  let prevPressTravelM = 0;

  for (let frameIdx = 0; frameIdx <= totalFrames; frameIdx++) {
    const timeSec = Number((frameIdx * dt).toFixed(4));

    // Smooth S-curve press stroke profile
    const uStroke = Math.max(0, Math.min(1, timeSec / Math.max(0.1, tCompressEnd)));
    const smoothStroke = uStroke * uStroke * (3 - 2 * uStroke);
    const totalStrokeTarget = press.initialGapM + maxAchievableCompressionM;

    let pressTravelM = totalStrokeTarget * smoothStroke;
    let compressionM = Math.max(0, pressTravelM - press.initialGapM);
    let appliedForceN = 0;

    if (compressionM > 1e-5) {
      // Elastic/elastoplastic spring force during travel
      const linearF = press.stiffnessNPerM * compressionM;
      if (timeSec <= tCompressEnd) {
        // As we approach solid height (uStroke > 0.78), coil-to-coil contact stiffens rapidly
        const bindRamp = Math.max(0, (uStroke - 0.76) / 0.24);
        appliedForceN = Math.min(
          effectiveMaxForceN,
          linearF + (effectiveMaxForceN - linearF) * bindRamp * bindRamp * 0.72
        );
      } else if (timeSec <= tOverloadEnd) {
        // Coil bind / solid crushing phase: force surges to full F_max = 2000 N with slight extra plastic crush
        const uOver = (timeSec - tCompressEnd) / Math.max(0.05, tOverloadEnd - tCompressEnd);
        const smoothOver = uOver * (2 - uOver);
        appliedForceN = Math.min(
          effectiveMaxForceN,
          linearF + (effectiveMaxForceN - linearF) * (0.72 + 0.28 * smoothOver)
        );
        const plasticExtraCrush = reachesBind ? press.solidHeightM * 0.22 * smoothOver : 0;
        compressionM = maxAchievableCompressionM + plasticExtraCrush;
        pressTravelM = press.initialGapM + compressionM;
      } else {
        // Post-overload phase: either shattered (force drops as press completes stop) or holding under load
        const uPost = (timeSec - tOverloadEnd) / Math.max(0.05, safeDuration - tOverloadEnd);
        const plasticExtraCrush = reachesBind ? press.solidHeightM * (fractured ? 0.34 : 0.22) : 0;
        compressionM = maxAchievableCompressionM + plasticExtraCrush;
        pressTravelM = press.initialGapM + compressionM;
        appliedForceN = fractured
          ? effectiveMaxForceN * Math.max(0.08, Math.exp(-uPost * 4.5))
          : effectiveMaxForceN * (1 - 0.15 * uPost);
      }
    }

    const pressSpeedMs = frameIdx > 0 ? Math.abs(pressTravelM - prevPressTravelM) / dt : 0;
    prevPressTravelM = pressTravelM;
    const pressMassKg = Math.max(0.05, press.pressPart.mesh.mass || 0.28);
    let kineticEnergyJ = 0.5 * pressMassKg * pressSpeedMs * pressSpeedMs;
    const scaleRef = Math.max(0.03, press.freeLengthM);
    const compressedCenter: Vec3 = add3(
      contactPoint,
      scale3(press.pushDir, compressionM * 0.5)
    );

    // Log contact event
    if (compressionM > 0.0002 && !contactLogged) {
      contactLogged = true;
      events.push({
        frameIndex: frameIdx,
        timeSec,
        type: 'contact',
        label: `Контакт пресса (${press.pressPart.name} → ${primarySpec.name})`,
        partId: primarySpec.id,
        stressMPa: 0,
      });
    }

    // Log coil bind event
    if (reachesBind && timeSec >= tCompressEnd * 0.92 && !coilBindLogged) {
      coilBindLogged = true;
      events.push({
        frameIndex: frameIdx,
        timeSec,
        type: 'tooth_shear',
        label: press.isHelicalSpring
          ? `Смыкание витков (−${(maxAchievableCompressionM * 1000).toFixed(1)} мм, F → ${Math.round(effectiveMaxForceN)} Н)`
          : `Предельное сжатие (−${(maxAchievableCompressionM * 1000).toFixed(1)} мм)`,
        partId: primarySpec.id,
        stressMPa: Number(maxStressOverallMPa.toFixed(1)),
      });
      spawnSparks(compressedCenter, 26, scaleRef * 5.2);
    }

    let framePeakStress = 0;
    let frameMaxPlasticPct = 0;
    let frameMaxDamage = 0;
    let frameMaxDispMm = 0;
    let totalElasticEnergyJ = 0;
    let totalPlasticEnergyJ = 0;

    const partStates: Record<string, CachedPartFrameState> = {};

    // Evaluate specimen fields
    for (const sp of press.specimenParts) {
      const mat = getMaterialById(sp.materialId, materials as MaterialSpec[]);
      const pAccum = plasticByPart.get(sp.id)!;
      const dAccum = damageByPart.get(sp.id)!;

      const evalRes = evaluateSpecimenDeformationField(
        sp,
        mat,
        press,
        compressionM,
        appliedForceN,
        pAccum,
        dAccum,
        frameIdx > 0 && !fractured ? dt : 0,
        config.brittleness ?? 0.55,
        helixByPart.get(sp.id) ?? null
      );

      const prevPlJ = plasticEnergyByPart.get(sp.id) || 0;
      const nextPlJ = prevPlJ + evalRes.plasticEnergyDeltaJ;
      plasticEnergyByPart.set(sp.id, nextPlJ);

      totalElasticEnergyJ += fractured ? evalRes.elasticEnergyJ * 0.15 : evalRes.elasticEnergyJ;
      totalPlasticEnergyJ += nextPlJ;

      if (evalRes.peakStressMPa > framePeakStress) framePeakStress = evalRes.peakStressMPa;
      if (evalRes.maxPlasticPct > frameMaxPlasticPct) frameMaxPlasticPct = evalRes.maxPlasticPct;
      if (evalRes.maxDamage > frameMaxDamage) frameMaxDamage = evalRes.maxDamage;
      if (evalRes.maxDispMm > frameMaxDispMm) frameMaxDispMm = evalRes.maxDispMm;

      if (!yieldLogged && evalRes.peakStressMPa >= yieldRefMPa) {
        yieldLogged = true;
        events.push({
          frameIndex: frameIdx,
          timeSec,
          type: 'yield_onset',
          label: `Начало текучести σ_vM > σ_y (${Math.round(yieldRefMPa)} МПа): ${sp.name}`,
          partId: sp.id,
          stressMPa: Number(evalRes.peakStressMPa.toFixed(1)),
        });
      }

      // Trigger overload rupture when damage saturates after coil bind under severe overload
      if (
        !fractured &&
        timeSec >= tOverloadEnd &&
        evalRes.maxDamage >= 0.88 &&
        evalRes.peakStressMPa > yieldRefMPa * 1.35
      ) {
        fractured = true;
        events.push({
          frameIndex: frameIdx,
          timeSec,
          type: 'body_fracture',
          label: press.isHelicalSpring
            ? `Разрушение витков от перегрузки (${Math.round(effectiveMaxForceN)} Н): ${sp.name}`
            : `Разрушение образца (${Math.round(effectiveMaxForceN)} Н): ${sp.name}`,
          partId: sp.id,
          stressMPa: Number(evalRes.peakStressMPa.toFixed(1)),
        });

        // Launch fractured coil shards radially outward from the compressed specimen zone
        const [sNear, sFar] = projectAabbOnAxis(getWorldAabb(sp), press.pushDir);
        const L0 = Math.max(0.004, sFar - sNear);
        const hasHelixShards = Boolean(helixByPart.get(sp.id));
        for (const sr of shardRuntimes) {
          if (sr.parentPartId !== sp.id) continue;
          sr.active = true;
          const worldOrig = add3(sp.position, sr.localCom);
          if (hasHelixShards) {
            // Shards were already extracted from the compressed/buckled coil geometry
            sr.pos = worldOrig;
          } else {
            const sCoord = dot3(worldOrig, press.pushDir);
            const eta = Math.max(0, Math.min(1, (sFar - sCoord) / L0));
            sr.pos = add3(worldOrig, scale3(press.pushDir, compressionM * eta));
          }
          sr.quat = [0, 0, 0, 1];
          sr.stressGlow = 1.0;
          sr.stressMPa = evalRes.peakStressMPa;
          sr.plasticStrain = evalRes.maxPlasticPct / 100;
          sr.damage = 1.0;

          // Burst radially perpendicular to pushDir (symmetrically out of the press gap, staying cleanly inside viewport)
          const relFromC = sub3(sr.pos, compressedCenter);
          const relAx = dot3(relFromC, press.pushDir);
          const radial = normalize3([
            relFromC[0] - relAx * press.pushDir[0] + (rng() - 0.5) * 0.004,
            relFromC[1] - relAx * press.pushDir[1] + (rng() - 0.5) * 0.004,
            relFromC[2] - relAx * press.pushDir[2] + (rng() - 0.5) * 0.004,
          ]);
          const overloadMul = 0.85 + Math.min(0.65, effectiveMaxForceN / 3500);
          const burstSpeed = scaleRef * (1.8 + rng() * 1.9) * overloadMul;
          sr.vel = [
            radial[0] * burstSpeed,
            radial[1] * burstSpeed + 0.16 + rng() * 0.14,
            radial[2] * burstSpeed,
          ];
          sr.omega = [
            (rng() - 0.5) * 18,
            (rng() - 0.5) * 18,
            (rng() - 0.5) * 18,
          ];
          totalFractureEnergyJ += 0.5 * sr.mass * dot3(sr.vel, sr.vel) + 0.08;
        }
        spawnSparks(compressedCenter, 48, scaleRef * 7.0);
      }

      partStates[sp.id] = {
        position: [...sp.position],
        rotation: [...sp.rotation],
        stressRatio: evalRes.peakStressMPa / Math.max(1, yieldRefMPa),
        intact: !fractured,
        partiallyFractured: false,
        deformedDisplacements: evalRes.displacements,
        nodalStressMPa: evalRes.nodalStressMPa,
        nodalPlasticStrain: evalRes.nodalPlasticStrain,
        nodalDamage: evalRes.nodalDamage,
        maxDisplacementMm: Number(evalRes.maxDispMm.toFixed(3)),
        maxPlasticStrainPct: Number(evalRes.maxPlasticPct.toFixed(2)),
        maxDamage: Number(evalRes.maxDamage.toFixed(3)),
      };
    }

    // Press plate moves smoothly along pushDir by pressTravelM
    const pressOffset = scale3(press.pushDir, pressTravelM);
    partStates[press.pressPart.id] = {
      position: add3(press.pressPart.position, pressOffset),
      rotation: [...press.pressPart.rotation],
      stressRatio: Math.min(0.45, (appliedForceN / 25000)),
      intact: true,
      partiallyFractured: false,
      maxDisplacementMm: Number((pressTravelM * 1000).toFixed(2)),
    };

    // Base plate and any other fixed supports remain stationary
    for (const p of visibleParts) {
      if (partStates[p.id]) continue;
      partStates[p.id] = {
        position: [...p.position],
        rotation: [...p.rotation],
        stressRatio: 0,
        intact: true,
        partiallyFractured: false,
      };
    }

    // Emit periodic grinding/yielding sparks during high-force coil bind
    if (coilBindLogged && !fractured && appliedForceN > bindForceN * 1.5 && frameIdx % 3 === 0) {
      spawnSparks(compressedCenter, 5, scaleRef * 4.2);
    }

    // Integrate shards & sparks
    if (frameIdx > 0) {
      for (const sr of shardRuntimes) {
        if (!sr.active) continue;
        sr.vel[1] += (config.gravityY ?? -9.81) * dt;
        sr.vel[0] *= 0.995;
        sr.vel[1] *= 0.996;
        sr.vel[2] *= 0.995;
        sr.pos[0] += sr.vel[0] * dt;
        sr.pos[1] += sr.vel[1] * dt;
        sr.pos[2] += sr.vel[2] * dt;
        sr.quat = integrateQuat(sr.quat, sr.omega, dt);
        sr.omega[0] *= 0.99;
        sr.omega[1] *= 0.99;
        sr.omega[2] *= 0.99;
        sr.stressGlow = Math.max(0, sr.stressGlow - dt * 0.9);

        const minY = floorY + sr.boundingRadius * 0.5;
        if (sr.pos[1] < minY) {
          sr.pos[1] = minY;
          if (sr.vel[1] < -0.05) {
            if (!floorBounceLogged) {
              floorBounceLogged = true;
              events.push({
                frameIndex: frameIdx,
                timeSec,
                type: 'floor_bounce',
                label: 'Падение осколков на опору',
              });
            }
            sr.vel[1] = -sr.vel[1] * Math.max(0.1, Math.min(0.75, config.restitution || 0.35));
            sr.vel[0] *= 0.78;
            sr.vel[2] *= 0.78;
          } else {
            sr.vel[1] = 0;
            sr.vel[0] *= 0.85;
            sr.vel[2] *= 0.85;
          }
        }
      }

      for (let i = sparks.length - 1; i >= 0; i--) {
        const sp = sparks[i];
        sp.life -= dt / sp.maxLife;
        if (sp.life <= 0) {
          sparks.splice(i, 1);
          continue;
        }
        sp.vel[1] += (config.gravityY ?? -9.81) * 0.65 * dt;
        sp.pos[0] += sp.vel[0] * dt;
        sp.pos[1] += sp.vel[1] * dt;
        sp.pos[2] += sp.vel[2] * dt;
        if (sp.pos[1] < floorY + 0.001) {
          sp.pos[1] = floorY + 0.001;
          sp.vel[1] = Math.abs(sp.vel[1]) * 0.4;
        }
      }
    }

    // Pack shards & sparks into frame typed arrays
    const shardPos = new Float32Array(shardRuntimes.length * 3);
    const shardQuat = new Float32Array(shardRuntimes.length * 4);
    const shardActive = new Uint8Array(shardRuntimes.length);
    const shardStress = new Float32Array(shardRuntimes.length);
    const shardStressMPa = new Float32Array(shardRuntimes.length);
    const shardPlasticStrain = new Float32Array(shardRuntimes.length);
    const shardDamage = new Float32Array(shardRuntimes.length);

    let activeShardsCount = 0;
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

    if (framePeakStress > maxStressOverallMPa) maxStressOverallMPa = framePeakStress;
    if (frameMaxPlasticPct > maxPlasticOverallPct) maxPlasticOverallPct = frameMaxPlasticPct;
    if (frameMaxDamage > maxDamageOverall) maxDamageOverall = frameMaxDamage;
    maxEnergyOverallJ = Math.max(
      maxEnergyOverallJ,
      kineticEnergyJ,
      totalElasticEnergyJ,
      totalPlasticEnergyJ,
      totalFractureEnergyJ
    );

    const statusLabel = fractured
      ? 'Разрушение витков от перегрузки'
      : coilBindLogged
        ? 'Смыкание витков (пластика)'
        : yieldLogged
          ? 'Пластическая текучесть (σ > σ_y)'
          : compressionM > 0.0002
            ? 'Упругое сжатие пружины'
            : 'Подвод нажимной плиты';

    const metrics: DestructionFrameMetrics = {
      peakStressMPa: Number(framePeakStress.toFixed(1)),
      yieldLimitMPa: Number(yieldRefMPa.toFixed(0)),
      contactForceN: Number(appliedForceN.toFixed(0)),
      kineticEnergyJ: Number(kineticEnergyJ.toFixed(4)),
      elasticEnergyJ: Number(totalElasticEnergyJ.toFixed(4)),
      plasticEnergyJ: Number(totalPlasticEnergyJ.toFixed(4)),
      fractureEnergyJ: Number(totalFractureEnergyJ.toFixed(4)),
      maxPlasticStrainPct: Number(frameMaxPlasticPct.toFixed(2)),
      maxDamage: Number(frameMaxDamage.toFixed(3)),
      maxDisplacementMm: Number(Math.max(frameMaxDispMm, pressTravelM * 1000).toFixed(2)),
      activeShardsCount,
      totalShardsCount: shardRuntimes.length,
      fracturedPartsCount: fractured ? press.specimenParts.length : 0,
      intactPartsCount: fractured ? 0 : press.specimenParts.length,
      statusLabel,
    };

    frames.push({
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
    });
  }

  let cacheBytes = 0;
  for (const s of allShards) {
    cacheBytes += s.vertices.byteLength + s.normals.byteLength + s.indices.byteLength;
  }
  for (const f of frames) {
    cacheBytes +=
      f.shardPos.byteLength +
      f.shardQuat.byteLength +
      f.shardActive.byteLength +
      f.sparks.byteLength +
      160;
    for (const ps of Object.values(f.partStates)) {
      cacheBytes +=
        (ps.deformedDisplacements?.byteLength ?? 0) +
        (ps.nodalStressMPa?.byteLength ?? 0) +
        (ps.nodalPlasticStrain?.byteLength ?? 0) +
        (ps.nodalDamage?.byteLength ?? 0);
    }
  }

  const endMs = typeof performance !== 'undefined' ? performance.now() : Date.now();

  return {
    id: `press_cache_${Date.now().toString(36)}_${config.seed || 42}`,
    createdAt: new Date().toISOString(),
    solverBackend: 'web_local_preview',
    solverLabel,
    config: { ...config, scenarioMode: 'impact_crush' },
    fps: safeFps,
    totalFrames,
    durationSec: safeDuration,
    floorY,
    contactPoint,
    bakeTimeMs: Math.max(1, Number((endMs - startMs).toFixed(1))),
    cacheBytes,
    shards: allShards,
    frames,
    events,
    maxStressOverallMPa: Math.max(1, Number(maxStressOverallMPa.toFixed(1))),
    yieldStrengthRefMPa: Number(yieldRefMPa.toFixed(0)),
    maxPlasticStrainOverallPct: Math.max(0.5, Number(maxPlasticOverallPct.toFixed(2))),
    maxDamageOverall: Number(Math.max(0.01, maxDamageOverall).toFixed(3)),
    maxEnergyOverallJ: Number(maxEnergyOverallJ.toFixed(4)),
  };
}

/**
 * Converts a live Rust `run_physics_simulation` trajectory bundle from `scene.json`
 * into an immediately playable `DestructionCache`.
 *
 * - Correctly computes relative rigid-body transforms (`deltaPos`, `relQuat`) around each part's
 *   initial COM so world-space STL meshes never double-translate.
 * - If the assembly is a prismatic press / coil-spring rig where Rust rigid-body contact locked the
 *   non-deformable spring near zero stroke, seamlessly upgrades to full elastoplastic compression
 *   & destruction driven by the exact agent joint force and stroke limit.
 */
export function buildDestructionCacheFromLivePhysics(
  parts: readonly CadPart[],
  joints: readonly KinematicJoint[],
  materials: readonly MaterialSpec[],
  physicsSim: LivePhysicsSimulation,
  config: DestructionSimConfig
): DestructionCache {
  const startMs = typeof performance !== 'undefined' ? performance.now() : Date.now();
  const visibleParts = parts.filter((p) => p.visible);

  // Measure actual press plate stroke observed in Rust rigid-body physics
  const peakJointForce = Object.values(physicsSim.peak_joint_forces || {}).reduce(
    (acc, v) => Math.max(acc, Number.isFinite(v) ? Math.abs(v) : 0),
    0
  );
  const detectedPress = detectPressAssembly(
    parts,
    joints,
    materials,
    peakJointForce > 0 ? peakJointForce : undefined
  );

  if (detectedPress) {
    const pressTraj = physicsSim.trajectories.find(
      (t) => t.part_id === detectedPress.pressPart.id
    );
    let observedStrokeM = 0;
    if (pressTraj && pressTraj.samples.length >= 2) {
      const p0 = pressTraj.samples[0].position_m;
      for (const s of pressTraj.samples) {
        const travel = dot3(sub3(s.position_m, p0), detectedPress.pushDir);
        if (travel > observedStrokeM) observedStrokeM = travel;
      }
    }
    // If rigid-body contact prevented the rigid spring from compressing (< 20% of stroke),
    // run the full elastoplastic press & destruction simulation using the exact MCP joint force!
    if (observedStrokeM < detectedPress.maxStrokeM * 0.2) {
      return bakePressDestructionCache(
        parts,
        joints,
        materials,
        {
          ...config,
          durationSec: Math.max(2.0, Math.min(4.0, physicsSim.duration_s || 2.5)),
        },
        detectedPress,
        'MCP Пресс-FEM'
      );
    }
  }

  // General rigid-body trajectory replay (with exact COM-centered delta transforms)
  const trajMap = new Map(physicsSim.trajectories.map((t) => [t.part_id, t]));
  const sampleCount = Math.max(
    2,
    ...physicsSim.trajectories.map((t) => t.samples.length)
  );
  const durationSec = Math.max(0.1, physicsSim.duration_s || 1.0);
  const fps = Math.max(
    15,
    Math.min(120, Math.round((sampleCount - 1) / durationSec))
  );

  let floorY = Infinity;
  let yieldRefMPa = Infinity;
  for (const p of visibleParts) {
    const aabb = getWorldAabb(p);
    if (aabb.min[1] < floorY) floorY = aabb.min[1];
    if (!p.isFixed) {
      const mat = getMaterialById(p.materialId, materials as MaterialSpec[]);
      const y = mat.yieldStrength / 1e6;
      if (y > 0 && y < yieldRefMPa) yieldRefMPa = y;
    }
  }
  if (!Number.isFinite(floorY)) floorY = -0.05;
  if (!Number.isFinite(yieldRefMPa)) yieldRefMPa = 350;

  const events: DestructionTimelineEvent[] = [];
  for (const col of physicsSim.collisions || []) {
    if (col.event_type !== 'started') continue;
    const fIdx = Math.max(
      0,
      Math.min(
        sampleCount - 1,
        Math.round((col.time_s / durationSec) * (sampleCount - 1))
      )
    );
    events.push({
      frameIndex: fIdx,
      timeSec: Number(col.time_s.toFixed(4)),
      type: 'contact',
      label: `Контакт: ${col.body_a} ↔ ${col.body_b}`,
      partId: col.body_a,
    });
  }

  const frames: DestructionCacheFrame[] = [];
  let maxEnergyJ = 1e-4;
  let maxStressMPa = 0;

  // Optional press specimen enrichment if Rust physics DID move the press plate
  const plasticAccum = detectedPress
    ? new Float32Array(detectedPress.specimenParts[0].mesh.vertexCount)
    : null;
  const damageAccum = detectedPress
    ? new Float32Array(detectedPress.specimenParts[0].mesh.vertexCount)
    : null;
  const liveHelixKinematics =
    detectedPress && isHelixPart(detectedPress.specimenParts[0])
      ? buildHelixKinematicsData(detectedPress.specimenParts[0], detectedPress)
      : null;

  for (let i = 0; i < sampleCount; i++) {
    const u = i / Math.max(1, sampleCount - 1);
    const timeSec = Number((u * durationSec).toFixed(4));
    const partStates: Record<string, CachedPartFrameState> = {};
    let kineticJ = 0;
    let maxDispMm = 0;
    let framePeakStress = 0;
    let frameMaxPlasticPct = 0;
    let frameMaxDamage = 0;
    let pressTravelM = 0;

    for (const part of visibleParts) {
      const traj = trajMap.get(part.id);
      if (traj && traj.samples.length > 0) {
        const s0 = traj.samples[0];
        const sIdx = Math.min(
          traj.samples.length - 1,
          Math.round(u * (traj.samples.length - 1))
        );
        const s = traj.samples[sIdx];

        const q0 = quatNormalize(s0.orientation_quat);
        const qt = quatNormalize(s.orientation_quat);
        const qRel = quatNormalize(quatMultiply(qt, quatConjugate(q0)));
        const p0 = s0.position_m;
        const pt = s.position_m;

        // World-space STL vertices sit at `part.position + v`, with initial COM at `p0`.
        // Rotating around `p0` and translating to `pt` places vertex at `pt + qRel * (v - p0)`
        // => mesh.position = part.position + pt - qRel * p0, mesh.quaternion = qRel.
        const rotatedP0 = rotateVecByQuat(p0, qRel);
        const meshPos: Vec3 = [
          part.position[0] + pt[0] - rotatedP0[0],
          part.position[1] + pt[1] - rotatedP0[1],
          part.position[2] + pt[2] - rotatedP0[2],
        ];
        const dispM = len3(sub3(pt, p0));
        if (dispM * 1000 > maxDispMm) maxDispMm = dispM * 1000;

        if (detectedPress && part.id === detectedPress.pressPart.id) {
          pressTravelM = Math.max(0, dot3(sub3(pt, p0), detectedPress.pushDir));
        }

        const mass = Math.max(0.005, part.mesh.mass || 0.05);
        const v2 = dot3(s.linear_velocity_m_s, s.linear_velocity_m_s);
        kineticJ += 0.5 * mass * v2;

        partStates[part.id] = {
          position: meshPos,
          rotation: eulerFromQuatXYZ(qRel),
          quaternion: qRel,
          stressRatio: 0,
          intact: true,
          partiallyFractured: false,
          maxDisplacementMm: Number((dispM * 1000).toFixed(3)),
        };
      } else {
        partStates[part.id] = {
          position: [...part.position],
          rotation: [...part.rotation],
          stressRatio: 0,
          intact: true,
          partiallyFractured: false,
        };
      }
    }

    // Enrich compressed specimen in press assemblies when Rust physics moved the press plate
    if (detectedPress && plasticAccum && damageAccum && pressTravelM > 0.0002) {
      const sp = detectedPress.specimenParts[0];
      const mat = getMaterialById(sp.materialId, materials as MaterialSpec[]);
      const compressionM = Math.max(0, pressTravelM - detectedPress.initialGapM);
      const loadN = Math.min(
        detectedPress.maxForceN,
        Math.max(detectedPress.stiffnessNPerM * compressionM, peakJointForce * u)
      );
      const dtStep = durationSec / Math.max(1, sampleCount - 1);
      const ev = evaluateSpecimenDeformationField(
        sp,
        mat,
        detectedPress,
        compressionM,
        loadN,
        plasticAccum,
        damageAccum,
        i > 0 ? dtStep : 0,
        config.brittleness ?? 0.5,
        liveHelixKinematics
      );
      framePeakStress = ev.peakStressMPa;
      frameMaxPlasticPct = ev.maxPlasticPct;
      frameMaxDamage = ev.maxDamage;
      if (ev.peakStressMPa > maxStressMPa) maxStressMPa = ev.peakStressMPa;

      partStates[sp.id] = {
        position: [...sp.position],
        rotation: [...sp.rotation],
        stressRatio: ev.peakStressMPa / Math.max(1, yieldRefMPa),
        intact: true,
        partiallyFractured: false,
        deformedDisplacements: ev.displacements,
        nodalStressMPa: ev.nodalStressMPa,
        nodalPlasticStrain: ev.nodalPlasticStrain,
        nodalDamage: ev.nodalDamage,
        maxDisplacementMm: Number(ev.maxDispMm.toFixed(3)),
        maxPlasticStrainPct: Number(ev.maxPlasticPct.toFixed(2)),
        maxDamage: Number(ev.maxDamage.toFixed(3)),
      };
    }

    if (kineticJ > maxEnergyJ) maxEnergyJ = kineticJ;

    const metrics: DestructionFrameMetrics = {
      peakStressMPa: Number(framePeakStress.toFixed(1)),
      yieldLimitMPa: Number(yieldRefMPa.toFixed(0)),
      contactForceN: Number(peakJointForce.toFixed(0)),
      kineticEnergyJ: Number(kineticJ.toFixed(4)),
      elasticEnergyJ: 0,
      plasticEnergyJ: 0,
      fractureEnergyJ: 0,
      maxPlasticStrainPct: Number(frameMaxPlasticPct.toFixed(2)),
      maxDamage: Number(frameMaxDamage.toFixed(3)),
      maxDisplacementMm: Number(maxDispMm.toFixed(2)),
      activeShardsCount: 0,
      totalShardsCount: 0,
      fracturedPartsCount: 0,
      intactPartsCount: visibleParts.length,
      statusLabel:
        physicsSim.collisions.length > 0
          ? 'Физика MCP (контакт и динамика)'
          : 'Физика MCP (кинематика f64)',
    };

    frames.push({
      frameIndex: i,
      timeSec,
      partStates,
      shardPos: new Float32Array(0),
      shardQuat: new Float32Array(0),
      shardActive: new Uint8Array(0),
      shardStress: new Float32Array(0),
      sparks: new Float32Array(0),
      metrics,
    });
  }

  const endMs = typeof performance !== 'undefined' ? performance.now() : Date.now();

  return {
    id: `live_phys_${Date.now().toString(36)}`,
    createdAt: new Date().toISOString(),
    solverBackend: 'rust_fem_f64',
    solverLabel: 'MCP Live Physics (f64)',
    config: { ...config, durationSec, fps },
    fps,
    totalFrames: frames.length - 1,
    durationSec,
    floorY,
    contactPoint: [0, 0, 0],
    bakeTimeMs: Math.max(1, Number((endMs - startMs).toFixed(1))),
    cacheBytes: frames.length * 512,
    shards: [],
    frames,
    events,
    maxStressOverallMPa: Math.max(1, Number(maxStressMPa.toFixed(1))),
    yieldStrengthRefMPa: Number(yieldRefMPa.toFixed(0)),
    maxPlasticStrainOverallPct: 0.5,
    maxDamageOverall: 0.01,
    maxEnergyOverallJ: Number(maxEnergyJ.toFixed(4)),
  };
}
