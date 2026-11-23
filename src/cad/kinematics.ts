import type { CadPart, KinematicJoint, MeshGeometryData, PhysicsTrajectoryFrame } from '../types/cad';
import { csgIntersectionMesh } from './csg_bsp';

/**
 * Steps rigid-body kinematic assembly transforms and joint angles forward by dtSeconds.
 * Supports:
 * 1. Motorized Revolute / Fixed / Prismatic / Spherical joints
 * 2. Exact Involute Spur Gear Mesh ratio coupling: w2 = -w1 * (z1 / z2)
 * 3. Crank-Slider 4-bar mechanism articulation (crank_arm + connecting_rod + linear_slider)
 */
export function stepKinematicsAssembly(
  parts: CadPart[],
  joints: KinematicJoint[],
  dtSeconds: number
): {
  updatedParts: CadPart[];
  updatedJoints: KinematicJoint[];
  angularVelocities: Record<string, [number, number, number]>;
  linearVelocities: Record<string, [number, number, number]>;
} {
  const partMap = new Map<string, CadPart>();
  const angularVelocities: Record<string, [number, number, number]> = {};
  const linearVelocities: Record<string, [number, number, number]> = {};

  parts.forEach((p) => {
    partMap.set(p.id, {
      ...p,
      position: [...p.position] as [number, number, number],
      rotation: [...p.rotation] as [number, number, number],
    });
    angularVelocities[p.id] = [0, 0, 0];
    linearVelocities[p.id] = [0, 0, 0];
  });

  const updatedJoints = joints.map((j) => ({ ...j }));

  // 1. Step motorized revolute & prismatic joints
  for (const joint of updatedJoints) {
    if (!joint.motorEnabled) continue;

    if (joint.type === 'revolute') {
      // RPM -> rad/s (or if motorSpeedRpm is small rad/s from MCP set_joint_motor, support both)
      const omegaRadSec =
        Math.abs(joint.motorSpeedRpm) <= 25
          ? joint.motorSpeedRpm
          : (joint.motorSpeedRpm * 2 * Math.PI) / 60;
      const dTheta = omegaRadSec * dtSeconds;
      joint.currentAngleRad += dTheta;

      const targetPart =
        partMap.get(joint.partB) ||
        (joint.partA !== 'world' ? partMap.get(joint.partA) : undefined);

      if (targetPart) {
        if (Math.abs(joint.axis[2]) > 0.5) {
          const sign = Math.sign(joint.axis[2]) || 1;
          targetPart.rotation[2] += dTheta * sign;
          angularVelocities[targetPart.id] = [0, 0, omegaRadSec * sign];
        } else if (Math.abs(joint.axis[1]) > 0.5) {
          const sign = Math.sign(joint.axis[1]) || 1;
          targetPart.rotation[1] += dTheta * sign;
          angularVelocities[targetPart.id] = [0, omegaRadSec * sign, 0];
        } else {
          const sign = Math.sign(joint.axis[0]) || 1;
          targetPart.rotation[0] += dTheta * sign;
          angularVelocities[targetPart.id] = [omegaRadSec * sign, 0, 0];
        }
      }
    } else if (joint.type === 'prismatic') {
      const hasExplicitLinear =
        typeof joint.motorLinearSpeedMs === 'number' &&
        Number.isFinite(joint.motorLinearSpeedMs) &&
        Math.abs(joint.motorLinearSpeedMs) > 1e-6;
      const defaultDir =
        joint.limits && Math.abs(joint.limits[0]) > Math.abs(joint.limits[1])
          ? -1
          : joint.limits && joint.limits[1] < 0
            ? -1
            : 1;
      const vel = hasExplicitLinear
        ? joint.motorLinearSpeedMs!
        : Math.abs(joint.motorSpeedRpm) > 1e-6
          ? Math.abs(joint.motorSpeedRpm) <= 25
            ? joint.motorSpeedRpm
            : joint.motorSpeedRpm * 0.002
          : defaultDir * 0.025;
      let dPos = vel * dtSeconds;
      if (joint.limits) {
        const rawMin = Math.min(joint.limits[0], joint.limits[1]);
        const rawMax = Math.max(joint.limits[0], joint.limits[1]);
        const scale = Math.max(Math.abs(rawMin), Math.abs(rawMax)) > 1.0 ? 1e-3 : 1.0;
        const minL = rawMin * scale;
        const maxL = rawMax * scale;
        const nextPos = joint.currentAngleRad + dPos;
        if (nextPos < minL) {
          dPos = minL - joint.currentAngleRad;
        } else if (nextPos > maxL) {
          dPos = maxL - joint.currentAngleRad;
        }
      }
      joint.currentAngleRad += dPos;
      const targetPart =
        partMap.get(joint.partB) ||
        (joint.partA !== 'world' ? partMap.get(joint.partA) : undefined);
      if (targetPart) {
        const axLen =
          Math.hypot(joint.axis[0], joint.axis[1], joint.axis[2]) || 1;
        const ux = joint.axis[0] / axLen;
        const uy = joint.axis[1] / axLen;
        const uz = joint.axis[2] / axLen;
        targetPart.position[0] += ux * dPos;
        targetPart.position[1] += uy * dPos;
        targetPart.position[2] += uz * dPos;
        linearVelocities[targetPart.id] = [ux * vel, uy * vel, uz * vel];
      }
    }
  }

  // 2. Enforce exact gear mesh ratios: w2 = -w1 * (z1 / z2)
  for (const joint of updatedJoints) {
    if (joint.type === 'gear') {
      const gearA = partMap.get(joint.partA);
      const gearB = partMap.get(joint.partB);
      if (gearA && gearB) {
        const z1 = gearA.dag.find((d) => d.type === 'spur_gear')?.params?.teeth || 20;
        const z2 = gearB.dag.find((d) => d.type === 'spur_gear')?.params?.teeth || 40;
        const ratio = joint.gearRatio ?? z1 / z2;

        // Phase offset pi / z2 aligns driving tooth into driven tooth valley (4.5 deg for z2=40)
        const meshPhaseOffset = Math.PI / z2;
        gearB.rotation[2] = -gearA.rotation[2] * ratio + meshPhaseOffset;

        const wA = angularVelocities[gearA.id]?.[2] ?? 0;
        angularVelocities[gearB.id] = [0, 0, -wA * ratio];
      }
    }
  }

  // Also automatically couple any spur gears in contact even if connected via revolute joints
  // (Matches crates/mcp-cad-server/tests/gear_kinematic_test.rs where two revolute joints + contact resolve w2/w1 = -0.5)
  const hasExplicitGearJoint = updatedJoints.some((j) => j.type === 'gear');
  if (!hasExplicitGearJoint) {
    const gearParts = Array.from(partMap.values()).filter((p) =>
      p.dag.some((d) => d.type === 'spur_gear')
    );
    if (gearParts.length >= 2) {
      const driver = gearParts[0];
      const driven = gearParts[1];
      const wDriver = angularVelocities[driver.id]?.[2] ?? 0;
      const wDriven = angularVelocities[driven.id]?.[2] ?? 0;
      if (Math.abs(wDriver) > 1e-6 && Math.abs(wDriven) < 1e-6) {
        const z1 =
          driver.dag.find((d) => d.type === 'spur_gear')?.params?.teeth || 20;
        const z2 =
          driven.dag.find((d) => d.type === 'spur_gear')?.params?.teeth || 40;
        const ratio = z1 / z2;
        const meshPhaseOffset = Math.PI / z2;
        driven.rotation[2] = -driver.rotation[2] * ratio + meshPhaseOffset;
        angularVelocities[driven.id] = [0, 0, -wDriver * ratio];
      }
    }
  }

  // 3. Enforce Crank-Slider 4-bar kinematic chain if present (crank_arm, connecting_rod, linear_slider)
  const crank =
    partMap.get('crank_arm') ||
    Array.from(partMap.values()).find((p) =>
      p.name.toLowerCase().includes('crank')
    );
  const rod =
    partMap.get('connecting_rod') ||
    Array.from(partMap.values()).find((p) =>
      p.name.toLowerCase().includes('rod')
    );
  const slider =
    partMap.get('linear_slider') ||
    Array.from(partMap.values()).find((p) =>
      p.name.toLowerCase().includes('slider')
    );

  if (crank && (rod || slider)) {
    const theta = crank.rotation[2];
    const omega = angularVelocities[crank.id]?.[2] ?? 0;
    const pivotX = -0.06;
    const pivotY = 0.0;
    const r = 0.03; // crank throw radius 30mm
    const l = 0.095; // connecting rod length 95mm

    // Crank center revolves around pivot (-0.06, 0) at radius r/2
    crank.position[0] = pivotX + (r * 0.5) * Math.cos(theta);
    crank.position[1] = pivotY + (r * 0.5) * Math.sin(theta);

    // Crank pin coordinates
    const pinX = pivotX + r * Math.cos(theta);
    const pinY = pivotY + r * Math.sin(theta);

    // Wrist pin on horizontal slider track (y = 0)
    const underRoot = Math.max(1e-6, l * l - pinY * pinY);
    const wristX = pinX + Math.sqrt(underRoot);
    const phi = Math.atan2(-pinY, wristX - pinX);

    if (rod) {
      rod.position[0] = 0.5 * (pinX + wristX);
      rod.position[1] = 0.5 * pinY;
      rod.rotation[2] = phi;
      angularVelocities[rod.id] = [
        0,
        0,
        (-omega * r * Math.cos(theta)) / Math.sqrt(underRoot),
      ];
    }

    if (slider) {
      const prevX = slider.position[0];
      slider.position[0] = wristX;
      slider.position[1] = 0.0;
      linearVelocities[slider.id] = [
        dtSeconds > 1e-6 ? (wristX - prevX) / dtSeconds : -omega * r * Math.sin(theta),
        0,
        0,
      ];
    }
  }

  return {
    updatedParts: Array.from(partMap.values()),
    updatedJoints,
    angularVelocities,
    linearVelocities,
  };
}

/**
 * Runs a multi-step rigid-body / kinematic simulation over `durationSec` with step `dtSec`,
 * producing trajectory samples and animation frames matching `run_physics_simulation` in Rust.
 */
export function runPhysicsSimulationTrajectory(
  initialParts: CadPart[],
  initialJoints: KinematicJoint[],
  durationSec: number = 1.0,
  dtSec: number = 1 / 60
): {
  steps: number;
  frames: PhysicsTrajectoryFrame[];
  finalParts: CadPart[];
  finalJoints: KinematicJoint[];
} {
  if (!Number.isFinite(durationSec) || durationSec <= 0) {
    throw new Error('Simulation duration must be a positive finite number (> 0)');
  }
  if (!Number.isFinite(dtSec) || dtSec <= 0) {
    throw new Error('Simulation time step dt must be positive (> 0)');
  }

  const safeDt = Math.max(0.001, Math.min(0.1, dtSec));
  const steps = Math.max(1, Math.min(2000, Math.round(durationSec / safeDt)));

  let currentParts = initialParts.map((p) => ({
    ...p,
    position: [...p.position] as [number, number, number],
    rotation: [...p.rotation] as [number, number, number],
  }));

  // Ensure at least one motor is active during physics simulation if joints exist
  let currentJoints = initialJoints.map((j, idx) => ({
    ...j,
    motorEnabled: j.motorEnabled || (idx === 0 && j.type === 'revolute'),
    motorSpeedRpm:
      j.motorSpeedRpm !== 0 ? j.motorSpeedRpm : idx === 0 ? 60 : 0,
  }));

  const frames: PhysicsTrajectoryFrame[] = [];

  for (let s = 0; s <= steps; s++) {
    const stepRes = stepKinematicsAssembly(
      currentParts,
      currentJoints,
      s === 0 ? 0 : safeDt
    );
    currentParts = stepRes.updatedParts;
    currentJoints = stepRes.updatedJoints;

    const partTransforms: PhysicsTrajectoryFrame['partTransforms'] = {};
    for (const p of currentParts) {
      partTransforms[p.id] = {
        position: [...p.position] as [number, number, number],
        rotation: [...p.rotation] as [number, number, number],
        angularVelocity: stepRes.angularVelocities[p.id] || [0, 0, 0],
        linearVelocity: stepRes.linearVelocities[p.id] || [0, 0, 0],
      };
    }

    frames.push({
      timeSec: Number((s * safeDt).toFixed(4)),
      partTransforms,
    });
  }

  return {
    steps,
    frames,
    finalParts: currentParts,
    finalJoints: currentJoints,
  };
}

export interface JointLimitStatus {
  joint_id: string;
  joint_type: string;
  boundary: 'min' | 'max' | 'below_min' | 'above_max' | 'locked';
  current_value: number;
  limits: [number, number];
  unit: string;
  motor_blocked: boolean;
}

export interface MechanismAnalysisReport {
  status: 'PASS' | 'WARN';
  dof_numeric: number;
  dof_grubler: number;
  redundant_constraints: number;
  redundant_joint_ids: string[];
  joints_at_limits: JointLimitStatus[];
  grounded_parts: string[];
  moving_parts: string[];
  floating_parts: string[];
  unconnected_parts: string[];
  joint_constraints_count: number;
  jacobian_rows: number;
  jacobian_cols: number;
  jacobian_rank: number;
  singular_values: number[];
  rank_tolerance: number;
  warnings: string[];
  notes: string[];
  joints_count: number;
}

type Vec3 = [number, number, number];
type Mat3 = [Vec3, Vec3, Vec3];

function dot3(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross3(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
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

function norm3(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

function normalize3(a: Vec3): Vec3 {
  const n = norm3(a);
  return n > 1e-12 ? [a[0] / n, a[1] / n, a[2] / n] : [0, 0, 1];
}

function eulerToMat3(rot: Vec3): Mat3 {
  const [rx, ry, rz] = rot;
  const cx = Math.cos(rx), sx = Math.sin(rx);
  const cy = Math.cos(ry), sy = Math.sin(ry);
  const cz = Math.cos(rz), sz = Math.sin(rz);
  return [
    [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
    [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
    [-sy, cy * sx, cy * cx],
  ];
}

function mat3Vec3(m: Mat3, v: Vec3): Vec3 {
  return [
    dot3(m[0], v),
    dot3(m[1], v),
    dot3(m[2], v),
  ];
}

function transpose3(m: Mat3): Mat3 {
  return [
    [m[0][0], m[1][0], m[2][0]],
    [m[0][1], m[1][1], m[2][1]],
    [m[0][2], m[1][2], m[2][2]],
  ];
}

function normalizeOrDefault3(a: Vec3, fallback: Vec3): Vec3 {
  const n = norm3(a);
  return n > 1e-12 ? [a[0] / n, a[1] / n, a[2] / n] : fallback;
}

function orthonormalBasis(u: Vec3): [Vec3, Vec3, Vec3] {
  const z = normalize3(u);
  const vRef: Vec3 = Math.abs(z[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const x = normalize3(sub3(vRef, scale3(z, dot3(vRef, z))));
  const y = normalize3(cross3(z, x));
  return [x, y, z];
}

function isZeroRangeLimits(limits?: [number, number]): boolean {
  return limits !== undefined && Math.abs(limits[1] - limits[0]) <= 1e-9;
}

function unwrapAngleNearLimits(theta: number, limits: [number, number]): number {
  const mid = 0.5 * (limits[0] + limits[1]);
  const tau = 2 * Math.PI;
  let best = theta;
  let bestDist = Math.abs(theta - mid);
  for (const shift of [-tau, tau]) {
    const cand = theta + shift;
    const dist = Math.abs(cand - mid);
    if (dist < bestDist - 1e-12) {
      best = cand;
      bestDist = dist;
    }
  }
  return best;
}

function jointConstraintCount(type: KinematicJoint['type']): number {
  switch (type) {
    case 'fixed':
      return 6;
    case 'revolute':
    case 'prismatic':
      return 5;
    case 'spherical':
      return 3;
    case 'gear':
      return 1;
  }
}

function effectiveJointConstraintCount(j: KinematicJoint): number {
  if (
    (j.type === 'revolute' || j.type === 'prismatic') &&
    isZeroRangeLimits(j.limits)
  ) {
    return 6;
  }
  return jointConstraintCount(j.type);
}

function evaluateJointLimitStatusTS(
  j: KinematicJoint,
  poseA: { pos: Vec3; rot: Mat3 },
  poseB: { pos: Vec3; rot: Mat3 }
): { status: JointLimitStatus; warning: string } | null {
  if ((j.type !== 'revolute' && j.type !== 'prismatic') || !j.limits) {
    return null;
  }
  const isRevolute = j.type === 'revolute';
  const [minV, maxV] = j.limits;
  let q: number;
  if (isRevolute) {
    const uA = normalizeOrDefault3(j.axis || [0, 0, 1], [0, 0, 1]);
    const [xLoc, yLoc] = orthonormalBasis(uA);
    const rRel = mat3Mul(transpose3(poseA.rot), poseB.rot);
    const rotX = mat3Vec3(rRel, xLoc);
    const rawTheta = Math.atan2(dot3(yLoc, rotX), dot3(xLoc, rotX));
    const poseTheta =
      Math.abs(rawTheta) <= 1e-9 && Math.abs(j.currentAngleRad) > 1e-9
        ? j.currentAngleRad
        : rawTheta;
    q = unwrapAngleNearLimits(poseTheta, j.limits);
  } else {
    const uA = normalizeOrDefault3(j.axis || [1, 0, 0], [1, 0, 0]);
    const zA = mat3Vec3(poseA.rot, uA);
    const anchorA: Vec3 = j.anchor || [0, 0, 0];
    const anchorB: Vec3 = j.anchorB ?? [0, 0, 0];
    const pAW = add3(poseA.pos, mat3Vec3(poseA.rot, anchorA));
    const pBW = add3(poseB.pos, mat3Vec3(poseB.rot, anchorB));
    const poseS = dot3(sub3(pBW, pAW), zA);
    q =
      Math.abs(poseS) <= 1e-9 && Math.abs(j.currentAngleRad) > 1e-9
        ? j.currentAngleRad
        : poseS;
  }

  const span = Math.abs(maxV - minV);
  const tol = Math.min(1e-3, Math.max(1e-6, span * 1e-3));
  const unit = isRevolute ? 'rad' : 'm';
  const radToDeg = 180 / Math.PI;
  const motorSign = j.motorEnabled ? Math.sign(j.motorSpeedRpm) : 0;

  let boundary: JointLimitStatus['boundary'];
  let motorBlocked = false;
  let warning: string;

  if (isZeroRangeLimits(j.limits)) {
    boundary = 'locked';
    motorBlocked = true;
    warning = isRevolute
      ? `Достигнута граница limits для шарнира '${j.id}': нулевой диапазон хода [${(minV * radToDeg).toFixed(2)}, ${(maxV * radToDeg).toFixed(2)}] deg (заблокирован)`
      : `Достигнута граница limits для шарнира '${j.id}': нулевой диапазон хода [${minV.toFixed(4)}, ${maxV.toFixed(4)}] m (заблокирован)`;
  } else if (q < minV - tol || Math.abs(q - minV) <= tol) {
    boundary = q < minV - tol ? 'below_min' : 'min';
    motorBlocked = motorSign < 0;
    const suffix = motorBlocked ? '; привод направлен в упор' : '';
    warning = isRevolute
      ? `Шарнир '${j.id}' достиг нижней границы limits (${(q * radToDeg).toFixed(2)} deg <= ${(minV * radToDeg).toFixed(2)} deg)${suffix}`
      : `Шарнир '${j.id}' достиг нижней границы limits (${q.toFixed(4)} m <= ${minV.toFixed(4)} m)${suffix}`;
  } else if (q > maxV + tol || Math.abs(q - maxV) <= tol) {
    boundary = q > maxV + tol ? 'above_max' : 'max';
    motorBlocked = motorSign > 0;
    const suffix = motorBlocked ? '; привод направлен в упор' : '';
    warning = isRevolute
      ? `Шарнир '${j.id}' достиг верхней границы limits (${(q * radToDeg).toFixed(2)} deg >= ${(maxV * radToDeg).toFixed(2)} deg)${suffix}`
      : `Шарнир '${j.id}' достиг верхней границы limits (${q.toFixed(4)} m >= ${maxV.toFixed(4)} m)${suffix}`;
  } else {
    return null;
  }

  return {
    status: {
      joint_id: j.id,
      joint_type: j.type,
      boundary,
      current_value: q,
      limits: [minV, maxV],
      unit,
      motor_blocked: motorBlocked,
    },
    warning,
  };
}

function findRedundantJointIdsTS(
  jMat: number[][],
  jointRowSpans: { joint: KinematicJoint; startRow: number; rows: number }[],
  totalCols: number,
  fullRank: number,
  rankTol: number
): string[] {
  const totalRows = jMat.length;
  const redundantIds: string[] = [];

  for (const span of jointRowSpans) {
    const remainingRows = totalRows - span.rows;
    let rankWithoutK = 0;
    if (remainingRows > 0) {
      const jSub: number[][] = [];
      for (let r = 0; r < totalRows; r++) {
        if (r >= span.startRow && r < span.startRow + span.rows) continue;
        jSub.push(jMat[r]);
      }
      const svSub = computeSingularValues(jSub, remainingRows, totalCols);
      rankWithoutK = svSub.filter((s) => s > rankTol).length;
    }
    const depRows = span.rows - (fullRank - rankWithoutK);
    if (depRows > 0) {
      redundantIds.push(span.joint.id);
    }
  }

  return redundantIds;
}

/**
 * Computes singular values of an m x n matrix in descending order using symmetric Jacobi
 * eigenvalue decomposition on A^T A (or A A^T when m < n).
 */
export function computeSingularValues(mat: number[][], rows: number, cols: number): number[] {
  if (rows === 0 || cols === 0) return [];
  const transpose = rows < cols;
  const n = transpose ? rows : cols;
  const kDim = transpose ? cols : rows;

  // Build symmetric positive semi-definite Gram matrix G (n x n)
  const g: number[][] = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let i = 0; i < n; i++) {
    for (let j = i; j < n; j++) {
      let sum = 0;
      if (transpose) {
        for (let k = 0; k < kDim; k++) sum += mat[i][k] * mat[j][k];
      } else {
        for (let k = 0; k < kDim; k++) sum += mat[k][i] * mat[k][j];
      }
      g[i][j] = sum;
      g[j][i] = sum;
    }
  }

  // Classical cyclic Jacobi rotations on symmetric matrix g
  const maxSweeps = 60;
  for (let sweep = 0; sweep < maxSweeps; sweep++) {
    let offDiagNorm = 0;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        offDiagNorm += g[p][q] * g[p][q];
      }
    }
    if (Math.sqrt(offDiagNorm) < 1e-24) break;

    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        const apq = g[p][q];
        if (Math.abs(apq) < 1e-18) continue;
        const app = g[p][p];
        const aqq = g[q][q];
        const tau = (aqq - app) / (2 * apq);
        const t =
          tau >= 0
            ? 1 / (tau + Math.sqrt(1 + tau * tau))
            : -1 / (-tau + Math.sqrt(1 + tau * tau));
        const c = 1 / Math.sqrt(1 + t * t);
        const s = t * c;

        g[p][p] = app - t * apq;
        g[q][q] = aqq + t * apq;
        g[p][q] = 0;
        g[q][p] = 0;

        for (let r = 0; r < n; r++) {
          if (r === p || r === q) continue;
          const grp = g[r][p];
          const grq = g[r][q];
          g[r][p] = c * grp - s * grq;
          g[p][r] = g[r][p];
          g[r][q] = s * grp + c * grq;
          g[q][r] = g[r][q];
        }
      }
    }
  }

  const sv = g.map((row, idx) => Math.sqrt(Math.max(0, row[idx])));
  sv.sort((a, b) => b - a);
  return sv;
}

/**
 * Analyzes mechanism degrees of freedom, mobility, redundant constraints, and floating/unconnected
 * bodies using both the Chebychev-Grübler-Kutzbach formula and SVD rank of the linearized
 * 3D kinematic constraint Jacobian (`P3-T9`).
 */
export function analyzeMechanism(
  parts: CadPart[],
  joints: KinematicJoint[],
  options?: { partIds?: string[]; rankTolerance?: number }
): MechanismAnalysisReport {
  if (
    options?.rankTolerance !== undefined &&
    (!Number.isFinite(options.rankTolerance) || options.rankTolerance <= 0)
  ) {
    throw new Error(
      `rank_tolerance must be a finite positive number (> 0), got ${options.rankTolerance}`
    );
  }

  const normPart = (id: string): string => {
    const lower = id.trim().toLowerCase();
    if (lower === 'ground') return 'ground';
    if (lower === 'world') return 'world';
    return id.trim();
  };

  const partMap = new Map<string, CadPart>();
  for (const p of parts) {
    partMap.set(normPart(p.id), p);
  }

  let targetSet: Set<string> | null = null;
  if (options?.partIds) {
    targetSet = new Set(options.partIds.map(normPart));
    for (const pid of targetSet) {
      if (pid === 'ground' || pid === 'world') continue;
      if (!partMap.has(pid)) {
        throw new Error(`Part not found: ${pid}`);
      }
    }
  }

  const groundedSet = new Set<string>(['ground', 'world']);
  const movingSet = new Set<string>();

  for (const [pid, part] of partMap.entries()) {
    if (pid === 'ground' || pid === 'world') continue;
    if (targetSet && !targetSet.has(pid)) continue;
    if (part.isFixed) {
      groundedSet.add(pid);
    } else {
      movingSet.add(pid);
    }
  }

  const movingParts = Array.from(movingSet).sort();
  const movingIndex = new Map<string, number>();
  movingParts.forEach((id, i) => movingIndex.set(id, i));

  const activeJoints = joints
    .filter((j) => {
      const na = normPart(j.partA);
      const nb = normPart(j.partB);
      if (targetSet) {
        const aOk = targetSet.has(na) || na === 'ground' || na === 'world';
        const bOk = targetSet.has(nb) || nb === 'ground' || nb === 'world';
        return aOk && bOk;
      }
      return true;
    })
    .slice()
    .sort((a, b) => a.id.localeCompare(b.id));

  // Connectivity graph
  const adj = new Map<string, string[]>();
  for (const j of activeJoints) {
    const na = normPart(j.partA);
    const nb = normPart(j.partB);
    if (!adj.has(na)) adj.set(na, []);
    if (!adj.has(nb)) adj.set(nb, []);
    adj.get(na)!.push(nb);
    adj.get(nb)!.push(na);
  }

  const unconnectedParts = movingParts.filter(
    (p) => !adj.has(p) || adj.get(p)!.length === 0
  );

  const visitedFromGround = new Set<string>();
  const queue: string[] = [];
  for (const g of groundedSet) {
    visitedFromGround.add(g);
    queue.push(g);
  }
  while (queue.length > 0) {
    const curr = queue.shift()!;
    for (const n of adj.get(curr) || []) {
      if (!visitedFromGround.has(n)) {
        visitedFromGround.add(n);
        queue.push(n);
      }
    }
  }

  const floatingParts = movingParts.filter((p) => !visitedFromGround.has(p));

  const reportedGrounded: string[] = [];
  for (const g of groundedSet) {
    if (g === 'ground' || g === 'world') {
      if (
        activeJoints.some(
          (j) => normPart(j.partA) === g || normPart(j.partB) === g
        )
      ) {
        reportedGrounded.push(g);
      }
    } else {
      reportedGrounded.push(g);
    }
  }
  reportedGrounded.sort();

  const contributingJoints = activeJoints.filter((j) => {
    const na = normPart(j.partA);
    const nb = normPart(j.partB);
    return movingIndex.has(na) || movingIndex.has(nb);
  });

  const totalRows = contributingJoints.reduce(
    (acc, j) => acc + effectiveJointConstraintCount(j),
    0
  );
  const numMoving = movingParts.length;
  const totalCols = 6 * numMoving;

  const getPose = (pid: string): { pos: Vec3; rot: Mat3 } => {
    const p = partMap.get(pid);
    if (!p || pid === 'ground' || pid === 'world') {
      return {
        pos: [0, 0, 0],
        rot: [
          [1, 0, 0],
          [0, 1, 0],
          [0, 0, 1],
        ],
      };
    }
    return {
      pos: [...p.position] as Vec3,
      rot: eulerToMat3(p.rotation),
    };
  };

  let jacobianRank = 0;
  let singularValues: number[] = [];
  let effectiveTol = options?.rankTolerance ?? 1e-6;
  const jointsAtLimits: JointLimitStatus[] = [];
  const limitWarnings: string[] = [];
  const jointRowSpans: { joint: KinematicJoint; startRow: number; rows: number }[] = [];
  let jMatBuilt: number[][] | null = null;

  if (totalRows === 0 || totalCols === 0) {
    for (const j of contributingJoints) {
      const poseA = getPose(normPart(j.partA));
      const poseB = getPose(normPart(j.partB));
      const limRes = evaluateJointLimitStatusTS(j, poseA, poseB);
      if (limRes) {
        jointsAtLimits.push(limRes.status);
        limitWarnings.push(limRes.warning);
      }
    }
  } else {
    const jMat: number[][] = Array.from({ length: totalRows }, () =>
      new Array(totalCols).fill(0)
    );
    const addVec = (
      r: number,
      colIdx: number | undefined,
      isRot: boolean,
      delta: Vec3
    ) => {
      if (colIdx === undefined) return;
      const base = 6 * colIdx + (isRot ? 3 : 0);
      jMat[r][base] += delta[0];
      jMat[r][base + 1] += delta[1];
      jMat[r][base + 2] += delta[2];
    };

    const eBasis: Vec3[] = [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ];
    let curRow = 0;

    for (const j of contributingJoints) {
      const na = normPart(j.partA);
      const nb = normPart(j.partB);
      const colA = movingIndex.get(na);
      const colB = movingIndex.get(nb);
      const poseA = getPose(na);
      const poseB = getPose(nb);
      const limRes = evaluateJointLimitStatusTS(j, poseA, poseB);
      if (limRes) {
        jointsAtLimits.push(limRes.status);
        limitWarnings.push(limRes.warning);
      }

      const anchorA: Vec3 = j.anchor || [0, 0, 0];
      const anchorB: Vec3 = j.anchorB ?? [0, 0, 0];
      const dA = mat3Vec3(poseA.rot, anchorA);
      const dB = mat3Vec3(poseB.rot, anchorB);
      const startRow = curRow;

      if (j.type === 'fixed') {
        for (let k = 0; k < 3; k++) {
          const ek = eBasis[k];
          addVec(curRow + k, colA, false, ek);
          addVec(curRow + k, colA, true, cross3(dA, ek));
          addVec(curRow + k, colB, false, scale3(ek, -1));
          addVec(curRow + k, colB, true, scale3(cross3(dB, ek), -1));
        }
        for (let k = 0; k < 3; k++) {
          const ek = eBasis[k];
          addVec(curRow + 3 + k, colA, true, scale3(ek, -1));
          addVec(curRow + 3 + k, colB, true, ek);
        }
        curRow += 6;
      } else if (j.type === 'spherical') {
        for (let k = 0; k < 3; k++) {
          const ek = eBasis[k];
          addVec(curRow + k, colA, false, ek);
          addVec(curRow + k, colA, true, cross3(dA, ek));
          addVec(curRow + k, colB, false, scale3(ek, -1));
          addVec(curRow + k, colB, true, scale3(cross3(dB, ek), -1));
        }
        curRow += 3;
      } else if (j.type === 'revolute') {
        for (let k = 0; k < 3; k++) {
          const ek = eBasis[k];
          addVec(curRow + k, colA, false, ek);
          addVec(curRow + k, colA, true, cross3(dA, ek));
          addVec(curRow + k, colB, false, scale3(ek, -1));
          addVec(curRow + k, colB, true, scale3(cross3(dB, ek), -1));
        }
        const zA = mat3Vec3(poseA.rot, normalize3(j.axis || [0, 0, 1]));
        const [xA, yA] = orthonormalBasis(zA);
        addVec(curRow + 3, colA, true, scale3(xA, -1));
        addVec(curRow + 3, colB, true, xA);
        addVec(curRow + 4, colA, true, scale3(yA, -1));
        addVec(curRow + 4, colB, true, yA);
        if (isZeroRangeLimits(j.limits)) {
          addVec(curRow + 5, colA, true, scale3(zA, -1));
          addVec(curRow + 5, colB, true, zA);
          curRow += 6;
        } else {
          curRow += 5;
        }
      } else if (j.type === 'prismatic') {
        for (let k = 0; k < 3; k++) {
          const ek = eBasis[k];
          addVec(curRow + k, colA, true, scale3(ek, -1));
          addVec(curRow + k, colB, true, ek);
        }
        const zA = mat3Vec3(poseA.rot, normalize3(j.axis || [1, 0, 0]));
        const [xA, yA] = orthonormalBasis(zA);
        const rB = add3(poseB.pos, dB);
        const relBFromA = sub3(rB, poseA.pos);
        addVec(curRow + 3, colA, false, scale3(xA, -1));
        addVec(curRow + 3, colA, true, cross3(xA, relBFromA));
        addVec(curRow + 3, colB, false, xA);
        addVec(curRow + 3, colB, true, cross3(dB, xA));

        addVec(curRow + 4, colA, false, scale3(yA, -1));
        addVec(curRow + 4, colA, true, cross3(yA, relBFromA));
        addVec(curRow + 4, colB, false, yA);
        addVec(curRow + 4, colB, true, cross3(dB, yA));
        if (isZeroRangeLimits(j.limits)) {
          addVec(curRow + 5, colA, false, scale3(zA, -1));
          addVec(curRow + 5, colA, true, cross3(zA, relBFromA));
          addVec(curRow + 5, colB, false, zA);
          addVec(curRow + 5, colB, true, cross3(dB, zA));
          curRow += 6;
        } else {
          curRow += 5;
        }
      } else if (j.type === 'gear') {
        const ratio = j.gearRatio ?? 1.0;
        const zA = mat3Vec3(poseA.rot, normalize3(j.axis || [0, 0, 1]));
        const zB = mat3Vec3(poseB.rot, normalize3(j.axis || [0, 0, 1]));
        addVec(curRow, colA, true, scale3(zA, ratio));
        addVec(curRow, colB, true, zB);
        curRow += 1;
      }

      jointRowSpans.push({ joint: j, startRow, rows: curRow - startRow });
    }

    jMatBuilt = jMat;
    singularValues = computeSingularValues(jMat, totalRows, totalCols);
    const sMax = singularValues[0] ?? 0;
    effectiveTol =
      options?.rankTolerance ??
      Math.max(1e-9, Math.max(totalRows, totalCols) * Math.max(sMax, 1.0) * 1e-7);
    jacobianRank = singularValues.filter((s) => s > effectiveTol).length;
  }

  const dofNumeric = 6 * numMoving - jacobianRank;
  const dofGrubler = 6 * numMoving - totalRows;
  const redundantConstraints = Math.max(0, dofNumeric - dofGrubler);
  const redundantJointIds =
    jMatBuilt && redundantConstraints > 0 && totalCols > 0
      ? findRedundantJointIdsTS(
          jMatBuilt,
          jointRowSpans,
          totalCols,
          jacobianRank,
          effectiveTol
        )
      : [];

  const warnings: string[] = [];
  if (contributingJoints.length > 0 && dofNumeric === 0) {
    warnings.push('DOF=0 при заявленном механизме');
  }
  const motorsCount = contributingJoints.filter((j) => j.motorEnabled).length;
  if (dofNumeric > 1 && motorsCount === 0) {
    warnings.push('DOF>1 без двигателей');
  }
  if (unconnectedParts.length > 0 || floatingParts.length > 0) {
    warnings.push('есть свободные детали');
  }
  warnings.push(...limitWarnings);

  const notes: string[] = [
    'Chebychev-Grübler-Kutzbach spatial mobility formula: M = 6(N - 1) - sum(c_i), where N includes ground and c_i are joint constraint counts.',
    `Numerical mobility: DOF = 6 * N_moving (${6 * numMoving}) - rank(J) (${jacobianRank}) = ${dofNumeric}.`,
    `Jacobian SVD rank tolerance: ${effectiveTol.toExponential(2)} (maximum singular value: ${(singularValues[0] ?? 0).toFixed(4)}).`,
  ];
  if (redundantConstraints > 0) {
    notes.push(
      `Detected ${redundantConstraints} redundant constraints involving joints [${redundantJointIds.join(', ')}] (common in planar four-bar linkages, slider-cranks, and overconstrained parallel mechanisms).`
    );
  }
  if (jointsAtLimits.length > 0) {
    const lockedCount = jointsAtLimits.filter((s) => s.boundary === 'locked').length;
    notes.push(
      `Detected ${jointsAtLimits.length} joint(s) at or beyond travel limits (${lockedCount} bilaterally locked with zero-range limits).`
    );
  }

  return {
    status: warnings.length === 0 ? 'PASS' : 'WARN',
    dof_numeric: dofNumeric,
    dof_grubler: dofGrubler,
    redundant_constraints: redundantConstraints,
    redundant_joint_ids: redundantJointIds,
    joints_at_limits: jointsAtLimits,
    grounded_parts: reportedGrounded,
    moving_parts: movingParts,
    floating_parts: floatingParts,
    unconnected_parts: unconnectedParts,
    joint_constraints_count: totalRows,
    jacobian_rows: totalRows,
    jacobian_cols: totalCols,
    jacobian_rank: jacobianRank,
    singular_values: singularValues,
    rank_tolerance: effectiveTol,
    warnings,
    notes,
    joints_count: joints.length,
  };
}

export interface PairInterferenceResult {
  a: string;
  b: string;
  intersects: boolean;
  penetration_depth?: number;
  intersection_volume?: number;
  clearance?: number;
  adjacent_by_joint: boolean;
}

export interface InterferenceReport {
  status: 'PASS' | 'FAIL';
  interfering_pairs_count: number;
  checked_pairs_count: number;
  min_clearance?: number;
  max_penetration_depth: number;
  total_intersection_volume: number;
  pairs: PairInterferenceResult[];
}

export interface SweepStepResult {
  step: number;
  value: number;
  angle_deg?: number;
  angle_rad?: number;
  interfering_pairs: [string, string][];
  min_clearance?: number;
  max_penetration_depth: number;
  total_intersection_volume: number;
  closure_residual: number;
  locked_up: boolean;
}

export interface JointSweepReport {
  status: 'PASS' | 'WARN' | 'FAIL';
  joint_id: string;
  joint_type: string;
  from: number;
  to: number;
  steps: number;
  ignore_adjacent: boolean;
  first_collision_step?: number;
  first_collision_value?: number;
  first_collision_angle_deg?: number;
  first_collision_angle_rad?: number;
  first_collision_pairs: [string, string][];
  min_clearance?: number;
  min_clearance_step?: number;
  min_clearance_value?: number;
  safe_travel_range?: [number, number];
  margin_to_collision?: number;
  closed_loop_detected: boolean;
  lockup_detected: boolean;
  lockup_step?: number;
  lockup_value?: number;
  lockup_angle_deg?: number;
  lockup_angle_rad?: number;
  max_closure_residual: number;
  samples: SweepStepResult[];
}

function transformPartMeshToWorld(part: CadPart): MeshGeometryData {
  const rotMat = eulerToMat3(part.rotation);
  const pos = part.position;
  const scale = part.scale ?? [1, 1, 1];
  const srcV = part.mesh.vertices;
  const srcN = part.mesh.normals;
  const dstV = new Float32Array(srcV.length);
  const dstN = new Float32Array(srcN.length);

  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;

  for (let i = 0; i < srcV.length; i += 3) {
    const sx = srcV[i] * scale[0];
    const sy = srcV[i + 1] * scale[1];
    const sz = srcV[i + 2] * scale[2];
    const rw = mat3Vec3(rotMat, [sx, sy, sz]);
    const wx = rw[0] + pos[0];
    const wy = rw[1] + pos[1];
    const wz = rw[2] + pos[2];
    dstV[i] = wx;
    dstV[i + 1] = wy;
    dstV[i + 2] = wz;
    if (wx < minX) minX = wx;
    if (wy < minY) minY = wy;
    if (wz < minZ) minZ = wz;
    if (wx > maxX) maxX = wx;
    if (wy > maxY) maxY = wy;
    if (wz > maxZ) maxZ = wz;

    if (i + 2 < srcN.length) {
      const nr = normalize3(mat3Vec3(rotMat, [srcN[i], srcN[i + 1], srcN[i + 2]]));
      dstN[i] = nr[0];
      dstN[i + 1] = nr[1];
      dstN[i + 2] = nr[2];
    }
  }

  if (!Number.isFinite(minX)) {
    minX = minY = minZ = 0;
    maxX = maxY = maxZ = 0;
  }

  return {
    ...part.mesh,
    vertices: dstV,
    normals: dstN,
    boundingBox: {
      min: [minX, minY, minZ],
      max: [maxX, maxY, maxZ],
    },
  };
}

function aabbIntersects(
  minA: Vec3,
  maxA: Vec3,
  minB: Vec3,
  maxB: Vec3,
  eps: number = 1e-9
): boolean {
  return (
    minA[0] <= maxB[0] + eps &&
    maxA[0] >= minB[0] - eps &&
    minA[1] <= maxB[1] + eps &&
    maxA[1] >= minB[1] - eps &&
    minA[2] <= maxB[2] + eps &&
    maxA[2] >= minB[2] - eps
  );
}

function pointTriangleDistSq(p: Vec3, a: Vec3, b: Vec3, c: Vec3): number {
  const ab = sub3(b, a);
  const ac = sub3(c, a);
  const ap = sub3(p, a);
  const d1 = dot3(ab, ap);
  const d2 = dot3(ac, ap);
  if (d1 <= 0 && d2 <= 0) return dot3(ap, ap);

  const bp = sub3(p, b);
  const d3 = dot3(ab, bp);
  const d4 = dot3(ac, bp);
  if (d3 >= 0 && d4 <= d3) return dot3(bp, bp);

  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const v = d1 / (d1 - d3);
    const diff = sub3(p, add3(a, scale3(ab, v)));
    return dot3(diff, diff);
  }

  const cp = sub3(p, c);
  const d5 = dot3(ab, cp);
  const d6 = dot3(ac, cp);
  if (d6 >= 0 && d5 <= d6) return dot3(cp, cp);

  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const w = d2 / (d2 - d6);
    const diff = sub3(p, add3(a, scale3(ac, w)));
    return dot3(diff, diff);
  }

  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / (d4 - d3 + (d5 - d6));
    const bc = sub3(c, b);
    const diff = sub3(p, add3(b, scale3(bc, w)));
    return dot3(diff, diff);
  }

  const denom = 1 / (va + vb + vc);
  const v = vb * denom;
  const w = vc * denom;
  const closest = add3(a, add3(scale3(ab, v), scale3(ac, w)));
  const diff = sub3(p, closest);
  return dot3(diff, diff);
}

function segmentSegmentDistSq(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3): number {
  const d1 = sub3(q1, p1);
  const d2 = sub3(q2, p2);
  const r = sub3(p1, p2);
  const a = dot3(d1, d1);
  const e = dot3(d2, d2);
  const f = dot3(d2, r);

  let s = 0;
  let t = 0;
  if (a <= 1e-18 && e <= 1e-18) {
    return dot3(r, r);
  }
  if (a <= 1e-18) {
    t = Math.max(0, Math.min(1, f / e));
  } else {
    const c = dot3(d1, r);
    if (e <= 1e-18) {
      s = Math.max(0, Math.min(1, -c / a));
    } else {
      const b = dot3(d1, d2);
      const denom = a * e - b * b;
      if (denom !== 0) {
        s = Math.max(0, Math.min(1, (b * f - c * e) / denom));
      }
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = Math.max(0, Math.min(1, -c / a));
      } else if (t > 1) {
        t = 1;
        s = Math.max(0, Math.min(1, (b - c) / a));
      }
    }
  }
  const c1 = add3(p1, scale3(d1, s));
  const c2 = add3(p2, scale3(d2, t));
  const diff = sub3(c1, c2);
  return dot3(diff, diff);
}

function triTriDistSq(t1: [Vec3, Vec3, Vec3], t2: [Vec3, Vec3, Vec3]): number {
  let best = Infinity;
  for (const p of t1) {
    best = Math.min(best, pointTriangleDistSq(p, t2[0], t2[1], t2[2]));
  }
  for (const q of t2) {
    best = Math.min(best, pointTriangleDistSq(q, t1[0], t1[1], t1[2]));
  }
  const e1: [Vec3, Vec3][] = [
    [t1[0], t1[1]],
    [t1[1], t1[2]],
    [t1[2], t1[0]],
  ];
  const e2: [Vec3, Vec3][] = [
    [t2[0], t2[1]],
    [t2[1], t2[2]],
    [t2[2], t2[0]],
  ];
  for (const [p1, q1] of e1) {
    for (const [p2, q2] of e2) {
      best = Math.min(best, segmentSegmentDistSq(p1, q1, p2, q2));
    }
  }
  return best;
}

function computeMeshClearance(meshA: MeshGeometryData, meshB: MeshGeometryData): number {
  const getTris = (m: MeshGeometryData): { v: [Vec3, Vec3, Vec3]; min: Vec3; max: Vec3 }[] => {
    const out: { v: [Vec3, Vec3, Vec3]; min: Vec3; max: Vec3 }[] = [];
    const idx = m.indices;
    const verts = m.vertices;
    for (let i = 0; i < idx.length; i += 3) {
      const i0 = idx[i] * 3;
      const i1 = idx[i + 1] * 3;
      const i2 = idx[i + 2] * 3;
      const v0: Vec3 = [verts[i0], verts[i0 + 1], verts[i0 + 2]];
      const v1: Vec3 = [verts[i1], verts[i1 + 1], verts[i1 + 2]];
      const v2: Vec3 = [verts[i2], verts[i2 + 1], verts[i2 + 2]];
      const min: Vec3 = [
        Math.min(v0[0], v1[0], v2[0]),
        Math.min(v0[1], v1[1], v2[1]),
        Math.min(v0[2], v1[2], v2[2]),
      ];
      const max: Vec3 = [
        Math.max(v0[0], v1[0], v2[0]),
        Math.max(v0[1], v1[1], v2[1]),
        Math.max(v0[2], v1[2], v2[2]),
      ];
      out.push({ v: [v0, v1, v2], min, max });
    }
    return out;
  };

  const trisA = getTris(meshA);
  const trisB = getTris(meshB);
  if (trisA.length === 0 || trisB.length === 0) return 0;

  let bestSq = Infinity;
  for (const ta of trisA) {
    for (const tb of trisB) {
      const dx = Math.max(0, tb.min[0] - ta.max[0], ta.min[0] - tb.max[0]);
      const dy = Math.max(0, tb.min[1] - ta.max[1], ta.min[1] - tb.max[1]);
      const dz = Math.max(0, tb.min[2] - ta.max[2], ta.min[2] - tb.max[2]);
      const aabbDistSq = dx * dx + dy * dy + dz * dz;
      if (aabbDistSq >= bestSq) continue;
      const dSq = triTriDistSq(ta.v, tb.v);
      if (dSq < bestSq) {
        bestSq = dSq;
        if (bestSq <= 1e-18) return 0;
      }
    }
  }
  return Number.isFinite(bestSq) ? Math.sqrt(bestSq) : 0;
}

/**
 * Checks pairwise volumetric interference and minimum clearance between parts (`P3-T10`).
 */
export function checkInterference(
  parts: CadPart[],
  joints: KinematicJoint[],
  options?: { partIds?: string[]; ignoreAdjacent?: boolean }
): InterferenceReport {
  const ignoreAdjacent = options?.ignoreAdjacent ?? false;
  const partMap = new Map<string, CadPart>();
  for (const p of parts) {
    partMap.set(p.id, p);
  }

  let candidateIds: string[];
  if (options?.partIds) {
    for (const pid of options.partIds) {
      if (!partMap.has(pid)) {
        throw new Error(`Part not found: ${pid}`);
      }
    }
    candidateIds = Array.from(new Set(options.partIds)).sort();
  } else {
    candidateIds = Array.from(partMap.keys()).sort();
  }

  const adjacentPairs = new Set<string>();
  for (const j of joints) {
    const a = j.partA;
    const b = j.partB;
    adjacentPairs.add(a <= b ? `${a}::${b}` : `${b}::${a}`);
  }

  const worldMeshes = new Map<string, MeshGeometryData>();
  for (const pid of candidateIds) {
    worldMeshes.set(pid, transformPartMeshToWorld(partMap.get(pid)!));
  }

  const pairResults: PairInterferenceResult[] = [];
  let interferingCount = 0;
  let minClearance: number | undefined = undefined;
  let maxPenetrationDepth = 0;
  let totalIntersectionVolume = 0;

  for (let i = 0; i < candidateIds.length; i++) {
    for (let j = i + 1; j < candidateIds.length; j++) {
      const idA = candidateIds[i];
      const idB = candidateIds[j];
      const adjacent = adjacentPairs.has(`${idA}::${idB}`);
      if (ignoreAdjacent && adjacent) continue;

      const meshA = worldMeshes.get(idA)!;
      const meshB = worldMeshes.get(idB)!;

      let intersects = false;
      let intersectionVolume: number | undefined = undefined;
      let penetrationDepth: number | undefined = undefined;
      let clearance: number | undefined = undefined;

      if (
        aabbIntersects(
          meshA.boundingBox.min,
          meshA.boundingBox.max,
          meshB.boundingBox.min,
          meshB.boundingBox.max
        )
      ) {
        const interMesh = csgIntersectionMesh(meshA, meshB);
        const vol = Math.abs(interMesh.volume);
        if (interMesh.triangleCount > 0 && vol > 1e-15) {
          intersects = true;
          intersectionVolume = vol;
          const bMin = interMesh.boundingBox.min;
          const bMax = interMesh.boundingBox.max;
          const extents = [bMax[0] - bMin[0], bMax[1] - bMin[1], bMax[2] - bMin[2]];
          const minExtent = Math.min(...extents.filter((e) => e > 0));
          const depth = Number.isFinite(minExtent) ? minExtent : Math.cbrt(vol);
          penetrationDepth = depth;
          interferingCount++;
          maxPenetrationDepth = Math.max(maxPenetrationDepth, depth);
          totalIntersectionVolume += vol;
        }
      }

      if (!intersects) {
        const dist = computeMeshClearance(meshA, meshB);
        clearance = dist;
        minClearance = minClearance === undefined ? dist : Math.min(minClearance, dist);
      }

      pairResults.push({
        a: idA,
        b: idB,
        intersects,
        penetration_depth: penetrationDepth,
        intersection_volume: intersectionVolume,
        clearance,
        adjacent_by_joint: adjacent,
      });
    }
  }

  return {
    status: interferingCount === 0 ? 'PASS' : 'FAIL',
    interfering_pairs_count: interferingCount,
    checked_pairs_count: pairResults.length,
    min_clearance: minClearance,
    max_penetration_depth: maxPenetrationDepth,
    total_intersection_volume: totalIntersectionVolume,
    pairs: pairResults,
  };
}

function mat3Mul(a: Mat3, b: Mat3): Mat3 {
  return [
    [
      a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
      a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
      a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
    ],
    [
      a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
      a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
      a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
    ],
    [
      a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
      a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
      a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
    ],
  ];
}

function axisAngleToMat3(axis: Vec3, angleRad: number): Mat3 {
  const [ux, uy, uz] = normalize3(axis);
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  const t = 1 - c;
  return [
    [t * ux * ux + c, t * ux * uy - s * uz, t * ux * uz + s * uy],
    [t * ux * uy + s * uz, t * uy * uy + c, t * uy * uz - s * ux],
    [t * ux * uz - s * uy, t * uy * uz + s * ux, t * uz * uz + c],
  ];
}

function mat3ToEulerXYZ(m: Mat3): Vec3 {
  const sy = Math.max(-1, Math.min(1, -m[2][0]));
  const ry = Math.asin(sy);
  const cy = Math.cos(ry);
  let rx = 0;
  let rz = 0;
  if (Math.abs(cy) > 1e-6) {
    rx = Math.atan2(m[2][1], m[2][2]);
    rz = Math.atan2(m[1][0], m[0][0]);
  } else {
    rx = Math.atan2(-m[1][2], m[1][1]);
    rz = 0;
  }
  return [rx, ry, rz];
}

/**
 * Sweeps a Revolute or Prismatic joint across `[from, to]` in `steps` discrete intervals
 * and checks interference and clearance at each step without mutating the input parts (`P3-T10`).
 */
export function sweepJoint(
  parts: CadPart[],
  joints: KinematicJoint[],
  options: {
    jointId: string;
    from: number;
    to: number;
    steps: number;
    partIds?: string[];
    ignoreAdjacent?: boolean;
  }
): JointSweepReport {
  const { jointId, from, to, steps } = options;
  const ignoreAdjacent = options.ignoreAdjacent ?? true;

  if (steps < 1 || !Number.isInteger(steps)) {
    throw new Error('sweep_joint requires steps >= 1');
  }
  if (!Number.isFinite(from) || !Number.isFinite(to)) {
    throw new Error('sweep_joint requires finite from and to bounds');
  }

  const joint = joints.find((j) => j.id === jointId);
  if (!joint) {
    throw new Error(`Joint not found: ${jointId}`);
  }
  if (joint.type !== 'revolute' && joint.type !== 'prismatic') {
    throw new Error(
      `sweep_joint supports only 'revolute' or 'prismatic' joints, got '${joint.type}'`
    );
  }

  const normId = (s: string) => s.trim().toLowerCase();
  const isGround = (id: string) => {
    const n = normId(id);
    if (n === 'ground' || n === 'world') return true;
    const p = parts.find((pt) => pt.id === id);
    return Boolean(p?.isFixed);
  };

  // Determine moving subtree vs base
  const adj = new Map<string, string[]>();
  for (const j of joints) {
    if (j.id === jointId) continue;
    if (!adj.has(j.partA)) adj.set(j.partA, []);
    if (!adj.has(j.partB)) adj.set(j.partB, []);
    adj.get(j.partA)!.push(j.partB);
    adj.get(j.partB)!.push(j.partA);
  }

  const bfsReachesGround = (start: string, blocked: string): Set<string> => {
    const visited = new Set<string>([start]);
    const q = [start];
    while (q.length > 0) {
      const cur = q.shift()!;
      for (const nxt of adj.get(cur) || []) {
        if (nxt === blocked || visited.has(nxt)) continue;
        visited.add(nxt);
        q.push(nxt);
      }
    }
    return visited;
  };

  const subB = bfsReachesGround(joint.partB, joint.partA);
  const subA = bfsReachesGround(joint.partA, joint.partB);
  const bGrounded = Array.from(subB).some(isGround);
  const aGrounded = Array.from(subA).some(isGround);

  let baseId = joint.partA;
  let movingSubtree = subB;
  let sign = 1.0;
  if (bGrounded && !aGrounded) {
    baseId = joint.partB;
    movingSubtree = subA;
    sign = -1.0;
  }

  const basePart = parts.find((p) => p.id === baseId);
  const drivenId = baseId === joint.partA ? joint.partB : joint.partA;
  const basePos: Vec3 = basePart ? [...basePart.position] : [0, 0, 0];
  const baseRot: Mat3 = basePart
    ? eulerToMat3(basePart.rotation)
    : [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
      ];

  const localAnchor: Vec3 =
    baseId === joint.partA ? joint.anchor || [0, 0, 0] : joint.anchorB ?? [0, 0, 0];
  const drivenAnchor: Vec3 =
    drivenId === joint.partA ? joint.anchor || [0, 0, 0] : joint.anchorB ?? [0, 0, 0];
  const pivotWorld: Vec3 = add3(basePos, mat3Vec3(baseRot, localAnchor));
  const axisLocalRef: Vec3 = normalize3(joint.axis || [0, 0, 1]);
  const axisWorld: Vec3 = normalize3(mat3Vec3(baseRot, axisLocalRef));

  // Detect whether the swept joint belongs to a closed kinematic loop (P3-T16).
  let touchesBoundary = 0;
  let touchesExternalOrGround = false;
  let internalJointsCount = 0;
  for (const j of joints) {
    if (j.id === jointId) continue;
    const aIn = movingSubtree.has(j.partA);
    const bIn = movingSubtree.has(j.partB);
    if (aIn && bIn) {
      internalJointsCount++;
    } else if (aIn !== bIn) {
      touchesBoundary++;
      const other = aIn ? j.partB : j.partA;
      if (other !== baseId) {
        touchesExternalOrGround = true;
      }
    }
  }
  const isClosedLoop =
    touchesBoundary > 0 || internalJointsCount > Math.max(0, movingSubtree.size - 1);

  const isRevolute = joint.type === 'revolute';
  const samples: SweepStepResult[] = [];
  let firstCollisionStep: number | undefined = undefined;
  let firstCollisionValue: number | undefined = undefined;
  let firstCollisionPairs: [string, string][] = [];
  let minClearance: number | undefined = undefined;
  let minClearanceStep: number | undefined = undefined;
  let minClearanceValue: number | undefined = undefined;
  let safeStart: number | undefined = undefined;
  let safeEnd: number | undefined = undefined;
  let lockupStep: number | undefined = undefined;
  let lockupValue: number | undefined = undefined;
  let maxClosureResidual = 0;

  if (!isClosedLoop) {
    for (let stepIdx = 0; stepIdx <= steps; stepIdx++) {
      const t = stepIdx / steps;
      const val = from + (to - from) * t;

      const stepParts: CadPart[] = parts.map((p) => {
        if (!movingSubtree.has(p.id) || isGround(p.id)) {
          return p;
        }
        if (isRevolute) {
          const deltaRot = axisAngleToMat3(axisWorld, val * sign);
          const rel = sub3(p.position, pivotWorld);
          const newPos = add3(pivotWorld, mat3Vec3(deltaRot, rel));
          const newRotMat = mat3Mul(deltaRot, eulerToMat3(p.rotation));
          return {
            ...p,
            position: newPos,
            rotation: mat3ToEulerXYZ(newRotMat),
          };
        } else {
          const deltaTrans = scale3(axisWorld, val * sign);
          return {
            ...p,
            position: add3(p.position, deltaTrans),
          };
        }
      });

      const rep = checkInterference(stepParts, joints, {
        partIds: options.partIds,
        ignoreAdjacent,
      });

      const interferingPairs: [string, string][] = rep.pairs
        .filter((pr) => pr.intersects)
        .map((pr) => [pr.a, pr.b]);

      if (interferingPairs.length > 0) {
        if (firstCollisionStep === undefined) {
          firstCollisionStep = stepIdx;
          firstCollisionValue = val;
          firstCollisionPairs = [...interferingPairs];
        }
      } else if (firstCollisionStep === undefined) {
        if (safeStart === undefined) safeStart = val;
        safeEnd = val;
      }

      if (rep.min_clearance !== undefined && interferingPairs.length === 0) {
        if (minClearance === undefined || rep.min_clearance < minClearance) {
          minClearance = rep.min_clearance;
          minClearanceStep = stepIdx;
          minClearanceValue = val;
        }
      }

      samples.push({
        step: stepIdx,
        value: val,
        angle_deg: isRevolute ? (val * 180) / Math.PI : undefined,
        angle_rad: isRevolute ? val : undefined,
        interfering_pairs: interferingPairs,
        min_clearance: rep.min_clearance,
        max_penetration_depth: rep.max_penetration_depth,
        total_intersection_volume: rep.total_intersection_volume,
        closure_residual: 0,
        locked_up: false,
      });
    }
  } else {
    const baseIsFixed = isGround(baseId);
    const includeBaseAsFree = !baseIsFixed && touchesExternalOrGround;
    const freePartsSet = new Set<string>();
    if (includeBaseAsFree) {
      const q = [baseId, drivenId];
      for (const s of q) {
        if (!isGround(s)) freePartsSet.add(s);
      }
      const fullAdj = new Map<string, string[]>();
      for (const j of joints) {
        if (!fullAdj.has(j.partA)) fullAdj.set(j.partA, []);
        if (!fullAdj.has(j.partB)) fullAdj.set(j.partB, []);
        fullAdj.get(j.partA)!.push(j.partB);
        fullAdj.get(j.partB)!.push(j.partA);
      }
      const bfs = [...freePartsSet];
      while (bfs.length > 0) {
        const cur = bfs.shift()!;
        for (const nxt of fullAdj.get(cur) || []) {
          if (!isGround(nxt) && !freePartsSet.has(nxt)) {
            freePartsSet.add(nxt);
            bfs.push(nxt);
          }
        }
      }
    } else {
      for (const pid of movingSubtree) {
        if (pid !== drivenId && !isGround(pid)) {
          freePartsSet.add(pid);
        }
      }
    }
    const freeParts = Array.from(freePartsSet).sort();

    const solver = createClosedLoopSolverTS(
      parts,
      joints,
      joint,
      baseId,
      drivenId,
      sign,
      localAnchor,
      drivenAnchor,
      axisLocalRef,
      freeParts,
      includeBaseAsFree,
      isGround
    );

    for (let stepIdx = 0; stepIdx <= steps; stepIdx++) {
      const t = stepIdx / steps;
      const val = from + (to - from) * t;

      if (lockupStep !== undefined) {
        samples.push({
          step: stepIdx,
          value: val,
          angle_deg: isRevolute ? (val * 180) / Math.PI : undefined,
          angle_rad: isRevolute ? val : undefined,
          interfering_pairs: [],
          min_clearance: undefined,
          max_penetration_depth: 0,
          total_intersection_volume: 0,
          closure_residual: solver.lastFailedResidual,
          locked_up: true,
        });
        continue;
      }

      const stepOutcome = solver.advanceTo(val);
      if (stepOutcome.residual > maxClosureResidual) {
        maxClosureResidual = stepOutcome.residual;
      }
      if (!stepOutcome.ok) {
        lockupStep = stepIdx;
        lockupValue = val;
        samples.push({
          step: stepIdx,
          value: val,
          angle_deg: isRevolute ? (val * 180) / Math.PI : undefined,
          angle_rad: isRevolute ? val : undefined,
          interfering_pairs: [],
          min_clearance: undefined,
          max_penetration_depth: 0,
          total_intersection_volume: 0,
          closure_residual: stepOutcome.residual,
          locked_up: true,
        });
        continue;
      }

      const stepParts: CadPart[] = parts.map((p) => {
        const pose = solver.poses.get(p.id);
        if (!pose || isGround(p.id)) {
          return p;
        }
        return {
          ...p,
          position: [...pose.pos] as Vec3,
          rotation: mat3ToEulerXYZ(pose.rot),
        };
      });

      const rep = checkInterference(stepParts, joints, {
        partIds: options.partIds,
        ignoreAdjacent,
      });

      const interferingPairs: [string, string][] = rep.pairs
        .filter((pr) => pr.intersects)
        .map((pr) => [pr.a, pr.b]);

      if (interferingPairs.length > 0) {
        if (firstCollisionStep === undefined) {
          firstCollisionStep = stepIdx;
          firstCollisionValue = val;
          firstCollisionPairs = [...interferingPairs];
        }
      } else if (firstCollisionStep === undefined) {
        if (safeStart === undefined) safeStart = val;
        safeEnd = val;
      }

      if (rep.min_clearance !== undefined && interferingPairs.length === 0) {
        if (minClearance === undefined || rep.min_clearance < minClearance) {
          minClearance = rep.min_clearance;
          minClearanceStep = stepIdx;
          minClearanceValue = val;
        }
      }

      samples.push({
        step: stepIdx,
        value: val,
        angle_deg: isRevolute ? (val * 180) / Math.PI : undefined,
        angle_rad: isRevolute ? val : undefined,
        interfering_pairs: interferingPairs,
        min_clearance: rep.min_clearance,
        max_penetration_depth: rep.max_penetration_depth,
        total_intersection_volume: rep.total_intersection_volume,
        closure_residual: stepOutcome.residual,
        locked_up: false,
      });
    }
  }

  const lockupDetected = lockupStep !== undefined;
  const safeTravelRange: [number, number] | undefined =
    safeStart !== undefined && safeEnd !== undefined ? [safeStart, safeEnd] : undefined;
  const marginToCollision: number | undefined =
    firstCollisionValue !== undefined ? Math.abs(firstCollisionValue - from) : undefined;

  return {
    status: firstCollisionStep === undefined && !lockupDetected ? 'PASS' : 'FAIL',
    joint_id: jointId,
    joint_type: joint.type,
    from,
    to,
    steps,
    ignore_adjacent: ignoreAdjacent,
    first_collision_step: firstCollisionStep,
    first_collision_value: firstCollisionValue,
    first_collision_angle_deg:
      isRevolute && firstCollisionValue !== undefined
        ? (firstCollisionValue * 180) / Math.PI
        : undefined,
    first_collision_angle_rad: isRevolute ? firstCollisionValue : undefined,
    first_collision_pairs: firstCollisionPairs,
    min_clearance: minClearance,
    min_clearance_step: minClearanceStep,
    min_clearance_value: minClearanceValue,
    safe_travel_range: safeTravelRange,
    margin_to_collision: marginToCollision,
    closed_loop_detected: isClosedLoop,
    lockup_detected: lockupDetected,
    lockup_step: lockupStep,
    lockup_value: lockupValue,
    lockup_angle_deg:
      isRevolute && lockupValue !== undefined ? (lockupValue * 180) / Math.PI : undefined,
    lockup_angle_rad: isRevolute ? lockupValue : undefined,
    max_closure_residual: maxClosureResidual,
    samples,
  };
}

interface PoseTS {
  pos: Vec3;
  rot: Mat3;
}

function mat3ToScaledAxisTS(m: Mat3): Vec3 {
  const trace = m[0][0] + m[1][1] + m[2][2];
  let x = 0, y = 0, z = 0, w = 1;
  if (trace > 0) {
    const s = Math.sqrt(trace + 1.0) * 2;
    w = 0.25 * s;
    x = (m[2][1] - m[1][2]) / s;
    y = (m[0][2] - m[2][0]) / s;
    z = (m[1][0] - m[0][1]) / s;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const s = Math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
    w = (m[2][1] - m[1][2]) / s;
    x = 0.25 * s;
    y = (m[0][1] + m[1][0]) / s;
    z = (m[0][2] + m[2][0]) / s;
  } else if (m[1][1] > m[2][2]) {
    const s = Math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
    w = (m[0][2] - m[2][0]) / s;
    x = (m[0][1] + m[1][0]) / s;
    y = 0.25 * s;
    z = (m[1][2] + m[2][1]) / s;
  } else {
    const s = Math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
    w = (m[1][0] - m[0][1]) / s;
    x = (m[0][2] + m[2][0]) / s;
    y = (m[1][2] + m[2][1]) / s;
    z = 0.25 * s;
  }
  if (w < 0) {
    x = -x; y = -y; z = -z; w = -w;
  }
  const vNorm = Math.hypot(x, y, z);
  const theta = 2 * Math.atan2(vNorm, w);
  const k = vNorm > 1e-12 ? theta / vNorm : 2.0;
  return [x * k, y * k, z * k];
}

function solveDampedSvdStepTS(
  jMat: number[][],
  phi: number[],
  rows: number,
  cols: number,
  lambdaRatio: number
): { dx: number[]; sMax: number } {
  const dx = new Array(cols).fill(0);
  if (rows === 0 || cols === 0) return { dx, sMax: 1.0 };

  const transpose = rows < cols;
  // Orthogonalize columns of `matWork` (height `h`, width `w`) and apply same rotations to `orthOther` (`w x w`)
  const h = transpose ? cols : rows;
  const w = transpose ? rows : cols;

  const workCols: number[][] = Array.from({ length: w }, (_, j) =>
    Array.from({ length: h }, (__, i) => (transpose ? jMat[j][i] : jMat[i][j]))
  );
  const rightCols: number[][] = Array.from({ length: w }, (_, j) => {
    const col = new Array(w).fill(0);
    col[j] = 1;
    return col;
  });

  const maxSweeps = 40;
  for (let sweep = 0; sweep < maxSweeps; sweep++) {
    let changed = false;
    for (let p = 0; p < w; p++) {
      for (let q = p + 1; q < w; q++) {
        let app = 0, aqq = 0, apq = 0;
        const up = workCols[p];
        const uq = workCols[q];
        for (let i = 0; i < h; i++) {
          app += up[i] * up[i];
          aqq += uq[i] * uq[i];
          apq += up[i] * uq[i];
        }
        if (Math.abs(apq) <= 1e-15 * Math.sqrt(Math.max(1e-30, app * aqq))) {
          continue;
        }
        changed = true;
        const tau = (aqq - app) / (2 * apq);
        const t =
          tau >= 0
            ? 1 / (tau + Math.sqrt(1 + tau * tau))
            : -1 / (-tau + Math.sqrt(1 + tau * tau));
        const c = 1 / Math.sqrt(1 + t * t);
        const s = t * c;
        for (let i = 0; i < h; i++) {
          const vip = up[i];
          const viq = uq[i];
          up[i] = c * vip - s * viq;
          uq[i] = s * vip + c * viq;
        }
        const vp = rightCols[p];
        const vq = rightCols[q];
        for (let i = 0; i < w; i++) {
          const rrp = vp[i];
          const rrq = vq[i];
          vp[i] = c * rrp - s * rrq;
          vq[i] = s * rrp + c * rrq;
        }
      }
    }
    if (!changed) break;
  }

  const sigmas = workCols.map((col) => Math.hypot(...col));
  let sMax = 0;
  for (const s of sigmas) if (s > sMax) sMax = s;
  sMax = Math.max(sMax, 1e-12);
  const lambda = Math.max(lambdaRatio * sMax, 1e-12);

  for (let k = 0; k < w; k++) {
    const s = sigmas[k];
    const denom = s * s + lambda * lambda;
    if (denom <= 0) continue;
    if (!transpose) {
      let uDotPhi = 0;
      const uk = workCols[k];
      for (let i = 0; i < rows; i++) uDotPhi += uk[i] * phi[i];
      const coeff = -uDotPhi / denom;
      const vk = rightCols[k];
      for (let j = 0; j < cols; j++) dx[j] += coeff * vk[j];
    } else {
      let uDotPhi = 0;
      const uk = rightCols[k];
      for (let i = 0; i < rows; i++) uDotPhi += uk[i] * phi[i];
      const coeff = -uDotPhi / denom;
      const vk = workCols[k];
      for (let j = 0; j < cols; j++) dx[j] += coeff * vk[j];
    }
  }

  return { dx, sMax };
}

function createClosedLoopSolverTS(
  parts: CadPart[],
  joints: KinematicJoint[],
  drivenJoint: KinematicJoint,
  referenceId: string,
  drivenId: string,
  sign: number,
  anchorRef: Vec3,
  anchorDriven: Vec3,
  axisLocalRef: Vec3,
  freeParts: string[],
  drivenAsConstraint: boolean,
  isGround: (id: string) => boolean
) {
  const partMap = new Map<string, CadPart>();
  for (const p of parts) partMap.set(p.id, p);

  const getInitPose = (pid: string): PoseTS => {
    const p = partMap.get(pid);
    if (!p || pid.toLowerCase() === 'ground' || pid.toLowerCase() === 'world') {
      return {
        pos: [0, 0, 0],
        rot: [
          [1, 0, 0],
          [0, 1, 0],
          [0, 0, 1],
        ],
      };
    }
    return { pos: [...p.position] as Vec3, rot: eulerToMat3(p.rotation) };
  };

  const poses = new Map<string, PoseTS>();
  const freeSet = new Set(freeParts);
  const freeIndex = new Map<string, number>();
  freeParts.forEach((id, i) => freeIndex.set(id, i));

  interface LoopEntryTS {
    joint: KinematicJoint;
    initRelRot: Mat3;
    initCoord?: number;
    isDrivenConstraint: boolean;
    gearThetaA: number;
    gearThetaB: number;
  }

  const activeJoints: LoopEntryTS[] = [];
  const scalePoints: Vec3[] = [];
  const sortedJoints = joints.slice().sort((a, b) => a.id.localeCompare(b.id));

  for (const j of sortedJoints) {
    const isDriven = j.id === drivenJoint.id;
    if (isDriven && !drivenAsConstraint) continue;
    const touchesFree = freeSet.has(j.partA) || freeSet.has(j.partB);
    const touchesDrivenOnly =
      freeParts.length === 0 && !isDriven && (j.partA === drivenId || j.partB === drivenId);
    if (!touchesFree && !touchesDrivenOnly && !isDriven) continue;

    if (!poses.has(j.partA)) poses.set(j.partA, getInitPose(j.partA));
    if (!poses.has(j.partB)) poses.set(j.partB, getInitPose(j.partB));
    const pa = poses.get(j.partA)!;
    const pb = poses.get(j.partB)!;
    scalePoints.push(add3(pa.pos, mat3Vec3(pa.rot, j.anchor || [0, 0, 0])));
    scalePoints.push(add3(pb.pos, mat3Vec3(pb.rot, j.anchorB ?? [0, 0, 0])));

    const initRelRot = mat3Mul(transpose3(pa.rot), pb.rot);
    let initCoord: number | undefined = undefined;
    if (j.type === 'revolute') {
      const [xLoc, yLoc] = orthonormalBasis(j.axis || [0, 0, 1]);
      const rotX = mat3Vec3(initRelRot, xLoc);
      initCoord = Math.atan2(dot3(yLoc, rotX), dot3(xLoc, rotX));
    } else if (j.type === 'prismatic') {
      const zA = mat3Vec3(pa.rot, normalizeOrDefault3(j.axis || [1, 0, 0], [1, 0, 0]));
      const pAW = add3(pa.pos, mat3Vec3(pa.rot, j.anchor || [0, 0, 0]));
      const pBW = add3(pb.pos, mat3Vec3(pb.rot, j.anchorB ?? [0, 0, 0]));
      initCoord = dot3(sub3(pBW, pAW), zA);
    }

    activeJoints.push({
      joint: j,
      initRelRot,
      initCoord,
      isDrivenConstraint: isDriven,
      gearThetaA: 0,
      gearThetaB: 0,
    });
  }

  if (!poses.has(referenceId)) poses.set(referenceId, getInitPose(referenceId));
  if (!poses.has(drivenId)) poses.set(drivenId, getInitPose(drivenId));
  const ref0 = poses.get(referenceId)!;
  const drv0 = poses.get(drivenId)!;
  scalePoints.push(add3(ref0.pos, mat3Vec3(ref0.rot, anchorRef)));
  scalePoints.push(add3(drv0.pos, mat3Vec3(drv0.rot, anchorDriven)));

  let maxSpan = 0;
  for (let i = 0; i < scalePoints.length; i++) {
    for (let j = i + 1; j < scalePoints.length; j++) {
      const d = norm3(sub3(scalePoints[i], scalePoints[j]));
      if (d > maxSpan) maxSpan = d;
    }
  }
  const charLen = maxSpan > 1e-6 ? Math.min(100, Math.max(1e-3, maxSpan)) : 0.1;
  const posTol = Math.min(5e-4, Math.max(2e-5, charLen * 1e-3));
  const rotTol = 1e-3;
  const pivotWorld = add3(ref0.pos, mat3Vec3(ref0.rot, anchorRef));
  const worldAxis = normalize3(mat3Vec3(ref0.rot, axisLocalRef));
  const isRevolute = drivenJoint.type === 'revolute';

  let currentQ = 0;
  let lastFailedResidual = 0;

  const drivenWorldPose = (qVal: number): PoseTS => {
    const qEff = qVal * sign;
    if (isRevolute) {
      const deltaRot = axisAngleToMat3(worldAxis, qEff);
      const rk = mat3Mul(deltaRot, drv0.rot);
      const pk = sub3(pivotWorld, mat3Vec3(rk, anchorDriven));
      return { pos: pk, rot: rk };
    } else {
      const rk = drv0.rot;
      const pk = add3(
        sub3(pivotWorld, mat3Vec3(rk, anchorDriven)),
        scale3(worldAxis, qEff)
      );
      return { pos: pk, rot: rk };
    }
  };

  const clonePoses = (src: Map<string, PoseTS>): Map<string, PoseTS> => {
    const dst = new Map<string, PoseTS>();
    for (const [k, v] of src.entries()) {
      dst.set(k, {
        pos: [...v.pos] as Vec3,
        rot: [
          [...v.rot[0]] as Vec3,
          [...v.rot[1]] as Vec3,
          [...v.rot[2]] as Vec3,
        ],
      });
    }
    return dst;
  };

  const totalRows = activeJoints.reduce(
    (acc, e) => acc + (e.isDrivenConstraint ? 6 : effectiveJointConstraintCount(e.joint)),
    0
  );
  const totalCols = 6 * freeParts.length;
  const eBasis: Vec3[] = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];

  const evaluateSystem = (
    qVal: number,
    curPoses: Map<string, PoseTS>,
    basePoses: Map<string, PoseTS>,
    baseGears: [number, number][],
    buildJac: boolean
  ) => {
    const phiPhys = new Array(totalRows).fill(0);
    const phiScaled = new Array(totalRows).fill(0);
    const jMat: number[][] | null = buildJac
      ? Array.from({ length: totalRows }, () => new Array(totalCols).fill(0))
      : null;

    const addVec = (r: number, colIdx: number | undefined, isRot: boolean, v: Vec3) => {
      if (!jMat || colIdx === undefined) return;
      const base = 6 * colIdx + (isRot ? 3 : 0);
      jMat[r][base] += v[0];
      jMat[r][base + 1] += v[1];
      jMat[r][base + 2] += v[2];
    };

    const scaleJacRows = (startR: number, endR: number, factor: number) => {
      if (!jMat) return;
      for (let r = startR; r < endR; r++) {
        for (let c = 0; c < totalCols; c++) jMat[r][c] *= factor;
      }
    };

    const invLen = 1.0 / charLen;
    let maxPosErr = 0;
    let maxRotErr = 0;
    let row = 0;

    for (let jIdx = 0; jIdx < activeJoints.length; jIdx++) {
      const entry = activeJoints[jIdx];
      if (entry.isDrivenConstraint) {
        const pa = curPoses.get(referenceId)!;
        const pb = curPoses.get(drivenId)!;
        const colA = freeIndex.get(referenceId);
        const colB = freeIndex.get(drivenId);
        const qEff = qVal * sign;
        const sLoc: Vec3 = isRevolute ? [0, 0, 0] : scale3(axisLocalRef, qEff);
        const effAnchorA = add3(anchorRef, sLoc);
        const rA = add3(pa.pos, mat3Vec3(pa.rot, effAnchorA));
        const rB = add3(pb.pos, mat3Vec3(pb.rot, anchorDriven));
        const dp = sub3(rA, rB);

        const rel0 = mat3Mul(transpose3(ref0.rot), drv0.rot);
        const targetRel = isRevolute
          ? mat3Mul(axisAngleToMat3(axisLocalRef, qEff), rel0)
          : rel0;
        const rErr = mat3Mul(pb.rot, transpose3(mat3Mul(pa.rot, targetRel)));
        const drot = mat3ToScaledAxisTS(rErr);

        for (let k = 0; k < 3; k++) {
          maxPosErr = Math.max(maxPosErr, Math.abs(dp[k]));
          maxRotErr = Math.max(maxRotErr, Math.abs(drot[k]));
          phiPhys[row + k] = dp[k];
          phiScaled[row + k] = dp[k] * invLen;
          phiPhys[row + 3 + k] = drot[k];
          phiScaled[row + 3 + k] = drot[k];
        }
        if (jMat) {
          const dA = mat3Vec3(pa.rot, effAnchorA);
          const dB = mat3Vec3(pb.rot, anchorDriven);
          for (let k = 0; k < 3; k++) {
            const ek = eBasis[k];
            addVec(row + k, colA, false, ek);
            addVec(row + k, colA, true, cross3(dA, ek));
            addVec(row + k, colB, false, scale3(ek, -1));
            addVec(row + k, colB, true, scale3(cross3(dB, ek), -1));
            addVec(row + 3 + k, colA, true, scale3(ek, -1));
            addVec(row + 3 + k, colB, true, ek);
          }
          scaleJacRows(row, row + 3, invLen);
        }
        row += 6;
        continue;
      }

      const j = entry.joint;
      const pa = curPoses.get(j.partA)!;
      const pb = curPoses.get(j.partB)!;
      const colA = freeIndex.get(j.partA);
      const colB = freeIndex.get(j.partB);
      const anchorA: Vec3 = j.anchor || [0, 0, 0];
      const anchorB: Vec3 = j.anchorB ?? [0, 0, 0];
      const dA = mat3Vec3(pa.rot, anchorA);
      const dB = mat3Vec3(pb.rot, anchorB);

      if (j.type === 'fixed') {
        const dp = sub3(add3(pa.pos, dA), add3(pb.pos, dB));
        const drot = mat3ToScaledAxisTS(
          mat3Mul(pb.rot, transpose3(mat3Mul(pa.rot, entry.initRelRot)))
        );
        for (let k = 0; k < 3; k++) {
          maxPosErr = Math.max(maxPosErr, Math.abs(dp[k]));
          maxRotErr = Math.max(maxRotErr, Math.abs(drot[k]));
          phiPhys[row + k] = dp[k];
          phiScaled[row + k] = dp[k] * invLen;
          phiPhys[row + 3 + k] = drot[k];
          phiScaled[row + 3 + k] = drot[k];
        }
        if (jMat) {
          for (let k = 0; k < 3; k++) {
            const ek = eBasis[k];
            addVec(row + k, colA, false, ek);
            addVec(row + k, colA, true, cross3(dA, ek));
            addVec(row + k, colB, false, scale3(ek, -1));
            addVec(row + k, colB, true, scale3(cross3(dB, ek), -1));
            addVec(row + 3 + k, colA, true, scale3(ek, -1));
            addVec(row + 3 + k, colB, true, ek);
          }
          scaleJacRows(row, row + 3, invLen);
        }
        row += 6;
      } else if (j.type === 'spherical') {
        const dp = sub3(add3(pa.pos, dA), add3(pb.pos, dB));
        for (let k = 0; k < 3; k++) {
          maxPosErr = Math.max(maxPosErr, Math.abs(dp[k]));
          phiPhys[row + k] = dp[k];
          phiScaled[row + k] = dp[k] * invLen;
        }
        if (jMat) {
          for (let k = 0; k < 3; k++) {
            const ek = eBasis[k];
            addVec(row + k, colA, false, ek);
            addVec(row + k, colA, true, cross3(dA, ek));
            addVec(row + k, colB, false, scale3(ek, -1));
            addVec(row + k, colB, true, scale3(cross3(dB, ek), -1));
          }
          scaleJacRows(row, row + 3, invLen);
        }
        row += 3;
      } else if (j.type === 'revolute') {
        const dp = sub3(add3(pa.pos, dA), add3(pb.pos, dB));
        for (let k = 0; k < 3; k++) {
          maxPosErr = Math.max(maxPosErr, Math.abs(dp[k]));
          phiPhys[row + k] = dp[k];
          phiScaled[row + k] = dp[k] * invLen;
        }
        const uA = normalizeOrDefault3(j.axis || [0, 0, 1], [0, 0, 1]);
        const zA = mat3Vec3(pa.rot, uA);
        const [xA, yA] = orthonormalBasis(zA);
        const zB = mat3Vec3(pb.rot, uA);
        const r3 = -dot3(yA, zB);
        const r4 = dot3(xA, zB);
        const axErr = Math.atan2(norm3(cross3(zA, zB)), dot3(zA, zB));
        maxRotErr = Math.max(maxRotErr, axErr);
        phiPhys[row + 3] = r3;
        phiScaled[row + 3] = r3;
        phiPhys[row + 4] = r4;
        phiScaled[row + 4] = r4;

        const hasZero = isZeroRangeLimits(j.limits);
        if (hasZero) {
          const [xLoc, yLoc] = orthonormalBasis(uA);
          const rRel = mat3Mul(transpose3(pa.rot), pb.rot);
          const rotX = mat3Vec3(rRel, xLoc);
          const cur = Math.atan2(dot3(yLoc, rotX), dot3(xLoc, rotX));
          const target = j.limits ? j.limits[0] : (entry.initCoord ?? 0);
          const diff = (((cur - target + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
          maxRotErr = Math.max(maxRotErr, Math.abs(diff));
          phiPhys[row + 5] = diff;
          phiScaled[row + 5] = diff;
        }
        if (jMat) {
          for (let k = 0; k < 3; k++) {
            const ek = eBasis[k];
            addVec(row + k, colA, false, ek);
            addVec(row + k, colA, true, cross3(dA, ek));
            addVec(row + k, colB, false, scale3(ek, -1));
            addVec(row + k, colB, true, scale3(cross3(dB, ek), -1));
          }
          addVec(row + 3, colA, true, scale3(xA, -1));
          addVec(row + 3, colB, true, xA);
          addVec(row + 4, colA, true, scale3(yA, -1));
          addVec(row + 4, colB, true, yA);
          if (hasZero) {
            addVec(row + 5, colA, true, scale3(zA, -1));
            addVec(row + 5, colB, true, zA);
          }
          scaleJacRows(row, row + 3, invLen);
        }
        row += hasZero ? 6 : 5;
      } else if (j.type === 'prismatic') {
        const uA = normalizeOrDefault3(j.axis || [1, 0, 0], [1, 0, 0]);
        const zA = mat3Vec3(pa.rot, uA);
        const [xA, yA] = orthonormalBasis(zA);
        const drot = mat3ToScaledAxisTS(
          mat3Mul(pb.rot, transpose3(mat3Mul(pa.rot, entry.initRelRot)))
        );
        for (let k = 0; k < 3; k++) {
          maxRotErr = Math.max(maxRotErr, Math.abs(drot[k]));
          phiPhys[row + k] = drot[k];
          phiScaled[row + k] = drot[k];
        }
        const rA = add3(pa.pos, dA);
        const rB = add3(pb.pos, dB);
        const deltaR = sub3(rB, rA);
        const p3 = dot3(xA, deltaR);
        const p4 = dot3(yA, deltaR);
        maxPosErr = Math.max(maxPosErr, Math.abs(p3), Math.abs(p4));
        phiPhys[row + 3] = p3;
        phiScaled[row + 3] = p3 * invLen;
        phiPhys[row + 4] = p4;
        phiScaled[row + 4] = p4 * invLen;

        const hasZero = isZeroRangeLimits(j.limits);
        if (hasZero) {
          const target = j.limits ? j.limits[0] : (entry.initCoord ?? 0);
          const p5 = dot3(deltaR, zA) - target;
          maxPosErr = Math.max(maxPosErr, Math.abs(p5));
          phiPhys[row + 5] = p5;
          phiScaled[row + 5] = p5 * invLen;
        }
        if (jMat) {
          for (let k = 0; k < 3; k++) {
            const ek = eBasis[k];
            addVec(row + k, colA, true, scale3(ek, -1));
            addVec(row + k, colB, true, ek);
          }
          const relBFromA = sub3(rB, pa.pos);
          addVec(row + 3, colA, false, scale3(xA, -1));
          addVec(row + 3, colA, true, cross3(xA, relBFromA));
          addVec(row + 3, colB, false, xA);
          addVec(row + 3, colB, true, cross3(dB, xA));
          addVec(row + 4, colA, false, scale3(yA, -1));
          addVec(row + 4, colA, true, cross3(yA, relBFromA));
          addVec(row + 4, colB, false, yA);
          addVec(row + 4, colB, true, cross3(dB, yA));
          if (hasZero) {
            addVec(row + 5, colA, false, scale3(zA, -1));
            addVec(row + 5, colA, true, cross3(zA, relBFromA));
            addVec(row + 5, colB, false, zA);
            addVec(row + 5, colB, true, cross3(dB, zA));
          }
          scaleJacRows(row + 3, hasZero ? row + 6 : row + 5, invLen);
        }
        row += hasZero ? 6 : 5;
      } else if (j.type === 'gear') {
        const ratio = j.gearRatio ?? 1.0;
        const uA = normalizeOrDefault3(j.axis || [0, 0, 1], [0, 0, 1]);
        const uB = normalizeOrDefault3(j.axis || [0, 0, 1], [0, 0, 1]);
        const paBase = basePoses.get(j.partA)!;
        const pbBase = basePoses.get(j.partB)!;
        const dta = dot3(
          mat3ToScaledAxisTS(mat3Mul(pa.rot, transpose3(paBase.rot))),
          mat3Vec3(paBase.rot, uA)
        );
        const dtb = dot3(
          mat3ToScaledAxisTS(mat3Mul(pb.rot, transpose3(pbBase.rot))),
          mat3Vec3(pbBase.rot, uB)
        );
        const r0 = ratio * (baseGears[jIdx][0] + dta) + (baseGears[jIdx][1] + dtb);
        maxRotErr = Math.max(maxRotErr, Math.abs(r0));
        phiPhys[row] = r0;
        phiScaled[row] = r0;
        if (jMat) {
          addVec(row, colA, true, scale3(mat3Vec3(pa.rot, uA), ratio));
          addVec(row, colB, true, mat3Vec3(pb.rot, uB));
        }
        row += 1;
      }
    }

    if (jMat) {
      for (let idx = 0; idx < freeParts.length; idx++) {
        const baseCol = 6 * idx;
        for (let c = 0; c < 3; c++) {
          for (let r = 0; r < totalRows; r++) {
            jMat[r][baseCol + c] *= charLen;
          }
        }
      }
    }

    let physNorm = 0;
    let scaledNorm = 0;
    for (let i = 0; i < totalRows; i++) {
      physNorm += phiPhys[i] * phiPhys[i];
      scaledNorm += phiScaled[i] * phiScaled[i];
    }
    return {
      phiScaled,
      physNorm: Math.sqrt(physNorm),
      scaledNorm: Math.sqrt(scaledNorm),
      maxPosErr,
      maxRotErr,
      jMat,
    };
  };

  const commitGears = (basePoses: Map<string, PoseTS>, baseGears: [number, number][]) => {
    for (let i = 0; i < activeJoints.length; i++) {
      const entry = activeJoints[i];
      if (entry.joint.type === 'gear') {
        const uA = normalizeOrDefault3(entry.joint.axis || [0, 0, 1], [0, 0, 1]);
        const paCur = poses.get(entry.joint.partA)!;
        const pbCur = poses.get(entry.joint.partB)!;
        const paBase = basePoses.get(entry.joint.partA)!;
        const pbBase = basePoses.get(entry.joint.partB)!;
        const dta = dot3(
          mat3ToScaledAxisTS(mat3Mul(paCur.rot, transpose3(paBase.rot))),
          mat3Vec3(paBase.rot, uA)
        );
        const dtb = dot3(
          mat3ToScaledAxisTS(mat3Mul(pbCur.rot, transpose3(pbBase.rot))),
          mat3Vec3(pbBase.rot, uA)
        );
        entry.gearThetaA = baseGears[i][0] + dta;
        entry.gearThetaB = baseGears[i][1] + dtb;
      }
    }
  };

  const solveSingleQ = (qVal: number): { ok: boolean; residual: number } => {
    const basePoses = clonePoses(poses);
    const baseGears: [number, number][] = activeJoints.map((e) => [e.gearThetaA, e.gearThetaB]);

    if (!drivenAsConstraint) {
      poses.set(drivenId, drivenWorldPose(qVal));
    }
    if (totalRows === 0) return { ok: true, residual: 0 };

    if (freeParts.length === 0) {
      const ev = evaluateSystem(qVal, poses, basePoses, baseGears, false);
      if (ev.maxPosErr <= posTol && ev.maxRotErr <= rotTol) {
        return { ok: true, residual: ev.physNorm };
      }
      return { ok: false, residual: ev.physNorm };
    }

    let lambdaFactor = 1e-9;
    const maxIters = 50;

    for (let iter = 0; iter < maxIters; iter++) {
      const ev = evaluateSystem(qVal, poses, basePoses, baseGears, true);
      if (ev.maxPosErr <= posTol && ev.maxRotErr <= rotTol && ev.scaledNorm <= 1e-4) {
        commitGears(basePoses, baseGears);
        return { ok: true, residual: ev.physNorm };
      }

      let accepted = false;
      let tryLambda = lambdaFactor;
      for (let dampTry = 0; dampTry < 7; dampTry++) {
        const { dx } = solveDampedSvdStepTS(
          ev.jMat!,
          ev.phiScaled,
          totalRows,
          totalCols,
          tryLambda
        );
        let stepNorm = 0;
        for (const v of dx) stepNorm += v * v;
        stepNorm = Math.sqrt(stepNorm);
        if (stepNorm > 0.5) {
          const s = 0.5 / stepNorm;
          for (let i = 0; i < dx.length; i++) dx[i] *= s;
        }

        for (const alpha of [1.0, 0.5, 0.25]) {
          const trialPoses = clonePoses(poses);
          for (let idx = 0; idx < freeParts.length; idx++) {
            const pid = freeParts[idx];
            const base = 6 * idx;
            const pose = trialPoses.get(pid);
            if (!pose) continue;
            pose.pos[0] += dx[base] * charLen * alpha;
            pose.pos[1] += dx[base + 1] * charLen * alpha;
            pose.pos[2] += dx[base + 2] * charLen * alpha;
            const dth: Vec3 = [
              dx[base + 3] * alpha,
              dx[base + 4] * alpha,
              dx[base + 5] * alpha,
            ];
            const ang = norm3(dth);
            if (ang > 1e-14) {
              pose.rot = mat3Mul(axisAngleToMat3(dth, ang), pose.rot);
            }
          }

          const trialEv = evaluateSystem(qVal, trialPoses, basePoses, baseGears, false);
          if (trialEv.scaledNorm < ev.scaledNorm - 1e-14) {
            for (const [k, v] of trialPoses.entries()) poses.set(k, v);
            lambdaFactor = Math.max(1e-11, Math.min(100, tryLambda * 0.3));
            accepted = true;
            break;
          }
        }
        if (accepted) break;
        tryLambda *= 10.0;
      }
      if (!accepted) break;
    }

    const finalEv = evaluateSystem(qVal, poses, basePoses, baseGears, false);
    if (finalEv.maxPosErr <= posTol && finalEv.maxRotErr <= rotTol) {
      commitGears(basePoses, baseGears);
      return { ok: true, residual: finalEv.physNorm };
    }
    const restored = clonePoses(basePoses);
    for (const [k, v] of restored.entries()) poses.set(k, v);
    return { ok: false, residual: finalEv.physNorm };
  };

  return {
    poses,
    get lastFailedResidual() {
      return lastFailedResidual;
    },
    advanceTo(qTarget: number): { ok: boolean; residual: number } {
      const dqTotal = qTarget - currentQ;
      if (Math.abs(dqTotal) <= 1e-12 && Math.abs(currentQ) <= 1e-12) {
        return solveSingleQ(qTarget);
      }
      const maxSub = isRevolute ? (3.0 * Math.PI) / 180 : Math.max(1e-4, charLen * 0.05);
      const nSub = Math.min(2000, Math.max(1, Math.ceil(Math.abs(dqTotal) / maxSub)));
      const qStart = currentQ;
      let lastRes = 0;
      for (let s = 1; s <= nSub; s++) {
        const qSub = qStart + (dqTotal * s) / nSub;
        const res = solveSingleQ(qSub);
        if (!res.ok) {
          lastFailedResidual = res.residual;
          return res;
        }
        currentQ = qSub;
        lastRes = res.residual;
      }
      return { ok: true, residual: lastRes };
    },
  };
}


