import type {
  CachedFemSurfaceMesh,
  CachedPartFrameState,
  CadPart,
  DeformationScalePreset,
  DestructionCache,
  DestructionCacheFrame,
  DestructionFieldMode,
  DestructionSimConfig,
  DestructionTimelineEvent,
  FractureShardGeometry,
  MaterialSpec,
} from '../types/cad';
import { getMaterialById } from './materials';

export const DEFORMATION_SCALE_PRESETS: readonly DeformationScalePreset[] = [
  1, 5, 20,
] as const;

export interface FieldLegendDescriptor {
  mode: DestructionFieldMode;
  shortLabel: string;
  titleRu: string;
  unit: string;
  minValue: number;
  midValue: number;
  maxValue: number;
  cssGradient: string;
  yieldMarkerPct: number | null;
  yieldLabel: string | null;
  yieldExceeded: boolean;
}

function clamp01(v: number): number {
  if (!Number.isFinite(v)) return 0;
  return v < 0 ? 0 : v > 1 ? 1 : v;
}

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

function sampleStops(
  t: number,
  stops: readonly (readonly [number, number, number, number])[]
): [number, number, number] {
  const s = clamp01(t);
  if (s <= stops[0][0]) return [stops[0][1], stops[0][2], stops[0][3]];
  for (let i = 0; i < stops.length - 1; i++) {
    const a = stops[i];
    const b = stops[i + 1];
    if (s <= b[0]) {
      const span = Math.max(1e-6, b[0] - a[0]);
      const u = (s - a[0]) / span;
      return [lerp(a[1], b[1], u), lerp(a[2], b[2], u), lerp(a[3], b[3], u)];
    }
  }
  const last = stops[stops.length - 1];
  return [last[1], last[2], last[3]];
}

// Engineering Jet colormap for Von Mises stress σ_vM (deep blue -> cyan -> emerald -> yellow -> red)
const VON_MISES_STOPS: readonly (readonly [number, number, number, number])[] = [
  [0.0, 0.11, 0.22, 0.56],
  [0.25, 0.04, 0.68, 0.86],
  [0.5, 0.08, 0.78, 0.45],
  [0.75, 0.96, 0.8, 0.1],
  [1.0, 0.94, 0.18, 0.15],
];

// High-contrast Plasma-inspired colormap for equivalent plastic strain ε_p:
// Elastic zero zone stays cool slate (#2c3648) so permanently dented plastic zones pop vividly.
const PLASTIC_STRAIN_STOPS: readonly (readonly [
  number,
  number,
  number,
  number,
])[] = [
  [0.0, 0.19, 0.24, 0.31],
  [0.04, 0.32, 0.18, 0.56],
  [0.3, 0.62, 0.14, 0.65],
  [0.65, 0.92, 0.36, 0.24],
  [1.0, 0.99, 0.88, 0.24],
];

// Continuum damage D ∈ [0, 1] colormap: intact slate -> crack initiation amber -> critical crimson
const DAMAGE_STOPS: readonly (readonly [number, number, number, number])[] = [
  [0.0, 0.22, 0.27, 0.34],
  [0.18, 0.36, 0.42, 0.5],
  [0.45, 0.92, 0.62, 0.1],
  [0.75, 0.96, 0.34, 0.09],
  [1.0, 0.86, 0.08, 0.16],
];

export function evalVonMisesColormap(
  stressMPa: number,
  maxStressMPa: number
): [number, number, number] {
  const denom = Math.max(1e-3, maxStressMPa);
  return sampleStops(stressMPa / denom, VON_MISES_STOPS);
}

export function evalPlasticStrainColormap(
  plasticStrain: number,
  maxPlasticStrain: number
): [number, number, number] {
  if (!Number.isFinite(plasticStrain) || plasticStrain <= 1e-6) {
    return [
      PLASTIC_STRAIN_STOPS[0][1],
      PLASTIC_STRAIN_STOPS[0][2],
      PLASTIC_STRAIN_STOPS[0][3],
    ];
  }
  const denom = Math.max(1e-4, maxPlasticStrain);
  // Slight gamma lift (0.78) makes incipient plastic denting clearly visible before rupture
  const norm = Math.pow(clamp01(plasticStrain / denom), 0.78);
  return sampleStops(Math.max(0.04, norm), PLASTIC_STRAIN_STOPS);
}

export function evalDamageColormap(damage01: number): [number, number, number] {
  return sampleStops(damage01, DAMAGE_STOPS);
}

export function computeYieldMarkerPct(
  yieldStrengthMPa: number,
  scaleMaxMPa: number
): { pct: number; exceeded: boolean } {
  const safeMax = Math.max(1, scaleMaxMPa);
  const safeYield = Math.max(1, yieldStrengthMPa);
  const rawPct = (safeYield / safeMax) * 100;
  return {
    pct: Math.max(4, Math.min(96, rawPct)),
    exceeded: safeMax >= safeYield,
  };
}

export function buildFieldLegend(
  mode: DestructionFieldMode,
  maxStressMPa: number,
  yieldStrengthMPa: number,
  maxPlasticStrainPct: number,
  maxDamage: number
): FieldLegendDescriptor {
  if (mode === 'von_mises') {
    const scaleMax = Math.max(yieldStrengthMPa * 1.15, maxStressMPa, 50);
    const { pct, exceeded } = computeYieldMarkerPct(yieldStrengthMPa, scaleMax);
    return {
      mode,
      shortLabel: 'σ_vM',
      titleRu: 'Напряжения Von Mises (σ_vM)',
      unit: 'МПа',
      minValue: 0,
      midValue: scaleMax * 0.5,
      maxValue: scaleMax,
      cssGradient:
        'linear-gradient(to right, #1c388f 0%, #0aadde 25%, #14c773 50%, #f5cc1a 75%, #f02e26 100%)',
      yieldMarkerPct: pct,
      yieldLabel: `σy ${Math.round(yieldStrengthMPa)}`,
      yieldExceeded: exceeded,
    };
  }

  if (mode === 'plastic_strain') {
    const scaleMaxPct = Math.max(1.0, maxPlasticStrainPct);
    return {
      mode,
      shortLabel: 'ε_p',
      titleRu: 'Пластическая деформация (ε_p)',
      unit: '%',
      minValue: 0,
      midValue: scaleMaxPct * 0.5,
      maxValue: scaleMaxPct,
      cssGradient:
        'linear-gradient(to right, #303d4f 0%, #522e8f 6%, #9e24a6 32%, #eb5c3d 68%, #fce03d 100%)',
      yieldMarkerPct: null,
      yieldLabel: null,
      yieldExceeded: false,
    };
  }

  if (mode === 'damage') {
    const scaleMax = Math.max(1.0, Math.min(1.0, Math.max(maxDamage, 1.0)));
    return {
      mode,
      shortLabel: 'D',
      titleRu: 'Повреждаемость (D)',
      unit: '0..1',
      minValue: 0,
      midValue: 0.5,
      maxValue: scaleMax,
      cssGradient:
        'linear-gradient(to right, #384557 0%, #5c6b80 18%, #eb9e1a 45%, #f55717 75%, #db1429 100%)',
      yieldMarkerPct: null,
      yieldLabel: null,
      yieldExceeded: false,
    };
  }

  return {
    mode: 'shaded',
    shortLabel: 'Shaded',
    titleRu: 'Shaded (Реальная деформация)',
    unit: 'мм',
    minValue: 0,
    midValue: 0,
    maxValue: 0,
    cssGradient:
      'linear-gradient(to right, #475569 0%, #94a3b8 50%, #cbd5e1 100%)',
    yieldMarkerPct: null,
    yieldLabel: null,
    yieldExceeded: false,
  };
}

/**
 * Pure zero-GC deformation helper: out[i] = base[i] + displacements[i] * scale.
 * Safely clamps non-finite entries and falls back to base positions when displacements are absent.
 */
export function applyScaledDeformation(
  baseVertices: Float32Array,
  displacements: Float32Array | undefined,
  scale: number,
  outVertices: Float32Array
): boolean {
  const len = Math.min(baseVertices.length, outVertices.length);
  if (!displacements || displacements.length < len || scale === 0) {
    outVertices.set(baseVertices.subarray(0, len));
    return false;
  }

  let moved = false;
  for (let i = 0; i < len; i++) {
    const d = displacements[i];
    if (d !== 0 && Number.isFinite(d)) {
      outVertices[i] = baseVertices[i] + d * scale;
      moved = true;
    } else {
      outVertices[i] = baseVertices[i];
    }
  }
  return moved;
}

export interface RigidTransform3D {
  /** Reference centroid X_bar */
  refCentroid: [number, number, number];
  /** Deformed centroid x_bar */
  defCentroid: [number, number, number];
  /** Proper 3x3 rotation matrix R in SO(3) (row-major, det(R) = +1) mapping (X - X_bar) -> (x - x_bar) */
  rotMat: [
    number, number, number,
    number, number, number,
    number, number, number,
  ];
  /** Rotation angle in radians [0, pi] */
  angleRad: number;
}

/**
 * Exact Horn (1987) closed-form 3D rigid-motion fit via 4x4 symmetric quaternion eigenproblem.
 * Guarantees a proper rotation matrix R in SO(3) (det(R) = +1) with no singularity even at 180 deg.
 */
export function fitRigidTransformHorn3D(
  baseVertices: Float32Array,
  displacements: Float32Array
): RigidTransform3D {
  const vCount = Math.floor(Math.min(baseVertices.length, displacements.length) / 3);
  if (vCount < 3) {
    return {
      refCentroid: [0, 0, 0],
      defCentroid: [0, 0, 0],
      rotMat: [1, 0, 0, 0, 1, 0, 0, 0, 1],
      angleRad: 0,
    };
  }

  let rx = 0, ry = 0, rz = 0;
  let dx = 0, dy = 0, dz = 0;
  let valid = 0;
  for (let i = 0; i < vCount; i++) {
    const o = i * 3;
    const bx = baseVertices[o], by = baseVertices[o + 1], bz = baseVertices[o + 2];
    const ux = displacements[o], uy = displacements[o + 1], uz = displacements[o + 2];
    if (
      Number.isFinite(bx) && Number.isFinite(by) && Number.isFinite(bz) &&
      Number.isFinite(ux) && Number.isFinite(uy) && Number.isFinite(uz)
    ) {
      rx += bx; ry += by; rz += bz;
      dx += bx + ux; dy += by + uy; dz += bz + uz;
      valid++;
    }
  }
  if (valid < 3) {
    return {
      refCentroid: [0, 0, 0],
      defCentroid: [0, 0, 0],
      rotMat: [1, 0, 0, 0, 1, 0, 0, 0, 1],
      angleRad: 0,
    };
  }

  const invN = 1 / valid;
  rx *= invN; ry *= invN; rz *= invN;
  dx *= invN; dy *= invN; dz *= invN;

  // Cross-covariance S = sum (X_i - X_bar)(x_i - x_bar)^T
  let sxx = 0, sxy = 0, sxz = 0;
  let syx = 0, syy = 0, syz = 0;
  let szx = 0, szy = 0, szz = 0;
  for (let i = 0; i < vCount; i++) {
    const o = i * 3;
    const bx = baseVertices[o], by = baseVertices[o + 1], bz = baseVertices[o + 2];
    const ux = displacements[o], uy = displacements[o + 1], uz = displacements[o + 2];
    if (
      !Number.isFinite(bx) || !Number.isFinite(by) || !Number.isFinite(bz) ||
      !Number.isFinite(ux) || !Number.isFinite(uy) || !Number.isFinite(uz)
    ) {
      continue;
    }
    const px = bx - rx, py = by - ry, pz = bz - rz;
    const qx = bx + ux - dx, qy = by + uy - dy, qz = bz + uz - dz;
    sxx += px * qx; sxy += px * qy; sxz += px * qz;
    syx += py * qx; syy += py * qy; syz += py * qz;
    szx += pz * qx; szy += pz * qy; szz += pz * qz;
  }

  // Symmetric 4x4 Davenport/Horn matrix N
  const a = new Float64Array([
    sxx + syy + szz, syz - szy, szx - sxz, sxy - syx,
    syz - szy, sxx - syy - szz, sxy + syx, szx + sxz,
    szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy,
    sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz,
  ]);
  const v = new Float64Array([
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ]);

  // Cyclic Jacobi eigensolver on 4x4 symmetric matrix (converges to machine precision in <= 12 sweeps)
  for (let sweep = 0; sweep < 16; sweep++) {
    let maxOff = 0;
    for (let p = 0; p < 3; p++) {
      for (let q = p + 1; q < 4; q++) {
        const apq = a[p * 4 + q];
        const absApq = Math.abs(apq);
        if (absApq > maxOff) maxOff = absApq;
        if (absApq < 1e-15) continue;

        const app = a[p * 4 + p];
        const aqq = a[q * 4 + q];
        const tau = (aqq - app) / (2 * apq);
        const t =
          tau >= 0
            ? 1 / (tau + Math.sqrt(1 + tau * tau))
            : -1 / (-tau + Math.sqrt(1 + tau * tau));
        const c = 1 / Math.sqrt(1 + t * t);
        const s = t * c;

        a[p * 4 + p] = app - t * apq;
        a[q * 4 + q] = aqq + t * apq;
        a[p * 4 + q] = 0;
        a[q * 4 + p] = 0;

        for (let r = 0; r < 4; r++) {
          if (r === p || r === q) continue;
          const arp = a[r * 4 + p];
          const arq = a[r * 4 + q];
          const newRp = c * arp - s * arq;
          const newRq = s * arp + c * arq;
          a[r * 4 + p] = newRp;
          a[p * 4 + r] = newRp;
          a[r * 4 + q] = newRq;
          a[q * 4 + r] = newRq;
        }
        for (let r = 0; r < 4; r++) {
          const vrp = v[r * 4 + p];
          const vrq = v[r * 4 + q];
          v[r * 4 + p] = c * vrp - s * vrq;
          v[r * 4 + q] = s * vrp + c * vrq;
        }
      }
    }
    if (maxOff < 1e-14) break;
  }

  let bestCol = 0;
  let bestVal = a[0];
  for (let c = 1; c < 4; c++) {
    if (a[c * 4 + c] > bestVal) {
      bestVal = a[c * 4 + c];
      bestCol = c;
    }
  }

  let qw = v[bestCol];
  let qx = v[4 + bestCol];
  let qy = v[8 + bestCol];
  let qz = v[12 + bestCol];
  const qNorm = Math.hypot(qw, qx, qy, qz);
  if (qNorm > 1e-12) {
    qw /= qNorm; qx /= qNorm; qy /= qNorm; qz /= qNorm;
    if (qw < 0) {
      qw = -qw; qx = -qx; qy = -qy; qz = -qz;
    }
  } else {
    qw = 1; qx = 0; qy = 0; qz = 0;
  }

  const rotMat: RigidTransform3D['rotMat'] = [
    1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
    2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
    2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy),
  ];
  const angleRad = 2 * Math.acos(Math.min(1, Math.max(-1, qw)));

  return {
    refCentroid: [rx, ry, rz],
    defCentroid: [dx, dy, dz],
    rotMat,
    angleRad,
  };
}

/**
 * Rigid-motion-aware deformation scaling for finite-rotation FEM visualization.
 *
 * - For `scale === 1`: outputs exact deformed coordinates `X + u`.
 * - For `scale !== 1` on rotating/translating bodies: separates the optimal SO(3) rigid-body
 *   pose `x_rigid = x_bar + R (X - X_bar)` from true structural strain `u_strain = x - x_rigid`,
 *   and outputs `x_rigid + scale * u_strain`. Rotating gears/shafts never bloat or inflate at 5x/20x.
 * - For small-displacement static bending/crushing without significant rigid rotation, preserves
 *   classic `X + scale * u` so clamped boundaries stay anchored.
 */
export function applyRigidAwareScaledDeformation(
  baseVertices: Float32Array,
  displacements: Float32Array | undefined,
  scale: number,
  outVertices: Float32Array
): boolean {
  const len = Math.min(baseVertices.length, outVertices.length);
  if (!displacements || displacements.length < len || scale === 0) {
    outVertices.set(baseVertices.subarray(0, len));
    return false;
  }
  if (Math.abs(scale - 1) < 1e-9) {
    return applyScaledDeformation(baseVertices, displacements, 1, outVertices);
  }

  const vCount = Math.floor(len / 3);
  if (vCount < 4) {
    return applyScaledDeformation(baseVertices, displacements, scale, outVertices);
  }

  const fit = fitRigidTransformHorn3D(baseVertices, displacements);
  const [rx, ry, rz] = fit.refCentroid;
  const [dx, dy, dz] = fit.defCentroid;
  const [r00, r01, r02, r10, r11, r12, r20, r21, r22] = fit.rotMat;

  let rigidSqSum = 0;
  let strainSqSum = 0;
  for (let i = 0; i < vCount; i++) {
    const o = i * 3;
    const bx = baseVertices[o], by = baseVertices[o + 1], bz = baseVertices[o + 2];
    const ux = displacements[o], uy = displacements[o + 1], uz = displacements[o + 2];
    if (!Number.isFinite(ux) || !Number.isFinite(uy) || !Number.isFinite(uz)) continue;
    const px = bx - rx, py = by - ry, pz = bz - rz;
    const xRigX = dx + r00 * px + r01 * py + r02 * pz;
    const xRigY = dy + r10 * px + r11 * py + r12 * pz;
    const xRigZ = dz + r20 * px + r21 * py + r22 * pz;
    const urx = xRigX - bx, ury = xRigY - by, urz = xRigZ - bz;
    const usx = bx + ux - xRigX, usy = by + uy - xRigY, usz = bz + uz - xRigZ;
    rigidSqSum += urx * urx + ury * ury + urz * urz;
    strainSqSum += usx * usx + usy * usy + usz * usz;
  }

  const useRigidDecomposition =
    fit.angleRad > 0.015 || rigidSqSum > 4.0 * strainSqSum + 1e-16;
  if (!useRigidDecomposition) {
    return applyScaledDeformation(baseVertices, displacements, scale, outVertices);
  }

  let moved = false;
  for (let i = 0; i < vCount; i++) {
    const o = i * 3;
    const bx = baseVertices[o], by = baseVertices[o + 1], bz = baseVertices[o + 2];
    const ux = displacements[o], uy = displacements[o + 1], uz = displacements[o + 2];
    if (
      !Number.isFinite(ux) || !Number.isFinite(uy) || !Number.isFinite(uz) ||
      (ux === 0 && uy === 0 && uz === 0 && fit.angleRad === 0)
    ) {
      outVertices[o] = bx;
      outVertices[o + 1] = by;
      outVertices[o + 2] = bz;
      continue;
    }
    const px = bx - rx, py = by - ry, pz = bz - rz;
    const xRigX = dx + r00 * px + r01 * py + r02 * pz;
    const xRigY = dy + r10 * px + r11 * py + r12 * pz;
    const xRigZ = dz + r20 * px + r21 * py + r22 * pz;
    const usx = bx + ux - xRigX;
    const usy = by + uy - xRigY;
    const usz = bz + uz - xRigZ;
    outVertices[o] = xRigX + usx * scale;
    outVertices[o + 1] = xRigY + usy * scale;
    outVertices[o + 2] = xRigZ + usz * scale;
    moved = true;
  }
  return moved;
}

/**
 * Populates a packed RGB Float32Array (length = vertexCount * 3) in-place for the selected field mode.
 */
export function fillVertexFieldColors(
  mode: Exclude<DestructionFieldMode, 'shaded'>,
  vertexCount: number,
  nodalStressMPa: Float32Array | undefined,
  nodalPlasticStrain: Float32Array | undefined,
  nodalDamage: Float32Array | undefined,
  stressScaleMaxMPa: number,
  plasticScaleMax: number,
  outRgb: Float32Array
): void {
  const count = Math.min(vertexCount, Math.floor(outRgb.length / 3));
  for (let i = 0; i < count; i++) {
    let rgb: [number, number, number];
    if (mode === 'von_mises') {
      const s = nodalStressMPa && i < nodalStressMPa.length ? nodalStressMPa[i] : 0;
      rgb = evalVonMisesColormap(s, stressScaleMaxMPa);
    } else if (mode === 'plastic_strain') {
      const ep =
        nodalPlasticStrain && i < nodalPlasticStrain.length
          ? nodalPlasticStrain[i]
          : 0;
      rgb = evalPlasticStrainColormap(ep, plasticScaleMax);
    } else {
      const d = nodalDamage && i < nodalDamage.length ? nodalDamage[i] : 0;
      rgb = evalDamageColormap(d);
    }
    const o = i * 3;
    outRgb[o] = rgb[0];
    outRgb[o + 1] = rgb[1];
    outRgb[o + 2] = rgb[2];
  }
}

export interface EnergyBalanceSeries {
  kineticPoints: string;
  elasticPoints: string;
  plasticPoints: string;
  fracturePoints: string;
  maxEnergyJ: number;
}

/**
 * Builds normalized SVG polyline coordinate strings for the 4 physical energy channels:
 * E_k (kinetic), E_e (elastic strain), E_p (plastic dissipation), E_f (fracture).
 */
export function buildEnergyBalanceSeries(
  frames: readonly DestructionCacheFrame[],
  width = 100,
  height = 48,
  padY = 3
): EnergyBalanceSeries {
  if (frames.length < 2) {
    return {
      kineticPoints: '',
      elasticPoints: '',
      plasticPoints: '',
      fracturePoints: '',
      maxEnergyJ: 1,
    };
  }

  let maxE = 1e-4;
  for (let i = 0; i < frames.length; i++) {
    const m = frames[i].metrics;
    if (m.kineticEnergyJ > maxE) maxE = m.kineticEnergyJ;
    if (m.elasticEnergyJ > maxE) maxE = m.elasticEnergyJ;
    if (m.plasticEnergyJ > maxE) maxE = m.plasticEnergyJ;
    if (m.fractureEnergyJ > maxE) maxE = m.fractureEnergyJ;
  }

  const usableH = Math.max(1, height - padY * 2);
  const total = frames.length - 1;
  const kPts: string[] = [];
  const ePts: string[] = [];
  const pPts: string[] = [];
  const fPts: string[] = [];

  for (let i = 0; i < frames.length; i++) {
    const x = ((i / total) * width).toFixed(2);
    const m = frames[i].metrics;
    const toY = (val: number) =>
      (height - padY - clamp01(val / maxE) * usableH).toFixed(2);

    kPts.push(`${x},${toY(m.kineticEnergyJ)}`);
    ePts.push(`${x},${toY(m.elasticEnergyJ)}`);
    pPts.push(`${x},${toY(m.plasticEnergyJ)}`);
    fPts.push(`${x},${toY(m.fractureEnergyJ)}`);
  }

  return {
    kineticPoints: kPts.join(' '),
    elasticPoints: ePts.join(' '),
    plasticPoints: pPts.join(' '),
    fracturePoints: fPts.join(' '),
    maxEnergyJ: maxE,
  };
}

export type RustVec3List = number[] | [number, number, number][];

/**
 * Local-space exterior triangle mesh of a detached FEM fragment (`P5-T15`),
 * centered at the fragment center of mass in meters.
 */
export interface RustCoupledSimFragmentMeshJson {
  fragment_id: string;
  source_part_id?: string;
  parent_part_id?: string;
  vertices_m: RustVec3List;
  normals?: RustVec3List;
  indices: number[];
  bounding_radius_m?: number;
  vertex_count?: number;
  triangle_count?: number;
}

/**
 * Raw JSON structures returned by `crates/mcp-cad-server/src/tools/coupled_sim.rs` (`P5-T6` / `P5-T15`).
 */
export interface RustCoupledSimPartFrameJson {
  part_id: string;
  node_count?: number;
  deformed_positions?: RustVec3List;
  displacements_mm?: RustVec3List;
  nodal_von_mises_mpa?: number[];
  nodal_equiv_plastic_strain?: number[];
  nodal_damage?: number[];
  /** Compacted exterior active FEM surface vertex positions in world meters (`P5-T15`) */
  surface_positions_m?: RustVec3List;
  /** Compacted exterior active FEM surface reference positions in world meters (`P5-T15`) */
  surface_reference_positions_m?: RustVec3List;
  /** Outward CCW triangle indices into `surface_positions_m` (`P5-T15`) */
  surface_indices?: number[];
  /** Mapping from compacted surface vertex index -> full volume FEM node index (`P5-T15`) */
  surface_node_indices?: number[];
  surface_vertex_count?: number;
  surface_triangle_count?: number;
  eroded_element_indices?: number[];
  eroded_element_count?: number;
  active_element_count?: number;
  max_displacement_mm?: number;
  max_von_mises_mpa?: number;
  max_equiv_plastic_strain?: number;
  max_damage?: number;
}

export interface RustCoupledSimFragmentJson {
  fragment_id: string;
  source_part_id?: string;
  parent_part_id?: string;
  position?: [number, number, number];
  position_m?: [number, number, number];
  orientation_quat?: [number, number, number, number];
  quaternion?: [number, number, number, number];
  linear_velocity_m_s?: [number, number, number];
  linear_velocity?: [number, number, number];
  angular_velocity_rad_s?: [number, number, number];
  mass_kg?: number;
  bounding_radius_m?: number;
  bounding_radius?: number;
  detached_at_time_s?: number;
  detachment_step?: number;
  element_count?: number;
  /** Optional inline local-space FEM triangle mesh (`P5-T15`) */
  mesh?: RustCoupledSimFragmentMeshJson;
}

export interface RustCoupledSimFrameJson {
  simulation_id: string;
  frame_index: number;
  total_frames: number;
  time_s: number;
  dt_s?: number;
  dt_used_s?: number;
  length_unit?: string;
  units?: { length?: string };
  parts: RustCoupledSimPartFrameJson[];
  detached_fragments?: RustCoupledSimFragmentJson[];
  contact_forces?: {
    pair_id?: string;
    body_a?: string;
    body_b?: string;
    normal_force_n: number;
    tangential_force_n?: number;
    point_m?: [number, number, number];
    contact_point?: [number, number, number];
  }[];
  energy_balance?: {
    kinetic_j?: number;
    elastic_j?: number;
    elastic_strain_j?: number;
    plastic_j?: number;
    plastic_dissipation_j?: number;
    fracture_j?: number;
    fracture_energy_j?: number;
    contact_work_j?: number;
    external_work_j?: number;
    total_energy_j?: number;
    total_mechanical_j?: number;
  };
}

export interface RustCoupledSimEventJson {
  frame_index?: number;
  time_s: number;
  kind?: string;
  event_type?: string;
  part_id?: string;
  detail?: string;
  description?: string;
  value?: number;
}

export interface RustCoupledSimBakeSummaryJson {
  simulation_id: string;
  accuracy_level?: string;
  duration_s?: number;
  total_frames?: number;
  frame_count?: number;
  bake_time_ms?: number;
  elapsed_ms?: number;
  events?: RustCoupledSimEventJson[];
  /** Cached detached fragment triangle meshes fetched once via `/api/coupled-sim/fragments/:sim_id` (`P5-T15`) */
  fragment_meshes?: RustCoupledSimFragmentMeshJson[];
}

function mapRustEventType(
  rawType: string
): DestructionTimelineEvent['type'] {
  const lower = rawType.toLowerCase();
  if (lower.includes('yield') || lower.includes('plastic')) return 'yield_onset';
  if (lower.includes('fragment') || lower.includes('detach')) {
    return 'fragment_detachment';
  }
  if (lower.includes('tooth') || lower.includes('erod') || lower.includes('crack')) {
    return 'tooth_shear';
  }
  if (lower.includes('fracture') || lower.includes('rupture')) {
    return 'body_fracture';
  }
  if (lower.includes('floor') || lower.includes('bounce')) return 'floor_bounce';
  return 'contact';
}

function flattenVec3List(
  input: RustVec3List | undefined
): Float32Array | undefined {
  if (!input || input.length === 0) return undefined;
  const first = input[0];
  if (Array.isArray(first)) {
    const arr = input as [number, number, number][];
    const out = new Float32Array(arr.length * 3);
    for (let i = 0; i < arr.length; i++) {
      const p = arr[i];
      out[i * 3] = Number.isFinite(p[0]) ? p[0] : 0;
      out[i * 3 + 1] = Number.isFinite(p[1]) ? p[1] : 0;
      out[i * 3 + 2] = Number.isFinite(p[2]) ? p[2] : 0;
    }
    return out;
  }
  const flat = input as number[];
  const out = new Float32Array(flat.length);
  for (let i = 0; i < flat.length; i++) {
    out[i] = Number.isFinite(flat[i]) ? flat[i] : 0;
  }
  return out;
}

function computeSmoothVertexNormals(
  vertices: Float32Array,
  indices: Uint32Array
): Float32Array {
  const normals = new Float32Array(vertices.length);
  const vCount = Math.floor(vertices.length / 3);
  for (let i = 0; i + 2 < indices.length; i += 3) {
    const i0 = indices[i];
    const i1 = indices[i + 1];
    const i2 = indices[i + 2];
    if (i0 >= vCount || i1 >= vCount || i2 >= vCount) continue;
    const ax = vertices[i0 * 3];
    const ay = vertices[i0 * 3 + 1];
    const az = vertices[i0 * 3 + 2];
    const bx = vertices[i1 * 3] - ax;
    const by = vertices[i1 * 3 + 1] - ay;
    const bz = vertices[i1 * 3 + 2] - az;
    const cx = vertices[i2 * 3] - ax;
    const cy = vertices[i2 * 3 + 1] - ay;
    const cz = vertices[i2 * 3 + 2] - az;
    const nx = by * cz - bz * cy;
    const ny = bz * cx - bx * cz;
    const nz = bx * cy - by * cx;
    normals[i0 * 3] += nx;
    normals[i0 * 3 + 1] += ny;
    normals[i0 * 3 + 2] += nz;
    normals[i1 * 3] += nx;
    normals[i1 * 3 + 1] += ny;
    normals[i1 * 3 + 2] += nz;
    normals[i2 * 3] += nx;
    normals[i2 * 3 + 1] += ny;
    normals[i2 * 3 + 2] += nz;
  }
  for (let i = 0; i < vCount; i++) {
    const o = i * 3;
    const nx = normals[o];
    const ny = normals[o + 1];
    const nz = normals[o + 2];
    const len = Math.hypot(nx, ny, nz);
    if (len > 1e-12) {
      normals[o] = nx / len;
      normals[o + 1] = ny / len;
      normals[o + 2] = nz / len;
    } else {
      normals[o] = 0;
      normals[o + 1] = 1;
      normals[o + 2] = 0;
    }
  }
  return normals;
}

function gatherSurfaceScalars(
  volumeField: number[] | undefined,
  surfaceNodeIndices: number[] | undefined,
  surfaceVertexCount: number
): Float32Array | undefined {
  if (!volumeField || volumeField.length === 0 || surfaceVertexCount <= 0) {
    return undefined;
  }
  if (surfaceNodeIndices && surfaceNodeIndices.length === surfaceVertexCount) {
    const out = new Float32Array(surfaceVertexCount);
    for (let i = 0; i < surfaceVertexCount; i++) {
      const nid = surfaceNodeIndices[i];
      const val = nid >= 0 && nid < volumeField.length ? volumeField[nid] : 0;
      out[i] = Number.isFinite(val) ? val : 0;
    }
    return out;
  }
  if (volumeField.length === surfaceVertexCount) {
    return new Float32Array(volumeField);
  }
  return undefined;
}

function buildFragmentGeometryFromRustMesh(
  mesh: RustCoupledSimFragmentMeshJson,
  fallbackParentPartId: string,
  mat: MaterialSpec,
  mass: number
): FractureShardGeometry | null {
  const vertices = flattenVec3List(mesh.vertices_m);
  if (!vertices || vertices.length < 9 || !mesh.indices || mesh.indices.length < 3) {
    return null;
  }
  const indices = new Uint32Array(mesh.indices);
  let normals = flattenVec3List(mesh.normals);
  if (!normals || normals.length !== vertices.length) {
    normals = computeSmoothVertexNormals(vertices, indices);
  }
  let radius = mesh.bounding_radius_m ?? 0;
  if (!(radius > 0)) {
    for (let i = 0; i < vertices.length; i += 3) {
      const d = Math.hypot(vertices[i], vertices[i + 1], vertices[i + 2]);
      if (d > radius) radius = d;
    }
  }
  return {
    id: mesh.fragment_id,
    parentPartId: mesh.source_part_id || mesh.parent_part_id || fallbackParentPartId,
    materialId: mat.id,
    colorHex: mat.colorHex,
    roughness: mat.roughness,
    metalness: mat.metalness,
    localCom: [0, 0, 0],
    boundingRadius: Math.max(1e-4, radius),
    mass: Math.max(1e-4, mass),
    distToContact: 0,
    vertices,
    normals,
    indices,
    innerVertices: new Float32Array(0),
    innerNormals: new Float32Array(0),
    innerIndices: new Uint32Array(0),
  };
}

function buildSmallFragmentBox(
  id: string,
  parentPartId: string,
  mat: MaterialSpec,
  radius: number,
  mass: number
): FractureShardGeometry {
  const r = Math.max(0.0015, radius);
  const v = new Float32Array([
    -r, -r, r, r, -r, r, r, r, r, -r, r, r,
    -r, -r, -r, -r, r, -r, r, r, -r, r, -r, -r,
  ]);
  const n = new Float32Array([
    0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
    0, 0, -1, 0, 0, -1, 0, 0, -1, 0, 0, -1,
  ]);
  const idx = new Uint32Array([
    0, 1, 2, 0, 2, 3,
    4, 5, 6, 4, 6, 7,
    3, 2, 6, 3, 6, 5,
    0, 4, 7, 0, 7, 1,
    1, 7, 6, 1, 6, 2,
    0, 3, 5, 0, 5, 4,
  ]);
  return {
    id,
    parentPartId,
    materialId: mat.id,
    colorHex: mat.colorHex,
    roughness: mat.roughness,
    metalness: mat.metalness,
    localCom: [0, 0, 0],
    boundingRadius: r,
    mass: Math.max(1e-4, mass),
    distToContact: 0,
    vertices: v,
    normals: n,
    indices: idx,
    innerVertices: new Float32Array(0),
    innerNormals: new Float32Array(0),
    innerIndices: new Uint32Array(0),
  };
}

/**
 * Pure adapter transforming native Rust `mcp-cad-server` coupled HHT-α simulation frames
 * (`GET /api/coupled-sim/frames/:sim_id/:frame_index?include_mesh_fields=true&length_unit=m`
 * + cached fragment meshes from `GET /api/coupled-sim/fragments/:sim_id`)
 * into the frontend `DestructionCache` consumed by `CadViewport3D`, `CadDestructionTimeline`,
 * and `CadDestructionPanel`.
 */
export function convertRustCoupledSimToDestructionCache(
  summary: RustCoupledSimBakeSummaryJson,
  rustFrames: readonly RustCoupledSimFrameJson[],
  parts: readonly CadPart[],
  materials: readonly MaterialSpec[],
  config: DestructionSimConfig
): DestructionCache {
  const partById = new Map<string, CadPart>();
  for (const p of parts) partById.set(p.id, p);

  // Index cached detached fragment triangle meshes (transferred once per simulation or inline)
  const cachedFragMeshById = new Map<string, RustCoupledSimFragmentMeshJson>();
  for (const fm of summary.fragment_meshes ?? []) {
    if (fm?.fragment_id) {
      cachedFragMeshById.set(fm.fragment_id, fm);
    }
  }
  for (const rf of rustFrames) {
    for (const frag of rf.detached_fragments ?? []) {
      if (frag.mesh && !cachedFragMeshById.has(frag.fragment_id)) {
        cachedFragMeshById.set(frag.fragment_id, frag.mesh);
      }
    }
  }

  // Collect all unique detached fragments across the Rust timeline using real FEM triangle geometry
  const fragIndexById = new Map<string, number>();
  const shards: FractureShardGeometry[] = [];
  for (const rf of rustFrames) {
    for (const frag of rf.detached_fragments ?? []) {
      if (!fragIndexById.has(frag.fragment_id)) {
        const parentPartId =
          frag.source_part_id || frag.parent_part_id || 'unknown_part';
        const parent = partById.get(parentPartId);
        const mat = getMaterialById(
          parent?.materialId ?? 'steel_4140',
          materials as MaterialSpec[]
        );
        fragIndexById.set(frag.fragment_id, shards.length);
        const realMesh = frag.mesh ?? cachedFragMeshById.get(frag.fragment_id);
        const builtReal = realMesh
          ? buildFragmentGeometryFromRustMesh(
              realMesh,
              parentPartId,
              mat,
              frag.mass_kg ?? 0.004
            )
          : null;
        shards.push(
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
  }

  let yieldRefMPa = Infinity;
  for (const p of parts) {
    if (!p.visible || p.isFixed) continue;
    const mat = getMaterialById(p.materialId, materials as MaterialSpec[]);
    const yMPa = mat.yieldStrength / 1e6;
    if (yMPa > 0 && yMPa < yieldRefMPa) yieldRefMPa = yMPa;
  }
  if (!Number.isFinite(yieldRefMPa)) yieldRefMPa = 350;

  let maxStressOverallMPa = 0;
  let maxPlasticOverallPct = 0;
  let maxDamageOverall = 0;
  let maxEnergyOverallJ = 1e-4;
  let contactPoint: [number, number, number] = [0, 0, 0];

  const frames: DestructionCacheFrame[] = rustFrames.map((rf, idx) => {
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

    // Seed defaults for all scene parts so static benches remain visible
    for (const p of parts) {
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
      const part = partById.get(rp.part_id);
      const mat = getMaterialById(
        part?.materialId ?? 'steel_4140',
        materials as MaterialSpec[]
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

      // P5-T15: Build active exterior FEM surface mesh directly in world meters
      // (independent of CadPart.mesh vertex count, reflecting eroded/detached cavities)
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
          // Derive reference positions from displacements_mm if reference array was omitted
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

      // Legacy fallback: convert mm displacements to meters if lengths match frontend CAD mesh
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

    const shardPos = new Float32Array(shards.length * 3);
    const shardQuat = new Float32Array(shards.length * 4);
    const shardActive = new Uint8Array(shards.length);
    const shardStress = new Float32Array(shards.length);
    const shardStressMPa = new Float32Array(shards.length);
    const shardPlasticStrain = new Float32Array(shards.length);
    const shardDamage = new Float32Array(shards.length);
    let activeShardsCount = 0;

    for (const frag of rf.detached_fragments ?? []) {
      const sIdx = fragIndexById.get(frag.fragment_id);
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
      if (pt) contactPoint = [...pt];
    }

    const eb = rf.energy_balance;
    const kineticJ = eb?.kinetic_j ?? 0;
    const elasticJ = eb?.elastic_j ?? eb?.elastic_strain_j ?? 0;
    const plasticJ = eb?.plastic_j ?? eb?.plastic_dissipation_j ?? 0;
    const fractureJ = eb?.fracture_j ?? eb?.fracture_energy_j ?? 0;

    if (framePeakStress > maxStressOverallMPa) maxStressOverallMPa = framePeakStress;
    if (frameMaxPlasticPct > maxPlasticOverallPct) {
      maxPlasticOverallPct = frameMaxPlasticPct;
    }
    if (frameMaxDamage > maxDamageOverall) maxDamageOverall = frameMaxDamage;
    maxEnergyOverallJ = Math.max(
      maxEnergyOverallJ,
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
          : framePeakStress >= yieldRefMPa
            ? 'Пластическая текучесть'
            : contactForceN > 10
              ? 'Упругий контакт'
              : 'Динамический расчёт HHT-α';

    return {
      frameIndex: rf.frame_index ?? idx,
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
        yieldLimitMPa: Number(yieldRefMPa.toFixed(0)),
        contactForceN: Number(contactForceN.toFixed(0)),
        kineticEnergyJ: Number(kineticJ.toFixed(4)),
        elasticEnergyJ: Number(elasticJ.toFixed(4)),
        plasticEnergyJ: Number(plasticJ.toFixed(4)),
        fractureEnergyJ: Number(fractureJ.toFixed(4)),
        maxPlasticStrainPct: Number(frameMaxPlasticPct.toFixed(2)),
        maxDamage: Number(frameMaxDamage.toFixed(3)),
        maxDisplacementMm: Number(frameMaxDispMm.toFixed(3)),
        activeShardsCount,
        totalShardsCount: shards.length,
        fracturedPartsCount,
        intactPartsCount,
        statusLabel,
      },
    };
  });

  const durationSec =
    summary.duration_s ??
    (frames.length > 0 ? frames[frames.length - 1].timeSec : config.durationSec);
  const totalFrames = Math.max(0, frames.length - 1);
  const fps =
    durationSec > 1e-6 && totalFrames > 0
      ? Math.max(10, Math.round(totalFrames / durationSec))
      : config.fps;

  const events: DestructionTimelineEvent[] = (summary.events ?? []).map((e) => {
    const rawType = e.kind ?? e.event_type ?? 'contact';
    return {
      frameIndex:
        e.frame_index ??
        Math.max(
          0,
          Math.min(
            totalFrames,
            Math.round((e.time_s / Math.max(1e-6, durationSec)) * totalFrames)
          )
        ),
      timeSec: e.time_s,
      type: mapRustEventType(rawType),
      label: e.detail || e.description || rawType,
      partId: e.part_id,
      stressMPa: e.value,
    };
  });

  let floorY = -0.05;
  for (const p of parts) {
    if (!p.visible) continue;
    const minY = p.position[1] + p.mesh.boundingBox.min[1] * Math.abs(p.scale[1] || 1);
    if (minY < floorY) floorY = minY;
  }

  let cacheBytes = 0;
  for (const s of shards) {
    cacheBytes +=
      s.vertices.byteLength + s.normals.byteLength + s.indices.byteLength;
  }
  for (const f of frames) {
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
    id: `rust_${summary.simulation_id}`,
    createdAt: new Date().toISOString(),
    solverBackend: 'rust_fem_f64',
    solverLabel: 'Rust FEM f64',
    rustSimulationId: summary.simulation_id,
    config: { ...config, durationSec, fps },
    fps,
    totalFrames,
    durationSec,
    floorY,
    contactPoint,
    bakeTimeMs: summary.elapsed_ms ?? summary.bake_time_ms ?? 1,
    cacheBytes: Math.max(256, cacheBytes),
    shards,
    frames,
    events,
    maxStressOverallMPa: Math.max(1, Number(maxStressOverallMPa.toFixed(1))),
    yieldStrengthRefMPa: Number(yieldRefMPa.toFixed(0)),
    maxPlasticStrainOverallPct: Math.max(
      0.1,
      Number(maxPlasticOverallPct.toFixed(2))
    ),
    maxDamageOverall: Number(Math.max(0.01, maxDamageOverall).toFixed(3)),
    maxEnergyOverallJ: Number(maxEnergyOverallJ.toFixed(4)),
  };
}
