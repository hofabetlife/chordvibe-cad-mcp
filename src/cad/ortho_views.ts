export type OrthoViewId = 'top' | 'front' | 'right';

export type Vec3Tuple = [number, number, number];

export interface OrthoViewAxisConfig {
  /** Exact cardinal unit vector from target toward the orthographic camera. */
  direction: Vec3Tuple;
  /** Exact cardinal unit up-vector strictly orthogonal to `direction`. */
  up: Vec3Tuple;
}

export interface OrthoCameraPose {
  position: Vec3Tuple;
  target: Vec3Tuple;
  up: Vec3Tuple;
  left: number;
  right: number;
  top: number;
  bottom: number;
  near: number;
  far: number;
}

/**
 * Exact cardinal axis definitions for the three orthographic CAD views.
 * Keeping every off-axis component strictly 0 guarantees zero angular tilt
 * when projecting onto the XZ (top), XY (front), and YZ (right) planes.
 */
export const ORTHO_VIEW_CONFIGS: Record<OrthoViewId, OrthoViewAxisConfig> = {
  top: {
    direction: [0, 1, 0],
    up: [0, 0, -1],
  },
  front: {
    direction: [0, 0, 1],
    up: [0, 1, 0],
  },
  right: {
    direction: [1, 0, 0],
    up: [0, 1, 0],
  },
};

/**
 * Computes a strictly axis-aligned orthographic camera pose centered on `target`.
 * In orthographic projection, camera standoff distance does not affect scale,
 * so we keep a generous distance (`>= 10m`) to prevent near-plane clipping when
 * zooming closely into large assemblies.
 */
export function computeOrthoCameraPose(
  view: OrthoViewId,
  target: Vec3Tuple,
  radius: number,
  aspect: number
): OrthoCameraPose {
  const { direction, up } = ORTHO_VIEW_CONFIGS[view];
  const safeRadius = Math.max(1e-4, radius);
  const safeAspect = Math.max(1e-4, aspect);
  const frustumSize = safeRadius * 1.1;
  const distance = Math.max(safeRadius * 2, 10);

  return {
    position: [
      target[0] + direction[0] * distance,
      target[1] + direction[1] * distance,
      target[2] + direction[2] * distance,
    ],
    target: [target[0], target[1], target[2]],
    up: [up[0], up[1], up[2]],
    left: (-frustumSize * safeAspect) / 2,
    right: (frustumSize * safeAspect) / 2,
    top: frustumSize / 2,
    bottom: -frustumSize / 2,
    near: 0.001,
    far: distance * 2 + 50,
  };
}
