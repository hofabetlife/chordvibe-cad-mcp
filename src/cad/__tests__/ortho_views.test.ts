import test from 'node:test';
import assert from 'node:assert/strict';
import {
  ORTHO_VIEW_CONFIGS,
  computeOrthoCameraPose,
  OrthoViewId,
  Vec3Tuple,
} from '../ortho_views';

function dot(a: Vec3Tuple, b: Vec3Tuple): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a: Vec3Tuple, b: Vec3Tuple): Vec3Tuple {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function length(v: Vec3Tuple): number {
  return Math.hypot(v[0], v[1], v[2]);
}

function projectToOrthoScreen(
  point: Vec3Tuple,
  view: OrthoViewId,
  target: Vec3Tuple,
  radius: number,
  aspect: number
): [number, number] {
  const pose = computeOrthoCameraPose(view, target, radius, aspect);
  const offset: Vec3Tuple = [
    pose.position[0] - pose.target[0],
    pose.position[1] - pose.target[1],
    pose.position[2] - pose.target[2],
  ];
  const dist = length(offset);
  const forward: Vec3Tuple = [
    -offset[0] / dist,
    -offset[1] / dist,
    -offset[2] / dist,
  ];
  const right = cross(forward, pose.up);
  const rel: Vec3Tuple = [
    point[0] - pose.target[0],
    point[1] - pose.target[1],
    point[2] - pose.target[2],
  ];
  return [dot(rel, right), dot(rel, pose.up)];
}

test('orthographic view directions are exact cardinal axes with zero tilt', () => {
  assert.deepEqual(ORTHO_VIEW_CONFIGS.front.direction, [0, 0, 1]);
  assert.deepEqual(ORTHO_VIEW_CONFIGS.front.up, [0, 1, 0]);

  assert.deepEqual(ORTHO_VIEW_CONFIGS.right.direction, [1, 0, 0]);
  assert.deepEqual(ORTHO_VIEW_CONFIGS.right.up, [0, 1, 0]);

  assert.deepEqual(ORTHO_VIEW_CONFIGS.top.direction, [0, 1, 0]);
  assert.deepEqual(ORTHO_VIEW_CONFIGS.top.up, [0, 0, -1]);

  for (const id of ['top', 'front', 'right'] as OrthoViewId[]) {
    const cfg = ORTHO_VIEW_CONFIGS[id];
    assert.equal(length(cfg.direction), 1);
    assert.equal(length(cfg.up), 1);
    assert.equal(dot(cfg.direction, cfg.up), 0);
  }
});

test('front (XY) view projects points along Z onto identical XY screen coordinates', () => {
  const target: Vec3Tuple = [0.015, 0.032, -0.02];
  const pFront: Vec3Tuple = [0.04, 0.05, 0.065];
  const pBack: Vec3Tuple = [0.04, 0.05, -0.065];

  const s1 = projectToOrthoScreen(pFront, 'front', target, 0.15, 1.6);
  const s2 = projectToOrthoScreen(pBack, 'front', target, 0.15, 1.6);

  assert.ok(Math.abs(s1[0] - s2[0]) < 1e-12);
  assert.ok(Math.abs(s1[1] - s2[1]) < 1e-12);
});

test('right (YZ) and top (XZ) views have zero angular drift along their depth axes', () => {
  const target: Vec3Tuple = [-0.01, 0.025, 0.03];

  const r1 = projectToOrthoScreen([0.1, 0.04, -0.02], 'right', target, 0.2, 1.5);
  const r2 = projectToOrthoScreen([-0.1, 0.04, -0.02], 'right', target, 0.2, 1.5);
  assert.ok(Math.abs(r1[0] - r2[0]) < 1e-12);
  assert.ok(Math.abs(r1[1] - r2[1]) < 1e-12);

  const t1 = projectToOrthoScreen([0.03, 0.12, 0.05], 'top', target, 0.2, 1.5);
  const t2 = projectToOrthoScreen([0.03, -0.12, 0.05], 'top', target, 0.2, 1.5);
  assert.ok(Math.abs(t1[0] - t2[0]) < 1e-12);
  assert.ok(Math.abs(t1[1] - t2[1]) < 1e-12);
});

test('close orthographic zoom never clips large parts with the near plane', () => {
  const pose = computeOrthoCameraPose('front', [0, 0, 0], 0.015, 1.6);
  const standoff = pose.position[2] - pose.target[2];
  assert.ok(standoff >= 10, `expected standoff >= 10m, got ${standoff}`);
  // A 1m-deep assembly (±0.5m) stays strictly within [near, far]
  assert.ok(standoff - 0.5 > pose.near);
  assert.ok(standoff + 0.5 < pose.far);
});
