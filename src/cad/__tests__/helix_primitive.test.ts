import test from 'node:test';
import assert from 'node:assert/strict';

import {
  createHelixMesh,
  evaluateFeatureTree,
  validatePartMesh,
} from '../primitives';
import {
  executeMcpToolOnEngineState,
  type EngineStateSnapshot,
} from '../mcp_dispatcher';
import { DEFAULT_MATERIALS } from '../materials';
import { optimizeCadFeatureTree } from '../query_optimizer';
import type { DagNode } from '../../types/cad';

function makeFreshState(): EngineStateSnapshot {
  return {
    parts: [],
    joints: [],
    materials: DEFAULT_MATERIALS.map((m) => ({ ...m })),
    selectedPartId: '',
    quality: 'standard',
    feaResult: null,
    physicsFrames: [],
    renderMode: 'shaded',
  };
}

test('createHelixMesh generates watertight manifold genus-0 coil spring for right- and left-hand', () => {
  // User's reference coil spring: R = 17.5 mm, r = 2.5 mm, pitch = 12 mm, height = 60 mm (5 turns)
  const rHelix = 0.0175;
  const rWire = 0.0025;
  const pitch = 0.012;
  const turns = 5;
  const height = pitch * turns;

  const wireLength = turns * Math.hypot(2 * Math.PI * rHelix, pitch);
  const expectedVolume = Math.PI * rWire * rWire * wireLength;

  for (const isRightHand of [true, false]) {
    const mesh = createHelixMesh(
      rHelix,
      rWire,
      pitch,
      height,
      1,
      isRightHand,
      'standard',
      7850
    );

    const report = validatePartMesh('coil_spring', mesh, 0, true);
    assert.equal(report.status, 'PASS', `rightHand=${isRightHand} issues: ${JSON.stringify(report.issues)}`);
    assert.equal(report.isWatertight, true, `rightHand=${isRightHand} must be watertight`);
    assert.equal(report.isManifold, true, `rightHand=${isRightHand} must be manifold`);
    assert.equal(report.isOutwardOriented, true, `rightHand=${isRightHand} must have outward normals`);
    assert.equal(report.boundaryEdgesCount, 0, 'must have 0 boundary edges');
    assert.equal(report.nonManifoldEdgesCount, 0, 'must have 0 non-manifold edges');
    assert.equal(report.degenerateTrianglesCount, 0, 'must have 0 degenerate triangles');
    assert.equal(report.eulerGenus, 0, 'single-start helical coil must have genus 0');
    assert.ok(mesh.triangleCount > 2000, 'mesh must have smooth tessellation');

    const relErr = Math.abs(mesh.volume - expectedVolume) / expectedVolume;
    assert.ok(
      relErr < 0.04,
      `volume ${mesh.volume} differs from analytical ${expectedVolume} by ${(relErr * 100).toFixed(2)}%`
    );

    const spanX = mesh.boundingBox.max[0] - mesh.boundingBox.min[0];
    const spanY = mesh.boundingBox.max[1] - mesh.boundingBox.min[1];
    const spanZ = mesh.boundingBox.max[2] - mesh.boundingBox.min[2];
    const expectedDiameter = 2 * (rHelix + rWire);
    assert.ok(
      Math.abs(spanX - expectedDiameter) < 0.001,
      `spanX=${spanX} expected ~${expectedDiameter}`
    );
    assert.ok(
      Math.abs(spanZ - expectedDiameter) < 0.001,
      `spanZ=${spanZ} expected ~${expectedDiameter}`
    );
    assert.ok(spanY >= height && spanY <= height + 2 * rWire + 1e-4, `spanY=${spanY}`);
  }
});

test('createHelixMesh rejects self-intersecting and non-positive helix invariants', () => {
  // wire_radius >= helix_radius
  assert.throws(
    () => createHelixMesh(0.01, 0.01, 0.03, 0.06),
    /strictly smaller than helix_radius/
  );
  // pitch <= 2 * wire_radius (adjacent coils collide)
  assert.throws(
    () => createHelixMesh(0.0175, 0.006, 0.012, 0.06),
    /exceed wire diameter/
  );
  // double-start with pitch / 2 <= 2 * wire_radius
  assert.throws(
    () => createHelixMesh(0.0175, 0.003, 0.012, 0.06, 2),
    /exceed wire diameter/
  );
  // non-positive parameters
  assert.throws(
    () => createHelixMesh(0, 0.0025, 0.012, 0.06),
    /positive and finite/
  );
});

test('evaluateFeatureTree supports helix parameter aliases and turns calculation', () => {
  const dagByTurns: DagNode[] = [
    {
      id: 'n1',
      name: 'Coil Spring',
      type: 'helix',
      params: {
        mean_diameter: 0.035,
        wire_diameter: 0.005,
        pitch: 0.012,
        turns: 5,
      },
      enabled: true,
    },
  ];
  const dagByHeight: DagNode[] = [
    {
      id: 'n1',
      name: 'Coil Spring',
      type: 'helix',
      params: {
        helix_radius: 0.0175,
        wire_radius: 0.0025,
        height: 0.06,
        active_coils: 5,
      },
      enabled: true,
    },
  ];

  const m1 = evaluateFeatureTree(dagByTurns, 'standard', 7850);
  const m2 = evaluateFeatureTree(dagByHeight, 'standard', 7850);
  assert.equal(m1.isWatertight, true);
  assert.equal(m2.isWatertight, true);
  assert.ok(
    Math.abs(m1.volume - m2.volume) / m1.volume < 1e-6,
    'equivalent alias specifications must produce identical volumes'
  );
});

test('MCP dispatcher supports add_feature_primitive (helix & coil_spring), modify_feature, and optimizer', () => {
  let state = makeFreshState();

  // 1. Add using 'coil_spring' alias
  const addRes = executeMcpToolOnEngineState(
    state,
    'add_feature_primitive',
    {
      part_id: 'spring_main',
      primitive_type: 'coil_spring',
      params: {
        helix_radius: 0.0175,
        wire_radius: 0.0025,
        pitch: 0.012,
        turns: 5,
      },
    }
  );
  state = addRes.nextState;

  assert.equal(addRes.response.status, 'ok');
  assert.equal(addRes.response.primitive_type, 'helix');
  assert.equal(addRes.response.is_watertight, true);
  assert.equal(addRes.response.euler_genus, 0);

  const volBefore = addRes.response.volume_m3 as number;
  const featureId = addRes.response.feature_id as string;

  // 2. Parametrically modify turns from 5 -> 8 via modify_feature
  const modRes = executeMcpToolOnEngineState(
    state,
    'modify_feature',
    {
      part_id: 'spring_main',
      feature_id: featureId,
      params: { turns: 8 },
    }
  );
  state = modRes.nextState;

  assert.equal(modRes.response.status, 'modified');
  const part = state.parts.find((p) => p.id === 'spring_main')!;
  assert.equal(part.mesh.isWatertight, true);
  const ratio = part.mesh.volume / volBefore;
  assert.ok(
    Math.abs(ratio - 8 / 5) < 0.03,
    `Expected volume ratio ~1.6 when turns changes 5->8, got ${ratio.toFixed(4)}`
  );

  // 3. Optimizer handles helix cleanly
  const optReport = optimizeCadFeatureTree(
    part.id,
    part.name,
    part.dag,
    'standard',
    7850
  );
  assert.equal(optReport.invariantCheck.equivalenceVerified, true);
  assert.ok(
    optReport.bestPlan.explainTree.some((line) =>
      line.includes('AnalyticalHelixSweep')
    )
  );
});
