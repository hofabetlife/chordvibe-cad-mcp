import test from 'node:test';
import assert from 'node:assert/strict';
import {
  mapLiveToolsToMcpHistory,
  mapRustDagNodesToWebStudio,
  mapRustFeaSummaryToWebStudio,
  mapRustJointsToWebStudio,
  mapRustMaterialToWebStudio,
  sanitizeLivePartId,
  shouldAutoFitCamera,
} from '../live_sync_adapter';
import { DEFAULT_MATERIALS } from '../materials';

test('mapRustMaterialToWebStudio resolves canonical Rust aliases to built-in Web Studio materials', () => {
  const steel = mapRustMaterialToWebStudio(
    {
      name: 'steel_carbon_aisi1045',
      youngs_modulus_gpa: 205,
      poissons_ratio: 0.29,
      yield_strength_mpa: 350,
      density_kg_m3: 7850,
      failure_mode: 'ductile',
    },
    DEFAULT_MATERIALS
  );
  assert.equal(steel.materialId, 'steel_1045');
  assert.equal(steel.customMaterial, undefined);

  const alu = mapRustMaterialToWebStudio(
    {
      name: 'aluminum_6061_t6',
      youngs_modulus_gpa: 68.9,
      poissons_ratio: 0.33,
      yield_strength_mpa: 276,
      density_kg_m3: 2700,
      failure_mode: 'ductile',
    },
    DEFAULT_MATERIALS
  );
  assert.equal(alu.materialId, 'alu_6061');
  assert.equal(alu.customMaterial, undefined);

  const ti = mapRustMaterialToWebStudio(
    {
      name: 'titanium_grade5_ti6al4v',
      youngs_modulus_gpa: 114,
      poissons_ratio: 0.34,
      yield_strength_mpa: 880,
      density_kg_m3: 4430,
      failure_mode: 'ductile',
    },
    DEFAULT_MATERIALS
  );
  assert.equal(ti.materialId, 'ti_6al_4v');
});

test('mapRustMaterialToWebStudio builds a valid custom MaterialSpec for unknown alloys', () => {
  const custom = mapRustMaterialToWebStudio(
    {
      name: 'Inconel 718',
      youngs_modulus_gpa: 200,
      poissons_ratio: 0.29,
      yield_strength_mpa: 1035,
      density_kg_m3: 8190,
      failure_mode: 'ductile',
    },
    DEFAULT_MATERIALS
  );
  assert.equal(custom.materialId, 'inconel_718');
  assert.ok(custom.customMaterial);
  assert.equal(custom.customMaterial.density, 8190);
  assert.equal(custom.customMaterial.youngsModulus, 200e9);
  assert.equal(custom.customMaterial.yieldStrength, 1035e6);
  assert.equal(custom.customMaterial.failureMode, 'ductile');
});

test('mapRustDagNodesToWebStudio preserves parametric op types and safely falls back for empty or unknown ops', () => {
  const mapped = mapRustDagNodesToWebStudio([
    {
      id: 'f1',
      name: 'Helical Coil Spring',
      op_type: 'helix',
      enabled: true,
      params: { helix_radius: 0.0175, wire_radius: 0.0025, pitch: 0.012 },
    },
    {
      id: 'f2',
      name: 'Unknown Experimental Op',
      op_type: 'sweep_spline_v9',
      enabled: false,
      params: { steps: 12 },
    },
  ]);

  assert.equal(mapped.length, 2);
  assert.equal(mapped[0].type, 'helix');
  assert.equal(mapped[0].enabled, true);
  assert.equal(mapped[0].params.helix_radius, 0.0175);
  assert.equal(mapped[1].type, 'stl_import');
  assert.equal(mapped[1].enabled, false);

  const fallback = mapRustDagNodesToWebStudio([], 'Spring STL');
  assert.equal(fallback.length, 1);
  assert.equal(fallback[0].type, 'stl_import');
  assert.equal(fallback[0].name, 'Spring STL');
});

test('mapRustJointsToWebStudio converts rad/s motor velocity to RPM and preserves gear ratio', () => {
  const joints = mapRustJointsToWebStudio([
    {
      id: 'j_drive',
      part_a: 'world',
      part_b: 'gear_1',
      joint_type: 'revolute',
      anchor_a_m: [0, 0, 0],
      axis: [0, 0, 1],
      motor_target_velocity: Math.PI * 2, // 1 rev/sec = 60 RPM
      motor_max_force: 42,
    },
    {
      id: 'j_mesh',
      part_a: 'gear_1',
      part_b: 'gear_2',
      joint_type: 'gear',
      ratio: 0.5,
    },
  ]);

  assert.equal(joints.length, 2);
  assert.equal(joints[0].type, 'revolute');
  assert.equal(joints[0].motorSpeedRpm, 60);
  assert.equal(joints[0].motorTorqueNm, 42);
  assert.equal(joints[0].motorEnabled, true);
  assert.equal(joints[1].type, 'gear');
  assert.equal(joints[1].gearRatio, 0.5);
  assert.equal(joints[1].motorEnabled, false);
});

test('mapRustFeaSummaryToWebStudio maps verdict, element order, and staleness warning', () => {
  const safeRes = mapRustFeaSummaryToWebStudio('spring_1', {
    max_von_mises_mpa: 120,
    yield_strength_mpa: 350,
    safety_factor: 2.91,
    max_displacement_mm: 0.45,
    verdict: 'SAFE',
    node_count: 1450,
    element_count: 820,
    element_order: 2,
    is_stale: true,
  });

  assert.ok(safeRes);
  assert.equal(safeRes.status, 'SAFE');
  assert.equal(safeRes.elementOrder, 'TET10');
  assert.equal(safeRes.warnings?.length, 1);

  const failRes = mapRustFeaSummaryToWebStudio('bracket', {
    max_von_mises_mpa: 410,
    yield_strength_mpa: 350,
    safety_factor: 0.85,
    max_displacement_mm: 2.1,
    verdict: 'FAIL_YIELD',
    node_count: 900,
    element_count: 500,
    element_order: 1,
    is_stale: false,
  });
  assert.ok(failRes);
  assert.equal(failRes.status, 'FAILURE_YIELD_EXCEEDED');
  assert.equal(failRes.elementOrder, 'TET4');
});

test('mapLiveToolsToMcpHistory and shouldAutoFitCamera behave deterministically', () => {
  const history = mapLiveToolsToMcpHistory([
    {
      tool_name: 'add_feature_primitive',
      status: 'success',
      summary: 'Added helix to spring_1',
      timestamp_ms: 1700000000000,
      duration_ms: 14.37,
      params_preview: { part_id: 'spring_1', primitive_type: 'helix' },
    },
  ]);

  assert.equal(history.length, 1);
  assert.equal(history[0].toolName, 'add_feature_primitive');
  assert.equal(history[0].durationMs, 14.4);
  assert.equal(history[0].status, 'success');

  assert.equal(shouldAutoFitCamera(['piston_01'], ['spring_1'], false), true);
  assert.equal(shouldAutoFitCamera(['spring_1'], ['spring_1'], true), false);
  assert.equal(
    shouldAutoFitCamera(['spring_1', 'cap_1'], ['cap_1', 'spring_1'], true),
    false
  );
  assert.equal(
    shouldAutoFitCamera(['spring_1'], ['spring_1', 'cap_1'], true),
    true
  );
  assert.equal(sanitizeLivePartId('spring/1:test'), 'spring_1_test');
});
