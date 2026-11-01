import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import {
  applyHolesToMesh,
  applyTransformToMesh,
  createBoxMesh,
  createConeMesh,
  createCylinderMesh,
  createPolygonPrismMesh,
  createRevolveMesh,
  createSphereMesh,
  createSpurGearMesh,
  createWheelRimMesh,
  exportMeshToStlBinary,
  parseStlBuffer,
} from './primitives';
import { runLinearFeaSolver } from './fea_solver';
import { stepKinematicsAssembly } from './kinematics';
import { DEFAULT_MATERIALS } from './materials';
import {
  EngineStateSnapshot,
  executeMcpToolOnEngineState,
} from './mcp_dispatcher';
import {
  AI_UNOPTIMIZED_SCENARIOS,
  optimizeCadFeatureTree,
} from './query_optimizer';
import {
  GOLDEN_WINDOWS_MAPPINGS,
  getWindowsWgpuArtifactUrl,
} from './native_bridge';

describe('MCP-CAD Core Geometry & Topological Verification Suite', () => {
  it('verifies Z-axis parity with Rust mcp-cad-core and Windows WGPU golden artifacts', () => {
    // 1. Hex Prism Chamfer along Z-axis (z in [-0.009, +0.009])
    const outerR = 0.024 / Math.sqrt(3);
    const hexChamfer = createPolygonPrismMesh(6, outerR, 0.018, 0.002, 7850);
    assert.equal(hexChamfer.isWatertight, true);
    assert.equal(hexChamfer.boundaryEdgesCount, 0);
    assert.ok(Math.abs(hexChamfer.boundingBox.min[2] - -0.009) < 1e-4);
    assert.ok(Math.abs(hexChamfer.boundingBox.max[2] - 0.009) < 1e-4);

    // 2. Partial Revolve 180 deg closed rectangular annular profile around Z-axis
    const rectProfile: [number, number][] = [
      [0.01, -0.02],
      [0.03, -0.02],
      [0.03, 0.02],
      [0.01, 0.02],
    ];
    const rev180 = createRevolveMesh(rectProfile, 180, 'standard', 7850);
    assert.equal(rev180.isWatertight, true);
    assert.equal(rev180.boundaryEdgesCount, 0);
    assert.equal(rev180.isOutwardOriented, true);
    assert.ok(Math.abs(rev180.boundingBox.min[2] - -0.02) < 1e-4);
    assert.ok(Math.abs(rev180.boundingBox.max[2] - 0.02) < 1e-4);

    // 3. 5-Spoke Wheel Rim with 5 bolt through-holes (genus 11, Z-axis width 0.18 -> [-0.09, +0.09])
    const rim = createWheelRimMesh(
      0.4,
      0.18,
      5,
      0.12,
      0.06,
      5,
      0.09,
      0.014,
      0.022,
      0.025
    );
    assert.equal(rim.isWatertight, true);
    assert.equal(rim.boundaryEdgesCount, 0);
    assert.equal(rim.eulerGenus, 11);
    assert.ok(Math.abs(rim.boundingBox.min[2] - -0.09) < 1e-3);
    assert.ok(Math.abs(rim.boundingBox.max[2] - 0.09) < 1e-3);

    // 4. Unified Bridge mappings for Windows WGPU artifacts and compiled STLs
    assert.equal(
      getWindowsWgpuArtifactUrl('spur_gear', 'shaded'),
      '/qa/artifacts/spur_gear_hub_keyway_shaded.png'
    );
    assert.equal(
      getWindowsWgpuArtifactUrl('wheel_rim', 'normals'),
      '/qa/artifacts/wheel_rim_5spoke_normals.png'
    );
    assert.equal(
      GOLDEN_WINDOWS_MAPPINGS['spur_gear'].compiledStlUrl,
      '/output/spur_gear.stl'
    );
  });

  it('computes exact Box volume, surface area, 6 planar faces, and watertight genus 0', () => {
    const w = 0.1,
      h = 0.05,
      d = 0.04;
    const mesh = createBoxMesh(w, h, d, 7850);
    const expectedVol = w * h * d;
    const expectedArea = 2 * (w * h + h * d + w * d);

    assert.ok(Math.abs(mesh.volume - expectedVol) / expectedVol < 1e-4);
    assert.ok(Math.abs(mesh.surfaceArea - expectedArea) / expectedArea < 1e-4);
    assert.equal(mesh.isWatertight, true);
    assert.equal(mesh.boundaryEdgesCount, 0);
    assert.equal(mesh.nonManifoldEdgesCount, 0);
    assert.equal(mesh.eulerGenus, 0);
    assert.equal(mesh.detectedPlanarFaces.length, 6);
  });

  it('computes watertight Cylinder, Sphere, and Cone within analytical tolerances', () => {
    const cyl = createCylinderMesh(0.03, 0.08, 'high', 7850);
    const expectedCylVol = Math.PI * 0.03 * 0.03 * 0.08;
    assert.ok(Math.abs(cyl.volume - expectedCylVol) / expectedCylVol < 0.005);
    assert.equal(cyl.isWatertight, true);
    assert.equal(cyl.eulerGenus, 0);

    const sph = createSphereMesh(0.04, 'high', 7850);
    const expectedSphVol = (4 / 3) * Math.PI * Math.pow(0.04, 3);
    assert.ok(Math.abs(sph.volume - expectedSphVol) / expectedSphVol < 0.015);
    assert.equal(sph.isWatertight, true);
    assert.equal(sph.eulerGenus, 0);

    const cone = createConeMesh(0.03, 0.01, 0.06, 'high', 7850);
    const expectedConeVol =
      (Math.PI * 0.06 * (0.03 * 0.03 + 0.03 * 0.01 + 0.01 * 0.01)) / 3;
    assert.ok(Math.abs(cone.volume - expectedConeVol) / expectedConeVol < 0.01);
    assert.equal(cone.isWatertight, true);
    assert.equal(cone.eulerGenus, 0);
  });

  it('validates DIN 934 chamfered Polygon Prism & M10 Nut (BUG-005)', () => {
    assert.throws(() => createPolygonPrismMesh(2, 0.01, 0.01, 0.001));

    const hexSolid = createPolygonPrismMesh(6, 0.0098, 0.008, 0.0008, 7850);
    assert.equal(hexSolid.isWatertight, true);
    assert.equal(hexSolid.boundaryEdgesCount, 0);
    assert.equal(hexSolid.eulerGenus, 0);

    const hexNut = createPolygonPrismMesh(
      6,
      0.0098,
      0.008,
      0.0008,
      7850,
      0.0042,
      0.0015,
      0.0008
    );
    assert.equal(hexNut.isWatertight, true);
    assert.equal(hexNut.boundaryEdgesCount, 0);
    assert.equal(hexNut.eulerGenus, 1);
  });

  it('constructs watertight Involute Spur Gear with hub, bore, and keyway (BUG-002)', () => {
    const gear = createSpurGearMesh(
      20,
      2.0,
      20,
      0.015,
      0.012,
      0.024,
      0.008,
      0.004,
      0.002,
      7850
    );
    assert.equal(gear.isWatertight, true);
    assert.equal(gear.boundaryEdgesCount, 0);
    assert.equal(gear.nonManifoldEdgesCount, 0);
    assert.equal(gear.eulerGenus, 1);
    assert.ok(gear.volume > 1e-6);
  });

  it('constructs watertight Partial Revolve (180 deg) with consistent outward normals (BUG-003)', () => {
    const profile: [number, number][] = [
      [0.015, -0.02],
      [0.025, 0.0],
      [0.018, 0.02],
    ];
    const rev180 = createRevolveMesh(profile, 180, 'standard', 7850);
    const rev360 = createRevolveMesh(profile, 360, 'standard', 7850);

    assert.equal(rev180.isWatertight, true);
    assert.equal(rev180.boundaryEdgesCount, 0);
    assert.equal(rev180.inconsistentEdgesCount, 0);
    assert.equal(rev180.isOutwardOriented, true);
    assert.equal(rev180.eulerGenus, 0);
    assert.ok(Math.abs(rev180.volume * 2 - rev360.volume) / rev360.volume < 0.02);
  });

  it('constructs watertight 5-spoke Wheel Rim (BUG-004)', () => {
    const rim = createWheelRimMesh(0.4572, 0.2159, 5, 0.15, 0.065, 5);
    assert.equal(rim.isWatertight, true);
    assert.equal(rim.boundaryEdgesCount, 0);
    assert.equal(rim.nonManifoldEdgesCount, 0);
    assert.equal(rim.eulerGenus, 11);
  });

  it('cuts chained through-holes into a Box and increments Euler genus to 2 (BUG-006)', () => {
    const baseBox = createBoxMesh(0.1, 0.04, 0.02, 7850);
    const perforated = applyHolesToMesh(baseBox, [
      {
        position: [-0.025, 0, 0],
        normal: [0, 0, -1],
        radius: 0.008,
        depth: 0.025,
      },
      {
        position: [0.025, 0, 0],
        normal: [0, 0, -1],
        radius: 0.008,
        depth: 0.025,
      },
    ]);
    assert.equal(perforated.isWatertight, true);
    assert.equal(perforated.boundaryEdgesCount, 0);
    assert.equal(perforated.eulerGenus, 2);
    assert.ok(perforated.volume < baseBox.volume);
  });

  it('reverses triangle winding on negative scale transform so signed volume remains outward (BUG-007)', () => {
    const cyl = createCylinderMesh(0.02, 0.05, 'standard', 7850);
    const mirrored = applyTransformToMesh(
      cyl,
      [0, 0, 0],
      [0, 0, 0],
      [-1, 1, 1],
      7850
    );
    assert.equal(mirrored.isOutwardOriented, true);
    assert.equal(mirrored.isWatertight, true);
    assert.ok(Math.abs(mirrored.volume - cyl.volume) / cyl.volume < 1e-4);
  });

  it('round-trips Binary STL export and import cleanly', () => {
    const box = createBoxMesh(0.06, 0.04, 0.02, 7850);
    const buf = exportMeshToStlBinary(box, 'test_box');
    const parsed = parseStlBuffer(buf, 7850);
    assert.equal(parsed.triangleCount, box.triangleCount);
    assert.equal(parsed.isWatertight, true);
    assert.ok(Math.abs(parsed.volume - box.volume) / box.volume < 1e-4);
  });

  it('solves Cantilever Beam FEA deflection and von Mises stress accurately', () => {
    const beamMesh = createBoxMesh(0.2, 0.02, 0.02, 7850);
    const steel = DEFAULT_MATERIALS[0];
    const part = {
      id: 'beam',
      name: 'Cantilever Beam',
      materialId: steel.id,
      dag: [],
      mesh: beamMesh,
      visible: true,
      position: [0, 0, 0] as [number, number, number],
      rotation: [0, 0, 0] as [number, number, number],
      scale: [1, 1, 1] as [number, number, number],
    };
    const { result } = runLinearFeaSolver(part, steel, [
      { region: 'min_x', type: 'fixed' },
      { region: 'max_x', type: 'force', forceVector: [0, -1000, 0] },
    ]);
    assert.ok(result.maxDisplacementMm > 0.9 && result.maxDisplacementMm < 1.15);
    assert.ok(result.maxVonMisesMPa > 120 && result.maxVonMisesMPa < 180);
    assert.ok(result.safetyFactor > 2.0);
  });

  it('enforces exact Gear Mesh kinematic velocity ratio w2/w1 = -0.5', () => {
    const g1Mesh = createSpurGearMesh(20, 2.0, 20, 0.015, 0.01, 0.02, 0.005, 0, 0);
    const g2Mesh = createSpurGearMesh(40, 2.0, 20, 0.015, 0.012, 0.028, 0.005, 0, 0);
    const parts = [
      {
        id: 'gear_1',
        name: 'Pinion Z=20',
        materialId: 'steel_1045',
        dag: [{ id: 'g1', name: 'G1', type: 'spur_gear' as const, params: { teeth: 20 }, enabled: true }],
        mesh: g1Mesh,
        visible: true,
        position: [0, 0, 0] as [number, number, number],
        rotation: [0, 0, 0] as [number, number, number],
        scale: [1, 1, 1] as [number, number, number],
      },
      {
        id: 'gear_2',
        name: 'Gear Z=40',
        materialId: 'steel_1045',
        dag: [{ id: 'g2', name: 'G2', type: 'spur_gear' as const, params: { teeth: 40 }, enabled: true }],
        mesh: g2Mesh,
        visible: true,
        position: [0.06, 0, 0] as [number, number, number],
        rotation: [0, 0, 0] as [number, number, number],
        scale: [1, 1, 1] as [number, number, number],
      },
    ];
    const joints = [
      {
        id: 'j1',
        name: 'Drive',
        type: 'revolute' as const,
        partA: 'world',
        partB: 'gear_1',
        anchor: [0, 0, 0] as [number, number, number],
        axis: [0, 0, 1] as [number, number, number],
        motorSpeedRpm: 6.283185, // rad/s
        motorTorqueNm: 50,
        motorEnabled: true,
        currentAngleRad: 0,
      },
    ];

    const stepped = stepKinematicsAssembly(parts, joints, 0.1);
    const w1 = stepped.angularVelocities['gear_1'][2];
    const w2 = stepped.angularVelocities['gear_2'][2];
    assert.ok(Math.abs(w2 / w1 - -0.5) < 1e-5);
  });

  it('enforces transactional rollback on modify_feature and batch_cad_operations (BUG-008, BUG-009, BUG-010)', () => {
    const initialState: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const s1 = executeMcpToolOnEngineState(initialState, 'add_feature_primitive', {
      part_id: 'hex_part',
      primitive_type: 'polygon_prism',
      params: { sides: 6, outer_radius: 0.01, height: 0.01, chamfer_radius: 0.001 },
    }).nextState;

    assert.equal(s1.parts.length, 1);

    // BUG-008: modify_feature with invalid sides=2 must throw and preserve s1 untouched
    assert.throws(() =>
      executeMcpToolOnEngineState(s1, 'modify_feature', {
        part_id: 'hex_part',
        node_id: 'node_root',
        params: { sides: 2 },
      })
    );
    assert.equal(s1.parts[0].dag[0].params.sides, 6);

    // BUG-009: batch_cad_operations with failing second step must roll back first step
    assert.throws(() =>
      executeMcpToolOnEngineState(s1, 'batch_cad_operations', {
        operations: [
          {
            tool: 'create_part',
            params: { part_id: 'temp_part', name: 'Should Rollback' },
          },
          {
            tool: 'add_feature_primitive',
            params: {
              part_id: 'temp_part',
              primitive_type: 'box',
              params: { width: -5, height: 1, depth: 1 },
            },
          },
        ],
      })
    );
    assert.equal(s1.parts.length, 1);

    // BUG-010: capture_viewport rejects 0x0 and open_preview_window rejects nonexistent part_id
    assert.throws(() =>
      executeMcpToolOnEngineState(s1, 'capture_viewport', {
        width: 0,
        height: 720,
      })
    );
    assert.throws(() =>
      executeMcpToolOnEngineState(s1, 'open_preview_window', {
        part_id: 'ghost_part_999',
      })
    );
  });

  it('optimizes coaxial 3D CSG bushing DAG into 1-pass 2D Annular Revolve via Cost-Based Optimizer', () => {
    const bushingScen = AI_UNOPTIMIZED_SCENARIOS[0];
    const report = optimizeCadFeatureTree(
      bushingScen.id,
      bushingScen.title,
      bushingScen.dag,
      'standard',
      bushingScen.density
    );

    assert.equal(report.originalNodeCount, 5);
    assert.equal(report.optimalNodeCount, 1);
    assert.equal(report.bestPlan.dag[0].type, 'revolve');
    assert.ok(report.costReductionPct > 70);
    assert.equal(report.bestPlan.costBreakdown.sliverRiskScore, 0);
    assert.equal(report.invariantCheck.watertightAfter, true);
    assert.equal(report.invariantCheck.genusBefore, 1);
    assert.equal(report.invariantCheck.genusAfter, 1);
    assert.equal(report.invariantCheck.equivalenceVerified, true);
  });

  it('applies De Morgan CSG batching, AABB out-of-bounds culling, and duplicate bore elimination on perforated plate', () => {
    const plateScen = AI_UNOPTIMIZED_SCENARIOS[1];
    const report = optimizeCadFeatureTree(
      plateScen.id,
      plateScen.title,
      plateScen.dag,
      'standard',
      plateScen.density
    );

    assert.equal(report.originalNodeCount, 8);
    // Remaining optimal nodes: 1 base box + 2 distinct in-bounds through-holes = 3 nodes
    assert.equal(report.optimalNodeCount, 3);
    assert.ok(report.costReductionPct > 65);
    assert.equal(report.invariantCheck.watertightAfter, true);
    assert.equal(report.invariantCheck.genusBefore, 2);
    assert.equal(report.invariantCheck.genusAfter, 2);
    assert.equal(report.invariantCheck.equivalenceVerified, true);
  });

  it('fuses PolygonPrism + Coaxial Bore + Metric Thread via Kernel Pushdown and executes optimize_feature_tree MCP tool', () => {
    const nutScen = AI_UNOPTIMIZED_SCENARIOS[2];
    const report = optimizeCadFeatureTree(
      nutScen.id,
      nutScen.title,
      nutScen.dag,
      'standard',
      nutScen.density
    );

    assert.equal(report.originalNodeCount, 5);
    assert.equal(report.optimalNodeCount, 1);
    assert.equal(report.bestPlan.dag[0].type, 'polygon_prism');
    assert.equal(report.invariantCheck.watertightAfter, true);
    assert.equal(report.invariantCheck.genusAfter, 1);

    // Verify MCP tool execution
    const initialState: EngineStateSnapshot = {
      parts: [
        {
          id: nutScen.id,
          name: nutScen.title,
          materialId: nutScen.materialId,
          dag: nutScen.dag,
          mesh: createPolygonPrismMesh(6, 0.0098, 0.008, 0.0012, 7850, 0.00425, 0.0015, 0.0008),
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        },
      ],
      selectedPartId: nutScen.id,
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const { nextState, response } = executeMcpToolOnEngineState(
      initialState,
      'optimize_feature_tree',
      { part_id: nutScen.id, apply_best_plan: true }
    );
    assert.equal(response.status, 'optimized_and_applied');
    assert.equal(nextState.parts[0].dag.length, 1);
    assert.equal(nextState.parts[0].mesh.isWatertight, true);
  });

  it('validates logical Engineering Workbenches and FEA quick load presets', async () => {
    const {
      ENGINEERING_WORKBENCHES,
      FEA_QUICK_LOAD_PRESETS,
      getWorkbenchMetadata,
      buildBoundaryConditionsFromPreset,
    } = await import('./workbenches');

    assert.equal(ENGINEERING_WORKBENCHES.length, 4);
    assert.deepEqual(
      ENGINEERING_WORKBENCHES.map((w) => w.id),
      ['modeling', 'fea', 'simulation', 'optimizer']
    );

    const feaMeta = getWorkbenchMetadata('fea');
    assert.equal(feaMeta.id, 'fea');
    assert.ok(feaMeta.label.includes('Нагрузки'));

    assert.equal(FEA_QUICK_LOAD_PRESETS.length, 4);
    const conds = buildBoundaryConditionsFromPreset('axial_tension');
    assert.equal(conds.length, 2);
    assert.equal(conds[0].type, 'fixed');
    assert.equal(conds[0].region, 'min_x');
    assert.equal(conds[1].type, 'force');
    assert.deepEqual(conds[1].forceVector, [2500, 0, 0]);
  });
});

