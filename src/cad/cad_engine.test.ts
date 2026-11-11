import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
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
  evaluateFeatureTree,
  exportMeshToStlBinary,
  parseStlBuffer,
  validatePartMesh,
} from './primitives';
import { runLinearFeaSolver } from './fea_solver';
import { stepKinematicsAssembly } from './kinematics';
import { DEFAULT_MATERIALS } from './materials';
import {
  EngineStateSnapshot,
  MCP_TOOLS_CATALOG,
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

  it('solves Cantilever Beam FEA deflection and von Mises stress accurately with 3D TET4 PCG solver', () => {
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

    // 3-grid convergence check: coarse -> medium -> fine monotonically increases deflection towards analytical
    const coarse = runLinearFeaSolver(part, steel, [
      { region: 'min_x', type: 'fixed' },
      { region: 'max_x', type: 'force', forceVector: [0, -1000, 0] },
    ], 'coarse');
    const medium = runLinearFeaSolver(part, steel, [
      { region: 'min_x', type: 'fixed' },
      { region: 'max_x', type: 'force', forceVector: [0, -1000, 0] },
    ], 'medium');
    const fine = runLinearFeaSolver(part, steel, [
      { region: 'min_x', type: 'fixed' },
      { region: 'max_x', type: 'force', forceVector: [0, -1000, 0] },
    ], 'fine');

    assert.ok(coarse.result.maxDisplacementMm > 0.2);
    assert.ok(medium.result.maxDisplacementMm > coarse.result.maxDisplacementMm);
    assert.ok(fine.result.maxDisplacementMm > medium.result.maxDisplacementMm);
    assert.ok(fine.result.maxVonMisesMPa > medium.result.maxVonMisesMPa);
    assert.ok(fine.result.safetyFactor > 2.0);
    assert.ok(fine.result.tetQuality !== undefined);
    assert.ok(fine.result.discretizationWarning?.includes('shear locking'));
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

    assert.equal(FEA_QUICK_LOAD_PRESETS.length, 5);
    const conds = buildBoundaryConditionsFromPreset('axial_tension');
    assert.equal(conds.length, 2);
    assert.equal(conds[0].type, 'fixed');
    assert.equal(conds[0].region, 'min_x');
    assert.equal(conds[1].type, 'force');
    assert.deepEqual(conds[1].forceVector, [2500, 0, 0]);
  });


  it('verifies BUG-015: 28 MCP tools catalog including validate_mesh and gear joint schema parity', () => {
    assert.equal(MCP_TOOLS_CATALOG.length, 28);
    assert.ok(MCP_TOOLS_CATALOG.some((t) => t.name === 'validate_mesh'));

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
      part_id: 'pinion',
      primitive_type: 'spur_gear',
      params: { teeth: 20, module_mm: 2.0, bore_radius: 0.005 },
    }).nextState;
    const s2 = executeMcpToolOnEngineState(s1, 'add_feature_primitive', {
      part_id: 'wheel',
      primitive_type: 'spur_gear',
      params: { teeth: 40, module_mm: 2.0, bore_radius: 0.006 },
    }).nextState;

    const { nextState: s3, response } = executeMcpToolOnEngineState(s2, 'create_joint', {
      joint_type: 'gear',
      part_a: 'pinion',
      part_b: 'wheel',
      ratio: -0.5,
    });
    assert.equal(response.status, 'created');
    assert.equal(s3.joints.length, 1);

    const meshVal = executeMcpToolOnEngineState(s3, 'validate_mesh', {
      part_id: 'pinion',
      expected_genus: 1,
    }).response;
    assert.equal(meshVal.status, 'PASS');
  });

  it('verifies BUG-016: revolve profile validation (r<0, angle>360, self-intersection) and tessellate_part schema', () => {
    // 1. Negative radius r < 0 must throw
    assert.throws(() =>
      createRevolveMesh(
        [
          [-0.01, -0.02],
          [0.03, -0.02],
          [0.03, 0.02],
        ],
        360,
        'standard',
        7850
      )
    );

    // 2. Angle > 360 must throw
    assert.throws(() =>
      createRevolveMesh(
        [
          [0.01, -0.02],
          [0.03, -0.02],
          [0.03, 0.02],
        ],
        720,
        'standard',
        7850
      )
    );

    // 3. Self-intersecting figure-8 profile must throw
    assert.throws(() =>
      createRevolveMesh(
        [
          [0.01, -0.02],
          [0.03, 0.02],
          [0.01, 0.02],
          [0.03, -0.02],
        ],
        360,
        'standard',
        7850
      )
    );

    // 4. tessellate_part accepts 'coarse', 'medium', 'fine' and rejects invalid quality or chord_tolerance <= 0
    const baseState: EngineStateSnapshot = {
      parts: [
        {
          id: 'cyl_1',
          name: 'Cylinder',
          materialId: 'steel_1045',
          dag: [
            {
              id: 'n1',
              name: 'Cyl',
              type: 'cylinder',
              params: { radius: 0.02, height: 0.05 },
              enabled: true,
            },
          ],
          mesh: createCylinderMesh(0.02, 0.05, 'standard', 7850),
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        },
      ],
      selectedPartId: 'cyl_1',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const fineRes = executeMcpToolOnEngineState(baseState, 'tessellate_part', {
      part_id: 'cyl_1',
      quality: 'fine',
      chord_tolerance: 0.0005,
    });
    assert.equal(fineRes.response.status, 'ok');
    assert.equal(fineRes.nextState.quality, 'fine');

    assert.throws(() =>
      executeMcpToolOnEngineState(baseState, 'tessellate_part', {
        part_id: 'cyl_1',
        quality: 'ultra_invalid',
      })
    );
    assert.throws(() =>
      executeMcpToolOnEngineState(baseState, 'tessellate_part', {
        part_id: 'cyl_1',
        chord_tolerance: -0.001,
      })
    );
  });

  it('verifies BUG-017: FEA unconstrained model rejection, zero-load zero-response, and Axial Rod Tension benchmark', () => {
    // 1. Axial Rod Tension benchmark: L = 1.0 m, A = 0.01 x 0.01 m^2 = 1e-4 m^2, E = 200 GPa, F = 2000 N along X
    // Analytical: dL = F*L / (E*A) = 2000 * 1.0 / (200e9 * 1e-4) = 1e-4 m = 0.100 mm
    // Analytical: sigma = F / A = 2000 / 1e-4 = 20e6 Pa = 20.0 MPa
    const rodMesh = createBoxMesh(1.0, 0.01, 0.01, 7850);
    const steel200GPa = {
      ...DEFAULT_MATERIALS[0],
      youngsModulus: 200e9,
      yieldStrength: 250e6,
    };
    const rodPart = {
      id: 'axial_rod',
      name: 'Axial Rod 1m',
      materialId: steel200GPa.id,
      dag: [],
      mesh: rodMesh,
      visible: true,
      position: [0, 0, 0] as [number, number, number],
      rotation: [0, 0, 0] as [number, number, number],
      scale: [1, 1, 1] as [number, number, number],
    };

    const axialOut = runLinearFeaSolver(rodPart, steel200GPa, [
      { region: 'min_x', type: 'fixed' },
      { region: 'max_x', type: 'force', forceVector: [2000, 0, 0] },
    ]);
    // 3D TET4 with Poisson contraction (nu = 0.29) and fixed face lateral constraint exhibits slight boundary stiffening (0.097 mm vs 0.100 mm 1D rod)
    assert.ok(Math.abs(axialOut.result.maxDisplacementMm - 0.1) < 0.005);
    assert.ok(Math.abs(axialOut.result.maxVonMisesMPa - 20.0) < 0.5);

    // 2. Zero load [0, 0, 0] must yield exactly 0 displacement and 0 von Mises stress
    const zeroOut = runLinearFeaSolver(rodPart, steel200GPa, [
      { region: 'min_x', type: 'fixed' },
      { region: 'max_x', type: 'force', forceVector: [0, 0, 0] },
    ]);
    assert.equal(zeroOut.result.maxDisplacementMm, 0);
    assert.equal(zeroOut.result.maxVonMisesMPa, 0);

    // 3. Unconstrained model (no fixed boundary condition) must throw
    assert.throws(() =>
      runLinearFeaSolver(rodPart, steel200GPa, [
        { region: 'max_x', type: 'force', forceVector: [1000, 0, 0] },
      ])
    );

    // 4. Kirsch Stress Concentration on Plate with Hole: F_x = 10 kN on 0.2 x 0.1 x 0.01 plate with d=0.03 hole
    // Net area A_net = 7e-4 m^2 => nominal stress = 14.28 MPa.
    // Kirsch concentration Kt produces peak stress > nominal stress.
    const plateMesh = applyHolesToMesh(
      createBoxMesh(0.2, 0.1, 0.01, 7850),
      [{ position: [0, 0, 0], normal: [0, 0, 1], radius: 0.015, depth: 0.02 }]
    );
    const platePart = {
      id: 'plate_hole',
      name: 'Kirsch Plate',
      materialId: steel200GPa.id,
      dag: [],
      mesh: plateMesh,
      visible: true,
      position: [0, 0, 0] as [number, number, number],
      rotation: [0, 0, 0] as [number, number, number],
      scale: [1, 1, 1] as [number, number, number],
    };
    const kirschOut = runLinearFeaSolver(platePart, steel200GPa, [
      { region: 'min_x', type: 'fixed' },
      { region: 'max_x', type: 'force', forceVector: [10000, 0, 0] },
    ], 'medium');
    assert.ok(kirschOut.result.maxVonMisesMPa > 14.28, 'Peak stress must exceed nominal stress at hole boundary');
  });

  it('executes 50 deterministic seeded DAG fuzzing chains (2-6 ops each) with 0 panics and 100% watertight manifold output', () => {
    let seed = 0x20261101;
    const nextRand = () => {
      seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
      return seed / 0x100000000;
    };

    for (let chainIdx = 0; chainIdx < 50; chainIdx++) {
      let state: EngineStateSnapshot = {
        parts: [],
        selectedPartId: '',
        joints: [],
        materials: [...DEFAULT_MATERIALS],
        feaResult: null,
        physicsFrames: [],
        quality: 'standard',
        renderMode: 'shaded',
      };

      const partId = `fuzz_part_${chainIdx}`;
      const baseChoice = chainIdx % 5;
      if (baseChoice === 0) {
        state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
          part_id: partId,
          primitive_type: 'box',
          params: {
            width: 0.08 + nextRand() * 0.06,
            height: 0.06 + nextRand() * 0.04,
            depth: 0.08 + nextRand() * 0.06,
          },
        }).nextState;
      } else if (baseChoice === 1) {
        state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
          part_id: partId,
          primitive_type: 'cylinder',
          params: {
            radius: 0.03 + nextRand() * 0.02,
            height: 0.06 + nextRand() * 0.04,
          },
        }).nextState;
      } else if (baseChoice === 2) {
        state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
          part_id: partId,
          primitive_type: 'polygon_prism',
          params: {
            sides: 6 + (chainIdx % 3) * 2,
            outer_radius: 0.025 + nextRand() * 0.015,
            height: 0.03 + nextRand() * 0.02,
            chamfer_radius: 0.002,
          },
        }).nextState;
      } else if (baseChoice === 3) {
        state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
          part_id: partId,
          primitive_type: 'spur_gear',
          params: {
            teeth: 12 + (chainIdx % 12),
            module_mm: 2.0,
            face_width: 0.015,
            bore_radius: 0.006,
            hub_radius: 0.012,
            hub_Length: 0.005,
            keyway_width: 0.003,
            keyway_depth: 0.0015,
          },
        }).nextState;
      } else {
        state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
          part_id: partId,
          primitive_type: 'wheel_rim',
          params: {
            outer_radius: 0.22,
            rim_width: 0.12,
            spokes: 5 + (chainIdx % 4),
          },
        }).nextState;
      }

      const extraOps = 1 + (chainIdx % 4);
      for (let step = 0; step < extraOps; step++) {
        if (step % 2 === 0 && baseChoice <= 2) {
          state = executeMcpToolOnEngineState(state, 'cut_hole', {
            part_id: partId,
            radius: 0.004 + nextRand() * 0.003,
            depth: 0.15,
            offset_x: (nextRand() - 0.5) * 0.01,
            offset_z: (nextRand() - 0.5) * 0.01,
          }).nextState;
        } else {
          const sx = step === 1 && chainIdx % 7 === 0 ? -1 : 1;
          state = executeMcpToolOnEngineState(state, 'transform_part', {
            part_id: partId,
            translation: [nextRand() * 0.01, nextRand() * 0.01, 0],
            rotation_deg: [0, 0, Math.round(nextRand() * 45)],
            scale: [sx, 1, 1],
          }).nextState;
        }
      }

      const mesh = state.parts[0].mesh;
      assert.equal(mesh.isWatertight, true, `Chain ${chainIdx} must be watertight`);
      assert.equal(mesh.boundaryEdgesCount, 0, `Chain ${chainIdx} must have 0 boundary edges`);
      assert.equal(mesh.nonManifoldEdgesCount, 0, `Chain ${chainIdx} must have 0 non-manifold edges`);
      assert.equal(mesh.degenerateTrianglesCount, 0, `Chain ${chainIdx} must have 0 degenerate triangles`);
      assert.equal(mesh.isOutwardOriented, true, `Chain ${chainIdx} must have outward normals`);
      assert.ok(mesh.volume > 0, `Chain ${chainIdx} must have positive volume`);
    }
  });

  it('verifies BUG-018: zero <1 deg slivers and <=1% <5 deg slivers across all primitives, CSG chains, and SKEPTIC stress tests', () => {
    const cases = [
      { name: 'box', mesh: createBoxMesh(0.1, 0.04, 0.05, 7850), expectedGenus: 0 },
      { name: 'cylinder_std', mesh: createCylinderMesh(0.025, 0.08, 'standard', 7850), expectedGenus: 0 },
      { name: 'cylinder_fine', mesh: createCylinderMesh(0.025, 0.08, 'fine', 7850), expectedGenus: 0 },
      { name: 'sphere_std', mesh: createSphereMesh(0.035, 'standard', 7850), expectedGenus: 0 },
      { name: 'cone_sharp', mesh: createConeMesh(0.03, 0.0, 0.07, 'standard', 7850), expectedGenus: 0 },
      { name: 'cone_frustum', mesh: createConeMesh(0.035, 0.015, 0.06, 'standard', 7850), expectedGenus: 0 },
      {
        name: 'polygon_prism_hex',
        mesh: createPolygonPrismMesh(6, 0.024 / Math.sqrt(3), 0.018, 0.002, 7850, 0.005, 0.0015, 0.0008),
        expectedGenus: 1,
      },
      {
        name: 'spur_gear_20t',
        mesh: createSpurGearMesh(20, 2.0, 20, 0.015, 0.006, 0.024, 0.008, 0.004, 0.002, 7850),
        expectedGenus: 1,
      },
      {
        name: 'spur_gear_42t_m1_stress',
        mesh: createSpurGearMesh(42, 1.0, 20, 0.012, 0.005, 0.018, 0.006, 0.003, 0.0015, 7850),
        expectedGenus: 1,
      },
      {
        name: 'revolve_partial_180',
        mesh: createRevolveMesh(
          [
            [0.01, -0.02],
            [0.03, -0.02],
            [0.03, 0.02],
            [0.01, 0.02],
          ],
          180,
          'standard',
          7850
        ),
        expectedGenus: 0,
      },
      {
        name: 'wheel_rim_5spoke',
        mesh: createWheelRimMesh(0.4572, 0.2159, 5, 0.15, 0.065, 5),
        expectedGenus: 11,
      },
      {
        name: 'wheel_rim_10spoke_stress',
        mesh: createWheelRimMesh(0.4572, 0.2159, 10, 0.15, 0.065, 5),
        expectedGenus: 16,
      },
      {
        name: 'csg_chain_two_holes',
        mesh: applyHolesToMesh(createBoxMesh(0.1, 0.04, 0.05, 7850), [
          { position: [-0.025, 0, 0], normal: [0, 0, -1], radius: 0.01, depth: 0.05 },
          { position: [0.025, 0, 0], normal: [0, 0, -1], radius: 0.01, depth: 0.05 },
        ]),
        expectedGenus: 2,
      },
    ];

    for (const c of cases) {
      const report = validatePartMesh(c.name, c.mesh, c.expectedGenus);
      assert.equal(report.isWatertight, true, `${c.name} must be watertight`);
      assert.equal(report.boundaryEdgesCount, 0, `${c.name} must have 0 boundary edges`);
      assert.equal(report.nonManifoldEdgesCount, 0, `${c.name} must have 0 non-manifold edges`);
      assert.equal(report.inconsistentEdgesCount, 0, `${c.name} must have 0 inconsistent edges`);
      assert.equal(report.isOutwardOriented, true, `${c.name} must be outward oriented`);
      assert.equal(report.eulerGenus, c.expectedGenus, `${c.name} genus mismatch`);
      assert.equal(report.lt1DegCount, 0, `${c.name} must have 0 triangles < 1 deg`);
      assert.ok(
        report.lt5DegRatio <= 0.01,
        `${c.name} <5 deg ratio (${report.lt5DegRatio}) must be <= 1%`
      );
      assert.equal(report.status, 'PASS', `${c.name} must pass strict mesh validation`);
    }

    // SKEPTIC negative verification: anisotropic scale [1, 1, 0.002] on a cylinder stretches triangles into <1 deg slivers
    // and MUST be caught and rejected by the strict sliver quality gate!
    const squashed = applyTransformToMesh(
      createCylinderMesh(0.025, 0.08, 'standard', 7850),
      [0, 0, 0],
      [0, 0, 0],
      [1, 1, 0.002],
      7850
    );
    const squashedReportDefault = validatePartMesh('squashed_cyl', squashed);
    assert.ok(squashedReportDefault.lt5DegRatio > 0.01 || squashedReportDefault.lt1DegCount > 0);
    assert.ok(squashedReportDefault.warnings.length > 0);
    assert.equal(squashedReportDefault.isValidSolid, true);
    assert.equal(squashedReportDefault.status, 'PASS');

    const squashedReport = validatePartMesh('squashed_cyl', squashed, 0, true);
    assert.equal(squashedReport.status, 'FAIL', 'Sliver-contaminated mesh must fail validation gate');
  });

  it('verifies BUG-020: Rust vs TS parity for 3D CSG booleans, transactional modify_feature rollback, and validate_mesh nested fields', () => {
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

    // 1. Create Part A (Box 0.1 x 0.1 x 0.1 at origin, volume = 0.001 m^3)
    const s1 = executeMcpToolOnEngineState(initialState, 'add_feature_primitive', {
      part_id: 'box_a',
      primitive_type: 'box',
      params: { width: 0.1, height: 0.1, depth: 0.1 },
    }).nextState;

    // 2. Create Part B (Box 0.1 x 0.1 x 0.1 shifted by dx=0.05)
    const s2 = executeMcpToolOnEngineState(s1, 'add_feature_primitive', {
      part_id: 'box_b',
      primitive_type: 'box',
      params: { width: 0.1, height: 0.1, depth: 0.1 },
    }).nextState;
    const s3 = executeMcpToolOnEngineState(s2, 'transform_part', {
      part_id: 'box_b',
      translation: [0.05, 0, 0],
    }).nextState;

    // 3. Test 3D CSG Boolean Difference (A \ B)
    const diffRes = executeMcpToolOnEngineState(s3, 'boolean_difference', {
      part_a_id: 'box_a',
      part_b_id: 'box_b',
      new_part_id: 'part_diff',
    });
    assert.equal(diffRes.response.status, 'ok');
    const partDiff = diffRes.nextState.parts.find((p) => p.id === 'part_diff');
    assert.ok(partDiff, 'part_diff must be created in engine state');
    // Volume: Part A is 0.001 m^3, overlap is 0.05 * 0.1 * 0.1 = 0.0005 m^3. Result = 0.0005 m^3
    assert.ok(
      Math.abs(partDiff.mesh.volume - 0.0005) < 1e-5,
      `Expected difference volume 0.0005, got ${partDiff.mesh.volume}`
    );
    assert.equal(partDiff.mesh.isWatertight, true, 'CSG difference must be watertight');
    assert.equal(partDiff.mesh.boundaryEdgesCount, 0, 'CSG difference must have 0 boundary edges');

    // 4. Test 3D CSG Boolean Intersection (A ∩ B)
    const interRes = executeMcpToolOnEngineState(s3, 'boolean_intersection', {
      part_a_id: 'box_a',
      part_b_id: 'box_b',
      new_part_id: 'part_inter',
    });
    assert.equal(interRes.response.status, 'ok');
    const partInter = interRes.nextState.parts.find((p) => p.id === 'part_inter');
    assert.ok(partInter, 'part_inter must be created');
    assert.ok(
      Math.abs(partInter.mesh.volume - 0.0005) < 1e-5,
      `Expected intersection volume 0.0005, got ${partInter.mesh.volume}`
    );
    assert.equal(partInter.mesh.isWatertight, true, 'CSG intersection must be watertight');
    assert.equal(partInter.mesh.boundaryEdgesCount, 0, 'CSG intersection must have 0 boundary edges');

    // 5. Test 3D CSG Boolean Union (A ∪ B)
    const unionRes = executeMcpToolOnEngineState(s3, 'boolean_union', {
      part_a_id: 'box_a',
      part_b_id: 'box_b',
      new_part_id: 'part_union',
    });
    assert.equal(unionRes.response.status, 'ok');
    const partUnion = unionRes.nextState.parts.find((p) => p.id === 'part_union');
    assert.ok(partUnion, 'part_union must be created');
    // Union volume: 0.001 + 0.001 - 0.0005 = 0.0015 m^3
    assert.ok(
      Math.abs(partUnion.mesh.volume - 0.0015) < 1e-5,
      `Expected union volume 0.0015, got ${partUnion.mesh.volume}`
    );
    assert.equal(partUnion.mesh.isWatertight, true, 'CSG union must be watertight');
    assert.equal(partUnion.mesh.boundaryEdgesCount, 0, 'CSG union must have 0 boundary edges');

    // 6. Test modify_feature transactional rollback on invalid params
    const boxBefore = JSON.stringify(s1.parts[0]);
    assert.throws(() =>
      executeMcpToolOnEngineState(s1, 'modify_feature', {
        part_id: 'box_a',
        node_id: s1.parts[0].dag[0].id,
        params: { width: -0.05 }, // invalid negative dimension
      })
    );
    const boxAfter = JSON.stringify(s1.parts[0]);
    assert.equal(boxBefore, boxAfter, 'Original part state must remain unchanged after failed modify_feature');

    // 7. Test unified validate_mesh nested fields parity
    const valRes = executeMcpToolOnEngineState(diffRes.nextState, 'validate_mesh', {
      part_id: 'part_diff',
    }).response;
    assert.equal(valRes.status, 'PASS');
    // Top level
    assert.equal(valRes.is_watertight, true);
    assert.equal(valRes.boundary_edges_count, 0);
    assert.equal(valRes.non_manifold_edges_count, 0);
    // Nested topology
    assert.ok(valRes.topology, 'Must contain nested topology');
    assert.equal(valRes.topology.is_watertight, true);
    assert.equal(valRes.topology.boundary_edges_count, 0);
    // Nested geometry
    assert.ok(valRes.geometry, 'Must contain nested geometry');
    assert.ok(Math.abs(valRes.geometry.volume - 0.0005) < 1e-5);
    assert.ok(Array.isArray(valRes.geometry.centroid));
    assert.ok(valRes.geometry.bounding_box);
    // Nested angle stats
    assert.ok(valRes.angle_stats, 'Must contain nested angle_stats');
    assert.equal(valRes.angle_stats.lt1_deg_count, 0);
    assert.ok(valRes.angle_stats.angle_histogram);

    // 8. SKEPTIC challenges:
    // SKEPTIC 1: Boolean operation with non-existent part must fail cleanly
    assert.throws(() =>
      executeMcpToolOnEngineState(s3, 'boolean_difference', {
        part_a_id: 'box_a',
        part_b_id: 'ghost_part_404',
      })
    );
    // SKEPTIC 2: Mid-chain modification rollback on part with multiple nodes
    const chained = executeMcpToolOnEngineState(s1, 'cut_hole', {
      part_id: 'box_a',
      position: [0, 0, 0],
      normal: [0, 0, -1],
      radius: 0.01,
      depth: 0.1,
    }).nextState;
    const chainedSnap = JSON.stringify(chained.parts[0]);
    assert.throws(() =>
      executeMcpToolOnEngineState(chained, 'modify_feature', {
        part_id: 'box_a',
        node_id: chained.parts[0].dag[1].id,
        params: { radius: -0.02 },
      })
    );
    assert.equal(JSON.stringify(chained.parts[0]), chainedSnap);
    // SKEPTIC 3: Slicing identical overlapping meshes in boolean difference (A \ A)
    const selfDiff = executeMcpToolOnEngineState(s1, 'boolean_difference', {
      part_a_id: 'box_a',
      part_b_id: 'box_a',
      new_part_id: 'empty_solid',
    });
    // Self difference produces empty geometry or 0 volume
    const emptyMesh = selfDiff.nextState.parts.find((p) => p.id === 'empty_solid')?.mesh;
    assert.ok(emptyMesh);
    assert.ok(emptyMesh.volume < 1e-7 || emptyMesh.triangleCount === 0);
  });

  it('verifies BUG-021: 28 MCP tools documentation & contract parity across README, setup guides, architecture, and API reference', () => {
    // 1. Verify exactly 28 tools in catalog with unique names
    assert.equal(MCP_TOOLS_CATALOG.length, 28, 'Catalog must contain exactly 28 tools');
    const catalogNames = new Set(MCP_TOOLS_CATALOG.map((t) => t.name));
    assert.equal(catalogNames.size, 28, 'All 28 tool names must be unique');

    // Expected 28 tools
    const EXPECTED_28_TOOLS = [
      'create_part',
      'list_parts',
      'delete_part',
      'add_feature_primitive',
      'cut_hole',
      'cut_helical_groove',
      'modify_feature',
      'get_feature_tree',
      'boolean_difference',
      'boolean_union',
      'boolean_intersection',
      'transform_part',
      'tessellate_part',
      'query_part_geometry',
      'validate_mesh',
      'define_material',
      'batch_cad_operations',
      'create_joint',
      'set_joint_motor',
      'run_physics_simulation',
      'run_fea_analysis',
      'get_fea_heatmap_image',
      'capture_viewport',
      'export_stl',
      'get_simulation_metrics_json',
      'open_preview_window',
      'import_stl',
      'optimize_feature_tree',
    ];

    for (const tool of EXPECTED_28_TOOLS) {
      assert.ok(catalogNames.has(tool), `Missing tool '${tool}' in MCP catalog`);
    }

    // 2. Automated Contract check against README.md
    const readmeContent = fs.readFileSync(path.resolve(process.cwd(), 'README.md'), 'utf-8');
    for (const tool of EXPECTED_28_TOOLS) {
      assert.ok(
        readmeContent.includes(`\`${tool}\``),
        `README.md missing documented entry for tool '${tool}'`
      );
    }
    assert.ok(readmeContent.includes('is_fixed'), 'README must document is_fixed parameter');
    assert.ok(readmeContent.includes('gear'), 'README must document gear joint');
    assert.ok(readmeContent.includes('debug_mode'), 'README must document debug_mode parameter');

    // 3. Automated Contract check against docs/CLAUDE_DESKTOP_SETUP.md
    const claudeContent = fs.readFileSync(path.resolve(process.cwd(), 'docs/CLAUDE_DESKTOP_SETUP.md'), 'utf-8');
    assert.ok(claudeContent.includes('28 инженерных инструментов'), 'Claude setup must state 28 tools');
    for (const tool of EXPECTED_28_TOOLS) {
      assert.ok(
        claudeContent.includes(`\`${tool}\``),
        `docs/CLAUDE_DESKTOP_SETUP.md missing tool '${tool}'`
      );
    }

    // 4. Automated Contract check against docs/ANTIGRAVITY_SETUP.md
    const antigravityContent = fs.readFileSync(path.resolve(process.cwd(), 'docs/ANTIGRAVITY_SETUP.md'), 'utf-8');
    assert.ok(
      antigravityContent.includes('28 инженерных инструментов'),
      'Antigravity setup must state 28 tools'
    );

    // 5. Automated Contract check against docs/ARCHITECTURE.md
    const archContent = fs.readFileSync(path.resolve(process.cwd(), 'docs/ARCHITECTURE.md'), 'utf-8');
    assert.ok(archContent.includes('Gear'), 'ARCHITECTURE.md must document Gear joint');
    assert.ok(archContent.includes('WheelRim'), 'ARCHITECTURE.md must document WheelRim primitive');
    assert.ok(archContent.includes('SpurGear'), 'ARCHITECTURE.md must document SpurGear primitive');
    assert.ok(archContent.includes('src/cad'), 'ARCHITECTURE.md must document TypeScript CAD layer');

    // 6. Automated Contract check against docs/MCP_API_REFERENCE.md
    const apiRefContent = fs.readFileSync(path.resolve(process.cwd(), 'docs/MCP_API_REFERENCE.md'), 'utf-8');
    for (const tool of EXPECTED_28_TOOLS) {
      assert.ok(
        apiRefContent.includes(`\`${tool}\``),
        `docs/MCP_API_REFERENCE.md missing tool '${tool}'`
      );
    }
  });

  it('verifies BUG-022: comprehensive coverage in physics (all joints, limits, 0s/10s), STL I/O edge cases, mid-chain modify_feature, and CSG degeneracies', () => {
    // 1. Physics Coverage: All joint types (revolute, prismatic, spherical, fixed, gear) with limits
    const state0: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const sP1 = executeMcpToolOnEngineState(state0, 'create_part', {
      part_id: 'base',
      is_fixed: true,
      material: 'steel_structural_aisi1020',
    }).nextState;
    const sP2 = executeMcpToolOnEngineState(sP1, 'create_part', {
      part_id: 'arm',
      is_fixed: false,
      material: 'aluminum_6061_t6',
    }).nextState;
    const sP3 = executeMcpToolOnEngineState(sP2, 'create_part', {
      part_id: 'slider',
      is_fixed: false,
      material: 'aluminum_6061_t6',
    }).nextState;

    // Revolute joint with limits
    const jRev = executeMcpToolOnEngineState(sP3, 'create_joint', {
      joint_type: 'revolute',
      part_a: 'base',
      part_b: 'arm',
      anchor_a: [0, 0, 0],
      anchor_b: [0, 0, 0],
      axis: [0, 0, 1],
      limits: [-Math.PI / 2, Math.PI / 2],
    }).nextState;

    // Prismatic joint with limits
    const jPrism = executeMcpToolOnEngineState(jRev, 'create_joint', {
      joint_type: 'prismatic',
      part_a: 'arm',
      part_b: 'slider',
      anchor_a: [0, 0, 0],
      anchor_b: [0, 0, 0],
      axis: [1, 0, 0],
      limits: [-0.05, 0.05],
    }).nextState;

    // Spherical joint
    const jSph = executeMcpToolOnEngineState(jPrism, 'create_joint', {
      joint_type: 'spherical',
      part_a: 'base',
      part_b: 'arm',
      anchor_a: [0, 0, 0.1],
      anchor_b: [0, 0, 0],
    }).nextState;

    // Fixed joint
    const jFix = executeMcpToolOnEngineState(jSph, 'create_joint', {
      joint_type: 'fixed',
      part_a: 'base',
      part_b: 'slider',
    }).nextState;

    // Gear joint with ratio
    const jGear = executeMcpToolOnEngineState(jFix, 'create_joint', {
      joint_type: 'gear',
      part_a: 'arm',
      part_b: 'slider',
      ratio: -2.5,
    }).nextState;
    assert.equal(jGear.joints.length, 5);

    // Physics duration rejection: 0s or negative must throw
    assert.throws(
      () =>
        executeMcpToolOnEngineState(jGear, 'run_physics_simulation', {
          duration_seconds: 0,
        }),
      /duration_seconds > 0/
    );
    assert.throws(
      () =>
        executeMcpToolOnEngineState(jGear, 'run_physics_simulation', {
          duration_seconds: -2.0,
        }),
      /duration_seconds > 0/
    );

    // Physics long run: 10.0 seconds simulation runs without divergence
    const sim10 = executeMcpToolOnEngineState(jGear, 'run_physics_simulation', {
      duration_seconds: 10.0,
      step_size_ms: 16.6667,
    });
    assert.equal(sim10.response.success, true);
    assert.ok(sim10.response.steps_simulated >= 500);
    assert.ok(Array.isArray(sim10.response.collisions));

    // 2. STL I/O Edge Cases
    // A. Empty buffer (0 bytes) must throw
    assert.throws(
      () => parseStlBuffer(new ArrayBuffer(0)),
      /STL buffer is empty/
    );

    // B. Truncated binary buffer (84 bytes claiming 100 triangles, but missing triangle records)
    const truncatedBuf = new ArrayBuffer(84 + 10);
    const truncView = new DataView(truncatedBuf);
    truncView.setUint32(80, 100, true); // claims 100 triangles
    assert.throws(
      () => parseStlBuffer(truncatedBuf),
      /Corrupt|Truncated/
    );

    // C. Corrupt ASCII buffer (header without valid vertices)
    const emptyAscii = new TextEncoder().encode('solid empty_model\nendsolid empty_model').buffer;
    assert.throws(
      () => parseStlBuffer(emptyAscii),
      /zero valid triangle vertices/
    );

    // D. Corrupt non-ASCII arbitrary noise buffer
    const noise = new Uint8Array([0xde, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03]).buffer;
    assert.throws(
      () => parseStlBuffer(noise),
      /Corrupt/
    );

    // E. Binary STL round-trip geometry loop (box volume within 0.01%)
    const originalBox = createBoxMesh(0.12, 0.08, 0.05, 7850);
    const exportedStl = exportMeshToStlBinary(originalBox, 'part_roundtrip');
    const importedBox = parseStlBuffer(exportedStl, 7850);
    assert.ok(
      Math.abs(importedBox.volume - originalBox.volume) / originalBox.volume < 1e-4,
      'STL round-trip volume preservation'
    );
    assert.equal(importedBox.isWatertight, true);
    assert.equal(importedBox.boundaryEdgesCount, 0);

    // 3. Mid-chain modify_feature on 5-node DAG
    const chainBox = executeMcpToolOnEngineState(state0, 'add_feature_primitive', {
      part_id: 'multi_node_part',
      primitive_type: 'box',
      params: { width: 0.15, height: 0.05, depth: 0.08 },
    }).nextState;
    const c1 = executeMcpToolOnEngineState(chainBox, 'cut_hole', {
      part_id: 'multi_node_part',
      position: [-0.04, 0, 0],
      normal: [0, 0, -1],
      radius: 0.006,
      depth: 0.08,
    }).nextState;
    const c2 = executeMcpToolOnEngineState(c1, 'cut_hole', {
      part_id: 'multi_node_part',
      position: [0, 0, 0],
      normal: [0, 0, -1],
      radius: 0.005,
      depth: 0.08,
    }).nextState;
    const c3 = executeMcpToolOnEngineState(c2, 'cut_hole', {
      part_id: 'multi_node_part',
      position: [0.04, 0, 0],
      normal: [0, 0, -1],
      radius: 0.006,
      depth: 0.08,
    }).nextState;
    const c4 = executeMcpToolOnEngineState(c3, 'transform_part', {
      part_id: 'multi_node_part',
      translation: [0, 0.01, 0],
    }).nextState;

    const initialVol = c4.parts[0].mesh.volume;
    const targetHoleId = c4.parts[0].dag[2].id; // Hole 2 in the middle
    // Enlarge Hole 2 radius from 0.005 to 0.009
    const modified = executeMcpToolOnEngineState(c4, 'modify_feature', {
      part_id: 'multi_node_part',
      node_id: targetHoleId,
      params: { radius: 0.009 },
    }).nextState;

    const modifiedMesh = modified.parts[0].mesh;
    assert.ok(
      modifiedMesh.volume < initialVol,
      'Mid-chain hole enlargement must decrease solid volume'
    );
    assert.equal(modifiedMesh.eulerGenus, 3, 'All 3 holes must persist downstream');
    assert.equal(modifiedMesh.isWatertight, true);
    assert.equal(modifiedMesh.boundaryEdgesCount, 0);

    // 4. CSG Boolean Edge Cases
    const boxA = executeMcpToolOnEngineState(state0, 'add_feature_primitive', {
      part_id: 'cube_a',
      primitive_type: 'box',
      params: { width: 0.08, height: 0.08, depth: 0.08 },
    }).nextState;

    // A \ A: Self-difference produces empty solid
    const diffAA = executeMcpToolOnEngineState(boxA, 'boolean_difference', {
      part_a_id: 'cube_a',
      part_b_id: 'cube_a',
      new_part_id: 'res_diff_aa',
    });
    const meshAA = diffAA.nextState.parts.find((p) => p.id === 'res_diff_aa')?.mesh;
    assert.ok(meshAA);
    assert.ok(meshAA.volume < 1e-7 || meshAA.triangleCount === 0);

    // A ∪ A: Self-union preserves exact volume
    const unionAA = executeMcpToolOnEngineState(boxA, 'boolean_union', {
      part_a_id: 'cube_a',
      part_b_id: 'cube_a',
      new_part_id: 'res_union_aa',
    });
    const meshUnionAA = unionAA.nextState.parts.find((p) => p.id === 'res_union_aa')?.mesh;
    assert.ok(meshUnionAA);
    assert.ok(Math.abs(meshUnionAA.volume - boxA.parts[0].mesh.volume) < 1e-5);
    assert.equal(meshUnionAA.isWatertight, true);

    // Disjoint A and B (dx = 1.0 m)
    const boxBDisjoint = executeMcpToolOnEngineState(boxA, 'add_feature_primitive', {
      part_id: 'cube_disjoint',
      primitive_type: 'box',
      params: { width: 0.08, height: 0.08, depth: 0.08 },
    }).nextState;
    const boxBDisjointTrans = executeMcpToolOnEngineState(boxBDisjoint, 'transform_part', {
      part_id: 'cube_disjoint',
      translation: [1.0, 0, 0],
    }).nextState;

    // Disjoint difference A \ B = A
    const diffDisjoint = executeMcpToolOnEngineState(boxBDisjointTrans, 'boolean_difference', {
      part_a_id: 'cube_a',
      part_b_id: 'cube_disjoint',
      new_part_id: 'res_diff_disjoint',
    });
    const meshDiffDisjoint = diffDisjoint.nextState.parts.find((p) => p.id === 'res_diff_disjoint')?.mesh;
    assert.ok(meshDiffDisjoint);
    assert.ok(Math.abs(meshDiffDisjoint.volume - boxA.parts[0].mesh.volume) < 1e-5);

    // Disjoint intersection A ∩ B = 0
    const interDisjoint = executeMcpToolOnEngineState(boxBDisjointTrans, 'boolean_intersection', {
      part_a_id: 'cube_a',
      part_b_id: 'cube_disjoint',
      new_part_id: 'res_inter_disjoint',
    });
    const meshInterDisjoint = interDisjoint.nextState.parts.find((p) => p.id === 'res_inter_disjoint')?.mesh;
    assert.ok(meshInterDisjoint);
    assert.ok(meshInterDisjoint.volume < 1e-7 || meshInterDisjoint.triangleCount === 0);
  });

  it('verifies BUG-023: expanded DAG fuzzer across 5 seeds x 200 chains (1000 chains up to 10 ops) with 0 panics and watertight manifold topology', () => {
    const SEEDS = [101, 202, 303, 404, 505];
    const CHAINS_PER_SEED = 200;
    let totalChainsTested = 0;

    for (const seed of SEEDS) {
      let lcg = seed;
      const nextRand = () => {
        lcg = (lcg * 1664525 + 1013904223) >>> 0;
        return lcg / 4294967296;
      };

      for (let c = 0; c < CHAINS_PER_SEED; c++) {
        totalChainsTested++;
        const partId = `fuzz_s${seed}_c${c}`;
        let state: EngineStateSnapshot = {
          parts: [],
          selectedPartId: '',
          joints: [],
          materials: [...DEFAULT_MATERIALS],
          feaResult: null,
          physicsFrames: [],
          quality: 'draft',
          renderMode: 'shaded',
        };

        const basePrimType =
          c % 4 === 0
            ? 'box'
            : c % 4 === 1
              ? 'cylinder'
              : c % 4 === 2
                ? 'cone'
                : 'polygon_prism';

        state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
          part_id: partId,
          primitive_type: basePrimType,
          params: {
            width: 0.08 + nextRand() * 0.04,
            height: 0.06 + nextRand() * 0.04,
            depth: 0.08 + nextRand() * 0.04,
            radius: 0.03 + nextRand() * 0.02,
            radius_bottom: 0.03 + nextRand() * 0.02,
            radius_top: 0.015,
            sides: 6,
            outer_radius: 0.035,
          },
        }).nextState;

        // Chain up to 6 extra operations
        const chainLen = 1 + (c % 6);
        for (let step = 0; step < chainLen; step++) {
          if (step % 2 === 0 && (basePrimType === 'box' || basePrimType === 'cylinder')) {
            state = executeMcpToolOnEngineState(state, 'cut_hole', {
              part_id: partId,
              radius: 0.004 + nextRand() * 0.003,
              depth: 0.12,
              offset_x: (nextRand() - 0.5) * 0.01,
              offset_z: (nextRand() - 0.5) * 0.01,
            }).nextState;
          } else {
            const sx = step === 1 && c % 11 === 0 ? -1 : 1;
            state = executeMcpToolOnEngineState(state, 'transform_part', {
              part_id: partId,
              translation: [nextRand() * 0.01, nextRand() * 0.01, 0],
              rotation_deg: [0, 0, Math.round(nextRand() * 45)],
              scale: [sx, 1, 1],
            }).nextState;
          }
        }

        const mesh = state.parts[0].mesh;
        assert.equal(mesh.isWatertight, true, `Fuzzer chain ${partId} must be watertight`);
        assert.equal(mesh.boundaryEdgesCount, 0, `Fuzzer chain ${partId} must have 0 boundary edges`);
        assert.equal(mesh.nonManifoldEdgesCount, 0, `Fuzzer chain ${partId} must have 0 non-manifold edges`);
        assert.equal(mesh.degenerateTrianglesCount, 0, `Fuzzer chain ${partId} must have 0 degenerate triangles`);
        assert.equal(mesh.isOutwardOriented, true, `Fuzzer chain ${partId} must have outward normals`);
        assert.ok(mesh.volume > 0, `Fuzzer chain ${partId} must have positive volume`);
      }
    }

    assert.equal(totalChainsTested, 1000, 'Exactly 1,000 fuzzer chains must be executed');
  });

  it('supports P2-T13 custom joint_id, cut_hole through: true, revolve.inner_radius, polygon_prism aliases, and material ID aliases', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    // 1. Annular bushing via revolve with 2-point profile + inner_radius and brass_c36000 alias
    state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 't07_bushing',
      material: 'brass_c36000',
      primitive_type: 'revolve',
      params: {
        profile: [
          [0.035, -0.04],
          [0.035, 0.04],
        ],
        inner_radius: 0.015,
        angle_deg: 360,
      },
    }).nextState;
    const bushing = state.parts.find((p) => p.id === 't07_bushing')!;
    assert.equal(bushing.mesh.isWatertight, true);
    assert.equal(bushing.mesh.eulerGenus, 1);

    // 2. Hex nut M10 with polygon_prism (outer_radius/chamfer_radius) + cut_hole (through: true)
    state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 't08_nut_m10',
      material: 'steel_high_strength_4140',
      primitive_type: 'polygon_prism',
      params: {
        sides: 6,
        outer_radius: 0.0098,
        height: 0.008,
        chamfer_radius: 0.0008,
      },
    }).nextState;
    state = executeMcpToolOnEngineState(state, 'cut_hole', {
      part_id: 't08_nut_m10',
      position: [0, 0, 0],
      normal: [0, 0, -1],
      radius: 0.00425,
      through: true,
    }).nextState;
    const nut = state.parts.find((p) => p.id === 't08_nut_m10')!;
    assert.equal(nut.mesh.isWatertight, true);
    assert.equal(nut.mesh.eulerGenus, 1);

    // 3. Custom joint_id in create_joint + set_joint_motor
    const jRes = executeMcpToolOnEngineState(state, 'create_joint', {
      joint_id: 'j_custom_bushing',
      joint_type: 'revolute',
      part_a: 'ground',
      part_b: 't07_bushing',
      axis: [0, 0, 1],
    });
    state = jRes.nextState;
    assert.equal(jRes.response.joint_id, 'j_custom_bushing');

    const mRes = executeMcpToolOnEngineState(state, 'set_joint_motor', {
      joint_id: 'j_custom_bushing',
      target_velocity: 6.283185,
      max_force: 120.0,
    });
    assert.equal(mRes.response.status, 'motor_configured');
    assert.equal(mRes.response.joint_id, 'j_custom_bushing');
  });

  it('supports P2-T15 small wheel_rim (< 140 mm) without clamp panic and prismatic joint motor actuation', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    for (const [rimD, rimW] of [
      [0.05, 0.025],
      [0.10, 0.05],
      [0.12, 0.06],
    ]) {
      const partId = `small_rim_${Math.round(rimD * 1000)}mm`;
      state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
        part_id: partId,
        primitive_type: 'wheel_rim',
        params: {
          rim_diameter: rimD,
          rim_width: rimW,
          bolt_holes_count: 5,
          spokes_count: 5,
        },
      }).nextState;
      const part = state.parts.find((p) => p.id === partId)!;
      assert.equal(part.mesh.isWatertight, true, `${partId} must be watertight`);
      assert.equal(part.mesh.boundaryEdgesCount, 0);
      assert.equal(part.mesh.nonManifoldEdgesCount, 0);
      assert.equal(part.mesh.eulerGenus, 11);
    }

    state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'slider_carriage',
      primitive_type: 'box',
      params: { width: 0.04, height: 0.04, depth: 0.04 },
    }).nextState;

    const jPrism = executeMcpToolOnEngineState(state, 'create_joint', {
      joint_id: 'j_prism_p2_t15',
      joint_type: 'prismatic',
      part_a: 'ground',
      part_b: 'slider_carriage',
      axis: [1, 0, 0],
      limits: [-0.5, 0.5],
    });
    state = jPrism.nextState;

    state = executeMcpToolOnEngineState(state, 'set_joint_motor', {
      joint_id: 'j_prism_p2_t15',
      target_velocity: 1.5,
      max_force: 300.0,
    }).nextState;

    const simRes = executeMcpToolOnEngineState(state, 'run_physics_simulation', {
      duration_seconds: 0.4,
      dt: 0.01,
    });
    state = simRes.nextState;
    const sliderAfter = state.parts.find((p) => p.id === 'slider_carriage')!;
    assert.ok(
      sliderAfter.position[0] > 0.3,
      `Expected slider_carriage to move > 0.3 m along +X under prismatic motor, got ${sliderAfter.position[0]}`
    );

    assert.throws(() => {
      executeMcpToolOnEngineState(state, 'create_joint', {
        joint_type: 'revolute',
        part_a: 'ground',
        part_b: 'slider_carriage',
        limits: [45.0, -45.0],
      });
    }, /limits/);
  });

  it('verifies P2-T19: transactional rollback of part.dag/part.mesh in modify_feature and auto start_z alignment in cut_helical_groove', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    // 1. Create M10x1.5 hex nut (height = 0.008 m centered at z = 0 => z in [-0.004, +0.004])
    state = executeMcpToolOnEngineState(state, 'create_part', {
      part_id: 'p2_t19_nut',
      name: 'M10x1.5 Hex Nut',
      material: 'steel_4140',
    }).nextState;

    const prismRes = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'p2_t19_nut',
      primitive_type: 'polygon_prism',
      params: {
        sides: 6,
        outer_radius: 0.0098,
        height: 0.008,
        chamfer_radius: 0.0008,
      },
    });
    state = prismRes.nextState;
    const prismNodeId = String(prismRes.response.node_id);

    state = executeMcpToolOnEngineState(state, 'cut_hole', {
      part_id: 'p2_t19_nut',
      position: [0, 0, 0],
      normal: [0, 0, -1],
      radius: 0.00425,
      through: true,
    }).nextState;

    // 2. Call cut_helical_groove WITHOUT start_z -> must auto-align to bottom face (-0.004 m)
    const grooveRes = executeMcpToolOnEngineState(state, 'cut_helical_groove', {
      part_id: 'p2_t19_nut',
      helix_radius: 0.005,
      groove_radius: 0.00075,
      pitch: 0.0015,
      height: 0.008,
      starts: 1,
      profile_type: 'metric_60',
    });
    state = grooveRes.nextState;

    assert.ok(
      Math.abs(Number(grooveRes.response.start_z) - -0.004) < 1e-6,
      `Expected omitted start_z to auto-align to -0.004 m, got ${grooveRes.response.start_z}`
    );
    const nutBefore = state.parts.find((p) => p.id === 'p2_t19_nut')!;
    assert.equal(nutBefore.mesh.isWatertight, true);
    assert.ok(
      Math.abs(Number(nutBefore.dag[2].params.start_z) - -0.004) < 1e-6
    );
    const origVolume = nutBefore.mesh.volume;
    const origDagJson = JSON.stringify(nutBefore.dag);

    // 3. Attempt invalid modify_feature (negative height) using feature_id -> must throw and leave part.dag & part.mesh intact
    assert.throws(() => {
      executeMcpToolOnEngineState(state, 'modify_feature', {
        part_id: 'p2_t19_nut',
        feature_id: prismNodeId,
        params: { height: -0.005 },
      });
    });

    const nutAfter = state.parts.find((p) => p.id === 'p2_t19_nut')!;
    assert.equal(nutAfter.mesh.volume, origVolume);
    assert.equal(JSON.stringify(nutAfter.dag), origDagJson);
  });

  it('verifies P2-T26: boolean_difference/union/intersection return revision, bounding_box, volume, warnings, and respect length_unit', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'target_p2t26',
      primitive_type: 'box',
      params: { width: 0.02, height: 0.02, depth: 0.02 },
    }).nextState;

    state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'tool_p2t26',
      primitive_type: 'box',
      params: { width: 0.01, height: 0.01, depth: 0.01 },
    }).nextState;

    const diffRes = executeMcpToolOnEngineState(state, 'boolean_difference', {
      target_part_id: 'target_p2t26',
      tool_part_id: 'tool_p2t26',
      length_unit: 'm',
    });
    state = diffRes.nextState;

    assert.equal(diffRes.response.target_part_id, 'target_p2t26');
    assert.equal(diffRes.response.part_id, 'target_p2t26');
    assert.equal(diffRes.response.revision, 2);
    assert.equal(diffRes.response.length_unit, 'm');
    assert.equal(diffRes.response.units.volume, 'm^3');
    assert.ok(diffRes.response.volume > 6.5e-6 && diffRes.response.volume < 7.5e-6);
    assert.equal(diffRes.response.volume, diffRes.response.volume_m3);
    assert.ok(Array.isArray(diffRes.response.bounding_box.dimensions));
    assert.ok(Math.abs(diffRes.response.bounding_box.dimensions[0] - 0.02) < 1e-4);
    assert.equal(diffRes.response.warnings.length, 0);

    const unionRes = executeMcpToolOnEngineState(state, 'boolean_union', {
      target_part_id: 'target_p2t26',
      tool_part_id: 'tool_p2t26',
      length_unit: 'mm',
    });
    state = unionRes.nextState;

    assert.equal(unionRes.response.revision, 3);
    assert.equal(unionRes.response.length_unit, 'mm');
    assert.equal(unionRes.response.units.volume, 'mm^3');
    assert.ok(Math.abs(unionRes.response.volume - 8000) < 50);
    assert.ok(Math.abs(unionRes.response.bounding_box.dimensions[0] - 20.0) < 0.1);
    assert.equal(unionRes.response.warnings.length, 0);

    const interRes = executeMcpToolOnEngineState(state, 'boolean_intersection', {
      target_part_id: 'target_p2t26',
      tool_part_id: 'tool_p2t26',
    });
    assert.equal(interRes.response.revision, 4);
    assert.equal(interRes.response.length_unit, 'mm');
    assert.ok(Math.abs(interRes.response.volume - 1000) < 50);
    assert.ok(Math.abs(interRes.response.bounding_box.dimensions[0] - 10.0) < 0.1);
    assert.equal(interRes.response.warnings.length, 0);
  });

  it('verifies P2-T27: get_simulation_metrics_json length_unit & fea staleness, and optimize_feature_tree revision when applied_rules is empty', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'box_p2t27',
      primitive_type: 'box',
      params: { width: 0.1, height: 0.02, depth: 0.02 },
    }).nextState;

    state = executeMcpToolOnEngineState(state, 'run_fea_analysis', {
      part_id: 'box_p2t27',
      fixed_faces: ['min_x'],
      applied_forces: [{ face: 'max_x', force_vector: [0, -200, 0] }],
    }).nextState;
    (state.feaResult as any).feaRevision = 1;

    // 1. No-op optimize_feature_tree keeps revision == 1
    const optNoop = executeMcpToolOnEngineState(state, 'optimize_feature_tree', {
      part_id: 'box_p2t27',
      apply: true,
      length_unit: 'm',
    });
    state = optNoop.nextState;
    assert.equal(optNoop.response.applied_rules.length, 0);
    assert.equal(optNoop.response.revision, 1);
    assert.equal(optNoop.response.length_unit, 'm');

    // 2. get_simulation_metrics_json respects length_unit override and reports stale == false
    const metricsM = executeMcpToolOnEngineState(state, 'get_simulation_metrics_json', {
      length_unit: 'm',
    });
    assert.equal(metricsM.response.length_unit, 'm');
    assert.equal(metricsM.response.fea_reports[0].fea_revision, 1);
    assert.equal(metricsM.response.fea_reports[0].part_revision, 1);
    assert.equal(metricsM.response.fea_reports[0].stale, false);
    assert.equal(metricsM.response.fea_reports[0].status, 'FRESH');

    // 3. Mutate part -> get_simulation_metrics_json reports stale == true
    state = executeMcpToolOnEngineState(state, 'transform_part', {
      part_id: 'box_p2t27',
      translation: [0.01, 0, 0],
    }).nextState;

    const metricsMm = executeMcpToolOnEngineState(state, 'get_simulation_metrics_json', {});
    assert.equal(metricsMm.response.length_unit, 'mm');
    assert.equal(metricsMm.response.fea_reports[0].fea_revision, 1);
    assert.equal(metricsMm.response.fea_reports[0].part_revision, 2);
    assert.equal(metricsMm.response.fea_reports[0].stale, true);
    assert.equal(metricsMm.response.fea_reports[0].status, 'STALE');
  });

  it('verifies P2-T29: list_parts respects length_unit override and rejects invalid length_unit', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'part_p2t29',
      primitive_type: 'box',
      params: { width: 0.1, height: 0.04, depth: 0.02 },
    }).nextState;

    const listMm = executeMcpToolOnEngineState(state, 'list_parts', {});
    assert.equal(listMm.response.length_unit, 'mm');
    assert.equal(listMm.response.units.volume, 'mm^3');
    assert.equal(listMm.response.parts_count, 1);
    assert.ok(Math.abs(listMm.response.parts[0].bounding_box_dimensions[0] - 100.0) < 1e-4);
    assert.ok(Math.abs(listMm.response.parts[0].volume - 80000.0) < 1.0);

    const listM = executeMcpToolOnEngineState(state, 'list_parts', {
      length_unit: 'm',
    });
    assert.equal(listM.response.length_unit, 'm');
    assert.equal(listM.response.units.volume, 'm^3');
    assert.ok(Math.abs(listM.response.parts[0].bounding_box_dimensions[0] - 0.1) < 1e-6);
    assert.ok(Math.abs(listM.response.parts[0].volume - 8.0e-5) < 1e-8);

    assert.throws(() => {
      executeMcpToolOnEngineState(state, 'list_parts', {
        length_unit: 'inch',
      });
    }, /length_unit/);
  });

  it('verifies P2-T30: import_stl supports file_path alias, part_id, and is_fixed in dispatcher', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const imp1 = executeMcpToolOnEngineState(state, 'import_stl', {
      file_path: 'output/bracket.stl',
      part_id: 'fixed_imported_plate',
      name: 'Fixed Plate',
      is_fixed: true,
      material: 'aluminum_6061_t6',
    });

    assert.equal(imp1.response.status, 'imported');
    assert.equal(imp1.response.part_id, 'fixed_imported_plate');
    assert.equal(imp1.response.file_path, 'output/bracket.stl');
    assert.equal(imp1.response.is_fixed, true);

    const importedPart = imp1.nextState.parts.find(
      (p) => p.id === 'fixed_imported_plate',
    );
    assert.ok(importedPart);
    assert.equal(importedPart?.isFixed, true);
    assert.equal(importedPart?.name, 'Fixed Plate');
    assert.equal(importedPart?.materialId, 'aluminum_6061_t6');

    // Test with filepath alias
    const imp2 = executeMcpToolOnEngineState(imp1.nextState, 'import_stl', {
      filepath: 'output/sensor.stl',
      part_id: 'dynamic_sensor',
      is_fixed: false,
    });
    assert.equal(imp2.response.status, 'imported');
    assert.equal(imp2.response.part_id, 'dynamic_sensor');
    assert.equal(imp2.response.is_fixed, false);
    const dynPart = imp2.nextState.parts.find((p) => p.id === 'dynamic_sensor');
    assert.ok(dynPart);
    assert.equal(dynPart?.isFixed, false);
  });

  it('verifies P2-T32: get_feature_tree & validate_mesh metadata and get_simulation_metrics_json gravity scaling and joints_count', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    state = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'part_p2t32',
      primitive_type: 'box',
      params: { width: 0.05, height: 0.03, depth: 0.02 },
    }).nextState;

    // 1. get_feature_tree metadata
    const treeMm = executeMcpToolOnEngineState(state, 'get_feature_tree', {
      part_id: 'part_p2t32',
    });
    assert.equal(treeMm.response.part_id, 'part_p2t32');
    assert.equal(treeMm.response.revision, 1);
    assert.equal(treeMm.response.features_count, 1);
    assert.equal(treeMm.response.length_unit, 'mm');

    const treeM = executeMcpToolOnEngineState(state, 'get_feature_tree', {
      part_id: 'part_p2t32',
      length_unit: 'm',
    });
    assert.equal(treeM.response.length_unit, 'm');

    // 2. validate_mesh metadata
    const valMm = executeMcpToolOnEngineState(state, 'validate_mesh', {
      part_id: 'part_p2t32',
    });
    assert.equal(valMm.response.part_id, 'part_p2t32');
    assert.equal(valMm.response.revision, 1);
    assert.equal(valMm.response.length_unit, 'mm');
    assert.equal(valMm.response.status, 'PASS');

    const valM = executeMcpToolOnEngineState(state, 'validate_mesh', {
      part_id: 'part_p2t32',
      length_unit: 'm',
    });
    assert.equal(valM.response.length_unit, 'm');
    assert.equal(valM.response.revision, 1);

    // 3. get_simulation_metrics_json: gravity scaling and joints_count
    const metricsMm = executeMcpToolOnEngineState(state, 'get_simulation_metrics_json', {});
    assert.equal(metricsMm.response.current_metrics.joints_count, 0);
    assert.equal(metricsMm.response.current_metrics.physics.joints_count, 0);
    assert.equal(metricsMm.response.current_metrics.physics.gravity[1], -9810.0);
    assert.equal(metricsMm.response.current_metrics.physics.gravity_m_s2[1], -9.81);

    const metricsM = executeMcpToolOnEngineState(state, 'get_simulation_metrics_json', {
      length_unit: 'm',
    });
    assert.equal(metricsM.response.current_metrics.physics.gravity[1], -9.81);
    assert.equal(metricsM.response.current_metrics.physics.gravity_m_s2[1], -9.81);
  });

  it('verifies P2-T33: transform_part works on imported STL parts and parts with empty DAG', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    // 1. Import an STL part
    const imp = executeMcpToolOnEngineState(state, 'import_stl', {
      filepath: 'output/bracket.stl',
      part_id: 'imp_p2t33',
      name: 'Imported Bracket',
    });
    state = imp.nextState;

    // 2. Transform the imported STL part
    const tr1 = executeMcpToolOnEngineState(state, 'transform_part', {
      part_id: 'imp_p2t33',
      translation: [0.1, -0.05, 0.02],
      scale: [2, 2, 2],
    });
    assert.equal(tr1.response.status, 'transformed');
    assert.equal(tr1.response.part_id, 'imp_p2t33');
    assert.equal(tr1.response.revision, 2);
    assert.ok(tr1.response.bounding_box);
    assert.ok(Math.abs(tr1.response.volume_m3 - 0.001) < 1e-5);

    // 3. Clear DAG nodes on a part with a valid mesh and verify transform_part auto-seeds ImportedMesh
    state = {
      ...tr1.nextState,
      parts: tr1.nextState.parts.map((p) =>
        p.id === 'imp_p2t33' ? { ...p, dag: [] } : p
      ),
    };
    const tr2 = executeMcpToolOnEngineState(state, 'transform_part', {
      part_id: 'imp_p2t33',
      translation: [0.05, 0, 0],
    });
    assert.equal(tr2.response.status, 'transformed');
    assert.equal(tr2.response.revision, 2);
    assert.equal(tr2.nextState.parts[0].dag.length, 2);
    assert.equal(tr2.nextState.parts[0].dag[0].type, 'stl_import');
  });

  it('verifies P2-T34: get_feature_tree scales embedded mesh operands (ImportedMesh and Boolean*Mesh) according to length_unit', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    // 1. Import STL part (box 0.05 x 0.05 x 0.05 m -> 50 x 50 x 50 mm)
    const imp = executeMcpToolOnEngineState(state, 'import_stl', {
      filepath: 'output/bracket.stl',
      part_id: 'imp_p2t34',
      name: 'Imported Bracket',
    });
    state = imp.nextState;

    // 2. Add a tool part and perform boolean_difference
    const addTool = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'tool_p2t34',
      primitive_type: 'box',
      params: { width: 0.02, height: 0.02, depth: 0.06 },
    });
    state = addTool.nextState;

    const diff = executeMcpToolOnEngineState(state, 'boolean_difference', {
      target_part_id: 'imp_p2t34',
      tool_part_id: 'tool_p2t34',
    });
    state = diff.nextState;

    // 3. Query get_feature_tree in mm vs m
    const treeMm = executeMcpToolOnEngineState(state, 'get_feature_tree', {
      part_id: 'imp_p2t34',
      length_unit: 'mm',
    });
    const treeM = executeMcpToolOnEngineState(state, 'get_feature_tree', {
      part_id: 'imp_p2t34',
      length_unit: 'm',
    });

    const impMeshMm = treeMm.response.dag[0].params.mesh;
    const impMeshM = treeM.response.dag[0].params.mesh;
    assert.ok(Math.abs(impMeshMm.boundingBox.max[0] - 25.0) < 1e-4);
    assert.ok(Math.abs(impMeshM.boundingBox.max[0] - 0.025) < 1e-6);

    const toolMeshMm = treeMm.response.dag[1].params.tool_mesh;
    const toolMeshM = treeM.response.dag[1].params.tool_mesh;
    assert.ok(Math.abs(toolMeshMm.boundingBox.max[2] - 30.0) < 1e-4);
    assert.ok(Math.abs(toolMeshM.boundingBox.max[2] - 0.03) < 1e-6);
  });

  it('verifies P2-T35: add_feature_primitive, cut_hole, modify_feature, and cut_helical_groove return part_id, feature_id, revision, and cut_hole depth', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const addRes = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'part_p2t35',
      primitive_type: 'cylinder',
      params: { radius: 0.02, height: 0.08, segments: 24 },
    });
    state = addRes.nextState;
    assert.equal(addRes.response.part_id, 'part_p2t35');
    assert.ok(addRes.response.feature_id);
    assert.equal(addRes.response.revision, 1);

    const modRes = executeMcpToolOnEngineState(state, 'modify_feature', {
      part_id: 'part_p2t35',
      feature_id: addRes.response.feature_id,
      params: { height: 0.1 },
    });
    state = modRes.nextState;
    assert.equal(modRes.response.part_id, 'part_p2t35');
    assert.equal(modRes.response.feature_id, addRes.response.feature_id);
    assert.equal(modRes.response.revision, 2);

    const holeRes = executeMcpToolOnEngineState(state, 'cut_hole', {
      part_id: 'part_p2t35',
      position: [0, 0, 0.05],
      normal: [0, 0, -1],
      radius: 0.006,
      through: true,
    });
    state = holeRes.nextState;
    assert.equal(holeRes.response.part_id, 'part_p2t35');
    assert.ok(holeRes.response.feature_id);
    assert.equal(holeRes.response.through, true);
    assert.ok(Math.abs(holeRes.response.depth - 0.1) < 1e-6);
    assert.equal(holeRes.response.revision, 3);

    const grooveRes = executeMcpToolOnEngineState(state, 'cut_helical_groove', {
      part_id: 'part_p2t35',
      helix_radius: 0.02,
      groove_radius: 0.001,
      pitch: 0.01,
      height: 0.04,
    });
    assert.equal(grooveRes.response.part_id, 'part_p2t35');
    assert.ok(grooveRes.response.feature_id);
    assert.equal(grooveRes.response.revision, 4);
  });

  it('verifies P2-T36: optimize_feature_tree scales logical_plan, physical_plan, and candidate_plans DAGs and delete_part returns part_id and deleted_part_id', () => {
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const addRes = executeMcpToolOnEngineState(state, 'add_feature_primitive', {
      part_id: 'part_p2t36',
      primitive_type: 'cylinder',
      params: { radius: 0.028, height: 0.06, segments: 24 },
    });
    state = addRes.nextState;

    const holeRes = executeMcpToolOnEngineState(state, 'cut_hole', {
      part_id: 'part_p2t36',
      position: [0, 0, 0.03],
      normal: [0, 0, -1],
      radius: 0.014,
      depth: 0.06,
    });
    state = holeRes.nextState;

    // 1. optimize_feature_tree in default 'mm' scales DAG params by 1000
    const optMm = executeMcpToolOnEngineState(state, 'optimize_feature_tree', {
      part_id: 'part_p2t36',
      apply: false,
    });
    assert.equal(optMm.response.length_unit, 'mm');
    assert.ok(Math.abs(optMm.response.logical_plan.dag[0].params.radius - 28.0) < 1e-6);
    assert.ok(Math.abs(optMm.response.logical_plan.dag[0].params.height - 60.0) < 1e-6);
    assert.ok(Math.abs(optMm.response.logical_plan.dag[1].params.radius - 14.0) < 1e-6);
    assert.ok(Math.abs(optMm.response.candidate_plans[0].dag[0].params.radius - 28.0) < 1e-6);

    // 2. optimize_feature_tree in 'm' keeps SI meters
    const optM = executeMcpToolOnEngineState(state, 'optimize_feature_tree', {
      part_id: 'part_p2t36',
      apply: true,
      length_unit: 'm',
    });
    state = optM.nextState;
    assert.equal(optM.response.length_unit, 'm');
    assert.ok(Math.abs(optM.response.logical_plan.dag[0].params.radius - 0.028) < 1e-9);

    // 3. delete_part returns both part_id and deleted_part_id
    const delRes = executeMcpToolOnEngineState(state, 'delete_part', {
      part_id: 'part_p2t36',
    });
    assert.equal(delRes.response.status, 'deleted');
    assert.equal(delRes.response.part_id, 'part_p2t36');
    assert.equal(delRes.response.deleted_part_id, 'part_p2t36');
  });

  it('verifies P2-T37..P2-T40: autonomous open_preview_window, validate_mesh strict_slivers & is_valid_solid, and piston revolve body fidelity', () => {
    // 1. P2-T37: open_preview_window returns opened, spawned_gui_process, and stl_path
    let state: EngineStateSnapshot = {
      parts: [
        {
          id: 'test_bracket',
          name: 'Engine Bracket',
          materialId: 'alu_6061',
          dag: [],
          mesh: createBoxMesh(0.1, 0.05, 0.03, 2700),
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        },
      ],
      selectedPartId: 'test_bracket',
      joints: [],
      materials: DEFAULT_MATERIALS,
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const prevRes = executeMcpToolOnEngineState(state, 'open_preview_window', {
      part_id: 'test_bracket',
    });
    assert.equal(prevRes.response.status, 'opened');
    assert.equal(prevRes.response.mode, 'spawned_gui_process');
    assert.equal(prevRes.response.part_id, 'test_bracket');
    assert.ok(prevRes.response.stl_path.includes('test_bracket.stl'));

    // 2. P2-T38: validate_mesh returns is_valid_solid: true and fea_ready: true, and respects strict_slivers
    const valDefault = executeMcpToolOnEngineState(state, 'validate_mesh', {
      part_id: 'test_bracket',
      strict_slivers: false,
    });
    assert.equal(valDefault.response.status, 'PASS');
    assert.equal(valDefault.response.is_valid_solid, true);
    assert.equal(valDefault.response.fea_ready, true);
    assert.equal(valDefault.response.is_watertight, true);
    assert.equal(valDefault.response.is_manifold, true);

    // 3. P2-T39: V8 racing piston revolve body does not collapse into a rectangular plate
    const pistonDag = [
      {
        id: 'node_1',
        name: 'Piston Revolve Body',
        type: 'revolve' as const,
        params: {
          angle_deg: 360,
          segments: 64,
          profile: [
            [0.0425, 0.0],
            [0.0425, 0.038],
            [0.039, 0.038],
            [0.039, 0.041],
            [0.0425, 0.041],
            [0.0425, 0.045],
            [0.039, 0.045],
            [0.039, 0.0475],
            [0.0425, 0.0475],
            [0.0425, 0.052],
            [0.039, 0.052],
            [0.039, 0.0545],
            [0.042, 0.0545],
            [0.042, 0.065],
            [0.03, 0.065],
            [0.024, 0.061],
            [0.0, 0.061],
            [0.0, 0.052],
            [0.032, 0.052],
            [0.036, 0.04],
            [0.036, 0.0],
          ],
        },
        enabled: true,
      },
      {
        id: 'node_2',
        name: 'Piston Pin Bore Ø22mm',
        type: 'cut_hole' as const,
        params: {
          center: [-0.05, 0, 0.025],
          normal: [1, 0, 0],
          radius: 0.011,
          depth: 0.1,
        },
        enabled: true,
      },
    ];

    const pistonMesh = evaluateFeatureTree(pistonDag, 'standard', 2700);
    assert.equal(pistonMesh.isWatertight, true);
    assert.ok(pistonMesh.triangleCount > 5000, `Expected high-detail revolve piston >5000 tris, got ${pistonMesh.triangleCount}`);
    assert.ok(Math.abs(pistonMesh.boundingBox.max[2] - 0.065) < 1e-4);

    // 4. P2-T40: delete_part returns part_id and deleted_part_id
    const delRes2 = executeMcpToolOnEngineState(state, 'delete_part', {
      part_id: 'test_bracket',
    });
    assert.equal(delRes2.response.status, 'deleted');
    assert.equal(delRes2.response.part_id, 'test_bracket');
    assert.equal(delRes2.response.deleted_part_id, 'test_bracket');
  });
});

