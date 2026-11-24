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
  exportMeshToStlBinary,
  parseStlBuffer,
  validatePartMesh,
} from './primitives';
import { runLinearFeaSolver } from './fea_solver';
import { stepKinematicsAssembly } from './kinematics';
import {
  DEFAULT_MATERIALS,
  MATERIALS_LIBRARY,
  assertValidMaterialSpec,
  getMaterialById,
  normalizeMaterialSpec,
  resolveMaterialAlias,
  validateMaterialSpec,
} from './materials';
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
  DEFAULT_BRIDGE_STATUS,
  GOLDEN_WINDOWS_MAPPINGS,
  RUST_MCP_TOOLS_COUNT,
  TS_MCP_TOOLS_COUNT,
  buildStandaloneNativeStlPart,
  getWindowsWgpuArtifactUrl,
  hydrateImportedStlPartInScene,
  resolveNativeStlMaterialSpec,
} from './native_bridge';
import { detectPressAssembly } from './press_simulation';
import { ENGINEERING_WORKBENCHES } from './workbenches';

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


  it('verifies BUG-015: 31 TS MCP tools catalog including validate_mesh, mechanism tools, and gear joint schema parity', () => {
    assert.equal(MCP_TOOLS_CATALOG.length, 31);
    assert.ok(MCP_TOOLS_CATALOG.some((t) => t.name === 'validate_mesh'));
    assert.ok(MCP_TOOLS_CATALOG.some((t) => t.name === 'analyze_mechanism'));
    assert.ok(MCP_TOOLS_CATALOG.some((t) => t.name === 'check_interference'));
    assert.ok(MCP_TOOLS_CATALOG.some((t) => t.name === 'sweep_joint'));

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
    const squashedDefaultReport = validatePartMesh('squashed_cyl', squashed);
    assert.equal(squashedDefaultReport.status, 'PASS');
    assert.ok(squashedDefaultReport.warnings.length > 0);
    const squashedReport = validatePartMesh('squashed_cyl', squashed, undefined, true);
    assert.ok(squashedReport.lt5DegRatio > 0.01 || squashedReport.lt1DegCount > 0);
    assert.equal(squashedReport.status, 'FAIL', 'Sliver-contaminated mesh must fail strict validation gate');
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

  it('verifies BUG-021: 43 Rust / 31 TS MCP tools documentation & contract parity across README, setup guides, architecture, PROJECT_CONTEXT, and API reference', () => {
    // 1. Verify exactly 31 tools in TS catalog with unique names (28 baseline + 3 P3-T9/T10 kinematics tools)
    assert.equal(MCP_TOOLS_CATALOG.length, 31, 'Catalog must contain exactly 31 TS tools');
    const catalogNames = new Set(MCP_TOOLS_CATALOG.map((t) => t.name));
    assert.equal(catalogNames.size, 31, 'All 31 tool names must be unique');
    assert.equal(TS_MCP_TOOLS_COUNT, 31, 'TS_MCP_TOOLS_COUNT must equal 31');
    assert.equal(RUST_MCP_TOOLS_COUNT, 43, 'RUST_MCP_TOOLS_COUNT must equal 43');
    assert.equal(DEFAULT_BRIDGE_STATUS.activeToolsCount, 31, 'DEFAULT_BRIDGE_STATUS must report 31 TS tools in hybrid mode');

    // Expected 28 baseline tools + 3 kinematics tools in TS catalog
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
    const EXPECTED_31_TS_TOOLS = [
      ...EXPECTED_28_TOOLS,
      'analyze_mechanism',
      'check_interference',
      'sweep_joint',
    ];
    const EXPECTED_43_RUST_TOOLS = [
      ...EXPECTED_31_TS_TOOLS,
      'set_units',
      'save_project',
      'load_project',
      'set_requirements',
      'check_requirements',
      'get_scene_summary',
      'get_part_load_cases',
      'run_fea_from_simulation',
      'check_gear_strength',
      'apply_recipe',
      'bake_coupled_simulation',
      'query_simulation_frame',
    ];

    for (const tool of EXPECTED_31_TS_TOOLS) {
      assert.ok(catalogNames.has(tool), `Missing tool '${tool}' in MCP catalog`);
    }

    // 2. Automated Contract check against README.md (all 43 Rust MCP tools)
    const readmeContent = fs.readFileSync(path.resolve(process.cwd(), 'README.md'), 'utf-8');
    assert.ok(readmeContent.includes('43 Tools'), 'README.md must state 43 Tools');
    assert.ok(readmeContent.includes('43 инструмент'), 'README.md must state 43 инструмент');
    for (const tool of EXPECTED_43_RUST_TOOLS) {
      assert.ok(
        readmeContent.includes(`\`${tool}\``),
        `README.md missing documented entry for tool '${tool}'`
      );
    }
    assert.ok(readmeContent.includes('is_fixed'), 'README must document is_fixed parameter');
    assert.ok(readmeContent.includes('gear'), 'README must document gear joint');
    assert.ok(readmeContent.includes('debug_mode'), 'README must document debug_mode parameter');

    // 3. Automated Contract check against docs/CLAUDE_DESKTOP_SETUP.md (all 43 Rust MCP tools)
    const claudeContent = fs.readFileSync(path.resolve(process.cwd(), 'docs/CLAUDE_DESKTOP_SETUP.md'), 'utf-8');
    assert.ok(claudeContent.includes('43 инженерных инструмента'), 'Claude setup must state 43 tools');
    for (const tool of EXPECTED_43_RUST_TOOLS) {
      assert.ok(
        claudeContent.includes(`\`${tool}\``),
        `docs/CLAUDE_DESKTOP_SETUP.md missing tool '${tool}'`
      );
    }

    // 4. Automated Contract check against docs/ANTIGRAVITY_SETUP.md
    const antigravityContent = fs.readFileSync(path.resolve(process.cwd(), 'docs/ANTIGRAVITY_SETUP.md'), 'utf-8');
    assert.ok(
      antigravityContent.includes('43 инженерного инструмента') || antigravityContent.includes('43 инженерных инструментов') || antigravityContent.includes('43 инженерных инструмента'),
      'Antigravity setup must state 43 tools'
    );

    // 5. Automated Contract check against docs/ARCHITECTURE.md
    const archContent = fs.readFileSync(path.resolve(process.cwd(), 'docs/ARCHITECTURE.md'), 'utf-8');
    assert.ok(archContent.includes('43 инструмента MCP') || archContent.includes('43 инструментов MCP'), 'ARCHITECTURE.md must state 43 MCP tools');
    assert.ok(archContent.includes('43 строгого JSON-Schema') || archContent.includes('43 строгих JSON-Schema'), 'ARCHITECTURE.md must state 43 JSON-Schema tool definitions');
    assert.ok(archContent.includes('Gear'), 'ARCHITECTURE.md must document Gear joint');
    assert.ok(archContent.includes('WheelRim'), 'ARCHITECTURE.md must document WheelRim primitive');
    assert.ok(archContent.includes('SpurGear'), 'ARCHITECTURE.md must document SpurGear primitive');
    assert.ok(archContent.includes('src/cad'), 'ARCHITECTURE.md must document TypeScript CAD layer');

    // 6. Automated Contract check against docs/MCP_API_REFERENCE.md and PROJECT_CONTEXT.md (all 43 Rust tools)
    const apiRefContent = fs.readFileSync(path.resolve(process.cwd(), 'docs/MCP_API_REFERENCE.md'), 'utf-8');
    const projectCtxContent = fs.readFileSync(path.resolve(process.cwd(), 'PROJECT_CONTEXT.md'), 'utf-8');
    for (const tool of EXPECTED_43_RUST_TOOLS) {
      assert.ok(
        apiRefContent.includes(`\`${tool}\``),
        `docs/MCP_API_REFERENCE.md missing tool '${tool}'`
      );
      assert.ok(
        projectCtxContent.includes(`\`${tool}\``),
        `PROJECT_CONTEXT.md missing tool '${tool}'`
      );
    }

    // 7. Verify TS workbenches metadata reflects current tool counts without stale '28'
    const optimizerBench = ENGINEERING_WORKBENCHES.find((w) => w.id === 'optimizer');
    assert.ok(optimizerBench);
    assert.ok(!optimizerBench.subtitle.includes('28'), 'Optimizer workbench subtitle must not contain stale 28 count');
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

    // Default length_unit is 'mm' (matches Rust create_joint contract: [-500, 500] mm -> [-0.5, 0.5] m)
    const jPrism = executeMcpToolOnEngineState(state, 'create_joint', {
      joint_id: 'j_prism_p2_t15',
      joint_type: 'prismatic',
      part_a: 'ground',
      part_b: 'slider_carriage',
      axis: [1, 0, 0],
      limits: [-500, 500],
    });
    assert.equal(jPrism.response.length_unit, 'mm');
    state = jPrism.nextState;
    const prismJoint = state.joints.find((j) => j.id === 'j_prism_p2_t15')!;
    assert.deepEqual(prismJoint.limits, [-0.5, 0.5]);

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

  it('verifies P4-T6: workbenches and MCP tools catalog reference f64 DAE solver without legacy 3rd-party engine labels', () => {
    const legacyPattern = new RegExp('rap' + 'ier', 'i');
    const simWb = ENGINEERING_WORKBENCHES.find((w) => w.id === 'simulation');
    assert.ok(simWb, 'simulation workbench must exist');
    assert.ok(
      simWb.subtitle.includes('f64 DAE'),
      `simulation workbench subtitle should mention f64 DAE, got: ${simWb.subtitle}`
    );
    assert.ok(
      !legacyPattern.test(simWb.subtitle),
      `simulation workbench subtitle must not mention legacy engine, got: ${simWb.subtitle}`
    );

    for (const tool of MCP_TOOLS_CATALOG) {
      assert.ok(
        !legacyPattern.test(tool.description),
        `MCP tool '${tool.name}' description must not mention legacy engine, got: ${tool.description}`
      );
    }
  });

  it('verifies P5-T1: all 10 standard materials carry valid J2 plasticity, fracture, and friction constants', () => {
    const requiredIds = [
      'steel_1045',
      'steel_structural_aisi1020',
      'steel_4140',
      'alu_6061',
      'ti_6al_4v',
      'brass_c360',
      'pla_3d',
      'petg_3d',
      'nylon_pa12',
      'borosilicate_glass',
    ];
    assert.equal(MATERIALS_LIBRARY.length, 10);

    for (const id of requiredIds) {
      const mat = getMaterialById(id);
      assert.equal(mat.id, id, `Expected material '${id}' in MATERIALS_LIBRARY`);
      const check = validateMaterialSpec(mat);
      assert.equal(
        check.valid,
        true,
        `Material '${id}' failed physical invariant validation: ${check.errors.join('; ')}`
      );
      assert.ok(mat.ultimateStrengthPa >= mat.yieldStrength);
      assert.ok(mat.hardeningModulusPa >= 0);
      assert.ok(mat.fractureStrain > 0);
      assert.ok(mat.fractureEnergyJM2 > 0);
      assert.ok(mat.frictionCoeff >= 0);
      assert.ok(mat.poissonsRatio > 0 && mat.poissonsRatio < 0.5);
    }

    // Physical class ordering checks
    const steel1020 = getMaterialById('steel_structural_aisi1020');
    const steel1045 = getMaterialById('steel_1045');
    const steel4140 = getMaterialById('steel_4140');
    const glass = getMaterialById('borosilicate_glass');
    assert.ok(steel4140.yieldStrength > steel1045.yieldStrength);
    assert.ok(steel1045.yieldStrength > steel1020.yieldStrength);
    assert.equal(steel4140.failureMode, 'ductile');
    assert.equal(glass.failureMode, 'brittle');
    assert.equal(glass.hardeningModulusPa, 0);
    assert.ok(steel4140.fractureEnergyJM2 > 1000 * glass.fractureEnergyJM2);
    assert.equal(getMaterialById('pyrex').id, 'borosilicate_glass');
    assert.equal(getMaterialById('glass').id, 'borosilicate_glass');
  });

  it('verifies P5-T11: steel_1045 and steel_structural_aisi1020 are distinct and all 10 presets match Rust MaterialRegistry exactly', () => {
    const steel1045 = getMaterialById('steel_1045');
    const steel1020 = getMaterialById('steel_structural_aisi1020');
    assert.equal(steel1045.id, 'steel_1045');
    assert.equal(steel1020.id, 'steel_structural_aisi1020');
    assert.equal(steel1045.yieldStrength, 350e6);
    assert.equal(steel1045.ultimateStrengthPa, 585e6);
    assert.equal(steel1045.hardeningModulusPa, 1.8e9);
    assert.equal(steel1045.fractureStrain, 0.16);
    assert.equal(steel1020.yieldStrength, 250e6);
    assert.equal(steel1020.ultimateStrengthPa, 420e6);
    assert.equal(steel1020.hardeningModulusPa, 1.2e9);
    assert.equal(steel1020.fractureStrain, 0.22);

    assert.equal(resolveMaterialAlias('steel_carbon_aisi1045'), 'steel_1045');
    assert.equal(resolveMaterialAlias('AISI 1045'), 'steel_1045');
    assert.equal(resolveMaterialAlias('steel_1020'), 'steel_structural_aisi1020');
    assert.equal(resolveMaterialAlias('AISI 1020'), 'steel_structural_aisi1020');
    assert.equal(resolveMaterialAlias('steel'), 'steel_structural_aisi1020');

    // Cross-check all standard presets and aliases directly against Rust crates/mcp-cad-core/src/material.rs
    const rustMaterialPath = path.resolve(
      process.cwd(),
      'crates/mcp-cad-core/src/material.rs'
    );
    const rustSrc = fs.readFileSync(rustMaterialPath, 'utf8');

    const regStart = rustSrc.indexOf('fn register_presets(&mut self)');
    const regEnd = rustSrc.indexOf('pub fn resolve_alias', regStart);
    assert.ok(regStart >= 0 && regEnd > regStart, 'Could not locate register_presets in material.rs');
    const regBody = rustSrc.slice(regStart, regEnd);

    const parseRustFloat = (s: string): number => Number(s.replace(/_/g, '').trim());
    const presetRegex =
      /Material::new\(\s*"([^"]+)",\s*([\d._eE+-]+),\s*([\d._eE+-]+),\s*([\d._eE+-]+),\s*([\d._eE+-]+),\s*"[^"]*",?\s*\)\s*\.with_nonlinear_properties\(\s*([\d._eE+-]+),\s*([\d._eE+-]+),\s*([\d._eE+-]+),\s*([\d._eE+-]+),\s*([\d._eE+-]+),\s*FailureMode::(Ductile|Brittle),?\s*\)/g;

    interface RustMatParsed {
      name: string;
      youngsModulusPa: number;
      poissonsRatio: number;
      yieldStrengthPa: number;
      densityKgM3: number;
      ultimateStrengthPa: number;
      hardeningModulusPa: number;
      fractureStrain: number;
      fractureEnergyJM2: number;
      frictionCoeff: number;
      failureMode: 'ductile' | 'brittle';
    }

    const rustPresets = new Map<string, RustMatParsed>();
    for (const match of regBody.matchAll(presetRegex)) {
      const parsed: RustMatParsed = {
        name: match[1],
        youngsModulusPa: parseRustFloat(match[2]) * 1e9,
        poissonsRatio: parseRustFloat(match[3]),
        yieldStrengthPa: parseRustFloat(match[4]) * 1e6,
        densityKgM3: parseRustFloat(match[5]),
        ultimateStrengthPa: parseRustFloat(match[6]),
        hardeningModulusPa: parseRustFloat(match[7]),
        fractureStrain: parseRustFloat(match[8]),
        fractureEnergyJM2: parseRustFloat(match[9]),
        frictionCoeff: parseRustFloat(match[10]),
        failureMode: match[11].toLowerCase() as 'ductile' | 'brittle',
      };
      rustPresets.set(parsed.name, parsed);
    }

    assert.equal(
      rustPresets.size,
      MATERIALS_LIBRARY.length,
      `Rust preset count (${rustPresets.size}) must equal TS MATERIALS_LIBRARY length (${MATERIALS_LIBRARY.length})`
    );

    const assertClose = (label: string, a: number, b: number) => {
      const scale = Math.max(Math.abs(a), Math.abs(b), 1);
      assert.ok(
        Math.abs(a - b) <= 1e-9 * scale,
        `Rust/TS mismatch on ${label}: Rust=${a} vs TS=${b}`
      );
    };

    const matchedTsIds = new Set<string>();
    for (const [rustId, rustMat] of rustPresets.entries()) {
      const tsMat = getMaterialById(rustId);
      assert.ok(
        !matchedTsIds.has(tsMat.id),
        `Rust preset '${rustId}' collided with already-mapped TS preset '${tsMat.id}'`
      );
      matchedTsIds.add(tsMat.id);

      assertClose(`${rustId}.density`, rustMat.densityKgM3, tsMat.density);
      assertClose(`${rustId}.youngsModulus`, rustMat.youngsModulusPa, tsMat.youngsModulus);
      assertClose(`${rustId}.poissonsRatio`, rustMat.poissonsRatio, tsMat.poissonsRatio);
      assertClose(`${rustId}.yieldStrength`, rustMat.yieldStrengthPa, tsMat.yieldStrength);
      assertClose(`${rustId}.ultimateStrengthPa`, rustMat.ultimateStrengthPa, tsMat.ultimateStrengthPa);
      assertClose(`${rustId}.hardeningModulusPa`, rustMat.hardeningModulusPa, tsMat.hardeningModulusPa);
      assertClose(`${rustId}.fractureStrain`, rustMat.fractureStrain, tsMat.fractureStrain);
      assertClose(`${rustId}.fractureEnergyJM2`, rustMat.fractureEnergyJM2, tsMat.fractureEnergyJM2);
      assertClose(`${rustId}.frictionCoeff`, rustMat.frictionCoeff, tsMat.frictionCoeff);
      assert.equal(
        rustMat.failureMode,
        tsMat.failureMode,
        `Rust/TS failureMode mismatch for ${rustId}`
      );
    }

    // Verify every alias defined in Rust resolve_alias resolves to the physically identical preset in TS
    const aliasEnd = rustSrc.indexOf('pub fn register(', regEnd);
    assert.ok(aliasEnd > regEnd, 'Could not locate end of resolve_alias in material.rs');
    const aliasBody = rustSrc.slice(regEnd, aliasEnd);
    const armRegex = /((?:"[^"]+"\s*\|\s*)*"[^"]+")\s*=>\s*(?:\{\s*)?"([^"]+)"/g;
    let checkedRustAliases = 0;
    for (const arm of aliasBody.matchAll(armRegex)) {
      const targetRustId = arm[2];
      const rustMat = rustPresets.get(targetRustId);
      assert.ok(rustMat, `Rust alias target '${targetRustId}' not found in register_presets`);
      for (const aliasMatch of arm[1].matchAll(/"([^"]+)"/g)) {
        const alias = aliasMatch[1];
        const tsMat = getMaterialById(alias);
        assertClose(`alias(${alias}).yieldStrength`, rustMat.yieldStrengthPa, tsMat.yieldStrength);
        assertClose(`alias(${alias}).ultimateStrengthPa`, rustMat.ultimateStrengthPa, tsMat.ultimateStrengthPa);
        assertClose(`alias(${alias}).hardeningModulusPa`, rustMat.hardeningModulusPa, tsMat.hardeningModulusPa);
        assertClose(`alias(${alias}).fractureStrain`, rustMat.fractureStrain, tsMat.fractureStrain);
        checkedRustAliases++;
      }
    }
    assert.ok(
      checkedRustAliases >= 50,
      `Expected full Rust resolve_alias coverage, checked only ${checkedRustAliases} aliases`
    );
  });

  it('verifies P5-T1: validateMaterialSpec rejects invariant violations and normalizeMaterialSpec supports legacy payloads', () => {
    const base = getMaterialById('steel_1045');
    assert.equal(validateMaterialSpec(base).valid, true);

    assert.equal(
      validateMaterialSpec({ ...base, ultimateStrengthPa: base.yieldStrength * 0.8 }).valid,
      false
    );
    assert.equal(validateMaterialSpec({ ...base, hardeningModulusPa: -1 }).valid, false);
    assert.equal(validateMaterialSpec({ ...base, fractureStrain: 0 }).valid, false);
    assert.equal(validateMaterialSpec({ ...base, fractureEnergyJM2: -5 }).valid, false);
    assert.equal(validateMaterialSpec({ ...base, poissonsRatio: 0.5 }).valid, false);
    assert.equal(validateMaterialSpec({ ...base, poissonsRatio: 0 }).valid, false);
    assert.equal(validateMaterialSpec({ ...base, frictionCoeff: -0.05 }).valid, false);

    // Legacy 6-field material (high yield titanium without Phase 5 fields)
    const legacy = normalizeMaterialSpec({
      id: 'legacy_ti',
      name: 'Legacy Titanium',
      category: 'Metal',
      density: 4430,
      youngsModulus: 114e9,
      poissonsRatio: 0.34,
      yieldStrength: 880e6,
    });
    assert.equal(validateMaterialSpec(legacy).valid, true);
    assert.ok(legacy.ultimateStrengthPa >= 880e6);
    assert.equal(legacy.failureMode, 'ductile');

    // Rust snake_case JSON payload compatibility
    const fromRust = normalizeMaterialSpec({
      name: 'borosilicate_glass',
      density_kg_m3: 2500,
      youngs_modulus_gpa: 64,
      poissons_ratio: 0.2,
      yield_strength_mpa: 120,
      ultimate_strength_pa: 120e6,
      hardening_modulus_pa: 0,
      fracture_strain: 0.002,
      fracture_energy_j_m2: 8.5,
      friction_coeff: 0.18,
      failure_mode: 'brittle',
    });
    assert.equal(validateMaterialSpec(fromRust).valid, true);
    assert.equal(fromRust.failureMode, 'brittle');
    assert.equal(fromRust.fractureEnergyJM2, 8.5);
  });

  it('verifies P5-T20: alias and Rust canonical material IDs resolve to matching library cards for UI highlighting', () => {
    const cases: Array<[string, string]> = [
      ['al_6061', 'alu_6061'],
      ['aluminum_6061_t6', 'alu_6061'],
      ['steel_carbon_aisi1045', 'steel_1045'],
      ['AISI 1045', 'steel_1045'],
      ['steel_1020', 'steel_structural_aisi1020'],
      ['steel_high_strength_4140', 'steel_4140'],
      ['pyrex', 'borosilicate_glass'],
      ['glass', 'borosilicate_glass'],
      ['titanium_grade5_ti6al4v', 'ti_6al_4v'],
      ['brass', 'brass_c360'],
    ];

    for (const [partMaterialId, expectedCardId] of cases) {
      const activeMaterial = getMaterialById(partMaterialId, MATERIALS_LIBRARY);
      const matchedCards = MATERIALS_LIBRARY.filter(
        (mat) => activeMaterial.id === mat.id
      );
      assert.equal(
        matchedCards.length,
        1,
        `Expected exactly 1 highlighted card for '${partMaterialId}', got ${matchedCards.length}`
      );
      assert.equal(matchedCards[0].id, expectedCardId);
    }

    // Verify CadLeftSidebar uses activeMaterial.id === mat.id rather than raw selectedPart.materialId === mat.id
    const sidebarPath = path.resolve(
      process.cwd(),
      'src/components/CadLeftSidebar.tsx'
    );
    const sidebarSrc = fs.readFileSync(sidebarPath, 'utf8');
    assert.ok(
      sidebarSrc.includes('activeMaterial?.id === mat.id') ||
        sidebarSrc.includes('activeMaterial.id === mat.id'),
      'CadLeftSidebar must compare activeMaterial.id === mat.id to highlight alias-assigned materials'
    );
    assert.equal(
      sidebarSrc.includes('selectedPart.materialId === mat.id'),
      false,
      'CadLeftSidebar must not use direct selectedPart.materialId === mat.id comparison'
    );
  });

  it('verifies P5-T28: run_fea_analysis, get_fea_heatmap_image, and CadViewport3D FEM surface meshes resolve material aliases via getMaterialById', () => {
    const baseState: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    // Create two identical cantilevers: one with canonical 'steel_1045' (E=205 GPa, sy=350 MPa)
    // and one with alias 'pyrex' -> 'borosilicate_glass' (E=64 GPa, sy=120 MPa)
    const { nextState: s1 } = executeMcpToolOnEngineState(baseState, 'create_part', {
      part_id: 'beam_steel',
      name: 'Beam Steel',
      material: 'steel_1045',
    });
    const { nextState: s2 } = executeMcpToolOnEngineState(s1, 'create_part', {
      part_id: 'beam_pyrex',
      name: 'Beam Pyrex',
      material: 'pyrex',
    });
    const { nextState: s3 } = executeMcpToolOnEngineState(s2, 'create_part', {
      part_id: 'beam_al_rust',
      name: 'Beam Al Rust ID',
      material: 'aluminum_6061_t6',
    });

    const feaSteel = executeMcpToolOnEngineState(s3, 'run_fea_analysis', {
      part_id: 'beam_steel',
      fixed_region: 'min_x',
      loads: [{ region: 'max_x', force_vector: [0, -1200, 0] }],
    }).response;

    const feaPyrex = executeMcpToolOnEngineState(s3, 'run_fea_analysis', {
      part_id: 'beam_pyrex',
      fixed_region: 'min_x',
      loads: [{ region: 'max_x', force_vector: [0, -1200, 0] }],
    }).response;

    const feaAl = executeMcpToolOnEngineState(s3, 'run_fea_analysis', {
      part_id: 'beam_al_rust',
      fixed_region: 'min_x',
      loads: [{ region: 'max_x', force_vector: [0, -1200, 0] }],
    }).response;

    // Pyrex has E = 64 GPa vs Steel 1045 E = 205 GPa (~3.2x larger displacement) and sy = 120 MPa vs 350 MPa
    assert.equal(feaSteel.yieldStrengthMPa, 350);
    assert.equal(feaPyrex.yieldStrengthMPa, 120);
    assert.equal(feaAl.yieldStrengthMPa, 276);
    assert.ok(
      feaPyrex.maxDisplacementMm > feaSteel.maxDisplacementMm * 2.5,
      `Expected pyrex (E=64 GPa) displacement (${feaPyrex.maxDisplacementMm} mm) to be >2.5x steel_1045 (${feaSteel.maxDisplacementMm} mm)`
    );
    assert.ok(
      feaAl.maxDisplacementMm > feaSteel.maxDisplacementMm * 2.5,
      `Expected aluminum_6061_t6 (E=68.9 GPa) displacement (${feaAl.maxDisplacementMm} mm) to be >2.5x steel_1045 (${feaSteel.maxDisplacementMm} mm)`
    );
    assert.ok(
      feaPyrex.safetyFactor < feaSteel.safetyFactor * 0.5,
      `Expected pyrex (sy=120 MPa) safety factor (${feaPyrex.safetyFactor}) to be <0.5x steel_1045 (${feaSteel.safetyFactor})`
    );

    // Verify get_fea_heatmap_image on a part without cached feaStressMPa also resolves alias 'pyrex'
    const heatmapPyrex = executeMcpToolOnEngineState(
      s3,
      'get_fea_heatmap_image',
      { part_id: 'beam_pyrex' }
    ).response;
    const heatmapSteel = executeMcpToolOnEngineState(
      s3,
      'get_fea_heatmap_image',
      { part_id: 'beam_steel' }
    ).response;
    assert.ok(
      heatmapPyrex.safety_factor < heatmapSteel.safety_factor * 0.5,
      `Expected get_fea_heatmap_image for 'pyrex' (${heatmapPyrex.safety_factor}) to use sy=120 MPa instead of falling back to steel_1045 (${heatmapSteel.safety_factor})`
    );

    // Verify custom material defined via define_material still resolves cleanly
    const { nextState: sCustom } = executeMcpToolOnEngineState(
      s3,
      'define_material',
      {
        material_id: 'custom_soft_rubber',
        density_kg_m3: 1100,
        youngs_modulus_gpa: 1.0,
        poissons_ratio: 0.45,
        yield_strength_mpa: 25,
        part_id: 'beam_steel',
      }
    );
    const feaCustom = executeMcpToolOnEngineState(sCustom, 'run_fea_analysis', {
      part_id: 'beam_steel',
    }).response;
    assert.equal(feaCustom.yieldStrengthMPa, 25);
    assert.ok(
      feaCustom.maxDisplacementMm > feaSteel.maxDisplacementMm * 50,
      `Expected custom 1 GPa material displacement (${feaCustom.maxDisplacementMm} mm) to scale with custom E`
    );

    // Verify CadViewport3D resolves FEM surface material properties via getMaterialById
    const viewportPath = path.resolve(
      process.cwd(),
      'src/components/CadViewport3D.tsx'
    );
    const viewportSrc = fs.readFileSync(viewportPath, 'utf8');
    assert.ok(
      viewportSrc.includes('getMaterialById(partObj.materialId, materials)'),
      'CadViewport3D must resolve FEM surface material aliases via getMaterialById(partObj.materialId, materials)'
    );
    assert.equal(
      viewportSrc.includes('materials.find((m) => m.id === partObj?.materialId)'),
      false,
      'CadViewport3D must not use raw materials.find((m) => m.id === partObj?.materialId)'
    );
    assert.equal(getMaterialById('pyrex', MATERIALS_LIBRARY).colorHex, '#dbeafe');
    assert.equal(
      getMaterialById('aluminum_6061_t6', MATERIALS_LIBRARY).colorHex,
      '#c4cdd6'
    );
  });

  it('verifies P5-T27: docs/MCP_API_REFERENCE.md define_material section matches Rust inputSchema and response properties', () => {
    const modRs = fs.readFileSync(
      path.resolve(process.cwd(), 'crates/mcp-cad-server/src/tools/mod.rs'),
      'utf8'
    );
    const cadRs = fs.readFileSync(
      path.resolve(process.cwd(), 'crates/mcp-cad-server/src/tools/cad.rs'),
      'utf8'
    );
    const apiRef = fs.readFileSync(
      path.resolve(process.cwd(), 'docs/MCP_API_REFERENCE.md'),
      'utf8'
    );

    // 1. Extract define_material input_schema properties & required fields from Rust mod.rs
    const schemaMatch = modRs.match(
      /name:\s*"define_material"\.to_string\(\)[\s\S]*?"properties":\s*\{([\s\S]*?)\n\s*\},\s*\n\s*"required":\s*\[([\s\S]*?)\]/
    );
    assert.ok(schemaMatch, 'Could not locate define_material input_schema in mod.rs');

    const rustInputParams = Array.from(
      schemaMatch[1].matchAll(/^\s*"([a-z0-9_]+)":\s*\{/gm),
      (m) => m[1]
    );
    const rustRequiredParams = new Set(
      Array.from(schemaMatch[2].matchAll(/"([a-z0-9_]+)"/g), (m) => m[1])
    );
    assert.ok(
      rustInputParams.length >= 13,
      `Expected >= 13 define_material schema params in mod.rs, found ${rustInputParams.length}`
    );
    assert.ok(rustRequiredParams.size === 5, 'Expected 5 required params in define_material');

    // 2. Extract define_material response "properties" keys from Rust cad.rs
    const cadRespMatch = cadRs.match(
      /"status":\s*"material_registered"[\s\S]*?"properties":\s*\{([\s\S]*?)\n\s*\},/
    );
    assert.ok(cadRespMatch, 'Could not locate define_material response JSON in cad.rs');
    const rustResponseProps = Array.from(
      cadRespMatch[1].matchAll(/^\s*"([a-z0-9_]+)":/gm),
      (m) => m[1]
    );
    assert.ok(
      rustResponseProps.length >= 14,
      `Expected >= 14 response properties in cad.rs define_material, found ${rustResponseProps.length}`
    );

    // 3. Extract section ### 25. `define_material` from docs/MCP_API_REFERENCE.md
    const secStart = apiRef.indexOf('### 25. `define_material`');
    const secEnd = apiRef.indexOf('### 26. `batch_cad_operations`');
    assert.ok(secStart !== -1 && secEnd > secStart, 'Could not isolate section 25 in MCP_API_REFERENCE.md');
    const section = apiRef.slice(secStart, secEnd);

    // Verify every Rust inputSchema parameter is documented in the parameters table with matching required flag
    const tableRowRe = /^\|\s*`([a-z0-9_]+)`\s*\|\s*(?:\\.|[^|])+\|\s*([^|]+)\|/gm;
    const docTable = new Map<string, string>();
    for (const m of section.matchAll(tableRowRe)) {
      docTable.set(m[1], m[2].trim());
    }

    for (const param of rustInputParams) {
      assert.ok(
        docTable.has(param),
        `docs/MCP_API_REFERENCE.md define_material table is missing Rust schema parameter '${param}'`
      );
      const reqCell = docTable.get(param)!;
      if (rustRequiredParams.has(param)) {
        assert.ok(
          reqCell.includes('Да'),
          `Parameter '${param}' must be marked required ('Да') in MCP_API_REFERENCE.md, got '${reqCell}'`
        );
      } else {
        assert.ok(
          reqCell.includes('Нет'),
          `Parameter '${param}' must be marked optional ('Нет') in MCP_API_REFERENCE.md, got '${reqCell}'`
        );
      }
    }

    // 4. Parse both JSON code blocks (request and response) in section 25
    const jsonBlocks = Array.from(section.matchAll(/```json\n([\s\S]*?)\n```/g), (m) => m[1]);
    assert.equal(jsonBlocks.length, 2, 'Expected exactly 2 JSON example blocks (request and response)');

    const reqObj = JSON.parse(jsonBlocks[0]);
    const respEnvelope = JSON.parse(jsonBlocks[1]);
    assert.equal(reqObj.params.name, 'define_material');

    const reqArgs = reqObj.params.arguments as Record<string, unknown>;
    const respPayload = JSON.parse(respEnvelope.result.content[0].text) as {
      status: string;
      material_id: string;
      properties: Record<string, unknown>;
      units: Record<string, unknown>;
    };

    assert.equal(respPayload.status, 'material_registered');
    assert.equal(respPayload.material_id, reqArgs.id);
    assert.ok(respPayload.units && respPayload.units.length === 'mm', 'Response example must include units metadata');

    for (const propKey of rustResponseProps) {
      assert.ok(
        Object.prototype.hasOwnProperty.call(respPayload.properties, propKey),
        `Response example in MCP_API_REFERENCE.md is missing properties.${propKey}`
      );
    }

    // 5. Verify example arguments satisfy physical material invariants and match response conversions
    const normalizedExample = normalizeMaterialSpec({
      id: String(reqArgs.id),
      name: String(reqArgs.name),
      youngsModulus: Number(reqArgs.young_modulus_gpa) * 1e9,
      poissonsRatio: Number(reqArgs.poisson_ratio),
      yieldStrength: Number(reqArgs.yield_strength_mpa) * 1e6,
      density: Number(reqArgs.density_kg_m3),
      ultimate_strength_mpa: reqArgs.ultimate_strength_mpa,
      hardening_modulus_pa: reqArgs.hardening_modulus_pa,
      fracture_strain: reqArgs.fracture_strain,
      fracture_energy_j_m2: reqArgs.fracture_energy_j_m2,
      friction_coeff: reqArgs.friction_coeff,
      failure_mode: reqArgs.failure_mode,
    });
    const validation = validateMaterialSpec(normalizedExample);
    assert.equal(
      validation.valid,
      true,
      `Example material in MCP_API_REFERENCE.md violates invariants: ${validation.errors.join('; ')}`
    );
    assert.equal(respPayload.properties.ultimate_strength_mpa, reqArgs.ultimate_strength_mpa);
    assert.equal(
      respPayload.properties.ultimate_strength_pa,
      Number(reqArgs.ultimate_strength_mpa) * 1e6
    );
    assert.equal(respPayload.properties.hardening_modulus_pa, reqArgs.hardening_modulus_pa);
    assert.equal(respPayload.properties.fracture_strain, reqArgs.fracture_strain);
    assert.equal(respPayload.properties.fracture_energy_j_m2, reqArgs.fracture_energy_j_m2);
    assert.equal(respPayload.properties.friction_coeff, reqArgs.friction_coeff);
    assert.equal(respPayload.properties.failure_mode, reqArgs.failure_mode);
  });

  it('verifies P5-T29: import_stl computes mesh mass and inertia from params.material density for presets, aliases, and custom materials', () => {
    const baseState: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const expectedVolume = 0.05 * 0.05 * 0.05; // 1.25e-4 m^3

    const assertPartMass = (part: any, expectedDensity: number, label: string) => {
      assert.ok(
        Math.abs(part.mesh.mass - part.mesh.volume * expectedDensity) < 1e-12,
        `Expected ${label} exact mass = volume * ${expectedDensity} (${part.mesh.volume * expectedDensity}), got ${part.mesh.mass}`
      );
      assert.ok(
        Math.abs(part.mesh.mass - expectedVolume * expectedDensity) < 1e-6,
        `Expected ${label} nominal mass ${expectedVolume * expectedDensity}, got ${part.mesh.mass}`
      );
    };

    // 1. Default material ('steel_1045', density = 7850 kg/m^3)
    const impDefault = executeMcpToolOnEngineState(baseState, 'import_stl', {
      file_path: 'output/default_block.stl',
      part_id: 'stl_default',
    });
    const partDefault = impDefault.nextState.parts.find((p) => p.id === 'stl_default');
    assert.ok(partDefault, 'stl_default should be imported');
    assert.equal(partDefault.materialId, 'steel_1045');
    assertPartMass(partDefault, 7850, 'steel_1045');

    // 2. Direct preset ID 'alu_6061' (density = 2700 kg/m^3)
    const impAlu = executeMcpToolOnEngineState(impDefault.nextState, 'import_stl', {
      file_path: 'output/alu_bracket.stl',
      part_id: 'stl_alu',
      material: 'alu_6061',
    });
    const partAlu = impAlu.nextState.parts.find((p) => p.id === 'stl_alu');
    assert.ok(partAlu, 'stl_alu should be imported');
    assert.equal(partAlu.materialId, 'alu_6061');
    assertPartMass(partAlu, 2700, 'alu_6061');

    // 3. Alias 'pyrex' -> 'borosilicate_glass' (density = 2500 kg/m^3)
    const impPyrex = executeMcpToolOnEngineState(impAlu.nextState, 'import_stl', {
      file_path: 'output/glass_bulb.stl',
      part_id: 'stl_pyrex',
      material: 'pyrex',
    });
    const partPyrex = impPyrex.nextState.parts.find((p) => p.id === 'stl_pyrex');
    assert.ok(partPyrex, 'stl_pyrex should be imported');
    assert.equal(partPyrex.materialId, 'pyrex');
    assertPartMass(partPyrex, 2500, 'pyrex');

    // 4. Canonical Rust ID alias 'aluminum_6061_t6' -> 'alu_6061' (density = 2700 kg/m^3)
    const impRustAl = executeMcpToolOnEngineState(impPyrex.nextState, 'import_stl', {
      file_path: 'output/rust_al_housing.stl',
      part_id: 'stl_rust_al',
      material: 'aluminum_6061_t6',
    });
    const partRustAl = impRustAl.nextState.parts.find((p) => p.id === 'stl_rust_al');
    assert.ok(partRustAl, 'stl_rust_al should be imported');
    assert.equal(partRustAl.materialId, 'aluminum_6061_t6');
    assertPartMass(partRustAl, 2700, 'aluminum_6061_t6');

    // 5. Custom material registered via define_material (density = 1500 kg/m^3)
    const { nextState: stateWithCustom } = executeMcpToolOnEngineState(
      impRustAl.nextState,
      'define_material',
      {
        material_id: 'custom_cfrp_1500',
        density_kg_m3: 1500,
        youngs_modulus_gpa: 70,
        poissons_ratio: 0.3,
        yield_strength_mpa: 450,
      }
    );
    const impCustom = executeMcpToolOnEngineState(stateWithCustom, 'import_stl', {
      file_path: 'output/cfrp_rib.stl',
      part_id: 'stl_cfrp',
      material: 'custom_cfrp_1500',
    });
    const partCustom = impCustom.nextState.parts.find((p) => p.id === 'stl_cfrp');
    assert.ok(partCustom, 'stl_cfrp should be imported');
    assert.equal(partCustom.materialId, 'custom_cfrp_1500');
    assertPartMass(partCustom, 1500, 'custom_cfrp_1500');

    // Verify inertia tensor scales linearly with density as well
    assert.ok(
      Math.abs(partPyrex.mesh.inertiaTensor[0] / partDefault.mesh.inertiaTensor[0] - 2500 / 7850) < 1e-9,
      'Expected inertiaTensor to scale proportionally with assigned material density'
    );

    // Verify list_parts reports consistent mass_kg for all imported parts
    const listed = executeMcpToolOnEngineState(impCustom.nextState, 'list_parts', {}).response;
    const byId = new Map<string, any>(listed.parts.map((p: any) => [p.part_id, p]));
    assert.equal(byId.get('stl_default').mass_kg, Number((expectedVolume * 7850).toFixed(4)));
    assert.equal(byId.get('stl_alu').mass_kg, Number((expectedVolume * 2700).toFixed(4)));
    assert.equal(byId.get('stl_pyrex').mass_kg, Number((expectedVolume * 2500).toFixed(4)));
    assert.equal(byId.get('stl_rust_al').mass_kg, Number((expectedVolume * 2700).toFixed(4)));
    assert.equal(byId.get('stl_cfrp').mass_kg, Number((expectedVolume * 1500).toFixed(4)));
  });

  it('verifies P5-T30: create_joint prismatic limits honor default mm and explicit m length_unit consistently with Rust contract', () => {
    const makeSliderState = (): EngineStateSnapshot => {
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
        part_id: 'slider_p5_t30',
        primitive_type: 'box',
        params: { width: 0.04, height: 0.04, depth: 0.04 },
      }).nextState;
      return state;
    };

    // 1. Default length_unit ('mm'): limits [-500, 500] mm -> internal [-0.5, 0.5] m
    let stateMm = makeSliderState();
    const resMm = executeMcpToolOnEngineState(stateMm, 'create_joint', {
      joint_id: 'j_prism_mm',
      joint_type: 'prismatic',
      part_a: 'ground',
      part_b: 'slider_p5_t30',
      axis: [1, 0, 0],
      limits: [-500, 500],
    });
    assert.equal(resMm.response.length_unit, 'mm');
    stateMm = resMm.nextState;
    assert.deepEqual(stateMm.joints[0].limits, [-0.5, 0.5]);

    stateMm = executeMcpToolOnEngineState(stateMm, 'set_joint_motor', {
      joint_id: 'j_prism_mm',
      target_velocity: 1.5,
      max_force: 300.0,
    }).nextState;
    stateMm = executeMcpToolOnEngineState(stateMm, 'run_physics_simulation', {
      duration_seconds: 0.4,
      dt: 0.01,
    }).nextState;
    const sliderMm = stateMm.parts.find((p) => p.id === 'slider_p5_t30')!;
    assert.ok(
      Math.abs(sliderMm.position[0] - 0.5) < 1e-6,
      `Expected slider to clamp at +0.5 m with limits [-500, 500] mm, got ${sliderMm.position[0]}`
    );

    // 2. Explicit length_unit ('m'): limits [-0.5, 0.5] m -> internal [-0.5, 0.5] m
    let stateM = makeSliderState();
    const resM = executeMcpToolOnEngineState(stateM, 'create_joint', {
      joint_id: 'j_prism_m',
      joint_type: 'prismatic',
      part_a: 'ground',
      part_b: 'slider_p5_t30',
      axis: [1, 0, 0],
      limits: [-0.5, 0.5],
      length_unit: 'm',
    });
    assert.equal(resM.response.length_unit, 'm');
    stateM = resM.nextState;
    assert.deepEqual(stateM.joints[0].limits, [-0.5, 0.5]);

    stateM = executeMcpToolOnEngineState(stateM, 'set_joint_motor', {
      joint_id: 'j_prism_m',
      target_velocity: 1.5,
      max_force: 300.0,
    }).nextState;
    stateM = executeMcpToolOnEngineState(stateM, 'run_physics_simulation', {
      duration_seconds: 0.4,
      dt: 0.01,
    }).nextState;
    const sliderM = stateM.parts.find((p) => p.id === 'slider_p5_t30')!;
    assert.ok(
      Math.abs(sliderM.position[0] - 0.5) < 1e-6,
      `Expected slider to clamp at +0.5 m with length_unit='m' limits [-0.5, 0.5], got ${sliderM.position[0]}`
    );

    // 3. Default length_unit ('mm') with small sub-millimeter limits [-0.5, 0.5] -> [-0.0005, 0.0005] m
    const resSubMm = executeMcpToolOnEngineState(makeSliderState(), 'create_joint', {
      joint_id: 'j_prism_submm',
      joint_type: 'prismatic',
      part_a: 'ground',
      part_b: 'slider_p5_t30',
      axis: [1, 0, 0],
      limits: [-0.5, 0.5],
    });
    assert.ok(
      Math.abs(resSubMm.nextState.joints[0].limits![0] - -0.0005) < 1e-12 &&
        Math.abs(resSubMm.nextState.joints[0].limits![1] - 0.0005) < 1e-12,
      `Expected [-0.5, 0.5] in default mm to convert to [-0.0005, 0.0005] m, got ${JSON.stringify(resSubMm.nextState.joints[0].limits)}`
    );

    // 4. Unsupported length_unit must be rejected (matches Rust state.resolve_length_unit)
    assert.throws(
      () =>
        executeMcpToolOnEngineState(makeSliderState(), 'create_joint', {
          joint_id: 'j_bad_unit',
          joint_type: 'prismatic',
          part_a: 'ground',
          part_b: 'slider_p5_t30',
          axis: [1, 0, 0],
          limits: [-10, 10],
          length_unit: 'inch',
        }),
      /length_unit/
    );

    // 5. Revolute limits default to 'deg' and honor limits_unit: 'rad'
    const resRevDeg = executeMcpToolOnEngineState(makeSliderState(), 'create_joint', {
      joint_id: 'j_rev_deg',
      joint_type: 'revolute',
      part_a: 'ground',
      part_b: 'slider_p5_t30',
      axis: [0, 0, 1],
      limits: [-90, 90],
    });
    assert.ok(Math.abs(resRevDeg.nextState.joints[0].limits![0] - -Math.PI / 2) < 1e-12);
    assert.ok(Math.abs(resRevDeg.nextState.joints[0].limits![1] - Math.PI / 2) < 1e-12);

    const resRevRad = executeMcpToolOnEngineState(makeSliderState(), 'create_joint', {
      joint_id: 'j_rev_rad',
      joint_type: 'revolute',
      part_a: 'ground',
      part_b: 'slider_p5_t30',
      axis: [0, 0, 1],
      limits: [-Math.PI / 2, Math.PI / 2],
      limits_unit: 'rad',
    });
    assert.ok(Math.abs(resRevRad.nextState.joints[0].limits![0] - -Math.PI / 2) < 1e-12);
    assert.ok(Math.abs(resRevRad.nextState.joints[0].limits![1] - Math.PI / 2) < 1e-12);
  });

  it('verifies P5-T31: strict failure_mode validation, Rust keys in normalizeMaterialSpec, and zero-value rejection in define_material', () => {
    // 1. normalizeMaterialSpec reads canonical Rust MCP keys (young_modulus_gpa, young_modulus_pa, poisson_ratio)
    const fromRustGpa = normalizeMaterialSpec({
      id: 'rust_inconel_718',
      name: 'Inconel 718',
      young_modulus_gpa: 200,
      poisson_ratio: 0.28,
      yield_strength_mpa: 1035,
      density_kg_m3: 8190,
      failure_mode: 'BRITTLE',
    });
    assert.equal(validateMaterialSpec(fromRustGpa).valid, true);
    assert.equal(fromRustGpa.youngsModulus, 200e9);
    assert.equal(fromRustGpa.poissonsRatio, 0.28);
    assert.equal(fromRustGpa.yieldStrength, 1035e6);
    assert.equal(fromRustGpa.density, 8190);
    assert.equal(fromRustGpa.failureMode, 'brittle');
    assert.equal(fromRustGpa.hardeningModulusPa, 0);
    assert.equal(fromRustGpa.fractureStrain, 0.002);

    const fromRustPa = normalizeMaterialSpec({
      id: 'rust_pa_alloy',
      young_modulus_pa: 72.5e9,
      poisson_ratio: 0.33,
      yield_strength_pa: 310e6,
      density_kg_m3: 2780,
      failure_mode: ' Ductile ',
    });
    assert.equal(validateMaterialSpec(fromRustPa).valid, true);
    assert.equal(fromRustPa.youngsModulus, 72.5e9);
    assert.equal(fromRustPa.poissonsRatio, 0.33);
    assert.equal(fromRustPa.failureMode, 'ductile');

    // 2. Unknown / empty / non-string failure_mode is preserved and rejected by validateMaterialSpec & assertValidMaterialSpec
    for (const badMode of ['ceramic', '', '   ', 42]) {
      const invalidSpec = normalizeMaterialSpec({
        id: 'invalid_mode_mat',
        young_modulus_gpa: 100,
        poisson_ratio: 0.3,
        yield_strength_mpa: 250,
        density_kg_m3: 4500,
        failure_mode: badMode,
      });
      const validation = validateMaterialSpec(invalidSpec);
      assert.equal(
        validation.valid,
        false,
        `Expected failure_mode=${JSON.stringify(badMode)} to fail validateMaterialSpec`
      );
      assert.ok(
        validation.errors.some((e) => e.includes('failureMode')),
        `Expected failureMode error for ${JSON.stringify(badMode)}, got: ${validation.errors.join('; ')}`
      );
      assert.throws(() => assertValidMaterialSpec(invalidSpec), /failureMode/);
    }

    const makeBaseState = (): EngineStateSnapshot => ({
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    });

    // 3. TS define_material rejects unknown / empty failure_mode
    assert.throws(
      () =>
        executeMcpToolOnEngineState(makeBaseState(), 'define_material', {
          id: 'bad_ceramic',
          young_modulus_gpa: 300,
          poisson_ratio: 0.22,
          yield_strength_mpa: 400,
          density_kg_m3: 3900,
          failure_mode: 'ceramic',
        }),
      /failureMode/
    );
    assert.throws(
      () =>
        executeMcpToolOnEngineState(makeBaseState(), 'define_material', {
          id: 'empty_mode',
          young_modulus_gpa: 300,
          poisson_ratio: 0.22,
          yield_strength_mpa: 400,
          density_kg_m3: 3900,
          failure_mode: '   ',
        }),
      /failureMode/
    );

    // 4. TS define_material rejects explicit 0 for youngs_modulus_pa, young_modulus_pa, and yield_strength_pa (no truthiness fallback)
    assert.throws(
      () =>
        executeMcpToolOnEngineState(makeBaseState(), 'define_material', {
          id: 'zero_e_pa',
          youngs_modulus_pa: 0,
          poisson_ratio: 0.3,
          yield_strength_mpa: 250,
          density_kg_m3: 7850,
        }),
      /positive density, youngs_modulus, and yield_strength/
    );
    assert.throws(
      () =>
        executeMcpToolOnEngineState(makeBaseState(), 'define_material', {
          id: 'zero_rust_e_pa',
          young_modulus_pa: 0,
          young_modulus_gpa: 205,
          poisson_ratio: 0.3,
          yield_strength_mpa: 250,
          density_kg_m3: 7850,
        }),
      /positive density, youngs_modulus, and yield_strength/
    );
    assert.throws(
      () =>
        executeMcpToolOnEngineState(makeBaseState(), 'define_material', {
          id: 'zero_yield_pa',
          young_modulus_gpa: 205,
          poisson_ratio: 0.3,
          yield_strength_pa: 0,
          yield_strength_mpa: 350,
          density_kg_m3: 7850,
        }),
      /positive density, youngs_modulus, and yield_strength/
    );
    assert.throws(
      () =>
        executeMcpToolOnEngineState(makeBaseState(), 'define_material', {
          id: 'nan_e_pa',
          youngs_modulus_pa: Number.NaN,
          poisson_ratio: 0.3,
          yield_strength_mpa: 350,
          density_kg_m3: 7850,
        }),
      /positive density, youngs_modulus, and yield_strength/
    );

    // 5. TS define_material succeeds with valid canonical Rust keys and brittle failure_mode
    const okRes = executeMcpToolOnEngineState(makeBaseState(), 'define_material', {
      id: 'silicon_carbide_p5_t31',
      name: 'Silicon Carbide',
      young_modulus_gpa: 410,
      poisson_ratio: 0.14,
      yield_strength_mpa: 550,
      density_kg_m3: 3100,
      failure_mode: 'brittle',
    });
    const registered = okRes.nextState.materials.find((m) => m.id === 'silicon_carbide_p5_t31');
    assert.ok(registered, 'Expected custom brittle material to be registered');
    assert.equal(registered.youngsModulus, 410e9);
    assert.equal(registered.poissonsRatio, 0.14);
    assert.equal(registered.yieldStrength, 550e6);
    assert.equal(registered.density, 3100);
    assert.equal(registered.failureMode, 'brittle');
    assert.equal(registered.hardeningModulusPa, 0);
    assert.equal(okRes.response.failure_mode, 'brittle');
    assert.equal(okRes.response.youngs_modulus_gpa, 410);
  });

  it('verifies P5-T32: docs/MCP_API_REFERENCE.md TOC contains working anchor links for all 43 documented MCP tools', () => {
    const modRs = fs.readFileSync(
      path.resolve(process.cwd(), 'crates/mcp-cad-server/src/tools/mod.rs'),
      'utf8'
    );
    const apiRef = fs.readFileSync(
      path.resolve(process.cwd(), 'docs/MCP_API_REFERENCE.md'),
      'utf8'
    );

    // 1. Extract all registered Rust MCP tool names from list_tools() in mod.rs
    const listToolsMatch = modRs.match(
      /pub fn list_tools\(\)\s*->\s*Vec<ToolDefinition>\s*\{([\s\S]*?)\n\}/
    );
    assert.ok(listToolsMatch, 'Could not locate list_tools() in crates/mcp-cad-server/src/tools/mod.rs');
    const rustToolNames = Array.from(
      listToolsMatch[1].matchAll(/ToolDefinition\s*\{\s*\n\s*name:\s*"([a-z0-9_]+)"\.to_string\(\)/g),
      (m) => m[1]
    );
    assert.equal(
      rustToolNames.length,
      43,
      `Expected 43 Rust MCP tools in list_tools(), found ${rustToolNames.length}`
    );

    // 2. Isolate the "## Оглавление" block in docs/MCP_API_REFERENCE.md
    const tocStart = apiRef.indexOf('## Оглавление');
    const tocEnd = apiRef.indexOf('## Общие положения');
    assert.ok(tocStart !== -1 && tocEnd > tocStart, 'Could not isolate ## Оглавление in docs/MCP_API_REFERENCE.md');
    const tocSection = apiRef.slice(tocStart, tocEnd);

    // Parse all numbered tool entries in the TOC: - [N. `tool_name`](#anchor)
    const tocEntries = Array.from(
      tocSection.matchAll(/^\s*-\s*\[(\d+)\.\s*`([a-z0-9_]+)`\]\(#([^)]+)\)/gm),
      (m) => ({
        index: Number(m[1]),
        name: m[2],
        anchor: m[3],
      })
    );

    assert.equal(
      tocEntries.length,
      43,
      `Expected 43 tool links in docs/MCP_API_REFERENCE.md TOC, found ${tocEntries.length}`
    );

    // Verify sequential numbering 1..43 and explicit presence of 42 and 43
    for (let i = 0; i < tocEntries.length; i++) {
      assert.equal(
        tocEntries[i].index,
        i + 1,
        `Expected TOC tool entry #${i + 1} to have index ${i + 1}, got ${tocEntries[i].index} (${tocEntries[i].name})`
      );
    }
    assert.deepEqual(tocEntries[41], {
      index: 42,
      name: 'bake_coupled_simulation',
      anchor: '42-bake_coupled_simulation',
    });
    assert.deepEqual(tocEntries[42], {
      index: 43,
      name: 'query_simulation_frame',
      anchor: '43-query_simulation_frame',
    });

    // 3. Verify 1:1 parity between TOC tool names and Rust ToolDefinition catalog
    const tocToolSet = new Set(tocEntries.map((e) => e.name));
    assert.equal(tocToolSet.size, 43, 'Duplicate tool names found in TOC');
    for (const rustName of rustToolNames) {
      assert.ok(
        tocToolSet.has(rustName),
        `Rust MCP tool '${rustName}' is missing from docs/MCP_API_REFERENCE.md TOC`
      );
    }

    // 4. Compute GitHub Flavored Markdown heading slugs for all ### headings in docs/MCP_API_REFERENCE.md
    //    and verify every TOC tool link resolves to an existing document heading
    const toGfmSlug = (headingText: string): string =>
      headingText
        .trim()
        .toLowerCase()
        .replace(/`/g, '')
        .replace(/[^\p{L}\p{N}_ -]/gu, '')
        .replace(/\s+/g, '-');

    const bodySection = apiRef.slice(tocEnd);
    const headingSlugs = new Set<string>();
    const toolHeadings = new Map<number, string>();
    for (const m of bodySection.matchAll(/^###\s+(.+)$/gm)) {
      const raw = m[1].trim();
      headingSlugs.add(toGfmSlug(raw));
      const toolMatch = raw.match(/^(\d+)\.\s*`([a-z0-9_]+)`$/);
      if (toolMatch) {
        toolHeadings.set(Number(toolMatch[1]), toolMatch[2]);
      }
    }

    assert.equal(
      toolHeadings.size,
      43,
      `Expected 43 tool section headings in docs/MCP_API_REFERENCE.md, found ${toolHeadings.size}`
    );

    for (const entry of tocEntries) {
      assert.ok(
        headingSlugs.has(entry.anchor),
        `Broken TOC anchor '#${entry.anchor}' for tool ${entry.index}. ${entry.name}`
      );
      assert.equal(
        toolHeadings.get(entry.index),
        entry.name,
        `TOC entry #${entry.index} ('${entry.name}') does not match section heading ('${toolHeadings.get(entry.index)}')`
      );
    }
  });

  it('verifies P5-T33: handleLoadNativeStlUrl resolves material density via getMaterialById and hydrates MCP import_stl without wiping scene or part metadata', () => {
    // 1. Standalone rim STL resolves alu_6061 (2700 kg/m³) rather than hardcoded steel (7850 kg/m³)
    const rimResolved = resolveNativeStlMaterialSpec('/output/car_rim.stl', MATERIALS_LIBRARY);
    assert.equal(rimResolved.partId, 'car_rim');
    assert.equal(rimResolved.materialId, 'alu_6061');
    assert.equal(rimResolved.density, 2700);

    const sampleBox = createBoxMesh(0.1, 0.08, 0.04, 1000); // V = 0.00032 m³
    const binaryStl = exportMeshToStlBinary(sampleBox, 'sample_rim');
    const rimMesh = parseStlBuffer(binaryStl, rimResolved.density, 1.0);
    const steelMesh = parseStlBuffer(binaryStl, 7850, 1.0);

    assert.ok(
      Math.abs(rimMesh.mass - sampleBox.volume * 2700) < 1e-6,
      `Expected rim mass ~${(sampleBox.volume * 2700).toFixed(6)} kg, got ${rimMesh.mass}`
    );
    assert.ok(
      Math.abs(rimMesh.mass / steelMesh.mass - 2700 / 7850) < 1e-6,
      'Expected rim mass ratio vs steel to match 2700 / 7850'
    );
    for (let axis = 0; axis < 3; axis++) {
      assert.ok(
        Math.abs(rimMesh.inertiaTensor[axis] / steelMesh.inertiaTensor[axis] - 2700 / 7850) < 1e-6,
        `Expected rim inertia[${axis}] ratio vs steel to match 2700 / 7850`
      );
    }

    const standaloneRimPart = buildStandaloneNativeStlPart(
      '/output/car_rim.stl',
      'car_rim.stl (31 629 Δ)',
      rimMesh,
      rimResolved.materialId,
      rimResolved.partId
    );
    assert.equal(standaloneRimPart.id, 'car_rim');
    assert.equal(standaloneRimPart.materialId, 'alu_6061');
    assert.equal(standaloneRimPart.mesh.mass, rimMesh.mass);

    // 2. Explicit canonical Rust IDs, aliases, and custom materials resolve exact density
    const pyrexResolved = resolveNativeStlMaterialSpec(
      '/output/lightbulb_e27.stl',
      MATERIALS_LIBRARY,
      'pyrex'
    );
    assert.equal(pyrexResolved.density, 2500);
    assert.equal(pyrexResolved.material.id, 'borosilicate_glass');

    const rustAlResolved = resolveNativeStlMaterialSpec(
      '/output/piston.stl',
      MATERIALS_LIBRARY,
      'aluminum_6061_t6'
    );
    assert.equal(rustAlResolved.density, 2700);
    assert.equal(rustAlResolved.material.id, 'alu_6061');

    // 3. End-to-end MCP import_stl + async hydration preserves other scene parts, part_id, name, materialId, isFixed, and density
    const existingPart = {
      id: 'existing_bracket',
      name: 'Existing Bracket',
      materialId: 'steel_1045',
      isFixed: false,
      dag: [],
      mesh: createBoxMesh(0.06, 0.04, 0.02, 7850),
      visible: true,
      position: [0.15, 0, 0] as [number, number, number],
      rotation: [0, 0, 0] as [number, number, number],
      scale: [1, 1, 1] as [number, number, number],
    };

    const baseState: EngineStateSnapshot = {
      parts: [existingPart],
      selectedPartId: 'existing_bracket',
      joints: [],
      materials: [...MATERIALS_LIBRARY],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const definedMat = executeMcpToolOnEngineState(baseState, 'define_material', {
      id: 'custom_composite',
      name: 'Carbon Composite',
      young_modulus_gpa: 70,
      poisson_ratio: 0.25,
      yield_strength_mpa: 450,
      density_kg_m3: 1600,
    });

    const impRes = executeMcpToolOnEngineState(definedMat.nextState, 'import_stl', {
      file_path: 'output/imported_housing.stl',
      part_id: 'housing_fixed_01',
      name: 'Fixed Pyrex Housing',
      material: 'pyrex',
      is_fixed: true,
    });

    assert.equal(impRes.nextState.parts.length, 2);
    const importedBeforeHydrate = impRes.nextState.parts.find((p) => p.id === 'housing_fixed_01');
    assert.ok(importedBeforeHydrate, 'import_stl must create target part with requested part_id');

    const hydrateSpec = resolveNativeStlMaterialSpec(
      impRes.response.native_stl_url,
      impRes.nextState.materials,
      importedBeforeHydrate.materialId
    );
    assert.equal(hydrateSpec.density, 2500);
    const parsedHousingMesh = parseStlBuffer(binaryStl, hydrateSpec.density, 1.0);

    const hydrated = hydrateImportedStlPartInScene(
      impRes.nextState.parts,
      String(impRes.response.part_id),
      String(impRes.response.native_stl_url),
      parsedHousingMesh,
      impRes.nextState.materials
    );

    assert.equal(hydrated.parts.length, 2, 'Hydration must preserve all existing parts in scene');
    assert.strictEqual(
      hydrated.parts[0],
      impRes.nextState.parts[0],
      'Unrelated scene parts must remain completely untouched during STL hydration'
    );
    assert.equal(hydrated.parts[0].id, 'existing_bracket');
    assert.equal(hydrated.parts[0].materialId, 'steel_1045');
    assert.equal(hydrated.parts[0].mesh.mass, existingPart.mesh.mass);
    assert.ok(hydrated.hydratedPart, 'Target imported part must be hydrated');
    assert.equal(hydrated.hydratedPart.id, 'housing_fixed_01');
    assert.equal(hydrated.hydratedPart.name, 'Fixed Pyrex Housing');
    assert.equal(hydrated.hydratedPart.materialId, 'pyrex');
    assert.equal(hydrated.hydratedPart.isFixed, true);
    assert.equal(hydrated.hydratedPart.nativeStlUrl, '/output/imported_housing.stl');
    assert.equal(hydrated.hydratedPart.mesh.triangleCount, parsedHousingMesh.triangleCount);
    assert.ok(
      Math.abs(hydrated.hydratedPart.mesh.mass - parsedHousingMesh.volume * 2500) < 1e-6,
      `Expected hydrated mass for pyrex (~${(parsedHousingMesh.volume * 2500).toFixed(6)} kg), got ${hydrated.hydratedPart.mesh.mass}`
    );
    assert.equal(
      hydrated.hydratedPart.dag[0].params.triangles,
      parsedHousingMesh.triangleCount
    );

    // 4. Race guard: if materialId was switched to custom_composite (1600 kg/m³) while STL fetch was in flight,
    //    hydration rescales mass & inertia to 1600 kg/m³ without mutating the cached mesh
    const partsWithUpdatedMat = impRes.nextState.parts.map((p) =>
      p.id === 'housing_fixed_01' ? { ...p, materialId: 'custom_composite' } : p
    );
    const parsedMassBeforeRace = parsedHousingMesh.mass;
    const rescaledHydrated = hydrateImportedStlPartInScene(
      partsWithUpdatedMat,
      'housing_fixed_01',
      String(impRes.response.native_stl_url),
      parsedHousingMesh,
      impRes.nextState.materials
    );
    assert.ok(rescaledHydrated.hydratedPart);
    assert.ok(
      Math.abs(rescaledHydrated.hydratedPart.mesh.mass - parsedHousingMesh.volume * 1600) < 1e-6,
      'Expected hydration to match updated custom material density (1600 kg/m³)'
    );
    assert.equal(
      parsedHousingMesh.mass,
      parsedMassBeforeRace,
      'Cached parsedMesh must not be mutated during density rescaling'
    );

    // 5. Deleted part guard: if the imported part was removed before hydration finishes, scene is untouched
    const missingHydrated = hydrateImportedStlPartInScene(
      [existingPart],
      'housing_fixed_01',
      '/output/imported_housing.stl',
      parsedHousingMesh,
      impRes.nextState.materials
    );
    assert.equal(missingHydrated.hydratedPart, null);
    assert.equal(missingHydrated.parts.length, 1);
    assert.strictEqual(missingHydrated.parts[0], existingPart);

    // 6. Contract check on src/App.tsx: no hardcoded 7850 in handleLoadNativeStlUrl and hydratePartId is wired
    const appTsx = fs.readFileSync(path.resolve(process.cwd(), 'src/App.tsx'), 'utf8');
    assert.ok(
      !appTsx.includes('loadCachedRustStlMesh(stlUrl, 7850)'),
      'src/App.tsx must not hardcode density 7850 in handleLoadNativeStlUrl'
    );
    assert.ok(
      appTsx.includes('hydratePartId: String(response.part_id)'),
      'src/App.tsx must pass hydratePartId when hydrating MCP import_stl results'
    );
  });

  it('verifies P5-T34: docs/MCP_API_REFERENCE.md create_joint section matches Rust inputSchema, physics.rs units/ratio, and response metadata', () => {
    const modRs = fs.readFileSync(
      path.resolve(process.cwd(), 'crates/mcp-cad-server/src/tools/mod.rs'),
      'utf8'
    );
    const physicsRs = fs.readFileSync(
      path.resolve(process.cwd(), 'crates/mcp-cad-server/src/tools/physics.rs'),
      'utf8'
    );
    const stateRs = fs.readFileSync(
      path.resolve(process.cwd(), 'crates/mcp-cad-server/src/state.rs'),
      'utf8'
    );
    const apiRef = fs.readFileSync(
      path.resolve(process.cwd(), 'docs/MCP_API_REFERENCE.md'),
      'utf8'
    );

    // 1. Extract create_joint input_schema JSON from Rust mod.rs
    const schemaMatch = modRs.match(
      /name:\s*"create_joint"\.to_string\(\)[\s\S]*?input_schema:\s*json!\((\{[\s\S]*?\})\),\s*\n\s*\},/
    );
    assert.ok(schemaMatch, 'Could not locate create_joint input_schema in mod.rs');
    const rustSchema = JSON.parse(schemaMatch[1]) as {
      properties: Record<string, unknown>;
      required: string[];
    };

    const rustSchemaParams = Object.keys(rustSchema.properties);
    const rustRequiredParams = new Set(rustSchema.required);
    assert.ok(
      rustSchemaParams.length >= 14,
      `Expected >= 14 create_joint schema params in mod.rs, found ${rustSchemaParams.length}: ${rustSchemaParams.join(', ')}`
    );
    assert.deepEqual(
      Array.from(rustRequiredParams).sort(),
      ['joint_type', 'part_a', 'part_b'],
      'Expected required params of create_joint to be joint_type, part_a, part_b'
    );

    // 2. Confirm physics.rs supports limits_unit, computes ratio = ta / tb, and returns joint_id + length_unit + units
    const createJointFnMatch = physicsRs.match(
      /pub fn create_joint\([\s\S]*?\n\}/
    );
    assert.ok(createJointFnMatch, 'Could not locate pub fn create_joint in physics.rs');
    const createJointFn = createJointFnMatch[0];
    assert.ok(
      createJointFn.includes('args.get("limits_unit")'),
      'Expected physics.rs create_joint to read limits_unit'
    );
    assert.ok(
      createJointFn.includes('let r = ta / tb;'),
      'Expected physics.rs create_joint to compute ratio = ta / tb'
    );
    assert.ok(
      createJointFn.includes('"joint_id": joint_id') &&
        createJointFn.includes('"length_unit": unit.as_str()') &&
        createJointFn.includes('"units": unit.units_metadata()'),
      'Expected physics.rs create_joint response to contain joint_id, length_unit, and units'
    );

    // Extract units_metadata() keys from state.rs
    const unitsMetaMatch = stateRs.match(
      /pub fn units_metadata\(&self\)\s*->\s*Value\s*\{\s*json!\(\{([\s\S]*?)\}\)/
    );
    assert.ok(unitsMetaMatch, 'Could not locate units_metadata() in state.rs');
    const rustUnitsKeys = Array.from(
      unitsMetaMatch[1].matchAll(/^\s*"([a-z0-9_]+)":/gm),
      (m) => m[1]
    );
    assert.ok(
      rustUnitsKeys.length >= 17,
      `Expected >= 17 keys in LengthUnit::units_metadata(), found ${rustUnitsKeys.length}`
    );

    // 3. Isolate section ### 11. `create_joint` in docs/MCP_API_REFERENCE.md
    const secStart = apiRef.indexOf('### 11. `create_joint`');
    const secEnd = apiRef.indexOf('### 12. `set_joint_motor`');
    assert.ok(
      secStart !== -1 && secEnd > secStart,
      'Could not isolate section 11 (create_joint) in docs/MCP_API_REFERENCE.md'
    );
    const section = apiRef.slice(secStart, secEnd);

    // 4. Parse parameters table rows: | `param` ... | type | required | description |
    const tableRowRe = /^\|\s*`([a-z0-9_]+)`[^|]*\|\s*[^|]+\|\s*([^|]+)\|\s*([^|]+)\|/gm;
    const docTable = new Map<string, { required: string; desc: string }>();
    for (const m of section.matchAll(tableRowRe)) {
      docTable.set(m[1], { required: m[2].trim(), desc: m[3].trim() });
    }

    const expectedParams = Array.from(new Set([...rustSchemaParams, 'limits_unit']));
    for (const param of expectedParams) {
      assert.ok(
        docTable.has(param),
        `docs/MCP_API_REFERENCE.md create_joint table is missing parameter '${param}'`
      );
      const entry = docTable.get(param)!;
      if (rustRequiredParams.has(param)) {
        assert.ok(
          entry.required.includes('Да'),
          `Parameter '${param}' must be marked required ('Да'), got '${entry.required}'`
        );
      } else {
        assert.ok(
          entry.required.includes('Нет'),
          `Parameter '${param}' must be marked optional ('Нет'), got '${entry.required}'`
        );
      }
    }

    // 5. Verify units and gear ratio descriptions match Rust contract (no stale strings)
    const anchorADesc = docTable.get('anchor_a')!.desc;
    const anchorBDesc = docTable.get('anchor_b')!.desc;
    assert.ok(
      anchorADesc.includes('length_unit') && anchorADesc.includes('"mm"') && anchorADesc.includes('"m"'),
      `anchor_a must document active length_unit ("mm" default, "m"), got: ${anchorADesc}`
    );
    assert.ok(
      anchorBDesc.includes('length_unit') && anchorBDesc.includes('"mm"') && anchorBDesc.includes('"m"'),
      `anchor_b must document active length_unit ("mm" default, "m"), got: ${anchorBDesc}`
    );
    assert.ok(
      !anchorADesc.includes('(м)') && !anchorBDesc.includes('(м)'),
      'anchor_a/anchor_b must not claim meters-only "(м)"'
    );

    const limitsDesc = docTable.get('limits')!.desc;
    assert.ok(
      limitsDesc.includes('deg') &&
        limitsDesc.includes('limits_unit') &&
        limitsDesc.includes('rad') &&
        limitsDesc.includes('length_unit') &&
        limitsDesc.includes('"mm"'),
      `limits must document deg/rad via limits_unit for revolute and length_unit ("mm"/"m") for prismatic, got: ${limitsDesc}`
    );
    assert.ok(
      !limitsDesc.includes('в радианах или метрах'),
      'limits must not contain stale "(в радианах или метрах)" wording'
    );

    const lengthUnitDesc = docTable.get('length_unit')!.desc;
    assert.ok(
      lengthUnitDesc.includes('"mm"') && lengthUnitDesc.includes('"m"'),
      `length_unit must document "mm" and "m", got: ${lengthUnitDesc}`
    );

    const limitsUnitDesc = docTable.get('limits_unit')!.desc;
    assert.ok(
      limitsUnitDesc.includes('"deg"') && limitsUnitDesc.includes('"rad"'),
      `limits_unit must document "deg" and "rad", got: ${limitsUnitDesc}`
    );

    const teethADesc = docTable.get('teeth_a')!.desc;
    const teethBDesc = docTable.get('teeth_b')!.desc;
    const ratioDesc = docTable.get('ratio')!.desc;
    assert.ok(
      teethADesc.includes('ratio = teeth_a / teeth_b') &&
        teethBDesc.includes('ratio = teeth_a / teeth_b'),
      'teeth_a and teeth_b must document ratio = teeth_a / teeth_b'
    );
    assert.ok(
      !section.includes('i = -z_A / z_B'),
      'create_joint section must not contain stale formula i = -z_A / z_B'
    );
    assert.ok(
      ratioDesc.includes('-\\text{ratio}') && ratioDesc.includes('Положительное значение'),
      `ratio must document positive counter-rotation convention (omega_B = -ratio * omega_A), got: ${ratioDesc}`
    );

    // 6. Parse and validate all JSON example blocks in section 11
    const jsonBlocks = Array.from(section.matchAll(/```json\n([\s\S]*?)\n```/g), (m) => m[1]);
    assert.ok(
      jsonBlocks.length >= 3,
      `Expected >= 3 JSON blocks (requests + response) in create_joint section, found ${jsonBlocks.length}`
    );

    const allowedKeys = new Set([...expectedParams, 'id']);
    const requestExamples: Array<Record<string, unknown>> = [];
    let responsePayload: Record<string, unknown> | null = null;

    for (const rawBlock of jsonBlocks) {
      const parsed = JSON.parse(rawBlock) as Record<string, any>;
      assert.equal(parsed.jsonrpc, '2.0');
      if (parsed.method === 'tools/call') {
        assert.equal(parsed.params.name, 'create_joint');
        const args = parsed.params.arguments as Record<string, unknown>;
        for (const reqKey of rustRequiredParams) {
          assert.ok(
            Object.prototype.hasOwnProperty.call(args, reqKey),
            `create_joint request example is missing required parameter '${reqKey}'`
          );
        }
        for (const key of Object.keys(args)) {
          assert.ok(
            allowedKeys.has(key),
            `create_joint request example uses undocumented parameter '${key}'`
          );
        }
        requestExamples.push(args);
      } else if (parsed.result?.content?.[0]?.text) {
        responsePayload = JSON.parse(parsed.result.content[0].text) as Record<string, unknown>;
      }
    }

    assert.ok(requestExamples.length >= 2, 'Expected at least 2 create_joint request examples');
    assert.ok(responsePayload, 'Expected a JSON-RPC response example for create_joint');
    assert.equal(typeof responsePayload.joint_id, 'string');
    assert.equal(responsePayload.length_unit, 'mm');
    assert.ok(
      responsePayload.units && typeof responsePayload.units === 'object',
      'Response example must include units metadata object'
    );
    for (const unitKey of rustUnitsKeys) {
      assert.ok(
        Object.prototype.hasOwnProperty.call(responsePayload.units, unitKey),
        `create_joint response example is missing units.${unitKey}`
      );
    }

    // 7. Execute revolute & prismatic request examples against TS dispatcher to verify unit conversion
    const dummyBox = createBoxMesh(0.05, 0.05, 0.05, 7850);
    const makePart = (id: string, isFixed = false) => ({
      id,
      name: id,
      materialId: 'steel_1045',
      isFixed,
      dag: [],
      mesh: dummyBox,
      visible: true,
      position: [0, 0, 0] as [number, number, number],
      rotation: [0, 0, 0] as [number, number, number],
      scale: [1, 1, 1] as [number, number, number],
    });
    const testState: EngineStateSnapshot = {
      parts: [
        makePart('gearbox_housing', true),
        makePart('pinion_gear', false),
        makePart('driven_gear', false),
        makePart('slider_carriage', false),
      ],
      selectedPartId: 'pinion_gear',
      joints: [],
      materials: [...MATERIALS_LIBRARY],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    const revExample = requestExamples.find((r) => r.joint_type === 'revolute');
    assert.ok(revExample, 'Expected a revolute request example');
    const revRes = executeMcpToolOnEngineState(testState, 'create_joint', revExample);
    assert.equal(revRes.response.length_unit, 'mm');
    const revJoint = revRes.nextState.joints.find((j) => j.id === revExample.joint_id);
    assert.ok(revJoint && revJoint.limits, 'Revolute example should register joint with limits');
    assert.ok(
      Math.abs(revJoint.limits[0] - -Math.PI) < 1e-6 &&
        Math.abs(revJoint.limits[1] - Math.PI) < 1e-6,
      `Expected revolute limits [-180, 180] deg -> [-pi, pi] rad, got ${JSON.stringify(revJoint.limits)}`
    );

    const prismExample = requestExamples.find((r) => r.joint_type === 'prismatic');
    assert.ok(prismExample, 'Expected a prismatic request example');
    const prismRes = executeMcpToolOnEngineState(revRes.nextState, 'create_joint', prismExample);
    assert.equal(prismRes.response.length_unit, 'mm');
    const prismJoint = prismRes.nextState.joints.find((j) => j.id === prismExample.joint_id);
    assert.ok(prismJoint && prismJoint.limits, 'Prismatic example should register joint with limits');
    assert.ok(
      Math.abs(prismJoint.limits[0] - -0.25) < 1e-9 &&
        Math.abs(prismJoint.limits[1] - 0.25) < 1e-9,
      `Expected prismatic limits [-250, 250] mm -> [-0.25, 0.25] m, got ${JSON.stringify(prismJoint.limits)}`
    );
  });

  it('verifies P5-T35: prismatic joint.limits > 1.0 m are honored in SI meters without spurious 1e-3 division in stepKinematicsAssembly and press_simulation', () => {
    // 1. Long-stroke prismatic gantry via MCP create_joint (limits: [-1500, 1500] mm -> [-1.5, 1.5] m)
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
      part_id: 'gantry_carriage_p5_t35',
      primitive_type: 'box',
      params: { width: 0.1, height: 0.1, depth: 0.1 },
    }).nextState;

    state = executeMcpToolOnEngineState(state, 'create_joint', {
      joint_id: 'j_gantry_1500mm',
      joint_type: 'prismatic',
      part_a: 'ground',
      part_b: 'gantry_carriage_p5_t35',
      axis: [1, 0, 0],
      limits: [-1500, 1500], // default length_unit='mm' -> internal [-1.5, 1.5] m
    }).nextState;

    assert.deepEqual(state.joints[0].limits, [-1.5, 1.5]);

    // Drive at +2.0 m/s for 1.0 s -> should clamp at +1.5 m (NOT +0.0015 m!)
    state = executeMcpToolOnEngineState(state, 'set_joint_motor', {
      joint_id: 'j_gantry_1500mm',
      target_velocity: 2.0,
      max_force: 500.0,
    }).nextState;

    state = executeMcpToolOnEngineState(state, 'run_physics_simulation', {
      duration_seconds: 1.0,
      dt: 0.02,
    }).nextState;

    const carriagePos = state.parts.find((p) => p.id === 'gantry_carriage_p5_t35')!;
    assert.ok(
      Math.abs(carriagePos.position[0] - 1.5) < 1e-6,
      `Expected gantry carriage to reach +1.5 m travel limit, got ${carriagePos.position[0]}`
    );

    // Reverse at -4.0 m/s for 1.0 s -> from +1.5 m to -1.5 m clamp
    state = executeMcpToolOnEngineState(state, 'set_joint_motor', {
      joint_id: 'j_gantry_1500mm',
      target_velocity: -4.0,
      max_force: 500.0,
    }).nextState;

    const steppedReverse = stepKinematicsAssembly(state.parts, state.joints, 1.0);
    const carriageNeg = steppedReverse.updatedParts.find((p) => p.id === 'gantry_carriage_p5_t35')!;
    assert.ok(
      Math.abs(carriageNeg.position[0] - -1.5) < 1e-6,
      `Expected gantry carriage to clamp at -1.5 m on reverse stroke, got ${carriageNeg.position[0]}`
    );

    // 2. Large-stroke press assembly (> 1.0 m prismatic limits: [0, -1.25] m) in detectPressAssembly
    const steelDensity = 7850;
    const aluDensity = 2700;
    const basePart = {
      id: 'heavy_base',
      name: 'Heavy Base Plate',
      materialId: 'steel_1045',
      dag: [{ id: 'b1', name: 'Box', type: 'box' as const, params: { width: 0.5, height: 0.5, depth: 0.1 }, enabled: true }],
      mesh: createBoxMesh(0.5, 0.5, 0.1, steelDensity),
      visible: true,
      isFixed: true,
      position: [0, 0, -1.05] as [number, number, number],
      rotation: [0, 0, 0] as [number, number, number],
      scale: [1, 1, 1] as [number, number, number],
    };
    const tallColumn = {
      id: 'tall_specimen',
      name: 'Tall Column Specimen',
      materialId: 'alu_6061',
      dag: [{ id: 'c1', name: 'Box', type: 'box' as const, params: { width: 0.2, height: 0.2, depth: 2.0 }, enabled: true }],
      mesh: createBoxMesh(0.2, 0.2, 2.0, aluDensity),
      visible: true,
      isFixed: false,
      position: [0, 0, 0] as [number, number, number],
      rotation: [0, 0, 0] as [number, number, number],
      scale: [1, 1, 1] as [number, number, number],
    };
    const pressRam = {
      id: 'heavy_press_ram',
      name: 'Heavy Press Ram',
      materialId: 'steel_1045',
      dag: [{ id: 'p1', name: 'Box', type: 'box' as const, params: { width: 0.5, height: 0.5, depth: 0.1 }, enabled: true }],
      mesh: createBoxMesh(0.5, 0.5, 0.1, steelDensity),
      visible: true,
      isFixed: false,
      position: [0, 0, 1.05] as [number, number, number],
      rotation: [0, 0, 0] as [number, number, number],
      scale: [1, 1, 1] as [number, number, number],
    };
    const pressJoint = {
      id: 'ram_slider',
      name: 'Ram Slider',
      type: 'prismatic' as const,
      partA: 'heavy_base',
      partB: 'heavy_press_ram',
      anchor: [0, 0, -1.0] as [number, number, number],
      anchorB: [0, 0, 1.0] as [number, number, number],
      axis: [0, 0, 1] as [number, number, number],
      limits: [0, -1.25] as [number, number], // 1.25 m stroke (> 1.0 m!)
      motorSpeedRpm: 0,
      motorLinearSpeedMs: -0.5,
      motorTorqueNm: 50000,
      motorEnabled: true,
      currentAngleRad: 0,
    };

    const detected = detectPressAssembly(
      [basePart, tallColumn, pressRam],
      [pressJoint],
      DEFAULT_MATERIALS
    );
    assert.ok(detected, 'Expected large-stroke press assembly to be detected');
    assert.ok(
      Math.abs(detected.maxStrokeM - 1.25) < 1e-6,
      `Expected detectPressAssembly maxStrokeM to preserve 1.25 m (> 1.0 m) without dividing by 1000, got ${detected.maxStrokeM}`
    );
  });
});
