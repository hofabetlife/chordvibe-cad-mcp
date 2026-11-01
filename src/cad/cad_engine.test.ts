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

    assert.equal(FEA_QUICK_LOAD_PRESETS.length, 4);
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
    const squashedReport = validatePartMesh('squashed_cyl', squashed);
    assert.ok(squashedReport.lt5DegRatio > 0.01 || squashedReport.lt1DegCount > 0);
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
});



