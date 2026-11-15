import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { analyzeMechanism, checkInterference, sweepJoint } from '../kinematics.ts';
import { createBoxMesh } from '../primitives.ts';
import type { CadPart, KinematicJoint, MeshGeometryData } from '../../types/cad.ts';

const DUMMY_MESH: MeshGeometryData = {
  vertices: new Float32Array(0),
  normals: new Float32Array(0),
  indices: new Uint32Array(0),
  triangleCount: 12,
  vertexCount: 8,
  weldedVertexCount: 8,
  uniqueEdgeCount: 18,
  volume: 8e-6,
  surfaceArea: 0.0024,
  mass: 0.0628,
  boundingBox: { min: [-0.01, -0.01, -0.01], max: [0.01, 0.01, 0.01] },
  centerOfMass: [0, 0, 0],
  inertiaTensor: [1e-6, 1e-6, 1e-6],
  isWatertight: true,
  isManifold: true,
  isOutwardOriented: true,
  eulerCharacteristic: 2,
  eulerGenus: 0,
  boundaryEdgesCount: 0,
  nonManifoldEdgesCount: 0,
  inconsistentEdgesCount: 0,
  degenerateTrianglesCount: 0,
  zeroNormalsCount: 0,
  detectedPlanarFaces: [],
};

function makeDummyPart(
  id: string,
  position: [number, number, number] = [0, 0, 0],
  isFixed: boolean = false,
  mesh: MeshGeometryData = DUMMY_MESH
): CadPart {
  return {
    id,
    name: id,
    materialId: 'steel_1045',
    dag: [],
    mesh,
    visible: true,
    isFixed,
    position,
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
  };
}

function makeJoint(
  id: string,
  type: KinematicJoint['type'],
  partA: string,
  partB: string,
  anchor: [number, number, number] = [0, 0, 0],
  anchorB: [number, number, number] = [0, 0, 0],
  axis: [number, number, number] = [0, 0, 1],
  gearRatio?: number,
  motorEnabled: boolean = false
): KinematicJoint {
  return {
    id,
    name: id,
    type,
    partA,
    partB,
    anchor,
    anchorB,
    axis,
    motorSpeedRpm: motorEnabled ? 60 : 0,
    motorTorqueNm: 10,
    motorEnabled,
    currentAngleRad: 0,
    gearRatio,
  };
}

describe('P3-T9: analyze_mechanism (Chebychev-Grübler-Kutzbach & Jacobian SVD)', () => {
  it('1. 1 body on Revolute to ground -> DOF = 1', () => {
    const parts = [makeDummyPart('rotor')];
    const joints = [makeJoint('j_rev', 'revolute', 'ground', 'rotor')];
    const rep = analyzeMechanism(parts, joints);
    assert.equal(rep.dof_numeric, 1);
    assert.equal(rep.dof_grubler, 1);
    assert.equal(rep.redundant_constraints, 0);
    assert.equal(rep.status, 'PASS');
  });

  it('2. 1 body on Fixed to ground -> DOF = 0 (with locked mechanism warning)', () => {
    const parts = [makeDummyPart('welded')];
    const joints = [makeJoint('j_fix', 'fixed', 'ground', 'welded')];
    const rep = analyzeMechanism(parts, joints);
    assert.equal(rep.dof_numeric, 0);
    assert.equal(rep.dof_grubler, 0);
    assert.equal(rep.redundant_constraints, 0);
    assert.ok(rep.warnings.includes('DOF=0 при заявленном механизме'));
  });

  it('3. 1 body on Spherical to ground -> DOF = 3 (with DOF>1 without motors warning)', () => {
    const parts = [makeDummyPart('pendulum')];
    const joints = [makeJoint('j_sph', 'spherical', 'ground', 'pendulum')];
    const rep = analyzeMechanism(parts, joints);
    assert.equal(rep.dof_numeric, 3);
    assert.equal(rep.dof_grubler, 3);
    assert.equal(rep.redundant_constraints, 0);
    assert.ok(rep.warnings.includes('DOF>1 без двигателей'));
  });

  it('4. 1 free body -> DOF = 6 (floating/unconnected)', () => {
    const parts = [makeDummyPart('free_part')];
    const rep = analyzeMechanism(parts, []);
    assert.equal(rep.dof_numeric, 6);
    assert.equal(rep.dof_grubler, 6);
    assert.deepEqual(rep.unconnected_parts, ['free_part']);
    assert.deepEqual(rep.floating_parts, ['free_part']);
    assert.ok(rep.warnings.includes('есть свободные детали'));
  });

  it('5. Planar 4-bar linkage (4 links incl. ground, 4 Revolute about Z) -> dof_numeric = 1, dof_grubler = -2, redundant_constraints = 3', () => {
    // Ground pins at A=(0,0,0) and D=(0.1,0,0)
    // Crank AB: center (0, 0.02, 0), length 0.04 -> A at (0, -0.02, 0), B at (0, +0.02, 0)
    // Coupler BC: center (0.05, 0.04, 0) -> B at (-0.05, 0, 0), C at (+0.05, 0, 0)
    // Rocker CD: center (0.1, 0.02, 0) -> C at (0, +0.02, 0), D at (0, -0.02, 0)
    const parts = [
      makeDummyPart('crank', [0, 0.02, 0]),
      makeDummyPart('coupler', [0.05, 0.04, 0]),
      makeDummyPart('rocker', [0.1, 0.02, 0]),
    ];
    const joints = [
      makeJoint('j1', 'revolute', 'ground', 'crank', [0, 0, 0], [0, -0.02, 0], [0, 0, 1]),
      makeJoint('j2', 'revolute', 'crank', 'coupler', [0, 0.02, 0], [-0.05, 0, 0], [0, 0, 1]),
      makeJoint('j3', 'revolute', 'coupler', 'rocker', [0.05, 0, 0], [0, 0.02, 0], [0, 0, 1]),
      makeJoint('j4', 'revolute', 'ground', 'rocker', [0.1, 0, 0], [0, -0.02, 0], [0, 0, 1]),
    ];
    const rep = analyzeMechanism(parts, joints);
    assert.equal(rep.dof_numeric, 1);
    assert.equal(rep.dof_grubler, -2);
    assert.equal(rep.redundant_constraints, 3);
    assert.equal(rep.jacobian_rows, 20);
    assert.equal(rep.jacobian_cols, 18);
    assert.equal(rep.jacobian_rank, 17);
  });

  it('6. Slider-crank (4 links incl. ground, 3 Revolute + 1 Prismatic) -> dof_numeric = 1, dof_grubler = -2, redundant_constraints = 3', () => {
    const parts = [
      makeDummyPart('crank', [0, 0.015, 0]),
      makeDummyPart('conrod', [0.04, 0.015, 0]),
      makeDummyPart('slider', [0.08, 0, 0]),
    ];
    const joints = [
      makeJoint('j_o', 'revolute', 'ground', 'crank', [0, 0, 0], [0, -0.015, 0], [0, 0, 1]),
      makeJoint('j_a', 'revolute', 'crank', 'conrod', [0, 0.015, 0], [-0.04, 0.015, 0], [0, 0, 1]),
      makeJoint('j_b', 'revolute', 'conrod', 'slider', [0.04, -0.015, 0], [0, 0, 0], [0, 0, 1]),
      makeJoint('j_s', 'prismatic', 'ground', 'slider', [0, 0, 0], [0, 0, 0], [1, 0, 0]),
    ];
    const rep = analyzeMechanism(parts, joints);
    assert.equal(rep.dof_numeric, 1);
    assert.equal(rep.dof_grubler, -2);
    assert.equal(rep.redundant_constraints, 3);
  });

  it('7. Two gears on Revolute + Gear -> dof_numeric = 1, dof_grubler = 1', () => {
    const parts = [
      makeDummyPart('gear_1', [0, 0, 0]),
      makeDummyPart('gear_2', [0.09, 0, 0]),
    ];
    const joints = [
      makeJoint('j_rev_1', 'revolute', 'ground', 'gear_1', [0, 0, 0], [0, 0, 0], [0, 0, 1]),
      makeJoint('j_rev_2', 'revolute', 'ground', 'gear_2', [0.09, 0, 0], [0, 0, 0], [0, 0, 1]),
      makeJoint('j_gear', 'gear', 'gear_1', 'gear_2', [0, 0, 0], [0, 0, 0], [0, 0, 1], 2.0),
    ];
    const rep = analyzeMechanism(parts, joints);
    assert.equal(rep.dof_numeric, 1);
    assert.equal(rep.dof_grubler, 1);
    assert.equal(rep.redundant_constraints, 0);
    assert.deepEqual(rep.redundant_joint_ids, []);
    assert.deepEqual(rep.joints_at_limits, []);
    assert.equal(rep.status, 'PASS');
  });

  it('8. P3-T15: redundant_joint_ids excludes independent joints and joints_at_limits flags boundary stops', () => {
    const parts = [
      makeDummyPart('link1', [0.05, 0, 0]),
      makeDummyPart('link2', [0.15, 0, 0]),
    ];
    const jBase = makeJoint('j_base', 'revolute', 'ground', 'link1', [0, 0, 0], [-0.05, 0, 0], [0, 0, 1]);
    jBase.limits = [-Math.PI / 4, Math.PI / 4];
    const jArm = makeJoint('j_arm', 'revolute', 'link1', 'link2', [0.05, 0, 0], [-0.05, 0, 0], [0, 0, 1]);
    const jArmDup = makeJoint('j_arm_dup', 'revolute', 'link1', 'link2', [0.05, 0, 0], [-0.05, 0, 0], [0, 0, 1]);

    const repInside = analyzeMechanism(parts, [jBase, jArm, jArmDup]);
    assert.equal(repInside.dof_numeric, 2);
    assert.equal(repInside.redundant_constraints, 5);
    assert.deepEqual(repInside.redundant_joint_ids, ['j_arm', 'j_arm_dup']);
    assert.deepEqual(repInside.joints_at_limits, []);

    // Push jBase to upper limit with motor enabled in positive direction
    parts[0].rotation = [0, 0, Math.PI / 4];
    jBase.motorEnabled = true;
    jBase.motorSpeedRpm = 10;
    const repAtMax = analyzeMechanism([parts[0]], [jBase]);
    assert.equal(repAtMax.status, 'WARN');
    assert.equal(repAtMax.joints_at_limits.length, 1);
    assert.equal(repAtMax.joints_at_limits[0].joint_id, 'j_base');
    assert.equal(repAtMax.joints_at_limits[0].boundary, 'max');
    assert.equal(repAtMax.joints_at_limits[0].motor_blocked, true);
    assert.ok(repAtMax.warnings.some((w) => w.includes('limits') && w.includes('j_base')));

    // Zero-range limits [0, 0] lock DOF to 0
    parts[0].rotation = [0, 0, 0];
    jBase.limits = [0, 0];
    jBase.motorEnabled = false;
    const repLocked = analyzeMechanism([parts[0]], [jBase]);
    assert.equal(repLocked.dof_numeric, 0);
    assert.equal(repLocked.joints_at_limits[0].boundary, 'locked');
    assert.ok(repLocked.warnings.some((w) => w.includes('DOF=0')));
  });
});

describe('P3-T10: check_interference & sweep_joint', () => {
  it('1. Two overlapping boxes -> intersects = true, intersection_volume within 1% of analytical', () => {
    const boxMesh = createBoxMesh(0.02, 0.02, 0.02);
    // Box A at origin [-0.01, 0.01]^3, Box B shifted by +0.01m in X -> overlap [0, 0.01] x [-0.01, 0.01]^2
    // Analytical intersection volume = 0.01 * 0.02 * 0.02 = 4.0e-6 m^3
    const parts = [
      makeDummyPart('box_a', [0, 0, 0], false, boxMesh),
      makeDummyPart('box_b', [0.01, 0, 0], false, boxMesh),
    ];
    const rep = checkInterference(parts, []);
    assert.equal(rep.status, 'FAIL');
    assert.equal(rep.interfering_pairs_count, 1);
    const pair = rep.pairs[0];
    assert.equal(pair.intersects, true);
    const expectedVol = 4.0e-6;
    const relErr = Math.abs((pair.intersection_volume ?? 0) - expectedVol) / expectedVol;
    assert.ok(relErr < 0.01, `Relative volume error ${relErr} >= 1%`);
    assert.ok(Math.abs((pair.penetration_depth ?? 0) - 0.01) < 1e-4);
  });

  it('2. Two disjoint boxes and ignore_adjacent=true', () => {
    const boxMesh = createBoxMesh(0.02, 0.02, 0.02);
    // Box A at [0,0,0] (X in [-0.01, 0.01]), Box C at [0.03,0,0] (X in [0.02, 0.04]) -> clearance = 0.01m
    const parts = [
      makeDummyPart('box_a', [0, 0, 0], false, boxMesh),
      makeDummyPart('box_b', [0.01, 0, 0], false, boxMesh),
      makeDummyPart('box_c', [0.03, 0, 0], false, boxMesh),
    ];
    const joints = [makeJoint('j_ab', 'revolute', 'box_a', 'box_b')];

    const repDisjoint = checkInterference(parts, joints, {
      partIds: ['box_a', 'box_c'],
    });
    assert.equal(repDisjoint.status, 'PASS');
    assert.equal(repDisjoint.interfering_pairs_count, 0);
    assert.equal(repDisjoint.pairs[0].intersects, false);
    assert.ok(Math.abs((repDisjoint.pairs[0].clearance ?? 0) - 0.01) < 1e-4);

    // With ignoreAdjacent=true, overlapping pair (box_a, box_b) is skipped
    const repIgnoreAdj = checkInterference(parts, joints, {
      partIds: ['box_a', 'box_b'],
      ignoreAdjacent: true,
    });
    assert.equal(repIgnoreAdj.status, 'PASS');
    assert.equal(repIgnoreAdj.checked_pairs_count, 0);
  });

  it('3. sweep_joint on hinged arm hitting fixed wall -> detects collision angle within step size', () => {
    // Arm: length 100mm (0.10m), width 10mm (0.01m), height 10mm (0.01m), center at (0.05, 0, 0)
    // Revolute joint at origin (0, 0, 0) about +Z.
    // Wall: 20x20x20mm box centered at (0.05, 0.06, 0) -> Y in [0.05, 0.07], X in [0.04, 0.06]
    // Arm hits wall around 45 deg (pi/4 rad)
    const armMesh = createBoxMesh(0.10, 0.01, 0.01);
    const wallMesh = createBoxMesh(0.02, 0.02, 0.02);
    const parts = [
      makeDummyPart('arm', [0.05, 0, 0], false, armMesh),
      makeDummyPart('wall', [0.05, 0.06, 0], true, wallMesh),
    ];
    const joints = [
      makeJoint('j_hinge', 'revolute', 'ground', 'arm', [0, 0, 0], [-0.05, 0, 0], [0, 0, 1]),
    ];

    const sweep = sweepJoint(parts, joints, {
      jointId: 'j_hinge',
      from: 0,
      to: Math.PI / 2,
      steps: 18, // 5 deg per step
      ignoreAdjacent: true,
    });

    assert.equal(sweep.status, 'FAIL');
    assert.notEqual(sweep.first_collision_value, undefined);
    const firstDeg = sweep.first_collision_angle_deg!;
    assert.ok(
      firstDeg >= 30 && firstDeg <= 50,
      `Expected collision around 35..45 deg within 5 deg step size, got ${firstDeg} deg`
    );
    assert.deepEqual(sweep.first_collision_pairs, [['arm', 'wall']]);
  });

  it('4. Benchmark check_interference on 10 parts (45 pairs)', () => {
    const boxMesh = createBoxMesh(0.02, 0.02, 0.02);
    const parts: CadPart[] = [];
    for (let i = 0; i < 10; i++) {
      const x = i === 1 ? 0.01 : i * 0.05;
      parts.push(makeDummyPart(`part_${i}`, [x, 0, 0], false, boxMesh));
    }
    const t0 = performance.now();
    const rep = checkInterference(parts, []);
    const elapsedMs = performance.now() - t0;
    assert.equal(rep.checked_pairs_count, 45);
    assert.equal(rep.interfering_pairs_count, 1);
    console.log(`[BENCHMARK P3-T10] 10 parts (45 pairs) checkInterference: ${elapsedMs.toFixed(2)} ms`);
  });

  it('5. P3-T16: sweepJoint on Grashof 4-bar linkage rotates full 360 deg and detects follower obstacle collision', () => {
    const crankMesh = createBoxMesh(0.008, 0.04, 0.008);
    const couplerMesh = createBoxMesh(0.13, 0.008, 0.008);
    const rockerMesh = createBoxMesh(0.008, 0.09, 0.008);
    const wallMesh = createBoxMesh(0.01, 0.06, 0.02);

    const buildParts = (withWall: boolean) => {
      const parts: CadPart[] = [
        makeDummyPart('crank', [0.0, 0.02, 0.0], false, crankMesh),
        makeDummyPart('coupler', [0.06, 0.065, 0.02], false, couplerMesh),
        makeDummyPart('rocker', [0.12, 0.045, 0.0], false, rockerMesh),
      ];
      if (withWall) {
        parts.push(makeDummyPart('wall', [0.09, 0.075, 0.0], true, wallMesh));
      }
      return parts;
    };

    const joints = [
      makeJoint('j1_crank', 'revolute', 'ground', 'crank', [0, 0, 0], [0, -0.02, 0], [0, 0, 1]),
      makeJoint('j2_coupler_a', 'revolute', 'crank', 'coupler', [0, 0.02, 0], [-0.06, -0.025, -0.02], [0, 0, 1]),
      makeJoint('j3_coupler_b', 'revolute', 'coupler', 'rocker', [0.06, 0.025, -0.02], [0, 0.045, 0], [0, 0, 1]),
      makeJoint('j4_rocker', 'revolute', 'rocker', 'ground', [0, -0.045, 0], [0.12, 0, 0], [0, 0, 1]),
    ];

    const freeRep = sweepJoint(buildParts(false), joints, {
      jointId: 'j1_crank',
      from: 0,
      to: Math.PI * 2,
      steps: 72,
      ignoreAdjacent: true,
    });
    assert.equal(freeRep.closed_loop_detected, true);
    assert.equal(freeRep.lockup_detected, false);
    assert.equal(freeRep.status, 'PASS');
    assert.ok(freeRep.max_closure_residual < 1e-4, `residual=${freeRep.max_closure_residual}`);
    assert.equal(freeRep.samples.length, 73);

    const obsRep = sweepJoint(buildParts(true), joints, {
      jointId: 'j1_crank',
      from: 0,
      to: Math.PI,
      steps: 36,
      ignoreAdjacent: true,
    });
    assert.equal(obsRep.closed_loop_detected, true);
    assert.equal(obsRep.lockup_detected, false);
    assert.equal(obsRep.status, 'FAIL');
    assert.ok(
      obsRep.first_collision_pairs.some(
        ([a, b]) => (a === 'rocker' && b === 'wall') || (a === 'wall' && b === 'rocker')
      ),
      `Expected rocker-wall collision, got ${JSON.stringify(obsRep.first_collision_pairs)}`
    );
  });

  it('6. P3-T16: sweepJoint on slider-crank completes 360 deg and detects slider-endstop collision', () => {
    const crankMesh = createBoxMesh(0.008, 0.03, 0.008);
    const rodMesh = createBoxMesh(0.05, 0.008, 0.008);
    const sliderMesh = createBoxMesh(0.02, 0.02, 0.02);
    const wallMesh = createBoxMesh(0.01, 0.04, 0.04);

    const buildParts = (withWall: boolean) => {
      const parts: CadPart[] = [
        makeDummyPart('sc_crank', [0.0, 0.015, 0.0], false, crankMesh),
        makeDummyPart('sc_rod', [0.02, 0.015, 0.02], false, rodMesh),
        makeDummyPart('sc_slider', [0.04, 0.0, 0.0], false, sliderMesh),
      ];
      if (withWall) {
        parts.push(makeDummyPart('endstop', [0.075, 0.0, 0.0], true, wallMesh));
      }
      return parts;
    };

    const joints = [
      makeJoint('j_sc_crank', 'revolute', 'ground', 'sc_crank', [0, 0, 0], [0, -0.015, 0], [0, 0, 1]),
      makeJoint('j_sc_pin_b', 'revolute', 'sc_crank', 'sc_rod', [0, 0.015, 0], [-0.02, 0.015, -0.02], [0, 0, 1]),
      makeJoint('j_sc_pin_c', 'revolute', 'sc_rod', 'sc_slider', [0.02, -0.015, -0.02], [0, 0, 0], [0, 0, 1]),
      makeJoint('j_sc_rail', 'prismatic', 'sc_slider', 'ground', [0, 0, 0], [0, 0, 0], [1, 0, 0]),
    ];

    const freeRep = sweepJoint(buildParts(false), joints, {
      jointId: 'j_sc_crank',
      from: 0,
      to: Math.PI * 2,
      steps: 36,
      ignoreAdjacent: true,
    });
    assert.equal(freeRep.closed_loop_detected, true);
    assert.equal(freeRep.lockup_detected, false);
    assert.equal(freeRep.status, 'PASS');
    assert.ok(freeRep.max_closure_residual < 1e-4, `residual=${freeRep.max_closure_residual}`);

    const obsRep = sweepJoint(buildParts(true), joints, {
      jointId: 'j_sc_crank',
      from: 0,
      to: -Math.PI / 2,
      steps: 18,
      ignoreAdjacent: true,
    });
    assert.equal(obsRep.closed_loop_detected, true);
    assert.equal(obsRep.lockup_detected, false);
    assert.equal(obsRep.status, 'FAIL');
    assert.ok(
      obsRep.first_collision_pairs.some(
        ([a, b]) => (a === 'sc_slider' && b === 'endstop') || (a === 'endstop' && b === 'sc_slider')
      ),
      `Expected sc_slider-endstop collision, got ${JSON.stringify(obsRep.first_collision_pairs)}`
    );
  });

  it('7. P3-T16: sweepJoint detects geometric lockup in non-Grashof 4-bar linkage', () => {
    const parts: CadPart[] = [
      makeDummyPart('lk_crank', [0.05, 0.0, 0.0], false, createBoxMesh(0.10, 0.008, 0.008)),
      makeDummyPart('lk_coupler', [0.136, 0.048, 0.02], false, createBoxMesh(0.12, 0.008, 0.008)),
      makeDummyPart('lk_rocker', [0.186, 0.048, 0.0], false, createBoxMesh(0.10, 0.008, 0.008)),
    ];
    const joints = [
      makeJoint('j1_lk', 'revolute', 'ground', 'lk_crank', [0, 0, 0], [-0.05, 0, 0], [0, 0, 1]),
      makeJoint('j2_lk', 'revolute', 'lk_crank', 'lk_coupler', [0.05, 0, 0], [-0.036, -0.048, -0.02], [0, 0, 1]),
      makeJoint('j3_lk', 'revolute', 'lk_coupler', 'lk_rocker', [0.036, 0.048, -0.02], [-0.014, 0.048, 0], [0, 0, 1]),
      makeJoint('j4_lk', 'revolute', 'lk_rocker', 'ground', [0.014, -0.048, 0], [0.20, 0, 0], [0, 0, 1]),
    ];

    const rep = sweepJoint(parts, joints, {
      jointId: 'j1_lk',
      from: 0,
      to: (120 * Math.PI) / 180,
      steps: 24,
      ignoreAdjacent: true,
    });
    assert.equal(rep.closed_loop_detected, true);
    assert.equal(rep.lockup_detected, true);
    assert.equal(rep.status, 'FAIL');
    assert.equal(rep.lockup_step, 18);
    assert.ok(Math.abs((rep.lockup_angle_deg ?? 0) - 90.0) < 0.05, `got ${rep.lockup_angle_deg}`);
    assert.equal(rep.samples[18].locked_up, true);
    assert.ok(rep.max_closure_residual > 2e-4, `residual=${rep.max_closure_residual}`);
  });
});


