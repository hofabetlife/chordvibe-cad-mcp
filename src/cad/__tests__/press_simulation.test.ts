import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';

import type { CadPart, KinematicJoint } from '../../types/cad';
import { bakeDestructionCacheSync, DEFAULT_DESTRUCTION_CONFIG } from '../destruction_engine';
import {
  mapRustJointsToWebStudio,
  shouldAutoSimulateLiveSnapshot,
  type LivePhysicsSimulation,
  type LiveSceneSnapshot,
} from '../live_sync_adapter';
import { DEFAULT_MATERIALS, getMaterialById } from '../materials';
import {
  buildDestructionCacheFromLivePhysics,
  detectPressAssembly,
} from '../press_simulation';
import { createBoxMesh, createHelixMesh } from '../primitives';
import { LiveSyncStore } from '../../server/live_sync_store';

function translateMeshInPlace(
  mesh: ReturnType<typeof createBoxMesh>,
  offset: [number, number, number]
): ReturnType<typeof createBoxMesh> {
  const verts = new Float32Array(mesh.vertices.length);
  for (let i = 0; i < mesh.vertices.length; i += 3) {
    verts[i] = mesh.vertices[i] + offset[0];
    verts[i + 1] = mesh.vertices[i + 1] + offset[1];
    verts[i + 2] = mesh.vertices[i + 2] + offset[2];
  }
  return {
    ...mesh,
    vertices: verts,
    boundingBox: {
      min: [
        mesh.boundingBox.min[0] + offset[0],
        mesh.boundingBox.min[1] + offset[1],
        mesh.boundingBox.min[2] + offset[2],
      ],
      max: [
        mesh.boundingBox.max[0] + offset[0],
        mesh.boundingBox.max[1] + offset[1],
        mesh.boundingBox.max[2] + offset[2],
      ],
    },
    centerOfMass: [
      mesh.centerOfMass[0] + offset[0],
      mesh.centerOfMass[1] + offset[1],
      mesh.centerOfMass[2] + offset[2],
    ],
  };
}

function rotateMeshX90(
  mesh: ReturnType<typeof createBoxMesh>
): ReturnType<typeof createBoxMesh> {
  const verts = new Float32Array(mesh.vertices.length);
  const norms = new Float32Array(mesh.normals.length);
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < mesh.vertices.length; i += 3) {
    const x = mesh.vertices[i];
    const y = mesh.vertices[i + 1];
    const z = mesh.vertices[i + 2];
    const nx = x;
    const ny = -z;
    const nz = y;
    verts[i] = nx;
    verts[i + 1] = ny;
    verts[i + 2] = nz;
    if (nx < minX) minX = nx;
    if (ny < minY) minY = ny;
    if (nz < minZ) minZ = nz;
    if (nx > maxX) maxX = nx;
    if (ny > maxY) maxY = ny;
    if (nz > maxZ) maxZ = nz;
  }
  for (let i = 0; i < mesh.normals.length; i += 3) {
    norms[i] = mesh.normals[i];
    norms[i + 1] = -mesh.normals[i + 2];
    norms[i + 2] = mesh.normals[i + 1];
  }
  return {
    ...mesh,
    vertices: verts,
    normals: norms,
    boundingBox: { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] },
    centerOfMass: [
      mesh.centerOfMass[0],
      -mesh.centerOfMass[2],
      mesh.centerOfMass[1],
    ],
  };
}

/**
 * Builds the exact clamped aluminum spring + steel press assembly from the user's MCP session:
 * - base_plate: 60x60x10 mm steel, fixed, top face at Z = -31.95 mm
 * - spring_1: aluminum 6061-T6 coil spring, wire Ø4 mm, height 60 mm (Z in [-31.95, +31.95] mm)
 * - press_plate: 60x60x10 mm steel, bottom face at Z = +31.95 mm
 * - press_slider: prismatic joint along Z [0,0,1], limits [0, -0.036] m, F_max = 2000 N
 */
function buildClampedSpringPressAssembly(): {
  parts: CadPart[];
  joints: KinematicJoint[];
} {
  const steelMat = getMaterialById('steel_1045', DEFAULT_MATERIALS);
  const aluMat = getMaterialById('alu_6061', DEFAULT_MATERIALS);

  // World-space baked STL meshes (just like Rust live_sync exports with position = [0,0,0])
  const baseMesh = translateMeshInPlace(
    createBoxMesh(0.06, 0.06, 0.01, steelMat.density),
    [0, 0, -0.03695]
  );
  const pressMesh = translateMeshInPlace(
    createBoxMesh(0.06, 0.06, 0.01, steelMat.density),
    [0, 0, 0.03695]
  );
  // Rust MCP helix is Z-up; createHelixMesh is Y-up so rotate +90° around X
  const springMesh = rotateMeshX90(
    createHelixMesh(
      0.016,
      0.002,
      0.01,
      0.06,
      1,
      true,
      'draft',
      aluMat.density
    )
  );

  const parts: CadPart[] = [
    {
      id: 'base_plate',
      name: 'Base Plate',
      materialId: 'steel_1045',
      dag: [{ id: 'd1', name: 'Box', type: 'box', params: { width: 0.06, height: 0.06, depth: 0.01 }, enabled: true }],
      mesh: baseMesh,
      visible: true,
      isFixed: true,
      position: [0, 0, 0],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
    },
    {
      id: 'spring_1',
      name: 'Aluminum Coil Spring',
      materialId: 'alu_6061',
      dag: [
        {
          id: 'd2',
          name: 'Helix',
          type: 'helix',
          params: { helix_radius: 0.016, wire_radius: 0.002, pitch: 0.01, height: 0.06 },
          enabled: true,
        },
      ],
      mesh: springMesh,
      visible: true,
      isFixed: false,
      position: [0, 0, 0],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
    },
    {
      id: 'press_plate',
      name: 'Press Plate',
      materialId: 'steel_1045',
      dag: [{ id: 'd3', name: 'Box', type: 'box', params: { width: 0.06, height: 0.06, depth: 0.01 }, enabled: true }],
      mesh: pressMesh,
      visible: true,
      isFixed: false,
      position: [0, 0, 0],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
    },
  ];

  const joints = mapRustJointsToWebStudio([
    {
      id: 'press_slider',
      part_a: 'base_plate',
      part_b: 'press_plate',
      joint_type: 'prismatic',
      anchor_a_m: [0, 0, -0.03195],
      anchor_b_m: [0, 0, 0.03195],
      axis: [0, 0, 1],
      limits: [0, -0.036],
      motor_target_velocity: -0.05,
      motor_max_force: 2000,
    },
  ]);

  return { parts, joints };
}

test('detectPressAssembly identifies clamped aluminum spring press rig along -Z', () => {
  const { parts, joints } = buildClampedSpringPressAssembly();

  assert.equal(joints[0].motorEnabled, true);
  assert.equal(joints[0].motorLinearSpeedMs, -0.05);
  assert.equal(joints[0].motorTorqueNm, 2000);

  const press = detectPressAssembly(parts, joints, DEFAULT_MATERIALS);
  assert.ok(press, 'Expected press assembly to be detected');
  assert.equal(press.basePart?.id, 'base_plate');
  assert.equal(press.pressPart.id, 'press_plate');
  assert.equal(press.specimenParts[0].id, 'spring_1');
  assert.equal(press.isHelicalSpring, true);
  assert.ok(press.pushDir[2] < -0.99, `Expected pushDir along -Z, got ${press.pushDir}`);
  assert.equal(press.maxForceN, 2000);
  assert.ok(press.maxStrokeM > 0.025 && press.maxStrokeM <= 0.038, `Stroke out of range: ${press.maxStrokeM}`);
});

test('bakeDestructionCacheSync simulates axial spring compression, yielding, coil bind & fracture under 2000 N', () => {
  const { parts, joints } = buildClampedSpringPressAssembly();
  const cache = bakeDestructionCacheSync(parts, joints, DEFAULT_MATERIALS, {
    ...DEFAULT_DESTRUCTION_CONFIG,
    scenarioMode: 'impact_crush',
    durationSec: 2.0,
    fps: 30,
  });

  assert.equal(cache.frames.length, 61);
  assert.ok(cache.maxStressOverallMPa > 276, `Expected σ_max > 276 MPa (Al 6061-T6 yield), got ${cache.maxStressOverallMPa}`);
  assert.ok(cache.maxPlasticStrainOverallPct > 1.0, `Expected significant plastic strain, got ${cache.maxPlasticStrainOverallPct}%`);
  assert.ok(cache.maxDamageOverall >= 0.85, `Expected critical damage D >= 0.85, got ${cache.maxDamageOverall}`);

  // Base plate must remain completely stationary across all frames
  for (const frame of cache.frames) {
    assert.deepEqual(frame.partStates.base_plate.position, [0, 0, 0]);
  }

  // Press plate moves downward along -Z and never penetrates base_plate
  const z0 = cache.frames[0].partStates.press_plate.position[2];
  const zMid = cache.frames[Math.floor(cache.frames.length * 0.5)].partStates.press_plate.position[2];
  const zEnd = cache.frames[cache.frames.length - 1].partStates.press_plate.position[2];
  assert.equal(z0, 0);
  assert.ok(zMid < -0.02, `Expected press_plate to move downward by mid-stroke, got z=${zMid}`);
  assert.ok(zEnd < zMid, `Expected press_plate to compress further, got zEnd=${zEnd}`);
  assert.ok(zEnd > -0.06, `Press plate must not pass through base_plate, got zEnd=${zEnd}`);

  // Mid-compression spring vertices: bottom anchored near 0, top tracking press plate downward
  const midSpring = cache.frames[Math.floor(cache.frames.length * 0.45)].partStates.spring_1;
  assert.ok(midSpring.deformedDisplacements, 'Expected per-vertex deformedDisplacements on spring_1');
  assert.ok(midSpring.nodalStressMPa, 'Expected per-vertex nodalStressMPa on spring_1');
  const springVerts = parts[1].mesh.vertices;
  const disp = midSpring.deformedDisplacements;
  let topDzSum = 0;
  let topCnt = 0;
  let botDzSum = 0;
  let botCnt = 0;
  for (let vi = 0; vi < parts[1].mesh.vertexCount; vi++) {
    const z = springVerts[vi * 3 + 2];
    if (z > 0.025) {
      topDzSum += disp[vi * 3 + 2];
      topCnt++;
    } else if (z < -0.025) {
      botDzSum += disp[vi * 3 + 2];
      botCnt++;
    }
  }
  assert.ok(topCnt > 0 && botCnt > 0);
  assert.ok(topDzSum / topCnt < -0.015, 'Top coils must compress downward along -Z');
  assert.ok(Math.abs(botDzSum / botCnt) < 0.004, 'Bottom coils must stay anchored on base_plate');

  // Verify rich timeline event milestones
  const eventTypes = new Set(cache.events.map((e) => e.type));
  assert.ok(eventTypes.has('contact'), 'Expected contact event');
  assert.ok(eventTypes.has('yield_onset'), 'Expected yield_onset event');
  assert.ok(eventTypes.has('tooth_shear'), 'Expected coil-bind event');
  assert.ok(eventTypes.has('body_fracture'), 'Expected overload fracture event');
  assert.ok(cache.shards.length >= 6, 'Expected fracture shards for spring rupture');
});

test('buildDestructionCacheFromLivePhysics upgrades rigid-locked press physics and handles free COM rotation without double-translation', () => {
  const { parts, joints } = buildClampedSpringPressAssembly();

  // Case A: Rigid-body solver locked press_plate against rigid spring_1 at Δz ≈ 0 with 2000 N impulse
  const lockedSim: LivePhysicsSimulation = {
    success: true,
    duration_s: 1.5,
    step_count: 90,
    step_size_s: 1 / 60,
    peak_joint_forces: { press_slider: 2000 },
    collisions: [
      {
        time_s: 0.016,
        body_a: 'press_plate',
        body_b: 'spring_1',
        event_type: 'started',
        max_impulse: 45,
      },
    ],
    trajectories: [
      {
        part_id: 'press_plate',
        samples: [
          { time_s: 0, position_m: [0, 0, 0.03695], orientation_quat: [0, 0, 0, 1], linear_velocity_m_s: [0, 0, -0.05], angular_velocity_rad_s: [0, 0, 0] },
          { time_s: 1.5, position_m: [0, 0, 0.0365], orientation_quat: [0, 0, 0, 1], linear_velocity_m_s: [0, 0, 0], angular_velocity_rad_s: [0, 0, 0] },
        ],
      },
    ],
  };

  const upgraded = buildDestructionCacheFromLivePhysics(
    parts,
    joints,
    DEFAULT_MATERIALS,
    lockedSim,
    DEFAULT_DESTRUCTION_CONFIG
  );
  assert.ok(upgraded.maxStressOverallMPa > 276, 'Upgraded press sim should show yielding stress > 276 MPa');
  assert.ok(
    upgraded.frames[upgraded.frames.length - 1].partStates.press_plate.position[2] < -0.025,
    'Upgraded press sim should animate full downward stroke'
  );

  // Case B: Pure free-body translation + 90° Z rotation around non-origin COM [0.05, 0, 0]
  const steelMat = getMaterialById('steel_1045', DEFAULT_MATERIALS);
  const freeBox: CadPart = {
    id: 'free_block',
    name: 'Free Block',
    materialId: 'steel_1045',
    dag: [],
    mesh: translateMeshInPlace(createBoxMesh(0.02, 0.02, 0.02, steelMat.density), [0.05, 0, 0]),
    visible: true,
    isFixed: false,
    position: [0, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
  };
  const freeSim: LivePhysicsSimulation = {
    success: true,
    duration_s: 1.0,
    step_count: 2,
    step_size_s: 0.5,
    peak_joint_forces: {},
    collisions: [],
    trajectories: [
      {
        part_id: 'free_block',
        samples: [
          { time_s: 0, position_m: [0.05, 0, 0], orientation_quat: [0, 0, 0, 1], linear_velocity_m_s: [0, 0, 0], angular_velocity_rad_s: [0, 0, 0] },
          { time_s: 1, position_m: [0.07, 0, 0], orientation_quat: [0, 0, Math.SQRT1_2, Math.SQRT1_2], linear_velocity_m_s: [0.02, 0, 0], angular_velocity_rad_s: [0, 0, 1.57] },
        ],
      },
    ],
  };

  const replayed = buildDestructionCacheFromLivePhysics(
    [freeBox],
    [],
    DEFAULT_MATERIALS,
    freeSim,
    DEFAULT_DESTRUCTION_CONFIG
  );
  // At t=0: meshPos must be [0,0,0] (no double translation!)
  assert.deepEqual(replayed.frames[0].partStates.free_block.position, [0, 0, 0]);
  // At t=1: p0 = [0.05,0,0] rotated +90 deg around Z -> [0, 0.05, 0], so meshPos = pt - rot(p0) = [0.07, -0.05, 0]
  const pEnd = replayed.frames[1].partStates.free_block.position;
  assert.ok(Math.abs(pEnd[0] - 0.07) < 1e-6);
  assert.ok(Math.abs(pEnd[1] - -0.05) < 1e-6);
  assert.ok(Math.abs(pEnd[2]) < 1e-6);
});

test('shouldAutoSimulateLiveSnapshot and LiveSyncStore preserve physics_simulation & coupled_sim.json', () => {
  const { joints } = buildClampedSpringPressAssembly();

  const staticSnap: LiveSceneSnapshot = {
    schema_version: '1.0.0',
    revision: 1,
    timestamp_ms: Date.now(),
    pid: 1,
    source: 'test',
    length_unit: 'mm',
    last_tool: { tool_name: 'add_feature_primitive', status: 'success', summary: 'box', timestamp_ms: 1, duration_ms: 1 },
    recent_tools: [],
    parts: [],
    joints: [],
  };
  assert.equal(shouldAutoSimulateLiveSnapshot(staticSnap, []), false);
  assert.equal(shouldAutoSimulateLiveSnapshot(staticSnap, joints), true);

  const tmpRoot = fs.mkdtempSync(path.join(os.tmpdir(), 'live-sim-store-'));
  const tmpBus = fs.mkdtempSync(path.join(os.tmpdir(), 'live-sim-bus-'));
  try {
    const store = new LiveSyncStore({ workspaceRoot: tmpRoot, tempLiveDir: tmpBus });
    store.pushSnapshot({
      parts: [{ id: 'spring_1', name: 'Spring' }],
      joints: [
        {
          id: 'press_slider',
          part_a: 'base_plate',
          part_b: 'press_plate',
          joint_type: 'prismatic',
          axis: [0, 0, 1],
          limits: [0, -0.036],
          motor_target_velocity: -0.05,
          motor_max_force: 2000,
        },
      ],
      physics_simulation: {
        success: true,
        duration_s: 1.0,
        step_count: 60,
        step_size_s: 1 / 60,
        peak_joint_forces: { press_slider: 2000 },
        collisions: [],
        trajectories: [],
      },
      coupled_simulation: {
        simulation_id: 'sim_1',
        accuracy_level: 'fast',
        duration_s: 1.0,
        frame_count: 10,
        peak_von_mises_mpa: 450,
        peak_damage: 0.9,
        detached_fragments_count: 4,
        events_count: 3,
        data_file: 'coupled_sim.json',
      },
      coupled_sim_payload: { summary: { simulation_id: 'sim_1' }, frames: [{ frame_index: 0 }] },
    });

    const read = store.readCurrentScene();
    assert.equal(read.active, true);
    assert.equal(read.snapshot?.physics_simulation?.peak_joint_forces.press_slider, 2000);
    assert.equal(read.snapshot?.coupled_simulation?.simulation_id, 'sim_1');
    const coupled = store.readCoupledSimJson() as { summary?: { simulation_id?: string } } | null;
    assert.equal(coupled?.summary?.simulation_id, 'sim_1');

    store.clearSnapshot();
    assert.equal(store.readCoupledSimJson(), null);
  } finally {
    fs.rmSync(tmpRoot, { recursive: true, force: true });
    fs.rmSync(tmpBus, { recursive: true, force: true });
  }
});
