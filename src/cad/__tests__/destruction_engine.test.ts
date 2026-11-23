import test from 'node:test';
import assert from 'node:assert/strict';
import { createBoxMesh, createSpurGearMesh } from '../primitives';
import { DEFAULT_MATERIALS } from '../materials';
import {
  bakeDestructionCacheSync,
  buildGearCollider,
  DEFAULT_DESTRUCTION_CONFIG,
  formatCacheBytes,
  fracturePartIntoShards,
  getPresetDestructionConfig,
  measureGearPairMaxPenetration,
} from '../destruction_engine';
import type { CadPart } from '../../types/cad';

function makeOpposingGearsAssembly(centerHalfX = 0.0254): CadPart[] {
  const gearLeftMesh = createSpurGearMesh(
    18,
    2.2,
    20,
    0.016,
    0.012,
    0.024,
    0.006,
    0.004,
    0.0018,
    7850
  );
  const gearRightMesh = createSpurGearMesh(
    22,
    2.2,
    20,
    0.016,
    0.014,
    0.028,
    0.006,
    0.004,
    0.0018,
    8500
  );
  const benchMesh = createBoxMesh(0.2, 0.012, 0.09, 7850);

  return [
    {
      id: 'gear_clash_left',
      name: 'Ведущая шестерня Z1=18',
      materialId: 'steel_4140',
      dag: [
        {
          id: 'n1',
          name: 'Spur Gear Z18',
          type: 'spur_gear',
          params: {
            teeth: 18,
            module: 2.2,
            pressure_angle_deg: 20,
            face_width: 0.016,
            bore_diameter: 0.012,
          },
          enabled: true,
        },
      ],
      mesh: gearLeftMesh,
      visible: true,
      position: [-centerHalfX, 0, 0],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
    },
    {
      id: 'gear_clash_right',
      name: 'Встречная шестерня Z2=22',
      materialId: 'brass_c360',
      dag: [
        {
          id: 'n2',
          name: 'Spur Gear Z22',
          type: 'spur_gear',
          params: {
            teeth: 22,
            module: 2.2,
            pressure_angle_deg: 20,
            face_width: 0.016,
            bore_diameter: 0.014,
          },
          enabled: true,
        },
      ],
      mesh: gearRightMesh,
      visible: true,
      position: [centerHalfX, 0, 0],
      rotation: [0, 0, Math.PI / 22],
      scale: [1, 1, 1],
    },
    {
      id: 'test_rig_bench',
      name: 'Испытательный стенд',
      materialId: 'steel_4140',
      dag: [
        {
          id: 'n3',
          name: 'Bench Plate',
          type: 'box',
          params: { width: 0.2, height: 0.012, depth: 0.09 },
          enabled: true,
        },
      ],
      mesh: benchMesh,
      visible: true,
      isFixed: true,
      position: [0, -0.048, 0],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
    },
  ];
}

function makeOverloadGearsAssembly(): CadPart[] {
  const pinionMesh = createSpurGearMesh(
    20,
    2.0,
    20,
    0.015,
    0.01,
    0.022,
    0.005,
    0.003,
    0.0015,
    7850
  );
  const drivenMesh = createSpurGearMesh(
    30,
    2.0,
    20,
    0.015,
    0.014,
    0.028,
    0.005,
    0.004,
    0.002,
    2700
  );
  const benchMesh = createBoxMesh(0.22, 0.012, 0.09, 7850);

  return [
    {
      id: 'gear_overload_pinion',
      name: 'Шестерня-привод Z=20',
      materialId: 'steel_4140',
      dag: [
        {
          id: 'ov1',
          name: 'Pinion Z20',
          type: 'spur_gear',
          params: {
            teeth: 20,
            module: 2.0,
            pressure_angle_deg: 20,
            face_width: 0.015,
            bore_diameter: 0.01,
          },
          enabled: true,
        },
      ],
      mesh: pinionMesh,
      visible: true,
      position: [-0.025, 0, 0],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
    },
    {
      id: 'gear_overload_driven',
      name: 'Ведомое колесо Z=30',
      materialId: 'alu_6061',
      dag: [
        {
          id: 'ov2',
          name: 'Driven Z30',
          type: 'spur_gear',
          params: {
            teeth: 30,
            module: 2.0,
            pressure_angle_deg: 20,
            face_width: 0.015,
            bore_diameter: 0.014,
          },
          enabled: true,
        },
      ],
      mesh: drivenMesh,
      visible: true,
      position: [0.025, 0, 0],
      rotation: [0, 0, Math.PI / 30],
      scale: [1, 1, 1],
    },
    {
      id: 'test_rig_bench',
      name: 'Плита стенда',
      materialId: 'steel_1045',
      dag: [
        {
          id: 'ov3',
          name: 'Bench',
          type: 'box',
          params: { width: 0.22, height: 0.012, depth: 0.09 },
          enabled: true,
        },
      ],
      mesh: benchMesh,
      visible: true,
      isFixed: true,
      position: [0, -0.052, 0],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
    },
  ];
}

test('fracturePartIntoShards generates valid closed 3D shards conserving total mass', () => {
  const [leftGear] = makeOpposingGearsAssembly();
  const mat = DEFAULT_MATERIALS.find((m) => m.id === 'steel_4140')!;
  const shards = fracturePartIntoShards(leftGear, mat, 20, [0.02, 0, 0], 1337);

  assert.ok(shards.length >= 8, `Expected >= 8 shards, got ${shards.length}`);
  let massSum = 0;
  let totalInnerTris = 0;

  for (const s of shards) {
    assert.ok(s.vertices.length >= 9, 'Shard should contain outer triangles');
    assert.equal(s.vertices.length, s.normals.length);
    assert.ok(s.boundingRadius > 0 && Number.isFinite(s.boundingRadius));
    for (let i = 0; i < s.vertices.length; i++) {
      assert.ok(Number.isFinite(s.vertices[i]), 'All outer coords must be finite');
    }
    totalInnerTris += s.innerIndices.length / 3;
    massSum += s.mass;
  }

  assert.ok(
    totalInnerTris > 0,
    'Shards must stitch inner crystalline fracture faces along crack seams'
  );
  assert.ok(
    Math.abs(massSum - leftGear.mesh.mass) < 1e-6,
    `Mass conservation failed: ${massSum} vs ${leftGear.mesh.mass}`
  );
});

test('counter_clash enforces zero tooth interpenetration across ALL intact frames before dual-gear fracture', () => {
  // Test BOTH separated clash engagement (0.0254m) AND pre-meshed fixed shafts (0.022m)
  for (const halfX of [0.0254, 0.022]) {
    const parts = makeOpposingGearsAssembly(halfX);
    const colA = buildGearCollider(parts[0])!;
    const colB = buildGearCollider(parts[1])!;
    const cfg = {
      ...DEFAULT_DESTRUCTION_CONFIG,
      durationSec: 2.0,
      fps: 60,
      shardCountPerPart: 18,
    };

    const cache = bakeDestructionCacheSync(parts, [], DEFAULT_MATERIALS, cfg);

    assert.equal(cache.totalFrames, 120);
    assert.equal(cache.frames.length, 121);
    const leftShards = cache.shards.filter((s) => s.parentPartId === 'gear_clash_left');
    const rightShards = cache.shards.filter((s) => s.parentPartId === 'gear_clash_right');
    const benchShards = cache.shards.filter((s) => s.parentPartId === 'test_rig_bench');
    assert.ok(leftShards.length >= 10, `Left gear should fracture into shards, got ${leftShards.length}`);
    assert.ok(
      rightShards.length >= 10,
      `Right gear (gear_clash_right) must not be misclassified as static rig; got ${rightShards.length} shards`
    );
    assert.equal(benchShards.length, 0, 'Static test_rig_bench must remain unfractured');

    // CRITICAL COLLISION INVARIANT: ZERO tooth pass-through on EVERY frame where both gears are intact!
    let maxPenMeters = 0;
    for (const f of cache.frames) {
      const sA = f.partStates['gear_clash_left'];
      const sB = f.partStates['gear_clash_right'];
      if (sA.intact && sB.intact) {
        const pen = measureGearPairMaxPenetration(
          colA,
          [sA.position[0], sA.position[1]],
          sA.rotation[2],
          colB,
          [sB.position[0], sB.position[1]],
          sB.rotation[2]
        );
        if (pen > maxPenMeters) maxPenMeters = pen;
      }
    }
    assert.ok(
      maxPenMeters < 0.00045,
      `Interpenetration violation at halfX=${halfX}: max tooth overlap was ${(maxPenMeters * 1000).toFixed(3)} mm (must be < 0.45 mm)`
    );

    // Contact, tooth shear, and full body fracture events must occur promptly after physical contact
    const contactEv = cache.events.find((e) => e.type === 'contact');
    const firstShearEv = cache.events.find((e) => e.type === 'tooth_shear');
    const bodyEv = cache.events.find((e) => e.type === 'body_fracture');
    assert.ok(contactEv, `Should record physical SAT contact event (halfX=${halfX})`);
    assert.ok(firstShearEv, `Should record stage-1 tooth shear event (halfX=${halfX})`);
    assert.ok(bodyEv, `Should record stage-2 body fracture event (halfX=${halfX})`);
    assert.ok(
      firstShearEv!.frameIndex >= contactEv!.frameIndex &&
        firstShearEv!.frameIndex - contactEv!.frameIndex <= 20,
      `Tooth shear must happen promptly after physical contact (${contactEv!.frameIndex} -> ${firstShearEv!.frameIndex})`
    );

    // Final frame: both gears shattered (high clash torque), bench stays intact, shards stay above floorY
    const fLast = cache.frames[cache.frames.length - 1];
    assert.equal(fLast.partStates['gear_clash_left'].intact, false);
    assert.equal(fLast.partStates['gear_clash_right'].intact, false);
    assert.equal(fLast.partStates['test_rig_bench'].intact, true);
    for (let i = 0; i < cache.shards.length; i++) {
      if (fLast.shardActive[i] === 1) {
        const y = fLast.shardPos[i * 3 + 1];
        assert.ok(
          y >= cache.floorY - 1e-4,
          `Shard ${i} fell below floorY (${y} < ${cache.floorY})`
        );
      }
    }
  }
});

test('low-torque opposing clash physically jams symmetric gears and overpowers asymmetric gears with zero interpenetration', () => {
  // 1. Symmetric Z=20 vs Z=20 at equal low torque -> exact tangential force balance -> physical deadlock!
  const gMesh = createSpurGearMesh(20, 2.0, 20, 0.015, 0.01, 0.022, 0.005, 0.003, 0.0015, 7850);
  const symA: CadPart = {
    id: 'sym_gear_a',
    name: 'Gear A Z20',
    materialId: 'steel_4140',
    dag: [{ id: 'sa', name: 'G', type: 'spur_gear', params: { teeth: 20, module: 2.0, pressure_angle_deg: 20, face_width: 0.015, bore_diameter: 0.01 }, enabled: true }],
    mesh: gMesh,
    visible: true,
    position: [-0.02, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
  };
  const symB: CadPart = {
    id: 'sym_gear_b',
    name: 'Gear B Z20',
    materialId: 'steel_4140',
    dag: [{ id: 'sb', name: 'G', type: 'spur_gear', params: { teeth: 20, module: 2.0, pressure_angle_deg: 20, face_width: 0.015, bore_diameter: 0.01 }, enabled: true }],
    mesh: gMesh,
    visible: true,
    position: [0.02, 0, 0],
    rotation: [0, 0, Math.PI / 20],
    scale: [1, 1, 1],
  };
  const lowCfg = {
    ...DEFAULT_DESTRUCTION_CONFIG,
    durationSec: 1.0,
    fps: 60,
    impactTorqueNm: 18,
    brittleness: 0.3,
    shardCountPerPart: 12,
  };
  const symCache = bakeDestructionCacheSync([symA, symB], [], DEFAULT_MATERIALS, lowCfg);
  const symColA = buildGearCollider(symA)!;
  const symColB = buildGearCollider(symB)!;
  let symMaxPen = 0;
  for (const f of symCache.frames) {
    const sA = f.partStates['sym_gear_a'];
    const sB = f.partStates['sym_gear_b'];
    assert.equal(sA.intact, true);
    assert.equal(sB.intact, true);
    const pen = measureGearPairMaxPenetration(
      symColA,
      [sA.position[0], sA.position[1]],
      sA.rotation[2],
      symColB,
      [sB.position[0], sB.position[1]],
      sB.rotation[2]
    );
    if (pen > symMaxPen) symMaxPen = pen;
  }
  assert.ok(symMaxPen < 0.00045, `Symmetric deadlock overlap ${(symMaxPen * 1000).toFixed(3)} mm`);
  const symLast = symCache.frames[symCache.frames.length - 1];
  assert.ok(
    Math.abs(symLast.partStates['sym_gear_a'].rotation[2]) < 0.35,
    `Equal-radius symmetric gears must deadlock near start angle, got ${symLast.partStates['sym_gear_a'].rotation[2]} rad`
  );

  // 2. Asymmetric Z=18 vs Z=22 at equal low torque -> smaller pinion physically overpowers larger gear in -18/22 ratio with ZERO overlap!
  const parts = makeOpposingGearsAssembly(0.022);
  const colA = buildGearCollider(parts[0])!;
  const colB = buildGearCollider(parts[1])!;
  const cache = bakeDestructionCacheSync(parts, [], DEFAULT_MATERIALS, lowCfg);
  let maxPen = 0;
  for (const f of cache.frames) {
    const sA = f.partStates['gear_clash_left'];
    const sB = f.partStates['gear_clash_right'];
    assert.equal(sA.intact, true, 'Low torque must NOT shatter steel gear');
    assert.equal(sB.intact, true, 'Low torque must NOT shatter brass gear');
    const pen = measureGearPairMaxPenetration(
      colA,
      [sA.position[0], sA.position[1]],
      sA.rotation[2],
      colB,
      [sB.position[0], sB.position[1]],
      sB.rotation[2]
    );
    if (pen > maxPen) maxPen = pen;
  }
  assert.ok(
    maxPen < 0.00045,
    `Asymmetric low-torque gears must never pass through each other; max overlap ${(maxPen * 1000).toFixed(3)} mm`
  );
  const fLast = cache.frames[cache.frames.length - 1];
  const rotA = fLast.partStates['gear_clash_left'].rotation[2];
  const rotB = fLast.partStates['gear_clash_right'].rotation[2];
  assert.ok(rotA > 1.0 && rotB < -0.8, 'Smaller pinion Z18 should physically reverse Z22 under equal torque');
  const ratio = rotB / rotA;
  assert.ok(
    Math.abs(ratio - -18 / 22) < 0.08,
    `Overpowered gear ratio should match kinematic -18/22 (-0.818), got ${ratio.toFixed(3)}`
  );
});

test('torque_overload meshes Z20/Z30 cleanly with zero tooth interpenetration before overload fracture', () => {
  const parts = makeOverloadGearsAssembly();
  const colA = buildGearCollider(parts[0])!;
  const colB = buildGearCollider(parts[1])!;
  const cfg = {
    ...getPresetDestructionConfig('torque_overload'),
    durationSec: 2.0,
    fps: 60,
    shardCountPerPart: 16,
  };

  const cache = bakeDestructionCacheSync(parts, [], DEFAULT_MATERIALS, cfg);
  let maxPen = 0;
  let intactFrames = 0;
  for (const f of cache.frames) {
    const sA = f.partStates['gear_overload_pinion'];
    const sB = f.partStates['gear_overload_driven'];
    if (sA.intact && sB.intact) {
      intactFrames++;
      const pen = measureGearPairMaxPenetration(
        colA,
        [sA.position[0], sA.position[1]],
        sA.rotation[2],
        colB,
        [sB.position[0], sB.position[1]],
        sB.rotation[2]
      );
      if (pen > maxPen) maxPen = pen;
    }
  }
  assert.ok(intactFrames >= 5, 'Should mesh for several frames while torque ramps up');
  assert.ok(
    maxPen < 0.00045,
    `Meshed overload gears must not interpenetrate; max overlap ${(maxPen * 1000).toFixed(3)} mm`
  );
  assert.ok(
    cache.events.some((e) => e.type === 'tooth_shear'),
    'High overload torque should shear gear teeth'
  );
});

test('higher torque and brittleness cause earlier fracture onset deterministically', () => {
  const parts = makeOpposingGearsAssembly(0.0254);
  const baseCfg = {
    ...DEFAULT_DESTRUCTION_CONFIG,
    durationSec: 2.0,
    fps: 60,
    shardCountPerPart: 14,
    impactTorqueNm: 140,
    brittleness: 0.35,
  };
  const harshCfg = {
    ...baseCfg,
    impactTorqueNm: 320,
    brittleness: 0.9,
  };

  const c1 = bakeDestructionCacheSync(parts, [], DEFAULT_MATERIALS, baseCfg);
  const c1Repeat = bakeDestructionCacheSync(parts, [], DEFAULT_MATERIALS, baseCfg);
  const c2 = bakeDestructionCacheSync(parts, [], DEFAULT_MATERIALS, harshCfg);

  assert.deepEqual(c1.events, c1Repeat.events);
  assert.deepEqual(
    Array.from(c1.frames[60].shardPos),
    Array.from(c1Repeat.frames[60].shardPos)
  );

  const shear1 = c1.events.find((e) => e.type === 'tooth_shear');
  const shear2 = c2.events.find((e) => e.type === 'tooth_shear');
  assert.ok(shear1 && shear2, 'Both runs should trigger fracture');
  assert.ok(
    shear2!.frameIndex <= shear1!.frameIndex,
    `Higher torque/brittleness should fracture earlier (${shear2!.frameIndex} <= ${shear1!.frameIndex})`
  );
});

test('impact_crush mode fractures a single arbitrary CAD part cleanly', () => {
  const singleBox: CadPart = {
    id: 'box_specimen',
    name: 'Test Block',
    materialId: 'borosilicate_glass',
    dag: [{ id: 'b1', name: 'Box', type: 'box', params: { width: 0.06, height: 0.04, depth: 0.04 }, enabled: true }],
    mesh: createBoxMesh(0.06, 0.04, 0.04, 2500),
    visible: true,
    position: [0, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
  };

  const cfg = {
    ...getPresetDestructionConfig('impact_crush'),
    durationSec: 1.5,
    fps: 60,
    shardCountPerPart: 16,
  };
  const cache = bakeDestructionCacheSync([singleBox], [], DEFAULT_MATERIALS, cfg);
  assert.ok(cache.shards.length >= 6);
  assert.ok(
    cache.frames[cache.frames.length - 1].metrics.activeShardsCount > 0,
    'Single CAD part should shatter in impact_crush mode'
  );
  assert.match(formatCacheBytes(cache.cacheBytes), /(КБ|МБ|Б)$/);
});

test('P5-T7: baked frames populate per-vertex σ_vM, ε_p, D, deformed displacements, energy balance, and physical events', () => {
  const parts = makeOpposingGearsAssembly(0.0254);
  const cfg = {
    ...DEFAULT_DESTRUCTION_CONFIG,
    durationSec: 1.5,
    fps: 45,
    shardCountPerPart: 14,
  };
  const cache = bakeDestructionCacheSync(parts, [], DEFAULT_MATERIALS, cfg);

  assert.equal(cache.solverBackend, 'web_local_preview');
  assert.equal(cache.solverLabel, 'Web Local Preview');
  assert.ok(cache.yieldStrengthRefMPa > 100);
  assert.ok(cache.maxPlasticStrainOverallPct > 0, 'Should accumulate plastic strain under clash overload');
  assert.ok(cache.maxDamageOverall > 0.5, 'Should accumulate continuum damage D before fracture');
  assert.ok(cache.maxEnergyOverallJ > 0);

  let prevMaxEp = 0;
  let seenDeformedFrame = false;
  let sawPositiveElastic = false;
  let sawPositivePlastic = false;
  let sawPositiveFracture = false;

  for (const frame of cache.frames) {
    const m = frame.metrics;
    assert.ok(m.kineticEnergyJ >= 0 && Number.isFinite(m.kineticEnergyJ));
    assert.ok(m.elasticEnergyJ >= 0 && Number.isFinite(m.elasticEnergyJ));
    assert.ok(m.plasticEnergyJ >= 0 && Number.isFinite(m.plasticEnergyJ));
    assert.ok(m.fractureEnergyJ >= 0 && Number.isFinite(m.fractureEnergyJ));

    if (m.elasticEnergyJ > 0) sawPositiveElastic = true;
    if (m.plasticEnergyJ > 0) sawPositivePlastic = true;
    if (m.fractureEnergyJ > 0) sawPositiveFracture = true;

    const ps = frame.partStates['gear_clash_right'];
    if (ps && ps.intact && ps.deformedDisplacements && ps.nodalStressMPa && ps.nodalPlasticStrain && ps.nodalDamage) {
      const expectedVertCount = parts[1].mesh.vertices.length / 3;
      assert.equal(ps.deformedDisplacements.length, expectedVertCount * 3);
      assert.equal(ps.nodalStressMPa.length, expectedVertCount);
      assert.equal(ps.nodalPlasticStrain.length, expectedVertCount);
      assert.equal(ps.nodalDamage.length, expectedVertCount);

      if ((ps.maxDisplacementMm ?? 0) > 0) {
        seenDeformedFrame = true;
      }
      const curEp = ps.maxPlasticStrainPct ?? 0;
      assert.ok(
        curEp + 1e-4 >= prevMaxEp,
        `Irreversible plastic strain must be non-decreasing on intact part (${curEp} < ${prevMaxEp})`
      );
      prevMaxEp = Math.max(prevMaxEp, curEp);
    }
  }

  assert.ok(seenDeformedFrame, 'At least one intact loaded frame must carry non-zero vertex deformation');
  assert.ok(sawPositiveElastic, 'Elastic strain energy E_e must be positive during tooth loading');
  assert.ok(sawPositivePlastic, 'Plastic dissipation energy E_p must be positive after yield onset');
  assert.ok(sawPositiveFracture, 'Fracture energy E_f must be positive upon tooth/body fracture');

  assert.ok(
    cache.events.some((e) => e.type === 'yield_onset'),
    'Event log should include yield_onset (σ_vM > σ_y)'
  );
  assert.ok(
    cache.events.some((e) => e.type === 'fragment_detachment'),
    'Event log should include fragment_detachment'
  );
});

