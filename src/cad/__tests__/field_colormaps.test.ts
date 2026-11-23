import test from 'node:test';
import assert from 'node:assert/strict';
import {
  applyRigidAwareScaledDeformation,
  applyScaledDeformation,
  buildEnergyBalanceSeries,
  buildFieldLegend,
  computeYieldMarkerPct,
  convertRustCoupledSimToDestructionCache,
  DEFORMATION_SCALE_PRESETS,
  evalDamageColormap,
  evalPlasticStrainColormap,
  evalVonMisesColormap,
  fillVertexFieldColors,
} from '../field_colormaps';
import { createBoxMesh } from '../primitives';
import { DEFAULT_MATERIALS } from '../materials';
import { DEFAULT_DESTRUCTION_CONFIG } from '../destruction_engine';
import type { CadPart, DestructionCacheFrame } from '../../types/cad';

test('colormaps stay within [0,1] RGB bounds and produce monotonic engineering ramps', () => {
  const lowVm = evalVonMisesColormap(0, 600);
  const midVm = evalVonMisesColormap(300, 600);
  const highVm = evalVonMisesColormap(600, 600);

  for (const rgb of [lowVm, midVm, highVm]) {
    for (const c of rgb) {
      assert.ok(c >= 0 && c <= 1 && Number.isFinite(c));
    }
  }
  // Low stress is blue-dominant, high stress is red-dominant
  assert.ok(lowVm[2] > lowVm[0], 'Zero von Mises stress should be blue-dominant');
  assert.ok(highVm[0] > highVm[2], 'Peak von Mises stress should be red-dominant');

  const zeroEp = evalPlasticStrainColormap(0, 0.2);
  const peakEp = evalPlasticStrainColormap(0.2, 0.2);
  assert.ok(peakEp[0] > zeroEp[0], 'Plastic strain peak should be brighter/warmer than elastic zero');

  const intactD = evalDamageColormap(0);
  const brokenD = evalDamageColormap(1);
  assert.ok(brokenD[0] > intactD[0] && brokenD[0] > 0.8, 'Critical damage D=1 should map to crimson');
});

test('computeYieldMarkerPct and buildFieldLegend expose σ_y marker accurately', () => {
  const marker = computeYieldMarkerPct(415, 830);
  assert.equal(marker.exceeded, true);
  assert.ok(Math.abs(marker.pct - 50) < 1e-6);

  const legendVm = buildFieldLegend('von_mises', 920, 415, 12.5, 0.95);
  assert.equal(legendVm.mode, 'von_mises');
  assert.equal(legendVm.unit, 'МПа');
  assert.equal(legendVm.yieldExceeded, true);
  assert.ok(legendVm.yieldMarkerPct !== null && legendVm.yieldMarkerPct > 40 && legendVm.yieldMarkerPct < 50);
  assert.equal(legendVm.yieldLabel, 'σy 415');

  const legendEp = buildFieldLegend('plastic_strain', 920, 415, 14.2, 0.95);
  assert.equal(legendEp.unit, '%');
  assert.equal(legendEp.yieldMarkerPct, null);
  assert.equal(legendEp.maxValue, 14.2);

  const legendD = buildFieldLegend('damage', 920, 415, 14.2, 0.95);
  assert.equal(legendD.unit, '0..1');
  assert.equal(legendD.maxValue, 1.0);

  assert.deepEqual(Array.from(DEFORMATION_SCALE_PRESETS), [1, 5, 20]);
});

test('applyScaledDeformation scales displacements cleanly for 1x, 5x, 20x and handles missing data', () => {
  const base = new Float32Array([1, 2, 3, -1, 0, 4]);
  const disp = new Float32Array([0.001, -0.002, 0, 0.003, 0.001, -0.001]);
  const out = new Float32Array(6);

  assert.equal(applyScaledDeformation(base, disp, 1, out), true);
  assert.ok(Math.abs(out[0] - 1.001) < 1e-6);
  assert.ok(Math.abs(out[1] - 1.998) < 1e-6);

  assert.equal(applyScaledDeformation(base, disp, 5, out), true);
  assert.ok(Math.abs(out[0] - 1.005) < 1e-6);
  assert.ok(Math.abs(out[3] - -0.985) < 1e-6);

  assert.equal(applyScaledDeformation(base, disp, 20, out), true);
  assert.ok(Math.abs(out[0] - 1.02) < 1e-6);
  assert.ok(Math.abs(out[1] - 1.96) < 1e-6);

  // Fallback when displacements absent
  assert.equal(applyScaledDeformation(base, undefined, 20, out), false);
  assert.deepEqual(Array.from(out), Array.from(base));
});

test('applyRigidAwareScaledDeformation prevents bloating on 90° and 180° rotating gears at 5x/20x while amplifying tooth strain', () => {
  // Build a 3D ring of 16 vertices centered at shaft (-0.022, 0, 0) with radius R = 0.020 m
  const cx = -0.022;
  const R = 0.02;
  const n = 16;
  const base = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const a = (i * 2 * Math.PI) / n;
    const z = i % 2 === 0 ? -0.008 : 0.008;
    base[i * 3] = cx + R * Math.cos(a);
    base[i * 3 + 1] = R * Math.sin(a);
    base[i * 3 + 2] = z;
  }

  for (const theta of [Math.PI / 2, Math.PI]) {
    const c = Math.cos(theta);
    const s = Math.sin(theta);
    const disp = new Float32Array(n * 3);
    // Pure rigid rotation + local 0.25 mm radial indentation on tooth vertex 0
    for (let i = 0; i < n; i++) {
      const rx = base[i * 3] - cx;
      const ry = base[i * 3 + 1];
      const defX = cx + c * rx - s * ry;
      const defY = s * rx + c * ry;
      disp[i * 3] = defX - base[i * 3];
      disp[i * 3 + 1] = defY - base[i * 3 + 1];
      disp[i * 3 + 2] = 0;
    }

    // 1. Pure rigid rotation at 5x and 20x must preserve exact radius R (zero bloat!)
    for (const scale of [5, 20]) {
      const out = new Float32Array(n * 3);
      assert.equal(applyRigidAwareScaledDeformation(base, disp, scale, out), true);
      for (let i = 0; i < n; i++) {
        const rOut = Math.hypot(out[i * 3] - cx, out[i * 3 + 1]);
        assert.ok(
          Math.abs(rOut - R) < 1e-5,
          `At theta=${theta.toFixed(2)} scale=${scale}x, vertex ${i} radius bloated to ${rOut} (expected ${R})`
        );
      }
    }

    // 2. Add 0.25 mm local radial dent on vertex 0 -> at 5x, intact vertices keep radius R while dented vertex amplifies inward ~5x
    const dentMm = 0.00025;
    const def0X = base[0] + disp[0] - cx;
    const def0Y = base[1] + disp[1];
    const r0 = Math.hypot(def0X, def0Y);
    disp[0] -= (def0X / r0) * dentMm;
    disp[1] -= (def0Y / r0) * dentMm;

    const out5 = new Float32Array(n * 3);
    applyRigidAwareScaledDeformation(base, disp, 5, out5);
    const rDent5 = Math.hypot(out5[0] - cx, out5[1]);
    // Vertex 0 should dent inward by roughly 5x (~1.1-1.3 mm) while opposite vertex stays ~R
    const rOpp5 = Math.hypot(out5[8 * 3] - cx, out5[8 * 3 + 1]);
    assert.ok(
      R - rDent5 > dentMm * 3.5 && R - rDent5 < dentMm * 6.0,
      `Expected ~5x tooth dent (~${(dentMm * 5 * 1000).toFixed(2)}mm), got ${((R - rDent5) * 1000).toFixed(2)}mm`
    );
    assert.ok(
      Math.abs(rOpp5 - R) < 0.0003,
      `Opposite undeformed tooth should keep nominal radius, got error ${Math.abs(rOpp5 - R)}`
    );
  }
});

test('fillVertexFieldColors populates packed RGB buffer in-place for all 3 scalar modes', () => {
  const stress = new Float32Array([0, 500]);
  const ep = new Float32Array([0, 0.15]);
  const dmg = new Float32Array([0, 1.0]);
  const out = new Float32Array(6);

  fillVertexFieldColors('von_mises', 2, stress, ep, dmg, 500, 0.15, out);
  assert.ok(out[2] > out[0], 'Vertex 0 with 0 MPa should be blue');
  assert.ok(out[3] > out[5], 'Vertex 1 with 500 MPa should be red');

  fillVertexFieldColors('plastic_strain', 2, stress, ep, dmg, 500, 0.15, out);
  assert.ok(out[3] > out[0], 'Vertex 1 with ε_p=15% should be much brighter than vertex 0');

  fillVertexFieldColors('damage', 2, stress, ep, dmg, 500, 0.15, out);
  assert.ok(out[3] > 0.8 && out[5] < 0.25, 'Vertex 1 with D=1.0 should be crimson');
});

test('buildEnergyBalanceSeries outputs 4 valid SVG polyline series for multi-frame timeline', () => {
  const dummyFrame = (
    idx: number,
    ke: number,
    ee: number,
    pe: number,
    fe: number
  ): DestructionCacheFrame => ({
    frameIndex: idx,
    timeSec: idx * 0.01,
    partStates: {},
    shardPos: new Float32Array(0),
    shardQuat: new Float32Array(0),
    shardActive: new Uint8Array(0),
    shardStress: new Float32Array(0),
    shardStressMPa: new Float32Array(0),
    shardPlasticStrain: new Float32Array(0),
    shardDamage: new Float32Array(0),
    sparks: new Float32Array(0),
    metrics: {
      peakStressMPa: 100,
      yieldLimitMPa: 415,
      contactForceN: 1000,
      kineticEnergyJ: ke,
      elasticEnergyJ: ee,
      plasticEnergyJ: pe,
      fractureEnergyJ: fe,
      maxPlasticStrainPct: 1,
      maxDamage: 0.2,
      maxDisplacementMm: 0.1,
      activeShardsCount: 0,
      totalShardsCount: 0,
      fracturedPartsCount: 0,
      intactPartsCount: 1,
      statusLabel: 'test',
    },
  });

  const series = buildEnergyBalanceSeries([
    dummyFrame(0, 10, 0, 0, 0),
    dummyFrame(1, 6, 3, 1, 0),
    dummyFrame(2, 2, 1, 4, 3),
  ]);

  assert.equal(series.maxEnergyJ, 10);
  assert.equal(series.kineticPoints.split(' ').length, 3);
  assert.equal(series.elasticPoints.split(' ').length, 3);
  assert.equal(series.plasticPoints.split(' ').length, 3);
  assert.equal(series.fracturePoints.split(' ').length, 3);
});

test('convertRustCoupledSimToDestructionCache maps native Rust FEM payload into Rust FEM f64 cache', () => {
  const boxMesh = createBoxMesh(0.04, 0.04, 0.04, 7850);
  const part: CadPart = {
    id: 'gear_1',
    name: 'Gear 1',
    materialId: 'steel_4140',
    dag: [],
    mesh: boxMesh,
    visible: true,
    position: [0, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
  };
  const vertexCount = boxMesh.vertices.length / 3;
  const dispMm = new Array<number>(boxMesh.vertices.length).fill(0.5);
  const nodalStress = new Array<number>(vertexCount).fill(720);
  const nodalPlastic = new Array<number>(vertexCount).fill(0.06);
  const nodalDamage = new Array<number>(vertexCount).fill(0.82);

  const cache = convertRustCoupledSimToDestructionCache(
    {
      simulation_id: 'sim_rust_01',
      duration_s: 0.2,
      total_frames: 2,
      bake_time_ms: 42,
      events: [
        { frame_index: 0, time_s: 0.05, event_type: 'yield_onset', part_id: 'gear_1', description: 'Начало текучести', value: 650 },
        { frame_index: 1, time_s: 0.15, event_type: 'fragment_detached', part_id: 'gear_1', description: 'Скол зуба' },
      ],
    },
    [
      {
        simulation_id: 'sim_rust_01',
        frame_index: 0,
        total_frames: 2,
        time_s: 0.0,
        parts: [
          {
            part_id: 'gear_1',
            displacements_mm: dispMm,
            nodal_von_mises_mpa: nodalStress,
            nodal_equiv_plastic_strain: nodalPlastic,
            nodal_damage: nodalDamage,
            active_element_count: 100,
            max_displacement_mm: 0.5,
            max_von_mises_mpa: 720,
            max_equiv_plastic_strain: 0.06,
            max_damage: 0.82,
          },
        ],
        energy_balance: {
          kinetic_j: 12.5,
          elastic_strain_j: 3.1,
          plastic_dissipation_j: 4.4,
          fracture_energy_j: 0.0,
        },
      },
      {
        simulation_id: 'sim_rust_01',
        frame_index: 1,
        total_frames: 2,
        time_s: 0.2,
        parts: [
          {
            part_id: 'gear_1',
            displacements_mm: dispMm,
            nodal_von_mises_mpa: nodalStress,
            nodal_equiv_plastic_strain: nodalPlastic,
            nodal_damage: nodalDamage,
            eroded_element_indices: [1, 2, 3],
            active_element_count: 97,
            max_displacement_mm: 0.85,
            max_von_mises_mpa: 890,
            max_equiv_plastic_strain: 0.11,
            max_damage: 1.0,
          },
        ],
        detached_fragments: [
          {
            fragment_id: 'frag_tooth_1',
            parent_part_id: 'gear_1',
            position: [0.02, 0.01, 0],
            quaternion: [0, 0, 0, 1],
            mass_kg: 0.005,
            bounding_radius: 0.004,
          },
        ],
        contact_forces: [{ pair_id: 'g1_g2', normal_force_n: 4200, contact_point: [0.01, 0, 0] }],
        energy_balance: {
          kinetic_j: 6.2,
          elastic_strain_j: 1.4,
          plastic_dissipation_j: 8.9,
          fracture_energy_j: 2.5,
        },
      },
    ],
    [part],
    DEFAULT_MATERIALS,
    DEFAULT_DESTRUCTION_CONFIG
  );

  assert.equal(cache.solverBackend, 'rust_fem_f64');
  assert.equal(cache.solverLabel, 'Rust FEM f64');
  assert.equal(cache.rustSimulationId, 'sim_rust_01');
  assert.equal(cache.shards.length, 1);
  assert.equal(cache.frames.length, 2);

  const f1 = cache.frames[1];
  const ps = f1.partStates['gear_1'];
  assert.equal(ps.intact, true);
  assert.equal(ps.partiallyFractured, true);
  assert.ok(ps.deformedDisplacements && Math.abs(ps.deformedDisplacements[0] - 0.0005) < 1e-7);
  assert.equal(ps.nodalStressMPa?.[0], 720);
  assert.equal(f1.shardActive[0], 1);
  assert.equal(f1.metrics.plasticEnergyJ, 8.9);
  assert.equal(f1.metrics.fractureEnergyJ, 2.5);
  assert.equal(cache.events[0].type, 'yield_onset');
  assert.equal(cache.events[1].type, 'fragment_detachment');
});

test('P5-T15: maps active FEM surface triangles, erosion cavity topology change, and real detached fragment meshes (no proxy boxes)', () => {
  // CAD part has a 24-vertex box mesh, intentionally DIFFERENT from the FEM active surface mesh vertex count (5 -> 4)
  const cadBox = createBoxMesh(0.08, 0.02, 0.02, 2230);
  const glassBar: CadPart = {
    id: 'glass_bar',
    name: 'Brittle Glass Bar',
    materialId: 'steel_4140',
    dag: [],
    mesh: cadBox,
    visible: true,
    position: [0, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
  };

  // Real tetrahedron-shaped detached fragment mesh fetched once from /api/coupled-sim/fragments/:sim_id
  const frag1Mesh = {
    fragment_id: 'frag_glass_bar_0',
    source_part_id: 'glass_bar',
    vertices_m: [
      [-0.005, -0.004, -0.004],
      [0.006, -0.004, -0.004],
      [0.0, 0.007, -0.004],
      [0.0, 0.0, 0.008],
    ] as [number, number, number][],
    normals: [
      [-0.577, -0.577, -0.577],
      [0.577, -0.577, -0.577],
      [0.0, 0.707, -0.707],
      [0.0, 0.0, 1.0],
    ] as [number, number, number][],
    indices: [0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    bounding_radius_m: 0.008,
    vertex_count: 4,
    triangle_count: 4,
  };

  // Second detached fragment transmitted inline via query_simulation_frame (include_fragment_meshes=true)
  const frag2InlineMesh = {
    fragment_id: 'frag_glass_bar_1',
    source_part_id: 'glass_bar',
    vertices_m: [
      [0.0, 0.0, 0.0],
      [0.01, 0.0, 0.0],
      [0.005, 0.009, 0.0],
      [0.005, 0.003, 0.009],
      [0.005, 0.003, -0.009],
    ] as [number, number, number][],
    indices: [0, 1, 3, 1, 2, 3, 2, 0, 3, 0, 2, 4, 2, 1, 4, 1, 0, 4],
    bounding_radius_m: 0.01,
  };

  const cache = convertRustCoupledSimToDestructionCache(
    {
      simulation_id: 'sim_p5_t15',
      accuracy_level: 'standard',
      duration_s: 0.004,
      frame_count: 2,
      elapsed_ms: 18,
      fragment_meshes: [frag1Mesh],
      events: [
        {
          time_s: 0.002,
          kind: 'fragment_detached',
          part_id: 'glass_bar',
          detail: 'Detached 2 real FEM fragments',
        },
      ],
    },
    [
      {
        simulation_id: 'sim_p5_t15',
        frame_index: 0,
        total_frames: 2,
        time_s: 0.0,
        units: { length: 'm' },
        parts: [
          {
            part_id: 'glass_bar',
            node_count: 6,
            active_element_count: 10,
            eroded_element_count: 0,
            eroded_element_indices: [],
            surface_vertex_count: 5,
            surface_triangle_count: 6,
            surface_reference_positions_m: [
              [0.0, 0.0, 0.0],
              [0.04, 0.0, 0.0],
              [0.02, 0.02, 0.0],
              [0.02, 0.01, 0.02],
              [0.02, 0.01, -0.02],
            ],
            surface_positions_m: [
              [0.0, 0.0, 0.0],
              [0.0402, 0.0, 0.0],
              [0.02, 0.0201, 0.0],
              [0.02, 0.01, 0.02],
              [0.02, 0.01, -0.02],
            ],
            surface_indices: [0, 1, 3, 1, 2, 3, 2, 0, 3, 0, 2, 4, 2, 1, 4, 1, 0, 4],
            surface_node_indices: [0, 2, 3, 4, 5],
            nodal_von_mises_mpa: [10, 999, 120, 45, 30, 25],
            nodal_equiv_plastic_strain: [0, 0.5, 0.01, 0.002, 0, 0],
            nodal_damage: [0, 1.0, 0.15, 0.05, 0, 0],
            max_displacement_mm: 0.2,
            max_von_mises_mpa: 120,
            max_equiv_plastic_strain: 0.01,
            max_damage: 0.15,
          },
        ],
        detached_fragments: [],
        energy_balance: {
          kinetic_j: 0.5,
          elastic_j: 0.25,
          plastic_j: 0.0,
          fracture_j: 0.0,
        },
      },
      {
        simulation_id: 'sim_p5_t15',
        frame_index: 1,
        total_frames: 2,
        time_s: 0.004,
        units: { length: 'm' },
        parts: [
          {
            part_id: 'glass_bar',
            node_count: 6,
            active_element_count: 6,
            eroded_element_count: 4,
            eroded_element_indices: [6, 7, 8, 9],
            surface_vertex_count: 4,
            surface_triangle_count: 4,
            surface_reference_positions_m: [
              [0.0, 0.0, 0.0],
              [0.02, 0.02, 0.0],
              [0.02, 0.01, 0.02],
              [0.02, 0.01, -0.02],
            ],
            surface_positions_m: [
              [0.0, 0.0, 0.0],
              [0.021, 0.0205, 0.0],
              [0.0205, 0.01, 0.02],
              [0.0205, 0.01, -0.02],
            ],
            surface_indices: [0, 1, 2, 0, 2, 3, 1, 3, 2, 0, 3, 1],
            surface_node_indices: [0, 3, 4, 5],
            nodal_von_mises_mpa: [20, 999, 800, 350, 210, 190],
            nodal_equiv_plastic_strain: [0, 0.5, 0.2, 0.04, 0.02, 0.01],
            nodal_damage: [0, 1.0, 1.0, 0.92, 0.65, 0.5],
            max_displacement_mm: 1.12,
            max_von_mises_mpa: 350,
            max_equiv_plastic_strain: 0.04,
            max_damage: 0.92,
          },
        ],
        detached_fragments: [
          {
            fragment_id: 'frag_glass_bar_0',
            source_part_id: 'glass_bar',
            position_m: [0.055, -0.008, 0.002],
            orientation_quat: [0.0, 0.0, 0.3826834, 0.9238795],
            mass_kg: 0.002,
            bounding_radius_m: 0.008,
          },
          {
            fragment_id: 'frag_glass_bar_1',
            source_part_id: 'glass_bar',
            position: [0.062, -0.012, -0.003],
            orientation_quat: [0, 0, 0, 1],
            mass_kg: 0.003,
            mesh: frag2InlineMesh,
          },
        ],
        contact_forces: [
          { body_a: 'glass_bar', body_b: 'anvil', normal_force_n: 8500, point_m: [0.04, 0.0, 0.0] },
        ],
        energy_balance: {
          kinetic_j: 1.8,
          elastic_j: 0.4,
          plastic_j: 0.9,
          fracture_j: 1.25,
        },
      },
    ],
    [glassBar],
    DEFAULT_MATERIALS,
    DEFAULT_DESTRUCTION_CONFIG
  );

  // 1. Verify active FEM surface mesh is built directly regardless of CAD mesh vertex count
  assert.ok(cadBox.vertexCount > 20 && cadBox.vertexCount !== 5);
  const s0 = cache.frames[0].partStates['glass_bar'].femSurfaceMesh;
  const s1 = cache.frames[1].partStates['glass_bar'].femSurfaceMesh;
  assert.ok(s0, 'Frame 0 must contain active FEM surface mesh');
  assert.ok(s1, 'Frame 1 must contain active FEM surface mesh');
  assert.equal(s0.vertexCount, 5);
  assert.equal(s0.triangleCount, 6);
  assert.equal(s1.vertexCount, 4, 'Erosion cavity must update compacted surface vertex count');
  assert.equal(s1.triangleCount, 4, 'Erosion cavity must update surface triangle count');

  // Verify surface-gathered nodal fields skip unreferenced interior node 1 (stress 999)
  assert.equal(s0.nodalStressMPa?.length, 5);
  assert.equal(s0.nodalStressMPa?.[0], 10);
  assert.equal(s0.nodalStressMPa?.[1], 120);
  assert.ok(Math.abs((s1.nodalDamage?.[1] ?? 0) - 0.92) < 1e-6);

  // Verify 20x deformation scaling operates on active FEM surface buffers cleanly
  const scaledOut = new Float32Array(s0.referenceVertices.length);
  const moved = applyScaledDeformation(s0.referenceVertices, s0.displacements, 20, scaledOut);
  assert.equal(moved, true);
  // Vertex 1 moved by +0.0002 m in X -> at 20x it should be 0.04 + 0.004 = 0.044 m
  assert.ok(Math.abs(scaledOut[3] - 0.044) < 1e-6);

  // 2. Verify detached fragments use real FEM triangle geometry (NOT 8-vertex/36-index proxy boxes!)
  assert.equal(cache.shards.length, 2);
  const shard0 = cache.shards[0];
  const shard1 = cache.shards[1];
  assert.equal(shard0.id, 'frag_glass_bar_0');
  assert.equal(shard0.parentPartId, 'glass_bar');
  assert.equal(shard0.vertices.length, 4 * 3, 'Shard 0 must use cached 4-vertex FEM mesh, not 8-vertex proxy box');
  assert.equal(shard0.indices.length, 12, 'Shard 0 must use cached 4-triangle FEM mesh, not 12-triangle proxy box');
  assert.ok(Math.abs(shard0.vertices[0] - -0.005) < 1e-7);

  assert.equal(shard1.id, 'frag_glass_bar_1');
  assert.equal(shard1.vertices.length, 5 * 3, 'Shard 1 must use inline 5-vertex FEM mesh, not proxy box');
  assert.equal(shard1.indices.length, 18, 'Shard 1 must use inline 6-triangle FEM mesh');
  assert.equal(shard1.normals.length, 5 * 3, 'Smooth vertex normals must be auto-computed when omitted');

  // 3. Verify synchronized world-meter poses, energy channels, and contact point from Rust wire format
  const f1 = cache.frames[1];
  assert.equal(f1.shardActive[0], 1);
  assert.equal(f1.shardActive[1], 1);
  assert.ok(Math.abs(f1.shardPos[0] - 0.055) < 1e-7);
  assert.ok(Math.abs(f1.shardQuat[2] - 0.3826834) < 1e-6);
  assert.equal(f1.metrics.elasticEnergyJ, 0.4);
  assert.equal(f1.metrics.plasticEnergyJ, 0.9);
  assert.equal(f1.metrics.fractureEnergyJ, 1.25);
  assert.deepEqual(cache.contactPoint, [0.04, 0.0, 0.0]);
  assert.equal(cache.bakeTimeMs, 18);
  assert.equal(cache.events[0].type, 'fragment_detachment');
});

