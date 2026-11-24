import test from 'node:test';
import assert from 'node:assert/strict';
import {
  estimateRemainingMs,
  formatDurationMsRu,
  IncrementalRustSimAssembler,
  liveProgressToSolveStatus,
} from '../coupled_sim_stream';
import { createBoxMesh, createSpurGearMesh } from '../primitives';
import { DEFAULT_MATERIALS } from '../materials';
import {
  bakeDestructionCacheAsync,
  DEFAULT_DESTRUCTION_CONFIG,
} from '../destruction_engine';
import type { CadPart, KinematicJoint } from '../../types/cad';

function makeTestGearPart(id: string, xOffset: number): CadPart {
  return {
    id,
    name: `Gear ${id}`,
    materialId: 'steel_1045',
    dag: [],
    mesh: createSpurGearMesh(18, 2.0, 20, 0.014, 0.01, 24),
    visible: true,
    isFixed: false,
    position: [xOffset, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
  };
}

test('estimateRemainingMs and formatDurationMsRu compute and format ETAs cleanly', () => {
  assert.equal(estimateRemainingMs(100, 25), null);
  assert.equal(estimateRemainingMs(2000, 1), null);
  assert.equal(estimateRemainingMs(5000, 100), null);
  assert.equal(estimateRemainingMs(3000, 25), 9000);

  assert.equal(formatDurationMsRu(null), '—');
  assert.equal(formatDurationMsRu(450), '450 мс');
  assert.equal(formatDurationMsRu(4200), '4.2 с');
  assert.equal(formatDurationMsRu(59_950), '1м 00с');
  assert.equal(formatDurationMsRu(125_000), '2м 05с');
  assert.equal(formatDurationMsRu(3_720_000), '1ч 02м');
});

test('liveProgressToSolveStatus maps running and terminal payloads accurately', () => {
  const running = liveProgressToSolveStatus(
    {
      job_id: 'job_42',
      simulation_id: 'sim_9',
      state: 'running',
      progress_pct: 35,
      time_s: 0.0175,
      duration_s: 0.05,
      step_count: 140,
      frame_count: 12,
      frames_cached: 4,
      latest_frame_index: 3,
      current_dt_s: 0.000125,
      bisections: 1,
      eroded_elements: 2,
      detached_fragments: 1,
      max_von_mises_mpa: 482.5,
      elapsed_ms: 3500,
      stage_label: 'Интегрирование HHT-α · кадр 4/12',
    },
    'live_sync'
  );

  assert.equal(running.active, true);
  assert.equal(running.source, 'live_sync');
  assert.equal(running.progressPct, 35);
  assert.equal(running.currentTimeSec, 0.0175);
  assert.equal(running.totalTimeSec, 0.05);
  assert.equal(running.framesCached, 4);
  assert.equal(running.latestFrameIndex, 3);
  assert.equal(running.cancellable, true);
  assert.equal(running.estimatedRemainingMs, 6500);
  assert.equal(running.maxVonMisesMPa, 482.5);

  const rustAliased = liveProgressToSolveStatus({
    job_id: 'job_alias',
    simulation_id: 'sim_alias',
    state: 'running',
    progress_pct: 50,
    current_time_s: 0.025,
    total_time_s: 0.05,
    step_count: 200,
    frame_count: 10,
    frames_cached: 5,
    current_dt_s: 0.000125,
    bisections: 0,
    eroded_elements: 0,
    detached_fragments: 0,
    max_von_mises_mpa: 400,
    elapsed_ms: 4000,
  });
  assert.equal(rustAliased.currentTimeSec, 0.025);
  assert.equal(rustAliased.totalTimeSec, 0.05);

  const completed = liveProgressToSolveStatus({
    job_id: 'job_42',
    simulation_id: 'sim_9',
    state: 'completed',
    progress_pct: 100,
    time_s: 0.05,
    duration_s: 0.05,
    step_count: 400,
    frame_count: 12,
    frames_cached: 12,
    current_dt_s: 0.000125,
    bisections: 1,
    eroded_elements: 4,
    detached_fragments: 2,
    max_von_mises_mpa: 610,
    elapsed_ms: 9200,
  });

  assert.equal(completed.active, false);
  assert.equal(completed.cancellable, false);
  assert.equal(completed.estimatedRemainingMs, null);
  assert.equal(completed.progressPct, 100);
});

test('IncrementalRustSimAssembler keeps stable cache ID, pads late fragments across earlier frames, and upgrades shard geometry in-place', () => {
  const part: CadPart = {
    id: 'beam_1',
    name: 'Test Beam',
    materialId: 'steel_1045',
    dag: [],
    mesh: createBoxMesh(0.05, 0.02, 0.02),
    visible: true,
    isFixed: false,
    position: [0, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
  };

  const assembler = new IncrementalRustSimAssembler(
    { simulation_id: 'sim_stream_1', duration_s: 0.1 },
    [part],
    DEFAULT_MATERIALS,
    { ...DEFAULT_DESTRUCTION_CONFIG, durationSec: 0.1, fps: 30 }
  );

  // Frame 0 arrives with no shards yet
  assembler.appendFrame({
    simulation_id: 'sim_stream_1',
    frame_index: 0,
    total_frames: 4,
    time_s: 0,
    parts: [
      {
        part_id: 'beam_1',
        max_von_mises_mpa: 45,
        max_damage: 0,
        active_element_count: 12,
        eroded_element_count: 0,
      },
    ],
    energy_balance: { kinetic_j: 1.2, elastic_j: 0.1, plastic_j: 0, fracture_j: 0 },
  });

  const c0 = assembler.buildCache();
  assert.equal(c0.id, 'rust_sim_stream_1');
  assert.equal(c0.totalFrames, 0);
  assert.equal(c0.frames.length, 1);
  assert.equal(c0.shards.length, 0);

  // Frame 1 discovers a newly detached fragment without its detailed mesh yet
  const f1Res = assembler.appendFrame({
    simulation_id: 'sim_stream_1',
    frame_index: 1,
    total_frames: 4,
    time_s: 0.025,
    parts: [
      {
        part_id: 'beam_1',
        max_von_mises_mpa: 410,
        max_damage: 0.48,
        active_element_count: 10,
        eroded_element_count: 2,
      },
    ],
    detached_fragments: [
      {
        fragment_id: 'frag_tip_1',
        source_part_id: 'beam_1',
        position_m: [0.03, 0.01, 0],
        orientation_quat: [0, 0, 0, 1],
        bounding_radius_m: 0.004,
        mass_kg: 0.008,
      },
    ],
    energy_balance: { kinetic_j: 0.9, elastic_j: 0.35, plastic_j: 0.12, fracture_j: 0.05 },
  });

  assert.equal(f1Res.shardsAdded, true);
  const c1 = assembler.buildCache();
  assert.equal(c1.id, c0.id);
  assert.equal(c1.totalFrames, 1);
  assert.equal(c1.frames.length, 2);
  assert.equal(c1.shards.length, 1);
  // Frame 0 typed arrays must be padded to length 1 and remain inactive
  assert.equal(c1.frames[0].shardActive.length, 1);
  assert.equal(c1.frames[0].shardActive[0], 0);
  assert.equal(c1.frames[0].shardPos.length, 3);
  assert.equal(c1.frames[1].shardActive[0], 1);
  assert.ok(Math.abs(c1.frames[1].shardPos[0] - 0.03) < 1e-6);
  const fallbackIdxCount = c1.shards[0].indices.length;

  // Later fragment mesh download upgrades the box proxy to the real FEM surface mesh in-place
  assembler.setFragmentMeshes([
    {
      fragment_id: 'frag_tip_1',
      source_part_id: 'beam_1',
      vertices_m: [
        0, 0, 0, 0.01, 0, 0, 0, 0.01, 0,
        0, 0, 0.01, 0.01, 0, 0.01,
      ],
      indices: [
        0, 1, 2, 0, 1, 3, 1, 4, 3, 0, 2, 3, 2, 4, 3, 0, 1, 4,
        0, 2, 4, 1, 2, 4, 0, 3, 4, 1, 3, 4, 2, 3, 4, 0, 1, 2, 0, 1, 3,
      ],
    },
  ]);

  const c2 = assembler.buildCache();
  assert.equal(c2.shards.length, 1);
  assert.ok(c2.shards[0].indices.length >= fallbackIdxCount);
  assert.equal(c2.maxStressOverallMPa, 410);
});

test('bakeDestructionCacheAsync streams partial snapshots and supports AbortController cancellation', async () => {
  const gearA = makeTestGearPart('gear_a', -0.02);
  const gearB = makeTestGearPart('gear_b', 0.02);
  const joints: KinematicJoint[] = [
    {
      id: 'j1',
      name: 'Drive A',
      type: 'revolute',
      partA: 'world',
      partB: 'gear_a',
      anchor: [-0.02, 0, 0],
      axis: [0, 0, 1],
      motorSpeedRpm: 120,
      motorTorqueNm: 180,
      motorEnabled: true,
      currentAngleRad: 0,
    },
    {
      id: 'j2',
      name: 'Drive B',
      type: 'revolute',
      partA: 'world',
      partB: 'gear_b',
      anchor: [0.02, 0, 0],
      axis: [0, 0, 1],
      motorSpeedRpm: -120,
      motorTorqueNm: 180,
      motorEnabled: true,
      currentAngleRad: 0,
    },
  ];

  const cfg = {
    ...DEFAULT_DESTRUCTION_CONFIG,
    durationSec: 0.4,
    fps: 20,
    shardCountPerPart: 6,
  };

  const emittedCounts: number[] = [];
  const emittedPcts: number[] = [];
  const fullCache = await bakeDestructionCacheAsync(
    [gearA, gearB],
    joints,
    DEFAULT_MATERIALS,
    cfg,
    (pct) => emittedPcts.push(pct),
    {
      yieldEveryFrames: 3,
      onPartialCache: (partial, latestIdx) => {
        emittedCounts.push(partial.frames.length);
        assert.equal(latestIdx, partial.frames.length - 1);
        assert.equal(partial.totalFrames, partial.frames.length - 1);
      },
    }
  );

  assert.ok(emittedCounts.length >= 3);
  assert.equal(emittedCounts[0], 1);
  assert.equal(emittedCounts[emittedCounts.length - 1], fullCache.frames.length);
  assert.equal(emittedPcts[emittedPcts.length - 1], 100);

  // Cancellation mid-bake via AbortSignal
  const ac = new AbortController();
  let seen = 0;
  await assert.rejects(
    () =>
      bakeDestructionCacheAsync(
        [gearA, gearB],
        joints,
        DEFAULT_MATERIALS,
        cfg,
        undefined,
        {
          signal: ac.signal,
          yieldEveryFrames: 2,
          onPartialCache: () => {
            seen += 1;
            if (seen === 2) ac.abort();
          },
        }
      ),
    (err: unknown) => err instanceof Error && err.name === 'AbortError'
  );
  assert.equal(seen, 2);
});
