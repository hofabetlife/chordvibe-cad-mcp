import test from 'node:test';
import assert from 'node:assert/strict';
import http from 'node:http';
import type { AddressInfo } from 'node:net';
import type {
  CadPart,
  DestructionSimConfig,
  KinematicJoint,
  MaterialSpec,
  MeshGeometryData,
} from '../../types/cad';
import { DEFAULT_MATERIALS } from '../materials';
import {
  buildRustSceneSyncPlan,
  resolveRustEndpoint,
  resolveRustMaterialForPart,
} from '../rust_scene_sync';
import {
  bakeNativeCoupledSimulation,
  probeWindowsMcpServer,
  RustBakeCancelledError,
} from '../native_bridge';

function makeDummyMesh(size = 0.04): MeshGeometryData {
  const h = size / 2;
  return {
    vertices: new Float32Array([-h, -h, -h, h, -h, -h, 0, h, h]),
    normals: new Float32Array([0, 0, 1, 0, 0, 1, 0, 0, 1]),
    indices: new Uint32Array([0, 1, 2]),
    vertexCount: 3,
    triangleCount: 1,
    boundingBox: { min: [-h, -h, -h], max: [h, h, h] },
    volume: size ** 3,
    surfaceArea: 6 * size * size,
    mass: 0.25,
    centerOfMass: [0, 0, 0],
    inertiaTensor: [1e-4, 1e-4, 1e-4],
    isWatertight: true,
    isManifold: true,
    isOutwardOriented: true,
    eulerCharacteristic: 2,
    eulerGenus: 0,
    weldedVertexCount: 3,
    uniqueEdgeCount: 3,
    boundaryEdgesCount: 0,
    nonManifoldEdgesCount: 0,
    inconsistentEdgesCount: 0,
    degenerateTrianglesCount: 0,
    zeroNormalsCount: 0,
    detectedPlanarFaces: [],
  };
}

function makeDualGearClashScene(): {
  parts: CadPart[];
  joints: KinematicJoint[];
  config: DestructionSimConfig;
} {
  // Mirrors Web Studio `gears_clash_destruction` preset (Z1=18, Z2=22, module=2.2mm -> pitchSum=44.0mm, outerSum=48.4mm, initial dist=50.8mm)
  const gearA: CadPart = {
    id: 'clash_gear_left',
    name: 'Левая шестерня Z=18',
    materialId: 'steel_1045',
    visible: true,
    isFixed: false,
    position: [-0.0254, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
    dag: [
      {
        id: 'n_gear_a',
        name: 'Spur Gear Z=18',
        type: 'spur_gear',
        enabled: true,
        params: {
          teeth: 18,
          module: 2.2,
          face_width: 0.016,
          pressure_angle_deg: 20,
          bore_diameter: 0.012,
        },
      },
    ],
    mesh: makeDummyMesh(0.052),
  };

  const gearB: CadPart = {
    id: 'clash_gear_right',
    name: 'Правая шестерня Z=22',
    materialId: 'alu_6061',
    visible: true,
    isFixed: false,
    position: [0.0254, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
    dag: [
      {
        id: 'n_gear_b',
        name: 'Spur Gear Z=22',
        type: 'spur_gear',
        enabled: true,
        params: {
          teeth: 22,
          module: 2.2,
          face_width: 0.012,
          pressure_angle_deg: 20,
          bore_diameter: 0.014,
        },
      },
    ],
    mesh: makeDummyMesh(0.052),
  };

  const bench: CadPart = {
    id: 'test_rig_bench',
    name: 'Станина стенда',
    materialId: 'steel_1020',
    visible: true,
    isFixed: true,
    position: [0, -0.04, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
    dag: [
      {
        id: 'n_bench',
        name: 'Rig Bench',
        type: 'box',
        enabled: true,
        params: { width: 0.16, height: 0.012, depth: 0.08 },
      },
    ],
    mesh: makeDummyMesh(0.1),
  };

  const joints: KinematicJoint[] = [
    {
      id: 'rev_left',
      name: 'Вал левой шестерни',
      type: 'revolute',
      partA: 'test_rig_bench',
      partB: 'clash_gear_left',
      anchor: [-0.0254, 0, 0],
      axis: [0, 0, 1],
      motorEnabled: true,
      motorSpeedRpm: 180,
      motorTorqueNm: 240,
      currentAngleRad: 0,
    },
    {
      id: 'rev_right',
      name: 'Вал правой шестерни',
      type: 'revolute',
      partA: 'test_rig_bench',
      partB: 'clash_gear_right',
      anchor: [0.0254, 0, 0],
      axis: [0, 0, 1],
      motorEnabled: true,
      motorSpeedRpm: 180,
      motorTorqueNm: 240,
      currentAngleRad: 0,
    },
  ];

  const config: DestructionSimConfig = {
    scenarioMode: 'counter_clash',
    durationSec: 2.5,
    fps: 60,
    motorRpmLeft: 210,
    motorRpmRight: 210,
    impactTorqueNm: 280,
    brittleness: 0.65,
    shardCountPerPart: 24,
    gravityY: -9.81,
    restitution: 0.32,
    seed: 1337,
  };

  return { parts: [gearA, gearB, bench], joints, config };
}

test('resolveRustEndpoint normalizes proxy/http URLs and rejects unsafe schemes', () => {
  for (const proxyVal of ['', '  ', 'auto', 'same-origin', '/api']) {
    const r = resolveRustEndpoint(proxyVal);
    assert.equal(r.ok, true);
    if (r.ok) {
      assert.equal(r.isSameOriginProxy, true);
      assert.equal(r.mcpUrl, '/api/mcp');
      assert.equal(r.coupledSimBaseUrl, '/api/coupled-sim');
      assert.equal(r.statusUrl, '/api/rust/status');
    }
  }

  const direct = resolveRustEndpoint('http://127.0.0.1:8080///');
  assert.equal(direct.ok, true);
  if (direct.ok) {
    assert.equal(direct.isSameOriginProxy, false);
    assert.equal(direct.endpointUrl, 'http://127.0.0.1:8080');
    assert.equal(direct.mcpUrl, 'http://127.0.0.1:8080/message');
    assert.equal(direct.coupledSimBaseUrl, 'http://127.0.0.1:8080/api/coupled-sim');
  }

  for (const bad of ['javascript:alert(1)', 'file:///etc/passwd', 'not-a-url']) {
    const r = resolveRustEndpoint(bad);
    assert.equal(r.ok, false);
  }
});

test('resolveRustMaterialForPart maps built-ins canonically and defines custom materials', () => {
  const alu = resolveRustMaterialForPart('alu_6061', DEFAULT_MATERIALS);
  assert.equal(alu.rustMaterialId, 'aluminum_6061_t6');
  assert.equal(alu.defineMaterialOp, undefined);

  const steel = resolveRustMaterialForPart('steel_1045', DEFAULT_MATERIALS);
  assert.equal(steel.rustMaterialId, 'steel_carbon_aisi1045');
  assert.equal(steel.defineMaterialOp, undefined);

  const customMat: MaterialSpec = {
    id: 'inconel_718',
    name: 'Inconel 718',
    category: 'Metal',
    density: 8190,
    youngsModulus: 205e9,
    poissonsRatio: 0.29,
    yieldStrength: 1035e6,
    ultimateStrengthPa: 1240e6,
    hardeningModulusPa: 3.2e9,
    fractureStrain: 0.14,
    fractureEnergyJM2: 45000,
    frictionCoeff: 0.22,
    failureMode: 'ductile',
    colorHex: '#9ca3af',
    metalness: 0.85,
    roughness: 0.3,
  };

  const custom = resolveRustMaterialForPart('inconel_718', [
    ...DEFAULT_MATERIALS,
    customMat,
  ]);
  assert.equal(custom.rustMaterialId, 'inconel_718');
  assert.ok(custom.defineMaterialOp);
  assert.equal(custom.defineMaterialOp.tool, 'define_material');
  assert.equal(custom.defineMaterialOp.args.young_modulus_gpa, 205);
  assert.equal(custom.defineMaterialOp.args.failure_mode, 'ductile');
});

test('buildRustSceneSyncPlan converts gear module mm->m, closes air gap, maps ground joints & motors', () => {
  const { parts, joints, config } = makeDualGearClashScene();
  const plan = buildRustSceneSyncPlan(
    parts,
    joints,
    DEFAULT_MATERIALS,
    config,
    'high'
  );

  // First op must set units to meters
  assert.equal(plan.operations[0].tool, 'set_units');
  assert.equal(plan.operations[0].args.length_unit, 'm');

  // Bench excluded from active destructible parts
  assert.deepEqual(plan.activePartIds, ['clash_gear_left', 'clash_gear_right']);

  // Spur gear module converted from 2.2 mm -> 0.0022 m
  const gearPrimitives = plan.operations.filter(
    (op) =>
      op.tool === 'add_feature_primitive' &&
      op.args.primitive_type === 'spur_gear'
  );
  assert.equal(gearPrimitives.length, 2);
  const firstParams = gearPrimitives[0].args.params as Record<string, unknown>;
  assert.equal(firstParams.module, 0.0022);
  assert.equal(firstParams.teeth, 18);

  // Air-gap shafts (50.8 mm > outerSum 48.4 mm) closed to pitchSum + 0.15 mm = 44.15 mm
  const transforms = plan.operations.filter((op) => op.tool === 'transform_part');
  assert.equal(transforms.length, 2);
  const tLeft = transforms[0].args.translation as [number, number, number];
  const tRight = transforms[1].args.translation as [number, number, number];
  const newDist = Math.hypot(tRight[0] - tLeft[0], tRight[1] - tLeft[1]);
  assert.ok(
    Math.abs(newDist - 0.04415) < 1e-5,
    `Expected shaft spacing ~0.04415m, got ${newDist}`
  );

  // Joints mapped bench -> "ground", plus automatic gear metadata coupling
  const createJoints = plan.operations.filter((op) => op.tool === 'create_joint');
  assert.equal(createJoints.length, 3);
  assert.equal(createJoints[0].args.part_a, 'ground');
  assert.equal(createJoints[1].args.part_a, 'ground');
  assert.equal(createJoints[2].args.joint_type, 'gear');

  // Both revolute joints receive motors driven by config RPM & torque
  const motors = plan.operations.filter((op) => op.tool === 'set_joint_motor');
  assert.equal(motors.length, 2);
  assert.equal(motors[0].args.max_force, 280);
  assert.ok(
    Math.abs(Number(motors[0].args.target_velocity) - (210 * 2 * Math.PI) / 60) < 1e-4
  );

  // Bake payload reflects high accuracy and full coupled FEM flags
  assert.equal(plan.bakePayload.accuracy_level, 'high');
  assert.equal(plan.bakePayload.enable_plasticity, true);
  assert.equal(plan.bakePayload.enable_fracture, true);
  assert.equal(plan.bakePayload.enable_contact, true);
  assert.equal(plan.bakePayload.async_mode, true);
  assert.equal(plan.bakePayload.length_unit, 'm');
});

test('buildRustSceneSyncPlan adds physical fixed_faces + ramp loads for single-part impact_crush', () => {
  const singlePart: CadPart = {
    id: 'crush_block',
    name: 'Тестовый блок',
    materialId: 'borosilicate_glass',
    visible: true,
    isFixed: false,
    position: [0, 0, 0],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
    dag: [
      {
        id: 'n_box',
        name: 'Box',
        type: 'box',
        enabled: true,
        params: { width: 0.04, height: 0.04, depth: 0.04 },
      },
    ],
    mesh: makeDummyMesh(0.04),
  };

  const config: DestructionSimConfig = {
    scenarioMode: 'impact_crush',
    durationSec: 1.5,
    fps: 60,
    motorRpmLeft: 0,
    motorRpmRight: 0,
    impactTorqueNm: 300,
    brittleness: 0.9,
    shardCountPerPart: 24,
    gravityY: -9.81,
    restitution: 0.25,
    seed: 42,
  };

  const plan = buildRustSceneSyncPlan(
    [singlePart],
    [],
    DEFAULT_MATERIALS,
    config,
    'ultra'
  );

  assert.equal(plan.bakePayload.accuracy_level, 'ultra');
  assert.equal(plan.bakePayload.enable_contact, false);
  assert.ok(Array.isArray(plan.bakePayload.fixed_faces));
  assert.ok(Array.isArray(plan.bakePayload.loads));
  assert.ok(plan.bakePayload.force_curve);
});

test('end-to-end Rust HTTP bridge round-trip (probe -> sync -> bake -> poll -> fragments/frames -> cancel)', async () => {
  const mcpCalls: { name: string; args: Record<string, unknown> }[] = [];
  const bakeRequests: Record<string, unknown>[] = [];
  const cancelledJobIds: string[] = [];
  let pollCount = 0;
  let holdJobPending = false;

  const server = http.createServer((req, res) => {
    const url = new URL(req.url || '/', 'http://127.0.0.1');
    const chunks: Buffer[] = [];
    req.on('data', (c) => chunks.push(c));
    req.on('end', () => {
      const bodyStr = Buffer.concat(chunks).toString('utf8');
      const body = bodyStr ? JSON.parse(bodyStr) : {};

      // JSON-RPC MCP endpoint
      if (req.method === 'POST' && url.pathname === '/message') {
        if (body.method === 'initialize') {
          res.writeHead(200, { 'Content-Type': 'application/json' });
          res.end(
            JSON.stringify({
              jsonrpc: '2.0',
              id: body.id,
              result: {
                protocolVersion: '2024-11-05',
                serverInfo: { name: 'mcp-cad-server-mock', version: '0.5.0-test' },
              },
            })
          );
          return;
        }

        if (body.method === 'tools/call') {
          const toolName = String(body.params?.name || '');
          const args = (body.params?.arguments || {}) as Record<string, unknown>;
          mcpCalls.push({ name: toolName, args });

          let payload: Record<string, unknown> = { ok: true, tool: toolName };
          if (toolName === 'list_parts') {
            payload = { parts: [{ id: 'stale_old_part' }] };
          }
          res.writeHead(200, { 'Content-Type': 'application/json' });
          res.end(
            JSON.stringify({
              jsonrpc: '2.0',
              id: body.id,
              result: {
                content: [{ type: 'text', text: JSON.stringify(payload) }],
                isError: false,
              },
            })
          );
          return;
        }
      }

      // POST /api/coupled-sim/bake
      if (req.method === 'POST' && url.pathname === '/api/coupled-sim/bake') {
        bakeRequests.push(body);
        pollCount = 0;
        const jobId = holdJobPending ? 'job_cancel_me' : 'job_test_001';
        res.writeHead(202, { 'Content-Type': 'application/json' });
        res.end(
          JSON.stringify({
            job_id: jobId,
            status: 'running',
            duration_s: body.duration_s,
          })
        );
        return;
      }

      // GET /api/coupled-sim/status/:jobId
      if (req.method === 'GET' && url.pathname.startsWith('/api/coupled-sim/status/')) {
        const jobId = url.pathname.split('/').pop() || '';
        pollCount += 1;
        if (holdJobPending || pollCount < 2) {
          res.writeHead(200, { 'Content-Type': 'application/json' });
          res.end(
            JSON.stringify({
              job: {
                job_id: jobId,
                state: 'running',
                progress_pct: 45,
              },
            })
          );
          return;
        }
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(
          JSON.stringify({
            job: {
              job_id: jobId,
              state: 'completed',
              simulation_id: 'sim_rust_001',
              frame_count: 2,
              elapsed_ms: 142,
              events: [
                {
                  event_type: 'tooth_shear',
                  frame_index: 1,
                  time_s: 0.05,
                  part_id: 'clash_gear_left',
                  description: 'Отрыв зуба при σ_vM > σ_u',
                },
              ],
            },
          })
        );
        return;
      }

      // POST /api/coupled-sim/cancel/:jobId
      if (req.method === 'POST' && url.pathname.startsWith('/api/coupled-sim/cancel/')) {
        const jobId = url.pathname.split('/').pop() || '';
        cancelledJobIds.push(jobId);
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ cancelled: true, job_id: jobId }));
        return;
      }

      // GET /api/coupled-sim/fragments/:simId (P5-T15 schema)
      if (req.method === 'GET' && url.pathname === '/api/coupled-sim/fragments/sim_rust_001') {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(
          JSON.stringify({
            simulation_id: 'sim_rust_001',
            fragments: [
              {
                fragment_id: 'frag_1',
                source_part_id: 'clash_gear_left',
                vertices_m: [0, 0, 0, 0.005, 0, 0, 0, 0.005, 0],
                indices: [0, 1, 2],
                bounding_radius_m: 0.004,
              },
            ],
          })
        );
        return;
      }

      // GET /api/coupled-sim/frames/:simId/:frameIdx (P5-T15 schema)
      if (req.method === 'GET' && url.pathname.startsWith('/api/coupled-sim/frames/sim_rust_001/')) {
        const idx = Number(url.pathname.split('/').pop() || '0');
        assert.equal(url.searchParams.get('include_mesh_fields'), 'true');
        assert.equal(url.searchParams.get('include_fragment_meshes'), 'false');
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(
          JSON.stringify({
            simulation_id: 'sim_rust_001',
            frame_index: idx,
            total_frames: 2,
            time_s: idx * 0.05,
            parts: [
              {
                part_id: 'clash_gear_left',
                node_count: 3,
                surface_reference_positions_m: [-0.01, 0, 0, 0.01, 0, 0, 0, 0.01, 0],
                surface_positions_m:
                  idx === 0
                    ? [-0.01, 0, 0, 0.01, 0, 0, 0, 0.01, 0]
                    : [-0.01, 0, 0, 0.0104, 0, 0, 0, 0.0106, 0],
                surface_indices: [0, 1, 2],
                surface_node_indices: [0, 1, 2],
                nodal_von_mises_mpa: idx === 0 ? [40, 80, 120] : [120, 420, 695],
                nodal_equiv_plastic_strain: idx === 0 ? [0, 0, 0] : [0, 0.01, 0.042],
                nodal_damage: idx === 0 ? [0, 0, 0] : [0, 0.3, 0.92],
                max_von_mises_mpa: idx === 0 ? 120 : 695,
                max_equiv_plastic_strain: idx === 0 ? 0.0 : 0.042,
                max_damage: idx === 0 ? 0.0 : 0.92,
                max_displacement_mm: idx === 0 ? 0.02 : 0.68,
                eroded_element_count: idx === 0 ? 0 : 12,
              },
            ],
            detached_fragments:
              idx === 1
                ? [
                    {
                      fragment_id: 'frag_1',
                      source_part_id: 'clash_gear_left',
                      position_m: [0.002, -0.004, 0.001],
                      orientation_quat: [0, 0, 0, 1],
                      linear_velocity_m_s: [0.8, -1.2, 0.2],
                      angular_velocity_rad_s: [12, 4, 20],
                      mass_kg: 0.012,
                    },
                  ]
                : [],
            energy_balance: {
              kinetic_j: 4.5,
              elastic_j: 1.8,
              plastic_j: idx === 0 ? 0 : 2.1,
              fracture_j: idx === 0 ? 0 : 0.9,
            },
          })
        );
        return;
      }

      res.writeHead(404);
      res.end();
    });
  });

  await new Promise<void>((resolve) => server.listen(0, '127.0.0.1', resolve));
  const { port } = server.address() as AddressInfo;
  const mockUrl = `http://127.0.0.1:${port}`;

  try {
    // 1. Probe verifies live Rust server connectivity
    const status = await probeWindowsMcpServer(mockUrl, true);
    assert.equal(status.connectedToWindowsExe, true);
    assert.equal(status.mode, 'windows_exe_connected');
    assert.equal(status.activeToolsCount, 43);
    assert.equal(status.serverVersion, '0.5.0-test');

    // 2. Full bake syncs scene, deletes stale part, runs HHT-α bake, fetches fragments & frames
    const { parts, joints, config } = makeDualGearClashScene();
    const receivedJobIds: string[] = [];
    const cache = await bakeNativeCoupledSimulation(
      mockUrl,
      parts,
      DEFAULT_MATERIALS,
      config,
      undefined,
      {
        joints,
        accuracyLevel: 'high',
        onJobId: (id) => receivedJobIds.push(id),
      }
    );

    assert.equal(cache.solverBackend, 'rust_fem_f64');
    assert.equal(cache.solverLabel, 'Rust FEM f64');
    assert.equal(cache.frames.length, 2);
    assert.deepEqual(receivedJobIds, ['job_test_001']);

    // Verify stale part was deleted before replaying scene
    assert.equal(mcpCalls[0].name, 'list_parts');
    assert.equal(mcpCalls[1].name, 'delete_part');
    assert.equal(mcpCalls[1].args.part_id, 'stale_old_part');
    assert.equal(mcpCalls[2].name, 'set_units');

    // Verify bake payload accuracy
    assert.equal(bakeRequests.length, 1);
    assert.equal(bakeRequests[0].accuracy_level, 'high');

    // Verify FEM surface mesh & detached fragment arrived in DestructionCache
    const f1 = cache.frames[1];
    assert.ok(f1.partStates['clash_gear_left']?.femSurfaceMesh);
    assert.equal(f1.partStates['clash_gear_left'].femSurfaceMesh?.vertexCount, 3);
    assert.equal(f1.metrics.peakStressMPa, 695);
    assert.equal(cache.shards.length, 1);
    assert.equal(cache.shards[0].id, 'frag_1');
    assert.equal(cache.shards[0].vertices.length, 9);
    assert.equal(f1.shardActive[0], 1);

    // 3. Verify cancellation aborts cleanly and sends POST /api/coupled-sim/cancel/:job_id
    holdJobPending = true;
    const ac = new AbortController();
    await assert.rejects(
      () =>
        bakeNativeCoupledSimulation(
          mockUrl,
          parts,
          DEFAULT_MATERIALS,
          config,
          undefined,
          {
            joints,
            accuracyLevel: 'standard',
            signal: ac.signal,
            onJobId: () => {
              setTimeout(() => ac.abort(), 30);
            },
          }
        ),
      (err: unknown) => err instanceof RustBakeCancelledError
    );

    // Allow fire-and-forget cancel HTTP POST to reach mock server
    await new Promise((r) => setTimeout(r, 80));
    assert.deepEqual(cancelledJobIds, ['job_cancel_me']);
  } finally {
    await new Promise<void>((resolve) => server.close(() => resolve()));
  }
});
