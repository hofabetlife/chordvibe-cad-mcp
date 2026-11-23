import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { LiveSyncStore, isSafeLivePartId } from '../live_sync_store';

function makeMinimalBinaryStl(triCount = 1): Buffer {
  const buf = Buffer.alloc(84 + triCount * 50, 0);
  buf.write('MCP-CAD TEST STL', 0, 'ascii');
  buf.writeUInt32LE(triCount, 80);
  // Triangle 0 vertices: (0,0,0), (1,0,0), (0,1,0)
  buf.writeFloatLE(0, 84 + 12);
  buf.writeFloatLE(0, 84 + 16);
  buf.writeFloatLE(0, 84 + 20);
  buf.writeFloatLE(1, 84 + 24);
  buf.writeFloatLE(0, 84 + 28);
  buf.writeFloatLE(0, 84 + 32);
  buf.writeFloatLE(0, 84 + 36);
  buf.writeFloatLE(1, 84 + 40);
  buf.writeFloatLE(0, 84 + 44);
  return buf;
}

test('LiveSyncStore handles empty state, pushSnapshot, STL streaming, heartbeat, and clear', () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'mcp-live-store-test-'));
  const tempLive = path.join(root, 'os_temp_live');
  const legacyStl = path.join(root, 'output', '_mcp_preview_live.stl');

  try {
    const store = new LiveSyncStore({
      workspaceRoot: root,
      tempLiveDir: tempLive,
      legacyStlPath: legacyStl,
    });

    const empty = store.readCurrentScene();
    assert.equal(empty.active, false);
    assert.equal(empty.sourceKind, 'none');
    assert.equal(empty.snapshot, null);

    let updateCount = 0;
    let clearCount = 0;
    store.on('update', () => {
      updateCount += 1;
    });
    store.on('clear', () => {
      clearCount += 1;
    });

    const stlBuf = makeMinimalBinaryStl(2);
    const pushed = store.pushSnapshot({
      source: 'test_agent',
      timestamp_ms: 1_700_000_000_000,
      last_tool: {
        tool_name: 'add_feature_primitive',
        status: 'success',
        summary: 'Created coil spring',
        timestamp_ms: 1_700_000_000_000,
        duration_ms: 8.5,
      },
      parts: [
        {
          id: 'spring_1',
          name: 'Coil Spring',
          triangle_count: 2,
          vertex_count: 6,
          stl_base64: stlBuf.toString('base64'),
          dag: [
            {
              id: 'n1',
              name: 'Helix',
              op_type: 'helix',
              enabled: true,
              params: { helix_radius: 0.0175 },
            },
          ],
        },
      ],
    });

    assert.equal(pushed.parts.length, 1);
    assert.equal(updateCount, 1);

    const active = store.readCurrentScene();
    assert.equal(active.active, true);
    assert.equal(active.sourceKind, 'manifest');
    assert.equal(active.snapshot?.parts[0].id, 'spring_1');

    const loadedStl = store.readPartStlBuffer('spring_1');
    assert.ok(loadedStl);
    assert.equal(loadedStl.byteLength, stlBuf.byteLength);

    // Path traversal rejection
    assert.equal(isSafeLivePartId('../etc/passwd'), false);
    assert.equal(isSafeLivePartId('spring..1'), false);
    assert.equal(store.readPartStlBuffer('../scene'), null);

    // Heartbeat file written to both workspace and temp dirs
    store.writeStudioHeartbeat(2);
    const hbPath = path.join(store.getWorkspaceLiveDir(), 'studio_heartbeat.json');
    assert.equal(fs.existsSync(hbPath), true);
    const hb = JSON.parse(fs.readFileSync(hbPath, 'utf8'));
    assert.equal(hb.connected_clients, 2);

    // Clear removes scene and emits clear event
    store.clearSnapshot(true);
    assert.equal(clearCount, 1);
    assert.equal(store.readCurrentScene().active, false);
  } finally {
    fs.rmSync(root, { recursive: true, force: true });
  }
});

test('LiveSyncStore falls back to legacy output/_mcp_preview_live.stl and picks freshest source across directories', () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'mcp-live-legacy-test-'));
  const tempLive = path.join(root, 'os_temp_live');
  const legacyStl = path.join(root, 'output', '_mcp_preview_live.stl');

  try {
    fs.mkdirSync(path.dirname(legacyStl), { recursive: true });
    const legacyBuf = makeMinimalBinaryStl(3);
    fs.writeFileSync(legacyStl, legacyBuf);

    const store = new LiveSyncStore({
      workspaceRoot: root,
      tempLiveDir: tempLive,
      legacyStlPath: legacyStl,
    });

    const fromLegacy = store.readCurrentScene();
    assert.equal(fromLegacy.active, true);
    assert.equal(fromLegacy.sourceKind, 'legacy_stl');
    assert.equal(fromLegacy.snapshot?.parts[0].id, 'mcp_preview_live');
    assert.equal(fromLegacy.snapshot?.parts[0].triangle_count, 3);

    const legacyBytes = store.readPartStlBuffer('mcp_preview_live');
    assert.ok(legacyBytes);
    assert.equal(legacyBytes.byteLength, legacyBuf.byteLength);

    // Now write a fresher manifest into tempLiveDir ONLY (simulating Antigravity spawned from arbitrary CWD)
    fs.mkdirSync(path.join(tempLive, 'parts'), { recursive: true });
    const newerStl = makeMinimalBinaryStl(5);
    fs.writeFileSync(path.join(tempLive, 'parts', 'gear_from_temp.stl'), newerStl);
    const newerTs = Date.now() + 60_000;
    fs.writeFileSync(
      path.join(tempLive, 'scene.json'),
      JSON.stringify({
        schema_version: '1.0.0',
        revision: 7,
        timestamp_ms: newerTs,
        pid: 4242,
        source: 'stdio',
        length_unit: 'm',
        recent_tools: [],
        parts: [
          {
            id: 'gear_from_temp',
            safe_id: 'gear_from_temp',
            name: 'Gear From Temp',
            revision: 1,
            is_fixed: false,
            material: {
              name: 'steel_carbon_aisi1045',
              youngs_modulus_gpa: 205,
              poissons_ratio: 0.29,
              yield_strength_mpa: 350,
              density_kg_m3: 7850,
              failure_mode: 'ductile',
            },
            triangle_count: 5,
            vertex_count: 15,
            volume_m3: 0.001,
            mass_kg: 7.85,
            bounding_box_m: [
              [0, 0, 0],
              [0.1, 0.1, 0.1],
            ],
            stl_file: 'parts/gear_from_temp.stl',
            dag: [],
          },
        ],
        joints: [],
      })
    );

    const freshest = store.readCurrentScene();
    assert.equal(freshest.active, true);
    assert.equal(freshest.sourceKind, 'manifest');
    assert.equal(freshest.snapshot?.parts[0].id, 'gear_from_temp');

    const tempStlBytes = store.readPartStlBuffer('gear_from_temp');
    assert.ok(tempStlBytes);
    assert.equal(tempStlBytes.byteLength, newerStl.byteLength);
  } finally {
    fs.rmSync(root, { recursive: true, force: true });
  }
});
