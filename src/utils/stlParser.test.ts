import test from 'node:test';
import assert from 'node:assert/strict';
import { parseBinaryStl, getMaterialPbrStyle, unwrapMcpToolResponse } from './stlParser';

test('parseBinaryStl parses a 1-triangle binary STL buffer and computes bounds', () => {
  // 80-byte header + 4-byte count (1) + 50-byte triangle = 134 bytes
  const buffer = new ArrayBuffer(134);
  const view = new DataView(buffer);
  view.setUint32(80, 1, true);

  // Normal = [0, 0, 0] -> should auto-compute [0, 0, 1] for CCW triangle in XY plane
  view.setFloat32(84, 0, true);
  view.setFloat32(88, 0, true);
  view.setFloat32(92, 0, true);

  // v0 = [0, 0, 0]
  view.setFloat32(96, 0, true);
  view.setFloat32(100, 0, true);
  view.setFloat32(104, 0, true);

  // v1 = [2, 0, 0]
  view.setFloat32(108, 2, true);
  view.setFloat32(112, 0, true);
  view.setFloat32(116, 0, true);

  // v2 = [0, 4, 0]
  view.setFloat32(120, 0, true);
  view.setFloat32(124, 4, true);
  view.setFloat32(128, 0, true);

  const parsed = parseBinaryStl(buffer);
  assert.equal(parsed.triangleCount, 1);
  assert.deepEqual(parsed.boundsMin, [0, 0, 0]);
  assert.deepEqual(parsed.boundsMax, [2, 4, 0]);
  assert.deepEqual(parsed.center, [1, 2, 0]);
  assert.equal(parsed.maxDimension, 4);
  assert.equal(parsed.normals[2], 1);
});

test('getMaterialPbrStyle returns distinct PBR properties for engineering materials', () => {
  const al = getMaterialPbrStyle('aluminum_6061_t6');
  const brass = getMaterialPbrStyle('brass_c360');
  const pla = getMaterialPbrStyle('pla_filament_3dprint');

  assert.equal(al.label, 'Al 6061-T6');
  assert.ok(al.metalness > 0.7);
  assert.equal(brass.label, 'Brass C360');
  assert.ok(pla.metalness < 0.2);
});

test('unwrapMcpToolResponse extracts JSON payload and image base64 from MCP response', () => {
  const sampleRpc = {
    jsonrpc: '2.0',
    id: 1,
    result: {
      content: [
        {
          type: 'text',
          text: JSON.stringify({ part_id: 'part-123', status: 'created' }),
        },
        {
          type: 'image',
          data: 'iVBORw0KGgoAAAANSUhEUg==',
          mimeType: 'image/png',
        },
      ],
    },
  };

  const unwrapped = unwrapMcpToolResponse(sampleRpc);
  assert.equal(unwrapped.isError, false);
  assert.equal(unwrapped.data.part_id, 'part-123');
  assert.equal(unwrapped.imageBase64, 'iVBORw0KGgoAAAANSUhEUg==');
});
