import type { ParsedStlGeometry } from '../types/cad';

/**
 * Parses a Binary STL ArrayBuffer into flat Float32Array position and normal buffers,
 * computing bounding box, center, and max dimension for camera framing.
 */
export function parseBinaryStl(buffer: ArrayBuffer): ParsedStlGeometry {
  if (buffer.byteLength < 84) {
    throw new Error(`Invalid binary STL buffer: expected at least 84 bytes, got ${buffer.byteLength}`);
  }

  const view = new DataView(buffer);
  const triangleCount = view.getUint32(80, true);
  const expectedLength = 84 + triangleCount * 50;

  if (buffer.byteLength < expectedLength) {
    throw new Error(
      `Truncated binary STL buffer: header declares ${triangleCount} triangles (${expectedLength} bytes), got ${buffer.byteLength}`,
    );
  }

  const positions = new Float32Array(triangleCount * 9);
  const normals = new Float32Array(triangleCount * 9);

  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;

  for (let i = 0; i < triangleCount; i++) {
    const offset = 84 + i * 50;
    let nx = view.getFloat32(offset, true);
    let ny = view.getFloat32(offset + 4, true);
    let nz = view.getFloat32(offset + 8, true);

    const v0x = view.getFloat32(offset + 12, true);
    const v0y = view.getFloat32(offset + 16, true);
    const v0z = view.getFloat32(offset + 20, true);

    const v1x = view.getFloat32(offset + 24, true);
    const v1y = view.getFloat32(offset + 28, true);
    const v1z = view.getFloat32(offset + 32, true);

    const v2x = view.getFloat32(offset + 36, true);
    const v2y = view.getFloat32(offset + 40, true);
    const v2z = view.getFloat32(offset + 44, true);

    // Recompute face normal if zero/degenerate in STL file
    const nLenSq = nx * nx + ny * ny + nz * nz;
    if (nLenSq < 1e-10) {
      const ux = v1x - v0x;
      const uy = v1y - v0y;
      const uz = v1z - v0z;
      const vx = v2x - v0x;
      const vy = v2y - v0y;
      const vz = v2z - v0z;
      const cx = uy * vz - uz * vy;
      const cy = uz * vx - ux * vz;
      const cz = ux * vy - uy * vx;
      const len = Math.hypot(cx, cy, cz);
      if (len > 1e-12) {
        nx = cx / len;
        ny = cy / len;
        nz = cz / len;
      } else {
        nx = 0;
        ny = 1;
        nz = 0;
      }
    }

    const base = i * 9;
    positions[base] = v0x;
    positions[base + 1] = v0y;
    positions[base + 2] = v0z;
    positions[base + 3] = v1x;
    positions[base + 4] = v1y;
    positions[base + 5] = v1z;
    positions[base + 6] = v2x;
    positions[base + 7] = v2y;
    positions[base + 8] = v2z;

    normals[base] = nx;
    normals[base + 1] = ny;
    normals[base + 2] = nz;
    normals[base + 3] = nx;
    normals[base + 4] = ny;
    normals[base + 5] = nz;
    normals[base + 6] = nx;
    normals[base + 7] = ny;
    normals[base + 8] = nz;

    minX = Math.min(minX, v0x, v1x, v2x);
    minY = Math.min(minY, v0y, v1y, v2y);
    minZ = Math.min(minZ, v0z, v1z, v2z);
    maxX = Math.max(maxX, v0x, v1x, v2x);
    maxY = Math.max(maxY, v0y, v1y, v2y);
    maxZ = Math.max(maxZ, v0z, v1z, v2z);
  }

  if (triangleCount === 0) {
    return {
      positions,
      normals,
      triangleCount: 0,
      boundsMin: [0, 0, 0],
      boundsMax: [0, 0, 0],
      center: [0, 0, 0],
      maxDimension: 0.1,
    };
  }

  const dx = maxX - minX;
  const dy = maxY - minY;
  const dz = maxZ - minZ;

  return {
    positions,
    normals,
    triangleCount,
    boundsMin: [minX, minY, minZ],
    boundsMax: [maxX, maxY, maxZ],
    center: [(minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5],
    maxDimension: Math.max(dx, dy, dz, 1e-4),
  };
}

/**
 * Maps an MCP-CAD engineering material identifier to PBR appearance properties.
 */
export function getMaterialPbrStyle(materialName: string): {
  colorHex: number;
  metalness: number;
  roughness: number;
  label: string;
} {
  const key = materialName.toLowerCase();
  if (key.includes('aluminum')) {
    return { colorHex: 0xd8dee9, metalness: 0.82, roughness: 0.24, label: 'Al 6061-T6' };
  }
  if (key.includes('titanium')) {
    return { colorHex: 0x94a3b8, metalness: 0.88, roughness: 0.28, label: 'Ti-6Al-4V' };
  }
  if (key.includes('brass')) {
    return { colorHex: 0xeab308, metalness: 0.85, roughness: 0.22, label: 'Brass C360' };
  }
  if (key.includes('4140')) {
    return { colorHex: 0x64748b, metalness: 0.85, roughness: 0.3, label: 'Steel 4140' };
  }
  if (key.includes('steel')) {
    return { colorHex: 0x9ca3af, metalness: 0.8, roughness: 0.28, label: 'AISI 1020 Steel' };
  }
  if (key.includes('pla')) {
    return { colorHex: 0x38bdf8, metalness: 0.08, roughness: 0.55, label: 'PLA Filament' };
  }
  if (key.includes('petg')) {
    return { colorHex: 0x10b981, metalness: 0.1, roughness: 0.38, label: 'PETG Filament' };
  }
  if (key.includes('nylon')) {
    return { colorHex: 0xe2e8f0, metalness: 0.05, roughness: 0.62, label: 'Nylon PA12' };
  }
  return { colorHex: 0x94a3b8, metalness: 0.75, roughness: 0.3, label: materialName };
}

/**
 * Extracts structured JSON and optional base64 PNG from an MCP `tools/call` JSON-RPC response.
 */
export function unwrapMcpToolResponse(rpcResponse: Record<string, unknown>): {
  isError: boolean;
  data: Record<string, unknown>;
  rawText: string;
  imageBase64?: string;
} {
  if (rpcResponse.error && typeof rpcResponse.error === 'object') {
    const errObj = rpcResponse.error as Record<string, unknown>;
    const msg = String(errObj.message || 'JSON-RPC Error');
    return {
      isError: true,
      data: { error: msg },
      rawText: msg,
    };
  }

  const result = (rpcResponse.result ?? {}) as Record<string, unknown>;
  const isError = Boolean(result.isError);
  const content = Array.isArray(result.content) ? result.content : [];

  let rawText = '';
  let imageBase64: string | undefined;

  for (const item of content) {
    if (item && typeof item === 'object') {
      const entry = item as Record<string, unknown>;
      if (entry.type === 'text' && typeof entry.text === 'string') {
        rawText = entry.text;
      } else if (entry.type === 'image' && typeof entry.data === 'string') {
        imageBase64 = entry.data;
      }
    }
  }

  let data: Record<string, unknown> = {};
  if (rawText) {
    try {
      const parsed = JSON.parse(rawText);
      if (parsed && typeof parsed === 'object') {
        data = parsed as Record<string, unknown>;
        if (!imageBase64 && typeof data.image_base64 === 'string') {
          imageBase64 = data.image_base64;
        }
      }
    } catch {
      data = { message: rawText };
    }
  }

  return {
    isError,
    data,
    rawText,
    imageBase64,
  };
}
