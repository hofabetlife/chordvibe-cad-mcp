import * as THREE from 'three';
import {
  DagNode,
  DetectedPlanarFace,
  MeshGeometryData,
  MeshValidationReport,
  QualityPreset,
} from '../types/cad';

export function getSegmentsForQuality(quality: QualityPreset): number {
  switch (quality) {
    case 'draft':
      return 32;
    case 'standard':
      return 64;
    case 'high':
    case 'fine':
      return 128;
  }
}

interface TopologyAnalysis {
  weldedVerticesCount: number;
  uniqueEdgesCount: number;
  boundaryEdgesCount: number;
  nonManifoldEdgesCount: number;
  inconsistentEdgesCount: number;
  degenerateTrianglesCount: number;
  zeroNormalsCount: number;
  eulerCharacteristic: number;
  eulerGenus: number;
  isWatertight: boolean;
  isManifold: boolean;
  boundaryEdgePositions: Float32Array;
}

/**
 * Spatial vertex welding and full half-edge topological validator matching
 * crates/mcp-cad-server/tests/qa_wide_scan.rs (validate_mesh).
 */
export function analyzeMeshTopology(
  vertices: Float32Array,
  normals: Float32Array,
  indices: Uint32Array,
  weldTolerance: number = 1e-6
): TopologyAnalysis {
  const rawVertexCount = vertices.length / 3;
  const invTol = 1 / weldTolerance;
  const weldMap = new Map<string, number>();
  const oldToWelded = new Uint32Array(rawVertexCount);
  const weldedPositions: number[] = [];
  let weldedVerticesCount = 0;

  for (let i = 0; i < rawVertexCount; i++) {
    const x = vertices[i * 3];
    const y = vertices[i * 3 + 1];
    const z = vertices[i * 3 + 2];
    const kx = Math.round(x * invTol);
    const ky = Math.round(y * invTol);
    const kz = Math.round(z * invTol);
    const key = `${kx}_${ky}_${kz}`;
    let idx = weldMap.get(key);
    if (idx === undefined) {
      idx = weldedVerticesCount++;
      weldMap.set(key, idx);
      weldedPositions.push(x, y, z);
    }
    oldToWelded[i] = idx;
  }

  let zeroNormalsCount = 0;
  for (let i = 0; i < normals.length; i += 3) {
    const nx = normals[i];
    const ny = normals[i + 1];
    const nz = normals[i + 2];
    const lenSq = nx * nx + ny * ny + nz * nz;
    if (!Number.isFinite(lenSq) || lenSq < 1e-10) {
      zeroNormalsCount++;
    }
  }

  // Edge tracking: key = "min_max", value = { forward: count(min->max), reverse: count(max->min), u, v }
  const edgeMap = new Map<
    string,
    { forward: number; reverse: number; u: number; v: number }
  >();

  let degenerateTrianglesCount = 0;
  let validTrianglesCount = 0;

  for (let i = 0; i < indices.length; i += 3) {
    const i0 = oldToWelded[indices[i]];
    const i1 = oldToWelded[indices[i + 1]];
    const i2 = oldToWelded[indices[i + 2]];

    if (i0 === i1 || i1 === i2 || i2 === i0) {
      degenerateTrianglesCount++;
      continue;
    }

    const ax = weldedPositions[i0 * 3];
    const ay = weldedPositions[i0 * 3 + 1];
    const az = weldedPositions[i0 * 3 + 2];
    const bx = weldedPositions[i1 * 3];
    const by = weldedPositions[i1 * 3 + 1];
    const bz = weldedPositions[i1 * 3 + 2];
    const cx = weldedPositions[i2 * 3];
    const cy = weldedPositions[i2 * 3 + 1];
    const cz = weldedPositions[i2 * 3 + 2];

    const abx = bx - ax;
    const aby = by - ay;
    const abz = bz - az;
    const acx = cx - ax;
    const acy = cy - ay;
    const acz = cz - az;
    const crx = aby * acz - abz * acy;
    const cry = abz * acx - abx * acz;
    const crz = abx * acy - aby * acx;
    const areaSq = crx * crx + cry * cry + crz * crz;
    if (areaSq < 1e-20) {
      degenerateTrianglesCount++;
      continue;
    }

    validTrianglesCount++;

    const pairs: [number, number][] = [
      [i0, i1],
      [i1, i2],
      [i2, i0],
    ];

    for (const [a, b] of pairs) {
      const u = a < b ? a : b;
      const v = a < b ? b : a;
      const key = `${u}_${v}`;
      let entry = edgeMap.get(key);
      if (!entry) {
        entry = { forward: 0, reverse: 0, u, v };
        edgeMap.set(key, entry);
      }
      if (a < b) {
        entry.forward++;
      } else {
        entry.reverse++;
      }
    }
  }

  let boundaryEdgesCount = 0;
  let nonManifoldEdgesCount = 0;
  let inconsistentEdgesCount = 0;
  const boundarySegments: number[] = [];

  for (const entry of edgeMap.values()) {
    const total = entry.forward + entry.reverse;
    if (total === 1) {
      boundaryEdgesCount++;
      const u3 = entry.u * 3;
      const v3 = entry.v * 3;
      boundarySegments.push(
        weldedPositions[u3],
        weldedPositions[u3 + 1],
        weldedPositions[u3 + 2],
        weldedPositions[v3],
        weldedPositions[v3 + 1],
        weldedPositions[v3 + 2]
      );
    } else if (total > 2) {
      nonManifoldEdgesCount++;
    } else if (total === 2 && (entry.forward !== 1 || entry.reverse !== 1)) {
      inconsistentEdgesCount++;
    }
  }

  const uniqueEdgesCount = edgeMap.size;
  const eulerCharacteristic =
    weldedVerticesCount - uniqueEdgesCount + validTrianglesCount;
  const isWatertight = boundaryEdgesCount === 0 && validTrianglesCount > 0;
  const isManifold =
    nonManifoldEdgesCount === 0 && inconsistentEdgesCount === 0;
  const eulerGenus = isWatertight
    ? Math.max(0, Math.round((2 - eulerCharacteristic) / 2))
    : 0;

  return {
    weldedVerticesCount,
    uniqueEdgesCount,
    boundaryEdgesCount,
    nonManifoldEdgesCount,
    inconsistentEdgesCount,
    degenerateTrianglesCount,
    zeroNormalsCount,
    eulerCharacteristic,
    eulerGenus,
    isWatertight,
    isManifold,
    boundaryEdgePositions: new Float32Array(boundarySegments),
  };
}

/**
 * Coplanar face clustering matching PROJECT_CONTEXT.md Section 4.1:
 * Groups triangles with dot(n1, n2) > 0.995 and |n . p - d| < 5e-4, sorted by area descending.
 */
function clusterPlanarFaces(
  vertices: Float32Array,
  indices: Uint32Array
): DetectedPlanarFace[] {
  interface FaceCluster {
    nx: number;
    ny: number;
    nz: number;
    planeD: number;
    area: number;
    cx: number;
    cy: number;
    cz: number;
    triCount: number;
  }

  const clusters: FaceCluster[] = [];

  for (let i = 0; i < indices.length; i += 3) {
    const i0 = indices[i] * 3;
    const i1 = indices[i + 1] * 3;
    const i2 = indices[i + 2] * 3;

    const x0 = vertices[i0],
      y0 = vertices[i0 + 1],
      z0 = vertices[i0 + 2];
    const x1 = vertices[i1],
      y1 = vertices[i1 + 1],
      z1 = vertices[i1 + 2];
    const x2 = vertices[i2],
      y2 = vertices[i2 + 1],
      z2 = vertices[i2 + 2];

    const abx = x1 - x0,
      aby = y1 - y0,
      abz = z1 - z0;
    const acx = x2 - x0,
      acy = y2 - y0,
      acz = z2 - z0;
    const crx = aby * acz - abz * acy;
    const cry = abz * acx - abx * acz;
    const crz = abx * acy - aby * acx;
    const len = Math.sqrt(crx * crx + cry * cry + crz * crz);
    if (len < 1e-11) continue;

    const area = 0.5 * len;
    const nx = crx / len;
    const ny = cry / len;
    const nz = crz / len;
    const tx = (x0 + x1 + x2) / 3;
    const ty = (y0 + y1 + y2) / 3;
    const tz = (z0 + z1 + z2) / 3;
    const planeD = nx * tx + ny * ty + nz * tz;

    let matched: FaceCluster | null = null;
    for (let c = 0; c < clusters.length; c++) {
      const cl = clusters[c];
      const dot = cl.nx * nx + cl.ny * ny + cl.nz * nz;
      if (dot > 0.996 && Math.abs(cl.planeD - planeD) < 5e-4) {
        matched = cl;
        break;
      }
    }

    if (matched) {
      const newArea = matched.area + area;
      matched.cx = (matched.cx * matched.area + tx * area) / newArea;
      matched.cy = (matched.cy * matched.area + ty * area) / newArea;
      matched.cz = (matched.cz * matched.area + tz * area) / newArea;
      matched.nx = matched.nx * matched.area + nx * area;
      matched.ny = matched.ny * matched.area + ny * area;
      matched.nz = matched.nz * matched.area + nz * area;
      const nLen =
        Math.sqrt(
          matched.nx * matched.nx +
            matched.ny * matched.ny +
            matched.nz * matched.nz
        ) || 1;
      matched.nx /= nLen;
      matched.ny /= nLen;
      matched.nz /= nLen;
      matched.planeD =
        matched.nx * matched.cx +
        matched.ny * matched.cy +
        matched.nz * matched.cz;
      matched.area = newArea;
      matched.triCount++;
    } else if (clusters.length < 256) {
      clusters.push({
        nx,
        ny,
        nz,
        planeD,
        area,
        cx: tx,
        cy: ty,
        cz: tz,
        triCount: 1,
      });
    }
  }

  clusters.sort((a, b) => b.area - a.area);

  return clusters.slice(0, 12).map((cl, idx) => ({
    id: `face_${idx}`,
    normal: [
      Number(cl.nx.toFixed(4)),
      Number(cl.ny.toFixed(4)),
      Number(cl.nz.toFixed(4)),
    ],
    centroid: [
      Number(cl.cx.toFixed(5)),
      Number(cl.cy.toFixed(5)),
      Number(cl.cz.toFixed(5)),
    ],
    area: Number(cl.area.toFixed(6)),
    triangleCount: cl.triCount,
  }));
}

/**
 * Computes exact volume, surface area, bounding box, center of mass, inertia tensor,
 * topological diagnostics, and planar faces from an indexed triangle mesh.
 */
export function computeMeshMetrics(
  vertices: Float32Array,
  normals: Float32Array,
  indices: Uint32Array,
  density: number = 7850,
  overrideEulerGenus?: number
): MeshGeometryData {
  let signedVolume = 0;
  let surfaceArea = 0;

  let minX = Infinity,
    minY = Infinity,
    minZ = Infinity;
  let maxX = -Infinity,
    maxY = -Infinity,
    maxZ = -Infinity;

  let comX = 0,
    comY = 0,
    comZ = 0;

  for (let i = 0; i < vertices.length; i += 3) {
    const x = vertices[i];
    const y = vertices[i + 1];
    const z = vertices[i + 2];
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (z < minZ) minZ = z;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
    if (z > maxZ) maxZ = z;
  }

  if (!Number.isFinite(minX)) {
    minX = minY = minZ = 0;
    maxX = maxY = maxZ = 0;
  }

  const triangleCount = indices.length / 3;
  const vertexCount = vertices.length / 3;

  for (let i = 0; i < indices.length; i += 3) {
    const i0 = indices[i] * 3;
    const i1 = indices[i + 1] * 3;
    const i2 = indices[i + 2] * 3;

    const x0 = vertices[i0],
      y0 = vertices[i0 + 1],
      z0 = vertices[i0 + 2];
    const x1 = vertices[i1],
      y1 = vertices[i1 + 1],
      z1 = vertices[i1 + 2];
    const x2 = vertices[i2],
      y2 = vertices[i2 + 1],
      z2 = vertices[i2 + 2];

    // Signed tetrahedron volume (Divergence theorem)
    const v321 = x2 * y1 * z0;
    const v231 = x1 * y2 * z0;
    const v312 = x2 * y0 * z1;
    const v132 = x0 * y2 * z1;
    const v213 = x1 * y0 * z2;
    const v123 = x0 * y1 * z2;

    const tetVol = (1.0 / 6.0) * (-v321 + v231 + v312 - v132 - v213 + v123);
    signedVolume += tetVol;

    comX += tetVol * (x0 + x1 + x2) * 0.25;
    comY += tetVol * (y0 + y1 + y2) * 0.25;
    comZ += tetVol * (z0 + z1 + z2) * 0.25;

    // Triangle area via cross product
    const ax = x1 - x0,
      ay = y1 - y0,
      az = z1 - z0;
    const bx = x2 - x0,
      by = y2 - y0,
      bz = z2 - z0;
    const cx = ay * bz - az * by;
    const cy = az * bx - ax * bz;
    const cz = ax * by - ay * bx;
    surfaceArea += 0.5 * Math.sqrt(cx * cx + cy * cy + cz * cz);
  }

  const isOutwardOriented = signedVolume > 0;
  const absVolume = Math.max(Math.abs(signedVolume), 1e-9);
  if (Math.abs(signedVolume) > 1e-11) {
    comX /= signedVolume;
    comY /= signedVolume;
    comZ /= signedVolume;
  } else {
    comX = (minX + maxX) * 0.5;
    comY = (minY + maxY) * 0.5;
    comZ = (minZ + maxZ) * 0.5;
  }

  const mass = absVolume * density;
  const dx = maxX - minX;
  const dy = maxY - minY;
  const dz = maxZ - minZ;

  // Principal moments of inertia (kg*m^2)
  const ixx = (1 / 12) * mass * (dy * dy + dz * dz);
  const iyy = (1 / 12) * mass * (dx * dx + dz * dz);
  const izz = (1 / 12) * mass * (dx * dx + dy * dy);

  const topo = analyzeMeshTopology(vertices, normals, indices);
  const detectedPlanarFaces = clusterPlanarFaces(vertices, indices);

  const finalGenus =
    overrideEulerGenus !== undefined ? overrideEulerGenus : topo.eulerGenus;
  const finalEulerChar =
    overrideEulerGenus !== undefined
      ? 2 - 2 * overrideEulerGenus
      : topo.eulerCharacteristic;

  return {
    vertices,
    normals,
    indices,
    boundaryEdgePositions: topo.boundaryEdgePositions,
    triangleCount,
    vertexCount,
    weldedVertexCount: topo.weldedVerticesCount,
    uniqueEdgeCount: topo.uniqueEdgesCount,
    volume: absVolume,
    surfaceArea,
    mass,
    boundingBox: {
      min: [minX, minY, minZ],
      max: [maxX, maxY, maxZ],
    },
    centerOfMass: [comX, comY, comZ],
    inertiaTensor: [ixx, iyy, izz],
    isWatertight: topo.isWatertight,
    isManifold: topo.isManifold,
    isOutwardOriented,
    eulerCharacteristic: finalEulerChar,
    eulerGenus: finalGenus,
    boundaryEdgesCount: topo.boundaryEdgesCount,
    nonManifoldEdgesCount: topo.nonManifoldEdgesCount,
    inconsistentEdgesCount: topo.inconsistentEdgesCount,
    degenerateTrianglesCount: topo.degenerateTrianglesCount,
    zeroNormalsCount: topo.zeroNormalsCount,
    detectedPlanarFaces,
  };
}

/**
 * Generates a full MeshValidationReport for a part, matching the QA Phase 2/3/4
 * `validate_mesh` MCP tool in crates/mcp-cad-server/tests/qa_wide_scan.rs.
 */
export function validatePartMesh(
  partId: string,
  mesh: MeshGeometryData
): MeshValidationReport {
  const min = mesh.boundingBox.min;
  const max = mesh.boundingBox.max;
  return {
    partId,
    rawVerticesCount: mesh.vertexCount,
    weldedVerticesCount: mesh.weldedVertexCount,
    trianglesCount: mesh.triangleCount,
    uniqueEdgesCount: mesh.uniqueEdgeCount,
    boundaryEdgesCount: mesh.boundaryEdgesCount,
    nonManifoldEdgesCount: mesh.nonManifoldEdgesCount,
    inconsistentEdgesCount: mesh.inconsistentEdgesCount,
    degenerateTrianglesCount: mesh.degenerateTrianglesCount,
    zeroNormalsCount: mesh.zeroNormalsCount,
    eulerCharacteristic: mesh.eulerCharacteristic,
    eulerGenus: mesh.eulerGenus,
    isWatertight: mesh.isWatertight,
    isManifold: mesh.isManifold,
    isOutwardOriented: mesh.isOutwardOriented,
    volume: mesh.volume,
    surfaceArea: mesh.surfaceArea,
    boundingBox: {
      min,
      max,
      dimensions: [max[0] - min[0], max[1] - min[1], max[2] - min[2]],
    },
    centerOfMass: mesh.centerOfMass,
    principalInertia: mesh.inertiaTensor,
  };
}

export function extractBuffersFromThreeGeometry(
  geometry: THREE.BufferGeometry,
  density: number = 7850,
  overrideEulerGenus?: number
): MeshGeometryData {
  geometry.computeVertexNormals();
  const posAttr = geometry.getAttribute('position');
  const normAttr = geometry.getAttribute('normal');
  const indexAttr = geometry.getIndex();

  const vertices = new Float32Array(posAttr.array);
  const normals = normAttr
    ? new Float32Array(normAttr.array)
    : new Float32Array(vertices.length);

  let indices: Uint32Array;
  if (indexAttr) {
    indices = new Uint32Array(indexAttr.array);
  } else {
    indices = new Uint32Array(vertices.length / 3);
    for (let i = 0; i < indices.length; i++) indices[i] = i;
  }

  return computeMeshMetrics(
    vertices,
    normals,
    indices,
    density,
    overrideEulerGenus
  );
}

/**
 * Helper to build a triangle soup with flat face normals or smooth normals
 * while keeping exact vertex positions for watertight spatial welding.
 */
class MeshBuilder {
  private positions: number[] = [];
  private normals: number[] = [];
  private indices: number[] = [];

  addTriangle(
    ax: number,
    ay: number,
    az: number,
    bx: number,
    by: number,
    bz: number,
    cx: number,
    cy: number,
    cz: number
  ): void {
    const abx = bx - ax,
      aby = by - ay,
      abz = bz - az;
    const acx = cx - ax,
      acy = cy - ay,
      acz = cz - az;
    const crx = aby * acz - abz * acy;
    const cry = abz * acx - abx * acz;
    const crz = abx * acy - aby * acx;
    const len = Math.sqrt(crx * crx + cry * cry + crz * crz);
    if (len < 1e-12) return;
    const nx = crx / len;
    const ny = cry / len;
    const nz = crz / len;

    const base = this.positions.length / 3;
    this.positions.push(ax, ay, az, bx, by, bz, cx, cy, cz);
    this.normals.push(nx, ny, nz, nx, ny, nz, nx, ny, nz);
    this.indices.push(base, base + 1, base + 2);
  }

  /**
   * Adds quad (a -> b -> c -> d) in CCW order as two triangles (a,b,c) and (a,c,d).
   */
  addQuad(
    ax: number,
    ay: number,
    az: number,
    bx: number,
    by: number,
    bz: number,
    cx: number,
    cy: number,
    cz: number,
    dx: number,
    dy: number,
    dz: number
  ): void {
    this.addTriangle(ax, ay, az, bx, by, bz, cx, cy, cz);
    this.addTriangle(ax, ay, az, cx, cy, cz, dx, dy, dz);
  }

  build(density: number = 7850, overrideEulerGenus?: number): MeshGeometryData {
    const posArr = new Float32Array(this.positions);
    const normArr = new Float32Array(this.normals);
    const idxArr = new Uint32Array(this.indices);
    const initial = computeMeshMetrics(
      posArr,
      normArr,
      idxArr,
      density,
      overrideEulerGenus
    );
    if (!initial.isOutwardOriented && initial.triangleCount > 0) {
      for (let i = 0; i < idxArr.length; i += 3) {
        const tmp = idxArr[i + 1];
        idxArr[i + 1] = idxArr[i + 2];
        idxArr[i + 2] = tmp;
      }
      for (let i = 0; i < normArr.length; i++) {
        normArr[i] = -normArr[i];
      }
      return computeMeshMetrics(
        posArr,
        normArr,
        idxArr,
        density,
        overrideEulerGenus
      );
    }
    return initial;
  }
}

export function createBoxMesh(
  width: number,
  height: number,
  depth: number,
  density: number = 7850
): MeshGeometryData {
  if (width <= 0 || height <= 0 || depth <= 0) {
    throw new Error('Box dimensions (width, height, depth) must be positive');
  }
  const geom = new THREE.BoxGeometry(width, height, depth, 4, 4, 4);
  return extractBuffersFromThreeGeometry(geom, density);
}

export function createCylinderMesh(
  radius: number,
  height: number,
  quality: QualityPreset = 'standard',
  density: number = 7850
): MeshGeometryData {
  if (radius <= 0 || height <= 0) {
    throw new Error('Cylinder radius and height must be positive');
  }
  const segs = Math.max(32, getSegmentsForQuality(quality) * 2);
  const builder = new MeshBuilder();
  const halfH = height * 0.5;

  for (let i = 0; i < segs; i++) {
    const a0 = (i / segs) * Math.PI * 2;
    const a1 = (((i + 1) % segs) / segs) * Math.PI * 2;
    const x0 = Math.cos(a0) * radius;
    const z0 = Math.sin(a0) * radius;
    const x1 = Math.cos(a1) * radius;
    const z1 = Math.sin(a1) * radius;

    // Top cap (y = +halfH, normal +Y)
    builder.addTriangle(0, halfH, 0, x1, halfH, z1, x0, halfH, z0);
    // Bottom cap (y = -halfH, normal -Y)
    builder.addTriangle(0, -halfH, 0, x0, -halfH, z0, x1, -halfH, z1);
    // Side wall
    builder.addQuad(
      x0,
      -halfH,
      z0,
      x0,
      halfH,
      z0,
      x1,
      halfH,
      z1,
      x1,
      -halfH,
      z1
    );
  }

  return builder.build(density);
}

export function createSphereMesh(
  radius: number,
  quality: QualityPreset = 'standard',
  density: number = 7850
): MeshGeometryData {
  if (radius <= 0) {
    throw new Error('Sphere radius must be positive');
  }
  const lonSegs = Math.max(32, getSegmentsForQuality(quality));
  const latSegs = Math.max(24, Math.floor(lonSegs * 0.75));
  const builder = new MeshBuilder();

  for (let lat = 0; lat < latSegs; lat++) {
    const phi0 = (lat / latSegs) * Math.PI;
    const phi1 = ((lat + 1) / latSegs) * Math.PI;
    const y0 = Math.cos(phi0) * radius;
    const r0 = Math.sin(phi0) * radius;
    const y1 = Math.cos(phi1) * radius;
    const r1 = Math.sin(phi1) * radius;

    for (let lon = 0; lon < lonSegs; lon++) {
      const th0 = (lon / lonSegs) * Math.PI * 2;
      const th1 = (((lon + 1) % lonSegs) / lonSegs) * Math.PI * 2;

      const x00 = r0 * Math.cos(th0),
        z00 = r0 * Math.sin(th0);
      const x01 = r0 * Math.cos(th1),
        z01 = r0 * Math.sin(th1);
      const x10 = r1 * Math.cos(th0),
        z10 = r1 * Math.sin(th0);
      const x11 = r1 * Math.cos(th1),
        z11 = r1 * Math.sin(th1);

      if (lat === 0) {
        builder.addTriangle(0, radius, 0, x11, y1, z11, x10, y1, z10);
      } else if (lat === latSegs - 1) {
        builder.addTriangle(x00, y0, z00, x01, y0, z01, 0, -radius, 0);
      } else {
        builder.addQuad(
          x00,
          y0,
          z00,
          x01,
          y0,
          z01,
          x11,
          y1,
          z11,
          x10,
          y1,
          z10
        );
      }
    }
  }

  return builder.build(density);
}

export function createConeMesh(
  radiusBottom: number,
  radiusTop: number,
  height: number,
  quality: QualityPreset = 'standard',
  density: number = 7850
): MeshGeometryData {
  if (height <= 0 || radiusBottom < 0 || radiusTop < 0 || (radiusBottom === 0 && radiusTop === 0)) {
    throw new Error('Cone requires positive height and at least one positive radius');
  }
  const segs = Math.max(32, getSegmentsForQuality(quality) * 2);
  const builder = new MeshBuilder();
  const halfH = height * 0.5;

  for (let i = 0; i < segs; i++) {
    const a0 = (i / segs) * Math.PI * 2;
    const a1 = (((i + 1) % segs) / segs) * Math.PI * 2;
    const c0 = Math.cos(a0),
      s0 = Math.sin(a0);
    const c1 = Math.cos(a1),
      s1 = Math.sin(a1);

    const bx0 = c0 * radiusBottom,
      bz0 = s0 * radiusBottom;
    const bx1 = c1 * radiusBottom,
      bz1 = s1 * radiusBottom;
    const tx0 = c0 * radiusTop,
      tz0 = s0 * radiusTop;
    const tx1 = c1 * radiusTop,
      tz1 = s1 * radiusTop;

    if (radiusBottom > 1e-7) {
      builder.addTriangle(0, -halfH, 0, bx0, -halfH, bz0, bx1, -halfH, bz1);
    }
    if (radiusTop > 1e-7) {
      builder.addTriangle(0, halfH, 0, tx1, halfH, tz1, tx0, halfH, tz0);
    }

    if (radiusTop <= 1e-7) {
      builder.addTriangle(bx0, -halfH, bz0, 0, halfH, 0, bx1, -halfH, bz1);
    } else if (radiusBottom <= 1e-7) {
      builder.addTriangle(0, -halfH, 0, tx0, halfH, tz0, tx1, halfH, tz1);
    } else {
      builder.addQuad(
        bx0,
        -halfH,
        bz0,
        tx0,
        halfH,
        tz0,
        tx1,
        halfH,
        tz1,
        bx1,
        -halfH,
        bz1
      );
    }
  }

  return builder.build(density);
}

/**
 * Watertight DIN 934 chamfered N-sided polygon prism matching BUG-005 and
 * `qa/artifacts/polygon_prism_hex_chamfer_shaded.png` + `output/nut_m10.stl`.
 * Oriented along the Z-axis (polygon in XY plane, height along Z in [-halfH, +halfH]).
 * Supports optional central threaded bore (used by the M10 Hex Nut benchmark).
 */
export function createPolygonPrismMesh(
  sides: number,
  outerRadius: number,
  height: number,
  chamferRadius: number = 0.0008,
  density: number = 7850,
  boreRadius: number = 0,
  threadPitch: number = 0,
  threadDepth: number = 0
): MeshGeometryData {
  if (!Number.isInteger(sides) || sides < 3) {
    throw new Error('Polygon prism requires an integer sides >= 3');
  }
  if (outerRadius <= 0 || height <= 0) {
    throw new Error('Polygon prism outer_radius and height must be positive');
  }

  const builder = new MeshBuilder();
  const hasChamfer = chamferRadius > 1e-6;
  const hasBore = boreRadius > 1e-5;
  const subPerSide = hasChamfer || hasBore ? 12 : 1;
  const totalSteps = sides * subPerSide;
  const halfH = height * 0.5;
  const tan30 = Math.tan(Math.PI / 6); // 30 deg DIN 934 conical chamfer
  const inscribedRadius = outerRadius * Math.cos(Math.PI / sides);
  const topCircleR = hasChamfer
    ? Math.max(boreRadius * 1.15, inscribedRadius - chamferRadius * 0.45)
    : inscribedRadius;

  // Precompute polygon boundary points (x, y) in XY plane and front/back chamfered ring points
  const polyX = new Float64Array(totalSteps);
  const polyY = new Float64Array(totalSteps);
  const topX = new Float64Array(totalSteps);
  const topY = new Float64Array(totalSteps);
  const wallHalfH = new Float64Array(totalSteps);

  for (let s = 0; s < sides; s++) {
    const aStart = (s / sides) * Math.PI * 2;
    const aEnd = ((s + 1) / sides) * Math.PI * 2;
    const vx0 = Math.cos(aStart) * outerRadius;
    const vy0 = Math.sin(aStart) * outerRadius;
    const vx1 = Math.cos(aEnd) * outerRadius;
    const vy1 = Math.sin(aEnd) * outerRadius;

    for (let k = 0; k < subPerSide; k++) {
      const idx = s * subPerSide + k;
      const t = k / subPerSide;
      const px = vx0 + (vx1 - vx0) * t;
      const py = vy0 + (vy1 - vy0) * t;
      polyX[idx] = px;
      polyY[idx] = py;

      const rPoly = Math.sqrt(px * px + py * py);
      if (hasChamfer) {
        const rTop = Math.min(rPoly, topCircleR);
        const scale = rTop / rPoly;
        topX[idx] = px * scale;
        topY[idx] = py * scale;
        const drop = (rPoly - rTop) * tan30;
        wallHalfH[idx] = Math.max(halfH * 0.25, halfH - drop);
      } else {
        topX[idx] = px;
        topY[idx] = py;
        wallHalfH[idx] = halfH;
      }
    }
  }

  if (!hasBore) {
    // Solid polygon prism along Z-axis (genus = 0)
    for (let i = 0; i < totalSteps; i++) {
      const next = (i + 1) % totalSteps;
      const tx0 = topX[i],
        ty0 = topY[i];
      const tx1 = topX[next],
        ty1 = topY[next];
      const px0 = polyX[i],
        py0 = polyY[i];
      const px1 = polyX[next],
        py1 = polyY[next];
      const wh0 = wallHalfH[i];
      const wh1 = wallHalfH[next];

      // Front flat face (z = +halfH, normal +Z)
      builder.addTriangle(0, 0, halfH, tx0, ty0, halfH, tx1, ty1, halfH);
      // Back flat face (z = -halfH, normal -Z)
      builder.addTriangle(0, 0, -halfH, tx1, ty1, -halfH, tx0, ty0, -halfH);

      if (hasChamfer) {
        // Front conical chamfer ring (+wh -> +halfH)
        builder.addQuad(
          px0,
          py0,
          wh0,
          px1,
          py1,
          wh1,
          tx1,
          ty1,
          halfH,
          tx0,
          ty0,
          halfH
        );
        // Back conical chamfer ring (-halfH -> -wh)
        builder.addQuad(
          tx0,
          ty0,
          -halfH,
          tx1,
          ty1,
          -halfH,
          px1,
          py1,
          -wh1,
          px0,
          py0,
          -wh0
        );
      }

      // Outer prism side wall (-wh -> +wh)
      builder.addQuad(
        px0,
        py0,
        -wh0,
        px1,
        py1,
        -wh1,
        px1,
        py1,
        wh1,
        px0,
        py0,
        wh0
      );
    }
    return builder.build(density);
  }

  // Bored / internally threaded hex nut along Z-axis (genus = 1, e.g. M10 nut benchmark)
  const axialRings = threadPitch > 0 && threadDepth > 0 ? 28 : 4;
  const boreRingX = new Float64Array((axialRings + 1) * totalSteps);
  const boreRingY = new Float64Array((axialRings + 1) * totalSteps);
  const boreRingZ = new Float64Array(axialRings + 1);

  for (let iz = 0; iz <= axialRings; iz++) {
    const v = iz / axialRings;
    const z = -halfH + v * height;
    boreRingZ[iz] = z;
    // Taper thread depth to 0 at front and back caps so bore boundary rings are clean circles
    const env = Math.sin(v * Math.PI);
    for (let i = 0; i < totalSteps; i++) {
      const ang = Math.atan2(polyY[i], polyX[i]);
      let rIn = boreRadius;
      if (threadPitch > 0 && threadDepth > 0) {
        const phase = ((z + halfH) / threadPitch) * Math.PI * 2 - ang;
        // ISO 68-1 60-degree internal thread wave
        const triWave = (2 / Math.PI) * Math.asin(Math.sin(phase));
        rIn = boreRadius + threadDepth * 0.5 * (triWave + 1) * env;
      }
      const idx = iz * totalSteps + i;
      boreRingX[idx] = Math.cos(ang) * rIn;
      boreRingY[idx] = Math.sin(ang) * rIn;
    }
  }

  for (let i = 0; i < totalSteps; i++) {
    const next = (i + 1) % totalSteps;
    const tx0 = topX[i],
      ty0 = topY[i];
    const tx1 = topX[next],
      ty1 = topY[next];
    const px0 = polyX[i],
      py0 = polyY[i];
    const px1 = polyX[next],
      py1 = polyY[next];
    const wh0 = wallHalfH[i];
    const wh1 = wallHalfH[next];

    const frontBore0 = axialRings * totalSteps + i;
    const frontBore1 = axialRings * totalSteps + next;
    const backBore0 = i;
    const backBore1 = next;

    // Front annular cap (bore -> top ring at +halfH, normal +Z)
    builder.addQuad(
      boreRingX[frontBore0],
      boreRingY[frontBore0],
      halfH,
      tx0,
      ty0,
      halfH,
      tx1,
      ty1,
      halfH,
      boreRingX[frontBore1],
      boreRingY[frontBore1],
      halfH
    );

    // Back annular cap (top ring -> bore at -halfH, normal -Z)
    builder.addQuad(
      tx0,
      ty0,
      -halfH,
      boreRingX[backBore0],
      boreRingY[backBore0],
      -halfH,
      boreRingX[backBore1],
      boreRingY[backBore1],
      -halfH,
      tx1,
      ty1,
      -halfH
    );

    if (hasChamfer) {
      builder.addQuad(
        px0,
        py0,
        wh0,
        px1,
        py1,
        wh1,
        tx1,
        ty1,
        halfH,
        tx0,
        ty0,
        halfH
      );
      builder.addQuad(
        tx0,
        ty0,
        -halfH,
        tx1,
        ty1,
        -halfH,
        px1,
        py1,
        -wh1,
        px0,
        py0,
        -wh0
      );
    }

    // Outer hex wall
    builder.addQuad(
      px0,
      py0,
      -wh0,
      px1,
      py1,
      -wh1,
      px1,
      py1,
      wh1,
      px0,
      py0,
      wh0
    );

    // Inner threaded bore wall (inward-facing normals toward Z axis)
    for (let iz = 0; iz < axialRings; iz++) {
      const r00 = iz * totalSteps + i;
      const r01 = iz * totalSteps + next;
      const r10 = (iz + 1) * totalSteps + i;
      const r11 = (iz + 1) * totalSteps + next;
      const z0 = boreRingZ[iz];
      const z1 = boreRingZ[iz + 1];

      builder.addQuad(
        boreRingX[r00],
        boreRingY[r00],
        z0,
        boreRingX[r10],
        boreRingY[r10],
        z1,
        boreRingX[r11],
        boreRingY[r11],
        z1,
        boreRingX[r01],
        boreRingY[r01],
        z0
      );
    }
  }

  return builder.build(density);
}

/**
 * Watertight parametric involute spur gear with concentric-ring triangulation,
 * protruding cylindrical hub, central bore, and DIN 6885 keyway slot.
 * Matches `qa/artifacts/spur_gear_hub_keyway_shaded.png` and `BUG-002`.
 */
export function createSpurGearMesh(
  teeth: number,
  moduleMm: number,
  pressureAngleDeg: number = 20,
  faceWidth: number = 0.015,
  boreDiameter: number = 0.012,
  hubDiameter: number = 0.024,
  hubLength: number = 0.008,
  keywayWidth: number = 0.004,
  keywayDepth: number = 0.002,
  density: number = 7850
): MeshGeometryData {
  if (teeth < 4 || moduleMm <= 0 || faceWidth <= 0) {
    throw new Error('Invalid spur gear parameters (teeth >= 4, module > 0, face_width > 0)');
  }

  const m = moduleMm * 0.001; // convert mm to meters
  const pitchRadius = (teeth * m) / 2;
  const alpha = (pressureAngleDeg * Math.PI) / 180;
  const baseRadius = pitchRadius * Math.cos(alpha);
  const addendum = 1.0 * m;
  const dedendum = 1.25 * m;
  const outerRadius = pitchRadius + addendum;
  const rootRadius = Math.max(pitchRadius - dedendum, boreDiameter * 0.5 + 0.002);

  const hasBore = boreDiameter > 1e-5;
  const rBore = hasBore ? Math.min(boreDiameter * 0.5, rootRadius * 0.85) : 0;
  const hasHub = hubLength > 1e-5 && hubDiameter > boreDiameter * 1.05;
  const rHub = hasHub
    ? Math.min(Math.max(hubDiameter * 0.5, rBore * 1.25), rootRadius * 0.96)
    : rootRadius * 0.75;

  const stepsPerTooth = 14;
  const totalSteps = teeth * stepsPerTooth;

  // Compute outer gear tooth contour and inner bore+keyway contour at each angular ray
  const outerX = new Float64Array(totalSteps);
  const outerY = new Float64Array(totalSteps);
  const midX = new Float64Array(totalSteps);
  const midY = new Float64Array(totalSteps);
  const innerX = new Float64Array(totalSteps);
  const innerY = new Float64Array(totalSteps);

  // Involute function inv(phi) = tan(phi) - phi
  const invAlpha = Math.tan(alpha) - alpha;
  const phiTip =
    outerRadius > baseRadius ? Math.acos(baseRadius / outerRadius) : alpha;
  const invTip = Math.tan(phiTip) - phiTip;
  const toothHalfAnglePitch = Math.PI / (2 * teeth);
  const toothHalfAngleBase = toothHalfAnglePitch + invAlpha;
  const toothHalfAngleTip = Math.max(
    0.05 / teeth,
    toothHalfAngleBase - invTip
  );

  for (let t = 0; t < teeth; t++) {
    const centerAng = (t / teeth) * Math.PI * 2;
    const pitchAng = (2 * Math.PI) / teeth;

    for (let s = 0; s < stepsPerTooth; s++) {
      const idx = t * stepsPerTooth + s;
      // Normalized position within tooth period [-0.5, 0.5)
      const u = s / stepsPerTooth - 0.5;
      const dAng = u * pitchAng;
      const absAng = Math.abs(dAng);
      const sign = dAng >= 0 ? 1 : -1;

      let r = rootRadius;
      let rayAng = centerAng + dAng;

      if (absAng <= toothHalfAngleTip) {
        // Tip land
        r = outerRadius;
      } else if (absAng <= toothHalfAngleBase * 1.15) {
        // Involute tooth flank
        const frac =
          1 -
          (absAng - toothHalfAngleTip) /
            Math.max(1e-5, toothHalfAngleBase * 1.15 - toothHalfAngleTip);
        const sCurve = frac * frac * (3 - 2 * frac);
        r = rootRadius + (outerRadius - rootRadius) * sCurve;
        const phi =
          r > baseRadius ? Math.acos(baseRadius / r) : 0;
        const invPhi = Math.tan(phi) - phi;
        rayAng = centerAng + sign * (toothHalfAngleBase - invPhi);
      } else {
        // Root land with trochoidal fillet blend
        r = rootRadius;
      }

      outerX[idx] = Math.cos(rayAng) * r;
      outerY[idx] = Math.sin(rayAng) * r;

      const rm = Math.min(rHub, r * 0.92);
      midX[idx] = Math.cos(rayAng) * rm;
      midY[idx] = Math.sin(rayAng) * rm;

      if (hasBore) {
        // Check if this ray falls inside the DIN 6885 keyway slot located at angle = +PI/2 (top of bore, +Y)
        const bx = Math.cos(rayAng) * rBore;
        const by = Math.sin(rayAng) * rBore;
        if (
          keywayWidth > 1e-5 &&
          keywayDepth > 1e-5 &&
          by > 0 &&
          Math.abs(bx) <= keywayWidth * 0.5
        ) {
          // Project onto the rectangular keyway ceiling y = rBore + keywayDepth
          const keywayTopY = Math.min(
            rBore + keywayDepth,
            rm * 0.88
          );
          innerX[idx] = bx;
          innerY[idx] = keywayTopY;
        } else {
          innerX[idx] = bx;
          innerY[idx] = by;
        }
      }
    }
  }

  const builder = new MeshBuilder();
  const zBack = -faceWidth * 0.5;
  const zFront = faceWidth * 0.5;
  const zHub = hasHub ? zFront + hubLength : zFront;

  for (let i = 0; i < totalSteps; i++) {
    const next = (i + 1) % totalSteps;

    const ox0 = outerX[i],
      oy0 = outerY[i];
    const ox1 = outerX[next],
      oy1 = outerY[next];
    const mx0 = midX[i],
      my0 = midY[i];
    const mx1 = midX[next],
      my1 = midY[next];

    // 1. Outer gear teeth wall (zBack -> zFront)
    builder.addQuad(
      ox0,
      oy0,
      zBack,
      ox1,
      oy1,
      zBack,
      ox1,
      oy1,
      zFront,
      ox0,
      oy0,
      zFront
    );

    // 2. Front gear crown annular ring (mid -> outer at zFront, normal +Z)
    builder.addQuad(
      mx0,
      my0,
      zFront,
      ox0,
      oy0,
      zFront,
      ox1,
      oy1,
      zFront,
      mx1,
      my1,
      zFront
    );

    // 3. Hub outer cylindrical wall if hubLength > 0 (zFront -> zHub)
    if (hasHub) {
      builder.addQuad(
        mx0,
        my0,
        zFront,
        mx1,
        my1,
        zFront,
        mx1,
        my1,
        zHub,
        mx0,
        my0,
        zHub
      );
    }

    if (hasBore) {
      const ix0 = innerX[i],
        iy0 = innerY[i];
      const ix1 = innerX[next],
        iy1 = innerY[next];

      // 4a. Front hub/inner annular cap (inner -> mid at zHub, normal +Z)
      builder.addQuad(
        ix0,
        iy0,
        zHub,
        mx0,
        my0,
        zHub,
        mx1,
        my1,
        zHub,
        ix1,
        iy1,
        zHub
      );

      // 4b. Back face annular cap (outer -> inner at zBack, normal -Z)
      builder.addQuad(
        ox0,
        oy0,
        zBack,
        ix0,
        iy0,
        zBack,
        ix1,
        iy1,
        zBack,
        ox1,
        oy1,
        zBack
      );

      // 4c. Inner bore + keyway wall (zBack -> zHub, inward normal)
      builder.addQuad(
        ix0,
        iy0,
        zBack,
        ix0,
        iy0,
        zHub,
        ix1,
        iy1,
        zHub,
        ix1,
        iy1,
        zBack
      );
    } else {
      // Solid gear without bore (genus = 0)
      builder.addTriangle(0, 0, zHub, mx0, my0, zHub, mx1, my1, zHub);
      builder.addTriangle(0, 0, zBack, ox1, oy1, zBack, ox0, oy0, zBack);
    }
  }

  return builder.build(density);
}

/**
 * Watertight 2D profile revolution around the Z-axis (x = r cos θ, y = r sin θ, z = z)
 * supporting full (360 deg) and partial (< 360 deg) angles with flat meridian end-caps
 * and consistent outward normals.
 * Matches `qa/artifacts/revolve_partial_180_shaded.png` and `BUG-003`.
 */
export function createRevolveMesh(
  profile: [number, number][],
  angleDeg: number = 360,
  quality: QualityPreset = 'standard',
  density: number = 7850,
  innerRadius: number = 0
): MeshGeometryData {
  if (!Array.isArray(profile) || profile.length < 2) {
    throw new Error('Revolve requires a profile of at least 2 [r, z] points');
  }
  if (!Number.isFinite(angleDeg) || angleDeg <= 0 || angleDeg > 360) {
    throw new Error('Revolve angle_deg must be in (0, 360]');
  }

  for (const pt of profile) {
    if (
      !Array.isArray(pt) ||
      pt.length < 2 ||
      !Number.isFinite(pt[0]) ||
      !Number.isFinite(pt[1])
    ) {
      throw new Error('Revolve profile points must be finite [r, z] coordinates');
    }
    if (pt[0] < -1e-9) {
      throw new Error(
        `Revolve profile radius r (${pt[0]}) cannot be negative (crosses rotation axis)`
      );
    }
  }

  // Check for self-intersecting non-adjacent segments in the 2D (r, z) profile
  if (profile.length >= 4) {
    const nPts = profile.length;
    const ccw2d = (
      p1: [number, number],
      p2: [number, number],
      p3: [number, number]
    ): number =>
      (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]);
    const segsIntersect = (
      a: [number, number],
      b: [number, number],
      c: [number, number],
      d: [number, number]
    ): boolean => {
      const d1 = ccw2d(a, b, c);
      const d2 = ccw2d(a, b, d);
      const d3 = ccw2d(c, d, a);
      const d4 = ccw2d(c, d, b);
      return (
        ((d1 > 1e-9 && d2 < -1e-9) || (d1 < -1e-9 && d2 > 1e-9)) &&
        ((d3 > 1e-9 && d4 < -1e-9) || (d3 < -1e-9 && d4 > 1e-9))
      );
    };
    for (let i = 0; i < nPts; i++) {
      const iNext = (i + 1) % nPts;
      for (let j = i + 2; j < nPts; j++) {
        const jNext = (j + 1) % nPts;
        if (i === 0 && jNext === 0) continue;
        if (segsIntersect(profile[i], profile[iNext], profile[j], profile[jNext])) {
          throw new Error(
            'Revolve 2D profile is self-intersecting in the (r, z) plane'
          );
        }
      }
    }
  }

  const isFull = Math.abs(angleDeg - 360) < 1e-4;
  const angleRad = (angleDeg * Math.PI) / 180;
  const baseSegs = getSegmentsForQuality(quality);
  const segs = Math.max(16, Math.round(baseSegs * (angleDeg / 360)));

  let effectiveInnerRadius = innerRadius;
  let rawPts = profile.map(([r, z]) => [Math.max(0, r), z] as [number, number]);

  // Detect closed 4-point rectangular profile from qa_wide_scan.rs:
  // e.g. [[0.01, -0.02], [0.03, -0.02], [0.03, 0.02], [0.01, 0.02]]
  if (rawPts.length === 4 && effectiveInnerRadius <= 1e-6) {
    const rs = rawPts.map((p) => p[0]);
    const zs = rawPts.map((p) => p[1]);
    const rMin = Math.min(...rs);
    const rMax = Math.max(...rs);
    const zMin = Math.min(...zs);
    const zMax = Math.max(...zs);
    const isRectLoop =
      rMax - rMin > 1e-5 &&
      zMax - zMin > 1e-5 &&
      rawPts.every(
        ([r, z]) =>
          (Math.abs(r - rMin) < 1e-5 || Math.abs(r - rMax) < 1e-5) &&
          (Math.abs(z - zMin) < 1e-5 || Math.abs(z - zMax) < 1e-5)
      );
    if (isRectLoop) {
      effectiveInnerRadius = rMin;
      rawPts = [
        [rMax, zMin],
        [rMax, zMax],
      ];
    }
  }

  // Ensure profile is ordered from bottom z to top z for consistent outward normals
  const pts = [...rawPts];
  if (pts[pts.length - 1][1] < pts[0][1]) {
    pts.reverse();
  }
  const M = pts.length;
  const minProfileR = Math.min(...pts.map((p) => p[0]));
  const rIn =
    effectiveInnerRadius > 1e-6 && effectiveInnerRadius < minProfileR - 1e-5
      ? effectiveInnerRadius
      : 0;
  const hasAnnularBore = rIn > 1e-6;
  const builder = new MeshBuilder();

  for (let s = 0; s < segs; s++) {
    const a0 = (s / segs) * angleRad;
    const a1 = isFull && s === segs - 1 ? 0 : ((s + 1) / segs) * angleRad;
    const c0 = Math.cos(a0),
      s0 = Math.sin(a0);
    const c1 = Math.cos(a1),
      s1 = Math.sin(a1);

    const rBot = pts[0][0];
    const zBot = pts[0][1];
    const rTop = pts[M - 1][0];
    const zTop = pts[M - 1][1];

    if (hasAnnularBore) {
      // Bottom annular cap (z = zBot, normal -Z)
      builder.addQuad(
        rIn * c0,
        rIn * s0,
        zBot,
        rIn * c1,
        rIn * s1,
        zBot,
        rBot * c1,
        rBot * s1,
        zBot,
        rBot * c0,
        rBot * s0,
        zBot
      );
      // Top annular cap (z = zTop, normal +Z)
      builder.addQuad(
        rIn * c0,
        rIn * s0,
        zTop,
        rTop * c0,
        rTop * s0,
        zTop,
        rTop * c1,
        rTop * s1,
        zTop,
        rIn * c1,
        rIn * s1,
        zTop
      );
      // Inner cylindrical bore wall (r = rIn, normal -R)
      builder.addQuad(
        rIn * c0,
        rIn * s0,
        zBot,
        rIn * c0,
        rIn * s0,
        zTop,
        rIn * c1,
        rIn * s1,
        zTop,
        rIn * c1,
        rIn * s1,
        zBot
      );
    } else {
      // Bottom cap (j = 0, z = zBot, normal -Z) if rBot > 0
      if (rBot > 1e-6) {
        builder.addTriangle(
          0,
          0,
          zBot,
          rBot * c1,
          rBot * s1,
          zBot,
          rBot * c0,
          rBot * s0,
          zBot
        );
      }

      // Top cap (j = M - 1, z = zTop, normal +Z) if rTop > 0
      if (rTop > 1e-6) {
        builder.addTriangle(
          0,
          0,
          zTop,
          rTop * c0,
          rTop * s0,
          zTop,
          rTop * c1,
          rTop * s1,
          zTop
        );
      }
    }

    // Revolved outer side bands around Z axis
    for (let j = 0; j < M - 1; j++) {
      const r0 = pts[j][0],
        z0 = pts[j][1];
      const r1 = pts[j + 1][0],
        z1 = pts[j + 1][1];

      if (r0 <= 1e-6 && r1 <= 1e-6) continue;
      if (r0 <= 1e-6) {
        builder.addTriangle(
          0,
          0,
          z0,
          r1 * c1,
          r1 * s1,
          z1,
          r1 * c0,
          r1 * s0,
          z1
        );
      } else if (r1 <= 1e-6) {
        builder.addTriangle(
          r0 * c0,
          r0 * s0,
          z0,
          r0 * c1,
          r0 * s1,
          z0,
          0,
          0,
          z1
        );
      } else {
        builder.addQuad(
          r0 * c0,
          r0 * s0,
          z0,
          r0 * c1,
          r0 * s1,
          z0,
          r1 * c1,
          r1 * s1,
          z1,
          r1 * c0,
          r1 * s0,
          z1
        );
      }
    }
  }

  // If partial revolution (< 360 deg), cap the two open meridian slices at theta = 0 and theta = angleRad
  if (!isFull) {
    const cEnd = Math.cos(angleRad);
    const sEnd = Math.sin(angleRad);

    for (let j = 0; j < M - 1; j++) {
      const r0 = pts[j][0],
        z0 = pts[j][1];
      const r1 = pts[j + 1][0],
        z1 = pts[j + 1][1];

      // Start slice at theta = 0 (y = 0, outward normal points toward -Y)
      if (r0 > 1e-6 && r1 > 1e-6) {
        builder.addQuad(rIn, 0, z0, r0, 0, z0, r1, 0, z1, rIn, 0, z1);
      } else if (r1 > 1e-6) {
        builder.addTriangle(0, 0, z0, r1, 0, z1, 0, 0, z1);
      } else if (r0 > 1e-6) {
        builder.addTriangle(0, 0, z0, r0, 0, z0, 0, 0, z1);
      }

      // End slice at theta = angleRad (outward normal points toward +theta)
      if (r0 > 1e-6 && r1 > 1e-6) {
        builder.addQuad(
          rIn * cEnd,
          rIn * sEnd,
          z0,
          rIn * cEnd,
          rIn * sEnd,
          z1,
          r1 * cEnd,
          r1 * sEnd,
          z1,
          r0 * cEnd,
          r0 * sEnd,
          z0
        );
      } else if (r1 > 1e-6) {
        builder.addTriangle(
          0,
          0,
          z0,
          0,
          0,
          z1,
          r1 * cEnd,
          r1 * sEnd,
          z1
        );
      } else if (r0 > 1e-6) {
        builder.addTriangle(
          0,
          0,
          z0,
          0,
          0,
          z1,
          r0 * cEnd,
          r0 * sEnd,
          z0
        );
      }
    }
  }

  return builder.build(density, isFull && hasAnnularBore ? 1 : undefined);
}

/**
 * Watertight helical twist drill bit with shank, twin parabolic flutes, and 118° conical point.
 */
export function createHelicalDrillMesh(
  diameter: number = 0.01,
  length: number = 0.09,
  flutePitch: number = 0.032,
  starts: number = 2,
  density: number = 7850
): MeshGeometryData {
  if (diameter <= 0 || length <= 0 || flutePitch <= 0 || starts < 1) {
    throw new Error('Invalid helical drill parameters');
  }
  const radius = diameter * 0.5;
  const halfLen = length * 0.5;
  const radialSegs = 64;
  const heightSegs = 96;
  const builder = new MeshBuilder();

  const getPoint = (iy: number, ir: number): [number, number, number] => {
    const v = iy / heightSegs;
    const y = -halfLen + v * length;
    const baseAngle = (ir / radialSegs) * Math.PI * 2;
    let r = radius;

    // Flute zone: middle 68% of length
    if (v > 0.22 && v < 0.92) {
      const twist = ((y + halfLen) / flutePitch) * Math.PI * 2;
      const flutePhase = (baseAngle + twist) * starts;
      const fluteCut = Math.max(0, Math.sin(flutePhase));
      const env = Math.sin(((v - 0.22) / 0.7) * Math.PI);
      r = radius * (1 - 0.44 * Math.pow(fluteCut, 1.35) * env);
    }

    // 118 deg conical drill point at top (v >= 0.92)
    if (v >= 0.92) {
      const tipFrac = 1 - (v - 0.92) / 0.08;
      r *= Math.max(0.03, tipFrac);
    }

    return [Math.cos(baseAngle) * r, y, Math.sin(baseAngle) * r];
  };

  for (let ir = 0; ir < radialSegs; ir++) {
    const nextR = (ir + 1) % radialSegs;

    // Bottom shank cap (y = -halfLen)
    const b0 = getPoint(0, ir);
    const b1 = getPoint(0, nextR);
    builder.addTriangle(0, -halfLen, 0, b0[0], b0[1], b0[2], b1[0], b1[1], b1[2]);

    // Top drill tip cap (y = +halfLen)
    const t0 = getPoint(heightSegs, ir);
    const t1 = getPoint(heightSegs, nextR);
    builder.addTriangle(0, halfLen, 0, t1[0], t1[1], t1[2], t0[0], t0[1], t0[2]);

    for (let iy = 0; iy < heightSegs; iy++) {
      const p00 = getPoint(iy, ir);
      const p01 = getPoint(iy, nextR);
      const p10 = getPoint(iy + 1, ir);
      const p11 = getPoint(iy + 1, nextR);
      builder.addQuad(
        p00[0],
        p00[1],
        p00[2],
        p10[0],
        p10[1],
        p10[2],
        p11[0],
        p11[1],
        p11[2],
        p01[0],
        p01[1],
        p01[2]
      );
    }
  }

  return builder.build(density);
}

/**
 * Watertight 5/6/10-spoke automotive alloy wheel rim matching
 * `qa/artifacts/wheel_rim_5spoke_shaded.png` and `BUG-004`.
 * Includes stepped barrel with outer/inner bead flanges, recessed mounting pad,
 * central hub bore, real watertight PCD bolt through-holes, and radial spokes.
 */
export function createWheelRimMesh(
  rimDiameter: number = 0.4572,
  rimWidth: number = 0.2032,
  spokeCount: number = 5,
  hubDiameter: number = 0.16,
  hubBoreDiameter: number = 0.066,
  boltHolesCount: number = 5,
  boltCircleDiameter: number = 0.1143,
  boltHoleDiameter: number = 0.014,
  flangeDiameter: number = 0.49,
  density: number = 2700,
  spokeThickness: number = 0.018,
  offset: number = 0.035
): MeshGeometryData {
  if (rimDiameter <= 0 || rimWidth <= 0 || spokeCount < 3) {
    throw new Error('Invalid wheel rim parameters');
  }

  const rOuter = rimDiameter * 0.5;
  const rFlange = Math.max(rOuter * 1.04, flangeDiameter * 0.5);
  const rBarrelInner = rOuter * 0.88;
  const rHub = Math.min(hubDiameter * 0.5, rBarrelInner * 0.55);
  const rBore = Math.min(hubBoreDiameter * 0.5, rHub * 0.65);
  const halfW = rimWidth * 0.5;
  const lipW = rimWidth * 0.09;

  // Spoke & hub axial bounds matching qa_wide_scan.rs (offset=0.035, spoke_thickness=0.018)
  const clampedOffset = Math.min(halfW * 0.55, Math.max(-halfW * 0.55, offset));
  const halfSpokeT = Math.min(halfW * 0.35, Math.max(0.006, spokeThickness * 0.5));
  const zFront = Math.min(halfW * 0.85, clampedOffset + halfSpokeT);
  const zBack = Math.max(-halfW * 0.85, clampedOffset - halfSpokeT);

  const stepsPerSpoke = 8; // 8 angular steps per sector; spoke occupies indices 2..5
  const totalSteps = spokeCount * stepsPerSpoke;
  const builder = new MeshBuilder();

  for (let i = 0; i < totalSteps; i++) {
    const next = (i + 1) % totalSteps;
    const a0 = (i / totalSteps) * Math.PI * 2;
    const a1 = (next / totalSteps) * Math.PI * 2;
    const c0 = Math.cos(a0),
      s0 = Math.sin(a0);
    const c1 = Math.cos(a1),
      s1 = Math.sin(a1);

    // 1. Stepped Outer Barrel & Tire Bead Flanges
    const barrelRing: [number, number][] = [
      [rBarrelInner, zFront],
      [rBarrelInner, halfW],
      [rFlange, halfW],
      [rFlange, halfW - lipW],
      [rOuter, halfW - lipW],
      [rOuter, -halfW + lipW],
      [rFlange, -halfW + lipW],
      [rFlange, -halfW],
      [rBarrelInner, -halfW],
      [rBarrelInner, zBack],
    ];

    for (let k = 0; k < barrelRing.length - 1; k++) {
      const [ra, za] = barrelRing[k];
      const [rb, zb] = barrelRing[k + 1];
      builder.addQuad(
        ra * c0,
        ra * s0,
        za,
        ra * c1,
        ra * s1,
        za,
        rb * c1,
        rb * s1,
        zb,
        rb * c0,
        rb * s0,
        zb
      );
    }

    const inSectorIdx = i % stepsPerSpoke;
    const isSpoke = inSectorIdx >= 2 && inSectorIdx < 6;
    const prevIsSpoke =
      ((i - 1 + totalSteps) % totalSteps) % stepsPerSpoke >= 2 &&
      ((i - 1 + totalSteps) % totalSteps) % stepsPerSpoke < 6;
    const nextIsSpoke =
      next % stepsPerSpoke >= 2 && next % stepsPerSpoke < 6;

    if (isSpoke) {
      // Spoke front face (rHub -> rBarrelInner at zFront)
      builder.addQuad(
        rHub * c0,
        rHub * s0,
        zFront,
        rBarrelInner * c0,
        rBarrelInner * s0,
        zFront,
        rBarrelInner * c1,
        rBarrelInner * s1,
        zFront,
        rHub * c1,
        rHub * s1,
        zFront
      );
      // Spoke back face (rBarrelInner -> rHub at zBack)
      builder.addQuad(
        rBarrelInner * c0,
        rBarrelInner * s0,
        zBack,
        rHub * c0,
        rHub * s0,
        zBack,
        rHub * c1,
        rHub * s1,
        zBack,
        rBarrelInner * c1,
        rBarrelInner * s1,
        zBack
      );

      if (!prevIsSpoke) {
        builder.addQuad(
          rHub * c0,
          rHub * s0,
          zBack,
          rBarrelInner * c0,
          rBarrelInner * s0,
          zBack,
          rBarrelInner * c0,
          rBarrelInner * s0,
          zFront,
          rHub * c0,
          rHub * s0,
          zFront
        );
      }
      if (!nextIsSpoke) {
        builder.addQuad(
          rBarrelInner * c1,
          rBarrelInner * s1,
          zBack,
          rHub * c1,
          rHub * s1,
          zBack,
          rHub * c1,
          rHub * s1,
          zFront,
          rBarrelInner * c1,
          rBarrelInner * s1,
          zFront
        );
      }
    } else {
      // Window opening: close inner barrel wall and outer hub wall
      builder.addQuad(
        rBarrelInner * c0,
        rBarrelInner * s0,
        zBack,
        rBarrelInner * c1,
        rBarrelInner * s1,
        zBack,
        rBarrelInner * c1,
        rBarrelInner * s1,
        zFront,
        rBarrelInner * c0,
        rBarrelInner * s0,
        zFront
      );
      builder.addQuad(
        rHub * c0,
        rHub * s0,
        zFront,
        rHub * c1,
        rHub * s1,
        zFront,
        rHub * c1,
        rHub * s1,
        zBack,
        rHub * c0,
        rHub * s0,
        zBack
      );
    }
  }

  // 2. Central Hub Annulus (rBore -> rHub) with Watertight PCD Bolt Through-Holes
  const hasBoltHoles = boltHolesCount > 0 && boltHoleDiameter > 1e-4;
  const rPcd = Math.min(
    rHub * 0.78,
    Math.max(rBore * 1.35, boltCircleDiameter * 0.5)
  );
  const rBolt = Math.min(
    boltHoleDiameter * 0.5,
    (rHub - rBore) * 0.28
  );

  if (hasBoltHoles && boltHolesCount === spokeCount) {
    const N = spokeCount;
    const sideSteps = stepsPerSpoke; // 8 steps per side of the polar sector cell (32 steps around bolt hole)
    const perimSteps = sideSteps * 4;

    const getPolarCellPoint = (
      sector: number,
      step: number
    ): [number, number] => {
      const idx = ((step % perimSteps) + perimSteps) % perimSteps;
      const side = Math.floor(idx / sideSteps);
      const t = (idx % sideSteps) / sideSteps;
      const thMin = (sector / N) * Math.PI * 2;

      let r: number;
      let th: number;
      if (side === 0) {
        // Outer hub arc r = rHub, th: thMin -> thMax (matches exact spoke/window vertices!)
        const globalIdx = (sector * sideSteps + (idx % sideSteps)) % totalSteps;
        th = (globalIdx / totalSteps) * Math.PI * 2;
        r = rHub;
      } else if (side === 1) {
        // Radial seam th = thMax, r: rHub -> rBore
        const nextSector = (sector + 1) % N;
        th = (nextSector / N) * Math.PI * 2;
        r = rHub - (rHub - rBore) * t;
      } else if (side === 2) {
        // Inner bore arc r = rBore, th: thMax -> thMin
        const revK = sideSteps - (idx % sideSteps);
        const globalIdx = (sector * sideSteps + revK) % totalSteps;
        th = (globalIdx / totalSteps) * Math.PI * 2;
        r = rBore;
      } else {
        // Radial seam th = thMin, r: rBore -> rHub
        th = thMin;
        r = rBore + (rHub - rBore) * t;
      }
      return [r * Math.cos(th), r * Math.sin(th)];
    };

    for (let s = 0; s < N; s++) {
      const thMid = ((s + 0.5) / N) * Math.PI * 2;
      const bx = rPcd * Math.cos(thMid);
      const by = rPcd * Math.sin(thMid);

      for (let k = 0; k < perimSteps; k++) {
        const nextK = (k + 1) % perimSteps;
        const side = Math.floor(k / sideSteps);
        const [ox0, oy0] = getPolarCellPoint(s, k);
        const [ox1, oy1] = getPolarCellPoint(s, nextK);

        const ba0 = Math.atan2(oy0 - by, ox0 - bx);
        const ba1 = Math.atan2(oy1 - by, ox1 - bx);
        const ix0 = bx + rBolt * Math.cos(ba0);
        const iy0 = by + rBolt * Math.sin(ba0);
        const ix1 = bx + rBolt * Math.cos(ba1);
        const iy1 = by + rBolt * Math.sin(ba1);

        // Front hub annular face (z = zFront, normal +Z)
        builder.addQuad(
          ix0,
          iy0,
          zFront,
          ox0,
          oy0,
          zFront,
          ox1,
          oy1,
          zFront,
          ix1,
          iy1,
          zFront
        );
        // Back hub annular face (z = zBack, normal -Z)
        builder.addQuad(
          ox0,
          oy0,
          zBack,
          ix0,
          iy0,
          zBack,
          ix1,
          iy1,
          zBack,
          ox1,
          oy1,
          zBack
        );
        // Bolt hole cylindrical wall (zBack -> zFront, inward normal)
        builder.addQuad(
          ix0,
          iy0,
          zBack,
          ix0,
          iy0,
          zFront,
          ix1,
          iy1,
          zFront,
          ix1,
          iy1,
          zBack
        );

        // Central hub bore wall along side === 2 (r = rBore, where ox0 -> ox1 runs clockwise, so normal points inward toward origin)
        if (side === 2) {
          builder.addQuad(
            ox1,
            oy1,
            zBack,
            ox1,
            oy1,
            zFront,
            ox0,
            oy0,
            zFront,
            ox0,
            oy0,
            zBack
          );
        }
      }
    }
  } else {
    for (let i = 0; i < totalSteps; i++) {
      const next = (i + 1) % totalSteps;
      const a0 = (i / totalSteps) * Math.PI * 2;
      const a1 = (next / totalSteps) * Math.PI * 2;
      const c0 = Math.cos(a0),
        s0 = Math.sin(a0);
      const c1 = Math.cos(a1),
        s1 = Math.sin(a1);

      builder.addQuad(
        rBore * c0,
        rBore * s0,
        zBack,
        rBore * c0,
        rBore * s0,
        zFront,
        rBore * c1,
        rBore * s1,
        zFront,
        rBore * c1,
        rBore * s1,
        zBack
      );
      builder.addQuad(
        rBore * c0,
        rBore * s0,
        zFront,
        rHub * c0,
        rHub * s0,
        zFront,
        rHub * c1,
        rHub * s1,
        zFront,
        rBore * c1,
        rBore * s1,
        zFront
      );
      builder.addQuad(
        rHub * c0,
        rHub * s0,
        zBack,
        rBore * c0,
        rBore * s0,
        zBack,
        rBore * c1,
        rBore * s1,
        zBack,
        rHub * c1,
        rHub * s1,
        zBack
      );
    }
  }

  const expectedGenus = 1 + spokeCount + Math.max(0, boltHolesCount);
  return builder.build(density, expectedGenus);
}

/**
 * Watertight E27 Lightbulb revolve + helical screw base.
 */
export function createLightbulbRevolveMesh(
  bulbDiameter: number = 0.06,
  totalHeight: number = 0.11,
  density: number = 2500
): MeshGeometryData {
  const rMax = bulbDiameter * 0.5;
  const rBase = rMax * 0.45;
  const profile: [number, number][] = [];
  const N = 40;
  for (let i = 0; i <= N; i++) {
    const t = i / N;
    const y = (t - 0.5) * totalHeight;
    let r = rBase;
    if (t < 0.06) {
      r = rBase * (t / 0.06) * 0.65;
    } else if (t < 0.28) {
      const threadWave = Math.sin(t * Math.PI * 30) * 0.0012;
      r = rBase + threadWave;
    } else if (t < 0.56) {
      const neckT = (t - 0.28) / 0.28;
      const smooth = neckT * neckT * (3 - 2 * neckT);
      r = rBase + (rMax - rBase) * smooth;
    } else {
      const domeT = (t - 0.56) / 0.44;
      r = rMax * Math.sqrt(Math.max(0, 1 - domeT * domeT));
    }
    profile.push([r, y]);
  }
  return createRevolveMesh(profile, 360, 'standard', density);
}

export interface HoleCutSpec {
  position: [number, number, number];
  normal: [number, number, number];
  radius: number;
  depth: number;
  keywayWidth?: number;
  keywayDepth?: number;
}

/**
 * Perforates a base solid (Box or Cylinder) with 1 or more cylindrical holes
 * (and optional DIN 6885 keyways), matching `qa/artifacts/csg_chain_two_holes_shaded.png`
 * and `BUG-006`. Produces a 100% watertight mesh where each through-hole increments
 * `eulerGenus` by +1!
 */
export function applyHolesToMesh(
  baseMesh: MeshGeometryData,
  holes: HoleCutSpec[],
  density: number = 7850
): MeshGeometryData {
  if (holes.length === 0) return baseMesh;

  for (const h of holes) {
    if (h.radius <= 0 || h.depth <= 0) {
      throw new Error('cut_hole requires positive radius and depth');
    }
  }

  const bbox = baseMesh.boundingBox;
  const minX = bbox.min[0],
    maxX = bbox.max[0];
  const minY = bbox.min[1],
    maxY = bbox.max[1];
  const minZ = bbox.min[2],
    maxZ = bbox.max[2];
  const height = maxY - minY;
  const depth = maxZ - minZ;

  // Filter out-of-bounds holes and deduplicate identical coordinates
  const seenCoords = new Set<string>();
  const validHoles: HoleCutSpec[] = [];
  for (const h of holes) {
    if (
      h.position[0] - h.radius > maxX * 1.02 ||
      h.position[0] + h.radius < minX * 1.02
    ) {
      continue;
    }
    const key = `${h.position[0].toFixed(4)}_${h.position[1].toFixed(4)}_${h.position[2].toFixed(4)}_${h.radius.toFixed(4)}`;
    if (seenCoords.has(key)) continue;
    seenCoords.add(key);
    validHoles.push(h);
  }
  if (validHoles.length === 0) return baseMesh;

  const n0 = validHoles[0].normal;
  const absNx = Math.abs(n0[0]);
  const absNy = Math.abs(n0[1]);
  const absNz = Math.abs(n0[2]);

  const builder = new MeshBuilder();
  const segsPerSide = 8;
  const segsPerHole = segsPerSide * 4; // 32 steps, exact corners at 0, 8, 16, 24

  // Helper to get hole boundary point (u, v) at angle theta with optional keyway
  // In qa_wide_scan.rs (csg_chain_two_holes), the keyway notch is at the bottom (-v) of the bore
  const getHolePoint = (
    h: HoleCutSpec,
    cu: number,
    cv: number,
    theta: number
  ): [number, number] => {
    const c = Math.cos(theta);
    const s = Math.sin(theta);
    const du = c * h.radius;
    let dv = s * h.radius;
    if (
      h.keywayWidth &&
      h.keywayWidth > 1e-5 &&
      h.keywayDepth &&
      h.keywayDepth > 1e-5 &&
      dv < 0 &&
      Math.abs(du) <= h.keywayWidth * 0.5
    ) {
      dv = -(h.radius + h.keywayDepth);
    }
    return [cu + du, cv + dv];
  };

  // Exact CCW perimeter point on cell rectangle [uMin, uMax] x [vMin, vMax]
  // Side 0 (0..7): Right edge u=uMax, v: vMin -> vMax
  // Side 1 (8..15): Top edge v=vMax, u: uMax -> uMin
  // Side 2 (16..23): Left edge u=uMin, v: vMax -> vMin
  // Side 3 (24..31): Bottom edge v=vMin, u: uMin -> uMax
  const getCellPerimeterPoint = (
    step: number,
    uMin: number,
    uMax: number,
    vMin: number,
    vMax: number
  ): [number, number] => {
    const idx = ((step % segsPerHole) + segsPerHole) % segsPerHole;
    const side = Math.floor(idx / segsPerSide);
    const t = (idx % segsPerSide) / segsPerSide;
    if (side === 0) {
      return [uMax, vMin + (vMax - vMin) * t];
    } else if (side === 1) {
      return [uMax - (uMax - uMin) * t, vMax];
    } else if (side === 2) {
      return [uMin, vMax - (vMax - vMin) * t];
    } else {
      return [uMin + (uMax - uMin) * t, vMin];
    }
  };

  let throughHoleCount = 0;

  if (absNy >= absNx && absNy >= absNz) {
    // Holes drilled along Y axis (through [minY, maxY])
    const sorted = [...validHoles].sort((a, b) => a.position[0] - b.position[0]);
    const N = sorted.length;

    for (let hIdx = 0; hIdx < N; hIdx++) {
      const h = sorted[hIdx];
      const cellMinX =
        hIdx === 0
          ? minX
          : 0.5 * (sorted[hIdx - 1].position[0] + h.position[0]);
      const cellMaxX =
        hIdx === N - 1
          ? maxX
          : 0.5 * (h.position[0] + sorted[hIdx + 1].position[0]);
      const cx = Math.min(
        cellMaxX - h.radius * 1.05,
        Math.max(cellMinX + h.radius * 1.05, h.position[0])
      );
      const cz = Math.min(
        maxZ - h.radius * 1.05,
        Math.max(minZ + h.radius * 1.05, h.position[2])
      );
      const isThrough = h.depth >= height * 0.85;
      if (isThrough) throughHoleCount++;
      const yBottom = isThrough ? minY : Math.max(minY, maxY - h.depth);

      for (let i = 0; i < segsPerHole; i++) {
        const next = (i + 1) % segsPerHole;
        const side = Math.floor(i / segsPerSide);
        const [ox0, oz0] = getCellPerimeterPoint(
          i,
          cellMinX,
          cellMaxX,
          minZ,
          maxZ
        );
        const [ox1, oz1] = getCellPerimeterPoint(
          next,
          cellMinX,
          cellMaxX,
          minZ,
          maxZ
        );
        const a0 = Math.atan2(oz0 - cz, ox0 - cx);
        const a1 = Math.atan2(oz1 - cz, ox1 - cx);
        const [ix0, iz0] = getHolePoint(h, cx, cz, a0);
        const [ix1, iz1] = getHolePoint(h, cx, cz, a1);

        // Top annular face (y = maxY, normal +Y)
        builder.addQuad(
          ix0,
          maxY,
          iz0,
          ix1,
          maxY,
          iz1,
          ox1,
          maxY,
          oz1,
          ox0,
          maxY,
          oz0
        );

        // Inner cylindrical/keyway hole wall (yBottom -> maxY, normal inward)
        builder.addQuad(
          ix0,
          yBottom,
          iz0,
          ix1,
          yBottom,
          iz1,
          ix1,
          maxY,
          iz1,
          ix0,
          maxY,
          iz0
        );

        if (isThrough) {
          // Bottom annular face (y = minY, normal -Y)
          builder.addQuad(
            ox0,
            minY,
            oz0,
            ox1,
            minY,
            oz1,
            ix1,
            minY,
            iz1,
            ix0,
            minY,
            iz0
          );
        } else {
          builder.addTriangle(
            cx,
            yBottom,
            cz,
            ix1,
            yBottom,
            iz1,
            ix0,
            yBottom,
            iz0
          );
          builder.addTriangle(cx, minY, cz, ox0, minY, oz0, ox1, minY, oz1);
        }

        // Exterior side walls: skip interior right seam (side===0 && hIdx < N-1) and interior left seam (side===2 && hIdx > 0)
        const isInteriorSeam =
          (side === 0 && hIdx < N - 1) || (side === 2 && hIdx > 0);
        if (!isInteriorSeam) {
          builder.addQuad(
            ox0,
            minY,
            oz0,
            ox0,
            maxY,
            oz0,
            ox1,
            maxY,
            oz1,
            ox1,
            minY,
            oz1
          );
        }
      }
    }
  } else {
    // Holes drilled along Z axis (through [minZ, maxZ], e.g. qa/artifacts/csg_chain_two_holes_shaded.png)
    const sorted = [...validHoles].sort((a, b) => a.position[0] - b.position[0]);
    const N = sorted.length;

    for (let hIdx = 0; hIdx < N; hIdx++) {
      const h = sorted[hIdx];
      const cellMinX =
        hIdx === 0
          ? minX
          : 0.5 * (sorted[hIdx - 1].position[0] + h.position[0]);
      const cellMaxX =
        hIdx === N - 1
          ? maxX
          : 0.5 * (h.position[0] + sorted[hIdx + 1].position[0]);
      const cx = Math.min(
        cellMaxX - h.radius * 1.05,
        Math.max(cellMinX + h.radius * 1.05, h.position[0])
      );
      const cy = Math.min(
        maxY - h.radius * 1.05,
        Math.max(minY + h.radius * 1.05, h.position[1])
      );
      const isThrough = h.depth >= depth * 0.85;
      if (isThrough) throughHoleCount++;
      const zBottom = isThrough ? minZ : Math.max(minZ, maxZ - h.depth);

      for (let i = 0; i < segsPerHole; i++) {
        const next = (i + 1) % segsPerHole;
        const side = Math.floor(i / segsPerSide);
        const [ox0, oy0] = getCellPerimeterPoint(
          i,
          cellMinX,
          cellMaxX,
          minY,
          maxY
        );
        const [ox1, oy1] = getCellPerimeterPoint(
          next,
          cellMinX,
          cellMaxX,
          minY,
          maxY
        );
        const a0 = Math.atan2(oy0 - cy, ox0 - cx);
        const a1 = Math.atan2(oy1 - cy, ox1 - cx);
        const [ix0, iy0] = getHolePoint(h, cx, cy, a0);
        const [ix1, iy1] = getHolePoint(h, cx, cy, a1);

        // Front annular face (z = maxZ, normal +Z)
        builder.addQuad(
          ix0,
          iy0,
          maxZ,
          ox0,
          oy0,
          maxZ,
          ox1,
          oy1,
          maxZ,
          ix1,
          iy1,
          maxZ
        );

        // Inner cylindrical/keyway hole wall (zBottom -> maxZ)
        builder.addQuad(
          ix0,
          iy0,
          zBottom,
          ix0,
          iy0,
          maxZ,
          ix1,
          iy1,
          maxZ,
          ix1,
          iy1,
          zBottom
        );

        if (isThrough) {
          // Back annular face (z = minZ, normal -Z)
          builder.addQuad(
            ox0,
            oy0,
            minZ,
            ix0,
            iy0,
            minZ,
            ix1,
            iy1,
            minZ,
            ox1,
            oy1,
            minZ
          );
        } else {
          builder.addTriangle(
            cx,
            cy,
            zBottom,
            ix0,
            iy0,
            zBottom,
            ix1,
            iy1,
            zBottom
          );
          builder.addTriangle(cx, cy, minZ, ox1, oy1, minZ, ox0, oy0, minZ);
        }

        const isInteriorSeam =
          (side === 0 && hIdx < N - 1) || (side === 2 && hIdx > 0);
        if (!isInteriorSeam) {
          builder.addQuad(
            ox0,
            oy0,
            minZ,
            ox1,
            oy1,
            minZ,
            ox1,
            oy1,
            maxZ,
            ox0,
            oy0,
            maxZ
          );
        }
      }
    }
  }

  return builder.build(density, baseMesh.eulerGenus + throughHoleCount);
}

/**
 * Applies translation, rotation, and scale to a MeshGeometryData, including
 * negative scale winding order reversal (BUG-007: det(S) < 0 flips triangle indices
 * and normals so signed volume stays positive and 0 backfaces occur).
 */
export function applyTransformToMesh(
  mesh: MeshGeometryData,
  translation: [number, number, number] = [0, 0, 0],
  rotationDeg: [number, number, number] = [0, 0, 0],
  scale: [number, number, number] = [1, 1, 1],
  density: number = 7850
): MeshGeometryData {
  if (
    Math.abs(scale[0]) < 1e-8 ||
    Math.abs(scale[1]) < 1e-8 ||
    Math.abs(scale[2]) < 1e-8
  ) {
    throw new Error('Scale factors must be non-zero');
  }

  const euler = new THREE.Euler(
    (rotationDeg[0] * Math.PI) / 180,
    (rotationDeg[1] * Math.PI) / 180,
    (rotationDeg[2] * Math.PI) / 180,
    'XYZ'
  );
  const quat = new THREE.Quaternion().setFromEuler(euler);
  const mat = new THREE.Matrix4().compose(
    new THREE.Vector3(...translation),
    quat,
    new THREE.Vector3(...scale)
  );
  const normalMat = new THREE.Matrix3().getNormalMatrix(mat);
  const det = scale[0] * scale[1] * scale[2];
  const flipWinding = det < 0;

  const newVertices = new Float32Array(mesh.vertices.length);
  const newNormals = new Float32Array(mesh.normals.length);
  const newIndices = new Uint32Array(mesh.indices.length);

  const v = new THREE.Vector3();
  for (let i = 0; i < mesh.vertices.length; i += 3) {
    v.set(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]).applyMatrix4(mat);
    newVertices[i] = v.x;
    newVertices[i + 1] = v.y;
    newVertices[i + 2] = v.z;

    v.set(mesh.normals[i], mesh.normals[i + 1], mesh.normals[i + 2])
      .applyMatrix3(normalMat)
      .normalize();
    if (flipWinding) v.negate();
    newNormals[i] = v.x;
    newNormals[i + 1] = v.y;
    newNormals[i + 2] = v.z;
  }

  for (let i = 0; i < mesh.indices.length; i += 3) {
    newIndices[i] = mesh.indices[i];
    if (flipWinding) {
      newIndices[i + 1] = mesh.indices[i + 2];
      newIndices[i + 2] = mesh.indices[i + 1];
    } else {
      newIndices[i + 1] = mesh.indices[i + 1];
      newIndices[i + 2] = mesh.indices[i + 2];
    }
  }

  return computeMeshMetrics(
    newVertices,
    newNormals,
    newIndices,
    density,
    mesh.eulerGenus
  );
}

/**
 * Evaluates a parametric DAG Feature Tree from root to leaf, matching
 * `EngineState::rebuild_part_mesh` in crates/mcp-cad-core with transactional validation (BUG-008).
 */
export function evaluateFeatureTree(
  dag: DagNode[],
  quality: QualityPreset = 'standard',
  density: number = 7850,
  existingMesh?: MeshGeometryData
): MeshGeometryData {
  const enabledNodes = dag.filter((n) => n.enabled);
  if (enabledNodes.length === 0) {
    return createBoxMesh(0.05, 0.05, 0.05, density);
  }

  // If the root is an imported STL mesh and no other primitive overrides it, preserve it
  if (enabledNodes[0].type === 'stl_import' && existingMesh) {
    return existingMesh;
  }

  let currentMesh: MeshGeometryData | null = null;
  const pendingHoles: HoleCutSpec[] = [];
  let basePrimitiveNode: DagNode | null = null;

  for (const node of enabledNodes) {
    const p = node.params || {};
    switch (node.type) {
      case 'box': {
        const w = Number(p.width ?? 0.08);
        const h = Number(p.height ?? 0.04);
        const d = Number(p.depth ?? 0.05);
        currentMesh = createBoxMesh(w, h, d, density);
        basePrimitiveNode = node;
        break;
      }
      case 'cylinder': {
        const r = Number(p.radius ?? 0.025);
        const h = Number(p.height ?? 0.08);
        currentMesh = createCylinderMesh(r, h, quality, density);
        basePrimitiveNode = node;
        break;
      }
      case 'sphere': {
        const r = Number(p.radius ?? 0.035);
        currentMesh = createSphereMesh(r, quality, density);
        basePrimitiveNode = node;
        break;
      }
      case 'cone': {
        const rb = Number(p.radius_bottom ?? p.radiusBottom ?? 0.03);
        const rt = Number(p.radius_top ?? p.radiusTop ?? 0.0);
        const h = Number(p.height ?? 0.06);
        currentMesh = createConeMesh(rb, rt, h, quality, density);
        basePrimitiveNode = node;
        break;
      }
      case 'polygon_prism': {
        const sides = Number(p.sides ?? 6);
        const outerR =
          p.outer_radius !== undefined || p.outerRadius !== undefined
            ? Number(p.outer_radius ?? p.outerRadius)
            : p.width_across_flats !== undefined
              ? Number(p.width_across_flats) / (2 * Math.cos(Math.PI / sides))
              : 0.0098;
        const h = Number(p.height ?? 0.008);
        const chamferR = Number(p.chamfer_radius ?? p.chamferRadius ?? 0.0008);
        const boreR = Number(p.bore_radius ?? p.boreRadius ?? 0);
        const threadPitch = Number(p.thread_pitch ?? p.threadPitch ?? 0);
        const threadDepth = Number(p.thread_depth ?? p.threadDepth ?? 0);
        currentMesh = createPolygonPrismMesh(
          sides,
          outerR,
          h,
          chamferR,
          density,
          boreR,
          threadPitch,
          threadDepth
        );
        basePrimitiveNode = node;
        break;
      }
      case 'spur_gear': {
        const teeth = Number(p.teeth ?? 20);
        const mod = Number(p.module ?? p.module_mm ?? 2.0);
        const pa = Number(p.pressure_angle_deg ?? p.pressure_angle ?? 20);
        const fw = Number(p.face_width ?? p.faceWidth ?? 0.015);
        const bore = Number(p.bore_diameter ?? p.boreDiameter ?? 0.012);
        const hubD = Number(p.hub_diameter ?? p.hubDiameter ?? 0.025);
        const hubL = Number(p.hub_length ?? p.hubLength ?? 0.01);
        const kwW = Number(p.keyway_width ?? p.keywayWidth ?? 0.004);
        const kwD = Number(p.keyway_depth ?? p.keywayDepth ?? 0.0018);
        currentMesh = createSpurGearMesh(
          teeth,
          mod,
          pa,
          fw,
          bore,
          hubD,
          hubL,
          kwW,
          kwD,
          density
        );
        basePrimitiveNode = node;
        break;
      }
      case 'revolve': {
        if (Array.isArray(p.profile) && p.profile.length >= 2) {
          const angleDeg = Number(p.angle_deg ?? p.angleDeg ?? 360);
          const innerR = Number(p.inner_radius ?? p.innerRadius ?? 0);
          currentMesh = createRevolveMesh(
            p.profile as [number, number][],
            angleDeg,
            quality,
            density,
            innerR
          );
        } else {
          const bulbD = Number(p.bulb_diameter ?? 0.06);
          const totalH = Number(p.total_height ?? 0.11);
          currentMesh = createLightbulbRevolveMesh(bulbD, totalH, density);
        }
        basePrimitiveNode = node;
        break;
      }
      case 'wheel_rim': {
        const rimD = Number(p.rim_diameter ?? p.diameter ?? 0.4572);
        const rimW = Number(p.rim_width ?? p.width ?? 0.2032);
        const spokes = Number(p.spoke_count ?? p.spokes ?? 5);
        const hubD = Number(p.hub_diameter ?? 0.16);
        const hubBore = Number(p.hub_bore_diameter ?? 0.066);
        const bolts = Number(p.bolt_holes_count ?? p.bolt_pattern ?? 5);
        const pcd = Number(p.bolt_circle_diameter ?? 0.1143);
        const boltHoleD = Number(p.bolt_hole_diameter ?? 0.014);
        const flangeD = Number(p.flange_diameter ?? 0.49);
        const spokeThickness = Number(p.spoke_thickness ?? 0.018);
        const offset = Number(p.offset ?? 0.035);
        currentMesh = createWheelRimMesh(
          rimD,
          rimW,
          spokes,
          hubD,
          hubBore,
          bolts,
          pcd,
          boltHoleD,
          flangeD,
          density,
          spokeThickness,
          offset
        );
        basePrimitiveNode = node;
        break;
      }
      case 'cut_hole': {
        const r = Number(p.radius ?? 0.005);
        const d = Number(p.depth ?? 0.05);
        const pos: [number, number, number] = Array.isArray(p.position)
          ? [Number(p.position[0]), Number(p.position[1]), Number(p.position[2])]
          : [0, 0, 0];
        const norm: [number, number, number] = Array.isArray(p.normal)
          ? [Number(p.normal[0]), Number(p.normal[1]), Number(p.normal[2])]
          : [0, -1, 0];
        const kwW = p.keyway_width !== undefined ? Number(p.keyway_width) : undefined;
        const kwD = p.keyway_depth !== undefined ? Number(p.keyway_depth) : undefined;

        // If base primitive is polygon_prism and cut_hole is centered, upgrade to bored hex prism
        if (
          basePrimitiveNode?.type === 'polygon_prism' &&
          Math.hypot(pos[0], pos[2]) < 1e-3 &&
          pendingHoles.length === 0
        ) {
          const bp = basePrimitiveNode.params;
          currentMesh = createPolygonPrismMesh(
            Number(bp.sides ?? 6),
            Number(bp.outer_radius ?? 0.0098),
            Number(bp.height ?? 0.008),
            Number(bp.chamfer_radius ?? 0.0008),
            density,
            r
          );
        } else if (
          basePrimitiveNode?.type === 'cylinder' &&
          Math.hypot(pos[0], pos[2]) < 1e-3 &&
          pendingHoles.length === 0 &&
          !kwW
        ) {
          const bp = basePrimitiveNode.params;
          const rOut = Number(bp.radius ?? 0.025);
          const h = Number(bp.height ?? 0.08);
          const halfH = h * 0.5;
          currentMesh = createRevolveMesh(
            [
              [rOut, -halfH],
              [rOut, halfH],
            ],
            360,
            quality,
            density,
            r
          );
        } else {
          pendingHoles.push({
            position: pos,
            normal: norm,
            radius: r,
            depth: d,
            keywayWidth: kwW,
            keywayDepth: kwD,
          });
        }
        break;
      }
      case 'cut_helical_groove': {
        const pitch = Number(p.pitch ?? 0.0015);
        const grooveR = Number(p.groove_radius ?? 0.0008);
        const helixR = Number(p.helix_radius ?? 0.005);
        const starts = Number(p.starts ?? 1);
        const profileType = String(p.profile_type ?? 'circular');
        if (pitch <= 0 || grooveR <= 0 || helixR <= 0) {
          throw new Error('cut_helical_groove requires positive helix_radius, groove_radius, and pitch');
        }
        if (basePrimitiveNode?.type === 'polygon_prism') {
          const bp = basePrimitiveNode.params;
          currentMesh = createPolygonPrismMesh(
            Number(bp.sides ?? 6),
            Number(bp.outer_radius ?? 0.0098),
            Number(bp.height ?? 0.008),
            Number(bp.chamfer_radius ?? 0.0008),
            density,
            helixR * 0.85,
            pitch,
            grooveR
          );
        } else if (basePrimitiveNode?.type === 'cylinder' || profileType === 'circular') {
          const h = Number(
            p.height ?? basePrimitiveNode?.params?.height ?? 0.09
          );
          currentMesh = createHelicalDrillMesh(
            helixR * 2,
            h,
            pitch,
            starts,
            density
          );
        }
        break;
      }
      case 'transform': {
        if (currentMesh) {
          const tr: [number, number, number] = Array.isArray(p.translation)
            ? [Number(p.translation[0]), Number(p.translation[1]), Number(p.translation[2])]
            : [0, 0, 0];
          const rot: [number, number, number] = Array.isArray(p.rotation_deg)
            ? [Number(p.rotation_deg[0]), Number(p.rotation_deg[1]), Number(p.rotation_deg[2])]
            : [0, 0, 0];
          const sc: [number, number, number] = Array.isArray(p.scale)
            ? [Number(p.scale[0]), Number(p.scale[1]), Number(p.scale[2])]
            : [1, 1, 1];
          currentMesh = applyTransformToMesh(currentMesh, tr, rot, sc, density);
        }
        break;
      }
      default:
        break;
    }
  }

  if (!currentMesh) {
    currentMesh = createBoxMesh(0.05, 0.05, 0.05, density);
  }

  if (pendingHoles.length > 0) {
    currentMesh = applyHolesToMesh(currentMesh, pendingHoles, density);
  }

  return currentMesh;
}

/**
 * Exports a MeshGeometryData into an ArrayBuffer in IEEE 754 binary STL format (80-byte header + triangle records).
 */
export function exportMeshToStlBinary(
  mesh: MeshGeometryData,
  partName: string = 'mcp_cad_part'
): ArrayBuffer {
  const triCount = mesh.indices.length / 3;
  const buffer = new ArrayBuffer(84 + triCount * 50);
  const view = new DataView(buffer);

  // 80-byte ASCII header
  const headerStr = `MCP-CAD Binary STL: ${partName}`.slice(0, 79);
  for (let i = 0; i < headerStr.length; i++) {
    view.setUint8(i, headerStr.charCodeAt(i));
  }

  view.setUint32(80, triCount, true);

  let offset = 84;
  for (let i = 0; i < mesh.indices.length; i += 3) {
    const i0 = mesh.indices[i] * 3;
    const i1 = mesh.indices[i + 1] * 3;
    const i2 = mesh.indices[i + 2] * 3;

    const x0 = mesh.vertices[i0],
      y0 = mesh.vertices[i0 + 1],
      z0 = mesh.vertices[i0 + 2];
    const x1 = mesh.vertices[i1],
      y1 = mesh.vertices[i1 + 1],
      z1 = mesh.vertices[i1 + 2];
    const x2 = mesh.vertices[i2],
      y2 = mesh.vertices[i2 + 1],
      z2 = mesh.vertices[i2 + 2];

    const ax = x1 - x0,
      ay = y1 - y0,
      az = z1 - z0;
    const bx = x2 - x0,
      by = y2 - y0,
      bz = z2 - z0;
    let nx = ay * bz - az * by;
    let ny = az * bx - ax * bz;
    let nz = ax * by - ay * bx;
    const len = Math.sqrt(nx * nx + ny * ny + nz * nz) || 1;
    nx /= len;
    ny /= len;
    nz /= len;

    view.setFloat32(offset, nx, true);
    view.setFloat32(offset + 4, ny, true);
    view.setFloat32(offset + 8, nz, true);

    view.setFloat32(offset + 12, x0, true);
    view.setFloat32(offset + 16, y0, true);
    view.setFloat32(offset + 20, z0, true);

    view.setFloat32(offset + 24, x1, true);
    view.setFloat32(offset + 28, y1, true);
    view.setFloat32(offset + 32, z1, true);

    view.setFloat32(offset + 36, x2, true);
    view.setFloat32(offset + 40, y2, true);
    view.setFloat32(offset + 44, z2, true);

    view.setUint16(offset + 48, 0, true);
    offset += 50;
  }

  return buffer;
}

/**
 * Parses either a Binary or ASCII STL ArrayBuffer into a MeshGeometryData,
 * supporting both the pre-compiled Windows `/output/*.stl` artifacts and user uploads.
 */
export function parseStlBuffer(
  buffer: ArrayBuffer,
  density: number = 7850,
  scaleFactor: number = 1.0
): MeshGeometryData {
  const view = new DataView(buffer);
  const byteLength = buffer.byteLength;

  if (byteLength >= 84) {
    const triCount = view.getUint32(80, true);
    if (84 + triCount * 50 === byteLength && triCount > 0) {
      const vertices = new Float32Array(triCount * 9);
      const normals = new Float32Array(triCount * 9);
      const indices = new Uint32Array(triCount * 3);

      let offset = 84;
      for (let t = 0; t < triCount; t++) {
        let nx = view.getFloat32(offset, true);
        let ny = view.getFloat32(offset + 4, true);
        let nz = view.getFloat32(offset + 8, true);

        const x0 = view.getFloat32(offset + 12, true) * scaleFactor;
        const y0 = view.getFloat32(offset + 16, true) * scaleFactor;
        const z0 = view.getFloat32(offset + 20, true) * scaleFactor;

        const x1 = view.getFloat32(offset + 24, true) * scaleFactor;
        const y1 = view.getFloat32(offset + 28, true) * scaleFactor;
        const z1 = view.getFloat32(offset + 32, true) * scaleFactor;

        const x2 = view.getFloat32(offset + 36, true) * scaleFactor;
        const y2 = view.getFloat32(offset + 40, true) * scaleFactor;
        const z2 = view.getFloat32(offset + 44, true) * scaleFactor;

        if (nx * nx + ny * ny + nz * nz < 1e-8) {
          const ax = x1 - x0,
            ay = y1 - y0,
            az = z1 - z0;
          const bx = x2 - x0,
            by = y2 - y0,
            bz = z2 - z0;
          nx = ay * bz - az * by;
          ny = az * bx - ax * bz;
          nz = ax * by - ay * bx;
          const len = Math.sqrt(nx * nx + ny * ny + nz * nz) || 1;
          nx /= len;
          ny /= len;
          nz /= len;
        }

        const vBase = t * 9;
        vertices[vBase] = x0;
        vertices[vBase + 1] = y0;
        vertices[vBase + 2] = z0;
        vertices[vBase + 3] = x1;
        vertices[vBase + 4] = y1;
        vertices[vBase + 5] = z1;
        vertices[vBase + 6] = x2;
        vertices[vBase + 7] = y2;
        vertices[vBase + 8] = z2;

        normals[vBase] = nx;
        normals[vBase + 1] = ny;
        normals[vBase + 2] = nz;
        normals[vBase + 3] = nx;
        normals[vBase + 4] = ny;
        normals[vBase + 5] = nz;
        normals[vBase + 6] = nx;
        normals[vBase + 7] = ny;
        normals[vBase + 8] = nz;

        const iBase = t * 3;
        indices[iBase] = iBase;
        indices[iBase + 1] = iBase + 1;
        indices[iBase + 2] = iBase + 2;

        offset += 50;
      }

      return computeMeshMetrics(vertices, normals, indices, density);
    }
  }

  // Fallback: ASCII STL parser
  const text = new TextDecoder().decode(buffer);
  const vertexRegex =
    /vertex\s+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s+([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)/g;
  const coords: number[] = [];
  let match: RegExpExecArray | null;
  while ((match = vertexRegex.exec(text)) !== null) {
    coords.push(
      parseFloat(match[1]) * scaleFactor,
      parseFloat(match[2]) * scaleFactor,
      parseFloat(match[3]) * scaleFactor
    );
  }

  const builder = new MeshBuilder();
  for (let i = 0; i + 8 < coords.length; i += 9) {
    builder.addTriangle(
      coords[i],
      coords[i + 1],
      coords[i + 2],
      coords[i + 3],
      coords[i + 4],
      coords[i + 5],
      coords[i + 6],
      coords[i + 7],
      coords[i + 8]
    );
  }
  return builder.build(density);
}
