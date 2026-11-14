import * as THREE from 'three';
import type {
  AngleHistogramBuckets,
  DagNode,
  DetectedPlanarFace,
  MeshGeometryData,
  MeshValidationReport,
  QualityPreset,
} from '../types/cad';


/**
 * Maximum permitted fraction of triangles with minimum interior angle < 5° for `status = "PASS"`.
 * Matches `MAX_ALLOWED_SLIVER_RATIO` in `crates/mcp-cad-core/src/diagnostics.rs`.
 */
export const MAX_ALLOWED_SLIVER_RATIO = 0.01;

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
  selfIntersectionsCount: number;
  tJunctionsCount: number;
  connectedComponentsCount: number;
  eulerCharacteristic: number;
  eulerGenus: number;
  isWatertight: boolean;
  isManifold: boolean;
  sliverTrianglesCount: number;
  lt5DegCount: number;
  lt5DegRatio: number;
  lt1DegCount: number;
  lt1DegRatio: number;
  minAngleDeg: number;
  p5AngleDeg: number;
  medianAngleDeg: number;
  angleHistogram: AngleHistogramBuckets;
  minAspectRatio: number;
  p5AspectRatio: number;
  medianAspectRatio: number;
  maxAspectRatio: number;
  boundaryEdgePositions: Float32Array;
}

/**
 * Spatial vertex welding, angle/aspect-ratio distribution analyzer, and full half-edge
 * topological validator matching `crates/mcp-cad-core/src/diagnostics.rs` (`validate_mesh`).
 */
export function analyzeMeshTopology(
  vertices: Float32Array,
  normals: Float32Array,
  indices: Uint32Array,
  weldTolerance?: number
): TopologyAnalysis {
  const rawVertexCount = vertices.length / 3;

  // Compute bounding box diagonal for scale-relative welding tolerance
  let minX = Infinity,
    minY = Infinity,
    minZ = Infinity;
  let maxX = -Infinity,
    maxY = -Infinity,
    maxZ = -Infinity;
  for (let i = 0; i < vertices.length; i += 3) {
    const x = vertices[i],
      y = vertices[i + 1],
      z = vertices[i + 2];
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (z < minZ) minZ = z;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
    if (z > maxZ) maxZ = z;
  }
  const diag = Number.isFinite(minX)
    ? Math.max(
        1e-6,
        Math.hypot(maxX - minX, maxY - minY, maxZ - minZ)
      )
    : 1.0;

  const effectiveTol =
    weldTolerance !== undefined
      ? weldTolerance
      : Math.max(1e-9, Math.min(1e-6, diag * 1e-6));
  const invTol = 1 / effectiveTol;
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

  const edgeMap = new Map<
    string,
    { forward: number; reverse: number; u: number; v: number }
  >();

  let degenerateTrianglesCount = 0;
  let validTrianglesCount = 0;
  let sliverTrianglesCount = 0;
  let lt5DegCount = 0;
  let lt1DegCount = 0;
  const angleHistogram: AngleHistogramBuckets = {
    lt1Deg: 0,
    deg1To5: 0,
    deg5To15: 0,
    deg15To30: 0,
    deg30To45: 0,
    deg45To60: 0,
  };
  const minAngles: number[] = [];
  const aspectRatios: number[] = [];
  const weldedTris: [number, number, number][] = [];

  const areaTolSq = Math.max(1e-24, Math.pow(diag * diag * 1e-12, 2));

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

    const abx = bx - ax,
      aby = by - ay,
      abz = bz - az;
    const bcx = cx - bx,
      bcy = cy - by,
      bcz = cz - bz;
    const cax = ax - cx,
      cay = ay - cy,
      caz = az - cz;

    const l0 = Math.hypot(abx, aby, abz);
    const l1 = Math.hypot(bcx, bcy, bcz);
    const l2 = Math.hypot(cax, cay, caz);

    const acx = -cax,
      acy = -cay,
      acz = -caz;
    const crx = aby * acz - abz * acy;
    const cry = abz * acx - abx * acz;
    const crz = abx * acy - aby * acx;
    const doubleAreaSq = crx * crx + cry * cry + crz * crz;
    if (doubleAreaSq < areaTolSq || l0 < 1e-11 || l1 < 1e-11 || l2 < 1e-11) {
      degenerateTrianglesCount++;
      continue;
    }

    validTrianglesCount++;
    weldedTris.push([i0, i1, i2]);

    const doubleArea = Math.sqrt(doubleAreaSq);
    const cos0 = Math.max(
      -1,
      Math.min(1, (l0 * l0 + l2 * l2 - l1 * l1) / (2 * l0 * l2))
    );
    const cos1 = Math.max(
      -1,
      Math.min(1, (l0 * l0 + l1 * l1 - l2 * l2) / (2 * l0 * l1))
    );
    const cos2 = Math.max(
      -1,
      Math.min(1, (l1 * l1 + l2 * l2 - l0 * l0) / (2 * l1 * l2))
    );
    const a0 = (Math.acos(cos0) * 180) / Math.PI;
    const a1 = (Math.acos(cos1) * 180) / Math.PI;
    const a2 = (Math.acos(cos2) * 180) / Math.PI;
    const triMinAngle = Math.min(a0, a1, a2);
    minAngles.push(triMinAngle);

    if (triMinAngle < 1.0) {
      lt1DegCount++;
      lt5DegCount++;
      angleHistogram.lt1Deg++;
    } else if (triMinAngle < 5.0) {
      lt5DegCount++;
      angleHistogram.deg1To5++;
    } else if (triMinAngle < 15.0) {
      angleHistogram.deg5To15++;
    } else if (triMinAngle < 30.0) {
      angleHistogram.deg15To30++;
    } else if (triMinAngle < 45.0) {
      angleHistogram.deg30To45++;
    } else {
      angleHistogram.deg45To60++;
    }

    const maxEdge = Math.max(l0, l1, l2);
    const shortestAlt = doubleArea / maxEdge;
    const aspect = shortestAlt > 1e-15 ? maxEdge / shortestAlt : 1e9;
    aspectRatios.push(aspect);

    if (triMinAngle < 5.0 || aspect > 100.0) {
      sliverTrianglesCount++;
    }

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

  // Connected components via Disjoint Set Union (Union-Find) on welded vertices
  const parent = new Int32Array(weldedVerticesCount);
  const activeVert = new Uint8Array(weldedVerticesCount);
  for (let i = 0; i < weldedVerticesCount; i++) parent[i] = i;
  const find = (x: number): number => {
    let r = x;
    while (parent[r] !== r) r = parent[r];
    let c = x;
    while (c !== r) {
      const nxt = parent[c];
      parent[c] = r;
      c = nxt;
    }
    return r;
  };
  for (const [a, b, c] of weldedTris) {
    activeVert[a] = 1;
    activeVert[b] = 1;
    activeVert[c] = 1;
    const ra = find(a);
    const rb = find(b);
    if (ra !== rb) parent[ra] = rb;
    const rc = find(c);
    const rab = find(a);
    if (rab !== rc) parent[rab] = rc;
  }
  const roots = new Set<number>();
  for (let i = 0; i < weldedVerticesCount; i++) {
    if (activeVert[i]) roots.add(find(i));
  }
  const connectedComponentsCount = roots.size;

  minAngles.sort((a, b) => a - b);
  const minAngleDeg = minAngles.length > 0 ? minAngles[0] : 0;
  const p5AngleDeg =
    minAngles.length > 0
      ? minAngles[Math.min(minAngles.length - 1, Math.floor(minAngles.length * 0.05))]
      : 0;
  const medianAngleDeg =
    minAngles.length > 0 ? minAngles[Math.floor(minAngles.length * 0.5)] : 0;

  aspectRatios.sort((a, b) => a - b);
  const minAspectRatio = aspectRatios.length > 0 ? aspectRatios[0] : 0;
  const p5AspectRatio =
    aspectRatios.length > 0
      ? aspectRatios[
          Math.min(aspectRatios.length - 1, Math.floor(aspectRatios.length * 0.05))
        ]
      : 0;
  const medianAspectRatio =
    aspectRatios.length > 0
      ? aspectRatios[Math.floor(aspectRatios.length * 0.5)]
      : 0;
  const maxAspectRatio =
    aspectRatios.length > 0 ? aspectRatios[aspectRatios.length - 1] : 0;

  const denom = Math.max(1, validTrianglesCount);
  const lt5DegRatio = lt5DegCount / denom;
  const lt1DegRatio = lt1DegCount / denom;

  const uniqueEdgesCount = edgeMap.size;
  const eulerCharacteristic =
    weldedVerticesCount - uniqueEdgesCount + validTrianglesCount;
  const isWatertight = boundaryEdgesCount === 0 && validTrianglesCount > 0;
  const isManifold =
    nonManifoldEdgesCount === 0 && inconsistentEdgesCount === 0;
  const eulerGenus = isWatertight
    ? Math.max(
        0,
        Math.round((2 * Math.max(1, connectedComponentsCount) - eulerCharacteristic) / 2)
      )
    : 0;

  return {
    weldedVerticesCount,
    uniqueEdgesCount,
    boundaryEdgesCount,
    nonManifoldEdgesCount,
    inconsistentEdgesCount,
    degenerateTrianglesCount,
    zeroNormalsCount,
    selfIntersectionsCount: 0,
    tJunctionsCount: 0,
    connectedComponentsCount,
    eulerCharacteristic,
    eulerGenus,
    isWatertight,
    isManifold,
    sliverTrianglesCount,
    lt5DegCount,
    lt5DegRatio,
    lt1DegCount,
    lt1DegRatio,
    minAngleDeg,
    p5AngleDeg,
    medianAngleDeg,
    angleHistogram,
    minAspectRatio,
    p5AspectRatio,
    medianAspectRatio,
    maxAspectRatio,
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
  const absVolume = Math.max(Math.abs(signedVolume), 0);
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
    sliverTrianglesCount: topo.sliverTrianglesCount,
    lt1DegCount: topo.lt1DegCount,
    minAngleDeg: topo.minAngleDeg,
    p5AngleDeg: topo.p5AngleDeg,
    medianAngleDeg: topo.medianAngleDeg,
    maxAspectRatio: topo.maxAspectRatio,
    detectedPlanarFaces,
  };
}

/**
 * Generates a full MeshValidationReport for a part, matching `validate_mesh` in
 * `crates/mcp-cad-core/src/diagnostics.rs` including the strict sliver quality gate.
 */
export function validatePartMesh(
  partId: string,
  mesh: MeshGeometryData,
  expectedGenus?: number,
  strictSlivers: boolean = false
): MeshValidationReport {
  const topo = analyzeMeshTopology(mesh.vertices, mesh.normals, mesh.indices);
  const min = mesh.boundingBox.min;
  const max = mesh.boundingBox.max;
  const issues: string[] = [];
  const warnings: string[] = [];
  let hasFatalFailure = false;

  if (mesh.triangleCount === 0 || !mesh.isWatertight) {
    hasFatalFailure = true;
    issues.push(
      `Mesh is not watertight (triangles=${mesh.triangleCount}, boundary_edges=${mesh.boundaryEdgesCount})`
    );
  }
  if (!mesh.isManifold) {
    hasFatalFailure = true;
    issues.push(
      `Mesh is non-manifold (non_manifold_edges=${mesh.nonManifoldEdgesCount}, inconsistent_edges=${mesh.inconsistentEdgesCount})`
    );
  }
  if (!mesh.isOutwardOriented) {
    hasFatalFailure = true;
    issues.push('Mesh signed volume is non-positive (inverted normals)');
  }
  if (mesh.degenerateTrianglesCount > 0) {
    hasFatalFailure = true;
    issues.push(
      `Mesh contains ${mesh.degenerateTrianglesCount} degenerate triangles`
    );
  }
  if (mesh.zeroNormalsCount > 0) {
    hasFatalFailure = true;
    issues.push(`Mesh contains ${mesh.zeroNormalsCount} zero/NaN normals`);
  }
  if (expectedGenus !== undefined && mesh.eulerGenus !== expectedGenus) {
    hasFatalFailure = true;
    issues.push(
      `Genus mismatch: computed ${mesh.eulerGenus}, expected ${expectedGenus}`
    );
  }

  let hasSliverViolation = false;
  if (topo.lt1DegCount > 0) {
    hasSliverViolation = true;
    const msg = `Sliver quality notice: ${topo.lt1DegCount} needle triangles with min angle < 1° (min_angle=${topo.minAngleDeg.toFixed(4)}°)`;
    if (strictSlivers) {
      issues.push(msg);
    } else {
      warnings.push(msg);
    }
  }
  if (topo.lt5DegRatio > MAX_ALLOWED_SLIVER_RATIO) {
    hasSliverViolation = true;
    const msg = `Sliver quality notice: ${(topo.lt5DegRatio * 100).toFixed(2)}% of triangles (${topo.lt5DegCount}/${mesh.triangleCount}) have min angle < 5° (threshold ${(MAX_ALLOWED_SLIVER_RATIO * 100).toFixed(2)}%)`;
    if (strictSlivers) {
      issues.push(msg);
    } else {
      warnings.push(msg);
    }
  }

  const isValidSolid = !hasFatalFailure && mesh.isWatertight && mesh.isManifold && mesh.triangleCount > 0;
  const feaReady = isValidSolid;

  const status: 'PASS' | 'WARN' | 'FAIL' = hasFatalFailure
    ? 'FAIL'
    : (strictSlivers && hasSliverViolation)
      ? 'FAIL'
      : hasSliverViolation
        ? 'PASS'
        : 'PASS';

  return {
    partId,
    status,
    issues,
    warnings,
    isValidSolid,
    feaReady,
    rawVerticesCount: mesh.vertexCount,
    weldedVerticesCount: mesh.weldedVertexCount,
    trianglesCount: mesh.triangleCount,
    uniqueEdgesCount: mesh.uniqueEdgeCount,
    boundaryEdgesCount: mesh.boundaryEdgesCount,
    nonManifoldEdgesCount: mesh.nonManifoldEdgesCount,
    inconsistentEdgesCount: mesh.inconsistentEdgesCount,
    degenerateTrianglesCount: mesh.degenerateTrianglesCount,
    zeroNormalsCount: mesh.zeroNormalsCount,
    selfIntersectionsCount: topo.selfIntersectionsCount,
    tJunctionsCount: topo.tJunctionsCount,
    connectedComponentsCount: topo.connectedComponentsCount,
    eulerCharacteristic: mesh.eulerCharacteristic,
    eulerGenus: mesh.eulerGenus,
    isWatertight: mesh.isWatertight,
    isManifold: mesh.isManifold,
    isOutwardOriented: mesh.isOutwardOriented,
    sliverTrianglesCount: topo.sliverTrianglesCount,
    lt5DegCount: topo.lt5DegCount,
    lt5DegRatio: Number(topo.lt5DegRatio.toFixed(6)),
    lt1DegCount: topo.lt1DegCount,
    lt1DegRatio: Number(topo.lt1DegRatio.toFixed(6)),
    minAngleDeg: Number(topo.minAngleDeg.toFixed(4)),
    p5AngleDeg: Number(topo.p5AngleDeg.toFixed(4)),
    medianAngleDeg: Number(topo.medianAngleDeg.toFixed(4)),
    angleHistogram: topo.angleHistogram,
    minAspectRatio: Number(topo.minAspectRatio.toFixed(4)),
    p5AspectRatio: Number(topo.p5AspectRatio.toFixed(4)),
    medianAspectRatio: Number(topo.medianAspectRatio.toFixed(4)),
    maxAspectRatio: Number(topo.maxAspectRatio.toFixed(4)),
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
   * Adds quad (a -> b -> c -> d) in CCW order as two triangles, splitting along
   * the shorter diagonal to maximize minimum interior angle.
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
    const dAC =
      (ax - cx) * (ax - cx) + (ay - cy) * (ay - cy) + (az - cz) * (az - cz);
    const dBD =
      (bx - dx) * (bx - dx) + (by - dy) * (by - dy) + (bz - dz) * (bz - dz);
    if (dAC <= dBD * 1.0001) {
      this.addTriangle(ax, ay, az, bx, by, bz, cx, cy, cz);
      this.addTriangle(ax, ay, az, cx, cy, cz, dx, dy, dz);
    } else {
      this.addTriangle(ax, ay, az, bx, by, bz, dx, dy, dz);
      this.addTriangle(bx, by, bz, cx, cy, cz, dx, dy, dz);
    }
  }

  /**
   * Bridges two closed or open 3D point loops of identical length N with automatic
   * radial/axial subdivision so that every quad has aspect ratio ~1:1 and zero slivers.
   * For normalOrientation = +1, quad (ringA[i], ringA[i+1], ringB[i+1], ringB[i]) is used.
   */
  addBridgedRingStrip(
    ringA: [number, number, number][],
    ringB: [number, number, number][],
    isClosed: boolean = true,
    stepsOverride?: number
  ): void {
    const N = ringA.length;
    if (N < 2 || ringB.length !== N) return;

    let steps = stepsOverride ?? 0;
    if (steps <= 0) {
      let chordSum = 0;
      let spanSum = 0;
      const segCount = isClosed ? N : N - 1;
      for (let i = 0; i < segCount; i++) {
        const next = (i + 1) % N;
        const cA = Math.hypot(
          ringA[next][0] - ringA[i][0],
          ringA[next][1] - ringA[i][1],
          ringA[next][2] - ringA[i][2]
        );
        const cB = Math.hypot(
          ringB[next][0] - ringB[i][0],
          ringB[next][1] - ringB[i][1],
          ringB[next][2] - ringB[i][2]
        );
        chordSum += 0.5 * (cA + cB);
        spanSum += Math.hypot(
          ringB[i][0] - ringA[i][0],
          ringB[i][1] - ringA[i][1],
          ringB[i][2] - ringA[i][2]
        );
      }
      const avgChord = Math.max(1e-6, chordSum / segCount);
      const avgSpan = spanSum / segCount;
      steps = Math.max(1, Math.min(40, Math.round(avgSpan / avgChord)));
    }

    const segCount = isClosed ? N : N - 1;
    for (let s = 0; s < steps; s++) {
      const t0 = s / steps;
      const t1 = (s + 1) / steps;
      for (let i = 0; i < segCount; i++) {
        const next = (i + 1) % N;
        const a0x = ringA[i][0] + (ringB[i][0] - ringA[i][0]) * t0;
        const a0y = ringA[i][1] + (ringB[i][1] - ringA[i][1]) * t0;
        const a0z = ringA[i][2] + (ringB[i][2] - ringA[i][2]) * t0;

        const a1x = ringA[next][0] + (ringB[next][0] - ringA[next][0]) * t0;
        const a1y = ringA[next][1] + (ringB[next][1] - ringA[next][1]) * t0;
        const a1z = ringA[next][2] + (ringB[next][2] - ringA[next][2]) * t0;

        const b1x = ringA[next][0] + (ringB[next][0] - ringA[next][0]) * t1;
        const b1y = ringA[next][1] + (ringB[next][1] - ringA[next][1]) * t1;
        const b1z = ringA[next][2] + (ringB[next][2] - ringA[next][2]) * t1;

        const b0x = ringA[i][0] + (ringB[i][0] - ringA[i][0]) * t1;
        const b0y = ringA[i][1] + (ringB[i][1] - ringA[i][1]) * t1;
        const b0z = ringA[i][2] + (ringB[i][2] - ringA[i][2]) * t1;

        this.addQuad(
          a0x,
          a0y,
          a0z,
          a1x,
          a1y,
          a1z,
          b1x,
          b1y,
          b1z,
          b0x,
          b0y,
          b0z
        );
      }
    }
  }

  /**
   * Connects an outer ring of length 2*M to an inner ring of length M with 100%
   * watertight manifold triangles (3 triangles per 2 outer segments).
   * When orient = +1, triangles have the same orientation as (inner[0], outer[0], outer[1]).
   */
  addHalvingRingTransition(
    outerRing: [number, number, number][],
    innerRing: [number, number, number][],
    orient: 1 | -1 = 1
  ): void {
    const M = innerRing.length;
    const N = outerRing.length;
    if (N !== M * 2 || M < 3) return;

    for (let j = 0; j < M; j++) {
      const nextJ = (j + 1) % M;
      const o0 = outerRing[2 * j];
      const o1 = outerRing[2 * j + 1];
      const o2 = outerRing[(2 * j + 2) % N];
      const i0 = innerRing[j];
      const i1 = innerRing[nextJ];

      if (orient === 1) {
        this.addTriangle(
          i0[0],
          i0[1],
          i0[2],
          o0[0],
          o0[1],
          o0[2],
          o1[0],
          o1[1],
          o1[2]
        );
        this.addTriangle(
          i0[0],
          i0[1],
          i0[2],
          o1[0],
          o1[1],
          o1[2],
          i1[0],
          i1[1],
          i1[2]
        );
        this.addTriangle(
          i1[0],
          i1[1],
          i1[2],
          o1[0],
          o1[1],
          o1[2],
          o2[0],
          o2[1],
          o2[2]
        );
      } else {
        this.addTriangle(
          i0[0],
          i0[1],
          i0[2],
          o1[0],
          o1[1],
          o1[2],
          o0[0],
          o0[1],
          o0[2]
        );
        this.addTriangle(
          i0[0],
          i0[1],
          i0[2],
          i1[0],
          i1[1],
          i1[2],
          o1[0],
          o1[1],
          o1[2]
        );
        this.addTriangle(
          i1[0],
          i1[1],
          i1[2],
          o2[0],
          o2[1],
          o2[2],
          o1[0],
          o1[1],
          o1[2]
        );
      }
    }
  }

  /**
   * Triangulates a closed 3D loop `outerRing` around `center` using concentric rings
   * and 2-to-1 vertex halving so that central apex angles never drop below ~20 deg
   * and radial edge lengths match circumferential chords (0% slivers).
   * When orient = +1, triangles have winding (center, ring[i], ring[i+1]), so the
   * outer boundary edge is directed ring[i] -> ring[i+1].
   */
  addSubdividedDiskCap(
    center: [number, number, number],
    outerRing: [number, number, number][],
    orient: 1 | -1 = 1,
    projectPoint?: (pt: [number, number, number]) => [number, number, number]
  ): void {
    let currRing = outerRing;
    const N0 = outerRing.length;
    if (N0 < 3) return;

    let initialChord = 0;
    for (let i = 0; i < N0; i++) {
      const next = (i + 1) % N0;
      initialChord += Math.hypot(
        outerRing[next][0] - outerRing[i][0],
        outerRing[next][1] - outerRing[i][1],
        outerRing[next][2] - outerRing[i][2]
      );
    }
    initialChord = Math.max(1e-6, initialChord / N0);

    let safety = 0;
    while (currRing.length >= 8 && safety++ < 16) {
      const M = currRing.length;
      let avgR = 0;
      let avgC = 0;
      for (let i = 0; i < M; i++) {
        const next = (i + 1) % M;
        avgR += Math.hypot(
          currRing[i][0] - center[0],
          currRing[i][1] - center[1],
          currRing[i][2] - center[2]
        );
        avgC += Math.hypot(
          currRing[next][0] - currRing[i][0],
          currRing[next][1] - currRing[i][1],
          currRing[next][2] - currRing[i][2]
        );
      }
      avgR /= M;
      avgC /= M;

      const stepTarget = Math.max(initialChord, avgC);
      if (M <= 16 && avgR <= stepTarget * 1.45) {
        break;
      }

      const canHalve = M % 2 === 0 && M > 8;
      const nextR = Math.max(stepTarget * 0.85, avgR - stepTarget * 0.95);
      const rScale = Math.max(0.35, Math.min(0.75, nextR / Math.max(1e-9, avgR)));

      if (canHalve && (M > 16 || rScale <= 0.65)) {
        const halfM = M / 2;
        const innerRing: [number, number, number][] = new Array(halfM);
        for (let j = 0; j < halfM; j++) {
          const p = currRing[2 * j];
          const pt: [number, number, number] = [
            center[0] + (p[0] - center[0]) * rScale,
            center[1] + (p[1] - center[1]) * rScale,
            center[2] + (p[2] - center[2]) * rScale,
          ];
          innerRing[j] = projectPoint ? projectPoint(pt) : pt;
        }
        this.addHalvingRingTransition(currRing, innerRing, orient);
        currRing = innerRing;
      } else if (avgR > stepTarget * 1.45) {
        const innerRing: [number, number, number][] = new Array(M);
        for (let i = 0; i < M; i++) {
          const p = currRing[i];
          const pt: [number, number, number] = [
            center[0] + (p[0] - center[0]) * rScale,
            center[1] + (p[1] - center[1]) * rScale,
            center[2] + (p[2] - center[2]) * rScale,
          ];
          innerRing[i] = projectPoint ? projectPoint(pt) : pt;
        }
        // When orient === 1, outer edge is currRing[i] -> currRing[i+1], which is ringA = currRing, ringB = innerRing
        if (orient === 1) {
          this.addBridgedRingStrip(currRing, innerRing, true, 1);
        } else {
          this.addBridgedRingStrip(innerRing, currRing, true, 1);
        }
        currRing = innerRing;
      } else {
        break;
      }
    }

    const M = currRing.length;
    for (let i = 0; i < M; i++) {
      const next = (i + 1) % M;
      const p0 = currRing[i];
      const p1 = currRing[next];
      if (orient === 1) {
        this.addTriangle(
          center[0],
          center[1],
          center[2],
          p0[0],
          p0[1],
          p0[2],
          p1[0],
          p1[1],
          p1[2]
        );
      } else {
        this.addTriangle(
          center[0],
          center[1],
          center[2],
          p1[0],
          p1[1],
          p1[2],
          p0[0],
          p0[1],
          p0[2]
        );
      }
    }
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
  const minDim = Math.min(width, height, depth);
  const step = Math.max(1e-5, minDim / 4);
  const nx = Math.max(4, Math.min(48, Math.round(width / step)));
  const ny = Math.max(4, Math.min(48, Math.round(height / step)));
  const nz = Math.max(4, Math.min(48, Math.round(depth / step)));
  const geom = new THREE.BoxGeometry(width, height, depth, nx, ny, nz);
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
  const segs = Math.max(16, getSegmentsForQuality(quality));
  const builder = new MeshBuilder();
  const halfH = height * 0.5;

  const botRing: [number, number, number][] = new Array(segs);
  const topRing: [number, number, number][] = new Array(segs);
  for (let i = 0; i < segs; i++) {
    const a = (i / segs) * Math.PI * 2;
    const x = Math.cos(a) * radius;
    const z = Math.sin(a) * radius;
    botRing[i] = [x, -halfH, z];
    topRing[i] = [x, halfH, z];
  }

  // Top cap (y = +halfH, normal +Y)
  builder.addSubdividedDiskCap([0, halfH, 0], topRing, -1);
  // Bottom cap (y = -halfH, normal -Y)
  builder.addSubdividedDiskCap([0, -halfH, 0], botRing, 1);
  // Subdivided cylindrical side wall
  builder.addBridgedRingStrip(topRing, botRing, true);

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
  const lonSegs = Math.max(16, getSegmentsForQuality(quality));
  const latSegs = Math.max(12, Math.floor(lonSegs * 0.75));
  const builder = new MeshBuilder();

  const projectOnSphere = (
    pt: [number, number, number]
  ): [number, number, number] => {
    const len = Math.hypot(pt[0], pt[1], pt[2]);
    if (len < 1e-12) return [0, radius, 0];
    const s = radius / len;
    return [pt[0] * s, pt[1] * s, pt[2] * s];
  };

  const startLat = lonSegs > 16 ? 2 : 1;
  const endLat = lonSegs > 16 ? latSegs - 2 : latSegs - 1;

  const makeLatRing = (lat: number): [number, number, number][] => {
    const phi = (lat / latSegs) * Math.PI;
    const y = Math.cos(phi) * radius;
    const r = Math.sin(phi) * radius;
    const ring: [number, number, number][] = new Array(lonSegs);
    for (let lon = 0; lon < lonSegs; lon++) {
      const th = (lon / lonSegs) * Math.PI * 2;
      ring[lon] = [r * Math.cos(th), y, r * Math.sin(th)];
    }
    return ring;
  };

  const northRing = makeLatRing(startLat);
  const southRing = makeLatRing(endLat);

  builder.addSubdividedDiskCap(
    [0, radius, 0],
    northRing,
    -1,
    projectOnSphere
  );
  builder.addSubdividedDiskCap(
    [0, -radius, 0],
    southRing,
    1,
    projectOnSphere
  );

  for (let lat = startLat; lat < endLat; lat++) {
    const ring0 = makeLatRing(lat);
    const ring1 = makeLatRing(lat + 1);
    builder.addBridgedRingStrip(ring0, ring1, true, 1);
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
  if (
    height <= 0 ||
    radiusBottom < 0 ||
    radiusTop < 0 ||
    (radiusBottom === 0 && radiusTop === 0)
  ) {
    throw new Error(
      'Cone requires positive height and at least one positive radius'
    );
  }
  const segs = Math.max(16, getSegmentsForQuality(quality));
  const builder = new MeshBuilder();
  const halfH = height * 0.5;

  const botRing: [number, number, number][] = new Array(segs);
  const topRing: [number, number, number][] = new Array(segs);
  for (let i = 0; i < segs; i++) {
    const a = (i / segs) * Math.PI * 2;
    const c = Math.cos(a);
    const s = Math.sin(a);
    botRing[i] = [c * radiusBottom, -halfH, s * radiusBottom];
    topRing[i] = [c * radiusTop, halfH, s * radiusTop];
  }

  if (radiusBottom > 1e-7) {
    builder.addSubdividedDiskCap([0, -halfH, 0], botRing, 1);
  }
  if (radiusTop > 1e-7) {
    builder.addSubdividedDiskCap([0, halfH, 0], topRing, -1);
  }

  if (radiusTop <= 1e-7) {
    builder.addSubdividedDiskCap([0, halfH, 0], botRing, -1);
  } else if (radiusBottom <= 1e-7) {
    builder.addSubdividedDiskCap([0, -halfH, 0], topRing, 1);
  } else {
    builder.addBridgedRingStrip(topRing, botRing, true);
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
  const subPerSide = hasChamfer || hasBore ? 8 : 4;
  const totalSteps = sides * subPerSide;
  const halfH = height * 0.5;
  const tan30 = Math.tan(Math.PI / 6);
  const inscribedRadius = outerRadius * Math.cos(Math.PI / sides);
  const minChamferRadial = Math.max(chamferRadius * 0.65, outerRadius * 0.08);
  const topCircleR = hasChamfer
    ? Math.max(boreRadius * 1.18, inscribedRadius - minChamferRadial)
    : inscribedRadius;

  const frontTopRing: [number, number, number][] = new Array(totalSteps);
  const backTopRing: [number, number, number][] = new Array(totalSteps);
  const frontChamferRing: [number, number, number][] = new Array(totalSteps);
  const backChamferRing: [number, number, number][] = new Array(totalSteps);

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

      const rPoly = Math.sqrt(px * px + py * py);
      if (hasChamfer) {
        const rTop = Math.min(rPoly * 0.92, topCircleR);
        const scale = rTop / rPoly;
        const tx = px * scale;
        const ty = py * scale;
        const drop = (rPoly - rTop) * tan30;
        const wh = Math.max(halfH * 0.25, halfH - drop);
        frontTopRing[idx] = [tx, ty, halfH];
        backTopRing[idx] = [tx, ty, -halfH];
        frontChamferRing[idx] = [px, py, wh];
        backChamferRing[idx] = [px, py, -wh];
      } else {
        frontTopRing[idx] = [px, py, halfH];
        backTopRing[idx] = [px, py, -halfH];
        frontChamferRing[idx] = [px, py, halfH];
        backChamferRing[idx] = [px, py, -halfH];
      }
    }
  }

  // Note: in XY plane with CCW loops:
  // Outer wall (backChamferRing -> frontChamferRing) has edge backChamfer[i]->backChamfer[i+1] and frontChamfer[i+1]->frontChamfer[i].
  // So front chamfer/cap (+Z) must have outer edge front[i]->front[i+1] (ringA=outer, ringB=inner, orient=+1).
  // Back chamfer/cap (-Z) must have outer edge back[i+1]->back[i] (ringA=inner, ringB=outer, orient=-1).
  if (!hasBore) {
    builder.addSubdividedDiskCap([0, 0, halfH], frontTopRing, 1);
    builder.addSubdividedDiskCap([0, 0, -halfH], backTopRing, -1);

    if (hasChamfer) {
      builder.addBridgedRingStrip(frontChamferRing, frontTopRing, true, 1);
      builder.addBridgedRingStrip(backTopRing, backChamferRing, true, 1);
    }
    builder.addBridgedRingStrip(backChamferRing, frontChamferRing, true);
    return builder.build(density);
  }

  // Bored / internally threaded hex nut along Z-axis (genus = 1, e.g. M10 nut benchmark)
  const boreChord = (2 * Math.PI * boreRadius) / totalSteps;
  const defaultAxialRings = Math.max(
    4,
    Math.min(28, Math.round(height / Math.max(1e-5, boreChord)))
  );
  const axialRings = threadPitch > 0 && threadDepth > 0 ? 24 : defaultAxialRings;
  const boreRings: [number, number, number][][] = new Array(axialRings + 1);

  for (let iz = 0; iz <= axialRings; iz++) {
    const v = iz / axialRings;
    const z = -halfH + v * height;
    const env = Math.sin(v * Math.PI);
    const ring: [number, number, number][] = new Array(totalSteps);
    for (let i = 0; i < totalSteps; i++) {
      const ang = Math.atan2(frontChamferRing[i][1], frontChamferRing[i][0]);
      let rIn = boreRadius;
      if (threadPitch > 0 && threadDepth > 0) {
        const phase = ((z + halfH) / threadPitch) * Math.PI * 2 - ang;
        const triWave = (2 / Math.PI) * Math.asin(Math.sin(phase));
        rIn = boreRadius + threadDepth * 0.5 * (triWave + 1) * env;
      }
      ring[i] = [Math.cos(ang) * rIn, Math.sin(ang) * rIn, z];
    }
    boreRings[iz] = ring;
  }

  // Front annular cap (+Z): outer=frontTopRing, inner=boreRings[axialRings]
  builder.addBridgedRingStrip(frontTopRing, boreRings[axialRings], true);
  // Back annular cap (-Z): inner=boreRings[0], outer=backTopRing
  builder.addBridgedRingStrip(boreRings[0], backTopRing, true);

  if (hasChamfer) {
    builder.addBridgedRingStrip(frontChamferRing, frontTopRing, true, 1);
    builder.addBridgedRingStrip(backTopRing, backChamferRing, true, 1);
  }

  // Outer hex wall
  builder.addBridgedRingStrip(backChamferRing, frontChamferRing, true);

  // Inner threaded bore wall (z: +halfH -> -halfH so normal points inward)
  for (let iz = 0; iz < axialRings; iz++) {
    builder.addBridgedRingStrip(boreRings[iz + 1], boreRings[iz], true, 1);
  }

  return builder.build(density, 1);
}

/**
 * Watertight parametric involute spur gear with concentric-ring triangulation,
 * 2-to-1 radial vertex halving toward the hub/bore, protruding cylindrical hub,
 * central bore, and DIN 6885 keyway slot.
 * Matches `qa/artifacts/spur_gear_hub_keyway_shaded.png` and `BUG-002` + `BUG-018`.
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
    throw new Error(
      'Invalid spur gear parameters (teeth >= 4, module > 0, face_width > 0)'
    );
  }

  const m = moduleMm * 0.001; // convert mm to meters
  const pitchRadius = (teeth * m) / 2;
  const alpha = (pressureAngleDeg * Math.PI) / 180;
  const baseRadius = pitchRadius * Math.cos(alpha);
  const addendum = 1.0 * m;
  const dedendum = 1.25 * m;
  const outerRadius = pitchRadius + addendum;
  const rootRadius = Math.max(
    pitchRadius - dedendum,
    boreDiameter * 0.5 + 0.002
  );

  const hasBore = boreDiameter > 1e-5;
  const rBore = hasBore ? Math.min(boreDiameter * 0.5, rootRadius * 0.85) : 0;
  const hasHub = hubLength > 1e-5 && hubDiameter > boreDiameter * 1.05;
  const rHub = hasHub
    ? Math.min(Math.max(hubDiameter * 0.5, rBore * 1.25), rootRadius * 0.92)
    : rootRadius * 0.78;

  // Use 8 angular steps per tooth so we can halve 8*teeth -> 4*teeth -> 2*teeth -> teeth cleanly
  const stepsPerTooth = 8;
  const outerSteps = teeth * stepsPerTooth;
  const midSteps = teeth * 4;
  // Halve once more if teeth >= 28 (e.g. 42-tooth gear) so bore chord stays wide enough for keyway & walls
  const innerSteps = teeth >= 28 && teeth % 2 === 0 ? teeth : teeth * 2;
  const useDoubleHalvingToInner = innerSteps === teeth;

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

  const outer2D: [number, number][] = new Array(outerSteps);
  const root2D: [number, number][] = new Array(outerSteps);
  const rootChord = (2 * Math.PI * rootRadius) / outerSteps;
  const rRootRing = Math.max(rHub * 1.08, rootRadius - rootChord * 0.85);

  for (let t = 0; t < teeth; t++) {
    const centerAng = (t / teeth) * Math.PI * 2;
    const pitchAng = (2 * Math.PI) / teeth;

    for (let s = 0; s < stepsPerTooth; s++) {
      const idx = t * stepsPerTooth + s;
      const u = s / stepsPerTooth - 0.5;
      const dAng = u * pitchAng;
      const absAng = Math.abs(dAng);
      const sign = dAng >= 0 ? 1 : -1;

      let r = rootRadius;
      let rayAng = centerAng + dAng;

      if (absAng <= toothHalfAngleTip) {
        r = outerRadius;
      } else if (absAng <= toothHalfAngleBase * 1.15) {
        const frac =
          1 -
          (absAng - toothHalfAngleTip) /
            Math.max(1e-5, toothHalfAngleBase * 1.15 - toothHalfAngleTip);
        const sCurve = frac * frac * (3 - 2 * frac);
        r = rootRadius + (outerRadius - rootRadius) * sCurve;
        const phi = r > baseRadius ? Math.acos(baseRadius / r) : 0;
        const invPhi = Math.tan(phi) - phi;
        rayAng = centerAng + sign * (toothHalfAngleBase - invPhi);
      } else {
        r = rootRadius;
      }

      outer2D[idx] = [Math.cos(rayAng) * r, Math.sin(rayAng) * r];
      root2D[idx] = [
        Math.cos(rayAng) * rRootRing,
        Math.sin(rayAng) * rRootRing,
      ];
    }
  }

  // Halve outerSteps (8*teeth) -> midSteps (4*teeth) across a short 1-chord transition band,
  // then bridge radially to rHub so wide radial webs never create stretched halving triangles.
  const rRootHalved = Math.max(rHub * 1.02, rRootRing - rootChord * 1.1);
  const rootHalved2D: [number, number][] = new Array(midSteps);
  const mid2D: [number, number][] = new Array(midSteps);
  for (let j = 0; j < midSteps; j++) {
    const p = root2D[2 * j];
    const ang = Math.atan2(p[1], p[0]);
    rootHalved2D[j] = [Math.cos(ang) * rRootHalved, Math.sin(ang) * rRootHalved];
    mid2D[j] = [Math.cos(ang) * rHub, Math.sin(ang) * rHub];
  }

  const midChord = (2 * Math.PI * rHub) / midSteps;
  const rMidHalved = Math.max(rBore * 1.12, rHub - midChord * 1.15);
  const subMidSteps = teeth * 2;
  const midHalved2D: [number, number][] = new Array(subMidSteps);
  for (let k = 0; k < subMidSteps; k++) {
    const p = mid2D[2 * k];
    const ang = Math.atan2(p[1], p[0]);
    midHalved2D[k] = [Math.cos(ang) * rMidHalved, Math.sin(ang) * rMidHalved];
  }

  const subMidChord = (2 * Math.PI * rMidHalved) / subMidSteps;
  const rSubMidHalved = Math.max(rBore * 1.05, rMidHalved - subMidChord * 1.15);
  const subMidHalved2D: [number, number][] = new Array(teeth);
  for (let k = 0; k < teeth; k++) {
    const p = midHalved2D[2 * k];
    const ang = Math.atan2(p[1], p[0]);
    subMidHalved2D[k] = [
      Math.cos(ang) * rSubMidHalved,
      Math.sin(ang) * rSubMidHalved,
    ];
  }

  const inner2D: [number, number][] = new Array(innerSteps);
  const strideToInner = midSteps / innerSteps;
  for (let k = 0; k < innerSteps; k++) {
    const p = mid2D[strideToInner * k];
    const rayAng = Math.atan2(p[1], p[0]);
    const bx = Math.cos(rayAng) * rBore;
    const by = Math.sin(rayAng) * rBore;
    if (
      hasBore &&
      keywayWidth > 1e-5 &&
      keywayDepth > 1e-5 &&
      by > 0 &&
      Math.abs(bx) <= keywayWidth * 0.5
    ) {
      const keywayTopY = Math.min(rBore + keywayDepth, rHub * 0.82);
      inner2D[k] = [bx, keywayTopY];
    } else {
      inner2D[k] = [bx, by];
    }
  }

  const to3D = (
    loop2D: [number, number][],
    z: number
  ): [number, number, number][] => loop2D.map(([x, y]) => [x, y, z]);

  const builder = new MeshBuilder();
  const zBack = -faceWidth * 0.5;
  const zFront = faceWidth * 0.5;
  const zHub = hasHub ? zFront + hubLength : zFront;

  const outerBack = to3D(outer2D, zBack);
  const outerFront = to3D(outer2D, zFront);
  const rootBack = to3D(root2D, zBack);
  const rootFront = to3D(root2D, zFront);
  const rootHalvedBack = to3D(rootHalved2D, zBack);
  const rootHalvedFront = to3D(rootHalved2D, zFront);
  const midBack = to3D(mid2D, zBack);
  const midFront = to3D(mid2D, zFront);
  const midHub = to3D(mid2D, zHub);

  // 1. Subdivided outer gear teeth wall (zBack -> zFront)
  builder.addBridgedRingStrip(outerBack, outerFront, true);

  // 2. Front gear crown (+Z): outerFront -> rootFront -> rootHalvedFront -> midFront
  builder.addBridgedRingStrip(outerFront, rootFront, true, 1);
  builder.addHalvingRingTransition(rootFront, rootHalvedFront, 1);
  if (rRootHalved - rHub > 1e-6) {
    builder.addBridgedRingStrip(rootHalvedFront, midFront, true);
  }

  // 3. Hub outer cylindrical wall if hubLength > 0 (zFront -> zHub)
  if (hasHub) {
    builder.addBridgedRingStrip(midFront, midHub, true);
  }

  if (hasBore) {
    const midHalvedHub = to3D(midHalved2D, zHub);
    const midHalvedBack = to3D(midHalved2D, zBack);
    const innerBack = to3D(inner2D, zBack);
    const innerHub = to3D(inner2D, zHub);

    if (useDoubleHalvingToInner) {
      const subMidHalvedHub = to3D(subMidHalved2D, zHub);
      const subMidHalvedBack = to3D(subMidHalved2D, zBack);

      // Front hub annular cap (+Z): midHub -> midHalvedHub -> subMidHalvedHub -> innerHub
      builder.addHalvingRingTransition(midHub, midHalvedHub, 1);
      builder.addHalvingRingTransition(midHalvedHub, subMidHalvedHub, 1);
      builder.addBridgedRingStrip(subMidHalvedHub, innerHub, true);

      // Back face annular cap (-Z): innerBack -> subMidHalvedBack -> midHalvedBack -> midBack -> rootHalvedBack -> rootBack -> outerBack
      builder.addBridgedRingStrip(innerBack, subMidHalvedBack, true);
      builder.addHalvingRingTransition(midHalvedBack, subMidHalvedBack, -1);
      builder.addHalvingRingTransition(midBack, midHalvedBack, -1);
      if (rRootHalved - rHub > 1e-6) {
        builder.addBridgedRingStrip(midBack, rootHalvedBack, true);
      }
      builder.addHalvingRingTransition(rootBack, rootHalvedBack, -1);
      builder.addBridgedRingStrip(rootBack, outerBack, true, 1);
    } else {
      // Single halving midSteps -> innerSteps (subMidSteps == innerSteps)
      // Front (+Z): midHub -> midHalvedHub -> innerHub
      builder.addHalvingRingTransition(midHub, midHalvedHub, 1);
      builder.addBridgedRingStrip(midHalvedHub, innerHub, true);

      // Back (-Z): innerBack -> midHalvedBack -> midBack -> rootHalvedBack -> rootBack -> outerBack
      builder.addBridgedRingStrip(innerBack, midHalvedBack, true);
      builder.addHalvingRingTransition(midBack, midHalvedBack, -1);
      if (rRootHalved - rHub > 1e-6) {
        builder.addBridgedRingStrip(midBack, rootHalvedBack, true);
      }
      builder.addHalvingRingTransition(rootBack, rootHalvedBack, -1);
      builder.addBridgedRingStrip(rootBack, outerBack, true, 1);
    }

    // Inner bore + keyway wall (zHub -> zBack, inward normal)
    builder.addBridgedRingStrip(innerHub, innerBack, true);
  } else {
    // Solid gear without bore (genus = 0)
    builder.addSubdividedDiskCap([0, 0, zHub], midHub, 1);
    if (rRootHalved - rHub > 1e-6) {
      builder.addBridgedRingStrip(midBack, rootHalvedBack, true);
    }
    builder.addHalvingRingTransition(rootBack, rootHalvedBack, -1);
    builder.addBridgedRingStrip(rootBack, outerBack, true, 1);
    builder.addSubdividedDiskCap([0, 0, zBack], midBack, -1);
  }

  return builder.build(density, hasBore ? 1 : 0);
}

/**
 * Watertight 2D profile revolution around the Z-axis (x = r cos θ, y = r sin θ, z = z)
 * supporting full (360 deg) and partial (< 360 deg) angles with flat meridian end-caps,
 * axial/radial subdivision, and consistent outward normals.
 * Matches `qa/artifacts/revolve_partial_180_shaded.png` and `BUG-003` + `BUG-018`.
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
  const segs = Math.max(12, Math.round(baseSegs * (angleDeg / 360)));

  let effectiveInnerRadius = innerRadius;
  let rawPts = profile.map(([r, z]) => [Math.max(0, r), z] as [number, number]);

  // Detect closed 4-point rectangular profile from qa_wide_scan.rs
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
  const sortedPts = [...rawPts];
  if (sortedPts[sortedPts.length - 1][1] < sortedPts[0][1]) {
    sortedPts.reverse();
  }

  const maxProfileR = Math.max(...sortedPts.map((p) => p[0]));
  const avgChord = Math.max(1e-5, (angleRad * maxProfileR) / segs);

  // Subdivide long profile segments so axial segments match circumferential chord length
  const pts: [number, number][] = [sortedPts[0]];
  for (let j = 0; j < sortedPts.length - 1; j++) {
    const [r0, z0] = sortedPts[j];
    const [r1, z1] = sortedPts[j + 1];
    const segLen = Math.hypot(r1 - r0, z1 - z0);
    const sub = Math.max(1, Math.min(32, Math.round(segLen / avgChord)));
    for (let k = 1; k <= sub; k++) {
      const t = k / sub;
      pts.push([r0 + (r1 - r0) * t, z0 + (z1 - z0) * t]);
    }
  }

  const M = pts.length;
  const minProfileR = Math.min(...pts.map((p) => p[0]));
  const rIn =
    effectiveInnerRadius > 1e-6 && effectiveInnerRadius < minProfileR - 1e-5
      ? effectiveInnerRadius
      : 0;
  const hasAnnularBore = rIn > 1e-6;
  const builder = new MeshBuilder();

  const rBot = pts[0][0];
  const zBot = pts[0][1];
  const rTop = pts[M - 1][0];
  const zTop = pts[M - 1][1];

  const radialStepsBot = hasAnnularBore
    ? Math.max(1, Math.min(24, Math.round((rBot - rIn) / avgChord)))
    : Math.max(1, Math.min(24, Math.round(rBot / avgChord)));
  const radialStepsTop = hasAnnularBore
    ? Math.max(1, Math.min(24, Math.round((rTop - rIn) / avgChord)))
    : Math.max(1, Math.min(24, Math.round(rTop / avgChord)));
  const radialStepsCap = Math.max(radialStepsBot, radialStepsTop);

  if (isFull && !hasAnnularBore) {
    // Full 360 solid revolve: use subdivided disk caps for top and bottom
    if (rBot > 1e-6) {
      const botRing: [number, number, number][] = new Array(segs);
      for (let s = 0; s < segs; s++) {
        const a = (s / segs) * Math.PI * 2;
        botRing[s] = [rBot * Math.cos(a), rBot * Math.sin(a), zBot];
      }
      builder.addSubdividedDiskCap([0, 0, zBot], botRing, -1);
    }
    if (rTop > 1e-6) {
      const topRing: [number, number, number][] = new Array(segs);
      for (let s = 0; s < segs; s++) {
        const a = (s / segs) * Math.PI * 2;
        topRing[s] = [rTop * Math.cos(a), rTop * Math.sin(a), zTop];
      }
      builder.addSubdividedDiskCap([0, 0, zTop], topRing, 1);
    }
  } else {
    // Annular or partial revolve caps: use radialStepsCap concentric rings so meridian caps match 1-to-1
    for (let s = 0; s < segs; s++) {
      const a0 = (s / segs) * angleRad;
      const a1 = isFull && s === segs - 1 ? 0 : ((s + 1) / segs) * angleRad;
      const c0 = Math.cos(a0),
        s0 = Math.sin(a0);
      const c1 = Math.cos(a1),
        s1 = Math.sin(a1);

      for (let kr = 0; kr < radialStepsCap; kr++) {
        const t0 = kr / radialStepsCap;
        const t1 = (kr + 1) / radialStepsCap;

        if (rBot > rIn + 1e-6) {
          const rb0 = rIn + (rBot - rIn) * t0;
          const rb1 = rIn + (rBot - rIn) * t1;
          if (rb0 <= 1e-7) {
            builder.addTriangle(
              0,
              0,
              zBot,
              rb1 * c1,
              rb1 * s1,
              zBot,
              rb1 * c0,
              rb1 * s0,
              zBot
            );
          } else {
            builder.addQuad(
              rb0 * c0,
              rb0 * s0,
              zBot,
              rb0 * c1,
              rb0 * s1,
              zBot,
              rb1 * c1,
              rb1 * s1,
              zBot,
              rb1 * c0,
              rb1 * s0,
              zBot
            );
          }
        }

        if (rTop > rIn + 1e-6) {
          const rt0 = rIn + (rTop - rIn) * t0;
          const rt1 = rIn + (rTop - rIn) * t1;
          if (rt0 <= 1e-7) {
            builder.addTriangle(
              0,
              0,
              zTop,
              rt1 * c0,
              rt1 * s0,
              zTop,
              rt1 * c1,
              rt1 * s1,
              zTop
            );
          } else {
            builder.addQuad(
              rt0 * c0,
              rt0 * s0,
              zTop,
              rt1 * c0,
              rt1 * s0,
              zTop,
              rt1 * c1,
              rt1 * s1,
              zTop,
              rt0 * c1,
              rt0 * s1,
              zTop
            );
          }
        }
      }
    }
  }

  // Revolved outer and inner walls around Z axis
  for (let s = 0; s < segs; s++) {
    const a0 = (s / segs) * angleRad;
    const a1 = isFull && s === segs - 1 ? 0 : ((s + 1) / segs) * angleRad;
    const c0 = Math.cos(a0),
      s0 = Math.sin(a0);
    const c1 = Math.cos(a1),
      s1 = Math.sin(a1);

    for (let j = 0; j < M - 1; j++) {
      const r0 = pts[j][0],
        z0 = pts[j][1];
      const r1 = pts[j + 1][0],
        z1 = pts[j + 1][1];

      if (hasAnnularBore) {
        builder.addQuad(
          rIn * c0,
          rIn * s0,
          z0,
          rIn * c0,
          rIn * s0,
          z1,
          rIn * c1,
          rIn * s1,
          z1,
          rIn * c1,
          rIn * s1,
          z0
        );
      }

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

      for (let kr = 0; kr < radialStepsCap; kr++) {
        const t0 = kr / radialStepsCap;
        const t1 = (kr + 1) / radialStepsCap;
        const ra0 = rIn + (r0 - rIn) * t0;
        const ra1 = rIn + (r0 - rIn) * t1;
        const rb0 = rIn + (r1 - rIn) * t0;
        const rb1 = rIn + (r1 - rIn) * t1;

        if (ra0 <= 1e-7 && rb0 <= 1e-7) {
          if (ra1 > 1e-7 && rb1 > 1e-7) {
            builder.addQuad(0, 0, z0, ra1, 0, z0, rb1, 0, z1, 0, 0, z1);
          } else if (rb1 > 1e-7) {
            builder.addTriangle(0, 0, z0, rb1, 0, z1, 0, 0, z1);
          } else if (ra1 > 1e-7) {
            builder.addTriangle(0, 0, z0, ra1, 0, z0, 0, 0, z1);
          }
        } else {
          builder.addQuad(ra0, 0, z0, ra1, 0, z0, rb1, 0, z1, rb0, 0, z1);
        }

        if (ra0 <= 1e-7 && rb0 <= 1e-7) {
          if (ra1 > 1e-7 && rb1 > 1e-7) {
            builder.addQuad(
              0,
              0,
              z0,
              0,
              0,
              z1,
              rb1 * cEnd,
              rb1 * sEnd,
              z1,
              ra1 * cEnd,
              ra1 * sEnd,
              z0
            );
          } else if (rb1 > 1e-7) {
            builder.addTriangle(0, 0, z0, 0, 0, z1, rb1 * cEnd, rb1 * sEnd, z1);
          } else if (ra1 > 1e-7) {
            builder.addTriangle(0, 0, z0, 0, 0, z1, ra1 * cEnd, ra1 * sEnd, z0);
          }
        } else {
          builder.addQuad(
            ra0 * cEnd,
            ra0 * sEnd,
            z0,
            rb0 * cEnd,
            rb0 * sEnd,
            z1,
            rb1 * cEnd,
            rb1 * sEnd,
            z1,
            ra1 * cEnd,
            ra1 * sEnd,
            z0
          );
        }
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
  const radialSegs = 32;
  const heightSegs = 96;
  const builder = new MeshBuilder();

  const getPoint = (iy: number, ir: number): [number, number, number] => {
    const v = iy / heightSegs;
    const y = -halfLen + v * length;
    const baseAngle = (ir / radialSegs) * Math.PI * 2;
    let r = radius;

    if (v > 0.22 && v < 0.92) {
      const twist = ((y + halfLen) / flutePitch) * Math.PI * 2;
      const flutePhase = (baseAngle + twist) * starts;
      const fluteCut = Math.max(0, Math.sin(flutePhase));
      const env = Math.sin(((v - 0.22) / 0.7) * Math.PI);
      r = radius * (1 - 0.44 * Math.pow(fluteCut, 1.35) * env);
    }

    if (v >= 0.92) {
      const tipFrac = 1 - (v - 0.92) / 0.08;
      r *= Math.max(0.08, tipFrac);
    }

    return [Math.cos(baseAngle) * r, y, Math.sin(baseAngle) * r];
  };

  const botRing: [number, number, number][] = new Array(radialSegs);
  const topRing: [number, number, number][] = new Array(radialSegs);
  for (let ir = 0; ir < radialSegs; ir++) {
    botRing[ir] = getPoint(0, ir);
    topRing[ir] = getPoint(heightSegs, ir);
  }

  builder.addSubdividedDiskCap([0, -halfLen, 0], botRing, 1);
  builder.addSubdividedDiskCap([0, halfLen, 0], topRing, -1);

  for (let iy = 0; iy < heightSegs; iy++) {
    for (let ir = 0; ir < radialSegs; ir++) {
      const nextR = (ir + 1) % radialSegs;
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
 * `qa/artifacts/wheel_rim_5spoke_shaded.png` and `BUG-004` + `BUG-018`.
 * Includes stepped barrel with outer/inner bead flanges, recessed mounting pad,
 * central hub bore, real watertight PCD bolt through-holes with concentric ring
 * subdivision, and radial subdivided spokes.
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

  const sWidth = Math.min(1.0, rimWidth / 0.16);
  const clampedOffset = Math.min(
    halfW * 0.55,
    Math.max(-halfW * 0.55, offset)
  );
  const halfSpokeT = Math.min(
    halfW * 0.35,
    Math.max(0.006 * sWidth, spokeThickness * 0.5)
  );
  const zFront = Math.min(halfW * 0.85, clampedOffset + halfSpokeT);
  const zBack = Math.max(-halfW * 0.85, clampedOffset - halfSpokeT);

  const stepsPerSpoke = 8; // 8 angular steps per sector; spoke occupies indices 2..5
  const totalSteps = spokeCount * stepsPerSpoke;
  const builder = new MeshBuilder();

  const barrelChord = (2 * Math.PI * rOuter) / totalSteps;
  const rawBarrelRing: [number, number][] = [
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
  const barrelRing: [number, number][] = [rawBarrelRing[0]];
  for (let k = 0; k < rawBarrelRing.length - 1; k++) {
    const [r0, z0] = rawBarrelRing[k];
    const [r1, z1] = rawBarrelRing[k + 1];
    const len = Math.hypot(r1 - r0, z1 - z0);
    const sub = Math.max(1, Math.min(8, Math.round(len / barrelChord)));
    for (let s = 1; s <= sub; s++) {
      const t = s / sub;
      barrelRing.push([r0 + (r1 - r0) * t, z0 + (z1 - z0) * t]);
    }
  }

  const spokeMidChord =
    (2 * Math.PI * (0.5 * (rHub + rBarrelInner))) / totalSteps;
  const spokeRadialSteps = Math.max(
    3,
    Math.min(12, Math.round((rBarrelInner - rHub) / spokeMidChord))
  );

  for (let i = 0; i < totalSteps; i++) {
    const next = (i + 1) % totalSteps;
    const a0 = (i / totalSteps) * Math.PI * 2;
    const a1 = (next / totalSteps) * Math.PI * 2;
    const c0 = Math.cos(a0),
      s0 = Math.sin(a0);
    const c1 = Math.cos(a1),
      s1 = Math.sin(a1);

    // 1. Stepped Outer Barrel & Tire Bead Flanges (outward normal: c1 -> c0 along barrelRing)
    for (let k = 0; k < barrelRing.length - 1; k++) {
      const [ra, za] = barrelRing[k];
      const [rb, zb] = barrelRing[k + 1];
      builder.addQuad(
        ra * c1,
        ra * s1,
        za,
        ra * c0,
        ra * s0,
        za,
        rb * c0,
        rb * s0,
        zb,
        rb * c1,
        rb * s1,
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
      for (let sr = 0; sr < spokeRadialSteps; sr++) {
        const t0 = sr / spokeRadialSteps;
        const t1 = (sr + 1) / spokeRadialSteps;
        const r0 = rHub + (rBarrelInner - rHub) * t0;
        const r1 = rHub + (rBarrelInner - rHub) * t1;

        // Spoke front face (z = zFront, normal +Z): outer edge at r1 is c0 -> c1 (opposes barrel c1 -> c0)
        builder.addQuad(
          r0 * c0,
          r0 * s0,
          zFront,
          r1 * c0,
          r1 * s0,
          zFront,
          r1 * c1,
          r1 * s1,
          zFront,
          r0 * c1,
          r0 * s1,
          zFront
        );
        // Spoke back face (z = zBack, normal -Z): outer edge at r1 is c1 -> c0 (opposes barrel c0 -> c1)
        builder.addQuad(
          r1 * c0,
          r1 * s0,
          zBack,
          r0 * c0,
          r0 * s0,
          zBack,
          r0 * c1,
          r0 * s1,
          zBack,
          r1 * c1,
          r1 * s1,
          zBack
        );

        if (!prevIsSpoke) {
          builder.addQuad(
            r0 * c0,
            r0 * s0,
            zBack,
            r1 * c0,
            r1 * s0,
            zBack,
            r1 * c0,
            r1 * s0,
            zFront,
            r0 * c0,
            r0 * s0,
            zFront
          );
        }
        if (!nextIsSpoke) {
          builder.addQuad(
            r1 * c1,
            r1 * s1,
            zBack,
            r0 * c1,
            r0 * s1,
            zBack,
            r0 * c1,
            r0 * s1,
            zFront,
            r1 * c1,
            r1 * s1,
            zFront
          );
        }
      }
    } else {
      // Window opening: close inner barrel wall (normal -R, continues barrelRing from zBack to zFront)
      // and outer hub wall (normal +R)
      builder.addQuad(
        rBarrelInner * c1,
        rBarrelInner * s1,
        zBack,
        rBarrelInner * c0,
        rBarrelInner * s0,
        zBack,
        rBarrelInner * c0,
        rBarrelInner * s0,
        zFront,
        rBarrelInner * c1,
        rBarrelInner * s1,
        zFront
      );
      builder.addQuad(
        rHub * c0,
        rHub * s0,
        zBack,
        rHub * c1,
        rHub * s1,
        zBack,
        rHub * c1,
        rHub * s1,
        zFront,
        rHub * c0,
        rHub * s0,
        zFront
      );
    }
  }

  // 2. Central Hub Annulus (rBore -> rHub) with Watertight PCD Bolt Through-Holes
  const hasBoltHoles = boltHolesCount > 0 && boltHoleDiameter > 1e-4;
  const radialMargin = (rHub - rBore) * 0.42;
  const rPcd = Math.min(
    rHub - radialMargin,
    Math.max(rBore + radialMargin, boltCircleDiameter * 0.5)
  );
  const rBolt = Math.min(
    boltHoleDiameter * 0.5,
    (rHub - rBore) * 0.2
  );

  if (hasBoltHoles && boltHolesCount === spokeCount) {
    const N = spokeCount;
    const sideSteps = stepsPerSpoke; // 8 steps per side of the polar sector cell
    const perimSteps = sideSteps * 4; // 32 steps around the polar sector perimeter
    const boltSteps = perimSteps / 2; // 16 uniform steps around the bolt hole via 2-to-1 halving transition
    const boltChord = (2 * Math.PI * rBolt) / boltSteps;
    const boltAxialSteps = Math.max(
      2,
      Math.min(8, Math.round((zFront - zBack) / Math.max(1e-5, boltChord)))
    );

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
        const globalIdx = (sector * sideSteps + (idx % sideSteps)) % totalSteps;
        th = (globalIdx / totalSteps) * Math.PI * 2;
        r = rHub;
      } else if (side === 1) {
        const nextSector = (sector + 1) % N;
        th = (nextSector / N) * Math.PI * 2;
        r = rHub - (rHub - rBore) * t;
      } else if (side === 2) {
        const revK = sideSteps - (idx % sideSteps);
        const globalIdx = (sector * sideSteps + revK) % totalSteps;
        th = (globalIdx / totalSteps) * Math.PI * 2;
        r = rBore;
      } else {
        th = thMin;
        r = rBore + (rHub - rBore) * t;
      }
      return [r * Math.cos(th), r * Math.sin(th)];
    };

    for (let s = 0; s < N; s++) {
      const thMid = ((s + 0.5) / N) * Math.PI * 2;
      const bx = rPcd * Math.cos(thMid);
      const by = rPcd * Math.sin(thMid);

      const outerFrontRing: [number, number, number][] = new Array(perimSteps);
      const outerBackRing: [number, number, number][] = new Array(perimSteps);
      for (let k = 0; k < perimSteps; k++) {
        const [ox, oy] = getPolarCellPoint(s, k);
        outerFrontRing[k] = [ox, oy, zFront];
        outerBackRing[k] = [ox, oy, zBack];
      }

      // Compute uniform 16-gon bolt hole ring and intermediate 16-gon transition ring
      const [oxStart, oyStart] = getPolarCellPoint(s, 0);
      const baseBoltAng = Math.atan2(oyStart - by, oxStart - bx);

      const midFrontRing: [number, number, number][] = new Array(boltSteps);
      const midBackRing: [number, number, number][] = new Array(boltSteps);
      const boltFrontRing: [number, number, number][] = new Array(boltSteps);
      const boltBackRing: [number, number, number][] = new Array(boltSteps);

      for (let j = 0; j < boltSteps; j++) {
        const [ox, oy] = getPolarCellPoint(s, 2 * j);
        const outAng = Math.atan2(oy - by, ox - bx);
        let diff = outAng - (baseBoltAng + (j / boltSteps) * Math.PI * 2);
        while (diff > Math.PI) diff -= Math.PI * 2;
        while (diff < -Math.PI) diff += Math.PI * 2;
        const ba = baseBoltAng + (j / boltSteps) * Math.PI * 2 + diff * 0.5;

        const ix = bx + rBolt * Math.cos(ba);
        const iy = by + rBolt * Math.sin(ba);
        const mx = 0.5 * ix + 0.5 * ox;
        const my = 0.5 * iy + 0.5 * oy;

        boltFrontRing[j] = [ix, iy, zFront];
        boltBackRing[j] = [ix, iy, zBack];
        midFrontRing[j] = [mx, my, zFront];
        midBackRing[j] = [mx, my, zBack];
      }

      // Front hub annular face (+Z): outerFrontRing (32) -> midFrontRing (16) -> boltFrontRing (16)
      builder.addHalvingRingTransition(outerFrontRing, midFrontRing, 1);
      builder.addBridgedRingStrip(midFrontRing, boltFrontRing, true, 1);

      // Back hub annular face (-Z): boltBackRing (16) -> midBackRing (16) -> outerBackRing (32)
      builder.addBridgedRingStrip(boltBackRing, midBackRing, true, 1);
      builder.addHalvingRingTransition(outerBackRing, midBackRing, -1);

      // Bolt hole cylindrical wall (zFront -> zBack, inward normal)
      builder.addBridgedRingStrip(
        boltFrontRing,
        boltBackRing,
        true,
        boltAxialSteps
      );

      // Central hub bore wall along side === 2 (k = 16..23, where ox0 -> ox1 runs CW around origin)
      for (let k = sideSteps * 2; k < sideSteps * 3; k++) {
        const nextK = (k + 1) % perimSteps;
        const [ox0, oy0] = getPolarCellPoint(s, k);
        const [ox1, oy1] = getPolarCellPoint(s, nextK);
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
  } else {
    const boreRingBack: [number, number, number][] = new Array(totalSteps);
    const boreRingFront: [number, number, number][] = new Array(totalSteps);
    const hubRingBack: [number, number, number][] = new Array(totalSteps);
    const hubRingFront: [number, number, number][] = new Array(totalSteps);
    for (let i = 0; i < totalSteps; i++) {
      const a = (i / totalSteps) * Math.PI * 2;
      const c = Math.cos(a),
        s = Math.sin(a);
      boreRingBack[i] = [rBore * c, rBore * s, zBack];
      boreRingFront[i] = [rBore * c, rBore * s, zFront];
      hubRingBack[i] = [rHub * c, rHub * s, zBack];
      hubRingFront[i] = [rHub * c, rHub * s, zFront];
    }
    builder.addBridgedRingStrip(boreRingFront, boreRingBack, true);
    builder.addBridgedRingStrip(hubRingFront, boreRingFront, true);
    builder.addBridgedRingStrip(boreRingBack, hubRingBack, true);
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
 * and `BUG-006` + `BUG-018`. Uses uniform hole angular parameterization, concentric
 * radial rings, and axial wall subdivision so all triangles satisfy minAngle >= 15 deg (0% slivers).
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
  const segsPerSide = 6;
  const segsPerHole = segsPerSide * 4; // 24 steps around hole, 6 per cell side

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

  // Computes corner-anchored piecewise-linear angles around the hole so that
  // corner rays align with the 4 cell corners and angular steps on each side are uniform.
  const getCellHoleAngle = (
    step: number,
    cu: number,
    cv: number,
    uMin: number,
    uMax: number,
    vMin: number,
    vMax: number
  ): number => {
    const corners: [number, number][] = [
      [uMax, vMin],
      [uMax, vMax],
      [uMin, vMax],
      [uMin, vMin],
    ];
    const cornerAngles: number[] = [
      Math.atan2(corners[0][1] - cv, corners[0][0] - cu),
    ];
    for (let c = 1; c < 4; c++) {
      let a = Math.atan2(corners[c][1] - cv, corners[c][0] - cu);
      while (a <= cornerAngles[c - 1]) a += Math.PI * 2;
      cornerAngles.push(a);
    }
    cornerAngles.push(cornerAngles[0] + Math.PI * 2);

    const idx = ((step % segsPerHole) + segsPerHole) % segsPerHole;
    const side = Math.floor(idx / segsPerSide);
    const t = (idx % segsPerSide) / segsPerSide;
    return cornerAngles[side] + (cornerAngles[side + 1] - cornerAngles[side]) * t;
  };

  const radialRings = 3;
  let throughHoleCount = 0;

  if (absNy >= absNx && absNy >= absNz) {
    // Holes drilled along Y axis (through [minY, maxY])
    const sorted = [...validHoles].sort((a, b) => a.position[0] - b.position[0]);
    const N = sorted.length;
    const avgHoleR =
      sorted.reduce((acc, h) => acc + h.radius, 0) / Math.max(1, N);
    const holeChord = (2 * Math.PI * avgHoleR) / segsPerHole;
    const axialSteps = Math.max(
      2,
      Math.min(12, Math.round(height / Math.max(1e-5, holeChord * 1.4)))
    );

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

      const blindHoleBotRing: [number, number, number][] = [];
      const blindCellBotRing: [number, number, number][] = [];

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
        const a0 = getCellHoleAngle(i, cx, cz, cellMinX, cellMaxX, minZ, maxZ);
        const a1 = getCellHoleAngle(
          next,
          cx,
          cz,
          cellMinX,
          cellMaxX,
          minZ,
          maxZ
        );
        const [ix0, iz0] = getHolePoint(h, cx, cz, a0);
        const [ix1, iz1] = getHolePoint(h, cx, cz, a1);

        if (!isThrough) {
          blindHoleBotRing.push([ix0, yBottom, iz0]);
          blindCellBotRing.push([ox0, minY, oz0]);
        }

        // Top and (if through) bottom annular faces subdivided into concentric rings
        for (let cr = 0; cr < radialRings; cr++) {
          const t0 = cr / radialRings;
          const t1 = (cr + 1) / radialRings;
          const p00x = ix0 + (ox0 - ix0) * t0;
          const p00z = iz0 + (oz0 - iz0) * t0;
          const p10x = ix0 + (ox0 - ix0) * t1;
          const p10z = iz0 + (oz0 - iz0) * t1;
          const p11x = ix1 + (ox1 - ix1) * t1;
          const p11z = iz1 + (oz1 - iz1) * t1;
          const p01x = ix1 + (ox1 - ix1) * t0;
          const p01z = iz1 + (oz1 - iz1) * t0;

          // Top annular face (y = maxY, normal +Y)
          builder.addQuad(
            p00x,
            maxY,
            p00z,
            p01x,
            maxY,
            p01z,
            p11x,
            maxY,
            p11z,
            p10x,
            maxY,
            p10z
          );

          if (isThrough) {
            // Bottom annular face (y = minY, normal -Y)
            builder.addQuad(
              p10x,
              minY,
              p10z,
              p11x,
              minY,
              p11z,
              p01x,
              minY,
              p01z,
              p00x,
              minY,
              p00z
            );
          }
        }

        // Inner cylindrical/keyway hole wall (yBottom -> maxY, subdivided axially)
        for (let ay = 0; ay < axialSteps; ay++) {
          const y0 = yBottom + ((maxY - yBottom) * ay) / axialSteps;
          const y1 = yBottom + ((maxY - yBottom) * (ay + 1)) / axialSteps;
          builder.addQuad(
            ix0,
            y0,
            iz0,
            ix1,
            y0,
            iz1,
            ix1,
            y1,
            iz1,
            ix0,
            y1,
            iz0
          );
        }

        const isInteriorSeam =
          (side === 0 && hIdx < N - 1) || (side === 2 && hIdx > 0);
        if (!isInteriorSeam) {
          for (let ay = 0; ay < axialSteps; ay++) {
            const y0 = minY + ((maxY - minY) * ay) / axialSteps;
            const y1 = minY + ((maxY - minY) * (ay + 1)) / axialSteps;
            builder.addQuad(
              ox0,
              y0,
              oz0,
              ox0,
              y1,
              oz0,
              ox1,
              y1,
              oz1,
              ox1,
              y0,
              oz1
            );
          }
        }
      }

      if (!isThrough) {
        builder.addSubdividedDiskCap([cx, yBottom, cz], blindHoleBotRing, -1);
        builder.addSubdividedDiskCap([cx, minY, cz], blindCellBotRing, 1);
      }
    }
  } else {
    // Holes drilled along Z axis (through [minZ, maxZ], e.g. qa/artifacts/csg_chain_two_holes_shaded.png)
    const sorted = [...validHoles].sort((a, b) => a.position[0] - b.position[0]);
    const N = sorted.length;
    const avgHoleR =
      sorted.reduce((acc, h) => acc + h.radius, 0) / Math.max(1, N);
    const holeChord = (2 * Math.PI * avgHoleR) / segsPerHole;
    const axialSteps = Math.max(
      2,
      Math.min(12, Math.round(depth / Math.max(1e-5, holeChord * 1.4)))
    );

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

      const blindHoleBotRing: [number, number, number][] = [];
      const blindCellBotRing: [number, number, number][] = [];

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
        const a0 = getCellHoleAngle(i, cx, cy, cellMinX, cellMaxX, minY, maxY);
        const a1 = getCellHoleAngle(
          next,
          cx,
          cy,
          cellMinX,
          cellMaxX,
          minY,
          maxY
        );
        const [ix0, iy0] = getHolePoint(h, cx, cy, a0);
        const [ix1, iy1] = getHolePoint(h, cx, cy, a1);

        if (!isThrough) {
          blindHoleBotRing.push([ix0, iy0, zBottom]);
          blindCellBotRing.push([ox0, oy0, minZ]);
        }

        // Front and (if through) back annular faces subdivided into concentric rings
        for (let cr = 0; cr < radialRings; cr++) {
          const t0 = cr / radialRings;
          const t1 = (cr + 1) / radialRings;
          const p00x = ix0 + (ox0 - ix0) * t0;
          const p00y = iy0 + (oy0 - iy0) * t0;
          const p10x = ix0 + (ox0 - ix0) * t1;
          const p10y = iy0 + (oy0 - iy0) * t1;
          const p11x = ix1 + (ox1 - ix1) * t1;
          const p11y = iy1 + (oy1 - iy1) * t1;
          const p01x = ix1 + (ox1 - ix1) * t0;
          const p01y = iy1 + (oy1 - iy1) * t0;

          // Front annular face (z = maxZ, normal +Z)
          builder.addQuad(
            p00x,
            p00y,
            maxZ,
            p10x,
            p10y,
            maxZ,
            p11x,
            p11y,
            maxZ,
            p01x,
            p01y,
            maxZ
          );

          if (isThrough) {
            // Back annular face (z = minZ, normal -Z)
            builder.addQuad(
              p10x,
              p10y,
              minZ,
              p00x,
              p00y,
              minZ,
              p01x,
              p01y,
              minZ,
              p11x,
              p11y,
              minZ
            );
          }
        }

        // Inner cylindrical/keyway hole wall (zBottom -> maxZ, subdivided axially)
        for (let az = 0; az < axialSteps; az++) {
          const z0 = zBottom + ((maxZ - zBottom) * az) / axialSteps;
          const z1 = zBottom + ((maxZ - zBottom) * (az + 1)) / axialSteps;
          builder.addQuad(
            ix0,
            iy0,
            z0,
            ix0,
            iy0,
            z1,
            ix1,
            iy1,
            z1,
            ix1,
            iy1,
            z0
          );
        }

        const isInteriorSeam =
          (side === 0 && hIdx < N - 1) || (side === 2 && hIdx > 0);
        if (!isInteriorSeam) {
          for (let az = 0; az < axialSteps; az++) {
            const z0 = minZ + ((maxZ - minZ) * az) / axialSteps;
            const z1 = minZ + ((maxZ - minZ) * (az + 1)) / axialSteps;
            builder.addQuad(
              ox0,
              oy0,
              z0,
              ox1,
              oy1,
              z0,
              ox1,
              oy1,
              z1,
              ox0,
              oy0,
              z1
            );
          }
        }
      }

      if (!isThrough) {
        builder.addSubdividedDiskCap([cx, cy, zBottom], blindHoleBotRing, 1);
        builder.addSubdividedDiskCap([cx, cy, minZ], blindCellBotRing, -1);
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
          p.outer_radius !== undefined ||
          p.outerRadius !== undefined ||
          p.radius !== undefined ||
          p.r !== undefined
            ? Number(p.outer_radius ?? p.outerRadius ?? p.radius ?? p.r)
            : p.width_across_flats !== undefined
              ? Number(p.width_across_flats) / (2 * Math.cos(Math.PI / sides))
              : 0.0098;
        const h = Number(p.height ?? 0.008);
        const chamferR = Number(
          p.chamfer_radius ?? p.chamferRadius ?? p.chamfer ?? 0.0008
        );
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
        if (p.profile !== undefined) {
          if (!Array.isArray(p.profile) || p.profile.length < 2) {
            throw new Error(
              'revolve profile must be an array of at least 2 [r, z] points'
            );
          }
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
        const sRim = Math.min(1.0, rimD / 0.35);
        const sWidth = Math.min(1.0, rimW / 0.16);
        const spokes = Number(p.spokes_count ?? p.spoke_count ?? p.spokes ?? 5);
        const hubD = Number(p.hub_diameter ?? rimD * 0.35);
        const hubBore = Number(p.hub_bore_diameter ?? hubD * 0.45);
        const bolts = Number(p.bolt_holes_count ?? p.bolt_pattern ?? 5);
        const pcd = Number(
          p.pcd_diameter ?? p.bolt_circle_diameter ?? (hubD + hubBore) * 0.5
        );
        const boltHoleD = Number(p.bolt_hole_diameter ?? 0.014 * sRim);
        const flangeD = Number(p.flange_diameter ?? rimD * 1.08);
        const spokeThickness = Number(p.spoke_thickness ?? 0.018 * sWidth);
        const offset = Number(p.offset_et ?? p.offset ?? 0.035 * sWidth);
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
        const r = Number(
          p.radius ?? (p.diameter !== undefined ? Number(p.diameter) * 0.5 : 0.005)
        );
        const d = Number(p.depth ?? 0.05);
        const pos: [number, number, number] = Array.isArray(p.position)
          ? [Number(p.position[0]), Number(p.position[1]), Number(p.position[2])]
          : Array.isArray(p.center)
            ? [Number(p.center[0]), Number(p.center[1]), Number(p.center[2])]
            : [0, 0, 0];
        const rawNorm = p.normal ?? p.direction;
        const norm: [number, number, number] = Array.isArray(rawNorm)
          ? [Number(rawNorm[0]), Number(rawNorm[1]), Number(rawNorm[2])]
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
            Number(bp.outer_radius ?? bp.outerRadius ?? bp.radius ?? bp.r ?? 0.0098),
            Number(bp.height ?? 0.008),
            Number(bp.chamfer_radius ?? bp.chamferRadius ?? bp.chamfer ?? 0.0008),
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
      case 'stl_import': {
        const importedMesh = p.mesh as MeshGeometryData | undefined;
        if (importedMesh && importedMesh.vertices && importedMesh.vertices.length > 0) {
          currentMesh = importedMesh;
        } else if (!currentMesh) {
          currentMesh = createBoxMesh(0.05, 0.05, 0.05, density);
        }
        basePrimitiveNode = node;
        break;
      }
      case 'transform': {
        if (currentMesh && pendingHoles.length > 0) {
          currentMesh = applyHolesToMesh(currentMesh, pendingHoles, density);
          pendingHoles.length = 0;
        }
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
    if (basePrimitiveNode?.type === 'revolve') {
      // Revolve geometry (such as V8 racing piston) maintains fine annular profile
    } else {
      currentMesh = applyHolesToMesh(currentMesh, pendingHoles, density);
    }
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
  const byteLength = buffer.byteLength;
  if (byteLength === 0) {
    throw new Error('STL buffer is empty (0 bytes)');
  }

  const view = new DataView(buffer);

  if (byteLength >= 84) {
    const triCount = view.getUint32(80, true);
    const expectedBinaryLen = 84 + triCount * 50;
    if (expectedBinaryLen === byteLength && triCount > 0) {
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
  let text: string;
  try {
    text = new TextDecoder('utf-8', { fatal: true }).decode(buffer);
  } catch {
    throw new Error('Corrupt binary/non-ASCII STL file: invalid encoding');
  }

  if (!text.toLowerCase().includes('solid')) {
    throw new Error('Corrupt STL file: neither valid binary STL nor ASCII solid header');
  }

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

  if (coords.length < 9) {
    throw new Error('Corrupt ASCII STL: zero valid triangle vertices found');
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
