import { MeshGeometryData } from '../types/cad';
import { computeMeshMetrics } from './primitives';

const EPSILON = 1e-7;

type Vector3 = [number, number, number];

function vSub(a: Vector3, b: Vector3): Vector3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function vAdd(a: Vector3, b: Vector3): Vector3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function vScale(a: Vector3, s: number): Vector3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

function vDot(a: Vector3, b: Vector3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function vCross(a: Vector3, b: Vector3): Vector3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function vNorm(a: Vector3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

function vNormalize(a: Vector3): Vector3 {
  const len = vNorm(a);
  return len > 1e-12 ? [a[0] / len, a[1] / len, a[2] / len] : [0, 0, 1];
}

export interface CsgVertex {
  pos: Vector3;
  normal: Vector3;
}

export class CsgPlane {
  normal: Vector3;
  w: number;

  constructor(normal: Vector3, w: number) {
    this.normal = normal;
    this.w = w;
  }

  static fromPoints(a: Vector3, b: Vector3, c: Vector3): CsgPlane | null {
    const normal = vCross(vSub(b, a), vSub(c, a));
    const len = vNorm(normal);
    if (len < 1e-12) return null;
    const unitN: Vector3 = [normal[0] / len, normal[1] / len, normal[2] / len];
    const w = vDot(unitN, a);
    return new CsgPlane(unitN, w);
  }

  clone(): CsgPlane {
    return new CsgPlane([this.normal[0], this.normal[1], this.normal[2]], this.w);
  }

  flip(): void {
    this.normal = [-this.normal[0], -this.normal[1], -this.normal[2]];
    this.w = -this.w;
  }

  splitPolygon(
    poly: CsgPolygon,
    coplanarFront: CsgPolygon[],
    coplanarBack: CsgPolygon[],
    front: CsgPolygon[],
    back: CsgPolygon[]
  ): void {
    const types: number[] = [];
    let hasFront = false;
    let hasBack = false;

    for (const v of poly.vertices) {
      const t = vDot(this.normal, v.pos) - this.w;
      if (t > EPSILON) {
        hasFront = true;
        types.push(1); // Front
      } else if (t < -EPSILON) {
        hasBack = true;
        types.push(-1); // Back
      } else {
        types.push(0); // Coplanar
      }
    }

    if (!hasFront && !hasBack) {
      if (vDot(this.normal, poly.plane.normal) > 0.0) {
        coplanarFront.push(poly);
      } else {
        coplanarBack.push(poly);
      }
    } else if (hasFront && !hasBack) {
      front.push(poly);
    } else if (!hasFront && hasBack) {
      back.push(poly);
    } else {
      // SPANNING: split polygon
      const fVerts: CsgVertex[] = [];
      const bVerts: CsgVertex[] = [];
      const count = poly.vertices.length;

      for (let i = 0; i < count; i++) {
        const j = (i + 1) % count;
        const ti = types[i];
        const tj = types[j];
        const vi = poly.vertices[i];
        const vj = poly.vertices[j];

        if (ti !== -1) fVerts.push(vi);
        if (ti !== 1) bVerts.push(vi);

        if ((ti === 1 && tj === -1) || (ti === -1 && tj === 1)) {
          const pi = vi.pos;
          const pj = vj.pos;

          // Deterministic canonical ordering of edge endpoints for exact bitwise matching
          const isOrdered =
            pi[0] !== pj[0]
              ? pi[0] < pj[0]
              : pi[1] !== pj[1]
                ? pi[1] < pj[1]
                : pi[2] < pj[2];
          const p0 = isOrdered ? pi : pj;
          const p1 = isOrdered ? pj : pi;
          const diff = vSub(p1, p0);
          const denom = vDot(this.normal, diff);
          const tFrac =
            Math.abs(denom) > 1e-14
              ? Math.max(0, Math.min(1, (this.w - vDot(this.normal, p0)) / denom))
              : 0.5;
          const pMid: Vector3 = [
            p0[0] + diff[0] * tFrac,
            p0[1] + diff[1] * tFrac,
            p0[2] + diff[2] * tFrac,
          ];

          const edgeLen = Math.max(1e-12, Math.hypot(pj[0] - pi[0], pj[1] - pi[1], pj[2] - pi[2]));
          const currDist = Math.hypot(pMid[0] - pi[0], pMid[1] - pi[1], pMid[2] - pi[2]);
          const interpT = Math.max(0, Math.min(1, currDist / edgeLen));
          const nMid = vNormalize(vAdd(vi.normal, vScale(vSub(vj.normal, vi.normal), interpT)));
          const vMid: CsgVertex = { pos: pMid, normal: nMid };
          fVerts.push(vMid);
          bVerts.push(vMid);
        }
      }

      if (fVerts.length >= 3) {
        front.push(new CsgPolygon(fVerts, poly.plane.clone()));
      }
      if (bVerts.length >= 3) {
        back.push(new CsgPolygon(bVerts, poly.plane.clone()));
      }
    }
  }
}

export class CsgPolygon {
  vertices: CsgVertex[];
  plane: CsgPlane;

  constructor(vertices: CsgVertex[], plane?: CsgPlane) {
    this.vertices = vertices.map((v) => ({
      pos: [v.pos[0], v.pos[1], v.pos[2]],
      normal: [v.normal[0], v.normal[1], v.normal[2]],
    }));
    if (plane) {
      this.plane = plane.clone();
    } else {
      const p = CsgPlane.fromPoints(
        vertices[0].pos,
        vertices[1].pos,
        vertices[2].pos
      );
      this.plane = p || new CsgPlane([0, 1, 0], 0);
    }
  }

  clone(): CsgPolygon {
    return new CsgPolygon(this.vertices, this.plane);
  }

  flip(): void {
    this.vertices.reverse();
    for (const v of this.vertices) {
      v.normal = [-v.normal[0], -v.normal[1], -v.normal[2]];
    }
    this.plane.flip();
  }
}

export class CsgBspNode {
  plane: CsgPlane | null = null;
  front: CsgBspNode | null = null;
  back: CsgBspNode | null = null;
  polygons: CsgPolygon[] = [];

  static fromMesh(mesh: MeshGeometryData): CsgBspNode {
    const polygons: CsgPolygon[] = [];
    const verts = mesh.vertices;
    const normals = mesh.normals;
    const idx = mesh.indices;

    for (let i = 0; i < idx.length; i += 3) {
      const i0 = idx[i] * 3;
      const i1 = idx[i + 1] * 3;
      const i2 = idx[i + 2] * 3;

      const p0: Vector3 = [verts[i0], verts[i0 + 1], verts[i0 + 2]];
      const p1: Vector3 = [verts[i1], verts[i1 + 1], verts[i1 + 2]];
      const p2: Vector3 = [verts[i2], verts[i2 + 1], verts[i2 + 2]];

      const n0: Vector3 = [normals[i0], normals[i0 + 1], normals[i0 + 2]];
      const n1: Vector3 = [normals[i1], normals[i1 + 1], normals[i1 + 2]];
      const n2: Vector3 = [normals[i2], normals[i2 + 1], normals[i2 + 2]];

      const plane = CsgPlane.fromPoints(p0, p1, p2);
      if (plane) {
        polygons.push(
          new CsgPolygon(
            [
              { pos: p0, normal: n0 },
              { pos: p1, normal: n1 },
              { pos: p2, normal: n2 },
            ],
            plane
          )
        );
      }
    }

    const node = new CsgBspNode();
    node.build(polygons);
    return node;
  }

  invert(): void {
    for (const poly of this.polygons) {
      poly.flip();
    }
    if (this.plane) this.plane.flip();
    if (this.front) this.front.invert();
    if (this.back) this.back.invert();
    const temp = this.front;
    this.front = this.back;
    this.back = temp;
  }

  clipPolygons(polygons: CsgPolygon[]): CsgPolygon[] {
    if (!this.plane) return polygons.slice();
    let frontList: CsgPolygon[] = [];
    let backList: CsgPolygon[] = [];

    for (const poly of polygons) {
      this.plane.splitPolygon(poly, frontList, backList, frontList, backList);
    }

    if (this.front) frontList = this.front.clipPolygons(frontList);
    if (this.back) {
      backList = this.back.clipPolygons(backList);
    } else {
      backList = [];
    }

    return frontList.concat(backList);
  }

  clipTo(other: CsgBspNode): void {
    this.polygons = other.clipPolygons(this.polygons);
    if (this.front) this.front.clipTo(other);
    if (this.back) this.back.clipTo(other);
  }

  allPolygons(): CsgPolygon[] {
    let result = this.polygons.slice();
    if (this.front) result = result.concat(this.front.allPolygons());
    if (this.back) result = result.concat(this.back.allPolygons());
    return result;
  }

  build(polygons: CsgPolygon[]): void {
    if (polygons.length === 0) return;
    if (!this.plane) {
      this.plane = polygons[0].plane.clone();
    }
    const frontList: CsgPolygon[] = [];
    const backList: CsgPolygon[] = [];

    for (const poly of polygons) {
      this.plane.splitPolygon(
        poly,
        this.polygons,
        this.polygons,
        frontList,
        backList
      );
    }

    if (frontList.length > 0) {
      if (!this.front) this.front = new CsgBspNode();
      this.front.build(frontList);
    }
    if (backList.length > 0) {
      if (!this.back) this.back = new CsgBspNode();
      this.back.build(backList);
    }
  }
}

/**
 * Converts a collection of CSG polygons back into a watertight MeshGeometryData.
 */
export function polygonsToMeshGeometry(
  polygons: CsgPolygon[],
  density: number = 7850
): MeshGeometryData {
  if (polygons.length === 0) {
    return computeMeshMetrics(
      new Float32Array(0),
      new Float32Array(0),
      new Uint32Array(0),
      density
    );
  }

  // Triangulate polygons into triangle list
  const triVertices: number[] = [];
  const triNormals: number[] = [];
  const triIndices: number[] = [];
  let vCount = 0;

  for (const poly of polygons) {
    const v = poly.vertices;
    if (v.length < 3) continue;

    // Ear clipping / fan triangulation from vertex 0
    for (let k = 1; k < v.length - 1; k++) {
      const v0 = v[0];
      const v1 = v[k];
      const v2 = v[k + 1];

      triVertices.push(
        v0.pos[0], v0.pos[1], v0.pos[2],
        v1.pos[0], v1.pos[1], v1.pos[2],
        v2.pos[0], v2.pos[1], v2.pos[2]
      );
      triNormals.push(
        v0.normal[0], v0.normal[1], v0.normal[2],
        v1.normal[0], v1.normal[1], v1.normal[2],
        v2.normal[0], v2.normal[1], v2.normal[2]
      );
      triIndices.push(vCount, vCount + 1, vCount + 2);
      vCount += 3;
    }
  }

  return computeMeshMetrics(
    new Float32Array(triVertices),
    new Float32Array(triNormals),
    new Uint32Array(triIndices),
    density
  );
}

/**
 * Computes CSG Boolean Union of two 3D meshes: A ∪ B
 */
export function csgUnionMesh(
  meshA: MeshGeometryData,
  meshB: MeshGeometryData,
  density: number = 7850
): MeshGeometryData {
  if (meshA.indices.length === 0) return meshB;
  if (meshB.indices.length === 0) return meshA;

  const a = CsgBspNode.fromMesh(meshA);
  const b = CsgBspNode.fromMesh(meshB);

  a.clipTo(b);
  b.clipTo(a);
  b.invert();
  b.clipTo(a);
  b.invert();

  a.build(b.allPolygons());
  return polygonsToMeshGeometry(a.allPolygons(), density);
}

/**
 * Computes CSG Boolean Difference of two 3D meshes: A \ B (A minus B)
 */
export function csgDifferenceMesh(
  meshA: MeshGeometryData,
  meshB: MeshGeometryData,
  density: number = 7850
): MeshGeometryData {
  if (meshA.indices.length === 0) return meshA;
  if (meshB.indices.length === 0) return meshA;

  const a = CsgBspNode.fromMesh(meshA);
  const b = CsgBspNode.fromMesh(meshB);

  a.invert();
  a.clipTo(b);
  b.clipTo(a);
  b.invert();
  b.clipTo(a);
  b.invert();

  a.build(b.allPolygons());
  a.invert();
  return polygonsToMeshGeometry(a.allPolygons(), density);
}

/**
 * Computes CSG Boolean Intersection of two 3D meshes: A ∩ B
 */
export function csgIntersectionMesh(
  meshA: MeshGeometryData,
  meshB: MeshGeometryData,
  density: number = 7850
): MeshGeometryData {
  if (meshA.indices.length === 0 || meshB.indices.length === 0) {
    return computeMeshMetrics(new Float32Array(0), new Float32Array(0), new Uint32Array(0), density);
  }

  const a = CsgBspNode.fromMesh(meshA);
  const b = CsgBspNode.fromMesh(meshB);

  a.invert();
  b.clipTo(a);
  b.invert();
  a.clipTo(b);
  b.clipTo(a);

  a.build(b.allPolygons());
  a.invert();
  return polygonsToMeshGeometry(a.allPolygons(), density);
}
