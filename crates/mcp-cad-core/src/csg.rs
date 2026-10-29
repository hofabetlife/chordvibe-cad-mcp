//! Constructive Solid Geometry (CSG) operations via Binary Space Partitioning (BSP) trees.
//! Provides Boolean operations: Union, Difference, and Intersection on 3D meshes.

use crate::topology::{TriangleMesh, Vertex};
use nalgebra::Vector3;

const EPSILON: f64 = 1e-6;

#[derive(Debug, Clone, Copy, PartialEq)]
enum VertexType {
    Coplanar,
    Front,
    Back,
}

/// A 3D infinite plane defined by equation: normal . p - w = 0
#[derive(Debug, Clone, PartialEq)]
pub struct Plane {
    pub normal: Vector3<f64>,
    pub w: f64,
}

impl Plane {
    /// Creates a plane from 3 non-collinear points.
    pub fn from_points(a: Vector3<f64>, b: Vector3<f64>, c: Vector3<f64>) -> Option<Self> {
        let normal = (b - a).cross(&(c - a));
        let len = normal.norm();
        if len < 1e-12 {
            return None;
        }
        let unit_n = normal / len;
        let w = unit_n.dot(&a);
        Some(Self { normal: unit_n, w })
    }

    /// Flips plane orientation.
    pub fn flip(&mut self) {
        self.normal = -self.normal;
        self.w = -self.w;
    }

    /// Splits a polygon by this plane into coplanar_front, coplanar_back, front, back lists.
    pub fn split_polygon(
        &self,
        poly: &CsgPolygon,
        coplanar_front: &mut Vec<CsgPolygon>,
        coplanar_back: &mut Vec<CsgPolygon>,
        front: &mut Vec<CsgPolygon>,
        back: &mut Vec<CsgPolygon>,
    ) {
        let mut types = Vec::with_capacity(poly.vertices.len());
        let mut has_front = false;
        let mut has_back = false;

        for v in &poly.vertices {
            let t = self.normal.dot(&v.pos_vec()) - self.w;
            let vtype = if t > EPSILON {
                has_front = true;
                VertexType::Front
            } else if t < -EPSILON {
                has_back = true;
                VertexType::Back
            } else {
                VertexType::Coplanar
            };
            types.push(vtype);
        }

        if !has_front && !has_back {
            // Entire polygon is coplanar with this plane
            if self.normal.dot(&poly.plane.normal) > 0.0 {
                coplanar_front.push(poly.clone());
            } else {
                coplanar_back.push(poly.clone());
            }
        } else if has_front && !has_back {
            front.push(poly.clone());
        } else if !has_front && has_back {
            back.push(poly.clone());
        } else {
            // SPANNING: polygon crosses the plane, split it
            let mut f_verts = Vec::new();
            let mut b_verts = Vec::new();

            let count = poly.vertices.len();
            for i in 0..count {
                let j = (i + 1) % count;
                let ti = types[i];
                let tj = types[j];
                let vi = &poly.vertices[i];
                let vj = &poly.vertices[j];

                if ti != VertexType::Back {
                    f_verts.push(*vi);
                }
                if ti != VertexType::Front {
                    b_verts.push(*vi);
                }

                if (ti == VertexType::Front && tj == VertexType::Back)
                    || (ti == VertexType::Back && tj == VertexType::Front)
                {
                    // Compute intersection along edge vi -> vj
                    let pi = vi.pos_vec();
                    let pj = vj.pos_vec();
                    let denom = self.normal.dot(&(pj - pi));
                    let t = if denom.abs() > 1e-12 {
                        ((self.w - self.normal.dot(&pi)) / denom).clamp(0.0, 1.0)
                    } else {
                        0.5
                    };

                    let p_mid = pi + (pj - pi) * t;
                    let n_mid = vi.normal_vec() + (vj.normal_vec() - vi.normal_vec()) * t;
                    let n_norm = n_mid.norm();
                    let n_unit = if n_norm > 1e-12 {
                        n_mid / n_norm
                    } else {
                        poly.plane.normal
                    };

                    let v_mid = Vertex::new([p_mid.x, p_mid.y, p_mid.z], [n_unit.x, n_unit.y, n_unit.z]);
                    f_verts.push(v_mid);
                    b_verts.push(v_mid);
                }
            }

            if f_verts.len() >= 3 {
                front.push(CsgPolygon {
                    vertices: f_verts,
                    plane: poly.plane.clone(),
                });
            }
            if b_verts.len() >= 3 {
                back.push(CsgPolygon {
                    vertices: b_verts,
                    plane: poly.plane.clone(),
                });
            }
        }
    }
}

/// A convex planar polygon carrying vertex normals and a plane equation.
#[derive(Debug, Clone, PartialEq)]
pub struct CsgPolygon {
    pub vertices: Vec<Vertex>,
    pub plane: Plane,
}

impl CsgPolygon {
    /// Creates a polygon from a list of vertices.
    pub fn new(vertices: Vec<Vertex>) -> Option<Self> {
        if vertices.len() < 3 {
            return None;
        }
        let plane = Plane::from_points(
            vertices[0].pos_vec(),
            vertices[1].pos_vec(),
            vertices[2].pos_vec(),
        )?;
        Some(Self { vertices, plane })
    }

    /// Flips vertex winding order and normal orientation.
    pub fn flip(&mut self) {
        self.vertices.reverse();
        for v in &mut self.vertices {
            v.normal = [-v.normal[0], -v.normal[1], -v.normal[2]];
        }
        self.plane.flip();
    }
}

/// A node in the Binary Space Partitioning (BSP) tree.
#[derive(Debug, Clone, Default)]
pub struct BspNode {
    pub plane: Option<Plane>,
    pub front: Option<Box<BspNode>>,
    pub back: Option<Box<BspNode>>,
    pub polygons: Vec<CsgPolygon>,
}

impl BspNode {
    /// Creates an empty BSP node.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builds a BSP tree from a triangle mesh.
    pub fn from_mesh(mesh: &TriangleMesh) -> Self {
        let mut polygons = Vec::with_capacity(mesh.indices.len());
        for tri in &mesh.indices {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];
            if let Some(poly) = CsgPolygon::new(vec![v0, v1, v2]) {
                polygons.push(poly);
            }
        }
        let mut node = Self::new();
        node.build(polygons);
        node
    }

    /// Inverts all polygons in the tree and swaps front/back subtrees.
    pub fn invert(&mut self) {
        for poly in &mut self.polygons {
            poly.flip();
        }
        if let Some(ref mut p) = self.plane {
            p.flip();
        }
        if let Some(ref mut f) = self.front {
            f.invert();
        }
        if let Some(ref mut b) = self.back {
            b.invert();
        }
        std::mem::swap(&mut self.front, &mut self.back);
    }

    /// Recursively clips a list of polygons against this BSP tree.
    pub fn clip_polygons(&self, list: &[CsgPolygon]) -> Vec<CsgPolygon> {
        if self.plane.is_none() {
            return list.to_vec();
        }
        let plane = self.plane.as_ref().unwrap();

        let mut front_list = Vec::new();
        let mut back_list = Vec::new();
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();

        for poly in list {
            plane.split_polygon(
                poly,
                &mut coplanar_front,
                &mut coplanar_back,
                &mut front_list,
                &mut back_list,
            );
        }

        front_list.extend(coplanar_front);
        back_list.extend(coplanar_back);

        let mut result_front = if let Some(ref f) = self.front {
            f.clip_polygons(&front_list)
        } else {
            front_list
        };

        let result_back = if let Some(ref b) = self.back {
            b.clip_polygons(&back_list)
        } else {
            // Inside the solid; discard back polygons
            Vec::new()
        };

        result_front.extend(result_back);
        result_front
    }

    /// Clips all polygons in this tree against another BSP tree.
    pub fn clip_to(&mut self, other: &BspNode) {
        self.polygons = other.clip_polygons(&self.polygons);
        if let Some(ref mut f) = self.front {
            f.clip_to(other);
        }
        if let Some(ref mut b) = self.back {
            b.clip_to(other);
        }
    }

    /// Collects all polygons in this BSP tree into a flat vector.
    pub fn all_polygons(&self) -> Vec<CsgPolygon> {
        let mut result = self.polygons.clone();
        if let Some(ref f) = self.front {
            result.extend(f.all_polygons());
        }
        if let Some(ref b) = self.back {
            result.extend(b.all_polygons());
        }
        result
    }

    /// Adds a list of polygons to the BSP tree.
    pub fn build(&mut self, polygons: Vec<CsgPolygon>) {
        if polygons.is_empty() {
            return;
        }

        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.as_ref().unwrap();

        let mut front_list = Vec::new();
        let mut back_list = Vec::new();
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();

        for poly in polygons {
            plane.split_polygon(
                &poly,
                &mut coplanar_front,
                &mut coplanar_back,
                &mut front_list,
                &mut back_list,
            );
        }

        self.polygons.extend(coplanar_front);
        self.polygons.extend(coplanar_back);

        if !front_list.is_empty() {
            if self.front.is_none() {
                self.front = Some(Box::new(BspNode::new()));
            }
            self.front.as_mut().unwrap().build(front_list);
        }

        if !back_list.is_empty() {
            if self.back.is_none() {
                self.back = Some(Box::new(BspNode::new()));
            }
            self.back.as_mut().unwrap().build(back_list);
        }
    }
}

/// Converts a collection of CSG polygons back into a triangulated surface mesh.
pub fn polygons_to_mesh(polygons: &[CsgPolygon]) -> TriangleMesh {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    for poly in polygons {
        if poly.vertices.len() < 3 {
            continue;
        }

        // Triangulate convex polygon as a triangle fan from vertex 0
        let base_idx = vertices.len() as u32;
        vertices.extend_from_slice(&poly.vertices);

        for i in 1..(poly.vertices.len() - 1) {
            let i0 = base_idx;
            let i1 = base_idx + i as u32;
            let i2 = base_idx + (i + 1) as u32;

            // Check triangle area to omit degenerate triangles
            let v0 = vertices[i0 as usize].pos_vec();
            let v1 = vertices[i1 as usize].pos_vec();
            let v2 = vertices[i2 as usize].pos_vec();
            let cross = (v1 - v0).cross(&(v2 - v0));
            if cross.norm_squared() > 1e-16 {
                indices.push([i0, i1, i2]);
            }
        }
    }

    TriangleMesh::new(vertices, indices)
}

/// Computes Boolean Union of two meshes: A ∪ B.
pub fn csg_union(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    if a.indices.is_empty() {
        return b.clone();
    }
    if b.indices.is_empty() {
        return a.clone();
    }

    let mut a_tree = BspNode::from_mesh(a);
    let mut b_tree = BspNode::from_mesh(b);

    a_tree.clip_to(&b_tree);
    b_tree.clip_to(&a_tree);
    b_tree.invert();
    b_tree.clip_to(&a_tree);
    b_tree.invert();

    a_tree.build(b_tree.all_polygons());
    polygons_to_mesh(&a_tree.all_polygons())
}

/// Computes Boolean Difference of two meshes: A \ B (A minus B).
pub fn csg_difference(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    if a.indices.is_empty() {
        return TriangleMesh::empty();
    }
    if b.indices.is_empty() {
        return a.clone();
    }

    let mut a_tree = BspNode::from_mesh(a);
    let mut b_tree = BspNode::from_mesh(b);

    a_tree.invert();
    a_tree.clip_to(&b_tree);
    b_tree.clip_to(&a_tree);
    b_tree.invert();
    b_tree.clip_to(&a_tree);
    b_tree.invert();

    a_tree.build(b_tree.all_polygons());
    a_tree.invert();
    polygons_to_mesh(&a_tree.all_polygons())
}

/// Computes Boolean Intersection of two meshes: A ∩ B.
pub fn csg_intersection(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    if a.indices.is_empty() || b.indices.is_empty() {
        return TriangleMesh::empty();
    }

    let mut a_tree = BspNode::from_mesh(a);
    let mut b_tree = BspNode::from_mesh(b);

    a_tree.invert();
    b_tree.clip_to(&a_tree);
    b_tree.invert();
    a_tree.clip_to(&b_tree);
    b_tree.clip_to(&a_tree);

    a_tree.build(b_tree.all_polygons());
    a_tree.invert();
    polygons_to_mesh(&a_tree.all_polygons())
}
