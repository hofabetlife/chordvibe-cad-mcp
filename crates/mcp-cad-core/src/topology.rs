//! Topological types: Vertex, TriangleMesh, and BoundingBox with geometric algorithms.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

pub use crate::tessellation::{TessellationQuality, TessellationSettings};

/// A 3D vertex with position and normal vector.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vertex {
    /// 3D position [x, y, z] in meters (or standard CAD length units).
    pub position: [f64; 3],
    /// Unit surface normal [nx, ny, nz].
    pub normal: [f64; 3],
}

impl Vertex {
    /// Creates a new vertex with position and unit normal.
    pub fn new(position: [f64; 3], normal: [f64; 3]) -> Self {
        Self { position, normal }
    }

    /// Creates a vertex with a specified position and zero normal.
    pub fn from_position(position: [f64; 3]) -> Self {
        Self {
            position,
            normal: [0.0, 0.0, 0.0],
        }
    }

    /// Position as an nalgebra Vector3.
    #[inline]
    pub fn pos_vec(&self) -> Vector3<f64> {
        Vector3::new(self.position[0], self.position[1], self.position[2])
    }

    /// Normal as an nalgebra Vector3.
    #[inline]
    pub fn normal_vec(&self) -> Vector3<f64> {
        Vector3::new(self.normal[0], self.normal[1], self.normal[2])
    }
}

/// An axis-aligned 3D bounding box (AABB).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    /// Minimum corner [min_x, min_y, min_z].
    pub min: [f64; 3],
    /// Maximum corner [max_x, max_y, max_z].
    pub max: [f64; 3],
}

impl BoundingBox {
    /// Creates a new bounding box.
    pub fn new(min: [f64; 3], max: [f64; 3]) -> Self {
        Self { min, max }
    }

    /// Creates an empty (inverted) bounding box.
    pub fn empty() -> Self {
        Self {
            min: [f64::INFINITY, f64::INFINITY, f64::INFINITY],
            max: [f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY],
        }
    }

    /// Computes bounding box enclosing all vertices.
    pub fn from_vertices(vertices: &[Vertex]) -> Self {
        if vertices.is_empty() {
            return Self::new([0.0; 3], [0.0; 3]);
        }

        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];

        for v in vertices {
            for i in 0..3 {
                if v.position[i] < min[i] {
                    min[i] = v.position[i];
                }
                if v.position[i] > max[i] {
                    max[i] = v.position[i];
                }
            }
        }

        Self { min, max }
    }

    /// Dimensions [width, height, depth] along X, Y, Z axes.
    #[inline]
    pub fn dimensions(&self) -> [f64; 3] {
        [
            (self.max[0] - self.min[0]).max(0.0),
            (self.max[1] - self.min[1]).max(0.0),
            (self.max[2] - self.min[2]).max(0.0),
        ]
    }

    /// Center point of the bounding box.
    #[inline]
    pub fn center(&self) -> [f64; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }

    /// Checks if the bounding box has non-positive or inverted volume.
    pub fn is_empty(&self) -> bool {
        self.min[0] > self.max[0] || self.min[1] > self.max[1] || self.min[2] > self.max[2]
    }
}

/// Geometric class of an analytical or mesh face surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum FaceKind {
    /// Planar face (caps, box/prism sides, chamfers, blind hole bottom, keyway walls).
    Plane,
    /// Cylindrical surface (cylinder side wall, round hole wall, shaft journal).
    Cylinder,
    /// Conical surface (cone/frustum side wall, countersink/revolved chamfer).
    Cone,
    /// Spherical surface.
    Sphere,
    /// Toroidal surface or revolved fillet/rounding surface.
    Torus,
    /// Helical surface (drill flute, thread profile `CutHelicalGroove`).
    Helical,
    /// Arbitrary or composite surface (involute tooth profile, wheel rim spoke, untagged mesh).
    #[default]
    Other,
}

#[allow(non_upper_case_globals)]
impl FaceKind {
    /// Alias for [`FaceKind::Plane`].
    pub const Planar: Self = Self::Plane;
    /// Alias for [`FaceKind::Cylinder`].
    pub const Cylindrical: Self = Self::Cylinder;
    /// Alias for [`FaceKind::Cone`].
    pub const Conical: Self = Self::Cone;
    /// Alias for [`FaceKind::Sphere`].
    pub const Spherical: Self = Self::Sphere;
    /// Alias for [`FaceKind::Torus`].
    pub const Toroidal: Self = Self::Torus;
    /// Alias for [`FaceKind::Other`].
    pub const Freeform: Self = Self::Other;

    /// Canonical lowercase snake_case string representation.
    pub fn as_str(&self) -> &'static str {
        match self {
            FaceKind::Plane => "plane",
            FaceKind::Cylinder => "cylinder",
            FaceKind::Cone => "cone",
            FaceKind::Sphere => "sphere",
            FaceKind::Torus => "torus",
            FaceKind::Helical => "helical",
            FaceKind::Other => "other",
        }
    }

    /// Parses a `FaceKind` from a string (case-insensitive).
    pub fn from_str_ignore_case(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "plane" | "planar" | "flat" => Some(FaceKind::Plane),
            "cylinder" | "cylindrical" => Some(FaceKind::Cylinder),
            "cone" | "conical" | "frustum" => Some(FaceKind::Cone),
            "sphere" | "spherical" => Some(FaceKind::Sphere),
            "torus" | "toroidal" | "fillet" => Some(FaceKind::Torus),
            "helical" | "helix" | "thread" => Some(FaceKind::Helical),
            "other" | "freeform" => Some(FaceKind::Other),
            _ => None,
        }
    }
}

/// Metadata for a named topological face of a `TriangleMesh`.
///
/// All dimensional values inside `mcp-cad-core` are stored strictly in SI (`m`, `m²`),
/// and converted to the active `length_unit` at the MCP API boundary (`crates/mcp-cad-server/src/tools/*.rs`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FaceInfo {
    /// Unique non-zero face identifier (`>= 1`) within this `TriangleMesh`.
    /// Value `0` in `TriangleMesh::face_ids` is reserved for untagged triangles.
    pub id: u32,
    /// Canonical stable face name (e.g. `"cap_top"`, `"feat_0:cap_top"`, `"hole_wall:1"`).
    pub name: String,
    /// Additional short or compatible aliases for convenient selector matching
    /// (e.g. `["cap_top", "top", "max_z", "+z"]`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub aliases: Vec<String>,
    /// Local role of the face within the originating primitive/operation (`"cap_top"`, `"side"`, `"hole_wall"`).
    pub role: String,
    /// Geometric surface type of the face.
    pub kind: FaceKind,
    /// Identifier of the originating `FeatureNode::id` (UUID string) or feature key in `FeatureTree`.
    /// `None` for meshes created directly outside the DAG.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin_feature: Option<String>,
    /// Total surface area of the face triangles in m² (SI).
    pub area: f64,
    /// Area-weighted centroid of the face `[x, y, z]` in m (SI):
    /// `sum(area_t * centroid_t) / sum(area_t)`.
    pub centroid: [f64; 3],
    /// A point guaranteed to lie on the face surface itself `[x, y, z]` in m (SI)
    /// (centroid of the triangle belonging to this face that is closest to `centroid`).
    #[serde(default)]
    pub sample_point: [f64; 3],
    /// For `FaceKind::Plane` — unit outward normal vector `[nx, ny, nz]`.
    /// For `Cylinder`, `Cone`, `Torus`, `Helical` — unit axis of revolution `[ax, ay, az]`.
    /// For `Sphere` / `Other` — average outward normal or `[0.0, 0.0, 1.0]`.
    pub normal_or_axis: [f64; 3],
    /// Characteristic radius in m (SI):
    /// - for `Cylinder`, `Sphere`, `Helical` — surface radius `Some(r)`;
    /// - for `Cone` — mean radius `Some(0.5 * (r_bottom + r_top))`;
    /// - for circular/annular/polygonal planar cap `Plane` — outer radius `Some(r_outer)`;
    /// - for rectangular box/prism side faces — `None`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub radius: Option<f64>,
    /// Number of triangles in `TriangleMesh::indices` belonging to this face (`face_ids[t] == id`).
    pub triangles_count: usize,
}

impl FaceInfo {
    /// Creates a face descriptor before computing triangle metrics (`area`, `centroid`, `sample_point`, `triangles_count`).
    pub fn new_descriptor(
        id: u32,
        role: impl Into<String>,
        aliases: Vec<String>,
        kind: FaceKind,
        normal_or_axis: [f64; 3],
        radius: Option<f64>,
    ) -> Self {
        let role_str = role.into();
        Self {
            id,
            name: role_str.clone(),
            aliases,
            role: role_str,
            kind,
            origin_feature: None,
            area: 0.0,
            centroid: [0.0, 0.0, 0.0],
            sample_point: [0.0, 0.0, 0.0],
            normal_or_axis,
            radius,
            triangles_count: 0,
        }
    }
}

/// Information about a detected planar face composed of coplanar triangles.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanarFaceInfo {
    /// Unit normal vector of the planar face [nx, ny, nz].
    pub normal: [f64; 3],
    /// Area-weighted centroid of the planar face [x, y, z].
    pub center: [f64; 3],
    /// Total surface area of the planar face.
    pub area: f64,
    /// Number of triangles comprising this planar face.
    pub triangles_count: usize,
}

/// Comprehensive geometry inspection result for CAD analysis and AI context.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeometryInspection {
    /// Axis-aligned bounding box (min, max, dimensions: [width, height, depth]).
    pub bounding_box: BoundingBox,
    /// Absolute volume of the solid.
    pub volume: f64,
    /// Total surface area.
    pub surface_area: f64,
    /// Center of mass (centroid) [x, y, z] of the homogeneous solid.
    pub center_of_mass: [f64; 3],
    /// Total number of triangular faces in the boundary mesh.
    pub faces_count: usize,
    /// Total number of vertices in the boundary mesh.
    pub vertices_count: usize,
    /// Top detected coplanar planar faces (up to 15, sorted by area descending).
    pub detected_planar_faces: Vec<PlanarFaceInfo>,
}

/// A 3D triangular surface mesh represented by vertex buffers and index buffers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TriangleMesh {
    /// Array of mesh vertices with positions and normals.
    pub vertices: Vec<Vertex>,
    /// Array of triangle vertex indices (3 indices per face).
    pub indices: Vec<[u32; 3]>,
    /// Parallel array of face IDs (1 `u32` per triangle in `indices`).
    /// Invariant: `face_ids.is_empty()` (untagged) OR `face_ids.len() == indices.len()`.
    /// Value `0` denotes an untagged triangle (`FaceKind::Other`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub face_ids: Vec<u32>,
    /// Registry of named topological faces of the solid, ordered by `id` ascending.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub faces: Vec<FaceInfo>,
}

#[inline]
fn clean_zero_vec3(v: [f64; 3]) -> [f64; 3] {
    [
        if v[0].abs() < 1e-15 { 0.0 } else { v[0] },
        if v[1].abs() < 1e-15 { 0.0 } else { v[1] },
        if v[2].abs() < 1e-15 { 0.0 } else { v[2] },
    ]
}

impl Default for TriangleMesh {
    fn default() -> Self {
        Self::empty()
    }
}

impl TriangleMesh {
    /// Creates a new triangle mesh without face tags.
    pub fn new(vertices: Vec<Vertex>, indices: Vec<[u32; 3]>) -> Self {
        Self {
            vertices,
            indices,
            face_ids: Vec::new(),
            faces: Vec::new(),
        }
    }

    /// Creates a new triangle mesh with per-triangle `face_ids` and `faces` descriptors,
    /// automatically computing exact `area`, `centroid`, `sample_point`, and `triangles_count` for each face.
    pub fn with_faces(
        vertices: Vec<Vertex>,
        indices: Vec<[u32; 3]>,
        face_ids: Vec<u32>,
        faces: Vec<FaceInfo>,
    ) -> Self {
        let mut mesh = Self {
            vertices,
            indices,
            face_ids,
            faces,
        };
        mesh.recompute_face_registry();
        mesh
    }

    /// Creates an empty mesh.
    pub fn empty() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            face_ids: Vec::new(),
            faces: Vec::new(),
        }
    }

    /// Recomputes `area`, `centroid`, `sample_point`, `triangles_count` (and planar `normal_or_axis`)
    /// for all registered faces in `self.faces` from `self.vertices`, `self.indices`, and `self.face_ids`.
    pub fn recompute_face_registry(&mut self) {
        if self.indices.is_empty() {
            self.face_ids.clear();
            self.faces.clear();
            return;
        }

        if !self.face_ids.is_empty() && self.face_ids.len() != self.indices.len() {
            self.face_ids.resize(self.indices.len(), 0);
        }

        if self.faces.is_empty() || self.face_ids.is_empty() {
            return;
        }

        for face in &mut self.faces {
            face.area = 0.0;
            face.centroid = [0.0, 0.0, 0.0];
            face.sample_point = [0.0, 0.0, 0.0];
            face.triangles_count = 0;
        }

        let mut weighted_centroids: Vec<Vector3<f64>> = vec![Vector3::zeros(); self.faces.len()];
        let mut accum_normals: Vec<Vector3<f64>> = vec![Vector3::zeros(); self.faces.len()];
        let mut tri_centroids: Vec<[f64; 3]> = Vec::with_capacity(self.indices.len());

        for (t_idx, tri) in self.indices.iter().enumerate() {
            let v0 = self.vertices[tri[0] as usize].pos_vec();
            let v1 = self.vertices[tri[1] as usize].pos_vec();
            let v2 = self.vertices[tri[2] as usize].pos_vec();

            let cross = (v1 - v0).cross(&(v2 - v0));
            let area = 0.5 * cross.norm();
            let c = (v0 + v1 + v2) / 3.0;
            tri_centroids.push([c.x, c.y, c.z]);

            let fid = self.face_ids[t_idx];
            if fid == 0 {
                continue;
            }

            if let Some(pos) = self.faces.iter().position(|f| f.id == fid) {
                self.faces[pos].area += area;
                self.faces[pos].triangles_count += 1;
                weighted_centroids[pos] += c * area;
                accum_normals[pos] += cross;
            }
        }

        for (f_idx, face) in self.faces.iter_mut().enumerate() {
            if face.triangles_count == 0 {
                continue;
            }
            if face.area > 1e-18 {
                let ctr = weighted_centroids[f_idx] / face.area;
                face.centroid = clean_zero_vec3([ctr.x, ctr.y, ctr.z]);
            }
            if face.kind == FaceKind::Plane {
                let n_len = accum_normals[f_idx].norm();
                if n_len > 1e-12 {
                    let n = accum_normals[f_idx] / n_len;
                    face.normal_or_axis = clean_zero_vec3([n.x, n.y, n.z]);
                }
            }

            // Find the triangle belonging to `face.id` whose centroid is closest to `face.centroid`
            let mut best_dist_sq = f64::INFINITY;
            let mut best_pt = face.centroid;
            for (t_idx, &fid) in self.face_ids.iter().enumerate() {
                if fid == face.id {
                    let tc = tri_centroids[t_idx];
                    let dx = tc[0] - face.centroid[0];
                    let dy = tc[1] - face.centroid[1];
                    let dz = tc[2] - face.centroid[2];
                    let d2 = dx * dx + dy * dy + dz * dz;
                    if d2 < best_dist_sq {
                        best_dist_sq = d2;
                        best_pt = tc;
                    }
                }
            }
            face.sample_point = clean_zero_vec3(best_pt);
        }

        self.faces.retain(|f| f.triangles_count > 0);
        self.faces.sort_by_key(|f| f.id);
    }

    /// Number of triangles in the mesh.
    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.indices.len()
    }

    /// Number of vertices in the mesh.
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Computes the axis-aligned bounding box.
    pub fn bounding_box(&self) -> BoundingBox {
        BoundingBox::from_vertices(&self.vertices)
    }

    /// Computes face normal for each triangle.
    pub fn compute_face_normals(&self) -> Vec<[f64; 3]> {
        self.indices
            .iter()
            .map(|tri| {
                let v0 = self.vertices[tri[0] as usize].pos_vec();
                let v1 = self.vertices[tri[1] as usize].pos_vec();
                let v2 = self.vertices[tri[2] as usize].pos_vec();
                let normal = (v1 - v0).cross(&(v2 - v0));
                let norm = normal.norm();
                if norm > 1e-12 {
                    let n = normal / norm;
                    [n.x, n.y, n.z]
                } else {
                    [0.0, 0.0, 1.0]
                }
            })
            .collect()
    }

    /// Recomputes smooth vertex normals by accumulating adjacent face normals.
    pub fn recompute_vertex_normals(&mut self) {
        let mut accum_normals = vec![Vector3::zeros(); self.vertices.len()];

        for tri in &self.indices {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;

            let v0 = self.vertices[i0].pos_vec();
            let v1 = self.vertices[i1].pos_vec();
            let v2 = self.vertices[i2].pos_vec();

            let fnorm = (v1 - v0).cross(&(v2 - v0));
            accum_normals[i0] += fnorm;
            accum_normals[i1] += fnorm;
            accum_normals[i2] += fnorm;
        }

        for (v, acc) in self.vertices.iter_mut().zip(accum_normals.iter()) {
            let norm = acc.norm();
            if norm > 1e-12 {
                let n = acc / norm;
                v.normal = [n.x, n.y, n.z];
            }
        }
    }

    /// Calculates total surface area by summing triangle areas.
    pub fn surface_area(&self) -> f64 {
        let mut area = 0.0;
        for tri in &self.indices {
            let v0 = self.vertices[tri[0] as usize].pos_vec();
            let v1 = self.vertices[tri[1] as usize].pos_vec();
            let v2 = self.vertices[tri[2] as usize].pos_vec();
            let cross = (v1 - v0).cross(&(v2 - v0));
            area += 0.5 * cross.norm();
        }
        area
    }

    /// Calculates the signed volume of a closed mesh using the divergence theorem:
    /// V = 1/6 * sum( v0 . (v1 x v2) )
    pub fn signed_volume(&self) -> f64 {
        let mut sum = 0.0;
        for tri in &self.indices {
            let v0 = self.vertices[tri[0] as usize].pos_vec();
            let v1 = self.vertices[tri[1] as usize].pos_vec();
            let v2 = self.vertices[tri[2] as usize].pos_vec();
            sum += v0.dot(&v1.cross(&v2));
        }
        sum / 6.0
    }

    /// Absolute volume of the closed mesh.
    pub fn volume(&self) -> f64 {
        self.signed_volume().abs()
    }

    /// Computes the exact center of mass (centroid) of a homogeneous solid
    /// bounded by this triangle mesh using signed tetrahedra decomposition.
    pub fn center_of_mass(&self) -> [f64; 3] {
        let mut total_vol = 0.0;
        let mut weighted_center = Vector3::zeros();

        for tri in &self.indices {
            let v0 = self.vertices[tri[0] as usize].pos_vec();
            let v1 = self.vertices[tri[1] as usize].pos_vec();
            let v2 = self.vertices[tri[2] as usize].pos_vec();

            // Signed volume of tetrahedron with origin
            let vol = v0.dot(&v1.cross(&v2)) / 6.0;
            // Centroid of tetrahedron with origin is 1/4 (v0 + v1 + v2 + 0)
            let tet_center = (v0 + v1 + v2) * 0.25;

            weighted_center += tet_center * vol;
            total_vol += vol;
        }

        if total_vol.abs() > 1e-12 {
            let com = weighted_center / total_vol;
            [com.x, com.y, com.z]
        } else {
            // Fallback to bounding box center for non-closed or zero-volume meshes
            self.bounding_box().center()
        }
    }

    /// Translates all vertices of the mesh by the given vector [dx, dy, dz].
    pub fn translate(&mut self, translation: [f64; 3]) {
        for v in &mut self.vertices {
            v.position[0] += translation[0];
            v.position[1] += translation[1];
            v.position[2] += translation[2];
        }
        for face in &mut self.faces {
            face.centroid = clean_zero_vec3([
                face.centroid[0] + translation[0],
                face.centroid[1] + translation[1],
                face.centroid[2] + translation[2],
            ]);
            face.sample_point = clean_zero_vec3([
                face.sample_point[0] + translation[0],
                face.sample_point[1] + translation[1],
                face.sample_point[2] + translation[2],
            ]);
        }
    }

    /// Scales all vertices of the mesh along X, Y, Z axes.
    pub fn scale(&mut self, scale_factors: [f64; 3]) {
        for v in &mut self.vertices {
            v.position[0] *= scale_factors[0];
            v.position[1] *= scale_factors[1];
            v.position[2] *= scale_factors[2];

            // Correct normal under non-uniform scaling: N' = (M^-1)^T * N
            let nx = v.normal[0] / scale_factors[0];
            let ny = v.normal[1] / scale_factors[1];
            let nz = v.normal[2] / scale_factors[2];
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len > 1e-12 {
                v.normal = [nx / len, ny / len, nz / len];
            }
        }

        // If an odd number of axes have negative scale factors (mirror reflection),
        // reverse triangle winding to preserve outward CCW face orientation and positive signed volume.
        if scale_factors[0] * scale_factors[1] * scale_factors[2] < 0.0 {
            for tri in &mut self.indices {
                tri.swap(1, 2);
            }
        }

        if !self.faces.is_empty() {
            let sx = scale_factors[0].abs();
            let sy = scale_factors[1].abs();
            let sz = scale_factors[2].abs();
            for face in &mut self.faces {
                if face.kind != FaceKind::Plane {
                    let ax = face.normal_or_axis[0];
                    let ay = face.normal_or_axis[1];
                    let az = face.normal_or_axis[2];
                    // Compute transverse scale factor perpendicular to axis
                    let perp_scale = if face.kind == FaceKind::Sphere {
                        (sx * sy * sz).cbrt()
                    } else {
                        let w_x = 1.0 - ax * ax;
                        let w_y = 1.0 - ay * ay;
                        let w_z = 1.0 - az * az;
                        let w_sum = (w_x + w_y + w_z).max(1e-12);
                        ((w_x * sx * sx + w_y * sy * sy + w_z * sz * sz) / w_sum).sqrt()
                    };
                    if let Some(ref mut r) = face.radius {
                        *r *= perp_scale;
                    }
                    let nax = ax * scale_factors[0];
                    let nay = ay * scale_factors[1];
                    let naz = az * scale_factors[2];
                    let n_len = (nax * nax + nay * nay + naz * naz).sqrt();
                    if n_len > 1e-12 {
                        face.normal_or_axis =
                            clean_zero_vec3([nax / n_len, nay / n_len, naz / n_len]);
                    }
                } else if let Some(ref mut r) = face.radius {
                    let nx = face.normal_or_axis[0];
                    let ny = face.normal_or_axis[1];
                    let nz = face.normal_or_axis[2];
                    let w_x = 1.0 - nx * nx;
                    let w_y = 1.0 - ny * ny;
                    let w_z = 1.0 - nz * nz;
                    let w_sum = (w_x + w_y + w_z).max(1e-12);
                    let perp_scale =
                        ((w_x * sx * sx + w_y * sy * sy + w_z * sz * sz) / w_sum).sqrt();
                    *r *= perp_scale;
                }
            }
            self.recompute_face_registry();
        }
    }

    /// Rotates the mesh around X, Y, Z axes using Roll-Pitch-Yaw angles in degrees.
    pub fn rotate_rpy_deg(&mut self, roll_deg: f64, pitch_deg: f64, yaw_deg: f64) {
        let r = roll_deg.to_radians();
        let p = pitch_deg.to_radians();
        let y = yaw_deg.to_radians();

        let rot = nalgebra::Rotation3::from_euler_angles(r, p, y);

        for v in &mut self.vertices {
            let pos = rot * v.pos_vec();
            let n = rot * v.normal_vec();
            v.position = [pos.x, pos.y, pos.z];
            v.normal = [n.x, n.y, n.z];
        }
        for face in &mut self.faces {
            let c = rot * Vector3::new(face.centroid[0], face.centroid[1], face.centroid[2]);
            let s = rot
                * Vector3::new(
                    face.sample_point[0],
                    face.sample_point[1],
                    face.sample_point[2],
                );
            let n = rot
                * Vector3::new(
                    face.normal_or_axis[0],
                    face.normal_or_axis[1],
                    face.normal_or_axis[2],
                );
            face.centroid = clean_zero_vec3([c.x, c.y, c.z]);
            face.sample_point = clean_zero_vec3([s.x, s.y, s.z]);
            face.normal_or_axis = clean_zero_vec3([n.x, n.y, n.z]);
        }
    }

    /// Transforms the mesh by rotating by RPY angles in degrees, then translating by translation vector.
    pub fn transform(&mut self, translation: [f64; 3], rotation_deg: [f64; 3]) {
        let r = rotation_deg[0].to_radians();
        let p = rotation_deg[1].to_radians();
        let y = rotation_deg[2].to_radians();

        let rot = nalgebra::Rotation3::from_euler_angles(r, p, y);
        let t = Vector3::new(translation[0], translation[1], translation[2]);

        for v in &mut self.vertices {
            let pos = rot * v.pos_vec() + t;
            let n = rot * v.normal_vec();
            v.position = [pos.x, pos.y, pos.z];
            v.normal = [n.x, n.y, n.z];
        }
        for face in &mut self.faces {
            let c = rot * Vector3::new(face.centroid[0], face.centroid[1], face.centroid[2]) + t;
            let s = rot
                * Vector3::new(
                    face.sample_point[0],
                    face.sample_point[1],
                    face.sample_point[2],
                )
                + t;
            let n = rot
                * Vector3::new(
                    face.normal_or_axis[0],
                    face.normal_or_axis[1],
                    face.normal_or_axis[2],
                );
            face.centroid = clean_zero_vec3([c.x, c.y, c.z]);
            face.sample_point = clean_zero_vec3([s.x, s.y, s.z]);
            face.normal_or_axis = clean_zero_vec3([n.x, n.y, n.z]);
        }
    }

    /// Inspects the mesh geometry and returns detailed geometric metrics including
    /// bounding box, volume, surface area, center of mass, face/vertex counts,
    /// and up to top-15 largest detected coplanar planar faces.
    ///
    /// Triangles with close normals (dot product n1 · n2 > 0.999) and lying in
    /// the same plane (|n · p - d| < 1e-4) are clustered into planar faces,
    /// limited to top-15 sorted by area in descending order.
    pub fn inspect_geometry(&self) -> GeometryInspection {
        let bounding_box = self.bounding_box();
        let volume = self.volume();
        let surface_area = self.surface_area();
        let center_of_mass = self.center_of_mass();
        let faces_count = self.triangle_count();
        let vertices_count = self.vertex_count();

        if self.indices.is_empty() {
            return GeometryInspection {
                bounding_box,
                volume,
                surface_area,
                center_of_mass,
                faces_count,
                vertices_count,
                detected_planar_faces: Vec::new(),
            };
        }

        struct TriInfo {
            normal: Vector3<f64>,
            centroid: Vector3<f64>,
            area: f64,
            d: f64,
        }

        let mut triangles = Vec::with_capacity(self.indices.len());
        for tri in &self.indices {
            let v0 = self.vertices[tri[0] as usize].pos_vec();
            let v1 = self.vertices[tri[1] as usize].pos_vec();
            let v2 = self.vertices[tri[2] as usize].pos_vec();

            let cross = (v1 - v0).cross(&(v2 - v0));
            let cross_len = cross.norm();
            let area = 0.5 * cross_len;
            if area > 1e-12 {
                let normal = cross / cross_len;
                let centroid = (v0 + v1 + v2) / 3.0;
                let d = normal.dot(&centroid);
                triangles.push(TriInfo {
                    normal,
                    centroid,
                    area,
                    d,
                });
            }
        }

        // Sort triangles descending by area so largest triangles seed planar clusters
        triangles.sort_by(|a, b| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal));

        struct Cluster {
            seed_normal: Vector3<f64>,
            seed_d: f64,
            weighted_normal: Vector3<f64>,
            weighted_center: Vector3<f64>,
            area: f64,
            triangles_count: usize,
        }

        let mut clusters: Vec<Cluster> = Vec::new();

        for tri in triangles {
            let mut best_match: Option<usize> = None;
            let mut best_dot = 0.999;

            for (idx, cluster) in clusters.iter().enumerate() {
                let dot = cluster.seed_normal.dot(&tri.normal);
                if dot > best_dot {
                    // Check if triangle centroid is coplanar with cluster plane: |n · p - d| < 1e-4
                    let dist = (cluster.seed_normal.dot(&tri.centroid) - cluster.seed_d).abs();
                    if dist < 1e-4 {
                        best_match = Some(idx);
                        best_dot = dot;
                    }
                }
            }

            if let Some(idx) = best_match {
                let cluster = &mut clusters[idx];
                cluster.weighted_normal += tri.normal * tri.area;
                cluster.weighted_center += tri.centroid * tri.area;
                cluster.area += tri.area;
                cluster.triangles_count += 1;
            } else {
                clusters.push(Cluster {
                    seed_normal: tri.normal,
                    seed_d: tri.d,
                    weighted_normal: tri.normal * tri.area,
                    weighted_center: tri.centroid * tri.area,
                    area: tri.area,
                    triangles_count: 1,
                });
            }
        }

        let mut detected_planar_faces: Vec<PlanarFaceInfo> = clusters
            .into_iter()
            .map(|c| {
                let normal = if c.weighted_normal.norm() > 1e-12 {
                    let n = c.weighted_normal.normalize();
                    [n.x, n.y, n.z]
                } else {
                    [c.seed_normal.x, c.seed_normal.y, c.seed_normal.z]
                };

                let center = if c.area > 1e-12 {
                    let ctr = c.weighted_center / c.area;
                    [ctr.x, ctr.y, ctr.z]
                } else {
                    [0.0, 0.0, 0.0]
                };

                PlanarFaceInfo {
                    normal,
                    center,
                    area: c.area,
                    triangles_count: c.triangles_count,
                }
            })
            .collect();

        // Sort descending by area and limit to top 15
        detected_planar_faces.sort_by(|a, b| {
            b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
        });
        detected_planar_faces.truncate(15);

        GeometryInspection {
            bounding_box,
            volume,
            surface_area,
            center_of_mass,
            faces_count,
            vertices_count,
            detected_planar_faces,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_mesh() {
        let mesh = TriangleMesh::empty();
        assert_eq!(mesh.triangle_count(), 0);
        assert_eq!(mesh.vertex_count(), 0);
        assert_eq!(mesh.surface_area(), 0.0);
        assert_eq!(mesh.volume(), 0.0);
    }

    #[test]
    fn test_bounding_box() {
        let vertices = vec![
            Vertex::from_position([-1.0, 2.0, -3.0]),
            Vertex::from_position([4.0, -5.0, 6.0]),
        ];
        let aabb = BoundingBox::from_vertices(&vertices);
        assert_eq!(aabb.min, [-1.0, -5.0, -3.0]);
        assert_eq!(aabb.max, [4.0, 2.0, 6.0]);
        assert_eq!(aabb.dimensions(), [5.0, 7.0, 9.0]);
        assert_eq!(aabb.center(), [1.5, -1.5, 1.5]);
    }

    #[test]
    fn test_mesh_transform() {
        let mut mesh = TriangleMesh::new(
            vec![
                Vertex::new([1.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
                Vertex::new([0.0, 1.0, 0.0], [0.0, 1.0, 0.0]),
                Vertex::new([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]),
            ],
            vec![[0, 1, 2]],
        );

        // Rotate 90 deg around Z, translate by [5.0, -2.0, 3.0]
        // [1, 0, 0] rotated 90 deg around Z -> [0, 1, 0] -> translated -> [5.0, -1.0, 3.0]
        mesh.transform([5.0, -2.0, 3.0], [0.0, 0.0, 90.0]);

        let v0 = mesh.vertices[0];
        assert!((v0.position[0] - 5.0).abs() < 1e-6);
        assert!((v0.position[1] - (-1.0)).abs() < 1e-6);
        assert!((v0.position[2] - 3.0).abs() < 1e-6);

        // Normal should only be rotated, not translated
        assert!(v0.normal[0].abs() < 1e-6);
        assert!((v0.normal[1] - 1.0).abs() < 1e-6);
        assert!(v0.normal[2].abs() < 1e-6);
    }

    #[test]
    fn test_inspect_geometry_empty() {
        let mesh = TriangleMesh::empty();
        let insp = mesh.inspect_geometry();
        assert_eq!(insp.faces_count, 0);
        assert_eq!(insp.vertices_count, 0);
        assert_eq!(insp.volume, 0.0);
        assert_eq!(insp.surface_area, 0.0);
        assert!(insp.detected_planar_faces.is_empty());
    }

    #[test]
    fn test_inspect_geometry_box() {
        let box_mesh = crate::primitives::create_box(10.0, 20.0, 30.0);
        let insp = box_mesh.inspect_geometry();

        assert_eq!(insp.faces_count, 12);
        assert_eq!(insp.vertices_count, 24);
        assert!((insp.volume - 6000.0).abs() < 1e-4);
        // Surface area: 2 * (10*20 + 20*30 + 10*30) = 2 * (200 + 600 + 300) = 2200
        assert!((insp.surface_area - 2200.0).abs() < 1e-4);
        assert_eq!(insp.bounding_box.dimensions(), [10.0, 20.0, 30.0]);
        assert_eq!(insp.bounding_box.center(), [0.0, 0.0, 0.0]);
        assert_eq!(insp.center_of_mass, [0.0, 0.0, 0.0]);

        // Box has 6 rectangular planar faces, each composed of 2 triangles
        assert_eq!(insp.detected_planar_faces.len(), 6);

        // Sorted descending by area:
        // Top 2 faces: 20 x 30 = 600 each
        assert!((insp.detected_planar_faces[0].area - 600.0).abs() < 1e-4);
        assert_eq!(insp.detected_planar_faces[0].triangles_count, 2);
        assert!((insp.detected_planar_faces[1].area - 600.0).abs() < 1e-4);
        assert_eq!(insp.detected_planar_faces[1].triangles_count, 2);

        // Next 2 faces: 10 x 30 = 300 each
        assert!((insp.detected_planar_faces[2].area - 300.0).abs() < 1e-4);
        assert_eq!(insp.detected_planar_faces[2].triangles_count, 2);
        assert!((insp.detected_planar_faces[3].area - 300.0).abs() < 1e-4);
        assert_eq!(insp.detected_planar_faces[3].triangles_count, 2);

        // Next 2 faces: 10 x 20 = 200 each
        assert!((insp.detected_planar_faces[4].area - 200.0).abs() < 1e-4);
        assert_eq!(insp.detected_planar_faces[4].triangles_count, 2);
        assert!((insp.detected_planar_faces[5].area - 200.0).abs() < 1e-4);
        assert_eq!(insp.detected_planar_faces[5].triangles_count, 2);

        let total_detected_area: f64 = insp.detected_planar_faces.iter().map(|f| f.area).sum();
        assert!((total_detected_area - 2200.0).abs() < 1e-4);
    }
}
