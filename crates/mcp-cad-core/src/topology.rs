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

    /// Returns true if the mesh contains no vertices or no triangles.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty() || self.indices.is_empty()
    }

    /// Appends the vertices and triangles from another mesh into `self`.
    pub fn append(&mut self, other: &Self) {
        let v_offset = self.vertices.len() as u32;
        self.vertices.extend(other.vertices.iter().cloned());
        for tri in &other.indices {
            self.indices.push([
                tri[0] + v_offset,
                tri[1] + v_offset,
                tri[2] + v_offset,
            ]);
        }
        if !other.face_ids.is_empty() {
            if self.face_ids.is_empty() {
                self.face_ids.resize(self.indices.len() - other.indices.len(), 0);
            }
            self.face_ids.extend(other.face_ids.iter().copied());
        } else if !self.face_ids.is_empty() {
            self.face_ids.resize(self.indices.len(), 0);
        }
        self.faces.extend(other.faces.iter().cloned());
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

    /// Recomputes vertex normals while preserving sharp crease balance and preventing
    /// large-area faces or diagonal triangulation asymmetry from skewing corner normals.
    ///
    /// Incident triangle normals at each vertex are grouped into smooth clusters
    /// (crease angle threshold 45°, `cos >= 0.7071`), weighted by interior corner angle
    /// within each smooth cluster, and normalized so each distinct face/smooth surface patch
    /// meeting at a sharp edge or corner contributes an equal unit normal vector.
    pub fn recompute_vertex_normals(&mut self) {
        const CREASE_COS_THRESHOLD: f64 = std::f64::consts::FRAC_1_SQRT_2; // cos(45°)
        let mut vertex_groups: Vec<Vec<(Vector3<f64>, Vector3<f64>)>> =
            vec![Vec::new(); self.vertices.len()];

        for tri in &self.indices {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;

            let v0 = self.vertices[i0].pos_vec();
            let v1 = self.vertices[i1].pos_vec();
            let v2 = self.vertices[i2].pos_vec();

            let e01 = v1 - v0;
            let e12 = v2 - v1;
            let e20 = v0 - v2;

            let cross = e01.cross(&(-e20));
            let cross_norm = cross.norm();
            if cross_norm <= 1e-18 {
                continue;
            }
            let unit_n = cross / cross_norm;

            let l01 = e01.norm();
            let l12 = e12.norm();
            let l20 = e20.norm();

            let a0 = ((-e20).dot(&e01) / (l20 * l01).max(1e-30))
                .clamp(-1.0, 1.0)
                .acos()
                .max(1e-6);
            let a1 = ((-e01).dot(&e12) / (l01 * l12).max(1e-30))
                .clamp(-1.0, 1.0)
                .acos()
                .max(1e-6);
            let a2 = ((-e12).dot(&e20) / (l12 * l20).max(1e-30))
                .clamp(-1.0, 1.0)
                .acos()
                .max(1e-6);

            for &(vi, angle) in &[(i0, a0), (i1, a1), (i2, a2)] {
                let groups = &mut vertex_groups[vi];
                let mut matched = false;
                for (rep_dir, accum) in groups.iter_mut() {
                    if rep_dir.dot(&unit_n) >= CREASE_COS_THRESHOLD {
                        *accum += unit_n * angle;
                        let n_len = accum.norm();
                        if n_len > 1e-12 {
                            *rep_dir = *accum / n_len;
                        }
                        matched = true;
                        break;
                    }
                }
                if !matched {
                    groups.push((unit_n, unit_n * angle));
                }
            }
        }

        for (v, groups) in self.vertices.iter_mut().zip(vertex_groups.iter()) {
            if groups.is_empty() {
                continue;
            }

            let mut unit_groups: Vec<Vector3<f64>> = Vec::with_capacity(groups.len());
            let mut sum = Vector3::zeros();
            for &(rep_dir, accum) in groups {
                let g_len = accum.norm();
                let g_unit = if g_len > 1e-12 { accum / g_len } else { rep_dir };
                unit_groups.push(g_unit);
                sum += g_unit;
            }

            // If multiple sharp faces meet at an acute wedge/corner, iteratively stabilize
            // the combined normal so it maintains a positive dot product with all incident faces.
            let mut n = if sum.norm() > 1e-12 {
                sum.normalize()
            } else {
                unit_groups[0]
            };

            if unit_groups.len() > 1 {
                for _ in 0..8 {
                    let mut min_dot = f64::INFINITY;
                    let mut min_g = unit_groups[0];
                    for &g in &unit_groups {
                        let d = n.dot(&g);
                        if d < min_dot {
                            min_dot = d;
                            min_g = g;
                        }
                    }
                    if min_dot >= 0.05 {
                        break;
                    }
                    let blended = n + min_g * (0.25 - min_dot).max(0.1);
                    if blended.norm() > 1e-12 {
                        n = blended.normalize();
                    }
                }
            }

            v.normal = clean_zero_vec3([n.x, n.y, n.z]);
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
}

fn closest_point_on_triangle(
    p: &Vector3<f64>,
    a: &Vector3<f64>,
    b: &Vector3<f64>,
    c: &Vector3<f64>,
) -> Vector3<f64> {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return *a;
    }

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return *b;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab * v;
    }

    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return *c;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac * w;
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a + ab * v + ac * w
}

#[inline]
fn point_to_segment_dist_sq(p: &Vector3<f64>, a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    let ab = b - a;
    let len_sq = ab.norm_squared();
    if len_sq <= 1e-30 {
        return (p - a).norm_squared();
    }
    let t = ((p - a).dot(&ab) / len_sq).clamp(0.0, 1.0);
    let proj = a + ab * t;
    (p - proj).norm_squared()
}

#[inline]
fn uf_find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

#[inline]
fn uf_union(parent: &mut [usize], i: usize, j: usize) {
    let ri = uf_find(parent, i);
    let rj = uf_find(parent, j);
    if ri != rj {
        parent[rj] = ri;
    }
}

impl TriangleMesh {
    /// Inspects the mesh geometry and returns detailed geometric metrics including
    /// bounding box, volume, surface area, center of mass, face/vertex counts,
    /// and up to top-15 largest detected connected coplanar planar faces.
    ///
    /// Triangles with close normals (dot product n1 · n2 > 0.999), lying in
    /// the same plane within a scale-relative tolerance (`|n · p - d| < diag * 1e-5`),
    /// and belonging to the same connected surface component are clustered into
    /// planar faces, limited to top-15 sorted by area in descending order.
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

        let dims = bounding_box.dimensions();
        let diag = (dims[0] * dims[0] + dims[1] * dims[1] + dims[2] * dims[2]).sqrt();
        let plane_tol = (diag * 1e-5).max(1e-12);
        let vertex_weld_tol = (diag * 1e-6).max(1e-12);
        let vertex_weld_tol_sq = vertex_weld_tol * vertex_weld_tol;
        let min_tri_area = (diag * diag * 1e-12).max(1e-18);

        // Weld coincident 3D vertices within `vertex_weld_tol` so triangles that duplicate
        // Vertex structs at shared corners/edges are recognized as topologically connected.
        let inv_cell = 1.0 / vertex_weld_tol;
        let mut grid: std::collections::HashMap<[i64; 3], Vec<(usize, Vector3<f64>)>> =
            std::collections::HashMap::with_capacity(self.vertices.len());
        let mut welded_vid_of = Vec::with_capacity(self.vertices.len());
        let mut next_welded_id = 0usize;

        for v in &self.vertices {
            let p = v.pos_vec();
            let cx = (p.x * inv_cell).floor() as i64;
            let cy = (p.y * inv_cell).floor() as i64;
            let cz = (p.z * inv_cell).floor() as i64;
            let mut found_id = None;
            'search: for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(bucket) = grid.get(&[cx + dx, cy + dy, cz + dz]) {
                            for &(wid, wp) in bucket {
                                if (p - wp).norm_squared() <= vertex_weld_tol_sq {
                                    found_id = Some(wid);
                                    break 'search;
                                }
                            }
                        }
                    }
                }
            }
            let wid = match found_id {
                Some(id) => id,
                None => {
                    let id = next_welded_id;
                    next_welded_id += 1;
                    grid.entry([cx, cy, cz]).or_default().push((id, p));
                    id
                }
            };
            welded_vid_of.push(wid);
        }

        struct TriInfo {
            v0: Vector3<f64>,
            v1: Vector3<f64>,
            v2: Vector3<f64>,
            welded: [usize; 3],
            aabb_min: [f64; 3],
            aabb_max: [f64; 3],
            normal: Vector3<f64>,
            centroid: Vector3<f64>,
            area: f64,
            d: f64,
        }

        let mut triangles = Vec::with_capacity(self.indices.len());
        for tri in &self.indices {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;
            let v0 = self.vertices[i0].pos_vec();
            let v1 = self.vertices[i1].pos_vec();
            let v2 = self.vertices[i2].pos_vec();

            let cross = (v1 - v0).cross(&(v2 - v0));
            let cross_len = cross.norm();
            let area = 0.5 * cross_len;
            if area > min_tri_area {
                let normal = cross / cross_len;
                let centroid = (v0 + v1 + v2) / 3.0;
                let d = normal.dot(&centroid);
                let aabb_min = [
                    v0.x.min(v1.x).min(v2.x),
                    v0.y.min(v1.y).min(v2.y),
                    v0.z.min(v1.z).min(v2.z),
                ];
                let aabb_max = [
                    v0.x.max(v1.x).max(v2.x),
                    v0.y.max(v1.y).max(v2.y),
                    v0.z.max(v1.z).max(v2.z),
                ];
                triangles.push(TriInfo {
                    v0,
                    v1,
                    v2,
                    welded: [welded_vid_of[i0], welded_vid_of[i1], welded_vid_of[i2]],
                    aabb_min,
                    aabb_max,
                    normal,
                    centroid,
                    area,
                    d,
                });
            }
        }

        // Sort triangles descending by area so largest triangles seed planar groups
        triangles.sort_by(|a, b| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal));

        struct PlaneGroup {
            seed_normal: Vector3<f64>,
            seed_d: f64,
            tris: Vec<TriInfo>,
        }

        let mut plane_groups: Vec<PlaneGroup> = Vec::new();

        for tri in triangles {
            let mut best_match: Option<usize> = None;
            let mut best_dot = 0.999;

            for (idx, group) in plane_groups.iter().enumerate() {
                let dot = group.seed_normal.dot(&tri.normal);
                if dot > best_dot {
                    let dist = (group.seed_normal.dot(&tri.centroid) - group.seed_d).abs();
                    if dist < plane_tol {
                        best_match = Some(idx);
                        best_dot = dot;
                    }
                }
            }

            if let Some(idx) = best_match {
                plane_groups[idx].tris.push(tri);
            } else {
                plane_groups.push(PlaneGroup {
                    seed_normal: tri.normal,
                    seed_d: tri.d,
                    tris: vec![tri],
                });
            }
        }

        let mut detected_planar_faces: Vec<PlanarFaceInfo> = Vec::new();

        for group in plane_groups {
            let n_tris = group.tris.len();
            if n_tris == 0 {
                continue;
            }

            // Split coplanar triangles into connected components.
            // 1. Connect triangles that share at least one welded vertex.
            let mut parent: Vec<usize> = (0..n_tris).collect();
            let mut vid_owner: std::collections::HashMap<usize, usize> =
                std::collections::HashMap::with_capacity(n_tris * 3);

            for (t_idx, tri) in group.tris.iter().enumerate() {
                for &wid in &tri.welded {
                    if let Some(&first_t) = vid_owner.get(&wid) {
                        uf_union(&mut parent, first_t, t_idx);
                    } else {
                        vid_owner.insert(wid, t_idx);
                    }
                }
            }

            // 2. If multiple components remain in the same plane, check for T-junction contacts
            // (a vertex of one triangle lying within `vertex_weld_tol` of an edge of another).
            let mut distinct_roots: Vec<usize> = (0..n_tris).map(|i| uf_find(&mut parent, i)).collect();
            distinct_roots.sort_unstable();
            distinct_roots.dedup();

            if distinct_roots.len() > 1 {
                for i in 0..n_tris {
                    for j in (i + 1)..n_tris {
                        if uf_find(&mut parent, i) == uf_find(&mut parent, j) {
                            continue;
                        }
                        let ta = &group.tris[i];
                        let tb = &group.tris[j];
                        if ta.aabb_max[0] + vertex_weld_tol < tb.aabb_min[0]
                            || tb.aabb_max[0] + vertex_weld_tol < ta.aabb_min[0]
                            || ta.aabb_max[1] + vertex_weld_tol < tb.aabb_min[1]
                            || tb.aabb_max[1] + vertex_weld_tol < ta.aabb_min[1]
                            || ta.aabb_max[2] + vertex_weld_tol < tb.aabb_min[2]
                            || tb.aabb_max[2] + vertex_weld_tol < ta.aabb_min[2]
                        {
                            continue;
                        }
                        let a_verts = [&ta.v0, &ta.v1, &ta.v2];
                        let b_verts = [&tb.v0, &tb.v1, &tb.v2];
                        let b_edges = [(&tb.v0, &tb.v1), (&tb.v1, &tb.v2), (&tb.v2, &tb.v0)];
                        let a_edges = [(&ta.v0, &ta.v1), (&ta.v1, &ta.v2), (&ta.v2, &ta.v0)];

                        let mut touches = false;
                        'tcheck: for &va in &a_verts {
                            for &(eb0, eb1) in &b_edges {
                                if point_to_segment_dist_sq(va, eb0, eb1) <= vertex_weld_tol_sq {
                                    touches = true;
                                    break 'tcheck;
                                }
                            }
                        }
                        if !touches {
                            'tcheck2: for &vb in &b_verts {
                                for &(ea0, ea1) in &a_edges {
                                    if point_to_segment_dist_sq(vb, ea0, ea1) <= vertex_weld_tol_sq {
                                        touches = true;
                                        break 'tcheck2;
                                    }
                                }
                            }
                        }
                        if touches {
                            uf_union(&mut parent, i, j);
                        }
                    }
                }
            }

            let mut components: std::collections::HashMap<usize, Vec<&TriInfo>> =
                std::collections::HashMap::new();
            for i in 0..n_tris {
                let root = uf_find(&mut parent, i);
                components.entry(root).or_default().push(&group.tris[i]);
            }

            for comp_tris in components.into_values() {
                let mut weighted_normal = Vector3::zeros();
                let mut weighted_center = Vector3::zeros();
                let mut area = 0.0;
                let triangles_count = comp_tris.len();

                for tri in &comp_tris {
                    weighted_normal += tri.normal * tri.area;
                    weighted_center += tri.centroid * tri.area;
                    area += tri.area;
                }

                let normal_vec = if weighted_normal.norm() > 1e-12 {
                    weighted_normal.normalize()
                } else {
                    group.seed_normal
                };
                let normal = [normal_vec.x, normal_vec.y, normal_vec.z];

                let center = if area > min_tri_area {
                    let raw_ctr = weighted_center / area;
                    // Verify whether the area-weighted centroid lies on the face surface.
                    // For annular faces (or faces with central cutouts), raw_ctr falls in the hole void.
                    let mut min_dist_sq = f64::INFINITY;
                    let mut nearest_pt = raw_ctr;
                    for tri in &comp_tris {
                        let q = closest_point_on_triangle(&raw_ctr, &tri.v0, &tri.v1, &tri.v2);
                        let d_sq = (raw_ctr - q).norm_squared();
                        if d_sq < min_dist_sq {
                            min_dist_sq = d_sq;
                            nearest_pt = q;
                        }
                    }

                    let surface_tol_sq = (plane_tol * 2.0) * (plane_tol * 2.0);
                    if min_dist_sq <= surface_tol_sq {
                        [raw_ctr.x, raw_ctr.y, raw_ctr.z]
                    } else {
                        // Project outward from raw_ctr through nearest_pt to find the mid-span on the face
                        let dir = nearest_pt - raw_ctr;
                        let dir_len = dir.norm();
                        let chosen = if dir_len > 1e-12 {
                            let u = dir / dir_len;
                            let t_enter = dir_len;
                            let mut t_exit = t_enter;
                            for tri in &comp_tris {
                                for v in [&tri.v0, &tri.v1, &tri.v2] {
                                    let rel = v - raw_ctr;
                                    let along = rel.dot(&u);
                                    if along > t_enter {
                                        let perp_sq = (rel - u * along).norm_squared();
                                        if perp_sq <= (along * 0.25).powi(2) + surface_tol_sq {
                                            t_exit = t_exit.max(along);
                                        }
                                    }
                                }
                            }
                            let mid_candidate = raw_ctr + u * (0.5 * (t_enter + t_exit));
                            // Snap candidate to the closest triangle of the component to guarantee it lies on the face
                            let mut best_q = comp_tris[0].centroid;
                            let mut best_q_dist_sq = f64::INFINITY;
                            for tri in &comp_tris {
                                let q = closest_point_on_triangle(
                                    &mid_candidate,
                                    &tri.v0,
                                    &tri.v1,
                                    &tri.v2,
                                );
                                let d_sq = (mid_candidate - q).norm_squared();
                                if d_sq < best_q_dist_sq {
                                    best_q_dist_sq = d_sq;
                                    best_q = q;
                                }
                            }
                            best_q
                        } else {
                            comp_tris[0].centroid
                        };
                        [chosen.x, chosen.y, chosen.z]
                    }
                } else {
                    [0.0, 0.0, 0.0]
                };

                detected_planar_faces.push(PlanarFaceInfo {
                    normal,
                    center,
                    area,
                    triangles_count,
                });
            }
        }

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

    #[test]
    fn test_inspect_geometry_disjoint_coplanar_pads_and_micro_steps() {
        // 1. Two disjoint coplanar pads at the exact same Z height (+0.005 m)
        // Base plate: 0.020 x 0.010 x 0.004 m centered at z = 0.002 (spans z in [0.0, 0.004])
        // Left pad at z = 0.005, Right pad at z = 0.005 (separated by gap in X)
        let mut mesh = crate::primitives::create_box(0.004, 0.006, 0.002);
        mesh.translate([-0.006, 0.0, 0.005]);
        let mut right_pad = crate::primitives::create_box(0.004, 0.006, 0.002);
        right_pad.translate([0.006, 0.0, 0.005]);
        mesh.append(&right_pad);

        let insp = mesh.inspect_geometry();
        // Both pads have top faces at z = +0.006 with normal [0, 0, 1], each of area 0.004 * 0.006 = 2.4e-5 m^2.
        // Because they are spatially disjoint, they must NOT merge into a single PlanarFaceInfo!
        let top_faces: Vec<&PlanarFaceInfo> = insp
            .detected_planar_faces
            .iter()
            .filter(|f| f.normal[2] > 0.99 && (f.center[2] - 0.006).abs() < 1e-6)
            .collect();
        assert_eq!(
            top_faces.len(),
            2,
            "Two disjoint coplanar pads at Z=0.006 must be detected as 2 separate planar faces, got {:?}",
            top_faces
        );
        for tf in &top_faces {
            assert!((tf.area - 2.4e-5).abs() < 1e-9);
            assert!(tf.center[0].abs() > 0.004, "Pad center X must lie on its respective pad");
        }

        // 2. Micro-stepped geometry in SI meters where step height is 2e-5 m (0.02 mm < old 1e-4 m threshold)
        // on a 2 mm (0.002 m) scale part:
        let mut step_low = crate::primitives::create_box(0.001, 0.002, 0.001);
        step_low.translate([-0.0005, 0.0, 0.0]);
        let mut step_high = crate::primitives::create_box(0.001, 0.002, 0.001);
        step_high.translate([0.0005, 0.0, 2.0e-5]); // 20 microns higher in Z
        step_low.append(&step_high);

        let step_insp = step_low.inspect_geometry();
        let z_top_faces: Vec<&PlanarFaceInfo> = step_insp
            .detected_planar_faces
            .iter()
            .filter(|f| f.normal[2] > 0.99)
            .collect();
        assert_eq!(
            z_top_faces.len(),
            2,
            "Parallel top faces separated by a 20um step on a 2mm part must not merge: {:?}",
            z_top_faces
        );
    }

    #[test]
    fn test_inspect_geometry_annular_face_center_lies_on_surface() {
        // Annular bushing created via revolve: r_in = 0.015 m, r_out = 0.030 m, h = 0.020 m
        let profile = vec![
            [0.015, -0.010],
            [0.030, -0.010],
            [0.030, 0.010],
            [0.015, 0.010],
        ];
        let mesh = crate::primitives::create_revolve(&profile, 360.0, 32);
        let insp = mesh.inspect_geometry();

        // Find top and bottom annular planar caps (normal Z = +1 and Z = -1)
        let cap_faces: Vec<&PlanarFaceInfo> = insp
            .detected_planar_faces
            .iter()
            .filter(|f| f.normal[2].abs() > 0.99)
            .collect();
        assert_eq!(cap_faces.len(), 2, "Annular bushing must have 2 planar end caps");

        for cap in cap_faces {
            let r = cap.center[0].hypot(cap.center[1]);
            assert!(
                (0.015 - 1e-4..=0.030 + 1e-4).contains(&r),
                "Annular cap center {:?} (r={}) must lie on the annular surface [0.015, 0.030], not in the central bore hole!",
                cap.center,
                r
            );
        }
    }
}
