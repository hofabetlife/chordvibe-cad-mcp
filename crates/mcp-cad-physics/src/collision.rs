//! Double-precision (`f64`) broad-phase and narrow-phase collision detection pipeline (`P4-T4` / `ADR-004`).
//!
//! Pure-Rust `f64` collision detection and contact manifold generation pipeline:
//! - **Broad-Phase**: Sweep-and-Prune on world-space AABBs with filtering of same-body colliders,
//!   static–static pairs, and joint-connected pairs with `contacts_enabled == false`.
//! - **TriMesh BVH**: Cached local-space binary AABB bounding volume hierarchy (`TriMeshBvh`)
//!   accelerating triangle queries to $O(\log T)$.
//! - **Narrow-Phase**:
//!   - Analytical primitive tests (`Sphere`–`Sphere`, `Sphere`–`Cube`, `Sphere`–`Cylinder`,
//!     `Cube`–`Cube` via 15-axis SAT + Sutherland–Hodgman face clipping);
//!   - Universal `f64` GJK + EPA for all convex shape pairs (`ConvexHull`, `Cylinder`, `Cube`,
//!     `Sphere`, and `Compound` convex sub-parts) with planar contact manifold extraction;
//!   - BVH-accelerated `TriMesh` narrow-phase (exact `Sphere`–triangle and GJK/EPA
//!     triangle–convex with outward-normal validation and coplanar manifold reduction).

use std::collections::{HashMap, HashSet};

use mcp_cad_core::topology::BoundingBox;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use crate::rigid_body::{BodyHandle, BodyRegistry, Collider, ColliderHandle, ColliderShape};
use crate::solver::SolvedJointRecord;

/// A single `f64` contact point produced by the collision pipeline.
#[derive(Debug, Clone, PartialEq)]
pub struct ContactPoint {
    /// Dense solver index of Body A.
    pub body_a: usize,
    /// Dense solver index of Body B.
    pub body_b: usize,
    /// Part identifier of Body A.
    pub part_a: String,
    /// Part identifier of Body B.
    pub part_b: String,
    /// World-space contact point on the interface ($\text{m}$).
    pub point: Vector3<f64>,
    /// Unit contact normal pointing **from Body A toward Body B**.
    pub normal: Vector3<f64>,
    /// Penetration depth $\delta > 0$ ($\text{m}$).
    pub depth: f64,
    /// Combined coefficient of restitution $e \in [0, 1]$.
    pub restitution: f64,
    /// Combined Coulomb friction coefficient $\mu \ge 0$.
    pub friction: f64,
}

/// Internal narrow-phase contact result prior to body/material attachment.
#[derive(Debug, Clone, Copy)]
struct RawContact {
    point: Vector3<f64>,
    /// Unit normal pointing from shape A toward shape B.
    normal: Vector3<f64>,
    depth: f64,
}

// ============================================================================
// Cached TriMesh BVH (AABB Tree) in f64
// ============================================================================

const BVH_LEAF_MAX_TRIS: usize = 4;

#[derive(Debug, Clone, PartialEq)]
enum BvhNode {
    Internal {
        aabb: BoundingBox,
        left: usize,
        right: usize,
    },
    Leaf {
        aabb: BoundingBox,
        tri_indices: Vec<usize>,
    },
}

impl BvhNode {
    fn aabb(&self) -> &BoundingBox {
        match self {
            Self::Internal { aabb, .. } | Self::Leaf { aabb, .. } => aabb,
        }
    }
}

/// Cached local-space Bounding Volume Hierarchy (AABB-tree) for a [`ColliderShape::TriMesh`].
#[derive(Debug, Clone, PartialEq)]
pub struct TriMeshBvh {
    nodes: Vec<BvhNode>,
    tri_aabbs: Vec<BoundingBox>,
}

impl TriMeshBvh {
    /// Builds a balanced top-down AABB BVH from `vertices` and `indices` in `f64`.
    #[must_use]
    pub fn build(vertices: &[[f64; 3]], indices: &[[u32; 3]]) -> Self {
        let mut tri_aabbs = Vec::with_capacity(indices.len());
        let mut valid_tris: Vec<(usize, [f64; 3])> = Vec::with_capacity(indices.len());

        for (t_idx, tri) in indices.iter().enumerate() {
            let (Some(&v0), Some(&v1), Some(&v2)) = (
                vertices.get(tri[0] as usize),
                vertices.get(tri[1] as usize),
                vertices.get(tri[2] as usize),
            ) else {
                tri_aabbs.push(BoundingBox::new([0.0; 3], [0.0; 3]));
                continue;
            };
            let min = [
                v0[0].min(v1[0]).min(v2[0]),
                v0[1].min(v1[1]).min(v2[1]),
                v0[2].min(v1[2]).min(v2[2]),
            ];
            let max = [
                v0[0].max(v1[0]).max(v2[0]),
                v0[1].max(v1[1]).max(v2[1]),
                v0[2].max(v1[2]).max(v2[2]),
            ];
            tri_aabbs.push(BoundingBox::new(min, max));
            let centroid = [
                (v0[0] + v1[0] + v2[0]) / 3.0,
                (v0[1] + v1[1] + v2[1]) / 3.0,
                (v0[2] + v1[2] + v2[2]) / 3.0,
            ];
            valid_tris.push((t_idx, centroid));
        }

        let mut nodes = Vec::new();
        if !valid_tris.is_empty() {
            Self::build_recursive(&mut valid_tris, &tri_aabbs, &mut nodes);
        }

        Self { nodes, tri_aabbs }
    }

    fn build_recursive(
        items: &mut [(usize, [f64; 3])],
        tri_aabbs: &[BoundingBox],
        nodes: &mut Vec<BvhNode>,
    ) -> usize {
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        let mut c_min = [f64::INFINITY; 3];
        let mut c_max = [f64::NEG_INFINITY; 3];

        for &(t_idx, c) in items.iter() {
            let b = &tri_aabbs[t_idx];
            for k in 0..3 {
                min[k] = min[k].min(b.min[k]);
                max[k] = max[k].max(b.max[k]);
                c_min[k] = c_min[k].min(c[k]);
                c_max[k] = c_max[k].max(c[k]);
            }
        }
        let aabb = BoundingBox::new(min, max);

        if items.len() <= BVH_LEAF_MAX_TRIS {
            let idx = nodes.len();
            nodes.push(BvhNode::Leaf {
                aabb,
                tri_indices: items.iter().map(|&(t, _)| t).collect(),
            });
            return idx;
        }

        let span = [
            c_max[0] - c_min[0],
            c_max[1] - c_min[1],
            c_max[2] - c_min[2],
        ];
        let axis = if span[0] >= span[1] && span[0] >= span[2] {
            0
        } else if span[1] >= span[2] {
            1
        } else {
            2
        };

        let mid = items.len() / 2;
        items.select_nth_unstable_by(mid, |a, b| {
            a.1[axis]
                .partial_cmp(&b.1[axis])
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let node_idx = nodes.len();
        nodes.push(BvhNode::Internal {
            aabb,
            left: 0,
            right: 0,
        });

        let (left_slice, right_slice) = items.split_at_mut(mid);
        let left = Self::build_recursive(left_slice, tri_aabbs, nodes);
        let right = Self::build_recursive(right_slice, tri_aabbs, nodes);
        nodes[node_idx] = BvhNode::Internal { aabb, left, right };
        node_idx
    }

    /// Returns `true` if the BVH contains no triangles.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Returns the number of BVH nodes.
    #[inline]
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Queries all triangle indices whose local AABB overlaps `query`.
    pub fn query_local_aabb(&self, query: &BoundingBox, out: &mut Vec<usize>) {
        out.clear();
        if self.nodes.is_empty() || !aabbs_overlap(query, self.nodes[0].aabb()) {
            return;
        }
        let mut stack = Vec::with_capacity(64);
        stack.push(0usize);

        while let Some(node_idx) = stack.pop() {
            match &self.nodes[node_idx] {
                BvhNode::Leaf {
                    aabb, tri_indices, ..
                } => {
                    if aabbs_overlap(query, aabb) {
                        for &t in tri_indices {
                            if aabbs_overlap(query, &self.tri_aabbs[t]) {
                                out.push(t);
                            }
                        }
                    }
                }
                BvhNode::Internal { aabb, left, right } => {
                    if aabbs_overlap(query, aabb) {
                        stack.push(*right);
                        stack.push(*left);
                    }
                }
            }
        }
    }
}

#[inline]
fn aabbs_overlap(a: &BoundingBox, b: &BoundingBox) -> bool {
    !(a.max[0] < b.min[0]
        || a.min[0] > b.max[0]
        || a.max[1] < b.min[1]
        || a.min[1] > b.max[1]
        || a.max[2] < b.min[2]
        || a.min[2] > b.max[2])
}

fn world_aabb_to_local(
    world_aabb: &BoundingBox,
    pos: Vector3<f64>,
    rot: UnitQuaternion<f64>,
    margin: f64,
) -> BoundingBox {
    let inv_rot = rot.inverse();
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for ix in [world_aabb.min[0], world_aabb.max[0]] {
        for iy in [world_aabb.min[1], world_aabb.max[1]] {
            for iz in [world_aabb.min[2], world_aabb.max[2]] {
                let lp = inv_rot * (Vector3::new(ix, iy, iz) - pos);
                min[0] = min[0].min(lp.x);
                min[1] = min[1].min(lp.y);
                min[2] = min[2].min(lp.z);
                max[0] = max[0].max(lp.x);
                max[1] = max[1].max(lp.y);
                max[2] = max[2].max(lp.z);
            }
        }
    }
    BoundingBox::new(
        [min[0] - margin, min[1] - margin, min[2] - margin],
        [max[0] + margin, max[1] + margin, max[2] + margin],
    )
}

// ============================================================================
// Broad-Phase + Top-Level Detection Entrypoint
// ============================================================================

/// Runs broad-phase AABB filtering and narrow-phase `f64` contact detection across all
/// active colliders, returning deterministic contact manifolds for the multi-body solver.
#[must_use]
pub fn detect_contacts(
    bodies: &BodyRegistry,
    colliders: &[(ColliderHandle, &Collider)],
    joints: &[SolvedJointRecord],
    handle_to_idx: &HashMap<BodyHandle, usize>,
) -> Vec<ContactPoint> {
    if colliders.len() < 2 {
        return Vec::new();
    }

    let mut disabled_pairs = HashSet::new();
    for j in joints {
        if !j.data.contacts_enabled {
            let pair = if j.body_a <= j.body_b {
                (j.body_a, j.body_b)
            } else {
                (j.body_b, j.body_a)
            };
            disabled_pairs.insert(pair);
        }
    }

    // Broad-phase: Sweep-and-Prune sorted along X axis
    let mut sorted: Vec<usize> = (0..colliders.len()).collect();
    sorted.sort_by(|&a, &b| {
        colliders[a]
            .1
            .world_aabb
            .min[0]
            .partial_cmp(&colliders[b].1.world_aabb.min[0])
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.cmp(&b))
    });

    let mut candidate_pairs: Vec<(usize, usize)> = Vec::new();
    for s_i in 0..sorted.len() {
        let idx_i = sorted[s_i];
        let ca = colliders[idx_i].1;
        let aabb_a = ca.world_aabb;

        for &idx_j in &sorted[(s_i + 1)..] {
            let cb = colliders[idx_j].1;
            let aabb_b = cb.world_aabb;
            if aabb_b.min[0] > aabb_a.max[0] {
                break;
            }
            if aabb_a.max[1] < aabb_b.min[1]
                || aabb_a.min[1] > aabb_b.max[1]
                || aabb_a.max[2] < aabb_b.min[2]
                || aabb_a.min[2] > aabb_b.max[2]
            {
                continue;
            }

            let (Some(ha), Some(hb)) = (ca.parent, cb.parent) else {
                continue;
            };
            if ha == hb {
                continue;
            }
            let pair_key = if ha <= hb { (ha, hb) } else { (hb, ha) };
            if disabled_pairs.contains(&pair_key) {
                continue;
            }
            let (Some(ba), Some(bb)) = (bodies.get(ha), bodies.get(hb)) else {
                continue;
            };
            if !ba.is_dynamic && !bb.is_dynamic {
                continue;
            }
            let pair = if idx_i < idx_j {
                (idx_i, idx_j)
            } else {
                (idx_j, idx_i)
            };
            candidate_pairs.push(pair);
        }
    }
    candidate_pairs.sort_unstable();

    // Narrow-phase dispatch
    let mut contacts = Vec::new();
    for (idx_a, idx_b) in candidate_pairs {
        let ca = colliders[idx_a].1;
        let cb = colliders[idx_b].1;
        let (Some(ha), Some(hb)) = (ca.parent, cb.parent) else {
            continue;
        };
        let (Some(ba), Some(bb)) = (bodies.get(ha), bodies.get(hb)) else {
            continue;
        };
        let (Some(&ia), Some(&ib)) = (handle_to_idx.get(&ha), handle_to_idx.get(&hb)) else {
            continue;
        };

        let raw_contacts = collide_colliders(ca, ba.orientation, cb, bb.orientation);
        if raw_contacts.is_empty() {
            continue;
        }

        let restitution = 0.5 * (ca.restitution + cb.restitution);
        let friction = (ca.friction * cb.friction).sqrt();

        for rc in raw_contacts {
            if rc.depth > 1e-12 && rc.normal.norm_squared() > 0.5 {
                contacts.push(ContactPoint {
                    body_a: ia,
                    body_b: ib,
                    part_a: ba.part_id.clone(),
                    part_b: bb.part_id.clone(),
                    point: rc.point,
                    normal: rc.normal.normalize(),
                    depth: rc.depth,
                    restitution,
                    friction,
                });
            }
        }
    }

    contacts
}

// ============================================================================
// Narrow-Phase Shape Dispatch & Support Functions
// ============================================================================

#[derive(Clone, Copy)]
enum ConvexView<'a> {
    Sphere {
        center: Vector3<f64>,
        radius: f64,
    },
    Cube {
        center: Vector3<f64>,
        rot: Matrix3<f64>,
        half_extents: Vector3<f64>,
    },
    Cylinder {
        center: Vector3<f64>,
        axis: Vector3<f64>,
        half_height: f64,
        radius: f64,
    },
    ConvexHull {
        pos: Vector3<f64>,
        rot: Matrix3<f64>,
        points: &'a [[f64; 3]],
    },
    TriPrism {
        verts: [Vector3<f64>; 6],
    },
}

impl<'a> ConvexView<'a> {
    fn from_simple_collider(col: &'a Collider, body_rot: UnitQuaternion<f64>) -> Option<Self> {
        let pos = col.world_translation;
        let rot = *col.world_rotation.to_rotation_matrix().matrix();
        match &col.shape {
            ColliderShape::Sphere { radius } => Some(Self::Sphere {
                center: pos,
                radius: *radius,
            }),
            ColliderShape::Cube { half_extents } => Some(Self::Cube {
                center: pos,
                rot,
                half_extents: Vector3::from(*half_extents),
            }),
            ColliderShape::Cylinder {
                half_height,
                radius,
                axis,
            } => {
                let local_ax = Vector3::from(*axis);
                let ax = if local_ax.norm() > 1e-12 {
                    (body_rot * local_ax).normalize()
                } else {
                    (body_rot * Vector3::y()).normalize()
                };
                Some(Self::Cylinder {
                    center: pos,
                    axis: ax,
                    half_height: *half_height,
                    radius: *radius,
                })
            }
            ColliderShape::ConvexHull { points, .. } => {
                if points.is_empty() {
                    None
                } else {
                    Some(Self::ConvexHull { pos, rot, points })
                }
            }
            ColliderShape::TriMesh { .. } | ColliderShape::Compound { .. } => None,
        }
    }

    fn center(&self) -> Vector3<f64> {
        match self {
            Self::Sphere { center, .. }
            | Self::Cube { center, .. }
            | Self::Cylinder { center, .. } => *center,
            Self::ConvexHull { pos, rot, points } => {
                let mut sum = Vector3::zeros();
                for p in *points {
                    sum += Vector3::from(*p);
                }
                let avg = if points.is_empty() {
                    Vector3::zeros()
                } else {
                    sum / (points.len() as f64)
                };
                *pos + *rot * avg
            }
            Self::TriPrism { verts } => {
                (verts[0] + verts[1] + verts[2] + verts[3] + verts[4] + verts[5]) / 6.0
            }
        }
    }

    fn support(&self, dir: Vector3<f64>) -> Vector3<f64> {
        let d = if dir.norm_squared() > 1e-24 {
            dir.normalize()
        } else {
            Vector3::x()
        };
        match self {
            Self::Sphere { center, radius } => *center + d * (*radius),
            Self::Cube {
                center,
                rot,
                half_extents,
            } => {
                let local_d = rot.transpose() * d;
                let local_p = Vector3::new(
                    if local_d.x >= 0.0 {
                        half_extents.x
                    } else {
                        -half_extents.x
                    },
                    if local_d.y >= 0.0 {
                        half_extents.y
                    } else {
                        -half_extents.y
                    },
                    if local_d.z >= 0.0 {
                        half_extents.z
                    } else {
                        -half_extents.z
                    },
                );
                *center + *rot * local_p
            }
            Self::Cylinder {
                center,
                axis,
                half_height,
                radius,
            } => {
                let d_ax = d.dot(axis);
                let axial = (*axis) * if d_ax >= 0.0 { *half_height } else { -(*half_height) };
                let d_perp = d - (*axis) * d_ax;
                let perp_norm = d_perp.norm();
                let radial = if perp_norm > 1e-12 {
                    (d_perp / perp_norm) * (*radius)
                } else {
                    Vector3::zeros()
                };
                *center + axial + radial
            }
            Self::ConvexHull { pos, rot, points } => {
                let local_d = rot.transpose() * d;
                let mut best = Vector3::from(points[0]);
                let mut best_dot = best.dot(&local_d);
                for &pt in &points[1..] {
                    let v = Vector3::from(pt);
                    let dot = v.dot(&local_d);
                    if dot > best_dot {
                        best_dot = dot;
                        best = v;
                    }
                }
                *pos + *rot * best
            }
            Self::TriPrism { verts } => {
                let mut best = verts[0];
                let mut best_dot = best.dot(&d);
                for &v in &verts[1..] {
                    let dot = v.dot(&d);
                    if dot > best_dot {
                        best_dot = dot;
                        best = v;
                    }
                }
                best
            }
        }
    }

    /// Collects coplanar extreme face vertices along `dir` (for polyhedral manifold clipping).
    fn planar_face_polygon(&self, dir: Vector3<f64>) -> Option<(f64, Vec<Vector3<f64>>)> {
        let d = dir.normalize();
        match self {
            Self::Cube {
                center,
                rot,
                half_extents,
            } => {
                let local_d = rot.transpose() * d;
                let mut best_ax = 0usize;
                let mut best_abs = local_d[0].abs();
                for k in 1..3 {
                    if local_d[k].abs() > best_abs {
                        best_abs = local_d[k].abs();
                        best_ax = k;
                    }
                }
                if best_abs < 0.99 {
                    return None;
                }
                let s = if local_d[best_ax] >= 0.0 { 1.0 } else { -1.0 };
                let face_u = (best_ax + 1) % 3;
                let face_v = (best_ax + 2) % 3;
                let mut corners = Vec::with_capacity(4);
                for (su, sv) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                    let mut lp = Vector3::zeros();
                    lp[best_ax] = s * half_extents[best_ax];
                    lp[face_u] = su * half_extents[face_u];
                    lp[face_v] = sv * half_extents[face_v];
                    corners.push(*center + *rot * lp);
                }
                let h = corners[0].dot(&d);
                Some((h, corners))
            }
            Self::Cylinder {
                center,
                axis,
                half_height,
                radius,
            } => {
                let ax_dot = d.dot(axis);
                if ax_dot.abs() < 0.995 {
                    return None;
                }
                let s = if ax_dot >= 0.0 { 1.0 } else { -1.0 };
                let cap_center = *center + (*axis) * (s * (*half_height));
                let (t1, t2) = orthonormal_pair(axis);
                let mut poly = Vec::with_capacity(8);
                for k in 0..8 {
                    let ang = (k as f64) * std::f64::consts::FRAC_PI_4;
                    poly.push(cap_center + t1 * (radius * ang.cos()) + t2 * (radius * ang.sin()));
                }
                Some((cap_center.dot(&d), poly))
            }
            Self::ConvexHull { pos, rot, points } => {
                let local_d = rot.transpose() * d;
                let mut max_proj = f64::NEG_INFINITY;
                for &pt in *points {
                    max_proj = max_proj.max(Vector3::from(pt).dot(&local_d));
                }
                let mut min_p = [f64::INFINITY; 3];
                let mut max_p = [f64::NEG_INFINITY; 3];
                for &pt in *points {
                    for k in 0..3 {
                        min_p[k] = min_p[k].min(pt[k]);
                        max_p[k] = max_p[k].max(pt[k]);
                    }
                }
                let diag = ((max_p[0] - min_p[0]).powi(2)
                    + (max_p[1] - min_p[1]).powi(2)
                    + (max_p[2] - min_p[2]).powi(2))
                .sqrt()
                .max(1e-4);
                let tol = (diag * 1e-4).clamp(1e-6, 5e-4);

                let mut world_pts = Vec::new();
                for &pt in *points {
                    let lv = Vector3::from(pt);
                    if max_proj - lv.dot(&local_d) <= tol {
                        world_pts.push(*pos + *rot * lv);
                    }
                }
                if world_pts.len() < 3 {
                    return None;
                }
                let h = world_pts[0].dot(&d);
                Some((h, world_pts))
            }
            Self::Sphere { .. } | Self::TriPrism { .. } => None,
        }
    }
}

fn collide_colliders(
    ca: &Collider,
    body_rot_a: UnitQuaternion<f64>,
    cb: &Collider,
    body_rot_b: UnitQuaternion<f64>,
) -> Vec<RawContact> {
    // Handle Compound on either side
    if let ColliderShape::Compound { parts: parts_a } = &ca.shape {
        let mut out = Vec::new();
        for part_a in parts_a {
            let sub_aabb = transform_local_aabb(&part_a.local_aabb, ca.world_translation, ca.world_rotation);
            if !aabbs_overlap(&sub_aabb, &cb.world_aabb) {
                continue;
            }
            let view_a = ConvexView::ConvexHull {
                pos: ca.world_translation,
                rot: *ca.world_rotation.to_rotation_matrix().matrix(),
                points: &part_a.points,
            };
            out.extend(collide_convex_with_collider(view_a, &sub_aabb, cb, body_rot_b));
        }
        return reduce_contact_manifold(out, 4);
    }

    if let ColliderShape::Compound { parts: parts_b } = &cb.shape {
        let mut out = Vec::new();
        for part_b in parts_b {
            let sub_aabb = transform_local_aabb(&part_b.local_aabb, cb.world_translation, cb.world_rotation);
            if !aabbs_overlap(&ca.world_aabb, &sub_aabb) {
                continue;
            }
            let view_b = ConvexView::ConvexHull {
                pos: cb.world_translation,
                rot: *cb.world_rotation.to_rotation_matrix().matrix(),
                points: &part_b.points,
            };
            let flipped = collide_convex_with_collider(view_b, &sub_aabb, ca, body_rot_a);
            for mut c in flipped {
                c.normal = -c.normal;
                out.push(c);
            }
        }
        return reduce_contact_manifold(out, 4);
    }

    // Handle TriMesh on either side
    if let ColliderShape::TriMesh { vertices, indices } = &ca.shape {
        return collide_trimesh_with_collider(
            vertices,
            indices,
            ca.tri_bvh.as_ref(),
            ca.world_translation,
            ca.world_rotation,
            cb,
            body_rot_b,
        );
    }
    if let ColliderShape::TriMesh { vertices, indices } = &cb.shape {
        let mut flipped = collide_trimesh_with_collider(
            vertices,
            indices,
            cb.tri_bvh.as_ref(),
            cb.world_translation,
            cb.world_rotation,
            ca,
            body_rot_a,
        );
        for c in &mut flipped {
            c.normal = -c.normal;
        }
        return flipped;
    }

    // Simple convex vs simple convex
    let (Some(va), Some(vb)) = (
        ConvexView::from_simple_collider(ca, body_rot_a),
        ConvexView::from_simple_collider(cb, body_rot_b),
    ) else {
        return Vec::new();
    };
    collide_convex_vs_convex(va, vb)
}

fn collide_convex_with_collider(
    va: ConvexView<'_>,
    aabb_a: &BoundingBox,
    cb: &Collider,
    body_rot_b: UnitQuaternion<f64>,
) -> Vec<RawContact> {
    match &cb.shape {
        ColliderShape::Compound { parts } => {
            let mut out = Vec::new();
            for part_b in parts {
                let sub_aabb =
                    transform_local_aabb(&part_b.local_aabb, cb.world_translation, cb.world_rotation);
                if !aabbs_overlap(aabb_a, &sub_aabb) {
                    continue;
                }
                let vb = ConvexView::ConvexHull {
                    pos: cb.world_translation,
                    rot: *cb.world_rotation.to_rotation_matrix().matrix(),
                    points: &part_b.points,
                };
                out.extend(collide_convex_vs_convex(va, vb));
            }
            out
        }
        ColliderShape::TriMesh { vertices, indices } => collide_trimesh_with_convex(
            vertices,
            indices,
            cb.tri_bvh.as_ref(),
            cb.world_translation,
            cb.world_rotation,
            va,
            aabb_a,
        )
        .into_iter()
        .map(|mut c| {
            c.normal = -c.normal;
            c
        })
        .collect(),
        _ => {
            if let Some(vb) = ConvexView::from_simple_collider(cb, body_rot_b) {
                collide_convex_vs_convex(va, vb)
            } else {
                Vec::new()
            }
        }
    }
}

fn collide_convex_vs_convex(va: ConvexView<'_>, vb: ConvexView<'_>) -> Vec<RawContact> {
    match (va, vb) {
        (
            ConvexView::Sphere {
                center: ca,
                radius: ra,
            },
            ConvexView::Sphere {
                center: cb,
                radius: rb,
            },
        ) => collide_sphere_sphere(ca, ra, cb, rb).into_iter().collect(),

        (
            ConvexView::Sphere {
                center: cs,
                radius: rs,
            },
            ConvexView::Cube {
                center: cb,
                rot: rb,
                half_extents: heb,
            },
        ) => collide_sphere_cube(cs, rs, cb, rb, heb)
            .into_iter()
            .collect(),

        (
            ConvexView::Cube {
                center: ca,
                rot: ra,
                half_extents: hea,
            },
            ConvexView::Sphere {
                center: cs,
                radius: rs,
            },
        ) => collide_sphere_cube(cs, rs, ca, ra, hea)
            .map(|mut c| {
                c.normal = -c.normal;
                c
            })
            .into_iter()
            .collect(),

        (
            ConvexView::Sphere {
                center: cs,
                radius: rs,
            },
            ConvexView::Cylinder {
                center: cc,
                axis,
                half_height,
                radius: rc,
            },
        ) => collide_sphere_cylinder(cs, rs, cc, axis, half_height, rc)
            .into_iter()
            .collect(),

        (
            ConvexView::Cylinder {
                center: cc,
                axis,
                half_height,
                radius: rc,
            },
            ConvexView::Sphere {
                center: cs,
                radius: rs,
            },
        ) => collide_sphere_cylinder(cs, rs, cc, axis, half_height, rc)
            .map(|mut c| {
                c.normal = -c.normal;
                c
            })
            .into_iter()
            .collect(),

        (
            ConvexView::Cube {
                center: ca,
                rot: ra,
                half_extents: hea,
            },
            ConvexView::Cube {
                center: cb,
                rot: rb,
                half_extents: heb,
            },
        ) => collide_cube_cube(ca, ra, hea, cb, rb, heb),

        _ => {
            let Some(epa) = gjk_epa_convex(va, vb) else {
                return Vec::new();
            };
            // Enhance planar polyhedral face-on-face contact with a multi-point clipped manifold
            if let (Some((_, poly_a)), Some((_, poly_b))) = (
                va.planar_face_polygon(epa.normal),
                vb.planar_face_polygon(-epa.normal),
            ) {
                let manifold = build_planar_manifold(&poly_a, &poly_b, epa.normal, epa.depth);
                if !manifold.is_empty() {
                    return manifold;
                }
            }
            vec![epa]
        }
    }
}

// ============================================================================
// Analytical Primitive-Primitive Tests
// ============================================================================

fn collide_sphere_sphere(
    ca: Vector3<f64>,
    ra: f64,
    cb: Vector3<f64>,
    rb: f64,
) -> Option<RawContact> {
    let d = cb - ca;
    let dist_sq = d.norm_squared();
    let r_sum = ra + rb;
    if dist_sq >= r_sum * r_sum {
        return None;
    }
    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-12 { d / dist } else { Vector3::y() };
    let depth = r_sum - dist;
    let point = ca + normal * (ra - 0.5 * depth);
    Some(RawContact {
        point,
        normal,
        depth,
    })
}

fn collide_sphere_cube(
    c_sph: Vector3<f64>,
    r_sph: f64,
    c_box: Vector3<f64>,
    r_box: Matrix3<f64>,
    he_box: Vector3<f64>,
) -> Option<RawContact> {
    let local_c = r_box.transpose() * (c_sph - c_box);
    let closest_local = Vector3::new(
        local_c.x.clamp(-he_box.x, he_box.x),
        local_c.y.clamp(-he_box.y, he_box.y),
        local_c.z.clamp(-he_box.z, he_box.z),
    );
    let diff = local_c - closest_local;
    let dist_sq = diff.norm_squared();

    if dist_sq > 1e-24 {
        if dist_sq >= r_sph * r_sph {
            return None;
        }
        let dist = dist_sq.sqrt();
        let n_box_to_sph_local = diff / dist;
        let normal = -(r_box * n_box_to_sph_local);
        let depth = r_sph - dist;
        let closest_world = c_box + r_box * closest_local;
        let point = closest_world - normal * (0.5 * depth);
        Some(RawContact {
            point,
            normal,
            depth,
        })
    } else {
        // Sphere center inside OBB
        let mut best_ax = 0usize;
        let mut min_pen = f64::INFINITY;
        for k in 0..3 {
            let p = he_box[k] - local_c[k].abs();
            if p < min_pen {
                min_pen = p;
                best_ax = k;
            }
        }
        let mut n_box_to_sph_local = Vector3::zeros();
        n_box_to_sph_local[best_ax] = if local_c[best_ax] >= 0.0 { 1.0 } else { -1.0 };
        let normal = -(r_box * n_box_to_sph_local);
        let depth = r_sph + min_pen;
        let point = c_sph + normal * (0.5 * (r_sph - min_pen));
        Some(RawContact {
            point,
            normal,
            depth,
        })
    }
}

fn collide_sphere_cylinder(
    c_sph: Vector3<f64>,
    r_sph: f64,
    c_cyl: Vector3<f64>,
    cyl_axis: Vector3<f64>,
    hh_cyl: f64,
    r_cyl: f64,
) -> Option<RawContact> {
    let d = c_sph - c_cyl;
    let ax_s = d.dot(&cyl_axis);
    let rad_vec = d - cyl_axis * ax_s;
    let rad_len = rad_vec.norm();

    if rad_len > r_cyl || ax_s.abs() > hh_cyl {
        let ax_clamped = ax_s.clamp(-hh_cyl, hh_cyl);
        let rad_clamped = if rad_len > r_cyl {
            rad_vec * (r_cyl / rad_len)
        } else {
            rad_vec
        };
        let closest = c_cyl + cyl_axis * ax_clamped + rad_clamped;
        let out = c_sph - closest;
        let dist_sq = out.norm_squared();
        if dist_sq >= r_sph * r_sph {
            return None;
        }
        let dist = dist_sq.sqrt();
        let n_cyl_to_sph = if dist > 1e-12 {
            out / dist
        } else if ax_s.abs() >= hh_cyl - 1e-9 {
            cyl_axis * if ax_s >= 0.0 { 1.0 } else { -1.0 }
        } else if rad_len > 1e-12 {
            rad_vec / rad_len
        } else {
            cyl_axis
        };
        let normal = -n_cyl_to_sph;
        let depth = r_sph - dist;
        let point = closest - normal * (0.5 * depth);
        Some(RawContact {
            point,
            normal,
            depth,
        })
    } else {
        // Sphere center inside cylinder
        let d_cap = hh_cyl - ax_s.abs();
        let d_rad = r_cyl - rad_len;
        if d_cap <= d_rad {
            let s = if ax_s >= 0.0 { 1.0 } else { -1.0 };
            let normal = -(cyl_axis * s);
            let depth = r_sph + d_cap;
            let point = c_sph + normal * (0.5 * (r_sph - d_cap));
            Some(RawContact {
                point,
                normal,
                depth,
            })
        } else {
            let rad_dir = if rad_len > 1e-12 {
                rad_vec / rad_len
            } else {
                orthonormal_pair(&cyl_axis).0
            };
            let normal = -rad_dir;
            let depth = r_sph + d_rad;
            let point = c_sph + normal * (0.5 * (r_sph - d_rad));
            Some(RawContact {
                point,
                normal,
                depth,
            })
        }
    }
}

/// 15-axis SAT + Sutherland–Hodgman face clipping for OBB vs OBB (`Cube` vs `Cube`).
fn collide_cube_cube(
    ca: Vector3<f64>,
    ra: Matrix3<f64>,
    hea: Vector3<f64>,
    cb: Vector3<f64>,
    rb: Matrix3<f64>,
    heb: Vector3<f64>,
) -> Vec<RawContact> {
    let axes_a = [
        Vector3::new(ra[(0, 0)], ra[(1, 0)], ra[(2, 0)]),
        Vector3::new(ra[(0, 1)], ra[(1, 1)], ra[(2, 1)]),
        Vector3::new(ra[(0, 2)], ra[(1, 2)], ra[(2, 2)]),
    ];
    let axes_b = [
        Vector3::new(rb[(0, 0)], rb[(1, 0)], rb[(2, 0)]),
        Vector3::new(rb[(0, 1)], rb[(1, 1)], rb[(2, 1)]),
        Vector3::new(rb[(0, 2)], rb[(1, 2)], rb[(2, 2)]),
    ];
    let d_centers = cb - ca;

    let project_box = |axes: &[Vector3<f64>; 3], he: Vector3<f64>, axis: &Vector3<f64>| -> f64 {
        he.x * axes[0].dot(axis).abs()
            + he.y * axes[1].dot(axis).abs()
            + he.z * axes[2].dot(axis).abs()
    };

    let mut best_face_depth = f64::INFINITY;
    let mut best_face_axis = Vector3::y();
    let mut best_face_code = 0usize; // 0..3 => A's face, 3..6 => B's face

    for (i, ax) in axes_a.iter().enumerate() {
        let proj = project_box(&axes_b, heb, ax) + hea[i];
        let dist = d_centers.dot(ax).abs();
        let pen = proj - dist;
        if pen <= 0.0 {
            return Vec::new();
        }
        if pen < best_face_depth {
            best_face_depth = pen;
            best_face_axis = if d_centers.dot(ax) >= 0.0 { *ax } else { -*ax };
            best_face_code = i;
        }
    }

    for (j, ax) in axes_b.iter().enumerate() {
        let proj = project_box(&axes_a, hea, ax) + heb[j];
        let dist = d_centers.dot(ax).abs();
        let pen = proj - dist;
        if pen <= 0.0 {
            return Vec::new();
        }
        if pen < best_face_depth {
            best_face_depth = pen;
            best_face_axis = if d_centers.dot(ax) >= 0.0 { *ax } else { -*ax };
            best_face_code = 3 + j;
        }
    }

    let mut best_edge_depth = f64::INFINITY;
    let mut best_edge_axis = Vector3::y();
    let mut best_edge_pair = (0usize, 0usize);

    for (i, ax_a) in axes_a.iter().enumerate() {
        for (j, ax_b) in axes_b.iter().enumerate() {
            let cross = ax_a.cross(ax_b);
            let len_sq = cross.norm_squared();
            if len_sq < 1e-12 {
                continue;
            }
            let axis = cross / len_sq.sqrt();
            let proj = project_box(&axes_a, hea, &axis) + project_box(&axes_b, heb, &axis);
            let dist = d_centers.dot(&axis).abs();
            let pen = proj - dist;
            if pen <= 0.0 {
                return Vec::new();
            }
            if pen < best_edge_depth {
                best_edge_depth = pen;
                best_edge_axis = if d_centers.dot(&axis) >= 0.0 {
                    axis
                } else {
                    -axis
                };
                best_edge_pair = (i, j);
            }
        }
    }

    // Prefer face contact unless an edge-edge axis has strictly smaller penetration beyond tolerance
    if best_edge_depth * 1.0001 + 1e-7 < best_face_depth {
        let (ia, ib) = best_edge_pair;
        let p_a = obb_edge_midpoint(ca, &axes_a, hea, ia, best_edge_axis);
        let p_b = obb_edge_midpoint(cb, &axes_b, heb, ib, -best_edge_axis);
        let a_dir = axes_a[ia] * hea[ia];
        let b_dir = axes_b[ib] * heb[ib];
        let (qa, qb) = closest_points_segments(p_a - a_dir, p_a + a_dir, p_b - b_dir, p_b + b_dir);
        return vec![RawContact {
            point: 0.5 * (qa + qb),
            normal: best_edge_axis,
            depth: best_edge_depth,
        }];
    }

    // Face-on-face Sutherland–Hodgman clipping
    let (ref_is_a, ref_idx) = if best_face_code < 3 {
        (true, best_face_code)
    } else {
        (false, best_face_code - 3)
    };

    let (ref_c, ref_axes, ref_he, ref_out_n, inc_c, inc_axes, inc_he) = if ref_is_a {
        (ca, &axes_a, hea, best_face_axis, cb, &axes_b, heb)
    } else {
        (cb, &axes_b, heb, -best_face_axis, ca, &axes_a, hea)
    };

    let ref_face_center = ref_c + ref_out_n * ref_he[ref_idx];
    let ref_u_idx = (ref_idx + 1) % 3;
    let ref_v_idx = (ref_idx + 2) % 3;
    let u_axis = ref_axes[ref_u_idx];
    let v_axis = ref_axes[ref_v_idx];
    let u_ext = ref_he[ref_u_idx];
    let v_ext = ref_he[ref_v_idx];

    // Incident face: the one on `inc` whose outward normal is most anti-parallel to `ref_out_n`
    let mut inc_idx = 0usize;
    let mut min_dot = f64::INFINITY;
    let mut inc_sign = 1.0_f64;
    for (k, ax) in inc_axes.iter().enumerate() {
        let dot = ax.dot(&ref_out_n);
        if dot < min_dot {
            min_dot = dot;
            inc_idx = k;
            inc_sign = 1.0;
        }
        if -dot < min_dot {
            min_dot = -dot;
            inc_idx = k;
            inc_sign = -1.0;
        }
    }

    let iu = (inc_idx + 1) % 3;
    let iv = (inc_idx + 2) % 3;
    let inc_face_center = inc_c + inc_axes[inc_idx] * (inc_sign * inc_he[inc_idx]);
    let mut poly = vec![
        inc_face_center - inc_axes[iu] * inc_he[iu] - inc_axes[iv] * inc_he[iv],
        inc_face_center + inc_axes[iu] * inc_he[iu] - inc_axes[iv] * inc_he[iv],
        inc_face_center + inc_axes[iu] * inc_he[iu] + inc_axes[iv] * inc_he[iv],
        inc_face_center - inc_axes[iu] * inc_he[iu] + inc_axes[iv] * inc_he[iv],
    ];

    // Clip incident face against the 4 side planes of reference face
    let side_planes = [
        (u_axis, ref_face_center.dot(&u_axis) + u_ext),
        (-u_axis, (-ref_face_center).dot(&u_axis) + u_ext),
        (v_axis, ref_face_center.dot(&v_axis) + v_ext),
        (-v_axis, (-ref_face_center).dot(&v_axis) + v_ext),
    ];
    for (p_n, p_d) in side_planes {
        poly = clip_polygon_halfspace(&poly, p_n, p_d);
        if poly.is_empty() {
            break;
        }
    }

    let ref_plane_d = ref_face_center.dot(&ref_out_n);
    let mut contacts = Vec::new();
    for v in poly {
        let depth = ref_plane_d - v.dot(&ref_out_n);
        if depth > -1e-6 {
            let eff_depth = depth.max(1e-7);
            contacts.push(RawContact {
                point: v + ref_out_n * (0.5 * eff_depth),
                normal: best_face_axis,
                depth: eff_depth,
            });
        }
    }

    if contacts.is_empty() {
        vec![RawContact {
            point: 0.5 * (ca + cb),
            normal: best_face_axis,
            depth: best_face_depth,
        }]
    } else {
        reduce_contact_manifold(contacts, 4)
    }
}

fn obb_edge_midpoint(
    center: Vector3<f64>,
    axes: &[Vector3<f64>; 3],
    he: Vector3<f64>,
    edge_axis: usize,
    dir: Vector3<f64>,
) -> Vector3<f64> {
    let mut p = center;
    for k in 0..3 {
        if k != edge_axis {
            let s = if axes[k].dot(&dir) >= 0.0 { 1.0 } else { -1.0 };
            p += axes[k] * (s * he[k]);
        }
    }
    p
}

fn closest_points_segments(
    p1: Vector3<f64>,
    q1: Vector3<f64>,
    p2: Vector3<f64>,
    q2: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    let (s, t) = if a <= 1e-18 && e <= 1e-18 {
        (0.0, 0.0)
    } else if a <= 1e-18 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(&r);
        if e <= 1e-18 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(&d2);
            let denom = a * e - b * b;
            let mut s = if denom.abs() > 1e-18 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    (p1 + d1 * s, p2 + d2 * t)
}

fn clip_polygon_halfspace(poly: &[Vector3<f64>], plane_n: Vector3<f64>, plane_d: f64) -> Vec<Vector3<f64>> {
    if poly.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(poly.len() + 1);
    let mut prev = *poly.last().unwrap();
    let mut prev_dist = prev.dot(&plane_n) - plane_d;

    for &curr in poly {
        let curr_dist = curr.dot(&plane_n) - plane_d;
        let prev_in = prev_dist <= 1e-9;
        let curr_in = curr_dist <= 1e-9;

        if curr_in != prev_in {
            let denom = prev_dist - curr_dist;
            let t = if denom.abs() > 1e-18 {
                (prev_dist / denom).clamp(0.0, 1.0)
            } else {
                0.5
            };
            out.push(prev + (curr - prev) * t);
        }
        if curr_in {
            out.push(curr);
        }
        prev = curr;
        prev_dist = curr_dist;
    }
    out
}

// ============================================================================
// Universal f64 GJK + EPA for Convex Shapes
// ============================================================================

#[derive(Debug, Clone, Copy)]
struct SupportPt {
    p: Vector3<f64>,
    a: Vector3<f64>,
    b: Vector3<f64>,
}

#[inline]
fn minkowski_support(va: ConvexView<'_>, vb: ConvexView<'_>, dir: Vector3<f64>) -> SupportPt {
    let a = va.support(dir);
    let b = vb.support(-dir);
    SupportPt { p: a - b, a, b }
}

fn gjk_intersects(va: ConvexView<'_>, vb: ConvexView<'_>) -> Option<Vec<SupportPt>> {
    let mut dir = va.center() - vb.center();
    if dir.norm_squared() < 1e-24 {
        dir = Vector3::x();
    }

    let first = minkowski_support(va, vb, dir);
    let mut simplex = vec![first];
    if first.p.norm_squared() < 1e-24 {
        return Some(simplex);
    }
    dir = -first.p;

    for _ in 0..64 {
        if dir.norm_squared() < 1e-24 {
            return Some(simplex);
        }
        let next = minkowski_support(va, vb, dir);
        if next.p.dot(&dir) < -1e-12 {
            return None;
        }
        // Check duplicate support point (stagnation on boundary)
        if simplex.iter().any(|s| (s.p - next.p).norm_squared() < 1e-24) {
            return None;
        }
        simplex.push(next);
        if update_simplex(&mut simplex, &mut dir) {
            return Some(simplex);
        }
    }
    None
}

fn update_simplex(simplex: &mut Vec<SupportPt>, dir: &mut Vector3<f64>) -> bool {
    match simplex.len() {
        2 => {
            let b = simplex[0];
            let a = simplex[1];
            let ab = b.p - a.p;
            let ao = -a.p;
            if ab.dot(&ao) > 0.0 {
                let d = ab.cross(&ao).cross(&ab);
                if d.norm_squared() < 1e-24 {
                    return true;
                }
                *dir = d;
            } else {
                *simplex = vec![a];
                *dir = ao;
            }
            false
        }
        3 => {
            let c = simplex[0];
            let b = simplex[1];
            let a = simplex[2];
            let ab = b.p - a.p;
            let ac = c.p - a.p;
            let ao = -a.p;
            let abc = ab.cross(&ac);

            if abc.cross(&ac).dot(&ao) > 0.0 {
                if ac.dot(&ao) > 0.0 {
                    *simplex = vec![c, a];
                    let d = ac.cross(&ao).cross(&ac);
                    if d.norm_squared() < 1e-24 {
                        return true;
                    }
                    *dir = d;
                } else {
                    *simplex = vec![b, a];
                    return update_simplex(simplex, dir);
                }
            } else if ab.cross(&abc).dot(&ao) > 0.0 {
                *simplex = vec![b, a];
                return update_simplex(simplex, dir);
            } else if abc.dot(&ao) > 0.0 {
                *dir = abc;
            } else if abc.dot(&ao) < 0.0 {
                *simplex = vec![b, c, a];
                *dir = -abc;
            } else {
                return true;
            }
            false
        }
        4 => {
            let d_pt = simplex[0];
            let c = simplex[1];
            let b = simplex[2];
            let a = simplex[3];

            let ab = b.p - a.p;
            let ac = c.p - a.p;
            let ad = d_pt.p - a.p;
            let ao = -a.p;

            let abc = ab.cross(&ac);
            let acd = ac.cross(&ad);
            let adb = ad.cross(&ab);

            // Ensure normals point outward relative to the 4th vertex
            let abc_out = if abc.dot(&ad) > 0.0 { -abc } else { abc };
            let acd_out = if acd.dot(&ab) > 0.0 { -acd } else { acd };
            let adb_out = if adb.dot(&ac) > 0.0 { -adb } else { adb };

            if abc_out.dot(&ao) > 0.0 {
                *simplex = vec![c, b, a];
                return update_simplex(simplex, dir);
            }
            if acd_out.dot(&ao) > 0.0 {
                *simplex = vec![d_pt, c, a];
                return update_simplex(simplex, dir);
            }
            if adb_out.dot(&ao) > 0.0 {
                *simplex = vec![b, d_pt, a];
                return update_simplex(simplex, dir);
            }
            true
        }
        _ => false,
    }
}

#[derive(Clone, Copy)]
struct EpaFace {
    idx: [usize; 3],
    normal: Vector3<f64>,
    dist: f64,
}

fn make_epa_face(verts: &[SupportPt], i0: usize, i1: usize, i2: usize, interior: Vector3<f64>) -> Option<EpaFace> {
    let a = verts[i0].p;
    let b = verts[i1].p;
    let c = verts[i2].p;
    let raw_n = (b - a).cross(&(c - a));
    let n_len = raw_n.norm();
    if n_len <= 1e-16 {
        return None;
    }
    let mut n = raw_n / n_len;
    let mut idx = [i0, i1, i2];
    if n.dot(&(a - interior)) < 0.0 {
        n = -n;
        idx = [i0, i2, i1];
    }
    let dist = a.dot(&n);
    Some(EpaFace { idx, normal: n, dist })
}

fn gjk_epa_convex(va: ConvexView<'_>, vb: ConvexView<'_>) -> Option<RawContact> {
    let mut verts = gjk_intersects(va, vb)?;

    // Expand degenerate 1/2/3-point simplex into a valid 4-vertex tetrahedron
    let basis_dirs = [
        Vector3::x(),
        -Vector3::x(),
        Vector3::y(),
        -Vector3::y(),
        Vector3::z(),
        -Vector3::z(),
    ];

    if verts.len() == 1 {
        for &d in &basis_dirs {
            let s = minkowski_support(va, vb, d);
            if (s.p - verts[0].p).norm_squared() > 1e-16 {
                verts.push(s);
                break;
            }
        }
    }
    if verts.len() == 2 {
        let edge = verts[1].p - verts[0].p;
        let (u, v) = orthonormal_pair(&edge);
        for d in [u, -u, v, -v] {
            let s = minkowski_support(va, vb, d);
            if (s.p - verts[0].p).cross(&edge).norm_squared() > 1e-16 {
                verts.push(s);
                break;
            }
        }
    }
    if verts.len() == 3 {
        let n = (verts[1].p - verts[0].p).cross(&(verts[2].p - verts[0].p));
        let n_unit = if n.norm_squared() > 1e-24 {
            n.normalize()
        } else {
            Vector3::z()
        };
        for d in [n_unit, -n_unit, Vector3::x(), Vector3::y(), Vector3::z()] {
            let s = minkowski_support(va, vb, d);
            if (s.p - verts[0].p).dot(&n_unit).abs() > 1e-9 {
                verts.push(s);
                break;
            }
        }
    }

    if verts.len() < 4 {
        return None;
    }

    let interior = (verts[0].p + verts[1].p + verts[2].p + verts[3].p) * 0.25;
    let mut faces = Vec::with_capacity(32);
    for [i0, i1, i2] in [[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]] {
        if let Some(f) = make_epa_face(&verts, i0, i1, i2, interior) {
            faces.push(f);
        }
    }
    if faces.len() < 4 {
        return None;
    }

    let mut best_face = faces[0];
    for _ in 0..64 {
        let (b_idx, &bf) = faces
            .iter()
            .enumerate()
            .min_by(|(_, f1), (_, f2)| {
                f1.dist
                    .partial_cmp(&f2.dist)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })?;
        best_face = bf;

        let s = minkowski_support(va, vb, best_face.normal);
        let s_dist = s.p.dot(&best_face.normal);
        if s_dist - best_face.dist <= 1e-7 {
            break;
        }
        if verts.iter().any(|v| (v.p - s.p).norm_squared() < 1e-20) {
            break;
        }

        let mut visible = vec![false; faces.len()];
        let mut any_vis = false;
        for (fi, f) in faces.iter().enumerate() {
            if (s.p - verts[f.idx[0]].p).dot(&f.normal) > 1e-10 {
                visible[fi] = true;
                any_vis = true;
            }
        }
        if !any_vis {
            visible[b_idx] = true;
        }

        let mut vis_edges = HashSet::new();
        for (fi, f) in faces.iter().enumerate() {
            if visible[fi] {
                vis_edges.insert((f.idx[0], f.idx[1]));
                vis_edges.insert((f.idx[1], f.idx[2]));
                vis_edges.insert((f.idx[2], f.idx[0]));
            }
        }

        let mut horizon = Vec::new();
        for &(u, v) in &vis_edges {
            if !vis_edges.contains(&(v, u)) {
                horizon.push((u, v));
            }
        }
        if horizon.len() < 3 {
            break;
        }

        let new_idx = verts.len();
        verts.push(s);

        let mut next_faces: Vec<EpaFace> = faces
            .into_iter()
            .enumerate()
            .filter_map(|(fi, f)| if visible[fi] { None } else { Some(f) })
            .collect();

        for (u, v) in horizon {
            if let Some(nf) = make_epa_face(&verts, u, v, new_idx, interior) {
                next_faces.push(nf);
            }
        }
        if next_faces.is_empty() {
            break;
        }
        faces = next_faces;
    }

    let proj = best_face.normal * best_face.dist;
    let va_pt = verts[best_face.idx[0]];
    let vb_pt = verts[best_face.idx[1]];
    let vc_pt = verts[best_face.idx[2]];
    let (u, v, w) = barycentric_3d(proj, va_pt.p, vb_pt.p, vc_pt.p);

    let pa = va_pt.a * u + vb_pt.a * v + vc_pt.a * w;
    let pb = va_pt.b * u + vb_pt.b * v + vc_pt.b * w;
    let depth = best_face.dist.max(0.0);

    Some(RawContact {
        point: 0.5 * (pa + pb),
        normal: best_face.normal,
        depth,
    })
}

fn barycentric_3d(
    p: Vector3<f64>,
    a: Vector3<f64>,
    b: Vector3<f64>,
    c: Vector3<f64>,
) -> (f64, f64, f64) {
    let v0 = b - a;
    let v1 = c - a;
    let v2 = p - a;
    let d00 = v0.dot(&v0);
    let d01 = v0.dot(&v1);
    let d11 = v1.dot(&v1);
    let d20 = v2.dot(&v0);
    let d21 = v2.dot(&v1);
    let denom = d00 * d11 - d01 * d01;
    if denom.abs() <= 1e-24 {
        return (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
    }
    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;
    let uc = u.clamp(0.0, 1.0);
    let vc = v.clamp(0.0, 1.0);
    let wc = w.clamp(0.0, 1.0);
    let sum = uc + vc + wc;
    if sum > 1e-12 {
        (uc / sum, vc / sum, wc / sum)
    } else {
        (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
    }
}

// ============================================================================
// Planar Face Clipping Manifold & TriMesh Narrow-Phase
// ============================================================================

fn build_planar_manifold(
    poly_a_3d: &[Vector3<f64>],
    poly_b_3d: &[Vector3<f64>],
    normal: Vector3<f64>,
    fallback_depth: f64,
) -> Vec<RawContact> {
    if poly_a_3d.is_empty() || poly_b_3d.is_empty() {
        return Vec::new();
    }
    let h_a = poly_a_3d
        .iter()
        .map(|p| p.dot(&normal))
        .fold(f64::NEG_INFINITY, f64::max);
    let h_b = poly_b_3d
        .iter()
        .map(|p| p.dot(&normal))
        .fold(f64::INFINITY, f64::min);
    let depth = (h_a - h_b).max(fallback_depth);
    if depth <= 0.0 {
        return Vec::new();
    }
    let (t1, t2) = orthonormal_pair(&normal);
    let hull_a = convex_hull_2d(poly_a_3d, t1, t2);
    let hull_b = convex_hull_2d(poly_b_3d, t1, t2);
    if hull_a.len() < 3 || hull_b.len() < 3 {
        return Vec::new();
    }

    let clipped = clip_convex_2d(&hull_b, &hull_a);
    if clipped.is_empty() {
        return Vec::new();
    }

    let mid_h = 0.5 * (h_a + h_b);
    let contacts: Vec<RawContact> = clipped
        .into_iter()
        .map(|[x, y]| RawContact {
            point: normal * mid_h + t1 * x + t2 * y,
            normal,
            depth,
        })
        .collect();
    reduce_contact_manifold(contacts, 4)
}

fn convex_hull_2d(pts_3d: &[Vector3<f64>], t1: Vector3<f64>, t2: Vector3<f64>) -> Vec<[f64; 2]> {
    let mut pts: Vec<[f64; 2]> = pts_3d.iter().map(|p| [p.dot(&t1), p.dot(&t2)]).collect();
    pts.sort_by(|a, b| {
        a[0].partial_cmp(&b[0])
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a[1].partial_cmp(&b[1]).unwrap_or(std::cmp::Ordering::Equal))
    });
    pts.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-8 && (a[1] - b[1]).abs() < 1e-8);
    if pts.len() <= 2 {
        return pts;
    }

    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| -> f64 {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };

    let mut lower = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 1e-12 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 1e-12 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// Clips subject CCW 2D polygon against clip CCW 2D polygon.
fn clip_convex_2d(subject: &[[f64; 2]], clip: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut output = subject.to_vec();
    for i in 0..clip.len() {
        if output.is_empty() {
            break;
        }
        let a = clip[i];
        let b = clip[(i + 1) % clip.len()];
        let edge = [b[0] - a[0], b[1] - a[1]];
        let inside = |p: [f64; 2]| -> f64 { edge[0] * (p[1] - a[1]) - edge[1] * (p[0] - a[0]) };

        let input = output;
        output = Vec::with_capacity(input.len() + 1);
        let mut prev = *input.last().unwrap();
        let mut prev_d = inside(prev);

        for &curr in &input {
            let curr_d = inside(curr);
            if curr_d >= -1e-9 {
                if prev_d < -1e-9 {
                    let denom = prev_d - curr_d;
                    let t = if denom.abs() > 1e-18 {
                        (prev_d / denom).clamp(0.0, 1.0)
                    } else {
                        0.5
                    };
                    output.push([
                        prev[0] + (curr[0] - prev[0]) * t,
                        prev[1] + (curr[1] - prev[1]) * t,
                    ]);
                }
                output.push(curr);
            } else if prev_d >= -1e-9 {
                let denom = prev_d - curr_d;
                let t = if denom.abs() > 1e-18 {
                    (prev_d / denom).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                output.push([
                    prev[0] + (curr[0] - prev[0]) * t,
                    prev[1] + (curr[1] - prev[1]) * t,
                ]);
            }
            prev = curr;
            prev_d = curr_d;
        }
    }
    output
}

fn reduce_contact_manifold(mut contacts: Vec<RawContact>, max_points: usize) -> Vec<RawContact> {
    if contacts.len() <= 1 {
        return contacts;
    }

    // Deduplicate nearly identical points
    let mut deduped: Vec<RawContact> = Vec::with_capacity(contacts.len());
    for c in contacts.drain(..) {
        if let Some(existing) = deduped
            .iter_mut()
            .find(|e| (e.point - c.point).norm_squared() < 1e-10 && e.normal.dot(&c.normal) > 0.99)
        {
            if c.depth > existing.depth {
                *existing = c;
            }
        } else {
            deduped.push(c);
        }
    }

    if deduped.len() <= max_points {
        return deduped;
    }

    // Furthest-point selection starting from deepest contact
    let mut selected = Vec::with_capacity(max_points);
    let first_idx = deduped
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| {
            a.depth
                .partial_cmp(&b.depth)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(i, _)| i)
        .unwrap_or(0);
    selected.push(deduped.swap_remove(first_idx));

    while selected.len() < max_points && !deduped.is_empty() {
        let mut best_i = 0usize;
        let mut best_min_d2 = -1.0_f64;
        for (i, c) in deduped.iter().enumerate() {
            let min_d2 = selected
                .iter()
                .map(|s| (s.point - c.point).norm_squared())
                .fold(f64::INFINITY, f64::min);
            if min_d2 > best_min_d2 {
                best_min_d2 = min_d2;
                best_i = i;
            }
        }
        selected.push(deduped.swap_remove(best_i));
    }

    selected
}

fn collide_trimesh_with_collider(
    vertices: &[[f64; 3]],
    indices: &[[u32; 3]],
    bvh: Option<&TriMeshBvh>,
    mesh_pos: Vector3<f64>,
    mesh_rot: UnitQuaternion<f64>,
    other: &Collider,
    other_body_rot: UnitQuaternion<f64>,
) -> Vec<RawContact> {
    match &other.shape {
        ColliderShape::Compound { parts } => {
            let mut all = Vec::new();
            for p in parts {
                let sub_aabb = transform_local_aabb(&p.local_aabb, other.world_translation, other.world_rotation);
                let view = ConvexView::ConvexHull {
                    pos: other.world_translation,
                    rot: *other.world_rotation.to_rotation_matrix().matrix(),
                    points: &p.points,
                };
                all.extend(collide_trimesh_with_convex(
                    vertices,
                    indices,
                    bvh,
                    mesh_pos,
                    mesh_rot,
                    view,
                    &sub_aabb,
                ));
            }
            reduce_contact_manifold(all, 4)
        }
        ColliderShape::TriMesh {
            vertices: verts_b,
            indices: inds_b,
        } => {
            let local_query = world_aabb_to_local(&other.world_aabb, mesh_pos, mesh_rot, 1e-6);
            let mut cand_a = Vec::new();
            if let Some(tree) = bvh {
                tree.query_local_aabb(&local_query, &mut cand_a);
            } else {
                cand_a.extend(0..indices.len());
            }

            let mut cand_b = Vec::new();
            let mesh_world_aabb = transform_local_aabb(
                &aabb_from_slice(vertices),
                mesh_pos,
                mesh_rot,
            );
            let local_query_b = world_aabb_to_local(
                &mesh_world_aabb,
                other.world_translation,
                other.world_rotation,
                1e-6,
            );
            if let Some(tree_b) = &other.tri_bvh {
                tree_b.query_local_aabb(&local_query_b, &mut cand_b);
            } else {
                cand_b.extend(0..inds_b.len());
            }

            let mut out = Vec::new();
            for &ta in &cand_a {
                let Some((av0, av1, av2, an)) = triangle_world(vertices, indices[ta], mesh_pos, mesh_rot) else {
                    continue;
                };
                let tri_aabb_a = triangle_aabb(av0, av1, av2);
                let h_a = ((av1 - av0).norm() + (av2 - av1).norm() + (av0 - av2).norm())
                    .mul_add(0.05, 0.0)
                    .clamp(1e-3, 0.05);
                let prism_a = ConvexView::TriPrism {
                    verts: [
                        av0,
                        av1,
                        av2,
                        av0 - an * h_a,
                        av1 - an * h_a,
                        av2 - an * h_a,
                    ],
                };
                for &tb in &cand_b {
                    let Some((bv0, bv1, bv2, bn)) = triangle_world(
                        verts_b,
                        inds_b[tb],
                        other.world_translation,
                        other.world_rotation,
                    ) else {
                        continue;
                    };
                    let tri_aabb_b = triangle_aabb(bv0, bv1, bv2);
                    if !aabbs_overlap(&tri_aabb_a, &tri_aabb_b) {
                        continue;
                    }
                    let h_b = ((bv1 - bv0).norm() + (bv2 - bv1).norm() + (bv0 - bv2).norm())
                        .mul_add(0.05, 0.0)
                        .clamp(1e-3, 0.05);
                    let prism_b = ConvexView::TriPrism {
                        verts: [
                            bv0,
                            bv1,
                            bv2,
                            bv0 - bn * h_b,
                            bv1 - bn * h_b,
                            bv2 - bn * h_b,
                        ],
                    };
                    if let Some(c) = gjk_epa_convex(prism_a, prism_b) {
                        if c.normal.dot(&an) >= -0.25 && c.depth < h_a * 0.9 {
                            out.push(c);
                        }
                    }
                }
            }
            reduce_contact_manifold(out, 4)
        }
        _ => {
            if let Some(view) = ConvexView::from_simple_collider(other, other_body_rot) {
                collide_trimesh_with_convex(
                    vertices,
                    indices,
                    bvh,
                    mesh_pos,
                    mesh_rot,
                    view,
                    &other.world_aabb,
                )
            } else {
                Vec::new()
            }
        }
    }
}

fn collide_trimesh_with_convex(
    vertices: &[[f64; 3]],
    indices: &[[u32; 3]],
    bvh: Option<&TriMeshBvh>,
    mesh_pos: Vector3<f64>,
    mesh_rot: UnitQuaternion<f64>,
    other: ConvexView<'_>,
    other_world_aabb: &BoundingBox,
) -> Vec<RawContact> {
    let local_query = world_aabb_to_local(other_world_aabb, mesh_pos, mesh_rot, 1e-6);
    let mut candidates = Vec::new();
    if let Some(tree) = bvh {
        tree.query_local_aabb(&local_query, &mut candidates);
    } else {
        candidates.extend(0..indices.len());
    }
    if candidates.is_empty() {
        return Vec::new();
    }

    let mut contacts = Vec::new();
    for &t_idx in &candidates {
        let Some((tv0, tv1, tv2, n_tri)) = triangle_world(vertices, indices[t_idx], mesh_pos, mesh_rot)
        else {
            continue;
        };
        let tri_box = triangle_aabb(tv0, tv1, tv2);
        if !aabbs_overlap(&tri_box, other_world_aabb) {
            continue;
        }

        match other {
            ConvexView::Sphere { center, radius } => {
                if let Some(c) = collide_sphere_triangle(center, radius, tv0, tv1, tv2, n_tri) {
                    contacts.push(c);
                }
            }
            _ => {
                // First, test exact zero-thickness triangle against `other` with GJK to prevent backface hits
                let flat_tri = ConvexView::TriPrism {
                    verts: [tv0, tv1, tv2, tv0, tv1, tv2],
                };
                if gjk_intersects(flat_tri, other).is_none() {
                    continue;
                }

                // Compute outward face penetration along `n_tri`
                let p_min_b = other.support(-n_tri);
                let p_max_b = other.support(n_tri);
                let pen_face = (tv0 - p_min_b).dot(&n_tri);
                let front_ext = (p_max_b - tv0).dot(&n_tri);
                let b_thickness = (p_max_b - p_min_b).dot(&n_tri).max(1e-6);

                let tri_perim = (tv1 - tv0).norm() + (tv2 - tv1).norm() + (tv0 - tv2).norm();
                let h = (tri_perim * 0.05).clamp(1e-3, 0.05);
                let prism = ConvexView::TriPrism {
                    verts: [
                        tv0,
                        tv1,
                        tv2,
                        tv0 - n_tri * h,
                        tv1 - n_tri * h,
                        tv2 - n_tri * h,
                    ],
                };

                if let Some(mut epa) = gjk_epa_convex(prism, other) {
                    if front_ext >= -1e-6
                        && pen_face > 0.0
                        && pen_face <= b_thickness * 0.65
                        && (epa.normal.dot(&n_tri) > 0.85 || pen_face <= epa.depth * 1.05)
                    {
                        epa.normal = n_tri;
                        epa.depth = pen_face;
                        epa.point = p_min_b + n_tri * (0.5 * pen_face);
                    }
                    if epa.normal.dot(&n_tri) >= -0.25 {
                        contacts.push(epa);
                    }
                }
            }
        }
    }

    if contacts.is_empty() {
        return contacts;
    }

    // Group by dominant normal to filter out opposite-side thin-wall hits, then check planar manifold
    let best_idx = contacts
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            a.depth
                .partial_cmp(&b.depth)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(i, _)| i)
        .unwrap_or(0);
    let dominant_n = contacts[best_idx].normal;
    let dominant_depth = contacts[best_idx].depth;

    // For planar contacts on a tessellated TriMesh floor, enhance with full face-polygon clipping
    let mut mesh_face_pts = Vec::new();
    for &t_idx in &candidates {
        if let Some((tv0, tv1, tv2, n_tri)) = triangle_world(vertices, indices[t_idx], mesh_pos, mesh_rot) {
            if n_tri.dot(&dominant_n) > 0.995 {
                mesh_face_pts.push(tv0);
                mesh_face_pts.push(tv1);
                mesh_face_pts.push(tv2);
            }
        }
    }
    if mesh_face_pts.len() >= 3 {
        if let Some((_, poly_b)) = other.planar_face_polygon(-dominant_n) {
            let manifold = build_planar_manifold(&mesh_face_pts, &poly_b, dominant_n, dominant_depth);
            if !manifold.is_empty() {
                return manifold;
            }
        }
    }

    contacts.retain(|c| c.normal.dot(&dominant_n) > 0.5 || c.depth <= dominant_depth * 2.0);
    reduce_contact_manifold(contacts, 4)
}

fn collide_sphere_triangle(
    c_sph: Vector3<f64>,
    r_sph: f64,
    a: Vector3<f64>,
    b: Vector3<f64>,
    c: Vector3<f64>,
    n_tri: Vector3<f64>,
) -> Option<RawContact> {
    let signed_plane = (c_sph - a).dot(&n_tri);
    if signed_plane < -r_sph * 0.5 {
        return None;
    }

    let closest = closest_point_on_triangle(c_sph, a, b, c);
    let diff = c_sph - closest;
    let dist_sq = diff.norm_squared();
    if dist_sq >= r_sph * r_sph {
        return None;
    }

    if signed_plane < 0.0 {
        let depth = r_sph - signed_plane;
        return Some(RawContact {
            point: closest,
            normal: n_tri,
            depth,
        });
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-12 { diff / dist } else { n_tri };
    let depth = r_sph - dist;
    Some(RawContact {
        point: closest + normal * (0.5 * depth),
        normal,
        depth,
    })
}

fn closest_point_on_triangle(
    p: Vector3<f64>,
    a: Vector3<f64>,
    b: Vector3<f64>,
    c: Vector3<f64>,
) -> Vector3<f64> {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
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
        return c;
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

type WorldTriangle = (Vector3<f64>, Vector3<f64>, Vector3<f64>, Vector3<f64>);

#[inline]
fn triangle_world(
    vertices: &[[f64; 3]],
    tri: [u32; 3],
    pos: Vector3<f64>,
    rot: UnitQuaternion<f64>,
) -> Option<WorldTriangle> {
    let v0 = pos + rot * Vector3::from(*vertices.get(tri[0] as usize)?);
    let v1 = pos + rot * Vector3::from(*vertices.get(tri[1] as usize)?);
    let v2 = pos + rot * Vector3::from(*vertices.get(tri[2] as usize)?);
    let cross = (v1 - v0).cross(&(v2 - v0));
    let len = cross.norm();
    if len <= 1e-14 {
        return None;
    }
    Some((v0, v1, v2, cross / len))
}

#[inline]
fn triangle_aabb(v0: Vector3<f64>, v1: Vector3<f64>, v2: Vector3<f64>) -> BoundingBox {
    BoundingBox::new(
        [
            v0.x.min(v1.x).min(v2.x),
            v0.y.min(v1.y).min(v2.y),
            v0.z.min(v1.z).min(v2.z),
        ],
        [
            v0.x.max(v1.x).max(v2.x),
            v0.y.max(v1.y).max(v2.y),
            v0.z.max(v1.z).max(v2.z),
        ],
    )
}

fn aabb_from_slice(vertices: &[[f64; 3]]) -> BoundingBox {
    if vertices.is_empty() {
        return BoundingBox::new([0.0; 3], [0.0; 3]);
    }
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for v in vertices {
        for k in 0..3 {
            min[k] = min[k].min(v[k]);
            max[k] = max[k].max(v[k]);
        }
    }
    BoundingBox::new(min, max)
}

fn transform_local_aabb(
    local: &BoundingBox,
    pos: Vector3<f64>,
    rot: UnitQuaternion<f64>,
) -> BoundingBox {
    let r = *rot.to_rotation_matrix().matrix();
    let abs_r = r.map(f64::abs);
    let c = Vector3::new(
        0.5 * (local.min[0] + local.max[0]),
        0.5 * (local.min[1] + local.max[1]),
        0.5 * (local.min[2] + local.max[2]),
    );
    let he = Vector3::new(
        0.5 * (local.max[0] - local.min[0]),
        0.5 * (local.max[1] - local.min[1]),
        0.5 * (local.max[2] - local.min[2]),
    );
    let wc = pos + r * c;
    let we = abs_r * he;
    BoundingBox::new(
        [wc.x - we.x, wc.y - we.y, wc.z - we.z],
        [wc.x + we.x, wc.y + we.y, wc.z + we.z],
    )
}

/// Computes an orthonormal tangent pair `(t1, t2)` perpendicular to unit vector `n`.
#[must_use]
pub(crate) fn orthonormal_pair(n: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let ref_vec = if n.x.abs() < 0.8 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let t1 = n.cross(&ref_vec).normalize();
    let t2 = n.cross(&t1).normalize();
    (t1, t2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_p4_t4_trimesh_bvh_query_and_analytical_primitives() {
        // 1. Analytical sphere-sphere
        let c_ss = collide_sphere_sphere(
            Vector3::new(0.0, 0.0, 0.0),
            0.5,
            Vector3::new(0.8, 0.0, 0.0),
            0.5,
        )
        .expect("spheres should overlap");
        assert!((c_ss.depth - 0.2).abs() < 1e-12);
        assert!((c_ss.normal - Vector3::x()).norm() < 1e-12);

        // 2. Analytical sphere-cube (rotated OBB)
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_4);
        let c_sc = collide_sphere_cube(
            Vector3::new(0.0, 1.0, 0.0),
            0.3,
            Vector3::new(0.0, 0.0, 0.0),
            *rot.to_rotation_matrix().matrix(),
            Vector3::new(0.5, 0.5, 0.5),
        )
        .expect("rotated cube should touch sphere");
        let expected_top = std::f64::consts::SQRT_2 * 0.5;
        let expected_depth = 0.3 - (1.0 - expected_top);
        assert!((c_sc.depth - expected_depth).abs() < 1e-12);
        assert!((c_sc.normal - (-Vector3::y())).norm() < 1e-12);

        // 3. Analytical sphere-cylinder
        let c_scyl = collide_sphere_cylinder(
            Vector3::new(0.0, 0.7, 0.0),
            0.25,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::y(),
            0.5,
            0.3,
        )
        .expect("sphere should hit cylinder top cap");
        assert!((c_scyl.depth - 0.05).abs() < 1e-12);
        assert!((c_scyl.normal - (-Vector3::y())).norm() < 1e-12);

        // 4. OBB-OBB SAT + face clipping yields 4 corner contacts
        let cube_contacts = collide_cube_cube(
            Vector3::new(0.0, 0.0, 0.0),
            Matrix3::identity(),
            Vector3::new(2.0, 0.1, 2.0),
            Vector3::new(0.0, 0.29, 0.0),
            Matrix3::identity(),
            Vector3::new(0.2, 0.2, 0.2),
        );
        assert_eq!(cube_contacts.len(), 4);
        for c in &cube_contacts {
            assert!((c.depth - 0.01).abs() < 1e-12);
            assert!((c.normal - Vector3::y()).norm() < 1e-12);
        }

        // 5. GJK/EPA general convex separation & overlap (Cylinder vs Cube)
        let cyl = ConvexView::Cylinder {
            center: Vector3::new(0.0, 0.295, 0.0),
            axis: Vector3::y(),
            half_height: 0.2,
            radius: 0.1,
        };
        let cube = ConvexView::Cube {
            center: Vector3::new(0.0, 0.0, 0.0),
            rot: Matrix3::identity(),
            half_extents: Vector3::new(1.0, 0.1, 1.0),
        };
        let cyl_cube = collide_convex_vs_convex(cube, cyl);
        assert!(!cyl_cube.is_empty());
        assert!((cyl_cube[0].depth - 0.005).abs() < 1e-6);
        assert!((cyl_cube[0].normal - Vector3::y()).norm() < 1e-5);

        // Disjoint cylinder vs cube
        let cyl_far = ConvexView::Cylinder {
            center: Vector3::new(0.0, 0.35, 0.0),
            axis: Vector3::y(),
            half_height: 0.2,
            radius: 0.1,
        };
        assert!(collide_convex_vs_convex(cube, cyl_far).is_empty());

        // 6. TriMesh BVH query
        let verts = vec![
            [-1.0, 0.0, -1.0],
            [1.0, 0.0, -1.0],
            [1.0, 0.0, 1.0],
            [-1.0, 0.0, 1.0],
            [10.0, 0.0, 10.0],
            [11.0, 0.0, 10.0],
            [11.0, 0.0, 11.0],
        ];
        let inds = vec![[0, 1, 2], [0, 2, 3], [4, 5, 6]];
        let bvh = TriMeshBvh::build(&verts, &inds);
        assert!(!bvh.is_empty());
        let mut hits = Vec::new();
        bvh.query_local_aabb(&BoundingBox::new([-0.2, -0.1, -0.2], [0.2, 0.1, 0.2]), &mut hits);
        assert!(hits.contains(&0) && hits.contains(&1));
        assert!(!hits.contains(&2));
    }
}
