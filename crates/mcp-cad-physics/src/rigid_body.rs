//! Rigid body and collider models in double precision (`f64`) with exact mesh mass-inertia
//! integration (`P4-T2` / `ADR-004`).
//!
//! Pure-Rust double-precision (`f64`) rigid-body and collision geometry primitives:
//! - Computes exact volume, center of mass $\mathbf{c}_{\text{loc}}$, and full $3\times 3$ inertia
//!   tensor $I_{\text{loc}}$ directly from the closed triangulated boundary mesh (`TriangleMesh`)
//!   of a CAD [`Part`] via signed tetrahedron decomposition (accounting for through-holes, bores,
//!   keyways, and internal cavities).
//! - Provides native `f64` [`RigidBody`], [`BodyHandle`], [`BodyRegistry`], [`Collider`],
//!   [`ColliderHandle`], [`ColliderShape`], and [`ColliderRegistry`] types for all [`ColliderType`]
//!   variants (`ConvexHull`, `TriMesh`, `Compound`, `Cube`, `Sphere`, `Cylinder`).

use std::collections::HashMap;

use mcp_cad_core::topology::{BoundingBox, TriangleMesh};
use mcp_cad_core::Part;
use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

use crate::collision::TriMeshBvh;
use crate::error::PhysicsError;

/// Stable, copyable handle identifying a [`RigidBody`] in a [`BodyRegistry`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct BodyHandle(pub usize);

/// Stable, copyable handle identifying a [`Collider`] in a [`ColliderRegistry`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ColliderHandle(pub usize);

/// Supported collider approximation strategies for CAD parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ColliderType {
    /// Convex hull computed from the part's surface mesh vertices (best for dynamic bodies).
    ConvexHull,
    /// Exact triangular boundary mesh (best for static/fixed bodies and ground).
    TriMesh,
    /// Approximate compound shape composed of multiple convex hulls via spatial decomposition.
    Compound,
    /// Axis-aligned bounding box cuboid primitive.
    Cube,
    /// Bounding sphere primitive enclosing the part.
    Sphere,
    /// Bounding cylinder primitive aligned along the part's major axis.
    Cylinder,
}

/// Exact mass-inertia properties of a homogeneous solid computed in `f64` from its closed
/// boundary `TriangleMesh` and material density $\rho$ ($\text{kg}/\text{m}^3$).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassProperties {
    /// Volume of the solid in $\text{m}^3$ (accounting for internal cavities and holes).
    pub volume: f64,
    /// Total mass $m = \rho V$ in $\text{kg}$.
    pub mass: f64,
    /// Inverse mass $m^{-1}$ in $\text{kg}^{-1}$ (`0.0` if `mass <= 1e-15`).
    pub inv_mass: f64,
    /// Exact 3D center of mass $[c_x, c_y, c_z]$ in body-local coordinates ($\text{m}$).
    pub center_of_mass: [f64; 3],
    /// Exact 3D center of mass vector $\mathbf{c}_{\text{loc}}$ in body-local coordinates ($\text{m}$).
    pub local_com: Vector3<f64>,
    /// Symmetric positive-definite $3\times 3$ inertia tensor $I_{\text{loc}}$ about `local_com`
    /// in body-local axes ($\text{kg}\cdot\text{m}^2$).
    pub inertia_tensor: Matrix3<f64>,
    /// Inverse of `inertia_tensor` ($I_{\text{loc}}^{-1}$) in body-local axes
    /// ($\text{kg}^{-1}\cdot\text{m}^{-2}$).
    pub inv_inertia_tensor: Matrix3<f64>,
    /// $3\times 3$ inertia tensor $I_{\text{origin}}$ about the body coordinate origin $[0, 0, 0]$
    /// in body-local axes ($\text{kg}\cdot\text{m}^2$), related to `inertia_tensor` by Steiner's
    /// parallel axis theorem:
    /// $$I_{\text{origin}} = I_{\text{loc}} + m\left((\mathbf{c}_{\text{loc}}\cdot\mathbf{c}_{\text{loc}})I_3 - \mathbf{c}_{\text{loc}}\mathbf{c}_{\text{loc}}^T\right).$$
    pub inertia_tensor_origin: Matrix3<f64>,
}

impl Default for MassProperties {
    fn default() -> Self {
        Self::zero()
    }
}

#[inline]
fn clean_zero_scalar(v: f64, scale: f64) -> f64 {
    if v.abs() <= scale * 1e-15 {
        0.0
    } else {
        v
    }
}

impl MassProperties {
    /// Returns zero mass properties (used for fixed/static bodies or empty meshes).
    #[must_use]
    pub fn zero() -> Self {
        Self {
            volume: 0.0,
            mass: 0.0,
            inv_mass: 0.0,
            center_of_mass: [0.0; 3],
            local_com: Vector3::zeros(),
            inertia_tensor: Matrix3::zeros(),
            inv_inertia_tensor: Matrix3::zeros(),
            inertia_tensor_origin: Matrix3::zeros(),
        }
    }

    /// Computes exact mass properties from a closed triangulated boundary mesh and material
    /// density $\rho$ ($\text{kg}/\text{m}^3$) using canonical signed-tetrahedron integration.
    ///
    /// ### Mathematical Formulation
    /// Each boundary triangle $T = (\mathbf{v}_0, \mathbf{v}_1, \mathbf{v}_2)$ forms a signed
    /// tetrahedron with the reference point:
    /// 1. **Pass 1 (Volume & Center of Mass)**:
    ///    $$V_T = \frac{1}{6}\,\mathbf{v}_0 \cdot (\mathbf{v}_1 \times \mathbf{v}_2), \qquad
    ///    \mathbf{c}_{\text{loc}} = \frac{1}{V_{\text{signed}}}\sum_T V_T\,\frac{\mathbf{v}_0 + \mathbf{v}_1 + \mathbf{v}_2}{4}.$$
    /// 2. **Pass 2 (Second-Order Moment Covariance about $\mathbf{c}_{\text{loc}}$)**:
    ///    Shifting vertices to $\mathbf{u}_k = \mathbf{v}_k - \mathbf{c}_{\text{loc}}$ eliminates
    ///    catastrophic cancellation for parts located far from the origin. Over each shifted
    ///    tetrahedron $(\mathbf{0}, \mathbf{u}_0, \mathbf{u}_1, \mathbf{u}_2)$ with signed volume
    ///    $V_T^{(u)} = \operatorname{sgn}(V_{\text{signed}})\,\frac{1}{6}\mathbf{u}_0\cdot(\mathbf{u}_1\times\mathbf{u}_2)$
    ///    and vertex sum $\mathbf{s} = \mathbf{u}_0 + \mathbf{u}_1 + \mathbf{u}_2$:
    ///    $$C = \int_\Omega \mathbf{u}\mathbf{u}^T\,dV
    ///    = \sum_T \frac{V_T^{(u)}}{20}\left(\sum_{k=0}^2 \mathbf{u}_k\mathbf{u}_k^T + \mathbf{s}\mathbf{s}^T\right),$$
    ///    $$I_{\text{loc}} = \rho\left(\operatorname{tr}(C)\,I_3 - C\right).$$
    ///    Because internal cavity and through-hole walls have outward normals pointing into the
    ///    void, their signed tetrahedra naturally subtract the void's volume, static moment, and
    ///    inertia tensor from the outer envelope.
    #[must_use]
    pub fn from_mesh_and_density(mesh: &TriangleMesh, density_kg_m3: f64) -> Self {
        if mesh.vertices.is_empty() || mesh.indices.is_empty() || density_kg_m3 <= 0.0 {
            return Self::zero();
        }

        let mut signed_vol = 0.0_f64;
        let mut weighted_com = Vector3::<f64>::zeros();

        for tri in &mesh.indices {
            let Some(v0_vert) = mesh.vertices.get(tri[0] as usize) else {
                continue;
            };
            let Some(v1_vert) = mesh.vertices.get(tri[1] as usize) else {
                continue;
            };
            let Some(v2_vert) = mesh.vertices.get(tri[2] as usize) else {
                continue;
            };
            let v0 = v0_vert.pos_vec();
            let v1 = v1_vert.pos_vec();
            let v2 = v2_vert.pos_vec();

            let det6 = v0.dot(&v1.cross(&v2)) / 6.0;
            let tet_com = (v0 + v1 + v2) * 0.25;
            signed_vol += det6;
            weighted_com += tet_com * det6;
        }

        let volume = signed_vol.abs();
        if volume <= 1e-18 {
            let fallback_c = mesh.bounding_box().center();
            return Self {
                volume: 0.0,
                mass: 0.0,
                inv_mass: 0.0,
                center_of_mass: fallback_c,
                local_com: Vector3::new(fallback_c[0], fallback_c[1], fallback_c[2]),
                inertia_tensor: Matrix3::zeros(),
                inv_inertia_tensor: Matrix3::zeros(),
                inertia_tensor_origin: Matrix3::zeros(),
            };
        }

        let raw_com = weighted_com / signed_vol;
        let aabb = mesh.bounding_box();
        let dims = aabb.dimensions();
        let char_len = dims[0].max(dims[1]).max(dims[2]).max(1.0);
        let local_com = Vector3::new(
            clean_zero_scalar(raw_com.x, char_len),
            clean_zero_scalar(raw_com.y, char_len),
            clean_zero_scalar(raw_com.z, char_len),
        );
        let center_of_mass = [local_com.x, local_com.y, local_com.z];

        let orient_sign = if signed_vol >= 0.0 { 1.0_f64 } else { -1.0_f64 };
        let mut cov = Matrix3::<f64>::zeros();

        for tri in &mesh.indices {
            let Some(v0_vert) = mesh.vertices.get(tri[0] as usize) else {
                continue;
            };
            let Some(v1_vert) = mesh.vertices.get(tri[1] as usize) else {
                continue;
            };
            let Some(v2_vert) = mesh.vertices.get(tri[2] as usize) else {
                continue;
            };
            let u0 = v0_vert.pos_vec() - local_com;
            let u1 = v1_vert.pos_vec() - local_com;
            let u2 = v2_vert.pos_vec() - local_com;

            let v_tet = orient_sign * (u0.dot(&u1.cross(&u2)) / 6.0);
            let s = u0 + u1 + u2;
            let w = v_tet / 20.0;

            for r in 0..3 {
                for c in r..3 {
                    let val = w
                        * (u0[r] * u0[c]
                            + u1[r] * u1[c]
                            + u2[r] * u2[c]
                            + s[r] * s[c]);
                    cov[(r, c)] += val;
                }
            }
        }
        cov[(1, 0)] = cov[(0, 1)];
        cov[(2, 0)] = cov[(0, 2)];
        cov[(2, 1)] = cov[(1, 2)];

        let mass = volume * density_kg_m3;
        let inv_mass = if mass > 1e-15 { 1.0 / mass } else { 0.0 };

        let sigma = cov * density_kg_m3;
        let trace_sigma = sigma.trace();
        let mut inertia_tensor = Matrix3::from_diagonal_element(trace_sigma) - sigma;

        // Symmetrize and scrub numerical roundoff noise on off-diagonal terms
        let inertia_scale = trace_sigma.abs().max(1e-30);
        for r in 0..3 {
            inertia_tensor[(r, r)] = inertia_tensor[(r, r)].max(0.0);
            for c in (r + 1)..3 {
                let sym = 0.5 * (inertia_tensor[(r, c)] + inertia_tensor[(c, r)]);
                let cleaned = clean_zero_scalar(sym, inertia_scale);
                inertia_tensor[(r, c)] = cleaned;
                inertia_tensor[(c, r)] = cleaned;
            }
        }

        let inv_inertia_tensor = inertia_tensor.try_inverse().unwrap_or_else(Matrix3::zeros);

        // Parallel axis theorem (Steiner) for inertia tensor about body origin [0, 0, 0]
        let com_norm_sq = local_com.norm_squared();
        let steiner_shift =
            mass * (Matrix3::from_diagonal_element(com_norm_sq) - local_com * local_com.transpose());
        let inertia_tensor_origin = inertia_tensor + steiner_shift;

        Self {
            volume,
            mass,
            inv_mass,
            center_of_mass,
            local_com,
            inertia_tensor,
            inv_inertia_tensor,
            inertia_tensor_origin,
        }
    }

    /// Computes exact mass properties for a CAD [`Part`] from its evaluated mesh and material density.
    #[must_use]
    pub fn from_part(part: &Part) -> Self {
        Self::from_mesh_and_density(part.mesh(), part.material.density_kg_m3)
    }

    /// Decomposes `self.inertia_tensor` into principal moments of inertia `(I_x, I_y, I_z)`
    /// and a right-handed principal rotation frame `UnitQuaternion<f64>`.
    #[must_use]
    pub fn principal_inertia_and_frame(&self) -> (Vector3<f64>, UnitQuaternion<f64>) {
        // Fast path for already-diagonal inertia tensors
        let off_diag = self.inertia_tensor[(0, 1)].abs()
            + self.inertia_tensor[(0, 2)].abs()
            + self.inertia_tensor[(1, 2)].abs();
        let trace = self.inertia_tensor.trace().abs().max(1e-30);
        if off_diag <= trace * 1e-12 {
            let diag = Vector3::new(
                self.inertia_tensor[(0, 0)].max(0.0),
                self.inertia_tensor[(1, 1)].max(0.0),
                self.inertia_tensor[(2, 2)].max(0.0),
            );
            return (diag, UnitQuaternion::identity());
        }

        let eig = self.inertia_tensor.symmetric_eigen();
        let mut basis = eig.eigenvectors;
        if basis.determinant() < 0.0 {
            basis.set_column(2, &(-basis.column(2)));
        }
        let rot = Rotation3::from_matrix_unchecked(basis);
        let principal = Vector3::new(
            eig.eigenvalues[0].max(0.0),
            eig.eigenvalues[1].max(0.0),
            eig.eigenvalues[2].max(0.0),
        );
        (principal, UnitQuaternion::from_rotation_matrix(&rot))
    }
}

/// Computes exact mass-inertia properties from a closed [`TriangleMesh`] and material density.
#[must_use]
pub fn compute_mesh_mass_properties(mesh: &TriangleMesh, density_kg_m3: f64) -> MassProperties {
    MassProperties::from_mesh_and_density(mesh, density_kg_m3)
}

/// A 3D rigid body in double precision (`f64`) with exact mesh-derived mass and $3\times 3$ inertia tensor.
#[derive(Debug, Clone, PartialEq)]
pub struct RigidBody {
    /// Unique handle assigned by [`BodyRegistry`].
    pub handle: BodyHandle,
    /// Identifier of the CAD [`Part`] (or `"ground"` / `"world"`).
    pub part_id: String,
    /// `true` if the body is movable (dynamic), `false` if fixed in world space.
    pub is_dynamic: bool,
    /// `true` if the body is fixed/static in world space (`!is_dynamic`).
    pub is_fixed: bool,
    /// Mass $m$ in kilograms (`0.0` for fixed bodies).
    pub mass: f64,
    /// Inverse mass $m^{-1}$ in $\text{kg}^{-1}$ (`0.0` for fixed bodies).
    pub inv_mass: f64,
    /// Exact center of mass $\mathbf{c}_{\text{loc}}$ in body-local coordinates ($\text{m}$).
    pub local_com: Vector3<f64>,
    /// Exact $3\times 3$ inertia tensor $I_{\text{loc}}$ about `local_com` in body-local axes
    /// ($\text{kg}\cdot\text{m}^2$).
    pub local_inertia: Matrix3<f64>,
    /// Inverse of `local_inertia` ($I_{\text{loc}}^{-1}$) in body-local axes
    /// ($\text{kg}^{-1}\cdot\text{m}^{-2}$).
    pub local_inv_inertia: Matrix3<f64>,
    /// $3\times 3$ inertia tensor about the body origin $[0, 0, 0]$ in body-local axes ($\text{kg}\cdot\text{m}^2$).
    pub local_inertia_origin: Matrix3<f64>,
    /// World-space position $[x, y, z]$ of the body frame origin ($\text{m}$).
    pub position: Vector3<f64>,
    /// World-space orientation unit quaternion of the body frame.
    pub orientation: UnitQuaternion<f64>,
    /// World-space linear velocity $[v_x, v_y, v_z]$ of the body ($\text{m}/\text{s}$).
    pub linear_velocity: Vector3<f64>,
    /// World-space angular velocity $[\omega_x, \omega_y, \omega_z]$ ($\text{rad}/\text{s}$).
    pub angular_velocity: Vector3<f64>,
    /// Accumulated continuous external force $[F_x, F_y, F_z]$ in world space ($\text{N}$).
    pub external_force: Vector3<f64>,
    /// Accumulated continuous external torque $[\tau_x, \tau_y, \tau_z]$ in world space ($\text{N}\cdot\text{m}$).
    pub external_torque: Vector3<f64>,
    /// Whether the body is currently sleeping (at rest).
    pub sleeping: bool,
    /// Handles of colliders attached to this rigid body.
    pub colliders: Vec<ColliderHandle>,
}

impl RigidBody {
    /// Creates a fixed (static/ground) rigid body at the given world pose.
    #[must_use]
    pub fn new_fixed(
        part_id: impl Into<String>,
        position: Vector3<f64>,
        orientation: UnitQuaternion<f64>,
    ) -> Self {
        Self {
            handle: BodyHandle(0),
            part_id: part_id.into(),
            is_dynamic: false,
            is_fixed: true,
            mass: 0.0,
            inv_mass: 0.0,
            local_com: Vector3::zeros(),
            local_inertia: Matrix3::zeros(),
            local_inv_inertia: Matrix3::zeros(),
            local_inertia_origin: Matrix3::zeros(),
            position,
            orientation,
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            external_force: Vector3::zeros(),
            external_torque: Vector3::zeros(),
            sleeping: false,
            colliders: Vec::new(),
        }
    }

    /// Constructs a [`RigidBody`] from a CAD [`Part`], computing exact `f64` mass, center of mass,
    /// and $3\times 3$ inertia tensor directly from the part's boundary mesh (including internal
    /// cavities and through-holes).
    #[must_use]
    pub fn from_part(
        part: &Part,
        is_dynamic: bool,
        initial_position: Option<[f64; 3]>,
        initial_rotation: Option<[f64; 4]>,
    ) -> Self {
        let mprops = MassProperties::from_part(part);
        let position = initial_position
            .map(|[x, y, z]| Vector3::new(x, y, z))
            .unwrap_or_else(Vector3::zeros);
        let orientation = initial_rotation
            .map(|[x, y, z, w]| {
                let q = Quaternion::new(w, x, y, z);
                if q.norm() > 1e-12 {
                    UnitQuaternion::from_quaternion(q)
                } else {
                    UnitQuaternion::identity()
                }
            })
            .unwrap_or_else(UnitQuaternion::identity);

        let effective_dynamic = is_dynamic && mprops.mass > 0.0;
        Self {
            handle: BodyHandle(0),
            part_id: part.id.clone(),
            is_dynamic,
            is_fixed: !is_dynamic,
            mass: mprops.mass,
            inv_mass: if effective_dynamic { mprops.inv_mass } else { 0.0 },
            local_com: mprops.local_com,
            local_inertia: mprops.inertia_tensor,
            local_inv_inertia: if effective_dynamic {
                mprops.inv_inertia_tensor
            } else {
                Matrix3::zeros()
            },
            local_inertia_origin: mprops.inertia_tensor_origin,
            position,
            orientation,
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            external_force: Vector3::zeros(),
            external_torque: Vector3::zeros(),
            sleeping: false,
            colliders: Vec::new(),
        }
    }

    /// Recomputes and updates mass, center of mass, and $3\times 3$ inertia tensor from `part`
    /// while preserving the body's handle, world pose, velocities, and attached joints.
    pub fn update_mass_properties_from_part(&mut self, part: &Part, is_dynamic: bool) {
        let mprops = MassProperties::from_part(part);
        let effective_dynamic = is_dynamic && mprops.mass > 0.0;
        self.is_dynamic = is_dynamic;
        self.is_fixed = !is_dynamic;
        self.mass = mprops.mass;
        self.inv_mass = if effective_dynamic { mprops.inv_mass } else { 0.0 };
        self.local_com = mprops.local_com;
        self.local_inertia = mprops.inertia_tensor;
        self.local_inv_inertia = if effective_dynamic {
            mprops.inv_inertia_tensor
        } else {
            Matrix3::zeros()
        };
        self.local_inertia_origin = mprops.inertia_tensor_origin;
        if !is_dynamic {
            self.linear_velocity = Vector3::zeros();
            self.angular_velocity = Vector3::zeros();
        }
        self.sleeping = false;
    }

    /// Constructs a dynamic [`RigidBody`] directly from exact FEA-integrated mass, center of mass,
    /// $3\times 3$ inertia tensor about COM, world pose, and initial linear/angular velocities (`P5-T5`).
    #[allow(clippy::too_many_arguments)]
    #[must_use]
    pub fn from_exact_mass_properties(
        part_id: impl Into<String>,
        mass_kg: f64,
        local_com: [f64; 3],
        inertia_tensor_com: [[f64; 3]; 3],
        position: [f64; 3],
        orientation_quat: [f64; 4],
        linear_velocity: [f64; 3],
        angular_velocity: [f64; 3],
    ) -> Self {
        let mass = mass_kg.max(0.0);
        let is_dynamic = mass > 1e-15;
        let inv_mass = if is_dynamic { 1.0 / mass } else { 0.0 };
        let com_v = Vector3::new(local_com[0], local_com[1], local_com[2]);
        let mut inertia = Matrix3::new(
            inertia_tensor_com[0][0], inertia_tensor_com[0][1], inertia_tensor_com[0][2],
            inertia_tensor_com[1][0], inertia_tensor_com[1][1], inertia_tensor_com[1][2],
            inertia_tensor_com[2][0], inertia_tensor_com[2][1], inertia_tensor_com[2][2],
        );
        for r in 0..3 {
            inertia[(r, r)] = inertia[(r, r)].max(0.0);
            for c in (r + 1)..3 {
                let sym = 0.5 * (inertia[(r, c)] + inertia[(c, r)]);
                inertia[(r, c)] = sym;
                inertia[(c, r)] = sym;
            }
        }
        let inv_inertia = if is_dynamic {
            inertia.try_inverse().unwrap_or_else(Matrix3::zeros)
        } else {
            Matrix3::zeros()
        };
        let com_norm_sq = com_v.norm_squared();
        let steiner_shift =
            mass * (Matrix3::from_diagonal_element(com_norm_sq) - com_v * com_v.transpose());
        let inertia_origin = inertia + steiner_shift;

        let q = Quaternion::new(
            orientation_quat[3],
            orientation_quat[0],
            orientation_quat[1],
            orientation_quat[2],
        );
        let orientation = if q.norm() > 1e-12 {
            UnitQuaternion::from_quaternion(q)
        } else {
            UnitQuaternion::identity()
        };

        Self {
            handle: BodyHandle(0),
            part_id: part_id.into(),
            is_dynamic,
            is_fixed: !is_dynamic,
            mass,
            inv_mass,
            local_com: com_v,
            local_inertia: inertia,
            local_inv_inertia: inv_inertia,
            local_inertia_origin: inertia_origin,
            position: Vector3::new(position[0], position[1], position[2]),
            orientation,
            linear_velocity: if is_dynamic {
                Vector3::new(linear_velocity[0], linear_velocity[1], linear_velocity[2])
            } else {
                Vector3::zeros()
            },
            angular_velocity: if is_dynamic {
                Vector3::new(angular_velocity[0], angular_velocity[1], angular_velocity[2])
            } else {
                Vector3::zeros()
            },
            external_force: Vector3::zeros(),
            external_torque: Vector3::zeros(),
            sleeping: false,
            colliders: Vec::new(),
        }
    }

    /// Updates exact mass, local COM, and $3\times 3$ inertia tensor about COM in place while
    /// preserving `BodyHandle`, world pose, velocities, and attached joints (`P5-T5`).
    pub fn update_exact_mass_properties(
        &mut self,
        mass_kg: f64,
        local_com: [f64; 3],
        inertia_tensor_com: [[f64; 3]; 3],
    ) {
        let mass = mass_kg.max(0.0);
        let effective_dynamic = self.is_dynamic && mass > 1e-15;
        let com_v = Vector3::new(local_com[0], local_com[1], local_com[2]);
        let mut inertia = Matrix3::new(
            inertia_tensor_com[0][0], inertia_tensor_com[0][1], inertia_tensor_com[0][2],
            inertia_tensor_com[1][0], inertia_tensor_com[1][1], inertia_tensor_com[1][2],
            inertia_tensor_com[2][0], inertia_tensor_com[2][1], inertia_tensor_com[2][2],
        );
        for r in 0..3 {
            inertia[(r, r)] = inertia[(r, r)].max(0.0);
            for c in (r + 1)..3 {
                let sym = 0.5 * (inertia[(r, c)] + inertia[(c, r)]);
                inertia[(r, c)] = sym;
                inertia[(c, r)] = sym;
            }
        }
        let inv_inertia = if effective_dynamic {
            inertia.try_inverse().unwrap_or_else(Matrix3::zeros)
        } else {
            Matrix3::zeros()
        };
        let com_norm_sq = com_v.norm_squared();
        let steiner_shift =
            mass * (Matrix3::from_diagonal_element(com_norm_sq) - com_v * com_v.transpose());

        self.mass = mass;
        self.inv_mass = if effective_dynamic { 1.0 / mass } else { 0.0 };
        self.local_com = com_v;
        self.local_inertia = inertia;
        self.local_inv_inertia = inv_inertia;
        self.local_inertia_origin = inertia + steiner_shift;
        self.sleeping = false;
    }

    /// Returns `true` if the rigid body is dynamic (movable).
    #[inline]
    #[must_use]
    pub fn is_dynamic(&self) -> bool {
        self.is_dynamic
    }

    /// Returns `true` if the rigid body is fixed (static).
    #[inline]
    #[must_use]
    pub fn is_fixed(&self) -> bool {
        self.is_fixed
    }

    /// Returns the mass $m$ in kilograms.
    #[inline]
    #[must_use]
    pub fn mass(&self) -> f64 {
        self.mass
    }

    /// Returns the inverse mass $m^{-1}$ in $\text{kg}^{-1}$ (`0.0` if fixed).
    #[inline]
    #[must_use]
    pub fn inv_mass(&self) -> f64 {
        if self.is_dynamic {
            self.inv_mass
        } else {
            0.0
        }
    }

    /// Returns the exact center of mass $\mathbf{c}_{\text{loc}}$ in body-local coordinates ($\text{m}$).
    #[inline]
    #[must_use]
    pub fn local_com(&self) -> Vector3<f64> {
        self.local_com
    }

    /// Returns the world-space center of mass $\mathbf{x}_{\text{com}} = \mathbf{p} + R\,\mathbf{c}_{\text{loc}}$.
    #[inline]
    #[must_use]
    pub fn world_com(&self) -> Vector3<f64> {
        self.position + self.orientation * self.local_com
    }

    /// Returns the $3\times 3$ world rotation matrix $R \in SO(3)$.
    #[inline]
    #[must_use]
    pub fn rotation_matrix(&self) -> Matrix3<f64> {
        *self.orientation.to_rotation_matrix().matrix()
    }

    /// Returns the $3\times 3$ inertia tensor $I_{\text{loc}}$ about `local_com` in body-local axes.
    #[inline]
    #[must_use]
    pub fn local_inertia(&self) -> Matrix3<f64> {
        self.local_inertia
    }

    /// Returns the $3\times 3$ inverse inertia tensor $I_{\text{loc}}^{-1}$ in body-local axes.
    #[inline]
    #[must_use]
    pub fn local_inv_inertia(&self) -> Matrix3<f64> {
        if self.is_dynamic {
            self.local_inv_inertia
        } else {
            Matrix3::zeros()
        }
    }

    /// Returns the $3\times 3$ world-space inertia tensor $I_w = R I_{\text{loc}} R^T$ about the COM.
    #[must_use]
    pub fn world_inertia(&self) -> Matrix3<f64> {
        if !self.is_dynamic {
            return Matrix3::zeros();
        }
        let r = self.rotation_matrix();
        r * self.local_inertia * r.transpose()
    }

    /// Returns the $3\times 3$ world-space inverse inertia tensor $I_w^{-1} = R I_{\text{loc}}^{-1} R^T$.
    #[must_use]
    pub fn world_inv_inertia(&self) -> Matrix3<f64> {
        if !self.is_dynamic {
            return Matrix3::zeros();
        }
        let r = self.rotation_matrix();
        r * self.local_inv_inertia * r.transpose()
    }

    /// World-space translation vector of the body origin ($\text{m}$).
    #[inline]
    #[must_use]
    pub fn translation(&self) -> Vector3<f64> {
        self.position
    }

    /// World-space orientation unit quaternion of the body.
    #[inline]
    #[must_use]
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        self.orientation
    }

    /// World-space linear velocity vector ($\text{m}/\text{s}$).
    #[inline]
    #[must_use]
    pub fn linvel(&self) -> Vector3<f64> {
        self.linear_velocity
    }

    /// World-space angular velocity vector ($\text{rad}/\text{s}$).
    #[inline]
    #[must_use]
    pub fn angvel(&self) -> Vector3<f64> {
        self.angular_velocity
    }

    /// Sets the world-space linear velocity vector ($\text{m}/\text{s}$).
    #[inline]
    pub fn set_linvel(&mut self, linvel: Vector3<f64>) {
        if self.is_dynamic {
            self.linear_velocity = linvel;
        } else {
            self.linear_velocity = Vector3::zeros();
        }
        self.sleeping = false;
    }

    /// Sets the world-space angular velocity vector ($\text{rad}/\text{s}$).
    #[inline]
    pub fn set_angvel(&mut self, angvel: Vector3<f64>) {
        if self.is_dynamic {
            self.angular_velocity = angvel;
        } else {
            self.angular_velocity = Vector3::zeros();
        }
        self.sleeping = false;
    }

    /// Sets the world-space position and orientation of the body.
    #[inline]
    pub fn set_pose(&mut self, position: Vector3<f64>, orientation: UnitQuaternion<f64>) {
        self.position = position;
        self.orientation = orientation;
        self.sleeping = false;
    }

    /// Adds a continuous external force vector in world space ($\text{N}$).
    #[inline]
    pub fn add_force(&mut self, force: Vector3<f64>) {
        if self.is_dynamic {
            self.external_force += force;
            self.sleeping = false;
        }
    }

    /// Adds a continuous external torque vector in world space ($\text{N}\cdot\text{m}$).
    #[inline]
    pub fn add_torque(&mut self, torque: Vector3<f64>) {
        if self.is_dynamic {
            self.external_torque += torque;
            self.sleeping = false;
        }
    }

    /// Resets accumulated continuous external forces and torques to zero.
    #[inline]
    pub fn clear_forces(&mut self) {
        self.external_force = Vector3::zeros();
        self.external_torque = Vector3::zeros();
    }

    /// Applies an instantaneous linear impulse $\mathbf{J}$ ($\text{N}\cdot\text{s}$) at the COM:
    /// $\Delta\mathbf{v} = m^{-1}\mathbf{J}$.
    #[inline]
    pub fn apply_impulse(&mut self, impulse: Vector3<f64>) {
        if self.is_dynamic && self.inv_mass > 0.0 {
            self.linear_velocity += impulse * self.inv_mass;
            self.sleeping = false;
        }
    }

    /// Applies an instantaneous angular impulse $\mathbf{L}$ ($\text{N}\cdot\text{m}\cdot\text{s}$):
    /// $\Delta\boldsymbol{\omega} = I_w^{-1}\mathbf{L}$.
    #[inline]
    pub fn apply_torque_impulse(&mut self, torque_impulse: Vector3<f64>) {
        if self.is_dynamic {
            self.angular_velocity += self.world_inv_inertia() * torque_impulse;
            self.sleeping = false;
        }
    }

    /// Wakes up the rigid body.
    #[inline]
    pub fn wake_up(&mut self) {
        self.sleeping = false;
    }

    /// Computes `(principal_inertia, principal_local_frame)` of `self.local_inertia`.
    #[must_use]
    pub fn principal_inertia_and_frame(&self) -> (Vector3<f64>, UnitQuaternion<f64>) {
        let mprops = MassProperties {
            volume: 0.0,
            mass: self.mass,
            inv_mass: self.inv_mass,
            center_of_mass: [self.local_com.x, self.local_com.y, self.local_com.z],
            local_com: self.local_com,
            inertia_tensor: self.local_inertia,
            inv_inertia_tensor: self.local_inv_inertia,
            inertia_tensor_origin: self.local_inertia_origin,
        };
        mprops.principal_inertia_and_frame()
    }
}

/// Deterministic registry storing [`RigidBody`] instances indexed by [`BodyHandle`].
#[derive(Debug, Clone, Default)]
pub struct BodyRegistry {
    bodies: HashMap<BodyHandle, RigidBody>,
    order: Vec<BodyHandle>,
    next_id: usize,
}

impl BodyRegistry {
    /// Creates an empty [`BodyRegistry`].
    #[must_use]
    pub fn new() -> Self {
        Self {
            bodies: HashMap::new(),
            order: Vec::new(),
            next_id: 0,
        }
    }

    /// Inserts a [`RigidBody`] and assigns it a unique [`BodyHandle`].
    pub fn insert(&mut self, mut body: RigidBody) -> BodyHandle {
        let handle = BodyHandle(self.next_id);
        self.next_id += 1;
        body.handle = handle;
        self.bodies.insert(handle, body);
        self.order.push(handle);
        handle
    }

    /// Retrieves a reference to a [`RigidBody`] by handle.
    #[inline]
    #[must_use]
    pub fn get(&self, handle: BodyHandle) -> Option<&RigidBody> {
        self.bodies.get(&handle)
    }

    /// Retrieves a mutable reference to a [`RigidBody`] by handle.
    #[inline]
    #[must_use]
    pub fn get_mut(&mut self, handle: BodyHandle) -> Option<&mut RigidBody> {
        self.bodies.get_mut(&handle)
    }

    /// Removes a [`RigidBody`] by handle, returning it if present.
    pub fn remove(&mut self, handle: BodyHandle) -> Option<RigidBody> {
        self.order.retain(|&h| h != handle);
        self.bodies.remove(&handle)
    }

    /// Returns `true` if `handle` is present in the registry.
    #[inline]
    #[must_use]
    pub fn contains(&self, handle: BodyHandle) -> bool {
        self.bodies.contains_key(&handle)
    }

    /// Number of bodies in the registry.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Returns `true` if the registry contains no bodies.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    /// Iterates over `(BodyHandle, &RigidBody)` in deterministic insertion order.
    pub fn iter(&self) -> impl Iterator<Item = (BodyHandle, &RigidBody)> {
        self.order
            .iter()
            .filter_map(|&h| self.bodies.get(&h).map(|b| (h, b)))
    }

    /// Returns the deterministic list of active [`BodyHandle`]s.
    #[inline]
    #[must_use]
    pub fn handles(&self) -> &[BodyHandle] {
        &self.order
    }
}

/// A single convex hull component inside a [`ColliderShape::Compound`] decomposition.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConvexHullPart {
    /// Convex hull extreme vertices in local part coordinates ($\text{m}$).
    pub points: Vec<[f64; 3]>,
    /// Outward-oriented triangular faces indexing into `points`.
    pub faces: Vec<[u32; 3]>,
    /// Local axis-aligned bounding box of this convex sub-part.
    pub local_aabb: BoundingBox,
}

/// Geometric shape representation of a [`Collider`] in double precision (`f64`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ColliderShape {
    /// 3D convex polyhedron (`points` and outward `faces`) enclosing the part.
    ConvexHull {
        /// Hull vertices in collider-local coordinates ($\text{m}$).
        points: Vec<[f64; 3]>,
        /// Outward triangular faces indexing into `points`.
        faces: Vec<[u32; 3]>,
    },
    /// Exact triangular boundary surface mesh in `f64`.
    TriMesh {
        /// Vertex positions in collider-local coordinates ($\text{m}$).
        vertices: Vec<[f64; 3]>,
        /// Triangle vertex index triples.
        indices: Vec<[u32; 3]>,
    },
    /// Compound shape composed of multiple convex polyhedra.
    Compound {
        /// Convex hull sub-parts.
        parts: Vec<ConvexHullPart>,
    },
    /// Box / cuboid primitive with half-extents $[h_x, h_y, h_z]$ ($\text{m}$).
    Cube {
        /// Half-extents along local X, Y, Z axes ($\text{m}$).
        half_extents: [f64; 3],
    },
    /// Sphere primitive with radius $r$ ($\text{m}$).
    Sphere {
        /// Sphere radius in meters.
        radius: f64,
    },
    /// Cylinder primitive with `half_height`, `radius`, and local symmetry `axis`.
    Cylinder {
        /// Half-height along the cylinder's axis ($\text{m}$).
        half_height: f64,
        /// Cylinder radius perpendicular to the axis ($\text{m}$).
        radius: f64,
        /// Unit axis direction in part-local coordinates (`[0,0,1]`, `[1,0,0]`, or `[0,1,0]`).
        axis: [f64; 3],
    },
}

impl ColliderShape {
    /// Computes the local-space `BoundingBox` of the shape around its local origin.
    #[must_use]
    pub fn compute_local_aabb(&self) -> BoundingBox {
        match self {
            ColliderShape::ConvexHull { points, .. } => aabb_from_points(points),
            ColliderShape::TriMesh { vertices, .. } => aabb_from_points(vertices),
            ColliderShape::Compound { parts } => {
                if parts.is_empty() {
                    return BoundingBox::new([0.0; 3], [0.0; 3]);
                }
                let mut min = [f64::INFINITY; 3];
                let mut max = [f64::NEG_INFINITY; 3];
                for p in parts {
                    for i in 0..3 {
                        min[i] = min[i].min(p.local_aabb.min[i]);
                        max[i] = max[i].max(p.local_aabb.max[i]);
                    }
                }
                BoundingBox::new(min, max)
            }
            ColliderShape::Cube { half_extents } => BoundingBox::new(
                [-half_extents[0], -half_extents[1], -half_extents[2]],
                [half_extents[0], half_extents[1], half_extents[2]],
            ),
            ColliderShape::Sphere { radius } => {
                BoundingBox::new([-radius, -radius, -radius], [*radius, *radius, *radius])
            }
            ColliderShape::Cylinder {
                half_height,
                radius,
                axis,
            } => {
                // In collider-local space (before `local_rotation`), the cylinder primitive is
                // aligned along local Y (`[0, 1, 0]`), and rotated by `local_rotation` to `axis`.
                let ax = Vector3::new(axis[0], axis[1], axis[2]);
                let a = if ax.norm() > 1e-12 {
                    ax.normalize()
                } else {
                    Vector3::y()
                };
                let ex = half_height * a.x.abs() + radius * (1.0 - a.x * a.x).max(0.0).sqrt();
                let ey = half_height * a.y.abs() + radius * (1.0 - a.y * a.y).max(0.0).sqrt();
                let ez = half_height * a.z.abs() + radius * (1.0 - a.z * a.z).max(0.0).sqrt();
                BoundingBox::new([-ex, -ey, -ez], [ex, ey, ez])
            }
        }
    }

    /// Computes the exact world-space `BoundingBox` given the collider's world translation
    /// `world_pos` and world rotation `world_rot` (and for `Cylinder`, the parent body's world rotation).
    #[must_use]
    pub fn compute_world_aabb(
        &self,
        world_pos: Vector3<f64>,
        world_rot: UnitQuaternion<f64>,
        body_rot: UnitQuaternion<f64>,
    ) -> BoundingBox {
        let r_mat = *world_rot.to_rotation_matrix().matrix();
        match self {
            ColliderShape::ConvexHull { points, .. } => {
                transform_points_aabb(points, world_pos, &r_mat)
            }
            ColliderShape::TriMesh { vertices, .. } => {
                transform_points_aabb(vertices, world_pos, &r_mat)
            }
            ColliderShape::Compound { parts } => {
                if parts.is_empty() {
                    return BoundingBox::new(
                        [world_pos.x, world_pos.y, world_pos.z],
                        [world_pos.x, world_pos.y, world_pos.z],
                    );
                }
                let mut min = [f64::INFINITY; 3];
                let mut max = [f64::NEG_INFINITY; 3];
                for part in parts {
                    let b = transform_points_aabb(&part.points, world_pos, &r_mat);
                    for i in 0..3 {
                        min[i] = min[i].min(b.min[i]);
                        max[i] = max[i].max(b.max[i]);
                    }
                }
                BoundingBox::new(min, max)
            }
            ColliderShape::Cube { half_extents } => {
                let abs_r = r_mat.map(f64::abs);
                let he = Vector3::new(half_extents[0], half_extents[1], half_extents[2]);
                let ext = abs_r * he;
                BoundingBox::new(
                    [world_pos.x - ext.x, world_pos.y - ext.y, world_pos.z - ext.z],
                    [world_pos.x + ext.x, world_pos.y + ext.y, world_pos.z + ext.z],
                )
            }
            ColliderShape::Sphere { radius } => BoundingBox::new(
                [
                    world_pos.x - radius,
                    world_pos.y - radius,
                    world_pos.z - radius,
                ],
                [
                    world_pos.x + radius,
                    world_pos.y + radius,
                    world_pos.z + radius,
                ],
            ),
            ColliderShape::Cylinder {
                half_height,
                radius,
                axis,
            } => {
                let local_axis = Vector3::new(axis[0], axis[1], axis[2]);
                let world_axis = (body_rot
                    * if local_axis.norm() > 1e-12 {
                        local_axis.normalize()
                    } else {
                        Vector3::y()
                    })
                .normalize();
                let ex = half_height * world_axis.x.abs()
                    + radius * (1.0 - world_axis.x * world_axis.x).max(0.0).sqrt();
                let ey = half_height * world_axis.y.abs()
                    + radius * (1.0 - world_axis.y * world_axis.y).max(0.0).sqrt();
                let ez = half_height * world_axis.z.abs()
                    + radius * (1.0 - world_axis.z * world_axis.z).max(0.0).sqrt();
                BoundingBox::new(
                    [world_pos.x - ex, world_pos.y - ey, world_pos.z - ez],
                    [world_pos.x + ex, world_pos.y + ey, world_pos.z + ez],
                )
            }
        }
    }
}

/// A double-precision (`f64`) collision shape attached to a [`RigidBody`].
#[derive(Debug, Clone, PartialEq)]
pub struct Collider {
    /// Unique collider handle assigned by [`ColliderRegistry`].
    pub handle: ColliderHandle,
    /// Parent [`BodyHandle`] if attached to a rigid body.
    pub parent: Option<BodyHandle>,
    /// Identifier of the associated CAD [`Part`].
    pub part_id: String,
    /// Approximation strategy used to construct `shape`.
    pub collider_type: ColliderType,
    /// Geometric shape representation in `f64`.
    pub shape: ColliderShape,
    /// Local translation offset relative to parent rigid body origin ($\text{m}$).
    pub local_translation: Vector3<f64>,
    /// Local rotation offset relative to parent rigid body frame.
    pub local_rotation: UnitQuaternion<f64>,
    /// Current world-space position of the collider ($\text{m}$).
    pub world_translation: Vector3<f64>,
    /// Current world-space orientation of the collider.
    pub world_rotation: UnitQuaternion<f64>,
    /// Material density $\rho$ ($\text{kg}/\text{m}^3$).
    pub density: f64,
    /// Exact computed part mass $m$ ($\text{kg}$).
    pub mass: f64,
    /// Exact mass-inertia properties computed from the CAD part mesh.
    pub mass_properties: MassProperties,
    /// Coulomb friction coefficient $\mu \ge 0$.
    pub friction: f64,
    /// Coefficient of restitution $e \in [0, 1]$.
    pub restitution: f64,
    /// Cached local-space axis-aligned bounding box ($\text{m}$).
    pub local_aabb: BoundingBox,
    /// Cached world-space axis-aligned bounding box ($\text{m}$).
    pub world_aabb: BoundingBox,
    /// Cached local-space BVH acceleration structure for [`ColliderShape::TriMesh`] colliders.
    pub tri_bvh: Option<TriMeshBvh>,
}

impl Collider {
    /// Builds a [`Collider`] in `f64` from a CAD [`Part`] and [`ColliderType`].
    pub fn from_part(part: &Part, collider_type: ColliderType) -> Result<Self, PhysicsError> {
        Self::from_mesh(
            &part.id,
            &part.mesh,
            part.material.density_kg_m3,
            collider_type,
            0.5,
            0.0,
        )
    }

    /// Builds a [`Collider`] in `f64` directly from a closed [`TriangleMesh`], density,
    /// [`ColliderType`], and contact properties (used for CAD parts and detached FEA fragments).
    pub fn from_mesh(
        part_id: impl Into<String>,
        mesh: &TriangleMesh,
        density_kg_m3: f64,
        collider_type: ColliderType,
        friction: f64,
        restitution: f64,
    ) -> Result<Self, PhysicsError> {
        let part_id = part_id.into();
        if mesh.vertices.is_empty() {
            return Err(PhysicsError::InvalidGeometry(
                part_id,
                "Part mesh is empty. Rebuild the part DAG first.".into(),
            ));
        }

        let mut local_translation = Vector3::zeros();
        let mut local_rotation = UnitQuaternion::identity();

        let shape = match collider_type {
            ColliderType::ConvexHull => {
                if mesh.vertices.len() < 4 {
                    return Err(PhysicsError::ColliderCreationFailed(
                        part_id,
                        "At least 4 vertices required for convex hull collider".into(),
                    ));
                }
                let raw_points: Vec<[f64; 3]> =
                    mesh.vertices.iter().map(|v| v.position).collect();
                let (points, faces) = compute_convex_hull_f64(&raw_points).ok_or_else(|| {
                    PhysicsError::ColliderCreationFailed(
                        part_id.clone(),
                        "Convex hull computation failed: points may be coplanar or collinear"
                            .into(),
                    )
                })?;
                ColliderShape::ConvexHull { points, faces }
            }
            ColliderType::TriMesh => {
                if mesh.indices.is_empty() {
                    return Err(PhysicsError::ColliderCreationFailed(
                        part_id,
                        "TriMesh collider requires at least one triangle index face".into(),
                    ));
                }
                let vertices: Vec<[f64; 3]> = mesh.vertices.iter().map(|v| v.position).collect();
                let indices = mesh.indices.clone();
                ColliderShape::TriMesh { vertices, indices }
            }
            ColliderType::Compound => {
                if mesh.vertices.len() < 4 || mesh.indices.is_empty() {
                    return Err(PhysicsError::ColliderCreationFailed(
                        part_id,
                        "Compound decomposition requires vertices and faces".into(),
                    ));
                }
                let parts = compute_compound_convex_parts_f64(mesh).ok_or_else(|| {
                    PhysicsError::ColliderCreationFailed(
                        part_id.clone(),
                        "Compound convex decomposition failed: mesh may be degenerate".into(),
                    )
                })?;
                ColliderShape::Compound { parts }
            }
            ColliderType::Cube => {
                let aabb = mesh.bounding_box();
                let dims = aabb.dimensions();
                let hx = (dims[0] * 0.5).max(1e-4);
                let hy = (dims[1] * 0.5).max(1e-4);
                let hz = (dims[2] * 0.5).max(1e-4);
                let center = aabb.center();
                local_translation = Vector3::new(center[0], center[1], center[2]);
                ColliderShape::Cube {
                    half_extents: [hx, hy, hz],
                }
            }
            ColliderType::Sphere => {
                let aabb = mesh.bounding_box();
                let dims = aabb.dimensions();
                let radius = (dims[0].max(dims[1]).max(dims[2]) * 0.5).max(1e-4);
                let center = aabb.center();
                local_translation = Vector3::new(center[0], center[1], center[2]);
                ColliderShape::Sphere { radius }
            }
            ColliderType::Cylinder => {
                let aabb = mesh.bounding_box();
                let dims = aabb.dimensions();
                let center = aabb.center();
                local_translation = Vector3::new(center[0], center[1], center[2]);

                if dims[2] > dims[0] && dims[2] > dims[1] {
                    let radius = (dims[0].max(dims[1]) * 0.5).max(1e-4);
                    let half_height = (dims[2] * 0.5).max(1e-4);
                    let rot = Rotation3::from_axis_angle(
                        &Vector3::x_axis(),
                        std::f64::consts::FRAC_PI_2,
                    );
                    local_rotation = UnitQuaternion::from_rotation_matrix(&rot);
                    ColliderShape::Cylinder {
                        half_height,
                        radius,
                        axis: [0.0, 0.0, 1.0],
                    }
                } else if dims[0] > dims[1] && dims[0] > dims[2] {
                    let radius = (dims[1].max(dims[2]) * 0.5).max(1e-4);
                    let half_height = (dims[0] * 0.5).max(1e-4);
                    let rot = Rotation3::from_axis_angle(
                        &Vector3::z_axis(),
                        std::f64::consts::FRAC_PI_2,
                    );
                    local_rotation = UnitQuaternion::from_rotation_matrix(&rot);
                    ColliderShape::Cylinder {
                        half_height,
                        radius,
                        axis: [1.0, 0.0, 0.0],
                    }
                } else {
                    let radius = (dims[0].max(dims[2]) * 0.5).max(1e-4);
                    let half_height = (dims[1] * 0.5).max(1e-4);
                    ColliderShape::Cylinder {
                        half_height,
                        radius,
                        axis: [0.0, 1.0, 0.0],
                    }
                }
            }
        };

        let density = density_kg_m3.max(0.0);
        let mass_properties = MassProperties::from_mesh_and_density(mesh, density);
        let mass = mass_properties.mass;

        let world_translation = local_translation;
        let world_rotation = local_rotation;
        let local_aabb = shape.compute_world_aabb(
            local_translation,
            local_rotation,
            UnitQuaternion::identity(),
        );
        let world_aabb = local_aabb;
        let tri_bvh = match &shape {
            ColliderShape::TriMesh { vertices, indices } => {
                Some(TriMeshBvh::build(vertices, indices))
            }
            _ => None,
        };

        Ok(Self {
            handle: ColliderHandle(0),
            parent: None,
            part_id,
            collider_type,
            shape,
            local_translation,
            local_rotation,
            world_translation,
            world_rotation,
            density,
            mass,
            mass_properties,
            friction: friction.max(0.0),
            restitution: restitution.clamp(0.0, 1.0),
            local_aabb,
            world_aabb,
            tri_bvh,
        })
    }

    /// Synchronizes the collider's world-space pose and cached `world_aabb` from the parent
    /// rigid body's world position `body_pos` and orientation `body_rot`.
    pub fn update_world_pose(&mut self, body_pos: Vector3<f64>, body_rot: UnitQuaternion<f64>) {
        self.world_translation = body_pos + body_rot * self.local_translation;
        self.world_rotation = body_rot * self.local_rotation;
        self.world_aabb =
            self.shape
                .compute_world_aabb(self.world_translation, self.world_rotation, body_rot);
    }

    /// Returns the cached world-space axis-aligned bounding box (`BoundingBox`).
    #[inline]
    #[must_use]
    pub fn compute_aabb(&self) -> BoundingBox {
        self.world_aabb
    }
}

/// Deterministic registry storing [`Collider`] instances indexed by [`ColliderHandle`].
#[derive(Debug, Clone, Default)]
pub struct ColliderRegistry {
    colliders: HashMap<ColliderHandle, Collider>,
    order: Vec<ColliderHandle>,
    next_id: usize,
}

impl ColliderRegistry {
    /// Creates an empty [`ColliderRegistry`].
    #[must_use]
    pub fn new() -> Self {
        Self {
            colliders: HashMap::new(),
            order: Vec::new(),
            next_id: 0,
        }
    }

    /// Inserts a [`Collider`] attached to `parent` in `bodies`, synchronizing its initial world
    /// pose and AABB from the parent [`RigidBody`].
    pub fn insert_with_parent(
        &mut self,
        mut collider: Collider,
        parent: BodyHandle,
        bodies: &mut BodyRegistry,
    ) -> ColliderHandle {
        let handle = ColliderHandle(self.next_id);
        self.next_id += 1;
        collider.handle = handle;
        collider.parent = Some(parent);

        if let Some(rb) = bodies.get_mut(parent) {
            collider.update_world_pose(rb.position, rb.orientation);
            if !rb.colliders.contains(&handle) {
                rb.colliders.push(handle);
            }
        }

        self.colliders.insert(handle, collider);
        self.order.push(handle);
        handle
    }

    /// Retrieves a reference to a [`Collider`] by handle.
    #[inline]
    #[must_use]
    pub fn get(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(&handle)
    }

    /// Retrieves a mutable reference to a [`Collider`] by handle.
    #[inline]
    #[must_use]
    pub fn get_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(&handle)
    }

    /// Removes a [`Collider`] by handle and detaches it from its parent in `bodies`.
    pub fn remove(&mut self, handle: ColliderHandle, bodies: &mut BodyRegistry) -> Option<Collider> {
        self.order.retain(|&h| h != handle);
        let removed = self.colliders.remove(&handle)?;
        if let Some(parent_h) = removed.parent {
            if let Some(rb) = bodies.get_mut(parent_h) {
                rb.colliders.retain(|&c| c != handle);
            }
        }
        Some(removed)
    }

    /// Number of colliders in the registry.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.colliders.len()
    }

    /// Returns `true` if the registry contains no colliders.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.colliders.is_empty()
    }

    /// Iterates over `(ColliderHandle, &Collider)` in deterministic insertion order.
    pub fn iter(&self) -> impl Iterator<Item = (ColliderHandle, &Collider)> {
        self.order
            .iter()
            .filter_map(|&h| self.colliders.get(&h).map(|c| (h, c)))
    }
}

/// Constructs an `f64` [`RigidBody`] for a given CAD [`Part`].
#[must_use]
pub fn create_rigid_body(
    part: &Part,
    is_dynamic: bool,
    initial_position: Option<[f64; 3]>,
    initial_rotation: Option<[f64; 4]>,
) -> RigidBody {
    RigidBody::from_part(part, is_dynamic, initial_position, initial_rotation)
}

/// Backwards-compatible alias for [`create_rigid_body`] returning an `f64` [`RigidBody`].
#[must_use]
pub fn create_rigid_body_builder(
    part: &Part,
    is_dynamic: bool,
    initial_position: Option<[f64; 3]>,
    initial_rotation: Option<[f64; 4]>,
) -> RigidBody {
    RigidBody::from_part(part, is_dynamic, initial_position, initial_rotation)
}

/// Constructs an `f64` [`Collider`] for a given CAD [`Part`] and [`ColliderType`].
pub fn create_collider(part: &Part, collider_type: ColliderType) -> Result<Collider, PhysicsError> {
    Collider::from_part(part, collider_type)
}

/// Backwards-compatible alias for [`create_collider`] returning an `f64` [`Collider`].
pub fn create_collider_builder(
    part: &Part,
    collider_type: ColliderType,
) -> Result<Collider, PhysicsError> {
    Collider::from_part(part, collider_type)
}

// ============================================================================
// Pure-Rust f64 3D Convex Hull & Compound Decomposition Helpers
// ============================================================================

fn aabb_from_points(points: &[[f64; 3]]) -> BoundingBox {
    if points.is_empty() {
        return BoundingBox::new([0.0; 3], [0.0; 3]);
    }
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for p in points {
        for i in 0..3 {
            min[i] = min[i].min(p[i]);
            max[i] = max[i].max(p[i]);
        }
    }
    BoundingBox::new(min, max)
}

fn transform_points_aabb(
    points: &[[f64; 3]],
    world_pos: Vector3<f64>,
    r_mat: &Matrix3<f64>,
) -> BoundingBox {
    if points.is_empty() {
        return BoundingBox::new(
            [world_pos.x, world_pos.y, world_pos.z],
            [world_pos.x, world_pos.y, world_pos.z],
        );
    }
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for p in points {
        let v = world_pos + r_mat * Vector3::new(p[0], p[1], p[2]);
        min[0] = min[0].min(v.x);
        min[1] = min[1].min(v.y);
        min[2] = min[2].min(v.z);
        max[0] = max[0].max(v.x);
        max[1] = max[1].max(v.y);
        max[2] = max[2].max(v.z);
    }
    BoundingBox::new(min, max)
}

type ConvexHullResult = (Vec<[f64; 3]>, Vec<[u32; 3]>);

/// Computes a 3D convex hull `(hull_points, hull_faces)` in `f64` from an input point cloud.
/// Returns `None` if fewer than 4 non-coplanar / non-collinear points exist.
fn compute_convex_hull_f64(raw_points: &[[f64; 3]]) -> Option<ConvexHullResult> {
    if raw_points.len() < 4 {
        return None;
    }

    let aabb = aabb_from_points(raw_points);
    let dims = aabb.dimensions();
    let diag = (dims[0] * dims[0] + dims[1] * dims[1] + dims[2] * dims[2]).sqrt();
    if !diag.is_finite() || diag <= 1e-9 {
        return None;
    }
    let tol = (diag * 1e-7).max(1e-11);

    // Deduplicate input vertices within tolerance
    let mut pts: Vec<Vector3<f64>> = Vec::with_capacity(raw_points.len().min(512));
    let inv_cell = 1.0 / tol;
    let mut seen_cells = std::collections::HashSet::with_capacity(raw_points.len());
    for &[x, y, z] in raw_points {
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return None;
        }
        let key = (
            (x * inv_cell).round() as i64,
            (y * inv_cell).round() as i64,
            (z * inv_cell).round() as i64,
        );
        if seen_cells.insert(key) {
            pts.push(Vector3::new(x, y, z));
        }
    }
    if pts.len() < 4 {
        return None;
    }

    // If the point set is very large, retain extreme directional support points + stride sample
    // so Quickhull runs in sub-millisecond time while preserving the exact AABB and convex envelope.
    if pts.len() > 256 {
        let mut selected_indices = std::collections::BTreeSet::new();
        // 26 directions on the unit cube (faces, edges, corners) + 38 Fibonacci sphere directions
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let dir = Vector3::new(dx as f64, dy as f64, dz as f64);
                    if let Some((idx, _)) = pts.iter().enumerate().max_by(|(_, a), (_, b)| {
                        a.dot(&dir)
                            .partial_cmp(&b.dot(&dir))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    }) {
                        selected_indices.insert(idx);
                    }
                }
            }
        }
        let golden_angle = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
        for k in 0..64 {
            let z = 1.0 - (2.0 * (k as f64) + 1.0) / 64.0;
            let r = (1.0 - z * z).max(0.0).sqrt();
            let theta = golden_angle * (k as f64);
            let dir = Vector3::new(r * theta.cos(), r * theta.sin(), z);
            if let Some((idx, _)) = pts.iter().enumerate().max_by(|(_, a), (_, b)| {
                a.dot(&dir)
                    .partial_cmp(&b.dot(&dir))
                    .unwrap_or(std::cmp::Ordering::Equal)
            }) {
                selected_indices.insert(idx);
            }
        }
        let step = (pts.len() / 128).max(1);
        for idx in (0..pts.len()).step_by(step) {
            selected_indices.insert(idx);
        }
        pts = selected_indices.into_iter().map(|i| pts[i]).collect();
    }

    // 1. Find initial non-degenerate simplex (tetrahedron i0, i1, i2, i3)
    let mut i0 = 0usize;
    let mut i1 = 0usize;
    let mut max_span = -1.0_f64;
    for axis in 0..3 {
        let mut min_idx = 0usize;
        let mut max_idx = 0usize;
        for (i, p) in pts.iter().enumerate() {
            if p[axis] < pts[min_idx][axis] {
                min_idx = i;
            }
            if p[axis] > pts[max_idx][axis] {
                max_idx = i;
            }
        }
        let d = (pts[max_idx] - pts[min_idx]).norm();
        if d > max_span {
            max_span = d;
            i0 = min_idx;
            i1 = max_idx;
        }
    }
    if i0 == i1 || max_span <= tol {
        return None;
    }

    let line_dir = (pts[i1] - pts[i0]).normalize();
    let mut i2 = i0;
    let mut max_line_dist = -1.0_f64;
    for (i, p) in pts.iter().enumerate() {
        if i == i0 || i == i1 {
            continue;
        }
        let d = (p - pts[i0]).cross(&line_dir).norm();
        if d > max_line_dist {
            max_line_dist = d;
            i2 = i;
        }
    }
    if i2 == i0 || max_line_dist <= tol {
        return None;
    }

    let plane_normal = (pts[i1] - pts[i0]).cross(&(pts[i2] - pts[i0])).normalize();
    let mut i3 = i0;
    let mut max_plane_dist = -1.0_f64;
    for (i, p) in pts.iter().enumerate() {
        if i == i0 || i == i1 || i == i2 {
            continue;
        }
        let d = (p - pts[i0]).dot(&plane_normal).abs();
        if d > max_plane_dist {
            max_plane_dist = d;
            i3 = i;
        }
    }
    if i3 == i0 || max_plane_dist <= tol {
        return None;
    }

    let interior = (pts[i0] + pts[i1] + pts[i2] + pts[i3]) * 0.25;
    let make_oriented_face = |a: usize, b: usize, c: usize| -> [usize; 3] {
        let n = (pts[b] - pts[a]).cross(&(pts[c] - pts[a]));
        if n.dot(&(pts[a] - interior)) >= 0.0 {
            [a, b, c]
        } else {
            [a, c, b]
        }
    };

    let mut faces: Vec<[usize; 3]> = vec![
        make_oriented_face(i0, i1, i2),
        make_oriented_face(i0, i2, i3),
        make_oriented_face(i0, i3, i1),
        make_oriented_face(i1, i3, i2),
    ];

    let mut used = vec![false; pts.len()];
    used[i0] = true;
    used[i1] = true;
    used[i2] = true;
    used[i3] = true;

    // 2. Incremental 3D Quickhull expansion
    let face_tol = (diag * 1e-6).max(1e-9);
    for _iter in 0..pts.len() {
        let mut best_pt: Option<usize> = None;
        let mut best_dist = face_tol;

        for &[a, b, c] in &faces {
            let n_raw = (pts[b] - pts[a]).cross(&(pts[c] - pts[a]));
            let n_len = n_raw.norm();
            if n_len <= 1e-18 {
                continue;
            }
            let n = n_raw / n_len;
            for (p_idx, p) in pts.iter().enumerate() {
                if used[p_idx] {
                    continue;
                }
                let d = (p - pts[a]).dot(&n);
                if d > best_dist {
                    best_dist = d;
                    best_pt = Some(p_idx);
                }
            }
        }

        let Some(apex) = best_pt else {
            break;
        };
        used[apex] = true;

        // Identify all faces visible from `apex`
        let mut visible = vec![false; faces.len()];
        let mut any_visible = false;
        for (f_idx, &[a, b, c]) in faces.iter().enumerate() {
            let n_raw = (pts[b] - pts[a]).cross(&(pts[c] - pts[a]));
            let n_len = n_raw.norm();
            if n_len <= 1e-18 {
                continue;
            }
            let d = (pts[apex] - pts[a]).dot(&(n_raw / n_len));
            if d > 1e-11 {
                visible[f_idx] = true;
                any_visible = true;
            }
        }
        if !any_visible {
            continue;
        }

        // Collect directed edges of visible faces; horizon edges are those whose reverse edge is not in a visible face
        let mut visible_edges = std::collections::HashSet::new();
        for (f_idx, &[a, b, c]) in faces.iter().enumerate() {
            if visible[f_idx] {
                visible_edges.insert((a, b));
                visible_edges.insert((b, c));
                visible_edges.insert((c, a));
            }
        }

        let mut horizon_edges = Vec::new();
        for &(u, v) in &visible_edges {
            if !visible_edges.contains(&(v, u)) {
                horizon_edges.push((u, v));
            }
        }
        if horizon_edges.len() < 3 {
            continue;
        }

        let mut next_faces: Vec<[usize; 3]> = faces
            .into_iter()
            .enumerate()
            .filter_map(|(idx, f)| if visible[idx] { None } else { Some(f) })
            .collect();

        for (u, v) in horizon_edges {
            next_faces.push(make_oriented_face(u, v, apex));
        }
        faces = next_faces;
    }

    // Compact vertex buffer to only vertices referenced by hull faces + any extreme AABB vertices
    let mut index_map: HashMap<usize, u32> = HashMap::new();
    let mut hull_points: Vec<[f64; 3]> = Vec::new();
    let mut hull_faces: Vec<[u32; 3]> = Vec::with_capacity(faces.len());

    for [a, b, c] in faces {
        let mut tri = [0u32; 3];
        for (slot, old_idx) in [a, b, c].into_iter().enumerate() {
            let new_idx = *index_map.entry(old_idx).or_insert_with(|| {
                let id = hull_points.len() as u32;
                let p = pts[old_idx];
                hull_points.push([p.x, p.y, p.z]);
                id
            });
            tri[slot] = new_idx;
        }
        hull_faces.push(tri);
    }

    if hull_points.len() < 4 || hull_faces.len() < 4 {
        return None;
    }

    Some((hull_points, hull_faces))
}

/// Decomposes a `TriangleMesh` into up to 8 convex hull sub-parts in `f64` by partitioning
/// triangles across the octants of the mesh centroid.
fn compute_compound_convex_parts_f64(mesh: &TriangleMesh) -> Option<Vec<ConvexHullPart>> {
    let raw_points: Vec<[f64; 3]> = mesh.vertices.iter().map(|v| v.position).collect();
    let (overall_points, overall_faces) = compute_convex_hull_f64(&raw_points)?;
    let overall_aabb = aabb_from_points(&overall_points);

    let center = overall_aabb.center();
    let mut octant_points: [Vec<[f64; 3]>; 8] = Default::default();

    for tri in &mesh.indices {
        let Some(v0) = mesh.vertices.get(tri[0] as usize) else {
            continue;
        };
        let Some(v1) = mesh.vertices.get(tri[1] as usize) else {
            continue;
        };
        let Some(v2) = mesh.vertices.get(tri[2] as usize) else {
            continue;
        };
        let cx = (v0.position[0] + v1.position[0] + v2.position[0]) / 3.0;
        let cy = (v0.position[1] + v1.position[1] + v2.position[1]) / 3.0;
        let cz = (v0.position[2] + v1.position[2] + v2.position[2]) / 3.0;
        let oct = usize::from(cx >= center[0])
            | (usize::from(cy >= center[1]) << 1)
            | (usize::from(cz >= center[2]) << 2);
        octant_points[oct].push(v0.position);
        octant_points[oct].push(v1.position);
        octant_points[oct].push(v2.position);
    }

    let mut parts = Vec::new();
    for pts in &octant_points {
        if pts.len() >= 4 {
            if let Some((hp, hf)) = compute_convex_hull_f64(pts) {
                let local_aabb = aabb_from_points(&hp);
                parts.push(ConvexHullPart {
                    points: hp,
                    faces: hf,
                    local_aabb,
                });
            }
        }
    }

    if parts.is_empty() {
        parts.push(ConvexHullPart {
            points: overall_points,
            faces: overall_faces,
            local_aabb: overall_aabb,
        });
    }

    Some(parts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mcp_cad_core::dag::FeatureOp;
    use mcp_cad_core::material::MaterialRegistry;

    #[test]
    fn test_p4_t2_exact_box_mass_com_and_inertia_tensor_f64() {
        let registry = MaterialRegistry::new();
        let steel = registry.get("steel_structural_aisi1020").unwrap().clone();
        let rho = steel.density_kg_m3;

        let w = 0.20_f64;
        let h = 0.10_f64;
        let d = 0.05_f64;
        let mut part = Part::new("box_exact", "Exact Box", steel);
        part.dag.add_feature(
            FeatureOp::AddBox {
                width: w,
                height: h,
                depth: d,
                position: Some([0.15, -0.05, 0.08]),
                rotation_deg: None,
            },
            None,
        );
        part.rebuild().unwrap();

        let mprops = MassProperties::from_part(&part);
        let expected_vol = w * h * d;
        let expected_mass = expected_vol * rho;
        let expected_ixx = (expected_mass / 12.0) * (h * h + d * d);
        let expected_iyy = (expected_mass / 12.0) * (w * w + d * d);
        let expected_izz = (expected_mass / 12.0) * (w * w + h * h);

        assert!((mprops.volume - expected_vol).abs() < 1e-14);
        assert!((mprops.mass - expected_mass).abs() < 1e-11);
        assert!((mprops.local_com.x - 0.15).abs() < 1e-14);
        assert!((mprops.local_com.y - (-0.05)).abs() < 1e-14);
        assert!((mprops.local_com.z - 0.08).abs() < 1e-14);

        assert!(
            (mprops.inertia_tensor[(0, 0)] - expected_ixx).abs() < 1e-12,
            "I_xx = {}, expected {}",
            mprops.inertia_tensor[(0, 0)],
            expected_ixx
        );
        assert!(
            (mprops.inertia_tensor[(1, 1)] - expected_iyy).abs() < 1e-12,
            "I_yy = {}, expected {}",
            mprops.inertia_tensor[(1, 1)],
            expected_iyy
        );
        assert!(
            (mprops.inertia_tensor[(2, 2)] - expected_izz).abs() < 1e-12,
            "I_zz = {}, expected {}",
            mprops.inertia_tensor[(2, 2)],
            expected_izz
        );
        assert!(mprops.inertia_tensor[(0, 1)].abs() < 1e-14);
        assert!(mprops.inertia_tensor[(0, 2)].abs() < 1e-14);
        assert!(mprops.inertia_tensor[(1, 2)].abs() < 1e-14);
    }

    #[test]
    fn test_p4_t2_hollow_bushing_inertia_accounts_for_internal_bore() {
        let registry = MaterialRegistry::new();
        let steel = registry.get("steel_structural_aisi1020").unwrap().clone();

        let r_outer = 0.05_f64;
        let r_inner = 0.03_f64;
        let height = 0.10_f64;

        let mut solid_cyl = Part::new("solid_cyl", "Solid Cylinder", steel.clone());
        solid_cyl.dag.add_feature(
            FeatureOp::AddCylinder {
                radius: r_outer,
                height,
                segments: 64,
                position: None,
                rotation_deg: None,
            },
            None,
        );
        solid_cyl.rebuild().unwrap();

        let mut hollow_bushing = solid_cyl.clone();
        hollow_bushing.id = "hollow_bushing".to_string();
        hollow_bushing.dag.add_feature(
            FeatureOp::CutHole {
                center: [0.0, 0.0, 0.0],
                normal: [0.0, 0.0, 1.0],
                radius: r_inner,
                depth: height * 1.2,
                keyway_width: None,
                keyway_depth: None,
            },
            None,
        );
        hollow_bushing.rebuild().unwrap();

        let rb_solid = RigidBody::from_part(&solid_cyl, true, None, None);
        let rb_hollow = RigidBody::from_part(&hollow_bushing, true, None, None);

        // Analytical hollow cylinder:
        // m_hollow / m_solid = (R^2 - r^2) / R^2 = (0.0025 - 0.0009) / 0.0025 = 0.64
        let mass_ratio = rb_hollow.mass() / rb_solid.mass();
        assert!(
            (mass_ratio - 0.64).abs() < 0.005,
            "Hollow mass ratio = {mass_ratio}, expected ~0.64"
        );

        // Specific axial inertia I_zz / m:
        // Solid cylinder:  I_zz / m = 0.5 * R^2 = 0.00125 m^2
        // Hollow cylinder: I_zz / m = 0.5 * (R^2 + r^2) = 0.5 * (0.0025 + 0.0009) = 0.00170 m^2
        // (Previously, ConvexHull ignored the bore and gave 0.5 * R^2 = 0.00125 m^2!)
        let spec_izz_solid = rb_solid.local_inertia()[(2, 2)] / rb_solid.mass();
        let spec_izz_hollow = rb_hollow.local_inertia()[(2, 2)] / rb_hollow.mass();
        let expected_spec_hollow = 0.5 * (r_outer * r_outer + r_inner * r_inner);

        assert!(
            (spec_izz_hollow - expected_spec_hollow).abs() / expected_spec_hollow < 0.01,
            "Hollow specific I_zz/m = {spec_izz_hollow:.6}, expected {expected_spec_hollow:.6} (solid = {spec_izz_solid:.6})"
        );
        assert!(
            spec_izz_hollow > spec_izz_solid * 1.30,
            "Hollow specific inertia must exceed solid convex-hull inertia"
        );
    }

    #[test]
    fn test_p4_t2_all_collider_shapes_and_registries_f64() {
        let registry = MaterialRegistry::new();
        let aluminum = registry.get("aluminum_6061_t6").unwrap().clone();
        let mut part = Part::new("col_test", "Collider Part", aluminum);
        part.dag.add_feature(
            FeatureOp::AddCylinder {
                radius: 0.04,
                height: 0.12,
                segments: 24,
                position: None,
                rotation_deg: None,
            },
            None,
        );
        part.rebuild().unwrap();

        let mut bodies = BodyRegistry::new();
        let mut colliders = ColliderRegistry::new();

        let rb = RigidBody::from_part(&part, true, Some([1.0, 2.0, 3.0]), None);
        let bh = bodies.insert(rb);
        assert_eq!(bh, BodyHandle(0));

        for ctype in [
            ColliderType::ConvexHull,
            ColliderType::TriMesh,
            ColliderType::Compound,
            ColliderType::Cube,
            ColliderType::Sphere,
            ColliderType::Cylinder,
        ] {
            let col = Collider::from_part(&part, ctype).expect("collider build should succeed");
            let ch = colliders.insert_with_parent(col, bh, &mut bodies);
            let stored = colliders.get(ch).unwrap();
            let aabb = stored.compute_aabb();
            assert!((aabb.center()[0] - 1.0).abs() < 1e-3);
            assert!((aabb.center()[1] - 2.0).abs() < 1e-3);
            assert!((aabb.center()[2] - 3.0).abs() < 1e-3);
        }
        assert_eq!(colliders.len(), 6);
    }
}
