//! Linear 4-node (TET4) and quadratic 10-node (TET10) tetrahedral finite element formulations.

use nalgebra::{Matrix3, Matrix4, Matrix6, SMatrix, SVector, Vector3};
use crate::elasticity::{radial_return_j2, GaussPointState, J2MaterialParams, ReturnMappingResult};
use crate::error::FeaError;

/// Type alias for the 6x12 strain-displacement matrix B (TET4).
pub type Matrix6x12<T> = SMatrix<T, 6, 12>;

/// Type alias for the 12x12 element stiffness matrix Ke (TET4).
pub type Matrix12<T> = SMatrix<T, 12, 12>;

/// Type alias for the 12-element displacement vector of a linear tetrahedron (TET4).
pub type Vector12<T> = SVector<T, 12>;

/// Type alias for the 6x30 strain-displacement matrix B(L) (TET10).
pub type Matrix6x30<T> = SMatrix<T, 6, 30>;

/// Type alias for the 30x30 element stiffness matrix Ke (TET10).
pub type Matrix30<T> = SMatrix<T, 30, 30>;

/// Type alias for the 30-element displacement vector of a quadratic tetrahedron (TET10).
pub type Vector30<T> = SVector<T, 30>;

/// Type alias for the 6-component stress or strain vector in Voigt notation.
pub type Vector6<T> = SVector<T, 6>;

/// Canonical edge pairs `(corner_a, corner_b)` for the 6 mid-edge nodes `4..10` (1-indexed `5..10`) of `TET10`:
/// `5=(1,2), 6=(2,3), 7=(3,1), 8=(1,4), 9=(2,4), 10=(3,4)` -> 0-indexed `[(0,1), (1,2), (2,0), (0,3), (1,3), (2,3)]`.
pub const TET10_EDGE_PAIRS: [(usize, usize); 6] = [
    (0, 1), // node 4 (1-indexed 5): edge (0, 1)
    (1, 2), // node 5 (1-indexed 6): edge (1, 2)
    (2, 0), // node 6 (1-indexed 7): edge (2, 0)
    (0, 3), // node 7 (1-indexed 8): edge (0, 3)
    (1, 3), // node 8 (1-indexed 9): edge (1, 3)
    (2, 3), // node 9 (1-indexed 10): edge (2, 3)
];

/// Dominant barycentric coordinate $a = \frac{5 + 3\sqrt{5}}{20}$ for 4-point degree-2 tetrahedral Gauss quadrature.
pub const TET10_GAUSS_A: f64 = 0.5854101966249685;

/// Minor barycentric coordinate $b = \frac{5 - \sqrt{5}}{20}$ for 4-point degree-2 tetrahedral Gauss quadrature.
pub const TET10_GAUSS_B: f64 = 0.1381966011250105;

/// Reference integration weight $w_q = \frac{1}{24}$ per Gauss point on the unit reference tetrahedron ($V_{\text{ref}} = 1/6$).
pub const TET10_GAUSS_WEIGHT_REF: f64 = 1.0 / 24.0;

/// Barycentric coordinates `[L0, L1, L2, L3]` of the 4 Gauss integration points.
pub const TET10_GAUSS_POINTS: [[f64; 4]; 4] = [
    [TET10_GAUSS_A, TET10_GAUSS_B, TET10_GAUSS_B, TET10_GAUSS_B],
    [TET10_GAUSS_B, TET10_GAUSS_A, TET10_GAUSS_B, TET10_GAUSS_B],
    [TET10_GAUSS_B, TET10_GAUSS_B, TET10_GAUSS_A, TET10_GAUSS_B],
    [TET10_GAUSS_B, TET10_GAUSS_B, TET10_GAUSS_B, TET10_GAUSS_A],
];

/// Evaluates the 10 quadratic shape functions $N_0..N_9$ at barycentric coordinates `l = [L0, L1, L2, L3]`:
/// - Corner nodes $i \in 0..4$: $N_i = L_i(2L_i - 1)$
/// - Mid-edge nodes $4+e$ ($e \in 0..6$): $N_{4+e} = 4 L_a L_b$
#[inline]
pub fn tet10_shape_functions(l: [f64; 4]) -> [f64; 10] {
    [
        l[0] * (2.0 * l[0] - 1.0),
        l[1] * (2.0 * l[1] - 1.0),
        l[2] * (2.0 * l[2] - 1.0),
        l[3] * (2.0 * l[3] - 1.0),
        4.0 * l[0] * l[1],
        4.0 * l[1] * l[2],
        4.0 * l[2] * l[0],
        4.0 * l[0] * l[3],
        4.0 * l[1] * l[3],
        4.0 * l[2] * l[3],
    ]
}

/// Evaluates the derivatives $\frac{\partial N_k}{\partial L_m}$ ($k \in 0..10, m \in 0..4$) with respect to the 4 barycentric coordinates.
#[inline]
pub fn tet10_barycentric_derivatives(l: [f64; 4]) -> [[f64; 4]; 10] {
    let mut dn_dl = [[0.0f64; 4]; 10];
    for i in 0..4 {
        dn_dl[i][i] = 4.0 * l[i] - 1.0;
    }
    for (e, &(a, b)) in TET10_EDGE_PAIRS.iter().enumerate() {
        dn_dl[4 + e][a] = 4.0 * l[b];
        dn_dl[4 + e][b] = 4.0 * l[a];
    }
    dn_dl
}

/// Linear 4-node tetrahedral element (TET4) providing volume, strain matrix B,
/// element stiffness matrix Ke, and stress recovery.
#[derive(Debug, Clone)]
pub struct Tet4Element {
    /// Element identifier
    pub id: usize,
    /// 4 node indices in the global mesh
    pub node_indices: [usize; 4],
    /// Volume of the tetrahedron $V_e$ in m³
    pub volume: f64,
    /// 6x12 strain-displacement matrix B
    pub b_matrix: Matrix6x12<f64>,
}

impl Tet4Element {
    /// Constructs a TET4 element from element ID, 4 node indices, and their 3D coordinates.
    pub fn new(
        id: usize,
        node_indices: [usize; 4],
        positions: [[f64; 3]; 4],
    ) -> Result<Self, FeaError> {
        let (volume, b_matrix) = compute_volume_and_b_matrix(id, positions)?;
        Ok(Self {
            id,
            node_indices,
            volume,
            b_matrix,
        })
    }

    /// Computes the 12x12 element stiffness matrix $K_e = V_e \cdot B^T D B$.
    #[inline]
    pub fn stiffness_matrix(&self, d_matrix: &Matrix6<f64>) -> Matrix12<f64> {
        let b = &self.b_matrix;
        let bt_d = b.transpose() * d_matrix;
        let ke = bt_d * b;
        ke * self.volume
    }

    /// Computes element strain vector $\epsilon = B \cdot u_e$ ($6 \times 1$).
    #[inline]
    pub fn compute_strain(&self, u_elem: &Vector12<f64>) -> Vector6<f64> {
        self.b_matrix * u_elem
    }

    /// Computes element Cauchy stress vector $\sigma = D \cdot B \cdot u_e$ ($6 \times 1$).
    #[inline]
    pub fn compute_stress(&self, d_matrix: &Matrix6<f64>, u_elem: &Vector12<f64>) -> Vector6<f64> {
        let strain = self.compute_strain(u_elem);
        d_matrix * strain
    }

    /// Computes equivalent von Mises stress from stress vector:
    ///
    /// $$\sigma_{vM} = \sqrt{\frac{1}{2}\left[(\sigma_x - \sigma_y)^2 + (\sigma_y - \sigma_z)^2 + (\sigma_z - \sigma_x)^2 + 6(\tau_{xy}^2 + \tau_{yz}^2 + \tau_{zx}^2)\right]}$$
    #[inline]
    pub fn von_mises_stress(stress: &Vector6<f64>) -> f64 {
        let sx = stress[0];
        let sy = stress[1];
        let sz = stress[2];
        let txy = stress[3];
        let tyz = stress[4];
        let tzx = stress[5];

        let s1 = (sx - sy).powi(2);
        let s2 = (sy - sz).powi(2);
        let s3 = (sz - sx).powi(2);
        let shear = 6.0 * (txy.powi(2) + tyz.powi(2) + tzx.powi(2));

        (0.5 * (s1 + s2 + s3 + shear)).sqrt()
    }

    /// Returns the 4 physical shape function gradients $\nabla_X N_i = [\partial N_i/\partial x, \partial N_i/\partial y, \partial N_i/\partial z]$
    /// extracted directly from the precomputed $B$-matrix.
    #[inline]
    pub fn shape_gradients(&self) -> [[f64; 3]; 4] {
        let mut grads = [[0.0; 3]; 4];
        for (i, grad) in grads.iter_mut().enumerate() {
            let col = 3 * i;
            grad[0] = self.b_matrix[(0, col)];
            grad[1] = self.b_matrix[(1, col + 1)];
            grad[2] = self.b_matrix[(2, col + 2)];
        }
        grads
    }

    /// Computes the $3 \times 3$ displacement gradient $\mathbf{H} = \nabla_X \mathbf{u} = \sum_{i=0}^3 \mathbf{u}_i \otimes \nabla_X N_i$.
    #[inline]
    pub fn displacement_gradient(&self, u_elem: &Vector12<f64>) -> Matrix3<f64> {
        let grads = self.shape_gradients();
        let mut h = Matrix3::<f64>::zeros();
        for (i, &[gx, gy, gz]) in grads.iter().enumerate() {
            let ux = u_elem[3 * i];
            let uy = u_elem[3 * i + 1];
            let uz = u_elem[3 * i + 2];
            h[(0, 0)] += ux * gx;
            h[(0, 1)] += ux * gy;
            h[(0, 2)] += ux * gz;
            h[(1, 0)] += uy * gx;
            h[(1, 1)] += uy * gy;
            h[(1, 2)] += uy * gz;
            h[(2, 0)] += uz * gx;
            h[(2, 1)] += uz * gy;
            h[(2, 2)] += uz * gz;
        }
        h
    }

    /// Evaluates nonlinear internal forces $\mathbf{f}_{\text{int}}$ ($12 \times 1$),
    /// symmetric tangent stiffness $\mathbf{K}_e$ ($12 \times 12$), updated $J_2$ Gauss-point state,
    /// and element rotation $\mathbf{R} \in SO(3)$ using Co-Rotated FEM (when `corotated = true`)
    /// and Radial Return Mapping.
    pub fn evaluate_nonlinear(
        &self,
        u_elem: &Vector12<f64>,
        prev_state: &GaussPointState,
        params: &J2MaterialParams,
        corotated: bool,
        include_geometric_stiffness: bool,
    ) -> (Vector12<f64>, Matrix12<f64>, ReturnMappingResult, Matrix3<f64>) {
        let grads = self.shape_gradients();
        let h_grad = self.displacement_gradient(u_elem);
        let (r_mat, u_stretch, eps_loc) = if corotated {
            corotated_biot_strain_voigt(&h_grad)
        } else {
            (
                Matrix3::identity(),
                Matrix3::identity(),
                linear_strain_from_grad_u(&h_grad),
            )
        };

        let rm = radial_return_j2(&eps_loc, prev_state, params);
        let sigma_mat = voigt_stress_to_matrix3(&rm.stress);
        let pk1 = r_mat * sigma_mat;

        let mut f_int = Vector12::<f64>::zeros();
        for (k, &g) in grads.iter().enumerate() {
            let fk = pk1 * Vector3::new(g[0], g[1], g[2]) * self.volume;
            f_int[3 * k] = fk.x;
            f_int[3 * k + 1] = fk.y;
            f_int[3 * k + 2] = fk.z;
        }

        let mut ke = Matrix12::<f64>::zeros();
        let r_t = r_mat.transpose();
        let mut b_rot = [SMatrix::<f64, 6, 3>::zeros(); 4];
        let mut cb_rot = [SMatrix::<f64, 6, 3>::zeros(); 4];
        for (k, &g) in grads.iter().enumerate() {
            let br = node_b_matrix(g) * r_t;
            cb_rot[k] = rm.tangent * br;
            b_rot[k] = br;
        }

        let use_geo = corotated && include_geometric_stiffness && rm.stress.norm() > 1e-12;
        let su_inv = if use_geo {
            let su = Matrix3::identity() * u_stretch.trace() - u_stretch;
            su.try_inverse()
        } else {
            None
        };

        let mut geo_a = [Matrix3::<f64>::zeros(); 4];
        let mut geo_m = [Matrix3::<f64>::zeros(); 4];
        if let Some(su_inv_mat) = su_inv {
            for (k, &g) in grads.iter().enumerate() {
                let gv = Vector3::new(g[0], g[1], g[2]);
                let tk = sigma_mat * gv;
                geo_a[k] = r_mat * skew_matrix(&tk);
                geo_m[k] = su_inv_mat * skew_matrix(&gv) * r_t;
            }
        }

        for a in 0..4 {
            let row = 3 * a;
            for b in a..4 {
                let col = 3 * b;
                let mut block = (b_rot[a].transpose() * cb_rot[b]) * self.volume;
                if su_inv.is_some() {
                    let g_ab = -(geo_a[a] * geo_m[b]) * self.volume;
                    let g_ba = -(geo_a[b] * geo_m[a]) * self.volume;
                    block += (g_ab + g_ba.transpose()) * 0.5;
                }

                for r in 0..3 {
                    for c in 0..3 {
                        ke[(row + r, col + c)] = block[(r, c)];
                    }
                }
                if b > a {
                    for r in 0..3 {
                        for c in 0..3 {
                            ke[(col + c, row + r)] = block[(r, c)];
                        }
                    }
                }
            }
        }

        (f_int, ke, rm, r_mat)
    }

    /// Evaluates nonlinear internal forces $\mathbf{f}_{\text{int}}$ ($12 \times 1$) and $J_2$
    /// Gauss-point state without assembling the tangent stiffness (fast path for line search).
    pub fn evaluate_nonlinear_forces_only(
        &self,
        u_elem: &Vector12<f64>,
        prev_state: &GaussPointState,
        params: &J2MaterialParams,
        corotated: bool,
    ) -> (Vector12<f64>, ReturnMappingResult, Matrix3<f64>) {
        let grads = self.shape_gradients();
        let h_grad = self.displacement_gradient(u_elem);
        let (r_mat, _u_stretch, eps_loc) = if corotated {
            corotated_biot_strain_voigt(&h_grad)
        } else {
            (
                Matrix3::identity(),
                Matrix3::identity(),
                linear_strain_from_grad_u(&h_grad),
            )
        };

        let rm = radial_return_j2(&eps_loc, prev_state, params);
        let pk1 = r_mat * voigt_stress_to_matrix3(&rm.stress);

        let mut f_int = Vector12::<f64>::zeros();
        for (k, &g) in grads.iter().enumerate() {
            let fk = pk1 * Vector3::new(g[0], g[1], g[2]) * self.volume;
            f_int[3 * k] = fk.x;
            f_int[3 * k + 1] = fk.y;
            f_int[3 * k + 2] = fk.z;
        }
        (f_int, rm, r_mat)
    }
}

/// Quadratic 10-node isoparametric tetrahedral finite element (`TET10`).
///
/// Eliminates linear shear locking in bending and captures linear strain/stress gradients
/// within each element using 4-point Gauss quadrature.
#[derive(Debug, Clone)]
pub struct Tet10Element {
    /// Element identifier
    pub id: usize,
    /// 10 node indices in the global mesh (`0..4` corners, `4..10` mid-edge nodes)
    pub node_indices: [usize; 10],
    /// Physical 3D coordinates `[x, y, z]` of the 10 nodes in meters
    pub positions: [[f64; 3]; 10],
    /// Total element volume $V_e = \sum_{q=0}^3 w_q$ in m³
    pub volume: f64,
    /// Physical integration weights $w_q = \frac{1}{24} \det J(\mathbf{L}^{(q)})$ at the 4 Gauss points
    pub gauss_weights: [f64; 4],
    /// Physical Cartesian gradients $[\partial N_k/\partial x, \partial N_k/\partial y, \partial N_k/\partial z]$
    /// for each of the 10 nodes at each of the 4 Gauss points
    pub gauss_gradients: [[[f64; 3]; 10]; 4],
}

impl Tet10Element {
    /// Constructs a `Tet10Element` from element ID, 10 global node indices, and their 3D coordinates.
    /// Validates that the corner volume and all 4 Gauss-point Jacobians are strictly positive.
    pub fn new(
        id: usize,
        node_indices: [usize; 10],
        positions: [[f64; 3]; 10],
    ) -> Result<Self, FeaError> {
        // Verify corner tetrahedron orientation/volume first
        let c = Matrix4::new(
            1.0, positions[0][0], positions[0][1], positions[0][2],
            1.0, positions[1][0], positions[1][1], positions[1][2],
            1.0, positions[2][0], positions[2][1], positions[2][2],
            1.0, positions[3][0], positions[3][1], positions[3][2],
        );
        let corner_vol = c.determinant() / 6.0;
        if corner_vol <= 1e-15 {
            return Err(FeaError::DegenerateElement {
                id,
                volume: corner_vol,
            });
        }

        let mut gauss_weights = [0.0f64; 4];
        let mut gauss_gradients = [[[0.0f64; 3]; 10]; 4];
        let mut volume = 0.0;

        for (q, &l) in TET10_GAUSS_POINTS.iter().enumerate() {
            let (det_j, grads) = compute_tet10_jacobian_and_gradients(id, &positions, l)?;
            let w_q = TET10_GAUSS_WEIGHT_REF * det_j;
            gauss_weights[q] = w_q;
            gauss_gradients[q] = grads;
            volume += w_q;
        }

        Ok(Self {
            id,
            node_indices,
            positions,
            volume,
            gauss_weights,
            gauss_gradients,
        })
    }

    /// Evaluates the physical Cartesian shape function gradients $\nabla N_k(\mathbf{L})$ ($k \in 0..10$)
    /// at arbitrary barycentric coordinates `l = [L0, L1, L2, L3]`.
    pub fn shape_gradients_at(&self, l: [f64; 4]) -> Result<[[f64; 3]; 10], FeaError> {
        let (_, grads) = compute_tet10_jacobian_and_gradients(self.id, &self.positions, l)?;
        Ok(grads)
    }

    /// Evaluates the $6 \times 30$ strain-displacement matrix $B(\mathbf{L})$ at arbitrary barycentric coordinates `l`.
    pub fn b_matrix_at(&self, l: [f64; 4]) -> Result<Matrix6x30<f64>, FeaError> {
        let grads = self.shape_gradients_at(l)?;
        Ok(build_b_matrix_from_gradients(&grads))
    }

    /// Returns the $6 \times 30$ strain-displacement matrix $B(\mathbf{L}^{(q)})$ at Gauss point `gp` ($0..4$).
    #[inline]
    pub fn b_matrix_at_gauss_point(&self, gp: usize) -> Matrix6x30<f64> {
        build_b_matrix_from_gradients(&self.gauss_gradients[gp])
    }

    /// Computes the $30 \times 30$ element stiffness matrix via 4-point Gauss quadrature:
    ///
    /// $$K_e = \sum_{q=0}^{3} w_q \, B(\mathbf{L}^{(q)})^T D \, B(\mathbf{L}^{(q)})$$
    pub fn stiffness_matrix(&self, d_matrix: &Matrix6<f64>) -> Matrix30<f64> {
        let d00 = d_matrix[(0, 0)];
        let d01 = d_matrix[(0, 1)];
        let d02 = d_matrix[(0, 2)];
        let d11 = d_matrix[(1, 1)];
        let d12 = d_matrix[(1, 2)];
        let d22 = d_matrix[(2, 2)];
        let d33 = d_matrix[(3, 3)];
        let d44 = d_matrix[(4, 4)];
        let d55 = d_matrix[(5, 5)];

        let mut ke = Matrix30::<f64>::zeros();

        for a in 0..10 {
            let row = 3 * a;
            for b in a..10 {
                let col = 3 * b;

                // Accumulate 3x3 gradient inner products across the 4 Gauss points:
                // I_rs = sum_q w_q * g_a[r] * g_b[s]
                let mut i00 = 0.0;
                let mut i01 = 0.0;
                let mut i02 = 0.0;
                let mut i10 = 0.0;
                let mut i11 = 0.0;
                let mut i12 = 0.0;
                let mut i20 = 0.0;
                let mut i21 = 0.0;
                let mut i22 = 0.0;

                for q in 0..4 {
                    let w = self.gauss_weights[q];
                    let ga = self.gauss_gradients[q][a];
                    let gb = self.gauss_gradients[q][b];
                    let w_ga0 = w * ga[0];
                    let w_ga1 = w * ga[1];
                    let w_ga2 = w * ga[2];

                    i00 += w_ga0 * gb[0];
                    i01 += w_ga0 * gb[1];
                    i02 += w_ga0 * gb[2];

                    i10 += w_ga1 * gb[0];
                    i11 += w_ga1 * gb[1];
                    i12 += w_ga1 * gb[2];

                    i20 += w_ga2 * gb[0];
                    i21 += w_ga2 * gb[1];
                    i22 += w_ga2 * gb[2];
                }

                let k00 = d00 * i00 + d33 * i11 + d55 * i22;
                let k01 = d01 * i01 + d33 * i10;
                let k02 = d02 * i02 + d55 * i20;

                let k10 = d01 * i10 + d33 * i01;
                let k11 = d11 * i11 + d33 * i00 + d44 * i22;
                let k12 = d12 * i12 + d44 * i21;

                let k20 = d02 * i20 + d55 * i02;
                let k21 = d12 * i21 + d44 * i12;
                let k22 = d22 * i22 + d44 * i11 + d55 * i00;

                ke[(row, col)] = k00;
                ke[(row, col + 1)] = k01;
                ke[(row, col + 2)] = k02;
                ke[(row + 1, col)] = k10;
                ke[(row + 1, col + 1)] = k11;
                ke[(row + 1, col + 2)] = k12;
                ke[(row + 2, col)] = k20;
                ke[(row + 2, col + 1)] = k21;
                ke[(row + 2, col + 2)] = k22;

                if b > a {
                    ke[(col, row)] = k00;
                    ke[(col + 1, row)] = k01;
                    ke[(col + 2, row)] = k02;
                    ke[(col, row + 1)] = k10;
                    ke[(col + 1, row + 1)] = k11;
                    ke[(col + 2, row + 1)] = k12;
                    ke[(col, row + 2)] = k20;
                    ke[(col + 1, row + 2)] = k21;
                    ke[(col + 2, row + 2)] = k22;
                }
            }
        }

        ke
    }

    /// Computes the $6 \times 1$ engineering strain vector $\boldsymbol{\epsilon}(\mathbf{L})$ at arbitrary barycentric coordinates `l`.
    pub fn compute_strain_at(&self, l: [f64; 4], u_elem: &Vector30<f64>) -> Result<Vector6<f64>, FeaError> {
        let grads = self.shape_gradients_at(l)?;
        Ok(compute_strain_from_gradients(&grads, u_elem))
    }

    /// Computes the $6 \times 1$ engineering strain vectors at each of the 4 Gauss integration points.
    #[inline]
    pub fn compute_gauss_strains(&self, u_elem: &Vector30<f64>) -> [Vector6<f64>; 4] {
        [
            compute_strain_from_gradients(&self.gauss_gradients[0], u_elem),
            compute_strain_from_gradients(&self.gauss_gradients[1], u_elem),
            compute_strain_from_gradients(&self.gauss_gradients[2], u_elem),
            compute_strain_from_gradients(&self.gauss_gradients[3], u_elem),
        ]
    }

    /// Computes the $6 \times 1$ Cauchy stress vectors $\boldsymbol{\sigma}^{(q)} = D \boldsymbol{\epsilon}^{(q)}$
    /// at each of the 4 Gauss integration points.
    #[inline]
    pub fn compute_gauss_stresses(
        &self,
        d_matrix: &Matrix6<f64>,
        u_elem: &Vector30<f64>,
    ) -> [Vector6<f64>; 4] {
        let strains = self.compute_gauss_strains(u_elem);
        [
            d_matrix * strains[0],
            d_matrix * strains[1],
            d_matrix * strains[2],
            d_matrix * strains[3],
        ]
    }

    /// Computes the volume-averaged centroid Cauchy stress tensor of the element.
    #[inline]
    pub fn compute_centroid_stress(
        &self,
        d_matrix: &Matrix6<f64>,
        u_elem: &Vector30<f64>,
    ) -> Vector6<f64> {
        let gp_stresses = self.compute_gauss_stresses(d_matrix, u_elem);
        let mut avg = Vector6::zeros();
        for (s, &w) in gp_stresses.iter().zip(self.gauss_weights.iter()) {
            avg += *s * w;
        }
        if self.volume > 1e-25 {
            avg / self.volume
        } else {
            (gp_stresses[0] + gp_stresses[1] + gp_stresses[2] + gp_stresses[3]) * 0.25
        }
    }

    /// Extrapolates Cauchy stress tensors from the 4 Gauss points to all 10 element nodes
    /// using the exact linear barycentric inverse mapping:
    /// - Corner nodes $i \in 0..4$: $\boldsymbol{\sigma}_i = \alpha \boldsymbol{\sigma}^{(i)} + \beta \sum_{j \ne i} \boldsymbol{\sigma}^{(j)}$
    ///   where $\alpha = \frac{1 + 3\sqrt{5}}{4}$ and $\beta = \frac{1 - \sqrt{5}}{4}$ ($\alpha + 3\beta = 1$).
    /// - Mid-edge nodes $4+e$ ($e \in 0..6$): $\boldsymbol{\sigma}_{4+e} = \frac{1}{2}(\boldsymbol{\sigma}_a + \boldsymbol{\sigma}_b)$.
    #[inline]
    pub fn extrapolate_nodal_stresses(
        &self,
        d_matrix: &Matrix6<f64>,
        u_elem: &Vector30<f64>,
    ) -> [Vector6<f64>; 10] {
        const ALPHA: f64 = 1.9270509831248424; // (1 + 3*sqrt(5)) / 4
        const BETA: f64 = -0.30901699437494745; // (1 - sqrt(5)) / 4

        let gp = self.compute_gauss_stresses(d_matrix, u_elem);
        let gp_sum = gp[0] + gp[1] + gp[2] + gp[3];

        let mut nodal = [Vector6::zeros(); 10];
        for i in 0..4 {
            nodal[i] = gp[i] * (ALPHA - BETA) + gp_sum * BETA;
        }
        for (e, &(a, b)) in TET10_EDGE_PAIRS.iter().enumerate() {
            nodal[4 + e] = (nodal[a] + nodal[b]) * 0.5;
        }
        nodal
    }

    /// Computes the $30 \times 1$ element internal nodal force vector
    /// $\mathbf{f}_{\text{int}} = K_e \mathbf{u}_e = \sum_{q=0}^3 w_q B(\mathbf{L}^{(q)})^T \boldsymbol{\sigma}^{(q)}$.
    #[inline]
    pub fn compute_internal_forces(
        &self,
        d_matrix: &Matrix6<f64>,
        u_elem: &Vector30<f64>,
    ) -> Vector30<f64> {
        let gp_stresses = self.compute_gauss_stresses(d_matrix, u_elem);
        let mut f_int = Vector30::zeros();

        for (q, s) in gp_stresses.iter().enumerate() {
            let w = self.gauss_weights[q];
            let sx = s[0];
            let sy = s[1];
            let sz = s[2];
            let txy = s[3];
            let tyz = s[4];
            let tzx = s[5];

            let grads = &self.gauss_gradients[q];
            for (k, grad_k) in grads.iter().enumerate() {
                let gx = grad_k[0];
                let gy = grad_k[1];
                let gz = grad_k[2];
                let base = 3 * k;
                f_int[base] += w * (gx * sx + gy * txy + gz * tzx);
                f_int[base + 1] += w * (gy * sy + gx * txy + gz * tyz);
                f_int[base + 2] += w * (gz * sz + gy * tyz + gx * tzx);
            }
        }

        f_int
    }

    /// Computes the $3 \times 3$ displacement gradient $\mathbf{H}^{(q)} = \nabla_X \mathbf{u}(\mathbf{L}^{(q)})$
    /// at Gauss point `gp` ($0..4$).
    #[inline]
    pub fn displacement_gradient_at_gp(&self, gp: usize, u_elem: &Vector30<f64>) -> Matrix3<f64> {
        let grads = &self.gauss_gradients[gp];
        let mut h = Matrix3::<f64>::zeros();
        for (k, &[gx, gy, gz]) in grads.iter().enumerate() {
            let ux = u_elem[3 * k];
            let uy = u_elem[3 * k + 1];
            let uz = u_elem[3 * k + 2];
            h[(0, 0)] += ux * gx;
            h[(0, 1)] += ux * gy;
            h[(0, 2)] += ux * gz;
            h[(1, 0)] += uy * gx;
            h[(1, 1)] += uy * gy;
            h[(1, 2)] += uy * gz;
            h[(2, 0)] += uz * gx;
            h[(2, 1)] += uz * gy;
            h[(2, 2)] += uz * gz;
        }
        h
    }

    /// Evaluates nonlinear internal forces $\mathbf{f}_{\text{int}}$ ($30 \times 1$),
    /// symmetric tangent stiffness $\mathbf{K}_e$ ($30 \times 30$), updated 4-point $J_2$
    /// Gauss-point states, and per-GP rotations $\mathbf{R}^{(q)} \in SO(3)$ using
    /// Co-Rotated FEM (when `corotated = true`) and Radial Return Mapping.
    pub fn evaluate_nonlinear(
        &self,
        u_elem: &Vector30<f64>,
        prev_states: &[GaussPointState; 4],
        params: &J2MaterialParams,
        corotated: bool,
        include_geometric_stiffness: bool,
    ) -> (
        Vector30<f64>,
        Matrix30<f64>,
        [ReturnMappingResult; 4],
        [Matrix3<f64>; 4],
    ) {
        let mut f_int = Vector30::<f64>::zeros();
        let mut ke = Matrix30::<f64>::zeros();
        let mut results: [Option<ReturnMappingResult>; 4] = [None, None, None, None];
        let mut rotations = [Matrix3::<f64>::identity(); 4];

        for q in 0..4 {
            let w = self.gauss_weights[q];
            let grads = &self.gauss_gradients[q];
            let h_grad = self.displacement_gradient_at_gp(q, u_elem);

            let (r_mat, u_stretch, eps_loc) = if corotated {
                corotated_biot_strain_voigt(&h_grad)
            } else {
                (
                    Matrix3::identity(),
                    Matrix3::identity(),
                    linear_strain_from_grad_u(&h_grad),
                )
            };

            let rm = radial_return_j2(&eps_loc, &prev_states[q], params);
            let sigma_mat = voigt_stress_to_matrix3(&rm.stress);
            let pk1 = r_mat * sigma_mat;

            for (k, &g) in grads.iter().enumerate() {
                let fk = pk1 * Vector3::new(g[0], g[1], g[2]) * w;
                f_int[3 * k] += fk.x;
                f_int[3 * k + 1] += fk.y;
                f_int[3 * k + 2] += fk.z;
            }

            let r_t = r_mat.transpose();
            let mut b_rot = [SMatrix::<f64, 6, 3>::zeros(); 10];
            let mut cb_rot = [SMatrix::<f64, 6, 3>::zeros(); 10];
            for (k, &g) in grads.iter().enumerate() {
                let br = node_b_matrix(g) * r_t;
                cb_rot[k] = rm.tangent * br;
                b_rot[k] = br;
            }

            let use_geo = corotated && include_geometric_stiffness && rm.stress.norm() > 1e-12;
            let su_inv = if use_geo {
                let su = Matrix3::identity() * u_stretch.trace() - u_stretch;
                su.try_inverse()
            } else {
                None
            };

            let mut geo_a = [Matrix3::<f64>::zeros(); 10];
            let mut geo_m = [Matrix3::<f64>::zeros(); 10];
            if let Some(su_inv_mat) = su_inv {
                for (k, &g) in grads.iter().enumerate() {
                    let gv = Vector3::new(g[0], g[1], g[2]);
                    let tk = sigma_mat * gv;
                    geo_a[k] = r_mat * skew_matrix(&tk);
                    geo_m[k] = su_inv_mat * skew_matrix(&gv) * r_t;
                }
            }

            for a in 0..10 {
                let row = 3 * a;
                for b in a..10 {
                    let col = 3 * b;
                    let mut block = (b_rot[a].transpose() * cb_rot[b]) * w;
                    if su_inv.is_some() {
                        let g_ab = -(geo_a[a] * geo_m[b]) * w;
                        let g_ba = -(geo_a[b] * geo_m[a]) * w;
                        block += (g_ab + g_ba.transpose()) * 0.5;
                    }

                    for r in 0..3 {
                        for c in 0..3 {
                            ke[(row + r, col + c)] += block[(r, c)];
                        }
                    }
                    if b > a {
                        for r in 0..3 {
                            for c in 0..3 {
                                ke[(col + c, row + r)] += block[(r, c)];
                            }
                        }
                    }
                }
            }

            rotations[q] = r_mat;
            results[q] = Some(rm);
        }

        let rm_arr = [
            results[0].unwrap(),
            results[1].unwrap(),
            results[2].unwrap(),
            results[3].unwrap(),
        ];
        (f_int, ke, rm_arr, rotations)
    }

    /// Evaluates nonlinear internal forces $\mathbf{f}_{\text{int}}$ ($30 \times 1$) and 4-point
    /// $J_2$ Gauss-point states without assembling the tangent stiffness (fast path for line search).
    pub fn evaluate_nonlinear_forces_only(
        &self,
        u_elem: &Vector30<f64>,
        prev_states: &[GaussPointState; 4],
        params: &J2MaterialParams,
        corotated: bool,
    ) -> (Vector30<f64>, [ReturnMappingResult; 4], [Matrix3<f64>; 4]) {
        let mut f_int = Vector30::<f64>::zeros();
        let mut results: [Option<ReturnMappingResult>; 4] = [None, None, None, None];
        let mut rotations = [Matrix3::<f64>::identity(); 4];

        for q in 0..4 {
            let w = self.gauss_weights[q];
            let grads = &self.gauss_gradients[q];
            let h_grad = self.displacement_gradient_at_gp(q, u_elem);

            let (r_mat, _u_stretch, eps_loc) = if corotated {
                corotated_biot_strain_voigt(&h_grad)
            } else {
                (
                    Matrix3::identity(),
                    Matrix3::identity(),
                    linear_strain_from_grad_u(&h_grad),
                )
            };

            let rm = radial_return_j2(&eps_loc, &prev_states[q], params);
            let pk1 = r_mat * voigt_stress_to_matrix3(&rm.stress);

            for (k, &g) in grads.iter().enumerate() {
                let fk = pk1 * Vector3::new(g[0], g[1], g[2]) * w;
                f_int[3 * k] += fk.x;
                f_int[3 * k + 1] += fk.y;
                f_int[3 * k + 2] += fk.z;
            }

            rotations[q] = r_mat;
            results[q] = Some(rm);
        }

        let rm_arr = [
            results[0].unwrap(),
            results[1].unwrap(),
            results[2].unwrap(),
            results[3].unwrap(),
        ];
        (f_int, rm_arr, rotations)
    }

    /// Extrapolates arbitrary 4-Gauss-point Voigt 6-vectors (e.g., world-frame Cauchy stresses
    /// or plastic strains) to all 10 element nodes via the exact linear barycentric inverse mapping.
    #[inline]
    pub fn extrapolate_gauss_vectors_to_nodes(gp: &[Vector6<f64>; 4]) -> [Vector6<f64>; 10] {
        const ALPHA: f64 = 1.9270509831248424; // (1 + 3*sqrt(5)) / 4
        const BETA: f64 = -0.30901699437494745; // (1 - sqrt(5)) / 4

        let gp_sum = gp[0] + gp[1] + gp[2] + gp[3];
        let mut nodal = [Vector6::zeros(); 10];
        for i in 0..4 {
            nodal[i] = gp[i] * (ALPHA - BETA) + gp_sum * BETA;
        }
        for (e, &(a, b)) in TET10_EDGE_PAIRS.iter().enumerate() {
            nodal[4 + e] = (nodal[a] + nodal[b]) * 0.5;
        }
        nodal
    }
}

// =========================================================================
// Co-Rotated FEM Kinematics & Polar Decomposition Utilities (P5-T3)
// =========================================================================

#[inline]
fn node_b_matrix(grad: [f64; 3]) -> SMatrix<f64, 6, 3> {
    let [gx, gy, gz] = grad;
    let mut b = SMatrix::<f64, 6, 3>::zeros();
    b[(0, 0)] = gx;
    b[(1, 1)] = gy;
    b[(2, 2)] = gz;
    b[(3, 0)] = gy;
    b[(3, 1)] = gx;
    b[(4, 1)] = gz;
    b[(4, 2)] = gy;
    b[(5, 0)] = gz;
    b[(5, 2)] = gx;
    b
}

#[inline]
fn skew_matrix(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y,
        v.z, 0.0, -v.x,
        -v.y, v.x, 0.0,
    )
}

#[inline]
fn linear_strain_from_grad_u(h: &Matrix3<f64>) -> Vector6<f64> {
    Vector6::new(
        h[(0, 0)],
        h[(1, 1)],
        h[(2, 2)],
        h[(0, 1)] + h[(1, 0)],
        h[(1, 2)] + h[(2, 1)],
        h[(0, 2)] + h[(2, 0)],
    )
}

/// Converts a symmetric Voigt Cauchy stress vector $[\sigma_{xx}, \sigma_{yy}, \sigma_{zz}, \tau_{xy}, \tau_{yz}, \tau_{zx}]$
/// into a symmetric $3 \times 3$ matrix.
#[inline]
pub fn voigt_stress_to_matrix3(s: &Vector6<f64>) -> Matrix3<f64> {
    Matrix3::new(
        s[0], s[3], s[5],
        s[3], s[1], s[4],
        s[5], s[4], s[2],
    )
}

/// Converts a symmetric $3 \times 3$ stress tensor into a Voigt 6-vector
/// $[\sigma_{xx}, \sigma_{yy}, \sigma_{zz}, \tau_{xy}, \tau_{yz}, \tau_{zx}]$.
#[inline]
pub fn matrix3_to_voigt_stress(m: &Matrix3<f64>) -> Vector6<f64> {
    Vector6::new(
        m[(0, 0)],
        m[(1, 1)],
        m[(2, 2)],
        0.5 * (m[(0, 1)] + m[(1, 0)]),
        0.5 * (m[(1, 2)] + m[(2, 1)]),
        0.5 * (m[(0, 2)] + m[(2, 0)]),
    )
}

/// Rotates a Voigt Cauchy stress vector from the co-rotated material frame into the world frame:
/// $\boldsymbol{\sigma}_{\text{world}} = \mathbf{R} \boldsymbol{\sigma}_{\text{loc}} \mathbf{R}^T$.
#[inline]
pub fn rotate_voigt_stress(r: &Matrix3<f64>, s_loc: &Vector6<f64>) -> Vector6<f64> {
    let m = voigt_stress_to_matrix3(s_loc);
    let world = r * m * r.transpose();
    matrix3_to_voigt_stress(&world)
}

/// Rotates an engineering Voigt strain vector $[\varepsilon_{xx}, \varepsilon_{yy}, \varepsilon_{zz}, \gamma_{xy}, \gamma_{yz}, \gamma_{zx}]$
/// from the co-rotated material frame into the world frame:
/// $\boldsymbol{\varepsilon}_{\text{world}} = \mathbf{R} \boldsymbol{\varepsilon}_{\text{loc}} \mathbf{R}^T$.
#[inline]
pub fn rotate_voigt_strain(r: &Matrix3<f64>, e_loc: &Vector6<f64>) -> Vector6<f64> {
    let m = Matrix3::new(
        e_loc[0],
        0.5 * e_loc[3],
        0.5 * e_loc[5],
        0.5 * e_loc[3],
        e_loc[1],
        0.5 * e_loc[4],
        0.5 * e_loc[5],
        0.5 * e_loc[4],
        e_loc[2],
    );
    let world = r * m * r.transpose();
    Vector6::new(
        world[(0, 0)],
        world[(1, 1)],
        world[(2, 2)],
        world[(0, 1)] + world[(1, 0)],
        world[(1, 2)] + world[(2, 1)],
        world[(0, 2)] + world[(2, 0)],
    )
}

/// Computes the exact polar decomposition $\mathbf{F} = \mathbf{R}\mathbf{U}$ with
/// proper rotation $\mathbf{R} \in SO(3)$ ($\det \mathbf{R} = +1$) and symmetric stretch
/// tensor $\mathbf{U} = \mathbf{R}^T \mathbf{F}$ via SVD with reflection safeguard and
/// Newton-Schulz orthogonal polishing.
pub fn polar_decomposition_so3(f: &Matrix3<f64>) -> (Matrix3<f64>, Matrix3<f64>) {
    let svd = f.svd(true, true);
    let (mut u_mat, v_t) = match (svd.u, svd.v_t) {
        (Some(u), Some(vt)) => (u, vt),
        _ => return (Matrix3::identity(), *f),
    };

    let mut r = u_mat * v_t;
    if r.determinant() < 0.0 {
        // Flip the singular vector corresponding to the smallest singular value to enforce det(R) = +1
        let s = svd.singular_values;
        let min_idx = if s[0] <= s[1] && s[0] <= s[2] {
            0
        } else if s[1] <= s[2] {
            1
        } else {
            2
        };
        for row in 0..3 {
            u_mat[(row, min_idx)] = -u_mat[(row, min_idx)];
        }
        r = u_mat * v_t;
    }

    // One Newton-Schulz orthogonal refinement if needed to guarantee R^T R = I to machine precision
    let ortho_err = (r.transpose() * r - Matrix3::identity()).norm();
    if ortho_err > 1e-15 && ortho_err < 0.5 {
        r = r * 1.5 - (r * r.transpose() * r) * 0.5;
    }

    let rt_f = r.transpose() * f;
    let u_stretch = (rt_f + rt_f.transpose()) * 0.5;
    (r, u_stretch)
}

/// Computes the Co-Rotated Biot strain tensor $\mathbf{E} = \mathbf{U} - \mathbf{I}$
/// in engineering Voigt notation from the displacement gradient $\mathbf{H} = \nabla_X \mathbf{u}$
/// (where $\mathbf{F} = \mathbf{I} + \mathbf{H}$), returning `(R, U, E_voigt)`.
///
/// Uses the cancellation-free identity
/// $\mathbf{E} = \frac{1}{2}(\mathbf{R}^T \mathbf{H} + \mathbf{H}^T \mathbf{R}) - \frac{1}{2}(\mathbf{R} - \mathbf{I})^T (\mathbf{R} - \mathbf{I})$
/// so both tiny elastic strains ($\sim 10^{-12}$) and finite rotations ($90^\circ, 180^\circ, 360^\circ$)
/// achieve full `f64` precision.
pub fn corotated_biot_strain_voigt(
    h_grad: &Matrix3<f64>,
) -> (Matrix3<f64>, Matrix3<f64>, Vector6<f64>) {
    let f = Matrix3::identity() + h_grad;
    let (r, mut u_stretch) = polar_decomposition_so3(&f);

    let dr = r - Matrix3::identity();
    let rt_h = r.transpose() * h_grad;
    let mut e_mat = (rt_h + rt_h.transpose()) * 0.5 - (dr.transpose() * dr) * 0.5;

    // Suppress pure floating-point cancellation noise under finite rigid rotations:
    // threshold scales strictly with ||R - I||_F (identically 0 when R = I, ~1e-14 at 180 deg).
    let rot_mag = dr.norm();
    let fp_noise_floor = 64.0 * f64::EPSILON * rot_mag;
    if rot_mag > 0.0 && e_mat.norm() <= fp_noise_floor {
        e_mat = Matrix3::zeros();
        u_stretch = Matrix3::identity();
    }

    let e_voigt = Vector6::new(
        e_mat[(0, 0)],
        e_mat[(1, 1)],
        e_mat[(2, 2)],
        2.0 * e_mat[(0, 1)],
        2.0 * e_mat[(1, 2)],
        2.0 * e_mat[(2, 0)],
    );
    (r, u_stretch, e_voigt)
}

/// Validates that a quadratic `TET10` element has strictly positive corner volume and
/// strictly positive Jacobian determinants $\det J(\mathbf{L}^{(q)}) > 0$ at all 4 Gauss points.
/// Returns the minimum Gauss-point Jacobian determinant on success.
pub fn check_tet10_gauss_jacobians(positions: &[[f64; 3]; 10]) -> Result<f64, FeaError> {
    let c = Matrix4::new(
        1.0, positions[0][0], positions[0][1], positions[0][2],
        1.0, positions[1][0], positions[1][1], positions[1][2],
        1.0, positions[2][0], positions[2][1], positions[2][2],
        1.0, positions[3][0], positions[3][1], positions[3][2],
    );
    let corner_vol = c.determinant() / 6.0;
    if !corner_vol.is_finite() || corner_vol <= 1e-15 {
        return Err(FeaError::DegenerateElement {
            id: 0,
            volume: corner_vol,
        });
    }

    let mut min_det_j = f64::INFINITY;
    for &l in &TET10_GAUSS_POINTS {
        let (det_j, _) = compute_tet10_jacobian_and_gradients(0, positions, l)?;
        if det_j < min_det_j {
            min_det_j = det_j;
        }
    }
    Ok(min_det_j)
}

/// Computes the minimum ratio $\min_{q \in 0..4} \frac{\det J(\mathbf{L}^{(q)})}{\det J_{\text{corner}}}$
/// for a quadratic `TET10` element.
///
/// For a straight-edged tetrahedron this ratio is identically `1.0`. For curved boundary-snapped
/// elements, keeping this ratio above a positive threshold (e.g. `>= 0.15`) guarantees that all
/// 4 Gauss points remain strictly positive and well-conditioned. Returns a non-positive value if
/// the corner tetrahedron or any Gauss point is degenerate or inverted.
pub fn tet10_min_gauss_jacobian_ratio(positions: &[[f64; 3]; 10]) -> f64 {
    let c = Matrix4::new(
        1.0, positions[0][0], positions[0][1], positions[0][2],
        1.0, positions[1][0], positions[1][1], positions[1][2],
        1.0, positions[2][0], positions[2][1], positions[2][2],
        1.0, positions[3][0], positions[3][1], positions[3][2],
    );
    let corner_det = c.determinant();
    if !corner_det.is_finite() || corner_det <= 1e-15 {
        return 0.0;
    }

    let mut min_det = f64::INFINITY;
    for &l in &TET10_GAUSS_POINTS {
        let dn_dl = tet10_barycentric_derivatives(l);
        let mut j_mat = Matrix3::<f64>::zeros();
        for k in 0..10 {
            let d0 = dn_dl[k][0];
            let dxi0 = dn_dl[k][1] - d0;
            let dxi1 = dn_dl[k][2] - d0;
            let dxi2 = dn_dl[k][3] - d0;
            let px = positions[k][0];
            let py = positions[k][1];
            let pz = positions[k][2];
            j_mat[(0, 0)] += px * dxi0;
            j_mat[(0, 1)] += px * dxi1;
            j_mat[(0, 2)] += px * dxi2;
            j_mat[(1, 0)] += py * dxi0;
            j_mat[(1, 1)] += py * dxi1;
            j_mat[(1, 2)] += py * dxi2;
            j_mat[(2, 0)] += pz * dxi0;
            j_mat[(2, 1)] += pz * dxi1;
            j_mat[(2, 2)] += pz * dxi2;
        }
        let det_j = j_mat.determinant();
        if !det_j.is_finite() || det_j <= 1e-15 {
            return det_j / corner_det;
        }
        if det_j < min_det {
            min_det = det_j;
        }
    }
    min_det / corner_det
}

/// Computes the isoparametric Jacobian determinant $\det J(\mathbf{L})$ and physical Cartesian
/// gradients $\nabla N_k(\mathbf{L})$ ($k \in 0..10$) at barycentric coordinates `l = [L0, L1, L2, L3]`.
fn compute_tet10_jacobian_and_gradients(
    elem_id: usize,
    positions: &[[f64; 3]; 10],
    l: [f64; 4],
) -> Result<(f64, [[f64; 3]; 10]), FeaError> {
    let dn_dl = tet10_barycentric_derivatives(l);

    // Reference coordinates (xi_1, xi_2, xi_3) = (L1, L2, L3) with L0 = 1 - xi_1 - xi_2 - xi_3:
    // dN_k / d(xi_m) = dN_k / dL_m - dN_k / dL_0 for m in 1..=3.
    let mut dn_dxi = [[0.0f64; 3]; 10];
    let mut j_mat = Matrix3::<f64>::zeros();

    for k in 0..10 {
        let d0 = dn_dl[k][0];
        let dxi0 = dn_dl[k][1] - d0;
        let dxi1 = dn_dl[k][2] - d0;
        let dxi2 = dn_dl[k][3] - d0;
        dn_dxi[k] = [dxi0, dxi1, dxi2];

        let px = positions[k][0];
        let py = positions[k][1];
        let pz = positions[k][2];

        // J_{r, c} = d x_r / d xi_c
        j_mat[(0, 0)] += px * dxi0;
        j_mat[(0, 1)] += px * dxi1;
        j_mat[(0, 2)] += px * dxi2;

        j_mat[(1, 0)] += py * dxi0;
        j_mat[(1, 1)] += py * dxi1;
        j_mat[(1, 2)] += py * dxi2;

        j_mat[(2, 0)] += pz * dxi0;
        j_mat[(2, 1)] += pz * dxi1;
        j_mat[(2, 2)] += pz * dxi2;
    }

    let det_j = j_mat.determinant();
    if det_j <= 1e-15 {
        return Err(FeaError::DegenerateElement {
            id: elem_id,
            volume: det_j / 6.0,
        });
    }

    let inv_j = j_mat.try_inverse().ok_or(FeaError::DegenerateElement {
        id: elem_id,
        volume: det_j / 6.0,
    })?;

    // Physical gradient: grad_x N_k = J^{-T} * grad_xi N_k
    let mut grads = [[0.0f64; 3]; 10];
    for k in 0..10 {
        let g0 = dn_dxi[k][0];
        let g1 = dn_dxi[k][1];
        let g2 = dn_dxi[k][2];

        grads[k][0] = inv_j[(0, 0)] * g0 + inv_j[(1, 0)] * g1 + inv_j[(2, 0)] * g2;
        grads[k][1] = inv_j[(0, 1)] * g0 + inv_j[(1, 1)] * g1 + inv_j[(2, 1)] * g2;
        grads[k][2] = inv_j[(0, 2)] * g0 + inv_j[(1, 2)] * g1 + inv_j[(2, 2)] * g2;
    }

    Ok((det_j, grads))
}

#[inline]
fn build_b_matrix_from_gradients(grads: &[[f64; 3]; 10]) -> Matrix6x30<f64> {
    let mut b = Matrix6x30::zeros();
    for (k, &[gx, gy, gz]) in grads.iter().enumerate() {
        let col = 3 * k;
        b[(0, col)] = gx;
        b[(1, col + 1)] = gy;
        b[(2, col + 2)] = gz;

        b[(3, col)] = gy;
        b[(3, col + 1)] = gx;

        b[(4, col + 1)] = gz;
        b[(4, col + 2)] = gy;

        b[(5, col)] = gz;
        b[(5, col + 2)] = gx;
    }
    b
}

#[inline]
fn compute_strain_from_gradients(grads: &[[f64; 3]; 10], u_elem: &Vector30<f64>) -> Vector6<f64> {
    let mut exx = 0.0;
    let mut eyy = 0.0;
    let mut ezz = 0.0;
    let mut gxy = 0.0;
    let mut gyz = 0.0;
    let mut gzx = 0.0;

    for (k, &[gx, gy, gz]) in grads.iter().enumerate() {
        let ux = u_elem[3 * k];
        let uy = u_elem[3 * k + 1];
        let uz = u_elem[3 * k + 2];

        exx += gx * ux;
        eyy += gy * uy;
        ezz += gz * uz;
        gxy += gy * ux + gx * uy;
        gyz += gz * uy + gy * uz;
        gzx += gz * ux + gx * uz;
    }

    Vector6::new(exx, eyy, ezz, gxy, gyz, gzx)
}

/// Computes tetrahedron volume and strain-displacement matrix B.
pub fn compute_volume_and_b_matrix(
    elem_id: usize,
    p: [[f64; 3]; 4],
) -> Result<(f64, Matrix6x12<f64>), FeaError> {
    // Coordinate matrix C:
    // [1, x0, y0, z0]
    // [1, x1, y1, z1]
    // [1, x2, y2, z2]
    // [1, x3, y3, z3]
    let c = Matrix4::new(
        1.0, p[0][0], p[0][1], p[0][2],
        1.0, p[1][0], p[1][1], p[1][2],
        1.0, p[2][0], p[2][1], p[2][2],
        1.0, p[3][0], p[3][1], p[3][2],
    );

    let det = c.determinant();
    let volume = det / 6.0;

    if volume <= 1e-15 {
        return Err(FeaError::DegenerateElement {
            id: elem_id,
            volume,
        });
    }

    let inv_c = c.try_inverse().ok_or(FeaError::DegenerateElement {
        id: elem_id,
        volume,
    })?;

    // Row 1 of inv_c: beta_i = dN_i/dx
    // Row 2 of inv_c: gamma_i = dN_i/dy
    // Row 3 of inv_c: delta_i = dN_i/dz
    let beta = [inv_c[(1, 0)], inv_c[(1, 1)], inv_c[(1, 2)], inv_c[(1, 3)]];
    let gamma = [inv_c[(2, 0)], inv_c[(2, 1)], inv_c[(2, 2)], inv_c[(2, 3)]];
    let delta = [inv_c[(3, 0)], inv_c[(3, 1)], inv_c[(3, 2)], inv_c[(3, 3)]];

    let mut b = Matrix6x12::zeros();

    for i in 0..4 {
        let col = 3 * i;

        // Row 0: eps_xx = dNi/dx * u_xi
        b[(0, col)] = beta[i];

        // Row 1: eps_yy = dNi/dy * u_yi
        b[(1, col + 1)] = gamma[i];

        // Row 2: eps_zz = dNi/dz * u_zi
        b[(2, col + 2)] = delta[i];

        // Row 3: gamma_xy = dNi/dy * u_xi + dNi/dx * u_yi
        b[(3, col)] = gamma[i];
        b[(3, col + 1)] = beta[i];

        // Row 4: gamma_yz = dNi/dz * u_yi + dNi/dy * u_zi
        b[(4, col + 1)] = delta[i];
        b[(4, col + 2)] = gamma[i];

        // Row 5: gamma_zx = dNi/dz * u_xi + dNi/dx * u_zi
        b[(5, col)] = delta[i];
        b[(5, col + 2)] = beta[i];
    }

    Ok((volume, b))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elasticity::isotropic_elasticity_matrix;

    fn make_straight_tet10(corners: [[f64; 3]; 4]) -> [[f64; 3]; 10] {
        let mut pos = [[0.0; 3]; 10];
        pos[..4].copy_from_slice(&corners);
        for (e, &(a, b)) in TET10_EDGE_PAIRS.iter().enumerate() {
            pos[4 + e] = [
                0.5 * (corners[a][0] + corners[b][0]),
                0.5 * (corners[a][1] + corners[b][1]),
                0.5 * (corners[a][2] + corners[b][2]),
            ];
        }
        pos
    }

    #[test]
    fn test_reference_tetrahedron_volume_and_gradients() {
        let p = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];

        let (vol, b) = compute_volume_and_b_matrix(0, p).unwrap();
        assert!((vol - 1.0 / 6.0).abs() < 1e-12);

        // Partition of unity derivative sum: d/dx sum(Ni) = 0
        let sum_beta = b[(0, 0)] + b[(0, 3)] + b[(0, 6)] + b[(0, 9)];
        let sum_gamma = b[(1, 1)] + b[(1, 4)] + b[(1, 7)] + b[(1, 10)];
        let sum_delta = b[(2, 2)] + b[(2, 5)] + b[(2, 8)] + b[(2, 11)];

        assert!(sum_beta.abs() < 1e-12);
        assert!(sum_gamma.abs() < 1e-12);
        assert!(sum_delta.abs() < 1e-12);
    }

    #[test]
    fn test_rigid_body_translation_produces_zero_strain() {
        let p = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let elem = Tet4Element::new(0, [0, 1, 2, 3], p).unwrap();

        // Translation by [3.5, -2.1, 8.4]
        let mut u_rigid = Vector12::zeros();
        for i in 0..4 {
            u_rigid[3 * i] = 3.5;
            u_rigid[3 * i + 1] = -2.1;
            u_rigid[3 * i + 2] = 8.4;
        }

        let strain = elem.compute_strain(&u_rigid);
        assert!(strain.norm() < 1e-12);
    }

    #[test]
    fn test_stiffness_symmetry() {
        let p = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let elem = Tet4Element::new(0, [0, 1, 2, 3], p).unwrap();
        let d = isotropic_elasticity_matrix(200.0e9, 0.3).unwrap();
        let ke = elem.stiffness_matrix(&d);

        for i in 0..12 {
            for j in 0..12 {
                assert!((ke[(i, j)] - ke[(j, i)]).abs() < 1e-4);
            }
        }
    }

    #[test]
    fn test_tet10_shape_functions_partition_of_unity_and_kronecker_delta() {
        let node_bary: [[f64; 4]; 10] = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.5, 0.5, 0.0, 0.0],
            [0.0, 0.5, 0.5, 0.0],
            [0.5, 0.0, 0.5, 0.0],
            [0.5, 0.0, 0.0, 0.5],
            [0.0, 0.5, 0.0, 0.5],
            [0.0, 0.0, 0.5, 0.5],
        ];

        for (i, &l) in node_bary.iter().enumerate() {
            let n = tet10_shape_functions(l);
            for (j, &val) in n.iter().enumerate() {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (val - expected).abs() < 1e-14,
                    "Kronecker delta failed at node {i}, shape {j}: got {val}"
                );
            }
        }

        for &gp in &TET10_GAUSS_POINTS {
            let n = tet10_shape_functions(gp);
            let sum: f64 = n.iter().sum();
            assert!((sum - 1.0).abs() < 1e-14, "Partition of unity failed: sum = {sum}");
        }
    }

    #[test]
    fn test_tet10_stiffness_symmetry_and_six_zero_eigenvalues() {
        // Non-trivial general tetrahedron
        let corners = [
            [0.02, -0.01, 0.005],
            [0.14, 0.03, -0.01],
            [0.04, 0.12, 0.02],
            [0.05, 0.02, 0.11],
        ];
        let pos = make_straight_tet10(corners);
        let elem = Tet10Element::new(0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], pos).unwrap();
        let d = isotropic_elasticity_matrix(200.0e9, 0.3).unwrap();
        let ke = elem.stiffness_matrix(&d);

        // Verify exact match with explicit 4-point B^T * D * B quadrature sum
        let mut ke_explicit = Matrix30::<f64>::zeros();
        for q in 0..4 {
            let b = elem.b_matrix_at_gauss_point(q);
            ke_explicit += (b.transpose() * d * b) * elem.gauss_weights[q];
        }
        let max_entry = ke.abs().max();
        assert!((ke - ke_explicit).abs().max() / max_entry < 1e-13);

        // Check symmetry
        for i in 0..30 {
            for j in 0..30 {
                assert!(
                    (ke[(i, j)] - ke[(j, i)]).abs() / max_entry < 1e-14,
                    "Ke not symmetric at ({i}, {j})"
                );
            }
        }

        // Check eigenvalues: 30x30 stiffness matrix of an unconstrained 3D TET10 element
        // must have exactly 6 zero eigenvalues (3 rigid translations + 3 rigid rotations)
        // and 24 strictly positive eigenvalues.
        let mut eigs: Vec<f64> = ke.symmetric_eigenvalues().iter().copied().collect();
        eigs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let max_eig = eigs[29];
        assert!(max_eig > 0.0);

        let zero_count = eigs.iter().filter(|&&lam| lam.abs() < max_eig * 1e-9).count();
        let pos_count = eigs.iter().filter(|&&lam| lam > max_eig * 1e-6).count();
        assert_eq!(
            zero_count, 6,
            "TET10 element stiffness matrix must have exactly 6 zero eigenvalues (rigid-body modes), got {zero_count}: {eigs:?}"
        );
        assert_eq!(
            pos_count, 24,
            "TET10 element stiffness matrix must have 24 strictly positive eigenvalues, got {pos_count}"
        );
    }

    #[test]
    fn test_tet10_single_element_patch_test_linear_displacement() {
        let corners = [
            [0.0, 0.0, 0.0],
            [0.08, 0.01, -0.005],
            [0.01, 0.09, 0.01],
            [0.02, -0.01, 0.07],
        ];
        let pos = make_straight_tet10(corners);
        let elem = Tet10Element::new(0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], pos).unwrap();
        let d = isotropic_elasticity_matrix(200.0e9, 0.3).unwrap();

        // Impose an arbitrary 3D affine displacement field:
        // ux = 1e-4 + 0.0012*x - 0.0004*y + 0.0007*z
        // uy = -2e-4 + 0.0006*x + 0.0009*y - 0.0003*z
        // uz = 3e-4 - 0.0005*x + 0.0008*y - 0.0011*z
        let mut u_elem = Vector30::<f64>::zeros();
        for k in 0..10 {
            let [x, y, z] = pos[k];
            u_elem[3 * k] = 1e-4 + 0.0012 * x - 0.0004 * y + 0.0007 * z;
            u_elem[3 * k + 1] = -2e-4 + 0.0006 * x + 0.0009 * y - 0.0003 * z;
            u_elem[3 * k + 2] = 3e-4 - 0.0005 * x + 0.0008 * y - 0.0011 * z;
        }

        let expected_strain = Vector6::new(
            0.0012,
            0.0009,
            -0.0011,
            -0.0004 + 0.0006,
            -0.0003 + 0.0008,
            0.0007 - 0.0005,
        );
        let expected_stress = d * expected_strain;

        let gp_strains = elem.compute_gauss_strains(&u_elem);
        for (q, s) in gp_strains.iter().enumerate() {
            assert!(
                (s - expected_strain).norm() < 1e-13,
                "Gauss point {q} strain mismatch: got {s:?}, expected {expected_strain:?}"
            );
        }

        let nodal_stresses = elem.extrapolate_nodal_stresses(&d, &u_elem);
        for (k, sigma) in nodal_stresses.iter().enumerate() {
            let rel_err = (sigma - expected_stress).norm() / expected_stress.norm();
            assert!(
                rel_err < 1e-12,
                "Node {k} extrapolated stress error {rel_err:.3e} exceeds tolerance"
            );
        }
    }

    #[test]
    fn test_tet10_gauss_jacobian_checks_straight_curved_and_inverted() {
        let corners = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        // 1. Straight-sided TET10 has uniform Gauss Jacobian equal to corner det6 = 1.0
        let straight = make_straight_tet10(corners);
        let det_min = check_tet10_gauss_jacobians(&straight).expect("straight TET10 must pass");
        assert!((det_min - 1.0).abs() < 1e-12);
        let ratio = tet10_min_gauss_jacobian_ratio(&straight);
        assert!((ratio - 1.0).abs() < 1e-12);

        // 2. Mildly curved edge: outward bow expands Jacobian (>= 1.0), inward bow reduces it (0.5 < r < 1.0)
        let mut curved_out = straight;
        curved_out[4][1] = -0.05;
        let curved_out_det =
            check_tet10_gauss_jacobians(&curved_out).expect("outward curved TET10 must pass");
        assert!(curved_out_det > 1.0);
        assert!(tet10_min_gauss_jacobian_ratio(&curved_out) > 1.0);

        let mut curved_in = straight;
        curved_in[4][1] = 0.05;
        let curved_in_det =
            check_tet10_gauss_jacobians(&curved_in).expect("inward curved TET10 must pass");
        assert!(curved_in_det > 0.5);
        let curved_in_ratio = tet10_min_gauss_jacobian_ratio(&curved_in);
        assert!(curved_in_ratio > 0.5 && curved_in_ratio < 1.0);

        // 3. Severely inverted mid-edge node pulled past the opposite face fails Jacobian check
        let mut inverted = straight;
        inverted[4] = [0.5, 0.8, 0.8];
        assert!(
            check_tet10_gauss_jacobians(&inverted).is_err(),
            "severely bowed TET10 must be rejected"
        );
        assert!(tet10_min_gauss_jacobian_ratio(&inverted) <= 0.0);
    }

    #[test]
    fn test_polar_decomposition_and_corotated_rigid_rotations_zero_stress() {
        let corners = [
            [0.01, -0.02, 0.005],
            [0.11, 0.01, -0.01],
            [0.02, 0.09, 0.015],
            [0.03, 0.01, 0.12],
        ];
        let tet4 = Tet4Element::new(0, [0, 1, 2, 3], corners).unwrap();
        let pos10 = make_straight_tet10(corners);
        let tet10 = Tet10Element::new(0, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], pos10).unwrap();

        let mat = mcp_cad_core::material::Material::new(
            "steel", 205.0, 0.29, 250.0, 7850.0, "steel",
        );
        let params = J2MaterialParams::from_material(&mat).unwrap();
        let state0 = GaussPointState::default();

        // Test exact 90 deg, 180 deg, 270 deg around Z and an arbitrary 3D axis-angle rotation (137 deg)
        let axis_arbitrary = nalgebra::Unit::new_normalize(Vector3::new(0.4, -0.7, 0.59));
        let rotations = [
            nalgebra::Rotation3::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
            nalgebra::Rotation3::from_axis_angle(&Vector3::z_axis(), std::f64::consts::PI),
            nalgebra::Rotation3::from_axis_angle(&Vector3::z_axis(), 1.5 * std::f64::consts::PI),
            nalgebra::Rotation3::from_axis_angle(&axis_arbitrary, 137.0_f64.to_radians()),
        ];

        for rot in rotations {
            let r_exact = *rot.matrix();
            let translation = Vector3::new(0.25, -0.4, 0.15);

            // TET4 rigid rotation
            let mut u4 = Vector12::<f64>::zeros();
            for k in 0..4 {
                let x0 = Vector3::from(corners[k]);
                let x1 = r_exact * x0 + translation;
                let du = x1 - x0;
                u4[3 * k] = du.x;
                u4[3 * k + 1] = du.y;
                u4[3 * k + 2] = du.z;
            }
            let (f4, _k4, rm4, r4) = tet4.evaluate_nonlinear(&u4, &state0, &params, true, true);
            assert!((r4 - r_exact).norm() < 1e-13, "TET4 rotation mismatch");
            assert!(
                Tet4Element::von_mises_stress(&rm4.stress) < 1e-9,
                "TET4 rigid rotation stress must be < 1e-9 Pa, got {}",
                Tet4Element::von_mises_stress(&rm4.stress)
            );
            assert!(f4.norm() < 1e-9, "TET4 rigid rotation internal force must vanish");

            // TET10 rigid rotation
            let mut u10 = Vector30::<f64>::zeros();
            for k in 0..10 {
                let x0 = Vector3::from(pos10[k]);
                let x1 = r_exact * x0 + translation;
                let du = x1 - x0;
                u10[3 * k] = du.x;
                u10[3 * k + 1] = du.y;
                u10[3 * k + 2] = du.z;
            }
            let (f10, _k10, rm10, r10) =
                tet10.evaluate_nonlinear(&u10, &[state0; 4], &params, true, true);
            for q in 0..4 {
                assert!((r10[q] - r_exact).norm() < 1e-13, "TET10 GP {q} rotation mismatch");
                let vm = Tet4Element::von_mises_stress(&rm10[q].stress);
                assert!(
                    vm < 1e-9,
                    "TET10 GP {q} rigid rotation stress must be < 1e-9 Pa, got {vm}"
                );
            }
            assert!(f10.norm() < 1e-9, "TET10 rigid rotation internal force must vanish");
        }
    }

    #[test]
    fn test_corotated_elastoplastic_tangent_matches_finite_differences() {
        let corners = [
            [0.0, 0.0, 0.0],
            [0.10, 0.01, -0.005],
            [0.01, 0.09, 0.01],
            [0.02, -0.01, 0.08],
        ];
        let tet4 = Tet4Element::new(0, [0, 1, 2, 3], corners).unwrap();
        let mat = mcp_cad_core::material::Material::new(
            "steel", 205.0, 0.29, 250.0, 7850.0, "steel",
        );
        let params = J2MaterialParams::from_material(&mat).unwrap();
        let state0 = GaussPointState::default();

        // Combine a finite 35-degree rotation with a small elastoplastic stretch (> yield)
        let rot = nalgebra::Rotation3::from_axis_angle(&Vector3::z_axis(), 35.0_f64.to_radians());
        let r_exact = *rot.matrix();
        let mut u4 = Vector12::<f64>::zeros();
        for k in 0..4 {
            let x0 = Vector3::from(corners[k]);
            let stretched = Vector3::new(x0.x * 1.004, x0.y * 0.9985, x0.z * 0.9985);
            let x1 = r_exact * stretched;
            let du = x1 - x0;
            u4[3 * k] = du.x;
            u4[3 * k + 1] = du.y;
            u4[3 * k + 2] = du.z;
        }

        let (_f0, ke, rm, _r) = tet4.evaluate_nonlinear(&u4, &state0, &params, true, true);
        assert!(rm.plastic_step, "Element must be in plastic regime");

        // Verify symmetry of Ke
        let ke_norm = ke.norm();
        assert!((ke - ke.transpose()).norm() / ke_norm < 1e-13);

        // Compare against symmetrized finite-difference Jacobian of f_int(u)
        let h_fd = 1e-8;
        let mut k_fd = Matrix12::<f64>::zeros();
        for j in 0..12 {
            let mut up = u4;
            let mut um = u4;
            up[j] += h_fd;
            um[j] -= h_fd;
            let (fp, _, _) = tet4.evaluate_nonlinear_forces_only(&up, &state0, &params, true);
            let (fm, _, _) = tet4.evaluate_nonlinear_forces_only(&um, &state0, &params, true);
            let col = (fp - fm) / (2.0 * h_fd);
            for i in 0..12 {
                k_fd[(i, j)] = col[i];
            }
        }
        let k_fd_sym = (k_fd + k_fd.transpose()) * 0.5;
        let rel_err = (ke - k_fd_sym).norm() / k_fd_sym.norm();
        assert!(
            rel_err < 1e-3,
            "Co-rotated elastoplastic tangent mismatch vs FD: rel_err = {rel_err:.3e}"
        );
    }
}



