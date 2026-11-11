//! Linear 4-node (TET4) and quadratic 10-node (TET10) tetrahedral finite element formulations.

use nalgebra::{Matrix3, Matrix4, Matrix6, SMatrix, SVector};
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
}

