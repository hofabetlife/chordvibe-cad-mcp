//! Linear 4-node tetrahedral finite element (TET4) formulation.

use nalgebra::{Matrix4, Matrix6, SMatrix, SVector};
use crate::error::FeaError;

/// Type alias for the 6x12 strain-displacement matrix B.
pub type Matrix6x12<T> = SMatrix<T, 6, 12>;

/// Type alias for the 12x12 element stiffness matrix Ke.
pub type Matrix12<T> = SMatrix<T, 12, 12>;

/// Type alias for the 12-element displacement vector of a tetrahedron.
pub type Vector12<T> = SVector<T, 12>;

/// Type alias for the 6-component stress or strain vector in Voigt notation.
pub type Vector6<T> = SVector<T, 6>;

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
}
