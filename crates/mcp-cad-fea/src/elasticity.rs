//! 3D Linear Elasticity constitutive relations (Hooke's Law for isotropic materials).

use nalgebra::Matrix6;
use mcp_cad_core::material::Material;
use crate::error::FeaError;

/// Computes the 6x6 isotropic elasticity matrix (stiffness tensor $D$ in Voigt notation):
///
/// $$\sigma = D \cdot \epsilon$$
///
/// where stress $\sigma = [\sigma_{xx}, \sigma_{yy}, \sigma_{zz}, \tau_{xy}, \tau_{yz}, \tau_{zx}]^T$
/// and strain $\epsilon = [\epsilon_{xx}, \epsilon_{yy}, \epsilon_{zz}, \gamma_{xy}, \gamma_{yz}, \gamma_{zx}]^T$.
///
/// $$D = \frac{E}{(1+\nu)(1-2\nu)} \begin{bmatrix}
/// 1-\nu & \nu & \nu & 0 & 0 & 0 \\
/// \nu & 1-\nu & \nu & 0 & 0 & 0 \\
/// \nu & \nu & 1-\nu & 0 & 0 & 0 \\
/// 0 & 0 & 0 & \frac{1-2\nu}{2} & 0 & 0 \\
/// 0 & 0 & 0 & 0 & \frac{1-2\nu}{2} & 0 \\
/// 0 & 0 & 0 & 0 & 0 & \frac{1-2\nu}{2}
/// \end{bmatrix}$$
pub fn isotropic_elasticity_matrix(
    youngs_modulus_pa: f64,
    poissons_ratio: f64,
) -> Result<Matrix6<f64>, FeaError> {
    if youngs_modulus_pa <= 0.0 || poissons_ratio <= 0.0 || poissons_ratio >= 0.5 {
        return Err(FeaError::InvalidMaterial {
            youngs_modulus_pa,
            poissons_ratio,
        });
    }

    let e = youngs_modulus_pa;
    let nu = poissons_ratio;

    let factor = e / ((1.0 + nu) * (1.0 - 2.0 * nu));
    let c11 = factor * (1.0 - nu);
    let c12 = factor * nu;
    let g = factor * (1.0 - 2.0 * nu) * 0.5; // Shear modulus G = E / (2 * (1 + nu))

    let mut d = Matrix6::zeros();
    // Normal components
    d[(0, 0)] = c11;
    d[(0, 1)] = c12;
    d[(0, 2)] = c12;

    d[(1, 0)] = c12;
    d[(1, 1)] = c11;
    d[(1, 2)] = c12;

    d[(2, 0)] = c12;
    d[(2, 1)] = c12;
    d[(2, 2)] = c11;

    // Shear components (engineering shear strain: gamma_xy, gamma_yz, gamma_zx)
    d[(3, 3)] = g;
    d[(4, 4)] = g;
    d[(5, 5)] = g;

    Ok(d)
}

/// Computes the 6x6 elasticity matrix directly from a CAD `Material`.
pub fn elasticity_matrix_from_material(material: &Material) -> Result<Matrix6<f64>, FeaError> {
    isotropic_elasticity_matrix(material.youngs_modulus_pa(), material.poissons_ratio)
}

/// Computes shear modulus $G = \frac{E}{2(1+\nu)}$ in Pascals.
#[inline]
pub fn shear_modulus(youngs_modulus_pa: f64, poissons_ratio: f64) -> f64 {
    youngs_modulus_pa / (2.0 * (1.0 + poissons_ratio))
}

/// Computes bulk modulus $K = \frac{E}{3(1-2\nu)}$ in Pascals.
#[inline]
pub fn bulk_modulus(youngs_modulus_pa: f64, poissons_ratio: f64) -> f64 {
    youngs_modulus_pa / (3.0 * (1.0 - 2.0 * poissons_ratio))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_steel_elasticity_matrix() {
        let e = 205.0e9; // 205 GPa
        let nu = 0.29;
        let d = isotropic_elasticity_matrix(e, nu).unwrap();

        let g_expected = e / (2.0 * (1.0 + nu));
        assert!((d[(3, 3)] - g_expected).abs() < 1e-4);
        assert!((d[(4, 4)] - g_expected).abs() < 1e-4);
        assert!((d[(5, 5)] - g_expected).abs() < 1e-4);

        // Symmetry
        assert_eq!(d[(0, 1)], d[(1, 0)]);
        assert_eq!(d[(0, 2)], d[(2, 0)]);
        assert_eq!(d[(1, 2)], d[(2, 1)]);
    }

    #[test]
    fn test_invalid_material() {
        assert!(isotropic_elasticity_matrix(-100.0, 0.3).is_err());
        assert!(isotropic_elasticity_matrix(200.0e9, 0.5).is_err());
        assert!(isotropic_elasticity_matrix(200.0e9, -0.1).is_err());
    }
}
