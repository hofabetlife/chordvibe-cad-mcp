//! 3D Linear Elasticity and J2 Elastoplasticity constitutive relations
//! (Hooke's Law, von Mises yield criterion, Radial Return Mapping, and consistent tangent).

use nalgebra::{Matrix6, Vector6};
use serde::{Deserialize, Serialize};
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
    if !youngs_modulus_pa.is_finite()
        || !poissons_ratio.is_finite()
        || youngs_modulus_pa <= 0.0
        || poissons_ratio <= 0.0
        || poissons_ratio >= 0.5
    {
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

// =========================================================================
// J2 (von Mises) Elastoplasticity & Radial Return Mapping (P5-T3)
// =========================================================================

/// Isotropic hardening law $\sigma_Y(\alpha)$ for $J_2$ plasticity.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum HardeningLaw {
    /// Linear isotropic hardening: $\sigma_Y(\alpha) = \sigma_{y0} + H \alpha$ ($H = 0$ is ideal plasticity).
    Linear {
        /// Hardening modulus $H \ge 0$ in Pascals.
        hardening_modulus_pa: f64,
    },
    /// Shifted Ludwik / Swift power-law isotropic hardening:
    /// $\sigma_Y(\alpha) = \sigma_{y0} + K \left((\alpha + \alpha_0)^n - \alpha_0^n\right)$.
    PowerLaw {
        /// Strength coefficient $K \ge 0$ in Pascals.
        strength_coeff_pa: f64,
        /// Hardening exponent $n \in (0, 1]$.
        exponent: f64,
        /// Regularization strain offset $\alpha_0 > 0$ (prevents infinite slope at $\alpha = 0$).
        strain_offset: f64,
    },
    /// Voce-type exponential saturation hardening from $\sigma_{y0}$ to $\sigma_u$:
    /// $\sigma_Y(\alpha) = \sigma_u - (\sigma_u - \sigma_{y0}) \exp\!\left(-\frac{H \alpha}{\sigma_u - \sigma_{y0}}\right)$.
    Saturation {
        /// Initial hardening slope $H \ge 0$ at $\alpha = 0$ in Pascals.
        initial_hardening_modulus_pa: f64,
        /// Saturation stress $\sigma_u \ge \sigma_{y0}$ in Pascals.
        saturation_stress_pa: f64,
    },
}

impl HardeningLaw {
    /// Evaluates current flow (yield) stress $\sigma_Y(\alpha)$ in Pascals.
    #[inline]
    pub fn yield_stress(&self, initial_yield_pa: f64, alpha: f64) -> f64 {
        let a = alpha.max(0.0);
        match *self {
            Self::Linear {
                hardening_modulus_pa,
            } => initial_yield_pa + hardening_modulus_pa.max(0.0) * a,
            Self::PowerLaw {
                strength_coeff_pa,
                exponent,
                strain_offset,
            } => {
                let k = strength_coeff_pa.max(0.0);
                let n = exponent.clamp(1e-4, 1.0);
                let a0 = strain_offset.max(1e-8);
                initial_yield_pa + k * ((a + a0).powf(n) - a0.powf(n))
            }
            Self::Saturation {
                initial_hardening_modulus_pa,
                saturation_stress_pa,
            } => {
                let h0 = initial_hardening_modulus_pa.max(0.0);
                let delta_s = (saturation_stress_pa - initial_yield_pa).max(0.0);
                if h0 <= 0.0 || delta_s <= 1e-12 {
                    initial_yield_pa
                } else {
                    let expo = (-(h0 * a) / delta_s).exp();
                    saturation_stress_pa - delta_s * expo
                }
            }
        }
    }

    /// Evaluates the instantaneous isotropic hardening modulus $H(\alpha) = \frac{d\sigma_Y}{d\alpha} \ge 0$ in Pascals.
    #[inline]
    pub fn hardening_slope(&self, initial_yield_pa: f64, alpha: f64) -> f64 {
        let a = alpha.max(0.0);
        match *self {
            Self::Linear {
                hardening_modulus_pa,
            } => hardening_modulus_pa.max(0.0),
            Self::PowerLaw {
                strength_coeff_pa,
                exponent,
                strain_offset,
            } => {
                let k = strength_coeff_pa.max(0.0);
                let n = exponent.clamp(1e-4, 1.0);
                let a0 = strain_offset.max(1e-8);
                k * n * (a + a0).powf(n - 1.0)
            }
            Self::Saturation {
                initial_hardening_modulus_pa,
                saturation_stress_pa,
            } => {
                let h0 = initial_hardening_modulus_pa.max(0.0);
                let delta_s = (saturation_stress_pa - initial_yield_pa).max(0.0);
                if h0 <= 0.0 || delta_s <= 1e-12 {
                    0.0
                } else {
                    h0 * (-(h0 * a) / delta_s).exp()
                }
            }
        }
    }
}

/// Constitutive parameters for 3D $J_2$ elastoplasticity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct J2MaterialParams {
    /// Young's modulus $E$ in Pascals.
    pub youngs_modulus_pa: f64,
    /// Poisson's ratio $\nu \in (0, 0.5)$.
    pub poissons_ratio: f64,
    /// Lamé shear modulus $G = \frac{E}{2(1+\nu)}$ in Pascals.
    pub shear_modulus_pa: f64,
    /// Bulk modulus $K = \frac{E}{3(1-2\nu)}$ in Pascals.
    pub bulk_modulus_pa: f64,
    /// Initial yield strength $\sigma_{y0}$ in Pascals.
    pub yield_strength_pa: f64,
    /// Ultimate tensile strength $\sigma_u$ in Pascals (metadata / saturation reference).
    pub ultimate_strength_pa: f64,
    /// Isotropic hardening law.
    pub hardening: HardeningLaw,
    /// Precomputed $6 \times 6$ isotropic elastic stiffness matrix $\mathbf{C}^e$.
    pub elastic_matrix: Matrix6<f64>,
}

impl J2MaterialParams {
    /// Builds validated `J2MaterialParams` from a CAD `Material`, using linear isotropic
    /// hardening with `material.hardening_modulus_pa`.
    pub fn from_material(material: &Material) -> Result<Self, FeaError> {
        let e = material.youngs_modulus_pa();
        let nu = material.poissons_ratio;
        let elastic_matrix = isotropic_elasticity_matrix(e, nu)?;
        let yield_pa = material.yield_strength_pa();
        if !yield_pa.is_finite() || yield_pa <= 0.0 {
            return Err(FeaError::General(format!(
                "Invalid yield strength for J2 plasticity: {yield_pa} Pa (must be > 0)"
            )));
        }
        let h = material.hardening_modulus_pa;
        if !h.is_finite() || h < 0.0 {
            return Err(FeaError::General(format!(
                "Invalid hardening modulus for J2 plasticity: {h} Pa (must be >= 0)"
            )));
        }
        let ultimate_pa = material.ultimate_strength_pa.max(yield_pa);
        Ok(Self {
            youngs_modulus_pa: e,
            poissons_ratio: nu,
            shear_modulus_pa: shear_modulus(e, nu),
            bulk_modulus_pa: bulk_modulus(e, nu),
            yield_strength_pa: yield_pa,
            ultimate_strength_pa: ultimate_pa,
            hardening: HardeningLaw::Linear {
                hardening_modulus_pa: h,
            },
            elastic_matrix,
        })
    }

    /// Replaces the isotropic hardening law.
    #[inline]
    pub fn with_hardening(mut self, hardening: HardeningLaw) -> Self {
        self.hardening = hardening;
        self
    }

    /// Switches to elastic–perfectly-plastic behavior ($H = 0$).
    #[inline]
    pub fn with_ideal_plasticity(mut self) -> Self {
        self.hardening = HardeningLaw::Linear {
            hardening_modulus_pa: 0.0,
        };
        self
    }
}

/// Internal constitutive state variables stored at a single Gauss integration point
/// in the co-rotated material frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GaussPointState {
    /// Engineering Voigt plastic strain tensor $[\varepsilon_{xx}^p, \varepsilon_{yy}^p, \varepsilon_{zz}^p, \gamma_{xy}^p, \gamma_{yz}^p, \gamma_{zx}^p]$.
    pub plastic_strain: [f64; 6],
    /// Accumulated equivalent plastic strain $\bar{\varepsilon}_p \ge 0$.
    pub equivalent_plastic_strain: f64,
    /// Isotropic hardening internal variable $\alpha \ge 0$ (equals $\bar{\varepsilon}_p$ for associative $J_2$).
    pub hardening_var: f64,
    /// Current Cauchy stress tensor $[\sigma_{xx}, \sigma_{yy}, \sigma_{zz}, \tau_{xy}, \tau_{yz}, \tau_{zx}]$ in Pascals (co-rotated frame).
    pub stress: [f64; 6],
    /// Accumulated plastic dissipation energy density $w_p = \int \boldsymbol{\sigma} : d\boldsymbol{\varepsilon}^p$ in J/m³.
    pub plastic_energy_density: f64,
}

impl Default for GaussPointState {
    fn default() -> Self {
        Self {
            plastic_strain: [0.0; 6],
            equivalent_plastic_strain: 0.0,
            hardening_var: 0.0,
            stress: [0.0; 6],
            plastic_energy_density: 0.0,
        }
    }
}

impl GaussPointState {
    /// Returns plastic strain as an `nalgebra::Vector6<f64>`.
    #[inline]
    pub fn plastic_strain_vec(&self) -> Vector6<f64> {
        Vector6::from_column_slice(&self.plastic_strain)
    }

    /// Returns current Cauchy stress as an `nalgebra::Vector6<f64>`.
    #[inline]
    pub fn stress_vec(&self) -> Vector6<f64> {
        Vector6::from_column_slice(&self.stress)
    }

    /// Evaluates the von Mises equivalent stress $\sigma_{vM}$ of the stored stress tensor (Pa).
    #[inline]
    pub fn von_mises_stress(&self) -> f64 {
        von_mises_from_voigt(&self.stress_vec())
    }
}

/// Result of a single Gauss-point $J_2$ radial return constitutive update.
#[derive(Debug, Clone, Copy)]
pub struct ReturnMappingResult {
    /// Updated internal state at the end of the increment.
    pub state: GaussPointState,
    /// Updated Cauchy stress vector $\boldsymbol{\sigma}_{n+1}$ in Pascals (co-rotated frame).
    pub stress: Vector6<f64>,
    /// Consistent algorithmic tangent modulus $\mathbf{C}^{\text{ep}} = \frac{\partial \boldsymbol{\sigma}_{n+1}}{\partial \boldsymbol{\varepsilon}_{n+1}}$ ($6 \times 6$, Pa).
    pub tangent: Matrix6<f64>,
    /// True if plastic flow occurred during this increment ($\Delta \bar{\varepsilon}_p > 0$).
    pub plastic_step: bool,
    /// Equivalent plastic strain increment $\Delta \bar{\varepsilon}_p \ge 0$.
    pub delta_eq_plastic_strain: f64,
    /// Elastic strain energy density $w_e = \frac{1}{2} (\boldsymbol{\varepsilon}^e)^T \mathbf{C}^e \boldsymbol{\varepsilon}^e$ in J/m³.
    pub elastic_energy_density: f64,
}

/// Computes von Mises equivalent stress $\sigma_{vM}$ from a Voigt 6-vector
/// $[\sigma_{xx}, \sigma_{yy}, \sigma_{zz}, \tau_{xy}, \tau_{yz}, \tau_{zx}]$.
#[inline]
pub fn von_mises_from_voigt(s: &Vector6<f64>) -> f64 {
    let p = (s[0] + s[1] + s[2]) / 3.0;
    let s0 = s[0] - p;
    let s1 = s[1] - p;
    let s2 = s[2] - p;
    let dev_norm_sq =
        s0 * s0 + s1 * s1 + s2 * s2 + 2.0 * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
    (1.5 * dev_norm_sq).sqrt()
}

/// Executes the 3D $J_2$ (von Mises) **Radial Return Mapping** algorithm with isotropic
/// hardening and returns the updated stress, updated internal variables, and exact
/// symmetric **consistent algorithmic tangent modulus** $\mathbf{C}^{\text{ep}}$.
///
/// - `total_strain`: total engineering Voigt strain $[\varepsilon_{xx}, \varepsilon_{yy}, \varepsilon_{zz}, \gamma_{xy}, \gamma_{yz}, \gamma_{zx}]^T$
///   in the co-rotated material frame.
/// - `prev_state`: converged internal state $(\boldsymbol{\varepsilon}^p_n, \bar{\varepsilon}_{p,n}, \alpha_n)$
///   from the **previous converged load increment** (never from a rejected Newton trial).
pub fn radial_return_j2(
    total_strain: &Vector6<f64>,
    prev_state: &GaussPointState,
    params: &J2MaterialParams,
) -> ReturnMappingResult {
    let eps_p_prev = prev_state.plastic_strain_vec();
    let eps_e_trial = total_strain - eps_p_prev;
    let sigma_trial = params.elastic_matrix * eps_e_trial;

    // Hydrostatic pressure and deviatoric trial stress
    let p = (sigma_trial[0] + sigma_trial[1] + sigma_trial[2]) / 3.0;
    let s_trial = Vector6::new(
        sigma_trial[0] - p,
        sigma_trial[1] - p,
        sigma_trial[2] - p,
        sigma_trial[3],
        sigma_trial[4],
        sigma_trial[5],
    );

    // Tensor Frobenius norm ||s_trial||_F and von Mises equivalent trial stress q_trial
    let s_norm_sq = s_trial[0] * s_trial[0]
        + s_trial[1] * s_trial[1]
        + s_trial[2] * s_trial[2]
        + 2.0 * (s_trial[3] * s_trial[3] + s_trial[4] * s_trial[4] + s_trial[5] * s_trial[5]);
    let s_norm = s_norm_sq.sqrt();
    let q_trial = (1.5_f64).sqrt() * s_norm;

    let yield_n = params
        .hardening
        .yield_stress(params.yield_strength_pa, prev_state.hardening_var);
    let f_trial = q_trial - yield_n;

    // Purely elastic step (or zero deviatoric trial stress / zero-increment point sitting on yield surface)
    if f_trial <= 1e-10 * yield_n || q_trial <= 1e-15 {
        let elastic_energy_density = 0.5 * eps_e_trial.dot(&sigma_trial).max(0.0);
        let mut next_state = *prev_state;
        next_state.stress = [
            sigma_trial[0],
            sigma_trial[1],
            sigma_trial[2],
            sigma_trial[3],
            sigma_trial[4],
            sigma_trial[5],
        ];
        return ReturnMappingResult {
            state: next_state,
            stress: sigma_trial,
            tangent: params.elastic_matrix,
            plastic_step: false,
            delta_eq_plastic_strain: 0.0,
            elastic_energy_density,
        };
    }

    // Plastic corrector: solve g(dp) = q_trial - 3*G*dp - sigma_Y(alpha_n + dp) = 0
    let g_shear = params.shear_modulus_pa;
    let k_bulk = params.bulk_modulus_pa;
    let three_g = 3.0 * g_shear;
    let alpha_n = prev_state.hardening_var;

    let delta_p = match params.hardening {
        HardeningLaw::Linear {
            hardening_modulus_pa,
        } => {
            let h = hardening_modulus_pa.max(0.0);
            (f_trial / (three_g + h)).max(0.0)
        }
        HardeningLaw::PowerLaw { .. } | HardeningLaw::Saturation { .. } => {
            // Safeguarded Newton-Raphson on strictly decreasing scalar g(dp) = q_trial - 3G*dp - sigma_Y(alpha_n + dp)
            let mut dp_lo = 0.0_f64;
            let mut dp_hi = f_trial / three_g;
            let mut dp = f_trial
                / (three_g
                    + params
                        .hardening
                        .hardening_slope(params.yield_strength_pa, alpha_n));
            dp = dp.clamp(dp_lo, dp_hi);
            let tol = 1e-12 * yield_n.max(1.0);

            for _ in 0..32 {
                let sy = params
                    .hardening
                    .yield_stress(params.yield_strength_pa, alpha_n + dp);
                let g_val = q_trial - three_g * dp - sy;
                if g_val.abs() <= tol {
                    break;
                }
                if g_val > 0.0 {
                    dp_lo = dp;
                } else {
                    dp_hi = dp;
                }
                let h_loc = params
                    .hardening
                    .hardening_slope(params.yield_strength_pa, alpha_n + dp);
                let dg = -(three_g + h_loc);
                let dp_newton = dp - g_val / dg;
                if dp_newton > dp_lo && dp_newton < dp_hi {
                    dp = dp_newton;
                } else {
                    dp = 0.5 * (dp_lo + dp_hi);
                }
            }
            dp.max(0.0)
        }
    };

    let alpha_new = alpha_n + delta_p;
    let h_local = params
        .hardening
        .hardening_slope(params.yield_strength_pa, alpha_new);

    // Radial scaling factor theta = 1 - 3*G*delta_p / q_trial = q_{n+1} / q_trial
    let theta = (1.0 - (three_g * delta_p) / q_trial).clamp(0.0, 1.0);
    let sigma_new = Vector6::new(
        p + theta * s_trial[0],
        p + theta * s_trial[1],
        p + theta * s_trial[2],
        theta * s_trial[3],
        theta * s_trial[4],
        theta * s_trial[5],
    );

    // Engineering Voigt plastic strain update:
    //   normal components:  deps_i^p = (3/2) * (delta_p / q_trial) * s_i^trial
    //   shear components:  dgamma_i^p = 2 * deps_ij^p = 3 * (delta_p / q_trial) * s_i^trial
    let factor_normal = 1.5 * delta_p / q_trial;
    let factor_shear = 3.0 * delta_p / q_trial;
    let delta_eps_p = Vector6::new(
        factor_normal * s_trial[0],
        factor_normal * s_trial[1],
        factor_normal * s_trial[2],
        factor_shear * s_trial[3],
        factor_shear * s_trial[4],
        factor_shear * s_trial[5],
    );
    let eps_p_new = eps_p_prev + delta_eps_p;
    let eps_e_new = total_strain - eps_p_new;

    // Elastic strain energy and backward-Euler plastic dissipation density
    let elastic_energy_density = 0.5 * eps_e_new.dot(&sigma_new).max(0.0);
    let q_new = theta * q_trial;
    let delta_wp = (q_new * delta_p).max(0.0);

    // Consistent algorithmic tangent modulus C^ep (symmetric 6x6 in engineering Voigt notation):
    //   C^ep = K (1 ⊗ 1) + 2*G*theta * I_dev - 2*G*theta_bar * (n ⊗ n)
    // where n = s_trial / ||s_trial||_F and theta_bar = 3G / (3G + H) - (1 - theta).
    let n_vec = s_trial / s_norm;
    let theta_bar = (three_g / (three_g + h_local)) - (1.0 - theta);

    let two_g_theta = 2.0 * g_shear * theta;
    let two_g_theta_bar = 2.0 * g_shear * theta_bar;

    let mut c_ep = Matrix6::<f64>::zeros();
    let c_diag_n = k_bulk + (4.0 / 3.0) * g_shear * theta;
    let c_off_n = k_bulk - (2.0 / 3.0) * g_shear * theta;

    c_ep[(0, 0)] = c_diag_n;
    c_ep[(1, 1)] = c_diag_n;
    c_ep[(2, 2)] = c_diag_n;
    c_ep[(0, 1)] = c_off_n;
    c_ep[(1, 0)] = c_off_n;
    c_ep[(0, 2)] = c_off_n;
    c_ep[(2, 0)] = c_off_n;
    c_ep[(1, 2)] = c_off_n;
    c_ep[(2, 1)] = c_off_n;

    // Shear diagonal in engineering strain convention is G * theta = 0.5 * (2*G*theta)
    let g_theta = 0.5 * two_g_theta;
    c_ep[(3, 3)] = g_theta;
    c_ep[(4, 4)] = g_theta;
    c_ep[(5, 5)] = g_theta;

    // Subtract rank-1 consistent normal projection: -2*G*theta_bar * (n * n^T)
    for r in 0..6 {
        let nr = n_vec[r];
        for c in 0..6 {
            c_ep[(r, c)] -= two_g_theta_bar * nr * n_vec[c];
        }
    }

    let next_state = GaussPointState {
        plastic_strain: [
            eps_p_new[0],
            eps_p_new[1],
            eps_p_new[2],
            eps_p_new[3],
            eps_p_new[4],
            eps_p_new[5],
        ],
        equivalent_plastic_strain: prev_state.equivalent_plastic_strain + delta_p,
        hardening_var: alpha_new,
        stress: [
            sigma_new[0],
            sigma_new[1],
            sigma_new[2],
            sigma_new[3],
            sigma_new[4],
            sigma_new[5],
        ],
        plastic_energy_density: prev_state.plastic_energy_density + delta_wp,
    };

    ReturnMappingResult {
        state: next_state,
        stress: sigma_new,
        tangent: c_ep,
        plastic_step: true,
        delta_eq_plastic_strain: delta_p,
        elastic_energy_density,
    }
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

    #[test]
    fn test_radial_return_elastic_unloading_and_uniaxial_exactness() {
        // Steel: E = 200 GPa, nu = 0.3, sigma_y = 250 MPa, H = 10 GPa
        let mat = Material::new("test_steel", 200.0, 0.30, 250.0, 7850.0, "test")
            .with_nonlinear_properties(
                500.0e6,
                10.0e9,
                0.20,
                25_000.0,
                0.25,
                mcp_cad_core::material::FailureMode::Ductile,
            );
        let params = J2MaterialParams::from_material(&mat).unwrap();

        // Target uniaxial stress = 350 MPa (> 250 MPa yield)
        let sigma_target = 350.0e6;
        let eps_p_exact = (sigma_target - 250.0e6) / 10.0e9; // 0.01
        let eps_e_exact = sigma_target / 200.0e9; // 0.00175
        let eps_x_total = eps_e_exact + eps_p_exact;
        // Exact lateral strain under uniaxial stress: -nu * eps_e - 0.5 * eps_p
        let eps_lat = -0.30 * eps_e_exact - 0.5 * eps_p_exact;
        let total_strain = Vector6::new(eps_x_total, eps_lat, eps_lat, 0.0, 0.0, 0.0);

        let res_load = radial_return_j2(&total_strain, &GaussPointState::default(), &params);
        assert!(res_load.plastic_step);
        assert!((res_load.stress[0] - sigma_target).abs() / sigma_target < 1e-12);
        assert!(res_load.stress[1].abs() < 1e-4);
        assert!(res_load.stress[2].abs() < 1e-4);
        assert!((res_load.state.equivalent_plastic_strain - eps_p_exact).abs() < 1e-13);
        assert!((res_load.state.plastic_strain[0] - eps_p_exact).abs() < 1e-13);
        assert!((res_load.state.plastic_strain[1] + 0.5 * eps_p_exact).abs() < 1e-13);

        // Complete unloading to zero stress: residual strain equals exact plastic strain
        let unload_strain = Vector6::new(eps_p_exact, -0.5 * eps_p_exact, -0.5 * eps_p_exact, 0.0, 0.0, 0.0);
        let res_unload = radial_return_j2(&unload_strain, &res_load.state, &params);
        assert!(!res_unload.plastic_step);
        assert!(res_unload.stress.norm() < 1e-4);
        assert!((res_unload.state.equivalent_plastic_strain - eps_p_exact).abs() < 1e-14);
        assert_eq!(res_unload.tangent, params.elastic_matrix);
    }

    #[test]
    fn test_consistent_algorithmic_tangent_matches_finite_differences_all_laws() {
        let mat = Material::new("fd_steel", 205.0, 0.29, 250.0, 7850.0, "test");
        let base_params = J2MaterialParams::from_material(&mat).unwrap();

        let laws = [
            HardeningLaw::Linear {
                hardening_modulus_pa: 5.0e9,
            },
            HardeningLaw::Linear {
                hardening_modulus_pa: 0.0, // ideal plasticity
            },
            HardeningLaw::PowerLaw {
                strength_coeff_pa: 300.0e6,
                exponent: 0.35,
                strain_offset: 1e-4,
            },
            HardeningLaw::Saturation {
                initial_hardening_modulus_pa: 8.0e9,
                saturation_stress_pa: 450.0e6,
            },
        ];

        // General 3D multiaxial strain state well into plastic regime (all 6 components active)
        let strain0 = Vector6::new(0.0045, -0.0018, -0.0012, 0.0032, -0.0021, 0.0027);
        let pre_strain = Vector6::new(0.0022, -0.0008, -0.0006, 0.0011, -0.0005, 0.0009);
        let h_fd = 1e-8;

        for law in laws {
            let params = base_params.with_hardening(law);
            // First create a non-zero initial plastic state to test continued plastic loading
            let prev = radial_return_j2(&pre_strain, &GaussPointState::default(), &params).state;
            let res = radial_return_j2(&strain0, &prev, &params);
            assert!(res.plastic_step, "Test strain must be in plastic regime for {law:?}");

            // Check symmetry of C^ep
            let c_norm = res.tangent.norm();
            for r in 0..6 {
                for c in 0..6 {
                    assert!(
                        (res.tangent[(r, c)] - res.tangent[(c, r)]).abs() / c_norm < 1e-13,
                        "C^ep must be symmetric for {law:?}"
                    );
                }
            }

            // Finite-difference Jacobian verification across all 6 Voigt strain components
            let mut c_fd = Matrix6::<f64>::zeros();
            for j in 0..6 {
                let mut e_plus = strain0;
                let mut e_minus = strain0;
                e_plus[j] += h_fd;
                e_minus[j] -= h_fd;
                let s_plus = radial_return_j2(&e_plus, &prev, &params).stress;
                let s_minus = radial_return_j2(&e_minus, &prev, &params).stress;
                let col = (s_plus - s_minus) / (2.0 * h_fd);
                for i in 0..6 {
                    c_fd[(i, j)] = col[i];
                }
            }

            let rel_err = (res.tangent - c_fd).norm() / c_fd.norm();
            assert!(
                rel_err < 1e-7,
                "Consistent tangent mismatch for {law:?}: rel_err = {rel_err:.3e}"
            );
        }
    }
}

