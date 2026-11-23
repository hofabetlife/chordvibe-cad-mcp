//! Continuum Damage Mechanics and Bažant Crack-Band Energy Regularization (`P5-T5`).
//!
//! Implements local scalar damage $D \in [0, 1]$ per finite element with mesh-independent
//! fracture energy regularization ($G_f / h_e$, Bažant & Oh 1983):
//! - **Ductile materials (`FailureMode::Ductile`)**: Johnson–Cook / Rice–Tracey triaxiality-dependent
//!   damage initiation after plastic yielding ($\bar{\varepsilon}_p \ge \varepsilon_{\text{onset}}(\eta)$
//!   with compressive cutoff $\eta \le -1/3$, guaranteeing zero damage during moderate yielding
//!   $\sigma_y < \sigma_{vM} < \sigma_u$) followed by crack-band softening controlled by $G_f / h_e$.
//! - **Brittle materials (`FailureMode::Brittle`)**: Rankine maximum principal tensile stress
//!   criterion ($\sigma_1 \ge \sigma_u$) with exact Bažant exponential softening (or instant brittle
//!   cleavage when $G_f / h_e \le \sigma_u^2 / (2 E)$, e.g. `borosilicate_glass`).

use mcp_cad_core::material::{FailureMode, Material};
use nalgebra::{Matrix3, Vector3, Vector6};
use serde::{Deserialize, Serialize};

use crate::elasticity::von_mises_from_voigt;
use crate::element::voigt_stress_to_matrix3;
use crate::error::FeaError;

/// Constitutive parameters governing continuum damage initiation, Bažant crack-band softening,
/// and element erosion / crack face splitting (`P5-T5`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DamageMaterialParams {
    /// Ductile (Rice–Tracey / Johnson–Cook) vs Brittle (Rankine $\sigma_1 \ge \sigma_u$).
    pub failure_mode: FailureMode,
    /// Mass density $\rho$ in kg/m³ (used for detached fragment mass & inertia integration).
    pub density_kg_m3: f64,
    /// Young's modulus $E$ in Pascals.
    pub youngs_modulus_pa: f64,
    /// Poisson's ratio $\nu \in (0, 0.5)$.
    pub poissons_ratio: f64,
    /// Initial yield strength $\sigma_y$ in Pascals.
    pub yield_strength_pa: f64,
    /// Ultimate tensile strength $\sigma_u$ in Pascals ($\sigma_u \ge \sigma_y$).
    pub ultimate_strength_pa: f64,
    /// Isotropic linear hardening modulus $H$ in Pascals ($H \ge 0$).
    pub hardening_modulus_pa: f64,
    /// Reference uniaxial fracture plastic strain $\varepsilon_f > 0$.
    pub fracture_strain: f64,
    /// Specific fracture energy $G_f$ in J/m² ($G_f > 0$).
    pub fracture_energy_j_m2: f64,
    /// Reference uniaxial ($\eta = 1/3$) ductile damage initiation plastic strain $\varepsilon_{\text{onset}, 0}$.
    /// By default calibrated to the necking strain $(\sigma_u - \sigma_y)/H$ (clamped to $[0.25, 0.75]\,\varepsilon_f$)
    /// so moderate plastic yielding with $\sigma_y < \sigma_{vM} < \sigma_u$ produces strictly $D = 0$.
    pub damage_onset_strain: f64,
    /// Rice–Tracey / Johnson–Cook triaxiality exponential sensitivity $d_3 \ge 0$ (default `1.5`).
    pub triaxiality_exponent: f64,
    /// Compressive stress triaxiality cutoff $\eta_{\text{cut}}$ below which ductile void-growth
    /// damage cannot initiate (default `-1/3`, Bao–Wierzbicki uniaxial compression cutoff).
    pub compression_triaxiality_cutoff: f64,
    /// Damage threshold $D_{\text{erode}} \in (0, 1]$ at which an element is eroded or a crack face is split
    /// (default `0.995`).
    pub erode_threshold: f64,
    /// Minimum residual stiffness scale $(1 - D_{\text{eff}}) \ge d_{\min}$ for severely damaged
    /// but not yet eroded elements (default `1e-3`, keeps tangent SPD prior to erosion).
    pub min_residual_stiffness: f64,
}

impl DamageMaterialParams {
    /// Builds validated [`DamageMaterialParams`] directly from a CAD [`Material`].
    pub fn from_material(material: &Material) -> Result<Self, FeaError> {
        let e = material.youngs_modulus_pa();
        let nu = material.poissons_ratio;
        if !e.is_finite() || e <= 0.0 || !nu.is_finite() || nu <= 0.0 || nu >= 0.5 {
            return Err(FeaError::InvalidMaterial {
                youngs_modulus_pa: e,
                poissons_ratio: nu,
            });
        }
        let yield_pa = material.yield_strength_pa();
        let ultimate_pa = material.ultimate_strength_pa.max(yield_pa);
        let hardening_pa = material.hardening_modulus_pa.max(0.0);
        let fracture_strain = material.fracture_strain;
        let fracture_energy = material.fracture_energy_j_m2;
        let density = material.density_kg_m3;

        if !density.is_finite() || density <= 0.0 {
            return Err(FeaError::General(format!(
                "Invalid material density for fracture model: {density} kg/m³ (must be > 0)"
            )));
        }
        if !fracture_strain.is_finite() || fracture_strain <= 0.0 {
            return Err(FeaError::General(format!(
                "Invalid fracture_strain for fracture model: {fracture_strain} (must be > 0)"
            )));
        }
        if !fracture_energy.is_finite() || fracture_energy <= 0.0 {
            return Err(FeaError::General(format!(
                "Invalid fracture_energy_j_m2 for fracture model: {fracture_energy} (must be > 0)"
            )));
        }

        // Calibrate reference uniaxial ductile damage initiation strain to the Considère/necking
        // strain (σ_u - σ_y) / H when H > 0, bounded within [0.25 * ε_f, 0.75 * ε_f] so that:
        // 1) Moderate plastic yielding (σ_y < σ_vM < σ_u) accumulates plastic strain with D == 0.
        // 2) There is always a positive post-onset softening band governed by Bažant G_f / h_e.
        let damage_onset_strain = if hardening_pa > 1e-6 {
            let necking_eps = ((ultimate_pa - yield_pa) / hardening_pa).max(0.0);
            if necking_eps > 0.0 {
                necking_eps.clamp(0.25 * fracture_strain, 0.75 * fracture_strain)
            } else {
                0.50 * fracture_strain
            }
        } else {
            0.50 * fracture_strain
        };

        Ok(Self {
            failure_mode: material.failure_mode,
            density_kg_m3: density,
            youngs_modulus_pa: e,
            poissons_ratio: nu,
            yield_strength_pa: yield_pa,
            ultimate_strength_pa: ultimate_pa,
            hardening_modulus_pa: hardening_pa,
            fracture_strain,
            fracture_energy_j_m2: fracture_energy,
            damage_onset_strain,
            triaxiality_exponent: 1.5,
            compression_triaxiality_cutoff: -1.0 / 3.0,
            erode_threshold: 0.995,
            min_residual_stiffness: 1e-3,
        })
    }

    /// Overrides the reference uniaxial ductile damage initiation plastic strain $\varepsilon_{\text{onset}, 0}$.
    #[inline]
    pub fn with_damage_onset_strain(mut self, onset_strain: f64) -> Self {
        self.damage_onset_strain = onset_strain.max(1e-6);
        self
    }

    /// Overrides the Rice–Tracey triaxiality exponent $d_3$.
    #[inline]
    pub fn with_triaxiality_exponent(mut self, exponent: f64) -> Self {
        self.triaxiality_exponent = exponent.max(0.0);
        self
    }

    /// Overrides the fracture energy $G_f$ in J/m².
    #[inline]
    pub fn with_fracture_energy(mut self, fracture_energy_j_m2: f64) -> Self {
        self.fracture_energy_j_m2 = fracture_energy_j_m2.max(1e-6);
        self
    }
}

/// Local continuum damage and fracture telemetry state stored per finite element.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ElementDamageState {
    /// Irreversible scalar damage variable $D \in [0.0, 1.0]$ (`0.0` = intact, `1.0` = fully fractured).
    pub damage: f64,
    /// Damage initiation history indicator:
    /// - Ductile: cumulative Johnson–Cook / Rice–Tracey integral $\omega_D = \int d\bar{\varepsilon}_p / \varepsilon_{\text{onset}}(\eta)$ (damage starts at $\omega_D \ge 1$).
    /// - Brittle: maximum historical Rankine equivalent tensile strain $\kappa = \max_{\tau \le t} (\sigma_1 / E)$ (damage starts at $\kappa \ge \sigma_u / E$).
    pub initiation_indicator: f64,
    /// Equivalent plastic strain recorded at the moment ductile damage initiated.
    pub onset_plastic_strain: f64,
    /// Reference flow stress $\sigma_{\text{onset}}$ (Pa) recorded when damage initiated.
    pub onset_stress_pa: f64,
    /// Peak positive maximum principal stress $\sigma_1$ (Pa) experienced by this element.
    pub max_principal_stress_pa: f64,
    /// Most recent stress triaxiality $\eta = \sigma_m / \sigma_{vM}$.
    pub triaxiality: f64,
    /// Unit normal $\mathbf{n}_1$ of the Mode-I tensile crack plane (eigenvector of $\sigma_1$ in world frame).
    pub crack_normal: [f64; 3],
    /// Accumulated Bažant crack-band fracture energy density $w_f$ in J/m³ ($\int_{V_e} w_f \, dV = G_f A_{\text{crack}}$).
    pub fracture_energy_density_j_m3: f64,
    /// Characteristic Bažant crack-band element length $h_e = V_e^{1/3}$ in meters.
    pub characteristic_length_m: f64,
    /// `true` once the element has reached complete failure ($D \ge D_{\text{erode}}$) and been eroded/split.
    pub eroded: bool,
}

impl Default for ElementDamageState {
    fn default() -> Self {
        Self {
            damage: 0.0,
            initiation_indicator: 0.0,
            onset_plastic_strain: 0.0,
            onset_stress_pa: 0.0,
            max_principal_stress_pa: 0.0,
            triaxiality: 0.0,
            crack_normal: [1.0, 0.0, 0.0],
            fracture_energy_density_j_m3: 0.0,
            characteristic_length_m: 1e-3,
            eroded: false,
        }
    }
}

impl ElementDamageState {
    /// Creates an intact initial damage state for an element of reference volume `volume_m3`.
    #[inline]
    pub fn intact(volume_m3: f64) -> Self {
        Self {
            characteristic_length_m: characteristic_element_length(volume_m3),
            ..Self::default()
        }
    }

    /// Returns the effective Kachanov stiffness/stress scaling factor $(1 - D_{\text{eff}}) \in [d_{\min}, 1]$
    /// for an active (non-eroded) element, or `0.0` if eroded.
    #[inline]
    pub fn stiffness_scale(&self, min_residual_stiffness: f64) -> f64 {
        if self.eroded {
            0.0
        } else {
            effective_stiffness_factor(self.damage, min_residual_stiffness)
        }
    }
}

/// Output of evaluating continuum damage evolution on a single finite element.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DamageUpdateResult {
    /// Updated irreversible element damage state.
    pub state: ElementDamageState,
    /// Increment in scalar damage $\Delta D = D_{n+1} - D_n \ge 0$.
    pub delta_damage: f64,
    /// Incremental fracture energy dissipated in this element $\Delta W_f = V_e \Delta w_f \ge 0$ (Joules).
    pub delta_fracture_energy_j: f64,
    /// `true` if the element transitioned from active to eroded during this update.
    pub newly_eroded: bool,
}

/// Computes the Bažant characteristic crack-band length $h_e = V_e^{1/3}$ (m) from tetrahedron volume $V_e$.
///
/// With $h_e = V_e^{1/3}$, the crack-band area is $A_{\text{crack}} = V_e / h_e = h_e^2$ and
/// integrating $g_f = G_f / h_e$ over $V_e$ yields $\int_{V_e} g_f \, dV = G_f A_{\text{crack}}$
/// independent of mesh refinement.
#[inline]
pub fn characteristic_element_length(volume_m3: f64) -> f64 {
    volume_m3.max(1e-24).cbrt()
}

/// Returns the positive Kachanov effective stiffness factor $(1 - D_{\text{eff}})$ bounded below by `min_residual_stiffness`.
#[inline]
pub fn effective_stiffness_factor(damage: f64, min_residual_stiffness: f64) -> f64 {
    let floor = min_residual_stiffness.clamp(1e-6, 0.5);
    (1.0 - damage.clamp(0.0, 1.0)).max(floor)
}

/// Computes the 3 principal stresses $\sigma_1 \ge \sigma_2 \ge \sigma_3$ (Pa) and their corresponding
/// unit principal directions $[\mathbf{n}_1, \mathbf{n}_2, \mathbf{n}_3]$ from a Voigt Cauchy stress vector
/// $[\sigma_{xx}, \sigma_{yy}, \sigma_{zz}, \tau_{xy}, \tau_{yz}, \tau_{zx}]$.
pub fn principal_stresses_and_directions(stress_voigt: &Vector6<f64>) -> ([f64; 3], [[f64; 3]; 3]) {
    let mat: Matrix3<f64> = voigt_stress_to_matrix3(stress_voigt);
    let eig = mat.symmetric_eigen();
    let mut pairs: [(f64, Vector3<f64>); 3] = [
        (eig.eigenvalues[0], eig.eigenvectors.column(0).into_owned()),
        (eig.eigenvalues[1], eig.eigenvectors.column(1).into_owned()),
        (eig.eigenvalues[2], eig.eigenvectors.column(2).into_owned()),
    ];
    // Sort descending: σ_1 >= σ_2 >= σ_3
    pairs.sort_by(|a, b| b.0.total_cmp(&a.0));

    let mut vals = [0.0_f64; 3];
    let mut dirs = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for i in 0..3 {
        vals[i] = pairs[i].0;
        let n = pairs[i].1;
        let len = n.norm();
        if len > 1e-15 {
            dirs[i] = [n.x / len, n.y / len, n.z / len];
        }
    }
    (vals, dirs)
}

/// Computes stress triaxiality $\eta = \sigma_m / \sigma_{vM}$, where $\sigma_m = \frac{1}{3}(\sigma_{xx}+\sigma_{yy}+\sigma_{zz})$
/// is the hydrostatic mean stress and $\sigma_{vM}$ is the von Mises equivalent stress.
///
/// - Uniaxial tension: $\eta = +1/3$
/// - Pure shear: $\eta = 0$
/// - Uniaxial compression: $\eta = -1/3$
/// - Notched / root-fillet tension: $\eta > 1/3$
#[inline]
pub fn stress_triaxiality(stress_voigt: &Vector6<f64>) -> f64 {
    let vm = von_mises_from_voigt(stress_voigt);
    if !vm.is_finite() || vm <= 1e-12 {
        return 0.0;
    }
    let sigma_m = (stress_voigt[0] + stress_voigt[1] + stress_voigt[2]) / 3.0;
    (sigma_m / vm).clamp(-3.0, 3.0)
}

/// Evaluates the normalized Rice–Tracey / Johnson–Cook triaxiality ductility multiplier:
///
/// $$f_{\text{RT}}(\eta) = \exp\!\left(-d_3 \left(\eta - \frac{1}{3}\right)\right)$$
///
/// Normalized so $f_{\text{RT}}(1/3) = 1.0$ in uniaxial tension, decreasing at high tensile
/// triaxiality ($\eta > 1/3$, such as tooth root fillets) and increasing in low-triaxiality shear.
#[inline]
pub fn rice_tracey_ductility_factor(triaxiality: f64, exponent: f64) -> f64 {
    let shift = triaxiality - (1.0 / 3.0);
    (-exponent.max(0.0) * shift).exp().clamp(0.15, 4.0)
}

/// Evaluates irreversible continuum damage evolution for one finite element over an increment.
///
/// - `prev`: converged [`ElementDamageState`] from the previous increment/substep.
/// - `effective_stress_world`: undamaged (effective) Cauchy stress $\tilde{\boldsymbol{\sigma}}$ in world Voigt notation (Pa).
/// - `eq_plastic_strain`: total accumulated equivalent plastic strain $\bar{\varepsilon}_{p, n+1}$.
/// - `delta_eq_plastic_strain`: incremental equivalent plastic strain $\Delta \bar{\varepsilon}_p \ge 0$ in this step.
/// - `volume_m3`: undeformed element volume $V_e$ in m³.
/// - `params`: [`DamageMaterialParams`].
pub fn evaluate_element_damage(
    prev: &ElementDamageState,
    effective_stress_world: &Vector6<f64>,
    eq_plastic_strain: f64,
    delta_eq_plastic_strain: f64,
    volume_m3: f64,
    params: &DamageMaterialParams,
) -> DamageUpdateResult {
    let h_e = characteristic_element_length(volume_m3);

    if prev.eroded {
        return DamageUpdateResult {
            state: *prev,
            delta_damage: 0.0,
            delta_fracture_energy_j: 0.0,
            newly_eroded: false,
        };
    }

    let (principals, principal_dirs) = principal_stresses_and_directions(effective_stress_world);
    let sigma_1 = principals[0];
    let n_1 = principal_dirs[0];
    let vm = von_mises_from_voigt(effective_stress_world);
    let eta = stress_triaxiality(effective_stress_world);

    let mut next = *prev;
    next.characteristic_length_m = h_e;
    next.triaxiality = eta;
    if sigma_1 > next.max_principal_stress_pa {
        next.max_principal_stress_pa = sigma_1;
        next.crack_normal = n_1;
    }

    let g_f_vol = params.fracture_energy_j_m2 / h_e;

    match params.failure_mode {
        FailureMode::Ductile => {
            let dep = delta_eq_plastic_strain.max(0.0);

            // Physical guard: moderate plastic yielding strictly below ultimate tensile strength
            // (σ_y < σ_vM < σ_u) AND below reference onset strain must produce pure plastic
            // deformation without damage. Also suppress void growth under compression (η <= -1/3).
            if dep > 0.0 && eta > params.compression_triaxiality_cutoff + 1e-9 {
                let f_rt = rice_tracey_ductility_factor(eta, params.triaxiality_exponent);
                let eps_onset_eta = (params.damage_onset_strain * f_rt).max(1e-6);

                let prev_omega = prev.initiation_indicator;
                let mut post_onset_dep = 0.0_f64;

                if prev_omega >= 1.0 {
                    // Already in post-initiation softening regime
                    next.initiation_indicator = prev_omega + dep / eps_onset_eta;
                    post_onset_dep = dep;
                } else {
                    // Advance Johnson–Cook / Rice–Tracey damage initiation integral
                    let d_omega = dep / eps_onset_eta;
                    let new_omega = prev_omega + d_omega;
                    next.initiation_indicator = new_omega;

                    if new_omega >= 1.0 {
                        // Exact fraction of this step's plastic strain occurring after initiation
                        let frac_post = ((new_omega - 1.0) / d_omega.max(1e-15)).clamp(0.0, 1.0);
                        post_onset_dep = dep * frac_post;
                        next.onset_plastic_strain = (eq_plastic_strain - post_onset_dep).max(0.0);
                        next.onset_stress_pa =
                            vm.max(params.yield_strength_pa).max(1.0);
                        next.crack_normal = n_1;
                    }
                }

                if post_onset_dep > 0.0 {
                    let sigma_onset = next
                        .onset_stress_pa
                        .max(params.yield_strength_pa)
                        .max(1.0);
                    // Bažant crack-band softening strain to complete rupture (D = 1):
                    //   u_f = 2 * G_f / σ_onset   =>   Δε_band = u_f / h_e = 2 * G_f / (σ_onset * h_e)
                    // Ensures ∫_{V_e} (1/2 * σ_onset * Δε_band) dV = G_f * (V_e / h_e) = G_f * A_crack
                    let delta_eps_band = ((2.0 * params.fracture_energy_j_m2)
                        / (sigma_onset * h_e))
                        .max(1e-6);
                    let d_inc = post_onset_dep / delta_eps_band;
                    next.damage = (prev.damage + d_inc).clamp(prev.damage, 1.0);
                }
            }
        }
        FailureMode::Brittle => {
            // Rankine maximum principal tensile stress criterion: σ_1 >= σ_u
            // Pure compression (σ_1 <= 0) never initiates brittle tensile cleavage.
            let eps_0 = params.ultimate_strength_pa / params.youngs_modulus_pa;
            let kappa_trial = (sigma_1 / params.youngs_modulus_pa).max(0.0);
            let kappa = prev.initiation_indicator.max(kappa_trial);
            next.initiation_indicator = kappa;

            if kappa >= eps_0 {
                if prev.damage == 0.0 {
                    next.onset_stress_pa = params.ultimate_strength_pa;
                    next.crack_normal = n_1;
                }
                let g_0 = (params.ultimate_strength_pa * params.ultimate_strength_pa)
                    / (2.0 * params.youngs_modulus_pa);

                if g_f_vol <= g_0 * (1.0 + 1e-6) {
                    // Macroscopic brittle snap-back regime (e.g. borosilicate_glass with tiny G_f):
                    // Stored elastic strain energy already exceeds crack-band fracture energy,
                    // causing immediate Mode-I brittle cleavage.
                    next.damage = 1.0;
                } else {
                    // Exact Oliver / Bažant exponential softening whose total area under σ(ε)
                    // equals g_f = G_f / h_e identically:
                    //   D(κ) = 1 - (ε_0 / κ) * exp(-(κ - ε_0) / ε_f_band)
                    //   where ε_f_band = (g_f - g_0) / σ_u
                    let eps_f_band = ((g_f_vol - g_0) / params.ultimate_strength_pa).max(1e-9);
                    let ratio = (eps_0 / kappa.max(eps_0)).clamp(0.0, 1.0);
                    let d_target = 1.0 - ratio * (-(kappa - eps_0) / eps_f_band).exp();
                    next.damage = d_target.clamp(prev.damage, 1.0);
                }
            }
        }
    }

    let mut newly_eroded = false;
    if next.damage >= params.erode_threshold {
        next.damage = 1.0;
        next.eroded = true;
        newly_eroded = true;
    }

    let delta_damage = (next.damage - prev.damage).max(0.0);
    // Exact Bažant crack-band fracture energy dissipation increment:
    //   Δw_f = (G_f / h_e) * ΔD  =>  ΔW_f = V_e * Δw_f = G_f * (V_e / h_e) * ΔD = G_f * A_crack * ΔD
    let delta_wf = g_f_vol * delta_damage;
    next.fracture_energy_density_j_m3 = prev.fracture_energy_density_j_m3 + delta_wf;
    let delta_fracture_energy_j = delta_wf * volume_m3.max(0.0);

    DamageUpdateResult {
        state: next,
        delta_damage,
        delta_fracture_energy_j,
        newly_eroded,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mcp_cad_core::material::MaterialRegistry;

    #[test]
    fn test_principal_stresses_and_triaxiality_canonical_states() {
        // 1. Uniaxial tension: σ = [300, 0, 0, 0, 0, 0] MPa => σ_1 = 300 MPa, η = +1/3
        let s_tens = Vector6::new(300e6, 0.0, 0.0, 0.0, 0.0, 0.0);
        let (p, dirs) = principal_stresses_and_directions(&s_tens);
        assert!((p[0] - 300e6).abs() < 1e-3);
        assert!(p[1].abs() < 1e-3 && p[2].abs() < 1e-3);
        assert!((dirs[0][0].abs() - 1.0).abs() < 1e-12);
        assert!((stress_triaxiality(&s_tens) - 1.0 / 3.0).abs() < 1e-12);

        // 2. Uniaxial compression: η = -1/3
        let s_comp = Vector6::new(-300e6, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!((stress_triaxiality(&s_comp) + 1.0 / 3.0).abs() < 1e-12);

        // 3. Pure shear: τ_xy = 150 MPa => σ_1 = +150 MPa, σ_3 = -150 MPa, η = 0
        let s_shear = Vector6::new(0.0, 0.0, 0.0, 150e6, 0.0, 0.0);
        let (ps, _) = principal_stresses_and_directions(&s_shear);
        assert!((ps[0] - 150e6).abs() < 1e-3);
        assert!((ps[2] + 150e6).abs() < 1e-3);
        assert!(stress_triaxiality(&s_shear).abs() < 1e-12);
    }

    #[test]
    fn test_ductile_moderate_yielding_and_compression_produce_zero_damage() {
        let reg = MaterialRegistry::new();
        let brass = reg.get("brass_c360").unwrap();
        let params = DamageMaterialParams::from_material(brass).unwrap();

        let vol = (0.002_f64).powi(3);
        let st0 = ElementDamageState::intact(vol);

        // Moderate plastic yielding (σ_y = 310 MPa < σ_vM = 345 MPa < σ_u = 400 MPa)
        // Plastic strain = (345 - 310) / 850 = 0.041 < damage_onset_strain (~0.106)
        let s_mod = Vector6::new(345e6, 0.0, 0.0, 0.0, 0.0, 0.0);
        let res_mod = evaluate_element_damage(&st0, &s_mod, 0.041, 0.041, vol, &params);
        assert_eq!(res_mod.state.damage, 0.0);
        assert!(!res_mod.state.eroded);
        assert_eq!(res_mod.delta_fracture_energy_j, 0.0);

        // Severe uniaxial compression (η = -1/3 <= cutoff) must NOT initiate ductile void-growth fracture
        let s_comp = Vector6::new(-500e6, 0.0, 0.0, 0.0, 0.0, 0.0);
        let res_comp = evaluate_element_damage(&st0, &s_comp, 0.30, 0.30, vol, &params);
        assert_eq!(res_comp.state.damage, 0.0);
        assert!(!res_comp.state.eroded);
    }

    #[test]
    fn test_ductile_triaxiality_accelerates_notch_initiation_and_bazant_energy_is_mesh_independent()
    {
        let reg = MaterialRegistry::new();
        let brass = reg.get("brass_c360").unwrap();
        let params = DamageMaterialParams::from_material(brass).unwrap();

        // Compare uniaxial tension (η = 1/3) vs notched root-fillet triaxial tension (η = 1.0)
        let vol_coarse = (0.002_f64).powi(3);
        let vol_fine = (0.001_f64).powi(3); // h_e halved!

        let s_uniax = Vector6::new(400e6, 0.0, 0.0, 0.0, 0.0, 0.0);
        // σ = [666.7, 266.7, 266.7] MPa => σ_m = 400 MPa, σ_vM = 400 MPa => η = 1.0
        let s_notch = Vector6::new(666.666_667e6, 266.666_667e6, 266.666_667e6, 0.0, 0.0, 0.0);

        let r_uniax = evaluate_element_damage(
            &ElementDamageState::intact(vol_coarse),
            &s_uniax,
            0.06,
            0.06,
            vol_coarse,
            &params,
        );
        let r_notch = evaluate_element_damage(
            &ElementDamageState::intact(vol_coarse),
            &s_notch,
            0.06,
            0.06,
            vol_coarse,
            &params,
        );
        assert!(
            r_notch.state.initiation_indicator > r_uniax.state.initiation_indicator * 2.0,
            "Higher tensile triaxiality must accelerate ductile damage initiation ({} vs {})",
            r_notch.state.initiation_indicator,
            r_uniax.state.initiation_indicator
        );

        // Verify Bažant crack-band energy invariance across 2x mesh refinement:
        // Total fracture energy per crack area W_f / A_crack = G_f must be identical for h_e = 2 mm and h_e = 1 mm!
        let run_to_failure = |vol: f64| -> (f64, f64) {
            let mut st = ElementDamageState::intact(vol);
            let mut total_wf = 0.0;
            let mut ep = 0.0;
            let dep = 0.001;
            for _ in 0..500 {
                ep += dep;
                let res = evaluate_element_damage(&st, &s_uniax, ep, dep, vol, &params);
                st = res.state;
                total_wf += res.delta_fracture_energy_j;
                if st.eroded {
                    break;
                }
            }
            assert!(st.eroded, "Element must reach complete erosion");
            let h_e = characteristic_element_length(vol);
            let crack_area = vol / h_e;
            (total_wf / crack_area, ep)
        };

        let (gf_coarse, ep_fail_coarse) = run_to_failure(vol_coarse);
        let (gf_fine, ep_fail_fine) = run_to_failure(vol_fine);

        assert!(
            (gf_coarse - params.fracture_energy_j_m2).abs() / params.fracture_energy_j_m2 < 1e-9,
            "Coarse mesh G_f mismatch: {gf_coarse} vs {}",
            params.fracture_energy_j_m2
        );
        assert!(
            (gf_fine - params.fracture_energy_j_m2).abs() / params.fracture_energy_j_m2 < 1e-9,
            "Fine mesh G_f mismatch: {gf_fine} vs {}",
            params.fracture_energy_j_m2
        );
        assert!(
            ep_fail_fine > ep_fail_coarse,
            "Finer crack band must sustain higher local strain to dissipate the same G_f ({ep_fail_fine} vs {ep_fail_coarse})"
        );
    }

    #[test]
    fn test_brittle_borosilicate_glass_rankine_cleavage_and_compression_immunity() {
        let reg = MaterialRegistry::new();
        let glass = reg.get("borosilicate_glass").unwrap();
        let params = DamageMaterialParams::from_material(glass).unwrap();
        let vol = (0.002_f64).powi(3);
        let st0 = ElementDamageState::intact(vol);

        // Below σ_u = 120 MPa: strictly intact
        let s_sub = Vector6::new(115e6, 0.0, 0.0, 0.0, 0.0, 0.0);
        let r_sub = evaluate_element_damage(&st0, &s_sub, 0.0, 0.0, vol, &params);
        assert_eq!(r_sub.state.damage, 0.0);
        assert!(!r_sub.state.eroded);

        // Pure compression (-300 MPa): σ_1 = 0 < σ_u => intact
        let s_comp = Vector6::new(-300e6, -50e6, 0.0, 0.0, 0.0, 0.0);
        let r_comp = evaluate_element_damage(&st0, &s_comp, 0.0, 0.0, vol, &params);
        assert_eq!(r_comp.state.damage, 0.0);
        assert!(!r_comp.state.eroded);

        // Above σ_u = 120 MPa in tension along Y: immediate brittle cleavage with crack normal along Y
        let s_crit = Vector6::new(0.0, 125e6, 0.0, 0.0, 0.0, 0.0);
        let r_crit = evaluate_element_damage(&st0, &s_crit, 0.0, 0.0, vol, &params);
        assert!(r_crit.newly_eroded);
        assert_eq!(r_crit.state.damage, 1.0);
        assert!((r_crit.state.crack_normal[1].abs() - 1.0).abs() < 1e-12);
    }
}
