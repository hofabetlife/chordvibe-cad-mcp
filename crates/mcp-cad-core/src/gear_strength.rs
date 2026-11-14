//! Analytical gear tooth strength calculation according to Lewis equation and Hertz / AGMA pitting theory.
//!
//! Provides rapid analytical verification of bending fatigue and contact surface durability
//! for spur gears without requiring expensive boundary-fitted FEA.
//!
//! References:
//! - Lewis, W. (1892). "Investigation of the Strength of Gear Teeth". Proceedings of the Engineers' Club of Philadelphia.
//! - Budynas, R. G., & Nisbett, J. K. (2014). Shigley's Mechanical Engineering Design (10th ed.), Table 14-2.
//! - Barth Velocity Factor: AGMA 2001-D04 / Shigley Eq. (14-8) in SI units.
//! - Hertz, H. (1881). "Ueber die Berührung fester elastischer Körper" / ISO 6336-1 / DIN 3990.

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Manufacturing method determining the Barth dynamic velocity factor Kv.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum GearManufacturingMethod {
    /// Commercially cut, milled, or shaped gears: Kv = (6.1 + V) / 6.1 (SI units).
    #[serde(rename = "commercial_cut", alias = "milled", alias = "cut")]
    #[default]
    CommercialCut,
    /// Precision hobbed, shaved, or ground gears: Kv = (15.2 + V) / 15.2 (SI units).
    #[serde(rename = "precision_ground", alias = "ground", alias = "hobbed", alias = "precision")]
    PrecisionGround,
    /// Quasi-static or low-speed load: Kv = 1.0.
    #[serde(rename = "static")]
    Static,
}

impl GearManufacturingMethod {
    /// Parses a string into a manufacturing method.
    pub fn parse(s: &str) -> Self {
        match s.trim().to_ascii_lowercase().as_str() {
            "ground" | "hobbed" | "precision" | "precision_ground" | "shaved" => Self::PrecisionGround,
            "static" | "quasi_static" => Self::Static,
            _ => Self::CommercialCut,
        }
    }
}

/// Standard tabulated Lewis form factor Y (Shigley Table 14-2) for 20° full-depth involute teeth.
pub const LEWIS_Y_TABLE_20_DEG: &[(usize, f64)] = &[
    (12, 0.245),
    (13, 0.261),
    (14, 0.277),
    (15, 0.290),
    (16, 0.296),
    (17, 0.303),
    (18, 0.309),
    (19, 0.314),
    (20, 0.322),
    (21, 0.328),
    (22, 0.331),
    (24, 0.338),
    (26, 0.346),
    (28, 0.353),
    (30, 0.359),
    (34, 0.371),
    (38, 0.384),
    (43, 0.397),
    (50, 0.409),
    (60, 0.422),
    (75, 0.435),
    (100, 0.447),
    (150, 0.460),
    (300, 0.472),
    (400, 0.480),
];

/// Asymptotic Lewis form factor Y for an infinite rack (z -> inf) at 20° full depth.
pub const LEWIS_Y_RACK_20_DEG: f64 = 0.485;

/// Computes the Lewis form factor Y for a spur gear with `teeth` and `pressure_angle_deg`.
///
/// For standard 20° full-depth teeth, evaluates piecewise linear interpolation
/// from Shigley's Mechanical Engineering Design, Table 14-2.
pub fn lewis_form_factor(teeth: usize, pressure_angle_deg: f64) -> f64 {
    if teeth == 0 {
        return 0.0;
    }

    let y_20 = if teeth <= 12 {
        // Extrapolate below 12 teeth with steep root undercutting penalty
        (0.245 - (12 - teeth) as f64 * 0.016).max(0.120)
    } else if teeth >= 400 {
        // Asymptotic interpolation toward rack
        let inv_z = 1.0 / teeth as f64;
        let y = LEWIS_Y_RACK_20_DEG - 400.0 * (LEWIS_Y_RACK_20_DEG - 0.480) * inv_z;
        y.min(LEWIS_Y_RACK_20_DEG)
    } else {
        // Search adjacent interval
        let mut lower = LEWIS_Y_TABLE_20_DEG[0];
        let mut upper = LEWIS_Y_TABLE_20_DEG[1];
        for pair in LEWIS_Y_TABLE_20_DEG.windows(2) {
            if teeth >= pair[0].0 && teeth <= pair[1].0 {
                lower = pair[0];
                upper = pair[1];
                break;
            }
        }
        let frac = (teeth - lower.0) as f64 / (upper.0 - lower.0) as f64;
        lower.1 + frac * (upper.1 - lower.1)
    };

    // If pressure angle deviates from 20°, apply kinematic scaling factor
    if (pressure_angle_deg - 20.0).abs() > 1e-4 {
        let alpha_rad = pressure_angle_deg * PI / 180.0;
        let std_alpha_rad = 20.0 * PI / 180.0;
        let factor = (alpha_rad.sin() * alpha_rad.cos()) / (std_alpha_rad.sin() * std_alpha_rad.cos());
        (y_20 * factor).max(0.10)
    } else {
        y_20
    }
}

/// Dynamic velocity factor Kv in SI units using the Barth equation.
///
/// - Commercial cut/milled gears: `Kv = (6.1 + V) / 6.1`
/// - Precision ground/hobbed gears: `Kv = (15.2 + V) / 15.2`
/// - Static or V = 0: `Kv = 1.0`
pub fn barth_dynamic_factor_si(pitch_line_velocity_m_s: f64, method: GearManufacturingMethod) -> f64 {
    if pitch_line_velocity_m_s <= 1e-9 {
        return 1.0;
    }
    match method {
        GearManufacturingMethod::CommercialCut => (6.1 + pitch_line_velocity_m_s) / 6.1,
        GearManufacturingMethod::PrecisionGround => (15.2 + pitch_line_velocity_m_s) / 15.2,
        GearManufacturingMethod::Static => 1.0,
    }
}

/// Transmitted tangential force Wt (N) from torque T (N*m) and pitch diameter d (m).
///
/// `Wt = 2 * T / d`
pub fn transmitted_tangential_force(torque_nm: f64, pitch_diameter_m: f64) -> f64 {
    if pitch_diameter_m <= 1e-9 {
        0.0
    } else {
        2.0 * torque_nm / pitch_diameter_m
    }
}

/// Lewis tooth bending stress (Pa).
///
/// `sigma = (Kv * Wt) / (b * m * Y)`
///
/// where:
/// - `wt_n`: tangential force in Newtons
/// - `face_width_m`: face width b in meters
/// - `module_m`: gear module m in meters
/// - `lewis_y`: dimensionless Lewis form factor Y
/// - `kv`: dynamic velocity factor Kv
pub fn lewis_bending_stress_pa(
    wt_n: f64,
    face_width_m: f64,
    module_m: f64,
    lewis_y: f64,
    kv: f64,
) -> f64 {
    let denominator = face_width_m * module_m * lewis_y;
    if denominator <= 1e-18 {
        0.0
    } else {
        (kv * wt_n) / denominator
    }
}

/// Hertz elastic coefficient Cp (Pa^0.5) for contacting spur gear materials.
///
/// `Cp = sqrt( 1 / (pi * ((1 - nu1^2)/E1 + (1 - nu2^2)/E2)) )`
pub fn hertz_elastic_coefficient_pa(e1_pa: f64, nu1: f64, e2_pa: f64, nu2: f64) -> f64 {
    let compliance = (1.0 - nu1 * nu1) / e1_pa + (1.0 - nu2 * nu2) / e2_pa;
    if compliance <= 1e-30 {
        0.0
    } else {
        (1.0 / (PI * compliance)).sqrt()
    }
}

/// Hertz contact surface stress (pitting) at the pitch point (Pa).
///
/// `sigma_H = Cp * sqrt( (Kv * Wt) / (b * cos(alpha)) * (1/r1 + 1/r2) )`
///
/// where radii of profile curvature are:
/// - `r1 = 0.5 * d1 * sin(alpha)`
/// - `r2 = 0.5 * d2 * sin(alpha)`
pub fn hertz_contact_stress_pa(
    wt_n: f64,
    face_width_m: f64,
    pressure_angle_rad: f64,
    pitch_diameter_1_m: f64,
    pitch_diameter_2_m: f64,
    cp_pa: f64,
    kv: f64,
) -> f64 {
    let sin_alpha = pressure_angle_rad.sin();
    let cos_alpha = pressure_angle_rad.cos();
    if cos_alpha.abs() <= 1e-9 || sin_alpha.abs() <= 1e-9 || face_width_m <= 1e-9 {
        return 0.0;
    }

    let r1 = 0.5 * pitch_diameter_1_m * sin_alpha;
    let r2 = 0.5 * pitch_diameter_2_m * sin_alpha;

    if r1 <= 1e-12 || r2 <= 1e-12 {
        return 0.0;
    }

    let curvature_sum = (1.0 / r1) + (1.0 / r2);
    let load_intensity = (kv * wt_n) / (face_width_m * cos_alpha);

    if load_intensity < 0.0 {
        0.0
    } else {
        cp_pa * (load_intensity * curvature_sum).sqrt()
    }
}

/// Parameters for gear strength verification. All linear dimensions are in SI meters,
/// loads in Newtons and N*m, material moduli and strengths in Pa and MPa.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GearStrengthCalculationParams {
    /// Gear module m in meters.
    pub module_m: f64,
    /// Number of teeth z of the evaluated gear.
    pub teeth: usize,
    /// Face width b in meters.
    pub face_width_m: f64,
    /// Pressure angle alpha in degrees (standard 20.0°).
    pub pressure_angle_deg: f64,
    /// Transmitted torque T in N*m.
    pub torque_nm: f64,
    /// Optional rotational speed in RPM (used for pitch line velocity and Barth dynamic factor).
    pub speed_rpm: Option<f64>,
    /// Manufacturing quality/process.
    pub manufacturing_method: GearManufacturingMethod,
    /// Material allowable bending stress in MPa (or yield strength).
    pub allowable_bending_stress_mpa: Option<f64>,
    /// Material yield strength in MPa.
    pub yield_strength_mpa: Option<f64>,
    /// Material Young's modulus E in Pa.
    pub youngs_modulus_pa: f64,
    /// Material Poisson's ratio nu.
    pub poissons_ratio: f64,
    /// Number of teeth z2 of the mating gear (defaults to evaluated gear teeth if None).
    pub mating_teeth: Option<usize>,
    /// Mating gear Young's modulus in Pa (defaults to primary material if None).
    pub mating_youngs_modulus_pa: Option<f64>,
    /// Mating gear Poisson's ratio (defaults to primary material if None).
    pub mating_poissons_ratio: Option<f64>,
    /// Mating gear surface endurance/allowable contact stress in MPa.
    pub allowable_contact_stress_mpa: Option<f64>,
}

/// Detailed analytical gear strength verification report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GearStrengthReport {
    /// Evaluated gear pitch diameter d in millimeters (mm).
    pub pitch_diameter_mm: f64,
    /// Transmitted tangential force Wt at pitch circle in Newtons (N).
    pub tangential_force_n: f64,
    /// Pitch line velocity V in meters per second (m/s).
    pub pitch_line_velocity_m_s: f64,
    /// Barth dynamic velocity factor Kv.
    pub dynamic_factor_kv: f64,
    /// Dimensionless Lewis tooth form factor Y (Shigley Table 14-2).
    pub lewis_form_factor_y: f64,
    /// Tooth root bending stress according to Lewis equation in Megapascals (MPa).
    pub bending_stress_mpa: f64,
    /// Factor of safety against tooth root bending failure (allowable / sigma_b).
    pub bending_safety_factor: Option<f64>,
    /// Mating gear pitch diameter d2 in millimeters (mm).
    pub mating_pitch_diameter_mm: Option<f64>,
    /// Curvature radius r1 of evaluated gear profile at pitch point in millimeters (mm).
    pub curvature_radius_1_mm: Option<f64>,
    /// Curvature radius r2 of mating gear profile at pitch point in millimeters (mm).
    pub curvature_radius_2_mm: Option<f64>,
    /// Hertz elasticity factor Cp in (MPa)^0.5.
    pub elastic_coefficient_cp_mpa_half: Option<f64>,
    /// Hertz contact stress (pitting) at pitch line in Megapascals (MPa).
    pub contact_stress_mpa: Option<f64>,
    /// Factor of safety against surface contact pitting (allowable / sigma_H).
    pub contact_safety_factor: Option<f64>,
    /// Engineering warnings if parameters fall outside standard application domains.
    pub warnings: Vec<String>,
    /// Explicit engineering assumptions made during the analysis.
    pub assumptions: Vec<String>,
    /// Academic and standard references.
    pub references: Vec<String>,
}

/// Evaluates tooth bending and surface contact durability for a spur gear.
pub fn evaluate_gear_strength(params: &GearStrengthCalculationParams) -> GearStrengthReport {
    let mut warnings = Vec::new();
    let d1_m = params.module_m * params.teeth as f64;
    let d1_mm = d1_m * 1000.0;

    // Transmitted tangential force
    let wt_n = transmitted_tangential_force(params.torque_nm, d1_m);

    // Pitch line velocity
    let v_m_s = if let Some(rpm) = params.speed_rpm {
        (PI * d1_m * rpm.abs()) / 60.0
    } else {
        0.0
    };

    // Dynamic factor Kv
    let kv = if params.speed_rpm.is_some() {
        barth_dynamic_factor_si(v_m_s, params.manufacturing_method)
    } else {
        1.0
    };

    // Lewis Form Factor Y
    let y = lewis_form_factor(params.teeth, params.pressure_angle_deg);

    // Bending stress
    let sigma_b_pa = lewis_bending_stress_pa(wt_n, params.face_width_m, params.module_m, y, kv);
    let sigma_b_mpa = sigma_b_pa / 1.0e6;

    // Bending safety factor
    let allowable_b_mpa = params
        .allowable_bending_stress_mpa
        .or(params.yield_strength_mpa);
    let bending_sf = allowable_b_mpa.map(|allowable| allowable / sigma_b_mpa.max(1e-6));

    // Hertz contact stress
    let mating_z = params.mating_teeth.unwrap_or(params.teeth);
    let d2_m = params.module_m * mating_z as f64;
    let d2_mm = d2_m * 1000.0;

    let e2_pa = params.mating_youngs_modulus_pa.unwrap_or(params.youngs_modulus_pa);
    let nu2 = params.mating_poissons_ratio.unwrap_or(params.poissons_ratio);

    let cp_pa = hertz_elastic_coefficient_pa(params.youngs_modulus_pa, params.poissons_ratio, e2_pa, nu2);
    // Cp in (MPa)^0.5 = Cp_pa / 1000.0 because sqrt(1e6) = 1000.
    let cp_mpa_half = cp_pa / 1000.0;

    let alpha_rad = params.pressure_angle_deg * PI / 180.0;
    let r1_m = 0.5 * d1_m * alpha_rad.sin();
    let r2_m = 0.5 * d2_m * alpha_rad.sin();

    let sigma_h_pa = hertz_contact_stress_pa(
        wt_n,
        params.face_width_m,
        alpha_rad,
        d1_m,
        d2_m,
        cp_pa,
        kv,
    );
    let sigma_h_mpa = sigma_h_pa / 1.0e6;

    let contact_sf = params
        .allowable_contact_stress_mpa
        .map(|allowable| allowable / sigma_h_mpa.max(1e-6));

    // Sanity checks & warnings
    if params.teeth < 12 {
        warnings.push(format!(
            "UNDERCUT_SEVERE: Teeth count z = {} < 12 leads to severe undercutting and base thinning without positive profile shift.",
            params.teeth
        ));
    } else if params.teeth < 17 {
        warnings.push(format!(
            "UNDERCUT_RISK: Teeth count z = {} < 17 has theoretical undercutting risk for 20° full-depth standard teeth.",
            params.teeth
        ));
    }

    if params.module_m > 1e-9 {
        let b_over_m = params.face_width_m / params.module_m;
        if b_over_m < 6.0 {
            warnings.push(format!(
                "FACE_WIDTH_TOO_NARROW: Face width ratio b/m = {:.1} < 6.0: gear is very narrow, sensitive to axial misalignment.",
                b_over_m
            ));
        } else if b_over_m > 16.0 {
            warnings.push(format!(
                "FACE_WIDTH_TOO_WIDE: Face width ratio b/m = {:.1} > 16.0: wide face width risks tooth edge loading from shaft deflection.",
                b_over_m
            ));
        }
    }

    if (params.pressure_angle_deg - 20.0).abs() > 0.5 {
        warnings.push(format!(
            "NON_STANDARD_PRESSURE_ANGLE: Pressure angle alpha = {:.1}° differs from standard 20.0° full depth.",
            params.pressure_angle_deg
        ));
    }

    if let Some(sf) = bending_sf {
        if sf < 1.0 {
            warnings.push(format!(
                "BENDING_SAFETY_FACTOR_CRITICAL: Factor of safety FoS_b = {:.2} < 1.0 indicates imminent tooth root fatigue/bending failure.",
                sf
            ));
        }
    }

    if let Some(sf) = contact_sf {
        if sf < 1.0 {
            warnings.push(format!(
                "CONTACT_SAFETY_FACTOR_CRITICAL: Factor of safety FoS_H = {:.2} < 1.0 indicates surface pitting/wear under contact stress.",
                sf
            ));
        }
    }

    let assumptions = vec![
        "Lewis bending formulation assumes full transmitted tangential load Wt is applied at the tip of a single tooth without load sharing.".to_string(),
        "Barth dynamic velocity factor Kv assumes commercially cut/milled spur gears in SI units: Kv = (6.1 + V) / 6.1.".to_string(),
        "Hertz contact stress assumes external spur gears in pitch point contact with cylindrical profile curvature r_i = 0.5 * d_i * sin(alpha).".to_string(),
        "ISO/AGMA nominal factors are assumed (rim thickness KB = 1.0, load distribution Km = 1.0, stress concentration Kf = 1.0).".to_string(),
    ];

    let references = vec![
        "Lewis, W. (1892). 'Investigation of the Strength of Gear Teeth'. Proceedings of the Engineers' Club of Philadelphia.".to_string(),
        "Budynas, R. G., & Nisbett, J. K. (2014). Shigley's Mechanical Engineering Design (10th ed.), Chapter 14: Gears - Mechanical Aspects, Table 14-2 (Lewis Form Factor Y).".to_string(),
        "Barth Velocity Factor: AGMA 2001-D04 / Shigley Eq. (14-8) in SI units.".to_string(),
        "ISO 6336-1:2006 / DIN 3990: Calculation of load capacity of spur and helical gears.".to_string(),
    ];

    GearStrengthReport {
        pitch_diameter_mm: d1_mm,
        tangential_force_n: wt_n,
        pitch_line_velocity_m_s: v_m_s,
        dynamic_factor_kv: kv,
        lewis_form_factor_y: y,
        bending_stress_mpa: sigma_b_mpa,
        bending_safety_factor: bending_sf,
        mating_pitch_diameter_mm: Some(d2_mm),
        curvature_radius_1_mm: Some(r1_m * 1000.0),
        curvature_radius_2_mm: Some(r2_m * 1000.0),
        elastic_coefficient_cp_mpa_half: Some(cp_mpa_half),
        contact_stress_mpa: Some(sigma_h_mpa),
        contact_safety_factor: contact_sf,
        warnings,
        assumptions,
        references,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lewis_form_factor_table_points() {
        assert!((lewis_form_factor(17, 20.0) - 0.303).abs() < 1e-4);
        assert!((lewis_form_factor(20, 20.0) - 0.322).abs() < 1e-4);
        assert!((lewis_form_factor(12, 20.0) - 0.245).abs() < 1e-4);
        assert!((lewis_form_factor(400, 20.0) - 0.480).abs() < 1e-4);
        // Linear interpolation between 20 (0.322) and 21 (0.328):
        assert!((lewis_form_factor(21, 20.0) - 0.328).abs() < 1e-4);
    }

    #[test]
    fn test_acceptance_criteria_manual_calculation() {
        // Acceptance criteria from specification:
        // m = 3 mm, z = 20, b = 30 mm, T = 100 N*m, Kv = 1:
        // d = 60 mm, Wt = 3333.3 N, at Y = 0.322 sigma = Wt / (b * m * Y) ≈ 115.0 MPa.
        let params = GearStrengthCalculationParams {
            module_m: 0.003,
            teeth: 20,
            face_width_m: 0.030,
            pressure_angle_deg: 20.0,
            torque_nm: 100.0,
            speed_rpm: None,
            manufacturing_method: GearManufacturingMethod::Static,
            allowable_bending_stress_mpa: Some(250.0), // structural steel
            yield_strength_mpa: Some(250.0),
            youngs_modulus_pa: 2.05e11,
            poissons_ratio: 0.29,
            mating_teeth: Some(20),
            mating_youngs_modulus_pa: Some(2.05e11),
            mating_poissons_ratio: Some(0.29),
            allowable_contact_stress_mpa: None,
        };

        let report = evaluate_gear_strength(&params);

        assert!((report.pitch_diameter_mm - 60.0).abs() < 1e-3);
        assert!((report.tangential_force_n - 3333.333).abs() < 1e-2);
        assert!((report.lewis_form_factor_y - 0.322).abs() < 1e-4);
        assert_eq!(report.dynamic_factor_kv, 1.0);
        assert!((report.bending_stress_mpa - 115.02).abs() < 0.1);

        // Bending safety factor = 250 / 115.02 ≈ 2.17
        let sf_b = report.bending_safety_factor.unwrap();
        assert!((sf_b - 2.173).abs() < 0.01);

        // Contact stress manual calculation verification:
        // Cp = sqrt(1 / (pi * 2 * (1 - 0.29^2) / 2.05e11)) ≈ 1.8874e5 Pa^0.5 ≈ 188.74 MPa^0.5
        // r1 = r2 = 0.5 * 0.06 * sin(20°) ≈ 0.0102606 m
        // 1/r1 + 1/r2 ≈ 194.92 m^-1
        // Wt / (b * cos(20°)) = 3333.333 / (0.03 * 0.939693) ≈ 118241.6 N/m
        // sigma_H = 1.8874e5 * sqrt(118241.6 * 194.92) ≈ 9.061e8 Pa = 906.1 MPa
        let sigma_h = report.contact_stress_mpa.unwrap();
        assert!((sigma_h - 906.1).abs() < 1.0);
    }

    #[test]
    fn test_dynamic_barth_factor_and_velocity() {
        let params = GearStrengthCalculationParams {
            module_m: 0.003,
            teeth: 20,
            face_width_m: 0.030,
            pressure_angle_deg: 20.0,
            torque_nm: 100.0,
            speed_rpm: Some(1500.0),
            manufacturing_method: GearManufacturingMethod::CommercialCut,
            allowable_bending_stress_mpa: Some(250.0),
            yield_strength_mpa: Some(250.0),
            youngs_modulus_pa: 2.05e11,
            poissons_ratio: 0.29,
            mating_teeth: Some(40),
            mating_youngs_modulus_pa: Some(2.05e11),
            mating_poissons_ratio: Some(0.29),
            allowable_contact_stress_mpa: Some(1500.0),
        };

        let report = evaluate_gear_strength(&params);

        // V = pi * 0.06 * 1500 / 60 ≈ 4.7124 m/s
        assert!((report.pitch_line_velocity_m_s - 4.7124).abs() < 1e-3);
        // Kv = (6.1 + 4.7124) / 6.1 ≈ 1.7725
        assert!((report.dynamic_factor_kv - 1.7725).abs() < 1e-3);
        // sigma_b = 1.7725 * 115.02 ≈ 203.88 MPa
        assert!((report.bending_stress_mpa - 203.88).abs() < 0.5);

        // For mating gear z2 = 40 (2:1 gear ratio):
        // r2 = 2 * r1, 1/r1 + 1/r2 = 1.5 / r1
        // Dynamic contact stress scales by sqrt(Kv):
        let sigma_h = report.contact_stress_mpa.unwrap();
        // 784.7 * sqrt(1.7725) ≈ 1044.8 MPa
        assert!((sigma_h - 1044.8).abs() < 2.0);
        assert!(report.contact_safety_factor.unwrap() > 1.4);
    }

    #[test]
    fn test_undercut_and_face_width_warnings() {
        let params = GearStrengthCalculationParams {
            module_m: 0.002,
            teeth: 10, // Severe undercut
            face_width_m: 0.008, // b/m = 4.0 (< 6.0)
            pressure_angle_deg: 20.0,
            torque_nm: 20.0,
            speed_rpm: None,
            manufacturing_method: GearManufacturingMethod::CommercialCut,
            allowable_bending_stress_mpa: Some(100.0),
            yield_strength_mpa: Some(100.0),
            youngs_modulus_pa: 2.05e11,
            poissons_ratio: 0.29,
            mating_teeth: None,
            mating_youngs_modulus_pa: None,
            mating_poissons_ratio: None,
            allowable_contact_stress_mpa: None,
        };

        let report = evaluate_gear_strength(&params);
        assert!(report.warnings.iter().any(|w| w.contains("UNDERCUT_SEVERE")));
        assert!(report.warnings.iter().any(|w| w.contains("FACE_WIDTH_TOO_NARROW")));
    }
}
