//! Material definitions and registry for physical, elastoplastic, fracture & FEA simulations.

use crate::error::CadError;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Constitutive failure mode determining fracture criterion in Phase 5 nonlinear FEM.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum FailureMode {
    /// Ductile failure controlled by J2 plasticity and critical equivalent plastic strain (ε_p >= ε_f).
    #[default]
    Ductile,
    /// Brittle fracture controlled by maximum principal tensile stress (Rankine: σ_1 >= σ_u).
    Brittle,
}

impl FailureMode {
    /// Parses a string representation (`"ductile"` | `"brittle"`, case-insensitive).
    pub fn parse(value: &str) -> Result<Self, CadError> {
        match value.trim().to_ascii_lowercase().as_str() {
            "ductile" => Ok(Self::Ductile),
            "brittle" => Ok(Self::Brittle),
            other => Err(CadError::InvalidParameter(format!(
                "Invalid material failure_mode '{}': expected 'ductile' or 'brittle'",
                other
            ))),
        }
    }

    /// Returns canonical snake_case string label.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Ductile => "ductile",
            Self::Brittle => "brittle",
        }
    }
}

/// Computes physically consistent default nonlinear plasticity/fracture parameters
/// when only linear-elastic properties are supplied (e.g. legacy project files or `Material::new`).
fn derived_nonlinear_defaults(
    youngs_modulus_gpa: f64,
    yield_strength_mpa: f64,
    failure_mode: FailureMode,
) -> (f64, f64, f64, f64, f64) {
    let yield_pa = yield_strength_mpa.max(0.0) * 1.0e6;
    let youngs_pa = youngs_modulus_gpa.max(0.0) * 1.0e9;

    match failure_mode {
        FailureMode::Ductile => {
            let ultimate_strength_pa = yield_pa * 1.25;
            let hardening_modulus_pa = youngs_pa * 0.01;
            let fracture_strain = 0.15;
            let fracture_energy_j_m2 = 15_000.0;
            let friction_coeff = 0.25;
            (
                ultimate_strength_pa,
                hardening_modulus_pa,
                fracture_strain,
                fracture_energy_j_m2,
                friction_coeff,
            )
        }
        FailureMode::Brittle => {
            let ultimate_strength_pa = yield_pa;
            let hardening_modulus_pa = 0.0;
            let fracture_strain = 0.002;
            let fracture_energy_j_m2 = 50.0;
            let friction_coeff = 0.20;
            (
                ultimate_strength_pa,
                hardening_modulus_pa,
                fracture_strain,
                fracture_energy_j_m2,
                friction_coeff,
            )
        }
    }
}

/// Raw serde helper enabling backward-compatible deserialization of legacy material JSON
/// that omits Phase 5 nonlinear plasticity and fracture fields.
#[derive(Deserialize)]
struct MaterialSerdeRaw {
    name: String,
    youngs_modulus_gpa: f64,
    poissons_ratio: f64,
    yield_strength_mpa: f64,
    density_kg_m3: f64,
    #[serde(default)]
    description: String,
    #[serde(default)]
    ultimate_strength_pa: Option<f64>,
    #[serde(default)]
    hardening_modulus_pa: Option<f64>,
    #[serde(default)]
    fracture_strain: Option<f64>,
    #[serde(default)]
    fracture_energy_j_m2: Option<f64>,
    #[serde(default)]
    friction_coeff: Option<f64>,
    #[serde(default)]
    failure_mode: Option<FailureMode>,
}

/// Material properties defining linear-elastic, J2-elastoplastic, contact friction,
/// and continuum fracture behavior.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(from = "MaterialSerdeRaw")]
pub struct Material {
    /// Identifier or common name of the material.
    pub name: String,
    /// Young's modulus in Gigapascals (GPa, must satisfy E > 0).
    pub youngs_modulus_gpa: f64,
    /// Poisson's ratio (dimensionless, strictly in (0.0, 0.5)).
    pub poissons_ratio: f64,
    /// Yield strength σ_y in Megapascals (MPa, must satisfy σ_y > 0).
    pub yield_strength_mpa: f64,
    /// Mass density ρ in kilograms per cubic meter (kg/m³, must satisfy ρ > 0).
    pub density_kg_m3: f64,
    /// Human-readable description and intended applications.
    pub description: String,
    /// Ultimate tensile strength σ_u in Pascals (Pa, must satisfy σ_u >= σ_y > 0).
    pub ultimate_strength_pa: f64,
    /// Isotropic linear hardening modulus H in Pascals (Pa, must satisfy H >= 0).
    pub hardening_modulus_pa: f64,
    /// Critical equivalent plastic strain to fracture ε_f (dimensionless, must satisfy ε_f > 0).
    pub fracture_strain: f64,
    /// Specific fracture energy G_f in Joules per square meter (J/m², must satisfy G_f > 0).
    pub fracture_energy_j_m2: f64,
    /// Dry sliding Coulomb friction coefficient μ (dimensionless, must satisfy μ >= 0).
    pub friction_coeff: f64,
    /// Constitutive failure mode (`ductile` | `brittle`).
    pub failure_mode: FailureMode,
}

/// Alias matching the Phase 5 specification and TypeScript `MaterialSpec` contract.
pub type MaterialSpec = Material;

impl From<MaterialSerdeRaw> for Material {
    fn from(raw: MaterialSerdeRaw) -> Self {
        let failure_mode = raw.failure_mode.unwrap_or(FailureMode::Ductile);
        let (
            default_ultimate_pa,
            default_hardening_pa,
            default_fracture_strain,
            default_fracture_energy,
            default_friction,
        ) = derived_nonlinear_defaults(raw.youngs_modulus_gpa, raw.yield_strength_mpa, failure_mode);

        Self {
            name: raw.name,
            youngs_modulus_gpa: raw.youngs_modulus_gpa,
            poissons_ratio: raw.poissons_ratio,
            yield_strength_mpa: raw.yield_strength_mpa,
            density_kg_m3: raw.density_kg_m3,
            description: raw.description,
            ultimate_strength_pa: raw.ultimate_strength_pa.unwrap_or(default_ultimate_pa),
            hardening_modulus_pa: raw.hardening_modulus_pa.unwrap_or(default_hardening_pa),
            fracture_strain: raw.fracture_strain.unwrap_or(default_fracture_strain),
            fracture_energy_j_m2: raw.fracture_energy_j_m2.unwrap_or(default_fracture_energy),
            friction_coeff: raw.friction_coeff.unwrap_or(default_friction),
            failure_mode,
        }
    }
}

impl Material {
    /// Creates a new `Material` with linear-elastic properties and physically consistent
    /// default ductile plasticity, fracture, and friction parameters.
    pub fn new(
        name: impl Into<String>,
        youngs_modulus_gpa: f64,
        poissons_ratio: f64,
        yield_strength_mpa: f64,
        density_kg_m3: f64,
        description: impl Into<String>,
    ) -> Self {
        let failure_mode = FailureMode::Ductile;
        let (
            ultimate_strength_pa,
            hardening_modulus_pa,
            fracture_strain,
            fracture_energy_j_m2,
            friction_coeff,
        ) = derived_nonlinear_defaults(youngs_modulus_gpa, yield_strength_mpa, failure_mode);

        Self {
            name: name.into(),
            youngs_modulus_gpa,
            poissons_ratio,
            yield_strength_mpa,
            density_kg_m3,
            description: description.into(),
            ultimate_strength_pa,
            hardening_modulus_pa,
            fracture_strain,
            fracture_energy_j_m2,
            friction_coeff,
            failure_mode,
        }
    }

    /// Overrides the Phase 5 nonlinear plasticity, fracture, and contact friction parameters.
    pub fn with_nonlinear_properties(
        mut self,
        ultimate_strength_pa: f64,
        hardening_modulus_pa: f64,
        fracture_strain: f64,
        fracture_energy_j_m2: f64,
        friction_coeff: f64,
        failure_mode: FailureMode,
    ) -> Self {
        self.ultimate_strength_pa = ultimate_strength_pa;
        self.hardening_modulus_pa = hardening_modulus_pa;
        self.fracture_strain = fracture_strain;
        self.fracture_energy_j_m2 = fracture_energy_j_m2;
        self.friction_coeff = friction_coeff;
        self.failure_mode = failure_mode;
        self
    }

    /// Validates all physical constitutive invariants:
    /// - `name` non-empty
    /// - `density_kg_m3 > 0`
    /// - `youngs_modulus_gpa > 0`
    /// - `0 < poissons_ratio < 0.5`
    /// - `yield_strength_mpa > 0`
    /// - `ultimate_strength_pa >= yield_strength_pa > 0`
    /// - `hardening_modulus_pa >= 0`
    /// - `fracture_strain > 0`
    /// - `fracture_energy_j_m2 > 0`
    /// - `friction_coeff >= 0`
    pub fn validate(&self) -> Result<(), CadError> {
        if self.name.trim().is_empty() {
            return Err(CadError::InvalidParameter(
                "Material 'name' cannot be empty".to_string(),
            ));
        }
        if !self.density_kg_m3.is_finite() || self.density_kg_m3 <= 0.0 {
            return Err(CadError::InvalidParameter(format!(
                "Material '{}' density_kg_m3 must be finite and > 0 (got {})",
                self.name, self.density_kg_m3
            )));
        }
        if !self.youngs_modulus_gpa.is_finite() || self.youngs_modulus_gpa <= 0.0 {
            return Err(CadError::InvalidParameter(format!(
                "Material '{}' youngs_modulus_gpa must be finite and > 0 (got {})",
                self.name, self.youngs_modulus_gpa
            )));
        }
        if !self.poissons_ratio.is_finite()
            || self.poissons_ratio <= 0.0
            || self.poissons_ratio >= 0.5
        {
            return Err(CadError::InvalidParameter(format!(
                "Material '{}' poissons_ratio must be finite and strictly in (0, 0.5) (got {})",
                self.name, self.poissons_ratio
            )));
        }
        if !self.yield_strength_mpa.is_finite() || self.yield_strength_mpa <= 0.0 {
            return Err(CadError::InvalidParameter(format!(
                "Material '{}' yield_strength_mpa must be finite and > 0 (got {})",
                self.name, self.yield_strength_mpa
            )));
        }
        let yield_pa = self.yield_strength_pa();
        if !self.ultimate_strength_pa.is_finite() || self.ultimate_strength_pa < yield_pa {
            return Err(CadError::InvalidParameter(format!(
                "Material '{}' ultimate_strength_pa ({}) must be finite and >= yield_strength_pa ({})",
                self.name, self.ultimate_strength_pa, yield_pa
            )));
        }
        if !self.hardening_modulus_pa.is_finite() || self.hardening_modulus_pa < 0.0 {
            return Err(CadError::InvalidParameter(format!(
                "Material '{}' hardening_modulus_pa must be finite and >= 0 (got {})",
                self.name, self.hardening_modulus_pa
            )));
        }
        if !self.fracture_strain.is_finite() || self.fracture_strain <= 0.0 {
            return Err(CadError::InvalidParameter(format!(
                "Material '{}' fracture_strain must be finite and > 0 (got {})",
                self.name, self.fracture_strain
            )));
        }
        if !self.fracture_energy_j_m2.is_finite() || self.fracture_energy_j_m2 <= 0.0 {
            return Err(CadError::InvalidParameter(format!(
                "Material '{}' fracture_energy_j_m2 must be finite and > 0 (got {})",
                self.name, self.fracture_energy_j_m2
            )));
        }
        if !self.friction_coeff.is_finite() || self.friction_coeff < 0.0 {
            return Err(CadError::InvalidParameter(format!(
                "Material '{}' friction_coeff must be finite and >= 0 (got {})",
                self.name, self.friction_coeff
            )));
        }
        Ok(())
    }

    /// Returns Young's Modulus in Pascals (N/m²).
    #[inline]
    pub fn youngs_modulus_pa(&self) -> f64 {
        self.youngs_modulus_gpa * 1.0e9
    }

    /// Returns Yield Strength in Pascals (N/m²).
    #[inline]
    pub fn yield_strength_pa(&self) -> f64 {
        self.yield_strength_mpa * 1.0e6
    }

    /// Returns Ultimate Tensile Strength in Megapascals (MPa).
    #[inline]
    pub fn ultimate_strength_mpa(&self) -> f64 {
        self.ultimate_strength_pa / 1.0e6
    }

    /// Returns isotropic hardening modulus H in Gigapascals (GPa).
    #[inline]
    pub fn hardening_modulus_gpa(&self) -> f64 {
        self.hardening_modulus_pa / 1.0e9
    }

    /// Returns Lamé shear modulus G = E / (2(1 + ν)) in Pascals (Pa).
    #[inline]
    pub fn shear_modulus_pa(&self) -> f64 {
        self.youngs_modulus_pa() / (2.0 * (1.0 + self.poissons_ratio))
    }

    /// Returns bulk modulus K = E / (3(1 - 2ν)) in Pascals (Pa).
    #[inline]
    pub fn bulk_modulus_pa(&self) -> f64 {
        self.youngs_modulus_pa() / (3.0 * (1.0 - 2.0 * self.poissons_ratio))
    }

    /// Returns elastoplastic tangent modulus E_t = (E * H) / (E + H) in Pascals (Pa).
    #[inline]
    pub fn tangent_modulus_pa(&self) -> f64 {
        if self.hardening_modulus_pa <= 0.0 {
            return 0.0;
        }
        let e = self.youngs_modulus_pa();
        (e * self.hardening_modulus_pa) / (e + self.hardening_modulus_pa)
    }

    /// Returns density in g/cm³ (useful for 3D printing and inspection).
    #[inline]
    pub fn density_g_cm3(&self) -> f64 {
        self.density_kg_m3 / 1000.0
    }
}

/// Registry of known materials, including engineering presets and custom additions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialRegistry {
    materials: HashMap<String, Material>,
}

impl Default for MaterialRegistry {
    fn default() -> Self {
        let mut reg = Self {
            materials: HashMap::new(),
        };
        reg.register_presets();
        reg
    }
}

impl MaterialRegistry {
    /// Creates a new registry preloaded with standard engineering materials.
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers standard engineering preset materials with handbook elasticity,
    /// J2 plasticity hardening, fracture energy, and friction constants (ADR-005).
    fn register_presets(&mut self) {
        self.register(
            Material::new(
                "steel_structural_aisi1020",
                205.0,
                0.29,
                250.0,
                7850.0,
                "Structural steel AISI 1020",
            )
            .with_nonlinear_properties(
                420.0e6,
                1.20e9,
                0.22,
                28_000.0,
                0.25,
                FailureMode::Ductile,
            ),
        );

        self.register(
            Material::new(
                "steel_carbon_aisi1045",
                205.0,
                0.29,
                350.0,
                7850.0,
                "Medium-carbon steel AISI 1045",
            )
            .with_nonlinear_properties(
                585.0e6,
                1.80e9,
                0.16,
                28_000.0,
                0.25,
                FailureMode::Ductile,
            ),
        );

        self.register(
            Material::new(
                "steel_high_strength_4140",
                210.0,
                0.30,
                655.0,
                7850.0,
                "High-strength alloy steel AISI 4140",
            )
            .with_nonlinear_properties(
                1020.0e6,
                2.60e9,
                0.14,
                35_000.0,
                0.22,
                FailureMode::Ductile,
            ),
        );

        self.register(
            Material::new(
                "aluminum_6061_t6",
                68.9,
                0.33,
                276.0,
                2700.0,
                "Aerospace / structural aluminum alloy 6061-T6",
            )
            .with_nonlinear_properties(
                310.0e6,
                0.45e9,
                0.12,
                11_000.0,
                0.30,
                FailureMode::Ductile,
            ),
        );

        self.register(
            Material::new(
                "titanium_grade5_ti6al4v",
                114.0,
                0.34,
                880.0,
                4430.0,
                "Titanium Grade 5 (Ti-6Al-4V) high-strength alloy",
            )
            .with_nonlinear_properties(
                950.0e6,
                0.90e9,
                0.10,
                44_000.0,
                0.32,
                FailureMode::Ductile,
            ),
        );

        self.register(
            Material::new(
                "brass_c360",
                100.0,
                0.34,
                310.0,
                8500.0,
                "Free-cutting brass C360",
            )
            .with_nonlinear_properties(
                400.0e6,
                0.85e9,
                0.15,
                14_000.0,
                0.20,
                FailureMode::Ductile,
            ),
        );

        self.register(
            Material::new(
                "pla_filament_3dprint",
                3.5,
                0.36,
                60.0,
                1240.0,
                "Polylactic acid (PLA) 3D printing filament",
            )
            .with_nonlinear_properties(
                65.0e6,
                0.18e9,
                0.04,
                3_000.0,
                0.35,
                FailureMode::Brittle,
            ),
        );

        self.register(
            Material::new(
                "petg_filament_3dprint",
                2.1,
                0.38,
                50.0,
                1270.0,
                "Polyethylene terephthalate glycol (PETG) filament",
            )
            .with_nonlinear_properties(
                55.0e6,
                0.12e9,
                0.18,
                7_200.0,
                0.32,
                FailureMode::Ductile,
            ),
        );

        self.register(
            Material::new(
                "nylon_pa12",
                1.8,
                0.40,
                45.0,
                1010.0,
                "Polyamide 12 (Nylon PA12) engineering thermoplastic",
            )
            .with_nonlinear_properties(
                55.0e6,
                0.15e9,
                0.25,
                9_500.0,
                0.20,
                FailureMode::Ductile,
            ),
        );

        self.register(
            Material::new(
                "borosilicate_glass",
                64.0,
                0.20,
                120.0,
                2500.0,
                "Borosilicate technical glass (Pyrex / Duran E27)",
            )
            .with_nonlinear_properties(
                120.0e6,
                0.0,
                0.002,
                8.5,
                0.18,
                FailureMode::Brittle,
            ),
        );
    }

    /// Resolves common material identifier aliases (such as `steel_1045`, `steel_4140`,
    /// `alu_6061`, `Al6061-T6`, `ti_6al_4v`, `Ti-6Al-4V`, `pla_3d`, `petg_3d`, `nylon_pa6`,
    /// `glass`, `pyrex`) to their canonical preset names in the registry.
    pub fn resolve_alias(name: &str) -> &str {
        let normalized = name.trim().to_ascii_lowercase().replace(['-', ' '], "_");
        match normalized.as_str() {
            "steel_structural_aisi1020"
            | "steel_1020"
            | "aisi_1020"
            | "aisi1020"
            | "steel"
            | "structural_steel"
            | "steel_structural" => "steel_structural_aisi1020",

            "steel_carbon_aisi1045"
            | "steel_1045"
            | "aisi_1045"
            | "aisi1045"
            | "1045"
            | "carbon_steel_1045"
            | "steel_carbon_1045" => "steel_carbon_aisi1045",

            "steel_high_strength_4140"
            | "steel_4140"
            | "aisi_4140"
            | "aisi4140"
            | "4140"
            | "alloy_steel_4140" => "steel_high_strength_4140",

            "aluminum_6061_t6"
            | "alu_6061"
            | "alu_6061_t6"
            | "al6061_t6"
            | "al_6061_t6"
            | "al6061"
            | "al_6061"
            | "aluminum_6061"
            | "aluminum"
            | "aluminium_6061_t6"
            | "aluminium_6061"
            | "aluminium" => "aluminum_6061_t6",

            "titanium_grade5_ti6al4v"
            | "ti_6al_4v"
            | "ti6al4v"
            | "titanium_ti6al4v"
            | "titanium_grade5"
            | "titanium_grade_5"
            | "ti_grade5"
            | "titanium" => "titanium_grade5_ti6al4v",

            "brass_c360" | "brass_c36000" | "c360" | "c36000" | "brass" => "brass_c360",

            "pla_filament_3dprint" | "pla_3d" | "pla_plastic" | "pla_filament" | "pla" => {
                "pla_filament_3dprint"
            }

            "petg_filament_3dprint" | "petg_3d" | "petg_plastic" | "petg_filament" | "petg" => {
                "petg_filament_3dprint"
            }

            "nylon_pa12" | "nylon_pa6" | "nylon" | "pa12" | "pa6" | "polyamide_12" => "nylon_pa12",

            "borosilicate_glass"
            | "borosilicate"
            | "glass_borosilicate"
            | "pyrex"
            | "glass" => "borosilicate_glass",

            _ => name,
        }
    }

    /// Registers or overwrites a material by its name.
    pub fn register(&mut self, material: Material) {
        self.materials.insert(material.name.clone(), material);
    }

    /// Validates physical invariants before registering a material into the registry.
    pub fn try_register(&mut self, material: Material) -> Result<(), CadError> {
        material.validate()?;
        self.register(material);
        Ok(())
    }

    /// Validates physical invariants for every material in the registry.
    pub fn validate_all(&self) -> Result<(), CadError> {
        for mat in self.materials.values() {
            mat.validate()?;
        }
        Ok(())
    }

    /// Looks up a material by name or known preset alias.
    pub fn get(&self, name: &str) -> Option<&Material> {
        self.materials
            .get(name)
            .or_else(|| self.materials.get(Self::resolve_alias(name)))
    }

    /// Returns a list of all registered materials.
    pub fn list(&self) -> Vec<&Material> {
        self.materials.values().collect()
    }

    /// Checks if a material (or known preset alias) is registered.
    pub fn contains(&self, name: &str) -> bool {
        self.materials.contains_key(name)
            || self.materials.contains_key(Self::resolve_alias(name))
    }

    /// Total number of materials registered.
    pub fn len(&self) -> usize {
        self.materials.len()
    }

    /// Returns true if empty.
    pub fn is_empty(&self) -> bool {
        self.materials.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_presets_loaded() {
        let registry = MaterialRegistry::new();
        assert!(registry.contains("steel_structural_aisi1020"));
        assert!(registry.contains("steel_carbon_aisi1045"));
        assert!(registry.contains("steel_high_strength_4140"));
        assert!(registry.contains("aluminum_6061_t6"));
        assert!(registry.contains("titanium_grade5_ti6al4v"));
        assert!(registry.contains("brass_c360"));
        assert!(registry.contains("pla_filament_3dprint"));
        assert!(registry.contains("petg_filament_3dprint"));
        assert!(registry.contains("nylon_pa12"));
        assert!(registry.contains("borosilicate_glass"));

        let steel_1020 = registry.get("steel_structural_aisi1020").unwrap();
        assert!((steel_1020.youngs_modulus_gpa - 205.0).abs() < 1e-4);
        assert!((steel_1020.yield_strength_mpa - 250.0).abs() < 1e-4);
        assert!((steel_1020.ultimate_strength_pa - 420.0e6).abs() < 1e-2);
        assert!((steel_1020.hardening_modulus_pa - 1.20e9).abs() < 1e-2);
        assert!((steel_1020.fracture_strain - 0.22).abs() < 1e-6);
        assert!((steel_1020.density_kg_m3 - 7850.0).abs() < 1e-4);
        assert_eq!(steel_1020.failure_mode, FailureMode::Ductile);

        let steel_1045 = registry.get("steel_carbon_aisi1045").unwrap();
        assert!((steel_1045.youngs_modulus_gpa - 205.0).abs() < 1e-4);
        assert!((steel_1045.poissons_ratio - 0.29).abs() < 1e-6);
        assert!((steel_1045.yield_strength_mpa - 350.0).abs() < 1e-4);
        assert!((steel_1045.ultimate_strength_pa - 585.0e6).abs() < 1e-2);
        assert!((steel_1045.hardening_modulus_pa - 1.80e9).abs() < 1e-2);
        assert!((steel_1045.fracture_strain - 0.16).abs() < 1e-6);
        assert!((steel_1045.fracture_energy_j_m2 - 28_000.0).abs() < 1e-4);
        assert!((steel_1045.friction_coeff - 0.25).abs() < 1e-6);
        assert!((steel_1045.density_kg_m3 - 7850.0).abs() < 1e-4);
        assert_eq!(steel_1045.failure_mode, FailureMode::Ductile);

        let glass = registry.get("borosilicate_glass").unwrap();
        assert!((glass.youngs_modulus_gpa - 64.0).abs() < 1e-4);
        assert!((glass.poissons_ratio - 0.20).abs() < 1e-6);
        assert!((glass.density_kg_m3 - 2500.0).abs() < 1e-4);
        assert_eq!(glass.failure_mode, FailureMode::Brittle);
        assert_eq!(glass.hardening_modulus_pa, 0.0);
        assert!(glass.fracture_energy_j_m2 < 20.0);
    }

    #[test]
    fn test_material_preset_aliases() {
        let registry = MaterialRegistry::new();
        // Default preset count is 10 (including steel_carbon_aisi1045 and borosilicate_glass; aliases do not duplicate entries)
        assert_eq!(registry.len(), 10);

        let alias_pairs = [
            ("steel_structural_aisi1020", "steel_structural_aisi1020"),
            ("steel_1020", "steel_structural_aisi1020"),
            ("AISI 1020", "steel_structural_aisi1020"),
            ("steel", "steel_structural_aisi1020"),
            ("structural_steel", "steel_structural_aisi1020"),
            ("steel_carbon_aisi1045", "steel_carbon_aisi1045"),
            ("steel_1045", "steel_carbon_aisi1045"),
            ("AISI 1045", "steel_carbon_aisi1045"),
            ("1045", "steel_carbon_aisi1045"),
            ("steel_4140", "steel_high_strength_4140"),
            ("AISI 4140", "steel_high_strength_4140"),
            ("alu_6061", "aluminum_6061_t6"),
            ("Al6061-T6", "aluminum_6061_t6"),
            ("Al-6061-T6", "aluminum_6061_t6"),
            ("aluminum_6061", "aluminum_6061_t6"),
            ("ti_6al_4v", "titanium_grade5_ti6al4v"),
            ("Ti-6Al-4V", "titanium_grade5_ti6al4v"),
            ("Ti6Al4V", "titanium_grade5_ti6al4v"),
            ("brass_c36000", "brass_c360"),
            ("pla_3d", "pla_filament_3dprint"),
            ("pla_plastic", "pla_filament_3dprint"),
            ("petg_3d", "petg_filament_3dprint"),
            ("petg_plastic", "petg_filament_3dprint"),
            ("nylon_pa6", "nylon_pa12"),
            ("borosilicate_glass", "borosilicate_glass"),
            ("pyrex", "borosilicate_glass"),
            ("glass", "borosilicate_glass"),
        ];

        for (alias, canonical) in alias_pairs {
            assert!(
                registry.contains(alias),
                "Registry should contain alias '{alias}'"
            );
            let mat = registry
                .get(alias)
                .unwrap_or_else(|| panic!("Alias '{alias}' failed to resolve"));
            assert_eq!(mat.name, canonical);
        }
    }

    #[test]
    fn test_p5_t1_all_standard_materials_pass_physical_invariants() {
        let registry = MaterialRegistry::new();
        registry
            .validate_all()
            .expect("All built-in materials must satisfy physical invariants");

        let required_ids = [
            "steel_structural_aisi1020",
            "steel_1045",
            "steel_4140",
            "alu_6061",
            "ti_6al_4v",
            "brass_c360",
            "pla_3d",
            "petg_3d",
            "nylon_pa12",
            "borosilicate_glass",
        ];

        for id in required_ids {
            let mat = registry
                .get(id)
                .unwrap_or_else(|| panic!("Standard material '{id}' must exist in registry"));
            mat.validate()
                .unwrap_or_else(|e| panic!("Material '{id}' failed invariant check: {e}"));
            assert!(mat.ultimate_strength_pa >= mat.yield_strength_pa());
            assert!(mat.hardening_modulus_pa >= 0.0);
            assert!(mat.fracture_strain > 0.0);
            assert!(mat.fracture_energy_j_m2 > 0.0);
            assert!(mat.friction_coeff >= 0.0);
            assert!(mat.poissons_ratio > 0.0 && mat.poissons_ratio < 0.5);
            assert!(mat.shear_modulus_pa() > 0.0);
            assert!(mat.bulk_modulus_pa() > 0.0);
            assert!(mat.tangent_modulus_pa() <= mat.youngs_modulus_pa());
        }

        // Physical ordering sanity checks across material classes
        let steel_1020 = registry.get("steel_structural_aisi1020").unwrap();
        let steel_1045 = registry.get("steel_1045").unwrap();
        let steel_4140 = registry.get("steel_4140").unwrap();
        let glass = registry.get("borosilicate_glass").unwrap();
        let brass = registry.get("brass_c360").unwrap();
        assert!(steel_4140.yield_strength_mpa > steel_1045.yield_strength_mpa);
        assert!(steel_1045.yield_strength_mpa > steel_1020.yield_strength_mpa);
        assert!(steel_4140.ultimate_strength_pa > steel_4140.yield_strength_pa());
        assert!(steel_4140.fracture_energy_j_m2 > 1000.0 * glass.fracture_energy_j_m2);
        assert!(brass.fracture_strain > 50.0 * glass.fracture_strain);
        assert_eq!(steel_4140.failure_mode, FailureMode::Ductile);
        assert_eq!(glass.failure_mode, FailureMode::Brittle);
    }

    #[test]
    fn test_p5_t11_rust_ts_standard_materials_parity() {
        let ts_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../src/cad/materials.ts");
        let ts_src = std::fs::read_to_string(&ts_path).unwrap_or_else(|e| {
            panic!(
                "Failed to read TypeScript material library at {}: {e}",
                ts_path.display()
            )
        });

        let lib_start = ts_src
            .find("export const MATERIALS_LIBRARY: MaterialSpec[] = [")
            .expect("MATERIALS_LIBRARY declaration not found in src/cad/materials.ts");
        let lib_after = &ts_src[lib_start..];
        let lib_end = lib_after
            .find("];")
            .expect("Closing '];' of MATERIALS_LIBRARY not found");
        let lib_body = &lib_after[..lib_end];

        #[derive(Debug)]
        struct TsMat {
            id: String,
            density: f64,
            youngs_modulus: f64,
            poissons_ratio: f64,
            yield_strength: f64,
            ultimate_strength_pa: f64,
            hardening_modulus_pa: f64,
            fracture_strain: f64,
            fracture_energy_j_m2: f64,
            friction_coeff: f64,
            failure_mode: String,
        }

        fn parse_ts_num(block: &str, key: &str) -> f64 {
            let needle = format!("{key}:");
            let idx = block
                .find(&needle)
                .unwrap_or_else(|| panic!("Missing numeric field '{key}' in TS block:\n{block}"));
            let rest = &block[idx + needle.len()..];
            let end = rest.find(',').unwrap_or(rest.len());
            let raw = rest[..end].trim().replace('_', "");
            raw.parse::<f64>().unwrap_or_else(|e| {
                panic!("Failed to parse TS numeric field '{key}' value '{raw}': {e}")
            })
        }

        fn parse_ts_str(block: &str, key: &str) -> String {
            let needle = format!("{key}:");
            let idx = block
                .find(&needle)
                .unwrap_or_else(|| panic!("Missing string field '{key}' in TS block:\n{block}"));
            let rest = block[idx + needle.len()..].trim_start();
            let quote = rest
                .chars()
                .next()
                .expect("Expected quote for TS string field");
            assert!(quote == '\'' || quote == '"');
            let inner = &rest[1..];
            let end = inner
                .find(quote)
                .expect("Missing closing quote for TS string field");
            inner[..end].to_string()
        }

        let mut ts_materials = Vec::new();
        let mut cursor = 0usize;
        while let Some(open_rel) = lib_body[cursor..].find('{') {
            let open = cursor + open_rel;
            let close_rel = lib_body[open..]
                .find('}')
                .expect("Unclosed object literal in MATERIALS_LIBRARY");
            let close = open + close_rel;
            let block = &lib_body[open + 1..close];
            ts_materials.push(TsMat {
                id: parse_ts_str(block, "id"),
                density: parse_ts_num(block, "density"),
                youngs_modulus: parse_ts_num(block, "youngsModulus"),
                poissons_ratio: parse_ts_num(block, "poissonsRatio"),
                yield_strength: parse_ts_num(block, "yieldStrength"),
                ultimate_strength_pa: parse_ts_num(block, "ultimateStrengthPa"),
                hardening_modulus_pa: parse_ts_num(block, "hardeningModulusPa"),
                fracture_strain: parse_ts_num(block, "fractureStrain"),
                fracture_energy_j_m2: parse_ts_num(block, "fractureEnergyJM2"),
                friction_coeff: parse_ts_num(block, "frictionCoeff"),
                failure_mode: parse_ts_str(block, "failureMode"),
            });
            cursor = close + 1;
        }

        let registry = MaterialRegistry::new();
        assert_eq!(
            ts_materials.len(),
            registry.len(),
            "Rust MaterialRegistry ({}) and TS MATERIALS_LIBRARY ({}) must define the same number of standard presets",
            registry.len(),
            ts_materials.len()
        );

        let mut ts_by_id: HashMap<String, &TsMat> = HashMap::new();
        let mut seen_rust_canonicals = std::collections::HashSet::new();

        for ts_mat in &ts_materials {
            ts_by_id.insert(ts_mat.id.clone(), ts_mat);
            let rust_mat = registry.get(&ts_mat.id).unwrap_or_else(|| {
                panic!(
                    "TS preset '{}' must resolve in Rust MaterialRegistry",
                    ts_mat.id
                )
            });
            assert!(
                seen_rust_canonicals.insert(rust_mat.name.clone()),
                "TS presets must map 1-to-1 to distinct Rust canonical presets (duplicate '{}')",
                rust_mat.name
            );

            let assert_close = |field: &str, r_val: f64, t_val: f64| {
                let scale = r_val.abs().max(t_val.abs()).max(1.0);
                assert!(
                    (r_val - t_val).abs() <= 1e-9 * scale,
                    "Mismatch for '{}' (Rust '{}') field '{}': Rust={} vs TS={}",
                    ts_mat.id,
                    rust_mat.name,
                    field,
                    r_val,
                    t_val
                );
            };

            assert_close("density", rust_mat.density_kg_m3, ts_mat.density);
            assert_close("youngs_modulus_pa", rust_mat.youngs_modulus_pa(), ts_mat.youngs_modulus);
            assert_close("poissons_ratio", rust_mat.poissons_ratio, ts_mat.poissons_ratio);
            assert_close("yield_strength_pa", rust_mat.yield_strength_pa(), ts_mat.yield_strength);
            assert_close("ultimate_strength_pa", rust_mat.ultimate_strength_pa, ts_mat.ultimate_strength_pa);
            assert_close("hardening_modulus_pa", rust_mat.hardening_modulus_pa, ts_mat.hardening_modulus_pa);
            assert_close("fracture_strain", rust_mat.fracture_strain, ts_mat.fracture_strain);
            assert_close("fracture_energy_j_m2", rust_mat.fracture_energy_j_m2, ts_mat.fracture_energy_j_m2);
            assert_close("friction_coeff", rust_mat.friction_coeff, ts_mat.friction_coeff);
            assert_eq!(
                rust_mat.failure_mode.as_str(),
                ts_mat.failure_mode,
                "Failure mode mismatch for '{}' (Rust '{}')",
                ts_mat.id,
                rust_mat.name
            );
        }

        // Verify every alias in TS MATERIAL_ALIAS_MAP resolves to the physically identical material in Rust
        let alias_start = ts_src
            .find("const MATERIAL_ALIAS_MAP: Record<string, string> = {")
            .expect("MATERIAL_ALIAS_MAP not found in src/cad/materials.ts");
        let alias_after = &ts_src[alias_start..];
        let alias_open = alias_after.find('{').unwrap();
        let alias_close = alias_after.find("};").unwrap();
        let alias_body = &alias_after[alias_open + 1..alias_close];

        let mut alias_count = 0usize;
        for line in alias_body.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with("//") {
                continue;
            }
            let Some((raw_k, raw_v)) = trimmed.split_once(':') else {
                continue;
            };
            let alias = raw_k.trim().trim_matches('\'').trim_matches('"');
            let ts_target = raw_v
                .trim()
                .trim_end_matches(',')
                .trim()
                .trim_matches('\'')
                .trim_matches('"');
            let ts_mat = ts_by_id.get(ts_target).unwrap_or_else(|| {
                panic!("TS alias '{alias}' points to unknown TS preset '{ts_target}'")
            });
            let rust_mat = registry.get(alias).unwrap_or_else(|| {
                panic!("TS alias '{alias}' (target '{ts_target}') failed to resolve in Rust MaterialRegistry")
            });
            assert!(
                (rust_mat.yield_strength_pa() - ts_mat.yield_strength).abs() <= 1e-3,
                "Alias '{alias}' diverged between Rust ('{}', σ_y={} Pa) and TS ('{}', σ_y={} Pa)",
                rust_mat.name,
                rust_mat.yield_strength_pa(),
                ts_mat.id,
                ts_mat.yield_strength
            );
            assert!(
                (rust_mat.ultimate_strength_pa - ts_mat.ultimate_strength_pa).abs() <= 1e-3,
                "Alias '{alias}' UTS diverged between Rust and TS"
            );
            alias_count += 1;
        }
        assert!(
            alias_count >= 50,
            "Expected full TS MATERIAL_ALIAS_MAP coverage, parsed only {alias_count} aliases"
        );
    }

    #[test]
    fn test_p5_t1_material_validation_rejects_invalid_invariants() {
        let base = Material::new("valid_alloy", 200.0, 0.30, 300.0, 7800.0, "Valid");
        assert!(base.validate().is_ok());

        // σ_u < σ_y must fail
        let mut bad_uts = base.clone();
        bad_uts.ultimate_strength_pa = 250.0e6;
        assert!(bad_uts.validate().is_err());

        // σ_y <= 0 must fail
        let mut bad_yield = base.clone();
        bad_yield.yield_strength_mpa = 0.0;
        assert!(bad_yield.validate().is_err());

        // H < 0 must fail
        let mut bad_h = base.clone();
        bad_h.hardening_modulus_pa = -1.0;
        assert!(bad_h.validate().is_err());

        // ε_f <= 0 must fail
        let mut bad_ef = base.clone();
        bad_ef.fracture_strain = 0.0;
        assert!(bad_ef.validate().is_err());

        // G_f <= 0 must fail
        let mut bad_gf = base.clone();
        bad_gf.fracture_energy_j_m2 = -10.0;
        assert!(bad_gf.validate().is_err());

        // ν outside (0, 0.5) must fail
        let mut bad_nu_low = base.clone();
        bad_nu_low.poissons_ratio = 0.0;
        assert!(bad_nu_low.validate().is_err());

        let mut bad_nu_high = base.clone();
        bad_nu_high.poissons_ratio = 0.5;
        assert!(bad_nu_high.validate().is_err());

        // μ < 0 or NaN must fail
        let mut bad_mu = base.clone();
        bad_mu.friction_coeff = -0.1;
        assert!(bad_mu.validate().is_err());

        let mut bad_nan = base.clone();
        bad_nan.fracture_energy_j_m2 = f64::NAN;
        assert!(bad_nan.validate().is_err());

        // try_register rejects invalid material without mutating registry
        let mut registry = MaterialRegistry::new();
        assert!(registry.try_register(bad_uts).is_err());
        assert!(!registry.contains("valid_alloy"));
    }

    #[test]
    fn test_p5_t1_backward_compatible_deserialization_and_roundtrip() {
        // Legacy 6-field JSON (pre-Phase 5) with high yield strength (880 MPa)
        let legacy_json = r#"{
            "name": "legacy_titanium",
            "youngs_modulus_gpa": 114.0,
            "poissons_ratio": 0.34,
            "yield_strength_mpa": 880.0,
            "density_kg_m3": 4430.0,
            "description": "Legacy project material"
        }"#;

        let mat: Material = serde_json::from_str(legacy_json)
            .expect("Legacy 6-field material JSON must deserialize cleanly");
        mat.validate()
            .expect("Deserialized legacy material must satisfy all Phase 5 invariants");
        assert!(mat.ultimate_strength_pa >= 880.0e6);
        assert!(mat.hardening_modulus_pa > 0.0);
        assert!(mat.fracture_strain > 0.0);
        assert!(mat.fracture_energy_j_m2 > 0.0);
        assert!(mat.friction_coeff >= 0.0);
        assert_eq!(mat.failure_mode, FailureMode::Ductile);

        // Full Phase 5 round-trip serialization preserves every field exactly
        let registry = MaterialRegistry::new();
        let glass = registry.get("borosilicate_glass").unwrap();
        let serialized = serde_json::to_string(glass).expect("Serialize Material");
        let roundtrip: Material =
            serde_json::from_str(&serialized).expect("Deserialize Material");
        assert_eq!(&roundtrip, glass);
        assert!(serialized.contains("\"failure_mode\":\"brittle\""));
    }

    #[test]
    fn test_custom_material_registration() {
        let mut registry = MaterialRegistry::new();
        let custom = Material::new(
            "carbon_fiber",
            150.0,
            0.28,
            500.0,
            1600.0,
            "Custom carbon composite",
        );
        registry.try_register(custom).expect("Custom material valid");

        assert!(registry.contains("carbon_fiber"));
        let retrieved = registry.get("carbon_fiber").unwrap();
        assert_eq!(retrieved.name, "carbon_fiber");
        assert!((retrieved.density_g_cm3() - 1.6).abs() < 1e-4);
        assert!(retrieved.validate().is_ok());

        // Unknown material lookup returns None
        assert!(registry.get("unknown_material_xyz").is_none());

        // Dynamic registration updates existing material
        let updated = Material::new(
            "carbon_fiber",
            180.0,
            0.30,
            600.0,
            1650.0,
            "Updated carbon composite",
        );
        registry.register(updated);
        assert_eq!(
            registry.get("carbon_fiber").unwrap().youngs_modulus_gpa,
            180.0
        );
    }
}
