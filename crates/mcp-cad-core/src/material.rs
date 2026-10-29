//! Material definitions and registry for physical & FEA simulations.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Material properties defining mechanical and physical behavior.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Material {
    /// Identifier or common name of the material.
    pub name: String,
    /// Young's modulus in Gigapascals (GPa).
    pub youngs_modulus_gpa: f64,
    /// Poisson's ratio (dimensionless, typically between 0.2 and 0.45).
    pub poissons_ratio: f64,
    /// Yield strength in Megapascals (MPa).
    pub yield_strength_mpa: f64,
    /// Density in kilograms per cubic meter (kg/m³).
    pub density_kg_m3: f64,
    /// Human-readable description and intended applications.
    pub description: String,
}

impl Material {
    /// Creates a new Material with given mechanical properties.
    pub fn new(
        name: impl Into<String>,
        youngs_modulus_gpa: f64,
        poissons_ratio: f64,
        yield_strength_mpa: f64,
        density_kg_m3: f64,
        description: impl Into<String>,
    ) -> Self {
        Self {
            name: name.into(),
            youngs_modulus_gpa,
            poissons_ratio,
            yield_strength_mpa,
            density_kg_m3,
            description: description.into(),
        }
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

    /// Registers standard preset materials.
    fn register_presets(&mut self) {
        self.register(Material::new(
            "steel_structural_aisi1020",
            205.0,
            0.29,
            250.0,
            7850.0,
            "Structural steel AISI 1020",
        ));

        self.register(Material::new(
            "steel_high_strength_4140",
            210.0,
            0.30,
            655.0,
            7850.0,
            "High-strength alloy steel AISI 4140",
        ));

        self.register(Material::new(
            "aluminum_6061_t6",
            68.9,
            0.33,
            276.0,
            2700.0,
            "Aerospace / structural aluminum alloy 6061-T6",
        ));

        self.register(Material::new(
            "titanium_grade5_ti6al4v",
            114.0,
            0.34,
            880.0,
            4430.0,
            "Titanium Grade 5 (Ti-6Al-4V) high-strength alloy",
        ));

        self.register(Material::new(
            "brass_c360",
            100.0,
            0.34,
            310.0,
            8500.0,
            "Free-cutting brass C360",
        ));

        self.register(Material::new(
            "pla_filament_3dprint",
            3.5,
            0.36,
            60.0,
            1240.0,
            "Polylactic acid (PLA) 3D printing filament",
        ));

        self.register(Material::new(
            "petg_filament_3dprint",
            2.1,
            0.38,
            50.0,
            1270.0,
            "Polyethylene terephthalate glycol (PETG) filament",
        ));

        self.register(Material::new(
            "nylon_pa12",
            1.8,
            0.40,
            45.0,
            1010.0,
            "Polyamide 12 (Nylon PA12) engineering thermoplastic",
        ));
    }

    /// Registers or overwrites a material by its name.
    pub fn register(&mut self, material: Material) {
        self.materials.insert(material.name.clone(), material);
    }

    /// Looks up a material by name.
    pub fn get(&self, name: &str) -> Option<&Material> {
        self.materials.get(name)
    }

    /// Returns a list of all registered materials.
    pub fn list(&self) -> Vec<&Material> {
        self.materials.values().collect()
    }

    /// Checks if a material is registered.
    pub fn contains(&self, name: &str) -> bool {
        self.materials.contains_key(name)
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
        assert!(registry.contains("aluminum_6061_t6"));
        assert!(registry.contains("titanium_grade5_ti6al4v"));
        assert!(registry.contains("pla_filament_3dprint"));
        assert!(registry.contains("petg_filament_3dprint"));
        assert!(registry.contains("nylon_pa12"));
        assert!(registry.contains("brass_c360"));

        let steel = registry.get("steel_structural_aisi1020").unwrap();
        assert!((steel.youngs_modulus_gpa - 205.0).abs() < 1e-4);
        assert!((steel.density_kg_m3 - 7850.0).abs() < 1e-4);
    }

    #[test]
    fn test_custom_material_registration() {
        let mut registry = MaterialRegistry::new();
        let custom = Material::new("carbon_fiber", 150.0, 0.28, 500.0, 1600.0, "Custom carbon composite");
        registry.register(custom);

        assert!(registry.contains("carbon_fiber"));
        let retrieved = registry.get("carbon_fiber").unwrap();
        assert_eq!(retrieved.name, "carbon_fiber");
        assert!((retrieved.density_g_cm3() - 1.6).abs() < 1e-4);

        // Unknown material lookup returns None
        assert!(registry.get("unknown_material_xyz").is_none());

        // Dynamic registration updates existing material
        let updated = Material::new("carbon_fiber", 180.0, 0.30, 600.0, 1650.0, "Updated carbon composite");
        registry.register(updated);
        assert_eq!(registry.get("carbon_fiber").unwrap().youngs_modulus_gpa, 180.0);
    }
}
