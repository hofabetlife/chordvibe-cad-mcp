//! Part representation: combines geometry, parametric DAG history, and physical material.

use serde::{Deserialize, Serialize};
use crate::dag::FeatureTree;
use crate::error::CadError;
use crate::material::Material;
use crate::tessellation::{TessellationQuality, TessellationSettings};
use crate::topology::{BoundingBox, GeometryInspection, TriangleMesh};

/// A complete CAD part comprising identity, material, parametric DAG, and generated boundary mesh.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Part {
    /// Unique part identifier (e.g. UUID string or user-defined part_id).
    pub id: String,
    /// User-friendly name of the part.
    pub name: String,
    /// Physical material assigned to this part.
    pub material: Material,
    /// Parametric history tree defining the part geometry.
    pub dag: FeatureTree,
    /// The currently evaluated triangular boundary surface mesh.
    pub mesh: TriangleMesh,
    /// Cached evaluation mesh for lazy tessellation.
    #[serde(default)]
    pub cached_mesh: Option<TriangleMesh>,
    /// Settings corresponding to cached_mesh.
    #[serde(default)]
    pub cached_settings: Option<TessellationSettings>,
    /// Indicates whether the part should be treated as fixed/static in physics simulations.
    #[serde(default)]
    pub is_fixed: bool,
}

impl Part {
    /// Creates a new empty part with assigned material.
    pub fn new(id: impl Into<String>, name: impl Into<String>, material: Material) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            material,
            dag: FeatureTree::new(),
            mesh: TriangleMesh::empty(),
            cached_mesh: None,
            cached_settings: None,
            is_fixed: false,
        }
    }

    /// Evaluates the parametric feature tree with the specified tessellation settings lazily.
    /// Returns a cached mesh if the requested settings match the cached settings.
    pub fn tessellate(&mut self, settings: &TessellationSettings) -> Result<&TriangleMesh, CadError> {
        if let (Some(ref _cached), Some(ref s)) = (&self.cached_mesh, &self.cached_settings) {
            if s == settings {
                return Ok(self.cached_mesh.as_ref().unwrap());
            }
        }

        let evaluated = self.dag.evaluate_with_settings(settings)?;
        self.mesh = evaluated.clone();
        self.cached_mesh = Some(evaluated);
        self.cached_settings = Some(settings.clone());
        Ok(self.cached_mesh.as_ref().unwrap())
    }

    /// Evaluates the part geometry with a quality preset.
    pub fn tessellate_quality(&mut self, quality: TessellationQuality) -> Result<&TriangleMesh, CadError> {
        let settings = TessellationSettings::from(quality);
        self.tessellate(&settings)
    }

    /// Re-evaluates the parametric feature tree and updates the stored triangle mesh.
    /// Uses cached settings if available, otherwise defaults to `TessellationSettings::default()`.
    pub fn rebuild(&mut self) -> Result<(), CadError> {
        let settings = self.cached_settings.clone().unwrap_or_default();
        let evaluated = self.dag.evaluate_with_settings(&settings)?;
        self.mesh = evaluated.clone();
        self.cached_mesh = Some(evaluated);
        self.cached_settings = Some(settings);
        Ok(())
    }

    /// Invalidates cached tessellation mesh (e.g. after modifying the DAG).
    pub fn invalidate_cache(&mut self) {
        self.cached_mesh = None;
        self.cached_settings = None;
    }

    /// Reference to the current evaluated triangle mesh.
    #[inline]
    pub fn mesh(&self) -> &TriangleMesh {
        if let Some(ref m) = self.cached_mesh {
            m
        } else {
            &self.mesh
        }
    }

    /// Mutable reference to the boundary mesh for legacy direct manipulations.
    #[inline]
    pub fn mesh_mut(&mut self) -> &mut TriangleMesh {
        &mut self.mesh
    }

    /// Volume of the evaluated mesh.
    /// In SI units (when coordinates are in meters), volume is in m³.
    #[inline]
    pub fn volume(&self) -> f64 {
        self.mesh().volume()
    }

    /// Total mass of the part in kilograms (assuming coordinates are in meters):
    /// mass = volume (m³) * density (kg/m³).
    #[inline]
    pub fn mass(&self) -> f64 {
        self.volume() * self.material.density_kg_m3
    }

    /// Mass in kilograms when the model is designed in millimeters (mm):
    /// 1 mm³ = 1e-9 m³.
    #[inline]
    pub fn mass_from_mm(&self) -> f64 {
        self.volume() * 1.0e-9 * self.material.density_kg_m3
    }

    /// Exact 3D center of mass [x, y, z] of the part.
    #[inline]
    pub fn center_of_mass(&self) -> [f64; 3] {
        self.mesh().center_of_mass()
    }

    /// Total surface area of the boundary mesh.
    #[inline]
    pub fn surface_area(&self) -> f64 {
        self.mesh().surface_area()
    }

    /// 3D Axis-Aligned Bounding Box (AABB) of the part.
    #[inline]
    pub fn bounding_box(&self) -> BoundingBox {
        self.mesh().bounding_box()
    }

    /// Detailed geometry inspection for CAD analysis and AI context.
    #[inline]
    pub fn inspect_geometry(&self) -> GeometryInspection {
        self.mesh().inspect_geometry()
    }

    /// Summary metadata for MCP / UI listings.
    pub fn summary(&self) -> PartSummary {
        let aabb = self.bounding_box();
        let mesh = self.mesh();
        PartSummary {
            id: self.id.clone(),
            name: self.name.clone(),
            material_name: self.material.name.clone(),
            volume: self.volume(),
            mass: self.mass(),
            triangle_count: mesh.triangle_count(),
            bounding_box_dimensions: aabb.dimensions(),
            is_fixed: self.is_fixed,
        }
    }
}

/// Lightweight summary metadata of a Part for API listings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PartSummary {
    pub id: String,
    pub name: String,
    pub material_name: String,
    pub volume: f64,
    pub mass: f64,
    pub triangle_count: usize,
    pub bounding_box_dimensions: [f64; 3],
    #[serde(default)]
    pub is_fixed: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dag::FeatureOp;
    use crate::material::MaterialRegistry;

    #[test]
    fn test_part_rebuild_and_mass() {
        let registry = MaterialRegistry::new();
        let aluminum = registry.get("aluminum_6061_t6").unwrap().clone();

        let mut part = Part::new("part-001", "Bracket Base", aluminum);
        // Box: 0.1m x 0.2m x 0.05m = 0.001 m³
        part.dag.add_feature(
            FeatureOp::AddBox {
                width: 0.1,
                height: 0.2,
                depth: 0.05,
                position: None,
                rotation_deg: None,
            },
            None,
        );

        part.rebuild().unwrap();

        let vol = part.volume();
        assert!((vol - 0.001).abs() < 1e-6);

        // Density of aluminum_6061_t6 is 2700 kg/m³
        // Mass = 0.001 * 2700 = 2.7 kg
        let mass = part.mass();
        assert!((mass - 2.7).abs() < 1e-3);
    }
}
