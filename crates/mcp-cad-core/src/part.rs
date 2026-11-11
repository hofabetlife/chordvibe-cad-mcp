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
    #[serde(default)]
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
    /// Monotonically increasing revision counter incremented on each DAG/geometry rebuild or transform.
    #[serde(default)]
    pub revision: u64,
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
            revision: 0,
        }
    }

    /// Increments the part's geometry revision counter.
    #[inline]
    pub fn bump_revision(&mut self) {
        self.revision = self.revision.saturating_add(1);
    }

    /// Ensures that if a part has a non-empty boundary mesh (`!self.mesh.vertices.is_empty()`)
    /// but an empty parametric history DAG (`self.dag.nodes.is_empty()`), a synthetic root
    /// [`crate::dag::FeatureOp::ImportedMesh`] node is seeded into `self.dag` so subsequent
    /// operations (`transform_part`, `cut_hole`, `boolean_*`, `rebuild`, `optimize_dag`)
    /// evaluate against the imported solid instead of failing with `Cannot ... empty solid` (P2-T33).
    pub fn ensure_imported_mesh_in_dag(&mut self) {
        if self.dag.nodes.is_empty() && !self.mesh.vertices.is_empty() {
            self.dag.add_feature(
                crate::dag::FeatureOp::ImportedMesh {
                    mesh: self.mesh.clone(),
                },
                Some(format!("Imported Mesh ({})", self.name)),
            );
        }
    }

    /// Evaluates the parametric feature tree with the specified tessellation settings lazily.
    /// Returns a cached mesh if the requested settings match the cached settings.
    /// Does not increment `self.revision` because changing tessellation resolution (`coarse`/`medium`/`fine`)
    /// does not modify the part's parametric DAG or structural geometry.
    pub fn tessellate(&mut self, settings: &TessellationSettings) -> Result<&TriangleMesh, CadError> {
        self.ensure_imported_mesh_in_dag();
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
        self.ensure_imported_mesh_in_dag();
        let settings = self.cached_settings.clone().unwrap_or_default();
        let evaluated = self.dag.evaluate_with_settings(&settings)?;
        self.mesh = evaluated.clone();
        self.cached_mesh = Some(evaluated);
        self.cached_settings = Some(settings);
        self.revision = self.revision.saturating_add(1);
        Ok(())
    }

    /// Invalidates cached tessellation mesh (e.g. after modifying the DAG).
    pub fn invalidate_cache(&mut self) {
        self.cached_mesh = None;
        self.cached_settings = None;
    }

    /// Runs the Cost-Based CAD Query Optimizer on this part's parametric FeatureTree (DAG).
    /// If `apply` is true, replaces `self.dag` with the optimized physical execution plan and rebuilds the mesh.
    pub fn optimize_dag(
        &mut self,
        apply: bool,
    ) -> Result<crate::query_optimizer::CadOptimizationReport, CadError> {
        self.ensure_imported_mesh_in_dag();
        let settings = self.cached_settings.clone().unwrap_or_default();
        let (report, optimized_tree) = crate::query_optimizer::optimize_feature_tree_dag(
            &self.id,
            &self.name,
            &self.dag,
            &settings,
            apply,
        )?;
        if apply {
            let dag_changed = !report.applied_rules.is_empty() || self.dag != optimized_tree;
            if dag_changed {
                let prev_dag = self.dag.clone();
                let prev_mesh = self.mesh.clone();
                let prev_cached_mesh = self.cached_mesh.clone();
                let prev_cached_settings = self.cached_settings.clone();
                let prev_revision = self.revision;
                self.dag = optimized_tree;
                self.invalidate_cache();
                if let Err(e) = self.rebuild() {
                    self.dag = prev_dag;
                    self.mesh = prev_mesh;
                    self.cached_mesh = prev_cached_mesh;
                    self.cached_settings = prev_cached_settings;
                    self.revision = prev_revision;
                    return Err(e);
                }
            } else if self.mesh.vertices.is_empty() && !self.dag.nodes.is_empty() {
                let _ = self.tessellate(&settings)?;
            }
        }
        Ok(report)
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
            revision: self.revision,
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
    #[serde(default)]
    pub revision: u64,
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
        assert_eq!(part.revision, 0);
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
        assert_eq!(part.revision, 1);

        let vol = part.volume();
        assert!((vol - 0.001).abs() < 1e-6);

        // Density of aluminum_6061_t6 is 2700 kg/m³
        // Mass = 0.001 * 2700 = 2.7 kg
        let mass = part.mass();
        assert!((mass - 2.7).abs() < 1e-3);

        // Re-tessellating at Coarse / Fine / Medium must not increment part.revision (P2-T24)
        part.tessellate_quality(TessellationQuality::Coarse).unwrap();
        assert_eq!(part.revision, 1);
        part.tessellate_quality(TessellationQuality::Fine).unwrap();
        assert_eq!(part.revision, 1);
        part.tessellate(&TessellationSettings::custom(0.0002, 5.0, None)).unwrap();
        assert_eq!(part.revision, 1);

        // Calling optimize_dag(true) when no rules apply must NOT increment part.revision (P2-T27)
        let rep_noop = part.optimize_dag(true).unwrap();
        assert!(rep_noop.applied_rules.is_empty());
        assert_eq!(part.revision, 1);
    }

    #[test]
    fn test_p2_t27_optimize_dag_only_bumps_revision_when_rules_applied() {
        let registry = MaterialRegistry::new();
        let steel = registry.get("steel_structural_aisi1020").unwrap().clone();

        let mut bushing = Part::new("bushing-p2t27", "Coaxial Bushing", steel);
        bushing.dag.add_feature(
            FeatureOp::AddCylinder {
                radius: 0.04,
                height: 0.06,
                segments: 32,
                position: None,
                rotation_deg: None,
            },
            None,
        );
        bushing.rebuild().unwrap();
        assert_eq!(bushing.revision, 1);

        bushing.dag.add_feature(
            FeatureOp::CutHole {
                center: [0.0, 0.0, 0.0],
                normal: [0.0, 0.0, 1.0],
                radius: 0.02,
                depth: 0.08,
                keyway_width: None,
                keyway_depth: None,
            },
            None,
        );
        bushing.rebuild().unwrap();
        assert_eq!(bushing.revision, 2);

        // Dry-run optimize_dag(false) must not bump revision
        let rep_dry = bushing.optimize_dag(false).unwrap();
        assert!(!rep_dry.applied_rules.is_empty());
        assert_eq!(bushing.revision, 2);

        // First optimize_dag(true) applies R4_COAXIAL_REVOLVE_REDUCTION -> bumps revision from 2 to 3
        let rep_applied = bushing.optimize_dag(true).unwrap();
        assert!(!rep_applied.applied_rules.is_empty());
        assert_eq!(bushing.revision, 3);
        assert_eq!(bushing.dag.nodes.len(), 1);

        // Second optimize_dag(true) on already-optimized single-node revolve has applied_rules.is_empty() -> revision stays 3
        let rep_second = bushing.optimize_dag(true).unwrap();
        assert!(rep_second.applied_rules.is_empty());
        assert_eq!(bushing.revision, 3);
    }

    #[test]
    fn test_p2_t33_ensure_imported_mesh_in_dag_allows_transform_on_mesh_only_part() {
        let registry = MaterialRegistry::new();
        let steel = registry.get("steel_structural_aisi1020").unwrap().clone();

        let mut part = Part::new("imported-stl-1", "Imported Housing", steel);
        part.mesh = crate::primitives::create_box(0.05, 0.04, 0.03);
        part.bump_revision();
        assert!(part.dag.nodes.is_empty());

        // Seed imported mesh before adding Transform
        part.ensure_imported_mesh_in_dag();
        assert_eq!(part.dag.nodes.len(), 1);
        part.dag.add_feature(
            FeatureOp::Transform {
                translation: [0.20, 0.10, -0.05],
                rotation_deg: [0.0, 0.0, 0.0],
                scale: [1.0, 1.0, 1.0],
            },
            None,
        );
        part.rebuild().unwrap();
        assert_eq!(part.dag.nodes.len(), 2);
        let center = part.bounding_box().center();
        assert!((center[0] - 0.20).abs() < 1e-6);
        assert!((center[1] - 0.10).abs() < 1e-6);
        assert!((center[2] - (-0.05)).abs() < 1e-6);
    }
}
