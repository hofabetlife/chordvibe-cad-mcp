//! Tests for adaptive on-demand tessellation, chordal error, Part caching, and STL export with quality.

use std::f64::consts::PI;
use mcp_cad_core::dag::{FeatureOp, FeatureTree};
use mcp_cad_core::export_stl::{export_part_stl_with_quality, write_part_stl_with_quality};
use mcp_cad_core::material::MaterialRegistry;
use mcp_cad_core::part::Part;
use mcp_cad_core::primitives::{
    create_cone_with_settings, create_cylinder_with_settings, create_revolve_with_settings,
    create_sphere_with_settings, create_spur_gear_with_settings,
};
use mcp_cad_core::tessellation::{TessellationQuality, TessellationSettings};
use tempfile::NamedTempFile;

#[test]
fn test_cylinder_adaptive_tessellation_density_and_convergence() {
    let radius = 0.05; // 50 mm radius
    let height = 0.10; // 100 mm height
    let v_analytical = PI * radius * radius * height;

    let coarse = TessellationSettings::coarse();
    let medium = TessellationSettings::medium();
    let fine = TessellationSettings::fine();

    let mesh_coarse = create_cylinder_with_settings(radius, height, &coarse);
    let mesh_medium = create_cylinder_with_settings(radius, height, &medium);
    let mesh_fine = create_cylinder_with_settings(radius, height, &fine);

    // 1. Density progression
    assert!(
        mesh_fine.triangle_count() > mesh_medium.triangle_count(),
        "Fine mesh ({}) must have more triangles than Medium ({})",
        mesh_fine.triangle_count(),
        mesh_medium.triangle_count()
    );
    assert!(
        mesh_medium.triangle_count() > mesh_coarse.triangle_count(),
        "Medium mesh ({}) must have more triangles than Coarse ({})",
        mesh_medium.triangle_count(),
        mesh_coarse.triangle_count()
    );

    // 2. Analytical volume convergence: inscribed polygon volume approaches true circle as N increases
    let err_coarse = (mesh_coarse.volume() - v_analytical).abs() / v_analytical;
    let err_medium = (mesh_medium.volume() - v_analytical).abs() / v_analytical;
    let err_fine = (mesh_fine.volume() - v_analytical).abs() / v_analytical;

    assert!(
        err_fine < err_medium,
        "Fine volume error ({:e}) must be smaller than Medium ({:e})",
        err_fine,
        err_medium
    );
    assert!(
        err_medium < err_coarse,
        "Medium volume error ({:e}) must be smaller than Coarse ({:e})",
        err_medium,
        err_coarse
    );
    assert!(
        err_fine < 0.001,
        "Fine tessellation error ({:e}) must be less than 0.1%",
        err_fine
    );
}

#[test]
fn test_sphere_and_cone_adaptive_tessellation() {
    let r = 0.04;
    let coarse = TessellationSettings::coarse();
    let fine = TessellationSettings::fine();

    // Sphere
    let sph_coarse = create_sphere_with_settings(r, &coarse);
    let sph_fine = create_sphere_with_settings(r, &fine);
    assert!(
        sph_fine.triangle_count() > sph_coarse.triangle_count() * 2,
        "Fine sphere ({} tris) should have significantly more triangles than Coarse ({} tris)",
        sph_fine.triangle_count(),
        sph_coarse.triangle_count()
    );

    let v_sph_analytical = (4.0 / 3.0) * PI * r.powi(3);
    let err_sph_coarse = (sph_coarse.volume() - v_sph_analytical).abs() / v_sph_analytical;
    let err_sph_fine = (sph_fine.volume() - v_sph_analytical).abs() / v_sph_analytical;
    assert!(err_sph_fine < err_sph_coarse);

    // Cone
    let cone_coarse = create_cone_with_settings(0.05, 0.02, 0.08, &coarse);
    let cone_fine = create_cone_with_settings(0.05, 0.02, 0.08, &fine);
    assert!(cone_fine.triangle_count() > cone_coarse.triangle_count());
}

#[test]
fn test_revolve_and_spur_gear_adaptive_tessellation() {
    let coarse = TessellationSettings::coarse();
    let fine = TessellationSettings::fine();

    // Revolve
    let profile = vec![
        [0.01, -0.02],
        [0.04, -0.02],
        [0.04, 0.02],
        [0.01, 0.02],
    ];
    let rev_coarse = create_revolve_with_settings(&profile, 360.0, &coarse);
    let rev_fine = create_revolve_with_settings(&profile, 360.0, &fine);
    assert!(rev_fine.triangle_count() > rev_coarse.triangle_count());

    // Spur gear
    let gear_coarse = create_spur_gear_with_settings(
        0.002, 16, 20.0, 0.01, None, None, None, None, None, &coarse,
    );
    let gear_fine = create_spur_gear_with_settings(
        0.002, 16, 20.0, 0.01, None, None, None, None, None, &fine,
    );
    assert!(gear_fine.triangle_count() > gear_coarse.triangle_count());
}

#[test]
fn test_dag_evaluate_with_settings_progression() {
    let mut tree = FeatureTree::new();
    tree.add_feature(
        FeatureOp::AddCylinder {
            radius: 0.03,
            height: 0.06,
            segments: 32,
            position: None,
            rotation_deg: None,
        },
        Some("Shaft".into()),
    );

    let coarse = TessellationSettings::coarse();
    let medium = TessellationSettings::medium();
    let fine = TessellationSettings::fine();

    let mesh_coarse = tree.evaluate_with_settings(&coarse).unwrap();
    let mesh_medium = tree.evaluate_with_settings(&medium).unwrap();
    let mesh_fine = tree.evaluate_with_settings(&fine).unwrap();

    assert!(mesh_fine.triangle_count() > mesh_medium.triangle_count());
    assert!(mesh_medium.triangle_count() > mesh_coarse.triangle_count());
}

#[test]
fn test_part_lazy_tessellation_and_caching() {
    let registry = MaterialRegistry::new();
    let steel = registry.get("steel_structural_aisi1020").unwrap().clone();
    let mut part = Part::new("part-test", "Bushing", steel);

    part.dag.add_feature(
        FeatureOp::AddCylinder {
            radius: 0.025,
            height: 0.05,
            segments: 32,
            position: None,
            rotation_deg: None,
        },
        Some("Outer Cylinder".into()),
    );

    // Initially part has no cached mesh
    assert!(part.cached_mesh.is_none());
    assert!(part.cached_settings.is_none());

    // 1. Tessellate with Coarse quality
    let coarse = TessellationSettings::coarse();
    let mesh_coarse_ptr = {
        let m = part.tessellate(&coarse).unwrap();
        m as *const _
    };
    let coarse_tris = part.mesh().triangle_count();
    assert!(coarse_tris > 0);
    assert!(part.cached_mesh.is_some());
    assert_eq!(part.cached_settings.as_ref().unwrap(), &coarse);

    // 2. Calling tessellate with identical settings must hit the cache
    let mesh_coarse_ptr_2 = {
        let m = part.tessellate(&coarse).unwrap();
        m as *const _
    };
    assert_eq!(mesh_coarse_ptr, mesh_coarse_ptr_2, "Cache hit must return the same memory address");

    // 3. Tessellate with Fine quality updates the cache
    let fine = TessellationSettings::fine();
    part.tessellate(&fine).unwrap();
    let fine_tris = part.mesh().triangle_count();
    assert!(fine_tris > coarse_tris, "Fine mesh must have higher triangle count than Coarse");
    assert_eq!(part.cached_settings.as_ref().unwrap(), &fine);

    // 4. Rebuild preserves cached settings
    part.rebuild().unwrap();
    assert_eq!(part.mesh().triangle_count(), fine_tris);

    // 5. Invalidate cache resets it
    part.invalidate_cache();
    assert!(part.cached_mesh.is_none());
    assert!(part.cached_settings.is_none());
}

#[test]
fn test_export_part_stl_with_quality() {
    let registry = MaterialRegistry::new();
    let alu = registry.get("aluminum_6061_t6").unwrap().clone();
    let mut part = Part::new("p-export", "Turned Spindle", alu);

    part.dag.add_feature(
        FeatureOp::AddCylinder {
            radius: 0.03,
            height: 0.08,
            segments: 32,
            position: None,
            rotation_deg: None,
        },
        Some("Spindle Body".into()),
    );

    // 1. In-memory binary STL export with Coarse vs Fine
    let mut bin_coarse = Vec::new();
    export_part_stl_with_quality(&mut part, TessellationQuality::Coarse, &mut bin_coarse, true)
        .expect("Export binary STL Coarse failed");

    let mut bin_fine = Vec::new();
    export_part_stl_with_quality(&mut part, TessellationQuality::Fine, &mut bin_fine, true)
        .expect("Export binary STL Fine failed");

    assert!(
        bin_fine.len() > bin_coarse.len(),
        "Fine binary STL ({} bytes) must be larger than Coarse ({} bytes)",
        bin_fine.len(),
        bin_coarse.len()
    );

    let tri_coarse = u32::from_le_bytes([bin_coarse[80], bin_coarse[81], bin_coarse[82], bin_coarse[83]]);
    let tri_fine = u32::from_le_bytes([bin_fine[80], bin_fine[81], bin_fine[82], bin_fine[83]]);
    assert!(tri_fine > tri_coarse);
    assert_eq!(bin_coarse.len(), 84 + (tri_coarse as usize) * 50);
    assert_eq!(bin_fine.len(), 84 + (tri_fine as usize) * 50);

    // 2. In-memory ASCII STL export with Fine
    let mut ascii_buf = Vec::new();
    export_part_stl_with_quality(&mut part, TessellationQuality::Fine, &mut ascii_buf, false)
        .expect("Export ASCII STL Fine failed");
    let ascii_str = String::from_utf8(ascii_buf).expect("Invalid UTF-8 in ASCII STL");
    assert!(ascii_str.starts_with("solid Turned Spindle"));
    assert!(ascii_str.trim_end().ends_with("endsolid Turned Spindle"));

    // 3. Write directly to disk with write_part_stl_with_quality
    let tmp_file = NamedTempFile::new().unwrap();
    write_part_stl_with_quality(&mut part, TessellationQuality::Fine, tmp_file.path(), true)
        .expect("Write binary STL to disk failed");

    let metadata = std::fs::metadata(tmp_file.path()).unwrap();
    assert_eq!(metadata.len() as usize, bin_fine.len());
}
