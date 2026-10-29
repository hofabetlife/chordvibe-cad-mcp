//! Integration tests for MCP-CAD Core: Primitives, DAG re-evaluation, Materials, Mass, and STL Export.

use std::f64::consts::PI;
use std::fs::File;
use std::io::Read;
use mcp_cad_core::dag::{FeatureOp, FeatureTree};
use mcp_cad_core::export_stl::{export_stl_ascii, export_stl_binary, write_stl_ascii_file, write_stl_binary_file};
use mcp_cad_core::material::MaterialRegistry;
use mcp_cad_core::part::Part;
use mcp_cad_core::primitives::{create_box, create_cylinder, create_sphere};
use tempfile::NamedTempFile;

#[test]
fn test_primitive_volumes_analytical_verification() {
    // 1. Box volume: V = w * h * d
    let (w, h, d) = (2.5, 4.0, 3.2);
    let box_mesh = create_box(w, h, d);
    let v_box_analytical = w * h * d;
    let v_box_mesh = box_mesh.volume();
    assert!(
        (v_box_mesh - v_box_analytical).abs() < 1e-6,
        "Box volume mismatch: computed={}, expected={}",
        v_box_mesh,
        v_box_analytical
    );

    // 2. Cylinder volume: V = pi * r^2 * h
    let r_cyl = 1.8;
    let h_cyl = 6.0;
    let cyl_mesh = create_cylinder(r_cyl, h_cyl, 128);
    let v_cyl_analytical = PI * r_cyl * r_cyl * h_cyl;
    let v_cyl_mesh = cyl_mesh.volume();
    let cyl_error_ratio = (v_cyl_mesh - v_cyl_analytical).abs() / v_cyl_analytical;
    assert!(
        cyl_error_ratio < 0.005,
        "Cylinder volume error ratio too high: computed={}, expected={}, ratio={}",
        v_cyl_mesh,
        v_cyl_analytical,
        cyl_error_ratio
    );

    // 3. Sphere volume: V = (4/3) * pi * r^3
    let r_sph = 3.0;
    let sphere_mesh = create_sphere(r_sph, 64, 32);
    let v_sph_analytical = (4.0 / 3.0) * PI * r_sph.powi(3);
    let v_sph_mesh = sphere_mesh.volume();
    let sph_error_ratio = (v_sph_mesh - v_sph_analytical).abs() / v_sph_analytical;
    assert!(
        sph_error_ratio < 0.02,
        "Sphere volume error ratio too high: computed={}, expected={}, ratio={}",
        v_sph_mesh,
        v_sph_analytical,
        sph_error_ratio
    );
}

#[test]
fn test_dag_modify_feature_volume_update() {
    let mut tree = FeatureTree::new();

    // Step 1: Create initial box 2.0 x 3.0 x 5.0 -> Volume = 30.0
    let box_id = tree.add_feature(
        FeatureOp::AddBox {
            width: 2.0,
            height: 3.0,
            depth: 5.0,
        },
        Some("Initial Base Block".to_string()),
    );

    let mesh_initial = tree.evaluate().expect("Initial DAG evaluation failed");
    let vol_initial = mesh_initial.volume();
    assert!(
        (vol_initial - 30.0).abs() < 1e-5,
        "Initial volume should be 30.0, got {}",
        vol_initial
    );

    // Step 2: Modify feature node - increase height from 3.0 to 9.0 -> Expected Volume = 2.0 * 9.0 * 5.0 = 90.0
    tree.modify_feature(
        box_id,
        FeatureOp::AddBox {
            width: 2.0,
            height: 9.0,
            depth: 5.0,
        },
    ).expect("Modify feature failed");

    let mesh_updated = tree.evaluate().expect("DAG re-evaluation failed");
    let vol_updated = mesh_updated.volume();
    assert!(
        (vol_updated - 90.0).abs() < 1e-5,
        "Updated volume should be 90.0, got {}",
        vol_updated
    );

    // Verify node properties
    let node = tree.get_feature(box_id).expect("Node should exist in tree");
    assert_eq!(node.name, "Initial Base Block");
    assert!(node.enabled);
}

#[test]
fn test_part_mass_calculation_across_materials() {
    let registry = MaterialRegistry::new();

    // Steel AISI 1020: rho = 7850 kg/m³
    let steel = registry.get("steel_structural_aisi1020").expect("Steel missing").clone();
    // Aluminum 6061-T6: rho = 2700 kg/m³
    let aluminum = registry.get("aluminum_6061_t6").expect("Aluminum missing").clone();
    // Titanium Grade 5: rho = 4430 kg/m³
    let titanium = registry.get("titanium_grade5_ti6al4v").expect("Titanium missing").clone();

    // Standard test block: 0.2m x 0.5m x 0.1m -> V = 0.01 m³
    let (w, h, d) = (0.2, 0.5, 0.1);
    let expected_vol = w * h * d; // 0.01 m³

    let mut steel_part = Part::new("p_steel", "Steel Bar", steel);
    steel_part.dag.add_feature(FeatureOp::AddBox { width: w, height: h, depth: d }, None);
    steel_part.rebuild().unwrap();

    assert!((steel_part.volume() - expected_vol).abs() < 1e-6);
    let expected_steel_mass = expected_vol * 7850.0; // 78.5 kg
    assert!((steel_part.mass() - expected_steel_mass).abs() < 1e-4);

    let mut alu_part = Part::new("p_alu", "Aluminum Bar", aluminum);
    alu_part.dag.add_feature(FeatureOp::AddBox { width: w, height: h, depth: d }, None);
    alu_part.rebuild().unwrap();

    let expected_alu_mass = expected_vol * 2700.0; // 27.0 kg
    assert!((alu_part.mass() - expected_alu_mass).abs() < 1e-4);

    let mut ti_part = Part::new("p_ti", "Titanium Bar", titanium);
    ti_part.dag.add_feature(FeatureOp::AddBox { width: w, height: h, depth: d }, None);
    ti_part.rebuild().unwrap();

    let expected_ti_mass = expected_vol * 4430.0; // 44.3 kg
    assert!((ti_part.mass() - expected_ti_mass).abs() < 1e-4);

    // Center of mass should be origin [0, 0, 0]
    let com = steel_part.center_of_mass();
    assert!(com[0].abs() < 1e-6);
    assert!(com[1].abs() < 1e-6);
    assert!(com[2].abs() < 1e-6);
}

#[test]
fn test_stl_export_structure_ascii_and_binary() {
    let mesh = create_box(1.5, 2.5, 3.5);
    assert_eq!(mesh.triangle_count(), 12);

    // 1. In-memory ASCII STL check
    let mut ascii_buf = Vec::new();
    export_stl_ascii(&mesh, "bracket_test", &mut ascii_buf).expect("ASCII export failed");
    let ascii_str = String::from_utf8(ascii_buf).expect("Invalid UTF-8 in ASCII STL");

    assert!(ascii_str.starts_with("solid bracket_test\n"));
    assert!(ascii_str.trim_end().ends_with("endsolid bracket_test"));
    // Count facet occurrences: 12 triangles -> 12 facets
    let facet_count = ascii_str.matches("facet normal").count();
    assert_eq!(facet_count, 12);
    let vertex_count = ascii_str.matches("vertex ").count();
    assert_eq!(vertex_count, 36); // 12 * 3 = 36 vertices

    // 2. In-memory Binary STL check
    let mut bin_buf = Vec::new();
    export_stl_binary(&mesh, &mut bin_buf).expect("Binary export failed");

    // Standard binary STL length: 80 (header) + 4 (tri count) + 50 * N
    let expected_bin_size = 80 + 4 + 12 * 50;
    assert_eq!(bin_buf.len(), expected_bin_size);

    // Check triangle count in binary STL
    let recorded_tri_count = u32::from_le_bytes([bin_buf[80], bin_buf[81], bin_buf[82], bin_buf[83]]);
    assert_eq!(recorded_tri_count, 12);

    // 3. Disk file writing verification with tempfile
    let tmp_ascii = NamedTempFile::new().unwrap();
    write_stl_ascii_file(&mesh, "file_bracket", tmp_ascii.path()).unwrap();
    let mut file_ascii = File::open(tmp_ascii.path()).unwrap();
    let mut content_ascii = String::new();
    file_ascii.read_to_string(&mut content_ascii).unwrap();
    assert!(content_ascii.contains("solid file_bracket"));

    let tmp_bin = NamedTempFile::new().unwrap();
    write_stl_binary_file(&mesh, tmp_bin.path()).unwrap();
    let mut file_bin = File::open(tmp_bin.path()).unwrap();
    let mut content_bin = Vec::new();
    file_bin.read_to_end(&mut content_bin).unwrap();
    assert_eq!(content_bin.len(), expected_bin_size);
}

#[test]
fn test_dag_cut_hole_evaluation() {
    let mut tree = FeatureTree::new();

    // Box: 10 x 10 x 10 = volume 1000
    tree.add_feature(
        FeatureOp::AddBox {
            width: 10.0,
            height: 10.0,
            depth: 10.0,
        },
        Some("Cube".to_string()),
    );

    // Cut hole on top face: center = [0, 5, 0], normal = [0, 1, 0], radius = 2.0, depth = 10.0
    tree.add_feature(
        FeatureOp::CutHole {
            center: [0.0, 5.0, 0.0],
            normal: [0.0, 1.0, 0.0],
            radius: 2.0,
            depth: 10.0,
        },
        Some("Thru Hole".to_string()),
    );

    let final_mesh = tree.evaluate().expect("DAG with CutHole evaluation failed");
    let vol = final_mesh.volume();

    // Hole cylinder volume ~ pi * 2^2 * 10 = 40 * pi ~ 125.66
    // Solid volume should be ~ 1000 - 125.66 ~ 874.34
    assert!(
        vol < 1000.0,
        "Volume after cut hole should be less than 1000, got {}",
        vol
    );
    assert!(
        vol > 800.0,
        "Volume after cut hole should be around 874, got {}",
        vol
    );
}
