//! Integration tests for MCP-CAD Core: Primitives, DAG re-evaluation, Materials, Mass, and STL Export.

use std::f64::consts::PI;
use std::fs::File;
use std::io::Read;
use mcp_cad_core::dag::{FeatureOp, FeatureTree};
use mcp_cad_core::export_stl::{
    export_stl_ascii, export_stl_binary, write_stl_ascii_file, write_stl_binary_file,
};
use mcp_cad_core::material::MaterialRegistry;
use mcp_cad_core::part::Part;
use mcp_cad_core::primitives::{
    create_box, create_cone, create_cylinder, create_helical_cutter, create_sphere,
};
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

    // 4. Cone volume: V = 1/3 * pi * h * (r1^2 + r1*r2 + r2^2)
    let (r_bot, r_top, h_cone) = (3.5, 1.5, 7.0);
    let cone_mesh = create_cone(r_bot, r_top, h_cone, 128);
    let v_cone_analytical = (1.0 / 3.0) * PI * h_cone * (r_bot * r_bot + r_bot * r_top + r_top * r_top);
    let v_cone_mesh = cone_mesh.volume();
    let cone_error_ratio = (v_cone_mesh - v_cone_analytical).abs() / v_cone_analytical;
    assert!(
        cone_error_ratio < 0.005,
        "Cone volume error ratio too high: computed={}, expected={}, ratio={}",
        v_cone_mesh,
        v_cone_analytical,
        cone_error_ratio
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
            position: None,
            rotation_deg: None,
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
            position: None,
            rotation_deg: None,
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
    steel_part.dag.add_feature(
        FeatureOp::AddBox {
            width: w,
            height: h,
            depth: d,
            position: None,
            rotation_deg: None,
        },
        None,
    );
    steel_part.rebuild().unwrap();

    assert!((steel_part.volume() - expected_vol).abs() < 1e-6);
    let expected_steel_mass = expected_vol * 7850.0; // 78.5 kg
    assert!((steel_part.mass() - expected_steel_mass).abs() < 1e-4);

    let mut alu_part = Part::new("p_alu", "Aluminum Bar", aluminum);
    alu_part.dag.add_feature(
        FeatureOp::AddBox {
            width: w,
            height: h,
            depth: d,
            position: None,
            rotation_deg: None,
        },
        None,
    );
    alu_part.rebuild().unwrap();

    let expected_alu_mass = expected_vol * 2700.0; // 27.0 kg
    assert!((alu_part.mass() - expected_alu_mass).abs() < 1e-4);

    let mut ti_part = Part::new("p_ti", "Titanium Bar", titanium);
    ti_part.dag.add_feature(
        FeatureOp::AddBox {
            width: w,
            height: h,
            depth: d,
            position: None,
            rotation_deg: None,
        },
        None,
    );
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
            position: None,
            rotation_deg: None,
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

#[test]
fn test_dag_cone_and_frustum_volume() {
    // 1. Sharp cone: r1 = 3.0, r2 = 0.0, h = 6.0
    // Analytical formula: V = 1/3 * pi * h * (r1^2 + r1*r2 + r2^2) = 1/3 * pi * 6.0 * 9.0 = 18 * pi
    let mut tree_sharp = FeatureTree::new();
    tree_sharp.add_feature(
        FeatureOp::AddCone {
            radius_bottom: 3.0,
            radius_top: 0.0,
            height: 6.0,
            segments: 64,
            position: None,
            rotation_deg: None,
        },
        Some("Sharp Cone".to_string()),
    );
    let sharp_mesh = tree_sharp.evaluate().expect("Sharp cone evaluation failed");
    let v_sharp_mesh = sharp_mesh.volume();
    let v_sharp_expected = (1.0 / 3.0) * PI * 6.0 * (3.0 * 3.0);
    let sharp_err = (v_sharp_mesh - v_sharp_expected).abs() / v_sharp_expected;
    assert!(
        sharp_err < 0.01,
        "Sharp cone volume error ratio too high: computed={}, expected={}, ratio={}",
        v_sharp_mesh,
        v_sharp_expected,
        sharp_err
    );

    // 2. Truncated cone (frustum): r1 = 4.0, r2 = 2.0, h = 5.0
    // Analytical formula: V = 1/3 * pi * 5.0 * (16.0 + 8.0 + 4.0) = 140/3 * pi
    let mut tree_frustum = FeatureTree::new();
    tree_frustum.add_feature(
        FeatureOp::AddCone {
            radius_bottom: 4.0,
            radius_top: 2.0,
            height: 5.0,
            segments: 64,
            position: Some([0.0, 0.0, 10.0]),
            rotation_deg: None,
        },
        Some("Frustum".to_string()),
    );
    let frustum_mesh = tree_frustum.evaluate().expect("Frustum evaluation failed");
    let v_frustum_mesh = frustum_mesh.volume();
    let v_frustum_expected = (1.0 / 3.0) * PI * 5.0 * (4.0 * 4.0 + 4.0 * 2.0 + 2.0 * 2.0);
    let frustum_err = (v_frustum_mesh - v_frustum_expected).abs() / v_frustum_expected;
    assert!(
        frustum_err < 0.01,
        "Frustum volume error ratio too high: computed={}, expected={}, ratio={}",
        v_frustum_mesh,
        v_frustum_expected,
        frustum_err
    );
    // Center along Z should be shifted to ~ 10.0
    let aabb = frustum_mesh.bounding_box();
    assert!((aabb.center()[2] - 10.0).abs() < 1e-4);
}

#[test]
fn test_dag_shifted_cylinder_stepped_shaft() {
    let mut tree = FeatureTree::new();

    // Shank (хвостовик): radius 2.0, height 10.0, positioned at z = 5.0 (spans z in [0, 10])
    let r_shank = 2.0;
    let h_shank = 10.0;
    tree.add_feature(
        FeatureOp::AddCylinder {
            radius: r_shank,
            height: h_shank,
            segments: 64,
            position: Some([0.0, 0.0, 5.0]),
            rotation_deg: None,
        },
        Some("Shank".to_string()),
    );

    // Body (тело вала): radius 4.0, height 20.0, positioned at z = 20.0 (spans z in [10, 30])
    let r_body = 4.0;
    let h_body = 20.0;
    tree.add_feature(
        FeatureOp::AddCylinder {
            radius: r_body,
            height: h_body,
            segments: 64,
            position: Some([0.0, 0.0, 20.0]),
            rotation_deg: None,
        },
        Some("Shaft Body".to_string()),
    );

    let shaft_mesh = tree.evaluate().expect("Stepped shaft evaluation failed");
    let vol_actual = shaft_mesh.volume();
    let vol_expected = PI * r_shank * r_shank * h_shank + PI * r_body * r_body * h_body;
    let err_ratio = (vol_actual - vol_expected).abs() / vol_expected;
    assert!(
        err_ratio < 0.01,
        "Stepped shaft volume mismatch: actual={}, expected={}, ratio={}",
        vol_actual,
        vol_expected,
        err_ratio
    );

    let aabb = shaft_mesh.bounding_box();
    assert!((aabb.min[2] - 0.0).abs() < 0.05);
    assert!((aabb.max[2] - 30.0).abs() < 0.05);
    assert!((aabb.dimensions()[2] - 30.0).abs() < 0.05);
}

#[test]
fn test_dag_helical_groove_drill_bit_model() {
    // 1. Direct helical cutter creation test
    let cutter = create_helical_cutter(4.5, 2.0, 30.0, 40.0, 5.0, 2, true, 40.0 / 30.0);
    assert!(cutter.triangle_count() > 0);
    assert!(cutter.volume() > 0.0);

    // 2. Full drill bit parametric DAG modeling:
    // Blank cylinder: radius 5.0, height 50.0, positioned at z = 25.0 (spans z in [0, 50])
    let mut tree = FeatureTree::new();
    let r_drill = 5.0;
    let h_drill = 50.0;
    tree.add_feature(
        FeatureOp::AddCylinder {
            radius: r_drill,
            height: h_drill,
            segments: 32,
            position: Some([0.0, 0.0, 25.0]),
            rotation_deg: None,
        },
        Some("Drill Blank".to_string()),
    );

    // Cut helical flutes into cylinder (2 starts, pitch 30.0, height 35.0, start_z 15.0)
    tree.add_feature(
        FeatureOp::CutHelicalGroove {
            helix_radius: 4.5,
            groove_radius: 2.0,
            pitch: 30.0,
            height: 35.0,
            start_z: 15.0,
            starts: 2,
            is_right_hand: true,
        },
        Some("Spiral Flutes".to_string()),
    );

    let drill_mesh = tree.evaluate().expect("Drill DAG evaluation failed");
    let vol_drill = drill_mesh.volume();
    let vol_blank = PI * r_drill * r_drill * h_drill;

    assert!(
        vol_drill < vol_blank,
        "Drill volume ({}) must be less than blank volume ({}) after groove cut",
        vol_drill,
        vol_blank
    );
    assert!(
        vol_drill > vol_blank * 0.5,
        "Drill core must remain solid: got {}",
        vol_drill
    );
    assert!(drill_mesh.triangle_count() > 0);

    // 3. Test BooleanDifferenceMesh with custom cutting tool
    let mut diff_tree = FeatureTree::new();
    diff_tree.add_feature(
        FeatureOp::AddBox {
            width: 10.0,
            height: 10.0,
            depth: 10.0,
            position: None,
            rotation_deg: None,
        },
        Some("Base Block".to_string()),
    );
    let tool_mesh = create_box(4.0, 4.0, 12.0);
    diff_tree.add_feature(
        FeatureOp::BooleanDifferenceMesh { tool_mesh },
        Some("Box Notch".to_string()),
    );
    let diff_mesh = diff_tree.evaluate().expect("BooleanDifferenceMesh evaluation failed");
    let vol_diff = diff_mesh.volume();
    // 1000 - 4*4*10 = 840
    assert!(
        (vol_diff - 840.0).abs() < 1e-4,
        "BooleanDifferenceMesh volume should be 840, got {}",
        vol_diff
    );
}
