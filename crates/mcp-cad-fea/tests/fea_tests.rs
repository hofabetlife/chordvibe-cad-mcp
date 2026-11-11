//! Verification and validation tests for FEA simulation engine.
//! Compares numerical FEA results against classical analytical solutions from strength of materials.

use mcp_cad_core::material::Material;
use mcp_cad_core::part::Part;
use mcp_cad_core::primitives::create_box;
use mcp_cad_fea::boundary::{AabbFace, BoundaryConditions};
use mcp_cad_fea::solver::SolverOptions;
use mcp_cad_fea::tetra_mesh::{MeshDensity, TetrahedralMesh};
use mcp_cad_fea::{analyze_mesh, analyze_part};

#[test]
fn test_cantilever_beam_analytical() {
    // Beam dimensions (meters)
    let length = 1.0f64;
    let height = 0.1f64;
    let width = 0.1f64;

    // Material: AISI 1020 Structural Steel
    let youngs_modulus = 205.0e9f64; // 205 GPa
    let poissons_ratio = 0.29f64;
    let density = 7850.0f64;
    let material = Material::new(
        "steel_structural_aisi1020",
        205.0,
        poissons_ratio,
        250.0,
        density,
        "Structural steel",
    );

    // Mesh: rectangular beam box [0, 0, 0] to [1.0, 0.1, 0.1]
    let mesh = TetrahedralMesh::from_box(
        [0.0, 0.0, 0.0],
        [length, height, width],
        30,
        6,
        4,
    )
    .expect("Mesh generation should succeed");

    // Theoretical analytical values:
    // Second moment of area: I = b * h^3 / 12
    let i_moment = width * height.powi(3) / 12.0;
    let force_y = -1000.0f64; // -1000 N downwards
    // Deflection: delta = |F| * L^3 / (3 * E * I)
    let expected_deflection = force_y.abs() * length.powi(3) / (3.0 * youngs_modulus * i_moment);
    // Max bending stress at root: sigma = 6 * |F| * L / (b * h^2)
    let expected_bending_stress = 6.0 * force_y.abs() * length / (width * height.powi(2));

    println!("Theoretical Deflection: {:.6} mm", expected_deflection * 1000.0);
    println!("Theoretical Max Stress: {:.2} MPa", expected_bending_stress / 1.0e6);

    // Boundary conditions:
    // Fixed support at x = 0 (MinX)
    // Downward transverse load Fy = -1000 N at x = L (MaxX)
    let mut bc = BoundaryConditions::new();
    bc.fix_aabb_face(AabbFace::MinX, 1e-4);
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, force_y, 0.0]);

    let solver_opts = SolverOptions {
        tolerance: 1e-8,
        max_iterations: 8000,
    };

    let report = analyze_mesh("cantilever_beam", &mesh, &material, &bc, Some(solver_opts))
        .expect("FEA solve should succeed");

    println!("Numerical Max Deflection: {:.6} mm", report.max_displacement_m * 1000.0);
    println!("Numerical Max von Mises Stress: {:.2} MPa", report.max_stress_von_mises_pa / 1.0e6);
    println!("Factor of Safety: {:.2}", report.factor_of_safety);
    println!("Verdict: {}", report.verdict);

    // Also inspect element stresses
    let d = mcp_cad_fea::elasticity::elasticity_matrix_from_material(&material).unwrap();
    let mut max_elem_stress = 0.0;
    for elem in &mesh.elements {
        let mut u_elem = nalgebra::SVector::<f64, 12>::zeros();
        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            u_elem[3 * local_i] = report.nodal_displacements[global_node][0];
            u_elem[3 * local_i + 1] = report.nodal_displacements[global_node][1];
            u_elem[3 * local_i + 2] = report.nodal_displacements[global_node][2];
        }
        let tet = mcp_cad_fea::element::Tet4Element::new(
            elem.id,
            elem.node_indices,
            [
                mesh.nodes[elem.node_indices[0]].position,
                mesh.nodes[elem.node_indices[1]].position,
                mesh.nodes[elem.node_indices[2]].position,
                mesh.nodes[elem.node_indices[3]].position,
            ],
        ).unwrap();
        let s = tet.compute_stress(&d, &u_elem);
        let vm = mcp_cad_fea::element::Tet4Element::von_mises_stress(&s);
        if vm > max_elem_stress {
            max_elem_stress = vm;
        }
    }
    println!("Max Element von Mises Stress: {:.2} MPa", max_elem_stress / 1.0e6);

    // For linear TET4 elements, deflection in bending shows standard TET4 shear stiffness
    // Typically within 15-25% of Euler-Bernoulli beam theory on moderate grid
    let deflection_rel_error = (report.max_displacement_m - expected_deflection).abs() / expected_deflection;
    println!("Deflection relative difference: {:.1}%", deflection_rel_error * 100.0);

    // Verify order of magnitude and qualitative agreement
    assert!(
        deflection_rel_error < 0.35,
        "Deflection error ({:.1}%) should be within acceptable TET4 range (< 35%)",
        deflection_rel_error * 100.0
    );

    // Max stress should be within 25% of analytical root bending stress
    let stress_rel_error = (report.max_stress_von_mises_pa - expected_bending_stress).abs() / expected_bending_stress;
    println!("Stress relative difference: {:.1}%", stress_rel_error * 100.0);
    assert!(
        stress_rel_error < 0.25,
        "Stress error ({:.1}%) should be within acceptable range (< 25%)",
        stress_rel_error * 100.0
    );

    assert_eq!(report.verdict, "PASS");
}

#[test]
fn test_axial_tension_analytical() {
    // Rod dimensions: L = 1.0 m, cross section 0.1 x 0.1 m
    let length = 1.0f64;
    let height = 0.1f64;
    let width = 0.1f64;
    let area = height * width; // 0.01 m²

    let youngs_modulus = 205.0e9f64;
    let poissons_ratio = 0.29f64;
    let material = Material::new(
        "steel_structural_aisi1020",
        205.0,
        poissons_ratio,
        250.0,
        7850.0,
        "Structural steel",
    );

    let mesh = TetrahedralMesh::from_box(
        [0.0, 0.0, 0.0],
        [length, height, width],
        10,
        2,
        2,
    )
    .expect("Mesh generation should succeed");

    // Axial tensile force Fx = +100 kN
    let force_x = 100_000.0f64;
    let expected_stress = force_x / area; // 10.0 MPa
    let expected_elongation = force_x * length / (youngs_modulus * area); // F * L / (E * A)

    println!("Theoretical Axial Stress: {:.2} MPa", expected_stress / 1.0e6);
    println!("Theoretical Elongation: {:.6} mm", expected_elongation * 1000.0);

    let mut bc = BoundaryConditions::new();
    // Fix Ux = 0 across the entire face at x = 0 (prevents axial motion)
    bc.dirichlet.push(mcp_cad_fea::boundary::DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinX,
        tolerance: 1e-4,
        components: mcp_cad_fea::boundary::DofComponent::X,
    });
    // Fix Uy and Uz on the center line/node at x = 0 to prevent rigid body translation/rotation
    // while allowing Poisson lateral contraction
    let center_nodes = mesh.find_nodes_by_predicate(|p| {
        p[0].abs() <= 1e-4 && (p[1] - 0.05).abs() <= 0.026 && (p[2] - 0.05).abs() <= 0.026
    });
    for cn in center_nodes {
        bc.dirichlet.push(mcp_cad_fea::boundary::DirichletConstraint::FixedNodes {
            node_ids: vec![cn],
            components: mcp_cad_fea::boundary::DofComponent::Y,
        });
        bc.dirichlet.push(mcp_cad_fea::boundary::DirichletConstraint::FixedNodes {
            node_ids: vec![cn],
            components: mcp_cad_fea::boundary::DofComponent::Z,
        });
    }
    // Tensile load at x = L
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [force_x, 0.0, 0.0]);

    let solver_opts = SolverOptions {
        tolerance: 1e-8,
        max_iterations: 5000,
    };

    let report = analyze_mesh("axial_rod", &mesh, &material, &bc, Some(solver_opts))
        .expect("Axial FEA solve should succeed");

    println!("Numerical Max Elongation: {:.6} mm", report.max_displacement_m * 1000.0);
    println!("Numerical Max Stress: {:.2} MPa", report.max_stress_von_mises_pa / 1.0e6);
    println!(
        "Critical node stress index: {}, position: {:?}",
        report.critical_node_stress, mesh.nodes[report.critical_node_stress].position
    );
    // Print stress at a middle node x ~ 0.5
    let mid_node = mesh
        .find_nodes_by_predicate(|p| (p[0] - 0.5).abs() <= 1e-4)
        .first()
        .copied();
    if let Some(mn) = mid_node {
        println!(
            "Middle node #{} at {:?} has stress: {:.2} MPa",
            mn,
            mesh.nodes[mn].position,
            report.nodal_von_mises_stresses[mn] / 1.0e6
        );
    }

    // In pure axial tension, TET4 elements represent the exact linear displacement field!
    let elongation_rel_error = (report.max_displacement_m - expected_elongation).abs() / expected_elongation;
    println!("Axial elongation error: {:.2}%", elongation_rel_error * 100.0);
    assert!(
        elongation_rel_error < 0.05,
        "Axial elongation should match theoretical formula to within 5%"
    );

    // Stress should match F / A very closely
    let stress_rel_error = (report.max_stress_von_mises_pa - expected_stress).abs() / expected_stress;
    println!("Axial stress error: {:.2}%", stress_rel_error * 100.0);
    assert!(
        stress_rel_error < 0.15,
        "Axial stress should match theoretical F/A to within 15%"
    );

    assert_eq!(report.verdict, "PASS");
}

#[test]
fn test_factor_of_safety_pass_and_fail() {
    let material = Material::new(
        "steel_structural_aisi1020",
        205.0,
        0.29,
        250.0, // Yield strength 250 MPa
        7850.0,
        "Structural steel",
    );

    let mesh = TetrahedralMesh::from_box(
        [0.0, 0.0, 0.0],
        [1.0, 0.1, 0.1],
        8,
        2,
        2,
    )
    .unwrap();

    // 1. Small load (Fx = 10 kN): stress ~ 1 MPa << 250 MPa -> PASS
    let mut bc_pass = BoundaryConditions::new();
    bc_pass.fix_aabb_face(AabbFace::MinX, 1e-4);
    bc_pass.add_aabb_face_force(AabbFace::MaxX, 1e-4, [10_000.0, 0.0, 0.0]);

    let report_pass = analyze_mesh("pass_beam", &mesh, &material, &bc_pass, None).unwrap();
    assert_eq!(report_pass.verdict, "PASS");
    assert!(report_pass.factor_of_safety > 10.0);

    // 2. Huge load (Fx = 5 MN): stress ~ 500 MPa > 250 MPa -> FAIL
    let mut bc_fail = BoundaryConditions::new();
    bc_fail.fix_aabb_face(AabbFace::MinX, 1e-4);
    bc_fail.add_aabb_face_force(AabbFace::MaxX, 1e-4, [5_000_000.0, 0.0, 0.0]);

    let report_fail = analyze_mesh("fail_beam", &mesh, &material, &bc_fail, None).unwrap();
    assert_eq!(report_fail.verdict, "FAIL");
    assert!(report_fail.factor_of_safety < 1.0);
}

#[test]
fn test_analyze_part_pipeline() {
    let material = Material::new(
        "aluminum_6061_t6",
        68.9,
        0.33,
        276.0,
        2700.0,
        "6061-T6 Aluminum",
    );

    let mut part = Part::new("test_beam", "Aluminum Cantilever", material);
    part.mesh = create_box(0.5, 0.05, 0.05);

    let mut bc = BoundaryConditions::new();
    bc.fix_aabb_face(AabbFace::MinX, 1e-4);
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, -500.0, 0.0]);

    let report = analyze_part(&part, &bc, MeshDensity::Coarse, None)
        .expect("analyze_part should mesh and solve successfully");

    assert!(report.node_count > 0);
    assert!(report.element_count > 0);
    assert!(report.max_displacement_m > 0.0);
    assert!(report.max_stress_von_mises_pa > 0.0);
    assert_eq!(report.normalized_stresses.len(), report.node_count);
    assert!(!report.to_json().unwrap().is_empty());
}

#[test]
fn test_adaptive_fea_part_pipeline() {
    use mcp_cad_core::dag::FeatureOp;
    use mcp_cad_fea::{analyze_part_adaptive, analyze_part_with_settings, TessellationSettings};

    let material = Material::new(
        "steel_structural_aisi1020",
        205.0,
        0.29,
        250.0,
        7850.0,
        "Structural steel",
    );

    let mut part = Part::new("param_beam", "Parametric Cantilever", material);
    part.dag.add_feature(
        FeatureOp::AddBox {
            width: 0.6,
            height: 0.08,
            depth: 0.08,
            position: None,
            rotation_deg: None,
        },
        Some("cantilever".to_string()),
    );

    let mut bc = BoundaryConditions::new();
    bc.fix_aabb_face(AabbFace::MinX, 1e-4);
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, -1000.0, 0.0]);

    // Test analyze_part_adaptive
    let report = analyze_part_adaptive(&mut part, &bc, MeshDensity::Coarse, None)
        .expect("analyze_part_adaptive should succeed with parametric DAG");

    assert!(report.node_count > 0);
    assert!(report.element_count > 0);
    assert!(report.max_displacement_m > 0.0);
    assert!(part.cached_mesh.is_some());

    // Test analyze_part_with_settings
    let fine_settings = TessellationSettings::fine().with_max_edge_length(Some(0.04));
    let report_fine = analyze_part_with_settings(&mut part, &bc, &fine_settings, MeshDensity::Coarse, None)
        .expect("analyze_part_with_settings should succeed");
    assert!(report_fine.node_count > 0);
}

#[test]
fn test_mesh_edge_refinement() {
    use mcp_cad_fea::{mesh_max_edge_length, refine_mesh_max_edge_length};

    let box_mesh = create_box(1.0, 1.0, 1.0);
    let initial_max_len = mesh_max_edge_length(&box_mesh);
    // In a 1x1x1 box, face diagonal is sqrt(1^2 + 1^2) ~ 1.414
    assert!(initial_max_len > 1.0);

    let refined = refine_mesh_max_edge_length(&box_mesh, 0.8);
    let refined_max_len = mesh_max_edge_length(&refined);
    assert!(
        refined_max_len <= 0.8,
        "Refined maximum edge length ({}) must be <= 0.8",
        refined_max_len
    );
    assert!(refined.indices.len() > box_mesh.indices.len());
    assert_eq!(
        refined.face_ids.len(),
        refined.indices.len(),
        "Refined mesh must preserve face_ids per triangle"
    );
    assert_eq!(
        refined.faces.len(),
        box_mesh.faces.len(),
        "Refined mesh must preserve all logical FaceInfo entries"
    );
}

#[test]
fn test_p2_t8_cylindrical_shaft_axial_tension_by_named_faces() {
    use mcp_cad_core::primitives::create_cylinder;
    use mcp_cad_fea::{extract_face_triangles, DofComponent};

    let e_gpa = 205.0;
    let e_pa = e_gpa * 1.0e9;
    let material = Material::new(
        "steel_structural_aisi1020",
        e_gpa,
        0.29,
        250.0,
        7850.0,
        "Structural steel",
    );

    let radius = 0.025; // 25 mm
    let length = 0.25; // 250 mm
    let cyl_mesh = create_cylinder(radius, length, 48);

    let cap_bottom = cyl_mesh
        .faces
        .iter()
        .find(|f| f.role == "cap_bottom")
        .expect("Cylinder must have cap_bottom face");
    let cap_top = cyl_mesh
        .faces
        .iter()
        .find(|f| f.role == "cap_top")
        .expect("Cylinder must have cap_top face");

    let bottom_tris = extract_face_triangles(&cyl_mesh, &[cap_bottom.id]);
    let top_tris = extract_face_triangles(&cyl_mesh, &[cap_top.id]);
    assert!(!bottom_tris.is_empty());
    assert!(!top_tris.is_empty());

    let tet_mesh = TetrahedralMesh::from_mesh(&cyl_mesh, MeshDensity::Medium)
        .expect("Cylinder tetrahedral mesh should succeed");

    let force_z = 50_000.0; // 50 kN axial tension
    let mut bc = BoundaryConditions::new();
    bc.fix_named_face(&cyl_mesh, cap_bottom.id, Some("cap_bottom".to_string()), 0.0, DofComponent::All);
    bc.add_named_face_force(&cyl_mesh, cap_top.id, Some("cap_top".to_string()), 0.0, [0.0, 0.0, force_z]);

    // 1. Verify exact nodal force conservation (< 1e-9 relative error)
    let resolved = bc.resolve(&tet_mesh).expect("Named face BCs must resolve");
    let mut sum_f = [0.0_f64; 3];
    for nf in &resolved.nodal_forces {
        sum_f[0] += nf[0];
        sum_f[1] += nf[1];
        sum_f[2] += nf[2];
    }
    let force_rel_err = ((sum_f[2] - force_z).powi(2) + sum_f[0].powi(2) + sum_f[1].powi(2)).sqrt() / force_z;
    assert!(
        force_rel_err < 1e-9,
        "Sum of nodal forces {sum_f:?} must match total force [0, 0, {force_z}] within 1e-9 (rel_err = {force_rel_err:e})"
    );

    // 2. Solve FEA and compare against analytical axial tension:
    //    sigma = F / A, delta = F * L / (E * A)
    let report = analyze_mesh("cyl_shaft", &tet_mesh, &material, &bc, None)
        .expect("FEA solve on cylindrical shaft must converge");

    let area = std::f64::consts::PI * radius * radius;
    let expected_sigma_pa = force_z / area;
    let expected_delta_m = (force_z * length) / (e_pa * area);

    let disp_rel_err = (report.max_displacement_m - expected_delta_m).abs() / expected_delta_m;
    assert!(
        disp_rel_err < 0.05,
        "Cylindrical shaft axial elongation {} m must match analytical {} m within 5% (rel_err = {:.2}%)",
        report.max_displacement_m,
        expected_delta_m,
        disp_rel_err * 100.0
    );

    let mean_stress_pa: f64 =
        report.nodal_von_mises_stresses.iter().sum::<f64>() / (report.nodal_von_mises_stresses.len() as f64);
    let stress_rel_err = (mean_stress_pa - expected_sigma_pa).abs() / expected_sigma_pa;
    assert!(
        stress_rel_err < 0.15,
        "Cylindrical shaft mean axial stress {:.2} MPa must match analytical {:.2} MPa within 15% (rel_err = {:.2}%)",
        mean_stress_pa / 1e6,
        expected_sigma_pa / 1e6,
        stress_rel_err * 100.0
    );

    // 3. Verify pure torsion moment on cap_top conserves zero net force and exact resultant torque
    let torque_nm = [0.0, 0.0, 250.0];
    let mut bc_moment = BoundaryConditions::new();
    bc_moment.fix_surface_triangles(Some("cap_bottom".to_string()), bottom_tris, 0.0, DofComponent::All);
    bc_moment.add_surface_triangles_moment(Some("cap_top".to_string()), top_tris, 0.0, torque_nm);
    let resolved_m = bc_moment.resolve(&tet_mesh).expect("Moment BC must resolve");
    let mut net_f = [0.0_f64; 3];
    let mut net_m = [0.0_f64; 3];
    for (nid, f) in resolved_m.nodal_forces.iter().enumerate() {
        net_f[0] += f[0];
        net_f[1] += f[1];
        net_f[2] += f[2];
        let p = tet_mesh.nodes[nid].position;
        let rx = p[0] - cap_top.centroid[0];
        let ry = p[1] - cap_top.centroid[1];
        let rz = p[2] - cap_top.centroid[2];
        net_m[0] += ry * f[2] - rz * f[1];
        net_m[1] += rz * f[0] - rx * f[2];
        net_m[2] += rx * f[1] - ry * f[0];
    }
    assert!(
        (net_f[0].powi(2) + net_f[1].powi(2) + net_f[2].powi(2)).sqrt() < 1e-8,
        "Pure moment load must have zero net translational force, got {net_f:?}"
    );
    let torque_rel_err = (net_m[2] - torque_nm[2]).abs() / torque_nm[2];
    assert!(
        torque_rel_err < 1e-9,
        "Resultant nodal torque {net_m:?} must match {torque_nm:?} within 1e-9 (rel_err = {torque_rel_err:e})"
    );
}

#[test]
fn test_p2_t8_plate_with_hole_bearing_load_and_pressure_on_hole_wall() {
    use mcp_cad_core::primitives::cut_hole;
    use mcp_cad_fea::{extract_face_triangles, DofComponent};

    let material = Material::new(
        "steel_structural_aisi1020",
        205.0,
        0.29,
        250.0,
        7850.0,
        "Structural steel",
    );

    let plate = create_box(0.10, 0.08, 0.02);
    let perforated = cut_hole(&plate, [0.0, 0.0, -0.015], [0.0, 0.0, 1.0], 0.015, 0.03, None, None);

    let left_face = perforated
        .faces
        .iter()
        .find(|f| f.role == "left")
        .expect("Perforated plate must preserve 'left' face");
    let hole_wall = perforated
        .faces
        .iter()
        .find(|f| f.role == "hole_wall")
        .expect("Perforated plate must have 'hole_wall' face");

    let left_tris = extract_face_triangles(&perforated, &[left_face.id]);
    let hole_tris = extract_face_triangles(&perforated, &[hole_wall.id]);

    let tet_mesh = TetrahedralMesh::from_mesh(&perforated, MeshDensity::Coarse)
        .expect("Perforated plate tetrahedral mesh should succeed");

    // 1. Bearing load [Fx, 0, 0] on hole_wall
    let bearing_force = [12_000.0, 0.0, 0.0];
    let mut bc_force = BoundaryConditions::new();
    bc_force.fix_surface_triangles(Some(left_face.name.clone()), left_tris.clone(), 0.0, DofComponent::All);
    bc_force.add_surface_triangles_force(Some(hole_wall.name.clone()), hole_tris.clone(), 0.0, bearing_force);

    let resolved_f = bc_force
        .resolve(&tet_mesh)
        .expect("Hole wall bearing force must match surface nodes");
    let mut sum_fx = 0.0_f64;
    for nf in &resolved_f.nodal_forces {
        sum_fx += nf[0];
    }
    let rel_err = (sum_fx - bearing_force[0]).abs() / bearing_force[0];
    assert!(
        rel_err < 1e-9,
        "Hole wall nodal force sum {sum_fx} N must equal {} N within 1e-9",
        bearing_force[0]
    );

    let report_force = analyze_mesh("perforated_bearing", &tet_mesh, &material, &bc_force, None)
        .expect("FEA solve with hole_wall bearing load must succeed");
    assert!(report_force.max_displacement_m > 0.0);
    assert!(report_force.max_stress_von_mises_pa > 0.0);

    // 2. Internal pressure load on hole_wall
    let mut bc_pressure = BoundaryConditions::new();
    bc_pressure.fix_surface_triangles(Some(left_face.name.clone()), left_tris, 0.0, DofComponent::All);
    bc_pressure.add_surface_triangles_pressure(Some(hole_wall.name.clone()), hole_tris, 0.0, 25.0e6); // 25 MPa

    let report_pressure = analyze_mesh("perforated_pressure", &tet_mesh, &material, &bc_pressure, None)
        .expect("FEA solve with hole_wall pressure load must succeed");
    assert!(report_pressure.max_displacement_m > 0.0);
    assert!(report_pressure.max_stress_von_mises_pa > 1.0e6);
}

#[test]
fn test_p3_t2_preflight_mesh_validation_equilibrium_and_singularity_zone() {
    use mcp_cad_fea::boundary::{DirichletConstraint, DofComponent};
    use mcp_cad_fea::FeaError;

    let material = Material::new(
        "steel_structural_aisi1020",
        205.0,
        0.30,
        250.0,
        7850.0,
        "Structural steel",
    );

    // 1. Open (non-watertight) surface mesh must be rejected by TetrahedralMesh::from_mesh with FeaError::InvalidSurfaceMesh
    let mut open_box = create_box(0.1, 0.02, 0.02);
    open_box.indices.pop(); // Remove 1 triangle -> 3 open boundary edges
    let err = TetrahedralMesh::from_mesh(&open_box, MeshDensity::Coarse)
        .expect_err("Non-watertight mesh must be rejected before voxel meshing");
    match err {
        FeaError::InvalidSurfaceMesh {
            boundary_edges_count,
            is_watertight,
            ..
        } => {
            assert!(!is_watertight);
            assert_eq!(boundary_edges_count, 3);
        }
        other => panic!("Expected FeaError::InvalidSurfaceMesh, got: {other:?}"),
    }

    // 2. Axial tension with minimal kinematic constraints:
    //    equilibrium_error_pct < 1e-4, reaction_total == [-Fx, 0, 0], constrained_rigid_body_modes_ok == true
    let rod_mesh = TetrahedralMesh::from_box([0.0, 0.0, 0.0], [1.0, 0.01, 0.01], 20, 2, 2).unwrap();
    let force_x = 2050.0;
    let mut bc_axial = BoundaryConditions::new();
    bc_axial.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinX,
        tolerance: 1e-5,
        components: DofComponent::X,
    });
    // Pin (0,0,0) in Y,Z and (0, 0.01, 0) in Z to eliminate rigid body modes without Poisson over-constraint
    let n_origin = rod_mesh
        .find_nodes_by_predicate(|p| p[0].abs() < 1e-6 && p[1].abs() < 1e-6 && p[2].abs() < 1e-6)
        .into_iter()
        .next()
        .unwrap();
    let n_y = rod_mesh
        .find_nodes_by_predicate(|p| p[0].abs() < 1e-6 && (p[1] - 0.01).abs() < 1e-6 && p[2].abs() < 1e-6)
        .into_iter()
        .next()
        .unwrap();
    bc_axial.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: vec![n_origin],
        components: DofComponent::YZ,
    });
    bc_axial.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: vec![n_y],
        components: DofComponent::Z,
    });
    bc_axial.add_aabb_face_force(AabbFace::MaxX, 1e-5, [force_x, 0.0, 0.0]);

    let rep_axial = analyze_mesh("axial_p3t2", &rod_mesh, &material, &bc_axial, None).unwrap();
    assert!(rep_axial.diagnostics.mesh_closed);
    assert!(rep_axial.diagnostics.constrained_rigid_body_modes_ok);
    assert!(
        rep_axial.diagnostics.equilibrium_error_pct < 1e-4,
        "Axial tension equilibrium_error_pct ({:e}%) must be < 1e-4",
        rep_axial.diagnostics.equilibrium_error_pct
    );
    assert!(
        (rep_axial.diagnostics.applied_total[0] - force_x).abs() < 1e-6,
        "Applied total Fx should be {force_x}, got {:?}",
        rep_axial.diagnostics.applied_total
    );
    assert!(
        (rep_axial.diagnostics.reaction_total[0] + force_x).abs() < 1e-3,
        "Reaction total Rx should be -{force_x}, got {:?}",
        rep_axial.diagnostics.reaction_total
    );
    assert!(rep_axial.diagnostics.element_quality.min > 0.05);
    assert!(rep_axial.diagnostics.element_quality.mean > 0.10);

    // 3. Cantilever beam with clamped MinX support:
    //    equilibrium_error_pct < 1e-4, critical_node_near_constraint == true,
    //    warning about singularity zone present, max_stress_excluding_constraint_zone < max_stress_von_mises_pa,
    //    confidence == "low", verdict == "PASS"
    let cant_box = create_box(0.2, 0.02, 0.02);
    let cant_tet = TetrahedralMesh::from_mesh(&cant_box, MeshDensity::Medium).unwrap();
    let mut bc_cant = BoundaryConditions::new();
    bc_cant.fix_aabb_face(AabbFace::MinX, 1e-4);
    bc_cant.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, -1000.0, 0.0]);

    let rep_cant = analyze_mesh("cantilever_p3t2", &cant_tet, &material, &bc_cant, None).unwrap();
    assert_eq!(rep_cant.verdict, "PASS");
    assert_eq!(rep_cant.confidence, "low");
    assert!(rep_cant.diagnostics.mesh_closed);
    assert!(rep_cant.diagnostics.constrained_rigid_body_modes_ok);
    assert!(
        rep_cant.diagnostics.equilibrium_error_pct < 1e-4,
        "Cantilever equilibrium_error_pct ({:e}%) must be < 1e-4",
        rep_cant.diagnostics.equilibrium_error_pct
    );
    assert!(
        (rep_cant.diagnostics.applied_total[1] - (-1000.0)).abs() < 1e-6,
        "Cantilever applied Fy must be -1000 N"
    );
    assert!(
        (rep_cant.diagnostics.reaction_total[1] - 1000.0).abs() < 1e-3,
        "Cantilever reaction Ry must be +1000 N"
    );
    assert!(
        rep_cant.diagnostics.moment_equilibrium_error_pct < 1e-3,
        "Cantilever moment equilibrium error ({:e}%) must be < 1e-3%",
        rep_cant.diagnostics.moment_equilibrium_error_pct
    );
    assert!(
        rep_cant.diagnostics.critical_node_near_constraint,
        "Cantilever peak bending stress at fixed root must flag critical_node_near_constraint = true"
    );
    assert!(
        rep_cant.diagnostics.max_stress_excluding_constraint_zone > 0.0
            && rep_cant.diagnostics.max_stress_excluding_constraint_zone
                < rep_cant.max_stress_von_mises_pa,
        "max_stress_excluding_constraint_zone ({:.2} MPa) must be positive and strictly less than peak root stress ({:.2} MPa)",
        rep_cant.diagnostics.max_stress_excluding_constraint_zone / 1e6,
        rep_cant.max_stress_von_mises_pa / 1e6
    );
    assert!(
        rep_cant
            .diagnostics
            .notes
            .iter()
            .any(|n| n.contains("CONSTRAINT_SINGULARITY")),
        "Diagnostics notes must include CONSTRAINT_SINGULARITY warning, got: {:?}",
        rep_cant.diagnostics.notes
    );

    // 4. Thin-walled part (min_dim <= 5 mm, e.g. 40 x 20 x 0.4 mm):
    //    Coarse, Medium, and Fine presets must produce distinct element sizes and element counts
    let thin_plate = create_box(0.04, 0.02, 0.0004);
    let aabb_thin = thin_plate.bounding_box();
    let h_c = MeshDensity::Coarse.target_element_size(&aabb_thin);
    let h_m = MeshDensity::Medium.target_element_size(&aabb_thin);
    let h_f = MeshDensity::Fine.target_element_size(&aabb_thin);
    assert!(
        h_c > h_m && h_m > h_f,
        "Thin-walled target element sizes must be strictly decreasing: Coarse={h_c}, Medium={h_m}, Fine={h_f}"
    );
    let tet_c = TetrahedralMesh::from_mesh(&thin_plate, MeshDensity::Coarse).unwrap();
    let tet_m = TetrahedralMesh::from_mesh(&thin_plate, MeshDensity::Medium).unwrap();
    let tet_f = TetrahedralMesh::from_mesh(&thin_plate, MeshDensity::Fine).unwrap();
    assert!(
        tet_c.element_count() < tet_m.element_count()
            && tet_m.element_count() < tet_f.element_count(),
        "Thin-walled mesh element counts must strictly increase across Coarse ({}), Medium ({}), Fine ({})",
        tet_c.element_count(),
        tet_m.element_count(),
        tet_f.element_count()
    );
}

#[test]
fn test_p3_t3_convergence_study_axial_cantilever_plate_with_hole_and_timeout() {
    use mcp_cad_core::primitives::cut_hole;
    use mcp_cad_fea::boundary::{DirichletConstraint, DofComponent};
    use mcp_cad_fea::{
        run_fea_with_convergence, CONVERGENCE_DISP_TOLERANCE_PCT, CONVERGENCE_STRESS_TOLERANCE_PCT,
    };

    let material = Material::new(
        "steel_structural_aisi1020",
        205.0,
        0.30,
        250.0,
        7850.0,
        "AISI 1020 Structural Steel",
    );
    let default_levels = [MeshDensity::Coarse, MeshDensity::Medium, MeshDensity::Fine];

    // 1. Axial tension rod (0.10 x 0.02 x 0.02 m, Fx = +4000 N -> sigma_exact = 10.0 MPa):
    //    Linear TET4 represents constant uniaxial strain exactly once minimal kinematic constraints
    //    are applied -> must be marked CONVERGED (converged = true, verdict = "CONVERGED").
    let axial_box = create_box(0.10, 0.02, 0.02);
    let (rep_axial, _mesh_axial, study_axial) = run_fea_with_convergence(
        "axial_p3t3",
        &axial_box,
        &material,
        &default_levels,
        None,
        None,
        |tet| {
            let mut bc = BoundaryConditions::new();
            let aabb = tet.bounding_box();
            bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
                face: AabbFace::MinX,
                tolerance: 1e-4,
                components: DofComponent::X,
            });
            // Pin min-corner in Y,Z and one adjacent MinX corner in Z
            let n0 = tet
                .find_nodes_by_predicate(|p| {
                    (p[0] - aabb.min[0]).abs() < 1e-4
                        && (p[1] - aabb.min[1]).abs() < 1e-4
                        && (p[2] - aabb.min[2]).abs() < 1e-4
                })
                .into_iter()
                .next()
                .unwrap();
            let n1 = tet
                .find_nodes_by_predicate(|p| {
                    (p[0] - aabb.min[0]).abs() < 1e-4
                        && (p[1] - aabb.max[1]).abs() < 1e-4
                        && (p[2] - aabb.min[2]).abs() < 1e-4
                })
                .into_iter()
                .next()
                .unwrap();
            bc.dirichlet.push(DirichletConstraint::FixedNodes {
                node_ids: vec![n0],
                components: DofComponent::YZ,
            });
            bc.dirichlet.push(DirichletConstraint::FixedNodes {
                node_ids: vec![n1],
                components: DofComponent::Z,
            });
            bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [4000.0, 0.0, 0.0]);
            Ok(bc)
        },
    )
    .expect("Axial convergence study should succeed");

    let acc_axial = rep_axial
        .accuracy
        .as_ref()
        .expect("FeaReport must contain accuracy block");
    assert!(
        acc_axial.converged,
        "Axial tension rod must converge, got accuracy: {acc_axial:?}"
    );
    assert_eq!(acc_axial.verdict, "CONVERGED");
    assert!(!acc_axial.partial_result);
    assert_eq!(acc_axial.levels.len(), 3);
    assert!(acc_axial.displacement_change_pct.abs() < CONVERGENCE_DISP_TOLERANCE_PCT);
    assert!(acc_axial.stress_change_pct.abs() < CONVERGENCE_STRESS_TOLERANCE_PCT);
    assert!(study_axial.is_converged);

    // 2. Cantilever beam (200 x 20 x 20 mm, Fy = -1000 N -> sigma_exact = 150 MPa, delta_exact = 0.9756 mm):
    //    Per docs/FEA_ACCURACY.md, Coarse -> Medium -> Fine exhibits >12% change between Medium and Fine
    //    due to linear TET4 shear locking, so it must be marked NOT_CONVERGED with confidence = "low",
    //    while monotonic convergence enables Richardson extrapolation close to the Euler-Bernoulli reference.
    let cant_box = create_box(0.20, 0.02, 0.02);
    let (rep_cant, _mesh_cant, study_cant) = run_fea_with_convergence(
        "cantilever_p3t3",
        &cant_box,
        &material,
        &default_levels,
        None,
        None,
        |_tet| {
            let mut bc = BoundaryConditions::new();
            bc.fix_aabb_face(AabbFace::MinX, 1e-4);
            bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, -1000.0, 0.0]);
            Ok(bc)
        },
    )
    .expect("Cantilever convergence study should succeed");

    let acc_cant = rep_cant
        .accuracy
        .as_ref()
        .expect("Cantilever FeaReport must contain accuracy block");
    assert!(
        !acc_cant.converged,
        "Cantilever beam on Coarse->Medium->Fine TET4 must be NOT_CONVERGED"
    );
    assert_eq!(acc_cant.verdict, "NOT_CONVERGED");
    assert_eq!(
        rep_cant.confidence, "low",
        "When accuracy is NOT_CONVERGED, report.confidence must be 'low'"
    );
    assert_eq!(acc_cant.levels.len(), 3);
    assert!(
        acc_cant.levels[0].element_count < acc_cant.levels[1].element_count
            && acc_cant.levels[1].element_count < acc_cant.levels[2].element_count
    );
    assert!(
        acc_cant.displacement_change_pct.abs() >= CONVERGENCE_DISP_TOLERANCE_PCT
            || acc_cant.stress_change_pct.abs() >= CONVERGENCE_STRESS_TOLERANCE_PCT,
        "Cantilever Medium->Fine change must exceed 5%/10% tolerance: disp={:.2}%, stress={:.2}%",
        acc_cant.displacement_change_pct,
        acc_cant.stress_change_pct
    );
    assert!(
        acc_cant.richardson_applicable,
        "Cantilever beam has 3 strictly refined levels with monotonic convergence, so Richardson extrapolation must be applicable: {acc_cant:?}"
    );
    let rich_stress_mpa = acc_cant.richardson_stress_mpa.unwrap();
    let rich_disp_mm = acc_cant.richardson_displacement_mm.unwrap();
    assert!(
        rich_stress_mpa > acc_cant.levels[2].max_von_mises_mpa && rich_stress_mpa < 180.0,
        "Richardson extrapolated stress ({rich_stress_mpa:.2} MPa) should improve toward 150 MPa above Fine ({:.2} MPa)",
        acc_cant.levels[2].max_von_mises_mpa
    );
    assert!(
        rich_disp_mm > acc_cant.levels[2].max_displacement_mm && rich_disp_mm < 1.15,
        "Richardson extrapolated displacement ({rich_disp_mm:.4} mm) should improve toward 0.9756 mm above Fine ({:.4} mm)",
        acc_cant.levels[2].max_displacement_mm
    );
    assert!(!study_cant.is_converged);

    // 3a. Perforated plate with central circular hole (120 x 40 x 10 mm, hole d = 8 mm) across Coarse -> Medium -> Fine:
    //     Curved hole boundary on Cartesian voxel staircase mesh must set voxel_staircase_detected = true
    //     and return NOT_CONVERGED with confidence = "low".
    let small_plate_box = create_box(0.12, 0.04, 0.01);
    let small_perforated = cut_hole(
        &small_plate_box,
        [0.0, 0.0, -0.01],
        [0.0, 0.0, 1.0],
        0.004,
        0.03,
        None,
        None,
    );
    let (rep_small_plate, _, study_small_plate) = run_fea_with_convergence(
        "small_plate_hole_p3t3",
        &small_perforated,
        &material,
        &default_levels,
        None,
        None,
        |_tet| {
            let mut bc = BoundaryConditions::new();
            bc.fix_aabb_face(AabbFace::MinX, 1e-4);
            bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [20_000.0, 0.0, 0.0]);
            Ok(bc)
        },
    )
    .expect("Small perforated plate convergence study should succeed");
    let acc_small_plate = rep_small_plate.accuracy.as_ref().unwrap();
    assert!(acc_small_plate.voxel_staircase_detected);
    assert!(!acc_small_plate.converged);
    assert_eq!(acc_small_plate.verdict, "NOT_CONVERGED");
    assert_eq!(rep_small_plate.confidence, "low");
    assert!(!study_small_plate.is_converged);

    // 3b. Benchmark 3 perforated plate (600 x 200 x 10 mm, hole d = 40 mm) from docs/FEA_ACCURACY.md §2.3-2.4:
    //     Between Medium and Fine, in-plane voxel grid clamping at 60x60 makes raw step change artificially < 1%
    //     (< 5% disp, < 10% stress) while true error vs Peterson is ~16.6-18.8%.
    //     The improved convergence criterion must detect both grid_clamped_detected and voxel_staircase_detected
    //     and return NOT_CONVERGED with confidence = "low".
    let plate_box = create_box(0.60, 0.20, 0.01);
    let perforated_plate = cut_hole(
        &plate_box,
        [0.0, 0.0, -0.01],
        [0.0, 0.0, 1.0],
        0.02,
        0.03,
        None,
        None,
    );

    let (rep_plate, _mesh_plate, study_plate) = run_fea_with_convergence(
        "plate_hole_p3t3",
        &perforated_plate,
        &material,
        &[MeshDensity::Medium, MeshDensity::Fine],
        None,
        Some(SolverOptions {
            tolerance: 1e-4,
            max_iterations: 600,
        }),
        |tet| {
            let mut bc = BoundaryConditions::new();
            bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
                face: AabbFace::MinX,
                tolerance: 1e-4,
                components: DofComponent::X,
            });
            let min_x = tet.bounding_box().min[0];
            let center_nodes = tet.find_nodes_by_predicate(|p| {
                (p[0] - min_x).abs() <= 1e-4 && p[1].abs() <= 0.02 && p[2].abs() <= 0.006
            });
            for cn in center_nodes {
                bc.dirichlet.push(DirichletConstraint::FixedNodes {
                    node_ids: vec![cn],
                    components: DofComponent::YZ,
                });
            }
            bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [100_000.0, 0.0, 0.0]);
            Ok(bc)
        },
    )
    .expect("Perforated plate convergence study should succeed");

    let acc_plate = rep_plate
        .accuracy
        .as_ref()
        .expect("Perforated plate FeaReport must contain accuracy block");
    // Verify that raw step-to-step change between Medium and Fine is indeed deceptive (< 5% disp, < 10% stress)
    // due to 60x60 voxel clamping in X/Y, YET our improved criterion catches it!
    let raw_disp_change = acc_plate.displacement_change_pct.abs();
    let raw_stress_change = acc_plate.stress_change_pct.abs();
    assert!(
        raw_disp_change < CONVERGENCE_DISP_TOLERANCE_PCT
            && raw_stress_change < CONVERGENCE_STRESS_TOLERANCE_PCT,
        "Expected deceptive raw step change (<5%/<10%) on 600x200x10 perforated plate, got disp={raw_disp_change:.2}%, stress={raw_stress_change:.2}%"
    );
    assert!(
        acc_plate.grid_clamped_detected,
        "Must detect 60-voxel grid clamping saturation on 600x200x10 plate (r_eff = {:.3})",
        acc_plate.effective_refinement_ratio
    );
    assert!(
        acc_plate.voxel_staircase_detected,
        "Must detect curved hole staircase stress concentration on perforated plate"
    );
    assert!(
        !acc_plate.converged,
        "Perforated plate with circular hole must NOT be marked CONVERGED (true error vs Peterson is ~16.6-18.8%)"
    );
    assert_eq!(acc_plate.verdict, "NOT_CONVERGED");
    assert_eq!(rep_plate.confidence, "low");
    assert!(!study_plate.is_converged);

    // 4. Time budget (max_seconds = 0.0): must stop after the first level and return partial_result = true,
    //    verdict = "NOT_CONVERGED", confidence = "low".
    let (rep_timeout, _mesh_timeout, study_timeout) = run_fea_with_convergence(
        "cantilever_timeout_p3t3",
        &cant_box,
        &material,
        &default_levels,
        Some(0.0),
        None,
        |_tet| {
            let mut bc = BoundaryConditions::new();
            bc.fix_aabb_face(AabbFace::MinX, 1e-4);
            bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, -1000.0, 0.0]);
            Ok(bc)
        },
    )
    .expect("Timeout convergence study should return partial result");

    let acc_timeout = rep_timeout.accuracy.as_ref().unwrap();
    assert!(acc_timeout.partial_result);
    assert!(!acc_timeout.converged);
    assert_eq!(acc_timeout.verdict, "NOT_CONVERGED");
    assert_eq!(rep_timeout.confidence, "low");
    assert_eq!(acc_timeout.levels.len(), 1);
    assert!(study_timeout.partial_result);
    assert!(acc_timeout.recommendation.contains("partial_result=true"));
}

#[test]
fn test_p3_t4_tet10_mesh_conversion_eigenvalues_and_multi_element_patch_test() {
    use std::collections::HashSet;
    use mcp_cad_fea::boundary::{DirichletConstraint, DofComponent, ResolvedBoundary};
    use mcp_cad_fea::elasticity::elasticity_matrix_from_material;
    use mcp_cad_fea::element::{Tet10Element, Tet4Element, Vector30, TET10_EDGE_PAIRS};
    use mcp_cad_fea::result::compute_fea_results_with_boundary_tet10;
    use mcp_cad_fea::solver::{assemble_and_solve_tet10, SolverOptions};

    let material = Material::new(
        "steel_p3t4",
        205.0,
        0.30,
        250.0,
        7850.0,
        "Structural steel",
    );
    let d_matrix = elasticity_matrix_from_material(&material).unwrap();

    // 1. Convert a 3x2x2 structured box mesh (72 tetrahedra) from TET4 to TET10
    let tet4_mesh = TetrahedralMesh::from_box([0.0, 0.0, 0.0], [0.12, 0.06, 0.04], 3, 2, 2).unwrap();
    assert_eq!(tet4_mesh.element_order(), 1);
    assert!(!tet4_mesh.is_tet10());

    let tet10_mesh = tet4_mesh.to_tet10();
    assert_eq!(tet10_mesh.element_order(), 2);
    assert!(tet10_mesh.is_tet10());
    assert_eq!(tet10_mesh.element_count(), tet4_mesh.element_count());
    // On a 3x2x2 grid: corner nodes = 4*3*3 = 36; mid-edge nodes must be uniquely shared across elements
    assert!(tet10_mesh.node_count() > tet4_mesh.node_count());

    // Verify no duplicate node coordinates and exact edge midpoint placement
    for ten in &tet10_mesh.tet10_elements {
        let unique_nodes: HashSet<usize> = ten.iter().copied().collect();
        assert_eq!(unique_nodes.len(), 10, "Each TET10 element must have 10 distinct nodes");
        for (e, &(la, lb)) in TET10_EDGE_PAIRS.iter().enumerate() {
            let pa = tet10_mesh.nodes[ten[la]].position;
            let pb = tet10_mesh.nodes[ten[lb]].position;
            let pm = tet10_mesh.nodes[ten[4 + e]].position;
            for axis in 0..3 {
                assert!(
                    (pm[axis] - 0.5 * (pa[axis] + pb[axis])).abs() < 1e-14,
                    "Mid-edge node {} must lie at exact edge midpoint",
                    4 + e
                );
            }
        }
    }

    // 2. Build all Tet10Elements and verify 30x30 stiffness matrix symmetry + 6 zero eigenvalues
    let mut elements = Vec::with_capacity(tet10_mesh.tet10_elements.len());
    for (id, &node_indices) in tet10_mesh.tet10_elements.iter().enumerate() {
        let mut positions = [[0.0f64; 3]; 10];
        for (k, &nid) in node_indices.iter().enumerate() {
            positions[k] = tet10_mesh.nodes[nid].position;
        }
        let elem = Tet10Element::new(id, node_indices, positions).unwrap();
        elements.push(elem);
    }

    // Check eigenvalues on the first element (and symmetry on all elements)
    let ke0 = elements[0].stiffness_matrix(&d_matrix);
    let mut eigs: Vec<f64> = ke0.symmetric_eigenvalues().iter().copied().collect();
    eigs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let max_eig = eigs[29];
    let zero_eigs = eigs.iter().filter(|&&lam| lam.abs() < max_eig * 1e-9).count();
    let pos_eigs = eigs.iter().filter(|&&lam| lam > max_eig * 1e-6).count();
    assert_eq!(zero_eigs, 6, "30x30 TET10 Ke must have 6 rigid-body zero eigenvalues");
    assert_eq!(pos_eigs, 24, "30x30 TET10 Ke must have 24 positive eigenvalues");

    // 3a. Multi-element 3D kinematic Patch Test:
    //     Prescribe an arbitrary 3D linear displacement field u(x,y,z) on all exterior boundary nodes,
    //     solve for interior free nodes via assemble_and_solve_tet10, and verify exact constant strain & stress everywhere.
    let surface_tris = tet10_mesh.extract_surface_triangles();
    let mut boundary_nodes = HashSet::new();
    for &[a, b, c] in &surface_tris {
        boundary_nodes.insert(a);
        boundary_nodes.insert(b);
        boundary_nodes.insert(c);
    }
    // Ensure there are interior (unconstrained) nodes in the 3x2x2 TET10 mesh
    let interior_count = tet10_mesh.node_count() - boundary_nodes.len();
    assert!(
        interior_count > 0,
        "3x2x2 TET10 mesh must have interior nodes for non-trivial multi-element patch test"
    );

    let eval_linear_u = |p: [f64; 3]| -> [f64; 3] {
        let [x, y, z] = p;
        [
            1.5e-4 + 0.0010 * x - 0.0003 * y + 0.0005 * z,
            -0.8e-4 + 0.0004 * x + 0.0008 * y - 0.0002 * z,
            2.1e-4 - 0.0006 * x + 0.0007 * y - 0.0009 * z,
        ]
    };
    let expected_strain = nalgebra::SVector::<f64, 6>::from([
        0.0010,
        0.0008,
        -0.0009,
        -0.0003 + 0.0004,
        -0.0002 + 0.0007,
        0.0005 - 0.0006,
    ]);
    let expected_stress = d_matrix * expected_strain;
    let expected_vm = Tet4Element::von_mises_stress(&expected_stress);

    let mut fixed_dofs = std::collections::HashMap::new();
    for &nid in &boundary_nodes {
        let u = eval_linear_u(tet10_mesh.nodes[nid].position);
        fixed_dofs.insert(3 * nid, u[0]);
        fixed_dofs.insert(3 * nid + 1, u[1]);
        fixed_dofs.insert(3 * nid + 2, u[2]);
    }
    let resolved_patch = ResolvedBoundary {
        fixed_dofs,
        nodal_forces: vec![[0.0; 3]; tet10_mesh.node_count()],
    };

    let u_full = assemble_and_solve_tet10(
        &tet10_mesh,
        &elements,
        &d_matrix,
        &resolved_patch,
        Some(SolverOptions {
            tolerance: 1e-12,
            max_iterations: 5000,
        }),
    )
    .expect("Multi-element TET10 patch test solve must succeed");

    // Verify all interior & boundary nodal displacements match the exact linear field
    for node in &tet10_mesh.nodes {
        let expected_u = eval_linear_u(node.position);
        for axis in 0..3 {
            let actual = u_full[3 * node.id + axis];
            let err = (actual - expected_u[axis]).abs();
            assert!(
                err < 1e-11,
                "Multi-element patch test displacement error {err:.3e} at node {} axis {axis}",
                node.id
            );
        }
    }

    // Verify all Gauss points across all 72 elements and all smoothed nodal von Mises stresses match expected_vm
    for elem in &elements {
        let mut u_elem = Vector30::zeros();
        for (local_i, &gn) in elem.node_indices.iter().enumerate() {
            u_elem[3 * local_i] = u_full[3 * gn];
            u_elem[3 * local_i + 1] = u_full[3 * gn + 1];
            u_elem[3 * local_i + 2] = u_full[3 * gn + 2];
        }
        for s in elem.compute_gauss_stresses(&d_matrix, &u_elem) {
            let rel_err = (s - expected_stress).norm() / expected_stress.norm();
            assert!(
                rel_err < 1e-8,
                "Multi-element patch test Gauss stress error {rel_err:.3e}"
            );
        }
    }

    let patch_report = compute_fea_results_with_boundary_tet10(
        "patch_test_tet10",
        &tet10_mesh,
        &elements,
        &d_matrix,
        Some(&resolved_patch),
        &u_full,
        material.yield_strength_pa(),
        1.0,
    );
    assert_eq!(patch_report.element_order, 2);
    for (nid, &vm) in patch_report.nodal_von_mises_stresses.iter().enumerate() {
        let rel_err = (vm - expected_vm).abs() / expected_vm;
        assert!(
            rel_err < 1e-8,
            "Multi-element patch test nodal von Mises error {rel_err:.3e} at node {nid}"
        );
    }

    // 3b. Multi-element 3D Neumann Traction Patch Test (uniaxial tension bar with roller BCs):
    //     Consistent quadratic surface traction on MaxX must produce exact constant stress sigma_xx = F / A everywhere.
    let area = 0.06 * 0.04; // 0.0024 m²
    let force_x = 48_000.0; // sigma_xx = 20.0 MPa
    let expected_sigma_pa = force_x / area;
    let expected_ux_m = expected_sigma_pa * 0.12 / 205.0e9;

    let mut bc_roller = BoundaryConditions::new();
    bc_roller.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinX,
        tolerance: 1e-6,
        components: DofComponent::X,
    });
    bc_roller.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinY,
        tolerance: 1e-6,
        components: DofComponent::Y,
    });
    bc_roller.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinZ,
        tolerance: 1e-6,
        components: DofComponent::Z,
    });
    bc_roller.add_aabb_face_force(AabbFace::MaxX, 1e-6, [force_x, 0.0, 0.0]);

    let roller_report = analyze_mesh(
        "uniaxial_patch_tet10",
        &tet10_mesh,
        &material,
        &bc_roller,
        Some(SolverOptions {
            tolerance: 1e-11,
            max_iterations: 5000,
        }),
    )
    .expect("Uniaxial traction patch test on TET10 must succeed");

    for (nid, &vm) in roller_report.nodal_von_mises_stresses.iter().enumerate() {
        let rel_err = (vm - expected_sigma_pa).abs() / expected_sigma_pa;
        assert!(
            rel_err < 1e-6,
            "Uniaxial traction patch test stress error {rel_err:.3e} at node {nid} (got {:.4} MPa vs 20.0 MPa)",
            vm / 1e6
        );
    }
    let max_x_nodes = tet10_mesh.find_nodes_by_predicate(|p| (p[0] - 0.12).abs() < 1e-6);
    for nid in max_x_nodes {
        let ux = roller_report.nodal_displacements[nid][0];
        let rel_err = (ux - expected_ux_m).abs() / expected_ux_m;
        assert!(
            rel_err < 1e-6,
            "Uniaxial traction patch test ux error {rel_err:.3e} at MaxX node {nid}"
        );
    }
}




