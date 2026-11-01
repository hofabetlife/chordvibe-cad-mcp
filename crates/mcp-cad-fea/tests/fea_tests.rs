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
}
