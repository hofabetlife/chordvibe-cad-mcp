//! Benchmark suite for validating FEA simulation accuracy against classical analytical solutions.
//! Covers 4 reference benchmarks from solid mechanics (P3-T1):
//! 1. Axial tension of a slender rod (Saint-Venant uniaxial tension)
//! 2. Cantilever beam bending (Euler-Bernoulli & Timoshenko beam theory)
//! 3. Perforated plate in tension with central circular hole (Peterson / Kirsch stress concentration)
//! 4. Thick-walled cylinder under internal pressure (Lamé equation)

use std::time::Instant;
use mcp_cad_core::material::Material;
use mcp_cad_core::primitives::{create_box, create_cylinder, cut_hole};
use mcp_cad_fea::boundary::{
    extract_face_triangles, AabbFace, BoundaryConditions, DirichletConstraint, DofComponent,
};
use mcp_cad_fea::solver::SolverOptions;
use mcp_cad_fea::tetra_mesh::{MeshDensity, TetrahedralMesh};
use mcp_cad_fea::analyze_mesh;

/// Structural steel (AISI 1020 / ASTM A36 reference material: E = 205 GPa, nu = 0.30)
fn benchmark_steel() -> Material {
    Material::new(
        "structural_steel_benchmark",
        205.0,  // E = 205 GPa
        0.30,   // nu = 0.30
        250.0,  // Yield = 250 MPa
        7850.0, // Density = 7850 kg/m³
        "Benchmark Structural Steel (E=205 GPa, nu=0.3)",
    )
}

#[derive(Debug, Clone)]
pub struct BenchmarkRow {
    pub name: String,
    pub density: &'static str,
    pub element_count: usize,
    pub node_count: usize,
    pub calculated_disp_mm: f64,
    pub analytical_disp_mm: f64,
    pub disp_rel_error_pct: f64,
    pub calculated_stress_mpa: f64,
    pub analytical_stress_mpa: f64,
    pub stress_rel_error_pct: f64,
    pub elapsed_ms: f64,
    pub additional_note: String,
}

impl BenchmarkRow {
    pub fn print_summary(&self) {
        println!(
            "[{}] {:<6} | Elems: {:>6} | Nodes: {:>5} | Disp: {:.6} mm vs {:.6} mm (err: {:>6.2}%) | Stress: {:>6.2} MPa vs {:>6.2} MPa (err: {:>6.2}%) | {:>6.1} ms | {}",
            self.name,
            self.density,
            self.element_count,
            self.node_count,
            self.calculated_disp_mm,
            self.analytical_disp_mm,
            self.disp_rel_error_pct,
            self.calculated_stress_mpa,
            self.analytical_stress_mpa,
            self.stress_rel_error_pct,
            self.elapsed_ms,
            self.additional_note,
        );
    }
}

// =========================================================================
// 1. AXIAL TENSION OF A SLENDER ROD
// Dimensions: 1000 x 10 x 10 mm (L = 1.0 m, cross section 0.01 x 0.01 m)
// Load: F = 2050 N in tension (+X)
// Material: E = 205 GPa, nu = 0.30
// Analytical:
//   A = 0.01 * 0.01 = 1e-4 m²
//   sigma = F / A = 2050 / 1e-4 = 20.50 MPa
//   delta = F * L / (E * A) = 2050 * 1.0 / (205e9 * 1e-4) = 0.1000 mm = 1e-4 m
// =========================================================================
fn run_axial_tension_case(density: MeshDensity, density_name: &'static str) -> BenchmarkRow {
    let t0 = Instant::now();
    let material = benchmark_steel();
    let length = 1.0f64;
    let width = 0.01f64;
    let height = 0.01f64;
    let area = width * height; // 1e-4 m²
    let force_x = 2050.0f64; // N

    let expected_stress_pa = force_x / area; // 20.5 MPa
    let expected_disp_m = (force_x * length) / (205.0e9 * area); // 0.1 mm = 1e-4 m

    let box_mesh = create_box(length, height, width);
    let tet_mesh = TetrahedralMesh::from_mesh(&box_mesh, density).expect("Meshing failed");

    let mut bc = BoundaryConditions::new();
    // Fix Ux = 0 at MinX (x = -0.5)
    bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinX,
        tolerance: 1e-4,
        components: DofComponent::X,
    });
    // Fix Uy and Uz on center line/nodes at MinX to allow lateral Poisson contraction
    let min_x = tet_mesh.bounding_box().min[0];
    let center_nodes = tet_mesh.find_nodes_by_predicate(|p| {
        (p[0] - min_x).abs() <= 1e-4 && p[1].abs() <= 0.003 && p[2].abs() <= 0.003
    });
    for cn in center_nodes {
        bc.dirichlet.push(DirichletConstraint::FixedNodes {
            node_ids: vec![cn],
            components: DofComponent::YZ,
        });
    }
    // Tensile force at MaxX (x = +0.5)
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [force_x, 0.0, 0.0]);

    let solver_opts = SolverOptions {
        tolerance: 1e-7,
        max_iterations: 5000,
    };

    let report = analyze_mesh("axial_rod", &tet_mesh, &material, &bc, Some(solver_opts))
        .expect("FEA solve failed");

    let calc_disp_mm = report.max_displacement_m * 1000.0;
    let an_disp_mm = expected_disp_m * 1000.0;
    let disp_err = (report.max_displacement_m - expected_disp_m).abs() / expected_disp_m * 100.0;

    let calc_stress_mpa = report.max_stress_von_mises_pa / 1e6;
    let an_stress_mpa = expected_stress_pa / 1e6;
    let stress_err =
        (report.max_stress_von_mises_pa - expected_stress_pa).abs() / expected_stress_pa * 100.0;

    let mean_stress_mpa = report.nodal_von_mises_stresses.iter().sum::<f64>()
        / report.nodal_von_mises_stresses.len() as f64
        / 1e6;

    BenchmarkRow {
        name: "1. Axial Tension (1000x10x10mm)".to_string(),
        density: density_name,
        element_count: tet_mesh.elements.len(),
        node_count: tet_mesh.nodes.len(),
        calculated_disp_mm: calc_disp_mm,
        analytical_disp_mm: an_disp_mm,
        disp_rel_error_pct: disp_err,
        calculated_stress_mpa: calc_stress_mpa,
        analytical_stress_mpa: an_stress_mpa,
        stress_rel_error_pct: stress_err,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
        additional_note: format!("mean_stress={:.2} MPa", mean_stress_mpa),
    }
}

// =========================================================================
// 2. CANTILEVER BEAM
// Dimensions: 200 x 20 x 20 mm (L = 0.2 m, b = 0.02 m, h = 0.02 m)
// Load: F = 1000 N downwards (-Y) at free end MaxX
// Clamp: MinX fully fixed
// Material: E = 205 GPa, nu = 0.30
// Analytical:
//   I = b * h³ / 12 = 0.02 * (0.02)³ / 12 = 1.333333e-8 m⁴
//   delta_EB = F * L³ / (3 * E * I) = 0.975610 mm
//   G = E / (2*(1+nu)) = 78.846 GPa, kappa = 5/6
//   delta_shear = F * L / (kappa * G * A) = 1000 * 0.2 / ((5/6) * 78.846e9 * 4e-4) = 0.007610 mm (+0.78%)
//   delta_Timoshenko = 0.983220 mm
//   sigma_root = 6 * F * L / (b * h²) = 150.00 MPa
//   sigma(x = 0.5h = 10mm from root) = 6 * F * (L - 0.5h) / (b * h²) = 142.50 MPa
// =========================================================================
fn run_cantilever_case(
    density: MeshDensity,
    density_name: &'static str,
) -> (BenchmarkRow, f64, f64) {
    let t0 = Instant::now();
    let material = benchmark_steel();
    let length = 0.2f64;
    let width = 0.02f64;
    let height = 0.02f64;
    let force_y = -1000.0f64; // N

    let i_moment = width * height.powi(3) / 12.0;
    let expected_disp_eb = force_y.abs() * length.powi(3) / (3.0 * 205.0e9 * i_moment); // 0.975610 mm
    let g_shear = 205.0e9 / (2.0 * (1.0 + 0.30));
    let expected_disp_timoshenko =
        expected_disp_eb + (force_y.abs() * length) / ((5.0 / 6.0) * g_shear * (width * height)); // 0.983220 mm
    let expected_stress_root = 6.0 * force_y.abs() * length / (width * height.powi(2)); // 150.0 MPa
    let expected_stress_half_h =
        6.0 * force_y.abs() * (length - 0.5 * height) / (width * height.powi(2)); // 142.5 MPa

    let box_mesh = create_box(length, height, width);
    let tet_mesh = TetrahedralMesh::from_mesh(&box_mesh, density).expect("Meshing failed");

    let mut bc = BoundaryConditions::new();
    bc.fix_aabb_face(AabbFace::MinX, 1e-4);
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, force_y, 0.0]);

    let solver_opts = SolverOptions {
        tolerance: 1e-7,
        max_iterations: 5000,
    };

    let report = analyze_mesh("cantilever", &tet_mesh, &material, &bc, Some(solver_opts))
        .expect("FEA solve failed");

    let calc_disp_mm = report.max_displacement_m * 1000.0;
    let an_disp_mm = expected_disp_eb * 1000.0;
    let disp_err = (report.max_displacement_m - expected_disp_eb).abs() / expected_disp_eb * 100.0;
    let disp_err_timo = (report.max_displacement_m - expected_disp_timoshenko).abs()
        / expected_disp_timoshenko
        * 100.0;

    let calc_stress_mpa = report.max_stress_von_mises_pa / 1e6;
    let an_stress_mpa = expected_stress_root / 1e6;
    let stress_err =
        (report.max_stress_von_mises_pa - expected_stress_root).abs() / expected_stress_root * 100.0;

    // Stress at distance 0.5h (10 mm) from clamped end (x = min_x + 0.5*h = -0.1 + 0.01 = -0.09 m)
    let min_x = tet_mesh.bounding_box().min[0];
    let target_x = min_x + 0.5 * height;
    let nodes_at_half_h = tet_mesh.find_nodes_by_predicate(|p| (p[0] - target_x).abs() <= 0.0025);
    let max_stress_at_half_h = nodes_at_half_h
        .iter()
        .map(|&nid| report.nodal_von_mises_stresses[nid])
        .fold(0.0_f64, f64::max)
        / 1e6;
    let half_h_err =
        (max_stress_at_half_h * 1e6 - expected_stress_half_h).abs() / expected_stress_half_h * 100.0;

    let row = BenchmarkRow {
        name: "2. Cantilever (200x20x20mm)".to_string(),
        density: density_name,
        element_count: tet_mesh.elements.len(),
        node_count: tet_mesh.nodes.len(),
        calculated_disp_mm: calc_disp_mm,
        analytical_disp_mm: an_disp_mm,
        disp_rel_error_pct: disp_err,
        calculated_stress_mpa: calc_stress_mpa,
        analytical_stress_mpa: an_stress_mpa,
        stress_rel_error_pct: stress_err,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
        additional_note: format!(
            "Timo_err={:.2}%, stress@0.5h={:.2} MPa vs {:.2} MPa (err={:.2}%)",
            disp_err_timo,
            max_stress_at_half_h,
            expected_stress_half_h / 1e6,
            half_h_err
        ),
    };
    (row, max_stress_at_half_h, half_h_err)
}

// =========================================================================
// 3. PERFORATED PLATE IN TENSION (PLATE WITH CENTRAL CIRCULAR HOLE)
// Dimensions: 600 x 200 x 10 mm (L = 0.6 m, w = 0.2 m, t = 0.01 m)
// Hole: central circular hole diameter d = 40 mm (r = 0.02 m)
// Load: F = 100 kN (100,000 N) along X in tension
// Material: E = 205 GPa, nu = 0.30
// Analytical:
//   Net area A_net = (w - d) * t = (0.2 - 0.04) * 0.01 = 0.0016 m²
//   Nominal net stress sigma_nom = F / A_net = 100,000 / 0.0016 = 62.50 MPa
//   Peterson polynomial (Roark's / Pilkey "Peterson's Stress Concentration Factors", Chart 4.1):
//     d/w = 0.04 / 0.2 = 0.20
//     Kt = 3.00 - 3.14*(0.2) + 3.667*(0.2)² - 1.527*(0.2)³ = 2.506464
//     sigma_max_Peterson = 2.506464 * 62.5 = 156.654 MPa
//   bench/README.md reference:
//     sigma_max = 157.44 MPa -> Kt = 157.44 / 62.5 = 2.51904 (0.50% difference vs Peterson)
// =========================================================================
fn run_plate_with_hole_case(density: MeshDensity, density_name: &'static str) -> BenchmarkRow {
    let t0 = Instant::now();
    let material = benchmark_steel();
    let length = 0.6f64;
    let width = 0.2f64;
    let thickness = 0.01f64;
    let hole_diam = 0.04f64;
    let hole_radius = hole_diam / 2.0;
    let force_x = 100_000.0f64; // 100 kN

    let net_area = (width - hole_diam) * thickness; // 0.0016 m²
    let sigma_nom = force_x / net_area; // 62.5 MPa
    let d_over_w: f64 = hole_diam / width; // 0.20
    #[allow(clippy::approx_constant)]
    let kt_peterson =
        3.00 - 3.14 * d_over_w + 3.667 * d_over_w.powi(2) - 1.527 * d_over_w.powi(3);
    let expected_stress_peterson = kt_peterson * sigma_nom; // 156.654 MPa
    let expected_stress_bench_mpa = 157.44f64; // bench/README.md

    let plate = create_box(length, width, thickness);
    let perforated = cut_hole(
        &plate,
        [0.0, 0.0, -0.01],
        [0.0, 0.0, 1.0],
        hole_radius,
        0.03,
        None,
        None,
    );

    let tet_mesh = TetrahedralMesh::from_mesh(&perforated, density).expect("Meshing failed");

    let mut bc = BoundaryConditions::new();
    // Fix Ux = 0 at MinX
    bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinX,
        tolerance: 1e-4,
        components: DofComponent::X,
    });
    // Fix center nodes at MinX in Y and Z to prevent rigid body motion while allowing Poisson contraction
    let min_x = tet_mesh.bounding_box().min[0];
    let center_nodes = tet_mesh.find_nodes_by_predicate(|p| {
        (p[0] - min_x).abs() <= 1e-4 && p[1].abs() <= 0.02 && p[2].abs() <= 0.006
    });
    for cn in center_nodes {
        bc.dirichlet.push(DirichletConstraint::FixedNodes {
            node_ids: vec![cn],
            components: DofComponent::YZ,
        });
    }
    // Tensile load at MaxX
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [force_x, 0.0, 0.0]);

    let solver_opts = SolverOptions {
        tolerance: 1e-6,
        max_iterations: 4000,
    };

    let report = analyze_mesh("perforated_plate", &tet_mesh, &material, &bc, Some(solver_opts))
        .expect("FEA solve failed");

    // Nominal elongation of unperforated strip: delta_0 = F * L / (E * w * t) = 0.146341 mm
    // With a small central hole (d/w = 0.2), compliance correction adds ~ (pi/2)*(d^2/(w*L)) * delta_0 ~ +1.05% -> ~0.14788 mm
    let gross_area = width * thickness;
    let delta_gross_m = (force_x * length) / (205.0e9 * gross_area);
    let hole_compliance_factor =
        1.0 + (std::f64::consts::PI / 2.0) * (hole_diam * hole_diam) / (width * length);
    let expected_disp_m = delta_gross_m * hole_compliance_factor; // ~0.14788 mm

    let calc_disp_mm = report.max_displacement_m * 1000.0;
    let an_disp_mm = expected_disp_m * 1000.0;
    let disp_err = (report.max_displacement_m - expected_disp_m).abs() / expected_disp_m * 100.0;

    let calc_stress_mpa = report.max_stress_von_mises_pa / 1e6;
    let an_stress_mpa = expected_stress_peterson / 1e6;
    let stress_err_peterson = (report.max_stress_von_mises_pa - expected_stress_peterson).abs()
        / expected_stress_peterson
        * 100.0;
    let stress_err_bench =
        (calc_stress_mpa - expected_stress_bench_mpa).abs() / expected_stress_bench_mpa * 100.0;
    let calc_kt = calc_stress_mpa / (sigma_nom / 1e6);

    BenchmarkRow {
        name: "3. Plate w/ Hole (600x200x10, d=40)".to_string(),
        density: density_name,
        element_count: tet_mesh.elements.len(),
        node_count: tet_mesh.nodes.len(),
        calculated_disp_mm: calc_disp_mm,
        analytical_disp_mm: an_disp_mm,
        disp_rel_error_pct: disp_err,
        calculated_stress_mpa: calc_stress_mpa,
        analytical_stress_mpa: an_stress_mpa,
        stress_rel_error_pct: stress_err_peterson,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
        additional_note: format!(
            "Kt_calc={:.3} vs Kt_Peterson={:.3} (156.65 MPa) / Kt_bench=2.519 (157.44 MPa, err={:.2}%)",
            calc_kt, kt_peterson, stress_err_bench
        ),
    }
}

// =========================================================================
// 4. THICK-WALLED CYLINDER UNDER INTERNAL PRESSURE (LAMÉ PROBLEM)
// Dimensions: inner radius a = 20 mm (0.02 m), outer radius b = 40 mm (0.04 m), height L = 40 mm (0.04 m)
// Internal pressure: p = 10.0 MPa (1.0e7 Pa) applied via SurfaceTrianglesPressure on `hole_wall`
// Material: E = 205 GPa, nu = 0.30
// Analytical (Lamé equations for open-ended cylinder, sigma_z = 0):
//   sigma_theta(r=a) = p * (b² + a²) / (b² - a²) = 10 * (16 + 4) / (16 - 4) = 16.6667 MPa
//   sigma_r(r=a)     = -p = -10.0000 MPa
//   sigma_vM(r=a)    = sqrt(sigma_theta² - sigma_theta * sigma_r + sigma_r²) = 23.3333 MPa
//   u_r(r=a)         = (p * a / E) * ((b² + a²) / (b² - a²) + nu)
//                    = (10e6 * 0.02 / 205e9) * (1.66667 + 0.30) = 0.001919 mm
// =========================================================================
fn run_lame_cylinder_case(density: MeshDensity, density_name: &'static str) -> BenchmarkRow {
    let t0 = Instant::now();
    let material = benchmark_steel();
    let a = 0.02f64; // 20 mm inner radius
    let b = 0.04f64; // 40 mm outer radius
    let length = 0.04f64; // 40 mm axial length
    let pressure_pa = 10.0e6f64; // 10 MPa

    let expected_sigma_theta: f64 = pressure_pa * (b * b + a * a) / (b * b - a * a); // 16.6667 MPa
    let expected_sigma_r: f64 = -pressure_pa; // -10.0 MPa
    let expected_vm_stress = (expected_sigma_theta.powi(2)
        - expected_sigma_theta * expected_sigma_r
        + expected_sigma_r.powi(2))
    .sqrt(); // 23.3333 MPa
    let expected_disp_m =
        (pressure_pa * a / 205.0e9) * ((b * b + a * a) / (b * b - a * a) + 0.30); // 1.9187e-6 m

    let outer_cyl = create_cylinder(b, length, 48);
    let hollow_cyl = cut_hole(
        &outer_cyl,
        [0.0, 0.0, -length * 0.6],
        [0.0, 0.0, 1.0],
        a,
        length * 1.2,
        None,
        None,
    );

    let hole_wall = hollow_cyl
        .faces
        .iter()
        .find(|f| f.role == "hole_wall")
        .expect("Hollow cylinder must contain hole_wall face");
    let hole_tris = extract_face_triangles(&hollow_cyl, &[hole_wall.id]);

    let tet_mesh = TetrahedralMesh::from_mesh(&hollow_cyl, density).expect("Meshing failed");

    let mut bc = BoundaryConditions::new();
    // 1. Constrain axial motion Uz = 0 on bottom face (z = min_z) -> eliminates Uz, Rx, Ry rigid modes
    bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinZ,
        tolerance: 1e-4,
        components: DofComponent::Z,
    });

    // 2. Constrain Uy = 0 for the single closest grid layer to y = 0 -> eliminates Uy and Rz rigid modes while allowing radial Ux
    let min_abs_y = tet_mesh
        .nodes
        .iter()
        .map(|n| n.position[1].abs())
        .fold(f64::INFINITY, f64::min);
    let y_sym_nodes = tet_mesh.find_nodes_by_predicate(|p| (p[1].abs() - min_abs_y).abs() <= 1e-5);
    assert!(
        !y_sym_nodes.is_empty(),
        "Must find nodes near y=0 symmetry plane"
    );
    bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: y_sym_nodes,
        components: DofComponent::Y,
    });

    // 3. Constrain Ux = 0 for the single closest grid layer to x = 0 -> eliminates Ux rigid mode while allowing radial Uy
    let min_abs_x = tet_mesh
        .nodes
        .iter()
        .map(|n| n.position[0].abs())
        .fold(f64::INFINITY, f64::min);
    let x_sym_nodes = tet_mesh.find_nodes_by_predicate(|p| (p[0].abs() - min_abs_x).abs() <= 1e-5);
    assert!(
        !x_sym_nodes.is_empty(),
        "Must find nodes near x=0 symmetry plane"
    );
    bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: x_sym_nodes,
        components: DofComponent::X,
    });

    // Apply internal pressure on hole_wall
    bc.add_surface_triangles_pressure(Some(hole_wall.name.clone()), hole_tris, 0.0, pressure_pa);

    let solver_opts = SolverOptions {
        tolerance: 1e-6,
        max_iterations: 5000,
    };

    let report = analyze_mesh("lame_cylinder", &tet_mesh, &material, &bc, Some(solver_opts))
        .expect("FEA solve failed");

    let calc_disp_mm = report.max_displacement_m * 1000.0;
    let an_disp_mm = expected_disp_m * 1000.0;
    let disp_err = (report.max_displacement_m - expected_disp_m).abs() / expected_disp_m * 100.0;

    let calc_stress_mpa = report.max_stress_von_mises_pa / 1e6;
    let an_stress_mpa = expected_vm_stress / 1e6;
    let stress_err =
        (report.max_stress_von_mises_pa - expected_vm_stress).abs() / expected_vm_stress * 100.0;

    // Also compute mean stress on inner bore nodes
    let inner_nodes = tet_mesh.find_nodes_by_predicate(|p| {
        let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
        r <= a + 0.005
    });
    let mean_inner_stress_mpa = if !inner_nodes.is_empty() {
        inner_nodes
            .iter()
            .map(|&nid| report.nodal_von_mises_stresses[nid])
            .sum::<f64>()
            / (inner_nodes.len() as f64)
            / 1e6
    } else {
        0.0
    };

    BenchmarkRow {
        name: "4. Lamé Cylinder (a=20, b=40mm)".to_string(),
        density: density_name,
        element_count: tet_mesh.elements.len(),
        node_count: tet_mesh.nodes.len(),
        calculated_disp_mm: calc_disp_mm,
        analytical_disp_mm: an_disp_mm,
        disp_rel_error_pct: disp_err,
        calculated_stress_mpa: calc_stress_mpa,
        analytical_stress_mpa: an_stress_mpa,
        stress_rel_error_pct: stress_err,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
        additional_note: format!(
            "sigma_theta={:.2} MPa, mean_bore_vM={:.2} MPa",
            expected_sigma_theta / 1e6,
            mean_inner_stress_mpa
        ),
    }
}

#[test]
fn test_fea_accuracy_benchmarks_suite() {
    println!("\n====================================================================================================");
    println!("MCP-CAD FEA ACCURACY BENCHMARK SUITE (P3-T1)");
    println!("====================================================================================================");

    // 1. Axial tension
    let ax_coarse = run_axial_tension_case(MeshDensity::Coarse, "Coarse");
    ax_coarse.print_summary();
    let ax_medium = run_axial_tension_case(MeshDensity::Medium, "Medium");
    ax_medium.print_summary();
    let ax_fine = run_axial_tension_case(MeshDensity::Fine, "Fine");
    ax_fine.print_summary();

    // 2. Cantilever beam
    let (cb_coarse, _, cb_half_err_c) = run_cantilever_case(MeshDensity::Coarse, "Coarse");
    cb_coarse.print_summary();
    let (cb_medium, _, cb_half_err_m) = run_cantilever_case(MeshDensity::Medium, "Medium");
    cb_medium.print_summary();
    let (cb_fine, _, cb_half_err_f) = run_cantilever_case(MeshDensity::Fine, "Fine");
    cb_fine.print_summary();

    // 3. Perforated plate with central circular hole
    let pl_coarse = run_plate_with_hole_case(MeshDensity::Coarse, "Coarse");
    pl_coarse.print_summary();
    let pl_medium = run_plate_with_hole_case(MeshDensity::Medium, "Medium");
    pl_medium.print_summary();
    let pl_fine = run_plate_with_hole_case(MeshDensity::Fine, "Fine");
    pl_fine.print_summary();

    // 4. Thick-walled cylinder under internal pressure (Lamé)
    let lm_coarse = run_lame_cylinder_case(MeshDensity::Coarse, "Coarse");
    lm_coarse.print_summary();
    let lm_medium = run_lame_cylinder_case(MeshDensity::Medium, "Medium");
    lm_medium.print_summary();
    let lm_fine = run_lame_cylinder_case(MeshDensity::Fine, "Fine");
    lm_fine.print_summary();

    println!("====================================================================================================\n");

    // -------------------------------------------------------------------------
    // RATCHET ASSERTIONS: error <= measured * 1.25 + 1.0%
    // Ensures future improvements can tighten thresholds while catching regressions.
    // -------------------------------------------------------------------------
    let check_ratchet = |label: &str, actual_err_pct: f64, measured_baseline_pct: f64| {
        let limit = measured_baseline_pct * 1.25 + 1.0;
        assert!(
            actual_err_pct <= limit,
            "[Ratchet FAIL] {}: error {:.2}% exceeded ratchet limit {:.2}% (baseline {:.2}% * 1.25 + 1.0%)",
            label,
            actual_err_pct,
            limit,
            measured_baseline_pct
        );
    };

    // Benchmark 1: Axial Tension (measured disp err: 0.07%, 0.11%, 0.01%; stress err: 3.94%, 1.44%, 3.87%)
    check_ratchet("Axial Tension Coarse Disp", ax_coarse.disp_rel_error_pct, 0.07);
    check_ratchet("Axial Tension Medium Disp", ax_medium.disp_rel_error_pct, 0.11);
    check_ratchet("Axial Tension Fine Disp", ax_fine.disp_rel_error_pct, 0.01);
    check_ratchet("Axial Tension Coarse Stress", ax_coarse.stress_rel_error_pct, 3.94);
    check_ratchet("Axial Tension Medium Stress", ax_medium.stress_rel_error_pct, 1.44);
    check_ratchet("Axial Tension Fine Stress", ax_fine.stress_rel_error_pct, 3.87);

    // Benchmark 2: Cantilever Beam (measured disp err: 48.15%, 20.14%, 10.24%; stress err: 46.18%, 21.91%, 11.63%)
    check_ratchet("Cantilever Coarse Disp", cb_coarse.disp_rel_error_pct, 48.15);
    check_ratchet("Cantilever Medium Disp", cb_medium.disp_rel_error_pct, 20.14);
    check_ratchet("Cantilever Fine Disp", cb_fine.disp_rel_error_pct, 10.24);
    check_ratchet("Cantilever Coarse Root Stress", cb_coarse.stress_rel_error_pct, 46.18);
    check_ratchet("Cantilever Medium Root Stress", cb_medium.stress_rel_error_pct, 21.91);
    check_ratchet("Cantilever Fine Root Stress", cb_fine.stress_rel_error_pct, 11.63);
    check_ratchet("Cantilever Coarse 0.5h Stress", cb_half_err_c, 56.86);
    check_ratchet("Cantilever Medium 0.5h Stress", cb_half_err_m, 28.21);
    check_ratchet("Cantilever Fine 0.5h Stress", cb_half_err_f, 18.18);

    // Benchmark 3: Plate with Hole (measured disp err: 1.73%, 1.45%, 1.39%; stress err vs Peterson: 18.39%, 16.19%, 16.58%)
    check_ratchet("Plate w/ Hole Coarse Disp", pl_coarse.disp_rel_error_pct, 1.73);
    check_ratchet("Plate w/ Hole Medium Disp", pl_medium.disp_rel_error_pct, 1.45);
    check_ratchet("Plate w/ Hole Fine Disp", pl_fine.disp_rel_error_pct, 1.39);
    check_ratchet("Plate w/ Hole Coarse Stress", pl_coarse.stress_rel_error_pct, 18.39);
    check_ratchet("Plate w/ Hole Medium Stress", pl_medium.stress_rel_error_pct, 16.19);
    check_ratchet("Plate w/ Hole Fine Stress", pl_fine.stress_rel_error_pct, 16.58);

    // Benchmark 4: Lamé Cylinder (measured disp err: 31.47%, 29.76%, 32.48%; stress err: 28.24%, 38.58%, 28.98%)
    check_ratchet("Lamé Cylinder Coarse Disp", lm_coarse.disp_rel_error_pct, 31.47);
    check_ratchet("Lamé Cylinder Medium Disp", lm_medium.disp_rel_error_pct, 29.76);
    check_ratchet("Lamé Cylinder Fine Disp", lm_fine.disp_rel_error_pct, 32.48);
    check_ratchet("Lamé Cylinder Coarse Stress", lm_coarse.stress_rel_error_pct, 28.24);
    check_ratchet("Lamé Cylinder Medium Stress", lm_medium.stress_rel_error_pct, 38.58);
    check_ratchet("Lamé Cylinder Fine Stress", lm_fine.stress_rel_error_pct, 28.98);
}

fn run_cantilever_case_tet10(
    density: MeshDensity,
    density_name: &'static str,
) -> (BenchmarkRow, f64, f64) {
    let t0 = Instant::now();
    let material = benchmark_steel();
    let length = 0.2f64;
    let width = 0.02f64;
    let height = 0.02f64;
    let force_y = -1000.0f64; // N

    let i_moment = width * height.powi(3) / 12.0;
    let expected_disp_eb = force_y.abs() * length.powi(3) / (3.0 * 205.0e9 * i_moment); // 0.975610 mm
    let g_shear = 205.0e9 / (2.0 * (1.0 + 0.30));
    let expected_disp_timoshenko =
        expected_disp_eb + (force_y.abs() * length) / ((5.0 / 6.0) * g_shear * (width * height)); // 0.983220 mm
    let expected_stress_root = 6.0 * force_y.abs() * length / (width * height.powi(2)); // 150.0 MPa
    let expected_stress_half_h =
        6.0 * force_y.abs() * (length - 0.5 * height) / (width * height.powi(2)); // 142.5 MPa

    let box_mesh = create_box(length, height, width);
    let tet4_mesh = TetrahedralMesh::from_mesh(&box_mesh, density).expect("Meshing failed");
    let tet10_mesh = tet4_mesh.to_tet10();

    let mut bc = BoundaryConditions::new();
    bc.fix_aabb_face(AabbFace::MinX, 1e-4);
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-4, [0.0, force_y, 0.0]);

    let solver_opts = SolverOptions {
        tolerance: 1e-8,
        max_iterations: 8000,
    };

    let report = analyze_mesh("cantilever_tet10", &tet10_mesh, &material, &bc, Some(solver_opts))
        .expect("TET10 FEA solve failed");
    assert_eq!(report.element_order, 2);

    let calc_disp_mm = report.max_displacement_m * 1000.0;
    let an_disp_mm = expected_disp_eb * 1000.0;
    let disp_err = (report.max_displacement_m - expected_disp_eb).abs() / expected_disp_eb * 100.0;
    let disp_err_timo = (report.max_displacement_m - expected_disp_timoshenko).abs()
        / expected_disp_timoshenko
        * 100.0;

    let calc_stress_mpa = report.max_stress_von_mises_pa / 1e6;
    let an_stress_mpa = expected_stress_root / 1e6;
    let stress_err =
        (report.max_stress_von_mises_pa - expected_stress_root).abs() / expected_stress_root * 100.0;

    let min_x = tet10_mesh.bounding_box().min[0];
    let target_x = min_x + 0.5 * height;
    let nodes_at_half_h = tet10_mesh.find_nodes_by_predicate(|p| (p[0] - target_x).abs() <= 0.0025);
    let max_stress_at_half_h = nodes_at_half_h
        .iter()
        .map(|&nid| report.nodal_von_mises_stresses[nid])
        .fold(0.0_f64, f64::max)
        / 1e6;
    let half_h_err =
        (max_stress_at_half_h * 1e6 - expected_stress_half_h).abs() / expected_stress_half_h * 100.0;

    let row = BenchmarkRow {
        name: "2b. Cantilever TET10 (200x20x20mm)".to_string(),
        density: density_name,
        element_count: tet10_mesh.tet10_elements.len(),
        node_count: tet10_mesh.nodes.len(),
        calculated_disp_mm: calc_disp_mm,
        analytical_disp_mm: an_disp_mm,
        disp_rel_error_pct: disp_err,
        calculated_stress_mpa: calc_stress_mpa,
        analytical_stress_mpa: an_stress_mpa,
        stress_rel_error_pct: stress_err,
        elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
        additional_note: format!(
            "Timoshenko_err={:.2}% | sigma(0.5h)={:.2} MPa (an=142.50, err={:.2}%)",
            disp_err_timo, max_stress_at_half_h, half_h_err
        ),
    };
    (row, max_stress_at_half_h, half_h_err)
}

#[test]
fn test_p3_t4_tet10_cantilever_benchmark() {
    println!("\n====================================================================================================");
    println!("MCP-CAD-FEA TET10 QUADRATIC ELEMENT CANTILEVER BENCHMARK (P3-T4)");
    println!("====================================================================================================");

    let (cb10_coarse, _, cb10_half_err_c) = run_cantilever_case_tet10(MeshDensity::Coarse, "Coarse");
    cb10_coarse.print_summary();
    let (cb10_medium, _, cb10_half_err_m) = run_cantilever_case_tet10(MeshDensity::Medium, "Medium");
    cb10_medium.print_summary();
    let (cb10_fine, _, cb10_half_err_f) = run_cantilever_case_tet10(MeshDensity::Fine, "Fine");
    cb10_fine.print_summary();
    println!("====================================================================================================\n");

    // P3-T4 acceptance criterion: on Cantilever Medium, TET10 deflection error <= 5% (vs 20.14% for TET4)
    assert!(
        cb10_medium.disp_rel_error_pct <= 5.0,
        "TET10 Cantilever Medium displacement error should be <= 5.0%, got {:.2}%",
        cb10_medium.disp_rel_error_pct
    );
    assert!(
        cb10_fine.disp_rel_error_pct <= 5.0,
        "TET10 Cantilever Fine displacement error should be <= 5.0%, got {:.2}%",
        cb10_fine.disp_rel_error_pct
    );
    assert!(
        cb10_coarse.disp_rel_error_pct < 10.0,
        "TET10 Cantilever Coarse displacement error should be < 10.0% (vs 48.15% for TET4), got {:.2}%",
        cb10_coarse.disp_rel_error_pct
    );
    let _ = (cb10_half_err_c, cb10_half_err_m, cb10_half_err_f);
}


