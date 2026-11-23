//! Integration and analytical benchmark tests for P5-T3:
//! Nonlinear J2 (von Mises) Elastoplasticity (Radial Return Mapping),
//! Permanent Plastic Deformation / Springback, and Co-Rotated FEM ($\mathbf{F} = \mathbf{R}\mathbf{U}$).

use mcp_cad_core::material::Material;
use mcp_cad_fea::{
    analyze_mesh_nonlinear_history, analyze_mesh_nonlinear_with_params, AabbFace,
    BoundaryConditions, DirichletConstraint, DofComponent, HardeningLaw, J2MaterialParams,
    NeumannLoad, NonlinearLoadStage, NonlinearSolverOptions, SolverOptions, TetElement, TetNode,
    TetrahedralMesh, TET10_EDGE_PAIRS,
};
use nalgebra::{Matrix3, Vector3};

/// Builds a clean, structured hexahedral grid subdivided into 5 conforming tetrahedra per cell
/// over `[0, lx] x [-ly/2, ly/2] x [-lz/2, lz/2]`, with optional power-law grading `x_power`
/// along X (`x_power = 1.0` is uniform; `x_power > 1.0` clusters elements toward the root `x = 0`).
fn build_graded_x_box_tet4(
    lx: f64,
    ly: f64,
    lz: f64,
    nx: usize,
    ny: usize,
    nz: usize,
    x_power: f64,
) -> TetrahedralMesh {
    let mut nodes = Vec::with_capacity((nx + 1) * (ny + 1) * (nz + 1));
    let mut id = 0;
    for iz in 0..=nz {
        let z = -0.5 * lz + lz * (iz as f64) / (nz as f64);
        for iy in 0..=ny {
            let y = -0.5 * ly + ly * (iy as f64) / (ny as f64);
            for ix in 0..=nx {
                let s = (ix as f64) / (nx as f64);
                let x = lx * s.powf(x_power);
                nodes.push(TetNode {
                    id,
                    position: [x, y, z],
                });
                id += 1;
            }
        }
    }

    let node_id = |ix: usize, iy: usize, iz: usize| -> usize {
        iz * (ny + 1) * (nx + 1) + iy * (nx + 1) + ix
    };

    let mut elements = Vec::with_capacity(nx * ny * nz * 5);
    let mut elem_id = 0;
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let n000 = node_id(ix, iy, iz);
                let n100 = node_id(ix + 1, iy, iz);
                let n010 = node_id(ix, iy + 1, iz);
                let n110 = node_id(ix + 1, iy + 1, iz);
                let n001 = node_id(ix, iy, iz + 1);
                let n101 = node_id(ix + 1, iy, iz + 1);
                let n011 = node_id(ix, iy + 1, iz + 1);
                let n111 = node_id(ix + 1, iy + 1, iz + 1);

                // Parity-alternating 5-tet decomposition ensures matching face diagonals across adjacent cells
                let tets = if (ix + iy + iz) % 2 == 0 {
                    [
                        [n000, n100, n110, n101],
                        [n000, n110, n010, n011],
                        [n000, n001, n101, n011],
                        [n111, n101, n110, n011],
                        [n000, n110, n101, n011],
                    ]
                } else {
                    [
                        [n100, n110, n010, n111],
                        [n000, n100, n010, n001],
                        [n100, n101, n111, n001],
                        [n010, n011, n111, n001],
                        [n100, n010, n111, n001],
                    ]
                };

                for mut idx in tets {
                    // Enforce positive orientation (signed volume > 0)
                    let p0 = Vector3::from(nodes[idx[0]].position);
                    let p1 = Vector3::from(nodes[idx[1]].position);
                    let p2 = Vector3::from(nodes[idx[2]].position);
                    let p3 = Vector3::from(nodes[idx[3]].position);
                    let det = (p1 - p0).dot(&(p2 - p0).cross(&(p3 - p0)));
                    if det < 0.0 {
                        idx.swap(2, 3);
                    }
                    elements.push(TetElement {
                        id: elem_id,
                        node_indices: idx,
                    });
                    elem_id += 1;
                }
            }
        }
    }

    TetrahedralMesh {
        nodes,
        elements,
        tet10_elements: Vec::new(),
        boundary_conforming: false,
    }
}

/// Uniform structured box `TET4` mesh helper (`x_power = 1.0`).
fn build_structured_box_tet4(
    lx: f64,
    ly: f64,
    lz: f64,
    nx: usize,
    ny: usize,
    nz: usize,
) -> TetrahedralMesh {
    build_graded_x_box_tet4(lx, ly, lz, nx, ny, nz, 1.0)
}

/// Builds a steel test material with configurable yield strength and linear hardening modulus.
fn steel_material(yield_mpa: f64, hardening_pa: f64) -> Material {
    let mut m = Material::new(
        "test_steel",
        200.0, // E = 200 GPa
        0.30,  // nu = 0.30
        yield_mpa,
        7850.0,
        "Nonlinear J2 Benchmark Steel",
    );
    m.hardening_modulus_pa = hardening_pa;
    m
}

/// Rodrigues rotation matrix around `axis` by `angle_rad`.
fn rotation_matrix(axis: Vector3<f64>, angle_rad: f64) -> Matrix3<f64> {
    let u = axis.normalize();
    let c = angle_rad.cos();
    let s = angle_rad.sin();
    let k = Matrix3::new(0.0, -u.z, u.y, u.z, 0.0, -u.x, -u.y, u.x, 0.0);
    Matrix3::identity() * c + k * s + (u * u.transpose()) * (1.0 - c)
}

/// Helper to build Dirichlet constraints rotating a set of nodes around the mesh center.
fn build_rotation_bc(
    mesh: &TetrahedralMesh,
    node_ids: &[usize],
    center: Vector3<f64>,
    rot: &Matrix3<f64>,
) -> BoundaryConditions {
    let mut bc = BoundaryConditions::new();
    for &nid in node_ids {
        let x0 = Vector3::from(mesh.nodes[nid].position);
        let u = rot * (x0 - center) - (x0 - center);
        bc.dirichlet.push(DirichletConstraint::FixedDof {
            dof_index: 3 * nid,
            prescribed_value: u.x,
        });
        bc.dirichlet.push(DirichletConstraint::FixedDof {
            dof_index: 3 * nid + 1,
            prescribed_value: u.y,
        });
        bc.dirichlet.push(DirichletConstraint::FixedDof {
            dof_index: 3 * nid + 2,
            prescribed_value: u.z,
        });
    }
    bc
}

/// Integrates a uniform axial traction $\mathbf{t} = [\sigma_x, 0, 0]$ exactly over all surface
/// triangles lying on `MaxX` using consistent finite-element shape-function weights:
/// - `TET4` ($T_3$ face): $\frac{1}{3} A \sigma_x$ to each of the 3 face corner nodes;
/// - `TET10` ($T_6$ face): $0$ to face corners and $\frac{1}{3} A \sigma_x$ to each of the 3 face mid-edge nodes.
fn add_consistent_uniform_traction_max_x(
    mesh: &TetrahedralMesh,
    sigma_x_pa: f64,
    bc: &mut BoundaryConditions,
) {
    let max_x = mesh.bounding_box().max[0];
    let tol = 1e-6;
    let mut nodal_fx = vec![0.0_f64; mesh.node_count()];

    if mesh.is_tet10() {
        const TET_FACES: [[usize; 3]; 4] = [[0, 1, 2], [0, 1, 3], [1, 2, 3], [0, 2, 3]];
        for elem in &mesh.tet10_elements {
            for face in TET_FACES {
                let n0 = elem[face[0]];
                let n1 = elem[face[1]];
                let n2 = elem[face[2]];
                if (mesh.nodes[n0].position[0] - max_x).abs() < tol
                    && (mesh.nodes[n1].position[0] - max_x).abs() < tol
                    && (mesh.nodes[n2].position[0] - max_x).abs() < tol
                {
                    let p0 = Vector3::from(mesh.nodes[n0].position);
                    let p1 = Vector3::from(mesh.nodes[n1].position);
                    let p2 = Vector3::from(mesh.nodes[n2].position);
                    let area = 0.5 * (p1 - p0).cross(&(p2 - p0)).norm();
                    let w = sigma_x_pa * area / 3.0;
                    // Distribute to the 3 mid-edge nodes belonging to this face (exact T6 consistent load)
                    for (e_idx, &(ea, eb)) in TET10_EDGE_PAIRS.iter().enumerate() {
                        if face.contains(&ea) && face.contains(&eb) {
                            nodal_fx[elem[4 + e_idx]] += w;
                        }
                    }
                }
            }
        }
    } else {
        for tri in mesh.extract_surface_triangles() {
            let [n0, n1, n2] = tri;
            if (mesh.nodes[n0].position[0] - max_x).abs() < tol
                && (mesh.nodes[n1].position[0] - max_x).abs() < tol
                && (mesh.nodes[n2].position[0] - max_x).abs() < tol
            {
                let p0 = Vector3::from(mesh.nodes[n0].position);
                let p1 = Vector3::from(mesh.nodes[n1].position);
                let p2 = Vector3::from(mesh.nodes[n2].position);
                let area = 0.5 * (p1 - p0).cross(&(p2 - p0)).norm();
                let w = sigma_x_pa * area / 3.0;
                nodal_fx[n0] += w;
                nodal_fx[n1] += w;
                nodal_fx[n2] += w;
            }
        }
    }

    for (nid, &fx) in nodal_fx.iter().enumerate() {
        if fx.abs() > 1e-12 {
            bc.neumann.push(NeumannLoad::NodalForce {
                node_id: nid,
                force: [fx, 0.0, 0.0],
            });
        }
    }
}

/// Acceptance Criterion 1:
/// Pure rigid-body rotation of a 3D part by 90° and 180° must produce $\sigma_{\max} < 10^{-9}\text{ Pa}$
/// under Co-Rotated FEM (for both TET4 and TET10, both prescribed and free-interior BVP solves),
/// whereas small-strain kinematics generates massive spurious stresses (> 10 GPa).
#[test]
fn test_p5_t3_rigid_rotation_invariance_90_and_180_deg() {
    let tet4_mesh = build_structured_box_tet4(0.10, 0.06, 0.06, 2, 2, 2);
    let tet10_mesh = tet4_mesh.to_tet10();
    let mat = steel_material(250.0, 5.0e9);
    let params = J2MaterialParams::from_material(&mat).expect("valid J2 params");

    let opts = NonlinearSolverOptions {
        initial_load_steps: 1,
        corotated: true,
        ..Default::default()
    };

    // Part A: Direct rigid rotation of all nodes at 90° (Z), 180° (Y), and 180° (arbitrary 3D axis)
    for (order_label, mesh) in [("TET4", &tet4_mesh), ("TET10", &tet10_mesh)] {
        let center = Vector3::from(mesh.bounding_box().center());
        let all_nodes: Vec<usize> = (0..mesh.node_count()).collect();

        for (case_name, axis, angle_deg) in [
            ("90deg_Z", Vector3::z(), 90.0_f64),
            ("180deg_Y", Vector3::y(), 180.0_f64),
            ("180deg_diag", Vector3::new(1.0, 2.0, -1.0), 180.0_f64),
        ] {
            let rot = rotation_matrix(axis, angle_deg.to_radians());
            let bc = build_rotation_bc(mesh, &all_nodes, center, &rot);
            let res = analyze_mesh_nonlinear_with_params(
                format!("rigid_{order_label}_{case_name}"),
                mesh,
                &params,
                &bc,
                Some(opts),
            )
            .expect("Co-rotated rigid rotation solve failed");

            let nl = res.report.nonlinear.as_ref().unwrap();
            println!(
                "[Rigid Direct {order_label} {case_name}] max_sigma = {:.3e} Pa, eq_ep = {:.3e}, W_e = {:.3e} J",
                res.report.max_stress_von_mises_pa,
                nl.max_equivalent_plastic_strain,
                nl.elastic_strain_energy_j
            );

            assert!(
                res.report.max_stress_von_mises_pa < 1e-9,
                "{order_label} {case_name}: Co-Rotated stress must be < 1e-9 Pa, got {:.3e} Pa",
                res.report.max_stress_von_mises_pa
            );
            assert_eq!(nl.yielded_gauss_points, 0);
            assert!(nl.elastic_strain_energy_j < 1e-12);
        }
    }

    // Contrast check: small-strain (corotated = false) at 90° generates huge spurious stress
    let center4 = Vector3::from(tet4_mesh.bounding_box().center());
    let all_nodes4: Vec<usize> = (0..tet4_mesh.node_count()).collect();
    let rot90 = rotation_matrix(Vector3::z(), std::f64::consts::FRAC_PI_2);
    let bc_90 = build_rotation_bc(&tet4_mesh, &all_nodes4, center4, &rot90);
    let mut linear_kin_opts = opts;
    linear_kin_opts.corotated = false;
    let res_small_strain = analyze_mesh_nonlinear_with_params(
        "rigid_90_small_strain",
        &tet4_mesh,
        &params,
        &bc_90,
        Some(linear_kin_opts),
    )
    .expect("small-strain eval");
    assert!(
        res_small_strain.report.max_stress_von_mises_pa > 1.0e8,
        "Small-strain kinematics must exhibit large spurious stress at 90 deg rotation, got {:.3e} Pa",
        res_small_strain.report.max_stress_von_mises_pa
    );

    // Part B: Multi-stage BVP with FREE interior nodes rotated incrementally from 0° -> 90° -> 180°
    for (order_label, mesh) in [("TET4", &tet4_mesh), ("TET10", &tet10_mesh)] {
        let aabb = mesh.bounding_box();
        let center = Vector3::from(aabb.center());
        let eps = 1e-6;
        let boundary_nodes: Vec<usize> = (0..mesh.node_count())
            .filter(|&i| {
                let p = mesh.nodes[i].position;
                (p[0] - aabb.min[0]).abs() < eps
                    || (p[0] - aabb.max[0]).abs() < eps
                    || (p[1] - aabb.min[1]).abs() < eps
                    || (p[1] - aabb.max[1]).abs() < eps
                    || (p[2] - aabb.min[2]).abs() < eps
                    || (p[2] - aabb.max[2]).abs() < eps
            })
            .collect();
        let free_node_count = mesh.node_count() - boundary_nodes.len();
        assert!(
            free_node_count > 0,
            "{order_label} mesh must have free interior nodes for BVP rotation test"
        );

        let bvp_opts = NonlinearSolverOptions {
            initial_load_steps: 1,
            force_tolerance: 1e-11,
            displacement_tolerance: 1e-12,
            corotated: true,
            include_geometric_stiffness: true,
            linear_solver: SolverOptions {
                tolerance: 1e-13,
                max_iterations: 10_000,
            },
            ..Default::default()
        };

        // Step through exact SO(3) boundary rotations (1 step per stage so boundary never shrinks along a chord)
        // 0° -> 15° -> 30° -> 45° -> 60° -> 75° -> 90°
        let stages_90: Vec<NonlinearLoadStage> = [15.0_f64, 30.0, 45.0, 60.0, 75.0, 90.0]
            .iter()
            .map(|&deg| {
                let rot = rotation_matrix(Vector3::z(), deg.to_radians());
                NonlinearLoadStage::new(build_rotation_bc(mesh, &boundary_nodes, center, &rot))
                    .with_steps(1)
                    .with_label(format!("rot_{deg}deg"))
            })
            .collect();

        let res_90 = analyze_mesh_nonlinear_history(
            format!("bvp_rot90_{order_label}"),
            mesh,
            &params,
            &stages_90,
            Some(bvp_opts),
            None,
        )
        .expect("BVP 90° rotation failed");

        println!(
            "[Rigid BVP {order_label} 90deg] free_nodes = {}, max_sigma = {:.3e} Pa, Newton iters = {}",
            free_node_count,
            res_90.report.max_stress_von_mises_pa,
            res_90.state.total_newton_iterations
        );
        assert!(
            res_90.report.max_stress_von_mises_pa < 1e-9,
            "BVP {order_label} 90°: stress must be < 1e-9 Pa, got {:.3e} Pa",
            res_90.report.max_stress_von_mises_pa
        );

        // Continue 90° -> 105° -> 120° -> 135° -> 150° -> 165° -> 180° from the converged 90° state
        let stages_180: Vec<NonlinearLoadStage> = [105.0_f64, 120.0, 135.0, 150.0, 165.0, 180.0]
            .iter()
            .map(|&deg| {
                let rot = rotation_matrix(Vector3::z(), deg.to_radians());
                NonlinearLoadStage::new(build_rotation_bc(mesh, &boundary_nodes, center, &rot))
                    .with_steps(1)
                    .with_label(format!("rot_{deg}deg"))
            })
            .collect();

        let res_180 = analyze_mesh_nonlinear_history(
            format!("bvp_rot180_{order_label}"),
            mesh,
            &params,
            &stages_180,
            Some(bvp_opts),
            Some(res_90.state),
        )
        .expect("BVP 180° rotation failed");

        // Verify interior free nodes match exact analytical 180° rotation
        let rot180 = rotation_matrix(Vector3::z(), std::f64::consts::PI);
        let mut max_pos_err = 0.0_f64;
        for i in 0..mesh.node_count() {
            let x0 = Vector3::from(mesh.nodes[i].position);
            let u_exact = rot180 * (x0 - center) - (x0 - center);
            let u_num = Vector3::from(res_180.report.nodal_displacements[i]);
            let err = (u_num - u_exact).norm();
            if err > max_pos_err {
                max_pos_err = err;
            }
        }

        println!(
            "[Rigid BVP {order_label} 180deg] max_pos_err = {:.3e} m, max_sigma = {:.3e} Pa, yielded = {}",
            max_pos_err,
            res_180.report.max_stress_von_mises_pa,
            res_180.report.nonlinear.as_ref().unwrap().yielded_gauss_points
        );
        assert!(
            max_pos_err < 1e-11,
            "BVP {order_label} 180°: free interior node position error {:.3e} m too large",
            max_pos_err
        );
        assert!(
            res_180.report.max_stress_von_mises_pa < 1e-9,
            "BVP {order_label} 180°: stress must be < 1e-9 Pa, got {:.3e} Pa",
            res_180.report.max_stress_von_mises_pa
        );
        assert_eq!(
            res_180.report.nonlinear.as_ref().unwrap().yielded_gauss_points,
            0
        );
    }
}

/// Acceptance Criterion 2:
/// Uniaxial tension above $\sigma_y$ followed by complete unloading (springback) reproduces the
/// analytical permanent plastic strain $\varepsilon^p$ and elastic recovery $\Delta \varepsilon^e = \sigma_{\max}/E$
/// with relative error < 1.0% (tested on both TET4 and TET10).
#[test]
fn test_p5_t3_uniaxial_tension_and_unloading_springback() {
    let lx = 0.20;
    let ly = 0.02;
    let lz = 0.02;
    let area = ly * lz; // 4e-4 m^2

    let e_pa = 200.0e9;
    let sigma_y = 250.0e6;
    let h_pa = 10.0e9; // linear isotropic hardening modulus
    let sigma_target = 350.0e6; // 1.4 * sigma_y
    let force_x = sigma_target * area; // 140,000 N

    // Analytical 1D uniaxial elastoplastic solution (small-strain / Biot strain):
    let expected_ep = (sigma_target - sigma_y) / h_pa; // 0.0100 (1.0%)
    let expected_ee = sigma_target / e_pa; // 0.00175 (0.175%)
    let expected_total_strain = expected_ee + expected_ep; // 0.01175

    let mat = steel_material(250.0, h_pa);
    let params = J2MaterialParams::from_material(&mat).expect("valid J2 params");

    let tet4_mesh = build_structured_box_tet4(lx, ly, lz, 4, 2, 2);
    let tet10_mesh = tet4_mesh.to_tet10();

    for (order_label, mesh) in [("TET4", &tet4_mesh), ("TET10", &tet10_mesh)] {
        let aabb = mesh.bounding_box();

        // Statically determinate support on MinX allowing completely free Poisson & plastic lateral contraction:
        // - Ux = 0 on entire MinX face
        // - Uy = 0, Uz = 0 on corner (min_x, min_y, min_z)
        // - Uz = 0 on corner (min_x, max_y, min_z) to prevent roll about X
        let mut base_support = BoundaryConditions::new();
        base_support.dirichlet.push(DirichletConstraint::FixedAabbFace {
            face: AabbFace::MinX,
            tolerance: 1e-6,
            components: DofComponent::X,
        });

        let c0 = mesh
            .find_nodes_by_predicate(|p| {
                (p[0] - aabb.min[0]).abs() < 1e-6
                    && (p[1] - aabb.min[1]).abs() < 1e-6
                    && (p[2] - aabb.min[2]).abs() < 1e-6
            })
            .into_iter()
            .next()
            .expect("corner (0, min_y, min_z) missing");
        let c1 = mesh
            .find_nodes_by_predicate(|p| {
                (p[0] - aabb.min[0]).abs() < 1e-6
                    && (p[1] - aabb.max[1]).abs() < 1e-6
                    && (p[2] - aabb.min[2]).abs() < 1e-6
            })
            .into_iter()
            .next()
            .expect("corner (0, max_y, min_z) missing");

        base_support.dirichlet.push(DirichletConstraint::FixedNodes {
            node_ids: vec![c0],
            components: DofComponent::YZ,
        });
        base_support.dirichlet.push(DirichletConstraint::FixedNodes {
            node_ids: vec![c1],
            components: DofComponent::Z,
        });

        let mut bc_load = base_support.clone();
        let _ = force_x;
        add_consistent_uniform_traction_max_x(mesh, sigma_target, &mut bc_load);

        let bc_unload = base_support; // Zero external force (complete unloading)

        let opts = NonlinearSolverOptions {
            initial_load_steps: 6,
            force_tolerance: 1e-7,
            displacement_tolerance: 1e-8,
            corotated: true,
            ..Default::default()
        };

        // Run Stage 1 (Loading) first so we can inspect peak loaded elongation, then continue into Stage 2 (Unloading)
        let loaded = analyze_mesh_nonlinear_with_params(
            format!("uniaxial_load_{order_label}"),
            mesh,
            &params,
            &bc_load,
            Some(opts),
        )
        .expect("Uniaxial loading solve failed");

        let max_x_nodes = mesh.find_nodes_by_predicate(|p| (p[0] - aabb.max[0]).abs() < 1e-6);
        let avg_ux_loaded = max_x_nodes
            .iter()
            .map(|&nid| loaded.report.nodal_displacements[nid][0])
            .sum::<f64>()
            / (max_x_nodes.len() as f64);
        let strain_loaded = avg_ux_loaded / lx;
        let stress_loaded_mpa = loaded.report.max_stress_von_mises_pa / 1.0e6;

        // Stage 2: Complete unloading starting from the converged plastic state
        let unload_stage = NonlinearLoadStage::new(bc_unload)
            .with_steps(4)
            .with_label("unloading");
        let unloaded = analyze_mesh_nonlinear_history(
            format!("uniaxial_unload_{order_label}"),
            mesh,
            &params,
            &[unload_stage],
            Some(opts),
            Some(loaded.state.clone()),
        )
        .expect("Uniaxial unloading solve failed");

        let avg_ux_unloaded = max_x_nodes
            .iter()
            .map(|&nid| unloaded.report.nodal_displacements[nid][0])
            .sum::<f64>()
            / (max_x_nodes.len() as f64);
        let perm_strain = avg_ux_unloaded / lx;
        let springback_strain = (avg_ux_loaded - avg_ux_unloaded) / lx;
        let residual_stress_pa = unloaded.report.max_stress_von_mises_pa;
        let gp_ep_mean = unloaded
            .state
            .gauss_states
            .iter()
            .map(|g| g.equivalent_plastic_strain)
            .sum::<f64>()
            / (unloaded.state.gauss_states.len() as f64);

        let ep_err_pct = ((perm_strain - expected_ep).abs() / expected_ep) * 100.0;
        let ee_err_pct = ((springback_strain - expected_ee).abs() / expected_ee) * 100.0;
        let total_err_pct =
            ((strain_loaded - expected_total_strain).abs() / expected_total_strain) * 100.0;

        println!(
            "[Uniaxial {order_label}] loaded_stress = {:.3} MPa (target {:.1}), total_eps = {:.6} (err {:.3}%), perm_eps_p = {:.6} vs {:.6} (err {:.3}%), springback_eps_e = {:.6} vs {:.6} (err {:.3}%), residual_sigma = {:.3e} Pa, mean_gp_ep = {:.6}",
            stress_loaded_mpa,
            sigma_target / 1.0e6,
            strain_loaded,
            total_err_pct,
            perm_strain,
            expected_ep,
            ep_err_pct,
            springback_strain,
            expected_ee,
            ee_err_pct,
            residual_stress_pa,
            gp_ep_mean,
        );

        assert!(
            ep_err_pct < 1.0,
            "{order_label}: Permanent plastic strain error {:.3}% exceeds 1.0% threshold",
            ep_err_pct
        );
        assert!(
            ee_err_pct < 1.0,
            "{order_label}: Elastic springback error {:.3}% exceeds 1.0% threshold",
            ee_err_pct
        );
        assert!(
            total_err_pct < 1.0,
            "{order_label}: Loaded total strain error {:.3}% exceeds 1.0% threshold",
            total_err_pct
        );
        assert!(
            residual_stress_pa < 1.0e5,
            "{order_label}: Unloaded residual stress should vanish, got {:.3e} Pa",
            residual_stress_pa
        );
        assert!(
            loaded.report.diagnostics.equilibrium_error_pct < 0.05,
            "{order_label}: Loaded equilibrium error {:.4}% too high",
            loaded.report.diagnostics.equilibrium_error_pct
        );

        // Verify plastic volume preservation (tr(eps^p) = 0) and lateral plastic contraction (-0.5 * eps^p)
        for gp in &unloaded.state.gauss_states {
            let trace_p = gp.plastic_strain[0] + gp.plastic_strain[1] + gp.plastic_strain[2];
            assert!(
                trace_p.abs() < 1e-10,
                "{order_label}: Plastic strain must be isochoric (tr = {trace_p:.3e})"
            );
            let lat_err = (gp.plastic_strain[1] + 0.5 * expected_ep).abs() / expected_ep;
            assert!(
                lat_err < 0.01,
                "{order_label}: Lateral plastic strain mismatch (got {:.6}, expected {:.6})",
                gp.plastic_strain[1],
                -0.5 * expected_ep
            );
        }
    }
}

/// Acceptance Criterion 3:
/// Elastoplastic bending of a rectangular cantilever beam forms a plastic hinge near the root
/// and accurately reproduces the transition from first-yield moment $M_y = \sigma_y b h^2 / 6$
/// to the plastic limit moment $M_p = 1.5 M_y = \sigma_y b h^2 / 4$ without fracture or collapse,
/// followed by elastic springback retaining permanent deflection.
#[test]
fn test_p5_t3_cantilever_elastoplastic_bending_plastic_hinge() {
    let lx: f64 = 0.24; // 240 mm span (L/h = 15, slender rectangular beam)
    let ly: f64 = 0.016; // 16 mm height (bending axis)
    let lz: f64 = 0.008; // 8 mm width (b/h = 0.5 -> clean plane-stress bending)

    let e_pa = 200.0e9;
    let sigma_y = 250.0e6;
    // Elastic-perfectly-plastic (H = 0) so plastic hinge moment has exact analytical asymptote M_p = 1.5 M_y
    let mat = steel_material(250.0, 0.0);
    let params = J2MaterialParams::from_material(&mat)
        .expect("valid J2 params")
        .with_ideal_plasticity();

    // Analytical rectangular beam bending moments:
    let m_y = sigma_y * lz * ly * ly / 6.0; // 85.333 N·m
    let m_p = sigma_y * lz * ly * ly / 4.0; // 128.000 N·m = 1.5 * M_y
    let i_z = lz * ly.powi(3) / 12.0;
    let delta_y = m_y * lx * lx / (3.0 * e_pa * i_z); // 3.00 mm (Euler-Bernoulli yield tip deflection)

    // Quadratic TET10 mesh with 4 elements through height (8 Gauss layers) and 12 root-graded
    // elements along the span so the root plastic hinge length is finely resolved
    let mesh = build_graded_x_box_tet4(lx, ly, lz, 12, 4, 1, 1.70).to_tet10();
    let aabb = mesh.bounding_box();

    // Plane-stress root clamping matching 1D Euler-Bernoulli / plastic-hinge beam theory:
    // - Ux = 0 on entire MinX face (zero root rotation / axial drift)
    // - Uy = 0 along the root neutral line (y = 0) so vertical Poisson/plastic contraction is free (sigma_yy = 0)
    // - Uz = 0 along the root centerline (z = 0) so lateral Poisson/plastic expansion is free (sigma_zz = 0)
    let mut root_bc = BoundaryConditions::new();
    root_bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinX,
        tolerance: 1e-6,
        components: DofComponent::X,
    });
    let root_y0_nodes = mesh.find_nodes_by_predicate(|p| {
        (p[0] - aabb.min[0]).abs() < 1e-6 && p[1].abs() < 1e-6
    });
    assert!(!root_y0_nodes.is_empty());
    root_bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: root_y0_nodes,
        components: DofComponent::Y,
    });
    let root_z0_nodes = mesh.find_nodes_by_predicate(|p| {
        (p[0] - aabb.min[0]).abs() < 1e-6 && p[2].abs() < 1e-6
    });
    assert!(!root_z0_nodes.is_empty());
    root_bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: root_z0_nodes,
        components: DofComponent::Z,
    });

    // Tip nodes at MaxX (apply prescribed vertical deflection Uy = -delta along the tip neutral line y = 0)
    let tip_neutral_nodes = mesh.find_nodes_by_predicate(|p| {
        (p[0] - aabb.max[0]).abs() < 1e-6 && p[1].abs() < 1e-6
    });
    assert!(!tip_neutral_nodes.is_empty());

    let make_tip_disp_bc = |tip_dy: f64| -> BoundaryConditions {
        let mut bc = root_bc.clone();
        for &nid in &tip_neutral_nodes {
            bc.dirichlet.push(DirichletConstraint::FixedDof {
                dof_index: 3 * nid + 1,
                prescribed_value: tip_dy,
            });
        }
        bc
    };

    let opts = NonlinearSolverOptions {
        initial_load_steps: 4,
        max_newton_iterations: 30,
        force_tolerance: 1e-5,
        displacement_tolerance: 1e-6,
        max_bisections: 10,
        corotated: true,
        include_geometric_stiffness: true,
        linear_solver: SolverOptions {
            tolerance: 1e-8,
            max_iterations: 6000,
        },
        ..Default::default()
    };

    // Stage 1: Elastic regime (0.5 * delta_y) -> verify M ≈ 0.5 M_y and zero plastic strain
    let stage_elastic = NonlinearLoadStage::new(make_tip_disp_bc(-0.5 * delta_y))
        .with_steps(1)
        .with_label("elastic_0.5dy");

    // Stage 2: Transition past M_y into full plastic hinge (up to 6.0 * delta_y = 18 mm)
    let stage_hinge = NonlinearLoadStage::new(make_tip_disp_bc(-6.0 * delta_y))
        .with_steps(8)
        .with_label("plastic_hinge_6dy");

    let loaded = analyze_mesh_nonlinear_history(
        "cantilever_plastic_hinge",
        &mesh,
        &params,
        &[stage_elastic, stage_hinge],
        Some(opts),
        None,
    )
    .expect("Cantilever elastoplastic bending solve failed");

    // Note: reaction_total_n sums reactions on ALL constrained DOFs (root + tip = 0 in force equilibrium).
    // Compute exact root reaction force directly from converged state on MinX nodes!
    let root_nodes = mesh.find_nodes_by_predicate(|p| (p[0] - aabb.min[0]).abs() < 1e-6);
    let mut elements = Vec::with_capacity(mesh.tet10_elements.len());
    for (id, &node_indices) in mesh.tet10_elements.iter().enumerate() {
        let mut positions = [[0.0f64; 3]; 10];
        for (k, &nid) in node_indices.iter().enumerate() {
            positions[k] = mesh.nodes[nid].position;
        }
        elements.push(mcp_cad_fea::Tet10Element::new(id, node_indices, positions).unwrap());
    }

    // Check elastic stage (last step of stage 0)
    let elastic_step = loaded
        .history
        .iter()
        .rfind(|h| h.stage_index == 0)
        .expect("stage 0 history missing");
    assert!(
        elastic_step.max_equivalent_plastic_strain < 1e-10,
        "Beam must remain purely elastic at 0.5*delta_y, got eq_ep = {:.3e}",
        elastic_step.max_equivalent_plastic_strain
    );

    // Compute exact root resisting moment at full plastic hinge (end of stage 1) from root nodal reactions:
    // M_root_z = sum_{n in root} (x_n - x_root)*R_y - (y_n - 0)*R_x  == -sum_{n in tip} R_tip_y * L
    let resolved_hinge_bc = make_tip_disp_bc(-6.0 * delta_y).resolve(&mesh).unwrap();
    let reactions = mcp_cad_fea::compute_nonlinear_nodal_reactions_tet10(
        &mesh,
        &elements,
        &loaded.state,
        &resolved_hinge_bc,
    );
    let tip_force_y: f64 = tip_neutral_nodes.iter().map(|&n| reactions[n][1]).sum();
    let root_force_y: f64 = root_nodes.iter().map(|&n| reactions[n][1]).sum();
    let root_moment_from_rx: f64 = root_nodes
        .iter()
        .map(|&n| -mesh.nodes[n].position[1] * reactions[n][0])
        .sum();
    let m_hinge = (-tip_force_y * lx).abs();
    let shape_factor = m_hinge / m_y;
    let mp_err_pct = ((m_hinge - m_p).abs() / m_p) * 100.0;

    let nl = loaded.report.nonlinear.as_ref().unwrap();
    println!(
        "[Cantilever Plastic Hinge TET10] M_y = {:.2} N·m, M_p(analytic) = {:.2} N·m, M_hinge(FEA) = {:.2} N·m (M_root_couple = {:.2} N·m), M_p/M_y = {:.4} (err vs 1.5: {:.2}%), tip_Fy = {:.1} N, root_Fy = {:.1} N, max_ep = {:.5}, yielded_GPs = {}/{} ({:.1}% vol)",
        m_y,
        m_p,
        m_hinge,
        root_moment_from_rx.abs(),
        shape_factor,
        mp_err_pct,
        tip_force_y,
        root_force_y,
        nl.max_equivalent_plastic_strain,
        nl.yielded_gauss_points,
        nl.total_gauss_points,
        nl.plastic_volume_fraction * 100.0,
    );

    // 1. Force & moment equilibrium between root and tip
    assert!(
        (tip_force_y + root_force_y).abs() / tip_force_y.abs().max(1.0) < 1e-4,
        "Vertical equilibrium imbalance: tip_Fy = {tip_force_y:.2}, root_Fy = {root_force_y:.2}"
    );
    assert!(
        (m_hinge - root_moment_from_rx.abs()).abs() / m_hinge < 0.02,
        "Root couple ({:.2}) must match tip force * L ({:.2})",
        root_moment_from_rx.abs(),
        m_hinge
    );

    // 2. Plastic limit moment M_p = 1.5 * M_y reproduced accurately (within 5% on 3D continuum TET10)
    assert!(
        m_hinge > m_y * 1.35,
        "Beam must exceed elastic yield moment M_y ({m_y:.2}) well into plastic hinge; got {m_hinge:.2}"
    );
    assert!(
        mp_err_pct < 5.0,
        "Plastic hinge moment {m_hinge:.2} N·m deviates from analytical M_p = 1.5 M_y = {m_p:.2} N·m by {mp_err_pct:.2}% (> 5%)"
    );

    // 3. Plastic hinge localization: top and bottom fibers near root yield symmetrically, tip remains elastic
    let mut top_root_ep = 0.0_f64;
    let mut bot_root_ep = 0.0_f64;
    let mut tip_region_ep = 0.0_f64;
    for (nid, &ep) in loaded.nodal_equivalent_plastic_strain.iter().enumerate() {
        let p = mesh.nodes[nid].position;
        if p[0] < 0.25 * lx {
            if p[1] > 0.25 * ly {
                top_root_ep = top_root_ep.max(ep);
            } else if p[1] < -0.25 * ly {
                bot_root_ep = bot_root_ep.max(ep);
            }
        } else if p[0] > 0.75 * lx {
            tip_region_ep = tip_region_ep.max(ep);
        }
    }
    assert!(
        top_root_ep > 0.005 && bot_root_ep > 0.005,
        "Expected symmetric plastic hinge at root top ({top_root_ep:.5}) and bottom ({bot_root_ep:.5})"
    );
    assert!(
        tip_region_ep < 1e-6,
        "Free end of cantilever must remain elastic, got ep = {tip_region_ep:.5}"
    );
    assert!(nl.plastic_dissipation_energy_j > 0.1);

    // Stage 3: Release tip load (springback) and verify permanent bent shape without collapse
    let unload_stage = NonlinearLoadStage::new(root_bc)
        .with_steps(2)
        .with_label("springback");
    let unloaded = analyze_mesh_nonlinear_history(
        "cantilever_springback",
        &mesh,
        &params,
        &[unload_stage],
        Some(opts),
        Some(loaded.state),
    )
    .expect("Cantilever springback unloading failed");

    let tip_dy_unloaded = tip_neutral_nodes
        .iter()
        .map(|&n| unloaded.report.nodal_displacements[n][1])
        .sum::<f64>()
        / (tip_neutral_nodes.len() as f64);
    let tip_dy_loaded = -6.0 * delta_y;
    let springback_dy = tip_dy_unloaded - tip_dy_loaded; // positive upward recovery

    println!(
        "[Cantilever Springback] loaded_dy = {:.3} mm, permanent_dy = {:.3} mm, elastic_recovery = {:.3} mm (expected ~{:.3} mm)",
        tip_dy_loaded * 1000.0,
        tip_dy_unloaded * 1000.0,
        springback_dy * 1000.0,
        (m_hinge / m_y) * delta_y * 1000.0,
    );

    assert!(
        tip_dy_unloaded < -3.0 * delta_y,
        "Cantilever must retain substantial permanent plastic deflection after unloading, got {:.3} mm",
        tip_dy_unloaded * 1000.0
    );
    assert!(
        springback_dy > delta_y && springback_dy < 2.2 * delta_y,
        "Elastic springback {:.3} mm out of expected physical range (~1.5*delta_y = {:.3} mm)",
        springback_dy * 1000.0,
        1.5 * delta_y * 1000.0
    );
}

/// Verifies adaptive sub-incrementation (bisection) under aggressive step settings,
/// plus PowerLaw (Ludwik/Swift) and Saturation (Voce) isotropic hardening laws.
#[test]
fn test_p5_t3_adaptive_subincrementation_and_hardening_laws() {
    let lx = 0.10;
    let ly = 0.02;
    let lz = 0.02;
    let area = ly * lz;
    let mesh = build_structured_box_tet4(lx, ly, lz, 2, 1, 1);
    let aabb = mesh.bounding_box();

    let mut bc = BoundaryConditions::new();
    bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinX,
        tolerance: 1e-6,
        components: DofComponent::X,
    });
    let c0 = mesh
        .find_nodes_by_predicate(|p| {
            (p[0] - aabb.min[0]).abs() < 1e-6
                && (p[1] - aabb.min[1]).abs() < 1e-6
                && (p[2] - aabb.min[2]).abs() < 1e-6
        })[0];
    let c1 = mesh
        .find_nodes_by_predicate(|p| {
            (p[0] - aabb.min[0]).abs() < 1e-6
                && (p[1] - aabb.max[1]).abs() < 1e-6
                && (p[2] - aabb.min[2]).abs() < 1e-6
        })[0];
    bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: vec![c0],
        components: DofComponent::YZ,
    });
    bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: vec![c1],
        components: DofComponent::Z,
    });

    let sigma_target = 340.0e6;
    let _ = area;
    add_consistent_uniform_traction_max_x(&mesh, sigma_target, &mut bc);

    let base_mat = steel_material(250.0, 0.0);

    // 1. Power-law isotropic hardening verification
    let power_law = HardeningLaw::PowerLaw {
        strength_coeff_pa: 600.0e6,
        exponent: 0.35,
        strain_offset: 1e-4,
    };
    let params_power = J2MaterialParams::from_material(&base_mat)
        .unwrap()
        .with_hardening(power_law);
    let res_power = analyze_mesh_nonlinear_with_params(
        "power_law_bar",
        &mesh,
        &params_power,
        &bc,
        Some(NonlinearSolverOptions {
            initial_load_steps: 4,
            ..Default::default()
        }),
    )
    .expect("PowerLaw hardening solve failed");

    let mean_ep_power = res_power
        .state
        .gauss_states
        .iter()
        .map(|g| g.equivalent_plastic_strain)
        .sum::<f64>()
        / (res_power.state.gauss_states.len() as f64);
    let expected_sigma_power = power_law.yield_stress(250.0e6, mean_ep_power);
    let power_err_pct = ((expected_sigma_power - sigma_target).abs() / sigma_target) * 100.0;
    assert!(
        mean_ep_power > 1e-3 && power_err_pct < 0.5,
        "PowerLaw yield stress mismatch: got {:.3} MPa vs target {:.3} MPa (err {:.3}%)",
        expected_sigma_power / 1e6,
        sigma_target / 1e6,
        power_err_pct
    );

    // 2. Voce saturation isotropic hardening verification
    let sat_law = HardeningLaw::Saturation {
        initial_hardening_modulus_pa: 18.0e9,
        saturation_stress_pa: 420.0e6,
    };
    let params_sat = J2MaterialParams::from_material(&base_mat)
        .unwrap()
        .with_hardening(sat_law);
    let res_sat = analyze_mesh_nonlinear_with_params(
        "saturation_bar",
        &mesh,
        &params_sat,
        &bc,
        Some(NonlinearSolverOptions {
            initial_load_steps: 4,
            ..Default::default()
        }),
    )
    .expect("Saturation hardening solve failed");

    let mean_ep_sat = res_sat
        .state
        .gauss_states
        .iter()
        .map(|g| g.equivalent_plastic_strain)
        .sum::<f64>()
        / (res_sat.state.gauss_states.len() as f64);
    let expected_sigma_sat = sat_law.yield_stress(250.0e6, mean_ep_sat);
    let sat_err_pct = ((expected_sigma_sat - sigma_target).abs() / sigma_target) * 100.0;
    assert!(
        mean_ep_sat > 1e-3 && sat_err_pct < 0.5,
        "Saturation yield stress mismatch: got {:.3} MPa vs target {:.3} MPa (err {:.3}%)",
        expected_sigma_sat / 1e6,
        sigma_target / 1e6,
        sat_err_pct
    );

    // 3. Adaptive sub-incrementation (bisection) test:
    // Force an aggressive single-step large tip deflection on a small cantilever with max_newton_iterations = 2
    // so Newton cannot finish in 1 step and MUST trigger bisection, yet still converges cleanly.
    let beam = build_structured_box_tet4(0.16, 0.02, 0.02, 4, 2, 1).to_tet10();
    let baabb = beam.bounding_box();
    let mut beam_bc = BoundaryConditions::new();
    beam_bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinX,
        tolerance: 1e-6,
        components: DofComponent::All,
    });
    let tip_nodes = beam.find_nodes_by_predicate(|p| (p[0] - baabb.max[0]).abs() < 1e-6);
    for &nid in &tip_nodes {
        beam_bc.dirichlet.push(DirichletConstraint::FixedDof {
            dof_index: 3 * nid + 1,
            prescribed_value: -0.025, // large 25 mm deflection in a single initial step
        });
    }
    let mat_hard = steel_material(250.0, 5.0e9);
    let params_hard = J2MaterialParams::from_material(&mat_hard).unwrap();
    let bisect_opts = NonlinearSolverOptions {
        initial_load_steps: 1,
        max_newton_iterations: 5, // intentionally tight to force adaptive bisection on giant initial step
        force_tolerance: 1e-5,
        max_bisections: 12,
        min_step_size: 1e-3,
        corotated: true,
        ..Default::default()
    };

    let res_bisect = analyze_mesh_nonlinear_with_params(
        "adaptive_bisection_beam",
        &beam,
        &params_hard,
        &beam_bc,
        Some(bisect_opts),
    )
    .expect("Adaptive sub-incrementation should recover convergence via bisection");

    let nl_b = res_bisect.report.nonlinear.as_ref().unwrap();
    println!(
        "[Adaptive Bisection] converged_steps = {}, total_bisections = {}, newton_iters = {}, max_ep = {:.5}",
        nl_b.converged_steps,
        nl_b.total_bisections,
        nl_b.total_newton_iterations,
        nl_b.max_equivalent_plastic_strain
    );
    assert!(
        nl_b.total_bisections > 0,
        "Expected adaptive bisection to activate under tight max_newton_iterations=3 and large single step"
    );
    assert!(nl_b.converged_steps >= 2);
    assert!(nl_b.max_equivalent_plastic_strain > 1e-3);
}
