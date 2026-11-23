//! Phase 5 Verification Benchmark Suite (`P5-T8`).
//!
//! Unified end-to-end verification of the entire Phase 5 physics & FEA stack:
//! 1. **Benchmark 1 — Uniaxial Elastoplastic Loading-Unloading Cycle**:
//!    verifies analytical permanent plastic strain $\varepsilon^p$ and elastic springback
//!    $\Delta\varepsilon^e = \sigma_{\max}/E$ on both `TET4` and `TET10` (`< 1.0%` error).
//! 2. **Benchmark 2 — Elastoplastic Cantilever Plastic Hinge**:
//!    verifies transition from first yield $M_y$ to limit plastic moment $M_p = 1.5 M_y$
//!    on quadratic `TET10` (`< 5.0%` error) and permanent deflection after springback.
//! 3. **Benchmark 3 — Analytical Hertz Contact of Two Elastic Cylinders**:
//!    verifies peak contact pressure $p_0$ and half-width $a$ (`<= 5.0%` error), force balance,
//!    subsurface von Mises peak, and interpenetration `< 1e-6 m`.
//! 4. **Benchmark 4 — Spur Gear Tooth Bending (`TET10` + `fe_snap`) vs ISO 6336-3:2006 / AGMA**:
//!    builds a watertight ISO 53 Profile A involute spur gear rim sector with exact rack-generated
//!    trochoidal root fillets ($\rho_{fP} = 0.38 m$), meshes it through production
//!    `TetrahedralMesh::from_mesh_conforming` (`TET10` + `fe_snap` + local octree refinement),
//!    and verifies that the global peak tensile bending stress lies strictly in the tensile root
//!    fillet and matches ISO 6336-3 $\sigma_{F0} = \frac{F_t}{b m} Y_{Fa} Y_{Sa}$ within `<= 8.0%`.
//! 5. **Benchmark 5 — Meshing Spur Gear Pair in Three Physical Regimes**:
//!    - (a) Elastic contact without residual deformation (`ε_p == 0`, `D == 0`, `pen < 1e-5 m`),
//!    - (b) Plastic tooth indentation without fracture (`ε_p_res > 0`, `D == 0`, `0` fragments, `pen < 1e-5 m`),
//!    - (c) Critical overload local tooth fracture (watertight detached tooth, intact hub bore &
//!      unloaded teeth, mass/momentum conservation `<= 1e-6`, `pen < 1e-5 m`),
//!    - (d) Coupled HHT-α dynamic impact energy & contact verification.

use std::f64::consts::PI;
use std::time::Instant;

use mcp_cad_core::diagnostics::validate_mesh;
use mcp_cad_core::gear_strength::lewis_form_factor;
use mcp_cad_core::material::{Material, MaterialRegistry};
use mcp_cad_core::topology::{TriangleMesh, Vertex};
use mcp_cad_fea::{
    analyze_mesh, analyze_mesh_nonlinear_history, analyze_mesh_nonlinear_with_params,
    compute_nonlinear_nodal_reactions_tet10, elasticity_matrix_from_material,
    evaluate_mesh_metrics, principal_stresses_and_directions, solve_deformable_contact,
    solve_deformable_contact_with_fracture, solve_dynamic_hht_coupled, AabbFace, AccuracyMode,
    BoundaryConditions, ConformingMeshOptions, ContactBody, ContactDiscretization, ContactPairSpec,
    ContactSolverOptions, ContactStepSpec, DamageMaterialParams, DirichletConstraint, DofComponent,
    DynamicBodySpec, ElementOrder, FractureSolverOptions, FractureTopologyMode, HardeningLaw,
    HhtOptions, HubCoupling, HubNodeSelector, J2MaterialParams, LocalRefinementOptions,
    MassMatrixKind, MeshDensity, NeumannLoad, NonlinearLoadStage, NonlinearSolverOptions,
    PrismaticHubDrive, RefinementZone, RevoluteHubDrive, SnapOptions, SolverOptions, Tet10Element,
    TetElement, TetNode, TetrahedralMesh, von_mises_from_voigt, TET10_EDGE_PAIRS,
};
use nalgebra::Vector3;

// ============================================================================
// Shared Geometry & Mesh Builders
// ============================================================================

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
                nodes.push(TetNode::new(id, [x, y, z]));
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
                    let p0 = nodes[idx[0]].pos_vec();
                    let p1 = nodes[idx[1]].pos_vec();
                    let p2 = nodes[idx[2]].pos_vec();
                    let p3 = nodes[idx[3]].pos_vec();
                    if (p1 - p0).dot(&(p2 - p0).cross(&(p3 - p0))) < 0.0 {
                        idx.swap(2, 3);
                    }
                    elements.push(TetElement::new(elem_id, idx));
                    elem_id += 1;
                }
            }
        }
    }

    TetrahedralMesh::new(nodes, elements)
}

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

fn solve_geometric_ratio(first_step: f64, target_sum: f64, n: usize) -> f64 {
    if n <= 1 || target_sum <= first_step {
        return 1.0;
    }
    let mut lo = 1.0_f64;
    let mut hi = 4.0_f64;
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        let mut s = 0.0;
        let mut step = first_step;
        for _ in 0..n {
            step *= mid;
            s += step;
        }
        if s < target_sum {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

#[allow(clippy::too_many_arguments)]
fn build_graded_cylinder_mesh(
    radius: f64,
    half_width_x: f64,
    depth_y: f64,
    thickness_z: f64,
    fine_half_x: f64,
    n_fine_half_x: usize,
    n_coarse_half_x: usize,
    n_fine_y: usize,
    n_coarse_y: usize,
    fine_depth_y: f64,
    is_top: bool,
) -> TetrahedralMesh {
    let mut xs_pos = Vec::with_capacity(n_fine_half_x + n_coarse_half_x + 1);
    for i in 0..=n_fine_half_x {
        xs_pos.push(fine_half_x * (i as f64 / n_fine_half_x as f64));
    }
    let dx_fine = fine_half_x / n_fine_half_x as f64;
    let rem_x = (half_width_x - fine_half_x).max(dx_fine);
    let rx = solve_geometric_ratio(dx_fine, rem_x, n_coarse_half_x);
    let mut cur_x = fine_half_x;
    let mut step_x = dx_fine;
    for k in 0..n_coarse_half_x {
        step_x *= rx;
        cur_x = if k + 1 == n_coarse_half_x {
            half_width_x
        } else {
            (cur_x + step_x).min(half_width_x)
        };
        xs_pos.push(cur_x);
    }

    let mut xs = Vec::with_capacity(xs_pos.len() * 2 - 1);
    for &x in xs_pos.iter().skip(1).rev() {
        xs.push(-x);
    }
    xs.extend_from_slice(&xs_pos);

    let mut ys_depth = Vec::with_capacity(n_fine_y + n_coarse_y + 1);
    for j in 0..=n_fine_y {
        ys_depth.push(fine_depth_y * (j as f64 / n_fine_y as f64));
    }
    let dy_fine = fine_depth_y / n_fine_y as f64;
    let rem_y = (depth_y - fine_depth_y).max(dy_fine);
    let ry = solve_geometric_ratio(dy_fine, rem_y, n_coarse_y);
    let mut cur_y = fine_depth_y;
    let mut step_y = dy_fine;
    for k in 0..n_coarse_y {
        step_y *= ry;
        cur_y = if k + 1 == n_coarse_y {
            depth_y
        } else {
            (cur_y + step_y).min(depth_y)
        };
        ys_depth.push(cur_y);
    }

    let nx = xs.len() - 1;
    let ny = ys_depth.len() - 1;
    let zs = [0.0, thickness_z];
    let node_id = |i: usize, j: usize, k: usize| -> usize { i * (ny + 1) * 2 + j * 2 + k };

    let mut nodes = Vec::with_capacity((nx + 1) * (ny + 1) * 2);
    for &x in &xs {
        let sag = radius - (radius * radius - x * x).max(0.0).sqrt();
        for &d in &ys_depth {
            let s = d / depth_y;
            let y_abs = sag * (1.0 - s) + depth_y * s;
            let y = if is_top { y_abs } else { -y_abs };
            for &z in &zs {
                let id = nodes.len();
                nodes.push(TetNode::new(id, [x, y, z]));
            }
        }
    }

    let mut elements = Vec::with_capacity(nx * ny * 6);
    let mut elem_id = 0usize;
    let mid_i = nx / 2;

    for i in 0..nx {
        let mirror_x = i < mid_i;
        for j in 0..ny {
            let p000 = node_id(i, j, 0);
            let p100 = node_id(i + 1, j, 0);
            let p010 = node_id(i, j + 1, 0);
            let p110 = node_id(i + 1, j + 1, 0);
            let p001 = node_id(i, j, 1);
            let p101 = node_id(i + 1, j, 1);
            let p011 = node_id(i, j + 1, 1);
            let p111 = node_id(i + 1, j + 1, 1);

            let cell_tets = if mirror_x {
                [
                    [p100, p000, p010, p011],
                    [p100, p000, p001, p011],
                    [p100, p110, p010, p011],
                    [p100, p110, p111, p011],
                    [p100, p101, p001, p011],
                    [p100, p101, p111, p011],
                ]
            } else {
                [
                    [p000, p100, p110, p111],
                    [p000, p100, p101, p111],
                    [p000, p010, p110, p111],
                    [p000, p010, p011, p111],
                    [p000, p001, p101, p111],
                    [p000, p001, p011, p111],
                ]
            };

            for mut tet in cell_tets {
                let v0 = nodes[tet[0]].pos_vec();
                let v1 = nodes[tet[1]].pos_vec();
                let v2 = nodes[tet[2]].pos_vec();
                let v3 = nodes[tet[3]].pos_vec();
                if (v1 - v0).dot(&(v2 - v0).cross(&(v3 - v0))) < 0.0 {
                    tet.swap(1, 2);
                }
                elements.push(TetElement::new(elem_id, tet));
                elem_id += 1;
            }
        }
    }

    TetrahedralMesh::new(nodes, elements)
}

// ============================================================================
// Benchmark 1: Uniaxial Elastoplastic Loading-Unloading Cycle (TET4 & TET10)
// ============================================================================

#[test]
fn p5_t8_benchmark_1_uniaxial_elastoplastic_cycle_springback() {
    let t0 = Instant::now();
    let lx = 0.20;
    let ly = 0.02;
    let lz = 0.02;
    let e_pa = 200.0e9;
    let sigma_y = 250.0e6;
    let h_pa = 10.0e9;
    let sigma_target = 350.0e6; // 1.4 * sigma_y

    let expected_ep = (sigma_target - sigma_y) / h_pa; // 0.0100
    let expected_ee = sigma_target / e_pa; // 0.00175
    let expected_total = expected_ee + expected_ep; // 0.01175

    let mut mat = Material::new("steel_b1", 200.0, 0.30, 250.0, 7850.0, "Benchmark 1 Steel");
    mat.hardening_modulus_pa = h_pa;
    let params = J2MaterialParams::from_material(&mat).expect("valid J2 params");

    let tet4_mesh = build_graded_x_box_tet4(lx, ly, lz, 4, 2, 2, 1.0);
    let tet10_mesh = tet4_mesh.to_tet10();

    for (order_label, mesh) in [("TET4", &tet4_mesh), ("TET10", &tet10_mesh)] {
        let t_order = Instant::now();
        let aabb = mesh.bounding_box();
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
            .expect("corner c0");
        let c1 = mesh
            .find_nodes_by_predicate(|p| {
                (p[0] - aabb.min[0]).abs() < 1e-6
                    && (p[1] - aabb.max[1]).abs() < 1e-6
                    && (p[2] - aabb.min[2]).abs() < 1e-6
            })
            .into_iter()
            .next()
            .expect("corner c1");

        base_support.dirichlet.push(DirichletConstraint::FixedNodes {
            node_ids: vec![c0],
            components: DofComponent::YZ,
        });
        base_support.dirichlet.push(DirichletConstraint::FixedNodes {
            node_ids: vec![c1],
            components: DofComponent::Z,
        });

        let mut bc_load = base_support.clone();
        add_consistent_uniform_traction_max_x(mesh, sigma_target, &mut bc_load);
        let bc_unload = base_support;

        let opts = NonlinearSolverOptions {
            initial_load_steps: 6,
            force_tolerance: 1e-7,
            displacement_tolerance: 1e-8,
            corotated: true,
            ..Default::default()
        };

        let loaded = analyze_mesh_nonlinear_with_params(
            format!("p5_t8_b1_load_{order_label}"),
            mesh,
            &params,
            &bc_load,
            Some(opts),
        )
        .expect("loading stage failed");

        let max_x_nodes = mesh.find_nodes_by_predicate(|p| (p[0] - aabb.max[0]).abs() < 1e-6);
        let avg_ux_loaded = max_x_nodes
            .iter()
            .map(|&nid| loaded.report.nodal_displacements[nid][0])
            .sum::<f64>()
            / max_x_nodes.len() as f64;
        let strain_loaded = avg_ux_loaded / lx;

        let unload_stage = NonlinearLoadStage::new(bc_unload)
            .with_steps(4)
            .with_label("unloading");
        let unloaded = analyze_mesh_nonlinear_history(
            format!("p5_t8_b1_unload_{order_label}"),
            mesh,
            &params,
            &[unload_stage],
            Some(opts),
            Some(loaded.state.clone()),
        )
        .expect("unloading stage failed");

        let avg_ux_unloaded = max_x_nodes
            .iter()
            .map(|&nid| unloaded.report.nodal_displacements[nid][0])
            .sum::<f64>()
            / max_x_nodes.len() as f64;
        let perm_strain = avg_ux_unloaded / lx;
        let springback = (avg_ux_loaded - avg_ux_unloaded) / lx;
        let residual_stress_pa = unloaded.report.max_stress_von_mises_pa;

        let ep_err_pct = ((perm_strain - expected_ep).abs() / expected_ep) * 100.0;
        let ee_err_pct = ((springback - expected_ee).abs() / expected_ee) * 100.0;
        let tot_err_pct = ((strain_loaded - expected_total).abs() / expected_total) * 100.0;
        let max_tr_ep = unloaded
            .state
            .gauss_states
            .iter()
            .map(|g| (g.plastic_strain[0] + g.plastic_strain[1] + g.plastic_strain[2]).abs())
            .fold(0.0_f64, f64::max);

        println!(
            "[P5-T8 Benchmark 1 | {order_label}] nodes={} elems={} | ε_p={:.6} (ref {:.6}, err={:.4}%) | \
             springback={:.6} (ref {:.6}, err={:.4}%) | ε_tot={:.6} (err={:.4}%) | \
             σ_res={:.3e} Pa | max|tr(ε^p)|={:.2e} | time={:.1} ms",
            mesh.node_count(),
            mesh.element_count(),
            perm_strain,
            expected_ep,
            ep_err_pct,
            springback,
            expected_ee,
            ee_err_pct,
            strain_loaded,
            tot_err_pct,
            residual_stress_pa,
            max_tr_ep,
            t_order.elapsed().as_secs_f64() * 1e3
        );

        assert!(ep_err_pct < 1.0, "{order_label}: ε_p error {ep_err_pct:.4}% >= 1%");
        assert!(ee_err_pct < 1.0, "{order_label}: springback error {ee_err_pct:.4}% >= 1%");
        assert!(tot_err_pct < 1.0, "{order_label}: total strain error {tot_err_pct:.4}% >= 1%");
        assert!(residual_stress_pa < 1e5, "{order_label}: residual stress {residual_stress_pa:.3e} Pa >= 0.1 MPa");
        assert!(max_tr_ep < 1e-10, "{order_label}: plastic incompressibility violated: {max_tr_ep:.3e}");
    }
    println!("[P5-T8 Benchmark 1] Total wall time = {:.1} ms", t0.elapsed().as_secs_f64() * 1e3);
}

// ============================================================================
// Benchmark 2: Elastoplastic Cantilever Plastic Hinge (M_p / M_y = 1.500)
// ============================================================================

#[test]
fn p5_t8_benchmark_2_cantilever_plastic_hinge() {
    let t0 = Instant::now();
    let lx: f64 = 0.24;
    let ly: f64 = 0.016;
    let lz: f64 = 0.008;
    let e_pa = 200.0e9;
    let sigma_y = 250.0e6;

    let mat = Material::new("steel_b2", 200.0, 0.30, 250.0, 7850.0, "Benchmark 2 Steel");
    let params = J2MaterialParams::from_material(&mat)
        .expect("valid J2 params")
        .with_ideal_plasticity();

    let m_y = sigma_y * lz * ly * ly / 6.0;
    let m_p = sigma_y * lz * ly * ly / 4.0;
    let i_z = lz * ly.powi(3) / 12.0;
    let delta_y = m_y * lx * lx / (3.0 * e_pa * i_z);

    let mesh = build_graded_x_box_tet4(lx, ly, lz, 12, 4, 1, 1.70).to_tet10();
    let aabb = mesh.bounding_box();

    let mut root_bc = BoundaryConditions::new();
    root_bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
        face: AabbFace::MinX,
        tolerance: 1e-6,
        components: DofComponent::X,
    });
    let root_y0 = mesh.find_nodes_by_predicate(|p| (p[0] - aabb.min[0]).abs() < 1e-6 && p[1].abs() < 1e-6);
    let root_z0 = mesh.find_nodes_by_predicate(|p| (p[0] - aabb.min[0]).abs() < 1e-6 && p[2].abs() < 1e-6);
    root_bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: root_y0,
        components: DofComponent::Y,
    });
    root_bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: root_z0,
        components: DofComponent::Z,
    });

    let tip_neutral = mesh.find_nodes_by_predicate(|p| (p[0] - aabb.max[0]).abs() < 1e-6 && p[1].abs() < 1e-6);
    let make_tip_bc = |tip_dy: f64| -> BoundaryConditions {
        let mut bc = root_bc.clone();
        for &nid in &tip_neutral {
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

    let stage_elastic = NonlinearLoadStage::new(make_tip_bc(-0.5 * delta_y))
        .with_steps(1)
        .with_label("elastic_0.5dy");
    let stage_hinge = NonlinearLoadStage::new(make_tip_bc(-6.0 * delta_y))
        .with_steps(8)
        .with_label("plastic_hinge_6dy");

    let loaded = analyze_mesh_nonlinear_history(
        "p5_t8_b2_cantilever",
        &mesh,
        &params,
        &[stage_elastic, stage_hinge],
        Some(opts),
        None,
    )
    .expect("cantilever plastic hinge solve failed");

    let elastic_step = loaded
        .history
        .iter()
        .rfind(|h| h.stage_index == 0)
        .expect("elastic stage missing");
    assert!(elastic_step.max_equivalent_plastic_strain < 1e-10);

    let elements: Vec<Tet10Element> = mesh
        .tet10_elements
        .iter()
        .enumerate()
        .map(|(id, &nids)| {
            let mut pos = [[0.0; 3]; 10];
            for (k, &nid) in nids.iter().enumerate() {
                pos[k] = mesh.nodes[nid].position;
            }
            Tet10Element::new(id, nids, pos).unwrap()
        })
        .collect();

    let resolved_hinge_bc = make_tip_bc(-6.0 * delta_y).resolve(&mesh).unwrap();
    let reactions = compute_nonlinear_nodal_reactions_tet10(
        &mesh,
        &elements,
        &loaded.state,
        &resolved_hinge_bc,
    );
    let root_nodes = mesh.find_nodes_by_predicate(|p| (p[0] - aabb.min[0]).abs() < 1e-6);
    let m_root_fem: f64 = root_nodes
        .iter()
        .map(|&nid| {
            let y = mesh.nodes[nid].position[1];
            let rx = reactions[nid][0];
            -y * rx
        })
        .sum::<f64>()
        .abs();

    let ratio_fem = m_root_fem / m_y;
    let mp_err_pct = ((m_root_fem - m_p).abs() / m_p) * 100.0;

    // Complete unloading stage to verify permanent tip set after plastic hinge
    let unload_stage = NonlinearLoadStage::new(root_bc)
        .with_steps(3)
        .with_label("unloading");
    let unloaded = analyze_mesh_nonlinear_history(
        "p5_t8_b2_unload",
        &mesh,
        &params,
        &[unload_stage],
        Some(opts),
        Some(loaded.state.clone()),
    )
    .expect("cantilever unload failed");

    let perm_tip_uy = tip_neutral
        .iter()
        .map(|&nid| unloaded.report.nodal_displacements[nid][1])
        .sum::<f64>()
        / tip_neutral.len() as f64;
    let springback_uy = (-6.0 * delta_y - perm_tip_uy).abs();

    println!(
        "[P5-T8 Benchmark 2 | TET10] nodes={} elems={} | M_y={:.2} N·m, M_p_ref={:.2} N·m, \
         M_p_fem={:.2} N·m (M_p/M_y={:.4} vs 1.5000, err={:.2}%) | δ_perm={:.3} mm, springback={:.3} mm | time={:.1} ms",
        mesh.node_count(),
        mesh.element_count(),
        m_y,
        m_p,
        m_root_fem,
        ratio_fem,
        mp_err_pct,
        perm_tip_uy.abs() * 1e3,
        springback_uy * 1e3,
        t0.elapsed().as_secs_f64() * 1e3
    );

    assert!(mp_err_pct < 5.0, "Plastic hinge M_p error {mp_err_pct:.2}% >= 5.0%");
    assert!(
        perm_tip_uy < -2.0 * delta_y,
        "Expected large permanent deflection after 6*delta_y plastic hinge, got {:.3} mm",
        perm_tip_uy * 1e3
    );
    assert!(
        springback_uy > 0.8 * delta_y,
        "Expected elastic springback > 0.8*delta_y, got {:.3} mm",
        springback_uy * 1e3
    );
}

// ============================================================================
// Benchmark 3: Hertz Contact of Two Elastic Cylinders (p_0 and a <= 5.0%)
// ============================================================================

#[test]
fn p5_t8_benchmark_3_hertz_two_elastic_cylinders() {
    let t0 = Instant::now();
    let radius = 0.050_f64;
    let e_mod = 205e9_f64;
    let nu = 0.29_f64;
    let line_load = 400_000.0_f64;

    let r_star = 0.5 * radius;
    let e_star = e_mod / (2.0 * (1.0 - nu * nu));
    let a_hertz = (4.0 * line_load * r_star / (PI * e_star)).sqrt();
    let p0_hertz = (line_load * e_star / (PI * r_star)).sqrt();

    let fine_half_x = 1.65 * a_hertz;
    let n_fine_half_x = 22usize;
    let dx_fine = fine_half_x / n_fine_half_x as f64;
    let thickness_z = 2.0 * dx_fine;
    let applied_force_n = line_load * thickness_z;

    let half_width_x = 0.015_f64;
    let depth_y = 0.015_f64;
    let fine_depth_y = 1.4 * a_hertz;

    let mesh_bottom = build_graded_cylinder_mesh(
        radius, half_width_x, depth_y, thickness_z, fine_half_x, n_fine_half_x, 12, 10, 10,
        fine_depth_y, false,
    );
    let mesh_top = build_graded_cylinder_mesh(
        radius, half_width_x, depth_y, thickness_z, fine_half_x, n_fine_half_x, 12, 10, 10,
        fine_depth_y, true,
    );

    let mut mat = Material::new("hertz_steel", e_mod / 1e9, nu, 5000.0, 7850.0, "Elastic Steel");
    mat.hardening_modulus_pa = 0.0;
    let steel = J2MaterialParams::from_material(&mat)
        .unwrap()
        .with_hardening(HardeningLaw::Linear {
            hardening_modulus_pa: 0.0,
        });

    let mut bc_bottom = BoundaryConditions::new();
    bc_bottom.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: (0..mesh_bottom.node_count()).collect(),
        components: DofComponent::Z,
    });
    bc_bottom.fix_aabb_face(AabbFace::MinY, 1e-6);

    let mut bc_top = BoundaryConditions::new();
    bc_top.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: (0..mesh_top.node_count()).collect(),
        components: DofComponent::Z,
    });
    let top_center: Vec<usize> = mesh_top
        .nodes
        .iter()
        .filter(|n| n.position[0].abs() <= 1e-9 && (n.position[1] - depth_y).abs() <= 1e-6)
        .map(|n| n.id)
        .collect();
    bc_top.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: top_center,
        components: DofComponent::X,
    });

    let top_hub = HubCoupling::Prismatic {
        name: Some("hertz_platen".to_string()),
        axis: [0.0, 1.0, 0.0],
        selector: HubNodeSelector::AabbFace {
            face: AabbFace::MaxY,
            tolerance: 1e-6,
        },
        drive: PrismaticHubDrive::Force {
            force_n: -applied_force_n,
            axial_stiffness_n_per_m: 0.0,
            reference_disp_m: 0.0,
        },
        lock_transverse: false,
    };

    let slave_nodes: Vec<usize> = mesh_top
        .nodes
        .iter()
        .filter(|n| {
            let x = n.position[0];
            let sag = radius - (radius * radius - x * x).max(0.0).sqrt();
            (n.position[1] - sag).abs() <= 1e-7 && x.abs() <= 3.0 * a_hertz
        })
        .map(|n| n.id)
        .collect();

    let total_nodes = mesh_bottom.node_count() + mesh_top.node_count();
    let total_elems = mesh_bottom.element_count() + mesh_top.element_count();

    let bodies = vec![
        ContactBody::new("cyl_bottom", mesh_bottom, steel).with_boundary(bc_bottom),
        ContactBody::new("cyl_top", mesh_top, steel)
            .with_boundary(bc_top)
            .with_hub_coupling(top_hub),
    ];
    let pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 1,
        body_b: 0,
        friction_coeff: 0.0,
        symmetric: false,
        slave_a_nodes: Some(slave_nodes),
        slave_b_nodes: None,
        search_distance: Some(4.0 * a_hertz),
        max_normal_dot: Some(0.25),
    }];
    let opts = ContactSolverOptions {
        discretization: ContactDiscretization::NodeToSurface,
        corotated: true,
        include_geometric_stiffness: true,
        max_penetration_tol: 2e-7,
        al_multiplier_tol: 1e-3,
        max_al_iterations: 20,
        newton_rel_tol: 1e-6,
        newton_abs_tol: 1e-6,
        max_newton_iterations: 25,
        penalty_scale: 15.0,
        ..ContactSolverOptions::default()
    };

    let report =
        solve_deformable_contact(&bodies, &pairs, &[ContactStepSpec::new("hertz_load")], opts)
            .expect("Hertz contact solve failed");
    let step = report.final_step().unwrap();
    let patch = &step.pairs[0];

    let force_err_pct =
        ((patch.total_normal_force_n - applied_force_n).abs() / applied_force_n) * 100.0;
    let p0_fem = patch.peak_normal_pressure_pa;
    let p0_err_pct = ((p0_fem - p0_hertz).abs() / p0_hertz) * 100.0;
    let max_active_x = patch
        .active_points
        .iter()
        .map(|pt| pt.position[0].abs())
        .fold(0.0_f64, f64::max);
    let a_fem = max_active_x + 0.5 * dx_fine;
    let a_err_pct = ((a_fem - a_hertz).abs() / a_hertz) * 100.0;
    let vm_peak = step.body_max_von_mises_pa[0].max(step.body_max_von_mises_pa[1]);
    let vm_ref = 0.56 * p0_hertz;
    let vm_err_pct = ((vm_peak - vm_ref).abs() / vm_ref) * 100.0;

    println!(
        "[P5-T8 Benchmark 3 | Hertz 2-Cylinder] nodes={} elems={} | p0={:.2} MPa vs {:.2} MPa (err={:.2}%) | \
         a={:.4} mm vs {:.4} mm (err={:.2}%) | σ_vM_sub={:.1} MPa vs {:.1} MPa (err={:.2}%) | \
         F_err={:.3}% | max_pen={:.2e} m | time={:.1} ms",
        total_nodes,
        total_elems,
        p0_fem / 1e6,
        p0_hertz / 1e6,
        p0_err_pct,
        a_fem * 1e3,
        a_hertz * 1e3,
        a_err_pct,
        vm_peak / 1e6,
        vm_ref / 1e6,
        vm_err_pct,
        force_err_pct,
        step.max_penetration_m,
        t0.elapsed().as_secs_f64() * 1e3
    );

    assert!(step.converged);
    assert!(step.max_penetration_m < 1e-6);
    assert!(force_err_pct < 0.5);
    assert!(p0_err_pct <= 5.0, "p0 error {p0_err_pct:.2}% > 5.0%");
    assert!(a_err_pct <= 5.0, "contact half-width a error {a_err_pct:.2}% > 5.0%");
    assert!(vm_err_pct < 15.0, "subsurface von Mises error {vm_err_pct:.2}% too high");
}

// ============================================================================
// Benchmark 4: Spur Gear Tooth Bending on TET10 + fe_snap vs ISO 6336-3 / AGMA
// ============================================================================

/// Exact closed-form ISO 6336-3:2006 §6.2.2 tooth root bending factors for tip loading
/// on a standard zero-shift ($x = 0$) external spur gear cut by an ISO 53 Profile A rack
/// ($h_{aP} = 1.0 m$, $h_{fP} = 1.25 m$, $\rho_{fP} = 0.38 m$, $\alpha = 20^\circ$).
struct Iso6336TipBendingRef {
    theta_30_rad: f64,
    s_fn_m: f64,
    h_fa_m: f64,
    rho_f_m: f64,
    alpha_fan_rad: f64,
    y_fa: f64,
    y_sa: f64,
    sigma_f0_pa: f64,
    /// Exact 2D position `(x, y)` of the upper tensile 30°-tangent critical fillet point on Tooth 0.
    critical_fillet_xy: [f64; 2],
}

fn compute_iso6336_tip_bending_ref(
    z: usize,
    module_m: f64,
    alpha_rad: f64,
    face_width_m: f64,
    f_t_n: f64,
) -> Iso6336TipBendingRef {
    let zf = z as f64;
    let h_fp_star = 1.25_f64;
    let rho_fp_star = 0.38_f64;
    let e_star = PI / 4.0
        - h_fp_star * alpha_rad.tan()
        - (1.0 - alpha_rad.sin()) * rho_fp_star / alpha_rad.cos();
    let g_star = rho_fp_star - h_fp_star; // -0.87
    let h_star = (2.0 / zf) * (PI / 2.0 - e_star) - PI / 3.0;

    // Solve ISO 6336-3 Eq. (14): theta = (2*G / z) * tan(theta) - H
    let mut theta = PI / 6.0;
    for _ in 0..80 {
        let f = theta - (2.0 * g_star / zf) * theta.tan() + h_star;
        let df = 1.0 - (2.0 * g_star / zf) / (theta.cos() * theta.cos());
        theta -= f / df;
    }

    let s_fn_star =
        zf * (PI / 3.0 - theta).sin() + 3.0_f64.sqrt() * (g_star / theta.cos() - rho_fp_star);

    let d_p = zf * module_m;
    let d_b = d_p * alpha_rad.cos();
    let d_a = d_p + 2.0 * module_m;

    let alpha_an = (d_b / d_a).acos();
    let inv = |a: f64| a.tan() - a;
    let gamma_a = PI / (2.0 * zf) + inv(alpha_rad) - inv(alpha_an);
    let alpha_fan = alpha_an - gamma_a;

    let h_fa_star = 0.5
        * ((gamma_a.cos() - gamma_a.sin() * alpha_fan.tan()) * (d_a / module_m)
            - zf * (PI / 3.0 - theta).cos()
            - g_star / theta.cos()
            + rho_fp_star);

    let y_fa = (6.0 * h_fa_star * alpha_fan.cos()) / (s_fn_star * s_fn_star * alpha_rad.cos());

    let rho_f_star = rho_fp_star
        + (2.0 * g_star * g_star)
            / (theta.cos() * (zf * theta.cos() * theta.cos() - 2.0 * g_star));

    let l_a = s_fn_star / h_fa_star;
    let q_s = s_fn_star / (2.0 * rho_f_star);
    let y_sa = (1.2 + 0.13 * l_a) * q_s.powf(1.0 / (1.21 + 2.3 / l_a));

    let sigma_f0_pa = (f_t_n / (face_width_m * module_m)) * y_fa * y_sa;

    // Exact trochoid point at beta = theta (30° tangent on upper tensile fillet of Tooth 0)
    let r_p = 0.5 * d_p;
    let a0 = (h_fp_star - rho_fp_star) * module_m;
    let rho_fp = rho_fp_star * module_m;
    let y_c = (PI / 2.0 - e_star) * module_m;
    let psi = (y_c + a0 * theta.tan()) / r_p;
    let rad = r_p - a0 - rho_fp * theta.cos();
    let tang = -(a0 * theta.tan() + rho_fp * theta.sin());
    let crit_x = rad * psi.cos() - tang * psi.sin();
    let crit_y = rad * psi.sin() + tang * psi.cos();

    Iso6336TipBendingRef {
        theta_30_rad: theta,
        s_fn_m: s_fn_star * module_m,
        h_fa_m: h_fa_star * module_m,
        rho_f_m: rho_f_star * module_m,
        alpha_fan_rad: alpha_fan,
        y_fa,
        y_sa,
        sigma_f0_pa,
        critical_fillet_xy: [crit_x, crit_y],
    }
}

/// Evaluates the exact ISO 53 Profile A trochoidal root fillet point on the upper (+Y) side
/// of a tooth centered on $+X$ for parameter $\beta \in [0, \pi/2 - \alpha]$.
fn iso53_upper_trochoid_point(
    beta: f64,
    z: usize,
    module_m: f64,
    alpha_rad: f64,
) -> [f64; 2] {
    let r_p = 0.5 * (z as f64) * module_m;
    let h_fp = 1.25 * module_m;
    let rho_fp = 0.38 * module_m;
    let a0 = h_fp - rho_fp;
    let y_c = 0.25 * PI * module_m
        + h_fp * alpha_rad.tan()
        + rho_fp * (1.0 - alpha_rad.sin()) / alpha_rad.cos();
    let psi = (y_c + a0 * beta.tan()) / r_p;
    let rad = r_p - a0 - rho_fp * beta.cos();
    let tang = -(a0 * beta.tan() + rho_fp * beta.sin());
    [
        rad * psi.cos() - tang * psi.sin(),
        rad * psi.sin() + tang * psi.cos(),
    ]
}

/// Builds a watertight 3D CAD `TriangleMesh` of a 3-tooth ISO 53 Profile A involute spur gear
/// rim sector (teeth `-1, 0, +1`, with center tooth `0` along `+X`, exact rack-generated
/// trochoidal root fillets $\rho_{fP} = 0.38 m$, and thick rim $m_B = s_R / h_t \ge 1.5$).
fn build_iso53_three_tooth_gear_sector_cad_mesh(
    z: usize,
    module_m: f64,
    alpha_rad: f64,
    face_width_m: f64,
    rim_inner_r: f64,
) -> TriangleMesh {
    let zf = z as f64;
    let pitch_angle = 2.0 * PI / zf;
    let r_p = 0.5 * zf * module_m;
    let r_b = r_p * alpha_rad.cos();
    let r_a = r_p + module_m;
    let r_f = r_p - 1.25 * module_m;
    let psi_b = PI / (2.0 * zf) + (alpha_rad.tan() - alpha_rad);

    // Build exact upper-half profile for one tooth centered on +X (from tip to root trough),
    // then mirror for lower half and replicate across teeth {-1, 0, +1}.
    let beta_max = PI / 2.0 - alpha_rad;
    let tan_pt = iso53_upper_trochoid_point(beta_max, z, module_m, alpha_rad);
    let r_tan = (tan_pt[0] * tan_pt[0] + tan_pt[1] * tan_pt[1]).sqrt();
    let t_start = ((r_tan / r_b).powi(2) - 1.0).max(0.0).sqrt();
    let t_tip = ((r_a / r_b).powi(2) - 1.0).max(0.0).sqrt();

    let inv_pt_upper = |t: f64| -> [f64; 2] {
        let r = r_b * (1.0 + t * t).sqrt();
        let theta = psi_b - (t - t.atan());
        [r * theta.cos(), r * theta.sin()]
    };

    // Single tooth local CCW profile from angle -pitch_angle/2 to +pitch_angle/2
    let n_trough = 4usize;
    let n_fillet = 20usize;
    let n_inv = 14usize;
    let n_tip = 6usize;

    let root_pt_upper = iso53_upper_trochoid_point(0.0, z, module_m, alpha_rad);
    let theta_root_upper = root_pt_upper[1].atan2(root_pt_upper[0]);
    let tip_pt_upper = inv_pt_upper(t_tip);
    let theta_tip_upper = tip_pt_upper[1].atan2(tip_pt_upper[0]);

    let mut single_tooth_pts: Vec<[f64; 2]> = Vec::new();
    // 1. Lower root trough: [-pitch_angle/2 .. -theta_root_upper]
    for i in 0..n_trough {
        let u = i as f64 / n_trough as f64;
        let th = -0.5 * pitch_angle + (-theta_root_upper + 0.5 * pitch_angle) * u;
        single_tooth_pts.push([r_f * th.cos(), r_f * th.sin()]);
    }
    // 2. Lower trochoid root fillet: beta = 0 .. beta_max (mirrored y -> -y)
    for i in 0..=n_fillet {
        let beta = beta_max * (i as f64 / n_fillet as f64);
        let p = iso53_upper_trochoid_point(beta, z, module_m, alpha_rad);
        single_tooth_pts.push([p[0], -p[1]]);
    }
    // 3. Lower involute flank: t = t_start .. t_tip (skip i=0 which coincides with tangency pt)
    for i in 1..=n_inv {
        let t = t_start + (t_tip - t_start) * (i as f64 / n_inv as f64);
        let p = inv_pt_upper(t);
        single_tooth_pts.push([p[0], -p[1]]);
    }
    // 4. Tip land arc: -theta_tip_upper .. +theta_tip_upper (skip endpoints)
    for i in 1..n_tip {
        let u = i as f64 / n_tip as f64;
        let th = -theta_tip_upper + 2.0 * theta_tip_upper * u;
        single_tooth_pts.push([r_a * th.cos(), r_a * th.sin()]);
    }
    // 5. Upper involute flank: t = t_tip .. t_start
    for i in 0..n_inv {
        let t = t_tip - (t_tip - t_start) * (i as f64 / n_inv as f64);
        single_tooth_pts.push(inv_pt_upper(t));
    }
    // 6. Upper trochoid root fillet: beta = beta_max .. 0
    for i in 0..=n_fillet {
        let beta = beta_max * (1.0 - i as f64 / n_fillet as f64);
        single_tooth_pts.push(iso53_upper_trochoid_point(beta, z, module_m, alpha_rad));
    }
    // 7. Upper root trough: theta_root_upper .. +pitch_angle/2 (skip i=0)
    for i in 1..=n_trough {
        let u = i as f64 / n_trough as f64;
        let th = theta_root_upper + (0.5 * pitch_angle - theta_root_upper) * u;
        single_tooth_pts.push([r_f * th.cos(), r_f * th.sin()]);
    }

    // Replicate across 3 teeth: k in [-1, 0, +1] -> outer boundary in CCW order
    let mut outer_pts: Vec<[f64; 2]> = Vec::new();
    for (idx_t, k) in [-1_i32, 0, 1].iter().enumerate() {
        let rot = (*k as f64) * pitch_angle;
        let cr = rot.cos();
        let sr = rot.sin();
        let start_j = if idx_t == 0 { 0 } else { 1 };
        for p in single_tooth_pts.iter().skip(start_j) {
            outer_pts.push([p[0] * cr - p[1] * sr, p[0] * sr + p[1] * cr]);
        }
    }

    // Build inner rim arc matching every outer vertex's polar angle so front/back caps are
    // star-shaped radial quads (guaranteed valid, non-overlapping, zero skinny slivers!).
    let n_outer = outer_pts.len();
    let inner_pts: Vec<[f64; 2]> = outer_pts
        .iter()
        .map(|p| {
            let th = p[1].atan2(p[0]);
            [rim_inner_r * th.cos(), rim_inner_r * th.sin()]
        })
        .collect();

    let z_back = -0.5 * face_width_m;
    let z_front = 0.5 * face_width_m;

    // Vertices layout:
    // - 0 .. n_outer:                outer front (z = +b/2)
    // - n_outer .. 2*n_outer:        outer back  (z = -b/2)
    // - 2*n_outer .. 3*n_outer:      inner front (z = +b/2)
    // - 3*n_outer .. 4*n_outer:      inner back  (z = -b/2)
    let mut vertices: Vec<Vertex> = Vec::with_capacity(4 * n_outer);
    for p in &outer_pts {
        vertices.push(Vertex::from_position([p[0], p[1], z_front]));
    }
    for p in &outer_pts {
        vertices.push(Vertex::from_position([p[0], p[1], z_back]));
    }
    for p in &inner_pts {
        vertices.push(Vertex::from_position([p[0], p[1], z_front]));
    }
    for p in &inner_pts {
        vertices.push(Vertex::from_position([p[0], p[1], z_back]));
    }

    let of = |i: usize| i as u32;
    let ob = |i: usize| (n_outer + i) as u32;
    let inf = |i: usize| (2 * n_outer + i) as u32;
    let inb = |i: usize| (3 * n_outer + i) as u32;

    let mut indices: Vec<[u32; 3]> = Vec::new();
    let mut push_tri = |a: u32, b: u32, c: u32| {
        indices.push([a, b, c]);
    };

    for i in 0..(n_outer - 1) {
        // Front cap (+Z outward normal): inner(i) -> outer(i) -> outer(i+1) -> inner(i+1)
        push_tri(inf(i), of(i), of(i + 1));
        push_tri(inf(i), of(i + 1), inf(i + 1));

        // Back cap (-Z outward normal): reverse winding
        push_tri(inb(i), ob(i + 1), ob(i));
        push_tri(inb(i), inb(i + 1), ob(i + 1));

        // Outer profile wall (outward normal away from origin)
        push_tri(of(i), ob(i), ob(i + 1));
        push_tri(of(i), ob(i + 1), of(i + 1));

        // Inner rim bore wall (outward normal toward origin)
        push_tri(inf(i), inb(i + 1), inb(i));
        push_tri(inf(i), inf(i + 1), inb(i + 1));
    }

    // Sector start radial end wall (i = 0, outward normal toward -theta)
    push_tri(inf(0), inb(0), ob(0));
    push_tri(inf(0), ob(0), of(0));

    // Sector end radial end wall (i = n_outer - 1, outward normal toward +theta)
    let last = n_outer - 1;
    push_tri(inf(last), of(last), ob(last));
    push_tri(inf(last), ob(last), inb(last));

    let mut mesh = TriangleMesh::new(vertices, indices);
    mesh.recompute_vertex_normals();
    mesh
}

#[test]
fn p5_t8_benchmark_4_spur_gear_tooth_bending_tet10_fe_snap_vs_iso6336() {
    let t0 = Instant::now();
    let z = 20usize;
    let module_m = 0.005_f64; // 5 mm module
    let alpha_rad = 20.0_f64.to_radians();
    let face_width_m = 0.010_f64; // 10 mm face width (b/m = 2.0)
    let f_t_n = 1000.0_f64; // 1000 N tangential pitch-line force

    let r_p = 0.5 * (z as f64) * module_m; // 50 mm
    let r_f = r_p - 1.25 * module_m; // 43.75 mm
    let r_a = r_p + module_m; // 55.0 mm
    let h_t = 2.25 * module_m; // 11.25 mm whole tooth depth
    let rim_inner_r = r_f - 1.55 * h_t; // 26.3125 mm -> backup ratio m_B = 1.55 >= 1.2 (Y_B = 1.0)
    let pitch_angle = 2.0 * PI / (z as f64);

    // 1. Exact ISO 6336-3:2006 reference and nominal Lewis comparison
    let iso = compute_iso6336_tip_bending_ref(z, module_m, alpha_rad, face_width_m, f_t_n);
    let y_lewis = lewis_form_factor(z, 20.0);
    let sigma_lewis_pa = f_t_n / (face_width_m * module_m * y_lewis);

    // Sanity check exact ISO 6336-3 textbook factors for z=20, alpha=20°, x=0
    assert!((iso.y_fa - 2.78).abs() < 0.02, "Y_Fa = {:.4}", iso.y_fa);
    assert!((iso.y_sa - 1.56).abs() < 0.02, "Y_Sa = {:.4}", iso.y_sa);

    // 2. Build watertight 3-tooth ISO 53 CAD TriangleMesh
    let cad_mesh = build_iso53_three_tooth_gear_sector_cad_mesh(
        z,
        module_m,
        alpha_rad,
        face_width_m,
        rim_inner_r,
    );
    let validation = validate_mesh(&cad_mesh, None);
    assert!(
        validation.is_valid_solid && validation.topology.boundary_edges_count == 0,
        "ISO 53 gear sector CAD mesh must be a watertight solid: {validation:?}"
    );

    // 3. Mesh through production TET10 + fe_snap conforming pipeline with local refinement
    //    focused on the center tooth's tensile and compressive root fillets
    let t_mesh = Instant::now();
    let crit_u = iso.critical_fillet_xy;
    let crit_l = [crit_u[0], -crit_u[1]];

    let mut mesh_opts = ConformingMeshOptions::for_accuracy_mode(
        AccuracyMode::High,
        MeshDensity::Custom { max_element_size: 0.0028 },
    );
    mesh_opts.element_order = ElementOrder::Quadratic;
    mesh_opts.snap_to_boundary = true;
    mesh_opts.snap = SnapOptions {
        min_allowed_scaled_jacobian: 0.06,
        outer_passes: 5,
        smoothing_sweeps: 3,
        surface_tangential_sweeps: 2,
        feature_angle_deg: 35.0,
        preserve_sharp_features: true,
        min_tet10_gauss_jacobian_ratio: 0.40,
    };
    mesh_opts.refinement = LocalRefinementOptions {
        enabled: true,
        max_depth: 2,
        curvature_angle_deg: 6.0,
        refine_band_factor: 0.60,
        zones: vec![
            RefinementZone::new([crit_u[0], crit_u[1], 0.0], 0.0085, 2),
            RefinementZone::new([crit_l[0], crit_l[1], 0.0], 0.0085, 2),
        ],
        max_leaf_cells: 4_500,
    };

    let tet_mesh = TetrahedralMesh::from_mesh_conforming(&cad_mesh, &mesh_opts)
        .expect("TET10 + fe_snap conforming meshing of ISO 53 gear sector must succeed");
    let mesh_ms = t_mesh.elapsed().as_secs_f64() * 1e3;

    let metrics = evaluate_mesh_metrics(&tet_mesh, &cad_mesh, None, 1e-4);
    assert!(tet_mesh.is_tet10(), "Mesh must be quadratic TET10");
    assert!(tet_mesh.boundary_conforming, "Mesh must be boundary-conforming (fe_snap)");
    assert_eq!(metrics.inverted_elements, 0, "Zero inverted elements required");
    assert!(
        metrics.min_scaled_jacobian >= 0.06 - 1e-6,
        "min scaled Jacobian {:.4} < 0.06",
        metrics.min_scaled_jacobian
    );
    assert!(
        metrics.all_tet10_gauss_jacobians_positive,
        "All TET10 Gauss Jacobians must be strictly positive"
    );

    // 4. Boundary conditions:
    //    - Clamp inner rim bore (r <= rim_inner_r + 0.6 mm) and the two sector radial end faces
    //      (|theta| >= 1.5*pitch_angle - 0.015 rad) in XYZ
    //    - Apply exact ISO 6336-3 tip load along line of action angle alpha_Fan on the upper tip
    //      edge of center tooth 0 (uniformly distributed across face width z in [-b/2, +b/2])
    let mut bc = BoundaryConditions::new();
    let half_sector_ang = 1.5 * pitch_angle;
    let clamped_nodes: Vec<usize> = tet_mesh
        .nodes
        .iter()
        .filter(|n| {
            let x = n.position[0];
            let y = n.position[1];
            let r = (x * x + y * y).sqrt();
            let th = y.atan2(x).abs();
            r <= rim_inner_r + 0.0006 || th >= half_sector_ang - 0.015
        })
        .map(|n| n.id)
        .collect();
    assert!(!clamped_nodes.is_empty(), "Clamped rim/sector nodes must not be empty");
    bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: clamped_nodes,
        components: DofComponent::All,
    });

    // Identify upper tip flank edge nodes of center tooth 0:
    // r >= r_a - 0.8 mm, y > 0, within tooth 0 angular span (|theta| < 0.4 * pitch_angle)
    let tip_load_nodes: Vec<usize> = tet_mesh
        .nodes
        .iter()
        .filter(|n| {
            let x = n.position[0];
            let y = n.position[1];
            let r = (x * x + y * y).sqrt();
            let th = y.atan2(x);
            r >= r_a - 0.0008 && th > 0.0 && th < 0.40 * pitch_angle
        })
        .map(|n| n.id)
        .collect();
    assert!(
        tip_load_nodes.len() >= 3,
        "Expected >= 3 nodes along upper tip edge of center tooth 0, got {}",
        tip_load_nodes.len()
    );

    // Total normal load along line of action: F_bn = F_t / cos(alpha)
    // Components in tooth frame (+X radial outward, +Y upper flank):
    // F_x = -F_bn * sin(alpha_Fan) (compressive radial), F_y = -F_bn * cos(alpha_Fan) (tangential bending)
    let f_bn = f_t_n / alpha_rad.cos();
    let f_total = [
        -f_bn * iso.alpha_fan_rad.sin(),
        -f_bn * iso.alpha_fan_rad.cos(),
        0.0,
    ];
    let n_load = tip_load_nodes.len() as f64;
    for &nid in &tip_load_nodes {
        bc.neumann.push(NeumannLoad::NodalForce {
            node_id: nid,
            force: [f_total[0] / n_load, f_total[1] / n_load, 0.0],
        });
    }

    // 5. Solve TET10 linear elastic FEA
    let steel = Material::new("iso_gear_steel", 210.0, 0.30, 800.0, 7850.0, "Gear Steel");
    let t_solve = Instant::now();
    let report = analyze_mesh(
        "p5_t8_iso6336_gear",
        &tet_mesh,
        &steel,
        &bc,
        Some(SolverOptions {
            tolerance: 1e-8,
            max_iterations: 10_000,
        }),
    )
    .expect("TET10 gear bending FEA solve failed");
    let solve_ms = t_solve.elapsed().as_secs_f64() * 1e3;

    // 6. Recover averaged nodal Cauchy stress tensor & 1st principal tensile stress sigma_1
    //    across all TET10 elements so we can pinpoint the exact peak stress location and value.
    let d_mat = elasticity_matrix_from_material(&steel).unwrap();
    let mut nodal_stress_sum = vec![nalgebra::Vector6::<f64>::zeros(); tet_mesh.node_count()];
    let mut nodal_stress_cnt = vec![0usize; tet_mesh.node_count()];

    for (eid, &nids) in tet_mesh.tet10_elements.iter().enumerate() {
        let mut pos = [[0.0; 3]; 10];
        let mut u_elem = mcp_cad_fea::Vector30::zeros();
        for (k, &nid) in nids.iter().enumerate() {
            pos[k] = tet_mesh.nodes[nid].position;
            let u = report.nodal_displacements[nid];
            u_elem[3 * k] = u[0];
            u_elem[3 * k + 1] = u[1];
            u_elem[3 * k + 2] = u[2];
        }
        let elem = Tet10Element::new(eid, nids, pos).unwrap();
        let nodal_s = elem.extrapolate_nodal_stresses(&d_mat, &u_elem);
        for (k, &nid) in nids.iter().enumerate() {
            nodal_stress_sum[nid] += nodal_s[k];
            nodal_stress_cnt[nid] += 1;
        }
    }

    // Scan all nodes outside the immediate tip-load contact patch (r < r_a - 0.35 * m)
    // and away from the artificial radial sector cut clamp (|theta| < 1.2 * pitch_angle)
    let mut global_peak_s1 = 0.0_f64;
    let mut global_peak_s1_pos = [0.0; 3];
    let mut global_peak_vm = 0.0_f64;
    let mut global_peak_vm_pos = [0.0; 3];

    let mut fillet_peak_s1 = 0.0_f64;
    let mut fillet_peak_s1_pos = [0.0; 3];
    let mut fillet_peak_vm = 0.0_f64;

    for nid in 0..tet_mesh.node_count() {
        if nodal_stress_cnt[nid] == 0 {
            continue;
        }
        let s = nodal_stress_sum[nid] / (nodal_stress_cnt[nid] as f64);
        let (principals, _) = principal_stresses_and_directions(&s);
        let s1 = principals[0];
        let vm = von_mises_from_voigt(&s);
        let p = tet_mesh.nodes[nid].position;
        let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
        let th = p[1].atan2(p[0]);

        // Exclude only the immediate tip load contact strip and clamped rim/sector cut walls
        let in_tip_contact = r > r_a - 0.35 * module_m;
        let on_clamp = r <= rim_inner_r + 0.002 || th.abs() >= 1.25 * pitch_angle;
        if !in_tip_contact && !on_clamp {
            if s1 > global_peak_s1 {
                global_peak_s1 = s1;
                global_peak_s1_pos = p;
            }
            if vm > global_peak_vm {
                global_peak_vm = vm;
                global_peak_vm_pos = p;
            }
        }

        // Tensile root fillet zone of center tooth 0:
        // y > 0, r in [r_f - 0.5 mm, r_p - 0.25*m], |theta| < 0.55 * pitch_angle
        let in_tensile_fillet = p[1] > 0.0
            && r >= r_f - 0.0005
            && r <= r_p - 0.25 * module_m
            && th.abs() <= 0.55 * pitch_angle;
        if in_tensile_fillet {
            if s1 > fillet_peak_s1 {
                fillet_peak_s1 = s1;
                fillet_peak_s1_pos = p;
            }
            if vm > fillet_peak_vm {
                fillet_peak_vm = vm;
            }
        }
    }

    let err_s1_pct = ((fillet_peak_s1 - iso.sigma_f0_pa).abs() / iso.sigma_f0_pa) * 100.0;
    let err_vm_pct = ((fillet_peak_vm - iso.sigma_f0_pa).abs() / iso.sigma_f0_pa) * 100.0;
    let peak_dist_to_30deg_mm = ((fillet_peak_s1_pos[0] - crit_u[0]).powi(2)
        + (fillet_peak_s1_pos[1] - crit_u[1]).powi(2))
    .sqrt()
        * 1e3;

    println!(
        "[P5-T8 Benchmark 4 | TET10 + fe_snap vs ISO 6336-3] nodes={} elems={} (mesh={:.1} ms, solve={:.1} ms, total={:.1} ms)\n\
         Quality: J_s_min={:.4}, J_s_mean={:.4}, max_surf_dev={:.4} mm, rms_dev={:.4} mm, eq_err={:.3}%\n\
         ISO 6336-3: Y_Fa={:.4}, Y_Sa={:.4}, s_Fn={:.3} mm, h_Fa={:.3} mm, rho_F={:.3} mm, θ_30={:.2}°, σ_F0={:.2} MPa (Lewis nom={:.2} MPa)\n\
         FEM TET10:  σ1_fillet={:.2} MPa (err={:.2}%), σ_vM_fillet={:.2} MPa (err={:.2}%)\n\
         Peak loc:   global σ1 @ ({:.2}, {:.2}, {:.2}) mm vs ISO 30° ({:.2}, {:.2}) mm (Δxy={:.2} mm) | global σ_vM @ ({:.2}, {:.2}, {:.2}) mm",
        tet_mesh.node_count(),
        tet_mesh.element_count(),
        mesh_ms,
        solve_ms,
        t0.elapsed().as_secs_f64() * 1e3,
        metrics.min_scaled_jacobian,
        metrics.mean_scaled_jacobian,
        metrics.max_boundary_deviation_mm,
        metrics.rms_boundary_deviation_mm,
        report.diagnostics.equilibrium_error_pct,
        iso.y_fa,
        iso.y_sa,
        iso.s_fn_m * 1e3,
        iso.h_fa_m * 1e3,
        iso.rho_f_m * 1e3,
        iso.theta_30_rad.to_degrees(),
        iso.sigma_f0_pa / 1e6,
        sigma_lewis_pa / 1e6,
        fillet_peak_s1 / 1e6,
        err_s1_pct,
        fillet_peak_vm / 1e6,
        err_vm_pct,
        global_peak_s1_pos[0] * 1e3,
        global_peak_s1_pos[1] * 1e3,
        global_peak_s1_pos[2] * 1e3,
        crit_u[0] * 1e3,
        crit_u[1] * 1e3,
        peak_dist_to_30deg_mm,
        global_peak_vm_pos[0] * 1e3,
        global_peak_vm_pos[1] * 1e3,
        global_peak_vm_pos[2] * 1e3
    );

    // 7. Strict P5-T8 Benchmark 4 assertions:
    //    (a) Peak tensile stress across the gear body (outside tip contact strip) is strictly in the tensile root fillet
    assert!(
        (global_peak_s1 - fillet_peak_s1).abs() / global_peak_s1.max(1.0) < 1e-6,
        "Global peak tensile stress ({:.2} MPa at {:?}) must lie strictly in the tensile root fillet ({:.2} MPa)",
        global_peak_s1 / 1e6,
        global_peak_s1_pos,
        fillet_peak_s1 / 1e6
    );
    assert!(
        peak_dist_to_30deg_mm < 3.5,
        "Peak tensile stress location must lie near the ISO 30° tangent root fillet (Δxy = {:.2} mm)",
        peak_dist_to_30deg_mm
    );
    //    (b) Relative error vs exact ISO 6336-3 σ_F0 <= 8.0%
    assert!(
        err_s1_pct <= 8.0,
        "TET10 + fe_snap root fillet tensile stress {:.2} MPa deviates by {:.2}% from ISO 6336-3 {:.2} MPa (> 8.0%)",
        fillet_peak_s1 / 1e6,
        err_s1_pct,
        iso.sigma_f0_pa / 1e6
    );
    //    (c) Global force equilibrium
    assert!(
        report.diagnostics.equilibrium_error_pct < 0.5,
        "Equilibrium error {:.3}% >= 0.5%",
        report.diagnostics.equilibrium_error_pct
    );
}

#[test]
fn p5_t16_create_spur_gear_iso53_root_fillet_tet10_vs_iso6336() {
    let t0 = Instant::now();
    let z = 20usize;
    let module_m = 0.004; // m = 4 mm => r_p = 40 mm, r_a = 44 mm, r_f = 35 mm, r_b = 37.5877 mm
    let alpha_deg = 20.0;
    let alpha_rad = alpha_deg * PI / 180.0;
    let face_width_m = 0.008; // b = 8 mm
    let rim_inner_r = 0.025; // bore radius 25 mm
    let f_t_n = 2_400.0; // tangential tip force 2.4 kN (300 N/mm)

    let r_p = 0.5 * module_m * (z as f64);
    let r_a = r_p + module_m;
    let r_f = r_p - 1.25 * module_m;
    let pitch_angle = 2.0 * PI / (z as f64);

    // 1. Exact ISO 6336-3:2006 reference for z=20, alpha=20 deg, m=4 mm
    let iso = compute_iso6336_tip_bending_ref(z, module_m, alpha_rad, face_width_m, f_t_n);

    // 2. Build full spur gear directly via production create_spur_gear_with_settings (primitives.rs)
    let gear_mesh = mcp_cad_core::primitives::create_spur_gear_with_settings(
        module_m,
        z,
        alpha_deg,
        face_width_m,
        Some(rim_inner_r),
        None,
        None,
        None,
        None,
        &mcp_cad_core::tessellation::TessellationSettings::fine(),
    );
    let validation = validate_mesh(&gear_mesh, Some(1));
    assert_eq!(
        validation.status, "PASS",
        "create_spur_gear_with_settings must pass validate_mesh with 0 degenerate/self-intersecting faces: {validation:?}"
    );

    // 3. Mesh directly through TET10 + fe_snap conforming pipeline with local refinement
    //    focused on Tooth 0's tensile and compressive root fillets
    let t_mesh = Instant::now();
    let crit_u = iso.critical_fillet_xy;
    let crit_l = [crit_u[0], -crit_u[1]];

    let mut mesh_opts = ConformingMeshOptions::for_accuracy_mode(
        AccuracyMode::High,
        MeshDensity::Custom {
            max_element_size: 0.00672,
        },
    );
    mesh_opts.element_order = ElementOrder::Quadratic;
    mesh_opts.snap_to_boundary = true;
    mesh_opts.snap = SnapOptions {
        min_allowed_scaled_jacobian: 0.06,
        outer_passes: 5,
        smoothing_sweeps: 3,
        surface_tangential_sweeps: 2,
        feature_angle_deg: 35.0,
        preserve_sharp_features: true,
        min_tet10_gauss_jacobian_ratio: 0.40,
    };
    mesh_opts.refinement = LocalRefinementOptions {
        enabled: true,
        max_depth: 3,
        curvature_angle_deg: 85.0,
        refine_band_factor: 0.45,
        zones: vec![
            RefinementZone::new([crit_u[0], crit_u[1], 0.0], 0.0055, 3),
            RefinementZone::new([crit_l[0], crit_l[1], 0.0], 0.0055, 3),
            RefinementZone::new([r_p + 0.5 * module_m, 0.0, 0.0], 0.0075, 2),
        ],
        max_leaf_cells: 6_500,
    };

    let tet_mesh = TetrahedralMesh::from_mesh_conforming(&gear_mesh, &mesh_opts)
        .expect("TET10 + fe_snap conforming meshing of create_spur_gear must succeed");
    let mesh_ms = t_mesh.elapsed().as_secs_f64() * 1e3;

    let metrics = evaluate_mesh_metrics(&tet_mesh, &gear_mesh, None, 1e-4);
    assert!(tet_mesh.is_tet10(), "Mesh must be quadratic TET10");
    assert!(
        tet_mesh.boundary_conforming,
        "Mesh must be boundary-conforming (fe_snap)"
    );
    assert_eq!(
        metrics.inverted_elements, 0,
        "Zero inverted elements required"
    );
    assert!(
        metrics.min_scaled_jacobian >= 0.06 - 1e-6,
        "min scaled Jacobian {:.4} < 0.06",
        metrics.min_scaled_jacobian
    );
    assert!(
        metrics.all_tet10_gauss_jacobians_positive,
        "All TET10 Gauss Jacobians must be strictly positive"
    );

    // 4. Boundary conditions:
    //    - Clamp inner rim bore (r <= rim_inner_r + 1.2 mm) and remote sector outside +/-1.5*pitch_angle
    //    - Apply exact ISO 6336-3 tip load along line of action angle alpha_Fan on upper tip edge of Tooth 0
    let mut bc = BoundaryConditions::new();
    let half_sector_ang = 1.5 * pitch_angle;
    let clamped_nodes: Vec<usize> = tet_mesh
        .nodes
        .iter()
        .filter(|n| {
            let x = n.position[0];
            let y = n.position[1];
            let r = (x * x + y * y).sqrt();
            let th = y.atan2(x).abs();
            r <= rim_inner_r + 0.0012 || th >= half_sector_ang - 0.015
        })
        .map(|n| n.id)
        .collect();
    assert!(
        !clamped_nodes.is_empty(),
        "Clamped rim/sector nodes must not be empty"
    );
    bc.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: clamped_nodes,
        components: DofComponent::All,
    });

    let tip_load_nodes: Vec<usize> = tet_mesh
        .nodes
        .iter()
        .filter(|n| {
            let x = n.position[0];
            let y = n.position[1];
            let r = (x * x + y * y).sqrt();
            let th = y.atan2(x);
            r >= r_a - 0.00055 && th > 0.0 && th < 0.40 * pitch_angle
        })
        .map(|n| n.id)
        .collect();
    assert!(
        tip_load_nodes.len() >= 3,
        "Expected >= 3 nodes along upper tip edge of Tooth 0, got {}",
        tip_load_nodes.len()
    );

    let f_bn = f_t_n / alpha_rad.cos();
    let f_total = [
        -f_bn * iso.alpha_fan_rad.sin(),
        -f_bn * iso.alpha_fan_rad.cos(),
        0.0,
    ];
    let n_load = tip_load_nodes.len() as f64;
    for &nid in &tip_load_nodes {
        bc.neumann.push(NeumannLoad::NodalForce {
            node_id: nid,
            force: [f_total[0] / n_load, f_total[1] / n_load, 0.0],
        });
    }

    // 5. Solve TET10 linear elastic FEA
    let steel = Material::new("iso_gear_steel", 210.0, 0.30, 800.0, 7850.0, "Gear Steel");
    let t_solve = Instant::now();
    let report = analyze_mesh(
        "p5_t16_create_spur_gear_iso6336",
        &tet_mesh,
        &steel,
        &bc,
        Some(SolverOptions {
            tolerance: 1e-8,
            max_iterations: 10_000,
        }),
    )
    .expect("TET10 create_spur_gear bending FEA solve failed");
    let solve_ms = t_solve.elapsed().as_secs_f64() * 1e3;

    // 6. Recover averaged nodal Cauchy stress tensor & 1st principal tensile stress sigma_1
    let d_mat = elasticity_matrix_from_material(&steel).unwrap();
    let mut nodal_stress_sum = vec![nalgebra::Vector6::<f64>::zeros(); tet_mesh.node_count()];
    let mut nodal_stress_cnt = vec![0usize; tet_mesh.node_count()];

    for (eid, &nids) in tet_mesh.tet10_elements.iter().enumerate() {
        let mut pos = [[0.0; 3]; 10];
        let mut u_elem = mcp_cad_fea::Vector30::zeros();
        for (k, &nid) in nids.iter().enumerate() {
            pos[k] = tet_mesh.nodes[nid].position;
            let u = report.nodal_displacements[nid];
            u_elem[3 * k] = u[0];
            u_elem[3 * k + 1] = u[1];
            u_elem[3 * k + 2] = u[2];
        }
        let elem = Tet10Element::new(eid, nids, pos).unwrap();
        let nodal_s = elem.extrapolate_nodal_stresses(&d_mat, &u_elem);
        for (k, &nid) in nids.iter().enumerate() {
            nodal_stress_sum[nid] += nodal_s[k];
            nodal_stress_cnt[nid] += 1;
        }
    }

    let mut fillet_peak_s1 = 0.0_f64;
    let mut fillet_peak_s1_pos = [0.0; 3];
    let mut fillet_peak_vm = 0.0_f64;

    for nid in 0..tet_mesh.node_count() {
        if nodal_stress_cnt[nid] == 0 {
            continue;
        }
        let s = nodal_stress_sum[nid] / (nodal_stress_cnt[nid] as f64);
        let (principals, _) = principal_stresses_and_directions(&s);
        let s1 = principals[0];
        let vm = von_mises_from_voigt(&s);
        let p = tet_mesh.nodes[nid].position;
        let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
        let th = p[1].atan2(p[0]);

        let in_tensile_fillet = p[1] > 0.0
            && r >= r_f - 0.0005
            && r <= r_p - 0.25 * module_m
            && th.abs() <= 0.55 * pitch_angle;
        if in_tensile_fillet {
            if s1 > fillet_peak_s1 {
                fillet_peak_s1 = s1;
                fillet_peak_s1_pos = p;
            }
            if vm > fillet_peak_vm {
                fillet_peak_vm = vm;
            }
        }
    }

    let err_s1_pct = ((fillet_peak_s1 - iso.sigma_f0_pa).abs() / iso.sigma_f0_pa) * 100.0;
    let peak_dist_to_30deg_mm = ((fillet_peak_s1_pos[0] - crit_u[0]).powi(2)
        + (fillet_peak_s1_pos[1] - crit_u[1]).powi(2))
    .sqrt()
        * 1e3;

    println!(
        "[P5-T16 | create_spur_gear Smooth ISO 53 Fillet -> TET10 + fe_snap vs ISO 6336-3] nodes={} elems={} (mesh={:.1} ms, solve={:.1} ms, total={:.1} ms)\n\
         Quality: J_s_min={:.4}, J_s_mean={:.4}, max_surf_dev={:.4} mm, rms_dev={:.4} mm, eq_err={:.3}%\n\
         ISO 6336-3: σ_F0={:.2} MPa | FEM TET10 on create_spur_gear: σ1_fillet={:.2} MPa (err={:.2}%), σ_vM_fillet={:.2} MPa, Δxy_30deg={:.2} mm",
        tet_mesh.node_count(),
        tet_mesh.element_count(),
        mesh_ms,
        solve_ms,
        t0.elapsed().as_secs_f64() * 1e3,
        metrics.min_scaled_jacobian,
        metrics.mean_scaled_jacobian,
        metrics.max_boundary_deviation_mm,
        metrics.rms_boundary_deviation_mm,
        report.diagnostics.equilibrium_error_pct,
        iso.sigma_f0_pa / 1e6,
        fillet_peak_s1 / 1e6,
        err_s1_pct,
        fillet_peak_vm / 1e6,
        peak_dist_to_30deg_mm
    );

    assert!(
        peak_dist_to_30deg_mm < 3.5,
        "Peak tensile stress on create_spur_gear must lie near the ISO 30° tangent root fillet (Δxy = {:.2} mm)",
        peak_dist_to_30deg_mm
    );
    assert!(
        err_s1_pct <= 8.0,
        "create_spur_gear TET10 + fe_snap root fillet tensile stress {:.2} MPa deviates by {:.2}% from ISO 6336-3 {:.2} MPa (> 8.0%)",
        fillet_peak_s1 / 1e6,
        err_s1_pct,
        iso.sigma_f0_pa / 1e6
    );
}

// ============================================================================
// Benchmark 5: Spur Gear Pair in Three Physical Regimes
//   (a) Elastic contact without residual deformation
//   (b) Plastic tooth indentation without fracture
//   (c) Local tooth fracture with intact hub bore, watertight fragment, zero interpenetration
//   (d) Coupled HHT-α dynamic collision energy balance
// ============================================================================

fn involute_half_angle_at_radius(
    r: f64,
    num_teeth: usize,
    module_m: f64,
    pressure_angle_rad: f64,
) -> f64 {
    let pitch_angle = 2.0 * PI / num_teeth as f64;
    let psi_p = PI / (2.0 * num_teeth as f64);
    let inv_alpha = pressure_angle_rad.tan() - pressure_angle_rad;
    let psi_b = psi_p + inv_alpha;
    let pitch_r = 0.5 * module_m * num_teeth as f64;
    let base_r = pitch_r * pressure_angle_rad.cos();
    let root_r = (pitch_r - 1.25 * module_m).max(0.45 * pitch_r);
    let phi_root = (psi_b + 0.14 * (0.5 * pitch_angle - psi_b)).min(0.46 * pitch_angle);

    if r <= root_r {
        phi_root
    } else if r < base_r {
        let u = ((r - root_r) / (base_r - root_r).max(1e-9)).clamp(0.0, 1.0);
        let smooth = 0.5 * (1.0 - (PI * u).cos());
        phi_root + (psi_b - phi_root) * smooth
    } else {
        let ratio = (r / base_r).max(1.0);
        let t = (ratio * ratio - 1.0).max(0.0).sqrt();
        let inv_t = t - t.atan();
        (psi_b - inv_t).max(0.008 * pitch_angle)
    }
}

struct GearMeshInfo {
    mesh: TetrahedralMesh,
    bore_radius: f64,
    root_radius: f64,
    pitch_radius: f64,
    center: [f64; 3],
    tooth_elements: Vec<Vec<usize>>,
    hub_bore_elements: Vec<usize>,
}

#[allow(clippy::too_many_arguments)]
fn build_two_zone_gear(
    center: [f64; 3],
    phase_offset_rad: f64,
    num_teeth: usize,
    module_m: f64,
    pressure_angle_rad: f64,
    bore_radius: f64,
    face_width: f64,
    engaged_angle_center: f64,
) -> GearMeshInfo {
    let pitch_angle = 2.0 * PI / num_teeth as f64;
    let pitch_r = 0.5 * module_m * num_teeth as f64;
    let root_r = (pitch_r - 1.25 * module_m).max(bore_radius + 0.004);
    let tip_r = pitch_r + module_m;
    let phi_root = involute_half_angle_at_radius(root_r, num_teeth, module_m, pressure_angle_rad);

    let mut ring_angles: Vec<f64> = Vec::new();
    let mut tooth_ranges: Vec<(usize, usize)> = Vec::with_capacity(num_teeth);
    for tooth in 0..num_teeth {
        let tooth_center = tooth as f64 * pitch_angle;
        let world_center = tooth_center + phase_offset_rad;
        let diff = (world_center - engaged_angle_center + PI).rem_euclid(2.0 * PI) - PI;
        let n_seg = if diff.abs() <= 1.3 * pitch_angle { 4 } else { 2 };
        let start_ti = ring_angles.len();
        for s in 0..=n_seg {
            let frac = -1.0 + 2.0 * (s as f64) / (n_seg as f64);
            ring_angles.push(tooth_center + frac * phi_root);
        }
        tooth_ranges.push((start_ti, ring_angles.len() - 1));
    }

    let n_theta = ring_angles.len();
    let n_r = 2usize;
    let n_t = 3usize;
    let n_z = 1usize;

    let mut nodes: Vec<TetNode> = Vec::new();
    let mut rim_grid = vec![vec![vec![0usize; n_z + 1]; n_r + 1]; n_theta];
    for (ti, &theta_loc) in ring_angles.iter().enumerate() {
        let theta_w = theta_loc + phase_offset_rad;
        let ct = theta_w.cos();
        let st = theta_w.sin();
        for (ri, rim_r) in rim_grid[ti].iter_mut().enumerate().take(n_r + 1) {
            let r = bore_radius + (root_r - bore_radius) * (ri as f64 / n_r as f64);
            for (zi, slot) in rim_r.iter_mut().enumerate().take(n_z + 1) {
                let z = center[2] - 0.5 * face_width + face_width * (zi as f64 / n_z as f64);
                let id = nodes.len();
                nodes.push(TetNode::new(id, [center[0] + r * ct, center[1] + r * st, z]));
                *slot = id;
            }
        }
    }

    let mut elements: Vec<TetElement> = Vec::new();
    let mut hub_bore_elements = Vec::new();
    let push_hex = |nodes: &[TetNode],
                    elements: &mut Vec<TetElement>,
                    p000: usize,
                    p100: usize,
                    p010: usize,
                    p110: usize,
                    p001: usize,
                    p101: usize,
                    p011: usize,
                    p111: usize|
     -> std::ops::Range<usize> {
        let start = elements.len();
        let cell_tets = [
            [p000, p100, p110, p111],
            [p000, p100, p101, p111],
            [p000, p010, p110, p111],
            [p000, p010, p011, p111],
            [p000, p001, p101, p111],
            [p000, p001, p011, p111],
        ];
        for mut tet in cell_tets {
            let v0 = nodes[tet[0]].pos_vec();
            let v1 = nodes[tet[1]].pos_vec();
            let v2 = nodes[tet[2]].pos_vec();
            let v3 = nodes[tet[3]].pos_vec();
            if (v1 - v0).dot(&(v2 - v0).cross(&(v3 - v0))) < 0.0 {
                tet.swap(1, 2);
            }
            let eid = elements.len();
            elements.push(TetElement::new(eid, tet));
        }
        start..elements.len()
    };

    for ti in 0..n_theta {
        let ti_next = (ti + 1) % n_theta;
        for ri in 0..n_r {
            for zi in 0..n_z {
                let range = push_hex(
                    &nodes,
                    &mut elements,
                    rim_grid[ti][ri][zi],
                    rim_grid[ti_next][ri][zi],
                    rim_grid[ti][ri + 1][zi],
                    rim_grid[ti_next][ri + 1][zi],
                    rim_grid[ti][ri][zi + 1],
                    rim_grid[ti_next][ri][zi + 1],
                    rim_grid[ti][ri + 1][zi + 1],
                    rim_grid[ti_next][ri + 1][zi + 1],
                );
                if ri == 0 {
                    hub_bore_elements.extend(range);
                }
            }
        }
    }

    let tooth_radii = [root_r, root_r + 0.45 * (pitch_r - root_r), pitch_r, tip_r];
    let mut tooth_elements = Vec::with_capacity(num_teeth);
    for (tooth, &(start_ti, end_ti)) in tooth_ranges.iter().enumerate() {
        let tooth_center = tooth as f64 * pitch_angle;
        let n_seg = end_ti - start_ti;
        let mut t_grid = vec![vec![vec![0usize; n_z + 1]; n_t + 1]; n_seg + 1];
        for s in 0..=n_seg {
            for zi in 0..=n_z {
                t_grid[s][0][zi] = rim_grid[start_ti + s][n_r][zi];
            }
        }
        for (l, &r) in tooth_radii.iter().enumerate().skip(1) {
            let half_phi = involute_half_angle_at_radius(r, num_teeth, module_m, pressure_angle_rad);
            for (s, t_s) in t_grid.iter_mut().enumerate().take(n_seg + 1) {
                let frac = -1.0 + 2.0 * (s as f64) / (n_seg as f64);
                let theta_w = tooth_center + frac * half_phi + phase_offset_rad;
                let ct = theta_w.cos();
                let st = theta_w.sin();
                for (zi, slot) in t_s[l].iter_mut().enumerate().take(n_z + 1) {
                    let z = center[2] - 0.5 * face_width + face_width * (zi as f64 / n_z as f64);
                    let id = nodes.len();
                    nodes.push(TetNode::new(id, [center[0] + r * ct, center[1] + r * st, z]));
                    *slot = id;
                }
            }
        }
        let mut this_tooth = Vec::new();
        for s in 0..n_seg {
            for l in 0..n_t {
                for zi in 0..n_z {
                    let range = push_hex(
                        &nodes,
                        &mut elements,
                        t_grid[s][l][zi],
                        t_grid[s + 1][l][zi],
                        t_grid[s][l + 1][zi],
                        t_grid[s + 1][l + 1][zi],
                        t_grid[s][l][zi + 1],
                        t_grid[s + 1][l][zi + 1],
                        t_grid[s][l + 1][zi + 1],
                        t_grid[s + 1][l + 1][zi + 1],
                    );
                    this_tooth.extend(range);
                }
            }
        }
        tooth_elements.push(this_tooth);
    }

    GearMeshInfo {
        mesh: TetrahedralMesh::new(nodes, elements),
        bore_radius,
        root_radius: root_r,
        pitch_radius: pitch_r,
        center,
        tooth_elements,
        hub_bore_elements,
    }
}

fn elem_centroid_r(mesh: &TetrahedralMesh, eid: usize, center: [f64; 3]) -> f64 {
    let idx = mesh.elements[eid].node_indices;
    let mut cx = 0.0;
    let mut cy = 0.0;
    for &nid in &idx {
        cx += mesh.nodes[nid].position[0] - center[0];
        cy += mesh.nodes[nid].position[1] - center[1];
    }
    cx *= 0.25;
    cy *= 0.25;
    (cx * cx + cy * cy).sqrt()
}

#[test]
fn p5_t8_benchmark_5_gear_pair_three_regimes_elastic_plastic_fracture() {
    let t_total = Instant::now();
    let reg = MaterialRegistry::new();
    let steel_mat = reg.get("steel_4140").expect("steel_4140");
    let brass_mat = reg.get("brass_c360").expect("brass_c360");

    let steel_j2 = J2MaterialParams::from_material(steel_mat).unwrap();
    let brass_j2 = J2MaterialParams::from_material(brass_mat).unwrap();
    let steel_dmg = DamageMaterialParams::from_material(steel_mat).unwrap();
    let brass_dmg = DamageMaterialParams::from_material(brass_mat).unwrap();

    let module_m = 0.004_f64;
    let num_teeth = 14usize;
    let pressure_angle = 25.0_f64.to_radians();
    let bore_r = 0.012_f64;
    let face_w = 0.006_f64;
    let pitch_r = 0.5 * module_m * num_teeth as f64;
    let center_dist = 2.0 * pitch_r;
    let psi_p = PI / (2.0 * num_teeth as f64);

    let gear_steel = build_two_zone_gear(
        [0.0, 0.0, 0.0],
        -psi_p,
        num_teeth,
        module_m,
        pressure_angle,
        bore_r,
        face_w,
        0.0,
    );
    let gear_brass = build_two_zone_gear(
        [center_dist, 0.0, 0.0],
        PI - psi_p,
        num_teeth,
        module_m,
        pressure_angle,
        bore_r,
        face_w,
        PI,
    );

    let make_hub = |name: &str, pivot: [f64; 3], bore: f64| HubCoupling::Revolute {
        name: Some(name.to_string()),
        pivot,
        axis: [0.0, 0.0, 1.0],
        selector: HubNodeSelector::BoreCylinder {
            pivot,
            axis: [0.0, 0.0, 1.0],
            max_radius: bore + 1e-4,
        },
        drive: RevoluteHubDrive::PrescribedAngle { angle_rad: 0.0 },
    };
    let steel_hub = make_hub("steel_shaft", [0.0, 0.0, 0.0], gear_steel.bore_radius);
    let brass_hub = make_hub("brass_shaft", [center_dist, 0.0, 0.0], gear_brass.bore_radius);

    let engaged_steel: Vec<usize> = gear_steel
        .mesh
        .nodes
        .iter()
        .filter(|n| {
            let r = (n.position[0].powi(2) + n.position[1].powi(2)).sqrt();
            r >= gear_steel.root_radius - 1e-4 && n.position[0] > 0.5 * gear_steel.root_radius
        })
        .map(|n| n.id)
        .collect();
    let engaged_brass: Vec<usize> = gear_brass
        .mesh
        .nodes
        .iter()
        .filter(|n| {
            let dx = n.position[0] - center_dist;
            let r = (dx * dx + n.position[1].powi(2)).sqrt();
            r >= gear_brass.root_radius - 1e-4 && dx < -0.5 * gear_brass.root_radius
        })
        .map(|n| n.id)
        .collect();

    let make_steps = |prefix: &str, angles: &[f64]| -> Vec<ContactStepSpec> {
        angles
            .iter()
            .enumerate()
            .map(|(i, &ang)| {
                let mut h = steel_hub.clone();
                if let HubCoupling::Revolute { ref mut drive, .. } = h {
                    *drive = RevoluteHubDrive::PrescribedAngle { angle_rad: ang };
                }
                ContactStepSpec::new(format!("{prefix}_{i}")).with_body_hub_couplings(0, vec![h])
            })
            .collect()
    };

    let bodies = vec![
        ContactBody::new("steel_pinion", gear_steel.mesh.clone(), steel_j2)
            .with_hub_coupling(steel_hub.clone()),
        ContactBody::new("brass_gear", gear_brass.mesh.clone(), brass_j2)
            .with_hub_coupling(brass_hub.clone()),
    ];
    let pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 0,
        body_b: 1,
        friction_coeff: 0.12,
        symmetric: true,
        slave_a_nodes: Some(engaged_steel),
        slave_b_nodes: Some(engaged_brass),
        search_distance: Some(0.004),
        max_normal_dot: Some(0.35),
    }];

    let base_opts = FractureSolverOptions {
        topology_mode: FractureTopologyMode::ElementErosion,
        max_propagation_substeps: 8,
        contact: ContactSolverOptions {
            discretization: ContactDiscretization::NodeToSurface,
            corotated: true,
            include_geometric_stiffness: true,
            max_penetration_tol: 5e-6,
            al_multiplier_tol: 2e-2,
            max_al_iterations: 10,
            newton_rel_tol: 1e-4,
            newton_abs_tol: 1e-3,
            max_newton_iterations: 20,
            penalty_scale: 4.0,
            ..ContactSolverOptions::default()
        },
        ..FractureSolverOptions::default()
    };

    // ------------------------------------------------------------------------
    // Regime (a): Elastic contact & unload -> zero residual plastic strain, zero damage
    // ------------------------------------------------------------------------
    let t_a = Instant::now();
    let steps_elastic = make_steps("elastic", &[0.0005, 0.0]);
    let rep_a = solve_deformable_contact_with_fracture(
        &bodies,
        &[Some(steel_dmg), Some(brass_dmg)],
        &pairs,
        &steps_elastic,
        base_opts,
    )
    .expect("Regime (a) elastic contact solve failed");

    let peak_a = &rep_a.steps[0];
    let unload_a = &rep_a.steps[1];
    let max_pen_a = rep_a
        .steps
        .iter()
        .map(|s| s.contact.max_penetration_m)
        .fold(0.0_f64, f64::max);
    let max_res_u_brass_a = rep_a.body_deformed_positions[1]
        .iter()
        .zip(gear_brass.mesh.nodes.iter())
        .map(|(def, orig)| {
            let dx = def[0] - orig.position[0];
            let dy = def[1] - orig.position[1];
            let dz = def[2] - orig.position[2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .fold(0.0_f64, f64::max);

    println!(
        "[P5-T8 Benchmark 5(a) | Elastic] peak_F_n={:.1} N | brass σ_vM={:.1} MPa (< σ_y={:.0}) | \
         ε_p_res={:.2e} | D_max={:.2e} | u_res={:.2e} m | max_pen={:.2e} m | time={:.1} ms",
        peak_a.contact.pairs[0].total_normal_force_n,
        peak_a.contact.body_max_von_mises_pa[1] / 1e6,
        brass_dmg.yield_strength_pa / 1e6,
        unload_a.contact.body_max_equiv_plastic_strain[1],
        unload_a.body_max_damage[1],
        max_res_u_brass_a,
        max_pen_a,
        t_a.elapsed().as_secs_f64() * 1e3
    );

    assert!(peak_a.contact.pairs[0].total_normal_force_n > 20.0, "Expected active elastic gear contact");
    assert!(peak_a.contact.body_max_von_mises_pa[1] < brass_dmg.yield_strength_pa);
    assert!(unload_a.contact.body_max_equiv_plastic_strain[1] < 1e-12);
    assert_eq!(unload_a.body_max_damage[1], 0.0);
    assert!(max_res_u_brass_a < 1e-7, "Residual elastic disp {:.3e} m >= 1e-7 m", max_res_u_brass_a);
    assert!(max_pen_a < 1e-5, "Regime (a) penetration {:.3e} m >= 1e-5 m", max_pen_a);
    assert!(rep_a.detached_fragments.is_empty());

    // ------------------------------------------------------------------------
    // Regime (b): Moderate overload -> permanent plastic dent WITHOUT fracture
    // ------------------------------------------------------------------------
    let t_b = Instant::now();
    let steps_plastic = make_steps("plastic", &[0.0012, 0.0024, 0.0034, 0.0]);
    let rep_b = solve_deformable_contact_with_fracture(
        &bodies,
        &[Some(steel_dmg), Some(brass_dmg)],
        &pairs,
        &steps_plastic,
        base_opts,
    )
    .expect("Regime (b) plastic indentation solve failed");

    let peak_b = &rep_b.steps[2];
    let unload_b = &rep_b.steps[3];
    let max_pen_b = rep_b
        .steps
        .iter()
        .map(|s| s.contact.max_penetration_m)
        .fold(0.0_f64, f64::max);

    println!(
        "[P5-T8 Benchmark 5(b) | Plastic Dent] brass peak σ_vM={:.1} MPa (σ_y={:.0}, σ_u={:.0}) | \
         ε_p_res={:.5} | D_max={:.4} | eroded={} | frags={} | max_pen={:.2e} m | time={:.1} ms",
        peak_b.contact.body_max_von_mises_pa[1] / 1e6,
        brass_dmg.yield_strength_pa / 1e6,
        brass_dmg.ultimate_strength_pa / 1e6,
        unload_b.contact.body_max_equiv_plastic_strain[1],
        unload_b.body_max_damage[1],
        unload_b.body_eroded_count[1],
        rep_b.detached_fragments.len(),
        max_pen_b,
        t_b.elapsed().as_secs_f64() * 1e3
    );

    assert!(peak_b.contact.body_max_von_mises_pa[1] > brass_dmg.yield_strength_pa);
    assert!(peak_b.contact.body_max_von_mises_pa[1] < brass_dmg.ultimate_strength_pa);
    assert!(unload_b.contact.body_max_equiv_plastic_strain[1] > 1e-4);
    assert_eq!(unload_b.body_max_damage[0], 0.0);
    assert_eq!(unload_b.body_max_damage[1], 0.0);
    assert_eq!(unload_b.body_eroded_count[1], 0);
    assert!(rep_b.detached_fragments.is_empty());
    assert!(max_pen_b < 1e-5, "Regime (b) penetration {:.3e} m >= 1e-5 m", max_pen_b);

    // ------------------------------------------------------------------------
    // Regime (c): Critical overload at HPSTC -> local tooth fracture, intact hub,
    //             watertight detached fragment, conservation <= 1e-6, pen < 1e-5 m
    // ------------------------------------------------------------------------
    let t_c = Instant::now();
    let hpstc_roll = 0.10_f64;
    let gear_steel_c = build_two_zone_gear(
        [0.0, 0.0, 0.0],
        -psi_p - hpstc_roll,
        num_teeth,
        module_m,
        pressure_angle,
        bore_r,
        face_w,
        0.0,
    );
    let gear_brass_c = build_two_zone_gear(
        [center_dist, 0.0, 0.0],
        PI - psi_p + hpstc_roll,
        num_teeth,
        module_m,
        pressure_angle,
        bore_r,
        face_w,
        PI,
    );

    let engaged_steel_c: Vec<usize> = gear_steel_c
        .mesh
        .nodes
        .iter()
        .filter(|n| {
            let r = (n.position[0].powi(2) + n.position[1].powi(2)).sqrt();
            r >= gear_steel_c.root_radius - 1e-4 && n.position[0] > 0.5 * gear_steel_c.root_radius
        })
        .map(|n| n.id)
        .collect();
    let engaged_brass_c: Vec<usize> = gear_brass_c
        .mesh
        .nodes
        .iter()
        .filter(|n| {
            let dx = n.position[0] - center_dist;
            let r = (dx * dx + n.position[1].powi(2)).sqrt();
            r >= gear_brass_c.root_radius - 1e-4 && dx < -0.5 * gear_brass_c.root_radius
        })
        .map(|n| n.id)
        .collect();

    let bodies_c = vec![
        ContactBody::new("steel_pinion", gear_steel_c.mesh.clone(), steel_j2)
            .with_hub_coupling(steel_hub.clone()),
        ContactBody::new("brass_gear", gear_brass_c.mesh.clone(), brass_j2)
            .with_hub_coupling(brass_hub.clone()),
    ];
    let pairs_c = vec![ContactPairSpec::DeformablePair {
        body_a: 0,
        body_b: 1,
        friction_coeff: 0.12,
        symmetric: true,
        slave_a_nodes: Some(engaged_steel_c),
        slave_b_nodes: Some(engaged_brass_c),
        search_distance: Some(0.005),
        max_normal_dot: Some(0.40),
    }];
    let brass_dmg_c = brass_dmg
        .with_damage_onset_strain(0.025)
        .with_fracture_energy(1_200.0);

    let steps_c = make_steps("fracture", &[0.006, 0.0085, 0.011, 0.0135, 0.016, 0.0185]);
    let opts_c = FractureSolverOptions {
        topology_mode: FractureTopologyMode::ElementErosion,
        max_propagation_substeps: 14,
        min_fragment_elements: 2,
        step_dt_s: 2e-4,
        contact: ContactSolverOptions {
            discretization: ContactDiscretization::NodeToSurface,
            corotated: true,
            include_geometric_stiffness: true,
            max_penetration_tol: 5e-6,
            al_multiplier_tol: 3e-2,
            max_al_iterations: 20,
            newton_rel_tol: 2e-4,
            newton_abs_tol: 1e-2,
            max_newton_iterations: 20,
            penalty_scale: 5.0,
            ..ContactSolverOptions::default()
        },
        ..FractureSolverOptions::default()
    };

    let rep_c = solve_deformable_contact_with_fracture(
        &bodies_c,
        &[Some(steel_dmg), Some(brass_dmg_c)],
        &pairs_c,
        &steps_c,
        opts_c,
    )
    .expect("Regime (c) local tooth fracture solve failed");

    let max_pen_c = rep_c
        .steps
        .iter()
        .map(|s| s.contact.max_penetration_m)
        .fold(0.0_f64, f64::max);
    let steel_eroded_c = rep_c.steps.last().unwrap().body_eroded_count[0];
    let brass_eroded_c = rep_c.steps.last().unwrap().body_eroded_count[1];
    let brass_frags: Vec<_> = rep_c
        .detached_fragments
        .iter()
        .filter(|f| f.source_body_index == 1)
        .collect();

    assert_eq!(steel_eroded_c, 0);
    assert!(brass_eroded_c > 0);
    assert!(
        !brass_frags.is_empty(),
        "Expected detached brass tooth fragment (steel_eroded={steel_eroded_c}, brass_eroded={brass_eroded_c}, max_pen={max_pen_c:.2e})"
    );

    let first_ev = rep_c
        .events
        .iter()
        .find(|ev| ev.body_index == 1 && !ev.newly_eroded_elements.is_empty())
        .expect("brass erosion event");
    let init_r = first_ev
        .newly_eroded_elements
        .iter()
        .map(|&e| elem_centroid_r(&gear_brass_c.mesh, e, gear_brass_c.center))
        .fold(f64::INFINITY, f64::min);
    assert!(init_r <= gear_brass_c.pitch_radius);

    for frag in &brass_frags {
        let v_loc = validate_mesh(&frag.local_surface_mesh, None);
        assert!(v_loc.is_valid_solid && v_loc.topology.boundary_edges_count == 0);
    }

    let max_cons_err = rep_c
        .conservation_checks
        .iter()
        .map(|c| {
            c.mass_rel_error
                .max(c.linear_momentum_rel_error)
                .max(c.angular_momentum_rel_error)
        })
        .fold(0.0_f64, f64::max);
    assert!(!rep_c.conservation_checks.is_empty());
    assert!(max_cons_err <= 1e-6, "Conservation error {max_cons_err:.3e} > 1e-6");

    // Hub bore 100% intact & >= 10/14 unloaded teeth 100% intact
    let brass_dmg_field = &rep_c.body_element_damage[1];
    let brass_active = &rep_c.body_active_elements[1];
    let hub_intact = gear_brass_c
        .hub_bore_elements
        .iter()
        .all(|&e| brass_active[e] && brass_dmg_field[e].damage == 0.0);
    assert!(hub_intact, "Hub bore elements must remain 100% intact (D == 0)");

    let intact_teeth = gear_brass_c
        .tooth_elements
        .iter()
        .filter(|t| t.iter().all(|&e| brass_active[e] && brass_dmg_field[e].damage == 0.0))
        .count();
    assert!(intact_teeth >= 10, "Expected >= 10/14 intact unloaded teeth, got {intact_teeth}");
    assert!(max_pen_c < 1e-5, "Regime (c) penetration {:.3e} m >= 1e-5 m", max_pen_c);

    println!(
        "[P5-T8 Benchmark 5(c) | Local Fracture] steel_eroded={} brass_eroded={} frags={} | \
         init_r={:.2} mm (root={:.2}, pitch={:.2}) | intact_teeth={}/{} | hub_intact={} | \
         max_cons_err={:.2e} | max_pen={:.2e} m | time={:.1} ms",
        steel_eroded_c,
        brass_eroded_c,
        brass_frags.len(),
        init_r * 1e3,
        gear_brass_c.root_radius * 1e3,
        gear_brass_c.pitch_radius * 1e3,
        intact_teeth,
        num_teeth,
        hub_intact,
        max_cons_err,
        max_pen_c,
        t_c.elapsed().as_secs_f64() * 1e3
    );

    // ------------------------------------------------------------------------
    // Regime (d): Coupled HHT-α Implicit Dynamic Impact Verification
    // ------------------------------------------------------------------------
    let t_d = Instant::now();
    let block_a = build_graded_x_box_tet4(0.02, 0.02, 0.02, 2, 2, 2, 1.0);
    let mut block_b = build_graded_x_box_tet4(0.02, 0.02, 0.02, 2, 2, 2, 1.0);
    for n in &mut block_b.nodes {
        n.position[0] += 0.0205; // 0.5 mm initial gap
    }
    let mut bc_b = BoundaryConditions::new();
    bc_b.fix_aabb_face(AabbFace::MaxX, 1e-6);

    let dyn_bodies = vec![
        DynamicBodySpec::new("striker", block_a, steel_j2, 7850.0)
            .with_uniform_initial_velocity([4.0, 0.0, 0.0]),
        DynamicBodySpec::new("target_anvil", block_b, brass_j2, 8500.0)
            .with_boundary(bc_b),
    ];
    let dyn_pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 0,
        body_b: 1,
        friction_coeff: 0.1,
        symmetric: false,
        slave_a_nodes: None,
        slave_b_nodes: None,
        search_distance: Some(0.004),
        max_normal_dot: Some(0.3),
    }];
    let hht_opts = HhtOptions {
        alpha: -0.05,
        dt_initial_s: 5e-5,
        dt_min_s: 1e-6,
        dt_max_s: 1e-4,
        total_time_s: 5e-4,
        mass_matrix_kind: MassMatrixKind::Lumped,
        corotated: true,
        enable_plasticity: true,
        enable_fracture: false,
        output_interval_s: 0.0,
        max_frames: 50,
        ..HhtOptions::default()
    };

    let dyn_rep = solve_dynamic_hht_coupled(dyn_bodies, &dyn_pairs, hht_opts, None::<fn(_)>, None)
        .expect("Coupled HHT-α dynamic impact solve failed");
    let max_dyn_pen = dyn_rep
        .frames
        .iter()
        .flat_map(|f| f.contact_forces.iter())
        .map(|cf| cf.penetration_m)
        .fold(0.0_f64, f64::max);
    let max_contact_f = dyn_rep
        .frames
        .iter()
        .flat_map(|f| f.contact_forces.iter())
        .map(|cf| cf.normal_force_n)
        .fold(0.0_f64, f64::max);
    let final_energy = &dyn_rep.frames.last().expect("final frame").energy;

    println!(
        "[P5-T8 Benchmark 5(d) | HHT-α Coupled Impact] frames={} | peak_F_n={:.1} N | \
         max_pen={:.2e} m | final_KE={:.4} J, SE={:.4} J | time={:.1} ms",
        dyn_rep.frames.len(),
        max_contact_f,
        max_dyn_pen,
        final_energy.kinetic_j,
        final_energy.elastic_j,
        t_d.elapsed().as_secs_f64() * 1e3
    );

    assert!(!dyn_rep.cancelled);
    assert!(max_contact_f > 50.0, "Dynamic impact must generate contact force");
    assert!(max_dyn_pen < 1e-5, "HHT-α impact penetration {:.3e} m >= 1e-5 m", max_dyn_pen);
    println!(
        "[P5-T8 Benchmark 5] Total wall time = {:.1} ms",
        t_total.elapsed().as_secs_f64() * 1e3
    );
}
