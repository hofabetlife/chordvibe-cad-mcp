//! Integration and physics verification tests for the implicit `HHT-α` dynamic FEA integrator,
//! consistent/lumped mass matrices, CFL-free stability, 100-period energy conservation,
//! adaptive time-step bisection on contact/plasticity/fracture, `rayon` parallel assembly,
//! and sparse direct $LDL^T$ / `IC(0)-PCG` solvers (`P5-T6`).

use std::f64::consts::PI;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use mcp_cad_core::material::MaterialRegistry;
use mcp_cad_fea::{
    analyze_mesh, estimate_cfl_critical_dt, solve_dynamic_hht_coupled,
    solve_dynamic_hht_single_body, AabbFace, AnalyticalRigidSurface, BoundaryConditions,
    ContactPairSpec, DamageMaterialParams, DynamicBodySpec, DynamicHubCoupling,
    DynamicPrismaticDrive, DynamicRevoluteDrive, HhtOptions, HubNodeSelector, J2MaterialParams,
    LinearSolverKind, MassMatrixKind, TetElement, TetNode, TetrahedralMesh, TimeForceCurve,
};

/// Builds a clean structured 6-tet-per-hex box mesh on `[x0, x1] x [y0, y1] x [z0, z1]`.
fn make_structured_box_mesh(
    origin: [f64; 3],
    dims: [f64; 3],
    nx: usize,
    ny: usize,
    nz: usize,
) -> TetrahedralMesh {
    let mut nodes = Vec::with_capacity((nx + 1) * (ny + 1) * (nz + 1));
    for k in 0..=nz {
        let z = origin[2] + dims[2] * (k as f64) / (nz as f64);
        for j in 0..=ny {
            let y = origin[1] + dims[1] * (j as f64) / (ny as f64);
            for i in 0..=nx {
                let x = origin[0] + dims[0] * (i as f64) / (nx as f64);
                nodes.push(TetNode {
                    id: nodes.len(),
                    position: [x, y, z],
                });
            }
        }
    }

    let idx = |i: usize, j: usize, k: usize| -> usize { (k * (ny + 1) + j) * (nx + 1) + i };

    let mut elements = Vec::with_capacity(nx * ny * nz * 6);
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let v000 = idx(i, j, k);
                let v100 = idx(i + 1, j, k);
                let v010 = idx(i, j + 1, k);
                let v110 = idx(i + 1, j + 1, k);
                let v001 = idx(i, j, k + 1);
                let v101 = idx(i + 1, j, k + 1);
                let v011 = idx(i, j + 1, k + 1);
                let v111 = idx(i + 1, j + 1, k + 1);

                // Kuhn 6-tetrahedra decomposition sharing main diagonal (v000 -> v111)
                let tets = [
                    [v000, v100, v110, v111],
                    [v000, v110, v010, v111],
                    [v000, v010, v011, v111],
                    [v000, v011, v001, v111],
                    [v000, v001, v101, v111],
                    [v000, v101, v100, v111],
                ];
                for mut tet in tets {
                    // Enforce positive signed volume
                    let p = tet.map(|n| nalgebra::Vector3::from(nodes[n].position));
                    let det = (p[1] - p[0]).dot(&(p[2] - p[0]).cross(&(p[3] - p[0])));
                    if det < 0.0 {
                        tet.swap(2, 3);
                    }
                    elements.push(TetElement {
                        id: elements.len(),
                        node_indices: tet,
                    });
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

#[test]
fn test_hht_100_period_free_elastic_oscillator_cfl_free_energy_conservation() {
    // Slender steel cantilever beam (L = 0.20 m, 0.02 x 0.02 m cross-section)
    let l = 0.20_f64;
    let b = 0.02_f64;
    let h = 0.02_f64;
    let mesh = make_structured_box_mesh([0.0, 0.0, 0.0], [l, b, h], 6, 1, 1);
    let reg = MaterialRegistry::new();
    let steel = reg.get("steel_structural_aisi1020").unwrap().clone();
    let j2 = J2MaterialParams::from_material(&steel).expect("valid steel J2 params");

    // Fundamental Euler-Bernoulli cantilever bending period T_1
    let area = b * h;
    let i_z = b * h.powi(3) / 12.0;
    let f1_hz = (1.875_f64.powi(2) / (2.0 * std::f64::consts::PI * l * l))
        * (steel.youngs_modulus_pa() * i_z / (steel.density_kg_m3 * area)).sqrt();
    let period_s = 1.0 / f1_hz;

    // Static initial bend (pure elastic, small deflection) -> released with zero external force
    let mut static_bc = BoundaryConditions::new();
    static_bc.fix_aabb_face(AabbFace::MinX, 1e-6);
    static_bc.add_aabb_face_force(AabbFace::MaxX, 1e-6, [0.0, -50.0, 0.0]);
    let static_report = analyze_mesh("beam_init", &mesh, &steel, &static_bc, None)
        .expect("static initial bend must succeed");

    let mut u0 = Vec::with_capacity(mesh.node_count() * 3);
    for disp in &static_report.nodal_displacements {
        u0.push(disp[0]);
        u0.push(disp[1]);
        u0.push(disp[2]);
    }

    let mut free_bc = BoundaryConditions::new();
    free_bc.fix_aabb_face(AabbFace::MinX, 1e-6);

    let cfl_dt = estimate_cfl_critical_dt(
        &mesh,
        steel.youngs_modulus_pa(),
        steel.poissons_ratio,
        steel.density_kg_m3,
    );
    // Choose Δt = T_1 / 12 (~200 µs) which is >> 30x explicit CFL (~3 µs)!
    let dt = period_s / 12.0;
    assert!(
        dt > 20.0 * cfl_dt,
        "Test must violate explicit CFL by > 20x: dt={dt:.3e}, cfl_dt={cfl_dt:.3e}"
    );
    let total_time_100_periods = 100.0 * period_s;

    // 1. Consistent mass + alpha = 0.0 (unconditionally stable trapezoidal limit of HHT-α)
    let body_consistent = DynamicBodySpec::new("cantilever", mesh.clone(), j2, steel.density_kg_m3)
        .with_boundary(free_bc.clone())
        .with_initial_displacement(u0.clone());

    let opts = HhtOptions {
        alpha: 0.0,
        total_time_s: total_time_100_periods,
        dt_initial_s: dt,
        dt_min_s: dt * 0.1,
        dt_max_s: dt,
        mass_matrix_kind: MassMatrixKind::Consistent,
        linear_solver: LinearSolverKind::SparseDirectLdlt,
        corotated: false,
        enable_plasticity: false,
        enable_fracture: false,
        force_tolerance: 1e-9,
        output_interval_s: period_s,
        max_frames: 120,
        ..HhtOptions::default()
    };

    let report = solve_dynamic_hht_single_body(
        body_consistent,
        opts,
        None::<fn(_)>,
        None,
    )
    .expect("100-period HHT-alpha consistent-mass solve must succeed");

    assert!(!report.cancelled);
    assert!(
        report.total_steps >= 1100,
        "Expected ~1200 steps for 100 periods, got {}",
        report.total_steps
    );
    assert!(
        report.cfl_ratio_max >= 20.0,
        "Expected CFL ratio >= 20x, got {:.2}x",
        report.cfl_ratio_max
    );
    assert!(
        report.initial_total_energy_j > 1e-6,
        "Initial elastic energy must be positive: {}",
        report.initial_total_energy_j
    );
    assert!(
        report.max_energy_drift_pct <= 0.5,
        "100-period energy drift must be <= 0.5%, got {:.6}%",
        report.max_energy_drift_pct
    );

    // 2. Lumped mass + Co-Rotated finite kinematics (still <= 0.5% drift over 100 periods!)
    let body_lumped = DynamicBodySpec::new("cantilever_lumped", mesh.clone(), j2, steel.density_kg_m3)
        .with_boundary(free_bc.clone())
        .with_initial_displacement(u0);
    let opts_lumped = HhtOptions {
        mass_matrix_kind: MassMatrixKind::Lumped,
        corotated: true,
        ..opts
    };
    let report_lumped = solve_dynamic_hht_single_body(
        body_lumped,
        opts_lumped,
        None::<fn(_)>,
        None,
    )
    .expect("100-period Co-Rotated lumped-mass solve must succeed");
    assert!(
        report_lumped.max_energy_drift_pct <= 0.5,
        "100-period Co-Rotated lumped energy drift must be <= 0.5%, got {:.6}%",
        report_lumped.max_energy_drift_pct
    );

    // 3. Quadratic TET10 with HRZ lumped mass and mild HHT high-frequency damping (alpha = -0.1)
    let mesh_tet10 = mesh.to_tet10();
    let static_tet10 = analyze_mesh("beam_t10", &mesh_tet10, &steel, &static_bc, None)
        .expect("static TET10 initial bend");
    let mut u0_t10 = Vec::with_capacity(mesh_tet10.node_count() * 3);
    for d in &static_tet10.nodal_displacements {
        u0_t10.extend_from_slice(d);
    }
    let body_t10 = DynamicBodySpec::new("cantilever_t10", mesh_tet10, j2, steel.density_kg_m3)
        .with_boundary(free_bc)
        .with_initial_displacement(u0_t10);
    let opts_t10 = HhtOptions {
        alpha: -0.1,
        total_time_s: 10.0 * period_s,
        dt_initial_s: dt,
        dt_min_s: dt * 0.1,
        dt_max_s: dt,
        mass_matrix_kind: MassMatrixKind::Lumped,
        linear_solver: LinearSolverKind::Auto,
        corotated: false,
        enable_plasticity: false,
        enable_fracture: false,
        output_interval_s: period_s,
        max_frames: 20,
        ..HhtOptions::default()
    };
    let report_t10 = solve_dynamic_hht_single_body(body_t10, opts_t10, None::<fn(_)>, None)
        .expect("TET10 HHT-alpha solve must succeed");
    assert!(report_t10.cfl_ratio_max >= 40.0);
    assert!(
        report_t10.final_total_energy_j <= report_t10.initial_total_energy_j * 1.005,
        "HHT alpha=-0.1 must be dissipative/bounded: E_final={} vs E_init={}",
        report_t10.final_total_energy_j,
        report_t10.initial_total_energy_j
    );
}

#[test]
fn test_hht_adaptive_bisection_on_plasticity_and_fracture_pulse() {
    // Ductile/brittle tensile bar loaded by a strong dynamic tensile pulse exceeding ultimate strength
    let mesh = make_structured_box_mesh([0.0, 0.0, 0.0], [0.06, 0.02, 0.02], 3, 1, 1);
    let reg = MaterialRegistry::new();
    let glass = reg.get("borosilicate_glass").unwrap().clone();
    let j2 = J2MaterialParams::from_material(&glass).expect("valid glass J2");
    let dmg = DamageMaterialParams::from_material(&glass).expect("valid glass damage");

    let mut bc = BoundaryConditions::new();
    bc.fix_aabb_face(AabbFace::MinX, 1e-6);
    // Tensile force: cross-section = 4e-4 m^2, σ_u = 60 MPa => failure force ~ 24 kN; apply 55 kN ramp
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-6, [55_000.0, 0.0, 0.0]);

    let body = DynamicBodySpec::new("glass_bar", mesh, j2, glass.density_kg_m3)
        .with_boundary(bc)
        .with_damage(dmg)
        .with_force_curve(TimeForceCurve::Ramp { duration_s: 0.002 });

    let mut progress_samples = Vec::new();
    let opts = HhtOptions {
        alpha: -0.05,
        total_time_s: 0.003,
        dt_initial_s: 5e-4,
        dt_min_s: 2e-5,
        dt_max_s: 5e-4,
        mass_matrix_kind: MassMatrixKind::Lumped,
        corotated: true,
        enable_plasticity: true,
        enable_fracture: true,
        max_damage_jump_per_step: 0.25,
        output_interval_s: 0.0,
        max_frames: 100,
        ..HhtOptions::default()
    };

    let report = solve_dynamic_hht_single_body(
        body,
        opts,
        Some(|p| progress_samples.push(p)),
        None,
    )
    .expect("fracture dynamic pulse must converge via adaptive bisection");

    assert!(!report.cancelled);
    assert!(
        report.total_bisections > 0,
        "Expected adaptive step bisection on brittle fracture shock, got 0"
    );
    assert!(
        report.min_dt_used_s < opts.dt_initial_s,
        "Expected min_dt_used_s ({}) < dt_initial_s ({})",
        report.min_dt_used_s,
        opts.dt_initial_s
    );
    let last_frame = report.frames.last().expect("at least 1 frame");
    assert!(
        last_frame.bodies[0].max_damage > 0.5,
        "Expected high damage during overload pulse, got {}",
        last_frame.bodies[0].max_damage
    );
    assert!(
        !last_frame.bodies[0].eroded_element_indices.is_empty(),
        "Expected at least one eroded element under 55 kN tensile shock on borosilicate glass"
    );
    assert!(progress_samples.len() >= 2);
    assert!((progress_samples.last().unwrap().progress_pct - 100.0).abs() < 1e-6);
}

#[test]
fn test_hht_multibody_deformable_contact_impact_and_cancellation() {
    // Block A flying at +4 m/s along X into a stationary deformable Block B (fixed on MaxX)
    let mesh_a = make_structured_box_mesh([0.0, 0.0, 0.0], [0.02, 0.02, 0.02], 2, 1, 1);
    let mesh_b = make_structured_box_mesh([0.021, 0.0, 0.0], [0.02, 0.02, 0.02], 2, 1, 1);

    let reg = MaterialRegistry::new();
    let alu = reg.get("aluminum_6061_t6").unwrap().clone();
    let j2 = J2MaterialParams::from_material(&alu).expect("valid aluminum J2");

    let mut bc_b = BoundaryConditions::new();
    bc_b.fix_aabb_face(AabbFace::MaxX, 1e-6);

    let body_a = DynamicBodySpec::new("impactor", mesh_a, j2, alu.density_kg_m3)
        .with_uniform_initial_velocity([4.0, 0.0, 0.0]);
    let body_b = DynamicBodySpec::new("target", mesh_b, j2, alu.density_kg_m3).with_boundary(bc_b);

    let pairs = vec![
        ContactPairSpec::DeformablePair {
            body_a: 0,
            body_b: 1,
            friction_coeff: 0.15,
            symmetric: true,
            slave_a_nodes: None,
            slave_b_nodes: None,
            search_distance: Some(0.01),
            max_normal_dot: Some(0.35),
        },
        // Also verify deformable-to-rigid floor plane below the blocks
        ContactPairSpec::DeformableToRigid {
            body_idx: 0,
            surface: AnalyticalRigidSurface::Plane {
                origin: [0.0, -0.005, 0.0],
                normal: [0.0, 1.0, 0.0],
            },
            friction_coeff: 0.1,
            slave_nodes: None,
            search_distance: Some(0.01),
        },
    ];

    // Gap = 1 mm, speed = 4 m/s => contact starts around t = 0.25 ms
    let opts = HhtOptions {
        alpha: -0.05,
        total_time_s: 0.0012,
        dt_initial_s: 1e-4,
        dt_min_s: 1e-5,
        dt_max_s: 1e-4,
        mass_matrix_kind: MassMatrixKind::Lumped,
        corotated: true,
        enable_plasticity: true,
        enable_fracture: false,
        output_interval_s: 0.0,
        max_frames: 50,
        ..HhtOptions::default()
    };

    let report = solve_dynamic_hht_coupled(
        vec![body_a.clone(), body_b.clone()],
        &pairs,
        opts,
        None::<fn(_)>,
        None,
    )
    .expect("two-body dynamic impact must succeed");

    assert!(!report.cancelled);
    let any_contact_force = report
        .frames
        .iter()
        .any(|f| f.contact_forces.iter().any(|c| c.normal_force_n > 1.0));
    assert!(
        any_contact_force,
        "Expected positive dynamic contact normal force when impactor closes 1 mm gap"
    );
    let max_target_vm = report
        .frames
        .iter()
        .map(|f| f.bodies[1].max_von_mises_pa)
        .fold(0.0_f64, f64::max);
    assert!(
        max_target_vm > 1e5,
        "Expected significant impact stress transmitted into target block, got {max_target_vm} Pa"
    );

    // Verify immediate cancellation via AtomicBool
    let cancel_flag = Arc::new(AtomicBool::new(false));
    let cancel_clone = Arc::clone(&cancel_flag);
    let cancelled_report = solve_dynamic_hht_coupled(
        vec![body_a, body_b],
        &pairs,
        opts,
        Some(move |upd: mcp_cad_fea::DynamicProgressUpdate| {
            if upd.step_count >= 3 {
                cancel_clone.store(true, Ordering::Relaxed);
            }
        }),
        Some(cancel_flag.as_ref()),
    )
    .expect("cancelled simulation should return cleanly");

    assert!(cancelled_report.cancelled);
    assert!(
        cancelled_report.total_steps <= 4,
        "Expected early cancellation around step 3, got {} steps",
        cancelled_report.total_steps
    );
    assert!(cancelled_report.events.iter().any(|e| e.kind == "cancelled"));
}

#[test]
fn test_rayon_parallel_assembly_and_linear_solvers_agree() {
    // 4 x 2 x 2 hexes * 6 tets = 96 elements (> PARALLEL_ELEMENT_THRESHOLD = 64)
    let mesh = make_structured_box_mesh([0.0, 0.0, 0.0], [0.10, 0.03, 0.03], 4, 2, 2);
    assert!(mesh.elements.len() >= 96);

    let reg = MaterialRegistry::new();
    let steel = reg.get("steel_structural_aisi1020").unwrap().clone();
    let j2 = J2MaterialParams::from_material(&steel).expect("steel J2");
    let mut bc = BoundaryConditions::new();
    bc.fix_aabb_face(AabbFace::MinX, 1e-6);
    bc.add_aabb_face_force(AabbFace::MaxX, 1e-6, [0.0, -200.0, 0.0]);

    let mut final_disps = Vec::new();
    for kind in [
        LinearSolverKind::SparseDirectLdlt,
        LinearSolverKind::IcPcg,
        LinearSolverKind::SsorPcg,
    ] {
        let body = DynamicBodySpec::new("par_beam", mesh.clone(), j2, steel.density_kg_m3)
            .with_boundary(bc.clone());
        let opts = HhtOptions {
            alpha: -0.05,
            total_time_s: 0.0006,
            dt_initial_s: 2e-4,
            dt_min_s: 5e-5,
            dt_max_s: 2e-4,
            mass_matrix_kind: MassMatrixKind::Consistent,
            linear_solver: kind,
            corotated: true,
            enable_plasticity: false,
            enable_fracture: false,
            ..HhtOptions::default()
        };
        let rep = solve_dynamic_hht_single_body(body, opts, None::<fn(_)>, None)
            .expect("parallel dynamic solve must succeed");
        final_disps.push(rep.frames.last().unwrap().bodies[0].max_displacement_m);
    }

    let ref_d = final_disps[0];
    assert!(ref_d > 1e-7);
    for (idx, &d) in final_disps.iter().enumerate() {
        let rel = (d - ref_d).abs() / ref_d;
        assert!(
            rel < 1e-6,
            "Linear solver variant {idx} diverged from SparseDirectLdlt: {d} vs {ref_d} (rel={rel})"
        );
    }
}

/// Evaluates outer radial profile $r(\theta)$ of a $20^\circ$ involute spur gear for conforming
/// dynamic contact verification (`P5-T14`).
fn involute_spur_outer_radius(
    theta: f64,
    num_teeth: usize,
    module_m: f64,
    pressure_angle_rad: f64,
) -> f64 {
    let pitch_r = 0.5 * module_m * num_teeth as f64;
    let base_r = pitch_r * pressure_angle_rad.cos();
    let tip_r = pitch_r + module_m;
    let root_r = (pitch_r - 1.25 * module_m).max(base_r * 0.72);

    let pitch_angle = 2.0 * PI / num_teeth as f64;
    let psi_p = PI / (2.0 * num_teeth as f64);
    let inv_alpha = pressure_angle_rad.tan() - pressure_angle_rad;
    let psi_b = psi_p + inv_alpha;

    let mut phi = theta.rem_euclid(pitch_angle);
    if phi > 0.5 * pitch_angle {
        phi -= pitch_angle;
    }
    let abs_phi = phi.abs();

    let t_tip = ((tip_r / base_r).powi(2) - 1.0).max(0.0).sqrt();
    let phi_tip = (psi_b - (t_tip - t_tip.atan())).max(0.0);
    let phi_root = (psi_b + 0.14 * (0.5 * pitch_angle - psi_b)).min(0.46 * pitch_angle);

    if abs_phi <= phi_tip {
        tip_r
    } else if abs_phi <= psi_b {
        let target_inv = psi_b - abs_phi;
        let mut lo = 0.0_f64;
        let mut hi = t_tip;
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            if mid - mid.atan() < target_inv {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let t = 0.5 * (lo + hi);
        base_r * (1.0 + t * t).sqrt()
    } else if abs_phi <= phi_root {
        let u = (abs_phi - psi_b) / (phi_root - psi_b).max(1e-9);
        let w = 0.5 * (1.0 + (PI * u).cos());
        root_r + (base_r - root_r) * w
    } else {
        root_r
    }
}

/// Builds a compact boundary-conforming 3D involute spur gear `TET4` mesh with a circular shaft
/// bore and refined flank resolution near `engaged_angle_center` (`P5-T14`).
#[allow(clippy::too_many_arguments)]
fn make_conforming_involute_gear_mesh(
    center: [f64; 3],
    phase_offset_rad: f64,
    num_teeth: usize,
    module_m: f64,
    pressure_angle_rad: f64,
    bore_radius: f64,
    face_width: f64,
    engaged_angle_center: f64,
    n_radial: usize,
    nz: usize,
) -> TetrahedralMesh {
    let pitch_angle = 2.0 * PI / num_teeth as f64;
    let psi_p = PI / (2.0 * num_teeth as f64);
    let inv_alpha = pressure_angle_rad.tan() - pressure_angle_rad;
    let psi_b = psi_p + inv_alpha;
    let pitch_r = 0.5 * module_m * num_teeth as f64;
    let base_r = pitch_r * pressure_angle_rad.cos();
    let tip_r = pitch_r + module_m;
    let t_tip = ((tip_r / base_r).powi(2) - 1.0).max(0.0).sqrt();
    let phi_tip = (psi_b - (t_tip - t_tip.atan())).max(0.0);
    let phi_root = (psi_b + 0.14 * (0.5 * pitch_angle - psi_b)).min(0.46 * pitch_angle);

    let mut angles = Vec::new();
    for tooth in 0..num_teeth {
        let tooth_center = tooth as f64 * pitch_angle;
        let world_center = tooth_center + phase_offset_rad;
        let diff = (world_center - engaged_angle_center + PI).rem_euclid(2.0 * PI) - PI;
        let is_engaged = diff.abs() <= 1.65 * pitch_angle;

        let local_pts: Vec<f64> = if is_engaged {
            let mut pts = Vec::new();
            pts.push(-0.5 * pitch_angle);
            pts.push(-phi_root);
            let n_flank = 3usize;
            for k in 0..=n_flank {
                let s = k as f64 / n_flank as f64;
                pts.push(-(psi_b * (1.0 - s) + phi_tip * s));
            }
            pts.push(0.0);
            for k in 0..=n_flank {
                let s = k as f64 / n_flank as f64;
                pts.push(phi_tip * (1.0 - s) + psi_b * s);
            }
            pts.push(phi_root);
            pts
        } else {
            vec![-0.5 * pitch_angle, -phi_tip, phi_tip]
        };

        for lp in local_pts {
            angles.push((tooth_center + lp).rem_euclid(2.0 * PI));
        }
    }
    angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut unique_angles: Vec<f64> = Vec::with_capacity(angles.len());
    for a in angles {
        if unique_angles
            .last()
            .is_none_or(|&prev| (a - prev).abs() > 1e-6)
        {
            unique_angles.push(a);
        }
    }
    if unique_angles.len() >= 2
        && (2.0 * PI - (*unique_angles.last().unwrap() - unique_angles[0])).abs() < 1e-6
    {
        unique_angles.pop();
    }

    let n_theta = unique_angles.len();
    let n_r = n_radial.max(2);
    let n_z = nz.max(1);
    let node_id = |ti: usize, ri: usize, zi: usize| -> usize {
        (ti % n_theta) * (n_r + 1) * (n_z + 1) + ri * (n_z + 1) + zi
    };

    let mut nodes = Vec::with_capacity(n_theta * (n_r + 1) * (n_z + 1));
    for &theta_local in &unique_angles {
        let r_outer = involute_spur_outer_radius(
            theta_local,
            num_teeth,
            module_m,
            pressure_angle_rad,
        );
        let theta_world = theta_local + phase_offset_rad;
        let ct = theta_world.cos();
        let st = theta_world.sin();
        for ri in 0..=n_r {
            let u = ri as f64 / n_r as f64;
            let r = bore_radius + (r_outer - bore_radius) * u;
            let x = center[0] + r * ct;
            let y = center[1] + r * st;
            for zi in 0..=n_z {
                let z = center[2] - 0.5 * face_width + face_width * (zi as f64 / n_z as f64);
                nodes.push(TetNode::new(nodes.len(), [x, y, z]));
            }
        }
    }

    let mut elements = Vec::with_capacity(n_theta * n_r * n_z * 6);
    let mut elem_id = 0usize;
    for ti in 0..n_theta {
        let ti_next = (ti + 1) % n_theta;
        for ri in 0..n_r {
            for zi in 0..n_z {
                let p000 = node_id(ti, ri, zi);
                let p100 = node_id(ti_next, ri, zi);
                let p010 = node_id(ti, ri + 1, zi);
                let p110 = node_id(ti_next, ri + 1, zi);
                let p001 = node_id(ti, ri, zi + 1);
                let p101 = node_id(ti_next, ri, zi + 1);
                let p011 = node_id(ti, ri + 1, zi + 1);
                let p111 = node_id(ti_next, ri + 1, zi + 1);
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
                    elements.push(TetElement::new(elem_id, tet));
                    elem_id += 1;
                }
            }
        }
    }

    TetrahedralMesh::new(nodes, elements)
}

#[test]
fn test_p5_t14_dynamic_gear_pair_revolute_motor_contact_and_torque_transmission() {
    let reg = MaterialRegistry::new();
    let steel = reg.get("steel_structural_aisi1020").unwrap().clone();
    let j2 = J2MaterialParams::from_material(&steel).expect("steel J2");

    let num_teeth = 12usize;
    let module_m = 0.003; // 3 mm module -> pitch radius 18 mm, center distance 36 mm
    let pressure_angle_rad = 20.0_f64.to_radians();
    let bore_r = 0.006;
    let face_width = 0.006;
    let center_dist = module_m * num_teeth as f64;
    let gear2_phase = PI / num_teeth as f64; // Half-tooth pitch so teeth mesh cleanly

    let pinion_mesh = make_conforming_involute_gear_mesh(
        [0.0, 0.0, 0.0],
        0.0,
        num_teeth,
        module_m,
        pressure_angle_rad,
        bore_r,
        face_width,
        0.0,
        2,
        1,
    );
    let gear_mesh = make_conforming_involute_gear_mesh(
        [center_dist, 0.0, 0.0],
        gear2_phase,
        num_teeth,
        module_m,
        pressure_angle_rad,
        bore_r,
        face_width,
        PI,
        2,
        1,
    );

    let pinion_bore_nodes: Vec<usize> = pinion_mesh
        .nodes
        .iter()
        .filter(|n| (n.position[0].hypot(n.position[1]) - bore_r).abs() <= 1e-4)
        .map(|n| n.id)
        .collect();
    let gear_bore_nodes: Vec<usize> = gear_mesh
        .nodes
        .iter()
        .filter(|n| {
            ((n.position[0] - center_dist).hypot(n.position[1]) - bore_r).abs() <= 1e-4
        })
        .map(|n| n.id)
        .collect();
    assert!(pinion_bore_nodes.len() >= 12);
    assert!(gear_bore_nodes.len() >= 12);

    // Pinion driven purely by a Revolute Motor hub (NO manual initial_velocity!)
    let pinion_body = DynamicBodySpec::new("pinion", pinion_mesh, j2, steel.density_kg_m3)
        .with_hub_coupling(DynamicHubCoupling::Revolute {
            name: Some("pinion_motor".to_string()),
            pivot: [0.0, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            selector: HubNodeSelector::Nodes(pinion_bore_nodes),
            drive: DynamicRevoluteDrive::Motor {
                target_velocity_rad_s: 14.0,
                max_torque_nm: 60.0,
                velocity_gain_nm_s_per_rad: 0.0,
                ramp_duration_s: 0.0004,
                limits: None,
            },
        });

    // Driven gear mounted on a passive Revolute bearing with realistic resistive viscous damping
    let gear_body = DynamicBodySpec::new("gear", gear_mesh, j2, steel.density_kg_m3)
        .with_hub_coupling(DynamicHubCoupling::Revolute {
            name: Some("gear_bearing".to_string()),
            pivot: [center_dist, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            selector: HubNodeSelector::Nodes(gear_bore_nodes),
            drive: DynamicRevoluteDrive::FreeTorque {
                applied_torque_nm: 0.0,
                viscous_damping_nm_s_per_rad: 0.05,
                torsional_stiffness_nm_per_rad: 0.0,
                reference_angle_rad: 0.0,
                limits: None,
            },
        });

    let contact_pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 0,
        body_b: 1,
        friction_coeff: 0.1,
        symmetric: true,
        slave_a_nodes: None,
        slave_b_nodes: None,
        search_distance: Some(0.004),
        max_normal_dot: Some(0.25),
    }];

    let opts = HhtOptions {
        alpha: -0.05,
        total_time_s: 0.003,
        dt_initial_s: 2.5e-4,
        dt_min_s: 2.5e-5,
        dt_max_s: 2.5e-4,
        mass_matrix_kind: MassMatrixKind::Lumped,
        corotated: true,
        enable_plasticity: false,
        enable_fracture: false,
        output_interval_s: 0.0,
        max_frames: 60,
        ..HhtOptions::default()
    };

    let report = solve_dynamic_hht_coupled(
        vec![pinion_body, gear_body],
        &contact_pairs,
        opts,
        None::<fn(_)>,
        None,
    )
    .expect("Dynamic HHT-alpha gear pair with Revolute motor must succeed");

    assert!(!report.cancelled);
    assert!(report.frames.len() >= 5);

    let last = report.frames.last().expect("final frame present");
    assert_eq!(last.hub_reactions.len(), 2);

    let pinion_hub = last
        .hub_reactions
        .iter()
        .find(|h| h.part_id == "pinion")
        .expect("pinion hub reaction");
    let gear_hub = last
        .hub_reactions
        .iter()
        .find(|h| h.part_id == "gear")
        .expect("gear hub reaction");

    let max_contact_n = report
        .frames
        .iter()
        .flat_map(|f| f.contact_forces.iter())
        .map(|c| c.normal_force_n)
        .fold(0.0_f64, f64::max);
    let max_pinion_torque = report
        .frames
        .iter()
        .flat_map(|f| f.hub_reactions.iter())
        .filter(|h| h.part_id == "pinion")
        .map(|h| h.reaction_effort.abs())
        .fold(0.0_f64, f64::max);
    let max_gear_torque = report
        .frames
        .iter()
        .flat_map(|f| f.hub_reactions.iter())
        .filter(|h| h.part_id == "gear")
        .map(|h| h.reaction_effort.abs())
        .fold(0.0_f64, f64::max);
    let max_pinion_vm = report
        .frames
        .iter()
        .map(|f| f.bodies[0].max_von_mises_pa)
        .fold(0.0_f64, f64::max);
    let max_gear_vm = report
        .frames
        .iter()
        .map(|f| f.bodies[1].max_von_mises_pa)
        .fold(0.0_f64, f64::max);

    assert!(
        pinion_hub.coordinate > 0.01 && pinion_hub.velocity > 2.0,
        "Pinion must rotate CCW under Revolute motor: theta={:.5} rad, omega={:.3} rad/s",
        pinion_hub.coordinate,
        pinion_hub.velocity
    );
    assert!(
        max_contact_n > 1.0,
        "Expected dynamic tooth-to-tooth contact force > 1 N, got {max_contact_n:.4} N"
    );
    assert!(
        gear_hub.coordinate < -0.002 && gear_hub.velocity < -1.0,
        "Driven gear must rotate CW via tooth meshing contact: theta={:.5} rad, omega={:.3} rad/s",
        gear_hub.coordinate,
        gear_hub.velocity
    );
    assert!(
        max_pinion_torque > 0.05 && max_gear_torque > 0.01,
        "Expected non-zero torque transmission on both hubs: pinion={max_pinion_torque:.4} N*m, gear={max_gear_torque:.4} N*m"
    );
    assert!(
        max_pinion_vm > 1e5 && max_gear_vm > 1e5,
        "Expected tooth contact stress > 0.1 MPa on both gears: pinion={:.2} MPa, gear={:.2} MPa",
        max_pinion_vm * 1e-6,
        max_gear_vm * 1e-6
    );
}

#[test]
fn test_p5_t14_dynamic_prismatic_motor_and_fixed_hub_couplings() {
    let reg = MaterialRegistry::new();
    let steel = reg.get("steel_structural_aisi1020").unwrap().clone();
    let j2 = J2MaterialParams::from_material(&steel).expect("steel J2");

    // Slider block driven along +X by a Prismatic motor hub into a Fixed-hub anvil across a 0.5 mm gap
    let slider_mesh = make_structured_box_mesh([0.0, 0.0, 0.0], [0.02, 0.02, 0.02], 2, 2, 2);
    let anvil_mesh = make_structured_box_mesh([0.0205, 0.0, 0.0], [0.02, 0.02, 0.02], 2, 2, 2);

    let slider_body = DynamicBodySpec::new("slider", slider_mesh, j2, steel.density_kg_m3)
        .with_hub_coupling(DynamicHubCoupling::Prismatic {
            name: Some("slider_motor".to_string()),
            axis: [1.0, 0.0, 0.0],
            selector: HubNodeSelector::AabbFace {
                face: AabbFace::MinX,
                tolerance: 1e-5,
            },
            drive: DynamicPrismaticDrive::Motor {
                target_velocity_m_s: 2.0,
                max_force_n: 5000.0,
                velocity_gain_n_s_per_m: 0.0,
                ramp_duration_s: 0.0002,
                limits: None,
            },
            lock_transverse: true,
        });

    let anvil_body = DynamicBodySpec::new("anvil", anvil_mesh, j2, steel.density_kg_m3)
        .with_hub_coupling(DynamicHubCoupling::Fixed {
            name: Some("anvil_anchor".to_string()),
            selector: HubNodeSelector::AabbFace {
                face: AabbFace::MaxX,
                tolerance: 1e-5,
            },
        });

    let pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 0,
        body_b: 1,
        friction_coeff: 0.1,
        symmetric: true,
        slave_a_nodes: None,
        slave_b_nodes: None,
        search_distance: Some(0.005),
        max_normal_dot: Some(0.25),
    }];

    let opts = HhtOptions {
        alpha: -0.05,
        total_time_s: 0.0012,
        dt_initial_s: 1.5e-4,
        dt_min_s: 1e-5,
        dt_max_s: 1.5e-4,
        mass_matrix_kind: MassMatrixKind::Lumped,
        corotated: true,
        enable_plasticity: false,
        enable_fracture: false,
        output_interval_s: 0.0,
        max_frames: 40,
        ..HhtOptions::default()
    };

    let report = solve_dynamic_hht_coupled(
        vec![slider_body, anvil_body],
        &pairs,
        opts,
        None::<fn(_)>,
        None,
    )
    .expect("Dynamic HHT-alpha Prismatic + Fixed hub simulation must succeed");

    assert!(!report.cancelled);
    let last = report.frames.last().expect("final frame");
    let slider_hub = last
        .hub_reactions
        .iter()
        .find(|h| h.part_id == "slider")
        .expect("slider hub");
    let anvil_hub = last
        .hub_reactions
        .iter()
        .find(|h| h.part_id == "anvil")
        .expect("anvil hub");

    let max_contact_n = report
        .frames
        .iter()
        .flat_map(|f| f.contact_forces.iter())
        .map(|c| c.normal_force_n)
        .fold(0.0_f64, f64::max);
    let max_anvil_rx = report
        .frames
        .iter()
        .flat_map(|f| f.hub_reactions.iter())
        .filter(|h| h.part_id == "anvil")
        .map(|h| h.reaction_force_n[0].abs())
        .fold(0.0_f64, f64::max);

    assert!(
        slider_hub.coordinate > 0.0004,
        "Prismatic slider must advance along +X under motor drive: q={:.6} m",
        slider_hub.coordinate
    );
    assert!(
        anvil_hub.coordinate.abs() < 1e-12 && anvil_hub.velocity.abs() < 1e-12,
        "Fixed hub must remain strictly stationary"
    );
    assert!(
        max_contact_n > 1.0 && max_anvil_rx > 1.0,
        "Expected slider-anvil impact contact ({max_contact_n:.2} N) and Fixed-hub reaction ({max_anvil_rx:.2} N)"
    );
}

