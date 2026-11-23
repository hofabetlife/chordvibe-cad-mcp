//! Integration & analytical benchmark tests for multi-body deformable contact (`P5-T4`).
//!
//! Covers:
//! 1. **Analytical Hertz Contact of Two Elastic Cylinders** (`<= 5.0%` error on both peak pressure
//!    $p_0$ and contact half-width $a$, exact force balance, subsurface von Mises peak).
//! 2. **Dry Coulomb Friction (Stick & Slip Regimes)** on deformable-to-deformable contact.
//! 3. **Two Meshing Deformable Involute Spur Gears Under Rotation & Torque** with `HubCoupling::Revolute`
//!    bore coupling, multi-step contact pressure along the 3D line of contact, and strict
//!    interpenetration `< 1e-5 m` on every step.
//! 4. **Elastoplastic Indentation & Dynamic Deformed Contact Surface Feedback**.
//! 5. **Quadratic `TET10` Mortar Gauss-Point Contact Verification**.

use std::f64::consts::PI;

use mcp_cad_core::material::Material;
use mcp_cad_fea::{
    solve_deformable_contact, AabbFace, BoundaryConditions, ContactBody, ContactDiscretization,
    ContactPairSpec, ContactSolverOptions, ContactStepSpec, DirichletConstraint, DofComponent,
    HardeningLaw, HubCoupling, HubNodeSelector, J2MaterialParams, PrismaticHubDrive,
    RevoluteHubDrive, TetElement, TetNode, TetrahedralMesh,
};

fn make_j2(e_pa: f64, nu: f64, yield_pa: f64, hardening_pa: f64) -> J2MaterialParams {
    let mut mat = Material::new(
        "test_mat",
        e_pa / 1e9,
        nu,
        yield_pa / 1e6,
        7850.0,
        "test material",
    );
    mat.hardening_modulus_pa = hardening_pa;
    mat.ultimate_strength_pa = (yield_pa * 1.6).max(yield_pa);
    J2MaterialParams::from_material(&mat)
        .expect("valid J2MaterialParams")
        .with_hardening(HardeningLaw::Linear {
            hardening_modulus_pa: hardening_pa,
        })
}

// ============================================================================
// Structured Graded Cylinder Sector Mesh Builder (for Hertz 2-Cylinder Benchmark)
// ============================================================================

/// Builds a 3D graded cylindrical sector `TET4` mesh with exact cylindrical surface curvature
/// $y = \pm (R - \sqrt{R^2 - x^2})$ around the contact apex $(0, 0, z)$ and mirrored x-symmetry.
///
/// - If `is_top == false`, the cylinder lies in $y \le 0$ with center at $(0, -R, 0)$ and apex at $y = 0$.
/// - If `is_top == true`, the cylinder lies in $y \ge 0$ with center at $(0, +R, 0)$ and apex at $y = 0$.
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
    // Build symmetric x coordinates: -half_width_x .. -fine_half_x .. 0 .. +fine_half_x .. +half_width_x
    let mut xs_pos = Vec::with_capacity(n_fine_half_x + n_coarse_half_x + 1);
    for i in 0..=n_fine_half_x {
        xs_pos.push(fine_half_x * (i as f64 / n_fine_half_x as f64));
    }
    let dx_fine = fine_half_x / n_fine_half_x as f64;
    let rem_x = (half_width_x - fine_half_x).max(dx_fine);
    // Solve geometric ratio r such that sum_{k=1..=n} dx_fine * r^k = rem_x
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

    // Build graded depth parameter s in [0, 1] from cylindrical surface (s = 0) to flat base (s = 1)
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
            // Interpolate from exact cylindrical arc (s = 0) to flat far-field base y = ±depth_y (s = 1)
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

            // Mirror Kuhn diagonal across x = 0 so the tetrahedralization is strictly symmetric
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

// ============================================================================
// Conforming 3D Involute Spur Gear Mesh Builder (for Two-Gear Contact Test)
// ============================================================================

/// Evaluates the exact outer radial boundary $r_{\text{outer}}(\theta)$ of a standard
/// $20^\circ$ involute spur gear at polar angle `theta` (matching `mcp-cad-core::create_spur_gear`).
fn involute_gear_outer_radius(
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

    // Fold angle into [-pitch_angle/2, +pitch_angle/2] relative to nearest tooth centerline
    let mut phi = theta.rem_euclid(pitch_angle);
    if phi > 0.5 * pitch_angle {
        phi -= pitch_angle;
    }
    let abs_phi = phi.abs();

    // Tip angle of involute flank
    let t_tip = ((tip_r / base_r).powi(2) - 1.0).max(0.0).sqrt();
    let phi_tip = (psi_b - (t_tip - t_tip.atan())).max(0.0);

    // Root transition angle slightly outside base angle for smooth root fillet
    let phi_root = (psi_b + 0.14 * (0.5 * pitch_angle - psi_b)).min(0.46 * pitch_angle);

    if abs_phi <= phi_tip {
        tip_r
    } else if abs_phi <= psi_b {
        // Invert involute function inv(t) = t - atan(t) = psi_b - abs_phi via Newton/bisection
        let target_inv = psi_b - abs_phi;
        let mut lo = 0.0_f64;
        let mut hi = t_tip;
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            let inv_mid = mid - mid.atan();
            if inv_mid < target_inv {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let t = 0.5 * (lo + hi);
        base_r * (1.0 + t * t).sqrt()
    } else if abs_phi <= phi_root {
        // Smooth cosine fillet from base_r down to root_r
        let u = (abs_phi - psi_b) / (phi_root - psi_b).max(1e-9);
        let w = 0.5 * (1.0 + (PI * u).cos());
        root_r + (base_r - root_r) * w
    } else {
        root_r
    }
}

/// Builds a full 360° boundary-conforming 3D involute spur gear `TET4` mesh with circular shaft
/// bore `bore_radius`, exact involute tooth flanks, fine angular sampling around the engaged
/// mesh zone `engaged_angle_center`, and `nz` axial layers across `face_width`.
#[allow(clippy::too_many_arguments)]
fn build_conforming_involute_spur_gear_mesh(
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

    // Build exact tooth-feature-aligned angular stations around [0, 2*PI)
    let mut angles = Vec::new();
    for tooth in 0..num_teeth {
        let tooth_center = tooth as f64 * pitch_angle;
        // Distance to engaged region (in world angle = tooth_center + phase_offset_rad)
        let world_center = tooth_center + phase_offset_rad;
        let diff = (world_center - engaged_angle_center + PI).rem_euclid(2.0 * PI) - PI;
        let is_engaged = diff.abs() <= 1.65 * pitch_angle;

        let local_pts: Vec<f64> = if is_engaged {
            // High-resolution involute flank sampling on engaged teeth
            let mut pts = Vec::new();
            pts.push(-0.5 * pitch_angle);
            pts.push(-phi_root);
            let n_flank = 4usize;
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
            // Coarse feature-preserving sampling on idle teeth
            vec![-0.5 * pitch_angle, -phi_tip, phi_tip]
        };

        for lp in local_pts {
            let a = (tooth_center + lp).rem_euclid(2.0 * PI);
            angles.push(a);
        }
    }
    angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
    // Deduplicate angles within 1e-6 rad
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
        let r_outer = involute_gear_outer_radius(
            theta_local,
            num_teeth,
            module_m,
            pressure_angle_rad,
        );
        let theta_world = theta_local + phase_offset_rad;
        let ct = theta_world.cos();
        let st = theta_world.sin();

        for ri in 0..=n_r {
            // Cluster radial layers slightly toward the outer tooth/root rim
            let u = ri as f64 / n_r as f64;
            let r = bore_radius + (r_outer - bore_radius) * u;
            let x = center[0] + r * ct;
            let y = center[1] + r * st;
            for zi in 0..=n_z {
                let z = center[2] - 0.5 * face_width
                    + face_width * (zi as f64 / n_z as f64);
                let id = nodes.len();
                nodes.push(TetNode::new(id, [x, y, z]));
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

// ============================================================================
// Test 1: Analytical Hertz Contact of Two Elastic Cylinders (Acceptance Criterion 1)
// ============================================================================

#[test]
fn test_p5_t4_hertz_two_elastic_cylinders_benchmark() {
    // Two identical parallel steel cylinders in plane-strain line contact:
    // R1 = R2 = 50 mm, E = 205 GPa, nu = 0.29, line load P' = 400,000 N/m.
    let radius = 0.050_f64;
    let e_mod = 205e9_f64;
    let nu = 0.29_f64;
    let line_load_n_per_m = 400_000.0_f64;

    // Exact Hertz analytical solution
    let r_star = 1.0 / (1.0 / radius + 1.0 / radius); // 0.025 m
    let e_star = 1.0 / (2.0 * (1.0 - nu * nu) / e_mod); // ~111.9118 GPa
    let a_hertz = (4.0 * line_load_n_per_m * r_star / (PI * e_star)).sqrt();
    let p0_hertz = (line_load_n_per_m * e_star / (PI * r_star)).sqrt();

    // Choose slice thickness matched to fine x-cell size for isotropic, well-conditioned TET4 cells
    let fine_half_x = 1.65 * a_hertz;
    let n_fine_half_x = 22usize;
    let dx_fine = fine_half_x / n_fine_half_x as f64;
    let thickness_z = 2.0 * dx_fine;
    let applied_force_n = line_load_n_per_m * thickness_z;

    let half_width_x = 0.015_f64; // > 44 * a_hertz
    let depth_y = 0.015_f64; // > 44 * a_hertz
    let fine_depth_y = 1.4 * a_hertz;

    let mesh_bottom = build_graded_cylinder_mesh(
        radius,
        half_width_x,
        depth_y,
        thickness_z,
        fine_half_x,
        n_fine_half_x,
        12,
        10,
        10,
        fine_depth_y,
        false,
    );
    let mesh_top = build_graded_cylinder_mesh(
        radius,
        half_width_x,
        depth_y,
        thickness_z,
        fine_half_x,
        n_fine_half_x,
        12,
        10,
        10,
        fine_depth_y,
        true,
    );

    // Purely elastic steel for analytical Hertz comparison
    let steel = make_j2(e_mod, nu, 5.0e9, 0.0);

    // Bottom cylinder: plane strain (uz = 0 everywhere), fixed far-field base (y = -depth_y)
    let mut bc_bottom = BoundaryConditions::new();
    let all_bot_nodes: Vec<usize> = (0..mesh_bottom.node_count()).collect();
    bc_bottom.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: all_bot_nodes,
        components: DofComponent::Z,
    });
    bc_bottom.fix_aabb_face(AabbFace::MinY, 1e-6);

    // Top cylinder: plane strain (uz = 0 everywhere), ux = 0 on top-centerline nodes only
    // (eliminates rigid X drift without constraining contact expansion), and top far-field face
    // coupled via Prismatic HubCoupling applying downward force -applied_force_n.
    let mut bc_top = BoundaryConditions::new();
    let all_top_nodes: Vec<usize> = (0..mesh_top.node_count()).collect();
    bc_top.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: all_top_nodes,
        components: DofComponent::Z,
    });
    let top_center_nodes: Vec<usize> = mesh_top
        .nodes
        .iter()
        .filter(|n| n.position[0].abs() <= 1e-9 && (n.position[1] - depth_y).abs() <= 1e-6)
        .map(|n| n.id)
        .collect();
    assert!(!top_center_nodes.is_empty());
    bc_top.dirichlet.push(DirichletConstraint::FixedNodes {
        node_ids: top_center_nodes,
        components: DofComponent::X,
    });

    let top_hub = HubCoupling::Prismatic {
        name: Some("hertz_top_platen".to_string()),
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

    // Restrict slave candidate nodes to the top cylinder's curved lower surface near the contact zone
    let slave_contact_nodes: Vec<usize> = mesh_top
        .nodes
        .iter()
        .filter(|n| {
            let x = n.position[0];
            let sag = radius - (radius * radius - x * x).max(0.0).sqrt();
            (n.position[1] - sag).abs() <= 1e-7 && x.abs() <= 3.0 * a_hertz
        })
        .map(|n| n.id)
        .collect();

    let bodies = vec![
        ContactBody::new("cylinder_bottom", mesh_bottom, steel).with_boundary(bc_bottom),
        ContactBody::new("cylinder_top", mesh_top, steel)
            .with_boundary(bc_top)
            .with_hub_coupling(top_hub),
    ];

    let pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 1, // top cylinder slave
        body_b: 0, // bottom cylinder master
        friction_coeff: 0.0,
        symmetric: false,
        slave_a_nodes: Some(slave_contact_nodes),
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

    let report = solve_deformable_contact(&bodies, &pairs, &[ContactStepSpec::new("hertz_load")], opts)
        .expect("Hertz deformable contact solve must succeed");
    let step = report.final_step().expect("step report present");
    let patch = &step.pairs[0];

    assert!(step.converged, "Hertz contact step did not converge");
    assert!(
        step.deformed_surface_updates >= step.total_newton_iterations,
        "Deformed surface must update on every Newton iteration"
    );
    assert!(
        step.max_penetration_m < 1e-6,
        "Max penetration {:.3e} m exceeded 1e-6 m",
        step.max_penetration_m
    );

    // 1. Force balance check
    let force_err_pct =
        ((patch.total_normal_force_n - applied_force_n).abs() / applied_force_n) * 100.0;
    assert!(
        force_err_pct < 0.5,
        "Contact normal force {:.4} N vs applied {:.4} N (err {:.3}%)",
        patch.total_normal_force_n,
        applied_force_n,
        force_err_pct
    );

    // 2. Peak Hertz pressure p0 error <= 5.0%
    let p0_fem = patch.peak_normal_pressure_pa;
    let p0_err_pct = ((p0_fem - p0_hertz).abs() / p0_hertz) * 100.0;

    // 3. Geometric contact half-width a_fem from active contact boundary:
    // Average the midpoint between outermost active node and first inactive neighbor on both +/-X sides
    let max_active_x = patch
        .active_points
        .iter()
        .map(|pt| pt.position[0].abs())
        .fold(0.0_f64, f64::max);
    let a_fem = max_active_x + 0.5 * dx_fine;
    let a_err_pct = ((a_fem - a_hertz).abs() / a_hertz) * 100.0;

    // 4. Subsurface peak von Mises stress (analytical plane-strain Hertz: ~0.56 * p0 at depth ~0.70a)
    let vm_peak = step.body_max_von_mises_pa[0].max(step.body_max_von_mises_pa[1]);
    let vm_hertz_ref = 0.56 * p0_hertz;
    let vm_err_pct = ((vm_peak - vm_hertz_ref).abs() / vm_hertz_ref) * 100.0;

    println!(
        "HERTZ 2-CYLINDER BENCHMARK: p0_fem={:.2} MPa vs p0_exact={:.2} MPa (err={:.2}%), \
         a_fem={:.4} mm vs a_exact={:.4} mm (err={:.2}%), vm_peak={:.2} MPa vs {:.2} MPa (err={:.2}%), \
         max_pen={:.3e} m, al_iters={}, newton_iters={}",
        p0_fem / 1e6,
        p0_hertz / 1e6,
        p0_err_pct,
        a_fem * 1e3,
        a_hertz * 1e3,
        a_err_pct,
        vm_peak / 1e6,
        vm_hertz_ref / 1e6,
        vm_err_pct,
        step.max_penetration_m,
        step.al_iterations,
        step.total_newton_iterations,
    );

    assert!(
        p0_err_pct <= 5.0,
        "Hertz peak pressure error {:.2}% exceeds 5.0% acceptance criterion (p0_fem={:.2} MPa, p0_exact={:.2} MPa)",
        p0_err_pct,
        p0_fem / 1e6,
        p0_hertz / 1e6
    );
    assert!(
        a_err_pct <= 5.0,
        "Hertz contact half-width error {:.2}% exceeds 5.0% acceptance criterion (a_fem={:.4} mm, a_exact={:.4} mm)",
        a_err_pct,
        a_fem * 1e3,
        a_hertz * 1e3
    );
    assert!(
        vm_err_pct <= 15.0,
        "Subsurface Hertz von Mises stress peak {:.2} MPa deviates from analytical ~{:.2} MPa by {:.2}%",
        vm_peak / 1e6,
        vm_hertz_ref / 1e6,
        vm_err_pct
    );
}

// ============================================================================
// Test 2: Dry Coulomb Friction (Stick & Slip Regimes on Deformable Pair)
// ============================================================================

#[test]
fn test_p5_t4_coulomb_dry_friction_stick_and_slip() {
    let steel = make_j2(205e9, 0.29, 1.0e9, 0.0);

    // Elastic foundation block: [-0.03, 0.03] x [-0.015, 0.0] x [-0.005, 0.015]
    // Wider in both X and Z than the slider so lateral Poisson expansion stays cleanly supported.
    let foundation =
        TetrahedralMesh::from_box([-0.03, -0.015, -0.005], [0.03, 0.0, 0.015], 6, 2, 2).unwrap();
    // Sliding deformable block: [-0.01, 0.01] x [0.0, 0.01] x [0.0, 0.01]
    let slider = TetrahedralMesh::from_box([-0.01, 0.0, 0.0], [0.01, 0.01, 0.01], 4, 2, 2).unwrap();

    let mut bc_found = BoundaryConditions::new();
    bc_found.fix_aabb_face(AabbFace::MinY, 1e-6);

    // Slider: lock Z on min-Z face to prevent lateral drift out of plane, apply normal load via top platen
    let mut bc_slider_base = BoundaryConditions::new();
    bc_slider_base
        .dirichlet
        .push(DirichletConstraint::FixedAabbFace {
            face: AabbFace::MinZ,
            tolerance: 1e-6,
            components: DofComponent::Z,
        });

    let mu = 0.30_f64;
    let f_normal = 10_000.0_f64;

    let top_platen_y = HubCoupling::Prismatic {
        name: Some("normal_press".to_string()),
        axis: [0.0, 1.0, 0.0],
        selector: HubNodeSelector::AabbFace {
            face: AabbFace::MaxY,
            tolerance: 1e-6,
        },
        drive: PrismaticHubDrive::Force {
            force_n: -f_normal,
            axial_stiffness_n_per_m: 0.0,
            reference_disp_m: 0.0,
        },
        lock_transverse: false,
    };

    // Step 1: Pure normal compression (zero tangential load)
    let mut bc_step1 = bc_slider_base.clone();
    bc_step1
        .dirichlet
        .push(DirichletConstraint::FixedAabbFace {
            face: AabbFace::MaxY,
            tolerance: 1e-6,
            components: DofComponent::X,
        });
    let step1 = ContactStepSpec::new("normal_compress")
        .with_body_boundary(1, bc_step1)
        .with_body_hub_couplings(1, vec![top_platen_y.clone()]);

    // Step 2 (STICK): Apply tangential force F_x = 1500 N = 0.5 * mu * F_N (< 3000 N)
    let f_stick = 1_500.0_f64;
    let mut bc_step2 = bc_slider_base.clone();
    bc_step2.add_aabb_face_force(AabbFace::MaxY, 1e-6, [f_stick, 0.0, 0.0]);
    let step2 = ContactStepSpec::new("coulomb_stick")
        .with_body_boundary(1, bc_step2)
        .with_body_hub_couplings(1, vec![top_platen_y.clone()]);

    // Step 3 (SLIP): Prescribe horizontal drag u_x = +50 um (>25x the ~1.9 um elastic shear
    // slip threshold) on top face while holding F_N = 10,000 N so all points enter full Coulomb slip
    let mut bc_step3 = bc_slider_base.clone();
    for node in &slider.nodes {
        if (node.position[1] - 0.01).abs() <= 1e-6 {
            bc_step3.dirichlet.push(DirichletConstraint::FixedDof {
                dof_index: 3 * node.id,
                prescribed_value: 5e-5,
            });
        }
    }
    let step3 = ContactStepSpec::new("coulomb_slip")
        .with_body_boundary(1, bc_step3)
        .with_body_hub_couplings(1, vec![top_platen_y.clone()]);

    let slider_bottom_nodes: Vec<usize> = slider
        .nodes
        .iter()
        .filter(|n| n.position[1].abs() <= 1e-6)
        .map(|n| n.id)
        .collect();

    let bodies = vec![
        ContactBody::new("foundation", foundation, steel).with_boundary(bc_found),
        ContactBody::new("slider", slider, steel)
            .with_boundary(bc_slider_base)
            .with_hub_coupling(top_platen_y),
    ];

    let pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 1,
        body_b: 0,
        friction_coeff: mu,
        symmetric: false,
        slave_a_nodes: Some(slider_bottom_nodes),
        slave_b_nodes: None,
        search_distance: Some(5e-3),
        max_normal_dot: Some(0.25),
    }];

    let opts = ContactSolverOptions {
        max_penetration_tol: 5e-7,
        al_multiplier_tol: 1e-3,
        max_al_iterations: 25,
        newton_rel_tol: 1e-6,
        penalty_scale: 4.0,
        ..ContactSolverOptions::default()
    };

    let report = solve_deformable_contact(&bodies, &pairs, &[step1, step2, step3], opts)
        .expect("Coulomb friction solve must succeed");

    // Verify Step 1 (normal compression)
    let s1 = &report.steps[0];
    assert!(s1.converged);
    assert!(s1.max_penetration_m < 1e-6);
    assert!(
        (s1.pairs[0].total_normal_force_n - f_normal).abs() / f_normal < 0.01,
        "Step 1 normal force {:.2} N vs {:.2} N",
        s1.pairs[0].total_normal_force_n,
        f_normal
    );

    // Verify Step 2 (STICK regime: F_x = 1500 N < mu * F_N = 3000 N)
    let s2 = &report.steps[1];
    assert!(s2.converged);
    assert!(s2.max_penetration_m < 1e-6);
    let s2_stick_count = s2.pairs[0]
        .active_point_count
        .saturating_sub(s2.pairs[0].slipping_point_count);
    assert!(
        s2_stick_count > 0,
        "Core contact points must remain in strict stick in Step 2 (stick={}, slip={})",
        s2_stick_count,
        s2.pairs[0].slipping_point_count
    );
    // Every active contact point must strictly obey the dry Coulomb cone ||t_T|| <= mu * p_n
    for pt in &s2.pairs[0].active_points {
        assert!(
            pt.tangential_stress_pa <= mu * pt.normal_pressure_pa * (1.0 + 1e-3) + 1e-3,
            "Coulomb cone violated in Step 2: |t_T|={:.2} Pa > mu*p_n={:.2} Pa",
            pt.tangential_stress_pa,
            mu * pt.normal_pressure_pa
        );
    }
    let ft_stick = s2.pairs[0].resultant_tangential_force_vec_n[0];
    assert!(
        (ft_stick + f_stick).abs() / f_stick < 0.01,
        "Stick friction force on slider ({:.2} N) must oppose +1500 N external load",
        ft_stick
    );

    // Verify Step 3 (SLIP regime: ||F_T|| = mu * F_N = 3000 N)
    let s3 = &report.steps[2];
    assert!(s3.converged);
    assert!(s3.max_penetration_m < 1e-6);
    assert!(
        s3.pairs[0].slipping_point_count == s3.pairs[0].active_point_count,
        "All active contact points must enter full Coulomb slip under 50 um drag (slip={} vs active={})",
        s3.pairs[0].slipping_point_count,
        s3.pairs[0].active_point_count
    );
    for pt in &s3.pairs[0].active_points {
        assert!(
            pt.tangential_stress_pa <= mu * pt.normal_pressure_pa * (1.0 + 1e-3) + 1e-3,
            "Coulomb cone violated in Step 3: |t_T|={:.2} Pa > mu*p_n={:.2} Pa",
            pt.tangential_stress_pa,
            mu * pt.normal_pressure_pa
        );
    }
    let expected_slip_force = mu * f_normal;
    let ft_slip = s3.pairs[0].resultant_tangential_force_vec_n[0];
    let slip_err_pct = ((ft_slip.abs() - expected_slip_force).abs() / expected_slip_force) * 100.0;
    println!(
        "COULOMB FRICTION: stick(stick_pts={}, slip_pts={}, Fx={:.2} N vs -{:.2} N), \
         slip(slip_pts={}/{}, Fx={:.2} N vs -{:.2} N, err={:.3}%), max_pen={:.3e} m",
        s2_stick_count,
        s2.pairs[0].slipping_point_count,
        ft_stick,
        f_stick,
        s3.pairs[0].slipping_point_count,
        s3.pairs[0].active_point_count,
        ft_slip,
        expected_slip_force,
        slip_err_pct,
        s3.max_penetration_m
    );
    assert!(
        ft_slip < 0.0 && slip_err_pct < 1.0,
        "Coulomb slip force {:.2} N vs expected -{:.2} N (err {:.3}%)",
        ft_slip,
        expected_slip_force,
        slip_err_pct
    );
}

// ============================================================================
// Test 3: Two Meshing Deformable Involute Spur Gears Under Rotation & Load
//         (Acceptance Criterion 2)
// ============================================================================

#[test]
fn test_p5_t4_two_meshing_deformable_spur_gears_under_load() {
    // Interference-free AGMA 25-degree involute spur gear pair:
    // z_min = 2 / sin^2(25 deg) = 11.2 < z = 14 => zero undercut / tip-root interference,
    // contact ratio eps_alpha = 1.35 (true involute meshing along the line of action).
    let module_m = 0.004_f64; // 4 mm module
    let num_teeth = 14usize;
    let pressure_angle = 25.0_f64.to_radians();
    let pitch_r = 0.5 * module_m * num_teeth as f64; // 28 mm
    let center_dist = 2.0 * pitch_r; // 56 mm
    let bore_r = 0.012_f64; // 12 mm shaft bore
    let face_width = 0.006_f64; // 6 mm face width

    // Gear 1 centered at (0, 0, 0), phased by -psi_p so Tooth 0's upper driving involute flank
    // passes EXACTLY through the pitch point (pitch_r, 0) at theta1 = 0.
    let psi_p = PI / (2.0 * num_teeth as f64);
    let gear1_mesh = build_conforming_involute_spur_gear_mesh(
        [0.0, 0.0, 0.0],
        -psi_p,
        num_teeth,
        module_m,
        pressure_angle,
        bore_r,
        face_width,
        0.0,
        2,
        2,
    );
    // Gear 2 centered at (56 mm, 0, 0), phased by PI - psi_p so Tooth 0's lower driven involute
    // flank passes EXACTLY through the pitch point (pitch_r, 0) opposing Gear 1 (n1 · n2 = -1).
    let gear2_mesh = build_conforming_involute_spur_gear_mesh(
        [center_dist, 0.0, 0.0],
        PI - psi_p,
        num_teeth,
        module_m,
        pressure_angle,
        bore_r,
        face_width,
        PI,
        2,
        2,
    );

    let gear_steel = make_j2(205e9, 0.29, 750e6, 3.0e9);

    // Identify engaged-zone exterior tooth nodes near the meshing line of action
    let gear1_engaged_nodes: Vec<usize> = gear1_mesh
        .nodes
        .iter()
        .filter(|n| {
            let r = (n.position[0].powi(2) + n.position[1].powi(2)).sqrt();
            r > bore_r + 0.008 && n.position[0] > 0.016 && n.position[1].abs() < 0.020
        })
        .map(|n| n.id)
        .collect();
    let gear2_engaged_nodes: Vec<usize> = gear2_mesh
        .nodes
        .iter()
        .filter(|n| {
            let dx = n.position[0] - center_dist;
            let dy = n.position[1];
            let r = (dx * dx + dy * dy).sqrt();
            r > bore_r + 0.008 && dx < -0.016 && dy.abs() < 0.020
        })
        .map(|n| n.id)
        .collect();

    assert!(!gear1_engaged_nodes.is_empty() && !gear2_engaged_nodes.is_empty());

    // Shaft bores coupled via HubCoupling::Revolute (all gear teeth remain 100% free!)
    let gear1_bore = HubNodeSelector::BoreCylinder {
        pivot: [0.0, 0.0, 0.0],
        axis: [0.0, 0.0, 1.0],
        max_radius: bore_r + 1e-4,
    };
    let gear2_bore = HubNodeSelector::BoreCylinder {
        pivot: [center_dist, 0.0, 0.0],
        axis: [0.0, 0.0, 1.0],
        max_radius: bore_r + 1e-4,
    };

    // Gear 2 resists rotation via a coupled torsional load on its shaft bore
    // (purely elastic torsional load + constant resistive torque, zero artificial gluing of teeth!)
    let gear2_hub = HubCoupling::Revolute {
        name: Some("driven_gear_shaft".to_string()),
        pivot: [center_dist, 0.0, 0.0],
        axis: [0.0, 0.0, 1.0],
        selector: gear2_bore,
        drive: RevoluteHubDrive::Torque {
            torque_nm: 8.0, // opposes CW (negative) rotation of driven gear
            torsional_stiffness_nm_per_rad: 600.0, // torsional shaft load resisting CW motion
            reference_angle_rad: 0.0,
        },
    };

    let bodies = vec![
        ContactBody::new("gear_driver", gear1_mesh, gear_steel),
        ContactBody::new("gear_driven", gear2_mesh, gear_steel).with_hub_coupling(gear2_hub.clone()),
    ];

    // Two-pass symmetric deformable-to-deformable contact with realistic lubricated steel friction mu = 0.08
    let pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 0,
        body_b: 1,
        friction_coeff: 0.08,
        symmetric: true,
        slave_a_nodes: Some(gear1_engaged_nodes),
        slave_b_nodes: Some(gear2_engaged_nodes),
        search_distance: Some(0.004),
        max_normal_dot: Some(0.25),
    }];

    // Drive Gear 1 CCW through 4 progressive rotation steps under load along the line of action
    let driver_angles = [0.003_f64, 0.007_f64, 0.012_f64, 0.018_f64];
    let steps: Vec<ContactStepSpec> = driver_angles
        .iter()
        .enumerate()
        .map(|(i, &theta)| {
            let driver_hub = HubCoupling::Revolute {
                name: Some("driver_gear_shaft".to_string()),
                pivot: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                selector: gear1_bore.clone(),
                drive: RevoluteHubDrive::PrescribedAngle { angle_rad: theta },
            };
            ContactStepSpec::new(format!("gear_step_{}", i + 1))
                .with_body_hub_couplings(0, vec![driver_hub])
                .with_body_hub_couplings(1, vec![gear2_hub.clone()])
        })
        .collect();

    let opts = ContactSolverOptions {
        discretization: ContactDiscretization::NodeToSurface,
        corotated: true,
        include_geometric_stiffness: true,
        max_penetration_tol: 2e-6, // 5x stricter than 1e-5 m acceptance criterion
        al_multiplier_tol: 1e-2,
        max_al_iterations: 12,
        newton_rel_tol: 1e-4,
        newton_abs_tol: 1e-3,
        max_newton_iterations: 15,
        penalty_scale: 4.0,
        linear_solver: mcp_cad_fea::SolverOptions {
            tolerance: 1e-6,
            max_iterations: 3000,
        },
        ..ContactSolverOptions::default()
    };

    let report = solve_deformable_contact(&bodies, &pairs, &steps, opts)
        .expect("Two-gear deformable contact solve must succeed");

    let mut prev_driven_angle = 0.0_f64;
    for (idx, step) in report.steps.iter().enumerate() {
        let patch = &step.pairs[0];
        let driver_hub_res = &step.hubs[0];
        let driven_hub_res = &step.hubs[1];

        println!(
            "GEAR STEP {}: theta1={:.4} rad, theta2={:.4} rad, T1={:.2} N·m, T2_react={:.2} N·m, \
             active_pts={}, p_max={:.2} MPa, Fn={:.1} N, max_pen={:.3e} m, vm1={:.1} MPa, surf_upd={}",
            idx + 1,
            driver_hub_res.coordinate,
            driven_hub_res.coordinate,
            driver_hub_res.reaction_effort,
            driven_hub_res.reaction_effort,
            patch.active_point_count,
            patch.peak_normal_pressure_pa / 1e6,
            patch.total_normal_force_n,
            step.max_penetration_m,
            step.body_max_von_mises_pa[0] / 1e6,
            step.deformed_surface_updates,
        );

        // Acceptance Criterion 2a: strict interpenetration < 1e-5 m on EVERY step
        assert!(
            step.max_penetration_m < 1e-5,
            "Gear step {} interpenetration {:.3e} m exceeded 1e-5 m tolerance!",
            idx + 1,
            step.max_penetration_m
        );

        // Deformed surfaces updated on every Newton iteration
        assert!(
            step.deformed_surface_updates >= step.total_newton_iterations,
            "Gear step {} must update deformed surfaces on every Newton iteration",
            idx + 1
        );

        // Acceptance Criterion 2b: physically realistic contact pressure along the line of contact
        assert!(
            patch.active_point_count >= 3,
            "Gear step {} expected multi-node line of contact, got {} active points",
            idx + 1,
            patch.active_point_count
        );
        assert!(
            patch.peak_normal_pressure_pa > 5.0e6,
            "Gear step {} contact pressure {:.2} MPa is unrealistically low",
            idx + 1,
            patch.peak_normal_pressure_pa / 1e6
        );
        assert!(
            patch.total_normal_force_n > 100.0,
            "Gear step {} contact force {:.2} N is too small under load",
            idx + 1,
            patch.total_normal_force_n
        );

        // Contact points span across the 3D face width (line of contact in Z)
        let z_min = patch
            .active_points
            .iter()
            .map(|p| p.position[2])
            .fold(f64::INFINITY, f64::min);
        let z_max = patch
            .active_points
            .iter()
            .map(|p| p.position[2])
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            z_max - z_min >= 0.45 * face_width,
            "Gear step {} contact points must span face width (z span = {:.4} m vs b = {:.4} m)",
            idx + 1,
            z_max - z_min,
            face_width
        );

        // Driven gear rotates monotonically CW (negative) under CCW driver motion
        assert!(
            driven_hub_res.coordinate < prev_driven_angle,
            "Driven gear must rotate CW (step {}: {:.5} rad vs prev {:.5} rad)",
            idx + 1,
            driven_hub_res.coordinate,
            prev_driven_angle
        );
        prev_driven_angle = driven_hub_res.coordinate;

        // Driver exerts positive CCW torque transmitting load through the teeth
        assert!(
            driver_hub_res.reaction_effort > 5.0,
            "Driver shaft must transmit positive torque (got {:.2} N·m)",
            driver_hub_res.reaction_effort
        );
    }

    assert!(
        report.worst_penetration_all_steps_m() < 1e-5,
        "Worst gear penetration across all steps {:.3e} m exceeded 1e-5 m",
        report.worst_penetration_all_steps_m()
    );
}

// ============================================================================
// Test 4: Elastoplastic Indentation & Deformed Surface Geometry Feedback
// ============================================================================

#[test]
fn test_p5_t4_plastic_indentation_updates_deformed_contact_surface() {
    // Soft elastoplastic aluminum substrate indented by a stiff steel punch
    let soft_al = make_j2(70e9, 0.33, 150e6, 1.5e9);
    let stiff_steel = make_j2(210e9, 0.29, 2.5e9, 0.0);

    let substrate =
        TetrahedralMesh::from_box([-0.020, -0.012, 0.0], [0.020, 0.0, 0.010], 8, 3, 2).unwrap();
    let punch =
        TetrahedralMesh::from_box([-0.005, 0.0, 0.0], [0.005, 0.010, 0.010], 3, 2, 2).unwrap();

    let mut bc_sub = BoundaryConditions::new();
    bc_sub.fix_aabb_face(AabbFace::MinY, 1e-6);

    // Step 1: Deep indentation beyond yield (-0.45 mm)
    let step_indent = ContactStepSpec::new("plastic_indent").with_body_hub_couplings(
        1,
        vec![HubCoupling::Prismatic {
            name: Some("punch_drive".to_string()),
            axis: [0.0, 1.0, 0.0],
            selector: HubNodeSelector::AabbFace {
                face: AabbFace::MaxY,
                tolerance: 1e-6,
            },
            drive: PrismaticHubDrive::PrescribedDisplacement {
                displacement_m: -4.5e-4,
            },
            lock_transverse: true,
        }],
    );

    // Step 2: Retract punch (+1.0 mm) to expose permanent plastic crater on deformed surface
    let step_unload = ContactStepSpec::new("retract_punch").with_body_hub_couplings(
        1,
        vec![HubCoupling::Prismatic {
            name: Some("punch_drive".to_string()),
            axis: [0.0, 1.0, 0.0],
            selector: HubNodeSelector::AabbFace {
                face: AabbFace::MaxY,
                tolerance: 1e-6,
            },
            drive: PrismaticHubDrive::PrescribedDisplacement {
                displacement_m: 1.0e-3,
            },
            lock_transverse: true,
        }],
    );

    let punch_bottom: Vec<usize> = punch
        .nodes
        .iter()
        .filter(|n| n.position[1].abs() <= 1e-6)
        .map(|n| n.id)
        .collect();

    let bodies = vec![
        ContactBody::new("substrate", substrate.clone(), soft_al).with_boundary(bc_sub),
        ContactBody::new("punch", punch, stiff_steel),
    ];

    let pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 1,
        body_b: 0,
        friction_coeff: 0.1,
        symmetric: false,
        slave_a_nodes: Some(punch_bottom),
        slave_b_nodes: None,
        search_distance: Some(5e-3),
        max_normal_dot: Some(0.25),
    }];

    let opts = ContactSolverOptions {
        max_penetration_tol: 1e-6,
        al_multiplier_tol: 2e-3,
        max_al_iterations: 20,
        ..ContactSolverOptions::default()
    };

    let report = solve_deformable_contact(&bodies, &pairs, &[step_indent, step_unload], opts)
        .expect("Elastoplastic indentation contact solve must succeed");

    let s_indent = &report.steps[0];
    let s_unload = &report.steps[1];

    assert!(s_indent.converged && s_unload.converged);
    assert!(s_indent.max_penetration_m < 1e-5);
    assert!(
        s_indent.body_max_equiv_plastic_strain[0] > 1e-3,
        "Substrate must yield plastically under punch indentation (ep={:.3e})",
        s_indent.body_max_equiv_plastic_strain[0]
    );
    assert_eq!(
        s_unload.pairs[0].active_point_count, 0,
        "Punch must fully separate on retraction"
    );

    // Check permanent crater depth on the unloaded deformed surface mesh
    let mut min_top_y = f64::INFINITY;
    for node in &substrate.nodes {
        if node.position[1].abs() <= 1e-6 && node.position[0].abs() <= 0.005 + 1e-6 {
            let def_y = report.body_deformed_positions[0][node.id][1];
            min_top_y = min_top_y.min(def_y);
        }
    }
    println!(
        "PLASTIC INDENTATION: max_ep={:.4e}, residual_crater_y={:.4} mm, max_pen={:.3e} m",
        s_unload.body_max_equiv_plastic_strain[0],
        min_top_y * 1e3,
        s_indent.max_penetration_m
    );
    assert!(
        min_top_y < -5e-5,
        "Unloaded deformed contact surface must retain permanent plastic indentation crater (y={:.3e} m)",
        min_top_y
    );
}

// ============================================================================
// Test 5: Quadratic TET10 Mortar Gauss-Point Contact Verification
// ============================================================================

#[test]
fn test_p5_t4_tet10_mortar_contact_smooth_pressure() {
    let steel = make_j2(200e9, 0.30, 1.0e9, 0.0);

    let base_tet4 =
        TetrahedralMesh::from_box([-0.02, -0.01, 0.0], [0.02, 0.0, 0.01], 4, 2, 1).unwrap();
    let top_tet4 =
        TetrahedralMesh::from_box([-0.01, 0.0, 0.0], [0.01, 0.01, 0.01], 2, 1, 1).unwrap();
    let base_tet10 = base_tet4.to_tet10();
    let top_tet10 = top_tet4.to_tet10();

    let mut bc_base = BoundaryConditions::new();
    bc_base.fix_aabb_face(AabbFace::MinY, 1e-6);

    let f_press = 8_000.0_f64;
    let mut bc_top = BoundaryConditions::new();
    bc_top
        .dirichlet
        .push(DirichletConstraint::FixedAabbFace {
            face: AabbFace::MaxY,
            tolerance: 1e-6,
            components: DofComponent::XZ,
        });

    let top_hub = HubCoupling::Prismatic {
        name: Some("tet10_platen".to_string()),
        axis: [0.0, 1.0, 0.0],
        selector: HubNodeSelector::AabbFace {
            face: AabbFace::MaxY,
            tolerance: 1e-6,
        },
        drive: PrismaticHubDrive::Force {
            force_n: -f_press,
            axial_stiffness_n_per_m: 0.0,
            reference_disp_m: 0.0,
        },
        lock_transverse: false,
    };

    let top_bottom_nodes: Vec<usize> = top_tet10
        .nodes
        .iter()
        .filter(|n| n.position[1].abs() <= 1e-6)
        .map(|n| n.id)
        .collect();

    let bodies = vec![
        ContactBody::new("base_tet10", base_tet10, steel).with_boundary(bc_base),
        ContactBody::new("top_tet10", top_tet10, steel)
            .with_boundary(bc_top)
            .with_hub_coupling(top_hub),
    ];

    let pairs = vec![ContactPairSpec::DeformablePair {
        body_a: 1,
        body_b: 0,
        friction_coeff: 0.0,
        symmetric: false,
        slave_a_nodes: Some(top_bottom_nodes),
        slave_b_nodes: None,
        search_distance: Some(5e-3),
        max_normal_dot: Some(0.25),
    }];

    // Auto selects MortarGauss for TET10
    let opts = ContactSolverOptions {
        discretization: ContactDiscretization::Auto,
        max_penetration_tol: 1e-6,
        al_multiplier_tol: 2e-3,
        max_al_iterations: 20,
        ..ContactSolverOptions::default()
    };

    let report =
        solve_deformable_contact(&bodies, &pairs, &[ContactStepSpec::new("tet10_load")], opts)
            .expect("TET10 Mortar contact solve must succeed");
    let step = report.final_step().unwrap();
    let patch = &step.pairs[0];

    println!(
        "TET10 MORTAR CONTACT: converged={}, al_iters={}, newton_iters={}, max_pen={:.3e} m, active={}, fn={:.2} N (target {:.2} N), p_peak={:.2} MPa",
        step.converged,
        step.al_iterations,
        step.total_newton_iterations,
        step.max_penetration_m,
        patch.active_point_count,
        patch.total_normal_force_n,
        f_press,
        patch.peak_normal_pressure_pa / 1e6,
    );
    assert!(step.converged, "TET10 Mortar contact step must converge");
    assert!(step.max_penetration_m < 1e-6);
    assert!(
        (patch.total_normal_force_n - f_press).abs() / f_press < 0.02,
        "TET10 Mortar contact force {:.2} N vs applied {:.2} N",
        patch.total_normal_force_n,
        f_press
    );
    // Average contact pressure = 8000 N / (0.02 * 0.01 m²) = 40 MPa; peak shouldn't wildly oscillate
    let p_avg = f_press / (0.02 * 0.01);
    assert!(
        patch.peak_normal_pressure_pa > 0.8 * p_avg && patch.peak_normal_pressure_pa < 2.2 * p_avg,
        "TET10 Mortar peak pressure {:.2} MPa should be smooth near {:.2} MPa",
        patch.peak_normal_pressure_pa / 1e6,
        p_avg / 1e6
    );
}
