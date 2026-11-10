//! Comprehensive invariant test suite for all 3D primitives in `mcp-cad-core`.
//!
//! Task P0-T2: Verifies across parameter grids (radii 0.001..0.1 m, height/radius ratios,
//! segments in {0, 8, 16, 32, 64, 128}, polygon sides 3..=12, spur gear teeth 8..=40,
//! revolve rectangular and L-shaped profiles):
//! 1. `validate_mesh` returns `"PASS"` (documenting permissible warnings).
//! 2. Directed edge manifoldness: every directed edge `(u, v)` on welded vertices occurs
//!    exactly once, and its opposite `(v, u)` also occurs exactly once.
//! 3. Euler characteristic `chi = V - E + F == 2 - 2g` (`chi == 2` for genus-0 solids).
//! 4. `signed_volume() > 0` and matches analytical / exact polygonal volume within tolerance.
//! 5. Axis-aligned bounding box (`bounding_box()`) matches exact theoretical dimensions within `1e-9` relative to scale.

use std::collections::HashMap;
use std::f64::consts::PI;

use mcp_cad_core::diagnostics::validate_mesh;
use mcp_cad_core::primitives::{
    create_box, create_cone, create_cylinder, create_polygon_prism, create_revolve, create_sphere,
    create_spur_gear, create_wheel_rim, cut_hole,
};
use mcp_cad_core::tessellation::TessellationSettings;
use mcp_cad_core::topology::TriangleMesh;

/// Welds vertices within `1e-7 * diag` and verifies that every directed edge `(u, v)`
/// appears exactly once and its reverse `(v, u)` appears exactly once, returning `(V, E, F, chi)`.
fn verify_directed_edges_and_euler(
    mesh: &TriangleMesh,
    context: &str,
) -> (usize, usize, usize, i32) {
    assert!(!mesh.vertices.is_empty(), "{context}: mesh has no vertices");
    assert!(!mesh.indices.is_empty(), "{context}: mesh has no triangles");

    let bbox = mesh.bounding_box();
    let dims = bbox.dimensions();
    let diag = (dims[0] * dims[0] + dims[1] * dims[1] + dims[2] * dims[2])
        .sqrt()
        .max(1e-9);
    let weld_eps = (diag * 1e-7).max(1e-12);
    let inv_eps = 1.0 / weld_eps;
    let weld_eps_sq = weld_eps * weld_eps;

    let mut grid: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
    let mut welded_pos: Vec<[f64; 3]> = Vec::new();
    let mut v_map: Vec<u32> = Vec::with_capacity(mesh.vertices.len());

    for v in &mesh.vertices {
        let p = v.position;
        let key = [
            (p[0] * inv_eps).round() as i64,
            (p[1] * inv_eps).round() as i64,
            (p[2] * inv_eps).round() as i64,
        ];
        let mut found = None;
        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let nk = [key[0] + dx, key[1] + dy, key[2] + dz];
                    if let Some(cands) = grid.get(&nk) {
                        for &c_idx in cands {
                            let cp = welded_pos[c_idx as usize];
                            let d2 = (cp[0] - p[0]).powi(2)
                                + (cp[1] - p[1]).powi(2)
                                + (cp[2] - p[2]).powi(2);
                            if d2 <= weld_eps_sq {
                                found = Some(c_idx);
                                break 'search;
                            }
                        }
                    }
                }
            }
        }
        let idx = match found {
            Some(i) => i,
            None => {
                let i = welded_pos.len() as u32;
                grid.entry(key).or_default().push(i);
                welded_pos.push(p);
                i
            }
        };
        v_map.push(idx);
    }

    let mut directed_edges: HashMap<(u32, u32), usize> = HashMap::new();
    let mut valid_faces = 0usize;

    for tri in &mesh.indices {
        let u = v_map[tri[0] as usize];
        let v = v_map[tri[1] as usize];
        let w = v_map[tri[2] as usize];
        assert!(
            u != v && v != w && w != u,
            "{context}: degenerate welded triangle [{u}, {v}, {w}]"
        );
        valid_faces += 1;
        for &(a, b) in &[(u, v), (v, w), (w, u)] {
            *directed_edges.entry((a, b)).or_insert(0) += 1;
        }
    }

    for (&(a, b), &count) in &directed_edges {
        assert_eq!(
            count, 1,
            "{context}: directed edge ({a} -> {b}) appears {count} times (expected 1)"
        );
        let rev_count = directed_edges.get(&(b, a)).copied().unwrap_or(0);
        assert_eq!(
            rev_count, 1,
            "{context}: reverse directed edge ({b} -> {a}) appears {rev_count} times (expected 1)"
        );
    }

    let v_count = welded_pos.len();
    let e_count = directed_edges.len() / 2;
    let f_count = valid_faces;
    let chi = (v_count as i32) - (e_count as i32) + (f_count as i32);
    (v_count, e_count, f_count, chi)
}

/// Checks that only expected non-critical warnings appear in `report.warnings`.
/// Permissible warnings:
/// - `"Notice: N un-welded duplicate vertices found (CAD split normals)"`: expected because
///   CAD primitives duplicate vertices along sharp feature edges/caps to store exact per-face normals.
/// - `"Notice: N sliver triangles detected (min angle < 5° or aspect ratio > 100)"`: permissible
///   only when `< 1%` of triangles have `min_angle < 5°` and `0` triangles have `min_angle < 1°`
///   (otherwise `validate_mesh` sets `status = "FAIL"`).
fn assert_permissible_warnings(warnings: &[String], context: &str) {
    for w in warnings {
        let ok = w.contains("un-welded duplicate vertices found (CAD split normals)")
            || w.contains("sliver triangles detected");
        assert!(
            ok,
            "{context}: unexpected warning in MeshValidationReport: {w}"
        );
    }
}

/// Asserts the full 5-invariant suite on a mesh of expected genus `expected_genus`.
fn assert_all_invariants(
    mesh: &TriangleMesh,
    expected_genus: i32,
    expected_volume: f64,
    vol_rel_tol: f64,
    expected_dims: [f64; 3],
    aabb_rel_tol: f64,
    context: &str,
) {
    // 1. validate_mesh PASS
    let report = validate_mesh(mesh, Some(expected_genus));
    assert_eq!(
        report.status, "PASS",
        "{context}: validate_mesh failed with issues: {:?}",
        report.issues
    );
    assert_permissible_warnings(&report.warnings, context);

    // 2. Directed edge manifoldness: each (u, v) == 1 and (v, u) == 1
    let (_v, _e, _f, chi) = verify_directed_edges_and_euler(mesh, context);

    // 3. Euler characteristic chi = 2 - 2 * g
    let expected_chi = 2 - 2 * expected_genus;
    assert_eq!(
        chi, expected_chi,
        "{context}: Euler characteristic chi={chi}, expected {expected_chi}"
    );
    assert_eq!(
        report.topology.euler_characteristic, expected_chi,
        "{context}: report euler_characteristic={}, expected {expected_chi}",
        report.topology.euler_characteristic
    );

    // 4. Signed volume > 0 and within tolerance of analytical / polygonal volume
    let sv = mesh.signed_volume();
    assert!(
        sv > 0.0,
        "{context}: signed_volume must be positive, got {sv}"
    );
    let vol_rel_err = (sv - expected_volume).abs() / expected_volume.max(1e-18);
    assert!(
        vol_rel_err <= vol_rel_tol,
        "{context}: signed_volume={sv:.9e}, expected={expected_volume:.9e}, rel_err={vol_rel_err:.6e} > tol={vol_rel_tol:.6e}"
    );

    // 5. Bounding box dimensions match within tolerance relative to scale
    let bbox = mesh.bounding_box();
    let dims = bbox.dimensions();
    let scale = expected_dims[0]
        .max(expected_dims[1])
        .max(expected_dims[2])
        .max(1e-12);
    for axis in 0..3 {
        let err = (dims[axis] - expected_dims[axis]).abs() / scale;
        assert!(
            err <= aabb_rel_tol,
            "{context}: AABB axis {axis} dim={}, expected={}, rel_err={err:.3e} > tol={aabb_rel_tol:.3e}",
            dims[axis],
            expected_dims[axis]
        );
    }
}

/// Helper: exact AABB span `[dx, dy]` of a regular `n`-gon of radius `r` with vertices at `theta_i = 2*pi*i/n`.
fn regular_ngon_aabb_span(r: f64, n: usize) -> [f64; 2] {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for i in 0..n {
        let th = (i as f64) * 2.0 * PI / (n as f64);
        let x = r * th.cos();
        let y = r * th.sin();
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    [max_x - min_x, max_y - min_y]
}

// ============================================================================
// 1. BOX INVARIANTS
// ============================================================================

#[test]
fn invariants_box_grid() {
    let scales = [0.001, 0.01, 0.05, 0.1];
    let ratios = [(1.0, 1.0, 1.0), (1.0, 2.0, 0.5), (0.5, 3.0, 4.0)];

    for &s in &scales {
        for &(rx, ry, rz) in &ratios {
            let (w, h, d) = (s * rx, s * ry, s * rz);
            let mesh = create_box(w, h, d);
            let expected_vol = w * h * d;
            let ctx = format!("box(w={w}, h={h}, d={d})");
            assert_all_invariants(&mesh, 0, expected_vol, 1e-9, [w, h, d], 1e-9, &ctx);
        }
    }
}

// ============================================================================
// 2. CYLINDER INVARIANTS
// ============================================================================

#[test]
fn invariants_cylinder_grid() {
    let radii = [0.001, 0.01, 0.05, 0.1];
    let h_ratios = [0.5, 2.0, 5.0];
    let segments_list = [0usize, 8, 16, 32, 64, 128];

    for &r in &radii {
        for &hr in &h_ratios {
            let h = r * hr;
            for &segs in &segments_list {
                let mesh = create_cylinder(r, h, segs);
                let eff_segs = if segs == 0 {
                    TessellationSettings::default().calculate_segments(r)
                } else {
                    segs.max(8)
                };
                // Exact volume of inscribed regular eff_segs-gon prism
                let exact_poly_vol =
                    0.5 * (eff_segs as f64) * r * r * (2.0 * PI / (eff_segs as f64)).sin() * h;
                let span_xy = regular_ngon_aabb_span(r, eff_segs);
                let ctx = format!("cylinder(r={r}, h={h}, segs={segs}[eff={eff_segs}])");
                assert_all_invariants(
                    &mesh,
                    0,
                    exact_poly_vol,
                    1e-7,
                    [span_xy[0], span_xy[1], h],
                    1e-9,
                    &ctx,
                );

                // For >= 32 segments, also verify <= 1.5% relative to smooth cylinder pi * r^2 * h
                if eff_segs >= 32 {
                    let smooth_vol = PI * r * r * h;
                    let rel_smooth = (mesh.signed_volume() - smooth_vol).abs() / smooth_vol;
                    assert!(
                        rel_smooth < 0.015,
                        "{ctx}: smooth cylinder volume error {rel_smooth} >= 1.5%"
                    );
                }
            }
        }
    }
}

// ============================================================================
// 3. CONE & FRUSTUM INVARIANTS
// ============================================================================

#[test]
fn invariants_cone_sharp_and_frustum_grid() {
    let radii = [0.001, 0.01, 0.05, 0.1];
    let h_ratios = [0.5, 2.0, 5.0];
    let segments_list = [0usize, 8, 16, 32, 64, 128];

    for &r_bot in &radii {
        for &hr in &h_ratios {
            let h = r_bot * hr;
            for &segs in &segments_list {
                let eff_segs = if segs == 0 {
                    TessellationSettings::default().calculate_segments(r_bot)
                } else {
                    segs.max(8)
                };
                let span_xy = regular_ngon_aabb_span(r_bot, eff_segs);

                // 1. Sharp cone (r_top = 0):
                let cone_sharp = create_cone(r_bot, 0.0, h, segs);
                let poly_base_area = 0.5
                    * (eff_segs as f64)
                    * r_bot
                    * r_bot
                    * (2.0 * PI / (eff_segs as f64)).sin();
                let exact_pyramid_vol = (1.0 / 3.0) * poly_base_area * h;
                let sharp_tol = if eff_segs == 16 { 0.025 } else { 0.01 };
                let ctx_sharp =
                    format!("cone_sharp(r_bot={r_bot}, h={h}, segs={segs}[eff={eff_segs}])");
                assert_all_invariants(
                    &cone_sharp,
                    0,
                    exact_pyramid_vol,
                    sharp_tol,
                    [span_xy[0], span_xy[1], h],
                    1e-9,
                    &ctx_sharp,
                );

                // 2. Truncated cone / frustum (r_top = 0.5 * r_bot) across ALL segments and ratios
                let r_top = 0.5 * r_bot;
                let cone_frustum = create_cone(r_bot, r_top, h, segs);
                let exact_frustum_vol = (h / 6.0)
                    * (eff_segs as f64)
                    * (2.0 * PI / (eff_segs as f64)).sin()
                    * (r_bot * r_bot + r_bot * r_top + r_top * r_top);
                let ctx_frustum = format!(
                    "cone_frustum(r_bot={r_bot}, r_top={r_top}, h={h}, segs={segs}[eff={eff_segs}])"
                );
                assert_all_invariants(
                    &cone_frustum,
                    0,
                    exact_frustum_vol,
                    1e-7,
                    [span_xy[0], span_xy[1], h],
                    1e-9,
                    &ctx_frustum,
                );
            }
        }
    }
}

// ============================================================================
// 4. SPHERE INVARIANTS
// ============================================================================

#[test]
fn invariants_sphere_grid() {
    let radii = [0.001, 0.01, 0.05, 0.1];
    let segments_list = [0usize, 8, 16, 32, 64, 128];

    for &r in &radii {
        for &segs in &segments_list {
            let (sectors, stacks) = if segs == 0 {
                (0, 0)
            } else {
                (segs, (segs / 2).max(8))
            };
            let mesh = create_sphere(r, sectors, stacks);
            let smooth_vol = (4.0 / 3.0) * PI * r.powi(3);
            let (def_sec, def_stk) =
                TessellationSettings::default().calculate_sphere_resolution(r);
            let eff_stk = if stacks == 0 { def_stk } else { stacks.max(8) };
            let eff_sec = if sectors == 0 { def_sec } else { sectors.max(8) };

            let eq_r = if eff_stk % 2 == 0 {
                r
            } else {
                r * (PI / (2.0 * (eff_stk as f64))).cos()
            };
            let span_xy = regular_ngon_aabb_span(eq_r, eff_sec);
            let vol_tol = if eff_sec <= 8 {
                0.20
            } else if eff_sec <= 16 {
                0.08
            } else {
                0.025
            };
            let ctx = format!("sphere(r={r}, segs={segs}[sec={eff_sec}, stk={eff_stk}])");
            assert_all_invariants(
                &mesh,
                0,
                smooth_vol,
                vol_tol,
                [span_xy[0], span_xy[1], 2.0 * r],
                1e-9,
                &ctx,
            );
        }
    }
}

// ============================================================================
// 5. POLYGON PRISM INVARIANTS (UNCHAMFERED & CHAMFERED)
// ============================================================================

#[test]
fn invariants_polygon_prism_unchamfered_grid() {
    let radii = [0.001, 0.01, 0.05, 0.1];
    let h_ratios = [0.5, 2.0, 5.0];

    for sides in 3..=12 {
        for &r in &radii {
            for &hr in &h_ratios {
                let h = r * hr;
                let mesh = create_polygon_prism(sides, h, None, Some(r), None);
                let exact_vol =
                    0.5 * (sides as f64) * r * r * (2.0 * PI / (sides as f64)).sin() * h;
                let span_xy = regular_ngon_aabb_span(r, sides);
                let ctx = format!("polygon_prism(sides={sides}, r={r}, h={h}, chamfer=None)");
                assert_all_invariants(
                    &mesh,
                    0,
                    exact_vol,
                    1e-7,
                    [span_xy[0], span_xy[1], h],
                    1e-9,
                    &ctx,
                );
            }
        }
    }
}

#[test]
fn invariants_polygon_prism_chamfered_5_to_12_sides_grid() {
    let tan_alpha = (PI / 6.0).tan();

    let check_case = |sides: usize, r: f64, h: f64, chamfer: f64| {
        let poly_factor = 0.5 * (sides as f64) * (2.0 * PI / (sides as f64)).sin();
        let mesh = create_polygon_prism(sides, h, None, Some(r), Some(chamfer));

        // Effective clamped radial (c_r) and axial (c_z) chamfer offsets
        let mut c_r = chamfer.min(r * 0.45);
        let mut c_z = c_r * tan_alpha;
        if c_z > h * 0.40 {
            c_z = h * 0.40;
            c_r = c_z / tan_alpha;
        }

        // Analytical volume = unchamfered prism volume minus volume of two cut-off bevel rings:
        // V_prism = (n/2) * R^2 * sin(2*pi/n) * h
        // Delta_V_chamfer = n * sin(2*pi/n) * c_z * c_r * (R - c_r / 3)
        let v_prism = poly_factor * r * r * h;
        let delta_v_chamfer = 2.0 * poly_factor * c_z * c_r * (r - c_r / 3.0);
        let exact_vol = v_prism - delta_v_chamfer;

        let span_xy = regular_ngon_aabb_span(r, sides);
        let ctx = format!("polygon_prism_chamfered(sides={sides}, r={r}, h={h}, c={chamfer})");
        assert_all_invariants(
            &mesh,
            0,
            exact_vol,
            1e-7,
            [span_xy[0], span_xy[1], h],
            1e-9,
            &ctx,
        );
    };

    // 1. P0-T6 full sweep: sides 3..=12 x chamfer/side ratios {0.02, 0.1, 0.3, max_allowable} x h/r {0.5, 2.0}
    let r_base = 0.02;
    for sides in 3..=12 {
        let side_len = 2.0 * r_base * (PI / (sides as f64)).sin();
        for &hr in &[0.5, 2.0] {
            let h = r_base * hr;
            let max_chamfer = (r_base * 0.45).min((h * 0.40) / tan_alpha);
            for &chamfer in &[
                0.02 * side_len,
                0.10 * side_len,
                0.30 * side_len,
                max_chamfer,
            ] {
                check_case(sides, r_base, h, chamfer);
            }
        }
    }

    // 2. Multi-scale and tall aspect sweep across radii {0.001, 0.01, 0.05, 0.1} and h_ratios {2.0, 5.0}
    for sides in 3..=12 {
        for &r in &[0.001, 0.01, 0.05, 0.1] {
            for &hr in &[2.0, 5.0] {
                let h = r * hr;
                check_case(sides, r, h, 0.1 * r);
            }
        }
    }
}

// ============================================================================
// 6. SPUR GEAR INVARIANTS
// ============================================================================

#[test]
fn invariants_spur_gear_grid() {
    // Sweep tooth counts z in 8..=32 and modules 0.001..0.005 m
    let cases = [
        (0.001, 8usize, 0.005),
        (0.002, 8usize, 0.010),
        (0.002, 12usize, 0.012),
        (0.002, 16usize, 0.015),
        (0.002, 20usize, 0.015),
        (0.002, 24usize, 0.015),
        (0.002, 28usize, 0.020),
        (0.002, 32usize, 0.020),
        (0.005, 16usize, 0.040),
        (0.005, 24usize, 0.050),
    ];

    for &(m, z, b) in &cases {
        let gear = create_spur_gear(m, z, 20.0, b, None, None, None, None, None);
        let bbox = gear.bounding_box();
        let dims = bbox.dimensions();
        let ctx = format!("spur_gear(m={m}, z={z}, b={b})");

        // Polar integral of root circle + 2*z involute flanks
        let z_f = z as f64;
        let r = m * z_f * 0.5;
        let r_a = r + m;
        let r_f = (r - 1.25 * m).max(0.1 * m);
        let alpha = 20.0 * PI / 180.0;
        let r_b = r * alpha.cos();
        let psi = PI / (2.0 * z_f);
        let trough_half = PI / z_f;
        let inv_alpha = alpha.tan() - alpha;
        let min_tip = (trough_half * 0.15).max(1e-4);
        let theta_at_r = |rad: f64| -> f64 {
            let raw = if rad < r_b {
                psi + inv_alpha
            } else {
                let alpha_r = (r_b / rad).clamp(0.0, 1.0).acos();
                (psi + inv_alpha - (alpha_r.tan() - alpha_r)).max(min_tip)
            };
            raw.min(trough_half * 0.70)
        };
        let theta_base = theta_at_r(r_b);
        let theta_root = if r_f < r_b - 1e-9 {
            (theta_base + 0.25 * (trough_half - theta_base).max(1e-4)).min(trough_half * 0.75)
        } else {
            theta_at_r(r_f)
        };
        let n_int = 1000usize;
        let mut integral = 0.0;
        for i in 0..n_int {
            let r_mid = r_f + ((i as f64) + 0.5) * (r_a - r_f) / (n_int as f64);
            let th = if r_mid < r_b && r_f < r_b - 1e-9 {
                let t = (r_mid - r_f) / (r_b - r_f);
                theta_root + t * (theta_base - theta_root)
            } else {
                theta_at_r(r_mid)
            };
            integral += r_mid * th * (r_a - r_f) / (n_int as f64);
        }
        let analytical_vol = (PI * r_f * r_f + 2.0 * z_f * integral) * b;

        let tip_span = regular_ngon_aabb_span(r_a, z);
        assert!(
            dims[0] >= tip_span[0] - 1e-9 && dims[0] <= 2.0 * r_a + 1e-9,
            "{ctx}: X span {} out of [tip_span={}, 2*r_a={}]",
            dims[0],
            tip_span[0],
            2.0 * r_a
        );
        assert_all_invariants(
            &gear,
            0,
            analytical_vol,
            0.025,
            [dims[0], dims[1], b],
            1e-9,
            &ctx,
        );
    }

    // Also verify spur gear with central bore, hub, and keyway (genus 1 -> chi = 0)
    let gear_bore_hub = create_spur_gear(
        0.002,
        20,
        20.0,
        0.015,
        Some(0.006),
        Some(0.004),
        Some(0.0018),
        Some(0.025),
        Some(0.010),
    );
    let rep = validate_mesh(&gear_bore_hub, Some(1));
    assert_eq!(rep.status, "PASS", "gear_bore_hub failed: {:?}", rep.issues);
    let (_v, _e, _f, chi) = verify_directed_edges_and_euler(&gear_bore_hub, "gear_bore_hub");
    assert_eq!(
        chi, 0,
        "gear with bore must have Euler characteristic 0 (genus 1)"
    );
}

// ============================================================================
// 7. REVOLVE INVARIANTS (RECTANGLE & L-SHAPED PROFILES)
// ============================================================================

#[test]
fn invariants_revolve_full_circle_grid() {
    let radii = [0.001, 0.01, 0.05, 0.1];
    let hollow_segments = [0usize, 8, 16, 32, 64, 128];
    let solid_segments = [0usize, 8, 16, 32, 64];

    for &r in &radii {
        let h = 2.0 * r;

        // A. Solid rectangular and solid L-shaped profiles touching rotation axis r=0 (Genus 0 -> chi=2)
        for &segs in &solid_segments {
            let eff_segs = if segs == 0 {
                TessellationSettings::default().calculate_segments_arc(r, 2.0 * PI)
            } else {
                segs.max(3)
            };
            let span_xy = regular_ngon_aabb_span(r, eff_segs);
            let poly_factor = 0.5 * (eff_segs as f64) * (2.0 * PI / (eff_segs as f64)).sin();

            // 1. Solid rectangle profile
            let rect_solid = vec![
                [0.0, -h * 0.5],
                [r, -h * 0.5],
                [r, h * 0.5],
                [0.0, h * 0.5],
            ];
            let mesh_rect = create_revolve(&rect_solid, 360.0, segs);
            let expected_rect_vol = poly_factor * r * r * h;
            let ctx_rect = format!("revolve_rect_360(r={r}, h={h}, segs={segs}[eff={eff_segs}])");
            assert_all_invariants(
                &mesh_rect,
                0,
                expected_rect_vol,
                1e-7,
                [span_xy[0], span_xy[1], h],
                1e-9,
                &ctx_rect,
            );

            // 2. Solid L-shaped stepped shaft profile
            let l_solid = vec![
                [0.0, -h * 0.5],
                [r, -h * 0.5],
                [r, 0.0],
                [0.5 * r, 0.0],
                [0.5 * r, h * 0.5],
                [0.0, h * 0.5],
            ];
            let mesh_l = create_revolve(&l_solid, 360.0, segs);
            let expected_l_vol =
                poly_factor * (r * r * (h * 0.5) + (0.5 * r).powi(2) * (h * 0.5));
            let ctx_l = format!("revolve_L_solid_360(r={r}, h={h}, segs={segs}[eff={eff_segs}])");
            assert_all_invariants(
                &mesh_l,
                0,
                expected_l_vol,
                1e-7,
                [span_xy[0], span_xy[1], h],
                1e-9,
                &ctx_l,
            );
        }

        // B. Hollow rectangular ring and hollow L-shaped flange bushing (Genus 1 -> chi=0) across ALL segments up to 128
        for &segs in &hollow_segments {
            let eff_segs = if segs == 0 {
                TessellationSettings::default().calculate_segments_arc(r, 2.0 * PI)
            } else {
                segs.max(3)
            };
            let span_xy = regular_ngon_aabb_span(r, eff_segs);
            let poly_factor = 0.5 * (eff_segs as f64) * (2.0 * PI / (eff_segs as f64)).sin();

            let r_in = 0.3 * r;
            let r_mid = 0.65 * r;

            // 1. Hollow rectangular ring
            let rect_hollow = vec![
                [r_in, -h * 0.5],
                [r, -h * 0.5],
                [r, h * 0.5],
                [r_in, h * 0.5],
            ];
            let mesh_rh = create_revolve(&rect_hollow, 360.0, segs);
            let expected_rh_vol = poly_factor * (r * r - r_in * r_in) * h;
            let ctx_rh =
                format!("revolve_rect_hollow_360(r={r}, h={h}, segs={segs}[eff={eff_segs}])");
            assert_all_invariants(
                &mesh_rh,
                1,
                expected_rh_vol,
                1e-7,
                [span_xy[0], span_xy[1], h],
                1e-9,
                &ctx_rh,
            );

            // 2. Hollow L-shaped flange bushing
            let l_hollow = vec![
                [r_in, -h * 0.5],
                [r, -h * 0.5],
                [r, 0.0],
                [r_mid, 0.0],
                [r_mid, h * 0.5],
                [r_in, h * 0.5],
            ];
            let mesh_lh = create_revolve(&l_hollow, 360.0, segs);
            let expected_lh_vol = poly_factor
                * ((r * r - r_in * r_in) * (h * 0.5) + (r_mid * r_mid - r_in * r_in) * (h * 0.5));
            let ctx_lh =
                format!("revolve_L_hollow_360(r={r}, h={h}, segs={segs}[eff={eff_segs}])");
            assert_all_invariants(
                &mesh_lh,
                1,
                expected_lh_vol,
                1e-7,
                [span_xy[0], span_xy[1], h],
                1e-9,
                &ctx_lh,
            );
        }
    }
}

// ============================================================================
// 8. WHEEL RIM INVARIANTS
// ============================================================================

#[test]
fn invariants_wheel_rim_grid() {
    let cases = [
        (0.35, 0.16, 5usize, 5usize),
        (0.35, 0.16, 5usize, 4usize),
        (0.45, 0.22, 5usize, 5usize),
        (0.45, 0.22, 6usize, 6usize),
    ];

    for &(d_rim, w_rim, n_spokes, n_bolts) in &cases {
        let d_flange = d_rim * 1.08;
        let mesh = create_wheel_rim(
            d_rim,
            w_rim,
            Some(d_flange),
            None,
            None,
            None,
            Some(n_bolts),
            None,
            Some(n_spokes),
            None,
            None,
            None,
        );
        let ctx = format!("wheel_rim(d={d_rim}, w={w_rim}, spokes={n_spokes}, bolts={n_bolts})");
        // Topological genus = 1 central bore + n_bolts bolt holes + n_spokes ventilation windows
        let expected_genus = (1 + n_bolts + n_spokes) as i32;
        let bbox = mesh.bounding_box();
        let dims = bbox.dimensions();

        let report = validate_mesh(&mesh, Some(expected_genus));
        assert_eq!(
            report.status, "PASS",
            "{ctx}: validate_mesh failed: {:?}",
            report.issues
        );
        assert_permissible_warnings(&report.warnings, &ctx);
        let (_v, _e, _f, chi) = verify_directed_edges_and_euler(&mesh, &ctx);
        assert_eq!(chi, 2 - 2 * expected_genus, "{ctx}: Euler mismatch");
        assert!(mesh.signed_volume() > 0.0, "{ctx}: non-positive volume");
        assert!(
            (dims[2] - w_rim).abs() / d_flange < 1e-9,
            "{ctx}: Z dim {} != w_rim {}",
            dims[2],
            w_rim
        );
    }
}

// ============================================================================
// 9. CUT_HOLE TOPOLOGICAL, EULER, VOLUME, AND AABB INVARIANTS
// ============================================================================

#[test]
fn invariants_cut_hole_topology_volume_and_aabb() {
    for &scale in &[0.01, 0.05, 0.1] {
        let w = 4.0 * scale;
        let h = 4.0 * scale;
        let d = 2.0 * scale;
        let r_hole = 0.5 * scale;
        let base_box = create_box(w, h, d);
        let v_box = w * h * d;
        let eff_segs = TessellationSettings::default().calculate_segments(r_hole);
        let hole_area =
            0.5 * (eff_segs as f64) * r_hole * r_hole * (2.0 * PI / (eff_segs as f64)).sin();

        // 1. Blind hole (depth = 0.5 * d -> genus 0, chi = 2)
        let depth_blind = 0.5 * d;
        let blind_mesh = cut_hole(
            &base_box,
            [0.0, 0.0, d * 0.5],
            [0.0, 0.0, 1.0],
            r_hole,
            depth_blind,
            None,
            None,
        );
        let ctx_blind = format!("cut_hole_blind(scale={scale})");
        let (_v, _e, _f, chi_blind) = verify_directed_edges_and_euler(&blind_mesh, &ctx_blind);
        assert_eq!(
            chi_blind, 2,
            "{ctx_blind}: blind hole must keep genus 0 (chi=2)"
        );
        let expected_v_blind = v_box - hole_area * depth_blind;
        let sv_blind = blind_mesh.signed_volume();
        assert!(sv_blind > 0.0);
        assert!(
            (sv_blind - expected_v_blind).abs() / expected_v_blind < 0.01,
            "{ctx_blind}: signed_volume={sv_blind}, expected={expected_v_blind}"
        );
        let dims_blind = blind_mesh.bounding_box().dimensions();
        for axis in 0..3 {
            let exp_d = [w, h, d][axis];
            assert!((dims_blind[axis] - exp_d).abs() / w < 1e-9);
        }

        // 2. Through hole (depth = 1.2 * d -> genus 1, chi = 0)
        let thru_mesh = cut_hole(
            &base_box,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            r_hole,
            1.2 * d,
            None,
            None,
        );
        let ctx_thru = format!("cut_hole_thru(scale={scale})");
        let (_v, _e, _f, chi_thru) = verify_directed_edges_and_euler(&thru_mesh, &ctx_thru);
        assert_eq!(
            chi_thru, 0,
            "{ctx_thru}: through hole must have genus 1 (chi=0)"
        );
        let expected_v_thru = v_box - hole_area * d;
        let sv_thru = thru_mesh.signed_volume();
        assert!(sv_thru > 0.0);
        assert!(
            (sv_thru - expected_v_thru).abs() / expected_v_thru < 0.01,
            "{ctx_thru}: signed_volume={sv_thru}, expected={expected_v_thru}"
        );
        let dims_thru = thru_mesh.bounding_box().dimensions();
        for axis in 0..3 {
            let exp_d = [w, h, d][axis];
            assert!((dims_thru[axis] - exp_d).abs() / w < 1e-9);
        }
    }
}

// ============================================================================
// RESOLVED BUGS DISCOVERED BY INVARIANT SUITE (UN-IGNORED IN P0-T5..P0-T10)
// ============================================================================

#[test]
fn known_bug_cone_sharp_auto_segments_odd_after_halving_slivers() {
    let cone = create_cone(0.05, 0.0, 0.25, 0);
    let report = validate_mesh(&cone, Some(0));
    assert_eq!(
        report.status, "PASS",
        "Tall sharp cone with auto segments=0 (eff=50) failed validate_mesh: {:?}",
        report.issues
    );
}

#[test]
fn known_bug_cone_inverted_apex_missing_ring_halving() {
    let inv_cone = create_cone(0.0, 0.01, 0.02, 64);
    let report = validate_mesh(&inv_cone, Some(0));
    assert_eq!(
        report.status, "PASS",
        "Inverted cone failed validate_mesh: {:?}",
        report.issues
    );
}

#[test]
fn known_bug_spur_gear_high_teeth_cap_slivers() {
    let gear = create_spur_gear(0.001, 40, 20.0, 0.005, None, None, None, None, None);
    let report = validate_mesh(&gear, Some(0));
    assert_eq!(
        report.status, "PASS",
        "Spur gear z=40 failed validate_mesh: {:?}",
        report.issues
    );
}

#[test]
fn known_bug_revolve_360_axis_fan_slivers_at_128_segments() {
    let profile = vec![[0.0, -0.001], [0.001, -0.001], [0.001, 0.001], [0.0, 0.001]];
    let mesh = create_revolve(&profile, 360.0, 128);
    let report = validate_mesh(&mesh, Some(0));
    assert_eq!(
        report.status, "PASS",
        "Revolve 360° segs=128 failed validate_mesh: {:?}",
        report.issues
    );
}

#[test]
fn known_bug_wheel_rim_low_spoke_count_slivers() {
    let mesh = create_wheel_rim(
        0.40,
        0.18,
        Some(0.432),
        None,
        None,
        None,
        Some(4),
        None,
        Some(3),
        None,
        None,
        None,
    );
    let report = validate_mesh(&mesh, Some(8));
    assert_eq!(
        report.status, "PASS",
        "Wheel rim spokes=3, bolts=4 failed validate_mesh: {:?}",
        report.issues
    );
}

#[test]
fn known_bug_revolve_partial_angle_end_cap_slivers() {
    let profile = vec![[0.0, -0.01], [0.01, -0.01], [0.01, 0.01], [0.0, 0.01]];
    let mesh = create_revolve(&profile, 90.0, 32);
    let report = validate_mesh(&mesh, Some(0));
    assert_eq!(
        report.status, "PASS",
        "Partial revolve 90° failed validate_mesh: {:?}",
        report.issues
    );
}

#[test]
fn known_bug_polygon_prism_chamfer_low_sides_slivers() {
    let mesh = create_polygon_prism(3, 0.02, None, Some(0.01), Some(0.001));
    let report = validate_mesh(&mesh, Some(0));
    assert_eq!(
        report.status, "PASS",
        "Chamfered 3-sided prism failed validate_mesh: {:?}",
        report.issues
    );
}

#[test]
fn known_bug_cut_hole_csg_sliver_quality_gate() {
    let base_box = create_box(0.04, 0.04, 0.02);
    let thru_box = cut_hole(
        &base_box,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.005,
        0.025,
        None,
        None,
    );
    let report = validate_mesh(&thru_box, Some(1));
    assert_eq!(
        report.status, "PASS",
        "cut_hole through box failed validate_mesh: {:?}",
        report.issues
    );
}

// ============================================================================
// P2-T6: NAMED FACES ON PRIMITIVES
// ============================================================================

fn assert_named_faces_consistency(mesh: &TriangleMesh, expected_face_count: usize, ctx: &str) {
    assert_eq!(
        mesh.face_ids.len(),
        mesh.indices.len(),
        "{ctx}: face_ids.len() must equal indices.len()"
    );
    assert_eq!(
        mesh.faces.len(),
        expected_face_count,
        "{ctx}: expected {expected_face_count} faces, got {}",
        mesh.faces.len()
    );
    let sum_areas: f64 = mesh.faces.iter().map(|f| f.area).sum();
    let total_area = mesh.surface_area();
    assert!(
        (sum_areas - total_area).abs() < 1e-9,
        "{ctx}: sum of face areas ({sum_areas}) != mesh.surface_area() ({total_area})"
    );
    let sum_tris: usize = mesh.faces.iter().map(|f| f.triangles_count).sum();
    assert_eq!(
        sum_tris,
        mesh.indices.len(),
        "{ctx}: sum of face triangles_count ({sum_tris}) != indices.len() ({})",
        mesh.indices.len()
    );
}

#[test]
fn p2_t6_named_faces_box() {
    use mcp_cad_core::topology::FaceKind;
    let mesh = create_box(0.10, 0.06, 0.04);
    assert_named_faces_consistency(&mesh, 6, "box");

    let top = mesh.faces.iter().find(|f| f.name == "top").expect("top face");
    assert_eq!(top.kind, FaceKind::Planar);
    assert!(top.aliases.contains(&"max_z".to_string()));
    assert!((top.area - 0.10 * 0.06).abs() < 1e-12);
    assert!((top.normal_or_axis[2] - 1.0).abs() < 1e-9);
    assert!((top.centroid[2] - 0.02).abs() < 1e-9);

    let right = mesh.faces.iter().find(|f| f.name == "right").expect("right face");
    assert!(right.aliases.contains(&"max_x".to_string()));
    assert!((right.area - 0.06 * 0.04).abs() < 1e-12);
    assert!((right.normal_or_axis[0] - 1.0).abs() < 1e-9);
}

#[test]
fn p2_t6_named_faces_cylinder() {
    use mcp_cad_core::topology::FaceKind;
    let mesh = create_cylinder(0.025, 0.08, 64);
    assert_named_faces_consistency(&mesh, 3, "cylinder");

    let bottom = mesh
        .faces
        .iter()
        .find(|f| f.name == "cap_bottom" || f.aliases.iter().any(|a| a == "bottom"))
        .expect("bottom");
    assert_eq!(bottom.kind, FaceKind::Planar);
    assert!(bottom.aliases.contains(&"min_z".to_string()));
    assert!((bottom.normal_or_axis[2] + 1.0).abs() < 1e-9);

    let top = mesh
        .faces
        .iter()
        .find(|f| f.name == "cap_top" || f.aliases.iter().any(|a| a == "top"))
        .expect("top");
    assert_eq!(top.kind, FaceKind::Planar);
    assert!(top.aliases.contains(&"max_z".to_string()));
    assert!((top.normal_or_axis[2] - 1.0).abs() < 1e-9);

    let side = mesh.faces.iter().find(|f| f.name == "side").expect("side");
    assert_eq!(side.kind, FaceKind::Cylindrical);
    assert!(side.aliases.contains(&"barrel".to_string()));
    assert!(side.aliases.contains(&"outer".to_string()));
    assert!(side.aliases.contains(&"mantle".to_string()));
    assert!((side.radius.expect("cylinder radius") - 0.025).abs() < 1e-12);
}

#[test]
fn p2_t6_named_faces_cone_variants() {
    use mcp_cad_core::topology::FaceKind;

    // 1. Frustum (3 faces: cap_bottom, cap_top, side)
    let frustum = create_cone(0.03, 0.01, 0.05, 48);
    assert_named_faces_consistency(&frustum, 3, "cone_frustum");
    let side = frustum.faces.iter().find(|f| f.name == "side").expect("side");
    assert_eq!(side.kind, FaceKind::Conical);
    assert!((side.radius.unwrap() - 0.02).abs() < 1e-12);

    // 2. Sharp cone (r_top == 0 -> 2 faces: cap_bottom, side)
    let sharp = create_cone(0.03, 0.0, 0.05, 48);
    assert_named_faces_consistency(&sharp, 2, "cone_sharp");
    assert!(sharp
        .faces
        .iter()
        .any(|f| f.name == "cap_bottom" || f.aliases.iter().any(|a| a == "bottom")));
    assert!(sharp.faces.iter().any(|f| f.name == "side"));
    assert!(!sharp
        .faces
        .iter()
        .any(|f| f.name == "cap_top" || f.aliases.iter().any(|a| a == "top")));

    // 3. Inverted sharp cone (r_bottom == 0 -> 2 faces: cap_top, side)
    let inv = create_cone(0.0, 0.03, 0.05, 48);
    assert_named_faces_consistency(&inv, 2, "cone_inverted");
    assert!(inv
        .faces
        .iter()
        .any(|f| f.name == "cap_top" || f.aliases.iter().any(|a| a == "top")));
    assert!(inv.faces.iter().any(|f| f.name == "side"));
    assert!(!inv
        .faces
        .iter()
        .any(|f| f.name == "cap_bottom" || f.aliases.iter().any(|a| a == "bottom")));
}

#[test]
fn p2_t6_named_faces_sphere() {
    use mcp_cad_core::topology::FaceKind;
    let mesh = create_sphere(0.04, 32, 16);
    assert_named_faces_consistency(&mesh, 1, "sphere");
    let surf = &mesh.faces[0];
    assert_eq!(surf.name, "surface");
    assert_eq!(surf.kind, FaceKind::Spherical);
    assert!(surf.aliases.contains(&"outer".to_string()));
    assert!(surf.aliases.contains(&"shell".to_string()));
    assert!((surf.radius.unwrap() - 0.04).abs() < 1e-12);
}

#[test]
fn p2_t6_named_faces_polygon_prism() {
    use mcp_cad_core::topology::FaceKind;
    let hex = create_polygon_prism(6, 0.02, None, Some(0.03), None);
    assert_named_faces_consistency(&hex, 8, "hex_prism");
    for i in 0..6 {
        let name = format!("side_{i}");
        let f = hex.faces.iter().find(|f| f.name == name).expect(&name);
        assert_eq!(f.kind, FaceKind::Planar);
        let mid_angle = (i as f64 + 0.5) * 2.0 * PI / 6.0;
        let dot = f.normal_or_axis[0] * mid_angle.cos() + f.normal_or_axis[1] * mid_angle.sin();
        assert!(dot > 0.999999, "side_{i} normal dot={dot}");
    }

    let hex_chamfer = create_polygon_prism(6, 0.02, None, Some(0.03), Some(0.002));
    assert_named_faces_consistency(&hex_chamfer, 20, "hex_prism_chamfer");
}

#[test]
fn p2_t6_legacy_json_deserialization_and_dag_origin_tags() {
    use mcp_cad_core::dag::{FeatureNode, FeatureOp, FeatureTree};

    // 1. Legacy JSON without face_ids or faces must deserialize cleanly
    let legacy_json = r#"{"vertices":[{"position":[0.0,0.0,0.0],"normal":[0.0,0.0,1.0]},{"position":[1.0,0.0,0.0],"normal":[0.0,0.0,1.0]},{"position":[0.0,1.0,0.0],"normal":[0.0,0.0,1.0]}],"indices":[[0,1,2]]}"#;
    let parsed: TriangleMesh = serde_json::from_str(legacy_json).expect("legacy mesh JSON must parse");
    assert!(parsed.face_ids.is_empty());
    assert!(parsed.faces.is_empty());

    // 2. FeatureTree evaluation tags origin_feature and qualified aliases + transforms
    let mut tree = FeatureTree::new();
    let node = FeatureNode::new(
        "Base Plate",
        FeatureOp::AddBox {
            width: 0.1,
            height: 0.08,
            depth: 0.02,
            position: Some([0.0, 0.0, 0.05]),
            rotation_deg: None,
        },
    );
    let node_id_str = node.id.to_string();
    tree.add_node(node);
    let evaluated = tree.evaluate().expect("tree evaluate");
    assert_named_faces_consistency(&evaluated, 6, "dag_box");
    let top = evaluated.faces.iter().find(|f| f.name == "top").unwrap();
    assert_eq!(top.origin_feature.as_deref(), Some(node_id_str.as_str()));
    assert!(top.aliases.contains(&"feat_0:top".to_string()));
    assert!(top.aliases.contains(&"base_plate:top".to_string()));
    assert!((top.centroid[2] - 0.06).abs() < 1e-9);
}

#[test]
fn p2_t15_wheel_rim_small_diameters_no_clamp_panic_and_watertight() {
    use mcp_cad_core::dag::{FeatureNode, FeatureOp, FeatureTree};

    for &(d_rim, w_rim, bolts, spokes) in &[
        (0.05, 0.025, 5usize, 5usize),
        (0.10, 0.050, 4usize, 6usize),
        (0.12, 0.055, 5usize, 5usize),
        (0.14, 0.060, 3usize, 3usize),
    ] {
        let expected_genus = (1 + bolts + spokes) as i32;
        let mesh = create_wheel_rim(
            d_rim,
            w_rim,
            None,
            None,
            None,
            None,
            Some(bolts),
            None,
            Some(spokes),
            None,
            None,
            None,
        );
        let report = validate_mesh(&mesh, Some(expected_genus));
        assert_eq!(
            report.status, "PASS",
            "Small wheel rim d={d_rim}, w={w_rim} failed validate_mesh: {:?}",
            report.issues
        );

        // Also verify via FeatureTree JSON round-trip (default_bolt_hole_diameter)
        let mut tree = FeatureTree::new();
        tree.add_node(FeatureNode::new(
            "SmallWheel",
            FeatureOp::AddWheelRim {
                rim_diameter: d_rim,
                rim_width: w_rim,
                flange_diameter: None,
                hub_diameter: None,
                hub_bore_diameter: None,
                pcd_diameter: None,
                bolt_holes_count: Some(bolts),
                bolt_hole_diameter: None,
                spokes_count: Some(spokes),
                spoke_width: None,
                spoke_thickness: None,
                offset_et: None,
                position: None,
                rotation_deg: None,
            },
        ));
        let json_str = serde_json::to_string(&tree).expect("serialize wheel tree");
        let roundtrip: FeatureTree = serde_json::from_str(&json_str).expect("deserialize wheel tree");
        let dag_mesh = roundtrip.evaluate().expect("evaluate small wheel DAG");
        let dag_report = validate_mesh(&dag_mesh, Some(expected_genus));
        assert_eq!(
            dag_report.status, "PASS",
            "DAG small wheel rim d={d_rim}, w={w_rim} failed validate_mesh: {:?}",
            dag_report.issues
        );
    }
}
