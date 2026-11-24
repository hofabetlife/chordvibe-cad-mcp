use std::collections::{HashSet, VecDeque};
use std::time::Instant;

use mcp_cad_core::material::MaterialRegistry;
use mcp_cad_core::primitives::create_helix_with_settings;
use mcp_cad_core::tessellation::TessellationSettings;
use mcp_cad_fea::{
    evaluate_mesh_metrics, solve_dynamic_hht_coupled, AabbFace, AccuracyMode, BoundaryConditions,
    DamageMaterialParams, DynamicBodySpec, HhtOptions, J2MaterialParams, MeshDensity, NeumannLoad,
    TetrahedralMesh, TimeForceCurve,
};
use nalgebra::Vector3;

fn count_node_connected_components(mesh: &TetrahedralMesh) -> usize {
    let n = mesh.node_count();
    if n == 0 || mesh.elements.is_empty() {
        return 0;
    }
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut used = vec![false; n];
    for elem in &mesh.elements {
        let ids = elem.node_indices;
        for &a in &ids {
            used[a] = true;
            for &b in &ids {
                if a != b {
                    adj[a].push(b);
                }
            }
        }
    }
    let mut visited = vec![false; n];
    let mut comps = 0usize;
    let mut q = VecDeque::new();
    for start in 0..n {
        if !used[start] || visited[start] {
            continue;
        }
        comps += 1;
        visited[start] = true;
        q.push_back(start);
        while let Some(u) = q.pop_front() {
            for &v in &adj[u] {
                if !visited[v] {
                    visited[v] = true;
                    q.push_back(v);
                }
            }
        }
    }
    comps
}

fn total_tet_volume(mesh: &TetrahedralMesh) -> f64 {
    let mut vol = 0.0;
    for elem in &mesh.elements {
        let [a, b, c, d] = elem.node_indices;
        let p0 = Vector3::from(mesh.nodes[a].position);
        let p1 = Vector3::from(mesh.nodes[b].position);
        let p2 = Vector3::from(mesh.nodes[c].position);
        let p3 = Vector3::from(mesh.nodes[d].position);
        vol += (p1 - p0).dot(&(p2 - p0).cross(&(p3 - p0))).abs() / 6.0;
    }
    vol
}

fn inspect_and_solve_preset(label: &str, density: MeshDensity, acc_mode: AccuracyMode, solve: bool) {
    let reg = MaterialRegistry::new();
    let steel = reg.get("steel_carbon_aisi1045").expect("steel_1045");
    let j2 = J2MaterialParams::from_material(steel).unwrap();
    let dmg = DamageMaterialParams::from_material(steel).unwrap();

    let surf = create_helix_with_settings(
        0.0175,
        0.0025,
        0.012,
        0.06,
        1,
        true,
        &TessellationSettings::default(),
    );
    let cad_vol = surf.volume().abs();

    let t_mesh = Instant::now();
    let tet_mesh =
        TetrahedralMesh::from_mesh_with_accuracy(&surf, density, acc_mode).expect("meshing failed");
    let mesh_ms = t_mesh.elapsed().as_secs_f64() * 1000.0;

    let comps = count_node_connected_components(&tet_mesh);
    let face_comps = mcp_cad_fea::find_face_connected_components(
        &tet_mesh,
        &vec![true; tet_mesh.element_count()],
        &HashSet::new(),
    )
    .len();
    let tet_vol = total_tet_volume(&tet_mesh);
    let metrics = evaluate_mesh_metrics(&tet_mesh, &surf, None, 1e-3);

    println!(
        "[{label}] mesh={mesh_ms:.1} ms | nodes={} (dofs={}) | elems={} (tet10={}) | node_comps={comps} | face_comps={face_comps} | vol_ratio={:.3} | min_Js={:.3} | rms_dev={:.3} mm",
        tet_mesh.node_count(),
        tet_mesh.node_count() * 3,
        tet_mesh.element_count(),
        tet_mesh.is_tet10(),
        tet_vol / cad_vol,
        metrics.min_scaled_jacobian,
        metrics.rms_boundary_deviation_mm,
    );

    if !solve {
        return;
    }

    let duration_s = 0.04;
    let (div, tol) = match label {
        "Standard" => (20.0, 5e-5),
        "High" => (30.0, 3e-5),
        _ => (30.0, 2.5e-5),
    };

    let mut bc = BoundaryConditions::new();
    bc.fix_aabb_face(AabbFace::MinZ, 1.6e-3);
    bc.neumann.push(NeumannLoad::AabbFaceForce {
        face: AabbFace::MaxZ,
        tolerance: 1.6e-3,
        total_force: [0.0, 0.0, -500.0],
    });

    let body = DynamicBodySpec::new("spring", tet_mesh, j2, steel.density_kg_m3)
        .with_boundary(bc)
        .with_damage(dmg)
        .with_force_curve(TimeForceCurve::Ramp {
            duration_s: duration_s * 0.65,
        });

    let dt0 = (duration_s / div).clamp(2.5e-5, 2e-3);
    let opts = HhtOptions {
        alpha: -0.05,
        total_time_s: duration_s,
        dt_initial_s: dt0,
        dt_min_s: 1e-7,
        dt_max_s: duration_s / 8.0,
        force_tolerance: tol,
        max_frames: 120,
        ..HhtOptions::default()
    };

    let t_solve = Instant::now();
    let mut last_print = 0usize;
    let res = solve_dynamic_hht_coupled(
        vec![body],
        &[],
        opts,
        Some(|p: mcp_cad_fea::DynamicProgressUpdate| {
            if p.step_count == 0 || p.step_count >= last_print + 10 || p.progress_pct >= 99.9 {
                last_print = p.step_count;
                println!(
                    "    step={} t={:.4}/{:.4} ({:.1}%) dt={:.2e} bisect={} vm={:.1} MPa elapsed={:.2}s",
                    p.step_count,
                    p.time_s,
                    p.total_time_s,
                    p.progress_pct,
                    p.current_dt_s,
                    p.bisections,
                    p.max_von_mises_pa * 1e-6,
                    t_solve.elapsed().as_secs_f64()
                );
            }
        }),
        None,
    );
    let solve_ms = t_solve.elapsed().as_secs_f64() * 1000.0;
    match res {
        Ok(rep) => println!(
            "  -> SOLVE ok in {solve_ms:.1} ms | steps={} | newton={} | bisections={} | frames={}",
            rep.total_steps,
            rep.total_newton_iterations,
            rep.total_bisections,
            rep.frames.len()
        ),
        Err(e) => println!("  -> SOLVE ERR in {solve_ms:.1} ms: {e}"),
    }
}

#[test]
#[ignore]
fn bench_coil_spring_mesh_only() {
    inspect_and_solve_preset("Standard", MeshDensity::Coarse, AccuracyMode::Fast, false);
    inspect_and_solve_preset("High", MeshDensity::Medium, AccuracyMode::Standard, false);
    inspect_and_solve_preset("Ultra", MeshDensity::Coarse, AccuracyMode::High, false);
}

#[test]
#[ignore]
fn bench_coil_spring_high_solve() {
    inspect_and_solve_preset("High", MeshDensity::Medium, AccuracyMode::Standard, true);
}

#[test]
#[ignore]
fn bench_coil_spring_ultra_solve() {
    inspect_and_solve_preset("Ultra", MeshDensity::Coarse, AccuracyMode::High, true);
}

#[test]
#[ignore]
fn bench_coil_spring_standard_solve() {
    inspect_and_solve_preset("Standard", MeshDensity::Coarse, AccuracyMode::Fast, true);
}
