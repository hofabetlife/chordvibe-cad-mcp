//! Spike A: `manifold-rust` (v0.19.2, Apache-2.0) vs `mcp-cad-core` BSP-CSG (Option D)
//!
//! Geometry specification (P1-T3):
//! 1. Cylinder: R_out = 15 mm, h = 50 mm (z in [-25, +25] mm)
//! 2. Minus coaxial through-hole: D = 20 mm (R_in = 10 mm, h = 52 mm)
//! 3. Plus box-flange: 40 x 40 x 10 mm collar flange (z in [-20, -10] mm) with coaxial bore
//! 4. Edge fillet R = 2 mm (evaluated via CSG corner fillet cutter on the 4 vertical flange edges,
//!    since `manifold-rust` is a mesh boolean kernel without general B-Rep rolling-ball filleting)
//! 5. Face provenance check: verify that `MeshGLP::face_id` and `run_original_id` preserve
//!    the top end-cap face identifier (`face_id = 101`, "cyl:cap_top") across boolean operations.

use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::time::Instant;

use manifold_rust::linalg::Vec3;
use manifold_rust::manifold::Manifold;
use manifold_rust::types::MeshGLP;
use mcp_cad_core::csg::{csg_difference, csg_union};
use mcp_cad_core::diagnostics::validate_mesh;
use mcp_cad_core::primitives::{create_box, create_cylinder};
use mcp_cad_core::topology::{TriangleMesh, Vertex};

/// Semantic face IDs assigned to input primitive triangles before boolean operations.
const FACE_CYL_CAP_TOP: u32 = 101;
const FACE_CYL_CAP_BOTTOM: u32 = 102;
const FACE_CYL_SIDE: u32 = 103;
const FACE_HOLE_CAP_TOP: u32 = 201;
const FACE_HOLE_CAP_BOTTOM: u32 = 202;
const FACE_HOLE_WALL: u32 = 203;
const FACE_FLANGE_TOP: u32 = 301;
const FACE_FLANGE_BOTTOM: u32 = 302;
const FACE_FLANGE_SIDE_POS_X: u32 = 303;
const FACE_FLANGE_SIDE_NEG_X: u32 = 304;
const FACE_FLANGE_SIDE_POS_Y: u32 = 305;
const FACE_FLANGE_SIDE_NEG_Y: u32 = 306;

fn classify_cylinder_face(v0: [f64; 3], v1: [f64; 3], v2: [f64; 3], half_h: f64, is_hole: bool) -> u32 {
    let cz = (v0[2] + v1[2] + v2[2]) / 3.0;
    if (cz - half_h).abs() < 1e-4 {
        if is_hole { FACE_HOLE_CAP_TOP } else { FACE_CYL_CAP_TOP }
    } else if (cz + half_h).abs() < 1e-4 {
        if is_hole { FACE_HOLE_CAP_BOTTOM } else { FACE_CYL_CAP_BOTTOM }
    } else if is_hole {
        FACE_HOLE_WALL
    } else {
        FACE_CYL_SIDE
    }
}

fn classify_box_face(v0: [f64; 3], v1: [f64; 3], v2: [f64; 3]) -> u32 {
    let ux = v1[0] - v0[0];
    let uy = v1[1] - v0[1];
    let uz = v1[2] - v0[2];
    let vx = v2[0] - v0[0];
    let vy = v2[1] - v0[1];
    let vz = v2[2] - v0[2];
    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    if nz.abs() >= nx.abs() && nz.abs() >= ny.abs() {
        if nz > 0.0 { FACE_FLANGE_TOP } else { FACE_FLANGE_BOTTOM }
    } else if nx.abs() >= ny.abs() {
        if nx > 0.0 { FACE_FLANGE_SIDE_POS_X } else { FACE_FLANGE_SIDE_NEG_X }
    } else if ny > 0.0 {
        FACE_FLANGE_SIDE_POS_Y
    } else {
        FACE_FLANGE_SIDE_NEG_Y
    }
}

/// Converts a `TriangleMesh` (which may have split vertices for sharp normals) into a welded
/// `MeshGLP<f64, u32>` with explicit per-triangle `face_id` tags and reserved `run_original_id`.
fn triangle_mesh_to_manifold_with_faces<F>(
    mesh: &TriangleMesh,
    mut classify_tri: F,
) -> (Manifold, u32)
where
    F: FnMut([f64; 3], [f64; 3], [f64; 3]) -> u32,
{
    let dims = mesh.bounding_box().dimensions();
    let diag = (dims[0] * dims[0] + dims[1] * dims[1] + dims[2] * dims[2])
        .sqrt()
        .max(1e-6);
    let weld_eps = (diag * 1e-7).max(1e-12);
    let weld_scale = 1.0 / weld_eps;
    let weld_eps_sq = weld_eps * weld_eps;

    let mut cell_to_welded: BTreeMap<[i64; 3], Vec<u32>> = BTreeMap::new();
    let mut vert_properties: Vec<f64> = Vec::new();
    let mut tri_verts: Vec<u32> = Vec::with_capacity(mesh.indices.len() * 3);
    let mut face_id: Vec<u32> = Vec::with_capacity(mesh.indices.len());

    for tri in &mesh.indices {
        let p0 = mesh.vertices[tri[0] as usize].position;
        let p1 = mesh.vertices[tri[1] as usize].position;
        let p2 = mesh.vertices[tri[2] as usize].position;
        let fid = classify_tri(p0, p1, p2);

        let mut mapped = [0u32; 3];
        for (k_i, p) in [p0, p1, p2].into_iter().enumerate() {
            let base = [
                (p[0] * weld_scale).round() as i64,
                (p[1] * weld_scale).round() as i64,
                (p[2] * weld_scale).round() as i64,
            ];
            let mut found = None;
            'search: for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let key = [base[0] + dx, base[1] + dy, base[2] + dz];
                        if let Some(cands) = cell_to_welded.get(&key) {
                            for &c in cands {
                                let off = (c as usize) * 3;
                                let dist_sq = (vert_properties[off] - p[0]).powi(2)
                                    + (vert_properties[off + 1] - p[1]).powi(2)
                                    + (vert_properties[off + 2] - p[2]).powi(2);
                                if dist_sq <= weld_eps_sq {
                                    found = Some(c);
                                    break 'search;
                                }
                            }
                        }
                    }
                }
            }
            let idx = match found {
                Some(c) => c,
                None => {
                    let new_idx = (vert_properties.len() / 3) as u32;
                    vert_properties.extend_from_slice(&p);
                    cell_to_welded.entry(base).or_default().push(new_idx);
                    new_idx
                }
            };
            mapped[k_i] = idx;
        }
        if mapped[0] != mapped[1] && mapped[1] != mapped[2] && mapped[2] != mapped[0] {
            tri_verts.extend_from_slice(&mapped);
            face_id.push(fid);
        }
    }

    let orig_id = Manifold::reserve_ids(1);
    let mgl = MeshGLP::<f64, u32> {
        num_prop: 3,
        vert_properties,
        tri_verts,
        run_original_id: vec![orig_id],
        face_id,
        ..Default::default()
    };
    let manifold = Manifold::from_mesh_glp(&mgl);
    (manifold, orig_id)
}

/// Converts a `Manifold` back to `mcp_cad_core::topology::TriangleMesh` and returns
/// the exported `MeshGLP<f64, u32>` for face provenance inspection.
fn manifold_to_triangle_mesh(m: &Manifold) -> (TriangleMesh, MeshGLP<f64, u32>) {
    let mgl: MeshGLP<f64, u32> = m.get_mesh_glp(-1);
    let mut vertices = Vec::with_capacity(mgl.num_vert());
    for i in 0..mgl.num_vert() {
        let pos = mgl.get_vert_pos(i);
        vertices.push(Vertex::new(pos, [0.0, 0.0, 1.0]));
    }
    let mut indices = Vec::with_capacity(mgl.num_tri());
    for t in 0..mgl.num_tri() {
        indices.push(mgl.get_tri_verts(t));
    }
    let mut mesh = TriangleMesh { vertices, indices };
    mesh.recompute_vertex_normals();
    (mesh, mgl)
}

/// Builds a corner cutter that rounds the 4 vertical corners of the 40x40x10 mm flange
/// with fillet radius R_f = 2 mm over z in [-21, -9] mm.
fn build_flange_vertical_corner_fillet_cutters(segments: i32) -> Manifold {
    let r_f = 2.0;
    let half_w = 20.0;
    let z_height = 12.0;
    let z_center = -15.0;
    let mut result = Manifold::empty();

    for &sx in &[-1.0_f64, 1.0_f64] {
        for &sy in &[-1.0_f64, 1.0_f64] {
            let block = Manifold::cube(Vec3::new(r_f, r_f, z_height), true).translate(Vec3::new(
                sx * (half_w - 0.5 * r_f),
                sy * (half_w - 0.5 * r_f),
                z_center,
            ));
            let cyl = Manifold::cylinder_centered(z_height + 2.0, r_f, r_f, segments, true)
                .translate(Vec3::new(
                    sx * (half_w - r_f),
                    sy * (half_w - r_f),
                    z_center,
                ));
            let corner_cutter = block.difference(&cyl);
            result = if result.is_empty() {
                corner_cutter
            } else {
                result.union(&corner_cutter)
            };
        }
    }
    result
}

fn main() {
    println!("================================================================================");
    println!("P1-T3 SPIKE 1: manifold-rust v0.19.2 (Pure Rust Manifold) vs mcp-cad-core BSP");
    println!("================================================================================");

    let segments: usize = 64;
    let r_out = 15.0; // mm
    let r_in = 10.0; // mm (diameter 20 mm)
    let h_cyl = 50.0; // mm
    let flange_w = 40.0; // mm
    let flange_h = 10.0; // mm (z in [-20, -10] mm)
    let r_fillet = 2.0; // mm

    // Analytical volumes (exact continuous cylinder/hole vs N-gon prism discretization)
    let v_bushing_exact = PI * (r_out * r_out - r_in * r_in) * h_cyl;
    let v_flange_add_exact = (flange_w * flange_w - PI * r_out * r_out) * flange_h;
    let v_flanged_bushing_exact = v_bushing_exact + v_flange_add_exact;
    let v_4_vertical_fillets_exact = 4.0 * (1.0 - PI / 4.0) * r_fillet * r_fillet * flange_h;
    let v_filleted_exact = v_flanged_bushing_exact - v_4_vertical_fillets_exact;

    // Exact volume for regular 64-gon cylinder approximation
    let n = segments as f64;
    let poly_factor = 0.5 * n * (2.0 * PI / n).sin();
    let v_flanged_bushing_polygon = poly_factor * (r_out * r_out - r_in * r_in) * h_cyl
        + (flange_w * flange_w - poly_factor * r_out * r_out) * flange_h;

    println!("Analytical Reference Volumes (mm^3):");
    println!("  Hollow bushing (R=15, r=10, h=50):       {:12.4} mm^3", v_bushing_exact);
    println!("  Flanged bushing (+ 40x40x10 flange):     {:12.4} mm^3 (64-gon exact: {:12.4})", v_flanged_bushing_exact, v_flanged_bushing_polygon);
    println!("  With 4x R=2mm flange corner fillets:     {:12.4} mm^3", v_filleted_exact);
    println!();

    // -------------------------------------------------------------------------
    // PART A: manifold-rust v0.19.2 (with face_id provenance tracking)
    // -------------------------------------------------------------------------
    let t0 = Instant::now();
    let cyl_mesh = create_cylinder(r_out, h_cyl, segments);
    let hole_mesh = create_cylinder(r_in, h_cyl + 2.0, segments);
    let mut flange_mesh = create_box(flange_w, flange_w, flange_h);
    flange_mesh.translate([0.0, 0.0, -15.0]);

    let (m_cyl, orig_cyl) = triangle_mesh_to_manifold_with_faces(&cyl_mesh, |p0, p1, p2| {
        classify_cylinder_face(p0, p1, p2, 0.5 * h_cyl, false)
    });
    let (m_hole, orig_hole) = triangle_mesh_to_manifold_with_faces(&hole_mesh, |p0, p1, p2| {
        classify_cylinder_face(p0, p1, p2, 0.5 * (h_cyl + 2.0), true)
    });
    let (m_flange, orig_flange) =
        triangle_mesh_to_manifold_with_faces(&flange_mesh, classify_box_face);

    let t_bool_start = Instant::now();
    // (Cylinder + Flange) - Coaxial Hole (equivalently: Cylinder - Hole + (Flange - Hole))
    let m_flanged_bushing = m_cyl.union(&m_flange).difference(&m_hole);
    let dt_manifold_bool = t_bool_start.elapsed();

    let (out_mesh, out_mgl) = manifold_to_triangle_mesh(&m_flanged_bushing);
    let dt_manifold_total = t0.elapsed();

    let val_manifold = validate_mesh(&out_mesh, Some(1));
    let vol_manifold = out_mesh.signed_volume();
    let err_vs_exact_pct =
        ((vol_manifold - v_flanged_bushing_exact) / v_flanged_bushing_exact).abs() * 100.0;
    let err_vs_poly_pct =
        ((vol_manifold - v_flanged_bushing_polygon) / v_flanged_bushing_polygon).abs() * 100.0;

    // Inspect face provenance (`face_id` and `run_original_id`) on exported MeshGLP
    let mut face_area_by_id: BTreeMap<u32, (usize, f64)> = BTreeMap::new();
    for t in 0..out_mgl.num_tri() {
        let fid = out_mgl.face_id[t];
        let tv = out_mgl.get_tri_verts(t);
        let p0 = out_mgl.get_vert_pos(tv[0] as usize);
        let p1 = out_mgl.get_vert_pos(tv[1] as usize);
        let p2 = out_mgl.get_vert_pos(tv[2] as usize);
        let ux = p1[0] - p0[0];
        let uy = p1[1] - p0[1];
        let uz = p1[2] - p0[2];
        let vx = p2[0] - p0[0];
        let vy = p2[1] - p0[1];
        let vz = p2[2] - p0[2];
        let area = 0.5
            * ((uy * vz - uz * vy).powi(2)
                + (uz * vx - ux * vz).powi(2)
                + (ux * vy - uy * vx).powi(2))
            .sqrt();
        let entry = face_area_by_id.entry(fid).or_insert((0, 0.0));
        entry.0 += 1;
        entry.1 += area;
    }

    let &(top_cap_tris, top_cap_area) = face_area_by_id.get(&FACE_CYL_CAP_TOP).unwrap_or(&(0, 0.0));
    let expected_top_cap_area_exact = PI * (r_out * r_out - r_in * r_in);
    let expected_top_cap_area_poly = poly_factor * (r_out * r_out - r_in * r_in);
    let &(hole_wall_tris, hole_wall_area) = face_area_by_id.get(&FACE_HOLE_WALL).unwrap_or(&(0, 0.0));

    println!("--- [1] Candidate A: manifold-rust v0.19.2 (Flanged Bushing) ---");
    println!("  Status:                     {:?}", m_flanged_bushing.status());
    println!("  Boolean time:               {:.3} ms (total w/ import+export: {:.3} ms)", dt_manifold_bool.as_secs_f64() * 1000.0, dt_manifold_total.as_secs_f64() * 1000.0);
    println!("  Signed volume:              {:.4} mm^3", vol_manifold);
    println!("  Error vs analytical circle: {:.4}% (vs 64-gon exact: {:.6}%)", err_vs_exact_pct, err_vs_poly_pct);
    println!("  Triangles / Vertices:       {} tris / {} verts", out_mesh.indices.len(), out_mesh.vertices.len());
    println!("  validate_mesh status:       {} (watertight={}, open_edges={}, non_manifold={})", val_manifold.status, val_manifold.topology.is_watertight, val_manifold.topology.boundary_edges_count, val_manifold.topology.non_manifold_edges_count);
    println!("  Topology (Euler / Genus):   chi = {}, genus = {:.1}", val_manifold.topology.euler_characteristic, val_manifold.topology.genus);
    println!("  Triangle angle quality:     min_angle = {:.3} deg, <1 deg = {}, <5 deg ratio = {:.2}%", val_manifold.angle_stats.min_angle_deg, val_manifold.geometry.lt_1_deg_count, val_manifold.geometry.lt_5_deg_ratio * 100.0);
    println!("  Topological Face Provenance (native MeshGLP.face_id + run_original_id):");
    println!("    Reserved run_original_ids: cyl={}, hole={}, flange={}", orig_cyl, orig_hole, orig_flange);
    println!("    Exported run_original_id:  {:?}", out_mgl.run_original_id);
    println!("    Top end-cap (face_id={}):  {} tris, area = {:.4} mm^2 (64-gon exact = {:.4} mm^2, circle = {:.4} mm^2)", FACE_CYL_CAP_TOP, top_cap_tris, top_cap_area, expected_top_cap_area_poly, expected_top_cap_area_exact);
    println!("    Hole wall   (face_id={}):  {} tris, area = {:.4} mm^2", FACE_HOLE_WALL, hole_wall_tris, hole_wall_area);

    // Also test fillet cutter in manifold-rust
    let t_fillet = Instant::now();
    let corner_cutters = build_flange_vertical_corner_fillet_cutters(segments as i32);
    let m_filleted = m_flanged_bushing.difference(&corner_cutters);
    let dt_fillet = t_fillet.elapsed();
    let (filleted_mesh, _) = manifold_to_triangle_mesh(&m_filleted);
    let val_filleted = validate_mesh(&filleted_mesh, Some(1));
    let vol_filleted = filleted_mesh.signed_volume();
    println!("  With R=2mm CSG corner fillets:");
    println!("    Fillet boolean time:      {:.3} ms", dt_fillet.as_secs_f64() * 1000.0);
    println!("    Volume:                   {:.4} mm^3 (analytical: {:.4} mm^3, err = {:.4}%)", vol_filleted, v_filleted_exact, ((vol_filleted - v_filleted_exact) / v_filleted_exact).abs() * 100.0);
    println!("    Triangles:                {}, watertight={}, genus={:.1}", filleted_mesh.indices.len(), val_filleted.topology.is_watertight, val_filleted.topology.genus);
    println!();

    // -------------------------------------------------------------------------
    // PART A2: manifold-rust v0.19.2 using native minimal constructors
    // -------------------------------------------------------------------------
    let t_nat = Instant::now();
    let nat_cyl = Manifold::cylinder_centered(h_cyl, r_out, r_out, segments as i32, true);
    let nat_hole = Manifold::cylinder_centered(h_cyl + 2.0, r_in, r_in, segments as i32, true);
    let nat_flange =
        Manifold::cube(Vec3::new(flange_w, flange_w, flange_h), true).translate(Vec3::new(0.0, 0.0, -15.0));
    let t_nat_bool = Instant::now();
    let nat_result = nat_cyl.union(&nat_flange).difference(&nat_hole);
    let dt_nat_bool = t_nat_bool.elapsed();
    let nat_filleted = nat_result.difference(&corner_cutters);
    let dt_nat_total = t_nat.elapsed();
    let (nat_mesh, _) = manifold_to_triangle_mesh(&nat_result);
    let val_nat = validate_mesh(&nat_mesh, Some(1));
    println!("--- [1b] Candidate A (Native Manifold Constructors, 64 segments) ---");
    println!("  Boolean time (union+diff):  {:.3} ms (with 4 fillets: {:.3} ms)", dt_nat_bool.as_secs_f64() * 1000.0, dt_nat_total.as_secs_f64() * 1000.0);
    println!("  Signed volume:              {:.4} mm^3 (filleted: {:.4} mm^3)", nat_mesh.signed_volume(), nat_filleted.volume());
    println!("  Triangles / Vertices:       {} tris / {} verts (filleted: {} tris)", nat_mesh.indices.len(), nat_mesh.vertices.len(), nat_filleted.num_tri());
    println!("  Watertight / Genus:         watertight={}, open_edges={}, non_manifold={}, genus={:.1}", val_nat.topology.is_watertight, val_nat.topology.boundary_edges_count, val_nat.topology.non_manifold_edges_count, val_nat.topology.genus);
    println!();

    // -------------------------------------------------------------------------
    // PART B: Option D Baseline — Current `mcp-cad-core` BSP-CSG on identical meshes
    // -------------------------------------------------------------------------
    let t_bsp_start = Instant::now();
    let bsp_step1 = csg_union(&cyl_mesh, &flange_mesh);
    let bsp_flanged_bushing = csg_difference(&bsp_step1, &hole_mesh);
    let dt_bsp_bool = t_bsp_start.elapsed();
    let val_bsp = validate_mesh(&bsp_flanged_bushing, Some(1));
    let vol_bsp = bsp_flanged_bushing.signed_volume();
    let err_bsp_exact_pct =
        ((vol_bsp - v_flanged_bushing_exact) / v_flanged_bushing_exact).abs() * 100.0;
    let err_bsp_poly_pct =
        ((vol_bsp - v_flanged_bushing_polygon) / v_flanged_bushing_polygon).abs() * 100.0;

    println!("--- [2] Option D Baseline: mcp-cad-core BSP-CSG (Flanged Bushing) ---");
    println!("  Boolean time:               {:.3} ms", dt_bsp_bool.as_secs_f64() * 1000.0);
    println!("  Signed volume:              {:.4} mm^3", vol_bsp);
    println!("  Error vs analytical circle: {:.4}% (vs 64-gon exact: {:.6}%)", err_bsp_exact_pct, err_bsp_poly_pct);
    println!("  Triangles / Vertices:       {} tris / {} verts", bsp_flanged_bushing.indices.len(), bsp_flanged_bushing.vertices.len());
    println!("  validate_mesh status:       {} (watertight={}, open_edges={}, non_manifold={})", val_bsp.status, val_bsp.topology.is_watertight, val_bsp.topology.boundary_edges_count, val_bsp.topology.non_manifold_edges_count);
    println!("  Topology (Euler / Genus):   chi = {}, genus = {:.1}", val_bsp.topology.euler_characteristic, val_bsp.topology.genus);
    println!("  Triangle angle quality:     min_angle = {:.3} deg, <1 deg = {}, <5 deg ratio = {:.2}%", val_bsp.angle_stats.min_angle_deg, val_bsp.geometry.lt_1_deg_count, val_bsp.geometry.lt_5_deg_ratio * 100.0);
    println!("  Topological Face Provenance: None (TriangleMesh has no face_id array yet; requires P2-T5..T7)");
    println!();

    // -------------------------------------------------------------------------
    // PART C: Bonus Stress-Test — T08_hex_nut_m10 Helical Thread Subtraction (P3-T11)
    // -------------------------------------------------------------------------
    use mcp_cad_core::primitives::{
        create_helical_cutter_with_settings, create_polygon_prism, cut_hole_with_settings,
    };
    use mcp_cad_core::tessellation::TessellationSettings;
    let settings = TessellationSettings::default();
    let hex_blank = create_polygon_prism(6, 0.008, Some(0.017), None, Some(0.0008));
    let nut_drilled = cut_hole_with_settings(
        &hex_blank,
        [0.0, 0.0, 0.005],
        [0.0, 0.0, 1.0],
        0.00425,
        0.020,
        None,
        None,
        &settings,
    );
    let turns = 0.010 / 0.0015;
    let thread_cutter = create_helical_cutter_with_settings(
        0.005,
        0.0008,
        0.0015,
        0.010,
        -0.005,
        1,
        true,
        turns,
        "metric_60",
        &settings,
    );

    let t_t08_m = Instant::now();
    let (m_nut, _) = triangle_mesh_to_manifold_with_faces(&nut_drilled, |_, _, _| 1);
    let (m_thread, _) = triangle_mesh_to_manifold_with_faces(&thread_cutter, |_, _, _| 2);
    let m_t08_res = m_nut.difference(&m_thread);
    let dt_t08_m = t_t08_m.elapsed();
    let (t08_m_mesh, _) = manifold_to_triangle_mesh(&m_t08_res);
    let val_t08_m = validate_mesh(&t08_m_mesh, Some(1));

    let t_t08_bsp = Instant::now();
    let bsp_t08_res = csg_difference(&nut_drilled, &thread_cutter);
    let dt_t08_bsp = t_t08_bsp.elapsed();
    let val_t08_bsp = validate_mesh(&bsp_t08_res, Some(1));

    println!("--- [3] Bonus Stress-Test: T08_hex_nut_m10 Helical Thread Subtraction (P3-T11) ---");
    println!(
        "  Input statuses:             m_nut={:?} (tris={}), m_thread={:?} (tris={})",
        m_nut.status(),
        m_nut.num_tri(),
        m_thread.status(),
        m_thread.num_tri()
    );
    println!(
        "  manifold-rust v0.19.2:      {:.3} ms | status={:?} | watertight={} (open_edges={}, non_manifold={}) | genus={:.1} | tris={}",
        dt_t08_m.as_secs_f64() * 1000.0,
        m_t08_res.status(),
        val_t08_m.topology.is_watertight,
        val_t08_m.topology.boundary_edges_count,
        val_t08_m.topology.non_manifold_edges_count,
        val_t08_m.topology.genus,
        t08_m_mesh.indices.len()
    );
    println!(
        "  mcp-cad-core BSP-CSG:       {:.3} ms | watertight={} (open_edges={}, non_manifold={}) | genus={:.1} | tris={}",
        dt_t08_bsp.as_secs_f64() * 1000.0,
        val_t08_bsp.topology.is_watertight,
        val_t08_bsp.topology.boundary_edges_count,
        val_t08_bsp.topology.non_manifold_edges_count,
        val_t08_bsp.topology.genus,
        bsp_t08_res.indices.len()
    );
}
