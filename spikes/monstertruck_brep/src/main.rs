//! Spike B: `monstertruck` B-Rep Kernel (`monstertruck-modeling` v0.4.1 + `monstertruck-solid` v0.4.1
//! + `monstertruck-fillet` v0.4.1 + `monstertruck-meshing` v0.4.1 + `monstertruck-io` v0.4.1, Apache-2.0)
//!
//! Geometry specification (P1-T3):
//! 1. Cylinder: R_out = 15 mm, h = 50 mm (z in [-25, +25] mm)
//! 2. Minus coaxial through-hole: D = 20 mm (R_in = 10 mm, h = 52 mm)
//! 3. Plus box-flange: 40 x 40 x 10 mm collar flange (z in [-20, -10] mm)
//! 4. Edge fillet R = 2 mm via native B-Rep rolling-ball `monstertruck_modeling::fillet_edges`
//! 5. Topological FaceID / EdgeID inspection for the top end-cap face (z = +25 mm) and STEP export.

use std::f64::consts::PI;
use std::time::Instant;

use mcp_cad_core::diagnostics::validate_mesh;
use mcp_cad_core::topology::{TriangleMesh, Vertex as CadVertex};
use monstertruck_io::step::save::{CompleteStepDisplay, StepHeaderDescriptor, StepModel};
use monstertruck_meshing::prelude::*;
use monstertruck_modeling::*;

/// Helper: build a B-Rep solid cylinder along the Z axis centered at `z in [-h/2, +h/2]`.
fn make_brep_cylinder(radius: f64, height: f64) -> Solid {
    let z0 = -0.5 * height;
    let v = builder::vertex(Point3::new(radius, 0.0, z0));
    let wire = builder::revolve(
        &v,
        Point3::new(0.0, 0.0, z0),
        Vector3::unit_z(),
        builder::SweepAngle::Closed,
        4,
    );
    let base_disk = builder::try_attach_plane(&[wire]).expect("valid circular wire");
    builder::extrude(&base_disk, Vector3::new(0.0, 0.0, height))
}

/// Helper: build a B-Rep solid box centered in XY and spanning `[z_min, z_max]`.
fn make_brep_box(width: f64, depth: f64, z_min: f64, z_max: f64) -> Solid {
    let bbox = BoundingBox::from_iter([
        Point3::new(-0.5 * width, -0.5 * depth, z_min),
        Point3::new(0.5 * width, 0.5 * depth, z_max),
    ]);
    primitive::cuboid(bbox)
}

/// Converts a `monstertruck_mesh::PolygonMesh` (after `.triangulation(tol).to_polygon()`)
/// into `mcp_cad_core::topology::TriangleMesh`.
fn polygon_mesh_to_cad_mesh(poly: &PolygonMesh) -> TriangleMesh {
    let positions = poly.positions();
    let mut vertices = Vec::with_capacity(positions.len());
    for p in positions {
        vertices.push(CadVertex::new([p.x, p.y, p.z], [0.0, 0.0, 1.0]));
    }
    let mut indices = Vec::new();
    for tri in poly.faces().triangle_iter() {
        indices.push([
            tri[0].pos as u32,
            tri[1].pos as u32,
            tri[2].pos as u32,
        ]);
    }
    let mut mesh = TriangleMesh { vertices, indices };
    mesh.recompute_vertex_normals();
    mesh
}

fn main() {
    println!("================================================================================");
    println!("P1-T3 SPIKE 2: monstertruck v0.4.1 (Pure Rust B-Rep Kernel, Apache-2.0)");
    println!("================================================================================");

    let r_out = 15.0; // mm
    let r_in = 10.0; // mm (D = 20 mm)
    let h_cyl = 50.0; // mm (z in [-25, +25] mm)
    let flange_w = 40.0; // mm
    let flange_z_min = -20.0; // mm
    let flange_z_max = -10.0; // mm
    let flange_h = flange_z_max - flange_z_min; // 10 mm
    let r_fillet = 2.0; // mm

    // Analytical volumes (exact continuous cylinder/hole)
    let v_bushing_exact = PI * (r_out * r_out - r_in * r_in) * h_cyl;
    let v_flange_add_exact = (flange_w * flange_w - PI * r_out * r_out) * flange_h;
    let v_flanged_bushing_exact = v_bushing_exact + v_flange_add_exact;
    let v_1_edge_fillet_exact = (1.0 - PI / 4.0) * r_fillet * r_fillet * flange_h;
    let v_4_vertical_fillets_exact = 4.0 * v_1_edge_fillet_exact;

    println!("Analytical Reference Volumes (mm^3):");
    println!("  Hollow bushing (R=15, r=10, h=50):       {:12.4} mm^3", v_bushing_exact);
    println!("  Flanged bushing (+ 40x40x10 flange):     {:12.4} mm^3", v_flanged_bushing_exact);
    println!("  With 1x R=2mm vertical edge fillet:      {:12.4} mm^3", v_flanged_bushing_exact - v_1_edge_fillet_exact);
    println!("  With 4x R=2mm vertical edge fillets:     {:12.4} mm^3", v_flanged_bushing_exact - v_4_vertical_fillets_exact);
    println!();

    // -------------------------------------------------------------------------
    // STEP 1 & 2: Construct B-Rep primitives & run B-Rep boolean operations
    // -------------------------------------------------------------------------
    let t_start = Instant::now();
    let cyl = make_brep_cylinder(r_out, h_cyl);
    let hole = make_brep_cylinder(r_in, h_cyl + 4.0);
    let flange = make_brep_box(flange_w, flange_w, flange_z_min, flange_z_max);

    // Tolerances in model units (mm):
    let bool_tol = 0.05;

    let t_bool1 = Instant::now();
    // Step 1: Cylinder minus coaxial hole -> Hollow bushing
    let bushing = monstertruck_solid::difference(&cyl, &hole, bool_tol)
        .expect("B-Rep difference(cyl, hole) succeeded");
    let dt_bool1 = t_bool1.elapsed();

    let t_bool2 = Instant::now();
    // Step 2: Union with box-flange and subtract coaxial hole
    let cyl_plus_flange = monstertruck_solid::or(&cyl, &flange, bool_tol)
        .expect("B-Rep union(cyl, flange) succeeded");
    let flanged_bushing = monstertruck_solid::difference(&cyl_plus_flange, &hole, bool_tol)
        .expect("B-Rep difference(cyl_plus_flange, hole) succeeded");
    let dt_bool_total = t_bool2.elapsed();

    println!("--- [1] B-Rep Construction & Boolean Operations (monstertruck-solid) ---");
    println!("  Step 1 (Cyl \\ Hole) time:   {:.3} ms (faces: {})", dt_bool1.as_secs_f64() * 1000.0, bushing.boundaries()[0].len());
    println!("  Step 2 (Flanged Bushing):   {:.3} ms (faces: {})", dt_bool_total.as_secs_f64() * 1000.0, flanged_bushing.boundaries()[0].len());

    // -------------------------------------------------------------------------
    // STEP 3: Inspect B-Rep Topological Naming (FaceID & EdgeID of Top End-Cap)
    // -------------------------------------------------------------------------
    let shell = &flanged_bushing.boundaries()[0];
    println!();
    println!("--- [2] B-Rep Topological Entities & End-Cap Face Identification ---");
    println!("  Total B-Rep faces in shell: {}", shell.len());

    let mut top_cap_faces = Vec::new();
    for (idx, face) in shell.face_iter().enumerate() {
        let fid = face.id();
        let boundaries = face.boundaries();
        let mut all_top_z = true;
        let mut v_count = 0usize;
        for wire in &boundaries {
            for edge in wire.edge_iter() {
                let p = edge.front().point();
                v_count += 1;
                if (p.z - 25.0).abs() > 1e-3 {
                    all_top_z = false;
                }
            }
        }
        if all_top_z && v_count > 0 {
            let surf = face.oriented_surface();
            let n = surf.normal(0.5, 0.5);
            top_cap_faces.push((idx, fid, boundaries.len(), v_count, n));
        }
    }

    for (idx, fid, wire_count, edge_count, normal) in &top_cap_faces {
        println!(
            "  Top End-Cap Face found: shell_index={}, FaceID={:?}, boundary_wires={} (outer R=15 + inner hole R=10), edges={}, normal=[{:.3}, {:.3}, {:.3}]",
            idx, fid, wire_count, edge_count, normal.x, normal.y, normal.z
        );
    }

    let orig_cyl_top_face_ids: Vec<_> = cyl.boundaries()[0]
        .face_iter()
        .filter(|f| {
            f.boundaries()
                .iter()
                .flat_map(|w| w.edge_iter())
                .all(|e| (e.front().point().z - 25.0).abs() < 1e-3)
        })
        .map(|f| f.id())
        .collect();
    println!("  Original Cylinder Top Cap FaceID(s): {:?}", orig_cyl_top_face_ids);

    // -------------------------------------------------------------------------
    // STEP 4: Tessellation & Mesh Validation (monstertruck-meshing -> validate_mesh)
    // -------------------------------------------------------------------------
    println!();
    println!("--- [3] B-Rep Tessellation & Mesh Diagnostics (tol = 0.05 mm) ---");
    let t_tess = Instant::now();
    let poly_mesh = flanged_bushing.robust_triangulation(0.05).to_polygon();
    let dt_tess = t_tess.elapsed();
    let cad_mesh = polygon_mesh_to_cad_mesh(&poly_mesh);
    let val = validate_mesh(&cad_mesh, Some(1));
    let vol_tess = cad_mesh.signed_volume();
    let vol_poly_direct = poly_mesh.volume();
    let err_pct = ((vol_tess - v_flanged_bushing_exact) / v_flanged_bushing_exact).abs() * 100.0;

    println!("  Tessellation time:          {:.3} ms (end-to-end: {:.3} ms)", dt_tess.as_secs_f64() * 1000.0, t_start.elapsed().as_secs_f64() * 1000.0);
    println!("  PolygonMesh.volume():       {:.4} mm^3", vol_poly_direct);
    println!("  TriangleMesh.signed_volume: {:.4} mm^3 (analytical: {:.4} mm^3, err = {:.4}%)", vol_tess, v_flanged_bushing_exact, err_pct);
    println!("  Triangles / Vertices:       {} tris / {} verts", cad_mesh.indices.len(), cad_mesh.vertices.len());
    println!("  validate_mesh status:       {} (watertight={}, open_edges={}, non_manifold={})", val.status, val.topology.is_watertight, val.topology.boundary_edges_count, val.topology.non_manifold_edges_count);
    println!("  Topology (Euler / Genus):   chi = {}, genus = {:.1}", val.topology.euler_characteristic, val.topology.genus);
    println!("  Triangle angle quality:     min_angle = {:.3} deg, <1 deg = {}, <5 deg ratio = {:.2}%", val.angle_stats.min_angle_deg, val.geometry.lt_1_deg_count, val.geometry.lt_5_deg_ratio * 100.0);

    // -------------------------------------------------------------------------
    // STEP 5: B-Rep Edge Fillet R = 2 mm (monstertruck-fillet)
    // -------------------------------------------------------------------------
    println!();
    println!("--- [4] B-Rep Rolling-Ball Edge Fillet R = 2 mm (monstertruck-fillet) ---");
    let mut filleted_shell = flanged_bushing.boundaries()[0].clone();
    let vertical_flange_edges: Vec<_> = filleted_shell
        .edge_iter()
        .filter(|e| {
            let p = e.front().point();
            let q = e.back().point();
            (p.x.abs() - 20.0).abs() < 1e-3
                && (p.y.abs() - 20.0).abs() < 1e-3
                && (q.x.abs() - 20.0).abs() < 1e-3
                && (q.y.abs() - 20.0).abs() < 1e-3
                && (p.z - q.z).abs() > 5.0
        })
        .collect();
    let mut unique_vertical_edges = Vec::new();
    for e in vertical_flange_edges {
        if !unique_vertical_edges.iter().any(|u: &Edge| u.id() == e.id()) {
            unique_vertical_edges.push(e);
        }
    }
    println!("  Found {} unique vertical flange corner edges (L=10 mm each)", unique_vertical_edges.len());

    let fillet_opts = FilletOptions {
        radius: RadiusSpec::Constant(r_fillet),
        ..Default::default()
    };
    let t_fillet = Instant::now();
    let fillet_res = fillet_edges(
        &mut filleted_shell,
        &unique_vertical_edges[..1],
        Some(&fillet_opts),
    );
    let dt_fillet = t_fillet.elapsed();
    match fillet_res {
        Ok(()) => {
            let poly_fillet = filleted_shell.robust_triangulation(0.05).to_polygon();
            let cad_fillet = polygon_mesh_to_cad_mesh(&poly_fillet);
            let val_f = validate_mesh(&cad_fillet, Some(1));
            let vol_f = cad_fillet.signed_volume();
            let expected_1_fillet = v_flanged_bushing_exact - v_1_edge_fillet_exact;
            println!(
                "  1-edge R=2mm fillet status: OK in {:.3} ms (shell faces: {} -> {})",
                dt_fillet.as_secs_f64() * 1000.0,
                flanged_bushing.boundaries()[0].len(),
                filleted_shell.len()
            );
            println!(
                "  Filleted volume:            {:.4} mm^3 (analytical: {:.4} mm^3, err = {:.4}%)",
                vol_f,
                expected_1_fillet,
                ((vol_f - expected_1_fillet) / expected_1_fillet).abs() * 100.0
            );
            println!(
                "  Filleted mesh diagnostics:  tris={}, watertight={}, open_edges={}, non_manifold={}, min_angle={:.3} deg",
                cad_fillet.indices.len(),
                val_f.topology.is_watertight,
                val_f.topology.boundary_edges_count,
                val_f.topology.non_manifold_edges_count,
                val_f.angle_stats.min_angle_deg
            );
        }
        Err(e) => {
            println!("  1-edge R=2mm fillet error: {:?}", e);
        }
    }

    // Also test filleting a top horizontal edge of the flange (L=40 mm) and the circular cylinder-flange intersection edge
    let cyl_flange_intersection_edges: Vec<_> = flanged_bushing.boundaries()[0]
        .edge_iter()
        .filter(|e| {
            let p = e.front().point();
            let q = e.back().point();
            let rp = (p.x * p.x + p.y * p.y).sqrt();
            let rq = (q.x * q.x + q.y * q.y).sqrt();
            (p.z - (-10.0)).abs() < 1e-2 && (q.z - (-10.0)).abs() < 1e-2 && (rp - 15.0).abs() < 1e-2 && (rq - 15.0).abs() < 1e-2
        })
        .collect();
    let mut unique_neck_edges = Vec::new();
    for e in cyl_flange_intersection_edges {
        if !unique_neck_edges.iter().any(|u: &Edge| u.id() == e.id()) {
            unique_neck_edges.push(e);
        }
    }
    let mut neck_shell = flanged_bushing.boundaries()[0].clone();
    let t_neck = Instant::now();
    let neck_res = fillet_edges(&mut neck_shell, &unique_neck_edges, Some(&fillet_opts));
    println!(
        "  Circular cylinder-flange neck edge (IntersectionCurve, {} edges) R=2mm fillet: {:?} ({:.3} ms)",
        unique_neck_edges.len(),
        neck_res.as_ref().map(|_| "OK"),
        t_neck.elapsed().as_secs_f64() * 1000.0
    );

    // -------------------------------------------------------------------------
    // STEP 6: ISO 10303-21 STEP Export (monstertruck-io)
    // -------------------------------------------------------------------------
    println!();
    println!("--- [5] ISO 10303-21 STEP Export (monstertruck-io) ---");
    let t_step = Instant::now();
    let compressed = flanged_bushing.compress();
    let step_string = CompleteStepDisplay::new(
        StepModel::from(&compressed),
        StepHeaderDescriptor {
            file_name: "flanged_bushing.step".to_string(),
            ..Default::default()
        },
    )
    .to_string();
    let dt_step = t_step.elapsed();
    println!(
        "  STEP export succeeded in {:.3} ms: {} bytes, {} lines (starts with {:?})",
        dt_step.as_secs_f64() * 1000.0,
        step_string.len(),
        step_string.lines().count(),
        step_string.lines().next().unwrap_or("")
    );
}
