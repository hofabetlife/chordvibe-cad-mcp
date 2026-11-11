//! CSG Boolean & Hole Invariants Suite (Task P0-T7).
//!
//! Verifies inclusion-exclusion volume identities and strict `validate_mesh`
//! topological/geometric invariants for pairs of solids across `csg_union`,
//! `csg_difference`, `csg_intersection`, and `cut_hole`:
//!   1. `V(A ∪ B) + V(A ∩ B) = V(A) + V(B)` (within 1% tolerance)
//!   2. `V(A \ B) = V(A) - V(A ∩ B)` and `V(B \ A) = V(B) - V(A ∩ B)` (within 1% tolerance)
//!   3. Every non-empty result passes `validate_mesh` (`status == "PASS"`, watertight,
//!      consistent outward orientation, Euler characteristic $\chi = 2 - 2g$,
//!      zero T-junctions, zero self-intersections, and sliver quality gate).

use mcp_cad_core::csg::{csg_difference, csg_intersection, csg_union};
use mcp_cad_core::diagnostics::validate_mesh;
use mcp_cad_core::primitives::{
    create_box, create_cylinder, create_sphere, cut_hole, cut_hole_with_settings,
};
use mcp_cad_core::tessellation::TessellationSettings;
use mcp_cad_core::topology::TriangleMesh;

/// Helper that verifies `validate_mesh` and inclusion-exclusion volume identities for `(a, b)`.
fn verify_csg_pair_invariants(
    label: &str,
    a: &TriangleMesh,
    b: &TriangleMesh,
    expected_union_genus: Option<i32>,
    expected_diff_ab_genus: Option<i32>,
    expected_inter_genus: Option<i32>,
) {
    let va = a.signed_volume();
    let vb = b.signed_volume();
    assert!(va > 0.0, "{label}: V(A) must be positive, got {va}");
    assert!(vb > 0.0, "{label}: V(B) must be positive, got {vb}");

    let m_union = csg_union(a, b);
    let m_inter = csg_intersection(a, b);
    let m_diff_ab = csg_difference(a, b);
    let m_diff_ba = csg_difference(b, a);

    let v_union = if m_union.indices.is_empty() {
        0.0
    } else {
        m_union.signed_volume()
    };
    let v_inter = if m_inter.indices.is_empty() {
        0.0
    } else {
        m_inter.signed_volume()
    };
    let v_diff_ab = if m_diff_ab.indices.is_empty() {
        0.0
    } else {
        m_diff_ab.signed_volume()
    };
    let v_diff_ba = if m_diff_ba.indices.is_empty() {
        0.0
    } else {
        m_diff_ba.signed_volume()
    };

    // 1. V(A u B) + V(A n B) = V(A) + V(B) (within 1%)
    let lhs_ie = v_union + v_inter;
    let rhs_ie = va + vb;
    let rel_err_ie = (lhs_ie - rhs_ie).abs() / rhs_ie.max(1e-12);
    assert!(
        rel_err_ie <= 0.01,
        "{label}: inclusion-exclusion failed: V(A∪B)={v_union:.6e} + V(A∩B)={v_inter:.6e} = {lhs_ie:.6e} vs V(A)+V(B)={rhs_ie:.6e} (rel_err={:.4}%)",
        rel_err_ie * 100.0
    );

    // 2. V(A \ B) = V(A) - V(A n B) (within 1% of V(A))
    let expected_diff_ab = (va - v_inter).max(0.0);
    let rel_err_diff_ab = (v_diff_ab - expected_diff_ab).abs() / va.max(1e-12);
    assert!(
        rel_err_diff_ab <= 0.01,
        "{label}: difference A\\B volume failed: V(A\\B)={v_diff_ab:.6e} vs V(A)-V(A∩B)={expected_diff_ab:.6e} (rel_err={:.4}%)",
        rel_err_diff_ab * 100.0
    );

    // 2b. V(B \ A) = V(B) - V(A n B) (within 1% of V(B))
    let expected_diff_ba = (vb - v_inter).max(0.0);
    let rel_err_diff_ba = (v_diff_ba - expected_diff_ba).abs() / vb.max(1e-12);
    assert!(
        rel_err_diff_ba <= 0.01,
        "{label}: difference B\\A volume failed: V(B\\A)={v_diff_ba:.6e} vs V(B)-V(A∩B)={expected_diff_ba:.6e} (rel_err={:.4}%)",
        rel_err_diff_ba * 100.0
    );

    // 3. Validate each non-empty resulting mesh
    if !m_union.indices.is_empty() {
        let rep = validate_mesh(&m_union, expected_union_genus);
        assert_eq!(
            rep.status, "PASS",
            "{label} [A∪B]: validate_mesh failed: issues={:?}, boundary={}, non_manifold={}, t_junc={}, lt_1={}, lt_5={:.2}%",
            rep.issues,
            rep.topology.boundary_edges_count,
            rep.topology.non_manifold_edges_count,
            rep.geometry.t_junctions_count,
            rep.geometry.lt_1_deg_count,
            rep.geometry.lt_5_deg_ratio * 100.0
        );
        assert_eq!(
            rep.geometry.t_junctions_count, 0,
            "{label} [A∪B]: T-junctions must be 0"
        );
    }

    if !m_diff_ab.indices.is_empty() {
        let rep = validate_mesh(&m_diff_ab, expected_diff_ab_genus);
        assert_eq!(
            rep.status, "PASS",
            "{label} [A\\B]: validate_mesh failed: issues={:?}, boundary={}, non_manifold={}, t_junc={}, lt_1={}, lt_5={:.2}%",
            rep.issues,
            rep.topology.boundary_edges_count,
            rep.topology.non_manifold_edges_count,
            rep.geometry.t_junctions_count,
            rep.geometry.lt_1_deg_count,
            rep.geometry.lt_5_deg_ratio * 100.0
        );
        assert_eq!(
            rep.geometry.t_junctions_count, 0,
            "{label} [A\\B]: T-junctions must be 0"
        );
    }

    if !m_inter.indices.is_empty() {
        let rep = validate_mesh(&m_inter, expected_inter_genus);
        assert_eq!(
            rep.status, "PASS",
            "{label} [A∩B]: validate_mesh failed: issues={:?}, boundary={}, non_manifold={}, t_junc={}, lt_1={}, lt_5={:.2}%",
            rep.issues,
            rep.topology.boundary_edges_count,
            rep.topology.non_manifold_edges_count,
            rep.geometry.t_junctions_count,
            rep.geometry.lt_1_deg_count,
            rep.geometry.lt_5_deg_ratio * 100.0
        );
        assert_eq!(
            rep.geometry.t_junctions_count, 0,
            "{label} [A∩B]: T-junctions must be 0"
        );
    }
}

#[test]
fn csg_invariants_box_box_shifted() {
    // Case 1a: Coplanar top/bottom/front/back faces with X shift (0.05 m)
    let box_a = create_box(0.1, 0.1, 0.1);
    let mut box_b = create_box(0.1, 0.1, 0.1);
    box_b.translate([0.05, 0.0, 0.0]);
    verify_csg_pair_invariants("box_box_coplanar_x_shift", &box_a, &box_b, Some(0), Some(0), Some(0));

    // Case 1b: General 3D diagonal shift (no coplanar faces)
    let mut box_c = create_box(0.08, 0.08, 0.08);
    box_c.translate([0.03, 0.025, 0.02]);
    verify_csg_pair_invariants("box_box_diagonal_shift", &box_a, &box_c, Some(0), Some(0), Some(0));

    // Case 1c: Disjoint boxes -> empty intersection
    let mut box_disjoint = create_box(0.05, 0.05, 0.05);
    box_disjoint.translate([0.25, 0.0, 0.0]);
    let inter_disjoint = csg_intersection(&box_a, &box_disjoint);
    assert!(
        inter_disjoint.indices.is_empty(),
        "Disjoint box intersection must be empty"
    );
}

#[test]
fn csg_invariants_cylinder_box() {
    // Case 2a: Cylinder piercing through a flat box plate (genus 1 for A \ B)
    let plate = create_box(0.1, 0.1, 0.04);
    let cyl_thru = create_cylinder(0.015, 0.10, 32);
    verify_csg_pair_invariants(
        "cylinder_box_thru_hole",
        &plate,
        &cyl_thru,
        Some(0),
        Some(1),
        Some(0),
    );

    // Case 2b: Blind cylinder shifted up into top face of box (genus 0 for A \ B)
    let mut cyl_blind = create_cylinder(0.015, 0.04, 32);
    cyl_blind.translate([0.0, 0.0, 0.01]);
    verify_csg_pair_invariants(
        "cylinder_box_blind_hole",
        &plate,
        &cyl_blind,
        Some(0),
        Some(0),
        Some(0),
    );

    // Case 2c: Offset cylinder intersecting side edge of box
    let mut cyl_edge = create_cylinder(0.02, 0.08, 32);
    cyl_edge.translate([0.05, 0.0, 0.0]);
    verify_csg_pair_invariants(
        "cylinder_box_side_notch",
        &plate,
        &cyl_edge,
        Some(0),
        Some(0),
        Some(0),
    );
}

#[test]
fn csg_invariants_cylinder_cylinder_coaxial_and_offset() {
    // Case 3a: Coaxial cylinders (outer cylinder minus inner through-bore -> bushing, genus 1)
    let outer_cyl = create_cylinder(0.04, 0.06, 32);
    let inner_cyl = create_cylinder(0.02, 0.10, 32);
    verify_csg_pair_invariants(
        "cylinder_cylinder_coaxial_bushing",
        &outer_cyl,
        &inner_cyl,
        Some(0),
        Some(1),
        Some(0),
    );

    // Case 3b: Coaxial stepped shaft union & overlap
    let shaft_a = create_cylinder(0.04, 0.06, 32);
    let mut shaft_b = create_cylinder(0.025, 0.06, 32);
    shaft_b.translate([0.0, 0.0, 0.04]);
    verify_csg_pair_invariants(
        "cylinder_cylinder_coaxial_stepped",
        &shaft_a,
        &shaft_b,
        Some(0),
        Some(0),
        Some(0),
    );

    // Case 3c: Parallel offset cylinders overlapping laterally
    let cyl_a = create_cylinder(0.03, 0.06, 32);
    let mut cyl_b = create_cylinder(0.025, 0.08, 32);
    cyl_b.translate([0.025, 0.0, 0.0]);
    verify_csg_pair_invariants(
        "cylinder_cylinder_parallel_offset",
        &cyl_a,
        &cyl_b,
        Some(0),
        Some(0),
        Some(0),
    );
}

#[test]
fn csg_invariants_box_sphere() {
    // Case 4a: Box and sphere shifted to top face center (hemispherical dimple in A \ B)
    let cube = create_box(0.08, 0.08, 0.08);
    let mut sphere_top = create_sphere(0.025, 24, 16);
    sphere_top.translate([0.0, 0.0, 0.04]);
    verify_csg_pair_invariants(
        "box_sphere_top_face_dimple",
        &cube,
        &sphere_top,
        Some(0),
        Some(0),
        Some(0),
    );

    // Case 4b: Box and sphere shifted to corner (0.04, 0.04, 0.04)
    let mut sphere_corner = create_sphere(0.03, 24, 16);
    sphere_corner.translate([0.04, 0.04, 0.04]);
    verify_csg_pair_invariants(
        "box_sphere_corner_overlap",
        &cube,
        &sphere_corner,
        Some(0),
        Some(0),
        Some(0),
    );
}

#[test]
fn csg_invariants_cut_hole_all_modes() {
    let settings = TessellationSettings::medium();
    let plate = create_box(0.1, 0.1, 0.04);

    let m_thru = cut_hole_with_settings(
        &plate,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.015,
        0.10,
        None,
        None,
        &settings,
    );
    let r_thru = validate_mesh(&m_thru, Some(1));
    assert_eq!(
        r_thru.status, "PASS",
        "Thru hole failed: {:?}",
        r_thru.issues
    );

    let m_blind = cut_hole_with_settings(
        &plate,
        [0.0, 0.0, 0.01],
        [0.0, 0.0, 1.0],
        0.015,
        0.03,
        None,
        None,
        &settings,
    );
    let r_blind = validate_mesh(&m_blind, Some(0));
    assert_eq!(
        r_blind.status, "PASS",
        "Blind hole failed: {:?}",
        r_blind.issues
    );

    let m_key = cut_hole_with_settings(
        &plate,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.015,
        0.10,
        Some(0.006),
        Some(0.003),
        &settings,
    );
    let r_key = validate_mesh(&m_key, Some(1));
    assert_eq!(
        r_key.status, "PASS",
        "Keyway hole failed: {:?}",
        r_key.issues
    );

    let m_ang = cut_hole_with_settings(
        &plate,
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 1.0],
        0.01,
        0.20,
        None,
        None,
        &settings,
    );
    let r_ang = validate_mesh(&m_ang, Some(1));
    assert_eq!(
        r_ang.status, "PASS",
        "Angled hole failed: {:?}",
        r_ang.issues
    );
}

// ============================================================================
// P2-T7: NAMED FACES PROPAGATION THROUGH CSG AND TRANSFORM
// ============================================================================

fn assert_csg_faces_area_identity(mesh: &mcp_cad_core::topology::TriangleMesh, ctx: &str) {
    assert_eq!(
        mesh.face_ids.len(),
        mesh.indices.len(),
        "{ctx}: face_ids.len() must equal indices.len()"
    );
    let sum_areas: f64 = mesh.faces.iter().map(|f| f.area).sum();
    let total_area = mesh.surface_area();
    assert!(
        (sum_areas - total_area).abs() < 1e-9,
        "{ctx}: sum(face.area)={sum_areas} != surface_area()={total_area}"
    );
    let sum_tris: usize = mesh.faces.iter().map(|f| f.triangles_count).sum();
    assert_eq!(
        sum_tris,
        mesh.indices.len(),
        "{ctx}: sum(face.triangles_count)={sum_tris} != indices.len()={}",
        mesh.indices.len()
    );
}

#[test]
fn p2_t7_box_minus_cylindrical_hole_named_faces() {
    use mcp_cad_core::dag::{FeatureNode, FeatureOp, FeatureTree};
    use mcp_cad_core::topology::FaceKind;
    use std::f64::consts::PI;

    let w = 0.10;
    let h = 0.08;
    let d = 0.04;
    let r = 0.015;

    let mut tree = FeatureTree::new();
    let box_node = FeatureNode::new(
        "Plate",
        FeatureOp::AddBox {
            width: w,
            height: h,
            depth: d,
            position: None,
            rotation_deg: None,
        },
    );
    let hole_node = FeatureNode::new(
        "Center Bore",
        FeatureOp::CutHole {
            center: [0.0, 0.0, 0.0],
            normal: [0.0, 0.0, 1.0],
            radius: r,
            depth: d * 1.2,
            keyway_width: None,
            keyway_depth: None,
        },
    );
    let hole_uuid = hole_node.id.to_string();
    tree.add_node(box_node);
    tree.add_node(hole_node);

    let mesh = tree.evaluate().expect("evaluate box with through-hole");
    assert_csg_faces_area_identity(&mesh, "box_minus_through_hole");
    assert_eq!(
        mesh.faces.len(),
        7,
        "Expected 6 box faces + 1 hole_wall face, got: {:?}",
        mesh.faces.iter().map(|f| &f.name).collect::<Vec<_>>()
    );

    // Verify hole_wall:* face
    let hole_wall = mesh
        .faces
        .iter()
        .find(|f| f.name.starts_with("hole_wall:") || f.role == "hole_wall")
        .expect("hole_wall face must exist");
    assert_eq!(hole_wall.name, "hole_wall:feat_1");
    assert_eq!(hole_wall.kind, FaceKind::Cylinder);
    assert_eq!(hole_wall.origin_feature.as_deref(), Some(hole_uuid.as_str()));
    assert!(hole_wall.aliases.contains(&"hole_wall".to_string()));
    assert!(hole_wall.aliases.contains(&"bore".to_string()));
    assert!((hole_wall.radius.expect("radius") - r).abs() < 1e-12);
    let expected_wall_area = 2.0 * PI * r * d;
    let rel_wall_err = (hole_wall.area - expected_wall_area).abs() / expected_wall_area;
    assert!(
        rel_wall_err < 0.01,
        "hole_wall area {} differs from 2*pi*r*h={} by {:.4}%",
        hole_wall.area,
        expected_wall_area,
        rel_wall_err * 100.0
    );

    // Verify all 6 original faces of the box are preserved and top/bottom are cut by pi*r^2
    let expected_cut_cap_area = w * h - PI * r * r;
    for cap_name in ["top", "bottom"] {
        let cap = mesh
            .faces
            .iter()
            .find(|f| f.name == cap_name)
            .unwrap_or_else(|| panic!("box face '{cap_name}' must be preserved"));
        assert_eq!(cap.kind, FaceKind::Plane);
        let rel_cap_err = (cap.area - expected_cut_cap_area).abs() / expected_cut_cap_area;
        assert!(
            rel_cap_err < 0.01,
            "{cap_name} area {} differs from w*h - pi*r^2={} by {:.4}%",
            cap.area,
            expected_cut_cap_area,
            rel_cap_err * 100.0
        );
    }
    for (side_name, expected_side_area) in [
        ("right", h * d),
        ("left", h * d),
        ("front", w * d),
        ("back", w * d),
    ] {
        let side = mesh
            .faces
            .iter()
            .find(|f| f.name == side_name)
            .unwrap_or_else(|| panic!("box face '{side_name}' must be preserved"));
        assert!(
            (side.area - expected_side_area).abs() < 1e-9,
            "{side_name} area {} != expected {expected_side_area}",
            side.area
        );
    }
}

#[test]
fn p2_t7_blind_hole_and_keyway_hole_named_faces() {
    use mcp_cad_core::topology::FaceKind;

    let plate = create_box(0.10, 0.08, 0.04);

    // 1. Blind hole (depth = 0.02 from top z = +0.02 -> bottom at z = 0.0)
    let blind = cut_hole(
        &plate,
        [0.0, 0.0, 0.02],
        [0.0, 0.0, 1.0],
        0.012,
        0.02,
        None,
        None,
    );
    assert_csg_faces_area_identity(&blind, "blind_hole");
    assert_eq!(blind.faces.len(), 8, "6 box faces + hole_wall + hole_bottom");
    let hole_bottom = blind
        .faces
        .iter()
        .find(|f| f.role == "hole_bottom")
        .expect("hole_bottom must survive in blind hole");
    assert_eq!(hole_bottom.kind, FaceKind::Plane);
    assert!(
        (hole_bottom.normal_or_axis[2] - 1.0).abs() < 1e-6,
        "hole_bottom normal must point out of blind hole (+Z), got {:?}",
        hole_bottom.normal_or_axis
    );

    // 2. Through hole with DIN 6885 keyway slot
    let keyway = cut_hole(
        &plate,
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.015,
        0.05,
        Some(0.006),
        Some(0.003),
    );
    assert_csg_faces_area_identity(&keyway, "keyway_hole");
    assert!(keyway.faces.iter().any(|f| f.role == "hole_wall"));
    assert!(keyway.faces.iter().any(|f| f.role == "keyway_wall"));
}

#[test]
fn p2_t7_csg_difference_cut_prefix_and_transform_negative_scale() {
    use mcp_cad_core::topology::FaceKind;

    // 1. Direct csg_difference(box, cylinder) produces cut:side with inverted/cylindrical metadata
    let a = create_box(0.08, 0.08, 0.04);
    let b = create_cylinder(0.015, 0.06, 48);
    let diff = csg_difference(&a, &b);
    assert_csg_faces_area_identity(&diff, "raw_csg_difference");
    let cut_side = diff
        .faces
        .iter()
        .find(|f| f.name == "cut:side")
        .expect("cut:side face");
    assert_eq!(cut_side.kind, FaceKind::Cylinder);
    assert!(cut_side.aliases.contains(&"hole_wall".to_string()));

    // 2. Negative scale transform preserves face_ids, flips winding, and updates normals
    let mut mirrored = diff.clone();
    mirrored.scale([-1.0, 1.0, 1.0]);
    assert_csg_faces_area_identity(&mirrored, "mirrored_csg");
    assert!(mirrored.signed_volume() > 0.0);
    let right_after = mirrored.faces.iter().find(|f| f.name == "right").unwrap();
    // Original "right" was at x = +0.04 with normal +X; after X mirror it is at x = -0.04 with normal -X
    assert!((right_after.centroid[0] + 0.04).abs() < 1e-6);
    assert!((right_after.normal_or_axis[0] + 1.0).abs() < 1e-6);
}

#[test]
fn p2_t7_rebuild_determinism_of_face_ids_and_names() {
    use mcp_cad_core::dag::{FeatureNode, FeatureOp, FeatureTree};

    let mut tree = FeatureTree::new();
    tree.add_node(FeatureNode::new(
        "Base",
        FeatureOp::AddBox {
            width: 0.10,
            height: 0.08,
            depth: 0.03,
            position: None,
            rotation_deg: None,
        },
    ));
    tree.add_node(FeatureNode::new(
        "Boss",
        FeatureOp::AddCylinder {
            radius: 0.02,
            height: 0.03,
            segments: 32,
            position: Some([0.0, 0.0, 0.025]),
            rotation_deg: None,
        },
    ));
    tree.add_node(FeatureNode::new(
        "Bore",
        FeatureOp::CutHole {
            center: [0.0, 0.0, 0.0],
            normal: [0.0, 0.0, 1.0],
            radius: 0.01,
            depth: 0.10,
            keyway_width: None,
            keyway_depth: None,
        },
    ));
    tree.add_node(FeatureNode::new(
        "Shift",
        FeatureOp::Transform {
            translation: [0.05, -0.02, 0.01],
            rotation_deg: [0.0, 0.0, 90.0],
            scale: [1.0, 1.0, 1.0],
        },
    ));

    let eval1 = tree.evaluate().expect("eval 1");
    let eval2 = tree.evaluate().expect("eval 2");
    assert_eq!(eval1.face_ids, eval2.face_ids, "face_ids must be 100% deterministic across rebuilds");
    assert_eq!(eval1.faces, eval2.faces, "faces registry must be 100% deterministic across rebuilds");
}

