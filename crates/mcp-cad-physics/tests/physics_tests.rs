//! Integration tests for mcp-cad-physics engine.

use mcp_cad_core::dag::FeatureOp;
use mcp_cad_core::material::MaterialRegistry;
use mcp_cad_core::Part;
use mcp_cad_physics::joint::{CadJointType, MotorConfig};
use mcp_cad_physics::rigid_body::ColliderType;
use mcp_cad_physics::simulator::PhysicsWorld;

/// Helper to create a standard rectangular CAD box part.
fn create_test_box(id: &str, name: &str, w: f64, h: f64, d: f64) -> Part {
    let registry = MaterialRegistry::new();
    let steel = registry.get("steel_structural_aisi1020").unwrap().clone();
    let mut part = Part::new(id, name, steel);
    part.dag.add_feature(
        FeatureOp::AddBox {
            width: w,
            height: h,
            depth: d,
            position: None,
            rotation_deg: None,
        },
        None,
    );
    part.rebuild().expect("rebuild box should succeed");
    part
}

/// Helper to create a standard CAD cylinder part.
fn create_test_cylinder(id: &str, name: &str, radius: f64, height: f64) -> Part {
    let registry = MaterialRegistry::new();
    let aluminum = registry.get("aluminum_6061_t6").unwrap().clone();
    let mut part = Part::new(id, name, aluminum);
    part.dag.add_feature(
        FeatureOp::AddCylinder {
            radius,
            height,
            segments: 24,
            position: None,
            rotation_deg: None,
        },
        None,
    );
    part.rebuild().expect("rebuild cylinder should succeed");
    part
}

/// Helper to create a standard CAD sphere part.
fn create_test_sphere(id: &str, name: &str, radius: f64) -> Part {
    let registry = MaterialRegistry::new();
    let steel = registry.get("steel_structural_aisi1020").unwrap().clone();
    let mut part = Part::new(id, name, steel);
    part.dag.add_feature(
        FeatureOp::AddSphere {
            radius,
            sectors: 16,
            stacks: 16,
            position: None,
            rotation_deg: None,
        },
        None,
    );
    part.rebuild().expect("rebuild sphere should succeed");
    part
}

#[test]
fn test_free_fall_under_gravity() {
    let mut world = PhysicsWorld::new();
    // g = -9.81 m/s² along Y
    assert!((world.gravity()[1] - (-9.81)).abs() < 1e-6);

    let box_part = create_test_box("box_falling", "Falling Weight", 0.2, 0.2, 0.2);
    let initial_y = 10.0;

    world
        .add_part_with_options(
            &box_part,
            true,
            ColliderType::ConvexHull,
            Some([0.0, initial_y, 0.0]),
            None,
        )
        .expect("part should be added");

    let duration_s = 1.0;
    let step_size_s = 0.005;
    let summary = world.simulate(duration_s, step_size_s);

    assert!(summary.success);
    assert!(summary.step_count > 0);

    let final_pos = summary
        .final_position("box_falling")
        .expect("trajectory should exist");
    let final_vel = summary
        .final_linear_velocity("box_falling")
        .expect("final velocity should exist");

    // Analytical free fall equations:
    // v(t) = g * t = -9.81 * 1.0 = -9.81 m/s
    // y(t) = y0 + 0.5 * g * t² = 10.0 - 0.5 * 9.81 * 1.0 = 5.095 m
    let expected_vy = -9.81;
    let expected_y = initial_y - 0.5 * 9.81 * duration_s * duration_s;

    // Small integration tolerance for discrete Euler integration
    assert!(
        (final_vel[1] - expected_vy).abs() < 0.15,
        "Actual vy = {}, Expected vy = {}",
        final_vel[1],
        expected_vy
    );
    assert!(
        (final_pos[1] - expected_y).abs() < 0.15,
        "Actual y = {}, Expected y = {}",
        final_pos[1],
        expected_y
    );

    // Lateral position and velocities should remain zero
    assert!(final_pos[0].abs() < 1e-3);
    assert!(final_pos[2].abs() < 1e-3);
    assert!(final_vel[0].abs() < 1e-3);
    assert!(final_vel[2].abs() < 1e-3);
}

#[test]
fn test_revolute_joint_with_motor() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]); // Zero gravity for pure kinematic test

    // Base body (fixed in space, positioned below the rotating arm)
    let base = create_test_box("base", "Fixed Base", 0.4, 0.4, 0.4);
    world
        .add_part_with_options(
            &base,
            false,
            ColliderType::Cube,
            Some([0.0, -0.3, 0.0]),
            None,
        )
        .expect("base added");

    // Rotating arm / flywheel (dynamic)
    let arm = create_test_box("arm", "Rotating Arm", 1.0, 0.1, 0.1);
    world
        .add_part_with_options(
            &arm,
            true,
            ColliderType::ConvexHull,
            Some([0.5, 0.0, 0.0]),
            None,
        )
        .expect("arm added");

    let target_omega = 2.0; // rad/s
    let joint_type = CadJointType::Revolute {
        anchor_a: [0.0, 0.3, 0.0],
        anchor_b: [-0.5, 0.0, 0.0],
        axis: [0.0, 1.0, 0.0], // Rotation around Y-axis
        limits: None,
    };
    let motor = MotorConfig::new(target_omega, 500.0);

    world
        .add_joint("joint_rev", "base", "arm", joint_type, Some(motor))
        .expect("revolute joint added");

    let summary = world.simulate(1.0, 0.005);
    assert!(summary.success);

    let final_angvel = world
        .part_angular_velocity("arm")
        .expect("arm angular velocity");

    // The angular velocity around Y axis should reach target_omega (2.0 rad/s)
    assert!(
        (final_angvel[1] - target_omega).abs() < 0.15,
        "Actual wy = {}, Target wy = {}",
        final_angvel[1],
        target_omega
    );

    // Other axes should remain near zero
    assert!(final_angvel[0].abs() < 0.05);
    assert!(final_angvel[2].abs() < 0.05);

    // Base must stay completely fixed
    let base_pos = world.part_position("base").unwrap();
    assert!(base_pos[0].abs() < 1e-4);
    assert!((base_pos[1] - (-0.3)).abs() < 1e-4);
    assert!(base_pos[2].abs() < 1e-4);
}

#[test]
fn test_prismatic_joint_limits() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    let base = create_test_box("base_rail", "Rail", 2.0, 0.2, 0.2);
    world
        .add_part_with_options(
            &base,
            false,
            ColliderType::Cube,
            Some([0.0, 0.0, 0.0]),
            None,
        )
        .expect("rail added");

    let slider = create_test_box("slider", "Slider Carriage", 0.2, 0.2, 0.2);
    world
        .add_part_with_options(
            &slider,
            true,
            ColliderType::Cube,
            Some([0.0, 0.0, 0.0]),
            None,
        )
        .expect("slider added");

    let limit_min = -0.15;
    let limit_max = 0.35;
    let joint_type = CadJointType::Prismatic {
        anchor_a: [0.0, 0.0, 0.0],
        anchor_b: [0.0, 0.0, 0.0],
        axis: [1.0, 0.0, 0.0], // Movement along X-axis
        limits: Some([limit_min, limit_max]),
    };

    // Drive forward aggressively to hit upper limit
    let motor = MotorConfig::new(5.0, 1000.0);

    world
        .add_joint("joint_prism", "base_rail", "slider", joint_type, Some(motor))
        .expect("prismatic joint added");

    let summary = world.simulate(0.5, 0.005);
    assert!(summary.success);

    let slider_pos = world.part_position("slider").unwrap();

    // Slider should hit the upper limit of 0.35 m (with constraint solver compliance margin)
    assert!(
        slider_pos[0] <= limit_max + 0.015 && slider_pos[0] >= limit_max - 0.05,
        "Slider pos X = {} not clamped at limit_max = {}",
        slider_pos[0],
        limit_max
    );

    // Lateral displacements Y and Z must remain locked at 0.0
    assert!(
        slider_pos[1].abs() < 1e-3,
        "Slider lateral drift Y = {}",
        slider_pos[1]
    );
    assert!(
        slider_pos[2].abs() < 1e-3,
        "Slider lateral drift Z = {}",
        slider_pos[2]
    );
}

#[test]
fn test_collision_between_bodies() {
    let mut world = PhysicsWorld::new(); // gravity = -9.81 along Y

    // Fixed floor
    let floor = create_test_box("floor", "Ground Floor", 10.0, 0.2, 10.0);
    world
        .add_part_with_options(
            &floor,
            false,
            ColliderType::Cube,
            Some([0.0, 0.0, 0.0]),
            None,
        )
        .expect("floor added");

    // Dynamic box dropped from y = 2.0 m
    let drop_box = create_test_box("drop_box", "Falling Box", 0.4, 0.4, 0.4);
    world
        .add_part_with_options(
            &drop_box,
            true,
            ColliderType::Cube,
            Some([0.0, 2.0, 0.0]),
            None,
        )
        .expect("falling box added");

    let summary = world.simulate(1.2, 0.01);
    assert!(summary.success);

    // Collision should be detected between floor and drop_box
    assert!(
        summary.has_collisions(),
        "Collisions should have been detected"
    );

    let floor_box_collisions: Vec<_> = summary
        .collisions
        .iter()
        .filter(|c| {
            (c.body_a == "drop_box" && c.body_b == "floor")
                || (c.body_a == "floor" && c.body_b == "drop_box")
        })
        .collect();

    assert!(
        !floor_box_collisions.is_empty(),
        "Collision record between drop_box and floor must exist"
    );

    // Box should rest on or bounce above floor (floor top surface is at y = 0.1, box half height is 0.2)
    // Box center should be >= 0.25 m, NOT below floor
    let final_pos = world.part_position("drop_box").unwrap();
    assert!(
        final_pos[1] >= 0.20,
        "Box penetrated floor: actual y = {}",
        final_pos[1]
    );
}

#[test]
fn test_collider_types_support() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    let box_part = create_test_box("p_box", "Box", 0.5, 0.5, 0.5);
    world
        .add_part_with_options(&box_part, true, ColliderType::Cube, None, None)
        .expect("cube collider");

    let sphere_part = create_test_sphere("p_sphere", "Sphere", 0.3);
    world
        .add_part_with_options(&sphere_part, true, ColliderType::Sphere, None, None)
        .expect("sphere collider");

    let cylinder_part = create_test_cylinder("p_cylinder", "Cylinder", 0.25, 0.8);
    world
        .add_part_with_options(&cylinder_part, true, ColliderType::Cylinder, None, None)
        .expect("cylinder collider");

    let trimesh_part = create_test_box("p_trimesh", "TriMesh Static", 1.0, 1.0, 1.0);
    world
        .add_part_with_options(&trimesh_part, false, ColliderType::TriMesh, None, None)
        .expect("trimesh collider");

    let compound_part = create_test_box("p_compound", "Compound Decomposition", 0.5, 0.5, 0.5);
    world
        .add_part_with_options(&compound_part, true, ColliderType::Compound, None, None)
        .expect("compound collider");

    let summary = world.simulate(0.1, 0.01);
    assert!(summary.success);
    assert_eq!(summary.trajectories.len(), 5);
}

#[test]
fn test_joint_runtime_motor_and_reactions() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    let base = create_test_box("j_base", "Base", 0.5, 0.5, 0.5);
    let rotor = create_test_cylinder("j_rotor", "Rotor", 0.2, 0.5);

    world
        .add_part(&base, false)
        .expect("base added");
    world
        .add_part_with_options(&rotor, true, ColliderType::Cylinder, Some([0.0, 1.0, 0.0]), None)
        .expect("rotor added");

    let joint_type = CadJointType::Revolute {
        anchor_a: [0.0, 0.5, 0.0],
        anchor_b: [0.0, -0.5, 0.0],
        axis: [0.0, 1.0, 0.0],
        limits: None,
    };

    world
        .add_joint("motor_joint", "j_base", "j_rotor", joint_type, None)
        .expect("joint added without initial motor");

    // Step a bit with no motor
    world.simulate(0.1, 0.01);
    let w_init = world.part_angular_velocity("j_rotor").unwrap();
    assert!(w_init[1].abs() < 1e-4);

    // Now actuate with motor at runtime
    world
        .set_joint_motor("motor_joint", MotorConfig::new(4.0, 100.0))
        .expect("motor activated");

    let summary = world.simulate(0.5, 0.01);
    let w_motor = world.part_angular_velocity("j_rotor").unwrap();
    assert!(
        (w_motor[1] - 4.0).abs() < 0.2,
        "Rotor wy = {}, expected 4.0",
        w_motor[1]
    );

    // Disable motor
    world
        .disable_joint_motor("motor_joint")
        .expect("motor disabled");

    // Verify peak forces are recorded
    assert!(summary.peak_joint_forces.contains_key("motor_joint"));
}

#[test]
fn test_prismatic_joint_runtime_motor_and_disable_reenable() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    let rail = create_test_box("p_rail", "Rail", 1.0, 0.2, 0.2);
    let carriage = create_test_box("p_carriage", "Carriage", 0.2, 0.2, 0.2);

    world.add_part(&rail, false).expect("rail added");
    world
        .add_part_with_options(&carriage, true, ColliderType::Cube, Some([0.0, 0.5, 0.0]), None)
        .expect("carriage added");

    let joint_type = CadJointType::Prismatic {
        anchor_a: [0.0, 0.5, 0.0],
        anchor_b: [0.0, 0.0, 0.0],
        axis: [1.0, 0.0, 0.0],
        limits: Some([-2.0, 2.0]),
    };

    world
        .add_joint("slider_joint", "p_rail", "p_carriage", joint_type, None)
        .expect("prismatic joint added");

    // Step without motor so body can settle/sleep
    world.simulate(2.5, 0.01);
    let v_idle = world.part_linear_velocity("p_carriage").unwrap();
    assert!(v_idle[0].abs() < 1e-4);

    // Enable prismatic motor on JointAxis::LinX at runtime (must wake sleeping body and set max_force)
    world
        .set_joint_motor("slider_joint", MotorConfig::new(1.5, 500.0))
        .expect("prismatic motor activated");

    world.simulate(0.4, 0.01);
    let v_active = world.part_linear_velocity("p_carriage").unwrap();
    assert!(
        (v_active[0] - 1.5).abs() < 0.15,
        "Carriage vx = {}, expected ~1.5 m/s",
        v_active[0]
    );
    let pos_after_fwd = world.part_position("p_carriage").unwrap();
    assert!(
        pos_after_fwd[0] > 0.3,
        "Carriage x = {}, expected > 0.3 m",
        pos_after_fwd[0]
    );

    // Disable motor (sets max_force = 0.0) and then re-enable in reverse (-1.5 m/s)
    world
        .disable_joint_motor("slider_joint")
        .expect("prismatic motor disabled");
    world
        .set_joint_motor("slider_joint", MotorConfig::new(-1.5, 500.0))
        .expect("prismatic motor re-enabled in reverse");

    world.simulate(0.4, 0.01);
    let v_reverse = world.part_linear_velocity("p_carriage").unwrap();
    assert!(
        (v_reverse[0] + 1.5).abs() < 0.15,
        "Carriage reverse vx = {}, expected ~-1.5 m/s after disable/re-enable",
        v_reverse[0]
    );
}

#[test]
fn test_p2_t17_set_joint_motor_transactional_and_gear_build_motor_rejected() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    let part_a = create_test_box("p_a", "Part A", 0.2, 0.2, 0.2);
    let part_b = create_test_box("p_b", "Part B", 0.2, 0.2, 0.2);
    world.add_part(&part_a, false).expect("part_a added");
    world.add_part(&part_b, true).expect("part_b added");

    let motor = MotorConfig::new(5.0, 100.0);

    // 1. Building or adding Spherical, Fixed, or Gear joint with initial motor must fail
    for (j_id, j_type) in [
        (
            "bad_sph",
            CadJointType::Spherical {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
            },
        ),
        (
            "bad_fix",
            CadJointType::Fixed {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
            },
        ),
        (
            "bad_gear",
            CadJointType::Gear {
                ratio: 2.0,
                axis_a: [0.0, 0.0, 1.0],
                axis_b: [0.0, 0.0, 1.0],
            },
        ),
    ] {
        assert!(
            j_type.build_generic_joint(Some(motor)).is_err(),
            "build_generic_joint(Some(motor)) must fail for {j_id}"
        );
        assert!(
            world
                .add_joint(j_id, "p_a", "p_b", j_type, Some(motor))
                .is_err(),
            "add_joint with motor must fail for {j_id}"
        );
        assert!(
            !world.joints().contains_key(j_id),
            "Failed add_joint must not insert '{j_id}' into world.joints()"
        );
    }

    // 2. Add valid Spherical, Fixed, and Gear joints without motor, then verify set_joint_motor
    //    fails transactionally without mutating record.motor
    world
        .add_joint(
            "j_sph",
            "p_a",
            "p_b",
            CadJointType::Spherical {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
            },
            None,
        )
        .expect("spherical joint added");
    world
        .add_joint(
            "j_fix",
            "p_a",
            "p_b",
            CadJointType::Fixed {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
            },
            None,
        )
        .expect("fixed joint added");
    world
        .add_joint(
            "j_gear",
            "p_a",
            "p_b",
            CadJointType::Gear {
                ratio: 0.5,
                axis_a: [0.0, 0.0, 1.0],
                axis_b: [0.0, 0.0, 1.0],
            },
            None,
        )
        .expect("gear joint added");

    for j_id in ["j_sph", "j_fix", "j_gear"] {
        let err = world
            .set_joint_motor(j_id, motor)
            .expect_err("set_joint_motor must fail on unsupported joint type");
        assert!(
            err.to_string().contains("motor") || err.to_string().contains("Motor"),
            "Unexpected error message for {j_id}: {err}"
        );
        assert_eq!(
            world.joints()[j_id].motor,
            None,
            "Transactional guarantee violated: record.motor must remain None after failed set_joint_motor on {j_id}"
        );
    }
}

#[test]
fn test_p2_t18_update_part_collider_preserves_joints_and_updates_mass_and_bounds() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    let base = create_test_box("base_fixed", "Fixed Base", 0.4, 0.4, 0.4);
    let mut rotor = create_test_cylinder("rotor_dyn", "Rotor", 0.1, 0.2);
    let follower = create_test_cylinder("follower_dyn", "Follower", 0.1, 0.2);

    world.add_part(&base, false).expect("base added");
    let initial_rb_handle = world.add_part(&rotor, true).expect("rotor added");
    world.add_part(&follower, true).expect("follower added");

    let rev_motor = MotorConfig::new(4.0, 500.0);
    world
        .add_joint(
            "j_rev_drive",
            "base_fixed",
            "rotor_dyn",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(rev_motor),
        )
        .expect("revolute joint added");

    world
        .add_joint(
            "j_gear_couple",
            "rotor_dyn",
            "follower_dyn",
            CadJointType::Gear {
                ratio: 0.5,
                axis_a: [0.0, 0.0, 1.0],
                axis_b: [0.0, 0.0, 1.0],
            },
            None,
        )
        .expect("gear joint added");

    let mass_before = world.part_mass("rotor_dyn").expect("initial mass");
    let (aabb_min_before, aabb_max_before) = world
        .part_collider_aabb("rotor_dyn")
        .expect("initial collider aabb");
    assert!(mass_before > 0.0);
    assert!((aabb_max_before[0] - aabb_min_before[0] - 0.2).abs() < 0.02);

    // Enlarge rotor geometry (add a wider cylinder R=0.25, H=0.4) and update its collider in place
    rotor.dag.add_feature(
        FeatureOp::AddCylinder {
            radius: 0.25,
            height: 0.4,
            segments: 24,
            position: None,
            rotation_deg: None,
        },
        None,
    );
    rotor.rebuild().expect("rebuild enlarged rotor");

    let updated_rb_handle = world
        .update_part_collider(&rotor)
        .expect("update_part_collider should succeed");
    assert_eq!(
        updated_rb_handle, initial_rb_handle,
        "update_part_collider must preserve the existing BodyHandle"
    );
    assert_eq!(
        world.part_body_handle("rotor_dyn").unwrap(),
        initial_rb_handle
    );

    // Verify mass and collider AABB increased to match the new geometry
    let mass_after = world.part_mass("rotor_dyn").expect("updated mass");
    let (aabb_min_after, aabb_max_after) = world
        .part_collider_aabb("rotor_dyn")
        .expect("updated collider aabb");
    assert!(
        mass_after > mass_before * 4.0,
        "Expected updated mass ({mass_after}) > 4x initial mass ({mass_before})"
    );
    assert!(
        (aabb_max_after[0] - aabb_min_after[0] - 0.5).abs() < 0.03,
        "Expected updated collider X span ~0.5 m, got [{}, {}]",
        aabb_min_after[0],
        aabb_max_after[0]
    );

    // Verify both joints and motor are preserved and active during simulation
    assert_eq!(world.joints().len(), 2);
    assert_eq!(world.joints()["j_rev_drive"].motor, Some(rev_motor));
    assert!(world.joints().contains_key("j_gear_couple"));

    let summary = world.simulate(0.5, 0.005);
    assert!(summary.success);
    let w_rotor = world.part_angular_velocity("rotor_dyn").unwrap();
    let w_follower = world.part_angular_velocity("follower_dyn").unwrap();
    assert!(
        (w_rotor[2] - 4.0).abs() < 0.3,
        "Rotor wz = {}, expected ~4.0 rad/s",
        w_rotor[2]
    );
    assert!(
        (w_follower[2] - (-2.0)).abs() < 0.3,
        "Follower wz = {}, expected ~-2.0 rad/s via preserved gear constraint",
        w_follower[2]
    );

    // Verify transactional error when called with an empty-mesh part: existing collider & joints remain intact
    let registry = MaterialRegistry::new();
    let steel = registry.get("steel_structural_aisi1020").unwrap().clone();
    let empty_rotor = Part::new("rotor_dyn", "Empty Rotor", steel);
    assert!(
        world.update_part_collider(&empty_rotor).is_err(),
        "update_part_collider on empty mesh must return error"
    );
    assert_eq!(world.joints().len(), 2);
    let (aabb_min_still, aabb_max_still) = world
        .part_collider_aabb("rotor_dyn")
        .expect("collider must remain intact after failed update");
    assert!((aabb_max_still[0] - aabb_min_still[0] - 0.5).abs() < 0.03);
}

#[test]
fn test_p2_t22_gear_joint_validation() {
    // 1. Zero ratio rejection
    let gear_zero = CadJointType::Gear {
        ratio: 0.0,
        axis_a: [0.0, 0.0, 1.0],
        axis_b: [0.0, 0.0, 1.0],
    };
    let err = gear_zero.build_generic_joint(None).unwrap_err();
    assert!(
        err.to_string().contains("Gear ratio must be a non-zero finite number"),
        "Unexpected error message: {err}"
    );

    // 2. NaN ratio rejection
    let gear_nan = CadJointType::Gear {
        ratio: f64::NAN,
        axis_a: [0.0, 0.0, 1.0],
        axis_b: [0.0, 0.0, 1.0],
    };
    assert!(gear_nan.build_generic_joint(None).is_err());

    // 3. Inf ratio rejection
    let gear_inf = CadJointType::Gear {
        ratio: f64::INFINITY,
        axis_a: [0.0, 0.0, 1.0],
        axis_b: [0.0, 0.0, 1.0],
    };
    assert!(gear_inf.build_generic_joint(None).is_err());

    // 4. Sub-epsilon ratio rejection
    let gear_tiny = CadJointType::Gear {
        ratio: 1e-12,
        axis_a: [0.0, 0.0, 1.0],
        axis_b: [0.0, 0.0, 1.0],
    };
    assert!(gear_tiny.build_generic_joint(None).is_err());

    // 5. Zero vector axis_a rejection
    let gear_zero_axis_a = CadJointType::Gear {
        ratio: 2.0,
        axis_a: [0.0, 0.0, 0.0],
        axis_b: [0.0, 0.0, 1.0],
    };
    let err_axis_a = gear_zero_axis_a.build_generic_joint(None).unwrap_err();
    assert!(
        err_axis_a.to_string().contains("axis_a cannot be a zero vector"),
        "Unexpected error message: {err_axis_a}"
    );

    // 6. Non-finite axis_a rejection
    let gear_nan_axis_a = CadJointType::Gear {
        ratio: 2.0,
        axis_a: [f64::NAN, 0.0, 1.0],
        axis_b: [0.0, 0.0, 1.0],
    };
    assert!(gear_nan_axis_a.build_generic_joint(None).is_err());

    // 7. Zero vector axis_b rejection
    let gear_zero_axis_b = CadJointType::Gear {
        ratio: 2.0,
        axis_a: [0.0, 0.0, 1.0],
        axis_b: [0.0, 0.0, 0.0],
    };
    let err_axis_b = gear_zero_axis_b.build_generic_joint(None).unwrap_err();
    assert!(
        err_axis_b.to_string().contains("axis_b cannot be a zero vector"),
        "Unexpected error message: {err_axis_b}"
    );

    // 8. Non-finite axis_b rejection
    let gear_nan_axis_b = CadJointType::Gear {
        ratio: 2.0,
        axis_a: [0.0, 0.0, 1.0],
        axis_b: [0.0, f64::INFINITY, 0.0],
    };
    assert!(gear_nan_axis_b.build_generic_joint(None).is_err());

    // 9. Valid gear joint passes build_generic_joint
    let gear_valid = CadJointType::Gear {
        ratio: -2.5,
        axis_a: [0.0, 0.0, 1.0],
        axis_b: [0.0, 0.0, 1.0],
    };
    assert!(gear_valid.build_generic_joint(None).is_ok());

    // 10. Verify world.add_joint respects validation on two rigid bodies
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
    let b1 = create_test_box("b1", "Box 1", 0.1, 0.1, 0.1);
    let b2 = create_test_box("b2", "Box 2", 0.1, 0.1, 0.1);
    world.add_part(&b1, true).unwrap();
    world.add_part(&b2, true).unwrap();

    let res_zero = world.add_joint("g_bad", "b1", "b2", gear_zero, None);
    assert!(res_zero.is_err());

    let res_ok = world.add_joint("g_good", "b1", "b2", gear_valid, None);
    assert!(res_ok.is_ok());
}

#[test]
fn test_p2_t23_reserved_ground_world_and_normalization() {
    let mut world = PhysicsWorld::with_gravity([0.0, -9.81, 0.0]);
    let box_part = create_test_box("arm_1", "Arm 1", 0.1, 0.1, 0.1);
    world.add_part(&box_part, true).unwrap();

    // 1. Cannot remove reserved static anchors
    for name in &["ground", "world", "GROUND", "World"] {
        let err = world.remove_part(name).unwrap_err();
        assert!(
            err.to_string().contains("Cannot remove reserved static anchor"),
            "Expected anchor protection error, got: {err}"
        );
    }

    // 2. Cannot add parts with reserved names
    for name in &["ground", "world", "GROUND", "World"] {
        let reserved_part = create_test_box(name, "Reserved Part", 0.1, 0.1, 0.1);
        let err = world.add_part(&reserved_part, true).unwrap_err();
        assert_eq!(err, mcp_cad_physics::PhysicsError::PartAlreadyExists(name.to_string()));
    }

    // 3. Normalization of ground/world in add_joint
    let rev_joint = CadJointType::Revolute {
        axis: [0.0, 0.0, 1.0],
        anchor_a: [0.0, 0.0, 0.0],
        anchor_b: [0.0, 0.0, 0.0],
        limits: None,
    };
    let j1 = world.add_joint("j_ground_uppercase", "GROUND", "arm_1", rev_joint.clone(), None);
    assert!(j1.is_ok(), "Expected add_joint with 'GROUND' to succeed: {:?}", j1.err());

    let j_rec = &world.joints()["j_ground_uppercase"];
    assert_eq!(j_rec.part_a, "ground");

    let j2 = world.add_joint("j_world_mixedcase", "World", "arm_1", rev_joint, None);
    assert!(j2.is_ok(), "Expected add_joint with 'World' to succeed: {:?}", j2.err());
    let j2_rec = &world.joints()["j_world_mixedcase"];
    assert_eq!(j2_rec.part_a, "world");
}

#[test]
fn test_p2_t36_gravity_f64_exact_precision_and_clone() {
    let mut world = PhysicsWorld::new();
    assert_eq!(world.gravity(), [0.0, -9.81, 0.0]);
    assert_eq!(world.gravity()[1] * 1000.0, -9810.0);

    let cloned = world.clone();
    assert_eq!(cloned.gravity(), [0.0, -9.81, 0.0]);

    world.set_gravity([0.0, 0.0, -9.80665]);
    assert_eq!(world.gravity(), [0.0, 0.0, -9.80665]);
}

#[test]
fn test_p3_t6_static_cantilever_beam_load_case_and_newton_third_law() {
    use mcp_cad_physics::LoadCaseSelector;

    let g = 9.81_f64;
    let mut world = PhysicsWorld::with_gravity([0.0, -g, 0.0]);

    // Fixed support block (`is_fixed = true`, added with `is_dynamic = false`)
    let mut wall = create_test_box("wall_support", "Wall Support", 0.05, 0.10, 0.10);
    wall.is_fixed = true;
    world.add_part(&wall, false).unwrap();

    // Horizontal cantilever beam of length L = 0.20 m along +X, cross-section 0.02 m x 0.02 m.
    // `create_box(L, W, H)` is centered at [0,0,0]; translate its mesh by [+L/2, 0, 0] so the
    // root face is at x = 0 and the center of mass is at [L/2, 0, 0] = [0.10, 0, 0] m.
    let length_m = 0.20_f64;
    let width_m = 0.02_f64;
    let height_m = 0.02_f64;
    let mut beam = create_test_box("cantilever_beam", "Cantilever Beam", length_m, width_m, height_m);
    for v in &mut beam.mesh.vertices {
        v.position[0] += length_m * 0.5;
    }
    world.add_part(&beam, true).unwrap();

    let mass_kg = world.part_mass("cantilever_beam").unwrap();
    let expected_force_y = mass_kg * g;
    let expected_torque_z = mass_kg * g * (length_m * 0.5);

    // Connect fixed wall (`part_a`) to cantilever beam (`part_b`) at root anchor [0, 0, 0]
    world
        .add_joint(
            "root_weld",
            "wall_support",
            "cantilever_beam",
            CadJointType::Fixed {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
            },
            None,
        )
        .unwrap();

    let dt = 1.0 / 240.0;
    let summary = world.simulate(0.5, dt);

    // Extract load case on the cantilever beam (`PeakForce`, `PeakTorque`, and `AtTime(0.4)`)
    for selector in [
        LoadCaseSelector::PeakForce,
        LoadCaseSelector::PeakTorque,
        LoadCaseSelector::AtTime(0.4),
    ] {
        let beam_lc = world
            .extract_part_load_case(&summary, "cantilever_beam", selector)
            .expect("extract_part_load_case for cantilever_beam");

        assert_eq!(beam_lc.joints.len(), 1);
        let j_beam = &beam_lc.joints[0];
        assert_eq!(j_beam.joint_id, "root_weld");
        assert_eq!(j_beam.role, "part_b");
        assert_eq!(j_beam.connected_part_id, "wall_support");
        assert_eq!(j_beam.anchor_local, [0.0, 0.0, 0.0]);

        // 1. Reaction force F_y ≈ +m*g supporting the beam against -Y gravity (< 0.01% error in f64)
        let force_rel_err = (j_beam.force[1] - expected_force_y).abs() / expected_force_y;
        assert!(
            force_rel_err < 1e-4,
            "Selector {:?}: expected F_y ≈ {:.6} N (m*g) within 0.01%, got F = {:?} (rel_err = {:.6e})",
            selector,
            expected_force_y,
            j_beam.force,
            force_rel_err
        );
        assert!(j_beam.force[0].abs() < 1e-9 * expected_force_y);
        assert!(j_beam.force[2].abs() < 1e-9 * expected_force_y);

        // 2. Reaction torque at root anchor [0,0,0]: T_z ≈ +m*g*L/2 counteracting gravity moment (< 0.01% error in f64)
        let torque_rel_err = (j_beam.torque[2] - expected_torque_z).abs() / expected_torque_z;
        assert!(
            torque_rel_err < 1e-4,
            "Selector {:?}: expected T_z ≈ {:.6} N*m (m*g*L/2) within 0.01%, got T = {:?} (rel_err = {:.6e})",
            selector,
            expected_torque_z,
            j_beam.torque,
            torque_rel_err
        );

        // 3. Dynamic equilibrium residual must be near machine zero (< 1e-6 of m*g)
        assert!(
            beam_lc.equilibrium_residual_force_norm < 1e-6 * expected_force_y,
            "Selector {:?}: expected near-zero force equilibrium residual, got {:.6e} N",
            selector,
            beam_lc.equilibrium_residual_force_norm
        );
        assert!(
            beam_lc.equilibrium_residual_torque_norm < 1e-6 * expected_torque_z,
            "Selector {:?}: expected near-zero torque equilibrium residual, got {:.6e} N*m",
            selector,
            beam_lc.equilibrium_residual_torque_norm
        );
    }

    // 4. Verify Newton's 3rd Law between `wall_support` (`part_a`) and `cantilever_beam` (`part_b`) to < 1e-12
    let beam_at_t = world
        .extract_part_load_case(&summary, "cantilever_beam", LoadCaseSelector::AtTime(0.4))
        .unwrap();
    let wall_at_t = world
        .extract_part_load_case(&summary, "wall_support", LoadCaseSelector::AtTime(0.4))
        .unwrap();
    let jb = &beam_at_t.joints[0];
    let jw = &wall_at_t.joints[0];
    for axis in 0..3 {
        assert!(
            (jb.force_world[axis] + jw.force_world[axis]).abs() < 1e-12,
            "Newton 3rd law violated for force axis {axis}: beam={:?}, wall={:?}",
            jb.force_world,
            jw.force_world
        );
        assert!(
            (jb.torque_world[axis] + jw.torque_world[axis]).abs() < 1e-12,
            "Newton 3rd law violated for torque axis {axis}: beam={:?}, wall={:?}",
            jb.torque_world,
            jw.torque_world
        );
    }
}

#[test]
fn test_p3_t6_rotating_rod_centripetal_reaction_force_in_local_frame() {
    use mcp_cad_physics::LoadCaseSelector;

    // Zero gravity so the only force is the centripetal reaction force at the revolute pivot
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    // Rod of length L = 0.20 m along +X with root at x = 0 and COM at r_c = L/2 = 0.10 m
    let length_m = 0.20_f64;
    let r_c = length_m * 0.5;
    let mut rod = create_test_box("rotating_rod", "Rotating Rod", length_m, 0.02, 0.02);
    for v in &mut rod.mesh.vertices {
        v.position[0] += r_c;
    }
    world.add_part(&rod, true).unwrap();

    let mass_kg = world.part_mass("rotating_rod").unwrap();

    // Revolute joint about +Z at anchor [0, 0, 0]
    world
        .add_joint(
            "pivot_z",
            "ground",
            "rotating_rod",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .unwrap();

    // Initial angular velocity omega = 10 rad/s about +Z and consistent COM linear velocity
    // v_com = omega x r_com = [0, omega * r_c, 0]
    let omega = 10.0_f64;
    world
        .set_part_angular_velocity("rotating_rod", [0.0, 0.0, omega])
        .unwrap();
    world
        .set_part_velocity("rotating_rod", [0.0, omega * r_c, 0.0])
        .unwrap();

    let expected_centripetal_force = mass_kg * omega * omega * r_c;

    let dt = 1.0 / 240.0;
    let summary = world.simulate(0.25, dt);

    // At t = 0.20 s, the rod has rotated by ~2.0 radians (~114.6 degrees) in world space,
    // yet in the rod's LOCAL coordinate frame the centripetal force must still point along -X!
    let rod_lc = world
        .extract_part_load_case(&summary, "rotating_rod", LoadCaseSelector::AtTime(0.20))
        .expect("extract_part_load_case for rotating_rod");

    assert_eq!(rod_lc.joints.len(), 1);
    let j_rod = &rod_lc.joints[0];
    assert_eq!(j_rod.joint_id, "pivot_z");
    assert_eq!(j_rod.anchor_local, [0.0, 0.0, 0.0]);

    // 1. Magnitude of reaction force F ≈ m * omega^2 * r_c (< 0.5% error per P4-T5 criteria)
    let f_mag_rel_err =
        (j_rod.force_magnitude - expected_centripetal_force).abs() / expected_centripetal_force;
    assert!(
        f_mag_rel_err < 5e-3,
        "Expected centripetal force magnitude ≈ {:.6} N (m*omega^2*r_c) within 0.5%, got {:.6} N (rel_err = {:.6e})",
        expected_centripetal_force,
        j_rod.force_magnitude,
        f_mag_rel_err
    );

    // 2. In the rod's local frame, the pivot pulls the rod inward along -X (F_x ≈ -m * omega^2 * r_c)
    let fx_rel_err =
        (j_rod.force[0] - (-expected_centripetal_force)).abs() / expected_centripetal_force;
    assert!(
        fx_rel_err < 5e-3,
        "Expected local F_x ≈ {:.6} N within 0.5%, got local force = {:?} (world force = {:?})",
        -expected_centripetal_force,
        j_rod.force,
        j_rod.force_world
    );
    assert!(
        j_rod.force[1].abs() < 1e-6 * expected_centripetal_force,
        "Expected negligible local tangential force F_y, got {:?}",
        j_rod.force
    );

    // 3. Free revolute axis Z transmits zero reaction torque
    assert!(
        j_rod.torque_magnitude < 1e-10,
        "Expected zero reaction torque on frictionless revolute joint in planar motion, got {:?}",
        j_rod.torque
    );

    // 4. Local COM centripetal acceleration a_x ≈ -omega^2 * r_c and small equilibrium residual
    let expected_ax = -omega * omega * r_c;
    let ax_rel_err = (rod_lc.inertia.linear_acceleration[0] - expected_ax).abs() / expected_ax.abs();
    assert!(
        ax_rel_err < 5e-3,
        "Expected local centripetal acceleration a_x ≈ {:.6} m/s^2 within 0.5%, got {:?}",
        expected_ax,
        rod_lc.inertia.linear_acceleration
    );
    assert!(
        rod_lc.equilibrium_residual_force_norm < 1e-6 * expected_centripetal_force,
        "Expected near-zero dynamic force equilibrium residual, got {:.6e} N (vec = {:?})",
        rod_lc.equilibrium_residual_force_norm,
        rod_lc.equilibrium_residual_force
    );
}

#[test]
fn test_p3_t9_analyze_mechanism_benchmarks() {
    // 1. One body on Revolute to ground -> 1 DOF
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let rod = create_test_cylinder("pendulum", "Pendulum", 0.01, 0.2);
        world.add_part(&rod, true).expect("rod added");
        world
            .add_joint(
                "j_rev",
                "ground",
                "pendulum",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, -0.1],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("joint added");

        let rep = world.analyze_mechanism(None, None).expect("analysis");
        assert_eq!(rep.dof_numeric, 1, "Revolute to ground must have DOF=1");
        assert_eq!(rep.dof_grubler, 1);
        assert_eq!(rep.redundant_constraints, 0);
        assert!(rep.redundant_joint_ids.is_empty());
        assert!(rep.joints_at_limits.is_empty());
        assert_eq!(rep.moving_parts, vec!["pendulum"]);
        assert!(rep.floating_parts.is_empty());
        assert!(rep.unconnected_parts.is_empty());
    }

    // 2. Fixed joint to ground -> 0 DOF (locks all 6 degrees of freedom)
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let bracket = create_test_box("bracket", "Bracket", 0.1, 0.1, 0.1);
        world.add_part(&bracket, true).expect("bracket added");
        world
            .add_joint(
                "j_fix",
                "ground",
                "bracket",
                CadJointType::Fixed {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                },
                None,
            )
            .expect("joint added");

        let rep = world.analyze_mechanism(None, None).expect("analysis");
        assert_eq!(rep.dof_numeric, 0, "Fixed to ground must have DOF=0");
        assert_eq!(rep.dof_grubler, 0);
        assert_eq!(rep.redundant_constraints, 0);
        assert!(rep.warnings.iter().any(|w| w.contains("DOF=0")));
    }

    // 3. Spherical joint to ground -> 3 DOFs (ball-and-socket rotation)
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let ball = create_test_sphere("ball", "Ball", 0.05);
        world.add_part(&ball, true).expect("ball added");
        world
            .add_joint(
                "j_sph",
                "ground",
                "ball",
                CadJointType::Spherical {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, -0.05],
                },
                None,
            )
            .expect("joint added");

        let rep = world.analyze_mechanism(None, None).expect("analysis");
        assert_eq!(rep.dof_numeric, 3, "Spherical to ground must have DOF=3");
        assert_eq!(rep.dof_grubler, 3);
        assert_eq!(rep.redundant_constraints, 0);
    }

    // 4. Free body (no joints) -> 6 DOFs
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let floater = create_test_box("floater", "Floater", 0.2, 0.2, 0.2);
        world.add_part(&floater, true).expect("floater added");

        let rep = world.analyze_mechanism(None, None).expect("analysis");
        assert_eq!(rep.dof_numeric, 6, "Free body must have DOF=6");
        assert_eq!(rep.dof_grubler, 6);
        assert_eq!(rep.redundant_constraints, 0);
        assert_eq!(rep.unconnected_parts, vec!["floater"]);
        assert_eq!(rep.floating_parts, vec!["floater"]);
        assert!(rep.warnings.iter().any(|w| w.contains("свободные детали")));
    }

    // 5. Planar four-bar linkage (ground + 3 moving bodies, 4 revolute joints)
    // Spatial Grübler: 6*(4-1) - 4*5 = 18 - 20 = -2
    // Numerical rank: 17, DOF = 18 - 17 = 1, redundant_constraints = 3
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let crank = create_test_cylinder("crank", "Crank", 0.01, 0.05);
        let coupler = create_test_cylinder("coupler", "Coupler", 0.01, 0.20);
        let rocker = create_test_cylinder("rocker", "Rocker", 0.01, 0.05);

        world
            .add_part_with_options(&crank, true, ColliderType::ConvexHull, Some([0.0, 0.025, 0.0]), None)
            .expect("crank added");
        world
            .add_part_with_options(&coupler, true, ColliderType::ConvexHull, Some([0.10, 0.05, 0.0]), None)
            .expect("coupler added");
        world
            .add_part_with_options(&rocker, true, ColliderType::ConvexHull, Some([0.20, 0.025, 0.0]), None)
            .expect("rocker added");

        // j1: ground -> crank at A (0,0,0)
        world
            .add_joint(
                "j1_ground_crank",
                "ground",
                "crank",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, -0.025, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j1");

        // j2: crank -> coupler at B (0, 0.05, 0)
        world
            .add_joint(
                "j2_crank_coupler",
                "crank",
                "coupler",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.025, 0.0],
                    anchor_b: [-0.10, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j2");

        // j3: coupler -> rocker at C (0.20, 0.05, 0)
        world
            .add_joint(
                "j3_coupler_rocker",
                "coupler",
                "rocker",
                CadJointType::Revolute {
                    anchor_a: [0.10, 0.0, 0.0],
                    anchor_b: [0.0, 0.025, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j3");

        // j4: rocker -> ground at D (0.20, 0, 0)
        world
            .add_joint(
                "j4_rocker_ground",
                "rocker",
                "ground",
                CadJointType::Revolute {
                    anchor_a: [0.0, -0.025, 0.0],
                    anchor_b: [0.20, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j4");

        let rep = world.analyze_mechanism(None, None).expect("four-bar analysis");
        assert_eq!(rep.moving_parts.len(), 3);
        assert_eq!(rep.jacobian_rows, 20);
        assert_eq!(rep.jacobian_cols, 18);
        assert_eq!(rep.jacobian_rank, 17, "Planar 4-bar Jacobian rank must be 17");
        assert_eq!(rep.dof_numeric, 1, "Planar 4-bar numeric DOF must be 1");
        assert_eq!(rep.dof_grubler, -2, "Planar 4-bar spatial Grübler DOF must be -2");
        assert_eq!(rep.redundant_constraints, 3, "Planar 4-bar redundant constraints must be 3");
        assert_eq!(
            rep.redundant_joint_ids,
            vec![
                "j1_ground_crank",
                "j2_crank_coupler",
                "j3_coupler_rocker",
                "j4_rocker_ground"
            ]
        );
        assert!(rep.floating_parts.is_empty());
        assert!(rep.unconnected_parts.is_empty());
    }

    // 6. Slider-crank mechanism (4 bodies: ground, crank, rod, slider; 3 revolute + 1 prismatic)
    // Spatial Grübler: 6*(4-1) - (3*5 + 1*5) = 18 - 20 = -2
    // Numerical rank: 17, DOF = 18 - 17 = 1, redundant_constraints = 3
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let crank = create_test_cylinder("sc_crank", "Crank", 0.01, 0.04);
        let conrod = create_test_cylinder("sc_rod", "Connecting Rod", 0.01, 0.12);
        let slider = create_test_box("sc_slider", "Slider", 0.04, 0.04, 0.04);

        // Crank at (0, 0.02, 0)
        world
            .add_part_with_options(&crank, true, ColliderType::ConvexHull, Some([0.0, 0.02, 0.0]), None)
            .expect("crank");
        // Slider at (0.10, 0.0, 0)
        world
            .add_part_with_options(&slider, true, ColliderType::ConvexHull, Some([0.10, 0.0, 0.0]), None)
            .expect("slider");
        // ConRod center at (0.05, 0.02, 0)
        world
            .add_part_with_options(&conrod, true, ColliderType::ConvexHull, Some([0.05, 0.02, 0.0]), None)
            .expect("conrod");

        // j1: ground -> crank (revolute at (0,0,0) along Z)
        world
            .add_joint(
                "j1_sc_rev",
                "ground",
                "sc_crank",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, -0.02, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j1");

        // j2: crank -> rod (revolute at (0, 0.04, 0) along Z)
        world
            .add_joint(
                "j2_sc_rev",
                "sc_crank",
                "sc_rod",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.02, 0.0],
                    anchor_b: [-0.05, 0.02, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j2");

        // j3: rod -> slider (revolute at (0.10, 0, 0) along Z)
        world
            .add_joint(
                "j3_sc_rev",
                "sc_rod",
                "sc_slider",
                CadJointType::Revolute {
                    anchor_a: [0.05, -0.02, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j3");

        // j4: slider -> ground (prismatic along X axis [1, 0, 0])
        world
            .add_joint(
                "j4_sc_prism",
                "sc_slider",
                "ground",
                CadJointType::Prismatic {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [1.0, 0.0, 0.0],
                    limits: None,
                },
                None,
            )
            .expect("j4");

        let rep = world.analyze_mechanism(None, None).expect("slider-crank analysis");
        assert_eq!(rep.moving_parts.len(), 3);
        assert_eq!(rep.jacobian_rows, 20);
        assert_eq!(rep.jacobian_cols, 18);
        assert_eq!(rep.jacobian_rank, 17, "Slider-crank Jacobian rank must be 17");
        assert_eq!(rep.dof_numeric, 1, "Slider-crank numeric DOF must be 1");
        assert_eq!(rep.dof_grubler, -2, "Slider-crank spatial Grübler DOF must be -2");
        assert_eq!(rep.redundant_constraints, 3, "Slider-crank redundant constraints must be 3");
        assert_eq!(
            rep.redundant_joint_ids,
            vec!["j1_sc_rev", "j2_sc_rev", "j3_sc_rev", "j4_sc_prism"]
        );
    }

    // 7. Two wheels on Revolute + Gear constraint -> 1 DOF
    // Spatial Grübler: 6*(3-1) - (5 + 5 + 1) = 12 - 11 = 1
    // Numerical rank: 11, DOF = 12 - 11 = 1, redundant_constraints = 0
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let wheel1 = create_test_cylinder("w1", "Wheel 1", 0.03, 0.02);
        let wheel2 = create_test_cylinder("w2", "Wheel 2", 0.06, 0.02);

        world
            .add_part_with_options(&wheel1, true, ColliderType::ConvexHull, Some([0.0, 0.0, 0.0]), None)
            .expect("w1");
        world
            .add_part_with_options(&wheel2, true, ColliderType::ConvexHull, Some([0.09, 0.0, 0.0]), None)
            .expect("w2");

        world
            .add_joint(
                "j_w1_rev",
                "ground",
                "w1",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("w1 rev");

        world
            .add_joint(
                "j_w2_rev",
                "ground",
                "w2",
                CadJointType::Revolute {
                    anchor_a: [0.09, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("w2 rev");

        world
            .add_joint(
                "j_gear_w1_w2",
                "w1",
                "w2",
                CadJointType::Gear {
                    ratio: 2.0,
                    axis_a: [0.0, 0.0, 1.0],
                    axis_b: [0.0, 0.0, 1.0],
                },
                None,
            )
            .expect("gear joint");

        let rep = world.analyze_mechanism(None, None).expect("gear analysis");
        assert_eq!(rep.moving_parts.len(), 2);
        assert_eq!(rep.jacobian_rows, 11);
        assert_eq!(rep.jacobian_cols, 12);
        assert_eq!(rep.jacobian_rank, 11);
        assert_eq!(rep.dof_numeric, 1, "Two geared wheels must have DOF=1");
        assert_eq!(rep.dof_grubler, 1, "Two geared wheels Grübler DOF must be 1");
        assert_eq!(rep.redundant_constraints, 0);
    }
}

#[test]
fn test_p3_t10_check_interference_overlapping_and_disjoint_boxes_and_ignore_adjacent() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    // Box A: 0.10 x 0.10 x 0.10 m centered at [0.0, 0.0, 0.0] -> x in [-0.05, +0.05]
    let box_a = create_test_box("box_a", "Box A", 0.10, 0.10, 0.10);
    // Box B: 0.10 x 0.10 x 0.10 m centered at [0.06, 0.0, 0.0] -> x in [+0.01, +0.11]
    // Analytical overlap with Box A: x in [0.01, 0.05] (0.04 m), y in [-0.05, 0.05] (0.10 m), z in [-0.05, 0.05] (0.10 m)
    // Analytical intersection_volume = 0.04 * 0.10 * 0.10 = 0.0004 m^3, penetration_depth = 0.04 m.
    let box_b = create_test_box("box_b", "Box B", 0.10, 0.10, 0.10);
    // Box C: 0.10 x 0.10 x 0.10 m centered at [0.25, 0.0, 0.0] -> x in [0.20, 0.30]
    // Analytical clearance from Box A: 0.20 - 0.05 = 0.15 m; from Box B: 0.20 - 0.11 = 0.09 m.
    let box_c = create_test_box("box_c", "Box C", 0.10, 0.10, 0.10);

    world
        .add_part_with_options(&box_a, true, ColliderType::ConvexHull, Some([0.0, 0.0, 0.0]), None)
        .expect("add box_a");
    world
        .add_part_with_options(&box_b, true, ColliderType::ConvexHull, Some([0.06, 0.0, 0.0]), None)
        .expect("add box_b");
    world
        .add_part_with_options(&box_c, true, ColliderType::ConvexHull, Some([0.25, 0.0, 0.0]), None)
        .expect("add box_c");

    let report = world
        .check_interference(None, false, None)
        .expect("check_interference");

    assert_eq!(report.status, "FAIL");
    assert_eq!(report.candidate_pairs_count, 3);
    assert_eq!(report.evaluated_pairs_count, 3);
    assert_eq!(report.interfering_pairs_count, 1);

    let pair_ab = report
        .pairs
        .iter()
        .find(|p| p.a == "box_a" && p.b == "box_b")
        .expect("pair box_a <-> box_b");
    assert!(pair_ab.intersects);
    let vol_ab = pair_ab.intersection_volume.expect("intersection_volume");
    let expected_vol = 0.0004_f64;
    let vol_rel_err = (vol_ab - expected_vol).abs() / expected_vol;
    assert!(
        vol_rel_err <= 0.01,
        "Expected intersection_volume = {expected_vol} m^3 within 1%, got {vol_ab} (rel_err = {vol_rel_err})"
    );
    let depth_ab = pair_ab.penetration_depth.expect("penetration_depth");
    assert!(
        (depth_ab - 0.04).abs() <= 1e-4,
        "Expected penetration_depth = 0.04 m, got {depth_ab}"
    );
    assert!(pair_ab.clearance.is_none());

    let pair_ac = report
        .pairs
        .iter()
        .find(|p| p.a == "box_a" && p.b == "box_c")
        .expect("pair box_a <-> box_c");
    assert!(!pair_ac.intersects);
    assert!(pair_ac.intersection_volume.is_none());
    let clr_ac = pair_ac.clearance.expect("clearance box_a <-> box_c");
    assert!(
        (clr_ac - 0.15).abs() <= 1e-6,
        "Expected clearance 0.15 m between box_a and box_c, got {clr_ac}"
    );

    let pair_bc = report
        .pairs
        .iter()
        .find(|p| p.a == "box_b" && p.b == "box_c")
        .expect("pair box_b <-> box_c");
    assert!(!pair_bc.intersects);
    let clr_bc = pair_bc.clearance.expect("clearance box_b <-> box_c");
    assert!(
        (clr_bc - 0.09).abs() <= 1e-6,
        "Expected clearance 0.09 m between box_b and box_c, got {clr_bc}"
    );
    assert!(
        (report.min_clearance.expect("min_clearance") - 0.09).abs() <= 1e-6,
        "Expected global min_clearance = 0.09 m"
    );

    // Connect box_a and box_b by a revolute joint and verify ignore_adjacent = true skips (box_a, box_b)
    world
        .add_joint(
            "j_ab",
            "box_a",
            "box_b",
            CadJointType::Revolute {
                anchor_a: [0.03, 0.0, 0.0],
                anchor_b: [-0.03, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("add joint j_ab");

    let report_ignore = world
        .check_interference(None, true, None)
        .expect("check_interference with ignore_adjacent = true");
    assert_eq!(report_ignore.status, "PASS");
    assert_eq!(report_ignore.candidate_pairs_count, 3);
    assert_eq!(report_ignore.skipped_adjacent_pairs_count, 1);
    assert_eq!(report_ignore.evaluated_pairs_count, 2);
    assert_eq!(report_ignore.interfering_pairs_count, 0);
}

#[test]
fn test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    // Hinged arm: length L = 0.10 m along +X (x in [0.0, 0.10]), width w = 0.01 m (y in [-0.005, 0.005]), depth 0.01 m.
    let mut arm = create_test_box("arm", "Hinged Arm", 0.10, 0.01, 0.01);
    for v in &mut arm.mesh.vertices {
        v.position[0] += 0.05;
    }
    world.add_part(&arm, true).expect("add arm");

    // Fixed wall at y in [0.05, 0.07], x in [0.02, 0.12], z in [-0.02, 0.02]
    let mut wall = create_test_box("wall", "Fixed Wall", 0.10, 0.02, 0.04);
    wall.is_fixed = true;
    for v in &mut wall.mesh.vertices {
        v.position[0] += 0.07;
        v.position[1] += 0.06;
    }
    world.add_part(&wall, false).expect("add wall");

    world
        .add_joint(
            "j_hinge",
            "ground",
            "arm",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("add j_hinge");

    // Analytical first-contact angle theta*:
    // Upper tip of arm at (x=0.10, y=0.005) has y(theta) = 0.10*sin(theta) + 0.005*cos(theta) = 0.05 m
    // => theta* ≈ 27.49 deg.
    // With steps = 90 over [0, 90 deg] (step_size = 1.0 deg), step 27 (27 deg) is clear and step 28 (28 deg) collides.
    let from_rad = 0.0_f64;
    let to_rad = std::f64::consts::FRAC_PI_2;
    let steps = 90usize;
    let step_size_rad = (to_rad - from_rad) / (steps as f64);

    let sweep = world
        .sweep_joint("j_hinge", from_rad, to_rad, steps, false)
        .expect("sweep_joint");

    assert_eq!(sweep.status, "FAIL");
    assert!(sweep.collision_detected);
    assert_eq!(sweep.samples.len(), 91);

    let analytical_theta_rad = 27.492_f64.to_radians();
    let detected_rad = sweep.first_collision_value.expect("first_collision_value");
    assert!(
        (detected_rad - analytical_theta_rad).abs() <= step_size_rad,
        "Detected collision angle {:.2} deg must be within step size (1.0 deg) of analytical {:.2} deg",
        detected_rad.to_degrees(),
        analytical_theta_rad.to_degrees()
    );
    assert_eq!(sweep.first_collision_step, Some(28));
    assert!(!sweep.first_collision_pairs.is_empty());
    // `min_clearance` / `min_clearance_step` track the global minimum across all non-colliding steps:
    // - At step 27 (27 deg, pre-collision approach), upper tip (0.10, 0.005) has y = 0.049854 m -> clearance ~0.146 mm to y=0.05.
    // - Steps 28..=77 intersect the finite wall (x in [0.02, 0.12], y in [0.05, 0.07]).
    // - At step 78 (78 deg, exiting past the wall corner (0.02, 0.07)), trailing edge y_loc = -0.005 m
    //   passes at y_loc(0.02, 0.07) = -0.02*sin(78°) + 0.07*cos(78°) ≈ -0.005009 m -> clearance ~0.009 mm < 0.146 mm.
    let pre_collision_clr = sweep.samples[27]
        .min_clearance
        .expect("step 27 must be clear");
    assert!(pre_collision_clr > 1e-4 && pre_collision_clr < 2e-4);
    let min_clr = sweep.min_clearance.expect("min_clearance");
    assert!(min_clr > 0.0 && min_clr < pre_collision_clr);
    assert_eq!(sweep.min_clearance_step, Some(78));
    assert!((sweep.safe_travel_range - 28.0_f64.to_radians()).abs() < 1e-9);

    // When the sweep stops while still inside the obstacle (e.g. [0, 60 deg], 1 deg/step),
    // the only clear steps are 0..=27 and `min_clearance_step` is the step right before collision (27).
    let sweep_before_exit = world
        .sweep_joint("j_hinge", 0.0, 60.0_f64.to_radians(), 60, false)
        .expect("sweep_joint [0, 60 deg]");
    assert_eq!(sweep_before_exit.first_collision_step, Some(28));
    assert_eq!(sweep_before_exit.min_clearance_step, Some(27));
    assert!(
        (sweep_before_exit.min_clearance.expect("min_clearance [0,60]") - pre_collision_clr).abs()
            < 1e-9
    );
}

#[test]
fn test_p3_t10_benchmark_check_interference_10_parts_45_pairs() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);

    // 10 parts arranged in 3D space (some overlapping, most separated) -> 10 * 9 / 2 = 45 pairs
    for i in 0..10 {
        let part_id = format!("part_{i:02}");
        let b = create_test_box(&part_id, &part_id, 0.05, 0.05, 0.05);
        // Parts 0 and 1 overlap slightly (dx = 0.03 < 0.05); parts 2..9 are spaced by 0.10 m
        let x = if i == 1 { 0.03 } else { (i as f64) * 0.10 };
        let y = ((i % 3) as f64) * 0.02;
        world
            .add_part_with_options(&b, true, ColliderType::ConvexHull, Some([x, y, 0.0]), None)
            .expect("add bench part");
    }

    let report = std::hint::black_box(
        world
            .check_interference(None, false, None)
            .expect("benchmark 10 parts"),
    );
    assert_eq!(report.status, "FAIL");
    assert_eq!(report.candidate_pairs_count, 45);
    assert_eq!(report.skipped_adjacent_pairs_count, 0);
    // Broad-phase AABB filtering must cull all 44 disjoint pairs and send only (part_00, part_01) to CSG narrow-phase
    assert_eq!(report.broadphase_overlap_pairs_count, 1);
    assert_eq!(report.evaluated_pairs_count, 45);
    assert_eq!(report.interfering_pairs_count, 1);
    assert_eq!(report.pairs.len(), 45);
    assert!(
        (report.max_intersection_volume - 3.0e-5).abs() < 1e-9,
        "expected analytical intersection volume 3.0e-5 m^3, got {:.6e}",
        report.max_intersection_volume
    );
    let min_clr = report.min_clearance.expect("disjoint pairs must yield min_clearance");
    assert!(
        (min_clr - 0.05).abs() < 1e-9,
        "expected analytical min_clearance 0.05 m across disjoint pairs, got {:.6e}",
        min_clr
    );
    assert!(
        report.elapsed_ms.is_finite() && report.elapsed_ms >= 0.0,
        "elapsed_ms must be finite and non-negative, got {}",
        report.elapsed_ms
    );
    // Generous catastrophic-hang guard immune to parallel CI/CPU contention (debug threshold was 250 ms)
    assert!(
        report.elapsed_ms < 5_000.0,
        "10-part (45-pair) interference check hung for {:.3} ms (expected < 5000 ms)",
        report.elapsed_ms
    );
}

#[test]
fn test_p3_t15_redundant_joint_attribution_and_active_limits() {
    // 1. Selective attribution of redundant_joint_ids: 2-link open chain + duplicate joint
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let link1 = create_test_box("link1", "Link 1", 0.1, 0.02, 0.02);
        let link2 = create_test_box("link2", "Link 2", 0.1, 0.02, 0.02);
        world
            .add_part_with_options(&link1, true, ColliderType::ConvexHull, Some([0.05, 0.0, 0.0]), None)
            .expect("link1");
        world
            .add_part_with_options(&link2, true, ColliderType::ConvexHull, Some([0.15, 0.0, 0.0]), None)
            .expect("link2");

        world
            .add_joint(
                "j_base",
                "ground",
                "link1",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [-0.05, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j_base");
        world
            .add_joint(
                "j_arm",
                "link1",
                "link2",
                CadJointType::Revolute {
                    anchor_a: [0.05, 0.0, 0.0],
                    anchor_b: [-0.05, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j_arm");
        world
            .add_joint(
                "j_arm_dup",
                "link1",
                "link2",
                CadJointType::Revolute {
                    anchor_a: [0.05, 0.0, 0.0],
                    anchor_b: [-0.05, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j_arm_dup");

        let rep = world.analyze_mechanism(None, None).expect("redundant chain");
        assert_eq!(rep.dof_numeric, 2);
        assert_eq!(rep.dof_grubler, -3);
        assert_eq!(rep.redundant_constraints, 5);
        assert_eq!(rep.redundant_joint_ids, vec!["j_arm", "j_arm_dup"]);
    }

    // 2. Revolute joint strictly inside limits -> no limit warning
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let arm = create_test_box("arm", "Arm", 0.1, 0.02, 0.02);
        world.add_part(&arm, true).expect("arm");
        world
            .add_joint(
                "j_hinge",
                "ground",
                "arm",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: Some([-std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_4]),
                },
                None,
            )
            .expect("j_hinge");

        let rep = world.analyze_mechanism(None, None).expect("inside limits");
        assert_eq!(rep.dof_numeric, 1);
        assert!(rep.joints_at_limits.is_empty());
        assert!(rep.warnings.is_empty(), "Unexpected warnings: {:?}", rep.warnings);
    }

    // 3. Revolute joint driven to upper limit by simulation -> warning + joints_at_limits + motor_blocked
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let arm = create_test_box("arm_lim", "Arm", 0.2, 0.02, 0.02);
        world.add_part(&arm, true).expect("arm_lim");
        let max_rad = 30.0_f64.to_radians();
        world
            .add_joint(
                "j_rev_lim",
                "ground",
                "arm_lim",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: Some([-max_rad, max_rad]),
                },
                Some(MotorConfig::new(5.0, 200.0)),
            )
            .expect("j_rev_lim");

        let summary = world.simulate(0.5, 0.005);
        assert!(summary.success);

        let rep = world.analyze_mechanism(None, None).expect("at upper limit");
        assert_eq!(rep.joints_at_limits.len(), 1, "Expected joint at limit: {:?}", rep);
        let lim = &rep.joints_at_limits[0];
        assert_eq!(lim.joint_id, "j_rev_lim");
        assert_eq!(lim.joint_type, "revolute");
        assert_eq!(lim.unit, "rad");
        assert!(
            lim.boundary == "max" || lim.boundary == "above_max",
            "Expected max/above_max, got {} (value={:.4}, limits={:?})",
            lim.boundary,
            lim.current_value,
            lim.limits
        );
        assert!(lim.motor_blocked);
        assert!(rep.warnings.iter().any(|w| w.contains("limits") && w.contains("j_rev_lim")));
    }

    // 4. Prismatic joint at lower limit without motor -> boundary = "min", motor_blocked = false
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let slider = create_test_box("slider_min", "Slider", 0.04, 0.04, 0.04);
        world
            .add_part_with_options(
                &slider,
                true,
                ColliderType::Cube,
                Some([-0.05, 0.0, 0.0]),
                None,
            )
            .expect("slider_min");
        world
            .add_joint(
                "j_prism_min",
                "ground",
                "slider_min",
                CadJointType::Prismatic {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [1.0, 0.0, 0.0],
                    limits: Some([-0.05, 0.10]),
                },
                None,
            )
            .expect("j_prism_min");

        let rep = world.analyze_mechanism(None, None).expect("prismatic at min");
        assert_eq!(rep.dof_numeric, 1);
        assert_eq!(rep.joints_at_limits.len(), 1);
        assert_eq!(rep.joints_at_limits[0].joint_id, "j_prism_min");
        assert_eq!(rep.joints_at_limits[0].boundary, "min");
        assert_eq!(rep.joints_at_limits[0].unit, "m");
        assert!(!rep.joints_at_limits[0].motor_blocked);
        assert!(rep.warnings.iter().any(|w| w.contains("limits") && w.contains("j_prism_min")));
    }

    // 5. Zero-range limits [0.0, 0.0] -> bilaterally locked (DOF = 0, boundary = "locked")
    {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let arm = create_test_box("arm_zero", "Arm Zero", 0.1, 0.02, 0.02);
        world.add_part(&arm, true).expect("arm_zero");
        world
            .add_joint(
                "j_zero_lim",
                "ground",
                "arm_zero",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: Some([0.0, 0.0]),
                },
                None,
            )
            .expect("j_zero_lim");

        let rep = world.analyze_mechanism(None, None).expect("zero-range limits");
        assert_eq!(rep.dof_numeric, 0);
        assert_eq!(rep.dof_grubler, 0);
        assert_eq!(rep.joint_constraints_count, 6);
        assert_eq!(rep.joints_at_limits.len(), 1);
        assert_eq!(rep.joints_at_limits[0].boundary, "locked");
        assert!(rep.joints_at_limits[0].motor_blocked);
        assert!(rep.warnings.iter().any(|w| w.contains("DOF=0")));
        assert!(rep.warnings.iter().any(|w| w.contains("limits") && w.contains("j_zero_lim")));
    }
}

#[test]
fn test_p3_t16_sweep_joint_four_bar_full_rotation_and_obstacle() {
    // Grashof crank-rocker 4-bar:
    // ground = 0.12 m, crank = 0.04 m, coupler = 0.13 m, rocker = 0.09 m
    // s + l = 0.04 + 0.13 = 0.17 < p + q = 0.12 + 0.09 = 0.21 -> continuous 360° crank rotation.
    // Initial geometry (exact 5-12-13 triangle):
    // A=(0, 0, 0), B=(0, 0.04, 0), C=(0.12, 0.09, 0), D=(0.12, 0, 0).
    let build_four_bar = |with_wall: bool| {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let crank = create_test_box("crank", "Crank", 0.008, 0.04, 0.008);
        let coupler = create_test_box("coupler", "Coupler", 0.13, 0.008, 0.008);
        let rocker = create_test_box("rocker", "Rocker", 0.008, 0.09, 0.008);

        world
            .add_part_with_options(&crank, true, ColliderType::ConvexHull, Some([0.0, 0.02, 0.0]), None)
            .expect("crank");
        world
            .add_part_with_options(
                &coupler,
                true,
                ColliderType::ConvexHull,
                Some([0.06, 0.065, 0.02]),
                None,
            )
            .expect("coupler");
        world
            .add_part_with_options(&rocker, true, ColliderType::ConvexHull, Some([0.12, 0.045, 0.0]), None)
            .expect("rocker");

        // j1: ground -> crank at A=(0,0,0)
        world
            .add_joint(
                "j1_crank",
                "ground",
                "crank",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, -0.02, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j1");

        // j2: crank -> coupler at B=(0, 0.04, 0)
        // coupler is offset in Z by +0.02 purely visually; anchor matches B in world space
        world
            .add_joint(
                "j2_coupler_a",
                "crank",
                "coupler",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.02, 0.0],
                    anchor_b: [-0.06, -0.025, -0.02],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j2");

        // j3: coupler -> rocker at C=(0.12, 0.09, 0)
        world
            .add_joint(
                "j3_coupler_b",
                "coupler",
                "rocker",
                CadJointType::Revolute {
                    anchor_a: [0.06, 0.025, -0.02],
                    anchor_b: [0.0, 0.045, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j3");

        // j4: rocker -> ground at D=(0.12, 0, 0)
        world
            .add_joint(
                "j4_rocker",
                "rocker",
                "ground",
                CadJointType::Revolute {
                    anchor_a: [0.0, -0.045, 0.0],
                    anchor_b: [0.12, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j4");

        if with_wall {
            // As crank turns CCW (0 -> +180 deg), rocker swings left (-X) toward the obstacle
            let wall = create_test_box("wall", "Wall", 0.01, 0.06, 0.02);
            world
                .add_part_with_options(
                    &wall,
                    false,
                    ColliderType::ConvexHull,
                    Some([0.09, 0.075, 0.0]),
                    None,
                )
                .expect("wall");
        }
        world
    };

    // 1. Full 360° sweep without obstacles or false lockup from planar redundant constraints
    {
        let world = build_four_bar(false);
        let rep = world
            .sweep_joint(
                "j1_crank",
                0.0,
                std::f64::consts::TAU,
                72,
                true,
            )
            .expect("4-bar full sweep should succeed");

        assert!(rep.closed_loop_detected, "Must detect closed loop");
        assert!(!rep.lockup_detected, "Grashof 4-bar must not false-lockup");
        assert_eq!(rep.lockup_step, None);
        assert_eq!(rep.lockup_value, None);
        assert_eq!(rep.status, "PASS", "Unexpected failure: collision={}, lockup={}", rep.collision_detected, rep.lockup_detected);
        assert!(
            rep.max_closure_residual < 5e-5,
            "Max closure residual too high: {}",
            rep.max_closure_residual
        );
        assert_eq!(rep.samples.len(), 73);
        assert!(rep.samples.iter().all(|s| !s.locked_up && s.closure_residual < 5e-5));
    }

    // 2. Rocker (follower link) hits fixed wall during crank sweep
    {
        let world = build_four_bar(true);
        let rep = world
            .sweep_joint(
                "j1_crank",
                0.0,
                std::f64::consts::PI,
                36,
                true,
            )
            .expect("4-bar obstacle sweep");

        assert!(rep.closed_loop_detected);
        assert!(!rep.lockup_detected);
        assert_eq!(rep.status, "FAIL");
        assert!(rep.collision_detected, "Follower rocker must collide with fixed wall");
        assert!(
            rep.first_collision_pairs.iter().any(|p| {
                (p.a == "rocker" && p.b == "wall")
                    || (p.b == "rocker" && p.a == "wall")
            }),
            "Expected rocker-wall collision, got {:?}",
            rep.first_collision_pairs
        );
    }
}

#[test]
fn test_p3_t16_sweep_joint_slider_crank_and_obstacle() {
    // Pythagorean slider-crank: crank = 0.03 m, conrod = 0.05 m.
    // Initial: A=(0,0,0), B=(0,0.03,0), C=(0.04,0,0). Since 0.03^2 + 0.04^2 = 0.05^2, residual=0.
    let build_sc = |with_wall: bool| {
        let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
        let crank = create_test_box("sc_crank", "Crank", 0.008, 0.03, 0.008);
        let conrod = create_test_box("sc_rod", "Rod", 0.05, 0.008, 0.008);
        let slider = create_test_box("sc_slider", "Slider", 0.02, 0.02, 0.02);

        world
            .add_part_with_options(&crank, true, ColliderType::ConvexHull, Some([0.0, 0.015, 0.0]), None)
            .expect("crank");
        world
            .add_part_with_options(
                &conrod,
                true,
                ColliderType::ConvexHull,
                Some([0.02, 0.015, 0.02]),
                None,
            )
            .expect("conrod");
        world
            .add_part_with_options(&slider, true, ColliderType::ConvexHull, Some([0.04, 0.0, 0.0]), None)
            .expect("slider");

        world
            .add_joint(
                "j_sc_crank",
                "ground",
                "sc_crank",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, -0.015, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j1");
        world
            .add_joint(
                "j_sc_pin_b",
                "sc_crank",
                "sc_rod",
                CadJointType::Revolute {
                    anchor_a: [0.0, 0.015, 0.0],
                    anchor_b: [-0.02, 0.015, -0.02],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j2");
        world
            .add_joint(
                "j_sc_pin_c",
                "sc_rod",
                "sc_slider",
                CadJointType::Revolute {
                    anchor_a: [0.02, -0.015, -0.02],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                },
                None,
            )
            .expect("j3");
        world
            .add_joint(
                "j_sc_rail",
                "sc_slider",
                "ground",
                CadJointType::Prismatic {
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    axis: [1.0, 0.0, 0.0],
                    limits: None,
                },
                None,
            )
            .expect("j4");

        if with_wall {
            let wall = create_test_box("endstop", "Endstop", 0.01, 0.04, 0.04);
            world
                .add_part_with_options(&wall, false, ColliderType::ConvexHull, Some([0.075, 0.0, 0.0]), None)
                .expect("wall");
        }
        world
    };

    // Full 360° slider-crank rotation
    {
        let world = build_sc(false);
        let rep = world
            .sweep_joint(
                "j_sc_crank",
                0.0,
                std::f64::consts::TAU,
                36,
                true,
            )
            .expect("slider-crank full sweep");
        assert!(rep.closed_loop_detected);
        assert!(!rep.lockup_detected);
        assert_eq!(rep.status, "PASS");
        assert!(rep.max_closure_residual < 5e-5);
    }

    // Sweeping crank toward -90 deg moves slider from x=0.04 toward x=0.08, hitting wall at x=0.075
    {
        let world = build_sc(true);
        let rep = world
            .sweep_joint(
                "j_sc_crank",
                0.0,
                -std::f64::consts::FRAC_PI_2,
                18,
                true,
            )
            .expect("slider-crank obstacle sweep");
        assert!(rep.closed_loop_detected);
        assert!(!rep.lockup_detected);
        assert!(rep.collision_detected, "Slider must hit endstop wall");
        assert_eq!(rep.status, "FAIL");
        assert!(
            rep.first_collision_pairs.iter().any(|p| {
                (p.a == "sc_slider" && p.b == "endstop")
                    || (p.b == "sc_slider" && p.a == "endstop")
            }),
            "Expected sc_slider-endstop collision, got {:?}",
            rep.first_collision_pairs
        );
    }
}

#[test]
fn test_p3_t16_sweep_joint_geometric_lockup_detection() {
    // Non-Grashof 4-bar: ground L1=0.20, crank L2=0.10, coupler L3=0.12, rocker L4=0.10.
    // Initial pose along +X: A=(0,0,0), B=(0.10,0,0), C=(0.172, 0.096, 0), D=(0.20,0,0).
    // Exact check: 0.072^2 + 0.096^2 = 0.12^2 and 0.028^2 + 0.096^2 = 0.10^2.
    // Analytical lockup angle: cos(theta_lock) = (0.20^2 + 0.10^2 - 0.22^2) / (2*0.20*0.10) = 0.04
    // => theta_lock = acos(0.04) = 87.707 deg.
    // Sweeping 0 -> 120 deg in 24 steps (5 deg/step) must lock up at step 18 (90 deg).
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
    let crank = create_test_box("lk_crank", "Crank", 0.10, 0.008, 0.008);
    let coupler = create_test_box("lk_coupler", "Coupler", 0.12, 0.008, 0.008);
    let rocker = create_test_box("lk_rocker", "Rocker", 0.10, 0.008, 0.008);

    world
        .add_part_with_options(&crank, true, ColliderType::ConvexHull, Some([0.05, 0.0, 0.0]), None)
        .expect("crank");
    world
        .add_part_with_options(&coupler, true, ColliderType::ConvexHull, Some([0.136, 0.048, 0.02]), None)
        .expect("coupler");
    world
        .add_part_with_options(&rocker, true, ColliderType::ConvexHull, Some([0.186, 0.048, 0.0]), None)
        .expect("rocker");

    world
        .add_joint(
            "j1_lk",
            "ground",
            "lk_crank",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [-0.05, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("j1");
    world
        .add_joint(
            "j2_lk",
            "lk_crank",
            "lk_coupler",
            CadJointType::Revolute {
                anchor_a: [0.05, 0.0, 0.0],
                anchor_b: [-0.036, -0.048, -0.02],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("j2");
    world
        .add_joint(
            "j3_lk",
            "lk_coupler",
            "lk_rocker",
            CadJointType::Revolute {
                anchor_a: [0.036, 0.048, -0.02],
                anchor_b: [-0.014, 0.048, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("j3");
    world
        .add_joint(
            "j4_lk",
            "lk_rocker",
            "ground",
            CadJointType::Revolute {
                anchor_a: [0.014, -0.048, 0.0],
                anchor_b: [0.20, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("j4");

    let rep = world
        .sweep_joint(
            "j1_lk",
            0.0,
            120.0_f64.to_radians(),
            24,
            true,
        )
        .expect("lockup sweep");

    assert!(rep.closed_loop_detected);
    assert!(rep.lockup_detected, "Expected geometric lockup past 87.7 deg");
    assert_eq!(rep.status, "FAIL");
    assert_eq!(rep.lockup_step, Some(18), "Lockup should trigger at 90 deg (step 18)");
    let lock_deg = rep.lockup_value.expect("lockup_value").to_degrees();
    assert!(
        (lock_deg - 90.0).abs() < 0.05,
        "Expected lockup at 90 deg, got {lock_deg}"
    );
    assert!(rep.samples[18].locked_up);
    assert!(
        rep.max_closure_residual > 2e-4,
        "Expected lockup residual > 2e-4, got {}",
        rep.max_closure_residual
    );
}

#[test]
fn test_p3_t16_sweep_joint_open_chain_backward_compat() {
    let mut world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
    let arm1 = create_test_box("oc_arm1", "Arm1", 0.10, 0.02, 0.02);
    let arm2 = create_test_box("oc_arm2", "Arm2", 0.10, 0.02, 0.02);
    world
        .add_part_with_options(&arm1, true, ColliderType::ConvexHull, Some([0.05, 0.0, 0.0]), None)
        .expect("arm1");
    world
        .add_part_with_options(&arm2, true, ColliderType::ConvexHull, Some([0.15, 0.0, 0.0]), None)
        .expect("arm2");
    world
        .add_joint(
            "j1_oc",
            "ground",
            "oc_arm1",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [-0.05, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("j1");
    world
        .add_joint(
            "j2_oc",
            "oc_arm1",
            "oc_arm2",
            CadJointType::Revolute {
                anchor_a: [0.05, 0.0, 0.0],
                anchor_b: [-0.05, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("j2");

    let rep = world
        .sweep_joint("j1_oc", 0.0, std::f64::consts::FRAC_PI_2, 10, true)
        .expect("open chain sweep");
    assert!(!rep.closed_loop_detected);
    assert!(!rep.lockup_detected);
    assert_eq!(rep.lockup_step, None);
    assert_eq!(rep.lockup_value, None);
    assert_eq!(rep.max_closure_residual, 0.0);
    assert_eq!(rep.status, "PASS");
}

#[test]
fn test_p4_t1_dae_f64_candidates_vs_analytical_and_physics_world_baseline() {
    use mcp_cad_physics::dae_prototype::{
        DaeSolverCandidate, run_free_fall_dae, run_gear_pair_10_revs_dae, run_rotating_rod_dae,
        run_slider_crank_dae,
    };
    use mcp_cad_physics::LoadCaseSelector;

    // -------------------------------------------------------------------------
    // Benchmark 1: Free Fall under Gravity (y0 = 10.0 m, g = -9.81 m/s^2, T = 1.0 s, dt = 5 ms)
    // -------------------------------------------------------------------------
    let ff_kkt = run_free_fall_dae(DaeSolverCandidate::DirectKktIndex2Projection);
    let ff_pgs = run_free_fall_dae(DaeSolverCandidate::SemiImplicitEulerPgsBaumgarte);

    let mut world_ff = PhysicsWorld::new();
    let box_part = create_test_box("box_falling", "Falling Weight", 0.2, 0.2, 0.2);
    world_ff
        .add_part_with_options(
            &box_part,
            true,
            ColliderType::ConvexHull,
            Some([0.0, 10.0, 0.0]),
            None,
        )
        .unwrap();
    let t0 = std::time::Instant::now();
    let summary_ff = world_ff.simulate(1.0, 0.005);
    let world_ff_us = t0.elapsed().as_secs_f64() * 1e6;
    let world_y = summary_ff.final_position("box_falling").unwrap()[1];
    let world_vy = summary_ff.final_linear_velocity("box_falling").unwrap()[1];
    let world_ff_pos_err = (world_y - 5.095).abs();
    let world_ff_vel_err = (world_vy - (-9.81)).abs();

    println!(
        "\n[P4-T1 Benchmark 1: Free Fall (T=1.0s, dt=5ms, y_exact=5.095m, vy_exact=-9.81m/s)]\n\
         - DirectKktIndex2 (f64): pos_err = {:.3e} m, vel_err = {:.3e} m/s, time = {:.1} us\n\
         - PgsBaumgarte (f64):    pos_err = {:.3e} m, vel_err = {:.3e} m/s, time = {:.1} us\n\
         - PhysicsWorld (f64):    pos_err = {:.3e} m, vel_err = {:.3e} m/s, time = {:.1} us",
        ff_kkt.pos_error_m,
        ff_kkt.vel_error_m_s,
        ff_kkt.elapsed_us,
        ff_pgs.pos_error_m,
        ff_pgs.vel_error_m_s,
        ff_pgs.elapsed_us,
        world_ff_pos_err,
        world_ff_vel_err,
        world_ff_us
    );

    assert!(ff_kkt.pos_error_m < 1e-12);
    assert!(ff_kkt.vel_error_m_s < 1e-12);
    assert!(world_ff_pos_err < 1e-12);
    assert!(world_ff_vel_err < 1e-12);

    // -------------------------------------------------------------------------
    // Benchmark 2: Rotating Rod Centripetal Reaction (L = 0.20 m, omega = 10 rad/s, t = 0.20 s)
    // -------------------------------------------------------------------------
    let rod_kkt = run_rotating_rod_dae(DaeSolverCandidate::DirectKktIndex2Projection);
    let rod_pgs = run_rotating_rod_dae(DaeSolverCandidate::SemiImplicitEulerPgsBaumgarte);

    let mut rod_world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
    let mut rod_part = create_test_box("rotating_rod", "Rotating Rod", 0.20, 0.02, 0.02);
    for v in &mut rod_part.mesh.vertices {
        v.position[0] += 0.10;
    }
    rod_world.add_part(&rod_part, true).unwrap();
    rod_world
        .add_joint(
            "pivot_z",
            "ground",
            "rotating_rod",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .unwrap();
    rod_world
        .set_part_angular_velocity("rotating_rod", [0.0, 0.0, 10.0])
        .unwrap();
    rod_world
        .set_part_velocity("rotating_rod", [0.0, 1.0, 0.0])
        .unwrap();
    let t0 = std::time::Instant::now();
    let summary_rod = rod_world.simulate(0.20, 1.0 / 240.0);
    let world_rod_us = t0.elapsed().as_secs_f64() * 1e6;
    let lc_rod = rod_world
        .extract_part_load_case(&summary_rod, "rotating_rod", LoadCaseSelector::AtTime(0.20))
        .unwrap();
    let world_fx = lc_rod.joints[0].force[0];
    let world_rod_err_pct =
        ((world_fx - rod_kkt.analytical_fx_local_n).abs() / rod_kkt.analytical_fx_local_n.abs())
            * 100.0;
    let world_w_drift =
        (rod_world.part_angular_velocity("rotating_rod").unwrap()[2] - 10.0).abs();

    println!(
        "\n[P4-T1 Benchmark 2: Rotating Rod (omega=10 rad/s, F_exact={:.4} N)]\n\
         - DirectKktIndex2 (f64): F_x = {:.6} N, rel_err = {:.3e}%, w_drift = {:.3e} rad/s, time = {:.1} us\n\
         - PgsBaumgarte (f64):    F_x = {:.6} N, rel_err = {:.3e}%, w_drift = {:.3e} rad/s, time = {:.1} us\n\
         - PhysicsWorld (f64):    F_x = {:.6} N, rel_err = {:.3e}%, w_drift = {:.3e} rad/s, time = {:.1} us",
        rod_kkt.analytical_fx_local_n,
        rod_kkt.simulated_fx_local_n,
        rod_kkt.force_rel_error_pct,
        rod_kkt.omega_drift_rad_s,
        rod_kkt.elapsed_us,
        rod_pgs.simulated_fx_local_n,
        rod_pgs.force_rel_error_pct,
        rod_pgs.omega_drift_rad_s,
        rod_pgs.elapsed_us,
        world_fx,
        world_rod_err_pct,
        world_w_drift,
        world_rod_us
    );

    assert!(rod_kkt.force_rel_error_pct < 1e-7);
    assert!(rod_kkt.omega_drift_rad_s < 1e-11);
    assert!(world_rod_err_pct < 1e-7);
    assert!(world_w_drift < 1e-11);

    // -------------------------------------------------------------------------
    // Benchmark 3: Slider-Crank 4-Bar Closed Kinematic Loop (R=0.03m, L=0.05m, 1 rev)
    // -------------------------------------------------------------------------
    let sc_kkt = run_slider_crank_dae(DaeSolverCandidate::DirectKktIndex2Projection);
    let sc_pgs = run_slider_crank_dae(DaeSolverCandidate::SemiImplicitEulerPgsBaumgarte);

    println!(
        "\n[P4-T1 Benchmark 3: Slider-Crank Closed Loop (R=0.03m, L=0.05m, 500 steps)]\n\
         - DirectKktIndex2 (f64): max_pos_err = {:.3e} m, rms_pos_err = {:.3e} m, max_closure = {:.3e}, time = {:.1} us\n\
         - PgsBaumgarte (f64):    max_pos_err = {:.3e} m, rms_pos_err = {:.3e} m, max_closure = {:.3e}, time = {:.1} us",
        sc_kkt.max_slider_pos_error_m,
        sc_kkt.rms_slider_pos_error_m,
        sc_kkt.max_closure_residual,
        sc_kkt.elapsed_us,
        sc_pgs.max_slider_pos_error_m,
        sc_pgs.rms_slider_pos_error_m,
        sc_pgs.max_closure_residual,
        sc_pgs.elapsed_us
    );

    assert!(sc_kkt.max_slider_pos_error_m < 1e-9);
    assert!(sc_kkt.max_closure_residual < 1e-11);
    assert!(sc_kkt.max_closure_residual < sc_pgs.max_closure_residual * 1e-5);

    // -------------------------------------------------------------------------
    // Benchmark 4: Gear Pair over 10 Revolutions (ratio = 0.5, load_torque = -4.0 N*m)
    // -------------------------------------------------------------------------
    let gear_kkt = run_gear_pair_10_revs_dae(DaeSolverCandidate::DirectKktIndex2Projection);
    let gear_pgs = run_gear_pair_10_revs_dae(DaeSolverCandidate::SemiImplicitEulerPgsBaumgarte);

    println!(
        "\n[P4-T1 Benchmark 4: Gear Pair 10 Revolutions (ratio=0.5, load=-4.0 N*m, tau_motor_exact=2.0 N*m)]\n\
         - DirectKktIndex2 (f64): phase_err = {:.3e} rad, ratio_err = {:.3e}%, tau_motor = {:.6} N*m (err {:.3e}%), time = {:.1} us\n\
         - PgsBaumgarte (f64):    phase_err = {:.3e} rad, ratio_err = {:.3e}%, tau_motor = {:.6} N*m (err {:.3e}%), time = {:.1} us",
        gear_kkt.phase_error_rad,
        gear_kkt.velocity_ratio_error_pct,
        gear_kkt.transmitted_motor_torque_nm,
        gear_kkt.torque_error_pct,
        gear_kkt.elapsed_us,
        gear_pgs.phase_error_rad,
        gear_pgs.velocity_ratio_error_pct,
        gear_pgs.transmitted_motor_torque_nm,
        gear_pgs.torque_error_pct,
        gear_pgs.elapsed_us
    );

    assert!(gear_kkt.phase_error_rad < 1e-11);
    assert!(gear_kkt.velocity_ratio_error_pct < 1e-9);
    assert!(gear_kkt.torque_error_pct < 1e-7);
}

#[test]
fn test_p4_t2_f64_rigid_body_hollow_inertia_and_update_part_collider() {
    use mcp_cad_physics::{ColliderShape, compute_mesh_mass_properties};

    let registry = MaterialRegistry::new();
    let steel = registry.get("steel_structural_aisi1020").unwrap().clone();
    let density = steel.density_kg_m3;
    let r_out = 0.05_f64;
    let r_in = 0.03_f64;
    let height = 0.12_f64;
    let segments = 128;

    // 1. Create a solid cylinder part and add it to PhysicsWorld with a Revolute joint
    let mut flywheel = Part::new("flywheel", "Flywheel", steel);
    flywheel.dag.add_feature(
        FeatureOp::AddCylinder {
            radius: r_out,
            height,
            segments,
            position: None,
            rotation_deg: None,
        },
        None,
    );
    flywheel.rebuild().expect("rebuild solid flywheel");

    let mut world = PhysicsWorld::with_gravity([0.0, -9.81, 0.0]);
    let initial_handle = world.add_part(&flywheel, true).expect("add solid flywheel");
    let joint_handle = world
        .add_joint(
            "flywheel_shaft",
            "ground",
            "flywheel",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("add revolute joint");

    let solid_mass = world.part_mass("flywheel").unwrap();
    let solid_inertia = world.part_inertia_tensor("flywheel").unwrap();
    let exact_solid_vol = std::f64::consts::PI * r_out * r_out * height;
    let exact_solid_mass = density * exact_solid_vol;
    let exact_solid_izz = 0.5 * exact_solid_mass * r_out * r_out;

    assert!(
        ((solid_mass - exact_solid_mass) / exact_solid_mass).abs() < 1e-3,
        "Solid cylinder mass mismatch: got {solid_mass}, expected {exact_solid_mass}"
    );
    assert!(
        ((solid_inertia[(2, 2)] - exact_solid_izz) / exact_solid_izz).abs() < 2e-3,
        "Solid cylinder I_zz mismatch: got {}, expected {exact_solid_izz}",
        solid_inertia[(2, 2)]
    );

    // 2. Bore an internal cylindrical hole through the flywheel via CutHole
    // Note: The outer convex hull of the hollow flywheel is IDENTICAL to the solid cylinder,
    // so a convex-hull collider approximation would falsely report the solid cylinder's mass & inertia.
    // Our P4-T2 signed-tetrahedron mesh integration must account for the internal bore!
    flywheel.dag.add_feature(
        FeatureOp::CutHole {
            center: [0.0, 0.0, 0.0],
            normal: [0.0, 0.0, 1.0],
            radius: r_in,
            depth: height * 1.2,
            keyway_width: None,
            keyway_depth: None,
        },
        None,
    );
    flywheel.rebuild().expect("rebuild hollow flywheel");

    let updated_handle = world
        .update_part_collider(&flywheel)
        .expect("update_part_collider on hollow flywheel");

    // BodyHandle and attached joint must be preserved
    assert_eq!(
        updated_handle, initial_handle,
        "update_part_collider must preserve BodyHandle"
    );
    assert_eq!(world.joints().len(), 1);
    assert_eq!(world.joints()["flywheel_shaft"].handle, joint_handle);

    let hollow_mass = world.part_mass("flywheel").unwrap();
    let hollow_com = world.part_local_com("flywheel").unwrap();
    let hollow_inertia = world.part_inertia_tensor("flywheel").unwrap();

    let exact_hollow_vol = std::f64::consts::PI * (r_out * r_out - r_in * r_in) * height;
    let exact_hollow_mass = density * exact_hollow_vol;
    let exact_hollow_izz = 0.5 * exact_hollow_mass * (r_out * r_out + r_in * r_in);
    let exact_hollow_ixx =
        exact_hollow_mass * (3.0 * (r_out * r_out + r_in * r_in) + height * height) / 12.0;

    assert!(
        hollow_mass < solid_mass * 0.70,
        "Hollow flywheel mass ({hollow_mass}) must be significantly smaller than solid ({solid_mass})"
    );
    assert!(
        ((hollow_mass - exact_hollow_mass) / exact_hollow_mass).abs() < 5e-3,
        "Hollow mass mismatch: got {hollow_mass}, expected {exact_hollow_mass}"
    );
    assert!(
        hollow_com[0].abs() < 1e-5 && hollow_com[1].abs() < 1e-5 && hollow_com[2].abs() < 1e-5,
        "Hollow COM must be at origin: got {hollow_com:?}"
    );
    assert!(
        ((hollow_inertia[(2, 2)] - exact_hollow_izz) / exact_hollow_izz).abs() < 1e-2,
        "Hollow I_zz mismatch: got {}, expected {exact_hollow_izz}",
        hollow_inertia[(2, 2)]
    );
    assert!(
        ((hollow_inertia[(0, 0)] - exact_hollow_ixx) / exact_hollow_ixx).abs() < 1e-2,
        "Hollow I_xx mismatch: got {}, expected {exact_hollow_ixx}",
        hollow_inertia[(0, 0)]
    );

    // Direct check against compute_mesh_mass_properties
    let direct_mprops = compute_mesh_mass_properties(&flywheel.mesh, density);
    assert!((direct_mprops.mass - hollow_mass).abs() < 1e-15);
    assert!((direct_mprops.inertia_tensor - hollow_inertia).norm() < 1e-15);

    // 3. Verify all 6 ColliderType variants populate ColliderRegistry with exact f64 shapes
    for (idx, col_type) in [
        ColliderType::ConvexHull,
        ColliderType::TriMesh,
        ColliderType::Compound,
        ColliderType::Cube,
        ColliderType::Sphere,
        ColliderType::Cylinder,
    ]
    .into_iter()
    .enumerate()
    {
        let part_id = format!("shape_part_{idx}");
        let p = create_test_box(&part_id, &part_id, 0.08, 0.06, 0.04);
        let bh = world
            .add_part_with_options(&p, true, col_type, Some([1.0 + idx as f64, 0.0, 0.0]), None)
            .expect("add_part_with_options");
        let rb = world.body_registry().get(bh).expect("body in registry");
        assert_eq!(rb.part_id, part_id);
        assert!((rb.position.x - (1.0 + idx as f64)).abs() < 1e-15);

        let (aabb_min, aabb_max) = world.part_collider_aabb(&part_id).expect("collider aabb");
        assert!(aabb_max[0] > aabb_min[0] && aabb_max[1] > aabb_min[1] && aabb_max[2] > aabb_min[2]);
    }

    assert!(world.collider_registry().iter().any(|(_, c)| matches!(c.shape, ColliderShape::ConvexHull { .. })));
    assert!(world.collider_registry().iter().any(|(_, c)| matches!(c.shape, ColliderShape::TriMesh { .. })));
    assert!(world.collider_registry().iter().any(|(_, c)| matches!(c.shape, ColliderShape::Compound { .. })));
    assert!(world.collider_registry().iter().any(|(_, c)| matches!(c.shape, ColliderShape::Cube { .. })));
    assert!(world.collider_registry().iter().any(|(_, c)| matches!(c.shape, ColliderShape::Sphere { .. })));
    assert!(world.collider_registry().iter().any(|(_, c)| matches!(c.shape, ColliderShape::Cylinder { .. })));
}

#[test]
fn test_p4_t3_f64_multibody_solver_coupled_gear_torque_limits_and_closed_loop() {
    // 1. Coupled Gear constraint with external load torque on driven gear:
    //    Verify exact velocity ratio w_b = -ratio * w_a AND real torque transmission
    //    through Lagrange multipliers without any post-step set_angvel hacks.
    let mut gear_world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
    let gear_a = create_test_box("gear_a", "Drive Gear", 0.06, 0.06, 0.02);
    let gear_b = create_test_box("gear_b", "Driven Gear", 0.12, 0.12, 0.02);
    gear_world
        .add_part_with_options(
            &gear_a,
            true,
            ColliderType::ConvexHull,
            Some([0.0, 0.0, 0.0]),
            None,
        )
        .unwrap();
    gear_world
        .add_part_with_options(
            &gear_b,
            true,
            ColliderType::ConvexHull,
            Some([0.20, 0.0, 0.0]),
            None,
        )
        .unwrap();

    // Revolute mount for gear_a with motor driving at 6.0 rad/s (max torque 50 N*m)
    gear_world
        .add_joint(
            "mount_a",
            "ground",
            "gear_a",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig {
                target_velocity: 6.0,
                max_force: 50.0,
            }),
        )
        .unwrap();

    // Revolute mount for gear_b with opposing load motor (target_velocity = 0.0, max_force = 4.0 N*m)
    gear_world
        .add_joint(
            "mount_b",
            "ground",
            "gear_b",
            CadJointType::Revolute {
                anchor_a: [0.20, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig {
                target_velocity: 0.0,
                max_force: 4.0,
            }),
        )
        .unwrap();

    // Coupled Gear joint: w_b + 0.5 * w_a = 0 => w_b = -3.0 rad/s
    let ratio = 0.5_f64;
    gear_world
        .add_joint(
            "gear_pair",
            "gear_a",
            "gear_b",
            CadJointType::Gear {
                axis_a: [0.0, 0.0, 1.0],
                axis_b: [0.0, 0.0, 1.0],
                ratio,
            },
            None,
        )
        .unwrap();

    let gear_summary = gear_world.simulate(0.5, 0.005);
    assert!(gear_summary.success);
    let wa = gear_world.part_angular_velocity("gear_a").unwrap()[2];
    let wb = gear_world.part_angular_velocity("gear_b").unwrap()[2];
    assert!(
        (wa - 6.0).abs() < 1e-9,
        "Drive gear must reach target velocity 6.0 rad/s, got {wa}"
    );
    assert!(
        (wb - (-3.0)).abs() < 1e-9,
        "Driven gear must follow coupled ratio -0.5 * 6.0 = -3.0 rad/s, got {wb}"
    );

    // At steady state, gear_b experiences -4.0 N*m resistive torque from mount_b's saturated damper motor
    // (since wb = -3.0 < target 0.0, mount_b applies +4.0 N*m trying to bring wb up to 0.0).
    // Through the coupled Gear constraint J = [ratio, 1], the reaction torque on gear_b is -4.0 N*m
    // and on gear_a is -ratio * 4.0 = -2.0 N*m, which mount_a's motor balances with +2.0 N*m!
    let gear_rf = gear_world.joint_reaction_force("gear_pair", 0.005).unwrap();
    assert!(
        (gear_rf.torque[2].abs() - 2.0).abs() < 1e-6,
        "Gear constraint reaction torque on drive gear_a should be 2.0 N*m, got {}",
        gear_rf.torque[2]
    );
    let mount_a_rf = gear_world.joint_reaction_force("mount_a", 0.005).unwrap();
    assert!(
        (mount_a_rf.torque[2].abs() - 2.0).abs() < 1e-6,
        "Drive motor mount_a reaction torque should balance transmitted load (2.0 N*m), got {}",
        mount_a_rf.torque[2]
    );

    // 2. Closed-loop 4-bar / slider-crank mechanism holonomic manifold projection:
    //    Verify that over 200 steps, joint closure residual stays at machine precision (< 1e-11).
    let mut loop_world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
    let crank = create_test_box("crank", "Crank", 0.03, 0.01, 0.01);
    let rod = create_test_box("rod", "Connecting Rod", 0.05, 0.01, 0.01);
    let slider = create_test_box("slider", "Slider", 0.02, 0.02, 0.02);
    loop_world
        .add_part_with_options(&crank, true, ColliderType::ConvexHull, Some([0.015, 0.0, 0.0]), None)
        .unwrap();
    loop_world
        .add_part_with_options(&rod, true, ColliderType::ConvexHull, Some([0.055, 0.0, 0.0]), None)
        .unwrap();
    loop_world
        .add_part_with_options(&slider, true, ColliderType::ConvexHull, Some([0.08, 0.0, 0.0]), None)
        .unwrap();

    loop_world
        .add_joint(
            "j_crank",
            "ground",
            "crank",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [-0.015, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig {
                target_velocity: 4.0,
                max_force: 100.0,
            }),
        )
        .unwrap();
    loop_world
        .add_joint(
            "j_crank_rod",
            "crank",
            "rod",
            CadJointType::Revolute {
                anchor_a: [0.015, 0.0, 0.0],
                anchor_b: [-0.025, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .unwrap();
    loop_world
        .add_joint(
            "j_rod_slider",
            "rod",
            "slider",
            CadJointType::Revolute {
                anchor_a: [0.025, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .unwrap();
    loop_world
        .add_joint(
            "j_slider_ground",
            "ground",
            "slider",
            CadJointType::Prismatic {
                anchor_a: [0.08, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [1.0, 0.0, 0.0],
                limits: Some([-0.10, 0.10]),
            },
            None,
        )
        .unwrap();

    let loop_summary = loop_world.simulate(1.0, 0.005);
    assert!(loop_summary.success);
    let slider_pos = loop_world.part_position("slider").unwrap();
    // Slider Y and Z must remain locked to 0.0 to within holonomic projection tolerance
    assert!(
        slider_pos[1].abs() < 1e-11 && slider_pos[2].abs() < 1e-11,
        "Slider off-axis drift must be < 1e-11 m, got y={}, z={}",
        slider_pos[1],
        slider_pos[2]
    );
    // Analytical slider X position for crank angle theta = 4.0 * 1.0 = 4.0 rad:
    // x_slider = R * cos(theta) + sqrt(L^2 - R^2 * sin^2(theta)), with R = 0.03, L = 0.05
    let theta = 4.0_f64;
    let r_len = 0.03_f64;
    let l_len = 0.05_f64;
    let exact_slider_x =
        r_len * theta.cos() + (l_len * l_len - (r_len * theta.sin()).powi(2)).sqrt();
    assert!(
        (slider_pos[0] - exact_slider_x).abs() < 1e-4,
        "Slider X position {} must match analytical {} within 1e-4 m",
        slider_pos[0],
        exact_slider_x
    );
}

#[test]
fn test_p4_t4_sphere_elastic_restitution_and_collision_events() {
    use mcp_cad_physics::CollisionEventType;

    // 1. Elastic bounce (e = 1.0): Sphere dropped onto static TriMesh floor
    let mut world = PhysicsWorld::new(); // g = [0, -9.81, 0]
    let floor = create_test_box("tri_floor", "TriMesh Floor", 4.0, 0.2, 4.0);
    world
        .add_part_with_options(&floor, false, ColliderType::TriMesh, Some([0.0, 0.0, 0.0]), None)
        .expect("floor added");

    let r_sph = 0.2_f64;
    let sphere = create_test_sphere("bounce_sphere", "Bouncing Sphere", r_sph);
    let y0 = 1.3_f64; // floor top is y = 0.1, clear drop height = 1.0 m above impact (center at 0.3)
    world
        .add_part_with_options(
            &sphere,
            true,
            ColliderType::Sphere,
            Some([0.0, y0, 0.0]),
            None,
        )
        .expect("sphere added");
    world.set_part_contact_properties("tri_floor", 0.0, 1.0).unwrap();
    world.set_part_contact_properties("bounce_sphere", 0.0, 1.0).unwrap();

    // Free fall t_impact = sqrt(2 * 1.0 / 9.81) = 0.4515 s -> full bounce cycle ~ 0.90 s
    let summary = world.simulate(0.90, 0.005);
    assert!(summary.success);

    let started: Vec<_> = summary
        .collisions
        .iter()
        .filter(|c| c.event_type == CollisionEventType::Started)
        .collect();
    let stopped: Vec<_> = summary
        .collisions
        .iter()
        .filter(|c| c.event_type == CollisionEventType::Stopped)
        .collect();
    assert!(!started.is_empty(), "Expected CollisionEventType::Started on bounce");
    assert!(!stopped.is_empty(), "Expected CollisionEventType::Stopped after bounce separation");
    assert!(
        started[0].time_s < stopped[0].time_s,
        "Collision Started ({}) must precede Stopped ({})",
        started[0].time_s,
        stopped[0].time_s
    );
    assert!(
        started[0].contact_normal.is_some(),
        "Started CollisionRecord must report contact_normal"
    );
    let imp = started[0].max_impulse.expect("Started CollisionRecord must report max_impulse");
    assert!(imp > 0.0, "Normal impulse on bounce must be positive, got {imp}");

    // Apex reached after bounce should recover at least 85% of drop height above floor surface
    let max_y = summary.trajectories["bounce_sphere"]
        .samples
        .iter()
        .skip(100) // after t = 0.5 s (post-impact)
        .map(|s| s.position[1])
        .fold(f64::NEG_INFINITY, f64::max);
    let apex_clear_height = max_y - (0.1 + r_sph);
    assert!(
        apex_clear_height > 0.85,
        "Elastic sphere (e=1.0) should rebound to >0.85 m clearance, got {apex_clear_height:.4} m"
    );

    // 2. Plastic impact (e = 0.0): Cube dropped onto TriMesh floor settles without bouncing or falling through
    let mut plastic_world = PhysicsWorld::new();
    plastic_world
        .add_part_with_options(&floor, false, ColliderType::TriMesh, Some([0.0, 0.0, 0.0]), None)
        .unwrap();
    let cube = create_test_box("rest_cube", "Resting Cube", 0.4, 0.4, 0.4);
    plastic_world
        .add_part_with_options(&cube, true, ColliderType::Cube, Some([0.0, 0.6, 0.0]), None)
        .unwrap();
    plastic_world.set_part_contact_properties("tri_floor", 0.5, 0.0).unwrap();
    plastic_world.set_part_contact_properties("rest_cube", 0.5, 0.0).unwrap();

    let plastic_summary = plastic_world.simulate(1.0, 0.005);
    assert!(plastic_summary.success);
    let cube_pos = plastic_world.part_position("rest_cube").unwrap();
    let cube_vel = plastic_world.part_velocity("rest_cube").unwrap();
    // Floor top = 0.1, cube half-height = 0.2 -> exact resting center y = 0.3
    assert!(
        (cube_pos[1] - 0.3).abs() < 5e-4,
        "Plastic cube should rest at y=0.3 m without bounce/sink, got {}",
        cube_pos[1]
    );
    assert!(
        cube_vel[1].abs() < 1e-3,
        "Plastic cube should have near-zero vertical velocity, got {}",
        cube_vel[1]
    );
}

#[test]
fn test_p4_t4_coulomb_friction_static_sticking_and_kinetic_inclined_plane() {
    use nalgebra::{UnitQuaternion, Vector3};

    // Tilted plane at theta = 30 deg (sin = 0.5, cos = sqrt(3)/2, tan = 0.57735)
    let theta = 30.0_f64.to_radians();
    let tilt_axis = Vector3::z_axis();
    let q_tilt = UnitQuaternion::from_axis_angle(&tilt_axis, -theta);
    let q_arr = [
        q_tilt.quaternion().coords.x,
        q_tilt.quaternion().coords.y,
        q_tilt.quaternion().coords.z,
        q_tilt.quaternion().coords.w,
    ];
    let n_slope = q_tilt * Vector3::y();
    let d_slope = q_tilt * Vector3::x(); // Downslope (+X tilted downward by 30 deg around Z)

    let ramp = create_test_box("ramp", "Inclined Ramp", 6.0, 0.2, 2.0);
    let block = create_test_box("block", "Sliding Block", 0.2, 0.2, 0.2);
    // Offset perpendicular to ramp = ramp half-thickness (0.1) + block half-height (0.1) = 0.2
    let block_pos = n_slope * 0.2;

    // Case A: Static sticking (mu = 0.8 > tan(30 deg) = 0.57735)
    let mut static_world = PhysicsWorld::new();
    static_world
        .add_part_with_options(&ramp, false, ColliderType::Cube, Some([0.0, 0.0, 0.0]), Some(q_arr))
        .unwrap();
    static_world
        .add_part_with_options(
            &block,
            true,
            ColliderType::Cube,
            Some([block_pos.x, block_pos.y, block_pos.z]),
            Some(q_arr),
        )
        .unwrap();
    static_world.set_part_contact_properties("ramp", 0.8, 0.0).unwrap();
    static_world.set_part_contact_properties("block", 0.8, 0.0).unwrap();

    let static_summary = static_world.simulate(0.5, 0.005);
    assert!(static_summary.success);
    let p_static = Vector3::from(static_world.part_position("block").unwrap());
    let v_static = Vector3::from(static_world.part_velocity("block").unwrap());
    assert!(
        (p_static - block_pos).norm() < 5e-4,
        "Block with mu=0.8 > tan(30 deg) must stick on incline, drifted by {} m",
        (p_static - block_pos).norm()
    );
    assert!(
        v_static.norm() < 1e-3,
        "Sticking block velocity must stay ~0, got {} m/s",
        v_static.norm()
    );

    // Case B: Kinetic sliding (mu = 0.2 < tan(30 deg))
    // Analytical acceleration along downslope: a = g * (sin(theta) - mu * cos(theta))
    let mu = 0.2_f64;
    let mut slide_world = PhysicsWorld::new();
    slide_world
        .add_part_with_options(&ramp, false, ColliderType::Cube, Some([0.0, 0.0, 0.0]), Some(q_arr))
        .unwrap();
    slide_world
        .add_part_with_options(
            &block,
            true,
            ColliderType::Cube,
            Some([block_pos.x, block_pos.y, block_pos.z]),
            Some(q_arr),
        )
        .unwrap();
    slide_world.set_part_contact_properties("ramp", mu, 0.0).unwrap();
    slide_world.set_part_contact_properties("block", mu, 0.0).unwrap();

    let t_total = 0.5_f64;
    let slide_summary = slide_world.simulate(t_total, 0.005);
    assert!(slide_summary.success);

    let p_slide = Vector3::from(slide_world.part_position("block").unwrap());
    let v_slide = Vector3::from(slide_world.part_velocity("block").unwrap());
    let exact_a = 9.81 * (theta.sin() - mu * theta.cos());
    let exact_speed = exact_a * t_total;
    let exact_dist = 0.5 * exact_a * t_total * t_total;

    let actual_speed = v_slide.dot(&d_slope);
    let actual_dist = (p_slide - block_pos).dot(&d_slope);

    assert!(
        (actual_speed - exact_speed).abs() / exact_speed < 1e-4,
        "Sliding speed = {actual_speed:.6} m/s, analytical = {exact_speed:.6} m/s"
    );
    assert!(
        (actual_dist - exact_dist).abs() / exact_dist < 0.015,
        "Sliding distance = {actual_dist:.6} m, analytical = {exact_dist:.6} m"
    );

    // Case C: Isotropic Coulomb cone invariance at 37 deg diagonal sliding on a flat TriMesh floor
    // Ensures ||gamma_t||_2 = mu * gamma_n rather than pyramid sqrt(2) inflation.
    let mut iso_world = PhysicsWorld::new();
    let flat_floor = create_test_box("iso_floor", "Flat Floor", 8.0, 0.2, 8.0);
    iso_world
        .add_part_with_options(&flat_floor, false, ColliderType::TriMesh, Some([0.0, 0.0, 0.0]), None)
        .unwrap();
    let puck = create_test_box("iso_puck", "Diagonal Puck", 0.2, 0.2, 0.2);
    iso_world
        .add_part_with_options(&puck, true, ColliderType::Cube, Some([0.0, 0.2, 0.0]), None)
        .unwrap();
    let mu_iso = 0.25_f64;
    iso_world.set_part_contact_properties("iso_floor", mu_iso, 0.0).unwrap();
    iso_world.set_part_contact_properties("iso_puck", mu_iso, 0.0).unwrap();
    let dir_37 = Vector3::new(37.0_f64.to_radians().cos(), 0.0, 37.0_f64.to_radians().sin());
    let v0_mag = 2.0_f64;
    iso_world
        .set_part_velocity("iso_puck", [dir_37.x * v0_mag, dir_37.y * v0_mag, dir_37.z * v0_mag])
        .unwrap();

    let t_iso = 0.4_f64;
    let iso_summary = iso_world.simulate(t_iso, 0.005);
    assert!(iso_summary.success);
    let v_puck = Vector3::from(iso_world.part_velocity("iso_puck").unwrap());
    let expected_v_mag = v0_mag - mu_iso * 9.81 * (t_iso - 0.005); // step 0 establishes contact
    assert!(
        (v_puck.norm() - expected_v_mag).abs() < 0.03,
        "Diagonal isotropic sliding speed = {:.4} m/s, expected ~{expected_v_mag:.4} m/s",
        v_puck.norm()
    );
    let heading_cos = v_puck.normalize().dot(&dir_37);
    assert!(
        heading_cos > 0.999,
        "Isotropic Coulomb friction must preserve sliding heading at 37 deg, cos={heading_cos:.6}"
    );
}

#[test]
fn test_p4_t4_compound_concavity_cylinder_convexhull_and_joint_contact_filter() {
    use mcp_cad_core::dag::FeatureOp;
    use mcp_cad_core::material::MaterialRegistry;
    use nalgebra::{UnitQuaternion, Vector3};

    let reg = MaterialRegistry::new();
    let steel = reg.get("steel_structural_aisi1020").unwrap().clone();

    // 1. L-shaped concave Compound collider: base [x: -0.3..0.3, y: 0.0..0.1] + wall [x: -0.3..-0.2, y: 0.0..0.6]
    let mut l_bracket = Part::new("l_bracket", "L Bracket", steel);
    l_bracket.dag.add_feature(
        FeatureOp::AddBox {
            width: 0.6,
            height: 0.1,
            depth: 0.4,
            position: Some([0.0, 0.05, 0.0]),
            rotation_deg: None,
        },
        None,
    );
    l_bracket.dag.add_feature(
        FeatureOp::AddBox {
            width: 0.1,
            height: 0.6,
            depth: 0.4,
            position: Some([-0.25, 0.30, 0.0]),
            rotation_deg: None,
        },
        None,
    );
    l_bracket.rebuild().unwrap();

    let mut world = PhysicsWorld::new();
    world
        .add_part_with_options(&l_bracket, false, ColliderType::Compound, None, None)
        .unwrap();

    // Drop sphere at x = 0.15, y = 0.45 into the concave pocket above the base (at y = 0.1).
    let probe = create_test_sphere("probe_sph", "Probe Sphere", 0.05);
    world
        .add_part_with_options(&probe, true, ColliderType::Sphere, Some([0.15, 0.45, 0.0]), None)
        .unwrap();

    let summary = world.simulate(0.6, 0.005);
    assert!(summary.success);
    let py = world.part_position("probe_sph").unwrap()[1];
    assert!(
        (0.145..0.20).contains(&py),
        "Sphere dropped into L-bracket pocket should land on base y=0.15, not false hull roof or fall through: y={py:.4}"
    );

    // 2. Horizontal Cylinder (r=0.15 -> y=0.25), Upright Rotated Cylinder (half_h=0.2 -> y=0.30),
    //    and ConvexHull (half_h=0.15 -> y=0.25) on a TriMesh floor
    let mut world2 = PhysicsWorld::new();
    let floor = create_test_box("floor2", "Floor", 4.0, 0.2, 4.0);
    world2
        .add_part_with_options(&floor, false, ColliderType::TriMesh, Some([0.0, 0.0, 0.0]), None)
        .unwrap();

    let cyl_horiz = create_test_cylinder("horiz_cyl", "Horizontal Cylinder", 0.15, 0.4);
    world2
        .add_part_with_options(&cyl_horiz, true, ColliderType::Cylinder, Some([-1.0, 0.255, 0.0]), None)
        .unwrap();

    let q_upright = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
    let q_up_arr = [
        q_upright.quaternion().coords.x,
        q_upright.quaternion().coords.y,
        q_upright.quaternion().coords.z,
        q_upright.quaternion().coords.w,
    ];
    let cyl_up = create_test_cylinder("stand_cyl", "Standing Cylinder", 0.15, 0.4);
    world2
        .add_part_with_options(&cyl_up, true, ColliderType::Cylinder, Some([-0.4, 0.305, 0.0]), Some(q_up_arr))
        .unwrap();

    let hull_box = create_test_box("hull_box", "Hull Box", 0.3, 0.3, 0.3);
    world2
        .add_part_with_options(&hull_box, true, ColliderType::ConvexHull, Some([0.5, 0.26, 0.0]), None)
        .unwrap();

    // Overlapping pair joined with contacts_enabled == false (default) must NOT collide or bounce apart
    let j_a = create_test_box("j_a", "Joined A", 0.2, 0.2, 0.2);
    let j_b = create_test_box("j_b", "Joined B", 0.2, 0.2, 0.2);
    world2
        .add_part_with_options(&j_a, false, ColliderType::Cube, Some([0.0, 2.0, 0.0]), None)
        .unwrap();
    world2
        .add_part_with_options(&j_b, true, ColliderType::Cube, Some([0.05, 2.0, 0.0]), None)
        .unwrap();
    world2
        .add_joint(
            "hinge_overlap",
            "j_a",
            "j_b",
            CadJointType::Revolute {
                anchor_a: [0.05, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .unwrap();

    let sum2 = world2.simulate(0.5, 0.005);
    assert!(sum2.success);
    let cyl_h_y = world2.part_position("horiz_cyl").unwrap()[1];
    let cyl_u_y = world2.part_position("stand_cyl").unwrap()[1];
    let hull_y = world2.part_position("hull_box").unwrap()[1];
    assert!((cyl_h_y - 0.25).abs() < 2e-3, "Horizontal Cylinder must rest on TriMesh floor at y=0.25, got {cyl_h_y}");
    assert!((cyl_u_y - 0.30).abs() < 2e-3, "Upright Cylinder must rest stably on TriMesh floor at y=0.30, got {cyl_u_y}");
    assert!((hull_y - 0.25).abs() < 2e-3, "ConvexHull must rest stably on TriMesh floor at y=0.25, got {hull_y}");

    let self_collisions = sum2
        .collisions
        .iter()
        .any(|c| (c.body_a == "j_a" && c.body_b == "j_b") || (c.body_a == "j_b" && c.body_b == "j_a"));
    assert!(
        !self_collisions,
        "Jointed pair with contacts_enabled=false must be filtered out of broad-phase contact detection"
    );
}

#[test]
fn test_p4_t5_exact_lagrange_reactions_limits_motors_gear_and_prismatic() {
    use mcp_cad_physics::LoadCaseSelector;

    // 1. Revolute joint with travel limits holding a horizontal arm against gravity:
    //    At limit theta = 0.0 (limits = [0.0, 0.5] rad), gravity (-Y) pulls the arm downward
    //    towards negative theta, so the lower travel limit stop (theta_min = 0.0) must exert an
    //    exact holding torque T_z = +m * g * (L / 2) and vertical pin force F_y = +m * g!
    let g = 9.81_f64;
    let mut limit_world = PhysicsWorld::with_gravity([0.0, -g, 0.0]);
    let length_m = 0.20_f64;
    let r_c = length_m * 0.5;
    let mut arm = create_test_box("limit_arm", "Limit Arm", length_m, 0.02, 0.02);
    for v in &mut arm.mesh.vertices {
        v.position[0] += r_c;
    }
    limit_world.add_part(&arm, true).unwrap();
    let arm_mass = limit_world.part_mass("limit_arm").unwrap();
    let expected_fy = arm_mass * g;
    let expected_tz = arm_mass * g * r_c;

    limit_world
        .add_joint(
            "hinge_with_stop",
            "ground",
            "limit_arm",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: Some([0.0, 0.5]),
            },
            None,
        )
        .unwrap();

    let limit_summary = limit_world.simulate(0.1, 1.0 / 240.0);
    assert!(limit_summary.success);

    let rf_stop = limit_world
        .joint_reaction_force("hinge_with_stop", 1.0 / 240.0)
        .unwrap();
    // On part_a ("ground"), force is -m*g along Y and torque is -m*g*r_c along Z
    assert!(
        ((rf_stop.force[1] + expected_fy).abs() / expected_fy) < 1e-4,
        "Limit stop joint_reaction_force Fy on ground should be -m*g, got {:?}",
        rf_stop.force
    );
    assert!(
        ((rf_stop.torque[2] + expected_tz).abs() / expected_tz) < 1e-4,
        "Limit stop joint_reaction_force Tz on ground should be -m*g*r_c, got {:?}",
        rf_stop.torque
    );

    let arm_lc = limit_world
        .extract_part_load_case(&limit_summary, "limit_arm", LoadCaseSelector::PeakTorque)
        .unwrap();
    let ground_lc = limit_world
        .extract_part_load_case(&limit_summary, "ground", LoadCaseSelector::PeakTorque)
        .unwrap();
    assert!(
        ((arm_lc.joints[0].force[1] - expected_fy).abs() / expected_fy) < 1e-4,
        "Limit arm Fy error must be < 0.01%, got {:?}",
        arm_lc.joints[0].force
    );
    assert!(
        ((arm_lc.joints[0].torque[2] - expected_tz).abs() / expected_tz) < 1e-4,
        "Limit arm Tz from travel stop must be < 0.01%, got {:?}",
        arm_lc.joints[0].torque
    );
    assert!(
        arm_lc.equilibrium_residual_force_norm < 1e-9,
        "Limit arm D'Alembert force residual must be < 1e-9, got {:.3e}",
        arm_lc.equilibrium_residual_force_norm
    );
    assert!(
        arm_lc.equilibrium_residual_torque_norm < 1e-9,
        "Limit arm D'Alembert torque residual must be < 1e-9, got {:.3e}",
        arm_lc.equilibrium_residual_torque_norm
    );
    for k in 0..3 {
        assert!(
            (arm_lc.joints[0].force_world[k] + ground_lc.joints[0].force_world[k]).abs() < 1e-12,
            "Newton 3rd law force on limit joint violated at axis {k}"
        );
        assert!(
            (arm_lc.joints[0].torque_world[k] + ground_lc.joints[0].torque_world[k]).abs() < 1e-12,
            "Newton 3rd law torque on limit joint violated at axis {k}"
        );
    }

    // 2. Motorized Revolute joint accelerating an eccentric rod with saturated torque (max_force = 3.0 N*m):
    //    Verify that the coupled KKT reaction solve extracts both the motor torque Tz = +3.0 N*m
    //    AND the coupled transverse pin shear force Fy = m * alpha * r_c so that D'Alembert
    //    dynamic equilibrium holds on the accelerating rod!
    let mut motor_world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
    let mut m_rod = create_test_box("motor_rod", "Motor Rod", length_m, 0.04, 0.04);
    for v in &mut m_rod.mesh.vertices {
        v.position[0] += r_c;
    }
    motor_world.add_part(&m_rod, true).unwrap();
    motor_world
        .add_joint(
            "motor_hinge",
            "ground",
            "motor_rod",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig::new(500.0, 3.0)),
        )
        .unwrap();

    let motor_summary = motor_world.simulate(0.05, 0.002);
    assert!(motor_summary.success);
    let m_rod_lc = motor_world
        .extract_part_load_case(&motor_summary, "motor_rod", LoadCaseSelector::AtTime(0.02))
        .unwrap();
    let m_gnd_lc = motor_world
        .extract_part_load_case(&motor_summary, "ground", LoadCaseSelector::AtTime(0.02))
        .unwrap();
    assert!(
        (m_rod_lc.joints[0].torque[2] - 3.0).abs() < 1e-6,
        "Expected motor torque +3.0 N*m on accelerating rod, got {:?}",
        m_rod_lc.joints[0].torque
    );
    assert!(
        m_rod_lc.equilibrium_residual_force_norm < 1e-6,
        "Accelerating motorized rod force equilibrium residual must be < 1e-6 N, got {:.3e}",
        m_rod_lc.equilibrium_residual_force_norm
    );
    assert!(
        m_rod_lc.equilibrium_residual_torque_norm < 1e-6,
        "Accelerating motorized rod torque equilibrium residual must be < 1e-6 N*m, got {:.3e}",
        m_rod_lc.equilibrium_residual_torque_norm
    );
    for k in 0..3 {
        assert!(
            (m_rod_lc.joints[0].force_world[k] + m_gnd_lc.joints[0].force_world[k]).abs() < 1e-12
        );
        assert!(
            (m_rod_lc.joints[0].torque_world[k] + m_gnd_lc.joints[0].torque_world[k]).abs() < 1e-12
        );
    }

    // 3. Coupled Gear joint load-case extraction on BOTH `gear_a` (`part_a`) and `gear_b` (`part_b`)
    //    with ratio = 0.5: verify exact torque equilibrium on both gears!
    let mut gear_world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
    let gear_a = create_test_cylinder("gear_a", "Drive Gear", 0.04, 0.02);
    let gear_b = create_test_cylinder("gear_b", "Driven Gear", 0.08, 0.02);
    gear_world
        .add_part_with_options(&gear_a, true, ColliderType::Cylinder, Some([0.0, 0.0, 0.0]), None)
        .unwrap();
    gear_world
        .add_part_with_options(&gear_b, true, ColliderType::Cylinder, Some([0.15, 0.0, 0.0]), None)
        .unwrap();
    gear_world
        .add_joint(
            "mount_a",
            "ground",
            "gear_a",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig::new(6.0, 20.0)),
        )
        .unwrap();
    gear_world
        .add_joint(
            "mount_b",
            "ground",
            "gear_b",
            CadJointType::Revolute {
                anchor_a: [0.15, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig::new(0.0, 4.0)),
        )
        .unwrap();
    gear_world
        .add_joint(
            "gear_pair",
            "gear_a",
            "gear_b",
            CadJointType::Gear {
                axis_a: [0.0, 0.0, 1.0],
                axis_b: [0.0, 0.0, 1.0],
                ratio: 0.5,
            },
            None,
        )
        .unwrap();

    let gear_summary = gear_world.simulate(0.5, 0.005);
    let lc_a = gear_world
        .extract_part_load_case(&gear_summary, "gear_a", LoadCaseSelector::AtTime(0.45))
        .unwrap();
    let lc_b = gear_world
        .extract_part_load_case(&gear_summary, "gear_b", LoadCaseSelector::AtTime(0.45))
        .unwrap();
    assert!(
        lc_a.equilibrium_residual_torque_norm < 1e-8,
        "Drive gear_a torque equilibrium residual must be < 1e-8 N*m, got {:.3e}",
        lc_a.equilibrium_residual_torque_norm
    );
    assert!(
        lc_b.equilibrium_residual_torque_norm < 1e-8,
        "Driven gear_b torque equilibrium residual must be < 1e-8 N*m, got {:.3e}",
        lc_b.equilibrium_residual_torque_norm
    );

    // 4. Prismatic joint with translated slider (s = 0.25 m along +X) under -Y gravity:
    //    Verify that the reaction couple on `part_b` about `anchor_b` properly accounts for the
    //    slider displacement `d = p_b - p_a` so that moment equilibrium on the slider holds to < 1e-9!
    let mut prism_world = PhysicsWorld::with_gravity([0.0, -g, 0.0]);
    let slider = create_test_box("slider_block", "Slider Block", 0.04, 0.04, 0.04);
    prism_world
        .add_part_with_options(&slider, true, ColliderType::Cube, Some([0.25, 0.0, 0.0]), None)
        .unwrap();
    prism_world
        .add_joint(
            "rail_guide",
            "ground",
            "slider_block",
            CadJointType::Prismatic {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [1.0, 0.0, 0.0],
                limits: Some([0.25, 0.25]),
            },
            None,
        )
        .unwrap();

    let prism_summary = prism_world.simulate(0.1, 0.005);
    let slider_lc = prism_world
        .extract_part_load_case(&prism_summary, "slider_block", LoadCaseSelector::AtTime(0.05))
        .unwrap();
    assert!(
        slider_lc.equilibrium_residual_force_norm < 1e-9,
        "Translated prismatic slider force equilibrium residual must be < 1e-9 N, got {:.3e}",
        slider_lc.equilibrium_residual_force_norm
    );
    assert!(
        slider_lc.equilibrium_residual_torque_norm < 1e-9,
        "Translated prismatic slider torque equilibrium residual must be < 1e-9 N*m, got {:.3e}",
        slider_lc.equilibrium_residual_torque_norm
    );
}

#[test]
fn test_p4_t6_native_handles_and_f64_motor_validation() {
    use mcp_cad_physics::{BodyHandle, JointHandle, PhysicsError};

    let mut world = PhysicsWorld::with_gravity([0.0, -9.81, 0.0]);
    let ground_handle: BodyHandle = world
        .part_body_handle("ground")
        .expect("ground handle must exist");
    let world_alias_handle: BodyHandle = world
        .part_body_handle("WORLD")
        .expect("world alias handle must exist");
    assert_eq!(ground_handle, world_alias_handle);

    let mut rotor = create_test_box("rotor_p4t6", "Rotor P4-T6", 0.1, 0.02, 0.02);
    let body_handle: BodyHandle = world.add_part(&rotor, true).expect("add rotor_p4t6");
    assert_eq!(world.part_body_handle("rotor_p4t6").unwrap(), body_handle);

    let joint_handle: JointHandle = world
        .add_joint(
            "j_rotor_p4t6",
            "ground",
            "rotor_p4t6",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .expect("add j_rotor_p4t6");
    assert_eq!(world.joint_handle("j_rotor_p4t6").unwrap(), joint_handle);
    assert_eq!(world.joints()["j_rotor_p4t6"].handle, joint_handle);

    // Updating geometry preserves both BodyHandle and JointHandle
    rotor.dag.add_feature(
        FeatureOp::AddBox {
            width: 0.12,
            height: 0.03,
            depth: 0.03,
            position: None,
            rotation_deg: None,
        },
        None,
    );
    rotor.rebuild().unwrap();
    let updated_body_handle = world.update_part_collider(&rotor).unwrap();
    assert_eq!(updated_body_handle, body_handle);
    assert_eq!(world.joint_handle("j_rotor_p4t6").unwrap(), joint_handle);

    // Non-existent part/joint handle queries return structured errors
    assert!(matches!(
        world.part_body_handle("missing_part"),
        Err(PhysicsError::PartNotFound(_))
    ));
    assert!(matches!(
        world.joint_handle("missing_joint"),
        Err(PhysicsError::JointNotFound(_))
    ));
}

#[test]
fn test_p4_t7_f64_solver_comprehensive_benchmarks_euler_top_10rev_gear_4bar_friction_and_fea() {
    use mcp_cad_physics::{
        BodyRegistry, ColliderRegistry, ColliderType, LoadCaseSelector, MotorConfig,
        MultiBodySolver, RigidBody,
    };

    // =========================================================================
    // 1. Free-fall under gravity in f64 (T = 1.0 s, dt = 0.005 s)
    // =========================================================================
    let g = 9.81_f64;
    let mut ff_world = PhysicsWorld::with_gravity([0.0, -g, 0.0]);
    let ball = create_test_box("ff_mass", "Free Fall Mass", 0.1, 0.1, 0.1);
    ff_world
        .add_part_with_options(
            &ball,
            true,
            ColliderType::Cube,
            Some([0.0, 10.0, 0.0]),
            None,
        )
        .unwrap();
    let ff_summary = ff_world.simulate(1.0, 0.005);
    assert!(ff_summary.success);
    let ff_final = ff_summary.trajectories["ff_mass"].samples.last().unwrap();
    let y_initial = ff_summary.trajectories["ff_mass"].samples[0].position[1];
    let y_expected = y_initial - 0.5 * g * 1.0_f64.powi(2);
    let vy_expected = -g * 1.0;
    assert!(
        (ff_final.position[1] - y_expected).abs() < 1e-12,
        "f64 free-fall position error must be < 1e-12 m, got {:.3e}",
        (ff_final.position[1] - y_expected).abs()
    );
    assert!(
        (ff_final.linear_velocity[1] - vy_expected).abs() < 1e-12,
        "f64 free-fall velocity error must be < 1e-12 m/s, got {:.3e}",
        (ff_final.linear_velocity[1] - vy_expected).abs()
    );

    // =========================================================================
    // 2. Torque-free 3D asymmetric spinning body (Euler Top: I_x != I_y != I_z)
    //    Verify conservation of rotational kinetic energy E_rot = 0.5 * w . (I_w * w)
    //    and world angular momentum L_w = I_w * w over 500 steps (T = 1.0 s)
    // =========================================================================
    let euler_top_part = create_test_box("euler_top", "Asymmetric Euler Top", 0.30, 0.20, 0.10);
    let mut top_bodies = BodyRegistry::new();
    let mut top_colliders = ColliderRegistry::new();
    let mut top_solver = MultiBodySolver::new();
    let mut top_rb = RigidBody::from_part(&euler_top_part, true, None, None);
    top_rb.angular_velocity = nalgebra::Vector3::new(2.5, 1.5, 6.0);
    let top_h = top_bodies.insert(top_rb);

    let (e_rot_0, l_world_0) = {
        let body = top_bodies.get(top_h).unwrap();
        let i_w = body.world_inertia();
        let w = body.angular_velocity;
        let l_w = i_w * w;
        (0.5 * w.dot(&l_w), l_w)
    };
    assert!(e_rot_0 > 0.0 && l_world_0.norm() > 0.0);

    let dt_top = 0.002_f64;
    let mut empty_joints = Vec::new();
    for step in 0..500 {
        let (ok, _) = top_solver.step(
            &mut top_bodies,
            &mut top_colliders,
            &mut empty_joints,
            [0.0, 0.0, 0.0],
            dt_top,
            step,
            (step as f64) * dt_top,
        );
        assert!(ok, "Euler top step {step} must succeed");
    }
    let (e_rot_final, l_world_final) = {
        let body = top_bodies.get(top_h).unwrap();
        let i_w = body.world_inertia();
        let w = body.angular_velocity;
        let l_w = i_w * w;
        (0.5 * w.dot(&l_w), l_w)
    };
    let rel_energy_err = (e_rot_final - e_rot_0).abs() / e_rot_0;
    let rel_ang_mom_norm_err = (l_world_final.norm() - l_world_0.norm()).abs() / l_world_0.norm();
    let rel_ang_mom_vec_err = (l_world_final - l_world_0).norm() / l_world_0.norm();
    assert!(
        rel_energy_err < 1e-13,
        "Torque-free 3D Euler top rotational kinetic energy relative error must be < 1e-13, got {:.3e}",
        rel_energy_err
    );
    assert!(
        rel_ang_mom_norm_err < 1e-13,
        "Torque-free 3D Euler top |L| relative error must be < 1e-13, got {:.3e}",
        rel_ang_mom_norm_err
    );
    assert!(
        rel_ang_mom_vec_err < 1e-4,
        "Torque-free 3D Euler top 3D vector L_world relative drift must be < 1e-4, got {:.3e}",
        rel_ang_mom_vec_err
    );

    // =========================================================================
    // 3. 10-Revolution Gear Pair with Torque Transmission in PhysicsWorld
    //    w_a = 20 rad/s, T = pi s => theta_a = 20 * pi rad (10 full revolutions),
    //    ratio = 0.5 => w_b = -10 rad/s, theta_b = -10 * pi rad (5 full revolutions).
    //    Resistive damper motor (target_velocity = 0.0, max_force = 4.0 N*m) on driven gear_b
    //    => transmitted drive reaction torque |tau_drive| = ratio * |tau_load| = 2.0 N*m.
    // =========================================================================
    let mut gear10_world = PhysicsWorld::with_gravity([0.0, 0.0, 0.0]);
    let gear_a = create_test_cylinder("gear10_a", "Pinion 10-Rev", 0.04, 0.02);
    let gear_b = create_test_cylinder("gear10_b", "Wheel 10-Rev", 0.08, 0.02);
    gear10_world
        .add_part_with_options(
            &gear_a,
            true,
            ColliderType::Cylinder,
            Some([0.0, 0.0, 0.0]),
            None,
        )
        .unwrap();
    gear10_world
        .add_part_with_options(
            &gear_b,
            true,
            ColliderType::Cylinder,
            Some([0.25, 0.0, 0.0]),
            None,
        )
        .unwrap();
    gear10_world
        .add_joint(
            "rev_a10",
            "ground",
            "gear10_a",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig::new(20.0, 500.0)),
        )
        .unwrap();
    gear10_world
        .add_joint(
            "rev_b10",
            "ground",
            "gear10_b",
            CadJointType::Revolute {
                anchor_a: [0.25, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig::new(0.0, 4.0)),
        )
        .unwrap();
    gear10_world
        .add_joint(
            "gear_mesh10",
            "gear10_a",
            "gear10_b",
            CadJointType::Gear {
                axis_a: [0.0, 0.0, 1.0],
                axis_b: [0.0, 0.0, 1.0],
                ratio: 0.5,
            },
            None,
        )
        .unwrap();

    let n_steps_10rev = 628usize;
    let dt_10rev = std::f64::consts::PI / (n_steps_10rev as f64);
    let mut accum_theta_a = 0.0_f64;
    let mut accum_theta_b = 0.0_f64;
    for step in 0..n_steps_10rev {
        let _ = gear10_world.step(dt_10rev);
        let wa = gear10_world.part_angular_velocity("gear10_a").unwrap()[2];
        let wb = gear10_world.part_angular_velocity("gear10_b").unwrap()[2];
        assert!(
            (wb + 0.5 * wa).abs() < 1e-11,
            "10-rev gear velocity constraint residual at step {step} must be < 1e-11, got {:.3e}",
            (wb + 0.5 * wa).abs()
        );
        accum_theta_a += wa * dt_10rev;
        accum_theta_b += wb * dt_10rev;
    }
    let expected_theta_a = 20.0 * std::f64::consts::PI; // 10 * 2*pi
    let expected_theta_b = -10.0 * std::f64::consts::PI; // -5 * 2*pi
    assert!(
        (accum_theta_a - expected_theta_a).abs() < 1e-9,
        "Pinion must complete 10 revolutions (20*pi rad), got {:.6}",
        accum_theta_a
    );
    assert!(
        (accum_theta_b - expected_theta_b).abs() < 1e-9,
        "Wheel must complete 5 opposite revolutions (-10*pi rad), got {:.6}",
        accum_theta_b
    );

    let gear_rf = gear10_world
        .joint_reaction_force("gear_mesh10", dt_10rev)
        .unwrap();
    let rev_a_rf = gear10_world
        .joint_reaction_force("rev_a10", dt_10rev)
        .unwrap();
    assert!(
        (gear_rf.torque[2].abs() - 2.0).abs() < 1e-7,
        "Gear constraint reaction torque on pinion must equal 2.0 N*m, got {:.6}",
        gear_rf.torque[2]
    );
    assert!(
        (rev_a_rf.torque[2].abs() - 2.0).abs() < 1e-7,
        "Drive shaft motor torque on pinion must equal 2.0 N*m, got {:.6}",
        rev_a_rf.torque[2]
    );

    // =========================================================================
    // 4. Closed-Loop Slider-Crank / 4-Bar Mechanism + Dynamic Load Case Extraction
    // =========================================================================
    let mut crank_world = PhysicsWorld::with_gravity([0.0, -g, 0.0]);
    let crank_part = create_test_box("p4t7_crank", "Crank", 0.03, 0.01, 0.01);
    let rod_part = create_test_box("p4t7_rod", "Connecting Rod", 0.05, 0.01, 0.01);
    let piston_part = create_test_box("p4t7_piston", "Piston", 0.02, 0.02, 0.02);

    crank_world
        .add_part_with_options(
            &crank_part,
            true,
            ColliderType::ConvexHull,
            Some([0.015, 0.0, 0.0]),
            None,
        )
        .unwrap();
    crank_world
        .add_part_with_options(
            &rod_part,
            true,
            ColliderType::ConvexHull,
            Some([0.055, 0.0, 0.0]),
            None,
        )
        .unwrap();
    crank_world
        .add_part_with_options(
            &piston_part,
            true,
            ColliderType::ConvexHull,
            Some([0.08, 0.0, 0.0]),
            None,
        )
        .unwrap();

    crank_world
        .add_joint(
            "j_crank_ground",
            "ground",
            "p4t7_crank",
            CadJointType::Revolute {
                anchor_a: [0.0, 0.0, 0.0],
                anchor_b: [-0.015, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            Some(MotorConfig::new(4.0, 100.0)),
        )
        .unwrap();
    crank_world
        .add_joint(
            "j_crank_rod",
            "p4t7_crank",
            "p4t7_rod",
            CadJointType::Revolute {
                anchor_a: [0.015, 0.0, 0.0],
                anchor_b: [-0.025, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .unwrap();
    crank_world
        .add_joint(
            "j_rod_piston",
            "p4t7_rod",
            "p4t7_piston",
            CadJointType::Revolute {
                anchor_a: [0.025, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                limits: None,
            },
            None,
        )
        .unwrap();
    crank_world
        .add_joint(
            "j_piston_slide",
            "ground",
            "p4t7_piston",
            CadJointType::Prismatic {
                anchor_a: [0.08, 0.0, 0.0],
                anchor_b: [0.0, 0.0, 0.0],
                axis: [1.0, 0.0, 0.0],
                limits: Some([-0.10, 0.10]),
            },
            None,
        )
        .unwrap();

    let crank_sim = crank_world.simulate(0.4, 0.005);
    assert!(crank_sim.success);
    assert!(crank_sim.step_count >= 80);

    let crank_lc = crank_world
        .extract_part_load_case(&crank_sim, "p4t7_crank", LoadCaseSelector::AtTime(0.20))
        .unwrap();
    assert_eq!(crank_lc.joints.len(), 2);
    assert!(
        crank_lc.equilibrium_residual_force_norm < 5e-3,
        "Closed-loop crank force equilibrium residual must be < 5e-3 N, got {:.3e}",
        crank_lc.equilibrium_residual_force_norm
    );
    assert!(
        crank_lc.equilibrium_residual_torque_norm < 1e-3,
        "Closed-loop crank torque equilibrium residual must be < 1e-3 N*m, got {:.3e}",
        crank_lc.equilibrium_residual_torque_norm
    );
}



