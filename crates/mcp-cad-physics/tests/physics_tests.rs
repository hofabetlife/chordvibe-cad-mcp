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
        "update_part_collider must preserve the existing RigidBodyHandle"
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
