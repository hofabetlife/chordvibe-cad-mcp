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

        // 1. Reaction force F_y ≈ +m*g supporting the beam against -Y gravity (within 1%)
        let force_rel_err = (j_beam.force[1] - expected_force_y).abs() / expected_force_y;
        assert!(
            force_rel_err < 0.01,
            "Selector {:?}: expected F_y ≈ {:.4} N (m*g), got F = {:?} (rel_err = {:.4})",
            selector,
            expected_force_y,
            j_beam.force,
            force_rel_err
        );
        assert!(j_beam.force[0].abs() < 1e-3 * expected_force_y);
        assert!(j_beam.force[2].abs() < 1e-3 * expected_force_y);

        // 2. Reaction torque at root anchor [0,0,0]: T_z ≈ +m*g*L/2 counteracting gravity moment (within 1%)
        let torque_rel_err = (j_beam.torque[2] - expected_torque_z).abs() / expected_torque_z;
        assert!(
            torque_rel_err < 0.01,
            "Selector {:?}: expected T_z ≈ {:.5} N*m (m*g*L/2), got T = {:?} (rel_err = {:.4})",
            selector,
            expected_torque_z,
            j_beam.torque,
            torque_rel_err
        );

        // 3. Dynamic equilibrium residual must be small (< 1% of m*g)
        assert!(
            beam_lc.equilibrium_residual_force_norm < 0.01 * expected_force_y,
            "Selector {:?}: expected small force equilibrium residual, got {:.6} N",
            selector,
            beam_lc.equilibrium_residual_force_norm
        );
        assert!(
            beam_lc.equilibrium_residual_torque_norm < 0.01 * expected_torque_z,
            "Selector {:?}: expected small torque equilibrium residual, got {:.6} N*m",
            selector,
            beam_lc.equilibrium_residual_torque_norm
        );
    }

    // 4. Verify Newton's 3rd Law between `wall_support` (`part_a`) and `cantilever_beam` (`part_b`)
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
            (jb.force_world[axis] + jw.force_world[axis]).abs() < 1e-9,
            "Newton 3rd law violated for force axis {axis}: beam={:?}, wall={:?}",
            jb.force_world,
            jw.force_world
        );
        assert!(
            (jb.torque_world[axis] + jw.torque_world[axis]).abs() < 1e-9,
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

    // 1. Magnitude of reaction force F ≈ m * omega^2 * r_c (within 2% discrete integrator tolerance)
    let f_mag_rel_err =
        (j_rod.force_magnitude - expected_centripetal_force).abs() / expected_centripetal_force;
    assert!(
        f_mag_rel_err < 0.02,
        "Expected centripetal force magnitude ≈ {:.4} N (m*omega^2*r_c), got {:.4} N (rel_err = {:.4})",
        expected_centripetal_force,
        j_rod.force_magnitude,
        f_mag_rel_err
    );

    // 2. In the rod's local frame, the pivot pulls the rod inward along -X (F_x ≈ -m * omega^2 * r_c)
    let fx_rel_err =
        (j_rod.force[0] - (-expected_centripetal_force)).abs() / expected_centripetal_force;
    assert!(
        fx_rel_err < 0.02,
        "Expected local F_x ≈ {:.4} N, got local force = {:?} (world force = {:?})",
        -expected_centripetal_force,
        j_rod.force,
        j_rod.force_world
    );
    assert!(
        j_rod.force[1].abs() < 0.02 * expected_centripetal_force,
        "Expected negligible local tangential force F_y, got {:?}",
        j_rod.force
    );

    // 3. Free revolute axis Z transmits zero reaction torque
    assert!(
        j_rod.torque_magnitude < 1e-6,
        "Expected zero reaction torque on frictionless revolute joint in planar motion, got {:?}",
        j_rod.torque
    );

    // 4. Local COM centripetal acceleration a_x ≈ -omega^2 * r_c and small equilibrium residual
    let expected_ax = -omega * omega * r_c;
    let ax_rel_err = (rod_lc.inertia.linear_acceleration[0] - expected_ax).abs() / expected_ax.abs();
    assert!(
        ax_rel_err < 0.02,
        "Expected local centripetal acceleration a_x ≈ {:.4} m/s^2, got {:?}",
        expected_ax,
        rod_lc.inertia.linear_acceleration
    );
    assert!(
        rod_lc.equilibrium_residual_force_norm < 0.02 * expected_centripetal_force,
        "Expected small dynamic force equilibrium residual, got {:.6} N (vec = {:?})",
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
    assert!(sweep.min_clearance.expect("min_clearance") > 0.0);
    assert_eq!(sweep.min_clearance_step, Some(27));
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

    let report = world
        .check_interference(None, false, None)
        .expect("benchmark 10 parts");
    assert_eq!(report.candidate_pairs_count, 45);
    assert_eq!(report.evaluated_pairs_count, 45);
    assert_eq!(report.interfering_pairs_count, 1);
    assert!(
        report.elapsed_ms < 250.0,
        "10-part (45-pair) interference check took {:.3} ms (expected < 250 ms)",
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


