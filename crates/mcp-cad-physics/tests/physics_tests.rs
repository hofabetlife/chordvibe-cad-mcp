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
