//! Physics simulation crate for MCP-CAD in double precision (`f64`).
#![deny(unsafe_code)]

pub mod collision;
pub mod dae_prototype;
pub mod error;
pub mod joint;
pub mod rigid_body;
pub mod simulator;
pub mod solver;
pub mod telemetry;

pub use collision::{
    hertz_cylinder_contact, hertz_sphere_contact, ContactPoint, DeformableSurfaceIndex,
    HertzCylinderSolution, HertzSphereSolution, SurfaceProjection, TriMeshBvh,
};
pub use error::PhysicsError;
pub use joint::{
    CadJointType, JointData, JointHandle, JointLimitStatus, MechanismAnalysisReport, MotorConfig,
    quat_to_matrix,
};
pub use rigid_body::{
    BodyHandle, BodyRegistry, Collider, ColliderHandle, ColliderRegistry, ColliderShape,
    ColliderType, ConvexHullPart, MassProperties, RigidBody, compute_mesh_mass_properties,
    create_collider, create_collider_builder, create_rigid_body, create_rigid_body_builder,
};
pub use simulator::{
    CadJointRecord, CoupledFragmentPose, FragmentSpawnSpec, PhysicsWorld, SystemMomentumSummary,
};
pub use solver::{
    extract_gear_joint_couplings, extract_joint_hub_couplings, sync_deformed_trimesh_collider,
    GearJointCouplingSpec, HubJointKind, JointHubCouplingSpec, MultiBodySolver, SolvedJointRecord,
    SolverConfig,
};
pub use telemetry::{
    BodyState, BodyTelemetry, CollisionEventType, CollisionRecord, InterferenceReport,
    JointLoadCase, JointReactionForce, JointSweepReport, LoadCaseSelector, PairInterferenceResult,
    PartInertialLoad, PartLoadCase, SimulationSummary, SweepStepResult,
};

