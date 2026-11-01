//! Kinematic and dynamic joints for CAD assemblies.

use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};

use crate::error::PhysicsError;

/// High-level CAD joint definitions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CadJointType {
    /// 1-DOF rotational joint around a specified axis.
    Revolute {
        anchor_a: [f64; 3],
        anchor_b: [f64; 3],
        axis: [f64; 3],
        limits: Option<[f64; 2]>,
    },
    /// 1-DOF linear sliding joint along a specified axis.
    Prismatic {
        anchor_a: [f64; 3],
        anchor_b: [f64; 3],
        axis: [f64; 3],
        limits: Option<[f64; 2]>,
    },
    /// 3-DOF spherical (ball-and-socket) joint.
    Spherical {
        anchor_a: [f64; 3],
        anchor_b: [f64; 3],
    },
    /// 0-DOF rigid lock joint.
    Fixed {
        anchor_a: [f64; 3],
        anchor_b: [f64; 3],
    },
    /// Gear kinematic constraint transmitting rotation between two axes: w_b = -ratio * w_a.
    Gear {
        ratio: f64,
        axis_a: [f64; 3],
        axis_b: [f64; 3],
    },
}

/// Motor configuration for actuated kinematic joints.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MotorConfig {
    /// Target velocity in rad/s (for revolute) or m/s (for prismatic).
    pub target_velocity: f64,
    /// Maximum actuation force (N) or torque (N*m).
    pub max_force: f64,
}

impl MotorConfig {
    /// Creates a new motor configuration.
    pub fn new(target_velocity: f64, max_force: f64) -> Self {
        Self {
            target_velocity,
            max_force,
        }
    }
}

impl CadJointType {
    /// Builds a Rapier [`GenericJoint`] configured with optional motor control.
    pub fn build_generic_joint(&self, motor: Option<MotorConfig>) -> Result<GenericJoint, PhysicsError> {
        match self {
            CadJointType::Revolute {
                anchor_a,
                anchor_b,
                axis,
                limits,
            } => {
                let axis_vec = vector![axis[0] as Real, axis[1] as Real, axis[2] as Real];
                let norm = axis_vec.norm();
                if norm < 1e-7 {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Revolute joint axis cannot be a zero vector".into(),
                    ));
                }
                let unit_axis = UnitVector::new_normalize(axis_vec);

                let mut builder = RevoluteJointBuilder::new(unit_axis)
                    .local_anchor1(point![
                        anchor_a[0] as Real,
                        anchor_a[1] as Real,
                        anchor_a[2] as Real
                    ])
                    .local_anchor2(point![
                        anchor_b[0] as Real,
                        anchor_b[1] as Real,
                        anchor_b[2] as Real
                    ]);

                if let Some([min, max]) = limits {
                    if min > max {
                        return Err(PhysicsError::InvalidJointConfiguration(format!(
                            "Revolute joint limits [min, max] invalid: {min} > {max}"
                        )));
                    }
                    builder = builder.limits([*min as Real, *max as Real]);
                }

                if let Some(m) = motor {
                    builder = builder.motor_velocity(m.target_velocity as Real, m.max_force as Real);
                }

                let mut generic: GenericJoint = builder.build().into();
                generic.set_contacts_enabled(false);
                Ok(generic)
            }
            CadJointType::Prismatic {
                anchor_a,
                anchor_b,
                axis,
                limits,
            } => {
                let axis_vec = vector![axis[0] as Real, axis[1] as Real, axis[2] as Real];
                let norm = axis_vec.norm();
                if norm < 1e-7 {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Prismatic joint axis cannot be a zero vector".into(),
                    ));
                }
                let unit_axis = UnitVector::new_normalize(axis_vec);

                let mut builder = PrismaticJointBuilder::new(unit_axis)
                    .local_anchor1(point![
                        anchor_a[0] as Real,
                        anchor_a[1] as Real,
                        anchor_a[2] as Real
                    ])
                    .local_anchor2(point![
                        anchor_b[0] as Real,
                        anchor_b[1] as Real,
                        anchor_b[2] as Real
                    ]);

                if let Some([min, max]) = limits {
                    if min > max {
                        return Err(PhysicsError::InvalidJointConfiguration(format!(
                            "Prismatic joint limits [min, max] invalid: {min} > {max}"
                        )));
                    }
                    builder = builder.limits([*min as Real, *max as Real]);
                }

                if let Some(m) = motor {
                    builder = builder.motor_velocity(m.target_velocity as Real, m.max_force as Real);
                }

                let mut generic: GenericJoint = builder.build().into();
                generic.set_contacts_enabled(false);
                Ok(generic)
            }
            CadJointType::Spherical { anchor_a, anchor_b } => {
                if motor.is_some() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Motors are not supported on Spherical joints".into(),
                    ));
                }

                let builder = SphericalJointBuilder::new()
                    .local_anchor1(point![
                        anchor_a[0] as Real,
                        anchor_a[1] as Real,
                        anchor_a[2] as Real
                    ])
                    .local_anchor2(point![
                        anchor_b[0] as Real,
                        anchor_b[1] as Real,
                        anchor_b[2] as Real
                    ]);

                let mut generic: GenericJoint = builder.build().into();
                generic.set_contacts_enabled(false);
                Ok(generic)
            }
            CadJointType::Fixed { anchor_a, anchor_b } => {
                if motor.is_some() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Motors are not supported on Fixed joints".into(),
                    ));
                }

                let builder = FixedJointBuilder::new()
                    .local_anchor1(point![
                        anchor_a[0] as Real,
                        anchor_a[1] as Real,
                        anchor_a[2] as Real
                    ])
                    .local_anchor2(point![
                        anchor_b[0] as Real,
                        anchor_b[1] as Real,
                        anchor_b[2] as Real
                    ]);

                let mut generic: GenericJoint = builder.build().into();
                generic.set_contacts_enabled(false);
                Ok(generic)
            }
            CadJointType::Gear { .. } => {
                let mut generic: GenericJoint = GenericJointBuilder::new(JointAxesMask::empty()).build();
                generic.set_contacts_enabled(false);
                Ok(generic)
            }
        }
    }

    /// Applies a motor configuration to an existing Rapier [`GenericJoint`].
    pub fn apply_motor(
        &self,
        joint: &mut GenericJoint,
        motor: MotorConfig,
    ) -> Result<(), PhysicsError> {
        match self {
            CadJointType::Revolute { .. } => {
                joint.set_motor_velocity(
                    JointAxis::AngX,
                    motor.target_velocity as Real,
                    motor.max_force as Real,
                );
                Ok(())
            }
            CadJointType::Prismatic { .. } => {
                joint.set_motor_velocity(
                    JointAxis::LinX,
                    motor.target_velocity as Real,
                    motor.max_force as Real,
                );
                Ok(())
            }
            CadJointType::Spherical { .. } | CadJointType::Fixed { .. } => {
                Err(PhysicsError::InvalidJointConfiguration(
                    "Cannot set motor on Spherical or Fixed joint".into(),
                ))
            }
            CadJointType::Gear { .. } => {
                Err(PhysicsError::InvalidJointConfiguration(
                    "Cannot set motor directly on Gear constraint; apply motor to one of the connected Revolute joints".into(),
                ))
            }
        }
    }

    /// Disables motor actuation on an existing Rapier [`GenericJoint`].
    pub fn disable_motor(&self, joint: &mut GenericJoint) -> Result<(), PhysicsError> {
        match self {
            CadJointType::Revolute { .. } => {
                joint.set_motor_velocity(JointAxis::AngX, 0.0, 0.0);
                joint.set_motor_max_force(JointAxis::AngX, 0.0);
                Ok(())
            }
            CadJointType::Prismatic { .. } => {
                joint.set_motor_velocity(JointAxis::LinX, 0.0, 0.0);
                joint.set_motor_max_force(JointAxis::LinX, 0.0);
                Ok(())
            }
            CadJointType::Spherical { .. } | CadJointType::Fixed { .. } | CadJointType::Gear { .. } => Ok(()),
        }
    }
}
