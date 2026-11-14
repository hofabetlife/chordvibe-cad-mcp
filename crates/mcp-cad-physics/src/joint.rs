//! Kinematic and dynamic joints for CAD assemblies.

use nalgebra::{DMatrix, Matrix3, Vector3};
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
                    builder = builder
                        .motor_velocity(m.target_velocity as Real, m.max_force as Real)
                        .motor_max_force(m.max_force as Real);
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
                    builder = builder
                        .motor_velocity(m.target_velocity as Real, m.max_force as Real)
                        .motor_max_force(m.max_force as Real);
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
            CadJointType::Gear { ratio, axis_a, axis_b } => {
                if motor.is_some() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Motors are not supported on Gear joints; apply motor to one of the connected Revolute joints".into(),
                    ));
                }

                if !ratio.is_finite() || ratio.abs() < 1e-9 {
                    return Err(PhysicsError::InvalidJointConfiguration(format!(
                        "Gear ratio must be a non-zero finite number, got {ratio}"
                    )));
                }

                let norm_a = (axis_a[0] * axis_a[0] + axis_a[1] * axis_a[1] + axis_a[2] * axis_a[2]).sqrt();
                if norm_a < 1e-7 || !norm_a.is_finite() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Gear joint axis_a cannot be a zero vector".into(),
                    ));
                }

                let norm_b = (axis_b[0] * axis_b[0] + axis_b[1] * axis_b[1] + axis_b[2] * axis_b[2]).sqrt();
                if norm_b < 1e-7 || !norm_b.is_finite() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Gear joint axis_b cannot be a zero vector".into(),
                    ));
                }

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
                joint.set_motor_max_force(JointAxis::AngX, motor.max_force as Real);
                Ok(())
            }
            CadJointType::Prismatic { .. } => {
                joint.set_motor_velocity(
                    JointAxis::LinX,
                    motor.target_velocity as Real,
                    motor.max_force as Real,
                );
                joint.set_motor_max_force(JointAxis::LinX, motor.max_force as Real);
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

    /// Returns the joint anchor in `part_a`'s local frame (meters).
    #[must_use]
    pub const fn anchor_a(&self) -> [f64; 3] {
        match self {
            Self::Fixed { anchor_a, .. }
            | Self::Revolute { anchor_a, .. }
            | Self::Prismatic { anchor_a, .. }
            | Self::Spherical { anchor_a, .. } => *anchor_a,
            Self::Gear { .. } => [0.0, 0.0, 0.0],
        }
    }

    /// Returns the joint anchor in `part_b`'s local frame (meters).
    #[must_use]
    pub const fn anchor_b(&self) -> [f64; 3] {
        match self {
            Self::Fixed { anchor_b, .. }
            | Self::Revolute { anchor_b, .. }
            | Self::Prismatic { anchor_b, .. }
            | Self::Spherical { anchor_b, .. } => *anchor_b,
            Self::Gear { .. } => [0.0, 0.0, 0.0],
        }
    }
}

/// Helper to add a 3D perturbation vector into the kinematic Jacobian matrix for a body.
fn add_vec_to_jacobian(
    j_mat: &mut DMatrix<f64>,
    r: usize,
    col_idx: Option<usize>,
    is_rotation: bool,
    delta: &Vector3<f64>,
) {
    if let Some(idx) = col_idx {
        let base = 6 * idx + if is_rotation { 3 } else { 0 };
        j_mat[(r, base)] += delta.x;
        j_mat[(r, base + 1)] += delta.y;
        j_mat[(r, base + 2)] += delta.z;
    }
}

impl CadJointType {
    /// Canonical lowercase string identifier for the joint type.
    #[must_use]
    pub const fn type_name(&self) -> &'static str {
        match self {
            Self::Fixed { .. } => "fixed",
            Self::Revolute { .. } => "revolute",
            Self::Prismatic { .. } => "prismatic",
            Self::Spherical { .. } => "spherical",
            Self::Gear { .. } => "gear",
        }
    }

    /// Number of spatial kinematic constraint equations enforced by this joint:
    /// - `Fixed`: 6 constraints
    /// - `Revolute`: 5 constraints
    /// - `Prismatic`: 5 constraints
    /// - `Spherical`: 3 constraints
    /// - `Gear`: 1 constraint
    #[must_use]
    pub const fn constraints_count(&self) -> usize {
        match self {
            Self::Fixed { .. } => 6,
            Self::Revolute { .. } => 5,
            Self::Prismatic { .. } => 5,
            Self::Spherical { .. } => 3,
            Self::Gear { .. } => 1,
        }
    }

    /// Populates rows of the kinematic constraint Jacobian matrix `j_mat` for this joint.
    /// Returns the number of constraint rows added.
    #[allow(clippy::too_many_arguments)]
    pub fn fill_jacobian_rows(
        &self,
        j_mat: &mut DMatrix<f64>,
        start_row: usize,
        col_a: Option<usize>,
        col_b: Option<usize>,
        pos_a: Vector3<f64>,
        rot_a: Matrix3<f64>,
        pos_b: Vector3<f64>,
        rot_b: Matrix3<f64>,
    ) -> usize {
        match self {
            Self::Fixed { anchor_a, anchor_b } => {
                let d_a = rot_a * Vector3::from(*anchor_a);
                let d_b = rot_b * Vector3::from(*anchor_b);
                let e = [
                    Vector3::new(1.0, 0.0, 0.0),
                    Vector3::new(0.0, 1.0, 0.0),
                    Vector3::new(0.0, 0.0, 1.0),
                ];
                // Rows 0..3: position coincidence (r_a - r_b = 0)
                for (k, ek) in e.iter().enumerate() {
                    let r = start_row + k;
                    add_vec_to_jacobian(j_mat, r, col_a, false, ek);
                    add_vec_to_jacobian(j_mat, r, col_a, true, &(d_a.cross(ek)));
                    add_vec_to_jacobian(j_mat, r, col_b, false, &(-ek));
                    add_vec_to_jacobian(j_mat, r, col_b, true, &(-d_b.cross(ek)));
                }
                // Rows 3..6: relative rotation locked (theta_b - theta_a = 0)
                for (k, ek) in e.iter().enumerate() {
                    let r = start_row + 3 + k;
                    add_vec_to_jacobian(j_mat, r, col_a, true, &(-ek));
                    add_vec_to_jacobian(j_mat, r, col_b, true, ek);
                }
                6
            }
            Self::Spherical { anchor_a, anchor_b } => {
                let d_a = rot_a * Vector3::from(*anchor_a);
                let d_b = rot_b * Vector3::from(*anchor_b);
                let e = [
                    Vector3::new(1.0, 0.0, 0.0),
                    Vector3::new(0.0, 1.0, 0.0),
                    Vector3::new(0.0, 0.0, 1.0),
                ];
                // Rows 0..3: position coincidence (r_a - r_b = 0)
                for (k, ek) in e.iter().enumerate() {
                    let r = start_row + k;
                    add_vec_to_jacobian(j_mat, r, col_a, false, ek);
                    add_vec_to_jacobian(j_mat, r, col_a, true, &(d_a.cross(ek)));
                    add_vec_to_jacobian(j_mat, r, col_b, false, &(-ek));
                    add_vec_to_jacobian(j_mat, r, col_b, true, &(-d_b.cross(ek)));
                }
                3
            }
            Self::Revolute {
                anchor_a,
                anchor_b,
                axis,
                ..
            } => {
                let d_a = rot_a * Vector3::from(*anchor_a);
                let d_b = rot_b * Vector3::from(*anchor_b);
                let e = [
                    Vector3::new(1.0, 0.0, 0.0),
                    Vector3::new(0.0, 1.0, 0.0),
                    Vector3::new(0.0, 0.0, 1.0),
                ];
                // Rows 0..3: position coincidence (r_a - r_b = 0)
                for (k, ek) in e.iter().enumerate() {
                    let r = start_row + k;
                    add_vec_to_jacobian(j_mat, r, col_a, false, ek);
                    add_vec_to_jacobian(j_mat, r, col_a, true, &(d_a.cross(ek)));
                    add_vec_to_jacobian(j_mat, r, col_b, false, &(-ek));
                    add_vec_to_jacobian(j_mat, r, col_b, true, &(-d_b.cross(ek)));
                }
                // Construct orthonormal axes perpendicular to hinge axis in world space
                let local_axis = Vector3::from(*axis);
                let norm = local_axis.norm();
                let u_a = if norm > 1e-12 {
                    local_axis / norm
                } else {
                    Vector3::new(0.0, 0.0, 1.0)
                };
                let z_a = rot_a * u_a;
                let v_ref = if z_a.x.abs() < 0.9 {
                    Vector3::new(1.0, 0.0, 0.0)
                } else {
                    Vector3::new(0.0, 1.0, 0.0)
                };
                let x_a = (v_ref - v_ref.dot(&z_a) * z_a).normalize();
                let y_a = z_a.cross(&x_a).normalize();

                // Rows 3, 4: lock out-of-axis rotations
                // Row 3: x_a . (theta_b - theta_a) = 0
                add_vec_to_jacobian(j_mat, start_row + 3, col_a, true, &(-x_a));
                add_vec_to_jacobian(j_mat, start_row + 3, col_b, true, &x_a);

                // Row 4: y_a . (theta_b - theta_a) = 0
                add_vec_to_jacobian(j_mat, start_row + 4, col_a, true, &(-y_a));
                add_vec_to_jacobian(j_mat, start_row + 4, col_b, true, &y_a);

                5
            }
            Self::Prismatic {
                anchor_b,
                axis,
                ..
            } => {
                let local_axis = Vector3::from(*axis);
                let norm = local_axis.norm();
                let u_a = if norm > 1e-12 {
                    local_axis / norm
                } else {
                    Vector3::new(0.0, 0.0, 1.0)
                };
                let z_a = rot_a * u_a;
                let v_ref = if z_a.x.abs() < 0.9 {
                    Vector3::new(1.0, 0.0, 0.0)
                } else {
                    Vector3::new(0.0, 1.0, 0.0)
                };
                let x_a = (v_ref - v_ref.dot(&z_a) * z_a).normalize();
                let y_a = z_a.cross(&x_a).normalize();

                let e = [
                    Vector3::new(1.0, 0.0, 0.0),
                    Vector3::new(0.0, 1.0, 0.0),
                    Vector3::new(0.0, 0.0, 1.0),
                ];
                // Rows 0..3: all 3 relative rotations locked (theta_b - theta_a = 0)
                for (k, ek) in e.iter().enumerate() {
                    let r = start_row + k;
                    add_vec_to_jacobian(j_mat, r, col_a, true, &(-ek));
                    add_vec_to_jacobian(j_mat, r, col_b, true, ek);
                }

                // Rows 3, 4: translation perpendicular to axis z_a is locked
                let r_b = pos_b + rot_b * Vector3::from(*anchor_b);
                let d_b = r_b - pos_b;

                // Row 3: x_a . (r_b - r_a) = 0
                add_vec_to_jacobian(j_mat, start_row + 3, col_a, false, &(-x_a));
                add_vec_to_jacobian(j_mat, start_row + 3, col_a, true, &(x_a.cross(&(r_b - pos_a))));
                add_vec_to_jacobian(j_mat, start_row + 3, col_b, false, &x_a);
                add_vec_to_jacobian(j_mat, start_row + 3, col_b, true, &(d_b.cross(&x_a)));

                // Row 4: y_a . (r_b - r_a) = 0
                add_vec_to_jacobian(j_mat, start_row + 4, col_a, false, &(-y_a));
                add_vec_to_jacobian(j_mat, start_row + 4, col_a, true, &(y_a.cross(&(r_b - pos_a))));
                add_vec_to_jacobian(j_mat, start_row + 4, col_b, false, &y_a);
                add_vec_to_jacobian(j_mat, start_row + 4, col_b, true, &(d_b.cross(&y_a)));

                5
            }
            Self::Gear { ratio, axis_a, axis_b } => {
                let local_a = Vector3::from(*axis_a);
                let norm_a = local_a.norm();
                let u_a = if norm_a > 1e-12 {
                    local_a / norm_a
                } else {
                    Vector3::new(0.0, 0.0, 1.0)
                };
                let z_a = rot_a * u_a;

                let local_b = Vector3::from(*axis_b);
                let norm_b = local_b.norm();
                let u_b = if norm_b > 1e-12 {
                    local_b / norm_b
                } else {
                    Vector3::new(0.0, 0.0, 1.0)
                };
                let z_b = rot_b * u_b;

                // Row 0: ratio * (theta_a . z_a) + 1.0 * (theta_b . z_b) = 0
                add_vec_to_jacobian(j_mat, start_row, col_a, true, &(z_a * *ratio));
                add_vec_to_jacobian(j_mat, start_row, col_b, true, &z_b);

                1
            }
        }
    }
}

/// Converts a unit quaternion `[x, y, z, w]` to a 3x3 rotation matrix.
#[must_use]
pub fn quat_to_matrix(q: [f64; 4]) -> Matrix3<f64> {
    let (x, y, z, w) = (q[0], q[1], q[2], q[3]);
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    let (x, y, z, w) = if norm > 1e-12 {
        (x / norm, y / norm, z / norm, w / norm)
    } else {
        (0.0, 0.0, 0.0, 1.0)
    };
    Matrix3::new(
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
        2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
        2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y),
    )
}

/// Kinematic summary of a mechanism degree-of-freedom and mobility analysis (`P3-T9`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MechanismAnalysisReport {
    /// Numerically computed degrees of freedom via SVD rank analysis of the constraint Jacobian: `DOF = 6(N - 1) - rank(J)`.
    pub dof_numeric: i32,
    /// Spatial mobility calculated via the Chebychev-Grübler-Kutzbach formula: `M = 6(N - 1) - sum(c_i)`.
    pub dof_grubler: i32,
    /// Count of redundant/overconstrained kinematic constraints: `max(0, dof_numeric - dof_grubler)`.
    pub redundant_constraints: i32,
    /// List of grounded/fixed parts acting as the inertial reference frame.
    pub grounded_parts: Vec<String>,
    /// List of active moving bodies.
    pub moving_parts: Vec<String>,
    /// Moving parts not connected to ground through any kinematic joint chain.
    pub floating_parts: Vec<String>,
    /// Parts in scene having zero kinematic joints attached.
    pub unconnected_parts: Vec<String>,
    /// Total number of joint constraints rows in the kinematic Jacobian.
    pub joint_constraints_count: usize,
    /// Number of rows in the constraint Jacobian matrix.
    pub jacobian_rows: usize,
    /// Number of columns in the constraint Jacobian matrix (6 per moving body).
    pub jacobian_cols: usize,
    /// Numerical rank of the constraint Jacobian matrix determined via SVD.
    pub jacobian_rank: usize,
    /// Singular values of the constraint Jacobian matrix in descending order.
    pub singular_values: Vec<f64>,
    /// Effective singular value cutoff threshold used for numerical rank determination.
    pub rank_tolerance: f64,
    /// Warnings regarding mechanism locking, under-actuation, or floating bodies.
    pub warnings: Vec<String>,
    /// Technical notes explaining Grübler formula, SVD tolerance, and constraints.
    pub notes: Vec<String>,
}
