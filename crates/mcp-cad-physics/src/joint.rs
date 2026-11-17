//! Kinematic and dynamic joints for CAD assemblies in double precision (`f64`).

use nalgebra::{DMatrix, Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use crate::error::PhysicsError;

/// Stable, copyable handle identifying a joint in [`crate::simulator::PhysicsWorld`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct JointHandle(pub usize);

/// Validated `f64` joint descriptor produced by [`CadJointType::build_generic_joint`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointData {
    /// Underlying CAD joint specification.
    pub joint_type: CadJointType,
    /// Unit primary axis $\hat{\mathbf{z}}$ in local frame of Body A (for axial joints).
    pub local_axis_a: [f64; 3],
    /// Unit primary axis $\hat{\mathbf{z}}$ in local frame of Body B (for `Gear` or axial joints).
    pub local_axis_b: [f64; 3],
    /// Orthonormal tangent basis vector $\hat{\mathbf{x}}$ perpendicular to `local_axis_a`.
    pub tangent_u: [f64; 3],
    /// Orthonormal tangent basis vector $\hat{\mathbf{y}} = \hat{\mathbf{z}} \times \hat{\mathbf{x}}$.
    pub tangent_v: [f64; 3],
    /// Local anchor on Body A ($\text{m}$).
    pub anchor_a: [f64; 3],
    /// Local anchor on Body B ($\text{m}$).
    pub anchor_b: [f64; 3],
    /// Optional travel limits `[min, max]` ($\text{rad}$ for `Revolute`, $\text{m}$ for `Prismatic`).
    pub limits: Option<[f64; 2]>,
    /// Optional active velocity motor configuration.
    pub motor: Option<MotorConfig>,
    /// Whether collision contacts between the two connected bodies are enabled.
    pub contacts_enabled: bool,
}

impl JointData {
    /// Returns `true` if collision contacts between the two connected bodies are enabled.
    #[inline]
    #[must_use]
    pub const fn contacts_enabled(&self) -> bool {
        self.contacts_enabled
    }

    /// Sets whether collision contacts between the two connected bodies are enabled.
    #[inline]
    pub fn set_contacts_enabled(&mut self, enabled: bool) {
        self.contacts_enabled = enabled;
    }
}

/// High-level CAD joint definitions in double precision (`f64`).
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
    /// Gear kinematic constraint transmitting rotation between two axes: $\omega_b \cdot \hat{\mathbf{z}}_b = -\text{ratio}\,(\omega_a \cdot \hat{\mathbf{z}}_a)$.
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
    #[must_use]
    pub fn new(target_velocity: f64, max_force: f64) -> Self {
        Self {
            target_velocity,
            max_force,
        }
    }
}

fn validate_motor_f64(motor: MotorConfig) -> Result<MotorConfig, PhysicsError> {
    if !motor.target_velocity.is_finite() || !motor.max_force.is_finite() {
        return Err(PhysicsError::InvalidJointConfiguration(format!(
            "Motor parameters must be finite numbers, got target_velocity={}, max_force={}",
            motor.target_velocity, motor.max_force
        )));
    }
    if motor.max_force < 0.0 {
        return Err(PhysicsError::InvalidJointConfiguration(format!(
            "Motor max_force must be non-negative, got {}",
            motor.max_force
        )));
    }
    Ok(motor)
}

impl CadJointType {
    /// Validates and builds an `f64` [`JointData`] descriptor configured with optional motor control.
    pub fn build_generic_joint(&self, motor: Option<MotorConfig>) -> Result<JointData, PhysicsError> {
        let validated_motor = match motor {
            Some(m) => Some(validate_motor_f64(m)?),
            None => None,
        };

        match self {
            CadJointType::Revolute {
                anchor_a,
                anchor_b,
                axis,
                limits,
            } => {
                if !axis[0].is_finite() || !axis[1].is_finite() || !axis[2].is_finite() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Revolute joint axis components must be finite".into(),
                    ));
                }
                let axis_vec = Vector3::new(axis[0], axis[1], axis[2]);
                let norm = axis_vec.norm();
                if norm < 1e-7 {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Revolute joint axis cannot be a zero vector".into(),
                    ));
                }
                let unit_axis = axis_vec / norm;
                let (u, v) = perpendicular_axes(&unit_axis);

                if let Some([min, max]) = limits {
                    if !min.is_finite() || !max.is_finite() {
                        return Err(PhysicsError::InvalidJointConfiguration(format!(
                            "Revolute joint limits [min, max] must be finite: [{min}, {max}]"
                        )));
                    }
                    if min > max {
                        return Err(PhysicsError::InvalidJointConfiguration(format!(
                            "Revolute joint limits [min, max] invalid: {min} > {max}"
                        )));
                    }
                }

                Ok(JointData {
                    joint_type: self.clone(),
                    local_axis_a: [unit_axis.x, unit_axis.y, unit_axis.z],
                    local_axis_b: [unit_axis.x, unit_axis.y, unit_axis.z],
                    tangent_u: [u.x, u.y, u.z],
                    tangent_v: [v.x, v.y, v.z],
                    anchor_a: *anchor_a,
                    anchor_b: *anchor_b,
                    limits: *limits,
                    motor: validated_motor,
                    contacts_enabled: false,
                })
            }
            CadJointType::Prismatic {
                anchor_a,
                anchor_b,
                axis,
                limits,
            } => {
                if !axis[0].is_finite() || !axis[1].is_finite() || !axis[2].is_finite() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Prismatic joint axis components must be finite".into(),
                    ));
                }
                let axis_vec = Vector3::new(axis[0], axis[1], axis[2]);
                let norm = axis_vec.norm();
                if norm < 1e-7 {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Prismatic joint axis cannot be a zero vector".into(),
                    ));
                }
                let unit_axis = axis_vec / norm;
                let (u, v) = perpendicular_axes(&unit_axis);

                if let Some([min, max]) = limits {
                    if !min.is_finite() || !max.is_finite() {
                        return Err(PhysicsError::InvalidJointConfiguration(format!(
                            "Prismatic joint limits [min, max] must be finite: [{min}, {max}]"
                        )));
                    }
                    if min > max {
                        return Err(PhysicsError::InvalidJointConfiguration(format!(
                            "Prismatic joint limits [min, max] invalid: {min} > {max}"
                        )));
                    }
                }

                Ok(JointData {
                    joint_type: self.clone(),
                    local_axis_a: [unit_axis.x, unit_axis.y, unit_axis.z],
                    local_axis_b: [unit_axis.x, unit_axis.y, unit_axis.z],
                    tangent_u: [u.x, u.y, u.z],
                    tangent_v: [v.x, v.y, v.z],
                    anchor_a: *anchor_a,
                    anchor_b: *anchor_b,
                    limits: *limits,
                    motor: validated_motor,
                    contacts_enabled: false,
                })
            }
            CadJointType::Spherical { anchor_a, anchor_b } => {
                if validated_motor.is_some() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Motors are not supported on Spherical joints".into(),
                    ));
                }

                Ok(JointData {
                    joint_type: self.clone(),
                    local_axis_a: [0.0, 0.0, 1.0],
                    local_axis_b: [0.0, 0.0, 1.0],
                    tangent_u: [1.0, 0.0, 0.0],
                    tangent_v: [0.0, 1.0, 0.0],
                    anchor_a: *anchor_a,
                    anchor_b: *anchor_b,
                    limits: None,
                    motor: None,
                    contacts_enabled: false,
                })
            }
            CadJointType::Fixed { anchor_a, anchor_b } => {
                if validated_motor.is_some() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Motors are not supported on Fixed joints".into(),
                    ));
                }

                Ok(JointData {
                    joint_type: self.clone(),
                    local_axis_a: [0.0, 0.0, 1.0],
                    local_axis_b: [0.0, 0.0, 1.0],
                    tangent_u: [1.0, 0.0, 0.0],
                    tangent_v: [0.0, 1.0, 0.0],
                    anchor_a: *anchor_a,
                    anchor_b: *anchor_b,
                    limits: None,
                    motor: None,
                    contacts_enabled: false,
                })
            }
            CadJointType::Gear {
                ratio,
                axis_a,
                axis_b,
            } => {
                if validated_motor.is_some() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Motors are not supported on Gear joints; apply motor to one of the connected Revolute joints".into(),
                    ));
                }

                if !ratio.is_finite() || ratio.abs() < 1e-9 {
                    return Err(PhysicsError::InvalidJointConfiguration(format!(
                        "Gear ratio must be a non-zero finite number, got {ratio}"
                    )));
                }

                let norm_a =
                    (axis_a[0] * axis_a[0] + axis_a[1] * axis_a[1] + axis_a[2] * axis_a[2]).sqrt();
                if norm_a < 1e-7 || !norm_a.is_finite() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Gear joint axis_a cannot be a zero vector".into(),
                    ));
                }
                let unit_a = Vector3::new(axis_a[0], axis_a[1], axis_a[2]) / norm_a;

                let norm_b =
                    (axis_b[0] * axis_b[0] + axis_b[1] * axis_b[1] + axis_b[2] * axis_b[2]).sqrt();
                if norm_b < 1e-7 || !norm_b.is_finite() {
                    return Err(PhysicsError::InvalidJointConfiguration(
                        "Gear joint axis_b cannot be a zero vector".into(),
                    ));
                }
                let unit_b = Vector3::new(axis_b[0], axis_b[1], axis_b[2]) / norm_b;
                let (u, v) = perpendicular_axes(&unit_a);

                Ok(JointData {
                    joint_type: self.clone(),
                    local_axis_a: [unit_a.x, unit_a.y, unit_a.z],
                    local_axis_b: [unit_b.x, unit_b.y, unit_b.z],
                    tangent_u: [u.x, u.y, u.z],
                    tangent_v: [v.x, v.y, v.z],
                    anchor_a: [0.0, 0.0, 0.0],
                    anchor_b: [0.0, 0.0, 0.0],
                    limits: None,
                    motor: None,
                    contacts_enabled: false,
                })
            }
        }
    }

    /// Applies a motor configuration to an existing [`JointData`] in `f64`.
    pub fn apply_motor(
        &self,
        joint: &mut JointData,
        motor: MotorConfig,
    ) -> Result<(), PhysicsError> {
        let m = validate_motor_f64(motor)?;
        match self {
            CadJointType::Revolute { .. } | CadJointType::Prismatic { .. } => {
                joint.motor = Some(m);
                Ok(())
            }
            CadJointType::Spherical { .. } | CadJointType::Fixed { .. } => {
                Err(PhysicsError::InvalidJointConfiguration(
                    "Cannot set motor on Spherical or Fixed joint".into(),
                ))
            }
            CadJointType::Gear { .. } => Err(PhysicsError::InvalidJointConfiguration(
                "Cannot set motor directly on Gear constraint; apply motor to one of the connected Revolute joints".into(),
            )),
        }
    }

    /// Disables motor actuation on an existing [`JointData`] in `f64`.
    pub fn disable_motor(&self, joint: &mut JointData) -> Result<(), PhysicsError> {
        match self {
            CadJointType::Revolute { .. } | CadJointType::Prismatic { .. } => {
                joint.motor = None;
                Ok(())
            }
            CadJointType::Spherical { .. }
            | CadJointType::Fixed { .. }
            | CadJointType::Gear { .. } => Ok(()),
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

/// Builds a right-handed orthonormal basis `(x_loc, y_loc, z_unit)` from a local axis vector.
pub(crate) fn local_orthonormal_basis(axis: [f64; 3], default_axis: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
    let local_axis = Vector3::from(axis);
    let norm = local_axis.norm();
    let u = if norm > 1e-12 {
        local_axis / norm
    } else {
        default_axis
    };
    let (x, y) = perpendicular_axes(&u);
    (x, y, u)
}

/// Computes two unit axes `(x, y)` orthogonal to unit vector `z` with `x × y = z`.
pub(crate) fn perpendicular_axes(z: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let v_ref = if z.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };
    let x = (v_ref - v_ref.dot(z) * z).normalize();
    let y = z.cross(&x).normalize();
    (x, y)
}

/// Unwraps an angle in `[-pi, pi]` by multiples of `2*pi` to lie closest to the midpoint of `limits`.
fn unwrap_angle_near_limits(theta: f64, limits: [f64; 2]) -> f64 {
    let mid = 0.5 * (limits[0] + limits[1]);
    let mut best = theta;
    let mut best_dist = (theta - mid).abs();
    for shift in [-std::f64::consts::TAU, std::f64::consts::TAU] {
        let cand = theta + shift;
        let dist = (cand - mid).abs();
        if dist < best_dist - 1e-12 {
            best = cand;
            best_dist = dist;
        }
    }
    best
}

/// Returns `true` when joint `limits` collapse to a zero-width travel interval `[c, c]`.
#[must_use]
pub fn is_zero_range_limits(limits: Option<[f64; 2]>) -> bool {
    matches!(limits, Some([min, max]) if (max - min).abs() <= 1e-9)
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

    /// Effective number of constraint rows enforced in the kinematic Jacobian, accounting for
    /// degenerate zero-range `limits: Some([c, c])` on `Revolute` / `Prismatic` joints (6 rows).
    #[must_use]
    pub fn effective_constraints_count(&self) -> usize {
        match self {
            Self::Revolute { limits, .. } | Self::Prismatic { limits, .. }
                if is_zero_range_limits(*limits) =>
            {
                6
            }
            _ => self.constraints_count(),
        }
    }

    /// Computes the current relative joint coordinate (`rad` for `Revolute`, `m` for `Prismatic`),
    /// or `None` for joints without a 1-DOF scalar coordinate (`Fixed`, `Spherical`, `Gear`).
    #[must_use]
    pub fn current_coordinate(
        &self,
        pos_a: Vector3<f64>,
        rot_a: Matrix3<f64>,
        pos_b: Vector3<f64>,
        rot_b: Matrix3<f64>,
    ) -> Option<f64> {
        match self {
            Self::Revolute { axis, limits, .. } => {
                let (x_loc, y_loc, _) =
                    local_orthonormal_basis(*axis, Vector3::new(0.0, 0.0, 1.0));
                let r_rel = rot_a.transpose() * rot_b;
                let rotated_x = r_rel * x_loc;
                let raw_theta = y_loc.dot(&rotated_x).atan2(x_loc.dot(&rotated_x));
                let theta = match limits {
                    Some(lim) => unwrap_angle_near_limits(raw_theta, *lim),
                    None => raw_theta,
                };
                Some(theta)
            }
            Self::Prismatic {
                anchor_a,
                anchor_b,
                axis,
                ..
            } => {
                let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::new(1.0, 0.0, 0.0));
                let z_a = rot_a * u_a;
                let p_a_w = pos_a + rot_a * Vector3::from(*anchor_a);
                let p_b_w = pos_b + rot_b * Vector3::from(*anchor_b);
                Some((p_b_w - p_a_w).dot(&z_a))
            }
            Self::Fixed { .. } | Self::Spherical { .. } | Self::Gear { .. } => None,
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
                limits,
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
                let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::new(0.0, 0.0, 1.0));
                let z_a = rot_a * u_a;
                let (x_a, y_a) = perpendicular_axes(&z_a);

                // Rows 3, 4: lock out-of-axis rotations
                // Row 3: x_a . (theta_b - theta_a) = 0
                add_vec_to_jacobian(j_mat, start_row + 3, col_a, true, &(-x_a));
                add_vec_to_jacobian(j_mat, start_row + 3, col_b, true, &x_a);

                // Row 4: y_a . (theta_b - theta_a) = 0
                add_vec_to_jacobian(j_mat, start_row + 4, col_a, true, &(-y_a));
                add_vec_to_jacobian(j_mat, start_row + 4, col_b, true, &y_a);

                if is_zero_range_limits(*limits) {
                    // Row 5: zero-width limits [c, c] bilaterally lock rotation about z_a
                    add_vec_to_jacobian(j_mat, start_row + 5, col_a, true, &(-z_a));
                    add_vec_to_jacobian(j_mat, start_row + 5, col_b, true, &z_a);
                    6
                } else {
                    5
                }
            }
            Self::Prismatic {
                anchor_b,
                axis,
                limits,
                ..
            } => {
                let (_, _, u_a) = local_orthonormal_basis(*axis, Vector3::new(1.0, 0.0, 0.0));
                let z_a = rot_a * u_a;
                let (x_a, y_a) = perpendicular_axes(&z_a);

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

                if is_zero_range_limits(*limits) {
                    // Row 5: zero-width limits [c, c] bilaterally lock translation along z_a
                    add_vec_to_jacobian(j_mat, start_row + 5, col_a, false, &(-z_a));
                    add_vec_to_jacobian(j_mat, start_row + 5, col_a, true, &(z_a.cross(&(r_b - pos_a))));
                    add_vec_to_jacobian(j_mat, start_row + 5, col_b, false, &z_a);
                    add_vec_to_jacobian(j_mat, start_row + 5, col_b, true, &(d_b.cross(&z_a)));
                    6
                } else {
                    5
                }
            }
            Self::Gear { ratio, axis_a, axis_b } => {
                let (_, _, u_a) = local_orthonormal_basis(*axis_a, Vector3::new(0.0, 0.0, 1.0));
                let z_a = rot_a * u_a;

                let (_, _, u_b) = local_orthonormal_basis(*axis_b, Vector3::new(0.0, 0.0, 1.0));
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

/// Diagnostic status for a `Revolute` or `Prismatic` joint at or beyond its travel `limits` (`P3-T15`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointLimitStatus {
    /// Joint identifier.
    pub joint_id: String,
    /// Joint type (`"revolute"` or `"prismatic"`).
    pub joint_type: String,
    /// Reached or exceeded boundary (`"min"`, `"max"`, `"below_min"`, `"above_max"`, or `"locked"`).
    pub boundary: String,
    /// Current joint coordinate (`rad` for `revolute`, `m` for `prismatic` internally).
    pub current_value: f64,
    /// Configured travel limits `[min, max]`.
    pub limits: [f64; 2],
    /// Unit of `current_value` and `limits` (`"rad"`/`"deg"` or `"m"`/`"mm"`).
    pub unit: String,
    /// True if a configured motor drives into the active limit stop (or `min == max`).
    pub motor_blocked: bool,
}

/// Kinematic summary of a mechanism degree-of-freedom and mobility analysis (`P3-T9`, `P3-T15`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MechanismAnalysisReport {
    /// Numerically computed degrees of freedom via SVD rank analysis of the constraint Jacobian: `DOF = 6(N - 1) - rank(J)`.
    pub dof_numeric: i32,
    /// Spatial mobility calculated via the Chebychev-Grübler-Kutzbach formula: `M = 6(N - 1) - sum(c_i)`.
    pub dof_grubler: i32,
    /// Count of redundant/overconstrained kinematic constraints: `max(0, dof_numeric - dof_grubler)`.
    pub redundant_constraints: i32,
    /// Sorted IDs of joints whose constraint rows are linearly dependent when `redundant_constraints > 0` (`P3-T15`).
    pub redundant_joint_ids: Vec<String>,
    /// Joints currently at or beyond their travel `limits` (`P3-T15`).
    pub joints_at_limits: Vec<JointLimitStatus>,
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
    /// Warnings regarding mechanism locking, under-actuation, active limit stops, or floating bodies.
    pub warnings: Vec<String>,
    /// Technical notes explaining Grübler formula, SVD tolerance, redundant joints, and constraints.
    pub notes: Vec<String>,
}
