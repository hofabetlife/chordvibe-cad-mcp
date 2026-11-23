//! Implicit Dynamic Time Integrator (`HHT-α`), Consistent/Lumped Mass Matrices,
//! Adaptive Time-Step Bisection, and Multi-Body Coupled Contact + Fracture Dynamics (`P5-T6`).
//!
//! Implements the unconditionally stable Hilber–Hughes–Taylor $\alpha$-method (`f64`):
//! - Parameters: $\alpha \in [-0.3, 0.0]$, $\beta = \frac{(1-\alpha)^2}{4}$, $\gamma = \frac{1-2\alpha}{2}$.
//! - Dynamic equilibrium on free DOFs:
//!   $$\mathbf{M}\mathbf{a}_{n+1} + (1+\alpha)\mathbf{C}\mathbf{v}_{n+1} - \alpha\mathbf{C}\mathbf{v}_n + (1+\alpha)\mathbf{f}_{\text{int}}(\mathbf{u}_{n+1}) - \alpha\mathbf{f}_{\text{int}}(\mathbf{u}_n) = (1+\alpha)\mathbf{f}_{\text{ext}}(t_{n+1}) - \alpha\mathbf{f}_{\text{ext}}(t_n)$$
//! - Effective Newton tangent:
//!   $$\mathbf{K}_{\text{eff}} = \frac{1}{\beta \Delta t^2}\mathbf{M} + \frac{(1+\alpha)\gamma}{\beta \Delta t}\mathbf{C} + (1+\alpha)\mathbf{K}_T(\mathbf{u}_{n+1})$$
//! - Exact consistent and strictly positive lumped mass matrices (`TET4` analytical; `TET10` exact
//!   quadratic consistent + HRZ positive lumping).
//! - Parallel `rayon` element evaluation, RCM sparse direct $LDL^T$ / `IC(0)-PCG` linear solves,
//!   adaptive step bisection on Newton non-convergence / plasticity / contact / fracture events,
//!   real-time progress callbacks, and lock-free `AtomicBool` cancellation.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

use nalgebra::{Matrix3, Vector3, Vector6};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::boundary::{BoundaryConditions, HubNodeSelector, ResolvedBoundary};
use crate::contact::{AnalyticalRigidSurface, ContactPairSpec, DeformedSurfaceMesh};
use crate::damage::{
    effective_stiffness_factor, evaluate_element_damage, DamageMaterialParams, ElementDamageState,
};
use crate::elasticity::{von_mises_from_voigt, GaussPointState, J2MaterialParams};
use crate::element::{rotate_voigt_stress, Tet10Element, Tet4Element, Vector12, Vector30};
use crate::error::FeaError;
use crate::fracture::{partition_fractured_body, DetachedFragment};
use crate::solver::{CsrMatrix, SolverOptions};
use crate::sparse_solver::{solve_linear_robust, solve_spd_newton_robust, LinearSolverKind};
use crate::tetra_mesh::TetrahedralMesh;

/// Minimum element count before switching element evaluation loops to `rayon` multi-threading.
const DYNAMICS_PAR_THRESHOLD: usize = 64;

// ============================================================================
// Public Configuration & Telemetry Data Structures
// ============================================================================

/// Finite-element mass matrix formulation for dynamic integration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum MassMatrixKind {
    /// Full consistent mass matrix $\mathbf{M}^e = \int_{\Omega_e} \rho \mathbf{N}^T \mathbf{N} \, dV$.
    #[default]
    Consistent,
    /// Strictly positive diagonal lumped mass matrix (exact $1/4$ for `TET4`; HRZ lumping
    /// $1/36$ corner, $4/27$ mid-edge for `TET10`).
    Lumped,
}

/// Time modulation curve scaling the static `BoundaryConditions` nodal forces over time.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum TimeForceCurve {
    /// Constant scale factor over the full simulation (default `1.0`).
    Constant(f64),
    /// Linear ramp from `0.0` at $t = 0$ to `1.0` at $t \ge \text{duration\_s}$.
    Ramp { duration_s: f64 },
    /// Rectangular pulse of amplitude `amplitude` active for `start_s <= t <= end_s`.
    Pulse {
        start_s: f64,
        end_s: f64,
        amplitude: f64,
    },
    /// Smooth sinusoidal excitation `amplitude * sin(2 * pi * frequency_hz * t + phase_rad)`.
    Sine {
        frequency_hz: f64,
        amplitude: f64,
        phase_rad: f64,
    },
}

impl Default for TimeForceCurve {
    fn default() -> Self {
        Self::Constant(1.0)
    }
}

impl TimeForceCurve {
    /// Evaluates the scalar force multiplier at time `t_s` (seconds).
    #[inline]
    pub fn evaluate(&self, t_s: f64) -> f64 {
        match *self {
            Self::Constant(c) => c,
            Self::Ramp { duration_s } => {
                if duration_s <= 1e-15 {
                    1.0
                } else {
                    (t_s / duration_s).clamp(0.0, 1.0)
                }
            }
            Self::Pulse {
                start_s,
                end_s,
                amplitude,
            } => {
                if t_s >= start_s && t_s <= end_s {
                    amplitude
                } else {
                    0.0
                }
            }
            Self::Sine {
                frequency_hz,
                amplitude,
                phase_rad,
            } => amplitude * (2.0 * std::f64::consts::PI * frequency_hz * t_s + phase_rad).sin(),
        }
    }
}

/// Configuration options for the implicit `HHT-α` dynamic FEA integrator (`P5-T6`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HhtOptions {
    /// Hilber–Hughes–Taylor algorithmic damping parameter $\alpha \in [-0.3, 0.0]$.
    /// `0.0` recovers exact energy-conserving trapezoidal Newmark ($\beta = 1/4, \gamma = 1/2$);
    /// `-0.05`..`-0.1` adds mild high-frequency numerical dissipation for contact/fracture shocks.
    pub alpha: f64,
    /// Total simulated physical duration $T$ in seconds.
    pub total_time_s: f64,
    /// Initial time step $\Delta t_0$ in seconds.
    pub dt_initial_s: f64,
    /// Minimum adaptive time step $\Delta t_{\min}$ in seconds before failing a step.
    pub dt_min_s: f64,
    /// Maximum adaptive time step $\Delta t_{\max}$ in seconds.
    pub dt_max_s: f64,
    /// Consistent vs lumped mass matrix formulation.
    pub mass_matrix_kind: MassMatrixKind,
    /// Mass-proportional Rayleigh damping coefficient $c_M$ ($\mathbf{C} = c_M \mathbf{M} + c_K \mathbf{K}_0$).
    pub rayleigh_mass_alpha: f64,
    /// Stiffness-proportional Rayleigh damping coefficient $c_K$.
    pub rayleigh_stiffness_beta: f64,
    /// Linear solver backend (`Auto`, `SparseDirectLdlt`, `IcPcg`, `SsorPcg`, `JacobiPcg`).
    pub linear_solver: LinearSolverKind,
    /// Linear solver tolerance and iteration cap.
    pub linear_solver_options: SolverOptions,
    /// Enable Co-Rotated FEM finite-rotation kinematics ($\mathbf{F} = \mathbf{R}\mathbf{U}$).
    pub corotated: bool,
    /// Include symmetrized geometric stiffness in the Newton tangent.
    pub include_geometric_stiffness: bool,
    /// Enable $J_2$ elastoplasticity (`false` enforces purely elastic material response).
    pub enable_plasticity: bool,
    /// Enable continuum damage, element erosion, and detached-fragment extraction.
    pub enable_fracture: bool,
    /// Maximum Newton–Raphson equilibrium iterations per time step (default `15`).
    pub max_newton_iterations: usize,
    /// Relative out-of-balance force residual convergence tolerance (default `1e-6`).
    pub force_tolerance: f64,
    /// Enable backtracking line search on nonlinear steps (default `true`).
    pub enable_line_search: bool,
    /// Maximum consecutive adaptive bisections allowed for a single time step (default `10`).
    pub max_bisections_per_step: usize,
    /// Maximum single-step scalar damage increment before triggering adaptive bisection (default `0.45`).
    pub max_damage_jump_per_step: f64,
    /// Maximum single-step equivalent plastic strain increment before triggering adaptive bisection (default `0.08`).
    pub max_plastic_jump_per_step: f64,
    /// Dimensionless contact penalty multiplier $\kappa$ where $\epsilon_n = \kappa E / h$ (default `8.0`).
    pub contact_penalty_scale: f64,
    /// Minimum physical interval between recorded frame snapshots in seconds (`0.0` records every step up to `max_frames`).
    pub output_interval_s: f64,
    /// Hard upper bound on stored frames in the report (default `600`).
    pub max_frames: usize,
    /// Minimum element count for a detached connected component to spawn as a [`DetachedFragment`].
    pub min_fragment_elements: usize,
}

impl Default for HhtOptions {
    fn default() -> Self {
        Self {
            alpha: -0.05,
            total_time_s: 0.02,
            dt_initial_s: 1e-3,
            dt_min_s: 1e-6,
            dt_max_s: 5e-3,
            mass_matrix_kind: MassMatrixKind::Consistent,
            rayleigh_mass_alpha: 0.0,
            rayleigh_stiffness_beta: 0.0,
            linear_solver: LinearSolverKind::Auto,
            linear_solver_options: SolverOptions {
                tolerance: 1e-10,
                max_iterations: 6000,
            },
            corotated: true,
            include_geometric_stiffness: false,
            enable_plasticity: true,
            enable_fracture: true,
            max_newton_iterations: 25,
            force_tolerance: 1e-6,
            enable_line_search: true,
            max_bisections_per_step: 10,
            max_damage_jump_per_step: 0.45,
            max_plastic_jump_per_step: 0.08,
            contact_penalty_scale: 2.0,
            output_interval_s: 0.0,
            max_frames: 600,
            min_fragment_elements: 1,
        }
    }
}

impl HhtOptions {
    /// Clamped HHT-$\alpha$ parameter guaranteed to lie in $[-0.3, 0.0]$ for unconditional stability.
    #[inline]
    pub fn clamped_alpha(&self) -> f64 {
        self.alpha.clamp(-0.3, 0.0)
    }

    /// Associated Newmark $\beta = \frac{(1 - \alpha)^2}{4}$ parameter.
    #[inline]
    pub fn beta(&self) -> f64 {
        let a = self.clamped_alpha();
        0.25 * (1.0 - a) * (1.0 - a)
    }

    /// Associated Newmark $\gamma = \frac{1 - 2\alpha}{2}$ parameter.
    #[inline]
    pub fn gamma(&self) -> f64 {
        let a = self.clamped_alpha();
        0.5 * (1.0 - 2.0 * a)
    }
}

fn default_lock_transverse_dyn() -> bool {
    true
}

/// Time-dependent actuation / constraint drive for a dynamic `Revolute` hub coupling (`P5-T14`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum DynamicRevoluteDrive {
    /// Prescribed finite shaft rotation law $\theta(t)$ driven at target angular velocity
    /// with optional $C^2$ cosine startup ramp over `ramp_duration_s` and optional travel limits.
    PrescribedVelocity {
        /// Target angular velocity $\omega$ in rad/s about the hub axis.
        angular_velocity_rad_s: f64,
        /// Initial angle $\theta_0$ at $t = 0$ in radians.
        #[serde(default)]
        initial_angle_rad: f64,
        /// Smooth $C^2$ startup ramp duration in seconds (`0.0` = constant velocity from $t = 0$).
        #[serde(default)]
        ramp_duration_s: f64,
        /// Optional travel limits `[min_rad, max_rad]`.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        limits: Option<[f64; 2]>,
    },
    /// 1-DOF dynamic rotational pilot driven by a smooth torque-limited velocity motor:
    /// $\tau_{\text{motor}}(\dot{\theta}) = \frac{c_m (\omega_{\text{ref}}(t) - \dot{\theta})}{\sqrt{1 + (c_m (\omega_{\text{ref}}(t) - \dot{\theta}) / \tau_{\max})^2}}$.
    Motor {
        /// Desired angular velocity $\omega_{\text{target}}$ in rad/s.
        target_velocity_rad_s: f64,
        /// Maximum motor torque magnitude $\tau_{\max} \ge 0$ in N·m.
        max_torque_nm: f64,
        /// Proportional velocity gain $c_m$ in N·m·s/rad (`<= 0.0` auto-tunes from rotor inertia and `max_torque_nm`).
        #[serde(default)]
        velocity_gain_nm_s_per_rad: f64,
        /// Optional smooth startup ramp duration in seconds for `target_velocity_rad_s`.
        #[serde(default)]
        ramp_duration_s: f64,
        /// Optional travel limits `[min_rad, max_rad]`.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        limits: Option<[f64; 2]>,
    },
    /// 1-DOF dynamic rotational pilot free to spin under contact/inertial forces plus optional
    /// applied torque, viscous damping, torsional load spring, and travel limits:
    /// $\tau_{\text{net}}(\theta, \dot{\theta}) = \tau_{\text{app}} - c_\theta \dot{\theta} - k_\theta (\theta - \theta_{\text{ref}})$.
    FreeTorque {
        /// Constant external axial torque $\tau_{\text{app}}$ in N·m.
        #[serde(default)]
        applied_torque_nm: f64,
        /// Viscous bearing/load damping $c_\theta \ge 0$ in N·m·s/rad.
        #[serde(default)]
        viscous_damping_nm_s_per_rad: f64,
        /// Torsional load spring stiffness $k_\theta \ge 0$ in N·m/rad.
        #[serde(default)]
        torsional_stiffness_nm_per_rad: f64,
        /// Torsional spring reference angle $\theta_{\text{ref}}$ in radians.
        #[serde(default)]
        reference_angle_rad: f64,
        /// Optional travel limits `[min_rad, max_rad]`.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        limits: Option<[f64; 2]>,
    },
}

/// Time-dependent actuation / constraint drive for a dynamic `Prismatic` hub coupling (`P5-T14`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum DynamicPrismaticDrive {
    /// Prescribed linear translation law $s(t)$ along the prismatic axis.
    PrescribedVelocity {
        /// Target linear velocity $v$ in m/s along the hub axis.
        velocity_m_s: f64,
        /// Initial displacement $s_0$ at $t = 0$ in meters.
        #[serde(default)]
        initial_displacement_m: f64,
        /// Smooth $C^2$ startup ramp duration in seconds (`0.0` = constant velocity from $t = 0$).
        #[serde(default)]
        ramp_duration_s: f64,
        /// Optional travel limits `[min_m, max_m]`.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        limits: Option<[f64; 2]>,
    },
    /// 1-DOF dynamic translational pilot driven by a smooth force-limited velocity motor.
    Motor {
        /// Desired linear velocity $v_{\text{target}}$ in m/s.
        target_velocity_m_s: f64,
        /// Maximum motor force magnitude $F_{\max} \ge 0$ in Newtons.
        max_force_n: f64,
        /// Proportional velocity gain $c_m$ in N·s/m (`<= 0.0` auto-tunes from body mass and `max_force_n`).
        #[serde(default)]
        velocity_gain_n_s_per_m: f64,
        /// Optional smooth startup ramp duration in seconds for `target_velocity_m_s`.
        #[serde(default)]
        ramp_duration_s: f64,
        /// Optional travel limits `[min_m, max_m]`.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        limits: Option<[f64; 2]>,
    },
    /// 1-DOF dynamic translational pilot free to slide under contact/inertial forces plus
    /// optional applied force, viscous damping, axial spring, and travel limits.
    FreeForce {
        /// Constant external axial force $F_{\text{app}}$ in Newtons.
        #[serde(default)]
        applied_force_n: f64,
        /// Viscous damping $c_s \ge 0$ in N·s/m.
        #[serde(default)]
        viscous_damping_n_s_per_m: f64,
        /// Axial spring stiffness $k_s \ge 0$ in N/m.
        #[serde(default)]
        axial_stiffness_n_per_m: f64,
        /// Axial spring reference displacement $s_{\text{ref}}$ in meters.
        #[serde(default)]
        reference_disp_m: f64,
        /// Optional travel limits `[min_m, max_m]`.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        limits: Option<[f64; 2]>,
    },
}

/// Dynamic kinematic RBE2 hub coupling attached to a deformable body inside `HHT-α` (`P5-T14`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DynamicHubCoupling {
    /// 1-DOF rotational shaft/bore hub coupling with exact finite-rotation $SO(3)$ kinematics.
    Revolute {
        /// Optional human-readable identifier (e.g., matching `JointHubCouplingSpec::joint_id`).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        name: Option<String>,
        /// Point on the rotation axis in meters.
        pivot: [f64; 3],
        /// Rotation axis direction (automatically normalized).
        axis: [f64; 3],
        /// Node selector identifying the hub/bore boundary nodes.
        selector: HubNodeSelector,
        /// Prescribed motion law, torque-limited velocity motor, or free/loaded rotational pilot.
        drive: DynamicRevoluteDrive,
    },
    /// 1-DOF translational slider/platen hub coupling along `axis`.
    Prismatic {
        /// Optional human-readable identifier.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        name: Option<String>,
        /// Translation axis direction (automatically normalized).
        axis: [f64; 3],
        /// Node selector identifying the coupled boundary nodes.
        selector: HubNodeSelector,
        /// Prescribed motion law, force-limited velocity motor, or free/loaded translational pilot.
        drive: DynamicPrismaticDrive,
        /// If `true`, transverse DOFs perpendicular to `axis` are locked to 0 (rigid slider).
        #[serde(default = "default_lock_transverse_dyn")]
        lock_transverse: bool,
    },
    /// 0-DOF rigidly fixed hub/bore mount.
    Fixed {
        /// Optional human-readable identifier.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        name: Option<String>,
        /// Node selector identifying the fixed boundary nodes.
        selector: HubNodeSelector,
    },
}

/// Instantaneous kinematic state and reaction wrench on a dynamic hub coupling (`P5-T14`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DynamicHubReaction {
    /// Body index in the coupled dynamic simulation.
    pub body_index: usize,
    /// Part identifier.
    pub part_id: String,
    /// Hub coupling name or joint ID.
    pub hub_name: String,
    /// Joint kind (`"revolute"`, `"prismatic"`, or `"fixed"`).
    pub kind: String,
    /// Number of active mesh nodes coupled to this hub.
    pub node_count: usize,
    /// Current scalar joint coordinate (`rad` for revolute, `m` for prismatic, `0.0` for fixed).
    pub coordinate: f64,
    /// Current scalar joint velocity (`rad/s` for revolute, `m/s` for prismatic, `0.0` for fixed).
    pub velocity: f64,
    /// Axial reaction torque (`N·m`) for revolute, axial reaction force (`N`) for prismatic, or net force norm (`N`) for fixed.
    pub reaction_effort: f64,
    /// Net 3D reaction force vector `[Fx, Fy, Fz]` exerted by the hub on the body (N).
    pub reaction_force_n: [f64; 3],
    /// Net 3D reaction torque vector `[Tx, Ty, Tz]` about the pivot exerted by the hub on the body (N·m).
    pub reaction_torque_nm: [f64; 3],
}

/// Deformable body specification for implicit dynamic `HHT-α` simulation.
#[derive(Debug, Clone)]
pub struct DynamicBodySpec {
    /// Unique identifier of the part/body.
    pub part_id: String,
    /// Volumetric tetrahedral mesh (`TET4` or `TET10`).
    pub mesh: TetrahedralMesh,
    /// $J_2$ elastoplastic constitutive parameters.
    pub material: J2MaterialParams,
    /// Material mass density $\rho$ in kg/m³.
    pub density_kg_m3: f64,
    /// Dirichlet supports and baseline Neumann nodal loads.
    pub boundary: BoundaryConditions,
    /// Dynamic RBE2 hub couplings (`Revolute`, `Prismatic`, `Fixed`) (`P5-T14`).
    pub hub_couplings: Vec<DynamicHubCoupling>,
    /// Optional initial nodal displacement vector (`node_count * 3`, meters).
    pub initial_displacement: Option<Vec<f64>>,
    /// Optional initial nodal velocity vector (`node_count * 3`, m/s).
    pub initial_velocity: Option<Vec<f64>>,
    /// Uniform gravitational acceleration vector in m/s² (default `[0.0; 3]`).
    pub gravity: [f64; 3],
    /// Optional continuum damage / fracture parameters.
    pub damage_params: Option<DamageMaterialParams>,
    /// Time modulation applied to `boundary` Neumann forces.
    pub force_curve: TimeForceCurve,
}

impl DynamicBodySpec {
    /// Creates a new dynamic body specification with zero initial displacement/velocity and zero gravity.
    pub fn new(
        part_id: impl Into<String>,
        mesh: TetrahedralMesh,
        material: J2MaterialParams,
        density_kg_m3: f64,
    ) -> Self {
        Self {
            part_id: part_id.into(),
            mesh,
            material,
            density_kg_m3,
            boundary: BoundaryConditions::new(),
            hub_couplings: Vec::new(),
            initial_displacement: None,
            initial_velocity: None,
            gravity: [0.0; 3],
            damage_params: None,
            force_curve: TimeForceCurve::Constant(1.0),
        }
    }

    /// Attaches boundary conditions.
    pub fn with_boundary(mut self, boundary: BoundaryConditions) -> Self {
        self.boundary = boundary;
        self
    }

    /// Sets initial nodal displacement vector (`node_count * 3`, meters).
    pub fn with_initial_displacement(mut self, u0: Vec<f64>) -> Self {
        self.initial_displacement = Some(u0);
        self
    }

    /// Sets initial nodal velocity vector (`node_count * 3`, m/s).
    pub fn with_initial_velocity(mut self, v0: Vec<f64>) -> Self {
        self.initial_velocity = Some(v0);
        self
    }

    /// Sets uniform initial velocity `[vx, vy, vz]` (m/s) on all free nodes.
    pub fn with_uniform_initial_velocity(mut self, velocity: [f64; 3]) -> Self {
        let n = self.mesh.node_count();
        let mut v0 = vec![0.0; n * 3];
        for i in 0..n {
            v0[3 * i] = velocity[0];
            v0[3 * i + 1] = velocity[1];
            v0[3 * i + 2] = velocity[2];
        }
        self.initial_velocity = Some(v0);
        self
    }

    /// Sets uniform gravity vector in m/s².
    pub fn with_gravity(mut self, gravity: [f64; 3]) -> Self {
        self.gravity = gravity;
        self
    }

    /// Attaches local continuum damage/fracture parameters.
    pub fn with_damage(mut self, damage: DamageMaterialParams) -> Self {
        self.damage_params = Some(damage);
        self
    }

    /// Sets time-modulation curve for external Neumann loads.
    pub fn with_force_curve(mut self, curve: TimeForceCurve) -> Self {
        self.force_curve = curve;
        self
    }

    /// Attaches a single dynamic hub coupling (`P5-T14`).
    pub fn with_hub_coupling(mut self, hub: DynamicHubCoupling) -> Self {
        self.hub_couplings.push(hub);
        self
    }

    /// Appends multiple dynamic hub couplings (`P5-T14`).
    pub fn with_hub_couplings(mut self, hubs: Vec<DynamicHubCoupling>) -> Self {
        self.hub_couplings.extend(hubs);
        self
    }
}

/// Active contact force record captured at a simulation frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DynamicContactForce {
    /// Contact pair index in `pairs`.
    pub pair_index: usize,
    /// Slave body index.
    pub slave_body: usize,
    /// Master body index (`None` for analytical rigid surface).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub master_body: Option<usize>,
    /// World contact point in meters.
    pub point_m: [f64; 3],
    /// Outward master unit normal at contact point.
    pub normal: [f64; 3],
    /// Compressive normal contact force magnitude in Newtons ($\ge 0$).
    pub normal_force_n: f64,
    /// Tangential frictional contact force vector in Newtons.
    pub tangential_force_n: [f64; 3],
    /// Geometric penetration depth in meters ($\ge 0$).
    pub penetration_m: f64,
}

/// Instantaneous system energy balance snapshot (Joules).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct DynamicEnergyBalance {
    /// Kinetic energy $E_k = \frac{1}{2}\mathbf{v}^T \mathbf{M} \mathbf{v}$ (J).
    pub kinetic_j: f64,
    /// Recoverable elastic strain energy $E_e = \int w_e \, dV$ (J).
    pub elastic_j: f64,
    /// Cumulative irreversible plastic dissipation $E_p = \int w_p \, dV$ (J).
    pub plastic_j: f64,
    /// Cumulative fracture surface energy dissipation $E_f$ (J).
    pub fracture_j: f64,
    /// Cumulative external work $W_{\text{ext}}$ (J).
    pub external_work_j: f64,
    /// Total mechanical energy $E_{\text{mech}} = E_k + E_e$ (J).
    pub total_mechanical_j: f64,
    /// Total internal + kinetic energy $E_{\text{tot}} = E_k + E_e + E_p + E_f$ (J).
    pub total_energy_j: f64,
}

/// Per-body per-frame deformed state and physical fields stored in the simulation cache.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DynamicBodyFrame {
    /// Identifier of the body/part.
    pub part_id: String,
    /// Total mesh nodes.
    pub node_count: usize,
    /// Deformed nodal world coordinates $\mathbf{x}(t) = \mathbf{X} + \mathbf{u}(t)$ in meters.
    pub deformed_positions: Vec<[f64; 3]>,
    /// Nodal displacement vectors $\mathbf{u}(t)$ in meters.
    pub displacements: Vec<[f64; 3]>,
    /// Nodal velocity vectors $\mathbf{v}(t)$ in m/s.
    pub velocities: Vec<[f64; 3]>,
    /// Nodal-averaged Cauchy von Mises stress $\sigma_{vM}$ in Pascals.
    pub nodal_von_mises_pa: Vec<f64>,
    /// Nodal-averaged equivalent plastic strain $\bar{\varepsilon}_p$ (dimensionless).
    pub nodal_equiv_plastic_strain: Vec<f64>,
    /// Nodal-averaged continuum scalar damage $D \in [0, 1]$.
    pub nodal_damage: Vec<f64>,
    /// Indices of currently eroded elements.
    pub eroded_element_indices: Vec<usize>,
    /// Number of currently active (non-eroded, non-detached) elements.
    pub active_element_count: usize,
    /// Peak nodal displacement magnitude $\|\mathbf{u}\|_{\max}$ in meters.
    pub max_displacement_m: f64,
    /// Peak element von Mises stress in Pascals.
    pub max_von_mises_pa: f64,
    /// Peak element equivalent plastic strain.
    pub max_equiv_plastic_strain: f64,
    /// Peak element scalar damage $D \in [0, 1]$.
    pub max_damage: f64,
}

/// Discrete event logged during a dynamic HHT-α bake.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DynamicTimelineEvent {
    /// Physical simulation time in seconds.
    pub time_s: f64,
    /// Event category (`"step_bisection"`, `"contact_impact"`, `"plastic_yield"`, `"crack_initiation"`,
    /// `"element_erosion"`, `"fragment_detachment"`, `"cancelled"`).
    pub kind: String,
    /// Optional affected part ID.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub part_id: Option<String>,
    /// Human-readable description.
    pub description: String,
}

/// Complete cached simulation frame at time $t$.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DynamicFrameSnapshot {
    /// Zero-based sequential frame index.
    pub frame_index: usize,
    /// Physical simulation time $t$ in seconds.
    pub time_s: f64,
    /// Adaptive time step $\Delta t$ used to reach this frame (seconds; `0.0` for initial frame 0).
    pub dt_used_s: f64,
    /// Number of Newton–Raphson iterations consumed on the step reaching this frame.
    pub newton_iterations: usize,
    /// Per-body deformed coordinates and physical fields.
    pub bodies: Vec<DynamicBodyFrame>,
    /// Active contact forces at this frame.
    pub contact_forces: Vec<DynamicContactForce>,
    /// Detached rigid fragments spawned up to this frame.
    pub detached_fragments: Vec<DetachedFragment>,
    /// Energy balance at this frame.
    pub energy: DynamicEnergyBalance,
    /// Per-hub kinematic coordinates, velocities, and reaction wrenches at this frame (`P5-T14`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub hub_reactions: Vec<DynamicHubReaction>,
}

/// Real-time progress telemetry emitted during an implicit dynamic bake (for SSE streaming).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DynamicProgressUpdate {
    /// Current simulated physical time in seconds.
    pub time_s: f64,
    /// Target total physical duration in seconds.
    pub total_time_s: f64,
    /// Completion percentage in `[0.0, 100.0]`.
    pub progress_pct: f64,
    /// Completed implicit time steps so far.
    pub step_count: usize,
    /// Cached frames stored so far.
    pub frame_count: usize,
    /// Current adaptive time step $\Delta t$ in seconds.
    pub current_dt_s: f64,
    /// Total adaptive bisections so far.
    pub bisections: usize,
    /// Total eroded elements across all bodies so far.
    pub eroded_elements: usize,
    /// Total detached rigid fragments spawned so far.
    pub detached_fragments: usize,
    /// Instantaneous peak von Mises stress across all bodies (Pa).
    pub max_von_mises_pa: f64,
}

/// Complete result of an implicit `HHT-α` coupled dynamic simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DynamicHhtReport {
    /// Cached timeline frames from $t = 0$ to $t_{\text{end}}$.
    pub frames: Vec<DynamicFrameSnapshot>,
    /// Discrete physical and solver events along the timeline.
    pub events: Vec<DynamicTimelineEvent>,
    /// All detached rigid fragments created across the simulation.
    pub all_detached_fragments: Vec<DetachedFragment>,
    /// Total converged implicit time steps executed.
    pub total_steps: usize,
    /// Total cumulative Newton–Raphson iterations.
    pub total_newton_iterations: usize,
    /// Total adaptive time-step bisections triggered.
    pub total_bisections: usize,
    /// Minimum adaptive time step used (seconds).
    pub min_dt_used_s: f64,
    /// Maximum adaptive time step used (seconds).
    pub max_dt_used_s: f64,
    /// Estimated explicit CFL critical time step $\Delta t_{\text{CFL}} = h_{\min} / c_d$ (seconds).
    pub cfl_critical_dt_s: f64,
    /// Maximum achieved CFL ratio $\Delta t_{\max,\text{used}} / \Delta t_{\text{CFL}}$.
    pub cfl_ratio_max: f64,
    /// Initial total mechanical energy $E_k(0) + E_e(0)$ in Joules.
    pub initial_total_energy_j: f64,
    /// Final total mechanical energy $E_k(T) + E_e(T)$ in Joules.
    pub final_total_energy_j: f64,
    /// Maximum percentage drift of mechanical energy $|E_{\text{mech}}(t) - E_{\text{mech}}(0)| / \max(E_{\text{mech}}(0), 10^{-18}) \times 100$.
    pub max_energy_drift_pct: f64,
    /// `true` if the simulation stopped early due to a cancellation request.
    pub cancelled: bool,
    /// Wall-clock execution time in milliseconds.
    pub elapsed_ms: f64,
}

// ============================================================================
// CFL Estimation & Mass Matrix Assembly
// ============================================================================

/// Estimates the explicit Courant–Friedrichs–Lewy (CFL) critical time step
/// $\Delta t_{\text{CFL}} = h_{\min} / c_d$ where $c_d = \sqrt{\frac{E(1-\nu)}{\rho(1+\nu)(1-2\nu)}}$
/// is the elastic dilatational (P-wave) speed.
pub fn estimate_cfl_critical_dt(
    mesh: &TetrahedralMesh,
    youngs_modulus_pa: f64,
    poissons_ratio: f64,
    density_kg_m3: f64,
) -> f64 {
    if mesh.is_empty() || youngs_modulus_pa <= 0.0 || density_kg_m3 <= 0.0 {
        return 1e-6;
    }
    let nu = poissons_ratio.clamp(0.0, 0.499);
    let c_d = ((youngs_modulus_pa * (1.0 - nu))
        / (density_kg_m3 * (1.0 + nu) * (1.0 - 2.0 * nu)))
        .sqrt();
    if !c_d.is_finite() || c_d <= 0.0 {
        return 1e-6;
    }

    let mut h_min = f64::INFINITY;
    let corners_iter: Vec<[usize; 4]> = if mesh.is_tet10() {
        mesh.tet10_elements
            .iter()
            .map(|n| [n[0], n[1], n[2], n[3]])
            .collect()
    } else {
        mesh.elements.iter().map(|e| e.node_indices).collect()
    };

    for [i0, i1, i2, i3] in corners_iter {
        let p = [
            Vector3::from(mesh.nodes[i0].position),
            Vector3::from(mesh.nodes[i1].position),
            Vector3::from(mesh.nodes[i2].position),
            Vector3::from(mesh.nodes[i3].position),
        ];
        // Shortest edge length in the tetrahedron
        for a in 0..4 {
            for b in (a + 1)..4 {
                let len = (p[b] - p[a]).norm();
                if len > 1e-12 {
                    h_min = h_min.min(len);
                }
            }
        }
    }
    if !h_min.is_finite() {
        return 1e-6;
    }
    // Quadratic TET10 has mid-edge nodes halving effective nodal spacing
    let h_eff = if mesh.is_tet10() { 0.5 * h_min } else { h_min };
    h_eff / c_d
}

/// Exact dimensionless consistent mass matrix coefficients for the 10-node tetrahedron (`TET10`):
/// $M^e_{ab} = \rho V_e \cdot \text{TET10\_MASS\_COEFF}[a][b] \cdot I_3$.
///
/// Exact analytical integration of $6 \int_{\hat{T}} N_a N_b \, d\hat{V}$:
/// - corner–same corner: `6/420`
/// - corner–other corner: `1/420`
/// - corner–adjacent edge: `-4/420`
/// - corner–opposite edge: `-6/420`
/// - edge–same edge: `32/420`
/// - edge–adjacent edge: `16/420`
/// - edge–opposite edge: `8/420`
fn tet10_consistent_mass_scalar(a: usize, b: usize) -> f64 {
    const EDGE_NODES: [(usize, usize); 6] = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)];
    if a < 4 && b < 4 {
        if a == b {
            6.0 / 420.0
        } else {
            1.0 / 420.0
        }
    } else if a < 4 && b >= 4 {
        let (e0, e1) = EDGE_NODES[b - 4];
        if a == e0 || a == e1 {
            -4.0 / 420.0
        } else {
            -6.0 / 420.0
        }
    } else if a >= 4 && b < 4 {
        let (e0, e1) = EDGE_NODES[a - 4];
        if b == e0 || b == e1 {
            -4.0 / 420.0
        } else {
            -6.0 / 420.0
        }
    } else if a == b {
        32.0 / 420.0
    } else {
        let (a0, a1) = EDGE_NODES[a - 4];
        let (b0, b1) = EDGE_NODES[b - 4];
        if a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1 {
            16.0 / 420.0
        } else {
            8.0 / 420.0
        }
    }
}

/// Strictly positive HRZ (Hinton–Rock–Zienkiewicz) lumped mass fraction for `TET10` local node `a`:
/// - corners (`0..4`): `1/36`
/// - mid-edge nodes (`4..10`): `4/27`
///
/// Sum = `4 * (1/36) + 6 * (4/27) = 1/9 + 8/9 = 1.0`.
#[inline]
fn tet10_hrz_lumped_fraction(local_node: usize) -> f64 {
    if local_node < 4 {
        1.0 / 36.0
    } else {
        4.0 / 27.0
    }
}

/// Assembles the full-DOF global mass matrix $\mathbf{M}$ (size `3 * num_nodes`) for `TET4` elements.
/// Inactive elements (where `active_elements[e] == false`) contribute zero mass.
pub fn assemble_mass_matrix_tet4(
    num_nodes: usize,
    elements: &[Tet4Element],
    density_kg_m3: f64,
    kind: MassMatrixKind,
    active_elements: Option<&[bool]>,
) -> CsrMatrix {
    let total_dofs = num_nodes * 3;
    let mut rows: Vec<Vec<(usize, f64)>> = vec![Vec::with_capacity(24); total_dofs];

    for (e_idx, elem) in elements.iter().enumerate() {
        if active_elements.is_some_and(|m| !m.get(e_idx).copied().unwrap_or(true)) {
            continue;
        }
        let elem_mass = density_kg_m3 * elem.volume;
        match kind {
            MassMatrixKind::Lumped => {
                let m_node = 0.25 * elem_mass;
                for &nid in &elem.node_indices {
                    let d0 = 3 * nid;
                    rows[d0].push((d0, m_node));
                    rows[d0 + 1].push((d0 + 1, m_node));
                    rows[d0 + 2].push((d0 + 2, m_node));
                }
            }
            MassMatrixKind::Consistent => {
                let base = elem_mass / 20.0;
                for (li, &ni) in elem.node_indices.iter().enumerate() {
                    let di = 3 * ni;
                    for (lj, &nj) in elem.node_indices.iter().enumerate() {
                        let dj = 3 * nj;
                        let val = if li == lj { 2.0 * base } else { base };
                        rows[di].push((dj, val));
                        rows[di + 1].push((dj + 1, val));
                        rows[di + 2].push((dj + 2, val));
                    }
                }
            }
        }
    }

    CsrMatrix::from_row_entries(total_dofs, total_dofs, rows)
}

/// Assembles the full-DOF global mass matrix $\mathbf{M}$ (size `3 * num_nodes`) for `TET10` elements
/// (exact quadratic consistent mass or strictly positive HRZ lumped mass).
pub fn assemble_mass_matrix_tet10(
    num_nodes: usize,
    elements: &[Tet10Element],
    density_kg_m3: f64,
    kind: MassMatrixKind,
    active_elements: Option<&[bool]>,
) -> CsrMatrix {
    let total_dofs = num_nodes * 3;
    let mut rows: Vec<Vec<(usize, f64)>> = vec![Vec::with_capacity(48); total_dofs];

    for (e_idx, elem) in elements.iter().enumerate() {
        if active_elements.is_some_and(|m| !m.get(e_idx).copied().unwrap_or(true)) {
            continue;
        }
        let elem_vol: f64 = elem.gauss_weights.iter().sum();
        let elem_mass = density_kg_m3 * elem_vol;
        match kind {
            MassMatrixKind::Lumped => {
                for (li, &nid) in elem.node_indices.iter().enumerate() {
                    let m_node = elem_mass * tet10_hrz_lumped_fraction(li);
                    let d0 = 3 * nid;
                    rows[d0].push((d0, m_node));
                    rows[d0 + 1].push((d0 + 1, m_node));
                    rows[d0 + 2].push((d0 + 2, m_node));
                }
            }
            MassMatrixKind::Consistent => {
                for (li, &ni) in elem.node_indices.iter().enumerate() {
                    let di = 3 * ni;
                    for (lj, &nj) in elem.node_indices.iter().enumerate() {
                        let coeff = tet10_consistent_mass_scalar(li, lj);
                        let val = elem_mass * coeff;
                        let dj = 3 * nj;
                        rows[di].push((dj, val));
                        rows[di + 1].push((dj + 1, val));
                        rows[di + 2].push((dj + 2, val));
                    }
                }
            }
        }
    }

    CsrMatrix::from_row_entries(total_dofs, total_dofs, rows)
}

// ============================================================================
// Internal Dynamic Body Runtime State
// ============================================================================

enum DynamicElements {
    Tet4(Vec<Tet4Element>),
    Tet10(Vec<Tet10Element>),
}

impl DynamicElements {
    fn from_mesh(mesh: &TetrahedralMesh) -> Result<Self, FeaError> {
        if mesh.is_empty() {
            return Err(FeaError::EmptyMesh);
        }
        if mesh.is_tet10() {
            let mut elems = Vec::with_capacity(mesh.tet10_elements.len());
            for (id, &node_indices) in mesh.tet10_elements.iter().enumerate() {
                let mut pos = [[0.0; 3]; 10];
                for (k, &nid) in node_indices.iter().enumerate() {
                    pos[k] = mesh.nodes[nid].position;
                }
                elems.push(Tet10Element::new(id, node_indices, pos)?);
            }
            Ok(Self::Tet10(elems))
        } else {
            let mut elems = Vec::with_capacity(mesh.elements.len());
            for e in &mesh.elements {
                let p = [
                    mesh.nodes[e.node_indices[0]].position,
                    mesh.nodes[e.node_indices[1]].position,
                    mesh.nodes[e.node_indices[2]].position,
                    mesh.nodes[e.node_indices[3]].position,
                ];
                elems.push(Tet4Element::new(e.id, e.node_indices, p)?);
            }
            Ok(Self::Tet4(elems))
        }
    }

    fn element_count(&self) -> usize {
        match self {
            Self::Tet4(v) => v.len(),
            Self::Tet10(v) => v.len(),
        }
    }

    fn gp_count(&self) -> usize {
        match self {
            Self::Tet4(v) => v.len(),
            Self::Tet10(v) => v.len() * 4,
        }
    }

    fn element_volumes(&self) -> Vec<f64> {
        match self {
            Self::Tet4(v) => v.iter().map(|e| e.volume).collect(),
            Self::Tet10(v) => v.iter().map(|e| e.gauss_weights.iter().sum()).collect(),
        }
    }

    fn assemble_mass(
        &self,
        num_nodes: usize,
        density: f64,
        kind: MassMatrixKind,
        active: &[bool],
    ) -> CsrMatrix {
        match self {
            Self::Tet4(v) => {
                assemble_mass_matrix_tet4(num_nodes, v, density, kind, Some(active))
            }
            Self::Tet10(v) => {
                assemble_mass_matrix_tet10(num_nodes, v, density, kind, Some(active))
            }
        }
    }
}

/// Coordinate-axis alignment helper for frictionless prismatic platens (`lock_transverse == false`).
fn aligned_coordinate_axis(axis: [f64; 3]) -> Option<(usize, f64)> {
    let eps = 1e-9;
    let ax = axis[0].abs();
    let ay = axis[1].abs();
    let az = axis[2].abs();
    if ax > 1.0 - eps && ay < eps && az < eps {
        Some((0, axis[0].signum()))
    } else if ay > 1.0 - eps && ax < eps && az < eps {
        Some((1, axis[1].signum()))
    } else if az > 1.0 - eps && ax < eps && ay < eps {
        Some((2, axis[2].signum()))
    } else {
        None
    }
}

/// Exact Rodrigues finite rotation of vector `r` by angle `theta` around unit `axis`.
#[inline]
fn rodrigues_rotate(axis: Vector3<f64>, theta: f64, r: Vector3<f64>) -> Vector3<f64> {
    if theta.abs() < 1e-15 {
        return r;
    }
    let (s, c) = theta.sin_cos();
    r * c + axis.cross(&r) * s + axis * (axis.dot(&r) * (1.0 - c))
}

/// Evaluates a prescribed constant-velocity law with optional $C^2$ cosine startup ramp and travel limits.
/// Returns `(q, q_dot, q_ddot)`.
fn eval_prescribed_velocity_law(
    t: f64,
    v_target: f64,
    q0: f64,
    ramp_duration_s: f64,
    limits: Option<[f64; 2]>,
) -> (f64, f64, f64) {
    let t = t.max(0.0);
    let (mut q, mut v, mut a) = if ramp_duration_s > 1e-12 {
        let tr = ramp_duration_s;
        if t < tr {
            let phase = std::f64::consts::PI * t / tr;
            let q_val = q0 + 0.5 * v_target * (t - (tr / std::f64::consts::PI) * phase.sin());
            let v_val = 0.5 * v_target * (1.0 - phase.cos());
            let a_val = 0.5 * v_target * (std::f64::consts::PI / tr) * phase.sin();
            (q_val, v_val, a_val)
        } else {
            let q_val = q0 + v_target * (t - 0.5 * tr);
            (q_val, v_target, 0.0)
        }
    } else {
        (q0 + v_target * t, v_target, 0.0)
    };

    if let Some([lo, hi]) = limits {
        let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
        if q < lo {
            q = lo;
            if v < 0.0 {
                v = 0.0;
                a = 0.0;
            }
        } else if q > hi {
            q = hi;
            if v > 0.0 {
                v = 0.0;
                a = 0.0;
            }
        }
    }
    (q, v, a)
}

/// Evaluates ramped motor target velocity at time `t`.
#[inline]
fn eval_ramped_motor_velocity(t: f64, v_target: f64, ramp_duration_s: f64) -> f64 {
    if ramp_duration_s > 1e-12 && t < ramp_duration_s {
        let phase = std::f64::consts::PI * t.max(0.0) / ramp_duration_s;
        0.5 * v_target * (1.0 - phase.cos())
    } else {
        v_target
    }
}

#[derive(Debug, Clone, Copy)]
enum ResolvedDynamicHubKind {
    Revolute {
        pivot: [f64; 3],
        axis: [f64; 3],
        drive: DynamicRevoluteDrive,
    },
    Prismatic {
        axis: [f64; 3],
        drive: DynamicPrismaticDrive,
        lock_transverse: bool,
    },
    Fixed,
}

#[derive(Debug, Clone)]
struct ResolvedDynamicHub {
    name: String,
    node_ids: Vec<usize>,
    kind: ResolvedDynamicHubKind,
    q: f64,
    q_dot: f64,
    q_ddot: f64,
    q_effort_prev: f64,
    /// Whole-body effective rotational inertia (kg·m²) or mass (kg) about this hub axis.
    body_inertia_est: f64,
}

impl ResolvedDynamicHub {
    fn from_spec(
        spec: &DynamicHubCoupling,
        mesh: &TetrahedralMesh,
        fallback_idx: usize,
    ) -> Result<Self, FeaError> {
        match spec {
            DynamicHubCoupling::Revolute {
                name,
                pivot,
                axis,
                selector,
                drive,
            } => {
                let node_ids = selector.resolve_nodes(mesh)?;
                let a_raw = Vector3::from(*axis);
                let a_norm = a_raw.norm();
                if a_norm <= 1e-12 {
                    return Err(FeaError::General(
                        "Dynamic Revolute hub axis must have non-zero length".to_string(),
                    ));
                }
                let unit_a = a_raw / a_norm;
                let (q0, v0, a0) = match *drive {
                    DynamicRevoluteDrive::PrescribedVelocity {
                        angular_velocity_rad_s,
                        initial_angle_rad,
                        ramp_duration_s,
                        limits,
                    } => eval_prescribed_velocity_law(
                        0.0,
                        angular_velocity_rad_s,
                        initial_angle_rad,
                        ramp_duration_s,
                        limits,
                    ),
                    DynamicRevoluteDrive::Motor { .. } | DynamicRevoluteDrive::FreeTorque { .. } => {
                        (0.0, 0.0, 0.0)
                    }
                };
                Ok(Self {
                    name: name
                        .clone()
                        .unwrap_or_else(|| format!("revolute_hub_{fallback_idx}")),
                    node_ids,
                    kind: ResolvedDynamicHubKind::Revolute {
                        pivot: *pivot,
                        axis: [unit_a.x, unit_a.y, unit_a.z],
                        drive: *drive,
                    },
                    q: q0,
                    q_dot: v0,
                    q_ddot: a0,
                    q_effort_prev: 0.0,
                    body_inertia_est: 1e-4,
                })
            }
            DynamicHubCoupling::Prismatic {
                name,
                axis,
                selector,
                drive,
                lock_transverse,
            } => {
                let node_ids = selector.resolve_nodes(mesh)?;
                let a_raw = Vector3::from(*axis);
                let a_norm = a_raw.norm();
                if a_norm <= 1e-12 {
                    return Err(FeaError::General(
                        "Dynamic Prismatic hub axis must have non-zero length".to_string(),
                    ));
                }
                let unit_a = a_raw / a_norm;
                let (q0, v0, a0) = match *drive {
                    DynamicPrismaticDrive::PrescribedVelocity {
                        velocity_m_s,
                        initial_displacement_m,
                        ramp_duration_s,
                        limits,
                    } => eval_prescribed_velocity_law(
                        0.0,
                        velocity_m_s,
                        initial_displacement_m,
                        ramp_duration_s,
                        limits,
                    ),
                    DynamicPrismaticDrive::Motor { .. }
                    | DynamicPrismaticDrive::FreeForce { .. } => (0.0, 0.0, 0.0),
                };
                Ok(Self {
                    name: name
                        .clone()
                        .unwrap_or_else(|| format!("prismatic_hub_{fallback_idx}")),
                    node_ids,
                    kind: ResolvedDynamicHubKind::Prismatic {
                        axis: [unit_a.x, unit_a.y, unit_a.z],
                        drive: *drive,
                        lock_transverse: *lock_transverse,
                    },
                    q: q0,
                    q_dot: v0,
                    q_ddot: a0,
                    q_effort_prev: 0.0,
                    body_inertia_est: 1.0,
                })
            }
            DynamicHubCoupling::Fixed { name, selector } => {
                let node_ids = selector.resolve_nodes(mesh)?;
                Ok(Self {
                    name: name
                        .clone()
                        .unwrap_or_else(|| format!("fixed_hub_{fallback_idx}")),
                    node_ids,
                    kind: ResolvedDynamicHubKind::Fixed,
                    q: 0.0,
                    q_dot: 0.0,
                    q_ddot: 0.0,
                    q_effort_prev: 0.0,
                    body_inertia_est: 1.0,
                })
            }
        }
    }

    #[inline]
    fn is_pilot(&self) -> bool {
        matches!(
            self.kind,
            ResolvedDynamicHubKind::Revolute {
                drive: DynamicRevoluteDrive::Motor { .. } | DynamicRevoluteDrive::FreeTorque { .. },
                ..
            } | ResolvedDynamicHubKind::Prismatic {
                drive: DynamicPrismaticDrive::Motor { .. } | DynamicPrismaticDrive::FreeForce { .. },
                ..
            }
        )
    }

    #[inline]
    fn kind_str(&self) -> &'static str {
        match self.kind {
            ResolvedDynamicHubKind::Revolute { .. } => "revolute",
            ResolvedDynamicHubKind::Prismatic { .. } => "prismatic",
            ResolvedDynamicHubKind::Fixed => "fixed",
        }
    }

    /// Evaluates prescribed kinematics `(q, q_dot, q_ddot)` at time `t` for non-pilot hubs.
    fn prescribed_kinematics_at(&self, t: f64) -> (f64, f64, f64) {
        match self.kind {
            ResolvedDynamicHubKind::Revolute {
                drive:
                    DynamicRevoluteDrive::PrescribedVelocity {
                        angular_velocity_rad_s,
                        initial_angle_rad,
                        ramp_duration_s,
                        limits,
                    },
                ..
            } => eval_prescribed_velocity_law(
                t,
                angular_velocity_rad_s,
                initial_angle_rad,
                ramp_duration_s,
                limits,
            ),
            ResolvedDynamicHubKind::Prismatic {
                drive:
                    DynamicPrismaticDrive::PrescribedVelocity {
                        velocity_m_s,
                        initial_displacement_m,
                        ramp_duration_s,
                        limits,
                    },
                ..
            } => eval_prescribed_velocity_law(
                t,
                velocity_m_s,
                initial_displacement_m,
                ramp_duration_s,
                limits,
            ),
            _ => (0.0, 0.0, 0.0),
        }
    }

    /// Evaluates scalar pilot generalized effort `Q(q, q_dot, t)` and positive semi-definite
    /// Newton tangent stiffness `(-dQ/dq) = dQ_dq_stiff + c1 * dQ_dqdot_damp` for pilot hubs.
    fn eval_pilot_effort_and_tangent(
        &self,
        q: f64,
        q_dot: f64,
        t: f64,
        c1: f64,
        dt_ref: f64,
    ) -> (f64, f64) {
        let mut q_eff = 0.0_f64;
        let mut k_tang = 0.0_f64;
        let inertia = self.body_inertia_est.max(1e-9);
        let dt_safe = dt_ref.max(1e-6);

        let limits = match self.kind {
            ResolvedDynamicHubKind::Revolute {
                drive:
                    DynamicRevoluteDrive::Motor {
                        target_velocity_rad_s,
                        max_torque_nm,
                        velocity_gain_nm_s_per_rad,
                        ramp_duration_s,
                        limits,
                    },
                ..
            } => {
                let v_ref = eval_ramped_motor_velocity(t, target_velocity_rad_s, ramp_duration_s);
                let e_max = max_torque_nm.max(0.0);
                if e_max > 1e-15 {
                    let c_m = if velocity_gain_nm_s_per_rad > 0.0 {
                        velocity_gain_nm_s_per_rad
                    } else {
                        let g_inertia = inertia / (2.0 * dt_safe).max(5e-4);
                        let g_torque = e_max / (0.12 * target_velocity_rad_s.abs()).max(0.25);
                        g_inertia.max(g_torque)
                    };
                    let err = v_ref - q_dot;
                    let ratio = (c_m * err) / e_max;
                    let s = (1.0 + ratio * ratio).sqrt();
                    q_eff += (c_m * err) / s;
                    // Exact analytical derivative d(-Q)/d(q_dot) = c_m / (1 + ratio^2)^(3/2) > 0;
                    // HHT-alpha inertial mass c0 * I_zz guarantees positive definiteness even at saturation.
                    let d_exact = c_m / (s * s * s);
                    k_tang += c1 * d_exact;
                }
                limits
            }
            ResolvedDynamicHubKind::Revolute {
                drive:
                    DynamicRevoluteDrive::FreeTorque {
                        applied_torque_nm,
                        viscous_damping_nm_s_per_rad,
                        torsional_stiffness_nm_per_rad,
                        reference_angle_rad,
                        limits,
                    },
                ..
            } => {
                let c_d = viscous_damping_nm_s_per_rad.max(0.0);
                let k_s = torsional_stiffness_nm_per_rad.max(0.0);
                q_eff += applied_torque_nm - c_d * q_dot - k_s * (q - reference_angle_rad);
                k_tang += c1 * c_d + k_s;
                limits
            }
            ResolvedDynamicHubKind::Prismatic {
                drive:
                    DynamicPrismaticDrive::Motor {
                        target_velocity_m_s,
                        max_force_n,
                        velocity_gain_n_s_per_m,
                        ramp_duration_s,
                        limits,
                    },
                ..
            } => {
                let v_ref = eval_ramped_motor_velocity(t, target_velocity_m_s, ramp_duration_s);
                let e_max = max_force_n.max(0.0);
                if e_max > 1e-15 {
                    let c_m = if velocity_gain_n_s_per_m > 0.0 {
                        velocity_gain_n_s_per_m
                    } else {
                        let g_mass = inertia / (2.0 * dt_safe).max(5e-4);
                        let g_force = e_max / (0.12 * target_velocity_m_s.abs()).max(0.05);
                        g_mass.max(g_force)
                    };
                    let err = v_ref - q_dot;
                    let ratio = (c_m * err) / e_max;
                    let s = (1.0 + ratio * ratio).sqrt();
                    q_eff += (c_m * err) / s;
                    let d_exact = c_m / (s * s * s);
                    k_tang += c1 * d_exact;
                }
                limits
            }
            ResolvedDynamicHubKind::Prismatic {
                drive:
                    DynamicPrismaticDrive::FreeForce {
                        applied_force_n,
                        viscous_damping_n_s_per_m,
                        axial_stiffness_n_per_m,
                        reference_disp_m,
                        limits,
                    },
                ..
            } => {
                let c_d = viscous_damping_n_s_per_m.max(0.0);
                let k_s = axial_stiffness_n_per_m.max(0.0);
                q_eff += applied_force_n - c_d * q_dot - k_s * (q - reference_disp_m);
                k_tang += c1 * c_d + k_s;
                limits
            }
            _ => None,
        };

        if let Some([lo, hi]) = limits {
            let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
            let k_lim = (1e3 * inertia / (dt_safe * dt_safe)).max(1e4);
            if q < lo {
                q_eff += k_lim * (lo - q);
                k_tang += k_lim;
            } else if q > hi {
                q_eff += k_lim * (hi - q);
                k_tang += k_lim;
            }
        }

        (q_eff, k_tang)
    }

    /// Applies exact hub kinematics `(q, q_dot, q_ddot)` onto this hub's coupled nodes in `u, v, a`.
    #[allow(clippy::too_many_arguments)]
    fn apply_kinematics_to_nodes(
        &self,
        mesh: &TetrahedralMesh,
        active_nodes: &[bool],
        q: f64,
        q_dot: f64,
        q_ddot: f64,
        u: &mut [f64],
        v: &mut [f64],
        a: &mut [f64],
    ) {
        match self.kind {
            ResolvedDynamicHubKind::Revolute { pivot, axis, .. } => {
                let p0 = Vector3::from(pivot);
                let ax = Vector3::from(axis);
                for &nid in &self.node_ids {
                    if !active_nodes.get(nid).copied().unwrap_or(false) {
                        continue;
                    }
                    let r0 = Vector3::from(mesh.nodes[nid].position) - p0;
                    let r_rot = rodrigues_rotate(ax, q, r0);
                    let u_i = r_rot - r0;
                    let t_i = ax.cross(&r_rot);
                    let v_i = t_i * q_dot;
                    let a_i = t_i * q_ddot + ax.cross(&t_i) * (q_dot * q_dot);
                    let d0 = 3 * nid;
                    u[d0] = u_i.x;
                    u[d0 + 1] = u_i.y;
                    u[d0 + 2] = u_i.z;
                    v[d0] = v_i.x;
                    v[d0 + 1] = v_i.y;
                    v[d0 + 2] = v_i.z;
                    a[d0] = a_i.x;
                    a[d0 + 1] = a_i.y;
                    a[d0 + 2] = a_i.z;
                }
            }
            ResolvedDynamicHubKind::Prismatic {
                axis,
                lock_transverse,
                ..
            } => {
                if !lock_transverse {
                    if let Some((comp, sign)) = aligned_coordinate_axis(axis) {
                        for &nid in &self.node_ids {
                            if !active_nodes.get(nid).copied().unwrap_or(false) {
                                continue;
                            }
                            let d = 3 * nid + comp;
                            u[d] = sign * q;
                            v[d] = sign * q_dot;
                            a[d] = sign * q_ddot;
                        }
                        return;
                    }
                }
                for &nid in &self.node_ids {
                    if !active_nodes.get(nid).copied().unwrap_or(false) {
                        continue;
                    }
                    let d0 = 3 * nid;
                    for (comp, &ax_c) in axis.iter().enumerate() {
                        u[d0 + comp] = ax_c * q;
                        v[d0 + comp] = ax_c * q_dot;
                        a[d0 + comp] = ax_c * q_ddot;
                    }
                }
            }
            ResolvedDynamicHubKind::Fixed => {
                for &nid in &self.node_ids {
                    if !active_nodes.get(nid).copied().unwrap_or(false) {
                        continue;
                    }
                    let d0 = 3 * nid;
                    u[d0] = 0.0;
                    u[d0 + 1] = 0.0;
                    u[d0 + 2] = 0.0;
                    v[d0] = 0.0;
                    v[d0 + 1] = 0.0;
                    v[d0 + 2] = 0.0;
                    a[d0] = 0.0;
                    a[d0 + 1] = 0.0;
                    a[d0 + 2] = 0.0;
                }
            }
        }
    }
}

/// Reduced-system mapping for each full nodal DOF in HHT-α (`P5-T14`).
#[derive(Debug, Clone, Copy)]
enum DynDofMap {
    /// Independent free nodal DOF mapped with unit weight.
    Free(usize),
    /// Hub node DOF kinematically coupled to a scalar pilot DOF with tangent weight `weight`.
    Coupled { reduced_idx: usize, weight: f64 },
    /// Fixed or prescribed-motion DOF (eliminated from the linear solve).
    Fixed,
}

impl DynDofMap {
    #[inline]
    fn reduced_and_weight(self) -> Option<(usize, f64)> {
        match self {
            Self::Free(idx) => Some((idx, 1.0)),
            Self::Coupled {
                reduced_idx,
                weight,
            } => Some((reduced_idx, weight)),
            Self::Fixed => None,
        }
    }
}

struct BodyRuntime {
    part_id: String,
    mesh: TetrahedralMesh,
    elements: DynamicElements,
    element_volumes: Vec<f64>,
    material: J2MaterialParams,
    density_kg_m3: f64,
    boundary: ResolvedBoundary,
    hubs: Vec<ResolvedDynamicHub>,
    gravity: [f64; 3],
    force_curve: TimeForceCurve,
    damage_params: Option<DamageMaterialParams>,
    anchored_nodes: HashSet<usize>,
    surface: DeformedSurfaceMesh,
    /// Full-DOF mass matrix $\mathbf{M}$ (rebuilt only when topology/erosion changes).
    mass_matrix: CsrMatrix,
    /// Full-DOF initial elastic stiffness $\mathbf{K}_0$ (assembled only when $c_K > 0$).
    rayleigh_k0: Option<CsrMatrix>,
    /// Nodal gravity body force vector $\mathbf{M} \mathbf{g}$ (full DOFs).
    gravity_force: Vec<f64>,
    // Time-marching state (full DOFs)
    u: Vec<f64>,
    v: Vec<f64>,
    a: Vec<f64>,
    f_int_prev: Vec<f64>,
    f_ext_prev: Vec<f64>,
    gp_states: Vec<GaussPointState>,
    gp_rotations: Vec<[[f64; 3]; 3]>,
    damage_states: Vec<ElementDamageState>,
    active_elements: Vec<bool>,
    active_nodes: Vec<bool>,
    eroded_elements: Vec<usize>,
    plastic_yield_logged: bool,
    crack_init_logged: bool,
    cumulative_fracture_energy_j: f64,
    next_fragment_counter: usize,
}

#[derive(Clone)]
struct BodyCheckpoint {
    u: Vec<f64>,
    v: Vec<f64>,
    a: Vec<f64>,
    f_int_prev: Vec<f64>,
    f_ext_prev: Vec<f64>,
    gp_states: Vec<GaussPointState>,
    gp_rotations: Vec<[[f64; 3]; 3]>,
    damage_states: Vec<ElementDamageState>,
    active_elements: Vec<bool>,
    active_nodes: Vec<bool>,
    eroded_elements: Vec<usize>,
    cumulative_fracture_energy_j: f64,
    hub_states: Vec<(f64, f64, f64, f64)>,
}

impl BodyRuntime {
    fn checkpoint(&self) -> BodyCheckpoint {
        BodyCheckpoint {
            u: self.u.clone(),
            v: self.v.clone(),
            a: self.a.clone(),
            f_int_prev: self.f_int_prev.clone(),
            f_ext_prev: self.f_ext_prev.clone(),
            gp_states: self.gp_states.clone(),
            gp_rotations: self.gp_rotations.clone(),
            damage_states: self.damage_states.clone(),
            active_elements: self.active_elements.clone(),
            active_nodes: self.active_nodes.clone(),
            eroded_elements: self.eroded_elements.clone(),
            cumulative_fracture_energy_j: self.cumulative_fracture_energy_j,
            hub_states: self
                .hubs
                .iter()
                .map(|h| (h.q, h.q_dot, h.q_ddot, h.q_effort_prev))
                .collect(),
        }
    }

    fn restore(&mut self, cp: &BodyCheckpoint, mass_kind: MassMatrixKind) {
        let topo_changed = self.active_elements != cp.active_elements;
        self.u = cp.u.clone();
        self.v = cp.v.clone();
        self.a = cp.a.clone();
        self.f_int_prev = cp.f_int_prev.clone();
        self.f_ext_prev = cp.f_ext_prev.clone();
        self.gp_states = cp.gp_states.clone();
        self.gp_rotations = cp.gp_rotations.clone();
        self.damage_states = cp.damage_states.clone();
        self.active_elements = cp.active_elements.clone();
        self.active_nodes = cp.active_nodes.clone();
        self.eroded_elements = cp.eroded_elements.clone();
        self.cumulative_fracture_energy_j = cp.cumulative_fracture_energy_j;
        for (h, &(q, qd, qdd, qe)) in self.hubs.iter_mut().zip(cp.hub_states.iter()) {
            h.q = q;
            h.q_dot = qd;
            h.q_ddot = qdd;
            h.q_effort_prev = qe;
        }
        if topo_changed {
            self.rebuild_mass_and_gravity(mass_kind);
        }
        self.surface.update_from_displacements(&self.mesh, &self.u);
    }

    fn update_hub_inertia_estimates(&mut self) {
        let n = self.mesh.node_count();
        for hub in &mut self.hubs {
            match hub.kind {
                ResolvedDynamicHubKind::Revolute { pivot, axis, .. } => {
                    let p0 = Vector3::from(pivot);
                    let ax = Vector3::from(axis);
                    let mut t_vec = vec![0.0; n * 3];
                    for i in 0..n {
                        if !self.active_nodes[i] {
                            continue;
                        }
                        let r0 = Vector3::from(self.mesh.nodes[i].position) - p0;
                        let t_i = ax.cross(&r0);
                        t_vec[3 * i] = t_i.x;
                        t_vec[3 * i + 1] = t_i.y;
                        t_vec[3 * i + 2] = t_i.z;
                    }
                    let mut mt = vec![0.0; n * 3];
                    self.mass_matrix.spmv(&t_vec, &mut mt);
                    hub.body_inertia_est = dot_slice(&t_vec, &mt).max(1e-9);
                }
                ResolvedDynamicHubKind::Prismatic { axis, .. } => {
                    let mut e_vec = vec![0.0; n * 3];
                    for i in 0..n {
                        if !self.active_nodes[i] {
                            continue;
                        }
                        e_vec[3 * i] = axis[0];
                        e_vec[3 * i + 1] = axis[1];
                        e_vec[3 * i + 2] = axis[2];
                    }
                    let mut me = vec![0.0; n * 3];
                    self.mass_matrix.spmv(&e_vec, &mut me);
                    hub.body_inertia_est = dot_slice(&e_vec, &me).max(1e-9);
                }
                ResolvedDynamicHubKind::Fixed => {
                    hub.body_inertia_est = 1.0;
                }
            }
        }
    }

    fn recompute_active_nodes(&mut self) {
        let mut mask = vec![false; self.mesh.node_count()];
        if self.mesh.is_tet10() {
            for (e_idx, nodes) in self.mesh.tet10_elements.iter().enumerate() {
                if self.active_elements[e_idx] {
                    for &nid in nodes {
                        mask[nid] = true;
                    }
                }
            }
        } else {
            for (e_idx, elem) in self.mesh.elements.iter().enumerate() {
                if self.active_elements[e_idx] {
                    for &nid in &elem.node_indices {
                        mask[nid] = true;
                    }
                }
            }
        }
        self.active_nodes = mask;
    }

    fn rebuild_mass_and_gravity(&mut self, mass_kind: MassMatrixKind) {
        self.mass_matrix = self.elements.assemble_mass(
            self.mesh.node_count(),
            self.density_kg_m3,
            mass_kind,
            &self.active_elements,
        );
        let n = self.mesh.node_count();
        let mut g_vec = vec![0.0; n * 3];
        if self.gravity[0] != 0.0 || self.gravity[1] != 0.0 || self.gravity[2] != 0.0 {
            for i in 0..n {
                g_vec[3 * i] = self.gravity[0];
                g_vec[3 * i + 1] = self.gravity[1];
                g_vec[3 * i + 2] = self.gravity[2];
            }
            let mut fg = vec![0.0; n * 3];
            self.mass_matrix.spmv(&g_vec, &mut fg);
            self.gravity_force = fg;
        } else {
            self.gravity_force = vec![0.0; n * 3];
        }
    }

    fn external_force_at(&self, time_s: f64) -> Vec<f64> {
        let n = self.mesh.node_count();
        let scale = self.force_curve.evaluate(time_s);
        let mut f = self.gravity_force.clone();
        for i in 0..n {
            if !self.active_nodes[i] {
                f[3 * i] = 0.0;
                f[3 * i + 1] = 0.0;
                f[3 * i + 2] = 0.0;
                continue;
            }
            let nf = self.boundary.nodal_forces.get(i).copied().unwrap_or([0.0; 3]);
            f[3 * i] += scale * nf[0];
            f[3 * i + 1] += scale * nf[1];
            f[3 * i + 2] += scale * nf[2];
        }
        f
    }

    fn effective_material(&self, enable_plasticity: bool) -> J2MaterialParams {
        if enable_plasticity {
            self.material
        } else {
            // Pure elastic response: raise yield strength to infinity while keeping E, nu exact.
            let mut m = self.material;
            m.yield_strength_pa = 1.0e30;
            m
        }
    }

    fn damping_force(&self, v: &[f64], opts: &HhtOptions) -> Vec<f64> {
        let n_dofs = v.len();
        let mut c_v = vec![0.0; n_dofs];
        if opts.rayleigh_mass_alpha.abs() > 1e-18 {
            let mut mv = vec![0.0; n_dofs];
            self.mass_matrix.spmv(v, &mut mv);
            for i in 0..n_dofs {
                c_v[i] += opts.rayleigh_mass_alpha * mv[i];
            }
        }
        if opts.rayleigh_stiffness_beta.abs() > 1e-18 {
            if let Some(ref k0) = self.rayleigh_k0 {
                let mut kv = vec![0.0; n_dofs];
                k0.spmv(v, &mut kv);
                for i in 0..n_dofs {
                    c_v[i] += opts.rayleigh_stiffness_beta * kv[i];
                }
            }
        }
        c_v
    }

    fn kinetic_energy(&self) -> f64 {
        let mut mv = vec![0.0; self.v.len()];
        self.mass_matrix.spmv(&self.v, &mut mv);
        0.5 * dot_slice(&self.v, &mv)
    }

    fn stiffness_scales(&self) -> Vec<f64> {
        let n_elem = self.elements.element_count();
        let mut scales = vec![1.0; n_elem];
        if let Some(dp) = self.damage_params {
            for (e, ds) in self.damage_states.iter().enumerate() {
                if !self.active_elements[e] || ds.eroded {
                    scales[e] = 0.0;
                } else {
                    scales[e] = effective_stiffness_factor(ds.damage, dp.min_residual_stiffness);
                }
            }
        } else {
            for (e, &act) in self.active_elements.iter().enumerate() {
                if !act {
                    scales[e] = 0.0;
                }
            }
        }
        scales
    }

    fn deformed_positions_slice(&self) -> Vec<[f64; 3]> {
        self.mesh
            .nodes
            .iter()
            .enumerate()
            .map(|(i, node)| {
                let p = node.position;
                [
                    p[0] + self.u[3 * i],
                    p[1] + self.u[3 * i + 1],
                    p[2] + self.u[3 * i + 2],
                ]
            })
            .collect()
    }

    fn nodal_velocities_slice(&self) -> Vec<[f64; 3]> {
        (0..self.mesh.node_count())
            .map(|i| [self.v[3 * i], self.v[3 * i + 1], self.v[3 * i + 2]])
            .collect()
    }

    fn build_frame_snapshot(&self) -> DynamicBodyFrame {
        let n = self.mesh.node_count();
        let mut deformed_positions = Vec::with_capacity(n);
        let mut displacements = Vec::with_capacity(n);
        let mut velocities = Vec::with_capacity(n);
        let mut max_disp = 0.0_f64;

        for i in 0..n {
            let p = self.mesh.nodes[i].position;
            let u = [self.u[3 * i], self.u[3 * i + 1], self.u[3 * i + 2]];
            let v = [self.v[3 * i], self.v[3 * i + 1], self.v[3 * i + 2]];
            if self.active_nodes[i] {
                let mag = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
                max_disp = max_disp.max(mag);
            }
            deformed_positions.push([p[0] + u[0], p[1] + u[1], p[2] + u[2]]);
            displacements.push(u);
            velocities.push(v);
        }

        // Volume-weighted nodal averaging of von Mises stress, equivalent plastic strain, and damage
        let mut vm_num = vec![0.0; n];
        let mut ep_num = vec![0.0; n];
        let mut dmg_num = vec![0.0; n];
        let mut vol_den = vec![0.0; n];

        let mut max_vm = 0.0_f64;
        let mut max_ep = 0.0_f64;
        let mut max_dmg = 0.0_f64;
        let mut active_elem_count = 0usize;

        match &self.elements {
            DynamicElements::Tet4(elems) => {
                for (e_idx, elem) in elems.iter().enumerate() {
                    let dmg = self
                        .damage_states
                        .get(e_idx)
                        .map(|d| d.damage)
                        .unwrap_or(0.0);
                    max_dmg = max_dmg.max(dmg);
                    if !self.active_elements[e_idx] {
                        continue;
                    }
                    active_elem_count += 1;
                    let gp = &self.gp_states[e_idx];
                    let stress_v = Vector6::from(gp.stress);
                    let vm = von_mises_from_voigt(&stress_v) * (1.0 - dmg).max(0.0);
                    let ep = gp.equivalent_plastic_strain;
                    max_vm = max_vm.max(vm);
                    max_ep = max_ep.max(ep);
                    let w = elem.volume;
                    for &nid in &elem.node_indices {
                        vm_num[nid] += vm * w;
                        ep_num[nid] += ep * w;
                        dmg_num[nid] += dmg * w;
                        vol_den[nid] += w;
                    }
                }
            }
            DynamicElements::Tet10(elems) => {
                for (e_idx, elem) in elems.iter().enumerate() {
                    let dmg = self
                        .damage_states
                        .get(e_idx)
                        .map(|d| d.damage)
                        .unwrap_or(0.0);
                    max_dmg = max_dmg.max(dmg);
                    if !self.active_elements[e_idx] {
                        continue;
                    }
                    active_elem_count += 1;
                    let base = e_idx * 4;
                    let mut vm_e = 0.0;
                    let mut ep_e = 0.0;
                    for q in 0..4 {
                        let gp = &self.gp_states[base + q];
                        let stress_v = Vector6::from(gp.stress);
                        let vm_q = von_mises_from_voigt(&stress_v) * (1.0 - dmg).max(0.0);
                        vm_e += 0.25 * vm_q;
                        ep_e += 0.25 * gp.equivalent_plastic_strain;
                        max_vm = max_vm.max(vm_q);
                        max_ep = max_ep.max(gp.equivalent_plastic_strain);
                    }
                    let w = self.element_volumes[e_idx];
                    for &nid in &elem.node_indices {
                        vm_num[nid] += vm_e * w;
                        ep_num[nid] += ep_e * w;
                        dmg_num[nid] += dmg * w;
                        vol_den[nid] += w;
                    }
                }
            }
        }

        let mut nodal_von_mises_pa = vec![0.0; n];
        let mut nodal_equiv_plastic_strain = vec![0.0; n];
        let mut nodal_damage = vec![0.0; n];
        for i in 0..n {
            if vol_den[i] > 1e-24 {
                nodal_von_mises_pa[i] = vm_num[i] / vol_den[i];
                nodal_equiv_plastic_strain[i] = ep_num[i] / vol_den[i];
                nodal_damage[i] = dmg_num[i] / vol_den[i];
            }
        }

        DynamicBodyFrame {
            part_id: self.part_id.clone(),
            node_count: n,
            deformed_positions,
            displacements,
            velocities,
            nodal_von_mises_pa,
            nodal_equiv_plastic_strain,
            nodal_damage,
            eroded_element_indices: self.eroded_elements.clone(),
            active_element_count: active_elem_count,
            max_displacement_m: max_disp,
            max_von_mises_pa: max_vm,
            max_equiv_plastic_strain: max_ep,
            max_damage: max_dmg,
        }
    }
}

#[inline]
fn dot_slice(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(&x, &y)| x * y).sum()
}

#[inline]
fn norm_slice(a: &[f64]) -> f64 {
    dot_slice(a, a).sqrt()
}

#[inline]
fn matrix3_to_rows(m: &Matrix3<f64>) -> [[f64; 3]; 3] {
    [
        [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
        [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
        [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
    ]
}

/// Evaluates element internal forces and optionally tangent stiffness for one body (parallelized via `rayon`).
#[allow(clippy::type_complexity)]
fn evaluate_body_continuum(
    body: &BodyRuntime,
    u_trial: &[f64],
    opts: &HhtOptions,
    assemble_tangent: bool,
    dof_map: &[DynDofMap],
    tangent_scale: f64,
    row_entries: &mut [Vec<(usize, f64)>],
) -> (Vec<f64>, Vec<GaussPointState>, Vec<[[f64; 3]; 3]>, f64, f64) {
    let n_dofs = body.mesh.node_count() * 3;
    let mut f_int = vec![0.0; n_dofs];
    let mat = body.effective_material(opts.enable_plasticity);
    let scales = body.stiffness_scales();
    let prev_gp = &body.gp_states;
    let active = &body.active_elements;
    let mut total_we = 0.0_f64;
    let mut total_wp = 0.0_f64;

    match &body.elements {
        DynamicElements::Tet4(elems) => {
            let eval_fn = |(e_idx, elem): (usize, &Tet4Element)| {
                if !active[e_idx] {
                    return (
                        false,
                        0.0_f64,
                        [0usize; 12],
                        Vector12::zeros(),
                        None,
                        prev_gp[e_idx],
                        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                        0.0_f64,
                        0.0_f64,
                    );
                }
                let d_scale = scales[e_idx];
                let mut u_elem = Vector12::zeros();
                let mut elem_dofs = [0usize; 12];
                for (li, &nid) in elem.node_indices.iter().enumerate() {
                    let d0 = 3 * nid;
                    elem_dofs[3 * li] = d0;
                    elem_dofs[3 * li + 1] = d0 + 1;
                    elem_dofs[3 * li + 2] = d0 + 2;
                    u_elem[3 * li] = u_trial[d0];
                    u_elem[3 * li + 1] = u_trial[d0 + 1];
                    u_elem[3 * li + 2] = u_trial[d0 + 2];
                }
                if assemble_tangent {
                    let (f_e, k_e, rm, r_mat) = elem.evaluate_nonlinear(
                        &u_elem,
                        &prev_gp[e_idx],
                        &mat,
                        opts.corotated,
                        opts.include_geometric_stiffness && d_scale >= 0.99,
                    );
                    (
                        true,
                        d_scale,
                        elem_dofs,
                        f_e,
                        Some(k_e),
                        rm.state,
                        matrix3_to_rows(&r_mat),
                        d_scale * rm.elastic_energy_density * elem.volume,
                        rm.state.plastic_energy_density * elem.volume,
                    )
                } else {
                    let (f_e, rm, r_mat) = elem.evaluate_nonlinear_forces_only(
                        &u_elem,
                        &prev_gp[e_idx],
                        &mat,
                        opts.corotated,
                    );
                    (
                        true,
                        d_scale,
                        elem_dofs,
                        f_e,
                        None,
                        rm.state,
                        matrix3_to_rows(&r_mat),
                        d_scale * rm.elastic_energy_density * elem.volume,
                        rm.state.plastic_energy_density * elem.volume,
                    )
                }
            };

            let results: Vec<_> = if elems.len() >= DYNAMICS_PAR_THRESHOLD {
                elems.par_iter().enumerate().map(eval_fn).collect()
            } else {
                elems.iter().enumerate().map(eval_fn).collect()
            };

            let mut new_gp = Vec::with_capacity(elems.len());
            let mut new_rots = Vec::with_capacity(elems.len());
            for (is_act, d_scale, elem_dofs, f_e, k_opt, gp, rot, we, wp) in results {
                new_gp.push(gp);
                new_rots.push(rot);
                if !is_act {
                    continue;
                }
                total_we += we;
                total_wp += wp;
                for a in 0..12 {
                    f_int[elem_dofs[a]] += d_scale * f_e[a];
                }
                if let Some(k_e) = k_opt {
                    let k_mult = tangent_scale * d_scale;
                    for a in 0..12 {
                        if let Some((fa, wa)) = dof_map[elem_dofs[a]].reduced_and_weight() {
                            let row_vec = &mut row_entries[fa];
                            for b in 0..12 {
                                if let Some((fb, wb)) = dof_map[elem_dofs[b]].reduced_and_weight() {
                                    let val = k_mult * wa * wb * k_e[(a, b)];
                                    if val.abs() > 1e-25 {
                                        row_vec.push((fb, val));
                                    }
                                }
                            }
                        }
                    }
                }
            }
            (f_int, new_gp, new_rots, total_we, total_wp)
        }
        DynamicElements::Tet10(elems) => {
            let eval_fn = |(e_idx, elem): (usize, &Tet10Element)| {
                let base = e_idx * 4;
                let eye = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
                if !active[e_idx] {
                    return (
                        false,
                        0.0_f64,
                        [0usize; 30],
                        Vector30::zeros(),
                        None,
                        [
                            prev_gp[base],
                            prev_gp[base + 1],
                            prev_gp[base + 2],
                            prev_gp[base + 3],
                        ],
                        [eye; 4],
                        0.0_f64,
                        0.0_f64,
                    );
                }
                let d_scale = scales[e_idx];
                let mut u_elem = Vector30::zeros();
                let mut elem_dofs = [0usize; 30];
                for (li, &nid) in elem.node_indices.iter().enumerate() {
                    let d0 = 3 * nid;
                    elem_dofs[3 * li] = d0;
                    elem_dofs[3 * li + 1] = d0 + 1;
                    elem_dofs[3 * li + 2] = d0 + 2;
                    u_elem[3 * li] = u_trial[d0];
                    u_elem[3 * li + 1] = u_trial[d0 + 1];
                    u_elem[3 * li + 2] = u_trial[d0 + 2];
                }
                let prev4 = [
                    prev_gp[base],
                    prev_gp[base + 1],
                    prev_gp[base + 2],
                    prev_gp[base + 3],
                ];
                if assemble_tangent {
                    let (f_e, k_e, rms, rots) = elem.evaluate_nonlinear(
                        &u_elem,
                        &prev4,
                        &mat,
                        opts.corotated,
                        opts.include_geometric_stiffness && d_scale >= 0.99,
                    );
                    let mut we = 0.0;
                    let mut wp = 0.0;
                    for (rm, &gw) in rms.iter().zip(elem.gauss_weights.iter()) {
                        we += d_scale * rm.elastic_energy_density * gw;
                        wp += rm.state.plastic_energy_density * gw;
                    }
                    (
                        true,
                        d_scale,
                        elem_dofs,
                        f_e,
                        Some(k_e),
                        [rms[0].state, rms[1].state, rms[2].state, rms[3].state],
                        [
                            matrix3_to_rows(&rots[0]),
                            matrix3_to_rows(&rots[1]),
                            matrix3_to_rows(&rots[2]),
                            matrix3_to_rows(&rots[3]),
                        ],
                        we,
                        wp,
                    )
                } else {
                    let (f_e, rms, rots) = elem.evaluate_nonlinear_forces_only(
                        &u_elem,
                        &prev4,
                        &mat,
                        opts.corotated,
                    );
                    let mut we = 0.0;
                    let mut wp = 0.0;
                    for (rm, &gw) in rms.iter().zip(elem.gauss_weights.iter()) {
                        we += d_scale * rm.elastic_energy_density * gw;
                        wp += rm.state.plastic_energy_density * gw;
                    }
                    (
                        true,
                        d_scale,
                        elem_dofs,
                        f_e,
                        None,
                        [rms[0].state, rms[1].state, rms[2].state, rms[3].state],
                        [
                            matrix3_to_rows(&rots[0]),
                            matrix3_to_rows(&rots[1]),
                            matrix3_to_rows(&rots[2]),
                            matrix3_to_rows(&rots[3]),
                        ],
                        we,
                        wp,
                    )
                }
            };

            let results: Vec<_> = if elems.len() >= DYNAMICS_PAR_THRESHOLD {
                elems.par_iter().enumerate().map(eval_fn).collect()
            } else {
                elems.iter().enumerate().map(eval_fn).collect()
            };

            let mut new_gp = Vec::with_capacity(elems.len() * 4);
            let mut new_rots = Vec::with_capacity(elems.len() * 4);
            for (is_act, d_scale, elem_dofs, f_e, k_opt, gp4, rots4, we, wp) in results {
                for q in 0..4 {
                    new_gp.push(gp4[q]);
                    new_rots.push(rots4[q]);
                }
                if !is_act {
                    continue;
                }
                total_we += we;
                total_wp += wp;
                for a in 0..30 {
                    f_int[elem_dofs[a]] += d_scale * f_e[a];
                }
                if let Some(k_e) = k_opt {
                    let k_mult = tangent_scale * d_scale;
                    for a in 0..30 {
                        if let Some((fa, wa)) = dof_map[elem_dofs[a]].reduced_and_weight() {
                            let row_vec = &mut row_entries[fa];
                            for b in 0..30 {
                                if let Some((fb, wb)) = dof_map[elem_dofs[b]].reduced_and_weight() {
                                    let val = k_mult * wa * wb * k_e[(a, b)];
                                    if val.abs() > 1e-25 {
                                        row_vec.push((fb, val));
                                    }
                                }
                            }
                        }
                    }
                }
            }
            (f_int, new_gp, new_rots, total_we, total_wp)
        }
    }
}

/// Projects point `p` onto an analytical rigid surface, returning `(signed_gap, outward_normal)`.
fn project_onto_rigid(p: [f64; 3], surface: AnalyticalRigidSurface) -> (f64, Vector3<f64>) {
    let vp = Vector3::from(p);
    match surface {
        AnalyticalRigidSurface::Plane { origin, normal } => {
            let n_raw = Vector3::from(normal);
            let n = if n_raw.norm() > 1e-12 {
                n_raw.normalize()
            } else {
                Vector3::new(0.0, 1.0, 0.0)
            };
            let gap = (vp - Vector3::from(origin)).dot(&n);
            (gap, n)
        }
        AnalyticalRigidSurface::Cylinder {
            origin,
            axis,
            radius,
        } => {
            let o = Vector3::from(origin);
            let a_raw = Vector3::from(axis);
            let a = if a_raw.norm() > 1e-12 {
                a_raw.normalize()
            } else {
                Vector3::new(0.0, 0.0, 1.0)
            };
            let rel = vp - o;
            let radial = rel - a * rel.dot(&a);
            let dist = radial.norm();
            let n = if dist > 1e-12 {
                radial / dist
            } else {
                Vector3::new(1.0, 0.0, 0.0)
            };
            (dist - radius, n)
        }
    }
}

/// Evaluates penalty normal + tangential dry-Coulomb contact forces and symmetric PSD contact tangent
/// contributions across all active contact pairs. Contact resisting forces are added directly into
/// `f_int_bodies` (consistent with $\mathbf{R} = \mathbf{f}_{\text{ext}} - \mathbf{M}\mathbf{a} - \mathbf{f}_{\text{resisting}}$).
/// Snapshot of an active/near-active deformable-to-deformable contact master-facet projection,
/// used to freeze facet-edge switching during late Newton polishing iterations (`iter >= 6`)
/// so coarse faceted meshes (e.g. voxel-snapped gear teeth) converge quadratically without
/// active-set 2-cycles (`P5-T14`).
#[derive(Debug, Clone)]
struct FrozenDeformableContact {
    pair_idx: usize,
    slave_body: usize,
    master_body: usize,
    slave_node: usize,
    master_nodes: [usize; 3],
    barycentric: [f64; 3],
    normal: [f64; 3],
    area: f64,
    eps_n: f64,
    eps_t: f64,
    delta_smooth: f64,
    friction_coeff: f64,
}

#[allow(clippy::too_many_arguments)]
fn evaluate_dynamic_contacts(
    bodies: &[BodyRuntime],
    pairs: &[ContactPairSpec],
    u_trial: &[Vec<f64>],
    v_trial: &[Vec<f64>],
    dt: f64,
    c1_vel: f64,
    opts: &HhtOptions,
    dof_maps: &[Vec<DynDofMap>],
    assemble_tangent: bool,
    tangent_scale: f64,
    f_resisting: &mut [Vec<f64>],
    row_entries: &mut [Vec<(usize, f64)>],
    frozen_deformable: Option<&[FrozenDeformableContact]>,
    mut capture_frozen: Option<&mut Vec<FrozenDeformableContact>>,
) -> (Vec<DynamicContactForce>, f64) {
    if pairs.is_empty() {
        return (Vec::new(), 0.0);
    }

    let need_surfaces = frozen_deformable.is_none()
        || pairs
            .iter()
            .any(|p| matches!(p, ContactPairSpec::DeformableToRigid { .. }));

    // Build updated deformed surfaces in parallel across bodies only when fresh BVH projection or rigid contact is needed
    let surfaces: Vec<DeformedSurfaceMesh> = if need_surfaces {
        if bodies.len() >= 2 {
            bodies
                .par_iter()
                .enumerate()
                .map(|(b, body)| {
                    let mut surf = body.surface.clone();
                    surf.update_from_displacements(&body.mesh, &u_trial[b]);
                    surf
                })
                .collect()
        } else {
            bodies
                .iter()
                .enumerate()
                .map(|(b, body)| {
                    let mut surf = body.surface.clone();
                    surf.update_from_displacements(&body.mesh, &u_trial[b]);
                    surf
                })
                .collect()
        }
    } else {
        Vec::new()
    };

    let mut contact_forces = Vec::new();
    let mut contact_strain_energy = 0.0_f64;

    // Fast $C^1$-smooth evaluation of frozen deformable-to-deformable contact pairs (when active)
    if let Some(frozen_list) = frozen_deformable {
        for fc in frozen_list {
            let sb = fc.slave_body;
            let mb = fc.master_body;
            let snid = fc.slave_node;
            if !bodies[sb].active_nodes.get(snid).copied().unwrap_or(false) {
                continue;
            }
            let x0_s = bodies[sb].mesh.nodes[snid].position;
            let p = [
                x0_s[0] + u_trial[sb][3 * snid],
                x0_s[1] + u_trial[sb][3 * snid + 1],
                x0_s[2] + u_trial[sb][3 * snid + 2],
            ];
            let mut x_m = [0.0_f64; 3];
            for k in 0..3 {
                let mn = fc.master_nodes[k];
                let xi = fc.barycentric[k];
                let x0_m = bodies[mb].mesh.nodes[mn].position;
                x_m[0] += xi * (x0_m[0] + u_trial[mb][3 * mn]);
                x_m[1] += xi * (x0_m[1] + u_trial[mb][3 * mn + 1]);
                x_m[2] += xi * (x0_m[2] + u_trial[mb][3 * mn + 2]);
            }
            let n = Vector3::from(fc.normal);
            let gap = (p[0] - x_m[0]) * n.x + (p[1] - x_m[1]) * n.y + (p[2] - x_m[2]) * n.z;
            if gap >= fc.delta_smooth {
                continue;
            }
            let (pen, p_n, k_n_eff, e_c) = if gap < 0.0 {
                let pen = -gap;
                (
                    pen,
                    fc.eps_n * (0.5 * fc.delta_smooth + pen),
                    fc.eps_n,
                    fc.eps_n
                        * fc.area
                        * (0.5 * pen * pen
                            + 0.5 * fc.delta_smooth * pen
                            + fc.delta_smooth * fc.delta_smooth / 6.0),
                )
            } else {
                let s = fc.delta_smooth - gap;
                let ratio = s / fc.delta_smooth;
                (
                    0.0,
                    0.5 * fc.eps_n * s * ratio,
                    fc.eps_n * ratio,
                    fc.eps_n * fc.area * s * s * s / (6.0 * fc.delta_smooth),
                )
            };
            let f_n_mag = p_n * fc.area;
            contact_strain_energy += e_c;

            let v_s = Vector3::new(
                v_trial[sb][3 * snid],
                v_trial[sb][3 * snid + 1],
                v_trial[sb][3 * snid + 2],
            );
            let mut v_m = Vector3::zeros();
            for k in 0..3 {
                let mn = fc.master_nodes[k];
                let xi = fc.barycentric[k];
                v_m += Vector3::new(
                    v_trial[mb][3 * mn],
                    v_trial[mb][3 * mn + 1],
                    v_trial[mb][3 * mn + 2],
                ) * xi;
            }
            let v_rel = v_s - v_m;
            let v_t = v_rel - n * v_rel.dot(&n);
            let mut f_t = Vector3::zeros();
            let mut k_t_mat = Matrix3::zeros();
            if fc.friction_coeff > 1e-12 && f_n_mag > 0.0 {
                let f_t_trial = -v_t * (fc.eps_t * fc.area * dt);
                let tau_mag = f_t_trial.norm();
                let tau_max = fc.friction_coeff * f_n_mag;
                let p_tang = Matrix3::identity() - n * n.transpose();
                if tau_mag <= tau_max {
                    f_t = f_t_trial;
                    k_t_mat = p_tang * (fc.eps_t * fc.area * dt * c1_vel);
                } else if tau_mag > 1e-18 {
                    let d_hat = f_t_trial / tau_mag;
                    f_t = d_hat * tau_max;
                    let p_ortho = p_tang - d_hat * d_hat.transpose();
                    k_t_mat = p_ortho * (fc.eps_t * fc.area * dt * c1_vel * (tau_max / tau_mag));
                }
            }

            let f_c = n * f_n_mag + f_t;
            let sd0 = 3 * snid;
            f_resisting[sb][sd0] -= f_c.x;
            f_resisting[sb][sd0 + 1] -= f_c.y;
            f_resisting[sb][sd0 + 2] -= f_c.z;
            for k in 0..3 {
                let mn = fc.master_nodes[k];
                let xi = fc.barycentric[k];
                let md0 = 3 * mn;
                f_resisting[mb][md0] += xi * f_c.x;
                f_resisting[mb][md0 + 1] += xi * f_c.y;
                f_resisting[mb][md0 + 2] += xi * f_c.z;
            }

            if assemble_tangent {
                let weights = [
                    (sb, snid, 1.0_f64),
                    (mb, fc.master_nodes[0], -fc.barycentric[0]),
                    (mb, fc.master_nodes[1], -fc.barycentric[1]),
                    (mb, fc.master_nodes[2], -fc.barycentric[2]),
                ];
                let k_c = ((n * n.transpose()) * (k_n_eff * fc.area) + k_t_mat) * tangent_scale;
                for &(ba, na, wa) in &weights {
                    if wa.abs() <= 1e-14 {
                        continue;
                    }
                    let da0 = 3 * na;
                    for i in 0..3 {
                        if let Some((fi, w_da)) = dof_maps[ba][da0 + i].reduced_and_weight() {
                            let row_vec = &mut row_entries[fi];
                            for &(bb, nb, wb) in &weights {
                                if wb.abs() <= 1e-14 {
                                    continue;
                                }
                                let db0 = 3 * nb;
                                for j in 0..3 {
                                    if let Some((fj, w_db)) =
                                        dof_maps[bb][db0 + j].reduced_and_weight()
                                    {
                                        let val = wa * wb * w_da * w_db * k_c[(i, j)];
                                        if val.abs() > 1e-25 {
                                            row_vec.push((fj, val));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            contact_forces.push(DynamicContactForce {
                pair_index: fc.pair_idx,
                slave_body: sb,
                master_body: Some(mb),
                point_m: p,
                normal: fc.normal,
                normal_force_n: f_n_mag,
                tangential_force_n: [f_t.x, f_t.y, f_t.z],
                penetration_m: pen,
            });
        }
    }

    for (pair_idx, pair) in pairs.iter().enumerate() {
        match pair {
            ContactPairSpec::DeformableToRigid {
                body_idx,
                surface,
                friction_coeff,
                slave_nodes,
                search_distance,
            } => {
                let b = *body_idx;
                if b >= bodies.len() {
                    continue;
                }
                let surf = &surfaces[b];
                let h = surf.char_edge_len.max(1e-5);
                let eps_n = opts.contact_penalty_scale * bodies[b].material.youngs_modulus_pa / h;
                let eps_t = 0.05 * eps_n;
                let max_search = search_distance.unwrap_or(2.0 * h);
                let candidates: Vec<usize> = match slave_nodes {
                    Some(list) => list.clone(),
                    None => surf.surface_nodes.clone(),
                };

                let delta_smooth = 1e-3 * h;
                for nid in candidates {
                    if !bodies[b].active_nodes.get(nid).copied().unwrap_or(false) {
                        continue;
                    }
                    let p = surf.current_positions[nid];
                    let (gap, n) = project_onto_rigid(p, *surface);
                    if gap >= delta_smooth || gap.abs() > max_search {
                        continue;
                    }
                    let area = surf.contact_nodal_area(nid, n).max(h * h * 1e-3);
                    // C1-continuous quadratic-to-linear penalty regularization eliminating active-set chatter
                    let (pen, p_n, k_n_eff, e_c) = if gap < 0.0 {
                        let pen = -gap;
                        (
                            pen,
                            eps_n * (0.5 * delta_smooth + pen),
                            eps_n,
                            eps_n * area * (0.5 * pen * pen + 0.5 * delta_smooth * pen + delta_smooth * delta_smooth / 6.0),
                        )
                    } else {
                        let s = delta_smooth - gap;
                        let ratio = s / delta_smooth;
                        (
                            0.0,
                            0.5 * eps_n * s * ratio,
                            eps_n * ratio,
                            eps_n * area * s * s * s / (6.0 * delta_smooth),
                        )
                    };
                    let f_n_mag = p_n * area;
                    contact_strain_energy += e_c;

                    // Tangential friction resisting relative slip + consistent symmetric PSD tangent
                    let v_node = Vector3::new(
                        v_trial[b][3 * nid],
                        v_trial[b][3 * nid + 1],
                        v_trial[b][3 * nid + 2],
                    );
                    let v_t = v_node - n * v_node.dot(&n);
                    let mut f_t = Vector3::zeros();
                    let mut k_t_mat = Matrix3::zeros();
                    if *friction_coeff > 1e-12 && f_n_mag > 0.0 {
                        let f_t_trial = -v_t * (eps_t * area * dt);
                        let tau_mag = f_t_trial.norm();
                        let tau_max = friction_coeff * f_n_mag;
                        let p_tang = Matrix3::identity() - n * n.transpose();
                        if tau_mag <= tau_max {
                            f_t = f_t_trial;
                            k_t_mat = p_tang * (eps_t * area * dt * c1_vel);
                        } else if tau_mag > 1e-18 {
                            let d_hat = f_t_trial / tau_mag;
                            f_t = d_hat * tau_max;
                            let p_ortho = p_tang - d_hat * d_hat.transpose();
                            k_t_mat = p_ortho * (eps_t * area * dt * c1_vel * (tau_max / tau_mag));
                        }
                    }

                    // Contact pushes slave along +n => resisting force on slave = -(f_n * n + f_t)
                    let f_contact_on_slave = n * f_n_mag + f_t;
                    let d0 = 3 * nid;
                    f_resisting[b][d0] -= f_contact_on_slave.x;
                    f_resisting[b][d0 + 1] -= f_contact_on_slave.y;
                    f_resisting[b][d0 + 2] -= f_contact_on_slave.z;

                    if assemble_tangent {
                        let k_c = ((n * n.transpose()) * (k_n_eff * area) + k_t_mat) * tangent_scale;
                        for i in 0..3 {
                            if let Some((fi, wi)) = dof_maps[b][d0 + i].reduced_and_weight() {
                                let row_vec = &mut row_entries[fi];
                                for j in 0..3 {
                                    if let Some((fj, wj)) = dof_maps[b][d0 + j].reduced_and_weight() {
                                        let val = wi * wj * k_c[(i, j)];
                                        if val.abs() > 1e-25 {
                                            row_vec.push((fj, val));
                                        }
                                    }
                                }
                            }
                        }
                    }

                    contact_forces.push(DynamicContactForce {
                        pair_index: pair_idx,
                        slave_body: b,
                        master_body: None,
                        point_m: p,
                        normal: [n.x, n.y, n.z],
                        normal_force_n: f_n_mag,
                        tangential_force_n: [f_t.x, f_t.y, f_t.z],
                        penetration_m: pen,
                    });
                }
            }
            ContactPairSpec::DeformablePair {
                body_a,
                body_b,
                friction_coeff,
                symmetric,
                slave_a_nodes,
                slave_b_nodes,
                search_distance,
                max_normal_dot,
            } => {
                if frozen_deformable.is_some()
                    || *body_a >= bodies.len()
                    || *body_b >= bodies.len()
                    || body_a == body_b
                {
                    continue;
                }
                let pass_weight = if *symmetric { 0.5 } else { 1.0 };
                let passes: Vec<(usize, usize, Option<&Vec<usize>>)> = if *symmetric {
                    vec![
                        (*body_a, *body_b, slave_a_nodes.as_ref()),
                        (*body_b, *body_a, slave_b_nodes.as_ref()),
                    ]
                } else {
                    vec![(*body_a, *body_b, slave_a_nodes.as_ref())]
                };

                let e_eff = bodies[*body_a]
                    .material
                    .youngs_modulus_pa
                    .min(bodies[*body_b].material.youngs_modulus_pa);

                for (sb, mb, slave_filter) in passes {
                    let s_surf = &surfaces[sb];
                    let m_surf = &surfaces[mb];
                    let h = s_surf.char_edge_len.min(m_surf.char_edge_len).max(1e-5);
                    let eps_n = opts.contact_penalty_scale * e_eff / h;
                    let eps_t = 0.05 * eps_n;
                    let max_search = search_distance.unwrap_or(2.0 * h);
                    let max_ndot = max_normal_dot.unwrap_or(0.35);

                    let candidates: Vec<usize> = match slave_filter {
                        Some(list) => list.clone(),
                        None => s_surf.surface_nodes.clone(),
                    };

                    let delta_smooth = 1e-3 * h;
                    let active_nodes_sb = &bodies[sb].active_nodes;
                    let proj_hits: Vec<(usize, [f64; 3], crate::contact::SurfaceContactProjection, f64)> =
                        if candidates.len() >= 128 {
                            candidates
                                .par_iter()
                                .filter_map(|&snid| {
                                    if !active_nodes_sb.get(snid).copied().unwrap_or(false) {
                                        return None;
                                    }
                                    let p = s_surf.current_positions[snid];
                                    let s_norm = s_surf.vertex_normals.get(snid).copied();
                                    let proj = m_surf.closest_point_signed(
                                        p,
                                        s_norm,
                                        max_search,
                                        max_ndot,
                                    )?;
                                    if proj.signed_gap >= 2.0 * delta_smooth {
                                        return None;
                                    }
                                    let n = Vector3::from(proj.normal);
                                    let area = (s_surf.contact_nodal_area(snid, n) * pass_weight)
                                        .max(h * h * 1e-4);
                                    Some((snid, p, proj, area))
                                })
                                .collect()
                        } else {
                            candidates
                                .iter()
                                .filter_map(|&snid| {
                                    if !active_nodes_sb.get(snid).copied().unwrap_or(false) {
                                        return None;
                                    }
                                    let p = s_surf.current_positions[snid];
                                    let s_norm = s_surf.vertex_normals.get(snid).copied();
                                    let proj = m_surf.closest_point_signed(
                                        p,
                                        s_norm,
                                        max_search,
                                        max_ndot,
                                    )?;
                                    if proj.signed_gap >= 2.0 * delta_smooth {
                                        return None;
                                    }
                                    let n = Vector3::from(proj.normal);
                                    let area = (s_surf.contact_nodal_area(snid, n) * pass_weight)
                                        .max(h * h * 1e-4);
                                    Some((snid, p, proj, area))
                                })
                                .collect()
                        };

                    for (snid, p, proj, area) in proj_hits {
                        let n = Vector3::from(proj.normal);
                        if proj.signed_gap < 2.0 * delta_smooth {
                            if let Some(ref mut buf) = capture_frozen {
                                buf.push(FrozenDeformableContact {
                                    pair_idx,
                                    slave_body: sb,
                                    master_body: mb,
                                    slave_node: snid,
                                    master_nodes: proj.master_nodes,
                                    barycentric: proj.barycentric,
                                    normal: proj.normal,
                                    area,
                                    eps_n,
                                    eps_t,
                                    delta_smooth,
                                    friction_coeff: *friction_coeff,
                                });
                            }
                        }
                        if proj.signed_gap >= delta_smooth {
                            continue;
                        }
                        let gap = proj.signed_gap;
                        let (pen, p_n, k_n_eff, e_c) = if gap < 0.0 {
                            let pen = -gap;
                            (
                                pen,
                                eps_n * (0.5 * delta_smooth + pen),
                                eps_n,
                                eps_n * area * (0.5 * pen * pen + 0.5 * delta_smooth * pen + delta_smooth * delta_smooth / 6.0),
                            )
                        } else {
                            let s = delta_smooth - gap;
                            let ratio = s / delta_smooth;
                            (
                                0.0,
                                0.5 * eps_n * s * ratio,
                                eps_n * ratio,
                                eps_n * area * s * s * s / (6.0 * delta_smooth),
                            )
                        };
                        let f_n_mag = p_n * area;
                        contact_strain_energy += e_c;

                        // Relative tangential velocity at contact point
                        let v_s = Vector3::new(
                            v_trial[sb][3 * snid],
                            v_trial[sb][3 * snid + 1],
                            v_trial[sb][3 * snid + 2],
                        );
                        let mut v_m = Vector3::zeros();
                        for k in 0..3 {
                            let mn = proj.master_nodes[k];
                            let xi = proj.barycentric[k];
                            v_m += Vector3::new(
                                v_trial[mb][3 * mn],
                                v_trial[mb][3 * mn + 1],
                                v_trial[mb][3 * mn + 2],
                            ) * xi;
                        }
                        let v_rel = v_s - v_m;
                        let v_t = v_rel - n * v_rel.dot(&n);
                        let mut f_t = Vector3::zeros();
                        let mut k_t_mat = Matrix3::zeros();
                        if *friction_coeff > 1e-12 && f_n_mag > 0.0 {
                            let f_t_trial = -v_t * (eps_t * area * dt);
                            let tau_mag = f_t_trial.norm();
                            let tau_max = friction_coeff * f_n_mag;
                            let p_tang = Matrix3::identity() - n * n.transpose();
                            if tau_mag <= tau_max {
                                f_t = f_t_trial;
                                k_t_mat = p_tang * (eps_t * area * dt * c1_vel);
                            } else if tau_mag > 1e-18 {
                                let d_hat = f_t_trial / tau_mag;
                                f_t = d_hat * tau_max;
                                let p_ortho = p_tang - d_hat * d_hat.transpose();
                                k_t_mat =
                                    p_ortho * (eps_t * area * dt * c1_vel * (tau_max / tau_mag));
                            }
                        }

                        // Action-reaction: +f_c on slave, -xi_k * f_c on master nodes
                        let f_c = n * f_n_mag + f_t;
                        let sd0 = 3 * snid;
                        f_resisting[sb][sd0] -= f_c.x;
                        f_resisting[sb][sd0 + 1] -= f_c.y;
                        f_resisting[sb][sd0 + 2] -= f_c.z;
                        for k in 0..3 {
                            let mn = proj.master_nodes[k];
                            let xi = proj.barycentric[k];
                            let md0 = 3 * mn;
                            f_resisting[mb][md0] += xi * f_c.x;
                            f_resisting[mb][md0 + 1] += xi * f_c.y;
                            f_resisting[mb][md0 + 2] += xi * f_c.z;
                        }

                        if assemble_tangent {
                            let weights = [
                                (sb, snid, 1.0_f64),
                                (mb, proj.master_nodes[0], -proj.barycentric[0]),
                                (mb, proj.master_nodes[1], -proj.barycentric[1]),
                                (mb, proj.master_nodes[2], -proj.barycentric[2]),
                            ];
                            let k_c =
                                ((n * n.transpose()) * (k_n_eff * area) + k_t_mat) * tangent_scale;
                            for &(ba, na, wa) in &weights {
                                if wa.abs() <= 1e-14 {
                                    continue;
                                }
                                let da0 = 3 * na;
                                for i in 0..3 {
                                    if let Some((fi, w_da)) =
                                        dof_maps[ba][da0 + i].reduced_and_weight()
                                    {
                                        let row_vec = &mut row_entries[fi];
                                        for &(bb, nb, wb) in &weights {
                                            if wb.abs() <= 1e-14 {
                                                continue;
                                            }
                                            let db0 = 3 * nb;
                                            for j in 0..3 {
                                                if let Some((fj, w_db)) =
                                                    dof_maps[bb][db0 + j].reduced_and_weight()
                                                {
                                                    let val =
                                                        wa * wb * w_da * w_db * k_c[(i, j)];
                                                    if val.abs() > 1e-25 {
                                                        row_vec.push((fj, val));
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        contact_forces.push(DynamicContactForce {
                            pair_index: pair_idx,
                            slave_body: sb,
                            master_body: Some(mb),
                            point_m: p,
                            normal: [n.x, n.y, n.z],
                            normal_force_n: f_n_mag,
                            tangential_force_n: [f_t.x, f_t.y, f_t.z],
                            penetration_m: pen,
                        });
                    }
                }
            }
        }
    }

    (contact_forces, contact_strain_energy)
}

/// Unified reduced DOF system mapping full nodal DOFs to independent free nodal DOFs
/// (`0..n_free_nodal`) and dynamic scalar hub pilot DOFs (`n_free_nodal..n_reduced`) (`P5-T14`).
struct DynamicDofSystem {
    dof_maps: Vec<Vec<DynDofMap>>,
    free_to_body_dof: Vec<(usize, usize)>,
    pilot_to_body_hub: Vec<(usize, usize)>,
    n_free_nodal: usize,
    n_reduced: usize,
}

impl DynamicDofSystem {
    fn build(bodies: &[BodyRuntime], pilot_q: Option<&[f64]>) -> Self {
        let mut dof_maps = Vec::with_capacity(bodies.len());
        let mut free_to_body_dof = Vec::new();
        let mut pilot_to_body_hub = Vec::new();

        // First collect all pilot hubs across bodies
        for (b_idx, body) in bodies.iter().enumerate() {
            for (h_idx, hub) in body.hubs.iter().enumerate() {
                if hub.is_pilot() {
                    pilot_to_body_hub.push((b_idx, h_idx));
                }
            }
        }

        // First pass: mark fixed/hub-controlled DOFs, then assign free nodal indices
        let mut hub_controlled_per_body: Vec<Vec<Option<usize>>> = Vec::with_capacity(bodies.len());
        for (b_idx, body) in bodies.iter().enumerate() {
            let n_dofs = body.mesh.node_count() * 3;
            // Some(usize::MAX) => fixed/prescribed hub; Some(pilot_local_idx) => coupled to pilot
            let mut ctrl = vec![None; n_dofs];
            for (h_idx, hub) in body.hubs.iter().enumerate() {
                let pilot_pos = pilot_to_body_hub
                    .iter()
                    .position(|&(pb, ph)| pb == b_idx && ph == h_idx);
                let marker = pilot_pos.unwrap_or(usize::MAX);
                match hub.kind {
                    ResolvedDynamicHubKind::Prismatic {
                        axis,
                        lock_transverse: false,
                        ..
                    } => {
                        if let Some((comp, _)) = aligned_coordinate_axis(axis) {
                            for &nid in &hub.node_ids {
                                if body.active_nodes.get(nid).copied().unwrap_or(false) {
                                    ctrl[3 * nid + comp] = Some(marker);
                                }
                            }
                        } else {
                            for &nid in &hub.node_ids {
                                if body.active_nodes.get(nid).copied().unwrap_or(false) {
                                    let d0 = 3 * nid;
                                    ctrl[d0] = Some(marker);
                                    ctrl[d0 + 1] = Some(marker);
                                    ctrl[d0 + 2] = Some(marker);
                                }
                            }
                        }
                    }
                    _ => {
                        for &nid in &hub.node_ids {
                            if body.active_nodes.get(nid).copied().unwrap_or(false) {
                                let d0 = 3 * nid;
                                ctrl[d0] = Some(marker);
                                ctrl[d0 + 1] = Some(marker);
                                ctrl[d0 + 2] = Some(marker);
                            }
                        }
                    }
                }
            }

            let n_nodes = body.mesh.node_count();
            let mut map = vec![DynDofMap::Fixed; n_dofs];
            for nid in 0..n_nodes {
                if !body.active_nodes[nid] {
                    continue;
                }
                for axis in 0..3 {
                    let d = 3 * nid + axis;
                    if body.boundary.fixed_dofs.contains_key(&d) || ctrl[d].is_some() {
                        continue;
                    }
                    let f_idx = free_to_body_dof.len();
                    map[d] = DynDofMap::Free(f_idx);
                    free_to_body_dof.push((b_idx, d));
                }
            }
            dof_maps.push(map);
            hub_controlled_per_body.push(ctrl);
        }

        let n_free_nodal = free_to_body_dof.len();
        let n_reduced = n_free_nodal + pilot_to_body_hub.len();

        let mut sys = Self {
            dof_maps,
            free_to_body_dof,
            pilot_to_body_hub,
            n_free_nodal,
            n_reduced,
        };

        // Populate initial pilot tangent weights
        let default_q: Vec<f64> = sys
            .pilot_to_body_hub
            .iter()
            .map(|&(b, h)| bodies[b].hubs[h].q)
            .collect();
        sys.refresh_pilot_weights(bodies, pilot_q.unwrap_or(&default_q));
        let _ = hub_controlled_per_body;
        sys
    }

    /// Updates `DynDofMap::Coupled` tangent weights from the current pilot coordinates `pilot_q`.
    fn refresh_pilot_weights(&mut self, bodies: &[BodyRuntime], pilot_q: &[f64]) {
        for (p_idx, &(b_idx, h_idx)) in self.pilot_to_body_hub.iter().enumerate() {
            let reduced_idx = self.n_free_nodal + p_idx;
            let body = &bodies[b_idx];
            let hub = &body.hubs[h_idx];
            let q = pilot_q.get(p_idx).copied().unwrap_or(hub.q);
            match hub.kind {
                ResolvedDynamicHubKind::Revolute { pivot, axis, .. } => {
                    let p0 = Vector3::from(pivot);
                    let ax = Vector3::from(axis);
                    for &nid in &hub.node_ids {
                        if !body.active_nodes.get(nid).copied().unwrap_or(false) {
                            continue;
                        }
                        let r0 = Vector3::from(body.mesh.nodes[nid].position) - p0;
                        let r_rot = rodrigues_rotate(ax, q, r0);
                        let t_i = ax.cross(&r_rot);
                        let d0 = 3 * nid;
                        self.dof_maps[b_idx][d0] = DynDofMap::Coupled {
                            reduced_idx,
                            weight: t_i.x,
                        };
                        self.dof_maps[b_idx][d0 + 1] = DynDofMap::Coupled {
                            reduced_idx,
                            weight: t_i.y,
                        };
                        self.dof_maps[b_idx][d0 + 2] = DynDofMap::Coupled {
                            reduced_idx,
                            weight: t_i.z,
                        };
                    }
                }
                ResolvedDynamicHubKind::Prismatic {
                    axis,
                    lock_transverse,
                    ..
                } => {
                    if !lock_transverse {
                        if let Some((comp, sign)) = aligned_coordinate_axis(axis) {
                            for &nid in &hub.node_ids {
                                if !body.active_nodes.get(nid).copied().unwrap_or(false) {
                                    continue;
                                }
                                self.dof_maps[b_idx][3 * nid + comp] = DynDofMap::Coupled {
                                    reduced_idx,
                                    weight: sign,
                                };
                            }
                            continue;
                        }
                    }
                    for &nid in &hub.node_ids {
                        if !body.active_nodes.get(nid).copied().unwrap_or(false) {
                            continue;
                        }
                        let d0 = 3 * nid;
                        for (comp, &ax_c) in axis.iter().enumerate() {
                            self.dof_maps[b_idx][d0 + comp] = DynDofMap::Coupled {
                                reduced_idx,
                                weight: ax_c,
                            };
                        }
                    }
                }
                ResolvedDynamicHubKind::Fixed => {}
            }
        }
    }
}

/// Adds scaled CSR matrix entries (`scale * T^T * mat * T`) projected via `dof_map` into `row_entries`.
fn add_csr_to_reduced_rows(
    mat: &CsrMatrix,
    dof_map: &[DynDofMap],
    scale: f64,
    row_entries: &mut [Vec<(usize, f64)>],
) {
    if scale.abs() <= 1e-25 {
        return;
    }
    for row in 0..mat.nrows {
        if let Some((fr, wr)) = dof_map[row].reduced_and_weight() {
            let rv = &mut row_entries[fr];
            let start = mat.row_ptr[row];
            let end = mat.row_ptr[row + 1];
            for idx in start..end {
                let col = mat.col_indices[idx];
                if let Some((fc, wc)) = dof_map[col].reduced_and_weight() {
                    let val = scale * wr * wc * mat.values[idx];
                    if val.abs() > 1e-25 {
                        rv.push((fc, val));
                    }
                }
            }
        }
    }
}

/// Computes per-hub reaction forces, torques, coordinates, and velocities across all bodies (`P5-T14`).
#[allow(clippy::too_many_arguments)]
fn compute_all_hub_reactions(
    bodies: &[BodyRuntime],
    u_cur: &[Vec<f64>],
    v_cur: &[Vec<f64>],
    a_cur: &[Vec<f64>],
    f_res_cur: &[Vec<f64>],
    f_ext_cur: &[Vec<f64>],
    q_coords: &[Vec<(f64, f64)>],
    alpha: f64,
    use_hht_prev: bool,
    c_v_prev: Option<&[Vec<f64>]>,
    opts: &HhtOptions,
) -> Vec<DynamicHubReaction> {
    let mut reactions = Vec::new();
    for (b_idx, body) in bodies.iter().enumerate() {
        if body.hubs.is_empty() {
            continue;
        }
        let nd = body.u.len();
        let mut m_a = vec![0.0; nd];
        body.mass_matrix.spmv(&a_cur[b_idx], &mut m_a);
        let c_v_cur = body.damping_force(&v_cur[b_idx], opts);

        for (h_idx, hub) in body.hubs.iter().enumerate() {
            let (coord, vel) = q_coords[b_idx][h_idx];
            let pivot_vec = match hub.kind {
                ResolvedDynamicHubKind::Revolute { pivot, .. } => Vector3::from(pivot),
                _ => {
                    let mut centroid = Vector3::zeros();
                    let mut cnt = 0usize;
                    for &nid in &hub.node_ids {
                        if body.active_nodes.get(nid).copied().unwrap_or(false) {
                            let p = body.mesh.nodes[nid].position;
                            centroid += Vector3::new(
                                p[0] + u_cur[b_idx][3 * nid],
                                p[1] + u_cur[b_idx][3 * nid + 1],
                                p[2] + u_cur[b_idx][3 * nid + 2],
                            );
                            cnt += 1;
                        }
                    }
                    if cnt > 0 {
                        centroid / (cnt as f64)
                    } else {
                        Vector3::zeros()
                    }
                }
            };

            let mut net_force = Vector3::zeros();
            let mut net_torque = Vector3::zeros();
            let mut active_cnt = 0usize;
            for &nid in &hub.node_ids {
                if !body.active_nodes.get(nid).copied().unwrap_or(false) {
                    continue;
                }
                active_cnt += 1;
                let d0 = 3 * nid;
                let mut f_node = [0.0; 3];
                for (c, fn_c) in f_node.iter_mut().enumerate() {
                    let d = d0 + c;
                    let (f_res_eff, f_ext_eff, f_damp_eff) = if use_hht_prev {
                        let cv_p = c_v_prev.map_or(0.0, |v| v[b_idx][d]);
                        (
                            (1.0 + alpha) * f_res_cur[b_idx][d] - alpha * body.f_int_prev[d],
                            (1.0 + alpha) * f_ext_cur[b_idx][d] - alpha * body.f_ext_prev[d],
                            (1.0 + alpha) * c_v_cur[d] - alpha * cv_p,
                        )
                    } else {
                        (f_res_cur[b_idx][d], f_ext_cur[b_idx][d], c_v_cur[d])
                    };
                    *fn_c = m_a[d] + f_damp_eff + f_res_eff - f_ext_eff;
                }
                let fv = Vector3::from(f_node);
                net_force += fv;
                let p_ref = body.mesh.nodes[nid].position;
                let x_def = Vector3::new(
                    p_ref[0] + u_cur[b_idx][d0],
                    p_ref[1] + u_cur[b_idx][d0 + 1],
                    p_ref[2] + u_cur[b_idx][d0 + 2],
                );
                net_torque += (x_def - pivot_vec).cross(&fv);
            }

            let reaction_effort = match hub.kind {
                ResolvedDynamicHubKind::Revolute { axis, .. } => {
                    net_torque.dot(&Vector3::from(axis))
                }
                ResolvedDynamicHubKind::Prismatic { axis, .. } => {
                    net_force.dot(&Vector3::from(axis))
                }
                ResolvedDynamicHubKind::Fixed => net_force.norm(),
            };

            reactions.push(DynamicHubReaction {
                body_index: b_idx,
                part_id: body.part_id.clone(),
                hub_name: hub.name.clone(),
                kind: hub.kind_str().to_string(),
                node_count: active_cnt,
                coordinate: coord,
                velocity: vel,
                reaction_effort,
                reaction_force_n: [net_force.x, net_force.y, net_force.z],
                reaction_torque_nm: [net_torque.x, net_torque.y, net_torque.z],
            });
        }
    }
    reactions
}

/// Solves $\mathbf{T}^T \mathbf{M} \mathbf{T}\,\ddot{\mathbf{y}}_0 = \mathbf{T}^T(\mathbf{f}_{\text{ext}}(0) - \mathbf{C}\mathbf{v}_0 - \mathbf{f}_{\text{res}}(0) - \mathbf{M}\mathbf{a}_{\text{presc},0}) + \mathbf{Q}_{\text{pilot}}(0)$
/// to obtain dynamically consistent initial nodal and pilot accelerations at $t = 0$.
fn compute_initial_accelerations(
    bodies: &mut [BodyRuntime],
    pairs: &[ContactPairSpec],
    opts: &HhtOptions,
) -> Result<(f64, f64, Vec<DynamicContactForce>, Vec<DynamicHubReaction>), FeaError> {
    let dof_sys = DynamicDofSystem::build(bodies, None);
    let mut dummy_rows: Vec<Vec<(usize, f64)>> = Vec::new();

    let mut f_int_bodies = Vec::with_capacity(bodies.len());
    let mut total_we = 0.0_f64;
    let mut total_wp = 0.0_f64;

    for (b, body) in bodies.iter_mut().enumerate() {
        let u0 = body.u.clone();
        let (f_int, gp, rots, we, wp) = evaluate_body_continuum(
            body,
            &u0,
            opts,
            false,
            &dof_sys.dof_maps[b],
            0.0,
            &mut dummy_rows,
        );
        body.gp_states = gp;
        body.gp_rotations = rots;
        body.f_ext_prev = body.external_force_at(0.0);
        f_int_bodies.push(f_int);
        total_we += we;
        total_wp += wp;
    }

    let u_curr: Vec<Vec<f64>> = bodies.iter().map(|b| b.u.clone()).collect();
    let v_curr: Vec<Vec<f64>> = bodies.iter().map(|b| b.v.clone()).collect();
    let (contact_forces, contact_e) = evaluate_dynamic_contacts(
        bodies,
        pairs,
        &u_curr,
        &v_curr,
        opts.dt_initial_s,
        0.0,
        opts,
        &dof_sys.dof_maps,
        false,
        0.0,
        &mut f_int_bodies,
        &mut dummy_rows,
        None,
        None,
    );
    total_we += contact_e;

    for (b, body) in bodies.iter_mut().enumerate() {
        body.f_int_prev = f_int_bodies[b].clone();
    }

    // Solve per-body reduced mass system M_red * a_red = rhs (M is block-diagonal by body)
    for (b, body) in bodies.iter_mut().enumerate() {
        let n_dofs = body.mesh.node_count() * 3;
        let mut local_map = vec![DynDofMap::Fixed; n_dofs];
        let mut local_free_list = Vec::new();
        for (d, slot) in local_map.iter_mut().enumerate().take(n_dofs) {
            if matches!(dof_sys.dof_maps[b][d], DynDofMap::Free(_)) {
                *slot = DynDofMap::Free(local_free_list.len());
                local_free_list.push(d);
            }
        }
        let n_loc_free = local_free_list.len();
        let mut local_pilots: Vec<usize> = Vec::new();
        for (h_idx, hub) in body.hubs.iter().enumerate() {
            if hub.is_pilot() {
                let p_red = n_loc_free + local_pilots.len();
                local_pilots.push(h_idx);
                match hub.kind {
                    ResolvedDynamicHubKind::Revolute { pivot, axis, .. } => {
                        let p0 = Vector3::from(pivot);
                        let ax = Vector3::from(axis);
                        for &nid in &hub.node_ids {
                            if !body.active_nodes.get(nid).copied().unwrap_or(false) {
                                continue;
                            }
                            let r0 = Vector3::from(body.mesh.nodes[nid].position) - p0;
                            let r_rot = rodrigues_rotate(ax, hub.q, r0);
                            let t_i = ax.cross(&r_rot);
                            let d0 = 3 * nid;
                            local_map[d0] = DynDofMap::Coupled {
                                reduced_idx: p_red,
                                weight: t_i.x,
                            };
                            local_map[d0 + 1] = DynDofMap::Coupled {
                                reduced_idx: p_red,
                                weight: t_i.y,
                            };
                            local_map[d0 + 2] = DynDofMap::Coupled {
                                reduced_idx: p_red,
                                weight: t_i.z,
                            };
                        }
                    }
                    ResolvedDynamicHubKind::Prismatic {
                        axis,
                        lock_transverse,
                        ..
                    } => {
                        if !lock_transverse {
                            if let Some((comp, sign)) = aligned_coordinate_axis(axis) {
                                for &nid in &hub.node_ids {
                                    if body.active_nodes.get(nid).copied().unwrap_or(false) {
                                        local_map[3 * nid + comp] = DynDofMap::Coupled {
                                            reduced_idx: p_red,
                                            weight: sign,
                                        };
                                    }
                                }
                                continue;
                            }
                        }
                        for &nid in &hub.node_ids {
                            if !body.active_nodes.get(nid).copied().unwrap_or(false) {
                                continue;
                            }
                            let d0 = 3 * nid;
                            for (comp, &ax_c) in axis.iter().enumerate() {
                                local_map[d0 + comp] = DynDofMap::Coupled {
                                    reduced_idx: p_red,
                                    weight: ax_c,
                                };
                            }
                        }
                    }
                    ResolvedDynamicHubKind::Fixed => {}
                }
            }
        }
        let n_loc_red = n_loc_free + local_pilots.len();

        // Preserve any prescribed hub t=0 acceleration (e.g. non-zero centripetal or ramp accel)
        let mut a_presc = vec![0.0; n_dofs];
        for hub in &body.hubs {
            if !hub.is_pilot() {
                let mut u_tmp = body.u.clone();
                let mut v_tmp = body.v.clone();
                hub.apply_kinematics_to_nodes(
                    &body.mesh,
                    &body.active_nodes,
                    hub.q,
                    hub.q_dot,
                    hub.q_ddot,
                    &mut u_tmp,
                    &mut v_tmp,
                    &mut a_presc,
                );
            }
        }

        if n_loc_red == 0 {
            body.a = a_presc;
            continue;
        }

        let mut m_a_presc = vec![0.0; n_dofs];
        body.mass_matrix.spmv(&a_presc, &mut m_a_presc);
        let cv0 = body.damping_force(&body.v, opts);

        let mut rhs = vec![0.0; n_loc_red];
        for d in 0..n_dofs {
            if let Some((ri, w)) = local_map[d].reduced_and_weight() {
                rhs[ri] += w * (body.f_ext_prev[d] - cv0[d] - body.f_int_prev[d] - m_a_presc[d]);
            }
        }
        for (lp_i, &h_idx) in local_pilots.iter().enumerate() {
            let hub = &mut body.hubs[h_idx];
            let (q_eff0, _) =
                hub.eval_pilot_effort_and_tangent(hub.q, hub.q_dot, 0.0, 0.0, opts.dt_initial_s);
            hub.q_effort_prev = q_eff0;
            rhs[n_loc_free + lp_i] += q_eff0;
        }

        let mut m_rows = vec![Vec::with_capacity(24); n_loc_red];
        add_csr_to_reduced_rows(&body.mass_matrix, &local_map, 1.0, &mut m_rows);
        let m_red = CsrMatrix::from_row_entries(n_loc_red, n_loc_red, m_rows);
        let a_red = solve_linear_robust(&m_red, &rhs, opts.linear_solver, opts.linear_solver_options)?;

        body.a = a_presc;
        for (fi, &d) in local_free_list.iter().enumerate() {
            body.a[d] = a_red[fi];
        }
        for (lp_i, &h_idx) in local_pilots.iter().enumerate() {
            body.hubs[h_idx].q_ddot = a_red[n_loc_free + lp_i];
            let (q, qd, qdd) = (
                body.hubs[h_idx].q,
                body.hubs[h_idx].q_dot,
                body.hubs[h_idx].q_ddot,
            );
            let hub_clone = body.hubs[h_idx].clone();
            hub_clone.apply_kinematics_to_nodes(
                &body.mesh,
                &body.active_nodes,
                q,
                qd,
                qdd,
                &mut body.u,
                &mut body.v,
                &mut body.a,
            );
        }
    }

    let u_all: Vec<Vec<f64>> = bodies.iter().map(|b| b.u.clone()).collect();
    let v_all: Vec<Vec<f64>> = bodies.iter().map(|b| b.v.clone()).collect();
    let a_all: Vec<Vec<f64>> = bodies.iter().map(|b| b.a.clone()).collect();
    let f_res_all: Vec<Vec<f64>> = bodies.iter().map(|b| b.f_int_prev.clone()).collect();
    let f_ext_all: Vec<Vec<f64>> = bodies.iter().map(|b| b.f_ext_prev.clone()).collect();
    let q_coords: Vec<Vec<(f64, f64)>> = bodies
        .iter()
        .map(|b| b.hubs.iter().map(|h| (h.q, h.q_dot)).collect())
        .collect();
    let init_hub_reactions = compute_all_hub_reactions(
        bodies,
        &u_all,
        &v_all,
        &a_all,
        &f_res_all,
        &f_ext_all,
        &q_coords,
        opts.clamped_alpha(),
        false,
        None,
        opts,
    );

    Ok((total_we, total_wp, contact_forces, init_hub_reactions))
}

// ============================================================================
// Public HHT-α Implicit Dynamic Solvers
// ============================================================================

/// Runs an implicit `HHT-α` dynamic FEA simulation on a single deformable body
/// with adaptive time stepping, optional $J_2$ plasticity, optional fracture/erosion,
/// progress callback, and cancellation token (`P5-T6`).
pub fn solve_dynamic_hht_single_body<F>(
    body: DynamicBodySpec,
    options: HhtOptions,
    progress_cb: Option<F>,
    cancel_flag: Option<&AtomicBool>,
) -> Result<DynamicHhtReport, FeaError>
where
    F: FnMut(DynamicProgressUpdate),
{
    solve_dynamic_hht_coupled(vec![body], &[], options, progress_cb, cancel_flag)
}

/// Runs a multi-body coupled implicit `HHT-α` dynamic FEA simulation supporting:
/// - Unconditional stability without CFL time-step restriction ($\Delta t \gg \Delta t_{\text{CFL}}$)
/// - Consistent or strictly positive lumped mass matrices (`TET4` & `TET10`)
/// - Parallel `rayon` element tangent/force evaluation + RCM sparse direct $LDL^T$ / `IC(0)-PCG`
/// - Deformable-to-deformable and deformable-to-rigid contact with friction
/// - $J_2$ elastoplasticity + continuum damage / element erosion + connected-component fragment detachment
/// - Adaptive time-step bisection on Newton/contact/plasticity/fracture events with full rollback
/// - Real-time progress callback (for SSE streaming) and `AtomicBool` cancellation.
pub fn solve_dynamic_hht_coupled<F>(
    body_specs: Vec<DynamicBodySpec>,
    pairs: &[ContactPairSpec],
    options: HhtOptions,
    mut progress_cb: Option<F>,
    cancel_flag: Option<&AtomicBool>,
) -> Result<DynamicHhtReport, FeaError>
where
    F: FnMut(DynamicProgressUpdate),
{
    let start_wall = Instant::now();

    if body_specs.is_empty() {
        return Err(FeaError::EmptyMesh);
    }
    if !options.total_time_s.is_finite() || options.total_time_s <= 0.0 {
        return Err(FeaError::General(format!(
            "Invalid HHT-alpha total_time_s: {} (must be > 0)",
            options.total_time_s
        )));
    }
    if !options.dt_initial_s.is_finite() || options.dt_initial_s <= 0.0 {
        return Err(FeaError::General(format!(
            "Invalid HHT-alpha dt_initial_s: {} (must be > 0)",
            options.dt_initial_s
        )));
    }

    let alpha = options.clamped_alpha();
    let beta = options.beta();
    let gamma = options.gamma();
    let dt_min = options.dt_min_s.max(1e-10).min(options.dt_initial_s);
    let dt_max = options.dt_max_s.max(options.dt_initial_s);

    // 1. Initialize per-body runtime structures and compute worst-case explicit CFL step
    let mut bodies = Vec::with_capacity(body_specs.len());
    let mut cfl_critical_dt_s = f64::INFINITY;

    for spec in body_specs {
        if spec.density_kg_m3 <= 0.0 || !spec.density_kg_m3.is_finite() {
            return Err(FeaError::General(format!(
                "Invalid density_kg_m3 ({}) for body '{}'",
                spec.density_kg_m3, spec.part_id
            )));
        }
        let body_cfl = estimate_cfl_critical_dt(
            &spec.mesh,
            spec.material.youngs_modulus_pa,
            spec.material.poissons_ratio,
            spec.density_kg_m3,
        );
        cfl_critical_dt_s = cfl_critical_dt_s.min(body_cfl);

        let elements = DynamicElements::from_mesh(&spec.mesh)?;
        let element_volumes = elements.element_volumes();
        let n_nodes = spec.mesh.node_count();
        let n_dofs = n_nodes * 3;
        let n_elems = elements.element_count();
        let n_gps = elements.gp_count();
        let resolved_bc = spec.boundary.resolve_allowing_unconstrained(&spec.mesh)?;

        let mut resolved_hubs = Vec::with_capacity(spec.hub_couplings.len());
        for (h_idx, h_spec) in spec.hub_couplings.iter().enumerate() {
            resolved_hubs.push(ResolvedDynamicHub::from_spec(h_spec, &spec.mesh, h_idx)?);
        }

        let mut anchored_nodes = HashSet::new();
        for &fixed_dof in resolved_bc.fixed_dofs.keys() {
            anchored_nodes.insert(fixed_dof / 3);
        }
        for hub in &resolved_hubs {
            for &nid in &hub.node_ids {
                anchored_nodes.insert(nid);
            }
        }

        let had_user_u0 = spec.initial_displacement.is_some();
        let had_user_v0 = spec.initial_velocity.is_some();

        let mut u0 = match spec.initial_displacement {
            Some(u) if u.len() == n_dofs => u,
            Some(u) => {
                return Err(FeaError::General(format!(
                    "Initial displacement length mismatch for '{}': got {}, expected {}",
                    spec.part_id,
                    u.len(),
                    n_dofs
                )));
            }
            None => vec![0.0; n_dofs],
        };
        // Enforce prescribed Dirichlet values at t = 0
        for (&d, &val) in &resolved_bc.fixed_dofs {
            if d < n_dofs {
                u0[d] = val;
            }
        }

        let mut v0 = match spec.initial_velocity {
            Some(v) if v.len() == n_dofs => v,
            Some(v) => {
                return Err(FeaError::General(format!(
                    "Initial velocity length mismatch for '{}': got {}, expected {}",
                    spec.part_id,
                    v.len(),
                    n_dofs
                )));
            }
            None => vec![0.0; n_dofs],
        };
        for &d in resolved_bc.fixed_dofs.keys() {
            if d < n_dofs {
                v0[d] = 0.0;
            }
        }

        let active_elements = vec![true; n_elems];
        let active_nodes = vec![true; n_nodes];

        // If a body has a single instant-start PrescribedVelocity hub and no manual initial
        // velocity, initialize the whole body with the kinematically consistent rigid velocity
        // (and initial angle/offset if q0 != 0) to avoid artificial elastic startup shock.
        if resolved_hubs.len() == 1 && !had_user_v0 && resolved_bc.fixed_dofs.is_empty() {
            let hub = &resolved_hubs[0];
            match hub.kind {
                ResolvedDynamicHubKind::Revolute {
                    pivot,
                    axis,
                    drive:
                        DynamicRevoluteDrive::PrescribedVelocity {
                            ramp_duration_s, ..
                        },
                } if ramp_duration_s <= 1e-12 => {
                    let p0 = Vector3::from(pivot);
                    let ax = Vector3::from(axis);
                    for i in 0..n_nodes {
                        let r0 = Vector3::from(spec.mesh.nodes[i].position) - p0;
                        let r_rot = rodrigues_rotate(ax, hub.q, r0);
                        if !had_user_u0 && hub.q.abs() > 1e-15 {
                            let ui = r_rot - r0;
                            u0[3 * i] = ui.x;
                            u0[3 * i + 1] = ui.y;
                            u0[3 * i + 2] = ui.z;
                        }
                        let vi = ax.cross(&r_rot) * hub.q_dot;
                        v0[3 * i] = vi.x;
                        v0[3 * i + 1] = vi.y;
                        v0[3 * i + 2] = vi.z;
                    }
                }
                ResolvedDynamicHubKind::Prismatic {
                    axis,
                    drive:
                        DynamicPrismaticDrive::PrescribedVelocity {
                            ramp_duration_s, ..
                        },
                    ..
                } if ramp_duration_s <= 1e-12 => {
                    for i in 0..n_nodes {
                        for c in 0..3 {
                            if !had_user_u0 && hub.q.abs() > 1e-15 {
                                u0[3 * i + c] = axis[c] * hub.q;
                            }
                            v0[3 * i + c] = axis[c] * hub.q_dot;
                        }
                    }
                }
                _ => {}
            }
        }

        let mut a0_init = vec![0.0; n_dofs];
        for hub in &resolved_hubs {
            hub.apply_kinematics_to_nodes(
                &spec.mesh,
                &active_nodes,
                hub.q,
                hub.q_dot,
                hub.q_ddot,
                &mut u0,
                &mut v0,
                &mut a0_init,
            );
        }

        let mut surface = DeformedSurfaceMesh::from_tetra_mesh(&spec.mesh);
        surface.update_from_displacements(&spec.mesh, &u0);

        let eye = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        // Assemble K0 if stiffness-proportional Rayleigh damping is requested
        let rayleigh_k0 = if options.rayleigh_stiffness_beta.abs() > 1e-18 {
            let d_mat = crate::elasticity::isotropic_elasticity_matrix(
                spec.material.youngs_modulus_pa,
                spec.material.poissons_ratio,
            )?;
            let mut k_rows: Vec<Vec<(usize, f64)>> = vec![Vec::with_capacity(48); n_dofs];
            match &elements {
                DynamicElements::Tet4(es) => {
                    for e in es {
                        let ke = e.stiffness_matrix(&d_mat);
                        let mut dofs = [0usize; 12];
                        for (li, &nid) in e.node_indices.iter().enumerate() {
                            dofs[3 * li] = 3 * nid;
                            dofs[3 * li + 1] = 3 * nid + 1;
                            dofs[3 * li + 2] = 3 * nid + 2;
                        }
                        for a in 0..12 {
                            for b in 0..12 {
                                let v = ke[(a, b)];
                                if v.abs() > 1e-25 {
                                    k_rows[dofs[a]].push((dofs[b], v));
                                }
                            }
                        }
                    }
                }
                DynamicElements::Tet10(es) => {
                    for e in es {
                        let ke = e.stiffness_matrix(&d_mat);
                        let mut dofs = [0usize; 30];
                        for (li, &nid) in e.node_indices.iter().enumerate() {
                            dofs[3 * li] = 3 * nid;
                            dofs[3 * li + 1] = 3 * nid + 1;
                            dofs[3 * li + 2] = 3 * nid + 2;
                        }
                        for a in 0..30 {
                            for b in 0..30 {
                                let v = ke[(a, b)];
                                if v.abs() > 1e-25 {
                                    k_rows[dofs[a]].push((dofs[b], v));
                                }
                            }
                        }
                    }
                }
            }
            Some(CsrMatrix::from_row_entries(n_dofs, n_dofs, k_rows))
        } else {
            None
        };

        let mut rt = BodyRuntime {
            part_id: spec.part_id,
            mesh: spec.mesh,
            elements,
            element_volumes,
            material: spec.material,
            density_kg_m3: spec.density_kg_m3,
            boundary: resolved_bc,
            hubs: resolved_hubs,
            gravity: spec.gravity,
            force_curve: spec.force_curve,
            damage_params: spec.damage_params,
            anchored_nodes,
            surface,
            mass_matrix: CsrMatrix::from_row_entries(n_dofs, n_dofs, vec![Vec::new(); n_dofs]),
            rayleigh_k0,
            gravity_force: vec![0.0; n_dofs],
            u: u0,
            v: v0,
            a: a0_init,
            f_int_prev: vec![0.0; n_dofs],
            f_ext_prev: vec![0.0; n_dofs],
            gp_states: vec![GaussPointState::default(); n_gps],
            gp_rotations: vec![eye; n_gps],
            damage_states: vec![ElementDamageState::default(); n_elems],
            active_elements,
            active_nodes,
            eroded_elements: Vec::new(),
            plastic_yield_logged: false,
            crack_init_logged: false,
            cumulative_fracture_energy_j: 0.0,
            next_fragment_counter: 0,
        };
        rt.recompute_active_nodes();
        rt.rebuild_mass_and_gravity(options.mass_matrix_kind);
        rt.update_hub_inertia_estimates();
        bodies.push(rt);
    }

    if !cfl_critical_dt_s.is_finite() {
        cfl_critical_dt_s = 1e-6;
    }

    // 2. Consistent initial acceleration a_0 at t = 0
    let (init_we, init_wp, init_contacts, init_hub_reactions) =
        compute_initial_accelerations(&mut bodies, pairs, &options)?;
    let init_ke: f64 = bodies.iter().map(BodyRuntime::kinetic_energy).sum();
    let initial_mech_energy = init_ke + init_we;

    let mut frames: Vec<DynamicFrameSnapshot> = Vec::new();
    let mut events: Vec<DynamicTimelineEvent> = Vec::new();
    let mut all_detached_fragments: Vec<DetachedFragment> = Vec::new();
    let mut cumulative_ext_work = 0.0_f64;

    // Frame 0 (initial state at t = 0)
    frames.push(DynamicFrameSnapshot {
        frame_index: 0,
        time_s: 0.0,
        dt_used_s: 0.0,
        newton_iterations: 0,
        bodies: bodies.iter().map(BodyRuntime::build_frame_snapshot).collect(),
        contact_forces: init_contacts,
        detached_fragments: Vec::new(),
        energy: DynamicEnergyBalance {
            kinetic_j: init_ke,
            elastic_j: init_we,
            plastic_j: init_wp,
            fracture_j: 0.0,
            external_work_j: 0.0,
            total_mechanical_j: initial_mech_energy,
            total_energy_j: initial_mech_energy + init_wp,
        },
        hub_reactions: init_hub_reactions,
    });

    if let Some(cb) = progress_cb.as_mut() {
        let peak_vm = frames[0]
            .bodies
            .iter()
            .map(|b| b.max_von_mises_pa)
            .fold(0.0_f64, f64::max);
        cb(DynamicProgressUpdate {
            time_s: 0.0,
            total_time_s: options.total_time_s,
            progress_pct: 0.0,
            step_count: 0,
            frame_count: 1,
            current_dt_s: options.dt_initial_s,
            bisections: 0,
            eroded_elements: 0,
            detached_fragments: 0,
            max_von_mises_pa: peak_vm,
        });
    }

    // 3. Adaptive HHT-α time-marching loop
    let mut t_curr = 0.0_f64;
    let mut dt = options.dt_initial_s.clamp(dt_min, dt_max);
    let mut total_steps = 0usize;
    let mut total_newton_iterations = 0usize;
    let mut total_bisections = 0usize;
    let mut min_dt_used = f64::INFINITY;
    let mut max_dt_used = 0.0_f64;
    let mut max_energy_drift_pct = 0.0_f64;
    let mut last_frame_time = 0.0_f64;
    let mut cancelled = false;
    let mut prev_had_contact = false;
    let max_frames = options.max_frames.max(2);

    while t_curr < options.total_time_s - 1e-13 {
        if cancel_flag.is_some_and(|flag| flag.load(Ordering::Relaxed)) {
            cancelled = true;
            events.push(DynamicTimelineEvent {
                time_s: t_curr,
                kind: "cancelled".to_string(),
                part_id: None,
                description: format!(
                    "Simulation cancelled at t = {:.6} s after {} steps",
                    t_curr, total_steps
                ),
            });
            break;
        }

        let remaining = options.total_time_s - t_curr;
        let mut step_dt = dt.min(remaining);
        let checkpoints: Vec<BodyCheckpoint> = bodies.iter().map(BodyRuntime::checkpoint).collect();
        let mut step_accepted = false;
        let mut step_bisections = 0usize;

        while !step_accepted {
            let t_next = t_curr + step_dt;
            let c0 = 1.0 / (beta * step_dt * step_dt);
            let c1 = gamma / (beta * step_dt);

            let mut dof_sys = DynamicDofSystem::build(&bodies, None);
            let n_free_nodal = dof_sys.n_free_nodal;
            let n_pilots = dof_sys.pilot_to_body_hub.len();
            let n_red = dof_sys.n_reduced;

            // Precompute Newmark predictors for pilot DOFs
            let mut q_tilde = Vec::with_capacity(n_pilots);
            let mut q_dot_tilde = Vec::with_capacity(n_pilots);
            let mut q_trial = Vec::with_capacity(n_pilots);
            let mut q_dot_trial = Vec::with_capacity(n_pilots);
            let mut q_ddot_trial = Vec::with_capacity(n_pilots);
            for &(b_idx, h_idx) in &dof_sys.pilot_to_body_hub {
                let hub = &bodies[b_idx].hubs[h_idx];
                let qt = hub.q
                    + step_dt * hub.q_dot
                    + 0.5 * step_dt * step_dt * (1.0 - 2.0 * beta) * hub.q_ddot;
                let qdt = hub.q_dot + step_dt * (1.0 - gamma) * hub.q_ddot;
                q_tilde.push(qt);
                q_dot_tilde.push(qdt);
                q_trial.push(qt);
                q_dot_trial.push(qdt);
                q_ddot_trial.push(0.0);
            }
            dof_sys.refresh_pilot_weights(&bodies, &q_trial);

            // Evaluate prescribed (non-pilot) hubs at t_next
            let mut prescribed_next: Vec<Vec<(f64, f64, f64)>> = Vec::with_capacity(bodies.len());
            for body in &bodies {
                let mut ph = Vec::with_capacity(body.hubs.len());
                for hub in &body.hubs {
                    if hub.is_pilot() {
                        ph.push((0.0, 0.0, 0.0));
                    } else {
                        ph.push(hub.prescribed_kinematics_at(t_next));
                    }
                }
                prescribed_next.push(ph);
            }

            // Precompute Newmark predictors u_tilde, v_tilde and external/damping terms at t_n, t_{n+1}
            let mut u_tilde = Vec::with_capacity(bodies.len());
            let mut v_tilde = Vec::with_capacity(bodies.len());
            let mut u_trial = Vec::with_capacity(bodies.len());
            let mut v_trial = Vec::with_capacity(bodies.len());
            let mut a_trial = Vec::with_capacity(bodies.len());
            let mut f_ext_next = Vec::with_capacity(bodies.len());
            let mut c_v_prev = Vec::with_capacity(bodies.len());

            for (b_idx, body) in bodies.iter().enumerate() {
                let nd = body.u.len();
                let mut ut = vec![0.0; nd];
                let mut vt = vec![0.0; nd];
                let mut u_tr = vec![0.0; nd];
                let mut v_tr = vec![0.0; nd];
                let mut a_tr = vec![0.0; nd];
                for d in 0..nd {
                    if body.boundary.fixed_dofs.contains_key(&d) {
                        let fixed_val = body.boundary.fixed_dofs[&d];
                        ut[d] = fixed_val;
                        vt[d] = 0.0;
                        u_tr[d] = fixed_val;
                        v_tr[d] = 0.0;
                        a_tr[d] = 0.0;
                    } else {
                        ut[d] = body.u[d]
                            + step_dt * body.v[d]
                            + 0.5 * step_dt * step_dt * (1.0 - 2.0 * beta) * body.a[d];
                        vt[d] = body.v[d] + step_dt * (1.0 - gamma) * body.a[d];
                        // Initial Newton guess = predictor (a_{n+1} = 0)
                        u_tr[d] = ut[d];
                        v_tr[d] = vt[d];
                        a_tr[d] = 0.0;
                    }
                }
                // Apply prescribed (non-pilot) hubs at t_next
                for (h_idx, hub) in body.hubs.iter().enumerate() {
                    if !hub.is_pilot() {
                        let (qp, qdp, qddp) = prescribed_next[b_idx][h_idx];
                        hub.apply_kinematics_to_nodes(
                            &body.mesh,
                            &body.active_nodes,
                            qp,
                            qdp,
                            qddp,
                            &mut u_tr,
                            &mut v_tr,
                            &mut a_tr,
                        );
                    }
                }
                u_tilde.push(ut);
                v_tilde.push(vt);
                u_trial.push(u_tr);
                v_trial.push(v_tr);
                a_trial.push(a_tr);
                f_ext_next.push(body.external_force_at(t_next));
                c_v_prev.push(body.damping_force(&body.v, &options));
            }
            // Apply pilot predictors onto coupled hub nodes
            for (p_idx, &(b_idx, h_idx)) in dof_sys.pilot_to_body_hub.iter().enumerate() {
                bodies[b_idx].hubs[h_idx].apply_kinematics_to_nodes(
                    &bodies[b_idx].mesh,
                    &bodies[b_idx].active_nodes,
                    q_trial[p_idx],
                    q_dot_trial[p_idx],
                    q_ddot_trial[p_idx],
                    &mut u_trial[b_idx],
                    &mut v_trial[b_idx],
                    &mut a_trial[b_idx],
                );
            }

            // Newton-Raphson iterations for HHT-α dynamic equilibrium
            let mut converged = n_red == 0;
            let mut iters_used = 0usize;
            let mut last_rel_res = f64::NAN;
            let mut final_f_res: Vec<Vec<f64>> = bodies.iter().map(|b| b.f_int_prev.clone()).collect();
            let mut final_gp: Vec<Vec<GaussPointState>> =
                bodies.iter().map(|b| b.gp_states.clone()).collect();
            let mut final_rots: Vec<Vec<[[f64; 3]; 3]>> =
                bodies.iter().map(|b| b.gp_rotations.clone()).collect();
            let mut final_contacts: Vec<DynamicContactForce> = Vec::new();
            let mut final_we = 0.0_f64;
            let mut final_wp = 0.0_f64;
            let mut final_pilot_efforts = vec![0.0_f64; n_pilots];

            if n_red > 0 {
                let mut frozen_contacts: Option<Vec<FrozenDeformableContact>> = None;
                for iter in 0..options.max_newton_iterations {
                    iters_used = iter + 1;
                    dof_sys.refresh_pilot_weights(&bodies, &q_trial);

                    let mut row_entries: Vec<Vec<(usize, f64)>> =
                        vec![Vec::with_capacity(64); n_red];
                    let mut f_res_bodies = Vec::with_capacity(bodies.len());
                    let mut trial_gp_bodies = Vec::with_capacity(bodies.len());
                    let mut trial_rot_bodies = Vec::with_capacity(bodies.len());
                    let mut iter_we = 0.0_f64;
                    let mut iter_wp = 0.0_f64;

                    // Assemble continuum internal forces & (1 + alpha) * T^T K_T T, plus mass/Rayleigh terms
                    for (b, body) in bodies.iter().enumerate() {
                        let (f_int, gp, rots, we, wp) = evaluate_body_continuum(
                            body,
                            &u_trial[b],
                            &options,
                            true,
                            &dof_sys.dof_maps[b],
                            1.0 + alpha,
                            &mut row_entries,
                        );
                        f_res_bodies.push(f_int);
                        trial_gp_bodies.push(gp);
                        trial_rot_bodies.push(rots);
                        iter_we += we;
                        iter_wp += wp;

                        // Add (c0 + (1 + alpha) * c1 * c_M) * T^T M T
                        let m_scale = c0 + (1.0 + alpha) * c1 * options.rayleigh_mass_alpha;
                        add_csr_to_reduced_rows(
                            &body.mass_matrix,
                            &dof_sys.dof_maps[b],
                            m_scale,
                            &mut row_entries,
                        );
                        // Add (1 + alpha) * c1 * c_K * T^T K0 T
                        if options.rayleigh_stiffness_beta.abs() > 1e-18 {
                            if let Some(ref k0) = body.rayleigh_k0 {
                                let ck_scale = (1.0 + alpha) * c1 * options.rayleigh_stiffness_beta;
                                add_csr_to_reduced_rows(
                                    k0,
                                    &dof_sys.dof_maps[b],
                                    ck_scale,
                                    &mut row_entries,
                                );
                            }
                        }
                    }

                    // Contact forces & symmetric PSD tangent.
                    // After 4 full BVH iterations without convergence on faceted contact meshes,
                    // freeze the master-facet pairing for the remaining polishing iterations of this step.
                    let mut captured_frozen = Vec::new();
                    let capture_this_iter = iter == 4 && frozen_contacts.is_none() && !pairs.is_empty();
                    let (contacts, contact_e) = evaluate_dynamic_contacts(
                        &bodies,
                        pairs,
                        &u_trial,
                        &v_trial,
                        step_dt,
                        c1,
                        &options,
                        &dof_sys.dof_maps,
                        true,
                        1.0 + alpha,
                        &mut f_res_bodies,
                        &mut row_entries,
                        frozen_contacts.as_deref(),
                        if capture_this_iter {
                            Some(&mut captured_frozen)
                        } else {
                            None
                        },
                    );
                    if capture_this_iter && !captured_frozen.is_empty() {
                        frozen_contacts = Some(captured_frozen);
                    }
                    iter_we += contact_e;

                    // Pilot actuation / motor / damping / spring / limit contributions
                    let mut pilot_efforts = vec![0.0_f64; n_pilots];
                    for (p_idx, &(b_idx, h_idx)) in dof_sys.pilot_to_body_hub.iter().enumerate() {
                        let hub = &bodies[b_idx].hubs[h_idx];
                        let (q_eff, k_pilot) = hub.eval_pilot_effort_and_tangent(
                            q_trial[p_idx],
                            q_dot_trial[p_idx],
                            t_next,
                            c1,
                            step_dt,
                        );
                        pilot_efforts[p_idx] = q_eff;
                        let pi = n_free_nodal + p_idx;
                        let k_scaled = (1.0 + alpha) * k_pilot;
                        if k_scaled.abs() > 1e-25 {
                            row_entries[pi].push((pi, k_scaled));
                        }
                    }

                    // Compute HHT-α dynamic residual R on reduced DOFs (free nodal + pilots)
                    let mut r_red = vec![0.0; n_red];
                    let mut scale_sq = 0.0_f64;

                    for (b, body) in bodies.iter().enumerate() {
                        let mut m_a = vec![0.0; body.u.len()];
                        body.mass_matrix.spmv(&a_trial[b], &mut m_a);
                        let c_v_next = body.damping_force(&v_trial[b], &options);

                        for d in 0..body.u.len() {
                            if let Some((ri, w)) = dof_sys.dof_maps[b][d].reduced_and_weight() {
                                let f_ext_hht = (1.0 + alpha) * f_ext_next[b][d]
                                    - alpha * body.f_ext_prev[d];
                                let f_int_hht = (1.0 + alpha) * f_res_bodies[b][d]
                                    - alpha * body.f_int_prev[d];
                                let f_damp_hht =
                                    (1.0 + alpha) * c_v_next[d] - alpha * c_v_prev[b][d];
                                r_red[ri] += w * (f_ext_hht - m_a[d] - f_damp_hht - f_int_hht);

                                let term = w.abs()
                                    * f_ext_hht
                                        .abs()
                                        .max(m_a[d].abs())
                                        .max(f_int_hht.abs());
                                scale_sq += term * term;
                            }
                        }
                    }
                    for (p_idx, &(b_idx, h_idx)) in dof_sys.pilot_to_body_hub.iter().enumerate() {
                        let pi = n_free_nodal + p_idx;
                        let q_prev = bodies[b_idx].hubs[h_idx].q_effort_prev;
                        let q_hht = (1.0 + alpha) * pilot_efforts[p_idx] - alpha * q_prev;
                        r_red[pi] += q_hht;
                        scale_sq += q_hht * q_hht;
                    }

                    let r_norm = norm_slice(&r_red);
                    if !r_norm.is_finite() {
                        break;
                    }
                    let f_ref = scale_sq.sqrt().max(1e-12);
                    let rel_res = r_norm / f_ref;
                    last_rel_res = rel_res;

                    if rel_res <= options.force_tolerance || r_norm <= 1e-11 {
                        converged = true;
                        final_f_res = f_res_bodies;
                        final_gp = trial_gp_bodies;
                        final_rots = trial_rot_bodies;
                        final_contacts = contacts;
                        final_we = iter_we;
                        final_wp = iter_wp;
                        final_pilot_efforts = pilot_efforts;
                        break;
                    }

                    // Solve K_eff * dy = R
                    let k_eff = CsrMatrix::from_row_entries(n_red, n_red, row_entries);
                    let du_res = solve_spd_newton_robust(
                        &k_eff,
                        &r_red,
                        options.linear_solver,
                        options.linear_solver_options,
                    );
                    let Ok(dy_red) = du_res else {
                        break;
                    };

                    // Backtracking line search (active when contacts/pilots exist or iter >= 1)
                    let mut step_scale = 1.0_f64;
                    if options.enable_line_search && (iter >= 1 || !pairs.is_empty() || n_pilots > 0)
                    {
                        let mut best_scale = 1.0_f64;
                        let mut best_rc = f64::INFINITY;
                        let mut s = 1.0_f64;
                        for _ls in 0..6 {
                            let mut u_cand = u_trial.clone();
                            let mut v_cand = v_trial.clone();
                            let mut a_cand = a_trial.clone();
                            let mut q_cand = q_trial.clone();
                            let mut q_dot_cand = q_dot_trial.clone();
                            let mut q_ddot_cand = q_ddot_trial.clone();

                            for (fi, &(b, d)) in dof_sys.free_to_body_dof.iter().enumerate() {
                                u_cand[b][d] += s * dy_red[fi];
                                a_cand[b][d] = c0 * (u_cand[b][d] - u_tilde[b][d]);
                                v_cand[b][d] = v_tilde[b][d] + c1 * (u_cand[b][d] - u_tilde[b][d]);
                            }
                            for (p_idx, &(b_idx, h_idx)) in
                                dof_sys.pilot_to_body_hub.iter().enumerate()
                            {
                                let pi = n_free_nodal + p_idx;
                                q_cand[p_idx] += s * dy_red[pi];
                                q_ddot_cand[p_idx] = c0 * (q_cand[p_idx] - q_tilde[p_idx]);
                                q_dot_cand[p_idx] =
                                    q_dot_tilde[p_idx] + c1 * (q_cand[p_idx] - q_tilde[p_idx]);
                                bodies[b_idx].hubs[h_idx].apply_kinematics_to_nodes(
                                    &bodies[b_idx].mesh,
                                    &bodies[b_idx].active_nodes,
                                    q_cand[p_idx],
                                    q_dot_cand[p_idx],
                                    q_ddot_cand[p_idx],
                                    &mut u_cand[b_idx],
                                    &mut v_cand[b_idx],
                                    &mut a_cand[b_idx],
                                );
                            }
                            dof_sys.refresh_pilot_weights(&bodies, &q_cand);

                            let mut dummy = Vec::new();
                            let mut f_cand = Vec::with_capacity(bodies.len());
                            for (b, body) in bodies.iter().enumerate() {
                                let (fi_b, _, _, _, _) = evaluate_body_continuum(
                                    body,
                                    &u_cand[b],
                                    &options,
                                    false,
                                    &dof_sys.dof_maps[b],
                                    0.0,
                                    &mut dummy,
                                );
                                f_cand.push(fi_b);
                            }
                            let _ = evaluate_dynamic_contacts(
                                &bodies,
                                pairs,
                                &u_cand,
                                &v_cand,
                                step_dt,
                                c1,
                                &options,
                                &dof_sys.dof_maps,
                                false,
                                0.0,
                                &mut f_cand,
                                &mut dummy,
                                frozen_contacts.as_deref(),
                                None,
                            );
                            let mut r_cand = vec![0.0; n_red];
                            for (b, body) in bodies.iter().enumerate() {
                                let mut m_a = vec![0.0; body.u.len()];
                                body.mass_matrix.spmv(&a_cand[b], &mut m_a);
                                let cv = body.damping_force(&v_cand[b], &options);
                                for d in 0..body.u.len() {
                                    if let Some((ri, w)) =
                                        dof_sys.dof_maps[b][d].reduced_and_weight()
                                    {
                                        let f_ext_hht = (1.0 + alpha) * f_ext_next[b][d]
                                            - alpha * body.f_ext_prev[d];
                                        let f_int_hht = (1.0 + alpha) * f_cand[b][d]
                                            - alpha * body.f_int_prev[d];
                                        let f_damp_hht =
                                            (1.0 + alpha) * cv[d] - alpha * c_v_prev[b][d];
                                        r_cand[ri] +=
                                            w * (f_ext_hht - m_a[d] - f_damp_hht - f_int_hht);
                                    }
                                }
                            }
                            for (p_idx, &(b_idx, h_idx)) in
                                dof_sys.pilot_to_body_hub.iter().enumerate()
                            {
                                let pi = n_free_nodal + p_idx;
                                let hub = &bodies[b_idx].hubs[h_idx];
                                let (q_eff_c, _) = hub.eval_pilot_effort_and_tangent(
                                    q_cand[p_idx],
                                    q_dot_cand[p_idx],
                                    t_next,
                                    c1,
                                    step_dt,
                                );
                                r_cand[pi] += (1.0 + alpha) * q_eff_c - alpha * hub.q_effort_prev;
                            }
                            let rc_norm = norm_slice(&r_cand);
                            if rc_norm.is_finite() && rc_norm < best_rc {
                                best_rc = rc_norm;
                                best_scale = s;
                            }
                            if rc_norm.is_finite() && rc_norm < 0.98 * r_norm {
                                break;
                            }
                            s *= 0.5;
                        }
                        step_scale = best_scale;
                    }

                    let mut du_norm_sq = 0.0_f64;
                    let mut u_norm_sq = 0.0_f64;
                    for (fi, &(b, d)) in dof_sys.free_to_body_dof.iter().enumerate() {
                        let delta = step_scale * dy_red[fi];
                        u_trial[b][d] += delta;
                        a_trial[b][d] = c0 * (u_trial[b][d] - u_tilde[b][d]);
                        v_trial[b][d] = v_tilde[b][d] + c1 * (u_trial[b][d] - u_tilde[b][d]);
                        du_norm_sq += delta * delta;
                        u_norm_sq += u_trial[b][d] * u_trial[b][d];
                    }
                    for (p_idx, &(b_idx, h_idx)) in dof_sys.pilot_to_body_hub.iter().enumerate() {
                        let pi = n_free_nodal + p_idx;
                        let dq = step_scale * dy_red[pi];
                        q_trial[p_idx] += dq;
                        q_ddot_trial[p_idx] = c0 * (q_trial[p_idx] - q_tilde[p_idx]);
                        q_dot_trial[p_idx] =
                            q_dot_tilde[p_idx] + c1 * (q_trial[p_idx] - q_tilde[p_idx]);
                        du_norm_sq += dq * dq;
                        u_norm_sq += q_trial[p_idx] * q_trial[p_idx];
                        bodies[b_idx].hubs[h_idx].apply_kinematics_to_nodes(
                            &bodies[b_idx].mesh,
                            &bodies[b_idx].active_nodes,
                            q_trial[p_idx],
                            q_dot_trial[p_idx],
                            q_ddot_trial[p_idx],
                            &mut u_trial[b_idx],
                            &mut v_trial[b_idx],
                            &mut a_trial[b_idx],
                        );
                    }

                    // Secondary displacement increment check (helpful on linear elastic steps where 1 Newton step is exact)
                    if du_norm_sq.sqrt() <= 1e-13 * u_norm_sq.sqrt().max(1.0) && iter >= 1 {
                        dof_sys.refresh_pilot_weights(&bodies, &q_trial);
                        // Re-evaluate final forces/energies at converged u_trial
                        let mut dummy = Vec::new();
                        let mut f_final = Vec::with_capacity(bodies.len());
                        let mut gp_final = Vec::with_capacity(bodies.len());
                        let mut rot_final = Vec::with_capacity(bodies.len());
                        let mut we_f = 0.0;
                        let mut wp_f = 0.0;
                        for (b, body) in bodies.iter().enumerate() {
                            let (fi_b, gp, rots, we, wp) = evaluate_body_continuum(
                                body,
                                &u_trial[b],
                                &options,
                                false,
                                &dof_sys.dof_maps[b],
                                0.0,
                                &mut dummy,
                            );
                            f_final.push(fi_b);
                            gp_final.push(gp);
                            rot_final.push(rots);
                            we_f += we;
                            wp_f += wp;
                        }
                        let (contacts_f, ce_f) = evaluate_dynamic_contacts(
                            &bodies,
                            pairs,
                            &u_trial,
                            &v_trial,
                            step_dt,
                            c1,
                            &options,
                            &dof_sys.dof_maps,
                            false,
                            0.0,
                            &mut f_final,
                            &mut dummy,
                            frozen_contacts.as_deref(),
                            None,
                        );
                        for (p_idx, &(b_idx, h_idx)) in
                            dof_sys.pilot_to_body_hub.iter().enumerate()
                        {
                            let (q_eff_f, _) = bodies[b_idx].hubs[h_idx]
                                .eval_pilot_effort_and_tangent(
                                    q_trial[p_idx],
                                    q_dot_trial[p_idx],
                                    t_next,
                                    c1,
                                    step_dt,
                                );
                            final_pilot_efforts[p_idx] = q_eff_f;
                        }
                        converged = true;
                        final_f_res = f_final;
                        final_gp = gp_final;
                        final_rots = rot_final;
                        final_contacts = contacts_f;
                        final_we = we_f + ce_f;
                        final_wp = wp_f;
                        break;
                    }
                }
            }

            // Check adaptive step-size criteria on plasticity & damage jumps
            let mut max_plastic_jump = 0.0_f64;
            let mut max_damage_jump = 0.0_f64;
            let mut trial_damage_updates: Vec<Vec<(usize, ElementDamageState, f64, bool)>> =
                vec![Vec::new(); bodies.len()];

            if converged {
                for (b, body) in bodies.iter().enumerate() {
                    if options.enable_plasticity {
                        for (gp_old, gp_new) in body.gp_states.iter().zip(final_gp[b].iter()) {
                            let dep = (gp_new.equivalent_plastic_strain
                                - gp_old.equivalent_plastic_strain)
                                .max(0.0);
                            max_plastic_jump = max_plastic_jump.max(dep);
                        }
                    }
                    if options.enable_fracture {
                        if let Some(dp) = body.damage_params {
                            let n_elems = body.elements.element_count();
                            for e_idx in 0..n_elems {
                                if !body.active_elements[e_idx] {
                                    continue;
                                }
                                let (eff_stress_world, ep_new, dep) = match &body.elements {
                                    DynamicElements::Tet4(_) => {
                                        let gp_old = &body.gp_states[e_idx];
                                        let gp_new = &final_gp[b][e_idx];
                                        let rot = Matrix3::from_row_slice(&[
                                            final_rots[b][e_idx][0][0],
                                            final_rots[b][e_idx][0][1],
                                            final_rots[b][e_idx][0][2],
                                            final_rots[b][e_idx][1][0],
                                            final_rots[b][e_idx][1][1],
                                            final_rots[b][e_idx][1][2],
                                            final_rots[b][e_idx][2][0],
                                            final_rots[b][e_idx][2][1],
                                            final_rots[b][e_idx][2][2],
                                        ]);
                                        let sw = rotate_voigt_stress(
                                            &rot,
                                            &Vector6::from(gp_new.stress),
                                        );
                                        (
                                            sw,
                                            gp_new.equivalent_plastic_strain,
                                            (gp_new.equivalent_plastic_strain
                                                - gp_old.equivalent_plastic_strain)
                                                .max(0.0),
                                        )
                                    }
                                    DynamicElements::Tet10(_) => {
                                        let base = e_idx * 4;
                                        let mut sw_avg = Vector6::zeros();
                                        let mut ep_max = 0.0_f64;
                                        let mut dep_max = 0.0_f64;
                                        for q in 0..4 {
                                            let gp_old = &body.gp_states[base + q];
                                            let gp_new = &final_gp[b][base + q];
                                            let r = final_rots[b][base + q];
                                            let rot = Matrix3::from_row_slice(&[
                                                r[0][0], r[0][1], r[0][2], r[1][0], r[1][1],
                                                r[1][2], r[2][0], r[2][1], r[2][2],
                                            ]);
                                            sw_avg += rotate_voigt_stress(
                                                &rot,
                                                &Vector6::from(gp_new.stress),
                                            ) * 0.25;
                                            ep_max = ep_max.max(gp_new.equivalent_plastic_strain);
                                            dep_max = dep_max.max(
                                                (gp_new.equivalent_plastic_strain
                                                    - gp_old.equivalent_plastic_strain)
                                                    .max(0.0),
                                            );
                                        }
                                        (sw_avg, ep_max, dep_max)
                                    }
                                };
                                let upd = evaluate_element_damage(
                                    &body.damage_states[e_idx],
                                    &eff_stress_world,
                                    ep_new,
                                    dep,
                                    body.element_volumes[e_idx],
                                    &dp,
                                );
                                max_damage_jump = max_damage_jump.max(upd.delta_damage);
                                if upd.delta_damage > 0.0 || upd.newly_eroded {
                                    trial_damage_updates[b].push((
                                        e_idx,
                                        upd.state,
                                        upd.delta_fracture_energy_j,
                                        upd.newly_eroded,
                                    ));
                                }
                            }
                        }
                    }
                }
            }

            let sudden_event_bisect = converged
                && step_dt > dt_min * 1.5
                && step_bisections < options.max_bisections_per_step
                && (max_damage_jump > options.max_damage_jump_per_step
                    || max_plastic_jump > options.max_plastic_jump_per_step);

            if !converged || sudden_event_bisect {
                // Roll back and bisect Δt
                for (b, body) in bodies.iter_mut().enumerate() {
                    body.restore(&checkpoints[b], options.mass_matrix_kind);
                }
                if step_dt <= dt_min * 1.01 || step_bisections >= options.max_bisections_per_step {
                    if converged {
                        // Accept at dt_min even with rapid fracture jump
                    } else {
                        return Err(FeaError::SolverConvergenceFailed {
                            iterations: iters_used,
                            residual: last_rel_res,
                            tolerance: options.force_tolerance,
                        });
                    }
                } else {
                    let prev_dt = step_dt;
                    step_dt = (0.5 * step_dt).max(dt_min);
                    dt = step_dt;
                    step_bisections += 1;
                    total_bisections += 1;
                    let reason = if !converged {
                        "Newton non-convergence"
                    } else if max_damage_jump > options.max_damage_jump_per_step {
                        "damage/fracture shock"
                    } else {
                        "plasticity jump"
                    };
                    events.push(DynamicTimelineEvent {
                        time_s: t_curr,
                        kind: "step_bisection".to_string(),
                        part_id: None,
                        description: format!(
                            "Adaptive Δt bisected {:.3e}s -> {:.3e}s ({reason})",
                            prev_dt, step_dt
                        ),
                    });
                    continue;
                }
            }

            // Compute HHT-α balanced hub reactions before overwriting prev-step force vectors
            let step_hub_coords: Vec<Vec<(f64, f64)>> = bodies
                .iter()
                .enumerate()
                .map(|(b_idx, body)| {
                    body.hubs
                        .iter()
                        .enumerate()
                        .map(|(h_idx, hub)| {
                            if hub.is_pilot() {
                                let p_idx = dof_sys
                                    .pilot_to_body_hub
                                    .iter()
                                    .position(|&(pb, ph)| pb == b_idx && ph == h_idx)
                                    .expect("pilot index");
                                (q_trial[p_idx], q_dot_trial[p_idx])
                            } else {
                                let (qp, qdp, _) = prescribed_next[b_idx][h_idx];
                                (qp, qdp)
                            }
                        })
                        .collect()
                })
                .collect();
            let step_hub_reactions = compute_all_hub_reactions(
                &bodies,
                &u_trial,
                &v_trial,
                &a_trial,
                &final_f_res,
                &f_ext_next,
                &step_hub_coords,
                alpha,
                true,
                Some(&c_v_prev),
                &options,
            );

            // Accumulate pilot actuator work and commit hub states
            for (p_idx, &(b_idx, h_idx)) in dof_sys.pilot_to_body_hub.iter().enumerate() {
                let hub = &mut bodies[b_idx].hubs[h_idx];
                let dq = q_trial[p_idx] - hub.q;
                let q_avg = 0.5 * (hub.q_effort_prev + final_pilot_efforts[p_idx]);
                cumulative_ext_work += q_avg * dq;
                hub.q = q_trial[p_idx];
                hub.q_dot = q_dot_trial[p_idx];
                hub.q_ddot = q_ddot_trial[p_idx];
                hub.q_effort_prev = final_pilot_efforts[p_idx];
            }
            for (b_idx, body) in bodies.iter_mut().enumerate() {
                for (h_idx, hub) in body.hubs.iter_mut().enumerate() {
                    if !hub.is_pilot() {
                        let (qp, qdp, qddp) = prescribed_next[b_idx][h_idx];
                        hub.q = qp;
                        hub.q_dot = qdp;
                        hub.q_ddot = qddp;
                    }
                }
            }

            // Accept converged step: accumulate external work (trapezoidal) and commit states
            for (b, body) in bodies.iter_mut().enumerate() {
                for d in 0..body.u.len() {
                    let du = u_trial[b][d] - body.u[d];
                    let f_avg = 0.5 * (body.f_ext_prev[d] + f_ext_next[b][d]);
                    cumulative_ext_work += f_avg * du;
                }
                body.u = u_trial[b].clone();
                body.v = v_trial[b].clone();
                body.a = a_trial[b].clone();
                body.f_int_prev = final_f_res[b].clone();
                body.f_ext_prev = f_ext_next[b].clone();
                body.gp_states = final_gp[b].clone();
                body.gp_rotations = final_rots[b].clone();
                body.surface.update_from_displacements(&body.mesh, &body.u);

                // Log first plastic yield event
                if !body.plastic_yield_logged
                    && body
                        .gp_states
                        .iter()
                        .any(|gp| gp.equivalent_plastic_strain > 1e-6)
                {
                    body.plastic_yield_logged = true;
                    events.push(DynamicTimelineEvent {
                        time_s: t_next,
                        kind: "plastic_yield".to_string(),
                        part_id: Some(body.part_id.clone()),
                        description: format!("Plastic yielding initiated in '{}'", body.part_id),
                    });
                }

                // Commit damage & element erosion + detached fragment extraction
                let mut newly_eroded = Vec::new();
                let pre_event_active: Vec<usize> = body
                    .active_elements
                    .iter()
                    .enumerate()
                    .filter_map(|(idx, &a)| if a { Some(idx) } else { None })
                    .collect();
                for &(e_idx, new_ds, d_gf, eroded_now) in &trial_damage_updates[b] {
                    if !body.crack_init_logged && new_ds.damage > 1e-4 {
                        body.crack_init_logged = true;
                        events.push(DynamicTimelineEvent {
                            time_s: t_next,
                            kind: "crack_initiation".to_string(),
                            part_id: Some(body.part_id.clone()),
                            description: format!(
                                "Local damage/crack initiated in '{}' (elem {}, D = {:.3})",
                                body.part_id, e_idx, new_ds.damage
                            ),
                        });
                    }
                    body.damage_states[e_idx] = new_ds;
                    body.cumulative_fracture_energy_j += d_gf;
                    if eroded_now && body.active_elements[e_idx] {
                        body.active_elements[e_idx] = false;
                        body.eroded_elements.push(e_idx);
                        newly_eroded.push(e_idx);
                    }
                }

                if !newly_eroded.is_empty() {
                    events.push(DynamicTimelineEvent {
                        time_s: t_next,
                        kind: "element_erosion".to_string(),
                        part_id: Some(body.part_id.clone()),
                        description: format!(
                            "Eroded {} element(s) in '{}' (total eroded: {})",
                            newly_eroded.len(),
                            body.part_id,
                            body.eroded_elements.len()
                        ),
                    });

                    // Detect detached connected components using exact momentum/inertia kinematics
                    let def_pos = body.deformed_positions_slice();
                    let nod_vel = body.nodal_velocities_slice();
                    let empty_splits = HashSet::new();
                    let (detached, _cons) = partition_fractured_body(
                        b,
                        &body.part_id,
                        total_steps + 1,
                        &body.mesh,
                        &def_pos,
                        &nod_vel,
                        body.density_kg_m3,
                        &mut body.active_elements,
                        &empty_splits,
                        &body.anchored_nodes,
                        &pre_event_active,
                        &newly_eroded,
                        options.min_fragment_elements,
                        &mut body.next_fragment_counter,
                        Some(&body.element_volumes),
                    );
                    if !detached.is_empty() {
                        for frag in &detached {
                            events.push(DynamicTimelineEvent {
                                time_s: t_next,
                                kind: "fragment_detachment".to_string(),
                                part_id: Some(body.part_id.clone()),
                                description: format!(
                                    "Detached fragment '{}' ({} elems, {:.4} kg)",
                                    frag.fragment_id,
                                    frag.element_indices.len(),
                                    frag.kinematics.mass_kg
                                ),
                            });
                        }
                        all_detached_fragments.extend(detached);
                    }
                    body.recompute_active_nodes();
                    body.rebuild_mass_and_gravity(options.mass_matrix_kind);
                }
            }

            // Log first contact impact event
            let has_contact = !final_contacts.is_empty();
            if has_contact && !prev_had_contact {
                let peak_fn = final_contacts
                    .iter()
                    .map(|c| c.normal_force_n)
                    .fold(0.0_f64, f64::max);
                events.push(DynamicTimelineEvent {
                    time_s: t_next,
                    kind: "contact_impact".to_string(),
                    part_id: None,
                    description: format!(
                        "Dynamic contact engaged ({} active points, peak F_n = {:.2} N)",
                        final_contacts.len(),
                        peak_fn
                    ),
                });
            }
            prev_had_contact = has_contact;

            t_curr = t_next;
            total_steps += 1;
            total_newton_iterations += iters_used;
            min_dt_used = min_dt_used.min(step_dt);
            max_dt_used = max_dt_used.max(step_dt);

            // Compute energy balance & drift
            let ke: f64 = bodies.iter().map(BodyRuntime::kinetic_energy).sum();
            let ef: f64 = bodies.iter().map(|b| b.cumulative_fracture_energy_j).sum();
            let mech_e = ke + final_we;
            if initial_mech_energy > 1e-18 {
                let drift_pct =
                    ((mech_e - initial_mech_energy).abs() / initial_mech_energy) * 100.0;
                max_energy_drift_pct = max_energy_drift_pct.max(drift_pct);
            }

            let is_final_step = t_curr >= options.total_time_s - 1e-12;
            let should_record = frames.len() < max_frames
                && (is_final_step
                    || options.output_interval_s <= 0.0
                    || (t_curr - last_frame_time) >= options.output_interval_s - 1e-12);

            if should_record {
                last_frame_time = t_curr;
                let frame_idx = frames.len();
                let body_frames: Vec<DynamicBodyFrame> =
                    bodies.iter().map(BodyRuntime::build_frame_snapshot).collect();
                frames.push(DynamicFrameSnapshot {
                    frame_index: frame_idx,
                    time_s: t_curr,
                    dt_used_s: step_dt,
                    newton_iterations: iters_used,
                    bodies: body_frames,
                    contact_forces: final_contacts,
                    detached_fragments: all_detached_fragments.clone(),
                    energy: DynamicEnergyBalance {
                        kinetic_j: ke,
                        elastic_j: final_we,
                        plastic_j: final_wp,
                        fracture_j: ef,
                        external_work_j: cumulative_ext_work,
                        total_mechanical_j: mech_e,
                        total_energy_j: mech_e + final_wp + ef,
                    },
                    hub_reactions: step_hub_reactions,
                });
            }

            if let Some(cb) = progress_cb.as_mut() {
                let total_eroded: usize = bodies.iter().map(|b| b.eroded_elements.len()).sum();
                let peak_vm = frames
                    .last()
                    .map(|f| {
                        f.bodies
                            .iter()
                            .map(|b| b.max_von_mises_pa)
                            .fold(0.0_f64, f64::max)
                    })
                    .unwrap_or(0.0);
                cb(DynamicProgressUpdate {
                    time_s: t_curr,
                    total_time_s: options.total_time_s,
                    progress_pct: ((t_curr / options.total_time_s) * 100.0).clamp(0.0, 100.0),
                    step_count: total_steps,
                    frame_count: frames.len(),
                    current_dt_s: step_dt,
                    bisections: total_bisections,
                    eroded_elements: total_eroded,
                    detached_fragments: all_detached_fragments.len(),
                    max_von_mises_pa: peak_vm,
                });
            }

            // Smooth adaptive step growth after easy convergence
            if step_bisections == 0 && iters_used <= 4 && max_damage_jump < 0.05 {
                dt = (step_dt * 1.5).min(dt_max);
            }

            step_accepted = true;
        }
    }

    if !min_dt_used.is_finite() {
        min_dt_used = options.dt_initial_s;
    }
    let final_mech_energy = frames
        .last()
        .map(|f| f.energy.total_mechanical_j)
        .unwrap_or(initial_mech_energy);
    let cfl_ratio_max = if cfl_critical_dt_s > 1e-18 {
        max_dt_used / cfl_critical_dt_s
    } else {
        0.0
    };

    Ok(DynamicHhtReport {
        frames,
        events,
        all_detached_fragments,
        total_steps,
        total_newton_iterations,
        total_bisections,
        min_dt_used_s: min_dt_used,
        max_dt_used_s: max_dt_used,
        cfl_critical_dt_s,
        cfl_ratio_max,
        initial_total_energy_j: initial_mech_energy,
        final_total_energy_j: final_mech_energy,
        max_energy_drift_pct,
        cancelled,
        elapsed_ms: start_wall.elapsed().as_secs_f64() * 1000.0,
    })
}

/// Convenience helper: maps a `part_id -> DynamicBodyFrame` for a given frame index.
pub fn frame_bodies_by_id(frame: &DynamicFrameSnapshot) -> HashMap<&str, &DynamicBodyFrame> {
    frame
        .bodies
        .iter()
        .map(|b| (b.part_id.as_str(), b))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tet10_consistent_and_hrz_mass_conservation() {
        let mut total_consistent = 0.0;
        for a in 0..10 {
            for b in 0..10 {
                total_consistent += tet10_consistent_mass_scalar(a, b);
            }
        }
        assert!(
            (total_consistent - 1.0).abs() < 1e-14,
            "TET10 consistent mass entries must sum to 1.0, got {total_consistent}"
        );

        let total_hrz: f64 = (0..10).map(tet10_hrz_lumped_fraction).sum();
        assert!(
            (total_hrz - 1.0).abs() < 1e-14,
            "TET10 HRZ lumped mass fractions must sum to 1.0, got {total_hrz}"
        );
        for a in 0..10 {
            assert!(tet10_hrz_lumped_fraction(a) > 0.0);
        }
    }

    #[test]
    fn test_hht_parameters_unconditional_stability_bounds() {
        let opts_trapezoidal = HhtOptions {
            alpha: 0.0,
            ..HhtOptions::default()
        };
        assert!((opts_trapezoidal.beta() - 0.25).abs() < 1e-15);
        assert!((opts_trapezoidal.gamma() - 0.5).abs() < 1e-15);

        let opts_damped = HhtOptions {
            alpha: -0.3,
            ..HhtOptions::default()
        };
        assert!((opts_damped.beta() - 0.4225).abs() < 1e-15);
        assert!((opts_damped.gamma() - 0.8).abs() < 1e-15);

        // Out-of-range alpha clamped safely into [-0.3, 0.0]
        let opts_clamped = HhtOptions {
            alpha: -0.9,
            ..HhtOptions::default()
        };
        assert!((opts_clamped.clamped_alpha() - (-0.3)).abs() < 1e-15);
    }
}
