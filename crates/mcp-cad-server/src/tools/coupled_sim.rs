//! Server-side Coupled Implicit `HHT-α` Dynamic FEA Baking, Per-Frame Caching,
//! SSE Progress Streaming, and MCP Tools (`bake_coupled_simulation`, `query_simulation_frame`) (`P5-T6`).

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use tokio::sync::broadcast;
use uuid::Uuid;

use mcp_cad_core::part::Part;
use mcp_cad_fea::boundary::{
    AabbFace, BoundaryConditions, DirichletConstraint, DofComponent, HubNodeSelector, NeumannLoad,
};
use mcp_cad_fea::conforming_mesh::AccuracyMode;
use mcp_cad_fea::contact::ContactPairSpec;
use mcp_cad_fea::damage::DamageMaterialParams;
use mcp_cad_fea::dynamics::{
    solve_dynamic_hht_coupled_with_frames, DynamicBodySpec, DynamicContactForce,
    DynamicEnergyBalance, DynamicFrameSnapshot, DynamicHhtReport, DynamicHubCoupling,
    DynamicHubReaction, DynamicPrismaticDrive, DynamicProgressUpdate, DynamicRevoluteDrive,
    DynamicTimelineEvent, HhtOptions, MassMatrixKind, TimeForceCurve,
};
use mcp_cad_fea::elasticity::J2MaterialParams;
use mcp_cad_fea::sparse_solver::LinearSolverKind;
use mcp_cad_fea::tetra_mesh::{MeshDensity, TetrahedralMesh};
use mcp_cad_physics::{
    ColliderType, FragmentSpawnSpec, GearJointCouplingSpec, HubJointKind, JointHubCouplingSpec,
    PhysicsWorld,
};

use crate::error::ServerError;
use crate::live_sync::{
    now_unix_ms, publish_live_coupled_sim_progress, LiveCoupledSimProgress,
};
use crate::state::{EngineState, LengthUnit};

/// Maximum number of completed coupled simulations retained in the in-memory frame cache.
const MAX_CACHED_SIMULATIONS: usize = 8;

/// Accuracy presets for server-side coupled dynamic HHT-α baking (`P5-T6`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CoupledAccuracyLevel {
    /// Fast engineering turnaround (`TET4`, `MeshDensity::Coarse`, `AccuracyMode::Fast`).
    #[default]
    Standard,
    /// High-resolution boundary-snapped volumetric mesh (`MeshDensity::Medium`, `AccuracyMode::Standard`).
    High,
    /// Curved quadratic `TET10` boundary-conforming mesh (`MeshDensity::Medium`, `AccuracyMode::High`).
    Ultra,
}

impl CoupledAccuracyLevel {
    /// Parses `"standard"`, `"high"`, or `"ultra"` (case-insensitive).
    pub fn parse(s: &str) -> Result<Self, ServerError> {
        match s.trim().to_ascii_lowercase().as_str() {
            "standard" | "std" => Ok(Self::Standard),
            "high" => Ok(Self::High),
            "ultra" => Ok(Self::Ultra),
            other => Err(ServerError::InvalidArgument(format!(
                "Invalid 'accuracy_level' value '{other}': expected 'standard', 'high', or 'ultra'"
            ))),
        }
    }

    /// Canonical lowercase label.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Standard => "standard",
            Self::High => "high",
            Self::Ultra => "ultra",
        }
    }

    /// Default tetrahedral mesh density and boundary-conforming accuracy mode for this preset.
    pub fn mesh_preset(&self) -> (MeshDensity, AccuracyMode) {
        match self {
            Self::Standard => (MeshDensity::Coarse, AccuracyMode::Fast),
            Self::High => (MeshDensity::Medium, AccuracyMode::Standard),
            Self::Ultra => (MeshDensity::Coarse, AccuracyMode::High),
        }
    }

    /// Default HHT-α solver parameters tuned for this accuracy level.
    pub fn default_hht_options(&self, duration_s: f64) -> HhtOptions {
        let mut opts = HhtOptions {
            total_time_s: duration_s,
            ..HhtOptions::default()
        };
        match self {
            Self::Standard => {
                opts.alpha = -0.05;
                opts.dt_initial_s = (duration_s / 20.0).clamp(1e-4, 2e-3);
                opts.dt_min_s = 1e-6;
                opts.dt_max_s = (duration_s / 5.0).clamp(opts.dt_initial_s, 1e-2);
                opts.force_tolerance = 5e-5;
                opts.max_frames = 300;
            }
            Self::High => {
                opts.alpha = -0.05;
                opts.dt_initial_s = (duration_s / 30.0).clamp(5e-5, 1.5e-3);
                opts.dt_min_s = 5e-7;
                opts.dt_max_s = (duration_s / 8.0).clamp(opts.dt_initial_s, 5e-3);
                opts.force_tolerance = 3e-5;
                opts.max_frames = 600;
            }
            Self::Ultra => {
                opts.alpha = -0.05;
                opts.dt_initial_s = (duration_s / 30.0).clamp(5e-5, 1.5e-3);
                opts.dt_min_s = 1e-7;
                opts.dt_max_s = (duration_s / 8.0).clamp(opts.dt_initial_s, 5e-3);
                opts.force_tolerance = 2.5e-5;
                opts.max_frames = 1200;
            }
        }
        opts
    }
}

/// Real-time SSE progress / lifecycle event emitted during coupled dynamic simulation baking.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub enum CoupledSimSseEvent {
    /// Emitted immediately when a bake job starts.
    Started {
        job_id: String,
        simulation_id: String,
        accuracy_level: String,
        part_ids: Vec<String>,
        total_time_s: f64,
    },
    /// Emitted periodically as implicit HHT-α steps converge.
    Progress {
        job_id: String,
        simulation_id: String,
        time_s: f64,
        total_time_s: f64,
        progress_pct: f64,
        step_count: usize,
        frame_count: usize,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        latest_frame_index: Option<usize>,
        current_dt_s: f64,
        bisections: usize,
        eroded_elements: usize,
        detached_fragments: usize,
        max_von_mises_mpa: f64,
        elapsed_ms: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        estimated_remaining_ms: Option<f64>,
    },
    /// Emitted when baking finishes and all frames are cached.
    Completed {
        job_id: String,
        simulation_id: String,
        frame_count: usize,
        total_steps: usize,
        total_bisections: usize,
        cfl_ratio_max: f64,
        max_energy_drift_pct: f64,
        elapsed_ms: f64,
    },
    /// Emitted when an active bake job was cancelled via API/MCP.
    Cancelled {
        job_id: String,
        simulation_id: String,
        frame_count: usize,
        elapsed_ms: f64,
    },
    /// Emitted if baking terminates with an error.
    Failed {
        job_id: String,
        simulation_id: String,
        error: String,
    },
}

/// Static mesh topology metadata for a part included in a baked simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BakedPartTopology {
    pub part_id: String,
    pub node_count: usize,
    pub element_count: usize,
    pub element_order: String,
    /// Reference undeformed nodal coordinates in meters.
    pub reference_positions_m: Vec<[f64; 3]>,
}

/// Compact outward-oriented exterior surface topology of the surviving active elements
/// of a tetrahedral mesh (`TET4` or `TET10`) at a timeline frame (`P5-T15`).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ActiveSurfaceTopology {
    /// Mapping from compacted surface vertex index `k in 0..surface_vertex_count`
    /// to the global FEM node index `nid in 0..node_count`.
    pub surface_node_indices: Vec<u32>,
    /// Flat triangle vertex indices referencing `surface_node_indices` (`0..surface_vertex_count`),
    /// strictly outward-oriented (CCW).
    pub surface_indices: Vec<u32>,
}

/// Cached outward-oriented, COM-centered triangle mesh of a detached fractured fragment (`P5-T15`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BakedFragmentMesh {
    pub fragment_id: String,
    pub source_part_id: String,
    /// Local-space vertex coordinates centered at the fragment COM (meters).
    pub vertices_m: Vec<[f64; 3]>,
    /// Outward unit vertex normals aligned 1:1 with `vertices_m`.
    pub normals: Vec<[f64; 3]>,
    /// Flat triangle vertex indices (length = `3 * triangle_count`, outward CCW winding).
    pub indices: Vec<u32>,
    /// Maximum vertex distance from the fragment local COM (meters).
    pub bounding_radius_m: f64,
    pub vertex_count: usize,
    pub triangle_count: usize,
}

impl BakedFragmentMesh {
    /// Builds a cached local-space fragment mesh directly from a [`DetachedFragment`].
    pub fn from_detached_fragment(frag: &mcp_cad_fea::fracture::DetachedFragment) -> Self {
        let mesh = &frag.local_surface_mesh;
        let vertices_m: Vec<[f64; 3]> = mesh.vertices.iter().map(|v| v.position).collect();
        let normals: Vec<[f64; 3]> = mesh.vertices.iter().map(|v| v.normal).collect();
        let mut indices = Vec::with_capacity(mesh.indices.len() * 3);
        for tri in &mesh.indices {
            indices.push(tri[0]);
            indices.push(tri[1]);
            indices.push(tri[2]);
        }
        let bounding_radius_m = vertices_m
            .iter()
            .map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt())
            .fold(0.0_f64, f64::max);
        Self {
            fragment_id: frag.fragment_id.clone(),
            source_part_id: frag.source_part_id.clone(),
            vertex_count: vertices_m.len(),
            triangle_count: mesh.indices.len(),
            vertices_m,
            normals,
            indices,
            bounding_radius_m,
        }
    }
}

/// Compact per-part field cache at a single timeline frame.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BakedPartFrame {
    pub part_id: String,
    pub node_count: usize,
    /// Deformed nodal world coordinates $\mathbf{x}(t) = \mathbf{X} + \mathbf{u}(t)$ in meters.
    pub deformed_positions_m: Vec<[f64; 3]>,
    /// Nodal displacement vectors $\mathbf{u}(t)$ in meters.
    pub displacements_m: Vec<[f64; 3]>,
    /// Nodal von Mises stress $\sigma_{vM}$ in MPa.
    pub nodal_von_mises_mpa: Vec<f64>,
    /// Nodal equivalent plastic strain $\bar{\varepsilon}_p$ (dimensionless).
    pub nodal_equiv_plastic_strain: Vec<f64>,
    /// Nodal continuum damage $D \in [0, 1]$.
    pub nodal_damage: Vec<f64>,
    /// Indices of currently eroded elements.
    pub eroded_element_indices: Vec<usize>,
    /// Count of surviving active elements.
    pub active_element_count: usize,
    /// Peak nodal displacement magnitude in mm.
    pub max_displacement_mm: f64,
    /// Peak von Mises stress in MPa.
    pub max_von_mises_mpa: f64,
    /// Peak equivalent plastic strain.
    pub max_equiv_plastic_strain: f64,
    /// Peak scalar damage $D \in [0, 1]$.
    pub max_damage: f64,
    /// Index into [`BakedSimulationRecord::surface_topologies`] for this part's active exterior surface (`P5-T15`).
    #[serde(default)]
    pub surface_topology_index: usize,
}

/// Compact per-fragment rigid-body state at a single timeline frame (updated via `PhysicsWorld`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BakedFragmentFrame {
    pub fragment_id: String,
    pub source_part_id: String,
    pub detachment_step: usize,
    pub mass_kg: f64,
    pub position_m: [f64; 3],
    pub orientation_quat: [f64; 4],
    pub linear_velocity_m_s: [f64; 3],
    pub angular_velocity_rad_s: [f64; 3],
    pub kinetic_energy_j: f64,
    pub element_count: usize,
    #[serde(default)]
    pub bounding_radius_m: f64,
}

/// Summary of a `PhysicsWorld` joint/motor automatically mapped into an `HHT-α` dynamic hub coupling (`P5-T14`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppliedJointCouplingSummary {
    pub joint_id: String,
    pub part_id: String,
    pub kind: String,
    pub axis_world: [f64; 3],
    pub pivot_world: [f64; 3],
    pub motor_target_velocity: Option<f64>,
    pub motor_max_effort: Option<f64>,
    pub coupled_node_count: usize,
}

/// Complete cached frame served by `query_simulation_frame` and `/api/coupled-sim/frames/*`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BakedFrameCache {
    pub frame_index: usize,
    pub time_s: f64,
    pub dt_used_s: f64,
    pub newton_iterations: usize,
    pub parts: Vec<BakedPartFrame>,
    pub contact_forces: Vec<DynamicContactForce>,
    pub detached_fragments: Vec<BakedFragmentFrame>,
    pub energy: DynamicEnergyBalance,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub hub_reactions: Vec<DynamicHubReaction>,
}

/// Complete cached coupled dynamic simulation stored on the server for timeline scrubbing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BakedSimulationRecord {
    pub simulation_id: String,
    pub job_id: String,
    pub accuracy_level: CoupledAccuracyLevel,
    pub part_ids: Vec<String>,
    pub duration_s: f64,
    pub topologies: Vec<BakedPartTopology>,
    /// Deduplicated active exterior surface topologies referenced by [`BakedPartFrame::surface_topology_index`] (`P5-T15`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub surface_topologies: Vec<ActiveSurfaceTopology>,
    /// Cached local-space triangle meshes for all detached fragments across the simulation (`P5-T15`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fragment_meshes: Vec<BakedFragmentMesh>,
    pub frames: Vec<BakedFrameCache>,
    pub events: Vec<DynamicTimelineEvent>,
    pub total_steps: usize,
    pub total_newton_iterations: usize,
    pub total_bisections: usize,
    pub min_dt_used_s: f64,
    pub max_dt_used_s: f64,
    pub cfl_critical_dt_s: f64,
    pub cfl_ratio_max: f64,
    pub initial_total_energy_j: f64,
    pub final_total_energy_j: f64,
    pub max_energy_drift_pct: f64,
    pub cancelled: bool,
    pub elapsed_ms: f64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub applied_joint_couplings: Vec<AppliedJointCouplingSummary>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub applied_gear_couplings: Vec<GearJointCouplingSpec>,
}

/// Lifecycle state of an active or finished coupled simulation bake job.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BakeJobState {
    Running,
    Cancelling,
    Completed,
    Cancelled,
    Failed,
}

/// Estimates remaining wall-clock solve time in milliseconds from elapsed time and `%` completion.
fn compute_estimated_remaining_ms(elapsed_ms: f64, progress_pct: f64) -> Option<f64> {
    if !elapsed_ms.is_finite() || !progress_pct.is_finite() {
        return None;
    }
    if progress_pct >= 99.9 {
        return Some(0.0);
    }
    if progress_pct >= 1.5 && elapsed_ms >= 50.0 {
        let remaining = (elapsed_ms * (100.0 - progress_pct)) / progress_pct;
        if remaining.is_finite() && remaining >= 0.0 {
            return Some((remaining * 10.0).round() / 10.0);
        }
    }
    None
}

/// Tracked status of a single bake job (for polling via MCP `status` or HTTP `/api/coupled-sim/status`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoupledJobStatus {
    pub job_id: String,
    pub simulation_id: String,
    pub state: BakeJobState,
    pub accuracy_level: CoupledAccuracyLevel,
    pub part_ids: Vec<String>,
    pub total_time_s: f64,
    pub progress_pct: f64,
    pub current_time_s: f64,
    pub step_count: usize,
    pub frame_count: usize,
    pub frames_cached: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub latest_frame_index: Option<usize>,
    pub current_dt_s: f64,
    pub bisections: usize,
    pub eroded_elements: usize,
    pub detached_fragments: usize,
    pub max_von_mises_mpa: f64,
    pub elapsed_ms: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub estimated_remaining_ms: Option<f64>,
    pub stage_label: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// Thread-safe server manager holding active bake jobs, cancellation tokens, and cached frame records.
pub struct CoupledSimulationManager {
    jobs: HashMap<String, CoupledJobStatus>,
    job_starts: HashMap<String, Instant>,
    cancel_flags: HashMap<String, Arc<AtomicBool>>,
    simulations: HashMap<String, Arc<BakedSimulationRecord>>,
    active_builds: HashMap<String, Arc<RwLock<BakedSimulationRecord>>>,
    active_sim_order: Vec<String>,
    simulation_order: Vec<String>,
    latest_simulation_id: Option<String>,
    latest_job_id: Option<String>,
}

impl Default for CoupledSimulationManager {
    fn default() -> Self {
        Self::new()
    }
}

impl CoupledSimulationManager {
    /// Creates an empty coupled simulation manager.
    pub fn new() -> Self {
        Self {
            jobs: HashMap::new(),
            job_starts: HashMap::new(),
            cancel_flags: HashMap::new(),
            simulations: HashMap::new(),
            active_builds: HashMap::new(),
            active_sim_order: Vec::new(),
            simulation_order: Vec::new(),
            latest_simulation_id: None,
            latest_job_id: None,
        }
    }

    /// Registers a newly started job and returns its atomic cancellation flag.
    pub fn register_job(
        &mut self,
        job_id: String,
        simulation_id: String,
        accuracy_level: CoupledAccuracyLevel,
        part_ids: Vec<String>,
        total_time_s: f64,
    ) -> Arc<AtomicBool> {
        let cancel_flag = Arc::new(AtomicBool::new(false));
        let status = CoupledJobStatus {
            job_id: job_id.clone(),
            simulation_id,
            state: BakeJobState::Running,
            accuracy_level,
            part_ids,
            total_time_s,
            progress_pct: 0.0,
            current_time_s: 0.0,
            step_count: 0,
            frame_count: 0,
            frames_cached: 0,
            latest_frame_index: None,
            current_dt_s: 0.0,
            bisections: 0,
            eroded_elements: 0,
            detached_fragments: 0,
            max_von_mises_mpa: 0.0,
            elapsed_ms: Some(0.0),
            estimated_remaining_ms: None,
            stage_label: "Инициализация HHT-α сетки и связей...".to_string(),
            error: None,
        };
        self.jobs.insert(job_id.clone(), status);
        self.job_starts.insert(job_id.clone(), Instant::now());
        self.cancel_flags
            .insert(job_id.clone(), Arc::clone(&cancel_flag));
        self.latest_job_id = Some(job_id);
        cancel_flag
    }

    /// Registers an active incremental `BakedSimulationRecord` so frames can be queried
    /// live while the solve is still in progress.
    pub fn begin_live_build(
        &mut self,
        sim_id: String,
        initial_record: BakedSimulationRecord,
    ) -> Arc<RwLock<BakedSimulationRecord>> {
        let handle = Arc::new(RwLock::new(initial_record));
        self.active_builds.insert(sim_id.clone(), Arc::clone(&handle));
        self.active_sim_order.retain(|id| id != &sim_id);
        self.active_sim_order.push(sim_id.clone());
        self.latest_simulation_id = Some(sim_id);
        handle
    }

    /// Updates live progress metrics for a running job and returns elapsed/ETA snapshot.
    pub fn update_progress(
        &mut self,
        job_id: &str,
        update: &DynamicProgressUpdate,
    ) -> (f64, Option<f64>, Option<usize>) {
        let elapsed_ms = self
            .job_starts
            .get(job_id)
            .map(|t| (t.elapsed().as_secs_f64() * 1000.0 * 10.0).round() / 10.0)
            .unwrap_or(0.0);
        let eta_ms = compute_estimated_remaining_ms(elapsed_ms, update.progress_pct);
        let mut latest_frame_idx = update.frame_count.checked_sub(1);

        if let Some(job) = self.jobs.get_mut(job_id) {
            job.current_time_s = update.time_s;
            job.progress_pct = update.progress_pct;
            job.step_count = update.step_count;
            job.frame_count = update.frame_count;
            job.current_dt_s = update.current_dt_s;
            job.bisections = update.bisections;
            job.eroded_elements = update.eroded_elements;
            job.detached_fragments = update.detached_fragments;
            job.max_von_mises_mpa = update.max_von_mises_pa * 1e-6;
            job.elapsed_ms = Some(elapsed_ms);
            job.estimated_remaining_ms = eta_ms;
            if job.frames_cached > 0 {
                latest_frame_idx = Some(job.frames_cached - 1);
            }
            job.latest_frame_index = latest_frame_idx;
            job.stage_label = format!(
                "HHT-α: {:.0}% · шаг {} · кадр {}",
                update.progress_pct,
                update.step_count,
                job.frames_cached.max(update.frame_count)
            );
        }
        (elapsed_ms, eta_ms, latest_frame_idx)
    }

    /// Records that a new incremental frame has been converted and cached for live queries.
    pub fn update_live_frame_count(
        &mut self,
        job_id: &str,
        frames_cached: usize,
        latest_frame_index: usize,
    ) {
        if let Some(job) = self.jobs.get_mut(job_id) {
            job.frames_cached = frames_cached;
            job.frame_count = job.frame_count.max(frames_cached);
            job.latest_frame_index = Some(latest_frame_index);
            if let Some(t0) = self.job_starts.get(job_id) {
                job.elapsed_ms = Some((t0.elapsed().as_secs_f64() * 1000.0 * 10.0).round() / 10.0);
            }
        }
    }

    /// Signals cancellation for a running job. Returns `true` if the job was found.
    pub fn cancel_job(&mut self, job_id: &str) -> bool {
        if let Some(flag) = self.cancel_flags.get(job_id) {
            flag.store(true, Ordering::Relaxed);
            if let Some(job) = self.jobs.get_mut(job_id) {
                if job.state == BakeJobState::Running {
                    job.state = BakeJobState::Cancelling;
                    job.stage_label = "Остановка расчёта...".to_string();
                }
            }
            return true;
        }
        false
    }

    /// Stores a completed (or partially cancelled) baked simulation record into the server frame cache.
    pub fn finish_job_with_record(&mut self, job_id: &str, record: BakedSimulationRecord) {
        let sim_id = record.simulation_id.clone();
        let cancelled = record.cancelled;
        let frame_count = record.frames.len();
        let elapsed_ms = record.elapsed_ms;
        let step_count = record.total_steps;
        let bisections = record.total_bisections;
        let latest_frame_idx = frame_count.checked_sub(1);

        self.active_builds.remove(&sim_id);
        self.active_sim_order.retain(|id| id != &sim_id);

        if let Some(job) = self.jobs.get_mut(job_id) {
            job.state = if cancelled {
                BakeJobState::Cancelled
            } else {
                BakeJobState::Completed
            };
            job.progress_pct = if cancelled { job.progress_pct } else { 100.0 };
            job.frame_count = frame_count;
            job.frames_cached = frame_count;
            job.latest_frame_index = latest_frame_idx;
            job.step_count = step_count;
            job.bisections = bisections;
            job.elapsed_ms = Some(elapsed_ms);
            job.estimated_remaining_ms = Some(0.0);
            job.stage_label = if cancelled {
                format!("Остановлено (сохранено кадров: {frame_count})")
            } else {
                format!("Расчёт завершён ({frame_count} кадр.)")
            };
        }

        self.simulations.insert(sim_id.clone(), Arc::new(record));
        self.simulation_order.retain(|id| id != &sim_id);
        self.simulation_order.push(sim_id.clone());
        self.latest_simulation_id = Some(sim_id);

        while self.simulation_order.len() > MAX_CACHED_SIMULATIONS {
            let oldest = self.simulation_order.remove(0);
            self.simulations.remove(&oldest);
        }
    }

    /// Marks a job as failed with an error message.
    pub fn fail_job(&mut self, job_id: &str, error: String) {
        if let Some(job) = self.jobs.get_mut(job_id) {
            let sim_id = job.simulation_id.clone();
            self.active_builds.remove(&sim_id);
            self.active_sim_order.retain(|id| id != &sim_id);
            job.state = BakeJobState::Failed;
            job.estimated_remaining_ms = None;
            job.stage_label = format!("Ошибка расчёта: {error}");
            job.error = Some(error);
        }
    }

    /// Returns job status for a specific `job_id` or the most recent job if `None`.
    pub fn job_status(&self, job_id: Option<&str>) -> Option<CoupledJobStatus> {
        let id = job_id.or(self.latest_job_id.as_deref())?;
        let mut st = self.jobs.get(id).cloned()?;
        if matches!(st.state, BakeJobState::Running | BakeJobState::Cancelling) {
            if let Some(t0) = self.job_starts.get(id) {
                let elapsed = (t0.elapsed().as_secs_f64() * 1000.0 * 10.0).round() / 10.0;
                st.elapsed_ms = Some(elapsed);
                st.estimated_remaining_ms = compute_estimated_remaining_ms(elapsed, st.progress_pct);
            }
        }
        Some(st)
    }

    /// Lists all tracked job statuses.
    pub fn list_jobs(&self) -> Vec<CoupledJobStatus> {
        self.jobs.keys().filter_map(|id| self.job_status(Some(id))).collect()
    }

    /// Retrieves a completed cached simulation record by ID or returns the latest cached simulation.
    pub fn get_simulation(&self, simulation_id: Option<&str>) -> Option<Arc<BakedSimulationRecord>> {
        let id = simulation_id.or(self.latest_simulation_id.as_deref())?;
        self.simulations.get(id).cloned()
    }

    /// Visits either a completed simulation or a currently active streaming simulation build.
    pub fn with_simulation_record<R>(
        &self,
        simulation_id: Option<&str>,
        f: impl FnOnce(&BakedSimulationRecord) -> R,
    ) -> Option<R> {
        if let Some(id) = simulation_id {
            if let Some(rec) = self.simulations.get(id) {
                return Some(f(rec));
            }
            if let Some(live) = self.active_builds.get(id) {
                let guard = live.read();
                return Some(f(&guard));
            }
            return None;
        }
        if let Some(latest_id) = self.latest_simulation_id.as_deref() {
            if let Some(live) = self.active_builds.get(latest_id) {
                let guard = live.read();
                return Some(f(&guard));
            }
            if let Some(rec) = self.simulations.get(latest_id) {
                return Some(f(rec));
            }
        }
        if let Some(active_id) = self.active_sim_order.last() {
            if let Some(live) = self.active_builds.get(active_id) {
                let guard = live.read();
                return Some(f(&guard));
            }
        }
        None
    }

    /// Lists summary metadata for all cached simulations.
    pub fn list_cached_simulation_ids(&self) -> Vec<String> {
        self.simulation_order.clone()
    }
}

/// Thread-safe handle to the coupled simulation cache & job manager.
pub type SharedCoupledSimManager = Arc<RwLock<CoupledSimulationManager>>;

/// Prepared self-contained inputs for executing an implicit `HHT-α` coupled bake off the `EngineState` lock.
#[derive(Debug, Clone)]
pub struct PreparedCoupledBake {
    pub job_id: String,
    pub simulation_id: String,
    pub accuracy_level: CoupledAccuracyLevel,
    pub bodies: Vec<DynamicBodySpec>,
    pub contact_pairs: Vec<ContactPairSpec>,
    pub options: HhtOptions,
    pub gravity: [f64; 3],
    pub async_mode: bool,
    pub async_mode_explicit: Option<bool>,
    pub wait_timeout_ms: Option<u64>,
    pub applied_joint_couplings: Vec<AppliedJointCouplingSummary>,
    pub applied_gear_couplings: Vec<GearJointCouplingSpec>,
}

/// Parses an AABB face label (`min_x`, `max_x`, etc.).
fn parse_aabb_face(s: &str) -> Option<AabbFace> {
    match s.to_lowercase().replace('-', "_").as_str() {
        "min_x" | "minx" | "-x" => Some(AabbFace::MinX),
        "max_x" | "maxx" | "+x" => Some(AabbFace::MaxX),
        "min_y" | "miny" | "-y" => Some(AabbFace::MinY),
        "max_y" | "maxy" | "+y" => Some(AabbFace::MaxY),
        "min_z" | "minz" | "-z" => Some(AabbFace::MinZ),
        "max_z" | "maxz" | "+z" => Some(AabbFace::MaxZ),
        _ => None,
    }
}

/// Parses a 3-element finite `f64` vector from JSON.
fn parse_vec3(v: &Value, field: &str) -> Result<[f64; 3], ServerError> {
    let arr = v.as_array().ok_or_else(|| {
        ServerError::InvalidArgument(format!("Invalid '{field}': expected [x, y, z] array"))
    })?;
    if arr.len() != 3 {
        return Err(ServerError::InvalidArgument(format!(
            "Invalid '{field}': expected 3 numbers, got {}",
            arr.len()
        )));
    }
    let mut out = [0.0; 3];
    for (i, item) in arr.iter().enumerate() {
        out[i] = item.as_f64().filter(|x| x.is_finite()).ok_or_else(|| {
            ServerError::InvalidArgument(format!(
                "Invalid '{field}': element {i} must be a finite number"
            ))
        })?;
    }
    Ok(out)
}

/// Parses optional `TimeForceCurve` from JSON object (`type`: `constant` | `ramp` | `pulse` | `sine`).
fn parse_force_curve(v: Option<&Value>) -> Result<TimeForceCurve, ServerError> {
    let Some(val) = v else {
        return Ok(TimeForceCurve::Constant(1.0));
    };
    if val.is_null() {
        return Ok(TimeForceCurve::Constant(1.0));
    }
    let kind = val
        .get("type")
        .and_then(|s| s.as_str())
        .unwrap_or("constant")
        .trim()
        .to_ascii_lowercase();
    match kind.as_str() {
        "constant" => {
            let amp = val.get("amplitude").and_then(|x| x.as_f64()).unwrap_or(1.0);
            Ok(TimeForceCurve::Constant(amp))
        }
        "ramp" => {
            let dur = val
                .get("duration_s")
                .and_then(|x| x.as_f64())
                .filter(|x| x.is_finite() && *x > 0.0)
                .ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Invalid 'force_curve' ramp: positive 'duration_s' is required".to_string(),
                    )
                })?;
            Ok(TimeForceCurve::Ramp { duration_s: dur })
        }
        "pulse" => {
            let start_s = val.get("start_s").and_then(|x| x.as_f64()).unwrap_or(0.0);
            let end_s = val
                .get("end_s")
                .and_then(|x| x.as_f64())
                .filter(|x| x.is_finite() && *x > start_s)
                .ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Invalid 'force_curve' pulse: 'end_s' must be > 'start_s'".to_string(),
                    )
                })?;
            let amplitude = val.get("amplitude").and_then(|x| x.as_f64()).unwrap_or(1.0);
            Ok(TimeForceCurve::Pulse {
                start_s,
                end_s,
                amplitude,
            })
        }
        "sine" => {
            let frequency_hz = val
                .get("frequency_hz")
                .and_then(|x| x.as_f64())
                .filter(|x| x.is_finite() && *x > 0.0)
                .ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Invalid 'force_curve' sine: positive 'frequency_hz' is required"
                            .to_string(),
                    )
                })?;
            let amplitude = val.get("amplitude").and_then(|x| x.as_f64()).unwrap_or(1.0);
            let phase_rad = val.get("phase_rad").and_then(|x| x.as_f64()).unwrap_or(0.0);
            Ok(TimeForceCurve::Sine {
                frequency_hz,
                amplitude,
                phase_rad,
            })
        }
        other => Err(ServerError::InvalidArgument(format!(
            "Invalid 'force_curve.type' '{other}': expected 'constant', 'ramp', 'pulse', or 'sine'"
        ))),
    }
}

/// Selects a robust, well-conditioned bore or core node set around `(pivot, axis)` for
/// attaching a `PhysicsWorld` joint (`Revolute`, `Prismatic`, or `Fixed`) as a dynamic RBE2 hub
/// while keeping outer functional features (such as gear teeth) free to deform and contact.
fn select_dynamic_hub_nodes_for_joint(
    tet_mesh: &TetrahedralMesh,
    pivot: [f64; 3],
    axis: [f64; 3],
) -> Vec<usize> {
    let n_nodes = tet_mesh.node_count();
    if n_nodes == 0 {
        return Vec::new();
    }

    let a_len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    let a = if a_len > 1e-12 {
        [axis[0] / a_len, axis[1] / a_len, axis[2] / a_len]
    } else {
        [0.0, 0.0, 1.0]
    };

    // Perpendicular radial distance of every node from the joint axis through `pivot`
    let mut node_radii: Vec<(usize, f64)> = Vec::with_capacity(n_nodes);
    let mut r_min = f64::INFINITY;
    let mut r_max = 0.0_f64;
    for node in &tet_mesh.nodes {
        let dp = [
            node.position[0] - pivot[0],
            node.position[1] - pivot[1],
            node.position[2] - pivot[2],
        ];
        let axial = dp[0] * a[0] + dp[1] * a[1] + dp[2] * a[2];
        let perp = [
            dp[0] - axial * a[0],
            dp[1] - axial * a[1],
            dp[2] - axial * a[2],
        ];
        let r = (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt();
        r_min = r_min.min(r);
        r_max = r_max.max(r);
        node_radii.push((node.id, r));
    }

    if r_max <= 1e-9 {
        return (0..n_nodes).collect();
    }

    // Estimate characteristic element edge length from a sample of elements
    let mut edge_sum = 0.0_f64;
    let mut edge_cnt = 0usize;
    for elem in tet_mesh.elements.iter().take(64) {
        let p0 = tet_mesh.nodes[elem.node_indices[0]].position;
        let p1 = tet_mesh.nodes[elem.node_indices[1]].position;
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        edge_sum += (dx * dx + dy * dy + dz * dz).sqrt();
        edge_cnt += 1;
    }
    let h_edge = if edge_cnt > 0 {
        edge_sum / edge_cnt as f64
    } else {
        0.1 * r_max
    };

    let radial_span = (r_max - r_min).max(1e-9);
    let is_hollow_bore = r_min > 0.05 * r_max;

    let mut selected: Vec<usize> = if is_hollow_bore {
        // Hollow bore (e.g. spur gear central bore): capture the inner bore wall ring while
        // staying strictly inside the root/tooth rim.
        let band = (0.35 * h_edge).max(1e-4).min(0.25 * radial_span);
        let r_cut = r_min + band;
        let mut pts: Vec<usize> = node_radii
            .iter()
            .filter(|&&(_, r)| r <= r_cut)
            .map(|&(id, _)| id)
            .collect();
        if pts.len() < 4 {
            let r_cut_wide = r_min + 0.35 * radial_span;
            pts = node_radii
                .iter()
                .filter(|&&(_, r)| r <= r_cut_wide)
                .map(|&(id, _)| id)
                .collect();
        }
        pts
    } else {
        // Solid hub / link: select an inner cylindrical core around the joint axis
        let r_cut = (0.20 * r_max).max(0.75 * h_edge).min(0.35 * r_max);
        node_radii
            .iter()
            .filter(|&&(_, r)| r <= r_cut)
            .map(|&(id, _)| id)
            .collect()
    };

    // Guarantee at least 4 nodes (when available) with non-degenerate radial lever arm
    if selected.len() < 4.min(n_nodes)
        || selected.iter().all(|&id| {
            node_radii
                .get(id)
                .is_none_or(|&(_, r)| r < 1e-6)
        })
    {
        let mut sorted = node_radii;
        sorted.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        let target_count = 6usize.min(n_nodes).max(4usize.min(n_nodes));
        selected = sorted
            .into_iter()
            .take(target_count)
            .map(|(id, _)| id)
            .collect();
    }

    selected.sort_unstable();
    selected.dedup();
    selected
}

/// Maps a [`JointHubCouplingSpec`] extracted from [`PhysicsWorld`] into a [`DynamicHubCoupling`]
/// for the implicit `HHT-α` integrator (`P5-T14`).
fn map_physics_joint_to_dynamic_hub(
    spec: &JointHubCouplingSpec,
    tet_mesh: &TetrahedralMesh,
    ramp_duration_s: f64,
) -> (DynamicHubCoupling, AppliedJointCouplingSummary) {
    let hub_nodes = select_dynamic_hub_nodes_for_joint(tet_mesh, spec.pivot_world, spec.axis_world);
    let coupled_node_count = hub_nodes.len();
    let selector = HubNodeSelector::Nodes(hub_nodes);

    let (coupling, kind_label) = match spec.kind {
        HubJointKind::Revolute => {
            let drive = if let Some(omega) = spec.motor_target_velocity {
                if let Some(max_t) = spec
                    .motor_max_effort
                    .filter(|v| v.is_finite() && *v > 0.0)
                {
                    DynamicRevoluteDrive::Motor {
                        target_velocity_rad_s: omega,
                        max_torque_nm: max_t,
                        velocity_gain_nm_s_per_rad: 0.0,
                        ramp_duration_s,
                        limits: spec.limits,
                    }
                } else {
                    DynamicRevoluteDrive::PrescribedVelocity {
                        angular_velocity_rad_s: omega,
                        initial_angle_rad: 0.0,
                        ramp_duration_s,
                        limits: spec.limits,
                    }
                }
            } else {
                // Passive bearing: free rotation with light physical bearing damping so
                // gear tooth contact transmits torque cleanly without zero-pivot chatter.
                DynamicRevoluteDrive::FreeTorque {
                    applied_torque_nm: 0.0,
                    viscous_damping_nm_s_per_rad: 0.005,
                    torsional_stiffness_nm_per_rad: 0.0,
                    reference_angle_rad: 0.0,
                    limits: spec.limits,
                }
            };
            (
                DynamicHubCoupling::Revolute {
                    name: Some(spec.joint_id.clone()),
                    pivot: spec.pivot_world,
                    axis: spec.axis_world,
                    selector,
                    drive,
                },
                "revolute".to_string(),
            )
        }
        HubJointKind::Prismatic => {
            let drive = if let Some(v_target) = spec.motor_target_velocity {
                if let Some(max_f) = spec
                    .motor_max_effort
                    .filter(|v| v.is_finite() && *v > 0.0)
                {
                    DynamicPrismaticDrive::Motor {
                        target_velocity_m_s: v_target,
                        max_force_n: max_f,
                        velocity_gain_n_s_per_m: 0.0,
                        ramp_duration_s,
                        limits: spec.limits,
                    }
                } else {
                    DynamicPrismaticDrive::PrescribedVelocity {
                        velocity_m_s: v_target,
                        initial_displacement_m: 0.0,
                        ramp_duration_s,
                        limits: spec.limits,
                    }
                }
            } else {
                DynamicPrismaticDrive::FreeForce {
                    applied_force_n: 0.0,
                    viscous_damping_n_s_per_m: 0.05,
                    axial_stiffness_n_per_m: 0.0,
                    reference_disp_m: 0.0,
                    limits: spec.limits,
                }
            };
            (
                DynamicHubCoupling::Prismatic {
                    name: Some(spec.joint_id.clone()),
                    axis: spec.axis_world,
                    selector,
                    drive,
                    lock_transverse: true,
                },
                "prismatic".to_string(),
            )
        }
        HubJointKind::Fixed => (
            DynamicHubCoupling::Fixed {
                name: Some(spec.joint_id.clone()),
                selector,
            },
            "fixed".to_string(),
        ),
    };

    let summary = AppliedJointCouplingSummary {
        joint_id: spec.joint_id.clone(),
        part_id: spec.part_id.clone(),
        kind: kind_label,
        axis_world: spec.axis_world,
        pivot_world: spec.pivot_world,
        motor_target_velocity: spec.motor_target_velocity,
        motor_max_effort: spec.motor_max_effort,
        coupled_node_count,
    };
    (coupling, summary)
}

/// Builds a [`PreparedCoupledBake`] directly from a map of CAD parts and JSON arguments
/// (without `PhysicsWorld` joint extraction).
pub fn prepare_coupled_bake_from_parts(
    args: &Value,
    parts: &HashMap<String, Part>,
    unit: LengthUnit,
) -> Result<PreparedCoupledBake, ServerError> {
    prepare_coupled_bake_from_parts_with_physics(args, parts, None, unit)
}

/// Builds a [`PreparedCoupledBake`] from CAD parts, JSON arguments, and an optional [`PhysicsWorld`].
///
/// When `physics_world` is provided and `use_physics_joints` is enabled (default `true`), active
/// `Revolute`, `Prismatic`, and `Fixed` joints (including any attached motors and limits) as well
/// as `Gear` relationships are automatically extracted from `physics_world` and mapped into
/// [`DynamicHubCoupling`] specifications on the corresponding [`DynamicBodySpec`] bodies (`P5-T14`).
pub fn prepare_coupled_bake_from_parts_with_physics(
    args: &Value,
    parts: &HashMap<String, Part>,
    physics_world: Option<&PhysicsWorld>,
    unit: LengthUnit,
) -> Result<PreparedCoupledBake, ServerError> {
    let accuracy_level = match args.get("accuracy_level").and_then(|v| v.as_str()) {
        Some(s) => CoupledAccuracyLevel::parse(s)?,
        None => CoupledAccuracyLevel::Standard,
    };

    let duration_s = args
        .get("duration_s")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.02);
    if !duration_s.is_finite() || duration_s <= 0.0 || duration_s > 600.0 {
        return Err(ServerError::InvalidArgument(format!(
            "Invalid 'duration_s' ({duration_s}): must be finite in (0, 600] seconds"
        )));
    }

    let mut options = accuracy_level.default_hht_options(duration_s);

    if let Some(alpha) = args.get("hht_alpha").and_then(|v| v.as_f64()) {
        if !alpha.is_finite() || !(-0.3..=0.0).contains(&alpha) {
            return Err(ServerError::InvalidArgument(format!(
                "Invalid 'hht_alpha' ({alpha}): must lie in [-0.3, 0.0] for unconditional stability"
            )));
        }
        options.alpha = alpha;
    }

    if let Some(dt_ms) = args.get("dt_ms").and_then(|v| v.as_f64()) {
        if !dt_ms.is_finite() || dt_ms <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "Invalid 'dt_ms': must be a positive finite number in milliseconds".to_string(),
            ));
        }
        options.dt_initial_s = dt_ms * 1e-3;
    }
    if let Some(dt_min_ms) = args.get("dt_min_ms").and_then(|v| v.as_f64()) {
        if !dt_min_ms.is_finite() || dt_min_ms <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "Invalid 'dt_min_ms': must be a positive finite number in milliseconds".to_string(),
            ));
        }
        options.dt_min_s = dt_min_ms * 1e-3;
    }
    if let Some(dt_max_ms) = args.get("dt_max_ms").and_then(|v| v.as_f64()) {
        if !dt_max_ms.is_finite() || dt_max_ms <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "Invalid 'dt_max_ms': must be a positive finite number in milliseconds".to_string(),
            ));
        }
        options.dt_max_s = dt_max_ms * 1e-3;
    }
    if options.dt_max_s < options.dt_initial_s {
        options.dt_max_s = options.dt_initial_s;
    }
    if options.dt_min_s > options.dt_initial_s {
        options.dt_min_s = (options.dt_initial_s * 0.01).max(1e-8);
    }

    if let Some(fps) = args.get("output_fps").and_then(|v| v.as_f64()) {
        if !fps.is_finite() || fps <= 0.0 {
            return Err(ServerError::InvalidArgument(
                "Invalid 'output_fps': must be a positive finite number".to_string(),
            ));
        }
        options.output_interval_s = 1.0 / fps;
    }
    if let Some(max_f) = args.get("max_frames").and_then(|v| v.as_u64()) {
        if max_f == 0 || max_f > 5000 {
            return Err(ServerError::InvalidArgument(
                "Invalid 'max_frames': must be an integer in [1, 5000]".to_string(),
            ));
        }
        options.max_frames = max_f as usize;
    }

    if let Some(mm) = args.get("mass_matrix").and_then(|v| v.as_str()) {
        options.mass_matrix_kind = match mm.trim().to_ascii_lowercase().as_str() {
            "consistent" => MassMatrixKind::Consistent,
            "lumped" => MassMatrixKind::Lumped,
            other => {
                return Err(ServerError::InvalidArgument(format!(
                    "Invalid 'mass_matrix' '{other}': expected 'consistent' or 'lumped'"
                )))
            }
        };
    }

    if let Some(ls) = args.get("linear_solver").and_then(|v| v.as_str()) {
        options.linear_solver = match ls.trim().to_ascii_lowercase().as_str() {
            "auto" => LinearSolverKind::Auto,
            "ldlt" | "sparse_direct" | "direct" => LinearSolverKind::SparseDirectLdlt,
            "ic_pcg" | "ic0_pcg" => LinearSolverKind::IcPcg,
            "ssor_pcg" => LinearSolverKind::SsorPcg,
            "jacobi_pcg" => LinearSolverKind::JacobiPcg,
            other => {
                return Err(ServerError::InvalidArgument(format!(
                    "Invalid 'linear_solver' '{other}': expected 'auto', 'ldlt', 'ic_pcg', 'ssor_pcg', or 'jacobi_pcg'"
                )))
            }
        };
    }

    if let Some(b) = args.get("enable_plasticity").and_then(|v| v.as_bool()) {
        options.enable_plasticity = b;
    }
    if let Some(b) = args.get("enable_fracture").and_then(|v| v.as_bool()) {
        options.enable_fracture = b;
    }
    if let Some(b) = args.get("corotated").and_then(|v| v.as_bool()) {
        options.corotated = b;
    }

    let enable_contact = args
        .get("enable_contact")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);

    let use_physics_joints = args
        .get("use_physics_joints")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);

    let gravity = match args.get("gravity") {
        Some(v) if !v.is_null() => parse_vec3(v, "gravity")?,
        _ => [0.0, 0.0, 0.0],
    };

    let force_curve = parse_force_curve(args.get("force_curve"))?;

    let async_mode_explicit = args
        .get("async_mode")
        .or_else(|| args.get("async"))
        .and_then(|v| v.as_bool());
    let async_mode = async_mode_explicit.unwrap_or(false);
    let wait_timeout_ms = args
        .get("wait_timeout_ms")
        .and_then(|v| v.as_u64())
        .map(|ms| ms.min(120_000));

    // Select parts to include
    let selected_ids: Vec<String> = if let Some(arr) = args.get("part_ids").and_then(|v| v.as_array())
    {
        if arr.is_empty() {
            return Err(ServerError::InvalidArgument(
                "Invalid 'part_ids': array must contain at least one part ID".to_string(),
            ));
        }
        let mut ids = Vec::with_capacity(arr.len());
        for item in arr {
            let pid = item.as_str().ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Invalid 'part_ids': all entries must be strings".to_string(),
                )
            })?;
            if !parts.contains_key(pid) {
                return Err(ServerError::PartNotFound(pid.to_string()));
            }
            ids.push(pid.to_string());
        }
        ids
    } else {
        let mut ids: Vec<String> = parts
            .iter()
            .filter(|(_, p)| !p.mesh.vertices.is_empty())
            .map(|(id, _)| id.clone())
            .collect();
        ids.sort();
        if ids.is_empty() {
            return Err(ServerError::InvalidArgument(
                "No non-empty parts exist in the scene to bake. Create parts and add geometry first."
                    .to_string(),
            ));
        }
        ids
    };

    // Per-part fixed faces map: accepts either simple strings ["min_x"] (applied to first part)
    // or objects [{"part_id": "p1", "face": "min_x", "tolerance": 0.001}].
    let mut fixed_by_part: HashMap<String, Vec<(String, Option<f64>)>> = HashMap::new();
    if let Some(arr) = args.get("fixed_faces").and_then(|v| v.as_array()) {
        for item in arr {
            if let Some(face_str) = item.as_str() {
                fixed_by_part
                    .entry(selected_ids[0].clone())
                    .or_default()
                    .push((face_str.to_string(), None));
            } else if let Some(obj) = item.as_object() {
                let pid = obj
                    .get("part_id")
                    .and_then(|v| v.as_str())
                    .unwrap_or(&selected_ids[0])
                    .to_string();
                let face_str = obj
                    .get("face")
                    .and_then(|v| v.as_str())
                    .ok_or_else(|| {
                        ServerError::InvalidArgument(
                            "Each object in 'fixed_faces' must specify 'face'".to_string(),
                        )
                    })?;
                let tol = obj
                    .get("tolerance")
                    .and_then(|v| v.as_f64())
                    .filter(|v| v.is_finite() && *v > 0.0);
                fixed_by_part
                    .entry(pid)
                    .or_default()
                    .push((face_str.to_string(), tol));
            }
        }
    }

    // Per-part loads: [{"part_id": ..., "face": "max_x", "force": [fx, fy, fz], "tolerance": 0.001}]
    let mut loads_by_part: HashMap<String, Vec<(String, [f64; 3], Option<f64>)>> = HashMap::new();
    if let Some(arr) = args.get("loads").and_then(|v| v.as_array()) {
        for item in arr {
            let pid = item
                .get("part_id")
                .and_then(|v| v.as_str())
                .unwrap_or(&selected_ids[0])
                .to_string();
            let face_str = item
                .get("face")
                .and_then(|v| v.as_str())
                .unwrap_or("max_x")
                .to_string();
            let force_val = item.get("force").ok_or_else(|| {
                ServerError::InvalidArgument(
                    "Each entry in 'loads' must specify 'force': [fx, fy, fz] in Newtons"
                        .to_string(),
                )
            })?;
            let force = parse_vec3(force_val, "loads[].force")?;
            let tol = item
                .get("tolerance")
                .and_then(|v| v.as_f64())
                .filter(|v| v.is_finite() && *v > 0.0);
            loads_by_part
                .entry(pid)
                .or_default()
                .push((face_str, force, tol));
        }
    }

    // Per-part initial uniform velocities: {"part_id": [vx, vy, vz]} or [{"part_id": ..., "velocity": [vx, vy, vz]}]
    let mut init_vel_by_part: HashMap<String, [f64; 3]> = HashMap::new();
    if let Some(iv) = args.get("initial_velocities") {
        if let Some(obj) = iv.as_object() {
            for (pid, vel_val) in obj {
                init_vel_by_part.insert(pid.clone(), parse_vec3(vel_val, "initial_velocities")?);
            }
        } else if let Some(arr) = iv.as_array() {
            for item in arr {
                let pid = item
                    .get("part_id")
                    .and_then(|v| v.as_str())
                    .unwrap_or(&selected_ids[0])
                    .to_string();
                let vel_val = item.get("velocity").ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "Each entry in 'initial_velocities' must specify 'velocity': [vx, vy, vz] in m/s"
                            .to_string(),
                    )
                })?;
                init_vel_by_part.insert(pid, parse_vec3(vel_val, "initial_velocities[].velocity")?);
            }
        }
    }

    let (default_density, acc_mode) = accuracy_level.mesh_preset();
    let mesh_density = match args.get("mesh_density").and_then(|v| v.as_str()) {
        Some("coarse") => MeshDensity::Coarse,
        Some("medium") => MeshDensity::Medium,
        Some("fine") => MeshDensity::Fine,
        Some(other) => {
            return Err(ServerError::InvalidArgument(format!(
                "Invalid 'mesh_density' '{other}': expected 'coarse', 'medium', or 'fine'"
            )))
        }
        None => default_density,
    };

    let _ = unit; // Future unit overrides for coordinate-based face selectors

    // Extract active PhysicsWorld joints, motors, and gear pairs when enabled (P5-T14)
    let selected_set: HashSet<&str> = selected_ids.iter().map(String::as_str).collect();
    let mut joints_by_part: HashMap<String, Vec<JointHubCouplingSpec>> = HashMap::new();
    let mut applied_gear_couplings: Vec<GearJointCouplingSpec> = Vec::new();
    if use_physics_joints {
        if let Some(pw) = physics_world {
            for jspec in pw.extract_joint_hub_couplings() {
                if selected_set.contains(jspec.part_id.as_str()) {
                    joints_by_part
                        .entry(jspec.part_id.clone())
                        .or_default()
                        .push(jspec);
                }
            }
            for gspec in pw.extract_gear_joint_couplings() {
                if selected_set.contains(gspec.part_a.as_str())
                    && selected_set.contains(gspec.part_b.as_str())
                {
                    applied_gear_couplings.push(gspec);
                }
            }
        }
    }

    let ramp_duration_s = (options.dt_initial_s * 2.0)
        .min(0.15 * duration_s)
        .max(0.0);

    let mut applied_joint_couplings: Vec<AppliedJointCouplingSummary> = Vec::new();
    let mut fully_fixed_body: Vec<bool> = Vec::with_capacity(selected_ids.len());
    let mut bodies = Vec::with_capacity(selected_ids.len());
    for pid in &selected_ids {
        let part = parts
            .get(pid)
            .ok_or_else(|| ServerError::PartNotFound(pid.clone()))?;
        if part.mesh.vertices.is_empty() {
            return Err(ServerError::InvalidArgument(format!(
                "Part '{pid}' has an empty mesh; add geometry before baking coupled simulation"
            )));
        }

        // Parts participating in a Gear joint require boundary-snapped teeth and sufficient
        // resolution so coarse tetrahedra cannot bridge tooth valleys or interpenetrate at t = 0.
        let is_gear_part = applied_gear_couplings
            .iter()
            .any(|g| g.part_a == *pid || g.part_b == *pid);
        let part_acc_mode = if is_gear_part && matches!(acc_mode, AccuracyMode::Fast) {
            AccuracyMode::Standard
        } else {
            acc_mode
        };
        let part_mesh_density = if is_gear_part
            && args.get("mesh_density").is_none()
            && matches!(mesh_density, MeshDensity::Coarse)
        {
            let dims = part.bounding_box().dimensions();
            let max_dim = dims[0].max(dims[1]).max(dims[2]);
            let min_dim = dims[0].min(dims[1]).min(dims[2]);
            let h_gear = (max_dim / 14.0).min(min_dim / 3.0).max(1e-4);
            MeshDensity::Custom {
                max_element_size: h_gear,
            }
        } else {
            mesh_density
        };
        let tet_mesh = TetrahedralMesh::from_mesh_with_accuracy(
            &part.mesh,
            part_mesh_density,
            part_acc_mode,
        )
        .map_err(ServerError::Fea)?;

        let j2_mat =
            J2MaterialParams::from_material(&part.material).map_err(ServerError::Fea)?;

        let mut bc = BoundaryConditions::new();
        let part_dims = part.bounding_box().dimensions();
        let min_dim = part_dims[0].min(part_dims[1]).min(part_dims[2]).max(1e-4);
        let default_aabb_tol = (0.04 * min_dim).clamp(2.5e-4, 2.0e-3);
        let has_explicit_fixed = fixed_by_part.get(pid).is_some_and(|v| !v.is_empty());
        if let Some(faces) = fixed_by_part.get(pid) {
            for (f_str, tol_opt) in faces {
                let face = parse_aabb_face(f_str).ok_or_else(|| {
                    ServerError::InvalidArgument(format!(
                        "Invalid AABB face '{f_str}' in 'fixed_faces': expected min_x, max_x, min_y, max_y, min_z, or max_z"
                    ))
                })?;
                bc.dirichlet.push(DirichletConstraint::FixedAabbFace {
                    face,
                    tolerance: tol_opt.unwrap_or(default_aabb_tol),
                    components: DofComponent::All,
                });
            }
        }
        if let Some(part_loads) = loads_by_part.get(pid) {
            for (f_str, force, tol_opt) in part_loads {
                let face = parse_aabb_face(f_str).ok_or_else(|| {
                    ServerError::InvalidArgument(format!(
                        "Invalid AABB face '{f_str}' in 'loads': expected min_x, max_x, min_y, max_y, min_z, or max_z"
                    ))
                })?;
                bc.neumann.push(NeumannLoad::AabbFaceForce {
                    face,
                    tolerance: tol_opt.unwrap_or(default_aabb_tol),
                    total_force: *force,
                });
            }
        }

        // Map extracted PhysicsWorld joints/motors for this part into DynamicHubCoupling
        let mut hub_couplings: Vec<DynamicHubCoupling> = Vec::new();
        if let Some(part_joints) = joints_by_part.get(pid) {
            for jspec in part_joints {
                let (hub, summary) =
                    map_physics_joint_to_dynamic_hub(jspec, &tet_mesh, ramp_duration_s);
                hub_couplings.push(hub);
                applied_joint_couplings.push(summary);
            }
        }

        // If the part is marked `is_fixed` in the CAD scene and has no moving joint or manual BC,
        // anchor all its nodes via a Fixed hub so static frames remain stationary during baking.
        let is_completely_fixed =
            use_physics_joints && part.is_fixed && !has_explicit_fixed && hub_couplings.is_empty();
        if is_completely_fixed {
            hub_couplings.push(DynamicHubCoupling::Fixed {
                name: Some(format!("{pid}_fixed_frame")),
                selector: HubNodeSelector::Nodes((0..tet_mesh.node_count()).collect()),
            });
        }
        fully_fixed_body.push(is_completely_fixed);

        let mut spec = DynamicBodySpec::new(
            pid.clone(),
            tet_mesh,
            j2_mat,
            part.material.density_kg_m3,
        )
        .with_boundary(bc)
        .with_gravity(gravity)
        .with_force_curve(force_curve)
        .with_hub_couplings(hub_couplings);

        if options.enable_fracture {
            let dmg =
                DamageMaterialParams::from_material(&part.material).map_err(ServerError::Fea)?;
            spec = spec.with_damage(dmg);
        }

        if let Some(&v0) = init_vel_by_part.get(pid) {
            spec = spec.with_uniform_initial_velocity(v0);
        }

        bodies.push(spec);
    }

    // Build all pairwise deformable-to-deformable contact pairs when >1 body and contact enabled
    let mut contact_pairs = Vec::new();
    if enable_contact && bodies.len() > 1 {
        let friction = args
            .get("contact_friction")
            .and_then(|v| v.as_f64())
            .filter(|v| v.is_finite() && *v >= 0.0)
            .unwrap_or(0.2);
        for i in 0..bodies.len() {
            for j in (i + 1)..bodies.len() {
                if fully_fixed_body[i] && fully_fixed_body[j] {
                    continue;
                }
                let pid_a = &selected_ids[i];
                let pid_b = &selected_ids[j];
                let is_gear_pair = applied_gear_couplings.iter().any(|g| {
                    (g.part_a == *pid_a && g.part_b == *pid_b)
                        || (g.part_a == *pid_b && g.part_b == *pid_a)
                });
                let gear_search_dist = if is_gear_pair {
                    let avg_edge = |m: &TetrahedralMesh| -> f64 {
                        let mut sum = 0.0_f64;
                        let mut cnt = 0usize;
                        for elem in m.elements.iter().take(64) {
                            let p0 = m.nodes[elem.node_indices[0]].position;
                            let p1 = m.nodes[elem.node_indices[1]].position;
                            let dx = p1[0] - p0[0];
                            let dy = p1[1] - p0[1];
                            let dz = p1[2] - p0[2];
                            sum += (dx * dx + dy * dy + dz * dz).sqrt();
                            cnt += 1;
                        }
                        if cnt > 0 {
                            sum / cnt as f64
                        } else {
                            0.004
                        }
                    };
                    let h_min = avg_edge(&bodies[i].mesh).min(avg_edge(&bodies[j].mesh));
                    Some(h_min.clamp(1e-3, 1e-2))
                } else {
                    None
                };
                contact_pairs.push(ContactPairSpec::DeformablePair {
                    body_a: i,
                    body_b: j,
                    friction_coeff: friction,
                    symmetric: true,
                    slave_a_nodes: None,
                    slave_b_nodes: None,
                    search_distance: gear_search_dist,
                    max_normal_dot: if is_gear_pair { Some(0.25) } else { None },
                });
            }
        }
    }

    let job_id = format!("bake_{}", &Uuid::new_v4().to_string()[..8]);
    let simulation_id = format!("sim_{}", &Uuid::new_v4().to_string()[..8]);

    Ok(PreparedCoupledBake {
        job_id,
        simulation_id,
        accuracy_level,
        bodies,
        contact_pairs,
        options,
        gravity,
        async_mode,
        async_mode_explicit,
        wait_timeout_ms,
        applied_joint_couplings,
        applied_gear_couplings,
    })
}

/// Orients a tetrahedron boundary face `[a, b, c]` strictly outward relative to the opposite
/// 4th corner `opp` using reference mesh coordinates. Returns `(oriented_tri, was_flipped)`.
#[inline]
fn orient_tet_face_outward(
    mesh: &TetrahedralMesh,
    tri: [usize; 3],
    opp: usize,
) -> ([usize; 3], bool) {
    let pa = mesh.nodes[tri[0]].position;
    let pb = mesh.nodes[tri[1]].position;
    let pc = mesh.nodes[tri[2]].position;
    let po = mesh.nodes[opp].position;
    let ab = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
    let ac = [pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]];
    let ao = [po[0] - pa[0], po[1] - pa[1], po[2] - pa[2]];
    let cx = ab[1] * ac[2] - ab[2] * ac[1];
    let cy = ab[2] * ac[0] - ab[0] * ac[2];
    let cz = ab[0] * ac[1] - ab[1] * ac[0];
    if cx * ao[0] + cy * ao[1] + cz * ao[2] > 0.0 {
        ([tri[0], tri[2], tri[1]], true)
    } else {
        (tri, false)
    }
}

/// Extracts the compacted, strictly outward-oriented exterior surface triangle mesh for the
/// surviving active elements (`active_mask[e] == true`) of a `TET4` or `TET10` mesh (`P5-T15`).
///
/// For quadratic `TET10` meshes, each active exterior 6-node face is subdivided into 4 conforming
/// outward-oriented sub-triangles so curved boundary geometry and mid-edge nodes are preserved.
pub fn extract_active_surface_topology(
    mesh: &TetrahedralMesh,
    active_mask: &[bool],
) -> ActiveSurfaceTopology {
    let mut ext_tris: Vec<[usize; 3]> = Vec::new();

    if mesh.is_tet10() {
        type Tet10FaceEntry = (usize, [usize; 3], [usize; 3], usize);
        let mut face_counts: HashMap<[usize; 3], Tet10FaceEntry> =
            HashMap::with_capacity(mesh.tet10_elements.len() * 2);

        for (e_idx, ten) in mesh.tet10_elements.iter().enumerate() {
            if !active_mask.get(e_idx).copied().unwrap_or(false) {
                continue;
            }
            let [n0, n1, n2, n3, m01, m12, m20, m03, m13, m23] = *ten;
            let faces = [
                ([n0, n1, n2], [m01, m12, m20], n3),
                ([n0, n2, n3], [m20, m23, m03], n1),
                ([n0, n3, n1], [m03, m13, m01], n2),
                ([n1, n3, n2], [m13, m23, m12], n0),
            ];
            for (corners, mids, opp) in faces {
                let mut key = corners;
                key.sort_unstable();
                face_counts
                    .entry(key)
                    .and_modify(|e| e.0 += 1)
                    .or_insert((1, corners, mids, opp));
            }
        }

        for (cnt, corners, mids, opp) in face_counts.into_values() {
            if cnt != 1 {
                continue;
            }
            let ([c0, c1, c2], flipped) = orient_tet_face_outward(mesh, corners, opp);
            let [m01, m12, m20] = if flipped {
                [mids[2], mids[1], mids[0]]
            } else {
                mids
            };
            ext_tris.push([c0, m01, m20]);
            ext_tris.push([m01, c1, m12]);
            ext_tris.push([m20, m12, c2]);
            ext_tris.push([m01, m12, m20]);
        }
    } else {
        let mut face_counts: HashMap<[usize; 3], (usize, [usize; 3], usize)> =
            HashMap::with_capacity(mesh.elements.len() * 2);

        for (e_idx, elem) in mesh.elements.iter().enumerate() {
            if !active_mask.get(e_idx).copied().unwrap_or(false) {
                continue;
            }
            let [n0, n1, n2, n3] = elem.node_indices;
            let faces = [
                ([n0, n1, n2], n3),
                ([n0, n2, n3], n1),
                ([n0, n3, n1], n2),
                ([n1, n3, n2], n0),
            ];
            for (corners, opp) in faces {
                let mut key = corners;
                key.sort_unstable();
                face_counts
                    .entry(key)
                    .and_modify(|e| e.0 += 1)
                    .or_insert((1, corners, opp));
            }
        }

        for (cnt, corners, opp) in face_counts.into_values() {
            if cnt != 1 {
                continue;
            }
            let (oriented, _) = orient_tet_face_outward(mesh, corners, opp);
            ext_tris.push(oriented);
        }
    }

    if ext_tris.is_empty() {
        return ActiveSurfaceTopology::default();
    }

    ext_tris.sort_unstable();

    // Compact unique surface nodes in deterministic ascending order
    let mut unique_nodes: Vec<usize> = Vec::with_capacity(ext_tris.len());
    for tri in &ext_tris {
        unique_nodes.push(tri[0]);
        unique_nodes.push(tri[1]);
        unique_nodes.push(tri[2]);
    }
    unique_nodes.sort_unstable();
    unique_nodes.dedup();

    let mut global_to_local: HashMap<usize, u32> = HashMap::with_capacity(unique_nodes.len());
    let mut surface_node_indices: Vec<u32> = Vec::with_capacity(unique_nodes.len());
    for (local_idx, &gid) in unique_nodes.iter().enumerate() {
        global_to_local.insert(gid, local_idx as u32);
        surface_node_indices.push(gid as u32);
    }

    let mut surface_indices: Vec<u32> = Vec::with_capacity(ext_tris.len() * 3);
    for tri in &ext_tris {
        surface_indices.push(global_to_local[&tri[0]]);
        surface_indices.push(global_to_local[&tri[1]]);
        surface_indices.push(global_to_local[&tri[2]]);
    }

    ActiveSurfaceTopology {
        surface_node_indices,
        surface_indices,
    }
}

/// Stateful incremental frame builder converting each [`DynamicFrameSnapshot`] into a
/// compact [`BakedFrameCache`] on-the-fly so partial simulations can be inspected and
/// streamed frame-by-frame while the HHT-α solver is still running.
struct IncrementalBakeBuilder {
    body_index_by_id: HashMap<String, usize>,
    body_meshes: Vec<TetrahedralMesh>,
    prev_active_masks: Vec<Option<Vec<bool>>>,
    prev_topo_indices: Vec<usize>,
    frag_world: PhysicsWorld,
    spawned_frag_ids: HashSet<String>,
    active_frag_ids: Vec<String>,
    frag_meta: HashMap<String, (String, usize, usize, f64)>,
    prev_time_s: f64,
}

impl IncrementalBakeBuilder {
    fn new(prepared: &PreparedCoupledBake) -> (Self, BakedSimulationRecord) {
        let topologies: Vec<BakedPartTopology> = prepared
            .bodies
            .iter()
            .map(|b| BakedPartTopology {
                part_id: b.part_id.clone(),
                node_count: b.mesh.node_count(),
                element_count: b.mesh.element_count(),
                element_order: if b.mesh.is_tet10() {
                    "TET10".to_string()
                } else {
                    "TET4".to_string()
                },
                reference_positions_m: b.mesh.nodes.iter().map(|n| n.position).collect(),
            })
            .collect();

        let body_index_by_id: HashMap<String, usize> = prepared
            .bodies
            .iter()
            .enumerate()
            .map(|(i, b)| (b.part_id.clone(), i))
            .collect();
        let body_meshes: Vec<TetrahedralMesh> =
            prepared.bodies.iter().map(|b| b.mesh.clone()).collect();
        let n_bodies = prepared.bodies.len();

        let record = BakedSimulationRecord {
            simulation_id: prepared.simulation_id.clone(),
            job_id: prepared.job_id.clone(),
            accuracy_level: prepared.accuracy_level,
            part_ids: prepared.bodies.iter().map(|b| b.part_id.clone()).collect(),
            duration_s: prepared.options.total_time_s,
            topologies,
            surface_topologies: Vec::new(),
            fragment_meshes: Vec::new(),
            frames: Vec::new(),
            events: Vec::new(),
            total_steps: 0,
            total_newton_iterations: 0,
            total_bisections: 0,
            min_dt_used_s: prepared.options.dt_initial_s,
            max_dt_used_s: prepared.options.dt_initial_s,
            cfl_critical_dt_s: 1e-6,
            cfl_ratio_max: 0.0,
            initial_total_energy_j: 0.0,
            final_total_energy_j: 0.0,
            max_energy_drift_pct: 0.0,
            cancelled: false,
            elapsed_ms: 0.0,
            applied_joint_couplings: prepared.applied_joint_couplings.clone(),
            applied_gear_couplings: prepared.applied_gear_couplings.clone(),
        };

        let builder = Self {
            body_index_by_id,
            body_meshes,
            prev_active_masks: vec![None; n_bodies],
            prev_topo_indices: vec![0; n_bodies],
            frag_world: PhysicsWorld::with_gravity(prepared.gravity),
            spawned_frag_ids: HashSet::new(),
            active_frag_ids: Vec::new(),
            frag_meta: HashMap::new(),
            prev_time_s: 0.0,
        };

        (builder, record)
    }

    fn push_frame(&mut self, snap: DynamicFrameSnapshot, record: &mut BakedSimulationRecord) -> usize {
        let dt_frame = (snap.time_s - self.prev_time_s).max(0.0);

        for frag in &snap.detached_fragments {
            if self.spawned_frag_ids.insert(frag.fragment_id.clone()) {
                let baked_mesh = BakedFragmentMesh::from_detached_fragment(frag);
                let bounding_radius_m = baked_mesh.bounding_radius_m;
                record.fragment_meshes.push(baked_mesh);

                let spec = FragmentSpawnSpec {
                    fragment_id: frag.fragment_id.clone(),
                    mesh_local: frag.local_surface_mesh.clone(),
                    density_kg_m3: frag.density_kg_m3,
                    mass_kg: frag.kinematics.mass_kg.max(1e-9),
                    local_com: [0.0, 0.0, 0.0],
                    inertia_tensor_com: frag.kinematics.inertia_tensor_com,
                    position: frag.kinematics.center_of_mass,
                    orientation_quat: [0.0, 0.0, 0.0, 1.0],
                    linear_velocity: frag.kinematics.linear_velocity,
                    angular_velocity: frag.kinematics.angular_velocity,
                    collider_type: ColliderType::ConvexHull,
                    friction: 0.3,
                    restitution: 0.3,
                };
                if self.frag_world.spawn_detached_fragment(&spec).is_ok() {
                    self.active_frag_ids.push(frag.fragment_id.clone());
                    self.frag_meta.insert(
                        frag.fragment_id.clone(),
                        (
                            frag.source_part_id.clone(),
                            frag.detachment_step,
                            frag.element_indices.len(),
                            bounding_radius_m,
                        ),
                    );
                }
            }
        }

        let mut frame_surf_topo_indices: Vec<usize> = Vec::with_capacity(snap.bodies.len());
        for b_frame in &snap.bodies {
            if let Some(&b_idx) = self.body_index_by_id.get(b_frame.part_id.as_str()) {
                let tet_mesh = &self.body_meshes[b_idx];
                let n_elems = tet_mesh.element_count();
                let mut active_mask = vec![true; n_elems];
                for &e in &b_frame.eroded_element_indices {
                    if e < n_elems {
                        active_mask[e] = false;
                    }
                }
                for frag in &snap.detached_fragments {
                    if frag.source_part_id == b_frame.part_id {
                        for &e in &frag.element_indices {
                            if e < n_elems {
                                active_mask[e] = false;
                            }
                        }
                    }
                }
                let topo_idx = if self.prev_active_masks[b_idx].as_deref()
                    == Some(active_mask.as_slice())
                {
                    self.prev_topo_indices[b_idx]
                } else {
                    let surf_topo = extract_active_surface_topology(tet_mesh, &active_mask);
                    let idx = record.surface_topologies.len();
                    record.surface_topologies.push(surf_topo);
                    self.prev_active_masks[b_idx] = Some(active_mask);
                    self.prev_topo_indices[b_idx] = idx;
                    idx
                };
                frame_surf_topo_indices.push(topo_idx);
            } else {
                let idx = record.surface_topologies.len();
                record.surface_topologies.push(ActiveSurfaceTopology::default());
                frame_surf_topo_indices.push(idx);
            }
        }

        let baked_fragments: Vec<BakedFragmentFrame> = if !self.active_frag_ids.is_empty() {
            let (poses, _cols) = self.frag_world.step_coupled_fragments_for_dt(
                dt_frame,
                0.002,
                &self.active_frag_ids,
                snap.time_s,
            );
            poses
                .into_iter()
                .map(|p| {
                    let (source_part_id, detachment_step, element_count, bounding_radius_m) =
                        self.frag_meta
                            .get(&p.fragment_id)
                            .cloned()
                            .unwrap_or_else(|| ("unknown".to_string(), 0, 0, 0.0));
                    BakedFragmentFrame {
                        fragment_id: p.fragment_id,
                        source_part_id,
                        detachment_step,
                        mass_kg: p.mass_kg,
                        position_m: p.position,
                        orientation_quat: p.orientation_quat,
                        linear_velocity_m_s: p.linear_velocity,
                        angular_velocity_rad_s: p.angular_velocity,
                        kinetic_energy_j: p.kinetic_energy_j,
                        element_count,
                        bounding_radius_m,
                    }
                })
                .collect()
        } else {
            Vec::new()
        };

        self.prev_time_s = snap.time_s;
        let cache = convert_snapshot_to_cache(snap, baked_fragments, &frame_surf_topo_indices);
        let idx = cache.frame_index;
        record.frames.push(cache);
        idx
    }

    fn apply_report_metadata(report: DynamicHhtReport, record: &mut BakedSimulationRecord) {
        record.events = report.events;
        record.total_steps = report.total_steps;
        record.total_newton_iterations = report.total_newton_iterations;
        record.total_bisections = report.total_bisections;
        record.min_dt_used_s = report.min_dt_used_s;
        record.max_dt_used_s = report.max_dt_used_s;
        record.cfl_critical_dt_s = report.cfl_critical_dt_s;
        record.cfl_ratio_max = report.cfl_ratio_max;
        record.initial_total_energy_j = report.initial_total_energy_j;
        record.final_total_energy_j = report.final_total_energy_j;
        record.max_energy_drift_pct = report.max_energy_drift_pct;
        record.cancelled = report.cancelled;
        record.elapsed_ms = report.elapsed_ms;
    }
}

/// Converts a raw [`DynamicHhtReport`] into a compact [`BakedSimulationRecord`], synchronizing any
/// detached fractured fragments through `mcp_cad_physics::PhysicsWorld::step_coupled_fragments_for_dt`
/// across the adaptive timeline frames and extracting deduplicated active exterior surface topologies
/// plus local-space fragment triangle meshes (`P5-T6`, `P5-T15`).
pub fn build_baked_simulation_record(
    prepared: &PreparedCoupledBake,
    mut report: DynamicHhtReport,
) -> BakedSimulationRecord {
    let (mut builder, mut record) = IncrementalBakeBuilder::new(prepared);
    let raw_frames = std::mem::take(&mut report.frames);
    record.frames.reserve(raw_frames.len());
    for snap in raw_frames {
        builder.push_frame(snap, &mut record);
    }
    IncrementalBakeBuilder::apply_report_metadata(report, &mut record);
    record
}

/// Converts a single `DynamicFrameSnapshot` into the compact `BakedFrameCache` with MPa/mm summary fields.
fn convert_snapshot_to_cache(
    snap: DynamicFrameSnapshot,
    detached_fragments: Vec<BakedFragmentFrame>,
    surface_topo_indices: &[usize],
) -> BakedFrameCache {
    let parts = snap
        .bodies
        .into_iter()
        .enumerate()
        .map(|(idx, b)| BakedPartFrame {
            part_id: b.part_id,
            node_count: b.node_count,
            deformed_positions_m: b.deformed_positions,
            displacements_m: b.displacements,
            nodal_von_mises_mpa: b.nodal_von_mises_pa.into_iter().map(|s| s * 1e-6).collect(),
            nodal_equiv_plastic_strain: b.nodal_equiv_plastic_strain,
            nodal_damage: b.nodal_damage,
            eroded_element_indices: b.eroded_element_indices,
            active_element_count: b.active_element_count,
            max_displacement_mm: b.max_displacement_m * 1e3,
            max_von_mises_mpa: b.max_von_mises_pa * 1e-6,
            max_equiv_plastic_strain: b.max_equiv_plastic_strain,
            max_damage: b.max_damage,
            surface_topology_index: surface_topo_indices.get(idx).copied().unwrap_or(0),
        })
        .collect();

    BakedFrameCache {
        frame_index: snap.frame_index,
        time_s: snap.time_s,
        dt_used_s: snap.dt_used_s,
        newton_iterations: snap.newton_iterations,
        parts,
        contact_forces: snap.contact_forces,
        detached_fragments,
        energy: snap.energy,
        hub_reactions: snap.hub_reactions,
    }
}

fn build_live_progress_snapshot(status: &CoupledJobStatus) -> LiveCoupledSimProgress {
    let state_str = match status.state {
        BakeJobState::Running => "running",
        BakeJobState::Cancelling => "cancelling",
        BakeJobState::Completed => "completed",
        BakeJobState::Cancelled => "cancelled",
        BakeJobState::Failed => "failed",
    };
    LiveCoupledSimProgress {
        job_id: status.job_id.clone(),
        simulation_id: status.simulation_id.clone(),
        state: state_str.to_string(),
        accuracy_level: status.accuracy_level.as_str().to_string(),
        part_ids: status.part_ids.clone(),
        total_time_s: status.total_time_s,
        current_time_s: status.current_time_s,
        progress_pct: status.progress_pct,
        step_count: status.step_count,
        frame_count: status.frame_count,
        frames_cached: status.frames_cached,
        latest_frame_index: status.latest_frame_index,
        current_dt_s: status.current_dt_s,
        bisections: status.bisections,
        eroded_elements: status.eroded_elements,
        detached_fragments: status.detached_fragments,
        max_von_mises_mpa: status.max_von_mises_mpa,
        elapsed_ms: status.elapsed_ms.unwrap_or(0.0),
        estimated_remaining_ms: status.estimated_remaining_ms,
        stage_label: status.stage_label.clone(),
        updated_at_ms: now_unix_ms(),
    }
}

/// Executes a prepared coupled bake synchronously on the calling thread, caching every
/// calculated frame immediately for live queries and streaming real-time SSE + `.mcp_live`
/// progress events while honoring cancellation via `cancel_flag`.
pub fn execute_prepared_bake_blocking(
    prepared: PreparedCoupledBake,
    manager: SharedCoupledSimManager,
    events_tx: broadcast::Sender<CoupledSimSseEvent>,
    cancel_flag: Arc<AtomicBool>,
) -> Result<Arc<BakedSimulationRecord>, ServerError> {
    let job_id = prepared.job_id.clone();
    let sim_id = prepared.simulation_id.clone();
    let part_ids: Vec<String> = prepared.bodies.iter().map(|b| b.part_id.clone()).collect();

    let (builder_init, initial_record) = IncrementalBakeBuilder::new(&prepared);
    let builder_shared = Arc::new(Mutex::new(builder_init));
    let live_record_handle = {
        let mut m = manager.write();
        m.begin_live_build(sim_id.clone(), initial_record)
    };

    let _ = events_tx.send(CoupledSimSseEvent::Started {
        job_id: job_id.clone(),
        simulation_id: sim_id.clone(),
        accuracy_level: prepared.accuracy_level.as_str().to_string(),
        part_ids,
        total_time_s: prepared.options.total_time_s,
    });

    let last_live_emit = Arc::new(Mutex::new(Instant::now() - Duration::from_secs(5)));
    let last_log_emit = Arc::new(Mutex::new(Instant::now() - Duration::from_secs(5)));

    let mgr_frame = Arc::clone(&manager);
    let job_frame = job_id.clone();
    let live_rec_frame = Arc::clone(&live_record_handle);
    let builder_frame = Arc::clone(&builder_shared);
    let last_emit_frame = Arc::clone(&last_live_emit);

    let frame_cb = move |snap: &DynamicFrameSnapshot| {
        let (frame_idx, frames_cached) = {
            let mut b = builder_frame.lock().expect("incremental builder lock");
            let mut rec = live_rec_frame.write();
            let idx = b.push_frame(snap.clone(), &mut rec);
            (idx, rec.frames.len())
        };
        let status_opt = {
            let mut m = mgr_frame.write();
            m.update_live_frame_count(&job_frame, frames_cached, frame_idx);
            m.job_status(Some(&job_frame))
        };
        if let Some(status) = status_opt {
            let should_emit = frame_idx == 0 || {
                let mut t = last_emit_frame.lock().expect("live emit lock");
                if t.elapsed() >= Duration::from_millis(250) {
                    *t = Instant::now();
                    true
                } else {
                    false
                }
            };
            if should_emit {
                let live_prog = build_live_progress_snapshot(&status);
                let rec_guard = live_rec_frame.read();
                publish_live_coupled_sim_progress(&live_prog, Some(&rec_guard));
            }
        }
    };

    let mgr_cb = Arc::clone(&manager);
    let tx_cb = events_tx.clone();
    let job_cb = job_id.clone();
    let sim_cb = sim_id.clone();
    let last_emit_prog = Arc::clone(&last_live_emit);
    let last_log_prog = Arc::clone(&last_log_emit);

    let progress_cb = move |upd: DynamicProgressUpdate| {
        let (elapsed_ms, eta_ms, latest_frame_idx, status_opt) = {
            let mut m = mgr_cb.write();
            let (el, eta, lfi) = m.update_progress(&job_cb, &upd);
            (el, eta, lfi, m.job_status(Some(&job_cb)))
        };
        let _ = tx_cb.send(CoupledSimSseEvent::Progress {
            job_id: job_cb.clone(),
            simulation_id: sim_cb.clone(),
            time_s: upd.time_s,
            total_time_s: upd.total_time_s,
            progress_pct: upd.progress_pct,
            step_count: upd.step_count,
            frame_count: upd.frame_count,
            latest_frame_index: latest_frame_idx,
            current_dt_s: upd.current_dt_s,
            bisections: upd.bisections,
            eroded_elements: upd.eroded_elements,
            detached_fragments: upd.detached_fragments,
            max_von_mises_mpa: upd.max_von_mises_pa * 1e-6,
            elapsed_ms,
            estimated_remaining_ms: eta_ms,
        });

        if let Some(status) = status_opt {
            let should_write_progress = {
                let mut t = last_emit_prog.lock().expect("live emit lock");
                if t.elapsed() >= Duration::from_millis(350) {
                    *t = Instant::now();
                    true
                } else {
                    false
                }
            };
            if should_write_progress {
                let live_prog = build_live_progress_snapshot(&status);
                publish_live_coupled_sim_progress(&live_prog, None);
            }

            let should_log = {
                let mut t = last_log_prog.lock().expect("live log lock");
                if t.elapsed() >= Duration::from_secs(1) {
                    *t = Instant::now();
                    true
                } else {
                    false
                }
            };
            if should_log {
                tracing::info!(
                    job_id = %job_cb,
                    simulation_id = %sim_cb,
                    progress_pct = upd.progress_pct,
                    step = upd.step_count,
                    frames_cached = status.frames_cached,
                    elapsed_ms,
                    eta_ms = ?eta_ms,
                    max_vm_mpa = upd.max_von_mises_pa * 1e-6,
                    "bake_coupled_simulation live progress"
                );
            }
        }
    };

    let solve_res = solve_dynamic_hht_coupled_with_frames(
        prepared.bodies.clone(),
        &prepared.contact_pairs,
        prepared.options,
        Some(progress_cb),
        Some(frame_cb),
        Some(cancel_flag.as_ref()),
    );

    match solve_res {
        Ok(report) => {
            let cancelled = report.cancelled;
            let total_steps = report.total_steps;
            let total_bisections = report.total_bisections;
            let cfl_ratio_max = report.cfl_ratio_max;
            let max_energy_drift_pct = report.max_energy_drift_pct;
            let elapsed_ms = report.elapsed_ms;

            let mut final_record = {
                let guard = live_record_handle.read();
                guard.clone()
            };
            // Ensure all frames exist if a solver skipped callback or differed in count
            if final_record.frames.len() < report.frames.len() {
                let mut builder = builder_shared.lock().expect("incremental builder lock");
                for snap in report.frames.iter().skip(final_record.frames.len()) {
                    builder.push_frame(snap.clone(), &mut final_record);
                }
            }
            let frame_count = final_record.frames.len();
            IncrementalBakeBuilder::apply_report_metadata(report, &mut final_record);

            let (stored, terminal_status) = {
                let mut m = manager.write();
                m.finish_job_with_record(&job_id, final_record);
                let rec = m
                    .get_simulation(Some(&sim_id))
                    .expect("just stored simulation record");
                let st = m.job_status(Some(&job_id));
                (rec, st)
            };

            if let Some(status) = terminal_status {
                let live_prog = build_live_progress_snapshot(&status);
                publish_live_coupled_sim_progress(&live_prog, Some(&stored));
            }

            if cancelled {
                let _ = events_tx.send(CoupledSimSseEvent::Cancelled {
                    job_id,
                    simulation_id: sim_id,
                    frame_count,
                    elapsed_ms,
                });
            } else {
                let _ = events_tx.send(CoupledSimSseEvent::Completed {
                    job_id,
                    simulation_id: sim_id,
                    frame_count,
                    total_steps,
                    total_bisections,
                    cfl_ratio_max,
                    max_energy_drift_pct,
                    elapsed_ms,
                });
            }

            Ok(stored)
        }
        Err(err) => {
            let err_msg = err.to_string();
            let terminal_status = {
                let mut m = manager.write();
                m.fail_job(&job_id, err_msg.clone());
                m.job_status(Some(&job_id))
            };
            if let Some(status) = terminal_status {
                let live_prog = build_live_progress_snapshot(&status);
                publish_live_coupled_sim_progress(&live_prog, None);
            }
            let _ = events_tx.send(CoupledSimSseEvent::Failed {
                job_id,
                simulation_id: sim_id,
                error: err_msg,
            });
            Err(ServerError::Fea(err))
        }
    }
}

fn format_running_job_ack_json(
    job: &CoupledJobStatus,
    applied_joint_couplings_count: usize,
    applied_gear_couplings_count: usize,
    unit: LengthUnit,
) -> Value {
    json!({
        "status": "running",
        "job_id": job.job_id,
        "simulation_id": job.simulation_id,
        "accuracy_level": job.accuracy_level.as_str(),
        "part_ids": job.part_ids,
        "duration_s": job.total_time_s,
        "progress_pct": job.progress_pct,
        "current_time_s": job.current_time_s,
        "step_count": job.step_count,
        "frame_count": job.frame_count,
        "frames_cached": job.frames_cached,
        "latest_frame_index": job.latest_frame_index,
        "current_dt_s": job.current_dt_s,
        "bisections": job.bisections,
        "eroded_elements": job.eroded_elements,
        "detached_fragments": job.detached_fragments,
        "max_von_mises_mpa": job.max_von_mises_mpa,
        "elapsed_ms": job.elapsed_ms,
        "estimated_remaining_ms": job.estimated_remaining_ms,
        "stage_label": job.stage_label,
        "applied_joint_couplings_count": applied_joint_couplings_count,
        "applied_gear_couplings_count": applied_gear_couplings_count,
        "sse_endpoint": "/api/coupled-sim/events",
        "message": "Coupled HHT-alpha bake is running in the background and streaming solved frames live.",
        "next_step": format!(
            "Call 'bake_coupled_simulation' with {{\"action\": \"status\", \"job_id\": \"{}\", \"wait_timeout_ms\": 5000}} to wait for the next frame/completion, or call 'query_simulation_frame' with {{\"simulation_id\": \"{}\"}} to inspect already-cached frames right away.",
            job.job_id, job.simulation_id
        ),
        "units": unit.units_metadata()
    })
}

/// Launches a prepared bake either asynchronously, synchronously (`async_mode: false`), or with
/// a bounded wait window (`async_mode` omitted -> yields `"running"` after `wait_timeout_ms`, default 12s)
/// so short solves return inline while heavy solves never hang an MCP agent.
pub fn launch_prepared_bake(
    prepared: PreparedCoupledBake,
    manager: SharedCoupledSimManager,
    events_tx: broadcast::Sender<CoupledSimSseEvent>,
    unit: LengthUnit,
) -> Result<Value, ServerError> {
    let job_id = prepared.job_id.clone();
    let sim_id = prepared.simulation_id.clone();
    let acc = prepared.accuracy_level;
    let part_ids: Vec<String> = prepared.bodies.iter().map(|b| b.part_id.clone()).collect();
    let duration_s = prepared.options.total_time_s;
    let async_mode_explicit = prepared.async_mode_explicit;
    let wait_timeout_ms = prepared.wait_timeout_ms;

    let cancel_flag = {
        let mut m = manager.write();
        m.register_job(
            job_id.clone(),
            sim_id.clone(),
            acc,
            part_ids.clone(),
            duration_s,
        )
    };

    let applied_joint_couplings_count = prepared.applied_joint_couplings.len();
    let applied_gear_couplings_count = prepared.applied_gear_couplings.len();

    if async_mode_explicit == Some(false) {
        let record = execute_prepared_bake_blocking(prepared, manager, events_tx, cancel_flag)?;
        return Ok(format_baked_summary_json(&record, unit));
    }

    let (done_tx, done_rx) =
        std::sync::mpsc::sync_channel::<Result<Arc<BakedSimulationRecord>, ServerError>>(1);
    let mgr_bg = Arc::clone(&manager);
    let tx_bg = events_tx.clone();
    std::thread::Builder::new()
        .name(format!("hht-bake-{job_id}"))
        .spawn(move || {
            let res = execute_prepared_bake_blocking(prepared, mgr_bg, tx_bg, cancel_flag);
            let _ = done_tx.send(res);
        })
        .map_err(|e| {
            ServerError::Internal(format!("Failed to spawn HHT-α bake worker thread: {e}"))
        })?;

    if async_mode_explicit == Some(true) {
        let status = {
            let m = manager.read();
            m.job_status(Some(&job_id))
        };
        if let Some(job) = status {
            return Ok(format_running_job_ack_json(
                &job,
                applied_joint_couplings_count,
                applied_gear_couplings_count,
                unit,
            ));
        }
    }

    // Bounded interactive wait when `async_mode` is omitted: fast solves complete inline,
    // heavy solves cleanly yield `"status": "running"` with live progress/ETA instead of hanging.
    let timeout = Duration::from_millis(wait_timeout_ms.unwrap_or(12_000));
    match done_rx.recv_timeout(timeout) {
        Ok(Ok(record)) => Ok(format_baked_summary_json(&record, unit)),
        Ok(Err(err)) => Err(err),
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
            let job = {
                let m = manager.read();
                m.job_status(Some(&job_id)).ok_or_else(|| {
                    ServerError::Internal(format!("Lost tracked bake job '{job_id}'"))
                })?
            };
            Ok(format_running_job_ack_json(
                &job,
                applied_joint_couplings_count,
                applied_gear_couplings_count,
                unit,
            ))
        }
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(ServerError::Internal(
            "HHT-α bake worker thread exited unexpectedly".to_string(),
        )),
    }
}

/// Formats a compact JSON summary for a completed or cancelled [`BakedSimulationRecord`].
pub fn format_baked_summary_json(record: &BakedSimulationRecord, unit: LengthUnit) -> Value {
    let peak_stress_mpa = record
        .frames
        .iter()
        .flat_map(|f| f.parts.iter())
        .map(|p| p.max_von_mises_mpa)
        .fold(0.0_f64, f64::max);
    let peak_disp_mm = record
        .frames
        .iter()
        .flat_map(|f| f.parts.iter())
        .map(|p| p.max_displacement_mm)
        .fold(0.0_f64, f64::max);
    let peak_damage = record
        .frames
        .iter()
        .flat_map(|f| f.parts.iter())
        .map(|p| p.max_damage)
        .fold(0.0_f64, f64::max);
    let total_fragments = record
        .frames
        .last()
        .map_or(0, |f| f.detached_fragments.len());

    json!({
        "status": if record.cancelled { "cancelled" } else { "completed" },
        "job_id": record.job_id,
        "simulation_id": record.simulation_id,
        "accuracy_level": record.accuracy_level.as_str(),
        "part_ids": record.part_ids,
        "duration_s": record.duration_s,
        "frame_count": record.frames.len(),
        "total_steps": record.total_steps,
        "total_newton_iterations": record.total_newton_iterations,
        "total_bisections": record.total_bisections,
        "min_dt_used_s": record.min_dt_used_s,
        "max_dt_used_s": record.max_dt_used_s,
        "cfl_critical_dt_s": record.cfl_critical_dt_s,
        "cfl_ratio_max": record.cfl_ratio_max,
        "initial_total_energy_j": record.initial_total_energy_j,
        "final_total_energy_j": record.final_total_energy_j,
        "max_energy_drift_pct": record.max_energy_drift_pct,
        "peak_von_mises_mpa": peak_stress_mpa,
        "peak_displacement_mm": peak_disp_mm,
        "peak_damage": peak_damage,
        "detached_fragments_count": total_fragments,
        "fragment_meshes_count": record.fragment_meshes.len(),
        "fragment_meshes_endpoint": format!("/api/coupled-sim/fragments/{}", record.simulation_id),
        "events_count": record.events.len(),
        "events": record.events,
        "applied_joint_couplings_count": record.applied_joint_couplings.len(),
        "applied_joint_couplings": record.applied_joint_couplings,
        "applied_gear_couplings": record.applied_gear_couplings,
        "cancelled": record.cancelled,
        "elapsed_ms": record.elapsed_ms,
        "units": unit.units_metadata()
    })
}

/// Retrieves all cached local-space triangle meshes of detached fragments for an active or completed simulation
/// (transferred once per simulation by HTTP clients via `/api/coupled-sim/fragments/:sim_id`, `P5-T15`).
pub fn query_cached_fragment_meshes_from_manager(
    sim_id_opt: Option<&str>,
    manager: &SharedCoupledSimManager,
    unit: LengthUnit,
) -> Result<Value, ServerError> {
    let m = manager.read();
    m.with_simulation_record(sim_id_opt, |record| {
        json!({
            "simulation_id": record.simulation_id,
            "fragment_count": record.fragment_meshes.len(),
            "fragments": record.fragment_meshes,
            "units": unit.units_metadata()
        })
    })
    .ok_or_else(|| {
        if let Some(id) = sim_id_opt {
            ServerError::InvalidArgument(format!(
                "SIMULATION_NOT_FOUND: No cached coupled simulation with id '{id}'. Run 'bake_coupled_simulation' first."
            ))
        } else {
            ServerError::InvalidArgument(
                "SIMULATION_NOT_FOUND: No cached coupled simulation exists yet. Run 'bake_coupled_simulation' first."
                    .to_string(),
            )
        }
    })
}

/// Queries a single cached frame from [`CoupledSimulationManager`] (completed or live streaming) by `frame_index` or nearest `time_s`.
pub fn query_cached_frame_from_manager(
    args: &Value,
    manager: &SharedCoupledSimManager,
    unit: LengthUnit,
) -> Result<Value, ServerError> {
    let sim_id_opt = args.get("simulation_id").and_then(|v| v.as_str());
    let part_filter = args.get("part_id").and_then(|v| v.as_str());
    let include_mesh_fields = args
        .get("include_mesh_fields")
        .and_then(|v| v.as_bool())
        .unwrap_or(true);
    let include_fragment_meshes = args
        .get("include_fragment_meshes")
        .and_then(|v| v.as_bool())
        .unwrap_or(include_mesh_fields);

    let m = manager.read();
    let res_opt = m.with_simulation_record(sim_id_opt, |record| -> Result<Value, ServerError> {
        if record.frames.is_empty() {
            return Err(ServerError::InvalidArgument(format!(
                "Cached simulation '{}' contains 0 frames so far.",
                record.simulation_id
            )));
        }

        let frame_idx = if let Some(idx_u64) = args.get("frame_index").and_then(|v| v.as_u64()) {
            let idx = idx_u64 as usize;
            if idx >= record.frames.len() {
                return Err(ServerError::InvalidArgument(format!(
                    "FRAME_OUT_OF_RANGE: frame_index {idx} exceeds available frames [0..{}]",
                    record.frames.len() - 1
                )));
            }
            idx
        } else if let Some(t_req) = args.get("time_s").and_then(|v| v.as_f64()) {
            if !t_req.is_finite() || t_req < 0.0 {
                return Err(ServerError::InvalidArgument(format!(
                    "Invalid 'time_s' ({t_req}): must be a non-negative finite number in seconds"
                )));
            }
            record
                .frames
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    (a.time_s - t_req)
                        .abs()
                        .partial_cmp(&(b.time_s - t_req).abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map_or(0, |(idx, _)| idx)
        } else {
            record.frames.len() - 1
        };

        let frame = &record.frames[frame_idx];
        let scale_len = unit.from_meters_factor();

        let topo_by_part: HashMap<&str, &BakedPartTopology> = record
            .topologies
            .iter()
            .map(|t| (t.part_id.as_str(), t))
            .collect();
        let frag_mesh_by_id: HashMap<&str, &BakedFragmentMesh> = record
            .fragment_meshes
            .iter()
            .map(|fm| (fm.fragment_id.as_str(), fm))
            .collect();

        let parts_json: Vec<Value> = frame
            .parts
            .iter()
            .filter(|p| part_filter.is_none_or(|f| f == p.part_id))
            .map(|p| {
                let surf_topo = record.surface_topologies.get(p.surface_topology_index);
                let surface_vertex_count = surf_topo.map_or(0, |s| s.surface_node_indices.len());
                let surface_triangle_count = surf_topo.map_or(0, |s| s.surface_indices.len() / 3);
                let mut obj = json!({
                    "part_id": p.part_id,
                    "node_count": p.node_count,
                    "active_element_count": p.active_element_count,
                    "eroded_element_count": p.eroded_element_indices.len(),
                    "eroded_element_indices": p.eroded_element_indices,
                    "surface_vertex_count": surface_vertex_count,
                    "surface_triangle_count": surface_triangle_count,
                    "max_displacement_mm": p.max_displacement_mm,
                    "max_von_mises_mpa": p.max_von_mises_mpa,
                    "max_equiv_plastic_strain": p.max_equiv_plastic_strain,
                    "max_damage": p.max_damage,
                });
                if include_mesh_fields {
                    let deformed_coords: Vec<[f64; 3]> = p
                        .deformed_positions_m
                        .iter()
                        .map(|c| [c[0] * scale_len, c[1] * scale_len, c[2] * scale_len])
                        .collect();
                    let displacements: Vec<[f64; 3]> = p
                        .displacements_m
                        .iter()
                        .map(|d| [d[0] * 1e3, d[1] * 1e3, d[2] * 1e3])
                        .collect();
                    let ref_positions_opt = topo_by_part
                        .get(p.part_id.as_str())
                        .map(|t| t.reference_positions_m.as_slice());

                    let (surface_positions_m, surface_reference_positions_m, surface_indices, surface_node_indices) =
                        if let Some(surf) = surf_topo {
                            let s_pos: Vec<[f64; 3]> = surf
                                .surface_node_indices
                                .iter()
                                .map(|&nid| {
                                    p.deformed_positions_m
                                        .get(nid as usize)
                                        .copied()
                                        .unwrap_or([0.0; 3])
                                })
                                .collect();
                            let s_ref: Vec<[f64; 3]> = surf
                                .surface_node_indices
                                .iter()
                                .map(|&nid| {
                                    ref_positions_opt
                                        .and_then(|rp| rp.get(nid as usize).copied())
                                        .unwrap_or_else(|| {
                                            let def = p
                                                .deformed_positions_m
                                                .get(nid as usize)
                                                .copied()
                                                .unwrap_or([0.0; 3]);
                                            let disp = p
                                                .displacements_m
                                                .get(nid as usize)
                                                .copied()
                                                .unwrap_or([0.0; 3]);
                                            [def[0] - disp[0], def[1] - disp[1], def[2] - disp[2]]
                                        })
                                })
                                .collect();
                            (
                                s_pos,
                                s_ref,
                                surf.surface_indices.clone(),
                                surf.surface_node_indices.clone(),
                            )
                        } else {
                            (Vec::new(), Vec::new(), Vec::new(), Vec::new())
                        };

                    let map = obj.as_object_mut().expect("json object");
                    map.insert("deformed_positions".to_string(), json!(deformed_coords));
                    map.insert("displacements_mm".to_string(), json!(displacements));
                    map.insert(
                        "nodal_von_mises_mpa".to_string(),
                        json!(p.nodal_von_mises_mpa),
                    );
                    map.insert(
                        "nodal_equiv_plastic_strain".to_string(),
                        json!(p.nodal_equiv_plastic_strain),
                    );
                    map.insert("nodal_damage".to_string(), json!(p.nodal_damage));
                    map.insert("surface_positions_m".to_string(), json!(surface_positions_m));
                    map.insert(
                        "surface_reference_positions_m".to_string(),
                        json!(surface_reference_positions_m),
                    );
                    map.insert("surface_indices".to_string(), json!(surface_indices));
                    map.insert(
                        "surface_node_indices".to_string(),
                        json!(surface_node_indices),
                    );
                }
                obj
            })
            .collect();

        if let Some(pid) = part_filter {
            if parts_json.is_empty() {
                return Err(ServerError::PartNotFound(pid.to_string()));
            }
        }

        let fragments_json: Vec<Value> = frame
            .detached_fragments
            .iter()
            .map(|f| {
                let mut fobj = json!({
                    "fragment_id": f.fragment_id,
                    "source_part_id": f.source_part_id,
                    "detachment_step": f.detachment_step,
                    "mass_kg": f.mass_kg,
                    "position": [
                        f.position_m[0] * scale_len,
                        f.position_m[1] * scale_len,
                        f.position_m[2] * scale_len
                    ],
                    "position_m": f.position_m,
                    "orientation_quat": f.orientation_quat,
                    "linear_velocity_m_s": f.linear_velocity_m_s,
                    "angular_velocity_rad_s": f.angular_velocity_rad_s,
                    "kinetic_energy_j": f.kinetic_energy_j,
                    "element_count": f.element_count,
                    "bounding_radius_m": f.bounding_radius_m,
                });
                if include_fragment_meshes {
                    if let Some(fm) = frag_mesh_by_id.get(f.fragment_id.as_str()) {
                        fobj.as_object_mut()
                            .expect("json object")
                            .insert("mesh".to_string(), json!(fm));
                    }
                }
                fobj
            })
            .collect();

        Ok(json!({
            "simulation_id": record.simulation_id,
            "job_id": record.job_id,
            "accuracy_level": record.accuracy_level.as_str(),
            "frame_index": frame.frame_index,
            "total_frames": record.frames.len(),
            "time_s": frame.time_s,
            "duration_s": record.duration_s,
            "dt_used_s": frame.dt_used_s,
            "newton_iterations": frame.newton_iterations,
            "parts": parts_json,
            "detached_fragments": fragments_json,
            "contact_forces": frame.contact_forces,
            "hub_reactions": frame.hub_reactions,
            "energy_balance": frame.energy,
            "units": unit.units_metadata()
        }))
    });

    match res_opt {
        Some(res) => res,
        None => Err(if let Some(id) = sim_id_opt {
            ServerError::InvalidArgument(format!(
                "SIMULATION_NOT_FOUND: No cached coupled simulation with id '{id}'. Run 'bake_coupled_simulation' first."
            ))
        } else {
            ServerError::InvalidArgument(
                "SIMULATION_NOT_FOUND: No cached coupled simulation exists yet. Run 'bake_coupled_simulation' first."
                    .to_string(),
            )
        }),
    }
}

/// MCP Tool #42: `bake_coupled_simulation`
///
/// Supports `action`: `"bake"` (default), `"status"`, or `"cancel"`.
pub fn bake_coupled_simulation(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;
    let action = args
        .get("action")
        .and_then(|v| v.as_str())
        .unwrap_or("bake")
        .trim()
        .to_ascii_lowercase();

    match action.as_str() {
        "status" => {
            let job_id = args.get("job_id").and_then(|v| v.as_str());
            let wait_ms = args
                .get("wait_timeout_ms")
                .and_then(|v| v.as_u64())
                .unwrap_or(0)
                .min(30_000);

            let initial_status = {
                let m = state.coupled_sim.read();
                m.job_status(job_id).ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "JOB_NOT_FOUND: No coupled simulation bake job matches the requested job_id."
                            .to_string(),
                    )
                })?
            };

            let status = if wait_ms > 0
                && matches!(
                    initial_status.state,
                    BakeJobState::Running | BakeJobState::Cancelling
                ) {
                let start = Instant::now();
                let timeout = Duration::from_millis(wait_ms);
                let initial_frames = initial_status.frames_cached;
                loop {
                    std::thread::sleep(Duration::from_millis(50));
                    let latest_opt = {
                        let m = state.coupled_sim.read();
                        m.job_status(Some(&initial_status.job_id))
                    };
                    let Some(latest) = latest_opt else {
                        break initial_status;
                    };
                    let finished = !matches!(
                        latest.state,
                        BakeJobState::Running | BakeJobState::Cancelling
                    );
                    let got_new_frame = latest.frames_cached > initial_frames;
                    if finished || got_new_frame || start.elapsed() >= timeout {
                        break latest;
                    }
                }
            } else {
                initial_status
            };

            let next_step = match status.state {
                BakeJobState::Running | BakeJobState::Cancelling => {
                    if let Some(latest_idx) = status.latest_frame_index {
                        format!(
                            "Inspect cached frame 0..{latest_idx} via query_simulation_frame {{\"simulation_id\": \"{}\", \"frame_index\": {latest_idx}}} or poll status again.",
                            status.simulation_id
                        )
                    } else {
                        "Solver is advancing the first step; poll status again with wait_timeout_ms.".to_string()
                    }
                }
                BakeJobState::Completed | BakeJobState::Cancelled => format!(
                    "Call query_simulation_frame {{\"simulation_id\": \"{}\"}} to inspect all {} cached frames.",
                    status.simulation_id, status.frames_cached
                ),
                BakeJobState::Failed => "Check parameters and retry bake_coupled_simulation.".to_string(),
            };

            let cached_simulations = {
                let m = state.coupled_sim.read();
                m.list_cached_simulation_ids()
            };
            Ok(json!({
                "job": status,
                "cached_simulations": cached_simulations,
                "next_step": next_step,
                "units": unit.units_metadata()
            }))
        }
        "cancel" => {
            let job_id = args
                .get("job_id")
                .and_then(|v| v.as_str())
                .map(str::to_string)
                .or_else(|| {
                    let m = state.coupled_sim.read();
                    m.job_status(None).map(|j| j.job_id)
                })
                .ok_or_else(|| {
                    ServerError::InvalidArgument(
                        "JOB_NOT_FOUND: Provide 'job_id' to cancel an active bake job.".to_string(),
                    )
                })?;
            let cancelled = {
                let mut m = state.coupled_sim.write();
                m.cancel_job(&job_id)
            };
            if !cancelled {
                return Err(ServerError::InvalidArgument(format!(
                    "JOB_NOT_FOUND: No bake job with job_id '{job_id}'"
                )));
            }
            Ok(json!({
                "status": "cancel_requested",
                "job_id": job_id,
                "units": unit.units_metadata()
            }))
        }
        "bake" => {
            let prepared = prepare_coupled_bake_from_parts_with_physics(
                args,
                &state.parts,
                Some(&state.physics_world),
                unit,
            )?;
            launch_prepared_bake(
                prepared,
                Arc::clone(&state.coupled_sim),
                state.coupled_sim_events_tx.clone(),
                unit,
            )
        }
        other => Err(ServerError::InvalidArgument(format!(
            "Invalid 'action' '{other}': expected 'bake', 'status', or 'cancel'"
        ))),
    }
}

/// MCP Tool #43: `query_simulation_frame`
pub fn query_simulation_frame(args: &Value, state: &mut EngineState) -> Result<Value, ServerError> {
    let unit = state.resolve_length_unit(args)?;
    query_cached_frame_from_manager(args, &state.coupled_sim, unit)
}
