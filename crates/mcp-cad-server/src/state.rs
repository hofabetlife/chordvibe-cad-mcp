//! Server state management and in-memory CAD engine registries.

use std::collections::HashMap;
use std::sync::Arc;
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use mcp_cad_core::material::MaterialRegistry;
use mcp_cad_core::part::Part;
use mcp_cad_core::topology::TriangleMesh;
use mcp_cad_fea::result::FeaReport;
use mcp_cad_fea::tetra_mesh::TetrahedralMesh;
use mcp_cad_physics::simulator::PhysicsWorld;
use mcp_cad_physics::telemetry::SimulationSummary;
use mcp_cad_viewer::offscreen::OffscreenRenderer;
use mcp_cad_viewer::window::{GuiEvent, GuiProxy};

use crate::error::ServerError;

/// Active linear dimension unit for the MCP boundary contract (`[DECISION-UNITS-1]`).
/// Internal CAD/FEA/Physics kernels strictly operate in SI (`m`, `Pa`, `N`, `kg`, `s`).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LengthUnit {
    #[default]
    #[serde(rename = "mm")]
    Mm,
    #[serde(rename = "m")]
    M,
}

impl LengthUnit {
    /// Parses `"mm"` or `"m"` (case-insensitive) into a `LengthUnit`.
    pub fn parse(s: &str) -> Result<Self, ServerError> {
        match s.trim().to_ascii_lowercase().as_str() {
            "mm" | "millimeter" | "millimeters" | "millimetre" | "millimetres" => Ok(Self::Mm),
            "m" | "meter" | "meters" | "metre" | "metres" => Ok(Self::M),
            other => Err(ServerError::InvalidArgument(format!(
                "Unsupported length_unit '{other}'. Supported values are 'mm' and 'm'."
            ))),
        }
    }

    /// Canonical string identifier (`"mm"` or `"m"`).
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Mm => "mm",
            Self::M => "m",
        }
    }

    /// Canonical area unit string (`"mm²"` or `"m²"`).
    pub fn area_str(&self) -> &'static str {
        match self {
            Self::Mm => "mm²",
            Self::M => "m²",
        }
    }

    /// Canonical volume unit string (`"mm³"` or `"m³"`).
    pub fn volume_str(&self) -> &'static str {
        match self {
            Self::Mm => "mm³",
            Self::M => "m³",
        }
    }

    /// Multiplicative factor to convert a linear dimension from `self` into internal SI meters (`m`).
    pub fn to_meters_factor(&self) -> f64 {
        match self {
            Self::Mm => 1e-3,
            Self::M => 1.0,
        }
    }

    /// Multiplicative factor to convert a linear dimension from internal SI meters (`m`) into `self`.
    pub fn from_meters_factor(&self) -> f64 {
        match self {
            Self::Mm => 1e3,
            Self::M => 1.0,
        }
    }

    /// Returns the full JSON `"units"` contract object for MCP tool responses.
    pub fn units_metadata(&self) -> Value {
        json!({
            "length": self.as_str(),
            "area": self.area_str(),
            "volume": self.volume_str(),
            "volume_si": "m³",
            "mass": "kg",
            "density": "kg/m³",
            "force": "N",
            "torque": "N·m",
            "stress": "MPa",
            "stress_output": "MPa",
            "young_modulus": "GPa",
            "displacement": "mm",
            "displacement_output": "mm",
            "angle": "deg",
            "angular_velocity": "rad/s",
            "linear_velocity": "m/s",
            "time": "s"
        })
    }
}

/// In-memory state of the MCP-CAD engine.
pub struct EngineState {
    /// Active linear length unit at the MCP boundary (defaults to `LengthUnit::Mm`).
    pub length_unit: LengthUnit,
    /// Engineering materials database (AISI 1020, Al 6061-T6, Ti-6Al-4V, PLA, etc.).
    pub materials: MaterialRegistry,
    /// Active CAD parts keyed by `part_id`.
    pub parts: HashMap<String, Part>,
    /// Rapier3D physics and kinematics simulation world.
    pub physics_world: PhysicsWorld,
    /// Latest FEA analysis reports keyed by `part_id`.
    pub last_fea_reports: HashMap<String, FeaReport>,
    /// Part geometry `revision` at the time each entry in `last_fea_reports` was computed.
    pub last_fea_revisions: HashMap<String, u64>,
    /// Named FEA load cases keyed by `(part_id, fea_case)` storing `(FeaReport, part_revision)`.
    pub fea_case_reports: HashMap<(String, String), (FeaReport, u64)>,
    /// Tetrahedral meshes associated with the last FEA analysis for heatmaps.
    pub last_fea_meshes: HashMap<String, TetrahedralMesh>,
    /// Most recent physics simulation results and telemetry.
    pub last_simulation: Option<SimulationSummary>,
    /// Declared engineering requirements (`set_requirements` / `check_requirements`) and `auto_check` config.
    pub requirements: crate::tools::requirements::RequirementsState,
    /// Headless WGPU offscreen renderer for capturing PNG snapshots and stress heatmaps.
    pub offscreen_renderer: Arc<OffscreenRenderer>,
    /// Optional Winit event loop proxy for sending mesh and FEA updates to the interactive GUI window.
    pub gui_proxy: Option<GuiProxy>,
}

/// Thread-safe shared handle to the engine state.
pub type SharedState = Arc<RwLock<EngineState>>;

impl EngineState {
    /// Initializes a new engine state with default material database and WGPU renderer.
    pub fn new() -> Result<Self, ServerError> {
        let offscreen_renderer = Arc::new(OffscreenRenderer::new()?);
        Ok(Self::with_renderer(offscreen_renderer))
    }

    /// Initializes engine state with an existing offscreen renderer instance.
    pub fn with_renderer(renderer: Arc<OffscreenRenderer>) -> Self {
        Self {
            length_unit: LengthUnit::Mm,
            materials: MaterialRegistry::new(),
            parts: HashMap::new(),
            physics_world: PhysicsWorld::new(),
            last_fea_reports: HashMap::new(),
            last_fea_revisions: HashMap::new(),
            fea_case_reports: HashMap::new(),
            last_fea_meshes: HashMap::new(),
            last_simulation: None,
            requirements: crate::tools::requirements::RequirementsState::default(),
            offscreen_renderer: renderer,
            gui_proxy: None,
        }
    }

    /// Resolves the effective `LengthUnit` for a tool invocation: uses optional `"length_unit"` override
    /// in `args` if provided, otherwise falls back to `self.length_unit`.
    pub fn resolve_length_unit(&self, args: &Value) -> Result<LengthUnit, ServerError> {
        if let Some(u_str) = args.get("length_unit").and_then(|v| v.as_str()) {
            LengthUnit::parse(u_str)
        } else {
            Ok(self.length_unit)
        }
    }

    /// Checks a part's internal SI dimensions and explicit feature dimensions (in meters)
    /// against engineering sanity heuristics (`docs/UNITS.md`) and returns any warnings.
    pub fn check_scale_sanity(
        &self,
        part: &Part,
        extra_dims_m: &[f64],
        wheel_rim_diameter_m: Option<f64>,
        unit: LengthUnit,
    ) -> Vec<String> {
        let mut warnings = Vec::new();
        let aabb = part.bounding_box();
        let dims = aabb.dimensions();
        let extents_m = [dims[0].max(0.0), dims[1].max(0.0), dims[2].max(0.0)];

        let max_extent_m = extents_m
            .iter()
            .copied()
            .chain(extra_dims_m.iter().copied().map(|d| d.abs()))
            .fold(0.0_f64, f64::max);

        let min_positive_dim_m = extents_m
            .iter()
            .copied()
            .chain(extra_dims_m.iter().copied().map(|d| d.abs()))
            .filter(|&d| d > 0.0)
            .fold(f64::INFINITY, f64::min);

        let u_str = unit.as_str();
        let k_from_m = unit.from_meters_factor();

        if max_extent_m > 10.0 {
            warnings.push(format!(
                "SCALE_TOO_LARGE: Characteristic dimension is {:.4} m ({:.2} {u_str}) > 10.0 m. Check whether values were passed in millimeters while length_unit='m', or represent an oversized structure.",
                max_extent_m,
                max_extent_m * k_from_m
            ));
        }

        if min_positive_dim_m.is_finite() && min_positive_dim_m < 1e-4 {
            warnings.push(format!(
                "SCALE_TOO_SMALL: Characteristic dimension is {:.6e} m ({:.6} {u_str}) < 0.1 mm (1e-4 m). Check whether values were passed in meters while length_unit='mm'.",
                min_positive_dim_m,
                min_positive_dim_m * k_from_m
            ));
        }

        let vol_m3 = part.volume();
        if vol_m3 > 0.0 && !(1e-10..=100.0).contains(&vol_m3) {
            warnings.push(format!(
                "SCALE_EXTREME_VOLUME: Part volume is {:.6e} m^3 (outside [1e-10, 100.0] m^3). Verify length_unit ('{u_str}') and input dimensions.",
                vol_m3
            ));
        }

        if let Some(rim_d_m) = wheel_rim_diameter_m {
            if rim_d_m < 0.15 {
                warnings.push(format!(
                    "WHEEL_RIM_SCALE: rim_diameter is {:.4} m ({:.2} {u_str}) < 0.15 m (150 mm), which is abnormally small for a wheel rim.",
                    rim_d_m,
                    rim_d_m * k_from_m
                ));
            }
        }

        warnings
    }

    /// Attaches an interactive GUI event loop proxy to the state.
    pub fn with_gui_proxy(mut self, proxy: GuiProxy) -> Self {
        self.gui_proxy = Some(proxy);
        self
    }

    /// Sends an updated CAD solid mesh to the interactive GUI preview window if active.
    pub fn send_gui_mesh(&self, mesh: &TriangleMesh) {
        if let Some(proxy) = &self.gui_proxy {
            let _ = proxy.send_event(GuiEvent::UpdateMesh(mesh.clone()));
        }
    }

    /// Sends FEA stress colormap data and surface mesh to the interactive GUI preview window if active.
    pub fn send_gui_fea(&self, mesh: &TriangleMesh, stresses: &[f32]) {
        if let Some(proxy) = &self.gui_proxy {
            let _ = proxy.send_event(GuiEvent::UpdateFea {
                mesh: mesh.clone(),
                stresses: stresses.to_vec(),
            });
        }
    }

    /// Sends multi-body physics simulation animation frames to the interactive GUI preview window for real-time playback.
    pub fn send_gui_animation(&self, frames: Vec<TriangleMesh>, fps: f32) {
        if let Some(proxy) = &self.gui_proxy {
            let _ = proxy.send_event(GuiEvent::SetAnimation { frames, fps });
        }
    }

    /// Wraps this state into an `Arc<RwLock<EngineState>>`.
    pub fn into_shared(self) -> SharedState {
        Arc::new(RwLock::new(self))
    }
}

