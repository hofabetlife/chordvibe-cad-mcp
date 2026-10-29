//! Server state management and in-memory CAD engine registries.

use std::collections::HashMap;
use std::sync::Arc;
use parking_lot::RwLock;

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

/// In-memory state of the MCP-CAD engine.
pub struct EngineState {
    /// Engineering materials database (AISI 1020, Al 6061-T6, Ti-6Al-4V, PLA, etc.).
    pub materials: MaterialRegistry,
    /// Active CAD parts keyed by `part_id`.
    pub parts: HashMap<String, Part>,
    /// Rapier3D physics and kinematics simulation world.
    pub physics_world: PhysicsWorld,
    /// Latest FEA analysis reports keyed by `part_id`.
    pub last_fea_reports: HashMap<String, FeaReport>,
    /// Tetrahedral meshes associated with the last FEA analysis for heatmaps.
    pub last_fea_meshes: HashMap<String, TetrahedralMesh>,
    /// Most recent physics simulation results and telemetry.
    pub last_simulation: Option<SimulationSummary>,
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
            materials: MaterialRegistry::new(),
            parts: HashMap::new(),
            physics_world: PhysicsWorld::new(),
            last_fea_reports: HashMap::new(),
            last_fea_meshes: HashMap::new(),
            last_simulation: None,
            offscreen_renderer: renderer,
            gui_proxy: None,
        }
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

    /// Wraps this state into an `Arc<RwLock<EngineState>>`.
    pub fn into_shared(self) -> SharedState {
        Arc::new(RwLock::new(self))
    }
}
