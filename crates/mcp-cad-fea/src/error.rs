//! Error types for Finite Element Analysis (FEA) computations.

use thiserror::Error;

/// Errors that can occur during FEA mesh generation, assembly, solving, or post-processing.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum FeaError {
    /// Raised when a tetrahedral mesh is empty (no nodes or elements).
    #[error("Empty mesh: tetrahedral mesh contains no nodes or elements")]
    EmptyMesh,

    /// Raised when an element has non-positive volume or degenerate geometry.
    #[error("Degenerate element #{id}: non-positive volume {volume:e} m³")]
    DegenerateElement {
        /// Element ID
        id: usize,
        /// Computed volume
        volume: f64,
    },

    /// Raised when material properties are physically invalid.
    #[error("Invalid material properties: Young's modulus E = {youngs_modulus_pa:e} Pa (must be > 0), Poisson's ratio ν = {poissons_ratio} (must be in 0.0..0.5)")]
    InvalidMaterial {
        /// Young's modulus in Pa
        youngs_modulus_pa: f64,
        /// Poisson's ratio
        poissons_ratio: f64,
    },

    /// Raised when the model has insufficient Dirichlet constraints (rigid body motion).
    #[error("Insufficient boundary conditions: model has unconstrained rigid body modes or no Dirichlet constraints")]
    UnconstrainedModel,

    /// Raised when the iterative linear solver fails to converge.
    #[error("Linear solver failed to converge after {iterations} iterations with residual {residual:e} (tolerance: {tolerance:e})")]
    SolverConvergenceFailed {
        /// Number of iterations executed
        iterations: usize,
        /// Final residual norm
        residual: f64,
        /// Target convergence tolerance
        tolerance: f64,
    },

    /// Raised when a referenced node index is out of bounds.
    #[error("Node index {node_id} out of bounds (total nodes: {total_nodes})")]
    NodeOutOfBounds {
        /// Requested node ID
        node_id: usize,
        /// Total nodes in mesh
        total_nodes: usize,
    },

    /// Raised when a boundary condition plane or AABB face matches zero nodes.
    #[error("Boundary selection matched 0 nodes with tolerance {tolerance}")]
    NoBoundaryNodesMatched {
        /// Search tolerance used
        tolerance: f64,
    },

    /// Raised when the input CAD surface mesh is not a closed watertight 2-manifold (`MESH_INVALID`).
    #[error(
        "Invalid CAD surface mesh for FEA (MESH_INVALID): {message} (boundary_edges: {boundary_edges_count}, non_manifold_edges: {non_manifold_edges_count}, non_manifold_vertices: {non_manifold_vertices_count})"
    )]
    InvalidSurfaceMesh {
        /// Human-readable summary of topological/geometric defects
        message: String,
        /// Number of open boundary edges (holes)
        boundary_edges_count: usize,
        /// Number of non-manifold edges (>= 3 incident triangles)
        non_manifold_edges_count: usize,
        /// Number of non-manifold vertices
        non_manifold_vertices_count: usize,
        /// Number of inconsistent/inverted directed edges
        inconsistent_edges_count: usize,
        /// Whether the mesh is watertight
        is_watertight: bool,
    },

    /// Raised when mesh generation fails.
    #[error("Mesh generation failed: {0}")]
    MeshGenerationFailed(String),

    /// Raised when a CAD core operation or evaluation fails.
    #[error("CAD core error: {0}")]
    Cad(String),

    /// General FEA error with descriptive message.
    #[error("FEA error: {0}")]
    General(String),
}

impl From<mcp_cad_core::CadError> for FeaError {
    fn from(err: mcp_cad_core::CadError) -> Self {
        Self::Cad(err.to_string())
    }
}
