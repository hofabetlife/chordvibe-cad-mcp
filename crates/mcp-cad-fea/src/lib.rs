//! MCP-CAD FEA: Pure-Rust 3D Finite Element Analysis (FEA) linear elasticity engine.
//!
//! Provides automatic tetrahedral meshing (TET4), isotropic Hooke's law elasticity matrix,
//! sparse global stiffness assembly, Preconditioned Conjugate Gradient (PCG) solver,
//! von Mises equivalent stress recovery, and Factor of Safety (FoS) evaluation.

#![deny(unsafe_code)]

pub mod boundary;
pub mod elasticity;
pub mod element;
pub mod error;
pub mod result;
pub mod solver;
pub mod tetra_mesh;

pub use boundary::{
    AabbFace, BoundaryConditions, DirichletConstraint, DofComponent, NeumannLoad, ResolvedBoundary,
};
pub use elasticity::{
    bulk_modulus, elasticity_matrix_from_material, isotropic_elasticity_matrix, shear_modulus,
};
pub use element::{Tet4Element, compute_volume_and_b_matrix};
pub use error::FeaError;
pub use result::{compute_fea_results, FeaReport};
pub use solver::{assemble_and_solve, solve_pcg, CsrMatrix, SolverOptions};
pub use tetra_mesh::{MeshDensity, TetElement, TetNode, TetrahedralMesh};

use std::time::Instant;
use mcp_cad_core::material::Material;
use mcp_cad_core::part::Part;

/// High-level entry point to execute an FEA simulation on a `TetrahedralMesh`.
pub fn analyze_mesh(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    material: &Material,
    boundary: &BoundaryConditions,
    options: Option<SolverOptions>,
) -> Result<FeaReport, FeaError> {
    let start_time = Instant::now();

    if mesh.is_empty() {
        return Err(FeaError::EmptyMesh);
    }

    // 1. Compute material elasticity matrix D
    let d_matrix = elasticity_matrix_from_material(material)?;

    // 2. Build Tet4Elements with volumes and B matrices
    let mut elements = Vec::with_capacity(mesh.elements.len());
    for elem in &mesh.elements {
        let p0 = mesh.nodes[elem.node_indices[0]].position;
        let p1 = mesh.nodes[elem.node_indices[1]].position;
        let p2 = mesh.nodes[elem.node_indices[2]].position;
        let p3 = mesh.nodes[elem.node_indices[3]].position;

        let tet = Tet4Element::new(elem.id, elem.node_indices, [p0, p1, p2, p3])?;
        elements.push(tet);
    }

    // 3. Resolve boundary conditions
    let resolved_boundary = boundary.resolve(mesh)?;

    // 4. Assemble global stiffness matrix and solve K * u = F
    let u_full = assemble_and_solve(mesh, &elements, &d_matrix, &resolved_boundary, options)?;

    // 5. Post-process displacements, stresses, and factor of safety
    let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
    let report = compute_fea_results(
        part_id,
        mesh,
        &elements,
        &d_matrix,
        &u_full,
        material.yield_strength_pa(),
        elapsed_ms,
    );

    Ok(report)
}

/// High-level entry point to mesh and execute FEA analysis directly on a CAD `Part`.
pub fn analyze_part(
    part: &Part,
    boundary: &BoundaryConditions,
    density: MeshDensity,
    options: Option<SolverOptions>,
) -> Result<FeaReport, FeaError> {
    let mesh = TetrahedralMesh::from_part(part, density)?;
    analyze_mesh(&part.id, &mesh, &part.material, boundary, options)
}
