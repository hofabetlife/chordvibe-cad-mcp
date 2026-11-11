//! MCP-CAD FEA: Pure-Rust 3D Finite Element Analysis (FEA) linear elasticity engine.
//!
//! Provides automatic tetrahedral meshing (`TET4` and quadratic `TET10`), isotropic Hooke's law
//! elasticity matrix, sparse global stiffness assembly, Preconditioned Conjugate Gradient (PCG)
//! solver, von Mises equivalent stress recovery, and Factor of Safety (FoS) evaluation.

#![deny(unsafe_code)]

pub mod boundary;
pub mod convergence;
pub mod elasticity;
pub mod element;
pub mod error;
pub mod result;
pub mod solver;
pub mod tetra_mesh;

pub use boundary::{
    extract_face_triangles, match_surface_nodes_and_weights, point_to_triangle_distance_3d,
    AabbFace, BoundaryConditions, DirichletConstraint, DofComponent, MatchedSurfaceNode,
    NeumannLoad, ResolvedBoundary,
};
pub use convergence::{
    density_label_str, has_curved_or_non_axial_surface, run_fea_with_convergence,
    run_fea_with_convergence_with_order, run_mesh_convergence_study_mesh,
    run_mesh_convergence_study_part, CONVERGENCE_DISP_TOLERANCE_PCT,
    CONVERGENCE_STRESS_TOLERANCE_PCT, CURVED_STAIRCASE_STRESS_RATIO_THRESHOLD, FeaAccuracy,
    MIN_EFFECTIVE_REFINEMENT_RATIO, MeshConvergenceStep, MeshConvergenceStudyReport,
};
pub use elasticity::{
    bulk_modulus, elasticity_matrix_from_material, isotropic_elasticity_matrix, shear_modulus,
};
pub use element::{
    compute_volume_and_b_matrix, tet10_barycentric_derivatives, tet10_shape_functions, Matrix12,
    Matrix30, Matrix6x12, Matrix6x30, TET10_EDGE_PAIRS, TET10_GAUSS_A, TET10_GAUSS_B,
    TET10_GAUSS_POINTS, TET10_GAUSS_WEIGHT_REF, Tet10Element, Tet4Element, Vector12, Vector30,
};
pub use error::FeaError;
pub use result::{
    compute_fea_results, compute_fea_results_tet10, compute_fea_results_with_boundary,
    compute_fea_results_with_boundary_tet10, compute_tet10_element_quality,
    compute_tet_element_quality, ElementQualityStats, FeaDiagnostics, FeaReport,
};
pub use solver::{
    assemble_and_solve, assemble_and_solve_tet10, compute_nodal_reactions,
    compute_nodal_reactions_tet10, solve_pcg, CsrMatrix, SolverOptions,
};
pub use tetra_mesh::{
    mesh_max_edge_length, refine_mesh_max_edge_length, ElementOrder, MeshDensity, TetElement,
    TetNode, TetrahedralMesh,
};
pub use mcp_cad_core::tessellation::{TessellationQuality, TessellationSettings};

use std::time::Instant;
use mcp_cad_core::material::Material;
use mcp_cad_core::part::Part;

/// High-level entry point to execute an FEA simulation on a `TetrahedralMesh`
/// (automatically dispatches to `TET10` when `mesh.is_tet10()` is `true`, otherwise `TET4`).
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

    // 2. If mesh is quadratic (TET10), run the 10-node isoparametric pipeline
    if mesh.is_tet10() {
        let mut elements = Vec::with_capacity(mesh.tet10_elements.len());
        for (id, &node_indices) in mesh.tet10_elements.iter().enumerate() {
            let mut positions = [[0.0f64; 3]; 10];
            for (k, &nid) in node_indices.iter().enumerate() {
                positions[k] = mesh.nodes[nid].position;
            }
            let tet10 = Tet10Element::new(id, node_indices, positions)?;
            elements.push(tet10);
        }

        let resolved_boundary = boundary.resolve(mesh)?;
        let u_full =
            assemble_and_solve_tet10(mesh, &elements, &d_matrix, &resolved_boundary, options)?;

        let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
        let report = compute_fea_results_with_boundary_tet10(
            part_id,
            mesh,
            &elements,
            &d_matrix,
            Some(&resolved_boundary),
            &u_full,
            material.yield_strength_pa(),
            elapsed_ms,
        );
        return Ok(report);
    }

    // 3. Build linear Tet4Elements with volumes and B matrices
    let mut elements = Vec::with_capacity(mesh.elements.len());
    for elem in &mesh.elements {
        let p0 = mesh.nodes[elem.node_indices[0]].position;
        let p1 = mesh.nodes[elem.node_indices[1]].position;
        let p2 = mesh.nodes[elem.node_indices[2]].position;
        let p3 = mesh.nodes[elem.node_indices[3]].position;

        let tet = Tet4Element::new(elem.id, elem.node_indices, [p0, p1, p2, p3])?;
        elements.push(tet);
    }

    // 4. Resolve boundary conditions
    let resolved_boundary = boundary.resolve(mesh)?;

    // 5. Assemble global stiffness matrix and solve K * u = F
    let u_full = assemble_and_solve(mesh, &elements, &d_matrix, &resolved_boundary, options)?;

    // 6. Post-process displacements, stresses, equilibrium reactions, and factor of safety
    let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
    let report = compute_fea_results_with_boundary(
        part_id,
        mesh,
        &elements,
        &d_matrix,
        Some(&resolved_boundary),
        &u_full,
        material.yield_strength_pa(),
        elapsed_ms,
    );

    Ok(report)
}

/// High-level entry point to execute an FEA simulation on a `TetrahedralMesh` with explicit `ElementOrder`.
pub fn analyze_mesh_with_order(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    material: &Material,
    boundary: &BoundaryConditions,
    element_order: ElementOrder,
    options: Option<SolverOptions>,
) -> Result<FeaReport, FeaError> {
    match element_order {
        ElementOrder::Linear => analyze_mesh(part_id, mesh, material, boundary, options),
        ElementOrder::Quadratic => {
            if mesh.is_tet10() {
                analyze_mesh(part_id, mesh, material, boundary, options)
            } else {
                let tet10_mesh = mesh.to_tet10();
                analyze_mesh(part_id, &tet10_mesh, material, boundary, options)
            }
        }
    }
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

/// High-level entry point to mesh and execute FEA analysis directly on a CAD `Part` with explicit `ElementOrder`.
pub fn analyze_part_with_order(
    part: &Part,
    boundary: &BoundaryConditions,
    density: MeshDensity,
    element_order: ElementOrder,
    options: Option<SolverOptions>,
) -> Result<FeaReport, FeaError> {
    let mesh = TetrahedralMesh::from_part(part, density)?;
    analyze_mesh_with_order(&part.id, &mesh, &part.material, boundary, element_order, options)
}

/// High-level entry point to mesh and execute FEA analysis directly on a CAD `Part`
/// using automatic adaptive tessellation matching the target mesh density.
pub fn analyze_part_adaptive(
    part: &mut Part,
    boundary: &BoundaryConditions,
    density: MeshDensity,
    options: Option<SolverOptions>,
) -> Result<FeaReport, FeaError> {
    let mesh = TetrahedralMesh::from_part_adaptive(part, density)?;
    analyze_mesh(&part.id, &mesh, &part.material, boundary, options)
}

/// High-level entry point to mesh and execute FEA analysis directly on a CAD `Part`
/// with explicit adaptive tessellation settings.
pub fn analyze_part_with_settings(
    part: &mut Part,
    boundary: &BoundaryConditions,
    settings: &TessellationSettings,
    density: MeshDensity,
    options: Option<SolverOptions>,
) -> Result<FeaReport, FeaError> {
    let mesh = TetrahedralMesh::from_part_with_settings(part, settings, density)?;
    analyze_mesh(&part.id, &mesh, &part.material, boundary, options)
}

/// High-level entry point to mesh and execute FEA analysis directly on a CAD `Part`
/// using a specified tessellation quality preset.
pub fn analyze_part_quality(
    part: &mut Part,
    boundary: &BoundaryConditions,
    quality: TessellationQuality,
    density: MeshDensity,
    options: Option<SolverOptions>,
) -> Result<FeaReport, FeaError> {
    let mesh = TetrahedralMesh::from_part_quality(part, quality, density)?;
    analyze_mesh(&part.id, &mesh, &part.material, boundary, options)
}

