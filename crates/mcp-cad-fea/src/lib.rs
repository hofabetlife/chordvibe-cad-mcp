//! MCP-CAD FEA: Pure-Rust 3D Finite Element Analysis (FEA) linear and nonlinear elastoplastic engine.
//!
//! Provides automatic tetrahedral meshing (`TET4` and quadratic `TET10`), isotropic Hooke's law
//! elasticity, $J_2$ (von Mises) Radial Return Mapping elastoplasticity with consistent tangent,
//! Co-Rotated FEM finite-rotation formulation ($\mathbf{F} = \mathbf{R}\mathbf{U}$), incremental
//! Newton-Raphson solver with adaptive sub-incrementation and line search, sparse PCG linear solver,
//! von Mises stress recovery, and Factor of Safety (FoS) evaluation.

#![deny(unsafe_code)]

pub mod boundary;
pub mod conforming_mesh;
pub mod contact;
pub mod convergence;
pub mod damage;
pub mod dynamics;
pub mod elasticity;
pub mod element;
pub mod error;
pub mod fracture;
pub mod result;
pub mod solver;
pub mod sparse_solver;
pub mod tetra_mesh;

pub use boundary::{
    build_simulation_boundary_conditions, extract_face_triangles, match_joint_anchor_to_cad_face,
    match_surface_nodes_and_weights, point_to_triangle_distance_3d,
    select_321_minimal_support_constraints, AabbFace, BoundaryConditions, DirichletConstraint,
    DofComponent, HubCoupling, HubNodeSelector, MatchedSurfaceNode, NeumannLoad,
    PrismaticHubDrive, ResolvedBoundary, ResolvedHubCoupling, ResolvedHubCouplingKind,
    RevoluteHubDrive, SimulationInertialLoadSpec, SimulationJointLoadSpec,
};
pub use contact::{
    solve_deformable_contact, AnalyticalRigidSurface, ContactBody, ContactDiscretization,
    ContactPairSpec, ContactPatchSummary, ContactPointResult, ContactSolverOptions,
    ContactStepReport, ContactStepSpec, DeformableContactReport, DeformedSurfaceMesh,
    HubCouplingResult, SurfaceContactProjection,
};
pub use conforming_mesh::{
    closest_point_on_segment_3d, closest_point_on_triangle_3d, evaluate_mesh_metrics,
    snap_mesh_to_surface, tet_det_6v, tet_liu_joe_quality, tet_scaled_jacobian,
    to_tet10_boundary_snapped, AccuracyMode, ConformingMeshOptions, LocalRefinementOptions,
    MeshQualityMetrics, RefinementZone, SnapOptions, SurfaceSpatialIndex,
};
pub use convergence::{
    density_label_str, has_curved_or_non_axial_surface, run_fea_with_convergence,
    run_fea_with_convergence_with_accuracy, run_fea_with_convergence_with_order,
    run_mesh_convergence_study_mesh, run_mesh_convergence_study_part,
    CONVERGENCE_DISP_TOLERANCE_PCT, CONVERGENCE_STRESS_TOLERANCE_PCT,
    CURVED_STAIRCASE_STRESS_RATIO_THRESHOLD, FeaAccuracy, MIN_EFFECTIVE_REFINEMENT_RATIO,
    MeshConvergenceStep, MeshConvergenceStudyReport,
};
pub use damage::{
    characteristic_element_length, effective_stiffness_factor, evaluate_element_damage,
    principal_stresses_and_directions, rice_tracey_ductility_factor, stress_triaxiality,
    DamageMaterialParams, DamageUpdateResult, ElementDamageState,
};
pub use dynamics::{
    assemble_mass_matrix_tet10, assemble_mass_matrix_tet4, estimate_cfl_critical_dt,
    frame_bodies_by_id, solve_dynamic_hht_coupled, solve_dynamic_hht_coupled_with_frames,
    solve_dynamic_hht_single_body,
    DynamicBodyFrame, DynamicBodySpec, DynamicContactForce, DynamicEnergyBalance,
    DynamicFrameSnapshot, DynamicHhtReport, DynamicHubCoupling, DynamicHubReaction,
    DynamicPrismaticDrive, DynamicProgressUpdate, DynamicRevoluteDrive, DynamicTimelineEvent,
    HhtOptions, MassMatrixKind, TimeForceCurve,
};
pub use elasticity::{
    bulk_modulus, elasticity_matrix_from_material, isotropic_elasticity_matrix, radial_return_j2,
    shear_modulus, von_mises_from_voigt, GaussPointState, HardeningLaw, J2MaterialParams,
    ReturnMappingResult,
};
pub use element::{
    check_tet10_gauss_jacobians, compute_volume_and_b_matrix, corotated_biot_strain_voigt,
    matrix3_to_voigt_stress, polar_decomposition_so3, rotate_voigt_strain, rotate_voigt_stress,
    tet10_barycentric_derivatives, tet10_min_gauss_jacobian_ratio, tet10_shape_functions,
    voigt_stress_to_matrix3, Matrix12, Matrix30, Matrix6x12, Matrix6x30, TET10_EDGE_PAIRS,
    TET10_GAUSS_A, TET10_GAUSS_B, TET10_GAUSS_POINTS, TET10_GAUSS_WEIGHT_REF, Tet10Element,
    Tet4Element, Vector12, Vector30,
};
pub use error::FeaError;
pub use fracture::{
    compute_subset_kinematics, extract_component_surface_mesh, find_face_connected_components,
    partition_fractured_body, solve_deformable_contact_with_fracture,
    solve_nonlinear_with_fracture, verify_detachment_conservation, ConservationCheck,
    DetachedFragment, FractureContactReport, FractureContactStepReport, FractureEvent,
    FractureReport, FractureSolverOptions, FractureStepReport, FractureTopologyMode,
    FragmentKinematics,
};
pub use result::{
    compute_fea_results, compute_fea_results_tet10, compute_fea_results_with_boundary,
    compute_fea_results_with_boundary_tet10, compute_nonlinear_fea_results_tet10,
    compute_nonlinear_fea_results_tet4, compute_tet10_element_quality, compute_tet_element_quality,
    ElementQualityStats, FeaDiagnostics, FeaJointLoadSource, FeaReport, FeaSimulationSource,
    NonlinearFeaReport, NonlinearFeaSummary,
};
pub use solver::{
    assemble_and_solve, assemble_and_solve_tet10, compute_nodal_reactions,
    compute_nodal_reactions_tet10, compute_nonlinear_nodal_reactions_tet10,
    compute_nonlinear_nodal_reactions_tet4, solve_nonlinear_history_tet10,
    solve_nonlinear_history_tet4, solve_nonlinear_tet10, solve_nonlinear_tet4, solve_pcg,
    CsrMatrix, NonlinearSolveResult, NonlinearSolverOptions, NonlinearState, ResolvedLoadStage,
    SolverOptions, StepHistoryPoint,
};
pub use sparse_solver::{
    rcm_ordering, solve_ic_pcg, solve_linear_robust, solve_spd_newton_robust, solve_ssor_pcg,
    LinearSolverKind, SparseLdlt,
};
pub use tetra_mesh::{
    mesh_max_edge_length, refine_mesh_max_edge_length, ElementOrder, MeshDensity, TetElement,
    TetNode, TetrahedralMesh,
};
pub use mcp_cad_core::tessellation::{TessellationQuality, TessellationSettings};

use std::time::Instant;
use mcp_cad_core::material::Material;
use mcp_cad_core::part::Part;
use mcp_cad_core::topology::TriangleMesh;

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

/// High-level entry point to mesh a closed surface `TriangleMesh` with the requested `AccuracyMode`
/// (`AccuracyMode::High` uses boundary-conforming locally-refined curved `TET10`) and execute FEA.
pub fn analyze_surface_mesh_with_accuracy(
    part_id: impl Into<String>,
    surface_mesh: &TriangleMesh,
    material: &Material,
    boundary: &BoundaryConditions,
    density: MeshDensity,
    accuracy_mode: AccuracyMode,
    options: Option<SolverOptions>,
) -> Result<(FeaReport, TetrahedralMesh), FeaError> {
    let tet_mesh = TetrahedralMesh::from_mesh_with_accuracy(surface_mesh, density, accuracy_mode)?;
    let report = analyze_mesh(part_id, &tet_mesh, material, boundary, options)?;
    Ok((report, tet_mesh))
}

/// High-level entry point to mesh a CAD `Part` with the requested `AccuracyMode`
/// (`AccuracyMode::High` uses boundary-conforming locally-refined curved `TET10`) and execute FEA.
pub fn analyze_part_with_accuracy(
    part: &Part,
    boundary: &BoundaryConditions,
    density: MeshDensity,
    accuracy_mode: AccuracyMode,
    options: Option<SolverOptions>,
) -> Result<FeaReport, FeaError> {
    let mesh = TetrahedralMesh::from_part_with_accuracy(part, density, accuracy_mode)?;
    analyze_mesh(&part.id, &mesh, &part.material, boundary, options)
}

/// High-level load stage descriptor for multi-stage nonlinear elastoplastic FEA histories
/// (e.g. monotonic loading followed by complete unloading / elastic springback).
#[derive(Debug, Clone)]
pub struct NonlinearLoadStage {
    /// Target boundary conditions at the end of this stage ($\lambda = 1$).
    pub boundary: BoundaryConditions,
    /// Optional override for the initial number of increments in this stage.
    pub load_steps: Option<usize>,
    /// Optional descriptive label for telemetry history.
    pub label: Option<String>,
}

impl NonlinearLoadStage {
    /// Creates a new load stage with target `boundary` conditions.
    pub fn new(boundary: BoundaryConditions) -> Self {
        Self {
            boundary,
            load_steps: None,
            label: None,
        }
    }

    /// Sets the initial number of load increments for this stage.
    pub fn with_steps(mut self, steps: usize) -> Self {
        self.load_steps = Some(steps);
        self
    }

    /// Attaches a human-readable stage label (e.g. `"loading"`, `"unloading"`).
    pub fn with_label(mut self, label: impl Into<String>) -> Self {
        self.label = Some(label.into());
        self
    }
}

/// Executes a multi-stage nonlinear elastoplastic Co-Rotated FEM simulation on a `TetrahedralMesh`
/// using explicit `J2MaterialParams` (supports linear, power-law, and saturation isotropic hardening).
/// Automatically dispatches to `TET10` when `mesh.is_tet10()` is `true`, otherwise `TET4`.
pub fn analyze_mesh_nonlinear_history(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    params: &J2MaterialParams,
    stages: &[NonlinearLoadStage],
    options: Option<NonlinearSolverOptions>,
    initial_state: Option<NonlinearState>,
) -> Result<NonlinearFeaReport, FeaError> {
    let start_time = Instant::now();

    if mesh.is_empty() {
        return Err(FeaError::EmptyMesh);
    }

    let opts = options.unwrap_or_default();
    let mut resolved_bcs = Vec::with_capacity(stages.len());
    for stage in stages {
        resolved_bcs.push(stage.boundary.resolve(mesh)?);
    }
    let resolved_stages: Vec<ResolvedLoadStage<'_>> = stages
        .iter()
        .zip(resolved_bcs.iter())
        .map(|(s, rb)| ResolvedLoadStage {
            boundary: rb,
            load_steps: s.load_steps,
            label: s.label.clone(),
        })
        .collect();
    let final_bc = resolved_bcs.last();

    if mesh.is_tet10() {
        let mut elements = Vec::with_capacity(mesh.tet10_elements.len());
        for (id, &node_indices) in mesh.tet10_elements.iter().enumerate() {
            let mut positions = [[0.0f64; 3]; 10];
            for (k, &nid) in node_indices.iter().enumerate() {
                positions[k] = mesh.nodes[nid].position;
            }
            elements.push(Tet10Element::new(id, node_indices, positions)?);
        }

        let solve_result = solve_nonlinear_history_tet10(
            mesh,
            &elements,
            params,
            &resolved_stages,
            opts,
            initial_state,
        )?;
        let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
        return Ok(compute_nonlinear_fea_results_tet10(
            part_id,
            mesh,
            &elements,
            final_bc,
            solve_result,
            params.yield_strength_pa,
            opts.corotated,
            elapsed_ms,
        ));
    }

    let mut elements = Vec::with_capacity(mesh.elements.len());
    for elem in &mesh.elements {
        let p0 = mesh.nodes[elem.node_indices[0]].position;
        let p1 = mesh.nodes[elem.node_indices[1]].position;
        let p2 = mesh.nodes[elem.node_indices[2]].position;
        let p3 = mesh.nodes[elem.node_indices[3]].position;
        elements.push(Tet4Element::new(elem.id, elem.node_indices, [p0, p1, p2, p3])?);
    }

    let solve_result = solve_nonlinear_history_tet4(
        mesh,
        &elements,
        params,
        &resolved_stages,
        opts,
        initial_state,
    )?;
    let elapsed_ms = start_time.elapsed().as_secs_f64() * 1000.0;
    Ok(compute_nonlinear_fea_results_tet4(
        part_id,
        mesh,
        &elements,
        final_bc,
        solve_result,
        params.yield_strength_pa,
        opts.corotated,
        elapsed_ms,
    ))
}

/// Executes a single-stage nonlinear elastoplastic Co-Rotated FEM simulation on a `TetrahedralMesh`
/// using explicit `J2MaterialParams`.
pub fn analyze_mesh_nonlinear_with_params(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    params: &J2MaterialParams,
    boundary: &BoundaryConditions,
    options: Option<NonlinearSolverOptions>,
) -> Result<NonlinearFeaReport, FeaError> {
    let stage = NonlinearLoadStage::new(boundary.clone()).with_label("load");
    analyze_mesh_nonlinear_history(part_id, mesh, params, &[stage], options, None)
}

/// Executes a single-stage nonlinear elastoplastic Co-Rotated FEM simulation on a `TetrahedralMesh`
/// deriving $J_2$ constitutive parameters from a CAD `Material`.
pub fn analyze_mesh_nonlinear(
    part_id: impl Into<String>,
    mesh: &TetrahedralMesh,
    material: &Material,
    boundary: &BoundaryConditions,
    options: Option<NonlinearSolverOptions>,
) -> Result<NonlinearFeaReport, FeaError> {
    let params = J2MaterialParams::from_material(material)?;
    analyze_mesh_nonlinear_with_params(part_id, mesh, &params, boundary, options)
}

/// Meshes a CAD `Part` and executes a single-stage nonlinear elastoplastic Co-Rotated FEM simulation.
pub fn analyze_part_nonlinear(
    part: &Part,
    boundary: &BoundaryConditions,
    density: MeshDensity,
    element_order: ElementOrder,
    options: Option<NonlinearSolverOptions>,
) -> Result<NonlinearFeaReport, FeaError> {
    let base_mesh = TetrahedralMesh::from_part(part, density)?;
    let mesh = match element_order {
        ElementOrder::Linear => base_mesh,
        ElementOrder::Quadratic => base_mesh.to_tet10(),
    };
    analyze_mesh_nonlinear(&part.id, &mesh, &part.material, boundary, options)
}
