//! Global stiffness matrix assembly, Preconditioned Conjugate Gradient (PCG) linear solver,
//! and incremental Newton-Raphson nonlinear elastoplastic Co-Rotated FEM solver (`P5-T3`).

use nalgebra::{Matrix3, Matrix6, Vector3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use crate::boundary::ResolvedBoundary;
use crate::elasticity::{GaussPointState, J2MaterialParams};
use crate::element::{Tet10Element, Tet4Element, Vector12, Vector30};
use crate::error::FeaError;
use crate::sparse_solver::{solve_spd_newton_robust, LinearSolverKind};
use crate::tetra_mesh::TetrahedralMesh;

/// Minimum element count before switching element evaluation loops to `rayon` multi-threading.
const PARALLEL_ELEMENT_THRESHOLD: usize = 64;

pub use crate::contact::{
    solve_deformable_contact, ContactBody, ContactDiscretization, ContactPairSpec,
    ContactSolverOptions, ContactStepReport, ContactStepSpec, DeformableContactReport,
};

/// Configuration options for the iterative FEA linear solver.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SolverOptions {
    /// Relative residual convergence tolerance (default: 1e-7)
    pub tolerance: f64,
    /// Maximum number of PCG iterations (default: 5000)
    pub max_iterations: usize,
}

impl Default for SolverOptions {
    fn default() -> Self {
        Self {
            tolerance: 1e-7,
            max_iterations: 5000,
        }
    }
}

/// Compressed Sparse Row (CSR) matrix representation.
#[derive(Debug, Clone)]
pub struct CsrMatrix {
    /// Number of rows
    pub nrows: usize,
    /// Number of columns
    pub ncols: usize,
    /// Row pointer offsets (length: nrows + 1)
    pub row_ptr: Vec<usize>,
    /// Column indices of non-zero entries
    pub col_indices: Vec<usize>,
    /// Non-zero coefficient values
    pub values: Vec<f64>,
}

impl CsrMatrix {
    /// Constructs a CSR matrix from pre-bucketed per-row `(col, value)` entries,
    /// sorting columns and accumulating duplicate entries.
    pub fn from_row_entries(
        nrows: usize,
        ncols: usize,
        mut row_entries: Vec<Vec<(usize, f64)>>,
    ) -> Self {
        let compress_row = |entries: &mut Vec<(usize, f64)>| {
            if entries.len() <= 1 {
                return;
            }
            entries.sort_unstable_by_key(|&(col, _)| col);
            let mut write = 0usize;
            for read in 1..entries.len() {
                if entries[read].0 == entries[write].0 {
                    entries[write].1 += entries[read].1;
                } else {
                    write += 1;
                    entries[write] = entries[read];
                }
            }
            entries.truncate(write + 1);
        };

        if nrows >= 512 {
            row_entries.par_iter_mut().for_each(compress_row);
        } else {
            row_entries.iter_mut().for_each(compress_row);
        }

        let total_nnz: usize = row_entries.iter().map(Vec::len).sum();
        let mut row_ptr = Vec::with_capacity(nrows + 1);
        let mut col_indices = Vec::with_capacity(total_nnz);
        let mut values = Vec::with_capacity(total_nnz);
        row_ptr.push(0);

        for entries in row_entries {
            for (col, val) in entries {
                col_indices.push(col);
                values.push(val);
            }
            row_ptr.push(col_indices.len());
        }

        Self {
            nrows,
            ncols,
            row_ptr,
            col_indices,
            values,
        }
    }

    /// Constructs a CSR matrix from a list of (row, col, value) triplets,
    /// automatically sorting columns and accumulating duplicate entries.
    pub fn from_triplets(nrows: usize, ncols: usize, triplets: &[(usize, usize, f64)]) -> Self {
        let mut row_entries: Vec<Vec<(usize, f64)>> = vec![Vec::new(); nrows];
        for &(r, c, val) in triplets {
            if r < nrows && c < ncols && val.abs() > 1e-25 {
                row_entries[r].push((c, val));
            }
        }
        Self::from_row_entries(nrows, ncols, row_entries)
    }

    /// Performs Sparse Matrix-Vector multiplication: $y = A \cdot x$.
    #[inline]
    pub fn spmv(&self, x: &[f64], y: &mut [f64]) {
        let nrows = self.nrows.min(y.len());
        if self.values.len() >= 16_384 && nrows >= 512 {
            let row_ptr = &self.row_ptr;
            let col_indices = &self.col_indices;
            let values = &self.values;
            y[..nrows]
                .par_iter_mut()
                .enumerate()
                .for_each(|(row, y_val)| {
                    let start = row_ptr[row];
                    let end = row_ptr[row + 1];
                    let mut sum = 0.0;
                    for idx in start..end {
                        sum += values[idx] * x[col_indices[idx]];
                    }
                    *y_val = sum;
                });
        } else {
            for (row, y_val) in y.iter_mut().enumerate().take(nrows) {
                let start = self.row_ptr[row];
                let end = self.row_ptr[row + 1];
                let mut sum = 0.0;
                for idx in start..end {
                    sum += self.values[idx] * x[self.col_indices[idx]];
                }
                *y_val = sum;
            }
        }
    }

    /// Extracts the diagonal entries $A_{ii}$ of the matrix.
    pub fn get_diagonal(&self) -> Vec<f64> {
        let mut diag = vec![0.0; self.nrows];
        for (row, diag_val) in diag.iter_mut().enumerate().take(self.nrows) {
            let start = self.row_ptr[row];
            let end = self.row_ptr[row + 1];
            for idx in start..end {
                if self.col_indices[idx] == row {
                    *diag_val = self.values[idx];
                    break;
                }
            }
        }
        diag
    }
}

/// Computes the dot product of two slices.
#[inline]
fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(&x, &y)| x * y).sum()
}

/// Solves $A \cdot x = b$ using the Preconditioned Conjugate Gradient method
/// with a Jacobi (diagonal) preconditioner.
pub fn solve_pcg(
    matrix: &CsrMatrix,
    b: &[f64],
    options: SolverOptions,
) -> Result<Vec<f64>, FeaError> {
    let n = matrix.nrows;
    let norm_b = dot(b, b).sqrt();

    // If RHS is effectively zero, solution is zero
    if norm_b <= 1e-15 {
        return Ok(vec![0.0; n]);
    }

    // Compute Jacobi preconditioner: inv_diag = 1 / A_ii
    let diag = matrix.get_diagonal();
    let mut inv_diag = vec![0.0; n];
    for (i, &d) in diag.iter().enumerate() {
        if d <= 1e-14 {
            return Err(FeaError::UnconstrainedModel);
        }
        inv_diag[i] = 1.0 / d;
    }

    let mut x = vec![0.0; n];
    let mut r = b.to_vec(); // r_0 = b - A * x_0 = b
    let mut z = vec![0.0; n];
    for i in 0..n {
        z[i] = r[i] * inv_diag[i];
    }
    let mut p = z.clone();
    let mut rho = dot(&r, &z);

    let mut ap = vec![0.0; n];
    let max_iter = options.max_iterations.max(n * 2);
    let effective_tol = if n <= 10_000 {
        options.tolerance.min(1e-9)
    } else {
        options.tolerance
    };

    for iter in 0..max_iter {
        let norm_r = dot(&r, &r).sqrt();
        if norm_r / norm_b < effective_tol || norm_r < 1e-14 {
            return Ok(x);
        }

        matrix.spmv(&p, &mut ap);
        let p_ap = dot(&p, &ap);

        if p_ap <= 1e-300 || !p_ap.is_finite() {
            if norm_r / norm_b < options.tolerance * 5.0 || norm_r < 1e-12 {
                return Ok(x);
            }
            return Err(FeaError::SolverConvergenceFailed {
                iterations: iter,
                residual: norm_r,
                tolerance: options.tolerance,
            });
        }

        let alpha = rho / p_ap;
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
            z[i] = r[i] * inv_diag[i];
        }

        let rho_new = dot(&r, &z);
        if rho.abs() <= 1e-300 {
            break;
        }
        let beta = rho_new / rho;
        rho = rho_new;

        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
    }

    // Check final convergence
    let final_norm_r = dot(&r, &r).sqrt();
    if final_norm_r / norm_b < options.tolerance * 5.0 {
        Ok(x)
    } else {
        Err(FeaError::SolverConvergenceFailed {
            iterations: max_iter,
            residual: final_norm_r,
            tolerance: options.tolerance,
        })
    }
}

/// Assembles the global stiffness matrix and solves for global nodal displacements: $K \cdot u = F$ (`TET4`).
pub fn assemble_and_solve(
    mesh: &TetrahedralMesh,
    elements: &[Tet4Element],
    d_matrix: &Matrix6<f64>,
    boundary: &ResolvedBoundary,
    options: Option<SolverOptions>,
) -> Result<Vec<f64>, FeaError> {
    let total_nodes = mesh.node_count();
    let total_dofs = total_nodes * 3;

    if total_nodes == 0 || elements.is_empty() {
        return Err(FeaError::EmptyMesh);
    }

    // Map unconstrained (free) DOFs to reduced system indices
    let mut global_to_free = vec![None; total_dofs];
    let mut free_to_global = Vec::with_capacity(total_dofs);
    let mut num_free = 0;

    for (d, slot) in global_to_free.iter_mut().enumerate().take(total_dofs) {
        if !boundary.fixed_dofs.contains_key(&d) {
            *slot = Some(num_free);
            free_to_global.push(d);
            num_free += 1;
        }
    }

    if num_free == 0 {
        // Fully prescribed system
        let mut u_full = vec![0.0; total_dofs];
        for (&d, &val) in &boundary.fixed_dofs {
            u_full[d] = val;
        }
        return Ok(u_full);
    }

    // Assemble RHS force vector for free DOFs
    let mut f_free = vec![0.0; num_free];
    for n in 0..total_nodes {
        let force = boundary.nodal_forces[n];
        if let Some(f_idx) = global_to_free[3 * n] {
            f_free[f_idx] += force[0];
        }
        if let Some(f_idx) = global_to_free[3 * n + 1] {
            f_free[f_idx] += force[1];
        }
        if let Some(f_idx) = global_to_free[3 * n + 2] {
            f_free[f_idx] += force[2];
        }
    }

    // Assemble per-row entries for free DOFs (parallel element stiffness evaluation via rayon)
    let mut row_entries: Vec<Vec<(usize, f64)>> = vec![Vec::with_capacity(48); num_free];

    let elem_stiffnesses: Vec<([usize; 12], crate::element::Matrix12<f64>)> =
        if elements.len() >= PARALLEL_ELEMENT_THRESHOLD {
            elements
                .par_iter()
                .map(|elem| {
                    let ke = elem.stiffness_matrix(d_matrix);
                    let mut elem_dofs = [0usize; 12];
                    for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
                        elem_dofs[3 * local_i] = 3 * global_node;
                        elem_dofs[3 * local_i + 1] = 3 * global_node + 1;
                        elem_dofs[3 * local_i + 2] = 3 * global_node + 2;
                    }
                    (elem_dofs, ke)
                })
                .collect()
        } else {
            elements
                .iter()
                .map(|elem| {
                    let ke = elem.stiffness_matrix(d_matrix);
                    let mut elem_dofs = [0usize; 12];
                    for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
                        elem_dofs[3 * local_i] = 3 * global_node;
                        elem_dofs[3 * local_i + 1] = 3 * global_node + 1;
                        elem_dofs[3 * local_i + 2] = 3 * global_node + 2;
                    }
                    (elem_dofs, ke)
                })
                .collect()
        };

    for (elem_dofs, ke) in &elem_stiffnesses {
        for a in 0..12 {
            let da = elem_dofs[a];
            if let Some(free_a) = global_to_free[da] {
                let row_vec = &mut row_entries[free_a];
                for b in 0..12 {
                    let db = elem_dofs[b];
                    let val = ke[(a, b)];
                    if val.abs() <= 1e-25 {
                        continue;
                    }

                    if let Some(free_b) = global_to_free[db] {
                        row_vec.push((free_b, val));
                    } else if let Some(&u_prescribed) = boundary.fixed_dofs.get(&db) {
                        // Prescribed displacement adjustment to RHS
                        if u_prescribed.abs() > 1e-15 {
                            f_free[free_a] -= val * u_prescribed;
                        }
                    }
                }
            }
        }
    }

    let k_free = CsrMatrix::from_row_entries(num_free, num_free, row_entries);
    let solver_opts = options.unwrap_or_default();
    let u_free = solve_pcg(&k_free, &f_free, solver_opts)?;

    // Reconstruct full displacement vector
    let mut u_full = vec![0.0; total_dofs];
    for (free_idx, &global_dof) in free_to_global.iter().enumerate() {
        u_full[global_dof] = u_free[free_idx];
    }
    for (&global_dof, &val) in &boundary.fixed_dofs {
        u_full[global_dof] = val;
    }

    Ok(u_full)
}

/// Assembles the global stiffness matrix and solves for global nodal displacements: $K \cdot u = F$ (`TET10`).
pub fn assemble_and_solve_tet10(
    mesh: &TetrahedralMesh,
    elements: &[Tet10Element],
    d_matrix: &Matrix6<f64>,
    boundary: &ResolvedBoundary,
    options: Option<SolverOptions>,
) -> Result<Vec<f64>, FeaError> {
    let total_nodes = mesh.node_count();
    let total_dofs = total_nodes * 3;

    if total_nodes == 0 || elements.is_empty() {
        return Err(FeaError::EmptyMesh);
    }

    // Map unconstrained (free) DOFs to reduced system indices
    let mut global_to_free = vec![None; total_dofs];
    let mut free_to_global = Vec::with_capacity(total_dofs);
    let mut num_free = 0;

    for (d, slot) in global_to_free.iter_mut().enumerate().take(total_dofs) {
        if !boundary.fixed_dofs.contains_key(&d) {
            *slot = Some(num_free);
            free_to_global.push(d);
            num_free += 1;
        }
    }

    if num_free == 0 {
        let mut u_full = vec![0.0; total_dofs];
        for (&d, &val) in &boundary.fixed_dofs {
            u_full[d] = val;
        }
        return Ok(u_full);
    }

    // Assemble RHS force vector for free DOFs
    let mut f_free = vec![0.0; num_free];
    for n in 0..total_nodes {
        let force = boundary.nodal_forces[n];
        if let Some(f_idx) = global_to_free[3 * n] {
            f_free[f_idx] += force[0];
        }
        if let Some(f_idx) = global_to_free[3 * n + 1] {
            f_free[f_idx] += force[1];
        }
        if let Some(f_idx) = global_to_free[3 * n + 2] {
            f_free[f_idx] += force[2];
        }
    }

    // Assemble per-row entries for free DOFs (parallel element stiffness evaluation via rayon)
    let mut row_entries: Vec<Vec<(usize, f64)>> = vec![Vec::with_capacity(96); num_free];

    let elem_stiffnesses: Vec<([usize; 30], crate::element::Matrix30<f64>)> =
        if elements.len() >= PARALLEL_ELEMENT_THRESHOLD {
            elements
                .par_iter()
                .map(|elem| {
                    let ke = elem.stiffness_matrix(d_matrix);
                    let mut elem_dofs = [0usize; 30];
                    for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
                        elem_dofs[3 * local_i] = 3 * global_node;
                        elem_dofs[3 * local_i + 1] = 3 * global_node + 1;
                        elem_dofs[3 * local_i + 2] = 3 * global_node + 2;
                    }
                    (elem_dofs, ke)
                })
                .collect()
        } else {
            elements
                .iter()
                .map(|elem| {
                    let ke = elem.stiffness_matrix(d_matrix);
                    let mut elem_dofs = [0usize; 30];
                    for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
                        elem_dofs[3 * local_i] = 3 * global_node;
                        elem_dofs[3 * local_i + 1] = 3 * global_node + 1;
                        elem_dofs[3 * local_i + 2] = 3 * global_node + 2;
                    }
                    (elem_dofs, ke)
                })
                .collect()
        };

    for (elem_dofs, ke) in &elem_stiffnesses {
        for a in 0..30 {
            let da = elem_dofs[a];
            if let Some(free_a) = global_to_free[da] {
                let row_vec = &mut row_entries[free_a];
                for b in 0..30 {
                    let db = elem_dofs[b];
                    let val = ke[(a, b)];
                    if val.abs() <= 1e-25 {
                        continue;
                    }

                    if let Some(free_b) = global_to_free[db] {
                        row_vec.push((free_b, val));
                    } else if let Some(&u_prescribed) = boundary.fixed_dofs.get(&db) {
                        if u_prescribed.abs() > 1e-15 {
                            f_free[free_a] -= val * u_prescribed;
                        }
                    }
                }
            }
        }
    }

    let k_free = CsrMatrix::from_row_entries(num_free, num_free, row_entries);
    let solver_opts = options.unwrap_or_default();
    let u_free = solve_pcg(&k_free, &f_free, solver_opts)?;

    // Reconstruct full displacement vector
    let mut u_full = vec![0.0; total_dofs];
    for (free_idx, &global_dof) in free_to_global.iter().enumerate() {
        u_full[global_dof] = u_free[free_idx];
    }
    for (&global_dof, &val) in &boundary.fixed_dofs {
        u_full[global_dof] = val;
    }

    Ok(u_full)
}

/// Computes the nodal support reaction force vectors $\mathbf{R}_n = (K \cdot u)_n - \mathbf{f}_n$ (in Newtons)
/// at all constrained DOFs (`TET4`). For unconstrained (free) DOFs, the support reaction component is `0.0`.
pub fn compute_nodal_reactions(
    mesh: &TetrahedralMesh,
    elements: &[Tet4Element],
    d_matrix: &Matrix6<f64>,
    boundary: &ResolvedBoundary,
    u_full: &[f64],
) -> Vec<[f64; 3]> {
    let total_nodes = mesh.node_count();
    let mut ku = vec![[0.0f64; 3]; total_nodes];

    for elem in elements {
        let mut u_elem = crate::element::Vector12::zeros();
        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            u_elem[3 * local_i] = u_full[3 * global_node];
            u_elem[3 * local_i + 1] = u_full[3 * global_node + 1];
            u_elem[3 * local_i + 2] = u_full[3 * global_node + 2];
        }

        // K_e * u_e = V_e * B^T * (D * B * u_e) = V_e * B^T * sigma_e
        let stress = elem.compute_stress(d_matrix, &u_elem);
        let f_int = (elem.b_matrix.transpose() * stress) * elem.volume;

        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            ku[global_node][0] += f_int[3 * local_i];
            ku[global_node][1] += f_int[3 * local_i + 1];
            ku[global_node][2] += f_int[3 * local_i + 2];
        }
    }

    let mut reactions = vec![[0.0f64; 3]; total_nodes];
    for n in 0..total_nodes {
        for axis in 0..3 {
            let dof = 3 * n + axis;
            if boundary.fixed_dofs.contains_key(&dof) {
                let f_ext = boundary
                    .nodal_forces
                    .get(n)
                    .map(|f| f[axis])
                    .unwrap_or(0.0);
                reactions[n][axis] = ku[n][axis] - f_ext;
            }
        }
    }

    reactions
}

/// Computes the nodal support reaction force vectors $\mathbf{R}_n = (K \cdot u)_n - \mathbf{f}_n$ (in Newtons)
/// at all constrained DOFs (`TET10`). For unconstrained (free) DOFs, the support reaction component is `0.0`.
pub fn compute_nodal_reactions_tet10(
    mesh: &TetrahedralMesh,
    elements: &[Tet10Element],
    d_matrix: &Matrix6<f64>,
    boundary: &ResolvedBoundary,
    u_full: &[f64],
) -> Vec<[f64; 3]> {
    let total_nodes = mesh.node_count();
    let mut ku = vec![[0.0f64; 3]; total_nodes];

    for elem in elements {
        let mut u_elem = Vector30::zeros();
        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            u_elem[3 * local_i] = u_full[3 * global_node];
            u_elem[3 * local_i + 1] = u_full[3 * global_node + 1];
            u_elem[3 * local_i + 2] = u_full[3 * global_node + 2];
        }

        let f_int = elem.compute_internal_forces(d_matrix, &u_elem);

        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            ku[global_node][0] += f_int[3 * local_i];
            ku[global_node][1] += f_int[3 * local_i + 1];
            ku[global_node][2] += f_int[3 * local_i + 2];
        }
    }

    let mut reactions = vec![[0.0f64; 3]; total_nodes];
    for n in 0..total_nodes {
        for axis in 0..3 {
            let dof = 3 * n + axis;
            if boundary.fixed_dofs.contains_key(&dof) {
                let f_ext = boundary
                    .nodal_forces
                    .get(n)
                    .map(|f| f[axis])
                    .unwrap_or(0.0);
                reactions[n][axis] = ku[n][axis] - f_ext;
            }
        }
    }

    reactions
}

// =========================================================================
// Incremental Nonlinear Newton-Raphson Elastoplastic Co-Rotated Solver (P5-T3)
// =========================================================================

/// Configuration options for the incremental nonlinear elastoplastic Co-Rotated FEM solver.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NonlinearSolverOptions {
    /// Initial number of equal load increments per load stage (default: 5).
    pub initial_load_steps: usize,
    /// Maximum Newton-Raphson equilibrium iterations per load increment (default: 25).
    pub max_newton_iterations: usize,
    /// Relative out-of-balance force residual convergence tolerance (default: 1e-6).
    pub force_tolerance: f64,
    /// Relative displacement increment convergence tolerance (default: 1e-7).
    pub displacement_tolerance: f64,
    /// Maximum number of adaptive step bisections allowed per load stage (default: 10).
    pub max_bisections: usize,
    /// Minimum allowed load-factor sub-increment $\Delta\lambda_{\min}$ (default: 1e-4).
    pub min_step_size: f64,
    /// Enables backtracking line search on the residual norm (default: true).
    pub enable_line_search: bool,
    /// Maximum backtracking halvings per Newton iteration (default: 8).
    pub max_line_search_steps: usize,
    /// Enables Co-Rotated FEM polar decomposition $\mathbf{F} = \mathbf{R}\mathbf{U}$ (default: true).
    pub corotated: bool,
    /// Includes the symmetrized geometric rotational stiffness $\mathbf{K}_{\text{geo}}$ in the tangent (default: true).
    pub include_geometric_stiffness: bool,
    /// Underlying linear PCG solver options for each Newton tangent solve.
    pub linear_solver: SolverOptions,
}

impl Default for NonlinearSolverOptions {
    fn default() -> Self {
        Self {
            initial_load_steps: 5,
            max_newton_iterations: 25,
            force_tolerance: 1e-6,
            displacement_tolerance: 1e-7,
            max_bisections: 10,
            min_step_size: 1e-4,
            enable_line_search: true,
            max_line_search_steps: 8,
            corotated: true,
            include_geometric_stiffness: true,
            linear_solver: SolverOptions {
                tolerance: 1e-10,
                max_iterations: 8000,
            },
        }
    }
}

/// Converged nonlinear elastoplastic state persisted across load increments and multi-stage histories
/// (e.g. loading followed by elastic springback unloading).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NonlinearState {
    /// Global nodal displacement vector $[u_{x0}, u_{y0}, u_{z0}, \dots]$ in meters.
    pub displacements: Vec<f64>,
    /// Converged internal constitutive variables at every Gauss integration point
    /// (`num_elements` entries for `TET4`, `4 * num_elements` entries for `TET10`).
    pub gauss_states: Vec<GaussPointState>,
    /// Co-rotated proper orthogonal rotation matrices $\mathbf{R}^{(q)} \in SO(3)$ (row-major `[[f64; 3]; 3]`)
    /// at every Gauss integration point.
    pub gauss_rotations: Vec<[[f64; 3]; 3]>,
    /// External nodal forces `[Fx, Fy, Fz]` (N) applied at the end of the last converged stage
    /// (used as the baseline when ramping to a subsequent stage such as unloading).
    pub prev_nodal_forces: Vec<[f64; 3]>,
    /// Internal nodal forces $[f_{x0}, f_{y0}, f_{z0}, \dots]$ (N) at the end of the last converged
    /// increment, enabling smooth reaction-force release (springback) when a previously constrained
    /// Dirichlet DOF is freed in a subsequent stage.
    #[serde(default)]
    pub prev_internal_forces: Vec<f64>,
    /// Total recoverable elastic strain energy $\int w_e \, dV$ in Joules.
    pub total_elastic_energy_j: f64,
    /// Total irreversible plastic dissipation energy $\int w_p \, dV$ in Joules.
    pub total_plastic_energy_j: f64,
    /// Total number of converged load increments completed across all stages.
    pub converged_steps: usize,
    /// Total cumulative Newton-Raphson iterations across all converged increments.
    pub total_newton_iterations: usize,
    /// Total number of adaptive step bisections triggered across all stages.
    pub total_bisections: usize,
    /// Maximum final relative force residual ratio observed across all converged steps.
    pub max_residual_ratio: f64,
}

impl NonlinearState {
    /// Creates an undeformed, stress-free initial nonlinear state.
    pub fn initial(num_nodes: usize, num_gauss_points: usize) -> Self {
        let eye = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        Self {
            displacements: vec![0.0; num_nodes * 3],
            gauss_states: vec![GaussPointState::default(); num_gauss_points],
            gauss_rotations: vec![eye; num_gauss_points],
            prev_nodal_forces: vec![[0.0; 3]; num_nodes],
            prev_internal_forces: vec![0.0; num_nodes * 3],
            total_elastic_energy_j: 0.0,
            total_plastic_energy_j: 0.0,
            converged_steps: 0,
            total_newton_iterations: 0,
            total_bisections: 0,
            max_residual_ratio: 0.0,
        }
    }
}

/// Telemetry record captured at each converged incremental step of a nonlinear FEA run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StepHistoryPoint {
    /// Zero-based index of the load stage (e.g. `0` = loading, `1` = unloading).
    pub stage_index: usize,
    /// Optional human-readable label for the stage.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stage_label: Option<String>,
    /// Load factor $\lambda \in (0, 1]$ within the current stage.
    pub load_factor: f64,
    /// Cumulative progress across stages (`stage_index as f64 + load_factor`).
    pub cumulative_load_factor: f64,
    /// Number of Newton-Raphson iterations required to converge this increment.
    pub newton_iterations: usize,
    /// Final relative out-of-balance force residual ratio for this increment.
    pub residual_ratio: f64,
    /// Maximum nodal displacement magnitude in meters at this step.
    pub max_displacement_m: f64,
    /// Maximum Gauss-point von Mises stress in Pascals at this step.
    pub max_von_mises_pa: f64,
    /// Maximum Gauss-point accumulated equivalent plastic strain $\bar{\varepsilon}_p$ at this step.
    pub max_equivalent_plastic_strain: f64,
    /// Total support reaction force vector `[Rx, Ry, Rz]` in Newtons at this step.
    pub reaction_total_n: [f64; 3],
    /// Total applied external force vector `[Fx, Fy, Fz]` in Newtons at this step.
    pub applied_total_n: [f64; 3],
    /// Total elastic strain energy in Joules at this step.
    pub elastic_strain_energy_j: f64,
    /// Total accumulated plastic dissipation energy in Joules at this step.
    pub plastic_dissipation_energy_j: f64,
}

/// A single resolved load stage in a multi-stage nonlinear history (e.g., load -> unload -> reload).
#[derive(Debug, Clone)]
pub struct ResolvedLoadStage<'a> {
    /// Target boundary conditions (Dirichlet displacements and Neumann nodal forces) at $\lambda = 1$.
    pub boundary: &'a ResolvedBoundary,
    /// Optional override for the initial number of increments in this stage.
    pub load_steps: Option<usize>,
    /// Optional descriptive label for telemetry history.
    pub label: Option<String>,
}

/// Output of the incremental nonlinear Newton-Raphson solver.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NonlinearSolveResult {
    /// Final converged nonlinear state (displacements, Gauss-point plasticity state, rotations, energies).
    pub state: NonlinearState,
    /// Step-by-step load-displacement, stress, plasticity, and reaction history.
    pub history: Vec<StepHistoryPoint>,
}

#[inline]
fn matrix3_to_rows(m: &Matrix3<f64>) -> [[f64; 3]; 3] {
    [
        [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
        [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
        [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
    ]
}

/// Internal element assembly result `(f_int_full, dirichlet_rhs, K_free, gp_states, gp_rotations, W_e, W_p)`.
type TangentEval = (
    Vec<f64>,
    Vec<f64>,
    CsrMatrix,
    Vec<GaussPointState>,
    Vec<[[f64; 3]; 3]>,
    f64,
    f64,
);

/// Internal force-only evaluation result `(f_int_full, gp_states, gp_rotations, W_e, W_p)`.
type InternalEval = (
    Vec<f64>,
    Vec<GaussPointState>,
    Vec<[[f64; 3]; 3]>,
    f64,
    f64,
);

/// Internal abstraction allowing `TET4` and `TET10` meshes to share one Newton-Raphson +
/// backtracking line search + adaptive sub-incrementation implementation.
trait NonlinearElementModel {
    fn gp_count(&self) -> usize;

    #[allow(clippy::too_many_arguments)]
    fn assemble_tangent_and_internal(
        &self,
        num_nodes: usize,
        u_full: &[f64],
        prev_gp: &[GaussPointState],
        params: &J2MaterialParams,
        corotated: bool,
        include_geo: bool,
        global_to_free: &[Option<usize>],
        du_fixed: Option<&[f64]>,
        num_free: usize,
    ) -> TangentEval;

    fn evaluate_internal_only(
        &self,
        num_nodes: usize,
        u_full: &[f64],
        prev_gp: &[GaussPointState],
        params: &J2MaterialParams,
        corotated: bool,
    ) -> InternalEval;
}

impl NonlinearElementModel for &[Tet4Element] {
    fn gp_count(&self) -> usize {
        self.len()
    }

    fn assemble_tangent_and_internal(
        &self,
        num_nodes: usize,
        u_full: &[f64],
        prev_gp: &[GaussPointState],
        params: &J2MaterialParams,
        corotated: bool,
        include_geo: bool,
        global_to_free: &[Option<usize>],
        du_fixed: Option<&[f64]>,
        num_free: usize,
    ) -> TangentEval {
        let total_dofs = num_nodes * 3;
        let mut f_int_full = vec![0.0; total_dofs];
        let mut dirichlet_rhs = vec![0.0; num_free];
        let mut row_entries: Vec<Vec<(usize, f64)>> = vec![Vec::with_capacity(48); num_free];
        let mut new_gp = Vec::with_capacity(self.len());
        let mut new_rots = Vec::with_capacity(self.len());
        let mut total_we = 0.0;
        let mut total_wp = 0.0;

        let eval_elem = |(elem_idx, elem): (usize, &Tet4Element)| {
            let mut u_elem = Vector12::zeros();
            let mut elem_dofs = [0usize; 12];
            for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
                let d0 = 3 * global_node;
                elem_dofs[3 * local_i] = d0;
                elem_dofs[3 * local_i + 1] = d0 + 1;
                elem_dofs[3 * local_i + 2] = d0 + 2;
                u_elem[3 * local_i] = u_full[d0];
                u_elem[3 * local_i + 1] = u_full[d0 + 1];
                u_elem[3 * local_i + 2] = u_full[d0 + 2];
            }

            let (f_e, k_e, rm, r_mat) = elem.evaluate_nonlinear(
                &u_elem,
                &prev_gp[elem_idx],
                params,
                corotated,
                include_geo,
            );
            (elem_dofs, elem.volume, f_e, k_e, rm, matrix3_to_rows(&r_mat))
        };

        let elem_results: Vec<_> = if self.len() >= PARALLEL_ELEMENT_THRESHOLD {
            self.par_iter().enumerate().map(eval_elem).collect()
        } else {
            self.iter().enumerate().map(eval_elem).collect()
        };

        for (elem_dofs, vol, f_e, k_e, rm, rot_rows) in elem_results {
            total_we += rm.elastic_energy_density * vol;
            total_wp += rm.state.plastic_energy_density * vol;
            new_gp.push(rm.state);
            new_rots.push(rot_rows);

            for a in 0..12 {
                let da = elem_dofs[a];
                f_int_full[da] += f_e[a];

                if let Some(free_a) = global_to_free[da] {
                    let row_vec = &mut row_entries[free_a];
                    for b in 0..12 {
                        let val = k_e[(a, b)];
                        if val.abs() <= 1e-25 {
                            continue;
                        }
                        let db = elem_dofs[b];
                        if let Some(free_b) = global_to_free[db] {
                            row_vec.push((free_b, val));
                        } else if let Some(du_f) = du_fixed {
                            let d_val = du_f[db];
                            if d_val.abs() > 1e-18 {
                                dirichlet_rhs[free_a] -= val * d_val;
                            }
                        }
                    }
                }
            }
        }

        let k_free = CsrMatrix::from_row_entries(num_free, num_free, row_entries);
        (
            f_int_full,
            dirichlet_rhs,
            k_free,
            new_gp,
            new_rots,
            total_we,
            total_wp,
        )
    }

    fn evaluate_internal_only(
        &self,
        num_nodes: usize,
        u_full: &[f64],
        prev_gp: &[GaussPointState],
        params: &J2MaterialParams,
        corotated: bool,
    ) -> InternalEval {
        let total_dofs = num_nodes * 3;
        let mut f_int_full = vec![0.0; total_dofs];
        let mut new_gp = Vec::with_capacity(self.len());
        let mut new_rots = Vec::with_capacity(self.len());
        let mut total_we = 0.0;
        let mut total_wp = 0.0;

        let eval_elem = |(elem_idx, elem): (usize, &Tet4Element)| {
            let mut u_elem = Vector12::zeros();
            for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
                let d0 = 3 * global_node;
                u_elem[3 * local_i] = u_full[d0];
                u_elem[3 * local_i + 1] = u_full[d0 + 1];
                u_elem[3 * local_i + 2] = u_full[d0 + 2];
            }

            let (f_e, rm, r_mat) =
                elem.evaluate_nonlinear_forces_only(&u_elem, &prev_gp[elem_idx], params, corotated);
            (elem.node_indices, elem.volume, f_e, rm, matrix3_to_rows(&r_mat))
        };

        let elem_results: Vec<_> = if self.len() >= PARALLEL_ELEMENT_THRESHOLD {
            self.par_iter().enumerate().map(eval_elem).collect()
        } else {
            self.iter().enumerate().map(eval_elem).collect()
        };

        for (node_indices, vol, f_e, rm, rot_rows) in elem_results {
            total_we += rm.elastic_energy_density * vol;
            total_wp += rm.state.plastic_energy_density * vol;
            new_gp.push(rm.state);
            new_rots.push(rot_rows);

            for (local_i, &global_node) in node_indices.iter().enumerate() {
                let d0 = 3 * global_node;
                f_int_full[d0] += f_e[3 * local_i];
                f_int_full[d0 + 1] += f_e[3 * local_i + 1];
                f_int_full[d0 + 2] += f_e[3 * local_i + 2];
            }
        }

        (f_int_full, new_gp, new_rots, total_we, total_wp)
    }
}

impl NonlinearElementModel for &[Tet10Element] {
    fn gp_count(&self) -> usize {
        self.len() * 4
    }

    fn assemble_tangent_and_internal(
        &self,
        num_nodes: usize,
        u_full: &[f64],
        prev_gp: &[GaussPointState],
        params: &J2MaterialParams,
        corotated: bool,
        include_geo: bool,
        global_to_free: &[Option<usize>],
        du_fixed: Option<&[f64]>,
        num_free: usize,
    ) -> TangentEval {
        let total_dofs = num_nodes * 3;
        let mut f_int_full = vec![0.0; total_dofs];
        let mut dirichlet_rhs = vec![0.0; num_free];
        let mut row_entries: Vec<Vec<(usize, f64)>> = vec![Vec::with_capacity(96); num_free];
        let mut new_gp = Vec::with_capacity(self.len() * 4);
        let mut new_rots = Vec::with_capacity(self.len() * 4);
        let mut total_we = 0.0;
        let mut total_wp = 0.0;

        let eval_elem = |(elem_idx, elem): (usize, &Tet10Element)| {
            let mut u_elem = Vector30::zeros();
            let mut elem_dofs = [0usize; 30];
            for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
                let d0 = 3 * global_node;
                elem_dofs[3 * local_i] = d0;
                elem_dofs[3 * local_i + 1] = d0 + 1;
                elem_dofs[3 * local_i + 2] = d0 + 2;
                u_elem[3 * local_i] = u_full[d0];
                u_elem[3 * local_i + 1] = u_full[d0 + 1];
                u_elem[3 * local_i + 2] = u_full[d0 + 2];
            }

            let base_gp = elem_idx * 4;
            let prev4 = [
                prev_gp[base_gp],
                prev_gp[base_gp + 1],
                prev_gp[base_gp + 2],
                prev_gp[base_gp + 3],
            ];
            let (f_e, k_e, rms, rots) =
                elem.evaluate_nonlinear(&u_elem, &prev4, params, corotated, include_geo);
            let rot_rows = [
                matrix3_to_rows(&rots[0]),
                matrix3_to_rows(&rots[1]),
                matrix3_to_rows(&rots[2]),
                matrix3_to_rows(&rots[3]),
            ];
            (elem_dofs, elem.gauss_weights, f_e, k_e, rms, rot_rows)
        };

        let elem_results: Vec<_> = if self.len() >= PARALLEL_ELEMENT_THRESHOLD {
            self.par_iter().enumerate().map(eval_elem).collect()
        } else {
            self.iter().enumerate().map(eval_elem).collect()
        };

        for (elem_dofs, weights, f_e, k_e, rms, rot_rows) in elem_results {
            for q in 0..4 {
                let w = weights[q];
                total_we += rms[q].elastic_energy_density * w;
                total_wp += rms[q].state.plastic_energy_density * w;
                new_gp.push(rms[q].state);
                new_rots.push(rot_rows[q]);
            }

            for a in 0..30 {
                let da = elem_dofs[a];
                f_int_full[da] += f_e[a];

                if let Some(free_a) = global_to_free[da] {
                    let row_vec = &mut row_entries[free_a];
                    for b in 0..30 {
                        let val = k_e[(a, b)];
                        if val.abs() <= 1e-25 {
                            continue;
                        }
                        let db = elem_dofs[b];
                        if let Some(free_b) = global_to_free[db] {
                            row_vec.push((free_b, val));
                        } else if let Some(du_f) = du_fixed {
                            let d_val = du_f[db];
                            if d_val.abs() > 1e-18 {
                                dirichlet_rhs[free_a] -= val * d_val;
                            }
                        }
                    }
                }
            }
        }

        let k_free = CsrMatrix::from_row_entries(num_free, num_free, row_entries);
        (
            f_int_full,
            dirichlet_rhs,
            k_free,
            new_gp,
            new_rots,
            total_we,
            total_wp,
        )
    }

    fn evaluate_internal_only(
        &self,
        num_nodes: usize,
        u_full: &[f64],
        prev_gp: &[GaussPointState],
        params: &J2MaterialParams,
        corotated: bool,
    ) -> InternalEval {
        let total_dofs = num_nodes * 3;
        let mut f_int_full = vec![0.0; total_dofs];
        let mut new_gp = Vec::with_capacity(self.len() * 4);
        let mut new_rots = Vec::with_capacity(self.len() * 4);
        let mut total_we = 0.0;
        let mut total_wp = 0.0;

        let eval_elem = |(elem_idx, elem): (usize, &Tet10Element)| {
            let mut u_elem = Vector30::zeros();
            for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
                let d0 = 3 * global_node;
                u_elem[3 * local_i] = u_full[d0];
                u_elem[3 * local_i + 1] = u_full[d0 + 1];
                u_elem[3 * local_i + 2] = u_full[d0 + 2];
            }

            let base_gp = elem_idx * 4;
            let prev4 = [
                prev_gp[base_gp],
                prev_gp[base_gp + 1],
                prev_gp[base_gp + 2],
                prev_gp[base_gp + 3],
            ];
            let (f_e, rms, rots) =
                elem.evaluate_nonlinear_forces_only(&u_elem, &prev4, params, corotated);
            let rot_rows = [
                matrix3_to_rows(&rots[0]),
                matrix3_to_rows(&rots[1]),
                matrix3_to_rows(&rots[2]),
                matrix3_to_rows(&rots[3]),
            ];
            (elem.node_indices, elem.gauss_weights, f_e, rms, rot_rows)
        };

        let elem_results: Vec<_> = if self.len() >= PARALLEL_ELEMENT_THRESHOLD {
            self.par_iter().enumerate().map(eval_elem).collect()
        } else {
            self.iter().enumerate().map(eval_elem).collect()
        };

        for (node_indices, weights, f_e, rms, rot_rows) in elem_results {
            for q in 0..4 {
                let w = weights[q];
                total_we += rms[q].elastic_energy_density * w;
                total_wp += rms[q].state.plastic_energy_density * w;
                new_gp.push(rms[q].state);
                new_rots.push(rot_rows[q]);
            }

            for (local_i, &global_node) in node_indices.iter().enumerate() {
                let d0 = 3 * global_node;
                f_int_full[d0] += f_e[3 * local_i];
                f_int_full[d0 + 1] += f_e[3 * local_i + 1];
                f_int_full[d0 + 2] += f_e[3 * local_i + 2];
            }
        }

        (f_int_full, new_gp, new_rots, total_we, total_wp)
    }
}

/// Solves a single linear Newton tangent system using RCM sparse direct $LDL^T$ / `IC(0)-PCG` /
/// `SSOR-PCG` with automatic Levenberg–Marquardt diagonal damping fallback if the tangent becomes
/// nearly singular (e.g., fully formed ideal-plastic hinge with $H = 0$).
fn solve_tangent_with_regularization(
    k_free: &CsrMatrix,
    rhs: &[f64],
    lin_opts: SolverOptions,
) -> Result<Vec<f64>, FeaError> {
    solve_spd_newton_robust(k_free, rhs, LinearSolverKind::Auto, lin_opts)
}

/// Core incremental Newton-Raphson driver shared by `TET4` and `TET10`.
fn solve_nonlinear_history_generic<M: NonlinearElementModel>(
    mesh: &TetrahedralMesh,
    model: M,
    params: &J2MaterialParams,
    stages: &[ResolvedLoadStage<'_>],
    options: NonlinearSolverOptions,
    initial_state: Option<NonlinearState>,
) -> Result<NonlinearSolveResult, FeaError> {
    let total_nodes = mesh.node_count();
    let total_dofs = total_nodes * 3;
    let total_gps = model.gp_count();

    if total_nodes == 0 || total_gps == 0 {
        return Err(FeaError::EmptyMesh);
    }
    if stages.is_empty() {
        let state = initial_state.unwrap_or_else(|| NonlinearState::initial(total_nodes, total_gps));
        return Ok(NonlinearSolveResult {
            state,
            history: Vec::new(),
        });
    }

    let mut state = match initial_state {
        Some(s) => {
            if s.displacements.len() != total_dofs || s.gauss_states.len() != total_gps {
                return Err(FeaError::General(format!(
                    "Initial NonlinearState dimensions mismatch (dofs: {} vs {}, gps: {} vs {})",
                    s.displacements.len(),
                    total_dofs,
                    s.gauss_states.len(),
                    total_gps
                )));
            }
            s
        }
        None => NonlinearState::initial(total_nodes, total_gps),
    };

    let dims = mesh.bounding_box().dimensions();
    let l_char = dims[0].max(dims[1]).max(dims[2]).max(1e-6);
    let mut history = Vec::new();
    let mut global_peak_force_scale = 0.0_f64;

    for (stage_idx, stage) in stages.iter().enumerate() {
        let bc = stage.boundary;

        // Build free-DOF mapping for this stage
        let mut global_to_free = vec![None; total_dofs];
        let mut free_to_global = Vec::with_capacity(total_dofs);
        let mut num_free = 0;
        for (d, slot) in global_to_free.iter_mut().enumerate().take(total_dofs) {
            if !bc.fixed_dofs.contains_key(&d) {
                *slot = Some(num_free);
                free_to_global.push(d);
                num_free += 1;
            }
        }

        let u_stage_start = state.displacements.clone();
        // Build baseline force vector for this stage: on free DOFs, start from the previously
        // converged internal nodal force so releasing a Dirichlet constraint (springback unloading)
        // ramps the held reaction force smoothly down to the target external load instead of jumping.
        let mut f_stage_start = state.prev_nodal_forces.clone();
        if f_stage_start.len() < total_nodes {
            f_stage_start.resize(total_nodes, [0.0; 3]);
        }
        if state.prev_internal_forces.len() == total_dofs {
            for (n, fs) in f_stage_start.iter_mut().enumerate().take(total_nodes) {
                for (axis, fs_comp) in fs.iter_mut().enumerate() {
                    let d = 3 * n + axis;
                    if global_to_free[d].is_some() {
                        *fs_comp = state.prev_internal_forces[d];
                    }
                }
            }
        }
        let f_stage_target = &bc.nodal_forces;

        // Compute stage reference force scale (used for force residual normalization, including unloading to F = 0)
        let mut f_start_norm_sq = 0.0;
        let mut f_target_norm_sq = 0.0;
        let mut f_delta_norm_sq = 0.0;
        for n in 0..total_nodes {
            let fs = f_stage_start.get(n).copied().unwrap_or([0.0; 3]);
            let ft = f_stage_target.get(n).copied().unwrap_or([0.0; 3]);
            for axis in 0..3 {
                f_start_norm_sq += fs[axis] * fs[axis];
                f_target_norm_sq += ft[axis] * ft[axis];
                let df = ft[axis] - fs[axis];
                f_delta_norm_sq += df * df;
            }
        }
        let stage_ext_force_scale = f_start_norm_sq
            .sqrt()
            .max(f_target_norm_sq.sqrt())
            .max(f_delta_norm_sq.sqrt());
        global_peak_force_scale = global_peak_force_scale.max(stage_ext_force_scale);

        let stage_steps = stage
            .load_steps
            .unwrap_or(options.initial_load_steps)
            .max(1);
        let base_d_lambda = 1.0 / (stage_steps as f64);
        let mut lambda_curr = 0.0_f64;
        let mut d_lambda = base_d_lambda;
        let mut stage_bisections = 0usize;

        while lambda_curr < 1.0 - 1e-12 {
            let lambda_next = (lambda_curr + d_lambda).min(1.0);

            // External nodal forces at lambda_next
            let mut f_ext_full = vec![0.0; total_dofs];
            let mut applied_total = Vector3::<f64>::zeros();
            for n in 0..total_nodes {
                let fs = f_stage_start.get(n).copied().unwrap_or([0.0; 3]);
                let ft = f_stage_target.get(n).copied().unwrap_or([0.0; 3]);
                let fx = fs[0] + lambda_next * (ft[0] - fs[0]);
                let fy = fs[1] + lambda_next * (ft[1] - fs[1]);
                let fz = fs[2] + lambda_next * (ft[2] - fs[2]);
                f_ext_full[3 * n] = fx;
                f_ext_full[3 * n + 1] = fy;
                f_ext_full[3 * n + 2] = fz;
                applied_total += Vector3::new(fx, fy, fz);
            }

            // Target prescribed Dirichlet values at lambda_next and increment from last converged step
            let mut du_fixed = vec![0.0; total_dofs];
            let mut has_nonzero_dirichlet_inc = false;
            for (&d, &target_val) in &bc.fixed_dofs {
                let presc = u_stage_start[d] + lambda_next * (target_val - u_stage_start[d]);
                let inc = presc - state.displacements[d];
                du_fixed[d] = inc;
                if inc.abs() > 1e-18 {
                    has_nonzero_dirichlet_inc = true;
                }
            }

            // Fully prescribed system (no free DOFs): evaluate constitutive state directly
            if num_free == 0 {
                let mut u_trial = state.displacements.clone();
                for (&d, &target_val) in &bc.fixed_dofs {
                    u_trial[d] = u_stage_start[d] + lambda_next * (target_val - u_stage_start[d]);
                }
                let (f_int_full, new_gp, new_rots, total_we, total_wp) = model
                    .evaluate_internal_only(
                        total_nodes,
                        &u_trial,
                        &state.gauss_states,
                        params,
                        options.corotated,
                    );

                let mut rx = Vector3::<f64>::zeros();
                for n in 0..total_nodes {
                    for axis in 0..3 {
                        let d = 3 * n + axis;
                        if bc.fixed_dofs.contains_key(&d) {
                            rx[axis] += f_int_full[d] - f_ext_full[d];
                        }
                    }
                }
                let max_disp = compute_max_nodal_disp(&u_trial, total_nodes);
                let (max_vm, max_ep) = compute_gp_peaks(&new_gp);

                state.displacements = u_trial;
                state.gauss_states = new_gp;
                state.gauss_rotations = new_rots;
                state.prev_internal_forces = f_int_full;
                state.total_elastic_energy_j = total_we;
                state.total_plastic_energy_j = total_wp;
                state.converged_steps += 1;

                history.push(StepHistoryPoint {
                    stage_index: stage_idx,
                    stage_label: stage.label.clone(),
                    load_factor: lambda_next,
                    cumulative_load_factor: (stage_idx as f64) + lambda_next,
                    newton_iterations: 0,
                    residual_ratio: 0.0,
                    max_displacement_m: max_disp,
                    max_von_mises_pa: max_vm,
                    max_equivalent_plastic_strain: max_ep,
                    reaction_total_n: [rx.x, rx.y, rx.z],
                    applied_total_n: [applied_total.x, applied_total.y, applied_total.z],
                    elastic_strain_energy_j: total_we,
                    plastic_dissipation_energy_j: total_wp,
                });

                lambda_curr = lambda_next;
                continue;
            }

            // Free-DOF Newton-Raphson iteration from last converged state
            let mut u_trial = state.displacements.clone();
            let mut dirichlet_applied = !has_nonzero_dirichlet_inc;
            let mut converged = false;
            let mut iters_used = 0usize;
            let mut final_res_ratio = f64::INFINITY;
            let mut converged_payload: Option<InternalEval> = None;

            for iter in 0..options.max_newton_iterations {
                iters_used = iter + 1;

                // On iter 0 with prescribed displacement increments, use the tangent predictor
                // at the valid converged configuration u_curr so interior nodes move synchronously
                // with boundary nodes (prevents element inversion under large rotations/deflections).
                let active_du_fixed = if !dirichlet_applied {
                    Some(du_fixed.as_slice())
                } else {
                    None
                };

                let (f_int_full, dirichlet_rhs, k_free, trial_gp, trial_rots, trial_we, trial_wp) =
                    model.assemble_tangent_and_internal(
                        total_nodes,
                        &u_trial,
                        &state.gauss_states,
                        params,
                        options.corotated,
                        options.include_geometric_stiffness,
                        &global_to_free,
                        active_du_fixed,
                        num_free,
                    );

                let mut r_free = vec![0.0; num_free];
                for (f_idx, &g_dof) in free_to_global.iter().enumerate() {
                    r_free[f_idx] = f_ext_full[g_dof] - f_int_full[g_dof] + dirichlet_rhs[f_idx];
                }

                let r_norm = dot(&r_free, &r_free).sqrt();
                if !r_norm.is_finite() {
                    break;
                }

                let f_ext_free_norm = free_to_global
                    .iter()
                    .map(|&d| f_ext_full[d] * f_ext_full[d])
                    .sum::<f64>()
                    .sqrt();
                let f_int_full_norm = dot(&f_int_full, &f_int_full).sqrt();
                let u_norm = dot(&u_trial, &u_trial).sqrt();
                let f_ref = f_ext_free_norm
                    .max(f_int_full_norm)
                    .max(global_peak_force_scale)
                    .max(1e-12);
                let res_ratio = if f_ref > 1e-11 { r_norm / f_ref } else { r_norm };

                // Check convergence only after prescribed Dirichlet values have been applied
                if dirichlet_applied && (res_ratio <= options.force_tolerance || r_norm <= 1e-10) {
                    converged = true;
                    final_res_ratio = res_ratio;
                    converged_payload =
                        Some((f_int_full, trial_gp, trial_rots, trial_we, trial_wp));
                    break;
                }

                // Solve K_t * du_free = r_free (fallback to pure material tangent if K_geo causes indefiniteness)
                let du_free_res = solve_tangent_with_regularization(
                    &k_free,
                    &r_free,
                    options.linear_solver,
                )
                .or_else(|_| {
                    if options.include_geometric_stiffness && options.corotated {
                        let (_, dir_rhs_mat, k_mat, _, _, _, _) = model
                            .assemble_tangent_and_internal(
                                total_nodes,
                                &u_trial,
                                &state.gauss_states,
                                params,
                                options.corotated,
                                false,
                                &global_to_free,
                                active_du_fixed,
                                num_free,
                            );
                        let mut r_mat = vec![0.0; num_free];
                        for (f_idx, &g_dof) in free_to_global.iter().enumerate() {
                            r_mat[f_idx] =
                                f_ext_full[g_dof] - f_int_full[g_dof] + dir_rhs_mat[f_idx];
                        }
                        solve_tangent_with_regularization(&k_mat, &r_mat, options.linear_solver)
                    } else {
                        Err(FeaError::SolverConvergenceFailed {
                            iterations: iter,
                            residual: r_norm,
                            tolerance: options.force_tolerance,
                        })
                    }
                });

                let du_free = match du_free_res {
                    Ok(v) => v,
                    Err(_) => break,
                };

                let du_norm = dot(&du_free, &du_free).sqrt();
                if !du_norm.is_finite() {
                    break;
                }

                // Apply pending Dirichlet increment simultaneously with the tangent predictor step
                if !dirichlet_applied {
                    for (&d, &target_val) in &bc.fixed_dofs {
                        u_trial[d] =
                            u_stage_start[d] + lambda_next * (target_val - u_stage_start[d]);
                    }
                    for (f_idx, &g_dof) in free_to_global.iter().enumerate() {
                        u_trial[g_dof] += du_free[f_idx];
                    }
                    dirichlet_applied = true;

                    // Evaluate post-predictor state and check immediate convergence (e.g. purely elastic step)
                    let (f_int_post, gp_post, rots_post, we_post, wp_post) = model
                        .evaluate_internal_only(
                            total_nodes,
                            &u_trial,
                            &state.gauss_states,
                            params,
                            options.corotated,
                        );
                    let r_post_norm = free_to_global
                        .iter()
                        .map(|&d| {
                            let ri = f_ext_full[d] - f_int_post[d];
                            ri * ri
                        })
                        .sum::<f64>()
                        .sqrt();
                    let f_post_ref = f_ext_free_norm
                        .max(dot(&f_int_post, &f_int_post).sqrt())
                        .max(global_peak_force_scale)
                        .max(1e-12);
                    let res_post_ratio = if f_post_ref > 1e-11 {
                        r_post_norm / f_post_ref
                    } else {
                        r_post_norm
                    };
                    if r_post_norm.is_finite()
                        && (res_post_ratio <= options.force_tolerance || r_post_norm <= 1e-10)
                    {
                        converged = true;
                        final_res_ratio = res_post_ratio;
                        converged_payload =
                            Some((f_int_post, gp_post, rots_post, we_post, wp_post));
                        break;
                    }
                    continue;
                }

                // Backtracking line search on the out-of-balance force residual
                let mut best_alpha = 1.0_f64;
                let mut best_u = u_trial.clone();
                let mut best_r_norm = f64::INFINITY;
                let mut best_eval: Option<InternalEval> = None;

                let max_ls = if options.enable_line_search {
                    options.max_line_search_steps.max(1)
                } else {
                    1
                };

                let mut alpha = 1.0_f64;
                for ls_step in 0..max_ls {
                    let mut u_cand = u_trial.clone();
                    for (f_idx, &g_dof) in free_to_global.iter().enumerate() {
                        u_cand[g_dof] += alpha * du_free[f_idx];
                    }

                    let (f_int_c, gp_c, rots_c, we_c, wp_c) = model.evaluate_internal_only(
                        total_nodes,
                        &u_cand,
                        &state.gauss_states,
                        params,
                        options.corotated,
                    );
                    let r_c_norm = free_to_global
                        .iter()
                        .map(|&d| {
                            let ri = f_ext_full[d] - f_int_c[d];
                            ri * ri
                        })
                        .sum::<f64>()
                        .sqrt();

                    if r_c_norm.is_finite() && r_c_norm < best_r_norm {
                        best_r_norm = r_c_norm;
                        best_alpha = alpha;
                        best_u = u_cand;
                        best_eval = Some((f_int_c, gp_c, rots_c, we_c, wp_c));
                    }

                    // Accept full Newton step immediately if sufficient residual decrease or already within tolerance
                    if r_c_norm.is_finite()
                        && (r_c_norm <= (1.0 - 1e-4 * alpha) * r_norm
                            || r_c_norm <= options.force_tolerance * f_ref
                            || r_c_norm <= 1e-10
                            || (ls_step == 0 && iter == 0 && r_c_norm <= r_norm * 1.05))
                    {
                        break;
                    }
                    alpha *= 0.5;
                }

                let Some((f_int_next, gp_next, rots_next, we_next, wp_next)) = best_eval else {
                    break;
                };
                u_trial = best_u;

                let u_scale = u_norm.max(l_char * 1e-6);
                let du_ratio = (best_alpha * du_norm) / u_scale;
                let f_next_ref = f_ext_free_norm
                    .max(dot(&f_int_next, &f_int_next).sqrt())
                    .max(global_peak_force_scale)
                    .max(1e-12);
                let next_res_ratio = if f_next_ref > 1e-11 {
                    best_r_norm / f_next_ref
                } else {
                    best_r_norm
                };

                // Convergence check: force equilibrium AND displacement increment (for zero-force rotation cases,
                // require tight displacement convergence so interior nodes hit machine precision)
                let zero_force_regime =
                    f_ext_free_norm <= 1e-10 && global_peak_force_scale <= 1e-10;
                let is_converged = if zero_force_regime {
                    (best_r_norm <= 1e-10 && du_ratio <= 1e-12)
                        || (best_r_norm <= 1e-8 && du_ratio <= 1e-14)
                } else {
                    next_res_ratio <= options.force_tolerance
                        || (next_res_ratio <= options.force_tolerance * 10.0
                            && du_ratio <= options.displacement_tolerance)
                        || best_r_norm <= 1e-10
                };

                if is_converged {
                    converged = true;
                    final_res_ratio = next_res_ratio;
                    converged_payload = Some((f_int_next, gp_next, rots_next, we_next, wp_next));
                    break;
                }
            }

            if converged {
                let (f_int_conv, new_gp, new_rots, total_we, total_wp) =
                    converged_payload.expect("converged payload must be present");

                let mut rx = Vector3::<f64>::zeros();
                for n in 0..total_nodes {
                    for axis in 0..3 {
                        let d = 3 * n + axis;
                        if bc.fixed_dofs.contains_key(&d) {
                            rx[axis] += f_int_conv[d] - f_ext_full[d];
                        }
                    }
                }
                let max_disp = compute_max_nodal_disp(&u_trial, total_nodes);
                let (max_vm, max_ep) = compute_gp_peaks(&new_gp);

                // Commit converged state
                state.displacements = u_trial;
                state.gauss_states = new_gp;
                state.gauss_rotations = new_rots;
                state.total_elastic_energy_j = total_we;
                state.total_plastic_energy_j = total_wp;
                state.converged_steps += 1;
                state.total_newton_iterations += iters_used;
                if final_res_ratio.is_finite() && final_res_ratio > state.max_residual_ratio {
                    state.max_residual_ratio = final_res_ratio;
                }

                // Update force scale from internal reaction force as well (important for displacement-controlled stages)
                let f_int_conv_norm = dot(&f_int_conv, &f_int_conv).sqrt();
                if f_int_conv_norm > global_peak_force_scale {
                    global_peak_force_scale = f_int_conv_norm;
                }
                state.prev_internal_forces = f_int_conv;

                history.push(StepHistoryPoint {
                    stage_index: stage_idx,
                    stage_label: stage.label.clone(),
                    load_factor: lambda_next,
                    cumulative_load_factor: (stage_idx as f64) + lambda_next,
                    newton_iterations: iters_used,
                    residual_ratio: final_res_ratio,
                    max_displacement_m: max_disp,
                    max_von_mises_pa: max_vm,
                    max_equivalent_plastic_strain: max_ep,
                    reaction_total_n: [rx.x, rx.y, rx.z],
                    applied_total_n: [applied_total.x, applied_total.y, applied_total.z],
                    elastic_strain_energy_j: total_we,
                    plastic_dissipation_energy_j: total_wp,
                });

                lambda_curr = lambda_next;

                // Adaptive step-size recovery after easy convergence
                let easy_iter_threshold = (options.max_newton_iterations / 2).clamp(1, 4);
                if iters_used <= easy_iter_threshold && d_lambda < base_d_lambda {
                    d_lambda = (d_lambda * 1.5).min(base_d_lambda);
                }
            } else {
                // Adaptive sub-incrementation (bisection): halve step size and retry from last converged state
                state.total_bisections += 1;
                stage_bisections += 1;
                d_lambda *= 0.5;

                if stage_bisections > options.max_bisections || d_lambda < options.min_step_size {
                    return Err(FeaError::SolverConvergenceFailed {
                        iterations: state.total_newton_iterations,
                        residual: final_res_ratio,
                        tolerance: options.force_tolerance,
                    });
                }
            }
        }

        // Persist stage end external nodal forces as baseline for any subsequent stage (e.g. unloading)
        state.prev_nodal_forces = f_stage_target.clone();
    }

    Ok(NonlinearSolveResult { state, history })
}

#[inline]
fn compute_max_nodal_disp(u: &[f64], num_nodes: usize) -> f64 {
    let mut max_d = 0.0_f64;
    for i in 0..num_nodes {
        let ux = u[3 * i];
        let uy = u[3 * i + 1];
        let uz = u[3 * i + 2];
        let mag = (ux * ux + uy * uy + uz * uz).sqrt();
        if mag > max_d {
            max_d = mag;
        }
    }
    max_d
}

#[inline]
fn compute_gp_peaks(gps: &[GaussPointState]) -> (f64, f64) {
    let mut max_vm = 0.0_f64;
    let mut max_ep = 0.0_f64;
    for gp in gps {
        let vm = gp.von_mises_stress();
        if vm > max_vm {
            max_vm = vm;
        }
        if gp.equivalent_plastic_strain > max_ep {
            max_ep = gp.equivalent_plastic_strain;
        }
    }
    (max_vm, max_ep)
}

/// Solves a multi-stage nonlinear elastoplastic Co-Rotated FEM problem on linear `TET4` elements.
pub fn solve_nonlinear_history_tet4(
    mesh: &TetrahedralMesh,
    elements: &[Tet4Element],
    params: &J2MaterialParams,
    stages: &[ResolvedLoadStage<'_>],
    options: NonlinearSolverOptions,
    initial_state: Option<NonlinearState>,
) -> Result<NonlinearSolveResult, FeaError> {
    solve_nonlinear_history_generic(mesh, elements, params, stages, options, initial_state)
}

/// Solves a multi-stage nonlinear elastoplastic Co-Rotated FEM problem on quadratic `TET10` elements.
pub fn solve_nonlinear_history_tet10(
    mesh: &TetrahedralMesh,
    elements: &[Tet10Element],
    params: &J2MaterialParams,
    stages: &[ResolvedLoadStage<'_>],
    options: NonlinearSolverOptions,
    initial_state: Option<NonlinearState>,
) -> Result<NonlinearSolveResult, FeaError> {
    solve_nonlinear_history_generic(mesh, elements, params, stages, options, initial_state)
}

/// Solves a single-stage nonlinear elastoplastic Co-Rotated FEM problem on `TET4` elements.
pub fn solve_nonlinear_tet4(
    mesh: &TetrahedralMesh,
    elements: &[Tet4Element],
    params: &J2MaterialParams,
    boundary: &ResolvedBoundary,
    options: Option<NonlinearSolverOptions>,
) -> Result<NonlinearSolveResult, FeaError> {
    let opts = options.unwrap_or_default();
    let stage = ResolvedLoadStage {
        boundary,
        load_steps: None,
        label: Some("load".to_string()),
    };
    solve_nonlinear_history_tet4(mesh, elements, params, &[stage], opts, None)
}

/// Solves a single-stage nonlinear elastoplastic Co-Rotated FEM problem on `TET10` elements.
pub fn solve_nonlinear_tet10(
    mesh: &TetrahedralMesh,
    elements: &[Tet10Element],
    params: &J2MaterialParams,
    boundary: &ResolvedBoundary,
    options: Option<NonlinearSolverOptions>,
) -> Result<NonlinearSolveResult, FeaError> {
    let opts = options.unwrap_or_default();
    let stage = ResolvedLoadStage {
        boundary,
        load_steps: None,
        label: Some("load".to_string()),
    };
    solve_nonlinear_history_tet10(mesh, elements, params, &[stage], opts, None)
}

/// Computes exact nonlinear support reaction forces $\mathbf{R}_n = \mathbf{f}_{\text{int}, n} - \mathbf{f}_{\text{ext}, n}$
/// (in Newtons) at all constrained DOFs from the converged Co-Rotated $J_2$ stresses (`TET4`).
pub fn compute_nonlinear_nodal_reactions_tet4(
    mesh: &TetrahedralMesh,
    elements: &[Tet4Element],
    state: &NonlinearState,
    boundary: &ResolvedBoundary,
) -> Vec<[f64; 3]> {
    let total_nodes = mesh.node_count();
    let mut f_int = vec![[0.0f64; 3]; total_nodes];

    for (elem_idx, elem) in elements.iter().enumerate() {
        let gp = &state.gauss_states[elem_idx];
        let r_rows = state.gauss_rotations[elem_idx];
        let r_mat = Matrix3::new(
            r_rows[0][0], r_rows[0][1], r_rows[0][2],
            r_rows[1][0], r_rows[1][1], r_rows[1][2],
            r_rows[2][0], r_rows[2][1], r_rows[2][2],
        );
        let sigma_mat = crate::element::voigt_stress_to_matrix3(&gp.stress_vec());
        let pk1 = r_mat * sigma_mat;
        let grads = elem.shape_gradients();

        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            let g = grads[local_i];
            let fk = pk1 * Vector3::new(g[0], g[1], g[2]) * elem.volume;
            f_int[global_node][0] += fk.x;
            f_int[global_node][1] += fk.y;
            f_int[global_node][2] += fk.z;
        }
    }

    let mut reactions = vec![[0.0f64; 3]; total_nodes];
    for n in 0..total_nodes {
        for axis in 0..3 {
            let dof = 3 * n + axis;
            if boundary.fixed_dofs.contains_key(&dof) {
                let f_ext = boundary
                    .nodal_forces
                    .get(n)
                    .map(|f| f[axis])
                    .unwrap_or(0.0);
                reactions[n][axis] = f_int[n][axis] - f_ext;
            }
        }
    }
    reactions
}

/// Computes exact nonlinear support reaction forces $\mathbf{R}_n = \mathbf{f}_{\text{int}, n} - \mathbf{f}_{\text{ext}, n}$
/// (in Newtons) at all constrained DOFs from the converged Co-Rotated $J_2$ stresses (`TET10`).
pub fn compute_nonlinear_nodal_reactions_tet10(
    mesh: &TetrahedralMesh,
    elements: &[Tet10Element],
    state: &NonlinearState,
    boundary: &ResolvedBoundary,
) -> Vec<[f64; 3]> {
    let total_nodes = mesh.node_count();
    let mut f_int = vec![[0.0f64; 3]; total_nodes];

    for (elem_idx, elem) in elements.iter().enumerate() {
        let base_gp = elem_idx * 4;
        for q in 0..4 {
            let w = elem.gauss_weights[q];
            let gp = &state.gauss_states[base_gp + q];
            let r_rows = state.gauss_rotations[base_gp + q];
            let r_mat = Matrix3::new(
                r_rows[0][0], r_rows[0][1], r_rows[0][2],
                r_rows[1][0], r_rows[1][1], r_rows[1][2],
                r_rows[2][0], r_rows[2][1], r_rows[2][2],
            );
            let sigma_mat = crate::element::voigt_stress_to_matrix3(&gp.stress_vec());
            let pk1 = r_mat * sigma_mat;
            let grads = &elem.gauss_gradients[q];

            for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
                let g = grads[local_i];
                let fk = pk1 * Vector3::new(g[0], g[1], g[2]) * w;
                f_int[global_node][0] += fk.x;
                f_int[global_node][1] += fk.y;
                f_int[global_node][2] += fk.z;
            }
        }
    }

    let mut reactions = vec![[0.0f64; 3]; total_nodes];
    for n in 0..total_nodes {
        for axis in 0..3 {
            let dof = 3 * n + axis;
            if boundary.fixed_dofs.contains_key(&dof) {
                let f_ext = boundary
                    .nodal_forces
                    .get(n)
                    .map(|f| f[axis])
                    .unwrap_or(0.0);
                reactions[n][axis] = f_int[n][axis] - f_ext;
            }
        }
    }
    reactions
}
