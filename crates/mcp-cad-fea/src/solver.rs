//! Global stiffness matrix assembly and Preconditioned Conjugate Gradient (PCG) linear solver.

use nalgebra::Matrix6;
use serde::{Deserialize, Serialize};
use crate::boundary::ResolvedBoundary;
use crate::element::Tet4Element;
use crate::error::FeaError;
use crate::tetra_mesh::TetrahedralMesh;

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
    /// Constructs a CSR matrix from a list of (row, col, value) triplets,
    /// automatically sorting columns and accumulating duplicate entries.
    pub fn from_triplets(nrows: usize, ncols: usize, triplets: &[(usize, usize, f64)]) -> Self {
        let mut row_entries: Vec<Vec<(usize, f64)>> = vec![Vec::new(); nrows];
        for &(r, c, val) in triplets {
            if r < nrows && c < ncols && val.abs() > 1e-25 {
                row_entries[r].push((c, val));
            }
        }

        let mut row_ptr = Vec::with_capacity(nrows + 1);
        let mut col_indices = Vec::new();
        let mut values = Vec::new();
        row_ptr.push(0);

        for mut entries in row_entries {
            // Sort by column index
            entries.sort_unstable_by_key(|&(col, _)| col);

            for (col, val) in entries {
                let current_row_start = *row_ptr.last().unwrap();
                if col_indices.len() > current_row_start && *col_indices.last().unwrap() == col {
                    *values.last_mut().unwrap() += val;
                } else {
                    col_indices.push(col);
                    values.push(val);
                }
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

    /// Performs Sparse Matrix-Vector multiplication: $y = A \cdot x$.
    #[inline]
    pub fn spmv(&self, x: &[f64], y: &mut [f64]) {
        for (row, y_val) in y.iter_mut().enumerate().take(self.nrows) {
            let start = self.row_ptr[row];
            let end = self.row_ptr[row + 1];
            let mut sum = 0.0;
            for idx in start..end {
                sum += self.values[idx] * x[self.col_indices[idx]];
            }
            *y_val = sum;
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

    for iter in 0..max_iter {
        let norm_r = dot(&r, &r).sqrt();
        if norm_r / norm_b < options.tolerance || norm_r < 1e-14 {
            return Ok(x);
        }

        matrix.spmv(&p, &mut ap);
        let p_ap = dot(&p, &ap);

        if p_ap.abs() <= 1e-25 {
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

/// Assembles the global stiffness matrix and solves for global nodal displacements: $K \cdot u = F$.
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

    // Assemble triplets for free DOFs
    let mut triplets = Vec::with_capacity(elements.len() * 144);

    for elem in elements {
        let ke = elem.stiffness_matrix(d_matrix);
        let mut elem_dofs = [0usize; 12];
        for (local_i, &global_node) in elem.node_indices.iter().enumerate() {
            elem_dofs[3 * local_i] = 3 * global_node;
            elem_dofs[3 * local_i + 1] = 3 * global_node + 1;
            elem_dofs[3 * local_i + 2] = 3 * global_node + 2;
        }

        for a in 0..12 {
            let da = elem_dofs[a];
            if let Some(free_a) = global_to_free[da] {
                for b in 0..12 {
                    let db = elem_dofs[b];
                    let val = ke[(a, b)];

                    if let Some(free_b) = global_to_free[db] {
                        triplets.push((free_a, free_b, val));
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

    let k_free = CsrMatrix::from_triplets(num_free, num_free, &triplets);
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
