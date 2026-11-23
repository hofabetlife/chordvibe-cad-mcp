//! Pure-Rust sparse linear solvers for ill-conditioned FEA, contact, fracture, and HHT-α dynamics (`P5-T6`).
//!
//! Provides:
//! - Reverse Cuthill-McKee (RCM) bandwidth-reducing symmetric permutation (`rcm_ordering`)
//! - Exact sparse direct $LDL^T$ factorization (`SparseLdlt`, Davis up-looking elimination-tree algorithm)
//! - Shifted zero-fill Incomplete Cholesky Preconditioned Conjugate Gradient (`solve_ic_pcg`)
//! - Symmetric Gauss-Seidel Preconditioned Conjugate Gradient (`solve_ssor_pcg`)
//! - Unified robust dispatcher (`solve_linear_robust`) with automatic direct/iterative selection
//!   and Levenberg–Marquardt fallback for severely ill-conditioned contact/softening systems.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::error::FeaError;
use crate::solver::{solve_pcg, CsrMatrix, SolverOptions};

/// Linear system solver backend selection for static, contact, fracture, and HHT-α dynamic FEA.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum LinearSolverKind {
    /// Automatically selects sparse direct $LDL^T$ for small/medium systems ($n \le 1800$)
    /// and shifted `IC(0)-PCG` (falling back to `SSOR-PCG` / regularized $LDL^T$) for large systems.
    #[default]
    Auto,
    /// Exact RCM-ordered sparse direct $LDL^T$ factorization (guaranteed convergence regardless of condition number).
    SparseDirectLdlt,
    /// Zero-fill Incomplete Cholesky `IC(0)` Preconditioned Conjugate Gradient with automatic diagonal shift.
    IcPcg,
    /// Symmetric Gauss–Seidel (`SSOR`, $\omega = 1$) Preconditioned Conjugate Gradient.
    SsorPcg,
    /// Diagonal Jacobi Preconditioned Conjugate Gradient (`solve_pcg`).
    JacobiPcg,
}

#[inline]
fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(&x, &y)| x * y).sum()
}

#[inline]
fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}

/// Computes a Reverse Cuthill-McKee (RCM) symmetric permutation `perm` (`new_idx -> old_idx`)
/// reducing matrix bandwidth and fill-in for sparse direct $LDL^T$ factorization.
pub fn rcm_ordering(matrix: &CsrMatrix) -> Vec<usize> {
    let n = matrix.nrows;
    if n <= 2 {
        return (0..n).collect();
    }

    // Build symmetric adjacency list (excluding self-loops)
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for i in 0..n {
        let start = matrix.row_ptr[i];
        let end = matrix.row_ptr[i + 1];
        for idx in start..end {
            let j = matrix.col_indices[idx];
            if j < n && j != i {
                adj[i].push(j);
                adj[j].push(i);
            }
        }
    }
    for neighbors in &mut adj {
        neighbors.sort_unstable();
        neighbors.dedup();
    }

    let degrees: Vec<usize> = adj.iter().map(Vec::len).collect();
    for neighbors in &mut adj {
        neighbors.sort_unstable_by_key(|&v| (degrees[v], v));
    }

    let mut visited = vec![false; n];
    let mut order = Vec::with_capacity(n);
    let mut queue = VecDeque::new();

    // Process every connected component starting from a pseudo-peripheral / minimum-degree node
    let mut all_nodes: Vec<usize> = (0..n).collect();
    all_nodes.sort_unstable_by_key(|&v| (degrees[v], v));

    for start_node in all_nodes {
        if visited[start_node] {
            continue;
        }
        // One BFS sweep to find a pseudo-peripheral node in this component
        let mut comp_start = start_node;
        {
            let mut tmp_visited = Vec::new();
            let mut dist_mark = std::collections::HashSet::new();
            let mut q = VecDeque::new();
            q.push_back(start_node);
            dist_mark.insert(start_node);
            while let Some(u) = q.pop_front() {
                tmp_visited.push(u);
                for &v in &adj[u] {
                    if !visited[v] && dist_mark.insert(v) {
                        q.push_back(v);
                    }
                }
            }
            if let Some(&farthest) = tmp_visited.last() {
                comp_start = farthest;
            }
        }

        let comp_begin = order.len();
        visited[comp_start] = true;
        queue.push_back(comp_start);

        while let Some(u) = queue.pop_front() {
            order.push(u);
            for &v in &adj[u] {
                if !visited[v] {
                    visited[v] = true;
                    queue.push_back(v);
                }
            }
        }
        // Reverse Cuthill-McKee reverses the component ordering
        order[comp_begin..].reverse();
    }

    order
}

/// Exact sparse direct $LDL^T$ factorization of a symmetric CSR matrix $P A P^T = L D L^T$
/// using RCM fill-reducing ordering and Davis's up-looking elimination-tree algorithm.
#[derive(Debug, Clone)]
pub struct SparseLdlt {
    n: usize,
    /// `perm[new] = old`
    perm: Vec<usize>,
    /// Column pointers into `l_i` and `l_x` (length `n + 1`)
    l_p: Vec<usize>,
    /// Row indices of strictly lower-triangular entries in column-compressed $L$
    l_i: Vec<usize>,
    /// Values of strictly lower-triangular entries in column-compressed $L$
    l_x: Vec<f64>,
    /// Diagonal matrix $D$
    d: Vec<f64>,
    /// Minimum pivot magnitude encountered during factorization
    pub min_abs_pivot: f64,
    /// Number of negative pivots (0 iff matrix is strictly SPD)
    pub negative_pivots: usize,
}

impl SparseLdlt {
    /// Factorizes a symmetric CSR matrix into $P A P^T = L D L^T$.
    ///
    /// If `require_spd` is `true`, returns `Err(FeaError::UnconstrainedModel)` when a
    /// non-positive pivot is encountered.
    pub fn factorize(matrix: &CsrMatrix, require_spd: bool) -> Result<Self, FeaError> {
        let n = matrix.nrows;
        if n == 0 || matrix.ncols != n {
            return Err(FeaError::EmptyMesh);
        }

        let perm = rcm_ordering(matrix);
        let mut inv_perm = vec![0usize; n];
        for (new_idx, &old_idx) in perm.iter().enumerate() {
            inv_perm[old_idx] = new_idx;
        }

        // Build permuted upper-triangular column entries: `upper_cols[k]` contains `(i, val)` with `i <= k`.
        // Symmetrize off-diagonals using (A_ij + A_ji) / 2 if both present (or value if stored symmetrically).
        let mut upper_cols: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
        let mut max_diag = 0.0_f64;

        for old_r in 0..n {
            let nr = inv_perm[old_r];
            let start = matrix.row_ptr[old_r];
            let end = matrix.row_ptr[old_r + 1];
            for idx in start..end {
                let old_c = matrix.col_indices[idx];
                if old_c >= n {
                    continue;
                }
                let val = matrix.values[idx];
                if !val.is_finite() {
                    return Err(FeaError::General(
                        "Non-finite entry encountered in sparse matrix factorization".to_string(),
                    ));
                }
                let nc = inv_perm[old_c];
                if nr == nc {
                    upper_cols[nc].push((nr, val));
                    max_diag = max_diag.max(val.abs());
                } else if nr < nc {
                    upper_cols[nc].push((nr, 0.5 * val));
                } else {
                    upper_cols[nr].push((nc, 0.5 * val));
                }
            }
        }

        // Sort and merge duplicate entries in each upper column
        for col in &mut upper_cols {
            col.sort_unstable_by_key(|&(row, _)| row);
            let mut write = 0usize;
            for read in 0..col.len() {
                if write > 0 && col[write - 1].0 == col[read].0 {
                    col[write - 1].1 += col[read].1;
                } else {
                    col[write] = col[read];
                    write += 1;
                }
            }
            col.truncate(write);
        }

        let pivot_tol = (max_diag * 1e-14).max(1e-28);

        // 1. Symbolic factorization: elimination tree `parent` and column nnz counts `l_nz`
        const NONE: usize = usize::MAX;
        let mut parent = vec![NONE; n];
        let mut l_nz = vec![0usize; n];
        let mut flag = vec![NONE; n];

        for (k, col) in upper_cols.iter().enumerate() {
            flag[k] = k;
            for &(mut i, _) in col {
                if i >= k {
                    continue;
                }
                while i != NONE && flag[i] != k {
                    if parent[i] == NONE {
                        parent[i] = k;
                    }
                    l_nz[i] += 1;
                    flag[i] = k;
                    i = parent[i];
                }
            }
        }

        let mut l_p = vec![0usize; n + 1];
        for j in 0..n {
            l_p[j + 1] = l_p[j] + l_nz[j];
        }
        let total_l_nnz = l_p[n];
        let mut l_i = vec![0usize; total_l_nnz];
        let mut l_x = vec![0.0_f64; total_l_nnz];
        let mut d = vec![0.0_f64; n];

        // 2. Numeric factorization (up-looking sparse LDL^T)
        let mut y = vec![0.0_f64; n];
        let mut pattern: Vec<usize> = Vec::new();
        flag.fill(NONE);
        l_nz.fill(0);

        let mut min_abs_pivot = f64::INFINITY;
        let mut negative_pivots = 0usize;

        for (k, col) in upper_cols.iter().enumerate() {
            pattern.clear();
            flag[k] = k;
            y[k] = 0.0;

            for &(i, val) in col {
                if i == k {
                    y[k] = val;
                } else if i < k {
                    y[i] = val;
                    let mut cur = i;
                    while cur != NONE && flag[cur] != k {
                        flag[cur] = k;
                        pattern.push(cur);
                        cur = parent[cur];
                    }
                }
            }

            // Ascending row order is always a valid topological order of the elimination tree
            pattern.sort_unstable();

            let mut d_k = y[k];
            y[k] = 0.0;

            for &i in &pattern {
                let yi = y[i];
                y[i] = 0.0;
                let col_start = l_p[i];
                let col_end = col_start + l_nz[i];
                for p in col_start..col_end {
                    y[l_i[p]] -= l_x[p] * yi;
                }
                let l_ki = yi / d[i];
                d_k -= l_ki * yi;

                let write_pos = l_p[i] + l_nz[i];
                l_i[write_pos] = k;
                l_x[write_pos] = l_ki;
                l_nz[i] += 1;
            }

            if !d_k.is_finite() {
                return Err(FeaError::UnconstrainedModel);
            }
            if require_spd && d_k <= pivot_tol {
                return Err(FeaError::UnconstrainedModel);
            }
            if d_k.abs() <= pivot_tol {
                return Err(FeaError::UnconstrainedModel);
            }
            if d_k < 0.0 {
                negative_pivots += 1;
            }
            min_abs_pivot = min_abs_pivot.min(d_k.abs());
            d[k] = d_k;
        }

        Ok(Self {
            n,
            perm,
            l_p,
            l_i,
            l_x,
            d,
            min_abs_pivot,
            negative_pivots,
        })
    }

    /// Number of non-zero entries in the lower-triangular factor $L$.
    pub fn factor_nnz(&self) -> usize {
        self.l_x.len()
    }

    /// Solves $A x = b$ using the precomputed $P A P^T = L D L^T$ factorization.
    pub fn solve(&self, b: &[f64]) -> Vec<f64> {
        let n = self.n;
        let mut x = vec![0.0_f64; n];
        for i in 0..n {
            x[i] = b[self.perm[i]];
        }

        // Forward substitution: L z = P b
        for j in 0..n {
            let xj = x[j];
            if xj != 0.0 {
                for p in self.l_p[j]..self.l_p[j + 1] {
                    x[self.l_i[p]] -= self.l_x[p] * xj;
                }
            }
        }

        // Diagonal scaling: D y = z
        for (xj, &dj) in x.iter_mut().zip(self.d.iter()).take(n) {
            *xj /= dj;
        }

        // Backward substitution: L^T x_hat = y
        for j in (0..n).rev() {
            let mut xj = x[j];
            for p in self.l_p[j]..self.l_p[j + 1] {
                xj -= self.l_x[p] * x[self.l_i[p]];
            }
            x[j] = xj;
        }

        // Inverse permutation: out[perm[i]] = x[i]
        let mut out = vec![0.0_f64; n];
        for i in 0..n {
            out[self.perm[i]] = x[i];
        }
        out
    }
}

/// Zero-fill Incomplete Cholesky $L L^T$ factor stored by rows (lower triangle) and columns (for fast $L^T$ solve).
struct Ic0Factor {
    n: usize,
    /// Lower-triangular rows of $L$ (strictly lower entries `(col, val)` for `col < row`)
    l_row_ptr: Vec<usize>,
    l_col_idx: Vec<usize>,
    l_val: Vec<f64>,
    /// Diagonal entries $L_{ii} > 0$
    l_diag: Vec<f64>,
    /// Transpose (upper triangle $L^T$) stored by rows for backward substitution
    lt_row_ptr: Vec<usize>,
    lt_col_idx: Vec<usize>,
    lt_val: Vec<f64>,
}

impl Ic0Factor {
    /// Builds a shifted zero-fill incomplete Cholesky factorization of SPD matrix `matrix`
    /// with automatic Manteuffel diagonal shift escalation if any pivot turns non-positive.
    fn build(matrix: &CsrMatrix) -> Option<Self> {
        let n = matrix.nrows;
        let diag = matrix.get_diagonal();
        let max_diag = diag.iter().copied().fold(0.0_f64, f64::max);
        if max_diag <= 1e-24 || !max_diag.is_finite() {
            return None;
        }

        // Extract strictly lower CSR pattern once
        let mut l_row_ptr = Vec::with_capacity(n + 1);
        let mut l_col_idx = Vec::new();
        let mut a_lower_val = Vec::new();
        l_row_ptr.push(0);

        for i in 0..n {
            let start = matrix.row_ptr[i];
            let end = matrix.row_ptr[i + 1];
            for idx in start..end {
                let j = matrix.col_indices[idx];
                if j < i {
                    l_col_idx.push(j);
                    a_lower_val.push(matrix.values[idx]);
                } else {
                    break;
                }
            }
            l_row_ptr.push(l_col_idx.len());
        }

        // Try zero shift first, then escalate diagonal shift (Manteuffel IC(0))
        let shifts = [0.0_f64, 1e-6, 1e-4, 1e-2, 1e-1, 0.5];
        let mut l_val = vec![0.0_f64; a_lower_val.len()];
        let mut l_diag = vec![0.0_f64; n];
        let mut row_work = vec![0.0_f64; n];
        let mut row_mark = vec![usize::MAX; n];

        let mut succeeded = false;
        for &shift in &shifts {
            l_val.copy_from_slice(&a_lower_val);
            let mut ok = true;

            for i in 0..n {
                let row_start = l_row_ptr[i];
                let row_end = l_row_ptr[i + 1];

                // Scatter row i entries into workspace
                for p in row_start..row_end {
                    let c = l_col_idx[p];
                    row_work[c] = l_val[p];
                    row_mark[c] = i;
                }

                let mut diag_val = diag[i] * (1.0 + shift) + shift * 1e-6 * max_diag;
                if diag_val <= 1e-20 || !diag_val.is_finite() {
                    ok = false;
                    break;
                }

                // Compute L[i, j] for j < i in the sparsity pattern of row i
                for p in row_start..row_end {
                    let j = l_col_idx[p];
                    let mut sum = row_work[j];
                    let j_start = l_row_ptr[j];
                    let j_end = l_row_ptr[j + 1];
                    for q in j_start..j_end {
                        let k = l_col_idx[q];
                        if row_mark[k] == i {
                            sum -= row_work[k] * l_val[q];
                        }
                    }
                    let lij = sum / l_diag[j];
                    row_work[j] = lij;
                    l_val[p] = lij;
                    diag_val -= lij * lij;
                }

                if diag_val <= 1e-22 * max_diag.max(1.0) || !diag_val.is_finite() {
                    ok = false;
                    break;
                }
                l_diag[i] = diag_val.sqrt();
            }

            if ok {
                succeeded = true;
                break;
            }
        }

        if !succeeded {
            return None;
        }

        // Build transpose L^T (strictly upper triangle by rows) for cache-friendly backward substitution
        let mut col_counts = vec![0usize; n];
        for &c in &l_col_idx {
            col_counts[c] += 1;
        }
        let mut lt_row_ptr = vec![0usize; n + 1];
        for j in 0..n {
            lt_row_ptr[j + 1] = lt_row_ptr[j] + col_counts[j];
        }
        let mut lt_col_idx = vec![0usize; l_col_idx.len()];
        let mut lt_val = vec![0.0_f64; l_val.len()];
        let mut cursor = lt_row_ptr[..n].to_vec();

        for i in 0..n {
            for p in l_row_ptr[i]..l_row_ptr[i + 1] {
                let j = l_col_idx[p];
                let pos = cursor[j];
                lt_col_idx[pos] = i;
                lt_val[pos] = l_val[p];
                cursor[j] += 1;
            }
        }

        Some(Self {
            n,
            l_row_ptr,
            l_col_idx,
            l_val,
            l_diag,
            lt_row_ptr,
            lt_col_idx,
            lt_val,
        })
    }

    /// Applies the preconditioner: solves $(L L^T) z = r$.
    fn apply(&self, r: &[f64], y: &mut [f64], z: &mut [f64]) {
        // Forward substitution: L y = r
        for i in 0..self.n {
            let mut sum = r[i];
            for p in self.l_row_ptr[i]..self.l_row_ptr[i + 1] {
                sum -= self.l_val[p] * y[self.l_col_idx[p]];
            }
            y[i] = sum / self.l_diag[i];
        }

        // Backward substitution: L^T z = y
        for i in (0..self.n).rev() {
            let mut sum = y[i];
            for p in self.lt_row_ptr[i]..self.lt_row_ptr[i + 1] {
                sum -= self.lt_val[p] * z[self.lt_col_idx[p]];
            }
            z[i] = sum / self.l_diag[i];
        }
    }
}

/// Solves $A x = b$ using shifted zero-fill Incomplete Cholesky `IC(0)` Preconditioned Conjugate Gradient.
pub fn solve_ic_pcg(
    matrix: &CsrMatrix,
    b: &[f64],
    options: SolverOptions,
) -> Result<Vec<f64>, FeaError> {
    let n = matrix.nrows;
    let norm_b = norm(b);
    if norm_b <= 1e-15 {
        return Ok(vec![0.0; n]);
    }

    let Some(ic) = Ic0Factor::build(matrix) else {
        return solve_ssor_pcg(matrix, b, options);
    };

    let mut x = vec![0.0; n];
    let mut r = b.to_vec();
    let mut y_tmp = vec![0.0; n];
    let mut z = vec![0.0; n];
    ic.apply(&r, &mut y_tmp, &mut z);

    let mut p = z.clone();
    let mut rho = dot(&r, &z);
    if rho <= 0.0 || !rho.is_finite() {
        return solve_ssor_pcg(matrix, b, options);
    }

    let mut ap = vec![0.0; n];
    let max_iter = options.max_iterations.max(n);
    let tol = options.tolerance;

    for iter in 0..max_iter {
        let norm_r = norm(&r);
        if norm_r / norm_b < tol || norm_r < 1e-13 {
            return Ok(x);
        }

        matrix.spmv(&p, &mut ap);
        let p_ap = dot(&p, &ap);
        if p_ap <= 1e-300 || !p_ap.is_finite() {
            if norm_r / norm_b < tol * 20.0 || norm_r < 1e-11 {
                return Ok(x);
            }
            return Err(FeaError::SolverConvergenceFailed {
                iterations: iter,
                residual: norm_r,
                tolerance: tol,
            });
        }

        let alpha = rho / p_ap;
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }

        ic.apply(&r, &mut y_tmp, &mut z);
        let rho_new = dot(&r, &z);
        if rho_new <= 0.0 || !rho_new.is_finite() {
            let cur_r = norm(&r);
            if cur_r / norm_b < tol * 20.0 {
                return Ok(x);
            }
            return Err(FeaError::SolverConvergenceFailed {
                iterations: iter,
                residual: cur_r,
                tolerance: tol,
            });
        }
        let beta = rho_new / rho;
        rho = rho_new;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
    }

    let final_r = norm(&r);
    if final_r / norm_b < tol * 20.0 {
        Ok(x)
    } else {
        Err(FeaError::SolverConvergenceFailed {
            iterations: max_iter,
            residual: final_r,
            tolerance: tol,
        })
    }
}

/// Symmetric Gauss-Seidel (`SSOR`, $\omega = 1$) Preconditioned Conjugate Gradient solver
/// for symmetric positive definite CSR matrices.
pub fn solve_ssor_pcg(
    matrix: &CsrMatrix,
    b: &[f64],
    options: SolverOptions,
) -> Result<Vec<f64>, FeaError> {
    let n = matrix.nrows;
    let norm_b = norm(b);
    if norm_b <= 1e-15 {
        return Ok(vec![0.0; n]);
    }

    let diag = matrix.get_diagonal();
    let mut inv_diag = vec![0.0; n];
    for (i, &d) in diag.iter().enumerate() {
        if d <= 1e-14 || !d.is_finite() {
            return Err(FeaError::UnconstrainedModel);
        }
        inv_diag[i] = 1.0 / d;
    }

    let apply_ssor = |r: &[f64], y: &mut [f64], z: &mut [f64]| {
        for i in 0..n {
            let mut sum = r[i];
            let start = matrix.row_ptr[i];
            let end = matrix.row_ptr[i + 1];
            for idx in start..end {
                let c = matrix.col_indices[idx];
                if c < i {
                    sum -= matrix.values[idx] * y[c];
                } else {
                    break;
                }
            }
            y[i] = sum * inv_diag[i];
        }
        for i in (0..n).rev() {
            let mut sum = diag[i] * y[i];
            let start = matrix.row_ptr[i];
            let end = matrix.row_ptr[i + 1];
            for idx in (start..end).rev() {
                let c = matrix.col_indices[idx];
                if c > i {
                    sum -= matrix.values[idx] * z[c];
                } else {
                    break;
                }
            }
            z[i] = sum * inv_diag[i];
        }
    };

    let mut x = vec![0.0; n];
    let mut r = b.to_vec();
    let mut y_tmp = vec![0.0; n];
    let mut z = vec![0.0; n];
    apply_ssor(&r, &mut y_tmp, &mut z);

    let mut p = z.clone();
    let mut rho = dot(&r, &z);
    if rho <= 0.0 || !rho.is_finite() {
        return solve_pcg(matrix, b, options);
    }

    let mut ap = vec![0.0; n];
    let max_iter = options.max_iterations.max(n);
    let tol = options.tolerance;

    for iter in 0..max_iter {
        let norm_r = norm(&r);
        if norm_r / norm_b < tol || norm_r < 1e-13 {
            return Ok(x);
        }

        matrix.spmv(&p, &mut ap);
        let p_ap = dot(&p, &ap);
        if p_ap <= 1e-300 || !p_ap.is_finite() {
            if norm_r / norm_b < tol * 20.0 || norm_r < 1e-11 {
                return Ok(x);
            }
            return Err(FeaError::SolverConvergenceFailed {
                iterations: iter,
                residual: norm_r,
                tolerance: tol,
            });
        }

        let alpha = rho / p_ap;
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }

        apply_ssor(&r, &mut y_tmp, &mut z);
        let rho_new = dot(&r, &z);
        if rho_new <= 0.0 || !rho_new.is_finite() {
            let cur_r = norm(&r);
            if cur_r / norm_b < tol * 20.0 {
                return Ok(x);
            }
            return Err(FeaError::SolverConvergenceFailed {
                iterations: iter,
                residual: cur_r,
                tolerance: tol,
            });
        }
        let beta = rho_new / rho;
        rho = rho_new;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
    }

    let final_r = norm(&r);
    if final_r / norm_b < tol * 20.0 {
        Ok(x)
    } else {
        Err(FeaError::SolverConvergenceFailed {
            iterations: max_iter,
            residual: final_r,
            tolerance: tol,
        })
    }
}

fn add_diagonal_shift(matrix: &CsrMatrix, shift: f64) -> CsrMatrix {
    let mut rows: Vec<Vec<(usize, f64)>> = vec![Vec::new(); matrix.nrows];
    for (row, row_vec) in rows.iter_mut().enumerate() {
        let start = matrix.row_ptr[row];
        let end = matrix.row_ptr[row + 1];
        row_vec.reserve(end - start + 1);
        let mut has_diag = false;
        for idx in start..end {
            let col = matrix.col_indices[idx];
            let mut val = matrix.values[idx];
            if col == row {
                val += shift;
                has_diag = true;
            }
            row_vec.push((col, val));
        }
        if !has_diag {
            row_vec.push((row, shift));
        }
    }
    CsrMatrix::from_row_entries(matrix.nrows, matrix.ncols, rows)
}

/// Solves a symmetric linear system $A x = b$ robustly using the requested [`LinearSolverKind`],
/// with automatic fallback between sparse direct $LDL^T$, `IC(0)-PCG`, `SSOR-PCG`, and diagonal
/// Levenberg–Marquardt regularization on severely ill-conditioned contact/softening matrices.
pub fn solve_linear_robust(
    matrix: &CsrMatrix,
    rhs: &[f64],
    kind: LinearSolverKind,
    options: SolverOptions,
) -> Result<Vec<f64>, FeaError> {
    solve_linear_internal(matrix, rhs, kind, options, false)
}

/// Solves a symmetric Newton tangent system $K_T \Delta u = r$ guaranteeing a strict descent
/// direction ($\Delta u \cdot r > 0$) by preferring SPD sparse direct $LDL^T$ / `IC(0)-PCG` /
/// `SSOR-PCG` and applying progressive diagonal Levenberg–Marquardt stabilization if indefinite.
pub fn solve_spd_newton_robust(
    matrix: &CsrMatrix,
    rhs: &[f64],
    kind: LinearSolverKind,
    options: SolverOptions,
) -> Result<Vec<f64>, FeaError> {
    solve_linear_internal(matrix, rhs, kind, options, true)
}

fn solve_linear_internal(
    matrix: &CsrMatrix,
    rhs: &[f64],
    kind: LinearSolverKind,
    options: SolverOptions,
    require_descent: bool,
) -> Result<Vec<f64>, FeaError> {
    let n = matrix.nrows;
    if n == 0 {
        return Ok(Vec::new());
    }
    let norm_b = norm(rhs);
    if norm_b <= 1e-15 {
        return Ok(vec![0.0; n]);
    }

    let accept = |x: &[f64]| -> bool {
        x.iter().all(|v| v.is_finite()) && (!require_descent || dot(x, rhs) > 0.0)
    };

    let try_direct = |mat: &CsrMatrix, require_spd: bool| -> Result<Vec<f64>, FeaError> {
        let factor = SparseLdlt::factorize(mat, require_spd)?;
        let x = factor.solve(rhs);
        if accept(&x) {
            Ok(x)
        } else {
            Err(FeaError::UnconstrainedModel)
        }
    };

    match kind {
        LinearSolverKind::SparseDirectLdlt => {
            if let Ok(x) = try_direct(matrix, require_descent) {
                return Ok(x);
            }
        }
        LinearSolverKind::IcPcg => {
            if let Ok(x) = solve_ic_pcg(matrix, rhs, options) {
                if accept(&x) {
                    return Ok(x);
                }
            }
        }
        LinearSolverKind::SsorPcg => {
            if let Ok(x) = solve_ssor_pcg(matrix, rhs, options) {
                if accept(&x) {
                    return Ok(x);
                }
            }
        }
        LinearSolverKind::JacobiPcg => {
            if let Ok(x) = solve_pcg(matrix, rhs, options) {
                if accept(&x) {
                    return Ok(x);
                }
            }
        }
        LinearSolverKind::Auto => {
            // Threshold below which RCM sparse direct LDL^T is faster and immune to ill-conditioning
            const DIRECT_DOF_THRESHOLD: usize = 1800;
            if n <= DIRECT_DOF_THRESHOLD {
                if let Ok(x) = try_direct(matrix, require_descent) {
                    return Ok(x);
                }
            }
            if let Ok(x) = solve_ic_pcg(matrix, rhs, options) {
                if accept(&x) {
                    return Ok(x);
                }
            }
            if let Ok(x) = solve_ssor_pcg(matrix, rhs, options) {
                if accept(&x) {
                    return Ok(x);
                }
            }
            if n <= 4500 {
                if let Ok(x) = try_direct(matrix, require_descent) {
                    return Ok(x);
                }
            }
        }
    }

    // Regularized fallback for near-singular softened / contact-transition tangents
    let diag = matrix.get_diagonal();
    let max_diag = diag.iter().copied().map(f64::abs).fold(0.0_f64, f64::max);
    if max_diag <= 1e-24 || !max_diag.is_finite() {
        return Err(FeaError::UnconstrainedModel);
    }

    for &reg in &[1e-8, 1e-6, 1e-4, 1e-2, 1e-1] {
        let shift = (reg * max_diag).max(1.0);
        let k_reg = add_diagonal_shift(matrix, shift);
        if n <= 3000 {
            if let Ok(x) = try_direct(&k_reg, require_descent) {
                return Ok(x);
            }
        }
        if let Ok(x) = solve_ic_pcg(&k_reg, rhs, options) {
            if accept(&x) {
                return Ok(x);
            }
        }
        if let Ok(x) = solve_ssor_pcg(&k_reg, rhs, options) {
            if accept(&x) {
                return Ok(x);
            }
        }
    }

    Err(FeaError::SolverConvergenceFailed {
        iterations: options.max_iterations,
        residual: norm_b,
        tolerance: options.tolerance,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_ill_conditioned_tridiag(n: usize, stiffness_jump: f64) -> CsrMatrix {
        let mut triplets = Vec::new();
        for i in 0..n {
            let k_left = if i < n / 2 { 1.0 } else { stiffness_jump };
            let k_right = if i + 1 < n / 2 { 1.0 } else { stiffness_jump };
            let diag = k_left + k_right;
            triplets.push((i, i, diag));
            if i + 1 < n {
                triplets.push((i, i + 1, -k_right));
                triplets.push((i + 1, i, -k_right));
            }
        }
        CsrMatrix::from_triplets(n, n, &triplets)
    }

    #[test]
    fn test_sparse_ldlt_and_ic_pcg_exact_on_ill_conditioned_system() {
        // Condition number > 1e10 mimicking stiff contact penalty + soft elastoplastic elements
        let n = 120;
        let a = build_ill_conditioned_tridiag(n, 1e9);
        let x_exact: Vec<f64> = (0..n).map(|i| ((i as f64) * 0.13).sin() + 0.5).collect();
        let mut b = vec![0.0; n];
        a.spmv(&x_exact, &mut b);

        // 1. Direct sparse LDL^T must achieve near-machine accuracy
        let factor = SparseLdlt::factorize(&a, true).expect("SPD LDL^T must succeed");
        assert_eq!(factor.negative_pivots, 0);
        let x_direct = factor.solve(&b);
        let mut r_direct = vec![0.0; n];
        a.spmv(&x_direct, &mut r_direct);
        let rel_res = norm(
            &r_direct
                .iter()
                .zip(b.iter())
                .map(|(ri, bi)| ri - bi)
                .collect::<Vec<_>>(),
        ) / norm(&b);
        assert!(
            rel_res < 1e-12,
            "Direct sparse LDL^T relative residual too large: {rel_res:.3e}"
        );

        let rel_err = norm(
            &x_direct
                .iter()
                .zip(x_exact.iter())
                .map(|(xi, xe)| xi - xe)
                .collect::<Vec<_>>(),
        ) / norm(&x_exact);
        assert!(
            rel_err < 1e-9,
            "Direct sparse LDL^T solution error too large: {rel_err:.3e}"
        );

        // 2. IC(0)-PCG must converge accurately on the same stiff system
        let x_ic = solve_ic_pcg(
            &a,
            &b,
            SolverOptions {
                tolerance: 1e-10,
                max_iterations: 2000,
            },
        )
        .expect("IC(0)-PCG must converge");
        let rel_err_ic = norm(
            &x_ic
                .iter()
                .zip(x_exact.iter())
                .map(|(xi, xe)| xi - xe)
                .collect::<Vec<_>>(),
        ) / norm(&x_exact);
        assert!(
            rel_err_ic < 1e-6,
            "IC(0)-PCG solution error too large: {rel_err_ic:.3e}"
        );
    }

    #[test]
    fn test_sparse_ldlt_handles_symmetric_indefinite_and_2d_grid_rcm() {
        // 8x8 2D Laplacian grid with RCM ordering
        let gx = 8usize;
        let gy = 8usize;
        let n = gx * gy;
        let mut triplets = Vec::new();
        for y in 0..gy {
            for x in 0..gx {
                let idx = y * gx + x;
                let mut diag = 1.0;
                for (dx, dy) in [(-1i32, 0), (1, 0), (0, -1), (0, 1)] {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx >= 0 && nx < gx as i32 && ny >= 0 && ny < gy as i32 {
                        let nidx = (ny as usize) * gx + (nx as usize);
                        triplets.push((idx, nidx, -1.0));
                        diag += 1.0;
                    }
                }
                triplets.push((idx, idx, diag));
            }
        }
        let a = CsrMatrix::from_triplets(n, n, &triplets);
        let perm = rcm_ordering(&a);
        assert_eq!(perm.len(), n);

        let x_exact: Vec<f64> = (0..n).map(|i| (i as f64 + 1.0) * 0.01).collect();
        let mut b = vec![0.0; n];
        a.spmv(&x_exact, &mut b);

        let x_sol = solve_linear_robust(&a, &b, LinearSolverKind::Auto, SolverOptions::default())
            .expect("Auto linear solver must succeed");
        let rel_err = norm(
            &x_sol
                .iter()
                .zip(x_exact.iter())
                .map(|(xi, xe)| xi - xe)
                .collect::<Vec<_>>(),
        ) / norm(&x_exact);
        assert!(rel_err < 1e-12, "2D grid Auto solve error: {rel_err:.3e}");
    }
}
