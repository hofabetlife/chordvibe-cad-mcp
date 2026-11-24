//! Pure-Rust sparse linear solvers for ill-conditioned FEA, contact, fracture, and HHT-α dynamics (`P5-T6`).
//!
//! Provides:
//! - Reverse Cuthill-McKee (RCM) bandwidth-reducing symmetric permutation (`rcm_ordering`)
//! - Exact sparse direct $LDL^T$ factorization (`SparseLdlt`, Davis up-looking elimination-tree algorithm)
//! - Shifted zero-fill Incomplete Cholesky Preconditioned Conjugate Gradient (`solve_ic_pcg`)
//! - Symmetric Gauss-Seidel Preconditioned Conjugate Gradient (`solve_ssor_pcg`)
//! - Unified robust dispatcher (`solve_linear_robust`) with automatic direct/iterative selection
//!   and Levenberg–Marquardt fallback for severely ill-conditioned contact/softening systems.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};
use std::sync::Arc;

use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::error::FeaError;
use crate::solver::{solve_pcg, CsrMatrix, SolverOptions};

/// Linear system solver backend selection for static, contact, fracture, and HHT-α dynamic FEA.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum LinearSolverKind {
    /// Automatically selects AMD-ordered sparse direct $LDL^T$ when factor fill/flops are tractable
    /// ($n \le 18{,}000$ and estimated flops $\le 1.5 \times 10^8$, or $n \le 2{,}500$),
    /// and shifted `IC(0)-PCG` (falling back to `SSOR-PCG` / regularized $LDL^T$) for massive dense blocks.
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
    if a.len() >= 4096 {
        a.par_iter().zip(b.par_iter()).map(|(&x, &y)| x * y).sum()
    } else {
        a.iter().zip(b.iter()).map(|(&x, &y)| x * y).sum()
    }
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

/// Computes a supervariable-compressed Approximate Minimum Degree (`AMD`) permutation
/// (`perm[new_idx] = old_idx`) using a Quotient Graph, reducing $LDL^T$ factor fill-in
/// and flops by an order of magnitude on 3D solid and slender helical meshes.
pub fn amd_ordering(matrix: &CsrMatrix) -> Vec<usize> {
    let n = matrix.nrows;
    if n <= 3 {
        return (0..n).collect();
    }

    // 1. Build symmetric sorted closed neighborhoods N[i] = sorted({i} U adj(i)) for supervariable compression
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for i in 0..n {
        adj[i].push(i);
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
    for nbrs in &mut adj {
        nbrs.sort_unstable();
        nbrs.dedup();
    }

    // 2. Supervariable compression: group DOFs with identical closed neighborhoods (e.g. 3 DOFs per node)
    let mut hashes: Vec<(u64, usize, usize)> = Vec::with_capacity(n);
    for (i, nbrs) in adj.iter().enumerate() {
        let mut h = 0x9e37_79b9_7f4a_7c15_u64 ^ (nbrs.len() as u64);
        for &v in nbrs {
            let x = (v as u64).wrapping_add(0x9e37_79b9_7f4a_7c15);
            let z = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
            h = h.rotate_left(7) ^ z;
        }
        hashes.push((h, nbrs.len(), i));
    }
    hashes.sort_unstable();

    let mut members: Vec<Vec<usize>> = Vec::new();
    let mut node_to_sn = vec![0usize; n];
    let mut rep_of_sn: Vec<usize> = Vec::new();

    let mut idx = 0usize;
    while idx < n {
        let (h0, len0, _) = hashes[idx];
        let mut end = idx + 1;
        while end < n && hashes[end].0 == h0 && hashes[end].1 == len0 {
            end += 1;
        }
        // Verify exact equality within hash bucket
        let mut assigned = vec![false; end - idx];
        for a in 0..(end - idx) {
            if assigned[a] {
                continue;
            }
            let va = hashes[idx + a].2;
            let sn = members.len();
            let mut group = vec![va];
            assigned[a] = true;
            node_to_sn[va] = sn;
            for b in (a + 1)..(end - idx) {
                if !assigned[b] {
                    let vb = hashes[idx + b].2;
                    if adj[va] == adj[vb] {
                        assigned[b] = true;
                        group.push(vb);
                        node_to_sn[vb] = sn;
                    }
                }
            }
            group.sort_unstable();
            rep_of_sn.push(va);
            members.push(group);
        }
        idx = end;
    }

    let m = members.len();
    let weight: Vec<usize> = members.iter().map(Vec::len).collect();

    // Build compressed supernode adjacency (excluding self)
    let mut var_adj: Vec<Vec<usize>> = Vec::with_capacity(m);
    for (sn, &rep) in rep_of_sn.iter().enumerate() {
        let mut s_nbrs: Vec<usize> = adj[rep]
            .iter()
            .map(|&v| node_to_sn[v])
            .filter(|&s| s != sn)
            .collect();
        s_nbrs.sort_unstable();
        s_nbrs.dedup();
        var_adj.push(s_nbrs);
    }

    // 3. Quotient Graph Minimum Degree elimination on the m supernodes
    let mut alive = vec![true; m];
    let mut elen: Vec<Vec<usize>> = vec![Vec::new(); m];
    let mut elem_vars: Vec<Vec<usize>> = vec![Vec::new(); m];
    let mut deg: Vec<usize> = Vec::with_capacity(m);
    let mut heap: BinaryHeap<Reverse<(usize, usize)>> = BinaryHeap::with_capacity(m);
    for sn in 0..m {
        let d: usize = var_adj[sn].iter().map(|&v| weight[v]).sum();
        deg.push(d);
        heap.push(Reverse((d, sn)));
    }

    let mut perm = Vec::with_capacity(n);
    let mut mark_v = vec![0usize; m];
    let mut mark_e = vec![0usize; m];
    let mut mark_d = vec![0usize; m];
    let mut stamp = 0usize;
    let mut stamp_d = 0usize;
    let mut lp: Vec<usize> = Vec::new();

    while let Some(Reverse((d_heap, p))) = heap.pop() {
        if !alive[p] || d_heap != deg[p] {
            continue;
        }
        alive[p] = false;
        perm.extend_from_slice(&members[p]);

        stamp += 1;
        mark_v[p] = stamp;
        lp.clear();

        // Collect reachable alive supernodes L_p = (adj(p) \cap alive) U (U_{e in elen[p]} elem_vars[e] \cap alive)
        for &v in &var_adj[p] {
            if alive[v] && mark_v[v] != stamp {
                mark_v[v] = stamp;
                lp.push(v);
            }
        }
        for &e in &elen[p] {
            mark_e[e] = stamp;
            for &v in &elem_vars[e] {
                if alive[v] && mark_v[v] != stamp {
                    mark_v[v] = stamp;
                    lp.push(v);
                }
            }
        }
        var_adj[p].clear();
        elen[p].clear();

        if lp.is_empty() {
            continue;
        }

        // Reuse slot p as the newly created quotient element
        elem_vars[p].clear();
        elem_vars[p].extend_from_slice(&lp);

        for &u in &lp {
            var_adj[u].retain(|&v| alive[v] && mark_v[v] != stamp);
            elen[u].retain(|&e| mark_e[e] != stamp);
            elen[u].push(p);

            // Recompute exact external degree of u in quotient graph
            stamp_d += 1;
            mark_d[u] = stamp_d;
            let mut ext_deg = 0usize;
            for &v in &var_adj[u] {
                if alive[v] && mark_d[v] != stamp_d {
                    mark_d[v] = stamp_d;
                    ext_deg += weight[v];
                }
            }
            for &e in &elen[u] {
                for &v in &elem_vars[e] {
                    if alive[v] && mark_d[v] != stamp_d {
                        mark_d[v] = stamp_d;
                        ext_deg += weight[v];
                    }
                }
            }
            deg[u] = ext_deg;
            heap.push(Reverse((ext_deg, u)));
        }
    }

    perm
}

const SUPERNODE_MAX_W: usize = 48;

#[derive(Debug, Clone, Copy)]
struct SupernodeMeta {
    col_start: usize,
    w: usize,
    m: usize,
    rows_start: usize,
    flops_est: f64,
}

#[derive(Debug, Clone, Copy)]
struct SupernodeDep {
    src_lv_id: u32,
    tail_q_start: u32,
    is_contig: bool,
    dst_c0: u16,
    q0_or_hits_start: u32,
    k_hits: u16,
}

#[derive(Debug, Clone)]
struct SupernodalPlan {
    num_snodes: usize,
    /// Supernodes ordered by elimination-tree level (all `deps` have `src_lv_id < level_start[lv]`).
    meta: Vec<SupernodeMeta>,
    /// Flat off-diagonal row indices (`u32`) for each supernode in `lv_id` order.
    snode_rows: Vec<u32>,
    /// Maps original sequential supernode index `0..num_snodes` -> `lv_id`.
    orig_to_lv: Vec<usize>,
    /// Elimination-tree level slices into `0..num_snodes`.
    level_start: Vec<usize>,
    /// Total estimated flops per level (`level_start.len() - 1`).
    level_flops: Vec<f64>,
    /// CSR-style dependencies targeting each `lv_id`: `dep_ptr[lv_id]..dep_ptr[lv_id + 1]`.
    dep_ptr: Vec<usize>,
    deps: Vec<SupernodeDep>,
    /// Jagged hit pairs `(q_in_src_rows, col_in_dst)` for non-full dependencies.
    dep_hits: Vec<(u32, u16)>,
}

/// Precomputed symbolic structure (AMD ordering, elimination tree, supernode plan, and CSR scatter map)
/// enabling zero-overhead repeat parallel supernodal $LDL^T$ factorizations on identical sparsity patterns.
#[derive(Debug, Clone)]
pub struct SymbolicLdlt {
    /// Matrix dimension `n`
    pub n: usize,
    /// Number of CSR non-zeros this symbolic pattern was analyzed against
    nnz: usize,
    /// Exact CSR pattern snapshot for O(1)-cost collision-free cache matching
    row_ptr: Vec<usize>,
    col_indices: Vec<usize>,
    /// `perm[new] = old`
    perm: Arc<Vec<usize>>,
    /// Column pointers into `L` (length `n + 1`)
    l_p: Arc<Vec<usize>>,
    /// Total non-zeros in strictly lower-triangular `L`
    total_l_nnz: usize,
    /// Compressed lower-triangular column pointers for permuted $P A P^T$ (length `n + 1`, rows `>= k`)
    lower_col_ptr: Vec<usize>,
    /// Row indices `i >= k` in permuted lower column `k` (`u32`)
    lower_col_row: Vec<u32>,
    /// Direct scatter slot into `lower_vals` for each CSR entry `0..nnz` (`usize::MAX` if out of bounds)
    csr_slot: Vec<usize>,
    /// Symmetrizing weight (`1.0` for diagonal, `0.5` for off-diagonal) per CSR entry
    csr_scale: Vec<f64>,
    /// Supernodal level-scheduled execution plan
    plan: Arc<SupernodalPlan>,
}

impl SymbolicLdlt {
    /// Performs symbolic analysis (`AMD` ordering + elimination tree + supernode plan + scatter map) for `matrix`.
    pub fn analyze(matrix: &CsrMatrix) -> Result<Self, FeaError> {
        let n = matrix.nrows;
        if n == 0 || matrix.ncols != n {
            return Err(FeaError::EmptyMesh);
        }

        let perm_raw = amd_ordering(matrix);
        let mut inv_perm_raw = vec![0usize; n];
        for (new_idx, &old_idx) in perm_raw.iter().enumerate() {
            inv_perm_raw[old_idx] = new_idx;
        }

        // Elimination-tree DFS postordering of the AMD permutation:
        // preserves exact fill-in & flops while making every subtree contiguous so
        // fundamental & relaxed supernodes grow much wider for BLAS-3 panel throughput.
        const NONE: usize = usize::MAX;
        let perm = {
            let mut u_cols0: Vec<Vec<usize>> = vec![Vec::new(); n];
            for old_r in 0..n {
                let nr = inv_perm_raw[old_r];
                let start = matrix.row_ptr[old_r];
                let end = matrix.row_ptr[old_r + 1];
                for idx in start..end {
                    let old_c = matrix.col_indices[idx];
                    if old_c >= n {
                        continue;
                    }
                    let nc = inv_perm_raw[old_c];
                    if nr < nc {
                        u_cols0[nc].push(nr);
                    } else if nc < nr {
                        u_cols0[nr].push(nc);
                    }
                }
            }
            let mut parent0 = vec![NONE; n];
            let mut anc = vec![NONE; n];
            for k in 0..n {
                for &i0 in &u_cols0[k] {
                    let mut i = i0;
                    while i != NONE && i < k {
                        let next = anc[i];
                        anc[i] = k;
                        if next == NONE {
                            parent0[i] = k;
                            break;
                        }
                        i = next;
                    }
                }
            }
            // Build linked-list children per parent (in reverse index order so DFS visits ascending)
            let mut first_child = vec![NONE; n];
            let mut next_sib = vec![NONE; n];
            for i in (0..n).rev() {
                let p = parent0[i];
                if p != NONE {
                    next_sib[i] = first_child[p];
                    first_child[p] = i;
                }
            }
            // Iterative DFS postorder
            let mut post_perm = Vec::with_capacity(n);
            let mut stack: Vec<(usize, usize)> = Vec::with_capacity(256);
            for root in 0..n {
                if parent0[root] != NONE {
                    continue;
                }
                stack.push((root, first_child[root]));
                while let Some((node, child)) = stack.pop() {
                    if child != NONE {
                        stack.push((node, next_sib[child]));
                        stack.push((child, first_child[child]));
                    } else {
                        post_perm.push(perm_raw[node]);
                    }
                }
            }
            debug_assert_eq!(post_perm.len(), n);
            post_perm
        };

        let mut inv_perm = vec![0usize; n];
        for (new_idx, &old_idx) in perm.iter().enumerate() {
            inv_perm[old_idx] = new_idx;
        }

        let nnz = matrix.values.len();
        let mut upper_cols: Vec<Vec<usize>> = vec![Vec::new(); n];
        let mut lower_cols: Vec<Vec<usize>> = vec![Vec::new(); n];
        for old_r in 0..n {
            let nr = inv_perm[old_r];
            let start = matrix.row_ptr[old_r];
            let end = matrix.row_ptr[old_r + 1];
            for idx in start..end {
                let old_c = matrix.col_indices[idx];
                if old_c >= n {
                    continue;
                }
                let nc = inv_perm[old_c];
                if nr <= nc {
                    upper_cols[nc].push(nr);
                    lower_cols[nr].push(nc);
                } else {
                    upper_cols[nr].push(nc);
                    lower_cols[nc].push(nr);
                }
            }
        }

        let mut upper_col_ptr = Vec::with_capacity(n + 1);
        let mut upper_col_row: Vec<u32> = Vec::new();
        upper_col_ptr.push(0);
        for col in &mut upper_cols {
            col.sort_unstable();
            col.dedup();
            for &r in col.iter() {
                upper_col_row.push(r as u32);
            }
            upper_col_ptr.push(upper_col_row.len());
        }

        let mut lower_col_ptr = Vec::with_capacity(n + 1);
        let mut lower_col_row: Vec<u32> = Vec::new();
        lower_col_ptr.push(0);
        for col in &mut lower_cols {
            col.sort_unstable();
            col.dedup();
            for &r in col.iter() {
                lower_col_row.push(r as u32);
            }
            lower_col_ptr.push(lower_col_row.len());
        }

        // Map each original CSR entry to its exact slot in `lower_col_row`
        let mut csr_slot = vec![usize::MAX; nnz];
        let mut csr_scale = vec![0.0_f64; nnz];
        for old_r in 0..n {
            let nr = inv_perm[old_r];
            let start = matrix.row_ptr[old_r];
            let end = matrix.row_ptr[old_r + 1];
            for idx in start..end {
                let old_c = matrix.col_indices[idx];
                if old_c >= n {
                    continue;
                }
                let nc = inv_perm[old_c];
                let (row_l, col_l, scale) = if nr == nc {
                    (nr, nc, 1.0)
                } else if nr > nc {
                    (nr, nc, 0.5)
                } else {
                    (nc, nr, 0.5)
                };
                let c_start = lower_col_ptr[col_l];
                let c_end = lower_col_ptr[col_l + 1];
                let rel_pos = lower_col_row[c_start..c_end]
                    .binary_search(&(row_l as u32))
                    .expect("symbolic lower slot must exist");
                csr_slot[idx] = c_start + rel_pos;
                csr_scale[idx] = scale;
            }
        }

        // Elimination tree & column nnz counts (on postordered permutation)
        let mut parent = vec![NONE; n];
        let mut l_nz = vec![0usize; n];
        let mut flag = vec![NONE; n];

        for k in 0..n {
            flag[k] = k;
            for p in upper_col_ptr[k]..upper_col_ptr[k + 1] {
                let mut i = upper_col_row[p] as usize;
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

        // Column row-indices of strictly lower L
        let mut l_i = vec![0u32; total_l_nnz];
        let mut l_fill = vec![0usize; n];
        flag.fill(NONE);
        for k in 0..n {
            flag[k] = k;
            for p in upper_col_ptr[k]..upper_col_ptr[k + 1] {
                let mut cur = upper_col_row[p] as usize;
                if cur >= k {
                    continue;
                }
                while cur != NONE && flag[cur] != k {
                    flag[cur] = k;
                    let write_pos = l_p[cur] + l_fill[cur];
                    l_i[write_pos] = k as u32;
                    l_fill[cur] += 1;
                    cur = parent[cur];
                }
            }
        }

        // 1. Build fundamental supernodes (capped at SUPERNODE_MAX_W)
        let mut child_count = vec![0usize; n];
        for &p in &parent {
            if p != NONE {
                child_count[p] += 1;
            }
        }

        let mut fund_snodes: Vec<(usize, usize)> = Vec::new();
        let mut c = 0usize;
        while c < n {
            let start = c;
            while c + 1 < n
                && parent[c] == c + 1
                && child_count[c + 1] == 1
                && l_nz[c] == l_nz[c + 1] + 1
                && (c + 1 - start) < SUPERNODE_MAX_W
            {
                c += 1;
            }
            c += 1;
            fund_snodes.push((start, c - start));
        }

        // 2. Relaxed supernode amalgamation along postordered elimination tree:
        //    When a processed supernode `prev` immediately precedes its parent `cur` in postorder,
        //    every off-diagonal row of `prev` above `cur` is structurally already in `cur`'s row set.
        //    Merging them trades a small bounded fraction of explicit zeros for wide BLAS-3 panels.
        let mut merged_snodes: Vec<(usize, usize, usize)> = Vec::with_capacity(fund_snodes.len());
        for (mut cur_start, mut cur_w) in fund_snodes {
            let cur_last = cur_start + cur_w - 1;
            let cur_m = l_nz[cur_last];
            while let Some(&(prev_start, prev_w, prev_m)) = merged_snodes.last() {
                let prev_last = prev_start + prev_w - 1;
                let p_col = parent[prev_last];
                let is_child_of_cur = p_col != NONE && p_col >= cur_start && p_col <= cur_last;
                if !is_child_of_cur {
                    break;
                }
                let w_new = prev_w + cur_w;
                if w_new > SUPERNODE_MAX_W {
                    break;
                }
                let extra_rows = (cur_w + cur_m).saturating_sub(prev_m);
                let extra_zeros = prev_w * extra_rows;
                let new_panel_entries = w_new * (w_new + cur_m);
                let allow_merge = extra_zeros <= 96
                    || (extra_zeros * 100 <= new_panel_entries * 18)
                    || (prev_w <= 6 && cur_w <= 12 && extra_zeros * 100 <= new_panel_entries * 30);
                if !allow_merge {
                    break;
                }
                merged_snodes.pop();
                cur_start = prev_start;
                cur_w = w_new;
            }
            merged_snodes.push((cur_start, cur_w, cur_m));
        }

        let num_snodes = merged_snodes.len();
        let mut orig_col_start = Vec::with_capacity(num_snodes);
        let mut orig_w = Vec::with_capacity(num_snodes);
        let mut col_to_orig_snode = vec![0usize; n];
        for (s_id, &(start, w, _)) in merged_snodes.iter().enumerate() {
            orig_col_start.push(start);
            orig_w.push(w);
            for col in start..start + w {
                col_to_orig_snode[col] = s_id;
            }
        }

        let mut s_level = vec![0usize; num_snodes];
        for s in 0..num_snodes {
            let last_col = orig_col_start[s] + orig_w[s] - 1;
            let p_col = parent[last_col];
            if p_col != NONE {
                let p_snode = col_to_orig_snode[p_col];
                s_level[p_snode] = s_level[p_snode].max(s_level[s] + 1);
            }
        }
        let max_level = s_level.iter().copied().max().unwrap_or(0);

        let mut lv_to_orig: Vec<usize> = (0..num_snodes).collect();
        lv_to_orig.sort_unstable_by_key(|&s| (s_level[s], s));
        let mut orig_to_lv = vec![0usize; num_snodes];
        for (lv_id, &orig_s) in lv_to_orig.iter().enumerate() {
            orig_to_lv[orig_s] = lv_id;
        }

        let mut level_start = vec![0usize; max_level + 2];
        for &orig_s in &lv_to_orig {
            level_start[s_level[orig_s] + 1] += 1;
        }
        for lv in 0..=max_level {
            level_start[lv + 1] += level_start[lv];
        }

        let mut meta = Vec::with_capacity(num_snodes);
        let mut snode_rows = Vec::new();
        for &orig_s in &lv_to_orig {
            let col_start = orig_col_start[orig_s];
            let w = orig_w[orig_s];
            let last_col = col_start + w - 1;
            let r_start = snode_rows.len();
            let off_slice = &l_i[l_p[last_col]..l_p[last_col + 1]];
            snode_rows.extend_from_slice(off_slice);
            let m = off_slice.len();
            let h = (w + m) as f64;
            meta.push(SupernodeMeta {
                col_start,
                w,
                m,
                rows_start: r_start,
                flops_est: (w as f64) * h * h,
            });
        }

        let mut level_flops = vec![0.0_f64; max_level + 1];
        for (lv, lf) in level_flops.iter_mut().enumerate() {
            let mut sum = 0.0;
            for lv_id in level_start[lv]..level_start[lv + 1] {
                sum += meta[lv_id].flops_est;
            }
            *lf = sum;
        }

        // Precompute exact Left-Looking ancestor dependencies `(I -> J)` grouped by target `lv_id`
        let mut raw_deps: Vec<Vec<SupernodeDep>> = vec![Vec::new(); num_snodes];
        let mut dep_hits: Vec<(u32, u16)> = Vec::new();
        for (src_lv_id, src_meta) in meta.iter().enumerate() {
            let m_i = src_meta.m;
            if m_i == 0 {
                continue;
            }
            let rows_i = &snode_rows[src_meta.rows_start..src_meta.rows_start + m_i];
            let mut q = 0usize;
            while q < m_i {
                let dst_orig = col_to_orig_snode[rows_i[q] as usize];
                let dst_lv_id = orig_to_lv[dst_orig];
                let dst_col0 = meta[dst_lv_id].col_start;
                let dst_w = meta[dst_lv_id].w;
                let dst_end = dst_col0 + dst_w;
                let q0 = q;
                let first_c = rows_i[q0] as usize - dst_col0;
                q += 1;
                while q < m_i && (rows_i[q] as usize) < dst_end {
                    q += 1;
                }
                let k_hits = q - q0;
                let last_c = rows_i[q - 1] as usize - dst_col0;
                let is_contig = (last_c - first_c + 1) == k_hits;
                let q0_or_hits_start = if is_contig {
                    q0 as u32
                } else {
                    let h_start = dep_hits.len();
                    for qq in q0..q {
                        dep_hits.push((qq as u32, (rows_i[qq] as usize - dst_col0) as u16));
                    }
                    h_start as u32
                };
                raw_deps[dst_lv_id].push(SupernodeDep {
                    src_lv_id: src_lv_id as u32,
                    tail_q_start: q as u32,
                    is_contig,
                    dst_c0: first_c as u16,
                    q0_or_hits_start,
                    k_hits: k_hits as u16,
                });
            }
        }

        let mut dep_ptr = Vec::with_capacity(num_snodes + 1);
        let mut deps = Vec::new();
        dep_ptr.push(0);
        for mut list in raw_deps {
            list.sort_unstable_by_key(|d| d.src_lv_id);
            deps.extend_from_slice(&list);
            dep_ptr.push(deps.len());
        }

        Ok(Self {
            n,
            nnz,
            row_ptr: matrix.row_ptr.clone(),
            col_indices: matrix.col_indices.clone(),
            perm: Arc::new(perm),
            l_p: Arc::new(l_p),
            total_l_nnz,
            lower_col_ptr,
            lower_col_row,
            csr_slot,
            csr_scale,
            plan: Arc::new(SupernodalPlan {
                num_snodes,
                meta,
                snode_rows,
                orig_to_lv,
                level_start,
                level_flops,
                dep_ptr,
                deps,
                dep_hits,
            }),
        })
    }

    /// Returns `true` if `matrix` has the exact same dimension and CSR sparsity structure.
    #[inline]
    pub fn matches(&self, matrix: &CsrMatrix) -> bool {
        self.n == matrix.nrows
            && self.n == matrix.ncols
            && self.nnz == matrix.values.len()
            && self.row_ptr == matrix.row_ptr
            && self.col_indices == matrix.col_indices
    }

    /// Estimated floating-point operation count for numeric $LDL^T$ factorization.
    pub fn estimated_flops(&self) -> f64 {
        let mut flops = 0.0_f64;
        for j in 0..self.n {
            let nz = (self.l_p[j + 1] - self.l_p[j]) as f64;
            flops += (nz + 1.0) * (nz + 1.0);
        }
        flops
    }

    /// Number of non-zero entries in strictly lower-triangular $L$.
    pub fn factor_nnz(&self) -> usize {
        self.total_l_nnz
    }
}

/// Exact parallel supernodal sparse direct $LDL^T$ factorization of a symmetric CSR matrix
/// $P A P^T = L D L^T$ using AMD fill-reducing ordering and elimination-tree level parallelism.
#[derive(Debug, Clone)]
pub struct SparseLdlt {
    n: usize,
    total_l_nnz: usize,
    /// `perm[new] = old`
    perm: Arc<Vec<usize>>,
    /// Supernodal topology & solve layout
    plan: Arc<SupernodalPlan>,
    /// Per-supernode row-major dense panels (`(w + m) * w` entries of `L`, followed by `w` pivots `d_J`)
    panels: Vec<Vec<f64>>,
    /// Diagonal matrix $D$
    d: Vec<f64>,
    /// Minimum pivot magnitude encountered during factorization
    pub min_abs_pivot: f64,
    /// Number of negative pivots (0 iff matrix is strictly SPD)
    pub negative_pivots: usize,
}

/// Reusable workspace for numeric [`SparseLdlt`] factorization and triangular solves.
#[derive(Debug, Clone, Default)]
pub struct SparseLdltScratch {
    lower_vals: Vec<f64>,
    workers: Vec<PanelWorkerScratch>,
    pub solve_work: Vec<f64>,
}

/// Reusable workspace for [`solve_pcg_with_ldlt_precond`].
#[derive(Debug, Clone, Default)]
pub struct PcgLdltScratch {
    r: Vec<f64>,
    z: Vec<f64>,
    p: Vec<f64>,
    ap: Vec<f64>,
    ldlt_work: Vec<f64>,
}

#[derive(Debug, Clone)]
struct PanelWorkerScratch {
    row_map: Vec<u32>,
    t_tr: Vec<f64>,
    panel_acc: Vec<f64>,
    u_col: [f64; SUPERNODE_MAX_W],
    l_col: [f64; SUPERNODE_MAX_W],
    ld_scaled: [f64; SUPERNODE_MAX_W * SUPERNODE_MAX_W],
}

impl PanelWorkerScratch {
    fn new(n: usize) -> Self {
        Self {
            row_map: vec![u32::MAX; n],
            t_tr: Vec::with_capacity(SUPERNODE_MAX_W * SUPERNODE_MAX_W),
            panel_acc: Vec::new(),
            u_col: [0.0; SUPERNODE_MAX_W],
            l_col: [0.0; SUPERNODE_MAX_W],
            ld_scaled: [0.0; SUPERNODE_MAX_W * SUPERNODE_MAX_W],
        }
    }

    fn ensure_dim(&mut self, n: usize) {
        if self.row_map.len() < n {
            self.row_map.resize(n, u32::MAX);
        }
    }
}

/// 4-row x 8-col register-blocked outer product:
/// computes `R[i][j] = sum_{a=0..w_i} L[i][a] * T[a, c_off + j]` for `i in 0..4, j in 0..8`.
/// Every inner FMA lane is independent, auto-vectorizing cleanly to 8x 256-bit AVX2 `vfmadd231pd`.
#[inline(always)]
fn outer_4x8(
    l0: &[f64],
    l1: &[f64],
    l2: &[f64],
    l3: &[f64],
    t_tr: &[f64],
    k_pad: usize,
    c_off: usize,
) -> ([f64; 8], [f64; 8], [f64; 8], [f64; 8]) {
    let mut r0 = [0.0_f64; 8];
    let mut r1 = [0.0_f64; 8];
    let mut r2 = [0.0_f64; 8];
    let mut r3 = [0.0_f64; 8];
    let mut t_base = c_off;
    for (((&u0, &u1), &u2), &u3) in l0.iter().zip(l1).zip(l2).zip(l3) {
        let tr = &t_tr[t_base..t_base + 8];
        for j in 0..8 {
            let tv = tr[j];
            r0[j] += u0 * tv;
            r1[j] += u1 * tv;
            r2[j] += u2 * tv;
            r3[j] += u3 * tv;
        }
        t_base += k_pad;
    }
    (r0, r1, r2, r3)
}

#[inline(always)]
fn outer_2x8(
    l0: &[f64],
    l1: &[f64],
    t_tr: &[f64],
    k_pad: usize,
    c_off: usize,
) -> ([f64; 8], [f64; 8]) {
    let mut r0 = [0.0_f64; 8];
    let mut r1 = [0.0_f64; 8];
    let mut t_base = c_off;
    for (&u0, &u1) in l0.iter().zip(l1) {
        let tr = &t_tr[t_base..t_base + 8];
        for j in 0..8 {
            let tv = tr[j];
            r0[j] += u0 * tv;
            r1[j] += u1 * tv;
        }
        t_base += k_pad;
    }
    (r0, r1)
}

#[inline(always)]
fn outer_1x8(l0: &[f64], t_tr: &[f64], k_pad: usize, c_off: usize) -> [f64; 8] {
    let mut r0 = [0.0_f64; 8];
    let mut t_base = c_off;
    for &u0 in l0 {
        let tr = &t_tr[t_base..t_base + 8];
        for j in 0..8 {
            r0[j] += u0 * tr[j];
        }
        t_base += k_pad;
    }
    r0
}

#[inline(always)]
fn outer_4x4(
    l0: &[f64],
    l1: &[f64],
    l2: &[f64],
    l3: &[f64],
    t_tr: &[f64],
) -> ([f64; 4], [f64; 4], [f64; 4], [f64; 4]) {
    let mut r0 = [0.0_f64; 4];
    let mut r1 = [0.0_f64; 4];
    let mut r2 = [0.0_f64; 4];
    let mut r3 = [0.0_f64; 4];
    let mut t_base = 0usize;
    for (((&u0, &u1), &u2), &u3) in l0.iter().zip(l1).zip(l2).zip(l3) {
        let tr = &t_tr[t_base..t_base + 4];
        for j in 0..4 {
            let tv = tr[j];
            r0[j] += u0 * tv;
            r1[j] += u1 * tv;
            r2[j] += u2 * tv;
            r3[j] += u3 * tv;
        }
        t_base += 4;
    }
    (r0, r1, r2, r3)
}

#[inline(always)]
fn outer_1x4(l0: &[f64], t_tr: &[f64]) -> [f64; 4] {
    let mut r0 = [0.0_f64; 4];
    let mut t_base = 0usize;
    for &u0 in l0 {
        let tr = &t_tr[t_base..t_base + 4];
        for j in 0..4 {
            r0[j] += u0 * tr[j];
        }
        t_base += 4;
    }
    r0
}

#[inline(always)]
fn sub_slice_8(dst: &mut [f64], acc: &[f64; 8], len: usize) {
    if len == 8 {
        for j in 0..8 {
            dst[j] -= acc[j];
        }
    } else {
        for j in 0..len {
            dst[j] -= acc[j];
        }
    }
}

#[inline]
fn accumulate_deps_into_panel(
    deps_slice: &[SupernodeDep],
    w: usize,
    rows_j: &[u32],
    p_buf: &mut [f64],
    done_panels: &[Vec<f64>],
    plan: &SupernodalPlan,
    ws: &mut PanelWorkerScratch,
) {
    if deps_slice.is_empty() {
        return;
    }
    for (idx, &r) in rows_j.iter().enumerate() {
        ws.row_map[r as usize] = (w + idx) as u32;
    }

    for dep in deps_slice {
        let src_id = dep.src_lv_id as usize;
        let meta_i = plan.meta[src_id];
        let w_i = meta_i.w;
        let m_i = meta_i.m;
        let p_i = &done_panels[src_id];
        let d_i_off = (w_i + m_i) * w_i;
        let d_i = &p_i[d_i_off..d_i_off + w_i];
        let rows_i = &plan.snode_rows[meta_i.rows_start..meta_i.rows_start + m_i];
        let k_hits = dep.k_hits as usize;
        let k_pad = if k_hits <= 4 {
            4
        } else {
            k_hits.next_multiple_of(8)
        };
        let t_len = w_i * k_pad;
        ws.t_tr.resize(t_len, 0.0);
        if k_pad != k_hits {
            ws.t_tr[..t_len].fill(0.0);
        }

        if dep.is_contig {
            let q0 = dep.q0_or_hits_start as usize;
            let c0 = dep.dst_c0 as usize;
            // Pack T_tr[a * k_pad + c] = L_i[q0 + c, a] * d_i[a]
            for c in 0..k_hits {
                let src_row = &p_i[(w_i + q0 + c) * w_i..(w_i + q0 + c + 1) * w_i];
                for a in 0..w_i {
                    ws.t_tr[a * k_pad + c] = src_row[a] * d_i[a];
                }
            }
            let t_tr = &ws.t_tr[..t_len];
            let tail_start = dep.tail_q_start as usize;

            if k_pad == 4 {
                // Diagonal k_hits x k_hits lower block
                for c2 in 0..k_hits {
                    let l_row = &p_i[(w_i + q0 + c2) * w_i..(w_i + q0 + c2 + 1) * w_i];
                    let acc = outer_1x4(l_row, t_tr);
                    let p_row = &mut p_buf[(c0 + c2) * w + c0..(c0 + c2) * w + c0 + c2 + 1];
                    for j in 0..=c2 {
                        p_row[j] -= acc[j];
                    }
                }
                // Tail rows 4 at a time
                let mut q = tail_start;
                while q + 4 <= m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let dst1 = ws.row_map[rows_i[q + 1] as usize] as usize;
                    let dst2 = ws.row_map[rows_i[q + 2] as usize] as usize;
                    let dst3 = ws.row_map[rows_i[q + 3] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let l1 = &p_i[(w_i + q + 1) * w_i..(w_i + q + 2) * w_i];
                    let l2 = &p_i[(w_i + q + 2) * w_i..(w_i + q + 3) * w_i];
                    let l3 = &p_i[(w_i + q + 3) * w_i..(w_i + q + 4) * w_i];
                    let (a0, a1, a2, a3) = outer_4x4(l0, l1, l2, l3, t_tr);
                    for j in 0..k_hits {
                        p_buf[dst0 * w + c0 + j] -= a0[j];
                        p_buf[dst1 * w + c0 + j] -= a1[j];
                        p_buf[dst2 * w + c0 + j] -= a2[j];
                        p_buf[dst3 * w + c0 + j] -= a3[j];
                    }
                    q += 4;
                }
                while q < m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let a0 = outer_1x4(l0, t_tr);
                    let base0 = dst0 * w + c0;
                    for j in 0..k_hits {
                        p_buf[base0 + j] -= a0[j];
                    }
                    q += 1;
                }
            } else {
                // Diagonal k_hits x k_hits lower block (tiled in 8-col strips)
                for c2 in 0..k_hits {
                    let l_row = &p_i[(w_i + q0 + c2) * w_i..(w_i + q0 + c2 + 1) * w_i];
                    let n_cols = c2 + 1;
                    let p_row = &mut p_buf[(c0 + c2) * w + c0..(c0 + c2) * w + c0 + n_cols];
                    let mut c = 0usize;
                    while c < n_cols {
                        let acc = outer_1x8(l_row, t_tr, k_pad, c);
                        let len = (n_cols - c).min(8);
                        sub_slice_8(&mut p_row[c..c + len], &acc, len);
                        c += 8;
                    }
                }
                // Tail rows 4 at a time x 8-col AVX2 tiles
                let mut q = tail_start;
                while q + 4 <= m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let dst1 = ws.row_map[rows_i[q + 1] as usize] as usize;
                    let dst2 = ws.row_map[rows_i[q + 2] as usize] as usize;
                    let dst3 = ws.row_map[rows_i[q + 3] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let l1 = &p_i[(w_i + q + 1) * w_i..(w_i + q + 2) * w_i];
                    let l2 = &p_i[(w_i + q + 2) * w_i..(w_i + q + 3) * w_i];
                    let l3 = &p_i[(w_i + q + 3) * w_i..(w_i + q + 4) * w_i];
                    let mut c = 0usize;
                    while c < k_hits {
                        let (a0, a1, a2, a3) = outer_4x8(l0, l1, l2, l3, t_tr, k_pad, c);
                        let len = (k_hits - c).min(8);
                        let b0 = dst0 * w + c0 + c;
                        sub_slice_8(&mut p_buf[b0..b0 + len], &a0, len);
                        let b1 = dst1 * w + c0 + c;
                        sub_slice_8(&mut p_buf[b1..b1 + len], &a1, len);
                        let b2 = dst2 * w + c0 + c;
                        sub_slice_8(&mut p_buf[b2..b2 + len], &a2, len);
                        let b3 = dst3 * w + c0 + c;
                        sub_slice_8(&mut p_buf[b3..b3 + len], &a3, len);
                        c += 8;
                    }
                    q += 4;
                }
                if q + 2 <= m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let dst1 = ws.row_map[rows_i[q + 1] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let l1 = &p_i[(w_i + q + 1) * w_i..(w_i + q + 2) * w_i];
                    let mut c = 0usize;
                    while c < k_hits {
                        let (a0, a1) = outer_2x8(l0, l1, t_tr, k_pad, c);
                        let len = (k_hits - c).min(8);
                        let b0 = dst0 * w + c0 + c;
                        sub_slice_8(&mut p_buf[b0..b0 + len], &a0, len);
                        let b1 = dst1 * w + c0 + c;
                        sub_slice_8(&mut p_buf[b1..b1 + len], &a1, len);
                        c += 8;
                    }
                    q += 2;
                }
                if q < m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let mut c = 0usize;
                    while c < k_hits {
                        let a0 = outer_1x8(l0, t_tr, k_pad, c);
                        let len = (k_hits - c).min(8);
                        let b0 = dst0 * w + c0 + c;
                        sub_slice_8(&mut p_buf[b0..b0 + len], &a0, len);
                        c += 8;
                    }
                }
            }
        } else {
            let h_start = dep.q0_or_hits_start as usize;
            let hits = &plan.dep_hits[h_start..h_start + k_hits];
            for (h, &(q_h, _)) in hits.iter().enumerate() {
                let src_row = &p_i[(w_i + q_h as usize) * w_i..(w_i + q_h as usize + 1) * w_i];
                for a in 0..w_i {
                    ws.t_tr[a * k_pad + h] = src_row[a] * d_i[a];
                }
            }
            let t_tr = &ws.t_tr[..t_len];
            let tail_start = dep.tail_q_start as usize;

            if k_pad == 4 {
                for h2 in 0..k_hits {
                    let (q2, c2_u16) = hits[h2];
                    let c2 = c2_u16 as usize;
                    let l_row = &p_i[(w_i + q2 as usize) * w_i..(w_i + q2 as usize + 1) * w_i];
                    let acc = outer_1x4(l_row, t_tr);
                    let p_row = &mut p_buf[c2 * w..c2 * w + w];
                    for h in 0..=h2 {
                        p_row[hits[h].1 as usize] -= acc[h];
                    }
                }
                let mut q = tail_start;
                while q + 4 <= m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let dst1 = ws.row_map[rows_i[q + 1] as usize] as usize;
                    let dst2 = ws.row_map[rows_i[q + 2] as usize] as usize;
                    let dst3 = ws.row_map[rows_i[q + 3] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let l1 = &p_i[(w_i + q + 1) * w_i..(w_i + q + 2) * w_i];
                    let l2 = &p_i[(w_i + q + 2) * w_i..(w_i + q + 3) * w_i];
                    let l3 = &p_i[(w_i + q + 3) * w_i..(w_i + q + 4) * w_i];
                    let (a0, a1, a2, a3) = outer_4x4(l0, l1, l2, l3, t_tr);
                    for h in 0..k_hits {
                        let ch = hits[h].1 as usize;
                        p_buf[dst0 * w + ch] -= a0[h];
                        p_buf[dst1 * w + ch] -= a1[h];
                        p_buf[dst2 * w + ch] -= a2[h];
                        p_buf[dst3 * w + ch] -= a3[h];
                    }
                    q += 4;
                }
                while q < m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let a0 = outer_1x4(l0, t_tr);
                    let p_row = &mut p_buf[dst0 * w..dst0 * w + w];
                    for h in 0..k_hits {
                        p_row[hits[h].1 as usize] -= a0[h];
                    }
                    q += 1;
                }
            } else {
                for h2 in 0..k_hits {
                    let (q2, c2_u16) = hits[h2];
                    let c2 = c2_u16 as usize;
                    let l_row = &p_i[(w_i + q2 as usize) * w_i..(w_i + q2 as usize + 1) * w_i];
                    let n_cols = h2 + 1;
                    let p_row = &mut p_buf[c2 * w..c2 * w + w];
                    let mut h = 0usize;
                    while h < n_cols {
                        let acc = outer_1x8(l_row, t_tr, k_pad, h);
                        let len = (n_cols - h).min(8);
                        for j in 0..len {
                            p_row[hits[h + j].1 as usize] -= acc[j];
                        }
                        h += 8;
                    }
                }
                let mut q = tail_start;
                while q + 4 <= m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let dst1 = ws.row_map[rows_i[q + 1] as usize] as usize;
                    let dst2 = ws.row_map[rows_i[q + 2] as usize] as usize;
                    let dst3 = ws.row_map[rows_i[q + 3] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let l1 = &p_i[(w_i + q + 1) * w_i..(w_i + q + 2) * w_i];
                    let l2 = &p_i[(w_i + q + 2) * w_i..(w_i + q + 3) * w_i];
                    let l3 = &p_i[(w_i + q + 3) * w_i..(w_i + q + 4) * w_i];
                    let mut h = 0usize;
                    while h < k_hits {
                        let (a0, a1, a2, a3) = outer_4x8(l0, l1, l2, l3, t_tr, k_pad, h);
                        let len = (k_hits - h).min(8);
                        for j in 0..len {
                            let ch = hits[h + j].1 as usize;
                            p_buf[dst0 * w + ch] -= a0[j];
                            p_buf[dst1 * w + ch] -= a1[j];
                            p_buf[dst2 * w + ch] -= a2[j];
                            p_buf[dst3 * w + ch] -= a3[j];
                        }
                        h += 8;
                    }
                    q += 4;
                }
                if q + 2 <= m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let dst1 = ws.row_map[rows_i[q + 1] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let l1 = &p_i[(w_i + q + 1) * w_i..(w_i + q + 2) * w_i];
                    let mut h = 0usize;
                    while h < k_hits {
                        let (a0, a1) = outer_2x8(l0, l1, t_tr, k_pad, h);
                        let len = (k_hits - h).min(8);
                        for j in 0..len {
                            let ch = hits[h + j].1 as usize;
                            p_buf[dst0 * w + ch] -= a0[j];
                            p_buf[dst1 * w + ch] -= a1[j];
                        }
                        h += 8;
                    }
                    q += 2;
                }
                if q < m_i {
                    let dst0 = ws.row_map[rows_i[q] as usize] as usize;
                    let l0 = &p_i[(w_i + q) * w_i..(w_i + q + 1) * w_i];
                    let p_row = &mut p_buf[dst0 * w..dst0 * w + w];
                    let mut h = 0usize;
                    while h < k_hits {
                        let a0 = outer_1x8(l0, t_tr, k_pad, h);
                        let len = (k_hits - h).min(8);
                        for j in 0..len {
                            p_row[hits[h + j].1 as usize] -= a0[j];
                        }
                        h += 8;
                    }
                }
            }
        }
    }

    for &r in rows_j {
        ws.row_map[r as usize] = u32::MAX;
    }
}

#[inline]
fn load_panel_lower_entries(
    meta_j: SupernodeMeta,
    rows_j: &[u32],
    p_j: &mut [f64],
    lower_col_ptr: &[usize],
    lower_col_row: &[u32],
    lower_vals: &[f64],
    row_map: &mut [u32],
) {
    let j0 = meta_j.col_start;
    let w = meta_j.w;
    let j_end = j0 + w;
    for (idx, &r) in rows_j.iter().enumerate() {
        row_map[r as usize] = (w + idx) as u32;
    }
    for c in 0..w {
        let k = j0 + c;
        for p in lower_col_ptr[k]..lower_col_ptr[k + 1] {
            let r = lower_col_row[p] as usize;
            let dst_row = if r < j_end {
                r - j0
            } else {
                row_map[r] as usize
            };
            p_j[dst_row * w + c] = lower_vals[p];
        }
    }
    for &r in rows_j {
        row_map[r as usize] = u32::MAX;
    }
}

#[inline]
fn finish_panel_ldlt(
    p_j: &mut [f64],
    w: usize,
    h_total: usize,
    require_spd: bool,
    pivot_tol: f64,
    ws: &mut PanelWorkerScratch,
) -> Result<(), FeaError> {
    let d_off = h_total * w;
    let mut inv_d = [0.0_f64; SUPERNODE_MAX_W];
    for c in 0..w {
        let d_c = p_j[c * w + c];
        if !d_c.is_finite() || (require_spd && d_c <= pivot_tol) || d_c.abs() <= pivot_tol {
            return Err(FeaError::UnconstrainedModel);
        }
        let inv_dc = 1.0 / d_c;
        inv_d[c] = inv_dc;
        p_j[d_off + c] = d_c;
        p_j[c * w + c] = 1.0;

        for r in (c + 1)..w {
            let u_rc = p_j[r * w + c];
            let l_rc = u_rc * inv_dc;
            ws.u_col[r] = u_rc;
            ws.l_col[r] = l_rc;
            p_j[r * w + c] = l_rc;
            ws.ld_scaled[r * w + c] = u_rc;
        }
        for r in (c + 1)..w {
            let l_rc = ws.l_col[r];
            let p_row = &mut p_j[r * w..r * w + w];
            for (pv, &uv) in p_row[(c + 1)..=r].iter_mut().zip(ws.u_col[(c + 1)..=r].iter()) {
                *pv -= l_rc * uv;
            }
        }
    }

    let (l_body, _) = p_j.split_at_mut(d_off);
    let mut r = w;
    while r + 4 <= h_total {
        let base = r * w;
        let (r0, rest) = l_body[base..base + 4 * w].split_at_mut(w);
        let (r1, rest) = rest.split_at_mut(w);
        let (r2, r3) = rest.split_at_mut(w);
        for c in 0..w {
            let mut v = [r0[c], r1[c], r2[c], r3[c]];
            let ld_row = &ws.ld_scaled[c * w..c * w + c];
            for prev in 0..c {
                let ldp = ld_row[prev];
                v[0] -= r0[prev] * ldp;
                v[1] -= r1[prev] * ldp;
                v[2] -= r2[prev] * ldp;
                v[3] -= r3[prev] * ldp;
            }
            let idc = inv_d[c];
            r0[c] = v[0] * idc;
            r1[c] = v[1] * idc;
            r2[c] = v[2] * idc;
            r3[c] = v[3] * idc;
        }
        r += 4;
    }
    while r < h_total {
        let p_row = &mut l_body[r * w..r * w + w];
        for c in 0..w {
            let mut v = p_row[c];
            let ld_row = &ws.ld_scaled[c * w..c * w + c];
            for prev in 0..c {
                v -= p_row[prev] * ld_row[prev];
            }
            p_row[c] = v * inv_d[c];
        }
        r += 1;
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn factorize_supernode_panel(
    lv_id: usize,
    p_j: &mut Vec<f64>,
    done_panels: &[Vec<f64>],
    plan: &SupernodalPlan,
    lower_col_ptr: &[usize],
    lower_col_row: &[u32],
    lower_vals: &[f64],
    require_spd: bool,
    pivot_tol: f64,
    ws: &mut PanelWorkerScratch,
) -> Result<(), FeaError> {
    let meta_j = plan.meta[lv_id];
    let w = meta_j.w;
    let m = meta_j.m;
    let h_total = w + m;
    let needed = h_total * w + w;
    p_j.resize(needed, 0.0);
    p_j.fill(0.0);

    let rows_j = &plan.snode_rows[meta_j.rows_start..meta_j.rows_start + m];
    load_panel_lower_entries(
        meta_j,
        rows_j,
        p_j,
        lower_col_ptr,
        lower_col_row,
        lower_vals,
        &mut ws.row_map,
    );

    let dep_range = plan.dep_ptr[lv_id]..plan.dep_ptr[lv_id + 1];
    accumulate_deps_into_panel(
        &plan.deps[dep_range],
        w,
        rows_j,
        p_j,
        done_panels,
        plan,
        ws,
    );

    finish_panel_ldlt(p_j, w, h_total, require_spd, pivot_tol, ws)
}

impl SparseLdlt {
    /// Factorizes a symmetric CSR matrix into $P A P^T = L D L^T$.
    ///
    /// If `require_spd` is `true`, returns `Err(FeaError::UnconstrainedModel)` when a
    /// non-positive pivot is encountered.
    pub fn factorize(matrix: &CsrMatrix, require_spd: bool) -> Result<Self, FeaError> {
        let sym = SymbolicLdlt::analyze(matrix)?;
        Self::factorize_with_symbolic(&sym, matrix, require_spd)
    }

    /// Factorizes `matrix` using a precomputed [`SymbolicLdlt`] pattern (zero ordering, tree walk, or sorting).
    pub fn factorize_with_symbolic(
        sym: &SymbolicLdlt,
        matrix: &CsrMatrix,
        require_spd: bool,
    ) -> Result<Self, FeaError> {
        let mut scratch = SparseLdltScratch::default();
        Self::factorize_with_symbolic_into(sym, matrix, require_spd, None, &mut scratch)
    }

    /// Factorizes `matrix` reusing `scratch` and an optional prior `SparseLdlt` allocation.
    pub fn factorize_with_symbolic_into(
        sym: &SymbolicLdlt,
        matrix: &CsrMatrix,
        require_spd: bool,
        reuse: Option<Self>,
        scratch: &mut SparseLdltScratch,
    ) -> Result<Self, FeaError> {
        let n = sym.n;
        if matrix.nrows != n || matrix.ncols != n {
            return Err(FeaError::EmptyMesh);
        }

        scratch.lower_vals.resize(sym.lower_col_row.len(), 0.0);
        scratch.lower_vals.fill(0.0);
        for (&val, (&slot, &scale)) in matrix
            .values
            .iter()
            .zip(sym.csr_slot.iter().zip(sym.csr_scale.iter()))
        {
            if slot != usize::MAX {
                if !val.is_finite() {
                    return Err(FeaError::General(
                        "Non-finite entry encountered in sparse matrix factorization".to_string(),
                    ));
                }
                scratch.lower_vals[slot] += scale * val;
            }
        }

        let mut max_diag = 0.0_f64;
        for k in 0..n {
            let start = sym.lower_col_ptr[k];
            let end = sym.lower_col_ptr[k + 1];
            if end > start && sym.lower_col_row[start] as usize == k {
                max_diag = max_diag.max(scratch.lower_vals[start].abs());
            }
        }
        let pivot_tol = (max_diag * 1e-14).max(1e-28);

        let num_snodes = sym.plan.num_snodes;
        let (mut panels, mut d) = match reuse {
            Some(old) if old.n == n && old.panels.len() == num_snodes => (old.panels, old.d),
            _ => (vec![Vec::new(); num_snodes], vec![0.0_f64; n]),
        };
        if d.len() != n {
            d.resize(n, 0.0);
        }

        let plan = &sym.plan;
        let lower_vals = scratch.lower_vals.as_slice();
        let n_workers = rayon::current_num_threads().max(1);
        if scratch.workers.is_empty() {
            scratch.workers = (0..n_workers)
                .map(|_| PanelWorkerScratch::new(n))
                .collect();
        } else {
            while scratch.workers.len() < n_workers {
                scratch.workers.push(PanelWorkerScratch::new(n));
            }
            for ws in &mut scratch.workers {
                ws.ensure_dim(n);
            }
        }

        let num_levels = plan.level_start.len() - 1;
        for lv in 0..num_levels {
            let lv_start = plan.level_start[lv];
            let lv_end = plan.level_start[lv + 1];
            let count = lv_end - lv_start;
            let (done_panels, todo_all) = panels.split_at_mut(lv_start);
            let (todo_lv, _) = todo_all.split_at_mut(count);

            if count >= 2 && plan.level_flops[lv] >= 500_000.0 && n_workers > 1 {
                let active_w = count.min(n_workers);
                let chunk_sz = count.div_ceil(active_w).max(1);
                todo_lv
                    .par_chunks_mut(chunk_sz)
                    .zip(scratch.workers[..active_w].par_iter_mut())
                    .enumerate()
                    .try_for_each(|(c_idx, (chunk, ws))| -> Result<(), FeaError> {
                        let base = lv_start + c_idx * chunk_sz;
                        for (off, p_j) in chunk.iter_mut().enumerate() {
                            factorize_supernode_panel(
                                base + off,
                                p_j,
                                done_panels,
                                plan,
                                &sym.lower_col_ptr,
                                &sym.lower_col_row,
                                lower_vals,
                                require_spd,
                                pivot_tol,
                                ws,
                            )?;
                        }
                        Ok(())
                    })?;
            } else if count == 1 && plan.level_flops[lv] >= 1_000_000.0 && n_workers > 1 {
                let lv_id = lv_start;
                let meta_j = plan.meta[lv_id];
                let w = meta_j.w;
                let m = meta_j.m;
                let h_total = w + m;
                let buf_len = h_total * w;
                let dep_start = plan.dep_ptr[lv_id];
                let dep_end = plan.dep_ptr[lv_id + 1];
                let n_deps = dep_end - dep_start;
                let p_j = &mut todo_lv[0];
                if n_deps >= 8 && buf_len <= 131_072 {
                    p_j.resize(buf_len + w, 0.0);
                    p_j.fill(0.0);
                    let rows_j = &plan.snode_rows[meta_j.rows_start..meta_j.rows_start + m];
                    load_panel_lower_entries(
                        meta_j,
                        rows_j,
                        p_j,
                        &sym.lower_col_ptr,
                        &sym.lower_col_row,
                        lower_vals,
                        &mut scratch.workers[0].row_map,
                    );
                    let deps_all = &plan.deps[dep_start..dep_end];
                    let dep_chunk = n_deps.div_ceil(n_workers).max(1);
                    let active_workers = n_deps.div_ceil(dep_chunk);
                    scratch.workers[..active_workers]
                        .par_iter_mut()
                        .enumerate()
                        .for_each(|(w_idx, ws)| {
                            let s = w_idx * dep_chunk;
                            let e = (s + dep_chunk).min(n_deps);
                            let mut acc = std::mem::take(&mut ws.panel_acc);
                            acc.resize(buf_len, 0.0);
                            acc[..buf_len].fill(0.0);
                            accumulate_deps_into_panel(
                                &deps_all[s..e],
                                w,
                                rows_j,
                                &mut acc[..buf_len],
                                done_panels,
                                plan,
                                ws,
                            );
                            ws.panel_acc = acc;
                        });
                    for ws in &scratch.workers[..active_workers] {
                        for (dst, &src) in p_j[..buf_len].iter_mut().zip(ws.panel_acc[..buf_len].iter()) {
                            *dst += src;
                        }
                    }
                    finish_panel_ldlt(
                        p_j,
                        w,
                        h_total,
                        require_spd,
                        pivot_tol,
                        &mut scratch.workers[0],
                    )?;
                } else {
                    factorize_supernode_panel(
                        lv_id,
                        p_j,
                        done_panels,
                        plan,
                        &sym.lower_col_ptr,
                        &sym.lower_col_row,
                        lower_vals,
                        require_spd,
                        pivot_tol,
                        &mut scratch.workers[0],
                    )?;
                }
            } else {
                let seq_ws = &mut scratch.workers[0];
                for (off, p_j) in todo_lv.iter_mut().enumerate() {
                    factorize_supernode_panel(
                        lv_start + off,
                        p_j,
                        done_panels,
                        plan,
                        &sym.lower_col_ptr,
                        &sym.lower_col_row,
                        lower_vals,
                        require_spd,
                        pivot_tol,
                        seq_ws,
                    )?;
                }
            }
        }

        let mut min_abs_pivot = f64::INFINITY;
        let mut negative_pivots = 0usize;
        for (lv_id, meta_j) in plan.meta.iter().enumerate() {
            let p_j = &panels[lv_id];
            let d_off = (meta_j.w + meta_j.m) * meta_j.w;
            for c in 0..meta_j.w {
                let d_k = p_j[d_off + c];
                if d_k < 0.0 {
                    negative_pivots += 1;
                }
                min_abs_pivot = min_abs_pivot.min(d_k.abs());
                d[meta_j.col_start + c] = d_k;
            }
        }

        Ok(Self {
            n,
            total_l_nnz: sym.total_l_nnz,
            perm: Arc::clone(&sym.perm),
            plan: Arc::clone(&sym.plan),
            panels,
            d,
            min_abs_pivot,
            negative_pivots,
        })
    }

    /// Number of non-zero entries in the lower-triangular factor $L$.
    pub fn factor_nnz(&self) -> usize {
        self.total_l_nnz
    }

    /// Dimension `n` of the factorized system.
    pub fn dimension(&self) -> usize {
        self.n
    }

    /// Solves $A x = b$ into caller-supplied `out` using workspace `work` (`len >= n`), with zero allocations.
    pub fn solve_into(&self, b: &[f64], out: &mut [f64], work: &mut [f64]) {
        let n = self.n;
        let perm = self.perm.as_slice();
        for i in 0..n {
            work[i] = b[perm[i]];
        }

        let plan = &self.plan;
        let num_snodes = plan.num_snodes;

        // Forward substitution: L y = P b (supernodes in increasing column order)
        for &lv_id in &plan.orig_to_lv {
            let meta = plan.meta[lv_id];
            let col0 = meta.col_start;
            let w = meta.w;
            let m = meta.m;
            let p_j = &self.panels[lv_id];
            let rows_j = &plan.snode_rows[meta.rows_start..meta.rows_start + m];

            for r in 1..w {
                let mut sum = work[col0 + r];
                let p_row = &p_j[r * w..r * w + r];
                for c in 0..r {
                    sum -= p_row[c] * work[col0 + c];
                }
                work[col0 + r] = sum;
            }

            let col_end = col0 + w;
            let (work_head, work_tail) = work.split_at_mut(col_end);
            let x_blk = &work_head[col0..col_end];
            let mut idx = 0usize;
            while idx + 4 <= m {
                let r0 = &p_j[(w + idx) * w..(w + idx + 1) * w];
                let r1 = &p_j[(w + idx + 1) * w..(w + idx + 2) * w];
                let r2 = &p_j[(w + idx + 2) * w..(w + idx + 3) * w];
                let r3 = &p_j[(w + idx + 3) * w..(w + idx + 4) * w];
                let mut acc = [0.0_f64; 4];
                for c in 0..w {
                    let xc = x_blk[c];
                    acc[0] += r0[c] * xc;
                    acc[1] += r1[c] * xc;
                    acc[2] += r2[c] * xc;
                    acc[3] += r3[c] * xc;
                }
                work_tail[rows_j[idx] as usize - col_end] -= acc[0];
                work_tail[rows_j[idx + 1] as usize - col_end] -= acc[1];
                work_tail[rows_j[idx + 2] as usize - col_end] -= acc[2];
                work_tail[rows_j[idx + 3] as usize - col_end] -= acc[3];
                idx += 4;
            }
            while idx < m {
                let p_row = &p_j[(w + idx) * w..(w + idx + 1) * w];
                let mut acc = 0.0_f64;
                for c in 0..w {
                    acc += p_row[c] * x_blk[c];
                }
                work_tail[rows_j[idx] as usize - col_end] -= acc;
                idx += 1;
            }
        }

        for (xj, &dj) in work.iter_mut().zip(self.d.iter()).take(n) {
            *xj /= dj;
        }

        // Backward substitution: L^T z = y (supernodes in decreasing column order)
        let mut t = [0.0_f64; SUPERNODE_MAX_W];
        for orig_s in (0..num_snodes).rev() {
            let lv_id = plan.orig_to_lv[orig_s];
            let meta = plan.meta[lv_id];
            let col0 = meta.col_start;
            let w = meta.w;
            let m = meta.m;
            let p_j = &self.panels[lv_id];
            let rows_j = &plan.snode_rows[meta.rows_start..meta.rows_start + m];

            if m > 0 {
                let tw = &mut t[..w];
                tw.fill(0.0);
                let mut idx = 0usize;
                while idx + 4 <= m {
                    let wr0 = work[rows_j[idx] as usize];
                    let wr1 = work[rows_j[idx + 1] as usize];
                    let wr2 = work[rows_j[idx + 2] as usize];
                    let wr3 = work[rows_j[idx + 3] as usize];
                    let r0 = &p_j[(w + idx) * w..(w + idx + 1) * w];
                    let r1 = &p_j[(w + idx + 1) * w..(w + idx + 2) * w];
                    let r2 = &p_j[(w + idx + 2) * w..(w + idx + 3) * w];
                    let r3 = &p_j[(w + idx + 3) * w..(w + idx + 4) * w];
                    for c in 0..w {
                        tw[c] += r0[c] * wr0 + r1[c] * wr1 + r2[c] * wr2 + r3[c] * wr3;
                    }
                    idx += 4;
                }
                while idx < m {
                    let wr = work[rows_j[idx] as usize];
                    if wr != 0.0 {
                        let p_row = &p_j[(w + idx) * w..(w + idx + 1) * w];
                        for c in 0..w {
                            tw[c] += p_row[c] * wr;
                        }
                    }
                    idx += 1;
                }
                for c in 0..w {
                    work[col0 + c] -= tw[c];
                }
            }

            for r in (1..w).rev() {
                let wr = work[col0 + r];
                if wr != 0.0 {
                    let p_row = &p_j[r * w..r * w + r];
                    for c in 0..r {
                        work[col0 + c] -= p_row[c] * wr;
                    }
                }
            }
        }

        for i in 0..n {
            out[perm[i]] = work[i];
        }
    }

    /// Solves $A x = b$ using the precomputed $P A P^T = L D L^T$ factorization.
    pub fn solve(&self, b: &[f64]) -> Vec<f64> {
        let n = self.n;
        let mut work = vec![0.0_f64; n];
        let mut out = vec![0.0_f64; n];
        self.solve_into(b, &mut out, &mut work);
        out
    }
}

/// Reusable linear solver cache across Newton-Raphson iterations and time steps.
#[derive(Debug, Clone, Default)]
pub struct SparseSolverCache {
    /// Cached symbolic $LDL^T$ analysis for the current sparsity pattern.
    pub symbolic_ldlt: Option<SymbolicLdlt>,
    /// Most recent valid direct $LDL^T$ numeric factorization (enables Modified Newton / PCG reuse).
    pub last_ldlt: Option<SparseLdlt>,
    /// Spare numeric factor buffer swapped in to avoid `mmap` allocations across steps.
    pub spare_ldlt: Option<SparseLdlt>,
    /// Time step $\Delta t$ at which `last_ldlt` was factorized.
    pub last_ldlt_dt: f64,
    /// Number of accepted time steps since `last_ldlt` was refreshed.
    pub last_ldlt_age: usize,
    /// Reusable numeric factorization / solve scratch buffers.
    pub scratch: SparseLdltScratch,
    /// Reusable Krylov buffers for lagged-$LDL^T$ Preconditioned CG.
    pub pcg_scratch: PcgLdltScratch,
}

impl SparseSolverCache {
    /// Clears cached factorizations (e.g. when DOF count or element topology changes).
    pub fn clear(&mut self) {
        self.symbolic_ldlt = None;
        self.last_ldlt = None;
        self.spare_ldlt = None;
        self.last_ldlt_dt = 0.0;
        self.last_ldlt_age = 0;
    }

    /// Ensures `symbolic_ldlt` matches `matrix` and returns a reference to it.
    pub fn ensure_symbolic(&mut self, matrix: &CsrMatrix) -> Result<&SymbolicLdlt, FeaError> {
        let hit = self
            .symbolic_ldlt
            .as_ref()
            .is_some_and(|s| s.matches(matrix));
        if !hit {
            self.symbolic_ldlt = Some(SymbolicLdlt::analyze(matrix)?);
        }
        Ok(self.symbolic_ldlt.as_ref().expect("symbolic analyzed"))
    }
}

/// Solves $A x = b$ via Preconditioned Conjugate Gradient using a lagged `SparseLdlt`
/// factor as preconditioner $M^{-1} = (L D L^T)^{-1}$ (`work` reused inside `scratch`).
/// Returns `Some((x, iters))` if converged within `max_iters`, else `None`.
pub fn solve_pcg_with_ldlt_precond(
    matrix: &CsrMatrix,
    precond: &SparseLdlt,
    b: &[f64],
    tol: f64,
    max_iters: usize,
    scratch: &mut PcgLdltScratch,
) -> Option<(Vec<f64>, usize)> {
    let n = matrix.nrows;
    if n == 0 || matrix.ncols != n || precond.dimension() != n || b.len() != n {
        return None;
    }
    if precond.negative_pivots > 0 {
        return None;
    }
    let norm_b = norm(b);
    if !norm_b.is_finite() {
        return None;
    }
    if norm_b <= 1e-15 {
        return Some((vec![0.0; n], 0));
    }

    scratch.r.resize(n, 0.0);
    scratch.r.copy_from_slice(b);
    scratch.z.resize(n, 0.0);
    scratch.p.resize(n, 0.0);
    scratch.ap.resize(n, 0.0);
    scratch.ldlt_work.resize(n, 0.0);

    precond.solve_into(&scratch.r, &mut scratch.z, &mut scratch.ldlt_work);
    let mut rho = dot(&scratch.r, &scratch.z);
    if rho <= 0.0 || !rho.is_finite() {
        return None;
    }
    scratch.p.copy_from_slice(&scratch.z);
    let mut x = vec![0.0_f64; n];

    for iter in 0..max_iters {
        matrix.spmv(&scratch.p, &mut scratch.ap);
        let p_ap = dot(&scratch.p, &scratch.ap);
        if p_ap <= 1e-300 || !p_ap.is_finite() {
            return None;
        }
        let alpha = rho / p_ap;
        for i in 0..n {
            x[i] += alpha * scratch.p[i];
            scratch.r[i] -= alpha * scratch.ap[i];
        }
        let norm_r = norm(&scratch.r);
        if !norm_r.is_finite() {
            return None;
        }
        if norm_r <= tol * norm_b || norm_r <= 1e-13 {
            if dot(&x, b) > 0.0 {
                return Some((x, iter + 1));
            }
            return None;
        }
        if iter + 1 == max_iters {
            return None;
        }
        precond.solve_into(&scratch.r, &mut scratch.z, &mut scratch.ldlt_work);
        let rho_new = dot(&scratch.r, &scratch.z);
        if rho_new <= 0.0 || !rho_new.is_finite() {
            return None;
        }
        let beta = rho_new / rho;
        rho = rho_new;
        for i in 0..n {
            scratch.p[i] = scratch.z[i] + beta * scratch.p[i];
        }
    }
    None
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
        for i in 0..self.n {
            let mut sum = r[i];
            for p in self.l_row_ptr[i]..self.l_row_ptr[i + 1] {
                sum -= self.l_val[p] * y[self.l_col_idx[p]];
            }
            y[i] = sum / self.l_diag[i];
        }

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
    let mut cache = SparseSolverCache::default();
    solve_linear_internal(matrix, rhs, kind, options, false, &mut cache)
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
    let mut cache = SparseSolverCache::default();
    solve_linear_internal(matrix, rhs, kind, options, true, &mut cache)
}

/// Cached variant of [`solve_spd_newton_robust`] that reuses symbolic $LDL^T$ analysis across
/// Newton iterations and stores the latest numeric [`SparseLdlt`] factor for Modified Newton steps.
pub fn solve_spd_newton_cached(
    matrix: &CsrMatrix,
    rhs: &[f64],
    kind: LinearSolverKind,
    options: SolverOptions,
    cache: &mut SparseSolverCache,
) -> Result<Vec<f64>, FeaError> {
    solve_linear_internal(matrix, rhs, kind, options, true, cache)
}

fn solve_linear_internal(
    matrix: &CsrMatrix,
    rhs: &[f64],
    kind: LinearSolverKind,
    options: SolverOptions,
    require_descent: bool,
    cache: &mut SparseSolverCache,
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

    let try_direct_cached =
        |mat: &CsrMatrix, require_spd: bool, cache: &mut SparseSolverCache| -> Result<Vec<f64>, FeaError> {
            let sym = cache.ensure_symbolic(mat)?.clone();
            let reuse = cache.spare_ldlt.take();
            if let Ok(factor) = SparseLdlt::factorize_with_symbolic_into(
                &sym,
                mat,
                require_spd,
                reuse,
                &mut cache.scratch,
            ) {
                cache.scratch.solve_work.resize(n, 0.0);
                let mut x = vec![0.0_f64; n];
                factor.solve_into(rhs, &mut x, &mut cache.scratch.solve_work);
                if accept(&x) {
                    if let Some(old) = cache.last_ldlt.replace(factor) {
                        cache.spare_ldlt = Some(old);
                    }
                    cache.last_ldlt_age = 0;
                    return Ok(x);
                }
                cache.spare_ldlt = Some(factor);
            }
            if require_spd {
                let reuse = cache.spare_ldlt.take();
                if let Ok(factor) = SparseLdlt::factorize_with_symbolic_into(
                    &sym,
                    mat,
                    false,
                    reuse,
                    &mut cache.scratch,
                ) {
                    cache.scratch.solve_work.resize(n, 0.0);
                    let mut x = vec![0.0_f64; n];
                    factor.solve_into(rhs, &mut x, &mut cache.scratch.solve_work);
                    if accept(&x) {
                        if let Some(old) = cache.last_ldlt.replace(factor) {
                            cache.spare_ldlt = Some(old);
                        }
                        cache.last_ldlt_age = 0;
                        return Ok(x);
                    }
                    cache.spare_ldlt = Some(factor);
                }
            }
            Err(FeaError::UnconstrainedModel)
        };

    match kind {
        LinearSolverKind::SparseDirectLdlt => {
            if let Ok(x) = try_direct_cached(matrix, require_descent, cache) {
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
            // Use AMD-ordered direct LDL^T with cached symbolic analysis whenever estimated flops are tractable.
            if n <= 80_000 {
                if let Ok(sym) = cache.ensure_symbolic(matrix) {
                    let flops_ok = sym.estimated_flops() <= 6.0e8 || n <= 2_500;
                    if flops_ok {
                        if let Ok(x) = try_direct_cached(matrix, require_descent, cache) {
                            return Ok(x);
                        }
                    }
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
            if n <= 6000 {
                if let Ok(x) = try_direct_cached(matrix, require_descent, cache) {
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
        if n <= 12_000 {
            if let Ok(x) = try_direct_cached(&k_reg, require_descent, cache) {
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
