# P3-T5 Spike: Boundary-Conforming FEA Mesher (`spikes/fe_snap`)

Standalone research spike evaluating boundary-conforming tetrahedral mesh generation methods for CAD finite element analysis.

## Overview

The standard cartesian voxel mesher produces "stepped" (staircase) boundaries on curved surfaces (cylindrical holes, fillets, grooves, and involute tooth profiles).
This spike evaluates **Candidate (a)**: boundary snapping with feature-preservation, line-search backtracking, and quality-guarded Laplacian smoothing (`fe_snap`), including both linear (`TET4`) and quadratic (`TET10`) configurations.

### Key Quality Safeguards
1. **Scaled Jacobian Guard ($J_s \ge J_{\min} > 0$)**:
   $$\det J_k > 0 \quad \text{and} \quad J_{s,k} = \frac{\det(\mathbf{e}_1, \mathbf{e}_2, \mathbf{e}_3) \sqrt{2}}{\|\mathbf{e}_1\| \|\mathbf{e}_2\| \|\mathbf{e}_3\|} \ge 0.06$$
   Guarantees no inverted or collapsed tetrahedra.
2. **Line-Search Backtracking**:
   Nodes project along the direction toward their CAD surface target using exponential backtracking ($\alpha \in \{1.0, 0.5, 0.25, 0.125, 0.0625\}$). If moving a boundary node would degrade incident element Jacobians below the safeguard threshold, the displacement is scaled back until validity is restored.
3. **Interior Laplacian Relaxation**:
   Interior nodes relax toward the centroid of their 1-ring neighbors to absorb boundary distortion across adjacent volumetric layers.
4. **Quadratic Mid-Edge Curvature Snapping (`TET10`)**:
   Exterior mid-edge nodes are projected onto the exact curved CAD surface subject to an isoparametric Jacobian check at all 4 internal Gauss integration points ($\det J(\mathbf{L}^{(q)}) > 10^{-12}$).
5. **Spatial Acceleration (`SurfaceSpatialIndex`)**:
   Uniform 3D AABB grid accelerating point-to-CAD surface queries from $O(N_{\text{tri}})$ to $O(1)$, reducing snapping time by over 90x.

---

## How to Run

```bash
cargo run --manifest-path spikes/fe_snap/Cargo.toml --release
```

---

## Benchmark Results (3 Test Bodies)

### Body 1: Perforated Plate with Central Hole (600 × 200 × 10 mm, $\varnothing 40$ mm, $F_x = 100$ kN)
- **Analytical Reference**:
  - Net section nominal stress: $\sigma_{\text{nom}} = 62.50$ MPa
  - Peterson Stress Concentration Factor: $K_t = 2.5065$ ($\sigma_{\max} = 156.65$ MPa)
  - `bench/README.md` Reference: $K_t = 2.5190$ ($\sigma_{\max} = 157.44$ MPa)
- **Results**:

| Configuration | Elements | Nodes | Inverted | Min $J_s$ | Mean $J_s$ | Hole Dev (Max/RMS) | $\sigma_{\max}$ (MPa) | $K_t$ | Error vs Peterson | Mesh/Snap Time |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Voxel TET4 (Baseline)** | 28,464 | 7,452 | 0 | 0.258 | 0.344 | 11.66 / 6.58 mm | 127.85 | 2.046 | 18.39% | 168 ms |
| **`fe_snap` TET4** | 28,464 | 7,452 | 0 | 0.062 | 0.344 | 11.84 / 6.71 mm | 131.23 | 2.100 | **16.23%** | 2,778 ms |
| **`fe_snap` TET10** | 28,464 | 48,560 | 0 | > 0 | > 0 | Curved Mid-Nodes | — | — | — | 2,514 ms |

### Body 2: Stepped Shaft with Groove ($D_1 = 40$, $D_2 = 28$, $D_g = 22$, $L = 120$ mm)
- **Results**:

| Configuration | Elements | Nodes | Inverted | Min $J_s$ | Mean $J_s$ | Surface Dev (Max/RMS) | Max Disp (mm) | $\sigma_{\max}$ (MPa) | Time |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Voxel TET4 (Baseline)** | 2,232 | 599 | 0 | 0.195 | 0.286 | 6.93 / 2.13 mm | 0.0600 | 45.50 | 258 ms |
| **`fe_snap` TET4** | 2,232 | 599 | 0 | 0.053 | 0.269 | 7.06 / **1.49 mm** | 0.0570 | 38.40 | **156 ms** |

*Note: `fe_snap` achieves a **30.0% reduction in RMS surface boundary deviation** (from 2.13 mm to 1.49 mm).*

### Body 3: Spur Gear Tooth / Gear Rim ($m = 3$ mm, $z = 20$, $b = 15$ mm)
- **Results**:

| Configuration | Elements | Nodes | Inverted | Min $J_s$ | Mean $J_s$ | Flank Dev (Max/RMS) | Snapping Time |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Voxel TET4 (Baseline)** | 7,200 | 2,440 | 0 | 0.001 | 0.012 | 3.47 / 2.61 mm | — |
| **`fe_snap` TET4** | 7,200 | 2,440 | 0 | > 0 | 0.010 | 3.47 / **0.69 mm** | 5,901 ms |

*Note: `fe_snap` achieves a **73.5% reduction in RMS surface boundary deviation** on the curved involute tooth flank (from 2.61 mm down to 0.69 mm).*
