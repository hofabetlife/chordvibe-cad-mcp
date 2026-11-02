# Engineering Benchmark Suite (`bench/`)

Reproducible 15-task evaluation suite verifying MCP tool execution, mesh watertightness, dimensional accuracy, linear static FEA, and multi-body kinematics.

## Execution

```bash
npx tsx bench/run_benchmark.ts
```

## Task Categories & Pass Criteria

The suite (`bench/tasks.json`) defines 15 engineering tasks across three categories:

1. **Simple Primitives & Booleans (`T01`–`T05`)**: Box, cylinder, sphere, cone frustum, and perforated block (`box` + `cut_hole`).
2. **Parametric Parts (`T06`–`T10`)**: Kirsch tension plate (`600×200×10 mm`, `R=20 mm`), revolved annular bushing (`360°`), M10 chamfered hex nut with helical thread (`polygon_prism` + `cut_hole` + `cut_helical_groove`), involute spur gear (`z=20, m=2 mm` with hub and keyway), and two-hole mounting plate (`genus=2`).
3. **Full CAD + FEA / Kinematics Cycles (`T11`–`T15`)**:
   - `T11`: Prismatic axial tension bar (`1000×10×10 mm`, $F_x = 2050\text{ N}$, analytical $\sigma = 20.50\text{ MPa}$, $\delta = 0.100\text{ mm}$).
   - `T12`: Cantilever beam bending with quadratic `TET10` elements (`200×20×20 mm`, $F_y = -1000\text{ N}$, analytical $\sigma = 150.0\text{ MPa}$, $\delta = 0.9756\text{ mm}$).
   - `T13`: Kirsch/Peterson plate stress concentration (`600×200×10 mm`, hole $d = 40\text{ mm}$, $F_x = 100\text{ kN}$, Peterson $\sigma_{\max} = 157.44\text{ MPa}$).
   - `T14`: External spur gear pair ($z_1 = 20, z_2 = 40$) over 10 full driver revolutions ($\omega_2/\omega_1 = -0.500$).
   - `T15`: Slider-crank mechanism ($r = 50\text{ mm}, l = 200\text{ mm}$) over 1 full crank revolution compared against analytical $x(\theta)$.

### Pass Criteria per Task
- All MCP tool calls execute without error (`calls_succeeded = true`).
- Resulting triangle mesh is watertight (`is_watertight = true`) with matching topological genus.
- Bounding-box dimensions match specification within $\le 1\%$ (`max_dimension_error_pct <= 1.0`).
- For `T11`–`T15`: FEA safety verdict (`SAFE`/`FAIL`) and peak von Mises stress match analytical reference within task tolerance, or kinematic velocity ratio / slider position match analytical equations.

## Issues Identified & Fixed During Benchmark Audit

During initial execution of the 15-task benchmark prior to fixes (`12/15` passed initially, `14/15` on second iteration, `15/15` after fixes):
1. **`run_fea_analysis` schema aliases (`src/cad/mcp_dispatcher.ts`)**: LLM tool calls frequently pass `face_id` inside `applied_forces` items (`{ "face_id": "max_x", "force_vector": [...] }`), `element_type: "TET10"` alongside `element_order`, and inspect `max_von_mises_stress_mpa` + `verdict` (`SAFE`/`FAIL`). Added support for `face_id`, `element_type`, `max_von_mises_stress_mpa`, and `verdict`.
2. **Axis convention for `revolve` and `polygon_prism`**: Both primitives orient their axial height along the $Z$ axis (with radial profiles in the $XY$ plane), whereas `cylinder` and `cone` orient height along $Y$. Documented explicitly in `bench/tasks.json` and `docs/MCP_API_REFERENCE.md`.
3. **3D Saint-Venant clamp effect in short axial bars**: Clamping all three translational DOFs ($u_x = u_y = u_z = 0$) on `min_x` for a short bar ($L/h = 10$) introduces a $\approx 1.57\%$ Poisson-effect stress concentration at the fixed corners; using a slender prismatic bar ($L/h = 100$ in `T11`) matches the 1D analytical $\sigma = F/A = 20.50\text{ MPa}$ to within $0.01\%$.

## Benchmark Results (`npx tsx bench/run_benchmark.ts`)

| Task ID | Category | Watertight | Max Dim Error (%) | Domain Verification Result | Duration (ms) | Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `T01_steel_block` | `simple_primitive_boolean` | `true` | `0.0000%` | Geometry & topology verified ($g=0$) | `38.55` | `PASS` |
| `T02_aluminum_shaft` | `simple_primitive_boolean` | `true` | `0.0000%` | Geometry & topology verified ($g=0$) | `182.30` | `PASS` |
| `T03_titanium_sphere` | `simple_primitive_boolean` | `true` | `0.0000%` | Geometry & topology verified ($g=0$) | `257.63` | `PASS` |
| `T04_tapered_cone_pin` | `simple_primitive_boolean` | `true` | `0.0000%` | Geometry & topology verified ($g=0$) | `103.30` | `PASS` |
| `T05_boolean_perforated_block` | `simple_primitive_boolean` | `true` | `0.0000%` | Geometry & topology verified ($g=1$) | `37.50` | `PASS` |
| `T06_kirsch_plate_600x200x10` | `parametric_part` | `true` | `0.0000%` | Geometry & topology verified ($g=1$) | `170.42` | `PASS` |
| `T07_annular_bushing_revolve` | `parametric_part` | `true` | `0.0000%` | Geometry & topology verified ($g=1$) | `103.61` | `PASS` |
| `T08_hex_nut_m10` | `parametric_part` | `true` | `0.0006%` | Geometry & topology verified ($g=1$) | `93.20` | `PASS` |
| `T09_spur_gear_z20_m2` | `parametric_part` | `true` | `0.0000%` | Geometry & topology verified ($g=1$) | `134.90` | `PASS` |
| `T10_two_hole_mounting_plate` | `parametric_part` | `true` | `0.0000%` | Geometry & topology verified ($g=2$) | `166.43` | `PASS` |
| `T11_axial_bar_fea_cycle` | `full_cycle_fea_or_kinematics` | `true` | `0.0000%` | FEA `SAFE`, $\sigma = 20.50\text{ MPa}$ (err `0.01%`) | `160.54` | `PASS` |
| `T12_cantilever_tet10_fea_cycle` | `full_cycle_fea_or_kinematics` | `true` | `0.0000%` | FEA `SAFE`, $\sigma = 149.09\text{ MPa}$ (err `0.61%`) | `771.75` | `PASS` |
| `T13_kirsch_plate_fea_cycle` | `full_cycle_fea_or_kinematics` | `true` | `0.0000%` | FEA `SAFE`, $\sigma = 160.29\text{ MPa}$ (err `1.81%`) | `5562.37` | `PASS` |
| `T14_gear_pair_kinematics_cycle` | `full_cycle_fea_or_kinematics` | `true` | `0.0000%` | Gear $\omega_2/\omega_1 = -0.500$, drift `0.00e+0 m` | `455.45` | `PASS` |
| `T15_slider_crank_kinematics_cycle` | `full_cycle_fea_or_kinematics` | `true` | `0.0000%` | Slider-crank max $x(\theta)$ err `0.0000%` | `21.72` | `PASS` |
