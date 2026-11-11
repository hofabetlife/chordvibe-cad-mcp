# Rust `mcp-cad-server` Baseline Benchmark Report (P1-T2)

> **Task**: `P1-T2` — Бенчмарк 15 задач против Rust-сервера  
> **Target Engine**: Native Rust `mcp-cad-server` (`crates/mcp-cad-server`, `crates/mcp-cad-core`, `crates/mcp-cad-fea`, `crates/mcp-cad-physics`)  
> **Benchmark Suite**: `bench/tasks.json` (15 reference tasks `T01`–`T15`, unchanged)  
> **Runners**:
> - `cargo test -p mcp-cad-server --test bench_suite -- --nocapture` (native Rust JSON-RPC harness via `stdio::handle_message`)
> - `npx tsx bench/run_benchmark_rust.ts` (TypeScript CLI spawning `mcp-cad-server --headless` over `stdin`/`stdout` JSON-RPC 2.0)

---

## 1. Executive Summary & Comparison Against `bench/README.md`

Previously, `bench/run_benchmark.ts` imported `createHeadlessDispatcher()` from `src/cad/mcp_dispatcher.ts` — the **TypeScript browser mock** — and never invoked the native Rust `mcp-cad-server` binary. Furthermore, `bench/README.md` claimed `15 / 15 PASSED (100.0%)` and asserted that the benchmark validated shared invariants across both engines.

Running the exact same 15 tasks from `bench/tasks.json` against the **real Rust `mcp-cad-server`** reveals the true baseline:

| Execution Mode | Description | Score | Pass Rate |
| :--- | :--- | :---: | :---: |
| **Claimed in `bench/README.md`** | TypeScript mock (`src/cad/mcp_dispatcher.ts`) | `15 / 15` | `100.0%` |
| **Mode 1: Strict As-Is** | `bench/tasks.json` steps sent verbatim over JSON-RPC to Rust `mcp-cad-server` | **`0 / 15`** | **`0.0%`** |
| **Mode 2: With `create_part` Pre-Init** | Explicit `create_part` injected before `task.steps` (with material ID mapping), `task.steps` sent verbatim | **`2 / 15`** | **`13.3%`** |
| **Mode 3: Schema-Bridged Diagnostic** | Tool argument schema differences bridged (`through` $\to$ `depth`, `outer_radius` $\to$ `radius`, `inner_radius` $\to$ 4-pt profile, `joint_id` extraction), keeping all numbers/units from `bench/tasks.json` untouched | **`6 / 15`** | **`40.0%`** |

### Fact-Check of Claims in `bench/README.md`

| Claim in `bench/README.md` | Reality on Native Rust `mcp-cad-server` |
| :--- | :--- |
| *"Benchmark Runner (`bench/run_benchmark.ts`) executes all 15 tasks against the headless MCP dispatcher"* | `bench/run_benchmark.ts` only tested the TypeScript mock (`src/cad/mcp_dispatcher.ts`), never the Rust server (`mcp-cad-server`). |
| *"Both TS and Rust engines generate cylinders and cones along the Y-axis (`[2R, H, 2R]`)"* | **False.** Rust `mcp-cad-core/src/primitives.rs` generates cylinders (`make_cylinder`) and cones (`make_cone`) along the **Z-axis** (`[2R, 2R, H]`), causing `T02` (`150.0%` dim err) and `T04` (`28.57%` dim err) to fail against `expected_dimensions_m`. |
| *"Units are strictly SI (`m`, `Pa`, `kg/m³`, `N`, `rad/s`)"* | **Violated in `T09` and `T14`.** `bench/tasks.json` passes `module: 2.0` (millimeters!) while expecting pitch diameter $d_p = m \cdot z = 0.040\text{ m}$. The TS mock (`src/cad/primitives.ts:392`) silently hid this via `rawModule > 0.1 ? rawModule * 0.001 : rawModule`. Rust (`crates/mcp-cad-core/src/primitives.rs:795`) treats `module: 2.0` as $2.0\text{ m}$, producing a $44\text{ m}$ gear (`99900%` dimensional error). |
| *"T08 M10 Hex Nut (`polygon_prism` + `cut_helical_groove`) is watertight (`expected_genus = 1`)"* | **False on Rust.** Even after bridging `outer_radius` $\to$ `radius`, `cut_helical_groove` in Rust (`crates/mcp-cad-core/src/dag.rs:676`) produces a **non-watertight** mesh (`is_watertight = false`, `open_boundary_edges = 413`, `genus = 68`). |
| *"T12 Cantilever Beam (`TET10` Quadratic Mesh) achieves `< 2.0%` error"* | **False on Rust.** `crates/mcp-cad-server/src/tools/fea.rs` ignores `element_type: "TET10"` and always meshes with linear `TET4` voxels (`max_cells_along_longest_axis = 60`), resulting in shear locking (`19.73%` displacement error, `21.65%` stress error). |
| *"T14 & T15 Kinematics (`create_joint` + `set_joint_motor` + `step_simulation`) pass"* | **False on Rust.** `crates/mcp-cad-server/src/tools/physics.rs:264` ignores `args.get("joint_id")` in `create_joint` and generates a random UUID (`joint-<uuid>`), so `set_joint_motor` immediately fails with `Joint with id '...' was not found`. Furthermore, `T15` in `bench/tasks.json` never connects `t15_connecting_rod` with joints — TS faked slider motion analytically by matching the substring `"slider"` in `part_id`! |

---

## 2. Mode 1: Strict As-Is (`0 / 15 PASSED — 0.0%`)

In **Mode 1**, each step in `bench/tasks.json` is sent verbatim to `mcp-cad-server` with no preprocessing.

Every single task (`T01` through `T15`) begins its `steps` array with `add_feature_primitive` referencing a `part_id` (e.g., `"t01_block"`) without calling `create_part` first:
- In the TypeScript mock (`src/cad/mcp_dispatcher.ts:517`), `ensurePart()` implicitly auto-creates missing parts on the fly and reads `task.material` from the benchmark harness.
- In the Rust server (`crates/mcp-cad-server/src/tools/cad.rs:483`), `add_feature_primitive` strictly checks `s.parts.get_mut(&part_id)` and returns `Part not found: <part_id>`.

| Task ID | Domain | Status | Calls | Watertight | Max Dim Err (%) | Domain Fact vs Ref | Failure Reason |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `T01_steel_block` | `geometry_primitive` | **FAIL** | `0/1` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.1200,0.0800,0.0400] m (g=none vs 0)` | `Step 1 (add_feature_primitive) failed: Part not found: t01_block` |
| `T02_aluminum_shaft` | `geometry_primitive` | **FAIL** | `0/1` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.0600,0.1500,0.0600] m (g=none vs 0)` | `Step 1 (add_feature_primitive) failed: Part not found: t02_shaft` |
| `T03_titanium_sphere` | `geometry_primitive` | **FAIL** | `0/1` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.0800,0.0800,0.0800] m (g=none vs 0)` | `Step 1 (add_feature_primitive) failed: Part not found: t03_sphere` |
| `T04_tapered_cone_pin` | `geometry_primitive` | **FAIL** | `0/1` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.0700,0.0900,0.0700] m (g=none vs 0)` | `Step 1 (add_feature_primitive) failed: Part not found: t04_cone` |
| `T05_boolean_perforated_block` | `csg_boolean` | **FAIL** | `0/2` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.1000,0.1000,0.0500] m (g=none vs 1)` | `Step 1 (add_feature_primitive) failed: Part not found: t05_perf` |
| `T06_kirsch_plate_600x200x10` | `csg_boolean` | **FAIL** | `0/2` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.6000,0.2000,0.0100] m (g=none vs 1)` | `Step 1 (add_feature_primitive) failed: Part not found: t06_plate` |
| `T07_annular_bushing_revolve` | `mechanical_feature` | **FAIL** | `0/1` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.0700,0.0700,0.0800] m (g=none vs 1)` | `Step 1 (add_feature_primitive) failed: Part not found: t07_bushing` |
| `T08_hex_nut_m10` | `mechanical_feature` | **FAIL** | `0/3` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.0185,0.0185,0.0080] m (g=none vs 1)` | `Step 1 (add_feature_primitive) failed: Part not found: t08_nut` |
| `T09_spur_gear_z20_m2` | `mechanical_feature` | **FAIL** | `0/1` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.0440,0.0440,0.0250] m (g=none vs 1)` | `Step 1 (add_feature_primitive) failed: Part not found: t09_gear` |
| `T10_two_hole_mounting_plate` | `csg_boolean` | **FAIL** | `0/3` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.2400,0.0800,0.0160] m (g=none vs 2)` | `Step 1 (add_feature_primitive) failed: Part not found: t10_mount` |
| `T11_axial_bar_fea_cycle` | `fea_analysis` | **FAIL** | `0/5` | `false` | `100.0000%` | `no FEA result` | `Step 1 (add_feature_primitive) failed: Part not found: t11_bar` |
| `T12_cantilever_tet10_fea_cycle` | `fea_analysis` | **FAIL** | `0/5` | `false` | `100.0000%` | `no FEA result` | `Step 1 (add_feature_primitive) failed: Part not found: t12_beam` |
| `T13_kirsch_plate_fea_cycle` | `fea_analysis` | **FAIL** | `0/6` | `false` | `100.0000%` | `no FEA result` | `Step 1 (add_feature_primitive) failed: Part not found: t13_kirsch` |
| `T14_gear_pair_kinematics_cycle` | `kinematics_physics` | **FAIL** | `0/8` | `false` | `100.0000%` | `no physics result` | `Step 1 (add_feature_primitive) failed: Part not found: t14_gear_1` |
| `T15_slider_crank_kinematics_cycle` | `kinematics_physics` | **FAIL** | `0/8` | `false` | `100.0000%` | `no physics result` | `Step 1 (add_feature_primitive) failed: Part not found: t15_crank` |

---

## 3. Mode 2: With `create_part` Pre-Initialization, Verbatim Steps (`2 / 15 PASSED — 13.3%`)

In **Mode 2**, the benchmark harness calls `create_part` before executing `task.steps` (matching how `bench/run_benchmark.ts` passed `task.material` to the TS dispatcher, mapping `"Al6061-T6"` $\to$ `"Al-6061-T6"` and `"Ti-6Al-4V"` $\to$ `"Ti6Al4V"` in `crates/mcp-cad-core/src/material.rs`), and then sends all `task.steps` **verbatim** without altering any tool arguments.

| Task ID | Domain | Status | Calls | Watertight | Max Dim Err (%) | Domain Fact vs Ref | Failure Reason |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `T01_steel_block` | `geometry_primitive` | **PASS** | `1/1` | `true` | `0.0000%` | `dims=[0.1200,0.0800,0.0400] m vs [0.1200,0.0800,0.0400] m (g=Some(0) vs 0)` | `OK` |
| `T02_aluminum_shaft` | `geometry_primitive` | **FAIL** | `1/1` | `true` | `150.0000%` | `dims=[0.0599,0.0600,0.1500] m vs [0.0600,0.1500,0.0600] m (g=Some(0) vs 0)` | `dim_err=150.00% > tol=1.00% (actual=[0.0599,0.0600,0.1500], expected=[0.0600,0.1500,0.0600])` |
| `T03_titanium_sphere` | `geometry_primitive` | **PASS** | `1/1` | `true` | `0.1218%` | `dims=[0.0799,0.0800,0.0800] m vs [0.0800,0.0800,0.0800] m (g=Some(0) vs 0)` | `OK` |
| `T04_tapered_cone_pin` | `geometry_primitive` | **FAIL** | `1/1` | `true` | `28.5714%` | `dims=[0.0700,0.0698,0.0900] m vs [0.0700,0.0900,0.0700] m (g=Some(0) vs 0)` | `dim_err=28.57% > tol=1.50% (actual=[0.0700,0.0698,0.0900], expected=[0.0700,0.0900,0.0700])` |
| `T05_boolean_perforated_block` | `csg_boolean` | **FAIL** | `1/2` | `true` | `0.0000%` | `dims=[0.1000,0.1000,0.0500] m vs [0.1000,0.1000,0.0500] m (g=Some(0) vs 1)` | `Step 2 (cut_hole) failed: Invalid argument: Missing required argument 'depth'` |
| `T06_kirsch_plate_600x200x10` | `csg_boolean` | **FAIL** | `1/2` | `true` | `0.0000%` | `dims=[0.6000,0.2000,0.0100] m vs [0.6000,0.2000,0.0100] m (g=Some(0) vs 1)` | `Step 2 (cut_hole) failed: Invalid argument: Missing required argument 'depth'` |
| `T07_annular_bushing_revolve` | `mechanical_feature` | **FAIL** | `0/1` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.0700,0.0700,0.0800] m (g=none vs 1)` | `Step 1 (add_feature_primitive) failed: Invalid argument: Revolve 'profile' must contain at least 3 [r, z] points` |
| `T08_hex_nut_m10` | `mechanical_feature` | **FAIL** | `0/3` | `false` | `100.0000%` | `dims=[0.0000,0.0000,0.0000] m vs [0.0185,0.0185,0.0080] m (g=none vs 1)` | `Step 1 (add_feature_primitive) failed: Invalid argument: Polygon prism requires either 'width_across_flats' or 'radius'` |
| `T09_spur_gear_z20_m2` | `mechanical_feature` | **FAIL** | `1/1` | `true` | `99900.0000%` | `dims=[44.0000,44.0000,0.0250] m vs [0.0440,0.0440,0.0250] m (g=Some(1) vs 1)` | `dim_err=99900.00% > tol=2.00% (actual=[44.0000,44.0000,0.0250], expected=[0.0440,0.0440,0.0250])` |
| `T10_two_hole_mounting_plate` | `csg_boolean` | **FAIL** | `1/3` | `true` | `0.0000%` | `dims=[0.2400,0.0800,0.0160] m vs [0.2400,0.0800,0.0160] m (g=Some(0) vs 2)` | `Step 2 (cut_hole) failed: Invalid argument: Missing required argument 'depth'` |
| `T11_axial_bar_fea_cycle` | `fea_analysis` | **FAIL** | `5/5` | `true` | `0.0000%` | `sigma=21.30 vs 20.50 MPa (err=3.93%), disp=0.1020 vs 0.1000 mm (err=2.00%), sf=11.73 vs 12.20, status=PASS vs SAFE` | `FEA sigma_err=3.93% > tol=1.00%; disp_err=2.00% > tol=1.00%; status='PASS' != 'SAFE'` |
| `T12_cantilever_tet10_fea_cycle` | `fea_analysis` | **FAIL** | `5/5` | `true` | `0.0000%` | `sigma=117.53 vs 150.00 MPa (err=21.65%), disp=0.7831 vs 0.9756 mm (err=19.73%), sf=2.13 vs 1.67, status=PASS vs SAFE` | `FEA sigma_err=21.65% > tol=2.00%; disp_err=19.73% > tol=2.00%; status='PASS' != 'SAFE'` |
| `T13_kirsch_plate_fea_cycle` | `fea_analysis` | **FAIL** | `1/6` | `true` | `0.0000%` | `no FEA result` | `Step 2 (cut_hole) failed: Invalid argument: Missing required argument 'depth'` |
| `T14_gear_pair_kinematics_cycle` | `kinematics_physics` | **FAIL** | `4/8` | `true` | `99900.0000%` | `no physics result` | `Step 5 (set_joint_motor) failed: Physics error: Joint with id 't14_rev_1' was not found in physics world` |
| `T15_slider_crank_kinematics_cycle` | `kinematics_physics` | **FAIL** | `5/8` | `true` | `0.0000%` | `no physics result` | `Step 6 (set_joint_motor) failed: Physics error: Joint with id 't15_joint_crank' was not found in physics world` |

---

## 4. Mode 3: Schema-Bridged Diagnostic Probe (`6 / 15 PASSED — 40.0%`)

Why did `T05`, `T06`, `T07`, `T08`, `T10`, `T13`, `T14`, `T15` fail on tool argument validation in Mode 2? To separate **API schema mismatches** (input for `P2-T1`) from **deeper geometry/FEA/physics bugs** (input for `P3-*`), Mode 3 bridges only the tool schema conventions while keeping every number and unit from `bench/tasks.json` unchanged:
1. `cut_hole`: converts `through: true` into explicit `depth` along `-direction` and shifts mid-plane `[x, y, 0]` start origin to the top face `[x, y, +H/2]` so the blind/through hole cutter traverses the full plate thickness.
2. `revolve`: converts 2-point outer line `[[r_out, z_min], [r_out, z_max]]` + `inner_radius: r_in` into a closed 4-point annular rectangle `[[r_in, z_min], [r_out, z_min], [r_out, z_max], [r_in, z_max]]`.
3. `polygon_prism`: aliases `outer_radius` $\to$ `radius` and `chamfer_radius` $\to$ `chamfer`.
4. `create_joint` $\to$ `set_joint_motor`: captures the server-generated `joint-<uuid>` returned by `create_joint` and forwards it to `set_joint_motor`.
5. FEA `status`: treats `"PASS"` as equivalent to `"SAFE"` when `safety_factor >= 1.0`.

| Task ID | Domain | Status | Calls | Watertight | Max Dim Err (%) | Domain Fact vs Ref | Failure Reason |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `T01_steel_block` | `geometry_primitive` | **PASS** | `1/1` | `true` | `0.0000%` | `dims=[0.1200,0.0800,0.0400] m vs [0.1200,0.0800,0.0400] m (g=Some(0) vs 0)` | `OK` |
| `T02_aluminum_shaft` | `geometry_primitive` | **FAIL** | `1/1` | `true` | `150.0000%` | `dims=[0.0599,0.0600,0.1500] m vs [0.0600,0.1500,0.0600] m (g=Some(0) vs 0)` | `dim_err=150.00% > tol=1.00% (actual=[0.0599,0.0600,0.1500], expected=[0.0600,0.1500,0.0600])` |
| `T03_titanium_sphere` | `geometry_primitive` | **PASS** | `1/1` | `true` | `0.1218%` | `dims=[0.0799,0.0800,0.0800] m vs [0.0800,0.0800,0.0800] m (g=Some(0) vs 0)` | `OK` |
| `T04_tapered_cone_pin` | `geometry_primitive` | **FAIL** | `1/1` | `true` | `28.5714%` | `dims=[0.0700,0.0698,0.0900] m vs [0.0700,0.0900,0.0700] m (g=Some(0) vs 0)` | `dim_err=28.57% > tol=1.50% (actual=[0.0700,0.0698,0.0900], expected=[0.0700,0.0900,0.0700])` |
| `T05_boolean_perforated_block` | `csg_boolean` | **PASS** | `2/2` | `true` | `0.0000%` | `dims=[0.1000,0.1000,0.0500] m vs [0.1000,0.1000,0.0500] m (g=Some(1) vs 1)` | `OK` |
| `T06_kirsch_plate_600x200x10` | `csg_boolean` | **PASS** | `2/2` | `true` | `0.0000%` | `dims=[0.6000,0.2000,0.0100] m vs [0.6000,0.2000,0.0100] m (g=Some(1) vs 1)` | `OK` |
| `T07_annular_bushing_revolve` | `mechanical_feature` | **PASS** | `1/1` | `true` | `0.0000%` | `dims=[0.0700,0.0700,0.0800] m vs [0.0700,0.0700,0.0800] m (g=Some(1) vs 1)` | `OK` |
| `T08_hex_nut_m10` | `mechanical_feature` | **FAIL** | `3/3` | `false` | `0.0006%` | `dims=[0.0185,0.0185,0.0080] m vs [0.0185,0.0185,0.0080] m (g=Some(68) vs 1)` | `mesh is not watertight; genus=Some(68) != expected=1` |
| `T09_spur_gear_z20_m2` | `mechanical_feature` | **FAIL** | `1/1` | `true` | `99900.0000%` | `dims=[44.0000,44.0000,0.0250] m vs [0.0440,0.0440,0.0250] m (g=Some(1) vs 1)` | `dim_err=99900.00% > tol=2.00% (actual=[44.0000,44.0000,0.0250], expected=[0.0440,0.0440,0.0250])` |
| `T10_two_hole_mounting_plate` | `csg_boolean` | **PASS** | `3/3` | `true` | `0.0000%` | `dims=[0.2400,0.0800,0.0160] m vs [0.2400,0.0800,0.0160] m (g=Some(2) vs 2)` | `OK` |
| `T11_axial_bar_fea_cycle` | `fea_analysis` | **FAIL** | `5/5` | `true` | `0.0000%` | `sigma=21.30 vs 20.50 MPa (err=3.93%), disp=0.1020 vs 0.1000 mm (err=2.00%), sf=11.73 vs 12.20, status=PASS vs SAFE` | `FEA sigma_err=3.93% > tol=1.00%; disp_err=2.00% > tol=1.00%` |
| `T12_cantilever_tet10_fea_cycle` | `fea_analysis` | **FAIL** | `5/5` | `true` | `0.0000%` | `sigma=117.53 vs 150.00 MPa (err=21.65%), disp=0.7831 vs 0.9756 mm (err=19.73%), sf=2.13 vs 1.67, status=PASS vs SAFE` | `FEA sigma_err=21.65% > tol=2.00%; disp_err=19.73% > tol=2.00%` |
| `T13_kirsch_plate_fea_cycle` | `fea_analysis` | **FAIL** | `6/6` | `true` | `0.0000%` | `sigma=128.06 vs 157.44 MPa (err=18.66%), disp=0.1533 vs 0.1463 mm (err=4.74%), sf=1.95 vs 1.59, status=PASS vs SAFE` | `FEA sigma_err=18.66% > tol=10.00%` |
| `T14_gear_pair_kinematics_cycle` | `kinematics_physics` | **FAIL** | `8/8` | `true` | `99900.0000%` | `omega1=0.0000 vs 10.0000 rad/s (err=100.00%), omega2=0.0000 vs -5.0000 rad/s (err=100.00%)` | `dim_err=99900.00% > tol=2.00%; omega1_err=100.00% > tol=1.00%; omega2_err=100.00% > tol=1.00%` |
| `T15_slider_crank_kinematics_cycle` | `kinematics_physics` | **FAIL** | `8/8` | `true` | `0.0000%` | `max_x=-0.0020 vs 0.1500 m (err=101.31%)` | `slider max_x_err=101.31% > tol=5.00%` |

---

## 5. Root-Cause Catalogue (Input for `P2-T1` and `P3-*`)

### Category A: Tool Contract & Schema Discrepancies (`P2-T1`)
1. **Implicit vs Explicit `create_part` (`T01`–`T15`)**:
   - `bench/tasks.json` omits `create_part` steps and stores `"material"` on the top-level task object.
   - Rust `add_feature_primitive` requires an existing `part_id` created via `create_part`.
2. **Material Preset ID Naming (`T02`, `T03`)**:
   - `bench/tasks.json` uses `"Al6061-T6"` and `"Ti-6Al-4V"`.
   - `crates/mcp-cad-core/src/material.rs` registers `"Al-6061-T6"` and `"Ti6Al4V"`.
3. **Cylinder and Cone Principal Axis (`T02`, `T04`)**:
   - Rust `make_cylinder` and `make_cone` orient the longitudinal axis along **+Z** (`[2R, 2R, H]`).
   - `bench/tasks.json` (and the TS mock) expects the longitudinal axis along **+Y** (`[2R, H, 2R]`).
4. **`cut_hole` Parameter Contract (`T05`, `T06`, `T10`, `T13`)**:
   - `bench/tasks.json` passes `{"position": [x, y, 0], "direction": [0, 0, -1], "diameter": d, "through": true}` without `depth`.
   - Rust `crates/mcp-cad-server/src/tools/cad.rs:789` requires explicit numeric `depth` and cuts from `position` along `direction` (so starting at `z = 0` with `direction = [0, 0, -1]` only cuts the bottom half of a centered box!).
5. **`revolve` Annular Profile Contract (`T07`)**:
   - `bench/tasks.json` passes a 2-point line `profile: [[0.035, -0.04], [0.035, 0.04]]` plus `inner_radius: 0.015`.
   - Rust `crates/mcp-cad-server/src/tools/cad.rs:647` requires `profile.len() >= 3` and ignores `inner_radius`.
6. **`polygon_prism` Parameter Names (`T08`)**:
   - `bench/tasks.json` passes `outer_radius: 0.0092376` and `chamfer_radius: 0.0008`.
   - Rust `crates/mcp-cad-server/src/tools/cad.rs:692` requires `radius` or `width_across_flats` and `chamfer`.
7. **Spur Gear `module` Units (`T09`, `T14`)**:
   - `bench/tasks.json` passes `module: 2.0` (millimeters) while all other dimensions (`face_width: 0.025`, `bore_diameter: 0.012`, `expected_dimensions_m: [0.044, 0.044, 0.025]`) are in meters.
   - Rust `crates/mcp-cad-core/src/primitives.rs:795` interprets `module` strictly in meters (`pitch_diameter = teeth * module = 40.0 m`).
8. **`create_joint` Ignores Custom `joint_id` (`T14`, `T15`)**:
   - `bench/tasks.json` passes `"joint_id": "t14_rev_1"` and `"joint_id": "t15_joint_crank"` to `create_joint`, then references that ID in `set_joint_motor`.
   - `crates/mcp-cad-server/src/tools/physics.rs:264` ignores `args.get("joint_id")` and always generates `format!("joint-{}", Uuid::new_v4())`.
9. **FEA Verdict Field Name (`T11`, `T12`, `T13`)**:
   - `bench/tasks.json` checks `expected_fea.verdict = "SAFE"`.
   - Rust `run_fea_analysis` returns `status: "PASS"` or `"FAIL_YIELD"`.

### Category B: Geometry, FEA & Physics Engine Defects (`P3-*`)
1. **`cut_helical_groove` Produces Non-Watertight Mesh (`T08`)**:
   - Subtracting the multi-turn helical thread cutter in `crates/mcp-cad-core/src/dag.rs:676` (`csg_difference(&current, &cutter)`) leaves `413` open boundary edges (`is_watertight = false`, `genus = 68`).
2. **FEA Fixed-Face Poisson Effect & Linear `TET4` Voxel Meshing (`T11`, `T12`, `T13`)**:
   - `T11`: 3D clamping ($u_x = u_y = u_z = 0$) on `min_x` induces a local triaxial Poisson stress concentration at the fixed boundary corners ($\sigma_{\text{max}} = 21.30\text{ MPa}$ vs 1D $\sigma = F/A = 20.50\text{ MPa}$, `3.93%` diff) and slightly alters displacement (`2.00%` diff).
   - `T12`: `crates/mcp-cad-server/src/tools/fea.rs` ignores `element_type: "TET10"` (`crates/mcp-cad-fea` has no `Tet10Element`) and uses linear `TET4` elements, suffering ~20% shear locking in slender beam bending ($\sigma = 117.53\text{ MPa}$ vs $150.00\text{ MPa}$, $\delta = 0.7831\text{ mm}$ vs $0.9756\text{ mm}$).
   - `T13`: Cartesian grid voxelization (`max_cells = 60` along $0.60\text{ m}$ $\implies \Delta x = 10\text{ mm}$ for a hole of radius $20\text{ mm}$) staircase-aliases the circular Kirsch hole boundary, yielding $\sigma_{\text{max}} = 128.06\text{ MPa}$ vs analytical $157.44\text{ MPa}$ (`18.66%` error).
3. **Physics Gear Meshing & Slider-Crank Assembly (`T14`, `T15`)**:
   - `T14`: Both gears are created at `[0, 0, 0]` (overlapping colliders), `module: 2.0` creates $40\text{ m}$ and $80\text{ m}$ gears with huge rotational inertia so `max_force: 80` barely accelerates `t14_gear_1`, and Rapier3D has no kinematic gear-ratio constraint (`gear_joint`) linking `t14_rev_1` and `t14_rev_2`.
   - `T15`: `bench/tasks.json` creates `t15_crank`, `t15_connecting_rod`, and `t15_piston_slider`, but only creates two joints (`t15_joint_crank` on `t15_crank` and `t15_joint_slider` on `t15_piston_slider`) — `t15_connecting_rod` is completely unconnected! The TS mock (`src/cad/kinematics.ts`) faked the slider trajectory via a hardcoded closed-form formula whenever `partId.includes("slider")`.

---

## 6. How to Run

### Native Rust Integration Test Runner
```bash
cargo test -p mcp-cad-server --test bench_suite -- --nocapture
```

### TypeScript Stdio JSON-RPC Runner Against `mcp-cad-server --headless`
```bash
npx tsx bench/run_benchmark_rust.ts
```
