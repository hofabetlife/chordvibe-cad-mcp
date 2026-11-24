---
name: mcp-cad
description: >-
  Operational guide for AI agents using the MCP-CAD Engine server (43 tools):
  parametric 3D CAD (FeatureTree DAG, CSG), mesh topology verification, named
  faces, static FEA (TET4/TET10), rigid-body kinematics/dynamics (f64 DAE),
  coupled nonlinear dynamics (HHT-alpha, plasticity, contact, fracture),
  offscreen rendering, and STL/project I/O.
---

# MCP-CAD Agent Guide

> **CRITICAL**: **Do NOT read Rust source code in `crates/**/*.rs`.** All tool contracts, coordinate conventions, and rules are defined here and in MCP `tools/list` (consult [`docs/MCP_API_REFERENCE.md`](docs/MCP_API_REFERENCE.md) only if an edge-case schema detail is needed).

---

## 1. Core Rules (Read First)

1. **Units — millimeters (`mm`) by default**:
   - Linear dimensions, radii, coordinates, gear modules, pitches, and bounds use `mm` (override globally via `set_units` or per call via `"length_unit": "mm" | "m"`).
   - **STL exception**: `.stl` files on disk (`export_stl`, `import_stl`) always store vertex coordinates in **SI meters (`m`)**.
   - Physical units are **invariant** to `length_unit`: force `N`, torque/moment `N·m`, stress/pressure `MPa`, Young's modulus `GPa`, FEA displacement `mm`, mass `kg`, density `kg/m³`, CAD/joint angles `deg`, motor angular velocity `rad/s`, linear velocity `m/s`, time `s` / `ms`.
2. **Coordinates — right-handed, Z-up, centered at `[0, 0, 0]`**:
   - **`box` axis trap**: `width` is along **X**, `height` is along **Y**, `depth` is along **Z** (`Z in [-depth/2, +depth/2]`). Axial primitives (`cylinder`, `cone`, `polygon_prism`, `helix`) use `height` along **Z** (`Z in [-height/2, +height/2]`).
3. **FeatureTree DAG semantics**:
   - First `add_feature_primitive` creates the base solid; every subsequent `add_feature_primitive` on the same `part_id` **auto-unions (`csg_union`)** rather than overwriting.
   - `cut_hole` and `cut_helical_groove` **auto-subtract (`csg_difference`)**.
   - Across two distinct parts, use `boolean_union`, `boolean_difference`, or `boolean_intersection` (mutates `target_part_id` in place).
4. **Use named faces, not raw coordinates**:
   - Call `query_part_geometry` before FEA or alignment to get `faces` (`"left"`, `"right"`, `"cap_top"`, `"cap_bottom"`, `"side"`, `"hole_wall:feat_N"`, `face_id`), then bind constraints/loads by name or `where` filter.
5. **Mesh quality gate**:
   - Always call `validate_mesh` after booleans, `cut_hole`, or `cut_helical_groove`. Run `run_fea_analysis` and `export_stl` only when `status == "PASS"` (`is_valid_solid: true`, `fea_ready: true`).
6. **FEA staleness (`STALE`)**:
   - Any geometry edit (`add_feature_primitive`, `cut_hole`, `modify_feature`, `transform_part`, `boolean_*`) increments `part.revision` and marks prior FEA results `STALE`—re-run FEA. `tessellate_part` updates surface mesh density only and does **not** invalidate `revision` or FEA.
7. **Error recovery (`isError: true`)**:
   - `result.content[0].text` contains `{"error": {"code", "message", "param", "hint", "retryable", "details"}}`. Fix `error.param` using `error.hint` and `error.details` instead of retrying blindly.

---

## 2. Task Routing

| Goal | Recommended Call Sequence |
|---|---|
| Inspect existing scene | `get_scene_summary` (compact overview) or `list_parts` |
| Standard component/assembly | **`apply_recipe`** (`flanged_bushing` \| `stepped_shaft` \| `gear_transmission` \| `coil_spring`) |
| Helical coil spring / wire spiral | `add_feature_primitive` (`primitive_type: "helix"` / `"coil_spring"`) or `apply_recipe("coil_spring")` — never invert `cut_helical_groove` |
| Build a part atomically | `batch_cad_operations` (`create_part` → `set_requirements` → `add_feature_primitive` → `cut_hole`) |
| Axisymmetric part (shaft, flange, bowl) | `add_feature_primitive` with `primitive_type: "revolve"` (cleaner than stacking cylinders) |
| Check mesh & simplify long DAG | `validate_mesh` → `optimize_feature_tree` (`apply: true`) → `validate_mesh` |
| Static structural FEA (von Mises, FoS) | `validate_mesh` → `query_part_geometry` → `run_fea_analysis` → `get_fea_heatmap_image` |
| Analytical spur gear tooth strength | `check_gear_strength` (via `part_id` or `module`, `teeth`, `face_width`, `torque`) |
| Mechanism kinematics & dynamics | `create_part` (`is_fixed: true` frame) → `create_joint` → `analyze_mechanism` → `check_interference` / `sweep_joint` → `set_joint_motor` → `run_physics_simulation` |
| FEA under dynamic simulation loads | `run_physics_simulation` → `get_part_load_cases` → `run_fea_from_simulation` |
| Impact, plasticity, contact, fracture | `bake_coupled_simulation` → `query_simulation_frame` |
| Visual inspection (headless) | `capture_viewport` (geometry) / `get_fea_heatmap_image` (FEA stress heatmap) |
| Interactive desktop 3D window | `open_preview_window` |
| Persist / load / manufacture | `save_project` / `load_project` / `export_stl` / `import_stl` |

---

## 3. Playbooks

### A. Parametric Part, Requirements & Static FEA
Flow: `get_scene_summary` → `batch_cad_operations` → `validate_mesh` → `query_part_geometry` → `run_fea_analysis` → `check_requirements` → (on `FAIL`: `modify_feature` → repeat) → `get_fea_heatmap_image` → `export_stl` / `save_project`.

1. **Atomic create + requirements + geometry (`batch_cad_operations`)** (no nested batches; rolls back fully on error):
```json
{
  "operations": [
    { "tool": "create_part", "params": { "part_id": "bracket", "name": "Support Bracket", "material": "aluminum_6061_t6" } },
    { "tool": "set_requirements", "params": {
        "part_id": "bracket", "auto_check": true,
        "requirements": [
          { "id": "req_bbox", "type": "bbox_max", "dims": [120.0, 40.0, 30.0] },
          { "id": "req_mass", "type": "mass_max", "kg": 0.35 },
          { "id": "req_mesh", "type": "mesh_valid", "expected_genus": 1 },
          { "id": "req_fos",  "type": "min_safety_factor", "value": 2.0, "fea_case": "tip_load" }
        ]
    }},
    { "tool": "add_feature_primitive", "params": { "part_id": "bracket", "primitive_type": "box", "params": { "width": 100.0, "height": 20.0, "depth": 20.0 } } },
    { "tool": "cut_hole", "params": { "part_id": "bracket", "position": [30.0, 0.0, 0.0], "normal": [0.0, 0.0, 1.0], "radius": 5.0, "through": true } }
  ]
}
```

2. **Validate mesh & inspect planar faces** (`validate_mesh`, then `query_part_geometry`):
```json
{ "part_id": "bracket", "expected_genus": 1 }
```
```json
{ "part_id": "bracket", "where": { "kind": "plane" } }
```

3. **Run static FEA (`run_fea_analysis`)** (use `"element_order": 2` [TET10] or `"accuracy_mode": "high"` for bending to prevent TET4 shear locking; add `"convergence_study": true` for mesh convergence):
```json
{
  "part_id": "bracket",
  "fea_case": "tip_load",
  "mesh_density": "medium",
  "element_order": 2,
  "accuracy_mode": "standard",
  "fixed": [{ "face": "left" }],
  "loads": [{ "face": "right", "force": [0.0, -500.0, 0.0] }]
}
```
> `loads[]` supports `"force": [Fx, Fy, Fz]` (`N`, area-distributed), `"pressure": P` (`MPa`, positive pushes inward along `-normal`), and `"moment": [Mx, My, Mz]` (`N·m` about face centroid).

4. **Check requirements & render stress** (`check_requirements`, `get_fea_heatmap_image`):
```json
{ "part_id": "bracket" }
```
```json
{ "part_id": "bracket", "camera_preset": "isometric", "width": 800, "height": 600 }
```
If `min_safety_factor` fails, fetch `feature_id` via `get_feature_tree`, update dimensions with `modify_feature`, and re-run `validate_mesh` → `run_fea_analysis` → `check_requirements`.

### B. Mechanism Kinematics, Clearance & Simulation-Driven FEA
1. Create a fixed frame (`create_part` with `"is_fixed": true`) or reference built-in `"ground"` / `"world"` (**never** name a user part `"ground"` or `"world"`).
2. Create moving links (`is_fixed: false`) and position them (`transform_part`).
3. Connect with `create_joint` (`revolute`, `prismatic`, `spherical`, `fixed`, `gear`): `anchor_a`/`anchor_b` are local `[x, y, z]` in `length_unit`; `limits` are `deg` (`revolute`) or `length_unit` (`prismatic`); `gear` requires `axis_a`, `axis_b`, and `ratio` or `teeth_a` + `teeth_b`.
4. Verify mobility and clashes **before** running motors:
   - `analyze_mechanism`: `{}` (checks Grübler + Jacobian SVD `dof_numeric`, `redundant_joint_ids`, `joints_at_limits`, `floating_parts`)
   - `check_interference`: `{ "ignore_adjacent": true }`
   - `sweep_joint`: `{ "joint_id": "j_crank", "from": 0.0, "to": 180.0, "steps": 36, "ignore_adjacent": true }` (non-mutating sweep; reports `first_collision_value`, `lockup_value`, `safe_travel_range`, `min_clearance`)
5. Actuate & simulate (`set_joint_motor` works **only** on `revolute` [`rad/s`, `N·m`] or `prismatic` [`m/s`, `N`]):
```json
{ "joint_id": "j_crank", "target_velocity": 6.283, "max_force": 50.0 }
```
```json
{ "duration_seconds": 1.0, "step_size_ms": 10.0 }
```
6. Transfer peak joint reactions and inertial loads to FEA (`get_part_load_cases` optional; `run_fea_from_simulation` applies 3-2-1 inertial relief):
```json
{ "part_id": "crank_arm", "select": "peak_force", "mesh_density": "medium", "element_order": 2 }
```

### C. Coupled Nonlinear Dynamics (Impact, Plasticity, Contact, Fracture)
Use `bake_coupled_simulation` + `query_simulation_frame` for transient deformable dynamics (implicit `HHT-alpha`, J2 plasticity, continuum damage/erosion with fragments, Coulomb contact). With `"use_physics_joints": true` (default), active `PhysicsWorld` joints/motors are coupled automatically.
```json
{
  "action": "bake",
  "accuracy_level": "standard",
  "part_ids": ["beam_1"],
  "duration_s": 0.02,
  "dt_ms": 1.0,
  "enable_plasticity": true,
  "enable_fracture": true,
  "enable_contact": true,
  "fixed_faces": [{ "part_id": "beam_1", "face": "min_x" }],
  "loads": [{ "part_id": "beam_1", "face": "max_x", "force": [0.0, -5000.0, 0.0] }],
  "async_mode": false
}
```
> For `async_mode: true`, poll with `{"action": "status", "job_id": "..."}` or abort with `{"action": "cancel", "job_id": "..."}`.

Query frames with `query_simulation_frame` (use `"include_mesh_fields": false` for compact scalar/energy/fragment summaries):
```json
{ "frame_index": 10, "include_mesh_fields": false }
```

### D. Engineering Recipes (`apply_recipe`)
Atomically creates verified watertight parts and coupled joints with rollback on error:
- `flanged_bushing`: `sleeve_outer_diameter`, `sleeve_length`, `bore_diameter`, `flange_diameter`, `flange_thickness`, `flange_position` (`"bottom"|"top"|"center"`), `pcd_diameter`, `bolt_holes_count`, `bolt_hole_diameter`
- `stepped_shaft`: `steps: [{diameter, length}, ...]`, `keyway_step_index`, `keyway_width`, `keyway_depth`, `keyway_length`
- `gear_transmission`: fixed frame + pinion + driven gear + 2 `revolute` + 1 `gear` joint + optional motor (`module`, `teeth_pinion`, `teeth_gear`, `face_width`, `pressure_angle_deg`, `frame_thickness`, `frame_clearance`, `motor_velocity_rad_s`, `motor_max_torque_nm`)
- `coil_spring`: round-wire helical spring with engineering metrics (`C`, `Kw`, stiffness `k` in N/mm, solid height `Ls`, wire length `Lw`): `helix_radius` / `mean_diameter`, `wire_radius` / `wire_diameter`, `pitch`, `active_coils` / `free_length`, `is_right_hand`

---

## 4. 3D Primitives & Coordinate Bounds (`add_feature_primitive`)

All primitives center at `[0, 0, 0]`. Pass `"position": [x, y, z]` (`mm`) and `"rotation_deg": [rx, ry, rz]` (Euler RPY `deg`) inside `params` or at top level.

| `primitive_type` | `Z` Range | `XY` Cross-Section | Required `params` | Optional `params` & Notes |
|---|---|---|---|---|
| `box` | `[-depth/2, +depth/2]` | `X in [-width/2, +width/2]`, `Y in [-height/2, +height/2]` | `width`, `height`, `depth` | **Note**: `depth` is Z, `height` is Y. |
| `cylinder` | `[-height/2, +height/2]` | `x² + y² <= radius²` | `radius`, `height` | `segments` (default 32). `cap_bottom` at `-height/2`, `cap_top` at `+height/2`. |
| `cone` | `[-height/2, +height/2]` | Linear taper `radius_bottom` → `radius_top` | `radius_bottom`, `height` | `radius_top` (`0.0` for sharp tip), `segments`. Base at `-height/2`, top at `+height/2`. |
| `sphere` | `[-radius, +radius]` | `x² + y² + z² <= radius²` | `radius` | `sectors` (default 32), `stacks` (default 16). |
| `polygon_prism` | `[-height/2, +height/2]` | Regular N-gon | `height` + (`radius` \| `outer_radius` \| `width_across_flats`) | `sides` (default 6, DIN 934 hex), `chamfer` / `chamfer_radius` (30° end bevel). |
| `spur_gear` | `[-face_width/2, +face_width/2]` | Involute teeth (DIN 867 / ISO 53) | `module`, `teeth`, `face_width` | `pressure_angle_deg` (20°), `bore_radius`/`bore_diameter`, `keyway_width`/`keyway_depth` (DIN 6885), `hub_diameter`, `hub_length`. |
| `revolve` | From `z` in `profile` | 2D contour `[[r, z], ...]` rotated around `Z` (`r >= 0`) | `profile` | `angle_deg` (360°; `< 360°` caps ends), `segments`, `inner_radius` (auto-closes 2-pt outer line into ring). |
| `wheel_rim` | `[-rim_width/2, +rim_width/2]` | Stepped rim, spokes, hub, PCD | `rim_diameter`, `rim_width` | `flange_diameter`, `hub_diameter`, `hub_bore_diameter`, `pcd_diameter`, `bolt_holes_count` (5), `bolt_hole_diameter` (14), `spokes_count` (5), `spoke_width`, `spoke_thickness`, `offset_et` (35). Use full-scale (`rim_diameter >= 150 mm`). |
| `helix` (`coil_spring`) | `[-height/2, +height/2]` | Circular wire (`wire_radius`) swept along `helix_radius` | `helix_radius` (`coil_radius`/`mean_diameter`), `wire_radius` (`wire_diameter`) + any 2 of `pitch`, `height`/`free_length`, `turns`/`active_coils` | `starts` (1), `is_right_hand` (`true`). Invariants: `wire_radius < helix_radius`, `pitch / starts > 2 * wire_radius`. Faces: `cap_bottom`, `cap_top`, `side`. |

---

## 5. Named Faces & Selectors

`query_part_geometry` returns `faces[]` with `id`, `name`, `aliases`, `role`, `kind` (`plane` \| `cylinder` \| `cone` \| `sphere` \| `torus` \| `helical` \| `other`), `area`, `centroid`, `sample_point`, `normal_or_axis`, `radius`.

- **`box`**: `top` (`+Z`, aliases `cap_top`, `max_z`), `bottom` (`-Z`, `cap_bottom`, `min_z`), `right` (`+X`, `max_x`), `left` (`-X`, `min_x`), `back` (`+Y`, `max_y`), `front` (`-Y`, `min_y`).
- **`cylinder` / `cone` / `polygon_prism` / `helix`**: `cap_top` (`+Z`), `cap_bottom` (`-Z`), `side` (or `side_0..side_{n-1}` on prism).
- **`cut_hole`**: cylindrical wall has role `hole_wall` (`hole_wall:feat_N`, supports `"hole_wall:*"`); blind bottom is `hole_bottom`; keyway walls are `keyway_wall`.
- **`boolean_difference`**: cut surfaces get `cut:` prefix.

**Selector formats** (in `query_part_geometry`, `run_fea_analysis`, `transform_part`):
- Name/alias/wildcard: `{"face": "cap_bottom"}` or `{"face": "hole_wall:*"}`
- Numeric ID: `{"face_id": 3}`
- Multiple names: `{"faces": ["left", "bottom"]}`
- Filter `where`: `{"where": {"kind": "cylinder", "radius": 5.0, "tolerance": 0.2}}` — supported keys: `kind`, `role`, `origin_feature`, `normal`, `axis`, `radius`, `tolerance`, `near`, `closest`, `min_area`, `max_area`.
- On `AMBIGUOUS_FACE_SELECTOR`, qualify with `feat_N:<role>` or `face_id` from `error.details.available_faces`.

---

## 6. Materials (`create_part`, `define_material`)

Built-in presets (10):

| Canonical ID | Aliases | E (GPa) | Yield `σ_y` (MPa) | Density `ρ` (kg/m³) |
|---|---|---:|---:|---:|
| `aluminum_6061_t6` | `alu_6061`, `Al6061-T6`, `aluminum_6061` | 68.9 | 276 | 2700 |
| `steel_structural_aisi1020` | `steel_1020`, `steel`, `structural_steel` | 205.0 | 250 | 7850 |
| `steel_carbon_aisi1045` | `steel_1045`, `1045`, `carbon_steel_1045` | 205.0 | 350 | 7850 |
| `steel_high_strength_4140` | `steel_4140`, `AISI 4140`, `4140` | 210.0 | 655 | 7850 |
| `titanium_grade5_ti6al4v` | `ti_6al_4v`, `Ti-6Al-4V`, `titanium_ti6al4v` | 114.0 | 880 | 4430 |
| `brass_c360` | `brass_c36000`, `brass` | 100.0 | 310 | 8500 |
| `pla_filament_3dprint` | `pla_3d`, `pla_plastic`, `pla` | 3.5 | 60 | 1240 |
| `petg_filament_3dprint` | `petg_3d`, `petg_plastic`, `petg` | 2.1 | 50 | 1270 |
| `nylon_pa12` | `nylon_pa6`, `nylon` | 1.8 | 45 | 1010 |
| `borosilicate_glass` | `pyrex`, `glass`, `borosilicate` | 64.0 | 120 (brittle) | 2500 |

Custom materials via `define_material`: required `id`, `young_modulus_gpa`, `poisson_ratio` in `(0, 0.5)`, `yield_strength_mpa`, `density_kg_m3`; optional nonlinear `ultimate_strength_mpa` (or `ultimate_strength_pa`), `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`, `failure_mode` (`"ductile"|"brittle"`).

---

## 7. Complete 43-Tool Index (Full Schemas in `tools/list`)

- **Scene, Units & Projects (5)**: `get_scene_summary` (compact state/FEA/joints/requirements overview), `set_units` (`"mm"|"m"`), `save_project` (`schema_version: "1.0.0"`), `load_project` (transactional), `get_simulation_metrics_json` (full telemetry).
- **Parts & Materials (4)**: `create_part` (`"ground"`/`"world"` forbidden), `list_parts`, `delete_part`, `define_material`.
- **Parametric Modeling & CSG (11)**: `add_feature_primitive` (1st=base, next=auto `csg_union`), `cut_hole` (`through: true` or `depth`; optional DIN 6885 `keyway_width`/`keyway_depth`), `cut_helical_groove` (`profile_type: "circular"|"metric_60"`; `start_z` defaults to `bounding_box.min[2]`; `starts` defaults: 2 circular / 1 metric_60), `modify_feature` (bumps `revision`), `get_feature_tree`, `transform_part` (`translation`, `rotation_deg`, `scale`, `pivot_face`, `align_face`), `boolean_difference`, `boolean_union`, `boolean_intersection`, `batch_cad_operations` (atomic), `apply_recipe`.
- **Topology, Tessellation & Optimization (4)**: `validate_mesh` (`strict_slivers: false` default reports flute/thread micro-slivers as warnings with `PASS`), `query_part_geometry` (AABB, volume, CoM, `faces`), `tessellate_part` (`coarse|medium|fine` or tolerances; keeps `revision`), `optimize_feature_tree` (CBO rules R1–R5, dead-op culling, coaxial revolve merge, `<= 1%` volume guard).
- **Requirements Layer (2)**: `set_requirements` (`bbox_max`, `bbox_min`, `mass_max`, `mesh_valid`, `min_safety_factor`, `no_interference`, `mechanism_dof`; `auto_check: true`), `check_requirements` (`PASS|FAIL|STALE|NOT_AVAILABLE`).
- **Kinematics & Dynamics (8)**: `create_joint` (`revolute|prismatic|spherical|fixed|gear`), `set_joint_motor` (`revolute`: `rad/s`, `N·m`; `prismatic`: `m/s`, `N`), `run_physics_simulation`, `analyze_mechanism` (Grübler + Jacobian SVD `dof_numeric`), `check_interference`, `sweep_joint` (non-mutating loop solver), `check_gear_strength` (Lewis + Barth `Kv`, Hertz/AGMA/ISO 6336), `get_part_load_cases`.
- **FEA & Coupled Dynamics (4)**: `run_fea_analysis` (`TET4`/`TET10`, `accuracy_mode: "fast"|"standard"|"high"`, `convergence_study`), `run_fea_from_simulation` (3-2-1 inertial relief, default `element_order: 2`), `bake_coupled_simulation` (`action: "bake"|"status"|"cancel"`, `accuracy_level: "standard"|"high"|"ultra"`), `query_simulation_frame` (`include_mesh_fields`, `include_fragment_meshes`).
- **Visualization & STL I/O (5)**: `capture_viewport` (`debug_mode: "none"|"wireframe"|"normals"|"backfaces"|"boundary_edges"|"flat_shading"`), `get_fea_heatmap_image`, `open_preview_window` (in `stdio` without `--gui`, writes `./output/_mcp_preview_live.stl` and spawns `mcp-cad-server --stl`), `export_stl` (blocked if invalid mesh), `import_stl` (disk coordinates read as meters).

---

## 8. Common Pitfalls & Error Recovery

### Pitfalls
- **Repeated `add_feature_primitive`**: Does not replace dimensions; call `get_feature_tree` + `modify_feature` instead.
- **`import_stl` millimeter scale**: External STL exported in mm (e.g., `16×16×8`) imports as `16×16×8 m` and warns `SCALE_TOO_LARGE`; rescale via `transform_part` with `"scale": [0.001, 0.001, 0.001]`.
- **FEA visualization**: `capture_viewport` has no stress mode; always use `get_fea_heatmap_image`.

### Error Codes (`result.isError: true`)
| `error.code` | Cause | Action |
|---|---|---|
| `PART_NOT_FOUND` | Unknown `part_id` | Read `error.details.available_parts` or call `list_parts`. |
| `UNKNOWN_MATERIAL` | Unregistered material | Pick from §6 (`error.details.builtin_materials`) or call `define_material`. |
| `FACE_NOT_FOUND` / `AMBIGUOUS_FACE_SELECTOR` | Bad or non-unique face selector | Call `query_part_geometry` or use `name`/`face_id` from `error.details.available_faces`. |
| `MESH_INVALID` | Topology/manifold failure | Call `validate_mesh`, fix overlaps/thin walls, or simplify via `revolve` / `optimize_feature_tree`. |
| `FEA_UNCONSTRAINED` / `FEA_NO_LOADS` / `FEA_NO_BOUNDARY_NODES` | Missing BCs/loads or empty node match | Bind `fixed`/`loads` to faces from `query_part_geometry`; raise `mesh_density` to `"medium"`/`"fine"` on thin parts. |
| `CSG_BOOLEAN_ERROR` | Degenerate coplanar boolean contact | Offset tool `0.5–1 mm` past surface (use `cut_hole` with `"through": true` for bores). |
| `INVALID_JOINT_CONFIGURATION` | Zero axis, `min > max`, or motor on non-1-DOF joint | Supply non-zero `axis`; attach `set_joint_motor` only to `revolute` or `prismatic`. |
| `SIMULATION_NOT_FOUND` | Frame queried before bake | Run `bake_coupled_simulation` first. |
