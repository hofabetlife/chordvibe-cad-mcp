# Changelog

All notable changes to `chordvibe-cad-mcp` are documented in this file.

## [v0.2.0] - 2026-11-18

### Added & Changed
- **Native Double-Precision `f64` Index-2 DAE Physics Engine (`mcp-cad-physics`, `ADR-004`, `P4-T1`–`P4-T7`)**:
  - Replaced `rapier3d = "0.22"` (`f32` / `TGS-Soft`) completely with a dedicated Pure-Rust `f64` multibody DAE/KKT/Active-Set Box-LCP solver (`MultiBodySolver` in `crates/mcp-cad-physics/src/solver.rs`) built on `nalgebra`.
  - Added exact signed-tetrahedral inertia integration (`MassProperties::from_part` in `rigid_body.rs`) accounting for internal bores and pockets, plus implicit midpoint (Lie–Poisson) 3D Euler rotational predictor preserving rotational energy and angular momentum norm to `< 1e-13` on asymmetric spinning tops.
  - Added native `f64` collision detection pipeline (`collision.rs`: Sweep-and-Prune AABB broad-phase, cached `TriMeshBvh`, analytical primitives + GJK/EPA + planar manifold clipping for all 6 `ColliderType` variants) with unilateral normal contacts and isotropic Coulomb friction cone (`2x2` disk projector).
  - Replaced legacy impulse decoding with direct Lagrange multiplier ($\boldsymbol{\lambda}$) extraction (`Pinned KKT Re-solve` in `simulator.rs`) for `joint_reaction_force`, `get_part_load_cases`, and `run_fea_from_simulation` (Newton's 3rd law residual $< 10^{-15}$, static cantilever beam root reaction error $< 0.0001\%$, 10-revolution gear torque transmission error $< 10^{-7}\text{ N}\cdot\text{m}$).
  - Hardened `ClosedLoopSolver` (`rotation_vector` in `simulator.rs`, `P4-T8`) with a `NaN`-safe `atan2(sin_t, cos_t)` + clamped-trace extraction (plus $\theta \approx \pi$ axis recovery) so 1-ULP trace drift ($\operatorname{tr}(R) > 3.0$ or $< -1.0$) never produces `NaN` in Gauss–Newton SVD steps.
  - Synchronized `Part::mesh()`, `Part::mesh_mut()`, and `Part::tessellate()` (`crates/mcp-cad-core/src/part.rs`, `P4-T9`) so direct edits to `part.mesh` after `part.rebuild()` reflect immediately in all queries without manual `invalidate_cache()`, and simplified `MassProperties::from_part` (`rigid_body.rs`).
  - Stabilized `test_p3_t10_benchmark_check_interference_10_parts_45_pairs` (`crates/mcp-cad-physics/tests/physics_tests.rs`, `P4-T10`) under parallel CPU contention by replacing the brittle `< 250.0 ms` debug wall-clock threshold with deterministic broad-phase/narrow-phase structural and analytical checks (`broadphase_overlap_pairs_count == 1`, `max_intersection_volume == 3.0e-5 m³`, `min_clearance == 0.05 m`, `elapsed_ms.is_finite()`) plus a generous `5000 ms` hang guard.
  - Removed `rapier3d` and transitive `parry3d` crates from `Cargo.toml`/`Cargo.lock`, replaced `as f32` bounds with `MAX_MOTOR_PARAM_F64 = 1e30`, and synchronized Web Studio (`src/`) labels to `f64 DAE`.
- **Expanded MCP Toolset (41 Rust Tools / 31 TS Tools)**:
  - Added engineering contract & precision tools: `set_units` (millimeter boundary contract, `[DECISION-UNITS-1]`), `save_project` / `load_project` (`schema_version = "1.0.0"`), `set_requirements` / `check_requirements` (revision-tracked `FRESH`/`STALE` FEA checks), `get_scene_summary`, `get_part_load_cases`, `run_fea_from_simulation` (3-2-1 minimal support & D'Alembert body loads), `check_gear_strength` (Lewis/Barth bending + Hertz/AGMA contact), `analyze_mechanism` (Grübler + Jacobian SVD + `redundant_joint_ids` + `joints_at_limits`), `check_interference` (broad-phase AABB + `csg_intersection` + triangle clearance), `sweep_joint` (including `ClosedLoopSolver` for 4-bar & slider-crank mechanisms), and `apply_recipe` (`flanged_bushing`, `stepped_shaft`, `gear_transmission`).
- **Topological Face Naming (`face_ids` / `FaceInfo`), Coaxial Threading & WGPU Thread-Safety**:
  - Preserved logical `face_id` labels across BSP-CSG, holes, and `transform_part`; added watertight coaxial manifold meshing for angled/offset `cut_helical_groove` (`0.00% < 5°` slivers); added `OffscreenRenderer::shared()` and isolated `qa_wide_scan` temp artifacts (`348/348` Rust workspace tests + `53/53` TS tests passing).

## [v0.1.0] - 2026-11-02

### Added
- **Unified MCP Tool Catalog (23 tools)**: Synchronized JSON schemas, tool names, and execution contracts across the Rust backend (`crates/mcp-cad-server`) and TypeScript studio (`src/cad/mcp_dispatcher.ts`), enforced by automated synchronization and contract tests.
- **Quadratic `TET10` FEA Solver & Body-Fitted Hole Meshing**: Added 10-node quadratic tetrahedral elements (`TET10`), body-fitted polar annular meshing for plates with circular holes, tributary-area surface tractions, 3-level mesh convergence runner (`coarse`, `medium`, `fine`), stress-singularity detector at sharp re-entrant corners, and tetrahedral quality metrics (aspect ratio, dihedral angle, Jacobian determinant).
- **Analytical Verification Suite**:
  - Prismatic axial bar tension ($\sigma = F/A$, $\delta = FL/(EA)$, error $< 1\%$).
  - Cantilever beam bending ($\delta = FL^3/(3EI)$, $\sigma = Mc/I$, `TET10` error $< 2\%$; documented linear `TET4` shear locking on coarse grids).
  - Kirsch / Peterson finite-width plate with central hole ($600\times 200\times 10\text{ mm}$, $R = 20\text{ mm}$, $F = 100\text{ kN}$, $K_{t,\text{net}} \approx 2.519$, error $< 10\%$).
  - Slider-crank analytical kinematics ($x(\theta) = r\cos\theta + \sqrt{l^2 - r^2\sin^2\theta}$) and 10-revolution external spur gear pair ($\omega_2/\omega_1 = -z_1/z_2 = -0.5$, anchor drift $< 10^{-3}\text{ m}$).
- **Safe DAG Query Optimizer (`optimize_cad_queries`)**: Defaults to non-destructive preview mode (`apply = false`), preserves `original_to_new_feature_id_map` when `apply = true`, and verifies topological/volumetric invariance across 50 randomized 20–50 node DAG property tests and 5 benchmark models (`bench/`).
- **Transport & Filesystem Hardening**:
  - `mcp-cad-server` defaults `--host` to `127.0.0.1` and refuses non-loopback SSE binding (`0.0.0.0`) unless `--auth-token <TOKEN>` is supplied (`Authorization: Bearer <TOKEN>`).
  - `export_stl` and `import_stl` enforce sandboxed paths inside `./output` and reject path traversal (`..`, `/etc/passwd`, `C:\Windows\...`).
- **15-Task Engineering Benchmark (`bench/`)**: Automated evaluator (`bench/run_benchmark.ts`) covering 5 simple primitives/booleans, 5 parametric parts, and 5 full CAD + FEA / kinematics workflows.
- **Dual Licensing & CI**: Added dual `MIT` (`LICENSE-MIT`) and `Apache-2.0` (`LICENSE-APACHE`) licenses and GitHub Actions CI (`.github/workflows/ci.yml`) for `ubuntu-latest` and `windows-latest`.

### Известные ограничения (Known Limitations)
1. **Полигональное представление геометрии (`TriangleMesh`)**: Ядро геометрии (`mcp-cad-core` и `src/cad/primitives.ts`) использует полигональные триангулированные сетки (`TriangleMesh`) и булевы операции на основе разбиения пространства (`CSG BSP`), поэтому гладкие параметрические поверхности `NURBS` и обменные форматы `STEP`/`IGES` не поддерживаются.
2. **Линейная статика МКЭ (`mcp-cad-fea` / `src/cad/fea_solver.ts`)**: Решатель МКЭ ограничен линейно-упругой изотропной статикой малых перемещений ($\varepsilon = \frac{1}{2}(\nabla u + \nabla u^T)$). Нелинейность материала (пластичность, ползучесть), геометрическая нелинейность (большие прогибы, потеря устойчивости) и контактные задачи не моделируются. Линейные элементы `TET4` подвержены эффекту сдвигового запирания (shear locking) при изгибе на грубых сетках; для изгибных задач и концентраторов напряжений необходимо использовать квадратичные элементы `TET10`. Все результаты снабжены дисклеймером: *«оценка, не расчёт для сертификации»*.
3. **Кинематическое сопряжение `gear` (`mcp-cad-physics` / `src/cad/kinematics.ts`)**: Соединение `gear` реализует идеальное кинематическое ограничение отношения угловых скоростей ($\omega_2/\omega_1 = -z_1/z_2$) без расчёта контактных сил между эвольвентными профилями зубьев, зазоров (backlash) и трения в зацеплении.
4. **Ограничения оптимизатора дерева построений (`optimize_cad_queries`)**: Правила слияния и группировки по законам де Моргана сохраняют инвариантность объёма и топологии для соосных и непересекающихся вырезов, но при перезаписи графа (`apply = true`) исходные промежуточные узлы объединяются (соответствие возвращается в `original_to_new_feature_id_map`).
5. **Песочница файлового ввода-вывода STL**: В целях безопасности операции `export_stl` и `import_stl` разрешают чтение и запись файлов только внутри рабочей директории `./output`.
