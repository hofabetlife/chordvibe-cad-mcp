# Changelog

All notable changes to `chordvibe-cad-mcp` are documented in this file.

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
