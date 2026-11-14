# Отчёт о выполнении задачи P3-T2

## Задача
**P3-T2 [M]: Предполётные проверки и баланс сил FEA (`crates/mcp-cad-fea`, `crates/mcp-cad-server`)**

---

## 1. Что сделано (файлы, функции)

1. **`crates/mcp-cad-fea/src/error.rs`**:
   - Добавлен структурированный вариант ошибки `FeaError::InvalidSurfaceMesh { message, boundary_edges_count, non_manifold_edges_count, non_manifold_vertices_count, inconsistent_edges_count, is_watertight }`, сигнализирующий о незамкнутой или неманифольдной поверхностной сетке CAD-детали до запуска воксельной тетраэдризации.

2. **`crates/mcp-cad-fea/src/tetra_mesh.rs`**:
   - В `TetrahedralMesh::from_mesh` добавлена обязательная предполётная проверка `mcp_cad_core::validate_mesh(mesh, None)`. Если сетка не водонепроницаема (`!is_watertight`), содержит неманифольдные рёбра/вершины (`non_manifold_edges_count > 0` или `non_manifold_vertices_count > 0`), несогласованную ориентацию нормалей (`!orientation_consistent`) или неположительный объём (`!is_positive_volume`), генерация сетки немедленно отклоняется с `FeaError::InvalidSurfaceMesh`.
   - В `MeshDensity::target_element_size` нижний порог шага сетки снижен с `1e-4` м (`0.1 мм`) до `1e-5` м (`0.01 мм`), благодаря чему пресеты `Coarse`, `Medium` и `Fine` дают строго убывающий целевой размер элемента и строго возрастающее число тетраэдров даже на тонкостенных деталях (`min_dim <= 5 мм`, вплоть до субмиллиметровых пластин `0.4 мм`).

3. **`crates/mcp-cad-fea/src/solver.rs`**:
   - Реализована публичная функция `compute_nodal_reactions(mesh, elements, d_matrix, boundary, u_full) -> Vec<[f64; 3]>`, вычисляющая узловые реакции опор $\mathbf{R}_n = (K \cdot \mathbf{u})_n - \mathbf{f}_n = \left(\sum_e V_e B_e^T \boldsymbol{\sigma}_e\right)_n - \mathbf{f}_n$ по всем закреплённым степеням свободы Дирихле.
   - В `solve_pcg` для умеренных систем (`n <= 10_000` свободных степеней свободы) порог сходимости по относительной невязке установлен в `options.tolerance.min(1e-9)`, что гарантирует выполнение глобального баланса сил $\text{equilibrium\_error\_pct} < 10^{-4}\%$ без замедления крупных эталонных сеток (`n > 10_000`).

4. **`crates/mcp-cad-fea/src/result.rs` и `crates/mcp-cad-fea/src/lib.rs`**:
   - Добавлены структуры `ElementQualityStats { min, mean }` (нормированный критерий качества формы тетраэдра Лю–Джо $q_e = \frac{12 (3 V_e)^{2/3}}{\sum_{i<j} \ell_{ij}^2} \in [0, 1]$) и `FeaDiagnostics`:
     - `mesh_closed: bool` — замкнутость внешней поверхности тетраэдральной сетки (`boundary_edges == 0`);
     - `equilibrium_error_pct: f64` и `equilibrium_relative_error: f64` — невязка главного вектора сил $\|\sum \mathbf{R} + \sum \mathbf{F}\| / \max(\|\sum \mathbf{F}\|, \sum \|\mathbf{f}_i\|, 10^{-12}) \times 100\%$;
     - `moment_equilibrium_error_pct: f64` — невязка главного момента относительно центра масс;
     - `applied_total: [f64; 3]`, `reaction_total: [f64; 3]`, `applied_moment_total: [f64; 3]`, `reaction_moment_total: [f64; 3]`;
     - `constrained_rigid_body_modes_ok: bool` — проверка полноты закрепления всех 6 мод движения жёсткого тела в 3D через собственные числа кинематической матрицы Грама $G^T G \in \mathbb{R}^{6 \times 6}$;
     - `critical_node_near_constraint: bool` — детектор попадания `critical_node_stress` в 1-элементную окрестность закреплённых узлов (`constraint_zone_nodes`);
     - `max_stress_excluding_constraint_zone: f64` (в Па) и `max_stress_excluding_constraint_zone_mpa: f64` (в МПа) — максимальное напряжение по Мизесу вне 1-элементной зоны сингулярности заделки;
     - `element_quality: ElementQualityStats { min, mean }`;
     - `notes: Vec<String>` — инженерные диагностические предупреждения (`[CONSTRAINT_SINGULARITY]`, `[EQUILIBRIUM_RESIDUAL]`, `[RIGID_BODY_MODES]`, `[ELEMENT_QUALITY]`).
   - В `FeaReport` добавлены поля `confidence: String` (`"high" | "medium" | "low"`, с `#[serde(default)]`) и `diagnostics: FeaDiagnostics` (при сохранении `verdict: "PASS" | "FAIL"` без изменений). Уровень `confidence = "low"` выставляется при `equilibrium_error_pct > 1.0%`, `critical_node_near_constraint == true`, `!constrained_rigid_body_modes_ok`, `!mesh_closed` или `element_quality.min < 0.05`.

5. **`crates/mcp-cad-server/src/error.rs` и `crates/mcp-cad-server/src/tools/fea.rs`**:
   - Ошибка `FeaError::InvalidSurfaceMesh` сопоставлена со структурированным кодом `MESH_INVALID` (`param: "part_id"`, деталями `is_watertight`, `boundary_edges_count`, `non_manifold_edges_count`, `non_manifold_vertices_count`, `inconsistent_edges_count` и подсказкой `hint`).
   - В JSON-ответ инструмента `run_fea_analysis` добавлены поля `"confidence"`, `"diagnostics"`, `"max_stress_excluding_constraint_zone_mpa"` и `"warnings"`.

6. **Новые тесты**:
   - `crates/mcp-cad-fea/tests/fea_tests.rs`: добавлен интеграционный тест `test_p3_t2_preflight_mesh_validation_equilibrium_and_singularity_zone` (проверка отклонения незамкнутой сетки с `InvalidSurfaceMesh`, баланса сил и реакций на растяжении стержня `equilibrium_error_pct < 1e-4`, баланса сил/моментов и детекции сингулярности заделки на консоли, а также прогрессии плотности сетки на тонкостенной пластине `0.4 мм`).
   - `crates/mcp-cad-server/src/tools/fea.rs`: добавлен юнит-тест `test_p3_t2_run_fea_analysis_diagnostics_confidence_and_mesh_invalid` (проверка кода `MESH_INVALID` при открытых граничных рёбрах и структуры `diagnostics` / `confidence` в ответе `run_fea_analysis`).
   - Попутно исправлена опечатка вызова `export_stl_binary` в `crates/mcp-cad-server/src/tools/render.rs:174`.

---

## 2. Как проверено (команды и итоговые строки `test result`)

- Запуск юнит- и интеграционных тестов `mcp-cad-fea`:
  ```bash
  cargo test -p mcp-cad-fea --lib && cargo test -p mcp-cad-fea --test fea_tests
  ```
  Результат:
  ```text
  running 5 tests
  test result: ok. 5 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

  running 9 tests
  test test_axial_tension_analytical ... ok
  test test_factor_of_safety_pass_and_fail ... ok
  test test_mesh_edge_refinement ... ok
  test test_analyze_part_pipeline ... ok
  test test_adaptive_fea_part_pipeline ... ok
  test test_p2_t8_plate_with_hole_bearing_load_and_pressure_on_hole_wall ... ok
  test test_cantilever_beam_analytical ... ok
  test test_p3_t2_preflight_mesh_validation_equilibrium_and_singularity_zone ... ok
  test test_p2_t8_cylindrical_shaft_axial_tension_by_named_faces ... ok
  test result: ok. 9 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 1.01s
  ```

- Запуск юнит-тестов `mcp-cad-server::tools::fea` и линтера `clippy`:
  ```bash
  cargo test -p mcp-cad-server --lib tools::fea && cargo clippy -p mcp-cad-fea -p mcp-cad-server --all-targets -- -D warnings
  ```
  Результат:
  ```text
  running 2 tests
  test tools::fea::tests::test_p2_t16_fea_rejects_unrecognized_fixed_faces_and_empty_or_invalid_applied_forces ... ok
  test tools::fea::tests::test_p3_t2_run_fea_analysis_diagnostics_confidence_and_mesh_invalid ... ok
  test result: ok. 2 passed; 0 failed; 0 ignored; 0 measured; 48 filtered out
  ```

- Запуск TypeScript-суита:
  ```bash
  npx tsx --test src/cad/cad_engine.test.ts
  ```
  Результат:
  ```text
  # tests 39
  # suites 1
  # pass 39
  # fail 0
  ```

---

## 3. Сравнение числа тестов до и после

| Набор тестов | До P3-T2 | После P3-T2 | Дельта |
|---|---:|---:|---:|
| `mcp-cad-fea` (`lib` + `fea_tests` + `fea_benchmarks`) | 14 passed, 0 failed | **15 passed, 0 failed** | **+1** |
| `mcp-cad-server` (`lib` + интеграционные суиты) | 101 passed, 0 failed | **102 passed, 0 failed** | **+1** |
| Всего в Rust Workspace | 236 passed, 0 failed | **238 passed, 0 failed** | **+2** |
| TypeScript (`src/cad/cad_engine.test.ts`) | 39 passed, 0 failed | **39 passed, 0 failed** | 0 |

---

## 4. Остаточные риски и что сознательно не вошло в объём

1. **Автоматический прогон сходимости (`convergence_study: true`)**: в рамках **P3-T2** реализованы предполётный контроль топологии, баланс реакций опор, детектор сингулярности закрепления и оценка `confidence` на одиночной сетке; трёхшаговая оценка сходимости Ричардсона (`Coarse -> Medium -> Fine`) выделена в следующую задачу **P3-T3**.
2. **Квадратичные элементы `TET10` и сглаживание воксельной границы**: для снижения сеточной погрешности изгиба и ступенчатой концентрации напряжений предусмотрены отдельные задачи **P3-T4** (`TET10`) и **P3-T5** (проекция граничных узлов).

---

## 5. Побочные находки

- В `crates/mcp-cad-server/src/tools/render.rs:174` обнаружен и исправлен вызов несуществующей функции `mcp_cad_core::export_stl::export_mesh_stl_binary` (заменён на `mcp_cad_core::export_stl::export_stl_binary`), оставшийся после задачи **P2-T37**.
