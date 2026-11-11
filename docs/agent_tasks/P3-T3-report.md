# Отчёт о выполнении задачи P3-T3 — Оценка сходимости в результате FEA (`convergence_study: true`)

## 1. Summary
В `mcp-cad-fea` подключён и существенно доработан модуль `crates/mcp-cad-fea/src/convergence.rs` (`pub mod convergence;` в `crates/mcp-cad-fea/src/lib.rs`), а в инструмент `run_fea_analysis` (`crates/mcp-cad-server/src/tools/fea.rs`, `tools/mod.rs`) добавлена поддержка параметров `convergence_study: true`, `levels?: ["coarse", "medium", "fine"]` и лимита времени `max_seconds?: number`. В структуру `FeaReport` и JSON-ответ `run_fea_analysis` добавлено поле `accuracy: { converged, verdict, stress_change_pct, displacement_change_pct, effective_refinement_ratio, singularity_detected, voxel_staircase_detected, grid_clamped_detected, richardson_applicable, richardson_stress_mpa, richardson_displacement_mm, partial_result, max_seconds, total_study_time_ms, levels, recommendation }`. При вердикте `NOT_CONVERGED` (включая таймаут `partial_result: true`) поле `confidence` отчёта автоматически переводится в `"low"`, а в `diagnostics.notes` записывается диагностическое предупреждение `[MESH_CONVERGENCE]`.

## 2. Root Cause
1. **Мёртвый код `convergence.rs`**: файл `crates/mcp-cad-fea/src/convergence.rs` не был объявлен в `crates/mcp-cad-fea/src/lib.rs`, не вызывался из `run_fea_analysis` и не прикреплял метрики сходимости к `FeaReport`.
2. **Ложная сходимость наивного критерия `< 5% / < 10%` на пластине с центральным отверстием (`docs/FEA_ACCURACY.md` Benchmark 3)**:
   - В `TetrahedralMesh::from_mesh` (`tetra_mesh.rs:373-375`) число вокселей вдоль каждой оси ограничено `.clamp(2, 60)`. На пластине `600 × 200 × 10 мм` с отверстием $\varnothing 40\text{ мм}$ уже на уровне `Medium` ($h = 2.5\text{ мм}$) сетка в плоскости $(X, Y)$ упирается в потолок `60 × 60` (`85 440` тетраэдров), а переход к `Fine` ($h = 1.67\text{ мм}$) оставляет $(n_x, n_y) = (60, 60)$ неизменными и дробит только толщину $n_z: 4 \to 6$ (`128 160` тетраэдров). В результате эффективный 3D-коэффициент измельчения составляет $r_{\text{eff}} = (N_{\text{fine}} / N_{\text{medium}})^{1/3} = 1.145 < 1.25$, а кажущееся изменение между `Medium` и `Fine` оказывается искусственно малым ($\Delta\sigma = -0.46\%$, $\Delta u = -0.06\%$), хотя истинная погрешность пикового напряжения относительно аналитического решения Петерсона (`156.65 МПа`) составляет **16.6%–18.8%**.
   - Даже при отсутствии упора в лимит 60 ячеек декартова воксельная сетка аппроксимирует криволинейные стенки отверстий и полостей ступенчатой границей (`staircase`), дающей немонотонную осцилляцию пикового напряжения (`127.85 -> 131.29 -> 130.68 МПа`) и систематическую ошибку коэффициента концентрации $K_t$ в `16%–39%`.

## 3. Changes Made
- **`crates/mcp-cad-fea/src/convergence.rs`**:
  - Введены документированные константы порогов:
    - `CONVERGENCE_DISP_TOLERANCE_PCT = 5.0` (`5.0%` по перемещению);
    - `CONVERGENCE_STRESS_TOLERANCE_PCT = 10.0` (`10.0%` по напряжению Мизеса);
    - `MIN_EFFECTIVE_REFINEMENT_RATIO = 1.25` (минимальное реальное 3D-измельчение $r_{\text{eff}} = (N_{\text{last}}/N_{\text{prev}})^{1/3}$ для защиты от насыщения воксельной сетки на лимите `60`);
    - `CURVED_STAIRCASE_STRESS_RATIO_THRESHOLD = 1.50` (порог $\sigma_{\max}/\sigma_{\text{mean}}$ для детекции криволинейного концентратора напряжений на ступенчатой воксельной сетке).
  - Расширена структура `MeshConvergenceStep` полями `density`, `max_displacement_mm`, `max_von_mises_mpa`, `computation_time_ms`.
  - Добавлена структура `FeaAccuracy` и метод `MeshConvergenceStudyReport::to_fea_accuracy()`.
  - Реализована двухшаговая не-равномерная экстраполяция Ричардсона `compute_richardson_value` с проверкой математических условий применимости (`richardson_applicable`: минимум 3 уровня или 2 уровня с $r_{\text{eff}} \ge 1.25$, отсутствие сингулярности и ступенчатого концентратора на криволинейной границе, монотонность $(\phi_2-\phi_1)(\phi_3-\phi_2) > 0$ и убывание шага $|\Delta_{23}| < |\Delta_{12}|$ в асимптотической области).
  - Реализована функция `run_fea_with_convergence` с поддержкой пересчёта граничных условий на каждом уровне сетки и бюджета времени `max_seconds` (при превышении возвращается частичный результат с `partial_result: true` и `verdict: "NOT_CONVERGED"`).
- **`crates/mcp-cad-fea/src/result.rs` и `crates/mcp-cad-fea/src/lib.rs`**:
  - Подключён модуль `pub mod convergence;` и реэкспортированы публичные типы/функции.
  - В `FeaReport` добавлено поле `pub accuracy: Option<FeaAccuracy>` и метод `FeaReport::with_accuracy(...)`, автоматически понижающий `confidence` до `"low"` при `!accuracy.converged`.
- **`crates/mcp-cad-server/src/tools/fea.rs` и `crates/mcp-cad-server/src/tools/mod.rs`**:
  - В схему и обработчик `run_fea_analysis` добавлены параметры `convergence_study`, `levels` (по умолчанию `["coarse", "medium", "fine"]`) и `max_seconds`.
  - В JSON-ответ `run_fea_analysis` добавлены объекты `accuracy` и `convergence_study`.
- **Тесты и документация**:
  - Добавлены юнит-тесты в `crates/mcp-cad-fea/src/convergence.rs`, интеграционный тест `test_p3_t3_convergence_study_axial_cantilever_plate_with_hole_and_timeout` в `crates/mcp-cad-fea/tests/fea_tests.rs` и тест MCP-инструмента `test_p3_t3_run_fea_analysis_convergence_study_and_timeout` в `crates/mcp-cad-server/src/tools/fea.rs`.
  - Обновлены `docs/MCP_API_REFERENCE.md`, `TASKS.md` и `PROJECT_CONTEXT.md`.

## 4. Verification
- `cargo test -p mcp-cad-fea` — все юнит- и интеграционные тесты (`convergence::tests`, `fea_tests`, включая `test_p3_t3_convergence_study_axial_cantilever_plate_with_hole_and_timeout`) проходят.
  - На стержне при осевом растяжении (`Axial Tension`): `verdict == "CONVERGED"`, `converged == true`.
  - На консольной балке (`Cantilever Beam 200×20×20 мм`): `verdict == "NOT_CONVERGED"`, `confidence == "low"`, `richardson_applicable == true` (экстраполированные $\sigma_{\text{extrap}}$ и $u_{\text{extrap}}$ сходятся к аналитическим `150 МПа` и `0.9756 мм`).
  - На пластине с центральным отверстием (`Perforated Plate with Hole 600×200×10 мм, d=40 мм` и `120×40×10 мм, d=8 мм`): детекторы `grid_clamped_detected` и `voxel_staircase_detected` предотвращают ложный `CONVERGED`, выставляя `verdict == "NOT_CONVERGED"` и `confidence == "low"` в полном соответствии с `docs/FEA_ACCURACY.md`.
  - При `max_seconds: 0.0`: возвращается `partial_result == true`, `verdict == "NOT_CONVERGED"`, `confidence == "low"`.
- `cargo test -p mcp-cad-server --lib tools::fea` — все тесты FEA-инструмента (`test_p3_t3_run_fea_analysis_convergence_study_and_timeout` и др.) проходят.
- `cargo clippy -p mcp-cad-fea -p mcp-cad-server --all-targets -- -D warnings` — 0 предупреждений.

## 5. Impact on Downstream Tasks
- **P3-T4 (Квадратичные тетраэдры TET10)** и **P3-T5 (Поверхностно-согласованная сетка в `mcp-cad-fea`)**: могут напрямую использовать `run_fea_with_convergence` и `FeaAccuracy` для проверки того, что после внедрения TET10 и boundary-fitted сетки консольная балка и пластина с отверстием переходят из `NOT_CONVERGED` в `CONVERGED` с погрешностью $< 5\%$ по перемещениям и $< 10\%$ по напряжениям.
- **P3-T6 (`verify_requirements` честный статус)**: может опираться на `report.accuracy` и `report.confidence == "low"` при проверке прочностных требований.
