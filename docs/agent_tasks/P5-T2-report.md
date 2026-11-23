# Отчёт по задаче P5-T2: Гранично-согласованная сетка `TET10` (`fe_snap` из `ADR-0002`) и локальное сгущение в концентраторах напряжений

- **Дата:** 19.11.2026
- **Задача:** `P5-T2 [L]` — *Гранично-согласованная сетка `TET10` в продакшн-решателе (`fe_snap` из `ADR-0002` + локальное сгущение в галтелях, отверстиях и корнях зубьев)*
- **Статус:** ✅ **ВЫПОЛНЕНО**

---

## 1. Резюме изменений

1. **Новый чистый Rust-модуль гранично-согласованного мешинга (`crates/mcp-cad-fea/src/conforming_mesh.rs`, `#![deny(unsafe_code)]`, 0 C/C++ зависимостей):**
   - **2:1-сбалансированное октодерево с согласованными переходными тетраэдрами (`build_octree_conforming_tet4`, `triangulate_balanced_octree`):**
     - Автоматически обнаруживает зоны высокой кривизны и концентраторы напряжений (`detect_stress_concentrator_features`: кромки отверстий, галтели, корни зубьев шестерён, острые рёбра и зоны соприкосновения граней с высоким углом перегиба), а также поддерживает явные сферические зоны сгущения (`RefinementZone`) для узлов приложения нагрузок и опор шарниров.
     - Выполняет 2:1-балансировку ячеек (соседи по граням и рёбрам отличаются не более чем на 1 уровень) и детерминированную триангуляцию общих граней (каноническая диагональ Куна для регулярных ячеек + веерная/разделяющая триангуляция на переходных гранях), гарантируя **нулевое количество висячих узлов** (100% замкнутое 2-многообразие внешней оболочки).
     - Приоритетная сортировка разбиения по расстоянию до концентраторов обеспечивает равномерное разрешение сложных циклических профилей (например, всех 20 зубьев прямозубой шестерни).
   - **Граничный снаппинг `fe_snap` с жёстким барьером качества (`snap_mesh_to_surface`):**
     - `SurfaceSpatialIndex` (3D пространственный хеш треугольников CAD) и `SharpFeatureIndex` (острые рёбра и углы CAD с хранением пар нормалей сопрягаемых граней `(n_a, n_b)` для точной классификации `SharpCorner` / `SharpCrease` / `SmoothSurface` без ложного подтягивания внутренних ступенек отверстия на торец).
     - Итерационная проекция с backtracking line-search и строгим барьером нормированного якобиана Кнуппа $J_s \ge 0.06$ ($\det J > 0$), чередуемая с тангенциальной релаксацией на поверхности/рёбрах, внутренним лапласовским сглаживанием, многоцикловой полировкой границы и финальным проходом строгого улучшения качества низкокачественных граничных элементов без отрыва узлов от CAD-поверхности.
   - **Криволинейный квадратичный `TET10` (`to_tet10_boundary_snapped`) и контроль якобианов Гаусса (`crates/mcp-cad-fea/src/element.rs`):**
     - Добавлены `check_tet10_gauss_jacobians` и `tet10_min_gauss_jacobian_ratio` для проверки $\det J(\mathbf{L}^{(q)}) > 0$ во всех 4 точках интегрирования Гаусса.
     - Серединные узлы внешних рёбер проецируются на криволинейные CAD-грани и криволинейные острые кромки (ободки отверстий/расточек) с откатом шага при нарушении барьера отношения якобиана `min_tet10_gauss_jacobian_ratio = 0.45` и гарантии $\det J > 0$ во всех точках Гаусса.
   - **Режимы точности и метрики качества (`AccuracyMode`, `ConformingMeshOptions`, `MeshQualityMetrics`, `evaluate_mesh_metrics`):**
     - `AccuracyMode::Fast` — быстрая воксельная сетка `TET4`.
     - `AccuracyMode::Standard` — гранично-согласованная сетка (без окто-сгущения по умолчанию).
     - `AccuracyMode::High` — **стандартный режим точных инженерных расчётов**: локально сгущённое 2:1-октодерево + граничный снаппинг `fe_snap` + криволинейный `TET10` (`element_order = 2`).

2. **Точное квадратичное интегрирование давления на криволинейных гранях (`crates/mcp-cad-fea/src/boundary.rs`):**
   - Добавлена функция `integrate_conforming_surface_pressure`, выполняющая точную 6-точечную симметричную квадратуру Дунаванта 4-го порядка по криволинейным 6-узловым граничным треугольникам `T6` ($\mathbf{f}_k = \int_{\hat{T}} N_k^{(6)} (-p) (\mathbf{x}_\xi \times \mathbf{x}_\eta) \, d\xi d\eta$) с локальным допуском по длине ребра и вычислением истинного квадратичного центроида грани `T6`.
   - Исправлена функция `estimate_voxel_steps` для гранично-согласованных сеток (расчёт средней длины ребра по вершинам вместо минимальной декартовой разницы координат).

3. **Восстановление напряжений, сходимость и интеграция API/MCP:**
   - В `crates/mcp-cad-fea/src/result.rs` для криволинейных гранично-согласованных сеток `TET10` в качестве элементного порога `vm_elem` используется осреднённый по 4 точкам Гаусса тензор Коши в центроиде элемента, что исключает перебивание суперсходящихся сглаженных узловых напряжений на границе осцилляцией единичной внутренней точки Гаусса.
   - В `crates/mcp-cad-fea/src/tetra_mesh.rs`, `convergence.rs` и `lib.rs` добавлены `boundary_conforming: bool`, `from_mesh_conforming`, `from_mesh_high_accuracy`, `from_mesh_with_accuracy`, `from_part_with_accuracy`, `snap_to_surface`, `to_tet10_boundary_snapped`, `evaluate_quality`, `analyze_surface_mesh_with_accuracy`, `analyze_part_with_accuracy`, `run_fea_with_convergence_with_accuracy` (с отключением флага `voxel_staircase_detected` на согласованных сетках).
   - В `crates/mcp-cad-server/src/tools/fea.rs` и `tools/mod.rs` добавлен опциональный параметр `accuracy_mode: "fast" | "standard" | "high"` в инструменты `run_fea_analysis` и `run_fea_from_simulation`, с автоматическим добавлением локальных зон сгущения `RefinementZone` в точках позиционных нагрузок и якорей шарниров симуляции, а также возвратом полей `accuracy_mode` и `boundary_conforming` в JSON-ответе.

---

## 2. Измеренные результаты эталонных бенчмарков (`fea_benchmarks.rs`)

Проверено командой `cargo test -p mcp-cad-fea --test fea_benchmarks test_p5_t2 -- --nocapture` (после всех финальных правок):

| Эталонная задача | Сетка (`AccuracyMode::High`) | Элементов / Узлов | Перемещение (ошибка) | Напряжение $\sigma_{\text{vM}}$ (аналитика) | Ошибка $\sigma_{\max}$ | Порог приёмки | Качество сетки (`Js_min`, `rms`, `inv`, `gauss_ok`) | Время | Статус |
|---|---|---|---|---|---|---|---|---|---|
| **3b. Пластина с отверстием (Кирш/Петерсон)** ($600\times 200\times 10\text{ мм}$, $\varnothing 40\text{ мм}$, $F_x = 100\text{ кН}$) | Conforming Octree `TET10` (`Medium`) | `5 896` / `10 372` | `0.151288 мм` vs `0.149406 мм` (**1.26%**) | **`161.41 МПа`** vs `156.65 МПа` ($K_t = 2.583$ vs `2.506`) | **3.04%** *(было 16.19% на `TET4`)* | `<= 5.0%` | `Js_min = 0.060`, `rms = 0.0725 мм`, `inv = 0`, `gauss_ok = true` | `36.9 с` | ✅ PASS |
| **4b. Толстостенный цилиндр Ламе** ($a=20, b=40, L=40\text{ мм}$, $p=10\text{ МПа}$) | Conforming Octree `TET10` (`Medium`) | `23 390` / `35 334` | `0.001965 мм` vs `0.001919 мм` (**2.43%**) | **`25.14 МПа`** vs `23.33 МПа` ($\bar{\sigma}_{\text{bore}} = 21.78\text{ МПа}$) | **7.75%** *(было 38.58% на `TET4`)* | `<= 8.0%` | `Js_min = 0.060`, `rms = 0.0452 мм`, `inv = 0`, `gauss_ok = true` | `40.7 с` | ✅ PASS |
| **5. Прямозубая шестерня** ($m=3\text{ мм}, z=20, \alpha=20^\circ, b=15\text{ мм}$, отверстие $r=7.5\text{ мм}$) | Voxel `TET4` vs Conforming Octree `TET10` (`Coarse`) | Voxel: `768`/`276` → Conf: `22 954`/`36 418` | — | — | **Снижение RMS: 74.2%** (`1.5144 мм` → `0.3914 мм`) | `>= 70.0%` | `Js_min = 0.060`, `inv = 0`, `gauss_ok = true` | `21.9 с` | ✅ PASS |

> **Наблюдение по производительности:** На сетках `~23 000` квадратичных элементов `TET10` (`~35 000` узлов / `~105 000` степеней свободы) однопоточный PCG-решатель с диагональным предобусловливателем Якоби отрабатывает за `~37–41 с` в тестовом профиле (`opt-level = 2`). Многопоточная сборка (`rayon`) и прямой разреженный / AMG-предобусловленный решатель запланированы в задаче **`P5-T6`**.

---

## 3. Верификация тестами и линтером

Выполнено после всех финальных правок:
- `cargo test -p mcp-cad-fea --no-fail-fast`: **26/26 PASS** (`0 failed`):
  - `--lib`: **9/9 PASS** (включая новый `element::tests::test_tet10_gauss_jacobian_checks_straight_curved_and_inverted`);
  - `tests/fea_benchmarks.rs`: **3/3 PASS** (`122.51s`, включая `test_p5_t2_boundary_conforming_tet10_benchmarks`, `test_p3_t4_tet10_cantilever_benchmark` и `test_fea_accuracy_benchmarks_suite`);
  - `tests/fea_tests.rs`: **14/14 PASS** (`70.69s`, включая 3 новых теста `P5-T2`: `test_p5_t2_conforming_octree_zero_hanging_nodes_and_local_refinement`, `test_p5_t2_explicit_refinement_zone_increases_local_density`, `test_p5_t2_high_accuracy_and_convergence_api`).
- `cargo test -p mcp-cad-core --no-fail-fast`: **136/136 PASS** (`0 failed`).
- `cargo test -p mcp-cad-physics --no-fail-fast`: **42/42 PASS** (`0 failed`).
- `cargo test -p mcp-cad-server --lib test_parse_accuracy_mode_variants`: **1/1 PASS** (из `mcp-cad-server --lib` прошли все `22/22` чистых unit-теста; `44` тестов с `EngineState::new()` ожидаемо падают из-за отсутствия Vulkan ICD в headless-контейнере — проблема уже зафиксирована в `TASKS.md` как `P5-T9`).
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).

---

## 4. Затронутые файлы

- `crates/mcp-cad-fea/src/conforming_mesh.rs` *(новый)*
- `crates/mcp-cad-fea/src/element.rs`
- `crates/mcp-cad-fea/src/tetra_mesh.rs`
- `crates/mcp-cad-fea/src/boundary.rs`
- `crates/mcp-cad-fea/src/result.rs`
- `crates/mcp-cad-fea/src/convergence.rs`
- `crates/mcp-cad-fea/src/lib.rs`
- `crates/mcp-cad-fea/tests/fea_benchmarks.rs`
- `crates/mcp-cad-fea/tests/fea_tests.rs`
- `crates/mcp-cad-server/src/tools/fea.rs`
- `crates/mcp-cad-server/src/tools/mod.rs`
- `docs/agent_tasks/P5-T2-report.md` *(новый)*
- `docs/FEA_ACCURACY.md`, `docs/PROGRESS.md`, `docs/adr/0002-fe-mesher.md`, `TASKS.md`
