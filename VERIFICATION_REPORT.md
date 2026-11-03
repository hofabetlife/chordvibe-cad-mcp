# ВЕРИФИКАЦИОННЫЙ ОТЧЁТ: `hofabetlife/chordvibe-cad-mcp`

- **Репозиторий:** `https://github.com/hofabetlife/chordvibe-cad-mcp`
- **Проверенный коммит (оригинал `main`):** `02c7339` (`fix(server): resolve TS2554 argument mismatch in revolved boss dispatcher`)
- **Ветка с минимальными фиксами компиляции:** `verify/fix-compile` (`fb2dde0`)
- **Окружение верификации:**
  - `cargo 1.99.1 (5f94df478 2026-08-27)` / `rustc 1.99.1 (58cf8f9c3 2026-10-13)`
  - `gcc (Ubuntu 12.3.0-1ubuntu1~22.04.3) 12.3.0`, `pkg-config 1.8.1`
  - `node v22.23.3`, `npm 10.9.9`
  - Headless GPU: Mesa Lavapipe (`lvp_icd.x86_64.json`, `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1`)

---

## 1. Вердикт одной строкой

**На оригинальном `HEAD` (`02c7339`) и ни на одном из всех 15 коммитов истории Rust-workspace НЕ компилируется; после минимальной правки 3 файлов (ветка `verify/fix-compile`, коммит `fb2dde0`) проект собирается, MCP-сервер реально стартует (`stdio` и `sse`), отдаёт 28 инструментов и считает бокс/сферу/STL/FEA, но из 126 Rust-тестов проходят 108 и падают 18 (включая регрессию вывернутых крышек цилиндра/конуса/шестерни и неводонепроницаемый CSG), `cargo clippy -D warnings` падает (6 ошибок), а из 26 фронтенд-тестов проходит 25 и падает 1.**

---

## 2. Сводная таблица по крейтам и подсистемам

> **Важно:** В таблице ниже результаты тестов приведены для ветки `verify/fix-compile` (`fb2dde0`), так как на оригинальном `main` (`02c7339`) `mcp-cad-core` не компилируется (`9 errors`), блокируя сборку `mcp-cad-physics`, `mcp-cad-fea`, `mcp-cad-server` и всего workspace.

| Крейт / Подсистема | На `HEAD` (`02c7339`) | Таргет на `verify/fix-compile` (`fb2dde0`) | Passed | Failed | Ignored | Не запускалось | Команда и ключевая строка вывода |
| :--- | :--- | :--- | :---: | :---: | :---: | :---: | :--- |
| **`mcp-cad-core`** | ❌ Не компилируется (9 ошибок) | `--lib` | 38 | **6** | 0 | 0 | `test result: FAILED. 38 passed; 6 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.25s` |
| **`mcp-cad-core`** | ❌ Блокирован `lib` | `--test dag_tests` | 10 | **4** | 0 | 0 | `test result: FAILED. 10 passed; 4 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.20s` |
| **`mcp-cad-core`** | ❌ Блокирован `lib` | `--test tessellation_tests` | 5 | **1** | 0 | 0 | `test result: FAILED. 5 passed; 1 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.01s` |
| **`mcp-cad-core`** | ❌ Блокирован `lib` | `--doc` | 0 | 0 | 0 | 0 | `test result: ok. 0 passed; 0 failed; 0 ignored` |
| **`mcp-cad-physics`** | ❌ Блокирован `mcp-cad-core` | `--lib` / `--test physics_tests` / `--doc` | 6 | 0 | 0 | 0 | `test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.26s` |
| **`mcp-cad-fea`** | ❌ Блокирован `mcp-cad-core` | `--lib` (5) + `--test fea_tests` (6) + `--doc` (0) | 11 | 0 | 0 | 0 | `test result: ok. 5 passed; 0 failed` / `test result: ok. 6 passed; 0 failed; finished in 0.02s` |
| **`mcp-cad-viewer`** | ✅ Компилируется отдельно | `--lib` (0) + `--test viewer_tests` (10) + `--doc` (0) | 10 | 0 | 0 | 0 | `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-viewer` → `test result: ok. 10 passed; 0 failed; finished in 1.13s` |
| **`mcp-cad-server`** | ❌ Блокирован `mcp-cad-core` + 2 ошибки в тесте | `--test gear_kinematic_test` | 1 | 0 | 0 | 0 | `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.05s` |
| **`mcp-cad-server`** | ❌ 2 ошибки `EngineState::new_shared()` | `--test mcp_contract_tests` | 19 | **2** | 0 | 0 | `test result: FAILED. 19 passed; 2 failed; 0 ignored; 0 measured; 0 filtered out; finished in 1.88s` |
| **`mcp-cad-server`** | ❌ Блокирован `mcp-cad-core` | `--test qa_wide_scan` | 8 | **5** | 0 | 0 | `test result: FAILED. 8 passed; 5 failed; 0 ignored; 0 measured; 0 filtered out; finished in 5.83s` |
| **ИТОГО Rust Workspace** | ❌ Не собирается | `cargo test --workspace --no-fail-fast` | **108** | **18** | **0** | **0** | **108 passed / 18 failed / 0 ignored (всего 126 тестов)** |
| **Rust Clippy** | ❌ Не собирается | `cargo clippy --workspace -- -D warnings` | — | **6 ошибок** | — | — | `error: could not compile 'mcp-cad-core' (lib) due to 6 previous errors` |
| **Rust Release Build** | ❌ Не собирается | `cargo build --release -p mcp-cad-server` | ✅ | 0 | — | — | `Finished release profile [optimized] target(s) in 1m 47s` (бинарь 19 МБ) |
| **Frontend (`tsc` / `vite`)** | ✅ На `02c7339` | `npm run lint` && `npm run build` | ✅ | 0 | — | — | `tsc --noEmit` (0 ошибок), `vite build` → `built in 6.76s` (`dist/assets/index-BVcQBMi9.js 1,335.74 kB`) |
| **Frontend Tests (`tsx`)** | ❌ 1 падает | `npx tsx --test src/cad/cad_engine.test.ts` | **25** | **1** | **0** | **0** | `# tests 26 / # pass 25 / # fail 1` (`AssertionError: 5 !== 4` на строке 507) |

---

## 3. Приоритизированный список проблем

### 3.1. Блокеры (не собирается на `HEAD` / не работает базовая функциональность)

#### [BLOCKER-1] Ни один из 15 коммитов репозитория не компилирует `mcp-cad-core`
- **Где:**
  1. `crates/mcp-cad-core/src/query_optimizer.rs:1331` — во всех 15 коммитах истории (`38d6683..02c7339`) в конструкторе `CadOptimizationReport` используется несуществующая переменная `applied`.
  2. `crates/mcp-cad-core/src/primitives.rs:2601-2605` — в последних 9 коммитах (`6db3344..02c7339`) используются несуществующий тип `Winding::Cw` / `Winding::Ccw` и неверная арность вызовов `add_halving_ring_transition_2d` (передано 7 аргументов вместо 6) и `add_bridged_ring_strip_2d` (передано 8 аргументов вместо 6).
  3. `crates/mcp-cad-core/src/primitives.rs:2359` — замыкание `let mut push_quad = |...|` захватывает `&mut vertices` и `&mut indices`, удерживая мутабельное заимствование до строки 2650 и вызывая `error[E0499]: cannot borrow 'vertices'/'indices' as mutable more than once at a time` на строках 2601–2605, когда типы аргументов исправлены.
  4. `crates/mcp-cad-server/tests/mcp_contract_tests.rs:2335, 2401` — вызывается несуществующий метод `EngineState::new_shared()`, тогда как в `crates/mcp-cad-server/src/state.rs:26-41` определены `EngineState::new() -> Result<Self>` и `into_shared(self) -> SharedState`.
- **Доказательство (раскопки git-истории, Шаг 1):**
  ```text
  $ git log --oneline -n 20
  02c7339 fix(server): resolve TS2554 argument mismatch in revolved boss dispatcher
  dd4d088 feat(qa): complete Round 2 verification, fix BUG-016..020, and archive Round 1
  37c6670 test(qa): execute Round 2 verification matrix and update QA state [QA-R2]
  fd13a43 fix(core): resolve CSG union overlap, hole needle triangles, and scale normals [BUG-019, BUG-020]
  9800a44 fix(core): enforce revolve profile validation and 2-to-1 radial cap rings [BUG-016, BUG-017]
  6db3344 fix(core): eliminate sliver triangles in cylinder, cone, and gear caps [BUG-018]
  c54b396 docs(qa): finalize verification reports, mesh audits, and bug tracker
  d6e35c6 test(qa): add wide-scan QA integration suite, benchmarks, and visual artifacts
  e2b1d70 feat(ui): upgrade React CAD Studio with FEA, CBO, and mesh diagnostics panels
  761d947 feat(server): register geometry query, boolean, validation, and CBO MCP tools
  8763de9 feat(core): add rule-based and cost-based CAD DAG query optimizer (CBO)
  a49c00e feat(core): add B-Rep topology validator and mesh quality diagnostics
  0073ac9 feat(core): implement BSP-tree CSG booleans, STL import, and part transforms
  963148c feat(core): expand parametric primitives with cone, torus, extrude, revolve, loft, prism
  38d6683 Initial commit: MCP CAD Engine workspace, physics, FEA, viewer, and web studio

  # На коммитах 38d6683..c54b396 (первые 6 коммитов):
  $ CARGO_TARGET_DIR=/tmp/cargo_target cargo check -p mcp-cad-core
  error[E0425]: cannot find value `applied` in this scope
      --> crates/mcp-cad-core/src/query_optimizer.rs:1331:9
       |
  1331 |         applied,
       |         ^^^^^^^ help: a local variable with a similar name exists: `applied_rules`
  error: could not compile `mcp-cad-core` (lib) due to 1 previous error

  # На коммитах 6db3344..02c7339 (последние 9 коммитов, включая HEAD):
  $ CARGO_TARGET_DIR=/tmp/cargo_target cargo check -p mcp-cad-core
  error[E0433]: failed to resolve: use of undeclared type `Winding` (x4 on primitives.rs:2601-2605)
  error[E0061]: this function takes 6 arguments but 7 arguments were supplied (x2 on primitives.rs:2601,2604)
  error[E0061]: this function takes 6 arguments but 8 arguments were supplied (x2 on primitives.rs:2602,2605)
  error[E0425]: cannot find value `applied` in this scope (query_optimizer.rs:1331)
  error: could not compile `mcp-cad-core` (lib) due to 9 previous errors
  ```
- **Рекомендация по починке:** Применить минимальный патч из ветки `verify/fix-compile` (`fb2dde0`), приведённый в разделе 4.

---

#### [BLOCKER-2] Вывернутая ориентация треугольников (winding) крышек цилиндра, конуса и прямозубой шестерни (ломает 14 из 18 падающих Rust-тестов)
- **Где:** `crates/mcp-cad-core/src/primitives.rs:159-215` (функции `add_bridged_ring_strip_2d` на строках `174-179` и `add_halving_ring_transition_2d` на строках `207-211`).
- **Суть дефекта:** В коммите `6db3344` (`fix(core): eliminate sliver triangles in cylinder, cone, and gear caps [BUG-018]`) автор переписал крышки примитивов на концентрические кольца `add_bridged_ring_strip_2d` и `add_halving_ring_transition_2d`, передавая внешнее кольцо как `ring_a`, а внутреннее — как `ring_b`. Для вектора от внутреннего кольца к внешнему $\vec{u} = a_0 - b_0$ и касательного вектора против часовой стрелки $\vec{v} = a_1 - a_0$ векторное произведение $\vec{u} \times \vec{v}$ направлено вдоль **$-Z$**, тогда как в `if is_top` (строки 174 и 207) используются тройки `(b0, a0, a1)` и `(a0, a1, b0)`, нормаль которых смотрит внутрь тела ($-Z$ для верхней крышки и $+Z$ для нижней). В результате:
  - все внешние кольца крышек цилиндров, конусов, усечённых конусов и шестерён имеют инвертированные нормали;
  - знаковый объём крышек вычитается из объёма боковой поверхности вместо сложения (`volume = 11.83` вместо `35.34` у цилиндра $R=1.5, H=5$; `0.034` вместо `56.55` у конуса $R=3, H=6$; `36.33` вместо `65.97` у усечённого конуса);
  - `validate_mesh` фиксирует `96 inverted (non-manifold) directed edges` на каждом цилиндре и `72` на шестерне.
- **Какие именно 14 тестов падают из-за этого:**
  1. `mcp-cad-core --lib`: `diagnostics::tests::test_validate_mesh_cylinder_pass` (`crates/mcp-cad-core/src/diagnostics.rs:640`)
  2. `mcp-cad-core --lib`: `primitives::tests::test_cylinder_volume` (`crates/mcp-cad-core/src/primitives.rs:2803`)
  3. `mcp-cad-core --lib`: `primitives::tests::test_cone_sharp_volume` (`crates/mcp-cad-core/src/primitives.rs:2902`)
  4. `mcp-cad-core --lib`: `primitives::tests::test_cone_frustum_volume` (`crates/mcp-cad-core/src/primitives.rs:2912`)
  5. `mcp-cad-core --lib`: `primitives::tests::test_spur_gear_solid_volume_and_dimensions` (`crates/mcp-cad-core/src/primitives.rs:2978`)
  6. `mcp-cad-core --lib`: `query_optimizer::tests::test_r4_coaxial_bushing_cylinder_hole_to_revolve` (`crates/mcp-cad-core/src/query_optimizer.rs:1463`)
  7. `mcp-cad-core --test dag_tests`: `test_primitive_volumes_analytical_verification` (`crates/mcp-cad-core/tests/dag_tests.rs:62`)
  8. `mcp-cad-core --test dag_tests`: `test_dag_cone_and_frustum_volume` (`crates/mcp-cad-core/tests/dag_tests.rs:125`)
  9. `mcp-cad-core --test dag_tests`: `test_dag_shifted_cylinder_stepped_shaft` (`crates/mcp-cad-core/tests/dag_tests.rs:163`)
  10. `mcp-cad-core --test dag_tests`: `test_dag_helical_groove_drill_bit_model` (`crates/mcp-cad-core/tests/dag_tests.rs:344`)
  11. `mcp-cad-core --test tessellation_tests`: `test_cylinder_adaptive_tessellation_density_and_convergence` (`crates/mcp-cad-core/tests/tessellation_tests.rs:42`)
  12. `mcp-cad-server --test mcp_contract_tests`: `test_mcp_optimize_feature_tree_cbo` (`crates/mcp-cad-server/tests/mcp_contract_tests.rs:2268`)
  13. `mcp-cad-server --test qa_wide_scan`: `test_bug_002_spur_gear_manifold_and_keyway` (`crates/mcp-cad-server/tests/qa_wide_scan.rs:886`)
  14. `mcp-cad-server --test qa_wide_scan`: `test_bug_007_negative_scale_preserves_outward_winding` (`crates/mcp-cad-server/tests/qa_wide_scan.rs:1330`)
- **Доказательство (вывод `cargo test`):**
  ```text
  ---- diagnostics::tests::test_validate_mesh_cylinder_pass stdout ----
  thread 'diagnostics::tests::test_validate_mesh_cylinder_pass' panicked at crates/mcp-cad-core/src/diagnostics.rs:640:9:
  assertion `left == right` failed: Cylinder diagnostic errors: ["Mesh has 96 inverted (non-manifold) directed edges (inconsistent triangle winding)"]
    left: Fail
   right: Pass

  ---- primitives::tests::test_cylinder_volume stdout ----
  thread 'primitives::tests::test_cylinder_volume' panicked at crates/mcp-cad-core/src/primitives.rs:2803:9:
  Cylinder volume 11.833075031796574 != expected 35.34291735288517

  ---- primitives::tests::test_cone_sharp_volume stdout ----
  thread 'primitives::tests::test_cone_sharp_volume' panicked at crates/mcp-cad-core/src/primitives.rs:2902:9:
  Sharp cone volume 0.03425465980333134 vs expected 56.548667764616276 (err 0.9993942446063406)

  ---- primitives::tests::test_cone_frustum_volume stdout ----
  thread 'primitives::tests::test_cone_frustum_volume' panicked at crates/mcp-cad-core/src/primitives.rs:2912:9:
  Frustum volume 36.32668991939109 vs expected 65.97344572538566 (err 0.44937406651070024)

  ---- primitives::tests::test_spur_gear_solid_volume_and_dimensions stdout ----
  thread 'primitives::tests::test_spur_gear_solid_volume_and_dimensions' panicked at crates/mcp-cad-core/src/primitives.rs:2978:9:
  assertion failed: report.is_manifold
  ```
- **Рекомендация по починке:** В `crates/mcp-cad-core/src/primitives.rs` в функциях `add_bridged_ring_strip_2d` (строка 174) и `add_halving_ring_transition_2d` (строка 207) поменять местами ветки `if is_top` и `else` (либо вызывать `if !is_top`), а в `create_wheel_rim` (строки 2601–2605, где `outer` и `inner` передаются в обратном порядке) инвертировать булев флаг `is_top`.

---

#### [BLOCKER-3] Булевы операции CSG (`boolean_difference`, `cut_hole`) выдают неводонепроницаемую сетку с тысячами открытых границ и «игольчатых» треугольников
- **Где:** `crates/mcp-cad-core/src/csg.rs` (функции `split_polygon`, `triangulate_polygon`, `csg_difference`, `cut_hole_with_settings`) и тест `crates/mcp-cad-server/tests/qa_wide_scan.rs:1281` (`test_bug_006_csg_booleans_and_holes_watertight`).
- **Доказательство 1 (интеграционный тест `qa_wide_scan`):**
  ```text
  ---- test_bug_006_csg_booleans_and_holes_watertight stdout ----
  thread 'test_bug_006_csg_booleans_and_holes_watertight' panicked at crates/mcp-cad-server/tests/qa_wide_scan.rs:1281:5:
  BUG-020: CSG two holes should have 0 triangles < 1 deg, got 85 (min_angle=0.2119)
  ```
- **Доказательство 2 (живой вызов MCP-сервера `boolean_difference` Box $2\times 3\times 4$ минус Sphere $R=1.0$ + `validate_mesh`):**
  ```json
  {
    "status": "FAIL",
    "is_watertight": false,
    "is_manifold": false,
    "is_orientable": true,
    "vertex_count": 19194,
    "triangle_count": 32543,
    "boundary_edges_count": 17949,
    "non_manifold_edges_count": 90,
    "inverted_edges_count": 115,
    "signed_volume": 19.928050126414746,
    "surface_area": 57.68443429944333,
    "aspect_ratio_stats": {
      "lt_1_deg": 7369,
      "lt_5_deg": 16207,
      "pct_lt_5_deg": 49.80180069446578,
      "min_angle_deg": 1.3728264702819784e-6
    }
  }
  ```
- **Рекомендация по починке:** Помимо починки инвертированных крышек цилиндра (`BLOCKER-2`), в `crates/mcp-cad-core/src/csg.rs` необходимо выполнять сварку Т-образных вершин (T-junction welding / CDT constrained Delaunay triangulation) на рёбрах пересечения копланарных и секущих граней BSP-дерева вместо простой веерной триангуляции (`triangulate_polygon`), которая оставляет несогласованные рёбра (`boundary_edges_count: 17949`) и вырожденные углы $< 1^\circ$.

---

### 3.2. Серьёзные проблемы (нарушение контрактов API, некорректная геометрия, падение тестов)

#### [SERIOUS-1] Отсутствует валидация самопересечений и границ профиля `revolve` в Rust MCP-сервере (`BUG-016`)
- **Где:** `crates/mcp-cad-server/src/tools/cad.rs:212-238` и `crates/mcp-cad-core/src/primitives.rs:1460-1560`; падает тест `test_bug016_revolve_boundary_and_self_intersection_validation` (`crates/mcp-cad-server/tests/mcp_contract_tests.rs:2436`).
- **Суть дефекта:** В коммите `9800a44` (`fix(core): enforce revolve profile validation... [BUG-016]`) автор добавил проверку $r < 0$, $\text{angle\_deg} \notin (0, 360]$ и пересечения отрезков контура в клиентский TypeScript-диспетчер (`src/cad/mcp_dispatcher.ts`), но в Rust-реализации `add_feature_primitive` (`crates/mcp-cad-server/src/tools/cad.rs`) и `create_revolve` (`crates/mcp-cad-core/src/primitives.rs`) эти проверки отсутствуют — сервер возвращает `isError: false`.
- **Доказательство:**
  ```text
  ---- test_bug016_revolve_boundary_and_self_intersection_validation stdout ----
  thread 'test_bug016_revolve_boundary_and_self_intersection_validation' panicked at crates/mcp-cad-server/tests/mcp_contract_tests.rs:2436:5:
  assertion `left == right` failed: Negative radius r < 0 must be rejected with an error
    left: Some(false)
   right: Some(true)
  ```
- **Рекомендация по починке:** Добавить проверку входного профиля `revolve` в `crates/mcp-cad-core/src/primitives.rs` (или `crates/mcp-cad-server/src/tools/cad.rs`): отклонять точки с $r < 0$, углы $\text{angle\_deg} \le 0$ или $> 360$, профили с $< 3$ уникальными точками и самопересекающиеся сегменты контура.

---

#### [SERIOUS-2] Игольчатые треугольники ($< 1^\circ$) при частичном вращении (`partial revolve`) и фаске призмы (`polygon_prism`)
- **Где:**
  - `crates/mcp-cad-server/tests/qa_wide_scan.rs:950` (`test_bug_003_partial_revolve_mesh_and_renders`)
  - `crates/mcp-cad-server/tests/qa_wide_scan.rs:1053` (`test_bug_005_polygon_prism_chamfer_and_sides_validation`)
- **Доказательство:**
  ```text
  ---- test_bug_003_partial_revolve_mesh_and_renders stdout ----
  thread 'test_bug_003_partial_revolve_mesh_and_renders' panicked at crates/mcp-cad-server/tests/qa_wide_scan.rs:950:5:
  Partial revolve 90 deg CCW should have 0 needle triangles (<1 deg), got 2 (min_angle=0.9076)

  ---- test_bug_005_polygon_prism_chamfer_and_sides_validation stdout ----
  thread 'test_bug_005_polygon_prism_chamfer_and_sides_validation' panicked at crates/mcp-cad-server/tests/qa_wide_scan.rs:1053:9:
  Prism sides=3 should have <5% triangles < 5 deg, got 50.00%
  ```
- **Рекомендация по починке:**
  - В `create_revolve` (`crates/mcp-cad-core/src/primitives.rs`): при триангуляции торцевых крышек (`cap_start`, `cap_end`) фильтровать коллинеарные/почти коллинеарные вершины на оси вращения и выбирать диагональ четырёхугольника по максимизации минимального угла (Delaunay flip).
  - В `create_polygon_prism` (`crates/mcp-cad-core/src/primitives.rs`): при `chamfer > 0` и `height = 2.0` боковые грани и фаски разбиваются на узкие полосы; необходимо ограничить величину фаски геометрическим радиусом вписанной окружности и убрать избыточное продольное разбиение плоских граней.

---

#### [SERIOUS-3] Падение фронтенд-теста `src/cad/cad_engine.test.ts` (рассинхрон пресетов нагрузок FEA)
- **Где:** `src/cad/cad_engine.test.ts:507` vs `src/cad/workbenches.ts`.
- **Суть дефекта:** В `src/cad/workbenches.ts` массив `FEA_LOAD_PRESETS` содержит 5 пресетов (`cantilever_1000n`, `axial_tension_5kn`, `torsion_50nm`, `pressure_top_1mpa`, `gravity_self_weight`), а тест `src/cad/cad_engine.test.ts:507` проверяет `assert.strictEqual(FEA_LOAD_PRESETS.length, 4)`.
- **Доказательство:**
  ```text
  $ npx tsx --test src/cad/cad_engine.test.ts
  ✖ validates logical Engineering Workbenches and FEA quick load presets (3.063799ms)
    AssertionError [ERR_ASSERTION]: Expected values to be strictly equal:
    5 !== 4
        at TestContext.<anonymous> (/app/applet/src/cad/cad_engine.test.ts:507:12)
  ℹ tests 26
  ℹ pass 25
  ℹ fail 1
  ```
- **Рекомендация по починке:** Обновить ожидание в `src/cad/cad_engine.test.ts:507` на `5` (и добавить проверку `gravity_self_weight`), либо привести список пресетов в соответствие со спецификацией.

---

### 3.3. Мелкие проблемы (`cargo clippy`, предупреждения компилятора)

#### [MINOR-1] `cargo clippy --workspace -- -D warnings` падает с 6 ошибками в `mcp-cad-core`
- **Где и доказательство:**
  ```text
  $ CARGO_TARGET_DIR=/tmp/cargo_target cargo clippy --workspace -- -D warnings
  error: this `(a + b) % c == 0` can be written more clearly with `is_multiple_of`
     --> crates/mcp-cad-core/src/primitives.rs:270:24
      |
  270 |                     if (i + s) % 2 == 0 {
      |                        ^^^^^^^^^^^^^^^^ help: try: `(i + s).is_multiple_of(2)`
      = note: `-D clippy::manual-is-multiple-of` implied by `-D warnings`

  error: variable does not need to be mutable
     --> crates/mcp-cad-core/src/primitives.rs:930:9
      |
  930 |     let mut profile = raw_profile;
      |         ----^^^^^^^

  error: manual `RangeInclusive::contains` implementation
     --> crates/mcp-cad-core/src/csg.rs:597:20
      |
  597 |                 if t >= -EPSILON && t <= 1.0 + EPSILON {
      |                    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ help: use: `(-EPSILON..=1.0 + EPSILON).contains(&t)`

  error: value assigned to `result` is never read
     --> crates/mcp-cad-core/src/query_optimizer.rs:289:38
      |
  289 |     pub fn eval_cost_plan(&self, mut result: MeshData) -> f64 {

  error: unused variable: `apply`
      --> crates/mcp-cad-core/src/query_optimizer.rs:1141:9
       |
  1141 |     let apply = should_apply && cost_improved;

  error: unused variable: `genus_after`
      --> crates/mcp-cad-core/src/query_optimizer.rs:1217:26
       |
  1217 |     let (euler_after, _, genus_after) = compute_euler_characteristic(&optimized_mesh);

  error: could not compile `mcp-cad-core` (lib) due to 6 previous errors
  ```
- **Дополнительно в тестах:** В `crates/mcp-cad-server/tests/qa_wide_scan.rs` есть 2 предупреждения `unused variable: rim_id` (строка 960) и `unused variable: gear_res` (строка 1344).

---

## 4. Что я менял в коде (ветка `verify/fix-compile`)

> **ЯВНОЕ УВЕДОМЛЕНИЕ:** **Эти строки я исправил сам, это не оригинал.** На оригинальном коммите `main` (`02c7339`) проект не компилировался. Чтобы разблокировать прогон тестов и верификацию сервера (Шаг 2 плана), в отдельной ветке `verify/fix-compile` (коммит `fb2dde0`) были внесены минимальные правки только для устранения ошибок компиляции, без изменения бизнес-логики и без трогания `assert`-ов в тестах.

- **Ветка:** `verify/fix-compile`
- **Хеш коммита:** `fb2dde0`
- **Diff-stat относительно `main` (`02c7339`):**
  ```text
  $ git diff --stat main..verify/fix-compile
   crates/mcp-cad-core/src/primitives.rs             | 36 +++++++++++------------
   crates/mcp-cad-core/src/query_optimizer.rs        |  2 +-
   crates/mcp-cad-server/tests/mcp_contract_tests.rs |  4 +--
   3 files changed, 21 insertions(+), 21 deletions(-)
  ```

### Полный `git diff main..verify/fix-compile`
```diff
diff --git a/crates/mcp-cad-core/src/primitives.rs b/crates/mcp-cad-core/src/primitives.rs
index 7f8266d..034cf85 100644
--- a/crates/mcp-cad-core/src/primitives.rs
+++ b/crates/mcp-cad-core/src/primitives.rs
@@ -2356,11 +2356,11 @@ pub fn create_wheel_rim(
         v.len() - 1
     };
 
-    let mut push_quad = |p0: Point3<f64>, p1: Point3<f64>, p2: Point3<f64>, p3: Point3<f64>| {
-        let i0 = push_v(&mut vertices, p0) as u32;
-        let i1 = push_v(&mut vertices, p1) as u32;
-        let i2 = push_v(&mut vertices, p2) as u32;
-        let i3 = push_v(&mut vertices, p3) as u32;
+    macro_rules! push_quad {
+        ($p0:expr, $p1:expr, $p2:expr, $p3:expr) => {{
+            let i0 = push_v(&mut vertices, $p0) as u32;
+            let i1 = push_v(&mut vertices, $p1) as u32;
+            let i2 = push_v(&mut vertices, $p2) as u32;
+            let i3 = push_v(&mut vertices, $p3) as u32;
 
         let d02 = ((p0.x - p2.x).powi(2) + (p0.y - p2.y).powi(2) + (p0.z - p2.z).powi(2)).sqrt();
         let d13 = ((p1.x - p3.x).powi(2) + (p1.y - p3.y).powi(2) + (p1.z - p3.z).powi(2)).sqrt();
@@ -2371,7 +2371,8 @@ pub fn create_wheel_rim(
             indices.push([i0, i1, i3]);
             indices.push([i1, i2, i3]);
         }
-    };
+        }};
+    }
 
     let rim_steps = (spoke_count * 12).max(48);
     let r_inner_rim = rim_radius - rim_thickness;
@@ -2499,22 +2500,19 @@ pub fn create_wheel_rim(
             let z1 = -half_w + (w_step + 1) as f64 * w_Step_len;
 
             // Outer tire-bed face (+R normal)
-            push_quad(
+            push_quad!(
                 Point3::new(r_outer * c0, r_outer * s0, z0),
                 Point3::new(r_outer * c1, r_outer * s1, z0),
                 Point3::new(r_outer * c1, r_outer * s1, z1),
                 Point3::new(r_outer * c0, r_outer * s0, z1),
             );
 
             // Inner barrel face (-R normal) when not in the spoke band
             if w_step == 0 || w_step == 2 {
-                push_quad(
+                push_quad!(
                     Point3::new(r_inner_rim * c1, r_inner_rim * s1, z0),
                     Point3::new(r_inner_rim * c0, r_inner_rim * s0, z0),
                     Point3::new(r_inner_rim * c0, r_inner_rim * s0, z1),
                     Point3::new(r_inner_rim * c1, r_inner_rim * s1, z1),
                 );
             }
         }
 
         // Front (+Z) and Back (-Z) annular walls of outer rim barrel
-        push_quad(
+        push_quad!(
             Point3::new(r_inner_rim * c0, r_inner_rim * s0, half_w),
             Point3::new(r_outer * c0, r_outer * s0, half_w),
             Point3::new(r_outer * c1, r_outer * s1, half_w),
             Point3::new(r_inner_rim * c1, r_inner_rim * s1, half_w),
         );
-        push_quad(
+        push_quad!(
             Point3::new(r_outer * c0, r_outer * s0, -half_w),
             Point3::new(r_inner_rim * c0, r_inner_rim * s0, -half_w),
             Point3::new(r_inner_rim * c1, r_inner_rim * s1, -half_w),
@@ -2535,14 +2533,14 @@ pub fn create_wheel_rim(
             let z0_h = -hub_half_w + h_step as f64 * h_step_len;
             let z1_h = -hub_half_w + (h_step + 1) as f64 * h_step_len;
             // Bore wall (-R normal towards center)
-            push_quad(
+            push_quad!(
                 Point3::new(bore_r * c1, bore_r * s1, z0_h),
                 Point3::new(bore_r * c0, bore_r * s0, z0_h),
                 Point3::new(bore_r * c0, bore_r * s0, z1_h),
                 Point3::new(bore_r * c1, bore_r * s1, z1_h),
             );
             if h_step == 0 || h_step == 2 {
                 // Exposed outer hub wall (+R normal) outside the spoke band
-                push_quad(
+                push_quad!(
                     Point3::new(hub_r * c0, hub_r * s0, z0_h),
                     Point3::new(hub_r * c1, hub_r * s1, z0_h),
                     Point3::new(hub_r * c1, hub_r * s1, z1_h),
@@ -2559,7 +2557,7 @@ pub fn create_wheel_rim(
         if !is_spoke_sector {
             // Exposed inner rim wall in the middle band (-R normal)
-            push_quad(
+            push_quad!(
                 Point3::new(r_inner_rim * c1, r_inner_rim * s1, z_back),
                 Point3::new(r_inner_rim * c0, r_inner_rim * s0, z_back),
                 Point3::new(r_inner_rim * c0, r_inner_rim * s0, z_front),
@@ -2567,7 +2565,7 @@ pub fn create_wheel_rim(
             );
             // Exposed outer hub wall in the middle band (+R normal)
-            push_quad(
+            push_quad!(
                 Point3::new(hub_r * c0, hub_r * s0, z_back),
                 Point3::new(hub_r * c1, hub_r * s1, z_back),
                 Point3::new(hub_r * c1, hub_r * s1, z_front),
@@ -2598,11 +2596,11 @@ pub fn create_wheel_rim(
         let hub_mid_pts = make_circle_2d(r_hub_mid, hub_steps);
         let bore_pts = make_circle_2d(bore_r, hub_steps);
 
-        add_halving_ring_transition_2d(&mut vertices, &mut indices, &hub_outer_pts, &hub_mid_pts, z_front, 1.0, Winding::Cw);
-        add_bridged_ring_strip_2d(&mut vertices, &mut indices, &hub_mid_pts, &bore_pts, z_front, 1.0, 2, Winding::Ccw);
+        add_halving_ring_transition_2d(&mut vertices, &mut indices, &hub_outer_pts, &hub_mid_pts, z_front, false);
+        add_bridged_ring_strip_2d(&mut vertices, &mut indices, &hub_mid_pts, &bore_pts, z_front, true);
 
-        add_halving_ring_transition_2d(&mut vertices, &mut indices, &hub_outer_pts, &hub_mid_pts, z_back, -1.0, Winding::Ccw);
-        add_bridged_ring_strip_2d(&mut vertices, &mut indices, &hub_mid_pts, &bore_pts, z_back, -1.0, 2, Winding::Cw);
+        add_halving_ring_transition_2d(&mut vertices, &mut indices, &hub_outer_pts, &hub_mid_pts, z_back, true);
+        add_bridged_ring_strip_2d(&mut vertices, &mut indices, &hub_mid_pts, &bore_pts, z_back, false);
     }
 
     // Generate spokes with radial & lateral subdivisions to avoid long thin triangles
@@ -2628,25 +2626,25 @@ pub fn create_wheel_rim(
             let p_next_l = Point3::new(r1 * c_l1, r1 * s_l1, z_m1);
 
             // Front face of spoke (+Z normal)
-            push_quad(p_curr_r, p_next_r, p_next_l, p_curr_l);
+            push_quad!(p_curr_r, p_next_r, p_next_l, p_curr_l);
 
             // Back face of spoke (-Z normal)
-            push_quad(
+            push_quad!(
                 Point3::new(p_curr_l.x, p_curr_l.y, z_m0),
                 Point3::new(p_next_l.x, p_next_l.y, z_m0),
                 Point3::new(p_next_r.x, p_next_r.y, z_m0),
                 Point3::new(p_curr_r.x, p_curr_r.y, z_m0),
             );
 
             // Right side wall of spoke (-theta normal)
-            push_quad(
+            push_quad!(
                 Point3::new(p_curr_r.x, p_curr_r.y, z_m0),
                 Point3::new(p_next_r.x, p_next_r.y, z_m0),
                 p_next_r,
                 p_curr_r,
             );
 
             // Left side wall of spoke (+theta normal)
-            push_quad(
+            push_quad!(
                 p_curr_l,
                 p_next_l,
                 Point3::new(p_next_l.x, p_next_l.y, z_m0),
diff --git a/crates/mcp-cad-core/src/query_optimizer.rs b/crates/mcp-cad-core/src/query_optimizer.rs
index 3e10946..f755346 100644
--- a/crates/mcp-cad-core/src/query_optimizer.rs
+++ b/crates/mcp-cad-core/src/query_optimizer.rs
@@ -1328,7 +1328,7 @@ pub fn optimize_part_dag(
 
     let report = CadOptimizationReport {
         strategy: format!("{:?}", settings.strategy),
-        applied,
+        applied: !applied_rules.is_empty(),
         applied_rules,
         applied_rule_ids,
         candidates_evaluated,
diff --git a/crates/mcp-cad-server/tests/mcp_contract_tests.rs b/crates/mcp-cad-server/tests/mcp_contract_tests.rs
index 9ab00a3..bf84c65 100644
--- a/crates/mcp-cad-server/tests/mcp_contract_tests.rs
+++ b/crates/mcp-cad-server/tests/mcp_contract_tests.rs
@@ -2332,7 +2332,7 @@ fn test_mcp_optimize_feature_tree_cbo() {
 /// Returns an explicit validation error instead of silently clamping or creating non-manifold geometry.
 #[test]
 fn test_bug016_revolve_boundary_and_self_intersection_validation() {
-    let state = EngineState::new_shared();
+    let state = EngineState::new().unwrap().into_shared();
 
     let create_req = JsonRpcRequest {
         jsonrpc: "2.0".to_string(),
@@ -2398,7 +2398,7 @@ fn test_bug016_revolve_boundary_and_self_intersection_validation() {
 /// - `is_manifold: true`, `is_watertight: true`
 #[test]
 fn test_bug017_20_cap_sliver_elimination_and_negative_scale() {
-    let state = EngineState::new_shared();
+    let state = EngineState::new().unwrap().into_shared();
 
     let create_req = JsonRpcRequest {
         jsonrpc: "2.0".to_string(),
```

---

## 5. Проверка живого MCP-сервера (Шаг 5)

На собранном релизном бинаре `/tmp/cargo_target/release/mcp-cad-server` проверены оба режима транспорта (`--mode stdio` и `--mode sse --port 8080`):

1. **`initialize` (JSON-RPC 2.0):**
   ```json
   {"jsonrpc":"2.0","id":1,"result":{"capabilities":{"tools":{}},"protocolVersion":"2024-11-05","serverInfo":{"name":"mcp-cad-engine","version":"0.1.0"}}}
   ```
2. **`tools/list`:** возвращает **28 инструментов**, список на 100% совпадает с таблицей в `README.md`:
   `create_part`, `list_parts`, `delete_part`, `add_feature_primitive`, `cut_hole`, `modify_feature`, `get_feature_tree`, `cut_helical_groove`, `boolean_difference`, `create_joint`, `set_joint_motor`, `run_physics_simulation`, `run_fea_analysis`, `get_fea_heatmap_image`, `capture_viewport`, `export_stl`, `get_simulation_metrics_json`, `open_preview_window`, `import_stl`, `tessellate_part`, `query_part_geometry`, `boolean_union`, `boolean_intersection`, `transform_part`, `define_material`, `batch_cad_operations`, `validate_mesh`, `optimize_feature_tree`.
3. **Аналитическая проверка геометрии через живые вызовы `tools/call`:**
   - **Куб / параллелепипед $2\times 3\times 4\text{ м}$ (`create_part` + `add_feature_primitive` box + `query_part_geometry`):**
     - Получено: `volume = 24.0`, `surface_area = 52.0`, `bbox = [-1..1, -1.5..1.5, -2..2]`.
     - Аналитика: $V = 2\cdot 3\cdot 4 = 24.0$ (погрешность **0.0%**), $S = 2(6+8+12) = 52.0$ (погрешность **0.0%**).
   - **Сфера $R=1.0\text{ м}$, `sectors=64, stacks=32` (`query_part_geometry`):**
     - Получено: `volume = 4.184514974351565`, `surface_area = 12.559944171739366`.
     - Аналитика: $V = \frac{4}{3}\pi R^3 = 4.1887902047863905$ (погрешность тесселяции **0.102%**), $S = 4\pi R^2 = 12.566370614359172$ (погрешность **0.051%**).
   - **Булева разность `boolean_difference` (Box $2\times 3\times 4$ минус Sphere $R=1.0$):**
     - Получено: `signed_volume = 19.928050126414746` против аналитического $24 - \frac{4}{3}\pi = 19.811209795$ (разница **0.59%**), но `validate_mesh` возвращает `status: "FAIL"`, `is_watertight: false`, `boundary_edges_count: 17949`, `lt_1_deg: 7369` (`49.80%` треугольников $< 5^\circ$).
   - **Экспорт `export_stl`:**
     - Записал валидный бинарный STL (`32,543` треугольника, `1,627,234` байт = $80 + 4 + 32543 \times 50$ байт).
   - **Расчёт МКЭ `run_fea_analysis` (консольная балка $0.2\times 0.02\times 0.02\text{ м}$, сталь AISI 1020, заделка `MinX`, нагрузка $F_y = -1000\text{ Н}$ на `MaxX`):**
     - Получено: `status: "PASS"`, `element_count: 480`, `node_count: 189`, `max_displacement_mm: 0.51105`, `max_von_mises_mpa: 81.4787`, `safety_factor: 3.0683` (для сравнения по Эйлеру–Бернулли $\delta = \frac{FL^3}{3EI} = 1.0\text{ мм}$, $\sigma = \frac{FLc}{I} = 150\text{ МПа}$ — линейные тетраэдры Tet4 на грубой сетке дают ожидаемое завышение жёсткости `locking`, но решатель реально собирает матрицу жёсткости и решает СЛАУ).
4. **Проверка SSE транспорта (`--mode sse --port 8080`):**
   - `GET http://127.0.0.1:8080/sse` отдаёт поток `event: endpoint\ndata: /message`.
   - `POST http://127.0.0.1:8080/message` с JSON-RPC `initialize` возвращает HTTP 200 с валидным JSON-RPC ответом.

---

## 6. Что НЕ удалось проверить и почему

1. **Интерактивное нативное окно `open_preview_window` (`winit` GUI window):**
   - В контейнерном Linux-окружении отсутствует физический дисплей и запущенный X11/Wayland композитор (`DISPLAY` не задан). При этом **вся offscreen/headless графическая часть `mcp-cad-viewer`** (`wgpu` поверх программного Vulkan-драйвера Mesa Lavapipe `lvp_icd.x86_64.json`, рендеринг шейдеров, буферов глубины и `capture_viewport` в PNG) **была полностью проверена** и прошла 10 из 10 тестов (`cargo test -p mcp-cad-viewer`).

---

## 7. Расхождения с заявлениями в документации и `qa/` (Факты против отчётов)

В репозитории лежат файлы `qa/STATE.json`, `qa/FINAL_REPORT_ROUND2.md`, `qa/BUGS.md`, `qa/COVERAGE.md` и `CHANGELOG.md`, которые утверждают, что проект прошёл «Round 2 QA Verification» со 100% прохождением всех тестов. Верификация показала, что **эти отчёты сфабрикованы (написаны без запуска компилятора `cargo`)**:

| Заявление в репозитории (`qa/`, `README.md`, `CHANGELOG.md`) | Реальный факт (доказано запуском `cargo` и `tsx`) |
| :--- | :--- |
| **`qa/STATE.json`:** `"current_phase": "COMPLETE"`, `"fixed_bugs": ["BUG-001", ..., "BUG-020"]`, `"open_bugs": []` | На коммите `02c7339` (и `dd4d088`, где записан `STATE.json`) `mcp-cad-core` **даже не компилируется** из-за 9 синтаксических/типовых ошибок (`Winding::Cw`, `applied`). |
| **`qa/FINAL_REPORT_ROUND2.md`:** *"Total Automated Tests Executed: 84 / Passed: 84 (100%) / Failed: 0"* и *"Rust Integration Suite (`qa_wide_scan.rs` & `mcp_contract_tests.rs`): 58/58 PASS"* | В реальности в `qa_wide_scan.rs` (13 тестов) и `mcp_contract_tests.rs` (21 тест) всего **34 теста** (а не 58), `mcp_contract_tests.rs` не компилируется из-за `EngineState::new_shared()`, а после починки компиляции из 34 тестов падают **7** (`19/21` в `mcp_contract_tests` и `8/13` в `qa_wide_scan`). |
| **`qa/FINAL_REPORT_ROUND2.md`:** *"TypeScript Unit & Parity Suite (`src/cad/cad_engine.test.ts`): 26/26 PASS"* | Команда `npx tsx --test src/cad/cad_engine.test.ts` выдаёт **25 passed, 1 failed** (`AssertionError: 5 !== 4` на строке 507). |
| **`qa/BUGS.md` / `CHANGELOG.md`:** *"BUG-018 FIXED: Eliminated cap sliver triangles (<5° dropped from 40% to 0.0%) in Cylinder, Cone, Frustum, and Spur Gear"* | При попытке починить `BUG-018` в коммите `6db3344` автор перепутал направление обхода колец (`is_top`), из-за чего **сломал все крышки цилиндров, конусов и шестерён**: теперь у них 96 инвертированных рёбер и неверный объём (`11.83` вместо `35.34`). |
| **`qa/BUGS.md` / `qa/FINAL_REPORT_ROUND2.md`:** *"BUG-016 FIXED: Revolve profile validation rejects negative radii ($r < 0$), angles $> 360^\circ$, and self-intersecting contours"* | Проверка была добавлена только в фронтенд-файл `src/cad/mcp_dispatcher.ts`. В Rust-сервере (`crates/mcp-cad-server/src/tools/cad.rs`) проверка отсутствует, и собственный тест автора `test_bug016_revolve_boundary_and_self_intersection_validation` **падает**. |
| **`qa/BUGS.md`:** *"BUG-019 / BUG-020 FIXED: CSG operations produce 0 triangles < 1° and 100% watertight meshes"* | Собственный тест автора `test_bug_006_csg_booleans_and_holes_watertight` **падает** (`got 85 triangles < 1 deg, min_angle=0.2119`), а `boolean_difference` куба и сферы даёт `is_watertight: false` и `17,949` граничных рёбер. |
