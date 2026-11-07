# Триаж остаточных падений после выполнения P0-T1 (`docs/TRIAGE_AFTER_P0T1.md`)

**Задача:** P0-T3 — Триаж остаточных падений (только анализ, без правок кода)  
**Дата проведения:** 04.11.2026  
**База проверки:** Workspace после применения исправления ориентации торцевых крышек **P0-T1** и добавления набора параметрических инвариантов **P0-T2**.  
**Команда полного прогона:**
```bash
export PATH=/usr/lib/rust-1.91/bin:$PATH
WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test --workspace --no-fail-fast
```

---

## 1. Общая сводка состояния тестов до и после P0-T1

На исходной базе (до **P0-T1**) из 126 тестов падало **19 тестов** (107 passed, 19 failed).  
После исправления ориентации торцевых крышек в `add_bridged_ring_strip_2d` и `add_halving_ring_transition_2d` (`crates/mcp-cad-core/src/primitives.rs`) в рамках **P0-T1** и добавления 18 тестов в `primitive_invariants.rs` (**P0-T2**):
- **Всего тестов в Rust Workspace:** `148`
- **Проходят (`passed`):** `136`
- **Падают (`failed`):** `4` (в 2 тестовых таргетах крейта `mcp-cad-server`)
- **Отложены (`ignored` с меткой `#[ignore = "BUG: ..."]` из P0-T2):** `8` (в `crates/mcp-cad-core/tests/primitive_invariants.rs`: 3 дублируют падения P0-T5..T7 на уровне ядра, 5 фиксируют новые краевые случаи `Sliver quality gate`)

### Итоги по тестовым таргетам Workspace

| Крейт | Таргет | Результат после P0-T1 | Passed | Failed | Ignored |
|---|---|---|---:|---:|---:|
| `mcp-cad-core` | `--lib` (unit-тесты) | `ok` | 48 | 0 | 0 |
| `mcp-cad-core` | `--test dag_tests` | `ok` | 14 | 0 | 0 |
| `mcp-cad-core` | `--test tessellation_tests` | `ok` | 6 | 0 | 0 |
| `mcp-cad-core` | `--test primitive_invariants` | `ok` | 10 | 0 | 8 |
| `mcp-cad-fea` | `--lib` + `--test fea_tests` | `ok` | 11 | 0 | 0 |
| `mcp-cad-physics` | `--lib` + `--test physics_tests` | `ok` | 6 | 0 | 0 |
| `mcp-cad-viewer` | `--lib` + `--test viewer_tests` | `ok` | 10 | 0 | 0 |
| `mcp-cad-server` | `--lib` + `--bin mcp-cad-server` | `ok` | 0 | 0 | 0 |
| `mcp-cad-server` | `--test gear_kinematic_test` | `ok` | 1 | 0 | 0 |
| `mcp-cad-server` | `--test mcp_contract_tests` | **`FAILED`** | 20 | **1** | 0 |
| `mcp-cad-server` | `--test qa_wide_scan` | **`FAILED`** | 10 | **3** | 0 |
| **ИТОГО** | **Все таргеты Workspace** | **`FAILED` (2 таргета)** | **136** | **4** | **8** |

---

## 2. Проверка кандидатов, которые перестали падать после P0-T1 (подтверждение каскадной природы)

В описании задачи **P0-T3** были выделены 4 теста-кандидата, падавших до P0-T1, для которых требовалось проверить, являлись ли они каскадным следствием перевёрнутых крышек цилиндра:

### 2.1. `dag_tests::test_dag_shifted_cylinder_stepped_shaft` и `dag_tests::test_dag_helical_groove_drill_bit_model`
- **Команда проверки:**
  ```bash
  cargo test -p mcp-cad-core --test dag_tests
  ```
- **Результат после P0-T1:** `test result: ok. 14 passed; 0 failed; 0 ignored; finished in 23.96s` — **ОБА ПРОХОДЯТ**.
- **Доказательство первопричины (каскад от P0-T1):**
  - В `test_dag_shifted_cylinder_stepped_shaft` (`crates/mcp-cad-core/tests/dag_tests.rs:446–493`) строятся два соосных цилиндра с `segments = 64`: хвостовик ($r=2, h=10, z \in [0, 10]$) и тело вала ($r=4, h=20, z \in [10, 30]$), объединённые через `csg_union` в `FeatureTree::evaluate()`. До P0-T1 торцевые крышки обоих цилиндров при `segments = 64` имели инвертированные нормали (внутрь тела), из-за чего BSP-дерево в `BspNode::clip_polygons` классифицировало внутреннее пространство цилиндров как внешнее и отсекало полигоны, давая нулевой объём. После починки ориентации крышек в P0-T1 объём ступенчатого вала совпадает с аналитическим $\pi \cdot 2^2 \cdot 10 + \pi \cdot 4^2 \cdot 20 = 1130.973$ с погрешностью $< 0.1\%$.
  - Аналогично в `test_dag_helical_groove_drill_bit_model` (`dag_tests.rs:496–550`) заготовка сверла `AddCylinder { radius: 5.0, height: 50.0, segments: 32 }` до P0-T1 имела вывернутые крышки, что разрушало `csg_difference` при вычитании винтовых канавок `CutHelicalGroove`. После P0-T1 тест стабильно проходит.

### 2.2. `query_optimizer::tests::test_r4_coaxial_bushing_cylinder_hole_to_revolve` и `mcp_contract_tests::test_mcp_optimize_feature_tree_cbo`
- **Команда проверки:**
  ```bash
  cargo test -p mcp-cad-core --lib query_optimizer::tests::test_r4_coaxial_bushing_cylinder_hole_to_revolve
  cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_optimize_feature_tree_cbo
  ```
- **Результат после P0-T1:** **ОБА ПРОХОДЯТ**.
- **Доказательство первопричины (каскад от P0-T1) и остаток для P0-T9:**
  - В `optimize_feature_tree_dag` (`crates/mcp-cad-core/src/query_optimizer.rs:1196–1231`) правило `R4_COAXIAL_REVOLVE_REDUCTION` (`apply_rule_r4_coaxial_revolve_reduction`, строки 880–1019) всегда успешно формировало кандидата с одним узлом `AddRevolve`, однако на этапе верификации эквивалентности (строки 1197–1221):
    ```rust
    let mesh_before = dag.evaluate_with_settings(settings)?;
    let vol_before = mesh_before.volume();
    ...
    let equivalence_ok = vol_delta_pct < 0.5
        && (!val_before.topology.is_watertight || val_after.topology.is_watertight);
    ```
    до P0-T1 исходное дерево `AddCylinder + CutHole` имело искажённый объём `vol_before` из-за вывернутых крышек цилиндра, тогда как `AddRevolve` (полный оборот $360^\circ$ без торцевых крышек `add_subdivided_disk_cap_2d`) давал правильный аналитический объём втулки. В результате `vol_delta_pct` превышал порог `0.5%`, срабатывал откат (fallback, строки 1225–1231) к исходным 3 узлам, и `applied_rules` очищался.
  - **Важное примечание для P0-T9:** Хотя оба теста теперь зелёные, проверка `equivalence_ok` в `query_optimizer.rs:1220` сейчас проверяет только `vol_delta_pct < 0.5` и `is_watertight`, но **не проверяет** `val_after.status == "PASS"` целиком, и в `query_optimizer::tests` отсутствует негативный тест на неосевое отверстие (требования задачи **P0-T9**).

---

## 3. Детальный разбор 4 остаточных падающих тестов

Ниже для каждого из 4 падающих тестов приведены: команда минимального воспроизведения, точное сообщение об ошибке, доказанная первопричина с указанием файлов и строк кода, а также целевая задача из пакета `fix/`.

---

### Падение №1: `mcp_contract_tests::test_bug016_revolve_boundary_and_self_intersection_validation`

- **Минимальное воспроизведение:**
  ```bash
  cargo test -p mcp-cad-server --test mcp_contract_tests test_bug016_revolve_boundary_and_self_intersection_validation -- --exact --nocapture
  ```
- **Точное сообщение об ошибке:**
  ```text
  thread 'test_bug016_revolve_boundary_and_self_intersection_validation' panicked at crates/mcp-cad-server/tests/mcp_contract_tests.rs:2440:5:
  Revolve with r < 0 must return an error
  ```
- **Гипотеза первопричины и доказательство (проверено по коду):**
  1. Валидация параметров `revolve` на стороне сервера **присутствует и корректно возвращает `Err(ServerError::InvalidArgument(...))`** в `crates/mcp-cad-server/src/tools/cad.rs` (строки 305–388):
     - строка 330: `if r < -1e-9 { return Err(ServerError::InvalidArgument(format!("Profile coordinate r ({r}) cannot be negative (crosses rotation axis)"))); }`
     - строка 358: `if segments_intersect_2d(...) { return Err(ServerError::InvalidArgument("Revolve 2D profile is self-intersecting in the (r, z) plane".to_string())); }`
     - строка 372: `if !angle_deg.is_finite() || angle_deg <= 0.0 || angle_deg > 360.0 { return Err(ServerError::InvalidArgument(format!("Revolve 'angle_deg' must be in (0, 360], got {angle_deg}"))); }`
  2. Однако обработчик JSON-RPC метода `"tools/call"` в `crates/mcp-cad-server/src/protocol.rs` (строки 222–237) преобразует любую ошибку `Err(err)` из `dispatch_tool` в успешный JSON-RPC ответ `JsonRpcResponse::success(id, res_value)`, внутри которого `result` содержит `ToolCallResult::error(err.to_string())`, то есть:
     ```json
     {
       "jsonrpc": "2.0",
       "id": 971,
       "result": {
         "content": [{ "type": "text", "text": "Invalid argument: Profile coordinate r (-0.01) cannot be negative (crosses rotation axis)" }],
         "isError": true
       }
     }
     ```
  3. Тест `test_bug016_revolve_boundary_and_self_intersection_validation` в `crates/mcp-cad-server/tests/mcp_contract_tests.rs` (строки 2440, 2459, 2478) ошибочно проверяет верхнеуровневое поле JSON-RPC `resp.get("error").is_some()`, которое остаётся `None`, тогда как остальные контрактные тесты в этом же файле проверяют `resp["result"]["isError"] == json!(true)`.
- **Какая задача закрывает:** **P0-T4** («Контракт ошибок: тест BUG-016»).

---

### Падение №2: `qa_wide_scan::test_bug_003_partial_revolve_mesh_and_renders`

- **Минимальное воспроизведение:**
  ```bash
  WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test qa_wide_scan test_bug_003_partial_revolve_mesh_and_renders -- --exact --nocapture
  ```
- **Точное сообщение об ошибке:**
  ```text
  thread 'test_bug_003_partial_revolve_mesh_and_renders' panicked at crates/mcp-cad-server/tests/qa_wide_scan.rs:950:9:
  assertion `left == right` failed: Partial revolve 90 deg CCW failed: ["Sliver quality gate failed: 2 needle triangles with min angle < 1° (min_angle=0.9076°)"]
    left: "FAIL"
   right: "PASS"
  ```
- **Гипотеза первопричины и доказательство (проверено по коду):**
  1. В `crates/mcp-cad-core/src/primitives.rs:2030–2043` функция `create_revolve` сначала подразделяет длинные прямолинейные рёбра 2D-профиля `raw_pts` на коллинеарные подсегменты (`n_sub` до 16 точек на одной прямой), чтобы выровнять соотношение сторон боковых четырёхугольников вращения. Для профиля `[[0.01, -0.02], [0.03, -0.02], [0.03, 0.02], [0.01, 0.02]]` при `angle_deg = 90.0, segments = 32` (`target_seg_len = 0.0045` м) 4-угольный контур превращается в многоугольник `pts` с 24 вершинами, большинство из которых лежат на одних и тех же вертикальных и горизонтальных прямых (внутренний угол $\approx 180^\circ$).
  2. Далее для неполного угла (`!is_full_circle`, строки 2118–2153) плоские торцевые крышки на углах $\theta = 0$ и $\theta = \text{sweep\_angle}$ триангулируются вызовом `triangulate_simple_polygon_2d(&pts)` (`primitives.rs:1896–1969`).
  3. В `triangulate_simple_polygon_2d` (строки 1920–1964):
     - Проверка выпуклости уха `if cross <= 1e-12 { return false; }` (строка 1926) отвергает коллинеарные тройки.
     - Наивный жадный цикл `for i in 0..len` (строка 1945) всегда отрезает **первое попавшееся** ухо на углу $90^\circ$, соединяя соседнюю точку на одной стороне прямоугольника с ближайшей точкой подразделённой перпендикулярной стороны. По мере отрезания таких узких клиньев через весь прямоугольник тянется веер длинных диагоналей ко всё более близким коллинеарным точкам, а при остатке только коллинеарных вершин срабатывает аварийный сброс `if stuck_count >= len { triangles.push([indices[0], indices[1], indices[2]]); }` (строки 1957–1960), порождающий вырожденные/игольчатые треугольники с углом `0.9076° < 1°` (и `0.3649°` на профиле с осью в `known_bug_revolve_partial_angle_end_cap_slivers`).
  4. В отличие от `csg.rs` (где используется выбор лучшего уха по минимальному углу `triangulate_subdivided_convex_loop` + `optimize_coplanar_delaunay_edges`), `triangulate_simple_polygon_2d` в `primitives.rs` не выбирает ухо с максимальным минимальным углом и не выполняет Delaunay-флип внутренних диагоналей плоской крышки.
- **Какая задача закрывает:** **P0-T5** («Игольчатые треугольники: частичный revolve»).

---

### Падение №3: `qa_wide_scan::test_bug_005_polygon_prism_chamfer_and_sides_validation`

- **Минимальное воспроизведение:**
  ```bash
  WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test qa_wide_scan test_bug_005_polygon_prism_chamfer_and_sides_validation -- --exact --nocapture
  ```
- **Точное сообщение об ошибке:**
  ```text
  thread 'test_bug_005_polygon_prism_chamfer_and_sides_validation' panicked at crates/mcp-cad-server/tests/qa_wide_scan.rs:1053:9:
  assertion `left == right` failed: Chamfered prism 3 sides failed: ["Sliver quality gate failed: 50.00% of triangles (12/24) have min angle < 5° (threshold 1.00%)"]
    left: "FAIL"
   right: "PASS"
  ```
- **Гипотеза первопричины и доказательство (проверено расчётом и по коду):**
  1. В `test_bug_005_polygon_prism_chamfer_and_sides_validation` (`crates/mcp-cad-server/tests/qa_wide_scan.rs:1046–1053`) вызывается `create_polygon_prism(3, 0.05, Some(0.04), None, Some(0.003))`.
  2. В `create_polygon_prism` (`crates/mcp-cad-core/src/primitives.rs:1050–1088`):
     - Для `sides = 3` (`half_angle = 60°`, $\cos 60^\circ = 0.5$) и `width_across_flats = 0.04` радиус описанной окружности равен:
       $$R = \frac{0.04}{2 \cos 60^\circ} = 0.04\text{ м}, \quad \text{длина стороны основания } L = 2 R \sin 60^\circ = 0.069282\text{ м}.$$
     - При `chamfer = 0.003` м радиальный отступ фаски $c_r = 0.003$ м, вертикальный отступ $c_z = c_r \tan 30^\circ = 0.001732$ м, а наклонная ширина полосы фаски равна:
       $$w_{\text{bevel}} = \sqrt{c_r^2 + c_z^2} = \frac{c_r}{\cos 30^\circ} = 0.003464\text{ м}.$$
  3. В строках 1180–1202 `create_polygon_prism` каждая из 3 верхних и 3 нижних наклонных граней фаски (трапеция шириной $L \approx 0.0693$ м и высотой всего $w_{\text{bevel}} \approx 0.00346$ м, отношение сторон $\approx 20:1$) передаётся целиком как **один четырёхугольник** в `add_quad_facet` (строки 1136–1157), который режет её одной диагональю на 2 длинных узких треугольника.
  4. Минимальный угол каждого из этих $6 \times 2 = 12$ треугольников фаски равен:
     $$\alpha_{\min} \approx \arctan\!\left(\frac{w_{\text{bevel}}}{L}\right) = \arctan\!\left(\frac{0.003464}{0.069282}\right) \approx 2.86^\circ < 5^\circ.$$
     Так как вся сетка 3-гранной призмы с фаской состоит всего из 24 треугольников (3 в верхней крышке + 3 в нижней + 6 боковых + 12 на фасках) без какого-либо продольного/поперечного разбиения рёбер многоугольника, ровно $12 / 24 = 50.00\%$ треугольников имеют угол $< 5^\circ$ (при пороге `MAX_ALLOWED_SLIVER_RATIO = 1.0%`). Та же проблема возникает при `sides = 4` (`16 / 32 = 50.00%` треугольников с углом $\approx 4.66^\circ < 5^\circ$).
- **Какая задача закрывает:** **P0-T6** («Игольчатые треугольники: фаска призмы»).

---

### Падение №4: `qa_wide_scan::test_bug_006_csg_booleans_and_holes_watertight`

- **Минимальное воспроизведение:**
  ```bash
  WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test qa_wide_scan test_bug_006_csg_booleans_and_holes_watertight -- --exact --nocapture
  ```
- **Точное сообщение об ошибке:**
  ```text
  thread 'test_bug_006_csg_booleans_and_holes_watertight' panicked at crates/mcp-cad-server/tests/qa_wide_scan.rs:1281:5:
  assertion `left == right` failed: Thru hole failed: ["Sliver quality gate failed: 85 needle triangles with min angle < 1° (min_angle=0.2119°)", "Sliver quality gate failed: 11.53% of triangles (136/1180) have min angle < 5° (threshold 1.00%)"]
    left: "FAIL"
   right: "PASS"
  ```
- **Гипотеза первопричины и доказательство (проверено по коду):**
  1. В `test_bug_006_csg_booleans_and_holes_watertight` (`crates/mcp-cad-server/tests/qa_wide_scan.rs:1278–1281`) в прямоугольной пластине `plate = create_box(0.1, 0.1, 0.04)` прорезается сквозное отверстие `cut_hole_with_settings(&plate, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.015, 0.10, None, None, &TessellationSettings::medium())`.
  2. Замкнутость, манифолдность, отсутствие T-образных вершин (`t_junctions_count == 0`), род поверхности ($\chi = 0, g = 1$) и объём выполняются строго (что также подтверждено тестом `invariants_cut_hole_topology_volume_and_aabb` в `primitive_invariants.rs:780–851`). Падение происходит исключительно на `Sliver quality gate`.
  3. Механизм образования 85 игольчатых треугольников в `crates/mcp-cad-core/src/csg.rs`:
     - При вычитании многогранного цилиндра отверстия из `box` в `BspNode::clip_polygons` (`csg.rs:285–325`) верхняя и нижняя квадратные грани пластины ($0.1 \times 0.1$ м) последовательно рассекаются плоскостями всех боковых граней цилиндра `Plane::split_polygon` (`csg.rs:44–129`). Каждое секущее полупространство отсекает от широкой грани пластины узкие клиновидные полосы по касательным к окружности отверстия, вершины которых лежат на границе квадрата на расстоянии до $0.05$ м от отверстия, а ширина основания у дуги отверстия составляет миллиметры.
     - Затем на этапе 3 в `polygons_to_mesh` (`csg.rs:453–538`) на длинные секущие рёбра этих выпуклых полигонов нанизываются дополнительные коллинеарные канонические вершины (`splits`), после чего `triangulate_subdivided_convex_loop` (`csg.rs:721–835`) триангулирует каждый узкий выпуклый полигон в отдельности, соединяя вершины мелкой дуги отверстия ($r = 0.015$ м) с далёкими углами внешнего квадрата ($0.05$ м).
     - На этапе 4b `optimize_coplanar_delaunay_edges` (`csg.rs:592–690`) выполняет всего `4` прохода (`for _pass in 0..4`, строка 601) с блокировкой обоих треугольников после одного флипа в пассе (`locked_tris[t1_idx] = true; locked_tris[t2_idx] = true;`, строки 678–679) и без вставки промежуточных точек Штейнера (Steiner points / концентрического кольца вокруг отверстия) на широкой плоской грани. На веере из десятков смежных тонких треугольников 4 проходов с пошаговой блокировкой недостаточно для перестроения топологии веера, а длинные диагонали от мелких сегментов отверстия к далёким углам пластины геометрически не могут дать угол $> 1^\circ..5^\circ$ без локального переходного кольца либо полного конвергентного Delaunay-флипа.
- **Какая задача закрывает:** **P0-T7** («CSG: инварианты объёма, отверстия, водонепроницаемость»).

---

## 4. Сводка дополнительных дефектов примитивов, выявленных в P0-T2 (`known_bug_*`)

Помимо 4 активных падений в существующих тестах, параметрический суит `crates/mcp-cad-core/tests/primitive_invariants.rs` (созданный в **P0-T2**) выявил **8 краевых случаев**, помеченных `#[ignore = "BUG: ..."]`. Из них 3 случая напрямую соответствуют задачам **P0-T5**, **P0-T6**, **P0-T7**, а **5 случаев являются новыми краевыми дефектами генераторов примитивов**, не покрытыми текущими задачами P0-T4…T9:

| № | Тест в `primitive_invariants.rs` | Воспроизведение | Суть дефекта (`primitives.rs`) | Закрывающая задача |
|---|---|---|---|---|
| 1 | `known_bug_revolve_partial_angle_end_cap_slivers` | `cargo test -p mcp-cad-core --test primitive_invariants known_bug_revolve_partial_angle_end_cap_slivers -- --ignored` | Иглы (`min_angle = 0.3649° < 1°`) в `triangulate_simple_polygon_2d` на торцах частичного `create_revolve(90°)` | **P0-T5** |
| 2 | `known_bug_polygon_prism_chamfer_low_sides_slivers` | `cargo test -p mcp-cad-core --test primitive_invariants known_bug_polygon_prism_chamfer_low_sides_slivers -- --ignored` | `50.00% (12/24)` треугольников `< 5°` на неразбитых гранях фаски `create_polygon_prism` при `sides ∈ {3, 4}` | **P0-T6** |
| 3 | `known_bug_cut_hole_csg_sliver_quality_gate` | `cargo test -p mcp-cad-core --test primitive_invariants known_bug_cut_hole_csg_sliver_quality_gate -- --ignored` | 100 игл (`min_angle = 0.0244° < 1°`, `13.60% < 5°`) при разбиении граней в `cut_hole` / `csg_difference` | **P0-T7** |
| 4 | `known_bug_cone_sharp_auto_segments_odd_after_halving_slivers` | `cargo test -p mcp-cad-core --test primitive_invariants known_bug_cone_sharp_auto_segments_odd_after_halving_slivers -- --ignored` | `create_cone(0.05, 0.0, 0.25, 0)`: `calculate_segments(0.05)` возвращает `50`; после первого деления 2-к-1 (`50 -> 25`) `curr_segs = 25` нечётно (`curr_segs % 2 == 0` ложно в `primitives.rs:865`), дальнейшее разрежение у высокой вершины останавливается → `1.85% (50/2700)` треугольников `< 5°` | **Новая задача (P0-T10)** |
| 5 | `known_bug_cone_inverted_apex_missing_ring_halving` | `cargo test -p mcp-cad-core --test primitive_invariants known_bug_cone_inverted_apex_missing_ring_halving -- --ignored` | `create_cone(0.0, 0.01, 0.02, 64)`: ветка перевёрнутого острого конуса (`!has_bot_cap`, `primitives.rs:928–975`) не содержит логики 2-к-1 разрежения колец у нижней вершины → `4.55% (128/2816)` треугольников `< 5°` (`min_angle = 2.51°`) | **Новая задача (P0-T10)** |
| 6 | `known_bug_spur_gear_high_teeth_cap_slivers` | `cargo test -p mcp-cad-core --test primitive_invariants known_bug_spur_gear_high_teeth_cap_slivers -- --ignored` | `create_spur_gear(0.001, 40, 20.0, 0.005, ...)`: при `teeth >= 36` переходные треугольники крышки между `root_poly` и основанием ножки зуба (`primitives.rs:1735–1747`) дают `1.63% (160/9800)` треугольников `< 5°` | **Новая задача (P0-T10)** |
| 7 | `known_bug_revolve_360_axis_fan_slivers_at_128_segments` | `cargo test -p mcp-cad-core --test primitive_invariants known_bug_revolve_360_axis_fan_slivers_at_128_segments -- --ignored` | `create_revolve` ($360^\circ$) на профиле с точками на оси $r = 0$ при `segments = 128` строит полюсный веер напрямую из 128 секторов (`primitives.rs:2103–2108`) без 2-к-1 разрежения → угол при полюсе $360^\circ / 128 = 2.8125^\circ < 5^\circ$ (`7.41%` slivers) | **Новая задача (P0-T10)** |
| 8 | `known_bug_wheel_rim_low_spoke_count_slivers` | `cargo test -p mcp-cad-core --test primitive_invariants known_bug_wheel_rim_low_spoke_count_slivers -- --ignored` | `create_wheel_rim` при `spokes = 3, bolts = 4` (с дефолтными `hub_diameter`/`pcd_diameter`): неравномерный угловой шаг `hub_outer_ring` в `add_subdivided_disk_cap_2d` даёт `1.76%..3.30%` треугольников `< 5°` | **Новая задача (P0-T10)** |

---

## 5. Предложение новой задачи для покрытия 5 краевых дефектов из P0-T2

Согласно требованиям **P0-T3** («Если нашёл падения, не покрытые задачами, предложи новую задачу в том же формате»), ниже сформулирована задача **P0-T10** для устранения 5 краевых случаев качества сетки примитивов и снятия `#[ignore]` со всех 8 тестов в `primitive_invariants.rs`:

```markdown
## P0-T10 — Устранение краевых sliver-треугольников в конусе, шестерне, полюсах revolve и ободе

**Размер:** M | **Зависит от:** P0-T2, P0-T5, P0-T6, P0-T7 | **Ветка:** `agent/P0-T10`

### Контекст (проверено в P0-T2 / P0-T3)
В `crates/mcp-cad-core/tests/primitive_invariants.rs` зафиксированы 5 краевых случаев нарушения `Sliver quality gate` (`lt_5_deg_ratio > 1.0%`), не входящих в P0-T5…T7:
1. `create_cone(0.05, 0.0, 0.25, 0)`: `TessellationSettings::calculate_segments` возвращает число сегментов, не кратное степени двойки (например, `50`), из-за чего в `primitives.rs:865` после одного шага 2-к-1 (`50 -> 25`) условие `curr_segs % 2 == 0` блокирует дальнейшее разрежение у вершины конуса (`1.85%` треугольников `< 5°`).
2. `create_cone(0.0, 0.01, 0.02, 64)`: ветка перевёрнутого острого конуса (`!has_bot_cap`, `primitives.rs:928–975`) строит боковую поверхность без 2-к-1 разрежения колец у нижней вершины (`4.55%` треугольников `< 5°`).
3. `create_spur_gear(0.001, 40, 20.0, 0.005, ...)`: при `teeth >= 36` сшивка основания зуба с `root_poly` (`primitives.rs:1726–1747`) даёт `1.63%` треугольников `< 5°`.
4. `create_revolve` ($360^\circ$) для профиля, касающегося оси $r = 0$, при `segments >= 72` (`128`) строит полюсный веер без 2-к-1 разрежения (`primitives.rs:2103–2108`), давая угол $360^\circ / 128 = 2.81^\circ < 5^\circ$.
5. `create_wheel_rim` при `spokes ∈ {3, 4}` с параметрами по умолчанию порождает `1.76%..3.30%` треугольников `< 5°` при замыкании ступицы.

### Задача
1. В `create_cone` округлять/приводить число сегментов острого конуса к значению, допускающему каскадное деление 2-к-1 до `<= 16` у вершины, и добавить симметричное 2-к-1 разрежение в ветку перевёрнутого конуса (`!has_bot_cap`).
2. В `create_spur_gear` скорректировать плотность `root_poly` / высоту первого уровня ножки зуба `radial_levels[1]`, чтобы при `teeth ∈ 8..=60` доля углов `< 5°` оставалась `< 1.0%`.
3. В `create_revolve` для сегментов профиля, примыкающих к оси вращения $r = 0$, использовать `add_subdivided_disk_cap_2d` (или 2-к-1 разрежение колец) вместо прямого полюсного веера при большом числе сегментов.
4. В `create_wheel_rim` устранить узкие треугольники перехода ступицы при `spokes ∈ {3, 4}`.
5. Снять атрибут `#[ignore]` со всех `known_bug_*` тестов в `crates/mcp-cad-core/tests/primitive_invariants.rs`.

### Критерии приёмки
- `cargo test -p mcp-cad-core --test primitive_invariants` проходит: `18 passed; 0 failed; 0 ignored`.
- Все тесты `mcp-cad-core` и `qa_wide_scan` проходят без регрессий; пороги `MAX_ALLOWED_SLIVER_RATIO` не менялись.
```

---

## 6. Итоговая матрица распределения по задачам Фазы 0 (P0-T4 … P0-T10)

| Задача | Статус | Что именно закрывает по итогам триажа P0-T3 |
|---|---|---|
| **P0-T4** | 📋 BACKLOG | Падение `mcp_contract_tests::test_bug016_revolve_boundary_and_self_intersection_validation` (приведение ассерта к `result.isError == true` через `assert_tool_error(&resp)` и документирование в `docs/MCP_API_REFERENCE.md`). |
| **P0-T5** | 📋 BACKLOG | Падение `qa_wide_scan::test_bug_003_partial_revolve_mesh_and_renders` и `primitive_invariants::known_bug_revolve_partial_angle_end_cap_slivers` (устранение игл `< 1°` в `triangulate_simple_polygon_2d` для плоских торцевых крышек частичного `create_revolve` + тест объёма по теореме Паппа). |
| **P0-T6** | 📋 BACKLOG | Падение `qa_wide_scan::test_bug_005_polygon_prism_chamfer_and_sides_validation` и `primitive_invariants::known_bug_polygon_prism_chamfer_low_sides_slivers` (подразбиение узких четырёхугольников фаски и крышек в `create_polygon_prism` при `sides ∈ 3..=12`). |
| **P0-T7** | 📋 BACKLOG | Падение `qa_wide_scan::test_bug_006_csg_booleans_and_holes_watertight` и `primitive_invariants::known_bug_cut_hole_csg_sliver_quality_gate` (устранение игл `< 1°` и избытка углов `< 5°` при булевом вычитании отверстий в `csg.rs` + создание `tests/csg_invariants.rs`). |
| **P0-T8** | 📋 BACKLOG | Инфраструктура и гигиена: `rust-toolchain.toml` (1.91), `Dockerfile` (`rust:1.91-bookworm`), `.github/workflows/ci.yml`, дисклеймеры в `qa/*.md` и `README.md`. |
| **P0-T9** | 📋 BACKLOG | Верификация правила `R4` (оба существующих теста `test_r4_coaxial_bushing_cylinder_hole_to_revolve` и `test_mcp_optimize_feature_tree_cbo` уже проходят после P0-T1): усиление проверки эквивалентности `validate_mesh` в `query_optimizer.rs` и добавление негативного теста на неосевое отверстие. |
| **P0-T10** *(предложена)* | 📋 BACKLOG | 5 краевых `known_bug_*` из `primitive_invariants.rs` (острый конус с `segments=0`, перевёрнутый конус `r_bot=0`, шестерня `teeth >= 36`, полюс `revolve` при `segments=128`, обод `spokes ∈ {3, 4}`). |
