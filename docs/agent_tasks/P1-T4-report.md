# Отчёт о выполнении задачи P1-T4

## 1. Задача и что было сделано
- **ID**: `P1-T4`
- **Название**: Шаблон задач реализации выбранного ядра
- **Фаза**: P1 — Архитектура и Геометрическое Ядро
- **Ветка**: `agent/P1-T4`

### Выполненные работы и созданные файлы
В соответствии со спецификацией задачи `P1-T4` (`fix/P1_architecture.md`: *«Прочитай `docs/adr/0001-geometry-kernel.md`, раздел «Оценка работ». Для каждого пункта создай файл-промт задачи в формате пакета `agent_prompts` (контекст с проверенными фактами и путями, задача, критерии приёмки командами, «не делать», размер S/M, зависимости), чтобы задачу можно было выполнить в пустом контексте. Не пиши код. Результат — `agent_prompts/P1_kernel_migration.md`»*):

1. **Подготовлен самодостаточный пакет промтов миграции ядра [`agent_prompts/P1_kernel_migration.md`](../../agent_prompts/P1_kernel_migration.md)**:
   - Включена актуализированная **Преамбула** (на базе `fix/00_README_PREAMBLE.md`), отражающая текущее верифицированное состояние репозитория после завершения `P0-T1..P0-T12` и `P1-T1..P1-T3` (`156/156` тестов `PASS`, `0` предупреждений `clippy`, наличие эталонных спайков `spikes/manifold_rust/` и `spikes/monstertruck_brep/`).
   - Сформирована сводная таблица зависимостей и матрица соответствия с задачами Фазы 2 (`P2-T5..P2-T8`) и Фазы 3 (`P3-T11`, `P3-T12`).
   - Детально расписаны 5 основных задач реализации выбранного ядра `manifold-rust v0.19.2` (`P1-K1`…`P1-K5`) и 1 опциональная задача гибридного STEP-экспорта через `monstertruck v0.4.1` (`P1-K6`):
     - **`P1-K1` [S]** (`agent/P1-K1`, зависит от `P1-T3`): подключение `manifold-rust = "0.19.2"` в `crates/mcp-cad-core/Cargo.toml`, добавление `#[serde(default)] pub face_ids: Vec<u32>` в `TriangleMesh` (`topology.rs:150`) и безпотерьный конвертер `mesh_to_manifold` / `manifold_to_mesh` со сваркой вершин `(diag * 1e-7).max(1e-12)` (`diagnostics.rs:224-266`) и расщеплением нормалей на острых рёбрах.
     - **`P1-K2` [M]** (`agent/P1-K2`, зависит от `P1-K1`): перевод `csg_union` (`csg.rs:2704`), `csg_difference` (`:2730`), `csg_intersection` (`:2759`) на `Manifold` в связке с постобработкой `optimize_csg_mesh_quality` (`csg.rs:808`) для гарантированного прохождения Sliver Gate (`lt_1_deg_count == 0`, `lt_5_deg_ratio <= 1.0%`), а также исправление формулы `genus` для $k$ связных компонент в `diagnostics.rs:414` (закрывает **`P3-T12`**).
     - **`P1-K3` [S]** (`agent/P1-K3`, зависит от `P1-K2`): доведение и верификация вычитания винтовой резьбы `FeatureOp::CutHelicalGroove` (`dag.rs:644-677`, `create_helical_cutter_with_settings` в `primitives.rs:1499`) на гайке М10×1.5 (`T08_hex_nut_m10`) и спиральном сверле (`test_dag_helical_groove_drill_bit_model`) до `validate_mesh == PASS`, `is_watertight == true`, `open_boundary_edges == 0`, `genus == 1.0` (закрывает **`P3-T11`**).
     - **`P1-K4` [M]** (`agent/P1-K4`, зависит от `P1-K2`): дизайн (`docs/FACE_NAMING_DESIGN.md`) и сквозная реализация именованных граней (`FaceInfo`, `FaceKind`, `face_ids`) в примитивах (`primitives.rs`), трансформациях (`topology.rs`) и булевых операциях через `MeshGLP::face_id` (`csg.rs`) с выдачей в `query_part_geometry` (`crates/mcp-cad-server/src/tools/cad.rs:1359`) — консолидирует **`P2-T5` + `P2-T6` + `P2-T7`**.
     - **`P1-K5` [M]** (`agent/P1-K5`, зависит от `P1-K4`): реализация `create_extrude_sketch_with_settings` (`primitives.rs`), `FeatureOp::ExtrudeSketch` (`dag.rs`) и поддержки `primitive_type: "extrude_sketch"` в `add_feature_primitive` / `modify_feature` (`tools/cad.rs`) на базе `manifold_rust::cross_section::CrossSection` (2D-контуры с отверстиями и скруглением углов `corner_radius` через `CrossSection::offset` + `Manifold::extrude`), закрывая исторический долг `TASK-007`/`TASK-008`.
     - **`P1-K6` [M] (опционально по `[DECISION-4]`)** (`agent/P1-K6`, зависит от `P1-T3`): экспорт параметрического дерева `FeatureTree` в ISO 10303-21 `STEP` (`export_feature_tree_to_step`) на базе `monstertruck-modeling` / `monstertruck-solid` / `monstertruck-step`.

2. **Обновлена документация в [`TASKS.md`](../../TASKS.md)**:
   - Задача `P1-T4` переведена в статус `✅ DONE` со ссылкой на `agent_prompts/P1_kernel_migration.md` и настоящий отчёт.
   - Зарегистрирована новая побочная находка **`P2-T14`**, выявленная при проверке исходников `dag.rs` и `topology.rs`.

---

## 2. Как проверено (точные команды и вывод)

Поскольку задача `P1-T4` является документационно-архитектурной («Не пиши код»), исходный код крейтов не модифицировался. Проверены все ссылки на файлы/строки и контрольное прохождение линтера и тестов:

```bash
export PATH="/root/.cargo/bin:$PATH"
cargo clippy --workspace --all-targets -- -D warnings
cargo test -p mcp-cad-core --lib
```

Итоговые строки вывода:
```text
Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.35s
Finished `test` profile [unoptimized + debuginfo] target(s) in 0.20s
Running unittests src/lib.rs (target/debug/deps/mcp_cad_core-223821f2875c6c75)
test result: ok. 50 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 27.34s
```

---

## 3. Тесты до и после

- **До изменений**: `156 passed, 0 failed, 0 ignored` (`cargo test --workspace`), `0 warnings` (`cargo clippy --workspace --all-targets -- -D warnings`).
- **После изменений**: `156 passed, 0 failed, 0 ignored`, `0 warnings` (код крейтов не изменялся).

---

## 4. Остаточные риски и заметки по выполнению `P1-K1..P1-K6`

- Как отмечено в `fix/P1_architecture.md`, задачи `P2-T5..P2-T8` (именованные грани) напрямую зависят от выбранного в `P1-T3` ядра. В `agent_prompts/P1_kernel_migration.md` показано, что при использовании `manifold-rust v0.19.2` задачи `P2-T5`, `P2-T6` и `P2-T7` объединяются в задачу **`P1-K4` [M]**, так как `MeshGLP::face_id` автоматически сохраняет идентификаторы граней при булевых операциях.
- При выполнении задачи `P1-K5` (`extrude_sketch`) агент должен учитывать, что контрактные тесты в `crates/mcp-cad-server/tests/mcp_contract_tests.rs` проверяют список инструментов `list_tools()`; поэтому `extrude_sketch` безопасно вводится как новый `primitive_type: "extrude_sketch"` внутри `add_feature_primitive` и `modify_feature` (либо при добавлении отдельного тула одновременно обновляется ожидаемое число тулов в контрактном тесте).

---

## 5. Побочные находки (занесены в `TASKS.md` как `P2-T14`, не исправлялись в рамках `P1-T4`)

При проверке номеров строк и сигнатур в `crates/mcp-cad-core/src/dag.rs` и `crates/mcp-cad-core/src/topology.rs` обнаружены два дефекта, зарегистрированные в `TASKS.md` под номером **`P2-T14`**:

1. **Игнорирование пользовательских параметров дискретизации `segments` / `sectors` / `stacks` при вычислении `FeatureTree::evaluate_with_settings` (`crates/mcp-cad-core/src/dag.rs:396-572`)**:
   - В `crates/mcp-cad-server/src/tools/cad.rs` (`add_feature_primitive` и `modify_feature`, строки 640, 655–656, 673, 775) сервер парсит и сохраняет пользовательские параметры `segments`, `sectors`, `stacks` в поля перечислений `FeatureOp::AddCylinder { segments, .. }`, `FeatureOp::AddSphere { sectors, stacks, .. }`, `FeatureOp::AddCone { segments, .. }`, `FeatureOp::AddRevolve { segments, .. }`.
   - Однако в `crates/mcp-cad-core/src/dag.rs` метод `FeatureTree::evaluate_with_settings` игнорирует эти поля паттерном `segments: _` (строки 399, 483, 546) и `sectors: _, stacks: _` (строки 460–461), безусловно перезаписывая их адаптивным расчётом `settings.calculate_segments(...)`. В результате явная передача `"segments": 16` или `"segments": 64` через MCP API в `add_feature_primitive` или `modify_feature` не оказывает никакого влияния на построенную сетку детали.
2. **Слияние несвязных копланарных граней, размерный порог `1e-4` и центроид в пустоте отверстия в `TriangleMesh::inspect_geometry` (`crates/mcp-cad-core/src/topology.rs:440-498`)**:
   - Метод `inspect_geometry()` группирует треугольники в `detected_planar_faces` только по близости нормали (`dot > 0.999`) и абсолютному расстоянию до плоскости `|n · p - d| < 1e-4` (строка 448):
     - Порог `1e-4` зашит в единицах модели (в СИ — `0.1 мм`), поэтому у мелких деталей в метрах параллельные ступеньки на расстоянии `< 0.1 мм` ошибочно сливаются в одну грань.
     - Отсутствует проверка связности по общим рёбрам, из-за чего раздельные копланарные площадки на одной высоте $Z$ сливаются в одну грань `PlanarFaceInfo`.
     - Поле `center` вычисляется как взвешенное среднее центроидов треугольников, которое для кольцевого торца полого цилиндра или шайбы попадает в пустое отверстие `[0, 0, z]`, а не на поверхность грани.
