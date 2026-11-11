# Отчёт о выполнении задачи P2-T19

## Задача
**P2-T19 [S]**: Транзакционный откат `part.dag`/`part.mesh` при ошибке `rebuild()` в `modify_feature` и авто-выравнивание `start_z` по нижней грани детали в `cut_helical_groove`.

---

## Диагностика проблемы

1. **Неполный транзакционный откат и пропуски валидации в `modify_feature` (`crates/mcp-cad-server/src/tools/cad.rs`, `crates/mcp-cad-core/src/{part.rs, dag.rs}`)**:
   - В `Part` (`crates/mcp-cad-core/src/part.rs`) помимо `dag` и `mesh` хранятся `cached_mesh: Option<TriangleMesh>`, `cached_settings: Option<TessellationSettings>` и монотонный счётчик `revision: u64`, причём `Part::mesh(&self)` возвращает `self.cached_mesh`, если он заполнен (например, после вызова `tessellate_part` с качеством `"fine"`). При откате только `part.dag` и `part.mesh` без восстановления `cached_mesh`, `cached_settings` и `revision` состояние детали могло рассинхронизироваться, а в `Part::optimize_dag(true)` (`part.rs:95–107`) откат при ошибке `self.rebuild()` отсутствовал вовсе.
   - В `modify_feature`:
     - В ветке `FeatureOp::AddRevolve` передача пустого или некорректного массива `"profile": []` негласно игнорировалась и возвращала старый `profile.clone()` вместо ошибки `INVALID_ARGUMENT`, а 2-точечный профиль с `inner_radius` не проверял `inner_r >= 0`, `inner_r < outer_r` и ненулевую осевую высоту `|z1 - z0| > 0`.
     - В ветке `FeatureOp::Transform` обращение по индексам `arr[0]`, `arr[1]`, `arr[2]` для `translation`, `rotation_deg` и `scale` выполнялось без проверки `arr.len() == 3`, что приводило к панике (`index out of bounds`) при передаче массива из 1–2 элементов.
     - В `FeatureTree::evaluate_with_settings` (`crates/mcp-cad-core/src/dag.rs`) отсутствовала полная проверка конечности (`is_finite()`), положительности размеров (`width_across_flats > 0`, `radius > 0`, `chamfer >= 0` в `AddPolygonPrism`; `helix_radius > 0`, `starts >= 1` в `CutHelicalGroove`; ненулевой вектор `normal` в `CutHole`) и проверка отсутствия 2D-самопересечений полигонального контура в `AddRevolve`.

2. **Смещение по умолчанию `start_z = 0.0` в `cut_helical_groove` (`crates/mcp-cad-server/src/tools/{cad.rs, mod.rs}`)**:
   - Все базовые примитивы ядра (`polygon_prism`, `cylinder`, `box`) центрированы относительно начала координат `z = 0` (`z ∈ [-height/2, +height/2]`).
   - В `cut_helical_groove` при опущенном аргументе `start_z` подставлялось значение `0.0` (а в JSON-схеме `tools/mod.rs` поле `"start_z"` числилось в `"required"`). В результате при вызове `cut_helical_groove` без `start_z` (как в эталонной задаче `T08_hex_nut_m10` из `bench/tasks.json`) резьба нарезалась только в верхней половине гайки `[0, +height]` вместо диапазона детали `[-height/2, +height/2]`.

---

## Выполненные изменения

1. **Полный транзакционный откат состояния `Part` (`crates/mcp-cad-core/src/part.rs`, `crates/mcp-cad-server/src/tools/cad.rs`)**:
   - В `modify_feature`, `add_feature_primitive`, `cut_hole`, `cut_helical_groove`, `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part` (`tools/cad.rs`) и `Part::optimize_dag` (`part.rs`) сохраняются и при любой ошибке `modify_feature`/`rebuild()` восстанавливаются все 5 полей состояния: `dag`, `mesh`, `cached_mesh`, `cached_settings` и `revision`.
2. **Строгая валидация параметров в `modify_feature` и `FeatureTree::evaluate_with_settings` (`crates/mcp-cad-core/src/dag.rs`, `crates/mcp-cad-server/src/tools/cad.rs`)**:
   - В `modify_feature` добавлена валидация длины 3-элементных массивов `translation`, `rotation_deg`, `scale`, `position`/`center` и `normal`/`direction` (устранена паника `arr[2]` в `FeatureOp::Transform`), а также строгая валидация `profile` и `inner_radius` в `FeatureOp::AddRevolve`.
   - В `FeatureTree::evaluate_with_settings` (`dag.rs`) добавлена проверка `is_finite()`, положительности геометрических параметров во всех вариантах `FeatureOp` и проверка отсутствия самопересечений контура `AddRevolve` (`revolve_profile_has_self_intersection`).
3. **Авто-выравнивание `start_z` по нижней грани детали в `cut_helical_groove` (`crates/mcp-cad-server/src/tools/{cad.rs, mod.rs}`, `src/cad/mcp_dispatcher.ts`)**:
   - При отсутствии `start_z` начальная координата автоматически вычисляется как `part.bounding_box().min[2]` (если сетка детали непуста) или `-height * 0.5` (если деталь пуста).
   - Поле `"start_z"` исключено из `"required"` в JSON-схеме `cut_helical_groove` (`crates/mcp-cad-server/src/tools/mod.rs`) и возвращается в ответе инструмента в активной единице `length_unit`.
   - Синхронизировано поведение `cut_helical_groove` и `modify_feature` в TypeScript-диспетчере `src/cad/mcp_dispatcher.ts` и `src/cad/primitives.ts`.

---

## Новые находки (заведены в `TASKS.md` со статусом `BACKLOG`)

- **P2-T24 [S]**: Вызов `export_stl` (`crates/mcp-cad-server/src/tools/cad.rs:1947`) вызывает `part.tessellate(&settings)` с качеством `"fine"` по умолчанию, что в `Part::tessellate` (`crates/mcp-cad-core/src/part.rs:73`) инкрементирует `part.revision += 1` при отличии настроек от кэшированных. Из-за этого экспорт детали в STL после расчёта МКЭ (`run_fea_analysis`) увеличивает `part.revision` и ложно переводит инженерное требование `min_safety_factor` (`check_requirements`) и флаг `get_scene_summary.last_fea.stale` в статус **`STALE`**, хотя параметрический DAG детали не менялся.

---

## Верификация и тесты

- Добавлен unit-тест `test_p2_t19_modify_feature_rollback_and_helical_groove_auto_start_z` в `crates/mcp-cad-server/src/tools/cad.rs`.
- Добавлен интеграционный контрактный тест `test_mcp_p2_t19_modify_feature_transactional_rollback_and_helical_groove_start_z` в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`.
- Добавлен unit-тест `verifies P2-T19: transactional rollback of part.dag/part.mesh in modify_feature and auto start_z alignment in cut_helical_groove` в `src/cad/cad_engine.test.ts` (**29 / 29 PASS**).
