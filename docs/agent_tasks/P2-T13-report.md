# Отчёт о выполнении задачи P2-T13

## 1. Задача
**P2-T13**: Поддержка пользовательского `joint_id` в `create_joint`, `through: true` в `cut_hole` и алиасов параметров (`revolve.inner_radius`, `polygon_prism.outer_radius`/`chamfer_radius`, ID материалов).

## 2. Выявленные проблемы (Root Cause)
По результатам прогона эталонного бенчмарка 15 задач (`bench/tasks.json`, `docs/BENCH_BASELINE_RUST.md`) были зафиксированы расхождения между входными контрактами ИИ-агентов и реализацией инструментов:
1. **`create_joint` игнорировал пользовательский `joint_id`** (`crates/mcp-cad-server/src/tools/physics.rs:280`): всегда генерировался случайный идентификатор `joint-<uuid>`, из-за чего последующий вызов `set_joint_motor` с переданным `joint_id` (например, в задачах `T14` и `T15`) завершался ошибкой `JOINT_NOT_FOUND`.
2. **`cut_hole` не поддерживал `"through": true` без `depth` и алиасы `direction` / `diameter`** (`crates/mcp-cad-server/src/tools/cad.rs:860–965`): при передаче `"through": true` без `"depth"` возвращалась ошибка `Missing required argument 'depth'`.
3. **`revolve` не поддерживал `inner_radius` при 2-точечном образующем профиле** (`crates/mcp-cad-server/src/tools/cad.rs:627–723`): контур `profile: [[r_out, z0], [r_out, z1]]` с параметром `inner_radius` (задача `T07_annular_bushing_revolve`) отклонялся проверкой `profile.len() < 3`.
4. **`polygon_prism` не поддерживал алиасы `outer_radius` и `chamfer_radius`** (`crates/mcp-cad-server/src/tools/cad.rs:519–564`): в задаче `T08_hex_nut_m10` передача `outer_radius` вместо `radius` приводила к ошибке отсутствия радиуса.
5. **`MaterialRegistry` не поддерживал распространённые алиасы материалов** (`crates/mcp-cad-core/src/material.rs`): идентификаторы `"steel_1045"`, `"steel_4140"`, `"alu_6061"`, `"Al6061-T6"`, `"Al-6061-T6"`, `"ti_6al_4v"`, `"Ti-6Al-4V"`, `"Ti6Al4V"`, `"brass_c36000"`, `"pla_3d"`, `"petg_3d"`, `"nylon_pa6"` отклонялись при вызове `create_part`.

## 3. Внесённые изменения
1. **`crates/mcp-cad-core/src/material.rs` и `src/cad/materials.ts`**:
   - Добавлен метод `MaterialRegistry::resolve_alias(name: &str) -> &str`, нормализующий регистр, дефисы и пробелы и сопоставляющий популярные инженерные обозначения (`steel_1045`, `steel_1020`, `AISI 1020`, `steel_4140`, `AISI 4140`, `alu_6061`, `Al6061-T6`, `Al-6061-T6`, `ti_6al_4v`, `Ti-6Al-4V`, `Ti6Al4V`, `brass_c36000`, `pla_3d`, `pla_plastic`, `petg_3d`, `petg_plastic`, `nylon_pa6`) с 8 базовыми пресетами без дублирования элементов в `list()` / `len()`.
   - Аналогичная таблица `resolveMaterialAlias` добавлена в `src/cad/materials.ts`.
2. **`crates/mcp-cad-server/src/tools/physics.rs` и `src/cad/mcp_dispatcher.ts`**:
   - В `create_joint` добавлено чтение опционального `args.get("joint_id").or_else(|| args.get("id"))` с валидацией непустой строки (`INVALID_ARGUMENT` на параметре `joint_id`) и проверкой уникальности через `PhysicsWorld::add_joint` (`JOINT_ALREADY_EXISTS`). При отсутствии `joint_id` сохранена автогенерация `joint-<uuid>`.
3. **`crates/mcp-cad-server/src/tools/cad.rs`, `src/cad/primitives.ts` и `src/cad/mcp_dispatcher.ts`**:
   - В `cut_hole` и `modify_feature` (`FeatureOp::CutHole`) добавлена поддержка флага `"through": true` (с автоматическим вычислением полной сквозной глубины `compute_through_hole_depth_m` по проекции вершин сетки детали вдоль оси сверления), а также алиасов `"direction"` (для `"normal"`) и `"diameter"` (`* 0.5`, для `"radius"`).
   - В `add_feature_primitive` и `modify_feature` для `revolve` добавлена поддержка `inner_radius` при 2-точечном внешнем профиле `[[r0, z0], [r1, z1]]`: профиль автоматически разворачивается в замкнутый 4-точечный кольцевой контур `[[inner_r, z0], [r0, z0], [r1, z1], [inner_r, z1]]` с валидацией `0 <= inner_r < min(r0, r1)` и `z0 != z1`.
   - В `add_feature_primitive` и `modify_feature` для `polygon_prism` добавлены алиасы `outer_radius` (для `radius`) и `chamfer_radius` (для `chamfer`).
4. **`crates/mcp-cad-server/src/tools/mod.rs`**:
   - Обновлены JSON-схемы и описания инструментов `create_part`, `add_feature_primitive`, `cut_hole` и `create_joint` с явным указанием единиц измерения для всех новых числовых свойств.

## 4. Новые проблемы, обнаруженные при аудите
- **P2-T19**:
  1. В `modify_feature` (`crates/mcp-cad-server/src/tools/cad.rs:1006–1335`) при ошибке `part.rebuild()` после `part.dag.update_feature(...)` не выполняется откат `part.dag` и `part.mesh` к предыдущему валидному состоянию (в отличие от `add_feature_primitive`, `cut_hole` и `cut_helical_groove`), а также не обновляется тело детали в `state.physics_world`.
  2. В `cut_helical_groove` (`crates/mcp-cad-server/src/tools/cad.rs:1440–1444`) при опущенном параметре `start_z` используется значение по умолчанию `0.0` вместо нижней границы детали `part.bounding_box().min[2]` (или `-height * 0.5`), из-за чего на деталях, центрированных в начале координат (например, `T08_hex_nut_m10`), резьба без явного `start_z` нарезается только в верхней половине `[0, +height/2]`.

## 5. Верификация и тесты
- Добавлены модульные и интеграционные тесты:
  - `crates/mcp-cad-core/src/material.rs`: `test_material_preset_aliases`
  - `crates/mcp-cad-server/src/tools/physics.rs`: `test_create_joint_custom_joint_id_and_duplicate_rejection`
  - `crates/mcp-cad-server/src/tools/cad.rs`: `test_p2_t13_through_hole_revolve_inner_radius_and_polygon_prism_aliases`
  - `crates/mcp-cad-server/tests/mcp_contract_tests.rs`: `test_mcp_p2_t13_custom_joint_id_through_hole_and_aliases`
  - `src/cad/cad_engine.test.ts`: тест `supports P2-T13 custom joint_id, cut_hole through: true, revolve.inner_radius, polygon_prism aliases, and material ID aliases`
