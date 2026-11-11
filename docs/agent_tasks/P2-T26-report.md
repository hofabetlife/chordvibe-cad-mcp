# Отчёт о выполнении задачи P2-T26

**Задача:** `P2-T26 [S]` — Вывод `revision`, `bounding_box`, `volume`, `warnings` и учёт `resolve_length_unit(args)` в `boolean_difference`/`boolean_union`/`boolean_intersection` (`tools/cad.rs`) и `limits`/`axis`/`ratio` шарниров в `load_project` (`project.rs`)  
**Дата:** 09.11.2026  
**Статус:** ✅ DONE  
**Затронутые файлы:**
- `crates/mcp-cad-server/src/tools/cad.rs`
- `crates/mcp-cad-server/src/tools/project.rs`
- `crates/mcp-cad-server/src/tools/mod.rs`
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs`
- `src/cad/mcp_dispatcher.ts`
- `src/cad/cad_engine.test.ts`
- `docs/MCP_API_REFERENCE.md`
- `TASKS.md`
- `docs/agent_tasks/P2-T26-report.md`

---

## 1. Диагностика проблемы

При аудите MCP-контракта в рамках **P2-T21** были выявлены два несоответствия в ответах инструментов:

1. **Неполные метаданные и игнорирование переопределения `length_unit` в булевых операциях (`crates/mcp-cad-server/src/tools/cad.rs`)**:
   - В `boolean_difference` (`cad.rs:1898-1964`), `boolean_union` (`cad.rs:2722-2796`) и `boolean_intersection` (`cad.rs:2799-2873`) единица измерения длины бралась напрямую из `state.length_unit` вместо `state.resolve_length_unit(args)?`, из-за чего передача локального аргумента `"length_unit": "m"` (или `"mm"`) игнорировалась.
   - В JSON-ответах `boolean_difference`, `boolean_union` и `boolean_intersection` отсутствовали поля `"revision"`, `"bounding_box"` (`min`, `max`, `dimensions` в активном `length_unit`), `"volume"` (в `mm³` или `m³`), `"volume_m3"` и `"warnings"` от `state.check_scale_sanity(target_part, &[], None, unit)` (а в `boolean_difference` также отсутствовали `"part_id"`, `"target_part_id"`, `"vertices_count"` и `"triangles_count"`, и в `transform_part` отсутствовало поле `"revision"`).
2. **Отсутствие `limits`, `limits_unit`, `axis` и `ratio` в сводке шарниров `load_project` (`crates/mcp-cad-server/src/tools/project.rs:387-402`)**:
   - При загрузке проекта через `load_project` элементы массива `joints` содержали только `joint_id`, `part_a`, `part_b`, `joint_type` и `has_motor`, опуская `anchor_a`, `anchor_b`, `axis`, `limits` (с переводом из радиан в градусы `deg` для `Revolute` и из метров в активный `length_unit` для `Prismatic`), `limits_unit`, `ratio` (для `Gear`) и детали `motor` (`target_velocity`, `max_force`).

---

## 2. Выполненные изменения

1. **Единообразные метаданные, `resolve_length_unit(args)` и `warnings` в CSG-инструментах (`crates/mcp-cad-server/src/tools/cad.rs`, `tools/mod.rs`, `src/cad/mcp_dispatcher.ts`)**:
   - В `boolean_difference`, `boolean_union` и `boolean_intersection` подключено разрешение единицы длины `let unit = state.resolve_length_unit(args)?;` (до мутации состояния детали) и расчёт предупреждений масштаба `state.check_scale_sanity(target_part, &[], None, unit)`.
   - Во все три булевых инструмента добавлен возврат полей `"part_id"`, `"target_part_id"`, `"feature_id"`, `"revision"`, `"vertices_count"`, `"triangles_count"`, `"bounding_box"` (`min`, `max`, `dimensions`), `"volume"`, `"volume_m3"`, `"length_unit"`, `"units"` и `"warnings"`.
   - В `transform_part` добавлено поле `"revision": part_rev` для полной консистентности со всеми мутирующими CAD-инструментами.
   - В `crates/mcp-cad-server/src/tools/mod.rs` в `input_schema` для `boolean_difference`, `boolean_union` и `boolean_intersection` добавлено опциональное свойство `"length_unit": { "type": "string", "enum": ["mm", "m"] }`.
   - Синхронизирован TypeScript-диспетчер `src/cad/mcp_dispatcher.ts`.
2. **Полная сводка шарниров и ревизий деталей в `load_project` (`crates/mcp-cad-server/src/tools/project.rs`)**:
   - В `load_project` для каждого шарнира в массиве `joints` добавлен вывод:
     - `anchor_a`, `anchor_b` (в активном `length_unit` проекта);
     - `axis` (для `Revolute`, `Prismatic` и `Gear`, плюс `axis_a` и `axis_b` для `Gear`);
     - `limits` (`[min_deg, max_deg]` для `Revolute`, `[min_u, max_u]` для `Prismatic`, иначе `null`) и `limits_unit` (`"deg"`, `"mm"`/`"m"` или `null`);
     - `ratio` (для `Gear`, иначе `null`);
     - `has_motor` и `motor` (`{"target_velocity", "max_force"}` или `null`).
   - В массив `parts` ответа `load_project` добавлены `"revision"` и `"volume_m3"`.
3. **Тесты**:
   - В `crates/mcp-cad-server/src/tools/cad.rs` добавлен модульный тест `test_p2_t26_boolean_ops_metadata_unit_override_and_warnings`.
   - В `crates/mcp-cad-server/src/tools/project.rs` добавлен модульный тест `test_p2_t26_load_project_joints_limits_axis_and_ratio`.
   - В `crates/mcp-cad-server/tests/mcp_contract_tests.rs` добавлен интеграционный контрактный тест `test_mcp_p2_t26_boolean_ops_metadata_and_load_project_joint_details`.
   - В `src/cad/cad_engine.test.ts` добавлен тест `verifies P2-T26: boolean_difference/union/intersection return revision, bounding_box, volume, warnings, and respect length_unit`.

---

## 3. Верификация

- `npx --yes tsx --test src/cad/cad_engine.test.ts` — **30 passed; 0 failed**.
- `npm run lint` (`tsc --noEmit`) — **0 errors**.
- Сборка приложения (`compile_applet` + `lint_applet`) — **PASS**.

---

## 4. Новые найденные проблемы (заведены в `TASKS.md` со статусом `BACKLOG`)

- **`P2-T29 [S]`**: В `crates/mcp-cad-server/src/tools/{cad.rs, metrics.rs}`:
  1. В `list_parts` (`crates/mcp-cad-server/src/tools/cad.rs:349-361`) единица длины берётся из `state.length_unit` напрямую вместо `state.resolve_length_unit(args)?` (и в `input_schema` `list_parts` в `tools/mod.rs` отсутствует опциональный параметр `length_unit`), из-за чего передача `{"length_unit": "m"}` в `list_parts` игнорируется.
  2. В `get_scene_summary` (`crates/mcp-cad-server/src/tools/metrics.rs:333-343`) для шарнира `CadJointType::Gear { ratio, axis_a, axis_b }` поле `axis` в JSON-массиве `joints` возвращается как `null` (`None`) вместо `Some(*axis_a)` (в отличие от `load_project` и текстовой сводки `summary_text`), а поля `axis_a` и `axis_b` не включаются в JSON-объект шарнира.
