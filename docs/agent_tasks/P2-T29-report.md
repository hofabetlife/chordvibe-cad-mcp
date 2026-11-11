# Отчёт о выполнении задачи P2-T29

**Задача:** Учёт `resolve_length_unit(args)` в `list_parts` (`tools/cad.rs:349-361`) и вывод `axis`/`axis_a`/`axis_b` для `CadJointType::Gear` в `get_scene_summary` (`metrics.rs:333-343`)  
**Фаза:** P2 (Agent Contract & Ergonomics)  
**Статус:** ✅ Выполнено  
**Дата:** 09.11.2026  

---

## 1. Диагностика проблемы

При выполнении задачи **P2-T26** ([`docs/agent_tasks/P2-T26-report.md`](P2-T26-report.md)) были выявлены два нарушения контракта единиц и консистентности кинематических метаданных:

1. **Игнорирование переопределения `length_unit` в `list_parts` (`crates/mcp-cad-server/src/tools/cad.rs:349-361` и `tools/mod.rs:140-150`)**:
   - В `list_parts` единица длины бралась напрямую из `state.length_unit` вместо вызова `state.resolve_length_unit(args)?`, а в `input_schema` инструмента `list_parts` в `crates/mcp-cad-server/src/tools/mod.rs` отсутствовало опциональное свойство `"length_unit": {"type": "string", "enum": ["mm", "m"]}`.
   - В результате при вызове `list_parts` с аргументом `{"length_unit": "m"}` (когда сессия находится в `"mm"`) габариты `bounding_box`, `bounding_box_dimensions` и объём `volume` возвращались в миллиметрах вместо запрошенных метров, а передача невалидного значения (например, `{"length_unit": "cm"}`) молча игнорировалась вместо возврата структурированной ошибки `INVALID_ARGUMENT` (`param: "length_unit"`). Кроме того, итерация `state.parts.values()` по `HashMap` не гарантировала детерминированного порядка деталей.

2. **Пропуск `axis`, `axis_a` и `axis_b` для `CadJointType::Gear` в `get_scene_summary` (`crates/mcp-cad-server/src/tools/metrics.rs:333-343, 418-438`)**:
   - В `get_scene_summary` при обходе `state.physics_world.joints()` для варианта `CadJointType::Gear { ratio, axis_a, axis_b }` кортеж возвращал `axis_opt = None`, из-за чего в JSON-массиве `joints` поле `"axis"` оказывалось равным `null`, а поля `"axis_a"` и `"axis_b"` вообще не добавлялись в `joint_obj` (и не отображались в `summary_text`), в отличие от `create_joint` (`tools/physics.rs`) и `load_project` (`tools/project.rs`).

---

## 2. Выполненные изменения

### 2.1. Разрешение `state.resolve_length_unit(args)?` и детерминированная сортировка в `list_parts` (`crates/mcp-cad-server/src/tools/cad.rs`, `tools/mod.rs`, `src/cad/mcp_dispatcher.ts`)
- В `list_parts` (`crates/mcp-cad-server/src/tools/cad.rs`):
  - Чтение `let unit = state.length_unit;` заменено на `let unit = state.resolve_length_unit(args)?;`, что обеспечивает как локальное переопределение единицы длины (`"mm"` или `"m"`) без мутации `state.length_unit`, так и строгую валидацию невалидных единиц (`INVALID_ARGUMENT`, `param: "length_unit"`).
  - Список деталей `state.parts.values()` предварительно сортируется по `a.id.cmp(&b.id)` для детерминированного порядка в JSON-ответе.
  - В JSON-ответ добавлено поле `"parts_count": summaries.len()`.
- В `crates/mcp-cad-server/src/tools/mod.rs` в `input_schema` инструмента №2 `list_parts` добавлено опциональное свойство `"length_unit": { "type": "string", "enum": ["mm", "m"], ... }`.
- В фронтенд-диспетчере `src/cad/mcp_dispatcher.ts` (`case 'list_parts'`) добавлена валидация и поддержка опционального `params.length_unit` (`"mm"` или `"m"`), масштабирование `bounding_box`, `bounding_box_dimensions`, `volume` и `volume_m3`, а также вывод `parts_count`, `length_unit` и `units`.

### 2.2. Вывод `axis`, `axis_a` и `axis_b` для `CadJointType::Gear` в `get_scene_summary` (`crates/mcp-cad-server/src/tools/metrics.rs`)
- В `get_scene_summary` (`crates/mcp-cad-server/src/tools/metrics.rs`) для варианта `CadJointType::Gear { ratio, axis_a, axis_b }`:
  - `axis_opt` теперь возвращает `Some(*axis_a)` (вместо `None`), а также заполняются `axis_a_opt = Some(*axis_a)` и `axis_b_opt = Some(*axis_b)`.
  - В текстовой сводке `summary_text` для зубчатой передачи выводятся `ratio`, `axis_a` и `axis_b`: `format!("ratio={ratio:.3}, axis_a=[{:.3}, {:.3}, {:.3}], axis_b=[{:.3}, {:.3}, {:.3}]", ...)`.
  - В JSON-объект шарнира `joint_obj` добавляются поля `"axis_a"` и `"axis_b"` наряду с `"axis": axis_a` и `"ratio": ratio`, что полностью синхронизирует схему `joints` в `get_scene_summary` с `create_joint` и `load_project`.

---

## 3. Новые проблемы, выявленные в ходе аудита (заведены в `TASKS.md`)

При проверке консистентности схем `input_schema` в `crates/mcp-cad-server/src/tools/mod.rs` и единицы длины в `metrics.rs` выявлена следующая задача, добавленная в `TASKS.md` со статусом `BACKLOG`:
- **`P2-T32 [S]`**: Добавление опционального `length_unit` в `input_schema` инструментов `get_feature_tree` и `validate_mesh` (`tools/mod.rs:278-290, 990-1008`), вывод `part_id`/`revision`/`features_count` в их ответах (`tools/cad.rs:1698, 2738`) и масштабирование `gravity` / вывод `joints_count` в `get_simulation_metrics_json` (`metrics.rs:62-66`).

---

## 4. Тестирование и верификация

Добавлены и прогнаны следующие тесты:
1. **`crates/mcp-cad-server/src/tools/cad.rs`**:
   - `test_p2_t29_list_parts_respects_length_unit_override_and_rejects_invalid_unit` — проверяет, что `list_parts` возвращает габариты и объём в единицах сессии (`mm`), корректно пересчитывает `bounding_box`, `bounding_box_dimensions` и `volume` при `{"length_unit": "m"}` без изменения `state.length_unit`, сортирует детали по `part_id` и отклоняет невалидный `{"length_unit": "cm"}` с кодом `INVALID_ARGUMENT` (`param: "length_unit"`).
2. **`crates/mcp-cad-server/src/tools/metrics.rs`**:
   - `test_get_scene_summary_unit_conversion_and_joints` — расширен проверкой зубчатой передачи `CadJointType::Gear` (`j_gear_0`): проверяет наличие `"axis"`, `"axis_a"`, `"axis_b"` и `"ratio"` в JSON-объекте шарнира и наличие `axis_a=[...]` / `axis_b=[...]` в `summary_text`.
3. **`crates/mcp-cad-server/tests/mcp_contract_tests.rs`**:
   - `test_mcp_p2_t29_list_parts_length_unit_override_and_scene_summary_gear_axes` — сквозной JSON-RPC тест через `McpServer::handle_request`: проверяет `tools/list` (`list_parts.inputSchema.properties.length_unit`), вызов `list_parts` с `length_unit: "mm"` и `"m"`, ошибку на `"inch"` и вывод `axis`/`axis_a`/`axis_b` для `gear` в `get_scene_summary`.
4. **`src/cad/cad_engine.test.ts`**:
   - `verifies P2-T29: list_parts respects length_unit override and rejects invalid length_unit` — проверяет фронтенд-диспетчер `McpDispatcher`.

Все тесты (`cargo test`, `cargo clippy -- -D warnings`, `npx tsx --test src/cad/cad_engine.test.ts`, `npm run lint`, `npm run build`) проходят без ошибок.
