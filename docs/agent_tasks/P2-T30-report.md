# Отчёт о выполнении задачи P2-T30

**Задача:** Поддержка `file_path`/`part_id`/`is_fixed`, регистрация в `physics_world` и метаданные в `import_stl` (`tools/cad.rs:2002-2071`) и учёт `resp["length_unit"]` в `attach_auto_check_if_enabled` (`requirements.rs:1102`)  
**Фаза:** P2 (Agent Contract & Ergonomics)  
**Статус:** ✅ Выполнено  
**Дата:** 09.11.2026  

---

## 1. Диагностика проблемы

При выполнении задачи **P2-T27** ([`docs/agent_tasks/P2-T27-report.md`](P2-T27-report.md)) и анализе работы `import_stl` и слоя автопроверки требований были выявлены расхождения в контрактах:

1. **Неполная поддержка параметров и отсутствие регистрации в `physics_world` в `import_stl` (`crates/mcp-cad-server/src/tools/cad.rs:2002-2071` и `tools/mod.rs`)**:
   - `import_stl` принимал только аргумент `"filepath"` и не поддерживал популярный алиас `"file_path"` (в отличие от `export_stl` и TypeScript-клиента `src/cad/mcp_dispatcher.ts`).
   - `import_stl` всегда генерировал случайный UUID `format!("part-{}", Uuid::new_v4().simple())`, не позволяя агенту передать детерминированный `part_id` (или `id`), не проверял зарезервированные статические якоря `"ground"` и `"world"` и не проверял дубликаты `part_id` с возвратом структурированной ошибки `PART_ALREADY_EXISTS`.
   - `import_stl` не принимал параметр `is_fixed` (для фиксации детали в симуляции физики) и вообще не регистрировал импортированную деталь в `state.physics_world`. Из-за этого деталь отсутствовала в физическом мире (`state.physics_world.has_part(&part_id) == false`), что ломало последующее создание суставов `create_joint` и физическую симуляцию `run_physics_simulation`.
   - В JSON-ответе `import_stl` отсутствовали поля `"revision"`, `"volume"`, `"volume_m3"`, `"mass_kg"` и габаритные размеры `"bounding_box.dimensions"`.

2. **Игнорирование переопределения `length_unit` конкретного вызова в `attach_auto_check_if_enabled` (`crates/mcp-cad-server/src/tools/requirements.rs:1102-1135`)**:
   - Функция автоматической проверки требований `attach_auto_check_if_enabled`, встраивающая блок `requirements_status` и `failed_requirements` в ответы всех мутирующих CAD-инструментов, безусловно передавала `state.length_unit` в `evaluate_requirements_report(state, filter, state.length_unit)`.
   - Если сессия находилась в миллиметрах (`"mm"`), но мутирующий инструмент был вызван с локальным переопределением единицы `{"length_unit": "m"}` (или наоборот), измеренные значения `measured` и запасы `margin` внутри встроенного блока `requirements_status` форматировались в единицах сессии, а не в единице ответа инструмента `resp["length_unit"]`.

---

## 2. Выполненные изменения

### 2.1. Доработка `import_stl` (`crates/mcp-cad-server/src/tools/{cad.rs, mod.rs}`, `src/cad/mcp_dispatcher.ts`)
- **Поддержка алиаса `file_path`**: чтение аргумента пути к файлу выполнено через `args.get("filepath").or_else(|| args.get("file_path"))`.
- **Поддержка `part_id` и валидация**:
  - Чтение `part_id` из `args.get("part_id").or_else(|| args.get("id"))`. Если не передан — генерация `part-<UUID>`.
  - Регистронезависимая проверка зарезервированных идентификаторов `"ground"` и `"world"` с возвратом ошибки `INVALID_ARGUMENT` (`param: "part_id"`).
  - Проверка уникальности `state.parts.contains_key(&part_id)` с возвратом ошибки `Part with id '{part_id}' already exists`, мапирующейся в структурированную ошибку `PART_ALREADY_EXISTS` (`param: "part_id"`).
- **Поддержка `is_fixed`**: чтение `let is_fixed = args.get("is_fixed").and_then(|v| v.as_bool()).unwrap_or(false);` и присвоение `part.is_fixed = is_fixed`.
- **Регистрация в `state.physics_world`**: при `!part.mesh.vertices.is_empty()` вызывается `state.physics_world.add_part(&part, !part.is_fixed)`, создающий соответствующий коллайдер (`TriMesh` для `is_fixed: true` или `ConvexHull` для `is_fixed: false`) и регистрирующий твердое тело в симуляторе.
- **Метаданные в JSON-ответе**:
  - Добавлены `"is_fixed": is_fixed`, `"revision": part.revision`, `"bounding_box.dimensions": scale_vec3(bbox.dimensions(), k_from_m)`, `"volume": volume_m3 * k_from_m^3`, `"volume_m3": volume_m3`, `"mass_kg": mass_kg`, `"mass": mass_kg`.
- **Схема и документация инструмента в `tools/mod.rs`**:
  - В `ToolDefinition` инструмента `import_stl` добавлены свойства `"file_path"`, `"part_id"`, `"is_fixed"` и `"length_unit"`, обновлены описание, пример вызова и список частых ошибок (`PART_ALREADY_EXISTS`).
- **Синхронизация с TypeScript (`src/cad/mcp_dispatcher.ts`)**:
  - В `case 'import_stl'` добавлена поддержка `params.filepath`, опциональные `name`/`material`, `isFixed: Boolean(params.is_fixed)` и возврат `is_fixed` в ответе.

### 2.2. Учёт `resp["length_unit"]` в `attach_auto_check_if_enabled` (`crates/mcp-cad-server/src/tools/requirements.rs`)
- В `attach_auto_check_if_enabled`:
  ```rust
  let active_unit = resp
      .get("length_unit")
      .and_then(|v| v.as_str())
      .and_then(|s| LengthUnit::parse(s).ok())
      .unwrap_or(state.length_unit);

  let report = evaluate_requirements_report(state, filter, active_unit);
  ```
  Теперь при локальном переопределении единицы измерения в мутирующем вызове блок `requirements_status` согласуется с `resp["length_unit"]`.

### 2.3. Структурированные ошибки (`crates/mcp-cad-server/src/error.rs`)
- В `ServerError::to_error_body` для сообщения об уже существующей детали добавлена привязка `param: "part_id"`, если сообщение содержит `"Part with id"`, гарантируя точную локализацию параметра ошибки.

---

## 3. Новые проблемы, выявленные в ходе аудита (заведены в `TASKS.md`)

При тестировании сквозных сценариев с импортированными деталями выявлена следующая проблема:
- **`P2-T33 [S]`**: Поддержка `transform_part` для импортированных STL-деталей при пустом DAG (`part.dag.nodes.is_empty()`) и предотвращение ошибки `Cannot transform empty solid` (`crates/mcp-cad-server/src/tools/cad.rs:3130`, `crates/mcp-cad-core/src/dag.rs:1016`).

---

## 4. Тестирование и верификация

1. **Unit-тесты в `crates/mcp-cad-server/src/tools/cad.rs`**:
   - `test_p2_t30_import_stl_parameters_physics_registration_and_metadata`:
     - Проверяет импорт через алиас `file_path` с `part_id: "imported_fixed_box"` и `is_fixed: true`.
     - Проверяет наличие полей `revision`, `triangles_count`, `vertices_count`, `bounding_box.dimensions`, `volume`, `volume_m3`, `mass_kg`.
     - Проверяет регистрацию в `state.physics_world`: `has_part("imported_fixed_box") == true`, `is_part_dynamic == false`.
     - Проверяет импорт с `is_fixed: false` и регистрацию динамического тела (`is_part_dynamic == true`).
     - Проверяет отказ на дубликат `part_id` с кодом `PART_ALREADY_EXISTS` (`param: "part_id"`).
     - Проверяет отказ на зарезервированный идентификатор `"Ground"` с кодом `INVALID_ARGUMENT` (`param: "part_id"`).
2. **Unit-тесты в `crates/mcp-cad-server/src/tools/requirements.rs`**:
   - `test_p2_t30_attach_auto_check_respects_response_length_unit_override`:
     - Проверяет, что при сессионной единице `length_unit = "mm"` передача ответа с `length_unit = "m"` форматирует `measured` и `margin` в метрах.
3. **Контрактные тесты в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`**:
   - `test_mcp_p2_t30_import_stl_contract_and_auto_check_length_unit`:
     - Сквозной JSON-RPC тест через `rpc_call`: экспорт детали, импорт через `import_stl` с `file_path`, `part_id`, `is_fixed: true`, проверка метаданных в ответе, проверка auto-check при мутации с переопределением `length_unit = "m"`, проверка ошибок `PART_ALREADY_EXISTS` и `INVALID_ARGUMENT`.
4. **TypeScript тесты в `src/cad/cad_engine.test.ts`**:
   - `verifies P2-T30: import_stl supports file_path alias, part_id, and is_fixed in dispatcher`:
     - Проверяет импорт в `executeMcpToolOnEngineState` с `file_path`/`filepath`, `part_id` и `is_fixed`.

Все 33 теста в TypeScript (`cad_engine.test.ts`) и 43 теста в `mcp-cad-server --lib` проходят на 100% (PASS). Сборка `npm run build` и `cargo check --workspace --all-targets` проходят без единой ошибки и предупреждения.
