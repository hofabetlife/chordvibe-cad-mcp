# Отчет о выполнении задачи P2-T32

## 1. Название и контекст задачи
**Задача:** `P2-T32 [S]`: Схема `length_unit` и метаданные `part_id`/`revision` в `get_feature_tree` и `validate_mesh` (`tools/mod.rs`, `tools/cad.rs`) и масштабирование `gravity` / вывод `joints_count` в `get_simulation_metrics_json` (`metrics.rs:62-66`).  
**Компоненты:** `crates/mcp-cad-server/src/tools/{mod.rs, cad.rs, metrics.rs}`, `src/cad/mcp_dispatcher.ts`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `src/cad/cad_engine.test.ts`.  
**Дата:** 10.11.2026.  
**Статус:** ✅ Выполнено.

---

## 2. Диагностика и выявленные проблемы

В ходе аудита контрактов MCP-инструментов были выявлены следующие расхождения:

1. **Неполная JSON-схема инструментов №7 `get_feature_tree` и №27 `validate_mesh` (`tools/mod.rs`):**
   - В `crates/mcp-cad-server/src/tools/mod.rs` в `input_schema.properties` для обоих инструментов отсутствовало опциональное свойство `"length_unit": {"type": "string", "enum": ["mm", "m"]}`, хотя их обработчики в `crates/mcp-cad-server/src/tools/cad.rs` (`get_feature_tree` и `validate_mesh_tool`) вызывали `state.resolve_length_unit(args)?`. Клиенты MCP, выполняющие клиентскую валидацию JSON-схем, не могли передавать `length_unit` без предупреждений или отказов валидации.

2. **Отсутствие базовых метаданных в ответах `get_feature_tree` и `validate_mesh`:**
   - В JSON-ответе `get_feature_tree` возвращались только `dag`, `length_unit` и `units`, но отсутствовали идентификатор детали `part_id`, монотонный счётчик ревизий `revision` и количество операций `features_count`.
   - В JSON-ответе `validate_mesh_tool` возвращались топологические и геометрические метрики, но отсутствовали `part_id` и `revision`, что требовало от агента дополнительных запросов для сопоставления результатов валидации с состоянием детали.

3. **Немасштабируемый вектор гравитации и отсутствие `joints_count` в `get_simulation_metrics_json` (`metrics.rs:62-66`):**
   - В `get_simulation_metrics_json` поле `current_metrics.physics.gravity` возвращало фиксированный вектор СИ `[0.0, -9.81, 0.0]` независимо от активной единицы длины (например, в сессии `"mm"` ускорение свободного падения должно быть выражено в `мм/с²` как `[0.0, -9810.0, 0.0]`).
   - Отсутствовало дублирующее эталонное поле в СИ `gravity_m_s2`, задокументированное в `docs/MCP_API_REFERENCE.md:1212`.
   - Отсутствовало поле `joints_count` как на верхнем уровне `current_metrics`, так и внутри блока `physics`.

---

## 3. Выполненные изменения

### 3.1. Обновление схем в `crates/mcp-cad-server/src/tools/mod.rs`
- В `get_feature_tree` (инструмент №7) и `validate_mesh` (инструмент №27) добавлено опциональное свойство `length_unit`:
  ```json
  "length_unit": {
      "type": "string",
      "enum": ["mm", "m"],
      "description": "Optional per-call length unit override ('mm' or 'm'; defaults to session length_unit)"
  }
  ```

### 3.2. Добавление метаданных в `crates/mcp-cad-server/src/tools/cad.rs`
- В `get_feature_tree` добавлены поля `part_id`, `revision` и `features_count`:
  ```rust
  Ok(json!({
      "part_id": part.id,
      "revision": part.revision,
      "features_count": scaled_nodes.len(),
      "dag": scaled_nodes,
      "length_unit": unit.as_str(),
      "units": unit.units_metadata(),
  }))
  ```
- В `validate_mesh_tool` в результирующий JSON-объект добавлены `part_id` и `revision`:
  ```rust
  if let Some(obj) = val.as_object_mut() {
      obj.insert("part_id".to_string(), json!(part.id));
      obj.insert("revision".to_string(), json!(part.revision));
      ...
  }
  ```

### 3.3. Масштабирование гравитации и `joints_count` в `crates/mcp-cad-server/src/tools/metrics.rs`
- В `get_simulation_metrics_json` реализовано:
  - Вычисление `scaled_gravity = [gx * k_from_m, gy * k_from_m, gz * k_from_m]`, где `k_from_m = 1000.0` для `"mm"` (`-9810.0 мм/с²`) и `1.0` для `"m"` (`-9.81 м/с²`).
  - Сохранение `gravity_m_s2` с неизменными значениями в СИ `[gx, gy, gz]`.
  - Добавление `joints_count = state.physics_world.joints.len()` на верхнем уровне `current_metrics` и в блоке `physics`.

### 3.4. Синхронизация TypeScript диспетчера (`src/cad/mcp_dispatcher.ts`)
- В диспетчере `get_feature_tree` добавлена валидация `params.length_unit` и возвращаются `part_id`, `revision`, `features_count`, `length_unit` и `units`.
- В диспетчере `validate_mesh` добавлена валидация `params.length_unit`, масштабирование `volume` и `surface_area`, и возврат `part_id`, `revision`, `volume_m3`, `surface_area_m2`, `length_unit` и `units`.
- В диспетчере `get_simulation_metrics_json` в `response.current_metrics` добавлен блок `physics` со `scaled_gravity`, `gravity_m_s2` и `joints_count`.

---

## 4. Верификация и тесты

1. **Rust unit-тесты (`crates/mcp-cad-server/src/tools/cad.rs`):**
   - Добавлен тест `test_p2_t32_get_feature_tree_and_validate_mesh_metadata_and_length_unit`:
     - Проверка `part_id`, `revision`, `features_count`, `dag`, `length_unit` для `get_feature_tree`.
     - Проверка переопределения `length_unit: "m"`.
     - Проверка `part_id`, `revision`, `volume`, `volume_m3`, `status: "PASS"` для `validate_mesh_tool`.
     - Проверка ошибки при некорректном `length_unit`.

2. **Rust unit-тесты (`crates/mcp-cad-server/src/tools/metrics.rs`):**
   - Добавлен тест `test_p2_t32_get_simulation_metrics_json_gravity_scaling_and_joints_count`:
     - Проверка `gravity: [0.0, -9810.0, 0.0]` при `"mm"` и `[0.0, -9.81, 0.0]` при `"m"`.
     - Проверка `gravity_m_s2: [0.0, -9.81, 0.0]` в обоих случаях.
     - Проверка изменения `joints_count` с 0 на 1 при вызове `create_joint`.

3. **Rust контрактные тесты (`crates/mcp-cad-server/tests/mcp_contract_tests.rs`):**
   - Добавлен тест `test_mcp_p2_t32_feature_tree_validate_mesh_and_metrics_gravity_scaling`:
     - Проверка наличия `length_unit` в схемах `tools/list` для `get_feature_tree` и `validate_mesh`.
     - Проверка метаданных ответов через RPC-вызовы.
     - Проверка масштабирования `physics.gravity` и значений `gravity_m_s2`.

4. **TypeScript тесты (`src/cad/cad_engine.test.ts`):**
   - Добавлен тест `verifies P2-T32: get_feature_tree & validate_mesh metadata and get_simulation_metrics_json gravity scaling and joints_count`.
   - Прогон через `npx tsx --test src/cad/cad_engine.test.ts`: **34 теста, 34 проходят, 0 падают (100% PASS)**.

---

## 5. Заключение
Задача **P2-T32** успешно реализована и полностью протестирована как в нативной подсистеме Rust, так и во фронтенд-диспетчере TypeScript. Все схемы MCP и контракты ответов приведены в строгое соответствие со спецификацией.
