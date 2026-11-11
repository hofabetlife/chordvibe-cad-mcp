# Отчёт о выполнении задачи P2-T27

**Задача:** `P2-T27 [S]` — Учёт `resolve_length_unit(args)` и `fea_revision`/`stale` в `get_simulation_metrics_json` (`metrics.rs:13-65`) и предотвращение инкремента `part.revision` при `applied_rules.is_empty()` в `optimize_feature_tree` (`part.rs:115`, `tools/cad.rs:3239`)  
**Дата:** 09.11.2026  
**Статус:** ✅ DONE

---

## 1. Описание проблемы

В ходе выполнения **P2-T24** ([`docs/agent_tasks/P2-T24-report.md`](P2-T24-report.md)) были выявлены две проблемы контракта инструментов `get_simulation_metrics_json` и `optimize_feature_tree`:

1. **Игнорирование `resolve_length_unit(args)` и отсутствие метаданных актуальности FEA (`fea_revision`, `part_revision`, `stale`, `status`) в `get_simulation_metrics_json` (`crates/mcp-cad-server/src/tools/metrics.rs:13-65`):**
   - `get_simulation_metrics_json` использовал `state.length_unit` напрямую вместо `state.resolve_length_unit(args)?` и не объявлял опциональный параметр `length_unit` в `input_schema` (`tools/mod.rs`), из-за чего передача `{"length_unit": "m"}` игнорировалась, а передача невалидной единицы не возвращала структурированную ошибку `INVALID_UNITS`.
   - Элементы массива `current_metrics.fea_reports` не содержали полей `safety_factor` (алиас к `factor_of_safety`), `fea_revision`, `part_revision`, `stale`, `is_stale` и `status` (`"FRESH"` / `"STALE"`), в отличие от `get_scene_summary`, поэтому ИИ-агент при вызове `get_simulation_metrics_json` не мог определить, устарел ли расчёт прочности после модификации детали.
   - Порядок элементов в `parts` и `fea_reports` зависел от итерации по `HashMap`, что приводило к недетерминированному порядку при нескольких деталях.

2. **Ложный инкремент `part.revision` при холостом запуске `optimize_feature_tree(apply: true)` (`applied_rules.is_empty()`) и отсутствие `resolve_length_unit(args)` / `revision` в ответе (`crates/mcp-cad-core/src/part.rs:115-131`, `crates/mcp-cad-server/src/tools/cad.rs:3239`):**
   - В `Part::optimize_dag(apply: true)` безусловно перезаписывался `self.dag = optimized_tree` и вызывался `self.rebuild()`, который инкрементировал `self.revision` даже тогда, когда `report.applied_rules.is_empty()` и дерево `optimized_tree` совпадало с исходным `self.dag`. В результате вызов `optimize_feature_tree(apply: true)` на уже оптимальной детали ложно переводил существующие результаты `run_fea_analysis` в статус `STALE` в `check_requirements` (`min_safety_factor`), `get_scene_summary` и `get_simulation_metrics_json`.
   - В `optimize_feature_tree` (`tools/cad.rs`) использовался `state.length_unit` вместо `state.resolve_length_unit(args)?`, а в JSON-ответе отсутствовали поля `revision`, `volume`, `volume_m3` и `equivalence_verification.volume_before` / `volume_after` (в активных единицах `length_unit^3`).

---

## 2. Выполненные изменения

1. **Предотвращение холостого инкремента `part.revision` в `Part::optimize_dag` (`crates/mcp-cad-core/src/part.rs`):**
   - В `Part::optimize_dag(&mut self, apply: bool)` при `apply == true` замена `self.dag = optimized_tree` и пересчёт `self.rebuild()` (с транзакционным откатом при ошибке) теперь выполняются только если `!report.applied_rules.is_empty() || self.dag != optimized_tree`.
   - Если ни одно правило оптимизации не сработало (`report.applied_rules.is_empty()`) и дерево не изменилось, `self.revision` остаётся неизменным, сохраняя актуальность (`FRESH`) кэшированных результатов FEA.

2. **Поддержка `resolve_length_unit(args)`, `revision` и объёмов в `optimize_feature_tree` (`crates/mcp-cad-server/src/tools/cad.rs`, `tools/mod.rs`, `src/cad/mcp_dispatcher.ts`):**
   - В `optimize_feature_tree` разрешение единицы длины `let unit = state.resolve_length_unit(args)?;` выполняется до мутации `state.parts`.
   - Обновление физического коллайдера (`update_part_collider` / `register_part`) выполняется только при реальном применении правил (`apply && !report.applied_rules.is_empty()`).
   - В JSON-ответ `optimize_feature_tree` добавлены поля `"revision"`, `"volume"` (в `length_unit^3`), `"volume_m3"`, а в объект `equivalence_verification` добавлены `"volume_before"` и `"volume_after"` в `length_unit^3` наряду с `"volume_before_cm3"` и `"volume_after_cm3"`.
   - В `tools/mod.rs` в `input_schema` инструмента №28 `optimize_feature_tree` добавлен опциональный параметр `"length_unit"`.

3. **Поддержка `resolve_length_unit(args)` и метаданных актуальности FEA в `get_simulation_metrics_json` (`crates/mcp-cad-server/src/tools/metrics.rs`, `tools/mod.rs`, `src/cad/mcp_dispatcher.ts`):**
   - В `get_simulation_metrics_json` подключён вызов `let unit = state.resolve_length_unit(args)?;`, а в `tools/mod.rs` в `input_schema` инструмента №17 добавлен опциональный параметр `"length_unit"`.
   - Списки `parts` и `fea_reports` отсортированы детерминированно по `part_id`.
   - В каждый элемент `current_metrics.fea_reports` добавлены поля:
     - `"safety_factor"` (алиас к `"factor_of_safety"`),
     - `"fea_revision"` (`u64`),
     - `"part_revision"` (`Option<u64>`),
     - `"stale"` и `"is_stale"` (`bool`, `part_revision != Some(fea_rev)`),
     - `"status"` (`"STALE"` или `"FRESH"`).

4. **Документация (`docs/MCP_API_REFERENCE.md`):**
   - Обновлены разделы `20. get_simulation_metrics_json` и `28. optimize_feature_tree` с описанием параметра `length_unit` и новых полей ответа.

---

## 3. Верификация и тесты

Добавлены и выполнены следующие тесты:
- `crates/mcp-cad-core/src/part.rs`:
  - `test_p2_t27_optimize_dag_only_bumps_revision_when_rules_applied` — проверяет, что `optimize_dag(true)` увеличивает `revision` при срабатывании `R1_TRANSFORM_FUSION`, но не увеличивает `revision` при повторном вызове на уже оптимизированном дереве (`applied_rules.is_empty()`).
- `crates/mcp-cad-server/src/tools/metrics.rs`:
  - `test_p2_t27_get_simulation_metrics_json_length_unit_and_fea_staleness` — проверяет переопределение `length_unit: "m"`, отклонение невалидного `length_unit: "cm"` (`INVALID_UNITS`) и переход `fea_reports[0]` из `stale: false, status: "FRESH"` в `stale: true, status: "STALE"` после модификации детали.
- `crates/mcp-cad-server/src/tools/cad.rs`:
  - `test_p2_t27_optimize_feature_tree_revision_and_length_unit_override` — проверяет сохранение `revision` и статуса `FRESH` у FEA при холостом `optimize_feature_tree(apply: true)` и инкремент `revision` (с переходом FEA в `STALE`) при реальном применении `R1_TRANSFORM_FUSION`, а также пересчёт объёмов при `length_unit: "m"`.
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs`:
  - `test_mcp_p2_t27_simulation_metrics_fea_staleness_and_optimize_feature_tree_revision` — сквозной JSON-RPC тест `create_part` → `add_feature_primitive` → `run_fea_analysis` → `optimize_feature_tree(apply: true)` (без правил, `revision == 2`, FEA `FRESH` в `get_simulation_metrics_json` и `check_requirements`) → `transform_part` (`revision == 3`, FEA `STALE`) → `optimize_feature_tree(apply: true, length_unit: "m")` (`R1_TRANSFORM_FUSION`, `revision == 4`).
- `src/cad/cad_engine.test.ts`:
  - `verifies P2-T27: get_simulation_metrics_json length_unit & fea staleness, and optimize_feature_tree revision when applied_rules is empty`.

Результаты запуска:
- `npx tsx --test src/cad/cad_engine.test.ts`: **31 passed, 0 failed**.
- `cargo test --workspace`: все unit- и контрактные тесты проходят (**100% PASS**).
- `cargo clippy --workspace --all-targets -- -D warnings`: **0 предупреждений, 0 ошибок**.

---

## 4. Новые выявленные замечания (заведены в `TASKS.md`)

- **`P2-T30 [S]`**:
  1. В `import_stl` (`crates/mcp-cad-server/src/tools/cad.rs:2002-2071`) не поддерживаются алиас `"file_path"` (в отличие от `export_stl` и TS `mcp_dispatcher.ts`), опциональные параметры `"part_id"` и `"is_fixed"`, деталь не регистрируется в `state.physics_world`, а в JSON-ответе отсутствуют `"revision"`, `"volume"`, `"volume_m3"`, `"mass_kg"` и `"bounding_box.dimensions"`.
  2. В `attach_auto_check_if_enabled` (`crates/mcp-cad-server/src/tools/requirements.rs:1102-1135`) вызов `evaluate_requirements_report(state, filter, state.length_unit)` использует `state.length_unit` вместо единицы длины конкретного вызова (`resp["length_unit"]`), из-за чего при передаче переопределения `length_unit` в мутирующий инструмент встроенный блок `requirements_status` форматируется в единицах сессии, а не вызова.
