# Отчёт о выполнении задачи P2-T35

**Задача:** Вывод `part_id` и геометрических метаданных (`depth` в `cut_hole`, `feature_id` в `modify_feature`) в ответах `add_feature_primitive`, `cut_hole`, `modify_feature` и `cut_helical_groove` (`tools/cad.rs`)  
**Статус:** ✅ DONE  
**Дата:** 2026-04-20

---

## 1. Контекст и проблема

В `crates/mcp-cad-server/src/tools/cad.rs` ответы инструментов модификации дерева фич (`add_feature_primitive`, `cut_hole`, `modify_feature`, `cut_helical_groove`) имели следующие пробелы в агентском контракте:
1. Ни один из четырёх инструментов не возвращал идентификатор модифицированной детали `part_id` в JSON-ответе, что затрудняло трассировку ответов при пакетных (batch) вызовах по нескольким деталям.
2. `cut_hole` не возвращал вычисленную глубину отверстия `depth` (особенно критично при `through: true`, когда глубина автоматически вычисляется по габаритам детали вдоль оси сверления) и флаг `through`.
3. `modify_feature` возвращал только `status`, `revision`, `updated_bounds` и `dimensions`, но не возвращал `part_id`, `feature_id`, а также унифицированные поля `bounding_box`, `volume` и `volume_m3`.
4. `add_feature_primitive`, `cut_hole` и `cut_helical_groove` не возвращали актуальные геометрические метаданные детали (`bounding_box`, `volume`, `volume_m3`) после применения операции.
5. В TypeScript-диспетчере (`src/cad/mcp_dispatcher.ts`) ответы `add_feature_primitive`, `cut_hole`, `cut_helical_groove` и `modify_feature` не включали `feature_id`, `revision` и `depth`/`through` (для `cut_hole`).

---

## 2. Выполненные изменения

1. **Rust MCP-сервер (`crates/mcp-cad-server/src/tools/cad.rs`)**:
   - `add_feature_primitive`: в JSON-ответ добавлены `"part_id"`, `"primitive_type"`, `"bounding_box"` (`min`, `max`, `dimensions` в выбранных единицах `length_unit`), `"volume"` и `"volume_m3"`.
   - `cut_hole`: в JSON-ответ добавлены `"part_id"`, `"depth"` (фактическая глубина отверстия в `length_unit`, включая автоматически вычисленную при `through: true`), `"depth_m"` (в метрах СИ), `"through"`, `"bounding_box"`, `"volume"` и `"volume_m3"`.
   - `modify_feature`: в JSON-ответ добавлены `"part_id"`, `"feature_id"`, `"bounding_box"`, `"volume"` и `"volume_m3"` при сохранении обратной совместимости с `"updated_bounds"` и `"dimensions"`.
   - `cut_helical_groove`: в JSON-ответ добавлены `"part_id"`, `"bounding_box"`, `"volume"` и `"volume_m3"`.
2. **TypeScript-диспетчер (`src/cad/mcp_dispatcher.ts`)**:
   - Синхронизированы ответы `add_feature_primitive`, `cut_hole`, `cut_helical_groove` и `modify_feature`: добавлены `"feature_id"`, `"revision"`, а для `cut_hole` — `"depth"` и `"through"`.
3. **Документация (`docs/MCP_API_REFERENCE.md`)**:
   - Обновлены примеры JSON-ответов для `add_feature_primitive`, `cut_hole`, `modify_feature` и `cut_helical_groove`.

---

## 3. Верификация и тесты

- Добавлен модульный тест `test_p2_t35_feature_mutation_tools_return_part_id_and_geometric_metadata` в `crates/mcp-cad-server/src/tools/cad.rs`.
- Добавлен интеграционный JSON-RPC контрактный тест `test_mcp_p2_t35_feature_mutation_tools_part_id_depth_and_feature_id` в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`.
- Добавлен тест `verifies P2-T35: add_feature_primitive, cut_hole, modify_feature, and cut_helical_groove return part_id, feature_id, revision, and cut_hole depth` в `src/cad/cad_engine.test.ts`.
- Все тесты (`cargo test`, `npx tsx --test src/cad/cad_engine.test.ts`, `cargo clippy --workspace --all-targets -- -D warnings`) проходят без ошибок и предупреждений.

---

## 4. Обнаруженные новые проблемы

В ходе аудита `crates/mcp-cad-server/src/tools/cad.rs` обнаружена проблема, зарегистрированная в `TASKS.md` как **P2-T36**:
- В `optimize_feature_tree` (`crates/mcp-cad-server/src/tools/cad.rs`) узлы DAG внутри `logical_plan.dag`, `physical_plan.dag` и `candidate_plans[i].dag` сериализуются напрямую из `CadOptimizationReport` с параметрами `FeatureNode.op` в внутренних единицах ядра (метрах СИ `m`) вместо масштабирования через `scale_feature_op_from_meters(&node.op, k_from_m)` в запрошенную единицу `length_unit` (`"mm"` или `"m"`), в отличие от `get_feature_tree`. Кроме того, `delete_part` в `tools/cad.rs` возвращает только `{"status": "deleted"}` без `part_id` / `deleted_part_id`.
