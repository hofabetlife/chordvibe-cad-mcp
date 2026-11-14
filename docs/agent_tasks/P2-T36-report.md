# Отчёт о выполнении задачи P2-T36

## Задача
**P2-T36 [S]: Масштабирование `logical_plan.dag`, `physical_plan.dag` и `candidate_plans[i].dag` через `scale_feature_op_from_meters` в `optimize_feature_tree`, вывод `part_id`/`deleted_part_id` в `delete_part` (`tools/cad.rs`) и точность `f64` в `PhysicsWorld::gravity()` (`simulator.rs:181`)**

---

## 1. Обнаруженные дефекты и причины

1. **Отсутствие масштабирования узлов `dag` внутри планов `optimize_feature_tree` (`crates/mcp-cad-server/src/tools/cad.rs:3501-3535` и `src/cad/mcp_dispatcher.ts:1861-1923`)**:
   - Инструмент `optimize_feature_tree` сериализовал `CadOptimizationReport` напрямую через `serde_json::to_value(&report)`, масштабируя только `volume` и `equivalence_verification.{volume_before, volume_after}`.
   - Массивы `logical_plan.dag`, `physical_plan.dag` и `candidate_plans[i].dag` возвращали операции `FeatureOp` (`AddCylinder`, `CutHole`, `AddRevolve`, `AddPolygonPrism` и т.д.) во внутренних единицах ядра (метрах СИ `m`), игнорируя активную или явно запрошенную единицу измерения `length_unit` (`"mm"` по умолчанию или `"m"`), в отличие от `get_feature_tree`.

2. **Отсутствие `part_id` и `deleted_part_id` в ответе `delete_part` (`crates/mcp-cad-server/src/tools/cad.rs:415-417` и `src/cad/mcp_dispatcher.ts:545`)**:
   - В Rust-сервере `delete_part` возвращал только `{"status": "deleted"}` без указания удалённого идентификатора детали, а в TypeScript-диспетчере отсутствовало поле `"part_id"`.

3. **Потеря точности `f32 -> f64` в `PhysicsWorld::gravity()` (`crates/mcp-cad-physics/src/simulator.rs:181-187`)**:
   - В `PhysicsWorld` вектор гравитации хранился только как `Vector<Real>` (`f32`), поэтому `gravity(&self) -> [f64; 3]` кастовал `-9.81_f32 as f64`, возвращая `-9.8100004196167` вместо `-9.81`. При переводе в `mm/s²` в `get_simulation_metrics_json` (`raw_gravity[1] * 1000.0`) это давало `-9810.0004196167` вместо точного `-9810.0`.

---

## 2. Внесённые изменения

1. **`crates/mcp-cad-physics/src/simulator.rs`**:
   - В структуру `PhysicsWorld` добавлено поле `gravity_f64: [f64; 3]`, сохраняющее точное значение `f64`, переданное в `PhysicsWorld::with_gravity` и `PhysicsWorld::set_gravity` (и копируемое в `Clone::clone`).
   - Метод `PhysicsWorld::gravity(&self) -> [f64; 3]` теперь возвращает `self.gravity_f64` без погрешности `f32`-округления.

2. **`crates/mcp-cad-server/src/tools/cad.rs`**:
   - В `delete_part` добавлены поля `"part_id": part_id` и `"deleted_part_id": part_id`.
   - В `optimize_feature_tree` перед сериализацией `report` все узлы в `report.logical_plan.dag`, `report.physical_plan.dag` и `report.candidate_plans[i].dag` масштабируются из метров СИ в целевой `length_unit` через `scale_feature_op_from_meters(&node.op, k_from_m)` (при этом внутреннее дерево `part.dag` в состоянии сцены остаётся в метрах СИ).

3. **`src/cad/mcp_dispatcher.ts`**:
   - В `delete_part` возвращаются оба поля `part_id` и `deleted_part_id`.
   - В `optimize_feature_tree` добавлено масштабирование `logical_plan.dag`, `physical_plan.dag` и `candidate_plans[i].dag` в соответствии с `length_unit`.

4. **Документация (`docs/MCP_API_REFERENCE.md`, `PROJECT_CONTEXT.md`, `TASKS.md`)**:
   - Обновлены описание и пример ответа `delete_part`, счётчики тестов и реестр выполненных задач.

---

## 3. Верификация и тесты

- Добавлен unit-тест `tools::cad::tests::test_p2_t36_optimize_feature_tree_dag_scaling_and_delete_part_id` в `crates/mcp-cad-server/src/tools/cad.rs`.
- Добавлен unit-тест `test_p2_t36_gravity_f64_exact_precision_and_clone` в `crates/mcp-cad-physics/tests/physics_tests.rs`.
- Добавлен интеграционный контрактный тест `test_mcp_p2_t36_optimize_feature_tree_dag_scaling_delete_part_id_and_gravity_f64` в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`.
- Добавлен unit-тест `verifies P2-T36: optimize_feature_tree scales logical_plan, physical_plan, and candidate_plans DAGs and delete_part returns part_id and deleted_part_id` в `src/cad/cad_engine.test.ts`.
- Полный прогон:
  - `cargo test --workspace --no-fail-fast`: **235 тестов, 235 проходят (100% PASS)**.
  - `cargo clippy --workspace --all-targets -- -D warnings`: **0 ошибок, 0 предупреждений**.
  - `npx tsx --test src/cad/cad_engine.test.ts`: **38 тестов, 38 проходят (100% PASS)**.

---

## 4. Новые выявленные проблемы (занесены в `TASKS.md`)

- **P2-T40 [S]**: В `crates/mcp-cad-server/src/tools/requirements.rs` (`evaluate_single_requirement`, строки 409, 502, 588, 672) при проверке требования с `scope: "scene"` (`spec.part_id.is_none()`), когда в сцене ровно 1 деталь (`state.parts.len() == 1`), функция `resolve_target_part(None, state)` возвращает эту единственную деталь, а ветки агрегации `MassMax`/`MeshValid` срабатывают только при `state.parts.len() > 1`. В результате требования уровня сцены (`bbox_max`, `bbox_min`, `mass_max`, `mesh_valid`) при одной детали в сцене возвращают `"part_id": "<single_part_id>"` и `"scope": "<single_part_id>"` вместо `"part_id": null, "scope": "scene"`, а инструмент `delete_part` (`crates/mcp-cad-server/src/tools/cad.rs:415`) не вызывает `attach_auto_check_if_enabled(state, None, &mut resp)` при активном `scene_auto_check`.
