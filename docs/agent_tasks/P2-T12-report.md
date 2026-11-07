# Отчёт по задаче P2-T12

**Задача**: P2-T12 — Откат `physics_world` в `batch_cad_operations`, сохранение `!part.is_fixed` при CSG/transform и удаление мёртвого `tools/export.rs`  
**Ветка**: `agent/P2-T12`  
**Дата**: 2026-04-20  
**Статус**: ✅ DONE

---

## 1) Что сделано (файлы, функции)

1. **`crates/mcp-cad-physics/src/simulator.rs`**:
   - Реализован типаж `Clone` для `PhysicsWorld`: клонируются все структуры состояния Rapier3D (`rigid_body_set`, `collider_set`, `impulse_joint_set`, `multibody_joint_set`, `integration_parameters`, `island_manager`, `broad_phase`, `narrow_phase`, `ccd_solver`, `gravity`), а также таблицы соответствия идентификаторов (`part_to_body`, `body_to_part`, `part_to_collider`, `joints`, `gear_constraints`), при этом создаются новые неблокирующие каналы событий столкновений (`crossbeam::channel::unbounded()`) и чистый экземпляр `PhysicsPipeline::new()`.
   - Добавлены публичные методы инспекции физического мира:
     - `PhysicsWorld::has_part(&self, part_id: &str) -> bool`
     - `PhysicsWorld::is_part_dynamic(&self, part_id: &str) -> Result<bool, PhysicsError>`

2. **`crates/mcp-cad-server/src/tools/export.rs` и `crates/mcp-cad-server/src/tools/mod.rs`**:
   - Удалён мёртвый файл `crates/mcp-cad-server/src/tools/export.rs` с неиспользуемыми функциями `export_stl` и `import_stl` (фактическая диспетчеризация MCP-инструментов `export_stl` / `import_stl` выполняется через `tools::cad`).
   - Из `crates/mcp-cad-server/src/tools/mod.rs` удалено объявление `pub mod export;`.

3. **`crates/mcp-cad-server/src/tools/cad.rs`**:
   - В `boolean_difference` добавлена синхронизация физического мира (`state.physics_world.remove_part(&target_id)` + `state.physics_world.add_part(target_part, !target_part.is_fixed)`) с проверкой результата и откатом состояния детали/DAG при ошибке.
   - В `boolean_union`, `boolean_intersection`, `transform_part` и `optimize_feature_tree` исправлен жёстко закодированный аргумент `is_dynamic = true` при перерегистрации детали в `state.physics_world`: теперь передаётся `!part.is_fixed`, что сохраняет статус закреплённой детали (`RigidBodyType::Fixed`), заданный при `create_part(..., is_fixed: true)`.
   - В `batch_cad_operations` добавлено сохранение снимка `let prev_physics_world = state.physics_world.clone();` перед выполнением пакета операций и полный откат `state.physics_world = prev_physics_world;` на всех трёх путях аварийного завершения (отсутствие поля `"tool"`, попытка рекурсивного вызова `"batch_cad_operations"`, ошибка выполнения любого шага пакета).
   - Добавлен модульный тест `tools::cad::tests::test_p2_t12_fixed_part_preserved_and_batch_physics_rollback`.

4. **`crates/mcp-cad-server/tests/mcp_contract_tests.rs`**:
   - Добавлен интеграционный контрактный тест `test_mcp_p2_t12_batch_physics_rollback_and_fixed_part_preservation`, проверяющий:
     - сохранение `RigidBodyType::Fixed` (`!part.is_fixed`) у закреплённой детали после `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part` и `optimize_feature_tree`;
     - транзакционный откат `state.physics_world` (включая созданные внутри пакета детали и шарниры `create_joint`) при ошибке на последующем шаге `batch_cad_operations`;
     - неподвижность закреплённой детали (`displacement < 1e-6`) под действием гравитации в `run_physics_simulation` после всех CSG/transform операций и отката батча.

5. **`docs/MCP_API_REFERENCE.md`**:
   - Актуализировано описание транзакционного отката в разделе `26. batch_cad_operations`: явно зафиксировано восстановление `physics_world` (твёрдых тел, коллайдеров, шарниров и приводов) наряду с `parts`, `materials`, кэшами FEA и `requirements`.

---

## 2) Как проверено (команды и результаты)

```bash
export PATH="/usr/lib/rust-1.91/bin:$PATH"
cargo test -p mcp-cad-physics
# test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

cargo test -p mcp-cad-server --lib
# test result: ok. 33 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

cargo test -p mcp-cad-server --test mcp_contract_tests
# test result: ok. 33 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

RUSTFLAGS="-D warnings" cargo check --workspace --all-targets
# Finished `dev` profile [unoptimized + debuginfo] target(s)
```

---

## 3) Тесты до / после

- **До правок**: 211 passed, 0 failed, 0 ignored (в т.ч. `mcp-cad-server --lib`: 32 passed, `mcp_contract_tests`: 32 passed).
- **После правок**: 213 passed, 0 failed, 0 ignored:
  - `mcp-cad-core`: 82 passed (47 unit + 12 `csg_invariants` + 5 `dag_tests` + 9 `primitive_invariants` + 9 `tessellation_tests`)
  - `mcp-cad-fea`: 13 passed (8 unit + 5 `fea_tests`)
  - `mcp-cad-physics`: 6 passed (`physics_tests`)
  - `mcp-cad-viewer`: 13 passed (9 unit + 4 `viewer_tests`)
  - `mcp-cad-server`: 99 passed (33 unit + 33 `mcp_contract_tests` + 25 `bench_suite` + 1 `gear_kinematic_test` + 7 `qa_wide_scan`)

---

## 4) Что не удалось и остаточные риски

- Все пункты критериев приёмки P2-T12 выполнены в полном объёме.
- Остаточный риск связан с тем, что `PhysicsWorld::remove_part` удаляет привязанные к детали шарниры при перерегистрации детали в CSG/transform (оформлено отдельной задачей P2-T18 в `TASKS.md`).

---

## 5) Побочные находки

- **P2-T18**: В `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part` и `optimize_feature_tree` (`crates/mcp-cad-server/src/tools/cad.rs`) перерегистрация детали в `physics_world` выполняется через `remove_part` + `add_part`. Поскольку `PhysicsWorld::remove_part` (`crates/mcp-cad-physics/src/simulator.rs:261-273`) удаляет все шарниры (`joints`) и зубчатые передачи (`gear_constraints`), связанные с этой деталью, любой шарнир, созданный *до* вызова `transform_part` / `boolean_*` / `optimize_feature_tree`, негласно теряется. Кроме того, инструменты модификации геометрии `add_feature_primitive`, `cut_hole`, `modify_feature`, `cut_helical_groove` и `tessellate_part` не обновляют коллайдер и массу детали в `state.physics_world`, если деталь уже была зарегистрирована в физическом мире. Заведено в `TASKS.md` как **P2-T18**.
