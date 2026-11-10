# Отчёт о выполнении задачи P2-T17

**Задача:** `P2-T17 [S]` — Транзакционность `record.motor` при отказе `apply_motor` в `PhysicsWorld::set_joint_motor` (`simulator.rs:325`) и запрет `motor.is_some()` для `CadJointType::Gear` в `build_generic_joint` (`joint.rs:200`)  
**Дата:** 08.11.2026  
**Ветка:** `agent/P2-T17`  
**Статус:** ✅ DONE  

---

## 1. Что сделано (файлы, функции)

1. **Запрет `motor.is_some()` для `CadJointType::Gear` в `CadJointType::build_generic_joint` (`crates/mcp-cad-physics/src/joint.rs`)**:
   - В ветке `CadJointType::Gear { .. }` метода `build_generic_joint` добавлена проверка `if motor.is_some()`, возвращающая `Err(PhysicsError::InvalidJointConfiguration("Motors are not supported on Gear joints; apply motor to one of the connected Revolute joints".into()))` — в полном соответствии с поведением `Spherical`, `Fixed` и `apply_motor`.

2. **Транзакционное обновление `record.motor` в `PhysicsWorld::set_joint_motor` и `disable_joint_motor` (`crates/mcp-cad-physics/src/simulator.rs`)**:
   - В `PhysicsWorld::set_joint_motor` запись `record.motor = Some(motor)` перенесена **после** успешного выполнения `joint_type.apply_motor(&mut joint.data, motor)?`. При попытке установить мотор на шарнир `Spherical`, `Fixed` или `Gear` возвращается ошибка `PhysicsError::InvalidJointConfiguration`, а состояние `record.motor` в `PhysicsWorld::joints` остаётся неизменным (`None`).
   - Аналогично в `PhysicsWorld::disable_joint_motor` сброс `record.motor = None` выполняется только после успешного вызова `joint_type.disable_motor(&mut joint.data)?`.

3. **Уточнение структурированной ошибки `INVALID_JOINT_CONFIGURATION` (`crates/mcp-cad-server/src/error.rs`)**:
   - В `map_physics_error` для `PhysicsError::InvalidJointConfiguration(msg)`, содержащего `"motor"` или `"Motor"`, добавлены привязка `.with_param("joint_id")` и предметная подсказка `hint`, поясняющая, что моторы поддерживаются только на шарнирах `'revolute'` и `'prismatic'` (а для `'gear'` мотор прикладывается к одному из связанных вращательных шарниров).

4. **Модульные и контрактные тесты**:
   - `crates/mcp-cad-physics/tests/physics_tests.rs`: добавлен тест `test_p2_t17_set_joint_motor_transactional_and_gear_build_motor_rejected`, проверяющий отказ `build_generic_joint(Some(motor))` и `add_joint(..., Some(motor))` для `Spherical`, `Fixed` и `Gear`, а также сохранение `record.motor == None` после неудачного вызова `set_joint_motor`.
   - `crates/mcp-cad-server/src/tools/physics.rs`: добавлен unit-тест `test_p2_t17_set_joint_motor_on_unsupported_joints_preserves_state_and_save_load`, проверяющий код `INVALID_JOINT_CONFIGURATION` (`param: "joint_id"`), отсутствие ложного `has_motor: true` в `get_scene_summary` и успешный round-trip `save_project` / `load_project`.
   - `crates/mcp-cad-server/tests/mcp_contract_tests.rs`: добавлен интеграционный JSON-RPC тест `test_mcp_p2_t17_joint_motor_transactionality_and_gear_motor_rejection` (включая проверку отклонения повреждённого файла проекта с мотором на `Gear`-шарнире в `load_project`).

---

## 2. Как проверено (команды и итоговые строки `test result`)

- **Тесты физического движка `mcp-cad-physics`:**
  - Команда: `cargo test -p mcp-cad-physics`
  - Результат: `test result: ok. 8 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out`
- **Unit-тесты сервера `mcp-cad-server --lib`:**
  - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --lib`
  - Результат: `test result: ok. 36 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out`
- **Контрактные тесты `mcp_contract_tests`:**
  - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_p2_t17`
  - Результат: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 36 filtered out`
- **Полный прогон `mcp_contract_tests`:**
  - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test mcp_contract_tests`
  - Результат: `test result: ok. 37 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out`
- **Линтер Rust (`clippy`):**
  - Команда: `cargo clippy --workspace --all-targets -- -D warnings`
  - Результат: `0 errors, 0 warnings`
- **Тесты и сборка фронтенда:**
  - Команда: `npx tsx --test src/cad/cad_engine.test.ts` -> `28 passed, 0 failed`
  - Команда: `npm run lint` -> `0 errors`

---

## 3. Тесты до / после

| Набор тестов | До P2-T17 | После P2-T17 | Дельта |
|---|---|---|---|
| `mcp-cad-physics` (`physics_tests`) | 7 passed | **8 passed** | +1 (`test_p2_t17_set_joint_motor_transactional_and_gear_build_motor_rejected`) |
| `mcp-cad-server` (`--lib`) | 35 passed | **36 passed** | +1 (`test_p2_t17_set_joint_motor_on_unsupported_joints_preserves_state_and_save_load`) |
| `mcp-cad-server` (`mcp_contract_tests`) | 36 passed | **37 passed** | +1 (`test_mcp_p2_t17_joint_motor_transactionality_and_gear_motor_rejection`) |
| Остальные крейты и суиты workspace | 147 passed | **147 passed** | 0 регрессий |
| **Всего по Rust Workspace** | **225 passed, 0 failed, 0 ignored** | **228 passed, 0 failed, 0 ignored** | **+3 теста (100% PASS)** |

---

## 4. Что не удалось и остаточные риски

- Все критерии приёмки задачи **P2-T17** выполнены полностью. Остаточных рисков нет.

---

## 5. Побочные находки (не чинить в рамках P2-T17, заведены в `TASKS.md`)

1. **Отсутствие валидации `teeth_a` / `teeth_b`, нулевых осей `axis_a` / `axis_b` и `ratio` для `CadJointType::Gear`, а также невалидного `step_size_ms` в `run_physics_simulation` (`P2-T22`)**:
   - В `crates/mcp-cad-server/src/tools/physics.rs:270-278` при вычислении передаточного числа `ta / tb` из `teeth_a` и `teeth_b` не проверяется `ta > 0.0`, `tb > 0.0` и конечность `is_finite()`, из-за чего `teeth_a: 0` порождает `ratio = 0.0`, приводящий к делению на ноль (`-wb_proj / gear.ratio` $\to$ `NaN`/`Inf`) в `PhysicsWorld::step` (`simulator.rs:496`). Кроме того, в `CadJointType::build_generic_joint` (`joint.rs:204`) для `Gear` не проверяются ненулевая длина осей `axis_a`/`axis_b` и ненулевой конечный `ratio`.
   - В `run_physics_simulation` (`crates/mcp-cad-server/src/tools/physics.rs:426-431`) параметр `step_size_ms` не проверяется на тип, конечность и положительность (`> 0.0`), молча зажимаясь через `.max(0.0001)`.
   - Заведено в `TASKS.md` как задача **P2-T22**.
