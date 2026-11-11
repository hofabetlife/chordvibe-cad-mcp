# Отчёт о выполнении задачи P2-T11

**Задача:** P2-T11 — Валидация числовых параметров в `set_joint_motor` (`target_velocity`, `max_force >= 0`)  
**Дата:** 07.11.2026  
**Ветка:** `agent/P2-T11`  
**Статус:** ✅ ВЫПОЛНЕНО  

---

## 1. Что сделано (файлы, функции)

1. **Строгая валидация `target_velocity` и `max_force` в `set_joint_motor` (`crates/mcp-cad-server/src/tools/physics.rs`)**:
   - Разделена проверка отсутствия обязательного параметра (`None | Some(Value::Null)` $\to$ `"Missing required argument '<param>'"`) и неверного типа/неконечного числа (`v.as_f64()`).
   - Добавлена проверка конечности `target_velocity.is_finite()` и отсутствия переполнения при приведении к типу `Real` (`f32`) физического движка Rapier3D (`(target_velocity as f32).is_finite()`).
   - Добавлена проверка конечности `max_force.is_finite() && (max_force as f32).is_finite()` и неотрицательности `max_force >= 0.0` (с поддержкой граничного случая `max_force == 0.0` для свободного выбега и отрицательного `target_velocity` для реверса).
   - Добавлен unit-тест `tools::physics::tests::test_set_joint_motor_rejects_negative_and_non_finite_parameters`.

2. **Уточнение структурированных подсказок ошибок в `map_invalid_argument_error` (`crates/mcp-cad-server/src/error.rs`)**:
   - Для ошибок валидации `'target_velocity'` и `'max_force'` возвращается код `INVALID_ARGUMENT`, заполненное поле `param` (`"target_velocity"` или `"max_force"`) и предметная подсказка `hint` с указанием размерностей ($\text{рад/с}$ / $\text{м/с}$ и $\text{Н}\cdot\text{м}$ / $\text{Н}$, $\ge 0.0$).

3. **Обновление JSON Schema и документации (`crates/mcp-cad-server/src/tools/mod.rs`, `docs/MCP_API_REFERENCE.md`)**:
   - В `ToolDefinition` для `set_joint_motor` (`tools/mod.rs`) добавлено ограничение `"minimum": 0.0` для `max_force` и упомянут код `INVALID_ARGUMENT (non-finite target_velocity or max_force < 0)` в `Common errors:`.
   - В `docs/MCP_API_REFERENCE.md` (раздел `12. set_joint_motor`) задокументированы правила валидации `target_velocity` и `max_force`.

4. **Контрактный интеграционный тест (`crates/mcp-cad-server/tests/mcp_contract_tests.rs`)**:
   - Добавлен тест `test_mcp_p2_t11_set_joint_motor_numeric_validation`, проверяющий через JSON-RPC `tools/call`:
     - отказ при `max_force < 0` (`-10.0`) с кодом `INVALID_ARGUMENT` и `param == "max_force"`;
     - отказ при переполняющем `f32` значении `target_velocity: 1e100` (`param == "target_velocity"`) и `max_force: 1e100` (`param == "max_force"`);
     - отказ при передаче строки `"NaN"` (`param == "target_velocity"`);
     - успешное выполнение при граничных допустимых значениях `target_velocity: -8.0` и `max_force: 0.0`.

---

## 2. Как проверено (команды и итоговые строки `test result`)

- **Unit-тесты `mcp-cad-server --lib`:**
  - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --lib`
  - Результат: `test result: ok. 32 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out`
- **Контрактные тесты `mcp_contract_tests`:**
  - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test mcp_contract_tests`
  - Результат: `test result: ok. 32 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out`
- **Линтер Rust (`clippy`):**
  - Команда: `cargo clippy --workspace --all-targets -- -D warnings`
  - Результат: `0 errors, 0 warnings`
- **Сборка и проверка фронтенда:**
  - `npm run lint` (`tsc --noEmit`): `0 errors`
  - `npm run build`: `Build succeeded`

---

## 3. Тесты до / после

| Набор тестов | До P2-T11 | После P2-T11 | Дельта |
|---|---|---|---|
| `mcp-cad-server` (`--lib`) | 31 passed | **32 passed** | +1 (`test_set_joint_motor_rejects_negative_and_non_finite_parameters`) |
| `mcp-cad-server` (`mcp_contract_tests`) | 31 passed | **32 passed** | +1 (`test_mcp_p2_t11_set_joint_motor_numeric_validation`) |
| Остальные крейты и суиты workspace | 147 passed | **147 passed** | 0 регрессий |
| **Всего по Rust Workspace** | **209 passed, 0 failed, 0 ignored** | **211 passed, 0 failed, 0 ignored** | **+2 теста (100% PASS)** |

---

## 4. Что не удалось и остаточные риски

- Все критерии приёмки задачи **P2-T11** выполнены в полном объёме.
- Остаточные риски в части валидации входа `set_joint_motor` на уровне MCP-сервера отсутствуют.

---

## 5. Побочные находки (не чинить в рамках P2-T11, заведены в `TASKS.md`)

1. **Неатомарное изменение `record.motor` перед валидацией типа шарнира в `PhysicsWorld::set_joint_motor` (`crates/mcp-cad-physics/src/simulator.rs:325`) и отсутствие проверки `motor.is_some()` для `CadJointType::Gear` в `build_generic_joint` (`crates/mcp-cad-physics/src/joint.rs:200`)**:
   - В `PhysicsWorld::set_joint_motor` (`simulator.rs:325`) присваивание `record.motor = Some(motor)` выполняется **до** вызова `joint_type.apply_motor(&mut joint.data, motor)`. Если `set_joint_motor` вызван для шарнира типа `Spherical`, `Fixed` или `Gear`, метод `apply_motor` возвращает `Err(PhysicsError::InvalidJointConfiguration(...))`, однако `record.motor` в `PhysicsWorld::joints` остаётся равным `Some(motor)`.
   - Вследствие этого `get_scene_summary` (`metrics.rs:316`) начинает ложно показывать `has_motor: true` у неподдерживающего мотор шарнира, а `save_project` (`project.rs:146`) сериализует этот `motor` в файл проекта, из-за чего последующий `load_project` падает с ошибкой `Motors are not supported on Spherical/Fixed joints` при пересборке сцены!
   - Заведено в `TASKS.md` как задача **P2-T17**.
