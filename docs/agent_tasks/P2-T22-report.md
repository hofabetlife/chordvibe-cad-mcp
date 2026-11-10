# Отчёт о выполнении задачи P2-T22

- **Статус**: DONE
- **Задача**: `P2-T22 [S]` — Валидация `teeth_a`/`teeth_b`, `ratio` и осей `axis_a`/`axis_b` для `CadJointType::Gear` (`joint.rs`, `tools/physics.rs`) и `step_size_ms > 0` в `run_physics_simulation`

## Затронутые файлы
- `crates/mcp-cad-physics/src/joint.rs` — в `CadJointType::build_generic_joint` для `CadJointType::Gear { ratio, axis_a, axis_b }` добавлена валидация конечности и ненулевого значения `ratio` (`!ratio.is_finite() || ratio.abs() < 1e-9`), а также ненулевых норм векторов `axis_a` и `axis_b` (`norm < 1e-7 || !norm.is_finite()`). При нарушении возвращается `PhysicsError::InvalidJointConfiguration`.
- `crates/mcp-cad-physics/tests/physics_tests.rs` — добавлен unit-тест `test_p2_t22_gear_joint_validation`, проверяющий отказ при `ratio == 0.0`, `NaN`, `Inf`, `1e-12`, нулевых и нечисловых векторах `axis_a`/`axis_b`, а также успешное добавление валидного зубчатого сопряжения в `PhysicsWorld`.
- `crates/mcp-cad-server/src/tools/physics.rs` — 
  - В `create_joint` для ветки `"gear"` добавлена строгая проверка типов и значений: `ratio` проверяется на конечность и `abs >= 1e-9`; `teeth_a` и `teeth_b` проверяются на строго положительные конечные числа (`> 0.0`); вычисляемое `ratio = teeth_a / teeth_b` проверяется на ненулевое конечное значение; оси `axis_a` и `axis_b` проверяются на ненулевые векторы (`norm >= 1e-7`). Добавлен unit-тест `test_p2_t22_gear_validation_and_step_size`.
  - В `run_physics_simulation` аргумент `step_size_ms` теперь строго проверяется на положительное конечное число `> 0.0` (вместо неявного `unwrap_or(16.6667)`), возвращая ошибку `INVALID_ARGUMENT` с `param: "step_size_ms"`.
- `crates/mcp-cad-server/src/error.rs` — в `map_invalid_argument_error` и `map_physics_error` добавлена привязка параметров `param: "teeth_a"`, `"teeth_b"`, `"ratio"`, `"axis_a"`, `"axis_b"`, `"step_size_ms"` и информативных подсказок `hint`.
- `crates/mcp-cad-server/src/tools/mod.rs` — в JSON-схемах `create_joint` и `run_physics_simulation` добавлены `exclusiveMinimum: 0.0` для `teeth_a`, `teeth_b`, `step_size_ms`, `duration_seconds`.
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs` — добавлен интеграционный MCP-тест `test_mcp_p2_t22_gear_joint_and_simulation_step_validation`, проверяющий структуру ошибок `code: "INVALID_ARGUMENT"` и точные имена `param`. Также исправлены вызовы MCP в ранее сломанных тестах `test_mcp_p2_t15` и `test_mcp_p2_t13`.
- `docs/MCP_API_REFERENCE.md` — обновлена документация параметров `create_joint` и `run_physics_simulation` с описанием правил валидации передаточного отношения, числа зубьев, осей и шага интегрирования.
- `TASKS.md` — статус задачи `P2-T22` переведён в `✅ DONE`.

## Корневая причина и решение
1. **Деление на ноль при `ratio == 0.0` и некорректных зубьях шестерней в симуляторе**:
   - В `crates/mcp-cad-physics/src/simulator.rs:564-581` синхронизация вращения шестерней на каждом шаге интегрирования выполняет `delta_wa = -wb_proj / gear.ratio`. Если `gear.ratio == 0.0` или `NaN`, это приводило к делению на ноль и заражению угловых скоростей `NaN`/`Inf`.
   - В `create_joint` (`crates/mcp-cad-server/src/tools/physics.rs:188-203`) при передаче отрицательных или нулевых значений `teeth_a` / `teeth_b` вычислялось отрицательное или нулевое `ratio`, а нечисловые строки не валидировались до парсинга.
   - **Решение**: Валидация вынесена как на уровень ядра физики (`build_generic_joint`), так и на уровень входного MCP-инструмента (`create_joint`). Проверяется `|ratio| >= 1e-9`, `teeth_a > 0`, `teeth_b > 0`, и длины векторов `axis_a`, `axis_b` $\ge 10^{-7}$.
2. **Шаг симуляции `step_size_ms <= 0`**:
   - В `run_physics_simulation` аргумент `step_size_ms` парсился через `.and_then(|v| v.as_f64()).unwrap_or(16.6667)`, из-за чего неположительные значения молча сглаживались через `(step_size_ms / 1000.0).max(0.0001)`.
   - **Решение**: Добавлена строгая проверка `step_size_ms > 0.0 && is_finite()`, возвращающая стандартную структурированную ошибку `INVALID_ARGUMENT` с `param: "step_size_ms"`.

## Результаты тестов (до / после)
- `cargo test -p mcp-cad-physics`: **9 passed → 10 passed, 0 failed**.
- `cargo test -p mcp-cad-server --lib`: **34 passed → 35 passed, 0 failed**.
- `cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_p2_t22`: **1 passed, 0 failed**.
- `cargo test -p mcp-cad-fea`: **13 passed, 0 failed**.
- `cargo test -p mcp-cad-viewer`: **10 passed, 0 failed**.
- `npx tsx --test src/cad/cad_engine.test.ts`: **29 passed, 0 failed**.
- `npm run lint` (`tsc --noEmit`): **0 errors**.

## Выполнение критериев приёмки
- [x] Валидация `teeth_a` и `teeth_b` (`> 0`) в `create_joint` с возвратом `INVALID_ARGUMENT` и `param: "teeth_a"` / `"teeth_b"`.
- [x] Валидация `ratio` (конечное, $|\text{ratio}| \ge 10^{-9}$) в `joint.rs` и `tools/physics.rs` с `param: "ratio"`.
- [x] Валидация `axis_a` и `axis_b` (ненулевые векторы, норма $\ge 10^{-7}$) с `param: "axis_a"` / `"axis_b"`.
- [x] Валидация `step_size_ms > 0` в `run_physics_simulation` с `param: "step_size_ms"`.
- [x] Unit-тесты в `physics_tests.rs` и `tools/physics.rs`, контрактный тест в `mcp_contract_tests.rs`.
- [x] Обновлена документация `MCP_API_REFERENCE.md`.
