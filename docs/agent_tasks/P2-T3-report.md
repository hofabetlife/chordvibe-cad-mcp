# Отчёт о выполнении задачи P2-T3

## 1. Задача и что было сделано
- **ID**: `P2-T3`
- **Название**: Структурированные ошибки (`to_error_body`)
- **Фаза**: P2 — Контракт «агент ↔ система»
- **Ветка**: `agent/P2-T3`

### Контекст
Ранее (`protocol.rs:233`) любая ошибка выполнения инструмента `ServerError` превращалась в вызов `ToolCallResult::error(err.to_string())`, где в массив `result.content[0].text` помещалась неструктурированная человекочитаемая строка при флаге `result.isError = true`. Из-за этого ИИ-агентам и автоматическим клиентам было сложно программно определять причину ошибки, извлекать имя проблемного параметра, понимать, возможно ли повторить операцию (`retryable`), и получать действенные подсказки (`hint`), такие как список существующих `part_id` или допустимых `primitive_type`.

### Что сделано
1. **Структура тела ошибки (`crates/mcp-cad-server/src/error.rs`)**:
   - Введены структуры `ErrorEnvelope { error: ErrorBody }` и `ErrorBody`:
     - `code: String` — стабильный машиночитаемый идентификатор ошибки (например, `PART_NOT_FOUND`, `INVALID_ARGUMENT`, `INVALID_PRIMITIVE_TYPE`, `UNKNOWN_MATERIAL`, `EMPTY_GEOMETRY`, `FEA_UNCONSTRAINED`, `FEA_SOLVER_CONVERGENCE_FAILED`);
     - `message: String` — человекочитаемое сообщение об ошибке (полная обратная совместимость);
     - `param: Option<String>` — имя проблемного параметра (например, `part_id`, `primitive_type`, `material`, `profile`, `angle_deg`, `fixed_faces`, `operations`);
     - `hint: Option<String>` — конкретное указание следующего шага для агента;
     - `retryable: bool` — флаг возможности повторения операции (например, при сходимости итерационного решателя МКЭ или симулятора физики);
     - `details: serde_json::Value` — контекстные данные (список существующих деталей, поддерживаемые типы примитивов, встроенные материалы, параметры сходимости, допуски).
   - Реализованы методы конструктора и билдера: `new`, `with_param`, `with_hint`, `with_retryable`, `with_details`, `to_envelope`, `to_json_string()`.

2. **Маппинг вариантов `ServerError` (`crates/mcp-cad-server/src/error.rs`)**:
   - Реализованы `ServerError::to_error_body(&self) -> ErrorBody` и `ServerError::to_error_body_with_parts(&self, existing_parts: Option<&[String]>) -> ErrorBody`.
   - Полностью покрыты все варианты `ServerError`:
     - `PartNotFound(id)`: код `PART_NOT_FOUND`, параметр `part_id`. Если передан контекст сессии `existing_parts`, подсказка и поле `details.available_parts` содержат список существующих деталей (`"Available parts: [part-1, part-2]. Inspect parts via 'list_parts' or create it with 'create_part'."`).
     - `FeatureNotFound(id)`: код `FEATURE_NOT_FOUND`, параметр `feature_id`, подсказка проверить фичи через `query_part_geometry`.
     - `JointNotFound(id)`: код `JOINT_NOT_FOUND`, параметр `joint_id`, подсказка проверить `create_joint`.
     - `ToolNotFound(name)`: код `TOOL_NOT_FOUND`, параметр `name`, подсказка вызвать `tools/list`.
     - `InvalidArgument(msg)` / `InvalidParameter(msg)`: выделение частых сценариев:
       - Неизвестный `primitive_type` → код `INVALID_PRIMITIVE_TYPE`, `param: "primitive_type"`, `details.supported_primitives: ["box", "cylinder", "sphere", "cone", "polygon_prism", "spur_gear", "revolve", "wheel_rim"]`.
       - Неизвестный `material` → код `UNKNOWN_MATERIAL`, `param: "material"`, `details.builtin_materials: [...]`.
       - Деталь уже существует → код `PART_ALREADY_EXISTS`, `param: "name"`.
       - Деталь без геометрии → код `EMPTY_GEOMETRY`, `param: "part_id"`, подсказка добавить примитивы.
       - Негативная координата `r` в `revolve` → `param: "profile"`.
       - Ошибка угла `angle_deg` в `revolve` → `param: "angle_deg"`.
       - Самопересечение профиля в `revolve` → `param: "profile"`.
       - Отсутствие граничных условий FEA → код `FEA_UNCONSTRAINED`, `param: "fixed_faces"`.
       - Отсутствие сил в FEA → код `FEA_NO_LOADS`, `param: "applied_forces"`.
       - Соединение детали самой с собой в `create_joint` → `param: "part_b"`.
       - Вложенный или сбойный вызов `batch_cad_operations` → `param: "operations"`.
       - Свойства материалов (`young_modulus_gpa`, `poisson_ratio`, `yield_strength_mpa`, `density_kg_m3`) → извлечение соответствующих имен параметров.
       - Общий хелпер `extract_quoted_param` для автоматического извлечения имени аргумента из кавычек `'...'`.
     - `Cad(CadError)`: маппинг `FeatureNotFound`, `MaterialNotFound`, `CsgError` (`CSG_BOOLEAN_ERROR`), `EvaluationError` (`MESH_INVALID` / `CAD_EVALUATION_ERROR`), `InvalidParameter`, `IoError`, `SerializationError`.
     - `Fea(FeaError)`: маппинг `EmptyMesh` (`FEA_EMPTY_MESH`), `DegenerateElement` (`FEA_DEGENERATE_ELEMENT`), `InvalidMaterial` (`FEA_INVALID_MATERIAL`), `UnconstrainedModel` (`FEA_UNCONSTRAINED`), `SolverConvergenceFailed` (`FEA_SOLVER_CONVERGENCE_FAILED`, `retryable: true`), `NoBoundaryNodesMatched` (`FEA_NO_BOUNDARY_NODES`), `NodeOutOfBounds`, `MeshGenerationFailed`, `Cad`, `General`.
     - `Physics(PhysicsError)`: маппинг `PartNotFound`, `JointNotFound`, `PartAlreadyExists`, `JointAlreadyExists`, `InvalidGeometry` (`PHYSICS_INVALID_GEOMETRY`), `ColliderCreationFailed` (`PHYSICS_COLLIDER_FAILED`), `InvalidJointConfiguration`, `SimulationError` (`PHYSICS_SIMULATION_ERROR`, `retryable: true`), `SerializationError`.
     - `Viewer(ViewerError)`: маппинг `EmptyMesh` (`VIEWER_EMPTY_MESH`), `InvalidDimensions` (`INVALID_DIMENSIONS`), `StressScalarMismatch` (`STRESS_SCALAR_MISMATCH`), `NoSuitableAdapter` (`GPU_ADAPTER_UNAVAILABLE`), `DeviceRequestFailed` (`GPU_DEVICE_ERROR`).
     - `Io` / `Json` / `Internal`: `IO_ERROR`, `JSON_ERROR`, `INTERNAL_ERROR` / `MESH_INVALID`.

3. **Интеграция в протокол сервера (`crates/mcp-cad-server/src/protocol.rs`)**:
   - В обработчике `tools/call` при возникновении `Err(err)` извлекается детерминированно отсортированный список существующих деталей `engine.parts.keys()`, формируется `err.to_error_body_with_parts(Some(&existing_parts))` и передается в `ToolCallResult::error(error_body.to_json_string())`.

4. **Экспорт наружу (`crates/mcp-cad-server/src/lib.rs`)**:
   - Добавлен экспорт `pub use error::{ErrorBody, ErrorEnvelope, ServerError};`.

5. **Тестирование**:
   - В `crates/mcp-cad-server/src/error.rs` добавлено 18 всесторонних unit-тестов для всех вариантов ошибок, подсказок со списком деталей, флагов retryable и round-trip десериализации.
   - В `crates/mcp-cad-server/tests/mcp_contract_tests.rs`:
     - Усилен общий хелпер `assert_tool_error(&resp)`, который теперь валидирует синтаксис JSON и наличие полей `error.code` и `error.message`.
     - Добавлен контрактный тест `test_structured_error_contract_and_part_not_found_hint` на проверку `PART_NOT_FOUND` (до и после создания деталей, со списком доступных ID), невалидного `primitive_type` со списком поддерживаемых типов и идентификации отсутствующего аргумента `width`.

6. **Документация**:
   - В [`docs/MCP_API_REFERENCE.md`](../MCP_API_REFERENCE.md) обновлен раздел «Ошибки»: задокументирован формат `ErrorBody`, приведены примеры `INVALID_ARGUMENT` и `PART_NOT_FOUND`, а также составлена полная таблица всех 35 стабильных машиночитаемых кодов ошибок.

---

## 2. Как проверено (команды и итоговые строки `test result`)

### 1. Unit-тесты `mcp-cad-server` (`src/error.rs`):
```bash
cargo test -p mcp-cad-server --lib
```
Результат:
```
running 18 tests
test error::tests::test_to_error_body_cad_csg_error ... ok
test error::tests::test_error_envelope_json_roundtrip ... ok
test error::tests::test_to_error_body_empty_geometry ... ok
test error::tests::test_to_error_body_fea_solver_convergence_failed ... ok
test error::tests::test_to_error_body_fea_unconstrained ... ok
test error::tests::test_to_error_body_feature_not_found ... ok
test error::tests::test_to_error_body_io_and_json ... ok
test error::tests::test_to_error_body_joint_not_found ... ok
test error::tests::test_to_error_body_part_not_found_with_parts ... ok
test error::tests::test_to_error_body_physics_simulation_error ... ok
test error::tests::test_to_error_body_invalid_primitive_type ... ok
test error::tests::test_to_error_body_missing_required_argument ... ok
test error::tests::test_to_error_body_part_not_found_without_parts ... ok
test error::tests::test_to_error_body_revolve_angle_deg ... ok
test error::tests::test_to_error_body_unknown_material ... ok
test error::tests::test_to_error_body_revolve_negative_r ... ok
test error::tests::test_to_error_body_viewer_empty_mesh ... ok
test error::tests::test_to_error_body_tool_not_found ... ok

test result: ok. 18 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

### 2. Контрактные тесты `mcp_contract_tests` (включая валидацию JSON в `assert_tool_error` и новый тест `test_structured_error_contract_and_part_not_found_hint`):
```bash
cargo test -p mcp-cad-server --test mcp_contract_tests
```
Результат:
```
test result: ok. 24 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 18.83s
```

### 3. Тесты `qa_wide_scan` и `bench_suite`:
```bash
cargo test -p mcp-cad-server --test qa_wide_scan
cargo test -p mcp-cad-server --test bench_suite
cargo test -p mcp-cad-server --test gear_kinematic_test
```
Результат:
- `qa_wide_scan`: `test result: ok. 13 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 66.47s`
- `bench_suite`: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 151.41s`
- `gear_kinematic_test`: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 1.52s`

### 4. Тесты остальных крейтов workspace:
```bash
cargo test -p mcp-cad-core --lib
cargo test -p mcp-cad-core --test csg_invariants
cargo test -p mcp-cad-core --test dag_tests
cargo test -p mcp-cad-core --test primitive_invariants
cargo test -p mcp-cad-core --test tessellation_tests
cargo test -p mcp-cad-fea
cargo test -p mcp-cad-physics
cargo test -p mcp-cad-viewer
```
Результат:
- `mcp-cad-core --lib`: 50 passed, 0 failed
- `csg_invariants`: 5 passed, 0 failed
- `dag_tests`: 14 passed, 0 failed
- `primitive_invariants`: 18 passed, 0 failed
- `tessellation_tests`: 6 passed, 0 failed
- `mcp-cad-fea`: 11 passed, 0 failed
- `mcp-cad-physics`: 6 passed, 0 failed
- `mcp-cad-viewer`: 10 passed, 0 failed

### 5. Проверка Clippy и TypeScript:
```bash
cargo clippy --workspace --all-targets -- -D warnings
npm run lint
```
Результат:
- `cargo clippy`: 0 ошибок, 0 предупреждений во всех крейтах и тестовых таргетах.
- `npm run lint`: `tsc --noEmit` завершился с кодом 0 (0 errors).

---

## 3. Тесты до и после

| Набор тестов | До P2-T3 | После P2-T3 | Изменение |
|---|---|---|---|
| `mcp-cad-server --lib` | 0 (отсутствовали) | **18 passed** | +18 новых тестов `ErrorBody` и вариантов `ServerError` |
| `mcp_contract_tests` | 23 passed | **24 passed** | +1 контрактный тест на `PART_NOT_FOUND` и валидацию структуры ошибок |
| `qa_wide_scan` | 13 passed | 13 passed | Без регрессий |
| `bench_suite` | 1 passed | 1 passed | Без регрессий |
| `gear_kinematic_test` | 1 passed | 1 passed | Без регрессий |
| `mcp-cad-core` (все тесты) | 93 passed | 93 passed | Без регрессий |
| `mcp-cad-fea` (все тесты) | 11 passed | 11 passed | Без регрессий |
| `mcp-cad-physics` (все тесты) | 6 passed | 6 passed | Без регрессий |
| `mcp-cad-viewer` (все тесты) | 10 passed | 10 passed | Без регрессий |
| **Итого в workspace** | **158 passed** | **177 passed (100% PASS, 0 failed, 0 ignored)** | **+19 тестов** |

---

## 4. Что не удалось и остаточные риски
- Все критерии приёмки P2-T3 закрыты в полном объёме.
- Остаточный риск: в `crates/mcp-cad-server/src/tools/cad.rs` часть кастомных проверок возвращает `ServerError::InvalidArgument` с текстом, не содержащим имени параметра в кавычках (например, `"Thickness must be greater than zero"`). Для таких случаев поле `param` остаётся `None`, а код — `"INVALID_ARGUMENT"`. В будущих задачах рефакторинга параметров можно продолжить стандартизацию строковых литералов или ввести типизированные варианты ошибок.

---

## 5. Побочные находки (не чинить, только перечислить)
1. В `src/cad/cad_engine.test.ts` остаётся падающий тест `FEA_QUICK_LOAD_PRESETS.length` (`5 !== 4`, заведён как **TS-FIX-01**).
2. Задачи **P2-T11**..**P2-T15** и **P3-T1**..**P3-T12** остаются в реестре `TASKS.md` для дальнейшего решения по приоритету.
