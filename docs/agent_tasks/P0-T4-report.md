# Отчёт о выполнении задачи P0-T4

**Задача:** P0-T4 — Контракт ошибок: тест BUG-016  
**Дата:** 04.11.2026  
**Статус:** ✅ ВЫПОЛНЕНО

---

## 1. Что сделано (файлы, функции)

1. **Аудит проверок ошибок в тестах `crates/mcp-cad-server/tests/*.rs`**:
   - Проверены все обращения к `get("error")`, `["error"]` и `["isError"]` в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `qa_wide_scan.rs` и `gear_kinematic_test.rs`.
   - Подтверждено, что все остальные тесты ошибок выполнения инструментов (`boolean_union` при `target == tool` на стр. 1857, `define_material` при отрицательном модуле Юнга и $\nu \ge 0.5$ на стр. 1993 и 2011, `batch_cad_operations` при вложенном вызове и ошибке шага на стр. 2084 и 2112, а также `call_tool` в `qa_wide_scan.rs:55-58`) проверяют флаг `resp["result"]["isError"] == true` и читают текст ошибки из `resp["result"]["content"][0]["text"]`.
   - Единственным тестом, ошибочно ожидавшим верхнеуровневое поле JSON-RPC `resp.get("error").is_some()` для доменной ошибки валидации инструмента, был `test_bug016_revolve_boundary_and_self_intersection_validation`.

2. **Введение общего хелпера `assert_tool_error` и адаптация тестов (`crates/mcp-cad-server/tests/mcp_contract_tests.rs`)**:
   - Добавлена вспомогательная функция `assert_tool_error(resp: &Value) -> &str`, которая проверяет:
     1. отсутствие верхнеуровневого поля JSON-RPC `error` (`resp.get("error").is_none() || resp["error"].is_null()`);
     2. наличие флага `resp["result"]["isError"] == true`;
     3. наличие непустого диагностического текста в `resp["result"]["content"][0]["text"]` и возвращает его для дальнейших проверок.
   - В `test_bug016_revolve_boundary_and_self_intersection_validation` все три негативных сценария переведены на `assert_tool_error(&resp)` с дополнительной проверкой смысла диагностического сообщения для каждого случая:
     - `r < 0` (пересечение оси вращения) $\to$ проверка подстроки `"cannot be negative"`;
     - `angle_deg = 720.0` (вне диапазона $(0, 360]$) $\to$ проверка подстроки `"angle_deg"`;
     - восьмёркообразный самопересекающийся контур $\to$ проверка подстроки `"self-intersecting"`.
   - Существующие проверки `assert_eq!(resp["result"]["isError"], true)` в `test_mcp_boolean_union_and_intersection`, `test_mcp_define_material` и `test_mcp_batch_cad_operations` также унифицированы через `assert_tool_error(&resp)`.
   - Файлы `crates/mcp-cad-server/src/protocol.rs` и `crates/mcp-cad-server/src/tools/cad.rs` не изменялись (согласно ограничению задачи «Не менять `protocol.rs` и валидацию в `cad.rs`»).

3. **Документирование контракта ошибок (`docs/MCP_API_REFERENCE.md`)**:
   - В раздел «Общие положения» добавлен подраздел **«Ошибки»**, чётко разграничивающий:
     - **Ошибки выполнения инструмента (`result.isError = true`)** — валидация аргументов и геометрии (`InvalidArgument`, `InvalidParameter`), ненайденные сущности (`PartNotFound`, `FeatureNotFound`, `JointNotFound`) и ошибки подсистем (`Cad`, `Fea`, `Physics`, `Viewer`, `Io`, `Internal`);
     - **Протокольные ошибки JSON-RPC 2.0 (верхнеуровневое поле `error`)** — коды `-32700` (`parse_error`), `-32600` (`invalid_request`), `-32601` (`method_not_found`), `-32602` (`invalid_params` при отсутствии `params.name` в `tools/call`) и `-32603` (`internal_error`).

4. **Рекомендация по архитектурному решению `[DECISION]`: `ServerError::ToolNotFound`**:
   - **Текущее поведение**: При вызове `tools/call` с неизвестным именем инструмента `params.name` функция `dispatch_tool` (`crates/mcp-cad-server/src/tools/mod.rs:982`) возвращает `Err(ServerError::ToolNotFound(name))`, который в `protocol.rs:224` преобразуется в `ToolCallResult::error(...)` с `result.isError = true`.
   - **Варианты**:
     - **Вариант А (протокольная ошибка JSON-RPC `-32601` / `-32602`)**: В `protocol.rs` (ветка `"tools/call"`) перехватывать `Err(ServerError::ToolNotFound(name))` отдельно от остальных вариантов `ServerError` и возвращать `JsonRpcResponse::error(id, JsonRpcError::invalid_params(format!("Unknown tool: '{name}'")))` (или `-32601 Method not found`), как рекомендует спецификация MCP для вызова несуществующего инструмента.
     - **Вариант Б (оставить `result.isError = true` с кодом `TOOL_NOT_FOUND` в P2-T3)**: Сохранить обработку всех `ServerError` через `result.isError = true`, чтобы ИИ-агент получал ошибку `TOOL_NOT_FOUND` со списком доступных инструментов в `hint` в том же формате, что и остальные ошибки вызова инструментов, без риска обрыва сессии у строгих JSON-RPC клиентов.
   - **Рекомендация по умолчанию**: При реализации **P2-T3** («Структурированные ошибки») обсудить выбор между Вариантом А и Вариантом Б; если приоритетом является самокоррекция агента без падения MCP-клиента, Вариант Б с подсказкой доступных инструментов в `to_error_body()` практичнее, а если приоритетом является полное формальное соответствие JSON-RPC 2.0 / MCP — выделить `ServerError::ToolNotFound` в `protocol.rs:222` в `JsonRpcError::invalid_params` (`-32602`). В рамках **P0-T4** поведение `protocol.rs` не менялось.

---

## 2. Как проверено (команды и итоговые строки `test result`)

- **Воспроизведение дефекта до правок:**
  - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test mcp_contract_tests test_bug016`
  - Результат до правок: `test result: FAILED. 0 passed; 1 failed; 0 ignored; 0 measured; 20 filtered out; finished in 0.11s` (`Revolve with r < 0 must return an error` на `mcp_contract_tests.rs:2440`).
- **Проверка целевого теста после правок:**
  - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test mcp_contract_tests test_bug016`
  - Результат: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 20 filtered out; finished in 0.15s`
- **Полный прогон контрактных тестов `mcp_contract_tests`:**
  - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test mcp_contract_tests`
  - Результат: `test result: ok. 21 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 30.25s`
- **Сборка всего workspace (`cargo build --workspace`):**
  - Команда: `cargo build --workspace`
  - Результат: `Finished dev profile [unoptimized + debuginfo] target(s)` (0 ошибок).

---

## 3. Тесты до / после

| Таргет / Тест | До P0-T4 | После P0-T4 |
|---|---|---|
| `mcp_contract_tests::test_bug016_revolve_boundary_and_self_intersection_validation` | ❌ FAILED (`0 passed; 1 failed`) | ✅ PASS (`1 passed; 0 failed`) |
| Весь файл `mcp-cad-server --test mcp_contract_tests` | `20 passed; 1 failed` | ✅ **`21 passed; 0 failed`** |
| Workspace всего (`cargo test --workspace --no-fail-fast`) | `136 passed; 4 failed; 8 ignored` | `137 passed; 3 failed; 8 ignored` |

Три оставшихся падения во всём workspace находятся исключительно в `qa_wide_scan.rs` и относятся к задачам качества сетки:
1. `test_bug_003_partial_revolve_mesh_and_renders` $\to$ **P0-T5**
2. `test_bug_005_polygon_prism_chamfer_and_sides_validation` $\to$ **P0-T6**
3. `test_bug_006_csg_booleans_and_holes_watertight` $\to$ **P0-T7**

---

## 4. Что не удалось и остаточные риски

- Все цели и критерии приёмки **P0-T4** выполнены полностью.
- Остаточный риск: в тесте `test_bug015_create_joint_and_create_part_schema_contract` (`mcp_contract_tests.rs:2423`) осталась проверка `assert!(j_resp.get("error").is_none(), ...)`, которая маскирует скрытую ошибку выполнения `create_joint` для пустой детали без геометрии (подробно описано ниже в разделе 5 и заведено в `TASKS.md` как **P0-T11**).

---

## 5. Побочные находки (не чинить, только перечислить)

1. **Скрытый сбой `create_joint` на детали без геометрии и ложноположительный `assert` в `test_bug015_create_joint_and_create_part_schema_contract` (`mcp_contract_tests.rs:2392–2424`, `tools/physics.rs:141–150`)**:
   - В `test_bug015` создаётся деталь `Rotor` вызовом `create_part` без добавления какого-либо примитива (`part.mesh.vertices.is_empty() == true`), после чего вызывается `create_joint` (`joint_type: "revolute"`, `part_a: "ground"`, `part_b: p_id`, без `anchor_a`/`anchor_b`).
   - В `crates/mcp-cad-server/src/tools/physics.rs:145` деталь регистрируется в физическом мире только при условии `if !part.mesh.vertices.is_empty()`. Поскольку у только что созданной детали сетка пуста, `state.physics_world.add_part(...)` пропускается, и последующий вызов `state.physics_world.add_joint(...)` (`tools/physics.rs:250–252`) возвращает `Err(PhysicsError::PartNotFound(p_id))` $\to$ `result.isError = true` (`"Physics error: Part not found in simulation world: ..."`).
   - Тест `test_bug015` на строке 2423 проверяет только `assert!(j_resp.get("error").is_none(), ...)`, поэтому проходит, хотя фактически создание шарнира завершилось ошибкой `result.isError = true`.
   - Заведена новая задача **P0-T11** в `TASKS.md`.
