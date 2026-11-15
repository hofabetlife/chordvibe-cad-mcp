# Отчёт о выполнении задачи P3-T15

**Задача:** `P3-T15 [S]` — Учёт активных упоров (`limits`) шарниров на границе хода и атрибуция избыточных связей (`redundant_joint_ids`) в `analyze_mechanism`  
**Статус:** ✅ DONE  
**Дата:** 15.11.2026  

---

## 1. Резюме изменений (Что сделано и почему)

1. **Атрибуция избыточных связей по конкретным шарнирам (`crates/mcp-cad-physics/src/{joint.rs, simulator.rs}`, `src/cad/kinematics.ts`)**:
   - В `MechanismAnalysisReport` и его TypeScript-зеркало добавлено поле `redundant_joint_ids: Vec<String>` / `string[]`.
   - При `redundant_constraints > 0` функция `find_redundant_joint_ids` (`findRedundantJointIdsTS` в TS) для каждого активного шарнира $k$ выполняет исключение его строк из матрицы Якоби $J$ (`leave-one-joint-out` SVD с тем же порогом `rank_tolerance`). Шарнир включается в `redundant_joint_ids`, если количество его линейно зависимых строк $\text{rows}_k - (\text{rank}(J) - \text{rank}(J \setminus J_k)) > 0$.
   - Использование leave-one-joint-out SVD вместо переборного QR делает атрибуцию детерминированной и нечувствительной к порядку шарниров (на симметричных замкнутых контурах, таких как плоский четырёхзвенник или кривошипно-ползунный механизм, в списке присутствуют все 4 участвующих шарнира, а независимый шарнир `j_base` при наличии дублированного `j_arm`/`j_arm_dup` в список не попадает).

2. **Диагностика активных упоров `limits` и двусторонней блокировки (`crates/mcp-cad-physics/src/{joint.rs, simulator.rs}`, `src/cad/kinematics.ts`)**:
   - Введена структура `JointLimitStatus` (`joint_id`, `joint_type`, `boundary`, `current_value`, `limits`, `motor_blocked`; на уровне MCP-ответа добавляется `unit`: `"deg"` для `Revolute` и активная `length_unit` `"mm"`/`"m"` для `Prismatic`).
   - В `joint.rs` выделены общие вспомогательные функции `local_orthonormal_basis`, `perpendicular_axes`, `unwrap_angle_near_limits`, `is_zero_range_limits`, `CadJointType::effective_constraints_count()` и `CadJointType::current_coordinate()` (без дублирования геометрии базиса в `fill_jacobian_rows` и расчёта координаты).
   - При одностороннем достижении или выходе за `limits` (`min < max`, допуск $10^{-5}$) формируется запись в `joints_at_limits` со статусом `"min"` / `"max"` / `"below_min"` / `"above_max"` и предупреждение в `warnings` (с указанием заблокированного мотора при `motor_blocked: true`), не уменьшая одностороннюю подвижность `dof_numeric`.
   - При вырожденном нулевом диапазоне `limits` (`|max - min| <= 1e-9`) упор трактуется как двусторонняя блокировка (`boundary: "locked"`): `effective_constraints_count()` возвращает `6`, а `fill_jacobian_rows` добавляет 6-ю строку Якобиана вдоль рабочей оси шарнира, снижая `dof_numeric` (например, с `1` до `0` на однозвенном механизме).

3. **Синхронизация MCP-контракта, единиц измерения и документации**:
   - В `crates/mcp-cad-server/src/tools/physics.rs` ответ `analyze_mechanism` расширен полями `redundant_joint_ids` и `joints_at_limits` (с переводом углов в `deg` и линейных координат в активную единицу `length_unit`).
   - В `crates/mcp-cad-server/src/tools/requirements.rs` поля `redundant_joint_ids` и `joints_at_limits` проброшены в диагностику требования `mechanism_dof`.
   - В `crates/mcp-cad-server/src/tools/mod.rs` обновлено описание инструмента `analyze_mechanism` и добавлен опциональный параметр `length_unit` (`["mm", "m"]`) в `input_schema`.
   - В `src/cad/mcp_dispatcher.ts` синхронизировано сохранение `limits` (с пересчётом в `rad` / `m`) в `create_joint` и масштабирование `joints_at_limits` в `analyze_mechanism`.
   - Обновлены [`docs/MCP_API_REFERENCE.md`](../MCP_API_REFERENCE.md) (§38), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md) и [`TASKS.md`](../../TASKS.md).

---

## 2. Изменённые файлы

- `crates/mcp-cad-physics/src/joint.rs` — `JointLimitStatus`, `MechanismAnalysisReport::{redundant_joint_ids, joints_at_limits}`, `effective_constraints_count()`, `current_coordinate()`, 6-я строка Якобиана при вырожденном `limits`, общие геометрические/угловые хелперы.
- `crates/mcp-cad-physics/src/lib.rs` — публичный реэкспорт `JointLimitStatus`.
- `crates/mcp-cad-physics/src/simulator.rs` — расчёт `redundant_joint_ids` (`find_redundant_joint_ids`), оценка упоров (`evaluate_joint_limit_status`) и генерация предупреждений в `analyze_mechanism_with_scene_parts`.
- `crates/mcp-cad-physics/tests/physics_tests.rs` — проверки `redundant_joint_ids` в `test_p3_t9_analyze_mechanism_benchmarks` и новый тест `test_p3_t15_redundant_joint_attribution_and_active_limits`.
- `crates/mcp-cad-server/src/tools/physics.rs` — поля `redundant_joint_ids` и `joints_at_limits` с конвертацией единиц и модульный тест `test_p3_t15_analyze_mechanism_limits_and_redundant_joints_unit`.
- `crates/mcp-cad-server/src/tools/requirements.rs` — проброс `redundant_joint_ids` и `joints_at_limits` в проверку `mechanism_dof`.
- `crates/mcp-cad-server/src/tools/mod.rs` — обновлённое описание `analyze_mechanism` и параметр `length_unit` в схеме.
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs` — проверка `redundant_joint_ids`/`joints_at_limits` в `test_mcp_p3_t9_analyze_mechanism_and_requirement_contract` и новый контрактный тест `test_mcp_p3_t15_analyze_mechanism_limits_and_redundant_joints`.
- `src/cad/kinematics.ts` — зеркальная реализация `JointLimitStatus`, `redundant_joint_ids`, `joints_at_limits`, `unwrapAngleNearLimits` и 6-й строки Якобиана при нулевом `limits`.
- `src/cad/mcp_dispatcher.ts` — сохранение `limits` в `create_joint` и конвертация единиц `joints_at_limits` в `analyze_mechanism`.
- `src/cad/__tests__/analyze_mechanism.test.ts` — проверки в тесте 7 и новый тест 8 (`P3-T15`).
- `docs/MCP_API_REFERENCE.md`, `PROJECT_CONTEXT.md`, `TASKS.md`, `docs/agent_tasks/P3-T15-report.md` *(новый)* — актуализированная документация и отчёт.

---

## 3. Как проверено (команды и результаты после финальной правки кода)

1. **`mcp-cad-physics` (`cargo test -p mcp-cad-physics --no-fail-fast`)**:
   ```text
   test result: FAILED. 18 passed; 1 failed; 0 ignored; 0 measured; 0 filtered out; finished in 4.01s
   ```
   - `test_p3_t9_analyze_mechanism_benchmarks` — **PASS (`ok`)**;
   - `test_p3_t15_redundant_joint_attribution_and_active_limits` — **PASS (`ok`)**;
   - Единственное падение — известная регрессия **`P3-T19`**: `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall` (`physics_tests.rs:1635:5`, `left: Some(78)`, `right: Some(27)`).

2. **`mcp-cad-server --lib` (`$env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --lib --no-fail-fast -- --test-threads=1`)**:
   ```text
   test result: ok. 60 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 56.69s
   ```
   - Включая `tools::physics::tests::test_p3_t9_analyze_mechanism_unit` (`ok`) и `tools::physics::tests::test_p3_t15_analyze_mechanism_limits_and_redundant_joints_unit` (`ok`).

3. **`mcp-cad-server` контрактные тесты (`$env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --test mcp_contract_tests --no-fail-fast -- --test-threads=1`)**:
   ```text
   test result: FAILED. 57 passed; 1 failed; 0 ignored; 0 measured; 0 filtered out; finished in 60.56s
   ```
   - `test_mcp_p3_t9_analyze_mechanism_and_requirement_contract` — **PASS (`ok`)**;
   - `test_mcp_p3_t15_analyze_mechanism_limits_and_redundant_joints` — **PASS (`ok`)**;
   - Единственное падение — известная задача **`P3-T20`**: `test_mcp_open_preview_window` (`mcp_contract_tests.rs:716:5`, `left: String("empty")`, `right: "headless"`).

4. **TypeScript (`npm run lint; npx tsx --test src/cad/__tests__/analyze_mechanism.test.ts`)**:
   ```text
   > tsc --noEmit (exit 0)
   ℹ tests 12
   ℹ suites 2
   ℹ pass 12
   ℹ fail 0
   ```
   - Включая тесты 7 (`P3-T9`) и 8 (`P3-T15`) — **12/12 PASS**.

5. **Линтер Rust (`cargo clippy --workspace --all-targets -- -D warnings`)**:
   ```text
   Finished `dev` profile [optimized + debuginfo] target(s) in 8.09s
   CLIPPY_EXIT=0 (0 ошибок, 0 предупреждений во всех 5 крейтах)
   ```

---

## 4. Новые проблемы и задачи

- Новых проблем, не учтённых в реестре `TASKS.md` (`P3-T19`, `P3-T20`, `P3-T22`, `P3-T23`), не выявлено.
