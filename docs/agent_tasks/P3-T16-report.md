# Отчёт о выполнении задачи P3-T16

**Задача:** P3-T16 — Итеративный геометрический решатель замкнутых кинематических цепей при пошаговом сканировании `sweep_joint` на четырёхзвенниках и кривошипно-ползунных механизмах  
**Статус:** ✅ Выполнено (`DONE`)  
**Дата:** 15.11.2026

---

## 1. Выполненные изменения

### 1.1. Физико-кинематическое ядро (`crates/mcp-cad-physics`)
- [`crates/mcp-cad-physics/src/telemetry.rs`](../../crates/mcp-cad-physics/src/telemetry.rs):
  - В `SweepStepResult` добавлены поля `closure_residual: f64` и `locked_up: bool` (с `#[serde(default)]` для обратной совместимости).
  - В `JointSweepReport` добавлены поля `closed_loop_detected: bool`, `lockup_detected: bool`, `lockup_step: Option<usize>`, `lockup_value: Option<f64>`, `max_closure_residual: f64` (с `#[serde(default)]`).
  - `safe_travel_range` учитывает как первую коллизию, так и первое геометрическое заклинивание (`lockup_value`); статус отчёта становится `"FAIL"` при обнаружении коллизии или заклинивания.
- [`crates/mcp-cad-physics/src/joint.rs`](../../crates/mcp-cad-physics/src/joint.rs):
  - Помощники `local_orthonormal_basis` и `perpendicular_axes` открыты с видимостью `pub(crate)` для переиспользования в решателе контуров.
- [`crates/mcp-cad-physics/src/simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs):
  - Реализована детекция замкнутых контуров в `sweep_joint_with_parts`: при наличии связей с внешней компонентой/землёй или внутренних петель (`touches_boundary > 0 || internal_joints_count > m_drv_set.len().saturating_sub(1)`) активируется итеративный многотельный решатель `ClosedLoopSolver`; для открытых кинематических цепей полностью сохранён прежний быстрый жёсткий проход с `closed_loop_detected = false`.
  - В `ClosedLoopSolver` реализовано пошаговое продолжение по параметру (continuation sub-stepping по $\le 3^\circ$ для `Revolute` и $\le 0.05\,L_{\text{char}}$ для `Prismatic`) и демпфированный Gauss-Newton / Levenberg–Marquardt через SVD на размерно-нормированном якобиане связей $\tilde{J}\,\Delta q = -\Phi(q)$ (`Fixed`, `Spherical`, `Revolute`, `Prismatic`, `Gear`, включая двустороннюю блокировку `limits: [c, c]` и сканирование внутренних шарниров контура).
  - Нормализация по характерному размеру механизма `char_len` и SVD-решение устраняют ложные срабатывания заклинивания на плоских избыточных связях (рангово-дефицитная система $20 \times 18$, ранг $17$).
  - При превышении физического допуска невязки контура фиксируются `lockup_step`, `lockup_value`, `locked_up = true` и `max_closure_residual` (исправлено также обновление `max_closure_residual` на заклинившем шаге).
- [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs):
  - Добавлены 4 интеграционных теста `P3-T16`: `test_p3_t16_sweep_joint_four_bar_full_rotation_and_obstacle`, `test_p3_t16_sweep_joint_slider_crank_and_obstacle`, `test_p3_t16_sweep_joint_geometric_lockup_detection`, `test_p3_t16_sweep_joint_open_chain_backward_compat`.

### 1.2. MCP-сервер и документация API (`crates/mcp-cad-server`, `docs/`)
- [`crates/mcp-cad-server/src/tools/physics.rs`](../../crates/mcp-cad-server/src/tools/physics.rs): ответ `sweep_joint` дополнен полями `closed_loop_detected`, `lockup_detected`, `lockup_step`, `lockup_value`, `lockup_angle_deg`, `lockup_angle_rad` (для `Revolute`), `max_closure_residual` и пошаговыми `closure_residual`, `locked_up`.
- [`crates/mcp-cad-server/src/tools/mod.rs`](../../crates/mcp-cad-server/src/tools/mod.rs): актуализировано описание инструмента №40 `sweep_joint`.
- [`crates/mcp-cad-server/tests/mcp_contract_tests.rs`](../../crates/mcp-cad-server/tests/mcp_contract_tests.rs): добавлен контрактный тест `test_mcp_p3_t16_sweep_joint_closed_loop_and_lockup` (четырёхзвенник Грасгофа $360^\circ$ в `mm`/`deg` и не-грасгофовский четырёхзвенник с геометрическим заклиниванием на шаге `18` / `90°`).
- [`docs/MCP_API_REFERENCE.md`](../MCP_API_REFERENCE.md): раздел №40 `sweep_joint` дополнен описанием решателя замкнутых контуров и выходных полей.

### 1.3. TypeScript-зеркало (`src/cad/`)
- [`src/cad/kinematics.ts`](../../src/cad/kinematics.ts): расширены типы `SweepStepResult` и `JointSweepReport`, реализованы `mat3ToScaledAxisTS`, `solveDampedSvdStepTS` (односторонний Jacobi SVD) и `createClosedLoopSolverTS` с полным математическим паритетом с Rust (`sweepJoint`).
- [`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts): обновлена сериализация ответа `case 'sweep_joint'`.
- [`src/cad/__tests__/analyze_mechanism.test.ts`](../../src/cad/__tests__/analyze_mechanism.test.ts): добавлены 3 теста (`5`–`7`) на четырёхзвенник, кривошипно-ползунный механизм с препятствием и детекцию геометрического заклинивания.

---

## 2. Результаты верификации (после финальной правки)

| Проверка | Команда | Результат |
|---|---|---|
| Physics suite | `cargo test -p mcp-cad-physics --no-fail-fast` | `tests/physics_tests.rs`: **22 passed, 1 failed** (новые `4/4` тестов `P3-T16` **PASS**; единственное падение `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall` — известная регрессия `P3-T19`); `src/lib.rs`: `0 tests`, doc-tests: `0 tests` |
| Server unit tests | `cargo test -p mcp-cad-server --lib --no-fail-fast -- --test-threads=1` | **60 passed, 0 failed** (`100% PASS`) |
| Server contract tests | `$env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --test mcp_contract_tests --no-fail-fast -- --test-threads=1` | **58 passed, 1 failed** (новый `test_mcp_p3_t16_sweep_joint_closed_loop_and_lockup` **PASS**; единственное падение `test_mcp_open_preview_window` — известная регрессия `P3-T20`) |
| TypeScript lint | `npm run lint` (`tsc --noEmit`) | **0 ошибок** (`exit 0`) |
| TypeScript mechanism tests | `npx tsx --test src/cad/__tests__/analyze_mechanism.test.ts` | **15 passed, 0 failed** (`3/3` новых теста `P3-T16` **PASS**) |
| Workspace Clippy | `cargo clippy --workspace --all-targets -- -D warnings` | **0 ошибок, 0 предупреждений** (`exit 0`) |
