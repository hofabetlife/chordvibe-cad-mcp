# Отчёт о выполнении задачи P3-T14

**Задача:** `P3-T14 [S]` — Синхронизация длины и обязательных токенов `AGENT_INSTRUCTIONS` (`protocol.rs:173`) с контрактными тестами `test_mcp_initialize` и `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`  
**Статус:** ✅ DONE (синхронизация уже присутствовала в текущей кодовой базе после коммита `281a3ed`; подтверждено прохождением обоих контрактных тестов и чистотой `clippy`; актуализированы `TASKS.md`, `PROJECT_CONTEXT.md` и реестр находок `P3-T23`)  
**Дата:** 15.11.2026  
**Затронутые файлы:**
- `docs/agent_tasks/P3-T14-report.md` *(новый)*
- `TASKS.md`
- `PROJECT_CONTEXT.md`

---

## 1. Что сделано (проверка воспроизводимости и обновление документации)

Перед внесением правок проверено фактическое состояние `crates/mcp-cad-server/src/protocol.rs` и `crates/mcp-cad-server/tests/mcp_contract_tests.rs`. Дефект уже был устранён в коммите `281a3ed` (`feat: apply latest ChordVibe CAD MCP snapshot (from f9442e4)`):

1. **`crates/mcp-cad-server/src/protocol.rs:173-193` (`AGENT_INSTRUCTIONS`)**:
   - Длина строки `AGENT_INSTRUCTIONS` составляет **1569 символов**, что укладывается в контрактный диапазон `800..=1650` в `mcp_contract_tests.rs:4521` (комментарий `~1580 characters`).
   - В тексте одновременно сохранены все обязательные ключевые слова и инварианты `P2-T10` (`docs/AGENT_GUIDE.md`, `get_scene_summary`, `length_unit`, `set_requirements`, `auto_check`, `batch_cad_operations`, `validate_mesh`, `run_fea_analysis`, `query_part_geometry`, `STALE`, `isError`) и `P2-T39` (`SKILL.md`, `crates/`, `[-height/2, +height/2]`, `csg_union`, `csg_difference`, `radius_bottom`, `radius_top`, `cut_helical_groove`, `capture_viewport`, `open_preview_window`).
2. **`crates/mcp-cad-server/tests/mcp_contract_tests.rs:86-123, 4495-4633`**:
   - `test_mcp_initialize` проверяет `docs/AGENT_GUIDE.md` и `get_scene_summary` — проходит (`ok`).
   - `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` проверяет длину `800..=1650`, все 21 обязательный токен `AGENT_INSTRUCTIONS`, `docs/AGENT_GUIDE.md` (152 строки в диапазоне `40..=160`) и схемы/описания всех 40 инструментов — проходит (`ok`).

Правки в Rust-код не потребовались; обновлены `TASKS.md` и `PROJECT_CONTEXT.md`, а выявленная при проверке проблема с зависанием параллельного прогона `mcp_contract_tests` и `qa_wide_scan` (и изменением tracked-файлов `qa/results/*.json`) оформлена в `TASKS.md` как новая задача **`P3-T23`**.

---

## 2. Как проверено (команды и итоговые строки `test result`)

- **Точечные контрактные тесты `P3-T14`:**
  ```text
  cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_initialize -- --nocapture
  test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 56 filtered out; finished in 1.11s

  cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract -- --nocapture
  test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 56 filtered out; finished in 1.00s
  ```
- **Полный суит `mcp_contract_tests` (`MCP_CAD_DISABLE_GUI_SPAWN=1`, `--test-threads=1`):**
  ```text
  $env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --test mcp_contract_tests --no-fail-fast -- --test-threads=1
  test result: FAILED. 56 passed; 1 failed; 0 ignored; 0 measured; 0 filtered out; finished in 57.86s
  ```
  Единственное падение — `test_mcp_open_preview_window` (`mcp_contract_tests.rs:716:5`, отслеживается в `P3-T20`: ожидается `"headless"` вместо `"empty"` на пустой сцене).
- **Остальные тесты `mcp-cad-server`:**
  - `cargo test -p mcp-cad-server --lib --no-fail-fast`: `test result: ok. 59 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 23.13s`
  - `cargo test -p mcp-cad-server --bin mcp-cad-server --no-fail-fast`: `test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s`
  - `cargo test -p mcp-cad-server --test bench_suite --no-fail-fast`: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 4.98s`
  - `cargo test -p mcp-cad-server --test gear_kinematic_test --no-fail-fast`: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.71s`
  - `$env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --test qa_wide_scan --no-fail-fast -- --test-threads=1`: `test result: ok. 13 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 18.78s`
  - Итого по `mcp-cad-server`: **131 тест: 130 проходят, 1 падает (`P3-T20`), 0 `ignored`**.
- **Clippy по всему workspace (`rustc 1.97.1 (8bab26f4f 2026-07-14)` / `clippy 0.1.97`):**
  ```text
  cargo clippy --workspace --all-targets -- -D warnings
  Finished `dev` profile [optimized + debuginfo] target(s) in 37.71s
  EXIT: 0 (0 ошибок, 0 предупреждений)
  ```

---

## 3. Тесты до и после

| Проверка | До `P3-T14` (в сводке `P3-T13`) | После проверки `P3-T14` (правки только в docs) |
|---|---|---|
| `test_mcp_initialize` | FAILED (устаревшая сводка до `281a3ed`) | **PASS (1/1)** |
| `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` | FAILED (устаревшая сводка до `281a3ed`) | **PASS (1/1)** |
| `mcp-cad-server --test mcp_contract_tests` (`--test-threads=1`, `MCP_CAD_DISABLE_GUI_SPAWN=1`) | 54/57 | **56/57 PASS** (1 падение: `P3-T20`) |
| `mcp-cad-server` (все таргеты) | 128/131 | **130/131 PASS** (1 падение: `P3-T20`) |
| `cargo clippy --workspace --all-targets -- -D warnings` | 0 ошибок, 0 предупреждений | **0 ошибок, 0 предупреждений** (`EXIT: 0`) |

---

## 4. Что не удалось и остаточные риски

- Остальные крейты workspace (`mcp-cad-core`, `mcp-cad-fea`, `mcp-cad-physics`, `mcp-cad-viewer`) и TypeScript-суит в рамках `P3-T14` повторно не прогонялись, так как Rust-код не изменялся.
- Единственное оставшееся падение в `mcp-cad-server` (`test_mcp_open_preview_window`) относится к отдельной задаче `P3-T20` и в рамках `P3-T14` не трогалось.

---

## 5. Побочные находки (не чинить, только перечислить — занесены в `TASKS.md`)

1. **`P3-T23`**: При запуске `cargo test -p mcp-cad-server --test mcp_contract_tests` и `cargo test -p mcp-cad-server --test qa_wide_scan` в многопоточном режиме по умолчанию на Windows (без `-- --test-threads=1` и `MCP_CAD_DISABLE_GUI_SPAWN=1`) тест-раннер зависает после выполнения первых тестов (конкурентная инициализация WGPU / `OffscreenRenderer` между потоками). Последовательный запуск (`-- --test-threads=1` с `MCP_CAD_DISABLE_GUI_SPAWN=1`) завершается стабильно (`57.86s` и `18.78s`). Дополнительно, `qa_wide_scan` при прогоне перезаписывает отслеживаемые в Git файлы `qa/results/*.json` и создаёт неотслеживаемый каталог `qa/artifacts/`.
