# Отчёт по задаче P3-T22: Восстановление `rust-toolchain.toml`, `.github/workflows/ci.yml` и актуализация `src/cad/cad_engine.test.ts`

**Дата выполнения:** 15.11.2026  
**Статус:** ✅ DONE  

---

## 1. Цель и анализ проблемы

При выполнении аудита **P3-T13** было выявлено, что в коммите `281a3ed` (`feat: apply latest ChordVibe CAD MCP snapshot (from f9442e4)`) из рабочей копии исчезли файлы `rust-toolchain.toml`, `.github/workflows/ci.yml` и `src/cad/cad_engine.test.ts`, ранее созданные/использованные в `P0-T8` (коммит `6b332b9`). При этом `README.md`, `PROJECT_CONTEXT.md` и `TASKS.md` продолжали ссылаться на эти файлы, а `package.json` не содержал скрипта `"test"`.

Проверка текущего слоя `src/cad/` показала:
- Все модули, импортируемые `src/cad/cad_engine.test.ts` (`primitives.ts`, `fea_solver.ts`, `kinematics.ts`, `materials.ts`, `mcp_dispatcher.ts`, `query_optimizer.ts`, `native_bridge.ts`, `workbenches.ts`), присутствуют и активно развиваются (включая `P3-T15`, `P3-T16`, `P3-T19`).
- При пробном запуске исторического `src/cad/cad_engine.test.ts` (из `6b332b9`) 34 из 37 тестов прошли сразу, а 3 теста требовали актуализации под изменения `P2-T38` и `P3-T9`/`P3-T10`:
  1. `BUG-015` и `BUG-021`: в `MCP_TOOLS_CATALOG` (`src/cad/mcp_dispatcher.ts`) теперь `31` инструмент (к `28` базовым добавлены `analyze_mechanism`, `check_interference`, `sweep_joint`).
  2. `BUG-018`: после `P2-T38` функция `validatePartMesh` по умолчанию (`strictSlivers = false`) переносит нарушения sliver-порога в `warnings` при `status === 'PASS'`, а строгий отказ `'FAIL'` срабатывает при `strictSlivers = true`.

---

## 2. Выполненные изменения

1. **[`rust-toolchain.toml`](../../rust-toolchain.toml)** *(восстановлен)*:
   - Зафиксирован канал `channel = "1.91"` и компоненты `["rustfmt", "clippy"]`, согласованные с `Dockerfile` (`rust:1.91-bookworm`), `Cargo.lock` (`rustc >= 1.89`) и локально установленным тулчейном `rustc 1.91.1 (ed61e7d7e 2025-11-07)`.
2. **[`.github/workflows/ci.yml`](../../.github/workflows/ci.yml)** *(восстановлен и актуализирован)*:
   - Добавлена глобальная переменная `MCP_CAD_DISABLE_GUI_SPAWN: "1"` для headless-тестов `open_preview_window` (`P2-T37`/`P3-T20`).
   - В job `rust` (`Rust Build & Test (1.91)`) прогон `mcp-cad-server` вынесен в отдельный шаг с `-- --test-threads=1` (учёт `P3-T23`), остальные крейты запускаются через `cargo test --workspace --exclude mcp-cad-server --no-fail-fast`.
   - Job `clippy` запускает `cargo clippy --workspace --all-targets -- -D warnings` как блокирующий гейт.
   - В job `web` (`Web Studio Lint, Build & Tests`) убран `continue-on-error: true`, добавлены `npm run build` и `npm test` (запускающий оба TS-сьюта: `cad_engine.test.ts` и `analyze_mechanism.test.ts`), а установка зависимостей поддерживает отсутствие `package-lock.json` в Git (`npm ci` при наличии файла, иначе `npm install --no-audit --no-fund`).
3. **[`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts)** *(восстановлен и актуализирован)*:
   - В тесте `BUG-015` проверка каталога обновлена до `31` инструмента (включая `validate_mesh`, `analyze_mechanism`, `check_interference`, `sweep_joint`).
   - В тесте `BUG-018` проверяются как режим по умолчанию (`strictSlivers = false` -> `'PASS'` и непустой `warnings`), так и строгий гейт (`strictSlivers = true` -> `'FAIL'`).
   - В тесте `BUG-021` актуализирована проверка паритета для `31` TS-инструмента (`docs/MCP_API_REFERENCE.md` и `PROJECT_CONTEXT.md`), с сохранением проверки `28` базовых инструментов в `README.md` и `docs/CLAUDE_DESKTOP_SETUP.md`.
4. **[`package.json`](../../package.json)**:
   - Добавлен скрипт `"test": "npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts"`.
5. **Документация ([`README.md`](../../README.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md), [`TASKS.md`](../../TASKS.md))**:
   - В `README.md` обновлены бейдж статуса тестов, раздел «Тестирование и верификация» (команды `npm test`, `npm run lint`, `npm run build`, раздельный запуск `mcp-cad-server` с `MCP_CAD_DISABLE_GUI_SPAWN=1 -- --test-threads=1`, актуальный статус `clippy`) и дерево репозитория.
   - В `PROJECT_CONTEXT.md` и `TASKS.md` обновлены дерево файлов и статус `P3-T22` -> `✅ DONE`, а выявленные попутно расхождения (устаревшие упоминания `28` инструментов вместо `40` в `README.md` / `docs/{CLAUDE_DESKTOP_SETUP,ANTIGRAVITY_SETUP,ARCHITECTURE}.md` / `src/cad/{native_bridge,workbenches}.ts` и отсутствие `package-lock.json` в Git) заведены как новая задача **`P3-T24`**.

---

## 3. Результаты верификации (после финальных правок)

| Проверка / Команда | Итог | Детали |
|---|---|---|
| `rustc --version` / `cargo --version` | **PASS** | `rustc 1.91.1 (ed61e7d7e 2025-11-07)`, `cargo 1.91.1 (ea2d97820 2025-10-10)` (подхвачено из `rust-toolchain.toml`) |
| `cargo clippy --workspace --all-targets -- -D warnings` | **PASS** (`exit 0`) | `0` ошибок, `0` предупреждений во всех 5 крейтах workspace (`0.47s`) |
| `npm run lint` (`tsc --noEmit`) | **PASS** (`exit 0`) | `0` ошибок TypeScript |
| `npm run build` (`vite build`) | **PASS** (`exit 0`) | Сборка фронтенда без ошибок (`✓ built in 3.72s`) |
| `npm test` (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts`) | **52 passed, 0 failed** (`exit 0`) | `37/37` в `cad_engine.test.ts` + `15/15` в `analyze_mechanism.test.ts` (`3` сьюта, `35726.38ms`) |
| `python -c "import yaml; yaml.safe_load(open('.github/workflows/ci.yml'))"` | **PASS** (`exit 0`) | `YAML valid! Jobs: ['rust', 'clippy', 'web']` |
