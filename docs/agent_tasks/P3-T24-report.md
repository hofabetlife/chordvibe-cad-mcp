# Отчёт по задаче P3-T24: Синхронизация счётчиков инструментов (`40` Rust / `31` TS), фиксация `package-lock.json` и `npm ci` в CI

**Дата выполнения:** 15.11.2026  
**Статус:** ✅ DONE  

---

## 1. Цель и анализ проблемы

При выполнении **P3-T22** было выявлено расхождение между фактическим набором инструментов и устаревшими счётчиками `28 Tools` / `28 инструментов` в документации и UI/TS-модулях:
- Фактический Rust MCP-сервер (`crates/mcp-cad-server/src/tools/mod.rs`) экспортирует **40 инструментов** (`28` базовых + `12` добавленных в фазах P2–P3: `set_units`, `save_project`, `load_project`, `set_requirements`, `check_requirements`, `get_scene_summary`, `get_part_load_cases`, `run_fea_from_simulation`, `check_gear_strength`, `analyze_mechanism`, `check_interference`, `sweep_joint`).
- TypeScript-каталог `MCP_TOOLS_CATALOG` (`src/cad/mcp_dispatcher.ts`) содержит **31 инструмент** (`28` базовых + `3` инструмента кинематики и интерференции `P3-T9`/`P3-T10`: `analyze_mechanism`, `check_interference`, `sweep_joint`).
- При этом `README.md`, `docs/CLAUDE_DESKTOP_SETUP.md`, `docs/ANTIGRAVITY_SETUP.md`, `docs/ARCHITECTURE.md`, `src/cad/native_bridge.ts`, `src/cad/workbenches.ts`, а также `src/App.tsx`, `src/components/CadRightSidebar.tsx`, `run_preview.bat`, `run_preview.ps1` и оглавление `docs/MCP_API_REFERENCE.md` содержали устаревшие ссылки на `28` инструментов (или пропускали №39–40 в оглавлении).
- Кроме того, в Git отсутствовал `package-lock.json`, а untracked-версия в рабочей копии содержала расхождение с `package.json` (`@types/node: ^26.6.6` вместо `^22.10.2`), из-за чего `npm ci --dry-run` падал с `EUSAGE`.

---

## 2. Выполненные изменения

1. **Документация ([`README.md`](../../README.md), [`docs/CLAUDE_DESKTOP_SETUP.md`](../../docs/CLAUDE_DESKTOP_SETUP.md), [`docs/ANTIGRAVITY_SETUP.md`](../../docs/ANTIGRAVITY_SETUP.md), [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md), [`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md))**:
   - В `README.md` все упоминания количества инструментов обновлены с `28` на `40`, а сводная таблица инструментов расширена строками **29–40** (`set_units` .. `sweep_joint`) с точными параметрами, выходными полями и описанием.
   - В `docs/CLAUDE_DESKTOP_SETUP.md` обновлено число инструментов (`40 инженерных инструментов`) и дополнены категории списка всех 40 инструментов.
   - В `docs/ANTIGRAVITY_SETUP.md` и `docs/ARCHITECTURE.md` обновлены счётчики (`40 инженерных инструментов`, `40 инструментов MCP`, `40 строгих JSON-Schema определений инструментов`).
   - В оглавлении `docs/MCP_API_REFERENCE.md` добавлены недостающие ссылки на разделы `39. check_interference` и `40. sweep_joint`.
2. **TypeScript-слой и UI ([`src/cad/native_bridge.ts`](../../src/cad/native_bridge.ts), [`src/cad/workbenches.ts`](../../src/cad/workbenches.ts), [`src/App.tsx`](../../src/App.tsx), [`src/components/CadRightSidebar.tsx`](../../src/components/CadRightSidebar.tsx), [`run_preview.bat`](../../run_preview.bat), [`run_preview.ps1`](../../run_preview.ps1))**:
   - В `src/cad/native_bridge.ts` введены константы `RUST_MCP_TOOLS_COUNT = 40` и `TS_MCP_TOOLS_COUNT = MCP_TOOLS_CATALOG.length` (`31`): в автономном режиме `unified_hybrid_kernel` (`DEFAULT_BRIDGE_STATUS` и fallback `probeWindowsMcpServer`) используется `TS_MCP_TOOLS_COUNT` (`31`), а при подключении к нативному Rust-серверу (`windows_exe_connected`) — `RUST_MCP_TOOLS_COUNT` (`40`).
   - В `src/App.tsx` статусбар `CadStatusBar` переключён с захардкоженного `28` на `bridgeStatus.activeToolsCount`.
   - В `src/components/CadRightSidebar.tsx` счётчики заменены на динамический `MCP_TOOLS_CATALOG.length` (`31`) с явным указанием `(40 Rust)` в сводке эталонов.
   - В `src/cad/workbenches.ts`, `run_preview.bat` и `run_preview.ps1` устранены устаревшие упоминания `28`.
3. **Контрактные тесты ([`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts))**:
   - В тесте `BUG-021` проверка расширена с `28` на все **40 инструментов Rust MCP** в `README.md`, `docs/CLAUDE_DESKTOP_SETUP.md`, `docs/MCP_API_REFERENCE.md` и `PROJECT_CONTEXT.md`, добавлены проверки `40` инструментов в `docs/ANTIGRAVITY_SETUP.md` и `docs/ARCHITECTURE.md`, а также проверки `TS_MCP_TOOLS_COUNT === 31`, `RUST_MCP_TOOLS_COUNT === 40`, `DEFAULT_BRIDGE_STATUS.activeToolsCount === 31` и отсутствия устаревшего `28` в `ENGINEERING_WORKBENCHES`.
4. **Зависимости и CI ([`package-lock.json`](../../package-lock.json), [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml))**:
   - `package-lock.json` перегенерирован и синхронизирован с `package.json` (`@types/node: ^22.10.2`), подтверждена совместимость с `npm ci` (`npm ci --dry-run` — `exit 0`).
   - В `.github/workflows/ci.yml` (job `web`) включены `cache: "npm"` в `actions/setup-node@v4` и детерминированная установка через `npm ci`.

---

## 3. Результаты верификации (после финальных правок)

| Проверка / Команда | Итог | Детали |
|---|---|---|
| `npm ci --dry-run` | **PASS** | `up to date in 457ms`, `exit 0` (`package-lock.json` полностью консистентен с `package.json`) |
| `npm run lint` (`tsc --noEmit`) | **PASS** | `0` ошибок, `exit 0` |
| `npm run build` (`vite build`) | **PASS** | `1592 modules transformed`, `✓ built in 4.12s`, `exit 0` |
| `npm test` (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts`) | **PASS** | **52/52 PASS** (`3` suites, `52` tests, `0` fail, `0` skipped, `35340.59 ms`: `37` в `cad_engine.test.ts` + `15` в `analyze_mechanism.test.ts`) |
| YAML-валидация `.github/workflows/ci.yml` (`yaml.safe_load`) | **PASS** | `Jobs: ['rust', 'clippy', 'web']`, в `web` настроены `cache: 'npm'` и `npm ci` |
| `cargo clippy --workspace --all-targets -- -D warnings` | **PASS** | `0` ошибок, `0` предупреждений во всех 5 крейтах (`0.55s`, `exit 0`) |
