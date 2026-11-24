# Отчёт по задаче P5-T32: Добавление пунктов `42. bake_coupled_simulation` и `43. query_simulation_frame` в оглавление `docs/MCP_API_REFERENCE.md`

**Статус:** ✅ Выполнено  
**Дата:** 24.11.2026  

---

## 1. Описание проблемы
При аудите [`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md) в `P5-T27` было выявлено, что раздел `## Оглавление` (строки `50–61`) обрывался на инструменте `41. apply_recipe`, хотя в теле справочника уже присутствовали разделы `### 42. bake_coupled_simulation` (строка `2284`) и `### 43. query_simulation_frame` (строка `2336`), соответствующие инструментам №42 и №43 в `crates/mcp-cad-server/src/tools/mod.rs`.

> **Примечание о выборе задачи в среде выполнения:** задачи `P5-T21`–`P5-T26` относятся к Rust-крейтам (`mcp-cad-fea`, `mcp-cad-server`) и требуют компиляции/проверки через `cargo test`, тогда как в данном веб-контейнере отсутствует Rust-тулчейн (`cargo`/`rustc`). `P5-T32` — первая открытая задача в `TASKS.md` (зависимости `P5-T6`, `P5-T27` выполнены), которую можно полностью выполнить и верифицировать в текущем окружении.

---

## 2. Реализованное решение и верификация
1. **[`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md) (`## Оглавление`, строки `61–62`)**:
   - В оглавление (после `- [41. \`apply_recipe\`](#41-apply_recipe)`) добавлены рабочие якорные ссылки:
     - `- [42. \`bake_coupled_simulation\`](#42-bake_coupled_simulation)`
     - `- [43. \`query_simulation_frame\`](#43-query_simulation_frame)`
2. **[`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts) (строки `3190–3305`)**:
   - Добавлен автоматический контрактный тест `verifies P5-T32: docs/MCP_API_REFERENCE.md TOC contains working anchor links for all 43 documented MCP tools`, который:
     - извлекает все 43 имени инструментов из `list_tools()` в `crates/mcp-cad-server/src/tools/mod.rs`;
     - разбирает блок `## Оглавление` в `docs/MCP_API_REFERENCE.md` и проверяет наличие ровно 43 последовательно пронумерованных (`1..43`) пунктов, включая `42. bake_coupled_simulation` и `43. query_simulation_frame`;
     - проверяет взаимно однозначное соответствие имён в оглавлении и в реестре Rust;
     - вычисляет GitHub Flavored Markdown slug для всех заголовков `###` в теле документа и подтверждает, что каждый из 43 якорей в оглавлении разрешается в существующий заголовок соответствующего инструмента.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T39 [S]`**: Таблица входных параметров раздела `### 42. bake_coupled_simulation` в [`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md) (строки `2291–2304`) не содержит **10 параметров** из Rust `input_schema` (`crates/mcp-cad-server/src/tools/mod.rs:1823-1934`: `wait_timeout_ms`, `output_fps`, `max_frames`, `mesh_density`, `enable_plasticity`, `enable_fracture`, `enable_contact`, `corotated`, `contact_friction`, `length_unit`), а также устаревшим образом описывает значение по умолчанию `async_mode` как `false (по умолчанию) — синхронное ожидание завершения` вместо фактического ограниченного ожидания `wait_timeout_ms` (`12000 ms`) с возвратом `status='running'`.
- **`P5-T40 [S]`**: В разделе `## Оглавление` [`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md) два групповых якоря не совпадают с фактическими заголовками `##` в теле документа (битые ссылки: строка `16` `#группа-2-параметрическое-dag-моделирование-и-тесселяция` vs заголовок `## Группа 2: Параметрическое DAG-моделирование` на строке `401`; строка `31` `#группа-5-визуализация-экспорт-и-импорт` vs заголовок `## Группа 5: Визуализация, превью, экспорт и импорт` на строке `1004`), а также отсутствует подпункт оглавления для `### Контракт единиц измерения ([DECISION-UNITS-1])` (строка `122`).

---

## 4. Затронутые и проверенные файлы
- `docs/MCP_API_REFERENCE.md`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T32-report.md`

---

## 5. Результаты проверок и тестов
1. **Целевой тест `P5-T32` (`npx --yes tsx --test --test-name-pattern="P5-T32" src/cad/cad_engine.test.ts`)**:
   - **1/1 PASS** (`0 failed`).
2. **Полный прогон всех 11 TypeScript-сьютов (`cad_engine`, `analyze_mechanism`, `destruction_engine`, `ortho_views`, `field_colormaps`, `coupled_sim_stream`, `helix_primitive`, `live_sync_adapter`, `press_simulation`, `rust_scene_sync`, `live_sync_store`)**:
   - **112/112 PASS** (`0 failed`: `82/82` в 5 базовых + `30/30` в 6 расширенных сьютах).
3. **`npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`)**:
   - **PASS** (`0` ошибок).
4. **Примечание по окружению**: `cargo` и `.git` в текущем контейнере отсутствуют (Rust-код в задаче `P5-T32` не изменялся; git-коммит невозможен без каталога `.git`).
