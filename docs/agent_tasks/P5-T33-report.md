# Отчёт по задаче P5-T33: Учёт плотности материала и сохранение параметров импортированной детали/сцены в `handleLoadNativeStlUrl`

**Статус:** ✅ Выполнено  
**Дата:** 24.11.2026  

---

## 1. Описание проблемы
При аудите `import_stl` в `P5-T29` были выявлены два дефекта в `src/App.tsx`:
1. В `handleLoadNativeStlUrl` (`src/App.tsx:2651-2662`) всегда вызывался `loadCachedRustStlMesh(stlUrl, 7850)` с зашитой плотностью стали (`7850 кг/м³`), даже когда для дисков (`partId.includes('rim')`) назначался `materialId: 'alu_6061'` (`2700 кг/м³`) или в `NATIVE_STL_PRESET_MAP` указывались `alu_6061`, `ti_6al_4v`, `borosilicate_glass`, из-за чего масса и тензор инерции обода завышались почти в `2.9×`.
2. В `handleExecuteMcpTool` (`src/App.tsx:2924-2929`) после выполнения MCP-инструмента `import_stl` асинхронно вызывался `handleLoadNativeStlUrl(response.native_stl_url, response.file_path || 'Imported STL')`, который выполнял `setParts([newPart])` (удаляя все остальные детали и шарниры сцены) и перезаписывал `part_id`, `name`, `materialId`, `isFixed` и плотность импортированной детали на `'steel_1045'` / `7850`.

> **Примечание о выборе задачи в среде выполнения:** задачи `P5-T21`–`P5-T26` относятся к Rust-крейтам (`mcp-cad-fea`, `mcp-cad-server`) и требуют компиляции/проверки через `cargo test`, тогда как в данном веб-контейнере отсутствует Rust-тулчейн (`cargo`/`rustc`). `P5-T33` — первая открытая задача в `TASKS.md` (зависимость `P5-T29` выполнена), которую можно полностью выполнить и верифицировать в текущем окружении.

---

## 2. Реализованное решение и верификация
1. **[`src/cad/native_bridge.ts`](../../src/cad/native_bridge.ts)** (добавлены чистые тестируемые помощники `NativeStlLoadOptions`, `resolveNativeStlMaterialSpec`, `rescaleMeshForDensity`, `buildStandaloneNativeStlPart`, `hydrateImportedStlPartInScene`):
   - `resolveNativeStlMaterialSpec(stlUrl, materials, explicitMaterialId?)` извлекает `partId`, выбирает `explicitMaterialId` (если передан) или `'alu_6061'` для `rim` / `'steel_1045'` по умолчанию и вычисляет фактическую плотность через `getMaterialById(materialId, materials).density` (с поддержкой пресетов, канонических Rust-ID, алиасов и пользовательских материалов).
   - `rescaleMeshForDensity(mesh, targetDensity)` при необходимости пересчитывает `mass` и `inertiaTensor` без мутации кэшированного объекта `MeshGeometryData`.
   - `hydrateImportedStlPartInScene(prevParts, targetPartId, stlUrl, parsedMesh, materials?)` обновляет меш и узел `stl_import` в DAG только у детали `p.id === targetPartId`, сохраняя без изменений все остальные детали сцены, а также `id`, `name`, `materialId`, `isFixed`, `visible`, `position`/`rotation`/`scale`, `nativeStlUrl` и прочие узлы DAG (плюс защита от гонки при смене материала или удалении детали во время асинхронной загрузки STL).
2. **[`src/App.tsx`](../../src/App.tsx)**:
   - В `handleLoadNativeStlUrl` добавлен опциональный параметр `options?: NativeStlLoadOptions`; плотность вычисляется через `resolveNativeStlMaterialSpec` и `getMaterialById` вместо зашитого `7850`.
   - При наличии `options?.hydratePartId` выполняется точечная гидратация через `hydrateImportedStlPartInScene` в `setParts((prev) => ...)` без перезаписи сцены (`setParts([newPart])`) и шарниров (`setJoints([...])`).
   - В `loadPresetScene` (`NATIVE_STL_PRESET_MAP` и `GOLDEN_WINDOWS_MAPPINGS`) пробрасывается `materialId` соответствующего пресета (`alu_6061`, `ti_6al_4v`, `borosilicate_glass`, `steel_4140`).
   - В `handleExecuteMcpTool` для `toolName === 'import_stl'` передаются `{ hydratePartId: String(response.part_id), materialId: importedPart?.materialId || String(params.material || 'steel_1045'), materials: nextState.materials }`.
3. **[`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts)**:
   - Добавлен unit-тест `verifies P5-T33: handleLoadNativeStlUrl resolves material density via getMaterialById and hydrates MCP import_stl without wiping scene or part metadata`, проверяющий плотность/массу/инерцию `car_rim.stl` (`2700 кг/м³`), алиасы и Rust-ID (`pyrex`, `aluminum_6061_t6`), сквозную гидратацию MCP `import_stl` (сохранение остальных деталей сцены, `part_id`, `name`, `materialId`, `isFixed`, плотности), защиту от гонки смены материала и удаления детали, а также контракт на отсутствие `loadCachedRustStlMesh(stlUrl, 7850)` в `src/App.tsx`.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T41 [S]`**: В [`src/cad/primitives.ts`](../../src/cad/primitives.ts) (`evaluateFeatureTree`, строки `3474–3476`) при `enabledNodes[0].type === 'stl_import' && existingMesh` функция немедленно возвращает `existingMesh` без пересчёта `mass` и `inertiaTensor` под новую `density` (из-за чего назначение нового материала импортированной STL-детали в `src/App.tsx:4007-4025` не обновляет её массу/инерцию, а последующие операции `cut_hole`/`transform` в DAG игнорируются).
- **`P5-T42 [S]`**: В [`src/components/CadLeftSidebar.tsx`](../../src/components/CadLeftSidebar.tsx) (`305`) инспектор параметров узла `stl_import` выводит внутренний бинарный буфер `node.params.mesh` как обрезанный сырой JSON (`{"vertices":{"0"...`), а в [`src/components/CadRightSidebar.tsx`](../../src/components/CadRightSidebar.tsx) (`1269`) указан устаревший счётчик `(41 Rust)` вместо `(43 Rust)` MCP-инструментов.

---

## 4. Затронутые и проверенные файлы
- `src/cad/native_bridge.ts`
- `src/App.tsx`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T33-report.md`

---

## 5. Результаты проверок и тестов
1. **Целевой тест `P5-T33` (`npx --yes tsx --test --test-name-pattern="P5-T33" src/cad/cad_engine.test.ts`)**:
   - **1/1 PASS** (`0 failed`).
2. **Полный прогон всех 11 TypeScript-сьютов (`cad_engine`, `analyze_mechanism`, `destruction_engine`, `ortho_views`, `field_colormaps`, `coupled_sim_stream`, `helix_primitive`, `live_sync_adapter`, `press_simulation`, `rust_scene_sync`, `live_sync_store`)**:
   - **113/113 PASS** (`0 failed`: `83/83` в 5 базовых + `30/30` в 6 расширенных сьютах).
3. **`npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`)**:
   - **PASS** (`0` ошибок).
4. **Живая браузерная верификация (`Playwright Chromium 1440×900`, `http://localhost:3000?view=studio`)**:
   - **PASS** (`0` ошибок, `0` предупреждений в консоли): выполнен сквозной вызов MCP `import_stl` (`part_id: "pyrex_housing_01"`, `name: "Pyrex Cube"`, `material: "pyrex"`, `is_fixed: true`, бинарный STL `0.1×0.1×0.1 м` = `1000 см³` / `12Δ`); подтверждено сохранение стартового `V8 Racing Piston` (`ДЕТАЛИ СЦЕНЫ (2)`), отображение `Borosilicate Glass E27` и точной массы `2500.00 г` (вместо ошибочных `7850.00 г`).
5. **Примечание по окружению**: `cargo` и `.git` в текущем контейнере отсутствуют (Rust-код в задаче `P5-T33` не изменялся; git-коммит невозможен без каталога `.git`).
