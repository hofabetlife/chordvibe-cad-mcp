# Отчёт по задаче P5-T29: Учёт `params.material` при расчёте плотности `density` в `import_stl` (`src/cad/mcp_dispatcher.ts`)

**Статус:** ✅ Выполнено  
**Дата:** 24.11.2026  

---

## 1. Описание проблемы
При аудите [`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts) в ходе `P5-T28` было выявлено, что в обработчике `case 'import_stl':` (строки `1988–1996`) плотность импортируемой заготовки вычислялась зашитым вызовом `const density = getDensity('steel_1045');`, хотя ниже в `importedPart.materialId` записывалось `String(params.material || 'steel_1045')`. В результате при передаче любого материала, отличного от стали AISI 1045 (например, `"alu_6061"`, `"pyrex"`, `"aluminum_6061_t6"` или пользовательского материала из `define_material`), масса детали `mesh.mass` и тензор инерции `mesh.inertiaTensor` рассчитывались по плотности стали (`7850 кг/м³`) вместо плотности назначенного материала.

> **Примечание о выборе задачи в среде выполнения:** задачи `P5-T21`–`P5-T26` относятся к Rust-крейтам (`mcp-cad-fea`, `mcp-cad-server`) и требуют компиляции/проверки через `cargo test`, тогда как в данном веб-контейнере отсутствует Rust-тулчейн (`cargo`/`rustc`). `P5-T29` — первая открытая задача в `TASKS.md` (зависимость `P5-T28` выполнена), которую можно полностью реализовать и верифицировать в текущем окружении.

---

## 2. Реализованное решение и верификация
1. **[`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts) (`case 'import_stl'`, строки `1988–1997`)**:
   - Извлечена переменная `const matId = String(params.material || 'steel_1045');` до вызова `getDensity(...)`.
   - Вызов `getDensity('steel_1045')` заменён на `getDensity(matId)`, а в `importedPart.materialId` передаётся тот же `matId`, что гарантирует согласованность плотности (с разрешением алиасов, канонических Rust-ID и пользовательских материалов из `state.materials`), массы `mesh.mass`, тензора инерции `mesh.inertiaTensor` и `materialId`.
2. **Unit-тестирование ([`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts), строки `2782–2889`)**:
   - Добавлен unit-тест `verifies P5-T29: import_stl computes mesh mass and inertia from params.material density for presets, aliases, and custom materials`, проверяющий:
     - материал по умолчанию (`steel_1045`, `7850 кг/м³`);
     - прямой пресет `alu_6061` (`2700 кг/м³`);
     - алиас `pyrex` → `borosilicate_glass` (`2500 кг/м³`);
     - канонический Rust-ID `aluminum_6061_t6` → `alu_6061` (`2700 кг/м³`);
     - пользовательский материал `custom_cfrp_1500` (`1500 кг/м³`), зарегистрированный через `define_material`;
     - пропорциональное масштабирование `mesh.inertiaTensor` и согласованность `mass_kg` в ответе `list_parts`.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T33 [S]`**: При аудите `import_stl` в [`src/App.tsx`](../../src/App.tsx) (`2651–2679`, `2924–2929`) обнаружены два дефекта асинхронной гидратации STL в `handleLoadNativeStlUrl`:
  1. `loadCachedRustStlMesh(stlUrl, 7850)` всегда передаёт зашитую плотность стали `7850 кг/м³`, даже когда на строке `2662` детали с `'rim'` в имени назначается `materialId: 'alu_6061'` (`2700 кг/м³`);
  2. При вызове MCP-инструмента `import_stl` из веб-студии (`App.tsx:2924-2929`) вызывается `handleLoadNativeStlUrl(response.native_stl_url, response.file_path || 'Imported STL')`, который перезаписывает всю сцену `setParts([newPart])` и сбрасывает `part_id`, `materialId`, `isFixed` и плотность импортированной детали на `'steel_1045'` / `7850`.

---

## 4. Затронутые и проверенные файлы
- `src/cad/mcp_dispatcher.ts`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T29-report.md`

---

## 5. Результаты проверок и тестов
1. **Тест `P5-T29` (`npx --yes tsx --test --test-name-pattern="P5-T29" src/cad/cad_engine.test.ts`)**:
   - **1/1 PASS** (`0 failed`).
2. **Полный прогон TypeScript-тестов (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts`)**:
   - **78/79 PASS, 1 pre-existing fail** (единственное падение — предсуществующий тест `P2-T15` на строке `1692`, отслеживаемый в `P5-T30`).
3. **`npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`)**:
   - **PASS** (`0` ошибок).
4. **Примечание по окружению**: `cargo` и `.git` в текущем контейнере отсутствуют (Rust-код в задаче `P5-T29` не изменялся; коммит в git невозможен без каталога `.git`).
