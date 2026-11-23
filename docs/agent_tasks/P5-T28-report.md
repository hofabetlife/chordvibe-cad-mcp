# Отчёт по задаче P5-T28: Разрешение алиасов материалов через `getMaterialById` в `CadViewport3D.tsx` и `src/cad/mcp_dispatcher.ts`

**Статус:** ✅ Выполнено  
**Дата:** 22.11.2026  

---

## 1. Описание проблемы
При аудите и верификации `P5-T20` было выявлено, что в трёх местах TypeScript-фронтенда и диспетчера MCP материал детали всё ещё искался по точному строковому совпадению ID без разрешения алиасов и канонических Rust-ID:
1. **[`src/components/CadViewport3D.tsx`](../../src/components/CadViewport3D.tsx) (`1106`)**: при предварительном создании мешей активных FEM-поверхностей (`P5-T15`) цвет и PBR-свойства определялись через `materials.find((m) => m.id === partObj?.materialId)`, из-за чего детали с алиасами (`al_6061`, `aluminum_6061_t6`, `steel_carbon_aisi1045`, `pyrex` и др.) получали `undefined` и сбрасывались на цвет по умолчанию `#94a3b8`.
2. **[`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts) (`1785` в `run_fea_analysis` и `1902` в `get_fea_heatmap_image`)**: материал искался через `state.materials.find((m) => m.id === part.materialId) || state.materials[0]`, из-за чего расчёт МКЭ для детали с алиасом или каноническим Rust-ID (`pyrex`, `aluminum_6061_t6`, `steel_1020`) молча использовал параметры `steel_1045` ($E = 205\text{ ГПа}$, $\sigma_y = 350\text{ МПа}$).

---

## 2. Реализованное решение и верификация
1. **[`src/components/CadViewport3D.tsx`](../../src/components/CadViewport3D.tsx) (`1106–1108`)**:
   - Заменён прямой поиск `materials.find((m) => m.id === partObj?.materialId)` на `partObj ? getMaterialById(partObj.materialId, materials) : undefined`.
2. **[`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts) (`1784`, `1900`)**:
   - В `run_fea_analysis` и `get_fea_heatmap_image` прямой поиск `state.materials.find(...) || state.materials[0]` заменён на `getMaterialById(part.materialId, state.materials)`, что обеспечивает разрешение как всех алиасов и канонических Rust-ID из `MATERIALS_LIBRARY`, так и пользовательских материалов из `define_material` (`state.materials`).
3. **Unit-тестирование ([`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts), `2518–2633`)**:
   - Добавлен тест `verifies P5-T28: run_fea_analysis, get_fea_heatmap_image, and CadViewport3D FEM surface meshes resolve material aliases via getMaterialById`, проверяющий:
     - реальное масштабирование перемещений и `yieldStrengthMPa` / `safetyFactor` в `run_fea_analysis` и `get_fea_heatmap_image` для `pyrex` ($\sigma_y = 120\text{ МПа}$, $E = 64\text{ ГПа}$), `aluminum_6061_t6` ($\sigma_y = 276\text{ МПа}$, $E = 68.9\text{ ГПа}$) и пользовательского `define_material` (`1.0 ГПа`, `25 МПа`);
     - использование `getMaterialById(partObj.materialId, materials)` и отсутствие сырого `materials.find` в `CadViewport3D.tsx` + точные PBR-цвета `pyrex` (`#dbeafe`) и `aluminum_6061_t6` (`#c4cdd6`).
4. **Живая браузерная верификация (`Playwright Chromium 1440×900`, `http://localhost:3000`)**:
   - Проверен запуск сцены и запекание кэша разрушения пары шестерён (`Steel 4140` + `Brass C360`): корректный 3D-рендер, `0` консольных ошибок и `0` предупреждений.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T29 [S]`**: В `src/cad/mcp_dispatcher.ts:1990` обработчик `import_stl` вычисляет плотность через `getDensity('steel_1045')`, хотя на строке `1995` записывает `materialId: String(params.material || 'steel_1045')`. При передаче `material` в `import_stl` (например, `"alu_6061"` или `"pyrex"`) масса сетки рассчитывается по плотности стали (`7850 кг/м³`) вместо назначенного материала.

---

## 4. Затронутые и проверенные файлы
- `src/components/CadViewport3D.tsx`
- `src/cad/mcp_dispatcher.ts`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T28-report.md`

---

## 5. Результаты проверок и тестов
1. **TypeScript Unit Tests (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts`)**:
   - **76 passed, 0 failed** (включая новый тест `P5-T28`).
2. **`npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`)**:
   - **PASS** (`0` ошибок).
3. **Playwright Chromium (`1440×900`, `http://localhost:3000`)**:
   - **PASS** (`0` console errors, `0` console warnings).
