# Отчёт по задаче P5-T35: Устранение эвристики `> 1.0 ? 1e-3 : 1.0` для призматических `joint.limits` в `src/cad/kinematics.ts` и `src/cad/press_simulation.ts`

**Статус:** ✅ Выполнено  
**Дата:** 24.11.2026  

---

## 1. Описание проблемы
При аудите `stepKinematicsAssembly` в `P5-T30` было выявлено, что внутреннее поле `KinematicJoint.limits` для призматических шарниров во всём TypeScript-контуре (`src/cad/mcp_dispatcher.ts:1475`, `src/cad/live_sync_adapter.ts:297`, `src/components/CadLeftSidebar.tsx:883`, `analyzeMechanism` в `src/cad/kinematics.ts:469`) хранится в **метрах СИ**. Однако в двух местах оставалась устаревшая эвристика повторного пересчёта из миллиметров в метры при численном значении `> 1.0`:
1. `src/cad/kinematics.ts:91` (`stepKinematicsAssembly`): `const scale = Math.max(Math.abs(rawMin), Math.abs(rawMax)) > 1.0 ? 1e-3 : 1.0;`
2. `src/cad/press_simulation.ts:446` (`detectPressAssembly`): `const limitM = jointSpan > 1.0 ? jointSpan * 1e-3 : jointSpan;`

Из-за этого любой призматический шарнир с ходом более `1.0 м` (например, `limits: [-1500, 1500]` мм → внутренние `[-1.5, 1.5]` м, или `limits: [0, -1.25]` м при `length_unit: "m"`) ошибочно делился ещё на `1000` и зажимался в `[-0.0015, 0.0015]` м (`1.5 мм`) вместо `1.5 м`.

> **Примечание о выборе задачи в среде выполнения:** задачи `P5-T21`–`P5-T26` относятся к Rust-крейтам (`mcp-cad-fea`, `mcp-cad-server`) и требуют компиляции/проверки через `cargo test`, тогда как в данном веб-контейнере отсутствует Rust-тулчейн (`cargo`/`rustc`). `P5-T35` — первая открытая задача в `TASKS.md` (зависимость `P5-T30` выполнена), которую можно полностью выполнить и верифицировать в текущем окружении.

---

## 2. Реализованное решение и верификация
1. **[`src/cad/kinematics.ts`](../../src/cad/kinematics.ts) (`stepKinematicsAssembly`, строки `88–97`)**:
   - Удалена эвристика `const scale = Math.max(Math.abs(rawMin), Math.abs(rawMax)) > 1.0 ? 1e-3 : 1.0;`. Пределы `minL = Math.min(joint.limits[0], joint.limits[1])` и `maxL = Math.max(joint.limits[0], joint.limits[1])` используются напрямую в метрах СИ.
2. **[`src/cad/press_simulation.ts`](../../src/cad/press_simulation.ts) (`detectPressAssembly`, строки `441–450`)**:
   - Удалена эвристика `jointSpan > 1.0 ? jointSpan * 1e-3 : jointSpan`; `limitM = jointSpan` трактуется строго в метрах СИ.
3. **[`src/types/cad.ts`](../../src/types/cad.ts) (`KinematicJoint.limits`, строка `204`)**:
   - Добавлен JSDoc-комментарий, фиксирующий инвариант единиц СИ для `KinematicJoint.limits` (радианы для `revolute`, метры для `prismatic`).
4. **[`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts)**:
   - Добавлен unit-тест `verifies P5-T35: prismatic joint.limits > 1.0 m are honored in SI meters without spurious 1e-3 division in stepKinematicsAssembly and press_simulation`, проверяющий:
     - создание длинноходового портального шарнира `create_joint` с `limits: [-1500, 1500]` (по умолчанию `mm` → внутренние `[-1.5, 1.5]` м), прямой ход при `+2.0 м/с` до упора `+1.5 м` в `run_physics_simulation` и обратный ход при `-4.0 м/с` до упора `-1.5 м` в `stepKinematicsAssembly`;
     - расчёт `maxStrokeM = 1.25 м` для крупногабаритного пресса с `limits: [0, -1.25]` м (`> 1.0 м`) в `detectPressAssembly` без деления на `1000`.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T45 [S]`**: В `src/cad/rust_scene_sync.ts:678-680` (`buildRustSceneSyncBatch`) при формировании MCP-запроса `create_joint` передаётся `createArgs.limits = joint.limits` с `length_unit: 'm'`, но без `limits_unit: 'rad'` для вращательных шарниров (`joint.type === 'revolute'`). Поскольку `KinematicJoint.limits` для `revolute` хранится в радианах, а Rust `physics.rs:232` по умолчанию считает `limits_unit = "deg"` и вызывает `.to_radians()`, при синхронизации сцены в Rust угловые пределы вращательных шарниров уменьшаются в $180/\pi \approx 57.3$ раза.
- **`P5-T46 [S]`**: В `src/cad/kinematics.ts:41-68` (`stepKinematicsAssembly`) при шагании моторизованного вращательного шарнира (`joint.type === 'revolute'`) приращение угла `dTheta` и `joint.currentAngleRad` не ограничиваются пределами `joint.limits` (в отличие от ветки `joint.type === 'prismatic'` на строках `88–97`), из-за чего вращательный шарнир с заданными `limits` и включённым мотором в `run_physics_simulation` вращается неограниченно.

---

## 4. Затронутые и проверенные файлы
- `src/cad/kinematics.ts`
- `src/cad/press_simulation.ts`
- `src/types/cad.ts`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T35-report.md`

---

## 5. Результаты проверок и тестов
1. **Целевой тест `P5-T35` (`npx --yes tsx --test --test-name-pattern="P5-T35" src/cad/cad_engine.test.ts`)**:
   - **1/1 PASS** (`0 failed`).
2. **Полный прогон всех 11 TypeScript-сьютов (`cad_engine`, `analyze_mechanism`, `destruction_engine`, `ortho_views`, `field_colormaps`, `coupled_sim_stream`, `helix_primitive`, `live_sync_adapter`, `press_simulation`, `rust_scene_sync`, `live_sync_store`)**:
   - **115/115 PASS** (`0 failed`: `85/85` в 5 базовых + `30/30` в 6 расширенных сьютах).
3. **`npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`)**:
   - **PASS** (`0` ошибок).
4. **Примечание по окружению**: `cargo` и `.git` в текущем контейнере отсутствуют (Rust-код в задаче `P5-T35` не изменялся; git-коммит невозможен без каталога `.git`).
