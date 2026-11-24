# Отчёт по задаче P5-T30: Исправление рассогласования единиц `limits` призматического шарнира в тесте `P2-T15` (`src/cad/cad_engine.test.ts`, `src/cad/mcp_dispatcher.ts`)

**Статус:** ✅ Выполнено  
**Дата:** 24.11.2026  

---

## 1. Описание проблемы
При полном прогоне TypeScript-тестов в `P5-T27` и `P5-T29` было зафиксировано предсуществующее падение теста `supports P2-T15 small wheel_rim (< 140 mm) without clamp panic and prismatic joint motor actuation` (`src/cad/cad_engine.test.ts:1629-1705`) с ошибкой:
`Expected slider_carriage to move > 0.3 m along +X under prismatic motor, got 0.0005`.

Причина: согласно контракту Rust (`crates/mcp-cad-server/src/tools/mod.rs:389, 469`, `crates/mcp-cad-server/src/tools/physics.rs:132-133, 257` и `mcp_contract_tests.rs:6008`), инструмент `create_joint` по умолчанию принимает `limits` призматического шарнира в активной единице длины (`length_unit = "mm"`, множитель `kToM = 0.001`), преобразуя их во внутренние метры СИ. Обработчик `create_joint` в `src/cad/mcp_dispatcher.ts:1447` корректно следовал этому контракту, тогда как тест `P2-T15` передавал `limits: [-0.5, 0.5]` без `length_unit: 'm'`, из-за чего ход ползунка ограничивался `[-0.0005 м, +0.0005 м]`. Кроме того, TS `create_joint` не валидировал некорректный `length_unit` и не возвращал поле `length_unit` в ответе, как это делает Rust-сервер (`physics.rs:381`).

> **Примечание о выборе задачи в среде выполнения:** задачи `P5-T21`–`P5-T26` относятся к Rust-крейтам (`mcp-cad-fea`, `mcp-cad-server`) и требуют компиляции/проверки через `cargo test`, тогда как в данном веб-контейнере отсутствует Rust-тулчейн (`cargo`/`rustc`). `P5-T30` — первая открытая задача в `TASKS.md` (зависимости `P2-T15`, `P5-T27` выполнены), которую можно полностью реализовать и верифицировать в текущем окружении.

---

## 2. Реализованное решение и верификация
1. **[`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts) (`case 'create_joint'`, строки `1430–1501`)**:
   - Добавлена строгая валидация `params.length_unit` (`'mm'` по умолчанию, поддерживаются только `'mm'` и `'m'`, при иных значениях выбрасывается ошибка `Unsupported length_unit`), приведённая в соответствие с `state.resolve_length_unit(args)` в Rust.
   - Единый множитель `kToM = rawUnit === 'm' ? 1.0 : 0.001` используется для преобразования `limits` призматического шарнира во внутренние метры СИ.
   - В ответ `create_joint` добавлено поле `length_unit: rawUnit` (паритет с `crates/mcp-cad-server/src/tools/physics.rs:381`).
2. **[`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts)**:
   - В тесте `P2-T15` (строки `1670–1682`) пределы призматического шарнира приведены в соответствие с контрактом по умолчанию (`limits: [-500, 500]` мм → `[-0.5, 0.5]` м, идентично Rust-тесту `test_mcp_p2_t15_small_wheel_rim_and_prismatic_motor_and_revolute_limits` в `mcp_contract_tests.rs:6008`), добавлена проверка `response.length_unit === 'mm'` и `prismJoint.limits === [-0.5, 0.5]`.
   - Добавлен отдельный unit-тест `verifies P5-T30: create_joint prismatic limits honor default mm and explicit m length_unit consistently with Rust contract` (строки `2886–3019`), проверяющий:
     - `prismatic` по умолчанию (`mm`): `limits: [-500, 500]` → `[-0.5, 0.5]` м и ход ползунка до `+0.5 м`;
     - `prismatic` с явным `length_unit: 'm'`: `limits: [-0.5, 0.5]` → `[-0.5, 0.5]` м и ход ползунка до `+0.5 м`;
     - `prismatic` с субмиллиметровыми `limits: [-0.5, 0.5]` при `length_unit` по умолчанию → `[-0.0005, 0.0005]` м;
     - отклонение неподдерживаемого `length_unit: 'inch'`;
     - `revolute` по умолчанию (`deg`: `[-90, 90]` → `[-π/2, π/2]`) и явный `limits_unit: 'rad'`.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T34 [S]`**: В [`docs/MCP_API_REFERENCE.md`](../MCP_API_REFERENCE.md) (`### 11. create_joint`, строки `694–710`) таблица входных параметров устарела относительно Rust-схемы и реализации (`mod.rs:388-473`, `physics.rs:132-316`): `anchor_a`/`anchor_b` указаны в метрах `(м)` вместо активного `length_unit` (`mm` по умолчанию / `m`), `limits` указаны как `(в радианах или метрах)` вместо `deg`/`rad` и `mm`/`m`, отсутствуют параметры `length_unit` и `limits_unit`, а формула `teeth_a`/`teeth_b` указана с минусом (`i = -z_A / z_B`) вместо `ratio = teeth_a / teeth_b`.
- **`P5-T35 [S]`**: В [`src/cad/kinematics.ts`](../../src/cad/kinematics.ts) (строка `91`) и [`src/cad/press_simulation.ts`](../../src/cad/press_simulation.ts) (строка `446`) эвристика `> 1.0 ? 1e-3 : 1.0` для `prismatic` `joint.limits` ошибочно делит на `1000` любой ход призматического шарнира больше `1.0 м` (например, `[-1500, 1500]` мм → `[-1.5, 1.5]` м → зажим на `[-0.0015, 0.0015]` м), хотя внутренние `KinematicJoint.limits` всегда хранятся в метрах.
- **`P5-T36 [S]`**: В [`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts) (`case 'create_joint'`, строки `1453–1466`) координаты `anchor_a`/`anchor_b` не масштабируются через `kToM` (`length_unit`), а отдельное поле `anchorB` не сохраняется в `KinematicJoint` (в отличие от Rust `physics.rs:202-204, 236-264`).

---

## 4. Затронутые и проверенные файлы
- `src/cad/mcp_dispatcher.ts`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T30-report.md`

---

## 5. Результаты проверок и тестов
1. **Целевые тесты `P2-T15` и `P5-T30` (`npx --yes tsx --test --test-name-pattern="P2-T15|P5-T30" src/cad/cad_engine.test.ts`)**:
   - **2/2 PASS** (`0 failed`).
2. **Полный прогон 5 базовых TypeScript-сьютов (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts`)**:
   - **80/80 PASS** (`0 failed`, 100% зелёный прогон).
3. **Расширенный прогон остальных 6 TypeScript-сьютов (`coupled_sim_stream`, `helix_primitive`, `live_sync_adapter`, `press_simulation`, `rust_scene_sync`, `live_sync_store`)**:
   - **30/30 PASS** (`0 failed`; итого **110/110 PASS** по всем 11 TS-тестам).
4. **`npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`)**:
   - **PASS** (`0` ошибок).
5. **Примечание по окружению**: `cargo` и `.git` в текущем контейнере отсутствуют (Rust-код в задаче `P5-T30` не изменялся; git-коммит невозможен без каталога `.git`).
