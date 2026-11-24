# Отчёт по задаче P5-T31: Строгая валидация `failure_mode`, поддержка Rust-ключей в `normalizeMaterialSpec` и отказ от truthiness-проверки `0` в TS `define_material` (`src/cad/materials.ts`, `src/cad/mcp_dispatcher.ts`)

**Статус:** ✅ Выполнено  
**Дата:** 24.11.2026  

---

## 1. Описание проблемы
При аудите TypeScript-зеркала `define_material` в ходе `P5-T27` были выявлены три расхождения с контрактом Rust (`crates/mcp-cad-core/src/material.rs:20-29`, `crates/mcp-cad-server/src/tools/cad.rs:3512-3604`):
1. В `src/cad/materials.ts:403-406` (`normalizeMaterialSpec`) любая неизвестная строка `failure_mode` (например, `"ceramic"`, `""` или нестроковое значение) молча преобразовывалась в `'ductile'`, из-за чего `validateMaterialSpec` / `assertValidMaterialSpec` и `define_material` не отклоняли некорректный режим разрушения.
2. В `src/cad/materials.ts:386-393` (`normalizeMaterialSpec`) считывались только `youngs_modulus_gpa` / `youngs_modulus_pa` / `poissons_ratio` (с буквой `s`), но игнорировались канонические ключи Rust MCP (`young_modulus_gpa`, `young_modulus_pa`, `poisson_ratio`).
3. В `src/cad/mcp_dispatcher.ts:1304-1311` (`case 'define_material'`) выражения `params.youngs_modulus_pa ? Number(params.youngs_modulus_pa) : E_gpa * 1e9` и `params.yield_strength_pa ? Number(params.yield_strength_pa) : yieldMpa * 1e6` использовали truthiness-проверку, из-за чего явная передача `0` молча откатывалась к значениям по умолчанию вместо отклонения невалидного параметра.

> **Примечание о выборе задачи в среде выполнения:** задачи `P5-T21`–`P5-T26` относятся к Rust-крейтам (`mcp-cad-fea`, `mcp-cad-server`) и требуют компиляции/проверки через `cargo test`, тогда как в данном веб-контейнере отсутствует Rust-тулчейн (`cargo`/`rustc`). `P5-T31` — первая открытая задача в `TASKS.md` (зависимости `P5-T1`, `P5-T27` выполнены), которую можно полностью реализовать и верифицировать в текущем окружении.

---

## 2. Реализованное решение и верификация
1. **[`src/cad/materials.ts`](../../src/cad/materials.ts) (`normalizeMaterialSpec`, строки `381–483`)**:
   - Добавлена поддержка канонических ключей Rust MCP `young_modulus_pa`, `young_modulus_gpa` и `poisson_ratio` наряду с `youngs_modulus_pa` / `youngs_modulus_gpa` / `poissons_ratio` и camelCase-полями, а проверки `!== undefined` заменены на `!= null` для безопасной обработки `null` без потери явного `0`.
   - Для `failureMode` / `failure_mode`: отсутствующие (`undefined` / `null`) значения по-прежнему получают дефолт `'ductile'`, валидные строки (`'ductile'` / `'brittle'` без учёта регистра и пробелов) нормализуются и задают соответствующие дефолты нелинейных параметров, а любые неизвестные, пустые или нестроковые значения сохраняются без подмены, чтобы `validateMaterialSpec` и `assertValidMaterialSpec` строго их отклоняли.
2. **[`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts) (`case 'define_material'`, строки `1303–1330`)**:
   - Truthiness-проверки для `params.youngs_modulus_pa` и `params.yield_strength_pa` заменены на явные `!== undefined && !== null` (плюс поддержка `params.young_modulus_pa`), а условие валидации дополнено проверками `Number.isFinite(...)` для `density`, `E_pa` и `yieldPa`.
3. **[`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts) (строки `3025–3186`)**:
   - Добавлен unit-тест `verifies P5-T31: strict failure_mode validation, Rust keys in normalizeMaterialSpec, and zero-value rejection in define_material`, проверяющий:
     - чтение канонических ключей Rust (`young_modulus_gpa`, `young_modulus_pa`, `poisson_ratio`) и регистронезависимую нормализацию `'BRITTLE'` / `' Ductile '` в `normalizeMaterialSpec`;
     - отклонение `'ceramic'`, `''`, `'   '` и числового `42` в `validateMaterialSpec`, `assertValidMaterialSpec` и `define_material`;
     - отклонение явных `youngs_modulus_pa: 0`, `young_modulus_pa: 0`, `yield_strength_pa: 0` и `NaN` в `define_material` без truthiness-отката;
     - успешную регистрацию хрупкого материала по каноническим ключам Rust в `define_material`.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T37 [S]`**: В [`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts) (`case 'define_material'`, строка `1301`) выражение `String(params.material_id || params.id || \`mat_${Date.now()}\`)` при явной передаче пустого `id: ""` или `material_id: ""` молча генерирует случайный идентификатор вместо ошибки валидации (в отличие от Rust `crates/mcp-cad-server/src/tools/cad.rs:3518-3520`).
- **`P5-T38 [S]`**: В [`crates/mcp-cad-server/src/live_sync.rs`](../../crates/mcp-cad-server/src/live_sync.rs) (строки `76–83`) и [`src/cad/live_sync_adapter.ts`](../../src/cad/live_sync_adapter.ts) (строки `46–53, 202–211`) структура `LiveMaterialInfo` передаёт только 5 линейных свойств + `failure_mode` и опускает нелинейные поля Фазы 5 (`ultimate_strength_pa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`), из-за чего при Live Sync пользовательских материалов их нелинейные свойства сбрасываются к расчётным дефолтам.

---

## 4. Затронутые и проверенные файлы
- `src/cad/materials.ts`
- `src/cad/mcp_dispatcher.ts`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T31-report.md`

---

## 5. Результаты проверок и тестов
1. **Целевой тест `P5-T31` (`npx --yes tsx --test --test-name-pattern="P5-T31" src/cad/cad_engine.test.ts`)**:
   - **1/1 PASS** (`0 failed`).
2. **Полный прогон всех 11 TypeScript-сьютов (`cad_engine`, `analyze_mechanism`, `destruction_engine`, `ortho_views`, `field_colormaps`, `coupled_sim_stream`, `helix_primitive`, `live_sync_adapter`, `press_simulation`, `rust_scene_sync`, `live_sync_store`)**:
   - **111/111 PASS** (`0 failed`: `81/81` в 5 базовых + `30/30` в 6 расширенных сьютах).
3. **`npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`)**:
   - **PASS** (`0` ошибок).
4. **Примечание по окружению**: `cargo` и `.git` в текущем контейнере отсутствуют (Rust-код в задаче `P5-T31` не изменялся; git-коммит невозможен без каталога `.git`).
