# Отчёт по задаче P5-T27: Документирование параметров Фазы 5 инструмента `define_material` в `docs/MCP_API_REFERENCE.md`

**Статус:** ✅ Выполнено  
**Дата:** 24.11.2026  

---

## 1. Описание проблемы
При аудите [`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md) в `P5-T19` было выявлено, что раздел `### 25. define_material` (строки `1440–1488`) описывал только 6 базовых линейно-упругих полей (`id`, `young_modulus_gpa`, `poisson_ratio`, `yield_strength_mpa`, `density_kg_m3`, `name`) и устаревший пример ответа без полей Фазы 5 и блока `"units"`, тогда как JSON-схема (`crates/mcp-cad-server/src/tools/mod.rs:982-1050`) и обработчик `cad::define_material` (`crates/mcp-cad-server/src/tools/cad.rs:3512-3647`) с `P5-T1` поддерживают все 7 конститутивных параметров Фазы 5 (`ultimate_strength_pa`, `ultimate_strength_mpa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`, `failure_mode`) и возвращают их в `properties` вместе с блоком `units`.

> **Примечание о выборе задачи в среде выполнения:** задачи `P5-T21`–`P5-T26` относятся к Rust-крейтам (`mcp-cad-fea`, `mcp-cad-server`) и требуют `cargo test` для компиляции и проверки, тогда как в данном веб-контейнере отсутствует Rust-тулчейн (`cargo`/`rustc`). `P5-T27` — первая открытая задача в `TASKS.md` (зависимости `P5-T1`, `P5-T19` выполнены), которую можно полностью выполнить и верифицировать в текущем окружении.

---

## 2. Реализованное решение и верификация
1. **[`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md) (`### 25. define_material`)**:
   - Расширена таблица `#### Входные параметры`: добавлены все 7 опциональных конститутивных параметров Фазы 5 (`ultimate_strength_pa`, `ultimate_strength_mpa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`, `failure_mode`) с точными типами, единицами измерения (`Pa`, `MPa`, `J/m²`, безразмерные), приоритетом `ultimate_strength_pa` над `ultimate_strength_mpa` и значениями по умолчанию из `derived_nonlinear_defaults` (`crates/mcp-cad-core/src/material.rs:42-80`) для режимов `"ductile"` и `"brittle"`.
   - Добавлен блок проверяемых физических инвариантов валидации (`INVALID_ARGUMENT`: $E > 0$, $0 < \nu < 0.5$, $\rho > 0$, $\sigma_u \ge \sigma_y > 0$, $H \ge 0$, $\varepsilon_f > 0$, $G_f > 0$, $\mu \ge 0$, `failure_mode ∈ {"ductile", "brittle"}`, конечность `f64::is_finite`).
   - Актуализированы `#### Пример запроса` и `#### Пример ответа` в точном соответствии с JSON-структурой `cad::define_material` (`crates/mcp-cad-server/src/tools/cad.rs:3626-3646`) и `LengthUnit::units_metadata` (`crates/mcp-cad-server/src/state.rs:90-110`).
2. **Автоматическая контрактная проверка ([`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts))**:
   - Добавлен unit-тест `verifies P5-T27: docs/MCP_API_REFERENCE.md define_material section matches Rust inputSchema and response properties`, который:
     - парсит `crates/mcp-cad-server/src/tools/mod.rs` (все 13 параметров `define_material.input_schema.properties` и 5 обязательных `required`) и `crates/mcp-cad-server/src/tools/cad.rs` (все 14 полей `properties` в ответе);
     - проверяет наличие каждого параметра в таблице раздела `### 25. define_material` с корректным статусом обязательности (`Да` / `Нет`);
     - парсит оба JSON-примера (запрос и ответ, включая встроенный `result.content[0].text`), сверяет наличие всех 14 ключей `properties` и блока `units`, а также валидирует физические инварианты примера через `validateMaterialSpec(normalizeMaterialSpec(...))`.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T30 [S]`**: В `src/cad/cad_engine.test.ts:1670-1695` предсуществующий тест `supports P2-T15 small wheel_rim (< 140 mm) without clamp panic and prismatic joint motor actuation` падает (`Expected slider_carriage to move > 0.3 m along +X under prismatic motor, got 0.0005`), поскольку `create_joint` в `src/cad/mcp_dispatcher.ts:1447` по умолчанию трактует `limits: [-0.5, 0.5]` призматического шарнира в миллиметрах (`length_unit ?? 'mm'`), зажимая ход ползунка на `0.0005 м`.
- **`P5-T31 [S]`**: В `src/cad/materials.ts:384-406` (`normalizeMaterialSpec`) и `src/cad/mcp_dispatcher.ts:1304-1317` (`define_material`): (1) `normalizeMaterialSpec` молча приводит любую неизвестную строку `failure_mode` к `'ductile'` вместо отклонения с ошибкой (в отличие от Rust `FailureMode::parse`); (2) `normalizeMaterialSpec` не читает канонические Rust-ключи `young_modulus_gpa` / `poisson_ratio` (без `s`); (3) `mcp_dispatcher.ts:1304, 1309` проверяет `params.youngs_modulus_pa` и `params.yield_strength_pa` через truthiness (`?`), из-за чего передача `0` молча игнорируется и откатывается к `E_gpa`/`yieldMpa` вместо выброса ошибки валидации.
- **`P5-T32 [S]`**: Оглавление [`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md) (строки `50–61`) заканчивается на инструменте `41. apply_recipe` и не содержит ссылок на документированные в конце файла разделы `42. bake_coupled_simulation` (строка `2268`) и `43. query_simulation_frame` (строка `2320`).

---

## 4. Затронутые и проверенные файлы
- `docs/MCP_API_REFERENCE.md`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T27-report.md`

---

## 5. Результаты проверок и тестов
1. **Тест `P5-T27` (`npx --yes tsx --test --test-name-pattern="P5-T27" src/cad/cad_engine.test.ts`)**:
   - **1/1 PASS** (`0 failed`).
2. **Полный прогон TypeScript-тестов (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts`)**:
   - **77/78 PASS, 1 pre-existing fail** (единственное падение — предсуществующий тест `P2-T15` на строке `1692`, не связанный с `P5-T27`, заведён в `P5-T30`).
3. **`npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`)**:
   - **PASS** (`0` ошибок).
4. **Примечание по окружению**: `cargo` и `.git` в текущем контейнере отсутствуют (Rust-код в задаче `P5-T27` не изменялся; коммит в git невозможен без каталога `.git`).
