# Отчёт по задаче P5-T18: Валидация неизвестного `material` / `mating_material` в `check_gear_strength` (`crates/mcp-cad-server/src/tools/cad.rs`)

**Статус:** ✅ Выполнено  
**Дата:** 22.11.2026  

---

## 1. Описание проблемы
При аудите `UNKNOWN_MATERIAL` в `P5-T10` было выявлено, что инструмент `check_gear_strength` (`crates/mcp-cad-server/src/tools/cad.rs:3787-3835`) при явной передаче несуществующего идентификатора `material` или `mating_material` (без полного ручного переопределения механических свойств) молча подставлял параметры стали AISI 1020 (`250 MPa`, `205 GPa`, `0.29`) вместо возврата структурированной ошибки `UNKNOWN_MATERIAL`, заявленной в контракте инструмента (`crates/mcp-cad-server/src/tools/mod.rs:1412`).

---

## 2. Реализованное решение
1. **`crates/mcp-cad-server/src/tools/cad.rs` (`check_gear_strength`)**:
   - При наличии `part_id` сохраняется полный объект `part.material` (`Option<Material>`), а не только имя материала (исключена потеря параметров для деталей с пользовательскими материалами).
   - Явные строковые параметры `material` и `mating_material` проверяются на строковый тип (`INVALID_ARGUMENT` при передаче не-строки, кроме `null`) и ищутся напрямую через `state.materials.get(...)` (с поддержкой пользовательских материалов из `define_material` и всех алиасов `MaterialRegistry`).
   - Если `material` явно задан и не найден в `state.materials`, проверяется наличие полного ручного переопределения всех первичных механических свойств (`(yield_strength_mpa.is_some() || allowable_bending_stress_mpa.is_some()) && youngs_modulus_gpa.is_some() && poissons_ratio.is_some()`). При отсутствии полного переопределения возвращается `ServerError::InvalidArgument(format!("Material '{}' not found in registry. Available presets: {}", ...))`, маппящийся в `ErrorEnvelope` с `code == "UNKNOWN_MATERIAL"`, `param == "material"` и актуальным `details.builtin_materials`.
   - Если `mating_material` явно задан и не найден в `state.materials`, возвращается `ServerError::InvalidArgument(format!("Mating material '{}' not found in registry. Available presets: {}", ...))`, также маппящийся в `UNKNOWN_MATERIAL` (`param == "material"`, `details.builtin_materials`).
   - Добавлена проверка положительности/допустимости явно переданных числовых переопределений (`yield_strength_mpa > 0`, `youngs_modulus_gpa > 0`, `-1 < poissons_ratio < 0.5`).
2. **Тесты**:
   - В `crates/mcp-cad-server/src/tools/cad.rs` добавлен unit-тест `tools::cad::tests::test_p5_t18_check_gear_strength_unknown_material_and_overrides`, проверяющий:
     - неизвестный `material` без переопределений → `UNKNOWN_MATERIAL` (`param == "material"`, полный `builtin_materials`);
     - неизвестный `material` с неполным переопределением (только `yield_strength_mpa` без `youngs_modulus_gpa`/`poissons_ratio`) → `UNKNOWN_MATERIAL` (исключён молчаливый фолбэк на `E`/`nu` AISI 1020);
     - неизвестный `mating_material` → `UNKNOWN_MATERIAL` (`param == "material"`, полный `builtin_materials`);
     - неизвестный `material` с полным набором переопределений (`yield_strength_mpa`, `youngs_modulus_gpa`, `poissons_ratio`) → успешный расчёт с использованием переопределённых констант;
     - валидные алиасы `material: "steel_1045"` и `mating_material: "brass"` → корректное разрешение в `steel_carbon_aisi1045` и `brass_c360`.
   - В `crates/mcp-cad-server/tests/mcp_contract_tests.rs` (`test_mcp_p3_t8_check_gear_strength_contract`) добавлены контрактные JSON-RPC проверки `UNKNOWN_MATERIAL` для неизвестных `material` и `mating_material`.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T25 [S]`**: В JSON-схеме `check_gear_strength` (`crates/mcp-cad-server/src/tools/mod.rs:1414-1487`) и в [`docs/MCP_API_REFERENCE.md`](../MCP_API_REFERENCE.md) отсутствуют поля `youngs_modulus_gpa` и `poissons_ratio`, хотя `cad.rs` их читает и использует при ручном переопределении свойств материала.

---

## 4. Изменённые файлы
- `crates/mcp-cad-server/src/tools/cad.rs`
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T18-report.md`

---

## 5. Результаты проверок и тестов
1. **`cargo test -p mcp-cad-server --no-fail-fast`** — **153/153 PASS (`0 failed`)**:
   - `--lib` (`src/lib.rs`): **75 passed, 0 failed** (включая `tools::cad::tests::test_p5_t18_check_gear_strength_unknown_material_and_overrides`);
   - `bench_suite`: **1 passed, 0 failed**;
   - `gear_kinematic_test`: **2 passed, 0 failed**;
   - `mcp_contract_tests`: **62 passed, 0 failed** (включая расширенный `test_mcp_p3_t8_check_gear_strength_contract`);
   - `qa_wide_scan`: **13 passed, 0 failed**.
2. **`cargo clippy --workspace --all-targets -- -D warnings`**:
   - **0 ошибок, 0 предупреждений** во всех 5 крейтах workspace (`6.53s`).
3. **TypeScript Unit Tests (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts`)**:
   - **74 passed, 0 failed**.
4. **`npm run lint` (`lint_applet`) & `npm run build` (`compile_applet`)**:
   - **PASS** (`0` ошибок).
