# Отчёт по задаче P5-T17: Актуализация двух тестов `mcp-cad-server` после расширения материалов в `P5-T1`/`P5-T11` (`project.rs:682` и `mcp_contract_tests.rs:4659`)

**Статус:** ✅ Выполнено  
**Дата:** 2026-11-22  

---

## 1. Описание проблемы
После реализации `P5-T9` (программного CPU-растеризатора `SoftwareCpu`, разблокировавшего запуск всех тестов `mcp-cad-server` в headless-контейнере без GPU и Vulkan Lavapipe) и расширения реестра материалов в `P5-T1` (`borosilicate_glass`, конститутивные поля Фазы 5 в `define_material`) и `P5-T11` (`steel_carbon_aisi1045`) в `mcp-cad-server` оставались два устаревших тестовых утверждения:
1. В `crates/mcp-cad-server/src/tools/project.rs:682` (`tools::project::tests::test_save_and_load_roundtrip`) было жёстко закодировано `assert_eq!(load_res["materials_count"], 9); // 8 builtins + 1 custom`, тогда как в `MaterialRegistry::new()` зарегистрировано 10 встроенных материалов (`10 builtins + 1 custom = 11`).
2. В `crates/mcp-cad-server/tests/mcp_contract_tests.rs:4597-4621` (`assert_numeric_schema_fields_have_units`, вызываемой из `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`) массив допустимых ключевых слов единиц измерения `unit_keywords` не включал единицу удельной энергии разрушения `"J/m²"` (`"j/m²"` / `"j/m^2"`) и `"Pascal"` (`"pascal"`) для новых числовых параметров `define_material.fracture_energy_j_m2`, `define_material.hardening_modulus_pa` и `define_material.ultimate_strength_pa`.

---

## 2. Реализованное решение
1. **`crates/mcp-cad-server/src/tools/project.rs`**:
   - В `tools::project::tests::test_save_and_load_roundtrip` жёсткая константа `9` заменена на динамическое вычисление от канонического реестра:
     ```rust
     let expected_materials_count = mcp_cad_core::MaterialRegistry::new().list().len() + 1;
     assert_eq!(load_res["materials_count"], expected_materials_count); // 10 builtins + 1 custom
     ```
2. **`crates/mcp-cad-server/tests/mcp_contract_tests.rs`**:
   - В `assert_numeric_schema_fields_have_units` список `unit_keywords` дополнен единицами `"pascal"`, `"j/m²"` и `"j/m^2"`, что обеспечивает строгую проверку единиц измерения для всех числовых полей всех 43 инструментов MCP, включая конститутивные параметры Фазы 5 в `define_material`.

---

## 3. Изменённые файлы
- `crates/mcp-cad-server/src/tools/project.rs`
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T17-report.md`

---

## 4. Результаты проверок и тестов
1. **`cargo test -p mcp-cad-server --no-fail-fast`** — **152/152 PASS (`0 failed`)**:
   - `--lib` (`src/lib.rs`): **74 passed, 0 failed** (включая `tools::project::tests::test_save_and_load_roundtrip`);
   - `bench_suite`: **1 passed, 0 failed**;
   - `gear_kinematic_test`: **2 passed, 0 failed**;
   - `mcp_contract_tests`: **62 passed, 0 failed** (включая `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`);
   - `qa_wide_scan`: **13 passed, 0 failed**.
2. **`cargo clippy --workspace --all-targets -- -D warnings`**:
   - **0 ошибок, 0 предупреждений** во всех 5 крейтах workspace.
3. **TypeScript Unit Tests (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts`)**:
   - **74 passed, 0 failed**.
4. **`npm run lint` & `npm run build`**:
   - **PASS** (`0` ошибок).
