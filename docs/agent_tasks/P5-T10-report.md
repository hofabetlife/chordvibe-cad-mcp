# Отчёт по задаче `P5-T10`: Синхронизация списка `builtin_materials` в ошибке `UNKNOWN_MATERIAL` с фактическим `MaterialRegistry`

- **Дата выполнения**: 21.11.2026
- **Статус**: ✅ DONE

---

## 1. Реализованные изменения

### 1.1. Динамическая синхронизация `builtin_materials` и `hint` (`crates/mcp-cad-server/src/error.rs`)
- Добавлена потокобезопасная вспомогательная функция `builtin_material_ids() -> &'static [String]` на базе `std::sync::OnceLock<Vec<String>>`, которая динамически извлекает канонические имена всех встроенных пресетов из `mcp_cad_core::MaterialRegistry::new().list()` и детерминированно сортирует их (`aluminum_6061_t6`, `borosilicate_glass`, `brass_c360`, `nylon_pa12`, `petg_filament_3dprint`, `pla_filament_3dprint`, `steel_high_strength_4140`, `steel_structural_aisi1020`, `titanium_grade5_ti6al4v`).
- В `map_invalid_argument_error` (ветка `UNKNOWN_MATERIAL`):
  - удалён устаревший захардкоженный список с несуществующими материалами (`steel_stainless_304`, `copper_c10100`, `abs_plastic`, `polycarbonate`);
  - `details.builtin_materials` и строка подсказки `hint` теперь формируются напрямую из `builtin_material_ids()`, гарантируя 100% совпадение с `MaterialRegistry` при любых будущих расширениях библиотеки материалов;
  - расширено условие распознавания неизвестного материала: помимо `"Unknown material"` теперь также корректно классифицируются фактические сообщения об отсутствии материала из `create_part`, `import_stl` и `load_project` (`"Material '...' not found in registry..."` / `"Part '...' references material '...' which was not found in registry"`), ранее ошибочно проваливавшиеся в общий код `INVALID_ARGUMENT` с извлечением имени материала в качестве `param`.
- В `map_cad_error` (ветка `CadError::MaterialNotFound`) добавлен тот же синхронизированный перечень `builtin_materials` в `hint` и `details.builtin_materials`.

### 1.2. Тесты (`crates/mcp-cad-server/src/error.rs`)
- Добавлен unit-тест `error::tests::test_p5_t10_unknown_material_builtin_materials_match_registry`, проверяющий:
  - все форматы сообщений об отсутствующем материале (`Unknown material`, `create_part`/`import_stl`, `load_project`) и `CadError::MaterialNotFound`;
  - 100% совпадение `details.builtin_materials` с отсортированным каноническим списком `MaterialRegistry::new()`, проверку `registry.contains(id)` и `registry.get(id).name == id` для каждого элемента;
  - присутствие всех 9 канонических пресетов в `hint` и отсутствие устаревших несуществующих идентификаторов (`steel_stainless_304`, `copper_c10100`, `abs_plastic`, `polycarbonate`).
- Добавлен сквозной тест `error::tests::test_p5_t10_create_part_unknown_material_returns_synchronized_envelope`, вызывающий `tools::cad::create_part` с неизвестным материалом `"vibranium"` и проверяющий, что `to_error_body()` возвращает `code == "UNKNOWN_MATERIAL"`, `param == Some("material")` и полный актуальный список `details.builtin_materials`.

---

## 2. Результаты верификации

1. `cargo test -p mcp-cad-server --lib error::tests --no-fail-fast`: **21/21 PASS** (`0.01s`, включая 2 новых теста `P5-T10`).
2. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
3. `cargo test -p mcp-cad-server --no-fail-fast`:
   - `--lib`: **70/71 PASS** (`70.42s`; 1 предсуществующее несвязанное падение `tools::project::tests::test_save_and_load_roundtrip` в `project.rs:682` отслеживается в `P5-T17`);
   - `tests/bench_suite.rs`: **1/1 PASS** (`7.48s`);
   - `tests/gear_kinematic_test.rs`: **2/2 PASS** (`0.83s`);
   - `tests/mcp_contract_tests.rs`: **61/62 PASS** (`46.94s`; 1 предсуществующее несвязанное падение `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` в `mcp_contract_tests.rs:4659` отслеживается в `P5-T17`);
   - `tests/qa_wide_scan.rs`: **13/13 PASS** (`8.83s`).

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)

- **`P5-T18 [S]`**: В `check_gear_strength` (`crates/mcp-cad-server/src/tools/cad.rs:3787-3835`) при передаче явного неизвестного значения `material` или `mating_material` без ручного переопределения механических свойств инструмент молча подставляет параметры стали AISI 1020 вместо возврата ошибки `UNKNOWN_MATERIAL`, заявленной в описании инструмента (`tools/mod.rs:1412`).
- **`P5-T19 [S]`**: В описании и JSON-схеме инструмента `create_part` (`crates/mcp-cad-server/src/tools/mod.rs:26, 36`) перечислены только 8 из 9 встроенных материалов — отсутствует добавленный в `P5-T1` пресет `borosilicate_glass`.
