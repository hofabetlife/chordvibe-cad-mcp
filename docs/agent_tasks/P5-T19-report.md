# Отчёт по задаче P5-T19: Актуализация перечня встроенных материалов в описании и схеме `create_part` (`crates/mcp-cad-server/src/tools/mod.rs`, `docs/MCP_API_REFERENCE.md`)

**Статус:** ✅ Выполнено  
**Дата:** 22.11.2026  

---

## 1. Описание проблемы
При аудите `P5-T10` и `P5-T11` было выявлено, что в описании инструмента `create_part` (`description` и `input_schema.properties.material.description` в `crates/mcp-cad-server/src/tools/mod.rs:26, 36`) и в справочнике [`docs/MCP_API_REFERENCE.md`](../MCP_API_REFERENCE.md) перечислены только 8 устаревших записей из 10 встроенных пресетов `MaterialRegistry::new()`:
- отсутствовал пресет `borosilicate_glass` (алиасы `pyrex`, `glass`) из `P5-T1`;
- алиас `steel_1045` всё ещё указывался под `steel_structural_aisi1020` вместо самостоятельного канонического пресета `steel_carbon_aisi1045` (`P5-T11`).

---

## 2. Реализованное решение
1. **`crates/mcp-cad-server/src/tools/mod.rs` (`list_tools` → `create_part`)**:
   - В `description` и `input_schema.properties.material.description` добавлены оба недостающих пресета: `steel_carbon_aisi1045` (`steel_1045`) и `borosilicate_glass` (`pyrex`, `glass`), а к `steel_structural_aisi1020` привязан корректный алиас `steel_1020` (убрано ошибочное смешение `steel_structural_aisi1020 / steel_1045`). Теперь обе строки перечисляют все 10 канонических встроенных материалов из `MaterialRegistry::new()`.
2. **`docs/MCP_API_REFERENCE.md` (раздел `### 1. create_part`)**:
   - Список доступных пресетов материалов расширен до 10 записей: исправлены алиасы `steel_structural_aisi1020` (`steel_1020`, `AISI 1020`, `steel`, `structural_steel`), добавлены `steel_carbon_aisi1045` (`steel_1045`, `AISI 1045`, `1045`, `carbon_steel_1045`, $E=205.0\text{ GPa}, \sigma_y=350\text{ MPa}, \rho=7850\text{ kg/m}^3$) и `borosilicate_glass` (`pyrex`, `glass`, `borosilicate`, $E=64.0\text{ GPa}, \sigma_y=120\text{ MPa}, \rho=2500\text{ kg/m}^3, \text{brittle}$).
3. **Контрактные тесты (`crates/mcp-cad-server/tests/mcp_contract_tests.rs`)**:
   - В `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` добавлена автоматическая проверка присутствия всех 10 канонических имён из `MaterialRegistry::new()` в `description` и `inputSchema.properties.material.description` инструмента `create_part` и отсутствия ложной связки `steel_structural_aisi1020 / steel_1045`.
   - Добавлен отдельный тест `test_mcp_p5_t19_create_part_material_schema_sync`, проверяющий `tools/list`, секцию `create_part` в `docs/MCP_API_REFERENCE.md`, а также реальный вызов `create_part` через JSON-RPC для `steel_carbon_aisi1045`, `steel_1045`, `borosilicate_glass` и `pyrex`.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T26 [S]`**: В `create_part` (`crates/mcp-cad-server/src/tools/cad.rs:317-345`) отсутствует валидация пустых/пробельных строк `name` и `part_id` (`name.trim().is_empty()` / `part_id.trim().is_empty()`), хотя `create_part.description` явно декларирует `INVALID_ARGUMENT (empty name)`; также в таблице входных параметров `create_part` в `docs/MCP_API_REFERENCE.md:263-268` не указан необязательный параметр `part_id`.
- **`P5-T27 [S]`**: В разделе `define_material` документа [`docs/MCP_API_REFERENCE.md`](../MCP_API_REFERENCE.md) (строки `1441-1485`) не задокументированы конститутивные параметры Фазы 5 (`ultimate_strength_pa`, `ultimate_strength_mpa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`, `failure_mode`).

---

## 4. Изменённые файлы
- `crates/mcp-cad-server/src/tools/mod.rs`
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs`
- `docs/MCP_API_REFERENCE.md`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T19-report.md`

---

## 5. Результаты проверок и тестов
1. **`cargo test -p mcp-cad-server --no-fail-fast`** — **154/154 PASS (`0 failed`)**:
   - `--lib` (`src/lib.rs`): **75 passed, 0 failed**;
   - `bench_suite`: **1 passed, 0 failed**;
   - `gear_kinematic_test`: **2 passed, 0 failed**;
   - `mcp_contract_tests`: **63 passed, 0 failed** (включая расширенный `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` и новый `test_mcp_p5_t19_create_part_material_schema_sync`);
   - `qa_wide_scan`: **13 passed, 0 failed**.
2. **`cargo test -p mcp-cad-core --lib material::tests --no-fail-fast`**:
   - **7 passed, 0 failed**.
3. **`cargo clippy --workspace --all-targets -- -D warnings`**:
   - **0 ошибок, 0 предупреждений** во всех 5 крейтах workspace (`6.98s`).
4. **TypeScript Unit Tests (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts`)**:
   - **74 passed, 0 failed**.
5. **`npm run lint` (`lint_applet`) & `npm run build` (`compile_applet`)**:
   - **PASS** (`0` ошибок).
