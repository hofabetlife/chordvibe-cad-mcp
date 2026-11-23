# Отчёт по задаче `P5-T11`: Устранение расхождения параметров стали `steel_1045` между Rust (`steel_structural_aisi1020`) и TypeScript (`steel_1045`)

- **Дата выполнения**: 21.11.2026
- **Статус**: ✅ DONE

---

## 1. Реализованные изменения

### 1.1. Разделение пресетов AISI 1020 и AISI 1045 в Rust (`crates/mcp-cad-core/src/material.rs`)
- В `MaterialRegistry::register_presets()` добавлен самостоятельный 10-й встроенный пресет `"steel_carbon_aisi1045"` (углеродистая сталь AISI 1045):
  - $\rho = 7850.0\text{ кг/м}^3$, $E = 205.0\text{ ГПа}$, $\nu = 0.29$;
  - $\sigma_y = 350.0\text{ МПа}$, $\sigma_u = 585.0\text{ МПа}$, $H = 1.80\text{ ГПа}$, $\varepsilon_f = 0.16$, $G_f = 28\,000.0\text{ Дж/м}^2$, $\mu = 0.25$, `failure_mode = Ductile`.
- Пресет `"steel_structural_aisi1020"` (AISI 1020) сохранён без изменений ($\sigma_y = 250.0\text{ МПа}$, $\sigma_u = 420.0\text{ МПа}$, $H = 1.20\text{ ГПа}$, $\varepsilon_f = 0.22$, $G_f = 28\,000.0\text{ Дж/м}^2$, $\mu = 0.25$, `Ductile`).
- В `MaterialRegistry::resolve_alias()` разделены таблицы алиасов:
  - `"steel_structural_aisi1020" | "steel_1020" | "aisi_1020" | "aisi1020" | "steel" | "structural_steel" | "steel_structural"` $\to$ `"steel_structural_aisi1020"`;
  - `"steel_carbon_aisi1045" | "steel_1045" | "aisi_1045" | "aisi1045" | "1045" | "carbon_steel_1045" | "steel_carbon_1045"` $\to$ `"steel_carbon_aisi1045"`.

### 1.2. Симметричное разделение пресетов в TypeScript (`src/cad/materials.ts`) и бенчмарке (`crates/mcp-cad-server/tests/bench_suite.rs`)
- В `MATERIALS_LIBRARY` (`src/cad/materials.ts`) сохранён `'steel_1045'` на позиции `0` (обратная совместимость веб-студии и тестов `DEFAULT_MATERIALS[0]`) и добавлен отдельный пресет `'steel_structural_aisi1020'` (AISI 1020, $\sigma_y = 250\text{ МПа}$, $\sigma_u = 420\text{ МПа}$, $H = 1.2\text{ ГПа}$, $\varepsilon_f = 0.22$, $G_f = 28\,000\text{ Дж/м}^2$, $\mu = 0.25$, `'ductile'`).
- В `MATERIAL_ALIAS_MAP` алиасы AISI 1020 (`steel_structural_aisi1020`, `steel_1020`, `aisi_1020`, `aisi1020`, `steel`, `structural_steel`, `steel_structural`) переведены на `'steel_structural_aisi1020'`, а алиасы AISI 1045 (`steel_carbon_aisi1045`, `steel_1045`, `aisi_1045`, `aisi1045`, `1045`, `carbon_steel_1045`, `steel_carbon_1045`) — на `'steel_1045'`.
- В `crates/mcp-cad-server/tests/bench_suite.rs` (`map_ts_material_to_rust`) `"steel_1045"` и `"steel_carbon_aisi1045"` теперь маппятся в `"steel_carbon_aisi1045"`, а `"steel_1020"` / `"steel_structural_aisi1020"` — в `"steel_structural_aisi1020"`.

### 1.3. Двусторонняя кросс-проверка паритета Rust ↔ TypeScript
- В **Rust** (`crates/mcp-cad-core/src/material.rs`: `test_p5_t11_rust_ts_standard_materials_parity`): тест напрямую читает `src/cad/materials.ts`, парсит все 10 записей `MATERIALS_LIBRARY` и все 50+ записей `MATERIAL_ALIAS_MAP`, проверяет взаимно-однозначное соответствие всех 10 стандартных пресетов и равенство всех 10 физических параметров $(\rho, E, \nu, \sigma_y, \sigma_u, H, \varepsilon_f, G_f, \mu, \text{failure\_mode})$ с точностью $\le 10^{-9}$, а также согласованность разрешения каждого алиаса.
- В **TypeScript** (`src/cad/cad_engine.test.ts`: `verifies P5-T11: steel_1045 and steel_structural_aisi1020 are distinct and all 10 presets match Rust MaterialRegistry exactly`): тест напрямую читает `crates/mcp-cad-core/src/material.rs`, парсит все регистрации в `register_presets()` и все ветки `resolve_alias()`, проверяя идентичность всех 10 параметров и каждого Rust-алиаса с TypeScript `getMaterialById()`.

---

## 2. Результаты верификации

1. `cargo test -p mcp-cad-core --lib material::tests --no-fail-fast`: **7/7 PASS** (`0.00s`, включая `test_p5_t11_rust_ts_standard_materials_parity`).
2. `cargo test -p mcp-cad-core --no-fail-fast`: **137/137 PASS** (`--lib`: `80/80`, `csg_invariants`: `10/10`, `dag_tests`: `16/16`, `primitive_invariants`: `25/25`, `tessellation_tests`: `6/6`).
3. `cargo test -p mcp-cad-fea --lib --test fea_tests --no-fail-fast`: **36/36 PASS** (`--lib`: `22/22`, `fea_tests`: `14/14`).
4. `cargo test -p mcp-cad-server --lib error::tests --no-fail-fast`: **21/21 PASS** (`0.01s`; динамический `builtin_material_ids()` автоматически синхронизировался со всеми 10 пресетами).
5. `cargo test -p mcp-cad-server --test bench_suite --test gear_kinematic_test --test qa_wide_scan --test mcp_contract_tests --no-fail-fast`:
   - `bench_suite`: **1/1 PASS** (`8.03s`);
   - `gear_kinematic_test`: **2/2 PASS** (`0.84s`);
   - `qa_wide_scan`: **13/13 PASS** (`8.73s`);
   - `mcp_contract_tests`: **61/62 PASS** (`47.65s`; единственное падение — предсуществующая задача `P5-T17` на строке `mcp_contract_tests.rs:4659`).
6. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
7. `npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts`: **73/73 PASS** (`0 failed`).
8. `npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`): **PASS** (финальная проверка после правок).

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)

- **`P5-T20 [S]`**: В `src/components/CadLeftSidebar.tsx:501` подсветка назначенного материала проверяет сырое равенство строк `selectedPart.materialId === mat.id` вместо канонического `activeMaterial.id === mat.id`. При назначении детали через MCP-инструмент или Rust-сессию канонического Rust-ID/алиаса (`aluminum_6061_t6`, `steel_carbon_aisi1045`, `steel_high_strength_4140`, `steel_1020` и др.) ни одна карточка материала в левой панели не подсвечивается активной, хотя `activeMaterial` на строке `108` уже разрешается корректно через `getMaterialById`.
- Дополнительно уточнены описания существующих задач `P5-T17` (счётчик материалов в `project.rs:682` теперь `11` = `10 builtins + 1 custom`) и `P5-T19` (перечень материалов в `create_part` должен включать все 10 встроенных пресетов, включая `steel_carbon_aisi1045` и `borosilicate_glass`).
