# Отчёт по задаче `P4-T0`: Каркас параметрических рецептов (`apply_recipe`) и три базовые заготовки

**Дата:** 15.11.2026  
**Статус:** ✅ Выполнено и верифицировано  

## 1. Цель и объём изменений

Реализован **41-й** инструмент Rust MCP-сервера — **`apply_recipe`**, предназначенный для атомарного построения типовых инженерных деталей и кинематических сборок по именованному рецепту с проверкой инвариантов, валидацией сетки и полным транзакционным откатом:

1. **`crates/mcp-cad-server/src/tools/recipes.rs`**:
   - Реализован `apply_recipe` с транзакционным снимком `StateSnapshot` (при любой ошибке на любом шаге построения или валидации состояние `EngineState` и `physics_world` полностью откатывается к исходному).
   - Поддержаны 3 машиностроительных рецепта:
     - **`flanged_bushing`**: фланцевая втулка со сквозным осевым отверстием и равномерной болтовой разболтовкой на фланце (`flange_position: "bottom" | "top" | "center"`). Коаксиальное тело со сквозным отверстием строится одним чистым кольцевым `revolve`-профилем, затем сверлятся отверстия болтового круга `pcd_diameter` через `cut_hole`. Проверяются геометрические инварианты зазоров до стенки втулки, кромки фланца и между соседними отверстиями.
     - **`stepped_shaft`**: многоступенчатый вал вращения (`revolve` по монотонному несамопересекающемуся профилю `steps: [{diameter, length}]` вдоль оси $Z$) с внешним шпоночным пазом на выбранной ступени (`keyway_step_index`), вырезанным через `boolean_difference` и временный режущий бокс с последующей очисткой. Проверяются хордовое условие глубины шпонки и выход паза за пределы диаметра ступени.
     - **`gear_transmission`**: эвольвентная прямозубая передача (`pinion` + `gear`, выровненная по фазе $180^\circ / z_2$ для чётного числа зубьев ведомого колеса) на общей зафиксированной плите основания (`frame`, `is_fixed: true`), с двумя вращательными шарнирами (`revolute`) на раме, зубчатым зацеплением (`gear`, `teeth_a`, `teeth_b`) и опциональным сервоприводом (`motor_velocity_rad_s`, `motor_max_torque_nm`).
   - Встроены:
     - поддержка `length_unit` (`"mm"` по умолчанию / `"m"` через `state.resolve_length_unit` с пересчётом дефолтов и метаданных `units`);
     - структурированные ошибки `ServerError` -> `ErrorEnvelope` (`result.isError = true`);
     - обязательная проверка каждой созданной детали через `cad::validate_mesh_tool` (запрет открытых/неманифольдных рёбер и незамкнутости);
     - агрегация `state.check_scale_sanity`;
     - вызов `requirements::attach_auto_check_if_enabled` перед возвратом ответа.
2. **`crates/mcp-cad-server/src/tools/mod.rs` и `crates/mcp-cad-server/src/protocol.rs`**:
   - Зарегистрирован модуль `recipes`, описание + строгий JSON Schema для инструмента №41 `apply_recipe` (`Units:`, `Example:`, `Common errors:`, `Next step:`, единицы на каждом числовом/целочисленном/массивном поле), и ветка диспетчеризации в `dispatch_tool`.
   - Актуализирована строка `AGENT_INSTRUCTIONS` (`protocol.rs`, в пределах `800..=1650` символов, с сохранением всех обязательных токенов `P2-T10` / `P2-T39` и упоминанием `apply_recipe`).
3. **`crates/mcp-cad-server/tests/mcp_contract_tests.rs`**:
   - Обновлены контрактные проверки списка инструментов (`41` инструмент, включая `"apply_recipe"`).
   - Добавлен сквозной JSON-RPC тест `test_mcp_p4_t0_apply_recipe_all_recipes_units_errors_and_auto_check`: проверка неизвестного рецепта и нарушения геометрических инвариантов (`ErrorEnvelope` + транзакционный откат), `flanged_bushing`, `stepped_shaft` в `"length_unit": "m"` с `auto_check` требований, `gear_transmission` с мотором, проверка `validate_mesh` (`is_watertight: true`, `boundary_edges_count: 0`) на каждой детали и `analyze_mechanism` (`dof_numeric = 1`, `redundant_constraints = 0`).
4. **Синхронизация счётчиков (`41` Rust инструмент), TS-слоя и документации**:
   - `src/cad/native_bridge.ts` (`RUST_MCP_TOOLS_COUNT = 41`), `src/cad/workbenches.ts`, `src/components/CadRightSidebar.tsx`, `src/cad/cad_engine.test.ts` (`BUG-021`).
   - `README.md`, `PROJECT_CONTEXT.md`, `docs/MCP_API_REFERENCE.md` (оглавление и §41 `apply_recipe`), `docs/CLAUDE_DESKTOP_SETUP.md`, `docs/ANTIGRAVITY_SETUP.md`, `docs/ARCHITECTURE.md`, `docs/AGENT_GUIDE.md` (`<= 160` строк), `SKILL.md`, `run_preview.bat`, `run_preview.ps1`.

## 2. Верификация после финальных правок кода

- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений, `7.40s`, `exit 0`).
- `cargo test -p mcp-cad-server --no-fail-fast`: **140/140 PASS** (`0 failed`):
  - `--lib`: `65/65 PASS` (`61.31s`, включая 4 unit-теста в `tools::recipes::tests`: `flanged_bushing` в `mm` и `m`, `stepped_shaft` со шпоночным пазом, `gear_transmission` + `analyze_mechanism` + `run_physics_simulation` с проверкой передаточного отношения `-0.5`, и транзакционный откат);
  - `mcp_contract_tests`: `60/60 PASS` (`41.51s`, включая `test_mcp_tools_list`, `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`, `test_bug015_create_joint_and_create_part_schema_contract` и `test_mcp_p4_t0_apply_recipe_all_recipes_units_errors_and_auto_check`);
  - `qa_wide_scan`: `13/13 PASS` (`8.52s`);
  - `bench_suite`: `1/1 PASS` (`5.64s`);
  - `gear_kinematic_test`: `1/1 PASS` (`0.57s`).
- `npm run lint` (`tsc --noEmit`): **PASS** (`0` ошибок, `exit 0`).
- `npm run build` (`vite build`): **PASS** (`1592` модулей, `4.16s`, `exit 0`).
- `npm test` (`npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts`): **52/52 PASS** (`31.48s`, `0 failed`, включая обновлённый `BUG-021`).
