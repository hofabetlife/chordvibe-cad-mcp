# Отчёт по задаче P1-T1 — План миграции TS→Rust (Анализ ADR-001)

## 1. Что было проверено и что оказалось правдой / неправдой из описания задачи

- **Заявление в `docs/DECISIONS.md` (ADR-001, ADR-002)**: утверждалось, что Rust-сервер уже является единственным вычислительным ядром, раздаёт статику `dist/` на порту `3000`, поддерживает CLI-флаги `--host` и `--auth-token`, а также предоставляет HTTP/SSE эндпоинты `/api/state`, `/api/events`, `/api/mcp`.
  - **Факт (НЕПРАВДА в `DECISIONS.md`, ПРАВДА в постановке `P1-T1`)**: В `crates/mcp-cad-server/src/sse.rs` (98 строк) реализованы **только два маршрута**: `GET /sse` и `POST /message` на захардкоженном адресе `127.0.0.1:8080`. Маршруты `/api/state`, `/api/events`, `/api/mcp`, раздача `dist/` и флаги `--host`/`--auth-token` в `crates/mcp-cad-server/src/main.rs` отсутствуют.
- **Заявление об объёме дублирующего TS-движка в `fix/P1_architecture.md`**: в постановке задачи указано «~10 000 строк TS-движка (`src/cad/*.ts`)» и в `00_README_PREAMBLE.md` — «18 695 строк в `src/cad/*.ts`».
  - **Факт (УТОЧНЕНО)**:
    - Суммарный объём 9 файлов в `src/cad/*.ts` составляет **9 909 строк** (из них **8 407 строк** рабочего кода движка и **1 502 строки** в `src/cad/cad_engine.test.ts`), плюс **283 строки** типов в `src/types/cad.ts` (итого **10 192 строки**).
    - Число **18 695 строк** — это точная сумма строк вообще всех `.ts`/`.tsx` файлов проекта (`src/cad/*.ts` + `src/components/*.tsx` + `src/App.tsx` + `src/types/cad.ts` + `bench/run_benchmark.ts`).
- **Проверка использования TS-движка в UI (`src/App.tsx`)**:
  - **Факт (ПРАВДА)**: `src/App.tsx` по умолчанию запускается в режиме `ts_fallback` и напрямую вызывает генераторы мешей из `src/cad/primitives.ts`, МКЭ-решатель `runLinearFeaSolver` из `src/cad/fea_solver.ts`, аниматор `stepKinematicsAssembly` из `src/cad/kinematics.ts`, оптимизатор `optimizeCadFeatureTree` из `src/cad/query_optimizer.ts` и локальный диспетчер `executeMcpToolOnEngineState` из `src/cad/mcp_dispatcher.ts`. Даже при включённом `native_windows_rust` (`App.tsx:266-318`) UI после JSON-RPC вызова в Rust всё равно пересчитывает геометрию и поля локально в TypeScript.
- **Сравнение 28 MCP-тулов между Rust и TS**:
  - **Факт (ПРАВДА + выявлены критические расхождения контрактов)**: имена всех 28 инструментов совпадают, но в 12 из 28 инструментов расходятся имена обязательных параметров или семантика:
    - несовместимые ID материалов (`steel_1045`, `alu_6061`, `ti_6al_4v`, `pla_3d` в TS против `steel_structural_aisi1020`, `aluminum_6061_t6`, `titanium_grade5_ti6al4v`, `pla_filament_3dprint` в Rust);
    - `register_material` (`id` в TS vs `name` в Rust) и `set_part_material` (`material_id` в TS vs `material_name` в Rust);
    - `modify_feature_param` и `suppress_feature` (`feature_id: string` в TS vs `node_index: usize` в Rust);
    - `boolean_union` и `boolean_intersection` (`part_a_id`/`part_b_id` с созданием новой детали в TS vs `target_part_id`/`tool_part_id` с модификацией in-place в Rust);
    - `transform_part` (`rotation` в радианах в TS vs `rotation_deg` в градусах в Rust);
    - `run_fea_analysis` (`boundary_conditions` со строками `"+X"/"-X"` в TS vs `fixed_faces`/`loads` с `{ axis, extremum }` в Rust);
    - `create_joint` (`parent_part_id`/`child_part_id`/`type` в TS vs `joint_id`/`part_id_a`/`part_id_b`/`joint_type` в Rust).

## 2. Какие файлы изменены и почему

Задача `P1-T1` — архитектурно-документационная («Только документ, без изменения кода»). Код на Rust и TypeScript не изменялся.

Созданы / обновлены следующие файлы документации:
1. `docs/MIGRATION_TS_TO_RUST.md` *(создан)* — полный документ плана миграции TS→Rust, включающий:
   - Раздел 1: Пофайловую инвентаризацию `src/cad/*.ts` (дублируемые модули и функции в Rust, граф импортов в TS, уникальная клиентская функциональность).
   - Раздел 2: Полную таблицу соответствия всех 28 MCP-тулов (`[есть в Rust / есть в TS / расхождения параметров, поведения и ответов]`).
   - Раздел 3: Спецификацию 9 HTTP/SSE/Binary маршрутов для `crates/mcp-cad-server`, необходимых для работы Thin Web UI без локального TS-движка.
   - Раздел 4: Пошаговый план миграции из 4 атомарных шагов (`P1-T2a` .. `P1-T2d`, каждый размером `<= 1 задача агента`) с чёткими критериями приёмки.
   - Раздел 5: Анализ `[DECISION]` по судьбе `src/cad/cad_engine.test.ts` (1 502 строки, 26 тестов).
2. `docs/agent_tasks/P1-T1-report.md` *(создан)* — настоящий отчёт о выполнении задачи.
3. `TASKS.md` *(обновлён)* — статус `P1-T1` переведён в `✅ DONE` со ссылками на план и отчёт; в бэклог добавлены новые проблемы, обнаруженные в ходе аудита (`P2-T12`).

## 3. Какие тесты добавлены / изменены, результаты прогона

Так как задача `P1-T1` не предполагает изменения кода, существующие тесты не модифицировались. Проведены контрольные прогоны:

- **Rust MCP Contract Tests (`cargo test -p mcp-cad-server --test mcp_contract_tests`)**:
  - Результат: **`5 passed; 0 failed; 0 ignored`**.
- **TypeScript Lint (`npm run lint` / `tsc --noEmit`)**:
  - Результат: **`0 errors`**.
- **TypeScript Unit Tests (`npx --yes tsx --test src/cad/cad_engine.test.ts`)**:
  - Результат: **`25 passed, 1 failed`** (единственное падение — известный из `P0-T1` тест `TS-FIX-01` на строке 507: `FEA_QUICK_LOAD_PRESETS.length` равен `5` после добавления `kirsch_tension`, тогда как тест ожидает `4`).

## 4. Какие новые проблемы найдены по ходу

При аудите кода `crates/mcp-cad-server/src/tools/` и сверке с TS-диспетчером обнаружены 3 новые проблемы в Rust-сервере, которые занесены в `TASKS.md` (задача **`P2-T12`** со статусом `BACKLOG`):

1. **Неполный транзакционный откат в `batch_cad_operations` (`crates/mcp-cad-server/src/tools/cad.rs:1709-1765`)**:
   - При старте батча сохраняются снимки `parts`, `materials`, `last_fea_reports`, `last_fea_meshes`, `last_simulation`, но **не сохраняется и не восстанавливается `state.physics_world`**. Если один из шагов батча модифицирует физический мир (`create_part`, `delete_part`, `create_joint`, `set_joint_motor`, `boolean_union`, `transform_part`), а последующий шаг завершается с ошибкой, `state.parts` откатывается, но `state.physics_world` остаётся в рассогласованном состоянии.
2. **Сброс флага закрепления `part.is_fixed` при обновлении геометрии в `cad.rs` (`crates/mcp-cad-server/src/tools/cad.rs:1464, 1530, 1595, 1809` и `:1052`)**:
   - В `boolean_union` (`:1464`), `boolean_intersection` (`:1530`), `transform_part` (`:1595`) и `optimize_feature_tree` (`:1809`) после перестроения меша вызывается `state.physics_world.add_part(part, true)` с захардкоженным `is_dynamic = true` вместо `!part.is_fixed` (как сделано в `add_feature` на `:908` и `modify_feature_param` на `:978`), из-за чего неподвижная станина (`is_fixed == true`) превращается в падающее динамическое тело. А в `boolean_difference` (`:1052-1170`) обновление `physics_world` для `target_part_id` пропущено вовсе.
3. **Мёртвый дублирующийся модуль `crates/mcp-cad-server/src/tools/export.rs`**:
   - Функция `tools::export::export_stl` (58 строк) полностью дублирует `tools::cad::export_stl` (`cad.rs:1224-1296`), нигде не вызывается (`tools/mod.rs:931` маршрутизирует `"export_stl"` в `cad::export_stl`) и не содержит обязательной проверки `validate_mesh` перед записью файла.

## 5. Решения по пунктам `[DECISION]`

По пункту **`[DECISION]` Судьба `src/cad/cad_engine.test.ts` (1 502 строки, 26 тестов)** в разделе 5 документа `docs/MIGRATION_TS_TO_RUST.md` сформулированы 3 варианта и предложен **рекомендуемый по умолчанию Вариант А**:
- Сохранить в TypeScript компактный набор клиентских тестов (`~200 строк`) для модулей, остающихся во фронтенде (`src/cad/workbenches.ts`, `src/cad/materials.ts`, клиентский транспорт и десериализация буферов в `src/cad/native_bridge.ts`).
- Перенести 3 уникальных сценария (профиль колбы E27 A60, сравнение планов `Naive/RBO/CBO` оптимизатора и транзакционный откат `batch_cad_operations` вместе с `physics_world`) в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`.
- Остальные 23 вычислительных теста TS-ядра удалить на завершающем шаге миграции (`P1-T2d`) вместе с удалением дублирующих модулей `primitives.ts`, `csg_bsp.ts`, `fea_solver.ts`, `kinematics.ts`, `query_optimizer.ts`, `mcp_dispatcher.ts`.
- До подтверждения пользователем файл `src/cad/cad_engine.test.ts` сохранён без изменений.
