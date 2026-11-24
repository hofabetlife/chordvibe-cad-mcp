# Отчёт по задаче P5-T34: Актуализация таблицы входных параметров `create_joint` в `docs/MCP_API_REFERENCE.md`

**Статус:** ✅ Выполнено  
**Дата:** 24.11.2026  

---

## 1. Описание проблемы
При сверке контракта инструмента `create_joint` в `P5-T30` было выявлено, что раздел `### 11. create_joint` в [`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md) (строки `694–769`) расходился с Rust JSON-схемой (`crates/mcp-cad-server/src/tools/mod.rs:388-473`), обработчиком (`crates/mcp-cad-server/src/tools/physics.rs:116-387`) и физическим решателем (`crates/mcp-cad-physics/src/{joint.rs:79, solver.rs:1158}`):
1. `anchor_a` и `anchor_b` были указаны в метрах `(м)`, хотя фактически принимаются в активной `length_unit` (`"mm"` по умолчанию, или `"m"`).
2. `limits` были указаны как `(в радианах или метрах)`, хотя для `"revolute"` принимаются в градусах `deg` по умолчанию (или `rad` через `limits_unit`), а для `"prismatic"` — в активной `length_unit` (`"mm"` по умолчанию, или `"m"`).
3. В таблице входных параметров отсутствовали опциональные поля `length_unit` и `limits_unit`, для `axis` ошибочно утверждалось «обязательна для revolute/prismatic» без указания значений по умолчанию (`[0, 0, 1]` / `[1, 0, 0]`), а в ячейках `axis`, `ratio`, `axis_a`, `axis_b` неэкранированные символы `|` внутри LaTeX (`$|ratio|$`, `$\|v\|$`) ломали разбиение колонок GFM-таблицы.
4. В описании `teeth_a`/`teeth_b` и `ratio` был инвертирован знак: указывалось `i = -z_A / z_B` и «отрицательное значение указывает на встречное вращение», тогда как `physics.rs:315` вычисляет `ratio = teeth_a / teeth_b > 0`, а решатель (`joint.rs:79`, `solver.rs:1158`) накладывает связь $\omega_B \cdot \hat{\mathbf{z}}_B = -\text{ratio}\,(\omega_A \cdot \hat{\mathbf{z}}_A)$ (положительное `ratio` задаёт встречное внешнее зацепление, отрицательное — сонаправленное внутреннее).
5. В примере ответа отсутствовали фактически возвращаемые поля `length_unit` и `units` (`physics.rs:379-383`, `state.rs:90-110`).

> **Примечание о выборе задачи в среде выполнения:** задачи `P5-T21`–`P5-T26` относятся к Rust-крейтам (`mcp-cad-fea`, `mcp-cad-server`) и требуют компиляции/проверки через `cargo test`, тогда как в данном веб-контейнере отсутствует Rust-тулчейн (`cargo`/`rustc`). `P5-T34` — первая открытая задача в `TASKS.md` (зависимость `P5-T30` выполнена), которую можно полностью выполнить и верифицировать в текущем окружении.

---

## 2. Реализованное решение и верификация
1. **[`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md) (`### 11. create_joint`, строки `692–794`)**:
   - Синхронизирована таблица входных параметров (все 14 полей из `mod.rs` + `limits_unit` из `physics.rs`): добавлены `length_unit` и `limits_unit`, исправлены единицы `anchor_a`/`anchor_b` (`"mm"` по умолчанию / `"m"`), `limits` (`deg`/`rad` через `limits_unit` для `"revolute"`, `"mm"`/`"m"` через `length_unit` для `"prismatic"`), значения по умолчанию `axis` (`[0, 0, 1]` / `[1, 0, 0]`), и соглашение знаков для `ratio` и `teeth_a`/`teeth_b` (`ratio = teeth_a / teeth_b`, $\omega_B \cdot \hat{\mathbf{z}}_B = -\text{ratio}\,(\omega_A \cdot \hat{\mathbf{z}}_A)$).
   - Неэкранированные `|` в математических формулах таблицы заменены на `\lvert...\rvert` и `\lVert...\rVert`, исключая разрыв столбцов Markdown-таблицы.
   - Обновлены примеры запросов (вращательный шарнир в `mm`/`deg`, зубчатое зацепление `teeth_a`/`teeth_b`, призматический шарнир с ходом в `mm`) и пример ответа с полным объектом `length_unit` и `units` (17 ключей из `LengthUnit::units_metadata()`).
2. **[`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts)**:
   - Добавлен автоматический контрактный тест `verifies P5-T34: docs/MCP_API_REFERENCE.md create_joint section matches Rust inputSchema, physics.rs units/ratio, and response metadata`, проверяющий:
     - парсинг JSON-схемы `create_joint` из `crates/mcp-cad-server/src/tools/mod.rs` и наличие всех параметров (+ `limits_unit` из `physics.rs`) с точным совпадением обязательности (`Да`/`Нет`);
     - отсутствие устаревших формулировок `(м)`, `в радианах или метрах`, `i = -z_A / z_B` и наличие актуального контракта единиц и знаков `ratio`;
     - валидность всех JSON-RPC примеров запроса и ответа (включая все 17 ключей `units` из `state.rs`);
     - сквозное исполнение примеров `revolute` (`[-180, 180]` deg → `[-π, π]` rad) и `prismatic` (`[-250, 250]` mm → `[-0.25, 0.25]` m) через TS-диспетчер `executeMcpToolOnEngineState`.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T43 [S]`**: В Rust `create_joint` (`crates/mcp-cad-server/src/tools/mod.rs:389-473`, `physics.rs:232-245`): (1) параметр `limits_unit` (`"deg" | "rad"`) поддерживается обработчиком `physics.rs:232`, но отсутствует в `input_schema.properties` и `description` в `mod.rs`; (2) описание `ratio` в `mod.rs:452` указывает инвертированную формулу `(omega_a / omega_b)` вместо фактической $\omega_B \cdot \hat{\mathbf{z}}_B = -\text{ratio}\,(\omega_A \cdot \hat{\mathbf{z}}_A)$ (`joint.rs:79`, `solver.rs:1158`, `physics.rs:315`); (3) `physics.rs:232, 240` не валидирует неизвестные значения `limits_unit`, молча трактуя любую строку кроме `"rad"` как `"deg"`.
- **`P5-T44 [S]`**: В TS `create_joint` (`src/cad/mcp_dispatcher.ts:1457-1502`) отсутствует строгая валидация `limits_unit` (`'deg' | 'rad'`), а также не обрабатываются параметры `teeth_a`/`teeth_b` (вычисление `gearRatio = teeth_a / teeth_b` с проверкой `> 0`) и `axis_a`/`axis_b` для `joint_type: "gear"`.

---

## 4. Затронутые и проверенные файлы
- `docs/MCP_API_REFERENCE.md`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `PROJECT_CONTEXT.md`
- `docs/PROGRESS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T34-report.md`

---

## 5. Результаты проверок и тестов
1. **Целевой тест `P5-T34` (`npx --yes tsx --test --test-name-pattern="P5-T34" src/cad/cad_engine.test.ts`)**:
   - **1/1 PASS** (`0 failed`).
2. **Полный прогон всех 11 TypeScript-сьютов (`cad_engine`, `analyze_mechanism`, `destruction_engine`, `ortho_views`, `field_colormaps`, `coupled_sim_stream`, `helix_primitive`, `live_sync_adapter`, `press_simulation`, `rust_scene_sync`, `live_sync_store`)**:
   - **114/114 PASS** (`0 failed`: `84/84` в 5 базовых + `30/30` в 6 расширенных сьютах).
3. **`npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`)**:
   - **PASS** (`0` ошибок).
4. **Примечание по окружению**: `cargo` и `.git` в текущем контейнере отсутствуют (Rust-код в задаче `P5-T34` не изменялся; git-коммит невозможен без каталога `.git`).
