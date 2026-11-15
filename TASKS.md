# MCP-CAD: План работ и реестр задач (Backlog)

> **Единый источник правды по архитектуре и принятым решениям:** Все фундаментальные архитектурные решения, таблицы систем координат примитивов, семантика Solid DAG и контракты 40 инструментов MCP перенесены и подробно описаны в **[`PROJECT_CONTEXT.md`](PROJECT_CONTEXT.md)** (разделы 4, 5 и 8).  
> **Пошаговые отчёты о выполненных задачах (`P0-T1..T12`, `P1-T1..T4`, `P2-T1..T40`, `P3-T1..T19`):** хранятся в [`docs/agent_tasks/`](docs/agent_tasks/).  
> **Текущий статус верификации (15.11.2026):**  
> - **Проверено в `P3-T19` после финальной правки `physics_tests.rs`, `src/cad/kinematics.ts` и `src/cad/__tests__/analyze_mechanism.test.ts`:**  
>   - `cargo test -p mcp-cad-physics --no-fail-fast`: **23/23 PASS** (`0 failed`, включая `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall`);  
>   - `cargo test -p mcp-cad-server --lib --no-fail-fast -- --test-threads=1` (`MCP_CAD_DISABLE_GUI_SPAWN=1`): **61/61 PASS** (`73.91s`);  
>   - `npm run lint` (`tsc --noEmit`): **0 ошибок** (`exit 0`);  
>   - `npx tsx --test src/cad/__tests__/analyze_mechanism.test.ts`: **15/15 PASS** (`0 failed`);  
>   - `cargo clippy --workspace --all-targets -- -D warnings`: **0 ошибок, 0 предупреждений** во всех 5 крейтах (`exit 0`).  
> - **Проверено ранее в `P3-T18` после финальной правки `spikes/manifold_rust/src/main.rs`:**  
>   - `cargo check --manifest-path spikes/manifold_rust/Cargo.toml --all-targets`: **PASS** (`exit 0`);  
>   - `cargo clippy --manifest-path spikes/manifold_rust/Cargo.toml --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений);  
>   - `cargo test --manifest-path spikes/manifold_rust/Cargo.toml`: **PASS** (`0 passed; 0 failed`, бинарный спайк без unit-тестов; `cargo run` — `exit 0`, все 3 секции `watertight=true`).  
> - **Проверено ранее в `P3-T17` (`MCP_CAD_DISABLE_GUI_SPAWN=1`, `-- --test-threads=1` для `mcp-cad-server`):**  
>   - `cargo test -p mcp-cad-core --no-fail-fast`: **131/131 PASS** (`74` `--lib` + `10` `csg_invariants` + `16` `dag_tests`, включая 2 новых теста `P3-T17` + `25` `primitive_invariants` + `6` `tessellation_tests`);  
>   - `cargo test -p mcp-cad-server --test qa_wide_scan --no-fail-fast -- --test-threads=1`: **13/13 PASS** (`21.22s`);  
>   - прочие таргеты/крейты не перезапускались в `P3-T17`–`P3-T19` (по прошлым проверкам: в `P3-T16` — `mcp_contract_tests` `58/59` [1 известное падение `P3-T20`]; в `P3-T14` — `bench_suite`/`gear_kinematic_test` `2/2`; в `P3-T13` — `mcp-cad-fea` `21/21`, `mcp-cad-viewer` `10/10`).

---

## 1. Актуальный бэклог задач (Active Backlog)

### Сводная таблица открытых задач

| ID | Фаза | Задача | Зависит от | Размер | Компонент | Статус |
|---|---|---|---|---|---|---|
| **P3-T5** | P3 Calculation | Сетка МКЭ, согласованная с границей (`docs/adr/0002-fe-mesher.md`, `spikes/fe_snap/`) **[DECISION]** | P1-T3, P3-T1 | M | Docs / FEA Spikes | ✅ DONE |
| **P3-T6** | P3 Calculation | Извлечение пиковых нагрузок из динамической симуляции (`get_part_load_cases`) | — | M | `mcp-cad-physics` | ✅ DONE |
| **P3-T7** | P3 Calculation | Квазистатический расчёт прочности по нагрузкам из симуляции (`run_fea_from_simulation`) | P3-T6, P2-T8 | L | `mcp-cad-fea`, `physics` | ✅ DONE |
| **P3-T8** | P3 Calculation | Аналитическая проверка прочности зубьев шестерён по Льюису и Герцу/AGMA (`check_gear_strength`) | P2-T2 | M | `mcp-cad-core`, `server` | ✅ DONE |
| **P3-T9** | P3 Calculation | Анализ степеней свободы и заклинивания механизма (`analyze_mechanism`: Грюблер & SVD Якобиана) | — | M | `mcp-cad-physics` | ✅ DONE |
| **P3-T10** | P3 Calculation | Детекция статической интерференции и развёртка хода шарниров (`check_interference`, `sweep_joint`) | P3-T9 | L | `mcp-cad-physics` | ✅ DONE |
| **P3-T11** | P3 Calculation | Водонепроницаемость и качество кромок `cut_helical_groove` на внутренней резьбе гайки (`T08_hex_nut_m10`) и спиральном сверле | P0-T7, P1-T2 | M | `mcp-cad-core` | ✅ DONE |
| **P3-T12** | P3 Calculation | Устранение открытых рёбер BSP-CSG на составной фланцевой втулке и учёт `connected_components` в формуле `genus` (`diagnostics.rs:414`) | P0-T7, P1-T3 | S | `mcp-cad-core` | ✅ DONE |
| **P3-T13** | P3 Calculation | Устранение предупреждений `clippy` на компиляторе `rustc 1.91.1` в `mcp-cad-core` и `mcp-cad-server` | P0-T12 | S | `mcp-cad-core`, `server` | ✅ DONE |
| **P3-T14** | P3 Calculation | Синхронизация длины и обязательных токенов `AGENT_INSTRUCTIONS` (`protocol.rs:173`) с контрактными тестами `test_mcp_initialize` и `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` после расширения `P2-T39` | P2-T10, P2-T39 | S | `mcp-cad-server` | ✅ DONE |
| **P3-T15** | P3 Calculation | Учёт активных упоров (`limits`) шарниров на границе хода и атрибуция избыточных связей (`redundant_joint_ids`) в `analyze_mechanism` | P3-T9 | S | `mcp-cad-physics` | ✅ DONE |
| **P3-T16** | P3 Calculation | Итеративный геометрический решатель замкнутых кинематических цепей при пошаговом сканировании `sweep_joint` на четырёхзвенниках и кривошипно-ползунных механизмах | P3-T10 | S | `mcp-cad-physics` | ✅ DONE |
| **P3-T17** | P3 Calculation | Поддержка несоосных и наклонных винтовых канавок `cut_helical_groove` при произвольной ориентации заготовки вне оси `Z` | P3-T11, P1-T4 | S | `mcp-cad-core`, `server` | ✅ DONE |
| **P3-T18** | P3 Calculation | Актуализация конструктора `TriangleMesh::new` в изолированном спайке `spikes/manifold_rust/src/main.rs:170` после добавления полей `face_ids` и `faces` (`P2-T6`) | P1-T3, P2-T6 | S | `spikes/manifold_rust` | ✅ DONE |
| **P3-T19** | P3 Calculation | Исправление проверки `min_clearance_step` в тесте `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall` (`physics_tests.rs:1632`: `Some(78)` на `[0°, 90°]` и `Some(27)` на `[0°, 60°]`) и паритет TS-зеркала `sweepJoint` | P3-T10 | S | `mcp-cad-physics`, TS | ✅ DONE |
| **P3-T20** | P3 Contract | Актуализация контрактного теста `test_mcp_open_preview_window` (`mcp_contract_tests.rs:716`: статус `"empty"` на пустой сцене после `P2-T37` вместо `"headless"`) | P2-T37 | S | `mcp-cad-server` | 📋 BACKLOG |
| **P3-T21** | P3 Calculation | Актуализация конструктора `TriangleMesh::new` в изолированном спайке `spikes/monstertruck_brep/src/main.rs:60` после добавления полей `face_ids` и `faces` (`P2-T6`) | P1-T3, P2-T6 | S | `spikes/monstertruck_brep` | 📋 BACKLOG |
| **P3-T22** | P0 Hygiene | Восстановление отсутствующих в рабочей копии файлов CI/тулчейна (`rust-toolchain.toml`, `.github/workflows/ci.yml`) и TS-теста `src/cad/cad_engine.test.ts` (либо актуализация ссылок в `README.md`) | P0-T8 | S | Repo / CI / TS | 📋 BACKLOG |
| **P3-T23** | P3 Testing | Устранение зависания многопоточного прогона `mcp_contract_tests` и `qa_wide_scan` на Windows (WGPU/GUI-конкуренция) и изоляция побочных записей `qa/results/*.json` и `qa/artifacts/` | P2-T37 | S | `mcp-cad-server` | 📋 BACKLOG |
| **P4-T0** | P4 Blueprints | Каркас параметрических «рецептов» (`apply_recipe`) и генерация типовых инженерных узлов | P0–P3 | M | `mcp-cad-server` | 📋 BACKLOG |

---

## 2. Детальная спецификация открытых задач

### Фаза 2: Завершение контракта «Агент ↔ Система» и устранение проблем интеграции с Antigravity

> **Контекст задач `P2-T37`..`P2-T39` (по результатам анализа работы агента в Google Antigravity):**  
> При реальном тестировании проекта в Antigravity обнаружено, что внешний ИИ-агент тратил шаги на чтение исходного кода (`primitives.rs`, `dag.rs`, `main.rs`) и сталкивался с двумя блокерами:
> 1. Искал в `primitives.rs` систему координат примитивов (где $Z=0$ и куда направлена вершина конуса).
> 2. Искал в `dag.rs` семантику последовательных вызовов `add_feature_primitive` (перезапись vs авто-объединение `csg_union`).
> 3. Получал отказ `"status": "headless"` от `open_preview_window` при стандартном запуске сервера по `stdio` и шёл в `main.rs` искать CLI-флаг `--stl <path>`.
> 4. Получал `status: "FAIL"` от `validate_mesh` на корректной замкнутой геометрии спирального сверла из-за жёсткого порога `Sliver quality gate` на острых кромках сбега винтовой канавки `cut_helical_groove`.

---

#### P2-T37 [M]: Автономный запуск нативного 3D-окна WGPU в `open_preview_window` из `stdio`-сессии без отказа `"status": "headless"`
- **Проблема**:
  В `crates/mcp-cad-server/src/tools/render.rs:80-140` инструмент `open_preview_window` проверяет `if let Some(proxy) = &state.gui_proxy`. Когда MCP-клиент (Google Antigravity, Claude Desktop, Cursor) запускает сервер со стандартными аргументами `["--mode", "stdio"]` (без `--gui`), `state.gui_proxy` равен `None`, и вызов `open_preview_window` возвращает отказ:
  `{"status": "headless", "message": "Server is running in headless mode without --gui. Interactive window not available, use capture_viewport instead."}`.
  Из-за этого агент в Antigravity лезет изучать `crates/mcp-cad-server/src/main.rs:122`, находит CLI-аргумент `--stl <path>` и вручную вызывает бинарник через shell в обход MCP.
- **Что сделать**:
  1. В `crates/mcp-cad-server/src/tools/render.rs` (`open_preview_window`): когда `state.gui_proxy.is_none()`, вместо возврата отказа `"status": "headless"` собрать целевую полигональную сетку (указанной детали `part_id` либо всех непустых деталей сцены; если сцена пуста — вернуть `VIEWER_EMPTY_MESH` или информативный статус), экспортировать её во временный бинарный STL-файл (`./output/_mcp_preview_live.stl`, с fallback в `std::env::temp_dir().join("mcp_cad_preview_live.stl")`) через `mcp_cad_core::export_stl::export_mesh_stl_binary`.
  2. Если не установлена переменная окружения `MCP_CAD_DISABLE_GUI_SPAWN=1` (для headless CI-тестов), запустить отдельный фоновый процесс текущего исполняемого файла:
     ```rust
     std::process::Command::new(std::env::current_exe()?)
         .arg("--stl")
         .arg(&preview_stl_path)
         .stdin(std::process::Stdio::null())
         .stdout(std::process::Stdio::null())
         .stderr(std::process::Stdio::null())
         .spawn()
     ```
     Это открывает нативное интерактивное окно WGPU на рабочем столе пользователя без блокировки цикла `stdio` JSON-RPC и без необходимости для агента читать `main.rs` или вызывать терминальные команды!
  3. Возвращать в JSON-ответе `{"status": "opened", "mode": "spawned_gui_process", "pid": child.id(), "stl_path": ..., "part_id": ...}` (а если спавн отключён или недоступен в headless-контейнере без дисплея — возвращать `"status": "headless"` с полями `exported_preview_stl` и подсказкой использовать `capture_viewport`).
  4. Обновить `docs/ANTIGRAVITY_SETUP.md` и `docs/CLAUDE_DESKTOP_SETUP.md`: описать как автоматическое открытие окна по вызову `open_preview_window` в стандартном `stdio`-режиме, так и постоянный режим `"args": ["--mode", "stdio", "--gui"]`.
- **Критерии приёмки**:
  - [ ] Вызов `open_preview_window` в `stdio`-режиме без `--gui` экспортирует актуальную сетку детали/сцены и запускает дочерний процесс `--stl` (с защитой `Stdio::null()`, не загрязняющей `stdout`).
  - [ ] В unit- и контрактных тестах (`mcp_contract_tests.rs`) проверена работа `open_preview_window` (с контролируемым тестовым режимом без зависания CI).
- **Компонент**: `crates/mcp-cad-server/src/tools/render.rs`, `crates/mcp-cad-server/src/tools/mod.rs`, `docs/ANTIGRAVITY_SETUP.md`, `docs/CLAUDE_DESKTOP_SETUP.md`

---

#### P2-T38 [M]: Разделение фатальных ошибок топологии (`FAIL`) и предупреждений о микро-гранях (`WARN` / `sliver_triangles`) в `validate_mesh` на производственной геометрии (`cut_helical_groove`)
- **Проблема**:
  В `crates/mcp-cad-core/src/diagnostics.rs:614-648` функция `validate_mesh` считает наличие хотя бы одного треугольника с углом `< 1°` (`lt_1_deg_count > 0`) или долю углов `< 5°` свыше `1.0%` (`lt_5_deg_ratio > MAX_ALLOWED_SLIVER_RATIO`) критической ошибкой в массиве `issues`, что переводит итоговый `status` в **`"FAIL"`**.
  При моделировании реального спирального сверла (`cylinder` + `cut_helical_groove` + `cone`) на острой кромке сбега винтовой канавки в коническую заточку неизбежно возникают узкие переходные треугольники (`sliver triangles`), хотя сама сетка **на 100% замкнута и корректна** (`is_watertight == true`, `is_manifold == true`, `boundary_edges_count == 0`, `non_manifold_edges_count == 0`, `self_intersections_count == 0`, `degenerate_triangles_count == 0`, `is_positive_volume == true`). Из-за статуса `"FAIL"` агент считает деталь бракованной и не может перейти к расчёту МКЭ (`run_fea_analysis`) или сдаче работы.
- **Что сделать**:
  1. В `crates/mcp-cad-core/src/diagnostics.rs`:
     - Разделить проблемы сетки на два списка в `MeshValidationReport`:
       - `issues` (фатальные топологические и геометрические дефекты, дающие `status = "FAIL"`): `!is_watertight`, `!is_manifold`, `!orientation_consistent`, `!is_positive_volume`, `degenerate_triangles_count > 0`, `self_intersections_count > 0`, несовпадение `expected_genus`;
       - `warnings: Vec<String>` (предупреждения углового качества поверхностной триангуляции при полностью замкнутом манифольдном теле): `lt_1_deg_count > 0`, `lt_5_deg_ratio > MAX_ALLOWED_SLIVER_RATIO`, `t_junctions_count > 0`, `coplanar_overlaps_count > 0`.
     - Добавить функцию `validate_mesh_strict(mesh: &TriangleMesh, expected_genus: Option<i32>) -> MeshValidationReport` (или параметр `strict_slivers: bool`):
       - В режиме по умолчанию для MCP-инструмента `validate_mesh` (`strict_slivers = false`), если `issues.is_empty()`, сетка получает `status = "PASS"` (или `"WARN"`, но с явным полем `"is_valid_solid": true` и `"fea_ready": true`), а сообщения о sliver-треугольниках помещаются в `warnings`.
       - При явной передаче `"strict_slivers": true` в `validate_mesh` (и в параметрических тестах генераторов базовых примитивов `primitive_invariants.rs` / `csg_invariants.rs`) сохраняется строгий Sliver Quality Gate (`lt_1_deg_count == 0` и `lt_5_deg_ratio <= 1.0%`), чтобы не допустить деградации качества базовых примитивов.
  2. В `crates/mcp-cad-server/src/tools/requirements.rs` (`RequirementSpec::MeshValid`): считать требование `mesh_valid` выполненным (`PASS`), когда тело водонепроницаемо, манифольдно, не имеет самопересечений и вырожденных граней (`issues.is_empty()`), возвращая в `hint` информацию о `warnings`, если на сложных винтовых канавках присутствуют узкие треугольники.
- **Критерии приёмки**:
  - [ ] Спиральное сверло (`cylinder` + `cut_helical_groove` + `cone`) проходит `validate_mesh` и требование `mesh_valid` со статусом `"PASS"` (`is_watertight == true`, `is_manifold == true`), а информация об узких треугольниках на кромке канавки возвращается в `warnings`.
  - [ ] Все 18 параметрических суитов в `primitive_invariants.rs` и `csg_invariants.rs` продолжают проверять строгий Sliver Quality Gate (`strict_slivers: true`) без ослабления порогов для базовых примитивов.
- **Компонент**: `crates/mcp-cad-core/src/diagnostics.rs`, `crates/mcp-cad-server/src/tools/{cad.rs, mod.rs, requirements.rs}`, `crates/mcp-cad-core/tests/primitive_invariants.rs`

---

#### P2-T39 [M]: Устранение геометрической неопределённости в `initialize.instructions`, `AGENT_GUIDE.md`, `SKILL.md` и `tools/mod.rs` (таблица привязок примитивов, авто-`csg_union` в DAG, спец-инструменты и `GUI vs Headless`)
- **Проблема**:
  Когда пользователь устанавливает проект в Google Antigravity, агент не видит внутренних деталей `primitives.rs` и `dag.rs`. В текущем `AGENT_INSTRUCTIONS` (`crates/mcp-cad-server/src/protocol.rs:173-181`) и `docs/AGENT_GUIDE.md` отсутствуют:
  1. **Точные координатные привязки примитивов**: где находится `origin` $[0,0,0]$, центрированы ли `box`, `cylinder`, `cone`, `polygon_prism`, `spur_gear`, `wheel_rim` по высоте $Z \in [-h/2, +h/2]$, и где у конуса (`cone`) расположено основание `radius_bottom` ($Z = -h/2$) и вершина `radius_top` ($Z = +h/2$).
  2. **Правило вычисления дерева DAG (`FeatureTree`)**: явное указание, что первый `add_feature_primitive` создаёт базовое тело детали, а все последующие `add_feature_primitive` на том же `part_id` **автоматически объединяются через `csg_union`** (не перезаписывая деталь!), тогда как `cut_hole` и `cut_helical_groove` автоматически вычитаются через `csg_difference`.
  3. **Сводка специализированных инструментов**: прямое перечисление возможностей `cut_helical_groove` (спиральные канавки свёрл `circular` и резьба 60° `metric_60`, авто-выравнивание `start_z` по нижней грани детали `bounding_box.min[2]`), `cut_hole` (`through: true`, шпоночный паз `keyway_width`/`keyway_depth`), `spur_gear` (DIN 867), `wheel_rim` и `polygon_prism` (DIN 934).
  4. **Раздел `GUI vs Headless` и запрет чтения исходников**: объяснение разницы между `capture_viewport` (оффскрин Base64 PNG для глаз агента) и `open_preview_window` (нативное 3D-окно WGPU для пользователя), плюс явная директива агенту **не ходить в файлы исходного кода (`crates/**/*.rs`)**, так как все геометрические привязки даны в инструкциях.
- **Что сделать**:
  1. Обновить `AGENT_INSTRUCTIONS` в `crates/mcp-cad-server/src/protocol.rs` (передаваемый клиенту при `initialize`), включив компактную шпаргалку по системам координат всех 8 примитивов (`Z-up`, `[-h/2, +h/2]`, `cone: radius_bottom at -h/2, radius_top at +h/2`), семантике DAG (auto-`csg_union` для последовательных `add_feature_primitive`, auto-`csg_difference` для `cut_hole`/`cut_helical_groove`), специализированным инструментам, `capture_viewport` vs `open_preview_window` и правило «DO NOT read Rust source files in `crates/` — all coordinate conventions are specified here».
  2. Дополнить описания `add_feature_primitive`, `cut_hole`, `cut_helical_groove`, `open_preview_window` и `validate_mesh` в `crates/mcp-cad-server/src/tools/mod.rs` точными диапазонами координат (`Z in [-height/2, +height/2]`, `X in [-width/2, +width/2]`, `Y in [-height/2, +height/2]`, `Z in [-depth/2, +depth/2]` для `box`, `radius_bottom at Z=-height/2, radius_top at Z=+height/2` для `cone`) и указанием на автоматический `csg_union`.
  3. Обновить `docs/AGENT_GUIDE.md` и добавить готовый файл навыка `SKILL.md` в корень/документацию проекта (с упоминанием в `docs/ANTIGRAVITY_SETUP.md`), содержащий все 3 запрошенных в ревью раздела:
     - Таблицу примитивов и их координатных привязок;
     - Таблицу специализированных машиностроительных инструментов (`cut_helical_groove`, `cut_hole`, `spur_gear`, `wheel_rim`, `polygon_prism`);
     - Раздел `GUI vs Headless` (`open_preview_window`, `capture_viewport`, `--gui`, `--stl`).
- **Критерии приёмки**:
  - [ ] Ответ `initialize.instructions` и `docs/AGENT_GUIDE.md` содержат точные координатные диапазоны всех 8 примитивов, правило auto-`csg_union` в DAG, список спец-инструментов и раздел `GUI vs Headless`.
  - [ ] Контрактный тест в `crates/mcp-cad-server/tests/mcp_contract_tests.rs` проверяет наличие ключевых координатных инвариантов (`[-height/2, +height/2]`, `csg_union`, `radius_bottom`) в `initialize.instructions` и `tools/list`.
- **Компонент**: `crates/mcp-cad-server/src/{protocol.rs, tools/mod.rs}`, `docs/AGENT_GUIDE.md`, `docs/ANTIGRAVITY_SETUP.md`, `SKILL.md`

---

#### P2-T40 [S]: Сохранение `scope: "scene"` / `part_id: null` в `check_requirements` при 1 детали в сцене и вызов `attach_auto_check_if_enabled` в `delete_part`
- **Проблема**: Выявлено при выполнении **P2-T36** ([`docs/agent_tasks/P2-T36-report.md`](docs/agent_tasks/P2-T36-report.md)):
  1. В `crates/mcp-cad-server/src/tools/requirements.rs` (`evaluate_single_requirement`, строки 409, 502, 588, 672) при проверке требования с `scope: "scene"` (`spec.part_id.is_none()` / `effective_part_id.is_none()`), когда в сцене ровно 1 деталь (`state.parts.len() == 1`), функция `resolve_target_part(None, state)` (строки 894–901) возвращает эту единственную деталь, а ветки агрегации в `MassMax` и `MeshValid` проверяют `effective_part_id.is_none() && state.parts.len() > 1`. В результате требования уровня сцены (`bbox_max`, `bbox_min`, `mass_max`, `mesh_valid`) при одной детали в сцене возвращают `"part_id": "<single_part_id>"` и `"scope": "<single_part_id>"` вместо `"part_id": null, "scope": "scene"`.
  2. В `crates/mcp-cad-server/src/tools/cad.rs:391-420` (`delete_part`) при удалении детали из сцены не вызывается `super::requirements::attach_auto_check_if_enabled(state, None, &mut resp)` при активном `scene_auto_check`.
- **Что сделать**:
  1. В `crates/mcp-cad-server/src/tools/requirements.rs` для требований с `effective_part_id.is_none()` (уровень сцены `"scene"`, когда они берутся из `state.requirements.scene_requirements`) сохранять `"part_id": null` и `"scope": "scene"` вне зависимости от того, 1 деталь в сцене или несколько (`!state.parts.is_empty()`).
  2. В `delete_part` (`crates/mcp-cad-server/src/tools/cad.rs`) вызывать `super::requirements::attach_auto_check_if_enabled(state, None, &mut resp)` перед возвратом ответа.
- **Компонент**: `crates/mcp-cad-server/src/tools/{requirements.rs, cad.rs}`

---

### Фаза 3: Достоверность расчётов прочности и механизмов (P3)

#### P3-T1 [M]: Базовая точность FEA: эталонные задачи
- **Что сделать**: Создать `crates/mcp-cad-fea/tests/fea_benchmarks.rs` и `docs/FEA_ACCURACY.md` с замерами относительных ошибок на 4 аналитических эталонах сопротивления материалов и теории упругости:
  1. Осевое растяжение цилиндрического стержня ($\sigma = F/A$, $\Delta L = \frac{FL}{EA}$);
  2. Поперечный изгиб консольной балки ($\delta = \frac{FL^3}{3EI}$, $\sigma_{\max} = \frac{FL \cdot c}{I}$);
  3. Пластина Кирша с центральным круговым отверстием (коэффициент концентрации напряжений $K_t \approx 3.0$);
  4. Толстостенный цилиндр под внутренним давлением (задача Ламе).
- **Компонент**: `crates/mcp-cad-fea/tests/fea_benchmarks.rs`, `docs/FEA_ACCURACY.md`

#### P3-T2 [M]: Предполётные проверки и баланс сил FEA — ✅ DONE ([`P3-T2-report.md`](docs/agent_tasks/P3-T2-report.md))
- **Выполнено**:
  1. Автоматическая предполётная проверка `mcp_cad_core::validate_mesh` в `TetrahedralMesh::from_mesh` перед запуском МКЭ-решателя со структурированным кодом ошибки `MESH_INVALID` (`FeaError::InvalidSurfaceMesh`).
  2. Вычисление вектора узловых реакций опор $\mathbf{R}_n = (K u)_n - \mathbf{f}_n$, проверка невязки статического равновесия сил (`equilibrium_error_pct < 1e-4` на растяжении и консоли) и моментов относительно ЦМ, а также проверка полноты закрепления 6 мод жёсткого тела (`constrained_rigid_body_modes_ok`).
  3. Детектирование 1-элементной зоны сингулярности закрепления (`critical_node_near_constraint`), расчёт `max_stress_excluding_constraint_zone`, метрик качества тетраэдров `element_quality: {min, mean}`, поля `confidence: "high" | "medium" | "low"` и `diagnostics` в `FeaReport` и `run_fea_analysis`.
  4. Исправлен нижний порог шага сетки в `MeshDensity::target_element_size` (`1e-5` м вместо `1e-4` м), обеспечивающий различное разрешение `Coarse`/`Medium`/`Fine` на тонкостенных деталях (`min_dim <= 5 мм`).
- **Компонент**: `crates/mcp-cad-fea/src/{solver.rs, tetra_mesh.rs, result.rs, error.rs}`, `crates/mcp-cad-server/src/{tools/fea.rs, error.rs}`

#### P3-T3 [M]: Оценка сходимости в результате FEA (`convergence_study: true`) — ✅ DONE ([`P3-T3-report.md`](docs/agent_tasks/P3-T3-report.md))
- **Выполнено**:
  1. Модуль `crates/mcp-cad-fea/src/convergence.rs` подключён в `crates/mcp-cad-fea/src/lib.rs` и интегрирован в `run_fea_analysis` (`crates/mcp-cad-server/src/tools/fea.rs`, `tools/mod.rs`) при передаче `"convergence_study": true`, опционального массива `"levels"` (по умолчанию `["coarse", "medium", "fine"]`) и лимита времени `"max_seconds"`.
  2. Реализованы пороги `CONVERGENCE_DISP_TOLERANCE_PCT = 5.0%`, `CONVERGENCE_STRESS_TOLERANCE_PCT = 10.0%`, `MIN_EFFECTIVE_REFINEMENT_RATIO = 1.25` (защита от ложной сходимости при упоре воксельной сетки в потолок `60` ячеек по оси) и `CURVED_STAIRCASE_STRESS_RATIO_THRESHOLD = 1.50` (детекция ступенчатого воксельного концентратора напряжений на криволинейной границе/отверстии), а также двухшаговая экстраполяция Ричардсона при монотонной асимптотической сходимости.
  3. В `FeaReport` и ответ `run_fea_analysis` добавлен блок `accuracy`; при вердикте `NOT_CONVERGED` (включая `partial_result: true` по таймауту `max_seconds`) поле `confidence` автоматически переводится в `"low"`.
- **Компонент**: `crates/mcp-cad-fea/src/{convergence.rs, result.rs, lib.rs}`, `crates/mcp-cad-server/src/tools/{fea.rs, mod.rs}`

#### P3-T4 [L]: Элемент TET10 (квадратичные 10-узловые тетраэдры) — ✅ DONE ([`P3-T4-report.md`](docs/agent_tasks/P3-T4-report.md))
- **Выполнено**:
  1. Реализован 10-узловой квадратичный изопараметрический тетраэдр `Tet10Element` (`crates/mcp-cad-fea/src/element.rs`: 4 угловых узла `0..4` и 6 mid-edge узлов `4..10` в порядке `[(0,1), (1,2), (2,0), (0,3), (1,3), (2,3)]`, квадратичные функции формы $N_i = L_i(2L_i-1)$, $N_{4+e} = 4L_a L_b$, матрица $B(\mathbf{L})$ $6\times 30$, 4-точечная квадратура Гаусса 2-го порядка с узлами $a = \frac{5+3\sqrt{5}}{20}$, $b = \frac{5-\sqrt{5}}{20}$, положительными весами $w_q = V/4$ и экстраполяцией напряжений из 4 точек Гаусса в 10 узлов).
  2. Реализована конвертация `TetrahedralMesh::to_tet10()` без дублирования mid-edge узлов на общих рёбрах (`crates/mcp-cad-fea/src/tetra_mesh.rs`), согласованные квадратичные веса поверхностных сил Неймана для 6-узловых граней (`crates/mcp-cad-fea/src/boundary.rs`), сборка глобальной матрицы жёсткости $30\times 30$ и вычисление реакций опор (`crates/mcp-cad-fea/src/solver.rs`, `result.rs`).
  3. Добавлен параметр `element_order: 1 | 2` (по умолчанию `1`) в `run_fea_analysis` (`crates/mcp-cad-server/src/tools/{fea.rs, mod.rs}`). На эталонной консольной балке (`200×20×20 мм`, $F_y = -1000\text{ Н}$) ошибка прогиба на плотности `Medium` снижена с **`20.14%`** (`TET4`) до **`0.25%`** (`TET10`, $\le 5.0\%$), на `Coarse` — с **`48.15%`** до **`0.06%`**, а ошибка изгибного напряжения $\sigma(0.5h)$ на `Medium` — с **`28.21%`** до **`0.09%`**.
- **Компонент**: `crates/mcp-cad-fea/src/{element.rs, solver.rs, tetra_mesh.rs, boundary.rs, result.rs, convergence.rs, lib.rs}`, `crates/mcp-cad-server/src/tools/{fea.rs, mod.rs}`, `docs/FEA_ACCURACY.md`

#### P3-T5 [M]: Исследование: сетка, согласованная с границей **[DECISION]** — ✅ DONE ([`P3-T5-report.md`](docs/agent_tasks/P3-T5-report.md))
- **Выполнено**:
  1. Подготовлен архитектурный документ `docs/adr/0002-fe-mesher.md` с анализом 5 альтернатив по 10 критериям, сравнительной матрицей, лицензионным аудитом (AGPL/GPL copyleft против Apache-2.0/MIT) и обоснованным решением о выборе Pure Rust генератора `fe_snap` + `TET10`.
  2. Разработан работающий прототип `spikes/fe_snap/` (`Cargo.toml`, `src/main.rs`, `README.md`) со снаппингом граничных узлов на CAD-поверхность `TriangleMesh`, барьером качества ($J_s \ge 0.06$, $\det J > 0$), backtracking line-search, spatial-ускорителем `SurfaceSpatialIndex` (3D AABB grid), лапласовским сглаживанием внутренних узлов и квадратичным проецированием `TET10` с контролем $\det J(\mathbf{L}^{(q)}) > 10^{-12}$ во всех 4 точках Гаусса.
  3. Проведены замеры на 3 телах: Body 1 (пластина с отверстием, снижение ошибки $K_t$ с 18.39% до 16.23%), Body 2 (ступенчатый вал, снижение RMS отклонения границы на 30.0% за 156 мс), Body 3 (шестерня, снижение RMS отклонения границы эвольвенты на 73.5%).
- **Компонент**: `docs/adr/0002-fe-mesher.md`, `spikes/fe_snap/`, `docs/FEA_ACCURACY.md`

#### P3-T6 [M]: Нагрузки из симуляции: извлечение (`get_part_load_cases`) — ✅ DONE ([`P3-T6-report.md`](docs/agent_tasks/P3-T6-report.md))
- **Выполнено**:
  1. В `mcp-cad-physics` (`simulator.rs`, `telemetry.rs`, `joint.rs`) реализовано точное восстановление 6-компонентного силового винта реакции (`force`, `torque`) в шарнирах Rapier3D (`GenericJoint`) с учётом TGS-Soft подшагов (`substep_dt = dt / num_substeps`), 1-body инверсии для статических тел (`!is_dynamic()`), полууглового якобиана кватернионов (`diff_conj1_2`) и поправки Модифицированного процесса Грама–Шмидта для чистого момента заделки относительно якоря шарнира.
  2. Реализован метод `PhysicsWorld::extract_part_load_case(summary, part_id, selector)` с выбором кадра `"peak_force"`, `"peak_torque"` или произвольного момента времени `time_s`, переводом реакций и ускорений в локальную СК детали (с соблюдением III закона Ньютона $\mathbf{F}_{A \leftarrow B} = -\mathbf{F}_{B \leftarrow A}$, $\boldsymbol{\tau}_{A \leftarrow B} = -\boldsymbol{\tau}_{B \leftarrow A}$), учётом переносного центростремительного ускорения $\boldsymbol{\omega} \times \mathbf{v}$ и расчётом невязки динамического равновесия Даламбера (`equilibrium_residual_force_norm`, `equilibrium_residual_torque_norm`).
  3. Добавлен 35-й MCP-инструмент `get_part_load_cases` в `mcp-cad-server` с поддержкой `length_unit` (`"mm"` / `"m"`). Написаны эталонные тесты консольной балки под тяжестью и вращающегося стержня (`centripetal reaction force`), а также контрактный тест JSON-RPC.
- **Компонент**: `crates/mcp-cad-physics/src/{joint.rs, simulator.rs, telemetry.rs}`, `crates/mcp-cad-server/src/tools/{physics.rs, mod.rs}`

#### P3-T7 [L]: Нагрузки из симуляции: приложение к FEA (`run_fea_from_simulation`) — ✅ DONE ([`P3-T7-report.md`](docs/agent_tasks/P3-T7-report.md))
- **Выполнено**:
  1. В `mcp-cad-fea` (`boundary.rs`, `result.rs`, `solver.rs`) реализованы функции `match_joint_anchor_to_cad_face` (автоматическая привязка якоря шарнира `anchor_local` к именованной грани `FaceInfo` или сферической окрестности `load_radius`), `compute_consistent_face_moments` и `distribute_force_and_moment_with_moments` (распределение силы $\mathbf{F}_j$ и момента $\boldsymbol{\tau}_j$ парой распределённых сил по согласованным виртуальным моментам функций формы первого порядка для `TET4` и `TET10` с точным соблюдением $\sum \mathbf{f}_i = \mathbf{F}_j$ и $\sum (\mathbf{x}_i - \mathbf{p}_j) \times \mathbf{f}_i = \boldsymbol{\tau}_j$), `select_321_minimal_support_constraints` (минимальное трёхузловое закрепление 6 степеней свободы по схеме **3-2-1** без искажения поля напряжений) и `build_simulation_boundary_conditions` (распределение объёмных сил тяжести и инерции Даламбера $m(\mathbf{g}_{\text{local}} - \mathbf{a}(\mathbf{x}))$ с точной проекцией невязки 6-DOF равновесия).
  2. В `solve_pcg` (`crates/mcp-cad-fea/src/solver.rs`) скорректирован порог останова по $\mathbf{p}^T A \mathbf{p}$ (`1e-300`), устраняющий преждевременный `SolverConvergenceFailed` при малых центробежных нагрузках в единицах СИ.
  3. Добавлен 36-й MCP-инструмент `run_fea_from_simulation` (`crates/mcp-cad-server/src/tools/{fea.rs, mod.rs}`), возвращающий полный отчёт FEA с блоком `source: { simulation_time_s, joints: [...] }` и интегрированный со слоем требований `check_requirements` (`min_safety_factor`). На консольной балке `P3-T6(a)` ($L=0.20\text{ м}$, $b=h=0.02\text{ м}$, сталь) получено напряжение у корня `0.44812 МПа` против аналитического `0.46205 МПа` (ошибка **`3.01%`**) при `equilibrium_error_pct < 1e-6%`.
- **Компонент**: `crates/mcp-cad-fea/src/{boundary.rs, result.rs, solver.rs, lib.rs}`, `crates/mcp-cad-server/src/tools/{fea.rs, mod.rs}`

#### P3-T8 [M]: Аналитическая проверка прочности зубьев шестерён по Льюису и Герцу/AGMA (`check_gear_strength`) — ✅ DONE ([`P3-T8-report.md`](docs/agent_tasks/P3-T8-report.md))
- **Выполнено**:
  1. В `mcp-cad-core` создан модуль `gear_strength.rs` с функциями расчёта формы зуба по Льюису (кусочно-линейная интерполяция таблицы 14-2 Shigley для 20° с $z=17 \to 0.303, z=20 \to 0.322$), динамического коэффициента скорости Барта в единицах СИ ($K_v = (6.1+V)/6.1$ для фрезерованных зубьев и $(15.2+V)/15.2$ для шлифованных), напряжений изгиба $\sigma_b = \frac{K_v W_t}{b m Y}$, упругого коэффициента Герца $C_p = \sqrt{1 / (\pi ((1-\nu_1^2)/E_1 + (1-\nu_2^2)/E_2))}$ и контактных напряжений питтинга $\sigma_H = C_p \sqrt{\frac{K_v W_t}{b \cos\alpha}(1/r_1 + 1/r_2)}$.
  2. В `mcp-cad-server` реализован 37-й MCP-инструмент `check_gear_strength` (`crates/mcp-cad-server/src/tools/{cad.rs, mod.rs}`), поддерживающий как автономный расчёт по параметрам (`module`, `teeth`, `face_width`, `torque` или `power_kw` + `speed_rpm`), так и расчёт по существующей детали в сцене `part_id` с извлечением параметров из DAG `AddSpurGear` и материала, с полным контрактом единиц `length_unit` (`"mm"` / `"m"`).
  3. На эталонной передаче ($m=3\text{ мм}, z=20, b=30\text{ мм}, T=100\text{ Н}\cdot\text{м}, K_v=1$) получено напряжение изгиба $\sigma_b = 115.02\text{ МПа}$ (критерий приёмки $\approx 115.0\text{ МПа}$), коэффициент запаса $\text{FoS}_b = 2.17$, контактное напряжение $\sigma_H = 906.1\text{ МПа}$. Написаны unit-тесты в ядре и сервере, а также сквозной контрактный тест JSON-RPC в `mcp_contract_tests.rs`.
- **Компонент**: `crates/mcp-cad-core/src/{gear_strength.rs, lib.rs}`, `crates/mcp-cad-server/src/tools/{cad.rs, mod.rs}`, `docs/agent_tasks/P3-T8-report.md`

#### P3-T9 [M]: Механизм: степени свободы (`analyze_mechanism`: Грюблер & SVD) — ✅ DONE ([`P3-T9-report.md`](docs/agent_tasks/P3-T9-report.md))
- **Выполнено**:
  1. В `mcp-cad-physics` (`joint.rs`, `simulator.rs`) реализованы `MechanismAnalysisReport`, `CadJointType::constraints_count()`, `CadJointType::fill_jacobian_rows()` и `PhysicsWorld::analyze_mechanism_with_scene_parts` с расчётом пространственной подвижности по формуле Чебышёва–Грюблера–Кутцбаха $M = 6(N-1) - \sum c_i$ (`Fixed`=6, `Revolute`=5, `Prismatic`=5, `Spherical`=3, `Gear`=1) и численного ранга матрицы Якоби связей через SVD (`nalgebra`): $\text{DOF}_{\text{numeric}} = 6 N_{\text{moving}} - \text{rank}(J)$, $\text{redundant\_constraints} = \max(0, \text{DOF}_{\text{numeric}} - \text{DOF}_{\text{grubler}})$.
  2. Реализован обход графа связности механизма от неподвижных звеньев (`grounded_parts`) с определением `floating_parts` и `unconnected_parts`, а также генерацией предупреждений (`"DOF=0 при заявленном механизме"`, `"DOF>1 без двигателей"`, `"есть свободные детали"`).
  3. Добавлен 38-й MCP-инструмент `analyze_mechanism` (`crates/mcp-cad-server/src/tools/{physics.rs, mod.rs}`), активирована проверка требования `mechanism_dof` в `check_requirements` (`tools/requirements.rs`), подключена автопроверка `attach_auto_check_if_enabled` в `create_joint` и `set_joint_motor`, а также добавлена зеркальная реализация и модульные тесты `node:test` в `src/cad/{kinematics.ts, mcp_dispatcher.ts, __tests__/analyze_mechanism.test.ts}`. Все 7 эталонных схем (вращательный = 1, жёсткий = 0, сферический = 3, свободное тело = 6, плоский четырёхзвенник = 1 при Грюблере -2 и 3 избыточных связях, кривошипно-ползунный = 1 при Грюблере -2 и 3 избыточных связях, два зубчатых колеса = 1) проходят на 100%.
- **Компонент**: `crates/mcp-cad-physics/src/{joint.rs, simulator.rs}`, `crates/mcp-cad-server/src/tools/{physics.rs, requirements.rs, mod.rs}`, `src/cad/kinematics.ts`, `docs/agent_tasks/P3-T9-report.md`

#### P3-T10 [L]: Механизм: интерференция и ход (`check_interference`, `sweep_joint`) — ✅ DONE ([`P3-T10-report.md`](docs/agent_tasks/P3-T10-report.md))
- **Выполнено**:
  1. В `mcp-cad-physics` (`telemetry.rs`, `simulator.rs`) реализованы структуры `PairInterferenceResult`, `InterferenceReport`, `SweepStepResult`, `JointSweepReport` и методы `PhysicsWorld::check_interference` / `sweep_joint` с двухфазной проверкой коллизий (broad-phase `AABB` + narrow-phase `csg_intersection` и точное евклидово расстояние треугольник–треугольник `compute_mesh_clearance`), поддержкой `ignore_adjacent: bool` и пошаговым сканированием хода шарниров `Revolute` / `Prismatic` без мутации состояния сцены.
  2. В `mcp-cad-server` добавлены MCP-инструменты №39 `check_interference` и №40 `sweep_joint` (`tools/physics.rs`, `tools/mod.rs`), активирована проверка требования `no_interference` в `check_requirements` (`tools/requirements.rs`), а также исправлен баг фильтрации `scene_requirements` при передаче `part_id_filter = Some(pid)` в `evaluate_requirements_report`.
  3. Добавлена зеркальная реализация и 4 теста `node:test` в `src/cad/{kinematics.ts, mcp_dispatcher.ts, __tests__/analyze_mechanism.test.ts}`, 3 теста в `physics_tests.rs` и сквозной контрактный тест в `mcp_contract_tests.rs`. Время выполнения `check_interference` на 10 деталях (45 пар) составляет **`55.79 мс`** (TS) / **`< 15 мс`** (Rust).
- **Компонент**: `crates/mcp-cad-physics/src/{simulator.rs, telemetry.rs}`, `crates/mcp-cad-server/src/tools/{physics.rs, requirements.rs, mod.rs}`, `src/cad/kinematics.ts`, `docs/agent_tasks/P3-T10-report.md`

#### P3-T11 [M]: Водонепроницаемость и качество кромок `cut_helical_groove` на внутренней резьбе гайки (`T08_hex_nut_m10`) и спиральном сверле — ✅ DONE ([`P3-T11-report.md`](docs/agent_tasks/P3-T11-report.md))
- **Выполнено**:
  1. В `crates/mcp-cad-core/src/primitives.rs` реализован конформный переход удвоения колец 1-к-2 `add_doubling_ring_transition_2d` (позволяющий переходить от $N$-угольного внутреннего кольца фаски призмы к $2N$-, $4N$- и $8N$-гранной сетке резьбы без разбиения внешних рёбер оболочки и без образования T-junctions) и добавлена передача точных аналитических параметров винтового профиля `helix_meta:...` в `create_helical_cutter_with_settings`.
  2. В `crates/mcp-cad-core/src/csg.rs` реализованы манифольдные цилиндрические мешеры `try_cut_coaxial_through_bore`, `try_union_coaxial_cone` и `try_cut_coaxial_helical_groove` (для внутренней резьбы гаек/втулок с родом `genus = 1` и наружных винтовых канавок свёрл/винтов с родом `genus = 0`, включая свёрла с конической вершиной в любом порядке операций DAG) с адаптивным осевым шагом `nz`, плавным осевым сбегом канавки у торцов и 3D spatial-grid ускорением `col_grid` в `polygons_to_mesh`.
  3. На задаче `T08_hex_nut_m10` (в СИ и в мм DIN 934), спиральном сверле с конической заточкой и обоих суитах `qa_wide_scan` (`cut_helical_groove_circular`, `cut_helical_groove_metric_60`) достигнуты `validate_mesh == PASS`, `is_watertight == true`, `boundary_edges_count == 0`, `non_manifold_edges_count == 0`, `self_intersections_count == 0`, `lt_1_deg_count == 0`, `lt_5_deg_ratio == 0.00%` и точное совпадение `genus` (`1` для гайки, `0` для сверла).
- **Компонент**: `crates/mcp-cad-core/src/{primitives.rs, csg.rs}`, `crates/mcp-cad-core/tests/dag_tests.rs`, `docs/agent_tasks/P3-T11-report.md`

#### P3-T12 [S]: Открытые рёбра BSP-CSG на составной фланцевой втулке и учёт `connected_components` в формуле `genus` (`diagnostics.rs:414`) — ✅ DONE ([`P3-T12-report.md`](docs/agent_tasks/P3-T12-report.md))
- **Выполнено**:
  1. В `crates/mcp-cad-core/src/diagnostics.rs` вычисление топологического рода `genus` перенесено после BFS-подсчёта `connected_components` ($k$) и приведено к обобщённой формуле Эйлера–Пуанкаре $g = (2k - \chi) / 2$ (при $k > 0$); добавлен модульный тест `test_validate_mesh_disjoint_union_connected_components_genus` на $k = 2$ и $k = 3$ несвязных оболочек.
  2. В `crates/mcp-cad-core/src/csg.rs` в `repair_manifold_triangles` снято ограничение `max_passes = 2` на сетках $> 2000$ треугольников и добавлена очистка дублирующих лепестковых треугольников (`is_dup_flap`), в `refine_coplanar_sliver_fans_and_edges` устранён досрочный `break` на первом нередуцируемом стыковом треугольнике за счёт реестра `unrefinable_tris` и предфильтрации `plane_verts`, а в `try_cut_coaxial_through_bore` поддержано вычитание соосного цилиндра из составных соосных тел.
  3. Добавлен интеграционный тест `p3_t12_flanged_bushing_union_and_difference_watertight` в `crates/mcp-cad-core/tests/csg_invariants.rs` (центрированный, смещённый и компланарный с нижним торцом фланец, а также чистый BSP-CSG на сетках без `face_ids`): все конфигурации проходят `validate_mesh_strict` со статусом `"PASS"`, `is_watertight == true`, `boundary_edges_count == 0`, `non_manifold_edges_count == 0` и `genus == 1.0`.
- **Компонент**: `crates/mcp-cad-core/src/{diagnostics.rs, csg.rs}`, `crates/mcp-cad-core/tests/csg_invariants.rs`, `docs/agent_tasks/P3-T12-report.md`

#### P3-T13 [S]: Устранение предупреждений `clippy` на компиляторе `rustc 1.91.1` в `mcp-cad-core` и `mcp-cad-server` — ✅ DONE ([`P3-T13-report.md`](docs/agent_tasks/P3-T13-report.md))
- **Проверено и подтверждено (дефект не воспроизводится, исправления уже присутствуют в коде)**:
  1. `crates/mcp-cad-core/src/csg.rs:1201, 1216, 1326`: псевдоним `type EarClippedTriangle = ([u32; 3], Vector3<f64>, u32);` уже выделен и используется в `ear_clip_planar_ring_maxmin` и `try_remove_vertex_ear_clip` (0 ошибок `clippy::type_complexity`).
  2. `crates/mcp-cad-server/src/tools/metrics.rs:90-92` (выравнивание doc-комментария `get_scene_summary`), `cad.rs` (0 `clippy::bool_assert_comparison`) и `physics.rs:1367` (`std::f64::consts::TAU` вместо `6.28`): 0 предупреждений Clippy.
  3. На `rustc 1.91.1 (ed61e7d7e 2025-11-07)` команда `cargo clippy --workspace --all-targets -- -D warnings` проходит с кодом `0` (**0 ошибок, 0 предупреждений** во всех 5 крейтах без `#![allow(...)]` на уровне крейтов).
- **Компонент**: `crates/mcp-cad-core/src/csg.rs`, `crates/mcp-cad-server/src/tools/{metrics.rs, cad.rs, physics.rs}`, `docs/agent_tasks/P3-T13-report.md`

#### P3-T14 [S]: Синхронизация длины и обязательных токенов `AGENT_INSTRUCTIONS` (`protocol.rs:173`) с тестами `test_mcp_initialize` и `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` — ✅ DONE ([`P3-T14-report.md`](docs/agent_tasks/P3-T14-report.md))
- **Проверено и подтверждено (синхронизация присутствует в коммите `281a3ed`)**:
  1. `crates/mcp-cad-server/src/protocol.rs:173-193`: константа `AGENT_INSTRUCTIONS` сжата до `1569` символов (в пределах `800..=1650`) и содержит все обязательные токены `P2-T10` (`docs/AGENT_GUIDE.md`, `get_scene_summary`, `length_unit`, `set_requirements`, `auto_check`, `batch_cad_operations`, `validate_mesh`, `run_fea_analysis`, `query_part_geometry`, `STALE`, `isError`) и `P2-T39` (`SKILL.md`, `crates/`, `[-height/2, +height/2]`, `csg_union`, `csg_difference`, `radius_bottom`, `radius_top`, `cut_helical_groove`, `capture_viewport`, `open_preview_window`).
  2. Контрактные тесты `test_mcp_initialize` (`mcp_contract_tests.rs:86-123`) и `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` (`mcp_contract_tests.rs:4495-4633`) проходят полностью (`1 passed; 0 failed` каждый).
- **Компонент**: `crates/mcp-cad-server/src/protocol.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/agent_tasks/P3-T14-report.md`

#### P3-T15 [S]: Учёт активных упоров (`limits`) шарниров на границе хода и атрибуция избыточных связей (`redundant_joint_ids`) в `analyze_mechanism` — ✅ DONE ([`P3-T15-report.md`](docs/agent_tasks/P3-T15-report.md))
- **Реализовано**:
  1. **Атрибуция избыточных связей (`redundant_joint_ids`)**: в `mcp-cad-physics` (`joint.rs`, `simulator.rs`) и TypeScript-зеркале (`src/cad/kinematics.ts`) при `redundant_constraints > 0` выполняется проверка ранга Якобиана без строк каждого шарнира (`leave-one-joint-out` SVD с тем же порогом `rank_tolerance`); каждый шарнир с линейно зависимыми строками включается в `redundant_joint_ids` (независимые шарниры исключены, порядок детерминирован).
  2. **Диагностика упоров `limits` (`joints_at_limits`)**: для `Revolute` (с развёрткой угла к центру диапазона по $2\pi$, в `deg`) и `Prismatic` (смещение вдоль оси в `length_unit` `mm`/`m`) вычисляются границы `min` / `max` / `below_min` / `above_max` / `locked` и флаг `motor_blocked`; формируются предупреждения `warnings` с указанием `limits` (при одностороннем упоре сохраняется `dof_numeric`, при нулевом диапазоне `limits: [c, c]` добавляется 6-я двусторонняя строка Якобиана, уменьшающая `dof_numeric`).
  3. **Синхронизация MCP-контракта и документации**: добавлены поля `redundant_joint_ids`, `joints_at_limits` и параметр `length_unit` в `crates/mcp-cad-server/src/tools/{physics.rs, requirements.rs, mod.rs}`, `src/cad/mcp_dispatcher.ts` и `docs/MCP_API_REFERENCE.md` §38; добавлены тесты в `physics_tests.rs`, `physics.rs`, `mcp_contract_tests.rs` и `analyze_mechanism.test.ts`.
- **Компонент**: `crates/mcp-cad-physics/src/{joint.rs, lib.rs, simulator.rs}`, `crates/mcp-cad-server/src/tools/{physics.rs, requirements.rs, mod.rs}`, `src/cad/{kinematics.ts, mcp_dispatcher.ts}`, `docs/agent_tasks/P3-T15-report.md`

#### P3-T16 [S]: Итеративный геометрический решатель замкнутых кинематических цепей при пошаговом сканировании `sweep_joint` — ✅ DONE (`15.11.2026`, см. [`P3-T16-report.md`](docs/agent_tasks/P3-T16-report.md))
- **Реализовано**:
  1. **Детекция замкнутых контуров и полная обратная совместимость открытых цепей**: в `sweep_joint_with_parts` (`simulator.rs`) и TypeScript-зеркале `sweepJoint` (`src/cad/kinematics.ts`) открытые кинематические цепи продолжают выполняться быстрым жёстким проходом (`closed_loop_detected = false`), а при наличии связей с внешней компонентой/землёй или внутренних петель активируется итеративный решатель контуров.
  2. **Многотельный решатель замыкания контура (`ClosedLoopSolver`)**: пошаговое продолжение (continuation sub-stepping $\le 3^\circ$ / $0.05\,L_{\text{char}}$) и демпфированный Levenberg–Marquardt / SVD на размерно-нормированном якобиане $\tilde{J}\,\Delta q = -\Phi(q)$ с поддержкой `Fixed`, `Spherical`, `Revolute`, `Prismatic`, `Gear`, двусторонних `limits: [c, c]` и сканирования внутренних шарниров без ложных заклиниваний на плоских избыточных связях.
  3. **Детекция геометрического заклинивания и синхронизация MCP-контракта**: при $\|\Phi(q)\| > \text{tol}$ фиксируются `lockup_detected`, `lockup_step`, `lockup_value` (`lockup_angle_deg`, `lockup_angle_rad` для `Revolute`), `max_closure_residual`, а на каждом шаге — `closure_residual` и `locked_up` (`telemetry.rs`, `tools/physics.rs`, `tools/mod.rs`, `mcp_dispatcher.ts`, `docs/MCP_API_REFERENCE.md` §40); добавлены тесты в `physics_tests.rs`, `mcp_contract_tests.rs` и `analyze_mechanism.test.ts`.
- **Компонент**: `crates/mcp-cad-physics/src/{joint.rs, simulator.rs, telemetry.rs}`, `crates/mcp-cad-server/src/tools/{physics.rs, mod.rs}`, `src/cad/{kinematics.ts, mcp_dispatcher.ts}`, `docs/agent_tasks/P3-T16-report.md`

#### P3-T17 [S]: Поддержка несоосных и наклонных винтовых канавок `cut_helical_groove` при произвольной ориентации заготовки вне оси `Z` — ✅ DONE (`15.11.2026`, см. [`P3-T17-report.md`](docs/agent_tasks/P3-T17-report.md))
- **Реализовано**:
  1. **Преобразование вершин и метаданных граней (`TriangleMesh::apply_rot_and_shift`, `topology.rs`)**: выделен общий метод, синхронно вращающий/сдвигающий `vertices` и `faces` (`centroid`, `sample_point`, `normal_or_axis`) с сохранением `face_ids`; `rotate_rpy_deg` и `transform` переведены на него без дублирования.
  2. **Локальный фрейм заготовки и общий конвейер (`csg.rs`)**: добавлены детекция осевой симметрии `detect_coaxial_workpiece_frame` (нормаль `cap_bottom`/`cap_top` и 2D-центр окружности по Косе `fit_circle_center_2d` для `hole_wall`/`side`) и хелпер `execute_coaxial_in_local_frame` для `try_cut_coaxial_helical_groove`, `try_cut_coaxial_through_bore` и `try_union_coaxial_cone` (для соосных с `Z` деталей сохраняется быстрый путь без трансформаций).
  3. **Авто-выбор `start_z` на наклонных заготовках и тесты (`cad.rs`, `dag_tests.rs`)**: добавлена `default_helical_start_z` (для прямостоящих — `bounding_box().min[2]`, для наклонённых соосных — `-0.5 * axial_span`); добавлены тесты на смещённую/повёрнутую гайку и сверло (включая `transform_part` и `Add*` под углом `45°`/`90°`/`180°`) с проверкой `validate_mesh_strict` (`PASS`, `is_watertight = true`, `genus = 1`/`0`).
- **Компонент**: `crates/mcp-cad-core/src/{csg.rs, topology.rs}`, `crates/mcp-cad-core/tests/dag_tests.rs`, `crates/mcp-cad-server/src/tools/cad.rs`, `docs/agent_tasks/P3-T17-report.md`

#### P3-T18 [S]: Актуализация конструктора `TriangleMesh::new` в изолированном спайке `spikes/manifold_rust/src/main.rs:170` после добавления полей `face_ids` и `faces` (`P2-T6`) — ✅ DONE (`15.11.2026`, см. [`P3-T18-report.md`](docs/agent_tasks/P3-T18-report.md))
- **Выполнено**:
  1. В `spikes/manifold_rust/src/main.rs:170` (функция `manifold_to_triangle_mesh`) структурный литерал `TriangleMesh { vertices, indices }` заменён на `TriangleMesh::new(vertices, indices)`.
  2. Подтверждена чистая сборка, отсутствие предупреждений `clippy` и успешный запуск бинарного спайка (`cargo check`, `cargo clippy -- -D warnings`, `cargo test`, `cargo run` на `spikes/manifold_rust/Cargo.toml` — `exit 0`), а также `cargo clippy --workspace --all-targets -- -D warnings` (`0` ошибок, `0` предупреждений).
- **Компонент**: `spikes/manifold_rust/src/main.rs`, `docs/agent_tasks/P3-T18-report.md`

#### P3-T19 [S]: Исправление проверки `min_clearance_step` в тесте `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall` и паритет TS-зеркала `sweepJoint` — ✅ DONE (`15.11.2026`, см. [`P3-T19-report.md`](docs/agent_tasks/P3-T19-report.md))
- **Выполнено**:
  1. В `crates/mcp-cad-physics/tests/physics_tests.rs` подтверждён контракт `min_clearance_step` (глобальный минимум зазора по всем непересекающимся шагам `[0°, 90°]` — шаг `78` с зазором `~0.009 мм` при выходе за угол стены `(0.02, 0.07)` против `~0.146 мм` на шаге `27`) и добавлена проверка поддиапазона `[0°, 60°]` (`min_clearance_step == Some(27)`).
  2. В `src/cad/kinematics.ts:1637` открытая ветка `sweepJoint` синхронизирована с замкнутым контуром и Rust (`&& interferingPairs.length === 0`), а в `src/cad/__tests__/analyze_mechanism.test.ts` добавлены ассерты на `min_clearance_step`.
  3. Подтверждены `mcp-cad-physics` (`23/23 PASS`), `mcp-cad-server --lib` (`61/61 PASS`), `npm run lint` (`0` ошибок), `analyze_mechanism.test.ts` (`15/15 PASS`) и `cargo clippy --workspace --all-targets -- -D warnings` (`0` ошибок, `0` предупреждений).
- **Компонент**: `crates/mcp-cad-physics/tests/physics_tests.rs`, `src/cad/kinematics.ts`, `src/cad/__tests__/analyze_mechanism.test.ts`, `docs/agent_tasks/P3-T19-report.md`

#### P3-T20 [S]: Синхронизация контрактного теста `test_mcp_open_preview_window` (`crates/mcp-cad-server/tests/mcp_contract_tests.rs:716`) с поведением `P2-T37`
- **Проблема**: Выявлено при аудите **P3-T13** ([`docs/agent_tasks/P3-T13-report.md`](docs/agent_tasks/P3-T13-report.md)): тест `test_mcp_open_preview_window` (`crates/mcp-cad-server/tests/mcp_contract_tests.rs:716:5`) вызывает `open_preview_window` на пустой сцене в headless-режиме и ожидает `"status": "headless"`, тогда как после доработки `P2-T37` инструмент на пустой сцене возвращает `"status": "empty"`.
- **Что сделать**: Актуализировать тест `test_mcp_open_preview_window` (проверять `"empty"` на пустой сцене и контролируемый headless-сценарий при наличии детали и `MCP_CAD_DISABLE_GUI_SPAWN=1`).
- **Компонент**: `crates/mcp-cad-server/tests/mcp_contract_tests.rs`

#### P3-T21 [S]: Актуализация конструктора `TriangleMesh::new` в изолированном спайке `spikes/monstertruck_brep/src/main.rs:60`
- **Проблема**: Выявлено при аудите **P3-T13** ([`docs/agent_tasks/P3-T13-report.md`](docs/agent_tasks/P3-T13-report.md)): в изолированном спайке `spikes/monstertruck_brep/src/main.rs:60:20` инициализируется `let mut mesh = TriangleMesh { vertices, indices };` без полей `face_ids` и `faces`, из-за чего `cargo clippy` / `cargo check` завершается ошибкой `error[E0063]: missing fields face_ids and faces in initializer of mcp_cad_core::TriangleMesh`.
- **Что сделать**: Заменить структурный литерал на `TriangleMesh::new(vertices, indices)` и убедиться, что `cargo clippy --manifest-path spikes/monstertruck_brep/Cargo.toml --all-targets -- -D warnings` проходит без ошибок.
- **Компонент**: `spikes/monstertruck_brep/src/main.rs`

#### P3-T22 [S]: Восстановление отсутствующих в рабочей копии файлов CI/тулчейна и TS-теста (`rust-toolchain.toml`, `.github/workflows/ci.yml`, `src/cad/cad_engine.test.ts`)
- **Проблема**: Выявлено при аудите **P3-T13** ([`docs/agent_tasks/P3-T13-report.md`](docs/agent_tasks/P3-T13-report.md)): в текущей рабочей копии отсутствуют файлы `rust-toolchain.toml` и `.github/workflows/ci.yml` (заявленные выполненными в `P0-T8`), а также `src/cad/cad_engine.test.ts` (на который ссылаются `README.md:327`, `TASKS.md` и `PROJECT_CONTEXT.md:306`; фактически в `src/cad/__tests__/` есть только `analyze_mechanism.test.ts` с 11 тестами).
- **Что сделать**: Либо восстановить `rust-toolchain.toml`, `.github/workflows/ci.yml` и актуальный TS-тест, либо синхронизировать ссылки в `README.md`, `PROJECT_CONTEXT.md` и `TASKS.md` с реальным составом репозитория.
- **Компонент**: `rust-toolchain.toml`, `.github/workflows/ci.yml`, `src/cad/`, `README.md`

#### P3-T23 [S]: Устранение зависания многопоточного прогона `mcp_contract_tests` и `qa_wide_scan` на Windows (WGPU/GUI-конкуренция) и изоляция побочных записей `qa/results/*.json`
- **Проблема**: Выявлено при верификации **P3-T14** ([`docs/agent_tasks/P3-T14-report.md`](docs/agent_tasks/P3-T14-report.md)): при запуске `cargo test -p mcp-cad-server --test mcp_contract_tests` и `cargo test -p mcp-cad-server --test qa_wide_scan` в многопоточном режиме по умолчанию на Windows (без `-- --test-threads=1` и `MCP_CAD_DISABLE_GUI_SPAWN=1`) тест-раннер зависает после первых нескольких тестов (конкурентная инициализация WGPU / `OffscreenRenderer` между потоками). При последовательном запуске с `MCP_CAD_DISABLE_GUI_SPAWN=1 -- --test-threads=1` оба суита завершаются штатно (`57.86s` и `18.78s`). Кроме того, `qa_wide_scan` при запуске перезаписывает отслеживаемые в Git файлы `qa/results/*.json` и создаёт каталог `qa/artifacts/`.
- **Что сделать**: Сериализовать инициализацию/использование WGPU-рендерера в интеграционных тестах (либо автоматически отключать GUI-спавн под `#[cfg(test)]` и использовать `std::sync::Mutex` / общий ленивый рендерер), а также писать тестовые прогоны `qa_wide_scan` во временный каталог вместо модификации отслеживаемых `qa/results/*.json`.
- **Компонент**: `crates/mcp-cad-server/tests/{mcp_contract_tests.rs, qa_wide_scan.rs}`, `crates/mcp-cad-server/src/tools/render.rs`

---

### Фаза 4: Рецепты и Заготовки (P4)

#### P4-T0 [M]: Каркас «рецептов» и три заготовки
- **Что сделать**: Внедрить инструмент `apply_recipe` для быстрой генерации проверенных типовых машиностроительных узлов (фланцевая втулка с разболтовкой, ступенчатый вал со шпоночным пазом, зубчатая передача на общей раме).
- **Компонент**: `crates/mcp-cad-server/src/tools/`

---

## 3. Архивный реестр выполненных задач (Completed Tasks P0–P3)

Все архитектурные решения и результаты этих задач интегрированы в [`PROJECT_CONTEXT.md`](PROJECT_CONTEXT.md) (раздел 5), а подробные протоколы верификации сохранены в `docs/agent_tasks/`.

| ID | Фаза | Краткое содержание выполненной задачи | Раздел в `PROJECT_CONTEXT.md` | Отчёт |
|---|---|---|---|---|
| **P0-T1** | P0 Foundation | Исправление ориентации торцевых крышек (`cylinder`, `cone`, `spur_gear`, `wheel_rim`) | §5.6 | [`P0-T1-report.md`](docs/agent_tasks/P0-T1-report.md) |
| **P0-T2** | P0 Foundation | Параметрический суит геометрических и топологических инвариантов всех примитивов (`primitive_invariants.rs`) | §5.6, §7 | [`P0-T2-report.md`](docs/agent_tasks/P0-T2-report.md) |
| **P0-T3** | P0 Foundation | Триаж каскадных и остаточных падений тестов после P0-T1 (`docs/TRIAGE_AFTER_P0T1.md`) | §5.6 | [`P0-T3-report.md`](docs/agent_tasks/P0-T3-report.md) |
| **P0-T4** | P0 Foundation | Контракт ошибок выполнения инструментов (`result.isError = true`) и адаптация теста `BUG-016` | §5.3 | [`P0-T4-report.md`](docs/agent_tasks/P0-T4-report.md) |
| **P0-T5** | P0 Foundation | Устранение игольчатых треугольников на крышках частичного `revolve` (Штейнер + Делоне + Папп) | §5.6 | [`P0-T5-report.md`](docs/agent_tasks/P0-T5-report.md) |
| **P0-T6** | P0 Foundation | Устранение игольчатых треугольников на конических фасках `polygon_prism` (2-к-1 разрежение колец) | §5.6 | [`P0-T6-report.md`](docs/agent_tasks/P0-T6-report.md) |
| **P0-T7** | P0 Foundation | Двухфазная постобработка качества BSP-CSG (`optimize_csg_mesh_quality`) и суит `csg_invariants.rs` | §5.6 | [`P0-T7-report.md`](docs/agent_tasks/P0-T7-report.md) |
| **P0-T8** | P0 Foundation | Настройка CI (`.github/workflows/ci.yml`), `rust-toolchain.toml` (`1.91`), `Dockerfile` и маркировка `qa/` | §2, §7 | [`P0-T8-report.md`](docs/agent_tasks/P0-T8-report.md) |
| **P0-T9** | P0 Foundation | Верификация эквивалентности (`validate_mesh`, `genus`, $\Delta V \le 1\%$) и правило `R4` в Query Optimizer | §5.6 | [`P0-T9-report.md`](docs/agent_tasks/P0-T9-report.md) |
| **P0-T10** | P0 Foundation | Качество сетки примитивов на краевых параметрах (снятие всех 5 `known_bug_*` в `primitive_invariants.rs`) | §5.6 | [`P0-T10-report.md`](docs/agent_tasks/P0-T10-report.md) |
| **P0-T11** | P0 Foundation | Запрет создания шарнира на детали без геометрии и самосоединения `part_a == part_b` в `create_joint` | §5.5 | [`P0-T11-report.md`](docs/agent_tasks/P0-T11-report.md) |
| **P0-T12** | P0 Foundation | Полное устранение предупреждений `cargo clippy --workspace --all-targets -- -D warnings` и блокирующий гейт CI | §7 | [`P0-T12-report.md`](docs/agent_tasks/P0-T12-report.md) |
| **P1-T1** | P1 Architecture | Инвентаризация и план миграции `TS -> Rust Single Source of Truth` (`docs/MIGRATION_TS_TO_RUST.md`) | §5.7 | [`P1-T1-report.md`](docs/agent_tasks/P1-T1-report.md) |
| **P1-T2** | P1 Architecture | Честный бенчмарк 15 задач против нативного Rust-сервера (`bench_suite.rs`, `docs/BENCH_BASELINE_RUST.md`) | §5.7 | [`P1-T2-report.md`](docs/agent_tasks/P1-T2-report.md) |
| **P1-T3** | P1 Architecture | ADR-0001 выбора ядра и спайки `spikes/manifold_rust` (`v0.19.2`) + `spikes/monstertruck_brep` (`v0.4.1`) | §5.7 | [`P1-T3-report.md`](docs/agent_tasks/P1-T3-report.md) |
| **P1-T4** | P1 Architecture | Пакет задач миграции геометрического ядра `P1-K1..P1-K6` (`agent_prompts/P1_kernel_migration.md`) | §5.7 | [`P1-T4-report.md`](docs/agent_tasks/P1-T4-report.md) |
| **P2-T1** | P2 Contract | Аудит единиц измерения во всех инструментах и константах ядра (`docs/UNITS.md`, `[DECISION-UNITS-1]`) | §5.1 | [`P2-T1-report.md`](docs/agent_tasks/P2-T1-report.md) |
| **P2-T2** | P2 Contract | Внедрение контракта единиц (`length_unit = "mm"`, инструмент №29 `set_units`, `units`, `warnings`) | §5.1 | [`P2-T2-report.md`](docs/agent_tasks/P2-T2-report.md) |
| **P2-T3** | P2 Contract | Структурированные ошибки `ErrorEnvelope` / `ErrorBody` (36 кодов, `param`, `hint`, `details`) | §5.3 | [`P2-T3-report.md`](docs/agent_tasks/P2-T3-report.md) |
| **P2-T4** | P2 Contract | Транзакционное сохранение и загрузка проектов (`save_project` №30, `load_project` №31, `schema_version = "1.0.0"`) | §5.5 | [`P2-T4-report.md`](docs/agent_tasks/P2-T4-report.md) |
| **P2-T5** | P2 Contract | Архитектура топологического именования граней (`docs/FACE_NAMING_DESIGN.md`, `[DECISION-FACES-1..4]`) | §5.2 | [`P2-T5-report.md`](docs/agent_tasks/P2-T5-report.md) |
| **P2-T6** | P2 Contract | Реализация `face_ids` и `faces: Vec<FaceInfo>` для примитивов и фильтрации `FaceSelector` в `query_part_geometry` | §4.1, §5.2 | [`P2-T6-report.md`](docs/agent_tasks/P2-T6-report.md) |
| **P2-T7** | P2 Contract | Сквозное сохранение именованных граней через BSP-CSG (`merge_csg_face_registries`), `cut_hole` и `transform_part` | §5.2 | [`P2-T7-report.md`](docs/agent_tasks/P2-T7-report.md) |
| **P2-T8** | P2 Contract | Граничные условия МКЭ (`fixed`, `loads`: сила, давление, момент) по именованным граням `FaceSelector` | §4.3, §5.2 | [`P2-T8-report.md`](docs/agent_tasks/P2-T8-report.md) |
| **P2-T9** | P2 Contract | Слой инженерных требований (`set_requirements` №32, `check_requirements` №33, `auto_check`, `Part::revision`) | §5.4 | [`P2-T9-report.md`](docs/agent_tasks/P2-T9-report.md) |
| **P2-T10** | P2 Contract | Руководство `docs/AGENT_GUIDE.md`, `initialize.instructions` и инструмент сводки №34 `get_scene_summary` | §5.8, §8 | [`P2-T10-report.md`](docs/agent_tasks/P2-T10-report.md) |
| **P2-T11** | P2 Contract | Строгая валидация конечности `target_velocity` и неотрицательности `max_force >= 0` в `set_joint_motor` | §5.3 | [`P2-T11-report.md`](docs/agent_tasks/P2-T11-report.md) |
| **P2-T12** | P2 Contract | Транзакционный откат `physics_world` в `batch_cad_operations` и сохранение `!part.is_fixed` | §5.5 | [`P2-T12-report.md`](docs/agent_tasks/P2-T12-report.md) |
| **P2-T13** | P2 Contract | Пользовательский `joint_id` в `create_joint`, `"through": true` в `cut_hole`, `inner_radius` в `revolve` и алиасы материалов | §4.2, §8 | [`P2-T13-report.md`](docs/agent_tasks/P2-T13-report.md) |
| **P2-T14** | P2 Contract | Учёт явных `segments`/`sectors`/`stacks` в `dag.rs` и связности/относительного допуска в `inspect_geometry` | §5.2 | [`P2-T14-report.md`](docs/agent_tasks/P2-T14-report.md) |
| **P2-T15** | P2 Contract | Защита от паники `f64::clamp` на малых колёсах (`safe_clamp`) и исправление оси мотора `Prismatic` (`LinX`) | §5.5, §5.6 | [`P2-T15-report.md`](docs/agent_tasks/P2-T15-report.md) |
| **P2-T16** | P2 Contract | Строгая валидация `fixed_faces` и `applied_forces` в `run_fea_analysis` и сварка вершин `weld_vertices` по 27 ячейкам | §5.3 | [`P2-T16-report.md`](docs/agent_tasks/P2-T16-report.md) |
| **P2-T17** | P2 Contract | Транзакционность `record.motor` при отказе `apply_motor` и запрет мотора на `CadJointType::Gear` | §5.5 | [`P2-T17-report.md`](docs/agent_tasks/P2-T17-report.md) |
| **P2-T18** | P2 Contract | Метод `PhysicsWorld::update_part_collider`: обновление коллайдера и массы детали без потери привязанных шарниров | §5.5 | [`P2-T18-report.md`](docs/agent_tasks/P2-T18-report.md) |
| **P2-T19** | P2 Contract | Транзакционный откат `modify_feature` при ошибке `rebuild()` и авто-выравнивание `start_z` по `min_z` в `cut_helical_groove` | §4.2, §5.5 | [`P2-T19-report.md`](docs/agent_tasks/P2-T19-report.md) |
| **P2-T20** | P2 Contract | Синхронизация аналитической модели стоимости `estimate_dag_cost` с явными `segments` и углом дуги `angle_deg` | §5.6 | [`P2-T20-report.md`](docs/agent_tasks/P2-T20-report.md) |
| **P2-T21** | P2 Contract | Проброс `wheel_rim_diameter_m` в `check_scale_sanity` при `modify_feature` и вывод `limits` шарниров в `get_scene_summary` | §5.1, §8 | [`P2-T21-report.md`](docs/agent_tasks/P2-T21-report.md) |
| **P2-T22** | P2 Contract | Валидация `teeth_a`/`teeth_b`, `ratio` и осей `axis_a`/`axis_b` для `Gear` и `step_size_ms > 0` в `run_physics_simulation` | §5.3 | [`P2-T22-report.md`](docs/agent_tasks/P2-T22-report.md) |
| **P2-T23** | P2 Contract | Защита зарезервированных ID `"ground"`/`"world"`, нормализация регистра в `create_joint` и устранение sliver на малом ободе | §4.3, §5.6 | [`P2-T23-report.md`](docs/agent_tasks/P2-T23-report.md) |
| **P2-T24** | P2 Contract | Предотвращение ложного устаревания (`STALE`) результатов FEA (`part.revision`) при вызове `export_stl` и `tessellate_part` | §5.4 | [`P2-T24-report.md`](docs/agent_tasks/P2-T24-report.md) |
| **P2-T25** | P2 Contract | Точный радиус $N$-угольника и учёт `rotation_deg` в `estimate_primitive_aabb`, очистка `seen_holes` на барьерах в `R2`/`R3` | §5.6 | [`P2-T25-report.md`](docs/agent_tasks/P2-T25-report.md) |
| **P2-T26** | P2 Contract | Вывод `revision`, `bounding_box`, `volume`, `warnings` и учёт `length_unit` в `boolean_*` и `limits`/`axis`/`ratio` в `load_project` | §5.1, §8 | [`P2-T26-report.md`](docs/agent_tasks/P2-T26-report.md) |
| **P2-T27** | P2 Contract | Вывод `fea_revision`/`stale` в `get_simulation_metrics_json` и защита от холостого инкремента `part.revision` в `optimize_feature_tree` | §5.4 | [`P2-T27-report.md`](docs/agent_tasks/P2-T27-report.md) |
| **P2-T28** | P2 Contract | Остановка поиска соосного отверстия на барьерных узлах в `R4`, проталкивание сдвига в `AddSpurGear`/`AddWheelRim` в `R1` и соосность в `R5` | §5.6 | [`P2-T28-report.md`](docs/agent_tasks/P2-T28-report.md) |
| **P2-T29** | P2 Contract | Поддержка `length_unit` в `list_parts` и вывод `axis`/`axis_a`/`axis_b` для `CadJointType::Gear` в `get_scene_summary` | §5.1, §8 | [`P2-T29-report.md`](docs/agent_tasks/P2-T29-report.md) |
| **P2-T30** | P2 Contract | Поддержка `file_path`/`part_id`/`is_fixed` и регистрация в `physics_world` в `import_stl`, учёт `resp["length_unit"]` в `auto_check` | §5.1, §5.5 | [`P2-T30-report.md`](docs/agent_tasks/P2-T30-report.md) |
| **P2-T31** | P2 Contract | Разбиение вычитаний по барьерным узлам в `R3`, оценка AABB для `AddSpurGear`/`AddWheelRim` и ветка `BspUnionJoin` в `estimate_dag_cost` | §5.6 | [`P2-T31-report.md`](docs/agent_tasks/P2-T31-report.md) |
| **P2-T32** | P2 Contract | Схема `length_unit` и `part_id`/`revision` в `get_feature_tree` и `validate_mesh`, масштабирование `gravity` в `get_simulation_metrics_json` | §5.1, §8 | [`P2-T32-report.md`](docs/agent_tasks/P2-T32-report.md) |
| **P2-T33** | P2 Contract | Узел `FeatureOp::ImportedMesh` и `ensure_imported_mesh_in_dag()` для трансформации и CSG на импортированных STL-деталях | §4.2, §5.5 | [`P2-T33-report.md`](docs/agent_tasks/P2-T33-report.md) |
| **P2-T34** | P2 Contract | Масштабирование вершин и метаданных граней меш-операндов (`Boolean*Mesh`, `ImportedMesh`) в `get_feature_tree` | §5.1 | [`P2-T34-report.md`](docs/agent_tasks/P2-T34-report.md) |
| **P2-T35** | P2 Contract | Вывод `part_id`, `bounding_box`, `volume` (`depth`/`through` в `cut_hole`, `feature_id` в `modify_feature`) во всех мутациях DAG | §8 | [`P2-T35-report.md`](docs/agent_tasks/P2-T35-report.md) |
| **P2-T36** | P2 Contract | Масштабирование `logical_plan.dag`, `physical_plan.dag` и `candidate_plans[i].dag` в `optimize_feature_tree`, `part_id`/`deleted_part_id` в `delete_part` и точность `f64` в `PhysicsWorld::gravity()` | §5.1, §8 | [`P2-T36-report.md`](docs/agent_tasks/P2-T36-report.md) |
| **P2-T37** | P2 Contract / Antigravity UX | Автономный запуск нативного 3D-окна WGPU в `open_preview_window` из `stdio`-сессии (фоновый спавн `--stl`) без отказа `"status": "headless"` | §5.8, §8 | [`P2-T37-report.md`](docs/agent_tasks/P2-T37-report.md) |
| **P2-T38** | P2 Contract / Antigravity UX | Разделение фатальных ошибок топологии (`FAIL`) и предупреждений о микро-гранях (`warnings` / `sliver_triangles`) в `validate_mesh`, добавление `strict_slivers`, `is_valid_solid` и `fea_ready` | §5.6, §8 | [`P2-T38-report.md`](docs/agent_tasks/P2-T38-report.md) |
| **P2-T39** | P2 Contract / Antigravity UX | Устранение геометрической неопределённости в `initialize.instructions`, `AGENT_GUIDE.md`, `SKILL.md` и `tools/mod.rs` (таблица привязок примитивов, авто-`csg_union` в DAG, спец-инструменты и `GUI vs Headless`) | §4.1, §5.8 | [`P2-T39-report.md`](docs/agent_tasks/P2-T39-report.md) |
| **P2-T40** | P2 Contract | Сохранение `scope: "scene"` / `part_id: null` в `check_requirements` при 1 детали в сцене и вызов `attach_auto_check_if_enabled` в `delete_part` | §5.4, §8 | [`P2-T40-report.md`](docs/agent_tasks/P2-T40-report.md) |
| **P3-T1** | P3 Calculation | Базовая точность FEA: эталонные задачи (`fea_benchmarks.rs`, `docs/FEA_ACCURACY.md`: осевое растяжение, консоль, пластина Кирша/Петерсона, цилиндр Ламе, трещотка регрессии) | §7 | [`P3-T1-report.md`](docs/agent_tasks/P3-T1-report.md) |
| **P3-T2** | P3 Calculation | Предполётная проверка `validate_mesh` (`MESH_INVALID`), баланс реакций опор ($\sum R + \sum F$), зона сингулярности заделки, блок `diagnostics` и поле `confidence` в `FeaReport` / `run_fea_analysis` | §4.3, §7, §8 | [`P3-T2-report.md`](docs/agent_tasks/P3-T2-report.md) |
| **P3-T3** | P3 Calculation | Оценка сеточной сходимости в результате FEA (`convergence_study: true`, `levels`, `max_seconds`, `accuracy`, экстраполяция Ричардсона, детекция насыщения сетки 60 ячеек и ступенчатого концентратора на криволинейной границе) | §4.3, §7, §8 | [`P3-T3-report.md`](docs/agent_tasks/P3-T3-report.md) |
| **P3-T4** | P3 Calculation | Квадратичный 10-узловой тетраэдр `TET10` (`Tet10Element`, 4-точечная квадратура Гаусса 2-го порядка, `to_tet10()`, `element_order: 1 \| 2` в `run_fea_analysis`, снижение ошибки прогиба консоли на `Medium` с `20.14%` до `0.25%`) | §4.3, §7, §8 | [`P3-T4-report.md`](docs/agent_tasks/P3-T4-report.md) |
| **P3-T5** | P3 Calculation | Архитектурное решение `ADR-0002` и прототип `spikes/fe_snap/` (снаппинг граничных узлов МКЭ на CAD-поверхность с барьером $J_s \ge 0.06$ и квадратичным проецированием `TET10`) | §4.3 | [`P3-T5-report.md`](docs/agent_tasks/P3-T5-report.md) |
| **P3-T6** | P3 Calculation | Извлечение пиковых динамических нагрузок, моментов заделки и ускорений из симуляции (`get_part_load_cases` №35, `extract_part_load_case`, баланс Даламбера и III закон Ньютона в локальной СК детали) | §4.4, §7, §8 | [`P3-T6-report.md`](docs/agent_tasks/P3-T6-report.md) |
| **P3-T7** | P3 Calculation | Квазистатический расчёт прочности по нагрузкам из симуляции (`run_fea_from_simulation` №36: распределение сил и моментов шарниров парой сил, объёмные силы инерции Даламбера, минимальная 6-DOF опора `3-2-1`, блок `source`) | §4.3, §7, §8 | [`P3-T7-report.md`](docs/agent_tasks/P3-T7-report.md) |
| **P3-T8** | P3 Calculation | Аналитическая проверка прочности зубьев шестерён по Льюису (с фактором Барта $K_v$) и контактных напряжений питтинга по Герцу / AGMA (`check_gear_strength` №37) | §7, §8 | [`P3-T8-report.md`](docs/agent_tasks/P3-T8-report.md) |
| **P3-T9** | P3 Calculation | Анализ степеней свободы и заклинивания механизма по формуле Чебышёва–Грюблера–Кутцбаха и SVD матрицы Якоби связей (`analyze_mechanism` №38, требование `mechanism_dof`) | §4.4, §5.4, §7, §8 | [`P3-T9-report.md`](docs/agent_tasks/P3-T9-report.md) |
| **P3-T10** | P3 Calculation | Детекция статической интерференции и зазоров (`check_interference` №39, требование `no_interference`) и пошаговая развёртка хода шарниров (`sweep_joint` №40) | §4.3, §5.4, §7, §8 | [`P3-T10-report.md`](docs/agent_tasks/P3-T10-report.md) |
| **P3-T11** | P3 Calculation | Водонепроницаемость (`is_watertight = true`, `genus = 1`) и полное устранение игольчатых граней (`0.00% < 5°`) в `cut_helical_groove` на гайке М10 (`T08_hex_nut_m10`) и спиральном сверле с конической вершиной | §4.2, §5.6, §7 | [`P3-T11-report.md`](docs/agent_tasks/P3-T11-report.md) |
| **P3-T12** | P3 Calculation | Устранение открытых рёбер BSP-CSG на составной фланцевой втулке (`union + difference`) и учёт `connected_components` в формуле рода Эйлера–Пуанкаре $g = (2k - \chi)/2$ (`diagnostics.rs`) | §5.6, §7 | [`P3-T12-report.md`](docs/agent_tasks/P3-T12-report.md) |
| **P3-T13** | P3 Calculation | Аудит и подтверждение отсутствия предупреждений `cargo clippy --workspace --all-targets -- -D warnings` на `rustc 1.91.1` в `mcp-cad-core` и `mcp-cad-server` (`EarClippedTriangle`, `metrics.rs`, `cad.rs`, `physics.rs`) | §7 | [`P3-T13-report.md`](docs/agent_tasks/P3-T13-report.md) |
| **P3-T14** | P3 Calculation | Проверка и подтверждение синхронизации длины (`1569` симв.) и всех обязательных токенов `P2-T10`/`P2-T39` в `AGENT_INSTRUCTIONS` (`protocol.rs:173`) с прохождением `test_mcp_initialize` и `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` | §5.8, §7 | [`P3-T14-report.md`](docs/agent_tasks/P3-T14-report.md) |
| **P3-T15** | P3 Calculation | Атрибуция избыточных связей по шарнирам (`redundant_joint_ids` через leave-one-joint-out SVD), диагностика упоров `limits` (`joints_at_limits`, `motor_blocked`, двусторонняя блокировка при нулевом диапазоне `[c, c]`) и синхронизация TS-зеркала/схемы в `analyze_mechanism` | §4.3, §7, §8 | [`P3-T15-report.md`](docs/agent_tasks/P3-T15-report.md) |
| **P3-T16** | P3 Calculation | Итеративный многотельный решатель замкнутых кинематических цепей (`ClosedLoopSolver` с SVD и continuation sub-stepping) и детекция геометрического заклинивания (`lockup_step`, `lockup_value`, `max_closure_residual`) в `sweep_joint` с зеркалом в TypeScript | §4.3, §7, §8 | [`P3-T16-report.md`](docs/agent_tasks/P3-T16-report.md) |
| **P3-T17** | P3 Calculation | Поддержка смещённых с оси `Z` и наклонённых/повёрнутых заготовок в `cut_helical_groove` (и смежных соосных операциях `try_cut_coaxial_through_bore`, `try_union_coaxial_cone`) через локальную СК симметрии (`detect_coaxial_workpiece_frame`, `execute_coaxial_in_local_frame`, `apply_rot_and_shift`, `default_helical_start_z`) | §5.6, §7, §8 | [`P3-T17-report.md`](docs/agent_tasks/P3-T17-report.md) |
| **P3-T18** | P3 Calculation | Актуализация инициализации `TriangleMesh::new(vertices, indices)` в изолированном спайке `spikes/manifold_rust/src/main.rs:170` после добавления полей `face_ids` и `faces` (`P2-T6`) | §5.7, §7 | [`P3-T18-report.md`](docs/agent_tasks/P3-T18-report.md) |
| **P3-T19** | P3 Calculation | Актуализация проверки `min_clearance_step` в `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall` (`Some(78)` на `[0°, 90°]`, `Some(27)` на `[0°, 60°]`) и синхронизация `&& interferingPairs.length === 0` в TS-ветке `sweepJoint` | §4.3, §7 | [`P3-T19-report.md`](docs/agent_tasks/P3-T19-report.md) |
| **TS-FIX-01** | TS Suite | Синхронизация числа пресетов МКЭ (5) и импортов типов в `src/cad/{cad_engine.test.ts, primitives.ts}` | §7 | — |
