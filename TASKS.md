# MCP-CAD: План работ и реестр задач (Backlog)

> **Единый источник правды по архитектуре и принятым решениям:** Все фундаментальные архитектурные решения, таблицы систем координат примитивов, семантика Solid DAG и контракты 34 инструментов MCP перенесены и подробно описаны в **[`PROJECT_CONTEXT.md`](PROJECT_CONTEXT.md)** (разделы 4, 5 и 8).  
> **Пошаговые отчёты о выполненных задачах (`P0-T1..T12`, `P1-T1..T4`, `P2-T1..T40`, `P3-T1..T4`):** хранятся в [`docs/agent_tasks/`](docs/agent_tasks/).  
> **Текущий статус верификации (11.11.2026):**  
> - **Rust Workspace (`cargo test --workspace --no-fail-fast`):** **250 тестов, 250 проходят, 0 падают (100% PASS)** (`mcp-cad-core`: 99, `mcp-cad-server`: 104, `mcp-cad-fea`: 25, `mcp-cad-physics`: 12, `mcp-cad-viewer`: 10).  
> - **TypeScript Suite (`npx tsx --test src/cad/cad_engine.test.ts`):** **39 тестов, 39 проходят (100% PASS)**; `npm run lint` (`tsc --noEmit`) — 0 ошибок.  
> - **Rust Clippy (`cargo clippy -p mcp-cad-fea -p mcp-cad-server --all-targets -- -D warnings`):** **0 ошибок, 0 предупреждений** (в `mcp-cad-core` на `rustc 1.91.1` выявлено 2 предупреждения `clippy::type_complexity`, заведено как `P3-T13`).

---

## 1. Актуальный бэклог задач (Active Backlog)

### Сводная таблица открытых задач

| ID | Фаза | Задача | Зависит от | Размер | Компонент | Статус |
|---|---|---|---|---|---|---|
| **P3-T5** | P3 Calculation | Сетка МКЭ, согласованная с границей (`docs/adr/0002-fe-mesher.md`, `spikes/fe_snap/`) **[DECISION]** | P1-T3, P3-T1 | M | Docs / FEA Spikes | ✅ DONE |
| **P3-T6** | P3 Calculation | Извлечение пиковых нагрузок из динамической симуляции (`get_part_load_cases`) | — | M | `mcp-cad-physics` | 📋 BACKLOG |
| **P3-T7** | P3 Calculation | Квазистатический расчёт прочности по нагрузкам из симуляции (`run_fea_from_simulation`) | P3-T6, P2-T8 | L | `mcp-cad-fea`, `physics` | 📋 BACKLOG |
| **P3-T8** | P3 Calculation | Аналитическая проверка прочности зубьев шестерён по Льюису и Герцу/AGMA (`check_gear_strength`) | P2-T2 | M | `mcp-cad-core`, `server` | 📋 BACKLOG |
| **P3-T9** | P3 Calculation | Анализ степеней свободы и заклинивания механизма (`analyze_mechanism`: Грюблер & SVD Якобиана) | — | M | `mcp-cad-physics` | 📋 BACKLOG |
| **P3-T10** | P3 Calculation | Детекция статической интерференции и развёртка хода шарниров (`check_interference`, `sweep_joint`) | P3-T9 | L | `mcp-cad-physics` | 📋 BACKLOG |
| **P3-T11** | P3 Calculation | Водонепроницаемость и качество кромок `cut_helical_groove` на внутренней резьбе гайки (`T08_hex_nut_m10`) и спиральном сверле | P0-T7, P1-T2 | M | `mcp-cad-core` | 📋 BACKLOG |
| **P3-T12** | P3 Calculation | Устранение открытых рёбер BSP-CSG на составной фланцевой втулке и учёт `connected_components` в формуле `genus` (`diagnostics.rs:414`) | P0-T7, P1-T3 | S | `mcp-cad-core` | 📋 BACKLOG |
| **P3-T13** | P3 Calculation | Устранение предупреждений `clippy::type_complexity` в `crates/mcp-cad-core/src/csg.rs:1175, 1285` на компиляторе `rustc 1.91.1` | P0-T12 | S | `mcp-cad-core` | 📋 BACKLOG |
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

#### P3-T6 [M]: Нагрузки из симуляции: извлечение (`get_part_load_cases`)
- **Что сделать**: Реализовать инструмент `get_part_load_cases` для извлечения пиковых динамических реакций в шарнирах и ускорений деталей по результатам `run_physics_simulation`.
- **Компонент**: `crates/mcp-cad-physics/src/telemetry.rs`, `crates/mcp-cad-server/src/tools/physics.rs`

#### P3-T7 [L]: Нагрузки из симуляции: приложение к FEA (`run_fea_from_simulation`)
- **Что сделать**: Реализовать инструмент `run_fea_from_simulation`, автоматически переносящий пиковые реакции шарниров и силы инерции Даламбера из физического симулятора на сопряжённые именованные грани детали в МКЭ-решатель.
- **Компонент**: `crates/mcp-cad-fea/`, `crates/mcp-cad-server/src/tools/{fea.rs, physics.rs}`

#### P3-T8 [M]: Проверка прочности зубьев (Lewis / AGMA)
- **Что сделать**: Реализовать инструмент `check_gear_strength` для быстрой аналитической оценки изгибных напряжений у корня зуба по формуле Льюиса и контактных напряжений по Герцу/AGMA (ISO 6336) для прямозубых шестерён `spur_gear`.
- **Компонент**: `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-server/src/tools/cad.rs`

#### P3-T9 [M]: Механизм: степени свободы (`analyze_mechanism`: Грюблер & SVD)
- **Что сделать**: Реализовать инструмент `analyze_mechanism` (и активировать проверку `mechanism_dof` в `check_requirements`) с расчётом подвижности механизма по формуле Чебышёва–Грюблера–Кузбаха и численным анализом ранга матрицы Якоби связей через SVD (`nalgebra`) для выявления избыточных связей и заклинивания.
- **Компонент**: `crates/mcp-cad-physics/src/joint.rs`, `crates/mcp-cad-server/src/tools/{physics.rs, requirements.rs}`

#### P3-T10 [L]: Механизм: интерференция и ход (`check_interference`, `sweep_joint`)
- **Что сделать**: Реализовать инструменты `check_interference` (проверка пересечения объёмов деталей сборки в текущем положении и активация требования `no_interference` в `check_requirements`) и `sweep_joint` (пошаговое сканирование угла/хода привода до первого столкновения деталей).
- **Компонент**: `crates/mcp-cad-physics/src/simulator.rs`, `crates/mcp-cad-server/src/tools/{physics.rs, requirements.rs}`

#### P3-T11 [M]: Водонепроницаемость и качество кромок `cut_helical_groove` на внутренней резьбе гайки (`T08_hex_nut_m10`) и спиральном сверле
- **Проблема**: Выявлено в рамках **P1-T2** ([`docs/BENCH_BASELINE_RUST.md`](docs/BENCH_BASELINE_RUST.md)) и подтверждено в ревью Antigravity:
  1. В задаче `T08_hex_nut_m10` вызов `cut_helical_groove` после `polygon_prism` и `cut_hole` (`crates/mcp-cad-core/src/dag.rs:664-676`, `csg_difference(&current, &cutter)`) на текущем BSP-CSG оставляет открытые граничные рёбра (`is_watertight = false`, `open_boundary_edges = 413`).
  2. На выходе спиральной канавки сверла (`drill_bit`) образуются узкие треугольники (`sliver triangles`) на пересечении винтовой фрезы с конической вершиной.
- **Что сделать**: Устранить потерю водонепроницаемости и минимизировать клиновидные грани при вычитании винтового тела `create_helical_cutter` (через конформный выход фрезы за торцы детали и/или переключение булева ядра на `manifold-rust` согласно `ADR-0001`), добившись `validate_mesh == PASS`, `is_watertight == true` и `genus == 1` на `T08_hex_nut_m10`.
- **Компонент**: `crates/mcp-cad-core/src/{dag.rs, primitives.rs, csg.rs}`

#### P3-T12 [S]: Открытые рёбра BSP-CSG на составной фланцевой втулке и учёт `connected_components` в формуле `genus` (`diagnostics.rs:414`)
- **Проблема**: Выявлено в рамках **P1-T3** ([`docs/adr/0001-geometry-kernel.md`](docs/adr/0001-geometry-kernel.md)):
  1. В `crates/mcp-cad-core/src/diagnostics.rs:414` топологический род вычисляется по однокомпонентной формуле `let genus = (2.0 - euler_characteristic as f64) / 2.0;` без учёта числа связных компонент `connected_components` ($k$), хотя для $k$ замкнутых оболочек действует обобщённая формула Эйлера $\chi = 2k - 2g \implies g = k - \chi / 2$.
  2. При цепочке `csg_union(&create_cylinder(15.0, 50.0, 64), &create_box(40.0, 40.0, 10.0))` и `csg_difference(&u, &create_cylinder(10.0, 52.0, 64))` текущий BSP-CSG теряет `24` граничных ребра.
- **Что сделать**: Исправить формулу `genus` в `crates/mcp-cad-core/src/diagnostics.rs:414` на `(2.0 * connected_components as f64 - euler_characteristic as f64) / 2.0` (при `connected_components > 0`), добавить unit-тест на несвязное объединение двух тел ($k = 2$), и устранить потерю граничных рёбер при цепочке `union + difference` фланцевой втулки.
- **Компонент**: `crates/mcp-cad-core/src/{diagnostics.rs, csg.rs}`

#### P3-T13 [S]: Устранение предупреждений `clippy` на компиляторе `rustc 1.91.1` в `mcp-cad-core` и `mcp-cad-server`
- **Проблема**: Выявлено при выполнении **P3-T1** ([`docs/agent_tasks/P3-T1-report.md`](docs/agent_tasks/P3-T1-report.md)) и дополнено при верификации **P3-T4** ([`docs/agent_tasks/P3-T4-report.md`](docs/agent_tasks/P3-T4-report.md)):
  1. В `crates/mcp-cad-core/src/csg.rs:1175, 1285` на `rustc 1.91.1` анализатор Clippy выдаёт 2 ошибки `clippy::type_complexity` на возвращаемом типе `Option<Vec<([u32; 3], Vector3<f64>, u32)>>` у функций `ear_clip_planar_ring_maxmin` и `try_remove_vertex_ear_clip`.
  2. В `crates/mcp-cad-server/src/tools/`:
     - `metrics.rs:91-92`: 2 предупреждения `clippy::doc_lazy_continuation` в doc-комментарии `get_scene_summary`;
     - `cad.rs:3636..3707, 4930..4947` (в unit-тестах): 8 предупреждений `clippy::bool_assert_comparison` (`assert_eq!(..., false/true)`);
     - `physics.rs:676` (в unit-тестах): 1 предупреждение `clippy::approx_constant` на тестовом литерале `"target_velocity": 6.28`.
     *(Предупреждения в `crates/mcp-cad-fea` и `crates/mcp-cad-server/src/tools/fea.rs` устранены в рамках `P3-T4`).*
- **Что сделать**: Вынести `type EarClippedTriangle = ([u32; 3], Vector3<f64>, u32);` в `crates/mcp-cad-core/src/csg.rs`, поправить отступ doc-комментария в `metrics.rs` и тестовые ассерты в `cad.rs` / `physics.rs`, убедившись, что `cargo clippy --workspace --all-targets -- -D warnings` проходит с 0 предупреждений на `rustc 1.91.1`.
- **Компонент**: `crates/mcp-cad-core/src/csg.rs`, `crates/mcp-cad-server/src/tools/{metrics.rs, cad.rs, physics.rs}`

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
| **TS-FIX-01** | TS Suite | Синхронизация числа пресетов МКЭ (5) и импортов типов в `src/cad/{cad_engine.test.ts, primitives.ts}` | §7 | — |
