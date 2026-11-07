# MCP-CAD: План работ и реестр задач (Backlog)

> **Статус аудита кода и результатов тестов:** Обновлен 07.11.2026 (после выполнения **P0-T1..P0-T12**, **P1-T1..P1-T4** и **P2-T1..P2-T12**).  
> **Итоги тестовой верификации:**  
> - **TypeScript/Frontend (`npx tsx --test src/cad/cad_engine.test.ts`):** 26 тестов, 25 проходят, 1 падает (`AssertionError: 5 !== 4` в тесте №17 `FEA_QUICK_LOAD_PRESETS.length`). `npm run lint` (`tsc --noEmit`) проходит без ошибок (0 errors).  
> - **Rust Workspace (`cargo test --workspace --no-fail-fast`):** **213 тестов** (включая 33 unit-теста в `mcp-cad-server --lib`, 33 теста в `mcp_contract_tests.rs`, 13 в `qa_wide_scan`, 1 в `bench_suite`, 1 в `gear_kinematic_test`, 50 в `mcp-cad-core --lib`, 9 в `csg_invariants`, 14 в `dag_tests`, 24 в `primitive_invariants`, 6 в `tessellation_tests`, 13 в `fea`, 6 в `physics`, 10 в `viewer`), **213 проходят, 0 падают, 0 ignored (100% PASS)**.  
> - **Транзакционный откат `physics_world` в `batch_cad_operations`, сохранение `!part.is_fixed` при CSG/transform и удаление мёртвого `tools/export.rs` (`P2-T12`, [`docs/agent_tasks/P2-T12-report.md`](docs/agent_tasks/P2-T12-report.md)):** Для `PhysicsWorld` (`crates/mcp-cad-physics/src/simulator.rs`) реализован `Clone` и методы инспекции `has_part`/`is_part_dynamic`, в `batch_cad_operations` (`tools/cad.rs`) добавлен полный откат `state.physics_world` на всех путях ошибки, в `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part` и `optimize_feature_tree` обеспечена перерегистрация детали с флагом `!part.is_fixed` (сохраняющим `RigidBodyType::Fixed`), а мёртвый дубликат `crates/mcp-cad-server/src/tools/export.rs` удалён.
> - **Валидация параметров мотора в `set_joint_motor` (`P2-T11`, [`docs/agent_tasks/P2-T11-report.md`](docs/agent_tasks/P2-T11-report.md)):** В `crates/mcp-cad-server/src/tools/physics.rs` и `error.rs` внедрена строгая проверка конечности (`is_finite()` в `f64` и `f32`) для `target_velocity` и `max_force`, а также неотрицательности `max_force >= 0.0` с возвратом структурированной ошибки `INVALID_ARGUMENT` (`param: "target_velocity" | "max_force"`).
> - **Документация и сводка сцены для агента (`P2-T10`, [`docs/AGENT_GUIDE.md`](docs/AGENT_GUIDE.md), [`docs/agent_tasks/P2-T10-report.md`](docs/agent_tasks/P2-T10-report.md)):** Создан гайд `docs/AGENT_GUIDE.md` ($\le 2$ стр., 7 правил + 5-шаговый цикл «Build → Validate → Analyze → Iterate» + 3 примера вызовов), обновлен `initialize.instructions` (`protocol.rs`, ~1480 символов), нормализованы описания и `inputSchema` всех 34 инструментов (`Units:`, `Example:`, `Common errors:`, `Next step:`, явные единицы во всех числовых полях), реализован инструмент №34 `get_scene_summary` (`tools/metrics.rs`).
> - **Слой инженерных требований (`P2-T9`, [`docs/agent_tasks/P2-T9-report.md`](docs/agent_tasks/P2-T9-report.md)):** В `Part` (`part.rs`) добавлен монотонный счётчик ревизий `revision: u64`, реализованы инструменты №32 `set_requirements` и №33 `check_requirements` (`crates/mcp-cad-server/src/tools/requirements.rs`) с поддержкой `bbox_max`, `bbox_min`, `mass_max`, `mesh_valid`, `min_safety_factor` (со статусом `STALE` при изменении `part.revision` после `run_fea_analysis`), `no_interference` и `mechanism_dof` (`NOT_AVAILABLE` до `P3-T10`/`P3-T9`), автоматическим добавлением `requirements_status` и `failed_requirements` в ответы мутирующих тулов при `auto_check: true` и сохранением в `save_project`/`load_project`.
> - **Граничные условия МКЭ по именам граней (`P2-T8`, [`docs/agent_tasks/P2-T8-report.md`](docs/agent_tasks/P2-T8-report.md)):** В `crates/mcp-cad-fea/src/boundary.rs` и `run_fea_analysis` (`crates/mcp-cad-server/src/tools/fea.rs`) реализовано задание закреплений (`fixed`, `fixed_faces`) и внешних нагрузок (`loads`, `applied_forces`: распределённая сила в `N`, нормальное давление в `MPa`, момент в `N·m`) через `FaceSelector` (`{"face": "<name>"}`, `{"faces": [...]}`, `{"face_id": ...}`, `{"where": {...}}`, шаблоны `"hole_wall:*"`) с проекционной привязкой поверхностных узлов воксельной сетки `TetrahedralMesh` к треугольникам CAD-грани и сохранением суммы узловых сил с точностью $< 10^{-9}$.
> - **Именованные грани на примитивах, CSG и transform (`P2-T5`, `P2-T6`, `P2-T7`, [`docs/FACE_NAMING_DESIGN.md`](docs/FACE_NAMING_DESIGN.md), [`docs/agent_tasks/P2-T6-report.md`](docs/agent_tasks/P2-T6-report.md), [`docs/agent_tasks/P2-T7-report.md`](docs/agent_tasks/P2-T7-report.md)):** В `TriangleMesh` (`topology.rs`) добавлены `face_ids: Vec<u32>` и `faces: Vec<FaceInfo>` (`FaceKind`), реализовано проставление канонических граней в `create_box`, `create_cylinder`, `create_cone`, `create_sphere`, `create_polygon_prism`, `cut_hole` (`hole_wall`, `hole_bottom`, `keyway_wall`) и `cut_helical_groove` (`primitives.rs`), сквозной пронос `face_id` и слияние реестров граней `merge_csg_face_registries` через все этапы BSP-CSG и постобработку сетки (`csg.rs`), тегирование `origin_feature` и алиасов `feat_N:<role>` / `hole_wall:feat_N` / `cut:feat_N:<role>` в `FeatureTree::evaluate_with_settings` (`dag.rs`), пересчёт при `translate`/`rotate_rpy_deg`/`scale`/`transform` (включая `pivot_face` и `align_face` в `transform_part`), а также вывод `"faces"` и фильтрация `face_selector`/`where` (включая шаблоны `"hole_wall:*"`, `"cut:*"`) в `query_part_geometry` (`tools/cad.rs`).
> - **Сохранение и загрузка проекта (`P2-T4`, [`docs/agent_tasks/P2-T4-report.md`](docs/agent_tasks/P2-T4-report.md)):** Внедрены инструменты №30 `save_project` и №31 `load_project` с версионированием схемы (`schema_version = "1.0.0"`), сохранением пользовательских материалов, кинематических шарниров с моторами `motor`, полной транзакционной гарантией (любая ошибка оставляет состояние нетронутым) и автоматическим пересчетом мешей из DAG при загрузке.
> - **Структурированные ошибки (`P2-T3`, [`docs/agent_tasks/P2-T3-report.md`](docs/agent_tasks/P2-T3-report.md)):** Внедрен контракт структурированных ошибок (`ErrorBody`, `ErrorEnvelope`) в `result.content[0].text` при `result.isError = true`: стабильные машиночитаемые коды (36 кодов, включая `PROJECT_SCHEMA_MISMATCH`), параметр `param`, действенные подсказки `hint`, флаг `retryable`, контекстные данные `details`.
> - **Аудит и внедрение контракта единиц (`P2-T1`, `P2-T2`, [`docs/UNITS.md`](docs/UNITS.md), [`docs/agent_tasks/P2-T2-report.md`](docs/agent_tasks/P2-T2-report.md)):** Внедрён контракт `[DECISION-UNITS-1]` (`length_unit = "mm"` по умолчанию с переключением в `"m"` через инструмент №29 `set_units` или аргумент тула; сила — `N`, напряжение — `MPa`, модуль Юнга — `GPa`, масса — `kg`, время — `s`, угол — `deg`) на границе `crates/mcp-cad-server/src/tools/*.rs` при сохранении строгого СИ внутри ядра, с возвратом метаданных `units` и предупреждений `warnings` (`check_scale_sanity`).  
> - **Прототипы и пакет миграции геометрического ядра (`P1-T3`, `P1-T4`, [`docs/adr/0001-geometry-kernel.md`](docs/adr/0001-geometry-kernel.md), [`agent_prompts/P1_kernel_migration.md`](agent_prompts/P1_kernel_migration.md)):** Созданы и проверены изолированные спайки `spikes/manifold_rust` (`manifold-rust v0.19.2`, Pure Rust) и `spikes/monstertruck_brep` (`monstertruck v0.4.1`, Pure Rust B-Rep) на эталонной фланцевой втулке со скруглением $R=2\text{ мм}$ и гайке М10 (`T08`), а также сформирован самодостаточный пакет промтов миграции ядра `P1-K1..P1-K6` в `agent_prompts/P1_kernel_migration.md`.  
> - **Бенчмарк 15 задач против нативного Rust-сервера (`P1-T2`, [`docs/BENCH_BASELINE_RUST.md`](docs/BENCH_BASELINE_RUST.md)):** Честный baseline `bench/tasks.json` на `mcp-cad-server` составляет **`0 / 15 (0.0%)`** в режиме Strict As-Is (без неявного `create_part`), **`2 / 15 (13.3%)`** с пред-созданием деталей (`T01`, `T03`) и **`6 / 15 (40.0%)`** после моста схем аргументов (`T01`, `T03`, `T05`, `T06`, `T07`, `T10`) ввиду расхождений контрактов/осей/единиц измерения и дефектов `cut_helical_groove` / `TET4` FEA / кинематики.  
> - **Rust Clippy (`cargo clippy --workspace --all-targets -- -D warnings`):** **0 ошибок, 0 предупреждений во всех крейтах и таргетах workspace**. В CI включён строгий блокирующий гейт.  
> - **Документация `qa/`:** Все отчёты в `qa/` помечены предупреждением об устаревании в рамках **P0-T8**. Источником истины являются фактические результаты выполнения тестов и задачи из пакета `fix/`.

---

## Сводная таблица задач (Phases P0–P4)

| ID | Фаза | Задача | Зависит от | Размер | Компонент | Статус |
|---|---|---|---|---|---|---|
| **P0-T1** | P0 Foundation | Исправить ориентацию торцевых крышек (цилиндр, конус, шестерня, обод) | — | M | `mcp-cad-core` | ✅ DONE ([отчёт](docs/agent_tasks/P0-T1-report.md)) |
| **P0-T2** | P0 Foundation | Инварианты для всех примитивов (набор тестов) | P0-T1 | M | `mcp-cad-core` | ✅ DONE ([отчёт](docs/agent_tasks/P0-T2-report.md)) |
| **P0-T3** | P0 Foundation | Триаж остаточных падений (анализ) | P0-T1 | S | `mcp-cad-core` | ✅ DONE ([триаж](docs/TRIAGE_AFTER_P0T1.md), [отчёт](docs/agent_tasks/P0-T3-report.md)) |
| **P0-T4** | P0 Foundation | Контракт ошибок: адаптация теста BUG-016 | — | S | `mcp-cad-server` | ✅ DONE ([отчёт](docs/agent_tasks/P0-T4-report.md)) |
| **P0-T5** | P0 Foundation | Игольчатые треугольники: частичный revolve | P0-T3 | M | `mcp-cad-core` | ✅ DONE ([отчёт](docs/agent_tasks/P0-T5-report.md)) |
| **P0-T6** | P0 Foundation | Игольчатые треугольники: фаска призмы | P0-T3 | M | `mcp-cad-core` | ✅ DONE ([отчёт](docs/agent_tasks/P0-T6-report.md)) |
| **P0-T7** | P0 Foundation | CSG: инварианты объёма, отверстия, водонепроницаемость | P0-T1, P0-T3 | L | `mcp-cad-core` | ✅ DONE ([отчёт](docs/agent_tasks/P0-T7-report.md)) |
| **P0-T8** | P0 Foundation | CI и гигиена репозитория (`rust-toolchain 1.91`, Docker, warnings) | — | S | DevOps / Repo | ✅ DONE ([отчёт](docs/agent_tasks/P0-T8-report.md)) |
| **P0-T9** | P0 Foundation | Оптимизатор DAG: правило R4 и CBO-тест | P0-T1 | S | `mcp-cad-core` | ✅ DONE ([отчёт](docs/agent_tasks/P0-T9-report.md)) |
| **P0-T10** | P0 Foundation | Качество сетки примитивов на краевых параметрах (5 `known_bug_*` из P0-T2) | P0-T2, P0-T3 | M | `mcp-cad-core` | ✅ DONE ([отчёт](docs/agent_tasks/P0-T10-report.md)) |
| **P0-T11** | P0 Foundation | Скрытый сбой `create_joint` на детали без геометрии и ложноположительный `assert` в `test_bug015` | P0-T4 | S | `mcp-cad-server` | ✅ DONE ([отчёт](docs/agent_tasks/P0-T11-report.md)) |
| **P0-T12** | P0 Foundation | Устранение предупреждений и ошибок `cargo clippy --workspace --all-targets -- -D warnings` (18 в `mcp-cad-core`) и включение строгого гейта в CI | P0-T8 | S | `mcp-cad-core`, CI | ✅ DONE ([отчёт](docs/agent_tasks/P0-T12-report.md)) |
| **P1-T1** | P1 Architecture | План миграции TS→Rust (Анализ ADR-001) | — | M | Docs / Arch | ✅ DONE ([план](docs/MIGRATION_TS_TO_RUST.md), [отчёт](docs/agent_tasks/P1-T1-report.md)) |
| **P1-T2** | P1 Architecture | Бенчмарк 15 задач против Rust-сервера | P0-T1 | M | `bench/`, `mcp-cad-server` | ✅ DONE ([baseline](docs/BENCH_BASELINE_RUST.md), [отчёт](docs/agent_tasks/P1-T2-report.md)) |
| **P1-T3** | P1 Architecture | ADR: выбор геометрического ядра **[DECISION]** | — | M | Docs / Core | ✅ DONE ([ADR](docs/adr/0001-geometry-kernel.md), [отчёт](docs/agent_tasks/P1-T3-report.md)) |
| **P1-T4** | P1 Architecture | Шаблон задач реализации выбранного ядра | P1-T3 | S | Docs / Prompts | ✅ DONE ([промты](agent_prompts/P1_kernel_migration.md), [отчёт](docs/agent_tasks/P1-T4-report.md)) |
| **P2-T1** | P2 Contract | Аудит единиц измерения **[DECISION]** | — | S | Docs / MCP | ✅ DONE ([аудит](docs/UNITS.md), [отчёт](docs/agent_tasks/P2-T1-report.md)) |
| **P2-T2** | P2 Contract | Внедрение контракта единиц | P2-T1 | M | `mcp-cad-server` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T2-report.md)) |
| **P2-T3** | P2 Contract | Структурированные ошибки (`to_error_body`) | P0-T4 | M | `mcp-cad-server` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T3-report.md)) |
| **P2-T4** | P2 Contract | Сохранение и загрузка проекта (`save_project`/`load_project`) | — | M | `mcp-cad-server` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T4-report.md)) |
| **P2-T5** | P2 Contract | Именованные грани: дизайн **[DECISION]** | P1-T3 | S | Docs / Topology | ✅ DONE ([дизайн](docs/FACE_NAMING_DESIGN.md), [отчёт](docs/agent_tasks/P2-T5-report.md)) |
| **P2-T6** | P2 Contract | Именованные грани: примитивы | P2-T5 | M | `mcp-cad-core` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T6-report.md)) |
| **P2-T7** | P2 Contract | Именованные грани: проход через CSG и transform | P2-T6, P0-T7 | L | `mcp-cad-core` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T7-report.md)) |
| **P2-T8** | P2 Contract | FEA: граничные условия по именам граней | P2-T7 | M | `mcp-cad-fea`, `server` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T8-report.md)) |
| **P2-T9** | P2 Contract | Слой требований (`set_requirements` / `check_requirements`) | P2-T2 | M | `mcp-cad-server` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T9-report.md)) |
| **P2-T10** | P2 Contract | Документация и сводка сцены для агента (`get_scene_summary`) | P2-T3, P2-T9 | M | Docs / `mcp-cad-server` | ✅ DONE ([гайд](docs/AGENT_GUIDE.md), [отчёт](docs/agent_tasks/P2-T10-report.md)) |
| **P2-T11** | P2 Contract | Валидация числовых параметров в `set_joint_motor` (`target_velocity`, `max_force >= 0`) | P0-T11 | S | `mcp-cad-server` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T11-report.md)) |
| **P2-T12** | P2 Contract | Откат `physics_world` в `batch_cad_operations`, сохранение `!part.is_fixed` при CSG/transform и удаление мёртвого `tools/export.rs` | P1-T1 | S | `mcp-cad-server`, `mcp-cad-physics` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T12-report.md)) |
| **P2-T13** | P2 Contract | Поддержка пользовательского `joint_id` в `create_joint`, `through: true` в `cut_hole` и алиасов параметров (`revolve.inner_radius`, `polygon_prism.outer_radius`/`chamfer_radius`, ID материалов) | P1-T2 | M | `mcp-cad-server` | ✅ DONE ([отчёт](docs/agent_tasks/P2-T13-report.md)) |
| **P2-T14** | P2 Contract | Учёт явных `segments`/`sectors`/`stacks` в `FeatureTree::evaluate_with_settings` (`dag.rs`) и связности/относительного допуска в `inspect_geometry` (`topology.rs`) | — | S | `mcp-cad-core` | 📋 BACKLOG |
| **P2-T15** | P2 Contract | Защита от паники `f64::clamp` (`min > max`) в `create_wheel_rim_with_settings` (`primitives.rs:3052`) и исправление оси мотора `JointAxis::LinX` для `Prismatic` в `Simulator::set_joint_motor` (`simulator.rs:223`) | P2-T1 | S | `mcp-cad-core`, `mcp-cad-physics` | 📋 BACKLOG |
| **P2-T16** | P2 Contract | Строгая валидация нераспознанных `fixed_faces` и пустых `applied_forces` в `run_fea_analysis` (`tools/fea.rs:99,130`) и учёт соседних ячеек в `weld_vertices` (`import_stl.rs:268`) | P2-T3 | S | `mcp-cad-server`, `mcp-cad-core` | 📋 BACKLOG |
| **P2-T17** | P2 Contract | Транзакционность `record.motor` при отказе `apply_motor` в `PhysicsWorld::set_joint_motor` (`simulator.rs:325`) и запрет `motor.is_some()` для `CadJointType::Gear` в `build_generic_joint` (`joint.rs:200`) | P2-T4, P2-T11 | S | `mcp-cad-physics` | 📋 BACKLOG |
| **P2-T18** | P2 Contract | Сохранение привязанных шарниров в `physics_world` при перерегистрации детали в CSG/transform и обновление коллайдера в `add_feature_primitive`/`cut_hole`/`modify_feature`/`cut_helical_groove` | P2-T12 | M | `mcp-cad-physics`, `mcp-cad-server` | 📋 BACKLOG |
| **P2-T19** | P2 Contract | Транзакционный откат `part.dag`/`part.mesh` при ошибке `rebuild()` в `modify_feature` и авто-выравнивание `start_z` по нижней грани детали в `cut_helical_groove` | P2-T13 | S | `mcp-cad-server` | 📋 BACKLOG |
| **P3-T1** | P3 Calculation | Базовая точность FEA: эталонные задачи | P0-T1 | M | `mcp-cad-fea` | 📋 BACKLOG |
| **P3-T2** | P3 Calculation | Предполётные проверки и баланс сил FEA | P0-T1 | M | `mcp-cad-fea` | 📋 BACKLOG |
| **P3-T3** | P3 Calculation | Оценка сходимости в результате FEA | P3-T1 | M | `mcp-cad-fea` | 📋 BACKLOG |
| **P3-T4** | P3 Calculation | Элемент TET10 (квадратичные тетраэдры) | P3-T1 | L | `mcp-cad-fea` | 📋 BACKLOG |
| **P3-T5** | P3 Calculation | Исследование: сетка, согласованная с границей **[DECISION]** | P1-T3, P3-T1 | M | Docs / FEA Spikes | 📋 BACKLOG |
| **P3-T6** | P3 Calculation | Нагрузки из симуляции: извлечение (`get_part_load_cases`) | — | M | `mcp-cad-physics` | 📋 BACKLOG |
| **P3-T7** | P3 Calculation | Нагрузки из симуляции: приложение к FEA | P3-T6, P2-T8 | L | `mcp-cad-fea`, `physics` | 📋 BACKLOG |
| **P3-T8** | P3 Calculation | Проверка прочности зубьев (Lewis/AGMA) | P2-T2 | M | `mcp-cad-core`, `server` | 📋 BACKLOG |
| **P3-T9** | P3 Calculation | Механизм: степени свободы (Грюbler & SVD) | — | M | `mcp-cad-physics` | 📋 BACKLOG |
| **P3-T10** | P3 Calculation | Механизм: интерференция и ход (`check_interference`, `sweep_joint`) | P3-T9 | L | `mcp-cad-physics` | 📋 BACKLOG |
| **P3-T11** | P3 Calculation | Водонепроницаемость `cut_helical_groove` при нарезке внутренней резьбы гайки (`T08_hex_nut_m10`) | P0-T7, P1-T2 | M | `mcp-cad-core` | 📋 BACKLOG |
| **P3-T12** | P3 Calculation | Открытые рёбра BSP-CSG на составной фланцевой втулке и учёт `connected_components` в формуле `genus` (`diagnostics.rs:414`) | P0-T7, P1-T3 | S | `mcp-cad-core` | 📋 BACKLOG |
| **P4-T0** | P4 Blueprints | Каркас «рецептов» и генерации заготовок деталей | P0–P3 | M | `mcp-cad-server` | 📋 BACKLOG |
| **TS-FIX-01** | TS Suite Fix | Исправление рассинхронизации числа пресетов в `src/cad/cad_engine.test.ts` (5 vs 4) и `import type { AngleHistogramBuckets }` в `src/cad/primitives.ts:3` | — | S | `src/cad/{cad_engine.test.ts, primitives.ts}` | 🔴 FAILING TEST |

---

## Детальное описание задач по фазам

### Фаза 0: Вернуть доверие к фундаменту (P0)

#### P0-T1 [M]: Исправить ориентацию торцевых крышек (цилиндр, конус, шестерня, обод) — ✅ DONE
- **Проблема**: `create_cylinder(1.5, 5.0, 64)` даёт `signed_volume() = 11.83` при аналитическом `35.34`. Крышки перевёрнуты (нормали смотрят внутрь тела). При `segments = 8` объём совпадает. Ломаются функции `add_bridged_ring_strip_2d` (~строка 121), `add_halving_ring_transition_2d` (~187) и `add_subdivided_disk_cap_2d` (~229) в `crates/mcp-cad-core/src/primitives.rs`. Это первопричина падения 14+ тестов в `mcp-cad-core` и `mcp-cad-server`.
- **Что сделано**: Исправлен порядок вершин в `add_bridged_ring_strip_2d` и `add_halving_ring_transition_2d` (`crates/mcp-cad-core/src/primitives.rs`), чтобы нормали верхней крышки были направлены вдоль $+Z$, а нижней — вдоль $-Z$. Добавлены 4 unit-теста `cap_orientation_{cylinder, cone_sharp, cone_frustum, spur_gear}`. Подробный отчёт: [`docs/agent_tasks/P0-T1-report.md`](docs/agent_tasks/P0-T1-report.md).
- **Критерии приёмки**:
  - [x] Проходят тесты: `diagnostics::tests::test_validate_mesh_cylinder_pass`, `primitives::tests::{test_cylinder_volume, test_cone_sharp_volume, test_cone_frustum_volume, test_spur_gear_solid_volume_and_dimensions}`, `dag_tests::test_primitive_volumes_analytical_verification`, `qa_wide_scan::{test_bug_002, test_bug_004, test_bug_007}`.
  - [x] Добавлены unit-тесты `cap_orientation_*` на проверку `n · (+Z) > 0` для верха и `n · (-Z) > 0` для низа, а также `signed_volume()` в пределах 1% от аналитики.
  - [x] `validate_mesh` не выдаёт ошибку «Orientation is inconsistent».
- **Компонент**: `crates/mcp-cad-core/src/primitives.rs`

#### P0-T2 [M]: Инварианты для всех примитивов (набор автоматических тестов) — ✅ DONE
- **Проблема**: Существующие тесты проверяли объёмы только для нескольких точечных параметров.
- **Что сделано**: Создан `crates/mcp-cad-core/tests/primitive_invariants.rs` без внешних зависимостей (10 активных параметрических суитов по сетке параметров и 8 тестов `known_bug_*` с `#[ignore = "BUG: ..."]` для выявленных краевых дефектов качества сетки). Проверяются: `validate_mesh` PASS, единственность каждого направленного ребра `(u, v)` и обратного `(v, u)`, Эйлерова характеристика $\chi = 2 - 2g$, `signed_volume() > 0` и совпадение с аналитикой, а также точность AABB ($< 10^{-9}$). Подробный отчёт: [`docs/agent_tasks/P0-T2-report.md`](docs/agent_tasks/P0-T2-report.md).
- **Критерии приёмки**:
  - [x] Файл компилируется без предупреждений, тесты детерминированы, время прогона ~20 с (`< 60` с).
  - [x] Выявленные новые и существующие дефекты оформлены отдельными тестами `known_bug_*` с `#[ignore = "BUG: ..."]` и сведены в таблицу в отчёте.
  - [x] В отчёте приведена полная таблица «примитив × инвариант → результат».
- **Компонент**: `crates/mcp-cad-core/tests/primitive_invariants.rs`

#### P0-T3 [S]: Триаж остаточных падений (только анализ) — ✅ DONE
- **Что сделано**: Прогнан полный суит `cargo test --workspace --no-fail-fast` после P0-T1 и P0-T2 (148 тестов: 136 passed, 4 failed, 8 ignored). Составлен [`docs/TRIAGE_AFTER_P0T1.md`](docs/TRIAGE_AFTER_P0T1.md) и отчёт [`docs/agent_tasks/P0-T3-report.md`](docs/agent_tasks/P0-T3-report.md) с доказательством каскадной природы 4 починенных тестов (`test_dag_shifted_cylinder_stepped_shaft`, `test_dag_helical_groove_drill_bit_model`, `test_r4_coaxial_bushing_cylinder_hole_to_revolve`, `test_mcp_optimize_feature_tree_cbo`), точной трассировкой первопричин 4 остаточных падений (`test_bug016` → P0-T4, `test_bug_003` → P0-T5, `test_bug_005` → P0-T6, `test_bug_006` → P0-T7) и предложением задачи **P0-T10** для 5 краевых случаев `known_bug_*` из `primitive_invariants.rs`.
- **Компонент**: `docs/TRIAGE_AFTER_P0T1.md`, `docs/agent_tasks/P0-T3-report.md`

#### P0-T4 [S]: Контракт ошибок: тест BUG-016 — ✅ DONE
- **Проблема**: Валидация профиля `revolve` уже есть в `tools/cad.rs` (~305-390). Тест `test_bug016_revolve_boundary_and_self_intersection_validation` падал, так как ожидал верхнеуровневое JSON-RPC поле `error` вместо `result.isError = true` по спецификации MCP.
- **Что сделано**: В `crates/mcp-cad-server/tests/mcp_contract_tests.rs` введён общий хелпер `assert_tool_error(&resp) -> &str` (проверяющий отсутствие верхнеуровневого поля `error`, `result.isError == true` и непустой диагностический текст в `result.content[0].text`), применённый в `test_bug016_revolve_boundary_and_self_intersection_validation` (с проверкой всех трёх случаев: `r < 0`, `angle_deg > 360`, самопересечение) и в остальных проверках ошибок инструментов файла. В `docs/MCP_API_REFERENCE.md` добавлен раздел «Ошибки» с контрактом разделения ошибок выполнения инструмента и протокольных ошибок JSON-RPC. В отчёте [`docs/agent_tasks/P0-T4-report.md`](docs/agent_tasks/P0-T4-report.md) оформлена рекомендация по `[DECISION]` для `ServerError::ToolNotFound` и зафиксирована новая находка **P0-T11**.
- **Критерии приёмки**:
  - [x] `cargo test -p mcp-cad-server --test mcp_contract_tests test_bug016` проходит; все 21 тест файла `mcp_contract_tests` проходят (`21 passed; 0 failed`).
  - [x] Тест проверяет все три случая (`r < 0`, `angle_deg > 360`, самопересечение) и непустое диагностическое сообщение каждого.
  - [x] Контракт ошибок задокументирован в `docs/MCP_API_REFERENCE.md`.
- **Компонент**: `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P0-T4-report.md`

#### P0-T5 [M]: Игольчатые треугольники: частичный revolve — ✅ DONE
- **Проблема**: `test_bug_003_partial_revolve_mesh_and_renders` падал: `Sliver quality gate failed: 2 needle triangles with min angle < 1° (min_angle=0.9076°)`.
- **Что сделано**: В `triangulate_simple_polygon_2d` (`crates/mcp-cad-core/src/primitives.rs`) устранён размерный порог `1e-9` при проверке попадания вершин внутрь уха (переведено на безразмерные барицентрические координаты), реализован выбор уха с максимальным минимальным углом и постобработка `optimize_2d_delaunay_edges` (Lawson flip по критерию Делоне). Для торцевых крышек неполного вращения добавлена функция `triangulate_cap_polygon_2d` со вставкой внутренних точек Штейнера методом звёздчатой полости Бойера–Уотсона, а для рёбер профиля, касающихся оси вращения `r = 0`, реализовано 2-к-1 разрежение угловых колец. Добавлен unit-тест `test_revolve_partial_angles_validation_and_pappus_volume` (`30°`, `90°`, `180°`, `270°`, `360°`, прямоугольный и Г-образный профили) и снят `#[ignore]` с `known_bug_revolve_partial_angle_end_cap_slivers`. Подробный отчёт: [`docs/agent_tasks/P0-T5-report.md`](docs/agent_tasks/P0-T5-report.md).
- **Критерии приёмки**:
  - [x] `test_bug_003_partial_revolve_mesh_and_renders` проходит; полный оборот (`360°`) и остальные тесты revolve не регрессируют.
  - [x] `validate_mesh` частичного revolve (`30°`, `90°`, `180°`, `270°`; прямоугольный и Г-образный профили): `PASS`, `min_angle >= 1°`, доля `< 5°` меньше `1%`.
  - [x] Объём совпадает с теореме Паппа ($V = \theta \cdot r_c \cdot A$) с точностью `1%` (добавлен unit-тест `test_revolve_partial_angles_validation_and_pappus_volume`).
- **Компонент**: `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/tests/primitive_invariants.rs`

#### P0-T6 [M]: Игольчатые треугольники: фаска призмы — ✅ DONE
- **Проблема**: `test_bug_005_polygon_prism_chamfer_and_sides_validation` падал: `50.00% of triangles (12/24) have min angle < 5° (threshold 1.00%)`.
- **Что сделано**: В `create_polygon_prism` (`crates/mcp-cad-core/src/primitives.rs`) реализовано адаптивное разбиение граней фаски вдоль ребра многоугольника на $m_{\text{edge}} = 2^p$ подсегментов (и по наклонной высоте на `n_bevel_steps` рядов с выбором кратчайшей диагонали `push_quad_shortest_diag`) в сочетании с конформным геометрическим разрежением 2-к-1 (`m_curr -> m_curr / 2`) внутрь плоских торцевых крышек (`emit_polygon_cap`) и вертикальных боковых граней. Выведена и проверена точная аналитическая формула объёма призмы со срезанными фасками $V = \frac{n}{2} R^2 \sin(\frac{2\pi}{n}) h - n \sin(\frac{2\pi}{n}) c_z c_r (R - \frac{c_r}{3})$. Расширен параметрический тест `invariants_polygon_prism_chamfered_5_to_12_sides_grid` на все `sides ∈ 3..=12` и отношения фаски к стороне `{0.02, 0.1, 0.3, max}` и снят `#[ignore]` с `known_bug_polygon_prism_chamfer_low_sides_slivers`. Подробный отчёт: [`docs/agent_tasks/P0-T6-report.md`](docs/agent_tasks/P0-T6-report.md).
- **Критерии приёмки**:
  - [x] `test_bug_005_polygon_prism_chamfer_and_sides_validation` проходит; остальные тесты призмы не регрессируют.
  - [x] Для всех комбинаций `sides ∈ 3..=12` и отношений фаски к стороне `(0.02, 0.1, 0.3, max)`: `validate_mesh` возвращает `PASS`, объём совпадает с аналитическим (с точностью `< 1e-7`).
  - [x] Тест-перебор добавлен в `crates/mcp-cad-core/tests/primitive_invariants.rs`, снят `#[ignore]` с `known_bug_polygon_prism_chamfer_low_sides_slivers`.
- **Компонент**: `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/tests/primitive_invariants.rs`, `docs/agent_tasks/P0-T6-report.md`

#### P0-T7 [L]: CSG: инварианты объёма, отверстия, водонепроницаемость — ✅ DONE
- **Проблема**: `test_bug_006_csg_booleans_and_holes_watertight` падал (85 игл с min_angle=0.21°), а также был помечен `#[ignore]` тест `known_bug_cut_hole_csg_sliver_quality_gate` в `primitive_invariants.rs`.
- **Что сделано**: Создан суит `crates/mcp-cad-core/tests/csg_invariants.rs` (5 тестов: `box–box`, `cylinder–box`, `cylinder–cylinder` соосно и со смещением, `box–sphere`, все 6 режимов `cut_hole`), проверяющий тождества включения-исключения объёма ($V(A \cup B) + V(A \cap B) = V(A) + V(B)$ и $V(A \setminus B) = V(A) - V(A \cap B)$ с точностью $< 1\%$), `validate_mesh` PASS, $\chi = 2 - 2g$ и `t_junctions_count == 0`. В `crates/mcp-cad-core/src/csg.rs` реализовано слияние копланарных треугольников в выпуклые квады (`BspNode::polygons_from_mesh`), коалесценция нетронутых поддеревьев при клиппинге (`BspNode::clip_single_polygon` с `ClipOutcome::Unclipped` / `Discarded`), отказ от повторного разбиения `a_tree.build(b_tree.all_polygons())` и топологически безопасная постобработка качества сетки (`optimize_csg_mesh_quality`: Delaunay-флипы копланарных рёбер, пакетное удаление избыточных вершин BSP через 1-ring ear-clipping и Link-Condition edge collapse, тангенциальное сглаживание внутренних копланарных вершин и вставка точек Штейнера в копланарные веера к дугам отверстий). В `build_keyway_cutter_prism` (`primitives.rs`) круговая дуга согласована с плоскостями боковых стенок паза $x = \pm \text{half\_kw}$. Снят `#[ignore]` с `known_bug_cut_hole_csg_sliver_quality_gate`. Подробный отчёт: [`docs/agent_tasks/P0-T7-report.md`](docs/agent_tasks/P0-T7-report.md).
- **Критерии приёмки**:
  - [x] `test_bug_006_csg_booleans_and_holes_watertight` (`qa_wide_scan`) и все тесты в `crates/mcp-cad-core/tests/csg_invariants.rs` проходят (`PASS`).
  - [x] Снят `#[ignore]` с `known_bug_cut_hole_csg_sliver_quality_gate` в `primitive_invariants.rs`, тест проходит.
  - [x] На всех тестовых парах выполняются тождества объёма в пределах `1%`, `validate_mesh` возвращает `PASS` (включая пороги Sliver Gate `lt_1_deg_count == 0` и `lt_5_deg_ratio <= 1.0%` без ослабления), `t_junctions_count == 0`.
  - [x] Существующие тесты `dag_tests` не регрессируют.
- **Компонент**: `crates/mcp-cad-core/src/csg.rs`, `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/tests/csg_invariants.rs`, `docs/agent_tasks/P0-T7-report.md`

#### P0-T8 [S]: CI и гигиена репозитория — ✅ DONE
- **Проблема**: Отсутствует `.github/workflows/ci.yml`. `Dockerfile` использует старый `rust:1.82`, тогда как `Cargo.lock` требует `rustc >= 1.89`. Отчёты в `qa/` и `README.md` содержат устаревшие утверждения о «100% PASS».
- **Что сделано**: Создан `rust-toolchain.toml` с версией `1.91`, обновлён `Dockerfile` на `rust:1.91-bookworm`, добавлен GitHub Actions workflow `.github/workflows/ci.yml` с job-ами `rust`, `clippy` (`continue-on-error: true`, задокументированы 20 ошибок `clippy`) и `web` (`continue-on-error: true`, задокументирован 1 падающий тест из 26). В начало `qa/FINAL_REPORT_ROUND2.md`, `qa/BUGS.md`, `qa/COVERAGE.md`, `qa/LOG.md`, `qa/MESH_REPORT.md` добавлено предупреждение об устаревании, в `qa/STATE.json` — поле `"_warning"`, в `README.md` актуализированы сведения о версии `Rust >= 1.91` и результатах тестов. Подробный отчёт: [`docs/agent_tasks/P0-T8-report.md`](docs/agent_tasks/P0-T8-report.md).
- **Критерии приёмки**:
  - [x] YAML валиден (`python3 -c "import yaml; yaml.safe_load(...)"`); `rust-toolchain.toml` и `qa/STATE.json` валидны.
  - [x] Каждая команда из workflow проверена локально (`cargo build --workspace`, `cargo test --workspace --no-fail-fast`, `cargo clippy --workspace --all-targets -- -D warnings`, `npm run lint`, `npx tsx --test src/cad/cad_engine.test.ts`).
  - [x] Исходный код и тесты не изменены.
- **Компонент**: `rust-toolchain.toml`, `.github/workflows/ci.yml`, `Dockerfile`, `README.md`, `qa/*`, `docs/agent_tasks/P0-T8-report.md`

#### P0-T9 [S]: Оптимизатор DAG: правило R4 и CBO-тест — ✅ DONE
- **Проблема**: `test_r4_coaxial_bushing_cylinder_hole_to_revolve` и `test_mcp_optimize_feature_tree_cbo` падали до P0-T1, получая 3 физических узла вместо 1 (починились после P0-T1, но в `query_optimizer.rs:1220` отсутствовала полная проверка `validate_mesh`, проверка сохранения топологического рода `genus` и негативный тест на неосевое отверстие).
- **Что сделано**: В `crates/mcp-cad-core/src/query_optimizer.rs` в функцию `optimize_feature_tree_dag` внедрена строгая проверка эквивалентности: кандидатное дерево обязано проходить `validate_mesh` (`val_after.status == "PASS"`, либо не ухудшать качество, если исходная модель содержала дефекты), сохранять топологический род (`genus_before == genus_after` для замкнутых сеток) и укладываться в допуск объёма $\le 1\%$. Устранено фиктивное присваивание `node_cost_ms` (0 ошибок clippy в `query_optimizer.rs`). Добавлен unit-тест `test_r4_negative_off_axis_hole_does_not_trigger_revolve_reduction` на неосевое сквозное отверстие в плоскости $XY$ и поперечное отверстие по нормали $X$. Подробный отчёт: [`docs/agent_tasks/P0-T9-report.md`](docs/agent_tasks/P0-T9-report.md).
- **Критерии приёмки**:
  - [x] Оба теста (`test_r4_coaxial_bushing_cylinder_hole_to_revolve` и `test_mcp_optimize_feature_tree_cbo`) проходят.
  - [x] Оптимизатор доказывает эквивалентность: объём оптимизированного дерева отличается от исходного не более чем на 1%, сетка проходит `validate_mesh`.
  - [x] Добавлен негативный тест: неосевое отверстие НЕ вызывает R4.
- **Компонент**: `crates/mcp-cad-core/src/query_optimizer.rs`, `docs/agent_tasks/P0-T9-report.md`

#### P0-T10 [M]: Качество сетки примитивов на краевых параметрах (5 `known_bug_*` из P0-T2) — ✅ DONE
- **Проблема**: Параметрический суит `crates/mcp-cad-core/tests/primitive_invariants.rs` (P0-T2/P0-T3) выявил 5 краевых нарушений `Sliver quality gate` в генераторах примитивов, не покрытых задачами P0-T5…T7: `create_cone` при `segments=0` (`r_top/r_bot = 0.5`) и при `r_bot = 0, r_top > 0`; `create_spur_gear` при `teeth >= 36` с центральным отверстием; `create_revolve` 360° при `r_min = 0` и `segments = 128`; `create_wheel_rim` при `spokes_count ∈ {3, 4}`.
- **Что сделано**:
  1. В `create_cone` (`primitives.rs`) внедрена поддержка нечётного числа сегментов при 2-к-1 разрежении (через конформный 3-к-1 сектор), что устранило блокировку разрежения у вершины при auto-tessellation с нечётным промежуточным числом сегментов (`known_bug_cone_sharp_auto_segments_odd_after_halving_slivers`).
  2. В ветке перевёрнутого конуса `!has_bot_cap` реализовано ступенчатое геометрическое разрежение колец у нижнего полюса $z = -h/2$ с сохранением ориентации нормалей (`known_bug_cone_inverted_apex_missing_ring_halving`).
  3. В `create_spur_gear` уровень $r_b$ теперь вставляется только при $(r_b - r_f) \ge 0.35 \cdot \text{step\_r}$, исключая появление микро-радиального пояска шириной $44\,\mu\text{m}$ при $z \ge 36$ (`known_bug_spur_gear_high_teeth_cap_slivers`).
  4. В `create_wheel_rim_with_settings` введён гарантированный безопасный зазор `min_barrel_dz`, исключающий сжатие сегмента 14 цилиндрической стенки обода в узкую полосу при малом числе спиц (`known_bug_wheel_rim_low_spoke_count_slivers`).
  5. Сняты `#[ignore]` со всех 5 тестов `known_bug_*` в `primitive_invariants.rs` (включая проверенный `known_bug_revolve_360_axis_fan_slivers_at_128_segments`); снято ограничение `if !(segs == 0 && r_bot >= 0.05)` в `invariants_cone_sharp_and_frustum_grid`.
- **Критерии приёмки**:
  - [x] Все 5 тестов `known_bug_*` проходят (`PASS`) без атрибутов `#[ignore]`.
  - [x] Все 18 тестов в `primitive_invariants.rs` активны и проходят на 100%.
  - [x] Все существующие тесты `mcp-cad-core` (`--lib`, `dag_tests`, `tessellation_tests`, `csg_invariants`) и тесты сервера/fea/physics/viewer проходят без регрессий (155 passed, 0 failed, 0 ignored).
  - [x] Пороги Sliver Quality Gate не ослаблялись.
- **Компонент**: `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/tests/primitive_invariants.rs`, `docs/agent_tasks/P0-T10-report.md`

#### P0-T11 [S]: Скрытый сбой `create_joint` на детали без геометрии и ложноположительный `assert` в `test_bug015` — ✅ DONE
- **Проблема**: Выявлено при аудите `get("error")` в **P0-T4**: в `test_bug015_create_joint_and_create_part_schema_contract` (`crates/mcp-cad-server/tests/mcp_contract_tests.rs:2392–2424`) создавалась деталь `Rotor` через `create_part` без примитивов (`part.mesh.vertices.is_empty()`), после чего вызывался `create_joint` (`part_a: "ground"`, `part_b: p_id`). В `crates/mcp-cad-server/src/tools/physics.rs:145` деталь добавлялась в `physics_world` только при `!part.mesh.vertices.is_empty()`, из-за чего `state.physics_world.add_joint` возвращал `Err(PhysicsError::PartNotFound(p_id))` (`result.isError = true`), а строка `assert!(j_resp.get("error").is_none(), ...)` маскировала ошибку.
- **Что сделано**: В `crates/mcp-cad-server/src/tools/physics.rs` внедрена валидация деталей перед созданием шарнира: если у детали нет геометрии (`part.mesh.vertices.is_empty()`), возвращается `ServerError::InvalidArgument` с понятным сообщением; запрещено соединение детали самой с собой (`part_a == part_b`) и `ground-to-ground`; исключено подавление ошибок `add_part` (игнорируется только `PartAlreadyExists`, остальные пробрасываются). В `test_bug015` добавлен цилиндрический примитив к детали `Rotor`, проверен успешный возврат `joint_id` с префиксом `joint-` при `result.isError != true`, а также добавлены негативные тесты на пустую деталь и соединение детали с самой собой с проверкой через `assert_tool_error`. Подробный отчёт: [`docs/agent_tasks/P0-T11-report.md`](docs/agent_tasks/P0-T11-report.md).
- **Критерии приёмки**:
  - [x] `test_bug015` проверяет успешное создание шарнира через `result.isError != true` и формат `joint_id`.
  - [x] Попытка создания шарнира на детали без геометрии возвращает явную ошибку инструмента с понятным сообщением.
  - [x] Добавлен тест запрета соединения детали с собой.
  - [x] Все 21 тест в `mcp_contract_tests.rs` и тесты физики проходят на 100%.
- **Компонент**: `crates/mcp-cad-server/src/tools/physics.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/agent_tasks/P0-T11-report.md`

#### P0-T12 [S]: Устранение предупреждений и ошибок `cargo clippy --workspace --all-targets -- -D warnings` и включение строгого гейта в CI — ✅ DONE
- **Проблема**: При проверке `cargo clippy --workspace --all-targets -- -D warnings` в рамках **P0-T8** зафиксировано 20 ошибок компиляции в `mcp-cad-core` (`lib`): 5 в `primitives.rs` (`is_multiple_of`, `unused_mut`, `too_many_arguments`, `div_ceil`), 2 в `query_optimizer.rs` (`unused_assignments`, `unused_variables`) и 13 в `csg.rs` (`manual_range_contains`, `too_many_arguments`, `needless_range_loop`, `unnecessary_map_or`, `collapsible_if`), что также блокировало прогон `clippy` по зависимым крейтам и тестовым таргетам.
- **Что сделано**: В `crates/mcp-cad-core/src/primitives.rs` исправлены замечания `is_multiple_of`, добавлена аннотация `too_many_arguments` для локального хелпера квада, применен `div_ceil(2)`. В `crates/mcp-cad-core/src/csg.rs` заменены ручные диапазоны на `!(..=..).contains()`, упрощены `map_or` на `is_none_or`, объединены вложенные `if`, заменены циклы по индексам на итераторы, добавлены аннотации `too_many_arguments` для алгоритмов триангуляции и удаления вершин. В `.github/workflows/ci.yml` снят `continue-on-error: true` с job `clippy`. Команда `cargo clippy --workspace --all-targets -- -D warnings` завершается с 0 предупреждений и 0 ошибок по всему workspace. Все 155 тестов workspace проходят на 100%. Подробный отчёт: [`docs/agent_tasks/P0-T12-report.md`](docs/agent_tasks/P0-T12-report.md).
- **Критерии приёмки**:
  - [x] Все замечания `clippy` и `rustc` устранены во всех крейтах и таргетах workspace (`cargo clippy --workspace --all-targets -- -D warnings` даёт 0 ошибок и 0 предупреждений).
  - [x] В `.github/workflows/ci.yml` снят `continue-on-error: true` с шага `clippy`.
  - [x] Все 155 тестов workspace проходят без регрессий (100% PASS).
- **Компонент**: `crates/mcp-cad-core/src/`, `.github/workflows/ci.yml`, `docs/agent_tasks/P0-T12-report.md`

---

### Фаза 1: Архитектура и Геометрическое Ядро (P1)

#### P1-T1 [M]: План миграции TS→Rust (Анализ ADR-001) — ✅ DONE
- **Контекст**: В `docs/DECISIONS.md` (ADR-001, ADR-002) заявлено, что Rust-сервер уже является единственным бэкендом с маршрутами `/api/state`, `/api/events`, `/api/mcp` и раздачей `dist/`, но фактически в `src/cad/*.ts` сохранено 9 909 строк TS-движка, от которого напрямую зависит `src/App.tsx`, а в `crates/mcp-cad-server/src/sse.rs` есть только `/sse` и `/message`.
- **Что сделано**: Подготовлен полный документ [`docs/MIGRATION_TS_TO_RUST.md`](docs/MIGRATION_TS_TO_RUST.md) и отчёт [`docs/agent_tasks/P1-T1-report.md`](docs/agent_tasks/P1-T1-report.md):
  1. Проведена инвентаризация всех 9 файлов `src/cad/*.ts` (9 909 строк) с точным указанием дублируемых модулей/функций в Rust, графа импортов в TS и уникальной клиентской функциональности (`extractBuffersFromThreeGeometry`, `createLightbulbRevolveMesh`, проекция узловых полей МКЭ на вершины поверхностного меша детали, покадровый шаг кинематики, `candidatePlans` в Query Optimizer).
  2. Составлена полная таблица соответствия всех 28 MCP-тулов (`Rust vs TS`), выявившая 12 расхождений в именах обязательных параметров и семантике (`id` vs `name` в `register_material`, `material_id` vs `material_name`, несовместимые ID материалов, `feature_id` vs `node_index`, `part_a_id`/`part_b_id` vs `target_part_id`/`tool_part_id`, `rotation` в рад vs `rotation_deg`, схема ГУ в `run_fea_analysis`, схема `create_joint`).
  3. Спроектированы 9 целевых HTTP/SSE/Binary маршрутов для `crates/mcp-cad-server` (`/api/state`, `/api/events`, `/api/mcp`, `/api/parts/:id/mesh`, `/api/parts/:id/fea-field`, `/api/physics/step`, `/api/physics/trajectory`, `/api/parts/:id/stl`, `/api/parts/import-stl-bytes`).
  4. Разработан 4-шаговый план миграции (`P1-T2a..P1-T2d`, каждый шаг `<= 1 задача агента`) без поломки UI.
  5. Оформлены варианты решения по `[DECISION]` для `src/cad/cad_engine.test.ts` (1 502 строки, 26 тестов) с рекомендацией Варианта А.
- **Критерии приёмки**:
  - [x] Все ссылки на файлы и функции соответствуют реальному дереву проекта.
  - [x] Каждый шаг плана можно выполнить за одну задачу агента, не ломая сборку.
- **Компонент**: `docs/MIGRATION_TS_TO_RUST.md`, `docs/agent_tasks/P1-T1-report.md`

#### P1-T2 [M]: Бенчмарк 15 задач против Rust-сервера — ✅ DONE
- **Проблема**: Существующий `bench/run_benchmark.ts` импортировал `createHeadlessDispatcher()` из `src/cad/mcp_dispatcher.ts` (браузерного TypeScript-мока) и никогда не вызывал нативный Rust-сервер `mcp-cad-server`, а `bench/README.md` заявлял `15 / 15 PASSED (100%)`.
- **Что сделано**:
  1. Создан интеграционный Rust-тест `crates/mcp-cad-server/tests/bench_suite.rs` (`cargo test -p mcp-cad-server --test bench_suite -- --nocapture`), прогоняющий все 15 эталонных задач из `bench/tasks.json` через `mcp_cad_server::stdio::handle_message` в трёх режимах (`Mode 1: Strict As-Is` $\to$ `0/15`, `Mode 2: With create_part Pre-Init` $\to$ `2/15`, `Mode 3: Schema-Bridged Diagnostic` $\to$ `6/15`) без модификации `bench/tasks.json`.
  2. Создан TypeScript CLI-раннер `bench/run_benchmark_rust.ts` (`npx tsx bench/run_benchmark_rust.ts`), запускающий бинарник `mcp-cad-server --headless` по `stdin`/`stdout` JSON-RPC 2.0.
  3. Сформирован базовый отчёт [`docs/BENCH_BASELINE_RUST.md`](docs/BENCH_BASELINE_RUST.md) и отчёт о задаче [`docs/agent_tasks/P1-T2-report.md`](docs/agent_tasks/P1-T2-report.md) с пошаговой таблицей `T01`…`T15`, опровержением ложных утверждений в `bench/README.md` (оси цилиндра/конуса Z vs Y, единицы `module: 2.0` мм vs м, `TET4` вместо `TET10`, `cut_helical_groove` non-watertight, игнорирование `joint_id` в `create_joint`, непривязанный шатун в `T15`) и каталогом первопричин для `P2-T1` и `P3-*`.
- **Критерии приёмки**:
  - [x] Бенчмарк запускается одной командой (`cargo test -p mcp-cad-server --test bench_suite -- --nocapture` и `npx tsx bench/run_benchmark_rust.ts`) и выводит таблицу результатов.
  - [x] Все 15 задач отражены в `docs/BENCH_BASELINE_RUST.md` с честными статусами и числами (без «озеленения» ожиданий в `bench/tasks.json`).
- **Компонент**: `crates/mcp-cad-server/tests/bench_suite.rs`, `bench/run_benchmark_rust.ts`, `docs/BENCH_BASELINE_RUST.md`, `docs/agent_tasks/P1-T2-report.md`

#### P1-T3 [M]: ADR: выбор геометрического ядра **[DECISION]** — ✅ DONE
- **Что сделано**:
  1. Подготовлен архитектурный документ [`docs/adr/0001-geometry-kernel.md`](docs/adr/0001-geometry-kernel.md) и отчёт [`docs/agent_tasks/P1-T3-report.md`](docs/agent_tasks/P1-T3-report.md) со сравнением 4 вариантов (A: `manifold-rust v0.19.2`, B: `opencascade v0.3.0` / `occt-sys v7.8.1`, C: `monstertruck v0.4.1`, D: текущий BSP-CSG в `mcp-cad-core`) по 10 критериям, проверенными по `crates.io` версиями/лицензиями (`MIT` vs `Apache-2.0` vs `LGPL-2.1`), оценкой влияния на 5 крейтов workspace, декомпозицией `P1-T4` на 5 атомарных подзадач (`P1-K1`…`P1-K5`) и секцией **`[DECISION]`** с рекомендацией гибридной стратегии **A (`manifold-rust v0.19.2`) + опциональный C (`monstertruck-step`)**.
  2. Реализован и проверен изолированный прототип **Кандидата A** (`spikes/manifold_rust`): на эталонной фланцевой втулке (`R=15, h=50` + фланец `40×40×10` - отверстие `Ø20` + галтель `R=2`) `manifold-rust` в **37–188 раз быстрее** текущего BSP-CSG (`25.01 мс` / `5.00 мс` против `940.89 мс`), даёт `is_watertight = true` (`open_edges = 0`, $\chi = 0, g = 1.0$, `validate_mesh = PASS` со скруглением `R=2`), сохраняет `face_id = 101` верхнего торца (`192` треугольника, площадь `392.0686 мм²`, ошибка `0.0000%`) и решает проблему водонепроницаемости резьбы гайки М10 `T08_hex_nut_m10` (`18.87 мс`, `open_edges = 0`, `genus = 1.0` против `615` открытых рёбер у текущего BSP-CSG).
  3. Реализован и проверен изолированный прототип **Кандидата C** (`spikes/monstertruck_brep`): проверены B-Rep булевы операции (`833.96 мс`), аналитическое скругление `monstertruck-fillet` (`7.29 мс` на прямом ребре, `3.80 мс` на круговом стыке цилиндра с фланцем) и экспорт ISO 10303-21 STEP (`1.25 мс`, `232 899 байт`); экспериментально выявлены сброс `FaceID` при булевых операциях и генерация `1 327` игольчатых треугольников (`min_angle = 0.002°`, `11.76% < 5°`) в `monstertruck-meshing`.
- **Критерии приёмки**:
  - [x] Документ `docs/adr/0001-geometry-kernel.md` содержит проверяемые ссылки и версии из crates.io.
  - [x] Оба прототипа (`spikes/manifold_rust` и `spikes/monstertruck_brep`) собираются и запускаются одной командой `cargo run --release --manifest-path spikes/<name>/Cargo.toml`, не ломая основной workspace.
  - [x] Результаты обоих прототипов сведены в таблицу в ADR.
- **Компонент**: `docs/adr/0001-geometry-kernel.md`, `spikes/manifold_rust/`, `spikes/monstertruck_brep/`, `docs/agent_tasks/P1-T3-report.md`

#### P1-T4 [S]: Шаблон задач реализации выбранного ядра — ✅ DONE
- **Что сделано**: На основе раздела 7 («Оценка работ по миграции в задачах») архитектурного решения [`docs/adr/0001-geometry-kernel.md`](docs/adr/0001-geometry-kernel.md) сформирован полный самодостаточный пакет промтов [`agent_prompts/P1_kernel_migration.md`](agent_prompts/P1_kernel_migration.md) и отчёт [`docs/agent_tasks/P1-T4-report.md`](docs/agent_tasks/P1-T4-report.md):
  1. Включена актуализированная преамбула для запуска агентов в пустом контексте и матрица связей с задачами `P2-T5..P2-T8`, `P3-T11` и `P3-T12`.
  2. Подготовлены спецификации 5 основных задач миграции на `manifold-rust v0.19.2` (`P1-K1` [S]: мост `TriangleMesh <-> MeshGLP<f64, u32>` и `face_ids`; `P1-K2` [M]: перевод `csg_union/difference/intersection` на `Manifold` + `optimize_csg_mesh_quality` и исправление `genus` из `P3-T12`; `P1-K3` [S]: водонепроницаемость резьбы `cut_helical_groove` на `T08_hex_nut_m10` из `P3-T11`; `P1-K4` [M]: сквозная трассировка именованных граней `FaceInfo` через примитивы и `MeshGLP`, консолидирующая `P2-T5..T7`; `P1-K5` [M]: инструмент `extrude_sketch` со скруглением 2D-контуров на базе `CrossSection` + `Manifold::extrude`) и опциональной задачи `P1-K6` [M] (экспорт `FeatureTree` в ISO 10303-21 `STEP` через `monstertruck-io`).
- **Критерии приёмки**:
  - [x] Создан `agent_prompts/P1_kernel_migration.md` с проверенными путями к файлам, номерами строк, сигнатурами, командами приёмки и секциями «Не делать» для каждой подзадачи.
  - [x] Код основного workspace не изменялся (`156/156` тестов `PASS`, `0` предупреждений `clippy`).
- **Компонент**: `agent_prompts/P1_kernel_migration.md`, `docs/agent_tasks/P1-T4-report.md`

---

### Фаза 2: Контракт «Агент ↔ Система» (P2)

#### ✅ P2-T1 [S]: Аудит единиц измерения — ВЫПОЛНЕНО ([аудит](docs/UNITS.md), [отчёт](docs/agent_tasks/P2-T1-report.md))
- **Что сделано**:
  - [x] Создан документ [`docs/UNITS.md`](docs/UNITS.md) с полной ревизией входных и выходных единиц всех 28 MCP-инструментов (`crates/mcp-cad-server/src/tools/mod.rs:17-859` и обработчики `cad.rs`, `fea.rs`, `physics.rs`, `render.rs`, `metrics.rs`), с явным выделением всех параметров и полей ответов, где единицы не указаны.
  - [x] Составлен полный реестр захардкоженных размерных констант, допусков и порогов в `mcp-cad-core`, `mcp-cad-fea`, `mcp-cad-physics`, `mcp-cad-viewer`, `mcp-cad-server` с анализом их чувствительности к масштабу модели (`m` vs `mm`).
  - [x] Сформулирован контракт **`[DECISION-UNITS-1]`** (по умолчанию на границе MCP API: длина — `mm` с переключением в `"m"` через `set_units` или аргумент тула, сила — `N`, момент — `N·m`, напряжение — `MPa`, модуль Юнга — `GPa`, масса — `kg`, плотность — `kg/m³`, время — `s`/`ms`, угол — `deg`, угловая скорость — `rad/s`; внутри ядра — строго СИ `m`/`Pa`/`N`/`kg`/`s` с конвертацией только в `crates/mcp-cad-server/src/tools/*.rs`).
  - [x] Описана эвристика sanity-check масштаба (`warnings: []` при размерах в СИ `> 10 м` или `< 0.1 мм`) и детальный пофайловый план изменений для `P2-T2`.
- **Компонент**: `docs/UNITS.md`, `docs/agent_tasks/P2-T1-report.md`

#### ✅ P2-T2 [M]: Внедрение контракта единиц — ВЫПОЛНЕНО ([отчёт](docs/agent_tasks/P2-T2-report.md))
- **Что сделано**:
  - [x] В `EngineState` (`crates/mcp-cad-server/src/state.rs`) добавлен `length_unit: LengthUnit` (`LengthUnit::Mm` по умолчанию, `"mm"` | `"m"`), метод `resolve_length_unit` и эвристика `check_scale_sanity` (`SCALE_TOO_LARGE`, `SCALE_TOO_SMALL`, `SCALE_EXTREME_VOLUME`, `WHEEL_RIM_SCALE`).
  - [x] Добавлен инструмент №29 `set_units` (`length_unit: "mm" | "m"`), возврат `length_unit` и объекта `units` в `initialize`, `list_parts` и во всех инструментах, возвращающих численные/геометрические/физические данные (`cad.rs`, `fea.rs`, `physics.rs`, `metrics.rs`).
  - [x] Выполнена конвертация всех линейных размеров, площадей и объёмов на границе `crates/mcp-cad-server/src/tools/*.rs` (включая `get_feature_tree`, `modify_feature`, `run_fea_analysis`, `create_joint`, `run_physics_simulation`, `get_simulation_metrics_json`) при сохранении строгого СИ внутри `mcp-cad-core`, `mcp-cad-fea`, `mcp-cad-physics` и координат в метрах в файлах `.stl`.
  - [x] Обновлены описания всех числовых параметров в JSON Schema (`crates/mcp-cad-server/src/tools/mod.rs`), документация `docs/UNITS.md` и `docs/MCP_API_REFERENCE.md`.
  - [x] Обновлены контрактные тесты и добавлены 2 новых теста в `mcp_contract_tests.rs`: `test_units_equivalence_mm_vs_m` (совпадение объёма, массы и МКЭ с точностью `< 1e-6`) и `test_units_scale_sanity_warnings_20m_and_micro`.
- **Компонент**: `crates/mcp-cad-server/src/{state.rs, protocol.rs, tools/}`, `crates/mcp-cad-server/tests/`, `docs/agent_tasks/P2-T2-report.md`

#### P2-T3 [M]: Структурированные ошибки (`to_error_body`) — ✅ DONE
- **Проблема**: При возникновении ошибки инструмента (`ServerError`) в `result.content[0].text` возвращалась неструктурированная строка `err.to_string()`, что затрудняло программную диагностику агентом, выделение проблемного параметра и получение действенного следующего шага (`hint`).
- **Что сделано**:
  1. В `crates/mcp-cad-server/src/error.rs` введены структуры `ErrorEnvelope { error: ErrorBody }` и `ErrorBody` с полями: `code` (35 стабильных кодов), `message` (человекочитаемый текст для обратной совместимости), `param` (имя параметра или `null`), `hint` (действенное указание следующего шага), `retryable` (флаг возможности повтора), `details` (контекстные данные).
  2. Реализованы `ServerError::to_error_body(&self)` и `ServerError::to_error_body_with_parts(&self, existing_parts: Option<&[String]>)` с полным покрытием всех вариантов `ServerError` (`PartNotFound`, `FeatureNotFound`, `JointNotFound`, `ToolNotFound`, `InvalidArgument`, `InvalidParameter`, `Cad`, `Fea`, `Physics`, `Viewer`, `Io`, `Json`, `Internal`).
  3. Для `PartNotFound` в `hint` и `details.available_parts` возвращается актуальный список существующих деталей в сессии.
  4. Для неверного `primitive_type` возвращается список поддерживаемых примитивов в `details.supported_primitives`.
  5. Реализовано извлечение параметров `param` для частых сценариев (`operations`, `profile`, `angle_deg`, `part_id`, `name`, `fixed_faces`, `applied_forces`, `material`, `young_modulus_gpa` и др.), а также эвристический парсер `extract_quoted_param`.
  6. В `crates/mcp-cad-server/src/protocol.rs` в ветке `tools/call` внедрена передача `error_body.to_json_string()` с контекстом существующих деталей сессии.
  7. Экспортированы `ErrorBody`, `ErrorEnvelope` в `crates/mcp-cad-server/src/lib.rs`.
  8. Добавлено 18 unit-тестов в `src/error.rs`, усилен `assert_tool_error` в `mcp_contract_tests.rs` на валидацию JSON-структуры и добавлен контрактный тест `test_structured_error_contract_and_part_not_found_hint`. Все 177 тестов workspace проходят на 100%, `cargo clippy` даёт 0 ошибок и 0 предупреждений.
  9. В [`docs/MCP_API_REFERENCE.md`](docs/MCP_API_REFERENCE.md) задокументирован контракт структурированных ошибок и приведена таблица всех стабильных кодов. Подробный отчёт: [`docs/agent_tasks/P2-T3-report.md`](docs/agent_tasks/P2-T3-report.md).
- **Критерии приёмки**:
  - [x] Юнит-тесты `to_error_body` для каждого варианта; контрактный тест на `PART_NOT_FOUND` с подсказкой-списком существующих деталей.
  - [x] Все существующие тесты проходят без регрессий (177 passed, 0 failed, 0 ignored).
  - [x] Обновлена документация в `docs/MCP_API_REFERENCE.md` с таблицей кодов.
- **Компонент**: `crates/mcp-cad-server/src/{error.rs, protocol.rs, lib.rs}`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P2-T3-report.md`

#### P2-T4 [M]: Сохранение и загрузка проекта (`save_project`/`load_project`) — ✅ DONE ([отчёт](docs/agent_tasks/P2-T4-report.md))
- **Что сделано**:
  1. Реализован модуль `crates/mcp-cad-server/src/tools/project.rs` с версионированной схемой проекта (`schema_version = "1.0.0"`).
  2. Инструмент `save_project(path)`: сохраняет метаданные, пользовательские материалы (исключая стандартные дефолты), параметрические детали с полным деревом операций DAG без тяжелых сеток треугольников (сохраняются байты и I/O), кинематические шарниры с их моторными конфигурациями (`target_velocity`, `max_force`, `axis`). Форматирует компактный читаемый JSON с детерминированной сортировкой.
  3. Инструмент `load_project(path)`: выполняет транзакционную загрузку. При синтаксических ошибках или несовпадении версий схем возвращает структурированную ошибку `PROJECT_SCHEMA_MISMATCH` или `INVALID_ARGUMENT`, сохраняя текущее состояние сессии полностью нетронутым. При успешной загрузке восстанавливает материалы, пересобирает геометрию через `part.rebuild()` из параметрического DAG (гарантируя точные сетки и физические свойства), и пересобирает физический мир Rapier3D со всеми шарнирами и моторами.
  4. Расширен `CadJointRecord` в `crates/mcp-cad-physics` полем `motor: Option<MotorConfig>` для сохранения и экспорта параметров моторов.
  5. В `Part` добавлена аннотация `#[serde(default)]` для поля `mesh`, позволяя десериализовать детали без сохранения тяжелой сетки.
  6. Инструменты зарегистрированы в `list_tools` (общее число MCP-инструментов увеличено до 31) и задокументированы в `docs/MCP_API_REFERENCE.md`.
  7. Покрыто unit-тестами в `project.rs`, `error.rs` и контрактным тестом в `tests/mcp_contract_tests.rs`.
- **Критерии приёмки**:
  - [x] Инструменты `save_project` и `load_project` реализованы и зарегистрированы в MCP-сервере (`list_tools`).
  - [x] Формат JSON включает версионирование схемы (`schema_version: "1.0.0"`).
  - [x] Проект сохраняет параметрический DAG деталей, пользовательские материалы и кинематические шарниры с моторами; тяжелые полигональные сетки регенерируются детерминированно при загрузке.
  - [x] Транзакционность: при ошибке загрузки (повреждённый файл, неподдерживаемая версия) состояние сервера не разрушается.
  - [x] Написаны unit-тесты и интеграционный MCP-контрактный тест (roundtrip save/load).
  - [x] Все тесты workspace проходят (184 passed, 0 failed), `cargo clippy --workspace --all-targets -- -D warnings` — 0 ошибок и предупреждений.
- **Компонент**: `crates/mcp-cad-server/src/tools/project.rs`, `crates/mcp-cad-physics/src/simulator.rs`, `crates/mcp-cad-core/src/part.rs`, `crates/mcp-cad-server/src/error.rs`, `crates/mcp-cad-server/src/tools/mod.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P2-T4-report.md`

#### P2-T5 [S]: Именованные грани: дизайн **[DECISION]** — ✅ DONE ([дизайн](docs/FACE_NAMING_DESIGN.md), [отчёт](docs/agent_tasks/P2-T5-report.md))
- **Что сделано**:
  - [x] Подготовлен документ [`docs/FACE_NAMING_DESIGN.md`](docs/FACE_NAMING_DESIGN.md) со сквозной архитектурой топологического именования граней в `TriangleMesh` (`face_ids: Vec<u32>` и `faces: Vec<FaceInfo>` с полной обратной совместимостью через `#[serde(default)]`).
  - [x] Определена каноническая схема стабильных имён `"<feature_id>:<role>"` и коротких алиасов `"<role>"` для всех 12 генераторов примитивов и 3 конструктивных вырезов (`cut_hole`, `cut_fillet`, `cut_helical_groove`), устранено расхождение осей `create_box` с остальными Z-up примитивами (`[DECISION-FACES-2]`).
  - [x] Спроектирован пронос `face_id` через `transform` (`translate`, `rotate_rpy_deg`, `scale`, `transform`), текущий BSP-CSG (`CsgPolygon`, `Plane::split_polygon`, `polygons_to_mesh`, `repair_manifold_triangles`, `optimize_csg_mesh_quality` в `csg.rs`) и целевое ядро `manifold-rust v0.19.2` (`MeshGLP::face_id` согласно `ADR-0001`).
  - [x] Специфицирован унифицированный селектор `FaceSelector` (`{"face": "<name>"}`, `{"faces": [...]}`, `{"where": {"kind": "cylinder", "radius": ...}}` и legacy AABB `"min_z"`..`"max_z"`) для `query_part_geometry`, `run_fea_analysis`, `create_joint` и `transform_part`, а также алгоритм привязки узлов воксельной сетки `TET4` к треугольникам именованной грани в `crates/mcp-cad-fea/src/boundary.rs`.
  - [x] Составлена пофайловая таблица всех затрагиваемых структур/строк и декомпозиция на задачи `P2-T6`, `P2-T7`, `P2-T8`, проведён анализ рисков и оформлены решения `[DECISION-FACES-1..4]`. Подробный отчёт: [`docs/agent_tasks/P2-T5-report.md`](docs/agent_tasks/P2-T5-report.md).
- **Компонент**: `docs/FACE_NAMING_DESIGN.md`, `docs/agent_tasks/P2-T5-report.md`

#### P2-T6 [M]: Именованные грани: примитивы — ✅ DONE ([отчёт](docs/agent_tasks/P2-T6-report.md))
- **Что сделано**:
  - [x] В `crates/mcp-cad-core/src/topology.rs` добавлены `FaceKind`, `FaceInfo`, поля `face_ids: Vec<u32>` и `faces: Vec<FaceInfo>` (`#[serde(default, skip_serializing_if = "Vec::is_empty")]`) в `TriangleMesh`, конструктор `TriangleMesh::with_faces` и пересчёт площадей/центроидов/поверхностных точек `TriangleMesh::recompute_face_registry`, а также обновление метаданных граней в `translate`, `scale`, `rotate_rpy_deg`, `transform` и сохранение в `import_stl::weld_vertices`.
  - [x] В `crates/mcp-cad-core/src/primitives.rs` проставлены канонические `face_ids` и `faces` для `create_box` (6 граней), `create_cylinder` и `create_cylinder_oriented` (3 грани), `create_cone` (3 грани для усечённого конуса, 2 грани для острого и перевёрнутого), `create_sphere` (1 грань `"surface"`), `create_polygon_prism` ($N + 2$ граней `"bottom"`, `"top"`, `"side_0..side_{N-1}"` без фаски и с фаской).
  - [x] В `crates/mcp-cad-core/src/dag.rs` (`FeatureTree::evaluate_with_settings`) реализовано проставление `origin_feature` и квалифицированных алиасов (`feat_{idx}:<role>`, `<slug>:<role>`, `<uuid>:<role>`).
  - [x] В `crates/mcp-cad-server/src/tools/cad.rs` обновлен `query_part_geometry` (вывод массива `"faces"` с пересчётом линейных размеров и площадей в активный `length_unit` и поддержкой селектора `face_selector` / `where`), добавлены коды `FACE_NOT_FOUND` и `AMBIGUOUS_FACE_SELECTOR` в `error.rs`, обновлен `docs/MCP_API_REFERENCE.md`.
  - [x] Добавлены 6 unit-тестов `p2_t6_*` в `primitive_invariants.rs` и контрактный тест `test_mcp_p2_t6_named_faces_query_and_selector` в `mcp_contract_tests.rs`.
- **Компонент**: `crates/mcp-cad-core/src/{topology.rs, primitives.rs, dag.rs, import_stl.rs, lib.rs}`, `crates/mcp-cad-server/src/{tools/cad.rs, tools/mod.rs, error.rs}`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P2-T6-report.md`

#### P2-T7 [L]: Именованные грани: проход через CSG и transform — ✅ DONE ([отчёт](docs/agent_tasks/P2-T7-report.md))
- **Что сделано**:
  - [x] В `crates/mcp-cad-core/src/csg.rs` добавлено поле `face_id: u32` в `CsgPolygon`, обеспечен сквозной пронос `face_id` через `Plane::split_polygon`, `BspNode::polygons_from_mesh` (слияние в квад только при `face_id_1 == face_id_2`), `polygons_to_mesh`, `triangulate_subdivided_convex_loop`, `repair_manifold_triangles` и все 4 прохода `optimize_csg_mesh_quality` (без Delaunay-флипов и сглаживания через границы между разными `face_id`), а также реализована функция слияния реестров `merge_csg_face_registries(a, b, is_difference)` с префиксом `cut:` и инверсией плоских нормалей выживших граней `b` при `csg_difference`.
  - [x] В `crates/mcp-cad-core/src/primitives.rs` размечены канонические грани вырезов `cut_hole_with_settings` (`hole_bottom`, `hole_top`, `hole_wall` с `FaceKind::Cylinder`), `build_keyway_cutter_prism` (`hole_wall`, `keyway_wall`) и `create_helical_cutter_with_settings` (`helical_groove`), а в `FeatureTree::evaluate_with_settings` (`dag.rs`) добавлен `tag_new_cutter_faces` с привязкой `origin_feature`, канонических имён `hole_wall:feat_k` / `cut:feat_k:<role>` и коротких алиасов (`"hole_wall"`).
  - [x] В `crates/mcp-cad-server/src/tools/cad.rs` и `tools/mod.rs` добавлена поддержка шаблонов `"hole_wall:*"` / `"cut:*"` в `resolve_face_by_name`, расширен парсинг `FaceKind` и добавлены опциональные параметры `pivot_face` и `align_face` в `transform_part`.
  - [x] Добавлены 4 теста `p2_t7_*` в `crates/mcp-cad-core/tests/csg_invariants.rs` (брусок минус сквозное отверстие с проверкой площади $2\pi r h$ и уменьшения `top`/`bottom` на $\pi r^2$ с точностью $< 1\%$, глухое и шпоночное отверстие, `csg_difference` + `scale(-1, 1, 1)`, 100% детерминизм `evaluate()`) и контрактный тест `test_mcp_p2_t7_csg_and_cut_hole_named_faces` в `mcp_contract_tests.rs`.
- **Компонент**: `crates/mcp-cad-core/src/{csg.rs, primitives.rs, dag.rs, topology.rs}`, `crates/mcp-cad-server/src/tools/{cad.rs, mod.rs}`, `crates/mcp-cad-core/tests/{csg_invariants.rs, primitive_invariants.rs}`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P2-T7-report.md`

#### P2-T8 [M]: FEA: граничные условия по именам граней — ✅ DONE ([отчёт](docs/agent_tasks/P2-T8-report.md))
- **Что сделано**:
  - [x] В `crates/mcp-cad-fea/src/boundary.rs` добавлены варианты `DirichletConstraint::FixedSurfaceTriangles`, `NeumannLoad::{SurfaceTrianglesForce, SurfaceTrianglesPressure, SurfaceTrianglesMoment}`, плоские комбинации `DofComponent::{XY, XZ, YZ}`, функция проекции `point_to_triangle_distance_3d` и функция сопоставления поверхностных узлов воксельной сетки `match_surface_nodes_and_weights` с отбором одиночного внешнего слоя для плоских граней и взвешиванием по грузовой площади (обеспечивающим сохранение суммы узловых сил с относительной погрешностью $< 10^{-9}$).
  - [x] В `crates/mcp-cad-fea/src/tetra_mesh.rs` обеспечено сохранение `face_ids` и `faces` при разбиении длинных рёбер `refine_mesh_max_edge_length` и гарантировано достаточное поперечное разрешение воксельной сетки для цилиндрических и перфорированных деталей.
  - [x] В `crates/mcp-cad-server/src/tools/fea.rs` и `tools/mod.rs` поддержаны массивы `fixed` и `loads` с унифицированным `FaceSelector` (`{"face": "<name>"}`, `{"faces": [...]}`, `{"face_id": ...}`, `{"where": {...}}`, шаблоны `"hole_wall:*"`), поддержка именованных граней в `fixed_faces` и `applied_forces`/`applied_force`, а также возврат структурированных ошибок `FACE_NOT_FOUND` (со списком `details.available_faces`), `AMBIGUOUS_FACE_SELECTOR` и `FEA_NO_BOUNDARY_NODES`.
  - [x] Добавлены 2 теста `test_p2_t8_*` в `crates/mcp-cad-fea/tests/fea_tests.rs` (осевое растяжение цилиндрического вала за `"cap_bottom"` / `"cap_top"` с проверкой аналитического удлинения $< 5\%$ и напряжения $< 15\%$, нагружение стенки отверстия `"hole_wall"` силой и давлением) и контрактный тест `test_mcp_p2_t8_fea_named_faces_fixed_and_loads` в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`.
- **Компонент**: `crates/mcp-cad-fea/src/{boundary.rs, tetra_mesh.rs, lib.rs}`, `crates/mcp-cad-fea/tests/fea_tests.rs`, `crates/mcp-cad-server/src/{tools/fea.rs, tools/mod.rs, tools/cad.rs, error.rs}`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P2-T8-report.md`

#### P2-T9 [M]: Слой требований (`set_requirements` / `check_requirements`) — ✅ DONE ([отчёт](docs/agent_tasks/P2-T9-report.md))
- **Что сделано**:
  - [x] В `Part` и `PartSummary` (`crates/mcp-cad-core/src/part.rs`) добавлено поле `#[serde(default)] pub revision: u64`, автоматически инкрементируемое при пересборке DAG (`rebuild`), тесселяции (`tessellate`) и прямых модификациях сетки.
  - [x] Создан модуль `crates/mcp-cad-server/src/tools/requirements.rs` и зарегистрированы инструменты №32 `set_requirements` и №33 `check_requirements` с поддержкой типов требований `bbox_max`, `bbox_min`, `mass_max`, `mesh_valid`, `min_safety_factor`, `no_interference` и `mechanism_dof`, возвращающих `{id, type, status: PASS|FAIL|STALE|NOT_AVAILABLE, measured, target, margin, hint}`.
  - [x] В `EngineState` и `run_fea_analysis` добавлено отслеживание `last_fea_revisions` и `fea_case_reports` (при изменении `part.revision` после расчёта МКЭ требование `min_safety_factor` переходит в статус `STALE` без автоматического перезапуска МКЭ).
  - [x] При `auto_check: true` все мутирующие инструменты (`add_feature_primitive`, `cut_hole`, `modify_feature`, `cut_helical_groove`, `boolean_*`, `transform_part`, `batch_cad_operations`, `optimize_feature_tree`, `import_stl`) добавляют `requirements_status` и `failed_requirements` в JSON-ответ; состояние требований сохраняется и восстанавливается в `save_project` / `load_project`.
  - [x] Добавлены 6 unit-тестов в `requirements.rs` и интеграционный контрактный тест `test_mcp_p2_t9_requirements_and_auto_check` в `mcp_contract_tests.rs`.
- **Компонент**: `crates/mcp-cad-core/src/part.rs`, `crates/mcp-cad-server/src/{state.rs, error.rs, tools/}`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P2-T9-report.md`

#### P2-T10 [M]: Документация и сводка сцены для агента (`get_scene_summary`) — ✅ DONE ([гайд](docs/AGENT_GUIDE.md), [отчёт](docs/agent_tasks/P2-T10-report.md))
- **Что сделано**:
  - [x] Создан `docs/AGENT_GUIDE.md` ($\le 2$ страниц, 87 строк): 7 принципов работы с сервером (`get_scene_summary`, единицы `[DECISION-UNITS-1]`, декларация `set_requirements` с `auto_check: true`, транзакции `batch_cad_operations`, обязательный `validate_mesh` после CSG/вырезов, МКЭ по именованным граням со статусом `STALE`, чтение `ErrorEnvelope`), стандартный 5-шаговый цикл «Build → Validate → Analyze → Iterate» и 3 коротких примера вызовов.
  - [x] В `crates/mcp-cad-server/src/protocol.rs` добавлена константа `AGENT_INSTRUCTIONS` (~1480 символов), возвращаемая в `initialize.instructions` со ссылкой на `docs/AGENT_GUIDE.md`.
  - [x] В `crates/mcp-cad-server/src/tools/mod.rs` нормализованы описания всех 34 MCP-инструментов (`Units:`, `Example:`, `Common errors:`, `Next step:`) и описания всех числовых полей в `inputSchema` (каждое числовое поле указывает единицу измерения или `dimensionless`).
  - [x] В `crates/mcp-cad-server/src/tools/metrics.rs` реализован инструмент №34 `get_scene_summary`, возвращающий за один вызов текстовую (`summary_text`) и JSON-сводку единиц проекта, всех деталей (`part_id`, `name`, `bounding_box`, `dimensions`, `volume`, `mass_kg`, `mesh_valid`, `last_fea` с `verdict`, `factor_of_safety`, `stale`), шарниров и статуса требований (`requirements`).
  - [x] Добавлены unit-тест `test_get_scene_summary_unit_conversion_and_joints` в `metrics.rs` и контрактные тесты `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` и `test_mcp_p2_t10_get_scene_summary_two_parts` в `mcp_contract_tests.rs`.
- **Компонент**: `docs/AGENT_GUIDE.md`, `crates/mcp-cad-server/src/{protocol.rs, tools/mod.rs, tools/metrics.rs}`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P2-T10-report.md`

#### P2-T11 [S]: Валидация числовых параметров в `set_joint_motor` (`target_velocity`, `max_force >= 0`) — ✅ DONE ([отчёт](docs/agent_tasks/P2-T11-report.md))
- **Проблема**: Выявлено в **P0-T11**: в `crates/mcp-cad-server/src/tools/physics.rs:293–320` параметры `target_velocity` и `max_force` парсились как `as_f64()`, но не проверялись на `is_finite()` (включая переполнение при сужении `f64 as f32` в решателе Rapier3D) и неотрицательность `max_force >= 0.0`.
- **Что сделано**:
  - [x] В `crates/mcp-cad-server/src/tools/physics.rs` (`set_joint_motor`) добавлена проверка `is_finite()` (как в `f64`, так и при приведении к `f32`) для `target_velocity` и `max_force`, а также проверка `max_force >= 0.0` с возвратом структурированной ошибки `ServerError::InvalidArgument` (`code: "INVALID_ARGUMENT"`, `param: "target_velocity" | "max_force"`).
  - [x] В `crates/mcp-cad-server/src/error.rs` (`map_invalid_argument_error`) добавлены предметные подсказки `hint` для невалидных `target_velocity` и `max_force`, а в `tools/mod.rs` и `docs/MCP_API_REFERENCE.md` отражено ограничение `minimum: 0.0`.
  - [x] Добавлены unit-тест `test_set_joint_motor_rejects_negative_and_non_finite_parameters` в `physics.rs` и контрактный тест `test_mcp_p2_t11_set_joint_motor_numeric_validation` в `mcp_contract_tests.rs`.
- **Компонент**: `crates/mcp-cad-server/src/{tools/physics.rs, tools/mod.rs, error.rs}`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P2-T11-report.md`

#### P2-T12 [S]: Откат `physics_world` в `batch_cad_operations`, сохранение `!part.is_fixed` при CSG/transform и удаление мёртвого `tools/export.rs` — ✅ DONE ([отчёт](docs/agent_tasks/P2-T12-report.md))
- **Проблема**: Выявлено при аудите `crates/mcp-cad-server/src/tools/` в рамках **P1-T1**:
  1. В `batch_cad_operations` (`crates/mcp-cad-server/src/tools/cad.rs:1709-1765`) при ошибке шага откатываются `parts`, `materials`, `last_fea_reports`, `last_fea_meshes`, `last_simulation`, но **не откатывается `state.physics_world`**, что оставляет физический мир Rapier3D рассинхронизированным со списком деталей.
  2. В `boolean_union` (`cad.rs:1464`), `boolean_intersection` (`:1530`), `transform_part` (`:1595`) и `optimize_feature_tree` (`:1809`) деталь перерегистрируется в `physics_world` вызовом `add_part(part, true)` вместо `add_part(part, !part.is_fixed)`, сбрасывая закрепление неподвижных деталей (`is_fixed == true`), а в `boolean_difference` (`:1052-1170`) обновление `physics_world` пропущено вовсе.
  3. Модуль `crates/mcp-cad-server/src/tools/export.rs` (58 строк) является мёртвым дубликатом `cad::export_stl` (`cad.rs:1224`) без проверки `validate_mesh`.
- **Что сделано**:
  - [x] В `crates/mcp-cad-physics/src/simulator.rs` реализован `Clone` для `PhysicsWorld` (с клонированием всех наборов Rapier3D и пересозданием неблокирующих каналов `crossbeam::channel::unbounded()` и `PhysicsPipeline::new()`) и добавлены методы `has_part` / `is_part_dynamic`.
  - [x] В `crates/mcp-cad-server/src/tools/cad.rs` добавлен снимок `prev_physics_world = state.physics_world.clone()` и его восстановление на всех путях аварийного выхода из `batch_cad_operations`, а также синхронизация `physics_world` с сохранением `!part.is_fixed` в `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part` и `optimize_feature_tree`.
  - [x] Удалён мёртвый файл `crates/mcp-cad-server/src/tools/export.rs` (и `pub mod export;` из `tools/mod.rs`), добавлены unit-тест `test_p2_t12_fixed_part_preserved_and_batch_physics_rollback` в `tools/cad.rs` и контрактный тест `test_mcp_p2_t12_batch_physics_rollback_and_fixed_part_preservation` в `mcp_contract_tests.rs`.
- **Компонент**: `crates/mcp-cad-physics/src/simulator.rs`, `crates/mcp-cad-server/src/tools/{cad.rs, mod.rs, export.rs}`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `docs/agent_tasks/P2-T12-report.md`

#### P2-T13 [M]: Поддержка пользовательского `joint_id` в `create_joint`, `through: true` в `cut_hole` и алиасов параметров (`revolve.inner_radius`, `polygon_prism.outer_radius`/`chamfer_radius`, ID материалов) — ✅ DONE ([отчёт](docs/agent_tasks/P2-T13-report.md))
- **Проблема**: Выявлено при прогоне 15 эталонных задач `bench/tasks.json` против нативного Rust-сервера в рамках **P1-T2** ([`docs/BENCH_BASELINE_RUST.md`](docs/BENCH_BASELINE_RUST.md)):
  1. В `crates/mcp-cad-server/src/tools/physics.rs:264` инструмент `create_joint` полностью игнорирует переданный аргумент `args.get("joint_id")` и всегда создаёт случайный `joint-<uuid>`, из-за чего последующий вызов `set_joint_motor` в задачах `T14` и `T15` падает с `Joint with id '...' was not found in physics world`.
  2. В `crates/mcp-cad-server/src/tools/cad.rs:789` инструмент `cut_hole` требует обязательный числовой `depth` и не поддерживает `"through": true` (а также при старте из середины пластины `z = 0` с `direction = [0, 0, -1]` прорезает только нижнюю половину детали в `T05`, `T06`, `T10`, `T13`).
  3. В `add_feature_primitive`: `revolve` (`cad.rs:647`) не поддерживает `inner_radius` при 2-точечном образующем отрезке (`T07`), `polygon_prism` (`cad.rs:692`) не поддерживает алиасы `outer_radius` и `chamfer_radius` (`T08`), а `crates/mcp-cad-core/src/material.rs` не распознаёт алиасы `"Al6061-T6"` (`"Al-6061-T6"`) и `"Ti-6Al-4V"` (`"Ti6Al4V"`).
- **Что сделано**:
  - [x] В `crates/mcp-cad-core/src/material.rs` (и `src/cad/materials.ts`) добавлено разрешение канонических алиасов материалов (`steel_1045`, `steel_4140`, `alu_6061`, `Al6061-T6`, `Al-6061-T6`, `ti_6al_4v`, `Ti-6Al-4V`, `Ti6Al4V`, `brass_c36000`, `pla_3d`, `pla_plastic`, `petg_3d`, `petg_plastic`, `nylon_pa6`) без дублирования пресетов в `list()`.
  - [x] В `crates/mcp-cad-server/src/tools/physics.rs` (и `src/cad/mcp_dispatcher.ts`) поддержан опциональный пользовательский `joint_id` (и алиас `id`) в `create_joint` с валидацией непустой строки и проверкой уникальности (`JOINT_ALREADY_EXISTS`).
  - [x] В `crates/mcp-cad-server/src/tools/cad.rs` (и `src/cad/{primitives.ts, mcp_dispatcher.ts}`) поддержан режим `"through": true` (с автоматическим вычислением сквозной глубины по проекции сетки детали вдоль оси сверления), алиасы `"direction"` и `"diameter"` в `cut_hole`, параметр `inner_radius` при 2-точечном профиле в `revolve` и алиасы `outer_radius` / `chamfer_radius` в `polygon_prism` (как в `add_feature_primitive`, так и в `modify_feature`).
  - [x] Обновлены схемы инструментов в `crates/mcp-cad-server/src/tools/mod.rs` и добавлены модульные и контрактные тесты (`test_material_preset_aliases`, `test_create_joint_custom_joint_id_and_duplicate_rejection`, `test_p2_t13_through_hole_revolve_inner_radius_and_polygon_prism_aliases`, `test_mcp_p2_t13_custom_joint_id_through_hole_and_aliases`).
- **Компонент**: `crates/mcp-cad-server/src/tools/{physics.rs, cad.rs, mod.rs}`, `crates/mcp-cad-core/src/material.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `src/cad/{materials.ts, primitives.ts, mcp_dispatcher.ts, cad_engine.test.ts}`, `docs/agent_tasks/P2-T13-report.md`

#### P2-T14 [S]: Учёт явных `segments`/`sectors`/`stacks` в `FeatureTree::evaluate_with_settings` (`dag.rs`) и связности/относительного допуска в `inspect_geometry` (`topology.rs`)
- **Проблема**: Выявлено при аудите сигнатур `mcp-cad-core` в рамках **P1-T4** ([`docs/agent_tasks/P1-T4-report.md`](docs/agent_tasks/P1-T4-report.md)):
  1. В `crates/mcp-cad-core/src/dag.rs:396-572` метод `FeatureTree::evaluate_with_settings` деструктурирует поля `segments: _` в `FeatureOp::{AddCylinder, AddCone, AddRevolve}` и `sectors: _, stacks: _` в `FeatureOp::AddSphere`, безусловно перезаписывая их адаптивным расчётом `settings.calculate_segments(...)`. Из-за этого явная передача `"segments"` / `"sectors"` / `"stacks"` в `add_feature_primitive` или `modify_feature` (`crates/mcp-cad-server/src/tools/cad.rs:640, 655, 673, 775`) сохраняется в узле DAG, но полностью игнорируется при построении сетки детали.
  2. В `crates/mcp-cad-core/src/topology.rs:440-498` метод `TriangleMesh::inspect_geometry` кластеризует треугольники в `detected_planar_faces` только по нормали и фиксированному размерному допуску плоскости `|n · p - d| < 1e-4` без проверки связности по рёбрам (сливая раздельные копланарные площадки на одной высоте и ступеньки `< 0.1 мм` при работе в метрах), а для кольцевых торцов с центральным отверстием возвращает `center` в пустоте отверстия `[0, 0, z]`.
- **Что сделать**: В `FeatureTree::evaluate_with_settings` использовать явно заданные в узле `segments`/`sectors`/`stacks` (либо масштабировать их только тогда, когда в узле стоит маркер авто-тесселяции `0`), а в `inspect_geometry` масштабировать плоскостной допуск от диагонали AABB (`diag * 1e-5`) и разделять несвязные копланарные компоненты связности.
- **Компонент**: `crates/mcp-cad-core/src/{dag.rs, topology.rs}`

#### P2-T15 [S]: Защита от паники `f64::clamp` (`min > max`) в `create_wheel_rim_with_settings` (`primitives.rs:3052`) и исправление оси мотора `JointAxis::LinX` для `Prismatic` в `Simulator::set_joint_motor` (`simulator.rs:223`)
- **Проблема**: Выявлено при построчном аудите размерных констант в рамках **P2-T1** ([`docs/UNITS.md`](docs/UNITS.md), [`docs/agent_tasks/P2-T1-report.md`](docs/agent_tasks/P2-T1-report.md)):
  1. В `crates/mcp-cad-core/src/primitives.rs:3052` при генерации обода колеса вычисляется `let d_pcd = pcd_diameter.unwrap_or((d_hub + d_bore) * 0.5).clamp(d_bore + 0.015, d_hub - 0.012);` с захардкоженными в метрах зазорами `15 мм` и `12 мм`. При создании уменьшенной модели колеса с `rim_diameter < 0.1403 м` (`140.3 мм`) нижняя граница `d_bore + 0.015` становится строго больше верхней `d_hub - 0.012`, что вызывает runtime-панику `f64::clamp` (`min > max`). Аналогичные абсолютные зазоры (`0.005..0.035 м`) зашиты в строках `3056–3108`.
  2. В `crates/mcp-cad-physics/src/simulator.rs:219-231` метод `Simulator::set_joint_motor` при обновлении мотора на существующем шарнире в `impulse_joint_set` безусловно вызывает `joint.data.set_motor_velocity(JointAxis::AngX, ...)` и `set_motor_max_force(JointAxis::AngX, ...)` даже для поступательных шарниров `JointDesc::Prismatic` (которые при создании на строке `164` используют ось `JointAxis::LinX`). Кроме того, в `create_joint` (`crates/mcp-cad-server/src/tools/physics.rs:248-259`) пределы `limits` для `revolute` передаются в Rapier3D как радианы без конвертации из градусов.
- **Что сделать**: В `create_wheel_rim_with_settings` заменить абсолютные зазоры `0.015` / `0.012` / `0.014` на относительные к `d_rim` / `d_hub` (с безопасной проверкой `min <= max` перед `.clamp()`), а в `Simulator::set_joint_motor` выбирать `JointAxis::LinX` для `JointDesc::Prismatic` и `JointAxis::AngX` для `JointDesc::Revolute`.
- **Компонент**: `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-physics/src/simulator.rs`

#### P2-T16 [S]: Строгая валидация нераспознанных `fixed_faces` и пустых `applied_forces` в `run_fea_analysis` (`tools/fea.rs:99,130`) и учёт соседних ячеек в `weld_vertices` (`import_stl.rs:268`)
- **Проблема**: Выявлено при аудите `crates/mcp-cad-server/src/tools/fea.rs` и `crates/mcp-cad-core/src/import_stl.rs` в рамках **P2-T5** ([`docs/agent_tasks/P2-T5-report.md`](docs/agent_tasks/P2-T5-report.md)):
  1. В `crates/mcp-cad-server/src/tools/fea.rs:99-108` любая строка в массиве `fixed_faces`, не распознанная `parse_aabb_face(f_str)` (например, `"cap_bottom"` до внедрения `P2-T8` или опечатка `"min_z "`), молча игнорируется вместо возврата структурированной ошибки `ServerError::InvalidArgument`.
  2. В `crates/mcp-cad-server/src/tools/fea.rs:119, 130-180` при передаче непустого массива `applied_forces` (`explicit_loads_key = true`), в котором элементы используют неверный ключ (например, `"force"` вместо `"force_vector"` или некорректный `"position"`), ни одна нагрузка не добавляется (`loads_added == false`), а резервная нагрузка по умолчанию на строке 178 (`if !loads_added && !explicit_loads_key`) пропускается. В результате МКЭ-расчёт запускается с нулевым вектором внешних сил и возвращает ложноположительный `max_von_mises_mpa: 0.0`, `status: "PASS"`.
  3. В `crates/mcp-cad-core/src/import_stl.rs:268-315` функция `weld_vertices` квантует вершины в единственный округлённый ключ `((v[0] * inv_tol).round() as i64, ...)` без проверки соседних ячеек `(-1..=1)^3` (из-за чего вершины на расстоянии `1e-15` по разные стороны полуцелого порога округления не свариваются), а на строке 313 вызывает `recompute_vertex_normals()`, сглаживая нормали через острые $90^\circ$ рёбра.
- **Что сделать**: В `run_fea_analysis` возвращать `ServerError::InvalidArgument` при любом нераспознанном элементе `fixed_faces` и при `explicit_loads_key && !loads_added` (если `applied_forces` не пуст, но ни одна нагрузка не распарсилась); в `weld_vertices` проверять 27 соседних ячеек сетки и сохранять вершинные нормали острых рёбер.
- **Компонент**: `crates/mcp-cad-server/src/tools/fea.rs`, `crates/mcp-cad-core/src/import_stl.rs`

#### P2-T17 [S]: Транзакционность `record.motor` при отказе `apply_motor` в `PhysicsWorld::set_joint_motor` (`simulator.rs:325`) и запрет `motor.is_some()` для `CadJointType::Gear` в `build_generic_joint` (`joint.rs:200`)
- **Проблема**: Выявлено при выполнении **P2-T11** ([`docs/agent_tasks/P2-T11-report.md`](docs/agent_tasks/P2-T11-report.md)):
  1. В `PhysicsWorld::set_joint_motor` (`crates/mcp-cad-physics/src/simulator.rs:325`) запись `record.motor = Some(motor);` выполняется **до** вызова `joint_type.apply_motor(&mut joint.data, motor)`. Если `set_joint_motor` вызван на шарнире `Spherical`, `Fixed` или `Gear`, `apply_motor` возвращает `Err(PhysicsError::InvalidJointConfiguration(...))`, но `record.motor` остаётся записанным в `PhysicsWorld::joints`. Из-за этого `get_scene_summary` (`metrics.rs:316`) ложно показывает `has_motor: true`, а `save_project` (`project.rs:146`) сохраняет `motor` для неподдерживаемого типа шарнира, ломая последующий `load_project`.
  2. В `CadJointType::build_generic_joint` (`crates/mcp-cad-physics/src/joint.rs:200-204`) ветка `CadJointType::Gear { .. }` не проверяет `if motor.is_some()` (в отличие от `Spherical` и `Fixed`), хотя `apply_motor` запрещает установку мотора на `Gear`.
- **Что сделать**: В `PhysicsWorld::set_joint_motor` обновлять `record.motor = Some(motor)` только после успешного возврата `joint_type.apply_motor(&mut joint.data, motor)?`, а в `CadJointType::build_generic_joint` добавить проверку `if motor.is_some()` для `CadJointType::Gear`.
- **Компонент**: `crates/mcp-cad-physics/src/{simulator.rs, joint.rs}`

#### P2-T18 [M]: Сохранение привязанных шарниров в `physics_world` при перерегистрации детали в CSG/transform и обновление коллайдера в `add_feature_primitive`/`cut_hole`/`modify_feature`/`cut_helical_groove`
- **Проблема**: Выявлено при выполнении **P2-T12** ([`docs/agent_tasks/P2-T12-report.md`](docs/agent_tasks/P2-T12-report.md)):
  1. В `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part` и `optimize_feature_tree` (`crates/mcp-cad-server/src/tools/cad.rs`) перерегистрация модифицированной детали в `state.physics_world` выполняется через `remove_part` + `add_part`. Поскольку `PhysicsWorld::remove_part` (`crates/mcp-cad-physics/src/simulator.rs:261-273`) удаляет все записи `joints` и `gear_constraints`, связанные с `part_id`, любой шарнир, созданный *до* вызова `transform_part` / `boolean_*` / `optimize_feature_tree` на этой детали, негласно теряется.
  2. Инструменты модификации геометрии `add_feature_primitive`, `cut_hole`, `modify_feature`, `cut_helical_groove` и `tessellate_part` обновляют `part.mesh`, но не обновляют коллайдер и массу детали в `state.physics_world`, если деталь уже была зарегистрирована в физическом мире ранее.
- **Что сделать**: Реализовать в `PhysicsWorld` метод обновления геометрии и массы существующего тела `update_part_collider(&mut self, part: &Part)` без удаления `RigidBodyHandle` и привязанных к нему шарниров (`joints` / `gear_constraints`), и вызывать его во всех мутирующих гео-операциях `tools/cad.rs`.
- **Компонент**: `crates/mcp-cad-physics/src/simulator.rs`, `crates/mcp-cad-server/src/tools/cad.rs`

#### P2-T19 [S]: Транзакционный откат `part.dag`/`part.mesh` при ошибке `rebuild()` в `modify_feature` и авто-выравнивание `start_z` по нижней грани детали в `cut_helical_groove`
- **Проблема**: Выявлено при выполнении **P2-T13** ([`docs/agent_tasks/P2-T13-report.md`](docs/agent_tasks/P2-T13-report.md)):
  1. В `modify_feature` (`crates/mcp-cad-server/src/tools/cad.rs:1006–1370`) после вызова `part.dag.update_feature(feature_id, updated_op)` ошибка перестроения `part.rebuild()?` пробрасывается без восстановления `part.dag` и `part.mesh` к состоянию до модификации (в отличие от `add_feature_primitive`, `cut_hole` и `cut_helical_groove`, где сохраняются `prev_dag` и `prev_mesh`).
  2. В `cut_helical_groove` (`crates/mcp-cad-server/src/tools/cad.rs:1488–1492`) при опущенном параметре `start_z` подставляется `0.0` вместо нижней границы детали `part.bounding_box().min[2]` (или `-height * 0.5`), из-за чего на деталях, центрированных вокруг `z = 0` (например, гайка `T08_hex_nut_m10` с диапазоном `[-4 мм, +4 мм]`), винтовая канавка без явного `start_z` прорезает только верхнюю половину детали `[0, +height/2]`.
- **Что сделать**: Сохранять `prev_dag` и `prev_mesh` перед `part.dag.update_feature` в `modify_feature` с откатом при ошибке `part.rebuild()`, а в `cut_helical_groove` при отсутствии `start_z` автоматически вычислять начальную координату `part.bounding_box().min[2]` (или `-height * 0.5`).
- **Компонент**: `crates/mcp-cad-server/src/tools/cad.rs`

---

### Фаза 3: Достоверность Расчётов Прочности и Механизмов (P3)

#### P3-T1 [M]: Базовая точность FEA: эталонные задачи
- **Что сделать**: Создать `crates/mcp-cad-fea/tests/fea_benchmarks.rs` и `docs/FEA_ACCURACY.md` с замерами относительных ошибок на 4 эталонах (растяжение стержня, изгиб консоли, пластина Кирсча с отверстием, толстостенный цилиндр).
- **Компонент**: `crates/mcp-cad-fea/tests/fea_benchmarks.rs`, `docs/FEA_ACCURACY.md`

#### P3-T2 [M]: Предполётные проверки и баланс сил FEA
- **Что сделать**: Выполнять `validate_mesh` перед МКЭ, проверять невязку равновесия реакций опор $\sum R + \sum F$, определять зоны сингулярности закреплений.
- **Компонент**: `crates/mcp-cad-fea/src/`

#### P3-T3 [M]: Оценка сходимости в результате FEA
- **Что сделать**: Вывести результаты `convergence.rs` в MCP (`convergence_study: true`) с вердиктом `CONVERGED`/`NOT_CONVERGED` по изменению напряжений и перемещений.
- **Компонент**: `crates/mcp-cad-fea/src/convergence.rs`

#### P3-T4 [L]: Элемент TET10 (квадратичные 10-узловые тетраэдры)
- **Что сделать**: Реализовать 10-узловой элемент `Tet10Element` с 4-точечным интегрированием Гаусса в `element10.rs`, конвертер сетки TET4 $\to$ TET10 и выбор порядка элемента `element_order: 1|2`.
- **Компонент**: `crates/mcp-cad-fea/src/element10.rs`, `solver.rs`

#### P3-T5 [M]: Исследование: сетка, согласованная с границей **[DECISION]**
- **Что сделать**: Подготовить `docs/adr/0002-fe-mesher.md` и прототип `spikes/fe_snap/` с проецированием узлов воксельной сетки на исходную CAD-поверхность.
- **Компонент**: `docs/adr/0002-fe-mesher.md`, `spikes/fe_snap/`

#### P3-T6 [M]: Нагрузки из симуляции: извлечение
- **Что сделать**: Реализовать инструмент `get_part_load_cases` для извлечения пиковых реакций в суставах и инерционных сил из симуляции Rapier3D.
- **Компонент**: `crates/mcp-cad-physics/src/telemetry.rs`, `mcp-cad-server`

#### P3-T7 [L]: Нагрузки из симуляции: приложение к FEA
- **Что сделать**: Инструмент `run_fea_from_simulation` для автоматической передачи реакций суставов с инерционной разгрузкой (d'Alembert) в МКЭ решатель.
- **Компонент**: `crates/mcp-cad-fea/`, `mcp-cad-server`

#### P3-T8 [M]: Проверка прочности зубьев (Lewis / AGMA)
- **Что сделать**: Инструмент `check_gear_strength` для быстрой аналитической проверки изгибных напряжений Льюиса и контактных напряжений Герца/AGMA для шестерен.
- **Компонент**: `crates/mcp-cad-core/src/primitives.rs`, `mcp-cad-server`

#### P3-T9 [M]: Механизм: степени свободы (Грюблер & SVD)
- **Что сделать**: Инструмент `analyze_mechanism` с численной оценкой ранга Якобиана связей через SVD (`nalgebra`) и сравнением с формулой Грюблера.
- **Компонент**: `crates/mcp-cad-physics/src/joint.rs`, `mcp-cad-server`

#### P3-T10 [L]: Механизм: интерференция и ход
- **Что сделать**: Инструменты `check_interference` (статические коллизии) и `sweep_joint` (поиск угла/хода первого столкновения).
- **Компонент**: `crates/mcp-cad-physics/src/simulator.rs`, `mcp-cad-server`

#### P3-T11 [M]: Водонепроницаемость `cut_helical_groove` при нарезке внутренней резьбы гайки (`T08_hex_nut_m10`)
- **Проблема**: Выявлено в рамках **P1-T2** ([`docs/BENCH_BASELINE_RUST.md`](docs/BENCH_BASELINE_RUST.md)) и подтверждено в `qa/results/operations_mesh_scan.json`: в задаче `T08_hex_nut_m10` вызов `cut_helical_groove` после `polygon_prism` и `cut_hole` (`crates/mcp-cad-core/src/dag.rs:664-676`, `csg_difference(&current, &cutter)`) разрушает водонепроницаемость сетки (`is_watertight = false`, `open_boundary_edges = 413`, `genus = 68` при ожидаемом `1`).
- **Что сделать**: Исследовать и устранить нарушение манифольдности при булевом вычитании винтового тела `create_helical_groove` (или реализовать прямое конформное построение резьбового канала в отверстии без вырожденных копланарных/касательных сечений BSP, либо переключить булево ядро на `manifold-rust` согласно `P1-T3`), добившись `validate_mesh == PASS`, `is_watertight == true` и `genus == 1` на `T08_hex_nut_m10`.
- **Компонент**: `crates/mcp-cad-core/src/{dag.rs, primitives.rs, csg.rs}`

#### P3-T12 [S]: Открытые рёбра BSP-CSG на составной фланцевой втулке и учёт `connected_components` в формуле `genus` (`diagnostics.rs:414`)
- **Проблема**: Выявлено при экспериментальном сравнении ядер в рамках **P1-T3** ([`docs/adr/0001-geometry-kernel.md`](docs/adr/0001-geometry-kernel.md), [`docs/agent_tasks/P1-T3-report.md`](docs/agent_tasks/P1-T3-report.md)):
  1. При выполнении цепочки `csg_union(&create_cylinder(15.0, 50.0, 64), &create_box(40.0, 40.0, 10.0))` и `csg_difference(&u, &create_cylinder(10.0, 52.0, 64))` в `crates/mcp-cad-core/src/csg.rs` сетка теряет водонепроницаемость (`open_boundary_edges = 24`, `is_watertight = false`, $\chi = -8$, `validate_mesh = FAIL`), тогда как `manifold-rust` на тех же входных сетках даёт `open_boundary_edges = 0`, `is_watertight = true`, $\chi = 0, g = 1.0$.
  2. В `crates/mcp-cad-core/src/diagnostics.rs:414` топологический род вычисляется по однокомпонентной формуле `let genus = (2.0 - euler_characteristic as f64) / 2.0;` без учёта уже вычисленного на строке 409 числа связных компонент `connected_components` ($k$), хотя для $k$ замкнутых компонент действует обобщённая формула Эйлера $\chi = 2k - 2g \implies g = k - \chi / 2$.
- **Что сделать**: Исправить формулу `genus` в `crates/mcp-cad-core/src/diagnostics.rs:414` на `(2.0 * connected_components as f64 - euler_characteristic as f64) / 2.0` (при `connected_components > 0`), добавить unit-тест на несвязное объединение двух тел ($k = 2$), а также устранить потерю `24` граничных рёбер при цепочке `union + difference` фланцевой втулки.
- **Компонент**: `crates/mcp-cad-core/src/{diagnostics.rs, csg.rs}`

---

### Фаза 4: Рецепты и Заготовки (P4)

#### P4-T0 [M]: Каркас «рецептов» и три заготовки
- **Что сделать**: Внедрить инструмент `apply_recipe` для генерации проверенных типовых инженерных узлов (например, вал с подшипником, зубчатая пара, фланец с разболтовкой).
- **Компонент**: `crates/mcp-cad-server/src/tools/`
