# MCP-CAD: План работ и реестр задач (Backlog)

> **Единый источник правды по архитектуре и принятым решениям:** Все фундаментальные архитектурные решения, таблицы систем координат примитивов, семантика Solid DAG и контракты 43 инструментов MCP перенесены и подробно описаны в **[`PROJECT_CONTEXT.md`](PROJECT_CONTEXT.md)** (разделы 4, 5 и 8).  
> **Пошаговые отчёты о выполненных задачах (`P0-T1..T12`, `P1-T1..T4`, `P2-T1..T40`, `P3-T1..T24`, `P4-T0..T10`, `P5-T1..T20`, `P5-T27..T35`):** хранятся в [`docs/agent_tasks/`](docs/agent_tasks/) и сведены в [Разделе 3](#3-архивный-реестр-выполненных-задач-completed-tasks-p0p5).  
> **Текущий статус верификации (24.11.2026):**  
> - **Последняя выполненная задача — `P5-T35`** (устранение эвристики `> 1.0 ? 1e-3 : 1.0` для призматических `joint.limits` в `src/cad/kinematics.ts:88-97` и `src/cad/press_simulation.ts:441-450`, трактовка `KinematicJoint.limits` строго в метрах СИ для ходов `> 1.0 м`, плюс unit-тест `P5-T35` в `src/cad/cad_engine.test.ts`):  
>   - Полный прогон всех 11 TS-сьютов (`cad_engine`, `analyze_mechanism`, `destruction_engine`, `ortho_views`, `field_colormaps`, `coupled_sim_stream`, `helix_primitive`, `live_sync_adapter`, `press_simulation`, `rust_scene_sync`, `live_sync_store`): **115/115 PASS** (`0 failed`: `85/85` в 5 базовых + `30/30` в 6 расширенных сьютах);  
>   - `npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`): **PASS** (`0` ошибок);  
>   - Нативные Rust-сьюты (`mcp-cad-core`: `137/137`, `mcp-cad-fea`: `66/66`, `mcp-cad-physics`: `47/47`, `mcp-cad-viewer`: `14/14`, `mcp-cad-server`: `154/154`, `cargo clippy --workspace --all-targets -- -D warnings`: `0` предупреждений): **PASS** (подробные логи по каждой задаче — в [`docs/agent_tasks/`](docs/agent_tasks/)).

---

## 1. Актуальный бэклог задач (Active Backlog)

### Сводная таблица открытых задач

| ID | Фаза | Задача | Зависит от | Размер | Компонент | Статус |
|---|---|---|---|---|---|---|
| **P5-T21** | P5 High-Accuracy FEM | Согласованное `T6`-интегрирование линейного поля усилий в `NeumannLoad::SurfaceTrianglesMoment` (`crates/mcp-cad-fea/src/boundary.rs:764-813`) для `TET10` вместо сосредоточенных весов `area_weight` (нулевых на угловых узлах `T6`) | P5-T12 | S | `mcp-cad-fea` | 📋 TODO |
| **P5-T22** | P5 High-Accuracy FEM | Самоконтакт берегов внутренних трещин и ограничение повторного невыровненного расщепления граней в `FractureTopologyMode::FaceSplitting` (`fracture.rs`, `contact.rs`) | P5-T13 | S | `mcp-cad-fea` | 📋 TODO |
| **P5-T23** | P5 High-Accuracy FEM | Подавление нефизического контактного удара при случайном перекрытии грубых сеток в $t=0$ и учёт геометрической жёсткости вращающихся RBE2-ступиц в `solve_dynamic_hht_coupled` (`dynamics.rs`) | P5-T14 | S | `mcp-cad-fea` | 📋 TODO |
| **P5-T24** | P5 Contract / Docs | Синхронизация единиц Фазы 5 в верхнеуровневом `description` инструмента `define_material` и добавление алиаса `hardening_modulus_gpa` (`crates/mcp-cad-server/src/tools/{mod.rs, cad.rs}`) | P5-T1, P5-T17 | S | `mcp-cad-server` | 📋 TODO |
| **P5-T25** | P5 Contract / Docs | Документирование параметров `youngs_modulus_gpa` и `poissons_ratio` в JSON-схеме `check_gear_strength` (`crates/mcp-cad-server/src/tools/mod.rs:1414-1487`) и `docs/MCP_API_REFERENCE.md` | P5-T18 | S | `mcp-cad-server`, Docs | 📋 TODO |
| **P5-T26** | P5 Contract Hygiene | Валидация пустых/пробельных `name` и `part_id` в `create_part` (`crates/mcp-cad-server/src/tools/cad.rs:317-345`) и документирование `part_id` в таблице `create_part` (`docs/MCP_API_REFERENCE.md:263-268`) | P5-T19 | S | `mcp-cad-server`, Docs | 📋 TODO |
| **P5-T36** | P5 TS Dispatcher | Масштабирование `anchor_a`/`anchor_b` по `length_unit` (`kToM`) и сохранение отдельного поля `anchorB` в TS `create_joint` (`src/cad/mcp_dispatcher.ts:1453-1466`) в паритете с Rust (`physics.rs:202-264`) | P5-T30 | S | `src/cad/` | 📋 TODO |
| **P5-T37** | P5 TS Dispatcher | Отклонение явных пустых `id: ""` / `material_id: ""` в TS `define_material` (`src/cad/mcp_dispatcher.ts:1301`) вместо молчаливой генерации `mat_${Date.now()}` (паритет с Rust `cad.rs:3518-3520`) | P5-T31 | S | `src/cad/` | 📋 TODO |
| **P5-T38** | P5 Live Sync | Проброс нелинейных параметров Фазы 5 (`ultimate_strength_pa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`) в `LiveMaterialInfo` (`live_sync.rs:76-83`, `live_sync_adapter.ts:46-53, 202-211`) | P5-T1, P5-T31 | S | `mcp-cad-server`, `src/cad/` | 📋 TODO |
| **P5-T39** | P5 Docs | Актуализация таблицы параметров `### 42. bake_coupled_simulation` в `docs/MCP_API_REFERENCE.md:2291-2304` (10 отсутствующих параметров из `mod.rs:1823-1934` и актуальное поведение `async_mode`/`wait_timeout_ms`) | P5-T6, P5-T32 | S | Docs | 📋 TODO |
| **P5-T40** | P5 Docs | Исправление двух битых групповых якорей оглавления `docs/MCP_API_REFERENCE.md:16, 31` (`Группа 2` и `Группа 5`) и добавление подпункта `Контракт единиц измерения` | P5-T32 | S | Docs | 📋 TODO |
| **P5-T41** | P5 TS Core | Пересчёт `mass` и `inertiaTensor` по новой `density` (и прогон последующих узлов DAG) в `evaluateFeatureTree` (`src/cad/primitives.ts:3474-3476`) для деталей с корневым узлом `stl_import` | P5-T33 | S | `src/cad/` | 📋 TODO |
| **P5-T42** | P5 Web UI | Скрытие внутреннего буфера `params.mesh` в инспекторе узла DAG (`src/components/CadLeftSidebar.tsx:305`) и актуализация счётчика `(41 Rust)` → `(43 Rust)` в `src/components/CadRightSidebar.tsx:1269` | P5-T33 | S | `src/components/` | 📋 TODO |
| **P5-T43** | P5 Contract / Rust | Добавление `limits_unit` в JSON-схему `create_joint` (`mod.rs:389-473`), исправление формулы `ratio` в `mod.rs:452` (`omega_a / omega_b` → `omega_b = -ratio * omega_a`) и строгая валидация `limits_unit` (`"deg" \| "rad"`) в `physics.rs:232-245` | P5-T34 | S | `mcp-cad-server` | 📋 TODO |
| **P5-T44** | P5 TS Dispatcher | Строгая валидация `limits_unit` (`'deg' \| 'rad'`), вычисление `gearRatio = teeth_a / teeth_b` (с проверкой `> 0`) и учёт `axis_a`/`axis_b` в TS `create_joint` (`src/cad/mcp_dispatcher.ts:1457-1502`) | P5-T34 | S | `src/cad/` | 📋 TODO |
| **P5-T45** | P5 Rust Scene Sync | Передача `limits_unit: 'rad'` для вращательных шарниров (`joint.type === 'revolute'`) при синхронизации `joint.limits` в `buildRustSceneSyncBatch` (`src/cad/rust_scene_sync.ts:678-680`) | P5-T35 | S | `src/cad/` | 📋 TODO |
| **P5-T46** | P5 TS Kinematics | Зажим угла вращательного шарнира (`joint.type === 'revolute'`) по `joint.limits` в `stepKinematicsAssembly` (`src/cad/kinematics.ts:41-68`) | P5-T35 | S | `src/cad/` | 📋 TODO |

---

## 2. Детальная спецификация открытых задач

### Фаза 5: Высокоточный физический движок деформаций, контактных нагрузок и разрушения (Accuracy-First FEM Engine)

> **Инженерный принцип Фазы 5 (приоритет точности над скоростью):**  
> Расчёт нагрузок, контакта, деформаций и разрушения опирается на единый высокоточный `f64` нелинейный упруго-пластический МКЭ-конвейер (`Co-Rotated TET10 + fe_snap + J2 Radial Return + Augmented Lagrangian Contact + Local Damage / Element Erosion / FaceSplitting + HHT-α`) с офлайн-запеканием в кэш, покадровым воспроизведением на таймлайне и строгим паритетом контрактов между Rust-сервером, документацией и TypeScript-слоем Web Studio.

---

#### P5-T21 [S]: Согласованное `T6`-интегрирование линейного поля усилий в `NeumannLoad::SurfaceTrianglesMoment` (`crates/mcp-cad-fea/src/boundary.rs:764-813`)
- **Проблема**:
  Выявлено при аудите `boundary.rs` в `P5-T12`: `NeumannLoad::SurfaceTrianglesMoment` (`crates/mcp-cad-fea/src/boundary.rs:764-813`) распределяет приложенный момент по узлам поверхности через сосредоточенную формулу `f_i = w_i * scale * (m_axis × r_i)`, где `w_i = m.area_weight` берётся из `match_surface_nodes_and_weights`. На квадратичных сетках `TET10` с плоскими гранями `T6` интеграл нулевого порядка для угловых узлов равен $w_{\text{corner}} = \int_T N_{\text{corner}}\,dA = 0$, поэтому угловые узлы вообще не получают силы от приложенного момента, хотя для линейно изменяющегося по поверхности поля усилий $\mathbf{t}(\mathbf{x}) \propto \mathbf{m}_{\text{axis}} \times (\mathbf{x} - \mathbf{c})$ точный согласованный интеграл первого порядка на углах отличен от нуля ($\int_T N_{\text{corner}}(\mathbf{x}-\mathbf{c})\,dA = \frac{A_T}{60}(2\mathbf{v}_a - \mathbf{v}_b - \mathbf{v}_c)$, ср. `compute_consistent_face_moments`). Кроме того, при `mesh.is_boundary_conforming()` не используется выделенный согласованный поверхностный интегратор.
- **Что сделать**:
  1. Заменить сосредоточенную узловую формулу в `NeumannLoad::SurfaceTrianglesMoment` на согласованное поверхностное интегрирование линейного поля $\mathbf{t}(\mathbf{x}) = \alpha\,(\mathbf{m}_{\text{axis}} \times (\mathbf{x} - \mathbf{c}))$ по граничным треугольникам `T3` (`TET4`) и `T6` (`TET10`, 6-точечная квадратура Дунаванта `T6_DUNAVANT_GP6`) с точным нормированием $\sum_i \mathbf{r}_i \times \mathbf{f}_i = \mathbf{M}$ и нулевой равнодействующей $\sum_i \mathbf{f}_i = \mathbf{0}$.
  2. Добавить unit-тест в `mcp-cad-fea`, проверяющий чистое кручение цилиндрического вала и чистый изгиб стержня через `SurfaceTrianglesMoment` на `TET4` и `TET10` (сохранение результирующей силы `0`, точный суммарный момент `M` и отсутствие локальных угловых артефактов на `TET10`).
- **Критерии приёмки**:
  - [ ] `SurfaceTrianglesMoment` на `TET4` и `TET10` обеспечивает $\sum \mathbf{f}_i = \mathbf{0}$ (`< 1e-12`), $\sum \mathbf{r}_i \times \mathbf{f}_i = \mathbf{M}$ (`< 1e-12`) и воспроизводит аналитический угол закручивания / чистого изгиба без деградации угловых узлов `TET10` и без регрессии в существующих тестах `mcp-cad-fea`.
- **Компонент**: `crates/mcp-cad-fea/src/boundary.rs`

---

#### P5-T22 [S]: Самоконтакт берегов внутренних трещин и ограничение повторного невыровненного расщепления граней в `FractureTopologyMode::FaceSplitting` (`fracture.rs`, `contact.rs`)
- **Проблема**:
  Выявлено при реализации и верификации многотельного `FaceSplitting` в `P5-T13`:
  1. После расщепления внутренних граней и дублирования вершин противоположные берега образовавшейся трещины не регистрируются как самоконтактные пары в `contact.rs` / `fracture.rs`. При изгибе или смене знака нагрузки (закрытии трещины сжатием) противоположные берега трещины могут механически проникать друг в друга без генерации сжимающего отпора.
  2. В `apply_face_splitting_step_with_map` (`crates/mcp-cad-fea/src/fracture.rs:1221-1228`) при повторной передаче уже разрушенных элементов на `prop_iter > 0` (`fracture.rs:2504-2510` и в однотельном цикле) fallback расщепляет следующую нерасщеплённую грань даже при низкой соосности с плоскостью трещины (`align < min_normal_alignment`), что может расщеплять все 4 грани разрушенных тетраэдров и порождать избыточное число мелких 1-элементных осколков вместо чистого разделения по плоскости максимального главного напряжения.
- **Что сделать**:
  1. Ограничить повторный fallback в `apply_face_splitting_step_with_map` так, чтобы на уже расщеплённом элементе не разрезались ортогональные/слабо выровненные грани ниже порога соосности (либо разрезать только для первичного разрушения, когда у элемента ещё нет ни одной расщеплённой грани).
  2. Добавить односторонний самоконтакт (penalty / AL без растяжения) для пар расщеплённых внутренних граней трещины, препятствующий взаимопроникновению берегов при изгибе и сжатии.
  3. Добавить интеграционный тест в `crates/mcp-cad-fea/tests/local_fracture_tests.rs`, проверяющий закрытие расщеплённой трещины при изгибе/сжатии без взаимопроникновения берегов и без паразитного дробления на 1-элементные фрагменты.
- **Критерии приёмки**:
  - [ ] В `FaceSplitting` расщепление локализуется вдоль плоскости трещины без каскадного разрезания невыровненных граней, а при сжатии/закрытии трещины противоположные берега не взаимопроникают сверх `max_penetration_tol` при `mass_eroded_kg == 0.0` и сохранении $\le 10^{-6}$.
- **Компонент**: `crates/mcp-cad-fea/src/fracture.rs`, `crates/mcp-cad-fea/src/contact.rs`, `crates/mcp-cad-fea/tests/local_fracture_tests.rs`

---

#### P5-T23 [S]: Подавление нефизического контактного удара при случайном перекрытии грубых сеток в $t=0$ и учёт геометрической жёсткости вращающихся RBE2-ступиц в `solve_dynamic_hht_coupled` (`dynamics.rs`)
- **Проблема**:
  Выявлено при реализации и отладке `P5-T14` на грубых беззазорных сетках `create_spur_gear`:
  1. Когда две грубые тетраэдральные сетки (например, беззазорная зубчатая пара при номинальном межосевом расстоянии $a = m(z_1+z_2)/2$ без бокового зазора) имеют артефактное геометрическое перекрытие в момент $t=0$, `compute_initial_accelerations` (`crates/mcp-cad-fea/src/dynamics.rs`) прикладывает полную штрафную контактную силу $\epsilon_n \cdot \text{pen}$ на нулевом шаге, разгоняя тела до нефизических скоростей и многогигапаскальных напряжений.
  2. При конечных поворотах `Revolute`-пилота в условиях экстремальных внутренних напряжений ($\sigma \sim \mathcal{O}(E)$) и отключённой по умолчанию `include_geometric_stiffness = false` касательная матрица Ньютона не включает геометрическую жёсткость $\mathbf{K}_{\text{geo}}(\boldsymbol{\sigma})$ и производную $\partial\mathbf{t}_i/\partial\theta = \mathbf{e}\times\mathbf{t}_i$ для узлов RBE2-ступицы, что может вызывать стагнацию backtracking line search.
- **Что сделать**:
  1. Добавить опциональное смещение/сглаживание начального зазора (initial penetration offset / ramp-in) или диагностическое предупреждение при обнаружении начального перекрытия контакта в $t = 0$ внутри `solve_dynamic_hht_coupled`.
  2. Добавить симметризованный вклад геометрической жёсткости RBE2-пилота (и автоматическое включение `include_geometric_stiffness` при сильном росте напряжений/стагнации line search) в `crates/mcp-cad-fea/src/dynamics.rs`.
  3. Добавить интеграционный тест в `crates/mcp-cad-fea/tests/dynamic_hht_tests.rs`, подтверждающий устойчивую сходимость `HHT-α` на беззазорной грубой зубчатой паре и при высоких внутренних напряжениях на вращающемся `Revolute`-пилоте.
- **Критерии приёмки**:
  - [ ] Грубая беззазорная пара `create_spur_gear` при номинальном межосевом расстоянии $a = m(z_1+z_2)/2$ устойчиво сходится в `solve_dynamic_hht_coupled` без нефизического взрыва скоростей в $t=0$ и без стагнации Ньютона.
- **Компонент**: `crates/mcp-cad-fea/src/dynamics.rs`, `crates/mcp-cad-fea/tests/dynamic_hht_tests.rs`

---

#### P5-T24 [S]: Синхронизация единиц Фазы 5 в верхнеуровневом `description` инструмента `define_material` и добавление алиаса `hardening_modulus_gpa` (`crates/mcp-cad-server/src/tools/{mod.rs, cad.rs}`)
- **Проблема**:
  Выявлено при аудите `define_material` в `P5-T17`:
  1. Верхнеуровневая строка `description` инструмента `define_material` (`crates/mcp-cad-server/src/tools/mod.rs:974`) в блоке `Units:` перечисляет только 4 базовых параметра (`young_modulus_gpa`, `poisson_ratio`, `yield_strength_mpa`, `density_kg_m3`) и не документирует конститутивные параметры Фазы 5 (`ultimate_strength_mpa`/`ultimate_strength_pa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`, `failure_mode`).
  2. Для предела прочности в `define_material` поддерживаются оба масштаба (`ultimate_strength_pa` в Па и `ultimate_strength_mpa` в МПа), тогда как модуль упрочнения принимается только в Паскалях (`hardening_modulus_pa`), хотя `young_modulus_gpa` задаётся в ГПа.
- **Что сделать**:
  1. Дополнить блок `Units:` в `define_material.description` (`crates/mcp-cad-server/src/tools/mod.rs:974`) единицами конститутивных параметров Фазы 5 и добавить опциональное поле `hardening_modulus_gpa` (в ГПа, с конвертацией `* 1.0e9` в `crates/mcp-cad-server/src/tools/cad.rs:3378` и возвратом `hardening_modulus_gpa` в `properties`).
  2. Добавить unit-тест, проверяющий регистрацию материала с `hardening_modulus_gpa` и `ultimate_strength_mpa`.
- **Критерии приёмки**:
  - [ ] `define_material` принимает `hardening_modulus_gpa` наряду с `hardening_modulus_pa`, документирует все параметры Фазы 5 в `description` и проходит `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`.
- **Компонент**: `crates/mcp-cad-server/src/tools/mod.rs`, `crates/mcp-cad-server/src/tools/cad.rs`

---

#### P5-T25 [S]: Документирование параметров `youngs_modulus_gpa` и `poissons_ratio` в JSON-схеме `check_gear_strength` (`crates/mcp-cad-server/src/tools/mod.rs:1414-1487`)
- **Проблема**:
  Выявлено при аудите `check_gear_strength` в `P5-T18`: обработчик `cad::check_gear_strength` (`crates/mcp-cad-server/src/tools/cad.rs`) поддерживает ручные переопределения модуля Юнга (`youngs_modulus_gpa` / `young_modulus_gpa`) и коэффициента Пуассона (`poissons_ratio` / `poisson_ratio`) и требует их вместе с `yield_strength_mpa`/`allowable_bending_stress_mpa` при передаче незарегистрированного пользовательского идентификатора `material`, однако эти два поля не объявлены в `input_schema.properties` инструмента `check_gear_strength` (`crates/mcp-cad-server/src/tools/mod.rs:1414-1487`) и отсутствуют в таблице параметров [`docs/MCP_API_REFERENCE.md`](docs/MCP_API_REFERENCE.md).
- **Что сделать**:
  1. Добавить свойства `youngs_modulus_gpa` (с единицами `GPa`) и `poissons_ratio` (безразмерный в диапазоне `(-1.0, 0.5)`) в `input_schema` инструмента `check_gear_strength` (`crates/mcp-cad-server/src/tools/mod.rs`).
  2. Синхронизировать таблицу входных параметров `check_gear_strength` в `docs/MCP_API_REFERENCE.md` и убедиться, что `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` проходит.
- **Критерии приёмки**:
  - [ ] `tools/list` для `check_gear_strength` и `docs/MCP_API_REFERENCE.md` документируют `youngs_modulus_gpa` и `poissons_ratio` с явными единицами измерения, а контрактный тест схемы проходит.
- **Компонент**: `crates/mcp-cad-server/src/tools/mod.rs`, `docs/MCP_API_REFERENCE.md`

---

#### P5-T26 [S]: Валидация пустых/пробельных `name` и `part_id` в `create_part` (`crates/mcp-cad-server/src/tools/cad.rs:317-345`) и документирование `part_id` в `docs/MCP_API_REFERENCE.md`
- **Проблема**:
  Выявлено при аудите `create_part` в `P5-T19`:
  1. В `crates/mcp-cad-server/src/tools/mod.rs:26` в разделе `Common errors:` указано `INVALID_ARGUMENT (empty name)`, однако обработчик `cad::create_part` (`crates/mcp-cad-server/src/tools/cad.rs:317-345`) проверяет только наличие строкового поля `name` и не отклоняет пустую или состоящую только из пробелов строку (`name.trim().is_empty()`), а также пустой/пробельный пользовательский `part_id`.
  2. В таблице входных параметров `create_part` в [`docs/MCP_API_REFERENCE.md`](docs/MCP_API_REFERENCE.md) (строки `263-268`) отсутствует необязательный параметр `part_id` (хотя он поддерживается схемой и обработчиком).
- **Что сделать**:
  1. Добавить проверку `name.trim().is_empty()` и `part_id.trim().is_empty()` (если `part_id` передан) с возвратом `ServerError::InvalidArgument` в `crates/mcp-cad-server/src/tools/cad.rs`.
  2. Добавить `part_id` в таблицу входных параметров `create_part` в `docs/MCP_API_REFERENCE.md` и покрыть тестом в `mcp_contract_tests.rs`.
- **Критерии приёмки**:
  - [ ] Передача `name: ""` / `"   "` или `part_id: ""` в `create_part` возвращает `ErrorEnvelope` с `code == "INVALID_ARGUMENT"`, а `docs/MCP_API_REFERENCE.md` документирует `part_id`.
- **Компонент**: `crates/mcp-cad-server/src/tools/cad.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`

---

#### P5-T36 [S]: Масштабирование `anchor_a`/`anchor_b` по `length_unit` и сохранение `anchorB` в TS `create_joint` (`src/cad/mcp_dispatcher.ts:1453-1466`)
- **Проблема**:
  Выявлено при сверке `create_joint` с Rust (`crates/mcp-cad-server/src/tools/physics.rs:202-204, 236-264`) в `P5-T30`: в `src/cad/mcp_dispatcher.ts:1453-1466` координаты `anchor_a`/`anchor_b` не масштабируются через `kToM` при `length_unit = 'mm'` (по умолчанию), а отдельное поле `anchorB` вообще не записывается в `KinematicJoint` (берётся лишь `params.anchor ?? params.anchor_a ?? params.anchor_b`), хотя `analyzeMechanism` и `sweepJoint` в `src/cad/kinematics.ts` используют `joint.anchor` и `joint.anchorB` в метрах.
- **Что сделать**:
  1. В `case 'create_joint'` (`src/cad/mcp_dispatcher.ts`) масштабировать `anchor_a` и `anchor_b` (и `anchor`) с коэффициентом `kToM` и сохранять `anchorB` в `KinematicJoint`.
  2. Согласовать существующие тесты (включая `BUG-022` в `src/cad/cad_engine.test.ts:1249-1280`) и добавить тест на `anchor_a`/`anchor_b` в `mm` и `m`.
- **Критерии приёмки**:
  - [ ] TS `create_joint` масштабирует `anchor_a`/`anchor_b` по `length_unit` и заполняет `anchorB` в полном паритете с Rust `physics.rs`.
- **Компонент**: `src/cad/mcp_dispatcher.ts`, `src/cad/cad_engine.test.ts`

---

#### P5-T37 [S]: Отклонение явных пустых `id: ""` / `material_id: ""` в TS `define_material` (`src/cad/mcp_dispatcher.ts:1301`)
- **Проблема**:
  Выявлено при аудите `case 'define_material'` в `P5-T31`: строка `const matId = String(params.material_id || params.id || \`mat_${Date.now()}\`);` (`src/cad/mcp_dispatcher.ts:1301`) использует логическое `||`, из-за чего явная передача пустой строки `id: ""` или `material_id: ""` молча подменяется сгенерированным идентификатором `mat_<timestamp>` вместо выброса ошибки валидации (в отличие от Rust `crates/mcp-cad-server/src/tools/cad.rs:3518-3520`, возвращающего `INVALID_ARGUMENT: Material 'id' cannot be empty`).
- **Что сделать**:
  1. В `case 'define_material'` (`src/cad/mcp_dispatcher.ts:1301`) различать отсутствие ключа (`undefined`) и явную передачу пустой/пробельной строки в `material_id` / `id`, отклоняя пустые значения ошибкой валидации.
  2. Добавить unit-тест в `src/cad/cad_engine.test.ts` на отклонение `id: ""`, `id: "   "` и `material_id: ""`.
- **Критерии приёмки**:
  - [ ] TS `define_material` отклоняет явные пустые/пробельные `id` и `material_id` в паритете с Rust `cad.rs`.
- **Компонент**: `src/cad/mcp_dispatcher.ts`, `src/cad/cad_engine.test.ts`

---

#### P5-T38 [S]: Проброс нелинейных параметров Фазы 5 в `LiveMaterialInfo` (`live_sync.rs:76-83`, `live_sync_adapter.ts:46-53, 202-211`)
- **Проблема**:
  Выявлено при аудите потребителей `normalizeMaterialSpec` в `P5-T31`: структура `LiveMaterialInfo` в Rust (`crates/mcp-cad-server/src/live_sync.rs:76-83`) и TypeScript (`src/cad/live_sync_adapter.ts:46-53, 202-211`) содержит только 5 линейно-упругих полей (`name`, `youngs_modulus_gpa`, `poissons_ratio`, `yield_strength_mpa`, `density_kg_m3`) и `failure_mode`, но не включает нелинейные параметры Фазы 5 (`ultimate_strength_pa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`). При Live Sync пользовательского материала, зарегистрированного в Rust через `define_material`, эти параметры теряются и заменяются расчётными дефолтами в Web Studio.
- **Что сделать**:
  1. Добавить опциональные поля `ultimate_strength_pa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2` и `friction_coeff` в `LiveMaterialInfo` (`crates/mcp-cad-server/src/live_sync.rs` и `src/cad/live_sync_adapter.ts`) с обратной совместимостью.
  2. Прокинуть эти поля в `mapRustMaterialToWebStudio` -> `normalizeMaterialSpec` и покрыть unit-тестом в `src/cad/__tests__/live_sync_adapter.test.ts`.
- **Критерии приёмки**:
  - [ ] Live Sync сохраняет все нелинейные конститутивные параметры Фазы 5 пользовательских материалов между Rust-сервером и Web Studio.
- **Компонент**: `crates/mcp-cad-server/src/live_sync.rs`, `src/cad/live_sync_adapter.ts`, `src/cad/__tests__/live_sync_adapter.test.ts`

---

#### P5-T39 [S]: Актуализация таблицы входных параметров `### 42. bake_coupled_simulation` в `docs/MCP_API_REFERENCE.md:2291-2304`
- **Проблема**:
  Выявлено при аудите раздела `### 42. bake_coupled_simulation` в `P5-T32`:
  1. В таблице входных параметров (`docs/MCP_API_REFERENCE.md:2291-2304`) отсутствуют **10 опциональных параметров**, объявленных в Rust `input_schema` (`crates/mcp-cad-server/src/tools/mod.rs:1823-1934`): `wait_timeout_ms`, `output_fps`, `max_frames`, `mesh_density`, `enable_plasticity`, `enable_fracture`, `enable_contact`, `corotated`, `contact_friction`, `length_unit`.
  2. Описание `async_mode` утверждает `false (по умолчанию) — синхронное ожидание завершения`, тогда как в `mod.rs:1810, 1928` при опущенном `async_mode` сервер ожидает не более `wait_timeout_ms` (`12000 ms` по умолчанию) и при превышении возвращает `status='running'` с `job_id`, `progress_pct`, ETA и `frames_cached`.
- **Что сделать**:
  1. Добавить все 10 отсутствующих параметров с единицами измерения и значениями по умолчанию в таблицу `### 42. bake_coupled_simulation` в `docs/MCP_API_REFERENCE.md` и актуализировать описание `async_mode` / `wait_timeout_ms`.
  2. Добавить автоматическую контрактную проверку синхронизации всех параметров `bake_coupled_simulation` и `query_simulation_frame` между `mod.rs` и `docs/MCP_API_REFERENCE.md` в `src/cad/cad_engine.test.ts`.
- **Критерии приёмки**:
  - [ ] Таблица параметров `### 42. bake_coupled_simulation` в `docs/MCP_API_REFERENCE.md` покрывает все 28 свойств `input_schema` из `mod.rs` и точно отражает поведение `async_mode` / `wait_timeout_ms`.
- **Компонент**: `docs/MCP_API_REFERENCE.md`, `src/cad/cad_engine.test.ts`

---

#### P5-T40 [S]: Исправление двух битых групповых якорей оглавления `docs/MCP_API_REFERENCE.md:16, 31` и добавление подпункта `Контракт единиц измерения`
- **Проблема**:
  Выявлено при проверке всех GFM-якорей оглавления `docs/MCP_API_REFERENCE.md` в `P5-T32`:
  1. Строка `16` оглавления ссылается на `#группа-2-параметрическое-dag-моделирование-и-тесселяция`, однако фактический заголовок на строке `401` — `## Группа 2: Параметрическое DAG-моделирование` (GFM-якорь `#группа-2-параметрическое-dag-моделирование`).
  2. Строка `31` оглавления ссылается на `#группа-5-визуализация-экспорт-и-импорт`, однако фактический заголовок на строке `1004` — `## Группа 5: Визуализация, превью, экспорт и импорт` (GFM-якорь `#группа-5-визуализация-превью-экспорт-и-импорт`).
  3. В подразделе `1. Общие положения` оглавления отсутствует ссылка на раздел `### Контракт единиц измерения (\`[DECISION-UNITS-1]\`)` (строка `122`).
- **Что сделать**:
  1. Исправить якоря и подписи пунктов `3` (`Группа 2`) и `6` (`Группа 5`) в оглавлении `docs/MCP_API_REFERENCE.md` и добавить подпункт для `Контракт единиц измерения`.
  2. Расширить тест оглавления в `src/cad/cad_engine.test.ts`, проверяя разрешение абсолютно всех ссылок (включая групповые заголовки `##` и подразделы `###`) внутри `## Оглавление`.
- **Критерии приёмки**:
  - [ ] 100% всех якорных ссылок (и групповых `##`, и инструментальных `###`) в `## Оглавление` `docs/MCP_API_REFERENCE.md` разрешаются в существующие заголовки документа.
- **Компонент**: `docs/MCP_API_REFERENCE.md`, `src/cad/cad_engine.test.ts`

---

#### P5-T41 [S]: Пересчёт `mass` и `inertiaTensor` по новой `density` (и прогон последующих узлов DAG) в `evaluateFeatureTree` (`src/cad/primitives.ts:3474-3476`) для деталей с `stl_import`
- **Проблема**:
  Выявлено при аудите пересчёта материалов для импортированных STL-деталей в `P5-T33`: в `evaluateFeatureTree` (`src/cad/primitives.ts:3474-3476`) ветка `if (enabledNodes[0].type === 'stl_import' && existingMesh) { return existingMesh; }` немедленно возвращает прежний `existingMesh` без пересчёта `mass` и `inertiaTensor` под переданную `density`. В результате назначение нового материала STL-детали через `onAssignMaterial` (`src/App.tsx:4007-4025`) меняет `part.materialId`, но оставляет прежнюю массу и тензор инерции; кроме того, ранний `return` пропускает последующие узлы DAG (`cut_hole`, `transform` и др.), если они добавлены поверх `stl_import`.
- **Что сделать**:
  1. В `evaluateFeatureTree` (`src/cad/primitives.ts:3474-3476`) при единственном активном узле `stl_import` пересчитывать `mass = existingMesh.volume * density` и пропорционально масштабировать `inertiaTensor` (если плотность изменилась), а при наличии последующих узлов (`enabledNodes.length > 1`) прогонять цикл DAG с инициализацией `currentMesh` из `existingMesh` / `params.mesh` и пересчитанной плотностью.
  2. Добавить unit-тест в `src/cad/cad_engine.test.ts`, проверяющий обновление массы/инерции STL-детали при смене материала и сохранение работы последующих узлов DAG поверх `stl_import`.
- **Критерии приёмки**:
  - [ ] Смена материала у импортированной STL-детали пересчитывает `mesh.mass` и `mesh.inertiaTensor` строго пропорционально новой плотности, а последующие операции `transform`/`cut_hole` в DAG не пропускаются.
- **Компонент**: `src/cad/primitives.ts`, `src/cad/cad_engine.test.ts`

---

#### P5-T42 [S]: Скрытие внутреннего буфера `params.mesh` в инспекторе узла DAG (`src/components/CadLeftSidebar.tsx:305`) и актуализация счётчика `(41 Rust)` → `(43 Rust)` в `src/components/CadRightSidebar.tsx:1269`
- **Проблема**:
  Выявлено при визуальной браузерной проверке `P5-T33`:
  1. В `src/components/CadLeftSidebar.tsx:305` инспектор параметров узла дерева DAG рендерит все ключи `Object.entries(node.params)` через `JSON.stringify(val)`. Поскольку узел `stl_import` хранит во внутреннем поле `params.mesh` объект `MeshGeometryData` с типизированными массивами (`Float32Array`), в UI выводится мусорная строка `mesh: {"vertices":{"0"...`, загромождающая панель и провоцирующая дорогую сериализацию больших сеток при каждом рендере.
  2. В `src/components/CadRightSidebar.tsx:1269` отображается устаревший счётчик `(41 Rust) MCP инструментов` вместо актуального `RUST_MCP_TOOLS_COUNT` (`43 Rust`).
- **Что сделать**:
  1. В `src/components/CadLeftSidebar.tsx:305` исключить внутренний бинарный ключ `mesh` (и/или отображать компактную сводку `vertices/triangles` вместо `JSON.stringify(MeshGeometryData)`) при отрисовке параметров узла DAG.
  2. В `src/components/CadRightSidebar.tsx:1269` использовать импортированную константу `RUST_MCP_TOOLS_COUNT` (`43`) вместо зашитого `41`.
- **Критерии приёмки**:
  - [ ] В инспекторе узла `stl_import` не рендерится сырой JSON типизированных массивов `params.mesh`, а `CadRightSidebar` отображает актуальное число `43 Rust` MCP-инструментов.
- **Компонент**: `src/components/CadLeftSidebar.tsx`, `src/components/CadRightSidebar.tsx`

---

#### P5-T43 [S]: Добавление `limits_unit` в JSON-схему `create_joint`, исправление описания `ratio` (`mod.rs:389-473`) и строгая валидация `limits_unit` в `physics.rs:232-245`
- **Проблема**:
  Выявлено при аудите `create_joint` в `P5-T34`:
  1. Обработчик `crates/mcp-cad-server/src/tools/physics.rs:232` принимает опциональный параметр `limits_unit` (`"deg"` или `"rad"`), однако в `input_schema.properties` и `description` инструмента `create_joint` (`crates/mcp-cad-server/src/tools/mod.rs:389-473`) поле `limits_unit` отсутствует.
  2. В `crates/mcp-cad-server/src/tools/mod.rs:452` описание `ratio` указывает инвертированную формулу `(omega_a / omega_b, dimensionless)`, тогда как физический решатель (`crates/mcp-cad-physics/src/joint.rs:79`, `solver.rs:1158`) накладывает $\omega_B \cdot \hat{\mathbf{z}}_B = -\text{ratio}\,(\omega_A \cdot \hat{\mathbf{z}}_A)$, а `physics.rs:315` вычисляет `ratio = teeth_a / teeth_b` (т.е. $|\omega_B / \omega_A| = |\text{ratio}|$).
  3. В `crates/mcp-cad-server/src/tools/physics.rs:232, 240` неизвестные/ошибочные значения `limits_unit` (например, `"degrees"` или `"foo"`) не отклоняются, а молча трактуются как `"deg"`.
- **Что сделать**:
  1. Добавить `limits_unit` (`enum: ["deg", "rad"]`) в `input_schema.properties` и блок `Units:` в `description` инструмента `create_joint` (`crates/mcp-cad-server/src/tools/mod.rs`), а также исправить описание `ratio` на `omega_b = -ratio * omega_a` (`-omega_b / omega_a`).
  2. Добавить строгую валидацию `limits_unit` (`"deg"` / `"rad"`, регистронезависимо; при ином значении или нестроковом типе возвращать `ServerError::InvalidArgument`) в `crates/mcp-cad-server/src/tools/physics.rs:232-245`.
  3. Покрыть тестом в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`.
- **Критерии приёмки**:
  - [ ] `tools/list` для `create_joint` включает `limits_unit` и корректное описание `ratio`, а передача некорректного `limits_unit` возвращает `ErrorEnvelope` с `code == "INVALID_ARGUMENT"`.
- **Компонент**: `crates/mcp-cad-server/src/tools/mod.rs`, `crates/mcp-cad-server/src/tools/physics.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`

---

#### P5-T44 [S]: Строгая валидация `limits_unit`, поддержка `teeth_a`/`teeth_b` и `axis_a`/`axis_b` в TS `create_joint` (`src/cad/mcp_dispatcher.ts:1457-1502`)
- **Проблема**:
  Выявлено при аудите `case 'create_joint'` в `P5-T34`:
  1. В `src/cad/mcp_dispatcher.ts:1458` `limits_unit` не валидируется — любая неизвестная строка кроме `'rad'` молча трактуется как `'deg'`.
  2. При создании `joint_type: "gear"` TS-диспетчер (`src/cad/mcp_dispatcher.ts:1496-1501`) читает только `params.gear_ratio` / `params.ratio` и игнорирует `teeth_a`/`teeth_b` (в отличие от Rust `physics.rs:286-322`, вычисляющего `ratio = teeth_a / teeth_b` с проверкой `> 0`), а также не использует `axis_a`/`axis_b`.
- **Что сделать**:
  1. В `src/cad/mcp_dispatcher.ts` (`case 'create_joint'`) добавить строгую валидацию `limits_unit` (`'deg' | 'rad'`), вычисление `gearRatio = teeth_a / teeth_b` (с проверкой `> 0` и конечности, а также валидацией ненулевого `ratio`) и поддержку `axis_a`/`axis_b`.
  2. Добавить unit-тест в `src/cad/cad_engine.test.ts`.
- **Критерии приёмки**:
  - [ ] TS `create_joint` отклоняет некорректный `limits_unit` и корректно вычисляет `gearRatio = teeth_a / teeth_b` для `joint_type: "gear"` в паритете с Rust `physics.rs`.
- **Компонент**: `src/cad/mcp_dispatcher.ts`, `src/cad/cad_engine.test.ts`

---

#### P5-T45 [S]: Передача `limits_unit: 'rad'` для вращательных шарниров при синхронизации `joint.limits` в `buildRustSceneSyncBatch` (`src/cad/rust_scene_sync.ts:678-680`)
- **Проблема**:
  Выявлено при аудите всех обращений к `joint.limits` в `P5-T35`: в `src/cad/rust_scene_sync.ts:678-680` при генерации пакетной операции `create_joint` для синхронизации сцены Web Studio с Rust-бэкендом передаётся `createArgs.limits = joint.limits` вместе с `length_unit: 'm'`, но для вращательных шарниров (`joint.type === 'revolute'`) не указывается `limits_unit: 'rad'`. Поскольку `KinematicJoint.limits` для `revolute` хранится в радианах, а Rust `physics.rs:232` по умолчанию трактует `limits` в градусах (`limits_unit = "deg"`) и вызывает `.to_radians()`, угловые пределы вращательного шарнира при синхронизации сжимаются в $180/\pi \approx 57.3$ раза.
- **Что сделать**:
  1. В `src/cad/rust_scene_sync.ts:678-680` при наличии `joint.limits` и `joint.type === 'revolute'` добавлять `createArgs.limits_unit = 'rad'`.
  2. Добавить unit-тест в `src/cad/__tests__/rust_scene_sync.test.ts`.
- **Критерии приёмки**:
  - [ ] `buildRustSceneSyncBatch` передаёт `limits_unit: 'rad'` для вращательных шарниров с `limits`, исключая повторный перевод из градусов в радианы на стороне Rust.
- **Компонент**: `src/cad/rust_scene_sync.ts`, `src/cad/__tests__/rust_scene_sync.test.ts`

---

#### P5-T46 [S]: Зажим угла вращательного шарнира (`joint.type === 'revolute'`) по `joint.limits` в `stepKinematicsAssembly` (`src/cad/kinematics.ts:41-68`)
- **Проблема**:
  Выявлено при аудите `stepKinematicsAssembly` в `P5-T35`: ветка `joint.type === 'prismatic'` (`src/cad/kinematics.ts:88-97`) зажимает перемещение `dPos` и `joint.currentAngleRad` в пределах `[minL, maxL]` из `joint.limits`, однако ветка `joint.type === 'revolute'` (`src/cad/kinematics.ts:41-68`) вообще не проверяет `joint.limits` и прибавляет `dTheta = omegaRadSec * dtSeconds` без ограничений.
- **Что сделать**:
  1. В `src/cad/kinematics.ts:41-68` при наличии `joint.limits` у вращательного шарнира ограничивать `nextAngle = joint.currentAngleRad + dTheta` диапазоном `[Math.min(joint.limits[0], joint.limits[1]), Math.max(joint.limits[0], joint.limits[1])]` (и обнулять угловую скорость при достижении упора).
  2. Добавить unit-тест в `src/cad/cad_engine.test.ts`.
- **Критерии приёмки**:
  - [ ] Вращательные шарниры с заданными `limits` в `stepKinematicsAssembly` и `run_physics_simulation` корректно останавливаются на угловых упорах `[minRad, maxRad]`.
- **Компонент**: `src/cad/kinematics.ts`, `src/cad/cad_engine.test.ts`

---

## 3. Архивный реестр выполненных задач (Completed Tasks P0–P5)

Все архитектурные решения и результаты этих задач интегрированы в [`PROJECT_CONTEXT.md`](PROJECT_CONTEXT.md) (раздел 5), а подробные протоколы верификации сохранены в [`docs/agent_tasks/`](docs/agent_tasks/).

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
| **P2-T31** | P2 Contract | Разбиение вычитаний по барьерным узлах в `R3`, оценка AABB для `AddSpurGear`/`AddWheelRim` и ветка `BspUnionJoin` в `estimate_dag_cost` | §5.6 | [`P2-T31-report.md`](docs/agent_tasks/P2-T31-report.md) |
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
| **P3-T20** | P3 Contract | Актуализация контрактного теста `test_mcp_open_preview_window` (`status: "empty"` на пустой сцене/детали без фич, контролируемый headless-сценарий при `MCP_CAD_DISABLE_GUI_SPAWN=1` и cleanup временного STL) | §7, §8 | [`P3-T20-report.md`](docs/agent_tasks/P3-T20-report.md) |
| **P3-T21** | P3 Calculation | Актуализация инициализации `TriangleMesh::new(vertices, indices)` в изолированном спайке `spikes/monstertruck_brep/src/main.rs:60` после добавления полей `face_ids` и `faces` (`P2-T6`) | §5.7, §7 | [`P3-T21-report.md`](docs/agent_tasks/P3-T21-report.md) |
| **P3-T22** | P0 Hygiene | Восстановление `rust-toolchain.toml` (`1.91`), актуализация `.github/workflows/ci.yml`, восстановление и адаптация `src/cad/cad_engine.test.ts` (`37/37` + `15/15` в `analyze_mechanism.test.ts`), добавление `npm test` в `package.json` и синхронизация `README.md` / `PROJECT_CONTEXT.md` | §2, §7 | [`P3-T22-report.md`](docs/agent_tasks/P3-T22-report.md) |
| **P3-T23** | P3 Testing | Общий потокобезопасный `OffscreenRenderer::shared()` (`WGPU_INIT_LOCK` + `render_lock`), автоотключение GUI-спавна и временный STL для тестовых бинарников в `open_preview_window`, RAII-изоляция `qa_wide_scan` во временный каталог (`QaDirGuard`), снятие `-- --test-threads=1` в CI и `README.md` (`135/135 PASS` многопоточно) | §7 | [`P3-T23-report.md`](docs/agent_tasks/P3-T23-report.md) |
| **P3-T24** | P2 Docs / CI | Синхронизация счётчиков и таблиц инструментов (`40` Rust / `31` TS) в `README.md`, `docs/{CLAUDE_DESKTOP_SETUP,ANTIGRAVITY_SETUP,ARCHITECTURE,MCP_API_REFERENCE}.md`, TS/UI (`native_bridge.ts`, `workbenches.ts`, `App.tsx`, `CadRightSidebar.tsx`) и скриптах запуска, обновление `BUG-021`, синхронизация и фиксация `package-lock.json` для `npm ci` (`cache: "npm"` в CI) | §7, §8 | [`P3-T24-report.md`](docs/agent_tasks/P3-T24-report.md) |
| **P4-T0** | P4 Blueprints | 41-й MCP-инструмент `apply_recipe` (`flanged_bushing`, `stepped_shaft`, `gear_transmission`) с транзакционным откатом, `validate_mesh`, `length_unit`, `auto_check` и полной синхронизацией счётчиков/документации (`41` Rust / `31` TS) | §7, §8 | [`P4-T0-report.md`](docs/agent_tasks/P4-T0-report.md) |
| **P4-T1** | P4 Physics DAE | Архитектурное решение `ADR-004` (`docs/DECISIONS.md`) и прототип собственного многотельного решателя DAE/LCP в `f64` (`dae_prototype.rs`: `DirectKktIndex2Projection` vs `PgsBaumgarte` vs `Rapier3D` на 4 эталонах против аналитики) | §4.4, §5.7 | [`P4-T1-report.md`](docs/agent_tasks/P4-T1-report.md) |
| **P4-T2** | P4 Physics DAE | Собственная модель твёрдого тела `RigidBody` (`f64`), точный тензор инерции `MassProperties` из замкнутой сетки `Part` с учётом полостей и геометрические коллайдеры `ColliderShape` (`BodyRegistry`, `ColliderRegistry`, `0` импортов `rapier3d` в `rigid_body.rs`) | §4.3, §5.5, §7 | [`P4-T2-report.md`](docs/agent_tasks/P4-T2-report.md) |
| **P4-T3** | P4 Physics DAE | Собственный многотельный решатель ограничений и динамики в `f64` (`solver.rs`: Ньютон–Эйлер + KKT/Box-LCP, моторы, упоры `limits`, замкнутые контуры $\Phi(q)=0$ и связанное ограничение `Gear` с передачей момента; `0` импортов `rapier3d` в `solver.rs`, `joint.rs`, `simulator.rs`) | §4.3, §5.5, §7 | [`P4-T3-report.md`](docs/agent_tasks/P4-T3-report.md) |
| **P4-T4** | P4 Physics DAE | Собственный `f64`-конвейер коллизий (`collision.rs`: Sweep-and-Prune AABB, `TriMeshBvh`, аналитика + GJK/EPA + планарное клиппирование для всех 6 `ColliderType`) и изотропный конусный контактный решатель Кулона $\|\boldsymbol{\gamma}_t\|_2 \le \mu\gamma_n$ с событиями `CollisionRecord` | §4.3, §5.5, §7 | [`P4-T4-report.md`](docs/agent_tasks/P4-T4-report.md) |
| **P4-T5** | P4 Physics DAE | Прямое извлечение точных реакций шарниров $\boldsymbol{\lambda}$ из KKT-решателя `f64` (`Pinned KKT Re-solve` для `MotorConfig` и `limits`, точный перенос реакций на `part_b` для `Prismatic` и `Gear`, удаление 280-строчного декодера импульсов `Rapier3D`) | §4.3, §4.4, §7 | [`P4-T5-report.md`](docs/agent_tasks/P4-T5-report.md) |
| **P4-T6** | P4 Physics DAE | Полное удаление `rapier3d` из `Cargo.toml`/`Cargo.lock`, методы `part_body_handle`/`joint_handle` в `PhysicsWorld`, очистка `f32`-приведений в `mcp-cad-server` и синхронизация UI веб-студии (`src/`) с `f64` DAE-решателем | §3, §4.3, §7 | [`P4-T6-report.md`](docs/agent_tasks/P4-T6-report.md) |
| **P4-T7** | P4 Physics Verification | Неявный среднеточечный Lie–Poisson предиктор для $SO(3)$-вращения (`solver.rs`), эталонная верификация `f64` DAE-решателя (`344/344` Rust, `53/53` TS, `0` warnings `clippy`) и финальная синхронизация всей проектной документации (`PRD.md`, `PROJECT_CONTEXT.md`, `ARCHITECTURE.md`, `PROGRESS.md`, `MCP_API_REFERENCE.md`, `README.md`, `CHANGELOG.md`) | §2, §4.3, §7 | [`P4-T7-report.md`](docs/agent_tasks/P4-T7-report.md) |
| **P4-T8** | P4 Physics DAE | Защита `ClosedLoopSolver` (`simulator.rs`, `rotation_vector`) от `NaN` в `Rotation3::scaled_axis()` при $\operatorname{tr}(R) > 3.0$ и $\operatorname{tr}(R) < -1.0$ через `atan2(sin_t, cos_t)` с `clamp(-1.0, 1.0)` и unit-тесты `simulator::tests` | §4.3, §7 | [`P4-T8-report.md`](docs/agent_tasks/P4-T8-report.md) |
| **P4-T9** | P4 Core Hygiene | Синхронизация `Part::mesh()` (`&self.mesh`), инвалидация кэша в `Part::mesh_mut()`, проверка `*cached == self.mesh` в `Part::tessellate()` и упрощение `MassProperties::from_part` | §5.5, §7 | [`P4-T9-report.md`](docs/agent_tasks/P4-T9-report.md) |
| **P4-T10** | P4 Testing | Стабилизация `test_p3_t10_benchmark_check_interference_10_parts_45_pairs` (`physics_tests.rs`): замена wall-clock порога `< 250.0 ms` на детерминированные структурные/аналитические проверки broad-phase/CSG, `std::hint::black_box` и защитный предел `< 5000.0 ms` | §7 | [`P4-T10-report.md`](docs/agent_tasks/P4-T10-report.md) |
| **P5-T1** | P5 High-Accuracy FEM | Архитектурное решение `ADR-005` (`docs/DECISIONS.md`) и расширение модели материалов (`material.rs`, `materials.ts`, `cad.ts`, `define_material`) параметрами $J_2$-пластичности, упрочнения, разрушения, трения, валидацией инвариантов, обратной совместимостью сериализации и пресетом `borosilicate_glass` | §3, §8 | [`P5-T1-report.md`](docs/agent_tasks/P5-T1-report.md) |
| **P5-T2** | P5 High-Accuracy FEM | Гранично-согласованная сетка `TET10` (`conforming_mesh.rs`: 2:1 octree без висячих узлов, `fe_snap` с барьером $J_s \ge 0.06$, проецирование серединных узлов `TET10` с контролем $\det J > 0$ в точках Гаусса, 6-точечное интегрирование давления на гранях `T6`, локальное сгущение и `accuracy_mode = "high"` в `run_fea_analysis`/`run_fea_from_simulation`): Кирш/Петерсон `3.04%`, Ламе `7.75%`, шестерня RMS `-74.2%` | §4.3, §5.7 | [`P5-T2-report.md`](docs/agent_tasks/P5-T2-report.md) |
| **P5-T3** | P5 High-Accuracy FEM | Нелинейный упруго-пластический Co-Rotated FEM решатель (`elasticity.rs`, `element.rs`, `solver.rs`, `result.rs`, `lib.rs`): $J_2$ фон Мизеса `Radial Return Mapping` (`Linear`/`PowerLaw`/`Saturation`), согласованный касательный модуль $\mathbf{C}^{\text{ep}}$, полярное разложение $\mathbf{F}=\mathbf{R}\mathbf{U}$ на $SO(3)$ для `TET4`/`TET10`, инкрементальный Ньютон–Рафсон с line search, плавным снятием реакций `springback` и адаптивной бисекцией: поворот $90^\circ$/$180^\circ$ `0.000e0 Pa`, одноосный цикл `0.000%`, пластический шарнир $M_p/M_y = 1.5671$ (`4.47%`) и `springback` `0.25%` | §4.3, §7 | [`P5-T3-report.md`](docs/agent_tasks/P5-T3-report.md) |
| **P5-T4** | P5 High-Accuracy FEM | Физический контакт деформируемых тел `Augmented Lagrangian` / `Mortar` с трением Кулона (`collision.rs`, `solver.rs`, `boundary.rs`, `contact.rs`): пятно контакта Герца ($p_0$ `2.84%`, $a$ `1.11%`), `stick`/`slip` Кулона (`0.000%`/`0.005%`), эвольвентное зацепление шестерён с проникновением `1.677e-8 м`, `TET10` Mortar (`0.000%`) | §4.3, §7 | [`P5-T4-report.md`](docs/agent_tasks/P5-T4-report.md) |
| **P5-T5** | P5 High-Accuracy FEM | Локальное разрушение на базе МКЭ (`damage.rs`, `fracture.rs`, `contact.rs`, `rigid_body.rs`, `simulator.rs`): регуляризация Бажанта $G_f/h_e$, Райс–Трейси + Бао–Вежбицкий ($-1/3$) и Ренкин ($\sigma_1$), `ElementErosion` и `FaceSplitting`, выделение 2-многообразных осколков с сохранением массы/импульса $\le 1.8\times 10^{-15}$ и спавном в `PhysicsWorld` | §4.3, §7 | [`P5-T5-report.md`](docs/agent_tasks/P5-T5-report.md) |
| **P5-T6** | P5 High-Accuracy FEM | Неявный динамический интегратор `HHT-α` (`sparse_solver.rs`, `dynamics.rs`, `simulator.rs`, `coupled_sim.rs`, `sse.rs`): `Consistent`/`HRZ Lumped` массы для `TET4`/`TET10`, `rayon` сборка, `RCM SparseLdlt` + `IC(0)-PCG`/`SSOR-PCG`, адаптивная бисекция, 100 периодов без CFL ($\le 0.5\%$), MCP-инструменты №42 `bake_coupled_simulation` и №43 `query_simulation_frame`, HTTP/SSE кэш кадров и отмена джоб | §4.3, §7, §8 | [`P5-T6-report.md`](docs/agent_tasks/P5-T6-report.md) |
| **P5-T7** | P5 High-Accuracy FEM | Визуализация реальных деформаций (`1×`/`5×`/`20×`), вершинных карт $\sigma_{vM}$ (с маркером $\sigma_y$) / $\bar{\varepsilon}_p$ / $D$, локальных сколов, 4-серийного баланса энергий ($E_k, E_e, E_p, E_f$), журнала физических событий и индикатора решателя (`Rust FEM f64` vs `Web Local Preview`) в 3D-вьюпорте, таймлайне и `mcp-cad-viewer` | §4.3, §7 | [`P5-T7-report.md`](docs/agent_tasks/P5-T7-report.md) |
| **P5-T8** | P5 Verification | Единый верификационный пакет точности Фазы 5 (`phase5_verification_suite.rs`: 5/5 PASS; `physics_tests::test_p5_t8_*`: 1/1 PASS — цикл `0.0000%`, шарнир `4.24%`, Герц `2.84%`/`1.11%`, зуб ISO 6336-3 `1.89%`/`Δxy = 0.10 мм`, 3-режимное столкновение шестерён + `HHT-α` + `f64` DAE) и отчёт `docs/PHYSICS_FEA_ACCURACY.md` | §3, §4.3, §7 | [`P5-T8-report.md`](docs/agent_tasks/P5-T8-report.md) |
| **P5-T9** | P5 Headless / Viewer | Реальный Pure-Rust программный CPU-растеризатор (`software.rs`, `OffscreenBackend::Wgpu \| SoftwareCpu` в `offscreen.rs`, `ADR-003`) с автоматическим переключением при отсутствии GPU и Vulkan Lavapipe (`mcp-cad-viewer`: `14/14 PASS`) | §4.3, §7 | [`P5-T9-report.md`](docs/agent_tasks/P5-T9-report.md) |
| **P5-T10** | P5 Contract Hygiene | Динамическая синхронизация `UNKNOWN_MATERIAL` / `MATERIAL_NOT_FOUND` (`builtin_material_ids()` на базе `OnceLock` + `MaterialRegistry::new()`) и распознавание сообщений `not found in registry` от `create_part` / `import_stl` / `load_project` (`error::tests`: `21/21 PASS`) | §5.3, §7 | [`P5-T10-report.md`](docs/agent_tasks/P5-T10-report.md) |
| **P5-T11** | P5 Materials Parity | Разделение сталей `steel_structural_aisi1020` ($\sigma_y = 250\text{ МПа}$) и `steel_carbon_aisi1045` / `steel_1045` ($\sigma_y = 350\text{ МПа}$) на 10 канонических пресетов в Rust и TypeScript с двусторонними кросс-тестами паритета | §3, §7 | [`P5-T11-report.md`](docs/agent_tasks/P5-T11-report.md) |
| **P5-T12** | P5 High-Accuracy FEM | Согласованное поверхностное интегрирование распределённой силы `T3`/`T6` (`t6_consistent_area_weights`, `integrate_conforming_surface_force`) в `AabbFaceForce` и `SurfaceTrianglesForce` (`boundary.rs`: однородность `< 1e-6`) | §4.3, §7 | [`P5-T12-report.md`](docs/agent_tasks/P5-T12-report.md) |
| **P5-T13** | P5 High-Accuracy FEM | Адаптивная бисекция шага с откатом `SessionSnapshot` и поддержка `FractureTopologyMode::FaceSplitting` в многотельном контактном решателе (`contact.rs`, `fracture.rs`) с нулевой потерей массы | §4.3, §7 | [`P5-T13-report.md`](docs/agent_tasks/P5-T13-report.md) |
| **P5-T14** | P5 High-Accuracy FEM | Автоматическая передача шарниров и моторов (`Revolute`, `Prismatic`, `Fixed`, `Gear`) из `PhysicsWorld` в динамический интегратор `HHT-α` и `bake_coupled_simulation` (`dynamics.rs`, `solver.rs`, `coupled_sim.rs`) | §4.3, §4.4, §7 | [`P5-T14-report.md`](docs/agent_tasks/P5-T14-report.md) |
| **P5-T15** | P5 Viewer / Bridge | Передача полной внешней поверхностной сетки МКЭ (`surface_positions_m`, `surface_reference_positions_m`, `surface_indices`, `surface_node_indices`) и кэшированной геометрии отделившихся фрагментов (`GET /api/coupled-sim/fragments/:sim_id`) из `query_simulation_frame` во фронтенд-рендерер (`coupled_sim.rs`, `sse.rs`, `field_colormaps.ts`, `CadViewport3D.tsx`) | §4.3, §7, §8 | [`P5-T15-report.md`](docs/agent_tasks/P5-T15-report.md) |
| **P5-T16** | P5 Core Geometry | Гладкая трохоидальная/круговая корневая галтель по ISO 53 ($\rho_{fP} = 0.38\,m$) в `create_spur_gear_with_settings` (`primitives.rs`: верификация `TET10 + fe_snap` против ISO 6336-3 с ошибкой `4.02% <= 8.0%`) | §4.1, §4.3, §7 | [`P5-T16-report.md`](docs/agent_tasks/P5-T16-report.md) |
| **P5-T17** | P5 Contract / Tests | Актуализация `test_save_and_load_roundtrip` (`project.rs:682`, динамический подсчёт `MaterialRegistry::new().list().len() + 1 == 11`) и `unit_keywords` (`"pascal"`, `"j/m²"`, `"j/m^2"` в `mcp_contract_tests.rs`, `152/152 PASS` в `mcp-cad-server`) | §7 | [`P5-T17-report.md`](docs/agent_tasks/P5-T17-report.md) |
| **P5-T18** | P5 Contract Hygiene | Валидация неизвестных `material` и `mating_material` в `check_gear_strength` (`tools/cad.rs`) с возвратом `UNKNOWN_MATERIAL` при отсутствии полного переопределения механических свойств | §5.3, §7, §8 | [`P5-T18-report.md`](docs/agent_tasks/P5-T18-report.md) |
| **P5-T19** | P5 Contract / Docs | Актуализация перечня всех 10 встроенных материалов в схеме `create_part` (`tools/mod.rs:26, 36`) и `docs/MCP_API_REFERENCE.md` (добавлены `steel_carbon_aisi1045` и `borosilicate_glass`, исправлен алиас `steel_1020`) | §3, §8 | [`P5-T19-report.md`](docs/agent_tasks/P5-T19-report.md) |
| **P5-T20** | P5 Web UI | Подсветка активного материала в `CadLeftSidebar.tsx:501` с учётом алиасов и канонических Rust-ID через `activeMaterial?.id === mat.id` | §3, §7 | [`P5-T20-report.md`](docs/agent_tasks/P5-T20-report.md) |
| **P5-T27** | P5 Docs | Документирование всех 7 параметров Фазы 5 инструмента `define_material` (`ultimate_strength_pa`/`mpa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`, `failure_mode`), значений по умолчанию, инвариантов валидации и полного JSON-ответа с `units` в `docs/MCP_API_REFERENCE.md` + контрактный тест синхронизации с Rust | §7, §8 | [`P5-T27-report.md`](docs/agent_tasks/P5-T27-report.md) |
| **P5-T28** | P5 Web UI / TS | Разрешение алиасов материалов и канонических Rust-ID через `getMaterialById` для FEM-поверхностей в `CadViewport3D.tsx:1106` и в `run_fea_analysis` / `get_fea_heatmap_image` в `mcp_dispatcher.ts` | §3, §7 | [`P5-T28-report.md`](docs/agent_tasks/P5-T28-report.md) |
| **P5-T29** | P5 TS Dispatcher | Учёт переданного `params.material` при расчёте плотности `density = getDensity(matId)` в `import_stl` (`src/cad/mcp_dispatcher.ts:1990`) | §5.1, §7 | [`P5-T29-report.md`](docs/agent_tasks/P5-T29-report.md) |
| **P5-T30** | P5 TS Tests / Dispatcher | Согласование единиц `limits` призматического шарнира в тесте `P2-T15` с Rust-контрактом по умолчанию `'mm'`, валидация и возврат `length_unit` в `create_joint` (`src/cad/mcp_dispatcher.ts`) | §5.1, §7, §8 | [`P5-T30-report.md`](docs/agent_tasks/P5-T30-report.md) |
| **P5-T31** | P5 TS Materials / Dispatcher | Строгая валидация неизвестного `failure_mode`, поддержка канонических Rust-ключей `young_modulus_gpa`/`poisson_ratio` в `normalizeMaterialSpec` (`src/cad/materials.ts`) и отказ от truthiness-проверки `0` в TS `define_material` | §3, §5.3, §7 | [`P5-T31-report.md`](docs/agent_tasks/P5-T31-report.md) |
| **P5-T32** | P5 Docs | Добавление якорных ссылок `42. bake_coupled_simulation` и `43. query_simulation_frame` в оглавление `docs/MCP_API_REFERENCE.md` и контрактный тест всех 43 пунктов оглавления | §7, §8 | [`P5-T32-report.md`](docs/agent_tasks/P5-T32-report.md) |
| **P5-T33** | P5 Web UI / STL | Учёт фактической плотности материала через `getMaterialById` в `resolveNativeStlMaterialSpec` / `handleLoadNativeStlUrl` и неразрушающая гидратация MCP `import_stl` через `hydrateImportedStlPartInScene` (`src/cad/native_bridge.ts`, `src/App.tsx`) | §5.1, §5.5, §7 | [`P5-T33-report.md`](docs/agent_tasks/P5-T33-report.md) |
| **P5-T34** | P5 Docs | Актуализация раздела `### 11. create_joint` в `docs/MCP_API_REFERENCE.md` (единицы `anchor_a`/`anchor_b`/`limits`, параметры `length_unit`/`limits_unit`, знак `ratio = teeth_a / teeth_b`, ответ с `length_unit` + `units`) + контрактный тест | §5.1, §7, §8 | [`P5-T34-report.md`](docs/agent_tasks/P5-T34-report.md) |
| **P5-T35** | P5 TS Kinematics | Устранение эвристики `> 1.0 ? 1e-3 : 1.0` для призматических `joint.limits` в `src/cad/kinematics.ts` и `src/cad/press_simulation.ts` (корректная симуляция ходов `> 1.0 м` в метрах СИ) | §4.3, §7 | [`P5-T35-report.md`](docs/agent_tasks/P5-T35-report.md) |
| **TS-FIX-01** | TS Suite | Синхронизация числа пресетов МКЭ (5) и импортов типов в `src/cad/{cad_engine.test.ts, primitives.ts}` | §7 | — |
