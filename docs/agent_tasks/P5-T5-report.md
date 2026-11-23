# Отчёт по задаче P5-T5: Физически корректное локальное разрушение на базе МКЭ (повреждаемость $D \in [0,1]$, `Element Erosion` / расщепление по граням и отделение связных фрагментов)

- **Дата:** 20.11.2026
- **Задача:** `P5-T5 [L]` — *Физически корректное локальное разрушение на базе МКЭ (повреждаемость $D \in [0,1]$, `Element Erosion` / расщепление по граням, отделение связных фрагментов — скол зубьев без разрушения ступицы)*
- **Статус:** ✅ **ВЫПОЛНЕНО**

---

## 1. Резюме изменений

1. **Конститутивная модель локальной повреждаемости с энергетической регуляризацией Бажанта (`crates/mcp-cad-fea/src/damage.rs`, новый модуль):**
   - Реализована структура `DamageMaterialParams` с прямым конструированием из `mcp_cad_core::Material` (автоматическая калибровка порога зарождения пластического повреждения $\varepsilon_{\text{onset}}$ по критерию шейкообразования Консидера, коэффициента чувствительности к трёхосности Райса–Трейси $\alpha_{\text{RT}} = 1.5$, порога отсечки сжатия Бао–Вежбицкого $\eta_{\text{cut}} = -1/3$ с допуском `1e-9` от ошибок округления с плавающей точкой, порога эрозии `erode_threshold = 0.995` и остаточной жёсткости `min_residual_stiffness = 1e-3`).
   - Реализованы вычисление главных напряжений и собственных направлений (`principal_stresses_and_directions`), трёхосности напряжённого состояния $\eta = \sigma_m / \sigma_{vM}$ (`stress_triaxiality`), характерного размера тетраэдра $h_e = (6 V_e)^{1/3}$ (`characteristic_element_length`) и фактора деградации жёсткости `effective_stiffness_factor`.
   - В `evaluate_element_damage` реализованы оба физических механизма разрушения с регуляризацией трещинной полосы Бажанта ($u_f = 2 G_f / \sigma_{\text{onset}}$):
     - **`FailureMode::Ductile`:** накопление необратимого индекса пластического повреждения при $\bar{\varepsilon}_p \ge \varepsilon_{\text{onset}}$ и $\eta > -1/3 + 10^{-9}$ с экспоненциальной зависимостью пластичности от трёхосности Райса–Трейси (`rice_tracey_ductility_factor`) и энергетическим разупрочнением по $G_f / h_e$;
     - **`FailureMode::Brittle`:** критерий максимального главного растягивающего напряжения Ренкина ($\sigma_1 \ge \sigma_u$) с разупрочнением Оливера–Бажанта по направлению $\mathbf{n}_1$ и учётом диссипированной энергии трещины $G_f$.

2. **Поддержка эрозии и деградации жёсткости в многотельном контактном решателе (`crates/mcp-cad-fea/src/contact.rs`):**
   - Добавлены `DeformedSurfaceMesh::from_tetra_mesh_filtered` (перестроение актуальной контактной поверхности только по активным элементам после каждого события разрушения), заморозка неактивных узлов в `StepDofSystem::build`, `compute_active_nodes_from_elements` и учёт `active_elements` / `element_stiffness_scale` в `evaluate_multibody_system` (с автоматическим отключением геометрической матрицы жёсткости $K_{\text{geo}}$ на разупрочняющихся элементах при `d_scale < 0.99`).
   - Реализована пошаговая сессия `MaskedContactSession` с прямым/резервным SPD-решателем (`solve_spd_with_fallback`), ограничением узлового шага Ньютона (`0.35 * char_length`) и строгим монотонным линейным поиском.

3. **Решатель локального разрушения, топологическое расщепление граней и выделение связных фрагментов (`crates/mcp-cad-fea/src/fracture.rs`, новый модуль; экспорт в `lib.rs`):**
   - Поддержаны оба топологических режима `FractureTopologyMode`:
     - **`ElementErosion`:** локальное исключение элементов с $D \ge D_{\text{erode}}$ строго в зоне пика напряжений;
     - **`FaceSplitting`:** расщепление общих узлов вдоль межэлементной грани, наиболее близкой к плоскости трещины (перпендикулярной $\mathbf{n}_1$), с **нулевой потерей массы** (`mass_eroded = 0.000 kg`) и пересборкой топологии сетки.
   - Реализован анализ графа смежности по граням (`find_face_connected_components`) и извлечение строго замкнутой 2-многообразной поверхностной сетки каждого фрагмента (`extract_component_surface_mesh`) со сваркой совпадающих узлов нулевых внутренних надрезов `FaceSplitting` и микро-разделением защемлённых вееров тетраэдров (`is_valid_solid == true`, `boundary_edges_count == 0`).
   - Реализовано точное симплекс-интегрирование кинематики фрагментов (`compute_subset_kinematics`: масса, центр масс, тензор инерции $3\times 3$, линейный импульс $\mathbf{P}$, угловой момент $\mathbf{L}_O / \mathbf{L}_C$, поступательная и угловая скорости $\mathbf{v}_C, \boldsymbol{\omega}_C$) и строгий контроль законов сохранения до/после отделения (`verify_detachment_conservation`, `ConservationCheck`).
   - Реализованы однотельный (`solve_nonlinear_with_fracture` для `TET4`/`TET10`, `ElementErosion` и `FaceSplitting`) и многотельный контактный (`solve_deformable_contact_with_fracture`) решатели разрушения.

4. **Динамическое спавн-отделение фрагментов и обновление родительского тела в `mcp-cad-physics` (`crates/mcp-cad-physics/src/{rigid_body.rs, simulator.rs, lib.rs}`):**
   - В `rigid_body.rs` добавлены `RigidBody::from_exact_mass_properties`, `RigidBody::update_exact_mass_properties` и `Collider::from_mesh` для точной передачи масс-инерционных характеристик и геометрии осколка/родителя без искажения момента инерции.
   - В `simulator.rs` добавлены `FragmentSpawnSpec`, `SystemMomentumSummary`, `PhysicsWorld::spawn_detached_fragment`, `PhysicsWorld::update_fractured_parent_geometry` (сохраняет `BodyHandle`, все шарниры `Revolute` и моторы родительской шестерни на валу) и `PhysicsWorld::total_system_momentum`.

---

## 2. Измеренные результаты эталонных тестов (`local_fracture_tests.rs`, `physics_tests.rs`, `damage::tests`, `fracture::tests`)

Проверено командами `cargo test -p mcp-cad-fea --no-fail-fast` и `cargo test -p mcp-cad-physics --no-fail-fast`:

| № | Эталонный тест | Режим / материалы | Измеренный результат | Критерий приёмки | Статус |
|---|---|---|---|---|---|
| **1** | **Умеренная перегрузка пары стальная–латунная шестерни** (`test_p5_t5_steel_brass_gears_moderate_overload_plastic_indentation_no_fracture`, $m=4\text{ мм}, z=14, \alpha=25^\circ, b=6\text{ мм}$) | `ElementErosion`, Steel 4140 vs Brass C360 | Пик в латуни $\sigma_{vM} = \mathbf{311.2\text{ МПа}}$ ($\sigma_y = 310 < \sigma_{vM} < \sigma_u = 400$), пик $\bar{\varepsilon}_p = \mathbf{0.00136}$; после разгрузки сохраняется остаточная деформация $\bar{\varepsilon}_p = \mathbf{0.00136}$ и остаточное напряжение `78.1 МПа` (сталь упруго возвращается к `0.0 МПа`); **`max D_brass = 0.0000`**, **`eroded = 0`**, **`frags = 0`** | Остаточное пластическое замятие зубьев при $\sigma_y < \sigma_{vM} < \sigma_u$ строго **без разрушения** | ✅ PASS |
| **2** | **Критический удар пары стальная–латунная шестерни — скол зуба в корневой галтели** (`test_p5_t5_steel_brass_gears_critical_impact_root_fillet_tooth_fracture_and_intact_hub`, зацепление ISO 6336 HPSTC) | `ElementErosion`, Steel 4140 vs Brass C360 | Стальная шестерня 4140: **`D_s = 0.000`**, **`steel_eroded = 0`**; в латуни трещина зарождается в растянутой корневой галтели (`r = 24.11 мм`, `root_r = 23.00 мм`) и прорезает основание зуба (`r = 23.54..24.11 мм`): **`brass_eroded = 25`**, **`detached_frags = 1`** (`brass_gear_frag_0`, масса **`2.092 г`**), **`intact_teeth = 12/14`** (100% активны и `D == 0`), ступица/отверстие вала 100% целы (`D == 0`); все сетки водонепроницаемы и строго многообразны (`is_valid_solid == true`, `boundary_edges_count == 0`); ошибки сохранения: **масса `1.43e-16`**, **$\mathbf{P}$ `2.02e-16`**, **$\mathbf{L}$ `3.60e-16`** | Зарождение трещины в корневой галтели, отлом контактирующего зуба, целые ступица и ненагруженные зубья, точность сохранения $\le 10^{-6}$ | ✅ PASS |
| **3** | **Хрупкое разрушение стеклянной шестерни по полю $\sigma_1$** (`test_p5_t5_brittle_borosilicate_glass_sigma1_deep_rim_crack_propagation`) | `ElementErosion`, `borosilicate_glass` (`Brittle`) | **`total_eroded = 38`**, из них в ободе **`eroded_rim = 15`** (`root_r = 19.00 мм`, `deepest_crack_r = 14.49 мм`, глубина прорастания в обод **`4.51 мм`** по полю $\sigma_1$), **`detached_frags = 5`**, `G_f_dissipated = 0.0008 Дж`, отверстие ступицы 100% цело | Естественное прорастание хрупкой трещины глубже в обод по $\sigma_1$ | ✅ PASS |
| **4** | **Однотельное разрушение бруса с надрезом: сравнение `ElementErosion` и `FaceSplitting`** (`test_p5_t5_face_splitting_and_element_erosion_single_body_fracture_conservation`) | `ElementErosion` vs `FaceSplitting`, `borosilicate_glass` | `ElementErosion`: `eroded = 36`, `frags = 3`, `mass_eroded = 3.750e-3 кг`, ошибки сохранения **`(m: 1.57e-15, P: 4.70e-16, L: 2.47e-16)`**; `FaceSplitting`: **`eroded = 0`**, `split_faces = 76`, `frags = 14`, **`mass_eroded = 0.000 кг`**, ошибки сохранения **`(m: 1.61e-16, P: 1.80e-16, L: 2.57e-17)`**; все фрагменты в обоих режимах `is_valid_solid == true` | Сохранение массы и импульса $\le 10^{-6}$ (точно при `FaceSplitting`, за вычетом эродированных микроэлементов при `ElementErosion`) | ✅ PASS |
| **5** | **Свободный полёт и коллизии отломанного зуба при продолжении вращения шестерни на валу** (`test_p5_t5_detached_tooth_free_flight_collisions_and_gear_continues_rotating`, `physics_tests.rs`) | `PhysicsWorld` (`f64` DAE + `TriMeshBvh`) | Точность сохранения импульса/момента при отделении в `PhysicsWorld`: **`< 1e-12`**; отломанный зуб летит свободно и ударяется о неподвижный испытательный стенд без туннелирования (`y > -0.052 м`); родительская шестерня остаётся на шарнире `Revolute` (`drift < 1e-4 м`) и продолжает вращаться с выбитым зубом | Автоматическое выделение независимого тела, свободный полёт с коллизиями о стенд, сохранение вращения родителя на валу | ✅ PASS |
| **6** | **Unit-тесты `damage::tests` (4 теста) и `fracture::tests` (1 тест)** | `TET4`/`TET10`, `Ductile`/`Brittle` | Проверка регуляризации Бажанта, отсечки сжатия Бао–Вежбицкого ($\eta = -1/3$), критерия Ренкина $\sigma_1$, точного симплекс-интегрирования и сохранения массы/импульса в обоих режимах | Машинная точность аналитических инвариантов | ✅ PASS |

---

## 3. Верификация тестами и линтером

Выполнено после всех финальных правок:
- `cargo test -p mcp-cad-fea --no-fail-fast` (все 6 целей: `--lib`: `18/18`, `deformable_contact_tests`: `5/5`, `fea_benchmarks`: `3/3`, `fea_tests`: `14/14`, `local_fracture_tests`: `4/4`, `nonlinear_elastoplastic_tests`: `4/4`): **48/48 PASS** (`0 failed`).
- `cargo test -p mcp-cad-physics --no-fail-fast` (`--lib`: `12/12`, `physics_tests`: `33/33`): **45/45 PASS** (`0 failed`).
- `cargo test -p mcp-cad-core --no-fail-fast`: **136/136 PASS** (`0 failed`).
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).

---

## 4. Выявленная сопутствующая проблема (заведена в `TASKS.md`)

- **`P5-T13 [S]`:** *Адаптивное дробление шага (bisection) при потере сходимости Ньютона и поддержка `FaceSplitting` в многотельном контактном решателе (`solve_deformable_contact` / `solve_deformable_contact_with_fracture`)* — однотельный решатель (`solve_nonlinear_elastoplastic` / `solve_nonlinear_with_fracture`) уже поддерживает адаптивную бисекцию шага и режим `FaceSplitting`, тогда как в многотельном контактном решателе `MaskedContactSession::solve_step` при резких сменах активного контактного набора на крупных шагах поворота ступицы отсутствует автоматическая бисекция, а `solve_deformable_contact_with_fracture` работает только в режиме `ElementErosion` (поскольку принимает `&[ContactBody]` неизменяемо).

---

## 5. Затронутые файлы

- `crates/mcp-cad-fea/src/damage.rs` *(новый)*
- `crates/mcp-cad-fea/src/fracture.rs` *(новый)*
- `crates/mcp-cad-fea/src/contact.rs`
- `crates/mcp-cad-fea/src/lib.rs`
- `crates/mcp-cad-fea/tests/local_fracture_tests.rs` *(новый)*
- `crates/mcp-cad-physics/src/rigid_body.rs`
- `crates/mcp-cad-physics/src/simulator.rs`
- `crates/mcp-cad-physics/src/lib.rs`
- `crates/mcp-cad-physics/tests/physics_tests.rs`
- `docs/agent_tasks/P5-T5-report.md` *(новый)*
- `TASKS.md`, `docs/PROGRESS.md`, `docs/FEA_ACCURACY.md`
