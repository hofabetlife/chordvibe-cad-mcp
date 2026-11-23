# Отчёт по задаче `P5-T13`: Адаптивное дробление шага (bisection) при потере сходимости Ньютона и поддержка `FaceSplitting` в многотельном контактном решателе (`contact.rs`, `fracture.rs`)

- **Дата выполнения**: 22.11.2026
- **Статус**: ✅ DONE

---

## 1. Реализованные изменения

### 1.1. Адаптивная бисекция шага с откатом состояния в многотельном контактном решателе (`crates/mcp-cad-fea/src/contact.rs`)
- Расширена конфигурация `ContactSolverOptions` полями с обратной совместимостью `#[serde(default = ...)]`:
  - `max_bisections: usize` (по умолчанию `8`; `0` отключает бисекцию);
  - `min_substep_fraction: f64` (по умолчанию `1e-3`, минимальная доля подшага $\Delta\lambda_{\min}$).
- Расширена телеметрия `ContactStepReport` полями `bisections: usize` (`#[serde(default)]`) и `substeps_completed: usize` (`#[serde(default = "default_one_usize")]`).
- В `StepDofSystem::build` и при расчёте реакций ступицы (`hub_reactions`) добавлена фильтрация неактивных/отсоединившихся узлов, чтобы при динамической перестройке топологии (`ElementErosion` и `FaceSplitting`) не создавались висячие DOF ступицы.
- Объединён механизм шагания `solve_deformable_contact` и `MaskedContactSession::solve_step` через внутренние `solve_step_internal` и `solve_single_substep`:
  - Реализован полный снимок и откат состояния сессии (`SessionSnapshot`, `take_snapshot`, `restore_snapshot`): `GaussPointState`, векторы перемещений `u`, активные множители Лагранжа `λ_n`/`λ_t` и штрафные коэффициенты контактных трафаретов, пилот-координаты `HubCoupling`, предписанные углы вращения, деформированные поверхности `DeformedSurfaceMesh` и кэш предыдущих граничных условий.
  - Добавлены интерполяторы граничных условий и приводов ступиц на промежуточной доле шага $\lambda \in (0, 1]$ (`interpolate_resolved_boundaries`, `interpolate_resolved_hubs`, `has_step_increment`), корректно интерполирующие предписанные углы поворота `Revolute`, смещения `Prismatic`, нагрузки Неймана и перемещения Дирихле.
  - Разделена стратегия отката:
    - при ненулевом инкременте нагрузки/поворота (`has_increment`) выполняется адаптивное дробление шага $\Delta\lambda \leftarrow \Delta\lambda / 2$ с откатом к последнему сошедшемуся состоянию и восстановлением шага при устойчивой сходимости; в поэтапном режиме разрушения (`cap_nodal_step`) предотвращается лишняя бисекция при уже достигнутом допуске проникновения на размягчённых элементах Качанова (`has_softened_elements`) и при отсутствии убывания невязки (`fracture_bisection_stalled`);
    - при переравновесии на фиксированной нагрузке (`!has_increment`, после эрозии или расщепления граней) сохраняется «тёплый» контактный старт, а откат с демпфированием/обнулением множителей (`[1.0, 0.25, 0.0]`) выполняется только при жёстком сбое (`SubstepOutcome::Failed` — отказ линейного решателя или `NaN`/`Inf`).
  - Добавлены мутабельный доступ `body_displacements_and_positions_mut` и флаг `rebuild_body_elements: &[bool]` в `rebuild_topology_from_masks` для пересборки `BodyElements` и контактных поверхностей после дублирования узлов в `FaceSplitting`.

### 1.2. Поддержка `FractureTopologyMode::FaceSplitting` в многотельном контакте (`crates/mcp-cad-fea/src/fracture.rs`)
- В `FractureContactStepReport` добавлено поле `#[serde(default)] pub body_split_face_count: Vec<usize>`, а в `FractureContactReport` — `#[serde(default)] pub body_total_split_faces: Vec<usize>`.
- Выделена функция `apply_face_splitting_step_with_map(...) -> (usize, Vec<(usize, usize)>)`, возвращающая как число расщеплённых граней, так и отображение `(old_node_id, new_node_id)` дублированных вершин.
- Добавлена функция `propagate_duplicated_nodes_to_contact_specs(...)`, автоматически распространяющая дублированные индексы узлов в `ContactPairSpec` (`slave_a_nodes`, `slave_b_nodes`, `slave_nodes`), `HubNodeSelector::Nodes` и `DirichletConstraint::FixedNodes` (для базового тела и всех шагов `ContactStepSpec`).
- Расширена `solve_deformable_contact_with_fracture` на работу с `FractureTopologyMode::FaceSplitting`:
  - элементы не удаляются (`body_eroded_count == 0`, `mass_eroded_kg == 0.0`), используется остаточный фактор жёсткости `effective_stiffness_factor`, накапливается множество расщеплённых пар `body_split_faces`, при дублировании узлов перестраиваются `BodyElements`, `DeformedSurfaceMesh` и контактные трафареты (`rebuild_topology_from_masks(..., &rebuild_body_elements)`).
- В `extract_component_surface_mesh` добавлена сварка дублированных `FaceSplitting`-узлов, совпадающих в исходной конфигурации (`mesh.nodes[nid].position` с точностью `1e-10 * ref_diag`) **внутри одной связной компоненты** (с усреднением их деформированных координат) в дополнение к сварке по деформированным координатам (`1e-7 * diag`). Это устраняет внутренние встречные щели (self-intersecting internal slits) в родительском теле при частичных надрезах, сохраняя точные внешние границы раздела между отделившимися фрагментами (`is_valid_solid == true`, `boundary_edges_count == 0`).

### 1.3. Новые интеграционные тесты (`crates/mcp-cad-fea/tests/local_fracture_tests.rs`)
1. `test_p5_t13_multibody_contact_adaptive_bisection_large_rotation_step` (Test 5):
   - Пара деформируемых эвольвентных шестерён (сталь 4140 + латунь C360) с одиночным крупным шагом предписанного поворота, который намеренно не сходится без бисекции;
   - Подтверждает автоматическую сходимость через адаптивную бисекцию без ручного разбиения шага вызывающим кодом: **`bisections = 3`**, **`substeps = 4`**, `Fn = 8182.3 N`, `max_pen = 4.27e-9 m <= max_penetration_tol`, `brass_vm = 329.1 MPa`.
2. `test_p5_t13_multibody_contact_face_splitting_zero_mass_loss` (Test 6):
   - Симметричные стальные захваты (`steel_4140`, привод `HubCoupling::Prismatic`) тянут закреплённый хрупкий Т-образный образец (`borosilicate_glass`) через `ContactPairSpec::DeformablePair` с явными `slave_a_nodes`/`slave_b_nodes` в режиме чистого растяжения Моды I с `FractureTopologyMode::FaceSplitting`;
   - Подтверждает **`eroded = 0`**, **`split_faces = 49`**, отделение **`16`** фрагментов, **`mass_eroded_kg == 0.0`**, ошибки сохранения массы/линейного/углового импульса **`(mass_err = 1.45e-15, p_err = 8.98e-18, l_err = 2.30e-16) <= 1e-6`**, а также строгую водонепроницаемость (`boundary_edges_count == 0`) и валидность (`is_valid_solid == true`) всех отделившихся фрагментов и уцелевшего родительского тела.

---

## 2. Результаты верификации

1. Полный прогон всех тестов `mcp-cad-fea` (**66/66 PASS**, `0 failed`):
   - `cargo test -p mcp-cad-fea --lib --no-fail-fast`: **25/25 PASS** (`0.30s`);
   - `deformable_contact_tests`: **5/5 PASS** (`15.28s`);
   - `dynamic_hht_tests`: **4/4 PASS** (`2.88s`);
   - `local_fracture_tests`: **6/6 PASS** (`32.88s`, включая оба новых теста `P5-T13` и все 4 эталона `P5-T5`);
   - `fea_tests`: **14/14 PASS** (`37.69s`);
   - `nonlinear_elastoplastic_tests`: **4/4 PASS** (`37.69s`);
   - `phase5_verification_suite`: **5/5 PASS** (`70.60s`);
   - `fea_benchmarks`: **3/3 PASS** (`82.92s`).
2. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах, `26.87s`).
3. `npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`): **PASS**.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)

- **`P5-T22 [S]`**: При разработке и тестировании `FractureTopologyMode::FaceSplitting` в многотельном контакте (`crates/mcp-cad-fea/src/fracture.rs`) выявлены два физических/топологических ограничения, требующих отдельной доработки:
  1. **Отсутствие самоконтакта берегов внутренних трещин**: после расщепления внутренних граней противоположные берега трещины не регистрируются как контактная пара (`self-contact`), поэтому при изгибе или переходе зоны трещины в сжатие противоположные берега могут механически проникать друг в друга.
  2. **Повторное fallback-расщепление невыровненных граней на `prop_iter > 0`**: в `apply_face_splitting_step_with_map` (`fracture.rs:1221-1228`) при повторной передаче уже разрушенных элементов (`fracture.rs:2504-2510` и аналогичном однотельном цикле) fallback расщепляет произвольную оставшуюся грань даже при низкой соосности с плоскостью трещины (`align < min_normal_alignment`), что может дробить зону разрыва на избыточное число мелких 1-элементных осколков.
