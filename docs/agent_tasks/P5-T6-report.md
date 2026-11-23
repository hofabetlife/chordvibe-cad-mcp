# Отчёт по задаче P5-T6: Неявный динамический интегратор `HHT-α` и серверное кэширование покадровых полей (MCP + SSE)

- **Дата:** 20.11.2026
- **Задача:** `P5-T6 [M]` — *Неявный динамический интегратор `HHT-α` и серверное кэширование покадровых полей (MCP + SSE)*
- **Статус:** ✅ **ВЫПОЛНЕНО**

---

## 1. Резюме изменений

1. **Разреженный прямой решатель `RCM LDL^T` и предобусловленные итерационные решатели `IC(0)-PCG` / `SSOR-PCG` (`crates/mcp-cad-fea/src/sparse_solver.rs`, новый модуль):**
   - Реализована перестановка обратного алгоритма Катхилла–Макки (`rcm_ordering`) для минимизации профиля заполнения симметричной матрицы.
   - Реализован разреженный прямой решатель `SparseLdlt` (символическая и численная $LDL^T$-факторизация по столбцам в формате CSC, устойчивая к симметричным плохо обусловленным и индефинитным системам с автоматической модификацией нулевых пивотов).
   - Реализованы итерационные решатели `solve_ic_pcg` (неполное разложение Холецкого `IC(0)` с автоматическим диагональным сдвигом на SPD-матрицах) и `solve_ssor_pcg`, а также диспетчер `solve_linear_robust` / `solve_spd_newton_robust` (`LinearSolverKind`: `Auto`, `SparseDirectLdlt`, `IcPcg`, `SsorPcg`).

2. **Безусловно устойчивый неявный динамический интегратор `HHT-α` с параллельной сборкой `rayon` (`crates/mcp-cad-fea/src/dynamics.rs`, новый модуль; зависимость `rayon` в `Cargo.toml`, экспорт в `lib.rs`):**
   - Реализован классический метод Хильбера–Хьюза–Тейлора (`HHT-α`) в двойной точности `f64` с параметрами $\alpha \in [-0.3, 0.0]$, $\beta = (1-\alpha)^2/4$, $\gamma = (1-2\alpha)/2$, гарантирующими безусловную устойчивость 2-го порядка и контролируемую высокочастотную диссипацию без ограничения Куранта (CFL).
   - Реализованы точные согласованные (`Consistent`) и строго положительные сосредоточенные (`Lumped`) матрицы масс $\mathbf{M}$ для обоих порядков элементов:
     - `TET4`: аналитическая согласованная матрица $M_{ab} = \frac{\rho V_e}{20}(1+\delta_{ab})$ и диагональная $\frac{\rho V_e}{4}$;
     - `TET10`: точная квадратичная согласованная матрица по точным барицентрическим интегралам (сумма строки $= \rho V_e$) и строго положительная диагональная матрица Хинтона–Рока–Зенкевича (HRZ: вершины $1/36$, серединные узлы $4/27$, сумма $4/36 + 24/27 = 1$).
   - Реализована многопоточная сборка векторов внутренних сил и матриц касательной жёсткости через `rayon::par_iter()` (`PARALLEL_ELEMENT_THRESHOLD = 64`), а также эффективная касательная Ньютона $\mathbf{K}_{\text{eff}} = \frac{1}{\beta \Delta t^2}\mathbf{M} + \frac{(1+\alpha)\gamma}{\beta \Delta t}\mathbf{C} + (1+\alpha)\mathbf{K}_T(\mathbf{u}_{n+1})$.
   - Реализовано адаптивное дробление шага (`bisection`) с полным откатом состояния узлов, точек Гаусса $J_2$-пластичности, истории повреждаемости и контактного набора при потере сходимости Ньютона, резком росте пластической деформации, скачке повреждаемости $\Delta D > \Delta D_{\max}$ или событии разрушения / контакта.
   - Добавлены однотельный (`solve_dynamic_hht_single_body`) и многотельный контактный (`solve_dynamic_hht_coupled`) решатели с поддержкой `ContactPairSpec` (`DeformablePair` / `DeformableToRigid`), `TimeForceCurve`, `estimate_cfl_critical_dt`, потоковых колбэков прогресса (`DynamicProgressUpdate`) и атомарной отмены через `AtomicBool`.

3. **Синхронизированное адаптивное шагание отделившихся фрагментов в `mcp-cad-physics` (`crates/mcp-cad-physics/src/{simulator.rs, lib.rs}`):**
   - Добавлены структура `CoupledFragmentPose` и методы `PhysicsWorld::step_adaptive_substeps` / `PhysicsWorld::step_coupled_fragments_for_dt`, позволяющие синхронно интегрировать траектории отделившихся в ходе `HHT-α` осколков на произвольном адаптивном шаге $\Delta t$ решателя МКЭ с внутренним подшагом безопасности и сохранением механической энергии.

4. **Серверное кэширование покадровых полей, MCP-инструменты №42–№43 и HTTP/SSE API (`crates/mcp-cad-server/src/{tools/coupled_sim.rs, tools/mod.rs, state.rs, sse.rs}`):**
   - Создан менеджер сопряжённых расчётов `CoupledSimulationManager` с поддержкой уровней точности `CoupledAccuracyLevel` (`standard` / `high` / `ultra`), хранением кэша кадров `BakedSimulationRecord` / `BakedFrameCache` (деформированные координаты узлов $\mathbf{x}(t) = \mathbf{X} + \mathbf{u}(t)$, поля $\sigma_{vM}$, $\bar{\varepsilon}_p$, $D$, эродированные элементы, синхронизированные позы отделившихся фрагментов из `PhysicsWorld`, контактные силы и баланс энергий $E_k, E_e, E_p, E_f$), потоковой рассылкой событий прогресса `CoupledSimSseEvent` (`tokio::sync::broadcast`) и безопасной атомарной отменой (`BakeJobState`: `Running` → `Cancelling` → `Cancelled` с коммитом частичного кэша до перехода в `Cancelled`).
   - Зарегистрированы и интегрированы в реестр (всего **43** Rust MCP-инструмента):
     - **№42 `bake_coupled_simulation`** — запуск синхронного или асинхронного серверного предрасчёта (`accuracy_level`, `duration_s`, `alpha`, `mass_matrix_kind`, `linear_solver`, `loads`, `initial_velocities`, `enable_plasticity`, `enable_fracture`, `async_execution`) с прогрессом по SSE и возвратом `sim_id`/`job_id`;
     - **№43 `query_simulation_frame`** — выдача компактного покадрового кэша по `sim_id` (или последнему расчёту) и `frame_index` с опциональным пересчётом единиц длины (`length_unit`), выборкой полей и сводкой прогресса/баланса энергий.
   - В `sse.rs` добавлены:
     - мультиплексирование событий `event: coupled_sim` на `/sse` и выделенный поток `GET /api/coupled-sim/events`;
     - REST-эндпоинты для веб-студии: `POST /api/coupled-sim/bake`, `GET /api/coupled-sim/status`, `GET /api/coupled-sim/status/:job_id`, `POST /api/coupled-sim/cancel/:job_id`, `GET /api/coupled-sim/frames/:frame_index`, `GET /api/coupled-sim/frames/:sim_id/:frame_index`;
     - независимый от Vulkan роутер `build_coupled_sim_router` для модульной верификации HTTP/SSE-контракта.
   - Синхронизированы счётчики и документация **43** Rust / **31** TS MCP-инструментов в `src/cad/native_bridge.ts`, `src/cad/cad_engine.test.ts` (тест паритета `BUG-021`), `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `README.md`, `PROJECT_CONTEXT.md`, `docs/ARCHITECTURE.md`, `docs/MCP_API_REFERENCE.md`, `docs/CLAUDE_DESKTOP_SETUP.md`, `docs/ANTIGRAVITY_SETUP.md`.

---

## 2. Измеренные результаты верификации (`dynamic_hht_tests.rs`, `physics_tests.rs`, `sse::tests`)

| № | Эталон / тест | Режим и конфигурация | Измеренный результат | Критерий приёмки | Статус |
|---|---|---|---|---|---|
| **1** | **Свободный упругий осциллятор на 100 периодах без CFL-ограничения** (`test_hht_100_period_free_elastic_oscillator_cfl_free_energy_conservation`) | Стальная консоль $0.20\times 0.02\times 0.02\text{ м}$, $\Delta t = T_1/12 > 20\times\Delta t_{\text{CFL}}$: (а) `TET4 Consistent` ($\alpha=0$), (б) `TET4 Lumped Co-Rotated` ($\alpha=0$), (в) `TET10 HRZ Lumped` ($\alpha=-0.1$, `cfl_ratio >= 40x`) | `total_steps >= 1100` (100 периодов), `cfl_ratio_max >= 20x` (`TET4`) и `>= 40x` (`TET10`), дрейф полной механической энергии **`max_energy_drift_pct <= 0.5%`** в обоих режимах (согласованная и Co-Rotated сосредоточенная масса), для `TET10` $\alpha=-0.1$ полная энергия строго ограничена (`E_final <= 1.005 * E_init`) | Безусловная устойчивость без CFL и сохранение энергии свободного осциллятора $\le 0.5\%$ за 100 периодов | ✅ PASS |
| **2** | **Адаптивная бисекция шага при динамическом пластическом/хрупком разрушении** (`test_hht_adaptive_bisection_on_plasticity_and_fracture_pulse`) | `borosilicate_glass`, импульс растяжения $55\text{ кН}$ (предел $\approx 24\text{ кН}$), `HHT-α = -0.05`, `dt_initial = 5e-4 s`, `max_damage_jump_per_step = 0.25` | `total_bisections > 0`, `min_dt_used_s < dt_initial_s`, `max_damage > 0.5`, непустой набор эродированных элементов, колбэки прогресса доходят до `100.0%` | Автоматическое дробление шага при возникновении разрушения и сходимость импульсной задачи | ✅ PASS |
| **3** | **Многотельный динамический контакт и атомарная отмена `AtomicBool`** (`test_hht_multibody_deformable_contact_impact_and_cancellation`) | Блок `impactor` ($v_x = 4\text{ м/с}$) → `target` (`aluminum_6061_t6`, зазор $1\text{ мм}$) + плоскость пола (`DeformablePair` + `DeformableToRigid`) | Возникновение контактной нормальной силы (`normal_force_n > 1.0 Н`) и передача ударного напряжения в мишень (`max_von_mises > 0.1 МПа`); при выставлении `AtomicBool` на 3-м шаге расчёт чисто прерывается (`cancelled == true`, `total_steps <= 4`, событие `"cancelled"`) | Многотельный контактный удар в `HHT-α` + немедленная отмена расчёта | ✅ PASS |
| **4** | **Эквивалентность `rayon`-сборки и линейных решателей** (`test_rayon_parallel_assembly_and_linear_solvers_agree`) | 96 тетраэдров (`> PARALLEL_ELEMENT_THRESHOLD = 64`), `SparseDirectLdlt` vs `IcPcg` vs `SsorPcg` | Относительное расхождение максимальных перемещений между всеми тремя решателями **`< 1e-6`** | Детерминизм многопоточной сборки и согласие прямого/итерационных решателей | ✅ PASS |
| **5** | **Unit-тесты матриц масс, параметров `HHT-α` и разреженных решателей** (`dynamics::tests` + `sparse_solver::tests`, 4 теста) | `TET4`/`TET10` (`Consistent` и `HRZ Lumped`), плохо обусловленные и индефинитные матрицы с `RCM` | Точное сохранение массы `TET10` (согласованной и строго положительной HRZ), границы устойчивости $\alpha \in [-0.3, 0.0]$, совпадение `SparseLdlt` и `IC(0)-PCG` | Машинная точность инвариантов | ✅ PASS |
| **6** | **Синхронизированное адаптивное шагание фрагментов в `PhysicsWorld`** (`test_p5_t6_adaptive_hht_coupled_fragment_stepping_and_energy_balance`, `physics_tests.rs`) | Неравномерные адаптивные шаги `HHT-α` (`[0.02, 0.0005, 0.015, ...]`) с коллизией о стенд | Корректное обновление `CoupledFragmentPose`, сохранение энергии на свободном участке и отсутствие туннелирования | Синхронизация `mcp-cad-physics` с адаптивным шагом `HHT-α` | ✅ PASS |
| **7** | **HTTP/SSE-поток прогресса, отмена джобы, кэш кадров и контракт 43 MCP-инструментов** (`sse::tests`, 3 теста) | `build_coupled_sim_router`: `POST /api/coupled-sim/bake`, `/api/coupled-sim/events`, `POST /api/coupled-sim/cancel/:job_id`, `GET /api/coupled-sim/frames/:sim_id/:idx`, `tools_list_json` | Реестр возвращает ровно **43** MCP-инструмента (включая `bake_coupled_simulation` и `query_simulation_frame`); SSE-подписчик получает события прогресса/завершения; REST-отмена переводит задачу в `cancelled` с доступными частичными кадрами; запрос кадра возвращает узловые координаты, поля $\sigma_{vM}/\bar{\varepsilon}_p/D$ и баланс энергий | Полный цикл предрасчёта, прогресса SSE, отмены и выдачи покадрового кэша | ✅ PASS |

---

## 3. Верификация тестами, сборкой и линтером

Выполнено после всех финальных правок:
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
- `cargo test -p mcp-cad-fea --no-fail-fast` (`--lib`: `22/22`, `dynamic_hht_tests`: `4/4`, `deformable_contact_tests`: `5/5`, `fea_benchmarks`: `3/3`, `fea_tests`: `14/14`, `local_fracture_tests`: `4/4`, `nonlinear_elastoplastic_tests`: `4/4`): **56/56 PASS** (`0 failed`).
- `cargo test -p mcp-cad-physics --no-fail-fast` (`--lib`: `12/12`, `physics_tests`: `34/34`): **46/46 PASS** (`0 failed`).
- `cargo test -p mcp-cad-core --no-fail-fast` (`--lib`: `79/79`, `csg_invariants`: `10/10`, `dag_tests`: `16/16`, `primitive_invariants`: `25/25`, `tessellation_tests`: `6/6`): **136/136 PASS** (`0 failed`).
- `cargo test -p mcp-cad-server --lib sse::tests --no-fail-fast`: **3/3 PASS** (`0 failed`). *(Примечание: интеграционные тесты `mcp-cad-server`, создающие `EngineState::new()`, требуют Vulkan ICD / CPU-fallback из задачи `P5-T9`.)*
- `npm test` (`src/cad/cad_engine.test.ts` + `src/cad/__tests__/analyze_mechanism.test.ts`, включая `BUG-021` на паритет 43 Rust / 31 TS MCP-инструментов): **55/55 PASS** (`0 failed`).
- `npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`): **PASS** (`exit 0`).

---

## 4. Выявленная сопутствующая задача (заведена в `TASKS.md`)

- **`P5-T14 [S]`:** *Передача активных шарниров и моторов (`Revolute`, `Prismatic`, `Gear`) из `PhysicsWorld` в динамический интегратор `HHT-α` (`solve_dynamic_hht_coupled`) и инструмент `bake_coupled_simulation` в виде динамических ограничений ступиц (`HubCoupling`)* — сейчас `bake_coupled_simulation` передаёт в `HHT-α` закреплённые детали, гравитацию, явные `loads`/`initial_velocities` и контактные пары между телами, а также шагает отделившиеся фрагменты через `PhysicsWorld::step_coupled_fragments_for_dt`. Автоматическое пробрасывание шарнирных моторов из `PhysicsWorld` в `HubCoupling` позволит запускать `bake_coupled_simulation` на готовых кинематических сборках (зубчатых передачах и кривошипах, созданных через `add_joint`) без ручного задания начальных скоростей.

---

## 5. Затронутые файлы

- `crates/mcp-cad-fea/Cargo.toml`
- `crates/mcp-cad-fea/src/sparse_solver.rs` *(новый)*
- `crates/mcp-cad-fea/src/dynamics.rs` *(новый)*
- `crates/mcp-cad-fea/src/lib.rs`
- `crates/mcp-cad-fea/tests/dynamic_hht_tests.rs` *(новый)*
- `crates/mcp-cad-physics/src/simulator.rs`
- `crates/mcp-cad-physics/src/lib.rs`
- `crates/mcp-cad-physics/tests/physics_tests.rs`
- `crates/mcp-cad-server/src/tools/coupled_sim.rs` *(новый)*
- `crates/mcp-cad-server/src/tools/mod.rs`
- `crates/mcp-cad-server/src/state.rs`
- `crates/mcp-cad-server/src/sse.rs`
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs`
- `src/cad/native_bridge.ts`
- `src/cad/cad_engine.test.ts`
- `README.md`, `PROJECT_CONTEXT.md`, `docs/ARCHITECTURE.md`, `docs/MCP_API_REFERENCE.md`, `docs/CLAUDE_DESKTOP_SETUP.md`, `docs/ANTIGRAVITY_SETUP.md`
- `docs/agent_tasks/P5-T6-report.md` *(новый)*
- `TASKS.md`, `docs/PROGRESS.md`
