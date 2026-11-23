# Отчёт по задаче `P5-T14`: Автоматическая передача шарниров и моторов (`Revolute`, `Prismatic`, `Gear`) из `PhysicsWorld` в динамический интегратор `HHT-α` и `bake_coupled_simulation`

- **Дата выполнения**: 22.11.2026
- **Статус**: ✅ DONE

---

## 1. Реализованные изменения

### 1.1. Извлечение шарниров, моторов и зубчатых пар из `PhysicsWorld` (`crates/mcp-cad-physics`)
- В `crates/mcp-cad-physics/src/solver.rs`:
  - Исправлена нормализация знака обобщённой координаты в `extract_joint_hub_couplings`: когда `part_b` статичен, а `part_a` динамический (`!bb.is_dynamic && ba.is_dynamic`), ось и знак мотора/упоров корректно инвертируются (`coord_sign = -1.0`), гарантируя единое направление вращения/поступательного хода независимо от порядка `(part_a, part_b)`.
  - Добавлена структура `GearJointCouplingSpec` (`joint_id`, `part_a`, `part_b`, `ratio`, `axis_a_world`, `axis_b_world`) и функция `extract_gear_joint_couplings`, извлекающая активные сочленения `CadJointType::Gear`.
- В `crates/mcp-cad-physics/src/simulator.rs` и `crates/mcp-cad-physics/src/lib.rs`:
  - Добавлены методы `PhysicsWorld::solved_joints()`, `PhysicsWorld::extract_joint_hub_couplings()` и `PhysicsWorld::extract_gear_joint_couplings()`, реэкспортированы `GearJointCouplingSpec` и `extract_gear_joint_couplings`.

### 1.2. Динамические ступицы `DynamicHubCoupling` в интеграторе `HHT-α` (`crates/mcp-cad-fea/src/dynamics.rs`, `lib.rs`)
- Добавлены публичные типы (`mcp_cad_fea::{DynamicRevoluteDrive, DynamicPrismaticDrive, DynamicHubCoupling, DynamicHubReaction}`):
  - `DynamicHubCoupling::Revolute` (режимы `PrescribedVelocity`, `Motor` с гладким насыщением по моменту и точным аналитическим касательным демпфированием $d(-Q)/d\dot{q} = c_m / (1 + (c_m \Delta\dot{q}/\tau_{\max})^2)^{3/2}$, и `FreeTorque` — пассивный подшипник/пружина/демпфер/упоры);
  - `DynamicHubCoupling::Prismatic` (режимы `PrescribedVelocity`, `Motor`, `FreeForce` с опциональной блокировкой поперечных DOF);
  - `DynamicHubCoupling::Fixed` (жёсткое закрепление выбранной группы узлов ступицы/опоры).
- Расширены `DynamicBodySpec` (`hub_couplings`, строители `with_hub_coupling` / `with_hub_couplings`) и `DynamicFrameSnapshot` (`hub_reactions: Vec<DynamicHubReaction>` с координатой $q$, скоростью $\dot{q}$, обобщённой реакцией `reaction_effort`, векторами силы и момента).
- Реализована полная динамическая кинематика RBE2 в `solve_dynamic_hht_coupled`:
  - Точный конечный поворот Родрига на $SO(3)$ для узлов ступицы ($\mathbf{u}_i = \mathbf{R}(\theta)\mathbf{r}_{0,i} - \mathbf{r}_{0,i}$, $\mathbf{v}_i = \dot{\theta}\mathbf{t}_i$, $\mathbf{a}_i = \ddot{\theta}\mathbf{t}_i + \dot{\theta}^2(\mathbf{e}\times\mathbf{t}_i)$) и $C^2$-косинусный плавный пуск моторов/предписанных скоростей (`ramp_duration_s`).
  - Единая редуцированная система степеней свободы `DynamicDofSystem` (свободные узловые DOF + скалярные пилот-DOF активных/свободных шарниров) с проекцией $\mathbf{T}^T \mathbf{K}_{\text{eff}} \mathbf{T}$, $\mathbf{T}^T \mathbf{M} \mathbf{T}$, согласованными начальными ускорениями $\ddot{\mathbf{y}}_0$ (`compute_initial_accelerations`), сохранением/откатом состояния пилотов при адаптивной бисекции и расчётом HHT-$\alpha$-сбалансированных реакций ступиц (`compute_all_hub_reactions`).
  - Добавлена стабилизация активного множества мастер-граней `FrozenDeformableContact` после 4 итераций BVH-поиска в цикле Ньютона (устраняет 2-циклы на рёбрах граней полиэдральных сеток при сохранении точной непрерывной оценки зазора и трения), а также передача фактической невязки `last_rel_res` в `FeaError::SolverConvergenceFailed`.

### 1.3. Автоматическая передача шарниров в `bake_coupled_simulation` и HTTP/SSE API (`crates/mcp-cad-server`)
- В `crates/mcp-cad-server/src/tools/coupled_sim.rs`:
  - Реализованы `select_dynamic_hub_nodes_for_joint` (автоматический выбор узлов внутреннего отверстия/ступицы вокруг оси шарнира с сохранением зубчатого венца свободным для деформируемого контакта), `map_physics_joint_to_dynamic_hub` и `prepare_coupled_bake_from_parts_with_physics`.
  - При `use_physics_joints: true` (по умолчанию) `bake_coupled_simulation` и `POST /api/coupled-sim/bake` автоматически извлекают активные `Revolute`, `Prismatic`, `Fixed` и `Gear` шарниры с моторами из `PhysicsWorld`, закрепляют неподвижные детали (`part.is_fixed`) без подвижных шарниров, для деталей в зубчатой паре `Gear` повышают режим сетки с `AccuracyMode::Fast` до гранично-согласованного `AccuracyMode::Standard` (чтобы воксельная «лесенка» не создавала ложного перекрытия зубьев при $t=0$) и настраивают контактные фильтры (`max_normal_dot: Some(0.25)`, ограниченный `search_distance`).
  - В сводку `bake_coupled_simulation` добавлены поля `applied_joint_couplings_count`, `applied_joint_couplings`, `applied_gear_couplings`, а в покадровый кэш `BakedFrameCache` / `query_simulation_frame` — поле `hub_reactions`.
- В `crates/mcp-cad-server/src/sse.rs` и `crates/mcp-cad-server/src/tools/mod.rs`:
  - В `CoupledSimHttpState` добавлен `physics_world: Arc<RwLock<PhysicsWorld>>`, проброшенный в обработчики `POST /api/coupled-sim/bake`.
  - В JSON-схему `bake_coupled_simulation` добавлен опциональный параметр `use_physics_joints` (сохраняя контракт ровно 43 инструментов).

### 1.4. Новые интеграционные тесты
1. `crates/mcp-cad-fea/tests/dynamic_hht_tests.rs`:
   - `test_p5_t14_dynamic_gear_pair_revolute_motor_contact_and_torque_transmission`: пара эвольвентных шестерён (12T + 12T, $m = 3\text{ мм}$), ведущая шестерня на `Revolute::Motor`, ведомая на пассивном `Revolute::FreeTorque`, без `initial_velocities` — подтверждает вращение ведущей CCW, контакт зубьев ($F_n > 1\text{ Н}$), вращение ведомой CW, передачу крутящего момента на обеих ступицах и ненулевые напряжения $\sigma_{vM}$.
   - `test_p5_t14_dynamic_prismatic_motor_and_fixed_hub_couplings`: ползун с приводом `Prismatic::Motor`, ударяющийся о наковальню с опорой `Fixed` — подтверждает поступательное движение ползуна, нулевое смещение `Fixed`-ступицы, контактную силу и реакцию опоры.
2. `crates/mcp-cad-server/src/sse.rs` (`sse::tests`):
   - `test_p5_t14_bake_coupled_simulation_auto_extracts_revolute_joints_and_motor`: собирает пару прямозубых шестерён (`create_spur_gear`, 8T + 8T, $m = 4\text{ мм}$, зазор `0.4 мм`), регистрирует их и 2 шарнира `Revolute` (один с `MotorConfig`) + `Gear` в `PhysicsWorld`, вызывает `POST /api/coupled-sim/bake` **без** ручных `initial_velocities`/`fixed_faces`/`loads`, подтверждает HTTP 200, извлечение 2 `Revolute` и 1 `Gear`, контакт зубьев ($> 1\text{ Н}$), вращение ведущей CCW ($\theta > 0.02\text{ рад}$, пик $\omega > 5\text{ рад/с}$), ведомой CW ($\theta < -0.002\text{ рад}$, $\min\omega < -0.5\text{ рад/с}$) и ненулевой момент реакции ступицы.

---

## 2. Результаты верификации

1. `cargo test -p mcp-cad-physics --no-fail-fast`: **47/47 PASS** (`--lib`: `12/12` за `2.15s`, `physics_tests`: `35/35` за `10.68s`, `0 failed`).
2. `cargo test -p mcp-cad-fea --lib --test dynamic_hht_tests --no-fail-fast`: **31/31 PASS** (`--lib`: `25/25` за `0.34s`, `dynamic_hht_tests`: `6/6` за `14.53s`, включая оба новых теста `P5-T14`).
3. `cargo test -p mcp-cad-server --lib sse::tests --no-fail-fast`: **4/4 PASS** (`2.98s`, включая `test_p5_t14_bake_coupled_simulation_auto_extracts_revolute_joints_and_motor` и контракт `test_p5_t6_mcp_tool_registry_contract_43_tools`). *(В полном прогоне `cargo test -p mcp-cad-server --lib` 71/72 PASS; единственное падение `tools::project::tests::test_save_and_load_roundtrip` — предсуществующая задача `P5-T17`.)*
4. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах, `8.61s`).
5. `npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`): **PASS**.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)

- **`P5-T23 [S]`**: В ходе отладки динамического контакта зубчатых пар в `HHT-α` (`crates/mcp-cad-fea/src/dynamics.rs`) выявлены две проблемы устойчивости при экстремальных начальных перекрытиях/напряжениях:
  1. **Начальное перекрытие грубых сеток при $t = 0$**: если две грубые тетраэдральные сетки (например, беззазорная пара `create_spur_gear` при номинальном межосевом расстоянии $a = m(z_1+z_2)/2$) имеют геометрическое перекрытие в момент $t=0$, `compute_initial_accelerations` прикладывает полную штрафную контактную силу в первый момент времени, порождая нефизический импульс и многогигапаскальные напряжения.
  2. **Геометрическая жёсткость вращающейся RBE2-ступицы**: при больших внутренних напряжениях ($\sigma \sim \mathcal{O}(E)$) и конечных поворотах `Revolute`-пилота отсутствие геометрической жёсткости $\mathbf{K}_{\text{geo}}(\boldsymbol{\sigma})$ и производной $\partial\mathbf{t}_i/\partial\theta$ в касательной матрице Ньютона может замедлять/останавливать backtracking line search.
