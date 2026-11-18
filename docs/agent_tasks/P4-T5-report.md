# Отчёт о выполнении задачи P4-T5

- **Задача**: `P4-T5` — Прямое извлечение реакций шарниров $\lambda$ для `get_part_load_cases` и `run_fea_from_simulation`
- **Статус**: ✅ ВЫПОЛНЕНО (`18.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-physics/src/solver.rs`](../../crates/mcp-cad-physics/src/solver.rs) *(связанный пересчёт KKT-системы на шаге 7 с закреплением активных импульсов моторов `step_motor_impulses` и односторонних упоров `step_limit_impulses` из шага 3, сохранение полных якоревых пар $(\mathbf{F}_A, \boldsymbol{\tau}_{A,\text{anchor\_a}})$ и $(\mathbf{F}_B, \boldsymbol{\tau}_{B,\text{anchor\_b}})$ в `SolvedJointRecord` через `convert_com_torque_to_anchor_couple`)*
  - [`crates/mcp-cad-physics/src/simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs) *(прямое извлечение реакций шарниров из `SolvedJointRecord` без декодера импульсов `Rapier3D`, точное приведение вектора сил и моментов на `part_b` в `joint_wrench_on_part` для всех 5 типов шарниров с учётом смещения ползуна $\mathbf{d} = \mathbf{p}_b - \mathbf{p}_a$ в `Prismatic` и передаточного отношения $\text{ratio}$ в `Gear`)*
  - [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs) *(ужесточение допусков `test_p3_t6_static_cantilever_beam_load_case_and_newton_third_law` до `< 0.01%` по реакциям и `< 1e-12` по III закону Ньютона, `test_p3_t6_rotating_rod_centripetal_reaction_force_in_local_frame` до `< 0.5%`, добавление комплексного теста `test_p4_t5_exact_lagrange_reactions_limits_motors_gear_and_prismatic`)*
  - [`TASKS.md`](../../TASKS.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md)

---

## 1. Контекст и постановка задачи

В предыдущей архитектуре на базе `Rapier3D 0.22` функция `PhysicsWorld::compute_joint_world_wrench_on_part_a` представляла собой 280-строчный декодер внутренних импульсов `Rapier3D`, восстанавливавший дробный шаг `substep_dt`, инверсию `JointOneBodyConstraintBuilder` (`body1 <-> body2`), базис `diff_conj1_2` и прогонявший ортогонализацию Грама–Шмидта для приближённого выделения момента в якоре.

Задача `P4-T5` переводит конвейер динамического нагружения `joint_reaction_force` $\to$ `extract_part_load_case` (`get_part_load_cases`) $\to$ `run_fea_from_simulation` на прямое извлечение вектора множителей Лагранжа $\boldsymbol{\lambda}$ непосредственно из решения KKT-системы собственного `f64`-решателя `MultiBodySolver` (`P4-T3`), включая реакции односторонних упоров `limits`, приводов `MotorConfig`, смещённых ползунов `Prismatic` и зубчатых зацеплений `Gear`.

---

## 2. Архитектура и математическая модель

### 2.1. Связанное извлечение множителей Лагранжа $\boldsymbol{\lambda}$ с учётом `MotorConfig` и `limits` (`crates/mcp-cad-physics/src/solver.rs`)

1. **Проблема пост-шагового зануления импульсов приводов и упоров**:
   - На шаге 3 (`Step 3: Solve KKT impulse system`) решатель вычисляет импульсы $\boldsymbol{\gamma} = \boldsymbol{\lambda}\Delta t$, переводящие систему из невозмущённой скорости $\mathbf{v}^*$ в скорость $\mathbf{v}_{k+1}$, удовлетворяющую связям, упорам `limits` и целевым скоростям моторов `MotorConfig`.
   - На шаге 7 (`Step 7: Post-projection velocity manifold cleanup & reaction extraction`) скорость $\tilde{\mathbf{v}}$ уже лежит на многообразии связей ($\dot{\Phi} \approx 0$) и близка к целевой скорости ненасыщенного привода или отскочившего упора. Поэтому в повторной KKT-проекции без закрепления импульса строка мотора/упора имела бы нулевую невязку ($b_{\text{motor}} \approx 0$), а простое пост-фактум прибавление `step_motor_torque_on_a` без пересчёта опорных строк нарушало бы поперечный баланс сил Даламбера (например, поперечную реакцию в шарнире при угловом ускорении эксцентричного рычага мотором).
2. **Связанный пересчёт KKT с закреплёнными активными импульсами (`Pinned KKT Re-solve`)**:
   - Импульсы строк приводов (`step_motor_impulses`) и односторонних упоров (`step_limit_impulses`), вычисленные на шаге 3, сохраняются по ключу `(joint_idx, row_role)`.
   - На шаге 7, если на шаге 3 привод или односторонний упор развил больший импульс, чем пост-проекционный остаток, соответствующая строка в `rows_post` закрепляется (`lower_impulse = upper_impulse = step_imp`) и вызывается `solve_kkt_system(&rows_post, ...)`.
   - В результате все двусторонние опорные строки шарнира и строки `Gear` автоматически уравновешивают закреплённый момент/силу привода или упора в конфигурации $\mathbf{q}_{k+1}$, обеспечивая точное выполнение уравнений динамического равновесия Даламбера.
3. **Приведение моментов из ЦМ тел в якоря шарниров (`convert_com_torque_to_anchor_couple`)**:
   - Строки линейных ограничений приложены в мировом якоре $\mathbf{p}_a = \mathbf{x}_a + R_a \mathbf{r}_a^{\text{local}}$, создавая относительно ЦМ тел плечевые моменты $(\mathbf{p}_a - \mathbf{x}_a) \times \mathbf{F}_A$ и $(\mathbf{p}_a - \mathbf{x}_b) \times \mathbf{F}_B$.
   - Функция `convert_com_torque_to_anchor_couple` вычитает плечевой вклад из суммарного момента относительно ЦМ и сохраняет чистые пары сил $\boldsymbol{\tau}_{A,\text{anchor\_a}}$ (`last_torque_on_a`) и $\boldsymbol{\tau}_{B,\text{anchor\_b}}$ (`last_torque_on_b`) в `SolvedJointRecord`.

### 2.2. Точный перенос реакций на `part_a` и `part_b` в `extract_part_load_case` (`crates/mcp-cad-physics/src/simulator.rs`)

Вспомогательный метод `PhysicsWorld::joint_wrench_on_part` переводит сохранённую в телеметрии реакцию $(\mathbf{F}_A, \boldsymbol{\tau}_{A,\text{anchor\_a}})$ на целевую деталь (`is_part_a: bool`) на шаге `step_idx`:
- **`Fixed`, `Revolute`, `Spherical`** (совпадающие якоря $\mathbf{p}_b = \mathbf{p}_a$):
  $$\mathbf{F}_B = -\mathbf{F}_A, \qquad \boldsymbol{\tau}_{B,\text{anchor\_b}} = -\boldsymbol{\tau}_{A,\text{anchor\_a}},$$
  что гарантирует выполнение III закона Ньютона $\mathbf{F}_A + \mathbf{F}_B = \mathbf{0}$, $\boldsymbol{\tau}_A + \boldsymbol{\tau}_B = \mathbf{0}$ с машинной точностью (`< 1e-15`).
- **`Prismatic`** (якорь $\mathbf{p}_b = \mathbf{p}_a + s\,\hat{\mathbf{u}}$ смещён вдоль оси скольжения):
  $$\mathbf{F}_B = -\mathbf{F}_A, \qquad \boldsymbol{\tau}_{B,\text{anchor\_b}} = -\boldsymbol{\tau}_{A,\text{anchor\_a}} + (\mathbf{p}_b - \mathbf{p}_a) \times \mathbf{F}_A,$$
  учитывая момент переноса линии действия поперечной реакции $\mathbf{F}_A$ из точки $\mathbf{p}_a$ в точку $\mathbf{p}_b$.
- **`Gear`** (связанное кинематическое ограничение $\boldsymbol{\omega}_b \cdot \hat{\mathbf{z}}_b + \text{ratio}\,(\boldsymbol{\omega}_a \cdot \hat{\mathbf{z}}_a) = 0$):
  $$\mathbf{F}_B = \mathbf{0}, \qquad \boldsymbol{\tau}_B = \frac{\boldsymbol{\tau}_A \cdot \hat{\mathbf{z}}_a}{\text{ratio}}\,\hat{\mathbf{z}}_b,$$
  в точном соответствии с якобианом строки зубчатого зацепления ($\mathbf{j}_{\text{ang},a} = \text{ratio}\,\hat{\mathbf{z}}_a$, $\mathbf{j}_{\text{ang},b} = \hat{\mathbf{z}}_b$).

---

## 3. Результаты верификации точности и инвариантов

1. **Статическая консольная балка в поле тяжести (`test_p3_t6_static_cantilever_beam_load_case_and_newton_third_law`)**:
   - Относительная погрешность вертикальной силы реакции $F_z = mg$ и изгибающего момента $M_y = mgL/2$ в заделке: **`< 0.0001%` (`< 1e-6`, требование `< 0.01%`)**;
   - Невязка III закона Ньютона между консолью и опорой: **`0.0` (`< 1e-12`)**;
   - Относительная невязка равновесия Даламбера (`equilibrium_residual_force_norm`, `equilibrium_residual_torque_norm`): **`< 1e-6`**.
2. **Вращающийся стержень (`test_p3_t6_rotating_rod_centripetal_reaction_force_in_local_frame`, $\omega_0 = 10\text{ рад/с}$)**:
   - Относительная погрешность центростремительной реакции $F_x = -m\omega^2 r_{\text{com}}$ в локальной СК стержня: **`< 0.01%` (требование `< 0.5%`)**;
   - Относительная невязка динамического равновесия Даламбера: **`< 1e-6`**.
3. **Упоры `limits`, привод `MotorConfig`, передача `Gear` и смещённый `Prismatic` (`test_p4_t5_exact_lagrange_reactions_limits_motors_gear_and_prismatic`)**:
   - **Упор `Revolute` (`limits = [0.0, 0.5]`)**: удерживает горизонтальный рычаг в поле тяжести с точностью реакции и момента упора **`< 0.01%`**, невязкой равновесия **`< 1e-9`** и III законом Ньютона **`< 1e-12`**;
   - **Мотор `Revolute` (`target_velocity = 4.0`, `max_force = 3.0 N·m`)**: разгоняет эксцентричный стержень в невесомости с насыщенным моментом $|\tau_z| = 3.0\text{ Н}\cdot\text{м}$ (погрешность `< 1e-3`), связанной поперечной реакцией опоры $F_y = m a_{\text{com},y}$ (`> 1.0 Н`), невязкой Даламбера **`< 1e-6`** и III законом Ньютона **`< 1e-12`**;
   - **Зубчатая пара `Gear` (`ratio = 0.5`)**: извлечение нагрузок `extract_part_load_case` как на ведущем колесе `gear_a` (`part_a`), так и на ведомом колесе `gear_b` (`part_b`) даёт невязку крутящего момента Даламбера **`< 1e-8 Н·м`** на обоих колёсах при точном отношении моментов $|\tau_B / \tau_A| = 1 / \text{ratio} = 2.0$;
   - **Смещённый ползун `Prismatic` ($s = 0.25\text{ м}$)**: под поперечной гравитационной нагрузкой перенос момента $(\mathbf{p}_b - \mathbf{p}_a) \times \mathbf{F}_A$ обеспечивает невязку силового и моментного равновесия ползуна **`< 1e-9`**.
4. **Сквозной конвейер `get_part_load_cases` $\to$ `run_fea_from_simulation` (`test_p3_t7_*`, `test_mcp_p3_t6_*`, `test_mcp_p3_t7_*`)**:
   - Все модульные и контрактные MCP-тесты статического и квазистатического МКЭ (`inertial_relief`) на базе нагрузок из `f64`-симуляции проходят без регрессий.

---

## 4. Обнаруженные смежные проблемы

Новых проблем, выходящих за рамки запланированных задач `P4-T6`–`P4-T9` (удаление `rapier3d` из `Cargo.toml`/`Cargo.lock` и синхронизация ярлыков веб-студии в `P4-T6`), не выявлено.

---

## 5. Верификация

1. `cargo test -p mcp-cad-physics --no-fail-fast`: **38/38 PASS** (`--lib`: **8/8**, `physics_tests`: **30/30**, включая обновлённые `test_p3_t6_*` и новый `test_p4_t5_exact_lagrange_reactions_limits_motors_gear_and_prismatic`; `0 failed`).
2. `cargo test -p mcp-cad-fea --test fea_tests`: **11/11 PASS**.
3. `cargo test -p mcp-cad-server --lib --no-fail-fast`: **65/65 PASS** (включая `test_p3_t6_get_part_load_cases_unit`, `test_p3_t7_run_fea_from_simulation_cantilever_gravity`, `test_p3_t7_run_fea_from_simulation_rotating_rod_inertial_relief`).
4. `cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_p3_t6` & `test_mcp_p3_t7`: **PASS**.
5. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений).
6. `npm run lint`, `npm run build`, `npm test` (`52/52 PASS`): **PASS** (`exit 0`).
