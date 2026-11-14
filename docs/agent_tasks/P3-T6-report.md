# P3-T6 Report: Извлечение пиковых нагрузок из динамической симуляции (`get_part_load_cases`)

**Task ID:** `P3-T6`  
**Status:** ✅ DONE  
**Component:** `mcp-cad-physics`, `mcp-cad-server`  
**Branch:** `agent/P3-T6`

---

## 1. Обзор задачи

Реализован расчётный и MCP-контрактный механизм извлечения пиковых и мгновенных динамических/квазистатических нагрузок, действующих на деталь со стороны кинематических шарниров и сил инерции, в **локальной системе координат детали** (`get_part_load_cases(part_id, select)`):
- Для каждого шарнира, подключённого к `part_id` (`part_a == part_id` или `part_b == part_id`), возвращаются `{joint_id, joint_type, connected_part_id, role, anchor_local: [x, y, z], force: [Fx, Fy, Fz], torque: [Tx, Ty, Tz], force_world, torque_world, force_magnitude, torque_magnitude}` с физически корректным знаком — сила и момент, действующие **НА эту деталь** (с выполнением 3-го закона Ньютона $\mathbf{F}_{A \leftarrow B} = -\mathbf{F}_{B \leftarrow A}$, $\boldsymbol{\tau}_{A \leftarrow B} = -\boldsymbol{\tau}_{B \leftarrow A}$ в мировой СК перед поворотом в локальную СК каждой детали).
- Поддерживаются три режима `select`: `"peak_force"` (шаг с максимальной реакцией в шарнирах), `"peak_torque"` (шаг с максимальным моментом реакции) и конкретный момент времени `time_s` (число $\ge 0$ в секундах).
- Возвращается инерционная часть (`mass`, `center_of_mass_local`, линейное ускорение ЦМ `linear_acceleration` в $\text{м/с}^2$, угловое ускорение `angular_acceleration` в $\text{рад/с}^2$) и невязка динамического равновесия по силам $\mathbf{R}_F = \sum \mathbf{F}_{\text{joints}} + m \mathbf{g} - m \mathbf{a}_{\text{COM}}$ и моментам относительно ЦМ $\mathbf{R}_T = \sum (\boldsymbol{\tau}_j + (\mathbf{p}_j - \mathbf{x}_{\text{COM}}) \times \mathbf{F}_j) - (\mathbf{I}\boldsymbol{\alpha} + \boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega})$.

---

## 2. Физико-математическое устройство импульсов шарниров в Rapier3D 0.22

Анализ исходного кода решателя ограничений Rapier3D 0.22 (`island_solver.rs`, `velocity_solver.rs`, `joint_constraint_builder.rs`, `joint_velocity_constraint.rs`) выявил три ключевых свойства внутренних импульсов `ImpulseJoint`, учтённых в `PhysicsWorld::compute_joint_world_wrench_on_part_a`:

1. **Дробный шаг TGS-Soft (`substep_dt = dt / num_solver_iterations`)**:
   - В `IslandSolver::init_and_solve` полный шаг интегрирования $\Delta t$ делится на `num_substeps = num_solver_iterations + additional_solver_iterations` (по умолчанию `4`) подшагов длительностью $\Delta t_{\text{substep}} = \Delta t / \text{num\_substeps}$.
   - На каждом подшаге `joint_constraints.update(...)` пересоздаёт строки ограничений с `impulse = 0`, решает импульс **одного подшага**, а `writeback_impulses` в конце шага записывает импульс последнего подшага в `joint.impulses[0..6]`, `joint.data.motors[0..6].impulse` и `joint.data.limits[0..6].impulse`.
   - Следовательно, физическая сила/момент равны $\text{impulse} / \Delta t_{\text{substep}} = \text{impulse} \cdot \text{num\_substeps} / \Delta t$.

2. **Перестановка тел в однотельных ограничениях (`JointOneBodyConstraintBuilder::generate`)**:
   - Если `body2` (`part_b`) — неподвижное тело (`!rb2.is_dynamic()`), а `body1` (`part_a`) — динамическое, Rapier3D меняет местами `handle1 <-> handle2` и вызывает `joint_data.flip()`, так что внутри `JointOneBodyConstraint` `body2` всегда является динамическим телом, к которому применяется импульс $-\lambda \mathbf{J}_{\text{lin}}$.
   - Также угловой якобиан `ang_basis` для заблокированных угловых осей формируется через `diff_conj1_2(q1, q2)` с множителем `0.5`.

3. **Модифицированная ортогонализация Грама–Шмидта в `finalize_constraints`**:
   - В `lock_axes` строки заблокированных угловых степеней свободы (`i = 3..6`) добавляются **перед** строками заблокированных линейных степеней свободы (`i = 0..3`).
   - Затем `finalize_constraints` выполняет модифицированную ортогонализацию Грама–Шмидта в пространстве приведённых обратных инерций ($\mathbf{J}_{\text{ang}} = \mathbf{I}^{-1/2} (\mathbf{r} \times \mathbf{u})$).
   - Для шарнира `Fixed` (где заблокированы все 3 угловые оси) ортогонализация вычитает момент плеча $\mathbf{r} \times \mathbf{u}$ из линейных строк, перенося эффективную линию действия линейных импульсов `joint.impulses[0..3]` в центр масс тела (из-за чего `joint.impulses[3..6]` для горизонтальной консольной балки под действием тяжести в ЦМ равны нулю).
   - Отслеживание вектора чистого момента в якоре $\boldsymbol{\tau}_{\text{anchor}}$ синхронно с $(\mathbf{J}_{\text{lin}}, \mathbf{J}_{\text{ang1}}, \mathbf{J}_{\text{ang2}})$ при прохождении того же цикла Грама–Шмидта точно восстанавливает физический реактивный момент $\boldsymbol{\tau}_{\text{anchor}}$ в точке крепления шарнира для всех типов соединений (`Fixed`, `Revolute`, `Prismatic`, `Spherical`, а также моторов и ограничителей хода).

---

## 3. Проверка критериев приёмки и аналитических эталонов

1. **Тест (a) — Статическая консольная балка (`test_p3_t6_static_cantilever_beam_load_case_and_newton_third_law`)**:
   - Балка длиной $L = 0.20\text{ м}$ ($200\text{ мм}$), сечением $20 \times 20\text{ мм}$, закреплённая шарниром `Fixed` в корне $[0, 0, 0]$ к неподвижной опоре (`wall_support`, `is_fixed = true`) в поле тяжести $g = 9.81\text{ м/с}^2$.
   - Реакция опоры на балку: $F_y \approx m g$ (отклонение $< 0.1\%$), реактивный момент в корневом якоре $[0, 0, 0]$: $T_z \approx m g L / 2$ (отклонение $< 0.1\%$).
   - 3-й закон Ньютона между `wall_support` (`part_a`) и `cantilever_beam` (`part_b`): $|\mathbf{F}_{\text{beam}} + \mathbf{F}_{\text{wall}}| < 10^{-9}\text{ Н}$, $|\boldsymbol{\tau}_{\text{beam}} + \boldsymbol{\tau}_{\text{wall}}| < 10^{-9}\text{ Н}\cdot\text{м}$.
   - Невязка динамического равновесия близка к нулю ($< 1\%$ от $m g$).

2. **Тест (b) — Вращающийся стержень на шарнире `Revolute` (`test_p3_t6_rotating_rod_centripetal_reaction_force_in_local_frame`)**:
   - Стержень длиной $L = 0.20\text{ м}$ ($r_c = L/2 = 0.10\text{ м}$) в невесомости на шарнире `Revolute` вокруг оси $+Z$ с угловой скоростью $\omega = 10\text{ рад/с}$.
   - В локальной системе координат стержня (даже после поворота на $\sim 2\text{ рад}$ в мировой СК) центростремительная реакция шарнира направлена строго вдоль локальной оси $-X$: $F_x \approx -m \omega^2 r_c$ (погрешность $< 2\%$), момент вокруг свободной оси вращения $Z$ равен $0$, ускорение ЦМ в локальной СК $a_x \approx -\omega^2 r_c$, невязка динамического равновесия мала.

3. **Контрактный MCP-тест (`test_mcp_p3_t6_get_part_load_cases_contract`)**:
   - Проверяет вызов `get_part_load_cases` через JSON-RPC для `"peak_force"`, `"peak_torque"`, числового `time_s = 0.25`, масштабирование координат `anchor_local` и `center_of_mass_local` между `"mm"` и `"m"`, 3-й закон Ньютона и коды ошибок (`PART_NOT_FOUND`, `INVALID_ARGUMENT` с `param: "select"`).
