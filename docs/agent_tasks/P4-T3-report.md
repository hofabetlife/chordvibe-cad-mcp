# Отчёт о выполнении задачи P4-T3

- **Задача**: `P4-T3` — Собственный многотельный решатель ограничений и динамики в `f64` (Ньютон–Эйлер + множители Лагранжа, моторы, упоры `limits`, замкнутые контуры и прямое ограничение `Gear`)
- **Статус**: ✅ ВЫПОЛНЕНО (`16.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-physics/src/solver.rs`](../../crates/mcp-cad-physics/src/solver.rs) *(новый модуль собственного многотельного решателя динамики и ограничений в двойной точности `f64`: `MultiBodySolver`, `SolverConfig`, `SolvedJointRecord`, `create_solved_joint_record`, `rotation_vector`; `0` импортов `rapier3d`)*
  - [`crates/mcp-cad-physics/src/joint.rs`](../../crates/mcp-cad-physics/src/joint.rs) *(полное удаление зависимости от `rapier3d::prelude::*` и `GenericJoint`; введение нативных `f64`-структур `JointHandle` и `JointData`, сохранение всех проверок параметров и ошибок `PhysicsError::InvalidJointConfiguration`)*
  - [`crates/mcp-cad-physics/src/simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs) *(удаление `PhysicsPipeline`, `ImpulseJointSet`, `MultibodyJointSet`, `IntegrationParameters`, `IslandManager`, `CCDSolver`, `DefaultBroadPhase`, `NarrowPhase` и пост-шагового хака `GearConstraintRecord` / `set_angvel`; подключение `MultiBodySolver` и `Vec<SolvedJointRecord>`)*
  - [`crates/mcp-cad-physics/src/lib.rs`](../../crates/mcp-cad-physics/src/lib.rs) *(экспорт модуля `solver` и публичных типов `JointData`, `JointHandle`, `MultiBodySolver`, `SolvedJointRecord`, `SolverConfig`)*
  - [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs) *(добавлен интеграционный тест `test_p4_t3_f64_multibody_solver_coupled_gear_torque_limits_and_closed_loop`, актуализированы проверки точности `PhysicsWorld` в `test_p4_t1_dae_f64_candidates_vs_analytical_and_rapier_baseline`)*
  - [`TASKS.md`](../../TASKS.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md)

---

## 1. Контекст и постановка задачи

До выполнения `P4-T3` метод `PhysicsWorld::step` (`crates/mcp-cad-physics/src/simulator.rs`) опирался на игровой конвейер `rapier3d::pipeline::PhysicsPipeline` (`TGS-Soft` в одинарной точности `f32`), `ImpulseJointSet` и `GenericJoint`. Это порождало следующие системные проблемы:
1. **Потеря точности `f32` и мягкость связей `TGS-Soft`**: на свободном падении ошибка позиции составляла $6.67\times 10^{-7}\text{ м}$ вместо машинного нуля `f64` ($< 10^{-15}\text{ м}$), а на вращающемся стержне ($\omega = 10\text{ рад/с}$) угловая скорость дрейфовала на $1.3\times 10^{-2}\text{ рад/с}$ с ошибкой центростремительной реакции $\approx 0.14\%$.
2. **Пост-шаговый кинематический хак для `CadJointType::Gear`**: поскольку `Rapier3D 0.22` не поддерживает зубчатое зацепление в `ImpulseJointSet`, связь `Gear` эмулировалась в конце шага прямым вызовом `body_b.set_angvel(...)`. В результате ведомая шестерня не оказывала обратного инерционного и силового сопротивления ведущему валу, а реакция в `Gear` не участвовала в единой системе уравнений Лагранжа.

В соответствии с архитектурным решением **ADR-004** (`docs/DECISIONS.md`), задача `P4-T3` заменяет шаговый решатель `Rapier3D` на собственный многотельный решатель ограничений и динамики Ньютона–Эйлера в двойной точности `f64` (`crates/mcp-cad-physics/src/solver.rs`).

---

## 2. Математическая модель и архитектура решателя (`crates/mcp-cad-physics/src/solver.rs`)

Решатель `MultiBodySolver` реализует **5-этапную симплектическую схему Индекса-2 с прямым регулярным KKT / активным Box-LCP решателем и проекцией на голономное многообразие** в `f64`:

### Этап 1. Неограниченный предиктор скоростей Ньютона–Эйлера ($\mathbf{v}^*$)
Для каждого динамического тела $i$ с массой $m_i$, мировым тензором инерции $I_{w,i} = R_i I_{\text{loc},i} R_i^T$ и скоростью центра масс $\mathbf{v}_{\text{com},i}$:
$$\mathbf{v}_{\text{com},i}^* = \mathbf{v}_{\text{com},i}^{(k)} + \Delta t\left(\mathbf{g} + m_i^{-1}\mathbf{F}_{\text{ext},i}\right), \qquad \boldsymbol{\omega}_i^* = \boldsymbol{\omega}_i^{(k)} + \Delta t\,I_{w,i}^{-1}\left(\boldsymbol{\tau}_{\text{ext},i} - \boldsymbol{\omega}_i^{(k)} \times (I_{w,i}\boldsymbol{\omega}_i^{(k)})\right).$$

### Этап 2. Сборка и решение связанной системы скоростных ограничений (KKT / Box-LCP)
В единую матрицу Якоби $J(\mathbf{q}_k) \in \mathbb{R}^{m \times 6N}$ собираются все 5 типов шарниров (`Spherical`, `Fixed`, `Revolute`, `Prismatic`, `Gear`), односторонние и двусторонние ограничители хода `limits` (включая нулевой диапазон `[c, c]`), приводы `MotorConfig` (`target_velocity`, `max_force`) и контактные ограничения с кулоновским трением:
- **Прямое связанное ограничение `CadJointType::Gear`**:
  $$J_{\text{gear}}\,\mathbf{v} = (\text{ratio}\cdot \hat{\mathbf{z}}_a)\cdot\boldsymbol{\omega}_a + \hat{\mathbf{z}}_b\cdot\boldsymbol{\omega}_b = -\frac{\beta}{\Delta t}\left(\text{ratio}\cdot\theta_a + \theta_b\right),$$
  что обеспечивает как точное кинематическое отношение угловых скоростей $\omega_b = -\text{ratio}\cdot\omega_a$, так и физически достоверную передачу крутящего момента $\tau_a = \text{ratio}\cdot\tau_b$ через множитель Лагранжа $\lambda_{\text{gear}}$ без пост-шаговых правок скоростей.
- **Приводы `MotorConfig` и упоры `limits`**:
  Моторы формулируются как строки скоростных ограничений с двусторонними ограничениями импульса $\gamma_r \in [-F_{\max}\Delta t, +F_{\max}\Delta t]$, а односторонние упоры — с $\gamma_r \in [0, +\infty)$ на нижней границе $s \le s_{\min}$ и $\gamma_r \in (-\infty, 0]$ на верхней границе $s \ge s_{\max}$.
- **Регуляризованное дополнение Шура (матрица Делассю)**:
  $$\left(J M^{-1} J^T + \text{diag}(\epsilon_r)\right)\boldsymbol{\gamma} = \mathbf{b}_{\text{rhs}} - J\mathbf{v}^*, \qquad \gamma_r \in [\underline{\gamma}_r, \overline{\gamma}_r],$$
  решается точным методом активных ограничений (Active-Set Box-LCP с разложением Холецкого/LU на свободном подмножестве и защитным PGS-полишингом), после чего $\mathbf{v}^{(k+1)} = \mathbf{v}^* + M^{-1} J^T \boldsymbol{\gamma}$.

### Этап 3. Трапецеидальное (Midpoint) интегрирование положения ЦМ и экспоненциальное отображение на $SO(3)$
С учётом произвольного смещения локального центра масс $\mathbf{c}_{\text{loc}}$ относительно начала координат детали:
$$\mathbf{x}_{\text{com},i}^{(k+1)} = \mathbf{x}_{\text{com},i}^{(k)} + \frac{\Delta t}{2}\left(\mathbf{v}_{\text{com},i}^{(k)} + \mathbf{v}_{\text{com},i}^{(k+1)}\right), \qquad q_i^{(k+1)} = \exp\!\left(\frac{\Delta t}{2}\left(\boldsymbol{\omega}_i^{(k)} + \boldsymbol{\omega}_i^{(k+1)}\right)\right) \otimes q_i^{(k)},$$
$$\mathbf{p}_i^{(k+1)} = \mathbf{x}_{\text{com},i}^{(k+1)} - R(q_i^{(k+1)})\,\mathbf{c}_{\text{loc},i}.$$

### Этап 4. Проекция на голономное многообразие связей $\Phi(\mathbf{q}_{k+1}) = \mathbf{0}$
До 6 итераций взвешенного по массе метода Гаусса–Ньютона устраняют позиционный и угловой дрейф всех голономных связей (включая замкнутые кинематические контуры 4-звенников и кривошипно-ползунных механизмов) до допуска $\|\Phi(\mathbf{q}_{k+1})\|_\infty \le 10^{-12}$. Для тел с единственным поступательным сочленением коррекция якоря переносит $\mathbf{x}_{\text{com}}$ напрямую без возмущения свободной угловой координаты шарнира.

### Этап 5. Проекция скоростей на касательное подпространство и извлечение точных реакций $\boldsymbol{\lambda}_{k+1}$
В финальной точке $(\mathbf{q}_{k+1}, \mathbf{v}_{k+1})$ скорости проецируются на касательное многообразие $J(\mathbf{q}_{k+1})\mathbf{v}_{k+1} = \mathbf{b}_{\text{motor}}$, после чего вычисляются мгновенные множители Лагранжа $\boldsymbol{\lambda}_{k+1} = \boldsymbol{\gamma}_{\text{react}} / \Delta t$ и приводятся к точке якоря `anchor_a`:
$$\mathbf{F}_A = \sum_r \mathbf{j}_{\text{lin},a}^{(r)} \lambda_r, \qquad \boldsymbol{\tau}_{A,\text{anchor}} = \sum_r \mathbf{j}_{\text{ang},a}^{(r)} \lambda_r - (\mathbf{p}_{\text{anchor},a} - \mathbf{x}_{\text{com},a}) \times \mathbf{F}_A.$$
Дополнительно проверяется конечность и ограниченность норм всех состояний ($\|\mathbf{p}\|, \|\mathbf{v}\|, \|\boldsymbol{\omega}\| < 10^6$), формирующая флаг сходимости `SimulationSummary.success`.

---

## 3. Результаты верификации точности

1. **Свободное падение под действием гравитации (`y0 = 10.0 м`, `g = -9.81 м/с²`, `T = 1.0 с`, `dt = 5 мс`)**:
   - Ошибка позиции `PhysicsWorld`: **$8.882 \times 10^{-16}\text{ м}$** (было $6.670 \times 10^{-7}\text{ м}$ на `Rapier3D f32` — улучшение в **$7.5 \times 10^8$ раз**);
   - Ошибка скорости `PhysicsWorld`: **$8.882 \times 10^{-15}\text{ м/с}$**.
2. **Вращающийся стержень (`L = 0.20 м`, $\omega_z = 10\text{ рад/с}$, аналитическая центростремительная реакция $F_x = -6.280000\text{ Н}$)**:
   - Ошибка центростремительной реакции в `PhysicsWorld::extract_part_load_case`: **$2.084 \times 10^{-10}\%$** (было $1.375 \times 10^{-1}\%$ на `Rapier3D f32` — улучшение на **9 порядков**);
   - Дрейф угловой скорости за $0.20\text{ с}$: **$1.169 \times 10^{-12}\text{ рад/с}$**.
3. **Связанная зубчатая передача `CadJointType::Gear` под внешней нагрузкой (`test_p4_t3_f64_multibody_solver_coupled_gear_torque_limits_and_closed_loop`)**:
   - При ведущем моторе $\omega_a = 6.0\text{ рад/с}$, передаточном отношении $\text{ratio} = 0.5$ и нагрузочном тормозном моменте $4.0\text{ Н}\cdot\text{м}$ на ведомом валу:
     - Угловая скорость ведомого вала: $\omega_b = -3.000000000\text{ рад/с}$ (погрешность $< 10^{-9}\text{ рад/с}$);
     - Реактивный крутящий момент, переданный через `Gear` на ведущий вал: $|\tau_{a,\text{gear}}| = 2.000000\text{ Н}\cdot\text{м}$ (погрешность $< 10^{-6}\text{ Н}\cdot\text{м}$).
4. **Замкнутый кривошипно-ползунный контур (`R = 0.03 м`, `L = 0.05 м`, 200 шагов)**:
   - Поперечный уход ползуна: $|y|, |z| < 10^{-11}\text{ м}$;
   - Совпадение продольной координаты $x(\theta)$ с точной аналитикой $R\cos\theta + \sqrt{L^2 - R^2\sin^2\theta}$: погрешность $< 10^{-4}\text{ м}$.

---

## 4. Обнаруженные смежные проблемы

Новых проблем, выходящих за рамки уже запланированных в бэклоге задач `P4-T4`–`P4-T9` (включая удаление неиспользуемой зависимости `rapier3d = "0.22"` из `Cargo.toml` в `P4-T6`), не выявлено.

---

## 5. Верификация

1. `cargo test -p mcp-cad-physics --no-fail-fast`: **33/33 PASS** (`7/7` unit-тестов в `--lib` + `26/26` интеграционных тестов в `tests/physics_tests.rs`, включая новый `test_p4_t3_f64_multibody_solver_coupled_gear_torque_limits_and_closed_loop`, `0 failed`).
2. `cargo test -p mcp-cad-server --test gear_kinematic_test --no-fail-fast`: **1/1 PASS** (`test_gear_kinematic_simulation_and_preview`).
3. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
4. `npm run lint` (`tsc --noEmit`): **PASS** (`0` ошибок).
5. `npm run build` (`vite build`): **PASS**.
6. `npm test`: **52/52 PASS**.
