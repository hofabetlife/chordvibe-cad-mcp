# Отчёт о выполнении задачи P4-T1

- **Задача**: `P4-T1` — Архитектурное решение `ADR-004` (`docs/DECISIONS.md`) и прототип собственного многотельного решателя DAE/LCP в `f64` против аналитики **[DECISION]**
- **Статус**: ✅ ВЫПОЛНЕНО (`15.11.2026` / `16.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-physics/src/dae_prototype.rs`](../../crates/mcp-cad-physics/src/dae_prototype.rs) *(новый модуль прототипа и 4 аналитических бенчмарков в `f64`)*
  - [`crates/mcp-cad-physics/src/lib.rs`](../../crates/mcp-cad-physics/src/lib.rs) *(экспорт `pub mod dae_prototype`)*
  - [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs) *(интеграционный сравнительный тест `test_p4_t1_dae_f64_candidates_vs_analytical_and_rapier_baseline`)*
  - [`docs/DECISIONS.md`](../DECISIONS.md) *(раздел `ADR-004`: математическая модель, структуры данных и таблица бенчмарков)*
  - [`TASKS.md`](../../TASKS.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md)

---

## 1. Контекст и постановка задачи

В фазах `P0`–`P3` подсистема динамики механизмов `crates/mcp-cad-physics` опиралась на игровой физический движок `rapier3d = "0.22"` (`f32`, импульсный решатель `TGS-Soft`). В ходе реализации инженерных расчётов (`P3-T6` `get_part_load_cases`, `P3-T7` `run_fea_from_simulation`, `P3-T9` `analyze_mechanism`, `P3-T16` `sweep_joint`) выявлены пять системных ограничений `Rapier3D`:

1. **Одинарная точность `f32`** ($\epsilon_{\text{mach}} \approx 1.19 \times 10^{-7}$): потеря точности на малых зазорах и больших перепадах масс, необходимость дублирования `gravity_f64` (`P2-T36`).
2. **Податливость и дрейф связей в `TGS-Soft`**: на замкнутых планарных механизмах с пространственно избыточными связями (четырёхзвенник, кривошипно-ползунный механизм: $\text{DOF}_{\text{grubler}} = -2$, $\text{DOF}_{\text{numeric}} = 1$, $3$ избыточные связи) последовательные итерации релаксации дают погрешность траектории ползуна $\sim 4 \times 10^{-4}\text{ м}$.
3. **Сложное декодирование реакций шарниров**: в `simulator.rs` потребовался 280-строчный декодер `compute_joint_world_wrench_on_part_a`, обращающий внутреннюю ортогонализацию Грама–Шмидта в пространстве обратной инерции `Rapier3D` и перестановку `body1 <-> body2` для неподвижных опор.
4. **Отсутствие прямого ограничения зубчатой передачи (`CadJointType::Gear`)**: в `Rapier3D 0.22` нет совместного скоростного/силового ограничения зубчатого зацепления. Временная коррекция `rb_b.set_angvel(-ratio * w_a)` после шага решателя не передаёт крутящий момент нагрузки с ведомого вала на ведущий ($\tau_{\text{motor},1} = 0$ при ненулевом $\tau_{\text{load},2}$).
5. **Огрубление геометрии через `ConvexHull`**: игнорирование сквозных отверстий и полостей детали при расчёте тензора инерции.

Целью `P4-T1` являлась разработка прототипа собственного многотельного решателя DAE/LCP в двойной точности `f64` на базе `nalgebra`, объективное сравнение двух архитектурных кандидатов с аналитическими решениями и базовой линией `Rapier3D 0.22` на 4 эталонных задачах и фиксация выбранной архитектуры в **ADR-004** (`docs/DECISIONS.md`).

---

## 2. Реализованные архитектурные кандидаты (`crates/mcp-cad-physics/src/dae_prototype.rs`)

В модуле `dae_prototype.rs` реализованы структуры `DaeBody`, `DaeJoint`, `DaeWorld` и два кандидата решателя `DaeSolverCandidate`:

### Кандидат A (Выбран в ADR-004): `DirectKktIndex2Projection`
1. **Предиктор свободных скоростей Ньютона–Эйлера с гироскопическим моментом**:
   $$\mathbf{v}^* = \mathbf{v}_k + M^{-1}\left(\mathbf{f}_{\text{ext}} - \begin{bmatrix} \mathbf{0} \\ \boldsymbol{\omega}_k \times (I_w \boldsymbol{\omega}_k) \end{bmatrix}\right)\Delta t.$$
2. **Сборка глобального якобиана $J(\mathbf{q}_k)$ и центростремительной кривизны $-\dot{J}\mathbf{v}_k$**:
   - Используется единая параметризация строк связей с `CadJointType::fill_jacobian_rows` (`P3-T9`): `Fixed` (6 строк), `Revolute` (5 + упор/мотор), `Prismatic` (5 + упор/мотор), `Spherical` (3) и **прямая строка зубчатого зацепления `Gear` (1 строка)**:
     $$r\,\hat{\mathbf{z}}_a^T \boldsymbol{\omega}_a + \hat{\mathbf{z}}_b^T \boldsymbol{\omega}_b = 0.$$
   - Для трансляционных строк шарниров вычисляется точное нормальное ускорение кривизны:
     $$(\dot{J}\mathbf{v})_{\text{lin}} = \boldsymbol{\omega}_a \times (\boldsymbol{\omega}_a \times \mathbf{r}_a) - \boldsymbol{\omega}_b \times (\boldsymbol{\omega}_b \times \mathbf{r}_b), \quad \mathbf{b}_{\text{rhs}} = -\dot{J}\mathbf{v}_k\,\Delta t,$$
     что обеспечивает мгновенное и безошибочное воспроизведение центростремительных реакций $F_r = -m\omega^2 r_c$.
3. **Прямой регуляризованный KKT / Active-Set Box-LCP решатель (`solve_box_constrained_kkt`)**:
   - Формируется матрица Делассю (дополнение Шура) с диагонально-пропорциональной регуляризацией Тихонова / CFM для устойчивости на переопределённых плоских контурах:
     $$A_{ii} \leftarrow A_{ii} + \epsilon_{\text{CFM}}\max(|A_{ii}|, 1), \quad \epsilon_{\text{CFM}} = 10^{-12}.$$
   - СЛАУ $A\,\boldsymbol{\Delta\Lambda} = \mathbf{b}_{\text{rhs}} - J\mathbf{v}^*$ решается напрямую разложением Холецкого ($LL^T$, с резервом через LU с полным выбором ведущего элемента), а при выходе на односторонние упоры `limits` или насыщении момента мотора `max_force` выполняется проекция активного множества (Active-Set Box-LCP).
4. **Трапецеидальное (Midpoint) интегрирование позы 2-го порядка**:
   $$\mathbf{v}_{\text{mid}} = \frac{1}{2}(\mathbf{v}_k + \mathbf{v}_{k+1}), \quad \mathbf{x}_{\text{com},k+1} = \mathbf{x}_{\text{com},k} + \mathbf{v}_{\text{lin,mid}}\Delta t, \quad \mathbf{q}_{k+1} = \exp\left(\frac{1}{2}\boldsymbol{\omega}_{\text{mid}}\Delta t\right)\otimes \mathbf{q}_k.$$
5. **Нелинейная проекция координат на многообразие связей $\Phi(\mathbf{q}) = \mathbf{0}$ (`project_holonomic_manifold`) и касательного пространства скоростей (`project_velocity_and_compute_reactions`)**:
   - Итерации Гаусса–Ньютона в координатах центров масс через разложение Холецкого / LU ($J_{\text{com}}(\mathbf{q})\,\Delta\mathbf{q}_{\text{com}} = -\Phi(\mathbf{q})$) устраняют позиционный дрейф замкнутых цепей и фазы зубчатых колёс до $\|\Phi(\mathbf{q})\|_\infty \le 5.20 \times 10^{-16}\text{ м/рад}$.
   - Касательная проекция скоростей $J(\mathbf{q}_{k+1})\mathbf{v}_{k+1} = \mathbf{v}_{\text{target}}$ и мгновенное решение уравнения Лагранжа на уровне ускорений в обновлённой позе $\mathbf{q}_{k+1}$ дают аналитически точные реакции шарниров $\boldsymbol{\lambda}_{k+1}$.
   - Для извлечения вектора угловой невязки из матрицы относительного поворота $R_{\text{err}} = R_b R_a^T$ реализована `NaN`-безопасная формула через `atan2(sin_t, cos_t)` с предварительным `clamp(-1.0, 1.0)` (взамен `Rotation3::from_matrix_unchecked(r).scaled_axis()`, склонного к `acos(1.0 + eps) = NaN` при $\text{tr}(R_{\text{err}}) > 3$ на 1 ULP).

### Кандидат B: `SemiImplicitEulerPgsBaumgarte`
- Полунеявный (симплектический) метод Эйлера 1-го порядка в сочетании с 60 итерациями Projected Gauss–Seidel (PGS) и стабилизацией Баумгарте ($\beta = 0.20$) в `f64`.

---

## 3. Сводная таблица результатов на 4 аналитических эталонах

Все четыре эталона проверяются в модульных тестах `dae_prototype::tests` и интеграционном тесте `test_p4_t1_dae_f64_candidates_vs_analytical_and_rapier_baseline`:

| Эталонная задача и аналитическое решение | Метрика точности | **Кандидат A: `DirectKktIndex2` (`f64`) [ВЫБРАН]** | Кандидат B: `PgsBaumgarte` (`f64`, 60 итер.) | Базовая линия: `Rapier3D 0.22` (`f32`, `TGS-Soft`) |
|---|---|---|---|---|
| **1. Свободное падение** ($y_0 = 10\text{ м}$, $g = -9.81\text{ м/с}^2$, $T = 1.0\text{ с}$, $\Delta t = 5\text{ мс}$)<br>Аналитика: $y(1) = 5.095\text{ м}$, $v_y(1) = -9.81\text{ м/с}$ | Ошибка позиции $|y - y_{\text{exact}}|$<br>Ошибка скорости $|v_y - v_{y,\text{exact}}|$<br>Время расчёта (200 шагов) | **`8.882e-16 м`**<br>**`8.882e-15 м/с`**<br>`385.2 мкс` | `2.453e-2 м` (`24.53 мм`)<br>`8.882e-15 м/с`<br>`275.3 мкс` | `6.118e-3 м` (`6.12 мм`)<br>`6.252e-5 м/с` |
| **2. Вращающийся стержень** ($L = 0.20\text{ м}$, $r_c = 0.10\text{ м}$, $m = 0.628\text{ кг}$, $\omega_0 = 10\text{ рад/с}$, $t = 0.20\text{ с}$)<br>Аналитика: $F_x^{\text{loc}} = -6.2800\text{ Н}$, $\Delta\omega = 0$ | Отн. ошибка реакции $F_x^{\text{loc}}$ (%)<br>Дрейф скорости $|\omega_z - \omega_0|$<br>Дрейф опоры $\|\mathbf{p}_{\text{pivot}}\|$ | **`2.084e-10 %`** (`-6.280000 Н`)<br>**`1.169e-12 рад/с`**<br>**`5.999e-15 м`** | `7.871e-1 %` (`-6.329433 Н`)<br>`6.171e-2 рад/с`<br>`5.736e-4 м` | `1.523 %` (`-6.375637 Н`)<br>`7.664e-2 рад/с`<br>`2.189e-4 м` |
| **3. Кривошипно-ползунный 4-звенник** ($R = 0.03\text{ м}$, $L = 0.05\text{ м}$, $\omega = 2\pi\text{ рад/с}$, $500$ шагов, 3 избыточные связи)<br>Аналитика: $x_C(\theta) = R\cos\theta + \sqrt{L^2 - R^2\sin^2\theta}$ | Макс. ошибка ползуна $|x_C - x_{\text{exact}}|$<br>RMS ошибка ползуна<br>Макс. невязка контура $\|\Phi(\mathbf{q})\|_\infty$ | **`2.422e-15 м`**<br>**`1.081e-15 м`**<br>**`5.204e-16 м/рад`** | `1.720e-5 м`<br>`1.209e-5 м`<br>`1.771e-4 м/рад` | `~4.2e-4 м`<br>`~2.9e-4 м`<br>`~3.1e-4 м/рад` |
| **4. Зубчатая пара — 10 оборотов под нагрузкой** ($z_1=12, z_2=24, i=0.5$, $\omega_1 = 20\pi\text{ рад/с}$, $\tau_{\text{load},2} = -4.0\text{ Н}\cdot\text{м}$)<br>Аналитика: $\Delta\phi = 0$, $\tau_{\text{motor},1} = +2.0\text{ Н}\cdot\text{м}$ | Ошибка фазы $|\theta_2 + i\theta_1|$ за 10 об.<br>Ошибка передаточного числа (%)<br>Переданный момент $\tau_{\text{motor},1}$ | **`0.000e0 рад`**<br>**`0.000e0 %`**<br>**`2.000000 Н·м` (`1.000e-9 %`)** | `0.000e0 рад`<br>`0.000e0 %`<br>`2.000000 Н·м` (`0.000e0 %`) | Прямая передача момента **не поддерживается** (`0.0 Н·м`, пост-шаговый `set_angvel`) |

---

## 4. Обнаруженные смежные проблемы (занесены в `TASKS.md`)

- **`P4-T8` (`simulator.rs:3645`, `ClosedLoopSolver::compute_closure_residual`)**: вызов `nalgebra::Rotation3::from_matrix_unchecked(r).scaled_axis()` без предварительного зажима следа матрицы $\frac{\text{tr}(R) - 1}{2} \in [-1, 1]$ может вернуть `Vector3::new(NaN, NaN, NaN)` при накоплении ошибки округления `f64` в 1 ULP ($\text{tr}(R) > 3.0$), что приводит к деградации или зависанию `SVD` в `ClosedLoopSolver`. В `dae_prototype.rs` внедрена безопасная функция `rotation_vector(r)` на базе `atan2`; перенос этого исправления в основной модуль `simulator.rs` занесён в `TASKS.md` (`P4-T8`).

---

## 5. Верификация

1. `cargo test -p mcp-cad-physics --no-fail-fast`: **28/28 PASS** (`4/4` unit-тестов в `dae_prototype::tests` + `24/24` интеграционных тестов в `tests/physics_tests.rs`, `0 failed`).
2. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
3. `npm run lint` (`tsc --noEmit`): **PASS** (`0` ошибок).
4. `npm run build` (`vite build`): **PASS**.
5. `npm test`: **52/52 PASS**.
