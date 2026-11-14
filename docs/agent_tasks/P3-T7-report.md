# P3-T7 Report: Квазистатический расчёт прочности по нагрузкам из симуляции (`run_fea_from_simulation`)

## 1. Цель задачи (`fix/P3_calculations.md` §P3-T7)
Реализовать 36-й MCP-инструмент `run_fea_from_simulation(part_id, select, load_radius?, mesh_density?, element_order?, fea_case?, convergence_study?, levels?, max_seconds?)`, обеспечивающий автоматический перенос пиковых (или мгновенных) реакций шарниров и сил инерции Даламбера из физического симулятора Rapier3D (`P3-T6`) на тетраэдральную сетку детали (`TET4` / `TET10`) в МКЭ-решатель (`mcp-cad-fea`):
1. Для каждого шарнира, соединённого с деталью `part_id`, распределить его реакцию $\mathbf{F}_j$ и момент реакции $\boldsymbol{\tau}_j$ по узлам тетраэдральной сетки в зоне крепления (автоматическое сопоставление с именованной гранью `FaceInfo` из `P2-T8`, либо узлы в сфере радиуса `load_radius` вокруг `anchor_local`), так чтобы:
   - $\sum_i \mathbf{f}_i = \mathbf{F}_j$ (точное сохранение главного вектора силы),
   - момент реакции $\boldsymbol{\tau}_j$ прикладывался парой распределённых сил с точным сохранением главного момента относительно якоря шарнира $\sum_i (\mathbf{x}_i - \mathbf{p}_j) \times \mathbf{f}_i = \boldsymbol{\tau}_j$.
2. Если деталь ускоряется или находится в поле тяжести ($\sum \mathbf{F}_{\text{joints}} \ne \mathbf{0}$):
   - добавить объёмные силы инерции Даламбера и гравитации $m(\mathbf{g}_{\text{local}} - \mathbf{a}(\mathbf{x})) = m(\mathbf{g}_{\text{local}} - \mathbf{a}_{\text{COM}} - \boldsymbol{\alpha} \times \mathbf{r}_n - \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r}_n))$ по объёму детали с точной проекцией невязки 6-DOF динамического равновесия ($\sum_n \mathbf{f}_n = \mathbf{0}$, $\sum_n (\mathbf{x}_n - \mathbf{x}_{\text{COM}}) \times \mathbf{f}_n = \mathbf{0}$);
   - закрепить **только 6 степеней свободы** минимальной трёхузловой опорой (**схема 3-2-1**, 3 неколлинеарных узла вне зоны нагружения: `n1` — 3 DOF `All`, `n2` — 2 DOF `YZ/XZ/XY`, `n3` — 1 DOF `X/Y/Z`), чтобы не искажать поле напряжений искусственной жёсткой заделкой границы.
3. Проверить баланс статического равновесия (`P3-T2`, `equilibrium_error_pct < 1%`) и вернуть отчёт FEA с блоком `source: { simulation_time_s, joints: [...] }`.
4. Верифицировать на консольной балке из `P3-T6(a)` ($L = 0.20\text{ м}$, $b = h = 0.02\text{ м}$, сталь $\rho = 7850\text{ кг/м}^3$, $g = 9.81\text{ м/с}^2$) совпадение максимального напряжения у корня с аналитическим $\sigma = \frac{(w L^2 / 2)\cdot(h/2)}{I} = \frac{3 \rho g L^2}{h} \approx 0.462051\text{ МПа}$ в пределах точности `docs/FEA_ACCURACY.md` и `equilibrium_error_pct < 1%`.

---

## 2. Выполненные изменения

### 2.1. `crates/mcp-cad-fea/src/boundary.rs`
- **`match_joint_anchor_to_cad_face(part_mesh, anchor_local_m, load_radius_m)`**:
  - Находит ближайшую именованную топологическую грань (`FaceInfo`, `P2-T8`) к локальным координатам якоря шарнира `anchor_local_m` (учитывая попадание точки на цилиндрическую ось отверстия `FaceKind::Cylinder` или расстояние до треугольников грани в пределах `search_tol`).
  - Если в аргументах явно задан `load_radius`, который меньше характерного радиуса грани, переключается на сферическую окрестность радиуса `load_radius`.
- **`compute_consistent_face_moments` и `distribute_force_and_moment_with_moments`**:
  - Для линейных (`TET4`) и квадратичных (`TET10`, 6-узловой треугольник) поверхностных граней вычисляет точные интегралы функций формы нулевого и первого порядка:
    $$w_i = \int_\Gamma N_i(\mathbf{x})\, dA, \qquad \mathbf{q}_i = \int_\Gamma N_i(\mathbf{x})(\mathbf{x} - \mathbf{c})\, dA.$$
    В частности, для 6-узлового квадратичного треугольника `TET10` угловые узлы имеют нулевой интеграл силы $\int N_a\, dA = 0$, но ненулевой первый момент $\mathbf{q}_a = \frac{A}{60}(2\mathbf{v}_a - \mathbf{v}_b - \mathbf{v}_c)$, а серединные узлы рёбер — $w_m = \frac{A}{3}$ и $\mathbf{q}_m = \frac{A}{15}(2\mathbf{v}_a + 2\mathbf{v}_b + \mathbf{v}_c) - \frac{A}{3}\mathbf{c}$.
  - Собирает согласованный тензор моментов инерции сечения $\mathbf{J} = \sum_i ((\mathbf{r}_i \cdot \mathbf{q}_i)\mathbf{I}_3 - \mathbf{q}_i \mathbf{r}_i^T) = \int_\Gamma ((\mathbf{r}\cdot\mathbf{r})\mathbf{I}_3 - \mathbf{r}\mathbf{r}^T)\, dA$ и распределяет узловые силы $\mathbf{f}_i = \frac{w_i}{A}\mathbf{F}_j + \boldsymbol{\theta} \times \mathbf{q}_i$, где $\boldsymbol{\theta} = \mathbf{J}^+ (\boldsymbol{\tau}_j + (\mathbf{p}_j - \mathbf{c}) \times \mathbf{F}_j)$. Это гарантирует как точное равенство суммы сил $\sum_i \mathbf{f}_i = \mathbf{F}_j$ и моментов $\sum_i (\mathbf{x}_i - \mathbf{p}_j) \times \mathbf{f}_i = \boldsymbol{\tau}_j$, так и точное линейное распределение нормальных напряжений Эйлера–Бернулли $\sigma_x(y) = -\frac{M_z y}{I_z}$ на сетках `TET10`.
- **`select_321_minimal_support_constraints`**:
  - Выбирает 3 неколлинеарных угловых узла `(n1, n2, n3)` на максимальном удалении от нагруженных якорей шарниров и фиксирует ровно $3 + 2 + 1 = 6$ степеней свободы жёсткого тела (`n1`: `DofComponent::All`, `n2`: 2 ортогональные компоненты перпендикулярно линии `p2 - p1`, `n3`: 1 компонента вдоль нормали плоскости `(p1, p2, p3)`).
- **`build_simulation_boundary_conditions`**:
  - Комбинирует реакции всех шарниров, распределяет объёмную силу тяжести и инерции Даламбера $m(\mathbf{g}_{\text{local}} - \mathbf{a}(\mathbf{x}))$ пропорционально узловым объёмам $V_n = \sum_{e \ni n} V_e / 4$, проецирует остаточную невязку дискретизации шага симулятора (`equilibrium_residual`) так, чтобы полный вектор узловых сил удовлетворял $\sum_n \mathbf{f}_n = \mathbf{0}$ и $\sum_n (\mathbf{x}_n - \mathbf{x}_{\text{COM}}) \times \mathbf{f}_n = \mathbf{0}$ с машинной точностью, и прикладывает минимальную опору `3-2-1`.

### 2.2. `crates/mcp-cad-fea/src/result.rs` и `crates/mcp-cad-fea/src/solver.rs`
- Добавлены структуры `FeaJointLoadSource` и `FeaSimulationSource`, а также поле `pub source: Option<FeaSimulationSource>` и конструктор `FeaReport::with_source` в `FeaReport`.
- Добавлена проверка `is_zero_reaction_321_datum` в `FeaReport::compute`: для самоуравновешенной 6-DOF опоры `3-2-1` (с нулевой суммарной реакцией опоры $\|\mathbf{R}_{\text{support}}\| \approx 0$) не выставляется ложное предупреждение о сингулярности жёсткой заделки `CONSTRAINT_SINGULARITY`.
- В `solve_pcg` (`crates/mcp-cad-fea/src/solver.rs:180`) порог преждевременного останова по знаменателю $\mathbf{p}^T A \mathbf{p}$ скорректирован с `1e-25` до `1e-300` (с проверкой достигнутой точности невязки), что устранило ложный `SolverConvergenceFailed` на задачах с малыми центробежными нагрузками в СИ (`< 0.1 Н` при жёсткости стали $K_{ii} \sim 10^{10}\text{ Н/м}$).

### 2.3. `crates/mcp-cad-server/src/tools/{fea.rs, mod.rs}`
- Реализована функция `run_fea_from_simulation(args, state)` в `crates/mcp-cad-server/src/tools/fea.rs`:
  - Извлекает `PartLoadCase` из `state.last_simulation` через `state.physics_world.extract_part_load_case(summary, part_id, selector)` (`"peak_force"`, `"peak_torque"` или `time_s`).
  - Строит тетраэдральную сетку (`TET4` или `TET10`), синтезирует граничные условия через `build_simulation_boundary_conditions`, решает задачу упругости (с опциональным `convergence_study: true`), сохраняет результат в `state.last_fea_reports` и `state.last_fea_cases` (для интеграции с `check_requirements` по `min_safety_factor`) и возвращает полный отчёт МКЭ с блоком `source`.
- Зарегистрирован 36-й инструмент `run_fea_from_simulation` в `list_tools()` и `dispatch_tool()` (`crates/mcp-cad-server/src/tools/mod.rs`).

---

## 3. Верификация и результаты тестов

1. **Консольная балка под собственным весом (`test_p3_t7_run_fea_from_simulation_cantilever_gravity`)**:
   - Параметры (`P3-T6(a)`): $L = 0.20\text{ м}$, $b = h = 0.02\text{ м}$, сталь $\rho = 7850\text{ кг/м}^3$, $E = 205\text{ ГПа}$, $\nu = 0.30$, $g = 9.81\text{ м/с}^2$, шарнир `Fixed` (`weld_root`) в корне $x = 0$.
   - Аналитическое изгибное напряжение у корня:
     $$\sigma_{\text{root}} = \frac{(w L^2 / 2)\cdot (h / 2)}{I} = \frac{3 \rho g L^2}{h} = \frac{3 \cdot 7850 \cdot 9.81 \cdot 0.20^2}{0.02} = 0.462051\text{ МПа}.$$
   - Результат `run_fea_from_simulation` (`mesh_density: "coarse"`, `element_order: 2` / `TET10`):
     - `max_von_mises_mpa = 0.44812 МПа` (относительная погрешность **`3.01%`**, в пределах допуска `docs/FEA_ACCURACY.md`);
     - `diagnostics.equilibrium_error_pct < 1e-6%` (требование `< 1%`);
     - `diagnostics.constrained_rigid_body_modes_ok = true`;
     - `source.joints[0].matched_face = "left"`, `source.support_mode = "3-2-1_minimal_kinematic_6dof"`.
2. **Вращающийся стержень в невесомости с разгрузкой Даламбера (`test_p3_t7_run_fea_from_simulation_rotating_rod_inertial_relief`)**:
   - Стержень $L = 0.20\text{ м}$ с шарниром `Revolute` в $x = 0$ при $\omega = 10\text{ рад/с}$: аналитическое центробежное растягивающее напряжение у корня $\sigma_{\text{root}} = \frac{1}{2}\rho \omega^2 L^2 = 0.01570\text{ МПа}$.
   - Результат `run_fea_from_simulation`: `equilibrium_error_pct < 1%`, `constrained_rigid_body_modes_ok = true`, погрешность пикового напряжения `< 20%`.
3. **Сквозной JSON-RPC контрактный тест (`test_mcp_p3_t7_run_fea_from_simulation_contract`)**:
   - Проверена валидация ошибок (вызов до симуляции, `PART_NOT_FOUND`, отрицательный `load_radius`), автопроверка требования `min_safety_factor` по `fea_case: "sim_gravity_root"`, точность напряжений и структура поля `source`.
