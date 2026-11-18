# Отчёт о выполнении задачи P4-T7

- **Задача**: `P4-T7` — Верификация нового `f64`-решателя (`physics_tests.rs`, `gear_kinematic_test.rs`, `mcp_contract_tests.rs`, CI) и финальное обновление всей документации
- **Статус**: ✅ ВЫПОЛНЕНО (`18.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-physics/src/solver.rs`](../../crates/mcp-cad-physics/src/solver.rs) *(замена явного Эйлерова предиктора угловой скорости в `MultiBodySolver::step` на неявный среднеточечный Lie–Poisson Ньютоновский предиктор `predict_angular_velocity_implicit_midpoint` в локальной СК тела и перенос угловой скорости $\boldsymbol{\Omega}_{\text{next}}$ в СК тела при $SO(3)$-экспоненциальном обновлении ориентации)*
  - [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs) *(добавлен комплексный эталонный тест `test_p4_t7_f64_solver_comprehensive_benchmarks_euler_top_10rev_gear_4bar_friction_and_fea`)*
  - [`crates/mcp-cad-server/tests/gear_kinematic_test.rs`](../../crates/mcp-cad-server/tests/gear_kinematic_test.rs) *(добавлен 10-оборотный интеграционный тест `test_gear_kinematic_10_revolutions_and_load_cases` через `apply_recipe("gear_transmission")`, `run_physics_simulation` и `get_part_load_cases`)*
  - [`crates/mcp-cad-server/tests/mcp_contract_tests.rs`](../../crates/mcp-cad-server/tests/mcp_contract_tests.rs) *(добавлен сквозной JSON-RPC E2E-тест `test_mcp_p4_t7_f64_dae_end_to_end_mechanisms_friction_and_fea_coupling`: `apply_recipe` $\to$ `run_physics_simulation` $\to$ `get_part_load_cases` $\to$ `run_fea_from_simulation`)*
  - [`PRD.md`](../../PRD.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md), [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md), [`docs/PROGRESS.md`](../../docs/PROGRESS.md), [`docs/MCP_API_REFERENCE.md`](../../docs/MCP_API_REFERENCE.md), [`docs/ANTIGRAVITY_SETUP.md`](../../docs/ANTIGRAVITY_SETUP.md), [`README.md`](../../README.md), [`CHANGELOG.md`](../../CHANGELOG.md), [`TASKS.md`](../../TASKS.md) *(синхронизация всей документации с `f64` Index-2 DAE решателем после прохождения всех проверок)*

---

## 1. Доработка численного интегрирования в `solver.rs`

При верификации сохранения кинетической энергии вращения $T_{\text{rot}} = \frac{1}{2}\boldsymbol{\omega}\cdot(I_w\boldsymbol{\omega})$ и нормы момента импульса $\|\mathbf{L}\| = \|I_w\boldsymbol{\omega}\|$ на свободном 3D асимметричном волчке Эйлера ($I_x \ne I_y \ne I_z$) выявлен дрейф порядка $O(\Delta t^2)$ из-за явного предиктора Эйлера и фиксации мирового вектора $\boldsymbol{\omega}_{\text{next}}$ при вращении $q_k \to q_{k+1}$.

Реализовано в [`crates/mcp-cad-physics/src/solver.rs`](../../crates/mcp-cad-physics/src/solver.rs):
1. `predict_angular_velocity_implicit_midpoint`: решает систему Эйлера в локальной системе координат тела методом Ньютона (до 5 итераций, критерий $\|\mathbf{r}\|^2 \le 10^{-30}$):
   $$I_b(\boldsymbol{\Omega}^* - \boldsymbol{\Omega}_k) + \Delta t\left(\boldsymbol{\Omega}_{\text{mid}} \times (I_b\boldsymbol{\Omega}_{\text{mid}}) - \boldsymbol{\tau}_{\text{ext},b}\right) = \mathbf{0}, \qquad \boldsymbol{\Omega}_{\text{mid}} = \frac{\boldsymbol{\Omega}_k + \boldsymbol{\Omega}^*}{2}.$$
2. В шаге обновления позы на $SO(3)$ вектор $\boldsymbol{\Omega}_{\text{next}}$ сначала переносится в локальную СК тела до поворота (`q_old.inverse() * w_next`) и возвращается в мировую СК уже в новой ориентации (`q_new * w_body_next`).

Для свободного волчка без внешних моментов оба инварианта ($T_{\text{rot}}$ и $\|\mathbf{L}\|$) теперь сохраняются за 500 шагов с относительной точностью **`< 1e-13`** (машинная точность `f64`), а дрейф 3D-вектора мирового момента импульса $\mathbf{L}_w$ за $T = 1.0\text{ с}$ не превышает `< 1e-4`.

---

## 2. Расширенные эталонные и E2E-тесты

1. **`test_p4_t7_f64_solver_comprehensive_benchmarks_euler_top_10rev_gear_4bar_friction_and_fea`** ([`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs)):
   - Свободное падение в `f64` ($T = 1.0\text{ с}$, $\Delta t = 5\text{ мс}$): ошибка положения и скорости **`< 1e-12`**.
   - Свободный 3D асимметричный волчок Эйлера ($0.30\times 0.20\times 0.10\text{ м}$, $\boldsymbol{\omega}_0 = [2.5, 1.5, 6.0]\text{ рад/с}$, $500$ шагов): сохранение $T_{\text{rot}}$ и $\|\mathbf{L}\|$ с относительной погрешностью **`< 1e-13`**, дрейф вектора $\mathbf{L}_w$ **`< 1e-4`**.
   - 10-оборотная зубчатая пара в `PhysicsWorld` ($\omega_a = 20\text{ рад/с}$, $T = \pi\text{ с}$, $\text{ratio} = 0.5$, демпфирующая нагрузка $4.0\text{ Н}\cdot\text{м}$ на ведомой шестерне): невязка скорости $|\omega_b + 0.5\omega_a| < 10^{-11}$, накопленные углы $20\pi$ и $-10\pi\text{ рад}$ ($< 10^{-9}$), переданный реактивный момент в зубчатом зацеплении и приводе `2.0 Н·м` ($< 10^{-7}\text{ Н}\cdot\text{м}$).
   - Замкнутый кривошипно-ползунный механизм с последующим извлечением динамических нагрузок `extract_part_load_case(AtTime(0.20))`: невязка равновесия сил $< 5\cdot 10^{-3}\text{ Н}$, моментов $< 10^{-3}\text{ Н}\cdot\text{м}$.
2. **`test_gear_kinematic_10_revolutions_and_load_cases`** ([`crates/mcp-cad-server/tests/gear_kinematic_test.rs`](../../crates/mcp-cad-server/tests/gear_kinematic_test.rs)):
   - E2E-проверка 10 полных оборотов редуктора из `apply_recipe("gear_transmission")` + `run_physics_simulation` ($\pi\text{ s}$) + `get_part_load_cases(select="peak_force")`.
3. **`test_mcp_p4_t7_f64_dae_end_to_end_mechanisms_friction_and_fea_coupling`** ([`crates/mcp-cad-server/tests/mcp_contract_tests.rs`](../../crates/mcp-cad-server/tests/mcp_contract_tests.rs)):
   - Полный цикл по протоколу MCP JSON-RPC: `apply_recipe` $\to$ `run_physics_simulation` $\to$ `get_part_load_cases` $\to$ `run_fea_from_simulation` (вычисление `max_von_mises_mpa`, напряжений, реакций и статуса валидации).

---

## 3. Полная верификация рабочего пространства и обновление документации

Перед обновлением документации выполнен полный зелёный прогон всего рабочего пространства:
- `cargo test --workspace --no-fail-fast` (`WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 MCP_CAD_DISABLE_GUI_SPAWN=1`): **344/344 PASS** (`0 failed`):
  - `mcp-cad-core`: **131/131** (`74` `--lib` в `20.06s`, `10` `csg_invariants` в `27.12s`, `16` `dag_tests` в `5.86s`, `25` `primitive_invariants` в `7.15s`, `6` `tessellation_tests` в `0.03s`);
  - `mcp-cad-fea`: **21/21** (`8` `--lib` в `0.00s`, `2` `fea_benchmarks` в `125.71s`, `11` `fea_tests` в `52.49s`);
  - `mcp-cad-physics`: **40/40** (`8` `--lib` в `2.17s`, `32` `physics_tests` в `10.04s`);
  - `mcp-cad-viewer`: **10/10** (`viewer_tests` в `12.53s`);
  - `mcp-cad-server`: **142/142** (`65` `--lib` в `77.79s`, `1` `bench_suite` в `8.48s`, `2` `gear_kinematic_test` в `0.95s`, `61` `mcp_contract_tests` в `47.48s`, `13` `qa_wide_scan` в `9.00s`).
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
- `npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`): **PASS** (`0` ошибок).
- `npm test` (`src/cad/cad_engine.test.ts` + `src/cad/__tests__/analyze_mechanism.test.ts`): **53/53 PASS** (`38/38` + `15/15`, включая контракт `BUG-021`, `0 failed`).
- После финальной правки документации повторно подтверждено отсутствие устаревших упоминаний `Rapier3D` в актуальной документации и повторно проверены `npm test` (включая `BUG-021`) и `cargo clippy`.
