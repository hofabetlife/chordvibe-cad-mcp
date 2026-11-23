# Отчёт по задаче P5-T3: Нелинейный упруго-пластический решатель МКЭ (`J2` фон Мизеса, `Radial Return Mapping`, остаточная деформация и Co-Rotated FEM)

- **Дата:** 19.11.2026
- **Задача:** `P5-T3 [L]` — *Нелинейный упруго-пластический решатель МКЭ (`J2` фон Мизеса, `Radial Return Mapping`, изотропное упрочнение, остаточная деформация и Co-Rotated FEM для больших поворотов)*
- **Статус:** ✅ **ВЫПОЛНЕНО**

---

## 1. Резюме изменений

1. **Конститутивная модель $J_2$-пластичности и состояние точек Гаусса (`crates/mcp-cad-fea/src/elasticity.rs`, `#![deny(unsafe_code)]`):**
   - Реализованы законы изотропного упрочнения `HardeningLaw`:
     - `Linear { modulus_pa }` ($\sigma_y(\alpha) = \sigma_{y0} + H \alpha$, с поддержкой идеальной пластичности $H = 0$);
     - `PowerLaw { k_pa, n }` ($\sigma_y(\alpha) = \sigma_{y0} + K \alpha^n$);
     - `Saturation { modulus_pa, saturation_stress_pa }` (экспоненциальное насыщение Воса $\sigma_y(\alpha) = \sigma_{y0} + (\sigma_{\infty} - \sigma_{y0})(1 - e^{-\delta \alpha})$ с начальным наклоном $H$).
   - Добавлены структуры `J2MaterialParams` (с автопостроением из `mcp_cad_core::Material`), `GaussPointState` (хранение пластического тензора деформаций $\boldsymbol{\varepsilon}^p$, накопленной эквивалентной пластической деформации $\bar{\varepsilon}_p$, внутренней переменной упрочнения $\alpha$, напряжения Коши $\boldsymbol{\sigma}$ в ко-ротированной системе и плотности пластической диссипации $w_p$) и `ReturnMappingResult`.
   - Реализован алгоритм радиального возврата `radial_return_j2`:
     - точная замкнутая формула для линейного упрочнения и защищённый метод Ньютона–бисекции для нелинейных законов (`PowerLaw`, `Saturation`);
     - вычисление точного симметричного согласованного алгоритмического касательного модуля $\mathbf{C}^{\text{ep}}$ для квадратичной сходимости метода Ньютона–Рафсона;
     - проверка согласованности $\mathbf{C}^{\text{ep}}$ против центральных конечных разностей (точность $< 10^{-6}$ для всех законов упрочнения).

2. **Ко-ротационная формулировка Co-Rotated FEM для `TET4` и `TET10` (`crates/mcp-cad-fea/src/element.rs`):**
   - Реализовано полярное разложение градиента деформации $\mathbf{F} = \mathbf{R}\mathbf{U}$ на группе собственных вращений $SO(3)$ (`polar_decomposition_so3` через SVD с контролем $\det \mathbf{R} = +1$), полностью исключающее паразитные напряжения при произвольных конечных жёстких поворотах на $360^\circ$.
   - Реализованы вращение симметричных тензоров в представлении Фойгта (`rotate_voigt_stress`, `rotate_voigt_strain`), ко-ротированная деформация Био (`corotated_biot_strain_voigt`: $\boldsymbol{\varepsilon}_{\text{rot}} = \mathbf{R}^T \mathbf{F} - \mathbf{I}$) и нелинейная поэлементная сборка `evaluate_nonlinear` / `evaluate_nonlinear_forces_only` для `Tet4Element` (1 точка Гаусса) и `Tet10Element` (4 точки Гаусса 2-го порядка), включающая материальную и геометрическую матрицы касательной жёсткости ($\mathbf{K}_T = \mathbf{K}_{\text{mat}} + \mathbf{K}_{\text{geo}}$).
   - Добавлена экстраполяция пластических переменных из точек Гаусса в 10 узлов элемента (`Tet10Element::extrapolate_gauss_vectors_to_nodes`).
   - Выполнена верификация элементной касательной матрицы против конечных разностей в режиме совместного большого поворота ($40^\circ$) и пластического течения.

3. **Нелинейный инкрементальный решатель Ньютона–Рафсона с адаптивным дроблением шага (`crates/mcp-cad-fea/src/solver.rs`):**
   - Реализованы `NonlinearSolverOptions`, `NonlinearState` (включая `prev_internal_forces` для плавного снятия реакций при переходе на стадию свободной разгрузки `springback` со снятием ограничений Дирихле), `StepHistoryPoint`, `ResolvedLoadStage`, `NonlinearSolveResult`.
   - Реализованы многостадийные и одностадийные решатели `solve_nonlinear_history_tet4`, `solve_nonlinear_history_tet10`, `solve_nonlinear_tet4`, `solve_nonlinear_tet10`, а также расчёт нелинейных узловых реакций `compute_nonlinear_nodal_reactions_tet4` / `compute_nonlinear_nodal_reactions_tet10`.
   - Реализованы:
     - касательный предиктор для предписанных смещений Дирихле;
     - backtracking line search по невязке внутренних и внешних сил;
     - автоматический fallback с полной касательной матрицы $\mathbf{K}_{\text{mat}} + \mathbf{K}_{\text{geo}}$ на $\mathbf{K}_{\text{mat}}$ и диагональную регуляризацию Левенберга–Марквардта для идеального пластического шарнира ($H = 0$);
     - адаптивное разбиение шага нагружения (бисекция `sub-incrementation`) при отказе сходимости Ньютона–Рафсона с контролируемым ростом шага после лёгкой сходимости.

4. **Постобработка результатов и публичный API (`crates/mcp-cad-fea/src/result.rs`, `crates/mcp-cad-fea/src/lib.rs`):**
   - В `FeaReport` добавлено опциональное поле `nonlinear: Option<NonlinearFeaSummary>` (`skip_serializing_if = "Option::is_none"` — 100% обратная совместимость линейных отчётов).
   - Добавлены `NonlinearFeaSummary`, `NonlinearFeaReport`, `compute_nonlinear_fea_results_tet4`, `compute_nonlinear_fea_results_tet10`, `NonlinearLoadStage` и высокоуровневые функции `analyze_mesh_nonlinear`, `analyze_mesh_nonlinear_with_params`, `analyze_mesh_nonlinear_history`, `analyze_part_nonlinear`.

---

## 2. Измеренные результаты эталонных тестов (`nonlinear_elastoplastic_tests.rs`)

Проверено командой `cargo test -p mcp-cad-fea --lib --test nonlinear_elastoplastic_tests -- --nocapture` (после всех финальных правок):

| № | Эталонный тест | Тип элемента | Измеренный результат | Аналитика / критерий приёмки | Погрешность | Статус |
|---|---|---|---|---|---|---|
| **1a** | **Жёсткий поворот (прямая проверка $90^\circ_Z$, $180^\circ_Y$, $180^\circ_{\text{diag}}$)** | `TET4` и `TET10` | `max_sigma = 0.000e0 Pa`, `eq_ep = 0.000e0`, `W_e = 0.000e0 J` | $\sigma_{\max} < 10^{-9}\text{ Па}$ | **`0.000e0 Pa`** | ✅ PASS |
| **1b** | **Краевая задача жёсткого поворота $90^\circ$ и $180^\circ$ со свободными внутренними узлами** | `TET4` (`1` св. узел) и `TET10` (`19` св. узлов) | `max_sigma = 0.000e0 Pa`, `yielded = 0`; ошибка положения внутренних узлов на $180^\circ$: `1.075e-18 м` (`TET4`) / `1.801e-16 м` (`TET10`) | $\sigma_{\max} < 10^{-9}\text{ Па}$ | **`0.000e0 Pa`** | ✅ PASS |
| **2** | **Одноосное растяжение выше $\sigma_y$ → полная разгрузка (`springback`)** ($\sigma_y = 250\text{ МПа}$, $\sigma_{\max} = 350\text{ МПа}$, $E = 200\text{ ГПа}$, $H = 10\text{ ГПа}$) | `TET4` и `TET10` | $\sigma_{\text{load}} = 350.000\text{ МПа}$, $\varepsilon_{\text{tot}} = 0.011750$, $\varepsilon^p = 0.010000$, $\Delta\varepsilon^e = 0.001750$; остаточное $\sigma_{\text{res}} = 2.492\times 10^{-3}\text{ Па}$ (`TET4`) / `4.992e-3 Pa` (`TET10`) | $\varepsilon^p = 0.010000$, $\Delta\varepsilon^e = 0.001750$ (ошибка `< 1.0%`) | **`0.000%`** | ✅ PASS |
| **3** | **Упруго-пластический изгиб консольной балки ($240\times 16\times 8\text{ мм}$, $H = 0$, пластический шарнир + `springback`)** | `TET10` (`12×4×1`, сгущение к заделке) | $M_y = 85.33\text{ Н·м}$, $M_{\text{hinge}} = 133.72\text{ Н·м}$ ($M_{\text{root}} = 133.42\text{ Н·м}$), $M_p/M_y = \mathbf{1.5671}$, `tip_Fy = -557.2 N`, `root_Fy = +557.2 N`, `yielded = 305/960` (`15.3%` объёма); разгрузка: `dy_load = -18.000 мм` → `dy_perm = -13.311 мм`, упругий возврат `4.689 мм` (ожид. `~4.701 мм`) | $M_p = 1.5 M_y = 128.00\text{ Н·м}$, переход $M_y \to M_p$ без разрушения | **`4.47%`** (момент), **`0.25%`** (упругий возврат) | ✅ PASS |
| **4** | **Нелинейные законы упрочнения (`PowerLaw`, `Saturation`) и принудительное адаптивное дробление шага (`max_newton_iterations = 5`)** | `TET4` | `PowerLaw` и `Saturation` `< 0.5%` ошибки; адаптивная бисекция: `converged_steps = 30`, `total_bisections = 6`, `newton_iters = 128`, `max_ep = 0.03536` | Сходимость с автоматической бисекцией | **`< 0.5%`** | ✅ PASS |

---

## 3. Верификация тестами и линтером

Выполнено после всех финальных правок:
- `cargo test -p mcp-cad-fea --lib --test nonlinear_elastoplastic_tests -- --nocapture`: **17/17 PASS** (`--lib`: `13/13` в `0.00s`, `nonlinear_elastoplastic_tests`: `4/4` в `112.73s`).
- `cargo test -p mcp-cad-fea --test fea_tests --test fea_benchmarks --no-fail-fast`: **17/17 PASS** (`fea_benchmarks`: `3/3` в `122.81s`, `fea_tests`: `14/14` в `70.48s`).
  - Итого по `mcp-cad-fea`: **34/34 PASS** (`0 failed`).
- `cargo test -p mcp-cad-core --no-fail-fast`: **136/136 PASS** (`0 failed`).
- `cargo test -p mcp-cad-physics --no-fail-fast`: **42/42 PASS** (`0 failed`).
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).

---

## 4. Обнаруженные смежные проблемы (заведены в `TASKS.md`)

- **`P5-T12 [S]`**: В `crates/mcp-cad-fea/src/boundary.rs` распределение силы на грани `AabbFaceForce` для `TET4` (строки `604–658`) использует эвристические прямоугольные веса (`0.25 / 0.5 / 1.0`), не совпадающие с точным согласованным интегрированием по треугольникам $T_3$ на сетке Куна и нерегулярных/согласованных сетках (даёт локальные пики напряжений на углах и паразитные остаточные напряжения после разгрузки), а `SurfaceTrianglesForce` (строки `660–688`) не использует согласованные веса серединных узлов $T_6$ для `TET10`.

---

## 5. Затронутые файлы

- `crates/mcp-cad-fea/src/elasticity.rs`
- `crates/mcp-cad-fea/src/element.rs`
- `crates/mcp-cad-fea/src/solver.rs`
- `crates/mcp-cad-fea/src/result.rs`
- `crates/mcp-cad-fea/src/lib.rs`
- `crates/mcp-cad-fea/tests/nonlinear_elastoplastic_tests.rs` *(новый)*
- `docs/agent_tasks/P5-T3-report.md` *(новый)*
- `TASKS.md`, `docs/PROGRESS.md`, `docs/FEA_ACCURACY.md`
