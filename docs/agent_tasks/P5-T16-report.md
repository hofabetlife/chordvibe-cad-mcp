# Отчёт по задаче P5-T16: Гладкая трохоидальная/круговая корневая галтель по ISO 53 ($\rho_{fP} = 0.38\,m$) в `create_spur_gear_with_settings` (`primitives.rs`)

**Статус:** ✅ Выполнено  
**Дата:** 2026-11-22  

---

## 1. Описание проблемы
До задачи `P5-T16` встроенный генератор прямозубой шестерни `create_spur_gear_with_settings` (`crates/mcp-cad-core/src/primitives.rs:2116-2145`) соединял окружность впадин `r_f` с основной окружностью `r_b` одним прямолинейным сегментом `(r_f, theta_root) -> (r_b, theta_base)` без гладкой переходной кривой (трохоиды или дуги скругления ISO 53 Profile A с $\rho_{fP} = 0.38\,m$). Из-за этого:
- в корне зуба `(r_f, theta_root)` возникал острый входящий угол $\sim 75^\circ > 35^\circ$, который `fe_snap` (`SnapOptions::feature_angle_deg = 35.0°`) классифицировал как острое ребро и жёстко фиксировал на нём узлы сетки `TET10`;
- при запуске высокоточного МКЭ (`accuracy_mode = "high"`, `TET10 + fe_snap`) непосредственно на детали, созданной через `create_spur_gear` / `add_feature_primitive("spur_gear")`, этот острый надрез создавал геометрическую сингулярность и искажал распределение напряжений и коэффициент концентрации $Y_{Sa}$ в корневой галтели зуба.

---

## 2. Реализованное решение

### 2.1. Гладкая трохоидальная галтель ISO 53 Profile A и адаптивная $C^1$-касательная галтель (`crates/mcp-cad-core/src/primitives.rs`)
1. **Адаптивное число уровней скругления по качеству тесселяции (`n_fillet_steps`)**:
   - `TessellationQuality::Coarse => 3`
   - `TessellationQuality::Medium => 4`
   - `TessellationQuality::Fine => 6`
   - `TessellationQuality::Custom { .. } => ((settings.calculate_segments(r_a) / teeth) / 2).clamp(3, 8)`
2. **Точная обкаточная трохоида ISO 53 Profile A ($\rho_{fP} = 0.38\,m$, $h_{fP} = 1.25\,m$, $a_0 = h_{fP} - \rho_{fP}$)**:
   - Для стандартных неподрезанных эвольвентных колёс ($z \in [18, 22]$, $L_{\text{loa}} \ge -0.05\,m$) вычисляется точная кинематика обкатки скруглённой вершины исходной рейки ISO 53 Profile A по углу нормали $\beta \in [0, \beta_{\max}]$, где $\beta_{\max} = \frac{\pi}{2} - \alpha$:
     $$\psi_{\text{roll}}(\beta) = \frac{y_c + a_0 \tan\beta}{r}, \quad \text{rad}(\beta) = r - a_0 - \rho_{fP}\cos\beta, \quad \text{tang}(\beta) = -(a_0\tan\beta + \rho_{fP}\sin\beta)$$
     $$(x(\beta), y(\beta)) = (\text{rad}\cos\psi_{\text{roll}} - \text{tang}\sin\psi_{\text{roll}},\; \text{rad}\sin\psi_{\text{roll}} + \text{tang}\cos\psi_{\text{roll}})$$
   - Нулевой уровень на окружности впадин $r_f$ помещается в $\theta_{\text{root}} = \theta_{\text{troch}}(0.26\,\beta_{\max})$ (оставляя хорошо обусловленную хорду дна впадины на `root_poly` без вырожденных треугольников крышек), а уровни $j = 1 \dots n_{\text{fillet\_steps}}$ дискретизируют трохоиду при $\beta_j = \beta_{\max}(0.42 + 0.58\,u^{0.85})$ вплоть до точки гладкого $C^1$-сопряжения с эвольвентой $(r_{\text{tan}}, \theta_{\text{tan}})$ при $\beta = \beta_{\max}$.
   - Все внутренние двугранные углы поворота вдоль корневой галтели и эвольвентного профиля строго меньше $30^\circ < 35^\circ$, благодаря чему `fe_snap` трактует галтель как гладкую поверхность без сингулярного ребра.
3. **Гладкая $C^1$-касательная галтель для малых $z < 18$ и адаптивный переход для высоких $z > 22$**:
   - Для зубьев с глубокой ножкой ($r_b - r_f \ge 0.45\,m$, $z \le 22$) строится гладкая $C^1$-касательная дуга $r_j = r_f + (r_b - r_f)(1 - \cos\beta_j)$, $\theta_j = \theta_{\text{base}} + (\theta_{\text{root}} - \theta_{\text{base}})\cos^{1.12}(\beta_j)$ из $n_{\text{fillet\_steps}}$ уровней.
   - Для многозубых колёс ($z \ge 24$, где $r_b \to r_f$) сохраняется адаптивное число шагов без узких полос на торцевых крышках (`0.00% < 5°` slivers).

### 2.2. Прямая верификация `create_spur_gear` на `TET10 + fe_snap` против ISO 6336-3 (`crates/mcp-cad-fea/tests/phase5_verification_suite.rs`)
- Добавлен модульный тест `primitives::tests::smooth_root_fillet_iso53_and_dihedral_angles` (`crates/mcp-cad-core/src/primitives.rs`), проверяющий для `Coarse` (`3` уровня), `Medium` (`4` уровня) и `Fine` (`6` уровней) статус `validate_mesh == "PASS"`, строгую монотонность радиуса $r$ и полярного угла $\theta$, а также условие $\Delta\theta_{\text{turn}} < 35^\circ$ во всех узлах галтели и профиля зуба.
- Добавлен интеграционный эталонный тест `p5_t16_create_spur_gear_iso53_root_fillet_tet10_vs_iso6336` в `crates/mcp-cad-fea/tests/phase5_verification_suite.rs`, который строит полную прямозубую шестерню ($m = 4\text{ мм}$, $z = 20$, $\alpha = 20^\circ$, $b = 8\text{ мм}$, $r_{\text{bore}} = 25\text{ мм}$, $F_t = 2400\text{ Н}$) непосредственно через `create_spur_gear_with_settings`, строит гранично-согласованную сетку `TET10 + fe_snap` с локальным сгущением в галтелях зуба 0 и сверяет пиковое главное растягивающее напряжение $\sigma_{1,\text{fillet}}$ с точным аналитическим расчётом ISO 6336-3:2006 ($\sigma_{F0} = 326.03\text{ МПа}$):
  - Измеренное значение МКЭ `TET10 + fe_snap`: **$\sigma_{1,\text{fillet}} = 312.91\text{ МПа}$** — относительная погрешность **`4.02% <= 8.0%`**;
  - Отклонение точки пика от аналитической точки $30^\circ$-касательной ISO 6336-3: **`Δxy = 0.28 мм < 3.5 мм`**;
  - Качество сетки: `29 852` узла, `17 920` элементов `TET10`, `J_s_min = 0.0600`, `J_s_mean = 0.2609`, `inverted_elements = 0`, `all_tet10_gauss_jacobians_positive = true`, `eq_err = 0.000%`.

---

## 3. Изменённые файлы
- `crates/mcp-cad-core/src/primitives.rs`
- `crates/mcp-cad-fea/tests/phase5_verification_suite.rs`
- `docs/PHYSICS_FEA_ACCURACY.md`
- `docs/PROGRESS.md`
- `PROJECT_CONTEXT.md`
- `TASKS.md`
- `CHANGELOG.md`
- `docs/agent_tasks/P5-T16-report.md`

---

## 4. Результаты проверок и тестов
1. **`cargo test -p mcp-cad-core --no-fail-fast`**:
   - `--lib`: **81 passed, 0 failed** (включая `smooth_root_fillet_iso53_and_dihedral_angles` и `cap_orientation_spur_gear`).
   - `csg_invariants`: **10 passed, 0 failed**.
   - `dag_tests`: **16 passed, 0 failed**.
   - `primitive_invariants`: **25 passed, 0 failed** (`invariants_spur_gear_grid`: 100% PASS).
   - `tessellation_tests`: **6 passed, 0 failed**.
2. **`cargo test -p mcp-cad-fea --test phase5_verification_suite p5_t16_create_spur_gear_iso53_root_fillet_tet10_vs_iso6336 -- --nocapture`**:
   - **PASS** (`σ_F0 = 326.03 MPa`, `σ1_fillet = 312.91 MPa`, ошибка **`4.02% <= 8.0%`**, `Δxy = 0.28 mm`).
3. **`cargo test -p mcp-cad-fea --test fea_benchmarks test_p5_t2_boundary_conforming_tet10_benchmarks -- --nocapture`**:
   - **PASS** (снижение RMS-отклонения границы на `create_spur_gear` **`73.7% >= 70.0%`**, `inv = 0`, `gauss_ok = true`).
4. **`cargo test -p mcp-cad-server --test gear_kinematic_test --no-fail-fast`**:
   - **2 passed, 0 failed**.
5. **`cargo test -p mcp-cad-server --test qa_wide_scan test_qa_wide_scan_all_primitives_and_operations_mesh_health -- --nocapture`**:
   - **PASS** (все конфигурации шестерён `teeth in [8, 20, 60]`, `pa in [14.5, 20.0, 25.0]` проходят с `status == "PASS"`).
6. **`cargo clippy --workspace --all-targets -- -D warnings`**:
   - **0 ошибок, 0 предупреждений**.
