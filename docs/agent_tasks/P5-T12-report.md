# Отчёт по задаче `P5-T12`: Согласованное поверхностное интегрирование распределённой силы (`T3`/`T6`) в `AabbFaceForce` и `SurfaceTrianglesForce` (`boundary.rs`)

- **Дата выполнения**: 21.11.2026
- **Статус**: ✅ DONE

---

## 1. Реализованные изменения

### 1.1. Согласованные веса поверхностных треугольников `T6` и `T3` (`crates/mcp-cad-fea/src/boundary.rs`)
- Вынесены на уровень модуля константы 6-точечной симметричной квадратуры Дунаванта 4-го порядка (`T6_DUNAVANT_GP6`), переиспользуемые в `integrate_conforming_surface_pressure` и в новом интеграторе поверхностной силы.
- Добавлена функция `t6_consistent_area_weights(p0, p1, p2, m0, m1, m2) -> ([f64; 6], f64)`, вычисляющая точные согласованные веса виртуальной работы $w_k = \int_{\hat{T}} N_k^{(6)}(\xi, \eta)\,\|\mathbf{x}_\xi \times \mathbf{x}_\eta\|\,d\xi\,d\eta$:
  - для прямолинейных треугольников `T6` (отклонение серединных узлов $\le 10^{-12} \cdot L_{\max}$) возвращает аналитически точный результат `([0, 0, 0, A/3, A/3, A/3], A)` без плавающего шума на углах;
  - для криволинейных граней или треугольников со смещёнными серединными узлами интегрирует 6-точечной квадратурой `T6_DUNAVANT_GP6` с точным соблюдением разбиения единицы $\sum_{k=0}^5 w_k = A_{\text{curved}}$.
- Добавлена вспомогательная функция `aabb_face_outward_normal(face: AabbFace) -> Vector3<f64>` для фильтрации граней по ориентации нормали.

### 1.2. Замена эвристики `0.25 / 0.5 / 1.0` в `NeumannLoad::AabbFaceForce`
- Ветка **`TET4`**: удалена эвристика прямоугольной сетки (`Corner = 0.25, Edge = 0.5, Interior = 1.0`), которая не совпадала с реальной валентностью треугольников на сетке Куна и давала угловые скачки напряжений. Теперь интегрирование выполняется по фактическим внешним треугольникам `T3` (`mesh.extract_surface_triangles()`), лежащим на целевой грани AABB и согласованным по нормали (`|face_n · aabb_n| > 0.25`), с точным вкладом $A_T / 3$ на каждую из трёх вершин.
- Ветка **`TET10`**: переведена на `t6_consistent_area_weights` с фильтрацией по нормали (`|face_n · aabb_n| > 0.25`), что корректно поддерживает как плоские (`0` на углах, `A_T / 3` на серединах рёбер), так и криволинейные грани `T6`.
- Равномерное распределение `1 / N` оставлено исключительно как защитный фолбэк для вырожденного точечного/рёберного контакта с границей AABB (когда на грани нет ни одного треугольника).

### 1.3. Согласованное `T3`/`T6`-интегрирование в `NeumannLoad::SurfaceTrianglesForce` и `match_surface_nodes_and_weights`
- Добавлена функция `integrate_conforming_surface_force(...)` для гранично-согласованных сеток (`mesh.is_boundary_conforming()`), поддерживающая `TET10` (`t6_consistent_area_weights`, квадратичный центроид `quad_centroid`, локальный допуск по ребру `local_tol`, фильтр по нормали и слою плоскости) и `TET4` (точное `T3`-интегрирование $A_T / 3$ по нормально-согласованным граням), по аналогии с `integrate_conforming_surface_pressure`.
- В `NeumannLoad::SurfaceTrianglesForce` сначала вызывается `integrate_conforming_surface_force` для согласованных сеток, а при фолбэке на `match_surface_nodes_and_weights` (структурированные сетки) обновлённый `match_surface_nodes_and_weights` также использует `t6_consistent_area_weights`, `quad_centroid` и проверку соосности нормали (`>= 0.35`) и расстояния до плоскости целевой грани (исключая боковые треугольники, ребро которых лишь касается целевой грани).

### 1.4. Новые unit-тесты (`crates/mcp-cad-fea/src/boundary.rs`, `mod tests`)
1. `test_p5_t12_t6_consistent_area_weights_flat_and_curved` — проверяет точные нули на углах и `A/3` на серединах рёбер для плоского `T6`, а также $A_{\text{curved}} > A_{\text{flat}}$ и разбиение единицы $\sum w_k = A_{\text{curved}}$ (`< 1e-12`) для параболически изогнутого `T6`.
2. `test_p5_t12_uniaxial_tension_uniform_stress_aabb_and_named_face_tet4_and_tet10` — проверяет все 8 комбинаций `{structured_TET4, structured_TET10, conforming_TET4, conforming_TET10} × {add_aabb_face_force, add_named_face_force}` при одноосном растяжении стержня: сохранение равнодействующей (`< 1e-12`), точные нулевые узловые силы на угловых узлах плоских граней `TET10` (`< 1e-12`), однородность поля напряжений $\sigma_{vM} = F/A$ во всех узлах с относительной ошибкой **`< 1e-6`** (при критерии `< 1e-5`) и точность аналитического 3D-поля перемещений $(u_x, u_y, u_z)$ во всех узлах **`< 1e-6`**.
3. `test_p5_t12_elastoplastic_uniaxial_springback_zero_spurious_corner_residual_stress` — проверяет упруго-пластический цикл нагружения выше $\sigma_y$ (`300 МПа > 250 МПа`) и полной разгрузки (`springback`) через `add_aabb_face_force` и `add_named_face_force` на `TET4` и `TET10`: отсутствие паразитных угловых остаточных напряжений (`max_stress_von_mises_pa < 1e3 Pa` вместо `~5.7 МПа`) и точность $\bar{\varepsilon}_p$ `< 1e-4`.

---

## 2. Результаты верификации

1. `cargo test -p mcp-cad-fea --lib --no-fail-fast`: **25/25 PASS** (`0.33s`, включая все 3 новых теста `boundary::tests::test_p5_t12_*`).
2. Полный прогон всех интеграционных наборов `mcp-cad-fea` (**64/64 PASS**, `0 failed`):
   - `fea_tests`: **14/14 PASS** (`41.30s`);
   - `fea_benchmarks`: **3/3 PASS** (`95.14s`);
   - `deformable_contact_tests`: **5/5 PASS** (`12.39s`);
   - `dynamic_hht_tests`: **4/4 PASS** (`3.23s`);
   - `local_fracture_tests`: **4/4 PASS** (`14.18s`);
   - `nonlinear_elastoplastic_tests`: **4/4 PASS** (`43.89s`);
   - `phase5_verification_suite`: **5/5 PASS** (`89.61s`).
3. Смежные наборы `mcp-cad-server`:
   - `bench_suite`: **1/1 PASS** (`13.52s`);
   - `qa_wide_scan`: **13/13 PASS** (`15.52s`).
4. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
5. `npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`): **PASS**.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)

- **`P5-T21 [S]`**: В `NeumannLoad::SurfaceTrianglesMoment` (`crates/mcp-cad-fea/src/boundary.rs:764-813`) распределение крутящего/изгибающего момента по поверхности использует сосредоточенные веса `m.area_weight` из `match_surface_nodes_and_weights` (`f_i ∝ w_i * (m_axis × r_i)`). Для квадратичных сеток `TET10` на плоских гранях `T6` согласованный нулевой момент площади угловых узлов равен $w_{\text{corner}} = \int_T N_{\text{corner}}\,dA = 0$, поэтому угловые узлы получают строго нулевую силу от момента, хотя для линейного по координатам поля усилий $\mathbf{t}(\mathbf{x}) \propto \mathbf{m}_{\text{axis}} \times \mathbf{r}$ точный первый момент $\int_T N_{\text{corner}}(\mathbf{x} - \mathbf{c})\,dA = \frac{A_T}{60}(2\mathbf{v}_a - \mathbf{v}_b - \mathbf{v}_c)$ отличен от нуля (см. `compute_consistent_face_moments`). Требуется перевести `SurfaceTrianglesMoment` на согласованное квадратурное интегрирование `T3`/`T6`.
