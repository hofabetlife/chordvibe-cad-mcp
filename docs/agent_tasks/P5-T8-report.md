# Отчёт по задаче `P5-T8`: Эталонный верификационный пакет точности Фазы 5 и отчёт `docs/PHYSICS_FEA_ACCURACY.md`

- **Дата выполнения**: 21.11.2026
- **Статус**: ✅ DONE

---

## 1. Реализованные изменения

### 1.1. Единый верификационный сьют Фазы 5 (`crates/mcp-cad-fea/tests/phase5_verification_suite.rs`)
Создан комплексный интеграционный сьют из 5 эталонных тестов, проверяющих всю сквозную расчётную цепочку (`CAD → TET10 + fe_snap → Co-Rotated J2 → AL/Mortar Contact → Local Damage/Fracture → HHT-α`):
1. **`p5_t8_benchmark_1_uniaxial_elastoplastic_cycle_springback`**: одноосный упруго-пластический цикл нагружения выше предела текучести ($\sigma_y = 250\text{ МПа} \to \sigma_{\max} = 350\text{ МПа}$, $E = 200\text{ ГПа}$, $H = 10\text{ ГПа}$) и полной разгрузки (`springback`) на `TET4` и `TET10`. Измерено: ошибка $\varepsilon^p$ = **`0.0000%`**, ошибка `springback` = **`0.0000%`** (допуск `< 1.0%`), остаточные напряжения $\le 6.48\times 10^{-5}\text{ Па}$, пластическая несжимаемость $\max|\operatorname{tr}(\varepsilon^p)| = 3.47\times 10^{-18}$.
2. **`p5_t8_benchmark_2_cantilever_plastic_hinge`**: упруго-пластический изгиб консольной балки ($240\times 16\times 8\text{ мм}$, $H = 0$, `TET10`) до формирования пластического шарнира с последующей разгрузкой `springback`. Измерено: $M_y = 85.33\text{ Н·м} \to M_{p,\text{FEM}} = \mathbf{133.42\text{ Н·м}}$ ($M_p/M_y = \mathbf{1.5635}$ против `1.5000`, ошибка **`4.24%`** `<= 5.0%`), $\delta_{\text{perm}} = 13.311\text{ мм}$, упругий возврат `4.689 мм` (ошибка **`0.25%`**).
3. **`p5_t8_benchmark_3_hertz_two_elastic_cylinders`**: контакт Герца двух параллельных упругих цилиндров ($R_1 = R_2 = 50\text{ мм}$, $E = 205\text{ ГПа}$, $\nu = 0.29$, $P' = 400\text{ кН/м}$). Измерено: $p_0 = \mathbf{776.36\text{ МПа}}$ против `754.96 МПа` (ошибка **`2.84%`** `<= 5.0%`), полуширина $a = \mathbf{0.3410\text{ мм}}$ против `0.3373 мм` (ошибка **`1.11%`** `<= 5.0%`), $\sigma_{vM,\text{sub}} = 440.5\text{ МПа}$ против `422.8 МПа` (`4.18%`), `F_err = 0.000%`, `max_pen = 4.90e-11 м`.
4. **`p5_t8_benchmark_4_spur_gear_tooth_bending_tet10_fe_snap_vs_iso6336`**: 3-зубый водонепроницаемый сектор стандартной прямозубой шестерни ($m = 5\text{ мм}, z = 20, \alpha = 20^\circ, b = 10\text{ мм}$) с точной трохоидальной переходной галтелью ISO 53 Profile A ($\rho_{fP} = 0.38\,m$), прогнанный через продакшн-генератор `TetrahedralMesh::from_mesh_conforming` (`TET10` + 2:1 octree + `fe_snap`, `17 369` узлов, `11 560` элементов, `J_s_min = 0.0600`, `0` вывернутых, `gauss_ok = true`, `eq_err = 0.000%`). Измерено: $\sigma_{1,\text{fillet}} = \mathbf{85.30\text{ МПа}}$ против точного эталона ISO 6336-3 $\sigma_{F0} = \mathbf{86.94\text{ МПа}}$ ($Y_{Fa} = 2.8000$, $Y_{Sa} = 1.5525$, ошибка **`1.89%`** `<= 8.0%`), глобальный максимум $\sigma_1$ лежит строго на растянутой корневой галтели в `(44.37, 4.92) мм` против аналитической точки касания $30^\circ$ `(44.46, 4.86) мм` (`Δxy = 0.10 мм`).
5. **`p5_t8_benchmark_5_gear_pair_three_regimes_elastic_plastic_fracture`**: зубчатая пара Steel 4140 vs Brass C360 в трёх последовательных режимах + динамический удар `HHT-α`:
   - **(a) Упругий контакт + разгрузка**: $F_{n,\max} = 3935.4\text{ Н}$, $\sigma_{vM} = 76.1\text{ МПа} < 310\text{ МПа}$, $\bar{\varepsilon}_{p,\text{res}} = 0.00$, $D = 0.00$, $u_{\text{res}} = 4.41\times 10^{-12}\text{ м}$, `max_pen = 1.26e-7 м`.
   - **(b) Пластическое замятие без разрушения**: пик латуни $\sigma_{vM} = 311.2\text{ МПа}$, остаточное $\bar{\varepsilon}_{p,\text{res}} = 0.00136$, $D_{\max} = 0.0000$, `eroded = 0`, `frags = 0`, `max_pen = 4.82e-7 м`.
   - **(c) Локальный скол зуба при целой ступице**: `steel_eroded = 0`, `brass_eroded = 67`, `frags = 1` (`is_valid_solid == true`, `boundary_edges == 0`), инициация в корневой галтели `init_r = 24.11 мм` (`root = 23.00`, `pitch = 28.00`), `intact_teeth = 12/14`, `hub_intact = true` ($D = 0$), ошибка сохранения $(m, \mathbf{P}, \mathbf{L}) = \mathbf{4.35\times 10^{-16}} \le 10^{-6}$, `max_pen = 2.54e-8 м < 1e-5 м`.
   - **(d) Неявный динамический удар `HHT-α`**: `frames = 8`, $F_{n,\max} = 3812.6\text{ Н}$, `max_pen = 0.00e0 м`, `32.4 мс`.

### 1.2. Компаньон-эталон `f64` DAE `PhysicsWorld` (`crates/mcp-cad-physics/tests/physics_tests.rs`)
Добавлен тест `test_p5_t8_gear_pair_dynamic_collision_three_regimes_and_fragment_conservation`, проверяющий 3 динамических режима столкновения и постотломный полёт осколка в `f64` DAE-решателе:
- Упругий удар ($e = 0.98$): ошибка сохранения импульса `2.21e-16`, $v_b = 1.980\text{ м/с}$;
- Неупругий удар ($e = 0.05$): ошибка импульса `0.00e0`, `KE_ratio = 0.501` (аналитика `0.50125`);
- Отделение фрагмента зуба и столкновение со стендом: сохранение `(m: 0.00e0, p: 2.04e-16, L: 0.00e0)`, дрейф оси родительской шестерни `1.49e-16 м`, $\omega_z = 20.00\text{ рад/с}$, `stand_hit = true` (`12.3 мс`).

### 1.3. Документация точности и синхронизация проекта
- Создан отчёт [`docs/PHYSICS_FEA_ACCURACY.md`](../PHYSICS_FEA_ACCURACY.md) с полной сводной таблицей измеренных погрешностей, аналитическим выводом формул для всех 5 эталонов, временем расчёта, честными границами применимости и командами воспроизведения.
- Обновлены [`docs/FEA_ACCURACY.md`](../FEA_ACCURACY.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md), [`TASKS.md`](../../TASKS.md) и [`CHANGELOG.md`](../../CHANGELOG.md).

---

## 2. Результаты верификации

1. `cargo test -p mcp-cad-fea --test phase5_verification_suite -- --nocapture`: **5/5 PASS** (`43.55s`, `0 failed`).
2. `cargo test -p mcp-cad-physics --test physics_tests test_p5_t8 -- --nocapture`: **1/1 PASS** (`0.01s`, `0 failed`).
3. Полные регрессионные прогоны (выполнены после всех правок кода):
   - `cargo test -p mcp-cad-fea --lib --test fea_tests --test dynamic_hht_tests --test local_fracture_tests --test deformable_contact_tests --test phase5_verification_suite --no-fail-fast`: **54/54 PASS** (`0 failed`; `fea_benchmarks` `3/3` и `nonlinear_elastoplastic_tests` `4/4` не затрагивались, библиотечный код `mcp-cad-fea` не изменялся);
   - `cargo test -p mcp-cad-physics --no-fail-fast`: **47/47 PASS** (`0 failed`);
   - `cargo test -p mcp-cad-core --no-fail-fast`: **136/136 PASS** (`0 failed`);
   - `cargo test -p mcp-cad-viewer --lib --no-fail-fast`: **2/2 PASS** (`0 failed`);
   - `cargo test -p mcp-cad-server --lib sse::tests --no-fail-fast`: **3/3 PASS** (`0 failed`; тесты с `EngineState::new()` / `OffscreenRenderer` в headless-контейнере без Vulkan ICD отслеживаются в `P5-T9`);
   - `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах);
   - `npm run lint` (`tsc --noEmit`), `npm run build` (`vite build`) & `npx --yes tsx --test ...`: **72/72 PASS** (`0 failed`).

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T16 [S]`**: В `crates/mcp-cad-core/src/primitives.rs:2116-2145` генератор `create_spur_gear_with_settings` строит переход от впадины $r_f$ к основной окружности $r_b$ одним прямолинейным сегментом без гладкой трохоидальной/круговой галтели ISO 53 ($\rho_{fP} = 0.38\,m$), что создаёт острый входящий угол-сингулярность в корне зуба при запуске МКЭ (`accuracy_mode = "high"`) непосредственно на геометрии примитива `spur_gear`.
