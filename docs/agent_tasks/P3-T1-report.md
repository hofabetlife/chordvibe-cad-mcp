# Отчёт о выполнении задачи P3-T1

## Задача
**P3-T1 [M]: Базовая точность FEA: эталонные задачи (`crates/mcp-cad-fea/tests/fea_benchmarks.rs`, `docs/FEA_ACCURACY.md`)**

---

## 1. Что сделано (файлы, функции)

1. **`crates/mcp-cad-fea/tests/fea_benchmarks.rs` (создан)**:
   - Реализован интеграционный суит `test_fea_accuracy_benchmarks_suite`, прогоняющий `MeshDensity::{Coarse, Medium, Fine}` на 4 классических аналитических эталонах механики деформируемого твёрдого тела (`E = 205 ГПа`, `nu = 0.30`):
     - `run_axial_tension_case`: осевое растяжение призматического стержня $1000 \times 10 \times 10\text{ мм}$ силой $F = 2050\text{ Н}$ ($\sigma = 20.50\text{ МПа}$, $\delta = 0.1000\text{ мм}$);
     - `run_cantilever_case`: поперечный изгиб консольной балки $200 \times 20 \times 20\text{ мм}$ силой $F = 1000\text{ Н}$ ($\delta_{\text{EB}} = 0.97561\text{ мм}$, $\delta_{\text{Timoshenko}} = 0.98322\text{ мм}$, $\sigma_{\text{root}} = 150.00\text{ МПа}$, а также замер $\sigma(0.5h) = 142.50\text{ МПа}$ вне сингулярности закрепления);
     - `run_plate_with_hole_case`: растяжение перфорированной пластины $600 \times 200 \times 10\text{ мм}$ с центральным круговым отверстием $\varnothing 40\text{ мм}$ силой $F = 100\text{ кН}$ ($\sigma_{\text{nom}} = 62.50\text{ МПа}$, сравнение с полиномом Петерсона $K_t = 2.5065 \implies 156.65\text{ МПа}$ и эталоном `bench/README.md` $K_t = 2.519 \implies 157.44\text{ МПа}$);
     - `run_lame_cylinder_case`: толстостенный цилиндр ($a = 20\text{ мм}$, $b = 40\text{ мм}$, $L = 40\text{ мм}$) под внутренним давлением $p = 10\text{ МПа}$ через `SurfaceTrianglesPressure` на именованной грани `hole_wall` ($\sigma_\theta(a) = 16.67\text{ МПа}$, $\sigma_r(a) = -10.00\text{ МПа}$, $\sigma_{\text{vM}}(a) = 23.33\text{ МПа}$, $u_r(a) = 0.001919\text{ мм}$).
   - Для всех 12 расчётных случаев (4 задачи × 3 плотности) внедрены проверки «трещоткой» (`check_ratchet`): `ошибка <= измеренная * 1.25 + 1.0%`.

2. **`docs/FEA_ACCURACY.md` (создан)**:
   - Оформлена сводная таблица «задача × плотность → число элементов и узлов, расчётные и аналитические перемещения/напряжения, относительные ошибки, время расчёта».
   - Приведены аналитические формулы с указанием первоисточников (Тимошенко–Гере, Roark's Formulas 8th ed., Pilkey «Peterson's Stress Concentration Factors» 3rd ed., Lamé).
   - Сформулирован инженерный вывод о границах применимости текущего МКЭ-движка (`TET4` + воксельная сетка).

3. **`Cargo.toml`**:
   - В `[profile.dev]` и `[profile.test]` добавлен `opt-level = 2`, чтобы итерационный решатель `solve_pcg` на плотных сетках (`Fine`: до 128 160 тетраэдров и 77 616 степеней свободы) укладывался в норматив `< 5 минут` (`~127 с` суммарно на все 12 расчётов).

4. **`TASKS.md` и `PROJECT_CONTEXT.md`**:
   - Статус задачи **P3-T1** переведён в выполненные, обновлено число тестов `mcp-cad-fea` (`14` вместо `13`, всего в workspace `236` тестов).
   - В `TASKS.md` зарегистрирована новая побочная находка **P3-T13** (два предупреждения `clippy::type_complexity` в `crates/mcp-cad-core/src/csg.rs:1175, 1285` на Rust `1.91.1`).

---

## 2. Как проверено (команды и итоговые строки `test result`)

- Запуск эталонного суита `fea_benchmarks`:
  ```bash
  cargo test -p mcp-cad-fea --test fea_benchmarks -- --nocapture
  ```
  Результат:
  ```text
  running 1 test
  [1. Axial Tension (1000x10x10mm)] Coarse | Elems:   1440 | Nodes:   549 | Disp: 0.100069 mm vs 0.100000 mm (err:   0.07%) | Stress:  21.31 MPa vs  20.50 MPa (err:   3.94%) |  145.5 ms | mean_stress=20.50 MPa
  [1. Axial Tension (1000x10x10mm)] Medium | Elems:   5760 | Nodes:  1525 | Disp: 0.100114 mm vs 0.100000 mm (err:   0.11%) | Stress:  20.80 MPa vs  20.50 MPa (err:   1.44%) | 1315.9 ms | mean_stress=20.46 MPa
  [1. Axial Tension (1000x10x10mm)] Fine   | Elems:  12960 | Nodes:  2989 | Disp: 0.099988 mm vs 0.100000 mm (err:   0.01%) | Stress:  21.29 MPa vs  20.50 MPa (err:   3.87%) | 4035.2 ms | mean_stress=20.48 MPa
  [2. Cantilever (200x20x20mm)] Coarse | Elems:    480 | Nodes:   189 | Disp: 0.505805 mm vs 0.975610 mm (err:  48.15%) | Stress:  80.73 MPa vs 150.00 MPa (err:  46.18%) |   11.3 ms | Timo_err=48.56%, stress@0.5h=61.47 MPa vs 142.50 MPa (err=56.86%)
  [2. Cantilever (200x20x20mm)] Medium | Elems:   3840 | Nodes:  1025 | Disp: 0.779117 mm vs 0.975610 mm (err:  20.14%) | Stress: 117.13 MPa vs 150.00 MPa (err:  21.91%) |  130.3 ms | Timo_err=20.76%, stress@0.5h=102.29 MPa vs 142.50 MPa (err=28.21%)
  [2. Cantilever (200x20x20mm)] Fine   | Elems:  12960 | Nodes:  2989 | Disp: 0.875693 mm vs 0.975610 mm (err:  10.24%) | Stress: 132.56 MPa vs 150.00 MPa (err:  11.63%) |  705.7 ms | Timo_err=20.94%, stress@0.5h=116.59 MPa vs 142.50 MPa (err=18.18%)
  [3. Plate w/ Hole (600x200x10, d=40)] Coarse | Elems:  28464 | Nodes:  7452 | Disp: 0.151986 mm vs 0.149406 mm (err:   1.73%) | Stress: 127.85 MPa vs 156.65 MPa (err:  18.39%) | 6916.2 ms | Kt_calc=2.046 vs Kt_Peterson=2.506 (156.65 MPa) / Kt_bench=2.519 (157.44 MPa, err=18.80%)
  [3. Plate w/ Hole (600x200x10, d=40)] Medium | Elems:  85440 | Nodes: 18480 | Disp: 0.151575 mm vs 0.149406 mm (err:   1.45%) | Stress: 131.29 MPa vs 156.65 MPa (err:  16.19%) | 36990.4 ms | Kt_calc=2.101 vs Kt_Peterson=2.506 (156.65 MPa) / Kt_bench=2.519 (157.44 MPa, err=16.61%)
  [3. Plate w/ Hole (600x200x10, d=40)] Fine   | Elems: 128160 | Nodes: 25872 | Disp: 0.151477 mm vs 0.149406 mm (err:   1.39%) | Stress: 130.68 MPa vs 156.65 MPa (err:  16.58%) | 74548.7 ms | Kt_calc=2.091 vs Kt_Peterson=2.506 (156.65 MPa) / Kt_bench=2.519 (157.44 MPa, err=16.99%)
  [4. Lamé Cylinder (a=20, b=40mm)] Coarse | Elems:   3072 | Nodes:   828 | Disp: 0.002522 mm vs 0.001919 mm (err:  31.47%) | Stress:  29.92 MPa vs  23.33 MPa (err:  28.24%) |  310.3 ms | sigma_theta=16.67 MPa, mean_bore_vM=14.49 MPa
  [4. Lamé Cylinder (a=20, b=40mm)] Medium | Elems:  14112 | Nodes:  3224 | Disp: 0.002490 mm vs 0.001919 mm (err:  29.76%) | Stress:  32.34 MPa vs  23.33 MPa (err:  38.58%) |  747.1 ms | sigma_theta=16.67 MPa, mean_bore_vM=17.36 MPa
  [4. Lamé Cylinder (a=20, b=40mm)] Fine   | Elems:  44160 | Nodes:  9248 | Disp: 0.002542 mm vs 0.001919 mm (err:  32.48%) | Stress:  30.10 MPa vs  23.33 MPa (err:  28.98%) | 1778.4 ms | sigma_theta=16.67 MPa, mean_bore_vM=18.49 MPa
  test test_fea_accuracy_benchmarks_suite ... ok
  test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 127.64s
  ```

- Полный прогон `cargo test -p mcp-cad-fea`:
  - Юнит-тесты `src/lib.rs`: `5 passed; 0 failed`
  - Интеграционные тесты `tests/fea_tests.rs`: `8 passed; 0 failed`
  - Бенчмарк-тесты `tests/fea_benchmarks.rs`: `1 passed; 0 failed`

---

## 3. Тесты до / после

- **До (`mcp-cad-fea`)**: 13 тестов (5 unit + 8 integration), 13 проходят, 0 падают.
- **После (`mcp-cad-fea`)**: 14 тестов (5 unit + 8 integration + 1 benchmark suite с 12 задачами), 14 проходят, 0 падают. Новых падений нет.

---

## 4. Что не удалось и остаточные риски

- Все 4 эталонные задачи (включая опциональную задачу Ламе №4 с внутренним давлением через `SurfaceTrianglesPressure`) успешно смоделированы и решены.
- **Выявленные физические ограничения текущего МКЭ (зафиксированы для P3-T3..P3-T5)**:
  1. На изгибе тонких сечений при плотности `Coarse` линейные элементы `TET4` дают ошибку прогиба **48.15%** из-за `shear locking` (решается переходом на `TET10` в **P3-T4**).
  2. На криволинейных концентраторах (пластина Кирша с отверстием и цилиндр Ламе) воксельная сетка (`TetrahedralMesh::from_mesh`) образует ступенчатую («лестничную») границу, из-за чего ошибка пикового напряжения не стремится к нулю при измельчении вокселей (`~16.5%` на пластине с отверстием и `~29%` на цилиндре Ламе). Решается проекцией граничных вершин вокселей на поверхность `TriangleMesh` в **P3-T5**.

---

## 5. Побочные находки (не чинить в этой задаче, занесены в `TASKS.md`)

1. **Предупреждение `clippy::type_complexity` на компиляторе `rustc 1.91.1`**:
   - В `crates/mcp-cad-core/src/csg.rs:1175` (`ear_clip_planar_ring_maxmin`) и `crates/mcp-cad-core/src/csg.rs:1285` (`try_remove_vertex_ear_clip`) возвращаемый тип `Option<Vec<([u32; 3], Vector3<f64>, u32)>>` вызывает ошибку `clippy::type_complexity` при запуске `cargo clippy -- -D warnings` на `rustc 1.91.1`. Заведено как отдельная задача **P3-T13** в `TASKS.md`.
2. **Неиспользуемый модуль `crates/mcp-cad-fea/src/convergence.rs`**:
   - Файл `convergence.rs` присутствует в дереве исходников `crates/mcp-cad-fea/src/`, но не подключён через `pub mod convergence;` в `crates/mcp-cad-fea/src/lib.rs` (запланировано к подключению в **P3-T3**).
