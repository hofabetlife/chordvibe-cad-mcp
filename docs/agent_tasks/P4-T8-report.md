# Отчёт о выполнении задачи P4-T8

- **Задача**: `P4-T8` — Защита `ClosedLoopSolver` (`crates/mcp-cad-physics/src/simulator.rs`) от `NaN` в `Rotation3::scaled_axis()` при дрейфе следа матрицы $\operatorname{tr}(R) > 3.0$ и $\operatorname{tr}(R) < -1.0$
- **Статус**: ✅ ВЫПОЛНЕНО (`18.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-physics/src/simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs) *(замена `nalgebra::Rotation3::from_matrix_unchecked(r).scaled_axis()` в `rotation_vector(r)` на `NaN`-безопасную реализацию на базе `atan2(sin_t, cos_t)` с зажимом `cos_t = (0.5 * (r.trace() - 1.0)).clamp(-1.0, 1.0)` и веткой извлечения оси при $\theta \approx \pi$; добавлен модуль unit-тестов `simulator::tests`)*
  - [`TASKS.md`](../../TASKS.md), [`docs/PROGRESS.md`](../PROGRESS.md), [`CHANGELOG.md`](../../CHANGELOG.md) *(актуализация статуса `P4-T8` и результатов верификации)*

---

## 1. Причина и реализация исправления

В [`crates/mcp-cad-physics/src/simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs) решатель замкнутых кинематических контуров `ClosedLoopSolver` (методы `evaluate_system` и `commit_gear_angles`) вычисляет вектор невязки ориентации и угловые приращения зубчатых пар через вспомогательную функцию `rotation_vector(r: Matrix3<f64>) -> Vector3<f64>`. Ранее она вызывала `nalgebra::Rotation3::from_matrix_unchecked(r).scaled_axis()`, где угол вычисляется через $\arccos\left(\frac{\operatorname{tr}(R)-1}{2}\right)$ без предварительного ограничения аргумента к $[-1, 1]$. При накоплении ошибки округления `f64` в 1 ULP ($\operatorname{tr}(R) > 3.0$ на соосных звеньях или $\operatorname{tr}(R) < -1.0$ при развороте на $\pi$) это возвращало `Vector3::new(NaN, NaN, NaN)` и нарушало работу `SVD` в шагах Гаусса–Ньютона.

Внесённые изменения в [`crates/mcp-cad-physics/src/simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs):
1. `rotation_vector(r: Matrix3<f64>)` переписана через:
   - кососимметричную часть $\mathbf{s} = \frac{1}{2}[R_{21}-R_{12},\; R_{02}-R_{20},\; R_{10}-R_{01}]^T$, $\sin\theta = \|\mathbf{s}\|_2$;
   - $\cos\theta = \operatorname{clamp}\left(\frac{\operatorname{tr}(R) - 1}{2},\; -1.0,\; 1.0\right)$;
   - малый угол ($\sin\theta < 10^{-12}$, $\cos\theta \ge 0$): возврат $\mathbf{s}$;
   - разворот на $\pi$ ($\sin\theta < 10^{-12}$, $\cos\theta < 0$): устойчивое извлечение единичной оси из столбца $R + I$ с максимальным диагональным элементом и масштабирование на $\theta = \operatorname{atan2}(\sin\theta, \cos\theta)$;
   - общий случай: $\mathbf{s}\cdot\frac{\operatorname{atan2}(\sin\theta, \cos\theta)}{\sin\theta}$.
2. Проверены смежные участки кодовой базы:
   - `mat3ToScaledAxisTS` в [`src/cad/kinematics.ts`](../../src/cad/kinematics.ts) уже использует метод Шепперда + `Math.atan2(vNorm, w)` и устойчива к дрейфу следа.
   - Все вызовы `.acos()` в крейтах `crates/` уже защищены `.clamp(...)`.

---

## 2. Новые тесты и верификация

В `crates/mcp-cad-physics/src/simulator.rs` (`mod tests`) добавлены 2 unit-теста:
1. `test_p4_t8_rotation_vector_handles_trace_above_three_and_pi_without_nan`: проверяет конечность и точность `rotation_vector` при $\operatorname{tr}(R) = 3.0 + 6\varepsilon$ (идентичная матрица и малый поворот), $\operatorname{tr}(R) = -1.0 - 4\varepsilon$ (поворот на $\pi$) и произвольном 3D-повороте.
2. `test_p4_t8_closed_loop_solver_evaluate_system_finite_with_trace_drift`: проверяет `ClosedLoopSolver::evaluate_system` и `commit_gear_angles` на замкнутом контуре (`Revolute` + `Fixed` + `Prismatic` + `Gear`) при принудительном дрейфе матриц поворота звеньев с $\operatorname{tr}(R) > 3.0$.

Результаты проверок после финальной правки:
- `cargo test -p mcp-cad-physics --no-fail-fast`: **42/42 PASS** (`--lib`: **10/10**, `physics_tests`: **32/32**, `0 failed`).
- `cargo test -p mcp-cad-server --no-fail-fast`: **142/142 PASS** (`--lib`: **65/65**, `bench_suite`: **1/1**, `gear_kinematic_test`: **2/2**, `mcp_contract_tests`: **61/61**, `qa_wide_scan`: **13/13**, `0 failed`).
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
- `npm run lint` (`tsc --noEmit`) и `npm run build` (`vite build`): **PASS** (`exit 0`).
- `npm test` (`src/cad/cad_engine.test.ts` + `src/cad/__tests__/analyze_mechanism.test.ts`): **53/53 PASS** (`38/38` + `15/15`, `0 failed`).
