# Отчёт по задаче P0-T10: Качество сетки примитивов на краевых параметрах

- **ID задачи:** `P0-T10`
- **Фаза:** `P0 Foundation`
- **Размер:** `M`
- **Компонент:** `crates/mcp-cad-core`
- **Статус:** `✅ DONE`
- **Затронутые файлы:**
  - `crates/mcp-cad-core/src/primitives.rs`
  - `crates/mcp-cad-core/tests/primitive_invariants.rs`
  - `TASKS.md`
  - `docs/agent_tasks/P0-T10-report.md` (новый)

---

## 1. Что сделано

### 1.1. Конус: поддержка нечётного числа сегментов при 2-к-1 разрежении (`create_cone`, острый конус)
- **Проблема (`known_bug_cone_sharp_auto_segments_odd_after_halving_slivers`):**  
  При авто-тесселяции высокого острого конуса (`radius_bottom = 0.05`, `radius_top = 0.0`, `height = 0.25`, `segments = 0`) функция `calculate_segments` возвращает 50 сегментов. После первого деления 2-к-1 (`50 -> 25`) условие `curr_segs % 2 == 0` становилось ложным, разрежение колец у вершины блокировалось и порождало 50 игольчатых треугольников (1.85% от общего числа, `min_angle < 5°`).
- **Решение:**  
  В `create_cone` (ветка `!has_top_cap`) порог разрежения снижен до `curr_segs >= 12` и добавлена поддержка перехода при нечётном `curr_segs`:
  Для $j = 0..\frac{\text{curr\_segs}}{2} - 1$ выполняется обычный переход 2-к-1 (3 треугольника), а для последнего сектора $j = \frac{\text{curr\_segs}}{2} - 1$ — конформный переход 3-к-1 (4 треугольника) к нечётному остаточному сегменту:
  $$(o_0, o_1, i_0), \quad (o_1, i_1, i_0), \quad (o_1, o_2, i_1), \quad (o_2, o_3, i_1)$$
  Это позволяет кольцам гладко уменьшаться до малого числа вершин (до 6 или 3) перед вершиной конуса, устраняя любые иглы независимо от исходной чётности `segs`. В параметрическом тесте `invariants_cone_sharp_and_frustum_grid` снято исключение `if !(segs == 0 && r_bot >= 0.05)`, теперь полный перебор проходит на 100%.

### 1.2. Перевёрнутый конус: внедрение кольцевого разрежения у вершины (`create_cone`, $r_{\text{bot}} = 0$)
- **Проблема (`known_bug_cone_inverted_apex_missing_ring_halving`):**  
  В ветке перевёрнутого конуса `!has_bot_cap` (`radius_bottom = 0`, `radius_top > 0`, вершина на $z = -h/2$) осевое разрежение колец 2-к-1 отсутствовало полностью: переменная `curr_segs` не изменялась вдоль высоты, и все 64/128 лучей сходились в одну точку на нижнем полюсе, создавая 128 игольчатых треугольников (`min_angle = 2.51° < 5°`).
- **Решение:**  
  В ветку `!has_bot_cap` внедрён механизм ступенчатого 2-к-1 (и 3-к-1 при нечётном числе вершин) геометрического разрежения колец с корректной ориентацией граней для нормалей с $n_z < 0$:
  $$(o_0, i_0, o_1), \quad (o_1, i_0, i_1), \quad (o_1, i_1, o_2)$$
  Число лучей у нижнего полюса сокращается до безопасного диапазона 4–8, где угол при вершине заведомо $\ge 45^\circ \gg 5^\circ$.

### 1.3. Цилиндрическая шестерня: устранение микро-радиального пояска при $z \ge 36$ (`create_spur_gear`)
- **Проблема (`known_bug_spur_gear_high_teeth_cap_slivers`):**  
  При $z \ge 36$ (например, $z = 40$, $m = 1\,\text{мм}$) радиус основной окружности $r_b = r \cos 20^\circ \approx 18.794\,\text{мм}$ оказывается крайне близок к радиусу окружности впадин $r_f = r - 1.25m = 18.750\,\text{мм}$ ($\Delta r = 43.85\,\mu\text{m}$). Код безоговорочно вставлял отдельный радиальный уровень $r_b$ при $r_f < r_b - 10^{-9}$. Ширина зуба в этом месте составляла $\approx 1.47\,\text{мм}$, в результате чего образующийся квад высотой $44\,\mu\text{m}$ и шириной $1.47\,\text{мм}$ рассекался диагональю на треугольники с углом $\arctan(0.044 / 1.47) \approx 1.7^\circ < 5^\circ$ (160 треугольников на верхней и нижней крышках).
- **Решение:**  
  В `create_spur_gear` промежуточный уровень $r_b$ теперь вставляется только при условии, что расстояние от впадины до основной окружности составляет существенную долю от радиального шага зуба: $(r_b - r_f) \ge 0.35 \cdot \text{step\_r}$. Если $\Delta r$ меньше этого порога, то переход от $r_f$ к следующему эвольвентному уровню объединяется в один полноразмерный шаг, исключая появление узких полос.

### 1.4. Тест неполного вращения вокруг оси (`create_revolve`, $360^\circ$, $r_{\text{min}} = 0$)
- **Проблема (`known_bug_revolve_360_axis_fan_slivers_at_128_segments`):**  
  Ранее при $N = 128$ полюсный веер имел углы $360^\circ / 128 = 2.8125^\circ < 5^\circ$.
- **Решение:**  
  В рамках задачи P0-T5 в `create_revolve` уже было внедрено адаптивное 2-к-1 радиальное разрежение колец при приближении к оси $r = 0$, поэтому тест стал проходить. С теста `known_bug_revolve_360_axis_fan_slivers_at_128_segments` снят атрибут `#[ignore]`.

### 1.5. Колёсный диск: предотвращение микро-полосы на внутреннем ободе (`create_wheel_rim`)
- **Проблема (`known_bug_wheel_rim_low_spoke_count_slivers`):**  
  При малом числе спиц ($n_{\text{spokes}} \in \{3, 4\}$) и $n_{\text{bolts}} = 4$ сегмент 14 профиля обода (между $z_{\text{well\_top}}$ и $z_{\text{front}}$) сжимался до высоты $\Delta z \approx 2.15\,\text{мм}$ при длине хорды по окружности $45.8\,\text{мм}$. Квады на цилиндрической стенке внутреннего обода $r = r_{\text{inner\_barrel}}$ делились диагоналями на 48 треугольников с минимальным углом $3.20^\circ < 5^\circ$.
- **Решение:**  
  В `create_wheel_rim_with_settings` переменные сетки `q_mult`, `k_s`, `k_w`, `k_sec`, `n_theta` вынесены выше расчёта осевого положения диска, и введён гарантированный безопасный зазор `min_barrel_dz = (2.0 * PI * r_inner_barrel / (n_theta as f64) * 0.12).max(0.006)`. Положение центра диска `z_disc_center` ограничивается так, чтобы расстояние от торцов диска ($z_{\text{front}}, z_{\text{back}}$) до границ углубления ($z_{\text{well\_top}}, z_{\text{well\_bot}}$) всегда составляло не менее `min_barrel_dz` ($\ge 6.15\,\text{мм}$), что обеспечивает угол диагонали $\ge 7.65^\circ \gg 5^\circ$.

### 1.6. Разблокировка тестов в `primitive_invariants.rs`
- Сняты атрибуты `#[ignore]` со всех 5 оставшихся тестов `known_bug_*` в `crates/mcp-cad-core/tests/primitive_invariants.rs`.
- Все 18 тестов в `primitive_invariants.rs` теперь активны и проходят на 100%.

---

## 2. Как проверено

Команды проверены на Rust `1.91.1` (`x86_64-unknown-linux-gnu`):

### 2.1. Таргетный запуск бывших `known_bug_*` тестов
```bash
cargo test -p mcp-cad-core --test primitive_invariants known_bug
```
Результат:
```text
running 8 tests
test known_bug_cone_inverted_apex_missing_ring_halving ... ok
test known_bug_cone_sharp_auto_segments_odd_after_halving_slivers ... ok
test known_bug_cut_hole_csg_sliver_quality_gate ... ok
test known_bug_polygon_prism_chamfer_low_sides_slivers ... ok
test known_bug_revolve_360_axis_fan_slivers_at_128_segments ... ok
test known_bug_revolve_partial_angle_end_cap_slivers ... ok
test known_bug_spur_gear_high_teeth_cap_slivers ... ok
test known_bug_wheel_rim_low_spoke_count_slivers ... ok

test result: ok. 8 passed; 0 failed; 0 ignored; 0 measured; 10 filtered out; finished in 2.22s
```

### 2.2. Полный параметрический прогон `primitive_invariants.rs`
```bash
cargo test -p mcp-cad-core --test primitive_invariants
```
Результат:
```text
running 18 tests
test invariants_box_grid ... ok
test invariants_polygon_prism_chamfered_5_to_12_sides_grid ... ok
test invariants_polygon_prism_unchamfered_grid ... ok
test invariants_revolve_full_circle_grid ... ok
test invariants_cut_hole_topology_volume_and_aabb ... ok
test invariants_cylinder_grid ... ok
test invariants_wheel_rim_grid ... ok
test known_bug_cone_inverted_apex_missing_ring_halving ... ok
test known_bug_cone_sharp_auto_segments_odd_after_halving_slivers ... ok
test invariants_sphere_grid ... ok
test known_bug_polygon_prism_chamfer_low_sides_slivers ... ok
test known_bug_revolve_360_axis_fan_slivers_at_128_segments ... ok
test known_bug_revolve_partial_angle_end_cap_slivers ... ok
test known_bug_spur_gear_high_teeth_cap_slivers ... ok
test known_bug_wheel_rim_low_spoke_count_slivers ... ok
test known_bug_cut_hole_csg_sliver_quality_gate ... ok
test invariants_spur_gear_grid ... ok
test invariants_cone_sharp_and_frustum_grid ... ok

test result: ok. 18 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 68.93s
```

### 2.3. Полный прогон тестов всего рабочего пространства
```bash
cargo test --workspace --no-fail-fast
```
Результат:
- `mcp-cad-core --lib`: **50 passed; 0 failed; 0 ignored**
- `mcp-cad-core --test csg_invariants`: **5 passed; 0 failed; 0 ignored**
- `mcp-cad-core --test dag_tests`: **14 passed; 0 failed; 0 ignored**
- `mcp-cad-core --test primitive_invariants`: **18 passed; 0 failed; 0 ignored** (было: 13 passed, 5 ignored)
- `mcp-cad-core --test tessellation_tests`: **6 passed; 0 failed; 0 ignored**
- `mcp-cad-fea`: **11 passed** (5 lib + 6 fea_tests); 0 failed
- `mcp-cad-physics`: **6 passed**; 0 failed
- `mcp-cad-server`: **35 passed** (21 mcp_contract_tests + 13 qa_wide_scan + 1 gear_kinematic_test); 0 failed
- `mcp-cad-viewer`: **10 passed**; 0 failed
- **ИТОГО:** **155 passed; 0 failed; 0 ignored (100% активных тестов workspace без единого пропуска!)**

### 2.4. Сборка и статический анализ
- `npm run build` / `compile_applet`: успешен (`Build succeeded - the applet is compiled`).
- `npm run lint` (`tsc --noEmit`): успешен (0 ошибок).
- `cargo check --workspace --tests`: успешен (0 ошибок, 0 предупреждений).
- `cargo clippy -p mcp-cad-core --lib`: предупреждение `unused_mut` на строке 944 устранено (число предупреждений сократилось с 18 до 17).

---

## 3. Сравнение состояния тестов до и после P0-T10

| Набор тестов | До P0-T10 | После P0-T10 | Дельта |
|---|---|---|---|
| `primitive_invariants.rs` | 13 passed, 5 ignored | **18 passed, 0 ignored** | **+5 тестов разблокировано и проходят** |
| `mcp-cad-core --lib` | 50 passed | **50 passed** | Без регрессий |
| `csg_invariants.rs` | 5 passed | **5 passed** | Без регрессий |
| `dag_tests.rs` | 14 passed | **14 passed** | Без регрессий |
| `tessellation_tests.rs` | 6 passed | **6 passed** | Без регрессий |
| `mcp-cad-server` (все тесты) | 35 passed | **35 passed** | Без регрессий |
| `mcp-cad-fea`, `physics`, `viewer` | 27 passed | **27 passed** | Без регрессий |
| **Всего по Rust workspace** | **150 passed, 5 ignored** | **155 passed, 0 ignored** | **100% тестов проходят, 0 падений, 0 пропусков** |

---

## 4. Что не удалось и остаточные риски

- Все краевые случаи генерации геометрии примитивов исправлены конформно, без ослабления порогов Sliver Quality Gate (`min_angle >= 1.0°`, доля `< 5.0°` не более `1.0%`).
- Риски регрессий отсутствуют: проверена неизменность топологии, объема и ориентации нормалей.

---

## 5. Побочные находки

1. В `crates/mcp-cad-core/src/primitives.rs:944` устранён `#[warn(unused_mut)]` благодаря использованию `curr_segs` в цикле разрежения перевёрнутого конуса.
2. В файле `TASKS.md` следующей по порядку задачей в статусе `📋 BACKLOG` становится **P0-T11** (`Скрытый сбой create_joint на детали без геометрии и ложноположительный assert в test_bug015`).
