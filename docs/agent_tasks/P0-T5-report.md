# Отчёт о выполнении задачи P0-T5

**Задача:** P0-T5 — Игольчатые треугольники: частичный revolve  
**Дата:** 04.11.2026  
**Статус:** ✅ ВЫПОЛНЕНО

---

## 1. Что сделано (файлы, функции)

1. **Устранение масштабной зависимости и выбор оптимального уха в `triangulate_simple_polygon_2d` (`crates/mcp-cad-core/src/primitives.rs`)**:
   - Выявлена точная причина образования игольчатых треугольников (`< 1°`) на плоских торцевых крышках `create_revolve`: функция `is_point_in_tri` ранее использовала размерный порог `c1 >= -1e-9 && c2 >= -1e-9 && c3 >= -1e-9` для 2D векторных произведений (имеющих размерность удвоенной площади $\text{м}^2$). Для миллиметровых и сантиметровых профилей (`r ~ 0.01..0.03` м) с подразделёнными коллинеарными рёбрами удвоенная площадь треугольника составляет `~1e-5..1e-8` $\text{м}^2$, поэтому коллинеарные вершины границы вне уха попадали в полосу `[-1e-9, 0]` и ложно классифицировались как лежащие внутри уха. Это блокировало все допустимые уши и приводило к срабатыванию аварийного сброса `stuck_count >= len`, создававшего вырожденные коллинеарные треугольники `[indices[0], indices[1], indices[2]]` (`min_angle = 0.9076°` в `test_bug_003` и `0.3649°` в `known_bug_revolve_partial_angle_end_cap_slivers`).
   - Проверка попадания вершин внутрь уха переведена на безразмерные барицентрические координаты `b1 = c1 / area2`, `b2 = c2 / area2`, `b3 = c3 / area2` с масштабно-инвариантным допуском `-1e-7`, а также добавлена явная проверка открытой хорды `(pa, pc)` на отсутствие коллинеарных вершин полигона на внутреннем диагональном ребре.
   - Вместо жадного отсечения первого попавшегося уха реализован выбор уха с максимальным минимальным углом треугольника (`min_triangle_angle_2d`), после чего к полученной 2D-триангуляции применяется итеративная оптимизация внутренних диагоналей `optimize_2d_delaunay_edges` (Lawson flip по точному котангенсному критерию окружности Делоне $\cot \alpha_w + \cot \alpha_x < -10^{-6}$ и строгому увеличению минимального угла пары треугольников).

2. **Локальное сгущение и перестройка полости Делоне (Bowyer–Watson) для торцевых крышек `triangulate_cap_polygon_2d` (`crates/mcp-cad-core/src/primitives.rs`)**:
   - Для узких углов частичного вращения (например, `30°` при `segments = 32`) внутренние вертикальные стенки профиля имеют малый окружной шаг $c_{\min} = r \cdot \Delta\theta$, что требует мелкого осевого подразбиения вертикальных рёбер (`n_sub` до 28 сегментов) ради соблюдения качества четырёхугольников боковой поверхности. Чтобы узкие граничные сегменты крышки не тянули длинные веерные диагонали через всю ширину профиля (что давало углы `~4.09° < 5°`), добавлена функция `triangulate_cap_polygon_2d`.
   - При наличии в триангуляции крышки треугольников с углом `< 5.5°` функция вставляет внутренние точки Штейнера (центроид треугольника или смещённую внутрь нормали середину длинной хорды у коллинеарной границы) методом звёздчатой полости описанных окружностей Бойера–Уотсона (`Bowyer-Watson star-shaped circumcircle cavity`), удаляя все смежные узкие треугольники полости и перестраивая триангуляцию с последующим прогоном `optimize_2d_delaunay_edges`. Поскольку вершины границы полигона не изменяются, крышка остаётся на 100% конформной (watertight & manifold) с боковыми гранями вращения.

3. **Геометрическое разрежение (2-to-1 halving) колец вращения у оси `r = 0` в `create_revolve` (`crates/mcp-cad-core/src/primitives.rs`)**:
   - Если профиль касается оси вращения (`r = 0`), сведение всех `segs = 32` угловых секторов непосредственно в вершину на оси при `angle_deg = 30°` или `90°` неизбежно даёт углы при вершине $\Delta\theta = \theta / \text{segs}$ (`2.81° < 5°` при `90°` и `0.94° < 1°` при `30°`).
   - В `create_revolve` для рёбер профиля, примыкающих к оси вращения (`r = 0`), при малом эффективном угловом шаге (`dtheta_eff < 0.18` рад $\approx 10.3^\circ$) реализованы переходы 2-к-1 (`curr_segs -> (curr_segs + 1) / 2`) по мере приближения к `r = 0`, поддерживающие как чётное, так и нечётное число сегментов кольца, при сохранении исходного числа сегментов `segs` на всех внешних рёбрах (`r > 0`).

4. **Добавление unit-теста по теореме Паппа–Гульдина и разблокировка теста инвариантов (`crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/tests/primitive_invariants.rs`)**:
   - В `crates/mcp-cad-core/src/primitives.rs` добавлен unit-тест `test_revolve_partial_angles_validation_and_pappus_volume`, проверяющий 4 профиля (прямоугольный полый `rect_hollow`, прямоугольный сплошной до оси `rect_solid`, Г-образный полый `l_hollow`, Г-образный сплошной до оси `l_solid`) на углах `30°`, `90°`, `180°`, `270°` и `360°`:
     - `validate_mesh` возвращает `status == "PASS"`, `min_angle >= 1.0°`, `lt_1_deg_count == 0`, `lt_5_deg_ratio < 0.01`;
     - `signed_volume()` совпадает с аналитическим объёмом по теореме Паппа $V = \theta \cdot r_c \cdot A$ с точностью `< 1%` (и с полигональной формулой $V_{\text{poly}} = V \cdot \frac{\sin\Delta\theta}{\Delta\theta}$ с точностью `< 1e-6`).
   - В `crates/mcp-cad-core/tests/primitive_invariants.rs` снят атрибут `#[ignore]` с теста `known_bug_revolve_partial_angle_end_cap_slivers`.
   - Пороги `Sliver quality gate` (`MAX_ALLOWED_SLIVER_RATIO = 0.01`, запрет треугольников `< 1°`) не ослаблялись.

---

## 2. Как проверено (команды и итоговые строки `test result`)

- **Воспроизведение дефектов до правок:**
  - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test qa_wide_scan test_bug_003_partial_revolve_mesh_and_renders -- --exact`
  - Результат до правок: `test result: FAILED. 0 passed; 1 failed; 0 ignored; 0 measured; 12 filtered out; finished in 0.89s` (`Partial revolve 90 deg CCW failed: ["Sliver quality gate failed: 2 needle triangles with min angle < 1° (min_angle=0.9076°)"]`).
  - Команда: `cargo test -p mcp-cad-core --test primitive_invariants known_bug_revolve_partial_angle_end_cap_slivers -- --ignored --exact`
  - Результат до правок: `test result: FAILED. 0 passed; 1 failed; 0 ignored; 0 measured; 17 filtered out; finished in 0.16s` (`Partial revolve 90° failed validate_mesh: ["Sliver quality gate failed: 10 needle triangles with min angle < 1° (min_angle=0.3649°)", "Sliver quality gate failed: 10.07% of triangles (182/1808) have min angle < 5° (threshold 1.00%)"]`).

- **Проверка после правок:**
  - Новый unit-тест по теореме Паппа и углам `30°, 90°, 180°, 270°, 360°` (прямоугольный и Г-образный профили):
    - Команда: `cargo test -p mcp-cad-core --lib test_revolve_partial_angles_validation_and_pappus_volume -- --nocapture`
    - Результат: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 48 filtered out; finished in 2.83s`
  - Целевой интеграционный тест `qa_wide_scan::test_bug_003_partial_revolve_mesh_and_renders`:
    - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test qa_wide_scan test_bug_003_partial_revolve_mesh_and_renders`
    - Результат: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 12 filtered out; finished in 5.65s`
  - Набор параметрических инвариантов примитивов `primitive_invariants` (включая разблокированный `known_bug_revolve_partial_angle_end_cap_slivers`):
    - Команда: `cargo test -p mcp-cad-core --test primitive_invariants`
    - Результат: `test result: ok. 11 passed; 0 failed; 7 ignored; 0 measured; 0 filtered out; finished in 4.30s`
  - Сборка и проверка фронтенда (`compile_applet` / `npm run build`):
    - Результат: `built in 5.56s` (0 ошибок).

---

## 3. Тесты до / после

| Таргет / Тест | До P0-T5 | После P0-T5 |
|---|---|---|
| `qa_wide_scan::test_bug_003_partial_revolve_mesh_and_renders` | ❌ FAILED (`0 passed; 1 failed`) | ✅ PASS (`1 passed; 0 failed`) |
| `primitive_invariants::known_bug_revolve_partial_angle_end_cap_slivers` | ❌ FAILED (`#[ignore]`, `10 needles < 1°`) | ✅ PASS (снят `#[ignore]`, `11 passed; 0 failed`) |
| `primitives::tests::test_revolve_partial_angles_validation_and_pappus_volume` | — (не существовал) | ✅ PASS (`1 passed; 0 failed`) |
| Workspace всего (`cargo test --workspace --no-fail-fast`) | `137 passed; 3 failed; 8 ignored` (148 всего) | **`140 passed; 2 failed; 7 ignored` (149 всего)** |

Два оставшихся падения во всём Rust Workspace находятся в `qa_wide_scan.rs` и относятся к следующим задачам очереди P0:
1. `test_bug_005_polygon_prism_chamfer_and_sides_validation` $\to$ **P0-T6**
2. `test_bug_006_csg_booleans_and_holes_watertight` $\to$ **P0-T7**

---

## 4. Что не удалось и остаточные риски

- Все критерии приёмки **P0-T5** выполнены в полном объёме без ослабления порогов `Sliver quality gate`.
- Остаточный риск: для крайне самопересекающихся или сверхтонких невыпуклых профилей с десятками коллинеарных подсегментов число итераций вставки точек Штейнера в `triangulate_cap_polygon_2d` ограничено сверху (`max_steiner = 96`), чтобы гарантировать детерминированное время работы.

---

## 5. Побочные находки (не чинить в рамках P0-T5, только перечислить)

1. **Положительный побочный эффект для P0-T10 (`known_bug_revolve_360_axis_fan_slivers_at_128_segments`)**:
   - Благодаря добавленному в `create_revolve` 2-к-1 разрежению колец вращения при приближении к оси `r = 0`, отложенный тест `known_bug_revolve_360_axis_fan_slivers_at_128_segments` из задачи **P0-T10** (`cargo test -p mcp-cad-core --test primitive_invariants known_bug_revolve_360_axis_fan_slivers_at_128_segments -- --ignored --exact`) теперь также **проходит** (`1 passed; 0 failed`). В соответствии с правилом «одна задача за раз» снятие `#[ignore]` с этого теста и остальных 4 тестов `P0-T10` оставлено для задачи **P0-T10**.
