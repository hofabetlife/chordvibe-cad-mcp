# Отчёт о выполнении задачи P0-T3

**Задача:** P0-T3 — Триаж остаточных падений (только анализ, без правок кода)  
**Дата:** 04.11.2026  
**Статус:** ✅ ВЫПОЛНЕНО

---

## 1. Что сделано (файлы, функции)

1. Выполнен полный прогон тестового суита Rust Workspace (`cargo test --workspace --no-fail-fast`) после применения исправлений **P0-T1** и добавления параметрических инвариантов **P0-T2**.
2. Проведён детальный анализ кода и трассировка первопричин каждого из остаточных падений, а также проверены все кандидаты из спецификации **P0-T3** (`test_dag_shifted_cylinder_stepped_shaft`, `test_dag_helical_groove_drill_bit_model`, `test_r4_coaxial_bushing_cylinder_hole_to_revolve`, `test_mcp_optimize_feature_tree_cbo`, `test_bug016_revolve_boundary_and_self_intersection_validation`, `test_bug_003`, `test_bug_005`, `test_bug_006`) и 8 отложенных тестов `known_bug_*` из `primitive_invariants.rs`.
3. Создан аналитический документ [`docs/TRIAGE_AFTER_P0T1.md`](../TRIAGE_AFTER_P0T1.md), содержащий:
   - полную сводку прогона по всем таргетам workspace (148 тестов: 136 passed, 4 failed, 8 ignored);
   - доказательство каскадной природы 4 кандидатов (`test_dag_shifted_cylinder_stepped_shaft`, `test_dag_helical_groove_drill_bit_model`, `test_r4_coaxial_bushing_cylinder_hole_to_revolve`, `test_mcp_optimize_feature_tree_cbo`), которые автоматически починились после исправления ориентации крышек в **P0-T1**;
   - минимальные команды воспроизведения, точные сообщения паники и доказанные первопричины в коде для всех **4 остаточных падающих тестов**;
   - классификацию всех 8 тестов `known_bug_*` из `primitive_invariants.rs` и спецификацию новой задачи **P0-T10** для 5 краевых случаев качества сетки примитивов, не покрытых задачами P0-T4…T9.
4. Исходный код крейтов и тесты не изменялись (согласно ограничению задачи **P0-T3** «Не править исходный код и тесты»).

---

## 2. Как проверено (команды и итоговые строки `test result`)

- **Полный прогон Workspace (`WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test --workspace --no-fail-fast`):**
  - `mcp-cad-core --lib`: `test result: ok. 48 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 19.86s`
  - `mcp-cad-core --test dag_tests`: `test result: ok. 14 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 23.96s`
  - `mcp-cad-core --test primitive_invariants`: `test result: ok. 10 passed; 0 failed; 8 ignored; 0 measured; 0 filtered out; finished in 20.10s`
  - `mcp-cad-core --test tessellation_tests`: `test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.08s`
  - `mcp-cad-fea --lib`: `test result: ok. 5 passed; 0 failed; 0 ignored`
  - `mcp-cad-fea --test fea_tests`: `test result: ok. 6 passed; 0 failed; 0 ignored`
  - `mcp-cad-physics --test physics_tests`: `test result: ok. 6 passed; 0 failed; 0 ignored`
  - `mcp-cad-viewer --test viewer_tests`: `test result: ok. 10 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 14.69s`
  - `mcp-cad-server --test gear_kinematic_test`: `test result: ok. 1 passed; 0 failed; 0 ignored`
  - `mcp-cad-server --test mcp_contract_tests`: `test result: FAILED. 20 passed; 1 failed; 0 ignored; 0 measured; 0 filtered out; finished in 30.32s`
  - `mcp-cad-server --test qa_wide_scan`: `test result: FAILED. 10 passed; 3 failed; 0 ignored; 0 measured; 0 filtered out; finished in 33.30s`

---

## 3. Тесты до / после (и классификация 4 остаточных падений)

Так как задача **P0-T3** — чисто аналитическая (без модификации кода и тестов), количество проходящих и падающих тестов до и после P0-T3 идентично (`136 passed, 4 failed, 8 ignored` из `148`).

| № | Падающий тест | Точное сообщение | Первопричина (кратко) | Целевая задача |
|---|---|---|---|---|
| 1 | `mcp_contract_tests::test_bug016_revolve_boundary_and_self_intersection_validation` | `Revolve with r < 0 must return an error` (`mcp_contract_tests.rs:2440`) | Валидация в `tools/cad.rs:305–388` работает и возвращает `Err(ServerError::InvalidArgument)`, который `protocol.rs:224` упаковывает в `result.isError = true`, но тест проверяет верхнеуровневое поле `resp.get("error").is_some()`. | **P0-T4** |
| 2 | `qa_wide_scan::test_bug_003_partial_revolve_mesh_and_renders` | `Partial revolve 90 deg CCW failed: ["Sliver quality gate failed: 2 needle triangles with min angle < 1° (min_angle=0.9076°)"]` | В `create_revolve` (`primitives.rs:2031–2043`) стороны 2D-профиля подразделяются коллинеарными точками, после чего `triangulate_simple_polygon_2d` (`primitives.rs:1896–1969`) жадно отрезает уши без выбора по минимальному углу и без Delaunay-флипа, создавая веер узких диагоналей и иглы `< 1°`. | **P0-T5** |
| 3 | `qa_wide_scan::test_bug_005_polygon_prism_chamfer_and_sides_validation` | `Chamfered prism 3 sides failed: ["Sliver quality gate failed: 50.00% of triangles (12/24) have min angle < 5° (threshold 1.00%)"]` | В `create_polygon_prism` (`primitives.rs:1180–1202`) каждая наклонная грань фаски (отношение длины стороны к ширине фаски $\approx 20:1$ при `sides=3, waf=0.04, chamfer=0.003`) режется одной диагональю без подразбиения вдоль стороны, давая 12 из 24 треугольников с углом $\approx 2.86^\circ < 5^\circ$. | **P0-T6** |
| 4 | `qa_wide_scan::test_bug_006_csg_booleans_and_holes_watertight` | `Thru hole failed: ["Sliver quality gate failed: 85 needle triangles with min angle < 1° (min_angle=0.2119°)", "Sliver quality gate failed: 11.53% of triangles (136/1180) have min angle < 5° (threshold 1.00%)"]` | В `csg.rs` секущие плоскости граней цилиндра отверстия рассекают широкие грани пластины на узкие полосы, а `triangulate_subdivided_convex_loop` (`csg.rs:721`) и 4-проходный `optimize_coplanar_delaunay_edges` (`csg.rs:592`) соединяют мелкие сегментов дуги отверстия ($r=0.015$) напрямую с далёкими углами пластины ($0.05$) без промежуточного переходного кольца. | **P0-T7** |

---

## 4. Что не удалось и остаточные риски

- Все цели триажа **P0-T3** выполнены в полном объёме.
- **Остаточный риск для P0-T7:** Устранение игольчатых треугольников на плоских гранях, рассечённых многогранным отверстием в BSP-CSG (`csg.rs`), не может быть решено только увеличением числа проходов `optimize_coplanar_delaunay_edges`, так как вершины лежат только на малой окружности отверстия и на внешнем периметре квадрата пластины (любая триангуляция без внутренних вершин между 32 точками круга $r=0.015$ и 4 углами квадрата $0.1 \times 0.1$ геометрически обязана содержать узкие треугольники с углом $\approx 360^\circ/32 \cdot (r / L) < 5^\circ$). В **P0-T7** потребуется либо локальное подразделение/вставка переходного кольца (Steiner vertices) на копланарных гранях вокруг выреза, либо предварительное разбиение крупных граней перед/после BSP.

---

## 5. Побочные находки (не чинить, только перечислить)

1. В `crates/mcp-cad-core/src/query_optimizer.rs:1220` проверка эквивалентности `equivalence_ok` после перезаписи DAG проверяет только `vol_delta_pct < 0.5` и сохранение `is_watertight`, но не проверяет отсутствие деградации `validate_mesh` (например, `orientation_consistent` или `status == "PASS"`). Это должно быть учтено при выполнении **P0-T9**.
2. В `crates/mcp-cad-core/tests/primitive_invariants.rs` 5 из 8 `#[ignore]` тестов `known_bug_*` обнаруживают краевые нарушения `Sliver quality gate` в генераторах примитивов (`create_cone` при `segments=0` и при $r_{\text{bot}}=0$, `create_spur_gear` при `teeth >= 36`, `create_revolve` 360° у оси при `segments = 128`, `create_wheel_rim` при `spokes ∈ {3, 4}`). Для них в `docs/TRIAGE_AFTER_P0T1.md` оформлено предложение новой задачи **P0-T10**.
