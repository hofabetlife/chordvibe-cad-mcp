# Отчёт о выполнении задачи P0-T1

**Задача:** P0-T1 — Исправить ориентацию торцевых крышек (цилиндр, конус, шестерня, обод)  
**Дата:** 04.11.2026  
**Статус:** ✅ ВЫПОЛНЕНО

---

## 1. Что сделано (файлы, функции)

В файле `crates/mcp-cad-core/src/primitives.rs`:
1. **`add_bridged_ring_strip_2d` (строки 174–179):**
   - Исправлен порядок передачи вершин четырёхугольника в `push_quad_shortest_diag`.
   - Так как `ring_a` (`prev_ring`) — внешнее кольцо радиуса $R_{\text{out}}$, а `ring_b` (`curr_ring`) — внутреннее кольцо радиуса $R_{\text{in}} < R_{\text{out}}$, и оба кольца упорядочены против часовой стрелки (CCW) в плоскости $XY$ при взгляде с $+Z$, четвёрка вершин `[p0, p1, p2, p3] = [a_i, a_{i+1}, b_{i+1}, b_i]` образует контур против часовой стрелки в плоскости $XY$: векторное произведение $(p_1 - p_0) \times (p_3 - p_0) \approx (+\hat{\theta}) \times (-\hat{r}) = +\hat{z}$ уже направлено вдоль **$+Z$**.
   - Ранее для `is_top == true` порядок ошибочно разворачивался в `(p0, p3, p2, p1)`, из-за чего верхние крышки получали нормаль $-Z$ (внутрь тела), а нижние (`is_top == false`) — $+Z$ (также внутрь тела). Теперь при `is_top == true` передаётся `(p0, p1, p2, p3)`, а при `is_top == false` — `(p0, p3, p2, p1)`.
2. **`add_halving_ring_transition_2d` (строки 207–211):**
   - Исправлен порядок индексов в замыкании `push_tri(a, b, c)` для переходов 2-к-1 между внешним кольцом `outer_ring` ($2k$ вершин) и внутренним кольцом `inner_ring` ($k$ вершин).
   - Тройки `(o0, o1, i0)`, `(o1, i1, i0)` и `(o1, o2, i1)` при обходе колец против часовой стрелки в плоскости $XY$ уже ориентированы против часовой стрелки (нормаль $+Z$).
   - Теперь для верхней крышки (`is_top == true`) записывается `[base, base + 1, base + 2]`, а для нижней (`is_top == false`) — `[base, base + 2, base + 1]`.
3. **Модульные тесты `cap_orientation_*` в `primitives::tests` (строки 3089–3244):**
   - Добавлена вспомогательная функция `verify_cap_normals_and_validation` и 4 модульных теста:
     - `cap_orientation_cylinder`
     - `cap_orientation_cone_sharp`
     - `cap_orientation_cone_frustum`
     - `cap_orientation_spur_gear`
   - Каждый тест проверяет, что геометрическая нормаль $\vec{n}_{\text{geo}} = (p_1 - p_0) \times (p_2 - p_0)$ каждого треугольника верхней крышки имеет $\vec{n}_{\text{geo}} \cdot (+\hat{z}) > 0$, нижней крышки — $\vec{n}_{\text{geo}} \cdot (-\hat{z}) > 0$, `validate_mesh` возвращает `status == "PASS"` и `inconsistent_edges_count == 0`, а `signed_volume()` положителен и отличается от аналитического объёма менее чем на $1\%$.

---

## 2. Как проверено (команды и итоговые строки `test result`)

- **Модульные тесты `mcp-cad-core` (`cargo test -p mcp-cad-core --lib`):**
  ```text
  test result: ok. 48 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 19.86s
  ```
- **Интеграционные тесты DAG (`cargo test -p mcp-cad-core --test dag_tests`):**
  ```text
  test result: ok. 14 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 23.96s
  ```
- **Тесты адаптивной тесселяции (`cargo test -p mcp-cad-core --test tessellation_tests`):**
  ```text
  test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.08s
  ```
- **Тесты МКЭ, физики и рендера (`WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-fea -p mcp-cad-physics -p mcp-cad-viewer`):**
  - `mcp-cad-fea --lib`: `test result: ok. 5 passed; 0 failed`
  - `mcp-cad-fea --test fea_tests`: `test result: ok. 6 passed; 0 failed`
  - `mcp-cad-physics --test physics_tests`: `test result: ok. 6 passed; 0 failed`
  - `mcp-cad-viewer --test viewer_tests`: `test result: ok. 10 passed; 0 failed`
- **Тесты сервера (`WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --no-fail-fast`):**
  - `gear_kinematic_test`: `test result: ok. 1 passed; 0 failed`
  - `mcp_contract_tests`: `test result: FAILED. 20 passed; 1 failed; 0 ignored`
  - `qa_wide_scan`: `test result: FAILED. 10 passed; 3 failed; 0 ignored`

### Способ измерения качества треугольников крышек (критерий 4)
Качество треугольников проверялось через `crate::diagnostics::validate_mesh(mesh, None)` (поля `geometry.lt_5_deg_count`, `geometry.lt_5_deg_ratio`, `geometry.lt_1_deg_count` и `angle_stats.min_angle_deg`). Поскольку исправление в `add_halving_ring_transition_2d` меняет только циклическую перестановку индексов треугольника `[base, base+2, base+1] -> [base, base+1, base+2]`, а в `add_bridged_ring_strip_2d` перестановка `(p0, p3, p2, p1) -> (p0, p1, p2, p3)` сохраняет обе диагонали `|p0 - p2|` и `|p1 - p3|` в `push_quad_shortest_diag`, длины всех сторон и внутренние углы каждого треугольника крышек остались **математически идентичными** до и после исправления (`lt_5_deg_count == 0`, `lt_1_deg_count == 0` на цилиндре, остром конусе, усечённом конусе и прямозубой шестерне).

---

## 3. Тесты до / после

| Набор тестов | До P0-T1 (Passed / Failed) | После P0-T1 (Passed / Failed) | Изменение |
|---|---|---|---|
| `mcp-cad-core --lib` | 38 / **6** | **48** / **0** | +6 починено, +4 новых теста `cap_orientation_*` |
| `mcp-cad-core --test dag_tests` | 10 / **4** | **14** / **0** | +4 починено |
| `mcp-cad-core --test tessellation_tests` | 5 / **1** | **6** / **0** | +1 починено |
| `mcp-cad-fea` (все) | 11 / 0 | 11 / 0 | Без изменений (100% PASS) |
| `mcp-cad-physics` (все) | 6 / 0 | 6 / 0 | Без изменений (100% PASS) |
| `mcp-cad-viewer` (все) | 10 / 0 | 10 / 0 | Без изменений (100% PASS) |
| `mcp-cad-server --test gear_kinematic_test` | 1 / 0 | 1 / 0 | Без изменений (100% PASS) |
| `mcp-cad-server --test mcp_contract_tests` | 19 / **2** | **20** / **1** | +1 починено (`test_mcp_optimize_feature_tree_cbo`) |
| `mcp-cad-server --test qa_wide_scan` | 7 / **6** | **10** / **3** | +3 починено (`test_bug_002`, `test_bug_004`, `test_bug_007`) |
| **ИТОГО по Rust Workspace** | **107 / 19** (из 126) | **126 / 4** (из 130) | **Починено 15 из 19 падавших тестов; 0 регрессий** |

### Полный список 15 исправленных тестов:
1. `mcp-cad-core --lib`: `diagnostics::tests::test_validate_mesh_cylinder_pass`
2. `mcp-cad-core --lib`: `primitives::tests::test_cylinder_volume`
3. `mcp-cad-core --lib`: `primitives::tests::test_cone_sharp_volume`
4. `mcp-cad-core --lib`: `primitives::tests::test_cone_frustum_volume`
5. `mcp-cad-core --lib`: `primitives::tests::test_spur_gear_solid_volume_and_dimensions`
6. `mcp-cad-core --lib`: `query_optimizer::tests::test_r4_coaxial_bushing_cylinder_hole_to_revolve`
7. `mcp-cad-core --test dag_tests`: `test_primitive_volumes_analytical_verification`
8. `mcp-cad-core --test dag_tests`: `test_dag_cone_and_frustum_volume`
9. `mcp-cad-core --test dag_tests`: `test_dag_shifted_cylinder_stepped_shaft`
10. `mcp-cad-core --test dag_tests`: `test_dag_helical_groove_drill_bit_model`
11. `mcp-cad-core --test tessellation_tests`: `test_cylinder_adaptive_tessellation_density_and_convergence`
12. `mcp-cad-server --test mcp_contract_tests`: `test_mcp_optimize_feature_tree_cbo`
13. `mcp-cad-server --test qa_wide_scan`: `test_bug_002_spur_gear_manifold_and_keyway`
14. `mcp-cad-server --test qa_wide_scan`: `test_bug_004_wheel_rim_manifold_and_renders`
15. `mcp-cad-server --test qa_wide_scan`: `test_bug_007_negative_scale_preserves_outward_winding`

---

## 4. Тесты, которые всё ещё падают после исправления (вход для P0-T3), и остаточные риски

После выполнения **P0-T1** в workspace остаётся ровно **4 падающих теста** (все 4 не связаны с ориентацией крышек цилиндра/конуса/шестерни/обода и относятся к последующим задачам Фазы 0):

1. `mcp_contract_tests::test_bug016_revolve_boundary_and_self_intersection_validation` — тест проверяет наличие верхнеуровневого поля JSON-RPC `resp.get("error")`, тогда как сервер по конвенции MCP возвращает ошибки инструментов внутри `result` с флагом `result.isError = true` (задача **P0-T4**).
2. `qa_wide_scan::test_bug_003_partial_revolve_mesh_and_renders` — падает на проверке качества сетки частичного вращения на 90° (`Sliver quality gate failed: 2 needle triangles with min angle < 1° (min_angle=0.9076°)` в триангуляции плоских торцевых крышек `create_revolve`, задача **P0-T5**).
3. `qa_wide_scan::test_bug_005_polygon_prism_chamfer_and_sides_validation` — падает на трёхгранной призме с фаской (`Chamfered prism 3 sides failed: 50.00% of triangles (12/24) have min angle < 5°` в `create_polygon_prism`, задача **P0-T6**).
4. `qa_wide_scan::test_bug_006_csg_booleans_and_holes_watertight` — падает на сквозном отверстии в пластине (`Thru hole failed: 85 needle triangles with min angle < 1° (min_angle=0.2119°), 11.53% of triangles (136/1180) have min angle < 5°` при разбиении полигонов в BSP-CSG `csg.rs`, задача **P0-T7**).

---

## 5. Побочные находки (не исправлялись в рамках P0-T1)

1. **Неточность рекомендации в `VERIFICATION_REPORT.md` по `create_wheel_rim`:** В `VERIFICATION_REPORT.md` (раздел 3.1, BLOCKER-2) предполагалось, что в `create_wheel_rim` (строки 2603–2607) кольца `perim_loop`, `mid_loop` и `hole_loop` передаются в обратном порядке и требуют инверсии флага `is_top`. Проверка показала, что `perim_loop` является внешним контуром сектора вокруг центра болтового отверстия `bolt_center` (обход CCW), а `mid_loop` и `hole_loop` — внутренними контурами (также CCW вокруг `bolt_center`), то есть конвенция аргументов полностью совпадает с остальными вызовами. После исправления `add_bridged_ring_strip_2d` и `add_halving_ring_transition_2d` тест `test_bug_004_wheel_rim_manifold_and_renders` сразу прошёл на 100% без правок `create_wheel_rim`.
2. **Самоисправление каскадных тестов CSG и CBO (P0-T9):** После починки ориентации крышек в P0-T1 автоматически начали проходить оба теста оптимизатора DAG (`query_optimizer::tests::test_r4_coaxial_bushing_cylinder_hole_to_revolve` и `mcp_contract_tests::test_mcp_optimize_feature_tree_cbo`), а также оба каскадных DAG-теста (`test_dag_shifted_cylinder_stepped_shaft` и `test_dag_helical_groove_drill_bit_model`), так как ранее кандидат правила `R4` и булевы операции отвергались/ломались из-за отрицательного вклада вывернутых крышек цилиндра в объём и проверку манифолдности. В рамках **P0-T9** останется добавить негативный тест на неосевое отверстие.
3. **Предупреждения компилятора (`cargo check`):** В `crates/mcp-cad-core` присутствуют 3 предупреждения (`unused_mut` на `primitives.rs:930`, `unused_assignments` на `query_optimizer.rs:289`, `unused_variables` на `query_optimizer.rs:1217`), которые запланированы к устранению в задаче **P0-T8**.
