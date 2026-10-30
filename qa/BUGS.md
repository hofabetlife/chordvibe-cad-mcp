# MCP-CAD Engine: Registry of Bugs & Discrepancies

| ID | Sev | Title | Component | Status | Discovered |
|---|---|---|---|---|---|
| BUG-001 | S4 | ARCHITECTURE.md indicates 15 tools instead of 26 | docs | OPEN | 2026-10-30 |
| BUG-002 | S2 | spur_gear: collinear origin-fan degenerate triangles when r_f < r_b and non-watertight CSG hub/bore/keyway | core/primitives | OPEN | 2026-10-30 |
| BUG-003 | S2 | revolve: partial revolution (<360 deg) end-caps have inverted triangle winding | core/primitives | OPEN | 2026-10-30 |
| BUG-004 | S2 | wheel_rim: non-watertight mesh with protruding spokes and 17956 boundary edges | core/primitives | OPEN | 2026-10-30 |
| BUG-005 | S2 | polygon_prism: chamfer fails watertightness on 6/8 sides, is ignored on 3/4 sides, and sides < 3 not rejected | core/primitives & dag | OPEN | 2026-10-30 |
| BUG-006 | S2 | csg: BSP boolean operations and cut_hole/cut_helical_groove drop small meter-scale triangles and leave T-junction boundary edges | core/csg | OPEN | 2026-10-30 |
| BUG-007 | S2 | topology: TriangleMesh::scale with negative scale factor inverts winding and produces negative signed volume | core/topology | OPEN | 2026-10-30 |
| BUG-008 | S2 | cad tools: failed rebuild leaves invalid FeatureNode in part DAG, poisoning subsequent operations | server/tools/cad | OPEN | 2026-10-30 |
| BUG-009 | S2 | batch_cad_operations: partial failure does not roll back state mutations from earlier steps | server/tools/cad | OPEN | 2026-10-30 |
| BUG-010 | S3 | open_preview_window: does not validate part_id existence when running in headless mode | server/tools/render | OPEN | 2026-10-30 |

---

## BUG-001 [S4][OPEN] ARCHITECTURE.md: устаревшее упоминание 15 инструментов вместо 26
- Воспроизведение: Прочитать `docs/ARCHITECTURE.md` строки 7 и 33 (`15 инструментов MCP`).
- Ожидание: 26 инструментов MCP согласно PRD, коду и `docs/MCP_API_REFERENCE.md`.
- Факт: Написано 15 инструментов.
- Артефакты: `docs/ARCHITECTURE.md`
- Гипотеза причины: Документ не был обновлен при расширении реестра инструментов до 26.
- Тест-регрессия: Проверка консистентности документации.
- Статус: OPEN

---

## BUG-002 [S2][OPEN] spur_gear: вырожденные коллинеарные треугольники крышек при r_f < r_b и негерметичность со ступицей/отверстием/шпонпазом
- Воспроизведение: `create_spur_gear(0.002, 20, 20.0, 0.015, ...)` (plain и с hub/bore/keyway для 8, 20, 60 зубьев).
- Ожидание: `status == "PASS"`, `watertight == true`, `degenerate_triangles == 0`, `self_intersections == 0`, `genus == 0` (plain) и `genus == 1` (с отверстием).
- Факт: При `r_f < r_b` радиальные участки ножки зуба лежат на лучах из `[0, 0]`, создавая $4 \times z$ вырожденных коллинеарных треугольников на верхней и нижней крышках; при добавлении ступицы и отверстия через CSG возникают тысячи граничных ребер (дыр) и T-junctions.
- Артефакты: `qa/results/primitives_mesh_scan.json`
- Гипотеза причины: `create_spur_gear_with_settings` в `crates/mcp-cad-core/src/primitives.rs` строит веер от `[0, 0, ±h2]` к радиальным ребрам и использует несшитый CSG для ступицы и отверстия со шпонпазом.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-003 [S2][OPEN] revolve: несогласованная ориентация треугольников на торцевых крышках частичного вращения (<360°)
- Воспроизведение: `create_revolve(&[[0.01, -0.02], [0.03, -0.02], [0.03, 0.02], [0.01, 0.02]], 180.0, 32)`
- Ожидание: `watertight == true`, `orientation_consistent == true`, 0 inverted edges.
- Факт: `orientation_consistent == false`, 8 инвертированных направленных ребер на торцевых крышках среза 180°.
- Артефакты: `qa/results/primitives_mesh_scan.json`
- Гипотеза причины: В `create_revolve` (`primitives.rs` строки 1548 и 1556) порядок обхода вершин стартовой крышки (`[i0, i2, i1]`) и конечной крышки (`[i0, i1, i2]`) противоположен ориентации боковых стенок.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-004 [S2][OPEN] wheel_rim: негерметичный меш обода, выступающие наружу спицы и 17956 граничных ребер
- Воспроизведение: `create_wheel_rim(0.4572, 0.2032, Some(0.490), Some(0.160), Some(0.066), Some(0.1143), Some(5), Some(0.014), Some(5), Some(0.028), Some(0.018), Some(0.035))`
- Ожидание: Герметичное манифолдное тело (`watertight == true`, `boundary_edges == 0`, `self_intersections == 0`), спицы аккуратно соединяют ступицу с внутренней стенкой обода.
- Факт: `is_watertight: false`, 17956–19362 boundary edges, 273–328 самопересечений; спицы (`r_spoke_outer = r_bead - t_w + 0.004 > r_well`) протыкают монтажный ручей обода наружу.
- Артефакты: `qa/artifacts/wheel_rim_wireframe.png`, `qa/artifacts/wheel_rim_normals.png`, `qa/results/primitives_mesh_scan.json`
- Гипотеза причины: `r_spoke_outer` превышает радиус ручья `r_well - t_w`, а сборка через `csg_union`/`csg_difference` криволинейных сеток оставляет тысячи T-junctions и дыр.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-005 [S2][OPEN] polygon_prism: фаска (chamfer) дает негерметичный меш на 6/8 гранях, игнорируется на 3/4 гранях, и не отклоняется sides < 3
- Воспроизведение: `create_polygon_prism(6, 0.05, Some(0.04), None, Some(0.002))` и `add_feature_primitive` с `sides: 2`.
- Ожидание: `status == "PASS"` (`watertight == true`, `genus == 0`) для всех `sides >= 3`, уменьшение объема при `chamfer > 0` на 3/4/6/8 гранях, и возврат ошибки при `sides < 3`.
- Факт: На 6 и 8 гранях 60 и 112 открытых граничных ребер (дыры); на 3 и 4 гранях фаска не меняет сетку; при `sides = 2` ошибка не возвращается.
- Артефакты: `qa/results/primitives_mesh_scan.json`, `qa/results/functional_matrix_scan.json`
- Гипотеза причины: `csg_intersection(&prism, &chamfer_body)` отбрасывает мелкие треугольники фаски и создает T-junctions; в `dag.rs` отсутствует проверка `*sides < 3`.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-006 [S2][OPEN] csg: булевы операции и cut_hole / cut_helical_groove удаляют мелкие треугольники в метрах и оставляют T-junctions
- Воспроизведение: `cut_hole`, `cut_helical_groove`, `csg_union`, `csg_difference`, `csg_intersection` в `operations_mesh_scan.json`.
- Ожидание: `status == "PASS"`, `watertight == true`, `boundary_edges == 0`, `t_junctions == 0`, `self_intersections == 0`, корректный `genus`.
- Факт: Все 9 проверенных CSG-операций имеют статус `FAIL` (от 12 до 24777 boundary edges и T-junctions).
- Артефакты: `qa/results/operations_mesh_scan.json`
- Гипотеза причины: (1) В `csg.rs` строка 357 фильтр `cross.norm_squared() > 1e-16` (в $\text{м}^4$) удаляет все валидные треугольники площадью $< 0.005\text{ мм}^2$, пробивая дыры; (2) веерная триангуляция от `v[0]` на полигонах с коллинеарными точками разбиения создает вырожденные треугольники; (3) отсутствует сшивка T-junctions между смежными гранями.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-007 [S2][OPEN] topology: TriangleMesh::scale с отрицательным коэффициентом масштаба выворачивает нормали и знак объема
- Воспроизведение: `box_mesh.scale([-1.0, 1.0, 1.0])` или `transform_part` с `scale: [-1.0, 1.0, 1.0]`.
- Ожидание: `signed_volume > 0`, нормали направлены наружу.
- Факт: `signed_volume = -0.006 < 0`, меш вывернут наизнанку (`status == "FAIL"`).
- Артефакты: `qa/results/operations_mesh_scan.json`
- Гипотеза причины: `TriangleMesh::scale` в `crates/mcp-cad-core/src/topology.rs` не меняет порядок обхода вершин треугольников (`tri.swap(1, 2)`) при `scale[0] * scale[1] * scale[2] < 0.0`.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-008 [S2][OPEN] cad tools: неудачный rebuild оставляет невалидный узел в DAG детали, ломая все последующие операции
- Воспроизведение: Вызвать `add_feature_primitive` с `width: -0.1` на существующей детали, затем вызвать валидный `cut_hole`.
- Ожидание: При ошибке `rebuild()` невалидный узел откатывается из `part.dag.nodes`, деталь остается валидной.
- Факт: Узел остается в `part.dag.nodes`, и все последующие валидные вызовы (`cut_hole`, `modify_feature`, и т.д.) падают с ошибкой старого узла.
- Артефакты: `qa/results/functional_matrix_scan.json`
- Гипотеза причины: В `crates/mcp-cad-server/src/tools/cad.rs` `part.dag.add_feature` вызывается до `part.rebuild()?`, без удаления добавленного узла в случае ошибки `rebuild()`.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-009 [S2][OPEN] batch_cad_operations: отсутствие атомарного отката состояния при ошибке на промежуточном шаге
- Воспроизведение: Вызвать `batch_cad_operations` с `[create_part("atomic-orphan-part"), delete_part("non-existent")]`.
- Ожидание: При ошибке пакета состояние `parts` и `materials` откатывается к состоянию до вызова пакета.
- Факт: `atomic-orphan-part` остается в `state.parts`.
- Артефакты: `qa/results/functional_matrix_scan.json`
- Гипотеза причины: `batch_cad_operations` мутирует `state` напрямую без сохранения снимка для отката при ошибке.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-010 [S3][OPEN] open_preview_window: не проверяет существование part_id в headless-режиме
- Воспроизведение: Вызвать `open_preview_window({"part_id": "non-existent-part-id"})` в headless-режиме.
- Ожидание: Возврат ошибки `PartNotFound`.
- Факт: Возвращает `Ok({"status": "headless"})`, игнорируя невалидный `part_id`.
- Артефакты: `qa/results/functional_matrix_scan.json`
- Гипотеза причины: Проверка `state.parts.get(part_id)` в `crates/mcp-cad-server/src/tools/render.rs` находится только внутри ветки `if let Some(proxy) = &state.gui_proxy`.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN
