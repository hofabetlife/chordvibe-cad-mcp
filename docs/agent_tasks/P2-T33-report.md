# P2-T33 Report: Поддержка `transform_part` для импортированных STL-деталей при пустом DAG (`part.dag.nodes.is_empty()`) и предотвращение ошибки `Cannot transform empty solid` (`tools/cad.rs:3130`)

- **Дата выполнения:** 10.11.2026
- **Статус:** ✅ DONE
- **Компоненты:** `crates/mcp-cad-core/src/{dag.rs, part.rs, query_optimizer.rs}`, `crates/mcp-cad-server/src/tools/cad.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `src/cad/{primitives.ts, mcp_dispatcher.ts, cad_engine.test.ts}`

---

## 1. Описание проблемы

В ходе выполнения **P2-T30** (`docs/agent_tasks/P2-T30-report.md`) было выявлено:
- При импорте детали через `import_stl` (`crates/mcp-cad-server/src/tools/cad.rs:2070-2073`) сетка треугольников записывалась напрямую в `part.mesh = mesh.clone()`, тогда как параметрическое дерево истории `part.dag.nodes` оставалось пустым (`part.dag.nodes.is_empty()`).
- При последующем вызове `transform_part` (`crates/mcp-cad-server/src/tools/cad.rs:3130-3143`), а также других операций над DAG (`cut_hole`, `cut_helical_groove`, `boolean_difference`, `boolean_union`, `boolean_intersection`, `add_feature_primitive`, `optimize_feature_tree`), добавлялся узел `FeatureOp::Transform { ... }` и вызывался `part.rebuild()`, который выполнял `part.dag.evaluate_with_settings(&settings)`.
- В `FeatureTree::evaluate_with_settings` (`crates/mcp-cad-core/src/dag.rs:1016-1018`) переменная `current_mesh` начиналась с `None`, и при отсутствии базового генеративного узла в `part.dag.nodes` выполнение `FeatureOp::Transform` завершалось ошибкой `CAD_EVALUATION_ERROR: Cannot transform empty solid`.

---

## 2. Выполненные изменения

1. **Базовый узел `FeatureOp::ImportedMesh { mesh: TriangleMesh }` в параметрическом DAG (`crates/mcp-cad-core/src/dag.rs`)**:
   - В перечисление `FeatureOp` добавлен вариант `ImportedMesh { mesh: TriangleMesh }`.
   - В `FeatureTree::add_feature` добавлено имя по умолчанию `"Imported Mesh"`.
   - В `FeatureTree::evaluate_with_settings` реализована обработка `FeatureOp::ImportedMesh { mesh }`:
     - проверяется непустота буфера вершин (`!mesh.vertices.is_empty()`);
     - проставляется атрибуция граней `tag_node_faces(&mut imported, node, node_idx)` (если `origin_feature` ещё не задан);
     - если `current_mesh` равен `None`, `imported` становится базовым телом (`Some(imported)`), а если в DAG уже было тело — выполняется булево объединение `csg_union(&m, &imported)`.

2. **Автоматическая инициализация корневого узла `ImportedMesh` (`crates/mcp-cad-core/src/part.rs`)**:
   - В структуру `Part` добавлен метод `pub fn ensure_imported_mesh_in_dag(&mut self)`: если `self.dag.nodes.is_empty() && !self.mesh.vertices.is_empty()`, в `self.dag` автоматически добавляется корневой узел `FeatureOp::ImportedMesh { mesh: self.mesh.clone() }` с именем `"Imported Mesh (<part.name>)"`.
   - Вызов `self.ensure_imported_mesh_in_dag()` встроен в начало `Part::rebuild`, `Part::tessellate` и `Part::optimize_dag`, что гарантирует сохранность импортированной геометрии даже для деталей, загруженных из legacy-проектов или созданных напрямую через `part.mesh`.
   - Для по-настоящему пустых деталей (`self.dag.nodes.is_empty() && self.mesh.vertices.is_empty()`) `ensure_imported_mesh_in_dag()` ничего не добавляет, сохраняя ожидаемую ошибку `Cannot transform empty solid`.

3. **Поддержка `FeatureOp::ImportedMesh` в стоимостном оптимизаторе (`crates/mcp-cad-core/src/query_optimizer.rs`)**:
   - В `estimate_primitive_aabb` добавлено вычисление `Some(mesh.bounding_box())` для непустого `FeatureOp::ImportedMesh { mesh }` (позволяет правилу `R2` отсекать непересекающиеся `CutHole` относительно импортированной STL-детали).
   - В `estimate_dag_cost` добавлена оценка стоимости `ImportedMeshScan` (`current_tris == 0`) и `BspUnionJoin(imported_mesh, ...)` (`current_tris > 0`).
   - Во всех проверках барьерных узлов правил `R2`, `R3`, `R4`, `R5` добавлен учёт `FeatureOp::ImportedMesh { .. }`.

4. **Интеграция в инструменты MCP-сервера и TypeScript-диспетчера (`crates/mcp-cad-server/src/tools/cad.rs`, `src/cad/{primitives.ts, mcp_dispatcher.ts}`)**:
   - В `import_stl` сразу вызывается `part.ensure_imported_mesh_in_dag()`, а перед добавлением операций в `transform_part`, `cut_hole`, `cut_helical_groove`, `boolean_difference`, `boolean_union`, `boolean_intersection` и `add_feature_primitive` также вызывается `ensure_imported_mesh_in_dag()`.
   - В `scale_feature_op_from_meters` и `modify_feature` добавлена поддержка `FeatureOp::ImportedMesh`.
   - В `src/cad/primitives.ts` (`evaluateFeatureTree`) и `src/cad/mcp_dispatcher.ts` (`transform_part`) обеспечена полная совместимость для узлов `stl_import` и деталей с пустым `dag`, включая возврат `feature_id`, `revision`, `bounding_box`, `volume` и `volume_m3`.

---

## 3. Верификация и тесты

- **`crates/mcp-cad-core/src/dag.rs`**: добавлен unit-тест `test_p2_t33_imported_mesh_and_transform_evaluation`.
- **`crates/mcp-cad-core/src/part.rs`**: добавлен unit-тест `test_p2_t33_ensure_imported_mesh_in_dag_allows_transform_on_mesh_only_part`.
- **`crates/mcp-cad-server/src/tools/cad.rs`**: добавлен unit-тест `test_p2_t33_transform_imported_stl_part_and_empty_dag_fallback` (экспорт STL -> `import_stl` -> `transform_part` со сдвигом и масштабированием -> проверка объёма и габаритов -> проверка авто-восстановления при очищенном `part.dag.nodes` -> проверка возврата ошибки `Cannot transform empty solid` на пустой детали без геометрии).
- **`crates/mcp-cad-server/tests/mcp_contract_tests.rs`**: добавлен сквозной JSON-RPC контрактный тест `test_mcp_p2_t33_transform_and_csg_on_imported_stl_part` (`import_stl` -> `transform_part` -> `cut_hole` -> `get_feature_tree`).
- **`src/cad/cad_engine.test.ts`**: добавлен тест `verifies P2-T33: transform_part works on imported STL parts and parts with empty DAG` (все 35 тестов проходят).

---

## 4. Новые выявленные замечания (занесены в `TASKS.md`)

- **`P2-T34 [S]`**: В `crates/mcp-cad-server/src/tools/cad.rs:200-211` функция `scale_feature_op_from_meters`, используемая в `get_feature_tree`, клонирует меш-операнды `FeatureOp::BooleanDifferenceMesh { tool_mesh }`, `FeatureOp::BooleanUnionMesh { tool_mesh }`, `FeatureOp::BooleanIntersectionMesh { tool_mesh }` и `FeatureOp::ImportedMesh { mesh }` без масштабирования координат вершин `vertices[i].position` и метрик граней (`faces[i].area`, `centroid`, `sample_point`, `radius`) на коэффициент `k_from_m`, из-за чего в `get_feature_tree` при `length_unit = "mm"` координаты вершин внутри этих узлов остаются в метрах СИ (`m`).
