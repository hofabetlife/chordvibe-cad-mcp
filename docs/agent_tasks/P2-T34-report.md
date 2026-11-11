# P2-T34 Report: Масштабирование вершин и метаданных граней меш-операндов (`Boolean*Mesh`, `ImportedMesh`) в `scale_feature_op_from_meters` (`tools/cad.rs:200-211`) при вызове `get_feature_tree`

- **Дата выполнения:** 10.11.2026
- **Статус:** ✅ DONE
- **Компоненты:** `crates/mcp-cad-server/src/tools/{cad.rs, project.rs}`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `src/cad/{mcp_dispatcher.ts, cad_engine.test.ts}`

---

## 1. Описание проблемы

В ходе выполнения **P2-T33** (`docs/agent_tasks/P2-T33-report.md`) было выявлено:
- В `crates/mcp-cad-server/src/tools/cad.rs` функция `scale_feature_op_from_meters`, используемая инструментом №7 `get_feature_tree`, конвертировала линейные параметры всех аналитических примитивов из метров СИ (`m`) в запрошенную единицу `length_unit` (`mm` или `m`), однако для вариантов с вложенными сетками `FeatureOp::BooleanDifferenceMesh { tool_mesh }`, `FeatureOp::BooleanUnionMesh { tool_mesh }`, `FeatureOp::BooleanIntersectionMesh { tool_mesh }` и `FeatureOp::ImportedMesh { mesh }` возвращала неконвертированную копию `TriangleMesh` в метрах СИ (`m`).
- В результате при вызове `get_feature_tree` с `length_unit = "mm"` (по умолчанию) координаты вершин `vertices[i].position` и геометрические метрики граней `faces[i]` (`area`, `centroid`, `sample_point`, `radius`) внутри этих узлов оставались в метрах СИ (`m` / `m²`) вместо миллиметров (`mm` / `mm²`).

---

## 2. Выполненные изменения

1. **Масштабирование `TriangleMesh` в `scale_feature_op_from_meters` (`crates/mcp-cad-server/src/tools/cad.rs`)**:
   - Добавлена функция `scale_mesh_from_meters(mesh: &TriangleMesh, k_from_m: f64) -> TriangleMesh`, которая при `|k_from_m - 1.0| > 1e-12`:
     - масштабирует координаты всех вершин `v.position` на коэффициент `k_from_m`;
     - масштабирует площадь каждой грани `face.area` на `k_from_m.powi(2)` (`k_area`);
     - масштабирует `face.centroid` и `face.sample_point` на `k_from_m`;
     - масштабирует характеристический радиус грани `face.radius` на `k_from_m`.
   - В `scale_feature_op_from_meters` вызов `scale_mesh_from_meters` применён ко всем меш-вариантам `FeatureOp`:
     - `FeatureOp::BooleanDifferenceMesh { tool_mesh }`;
     - `FeatureOp::BooleanUnionMesh { tool_mesh }`;
     - `FeatureOp::BooleanIntersectionMesh { tool_mesh }`;
     - `FeatureOp::ImportedMesh { mesh }`.

2. **Авто-инициализация `ImportedMesh` при загрузке legacy-проектов в `load_project` (`crates/mcp-cad-server/src/tools/project.rs`)**:
   - В `load_project` для деталей с пустым `part.dag.nodes` и непустым `part.mesh.vertices` добавлен вызов `part.ensure_imported_mesh_in_dag()` с сохранением `saved_revision`, чтобы загруженные из проекта импортированные детали сразу имели корневой узел `ImportedMesh` в `get_feature_tree`.

3. **Синхронизация TypeScript-диспетчера (`src/cad/mcp_dispatcher.ts`)**:
   - В `get_feature_tree` добавлено масштабирование вложенных сеток `mesh` (`stl_import`) и `tool_mesh` (`boolean_*`) в соответствии с `length_unit` (`mm` или `m`).

---

## 3. Верификация и тесты

- **`crates/mcp-cad-server/src/tools/cad.rs`**: добавлен модульный тест `test_p2_t34_get_feature_tree_scales_mesh_operands_and_face_metrics` (проверяет масштабирование координат вершин `vertices[i].position`, площадей `faces[i].area` в $\times 10^6$, центроидов `centroid`, точек `sample_point` и радиусов `radius` в $\times 10^3$ между `length_unit: "mm"` и `"m"` для `BooleanDifferenceMesh`, `BooleanUnionMesh`, `BooleanIntersectionMesh` и `ImportedMesh`).
- **`crates/mcp-cad-server/tests/mcp_contract_tests.rs`**: добавлен сквозной JSON-RPC контрактный тест `test_mcp_p2_t34_get_feature_tree_mesh_operands_scaling`.
- **`src/cad/cad_engine.test.ts`**: добавлен тест `verifies P2-T34: get_feature_tree scales embedded mesh operands (ImportedMesh and Boolean*Mesh) according to length_unit` (`36 passed, 0 failed`).

---

## 4. Новые выявленные замечания (занесены в `TASKS.md`)

- **`P2-T35 [S]`**: В `crates/mcp-cad-server/src/tools/cad.rs` ответы мутирующих инструментов `add_feature_primitive` (`cad.rs:948-954`), `cut_hole` (`cad.rs:1164-1170`), `modify_feature` (`cad.rs:1698-1713`) и `cut_helical_groove` (`cad.rs:1922-1930`) не включают поле `"part_id"` (и в `modify_feature` также отсутствует `"feature_id"`), а в `cut_hole` не возвращается вычисленная глубина `"depth"` в единицах `length_unit` (особенно важная при `through: true`, когда глубина вычисляется автоматически по габариту детали).
