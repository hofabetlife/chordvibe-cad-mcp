# Отчёт о выполнении задачи P2-T7: Именованные грани — проход через CSG и transform

- **ID задачи**: `P2-T7` (Фаза 2: Агентский Контракт)
- **Размер**: `L`
- **Зависимости**: `P2-T6` ([`docs/agent_tasks/P2-T6-report.md`](P2-T6-report.md)), `P0-T7` ([`docs/agent_tasks/P0-T7-report.md`](P0-T7-report.md)), `P2-T5` ([`docs/FACE_NAMING_DESIGN.md`](../FACE_NAMING_DESIGN.md))
- **Статус**: ✅ DONE
- **Дата**: 2026-11-06
- **Ветка**: `agent/P2-T7`

---

## 1. Что сделано

Реализован сквозной детерминированный пронос идентификаторов топологических граней (`face_id: u32`) и слияние реестров граней (`faces: Vec<FaceInfo>`) через все этапы BSP-CSG (`csg_union`, `csg_difference`, `csg_intersection`), конструктивные вырезы (`cut_hole` со сквозными/глухими отверстиями и шпоночным пазом DIN 6885, `cut_helical_groove`) и пространственные преобразования (`transform`, `transform_part`):

1. **Пронос `face_id` через конвейер BSP-CSG и топологическую постобработку (`crates/mcp-cad-core/src/csg.rs`)**:
   - В структуру `CsgPolygon` добавлено поле `pub face_id: u32` и конструктор `CsgPolygon::with_face_id(vertices, face_id)` (при `flip()` идентификатор `face_id` сохраняется).
   - В `Plane::split_polygon` обе половины разрезаемого полигона (`f`, `b`) и копланарные ветви (`coplanar_front`, `coplanar_back`) наследуют `poly.face_id`.
   - В `BspNode::polygons_from_mesh` чтение `mesh.face_ids` интегрировано со слиянием копланарных пар треугольников в выпуклые квады: слияние разрешается только при совпадении `face_id_1 == face_id_2`, что исключает смешивание соседних копланарных граней с разными идентификаторами.
   - В `polygons_to_mesh`, `triangulate_subdivided_convex_loop` и `repair_manifold_triangles` каждый порождаемый треугольник получает `face_id` родительского полигона (включая разбиение длинных рёбер при устранении T-junctions и закрытие 3-циклов по мажоритарному `face_id` смежных граней).
   - Во всех 4 проходах `optimize_csg_mesh_quality` обеспечено сохранение `face_ids` и запрет смешивания границ между разными `face_id`:
     - `smooth_coplanar_interior_vertices`: тангенциальное сглаживание Лапласа применяется только к вершинам внутри одного `face_id`;
     - `optimize_coplanar_delaunay_edges`: Lawson Delaunay-флип разрешается только для рёбер между двумя треугольниками с одинаковым `face_id`;
     - `ear_clip_planar_ring_maxmin`, `try_remove_vertex_ear_clip` и `collapse_redundant_and_sliver_vertices`: 1-ring ear-clipping выполняется только внутри однородного `face_id`, а при Link-Condition edge collapse вектор `face_ids` синхронно фильтруется вместе с `indices`;
     - `refine_coplanar_sliver_fans_and_edges`: новые треугольники со вставленными точками Штейнера наследуют `face_id` исходного треугольника.
   - Реализована функция `merge_csg_face_registries(a, b, is_difference)`:
     - смещает `face_id` операнда `b` на `id_offset = max_id(a)` до построения BSP-дерева;
     - для `csg_difference(a, b)` добавляет канонический префикс `cut:` (для общих тел `b`, не являющихся специализированными `hole_*` / `keyway_*`) и инвертирует нормали плоских граней вычитаемого тела (`FaceKind::Plane`), так как их ориентация обращается внутрь выреза;
     - вызывает `mesh.recompute_face_registry()`, пересчитывающий площади, центроиды, `sample_point` и отбрасывающий полностью срезанные грани с `triangles_count == 0`.

2. **Именованные грани инструментов выреза (`crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/src/dag.rs`)**:
   - В `cut_hole_with_settings` (`primitives.rs`) грани цилиндрического инструмента переименовываются до вызова `csg_difference`:
     - `cap_top` $\to$ роль `"hole_bottom"` (при глухом отверстии выживает внутри детали с нормалью $-n_{\text{hole}}$; при сквозном отверстии отсекается и автоматически удаляется из `faces`);
     - `cap_bottom` $\to$ роль `"hole_top"` (входная крышка за пределами детали, отсекается при CSG);
     - `side` $\to$ роль `"hole_wall"` (`FaceKind::Cylinder`, `radius: Some(r)`, ось `normal_or_axis = unit_norm`, алиасы `"hole_wall"`, `"bore"`, `"inner"`).
   - В `build_keyway_cutter_prism` (`primitives.rs`) для отверстия со шпоночным пазом DIN 6885 размечены 4 топологические грани: `id=1` `"hole_bottom"`, `id=2` `"hole_top"`, `id=3` `"hole_wall"` (дуга цилиндра `i >= 3`, `FaceKind::Cylinder`, `radius: Some(r)`) и `id=4` `"keyway_wall"` (прямоугольный паз `i < 3`, `FaceKind::Plane`).
   - В `create_helical_cutter_with_settings` (`primitives.rs`) размечена грань `id=1` `"helical_groove"` (`FaceKind::Helical`, `radius: Some(helix_radius)`).
   - В `FeatureTree::evaluate_with_settings` (`dag.rs`) добавлены метод `FeatureTree::add_node` и функция `tag_new_cutter_faces`, привязывающая `origin_feature = Some(node.id.to_string())`, канонические имена `hole_wall:feat_k` / `hole_bottom:feat_k` / `keyway_wall:feat_k` / `cut:feat_k:<role>` и короткие алиасы (`"hole_wall"`, `"hole_wall:k"`, `"feat_k:hole_wall"`) для `CutHole`, `CutHelicalGroove`, `BooleanDifferenceMesh`, `BooleanUnionMesh` и `BooleanIntersectionMesh`.

3. **Пространственные преобразования `transform` и расширение `FaceSelector` (`crates/mcp-cad-core/src/topology.rs`, `crates/mcp-cad-server/src/tools/cad.rs`, `tools/mod.rs`)**:
   - Проверена и протестирована работа `TriangleMesh::{translate, rotate_rpy_deg, scale, transform}` при положительном и отрицательном масштабе ($\det(S) < 0$): при нечётном числе отрицательных факторов масштаба порядок обхода треугольников обращается, `face_ids` остаются синхронными с `indices`, а нормали и площади граней пересчитываются в `recompute_face_registry`.
   - В `resolve_face_by_name` (`tools/cad.rs`) добавлена поддержка шаблонного сопоставления с суффиксом `:*` (например, `"hole_wall:*"` или `"cut:*"`), а в `parse_face_kind` — поддержка всех синонимов `FaceKind` (`plane`/`planar`, `cylinder`/`cylindrical`, `cone`/`conical`, `sphere`/`spherical`, `torus`/`toroidal`, `helical`, `other`/`freeform`).
   - В инструмент `transform_part` (`tools/cad.rs`, `tools/mod.rs`, `docs/MCP_API_REFERENCE.md`) добавлены опциональные параметры `pivot_face` (вращение/масштабирование вокруг центроида выбранной грани) и `align_face` (`{"face": ..., "target_point": [x, y, z]}`).

---

## 2. Как проверено

1. В `crates/mcp-cad-core/tests/csg_invariants.rs` добавлены 4 специализированных теста `p2_t7_*`:
   - `p2_t7_box_minus_cylindrical_hole_named_faces`: брусок $100 \times 80 \times 20\text{ мм}$ минус сквозное цилиндрическое отверстие $r = 15\text{ мм}$ — проверяет наличие грани `hole_wall:feat_1` (и выборку по `"hole_wall:*"`) типа `FaceKind::Cylinder` с радиусом $15\text{ мм}$ и площадью $2\pi r h$ с точностью $< 1\%$, сохранение всех 6 исходных граней бруска (`top`, `bottom`, `front`, `back`, `left`, `right`), уменьшение площадей `top` и `bottom` ровно на $\pi r^2$ с точностью $< 1\%$ и тождество $\sum f.\text{area} = \text{mesh.surface\_area}()$ ($< 10^{-9}$);
   - `p2_t7_blind_hole_and_keyway_hole_named_faces`: проверяет глухое отверстие (сохранение `hole_bottom` с площадью $\pi r^2$ и `hole_wall` с площадью $2\pi r d$) и отверстие со шпоночным пазом DIN 6885 (сохранение как `hole_wall`, так и `keyway_wall`);
   - `p2_t7_csg_difference_cut_prefix_and_transform_negative_scale`: проверяет префикс `cut:` и инверсию нормалей выживших граней вычитаемого бруска в `csg_difference`, а также сохранение `face_ids`, `faces` и положительного объёма после `scale(-1.0, 1.0, 1.0)` + `translate` + `rotate_rpy_deg`;
   - `p2_t7_rebuild_determinism_of_face_ids_and_names`: проверяет 100% побитовый детерминизм `face_ids`, `faces` (`id`, `name`, `aliases`, `role`, `kind`, `area`, `centroid`) при повторных вызовах `tree.evaluate()`.
2. В `crates/mcp-cad-server/tests/mcp_contract_tests.rs` добавлен контрактный тест `test_mcp_p2_t7_csg_and_cut_hole_named_faces`, проверяющий `cut_hole`, выборку `"hole_wall:*"` в `query_part_geometry`, площади граней в `mm²` и выравнивание детали по грани через `transform_part` (`align_face`).
3. Проверены сборка и линтер TypeScript/Web (`compile_applet` и `lint_applet` — 0 ошибок).

---

## 3. Тесты до/после

| Набор тестов / Проверка | До `P2-T7` | После `P2-T7` | Дельта |
|---|---|---|---|
| **`csg_invariants.rs` (`mcp-cad-core`)** | 5 тестов | **9 тестов** | **+4 новых теста `p2_t7_*`** |
| **`mcp_contract_tests.rs` (`mcp-cad-server`)** | 26 тестов | **27 тестов** | **+1 новый тест `test_mcp_p2_t7_*`** |
| **Всего тестов в Rust Workspace** | 191 тест | **196 тестов** | **+5 новых тестов** |
| **TypeScript (`npm run lint` / `compile_applet`)** | 0 errors | **0 errors** | Без регрессий |

---

## 4. Что не удалось и остаточные риски

- Все критерии приёмки задачи `P2-T7` выполнены в полном объёме.
- Привязка граничных условий и внешних сил МКЭ по именам граней (`fixed: [{"face": "bottom"}]`, `loads: [{"face": "hole_wall:feat_1", ...}]`) в `crates/mcp-cad-fea/src/boundary.rs` и `run_fea_analysis` относится к следующей задаче **`P2-T8`**.

---

## 5. Побочные находки

1. В `crates/mcp-cad-server/tests/mcp_contract_tests.rs:3500-3545` и `crates/mcp-cad-core/tests/primitive_invariants.rs:1254-1370` (добавленных в `P2-T6`) использовались строки `"cylindrical"`/`"planar"` и имя `"top"` для крышки цилиндра, тогда как `FaceKind` сериализуется в `snake_case` как `"cylinder"`/`"plane"`, а `create_cylinder` именует торцевые крышки каноническими ролями `"cap_top"`/`"cap_bottom"` (с короткими алиасами `"top"`/`"bottom"`). В рамках `P2-T7` добавлены ассоциированные константы-алиасы `FaceKind::{Planar, Cylindrical, Conical, Spherical, Toroidal, Freeform}` в `topology.rs`, расширен парсер `FaceKind::from_str_ignore_case` и синхронизированы оба теста.
2. В `src/cad/primitives.ts:3` интерфейс `AngleHistogramBuckets` импортируется как значение, а не как `type AngleHistogramBuckets`, из-за чего запуск `node --experimental-strip-types` без предварительной компиляции `tsc` падает на этапе загрузки модуля (зафиксировано в описании задачи `TS-FIX-01` в `TASKS.md`).
