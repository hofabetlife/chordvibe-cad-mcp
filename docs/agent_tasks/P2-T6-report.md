# Отчёт о выполнении задачи P2-T6: Именованные грани — примитивы

- **ID задачи**: `P2-T6` (Фаза 2: Агентский Контракт)
- **Размер**: `M`
- **Зависимости**: `P2-T5` ([`docs/FACE_NAMING_DESIGN.md`](../FACE_NAMING_DESIGN.md))
- **Статус**: ✅ DONE
- **Дата**: 2026-11-06
- **Ветка**: `agent/P2-T6`

---

## 1. Что сделано

Реализован реестр именованных топологических граней на `TriangleMesh` и генерация канонических граней для всех базовых твердотельных примитивов с выводом и фильтрацией в инструменте `query_part_geometry`:

1. **Структуры данных и пересчёт геометрии граней (`crates/mcp-cad-core/src/topology.rs`, `crates/mcp-cad-core/src/lib.rs`, `crates/mcp-cad-core/src/import_stl.rs`)**:
   - Добавлены типы `FaceKind` (`Planar`, `Cylindrical`, `Conical`, `Spherical`, `Toroidal`, `Freeform`) и `FaceInfo` (`id`, `name`, `aliases`, `role`, `kind`, `origin_feature`, `area`, `centroid`, `sample_point`, `normal_or_axis`, `radius`, `triangles_count`).
   - В `TriangleMesh` добавлены обратно совместимые поля `face_ids: Vec<u32>` и `faces: Vec<FaceInfo>` с атрибутами `#[serde(default, skip_serializing_if = "Vec::is_empty")]`, благодаря чему старые JSON-файлы без `face_ids`/`faces` десериализуются без ошибок, а компактные представления не раздуваются при пустых списках.
   - Реализованы конструктор `TriangleMesh::with_faces(...)` и метод `TriangleMesh::recompute_face_registry(&mut self)`, вычисляющий точную площадь каждой грани как сумму площадей принадлежащих ей треугольников ($\sum_{f \in \text{faces}} f.\text{area} = \text{mesh.surface\_area}()$ с машинной точностью $< 10^{-15}$), взвешенный по площади центроид и гарантированную поверхностную точку `sample_point` (барицентр треугольника грани, ближайшего к `centroid`).
   - Во всех методах пространственных преобразований `TriangleMesh` (`translate`, `scale`, `rotate_rpy_deg`, `transform`) добавлено обновление метаданных `faces` (смещение/поворот `centroid` и `sample_point`, поворот/инверсия `normal_or_axis` при $\det(S) < 0$, масштабирование `radius` и пересчёт площадей через `recompute_face_registry`).
   - В `import_stl::weld_vertices` добавлено сохранение `face_ids` и `faces` при фильтрации невырожденных треугольников.

2. **Проставление `face_ids` и `faces` в генераторах примитивов (`crates/mcp-cad-core/src/primitives.rs`)**:
   - **`create_box(w, h, d)`**: 6 плоских граней (`FaceKind::Planar`): `id=1` `"top"` (`max_z`, $+Z$), `id=2` `"bottom"` (`min_z`, $-Z$), `id=3` `"right"` (`max_x`, $+X$), `id=4` `"left"` (`min_x`, $-X$), `id=5` `"front"` (`max_y`, $+Y`), `id=6` `"back"` (`min_y`, $-Y`).
   - **`create_cylinder(r, h, segs)` и `create_cylinder_oriented`**: 3 грани: `id=1` `"bottom"` (`min_z`, `Planar`), `id=2` `"top"` (`max_z`, `Planar`), `id=3` `"side"` (`barrel`, `outer`, `mantle`, `Cylindrical`, `radius: Some(r)`).
   - **`create_cone(r_bottom, r_top, h, segs)`**:
     - усечённый конус (`r_bottom > 0, r_top > 0`): 3 грани (`"bottom"`, `"top"`, `"side"` с `FaceKind::Conical` и средним радиусом `0.5 * (r_bottom + r_top)`);
     - острый конус (`r_top == 0`): 2 грани (`"bottom"`, `"side"`);
     - перевёрнутый острый конус (`r_bottom == 0`): 2 грани (`"top"`, `"side"`).
   - **`create_sphere(r, sectors, stacks)`**: 1 сферическая грань `id=1` `"surface"` (`outer`, `shell`, `FaceKind::Spherical`, `radius: Some(r)`).
   - **`create_polygon_prism(sides, ...)`** (с фаской и без фаски): $N + 2$ граней (`id=1` `"bottom"`, `id=2` `"top"`, `id=3..N+2` `"side_0"`..`"side_{N-1}"` с точными внешними нормалями $[\cos\theta_i, \sin\theta_i, 0]$; при наличии фаски переходы фаски относятся к соответствующей боковой грани `side_i`, сохраняя инвариант $\sum f.\text{area} = \text{surface\_area}()$).
   - Для сложных составных генераторов (`create_spur_gear`, `create_wheel_rim`, `create_revolve`, `create_helical_cutter`) заполняется `face_ids = vec![0; mesh.indices.len()]` до этапа `P2-T7`.

3. **Привязка `origin_feature` и квалифицированных алиасов в DAG (`crates/mcp-cad-core/src/dag.rs`)**:
   - В `FeatureTree::evaluate_with_settings` добавлена функция `tag_node_faces`, проставляющая `origin_feature = Some(node.id.to_string())` и квалифицированные алиасы `feat_{idx}:<role>`, `<node_slug>:<role>` и `<uuid>:<role>` перед локальным позиционированием/поворотом узла.

4. **Интеграция в `query_part_geometry` и `FaceSelector` (`crates/mcp-cad-server/src/tools/cad.rs`, `tools/mod.rs`, `error.rs`)**:
   - В `query_part_geometry` добавлен массив `"faces"` с автоматическим пересчётом линейных величин (`centroid`, `sample_point`, `radius`) и площадей (`area`, плюс эталонное `area_m2`) в активный `length_unit` (`"mm"` или `"m"`), с сохранением существующего поля `"detected_planar_faces"`.
   - Реализован `resolve_face_selector` с поддержкой выбора по каноническому имени/алиасу (`"top"`, `"side"`, `"feat_0:top"`), по `{"face": ...}`, `{"face_id": id}`, списку `{"faces": [...]}` и геометрическому предикату `{"where": {"kind", "radius", "tolerance", "normal", "axis", "near", "closest", "origin_feature", "role", "min_area", "max_area"}}`.
   - В `crates/mcp-cad-server/src/error.rs` добавлены машиночитаемые коды `FACE_NOT_FOUND` и `AMBIGUOUS_FACE_SELECTOR`.

---

## 2. Как проверено

1. Добавлены 6 модульных тестов в `crates/mcp-cad-core/tests/primitive_invariants.rs`:
   - `p2_t6_named_faces_box` (6 граней, площади, нормали, алиасы `max_z`/`max_x`, тождество суммы площадей `< 1e-9`);
   - `p2_t6_named_faces_cylinder` (3 грани, `Planar` + `Cylindrical`, `radius = 0.025`, тождество суммы площадей `< 1e-9`);
   - `p2_t6_named_faces_cone_variants` (усечённый конус — 3 грани, острый — 2 грани, перевёрнутый — 2 грани, тождество суммы площадей `< 1e-9`);
   - `p2_t6_named_faces_sphere` (1 грань `"surface"`, `Spherical`, `radius = 0.04`, тождество суммы площадей `< 1e-9`);
   - `p2_t6_named_faces_polygon_prism` (призма с 6 гранями без фаски и с фаской: 8 граней `"bottom"`, `"top"`, `"side_0..side_5"`, нормали, тождество суммы площадей `< 1e-9`);
   - `p2_t6_legacy_json_deserialization_and_dag_origin_tags` (десериализация legacy JSON без `face_ids`/`faces`, проверка `origin_feature` и алиасов `feat_0:top`, `base_plate:top` после `FeatureTree::evaluate`).
2. Добавлен контрактный тест `test_mcp_p2_t6_named_faces_query_and_selector` в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, проверяющий вывод `"faces"` в `mm` и `m`, фильтрацию `face_selector.where` по `kind: "cylindrical"` и `radius`, а также возврат структурированного кода ошибки `FACE_NOT_FOUND`.
3. Проверены сборка и линтер фронтенда (`npm run lint` / `compile_applet` — 0 ошибок).

---

## 3. Тесты до/после

| Набор тестов / Проверка | До `P2-T6` | После `P2-T6` | Дельта |
|---|---|---|---|
| **`primitive_invariants.rs` (`mcp-cad-core`)** | 18 тестов | **24 теста** | **+6 новых тестов `p2_t6_*`** |
| **`mcp_contract_tests.rs` (`mcp-cad-server`)** | 25 тестов | **26 тестов** | **+1 новый тест `test_mcp_p2_t6_*`** |
| **Всего тестов в Rust Workspace** | 184 теста | **191 тест** | **+7 новых тестов** |
| **TypeScript (`npm run lint` / `compile_applet`)** | 0 errors | **0 errors** | Без регрессий |

---

## 4. Что не удалось и остаточные риски

- Все критерии приёмки задачи `P2-T6` выполнены в полном объёме.
- В соответствии с декомпозицией `docs/FACE_NAMING_DESIGN.md` пронос `face_ids` и слияние реестров `faces` через булевы операции BSP-CSG (`csg.rs`, `cut_hole`, `cut_helical_groove`) относятся к следующей задаче **`P2-T7`**.

---

## 5. Побочные находки

- Новых дефектов, выходящих за рамки уже зарегистрированных задач в `TASKS.md`, не обнаружено.
