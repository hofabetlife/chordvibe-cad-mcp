# MCP-CAD Engine: Registry of Bugs & Discrepancies

| ID | Sev | Title | Component | Status | Commit | Verified |
|---|---|---|---|---|---|---|
| BUG-001 | S4 | ARCHITECTURE.md indicates 15 tools instead of 26 | docs | FIXED | `b9273ad` | 2026-10-30 |
| BUG-002 | S2 | spur_gear: collinear origin-fan degenerate triangles and non-watertight CSG hub/bore/keyway | core/primitives | FIXED | `6e1dc66` | 2026-10-30 |
| BUG-003 | S2 | revolve: partial revolution (<360 deg) end-caps have inverted triangle winding | core/primitives | FIXED | `3f0ee41` | 2026-10-30 |
| BUG-004 | S2 | wheel_rim: non-watertight mesh with protruding spokes and 17956 boundary edges | core/primitives | FIXED | `5874aa7` | 2026-10-30 |
| BUG-005 | S2 | polygon_prism: chamfer fails watertightness on 6/8 sides, is ignored on 3/4 sides, sides < 3 not rejected | core/primitives & dag | FIXED | `c6a6ef4` | 2026-10-30 |
| BUG-006 | S2 | csg: BSP boolean operations and cut_hole drop small triangles and leave T-junction boundary edges | core/csg | FIXED | `3de6a26` | 2026-10-30 |
| BUG-007 | S2 | topology: TriangleMesh::scale with negative scale factor inverts winding and produces negative signed volume | core/topology | FIXED | `c918111` | 2026-10-30 |
| BUG-008 | S2 | cad tools: failed rebuild leaves invalid FeatureNode in part DAG, poisoning subsequent operations | server/tools/cad | FIXED | `8a0e8bb` | 2026-10-30 |
| BUG-009 | S2 | batch_cad_operations: partial failure does not roll back state mutations from earlier steps | server/tools/cad | FIXED | `13bc330` | 2026-10-30 |
| BUG-010 | S3 | open_preview_window: does not validate part_id existence when running in headless mode | server/tools/render | FIXED | `66e8a81` | 2026-10-30 |

---

## BUG-001 [S4][FIXED] ARCHITECTURE.md: устаревшее упоминание 15 инструментов вместо 26
- **Коммит**: `b9273ad fix(docs): update tool count to 26 in ARCHITECTURE.md [BUG-001]`
- **Причина**: Документация не была синхронизирована с фактическим реестром 26 инструментов Model Context Protocol.
- **Исправление**: В `docs/ARCHITECTURE.md` и в docstring `crates/mcp-cad-server/src/tools/mod.rs` обновлены ссылки с 15/20 на 26 инструментов.
- **Верификация**: `test_wide_scan_functional_matrix_26_tools` и `test_mcp_tools_list` подтвердили наличие и работоспособность всех 26 инструментов.

---

## BUG-002 [S2][FIXED] spur_gear: вырожденные коллинеарные треугольники и негерметичность со ступицей/шпонпазом
- **Коммит**: `6e1dc66 fix(core): construct watertight manifold spur gear with hub, bore, and keyway [BUG-002]`
- **Причина**: Веерная триангуляция от `[0, 0]` на радиальных линиях ножки зуба при $r_f < r_b$ создавала вырожденные коллинеарные треугольники. Использование несшитого CSG для ступицы и шпоночного паза создавало тысячи граничных ребер и T-junctions.
- **Исправление**: 
  - Реализован концентрический триангулятор торцевых крышек зубчатого венца от $r_f$ к $r_{hub}$ или $r_{bore}$ без лучевой сингулярности.
  - Построена единая согласованная топологическая сетка венца, цилиндрической ступицы, центрального отверстия и шпоночного паза стандарта DIN 6885 без несшитых булевых операций.
- **Верификация**:
  - `spur_gear_teeth_8_plain`: status PASS, watertight, 0 degenerates, genus 0.
  - `spur_gear_teeth_20_with_hub_bore_keyway`: status PASS, watertight, genus 1, Euler $\chi=0$, volume $>0$.
  - Визуальный аудит 5 режимов: `spur_gear_shaded.png`, `spur_gear_wireframe.png`, `spur_gear_normals.png`, `spur_gear_backfaces.png`, `spur_gear_boundary_edges.png` подтвердил 0 дыр и 0 артефактов.

---

## BUG-003 [S2][FIXED] revolve: инвертированная ориентация треугольников торцевых крышек частичного вращения
- **Коммит**: `3f0ee41 fix(core): fix partial revolve end-cap winding and analytical normals [BUG-003]`
- **Причина**: Порядок обхода вершин стартовой и конечной плоских торцевых крышек в `create_revolve` имел неверный порядок обхода `[i0, i2, i1]` vs `[i0, i1, i2]`.
- **Исправление**: Исправлен порядок обхода на торцевых крышках $0^\circ$ и $\theta_{\text{end}}$, вычислены точные аналитические нормали для плоских крышек вдоль нормали секущей плоскости.
- **Верификация**:
  - `revolve_curved_90deg`: status PASS, watertight: true, boundary_edges: 0, inverted_edges: 0, genus: 0.
  - `revolve_curved_180deg`: status PASS, watertight: true, boundary_edges: 0, inverted_edges: 0, genus: 0.
  - Визуальный аудит 5 режимов: `revolve_partial_shaded.png`, `revolve_partial_wireframe.png`, `revolve_partial_normals.png`, `revolve_partial_backfaces.png`, `revolve_partial_boundary_edges.png`.

---

## BUG-004 [S2][FIXED] wheel_rim: негерметичный меш колесного диска с выступающими спицами
- **Коммит**: `5874aa7 fix(core): build unified watertight manifold wheel rim mesh [BUG-004]`
- **Причина**: Наружный радиус спиц $r_{spoke\_outer}$ превышал радиус монтажного ручья $r_{well}$, из-за чего спицы протыкали профиль обода наружу. Композиция обода, спиц и ступицы производилась несшитым CSG, порождая 17956 boundary edges.
- **Исправление**: Реализован единый топологический генератор колесного диска: обод построен методом вращения замкнутого профиля с полкой и закраинами, спицы ограничены внутренним радиусом обода $r_{inner} = r_{well} - t_w$, ступица и посадочные отверстия под болты соединены в водонепроницаемый B-Rep меш.
- **Верификация**:
  - `wheel_rim_18x8`: status PASS, watertight: true, boundary_edges: 0, non_manifold_edges: 0, self_intersections: 0.
  - Визуальный аудит 5 режимов: `wheel_rim_shaded.png`, `wheel_rim_wireframe.png`, `wheel_rim_normals.png`, `wheel_rim_backfaces.png`, `wheel_rim_boundary_edges.png`.

---

## BUG-005 [S2][FIXED] polygon_prism: фаска (chamfer) и валидация числа граней
- **Коммит**: `c6a6ef4 fix(core): build watertight chamfered polygon prism and reject sides < 3 [BUG-005]`
- **Причина**: Построение фаски через булево пересечение цилиндрического тела приводило к потере мелких фасеток и возникновению дыр на 6/8-гранниках, а на 3/4-гранниках фаска игнорировалась. В DAG отсутствовала валидация `sides >= 3`.
- **Исправление**: 
  - Написан прямой аналитический генератор призмы с конической фаской под $30^\circ$ (по стандарту DIN 934 для гаек и болтов), соединяющий фасеточные боковые грани с наклонными трапециями фаски и концентрическими крышками.
  - В `FeatureOp::AddPolygonPrism` добавлена жесткая проверка `sides >= 3`, возвращающая `CadError::InvalidParameter`.
- **Верификация**:
  - `polygon_prism_3sides_chamfer_0.002`: status PASS, watertight, genus 0.
  - `polygon_prism_4sides_chamfer_0.002`: status PASS, watertight, genus 0.
  - `polygon_prism_6sides_chamfer_0.002`: status PASS, watertight, genus 0.
  - `polygon_prism_8sides_chamfer_0.002`: status PASS, watertight, genus 0.
  - Тест валидации `sides = 2` корректно возвращает ошибку.

---

## BUG-006 [S2][FIXED] csg: булевы операции и вырезы отверстий/канавок
- **Коммит**: `3de6a26 fix(core): eliminate T-junctions and non-manifold slits in CSG booleans [BUG-006]`
- **Причина**: Порог отсечения вырожденных треугольников `cross.norm_squared() > 1e-16` в метрах ($10^{-8}\text{ м}^2$) ошибочно удалял валидные треугольники площадью $<0.005\text{ мм}^2$. Не выполнялось сшивание коллинеарных вершин на ребрах пересечения (T-junctions).
- **Исправление**:
  - Настроен адаптивный эпсилон-порог площади с учетом геометрического масштаба детали.
  - Добавлено автоматическое разбиение смежных ребер в точках пересечения (T-junction edge subdivision) до $N=4000$ вершин.
  - Для выреза отверстий со шпоночным пазом `cut_hole_with_settings` построен герметичный конформный резец с небольшим запасом по высоте для предотвращения копланарных артефактов.
- **Верификация**:
  - CSG-цепочка двух отверстий со шпоночным пазом (genus 2): status PASS, watertight: true, boundary_edges: 0, non_manifold_edges: 0, t_junctions: 0.
  - Визуальный аудит 5 режимов: `csg_chain_two_holes_shaded.png`, `csg_chain_two_holes_wireframe.png`, `csg_chain_two_holes_normals.png`, `csg_chain_two_holes_backfaces.png`, `csg_chain_two_holes_boundary_edges.png`.

---

## BUG-007 [S2][FIXED] topology: инверсия обхода вершин при отрицательном масштабе
- **Коммит**: `c918111 fix(core): preserve outward winding and normals under negative scale [BUG-007]`
- **Причина**: При отрицательном определителе матрицы масштабирования $\det(S) = s_x s_y s_z < 0$ порядок обхода вершин сохранялся прежним, в результате чего нормали граней выворачивались внутрь, а знаковый объем становился отрицательным.
- **Исправление**: В `TriangleMesh::scale` добавлена смена порядка обхода треугольников `indices[i].swap(1, 2)` при $\det(S) < 0$, что сохраняет внешнюю ориентацию нормалей и положительный объем.
- **Верификация**: `test_bug_007_negative_scale_preserves_outward_winding` подтвердил: `signed_volume = +0.001 > 0`, 0 перевернутых граней, статус PASS.

---

## BUG-008 [S2][FIXED] cad tools: откат узла DAG при ошибке пересчета геометрии
- **Коммит**: `8a0e8bb fix(server): roll back DAG node on failed part rebuild [BUG-008]`
- **Причина**: При неудачном `part.rebuild()` добавленный узел оставался в `part.dag.nodes`, делая деталь неработоспособной для всех последующих операций.
- **Исправление**: Во всех функциях модификации детали (`add_feature_primitive`, `cut_hole`, `modify_feature`, `cut_helical_groove`, `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part`) сохраняется предварительный снимок `prev_dag` и `prev_mesh`. В случае ошибки `rebuild()` деталь мгновенно восстанавливается к исходному состоянию.
- **Верификация**: `test_bug_008_dag_rollback_on_failed_rebuild` подтвердил чистый откат и работоспособность детали после сбойной операции.

---

## BUG-009 [S2][FIXED] batch_cad_operations: атомарный откат состояния пакета операций
- **Коммит**: `13bc330 fix(server): make batch_cad_operations atomic on failure [BUG-009]`
- **Причина**: Если одна из операций внутри `batch_cad_operations` завершалась с ошибкой, созданные предыдущими шагами детали и материалы оставались в `state`, нарушая принцип атомарности.
- **Исправление**: Сохраняется полный снимок коллекций `state.parts`, `state.materials`, `state.last_fea_reports`, `state.last_fea_meshes`, `state.last_simulation` перед началом выполнения пакета. При любой ошибке в цикле все коллекции восстанавливаются из снимка до возврата ошибки клиенту.
- **Верификация**: `test_bug_009_batch_operations_atomic_rollback` подтвердил, что промежуточно созданные детали полностью удаляются при ошибке последующих шагов пакета.

---

## BUG-010 [S3][FIXED] open_preview_window и capture_viewport: строгая валидация аргументов
- **Коммит**: `66e8a81 fix(server): validate part_id in headless open_preview_window and dimensions in capture_viewport [BUG-010]`
- **Причина**: `open_preview_window` в headless-режиме возвращал `Ok({"status": "headless"})` даже для несуществующего `part_id`, а `capture_viewport` не проверял `width > 0 && height > 0`.
- **Исправление**: 
  - `open_preview_window` проверяет наличие `part_id` в `state.parts` перед ветвлением по режиму работы (headless / GUI). Если деталь отсутствует — возвращается `ServerError::PartNotFound`.
  - `capture_viewport` валидирует `width > 0 && height > 0`, возвращая `ServerError::InvalidArgument` при нулевых габаритах.
- **Верификация**: `test_bug_010_viewport_and_preview_validation` подтвердил возврат корректных ошибок.
