# Отчёт о выполнении задачи P2-T14

**Задача:** `P2-T14 [S]` — Учёт явных `segments`/`sectors`/`stacks` в `FeatureTree::evaluate_with_settings` (`dag.rs`) и связности/относительного допуска в `inspect_geometry` (`topology.rs`)  
**Дата:** 07.11.2026  
**Статус:** ✅ DONE  
**Затронутые файлы:**
- `crates/mcp-cad-core/src/dag.rs`
- `crates/mcp-cad-core/src/topology.rs`
- `TASKS.md`
- `docs/agent_tasks/P2-T14-report.md`

---

## 1. Описание проблемы

При аудите сигнатур и реализации `mcp-cad-core` в рамках **P1-T4** были выявлены две проблемы:

1. **Игнорирование явных `segments` / `sectors` / `stacks` в `FeatureTree::evaluate_with_settings` (`crates/mcp-cad-core/src/dag.rs`)**:
   - Ветви `FeatureOp::AddCylinder`, `FeatureOp::AddCone` и `FeatureOp::AddRevolve` деструктурировали поле `segments: _`, а `FeatureOp::AddSphere` — `sectors: _, stacks: _`, безусловно перезаписывая их адаптивным расчётом `settings.calculate_segments(...)` / `settings.calculate_sphere_resolution(...)` / `create_revolve_with_settings(...)`.
   - Из-за этого явная передача `"segments"` / `"sectors"` / `"stacks"` в `FeatureOp` (или через MCP-инструменты `add_feature_primitive` / `modify_feature`) сохранялась в узле DAG, но полностью игнорировалась при построении треугольной сетки.

2. **Фиксированный абсолютный плоскостной допуск и отсутствие связности в `TriangleMesh::inspect_geometry` (`crates/mcp-cad-core/src/topology.rs`)**:
   - Метод `TriangleMesh::inspect_geometry` кластеризовал треугольники в `detected_planar_faces` только по близости нормали (`dot > 0.999`) и фиксированному абсолютному допуску `|n · p - d| < 1e-4` (`0.1 мм` в СИ-метрах) без проверки связности по вершинам/рёбрам.
   - В результате:
     - На малоразмерных деталях в метрах ступеньки высотой `< 0.1 мм` (например, `20 мкм` на `2 мм` детали) ошибочно сливались в одну плоскую грань.
     - Пространственно раздельные копланарные площадки/бобышки на одной высоте `Z` сливались в единственный `PlanarFaceInfo` с центром в пустом промежутке между ними.
     - На кольцевых торцах с центральным сквозным отверстием (например, втулка `create_revolve`) взвешенный по площади центроид `[0, 0, z]` попадал в пустоту отверстия вместо поверхности грани.

---

## 2. Выполненные изменения

1. **Учёт явных параметров тесселяции в `crates/mcp-cad-core/src/dag.rs`**:
   - Добавлены функции разрешения параметров дискретизации `resolve_radial_segments`, `resolve_revolve_segments` и `resolve_sphere_resolution`:
     - При `segments == 0` (маркер авто-тесселяции) используется адаптивный расчёт `settings.calculate_segments(...)` / `calculate_segments_arc(...)` / `calculate_sphere_resolution(...)`.
     - При стандартных настройках `TessellationSettings::default()` используются явно заданные в узле `segments` (с нижней границей `8` для цилиндра и `3` для конуса/revolve) и `(sectors.max(3), stacks.max(2))` для сферы.
     - При явном запросе пресета качества (`Coarse`, `Fine`, `Custom`), отличного от `TessellationSettings::default()`, значения по умолчанию масштабируются адаптивно от `settings`, а пользовательские нестандартные `segments`/`sectors`/`stacks` пропорционально масштабируются относительно базовой плотности.
   - В `FeatureOp::AddRevolve` вызов `create_revolve_with_settings` заменён на `create_revolve(profile, *angle_deg, segs)` с вычисленным `segs`.

2. **Относительный плоскостной допуск, связность компонент и привязка `center` к поверхности грани в `crates/mcp-cad-core/src/topology.rs`**:
   - Плоскостной допуск в `TriangleMesh::inspect_geometry` привязан к диагонали габаритного бокса детали: `plane_tol = (diag * 1e-5).max(1e-12)`, а порог фильтрации вырожденных треугольников — `min_tri_area = (diag * diag * 1e-12).max(1e-18)`.
   - Добавлена пространственная сварка совпадающих 3D-вершин с допуском `vertex_weld_tol = (diag * 1e-6).max(1e-12)` по 27 соседним ячейкам `(-1..=1)^3` и разбиение каждой копланарной группы на связные компоненты через систему непересекающихся множеств (`uf_find` / `uf_union`) по общим сваренным вершинам и T-стыкам (`point_to_segment_dist_sq <= vertex_weld_tol_sq`).
   - Добавлена проверка принадлежности центроида `raw_ctr` поверхности компоненты через `closest_point_on_triangle`: если центроид попадает в отверстие/вырез (`min_dist_sq > (plane_tol * 2.0)^2`), `center` проецируется вдоль радиального луча на середину сечения кольцевой грани и привязывается к ближайшему треугольнику компоненты.

3. **Модульные тесты**:
   - В `crates/mcp-cad-core/src/dag.rs`: добавлен тест `test_tree_explicit_segments_and_resolution_honored` (проверка явных `segments` на `AddCylinder`, `AddCone`, `AddRevolve` и `sectors`/`stacks` на `AddSphere`).
   - В `crates/mcp-cad-core/src/topology.rs`: добавлены тесты `test_inspect_geometry_disjoint_coplanar_pads_and_micro_steps` (разделение двух копланарных площадок на одной высоте и микро-ступеньки `20 мкм` на `2 мм` детали) и `test_inspect_geometry_annular_face_center_lies_on_surface` (проверка того, что `center` кольцевого торца втулки лежит на кольцевой поверхности `[r_in, r_out]`, а не в отверстии).

---

## 3. Новые найденные проблемы (добавлены в `TASKS.md`)

- **`P2-T20` [S]**: В `crates/mcp-cad-core/src/query_optimizer.rs:310-395` функция аналитической оценки стоимости `estimate_dag_cost` для `FeatureOp::{AddCylinder, AddCone, AddSphere, AddRevolve}` игнорирует явно заданные в узле `segments`/`sectors`/`stacks` (а для `AddRevolve` вызывает `settings.calculate_segments(max_r)` без учёта неполного угла `angle_deg < 360.0`), из-за чего оценка числа треугольников `prim_tris` расходится с фактической оценкой `FeatureTree::evaluate_with_settings`.
