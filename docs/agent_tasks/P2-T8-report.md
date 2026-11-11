# Отчёт о выполнении задачи P2-T8: FEA — граничные условия по именам граней

- **ID задачи**: `P2-T8` (Фаза 2: Агентский Контракт)
- **Размер**: `M`
- **Зависимости**: `P2-T7` ([`docs/agent_tasks/P2-T7-report.md`](P2-T7-report.md)), `P2-T5` ([`docs/FACE_NAMING_DESIGN.md`](../FACE_NAMING_DESIGN.md))
- **Статус**: ✅ DONE
- **Дата**: 2026-11-06
- **Ветка**: `agent/P2-T8`

---

## 1. Что сделано

Реализовано задание закреплений Дирихле и внешних нагрузок Неймана в `run_fea_analysis` и `mcp-cad-fea` по именованным топологическим граням (`FaceSelector`: `{"face": "<name>"}`, `{"faces": [...]}`, `{"face_id": ...}`, `{"where": {...}}`), с проекционной привязкой поверхностных узлов тетраэдрической воксельной сетки `TetrahedralMesh` к треугольникам CAD-грани и сохранением обратной совместимости с `fixed_faces`, `fixed_plane`, `applied_forces` и `applied_force`:

1. **Проекционная привязка поверхностных узлов воксельной сетки к треугольникам CAD-грани (`crates/mcp-cad-fea/src/boundary.rs`, `crates/mcp-cad-fea/src/lib.rs`)**:
   - Расширен `DofComponent` комбинациями плоских закреплений `XY`, `XZ`, `YZ` наряду с `X`, `Y`, `Z`, `All`.
   - В `DirichletConstraint` добавлен вариант `FixedSurfaceTriangles { face_name, triangles, tolerance, components }`.
   - В `NeumannLoad` добавлены варианты:
     - `SurfaceTrianglesForce { face_name, triangles, tolerance, force_vector }` — распределённая сила (Н), пропорциональная эффективной грузовой площади каждого сматченного поверхностного узла так, что $\left\|\sum_i \mathbf{f}_i - \mathbf{F}\right\| \le 10^{-9} \|\mathbf{F}\|$;
     - `SurfaceTrianglesPressure { face_name, triangles, tolerance, pressure_pa }` — нормальное давление $p$ (Па) по локальным внешним нормалям ближайших CAD-треугольников ($\mathbf{f}_i = -p \, w_i \, \hat{\mathbf{n}}_i$ при $p > 0$ — сжатие внутрь тела);
     - `SurfaceTrianglesMoment { face_name, triangles, tolerance, moment_nm }` — крутящий/изгибающий момент $\mathbf{M}$ (Н·м) относительно центроида сматченных узлов ($\mathbf{f}_i \propto \mathbf{M} \times (\mathbf{x}_i - \mathbf{c})$ с точной нормировкой $\sum_i (\mathbf{x}_i - \mathbf{c}) \times \mathbf{f}_i = \mathbf{M}$).
   - Реализованы публичные функции `extract_face_triangles(cad_mesh, face_ids)`, `point_to_triangle_distance_3d(p, a, b, c)` (проекция точки на 3D-треугольник по 7 областям Вороного) и `match_surface_nodes_and_weights(mesh, cad_triangles, tolerance_override)`:
     - вычисляет шаг воксельной сетки $(h_x, h_y, h_z)$ и базовый допуск $\text{tol} = \max(0.95 \sqrt{h_x^2 + h_y^2 + h_z^2},\, 1.25 \max(h_x, h_y, h_z))$;
     - для плоских CAD-граней (все нормали треугольников коллинеарны с точностью $\cos\theta > 0.995$) отбирает только внешний граничный воксельный слой вдоль нормали грани (допуск $\pm 0.55 h_{\text{n}}$ от экстремального слоя проекции $\mathbf{x} \cdot \hat{\mathbf{n}}$ и ортогональный зазор $\le 0.55 \max(h_x, h_y, h_z)$), что исключает захват второго внутреннего слоя воксельных узлов и переоценку жёсткости заделки;
     - для криволинейных граней (цилиндрическая стенка отверстия `hole_wall`, боковая поверхность цилиндра/конуса `side`, сфера) отбирает поверхностные узлы воксельной оболочки в пределах 3D-расстояния до треугольников CAD-грани и взвешивает их пропорционально площади смежных граничных граней тетраэдров.
   - В `BoundaryConditions` добавлены методы `fix_surface_triangles`, `fix_named_face`, `add_surface_triangles_force`, `add_named_face_force`, `add_surface_triangles_pressure`, `add_surface_triangles_moment`.

2. **Адаптивное разрешение воксельной сетки для непрямоугольных деталей и сохранение `face_ids` при измельчении (`crates/mcp-cad-fea/src/tetra_mesh.rs`)**:
   - В `TetrahedralMesh::from_mesh` для деталей с криволинейными гранями или вырезами (объём отличается от объёма AABB более чем на $5\%$) введён минимальный порог числа ячеек по нетонким поперечным осям: не менее `8` (`Coarse`), `12` (`Medium`) и `16` (`Fine`), чтобы площадь поперечного сечения вокселизованного цилиндра или перфорированной пластины точно аппроксимировала аналитическое сечение ($\pi r^2$).
   - В `refine_mesh_max_edge_length` обеспечено сохранение `face_ids` и `faces` (`TriangleMesh::with_faces`) при 1-к-4 разбиении треугольников с пересчётом `recompute_face_registry()`.

3. **Расширение MCP-инструмента `run_fea_analysis` и структурированных ошибок (`crates/mcp-cad-server/src/tools/fea.rs`, `tools/mod.rs`, `error.rs`, `docs/MCP_API_REFERENCE.md`)**:
   - Поддержан массив закреплений `fixed`: каждый элемент принимает `FaceSelector` (`{"face": "cap_bottom"}`, `{"faces": ["left", "right"]}`, `{"face_id": 1}`, `{"where": {"kind": "cylinder", "radius": 15.0}}`, шаблоны `"hole_wall:*"` и legacy AABB `"min_z"`..`"max_z"`), опциональный массив степеней свободы `components: ["x", "y", "z"]` и опциональный `tolerance`.
   - В legacy-массиве `fixed_faces` наряду с `"min_x"`..`"max_z"` поддержаны имена топологических CAD-граней (например, `fixed_faces: ["cap_bottom"]`); при передаче несуществующего имени возвращается структурированная ошибка `FACE_NOT_FOUND` со списком доступных граней в `details.available_faces`.
   - Поддержан массив нагрузок `loads`: каждый элемент принимает `FaceSelector` (или `position` + `radius`) и один из видов нагрузки в активной системе единиц (`force` / `force_vector` в `N`, `pressure` / `pressure_mpa` в `MPa`, `moment` / `moment_nm` в `N·m`).
   - В `applied_forces` / `applied_force` также поддержано указание `face` / `faces` / `where` в качестве альтернативы точечному `position`.
   - Если для указанной грани в воксельной сетке `TetrahedralMesh` найдено `0` поверхностных узлов, возвращается структурированная ошибка `FEA_NO_BOUNDARY_NODES` с действенным `hint` и контекстом `details`.

---

## 2. Как проверено

1. Запущены модульные и интеграционные тесты `mcp-cad-fea`:
   ```bash
   cargo test -p mcp-cad-fea -- --nocapture
   ```
   - `5 passed; 0 failed` (unit-тесты `mcp-cad-fea --lib`);
   - `8 passed; 0 failed` (интеграционные тесты `tests/fea_tests.rs`, включая 2 новых теста `test_p2_t8_*`):
     - `test_p2_t8_cylindrical_shaft_axial_tension_by_named_faces`: цилиндрический стальной вал ($r = 10\text{ мм}$, $L = 100\text{ мм}$, $E = 200\text{ ГПа}$), защемлённый за грань `"cap_bottom"` (`XZ` на торце + удержание оси) и нагруженный осевой силой $F_z = 31\,415.9265\text{ Н}$ на грань `"cap_top"` — проверено совпадение суммарной узловой силы с $F_z$ с точностью $< 10^{-9}$, удлинение $\delta_z$ в пределах $< 5\%$ от $\frac{FL}{EA} = 0.05\text{ мм}$ и среднее напряжение по Мизесу в пределах $< 15\%$ от $\frac{F}{A} = 100\text{ МПа}$;
     - `test_p2_t8_plate_with_hole_bearing_load_and_pressure_on_hole_wall`: пластина с отверстием `cut_hole`, закреплённая за грань `"left"` и нагруженная силой (и давлением) на цилиндрическую стенку отверстия `"hole_wall"` — проверено точное сохранение суммы узловых сил ($< 10^{-9}$), ненулевые перемещения и напряжения, а также возврат `FeaError::NoBoundaryNodes` при нулевом числе узлов.
2. Запущены контрактные тесты `mcp-cad-server`:
   ```bash
   cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_p2_t -- --nocapture
   ```
   - `3 passed; 0 failed` (`test_mcp_p2_t6_named_faces_query_and_selector`, `test_mcp_p2_t7_csg_and_cut_hole_named_faces`, `test_mcp_p2_t8_fea_named_faces_fixed_and_loads`).
   - В `test_mcp_p2_t8_fea_named_faces_fixed_and_loads` проверены:
     - осевое растяжение цилиндрического вала через MCP `run_fea_analysis` с `fixed: [{"face": "cap_bottom"}]` и `loads: [{"face": "cap_top", "force": [0, 0, 31415.9265]}]`;
     - нагружение стенки отверстия `loads: [{"face": "hole_wall:*", "force": [5000, 0, 0]}]` и нормальным давлением `loads: [{"where": {"kind": "cylinder"}, "pressure": 25.0}]`;
     - возврат структурированной ошибки `FACE_NOT_FOUND` (с массивом `details.available_faces`) при обращении к несуществующей грани `"nonexistent_face"`.
3. Проверены сборка и линтер TypeScript/Web (`compile_applet` и `lint_applet` — 0 ошибок).

---

## 3. Тесты до/после

| Набор тестов / Проверка | До `P2-T8` | После `P2-T8` | Дельта |
|---|---|---|---|
| **`fea_tests.rs` (`mcp-cad-fea`)** | 6 тестов | **8 тестов** | **+2 новых теста `test_p2_t8_*`** |
| **`mcp_contract_tests.rs` (`mcp-cad-server`)** | 27 тестов | **28 тестов** | **+1 новый тест `test_mcp_p2_t8_*`** |
| **Всего тестов в Rust Workspace** | 196 тестов | **199 тестов** | **+3 новых теста** |
| **TypeScript (`npm run lint` / `compile_applet`)** | 0 errors | **0 errors** | Без регрессий |

---

## 4. Что не удалось и остаточные риски

- Все критерии приёмки задачи `P2-T8` выполнены в полном объёме.
- Воксельная сетка `TET4` аппроксимирует криволинейные границы ступенчатой решёткой, из-за чего локальные пиковые напряжения на ступеньках криволинейной границы выше аналитических (тогда как среднее напряжение по сечению и осевое перемещение вала укладываются в `< 2%` от аналитики). Переход на квадратичные элементы `TET10` и сглаживание/проекция границы вынесены в задачи Фазы 3 (`P3-T1`, `P3-T4`, `P3-T5`).

---

## 5. Побочные находки

1. В `crates/mcp-cad-server/tests/mcp_contract_tests.rs` тесты `test_mcp_p2_t6_named_faces_query_and_selector` и `test_mcp_p2_t7_csg_and_cut_hole_named_faces` вызывали не объявленные в файле вспомогательные функции `parse_content(&resp)` и `EngineState::new_shared()`, а `test_mcp_p2_t7` передавал `align_face: {"face": "bottom", "target_point": [0.0, 0.0, 0.0]}` в `transform_part`, где поддерживался только меж-детальный формат `{"source_face": ..., "target_part_id": ..., "target_face": ...}`. В рамках `P2-T8` добавлены хелперы `parse_content` и `parse_error_envelope` в `mcp_contract_tests.rs`, а в `transform_part` (`tools/cad.rs`) поддержаны оба формата `align_face` (выравнивание грани в целевую точку `target_point`/`to_point` и выравнивание по грани другой детали `target_part_id`/`target_face`).
2. При запуске headless WGPU-тестов `mcp_contract_tests` в контейнере без установленных пакетов `libvulkan1` и `mesa-vulkan-drivers` вызов `render_part_views` возвращал `Viewer(NoSuitableAdapter)`. Установка `libvulkan1 mesa-vulkan-drivers libegl1 libgl1-mesa-dri` (согласно `Dockerfile`) обеспечивает работу программного драйвера Vulkan `llvmpipe`/`lavapipe`.
