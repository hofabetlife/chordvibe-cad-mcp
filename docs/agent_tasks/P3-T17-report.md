# Отчёт по задаче P3-T17: Поддержка несоосных и наклонных винтовых канавок `cut_helical_groove` при произвольной ориентации заготовки вне оси `Z`

**Дата выполнения:** 15.11.2026  
**Статус:** ✅ DONE  

---

## 1. Цель и проблема

Аналитический манифольдный генератор `try_cut_coaxial_helical_groove` (добавленный в [`P3-T11`](P3-T11-report.md)) и смежные быстрые пути (`try_cut_coaxial_through_bore`, `try_union_coaxial_cone`) в [`crates/mcp-cad-core/src/csg.rs`](../../crates/mcp-cad-core/src/csg.rs) ранее требовали строгой соосности заготовки с мировой осью $Z$ (`normal_or_axis[2].abs() > 0.999`, центр `(0, 0)`). При смещении заготовки в плоскости `XY`/`XYZ` или её повороте (`position`/`rotation_deg` или `transform_part` / `FeatureOp::Transform` до `cut_helical_groove`) быстрый путь возвращал `None`, и операция откатывалась на общий BSP-CSG с потерей качества/водонепроницаемости, а при опущенном `start_z` в [`crates/mcp-cad-server/src/tools/cad.rs`](../../crates/mcp-cad-server/src/tools/cad.rs) использовался мировой `bounding_box.min[2]` даже для наклонённой детали.

---

## 2. Реализация

1. **Единое преобразование вершин и метаданных граней (`TriangleMesh::apply_rot_and_shift`)** — [`crates/mcp-cad-core/src/topology.rs`](../../crates/mcp-cad-core/src/topology.rs#L783-L833):
   - Выделен общий `pub(crate)` метод `apply_rot_and_shift(&mut self, rot: &nalgebra::Rotation3<f64>, t: &Vector3<f64>)`, синхронно трансформирующий `vertices` (`position`, `normal`) и реестр `faces` (`centroid`, `sample_point`, `normal_or_axis`) с сохранением всех `face_ids`, `id`, `role`, `kind`, `radius` и `alias`.
   - `rotate_rpy_deg` и `transform` (для ортонормированного случая) переведены на общий хелпер без дублирования кода.

2. **Детекция локальной СК заготовки и общий локальный конвейер** — [`crates/mcp-cad-core/src/csg.rs`](../../crates/mcp-cad-core/src/csg.rs):
   - В `HelicalGrooveParams` добавлено поле `height: f64` (из 5-го поля метаданных `helix_meta:...`) для точного определения осевого диапазона `[start_z, start_z + height]`.
   - Реализована детекция осевой симметрии заготовки `detect_coaxial_workpiece_frame`:
     - ось `axis` определяется по нормали `cap_bottom` (и проверяется согласованность с `cap_top`, если он присутствует);
     - базовое положение `s_bot` на оси проверяется относительно проекций `[s_min, s_max]` всей сетки;
     - при наличии опорной цилиндрической грани (`hole_wall` или `side`) 2D-центр оси в плоскости `(e1, e2)` вычисляется масштабно-нормированной аппроксимацией окружности по методу Косы (`fit_circle_center_2d`) с допуском радиуса `5e-3 * r_expected` (что автоматически отклоняет эллиптически масштабированные детали).
   - Добавлен общий хелпер `execute_coaxial_in_local_frame`: переносит заготовку (и инструмент `b`, если применимо) в локальную СК (`axis -> +Z`, `origin_bot -> [0, 0, target_bot_z]`), пересчитывает `merge_csg_face_registries(&a_local, b_local, is_difference)`, вызывает соответствующий `*_local` генератор (`try_cut_coaxial_through_bore_local`, `try_union_coaxial_cone_local`, `try_cut_coaxial_helical_groove_local`) и возвращает меш обратно в мировую СК через `apply_rot_and_shift(&rot_to_world, &inv_shift)`.
   - Для строго соосных с осью `Z` заготовок (`is_already_coaxial`) сохранён нулевой оверхед без трансформаций.
   - В `try_cut_coaxial_helical_groove` выбирается `target_bot_z` по максимальному перекрытию `[start_z, start_z + height]` с локальным диапазоном заготовки (`frame.origin_bot[2]` для прямостоящих смещённых заготовок при наличии перекрытия в мировом `Z`, иначе `-0.5 * axial_span` против `0.0`).
   - Добавлена публичная функция `default_helical_start_z(mesh: &TriangleMesh) -> f64` и подключена в [`crates/mcp-cad-server/src/tools/cad.rs`](../../crates/mcp-cad-server/src/tools/cad.rs#L1914-L1925): при опущенном `start_z` для прямостоящих деталей сохраняется `bounding_box().min[2]`, а для наклонённых/перевёрнутых соосных заготовок выбирается локальный `-0.5 * axial_span`.

3. **Новые тесты**:
   - [`crates/mcp-cad-core/tests/dag_tests.rs`](../../crates/mcp-cad-core/tests/dag_tests.rs):
     - `test_p3_t17_off_axis_and_tilted_hex_nut_helical_groove` — шестигранная гайка с фасками после `FeatureOp::Transform` (смещение `[25, -18, 12]`, поворот `[35, -22, 48]`), прямостоящая со сдвигом `XY`, построенная напрямую в наклонной позе (`AddPolygonPrism(pos, rot) + CutHole`) и перевёрнутая на `180°`;
     - `test_p3_t17_off_axis_and_tilted_drill_bit_helical_groove` — сверло (`AddCylinder + AddCone`) после `FeatureOp::Transform` (`[-30, 40, 15]`, `[45, 30, -15]`), горизонтальный вал `90°` и напрямую собранное горизонтальное сверло (`AddCylinder + AddCone` под `90°`);
     - во всех случаях проверяется `validate_mesh_strict` (`PASS`, `is_watertight = true`, `boundary_edges_count = 0`, `non_manifold_edges_count = 0`, `self_intersections_count = 0`, `lt_1_deg_count = 0`, `lt_5_deg_ratio <= 0.01`, `genus = 1` для гайки и `genus = 0` для сверла) и сохранность `face_ids` / `helical_groove`.
   - [`crates/mcp-cad-server/src/tools/cad.rs`](../../crates/mcp-cad-server/src/tools/cad.rs):
     - `test_p3_t17_transform_part_then_cut_helical_groove_nut_and_drill` — проверка MCP-инструментов `create_part` → `add_feature_primitive` → `cut_hole` → `transform_part` → `cut_helical_groove` → `query_part_geometry` и `validate_mesh_strict` на наклонённой гайке (авто-`start_z = -4.0` мм), прямостоящей смещённой гайке (авто-`start_z = +6.0` мм) и наклонённом сверле с конической вершиной.

---

## 3. Результаты верификации (после финальной правки кода)

| Проверка / Команда | Итог | Детали |
|---|---|---|
| `cargo clippy --workspace --all-targets -- -D warnings` | **PASS** (`0` ошибок, `0` предупреждений) | Проверены все 5 крейтов (`8.80s`) |
| `cargo test -p mcp-cad-core --no-fail-fast` | **131 passed, 0 failed** | `74` lib (`12.26s`) + `10` `csg_invariants` (`13.39s`) + `16` `dag_tests` (`5.85s`) + `25` `primitive_invariants` (`5.43s`) + `6` `tessellation_tests` (`0.02s`) |
| `$env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --lib --no-fail-fast -- --test-threads=1` | **61 passed, 0 failed** | Последовательный прогон `--lib` (`69.05s`, без `-- --test-threads=1` на Windows наблюдалось зависание как и в `P3-T23`) |
| `$env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --test qa_wide_scan --no-fail-fast -- --test-threads=1` | **13 passed, 0 failed** | Прошли все 13 сценариев (`21.22s`); побочные изменения `qa/results/` и `qa/artifacts/` откатаны |
