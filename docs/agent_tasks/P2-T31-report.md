# Отчёт о выполнении задачи P2-T31

**Задача:** `P2-T31 [S]` — Учёт барьерных узлов при группировке вычитаний в правиле `R3`, оценка AABB для `AddSpurGear`/`AddWheelRim` в `estimate_primitive_aabb` и учёт `current_tris > 0` в `estimate_dag_cost` (`query_optimizer.rs`)  
**Дата:** 10.11.2026  
**Статус:** ✅ DONE  
**Затронутые файлы:**
- `crates/mcp-cad-core/src/query_optimizer.rs`
- `TASKS.md`
- `docs/agent_tasks/P2-T31-report.md`

---

## 1. Диагностика проблемы

В ходе выполнения **P2-T28** ([`docs/agent_tasks/P2-T28-report.md`](P2-T28-report.md)) в оптимизаторе DAG (`crates/mcp-cad-core/src/query_optimizer.rs`) были выявлены следующие дефекты:

1. **Ложное срабатывание правила `R3` через барьерные узлы (`query_optimizer.rs:980-1108`):**
   - Метод `apply_rule_r3_demorgan_csg_batch` собирал все активные субтрактивные операции (`CutHole`, `BooleanDifferenceMesh`) в единый плоский вектор `subtract_indices` без разделения на связные сегменты.
   - Если дерево содержало $\ge 2$ вычитаний, разделённых барьерными узлами (`Transform`, аддитивными `Add*`, `BooleanUnionMesh`, `BooleanIntersectionMesh`), и ни одно отверстие не было поглощено (`subsumed_count == 0`), правило $R3$ всё равно безусловно возвращало `Some(AppliedRewriteRule { rule_id: "R3_DEMORGAN_CSG_BATCH", .. })`, хотя группировка Де Моргана $(A \setminus B_1) \setminus B_2 \mapsto A \setminus (B_1 \cup B_2)$ через промежуточные трансформации или добавленные бобышки математически некорректна.
   - Это приводило к ложному отчёту об оптимизации и холостому инкременту `part.revision` при вызове `optimize_dag(true)`.

2. **Отсутствие аналитической оценки AABB для `AddSpurGear` и `AddWheelRim` (`query_optimizer.rs:166-293`):**
   - В `estimate_primitive_aabb` ветки `FeatureOp::AddSpurGear` и `FeatureOp::AddWheelRim` попадали в `_ => None`.
   - В результате правило `R2_AABB_DEAD_OP_CULLING` не могло определить габариты шестерни или обода колеса и пропускало холостые отверстия `CutHole`, расположенные далеко за пределами детали.

3. **Отсутствие ветки `current_tris > 0` (`BspUnionJoin`) в модели стоимости `estimate_dag_cost` (`query_optimizer.rs:413-491`):**
   - Для `AddSphere`, `AddCone`, `AddSpurGear`, `AddRevolve` и `AddWheelRim` при `current_tris > 0` (когда они являются вторым или последующим аддитивным телом, объединяемым через CSG-юнион `csg_union`) модель стоимости считала их как первичный аналитический скан (`full_mesh_passes += 1`) вместо дорогой BSP-булевой операции объединения (`bsp_depth += 1`, `full_mesh_passes += 2`, штраф за `sliver_risk`).

---

## 2. Выполненные изменения

1. **Разбиение субтрактивных операций на непрерывные сегменты в `apply_rule_r3_demorgan_csg_batch`:**
   - Реализована группировка активных узлов `CutHole` и `BooleanDifferenceMesh` в связные сегменты `contiguous_segments: Vec<Vec<usize>>`, разделяемые любыми барьерными узлами (`FeatureOp::Transform`, `FeatureOp::Add*`, `FeatureOp::BooleanUnionMesh`, `FeatureOp::BooleanIntersectionMesh`).
   - Поглощение соосных отверстий (`subsumed_count`) дополнено проверкой отсутствия барьерных узлов (включая `BooleanIntersectionMesh`).
   - После удаления поглощённых отверстий подсчитывается число оставшихся операций в каждом связном сегменте. Пакетная оптимизация Де Моргана применяется только к сегментам длиной $\ge 2$.
   - Если ни в одном сегменте нет $\ge 2$ вычитаний и `subsumed_count == 0`, правило `R3` возвращает `(nodes.to_vec(), None)`, предотвращая ложные срабатывания и холостой инкремент ревизии детали.

2. **Аналитическая оценка AABB для `AddSpurGear` и `AddWheelRim` в `estimate_primitive_aabb`:**
   - Для `AddSpurGear`: вычислен максимальный вершинный радиус $r_a = m \cdot z / 2 + m$, осевой интервал ступицы $[-h_2, \max(h_2, -h_2 + L_{\text{hub}})]$ и сформирован ограничивающий цилиндр с учётом мировой позиции `position` и углов Эйлера `rotation_deg` через `estimate_axial_cylinder_aabb`.
   - Для `AddWheelRim`: вычислен максимальный радиус по закраине $r_{\text{max}} = D_{\text{flange}} / 2$, осевой интервал $[-w/2, w/2]$ и мировая рамка AABB с учётом поворота и смещения.
   - Теперь правило `R2` успешно отсекает любые непересекающиеся `CutHole` на зубчатых колёсах и колёсных дисках.

3. **Учёт булева объединения (`BspUnionJoin`) при `current_tris > 0` в `estimate_dag_cost`:**
   - Для `AddSphere`, `AddCone`, `AddSpurGear`, `AddRevolve`, `AddWheelRim` и `AddPolygonPrism` добавлено ветвление `if current_tris == 0 { ... } else { ... }`.
   - При `current_tris > 0` корректно увеличиваются `bsp_depth += 1`, `full_mesh_passes += 2`, рассчитываются квадратичные затраты на BSP-пересечение с учётом `bsp_depth`, увеличивается оценка риска `sliver_risk` и проставляется дескриптор `BspUnionJoin(..., depth=...)`.

---

## 3. Верификация и тесты

В `crates/mcp-cad-core/src/query_optimizer.rs` добавлены 3 новых модульных теста:
- `test_p2_t31_r3_does_not_trigger_when_subtractions_separated_by_barriers_and_no_subsumption` — проверяет, что:
  - 2 отверстия, разделённые `Transform` $\to$ `r3_opt.is_none()`;
  - 2 отверстия, разделённые аддитивной бобышкой `AddCylinder` $\to$ `r3_opt.is_none()`;
  - 2 отверстия в одном непрерывном сегменте $\to$ `r3_opt.is_some()`, применяется Де Морган;
  - 2 соосных отверстия без барьера $\to$ меньшее поглощается (`subsumed_count == 1`), узел удаляется из дерева.
- `test_p2_t31_estimate_primitive_aabb_for_spur_gear_and_wheel_rim_and_r2_culling` — проверяет точные аналитические границы AABB для `AddSpurGear` и `AddWheelRim` со смещениями, а также отсечение непересекающихся отверстий правилом `R2`.
- `test_p2_t31_estimate_dag_cost_joins_sphere_cone_gear_wheel_rim` — проверяет ветвление `current_tris > 0` (`BspUnionJoin`, `bsp_depth = 1`, `full_mesh_passes = 3`) для всех составных аддитивных тел.

Результаты запуска тестов:
- `cargo test -p mcp-cad-core --lib query_optimizer`: **13 passed; 0 failed (100% PASS)**.
- TypeScript/Frontend (`npx tsx --test src/cad/cad_engine.test.ts`): **33 passed; 0 failed (100% PASS)**.
- `compile_applet`: **Build succeeded**.
- `lint_applet` (`tsc --noEmit`): **0 errors**.
