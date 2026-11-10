# Отчёт о выполнении задачи P2-T20

**Задача:** `P2-T20 [S]` — Учёт явных `segments`/`sectors`/`stacks` и дуги `angle_deg` (`AddRevolve`) в аналитической модели стоимости `estimate_dag_cost` (`query_optimizer.rs:310-395`)  
**Дата:** 08.11.2026  
**Статус:** ✅ DONE  
**Затронутые файлы:**
- `crates/mcp-cad-core/src/dag.rs`
- `crates/mcp-cad-core/src/query_optimizer.rs`
- `TASKS.md`
- `docs/agent_tasks/P2-T20-report.md`

---

## 1. Диагностика проблемы

При выполнении **P2-T14** ([`docs/agent_tasks/P2-T14-report.md`](P2-T14-report.md)) в `FeatureTree::evaluate_with_settings` (`crates/mcp-cad-core/src/dag.rs`) были внедрены функции разрешения параметров дискретизации `resolve_radial_segments`, `resolve_revolve_segments` и `resolve_sphere_resolution`, учитывающие явно заданные в узле DAG параметры `segments` / `sectors` / `stacks` и угловую дугу `angle_deg` для `AddRevolve`.

Однако в аналитической модели стоимости оптимизатора запросов `estimate_dag_cost` (`crates/mcp-cad-core/src/query_optimizer.rs:310-395`):
1. Для `FeatureOp::AddCylinder` и `FeatureOp::AddCone` вызывался `settings.calculate_segments(...)` с полным игнорированием поля `segments` узла DAG.
2. Для `FeatureOp::AddSphere` вызывался `settings.calculate_sphere_resolution(*radius)` с игнорированием полей `sectors` и `stacks`.
3. Для `FeatureOp::AddRevolve` вызывался `settings.calculate_segments(max_r)` (для полного круга $360^\circ$) с игнорированием как поля `segments`, так и фактического угла вращения `angle_deg` (`calculate_segments_arc`) и торцевых крышек неполного сектора (`angle_deg < 360.0`).

В результате оценка числа треугольников `estimated_triangles` (`prim_tris`), времени выполнения `estimated_time_ms` и дерева `EXPLAIN` в `estimate_dag_cost` расходилась с фактической тесселяцией в `FeatureTree::evaluate_with_settings`.

---

## 2. Выполненные изменения

1. **Переиспользование функций разрешения тесселяции из `dag.rs` (`crates/mcp-cad-core/src/dag.rs`)**:
   - Функции `resolve_radial_segments`, `resolve_revolve_segments` и `resolve_sphere_resolution` экспортированы внутри крейта как `pub(crate)`.
2. **Синхронизация `estimate_dag_cost` (`crates/mcp-cad-core/src/query_optimizer.rs`)**:
   - В `FeatureOp::AddCylinder`: расчёт `segs` переведён на `resolve_radial_segments(*segments, *radius, 8, settings)`.
   - В `FeatureOp::AddCone`: расчёт `segs` переведён на `resolve_radial_segments(*segments, radius_bottom.max(*radius_top), 3, settings)`.
   - В `FeatureOp::AddSphere`: расчёт `(sec, stk)` переведён на `resolve_sphere_resolution(*sectors, *stacks, *radius, settings)`, а в строку плана `EXPLAIN` добавлены параметры `UvSphereScan(sec={sec}, stk={stk}, tris={prim_tris})`.
   - В `FeatureOp::AddRevolve`: расчёт `segs` переведён на `resolve_revolve_segments(*segments, max_r, *angle_deg, settings)` (учитывающий как явный `segments`, так и дугу `angle_deg` через `calculate_segments_arc`), добавлен учёт треугольников двух плоских торцевых крышек при неполном угле вращения (`is_partial`: `(profile.len() - 2) * 2`), а в строку `EXPLAIN` добавлен параметр `segs={segs}`.
3. **Модульные тесты (`crates/mcp-cad-core/src/query_optimizer.rs`)**:
   - Добавлен unit-тест `test_p2_t20_estimate_dag_cost_honors_explicit_segments_and_revolve_angle`, проверяющий учёт явных `segments` на `AddCylinder` (`16` vs `64`), `AddCone` (`12` vs `48`), `AddRevolve` (`12` vs `48`), явных `(sectors, stacks)` на `AddSphere` (`(12, 8)` vs `(48, 24)`), а также масштабирование числа сегментов и треугольников по углу дуги `angle_deg` (`90°` vs `360°`) при авто-тесселяции `AddRevolve`.

---

## 3. Новые найденные проблемы (заведены в `TASKS.md` со статусом `BACKLOG`)

- **`P2-T25 [S]`**: В `crates/mcp-cad-core/src/query_optimizer.rs`:
  1. Функция `estimate_primitive_aabb` (`query_optimizer.rs:170-226`) для `FeatureOp::AddPolygonPrism` вычисляет описанный радиус из `width_across_flats` по формуле шестиугольника `w / 3.0_f64.sqrt() * 1.15` вместо точной формулы правильного $N$-угольника $R = \frac{w}{2 \cos(\pi / \text{sides})}$ (для `sides = 3` и `sides = 4` множитель больше, чем `1.15 / √3`), а для `AddPolygonPrism`, `AddCone` и `AddRevolve` игнорирует `rotation_deg`, что при повёрнутом базовом примитиве может приводить к неверному AABB в правиле `R2` (`apply_rule_r2_aabb_dead_op_culling`).
  2. В `apply_rule_r2_aabb_dead_op_culling` (`query_optimizer.rs:538-585`) при появлении промежуточной аддитивной операции или `Transform` сбрасывается `base_aabb = None`, но не очищается список `seen_holes`, из-за чего повторное сверление отверстия с теми же координатами после добавления новой бобышки или сдвига детали ошибочно отбрасывается как дубликат.
