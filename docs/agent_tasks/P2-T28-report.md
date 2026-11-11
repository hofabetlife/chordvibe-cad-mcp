# Отчёт о выполнении задачи P2-T28

**Задача:** `P2-T28 [S]` — Остановка поиска соосного отверстия на барьерных узлах в правиле `R4`, проталкивание сдвига в `AddSpurGear`/`AddWheelRim` в `R1` и проверка соосности в `R5` (`query_optimizer.rs:804-825, 962-1007, 1068-1107`)  
**Дата:** 09.11.2026  
**Статус:** ✅ DONE  
**Затронутые файлы:**
- `crates/mcp-cad-core/src/query_optimizer.rs`
- `TASKS.md`
- `docs/agent_tasks/P2-T28-report.md`

---

## 1. Диагностика проблемы

В ходе выполнения **P2-T25** ([`docs/agent_tasks/P2-T25-report.md`](P2-T25-report.md)) в модуле оптимизатора дерева признаков (`crates/mcp-cad-core/src/query_optimizer.rs`) были выявлены три дефекта в алгебраических правилах перезаписи `R4`, `R1` и `R5`:

1. **Отсутствие остановки на барьерных узлах в `apply_rule_r4_coaxial_revolve_reduction` (`query_optimizer.rs:990-1129`):**
   - Поиск соосного сквозного отверстия `CutHole` для слияния с корневым `AddCylinder` в единое кольцевое тело вращения `AddRevolve` выполнялся по всему списку `nodes.iter().enumerate().skip(1)` без остановки на промежуточных активных барьерных узлах (`Transform`, аддитивных `Add*`/`BooleanUnionMesh` или `BooleanIntersectionMesh`).
   - Если между корневым `AddCylinder([0, 0, 0])` и последующим `CutHole([0, 0, 0])` находился узел `Transform` (сдвигающий или поворачивающий цилиндр) или аддитивный узел (например, `AddBox` фланца, перекрывающий ось $Z$), правило `R4` пыталось перенести `CutHole` через барьер в корневой цилиндр, что либо меняло геометрию (и вызывало откат по `equivalence_verification`), либо оставляло промежуточную бобышку непрорезанной.

2. **Пропуск `AddSpurGear` и `AddWheelRim` при проталкивании чистого сдвига в `apply_rule_r1_transform_fusion` (`query_optimizer.rs:806-828`):**
   - При проталкивании чистого сдвига (`Transform` с нулевым вращением и единичным масштабом) в единственный активный корневой примитив обрабатывались `AddBox`, `AddCylinder`, `AddPolygonPrism`, `AddSphere`, `AddCone` и `AddRevolve`, но пропускались `FeatureOp::AddSpurGear` и `FeatureOp::AddWheelRim`, хотя оба варианта имеют поле `position: Option<[f64; 3]>` и обрабатываются через `apply_pos_rot` в `FeatureTree::evaluate_with_settings`.

3. **Отсутствие проверки соосности, отсутствия поворота и барьерных узлов в `check_rule_r5_prism_thread_fusion` (`query_optimizer.rs:1132-1171`):**
   - Правило `R5_ANALYTICAL_PRISM_THREAD_FUSION` срабатывало при наличии в `nodes` любых активных `AddPolygonPrism`, `CutHole` и `CutHelicalGroove` (`nodes.iter().any(...)`), не проверяя, что:
     - `AddPolygonPrism` является первым активным (корневым) узлом дерева, не повёрнут (`rotation_deg == [0, 0, 0]`) и расположен соосно с мировой осью $Z$ (`position.x == 0, position.y == 0`, вдоль которой действует `CutHelicalGroove`);
     - `CutHole` соосно с призмой вдоль оси $Z$, не имеет шпоночного паза, имеет радиус внутри описанного радиуса призмы и прорезает её высоту;
     - `CutHelicalGroove` имеет радиус резьбы внутри призмы и перекрывает её по оси $Z$;
     - между `AddPolygonPrism`, `CutHole` и `CutHelicalGroove` нет промежуточных барьерных узлов (`Transform`, аддитивных `Add*`/`BooleanUnionMesh` или `BooleanIntersectionMesh`).

---

## 2. Выполненные изменения

1. **Остановка на барьерных узлах в `apply_rule_r4_coaxial_revolve_reduction` (`crates/mcp-cad-core/src/query_optimizer.rs`):**
   - Поиск корневого цилиндра привязан к первому активному узлу `nodes.iter().enumerate().find(|(_, n)| n.enabled)`.
   - В цикле поиска соосного `CutHole` добавлена немедленная остановка (`break`) при встрече любого активного барьерного узла (`FeatureOp::Transform`, любого аддитивного `FeatureOp::Add*`, `FeatureOp::BooleanUnionMesh` или `FeatureOp::BooleanIntersectionMesh`). При этом коммутирующие субтрактивные операции (например, внеосевой `CutHole` перед центральным отверстием) корректно пропускаются.

2. **Проталкивание чистого сдвига в `AddSpurGear` и `AddWheelRim` в `apply_rule_r1_transform_fusion` (`crates/mcp-cad-core/src/query_optimizer.rs`):**
   - В ветку проталкивания чистого сдвига в единственный активный корневой примитив добавлены варианты `FeatureOp::AddSpurGear { position, .. }` и `FeatureOp::AddWheelRim { position, .. }`.
   - Добавлена защита от повторного дублирования суффикса `" (Fused Translation)"` в имени корневого узла при слиянии нескольких последовательных сдвигов.

3. **Проверка корневой неповёрнутой призмы, соосности и отсутствия барьеров в `check_rule_r5_prism_thread_fusion` (`crates/mcp-cad-core/src/query_optimizer.rs`):**
   - Проверяется, что первый активный узел дерева — `FeatureOp::AddPolygonPrism` с нулевым поворотом (`|rot| <= 1e-4`) и центром на оси $Z$ (`|pos.x| <= 1e-5, |pos.y| <= 1e-5`).
   - Вычисляется точный описанный радиус призмы $R_{\text{outer}} = \frac{w}{2 \cos(\pi / \text{sides})}$ (или `radius`) и её интервал по оси $Z$.
   - При сканировании последующих активных узлов цикл прерывается (`break`) на любом барьерном узле (`Transform`, `Add*`, `BooleanUnionMesh`, `BooleanIntersectionMesh`), а для `CutHole` и `CutHelicalGroove` проверяются соосность с призмой вдоль оси $Z$, попадание радиуса внутрь $R_{\text{outer}}$ и перекрытие по высоте.

---

## 3. Верификация и тесты

В `crates/mcp-cad-core/src/query_optimizer.rs` добавлены 3 новых модульных теста:
- `test_p2_t28_r4_stops_at_barrier_nodes` — проверяет, что `R4` не переносит `CutHole` через `Transform`, аддитивный `AddBox` или `BooleanIntersectionMesh`, но корректно сворачивает соосное отверстие после коммутирующего внеосевого `CutHole`.
- `test_p2_t28_r1_pushes_translation_into_spur_gear_and_wheel_rim` — проверяет проталкивание сдвига `Transform` в `AddSpurGear.position` и `AddWheelRim.position` с полной эквивалентностью объёма (`volume_delta_pct < 0.01%`).
- `test_p2_t28_r5_validates_coaxiality_unrotated_root_and_no_intervening_transform` — проверяет срабатывание `R5` на валидной соосной гайке и отклонение `R5` при повёрнутой/смещённой призме, внеосевом/поперечном отверстии, промежуточном `Transform` или некорневой призме.

Результаты запуска тестов:
- `cargo test -p mcp-cad-core --lib query_optimizer` — **10 passed; 0 failed**.
- `cargo test -p mcp-cad-core --lib` — **64 passed; 0 failed**.
- `npx --yes tsx --test src/cad/cad_engine.test.ts` — **31 passed; 0 failed**.

---

## 4. Новые найденные проблемы (заведены в `TASKS.md` со статусом `BACKLOG`)

- **`P2-T31 [S]`**: В `crates/mcp-cad-core/src/query_optimizer.rs`:
  1. В `apply_rule_r3_demorgan_csg_batch` (`query_optimizer.rs:864-878`) список `subtract_indices` собирает все узлы `CutHole` и `BooleanDifferenceMesh` по всему дереву без учёта разделяющих их барьерных узлов (`Transform`, аддитивных `Add*`, `BooleanUnionMesh`, `BooleanIntersectionMesh`) и при `subtract_indices.len() >= 2` безусловно возвращает `Some(AppliedRewriteRule { rule_id: "R3_DEMORGAN_CSG_BATCH", .. })` даже тогда, когда вычитания разделены `Transform` или новой бобышкой (где группировка Де Моргана $(A \setminus B_1) \setminus B_2 \mapsto A \setminus (B_1 \cup B_2)$ неприменима) и ни одно отверстие не было поглощено (`subsumed_count == 0`), что приводит к ложному срабатыванию правила `R3` и холостому инкременту `part.revision` при `optimize_dag(true)`.
  2. В `estimate_primitive_aabb` (`query_optimizer.rs:166-293`) отсутствует аналитическая оценка AABB для `FeatureOp::AddSpurGear` и `FeatureOp::AddWheelRim` (возвращается `None`, из-за чего правило `R2` не отсекает промахи `CutHole` мимо шестерни или колёсного обода), а в `estimate_dag_cost` (`query_optimizer.rs:413-450, 483-491`) для `AddSphere`, `AddCone`, `AddSpurGear` и `AddWheelRim` при `current_tris > 0` (когда они идут вторым или последующим аддитивным узлом и вычисляются через `csg_union`) не учитываются `bsp_depth += 1`, `full_mesh_passes += 2` и `sliver_risk`.
