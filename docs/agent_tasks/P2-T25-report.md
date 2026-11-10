# Отчёт о выполнении задачи P2-T25

**Задача:** `P2-T25 [S]` — Учёт `rotation_deg` и точного радиуса $N$-угольника в `estimate_primitive_aabb` и очистка `seen_holes` после аддитивных/трансформационных узлов в правиле `R2` (`query_optimizer.rs:170-226, 538-585`)  
**Дата:** 09.11.2026  
**Статус:** ✅ DONE  
**Затронутые файлы:**
- `crates/mcp-cad-core/src/query_optimizer.rs`
- `TASKS.md`
- `docs/agent_tasks/P2-T25-report.md`

---

## 1. Диагностика проблемы

При аудите оптимизатора дерева признаков (`crates/mcp-cad-core/src/query_optimizer.rs`) в рамках **P2-T20** были выявлены два дефекта в правиле `R2_AABB_DEAD_OP_CULLING`:

1. **Неточный радиус правильной $N$-угольной призмы и игнорирование `rotation_deg` в `estimate_primitive_aabb`**:
   - Для `FeatureOp::AddPolygonPrism` радиус описанной окружности из размера «под ключ» `width_across_flats` ($w$) вычислялся по захардкоженной формуле шестиугольника `w / 3.0_f64.sqrt() * 1.15` ($\approx 0.66395 w$) вместо точной формулы правильного $N$-угольника:
     $$R = \frac{w}{2 \cos(\pi / \text{sides})}.$$
     Для треугольной (`sides = 3`, $R = w = 1.0 w$) и квадратной (`sides = 4`, $R = w / \sqrt{2} \approx 0.7071 w$) призм старая формула существенно занижала габаритный бокс `base_aabb`, из-за чего правило `R2` ошибочно отбрасывало валидные отверстия `CutHole` вблизи вершин трёх- и четырёхгранных призм как непересекающиеся (`AABB(A) ∩ AABB(B) = ∅`).
   - Для `FeatureOp::AddPolygonPrism`, `FeatureOp::AddCone` и `FeatureOp::AddRevolve` в `estimate_primitive_aabb` полностью игнорировался параметр `rotation_deg` (а для `AddRevolve` с пустым профилем возвращался некорректный интервал `[+∞, -∞]`). При повороте базового примитива (например, на $90^\circ$ вокруг оси $X$, когда высота вдоль локальной оси $Z$ переходит в ось $Y$) `base_aabb` оставался неповёрнутым, что приводило к ложному удалению отверстий, просверленных вдоль повёрнутого тела.

2. **Отсутствие очистки `seen_holes` после аддитивных и трансформационных узлов в `apply_rule_r2_aabb_dead_op_culling`**:
   - При появлении промежуточного аддитивного узла (`AddBox`, `AddCylinder`, `AddPolygonPrism`, `AddSphere`, `AddCone`, `AddSpurGear`, `AddRevolve`, `AddWheelRim`, `BooleanUnionMesh`) или пространственной трансформации `Transform` сбрасывался `base_aabb = None`, но список уже встреченных отверстий `seen_holes` не очищался (кроме того, при следующем аддитивном узле проверка `if base_aabb.is_none()` повторно инициализировала `base_aabb` только последним примитивом).
   - Аналогично в `apply_rule_r3_demorgan_csg_batch` проверка поглощения соосных отверстий не учитывала наличие промежуточных аддитивных или трансформационных узлов между отверстиями `i` и `j`.
   - В результате повторный вызов `CutHole` с теми же координатами после сдвига детали (`Transform`) или после добавления новой бобышки, перекрывшей старое отверстие, ошибочно удалялся как дубликат.

---

## 2. Выполненные изменения

1. **Точный расчёт AABB базовых примитивов с учётом `rotation_deg` (`crates/mcp-cad-core/src/query_optimizer.rs`)**:
   - Добавлена вспомогательная функция `estimate_axial_cylinder_aabb(r_radial, z_min, z_max, position, rotation_deg)`, вычисляющая точный мировой AABB осесимметричного (или ограниченного радиусом `r_radial` вокруг локальной оси $Z$ на отрезке $[z_{\min}, z_{\max}]$) тела при произвольном повороте Эйлера `nalgebra::Rotation3::from_euler_angles` и сдвиге `position`.
   - В `estimate_primitive_aabb`:
     - Для `FeatureOp::AddPolygonPrism` внедрена точная формула радиуса описанной окружности правильного $N$-угольника $R = \frac{w}{2 \cos(\pi / \text{sides})}$ (согласованная с `create_polygon_prism` в `primitives.rs`) и учёт `rotation_deg`.
     - Для `FeatureOp::AddCone`, `FeatureOp::AddRevolve` и `FeatureOp::AddCylinder` подключён расчёт через `estimate_axial_cylinder_aabb` с учётом `rotation_deg` (и защитой от пустого `profile` в `AddRevolve`).
     - Для `FeatureOp::AddBox` при ненулевом `rotation_deg` сферическая оценка по диагонали заменена на точный AABB повёрнутого параллелепипеда $e'_i = \sum_{j=0}^2 |R_{i,j}| e_j$.
2. **Сброс `seen_holes` и защита `base_aabb` при аддитивных и трансформационных операциях (`crates/mcp-cad-core/src/query_optimizer.rs`)**:
   - В `apply_rule_r2_aabb_dead_op_culling` добавлен флаг `seen_first_enabled`, гарантирующий, что `base_aabb` инициализируется только первым активным узлом дерева и не перезаписывается последующими аддитивными узлами после сброса в `None`, а при встрече любого активного аддитивного узла (`Add*`, `BooleanUnionMesh`) или `Transform` выполняется `seen_holes.clear()`.
   - В `apply_rule_r3_demorgan_csg_batch` добавлена проверка отсутствия промежуточного активного аддитивного или трансформационного узла (`has_intervening_barrier`) между индексами `i` и `j` перед поглощением соосного отверстия.
3. **Модульные тесты (`crates/mcp-cad-core/src/query_optimizer.rs`)**:
   - Добавлен тест `test_p2_t25_polygon_prism_exact_circumradius_and_rotated_primitive_aabb`: проверяет точный радиус $R = \frac{w}{2 \cos(\pi / \text{sides})}$ для `sides ∈ {3, 4, 6}`, сохранение углового отверстия на 3-гранной призме (`x = 0.032` при $w = 0.040, R = 0.040$) и сохранение отверстия на повёрнутых на $90^\circ$ вокруг оси $X$ примитивах `AddPolygonPrism`, `AddCone` и `AddRevolve`.
   - Добавлен тест `test_p2_t25_r2_clears_seen_holes_after_additive_and_transform_nodes`: проверяет сохранение повторного `CutHole([0, 0, 0])` после `Transform` (сдвиг пластины) и после аддитивного `AddCylinder` (бобышка поверх отверстия) как в `apply_rule_r2_aabb_dead_op_culling`, так и в сквозном `optimize_feature_tree_dag` (`equivalence_verification.passed == true`).

---

## 3. Верификация

- `cargo test -p mcp-cad-core --lib query_optimizer` — **7 passed; 0 failed**.
- `cargo test -p mcp-cad-core --lib` — **60 passed; 0 failed**.
- `npx --yes tsx --test src/cad/cad_engine.test.ts` — **29 passed; 0 failed**.

---

## 4. Новые найденные проблемы (заведены в `TASKS.md` со статусом `BACKLOG`)

- **`P2-T28 [S]`**: В `crates/mcp-cad-core/src/query_optimizer.rs`:
  1. В `apply_rule_r4_coaxial_revolve_reduction` (`query_optimizer.rs:941-985`) цикл `for (idx, node) in nodes.iter().enumerate().skip(1)` ищет соосное отверстие `CutHole` по всему дереву без остановки на промежуточных активных узлах `Transform`, аддитивных операциях (`Add*`, `BooleanUnionMesh`) или `BooleanIntersectionMesh`, из-за чего отверстие, просверленное после сдвига/поворота цилиндра (`Transform`), ошибочно переносится до `Transform` и сливается с корневым `AddCylinder`.
  2. В `apply_rule_r1_transform_fusion` (`query_optimizer.rs:743-764`) проталкивание чистого сдвига `Transform` в корневой примитив (`out.len() == 1`) поддерживает `AddBox`, `AddCylinder`, `AddPolygonPrism`, `AddSphere`, `AddCone` и `AddRevolve`, но пропускает `FeatureOp::AddSpurGear` и `FeatureOp::AddWheelRim`, хотя оба варианта имеют поле `position: Option<[f64; 3]>`.
  3. В `check_rule_r5_prism_thread_fusion` (`query_optimizer.rs:1047-1086`) правило `R5_ANALYTICAL_PRISM_THREAD_FUSION` срабатывает при наличии `AddPolygonPrism`, `CutHole` и `CutHelicalGroove` в любом месте дерева без проверки того, что `AddPolygonPrism` является неповёрнутым базовым узлом, а `CutHole` и `CutHelicalGroove` соосны с ним и не разделены узлом `Transform`.
