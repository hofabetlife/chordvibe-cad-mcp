# Отчёт по задаче P0-T9: Оптимизатор DAG: правило R4 и CBO-тест

- **ID задачи:** `P0-T9`
- **Фаза:** `P0 Foundation`
- **Размер:** `S`
- **Компонент:** `crates/mcp-cad-core`
- **Статус:** `✅ DONE`
- **Затронутые файлы:**
  - `crates/mcp-cad-core/src/query_optimizer.rs`
  - `TASKS.md`
  - `docs/agent_tasks/P0-T9-report.md` (новый)

---

## 1. Что сделано

### 1.1. Верификация правила R4 и CBO-теста
- Проверена работа правила `R4_COAXIAL_REVOLVE_REDUCTION` и сквозного теста оптимизатора `test_mcp_optimize_feature_tree_cbo` в `mcp-cad-server`.
- Подтверждено, что первопричиной их сбоя до P0-T1 являлась обратная ориентация торцевых крышек цилиндра (дефект P0-T1), из-за которого эквивалентность отвергалась из-за дельты объёма. После выполнения P0-T1 базовые тесты стали проходить.

### 1.2. Усиление проверки строгой эквивалентности (`equivalence_ok` в `query_optimizer.rs`)
В функции `optimize_feature_tree_dag` (`crates/mcp-cad-core/src/query_optimizer.rs`):
1. **Проверка качества сетки (`validate_mesh`)**:
   Добавлена проверка `mesh_validation_passed`:
   - Если исходное логическое дерево проходило валидацию (`val_before.status == "PASS"`), кандидатное физическое дерево обязано строго проходить полную валидацию сетки без единой критической ошибки (`val_after.status == "PASS"`: отсутствие не-многообразий, замкнутость, отсутствие самопересечений, вырожденных треугольников и соблюдение sliver-порогов).
   - Если исходная модель уже содержала дефекты (например, иглы от вычитания резьбовой спирали в R5), кандидатное дерево не имеет права ухудшать качество сетки (`val_after.issues.len() <= val_before.issues.len()`).
2. **Проверка топологического рода (genus)**:
   Добавлена проверка `genus_ok` (`genus_before == genus_after` для водонепроницаемых сеток), гарантирующая сохранение числа сквозных отверстий и топологической эквивалентности при переписывании. Это также устранило неиспользуемую переменную `genus_after`.
3. **Обновление статуса эквивалентности**:
   `report.equivalence_verification.passed` привязан к фактическому флагу `equivalence_ok` вместо жестко зашитого `true`.
4. **Устранение неиспользуемого присваивания**:
   Устранено фиктивное инициализирующее присваивание `let mut node_cost_ms = 0.5` в `estimate_dag_cost`, что сразу сняло 2 предупреждения компилятора (`unused_assignments` и `unused_variables`) и уменьшило количество ошибок Clippy на 2.

### 1.3. Добавление негативных тестов на неосевое отверстие
В модуль тестов `crates/mcp-cad-core/src/query_optimizer.rs` добавлен unit-тест `test_r4_negative_off_axis_hole_does_not_trigger_revolve_reduction`, проверяющий, что правило R4 **НЕ** срабатывает:
1. **Случай 1 (смещение в плоскости XY):** сквозное отверстие, параллельное оси $Z$, но смещённое от центра цилиндра на $10\,\text{мм}$ ($dx = 0.010 > 10^{-5}$):
   - `applied_rules` не содержит `R4_COAXIAL_REVOLVE_REDUCTION`;
   - В физическом плане сохраняются оба узла (`physical_nodes_count == 2`), базовый узел остаётся `AddCylinder`;
   - Выполняется стандартное вычитание через CSG.
2. **Случай 2 (поперечное отверстие):** отверстие с нормалью вдоль оси $X$ вместо $Z$ ($nz = 0 < 0.999$):
   - `applied_rules` не содержит `R4_COAXIAL_REVOLVE_REDUCTION`;
   - В физическом плане сохраняются оба узла (`physical_nodes_count == 2`), базовый узел остаётся `AddCylinder`.

---

## 2. Как проверено

Команды проверены на версии Rust `1.91.1` (`x86_64-unknown-linux-gnu`):

### 2.1. Таргетные тесты оптимизатора
```bash
cargo test -p mcp-cad-core query_optimizer
```
Результат:
```text
running 4 tests
test query_optimizer::tests::test_r2_aabb_dead_op_culling_and_r1_transform_fusion ... ok
test query_optimizer::tests::test_r4_coaxial_bushing_cylinder_hole_to_revolve ... ok
test query_optimizer::tests::test_r5_prism_thread_fusion ... ok
test query_optimizer::tests::test_r4_negative_off_axis_hole_does_not_trigger_revolve_reduction ... ok

test result: ok. 4 passed; 0 failed; 0 ignored; 0 measured; 46 filtered out; finished in 21.15s
```

### 2.2. Сквозной CBO-тест на стороне сервера
```bash
cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_optimize_feature_tree_cbo
```
Результат:
```text
running 1 test
test test_mcp_optimize_feature_tree_cbo ... ok

test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 20 filtered out; finished in 7.89s
```

### 2.3. Полный прогон тестов всего рабочего пространства
```bash
cargo test --workspace --no-fail-fast
```
Результат:
- `mcp-cad-core --lib`: 50 passed; 0 failed; 0 ignored (включая новый `test_r4_negative_off_axis_hole_does_not_trigger_revolve_reduction`).
- `mcp-cad-core --test csg_invariants`: 5 passed; 0 failed; 0 ignored.
- `mcp-cad-core --test dag_tests`: 14 passed; 0 failed; 0 ignored.
- `mcp-cad-core --test primitive_invariants`: 13 passed; 0 failed; 5 ignored.
- `mcp-cad-core --test tessellation_tests`: 6 passed; 0 failed; 0 ignored.
- `mcp-cad-fea`: 11 passed (5 lib + 6 fea_tests); 0 failed.
- `mcp-cad-physics`: 6 passed; 0 failed.
- `mcp-cad-server`: 35 passed (21 mcp_contract_tests + 13 qa_wide_scan + 1 gear_kinematic_test); 0 failed.
- `mcp-cad-viewer`: 10 passed; 0 failed.
**Итого:** 150 passed; 0 failed; 5 ignored (155 тестов).

### 2.4. TypeScript / Frontend
```bash
npm run lint
```
Результат: `tsc --noEmit` завершился с кодом 0 (0 ошибок).

---

## 3. Состояние тестов до и после

| Тестовый набор / Проверка | До (после P0-T8) | После P0-T9 | Изменение |
|---|---|---|---|
| `query_optimizer::tests` | 3 passed | 4 passed | +1 новый негативный тест |
| `test_mcp_optimize_feature_tree_cbo` | passed | passed | Подтверждена стабильность |
| `mcp-cad-core --lib` | 49 passed | 50 passed | +1 passed |
| `cargo test --workspace --no-fail-fast` | 149 passed / 5 ignored | 150 passed / 5 ignored | 0 регрессий, +1 passed |
| Ошибки `clippy` в `query_optimizer.rs` | 2 ошибки (`unused_*`) | 0 ошибок | -2 ошибки |
| Ошибки `clippy` в `mcp-cad-core (lib)` | 20 ошибок | 18 ошибок | -2 ошибки (P0-T12) |

---

## 4. Что не удалось и остаточные риски

- Нет. Все критерии приёмки P0-T9 полностью выполнены:
  - Оба теста (`test_r4_coaxial_bushing_cylinder_hole_to_revolve` и `test_mcp_optimize_feature_tree_cbo`) проходят.
  - Оптимизатор строго валидирует эквивалентность через `validate_mesh`, допуск объёма $\le 1\%$ и сохранение рода поверхности.
  - Добавлен негативный тест для обоих типов неосевых отверстий (смещение в плоскости $XY$ и поперечный угол наклона нормали).

---

## 5. Побочные находки

1. В `crates/mcp-cad-core/src/query_optimizer.rs:952` условие `*radius < cyl_radius * 0.96` отсекает коаксиальные отверстия с толщиной стенки менее 4% от радиуса цилиндра (во избежание экстремально тонких стенок). Это осознанное инженерное ограничение, его стоит сохранить.
2. Правило `R5_ANALYTICAL_PRISM_THREAD_FUSION` в настоящее время является аналитическим правилом оценки стоимости/времени симуляции без генерации B-Rep/CSG геометрии резьбы в `current_nodes`. Это задокументировано в коде правила.
