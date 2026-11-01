

## Итерация 2 — Фаза 2 (Завершение полного сканирования операций, аналитических эталонов и матрицы 26 инструментов)
- **Дата/время**: 2026-10-30T00:18:00Z
- **Выполнено**:
  - Расширен `crates/mcp-cad-server/tests/qa_wide_scan.rs` (4 тестовых набора: `test_wide_scan_primitives_mesh_quality`, `test_wide_scan_operations_and_chains`, `test_wide_scan_analytical_benchmarks`, `test_wide_scan_functional_matrix_26_tools`).
  - Собраны полные JSON-отчеты в `qa/results/primitives_mesh_scan.json` (37 конфигураций), `qa/results/operations_mesh_scan.json` (11 конфигураций операций и цепочек), `qa/results/analytical_benchmarks.json` (6 аналитических бенчмарков геометрии и FEA), `qa/results/functional_matrix_scan.json`.
  - Проведена глубокая диагностика корневых причин всех обнаруженных дефектов (BUG-001 .. BUG-010).
- **Обнаруженные баги**:
  - S2: `BUG-002` (`spur_gear`), `BUG-003` (`revolve`), `BUG-004` (`wheel_rim`), `BUG-005` (`polygon_prism`), `BUG-006` (`csg`), `BUG-007` (`topology::scale`), `BUG-008` (`cad` DAG rollback), `BUG-009` (`batch_cad_operations` atomicity).
  - S3: `BUG-010` (`open_preview_window` part_id check in headless mode).
  - S4: `BUG-001` (`docs/ARCHITECTURE.md` tool count).
- **Переход**: Фаза 2 полностью завершена. Переход в Фазу 3 (Итеративный цикл исправления багов по приоритету S2 -> S3 -> S4).

---

## Итерация 3 — Фаза 3: Исправление BUG-003 (`revolve`)
- **Дата/время**: 2026-10-30T01:30:00Z
- **Коммит**: `3f0ee41 fix(core): fix partial revolve end-cap winding and analytical normals [BUG-003]`
- **Суть**: Устранена инверсия порядка обхода вершин на плоских торцевых крышках частичного вращения (<360°), вычислены аналитические нормали.
- **Верификация**: `test_bug_003_partial_revolve_mesh_and_renders`, сгенерированы 5 артефактов PNG. Inverted edges = 0, status = PASS.

---

## Итерация 4 — Фаза 3: Исправление BUG-007 (`topology::scale`)
- **Дата/время**: 2026-10-30T02:00:00Z
- **Коммит**: `c918111 fix(core): preserve outward winding and normals under negative scale [BUG-007]`
- **Суть**: При отрицательном детерминанте матрицы масштабирования добавлен разворот треугольников `tri.swap(1, 2)`.
- **Верификация**: `test_bug_007_negative_scale_preserves_outward_winding`. `signed_volume` > 0, 0 перевернутых граней, status = PASS.

---

## Итерация 5 — Фаза 3: Исправление BUG-005 (`polygon_prism`)
- **Дата/время**: 2026-10-30T02:45:00Z
- **Коммит**: `c6a6ef4 fix(core): build watertight chamfered polygon prism and reject sides < 3 [BUG-005]`
- **Суть**: Прямой аналитический генератор DIN 934 конической фаски (30°) для призм с любым числом граней $\ge 3$. Введена валидация `sides >= 3` в `dag.rs`.
- **Верификация**: `test_bug_005_polygon_prism_chamfer_and_sides_validation`. Все призмы 3, 4, 6, 8 граней водонепроницаемы (0 дыр), `sides < 3` отклоняются с ошибкой.

---

## Итерация 6 — Фаза 3: Исправление BUG-002 (`spur_gear`)
- **Дата/время**: 2026-10-30T03:30:00Z
- **Коммит**: `6e1dc66 fix(core): construct watertight manifold spur gear with hub, bore, and keyway [BUG-002]`
- **Суть**: Устранена веерная сингулярность при $r_f < r_b$ через концентрический триангулятор; построена единая герметичная топология венца, ступицы, отверстия и шпоночного паза DIN 6885.
- **Верификация**: `test_bug_002_spur_gear_manifold_and_keyway`. 0 вырожденных треугольников, Euler genus 1, 0 граничных ребер. Сгенерированы 5 диагностических рендеров.

---

## Итерация 7 — Фаза 3: Исправление BUG-004 (`wheel_rim`)
- **Дата/время**: 2026-10-30T04:15:00Z
- **Коммит**: `5874aa7 fix(core): build unified watertight manifold wheel rim mesh [BUG-004]`
- **Суть**: Полностью переписан генератор колесного диска: обод ограничен внутренним монтажным ручьем, спицы не выступают наружу, ступица и крепежные отверстия топологически сшиты в единый B-Rep меш.
- **Верификация**: `test_bug_004_wheel_rim_manifold_and_renders`. 0 граничных ребер (было 17956), 0 самопересечений. Сгенерированы 5 диагностических рендеров.

---

## Итерация 8 — Фаза 3: Исправление BUG-006 (`csg booleans`)
- **Дата/время**: 2026-10-30T05:30:00Z
- **Коммит**: `3de6a26 fix(core): eliminate T-junctions and non-manifold slits in CSG booleans [BUG-006]`
- **Суть**: Настроен адаптивный эпсилон площади граней; внедрено автоматическое разбиение смежных ребер в точках пересечения (T-junction resolution) для сеток до 4000 вершин; конформные резцы для отверстий и пазов.
- **Верификация**: `test_bug_006_csg_booleans_and_holes_watertight`. Цепочка отверстий (genus 2) водонепроницаема, 0 T-junctions, 0 дыр. 5 диагностических рендеров.

---

## Итерация 9 — Фаза 3: Исправление BUG-008, BUG-009, BUG-010, BUG-001
- **Дата/время**: 2026-10-30T06:30:00Z
- **Коммиты**:
  - `8a0e8bb fix(server): roll back DAG node on failed part rebuild [BUG-008]`
  - `13bc330 fix(server): make batch_cad_operations atomic on failure [BUG-009]`
  - `66e8a81 fix(server): validate part_id in headless open_preview_window and dimensions in capture_viewport [BUG-010]`
  - `b9273ad fix(docs): update tool count to 26 in ARCHITECTURE.md [BUG-001]`
- **Суть**:
  - Реализован откат узла DAG и меша при ошибке `rebuild()` во всех модифицирующих операциях.
  - Реализована полная атомарность `batch_cad_operations` с сохранением и откатом снимка состояния движка.
  - Добавлена валидация `part_id` в headless-режиме и проверка ненулевых габаритов вьюпорта.
  - Документация синхронизирована с фактическим реестром 26 инструментов.
- **Верификация**: Добавлены и успешно пройдены тесты `test_bug_008_dag_rollback_on_failed_rebuild`, `test_bug_009_batch_operations_atomic_rollback`, `test_bug_010_viewport_and_preview_validation`.

---

## Итерация 10 — Фаза 4: Финальная валидация и выпуск релиза
- **Дата/время**: 2026-10-30T07:15:00Z
- **Выполнено**:
  - Выполнены 2 последовательных чистых прогона всех тестов рабочей области: `cargo test --workspace` (120/120 тестов PASS).
  - Выполнен строгий статический анализ: `cargo clippy --workspace -- -D warnings` (0 предупреждений).
  - Собрана оптимизированная релизная версия: `cargo build --release -p mcp-cad-server` (успешно).
  - Обновлены и согласованы все отчеты QA: `STATE.json`, `BUGS.md`, `COVERAGE.md`, `MESH_REPORT.md`, `LOG.md`, `FINAL_REPORT.md`.
- **Итог**: Открытых багов S1/S2/S3/S4 нет (0). Все критерии остановки достигнуты. Релиз готов к эксплуатации.

---

## Итерация 11–14 — Фаза 3: Расширение до 28 инструментов (`validate_mesh`, `optimize_feature_tree`) и закрытие BUG-011..BUG-014
- **Дата/время**: 2026-11-01T10:10:00Z
- **Коммит**: `38d6683 chore(qa): baseline snapshot before iteration 15`
- **Суть**:
  - **BUG-011 [S2]**: Исправлен эйлеров род `spur_gear` при $Z=8$ со ступицей и шпоночным пазом ($g=1$, $\chi=0$) и полярная O-сетка болтовых отверстий PCD в `wheel_rim` ($g=11, 12, 16$).
  - **BUG-012 [S3]**: Устранены открытые граничные ребра (`boundary_edges = 0`) в `cut_helical_groove` для круглого и метрического ($60^\circ$) профилей резьбы.
  - **BUG-013 [S4]**: Унифицирована документация по всем 28 инструментам MCP (включая `validate_mesh` и `optimize_feature_tree`).
  - **BUG-014 [S3]**: Исправлены типы TypeScript в веб-студии `mcp-cad-studio` (`tsc --noEmit` = 0 ошибок).

---

## Итерация 15 — Фаза 2: Глубокий аудит контрактов API, 27 PNG-рендеров и граничных условий
- **Дата/время**: 2026-11-01T16:15:00Z
- **Выполнено**:
  - Просмотрены и описаны по 11-пунктному чек-листу (Раздел 8 ТЗ, Правила R1/R2) все 27 диагностических рендеров в `qa/artifacts/*.png` (`spur_gear_hub_keyway_*`, `wheel_rim_5spoke_*`, `revolve_partial_180_*`, `polygon_prism_hex_chamfer_*`, `csg_chain_two_holes_*`, `neg_scale_cyl_*`).
  - Обнаружены и задокументированы 3 новых дефекта:
    - **BUG-015 [S3]**: В `tools/list` (`crates/mcp-cad-server/src/tools/mod.rs`) для `create_joint` отсутствовал тип `"gear"` в `enum` и требовались обязательные `anchor_a`/`anchor_b`, а в `create_part` отсутствовало свойство `is_fixed`.
    - **BUG-016 [S2]**: В `revolve` (`crates/mcp-cad-server/src/tools/cad.rs` и `src/cad/primitives.ts`) отсутствовала проверка пересечения оси вращения ($r < 0$), угла $\notin (0, 360]$ и самопересечения 2D-профиля («восьмерка»); в TS-ядре `createWheelRimMesh` строил контур с отрицательным знаковым объемом, а `weldTolerance = 1e-5` м схлопывал соседние вершины мелких отверстий.
    - **BUG-017 [S2]**: В TS FEA-решателе (`src/cad/fea_solver.ts`) отсутствовала проверка незакрепленного тела (Rigid Body Motion при 0 `'fixed'`-ограничений) и декомпозиция силы на осевое растяжение/сжатие Гука ($\Delta L = \frac{FL}{EA}$) и поперечный изгиб.

---

## Итерация 16–17 — Фаза 3: Атомарное исправление BUG-015, BUG-016, BUG-017 и 50-цепочечный DAG-фаззинг
- **Дата/время**: 2026-11-01T16:45:00Z
- **Коммиты**:
  - `0d41f00 fix(server): align create_joint gear schema and create_part is_fixed schema [BUG-015]`
  - `208cf2a fix(core): enforce revolve profile boundary and self-intersection validation and outward MeshBuilder orientation [BUG-016]`
  - `02e932b fix(fea): reject unconstrained FEA models and decompose axial Hooke vs transverse bending [BUG-017]`
- **Верификация**:
  - Написаны и пройдены регрессионные тесты `test_bug015_create_joint_and_create_part_schema_contract`, `test_bug016_revolve_boundary_and_self_intersection_validation` в `mcp_contract_tests.rs` и тесты 18–21 в `src/cad/cad_engine.test.ts`.
  - Выполнен детерминированный фаззинг 50 случайных цепочек DAG (по 2–6 операций: примитивы + `cut_hole` + `transform_part` с отражением): **50/50 PASS** (0 паник, 100% watertight, 0 boundary edges, 0 non-manifold edges, 0 degenerate triangles, 100% outward normals).

---

## Итерация 18 — Фаза 4: Финальный контрольный прогон и аудит UI в браузере
- **Дата/время**: 2026-11-01T16:50:00Z
- **Выполнено**:
  - Пройдены 2 последовательных чистых прогона всех тестов (`npx --yes tsx --test src/cad/cad_engine.test.ts`: 21/21 PASS; Rust workspace: 126/126 PASS).
  - Сборка `compile_applet` и проверка `lint_applet`: 0 ошибок.
  - Актуализированы все отчеты и JSON-метрики в `qa/`. Открытых багов S1/S2/S3/S4: **0**.


