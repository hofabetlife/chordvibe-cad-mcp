# Пакет задач реализации выбранного геометрического ядра (`P1-T4` → `P1-K1..P1-K6`)

- **Основание:** [`docs/adr/0001-geometry-kernel.md`](../docs/adr/0001-geometry-kernel.md) (раздел 7 «Оценка работ по миграции в задачах» и раздел 8 «Риски и способы их минимизации»)
- **Подготовлено в рамках задачи:** `P1-T4` (`fix/P1_architecture.md`)
- **Прототипы-эталоны в репозитории:** [`spikes/manifold_rust/src/main.rs`](../spikes/manifold_rust/src/main.rs) (`manifold-rust v0.19.2`, Pure Rust) и [`spikes/monstertruck_brep/src/main.rs`](../spikes/monstertruck_brep/src/main.rs) (`monstertruck v0.4.1`, Pure Rust B-Rep & STEP)

---

## 1. Как пользоваться этим пакетом

1. **Один промт = Преамбула (раздел 3 ниже) + одна задача целиком (`P1-K1`…`P1-K6`)** из раздела 4. Каждая задача рассчитана на запуск ИИ-агента в **пустом контексте** без чтения прошлых чатов.
2. Один агент выполняет строго одну задачу в ветке `agent/<ID>` (например, `agent/P1-K1`). По завершении остаётся отчёт `docs/agent_tasks/<ID>-report.md` и обновляется статус в `TASKS.md`.
3. Метка **`[DECISION]`** обозначает развилку, требующую подтверждения архитектора. Если решение не переопределено явно, агент использует задокументированный вариант по умолчанию и фиксирует это в отчёте.
4. Все пути к файлам, номера строк, сигнатуры функций и численные результаты замеров ниже **проверены по фактическому состоянию репозитория** на коммите `d5c743a` (после выполнения `P0-T1..P0-T12` и `P1-T1..P1-T3`).

---

## 2. Порядок выполнения, зависимости и связь с задачами `P2`/`P3`

### 2.1. Сводная таблица задач миграции ядра

| ID | Задача | Зависит от | Размер | Ветка | Затрагиваемые файлы | Закрывает / Заменяет в `TASKS.md` |
|---|---|---|---|---|---|---|
| **`P1-K1`** | Подключение `manifold-rust v0.19.2` и безпотерьный конвертер `TriangleMesh <-> MeshGLP<f64, u32>` со сваркой вершин и сохранением `face_id` | `P1-T3` | **S** | `agent/P1-K1` | `crates/mcp-cad-core/Cargo.toml`, `crates/mcp-cad-core/src/topology.rs`, `crates/mcp-cad-core/src/csg.rs` | Фундамент для `P1-K2..K5` и `P2-T5..T8` |
| **`P1-K2`** | Перевод `csg_union`, `csg_difference`, `csg_intersection` на `Manifold` + постобработка `optimize_csg_mesh_quality` для Sliver Gate | `P1-K1` | **M** | `agent/P1-K2` | `crates/mcp-cad-core/src/csg.rs`, `crates/mcp-cad-core/src/diagnostics.rs`, `crates/mcp-cad-core/tests/csg_invariants.rs` | Закрывает **`P3-T12`** (открытые рёбра на фланцевой втулке + формула `genus` для $k$ компонент) |
| **`P1-K3`** | Водонепроницаемость резьбы `cut_helical_groove` через `manifold-rust` (`T08_hex_nut_m10` и спиральные канавки) | `P1-K2` | **S** | `agent/P1-K3` | `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/src/dag.rs`, `crates/mcp-cad-core/tests/dag_tests.rs`, `crates/mcp-cad-server/tests/bench_suite.rs` | Полностью закрывает **`P3-T11`** (`T08_hex_nut_m10`) |
| **`P1-K4`** | Нативное прохождение именованных граней (`face_ids` + `FaceInfo`) через примитивы, `transform` и `MeshGLP` в CSG | `P1-K2` | **M** | `agent/P1-K4` | `crates/mcp-cad-core/src/topology.rs`, `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/src/csg.rs`, `crates/mcp-cad-core/src/dag.rs`, `crates/mcp-cad-server/src/tools/cad.rs`, `docs/FACE_NAMING_DESIGN.md` | Консолидирует и ускоряет **`P2-T5` [S] + `P2-T6` [M] + `P2-T7` [L]** в одну задачу размера **M** (вход для `P2-T8`) |
| **`P1-K5`** | Инструмент `extrude_sketch` (2D-контуры с вырезами и скруглением углов `corner_radius`) на базе `CrossSection` + `Manifold::extrude` | `P1-K4` | **M** | `agent/P1-K5` | `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/src/dag.rs`, `crates/mcp-cad-server/src/tools/cad.rs`, `crates/mcp-cad-server/src/tools/mod.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs` | Закрывает исторический долг `TASK-007` (`ExtrudeSketch`) и 2D-профильный `TASK-008` (`Fillet`) |
| **`P1-K6`** *(опц.)* | Экспорт параметрического дерева `FeatureTree` в стандарт ISO 10303-21 `STEP` (`export_step`) через `monstertruck-io` | `P1-T3` | **M** | `agent/P1-K6` | `crates/mcp-cad-core/Cargo.toml`, `crates/mcp-cad-core/src/export_step.rs`, `crates/mcp-cad-core/src/lib.rs`, `crates/mcp-cad-server/src/tools/cad.rs`, `crates/mcp-cad-server/src/tools/mod.rs` | Опциональное расширение Гибридного Варианта A+C (`[DECISION-4]` в ADR-0001) |

### 2.2. Влияние на задачи Фазы 2 (`P2-T5..P2-T8`) и Фазы 3 (`P3-T11`, `P3-T12`)

- **Замена ручной проброски `face_id` в BSP (`P2-T5..T7`):** в исходном плане `fix/P2_agent_contract.md` задача `P2-T7` имела размер `L`, так как требовала ручного протаскивания `face_id` сквозь рекурсивное рассечение полигонов `Plane::split_polygon` и эвристики `repair_manifold_triangles` в старом BSP-CSG. Поскольку `manifold-rust v0.19.2` нативно хранит `MeshGLP::face_id: Vec<u32>` и `run_original_id: Vec<u32>` и автоматически наследует их при триангуляции граней пересечения (проверено в `spikes/manifold_rust/src/main.rs:274-315` — ошибка площади верхнего торца `0.0000%`), задачи `P2-T5`, `P2-T6` и `P2-T7` выполняются на базе `MeshGLP` в рамках **`P1-K4`** (либо поочерёдно по тому же дизайну с уменьшением сложности `P2-T7` с `L` до `S`).
- **Готовность для `P2-T8` (Граничные условия FEA по именам граней):** после завершения `P1-K4` структура `TriangleMesh` содержит `pub face_ids: Vec<u32>` и `pub faces: Vec<FaceInfo>`, что напрямую используется в `P2-T8` (`crates/mcp-cad-fea/src/boundary.rs` и `crates/mcp-cad-server/src/tools/fea.rs`).
- **Закрытие `P3-T11` и `P3-T12`:** переход на `manifold-rust` в `P1-K2` и `P1-K3` автоматически устраняет нарушение водонепроницаемости на составной фланцевой втулке (`24` открытых ребра $\to$ `0` открытых рёбер, ускорение с `940.9 мс` до `25.0 мс`) и на внутренней резьбе гайки М10 `T08_hex_nut_m10` (`615` открытых рёбер $\to$ `0` открытых рёбер, ускорение с `257.5 мс` до `18.9 мс`).

---

## 3. ПРЕАМБУЛА (копировать в начало каждого промта `P1-K1..P1-K6`)

```text
Ты — инженер-программист на Rust. Работаешь в репозитории
https://github.com/hofabetlife/chordvibe-cad-mcp (актуальная база после P0-T1..P0-T12 и P1-T1..P1-T3).
Контекста прошлых разговоров у тебя нет: всё нужное — в этом промте и в репозитории.

ПРОЕКТ. MCP-сервер для CAD/CAE: ИИ-агент строит детали через JSON-RPC-инструменты
(28 штук), проверяет геометрию, считает прочность (МКЭ) и механизмы (физика).
Цель проекта — чтобы ИИ легко и надёжно моделировал и сразу проверял работоспособность
и прочность. Rust-workspace из 5 крейтов (crates/):
- mcp-cad-core (~12.6k строк): TriangleMesh (topology.rs, 621 строка), примитивы (primitives.rs,
  4143 строки), CSG и постобработка качества сетки (csg.rs, 2784 строки), DAG параметрической
  истории (dag.rs, 839 строк), диагностика сетки validate_mesh (diagnostics.rs, 1099 строк),
  оптимизатор DAG (query_optimizer.rs), тесселяция (tessellation.rs), материалы (material.rs), STL.
- mcp-cad-fea: МКЭ. Элемент TET4 (element.rs), PCG-солвер (solver.rs), воксельная
  тетра-сетка (tetra_mesh.rs), граничные условия (boundary.rs), отчёт FeaReport
  (result.rs), исследование сходимости (convergence.rs).
- mcp-cad-physics: Rapier3D, суставы (joint.rs), телеметрия и реакции (telemetry.rs).
- mcp-cad-viewer: wgpu-рендер (offscreen и окно).
- mcp-cad-server: MCP/JSON-RPC (protocol.rs), инструменты (tools/*.rs; диспетчер
  dispatch_tool в tools/mod.rs), состояние EngineState (state.rs).
В каталоге spikes/ лежат два проверенных изолированных прототипа из P1-T3:
- spikes/manifold_rust/ (manifold-rust v0.19.2, Pure Rust, Apache-2.0)
- spikes/monstertruck_brep/ (monstertruck v0.4.1, Pure Rust B-Rep & STEP, Apache-2.0)
Архитектурное решение записано в docs/adr/0001-geometry-kernel.md.

ОКРУЖЕНИЕ И ТЕКУЩЕЕ СОСТОЯНИЕ БАЗЫ (проверено):
- Rust toolchain >= 1.91 (в контейнере установлен rustc/cargo 1.99.1).
- cargo test --workspace --no-fail-fast: 156 тестов, 156 проходят, 0 падают, 0 ignored (100% PASS):
  * mcp-cad-core --lib: 50/50
  * mcp-cad-core --test dag_tests: 14/14
  * mcp-cad-core --test tessellation_tests: 6/6
  * mcp-cad-core --test primitive_invariants: 18/18
  * mcp-cad-core --test csg_invariants: 5/5
  * mcp-cad-fea --test fea_tests: 11/11
  * mcp-cad-physics --test physics_tests: 6/6
  * mcp-cad-viewer --test viewer_tests: 10/10
  * mcp-cad-server --test gear_kinematic_test: 1/1
  * mcp-cad-server --test mcp_contract_tests: 21/21
  * mcp-cad-server --test qa_wide_scan: 13/13
  * mcp-cad-server --test bench_suite: 1/1
- cargo clippy --workspace --all-targets -- -D warnings: 0 ошибок, 0 предупреждений.
- Тесты запускай прицельно: cargo test -p <крейт> --test <файл> <фильтр>, а перед финишем
  прогоняй полный суит затронутых крейтов и cargo clippy --workspace --all-targets -- -D warnings.

ЕДИНИЦЫ. Ядро работает в СИ: метры, Паскали, Ньютоны, килограммы. Material хранит
GPa / MPa / кг/м³. Часть тестов использует безразмерные значения (r=1.5, h=5) или мм (r=15, h=50).
Поэтому все геометрические допуски сварки и вырожденности должны масштабироваться от диагонали
габаритного параллелепипеда diag: weld_eps = (diag * 1e-7).max(1e-12).

ПРАВИЛА.
1. Сначала воспроизведи описанный сценарий или проверь базовые тесты на своей копии.
2. Не ослабляй проверки: не меняй пороги Sliver Quality Gate в diagnostics.rs
   (lt_1_deg_count == 0, lt_5_deg_ratio <= 1.0%, MAX_ALLOWED_SLIVER_RATIO = 0.01),
   не ослабляй допуски существующих тестов, не добавляй #[ignore], не удаляй тесты.
3. Минимальный дифф, без постороннего рефакторинга. Разрешены только зависимости,
   прямо указанные в задаче.
4. Работай в ветке agent/<ID>. Коммиты атомарные: feat(core): …, fix(core): …, test(…): …, docs(…): …
5. Береги контекст: используй grep -n и просмотр нужных диапазонов строк.
6. Перед завершением: cargo build --workspace, cargo test затронутых крейтов и
   cargo clippy --workspace --all-targets -- -D warnings (должно быть 0 предупреждений).
7. Для пунктов [DECISION] не выбирай молча: используй документированный вариант по умолчанию,
   если человек не указал иное, и явно зафиксируй это в отчёте.

ФОРМАТ ОТЧЁТА (в чат и в docs/agent_tasks/<ID>-report.md):
1) что сделано (файлы, структуры, функции); 2) как проверено (точные команды и строки
`test result`); 3) тесты до/после; 4) остаточные риски и открытые вопросы;
5) побочные находки (занести в TASKS.md, не чинить вне рамок задачи).
```

---

## 4. Задачи реализации геометрического ядра (`P1-K1`…`P1-K6`)

---

### P1-K1 — Подключение `manifold-rust v0.19.2` и конвертер `TriangleMesh <-> MeshGLP<f64, u32>`

**Размер:** S | **Зависит от:** `P1-T3` | **Ветка:** `agent/P1-K1`

#### Контекст (проверено)
- В [`docs/adr/0001-geometry-kernel.md`](../docs/adr/0001-geometry-kernel.md) в качестве основного ядра булевых операций и трассировки граней рекомендован крейт **`manifold-rust = "0.19.2"`** (100% Pure Rust порт библиотеки Manifold v3.5.4, лицензия `Apache-2.0`, `build = false`, без C++/FFI).
- Крейт уже скачан в cargo-кэше и проверен в изолированном прототипе [`spikes/manifold_rust/src/main.rs`](../spikes/manifold_rust/src/main.rs) (строки 74–173: функции `triangle_mesh_to_manifold_with_faces` и `manifold_to_triangle_mesh`).
- В [`crates/mcp-cad-core/src/topology.rs`](../crates/mcp-cad-core/src/topology.rs) (строки 149–155) структура `TriangleMesh` сейчас хранит только `pub vertices: Vec<Vertex>` и `pub indices: Vec<[u32; 3]>`. На острых рёбрах примитивов (например, между торцевой крышкой и боковой стенкой цилиндра или на рёбрах бокса в `create_box`, где `24` вершины на `12` треугольников) вершины с одинаковыми `position` продублированы ради раздельных нормалей `Vertex::normal`.
- Библиотека `manifold-rust` (`manifold_rust::manifold::Manifold::from_mesh_glp`) требует **топологически сваренную 2-манифолдную сетку** (`MeshGLP<f64, u32>`), где совпадающие в пространстве вершины имеют единый индекс, иначе `Manifold::status()` вернёт `Error::NotManifold`.
- В [`crates/mcp-cad-core/src/diagnostics.rs`](../crates/mcp-cad-core/src/diagnostics.rs) (строки 219–266) уже реализован эталонный алгоритм пространственной сварки вершин с допуском `weld_eps = (diag * 1e-7).max(1e-12)` и проверкой соседних ячеек $3\times 3\times 3$.

#### Задача
1. Добавь зависимость `manifold-rust = "0.19.2"` в [`crates/mcp-cad-core/Cargo.toml`](../crates/mcp-cad-core/Cargo.toml) (секция `[dependencies]`).
2. В [`crates/mcp-cad-core/src/topology.rs`](../crates/mcp-cad-core/src/topology.rs) добавь в `TriangleMesh` опциональный (с обратной совместимостью `serde`) параллельный массив идентификаторов граней:
   ```rust
   #[serde(default, skip_serializing_if = "Vec::is_empty")]
   pub face_ids: Vec<u32>,
   ```
   Обнови конструкторы `TriangleMesh::new`, `TriangleMesh::empty` и методы модификации (`scale` при инверсии обхода сохраняет порядок `face_ids`), чтобы `face_ids` по умолчанию инициализировался пустым вектором `Vec::new()` (или вектором длины `indices.len()`, если задан) и не ломал ни один из существующих вызовов `TriangleMesh { vertices, indices }` (при необходимости предоставь вспомогательный конструктор `TriangleMesh::with_face_ids(vertices, indices, face_ids)` или обнови литералы внутри `mcp-cad-core`).
3. В [`crates/mcp-cad-core/src/csg.rs`](../crates/mcp-cad-core/src/csg.rs) (или в новом подмодуле `crates/mcp-cad-core/src/manifold_bridge.rs`, реэкспортированном в `csg.rs`) реализуй пару функций конвертации:
   - `pub fn mesh_to_manifold(mesh: &TriangleMesh) -> Result<(manifold_rust::manifold::Manifold, u32), CadError>`:
     - Сваривает вершины `mesh.vertices` с допуском `weld_eps = (diag * 1e-7).max(1e-12)` по окрестности $3\times 3\times 3$ (как в `diagnostics.rs:224-266` и `spikes/manifold_rust/src/main.rs:81-143`).
     - Отбрасывает вырожденные по индексам треугольники (`w0 == w1 || w1 == w2 || w2 == w0`).
     - Передаёт `face_id` из `mesh.face_ids` (если `mesh.face_ids.len() == mesh.indices.len()`, иначе `0`) в `MeshGLP::<f64, u32>::face_id`, резервируя `orig_id = Manifold::reserve_ids(1)` в `run_original_id`.
     - Проверяет `manifold.status() == manifold_rust::types::Error::NoError`; при ошибке возвращает `Err(CadError::BooleanFailed(...))` или `Err(CadError::InvalidTopology(...))`.
   - `pub fn manifold_to_mesh(manifold: &manifold_rust::manifold::Manifold) -> TriangleMesh`:
     - Извлекает `MeshGLP<f64, u32>` через `manifold.get_mesh_glp(-1)`.
     - Восстанавливает `vertices`, `indices` и `face_ids: mgl.face_id`.
     - Вычисляет корректные нормали вершин: чтобы не сглаживать острые углы (например, $90^\circ$ между крышкой и стенкой цилиндра или гранями бокса, что вызвало бы `normals_vs_winding_mismatches > 0` в `validate_mesh`, см. `diagnostics.rs:371-377`), разделяй вершины (split normals) между треугольниками, угол между нормалями которых превышает порог (например, $\cos \theta < 0.5$, т.е. $> 60^\circ$, либо разный `face_id` при `face_id != 0`), и усредняй нормали внутри гладких групп.
4. Напиши модульные тесты в `crates/mcp-cad-core`:
   - `test_manifold_bridge_roundtrip_all_primitives`: проверяет `mesh -> mesh_to_manifold -> manifold_to_mesh` на всех 8 генераторах примитивов (`create_box`, `create_cylinder`, `create_cone`, `create_sphere`, `create_polygon_prism` с фаской, `create_spur_gear`, `create_revolve`, `create_wheel_rim`):
     - `manifold.status() == Error::NoError`;
     - `validate_mesh(&roundtrip_mesh, None).status == "PASS"` (`normals_vs_winding_mismatches == 0`, `is_watertight == true`);
     - относительное отклонение `signed_volume()` и `surface_area()` от исходного меша $< 10^{-9}$;
     - `bounding_box()` совпадает с точностью $< 10^{-9}$.
   - `test_manifold_bridge_preserves_face_ids`: размечает 6 граней бокса `face_ids = [1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6]`, выполняет round-trip и проверяет сохранение всех 6 идентификаторов и площадей граней.

#### Критерии приёмки
- `cargo test -p mcp-cad-core --lib manifold_bridge` проходит (`PASS`).
- Все существующие тесты `cargo test --workspace --no-fail-fast` (156+ тестов) проходят без регрессий.
- `cargo clippy --workspace --all-targets -- -D warnings` выдаёт 0 ошибок и 0 предупреждений.
- Сериализация/десериализация существующих JSON-примеров (`examples/*.json`) и `Part` в `dag_tests.rs` работает без изменений формата.

#### Не делать
- В рамках `P1-K1` ещё не переключать `csg_union`, `csg_difference`, `csg_intersection` на `Manifold` (это предмет задачи `P1-K2`).
- Не менять пороги `validate_mesh` в `diagnostics.rs`.

---

### P1-K2 — Перевод `csg_union`, `csg_difference`, `csg_intersection` на `Manifold` + Sliver Gate постобработка и исправление `genus` (`P3-T12`)

**Размер:** M | **Зависит от:** `P1-K1` | **Ветка:** `agent/P1-K2`

#### Контекст (проверено)
- В [`crates/mcp-cad-core/src/csg.rs`](../crates/mcp-cad-core/src/csg.rs) функции `csg_union` (строка 2704), `csg_difference` (строка 2730) и `csg_intersection` (строка 2759) сейчас используют самописное BSP-дерево (`BspNode::clip_polygons` + `polygons_to_mesh`).
- Как доказано в `P1-T3` ([`spikes/manifold_rust/src/main.rs`](../spikes/manifold_rust/src/main.rs), строки 356–376), на составной фланцевой втулке (`create_cylinder(15.0, 50.0, 64)` $\cup$ `create_box(40.0, 40.0, 10.0)` $\setminus$ `create_cylinder(10.0, 52.0, 64)`) текущий BSP-CSG работает `940.89 мс` и теряет водонепроницаемость (`open_boundary_edges = 24`, `is_watertight = false`, $\chi = -8$, `validate_mesh = FAIL`), тогда как `manifold-rust` выполняет ту же операцию за `25.01 мс` с `open_boundary_edges = 0`, `is_watertight = true`, $\chi = 0, g = 1.0$.
- Одновременно в `P1-T3` зафиксирован Риск №1 ([`docs/adr/0001-geometry-kernel.md`](../docs/adr/0001-geometry-kernel.md), раздел 8): вдоль линий пересечения криволинейных сеток сырой выход `Manifold` может содержать несколько узких треугольников с минимальным углом $< 1^\circ$ (например, на фланцевой втулке без постобработки `lt_1_deg_count = 16`, `min_angle = 0.303°`).
- В [`crates/mcp-cad-core/src/csg.rs`](../crates/mcp-cad-core/src/csg.rs) уже реализован топологически безопасный конвейер улучшения качества сетки `optimize_csg_mesh_quality` (строка 808), включающий:
  - `smooth_coplanar_interior_vertices` (строка 845),
  - `optimize_coplanar_delaunay_edges` (строка 1017),
  - `try_remove_vertex_ear_clip` (строка 1249),
  - `collapse_redundant_and_sliver_vertices` (строка 1521),
  - `refine_coplanar_sliver_fans_and_edges` (строка 1844).
- Кроме того, в [`crates/mcp-cad-core/src/diagnostics.rs`](../crates/mcp-cad-core/src/diagnostics.rs) на строке 414 обнаружена ошибка формулы топологического рода (`P3-T12`):
  ```rust
  let genus = (2.0 - euler_characteristic as f64) / 2.0;
  ```
  Она вычисляется до подсчёта `connected_components` (строки 428–450) и предполагает $k = 1$ связную компоненту, хотя для $k$ замкнутых ориентируемых компонент обобщённая характеристика Эйлера равна $\chi = 2k - 2g \implies g = k - \chi / 2$. Из-за этого при `csg_union` двух непересекающихся тел ($k = 2, \chi = 4$) `validate_mesh` ошибочно выдаёт `genus = -1.0` вместо `0.0`.

#### Задача
1. В [`crates/mcp-cad-core/src/diagnostics.rs`](../crates/mcp-cad-core/src/diagnostics.rs) перенеси вычисление `let genus = ...` ниже цикла подсчёта `connected_components` (после строки 450) и исправь формулу:
   ```rust
   let k = connected_components.max(1) as f64;
   let genus = (2.0 * k - euler_characteristic as f64) / 2.0;
   ```
   Добавь unit-тест в `diagnostics.rs` на объединение двух непересекающихся кубов/сфер ($k = 2, \chi = 4 \implies \text{genus} = 0.0$).
2. В [`crates/mcp-cad-core/src/csg.rs`](../crates/mcp-cad-core/src/csg.rs) переведи `csg_union`, `csg_difference`, `csg_intersection` на выполнение булевой операции через `manifold-rust` (`Manifold::union`, `Manifold::difference`, `Manifold::intersection` поверх конвертера из `P1-K1`) с последующим прогоном результата через `optimize_csg_mesh_quality` для устранения игольчатых треугольников вдоль кривых пересечения:
   - Если оба входных меша валидны в `mesh_to_manifold`, выполняется `Manifold` + `optimize_csg_mesh_quality` (с сохранением `face_ids`).
   - Если результат `Manifold` пуст (например, `csg_intersection` непересекающихся тел или `A \ A`), возвращается `TriangleMesh::empty()`.
   - Проследи, чтобы `optimize_csg_mesh_quality` гарантированно доводил сетку до прохождения Sliver Gate (`lt_1_deg_count == 0`, `lt_5_deg_ratio <= 1.0%`), не нарушая `is_watertight == true` и `t_junctions_count == 0`.
3. Добавь в [`crates/mcp-cad-core/tests/csg_invariants.rs`](../crates/mcp-cad-core/tests/csg_invariants.rs) два новых теста:
   - `test_csg_flanged_bushing_watertight_and_volume` (эталонная деталь из `P1-T3`: цилиндр $R=15, h=50$ $\cup$ бокс $40\times 40\times 10$ на $z=-15$ $\setminus$ отверстие $R=10, h=52$ при `segments = 64`): проверяет `validate_mesh(&res, Some(1)).status == "PASS"`, `is_watertight == true`, `boundary_edges_count == 0`, `genus == 1.0`, отклонение объёма от $28\,546.194\text{ мм}^3$ менее $0.2\%$.
   - `test_csg_disjoint_union_two_components_genus_zero`: проверяет объединение двух разнесённых тел (`connected_components == 2`, `euler_characteristic == 4`, `genus == 0.0`, `validate_mesh(&res, Some(0)).status == "PASS"`).
4. Удали или пометь `#[cfg(test)]` неиспользуемый мёртвый код старого BSP-дерева в `csg.rs`, чтобы `cargo clippy --workspace --all-targets -- -D warnings` проходил с 0 предупреждений `dead_code`.

#### Критерии приёмки
- `cargo test -p mcp-cad-core --test csg_invariants` — все тесты (включая `test_csg_flanged_bushing_watertight_and_volume` и `test_csg_disjoint_union_two_components_genus_zero`) проходят (`PASS`).
- `cargo test -p mcp-cad-core --test primitive_invariants` — все 18 тестов проходят (`PASS`).
- `cargo test -p mcp-cad-core --test dag_tests` — все 14 тестов проходят (`PASS`).
- `cargo test -p mcp-cad-server --test qa_wide_scan` — все 13 тестов (включая `test_bug_006_csg_booleans_and_holes_watertight`) проходят (`PASS`).
- `cargo clippy --workspace --all-targets -- -D warnings` — 0 ошибок, 0 предупреждений.
- Задача **`P3-T12`** в `TASKS.md` переведена в статус `✅ DONE`.

#### Не делать
- Не ослаблять пороги Sliver Quality Gate (`lt_1_deg_count == 0`, `lt_5_deg_ratio <= 1.0%`) в `diagnostics.rs`.
- Не менять сигнатуры публичных функций `csg_union`, `csg_difference`, `csg_intersection`.

---

### P1-K3 — Водонепроницаемость резьбы `cut_helical_groove` через `manifold-rust` (`T08_hex_nut_m10` / `P3-T11`)

**Размер:** S | **Зависит от:** `P1-K2` | **Ветка:** `agent/P1-K3`

#### Контекст (проверено)
- В рамках `P1-T2` ([`docs/BENCH_BASELINE_RUST.md`](../docs/BENCH_BASELINE_RUST.md)) и `P3-T11` (`TASKS.md`) установлено, что в эталонной задаче `T08_hex_nut_m10` (шестигранная гайка М10×1.5: `create_polygon_prism(6, 0.008, Some(0.017), None, Some(0.0008))` + сквозное отверстие $\varnothing 8.5\text{ мм}$ + внутренняя метрическая резьба `CutHelicalGroove`) старый BSP-CSG выдавал `open_boundary_edges = 615`, `is_watertight = false`, `genus = 98.0`.
- В спайке [`spikes/manifold_rust/src/main.rs`](../spikes/manifold_rust/src/main.rs) (строки 379–450) проверено, что при вычитании винтового резца `create_helical_cutter_with_settings` через `manifold-rust v0.19.2` булева операция завершается за `18.87 мс` со статусом `Error::NoError`, давая **`is_watertight = true`, `open_boundary_edges = 0`, `non_manifold_edges = 0`, `genus = 1.0`**.
- Генератор винтового резца находится в [`crates/mcp-cad-core/src/primitives.rs`](../crates/mcp-cad-core/src/primitives.rs) (`create_helical_cutter` на строке 1472 и `create_helical_cutter_with_settings` на строке 1499), а вызов в DAG — в [`crates/mcp-cad-core/src/dag.rs`](../crates/mcp-cad-core/src/dag.rs) (ветка `FeatureOp::CutHelicalGroove`, строки 644–677).

#### Задача
1. Проверь на ветке после `P1-K2` выполнение `FeatureOp::CutHelicalGroove` на двух ключевых моделях:
   - Внутренняя резьба гайки М10×1.5 (`T08_hex_nut_m10` из `bench/tasks.json` и `spikes/manifold_rust/src/main.rs:385-417`).
   - Спиральные стружечные канавки сверла (`test_dag_helical_groove_drill_bit_model` в [`crates/mcp-cad-core/tests/dag_tests.rs`](../crates/mcp-cad-core/tests/dag_tests.rs)).
2. Проверь качество треугольников (`validate_mesh`) после вычитания `create_helical_cutter_with_settings`:
   - На спайке `P1-T3` сырой выход `manifold-rust` на `T08_hex_nut_m10` имел `is_watertight = true`, `open_edges = 0`, `genus = 1.0`, но статус `WARN` из-за доли узких треугольников на выходе винтового витка за пределы торцевой фаски гайки.
   - При необходимости скорректируй шаг угловой/осевой дискретизации `create_helical_cutter_with_settings` (`primitives.rs:1499`) и/или постобработку `optimize_csg_mesh_quality` так, чтобы как сам винтовой резец `create_helical_cutter_with_settings`, так и результирующая гайка М10 после `csg_difference` проходили `validate_mesh` со статусом **`PASS`** (`is_watertight == true`, `boundary_edges_count == 0`, `non_manifold_edges_count == 0`, `lt_1_deg_count == 0`, `lt_5_deg_ratio <= 1.0%`, `genus == 1.0`).
3. Добавь в [`crates/mcp-cad-core/tests/dag_tests.rs`](../crates/mcp-cad-core/tests/dag_tests.rs) интеграционный тест `test_dag_hex_nut_m10_helical_thread_watertight_and_valid`:
   - Строит `FeatureTree` с `AddPolygonPrism` ($s=17\text{ мм}, h=8\text{ мм}, c=0.8\text{ мм}$), `CutHole` ($r=4.25\text{ мм}$) и `CutHelicalGroove` ($r_{\text{helix}}=5.0\text{ мм}, r_{\text{groove}}=0.8\text{ мм}, \text{pitch}=1.5\text{ мм}$, профиль `"metric_60"`).
   - Проверяет `validate_mesh(&mesh, Some(1)).status == "PASS"`, `topology.is_watertight == true`, `topology.boundary_edges_count == 0`, `topology.genus == 1.0`, положительный объём в аналитических границах.

#### Критерии приёмки
- `cargo test -p mcp-cad-core --test dag_tests test_dag_hex_nut_m10_helical_thread_watertight_and_valid` проходит (`PASS`).
- `cargo test -p mcp-cad-core --test dag_tests test_dag_helical_groove_drill_bit_model` проходит (`PASS`).
- `cargo test -p mcp-cad-server --test bench_suite` подтверждает, что в `T08_hex_nut_m10` сетка после `cut_helical_groove` является водонепроницаемой (`is_watertight == true`, `open_edges == 0`).
- `cargo clippy --workspace --all-targets -- -D warnings` — 0 ошибок, 0 предупреждений.
- Задача **`P3-T11`** в `TASKS.md` переведена в статус `✅ DONE`.

#### Не делать
- Не заменять реальную винтовую геометрию `CutHelicalGroove` фиктивным цилиндром (резьбовой канал должен реально формироваться в сетке и уменьшать объём гайки).
- Не ослаблять пороги `validate_mesh`.

---

### P1-K4 — Нативное прохождение именованных граней (`face_id` и `FaceInfo`) через примитивы, `transform` и `MeshGLP` (реализация `P2-T5..T7`)

**Размер:** M | **Зависит от:** `P1-K2` | **Ветка:** `agent/P1-K4`

#### Контекст (проверено)
- В `P1-T3` ([`docs/adr/0001-geometry-kernel.md`](../docs/adr/0001-geometry-kernel.md), разделы 3.3, 6, 7 и `[DECISION-3]`) экспериментально подтверждено, что `manifold-rust v0.19.2` сохраняет `MeshGLP::face_id` каждого треугольника сквозь любые цепочки булевых операций `union`, `difference`, `intersection` с точностью площади `0.0000%` (`spikes/manifold_rust/src/main.rs:273-316`).
- Это позволяет выполнить дизайн и реализацию именованных граней (`P2-T5`, `P2-T6`, `P2-T7` из `fix/P2_agent_contract.md`) единым чистым слоем поверх `TriangleMesh::face_ids` и `MeshGLP::face_id` без ручного разбора BSP-полигонов.
- Сейчас инструмент `query_part_geometry` ([`crates/mcp-cad-server/src/tools/cad.rs`](../crates/mcp-cad-server/src/tools/cad.rs), строки 1359–1388) возвращает только геометрически кластеризованные плоские грани `detected_planar_faces` из `TriangleMesh::inspect_geometry` (`topology.rs:377-515`), которые не имеют имён, не различают цилиндрические/конические/сферические поверхности и сливают несвязные копланарные участки в одну грань.

#### Задача
1. **Документ дизайна (`docs/FACE_NAMING_DESIGN.md`, закрывает `P2-T5`):**
   Кратко и чётко зафиксируй архитектуру именованных граней на базе `TriangleMesh` + `manifold-rust`:
   - Типы в `crates/mcp-cad-core/src/topology.rs`:
     ```rust
     #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
     #[serde(rename_all = "snake_case")]
     pub enum FaceKind {
         Plane,
         Cylinder,
         Cone,
         Sphere,
         Torus,
         Helical,
         Other,
     }

     #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
     pub struct FaceInfo {
         pub id: u32,
         pub name: String,
         pub kind: FaceKind,
         pub origin_feature: Option<String>,
         pub area: f64,
         pub centroid: [f64; 3],
         pub normal_or_axis: [f64; 3],
         pub radius: Option<f64>,
         pub triangles_count: usize,
     }
     ```
   - Поля в `TriangleMesh` (с `#[serde(default, skip_serializing_if = "Vec::is_empty")]`):
     - `pub face_ids: Vec<u32>` (длина равна `indices.len()`, `0` — неразмеченная грань);
     - `pub faces: Vec<FaceInfo>` (реестр семантических граней тела, пересчитываемый по актуальным треугольникам сетки).
   - Каноническая схема именования граней примитивов:
     - `create_box`: `top` ($+Z$), `bottom` ($-Z$), `front` ($+Y$), `back` ($-Y$), `right` ($+X$), `left` ($-X$);
     - `create_cylinder`: `cap_top` ($+Z$), `cap_bottom` ($-Z$), `side` (цилиндрическая боковая поверхность, `radius = Some(r)`);
     - `create_cone`: `cap_top` (если $r_{\text{top}} > 0$), `cap_bottom` (если $r_{\text{bottom}} > 0$), `side` (`FaceKind::Cone`);
     - `create_sphere`: `surface` (`FaceKind::Sphere`, `radius = Some(r)`);
     - `create_polygon_prism`: `cap_top`, `cap_bottom`, `side_0`…`side_{n-1}`, и при `chamfer > 0` — `chamfer_top_0`…`chamfer_top_{n-1}`, `chamfer_bottom_0`…`chamfer_bottom_{n-1}`.
     - `cut_hole`: `hole_wall` (`FaceKind::Cylinder`, `radius = Some(r)`), `hole_bottom` (для глухого отверстия), `keyway_wall` (если задан шпоночный паз).
     - При оценке `FeatureTree` в `dag.rs`: к локальному имени роли добавляется префикс узла или каноническое имя (при наличии одного базового примитива сохраняются короткие алиасы `cap_top`, `cap_bottom`, `side`, а также полные имена `<feature_name_or_id>:<role>` и `hole_wall:<feature_id>` для вырезов).
2. **Разметка базовых примитивов (`crates/mcp-cad-core/src/primitives.rs`, закрывает `P2-T6`):**
   - Заполняй `face_ids` и начальный реестр `faces` в `create_box`, `create_cylinder`, `create_cone`, `create_sphere`, `create_polygon_prism` и `cut_hole_with_settings`.
   - Для `create_spur_gear`, `create_wheel_rim`, `create_revolve` назначь базовые роли (`cap_top`, `cap_bottom`, `bore_wall`, `other`) или `face_id = 0` (`FaceKind::Other`), чтобы `face_ids.len() == indices.len()`.
   - Добавь метод `TriangleMesh::recompute_face_registry(&mut self)`: пересчитывает `area`, `centroid`, `triangles_count` каждой грани из `self.faces` по фактическим треугольникам с данным `face_id`, удаляет грани с `triangles_count == 0` (полностью срезанные булевой операцией) и корректно обновляет `normal_or_axis` после `translate`/`rotate_rpy_deg`/`scale`/`transform`.
3. **Прохождение `face_id` сквозь CSG (`crates/mcp-cad-core/src/csg.rs`, закрывает `P2-T7`):**
   - В `csg_union`, `csg_difference`, `csg_intersection` перед передачей операндов $A$ и $B$ в `mesh_to_manifold` перенумеруй `face_ids` операнда $B$ со смещением `offset = max_face_id(A) + 1`, чтобы идентификаторы граней $A$ и $B$ не пересекались.
   - Для `csg_difference(A, B)` грани операнда $B$, выжившие в результате (стенки выреза), получают в реестре `faces` префикс `cut:` (или `hole_wall:` при вызове из `cut_hole`), а их `normal_or_axis` (для плоских граней) учитывает инверсию нормали вычитаемого тела.
   - В `optimize_csg_mesh_quality` (`csg.rs:808`) проследи, чтобы операции Delaunay edge flip, ear-clipping и Link-Condition collapse выполнялись только внутри одного и того же `face_id` (не смешивали треугольники с разными `face_id` и не сдвигали вершины, лежащие на границе между разными `face_id`).
4. **Вывод в `query_part_geometry` (`crates/mcp-cad-server/src/tools/cad.rs:1359`):**
   - Добавь в JSON-ответ `query_part_geometry` поле `"faces": part.mesh.faces` (сохранив существующее поле `"detected_planar_faces"` для обратной совместимости).

#### Критерии приёмки
- Для каждого из 5 примитивов (`box`, `cylinder`, `cone`, `sphere`, `polygon_prism`):
  - `mesh.face_ids.len() == mesh.indices.len()`;
  - сумма `face.area` по всем граням из `mesh.faces` равна `mesh.surface_area()` с точностью `< 1e-9`;
  - имена и типы граней (`FaceKind`) соответствуют `docs/FACE_NAMING_DESIGN.md`.
- Тест прохождения через CSG: бокс $40\times 40\times 10\text{ мм}$ минус цилиндрическое сквозное отверстие $r = 10\text{ мм}$ (`cut_hole`):
  - в `mesh.faces` присутствуют все 6 исходных граней бокса (`top` и `bottom` уменьшили площадь ровно на $\pi r^2$ в пределах $1\%$ полигональной аппроксимации) и цилиндрическая грань стенки отверстия (`kind == FaceKind::Cylinder`, `radius == Some(10.0)`, площадь $2\pi r h$ с точностью $< 1\%$).
- Повторный вызов `part.rebuild()` (`dag.evaluate()`) выдаёт детерминированные, стабильные имена граней.
- Время выполнения `cargo test -p mcp-cad-core --test csg_invariants` не ухудшилось более чем на 20%.
- Задачи **`P2-T5`**, **`P2-T6`**, **`P2-T7`** в `TASKS.md` переведены в статус `✅ DONE`.

#### Не делать
- Не удалять поле `detected_planar_faces` из `GeometryInspection` и ответа `query_part_geometry`, чтобы не сломать существующие контрактные тесты в `mcp_contract_tests.rs`.
- Не менять формат сериализации старых JSON без `face_ids` (`#[serde(default)]` обязателен).

---

### P1-K5 — Инструмент `extrude_sketch` и скругление 2D-контуров на базе `CrossSection` + `Manifold::extrude`

**Размер:** M | **Зависит от:** `P1-K4` | **Ветка:** `agent/P1-K5`

#### Контекст (проверено)
- В историческом бэклоге проекта (`PRD.md`, `TASKS.md` `TASK-007` «ExtrudeSketch» и `TASK-008` «Fillet») отсутствовала возможность выдавливания произвольного 2D-контура (с внутренними отверстиями/пазами и скруглением углов контура).
- В подключённом крейте `manifold-rust v0.19.2` есть готовый модуль 2D-геометрии `manifold_rust::cross_section::CrossSection` (на базе `clipper2-rust`) и метод `Manifold::extrude`, поддерживающие:
  - построение 2D-полигонов с произвольным числом внешних контуров и внутренних вырезов (`CrossSection::from_polygons` с правилом заполнения `FillRule::EvenOdd` / `Positive`);
  - булевы операции над 2D-контурами (`union`, `difference`, `intersection`);
  - точное скругление углов 2D-профиля радиусом $R$ через `CrossSection::offset` (`JoinType::Round`: скругление наружных углов комбинацией `offset(-r)` $\to$ `offset(+r)`, скругление внутренних углов комбинацией `offset(+r)` $\to$ `offset(-r)`);
  - линейное выдавливание вдоль оси $Z$ с заданным числом делений по высоте, углом крутки `twist_degrees` и масштабом верхнего торца `scale_top` (`Manifold::extrude`).

#### Задача
1. В [`crates/mcp-cad-core/src/primitives.rs`](../crates/mcp-cad-core/src/primitives.rs) реализуй функцию:
   ```rust
   pub fn create_extrude_sketch_with_settings(
       outer_loop: &[[f64; 2]],
       holes: &[Vec<[f64; 2]>],
       height: f64,
       corner_radius: Option<f64>,
       twist_deg: Option<f64>,
       scale_top: Option<[f64; 2]>,
       settings: &TessellationSettings,
   ) -> Result<TriangleMesh, CadError>
   ```
   - Проверяет валидность: `outer_loop.len() >= 3`, `height > 0.0`, ненулевая площадь контура после применения `corner_radius`, отсутствие вырождения.
   - Строит `CrossSection` (автоматически нормализуя обход `outer_loop` против часовой стрелки и `holes` по часовой стрелке для `FillRule::EvenOdd`).
   - Если задан `corner_radius = Some(r)` ($r > 0$), применяет скругление 2D-углов профиля через `CrossSection::offset` с `JoinType::Round` и плотностью сегментов из `settings.calculate_segments(r)`.
   - Выполняет `Manifold::extrude` на высоту `height` и центрирует полученное тело по $Z \in [-h/2, +h/2]$ (согласованно с `create_box`, `create_cylinder`, `create_polygon_prism`).
   - Прогоняет сетку через `manifold_to_mesh` + `optimize_csg_mesh_quality`, размечает именованные грани (`cap_top`, `cap_bottom`, `side_outer`, `hole_0_wall`, …) и возвращает `TriangleMesh`.
2. В [`crates/mcp-cad-core/src/dag.rs`](../crates/mcp-cad-core/src/dag.rs) добавь вариант `FeatureOp::ExtrudeSketch`:
   ```rust
   ExtrudeSketch {
       outer_loop: Vec<[f64; 2]>,
       #[serde(default)]
       holes: Vec<Vec<[f64; 2]>>,
       height: f64,
       #[serde(default)]
       corner_radius: Option<f64>,
       #[serde(default)]
       twist_deg: Option<f64>,
       #[serde(default)]
       scale_top: Option<[f64; 2]>,
       #[serde(default)]
       position: Option<[f64; 3]>,
       #[serde(default)]
       rotation_deg: Option<[f64; 3]>,
   }
   ```
   и его обработку в `FeatureTree::evaluate_with_settings` (первый узел — создаёт тело, последующий — объединяет через `csg_union`).
3. В [`crates/mcp-cad-server/src/tools/cad.rs`](../crates/mcp-cad-server/src/tools/cad.rs) и [`crates/mcp-cad-server/src/tools/mod.rs`](../crates/mcp-cad-server/src/tools/mod.rs):
   - Поддержи `"primitive_type": "extrude_sketch"` (и алиас `"sketch_extrude"`) в `add_feature_primitive` и `modify_feature`, а также зарегистрируй удобный инструмент верхнего уровня `extrude_sketch` (или задокументируй в схеме `add_feature_primitive` в `tools/mod.rs` — не ломая существующие контрактные тесты на список инструментов в `mcp_contract_tests.rs`! Проверь `grep -n "list_tools" crates/mcp-cad-server/tests/*.rs`, прежде чем менять число инструментов в `list_tools()`).
4. Добавь тесты:
   - В `crates/mcp-cad-core/tests/primitive_invariants.rs`: Г-образный кронштейн (`L-bracket`) с круглым/прямоугольным вырезом в `holes` и скруглением углов `corner_radius = 0.002`:
     - `validate_mesh` возвращает `PASS` (`is_watertight == true`, `lt_1_deg_count == 0`, `lt_5_deg_ratio <= 1.0%`);
     - `signed_volume()` совпадает с аналитическим $(A_{\text{outer}} - A_{\text{holes}} - \Delta A_{\text{fillet}}) \cdot h$ с точностью $< 0.5\%$;
     - `faces` содержит `cap_top` и `cap_bottom` с площадями, равными площади 2D-сечения.
   - В `crates/mcp-cad-server/tests/mcp_contract_tests.rs`: контрактный тест создания детали через `add_feature_primitive` с `primitive_type: "extrude_sketch"`, модификации `height` через `modify_feature` и инспекции через `query_part_geometry`.

#### Критерии приёмки
- `cargo test -p mcp-cad-core --test primitive_invariants` проходит (`PASS`).
- `cargo test -p mcp-cad-server --test mcp_contract_tests` проходит (`PASS`).
- `cargo clippy --workspace --all-targets -- -D warnings` — 0 ошибок, 0 предупреждений.

#### Не делать
- Не ломать проверку фиксированного списка инструментов в `mcp_contract_tests.rs` (если тест в `mcp_contract_tests.rs` проверяет точное число `tools.len() == 28`, реализуй `extrude_sketch` как `primitive_type: "extrude_sketch"` внутри `add_feature_primitive` и обнови схему `add_feature_primitive` в `tools/mod.rs`).

---

### P1-K6 *(Опционально — по решению `[DECISION-4]`)* — Экспорт параметрического дерева `FeatureTree` в ISO 10303-21 `STEP` через `monstertruck-io`

**Размер:** M | **Зависит от:** `P1-T3` | **Ветка:** `agent/P1-K6`

#### Контекст (проверено)
- В [`docs/adr/0001-geometry-kernel.md`](../docs/adr/0001-geometry-kernel.md) (`[DECISION-1]` и `[DECISION-4]`) описан гибридный вариант **A + C**: основное сеточное ядро `manifold-rust v0.19.2` дополняется чистым Rust B-Rep пакетом `monstertruck v0.4.1` (`Apache-2.0`) для экспорта аналитических параметрических деталей в промышленный стандарт **ISO 10303-21 STEP (`AUTOMOTIVE_DESIGN` AP214)**.
- Работоспособность связки `monstertruck-modeling v0.4.1` + `monstertruck-solid v0.4.1` + `monstertruck-fillet v0.4.1` + `monstertruck-io v0.4.1` уже доказана в прототипе [`spikes/monstertruck_brep/src/main.rs`](../spikes/monstertruck_brep/src/main.rs) (строки 263–295: экспорт B-Rep тела фланцевой втулки в STEP занимает `1.25 мс`, генерируя валидный файл `ISO-10303-21;` размером `232 899 байт`).

#### Задача
1. Добавь в [`crates/mcp-cad-core/Cargo.toml`](../crates/mcp-cad-core/Cargo.toml) опциональные зависимости под feature-флагом `step-export` (включённым по умолчанию в `default = ["step-export"]`):
   - `monstertruck-modeling = "0.4.1"`
   - `monstertruck-solid = "0.4.1"`
   - `monstertruck-fillet = "0.4.1"`
   - `monstertruck-step = "0.4.1"` (или `monstertruck-io = "0.4.1"`)
2. Создай модуль `crates/mcp-cad-core/src/export_step.rs` с функцией:
   ```rust
   pub fn export_feature_tree_to_step(tree: &FeatureTree, path: &Path) -> Result<usize, CadError>
   ```
   - Транслирует поддерживаемые аналитические узлы `FeatureTree` (`AddBox`, `AddCylinder`, `AddCone`, `AddPolygonPrism`, `AddRevolve`, `CutHole`, `Transform`) в B-Rep `monstertruck_modeling::Solid` (по образцу `spikes/monstertruck_brep/src/main.rs:36-158`).
   - Сериализует результирующий `Solid` в строку ISO 10303-21 STEP (`monstertruck_step::out::StepModel` / `CompleteStepDisplay`) и записывает в файл `path`.
   - Если `FeatureTree` содержит чисто сеточные операции без B-Rep эквивалента (`BooleanDifferenceMesh`, `AddWheelRim`), возвращает информативную ошибку `CadError::ExportError("STEP export requires parametric B-Rep features; use export_stl for arbitrary triangle meshes")`.
3. Добавь unit-тест `test_export_flanged_bushing_to_step` в `crates/mcp-cad-core`:
   - Строит `FeatureTree` фланцевой втулки (`AddCylinder` + `AddBox` + `CutHole`), экспортирует во временный `.step` файл (`tempfile`), проверяет заголовок `ISO-10303-21;`, наличие секций `HEADER;`, `DATA;`, `CLOSED_SHELL`, `ADVANCED_FACE`, `END-ISO-10303-21;` и ненулевой размер файла.

#### Критерии приёмки
- `cargo test -p mcp-cad-core export_step` проходит (`PASS`).
- `cargo clippy --workspace --all-targets -- -D warnings` — 0 ошибок, 0 предупреждений.

#### Не делать
- Не использовать `monstertruck-meshing` в основном контуре оценки `FeatureTree::evaluate` (так как в `P1-T3` доказано, что триангулятор `monstertruck-meshing` генерирует `1 327` игольчатых треугольников `< 1°` и проваливает `validate_mesh`; тесселяция и булевы операции основного контура остаются на `primitives.rs` + `manifold-rust`).
