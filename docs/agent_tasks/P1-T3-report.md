# Отчёт о выполнении задачи P1-T3

## 1. Задача и что было сделано
- **ID**: `P1-T3`
- **Название**: ADR: выбор геометрического ядра **[DECISION]**
- **Фаза**: P1 — Архитектура и Геометрическое Ядро
- **Ветка**: `agent/P1-T3`

### Выполненные работы и созданные файлы
В соответствии со спецификацией задачи `P1-T3` (`fix/P1_architecture.md`) и правилами для задач с маркером **`[DECISION]`** (`fix/00_README_PREAMBLE.md`: *изложить варианты с плюсами и минусами, предложить выбор по умолчанию, выполнить только безопасную часть — документ и изолированные прототипы в `spikes/` вне основного Cargo workspace*):

1. **Подготовлен архитектурный документ [`docs/adr/0001-geometry-kernel.md`](../adr/0001-geometry-kernel.md)**:
   - Проведено сравнение 4 архитектурных вариантов по 10 критериям из `fix/P1_architecture.md`:
     - **Вариант A**: Поверхностные сетки + топологически гарантированный Mesh Boolean (`manifold-rust v0.19.2`, 100% Pure Rust порт C++ библиотеки Manifold Эмметта Лалора, `Apache-2.0`, `build = false`, без FFI/C++ компилятора). Дополнительно проверены альтернативные крейты на crates.io (`manifold-csg v0.4.1`, `boolmesh v0.2.2`, `csgrs v0.20.1`).
     - **Вариант B**: Полноценное C++ B-Rep ядро OpenCascade (`opencascade v0.3.0` / `occt-sys v7.8.1`, `LGPL-2.1`).
     - **Вариант C**: Pure Rust B-Rep ядро `truck` / активно поддерживаемый форк `monstertruck v0.4.1` (`monstertruck-modeling`, `monstertruck-solid`, `monstertruck-fillet`, `monstertruck-meshing`, `monstertruck-io`, `Apache-2.0`).
     - **Вариант D**: Доработка текущего самописного BSP-CSG (`crates/mcp-cad-core/src/csg.rs`).
   - Приведены проверенные по `cargo info` (реестр crates.io) версии, лицензии (`MIT` проекта против `Apache-2.0` и `LGPL-2.1`), статус `build.rs`, системные зависимости и ссылки на репозитории.
   - Сформирована таблица влияния каждого варианта на все 5 крейтов workspace (`mcp-cad-core`, `mcp-cad-fea`, `mcp-cad-physics`, `mcp-cad-viewer`, `mcp-cad-server`).
   - Подготовлена декомпозиция миграции для `P1-T4` на 5 атомарных подзадач (`P1-K1`…`P1-K5`, каждая `<= 1 задача агента`).
   - Вынесены 2 вопроса **`[DECISION]`** с явной рекомендацией по умолчанию (**Гибридная стратегия A + опциональный C**: `manifold-rust v0.19.2` как основное булево ядро и трекер `face_id` в `mcp-cad-core` + `monstertruck-step` для нативной записи ISO 10303-21 STEP).

2. **Реализован и проверен Прототип №1 (Кандидат A — `manifold-rust v0.19.2`) в `spikes/manifold_rust/`**:
   - `spikes/manifold_rust/Cargo.toml` — отдельный пакет (`[workspace]`), не изменяющий корневой `Cargo.toml`.
   - `spikes/manifold_rust/src/main.rs` — реализация эталонной детали `P1-T3` (цилиндр $R=15\text{ мм}, h=50\text{ мм}$ минус соосное отверстие $\varnothing 20\text{ мм}$ плюс квадратный фланец $40\times 40\times 10\text{ мм}$ плюс скругление кромки $R=2\text{ мм}$):
     - Прямое сравнение одного и того же булева конвейера на идентичных входных сетках `mcp-cad-core` между `manifold-rust` и текущим BSP-CSG (`crates/mcp-cad-core/src/csg.rs`), а также нативными примитивами `Manifold::cylinder` / `Manifold::cube` и конструктивным кольцевым скруглением $R=2\text{ мм}$.
     - Проверка сквозного прохождения идентификаторов граней через `MeshGLP::face_id` и `run_original_id` и точное извлечение верхнего торца (`cap_top`, `face_id = 101`).
     - Проверка проблемного бенчмарка `T08_hex_nut_m10` (`P3-T11` — вычитание винтовой канавки `create_helical_groove` из шестигранной гайки М10 с фаской).
   - `spikes/manifold_rust/README.md` — инструкция по запуску и сводка метрик.

3. **Реализован и проверен Прототип №2 (Кандидат C — `monstertruck v0.4.1` Pure Rust B-Rep) в `spikes/monstertruck_brep/`**:
   - `spikes/monstertruck_brep/Cargo.toml` — отдельный пакет (`[workspace]`) с `monstertruck-modeling`, `monstertruck-solid`, `monstertruck-fillet`, `monstertruck-meshing`, `monstertruck-io`.
   - `spikes/monstertruck_brep/src/main.rs` — построение B-Rep тел (`Solid`) через `builder::tsweep` и `builder::rsweep`, булевы операции `solid::or` / `solid::and`, аналитическое скругление $R=2\text{ мм}$ через `monstertruck_fillet::fillet_with_side` и `fillet_along_wire`, триангуляция в `PolygonMesh` и конвертация в `mcp_cad_core::topology::TriangleMesh` с проверкой `validate_mesh`, проверка сохранности `FaceID` до/после булевых операций и экспорт в ISO 10303-21 STEP.
   - `spikes/monstertruck_brep/README.md` — инструкция по запуску и сводка метрик.

---

## 2. Как проверено (точные команды и вывод)

### 2.1. Запуск Прототипа №1 (`spikes/manifold_rust`)
Команда:
```bash
cargo run --release --manifest-path spikes/manifold_rust/Cargo.toml
```
Вывод:
```text
=== P1-T3 Spike 1: manifold-rust (v0.19.2, Pure Rust port of Manifold) ===
Analytical targets (64-gon exact / smooth):
  Bushing (no fillet):   28546.1940 mm³ (smooth: 28576.1052 mm³)
  Bushing (with R=2mm):  28630.6928 mm³ (smooth: 28661.8256 mm³)
  Top end-cap area:      392.0686 mm² (smooth: 392.6991 mm²)

--- [1] Baseline: Current mcp-cad-core BSP-CSG (flanged bushing, no fillet) ---
  Time:               940.892 ms
  Triangles:          708
  Vertices:           350
  Volume:             28546.1940 mm³ (err vs 64-gon: 0.0000%, err vs smooth: 0.1047%)
  validate_mesh:      FAIL
  is_watertight:      false (open_edges=24, non_manifold=0)
  Euler χ / genus:    χ=-8, genus=5.0
  Sliver gate:        lt_1_deg=0, lt_5_deg_ratio=0.14% (min_angle=3.266°)

--- [2] Candidate A1: manifold-rust on mcp-cad-core meshes (no fillet) ---
  Time:               25.007 ms
  Triangles:          1236
  Vertices:           618
  Volume:             28546.1940 mm³ (err vs 64-gon: 0.000000%, err vs smooth: 0.1047%)
  validate_mesh:      WARN
  is_watertight:      true (open_edges=0, non_manifold=0)
  Euler χ / genus:    χ=0, genus=1.0
  Sliver gate:        lt_1_deg=0, lt_5_deg_ratio=2.91% (min_angle=2.142°)
  Top end-cap query:  face_id=101 -> 192 triangles, area=392.0686 mm² (err vs 64-gon: 0.0000%)

--- [3] Candidate A2: manifold-rust native primitives (no fillet) ---
  Time:               5.002 ms
  Triangles:          796
  Vertices:           398
  Volume:             28546.1940 mm³ (err vs 64-gon: 0.000000%)
  validate_mesh:      WARN
  is_watertight:      true (open_edges=0, non_manifold=0)
  Euler χ / genus:    χ=0, genus=1.0
  Sliver gate:        lt_1_deg=0, lt_5_deg_ratio=4.52% (min_angle=2.155°)

--- [4] Candidate A3: manifold-rust WITH R=2mm neck fillet ---
  Time:               21.559 ms
  Triangles:          1796
  Vertices:           898
  Volume:             28629.5919 mm³ (err vs 64-gon analytical: 0.0038%, err vs smooth: 0.1125%)
  validate_mesh:      PASS
  is_watertight:      true (open_edges=0, non_manifold=0)
  Euler χ / genus:    χ=0, genus=1.0
  Sliver gate:        lt_1_deg=0, lt_5_deg_ratio=0.67% (min_angle=1.805°)

--- [5] Bonus P3-T11 check: T08 Hex Nut M10 with Helical Thread Groove ---
  Current BSP-CSG:    time=257.54 ms, tris=3215, watertight=false, open_edges=615, genus=98.0, status=FAIL
  manifold-rust:      time=18.87 ms, tris=3534, watertight=true, open_edges=0, genus=1.0, status=WARN
```

### 2.2. Запуск Прототипа №2 (`spikes/monstertruck_brep`)
Команда:
```bash
cargo run --release --manifest-path spikes/monstertruck_brep/Cargo.toml
```
Вывод:
```text
=== P1-T3 Spike 2: monstertruck v0.4.1 (Pure Rust B-Rep Kernel, fork of truck) ===
Analytical targets (smooth B-Rep):
  Bushing (no fillet):   28576.1052 mm³
  Bushing (R=2mm neck):  28661.8256 mm³
  Top end-cap area:      392.6991 mm²

--- [1] B-Rep Primitive Construction & Face Topology Inspection ---
  Cylinder solid:     3 faces, top cap FaceID=0x557706af5cf0
  Flange box solid:   6 faces
  Hole cutter solid:  3 faces

--- [2] B-Rep Boolean Operations (Union + Difference) ---
  Boolean Union (Cyl ∪ Flange): 782.138 ms -> OK (faces=9)
  Boolean Diff  ((Cyl ∪ Flange) \ Hole): 51.821 ms -> OK (faces=10)
  Total B-Rep boolean time: 833.958 ms
  Face count after booleans: 10
  Top cap FaceID preserved across booleans? false (orig=0x557706af5cf0, post=Some(0x557706b33fe0))

--- [3] B-Rep Edge Fillet R=2.0 mm (monstertruck-fillet) ---
  3a. Fillet R=2mm on straight flange edge: 7.291 ms -> OK (7 faces)
  3b. Fillet R=2mm on cylinder-flange circular intersection seam (4 quarter-arcs):
      fillet_along_wire result: 3.800 ms -> OK! (faces=14)

--- [4] B-Rep Triangulation & mcp-cad-core::validate_mesh ---
  Meshing time (tol=0.05 mm): 25.495 ms
  Total pipeline time (B-Rep booleans + meshing): 859.454 ms
  Triangles:          11284
  Vertices:           5642
  Volume:             28551.8398 mm³ (err vs exact smooth: 0.0849%)
  validate_mesh:      FAIL
  is_watertight:      true (open_edges=0, non_manifold=0)
  Euler χ / genus:    χ=0, genus=1.0
  Sliver gate:        lt_1_deg=1327, lt_5_deg_ratio=11.76% (min_angle=0.002°)
  Top end-cap query (by B-Rep face / Z=25 plane): 622 triangles, area=392.2016 mm² (err vs exact: 0.1267%)

  [4b] Triangulating filleted flange solid (R=2mm edge fillet):
  Filleted flange:    time=3.180 ms, tris=324, watertight=true, open_edges=0, min_angle=2.456°, status=WARN

--- [5] Native ISO 10303-21 STEP Export (monstertruck-step) ---
  STEP generation:    1.253 ms, size=232899 bytes, header valid=true
```

---

## 3. Тесты до и после

Основной Cargo workspace не модифицировался. Контрольный прогон тестов и линтера:
- **До изменений**: `156 passed, 0 failed, 0 ignored` (`cargo test --workspace`), `0 warnings` (`cargo clippy --workspace --all-targets -- -D warnings`).
- **После изменений**:
  - `cargo clippy --workspace --all-targets -- -D warnings` $\to$ **0 ошибок, 0 предупреждений**.
  - `cargo test -p mcp-cad-core --lib` $\to$ **50 passed; 0 failed; 0 ignored**.
  - `cargo run --release --manifest-path spikes/manifold_rust/Cargo.toml` $\to$ **PASS (exit code 0)**.
  - `cargo run --release --manifest-path spikes/monstertruck_brep/Cargo.toml` $\to$ **PASS (exit code 0)**.

---

## 4. Открытые вопросы `[DECISION]`, ограничения и остаточные риски

В [`docs/adr/0001-geometry-kernel.md`](../adr/0001-geometry-kernel.md) (раздел 8) зафиксированы два архитектурных вопроса **`[DECISION]`** для подтверждения перед запуском `P1-T4` / `P2-T5`:

1. **`[DECISION-1]` Выбор целевого геометрического ядра для `mcp-cad-core`**:
   - **Вариант 1 (Рекомендуемый по умолчанию — Гибридный A + опциональный C для STEP)**:
     - Использовать **`manifold-rust v0.19.2`** (`Apache-2.0`, 100% Pure Rust) как основное булево ядро и механизм сквозного отслеживания граней (`face_id`) в `mcp-cad-core`. Это ускоряет CSG в **37–188 раз** относительно текущего BSP-CSG, гарантирует `is_watertight = true` (`open_edges = 0`, $\chi = 0, g = 1$) как на фланцевой втулке, так и на резьбе гайки М10 (`T08_hex_nut_m10`), и сохраняет лицензионную чистоту (`MIT` + `Apache-2.0`) и нулевые C++ зависимости.
     - Использовать существующий пост-оптимизатор `optimize_csg_mesh_quality` (после `Manifold::to_mesh` с защитой границ `face_id`), чтобы гарантировать прохождение Sliver Gate (`lt_5_deg_ratio <= 1.0%`).
     - При необходимости экспорта в STEP подключать `monstertruck-step` (`Apache-2.0`) как изолированный сериализатор аналитических примитивов.
   - **Вариант 2 (Чистый Вариант A)**: только `manifold-rust v0.19.2` без `monstertruck-step`.
   - **Вариант 3 (Чистый Вариант C)**: полный переход на B-Rep `monstertruck v0.4.1` (высокий риск из-за $834\text{ мс}$ на булевых операциях, сброса `FaceID` и $1\,327$ игольчатых треугольников `< 1°` в `monstertruck-meshing`).
   - **Вариант 4 (Вариант D)**: остаться на самописном BSP-CSG и чинить топологические разрывы вручную.

2. **`[DECISION-2]` Гранулярность `face_id` на криволинейных поверхностях (вход для `P2-T5`)**:
   - **Рекомендация по умолчанию**: присваивать единый логический `face_id` каждой семантической поверхности примитива (`cap_top`, `cap_bottom`, `side_outer`, `hole_<id>_wall`, `fillet_<id>`), а не отдельному четырехугольному сегменту разбиения окружности. Как показал спайк `manifold-rust`, это позволяет за $O(F)$ напрямую извлекать все треугольники торца (`face_id = 101` $\to$ `192` треугольника, площадь `392.0686 мм²`, ошибка `0.0000%`).

---

## 5. Побочные находки (занесены в `TASKS.md`)

При проведении экспериментальных замеров в рамках `P1-T3` обнаружены 2 новых дефекта в `crates/mcp-cad-core`, которые зарегистрированы в `TASKS.md` (задача **`P3-T12`**):

1. **Нарушение водонепроницаемости (`open_boundary_edges = 24`) в текущем BSP-CSG (`crates/mcp-cad-core/src/csg.rs`) на составной фланцевой втулке**:
   - При последовательном выполнении `csg_union(&create_cylinder(15.0, 50.0, 64), &create_box(40.0, 40.0, 10.0))` и `csg_difference(&u, &create_cylinder(10.0, 52.0, 64))` текущий BSP-CSG тратит `940.89 мс` и выдаёт сетку с `24` открытыми граничными рёбрами (`is_watertight = false`, $\chi = -8$, `genus = 5.0`, `validate_mesh = FAIL`), тогда как `manifold-rust` на тех же самых входных сетках даёт `open_boundary_edges = 0`, `is_watertight = true`, $\chi = 0, g = 1.0$ за `25.01 мс`.
2. **Некорректная формула топологического рода (`genus`) для многокомпонентных сеток в `crates/mcp-cad-core/src/diagnostics.rs:414`**:
   - В `diagnostics.rs:414` род поверхности вычисляется по формуле для одной связной компоненты: `let genus = (2.0 - euler_characteristic as f64) / 2.0;`, игнорируя уже посчитанное на строке 409 число связных компонент `connected_components` ($k$). Для замкнутой ориентируемой поверхности из $k$ связных компонент обобщённая формула Эйлера имеет вид $\chi = 2k - 2\sum g_i \implies g = k - \chi / 2$ (при `open_boundary_edges == 0`). Из-за использования $(2 - \chi)/2$ для двух непересекающихся сфер ($k = 2, \chi = 4$) диагностика вернёт `genus = -1.0` вместо `0.0`.
