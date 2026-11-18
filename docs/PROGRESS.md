# Журнал выполнения и верификации (`docs/PROGRESS.md`)

Данный документ ведётся по ходу работ для обеспечения непрерывности при возможных прерываниях среды и фиксации реальных результатов команд после каждой фазы.

---

## Фаза 1: Окружение и базовая линия

### 1.1. Установка и проверка инструментов окружения
- **ОС / ядро**: `Linux localhost 4.19.0-gvisor #1 SMP x86_64 GNU/Linux` (4 vCPU, 12 GiB RAM, 186 GiB свободного места на диске).
- **Системные пакеты**: установлены `build-essential` (`gcc 12.3.0`), `pkg-config 1.8.1`, `libssl-dev 3.0.22`, `mesa-vulkan-drivers` (программный драйвер Vulkan Lavapipe `lvp_icd.x86_64.json`), кросс-компилятор `gcc-mingw-w64-x86-64` (`x86_64-w64-mingw32-gcc 12-win32`).
- **Rust toolchain**: установлен через `rustup` (`stable-x86_64-unknown-linux-gnu`):
  - `cargo 1.99.1 (5f94df478 2026-08-27)`
  - `rustc 1.99.1 (58cf8f9c3 2026-10-13)`
  - Компоненты: `clippy`, таргет `x86_64-pc-windows-gnu`.
- **Node.js / TypeScript**: `node v22.23.3`, `npm 10.9.9`, `tsx`.
- **Python / эталонные пакеты верификации физики**: `Python 3.11.2`, `pip 26.3`, `mujoco 3.17.0`, `numpy 2.4.6`, `scipy 1.17.1`.

### 1.2. Аудит исходного состояния (проверено запуском)
1. **Сборка на оригинальном коммите `02c7339`**: `mcp-cad-core` не компилировался (9 ошибок: неопределённый тип `Winding`, неверная арность `add_halving_ring_transition_2d` / `add_bridged_ring_strip_2d`, конфликт мутабельного заимствования `push_quad` в `primitives.rs`, неопределённая переменная `applied` в `query_optimizer.rs:1331`; плюс `EngineState::new_shared()` в `mcp_contract_tests.rs`).
2. **Дефект крышек примитивов (`primitives.rs:174-211`)**: в `add_bridged_ring_strip_2d` и `add_halving_ring_transition_2d` ветки `if is_top` / `else` имели инвертированный порядок обхода вершин (для перехода от внешнего кольца `ring_a` к внутреннему `ring_b` векторное произведение $(a_{i+1}-a_i)\times(b_i-a_i)$ направлено вдоль $+Z$, но код реверсировал обход для `is_top == true`), что выворачивало крышки цилиндров, конусов, усечённых конусов и шестерён и роняло 14 тестов.
3. **Замечания линтера `cargo clippy --workspace --all-targets -- -D warnings`**:
   - `unused_mut` в `primitives.rs:930`;
   - `unused_assignments` и `unused_variables` в `query_optimizer.rs:289, 1141, 1217`;
   - `clippy::manual_is_multiple_of` в `primitives.rs:270`;
   - `clippy::manual_range_contains` в `csg.rs:597`.
4. **Тесты TypeScript (`npx --yes tsx --test src/cad/cad_engine.test.ts`)**: 25 из 26 проходили, падал `validates logical Engineering Workbenches and FEA quick load presets` (`5 !== 4` на строке 507 из-за добавления 5-го пресета `combined_bending_torsion` в `workbenches.ts`).
5. **Бенчмарк `bench/run_benchmark.ts`**: на исходном коде проходило 11 из 15 задач (`T11`, `T12`, `T13`, `T15` падали).
6. **Несоответствия `CHANGELOG.md` реальности на исходном коммите**:
   - отсутствовали `.github/workflows/ci.yml`, `LICENSE-MIT`, `LICENSE-APACHE`;
   - в `mcp-cad-server` отсутствовали флаги `--host` и `--auth-token`;
   - в `export_stl` / `import_stl` на стороне Rust отсутствовала проверка песочницы `./output`;
   - в `mcp-cad-fea` реализован только линейный элемент `TET4` (элемент `TET10` в Rust-коде отсутствовал).

### 1.3. Выполненные исправления и верификация Фазы 1
- Исправлен порядок обхода треугольников в `add_bridged_ring_strip_2d`, `add_halving_ring_transition_2d` и `create_wheel_rim` (`crates/mcp-cad-core/src/primitives.rs`).
- Исправлена адаптивная триангуляция фасок призмы (`create_polygon_prism`) и торцевых крышек неполного тела вращения (`triangulate_simple_polygon_2d` с выбором максимального минимального угла и переворотами Делоне) в `crates/mcp-cad-core/src/primitives.rs`.
- В `crates/mcp-cad-core/src/csg.rs` реализована двухфазная постобработка CSG-сеток (`collapse_coplanar_sliver_edges` с сохранением плоскостей граней и условием связности 2-многообразия + `refine_edges_above_length` + `optimize_coplanar_delaunay_edges`), устраняющая игольчатые (`< 1°`) и вырожденные (`< 5°`) треугольники на сквозных, глухих, наклонных и шпоночных отверстиях и цепочках булевых операций.
- Устранены все предупреждения компилятора и ошибки `clippy` в `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/src/csg.rs`, `crates/mcp-cad-core/src/query_optimizer.rs`.
- Исправлена проверка контракта ошибки MCP в `test_bug016_revolve_boundary_and_self_intersection_validation` (`crates/mcp-cad-server/tests/mcp_contract_tests.rs`) и проверка количества пресетов FEA в `src/cad/cad_engine.test.ts`.
- Результат `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test --workspace`: **126 из 126 тестов проходят** (`mcp-cad-core`: 64, `mcp-cad-fea`: 11, `mcp-cad-physics`: 6, `mcp-cad-viewer`: 10, `mcp-cad-server`: 35).
- Результат `cargo clippy --workspace --all-targets -- -D warnings`: **0 ошибок, 0 предупреждений**.
- Результат `npx --yes tsx --test src/cad/cad_engine.test.ts`: **26 из 26 тестов проходят**.

---

## План и сводный статус фаз P2–P4 (актуализировано в `P4-T10`, 18.11.2026)

- **Фаза 2 (P2-T1..T40 — ✅ ВЫПОЛНЕНО)**: Единый слой команд и протокольных ошибок (`ErrorEnvelope`, 36 кодов) в Rust, контракт единиц измерения (`mm` по умолчанию / `m`), топологическое именование граней (`FaceInfo`/`FaceSelector`), слой инженерных требований (`set_requirements`/`check_requirements`), персистентность (`save_project`/`load_project`), автономный запуск превью из `stdio` и руководство `SKILL.md` / `AGENT_GUIDE.md`.
- **Фаза 3 (P3-T1..T24 — ✅ ВЫПОЛНЕНО)**:
  - Аналитическая верификация FEA, предполётная проверка `validate_mesh`, баланс реакций, сходимость Ричардсона и квадратичные элементы **`TET10`** (`element_order = 2`, ошибка прогиба консоли снижена с `20.14%` до `0.25%`).
  - Извлечение пиковых нагрузок из симуляции и квазистатический МКЭ (`get_part_load_cases`, `run_fea_from_simulation`), расчёт зубчатых передач по Льюису и Герцу/AGMA (`check_gear_strength`), анализ подвижности по Грюблеру и SVD (`analyze_mechanism`), детекция интерференции и развёртка хода замкнутых механизмов (`check_interference`, `sweep_joint`), изоляция `qa_wide_scan` и многопоточного рендеринга WGPU.
- **Фаза 4 (`P4-T0`–`P4-T10` — ✅ ВЫПОЛНЕНО; открытых задач в бэклоге нет)**:
  - Добавлен 41-й MCP-инструмент `apply_recipe` (`flanged_bushing`, `stepped_shaft`, `gear_transmission`).
  - Реализован и утверждён (`ADR-004`) собственный многотельный физический решатель в двойной точности **`f64`** (`mcp-cad-physics`):
    - `rigid_body.rs` (`P4-T2`, `P4-T9`): `RigidBody`, `MassProperties` (двухпроходный тензор инерции по знаковым тетраэдрам с учётом полостей и отверстий, прямое чтение `part.mesh()`) и `ColliderShape` (`ConvexHull`, `TriMesh`, `Compound`, `Cube`, `Sphere`, `Cylinder`).
    - `solver.rs` (`P4-T3`, `P4-T7`): `MultiBodySolver` (`DirectKktIndex2Projection` — неявно-среднеточечный Lie–Poisson предиктор вращения в СК тела + регуляризованная KKT / Active-Set Box-LCP система + трапецеидальный шаг + SVD-проекция на многообразие $\Phi(\mathbf{q})=\mathbf{0}$, прямая передача момента в `Gear`).
    - `collision.rs` (`P4-T4`): AABB Sweep-and-Prune, `TriMeshBvh`, аналитика + GJK/EPA + планарный клиппинг и изотропный конусный контактный решатель Кулона ($\|\boldsymbol{\gamma}_t\|_2 \le \mu \gamma_n$).
    - `simulator.rs` (`P4-T5`, `P4-T6`, `P4-T8`): прямое извлечение реакций шарниров $\boldsymbol{\lambda}$ (`Pinned KKT Re-solve`), полное удаление зависимости `rapier3d`/`parry3d` из `Cargo.toml`/`Cargo.lock`, очистка `f32`-приведений, обновление UI веб-студии (`src/`) и `NaN`-безопасный `rotation_vector` в `ClosedLoopSolver` (`atan2` + `clamp(-1.0, 1.0)`).
  - Гигиена ядра и тестов (`part.rs`, `physics_tests.rs`, `P4-T9`–`P4-T10`): `Part::mesh()` возвращает каноническую сетку `&self.mesh`, `Part::mesh_mut()` сбрасывает кэш `cached_mesh`/`cached_settings`, `Part::tessellate()` проверяет `*cached == self.mesh`; тест `test_p3_t10_benchmark_check_interference_10_parts_45_pairs` стабилизирован под параллельной нагрузкой на CPU (детерминированные инварианты broad-phase/CSG + защитный предел `< 5000.0 ms`).
- **Итоговая верификация (`P4-T10`, 18.11.2026)**:
  - Перепроверено в `P4-T10` после правки `crates/mcp-cad-physics/tests/physics_tests.rs`:
    - `cargo test -p mcp-cad-physics --no-fail-fast`: **42/42 PASS** (`10` `--lib`, `32` `physics_tests`, `0 failed`), включая стресс-прогон параллельно с `npm run build && npm test` на 4-vCPU (`rc_rust=0`, `rc_web=0`);
    - `cargo clippy --workspace --all-targets -- -D warnings`: **0 ошибок, 0 предупреждений**;
    - `npm run lint` (`tsc --noEmit`) & `npm run build` (`vite build`): **PASS** (`0` ошибок);
    - `npm test` (`src/cad/cad_engine.test.ts` + `src/cad/__tests__/analyze_mechanism.test.ts`): **53/53 PASS** (`38/38` + `15/15`, `0 failed`).
  - Полный прогон Rust Workspace (подтверждён в `P4-T9`): **348/348 PASS** (`0 failed`):
    - `mcp-cad-core`: **133/133 PASS** (`76` `--lib`, `10` `csg_invariants`, `16` `dag_tests`, `25` `primitive_invariants`, `6` `tessellation_tests`);
    - `mcp-cad-fea`: **21/21 PASS** (`8` `--lib`, `2` `fea_benchmarks`, `11` `fea_tests`);
    - `mcp-cad-physics`: **42/42 PASS** (`10` `--lib`, `32` `physics_tests`);
    - `mcp-cad-viewer`: **10/10 PASS** (`10` `viewer_tests`);
    - `mcp-cad-server`: **142/142 PASS** (`65` `--lib`, `1` `bench_suite`, `2` `gear_kinematic_test`, `61` `mcp_contract_tests`, `13` `qa_wide_scan`).
  - `cargo tree | grep -i -E 'rapier|parry'` и `grep -rnI -i 'rapier\|parry' crates src`: **0 совпадений**.
