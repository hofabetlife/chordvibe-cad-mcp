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

## План следующих шагов
- **Фаза 2**: Единый слой команд в Rust, вынос нативного окна `winit` под опциональный feature-флаг `native-gui` (по умолчанию выключен), единый реестр 28 инструментов MCP как источник правды.
- **Фаза 3**: Веб-транспорт и статика/API на порту 3000 (`0.0.0.0` с обязательным Bearer-токеном для не-loopback адреса и всех мутирующих запросов), песочница путей `./output` для STL, `TET10` в Rust МКЭ, тонкий клиент на TypeScript без дублирования вычислений.
- **Фаза 4**: Разработка и бенчмарк собственного физического движка механизмов (DAE / обобщённый $\alpha$-метод / стабилизированный индекс 2, прямая и обратная динамика, анализ рангов якобиана и сингулярностей, регуляризованный контакт с трением), перекрёстная проверка с аналитикой и `MuJoCo 3.17.0`, полное удаление `rapier3d`.
- **Фаза 5**: Сквозная проверка в среде (порт 3000, MCP по `stdio` и HTTP/SSE с токеном, проверка через headless Chromium / `playwright-cli`, кросс-проверка `cargo check --target x86_64-pc-windows-gnu`).
- **Фаза 6**: Актуализация документации (`README.md`, `CHANGELOG.md`, `PRD.md`, `docs/`), добавление `LICENSE-MIT`, `LICENSE-APACHE` и `.github/workflows/ci.yml`.
