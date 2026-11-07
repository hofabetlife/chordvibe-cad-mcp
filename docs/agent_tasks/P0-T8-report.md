# Отчёт по задаче P0-T8: CI и гигиена репозитория

- **ID задачи:** `P0-T8`
- **Статус:** `✅ DONE`
- **Затронутые файлы:**
  - `rust-toolchain.toml` *(новый)*
  - `Dockerfile`
  - `.github/workflows/ci.yml` *(новый)*
  - `README.md`
  - `qa/FINAL_REPORT_ROUND2.md`
  - `qa/BUGS.md`
  - `qa/COVERAGE.md`
  - `qa/LOG.md`
  - `qa/MESH_REPORT.md`
  - `qa/STATE.json`
  - `TASKS.md`

---

## 1. Что сделано

### 1.1. Фиксация версии инструментария Rust (`rust-toolchain.toml` и `Dockerfile`)
1. Создан файл [`rust-toolchain.toml`](../../rust-toolchain.toml) с каналом `1.91` и компонентами `["rustfmt", "clippy"]`, так как зависимости в `Cargo.lock` (`uuid 1.29`, `image 0.25.10`, `wayland-protocols 0.32.13`) требуют `rustc >= 1.89`.
2. В [`Dockerfile`](../../Dockerfile) базовый образ стадии сборки обновлён с устаревшего `FROM rust:1.82-bookworm AS builder` на `FROM rust:1.91-bookworm AS builder`. Системные пакеты для программного рендеринга Vulkan/Mesa Lavapipe (`libvulkan1`, `mesa-vulkan-drivers`, `libegl1`, `libgl1-mesa-dri`) и переменные окружения `WGPU_BACKEND=vulkan`, `LIBGL_ALWAYS_SOFTWARE=1` сохранены.

### 1.2. Конфигурация GitHub Actions CI (`.github/workflows/ci.yml`)
Создан workflow [`.github/workflows/ci.yml`](../../.github/workflows/ci.yml), запускающийся на `push` и `pull_request` с тремя задачами (jobs):
1. **`rust` (`Rust Build & Test (1.91)`)**:
   - Установка системных пакетов для headless-рендеринга `wgpu` (`build-essential`, `pkg-config`, `libvulkan1`, `mesa-vulkan-drivers`, `libegl1`, `libgl1-mesa-dri`), переменные окружения `WGPU_BACKEND=vulkan`, `LIBGL_ALWAYS_SOFTWARE=1`.
   - Установка `Rust 1.91` (`dtolnay/rust-toolchain`) и кэширование Cargo (`actions/cache@v4` по ключу `Cargo.lock`).
   - Выполнение `cargo build --workspace` и `cargo test --workspace --no-fail-fast`.
2. **`clippy` (`Rust Clippy Linter`)**:
   - Установка `Rust 1.91` с компонентом `clippy` и системных библиотек.
   - Запуск `cargo clippy --workspace --all-targets -- -D warnings`.
   - Временно снабжён `continue-on-error: true` с подробным комментарием о 20 текущих ошибках в `mcp-cad-core` (`lib`), чтобы не блокировать пайплайн до их устранения (вынесено в задачу **P0-T12**).
3. **`web` (`Web Studio Lint & Tests`)**:
   - Установка Node.js 22 (`actions/setup-node@v4` с кэшем `npm`).
   - Выполнение `npm ci`, `npm run lint` (`tsc --noEmit`) и `npx tsx --test src/cad/cad_engine.test.ts`.
   - Временно снабжён `continue-on-error: true` с комментарием о 25/26 проходящих тестах и 1 падающем (`AssertionError: 5 !== 4` в тесте №17 `FEA_QUICK_LOAD_PRESETS.length`, задача **TS-FIX-01**).

### 1.3. Гигиена отчётов `qa/` и актуализация `README.md`
1. В начало файлов [`qa/FINAL_REPORT_ROUND2.md`](../../qa/FINAL_REPORT_ROUND2.md), [`qa/BUGS.md`](../../qa/BUGS.md), [`qa/COVERAGE.md`](../../qa/COVERAGE.md), [`qa/LOG.md`](../../qa/LOG.md) и [`qa/MESH_REPORT.md`](../../qa/MESH_REPORT.md) добавлена плашка:
   > `⚠ Устарело: утверждения не подтверждаются CI; источник истины — результаты cargo test (см. TASKS.md и VERIFICATION_REPORT.md).`
2. В [`qa/STATE.json`](../../qa/STATE.json) добавлено верхнеуровневое поле `"_warning"` с тем же предупреждением.
3. В [`README.md`](../../README.md):
   - Обновлён бейдж версии Rust (`1.80+` $\to$ `1.91+`) и бейдж статуса тестов (`100% Passing (99/99)` $\to$ `Rust 149/154 (5 ignored) | TS 25/26` со ссылкой на `TASKS.md`).
   - В разделе «Системные требования» указано требование **Rust $\ge 1.91$** (`rust-toolchain.toml`) и переменные окружения `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1` для headless-режима на Linux.
   - В разделе «Тестирование и верификация» (~строка 300) исправлены устаревшие утверждения о «18 тестах контракта, 100% Passing» и «5% точности балки», приведена фактическая статистика по `cargo test --workspace --no-fail-fast`, `cargo clippy --workspace --all-targets -- -D warnings` и `npx tsx --test src/cad/cad_engine.test.ts`, а также указана требуемая версия **Rust $\ge 1.91$**.

---

## 2. Как проверено (выполнение команд workflow локально)

Все проверки выполнены на `rustc 1.91.1 (ed61e7d7e 2025-11-07)` / `cargo 1.91.1` / `clippy 0.1.91` с переменными `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1`. Исходный код крейтов и тесты не изменялись.

### 2.1. Валидация синтаксиса конфигурационных файлов
- **YAML (`.github/workflows/ci.yml`)**:
  ```text
  $ python3 -c 'import yaml; wf = yaml.safe_load(open(".github/workflows/ci.yml")); print("YAML valid! Jobs:", list(wf["jobs"].keys()))'
  YAML valid! Jobs: ['rust', 'clippy', 'web']
  ```
- **TOML (`rust-toolchain.toml`) и JSON (`qa/STATE.json`)**:
  ```text
  qa/STATE.json valid JSON, _warning: ⚠ Устарело: утверждения не подтверждаются CI; источник истины — результаты `cargo test` (см. TASKS.md и VERIFICATION_REPORT.md).
  rust-toolchain.toml valid TOML: {'toolchain': {'channel': '1.91', 'components': ['rustfmt', 'clippy']}}
  ```

### 2.2. Job `rust`: `cargo build --workspace` и `cargo test --workspace --no-fail-fast`
- `cargo build --workspace` — **PASS** (`Finished dev profile [unoptimized + debuginfo] target(s)`).
- `cargo test --workspace --no-fail-fast` — **154 теста: 149 passed, 0 failed, 5 ignored**:
  - `mcp-cad-core --lib`: `test result: ok. 49 passed; 0 failed; 0 ignored; finished in 11.64s`
  - `mcp-cad-core --test csg_invariants`: `test result: ok. 5 passed; 0 failed; 0 ignored; finished in 14.30s`
  - `mcp-cad-core --test dag_tests`: `test result: ok. 14 passed; 0 failed; 0 ignored; finished in 12.29s`
  - `mcp-cad-core --test primitive_invariants`: `test result: ok. 13 passed; 0 failed; 5 ignored; finished in 64.07s`
  - `mcp-cad-core --test tessellation_tests`: `test result: ok. 6 passed; 0 failed; 0 ignored; finished in 0.07s`
  - `mcp-cad-fea --lib`: `test result: ok. 5 passed; 0 failed; 0 ignored`
  - `mcp-cad-fea --test fea_tests`: `test result: ok. 6 passed; 0 failed; 0 ignored; finished in 3.93s`
  - `mcp-cad-physics --test physics_tests`: `test result: ok. 6 passed; 0 failed; 0 ignored; finished in 0.88s`
  - `mcp-cad-server --test gear_kinematic_test`: `test result: ok. 1 passed; 0 failed; 0 ignored; finished in 1.65s`
  - `mcp-cad-server --test mcp_contract_tests`: `test result: ok. 21 passed; 0 failed; 0 ignored; finished in 10.73s`
  - `mcp-cad-server --test qa_wide_scan`: `test result: ok. 13 passed; 0 failed; 0 ignored; finished in 57.30s`
  - `mcp-cad-viewer --test viewer_tests`: `test result: ok. 10 passed; 0 failed; 0 ignored; finished in 12.09s`

### 2.3. Job `clippy`: `cargo clippy --workspace --all-targets -- -D warnings`
- Команда запускается и ожидаемо завершается с кодом ошибки на `mcp-cad-core (lib)` (**20 ошибок** при `-D warnings`):
  1. `crates/mcp-cad-core/src/primitives.rs:270:23` — `manual implementation of .is_multiple_of()`
  2. `crates/mcp-cad-core/src/primitives.rs:944:13` — `variable does not need to be mutable`
  3. `crates/mcp-cad-core/src/primitives.rs:1227:5` — `this function has too many arguments (8/7)` (`emit_polygon_cap`)
  4. `crates/mcp-cad-core/src/primitives.rs:2667:30` — `manually reimplementing div_ceil`
  5. `crates/mcp-cad-core/src/primitives.rs:2671:25` — `manually reimplementing div_ceil`
  6. `crates/mcp-cad-core/src/query_optimizer.rs:289:17` — `value assigned to node_cost_ms is never read`
  7. `crates/mcp-cad-core/src/query_optimizer.rs:1217:9` — `unused variable: genus_after`
  8. `crates/mcp-cad-core/src/csg.rs:851:8` — `manual !RangeInclusive::contains implementation`
  9. `crates/mcp-cad-core/src/csg.rs:1023:8` — `manual !RangeInclusive::contains implementation`
  10. `crates/mcp-cad-core/src/csg.rs:1141:1` — `this function has too many arguments (8/7)` (`ear_clip_planar_ring_maxmin`)
  11. `crates/mcp-cad-core/src/csg.rs:1190:22` — `the loop variable j is only used to index cur`
  12. `crates/mcp-cad-core/src/csg.rs:1221:16` — `this map_or can be simplified` (`is_none_or`)
  13. `crates/mcp-cad-core/src/csg.rs:1248:1` — `this function has too many arguments (11/7)` (`try_remove_vertex_ear_clip`)
  14. `crates/mcp-cad-core/src/csg.rs:1785:20` — `this map_or can be simplified`
  15. `crates/mcp-cad-core/src/csg.rs:1860:13` — `this if statement can be collapsed`
  16. `crates/mcp-cad-core/src/csg.rs:1861:20` — `this map_or can be simplified`
  17. `crates/mcp-cad-core/src/csg.rs:1954:28` — `manual !RangeInclusive::contains implementation`
  18. `crates/mcp-cad-core/src/csg.rs:2038:36` — `this map_or can be simplified`
  19. `crates/mcp-cad-core/src/csg.rs:2127:17` — `this if statement can be collapsed`
  20. `crates/mcp-cad-core/src/csg.rs:2147:24` — `this map_or can be simplified`

### 2.4. Job `web`: `npm run lint` и `npx tsx --test src/cad/cad_engine.test.ts`
- `npm run lint` (`tsc --noEmit`) — **PASS** (`0 errors`).
- `npx tsx --test src/cad/cad_engine.test.ts` — **26 tests: 25 pass, 1 fail** (`AssertionError: 5 !== 4` в тесте №17 `FEA_QUICK_LOAD_PRESETS.length`; контрактный тест №24 `verifies BUG-021: 28 MCP tools documentation & contract parity across README...` проходит после обновления `README.md`).

---

## 3. Тесты до и после

| Набор тестов / Проверка | До P0-T8 | После P0-T8 |
|---|---|---|
| `cargo build --workspace` | PASS | PASS |
| `cargo test --workspace --no-fail-fast` | 149 passed, 0 failed, 5 ignored | 149 passed, 0 failed, 5 ignored |
| `cargo clippy --workspace --all-targets -- -D warnings` | 20 errors (`mcp-cad-core`) | 20 errors (отслеживается в CI с `continue-on-error: true` и в `P0-T12`) |
| `npm run lint` (`tsc --noEmit`) | PASS (0 errors) | PASS (0 errors) |
| `npx tsx --test src/cad/cad_engine.test.ts` | 25 pass, 1 fail | 25 pass, 1 fail (Test 24 по `README.md` PASS) |

---

## 4. Что нельзя проверить локально и остаточные риски

1. **Исполнение облачных раннеров GitHub Actions (`actions/checkout@v4`, `dtolnay/rust-toolchain@master`, `actions/cache@v4`, `actions/setup-node@v4`)**:
   - Невозможно запустить сам облачный оркестратор GitHub Actions локально в контейнере без внешнего ранера GitHub, однако синтаксис `.github/workflows/ci.yml` проверен парсером `PyYAML` (`yaml.safe_load`), а каждая shell-команда всех трёх job-ов (`rust`, `clippy`, `web`) выполнена локально на том же наборе пакетов Ubuntu/Debian и Rust 1.91.
2. **Сборка Docker-образа (`docker build -t mcp-cad-server .`)**:
   - В текущем контейнере отсутствует демон Docker (`dockerd`), поэтому `Dockerfile` проверен статически и путём локального выполнения эквивалентной команды `cargo build --workspace` на `rustc 1.91.1` с тем же набором системных библиотек (`libvulkan1`, `mesa-vulkan-drivers`, `libegl1`, `libgl1-mesa-dri`).

---

## 5. Побочные находки (заведены в `TASKS.md`)

- **P0-T12**: Устранение 20 предупреждений/ошибок `cargo clippy --workspace --all-targets -- -D warnings` (в `crates/mcp-cad-core/src/primitives.rs`, `query_optimizer.rs`, `csg.rs` и тестовых таргетах) и снятие `continue-on-error: true` с job `clippy` в `.github/workflows/ci.yml`.
