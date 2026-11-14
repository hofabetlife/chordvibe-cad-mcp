# Отчёт о выполнении задачи P3-T13

**Задача:** `P3-T13 [S]` — Устранение предупреждений `clippy` на компиляторе `rustc 1.91.1` в `mcp-cad-core` и `mcp-cad-server`  
**Статус:** ✅ DONE (дефект не воспроизводится в исходном коде — все 4 пункта уже были исправлены ранее; актуализированы `TASKS.md`, `PROJECT_CONTEXT.md` и аудит тестов/линтера)  
**Дата:** 14.11.2026  
**Затронутые файлы:**
- `docs/agent_tasks/P3-T13-report.md` *(новый)*
- `TASKS.md`
- `PROJECT_CONTEXT.md`

---

## 1. Что сделано (проверка воспроизводимости и обновление документации)

В соответствии с правилом 1 (`fix/00_README_PREAMBLE.md`) выполнена попытка воспроизведения всех 4 пунктов `P3-T13` на компиляторе `rustc 1.91.1 (ed61e7d7e 2025-11-07)` / `cargo 1.91.1` / `clippy 0.1.91`. Правки в Rust-код не вносились, так как все заявленные предупреждения уже устранены в текущей кодовой базе, а `#![allow(...)]` на уровне крейтов не используются (`crates/mcp-cad-core/src/lib.rs`, `crates/mcp-cad-server/src/{lib.rs, main.rs}`):

1. **`crates/mcp-cad-core/src/csg.rs:1201, 1216, 1326`**: псевдоним `type EarClippedTriangle = ([u32; 3], Vector3<f64>, u32);` уже выделен и используется в возвращаемых типах функций `ear_clip_planar_ring_maxmin` и `try_remove_vertex_ear_clip` (0 предупреждений `clippy::type_complexity`).
2. **`crates/mcp-cad-server/src/tools/metrics.rs:90-92`**: doc-комментарий функции `get_scene_summary` уже отформатирован с корректным 4-пробельным выравниванием продолжения списка `/// 34. ...` (0 предупреждений `clippy::doc_lazy_continuation`).
3. **`crates/mcp-cad-server/src/tools/cad.rs`**: 0 предупреждений `clippy::bool_assert_comparison` по всем таргетам крейта.
4. **`crates/mcp-cad-server/src/tools/physics.rs:1367`**: литерал `6.28` уже заменён на `std::f64::consts::TAU` (0 предупреждений `clippy::approx_constant`).

После подтверждения чистоты `cargo clippy --workspace --all-targets -- -D warnings` и полного прогона тестов workspace обновлены статус задачи `P3-T13`, сводка верификации и реестр новых находок (`P3-T19`..`P3-T22`) в `TASKS.md` и `PROJECT_CONTEXT.md`.

---

## 2. Как проверено (команды и итоговые строки `test result`)

- **Clippy по всему workspace (`rustc 1.91.1`):**
  ```text
  cargo clippy --workspace --all-targets -- -D warnings
  Finished `dev` profile [optimized + debuginfo] target(s) in 0.53s
  EXIT: 0 (0 ошибок, 0 предупреждений во всех 5 крейтах: mcp-cad-core, mcp-cad-fea, mcp-cad-physics, mcp-cad-viewer, mcp-cad-server)
  ```
- **Тесты Batch 1 (`WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-core -p mcp-cad-physics -p mcp-cad-viewer --no-fail-fast`):**
  - `mcp-cad-core --lib`: `test result: ok. 74 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 17.68s`
  - `mcp-cad-core --test csg_invariants`: `test result: ok. 10 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 27.19s`
  - `mcp-cad-core --test dag_tests`: `test result: ok. 14 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 7.88s`
  - `mcp-cad-core --test primitive_invariants`: `test result: ok. 25 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 7.90s`
  - `mcp-cad-core --test tessellation_tests`: `test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.05s`
  - `mcp-cad-physics --lib`: `test result: ok. 0 passed; 0 failed; 0 ignored`
  - `mcp-cad-physics --test physics_tests`: `test result: FAILED. 17 passed; 1 failed; 0 ignored; 0 measured; 0 filtered out; finished in 7.66s` (упал `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall`)
  - `mcp-cad-viewer --lib`: `test result: ok. 0 passed; 0 failed; 0 ignored`
  - `mcp-cad-viewer --test viewer_tests`: `test result: ok. 10 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 20.03s`
- **Тесты Batch 2 (`WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-fea -p mcp-cad-server --no-fail-fast`):**
  - `mcp-cad-fea --lib`: `test result: ok. 8 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s`
  - `mcp-cad-fea --test fea_benchmarks`: `test result: ok. 2 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 131.86s`
  - `mcp-cad-fea --test fea_tests`: `test result: ok. 11 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 83.10s`
  - `mcp-cad-server --lib`: `test result: ok. 59 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 41.55s`
  - `mcp-cad-server (bin)`: `test result: ok. 0 passed; 0 failed; 0 ignored`
  - `mcp-cad-server --test bench_suite`: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 10.42s`
  - `mcp-cad-server --test gear_kinematic_test`: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.46s`
  - `mcp-cad-server --test mcp_contract_tests`: `test result: FAILED. 54 passed; 3 failed; 0 ignored; 0 measured; 0 filtered out; finished in 22.64s` (упали `test_mcp_initialize`, `test_mcp_open_preview_window`, `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`)
  - `mcp-cad-server --test qa_wide_scan`: `test result: ok. 13 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 12.74s`
- **Спайки вне workspace:**
  - `cargo clippy --manifest-path spikes/fe_snap/Cargo.toml --all-targets -- -D warnings` — `EXIT: 0` (0 предупреждений).
  - `cargo clippy --manifest-path spikes/monstertruck_brep/Cargo.toml --all-targets -- -D warnings` — `EXIT: 101` (`error[E0063]` в `spikes/monstertruck_brep/src/main.rs:60:20`).
- **TypeScript:**
  - `npm run lint` (`tsc --noEmit`) — `0` ошибок.
  - `npx --yes tsx --test src/cad/__tests__/analyze_mechanism.test.ts` — `11 passed; 0 failed`.

---

## 3. Тесты до и после

| Проверка | До проверки `P3-T13` (исходное состояние копии) | После проверки `P3-T13` (правки только в docs) |
|---|---|---|
| `cargo clippy --workspace --all-targets -- -D warnings` (`rustc 1.91.1`) | 0 ошибок, 0 предупреждений (`EXIT: 0`) | 0 ошибок, 0 предупреждений (`EXIT: 0`) |
| `cargo test --workspace` (по крейтам) | 309 тестов: 305 passed, 4 failed (`mcp-cad-core`: 129/129, `mcp-cad-fea`: 21/21, `mcp-cad-viewer`: 10/10, `mcp-cad-physics`: 17/18, `mcp-cad-server`: 128/131) | 309 тестов: 305 passed, 4 failed (код крейтов не менялся) |
| `npm run lint` + `analyze_mechanism.test.ts` | 0 ошибок TS, 11/11 PASS | 0 ошибок TS, 11/11 PASS |

---

## 4. Что не удалось и остаточные риски

- **Коммит в Git не выполнен**: в рабочей директории `/app/applet` отсутствует каталог `.git` (`git status` завершается с `fatal: not a git repository (or any of the parent directories): .git`), поэтому создание коммита в существующей истории Git невозможно без внешней Git-инициализации / переноса патча в клон репозитория.
- Выявленные 4 падения тестов и расхождения в документации/спайках относятся к другим задачам и по правилам не правились в рамках `P3-T13`, а занесены в `TASKS.md`.

---

## 5. Побочные находки (не чинить, только перечислить — занесены в `TASKS.md`)

1. **`P3-T19`**: В `crates/mcp-cad-physics/tests/physics_tests.rs:1620:5` падает `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall` (`assertion left == right failed: left: Some(78), right: Some(27)`): после пролёта рычага мимо стены шаг 78 с обратной стороны даёт меньший положительный зазор, чем шаг 27 до столкновения.
2. **`P3-T20`** (и расширение `P3-T14`): В `crates/mcp-cad-server/tests/mcp_contract_tests.rs` кроме известного падения `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` (`P3-T14`, строка 4517) падают ещё два контрактных теста:
   - `test_mcp_initialize` (`mcp_contract_tests.rs:119:5`): не находит `docs/AGENT_GUIDE.md` и `get_scene_summary` в `initialize.instructions` после `P2-T39`;
   - `test_mcp_open_preview_window` (`mcp_contract_tests.rs:716:5`): ожидает `"status": "headless"` на пустой сцене, тогда как после `P2-T37` возвращается `"status": "empty"`.
3. **`P3-T21`**: Вне-workspace спайк `spikes/monstertruck_brep/src/main.rs:60:20` также не компилируется (`error[E0063]: missing fields face_ids and faces in initializer of mcp_cad_core::TriangleMesh`), аналогично `P3-T18` в `spikes/manifold_rust`.
4. **`P3-T22`**: В рабочей копии отсутствуют файлы `.github/workflows/ci.yml` и `rust-toolchain.toml` (заявленные выполненными в `P0-T8`), а также `src/cad/cad_engine.test.ts`, на которые ссылаются `TASKS.md`, `PROJECT_CONTEXT.md` и `README.md`.
5. Расхождение учётных счётчиков тестов в `TASKS.md:6-7` и `PROJECT_CONTEXT.md:71-78` (фактически в Rust workspace 309 тестов вместо заявленных 274: `core` 129, `fea` 21 вместо 25, `server` 131 вместо 116; в TS 11 тестов в `analyze_mechanism.test.ts`). Сводки в `TASKS.md` и `PROJECT_CONTEXT.md` актуализированы.
