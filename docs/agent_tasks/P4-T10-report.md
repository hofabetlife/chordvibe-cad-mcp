# Отчёт о выполнении задачи P4-T10

- **Задача**: `P4-T10` — Стабилизация wall-clock порога `< 250.0 ms` в тесте `test_p3_t10_benchmark_check_interference_10_parts_45_pairs` (`crates/mcp-cad-physics/tests/physics_tests.rs:1687`) при параллельной нагрузке на CPU
- **Статус**: ✅ ВЫПОЛНЕНО (`18.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs) *(стабилизирован `test_p3_t10_benchmark_check_interference_10_parts_45_pairs`: вместо хрупкого wall-clock ассерта `< 250.0 ms` введены детерминированные структурные, алгоритмические и аналитические проверки `broadphase_overlap_pairs_count == 1`, `max_intersection_volume ≈ 3.0e-5 м³`, `min_clearance ≈ 0.05 м`, `elapsed_ms.is_finite()`, обёртка `std::hint::black_box` и защитный предел от зависания `< 5000.0 ms`)*
  - [`TASKS.md`](../../TASKS.md), [`docs/PROGRESS.md`](../PROGRESS.md), [`CHANGELOG.md`](../../CHANGELOG.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md) *(актуализация статуса `P4-T10` и результатов верификации)*

---

## 1. Причина и реализация исправления

При параллельном выполнении `cargo test -p mcp-cad-physics --no-fail-fast` одновременно со сборкой и тестами Node/TypeScript на 4-vCPU окружении интеграционный тест `test_p3_t10_benchmark_check_interference_10_parts_45_pairs` (`crates/mcp-cad-physics/tests/physics_tests.rs:1687`) мог падать по жёсткому wall-clock порогу `report.elapsed_ms < 250.0` (`292.252 ms`), хотя сам алгоритм работал корректно и при изолированном прогоне завершался значительно быстрее.

Аудит всех крейтов рабочего пространства (`grep -rnE "elapsed_ms\s*<|elapsed_us\s*<|as_millis\(\)\s*<" crates/`) показал, что `physics_tests.rs:1688` был **единственным** жёстким wall-clock ассертом во всех Rust-тестах (аналогичный TS-тест в `src/cad/__tests__/analyze_mechanism.test.ts:307` лишь логирует `elapsedMs` без хрупкого порога).

Внесённые изменения в [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs) (`test_p3_t10_benchmark_check_interference_10_parts_45_pairs`):
1. Вызов `world.check_interference(None, false, None)` обёрнут в `std::hint::black_box(...)`.
2. Усилены детерминированные структурные и аналитические инварианты алгоритма 10 деталей / 45 пар:
   - `report.status == "FAIL"`, `candidate_pairs_count == 45`, `skipped_adjacent_pairs_count == 0`, `evaluated_pairs_count == 45`, `interfering_pairs_count == 1`, `pairs.len() == 45`;
   - **эффективность broad-phase AABB**: `report.broadphase_overlap_pairs_count == 1` (подтверждает, что все 44 непересекающиеся пары отсечены до дорогостоящего CSG narrow-phase и лишь одна пара `(part_00, part_01)` прошла в CSG);
   - **аналитический объём пересечения**: `(report.max_intersection_volume - 3.0e-5).abs() < 1e-9` ($0.02 \times 0.03 \times 0.05 = 3.0 \times 10^{-5}\text{ м}^3$);
   - **аналитический минимальный зазор**: `(min_clr - 0.05).abs() < 1e-9` ($0.05\text{ м}$ между ближайшими непересекающимися кубами).
3. Вместо хрупкого порога `< 250.0 ms` проверяется валидность метрики `report.elapsed_ms.is_finite() && report.elapsed_ms >= 0.0` и широкий защитный предел от катастрофического зависания `report.elapsed_ms < 5_000.0`, устойчивый к параллельной нагрузке на CPU в CI.

---

## 2. Верификация

Проверки после финальной правки кода:
- `cargo test -p mcp-cad-physics --no-fail-fast`: **42/42 PASS** (`--lib`: **10/10**, `physics_tests`: **32/32**, `0 failed`).
- Стресс-проверка под параллельной нагрузкой на CPU (`cargo test -p mcp-cad-physics --no-fail-fast` одновременно с `npm run build && npm test` на 4-vCPU контейнере): **PASS** (`rc_rust=0`, `rc_web=0`, **42/42** Rust physics + **53/53** TS).
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
- `npm run lint` (`tsc --noEmit`), `npm run build` (`vite build`) и `npm test` (`src/cad/cad_engine.test.ts` + `src/cad/__tests__/analyze_mechanism.test.ts`): **PASS** (**53/53**, включая контракт `BUG-021`).

Новых проблем в ходе выполнения задачи не выявлено.
