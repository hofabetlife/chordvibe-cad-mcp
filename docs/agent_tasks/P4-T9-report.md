# Отчёт о выполнении задачи P4-T9

- **Задача**: `P4-T9` — Синхронизация `Part::mesh()` (`crates/mcp-cad-core/src/part.rs:162-167`) при прямой мутации публичного поля `part.mesh` после `part.rebuild()` без вызова `invalidate_cache()`
- **Статус**: ✅ ВЫПОЛНЕНО (`18.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-core/src/part.rs`](../../crates/mcp-cad-core/src/part.rs) *(`Part::mesh(&self)` теперь напрямую возвращает каноническую сетку `&self.mesh`; `Part::mesh_mut(&mut self)` автоматически вызывает `self.invalidate_cache()`; `Part::tessellate(&mut self, settings)` проверяет совпадение `*cached == self.mesh` при попадании в кэш и возвращает `&self.mesh`; добавлены 2 unit-теста `P4-T9`)*
  - [`crates/mcp-cad-physics/src/rigid_body.rs`](../../crates/mcp-cad-physics/src/rigid_body.rs) *(упрощён `MassProperties::from_part` до прямого вызова `Self::from_mesh_and_density(part.mesh(), part.material.density_kg_m3)` без обходной проверки из `P4-T2`)*
  - [`TASKS.md`](../../TASKS.md), [`docs/PROGRESS.md`](../PROGRESS.md), [`CHANGELOG.md`](../../CHANGELOG.md), [`README.md`](../../README.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md) *(актуализация статуса `P4-T9`, счётчиков тестов и постановка новой задачи `P4-T10`)*

---

## 1. Причина и реализация исправления

В [`crates/mcp-cad-core/src/part.rs`](../../crates/mcp-cad-core/src/part.rs) структура `Part` содержит публичное поле `pub mesh: TriangleMesh`, а также кэш ленивой тесселяции `pub cached_mesh: Option<TriangleMesh>` и `pub cached_settings: Option<TessellationSettings>`. Оба метода построения (`Part::rebuild()` и `Part::tessellate()`) всегда записывают актуальную сетку в `self.mesh`, однако ранее `Part::mesh(&self)` возвращал `self.cached_mesh.as_ref().unwrap_or(&self.mesh)`.

Из-за этого при прямой мутации `part.mesh.vertices` после `part.rebuild()` (или `part.tessellate()`) без ручного вызова `part.invalidate_cache()` все методы чтения (`part.mesh()`, `part.volume()`, `part.mass()`, `part.mass_from_mm()`, `part.center_of_mass()`, `part.surface_area()`, `part.bounding_box()`, `part.inspect()`, `part.summary()`) продолжали читать устаревшую копию из `cached_mesh`. Это также требовало обходного кода в `MassProperties::from_part` (`crates/mcp-cad-physics/src/rigid_body.rs`).

Внесённые изменения:
1. **Единый источник истины `Part::mesh(&self)` ([`crates/mcp-cad-core/src/part.rs`](../../crates/mcp-cad-core/src/part.rs))**:
   - `Part::mesh(&self)` упрощён до прямого возврата `&self.mesh`. Поскольку `rebuild()` и `tessellate()` всегда обновляют `self.mesh`, а внешние мутации модифицируют именно `self.mesh`, все производные геометрические запросы автоматически видят актуальную сетку без ручного `invalidate_cache()`.
2. **Инвалидация кэша при `Part::mesh_mut(&mut self)`**:
   - `Part::mesh_mut(&mut self)` теперь вызывает `self.invalidate_cache()` перед возвратом `&mut self.mesh`, чтобы `cached_mesh`/`cached_settings` не оставались рассинхронизированными.
3. **Защита от устаревшего кэша в `Part::tessellate(&mut self, settings)`**:
   - Попадание в кэш теперь проверяет `s == settings && *cached == self.mesh` и возвращает `Ok(&self.mesh)` (сохраняя стабильность адреса `&self.mesh` и предотвращая возврат устаревшего `cached_mesh`, если `part.mesh` был изменён напрямую).
   - Публичные поля `pub cached_mesh` и `pub cached_settings` сохранены для обратной совместимости с тестами (`tessellation_tests.rs`, `fea_tests.rs`) и транзакционным откатом в `crates/mcp-cad-server/src/tools/cad.rs`.
4. **Очистка обходного кода в физике ([`crates/mcp-cad-physics/src/rigid_body.rs`](../../crates/mcp-cad-physics/src/rigid_body.rs))**:
   - `MassProperties::from_part` теперь вызывает `Self::from_mesh_and_density(part.mesh(), part.material.density_kg_m3)` напрямую.

---

## 2. Новые тесты, верификация и обнаруженная проблема (`P4-T10`)

В `crates/mcp-cad-core/src/part.rs` (`mod tests`) добавлены 2 unit-теста:
1. `test_p4_t9_direct_mesh_mutation_after_rebuild_updates_part_queries`: после `part.rebuild()` (при заполненном `part.cached_mesh`) напрямую модифицирует `part.mesh.vertices` без вызова `invalidate_cache()` и проверяет немедленное обновление `part.mesh()`, `part.center_of_mass()`, `part.bounding_box()`, `part.volume()` и `part.summary()`.
2. `test_p4_t9_mesh_mut_invalidates_cache_and_tessellate_detects_direct_mutation`: проверяет, что `part.tessellate(&coarse)` после прямой мутации `part.mesh` не отдаёт устаревший `cached_mesh`, а `part.mesh_mut()` сбрасывает `cached_mesh` и `cached_settings`.

Результаты проверок после финальных правок кода:
- `cargo test -p mcp-cad-core --no-fail-fast`: **133/133 PASS** (`--lib`: **76/76** включая 2 новых теста `P4-T9`, `csg_invariants`: **10/10**, `dag_tests`: **16/16**, `primitive_invariants`: **25/25**, `tessellation_tests`: **6/6**, `0 failed`).
- `cargo test -p mcp-cad-physics --no-fail-fast`: **42/42 PASS** (`--lib`: **10/10**, `physics_tests`: **32/32**, `0 failed`).
- `cargo test -p mcp-cad-fea --no-fail-fast`: **21/21 PASS** (`--lib`: **8/8**, `fea_benchmarks`: **2/2**, `fea_tests`: **11/11**, `0 failed`).
- `cargo test -p mcp-cad-viewer --no-fail-fast`: **10/10 PASS** (`viewer_tests`: **10/10**, `0 failed`).
- `cargo test -p mcp-cad-server --no-fail-fast`: **142/142 PASS** (`--lib`: **65/65**, `bench_suite`: **1/1**, `gear_kinematic_test`: **2/2**, `mcp_contract_tests`: **61/61**, `qa_wide_scan`: **13/13**, `0 failed`).
- Итого по Rust Workspace: **348/348 PASS** (`0 failed`).
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
- `npm run lint` (`tsc --noEmit`) и `npm run build` (`vite build`): **PASS** (`exit 0`).
- `npm test` (`src/cad/cad_engine.test.ts` + `src/cad/__tests__/analyze_mechanism.test.ts`): **53/53 PASS** (`38/38` + `15/15`, `0 failed`).

### Обнаруженная при проверке проблема (заведена в `TASKS.md` как `P4-T10`)
- При параллельном запуске `cargo test -p mcp-cad-physics --no-fail-fast` одновременно с `npm run build && npm test` на 4-vCPU контейнере тест `test_p3_t10_benchmark_check_interference_10_parts_45_pairs` (`crates/mcp-cad-physics/tests/physics_tests.rs:1687`) упал из-за конкуренции за CPU по жёсткому wall-clock порогу `report.elapsed_ms < 250.0` (`took 292.252 ms (expected < 250 ms)`). При изолированном прогоне тест стабильно проходит (`42/42 PASS`). Задача стабилизации тайминга/критерия теста оформлена в `TASKS.md` как **`P4-T10`**.
