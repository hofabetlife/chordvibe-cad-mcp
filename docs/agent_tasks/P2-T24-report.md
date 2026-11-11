# Отчет по задаче P2-T24: Предотвращение ложного устаревания (STALE) результатов FEA (part.revision) при вызове export_stl и tessellate_part

**Статус:** ✅ ВЫПОЛНЕНО  
**Ветка:** `agent/P2-T24`  
**Дата:** 09.11.2026  
**Компоненты:** `crates/mcp-cad-core/src/part.rs`, `crates/mcp-cad-core/tests/tessellation_tests.rs`, `crates/mcp-cad-server/src/tools/cad.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`

---

## 1. Контекст и выявленная проблема

В ходе выполнения задачи **P2-T19** ([`docs/agent_tasks/P2-T19-report.md`](P2-T19-report.md)) был обнаружен дефект семантики счётчика ревизий детали `part.revision`:

- В `Part::tessellate` (`crates/mcp-cad-core/src/part.rs:73`) при перевычислении кэшированной сетки `cached_mesh` с новыми настройками тесселяции `TessellationSettings` выполнялся инкремент `self.revision = self.revision.saturating_add(1)`.
- Поскольку инструмент `export_stl` (`crates/mcp-cad-server/src/tools/cad.rs:2150`) по умолчанию запрашивает качество тесселяции `"fine"`, а `Part::rebuild()` использует `"medium"`, любой вызов `export_stl` или `tessellate_part` после проведения МКЭ-расчёта `run_fea_analysis` увеличивал `part.revision`.
- В результате проверка инженерных требований `check_requirements` (`crates/mcp-cad-server/src/tools/requirements.rs:838`, требование `min_safety_factor`) и сводка сцены `get_scene_summary` (`crates/mcp-cad-server/src/tools/metrics.rs:146`, поле `last_fea.stale`) сравнивали сохранённую ревизию расчёта `fea_rev` с увеличенным `part.revision` и ложно переводили статус расчёта прочности в **`STALE`**, хотя параметрический граф признаков (`part.dag`) и конструктивная геометрия детали не изменялись.
- Аналогичный побочный эффект возникал при вызове `TetrahedralMesh::from_part_adaptive` (`crates/mcp-cad-fea/src/tetra_mesh.rs:64`) и `OffscreenRenderer::render_part_png` (`crates/mcp-cad-viewer/src/offscreen.rs:344`), если бы они вызывались на мутабельном экземпляре `Part`.

---

## 2. Что было сделано

### 2.1. Геометрическое ядро (`crates/mcp-cad-core/src/part.rs`)
- Из метода `Part::tessellate(&mut self, settings: TessellationSettings)` удалён инкремент `self.revision = self.revision.saturating_add(1)`.
- Теперь `Part::revision` монотонно увеличивается **только** при фактическом изменении параметрической геометрии детали:
  - в `Part::rebuild()` (вызываемом после добавления, модификации или удаления узлов `FeatureTree`, булевых операций, трансформаций и оптимизации DAG);
  - в `Part::bump_revision()` (вызываемом при прямом импорте STL в `import_stl`).
- При смене качества тесселяции (`coarse`, `medium`, `fine`, `custom`) в `Part::tessellate` обновляются `self.mesh`, `self.cached_mesh` и `self.cached_settings`, а `self.revision` остаётся неизменным.

### 2.2. Сервер и MCP-инструменты (`crates/mcp-cad-server/src/tools/cad.rs`)
- В `resolve_tessellation_settings`:
  - Добавлена проверка конечности (`is_finite()`) пользовательских параметров тесселяции `chord_tolerance`, `angular_tolerance_deg` и `max_edge_length` (с возвратом `INVALID_ARGUMENT`).
- В `export_stl`:
  - Добавлен проброс `angular_tolerance_deg` и `max_edge_length` в `resolve_tessellation_settings` (наравне с `tessellate_part`).
  - Обеспечено транзакционное сохранение и откат `mesh`, `cached_mesh`, `cached_settings` и `revision` на случай ошибки записи файла или валидации пути.
  - В JSON-ответ `export_stl` добавлены поля `"part_id"` и `"revision"`, подтверждающие неизменность ревизии детали после экспорта.
- В `tessellate_part`:
  - Гарантировано сохранение `part.revision` при перестроении сетки с новыми `TessellationSettings`.
  - В JSON-ответ `tessellate_part` добавлено поле `"revision"`.

### 2.3. Документация (`docs/MCP_API_REFERENCE.md`)
- Обновлено описание инструментов `tessellate_part` и `export_stl`: зафиксировано наличие поля `revision` в ответе и инвариант сохранения актуальности (`FRESH` / `PASS`) результатов `run_fea_analysis` в `check_requirements` и `get_scene_summary` при ре-тесселяции и экспорте в STL.

---

## 3. Новые находки, заведённые в `TASKS.md`

В ходе аудита смежных инструментов отчётности FEA и оптимизации DAG выявлена и зарегистрирована задача **`P2-T27 [S]`**:
1. В `get_simulation_metrics_json` (`crates/mcp-cad-server/src/tools/metrics.rs:13-65`) единица длины берётся из `state.length_unit` напрямую вместо `state.resolve_length_unit(args)?`, а элементы массива `fea_reports` не содержат полей `fea_revision`, `part_revision` и `stale` (`fea_rev != part.revision`), в отличие от `get_scene_summary`.
2. В `optimize_feature_tree` (`crates/mcp-cad-server/src/tools/cad.rs:3239`) используется `state.length_unit` вместо `state.resolve_length_unit(args)?`, в JSON-ответе отсутствует `revision`, а в `Part::optimize_dag(true)` (`crates/mcp-cad-core/src/part.rs:115-131`) вызывается `self.rebuild()` (инкрементируя `part.revision` и переводя результаты FEA в `STALE`) даже тогда, когда `report.applied_rules.is_empty()` (ни одно правило оптимизации не сработало и DAG не изменился).

---

## 4. Верификация и тесты

1. **Unit-тесты `mcp-cad-core` (`crates/mcp-cad-core/src/part.rs`, `crates/mcp-cad-core/tests/tessellation_tests.rs`):**
   - Расширен `test_part_rebuild_and_mass` в `part.rs`: проверка `part.revision == 1` после `tessellate(Coarse)`, `tessellate(Fine)` и кастомных настроек.
   - Расширены `test_part_lazy_tessellation_and_caching` и `test_export_part_stl_with_quality` в `tessellation_tests.rs`: проверка неизменности `part.revision` при вызовах `tessellate`, `export_part_stl_with_quality`, `write_part_stl_with_quality` и увеличения `part.revision` строго при `part.rebuild()`.
2. **Unit-тесты `mcp-cad-server` (`crates/mcp-cad-server/src/tools/cad.rs`):**
   - Добавлен тест `test_p2_t24_export_stl_and_tessellate_part_preserve_fea_freshness`:
     - Создаёт деталь, задаёт требование `min_safety_factor: 1.5`, выполняет `run_fea_analysis` (`revision == 1`, `check_requirements == PASS`, `get_scene_summary.last_fea.stale == false`).
     - Вызывает `export_stl` (`quality: "fine"`) и `tessellate_part` (`quality: "coarse"` и `"custom"`), проверяя, что `part.revision` остаётся `1`, `check_requirements` возвращает `PASS` (`status == "PASS"`), а `get_scene_summary.last_fea.stale == false`.
     - Выполняет `cut_hole` и проверяет, что `part.revision` увеличивается до `2`, а `check_requirements` и `get_scene_summary` корректно переводят статус FEA в `STALE`.
3. **Контрактные MCP-тесты (`crates/mcp-cad-server/tests/mcp_contract_tests.rs`):**
   - Добавлен сквозной JSON-RPC тест `test_mcp_p2_t24_export_stl_and_tessellate_part_do_not_stale_fea`, проверяющий цепочку `create_part` → `add_feature_primitive` → `set_requirements` → `run_fea_analysis` → `export_stl` → `tessellate_part` → `check_requirements` (`PASS`) → `get_scene_summary` (`stale: false`) → `add_feature_primitive` (`revision: 2`, `STALE`).
4. **TypeScript / Frontend и сборка:**
   - `npx tsx --test src/cad/cad_engine.test.ts`: **29 passed, 0 failed (100% PASS)**.
   - `compile_applet` и `lint_applet`: **0 errors**.
