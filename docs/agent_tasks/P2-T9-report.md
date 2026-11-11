# Отчёт о выполнении задачи P2-T9: Слой требований (`set_requirements` / `check_requirements`)

## 1. Что сделано (список изменённых и созданных файлов с кратким объяснением)

1. **`crates/mcp-cad-core/src/part.rs`**:
   - В структуры `Part` и `PartSummary` добавлено поле `#[serde(default)] pub revision: u64`, инициализируемое нулём в `Part::new`.
   - Добавлен метод `Part::bump_revision(&mut self)` и автоматический инкремент `revision` при каждом вызове `Part::rebuild()` и `Part::tessellate()`.
2. **`crates/mcp-cad-server/src/tools/requirements.rs`** *(новый модуль)*:
   - Реализованы структуры `RequirementStatus` (`PASS`, `FAIL`, `STALE`, `NOT_AVAILABLE`), `RequirementKind` (`bbox_max`, `bbox_min`, `mass_max`, `mesh_valid`, `min_safety_factor`, `no_interference`, `mechanism_dof`), `RequirementSpec` и `RequirementsState`.
   - Реализован обработчик `set_requirements(args, state)` с привязкой к конкретной детали (`part_id`) или ко всей сцене (`scope: "scene"`), конвертацией линейных размеров `bbox_max` / `bbox_min` во внутренние метры СИ согласно активному `length_unit` (`[DECISION-UNITS-1]`) и управлением флагом `auto_check`.
   - Реализован обработчик `check_requirements(args, state)` и `evaluate_requirements_report`, возвращающие по каждому требованию `{id, type, status, measured, target, margin, hint}` и агрегированную сводку `{total, passed, failed, stale, not_available}`.
   - Для `min_safety_factor` реализовано сопоставление `part.revision` с ревизией последнего МКЭ-расчёта (`state.last_fea_revisions` или `state.fea_case_reports`): при отсутствии отчёта или при изменении геометрии детали возвращается статус `"STALE"` с подсказкой вызвать `run_fea_analysis`.
   - Для `no_interference` и `mechanism_dof` возвращается статус `"NOT_AVAILABLE"` с пояснением о предстоящих задачах `P3-T10` и `P3-T9`.
   - Реализована функция `attach_auto_check_if_enabled`, добавляющая `requirements_status` и `failed_requirements` в JSON-ответы всех мутирующих CAD-инструментов при `auto_check: true`.
   - Добавлены 6 модульных unit-тестов (`test_bbox_and_mass_requirements_pass_and_fail`, `test_mesh_valid_and_expected_genus`, `test_fea_safety_factor_stale_after_part_revision_bump`, `test_not_available_requirements_no_interference_and_mechanism_dof`, `test_auto_check_attaches_requirements_status_and_failed_list`, `test_scene_scope_bbox_and_mass`).
3. **`crates/mcp-cad-server/src/state.rs`**:
   - В `EngineState` добавлены поля `last_fea_revisions: HashMap<String, u64>`, `fea_case_reports: HashMap<(String, String), (FeaReport, u64)>` и `requirements: RequirementsState`.
4. **`crates/mcp-cad-server/src/tools/fea.rs`**:
   - В `run_fea_analysis` сохраняется `part.revision` в `state.last_fea_revisions` и опциональный именованный случай расчёта `fea_case` в `state.fea_case_reports`; в JSON-ответ добавлены `part_revision` и `fea_case`.
5. **`crates/mcp-cad-server/src/tools/cad.rs`**:
   - В `part_summary_in_unit` и `query_part_geometry` добавлено поле `"revision"`.
   - В `delete_part` добавлена очистка `last_fea_revisions`, `fea_case_reports` и `requirements` для удаляемой детали.
   - В `batch_cad_operations` добавлено сохранение и откат `last_fea_revisions`, `fea_case_reports` и `requirements` при сбое любого шага батча.
   - Во все мутирующие инструменты (`add_feature_primitive`, `cut_hole`, `modify_feature`, `cut_helical_groove`, `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part`, `batch_cad_operations`, `optimize_feature_tree`, `import_stl`) интегрирован вызов `attach_auto_check_if_enabled` и инкремент `revision` для прямых мутаций сетки.
6. **`crates/mcp-cad-server/src/tools/project.rs`**:
   - В `ProjectPart` добавлено поле `#[serde(default)] pub revision: u64`, а в `ProjectFile` — `#[serde(default, skip_serializing_if = "Option::is_none")] pub requirements: Option<RequirementsState>`.
   - В `save_project` и `load_project` обеспечено сохранение и восстановление `part.revision` и `state.requirements`.
7. **`crates/mcp-cad-server/src/tools/mod.rs` и `crates/mcp-cad-server/src/error.rs`**:
   - Зарегистрированы инструменты `#32 set_requirements` и `#33 check_requirements` в `list_tools()` и `dispatch_tool()`.
   - В `map_invalid_argument_error` (`error.rs`) добавлена структурированная обработка `Unknown requirement type` со списком `supported_requirement_types`.
8. **`crates/mcp-cad-server/tests/mcp_contract_tests.rs` и `docs/MCP_API_REFERENCE.md`**:
   - Добавлен сквозной контрактный тест `test_mcp_p2_t9_requirements_and_auto_check` и документация по инструментам `set_requirements` и `check_requirements`.
9. **`package.json`**:
   - Исправлена несуществующая версия `"@types/node": "^26.6.6"` на `"@types/node": "^22.10.2"`, блокировавшая `bun install`.

## 2. Результаты проверок

- **`npm run lint` (`tsc --noEmit`) и сборка веб-приложения (`compile_applet` / `lint_applet`)**: проходят без ошибок (`0 errors`).
- **Модульные и контрактные тесты `P2-T9`**:
  - `requirements::tests::*` (6 unit-тестов в `crates/mcp-cad-server/src/tools/requirements.rs`): проверка `bbox_max`, `bbox_min`, `mass_max`, `mesh_valid` (с `expected_genus`), перехода `min_safety_factor` в `STALE` после `bump_revision()`, статусов `NOT_AVAILABLE` для `no_interference` и `mechanism_dof`, а также `attach_auto_check_if_enabled`.
  - `test_mcp_p2_t9_requirements_and_auto_check` в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`: сквозная проверка жизненного цикла требований, пересчёта единиц `mm` ↔ `m`, `STALE` → `PASS` → `STALE` при `run_fea_analysis` и `modify_feature`, автопроверки `auto_check: true` и сохранения/загрузки через `save_project` / `load_project`.

## 3. Новые находки (не тронутые в рамках этой задачи)

- В `package.json:22` была указана несуществующая в реестре npm версия `"@types/node": "^26.6.6"`, вызывавшая ошибку `bun install failed: exit status 1` при инициализации контейнера. Исправлено на `"^22.10.2"`.

## 4. Открытые вопросы `[DECISION]`

- Открытых архитектурных вопросов `[DECISION]` нет. Типы требований `no_interference` и `mechanism_dof` возвращают `status: "NOT_AVAILABLE"` с явной ссылкой на задачи `P3-T10` и `P3-T9` согласно спецификации `fix/P2_agent_contract.md`.

## 5. Готовность к следующей задаче

- Задача **P2-T9** выполнена; разблокирована зависимая задача **P2-T10** («Документация и сводка сцены для агента (`get_scene_summary`)»).
