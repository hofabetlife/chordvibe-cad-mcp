# Отчёт о выполнении задачи P2-T10: Документация и сводка сцены для агента (`get_scene_summary`)

**Дата:** 06.11.2026  
**Статус:** ✅ ВЫПОЛНЕНО  
**Компоненты:** `docs/AGENT_GUIDE.md`, `crates/mcp-cad-server/src/{protocol.rs, tools/mod.rs, tools/metrics.rs, tools/cad.rs, tools/fea.rs, tools/requirements.rs}`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`, `docs/MCP_API_REFERENCE.md`, `TASKS.md`

---

## 1. Что сделано

1. **Создан краткий инженерный гайд `docs/AGENT_GUIDE.md` ($\le 2$ страниц, 87 строк)**:
   - Описаны 7 базовых принципов работы ИИ-агента с MCP-CAD:
     1. Разведка сцены одним вызовом (`get_scene_summary`);
     2. Контракт единиц измерения (`[DECISION-UNITS-1]`: `length_unit = "mm"` по умолчанию, переключение через `set_units`, сила в `N`, напряжение в `MPa`, модуль Юнга в `GPa`, масса в `kg`, время в `s`, углы в `deg`);
     3. Декларация инженерных требований (`set_requirements` с `auto_check: true`);
     4. Транзакционная сборка геометрии через `batch_cad_operations` с автоматическим откатом при ошибке;
     5. Обязательный гейт `validate_mesh` после любых CSG-операций и вырезов (`cut_hole`, `cut_helical_groove`, `boolean_*`);
     6. Расчёт прочности МКЭ (`run_fea_analysis`) строго по именованным граням (`query_part_geometry`) и отслеживание устаревания (`STALE`) при изменении `part.revision`;
     7. Чтение структурированных ошибок `ErrorEnvelope` (`code`, `param`, `hint`, `retryable`, `details`) при `result.isError == true`.
   - Приведён стандартный 5-шаговый инженерный цикл **«Build → Validate → Analyze → Iterate»** и 3 коротких примера JSON-RPC вызовов инструментов.

2. **Обновлено поле `initialize.instructions` в `crates/mcp-cad-server/src/protocol.rs`**:
   - Введена константа `pub const AGENT_INSTRUCTIONS: &str` (~1480 символов), возвращаемая в `result.instructions` при вызове JSON-RPC метода `initialize`.
   - Текст содержит выжимку 7 правил контракта агента, стандартный 5-шаговый цикл и прямую ссылку на `docs/AGENT_GUIDE.md` и `docs/MCP_API_REFERENCE.md`.

3. **Нормализованы описания и JSON-схемы всех 34 MCP-инструментов в `crates/mcp-cad-server/src/tools/mod.rs`**:
   - В описание (`description`) каждого из 34 инструментов (включая все 28 базовых инструментов) добавлены обязательные разделы:
     - `Units: ...` — явные единицы измерения всех входных и выходных величин;
     - `Example: ...` — конкретный JSON-пример аргументов вызова;
     - `Common errors: ...` — типичные коды структурированных ошибок (`PART_NOT_FOUND`, `INVALID_ARGUMENT`, `MESH_VALIDATION_FAILED`, `FACE_NOT_FOUND` и др.);
     - `Next step: ...` — рекомендуемый следующий инструмент в инженерном цикле.
   - Проведена ревизия всех числовых (`number`, `integer`) и числовых массивных (`array` of `number`/`integer`) полей в `inputSchema` всех 34 инструментов: каждое поле содержит явное указание единицы измерения (`length_unit`, `mm`, `deg`, `N`, `MPa`, `GPa`, `kg`, `rad/s`, `m/s`, `s`, `px`) либо помечено как `(dimensionless)` / `(dimensionless integer)`.

4. **Реализован инструмент №34 `get_scene_summary` в `crates/mcp-cad-server/src/tools/metrics.rs`**:
   - Возвращает за один вызов как компактную текстовую сводку (`summary_text`), так и структурированный JSON-объект для быстрой ориентации агента в новом контексте:
     - единицы проекта (`length_unit`, `units`);
     - список деталей (`parts_count`, `parts`), отсортированный по `part_id`: `part_id`, `name`, `material`, `is_fixed`, `revision`, `features_count`, `bounding_box` (`min`, `max`, `dimensions` в активном `length_unit`), `dimensions`, `volume` (в `mm^3` или `m^3`), `volume_m3`, `mass_kg`, `triangles_count`, `mesh_valid`, `mesh_status` (`PASS` / `FAIL` / `EMPTY`), `is_watertight`, `is_manifold`, `open_edges_count`, `genus`, а также `last_fea` (`verdict`, `factor_of_safety`, `max_von_mises_mpa`, `max_displacement_mm`, `fea_revision`, `part_revision`, `stale`, `status`: `"FRESH"` | `"STALE"` или `null`);
     - список кинематических шарниров (`joints_count`, `joints`), отсортированный по `joint_id`: `joint_id`, `joint_type`, `part_a`, `part_b`, `anchor_a`/`anchor_b` (в активном `length_unit`), `has_motor`, `motor` (`target_velocity`, `max_force`);
     - статус инженерных требований (`requirements`): `status` (`PASS` | `FAIL` | `STALE` | `NOT_AVAILABLE`), `auto_check`, счётчики `total`/`passed`/`failed`/`stale`/`not_available`, полная таблица `items` и список `failed_requirements`.
   - Зарегистрирован в `list_tools()` и `dispatch_tool()` (`crates/mcp-cad-server/src/tools/mod.rs`), задокументирован в `docs/MCP_API_REFERENCE.md`.

---

## 2. Как проверено

- **Unit-тесты (`mcp-cad-server --lib`)**:
  - `tools::metrics::tests::test_get_scene_summary_unit_conversion_and_joints` — проверяет сводку сцены с двумя деталями (`Plate A`, `Pin B`) и шарниром `Revolute` с мотором, а также пересчёт линейных габаритов и координат якорей между `"mm"` (`100.0 мм`, `50.0 мм`) и `"m"` (`0.1 м`, `0.05 м`).
- **Контрактные интеграционные тесты (`crates/mcp-cad-server/tests/mcp_contract_tests.rs`)**:
  - `test_mcp_initialize` — проверяет наличие непустого `result.instructions` со ссылкой на `docs/AGENT_GUIDE.md` и упоминанием `get_scene_summary`.
  - `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` — проверяет:
    1. длину `initialize.instructions` (`800..=1650` символов) и наличие ключевых тем (`docs/AGENT_GUIDE.md`, `get_scene_summary`, `length_unit`, `set_requirements`, `auto_check`, `batch_cad_operations`, `validate_mesh`, `run_fea_analysis`, `query_part_geometry`, `STALE`, `isError`);
    2. наличие и компактность файла `docs/AGENT_GUIDE.md` (`40..=160` строк);
    3. наличие подстрок `"Example"`, `"Units:"`, `"Common errors:"`, `"Next step:"` в `description` каждого из 34 инструментов `tools/list`;
    4. рекурсивный обход `inputSchema` каждого из 34 инструментов (`assert_numeric_schema_fields_have_units`), гарантирующий, что ни одно числовое поле (`number`, `integer` или числовой массив) не осталось без указания единицы измерения или `dimensionless`.
  - `test_mcp_p2_t10_get_scene_summary_two_parts` — проверяет `get_scene_summary` на пустой сцене, затем на сцене из двух деталей (`base_bracket` и `rotor_arm`), созданных через `batch_cad_operations`, с шарниром `revolute` и мотором, требованиями `set_requirements` (`auto_check: true`) и расчётом МКЭ `run_fea_analysis` (`last_fea.status == "FRESH"`, `requirements.status == "PASS"`), а затем после модификации геометрии `modify_feature` проверяет переход `last_fea.status` и `requirements.status` в `"STALE"` и пересчёт в `length_unit: "m"`.
- **Линтер**:
  - `cargo clippy --workspace --all-targets -- -D warnings` — 0 ошибок и 0 предупреждений.

---

## 3. Тесты до / после

| Набор тестов | До P2-T10 | После P2-T10 | Дельта |
|---|---|---|---|
| `mcp-cad-server` (`--lib`) | 30 passed | **31 passed** | +1 (`test_get_scene_summary_unit_conversion_and_joints`) |
| `mcp-cad-server` (`mcp_contract_tests`) | 29 passed | **31 passed** | +2 (`test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`, `test_mcp_p2_t10_get_scene_summary_two_parts`) |
| Остальные крейты и суиты workspace | 147 passed | **147 passed** | 0 регрессий |
| **Всего по Rust Workspace** | **206 passed, 0 failed, 0 ignored** | **209 passed, 0 failed, 0 ignored** | **+3 теста (100% PASS)** |

---

## 4. Что не удалось и остаточные риски

- Все критерии приёмки задачи **P2-T10** выполнены в полном объёме.
- **Остаточные риски**:
  - При добавлении новых инструментов в будущих фазах (`P3-T6` `get_part_load_cases`, `P3-T8` `check_gear_strength`, `P3-T10` `check_interference` / `sweep_joint`) необходимо соблюдать контракт описаний (`Units:`, `Example:`, `Common errors:`, `Next step:`) и единиц в `inputSchema` — теперь это автоматически контролируется тестом `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`.

---

## 5. Побочные находки

- При прогоне интеграционного суита `mcp_contract_tests` синхронизированы поля `revision` в JSON-ответах `create_part`, `add_feature_primitive`, `cut_hole`, `modify_feature`, поле `completed_steps` в `batch_cad_operations`, поле `verdict` в `run_fea_analysis` и поля `auto_check` / `requirements_count` в `requirements.rs`. Новых дефектов сверх уже заведённых в `TASKS.md` не обнаружено.
