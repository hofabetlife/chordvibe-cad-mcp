# Отчёт о выполнении задачи P2-T4

## 1. Задача и что было сделано
- **ID**: `P2-T4`
- **Название**: Сохранение и загрузка проекта (`save_project` / `load_project`)
- **Фаза**: P2 — Контракт «агент ↔ система»
- **Ветка**: `agent/P2-T4`

### Контекст
Ранее `EngineState` (`crates/mcp-cad-server/src/state.rs`) хранил детали, материалы и физические шарниры исключительно в оперативной памяти процесса сервера. При перезапуске сервера или сбросе сессии вся выполненная ИИ-агентом или инженером работа безвозвратно терялась. При этом `Part` (`mcp-cad-core/src/part.rs`) содержал полную параметрическую историю в виде дерева фич DAG (`FeatureTree`), а `PhysicsWorld` управлял кинематическими шарнирами Rapier3D. Требовалось реализовать версионированное сохранение и транзакционную загрузку проекта, при которой файлы остаются компактными (без сохранения кэшей и полигональных сеток треугольников), а сетки гарантированно пересчитываются из DAG при загрузке.

### Что сделано
1. **Модель персистентности проекта (`crates/mcp-cad-server/src/tools/project.rs`)**:
   - Реализована константа `PROJECT_SCHEMA_VERSION = "1.0.0"`.
   - Введена структура `ProjectFile`:
     - `$schema`: ссылка на мета-схему JSON-Draft 2020-12;
     - `schema_version`: стабильная версия схемы (`"1.0.0"`);
     - `project`: название проекта или механизма;
     - `length_unit`: активная линейная единица сессии (`"mm"` или `"m"`);
     - `units`: словарь контракта единиц измерения;
     - `materials`: список пользовательских материалов, зарегистрированных через `define_material` (стандартные встроенные пресеты не дублируются, если не были переопределены);
     - `parts`: список деталей `ProjectPart` (содержит `id`, `name`, `material`, `dag: FeatureTree`, флаг `is_fixed`);
     - `joints`: список кинематических суставов `SavedJoint` (содержит `id`, `part_a`, `part_b`, `joint_type: CadJointType`, опциональный `motor: Option<MotorConfig>`).
   - Гибкий enum `PartMaterialRef` (`#[serde(untagged)]`): поддерживает как полный объект `Material`, так и строковый идентификатор пресета (например, `"aluminum_6061_t6"`).
   - Принцип компактности мешей: для процедурных деталей поле `mesh` опускается (`None`), а при загрузке вызывается `part.rebuild()`, пересчитывающий сетку из DAG. Если деталь была импортирована из сырого STL без дерева DAG (`nodes.is_empty()`), исходная сетка сохраняется, предотвращая потерю геометрии.

2. **Сохранение и отслеживание моторов шарниров (`crates/mcp-cad-physics/src/simulator.rs`)**:
   - В структуру `CadJointRecord` добавлено поле `pub motor: Option<MotorConfig>`.
   - При вызове `add_joint` конфигурация мотора сохраняется в записи шарнира.
   - Методы `set_joint_motor` и `disable_joint_motor` обновляют поле `record.motor = Some(motor)` / `None`.
   - Добавлен публичный метод `PhysicsWorld::joints(&self) -> &HashMap<String, CadJointRecord>`.

3. **Инструмент №30 `save_project` (`crates/mcp-cad-server/src/tools/project.rs`)**:
   - Принимает `path` (или `filepath`) и опциональный `project_name`.
   - Собирает детали и шарниры, детерминированно сортируя их по `id`.
   - Автоматически создает родительские директории (`std::fs::create_dir_all`).
   - Сериализует форматированный JSON (`serde_json::to_string_pretty`).
   - Возвращает сводку: путь к файлу, размер в байтах, число сохраненных деталей, шарниров и материалов, единицы измерения.

4. **Инструмент №31 `load_project` с транзакционной гарантией (`crates/mcp-cad-server/src/tools/project.rs`)**:
   - Проверяет версию схемы `project_file.schema_version == PROJECT_SCHEMA_VERSION`. При несовпадении возвращает `PROJECT_SCHEMA_MISMATCH`.
   - Парсит `LengthUnit`.
   - **Фаза транзакционной валидации**: строит кандидатный реестр материалов `MaterialRegistry`, восстанавливает и пересчитывает меши всех деталей (`part.rebuild()`), инициализирует новый экземпляр `PhysicsWorld`, регистрирует детали и шарниры с моторами.
   - Если любой этап завершается ошибкой (поврежденный JSON, отсутствующий файл, сбой пересчета фичи DAG, ссылка шарнира на несуществующую деталь), операция немедленно возвращает ошибку, а текущее состояние `EngineState` остается **полностью нетронутым**.
   - При успехе выполняется атомарная замена `state.length_unit`, `state.materials`, `state.parts`, `state.physics_world`, сброс кэшей `last_fea_reports`, `last_fea_meshes`, `last_simulation` и отправка обновленного меша в GUI.
   - Возвращает подробную сводку: список загруженных деталей с объемами и массами в активных единицах, список шарниров и флаги активности моторов.

5. **Регистрация и диспетчеризация инструментов (`crates/mcp-cad-server/src/tools/mod.rs`, `crates/mcp-cad-server/src/lib.rs`)**:
   - Экспортирован модуль `pub mod project;`.
   - В `list_tools()` добавлены определения инструментов №30 `save_project` и №31 `load_project` с полной JSON Schema.
   - В `dispatch_tool` добавлены ветки вызова `project::save_project` и `project::load_project`.
   - Экспортированы наружу `ProjectFile`, `ProjectPart`, `SavedJoint`, `PROJECT_SCHEMA_VERSION`.

6. **Структурированная ошибка несовпадения схемы (`crates/mcp-cad-server/src/error.rs`)**:
   - Добавлен маппинг `msg.contains("schema_version")` в машиночитаемый код `PROJECT_SCHEMA_MISMATCH` с параметром `schema_version` и подсказкой о текущей версии `1.0.0`.
   - Добавлен unit-тест `test_to_error_body_project_schema_mismatch`.

7. **Документация**:
   - Обновлен [`docs/MCP_API_REFERENCE.md`](../MCP_API_REFERENCE.md): добавлена Группа 7 («Персистентность и управление проектами»), описание и примеры запросов/ответов для инструментов 30 (`save_project`) и 31 (`load_project`), код `PROJECT_SCHEMA_MISMATCH` в реестр ошибок.

---

## 2. Как проверено (команды и итоговые строки `test result`)

### 1. Unit-тесты `tools/project.rs` и `error.rs` (`mcp-cad-server --lib`):
```bash
cargo test -p mcp-cad-server --lib
```
Результат:
```
running 24 tests
test error::tests::test_to_error_body_empty_geometry ... ok
test error::tests::test_to_error_body_fea_solver_convergence_failed ... ok
test error::tests::test_to_error_body_cad_csg_error ... ok
test error::tests::test_error_envelope_json_roundtrip ... ok
test error::tests::test_to_error_body_invalid_primitive_type ... ok
test error::tests::test_to_error_body_fea_unconstrained ... ok
test error::tests::test_to_error_body_feature_not_found ... ok
test error::tests::test_to_error_body_io_and_json ... ok
test error::tests::test_to_error_body_joint_not_found ... ok
test error::tests::test_to_error_body_part_not_found_without_parts ... ok
test error::tests::test_to_error_body_missing_required_argument ... ok
test error::tests::test_to_error_body_part_not_found_with_parts ... ok
test error::tests::test_to_error_body_physics_simulation_error ... ok
test error::tests::test_to_error_body_project_schema_mismatch ... ok
test error::tests::test_to_error_body_revolve_angle_deg ... ok
test error::tests::test_to_error_body_revolve_negative_r ... ok
test error::tests::test_to_error_body_unknown_material ... ok
test error::tests::test_to_error_body_tool_not_found ... ok
test error::tests::test_to_error_body_viewer_empty_mesh ... ok
test tools::project::tests::test_part_material_ref_deserialize ... ok
test tools::project::tests::test_project_schema_version_constant ... ok
test tools::project::tests::test_load_incompatible_schema_version ... ok
test tools::project::tests::test_load_corrupted_file_preserves_state ... ok
test tools::project::tests::test_save_and_load_roundtrip ... ok

test result: ok. 24 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.79s
```

### 2. Контрактные тесты сервера (`tests/mcp_contract_tests.rs`):
```bash
cargo test -p mcp-cad-server --test mcp_contract_tests
```
Результат:
```
running 25 tests
...
test test_mcp_tools_list ... ok
test test_bug015_create_joint_and_create_part_schema_contract ... ok
test test_project_save_and_load_mcp_contract ... ok
...
test result: ok. 25 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 20.21s
```

### 3. Тесты физики (`mcp-cad-physics`):
```bash
cargo test -p mcp-cad-physics
```
Результат:
```
test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 1.03s
```

### 4. Тесты геометрии ядра (`mcp-cad-core`):
```bash
cargo test -p mcp-cad-core
```
Результат:
```
test result: ok. 50 passed; 0 failed; 0 ignored (lib)
test result: ok. 5 passed; 0 failed; 0 ignored (csg_invariants)
test result: ok. 14 passed; 0 failed; 0 ignored (dag_tests)
test result: ok. 18 passed; 0 failed; 0 ignored (primitive_invariants)
test result: ok. 6 passed; 0 failed; 0 ignored (tessellation_tests)
```

### 5. Полный суит интеграционных и QA-тестов сервера:
```bash
cargo test -p mcp-cad-server --test qa_wide_scan
cargo test -p mcp-cad-server --test gear_kinematic_test
cargo test -p mcp-cad-server --test bench_suite
cargo test -p mcp-cad-fea
cargo test -p mcp-cad-viewer
```
Результат:
- `qa_wide_scan`: 13 passed, 0 failed
- `gear_kinematic_test`: 1 passed, 0 failed
- `bench_suite`: 1 passed, 0 failed
- `mcp-cad-fea`: 11 passed (5 lib + 6 tests), 0 failed
- `mcp-cad-viewer`: 10 passed, 0 failed

### 6. Строгая проверка `cargo clippy`:
```bash
cargo clippy --workspace --all-targets -- -D warnings
```
Результат:
```
Finished dev profile [unoptimized + debuginfo] target(s) in 0.34s
0 errors, 0 warnings across all crates and targets.
```

---

## 3. Тесты до и после

| Набор тестов | До P2-T4 | После P2-T4 | Статус |
|---|---|---|---|
| `mcp-cad-core --lib` | 50 passed | 50 passed | 100% PASS |
| `csg_invariants` | 5 passed | 5 passed | 100% PASS |
| `dag_tests` | 14 passed | 14 passed | 100% PASS |
| `primitive_invariants` | 18 passed | 18 passed | 100% PASS |
| `tessellation_tests` | 6 passed | 6 passed | 100% PASS |
| `mcp-cad-fea` (lib + tests) | 11 passed | 11 passed | 100% PASS |
| `mcp-cad-physics` | 6 passed | 6 passed | 100% PASS |
| `mcp-cad-viewer` | 10 passed | 10 passed | 100% PASS |
| `mcp-cad-server --lib` | 18 passed | **24 passed** (+6 новых тестов проекта и схемы) | 100% PASS |
| `mcp_contract_tests` | 24 passed | **25 passed** (+1 новый тест контракта сохранения/загрузки) | 100% PASS |
| `qa_wide_scan` | 13 passed | 13 passed | 100% PASS |
| `gear_kinematic_test` | 1 passed | 1 passed | 100% PASS |
| `bench_suite` | 1 passed | 1 passed | 100% PASS |
| **Всего в workspace** | **177 passed** | **184 passed (0 failed, 0 ignored)** | **100% PASS** |
| `cargo clippy` | 0 errors, 0 warnings | **0 errors, 0 warnings** | PASS |

---

## 4. Что не удалось и остаточные риски
- В рамках P2-T4 сериализуются только процедурные деревья DAG и параметры шарниров/материалов, а результаты FEA и рендеры намеренно не сохраняются (в соответствии с критерием «Не делать: не сохранять FEA-результаты и рендеры»). При последующей задаче P2-T9 (слой требований) потребуется проверять валидность сохраненных требований при загрузке проекта.
- Если в будущем потребуется сохранение внешних ссылок на файлы геометрии (например, STL-файлов, импортированных без DAG), формат `ProjectPart.mesh` уже готов хранить эти вершины без изменения версии схемы.

---

## 5. Побочные находки (каталог для реестра задач)
- Все выявленные в ходе работы нюансы (регистрация алиасов материалов в `define_material`, количество инструментов в `tools/list`) учтены и закрыты тестами. Новых блокирующих дефектов в ядре не обнаружено.
