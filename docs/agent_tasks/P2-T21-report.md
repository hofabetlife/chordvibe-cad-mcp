# Отчёт о выполнении задачи P2-T21

- **Статус**: DONE
- **Задача**: `P2-T21 [S]` — Проброс `wheel_rim_diameter_m` в `check_scale_sanity` при `modify_feature` (`AddWheelRim`) и вывод `limits` шарниров в `get_scene_summary` (`metrics.rs:427`)

## Затронутые файлы
- `crates/mcp-cad-server/src/tools/cad.rs` — в функции `modify_feature` (строки `1156–1610`) добавлены сбор линейных размеров `extra_dims_m: Vec<(&str, f64)>` для всех модифицируемых вариантов `FeatureOp` и передача `wheel_rim_diameter_m = Some(new_dia)` для `FeatureOp::AddWheelRim` в вызов `state.check_scale_sanity(part, &extra_dims_m, wheel_rim_diameter_m, unit)`. Добавлен unit-тест `test_p2_t21_modify_feature_wheel_rim_scale_warning`.
- `crates/mcp-cad-server/src/tools/metrics.rs` — в функции `get_scene_summary` (строки `226–375`) при формировании массива `joints` и текстовой сводки `summary_text` добавлен вывод ограничений хода `limits` (`[min, max]` с пересчётом из радиан в градусы `deg` для `CadJointType::Revolute` и из метров в активный `length_unit` для `CadJointType::Prismatic`), поля `limits_unit` (`"deg"`, `"mm"`/`"m"` или `null`), вектора оси `axis` и передаточного отношения `ratio` (для `CadJointType::Gear`). Обновлён unit-тест `test_get_scene_summary_unit_conversion_and_joints`.
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs` — добавлен интеграционный MCP-тест `test_mcp_p2_t21_modify_feature_wheel_rim_scale_warning_and_scene_summary_limits`.
- `docs/MCP_API_REFERENCE.md` — обновлено описание выходных полей массива `joints` в разделе `34. get_scene_summary`.
- `TASKS.md` — статус задачи `P2-T21` переведён в `✅ DONE`, обновлён блок аудита и зарегистрирована новая задача `P2-T26`.

## Корневая причина и решение
1. **Пропуск предупреждения `WHEEL_RIM_SCALE` при `modify_feature` (`AddWheelRim`)**:
   - В `add_feature_primitive` (`crates/mcp-cad-server/src/tools/cad.rs:795`) для `primitive_type == "wheel_rim"` в `state.check_scale_sanity` передавался `Some(rim_diameter)`, что генерировало предупреждение `WHEEL_RIM_SCALE` при `rim_diameter < 0.15 м` (`150 мм`).
   - В `modify_feature` (`crates/mcp-cad-server/src/tools/cad.rs`) после пересчёта `part.rebuild()` вызывался `state.check_scale_sanity(part, &[], None, unit)` с захардкоженными пустыми `extra_dims_m` и `wheel_rim_diameter_m = None`. В результате уменьшение `rim_diameter` ниже `150 мм` через `modify_feature` не выдавало предупреждение `WHEEL_RIM_SCALE`.
   - **Решение**: В `modify_feature` для каждой ветки `FeatureOp` формируется список обновлённых линейных параметров `extra_dims_m` (в метрах), а для `FeatureOp::AddWheelRim` устанавливается `wheel_rim_diameter_m = Some(new_dia)`, которые передаются в `state.check_scale_sanity(part, &extra_dims_m, wheel_rim_diameter_m, unit)`.
2. **Отсутствие `limits` шарниров в `get_scene_summary` (`metrics.rs`)**:
   - В `get_scene_summary` (`crates/mcp-cad-server/src/tools/metrics.rs`) элементы массива `joints` содержали только `joint_id`, `joint_type`, `part_a`, `part_b`, `anchor_a`, `anchor_b` и `motor`, опуская `limits`, `limits_unit`, `axis` и `ratio`.
   - **Решение**: Для каждого шарнира в `get_scene_summary` извлекаются:
     - `Revolute { axis, limits }`: `axis`, `limits: [min.to_degrees(), max.to_degrees()]`, `limits_unit: "deg"`;
     - `Prismatic { axis, limits }`: `axis`, `limits: [unit.from_meters(min), unit.from_meters(max)]`, `limits_unit: unit.as_str()`;
     - `Gear { axis_a, ratio, .. }`: `axis: axis_a`, `ratio`.
     В текстовую сводку `summary_text` также добавлен вывод `limits=[min, max] <unit>` при наличии ограничений хода.

## Результаты тестов (до / после)
- `cargo test -p mcp-cad-server --lib`: **33 passed → 34 passed, 0 failed**.
- `cargo test -p mcp-cad-server --test mcp_contract_tests`: **39 passed → 40 passed, 0 failed**.
- `cargo clippy --workspace --all-targets -- -D warnings`: **0 warnings, 0 errors**.
- `npx tsx --test src/cad/cad_engine.test.ts`: **29 passed, 0 failed**.

## Новые найденные проблемы
- Зарегистрирована в `TASKS.md` как **`P2-T26 [S]`**:
  1. В `boolean_difference`, `boolean_union` и `boolean_intersection` (`crates/mcp-cad-server/src/tools/cad.rs:1846-1912, 2622-2773`) ответы не включают `revision`, `bounding_box`, `volume`, `volume_m3` и `warnings` (`check_scale_sanity`), а единица длины берётся из `state.length_unit` без учёта локального переопределения `state.resolve_length_unit(args)?`.
  2. В `load_project` (`crates/mcp-cad-server/src/tools/project.rs:370-385`) сводка загруженных шарниров `joints` не включает поля `limits`, `limits_unit`, `axis` и `ratio`.

## Выполнение критериев приёмки
- [x] `modify_feature` для `FeatureOp::AddWheelRim` передаёт `wheel_rim_diameter_m = Some(new_dia)` и `extra_dims_m` в `state.check_scale_sanity`, возвращая предупреждение `WHEEL_RIM_SCALE` при `rim_diameter < 150 мм`.
- [x] `get_scene_summary` включает `limits` (в `deg` для `Revolute` и в активном `length_unit` для `Prismatic`), `limits_unit`, `axis` и `ratio` как в JSON-массиве `joints`, так и в `summary_text`.
- [x] Добавлены unit- и контрактные тесты, подтверждающие оба исправления.
