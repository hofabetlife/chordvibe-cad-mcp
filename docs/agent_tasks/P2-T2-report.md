# Отчёт о выполнении задачи P2-T2

## 1. Задача и что было сделано
- **ID**: `P2-T2`
- **Название**: Внедрение контракта единиц
- **Фаза**: P2 — Контракт для ИИ-агента и сквозные сценарии
- **Ветка**: `agent/P2-T2`

### Реализованный контракт (`[DECISION-UNITS-1]` из [`docs/UNITS.md`](../UNITS.md))
На основе спецификации из `docs/UNITS.md` (раздел 3, вариант по умолчанию `[DECISION-UNITS-1]`) реализовано сквозное управление и конвертация единиц измерения на границе MCP-сервера (`crates/mcp-cad-server`) при полном сохранении строгой системы СИ (`m`, `Pa`, `N`, `kg`, `s`) внутри библиотечных ядер `mcp-cad-core`, `mcp-cad-fea`, `mcp-cad-physics` (код этих крейтов **не изменялся**):

- **Линейные размеры по умолчанию**: `length_unit = "mm"` (`LengthUnit::Mm`, $k_{\text{to\_m}} = 10^{-3}$, $k_{\text{from\_m}} = 10^{3}$), с поддержкой переключения сессии в `"m"` (`LengthUnit::M`, $k_{\text{to\_m}} = 1.0$) через инструмент №29 `set_units` или локального переопределения `"length_unit": "mm" | "m"` в аргументах любого геометрического/физического/прочностного инструмента.
- **Сила и крутящий момент**: `N` и `N·m` (СИ).
- **Напряжения и предел текучести**: `MPa` ($10^6\text{ Па}$).
- **Модуль Юнга**: `GPa` ($10^9\text{ Па}$).
- **Масса и плотность**: `kg` и `kg/m³` (СИ).
- **Перемещения МКЭ**: `mm` (`max_displacement_mm`).
- **Углы и скорости**: углы поворота и пределы вращательных шарниров — `deg`; угловая скорость привода — `rad/s`; линейная скорость привода — `m/s`.
- **Экспорт/импорт STL на диск**: координаты вершин в файлах `.stl` сохраняются в канонических метрах СИ (`m`) без масштабирования (согласно требованию *«Не менять сохранение в STL (оставить метры, зафиксировать в документации)»*), что отражено в `docs/MCP_API_REFERENCE.md`, `docs/UNITS.md` и в поле `"stl_file_coordinates": "m"` ответа `export_stl`.

---

## 2. Полный список изменённых и добавленных MCP-инструментов (29 инструментов)

| № | Инструмент | Файл обработчика | Входные параметры (конвертация на границе) | Выходные поля (`units`, `length_unit`, `warnings`) |
|---|---|---|---|---|
| — | `initialize` (JSON-RPC) | `src/protocol.rs` | — | Добавлены `result.length_unit` (`"mm"`) и `result.units` |
| 1 | `create_part` | `src/tools/cad.rs` | — | Добавлены `length_unit`, `units` |
| 2 | `list_parts` | `src/tools/cad.rs` | — | `bounding_box`, `bounding_box_dimensions` в `length_unit`, `volume` в `mm³`/`m³`, `volume_m3` в `m³`, `mass`/`mass_kg` в `kg`, верхнеуровневые `length_unit`, `units` |
| 3 | `delete_part` | `src/tools/cad.rs` | — | Без числовых полей |
| 4 | `add_feature_primitive` | `src/tools/cad.rs` | Все линейные параметры `box`, `cylinder`, `sphere`, `cone`, `polygon_prism`, `spur_gear` (`module`, `face_width`, `bore_radius`, `keyway_*`, `hub_*`), `revolve` (`profile [r, z]`), `wheel_rim`, `position` умножаются на `k_to_m` | Добавлены `length_unit`, `units`, `warnings` (`check_scale_sanity`) |
| 5 | `cut_hole` | `src/tools/cad.rs` | `position`, `radius`, `depth`, `keyway_width`, `keyway_depth` умножаются на `k_to_m` | Добавлены `length_unit`, `units`, `warnings` |
| 6 | `modify_feature` | `src/tools/cad.rs` | Все линейные параметры узла DAG (`AddBox`, `AddCylinder`, `AddSphere`, `AddCone`, `AddPolygonPrism`, `AddSpurGear`, `AddRevolve`, `AddWheelRim`, `CutHole`, `CutHelicalGroove`, `Transform.translation`) умножаются на `k_to_m` | Добавлены `length_unit`, `units`, `warnings` |
| 7 | `get_feature_tree` | `src/tools/cad.rs` | Опциональный `length_unit` | Все линейные параметры узлов `dag` масштабируются из метров через `scale_feature_op_from_meters(..., k_from_m)`; добавлены `length_unit`, `units` |
| 8 | `cut_helical_groove` | `src/tools/cad.rs` | `helix_radius`, `groove_radius`, `pitch`, `height`, `start_z` умножаются на `k_to_m` | Добавлены `length_unit`, `units`, `warnings` |
| 9 | `boolean_difference` | `src/tools/cad.rs` | Опциональный `length_unit` | `resulting_volume` в `mm³`/`m³`, `resulting_volume_m3` в `m³`, `length_unit`, `units` |
| 10 | `import_stl` | `src/tools/cad.rs` | `weld_tolerance` умножается на `k_to_m` (дефолт `1e-6 м`); добавлена проверка `STL_LIKELY_MILLIMETERS` при `max_extent > 10.0` | `volume` в `mm³`/`m³`, `volume_m3` в `m³`, `bounding_box` в `length_unit`, `length_unit`, `units`, `warnings` |
| 11 | `export_stl` | `src/tools/cad.rs` | `chord_tolerance` умножается на `k_to_m`; координаты в `.stl`-файле остаются в `m` | Добавлены `length_unit`, `units` (с `"stl_file_coordinates": "m"`) |
| 12 | `tessellate_part` | `src/tools/cad.rs` | `chord_tolerance` умножается на `k_to_m` | `volume` в `mm³`/`m³`, `volume_m3` в `m³`, `surface_area` в `mm²`/`m²`, `surface_area_m2` в `m²`, `chord_tolerance` в `length_unit`, `chord_tolerance_m`, `length_unit`, `units` |
| 13 | `query_part_geometry` | `src/tools/cad.rs` | Опциональный `length_unit` | `bounding_box`, `dimensions`, `center_of_mass`, `detected_planar_faces[].center` в `length_unit`; `surface_area` в `mm²`/`m²` (`surface_area_m2` в `m²`); `volume` в `mm³`/`m³` (`volume_m3` в `m³`); `mass` и `mass_kg` в `kg`; `length_unit`, `units` |
| 14 | `validate_mesh` | `src/tools/cad.rs` | Опциональный `length_unit` | Добавлены `signed_volume` (в `mm³`/`m³`), `signed_volume_m3` (`m³`), `length_unit`, `units` |
| 15 | `boolean_union` | `src/tools/cad.rs` | Опциональный `length_unit` | `resulting_volume` в `mm³`/`m³`, `resulting_volume_m3` в `m³`, `length_unit`, `units` |
| 16 | `boolean_intersection` | `src/tools/cad.rs` | Опциональный `length_unit` | `resulting_volume` в `mm³`/`m³`, `resulting_volume_m3` в `m³`, `length_unit`, `units` |
| 17 | `transform_part` | `src/tools/cad.rs` | `translation` умножается на `k_to_m`; `rotation_deg` в `deg`; `scale` безразмерный | `bounding_box` и `center_of_mass` в `length_unit`; `volume` в `mm³`/`m³`, `volume_m3` в `m³`; `length_unit`, `units`, `warnings` (`check_scale_sanity`) |
| 18 | `define_material` | `src/tools/cad.rs` | `youngs_modulus_gpa` (`GPa`), `yield_strength_mpa` (`MPa`), `density_kg_m3` (`kg/m³`) | Добавлено поле `units` |
| 19 | `batch_cad_operations` | `src/tools/cad.rs` | Откатывает и `state.length_unit` при ошибке шага | Добавлены `length_unit`, `units` |
| 20 | `optimize_feature_tree` | `src/tools/cad.rs` | Опциональный `length_unit` | `volume_before` / `volume_after` в `mm³`/`m³`, `volume_before_m3` / `volume_after_m3` в `m³`, `length_unit`, `units` |
| 21 | `create_joint` | `src/tools/physics.rs` | `anchor_a`, `anchor_b` умножаются на `k_to_m`; `limits` для `prismatic` умножаются на `k_to_m`, для `revolute` переводятся из `deg` в `rad` (`.to_radians()`) | Добавлены `length_unit`, `units` |
| 22 | `set_joint_motor` | `src/tools/physics.rs` | `target_velocity` (`rad/s` или `m/s`), `max_force` (`N·m` или `N`) | Добавлено поле `units` |
| 23 | `run_physics_simulation` | `src/tools/physics.rs` | `duration_seconds` (`s`), `step_size_ms` (`ms`) | Координаты `position` в `trajectory_summary` масштабируются на `k_from_m` (при сохранении СИ в `state.last_simulation`); добавлены `length_unit`, `units` |
| 24 | `run_fea_analysis` | `src/tools/fea.rs` | `fixed_plane.origin`, `fixed_plane.tolerance` и `applied_forces[].position` умножаются на `k_to_m`; `force_vector` в `N` | `max_von_mises_mpa`, `yield_strength_mpa` в `MPa`, `max_displacement_mm` в `mm`, добавлены `length_unit`, `units` |
| 25 | `get_fea_heatmap_image` | `src/tools/fea.rs` | `width`, `height` (`px`) | Добавлены `length_unit`, `units` |
| 26 | `capture_viewport` | `src/tools/render.rs` | `width`, `height` (`px`), `camera_angle` | Изображение PNG |
| 27 | `open_preview_window` | `src/tools/render.rs` | — | Статус окна предпросмотра |
| 28 | `get_simulation_metrics_json` | `src/tools/metrics.rs` | — | Сводки деталей (`part_summary_in_unit`) и позиции в `last_simulation` конвертируются в `state.length_unit`; добавлены `length_unit`, `units` |
| 29 | `set_units` (**новый**) | `src/tools/cad.rs` | `length_unit: "mm" \| "m"` | Возвращает `status: "units_updated"`, `previous_length_unit`, `length_unit`, `units` |

---

## 3. Как проверено (точные команды и вывод)

```bash
export PATH="/root/.cargo/bin:$PATH"
cargo clippy --workspace --all-targets -- -D warnings
cargo test -p mcp-cad-server --test mcp_contract_tests --test gear_kinematic_test --test qa_wide_scan --test bench_suite
cargo test -p mcp-cad-core -p mcp-cad-fea -p mcp-cad-physics -p mcp-cad-viewer
```

Вывод тестовых наборов:
```text
Running tests/gear_kinematic_test.rs
test test_gear_kinematic_simulation_and_preview ... ok
test result: ok. 1 passed; 0 failed; 0 ignored

Running tests/mcp_contract_tests.rs
test test_units_equivalence_mm_vs_m ... ok
test test_units_scale_sanity_warnings_20m_and_micro ... ok
...
test result: ok. 23 passed; 0 failed; 0 ignored

Running tests/qa_wide_scan.rs
test result: ok. 13 passed; 0 failed; 0 ignored

Running tests/bench_suite.rs
test test_rust_benchmark_15_tasks_baseline ... ok
test result: ok. 1 passed; 0 failed; 0 ignored
```

---

## 4. Тесты до и после

- **До изменений**: `156 passed, 0 failed, 0 ignored` (`mcp_contract_tests`: 21 тест).
- **После изменений**: `158 passed, 0 failed, 0 ignored` (`mcp_contract_tests`: 23 теста, включая два новых интеграционных теста):
  1. `test_units_equivalence_mm_vs_m`: создаёт одну и ту же консольную балку `200×20×20 мм` (`0.2×0.02×0.02 м`) со сквозным отверстием `R=5 мм` (`0.005 м`), закреплённую по `min_x`, под поперечной нагрузкой `500 Н` в точке `x = 100 мм` (`0.1 м`) в режиме по умолчанию `"mm"` и после `set_units({"length_unit": "m"})`, и проверяет совпадение физических инвариантов (`volume_m3`, `mass_kg`, `max_von_mises_mpa`, `max_displacement_mm`, `safety_factor`) с относительной погрешностью `< 1e-6`.
  2. `test_units_scale_sanity_warnings_20m_and_micro`: проверяет срабатывание предупреждений `SCALE_TOO_LARGE` при создании детали габаритом `20 000 мм` (`20 м`) и при масштабировании детали в `200` раз через `transform_part`, `SCALE_TOO_SMALL` при габарите `0.05 мм` (`< 0.1 мм`) и `WHEEL_RIM_SCALE` при диаметре обода `145 мм` (`< 150 мм`).

---

## 5. Остаточные риски и побочные находки

- При написании теста `test_units_scale_sanity_warnings_20m_and_micro` экспериментально подтвердилась ранее зарегистрированная в `P2-T1` задача **`P2-T15`**: вызов `wheel_rim` с `rim_diameter < 140.3 мм` без явного `hub_diameter` и `hub_bore_diameter` вызывает панику `f64::clamp` (`min > max`) в `crates/mcp-cad-core/src/primitives.rs:3052` (так как `mcp-cad-core` не должен модифицироваться в рамках `P2-T2`, исправление остаётся в задаче `P2-T15`).
