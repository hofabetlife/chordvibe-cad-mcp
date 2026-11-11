# Отчёт о выполнении задачи P2-T15

**Задача:** `P2-T15 [S]` — Защита от паники `f64::clamp` (`min > max`) в `create_wheel_rim_with_settings` (`primitives.rs`) и исправление мотора `JointAxis::LinX` для `Prismatic` / `JointAxis::AngX` для `Revolute` в `PhysicsWorld::set_joint_motor` (`joint.rs`, `simulator.rs`, `tools/physics.rs`)  
**Дата:** 07.11.2026  
**Статус:** ✅ DONE  
**Затронутые файлы:**
- `crates/mcp-cad-core/src/primitives.rs`
- `crates/mcp-cad-core/src/dag.rs`
- `crates/mcp-cad-core/tests/primitive_invariants.rs`
- `crates/mcp-cad-physics/src/joint.rs`
- `crates/mcp-cad-physics/src/simulator.rs`
- `crates/mcp-cad-physics/tests/physics_tests.rs`
- `crates/mcp-cad-server/src/tools/physics.rs`
- `crates/mcp-cad-server/src/error.rs`
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs`
- `src/cad/primitives.ts`
- `src/cad/kinematics.ts`
- `src/cad/mcp_dispatcher.ts`
- `src/cad/cad_engine.test.ts`
- `TASKS.md`
- `docs/agent_tasks/P2-T15-report.md`

---

## 1. Описание проблемы

При аудите размерных констант и физического движка в рамках **P2-T1** ([`docs/UNITS.md`](../UNITS.md)) были выявлены следующие дефекты:

1. **Паника `f64::clamp` (`min > max`) в `create_wheel_rim_with_settings` (`crates/mcp-cad-core/src/primitives.rs:3328–3495`) на колёсах малого диаметра (`< 140.3 мм`) или малой ширины**:
   - Вычисление диаметра расположения болтовых отверстий `let d_pcd = pcd_diameter.unwrap_or((d_hub + d_bore) * 0.5).clamp(d_bore + 0.015, d_hub - 0.012);` использовало захардкоженные в метрах абсолютные зазоры `15 мм` и `12 мм`. При `rim_diameter < 0.1403 м` (`140.3 мм`) нижняя граница `d_bore + 0.015` становилась строго больше верхней `d_hub - 0.012`, вызывая панику `f64::clamp` (`min > max`).
   - Аналогичные захардкоженные в метрах нижние пороги в `.clamp(min, max)` присутствовали для ширины спицы `w_spoke` (`0.010`), толщины спицы `t_spoke` (`0.008`), радиуса болта `r_bolt_eff` (`0.002`), толщины ступичного диска `t_disc` (`0.010`) и осевого положения диска `z_disc_center` (`min_barrel_dz = 0.006`), что вызывало панику или выворачивание O-сетки болтовых секторов при уменьшении `rim_diameter` или `rim_width`.
   - Кроме того, в `crates/mcp-cad-core/src/dag.rs:200` функция `default_bolt_hole_diameter()` возвращала фиксированное `Some(0.014)` (`14 мм`), препятствуя пропорциональному масштабированию диаметра болтовых отверстий при десериализации `FeatureOp::AddWheelRim`.

2. **Пропуск `set_motor_max_force` / `wake_up` для моторов `Prismatic` (`JointAxis::LinX`) и `Revolute` (`JointAxis::AngX`) и отсутствие валидации `limits` в `create_joint`**:
   - В `CadJointType::apply_motor` (`crates/mcp-cad-physics/src/joint.rs:209–242`) и `CadJointType::build_generic_joint` (`joint.rs:102, 146`) для `Revolute` (`JointAxis::AngX`) и `Prismatic` (`JointAxis::LinX`) вызывался только `set_motor_velocity(axis, target_velocity, max_force)` (где 3-й аргумент в Rapier3D `0.22` задаёт коэффициент демпфирования `factor`, а не предельное усилие `max_force`), без вызова `set_motor_max_force(axis, max_force)`. В результате после `disable_joint_motor` (обнуляющего `max_force = 0.0`) повторный вызов `set_joint_motor` оставлял `max_force == 0.0`, и мотор не создавал усилия.
   - В `PhysicsWorld::set_joint_motor` и `disable_joint_motor` (`crates/mcp-cad-physics/src/simulator.rs:351–390`) изменение параметров мотора не пробуждало уснувшие тела (`rb.wake_up(true)`), из-за чего тела, перешедшие в состояние покоя (sleep) на предыдущих шагах симуляции, не приходили в движение при включении мотора.
   - В `create_joint` (`crates/mcp-cad-server/src/tools/physics.rs:195–201`) некорректно заданный параметр `limits` (не массив, массив длины $\ne 2$ или содержащий `NaN`/`null`/строку) молча игнорировался или заменялся на `0.0` вместо возврата структурированной ошибки `INVALID_ARGUMENT`.

---

## 2. Выполненные изменения

1. **Пропорциональное масштабирование зазоров и защита `safe_clamp` в `create_wheel_rim_with_settings` (`crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-core/src/dag.rs`, `src/cad/primitives.ts`)**:
   - Введены коэффициенты пропорционального масштабирования `s_rim = (d_rim / 0.35).min(1.0)`, `s_width = (w_rim / 0.16).min(1.0)` и `s_min = s_rim.min(s_width)`, а также безопасная обёртка `safe_clamp(val, lo, hi)` (возвращающая `val.clamp(lo, hi)` при `lo <= hi` и `(lo + hi) * 0.5` при `lo > hi`).
   - Для полноразмерных автомобильных дисков (`d_rim >= 0.35 м`, `w_rim >= 0.16 м`) `s_rim == 1.0` и `s_width == 1.0`, что побитово сохраняет существующую геометрию, а для малоразмерных моделей (`d_rim < 0.35 м`, вплоть до `5 мм`) все внутренние зазоры (`d_pcd`, `d_bolt`, `w_spoke`, `t_spoke`, `et`, `t_w`, `drop_depth`, `r_well`, `r_hump`, `r_hub_eff`, `r_bolt_ring`, `r_bolt_eff`, `t_disc`, `min_barrel_dz`, `barrel_pts`) масштабируются гомотетично с сохранением углов треугольников и топологии (`genus = 1 + n_bolts + n_spokes`, `validate_mesh == PASS`).
   - В `crates/mcp-cad-core/src/dag.rs` функция `default_bolt_hole_diameter()` изменена на `None`, чтобы `create_wheel_rim_with_settings` применял пропорциональный диаметр болтовых отверстий `0.014 * s_rim`.

2. **Исправление управления моторами `Prismatic` (`JointAxis::LinX`) и `Revolute` (`JointAxis::AngX`) и валидации `limits` (`crates/mcp-cad-physics/src/{joint.rs, simulator.rs}`, `crates/mcp-cad-server/src/{tools/physics.rs, error.rs}`, `src/cad/{kinematics.ts, mcp_dispatcher.ts}`)**:
   - В `CadJointType::build_generic_joint` и `CadJointType::apply_motor` (`crates/mcp-cad-physics/src/joint.rs`) добавлена явная установка `.motor_max_force(m.max_force as Real)` и `joint.set_motor_max_force(axis, motor.max_force as Real)` для `JointAxis::AngX` (`Revolute`) и `JointAxis::LinX` (`Prismatic`).
   - В `PhysicsWorld::set_joint_motor` и `PhysicsWorld::disable_joint_motor` (`crates/mcp-cad-physics/src/simulator.rs`) добавлено пробуждение обоих соединённых тел `body1` и `body2` (`rb.wake_up(true)`).
   - В `create_joint` (`crates/mcp-cad-server/src/tools/physics.rs`) добавлена строгая валидация параметра `limits` (проверка массива из 2 конечных чисел `[min, max]` с возвратом `ServerError::InvalidArgument` при нарушении формата и конвертацией `deg -> rad` для `revolute` и `length_unit -> m` для `prismatic`), а в `map_physics_error` (`crates/mcp-cad-server/src/error.rs`) добавлена привязка `.with_param("limits")` / `.with_param("axis")` для `INVALID_JOINT_CONFIGURATION`.

3. **Тестовое покрытие**:
   - В `crates/mcp-cad-core/src/primitives.rs`: добавлен unit-тест `test_wheel_rim_small_diameters_and_extreme_ratios_no_panic_and_valid` (`d_rim` от `5 мм` до `140 мм`, а также экстремальные отношения ширины к диаметру).
   - В `crates/mcp-cad-core/tests/primitive_invariants.rs`: добавлен тест `p2_t15_wheel_rim_small_diameters_no_clamp_panic_and_watertight` (включая проверку сериализации/десериализации `FeatureTree` с `AddWheelRim`).
   - В `crates/mcp-cad-physics/tests/physics_tests.rs`: добавлен тест `test_prismatic_joint_runtime_motor_and_disable_reenable` (проверка запуска мотора `JointAxis::LinX` после засыпания тела, остановки `disable_joint_motor` и реверса).
   - В `crates/mcp-cad-server/tests/mcp_contract_tests.rs`: добавлен контрактный тест `test_mcp_p2_t15_small_wheel_rim_and_prismatic_motor_and_revolute_limits`.
   - В `src/cad/cad_engine.test.ts`: добавлен тест `supports P2-T15 small wheel_rim (< 140 mm) without clamp panic and prismatic joint motor actuation`.

---

## 3. Новые найденные проблемы (добавлены в `TASKS.md`)

- **`P2-T21` [S]**:
  1. В `crates/mcp-cad-server/src/tools/cad.rs:1328–1376` инструмент `modify_feature` при модификации `FeatureOp::AddWheelRim` вызывает `check_scale_sanity(part.bounding_box().size(), &part.name, None)` (передавая `wheel_rim_diameter_m = None` вместо `Some(new_rim_diameter_m)`), из-за чего уменьшение `rim_diameter` ниже `150 мм` через `modify_feature` не генерирует предупреждение `WHEEL_RIM_SCALE` (в отличие от `add_feature_primitive` на строке `837`).
  2. В `crates/mcp-cad-server/src/tools/metrics.rs:427–451` инструмент `get_scene_summary` при формировании массива `joints` не включает установленные пределы перемещения/угла `limits` (с пересчётом из `rad` в `deg` для `Revolute` и из `m` в активный `length_unit` для `Prismatic`), хотя `limits` хранятся в `CadJointType`.
