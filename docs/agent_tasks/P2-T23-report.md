# Отчет по задаче P2-T23: Защита зарезервированных ID "ground"/"world" и устранение sliver на малом ободе

**Статус:** ✅ ВЫПОЛНЕНО  
**Ветка:** `agent/P2-T23`  
**Дата:** 08.11.2026 (завершено 09.11.2026)  
**Компоненты:** `crates/mcp-cad-server/src/tools/{cad.rs, physics.rs, project.rs}`, `crates/mcp-cad-physics/src/simulator.rs`, `crates/mcp-cad-core/src/primitives.rs`, `crates/mcp-cad-server/src/error.rs`

---

## 1. Контекст и выявленные проблемы

В ходе выполнения задач **P2-T18** и **P2-T15** были обнаружены следующие дефекты контракта и геометрического генератора:

1. **Незащищенность зарезервированных статических якорей `"ground"` и `"world"`:**
   - В `crates/mcp-cad-server/src/tools/cad.rs:315` инструмент `create_part` позволял создать деталь с `part_id: "ground"` или `"world"`.
   - В `crates/mcp-cad-server/src/tools/cad.rs:358` инструмент `delete_part` и в `crates/mcp-cad-physics/src/simulator.rs:327` метод `remove_part` позволяли удалить статический якорь мира, что приводило к разрушению физического окружения Rapier3D.
   - В `crates/mcp-cad-server/src/tools/physics.rs:135` (`create_joint`) проверка `is_world_or_ground` была регистронезависимой (`eq_ignore_ascii_case`), но при передаче `"part_a": "GROUND"` или `"World"` в `state.physics_world.add_joint` поиск производился по точному совпадению с `"ground"` / `"world"`, что приводило к ошибке `PartNotFound("GROUND")`.
   - В `crates/mcp-cad-server/src/tools/project.rs:259` (`load_project`) не проверялись зарезервированные имена деталей `"ground"` / `"world"`, а в `project_file.joints` имена не нормализовались к нижнему регистру.

2. **Дефект качества сетки малого обода (sliver triangles):**
   - В `crates/mcp-cad-core/tests/primitive_invariants.rs:1195` параметрический тест `p2_t15_wheel_rim_small_diameters_no_clamp_panic_and_watertight` на комбинации `d=0.14, w=0.06, bolts=3, spokes=3` падал с ошибкой:
     ```
     Small wheel rim d=0.14, w=0.06 failed validate_mesh: ["Sliver quality gate failed: 1.44% of triangles (36/2496) have min angle < 5° (threshold 1.00%)"]
     ```
   - Анализ показал, что в `create_wheel_rim_with_settings` (`crates/mcp-cad-core/src/primitives.rs:3751-3760`) в секции построения O-Grid болтовых отверстий углы отверстия рассчитывались по искусственной сетке `psi = -0.75 * PI + (j as f64) * 2.0 * PI / n_bolt`, не совпадающей с истинными радиальными направлениями от центра болта к вершинам сектора `perim_loop[2*j]`. Это вызывало угловой перекос серединного кольца `mid_loop` и порождало ровно 36 игловидных треугольников (по 6 на верхней и нижней плоскости для каждого из 3 болтов) с минимальными углами $2.67^\circ$ и $3.07^\circ$.

---

## 2. Что было сделано

### 2.1. Геометрическое ядро (`crates/mcp-cad-core/src/primitives.rs`)
- В `create_wheel_rim_with_settings` переработано построение болтовых отверстий: координаты точек `hole_loop[j]` теперь вычисляются строго вдоль истинного нормализованного радиального луча от центра болта к периметру:
  ```rust
  let dir_x = ox - bolt_center[0];
  let dir_y = oy - bolt_center[1];
  let d_len = dir_x.hypot(dir_y).max(1e-12);
  let u_x = dir_x / d_len;
  let u_y = dir_y / d_len;
  let ix = bolt_center[0] + r_bolt_eff * u_x;
  let iy = bolt_center[1] + r_bolt_eff * u_y;
  ```
  Это полностью устранило угловую деформацию между концентрическими кольцами переходной триангуляции и снизило количество треугольников $< 5^\circ$ до нуля.

### 2.2. Физический движок (`crates/mcp-cad-physics/src/simulator.rs`)
- В `PhysicsWorld::remove_part` запрещено удаление статических якорей `"ground"` и `"world"` (без учета регистра) с возвратом ошибки `PhysicsError::InvalidJointConfiguration("Cannot remove reserved static anchor ...")`.
- В `PhysicsWorld::add_part_with_options` и `update_part_collider` добавлен запрет на создание/модификацию деталей с зарезервированными идентификаторами.
- В `PhysicsWorld::add_joint`, `get_body`, `get_body_mut` и `has_part` внедрена автоматическая нормализация имен `"ground"` / `"world"` (регистронезависимо к нижнему регистру).

### 2.3. Сервер и MCP-инструменты (`crates/mcp-cad-server`)
- В `crates/mcp-cad-server/src/tools/cad.rs`:
  - `create_part`: запрещено создание деталей с `part_id` `"ground"` / `"world"` (возврат `INVALID_ARGUMENT`, `param: "part_id"`).
  - `delete_part`: запрещено удаление `"ground"` / `"world"` (возврат `INVALID_ARGUMENT`, `param: "part_id"`).
- В `crates/mcp-cad-server/src/tools/physics.rs`:
  - `create_joint`: нормализация `part_a` и `part_b` к нижнему регистру при совпадении с `"ground"` / `"world"` (любой регистр, например `"GROUND"`, `"World"`).
- В `crates/mcp-cad-server/src/tools/project.rs`:
  - `load_project`: валидация отсутствия деталей с зарезервированными ID `"ground"` / `"world"` и нормализация `part_a` / `part_b` в массиве `joints`.
- В `crates/mcp-cad-server/src/error.rs`:
  - Добавлена структурированная привязка `INVALID_ARGUMENT` (`param: "part_id"`) и информативные подсказки `hint` для зарезервированных статических якорей.

---

## 3. Верификация и тесты

1. **Unit-тесты примитивов (`crates/mcp-cad-core/tests/primitive_invariants.rs`):**
   - Тест `p2_t15_wheel_rim_small_diameters_no_clamp_panic_and_watertight` на комбинации `(0.14, 0.06, 3, 3)` проходит со статусом **`PASS`** (0 дефектов, 0 sliver-треугольников).
   - Все 25 параметрических тестов `primitive_invariants` успешно проходят (`25 passed; 0 failed`).

2. **Unit-тесты физики (`crates/mcp-cad-physics/tests/physics_tests.rs`):**
   - Добавлен тест `test_p2_t23_reserved_ground_world_and_normalization`:
     - Проверка защиты от удаления `"ground"`, `"world"`, `"GROUND"`, `"World"` через `remove_part`.
     - Проверка отказа регистрации деталей с именами `"ground"`, `"world"`.
     - Проверка нормализации регистра `"GROUND"` и `"World"` в `add_joint`.
   - Все 11 тестов `physics_tests` проходят (`11 passed; 0 failed`).

3. **Контрактные MCP-тесты (`crates/mcp-cad-server/tests/mcp_contract_tests.rs`):**
   - Добавлен сквозной тест `test_mcp_p2_t23_reserved_ids_and_ground_normalization`:
     - Проверка отклонения `create_part` и `delete_part` для `"ground"`, `"world"`, `"GROUND"`, `"World"`.
     - Создание кинематических шарниров с `"GROUND"` и `"World"`.
     - Проверка вывода нормализованных идентификаторов в `get_scene_summary`.
   - Тест проходит со статусом **`ok`**.

4. **Frontend и компиляция:**
   - `npx tsx --test src/cad/cad_engine.test.ts`: **29 passed, 0 failed (100% PASS)**.
   - `npm run lint` (`tsc --noEmit`): **0 errors**.
   - `compile_applet` & `lint_applet`: **Build succeeded, Linting completed successfully**.
