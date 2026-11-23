---
name: mcp-cad
description: >-
  Полное операционное руководство для ИИ-агента по работе с MCP-сервером
  MCP-CAD Engine (43 инструмента): параметрическое 3D-моделирование (FeatureTree DAG, CSG),
  топологическая верификация сеток, именованные грани, расчёт прочности МКЭ (TET4/TET10),
  кинематика и динамика механизмов (f64 DAE), сопряжённая нелинейная динамика (HHT-alpha,
  пластичность, контакт, разрушение), визуализация и экспорт STL/проектов.
---

# MCP-CAD: Руководство агента по работе с MCP-сервером

Этот документ — **единственная точка входа** для ИИ-агента, работающего с инструментами MCP-CAD. Все правила координат, единиц измерения, точные имена параметров, типовые сценарии и протокол восстановления после ошибок зафиксированы ниже.

> **КРИТИЧЕСКОЕ ПРАВИЛО**:
> **НЕ читайте исходный код Rust в `crates/**/*.rs`.** Вся спецификация инструментов, систем координат и контрактов задана в этом файле, в схемах `tools/list` и в [`docs/AGENT_GUIDE.md`](docs/AGENT_GUIDE.md) / [`docs/MCP_API_REFERENCE.md`](docs/MCP_API_REFERENCE.md).

---

## 1. Семь главных правил (прочитать перед первым вызовом)

1. **Единицы измерения — миллиметры (`mm`) по умолчанию**:
   - Все линейные размеры, радиусы, координаты, модули шестерён, шаги резьбы и габариты передаются и возвращаются в `mm` (можно переключить сессию через `set_units` или переопределить для одного вызова параметром `"length_unit": "mm" | "m"`).
   - **Исключение STL**: файлы `.stl` на диске (`export_stl`, `import_stl`) всегда хранят координаты вершин в **метрах СИ (`m`)**.
   - Физические единицы **не зависят** от `length_unit`: сила `N`, момент `N·m`, напряжение и давление `MPa`, модуль Юнга `GPa`, перемещение МКЭ `mm`, масса `kg`, плотность `kg/m³`, углы в CAD / шарнирах `deg`, угловая скорость мотора `rad/s`, линейная скорость `m/s`, время `s` / `ms`.
2. **Система координат — правая, Z-up, центр `[0, 0, 0]`**:
   - **Частая ошибка с `box`**: у `box` параметр `width` идёт вдоль **X**, `height` — вдоль **Y**, а `depth` — вдоль **Z** (`Z in [-depth/2, +depth/2]`). У осевых примитивов (`cylinder`, `cone`, `polygon_prism`) вдоль **Z** идёт именно `height` (`Z in [-height/2, +height/2]`).
3. **Семантика дерева операций (`FeatureTree` DAG)**:
   - Первый `add_feature_primitive` на детали создаёт базовое твёрдое тело.
   - Каждый **последующий** `add_feature_primitive` на том же `part_id` **автоматически объединяется через `csg_union`** — он **не перезаписывает** деталь.
   - `cut_hole` и `cut_helical_groove` **автоматически вычитаются через `csg_difference`**.
   - Между **двумя разными** деталями используйте `boolean_union`, `boolean_difference`, `boolean_intersection` (модифицируют `target_part_id` на месте).
4. **Не угадывайте координаты — используйте именованные грани**:
   - Перед расчётом МКЭ или привязкой вызовите `query_part_geometry` — получите реестр граней `faces` (`"left"`, `"right"`, `"cap_top"`, `"cap_bottom"`, `"side"`, `"hole_wall:feat_N"`, `face_id`) и задавайте закрепления/нагрузки по именам или фильтрам `where`.
5. **Ворота качества перед МКЭ и экспортом**:
   - После любых булевых операций, `cut_hole` и `cut_helical_groove` вызывайте `validate_mesh`.
   - Запускайте `run_fea_analysis` и `export_stl` только при `status == "PASS"` (`is_valid_solid: true`, `fea_ready: true`).
6. **Инвалидация МКЭ (`STALE`)**:
   - Любое изменение геометрии (`add_feature_primitive`, `cut_hole`, `modify_feature`, `transform_part`, `boolean_*`) увеличивает `part.revision` и переводит прежний отчёт МКЭ в статус `STALE` — пересчитайте МКЭ повторно.
   - `tessellate_part` меняет только плотность поверхностной сетки и **не сбрасывает** `revision` и статус МКЭ.
7. **Чтение ошибок (`isError: true`)**:
   - При ошибке инструмента `result.isError == true`, а `result.content[0].text` содержит JSON `{"error": {"code", "message", "param", "hint", "retryable", "details"}}`. Не повторяйте тот же вызов вслепую — исправьте поле `error.param` по инструкции `error.hint` и спискам из `error.details`.

---

## 2. Быстрая маршрутизация: какой инструмент выбрать

| Задача пользователя | Что вызывать (в правильном порядке) |
|---|---|
| Войти в существующую сцену / понять, что уже построено | `get_scene_summary` (компактная сводка) или `list_parts` |
| Построить типовой узел (фланцевая втулка, ступенчатый вал, зубчатая передача, винтовая пружина) | **`apply_recipe`** (`flanged_bushing` \| `stepped_shaft` \| `gear_transmission` \| `coil_spring`) — быстрее и надёжнее ручной сборки |
| Построить винтовую пружину / спираль из круглой проволоки | `add_feature_primitive` с `primitive_type: "helix"` (алиас `"coil_spring"`) или `apply_recipe` (`"coil_spring"`) — **не используйте** инверсию `cut_helical_groove` + `boolean_difference` |
| Построить деталь с нуля за 1 запрос | `batch_cad_operations` (`create_part` → `set_requirements` → `add_feature_primitive` → `cut_hole`) |
| Построить тело вращения (ступенчатый вал, фланец, чаша) | `add_feature_primitive` с `primitive_type: "revolve"` (лучше и чище, чем склеивать много цилиндров) |
| Проверить сетку и оптимизировать длинную историю DAG | `validate_mesh` → при длинном DAG `optimize_feature_tree` (`apply: true`) → `validate_mesh` |
| Статический расчёт прочности детали (напряжения Мизеса, FoS) | `validate_mesh` → `query_part_geometry` → `run_fea_analysis` → `get_fea_heatmap_image` |
| Проверить прочность зубьев прямозубой шестерни | `check_gear_strength` (по `part_id` или напрямую по `module`, `teeth`, `face_width`, `torque`) |
| Собрать и проверить подвижный механизм | `create_part` (`is_fixed: true` для станины) → `create_joint` → `analyze_mechanism` → `check_interference` / `sweep_joint` → `set_joint_motor` → `run_physics_simulation` |
| Прочность детали под реальными динамическими нагрузками из симуляции | `run_physics_simulation` → `get_part_load_cases` → `run_fea_from_simulation` |
| Нелинейная динамика: удар, пластичность, контакт, разрушение, осколки | `bake_coupled_simulation` → `query_simulation_frame` |
| Посмотреть на деталь глазами агента (в т.ч. headless) | `capture_viewport` (геометрия) / `get_fea_heatmap_image` (тепловая карта МКЭ) |
| Показать интерактивное 3D-окно пользователю на экране | `open_preview_window` |
| Сохранить / загрузить / выгрузить в производство | `save_project` / `load_project` / `export_stl` / `import_stl` |

---

## 3. Типовые сценарии (Playbooks) с точными JSON-примерами

### Сценарий A: Проектирование детали, проверка требований и статический МКЭ

**Порядок шагов**: `get_scene_summary` → `batch_cad_operations` → `validate_mesh` → `query_part_geometry` → `run_fea_analysis` → `check_requirements` → (если `FAIL` — `modify_feature` → повторить) → `capture_viewport` / `get_fea_heatmap_image` → `export_stl` / `save_project`.

**Шаг 1. Атомарно создать деталь, требования и базовую геометрию (`batch_cad_operations`)**:
> Важно: внутри `operations` каждый элемент имеет поля `"tool"` и `"params"`. Вложенный `batch_cad_operations` запрещён. При ошибке на любом шаге вся транзакция откатывается.

```json
{
  "operations": [
    {
      "tool": "create_part",
      "params": {
        "part_id": "bracket",
        "name": "Support Bracket",
        "material": "aluminum_6061_t6"
      }
    },
    {
      "tool": "set_requirements",
      "params": {
        "part_id": "bracket",
        "auto_check": true,
        "requirements": [
          { "id": "req_bbox", "type": "bbox_max", "dims": [120.0, 40.0, 30.0] },
          { "id": "req_mass", "type": "mass_max", "kg": 0.35 },
          { "id": "req_mesh", "type": "mesh_valid", "expected_genus": 1 },
          { "id": "req_fos",  "type": "min_safety_factor", "value": 2.0, "fea_case": "tip_load" }
        ]
      }
    },
    {
      "tool": "add_feature_primitive",
      "params": {
        "part_id": "bracket",
        "primitive_type": "box",
        "params": { "width": 100.0, "height": 20.0, "depth": 20.0 }
      }
    },
    {
      "tool": "cut_hole",
      "params": {
        "part_id": "bracket",
        "position": [30.0, 0.0, 0.0],
        "normal": [0.0, 0.0, 1.0],
        "radius": 5.0,
        "through": true
      }
    }
  ]
}
```

**Шаг 2. Проверить сетку и узнать имена граней**:
```json
// validate_mesh
{ "part_id": "bracket", "expected_genus": 1 }

// query_part_geometry
{ "part_id": "bracket", "where": { "kind": "plane" } }
```

**Шаг 3. Запустить МКЭ по именованным граням (`run_fea_analysis`)**:
> Для изгибных задач используйте `"element_order": 2` (квадратичные TET10) или `"accuracy_mode": "high"` — это устраняет сдвиговое запирание (shear locking) линейных TET4. Для оценки точности можно включить `"convergence_study": true`.

```json
{
  "part_id": "bracket",
  "fea_case": "tip_load",
  "mesh_density": "medium",
  "element_order": 2,
  "accuracy_mode": "standard",
  "fixed": [
    { "face": "left" }
  ],
  "loads": [
    { "face": "right", "force": [0.0, -500.0, 0.0] }
  ]
}
```
> В элементах `loads` поддерживаются три типа нагрузки на грань:
> - `"force": [Fx, Fy, Fz]` в `N` (распределяется по площади грани),
> - `"pressure": P` в `MPa` (положительное значение давит **внутрь** тела вдоль `-normal`),
> - `"moment": [Mx, My, Mz]` в `N·m` относительно центроида грани.

**Шаг 4. Проверить требования и визуализировать**:
```json
// check_requirements
{ "part_id": "bracket" }

// get_fea_heatmap_image
{ "part_id": "bracket", "camera_preset": "isometric", "width": 800, "height": 600 }
```
Если `min_safety_factor` в статусе `FAIL` — получите `feature_id` через `get_feature_tree`, измените сечение через `modify_feature`, проверьте `validate_mesh`, повторно запустите `run_fea_analysis` и `check_requirements`.

---

### Сценарий B: Кинематика механизма, проверка зазоров и МКЭ из симуляции

**Порядок шагов**:
1. Создайте неподвижную станину (`create_part` с `"is_fixed": true`) или используйте встроенный статический якорь `"ground"` / `"world"`.
   - **Запрещено** называть пользовательские детали `"ground"` или `"world"`.
2. Создайте подвижные звенья (`is_fixed: false`) и разместите их в начальном положении (`position` / `transform_part`).
3. Соедините детали через `create_joint` (`revolute`, `prismatic`, `spherical`, `fixed`, `gear`).
   - `anchor_a` и `anchor_b` — локальные точки на деталях A и B в `length_unit` (`mm`).
   - `limits`: для `revolute` в **градусах (`deg`)**, для `prismatic` в **`length_unit` (`mm`)**.
   - Для `gear` задайте `axis_a`, `axis_b` и либо `ratio`, либо `teeth_a` + `teeth_b`.
4. Проанализируйте кинематику и столкновения **до** запуска мотора:
   ```json
   // analyze_mechanism — подвижность (Грюблер + SVD Якобиана), избыточные связи, «висящие» детали
   {}

   // check_interference — статические пересечения и зазоры
   { "ignore_adjacent": true }

   // sweep_joint — сканирование хода шарнира без мутации сцены
   { "joint_id": "j_crank", "from": 0.0, "to": 180.0, "steps": 36, "ignore_adjacent": true }
   ```
5. Подключите привод и прогоните динамику:
   ```json
   // set_joint_motor (только для revolute или prismatic!)
   // revolute: target_velocity в rad/s, max_force в N*m
   // prismatic: target_velocity в m/s, max_force в N
   { "joint_id": "j_crank", "target_velocity": 6.283, "max_force": 50.0 }

   // run_physics_simulation
   { "duration_seconds": 1.0, "step_size_ms": 10.0 }
   ```
6. Передайте пиковые реакции шарниров и инерционные нагрузки в МКЭ:
   ```json
   // get_part_load_cases (опционально для инспекции сил)
   { "part_id": "crank_arm", "select": "peak_force" }

   // run_fea_from_simulation (автоматически ставит 3-2-1 опору инерционной разгрузки)
   { "part_id": "crank_arm", "select": "peak_force", "mesh_density": "medium", "element_order": 2 }
   ```

---

### Сценарий C: Сопряжённая нелинейная динамика (удар, пластичность, контакт, разрушение)

Используйте `bake_coupled_simulation` + `query_simulation_frame`, если нужна **нестационарная** динамика деформируемых тел (неявный `HHT-alpha`, упругопластичность J2, повреждаемость и эрозия элементов с образованием осколков, деформируемый контакт с трением Кулона).
- При `"use_physics_joints": true` (по умолчанию) решатель автоматически подхватывает активные шарниры и моторы из `PhysicsWorld`.

```json
// 1. Запечь симуляцию (async_mode: false ждёт завершения; true возвращает job_id сразу)
{
  "action": "bake",
  "accuracy_level": "standard",
  "part_ids": ["beam_1"],
  "duration_s": 0.02,
  "dt_ms": 1.0,
  "enable_plasticity": true,
  "enable_fracture": true,
  "enable_contact": true,
  "fixed_faces": [{ "part_id": "beam_1", "face": "min_x" }],
  "loads": [{ "part_id": "beam_1", "face": "max_x", "force": [0.0, -5000.0, 0.0] }],
  "async_mode": false
}

// 2. Получить сводку кадра без тяжёлых массивов вершин (или include_mesh_fields: true для полных полей)
{
  "frame_index": 10,
  "include_mesh_fields": false
}
```
> Для фонового расчёта (`async_mode: true`) опрашивайте статус через `{"action": "status", "job_id": "..."}` или отменяйте через `{"action": "cancel", "job_id": "..."}`.

---

### Сценарий D: Быстрая сборка типовых узлов (`apply_recipe`)

Вместо ручного конструирования типовых машиностроительных узлов вызывайте `apply_recipe` — он строит гарантированно водонепроницаемые детали и шарниры атомарно:
- `"flanged_bushing"` — фланцевая втулка с отверстиями под болты (`sleeve_outer_diameter`, `sleeve_length`, `bore_diameter`, `flange_diameter`, `flange_thickness`, `flange_position: "bottom"|"top"|"center"`, `pcd_diameter`, `bolt_holes_count`, `bolt_hole_diameter`).
- `"stepped_shaft"` — ступенчатый вал с внешним шпоночным пазом (`steps: [{diameter, length}, ...]`, `keyway_step_index`, `keyway_width`, `keyway_depth`, `keyway_length`).
- `"gear_transmission"` — цилиндрическая зубчатая передача на станине с двумя `revolute` и одним `gear` шарнирами + опциональным мотором (`module`, `teeth_pinion`, `teeth_gear`, `face_width`, `pressure_angle_deg`, `frame_thickness`, `frame_clearance`, `motor_velocity_rad_s`, `motor_max_torque_nm`).
- `"coil_spring"` — винтовая цилиндрическая пружина из круглой проволоки с расчётом инженерных параметров (индекс пружины $C$, коэффициент Валя $K_w$, жёсткость $k$ в Н/мм, сомкнутая высота $L_s$, длина проволоки $L_w$): `helix_radius` / `mean_diameter`, `wire_radius` / `wire_diameter`, `pitch`, `active_coils` / `free_length`, `is_right_hand`.

---

## 4. Примитивы 3D, оси координат и границы (`add_feature_primitive`)

Все примитивы в локальной системе центрированы в `[0, 0, 0]`. Для смещения и поворота любого примитива передавайте `"position": [x, y, z]` (в `mm`) и `"rotation_deg": [rx, ry, rz]` (в `deg`, углы Эйлера Roll-Pitch-Yaw) внутри `params` или на верхнем уровне аргументов `add_feature_primitive`.

| `primitive_type` | Диапазон по `Z` | Сечение в `XY` | Обязательные параметры `params` | Опциональные параметры и особенности |
|---|---|---|---|---|
| `box` | `[-depth/2, +depth/2]` | `X in [-width/2, +width/2]`<br>`Y in [-height/2, +height/2]` | `width`, `height`, `depth` | **Внимание**: вдоль Z идёт `depth`, а `height` — вдоль Y! |
| `cylinder` | `[-height/2, +height/2]` | `x² + y² <= radius²` | `radius`, `height` | `segments` (умолч. 32). Нижний торец `cap_bottom` при `Z = -height/2`, верхний `cap_top` при `Z = +height/2`. |
| `cone` | `[-height/2, +height/2]` | Линейный переход от `radius_bottom` к `radius_top` | `radius_bottom`, `height` | `radius_top` (для острой вершины `0.0`), `segments`. **Основание `radius_bottom` при `Z = -height/2`**, вершина `radius_top` при `Z = +height/2`. |
| `sphere` | `[-radius, +radius]` | `x² + y² + z² <= radius²` | `radius` | `sectors` (умолч. 32), `stacks` (умолч. 16). |
| `polygon_prism` | `[-height/2, +height/2]` | Правильный N-угольник | `height` + размер (`radius` \| `outer_radius` \| `width_across_flats`) | `sides` (умолч. 6 — шестигранник DIN 934), `chamfer` / `chamfer_radius` (коническая фаска 30° на обоих торцах). |
| `spur_gear` | `[-face_width/2, +face_width/2]` | Эвольвентный профиль (DIN 867 / ISO 53) | `module`, `teeth`, `face_width` | `pressure_angle_deg` (умолч. 20°), `bore_radius` / `bore_diameter`, `keyway_width`, `keyway_depth` (DIN 6885), `hub_diameter`, `hub_length`. |
| `revolve` | Определяется точками `z` в `profile` | Вращение 2D-контура `[[r, z], ...]` вокруг оси `Z` (`r >= 0`) | `profile` | `angle_deg` (умолч. 360°; при `< 360°` строит торцевые крышки), `segments`, `inner_radius` (для 2-точечной внешней образующей `[[r0, z0], [r1, z1]]` автоматически строит замкнутое кольцо). |
| `wheel_rim` | `[-rim_width/2, +rim_width/2]` | Ступенчатый обод, спицы, ступица, PCD | `rim_diameter`, `rim_width` | `flange_diameter`, `hub_diameter`, `hub_bore_diameter`, `pcd_diameter`, `bolt_holes_count` (5), `bolt_hole_diameter` (14 мм), `spokes_count` (5), `spoke_width`, `spoke_thickness`, `offset_et` (35 мм). **Масштаб**: используйте полноразмерный обод (`rim_diameter >= 150 mm`). |
| `helix` (алиас `coil_spring`) | `[-height/2, +height/2]` | Круглая проволока радиуса `wire_radius` по винтовой траектории радиуса `helix_radius` | `helix_radius` (или `coil_radius`/`mean_diameter`), `wire_radius` (или `wire_diameter`) + любые 2 из `pitch`, `height`/`free_length`, `turns`/`active_coils` | `starts` (умолч. 1), `is_right_hand` (умолч. `true`). Инварианты: `wire_radius < helix_radius` и `pitch / starts > 2 * wire_radius`. Именованные грани: `cap_bottom`, `cap_top`, `side`. |

---

## 5. Именованные грани и семантические селекторы

Инструмент `query_part_geometry` возвращает массив `faces[]`, где каждая грань содержит `id`, `name`, `aliases`, `role`, `kind` (`plane` \| `cylinder` \| `cone` \| `sphere` \| `torus` \| `helical` \| `other`), `area`, `centroid`, `sample_point`, `normal_or_axis`, `radius`.

### Стандартные имена граней базовых примитивов
- **`box`**: `top` (`+Z`, алиасы `cap_top`, `max_z`), `bottom` (`-Z`, `cap_bottom`, `min_z`), `right` (`+X`, `max_x`), `left` (`-X`, `min_x`), `back` (`+Y`, `max_y`), `front` (`-Y`, `min_y`).
- **`cylinder` / `cone` / `polygon_prism` / `helix`**: `cap_top` (`+Z`), `cap_bottom` (`-Z`), `side` (или `side_0..side_{n-1}` у призмы).
- **`cut_hole`**: цилиндрическая стенка отверстия получает роль `hole_wall` (в сложных деталях — `hole_wall:feat_N`, поддерживает шаблон `"hole_wall:*"`), дно глухого отверстия — `hole_bottom`, стенки шпоночного паза — `keyway_wall`.
- **`boolean_difference`**: поверхности реза получают префикс `cut:`.

### Как указывать селектор грани (в `query_part_geometry`, `run_fea_analysis`, `transform_part`)
- По имени/алиасу: `{"face": "cap_bottom"}` или с маской `{"face": "hole_wall:*"}`.
- По числовому ID из `query_part_geometry`: `{"face_id": 3}`.
- По нескольким именам: `{"faces": ["left", "bottom"]}`.
- По геометрическому фильтру `where`:
  ```json
  { "where": { "kind": "cylinder", "radius": 5.0, "tolerance": 0.2 } }
  ```
  Поддерживаемые ключи `where`: `kind`, `role`, `origin_feature`, `normal`, `axis`, `radius`, `tolerance`, `near`, `closest`, `min_area`, `max_area`.
- Если пришло `AMBIGUOUS_FACE_SELECTOR`, уточните имя через квалифицированный формат `feat_N:<role>` или `face_id` из `error.details.available_faces`.

---

## 6. Материалы (`create_part` и `define_material`)

Встроенные пресеты (10 материалов; поддерживаются и короткие алиасы):

| Канонический ID | Алиасы | E (GPa) | Предел текучести `σ_y` (MPa) | Плотность `ρ` (kg/m³) |
|---|---|---:|---:|---:|
| `aluminum_6061_t6` | `alu_6061`, `Al6061-T6`, `aluminum_6061` | 68.9 | 276 | 2700 |
| `steel_structural_aisi1020` | `steel_1020`, `steel`, `structural_steel` | 205.0 | 250 | 7850 |
| `steel_carbon_aisi1045` | `steel_1045`, `1045`, `carbon_steel_1045` | 205.0 | 350 | 7850 |
| `steel_high_strength_4140` | `steel_4140`, `AISI 4140`, `4140` | 210.0 | 655 | 7850 |
| `titanium_grade5_ti6al4v` | `ti_6al_4v`, `Ti-6Al-4V`, `titanium_ti6al4v` | 114.0 | 880 | 4430 |
| `brass_c360` | `brass_c36000`, `brass` | 100.0 | 310 | 8500 |
| `pla_filament_3dprint` | `pla_3d`, `pla_plastic`, `pla` | 3.5 | 60 | 1240 |
| `petg_filament_3dprint` | `petg_3d`, `petg_plastic`, `petg` | 2.1 | 50 | 1270 |
| `nylon_pa12` | `nylon_pa6`, `nylon` | 1.8 | 45 | 1010 |
| `borosilicate_glass` | `pyrex`, `glass`, `borosilicate` | 64.0 | 120 (хрупкий) | 2500 |

Для нестандартного материала зарегистрируйте его заранее через `define_material` (обязательны: `id`, `young_modulus_gpa`, `poisson_ratio` в `(0, 0.5)`, `yield_strength_mpa`, `density_kg_m3`; для нелинейного разрушения опционально `ultimate_strength_mpa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`, `failure_mode: "ductile"|"brittle"`).

---

## 7. Полный каталог всех 43 инструментов MCP

### 7.1. Сцена, единицы и проекты
| Инструмент | Ключевые параметры | Назначение |
|---|---|---|
| `get_scene_summary` | `length_unit?` | Компактная сводка всей сцены: детали, габариты, массы, валидность сеток, FEA (`STALE`/FoS), шарниры, требования. |
| `set_units` | `length_unit: "mm" \| "m"` | Переключает активную единицу длины сессии (по умолчанию `"mm"`). |
| `save_project` | `path`, `project_name?` | Сохраняет всю сцену (DAG, материалы, шарниры, требования, `length_unit`) в JSON (`schema_version: "1.0.0"`). |
| `load_project` | `path` | Транзакционно загружает JSON-проект и перестраивает геометрию и физику. |
| `get_simulation_metrics_json` | `length_unit?` | Полная JSON-телеметрия по деталям, физике и последним отчётам МКЭ. |

### 7.2. Детали и материалы
| Инструмент | Ключевые параметры | Назначение |
|---|---|---|
| `create_part` | `name`, `material`, `part_id?`, `is_fixed?` | Создаёт пустую деталь. `"ground"`/`"world"` запрещены. |
| `list_parts` | `length_unit?` | Список всех деталей с AABB, объёмом, массой, ревизией и числом треугольников. |
| `delete_part` | `part_id` | Удаляет деталь из сцены, физики, требований и кэша МКЭ. |
| `define_material` | `id`, `young_modulus_gpa`, `poisson_ratio`, `yield_strength_mpa`, `density_kg_m3`, ... | Регистрирует пользовательский материал в сессии. |

### 7.3. Параметрическое 3D-моделирование (DAG) и CSG
| Инструмент | Ключевые параметры | Назначение |
|---|---|---|
| `add_feature_primitive` | `part_id`, `primitive_type`, `params`, `position?`, `rotation_deg?`, `length_unit?` | Добавляет примитив в DAG (1-й — база, последующие — авто `csg_union`). |
| `cut_hole` | `part_id`, `position`, `normal` (или `direction`), `radius` (или `diameter`), `depth` или `through: true`, `keyway_width?`, `keyway_depth?` | Сверлит сквозное/глухое цилиндрическое отверстие (авто `csg_difference`), опционально со шпоночным пазом DIN 6885. |
| `cut_helical_groove` | `part_id`, `helix_radius`, `groove_radius`, `pitch`, `height`, `start_z?`, `starts?`, `is_right_hand?`, `profile_type?: "circular"\|"metric_60"` | Нарезает винтовые канавки (`circular`, умолч. `starts: 2`) или метрическую резьбу 60° (`metric_60`, умолч. `starts: 1`). Если `start_z` не задан, выравнивается по `bounding_box.min[2]`. |
| `modify_feature` | `part_id`, `feature_id`, `params`, `length_unit?` | Изменяет параметры существующего шага DAG с пересборкой сетки (инкрементирует `revision`). |
| `get_feature_tree` | `part_id`, `length_unit?` | Возвращает упорядоченную историю узлов DAG (`feature_id` для `modify_feature`). |
| `transform_part` | `part_id`, `translation?`, `rotation_deg?`, `scale?`, `pivot_face?`, `align_face?: {face, target_point}` | Сдвиг, поворот RPY, масштабирование или выравнивание по именованной грани. |
| `boolean_difference` | `target_part_id`, `tool_part_id`, `length_unit?` | Вычитает `tool_part_id` из `target_part_id`. |
| `boolean_union` | `target_part_id`, `tool_part_id`, `length_unit?` | Объединяет `tool_part_id` с `target_part_id`. |
| `boolean_intersection` | `target_part_id`, `tool_part_id`, `length_unit?` | Оставляет пересечение объёмов `target_part_id` и `tool_part_id`. |
| `batch_cad_operations` | `operations: [{tool, params}, ...]` | Атомарно выполняет цепочку операций с полным откатом при ошибке. |
| `apply_recipe` | `recipe: "flanged_bushing"\|"stepped_shaft"\|"gear_transmission"\|"coil_spring"`, `params?`, `length_unit?` | Атомарно строит типовой верифицированный узел/сборку. |

### 7.4. Топология, тесселяция и оптимизация
| Инструмент | Ключевые параметры | Назначение |
|---|---|---|
| `validate_mesh` | `part_id`, `expected_genus?`, `strict_slivers?` | Проверяет замкнутость (`is_watertight`), многообразие, объём, род `genus`. По умолчанию `strict_slivers: false` (микро-треугольники канавок идут в `warnings`, сохраняя `PASS` / `fea_ready: true`). |
| `query_part_geometry` | `part_id`, `face_selector?`, `where?`, `length_unit?` | Возвращает AABB, объём, площадь, центр масс и реестр именованных граней `faces`. |
| `tessellate_part` | `part_id`, `quality?: "coarse"\|"medium"\|"fine"`, `chord_tolerance?`, `angular_tolerance_deg?`, `max_edge_length?` | Перестраивает поверхностную сетку с нужной точностью (не сбрасывает `revision` и МКЭ). |
| `optimize_feature_tree` | `part_id`, `apply?: true` | Стоимостной оптимизатор DAG (правила R1–R5, отсечение мёртвых операций, слияние соосных тел в `revolve`) с проверкой сохранения объёма (`<= 1%`). |

### 7.5. Инженерные требования (Requirements Layer)
| Инструмент | Ключевые параметры | Назначение |
|---|---|---|
| `set_requirements` | `requirements: [{id?, type, ...}]`, `part_id?`, `scope?: "part"\|"scene"`, `auto_check?`, `length_unit?` | Задаёт проверяемые ограничения: `bbox_max` (`dims`), `bbox_min` (`dims`), `mass_max` (`kg`), `mesh_valid` (`expected_genus?`), `min_safety_factor` (`value`, `fea_case?`), `no_interference` (`pairs?`), `mechanism_dof` (`expected`). |
| `check_requirements` | `part_id?`, `length_unit?` | Возвращает `overall_status` (`PASS`\|`FAIL`\|`STALE`), измеренные значения, запасы и подсказки. |

### 7.6. Кинематика, интерференция и динамика механизмов
| Инструмент | Ключевые параметры | Назначение |
|---|---|---|
| `create_joint` | `joint_type: "revolute"\|"prismatic"\|"spherical"\|"fixed"\|"gear"`, `part_a`, `part_b`, `joint_id?`, `anchor_a?`, `anchor_b?`, `axis?`, `axis_a?`, `axis_b?`, `ratio?`, `teeth_a?`, `teeth_b?`, `limits?` | Создаёт кинематическую связь в `f64` DAE-мире (якоря `"ground"`/`"world"` для станины). |
| `set_joint_motor` | `joint_id`, `target_velocity`, `max_force` | Включает привод скорости на `revolute` (`rad/s`, `N·m`) или `prismatic` (`m/s`, `N`). |
| `run_physics_simulation` | `duration_seconds`, `step_size_ms?` | Шагает динамику твёрдых тел, записывая траектории, реакции шарниров и столкновения. |
| `analyze_mechanism` | `part_ids?`, `rank_tolerance?`, `length_unit?` | Вычисляет подвижность по Грюблеру и SVD Якобиана (`dof_numeric`), находит `redundant_joint_ids`, `joints_at_limits` и `floating_parts`. |
| `check_interference` | `part_ids?`, `ignore_adjacent?`, `pairs?`, `length_unit?` | Попарная проверка пересечений (`intersection_volume`, `penetration_depth`) и точных зазоров (`clearance`). |
| `sweep_joint` | `joint_id`, `from`, `to`, `steps`, `ignore_adjacent?`, `angle_unit?: "deg"\|"rad"`, `length_unit?` | Пошагово прокручивает/сдвигает шарнир (включая замкнутые контуры) без мутации сцены; находит `first_collision_value`, `lockup_value`, `safe_travel_range`, `min_clearance`. |
| `check_gear_strength` | `part_id?` или (`module`, `teeth`, `face_width`), `torque` (или `power_watts`/`power_kw` + `speed_rpm`), `material?`, `mating_teeth?`, `manufacturing_method?` | Аналитический расчёт зубьев на изгиб (Льюис + Барт `Kv`) и контактную выносливость (Герц / AGMA / ISO 6336). |
| `get_part_load_cases` | `part_id`, `select?: "peak_force"\|"peak_torque"\|<time_s>`, `length_unit?` | Извлекает локальные реакции шарниров и инерционные ускорения из последней симуляции. |

### 7.7. МКЭ-анализ прочности и сопряжённая нелинейная динамика
| Инструмент | Ключевые параметры | Назначение |
|---|---|---|
| `run_fea_analysis` | `part_id`, `fixed` / `fixed_faces` / `fixed_plane`, `loads` / `applied_force` / `applied_forces`, `mesh_density?: "coarse"\|"medium"\|"fine"`, `element_order?: 1\|2`, `accuracy_mode?: "fast"\|"standard"\|"high"`, `convergence_study?`, `levels?`, `max_seconds?`, `fea_case?` | Линейно-упругий 3D МКЭ (`TET4`/`TET10`), напряжения по Мизесу (`MPa`), перемещения (`mm`), `safety_factor`, опциональная проверка сходимости сетки. |
| `run_fea_from_simulation` | `part_id`, `select?`, `load_radius?`, `mesh_density?`, `element_order?` (умолч. `2`), `accuracy_mode?`, `convergence_study?`, `fea_case?`, `joint_faces?` | Квазистатический МКЭ по нагрузкам последней физической симуляции с автоматической 3-2-1 опорой инерционной разгрузки. |
| `bake_coupled_simulation` | `action?: "bake"\|"status"\|"cancel"`, `job_id?`, `accuracy_level?: "standard"\|"high"\|"ultra"`, `part_ids?`, `duration_s?`, `dt_ms?`, `hht_alpha?`, `enable_plasticity?`, `enable_fracture?`, `enable_contact?`, `use_physics_joints?`, `fixed_faces?`, `loads?`, `initial_velocities?`, `force_curve?`, `gravity?`, `async_mode?` | Серверное запекание неявной динамики `HHT-alpha` с пластичностью, контактом и разрушением. |
| `query_simulation_frame` | `simulation_id?`, `frame_index?`, `time_s?`, `part_id?`, `include_mesh_fields?`, `include_fragment_meshes?`, `length_unit?` | Возвращает кадр запечённой симуляции: деформированные узлы, `σ_vM`, пластическую деформацию, повреждаемость `D`, контактные силы, осколки и баланс энергий. |

### 7.8. Визуализация и файловый обмен (STL)
| Инструмент | Ключевые параметры | Назначение |
|---|---|---|
| `capture_viewport` | `part_id?`, `camera_preset?: "isometric"\|"iso"\|"front"\|"top"\|"right"`, `debug_mode?: "none"\|"wireframe"\|"normals"\|"backfaces"\|"boundary_edges"\|"flat_shading"`, `width?`, `height?` | Оффскрин 3D-рендер в PNG/Base64 для агента (работает в headless-средах и CI). |
| `get_fea_heatmap_image` | `part_id`, `camera_preset?`, `width?`, `height?` | Оффскрин 3D-карта напряжений по Мизесу в PNG/Base64 (вызывать после `run_fea_analysis`). |
| `open_preview_window` | `part_id?` | Открывает нативное интерактивное 3D-окно WGPU для пользователя (в `stdio`-режиме без `--gui` автоматически экспортирует временный `./output/_mcp_preview_live.stl` и запускает `mcp-cad-server --stl <path>`). |
| `export_stl` | `part_id`, `filepath`, `binary?`, `quality?: "coarse"\|"medium"\|"fine"`, `chord_tolerance?` | Экспортирует проверенную сетку в STL (**вершины в файле всегда в метрах `m`**; блокируется при невалидной сетке). |
| `import_stl` | `filepath` (или `file_path`), `part_id?`, `name?`, `material?`, `is_fixed?` | Импортирует STL с диска (**координаты файла трактуются как метры `m`**). После импорта проверьте габарит и вызовите `validate_mesh`. |

---

## 8. Типичные ловушки и протокол обработки ошибок

### Частые ловушки агентов
- **Перепутаны оси `box`**: `depth` — это ось **Z**, а `height` — ось **Y**. Чтобы поставить цилиндр сверху на `box`, его нижний торец должен быть на `Z = +box.depth / 2` (центр цилиндра на `Z = box.depth / 2 + cyl.height / 2`).
- **Повторный `add_feature_primitive` не стирает тело**: чтобы заменить размер примитива, используйте `modify_feature` (узнав `feature_id` через `get_feature_tree`), а не добавляйте новый примитив поверх.
- **Сравнение масштаба после `import_stl`**: если внешний STL был сохранён в миллиметрах (например, гайка `16×16×8`), движок прочтёт его как `16×16×8` **метров** и выдаст предупреждение `SCALE_TOO_LARGE` — уменьшите деталь через `transform_part` с `"scale": [0.001, 0.001, 0.001]`.
- **Тепловая карта МКЭ**: `capture_viewport` не имеет режима `fea_stress` — для визуализации напряжений всегда вызывайте `get_fea_heatmap_image`.
- **Запрос тяжёлых полей кадров**: в `query_simulation_frame` передавайте `"include_mesh_fields": false`, если вам нужны только пиковые напряжения, повреждаемость, число осколков и баланс энергии (иначе ответ будет огромным).

### Как реагировать на ошибки (`result.isError: true`)
| `error.code` | Причина | Действие агента |
|---|---|---|
| `PART_NOT_FOUND` | Неверный `part_id` | Взять актуальный ID из `error.details.available_parts` или вызвать `list_parts`. |
| `UNKNOWN_MATERIAL` | Неизвестный материал | Выбрать ID из таблицы §6 (`error.details.builtin_materials`) или создать через `define_material`. |
| `FACE_NOT_FOUND` / `AMBIGUOUS_FACE_SELECTOR` | Неточное или неоднозначное имя грани | Вызвать `query_part_geometry` или взять точный `name` / `face_id` из `error.details.available_faces`. |
| `MESH_INVALID` | Сетка не прошла проверку | Вызвать `validate_mesh`, проверить пересечения/тонкие стенки, скорректировать геометрию (или упростить через `revolve` / `optimize_feature_tree`). |
| `FEA_UNCONSTRAINED` / `FEA_NO_LOADS` / `FEA_NO_BOUNDARY_NODES` | Не заданы опоры, нагрузки или селектор не захватил узлы | Задать `fixed` и `loads` по реальным граням из `query_part_geometry`; для тонкой детали увеличить `mesh_density` до `"medium"`/`"fine"`. |
| `CSG_BOOLEAN_ERROR` | Вырожденный копланарный контакт при булевой операции | Сдвинуть инструмент на `0.5..1 mm` за пределы грани (для отверстий использовать `cut_hole` с `"through": true`). |
| `INVALID_JOINT_CONFIGURATION` | Нулевая ось, `min > max` в `limits` или мотор на `fixed`/`spherical`/`gear` | Проверить ненулевой вектор `axis` и ставить `set_joint_motor` только на `revolute` или `prismatic`. |
| `SIMULATION_NOT_FOUND` | Вызван `query_simulation_frame` без запечённой симуляции | Сначала выполнить `bake_coupled_simulation`. |
