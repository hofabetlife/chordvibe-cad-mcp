# MCP-CAD Engine: Справочник API инструментов (Tools Reference)

Спецификация протокола: **Model Context Protocol (MCP)**, версия `2024-11-05`.  
Формат сообщений: **JSON-RPC 2.0**.

---

## Оглавление

1. [Общие положения](#общие-положения)
   - [Ошибки (Контракт ошибок MCP и JSON-RPC)](#ошибки)
2. [Группа 1: Управление деталями и материалами](#группа-1-управление-деталями-и-материалами)
   - [1. `create_part`](#1-create_part)
   - [2. `list_parts`](#2-list_parts)
   - [3. `delete_part`](#3-delete_part)
3. [Группа 2: Параметрическое DAG-моделирование и тесселяция](#группа-2-параметрическое-dag-моделирование-и-тесселяция)
   - [4. `add_feature_primitive`](#4-add_feature_primitive)
   - [5. `cut_hole`](#5-cut_hole)
   - [6. `modify_feature`](#6-modify_feature)
   - [7. `get_feature_tree`](#7-get_feature_tree)
   - [8. `cut_helical_groove`](#8-cut_helical_groove)
   - [9. `boolean_difference`](#9-boolean_difference)
   - [10. `tessellate_part`](#10-tessellate_part)
4. [Группа 3: Кинематика и физика механизмов](#группа-3-кинематика-и-физика-механизмов)
   - [11. `create_joint`](#11-create_joint)
   - [12. `set_joint_motor`](#12-set_joint_motor)
   - [13. `run_physics_simulation`](#13-run_physics_simulation)
5. [Группа 4: Конечно-элементный анализ прочности (FEA)](#группа-4-конечно-элементный-анализ-прочности-fea)
   - [14. `run_fea_analysis`](#14-run_fea_analysis)
   - [15. `get_fea_heatmap_image`](#15-get_fea_heatmap_image)
6. [Группа 5: Визуализация, экспорт и импорт](#группа-5-визуализация-экспорт-и-импорт)
   - [16. `capture_viewport`](#16-capture_viewport)
   - [17. `open_preview_window`](#17-open_preview_window)
   - [18. `import_stl`](#18-import_stl)
   - [19. `export_stl`](#19-export_stl)
   - [20. `get_simulation_metrics_json`](#20-get_simulation_metrics_json)
7. [Группа 6: Топология, CSG операции, кастомные материалы и стоимостная оптимизация DAG](#группа-6-топология-csg-операции-кастомные-материалы-и-пакетная-оптимизация)
   - [21. `query_part_geometry`](#21-query_part_geometry)
   - [22. `boolean_union`](#22-boolean_union)
   - [23. `boolean_intersection`](#23-boolean_intersection)
   - [24. `transform_part`](#24-transform_part)
   - [25. `define_material`](#25-define_material)
   - [26. `batch_cad_operations`](#26-batch_cad_operations)
   - [27. `validate_mesh`](#27-validate_mesh)
   - [28. `optimize_feature_tree`](#28-optimize_feature_tree)
   - [29. `set_units`](#29-set_units)
8. [Группа 7: Персистентность и управление проектами](#группа-7-персистентность-и-управление-проектами)
   - [30. `save_project`](#30-save_project)
   - [31. `load_project`](#31-load_project)
9. [Группа 8: Слой инженерных требований и сводка сцены (Requirements & Scene Summary)](#группа-8-слой-инженерных-требований-requirements-layer)
   - [32. `set_requirements`](#32-set_requirements)
   - [33. `check_requirements`](#33-check_requirements)
   - [34. `get_scene_summary`](#34-get_scene_summary)

---

## Общие положения

Все вызовы инструментов осуществляются через метод JSON-RPC `tools/call`.

### Стандартный конверт запроса:
```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "tools/call",
  "params": {
    "name": "<TOOL_NAME>",
    "arguments": {
      "<PARAM_1>": "<VALUE_1>"
    }
  }
}
```

### Стандартный конверт ответа:
```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"status\": \"...\"\n}"
      }
    ]
  }
}
```

Для визуальных инструментов (`capture_viewport`, `get_fea_heatmap_image`) массив `content` содержит два элемента: текстовый JSON с метаданными и изображение в формате `image/png`:
```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"camera_preset\": \"Isometric\"\n}"
      },
      {
        "type": "image",
        "data": "iVBORw0KGgoAAAANSUhEUgAA...",
        "mimeType": "image/png"
      }
    ]
  }
}
```

### Контракт единиц измерения (`[DECISION-UNITS-1]`)

В соответствии с [`docs/UNITS.md`](UNITS.md):
- **Внутри вычислительных ядер (`mcp-cad-core`, `mcp-cad-fea`, `mcp-cad-physics`)** строго используется система **СИ**: длина — метры (`m`), площадь — `m²`, объём — `m³`, масса — `kg`, плотность — `kg/m³`, сила — `N`, момент — `N·m`, напряжение и модуль упругости — паскали (`Pa`), время — секунды (`s`), углы в динамике — радианы (`rad`).
- **На границе MCP-сервера (`crates/mcp-cad-server/src/tools/*.rs`)** действует настройка единицы длины сессии `EngineState::length_unit`:
  - **По умолчанию `length_unit = "mm"` (миллиметры)** для всех линейных входов и выходов (`width`, `height`, `depth`, `radius`, `module`, `pitch`, `chamfer`, `position`, `translation`, `anchor_a`, `anchor_b`, `fixed_plane.origin`, `fixed_plane.tolerance`, `applied_force.position`, `chord_tolerance`, `max_edge_length`, `bounding_box`, `dimensions`, `center_of_mass`).
  - Площади (`surface_area`, `detected_planar_faces[].area`) возвращаются в `mm²` (или `m²` при `length_unit = "m"`), объёмы (`volume`) — в `mm³` (или `m³`), при этом дополнительно дублируются эталонные СИ-поля `volume_m3`, `surface_area_m2`, `mass_kg`.
  - Физические величины, не зависящие от переключения `mm` ↔ `m`: сила — `N`, крутящий момент — `N·m`, напряжение МКЭ — `MPa` (`max_von_mises_mpa`, `yield_strength_mpa`), модуль Юнга — `GPa` (`young_modulus_gpa`), перемещение МКЭ — `mm` (`max_displacement_mm`), масса — `kg`, плотность — `kg/m³`, углы в CAD и вращательных шарнирах — `deg` (`limits` вращательного шарнира конвертируются из градусов в радианы на границе сервера; опционально поддерживается `limits_unit: "deg" | "rad"`), угловая скорость — `rad/s`, время — `s` (`duration_seconds`) / `ms` (`step_size_ms`, `computation_time_ms`).
  - Переключение единицы длины сессии выполняется инструментом **`set_units`** (`{"length_unit": "mm" | "m"}`), либо локально для одного вызова передачей опционального поля `"length_unit": "mm" | "m"` в аргументах инструмента.
  - Все инструменты, возвращающие численные/геометрические данные (а также ответ `initialize`), включают поле `"length_unit"` и объект `"units"`, а геометрические мутации (`add_feature_primitive`, `cut_hole`, `modify_feature`, `cut_helical_groove`, `transform_part`, `import_stl`) возвращают массив `"warnings": []` с проверкой адекватности масштаба (`SCALE_TOO_LARGE` при $> 10\text{ м}$, `SCALE_TOO_SMALL` при $< 0.1\text{ мм}$, `SCALE_EXTREME_VOLUME`, `WHEEL_RIM_SCALE` при диаметре обода $< 150\text{ мм}`).
  - **Формат файлов STL (`export_stl`, `import_stl`)**: координаты вершин внутри самих файлов `.stl` на диске записываются и читаются во внутренних единицах ядра — **метрах (`m`)** (`"stl_vertex_unit": "m"`), тогда как параметры `chord_tolerance` и `bounding_box` в JSON-ответах MCP масштабируются согласно активному `length_unit`.

### Ошибки

В соответствии со спецификацией **Model Context Protocol (MCP `2024-11-05`)** и реализацией сервера (`crates/mcp-cad-server/src/protocol.rs`), ошибки строго разделяются на два уровня: **ошибки выполнения инструмента** и **протокольные ошибки JSON-RPC 2.0**.

#### 1. Ошибки выполнения инструмента (`result.isError = true`, контракт P2-T3)

Любая ошибка, возникшая внутри диспетчера `dispatch_tool` (`ServerError`), считается **ошибкой выполнения инструмента**.

Для таких ошибок сервер возвращает **успешный JSON-RPC конверт** (верхнеуровневое поле `"error"` отсутствует, присутствует `"result"`), где в объекте `result` установлен флаг `"isError": true`, а массив `content` содержит структурированный JSON-объект в строке `text` (`ErrorEnvelope`):

```json
{
  "jsonrpc": "2.0",
  "id": 971,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\"error\":{\"code\":\"INVALID_ARGUMENT\",\"message\":\"Invalid argument: Revolve profile r-coordinate cannot be negative: -10\",\"param\":\"profile\",\"hint\":\"All radial coordinates r in revolve profile must be >= 0 because the axis of rotation is at r=0.\",\"retryable\":false,\"details\":{}}}"
      }
    ],
    "isError": true
  }
}
```

##### Структура тела ошибки (`ErrorBody`):
| Поле | Тип | Описание |
|---|---|---|
| `code` | `string` | Стабильный машиночитаемый код ошибки (см. реестр ниже) |
| `message` | `string` | Человекочитаемое сообщение об ошибке (сохранено для обратной совместимости) |
| `param` | `string \| null` | Имя невалидного или пропущенного параметра аргументов инструмента |
| `hint` | `string \| null` | Действенное указание следующего шага для ИИ-агента или инженера |
| `retryable` | `boolean` | `true`, если повторный вызов (после коррекции шага/допуска) может увенчаться успехом |
| `details` | `object` | Контекстные данные (доступные детали, поддерживаемые типы примитивов, параметры решателя) |

##### Пример ошибки `PART_NOT_FOUND` с перечислением доступных деталей:
```json
{
  "error": {
    "code": "PART_NOT_FOUND",
    "message": "Part not found: shaft_pin",
    "param": "part_id",
    "hint": "Part 'shaft_pin' was not found. Available parts: [part-19ea129a, part-d9d1370f]. Inspect parts via 'list_parts' or create it with 'create_part'.",
    "retryable": false,
    "details": {
      "part_id": "shaft_pin",
      "available_parts": ["part-19ea129a", "part-d9d1370f"]
    }
  }
}
```

##### Реестр стабильных кодов ошибок (`code`):
| Код ошибки | Домен | Описание | `param` | `retryable` |
|---|---|---|---|---|
| `PART_NOT_FOUND` | Сцена / Сервер | Деталь с указанным `part_id` не найдена в сессии | `part_id` | `false` |
| `FEATURE_NOT_FOUND` | CAD / DAG | Фича с `feature_id` отсутствует в истории детали | `feature_id` | `false` |
| `JOINT_NOT_FOUND` | Физика | Шарнир с `joint_id` не найден в `physics_world` | `joint_id` | `false` |
| `TOOL_NOT_FOUND` | MCP | Запрошенный инструмент не зарегистрирован на сервере | `name` | `false` |
| `INVALID_ARGUMENT` | Валидация | Некорректный аргумент вызова (выход из диапазона, нечисловое значение) | Имя параметра | `false` |
| `INVALID_PARAMETER` | CAD | Недопустимый геометрический параметр операции | Имя параметра | `false` |
| `INVALID_PRIMITIVE_TYPE` | CAD | Неизвестный тип примитива (возвращает `supported_primitives`) | `primitive_type` | `false` |
| `UNKNOWN_MATERIAL` | Материалы | Неизвестный материал (возвращает `builtin_materials`) | `material` | `false` |
| `PART_ALREADY_EXISTS` | Сцена | Попытка создать деталь с уже занятым идентификатором/именем | `name` / `part_id` | `false` |
| `JOINT_ALREADY_EXISTS` | Физика | Шарнир с данным идентификатором уже существует | `joint_id` | `false` |
| `EMPTY_GEOMETRY` | CAD / Сцена | Деталь не содержит геометрии (пустая полигональная сетка) | `part_id` | `false` |
| `CSG_BOOLEAN_ERROR` | CAD / CSG | Сбой булевой операции BSP (самопересечения, открытые рёбра) | `null` | `false` |
| `CAD_EVALUATION_ERROR` | CAD / DAG | Ошибка пересчёта параметрического дерева фич детали | `null` | `false` |
| `MESH_INVALID` | Топология | Сетка не прошла валидацию `validate_mesh` (не-многообразие, дыры) | `null` | `false` |
| `FEA_EMPTY_MESH` | МКЭ | Тетраэдрическая сетка МКЭ пуста (0 элементов) | `part_id` | `false` |
| `FEA_DEGENERATE_ELEMENT` | МКЭ | Тетраэдр сетки имеет нулевой или отрицательный объём | `null` | `false` |
| `FEA_INVALID_MATERIAL` | МКЭ | Физически невалидные свойства материала ($E \le 0$ или $\nu \notin (0, 0.5)$) | `material` | `false` |
| `FEA_UNCONSTRAINED` | МКЭ | Отсутствуют граничные условия Дирихле (свободное твёрдое тело) | `fixed_faces` | `false` |
| `FEA_NO_LOADS` | МКЭ | Не задана ни одна приложенная сила или давление | `applied_forces` | `false` |
| `FEA_NO_BOUNDARY_NODES` | МКЭ | Селектор грани или плоскости не захватил ни одного узла тетра-сетки | `fixed_faces` | `false` |
| `FEA_NODE_OUT_OF_BOUNDS` | МКЭ | Индекс узла МКЭ выходит за границы сетки | `null` | `false` |
| `FEA_SOLVER_CONVERGENCE_FAILED` | МКЭ | Итерационный PCG-решатель МКЭ не сошёлся за заданное число итераций | `null` | `true` |
| `FEA_MESH_GENERATION_FAILED` | МКЭ | Сбой воксельной тетраэдризации геометрии детали | `null` | `false` |
| `PHYSICS_INVALID_GEOMETRY` | Физика | Невалидная или вырожденная геометрия для физического твердого тела | `part_id` | `false` |
| `PHYSICS_COLLIDER_FAILED` | Физика | Ошибка построения коллидера твердого тела в Rapier | `part_id` | `false` |
| `INVALID_JOINT_CONFIGURATION` | Физика | Некорректная кинематическая конфигурация сустава/шарнира | `null` | `false` |
| `PHYSICS_SIMULATION_ERROR` | Физика | Сбой численного шага физической симуляции твердых тел | `null` | `true` |
| `VIEWER_EMPTY_MESH` | Визуализация | Попытка отрендерить деталь без полигональной сетки | `part_id` | `false` |
| `INVALID_DIMENSIONS` | Визуализация | Недопустимые размеры окна/изображения рендера ($W, H \le 0$) | `dimensions` | `false` |
| `STRESS_SCALAR_MISMATCH` | Визуализация | Число скаляров напряжений МКЭ не совпадает с числом вершин сетки | `null` | `false` |
| `GPU_ADAPTER_UNAVAILABLE` | Визуализация | Отсутствует подходящий графический адаптер GPU/wgpu | `null` | `false` |
| `GPU_DEVICE_ERROR` | Визуализация | Сбой инициализации логического устройства GPU | `null` | `false` |
| `IO_ERROR` | Система | Ошибка файлового ввода-вывода при импорте/экспорте | `null` | `false` |
| `JSON_ERROR` | Протокол | Ошибка сериализации или десериализации JSON | `null` | `false` |
| `PROJECT_SCHEMA_MISMATCH` | Проект / Схема | Несовместимая версия схемы проекта (поддерживается `1.0.0`) | `schema_version` | `false` |
| `FACE_NOT_FOUND` | Топология / Грани | Именованная грань или фильтр `face_selector` не совпали ни с одной гранью детали | `face_selector` | `false` |
| `AMBIGUOUS_FACE_SELECTOR` | Топология / Грани | Неквалифицированное имя/алиас совпало более чем с одной гранью (требуется `feat_N:<role>` или `face_id`) | `face_selector` | `false` |
| `INTERNAL_ERROR` | Движок | Непредвиденная внутренняя ошибка ядра сервера | `null` | `false` |

#### 2. Протокольные ошибки JSON-RPC 2.0 (верхнеуровневое поле `"error"`)

Верхнеуровневое поле `"error"` (объект `JsonRpcError { code, message, data }`, при отсутствии поля `"result"`) возвращается только при нарушениях протокола JSON-RPC 2.0 до запуска инструмента:

| Код JSON-RPC | Константа / Конструктор | Условие возникновения |
|---|---|---|
| `-32700` | `JsonRpcError::parse_error` | Входная строка не является синтаксически валидным JSON (`handle_message`) |
| `-32600` | `JsonRpcError::invalid_request` | Некорректная структура конверта JSON-RPC 2.0 |
| `-32601` | `JsonRpcError::method_not_found` | Неизвестный метод JSON-RPC (метод отличен от `"initialize"`, `"notifications/initialized"`, `"ping"`, `"tools/list"`, `"tools/call"`) |
| `-32602` | `JsonRpcError::invalid_params` | В запросе `"tools/call"` отсутствует обязательное строковое поле `params.name` (`"Missing tool 'name' in arguments"`) |
| `-32603` | `JsonRpcError::internal_error` | Внутренняя ошибка сериализации структуры ответа (`serde_json::to_value`) |

Пример протокольной ошибки:
```json
{
  "jsonrpc": "2.0",
  "id": 99,
  "error": {
    "code": -32601,
    "message": "Unknown MCP method: 'tools/unknown'"
  }
}
```

> **Примечание о неизвестном имени инструмента (`ServerError::ToolNotFound`):**  
> В текущей реализации вызов метода `"tools/call"` с несуществующим именем инструмента в `params.name` обрабатывается внутри `dispatch_tool` как `Err(ServerError::ToolNotFound(name))` и возвращается через `result.isError = true` (`"Tool not found: <name>"`), а не через протокольную ошибку JSON-RPC (`-32601` / `-32602`).

---

## Группа 1: Управление деталями и материалами

### 1. `create_part`

Создает новую пустую деталь и прикрепляет к ней инженерный материал из реестра.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `name` | `string` | Да | Человекочитаемое имя детали (например, `"mounting_bracket"`). Идентификаторы `"ground"` и `"world"` (в любом регистре) зарезервированы под статический якорь сцены/физики и запрещены к использованию (возвращают `INVALID_ARGUMENT`) |
| `material` | `string` | Да | Идентификатор материала из базы пресетов |
| `is_fixed` | `boolean` | Нет | Закрепить деталь неподвижно в мировом пространстве (статическое тело с бесконечной массой для физических расчетов и кинематики, например станина или корпус редуктора). По умолчанию `false` |

**Доступные пресеты материалов (и поддерживаемые канонические алиасы P2-T13)**:
- `aluminum_6061_t6` (алиасы: `alu_6061`, `Al6061-T6`, `Al-6061-T6`, `aluminum_6061`): Авиационный алюминиевый сплав ($E=68.9$ GPa, $\sigma_y=276$ MPa, $\rho=2700$ kg/m³).
- `steel_structural_aisi1020` (алиасы: `steel_1045`, `steel_1020`, `AISI 1020`, `steel`): Конструкционная углеродистая сталь ($E=205.0$ GPa, $\sigma_y=250$ MPa, $\rho=7850$ kg/m³).
- `steel_high_strength_4140` (алиасы: `steel_4140`, `AISI 4140`): Высокопрочная легированная сталь ($E=210.0$ GPa, $\sigma_y=655$ MPa, $\rho=7850$ kg/m³).
- `titanium_grade5_ti6al4v` (алиасы: `ti_6al_4v`, `Ti-6Al-4V`, `Ti6Al4V`, `titanium_ti6al4v`): Титановый сплав Grade 5 ($E=114.0$ GPa, $\sigma_y=880$ MPa, $\rho=4430$ kg/m³).
- `brass_c360` (алиасы: `brass_c36000`, `brass`): Автоматная свинцовая латунь ($E=100.0$ GPa, $\sigma_y=310$ MPa, $\rho=8500$ kg/m³).
- `pla_filament_3dprint` (алиасы: `pla_3d`, `pla_plastic`, `pla`): Пластик PLA для 3D-печати ($E=3.5$ GPa, $\sigma_y=60$ MPa, $\rho=1240$ kg/m³).
- `petg_filament_3dprint` (алиасы: `petg_3d`, `petg_plastic`, `petg`): Пластик PETG для 3D-печати ($E=2.1$ GPa, $\sigma_y=50$ MPa, $\rho=1270$ kg/m³).
- `nylon_pa12` (алиасы: `nylon_pa6`, `nylon`): Инженерный полиамид PA12 ($E=1.8$ GPa, $\sigma_y=45$ MPa, $\rho=1010$ kg/m³).

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "tools/call",
  "params": {
    "name": "create_part",
    "arguments": {
      "name": "cantilever_beam",
      "material": "steel_structural_aisi1020"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-a1b2c3d4e5f64789\"\n}"
      }
    ]
  }
}
```

---

### 2. `list_parts`

Возвращает список всех активных деталей на сцене (отсортированных по `part_id`) с геометрическими габаритами (`bounding_box`, `bounding_box_dimensions`), объёмом (`volume` в `mm³` или `m³` и `volume_m3`), расчётной массой (`mass_kg`), ревизией (`revision`), количеством конструктивных элементов (`features_count`) и треугольников в сетке (`triangles_count`).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `length_unit` | `string` | Нет | Локальное переопределение единицы длины (`"mm"` или `"m"`, по умолчанию используется единица сессии) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/call",
  "params": {
    "name": "list_parts",
    "arguments": {
      "length_unit": "mm"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"parts\": [\n    {\n      \"part_id\": \"part-a1b2c3d4e5f64789\",\n      \"name\": \"cantilever_beam\",\n      \"material\": \"steel_structural_aisi1020\",\n      \"is_fixed\": false,\n      \"revision\": 2,\n      \"volume\": 1000000.0,\n      \"volume_m3\": 0.001,\n      \"mass_kg\": 7.85,\n      \"triangles_count\": 12,\n      \"vertices_count\": 8,\n      \"features_count\": 1,\n      \"bounding_box\": {\n        \"min\": [-500.0, -25.0, -10.0],\n        \"max\": [500.0, 25.0, 10.0]\n      },\n      \"bounding_box_dimensions\": [1000.0, 50.0, 20.0]\n    }\n  ],\n  \"parts_count\": 1,\n  \"length_unit\": \"mm\",\n  \"units\": {\n    \"length\": \"mm\",\n    \"volume\": \"mm3\",\n    \"mass\": \"kg\"\n  }\n}"
      }
    ]
  }
}
```

---

### 3. `delete_part`

Удаляет деталь из сцены, дерева операций, физического мира симуляции и кэша результатов МКЭ.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Уникальный идентификатор детали. Статические якоря `"ground"` и `"world"` удалять запрещено (возвращают ошибку `INVALID_ARGUMENT`) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 3,
  "method": "tools/call",
  "params": {
    "name": "delete_part",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 3,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"status\": \"deleted\",\n  \"part_id\": \"part-a1b2c3d4e5f64789\",\n  \"deleted_part_id\": \"part-a1b2c3d4e5f64789\"\n}"
      }
    ]
  }
}
```

---

## Группа 2: Параметрическое DAG-моделирование

### 4. `add_feature_primitive`

Добавляет базовый 3D-примитив в дерево операций (DAG) детали и инициирует автоматический пересчет геометрии.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор целевой детали |
| `primitive_type` | `string` | Да | Тип примитива: `"box"`, `"cylinder"`, `"sphere"`, `"cone"`, `"polygon_prism"`, `"spur_gear"`, `"revolve"`, `"wheel_rim"` |
| `params` | `object` | Да | Геометрические размеры в метрах (см. таблицу ниже) |

**Спецификация параметров примитивов**:
- `"box"`: `{"width": number, "height": number, "depth": number}`
- `"cylinder"`: `{"radius": number, "height": number, "segments"?: integer}`
- `"sphere"`: `{"radius": number, "sectors"?: integer, "stacks"?: integer}`
- `"cone"`: `{"radius_bottom": number, "radius_top"?: number, "height": number, "segments"?: integer}`
- `"polygon_prism"`: `{"height": number, "sides"?: integer, "width_across_flats"?: number, "radius"?: number, "outer_radius"?: number, "chamfer"?: number, "chamfer_radius"?: number}` (правильная призма для гаек/болтов с коническими торцевыми фасками 30° по DIN 934; поддерживаются алиасы `outer_radius` для `radius` и `chamfer_radius` для `chamfer`)
- `"spur_gear"`: `{"module": number, "teeth": integer, "face_width": number, "pressure_angle_deg"?: number, "bore_radius"?: number, "bore_diameter"?: number, "keyway_width"?: number, "keyway_depth"?: number, "hub_diameter"?: number, "hub_length"?: number}` (прямозубая цилиндрическая шестерня с точным эвольвентным профилем по DIN 867 / ISO 53, опциональной ступицей и шпоночным пазом DIN 6885)
- `"revolve"`: `{"profile": [[r, z], ...], "inner_radius"?: number, "angle_deg"?: number, "segments"?: integer}` (тело вращения 2D-профиля вокруг оси Z; при задании 2-точечной внешней образующей `[[r0, z0], [r1, z1]]` и `inner_radius` автоматически формируется 4-точечный кольцевой профиль `[[inner_r, z0], [r0, z0], [r1, z1], [inner_r, z1]]`; угол поворота `angle_deg` по умолчанию 360°, с плоскими крышками для углов < 360°)
- `"wheel_rim"`: `{"rim_diameter": number, "rim_width": number, "flange_diameter"?: number, "hub_diameter"?: number, "hub_bore_diameter"?: number, "pcd_diameter"?: number, "bolt_holes_count"?: integer, "bolt_hole_diameter"?: number, "spokes_count"?: integer, "spoke_width"?: number, "spoke_thickness"?: number, "offset_et"?: number}` (параметрический легкосплавный колесный диск со ступенчатым ободом, закраинами, монтажным ручьем, ступицей, PCD разболтовкой и спицами с открытыми окнами вентиляции тормозов)
- *Общие опциональные параметры*: `"position"?: [x, y, z]`, `"rotation_deg"?: [rx, ry, rz]`

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 4,
  "method": "tools/call",
  "params": {
    "name": "add_feature_primitive",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789",
      "primitive_type": "box",
      "params": {
        "width": 0.20,
        "height": 0.05,
        "depth": 0.05
      }
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 4,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-a1b2c3d4e5f64789\",\n  \"feature_id\": \"d7a8e2f0-1234-5678-90ab-cdef11223344\",\n  \"primitive_type\": \"box\",\n  \"revision\": 1,\n  \"bounding_box\": {\n    \"min\": [-100.0, -25.0, -25.0],\n    \"max\": [100.0, 25.0, 25.0],\n    \"dimensions\": [200.0, 50.0, 50.0]\n  },\n  \"volume\": 500000.0,\n  \"volume_m3\": 0.0005\n}"
      }
    ]
  }
}
```

---

### 5. `cut_hole`

Вырезает сквозное или глухое цилиндрическое отверстие вдоль заданного нормального вектора, с опциональным шпоночным пазом по стандарту DIN 6885. Возвращает `part_id`, `feature_id`, `revision`, фактическую вычисленную глубину отверстия `depth` (в `length_unit`) и `depth_m` (в метрах СИ), флаг `through`, а также обновлённые `bounding_box`, `volume` и `volume_m3`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор целевой детали |
| `position` | `[number, number, number]` | Да | Координаты центра отверстия `[x, y, z]` (в активном `length_unit`: `mm` или `m`) |
| `normal` (или `direction`) | `[number, number, number]` | Да | Ненулевой вектор направления оси отверстия `[nx, ny, nz]` |
| `radius` (или `diameter`) | `number` | Да | Радиус (или диаметр `diameter = 2 * radius`) отверстия (в активном `length_unit`) |
| `depth` | `number` | Нет (если `through: true`) | Глубина выреза (в активном `length_unit`). Обязателен, если не задан `"through": true` |
| `through` | `boolean` | Нет | Если `true`, автоматически прорезает сквозное отверстие через всю толщину детали вдоль `normal`/`direction` без необходимости указывать `depth` |
| `keyway_width` | `number` | Нет | Ширина шпоночного паза DIN 6885 (в активном `length_unit`) |
| `keyway_depth` | `number` | Нет | Глубина шпоночного паза DIN 6885 в стенку (в активном `length_unit`) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 5,
  "method": "tools/call",
  "params": {
    "name": "cut_hole",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789",
      "position": [0.05, 0.0, 0.0],
      "normal": [0.0, 1.0, 0.0],
      "radius": 0.006,
      "depth": 0.06
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 5,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-a1b2c3d4e5f64789\",\n  \"feature_id\": \"f8b9c0d1-2345-6789-01bc-def223344556\",\n  \"revision\": 2,\n  \"depth\": 60.0,\n  \"depth_m\": 0.06,\n  \"through\": false,\n  \"bounding_box\": {\n    \"min\": [-100.0, -25.0, -25.0],\n    \"max\": [100.0, 25.0, 25.0],\n    \"dimensions\": [200.0, 50.0, 50.0]\n  },\n  \"volume\": 494345.1,\n  \"volume_m3\": 0.0004943451\n}"
      }
    ]
  }
}
```

---

### 6. `modify_feature`

Модифицирует численные параметры любого существующего шага в истории DAG без разрушения структуры детали. Возвращает `part_id`, `feature_id`, `status`, `revision`, `updated_bounds`, `dimensions`, `bounding_box`, `volume` и `volume_m3`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор целевой детали |
| `feature_id` | `string` | Да | UUID узла в дереве операций |
| `params` | `object` | Да | Обновленный набор геометрических параметров |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 6,
  "method": "tools/call",
  "params": {
    "name": "modify_feature",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789",
      "feature_id": "d7a8e2f0-1234-5678-90ab-cdef11223344",
      "params": {
        "width": 0.25,
        "height": 0.06,
        "depth": 0.05
      }
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 6,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-a1b2c3d4e5f64789\",\n  \"feature_id\": \"d7a8e2f0-1234-5678-90ab-cdef11223344\",\n  \"status\": \"updated\",\n  \"revision\": 3,\n  \"updated_bounds\": [-0.125, -0.03, -0.025, 0.125, 0.03, 0.025],\n  \"dimensions\": [0.25, 0.06, 0.05],\n  \"bounding_box\": {\n    \"min\": [-0.125, -0.03, -0.025],\n    \"max\": [0.125, 0.03, 0.025],\n    \"dimensions\": [0.25, 0.06, 0.05]\n  },\n  \"volume\": 0.00075,\n  \"volume_m3\": 0.00075\n}"
      }
    ]
  }
}
```

---

### 7. `get_feature_tree`

Возвращает полный упорядоченный список всех узлов истории операций DAG для анализа цепочки моделирования, а также текущую ревизию детали `revision`. Все линейные параметры аналитических операций `FeatureOp` и вложенные меш-операнды (`BooleanDifferenceMesh`, `BooleanUnionMesh`, `BooleanIntersectionMesh`, `ImportedMesh`: координаты вершин `vertices[i].position`, центроиды `faces[i].centroid`, точки `faces[i].sample_point`, радиусы `faces[i].radius` в `length_unit` и площади `faces[i].area` в `length_unit²`) масштабируются в активную единицу длины (`mm` по умолчанию или `m`).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор целевой детали |
| `length_unit` | `"mm" \| "m"` | Нет | Опциональное переопределение единицы длины для этого вызова (по умолчанию используется единица сессии) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 7,
  "method": "tools/call",
  "params": {
    "name": "get_feature_tree",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 7,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-a1b2c3d4e5f64789\",\n  \"revision\": 1,\n  \"dag\": [\n    {\n      \"id\": \"d7a8e2f0-1234-5678-90ab-cdef11223344\",\n      \"name\": \"AddBox_1\",\n      \"enabled\": true,\n      \"created_at\": 1700000000,\n      \"op\": {\n        \"AddBox\": {\n          \"width\": 250.0,\n          \"height\": 60.0,\n          \"depth\": 50.0,\n          \"position\": [0.0, 0.0, 0.0]\n        }\n      },\n      \"parent_id\": null\n    }\n  ],\n  \"units\": {\n    \"length\": \"mm\",\n    \"area\": \"mm2\",\n    \"volume\": \"mm3\",\n    \"angle\": \"deg\",\n    \"mass\": \"kg\",\n    \"force\": \"N\",\n    \"stress\": \"MPa\",\n    \"time\": \"s\"\n  }\n}"
      }
    ]
  }
}
```

---

### 8. `cut_helical_groove`

Вырезает винтовые спиральные канавки (стружечные канавки сверл, теплоотводящие спирали, нарезка метрической треугольной резьбы 60° по ISO 68-1). Возвращает `part_id`, `feature_id`, `revision`, `start_z`, `status`, `bounding_box`, `volume` и `volume_m3`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор детали |
| `helix_radius` | `number` | Да | Радиус центральной траектории спирали (м) |
| `groove_radius` | `number` | Да | Радиус сечения или глубина резьбы (м) |
| `pitch` | `number` | Да | Осевой шаг винта (м на оборот 360°) |
| `height` | `number` | Да | Полная осевая длина спирали (м) |
| `start_z` | `number` | Нет | Z-координата начала спирали (в активной единице `length_unit`: `mm` или `m`). Если не указана, автоматически выравнивается по нижней грани детали `bounding_box.min[2]` (или `-height / 2`, если сетка пуста) |
| `starts` | `integer` | Нет | Число заходов резьбы/канавок (по умолчанию `1` для `metric_60` и `2` для `circular`) |
| `is_right_hand` | `boolean` | Нет | Направление нарезки (true - правая, false - левая, по умолчанию true) |
| `profile_type` | `string` | Нет | Профиль сечения инструмента: `"circular"` (по умолчанию) или `"metric_60"` (треугольный профиль метрической резьбы 60° по ISO 68-1) |

---

### 9. `boolean_difference`

Выполняет точное аналитическое булево вычитание сетки инструмента из целевой детали (CSG Difference). Возвращает обновлённый счётчик ревизии геометрии `revision`, габаритный бокс `bounding_box`, объём `volume` (в `mm³` или `m³`) и `volume_m3`, а также массив диагностических предупреждений масштаба `warnings`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `target_part_id` | `string` | Да | Идентификатор детали, из которой вычитается объем |
| `tool_part_id` | `string` | Да | Идентификатор детали-инструмента |
| `length_unit` | `"mm" \| "m"` | Нет | Опциональное переопределение единицы длины для этого вызова (по умолчанию используется единица сессии) |

---

### 10. `tessellate_part`

Выполняет ленивую адаптивную тесселяцию криволинейных поверхностей детали по хорде и углу отклонения с обновлением кэша полигональной сетки. Перевычисление полигональной сетки с другим разрешением (`coarse` / `medium` / `fine`) не изменяет параметрический DAG детали, поэтому `part.revision` остаётся неизменным, а ранее выполненные расчёты МКЭ (`run_fea_analysis`) и требования `min_safety_factor` сохраняют актуальный статус `FRESH` / `PASS`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор детали |
| `quality` | `string` | Нет | Пресет качества: `"coarse"` (~0.5 мм), `"medium"` (~0.1 мм, default), `"fine"` (~0.02 мм) |
| `chord_tolerance` | `number` | Нет | Пользовательская максимальная стрела прогиба (хорда $\delta$) в метрах |
| `angular_tolerance_deg` | `number` | Нет | Максимальное угловое отклонение между хордами в градусах |
| `max_edge_length` | `number` | Нет | Ограничение максимальной длины ребра в метрах |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 10,
  "method": "tools/call",
  "params": {
    "name": "tessellate_part",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789",
      "quality": "fine"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 10,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-a1b2c3d4e5f64789\",\n  \"quality\": \"Fine\",\n  \"triangles_count\": 12480,\n  \"vertices_count\": 6242\n}"
      }
    ]
  }
}
```

---

## Группа 3: Кинематика и физика механизмов

### 11. `create_joint`

Устанавливает кинематическое или силовое сопряжение между двумя деталями в физическом мире Rapier3D. Поддерживает сопряжения с неподвижным пространством (`"ground"` / `"world"`), а также кинематические зубчатые передачи (`"gear"`).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `joint_id` (или `id`) | `string` | Нет | Пользовательский идентификатор сочленения (по умолчанию автогенерируется `joint-<uuid>`). При дублировании возвращает ошибку `JOINT_ALREADY_EXISTS` |
| `joint_type` | `string` | Да | `"revolute"`, `"prismatic"`, `"spherical"`, `"fixed"`, `"gear"` |
| `part_a` | `string` | Да | Идентификатор первой сопрягаемой детали (или `"ground"` / `"world"` для привязки к неподвижному пространству; обрабатывается регистронезависимо) |
| `part_b` | `string` | Да | Идентификатор второй сопрягаемой детали (или `"ground"` / `"world"`; обрабатывается регистронезависимо) |
| `anchor_a` | `[number, number, number]` | Нет | Точка крепления `[x, y, z]` в СК детали A (м). По умолчанию `[0, 0, 0]` |
| `anchor_b` | `[number, number, number]` | Нет | Точка крепления `[x, y, z]` в СК детали B (м). По умолчанию `[0, 0, 0]` |
| `axis` | `[number, number, number]` | Нет | Направляющая ось `[x, y, z]` (обязательна для revolute/prismatic) |
| `limits` | `[number, number]` | Нет | Пределы движения `[min, max]` (в радианах или метрах) |
| `ratio` | `number` | Нет | Передаточное отношение для `joint_type: "gear"` ($i = \omega_B / \omega_A$). Должно быть конечным ненулевым числом ($|\text{ratio}| \ge 10^{-9}$). Отрицательное значение указывает на встречное вращение (внешнее зацепление) |
| `teeth_a` | `number` | Нет | Число зубьев ведущей шестерни $z_A$ (альтернатива `ratio`; строго положительное число $> 0$; рассчитывает $i = -z_A / z_B$) |
| `teeth_b` | `number` | Нет | Число зубьев ведомой шестерни $z_B$ (альтернатива `ratio`; строго положительное число $> 0$) |
| `axis_a` | `[number, number, number]` | Нет | Ось вращения шестерни A (по умолчанию `[0, 0, 1]`; ненулевой вектор) |
| `axis_b` | `[number, number, number]` | Нет | Ось вращения шестерни B (по умолчанию `[0, 0, 1]`; ненулевой вектор) |

> **Примечание по зубчатым передачам (`"gear"`):**  
> При создании сопряжения типа `"gear"` физический движок строго валидирует входные параметры: передаточное отношение `ratio` должно быть конечным ненулевым ($|\text{ratio}| \ge 10^{-9}$), параметры `teeth_a` и `teeth_b` должны быть строго положительными ($> 0$), а оси вращения `axis_a` и `axis_b` — ненулевыми векторами ($\|v\| \ge 10^{-7}$). При некорректных параметрах возвращается ошибка `INVALID_ARGUMENT` с указанием конкретного `param`. Движок автоматически отключает прямой полигональный контакт между зубьями (`contacts_enabled(false)`), исключая заклинивание выпуклых оболочек colliders, и на каждом шаге времени строго синхронизирует угловые скорости шестерней согласно передаточному отношению $\omega_B = -\text{ratio} \cdot \omega_A$.

#### Пример запроса (вращательный шарнир на неподвижную раму)
```json
{
  "jsonrpc": "2.0",
  "id": 11,
  "method": "tools/call",
  "params": {
    "name": "create_joint",
    "arguments": {
      "joint_type": "revolute",
      "part_a": "gearbox_housing",
      "part_b": "pinion_gear",
      "anchor_a": [0.0, 0.0, 0.0],
      "anchor_b": [0.0, 0.0, 0.0],
      "axis": [0.0, 0.0, 1.0]
    }
  }
}
```

#### Пример запроса (зубчатое зацепление шестерней)
```json
{
  "jsonrpc": "2.0",
  "id": 12,
  "method": "tools/call",
  "params": {
    "name": "create_joint",
    "arguments": {
      "joint_type": "gear",
      "part_a": "pinion_gear",
      "part_b": "driven_gear",
      "teeth_a": 20,
      "teeth_b": 40
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 11,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"joint_id\": \"joint-550e8400-e29b-41d4-a716-446655440000\"\n}"
      }
    ]
  }
}
```

---

### 12. `set_joint_motor`

Активирует сервопривод или мотор поддержания скорости на сочленении (`revolute` или `prismatic`).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `joint_id` | `string` | Да | Идентификатор сочленения |
| `target_velocity` | `number` | Да | Конечная (`is_finite()`) целевая скорость: угловая в $\text{рад/с}$ (для `revolute`) или линейная в $\text{м/с}$ (для `prismatic`). Отрицательные значения задают реверсивное направление движения |
| `max_force` | `number` | Да | Неотрицательный конечный (`>= 0.0`, `is_finite()`) предельный крутящий момент в $\text{Н}\cdot\text{м}$ (для `revolute`) или усилие в $\text{Н}$ (для `prismatic`) |

> **Валидация (P2-T11):** передача нечисловых значений, `NaN`, `±Infinity` (включая значения `f64`, переполняющие диапазон `f32` в решателе Rapier3D) для `target_velocity` или `max_force`, а также отрицательного `max_force < 0.0` возвращает структурированную ошибку `INVALID_ARGUMENT` с указанием `param: "target_velocity"` или `param: "max_force"`.

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 12,
  "method": "tools/call",
  "params": {
    "name": "set_joint_motor",
    "arguments": {
      "joint_id": "joint-550e8400-e29b-41d4-a716-446655440000",
      "target_velocity": 6.283,
      "max_force": 50.0
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 12,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"status\": \"motor_active\",\n  \"length_unit\": \"mm\",\n  \"units\": {\n    \"length\": \"mm\",\n    \"force\": \"N\",\n    \"stress\": \"MPa\"\n  }\n}"
      }
    ]
  }
}
```

---

### 13. `run_physics_simulation`

Запускает динамическую симуляцию механизма вперед во времени с записью траекторий, реакций в шарнирах и детекцией соударений. При наличии открытого окна предварительного просмотра автоматически генерирует ключевые кадры динамической анимации и передает их в графический конвейер для живого интерактивного воспроизведения.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `duration_seconds` | `number` | Да | Длительность физической симуляции в секундах |
| `step_size_ms` | `number` | Нет | Шаг численного интегрирования в мс (по умолчанию `16.67` мс = 60 Гц) |

> **Интерактивное воспроизведение:**  
> После завершения симуляции все трансформации деталей компонуются в массив кадров полигональной сетки. Окно предварительного просмотра (`open_preview_window`) переходит в режим циклического воспроизведения анимации движения со скоростью симуляции. Пользователь может приостанавливать и продолжать анимацию нажатием клавиши `P`.

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 10,
  "method": "tools/call",
  "params": {
    "name": "run_physics_simulation",
    "arguments": {
      "duration_seconds": 1.5,
      "step_size_ms": 16.6667
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 10,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"success\": true,\n  \"steps_simulated\": 90,\n  \"peak_joint_forces\": {\n    \"joint-550e8400-e29b-41d4-a716-446655440000\": 34.2\n  },\n  \"collisions\": [],\n  \"trajectory_summary\": {\n    \"part-crank\": [\n      {\"time\": 0.0, \"pos\": [0.0, 0.05, 0.0]},\n      {\"time\": 1.5, \"pos\": [0.0, 0.05, 0.0]}\n    ]\n  }\n}"
      }
    ]
  }
}
```

---

## Группа 4: Конечно-элементный анализ прочности (FEA)

### 14. `run_fea_analysis`

Выполняет автоматическую тетраэдризацию детали (`TET4` при `element_order = 1` или квадратичные 10-узловые тетраэдры `TET10` при `element_order = 2`), решает 3D уравнения линейной упругости ($K \cdot u = F$), вычисляет эквивалентные напряжения по Мизесу и запас прочности. Поддерживает задание граничных условий закрепления и нагрузок по именам граней / селекторам (`fixed`, `loads`), а также по осевым граням AABB и координатам (`fixed_faces`, `fixed_plane`, `applied_forces`).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор анализируемой детали |
| `length_unit` | `string` | Нет | Переопределение единиц длины на вызов (`"mm"` или `"m"`) |
| `mesh_density` | `string` | Нет | Плотность сетки: `"coarse"` (по умолчанию), `"medium"`, `"fine"` |
| `element_order` | `integer` | Нет | Порядок конечных элементов: `1` (линейные 4-узловые тетраэдры `TET4`, по умолчанию) или `2` (квадратичные 10-узловые изопараметрические тетраэдры `TET10` с 4-точечной квадратурой Гаусса, устраняющие эффект сдвигового запирания `shear locking` при изгибе) |
| `fixed` | `object[]` | Нет | Закрепления Дирихле по именованным граням или селекторам: `[{"face": "cap_bottom", "components"?: "all"\|"x"\|"y"\|"z", "tolerance"?: number}, {"where": {...}}]` |
| `fixed_faces` | `string[]` | Нет | Грани AABB (`"min_x"`, `"max_x"`, `"min_y"`, `"max_y"`, `"min_z"`, `"max_z"`) или имена граней детали (`"cap_bottom"`, `"left"`) для жесткого закрепления |
| `fixed_plane` | `object` | Нет | Опорная плоскость: `{"origin": [x,y,z], "normal": [nx,ny,nz], "tolerance"?: number}` (в активных `length_unit`) |
| `loads` | `object[]` | Нет | Нагрузки по именованным граням или селекторам: суммарная сила `{"face": "cap_top", "force": [Fx, Fy, Fz]}` (в `N`, распределяется пропорционально грузовым площадям узлов $\sum \mathbf{f}_i = \mathbf{F}$), нормальное давление `{"face": "hole_wall", "pressure": p_mpa}` (в `MPa`) или момент `{"face": "cap_top", "moment": [Mx, My, Mz]}` (в `N·m`) |
| `applied_force` | `object` | Нет | Одиночная нагрузка: `{"position"?: [x,y,z], "face"?: string, "force_vector": [Fx,Fy,Fz]}` |
| `applied_forces` | `object[]` | Нет | Массив распределенных или точечных нагрузок |
| `convergence_study` | `boolean` | Нет | Если `true`, выполняет прогон расчёта на последовательности сеток и добавляет блок оценки сходимости `accuracy` (и полную сводку `convergence_study`). Если вердикт сходимости `NOT_CONVERGED`, поле `confidence` автоматически понижается до `"low"` |
| `levels` | `string[]` | Нет | Список плотностей сетки для исследования сходимости (минимум 2 уровня из `"coarse"`, `"medium"`, `"fine"`). По умолчанию `["coarse", "medium", "fine"]` |
| `max_seconds` | `number` | Нет | Бюджет времени на исследование сходимости в секундах (`>= 0`). При превышении лимита после завершения очередного уровня возвращается частичный результат с явным маркером `accuracy.partial_result: true` и `verdict: "NOT_CONVERGED"` |

#### Оценка сходимости (`accuracy`) и экстраполяция Ричардсона (`P3-T3`)
При `convergence_study: true` в ответ добавляется объект `accuracy`:
- `levels`: массив пройденных уровней сетки (`density`, `target_element_size_m`, `node_count`, `element_count`, `max_displacement_m`, `max_displacement_mm`, `max_von_mises_pa`, `max_von_mises_mpa`, `computation_time_ms`).
- `displacement_change_pct` / `stress_change_pct`: относительное изменение (в `%`) пикового перемещения и напряжения Мизеса между двумя последними уровнями сетки.
- `effective_refinement_ratio`: фактический кубический корень отношения числа тетраэдров $r_{\text{eff}} = (N_{\text{last}} / N_{\text{prev}})^{1/3}$.
- `converged` / `verdict` (`"CONVERGED"` | `"NOT_CONVERGED"`): вердикт `CONVERGED` выставляется только при одновременном выполнении условий:
  1. Исследование завершено полностью (`partial_result == false`, минимум 2 уровня);
  2. $|\Delta u| < 5.0\%$ (`CONVERGENCE_DISP_TOLERANCE_PCT`) и $|\Delta \sigma| < 10.0\%$ (`CONVERGENCE_STRESS_TOLERANCE_PCT`);
  3. Сетка реально измельчалась между двумя последними уровнями ($r_{\text{eff}} \ge 1.25$, `grid_clamped_detected == false` — защита от ложной сходимости при упоре воксельной сетки в лимит `60` ячеек по оси);
  4. Отсутствует сингулярность напряжения на закреплении (`singularity_detected == false`) и ступенчатый воксельный концентратор напряжений на криволинейной границе/отверстии (`voxel_staircase_detected == false`).
- **Условия применимости экстраполяции Ричардсона** (`richardson_applicable == true`):
  - Выполнено не менее 3 уровней сетки со строго возрастающим числом элементов ($r_{21} > 1.05$, $r_{32} > 1.05$);
  - Отсутствует сингулярность в точке измерения (`!singularity_detected`) и ступенчатое искажение криволинейного концентратора (`!voxel_staircase_detected`);
  - Наблюдается монотонная сходимость ($(\phi_2 - \phi_1)\cdot(\phi_3 - \phi_2) > 0$) с уменьшающимся приращением в асимптотической области (наблюдаемый порядок сходимости $p_{\text{obs}} \in [0.25, 4.0]$). При выполнении этих условий вычисляются `richardson_displacement_mm`, `richardson_stress_mpa` и `observed_order_p`.

#### Пример запроса (по именованным граням со сходимостью)
```json
{
  "jsonrpc": "2.0",
  "id": 14,
  "method": "tools/call",
  "params": {
    "name": "run_fea_analysis",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789",
      "convergence_study": true,
      "levels": ["coarse", "medium", "fine"],
      "max_seconds": 30.0,
      "fixed": [
        { "face": "cap_bottom" }
      ],
      "loads": [
        {
          "face": "cap_top",
          "force": [0.0, 0.0, 50000.0]
        }
      ]
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 14,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"computation_time_ms\": 245,\n  \"confidence\": \"low\",\n  \"element_count\": 10290,\n  \"max_displacement_mm\": 0.7284,\n  \"max_stress_excluding_constraint_zone_mpa\": 82.97,\n  \"max_von_mises_mpa\": 112.97,\n  \"node_count\": 2528,\n  \"safety_factor\": 2.21,\n  \"status\": \"PASS\",\n  \"verdict\": \"PASS\",\n  \"yield_strength_mpa\": 250.0,\n  \"accuracy\": {\n    \"converged\": false,\n    \"verdict\": \"NOT_CONVERGED\",\n    \"partial_result\": false,\n    \"displacement_change_pct\": 12.40,\n    \"stress_change_pct\": 13.17,\n    \"effective_refinement_ratio\": 1.498,\n    \"richardson_applicable\": true,\n    \"richardson_displacement_mm\": 0.9032,\n    \"richardson_stress_mpa\": 144.89,\n    \"observed_order_p\": 1.0,\n    \"recommendation\": \"NOT_CONVERGED: Mesh change between last two levels (disp +12.40%, stress +13.17%) exceeds tolerances (5.0% disp / 10.0% stress). Confidence is low.\"\n  }\n}"
      }
    ]
  }
}
```

---

### 15. `get_fea_heatmap_image`

Рендерит оффскрин 3D-снимок детали, раскрашенной градиентной тепловой картой напряжений по Мизесу, в формат PNG/Base64.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор детали |
| `camera_preset` | `string` | Нет | Ракурс камеры: `"isometric"`, `"front"`, `"top"`, `"right"` (дефолт: `"isometric"`) |
| `width` | `integer` | Нет | Ширина кадра в пикселях (по умолчанию `800`) |
| `height` | `integer` | Нет | Высота кадра в пикселях (по умолчанию `600`) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 15,
  "method": "tools/call",
  "params": {
    "name": "get_fea_heatmap_image",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789",
      "camera_preset": "isometric",
      "width": 1024,
      "height": 768
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 15,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"colormap_legend\": {\n    \"colormap_scale\": [\n      \"blue (0.0 MPa - lowest stress)\",\n      \"cyan\",\n      \"green (safe)\",\n      \"yellow (moderate)\",\n      \"red (peak stress)\"\n    ],\n    \"factor_of_safety\": 2.54,\n    \"max_mpa\": 98.42,\n    \"min_mpa\": 0.0,\n    \"verdict\": \"PASS\",\n    \"yield_strength_mpa\": 250.0\n  }\n}"
      },
      {
        "type": "image",
        "data": "iVBORw0KGgoAAAANSUhEUgAABAAAAAMACAYAA...",
        "mimeType": "image/png"
      }
    ]
  }
}
```

---

## Группа 5: Визуализация, превью, экспорт и импорт

### 16. `capture_viewport`

Делает 3D-снимок всей сборки или отдельной детали с затенением Blinn-Phong и контурной обводкой ребер.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `camera_preset` | `string` | Нет | `"isometric"`, `"front"`, `"top"`, `"right"` |
| `part_id` | `string` | Нет | Если не указан — рендерится вся сцена сборки |
| `width` | `integer` | Нет | Ширина кадра в пикселях (по умолчанию `800`) |
| `height` | `integer` | Нет | Высота кадра в пикселях (по умолчанию `600`) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 16,
  "method": "tools/call",
  "params": {
    "name": "capture_viewport",
    "arguments": {
      "camera_preset": "isometric",
      "width": 1280,
      "height": 720
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 16,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"camera_preset\": \"Isometric\",\n  \"format\": \"image/png\",\n  \"height\": 720,\n  \"width\": 1280\n}"
      },
      {
        "type": "image",
        "data": "iVBORw0KGgoAAAANSUhEUgA...",
        "mimeType": "image/png"
      }
    ]
  }
}
```

---

### 17. `open_preview_window`

Открывает или фокусирует интерактивное нативное окно 3D-просмотра на Windows (Winit + WGPU DirectX 12/Vulkan). Поддерживает вращение, панорамирование и зум орбитальной камерой мышь/клавиатура, переключение качества тесселяции клавишей `'Q'`, а также воспроизведение и паузу динамической анимации симуляции механизмов клавишей `'P'`.

Если симуляция (`run_physics_simulation`) уже была выполнена, окно автоматически инициализируется в режиме циклического воспроизведения рассчитанной анимации (`mode: "animation_playback"`).

#### Горячие клавиши окна:
- **ЛКМ + Перетаскивание**: Вращение орбитальной камеры вокруг сцены.
- **ПКМ + Перетаскивание** или **Колесико мыши**: Панорамирование и зум.
- **Клавиша `Q`**: Переключение качества адаптивной тесселяции (`Coarse` $\to$ `Medium` $\to$ `Fine`).
- **Клавиша `P`**: Пауза / продолжение циклического воспроизведения анимации физической симуляции.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Нет | Опциональный идентификатор детали для фокусировки |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 17,
  "method": "tools/call",
  "params": {
    "name": "open_preview_window",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 17,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"status\": \"preview_window_opened\"\n}"
      }
    ]
  }
}
```

---

### 18. `import_stl`

Импортирует внешнюю 3D-модель из файла формата STL (бинарного или ASCII) в сцену как новую деталь с автоматической оптимизацией и сшивкой вершин.

> **Важно (единицы координат STL):** Координаты вершин из файла `.stl` читаются во внутреннее СИ-ядро как **метры (`m`)**. В JSON-ответе габаритный бокс `bounding_box` возвращается в активной единице `length_unit` (`mm` по умолчанию), а при подозрении на несоответствие масштаба (например, если STL был сохранён в миллиметрах без масштабирования и его габарит превышает `10 м`) в массив `warnings` добавляется предупреждение `SCALE_TOO_LARGE`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `filepath` | `string` | Да (или `file_path`) | Путь к STL-файлу на диске |
| `file_path` | `string` | Нет | Алиас параметра `filepath` |
| `part_id` | `string` | Нет | Пользовательский уникальный идентификатор детали (по умолчанию `part-<UUID>`). Запрещены зарезервированные имена `ground` и `world`. При дублировании возвращается ошибка `PART_ALREADY_EXISTS`. |
| `name` | `string` | Нет | Пользовательское имя детали (по умолчанию имя файла) |
| `material` | `string` | Нет | Идентификатор материала из реестра (по умолчанию `steel_structural_aisi1020`) |
| `is_fixed` | `boolean` | Нет | Закреплена ли деталь в пространстве симуляции физики (`true` — статический `TriMesh`, `false` — динамический `ConvexHull`; по умолчанию `false`). Деталь автоматически регистрируется в `physics_world`. |
| `length_unit` | `string` | Нет | Переопределение единицы длины в ответе (`"mm"` или `"m"`) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 18,
  "method": "tools/call",
  "params": {
    "name": "import_stl",
    "arguments": {
      "file_path": "./output/external_bracket.stl",
      "part_id": "bracket_imported",
      "name": "imported_bracket",
      "material": "aluminum_6061_t6",
      "is_fixed": true,
      "length_unit": "mm"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 18,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"bracket_imported\",\n  \"name\": \"imported_bracket\",\n  \"is_fixed\": true,\n  \"revision\": 1,\n  \"triangles_count\": 5420,\n  \"vertices_count\": 2712,\n  \"bounding_box\": {\n    \"min\": [-25.0, -15.0, -10.0],\n    \"max\": [25.0, 15.0, 10.0],\n    \"dimensions\": [50.0, 30.0, 20.0]\n  },\n  \"volume\": 30000.0,\n  \"volume_m3\": 0.00003,\n  \"mass_kg\": 0.081,\n  \"mass\": 0.081,\n  \"status\": \"stl_imported\",\n  \"length_unit\": \"mm\",\n  \"units\": {\n    \"length\": \"mm\",\n    \"volume\": \"mm³\",\n    \"mass\": \"kg\"\n  },\n  \"warnings\": []\n}"
      }
    ]
  }
}
```

---

### 19. `export_stl`

Экспортирует поверхностную сетку детали в файл стандартного формата STL с поддержкой адаптивной тесселяции по хорде для прецизионного производства и 3D-печати. Не увеличивает счётчик ревизии геометрии `part.revision` и не переводит результаты МКЭ (`min_safety_factor`, `last_fea`) в статус `STALE`.

> **Важно (единицы координат STL):** Координаты вершин в записываемом на диск файле `.stl` всегда сохраняются во внутренних единицах СИ — **метрах (`m`)** (в JSON-ответе возвращается `"stl_vertex_unit": "m"`). Входной параметр `chord_tolerance` интерпретируется в активной единице `length_unit` (`mm` по умолчанию).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор детали |
| `filepath` | `string` | Да | Путь для сохранения файла на диске (например, `"./output/bracket.stl"`) |
| `binary` | `boolean` | Нет | `true` — компактный Binary STL (по умолчанию); `false` — ASCII STL |
| `quality` | `string` | Нет | Пресет тесселяции: `"coarse"`, `"medium"`, `"fine"` (по умолчанию: `"fine"`) |
| `chord_tolerance` | `number` | Нет | Пользовательский допуск хорды (стрела прогиба) в активных `length_unit` (по умолчанию в `mm`, например `0.05`) |
| `length_unit` | `string` | Нет | Переопределение единицы длины для `chord_tolerance` (`"mm"` или `"m"`) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 19,
  "method": "tools/call",
  "params": {
    "name": "export_stl",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789",
      "filepath": "./output/cantilever_beam.stl",
      "binary": true,
      "quality": "fine"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 19,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"file_path\": \"./output/cantilever_beam.stl\",\n  \"triangles_count\": 2480,\n  \"stl_vertex_unit\": \"m\",\n  \"length_unit\": \"mm\"\n}"
      }
    ]
  }
}
```

---

### 20. `get_simulation_metrics_json`

Возвращает полный агрегированный инженерный отчет по всей сцене: геометрические метаданные всех деталей (с учётом активного или переопределённого `length_unit`), результаты последних МКЭ-расчетов (включая `fea_revision`, `part_revision`, `stale` и `status`) и телеметрию физической симуляции.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `length_unit` | `"mm" \| "m"` | Нет | Опциональное переопределение единицы длины для этого вызова (по умолчанию используется единица сессии) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 20,
  "method": "tools/call",
  "params": {
    "name": "get_simulation_metrics_json",
    "arguments": {}
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 20,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"current_metrics\": {\n    \"parts_count\": 1,\n    \"parts\": [\n      {\n        \"part_id\": \"part-a1b2c3d4e5f64789\",\n        \"name\": \"cantilever_beam\",\n        \"revision\": 1,\n        \"material\": \"steel_structural_aisi1020\",\n        \"volume\": 1000000.0,\n        \"volume_m3\": 0.001,\n        \"mass_kg\": 7.85,\n        \"triangle_count\": 12,\n        \"bounding_box\": {\"min\": [-500.0, -25.0, -10.0], \"max\": [500.0, 25.0, 10.0], \"dimensions\": [1000.0, 50.0, 20.0]}\n      }\n    ],\n    \"fea_reports\": [\n      {\n        \"part_id\": \"part-a1b2c3d4e5f64789\",\n        \"verdict\": \"PASS\",\n        \"factor_of_safety\": 2.54,\n        \"safety_factor\": 2.54,\n        \"max_von_mises_mpa\": 98.42,\n        \"yield_strength_mpa\": 250.0,\n        \"max_displacement_mm\": 0.184,\n        \"max_displacement\": 0.184,\n        \"nodes\": 275,\n        \"elements\": 864,\n        \"time_ms\": 245,\n        \"fea_revision\": 1,\n        \"part_revision\": 1,\n        \"stale\": false,\n        \"is_stale\": false,\n        \"status\": \"FRESH\"\n      }\n    ],\n    \"physics\": {\n      \"accumulated_time_s\": 1.5,\n      \"gravity\": [0.0, -9810.0, 0.0],\n      \"gravity_m_s2\": [0.0, -9.81, 0.0],\n      \"last_simulation\": {\n        \"step_count\": 90,\n        \"peak_joint_forces\": {\"joint-550e8400-e29b-41d4-a716-446655440000\": 34.2},\n        \"collisions\": []\n      }\n    }\n  },\n  \"length_unit\": \"mm\"\n}"
      }
    ]
  }
}
```

---

## Группа 6: Топология, CSG операции, кастомные материалы и пакетная оптимизация

### 21. `query_part_geometry`

Выполняет глубокую топологическую и геометрическую инспекцию твердотельной детали. Возвращает точный габаритный ящик AABB (`min`, `max`, `dimensions`), абсолютный объем, площадь поверхности, центр масс (centroid), количество полигонов и вершин, реестр именованных топологических граней (`faces`: `id`, `name`, `aliases`, `role`, `kind`, `origin_feature`, `area`, `area_m2`, `centroid`, `sample_point`, `normal_or_axis`, `radius`, `triangles_count`), а также массив крупнейших компланарных плоских граней (`detected_planar_faces`). Поддерживает фильтрацию граней через `face_selector` или `where` (см. [`docs/FACE_NAMING_DESIGN.md`](FACE_NAMING_DESIGN.md)).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `part_id` | `string` | **Да** | Уникальный идентификатор инспектируемой детали |
| `length_unit` | `"mm" \| "m"` | Нет | Опциональное переопределение единицы длины для этого вызова (по умолчанию используется единица сессии) |
| `face_selector` | `string \| object` | Нет | Селектор граней: имя/алиас/шаблон (`"top"`, `"side"`, `"feat_0:top"`, `"hole_wall:*"`, `"cut:*"`), `{"face": name}`, `{"face_id": id}`, `{"faces": [...]}` или `{"where": {"kind", "radius", "tolerance", "normal", "axis", "near", "closest", "origin_feature", "role", "min_area", "max_area"}}` |
| `where` | `object` | Нет | Сокращённая запись геометрического/топологического фильтра граней (эквивалентна `face_selector.where`) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 21,
  "method": "tools/call",
  "params": {
    "name": "query_part_geometry",
    "arguments": {
      "part_id": "part-bracket-001"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 21,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-bracket-001\",\n  \"bounding_box\": {\n    \"min\": [-60.0, -15.0, -30.0],\n    \"max\": [60.0, 15.0, 30.0],\n    \"dimensions\": [120.0, 30.0, 60.0]\n  },\n  \"volume\": 216000.0,\n  \"volume_m3\": 0.000216,\n  \"surface_area\": 25200.0,\n  \"surface_area_m2\": 0.0252,\n  \"center_of_mass\": [0.0, 0.0, 0.0],\n  \"faces_count\": 12,\n  \"vertices_count\": 24,\n  \"faces\": [\n    {\n      \"id\": 1,\n      \"name\": \"feat_0:top\",\n      \"aliases\": [\"top\", \"max_z\", \"feat_0:top\"],\n      \"role\": \"top\",\n      \"kind\": \"plane\",\n      \"origin_feature\": \"d7a8e2f0-1234-5678-90ab-cdef11223344\",\n      \"area\": 3600.0,\n      \"area_m2\": 0.0036,\n      \"centroid\": [0.0, 0.0, 30.0],\n      \"sample_point\": [0.0, 0.0, 30.0],\n      \"normal_or_axis\": [0.0, 0.0, 1.0],\n      \"radius\": null,\n      \"triangles_count\": 2\n    }\n  ],\n  \"detected_planar_faces\": [\n    {\n      \"normal\": [0.0, 1.0, 0.0],\n      \"center\": [0.0, 15.0, 0.0],\n      \"area\": 7200.0,\n      \"area_m2\": 0.0072\n    }\n  ],\n  \"length_unit\": \"mm\"\n}"
      }
    ]
  }
}
```

---

### 22. `boolean_union`

Выполняет CSG булево объединение (Union) твердотельных сеток: объединяет форму детали-инструмента (`tool_part_id`) с целевой деталью (`target_part_id`). Добавляет узел `BooleanUnionMesh` в параметрическое дерево DAG целевой детали, пересчитывает геометрию, обновляет отображение во вьюпорте, синхронизирует физический коллайдер в Rapier3D и возвращает `revision`, `bounding_box`, `volume`, `volume_m3` и `warnings`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `target_part_id` | `string` | **Да** | ID целевой детали, принимающей объединение |
| `tool_part_id` | `string` | **Да** | ID детали-инструмента, чья сетка объединяется с целевой (должен отличаться от target_part_id) |
| `length_unit` | `"mm" \| "m"` | Нет | Опциональное переопределение единицы длины для этого вызова (по умолчанию используется единица сессии) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 22,
  "method": "tools/call",
  "params": {
    "name": "boolean_union",
    "arguments": {
      "target_part_id": "part-housing-main",
      "tool_part_id": "part-flange-boss"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 22,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-housing-main\",\n  \"target_part_id\": \"part-housing-main\",\n  \"feature_id\": \"9b1deb4d-3b7d-4bad-9bdd-2b0d7b3dcb6d\",\n  \"revision\": 2,\n  \"status\": \"union_applied\",\n  \"vertices_count\": 842,\n  \"triangles_count\": 1680,\n  \"bounding_box\": {\n    \"min\": [-10.0, -10.0, -10.0],\n    \"max\": [15.0, 10.0, 10.0],\n    \"dimensions\": [25.0, 20.0, 20.0]\n  },\n  \"volume\": 8500.0,\n  \"volume_m3\": 0.0000085,\n  \"length_unit\": \"mm\",\n  \"warnings\": []\n}"
      }
    ]
  }
}
```

---

### 23. `boolean_intersection`

Выполняет CSG булево пересечение (Intersection) твердотельных сеток: сохраняет только общий объем пересечения целевой детали (`target_part_id`) и инструмента (`tool_part_id`). Добавляет узел `BooleanIntersectionMesh` в параметрическое дерево DAG, перестраивает меш, обновляет вьюпорт и физический мир, возвращая `revision`, `bounding_box`, `volume`, `volume_m3` и `warnings`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `target_part_id` | `string` | **Да** | ID целевой детали |
| `tool_part_id` | `string` | **Да** | ID детали-инструмента (должен отличаться от target_part_id) |
| `length_unit` | `"mm" \| "m"` | Нет | Опциональное переопределение единицы длины для этого вызова (по умолчанию используется единица сессии) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 23,
  "method": "tools/call",
  "params": {
    "name": "boolean_intersection",
    "arguments": {
      "target_part_id": "part-cylinder-base",
      "tool_part_id": "part-sphere-cutter"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 23,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-cylinder-base\",\n  \"target_part_id\": \"part-cylinder-base\",\n  \"feature_id\": \"1f38e4a9-82bc-4402-9988-6617a23c3a4f\",\n  \"revision\": 2,\n  \"status\": \"intersection_applied\",\n  \"vertices_count\": 512,\n  \"triangles_count\": 1020,\n  \"bounding_box\": {\n    \"min\": [-5.0, -5.0, -5.0],\n    \"max\": [5.0, 5.0, 5.0],\n    \"dimensions\": [10.0, 10.0, 10.0]\n  },\n  \"volume\": 1000.0,\n  \"volume_m3\": 0.000001,\n  \"length_unit\": \"mm\",\n  \"warnings\": []\n}"
      }
    ]
  }
}
```

---

### 24. `transform_part`

Применяет пространственные преобразования к твердотельному телу в дереве построений DAG (`FeatureOp::Transform`): линейное перемещение (`translation`), углы вращения Эйлера Roll-Pitch-Yaw в градусах (`rotation_deg`) и масштабирование (`scale`), с сохранением `face_id` и автоматическим обновлением нормалей, осей, центроидов, площадей и радиусов именованных граней (включая зеркальное отражение при отрицательном масштабе $\det(S) < 0$). Поддерживает привязку точки поворота/масштабирования к центроиду выбранной грани (`pivot_face`) и выравнивание грани по целевой точке (`align_face`).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `part_id` | `string` | **Да** | ID трансформируемой детали |
| `translation` | `[f64; 3]` | Нет | Вектор смещения $[x, y, z]$ в активных единицах длины (`mm` по умолчанию, или `m`; по умолчанию `[0.0, 0.0, 0.0]`) |
| `rotation_deg` | `[f64; 3]` | Нет | Углы поворота $[roll_x, pitch_y, yaw_z]$ в градусах (по умолчанию `[0.0, 0.0, 0.0]`) |
| `scale` | `[f64; 3]` | Нет | Коэффициенты масштабирования $[s_x, s_y, s_z]$ (по умолчанию `[1.0, 1.0, 1.0]`, компоненты $\ne 0$) |
| `pivot_face` | `string \| object` | Нет | Опциональный селектор именованной грани, чей центроид используется как центр вращения/масштабирования |
| `align_face` | `object` | Нет | Опциональное выравнивание грани `{"face": <selector>, "target_point": [x, y, z]}`: сдвигает деталь так, чтобы центроид выбранной грани совпал с `target_point` |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 24,
  "method": "tools/call",
  "params": {
    "name": "transform_part",
    "arguments": {
      "part_id": "part-bracket-001",
      "translation": [0.05, 0.0, 0.1],
      "rotation_deg": [0.0, 90.0, 0.0],
      "scale": [1.0, 1.0, 1.0]
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 24,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-bracket-001\",\n  \"feature_id\": \"7c28a1e4-6a84-48f5-93df-f41cb87a1928\",\n  \"revision\": 2,\n  \"status\": \"transformed\",\n  \"bounding_box\": {\n    \"min\": [0.02, -0.015, 0.04],\n    \"max\": [0.08, 0.015, 0.16],\n    \"dimensions\": [0.06, 0.03, 0.12]\n  },\n  \"volume\": 0.000216\n}"
      }
    ]
  }
}
```

---

### 25. `define_material`

Динамически регистрирует новый инженерный конструкционный материал в глобальном реестре материалов системы. Зарегистрированный материал немедленно становится доступным для назначения деталям (`create_part`), расчета массы деталей и тензоров инерции в физике (`Rapier3D`), а также для 3D МКЭ-расчета прочности (`run_fea_analysis`).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `id` | `string` | **Да** | Уникальный буквенно-цифровой идентификатор материала (например, `"carbon_fiber_t700"`) |
| `young_modulus_gpa` | `number` | **Да** | Модуль продольной упругости Юнга $E$ в гигапаскалях ($GPa > 0$) |
| `poisson_ratio` | `number` | **Да** | Коэффициент Пуассона $\nu$ (безразмерный, $0.0 < \nu < 0.5$) |
| `yield_strength_mpa` | `number` | **Да** | Предел текучести $\sigma_y$ в мегапаскалях ($MPa > 0$) |
| `density_kg_m3` | `number` | **Да** | Плотность материала $\rho$ в $кг/м^3$ ($> 0$) |
| `name` | `string` | Нет | Человекочитаемое наименование материала (создает синоним) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 25,
  "method": "tools/call",
  "params": {
    "name": "define_material",
    "arguments": {
      "id": "carbon_fiber_t700",
      "name": "Carbon Fiber High Modulus T700",
      "young_modulus_gpa": 230.0,
      "poisson_ratio": 0.30,
      "yield_strength_mpa": 1600.0,
      "density_kg_m3": 1750.0
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 25,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"status\": \"material_registered\",\n  \"material_id\": \"carbon_fiber_t700\",\n  \"properties\": {\n    \"name\": \"Carbon Fiber High Modulus T700\",\n    \"young_modulus_gpa\": 230.0,\n    \"young_modulus_pa\": 230000000000.0,\n    \"poisson_ratio\": 0.3,\n    \"yield_strength_mpa\": 1600.0,\n    \"yield_strength_pa\": 1600000000.0,\n    \"density_kg_m3\": 1750.0\n  }\n}"
      }
    ]
  }
}
```

---

### 26. `batch_cad_operations`

Последовательно выполняет массив инструментов MCP CAD в едином контексте движка за один сетевой запрос JSON-RPC. Инструмент устраняет десятки повторных сетевых вызовов (roundtrip latency) при моделировании составных деталей. Все шаги валидируются и выполняются по порядку в рамках транзакции: если любая операция завершается с ошибкой, выполнение немедленно прерывается с указанием индекса сбойного шага и подробной диагностикой, а состояние сцены (`parts`, `materials`, `physics_world` со всеми телами/шарнирами/моторами, кэши FEA и `requirements`) полностью откатывается к состоянию до начала пакета. Вложенные вызовы `batch_cad_operations` запрещены.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `operations` | `array` | **Да** | Массив объектов операций `[{ tool: string, params: object }]` |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 26,
  "method": "tools/call",
  "params": {
    "name": "batch_cad_operations",
    "arguments": {
      "operations": [
        {
          "tool": "create_part",
          "params": {
            "name": "stepped_shaft",
            "material": "steel_structural_aisi1020",
            "part_id": "part-shaft-01"
          }
        },
        {
          "tool": "add_feature_primitive",
          "params": {
            "part_id": "part-shaft-01",
            "primitive_type": "cylinder",
            "params": {
              "radius": 0.02,
              "height": 0.1
            }
          }
        },
        {
          "tool": "cut_hole",
          "params": {
            "part_id": "part-shaft-01",
            "position": [0.0, 0.0, 0.05],
            "normal": [0.0, 0.0, 1.0],
            "radius": 0.008,
            "depth": 0.04
          }
        }
      ]
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 26,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"success\": true,\n  \"count\": 3,\n  \"results\": [\n    {\n      \"part_id\": \"part-shaft-01\"\n    },\n    {\n      \"feature_id\": \"8a9c3f2b-1a2d-4e5f-b6a7-c8d9e0f1a2b3\"\n    },\n    {\n      \"feature_id\": \"4c5d6e7f-8a9b-0c1d-2e3f-4a5b6c7d8e9f\"\n    }\n  ]\n}"
      }
    ]
  }
}
```

---

### 27. `validate_mesh`

Выполняет строгую топологическую диагностику полигональной сетки детали на замкнутость (**Watertight**), наличие открытых граничных ребер ($\partial M = \emptyset$), не-многообразных (non-manifold) ребер, вырожденных треугольников нулевой площади, а также вычисляет характеристику Эйлера — Пуанкаре ($\chi = V - E + F$) и топологический род поверхности ($g = 1 - \chi / 2$).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `part_id` | `string` | **Да** | Идентификатор инспектируемой детали |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 27,
  "method": "tools/call",
  "params": {
    "name": "validate_mesh",
    "arguments": {
      "part_id": "part-bushing-opt"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 27,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-bushing-opt\",\n  \"is_watertight\": true,\n  \"is_manifold\": true,\n  \"boundary_edges_count\": 0,\n  \"non_manifold_edges_count\": 0,\n  \"degenerate_triangles_count\": 0,\n  \"euler_characteristic\": 0,\n  \"genus\": 1,\n  \"vertices_count\": 256,\n  \"triangles_count\": 512,\n  \"status\": \"PASS\"\n}"
      }
    ]
  }
}
```

---

### 28. `optimize_feature_tree`

Запускает **Cost-Based CAD Query Optimizer (`EXPLAIN ANALYZE`)** над деревом операций Solid DAG детали. Оптимизатор анализирует построенную ИИ-агентом логическую цепочку шагов, применяет правила алгебраической и геометрической эквивалентной перезаписи, оценивает кандидатов по функции стоимости (Cost Model) и проверяет сохранение топологических инвариантов (объем $\Delta V < 0.5\%$, AABB и замкнутость сетки Watertight).

Поддерживаемые правила перезаписи:
- `R1_TRANSFORM_FUSION`: Свертка цепочки матриц трансформаций $T_n \circ \dots \circ T_1 \to \mathbf{M} \in SE(3)$, удаление нейтральных сдвигов и проталкивание (Pushdown) смещения в примитив/отверстие.
- `R2_AABB_DEAD_OP_CULLING`: Отсечение «мертвых» булевых операций и отверстий вне габарита детали ($\text{AABB}_{\text{tool}} \cap \text{AABB}_{\text{target}} = \emptyset$) и дедупликация совпадающих отверстий.
- `R3_DEMORGAN_CSG_BATCH`: Группировка серии компланарных `cut_hole` по закону Де Моргана $A \setminus B_1 \setminus \dots \setminus B_k \equiv A \setminus \bigcup B_i$ в единый проход перфорации профиля.
- `R4_COAXIAL_REVOLVE_REDUCTION`: Понижение размерности соосного цилиндра и центрального отверстия (`cylinder` + `cut_hole`) до однопроходного вращения 2D кольцевого профиля (`revolve` с `inner_radius`), гарантирующее 100% замкнутую сетку без 3D CSG.
- `R5_ANALYTICAL_PRISM_THREAD_FUSION`: Слияние шестигранной призмы (`polygon_prism`), отверстия (`cut_hole`) и метрической резьбы (`cut_helical_groove`) в единый аналитический проход генерации резьбовой детали.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `part_id` | `string` | **Да** | Идентификатор оптимизируемой детали |
| `apply` | `boolean` | Нет | Применить ли скомпилированный физический план к детали и перестроить 3D-сетку (по умолчанию `true`; при `false` работает в режиме инспекции `EXPLAIN ANALYZE` без мутации DAG; если ни одно правило не сработало, `revision` не инкрементируется) |
| `length_unit` | `"mm" \| "m"` | Нет | Опциональное переопределение единицы длины для этого вызова (по умолчанию используется единица сессии) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 28,
  "method": "tools/call",
  "params": {
    "name": "optimize_feature_tree",
    "arguments": {
      "part_id": "part-bushing-opt",
      "apply": true
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 28,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"part_id\": \"part-bushing-opt\",\n  \"revision\": 6,\n  \"status\": \"optimized\",\n  \"applied\": true,\n  \"logical_nodes_count\": 5,\n  \"physical_nodes_count\": 1,\n  \"logical_cost\": 562.4,\n  \"physical_cost\": 98.2,\n  \"cost_reduction_pct\": 82.5,\n  \"estimated_speedup_x\": 5.73,\n  \"applied_rules\": [\n    \"R2_AABB_DEAD_OP_CULLING\",\n    \"R1_TRANSFORM_FUSION\",\n    \"R4_COAXIAL_REVOLVE_REDUCTION\"\n  ],\n  \"volume\": 79420.0,\n  \"volume_m3\": 0.00007942,\n  \"equivalence_verification\": {\n    \"passed\": true,\n    \"volume_before\": 79420.0,\n    \"volume_after\": 79420.0,\n    \"volume_before_cm3\": 79.42,\n    \"volume_after_cm3\": 79.42,\n    \"volume_delta_pct\": 0.0,\n    \"watertight_before\": false,\n    \"watertight_after\": true\n  },\n  \"explain_analyze_text\": \"EXPLAIN (ANALYZE, COSTS, TOPOLOGY) FOR PART 'AI_Coaxial_Bushing_Unoptimized' ...\",\n  \"length_unit\": \"mm\"\n}"
      }
    ]
  }
}
```

---

### 29. `set_units`

Устанавливает активную единицу длины сессии (`EngineState::length_unit`: `"mm"` или `"m"`) для всех последующих вызовов инструментов MCP. Уже созданные в сцене детали хранятся во внутреннем ядре в СИ (`m`) и не требуют перестроения при переключении `set_units`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `length_unit` | `string` | **Да** | Активная единица длины: `"mm"` (миллиметры, по умолчанию при старте сервера) или `"m"` (метры) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 29,
  "method": "tools/call",
  "params": {
    "name": "set_units",
    "arguments": {
      "length_unit": "mm"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 29,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"status\": \"units_updated\",\n  \"previous_length_unit\": \"m\",\n  \"length_unit\": \"mm\",\n  \"units\": {\n    \"length\": \"mm\",\n    \"area\": \"mm²\",\n    \"volume\": \"mm³\",\n    \"volume_si\": \"m³\",\n    \"mass\": \"kg\",\n    \"density\": \"kg/m³\",\n    \"force\": \"N\",\n    \"torque\": \"N·m\",\n    \"stress\": \"MPa\",\n    \"stress_output\": \"MPa\",\n    \"young_modulus\": \"GPa\",\n    \"displacement\": \"mm\",\n    \"displacement_output\": \"mm\",\n    \"angle\": \"deg\",\n    \"angular_velocity\": \"rad/s\",\n    \"linear_velocity\": \"m/s\",\n    \"time\": \"s\"\n  }\n}"
      }
    ]
  }
}
```

---

## Группа 7: Персистентность и управление проектами

### 30. `save_project`

Сохраняет текущее полное состояние CAD-движка (детали с их параметрическими деревьями фич DAG, пользовательские материалы из `define_material`, кинематические шарниры с моторами из `physics_world`, активный `length_unit` и контракт единиц) в версионированный JSON-файл проекта на диске (`schema_version = "1.0.0"`).

Полигональные поверхностные сетки треугольников не сериализуются для процедурных деталей (пересчитываются из DAG при загрузке), что обеспечивает компактность файла (~несколько килобайт вместо сотен мегабайт) и сохранение полной параметричности.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `path` | `string` | **Да** | Путь к сохраняемому JSON-файлу проекта (например, `"./output/crank_slider_mechanism.json"`) |
| `project_name` | `string` | Нет | Человекочитаемое название проекта (по умолчанию `"MCP-CAD Project"`) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 30,
  "method": "tools/call",
  "params": {
    "name": "save_project",
    "arguments": {
      "path": "./output/my_assembly.json",
      "project_name": "Linkage Arm Assembly"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 30,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"status\": \"saved\",\n  \"path\": \"./output/my_assembly.json\",\n  \"schema_version\": \"1.0.0\",\n  \"project\": \"Linkage Arm Assembly\",\n  \"parts_count\": 2,\n  \"joints_count\": 1,\n  \"materials_count\": 1,\n  \"file_size_bytes\": 3280,\n  \"length_unit\": \"mm\",\n  \"units\": {\n    \"length\": \"mm\",\n    \"mass\": \"kg\",\n    \"force\": \"N\"\n  }\n}"
      }
    ]
  }
}
```

---

### 31. `load_project`

Транзакционно загружает проект из JSON-файла, полностью заменяя текущее состояние сессии (материалы, детали, симулятор физики и шарниры, единицы длины). Пересчитывает параметрические деревья DAG для каждой детали, восстанавливая точные граничные 3D-сетки и инерционные характеристики.

**Транзакционность:** Если файл повреждён, отсутствует, содержит синтаксические ошибки JSON или несовместимую версию схемы (`schema_version`), операция немедленно возвращает структурированную ошибку (`IO_ERROR`, `JSON_ERROR` или `PROJECT_SCHEMA_MISMATCH`), а текущее состояние сессии остаётся нетронутым.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `path` | `string` | **Да** | Путь к загружаемому JSON-файлу проекта |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 31,
  "method": "tools/call",
  "params": {
    "name": "load_project",
    "arguments": {
      "path": "./output/my_assembly.json"
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 31,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "{\n  \"status\": \"loaded\",\n  \"path\": \"./output/my_assembly.json\",\n  \"schema_version\": \"1.0.0\",\n  \"project\": \"Linkage Arm Assembly\",\n  \"parts_count\": 2,\n  \"joints_count\": 1,\n  \"materials_count\": 9,\n  \"length_unit\": \"mm\",\n  \"units\": {\n    \"length\": \"mm\"\n  },\n  \"parts\": [\n    {\n      \"part_id\": \"part_base\",\n      \"name\": \"Base Plate\",\n      \"revision\": 1,\n      \"material\": \"inconel_718\",\n      \"volume\": 48000.0,\n      \"volume_m3\": 0.000048,\n      \"mass_kg\": 0.3931,\n      \"triangles_count\": 12,\n      \"is_fixed\": true\n    },\n    {\n      \"part_id\": \"part_arm\",\n      \"name\": \"Link Arm\",\n      \"revision\": 1,\n      \"material\": \"aluminum_6061_t6\",\n      \"volume\": 10053.09,\n      \"volume_m3\": 0.000010053,\n      \"mass_kg\": 0.0271,\n      \"triangles_count\": 124,\n      \"is_fixed\": false\n    }\n  ],\n  \"joints\": [\n    {\n      \"joint_id\": \"joint-1\",\n      \"part_a\": \"part_base\",\n      \"part_b\": \"part_arm\",\n      \"joint_type\": \"revolute\",\n      \"anchor_a\": [0.0, 20.0, 0.0],\n      \"anchor_b\": [0.0, 0.0, 0.0],\n      \"axis\": [0.0, 1.0, 0.0],\n      \"limits\": [-45.0, 90.0],\n      \"limits_unit\": \"deg\",\n      \"ratio\": null,\n      \"has_motor\": true,\n      \"motor\": {\n        \"target_velocity\": 5.0,\n        \"max_force\": 50.0\n      }\n    }\n  ]\n}"
      }
    ]
  }
}
```

---

## Группа 8: Слой инженерных требований (Requirements Layer)

### 32. `set_requirements`

Объявляет формальные инженерные требования к конкретной детали (`part_id`) или ко всей сцене (`scope: "scene"`). Поддерживает режим автоматической обратной связи (`auto_check: true`), при котором каждый мутирующий CAD-инструмент (`add_feature_primitive`, `cut_hole`, `modify_feature`, `cut_helical_groove`, `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part`, `batch_cad_operations`, `optimize_feature_tree`, `import_stl`) автоматически добавляет в свой JSON-ответ сводку `requirements_status` и список нарушенных требований `failed_requirements`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `part_id` | `string` | Нет* | Идентификатор целевой детали (обязателен при `scope = "part"`, не указывается при `scope = "scene"`) |
| `scope` | `string` | Нет | Область действия требований: `"part"` (по умолчанию при наличии `part_id`) или `"scene"` (сборка целиком) |
| `length_unit` | `string` | Нет | Локальное переопределение единицы длины (`"mm"` или `"m"`) для габаритов `bbox_max` / `bbox_min` |
| `auto_check` | `boolean` | Нет | Если `true`, включает автоматическое добавление `requirements_status` и `failed_requirements` в ответы мутирующих CAD-инструментов |
| `requirements` | `array<object>` | **Да** | Массив спецификаций инженерных требований |

#### Поддерживаемые типы требований (`requirements[].type`)
| `type` | Параметры | Описание |
|---|---|---|
| `"bbox_max"` | `"dims": [dx, dy, dz]` | Максимально допустимые габариты AABB детали вдоль осей `[X, Y, Z]` в активном `length_unit` (`mm` или `m`) |
| `"bbox_min"` | `"dims": [dx, dy, dz]` | Минимально допустимые габариты AABB детали вдоль осей `[X, Y, Z]` в активном `length_unit` (`mm` или `m`) |
| `"mass_max"` | `"kg": number` | Максимально допустимая расчётная масса детали в килограммах (`kg`) |
| `"mesh_valid"` | `"expected_genus"?: integer` | Требование прохождения 12-пунктовой топологической валидации `validate_mesh` (`status == "PASS"`) с опциональной проверкой рода поверхности `expected_genus` |
| `"min_safety_factor"` | `"value": number, "fea_case"?: string` | Минимальный коэффициент запаса прочности по Мизесу (`safety_factor >= value`). Не перезапускает МКЭ автоматически: если `part.revision` изменился после последнего вызова `run_fea_analysis`, возвращает статус `"STALE"` |
| `"no_interference"` | `"pairs"?: [[part_a, part_b], ...]` | Отсутствие геометрических коллизий/пересечений между деталями (возвращает `"NOT_AVAILABLE"` до реализации `P3-T10`) |
| `"mechanism_dof"` | `"expected": integer` | Ожидаемое число степеней свободы кинематической цепи механизма (возвращает `"NOT_AVAILABLE"` до реализации `P3-T9`) |

---

### 33. `check_requirements`

Проверяет выполнение зарегистрированных инженерных требований для указанной детали (`part_id`) или для всей сцены (если `part_id` не передан). Возвращает по каждому требованию статус (`PASS | FAIL | STALE | NOT_AVAILABLE`), измеренное значение `measured`, целевой порог `target`, запас `margin` и действенную подсказку `hint`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `part_id` | `string` | Нет | Опциональный идентификатор детали для фильтрации проверки только по одной детали |
| `length_unit` | `string` | Нет | Локальное переопределение единицы длины (`"mm"` или `"m"`) для возвращаемых габаритов и запасов |

---

### 34. `get_scene_summary`

Возвращает компактную текстовую (`summary_text`) и структурированную JSON-сводку всей сцены для первого обращения ИИ-агента в новом контексте (или после загрузки проекта через `load_project`).

Сводка включает:
- **Единицы проекта**: активный `length_unit` (`"mm"` или `"m"`) и полный словарь `"units"`.
- **Детали (`parts`)**: `part_id`, `name`, `material`, `is_fixed`, `revision`, `features_count`, `bounding_box` и `dimensions` `[dx, dy, dz]` в активном `length_unit`, `volume` (`mm³` или `m³`) и `volume_m3`, `mass_kg`, `triangles_count`, валидность сетки (`mesh_valid`, `mesh_status`: `PASS | FAIL | WARN | EMPTY`, `is_watertight`, `is_manifold`, `open_edges_count`, `genus`), а также статус последнего расчёта МКЭ (`last_fea`: `verdict`, `factor_of_safety`, `max_von_mises_mpa`, `max_displacement_mm`, `fea_revision`, `part_revision`, `stale`, `status: "FRESH" | "STALE"`).
- **Кинематические шарниры (`joints`)**: `joint_id`, `joint_type` (`revolute`, `prismatic`, `spherical`, `fixed`, `gear`), соединённые детали `part_a` и `part_b`, якоря `anchor_a`/`anchor_b` в активном `length_unit`, ось `axis` (для `gear` равна `axis_a`), оси вращения валов `axis_a` и `axis_b` (для `gear`), ограничения хода `limits` (`[min, max]` в `deg` для `revolute` или в активном `length_unit` для `prismatic`, иначе `null`), единица ограничений `limits_unit` (`"deg"`, `"mm"`/`"m"` или `null`), передаточное отношение `ratio` (для `gear`) и параметры мотора `motor` (`target_velocity`, `max_force`).
- **Инженерные требования (`requirements`)**: общий статус (`PASS | FAIL | STALE | NOT_AVAILABLE`), флаг `auto_check`, счётчики `total`, `passed`, `failed`, `stale`, `not_available`, полный список проверенных требований `items` и список нарушений `failed_requirements`.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `length_unit` | `string` | Нет | Локальное переопределение единицы длины (`"mm"` или `"m"`, по умолчанию используется единица сессии) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 34,
  "method": "tools/call",
  "params": {
    "name": "get_scene_summary",
    "arguments": {}
  }
}
```



