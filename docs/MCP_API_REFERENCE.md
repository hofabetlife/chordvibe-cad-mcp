# MCP-CAD Engine: Справочник API инструментов (Tools Reference)

Спецификация протокола: **Model Context Protocol (MCP)**, версия `2024-11-05`.  
Формат сообщений: **JSON-RPC 2.0**.

---

## Оглавление

1. [Общие положения](#общие-положения)
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
7. [Группа 6: Топология, CSG операции, кастомные материалы и пакетная оптимизация](#группа-6-топология-csg-операции-кастомные-материалы-и-пакетная-оптимизация)
   - [21. `query_part_geometry`](#21-query_part_geometry)
   - [22. `boolean_union`](#22-boolean_union)
   - [23. `boolean_intersection`](#23-boolean_intersection)
   - [24. `transform_part`](#24-transform_part)
   - [25. `define_material`](#25-define_material)
   - [26. `batch_cad_operations`](#26-batch_cad_operations)

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

---

## Группа 1: Управление деталями и материалами

### 1. `create_part`

Создает новую пустую деталь и прикрепляет к ней инженерный материал из реестра.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `name` | `string` | Да | Человекочитаемое имя детали (например, `"mounting_bracket"`) |
| `material` | `string` | Да | Идентификатор материала из базы пресетов |
| `is_fixed` | `boolean` | Нет | Закрепить деталь неподвижно в мировом пространстве (статическое тело с бесконечной массой для физических расчетов и кинематики, например станина или корпус редуктора). По умолчанию `false` |

**Доступные пресеты материалов**:
- `aluminum_6061_t6`: Авиационный алюминиевый сплав ($E=68.9$ GPa, $\sigma_y=276$ MPa, $\rho=2700$ kg/m³).
- `steel_structural_aisi1020`: Конструкционная углеродистая сталь ($E=205.0$ GPa, $\sigma_y=250$ MPa, $\rho=7850$ kg/m³).
- `steel_high_strength_4140`: Высокопрочная легированная сталь ($E=210.0$ GPa, $\sigma_y=655$ MPa, $\rho=7850$ kg/m³).
- `titanium_grade5_ti6al4v`: Титановый сплав Grade 5 ($E=114.0$ GPa, $\sigma_y=880$ MPa, $\rho=4430$ kg/m³).
- `brass_c360`: Автоматная свинцовая латунь ($E=100.0$ GPa, $\sigma_y=310$ MPa, $\rho=8500$ kg/m³).
- `pla_filament_3dprint`: Пластик PLA для 3D-печати ($E=3.5$ GPa, $\sigma_y=60$ MPa, $\rho=1240$ kg/m³).
- `petg_filament_3dprint`: Пластик PETG для 3D-печати ($E=2.1$ GPa, $\sigma_y=50$ MPa, $\rho=1270$ kg/m³).
- `nylon_pa12`: Инженерный полиамид PA12 ($E=1.8$ GPa, $\sigma_y=45$ MPa, $\rho=1010$ kg/m³).

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

Возвращает список всех активных деталей на сцене с геометрическими габаритами, объемом, расчетной массой и количеством треугольников в сетке.

#### Входные параметры
*Нет параметров.*

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/call",
  "params": {
    "name": "list_parts",
    "arguments": {}
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
        "text": "{\n  \"parts\": [\n    {\n      \"part_id\": \"part-a1b2c3d4e5f64789\",\n      \"name\": \"cantilever_beam\",\n      \"material\": \"steel_structural_aisi1020\",\n      \"volume_m3\": 0.001,\n      \"mass_kg\": 7.85,\n      \"triangle_count\": 12,\n      \"bounding_box\": {\n        \"min\": [-0.5, -0.025, -0.01],\n        \"max\": [0.5, 0.025, 0.01]\n      }\n    }\n  ]\n}"
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
| `part_id` | `string` | Да | Уникальный идентификатор детали |

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
        "text": "{\n  \"status\": \"deleted\"\n}"
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
- `"polygon_prism"`: `{"height": number, "sides"?: integer, "width_across_flats"?: number, "radius"?: number, "chamfer"?: number}` (правильная призма для гаек/болтов с коническими торцевыми фасками 30° по DIN 934)
- `"spur_gear"`: `{"module": number, "teeth": integer, "face_width": number, "pressure_angle_deg"?: number, "bore_radius"?: number, "bore_diameter"?: number, "keyway_width"?: number, "keyway_depth"?: number, "hub_diameter"?: number, "hub_length"?: number}` (прямозубая цилиндрическая шестерня с точным эвольвентным профилем по DIN 867 / ISO 53, опциональной ступицей и шпоночным пазом DIN 6885)
- `"revolve"`: `{"profile": [[r, z], ...], "angle_deg"?: number, "segments"?: integer}` (тело вращения 2D-профиля вокруг оси Z; угол поворота `angle_deg` по умолчанию 360°, с плоскими крышками для углов < 360°)
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
        "text": "{\n  \"feature_id\": \"d7a8e2f0-1234-5678-90ab-cdef11223344\"\n}"
      }
    ]
  }
}
```

---

### 5. `cut_hole`

Вырезает сквозное или глухое цилиндрическое отверстие вдоль заданного нормального вектора, с опциональным шпоночным пазом по стандарту DIN 6885.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор целевой детали |
| `position` | `[number, number, number]` | Да | Координаты точки входа отверстия `[x, y, z]` (м) |
| `normal` | `[number, number, number]` | Да | Единичный вектор направления оси отверстия `[nx, ny, nz]` |
| `radius` | `number` | Да | Радиус отверстия в метрах |
| `depth` | `number` | Да | Глубина выреза в метрах |
| `keyway_width` | `number` | Нет | Ширина шпоночного паза DIN 6885 в метрах |
| `keyway_depth` | `number` | Нет | Глубина шпоночного паза DIN 6885 в стенку в метрах |

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
        "text": "{\n  \"feature_id\": \"f8b9c0d1-2345-6789-01bc-def223344556\"\n}"
      }
    ]
  }
}
```

---

### 6. `modify_feature`

Модифицирует численные параметры любого существующего шага в истории DAG без разрушения структуры детали.

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
        "text": "{\n  \"status\": \"updated\",\n  \"updated_bounds\": [-0.125, -0.03, -0.025, 0.125, 0.03, 0.025],\n  \"dimensions\": [0.25, 0.06, 0.05]\n}"
      }
    ]
  }
}
```

---

### 7. `get_feature_tree`

Возвращает полный упорядоченный список всех узлов истории операций DAG для анализа цепочки моделирования.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор целевой детали |

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
        "text": "{\n  \"dag\": [\n    {\n      \"id\": \"d7a8e2f0-1234-5678-90ab-cdef11223344\",\n      \"name\": \"AddBox_1\",\n      \"op\": {\n        \"AddBox\": {\n          \"width\": 0.25,\n          \"height\": 0.06,\n          \"depth\": 0.05\n        }\n      },\n      \"parent_id\": null\n    }\n  ]\n}"
      }
    ]
  }
}
```

---

### 8. `cut_helical_groove`

Вырезает винтовые спиральные канавки (стружечные канавки сверл, теплоотводящие спирали, нарезка метрической треугольной резьбы 60° по ISO 68-1).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор детали |
| `helix_radius` | `number` | Да | Радиус центральной траектории спирали (м) |
| `groove_radius` | `number` | Да | Радиус сечения или глубина резьбы (м) |
| `pitch` | `number` | Да | Осевой шаг винта (м на оборот 360°) |
| `height` | `number` | Да | Полная осевая длина спирали (м) |
| `start_z` | `number` | Да | Z-координата начала спирали (м) |
| `starts` | `integer` | Нет | Число заходов резьбы/канавок (по умолчанию 2) |
| `is_right_hand` | `boolean` | Нет | Направление нарезки (true - правая, false - левая, по умолчанию true) |
| `profile_type` | `string` | Нет | Профиль сечения инструмента: `"circular"` (по умолчанию) или `"metric_60"` (треугольный профиль метрической резьбы 60° по ISO 68-1) |

---

### 9. `boolean_difference`

Выполняет точное аналитическое булево вычитание сетки инструмента из целевой детали (CSG Difference).

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `target_part_id` | `string` | Да | Идентификатор детали, из которой вычитается объем |
| `tool_part_id` | `string` | Да | Идентификатор детали-инструмента |

---

### 10. `tessellate_part`

Выполняет ленивую адаптивную тесселяцию криволинейных поверхностей детали по хорде и углу отклонения с обновлением кэша полигональной сетки.

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
| `joint_type` | `string` | Да | `"revolute"`, `"prismatic"`, `"spherical"`, `"fixed"`, `"gear"` |
| `part_a` | `string` | Да | Идентификатор первой сопрягаемой детали (или `"ground"` / `"world"` для привязки к неподвижному пространству) |
| `part_b` | `string` | Да | Идентификатор второй сопрягаемой детали (или `"ground"` / `"world"`) |
| `anchor_a` | `[number, number, number]` | Нет | Точка крепления `[x, y, z]` в СК детали A (м). По умолчанию `[0, 0, 0]` |
| `anchor_b` | `[number, number, number]` | Нет | Точка крепления `[x, y, z]` в СК детали B (м). По умолчанию `[0, 0, 0]` |
| `axis` | `[number, number, number]` | Нет | Направляющая ось `[x, y, z]` (обязательна для revolute/prismatic) |
| `limits` | `[number, number]` | Нет | Пределы движения `[min, max]` (в радианах или метрах) |
| `ratio` | `number` | Нет | Передаточное отношение для `joint_type: "gear"` ($i = \omega_B / \omega_A$). Отрицательное значение указывает на встречное вращение (внешнее зацепление) |
| `teeth_a` | `integer` | Нет | Число зубьев ведущей шестерни $z_A$ (альтернатива `ratio`; рассчитывает $i = -z_A / z_B$) |
| `teeth_b` | `integer` | Нет | Число зубьев ведомой шестерни $z_B$ (альтернатива `ratio`) |
| `axis_a` | `[number, number, number]` | Нет | Ось вращения шестерни A (по умолчанию `[0, 0, 1]`) |
| `axis_b` | `[number, number, number]` | Нет | Ось вращения шестерни B (по умолчанию `[0, 0, 1]`) |

> **Примечание по зубчатым передачам (`"gear"`):**  
> При создании сопряжения типа `"gear"` физический движок автоматически отключает прямой полигональный контакт между зубьями (`contacts_enabled(false)`), исключая заклинивание выпуклых оболочек colliders, и на каждом шаге времени строго синхронизирует угловые скорости шестерней согласно передаточному отношению $\omega_B = -\text{ratio} \cdot \omega_A$.

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

Активирует сервопривод или мотор поддержания скорости на сочленении.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `joint_id` | `string` | Да | Идентификатор сочленения |
| `target_velocity` | `number` | Да | Целевая скорость: угловая в рад/с или линейная в м/с |
| `max_force` | `number` | Да | Предельный крутящий момент ($Н\cdot м$) или сила ($Н$) |

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
        "text": "{\n  \"status\": \"motor_active\"\n}"
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

Выполняет автоматическую тетраэдризацию детали (TET4), решает 3D уравнения линейной упругости ($K \cdot u = F$), вычисляет эквивалентные напряжения по Мизесу и запас прочности.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор анализируемой детали |
| `mesh_density` | `string` | Нет | Плотность сетки: `"coarse"` (по умолчанию), `"medium"`, `"fine"` |
| `fixed_faces` | `string[]` | Нет | Грани AABB для жесткого закрепления: `"min_x"`, `"max_x"`, `"min_y"`, `"max_y"`, `"min_z"`, `"max_z"` |
| `fixed_plane` | `object` | Нет | Опорная плоскость: `{"origin": [x,y,z], "normal": [nx,ny,nz], "tolerance"?: number}` |
| `applied_force` | `object` | Нет | Одиночная нагрузка: `{"position"?: [x,y,z], "force_vector": [Fx,Fy,Fz]}` |
| `applied_forces` | `object[]` | Нет | Массив распределенных или точечных нагрузок |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 14,
  "method": "tools/call",
  "params": {
    "name": "run_fea_analysis",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789",
      "mesh_density": "medium",
      "fixed_faces": ["min_x"],
      "applied_forces": [
        {
          "position": [0.20, 0.025, 0.0],
          "force_vector": [0.0, -1200.0, 0.0]
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
        "text": "{\n  \"computation_time_ms\": 245,\n  \"element_count\": 864,\n  \"max_displacement_mm\": 0.184,\n  \"max_von_mises_mpa\": 98.42,\n  \"node_count\": 275,\n  \"safety_factor\": 2.54,\n  \"status\": \"PASS\",\n  \"yield_strength_mpa\": 250.0\n}"
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

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `filepath` | `string` | Да | Путь к STL-файлу на диске |
| `name` | `string` | Нет | Пользовательское имя детали (по умолчанию имя файла) |
| `material` | `string` | Нет | Идентификатор материала из реестра (по умолчанию `steel_structural_aisi1020`) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 18,
  "method": "tools/call",
  "params": {
    "name": "import_stl",
    "arguments": {
      "filepath": "./output/external_bracket.stl",
      "name": "imported_bracket",
      "material": "aluminum_6061_t6"
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
        "text": "{\n  \"material\": \"aluminum_6061_t6\",\n  \"name\": \"imported_bracket\",\n  \"part_id\": \"part-imported-1234\",\n  \"status\": \"stl_imported\",\n  \"triangles_count\": 5420,\n  \"vertices_count\": 2712\n}"
      }
    ]
  }
}
```

---

### 19. `export_stl`

Экспортирует поверхностную сетку детали в файл стандартного формата STL с поддержкой адаптивной тесселяции по хорде для прецизионного производства и 3D-печати.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор детали |
| `filepath` | `string` | Да | Путь для сохранения файла на диске (например, `"./output/bracket.stl"`) |
| `binary` | `boolean` | Нет | `true` — компактный Binary STL (по умолчанию); `false` — ASCII STL |
| `quality` | `string` | Нет | Пресет тесселяции: `"coarse"`, `"medium"`, `"fine"` (по умолчанию: `"fine"`) |
| `chord_tolerance` | `number` | Нет | Пользовательский допуск хорды (стрела прогиба) в метрах (например, `0.00005`) |

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
        "text": "{\n  \"file_path\": \"./output/cantilever_beam.stl\",\n  \"triangles_count\": 2480\n}"
      }
    ]
  }
}
```

---

### 20. `get_simulation_metrics_json`

Возвращает полный агрегированный инженерный отчет по всей сцене: геометрические метаданные всех деталей, результаты последних МКЭ-расчетов и телеметрию физической симуляции.

#### Входные параметры
*Нет параметров.*

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
        "text": "{\n  \"current_metrics\": {\n    \"parts_count\": 1,\n    \"parts\": [\n      {\n        \"part_id\": \"part-a1b2c3d4e5f64789\",\n        \"name\": \"cantilever_beam\",\n        \"material\": \"steel_structural_aisi1020\",\n        \"volume_m3\": 0.001,\n        \"mass_kg\": 7.85,\n        \"triangle_count\": 12,\n        \"bounding_box\": {\"min\": [-0.5, -0.025, -0.01], \"max\": [0.5, 0.025, 0.01]}\n      }\n    ],\n    \"fea_reports\": [\n      {\n        \"part_id\": \"part-a1b2c3d4e5f64789\",\n        \"verdict\": \"PASS\",\n        \"factor_of_safety\": 2.54,\n        \"max_von_mises_mpa\": 98.42,\n        \"yield_strength_mpa\": 250.0,\n        \"max_displacement_mm\": 0.184,\n        \"nodes\": 275,\n        \"elements\": 864,\n        \"time_ms\": 245\n      }\n    ],\n    \"physics\": {\n      \"accumulated_time_s\": 1.5,\n      \"gravity\": [0.0, -9.81, 0.0],\n      \"last_simulation\": {\n        \"step_count\": 90,\n        \"peak_joint_forces\": {\"joint-550e8400-e29b-41d4-a716-446655440000\": 34.2},\n        \"collisions\": []\n      }\n    }\n  }\n}"
      }
    ]
  }
}
```

---

## Группа 6: Топология, CSG операции, кастомные материалы и пакетная оптимизация

### 21. `query_part_geometry`

Выполняет глубокую топологическую и геометрическую инспекцию твердотельной детали. Возвращает точный габаритный ящик AABB (`min`, `max`, `dimensions`), абсолютный объем, площадь поверхности, центр масс (centroid), количество полигонов и вершин, а также массив до 15 крупнейших компланарных плоских граней (`detected_planar_faces`) с вычисленными нормалями и средневзвешенными центрами. Это исключает проблему "слепоты координат" у ИИ-агентов при сверлении отверстий и закреплении граней.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `part_id` | `string` | **Да** | Уникальный идентификатор инспектируемой детали |

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
        "text": "{\n  \"part_id\": \"part-bracket-001\",\n  \"bounding_box\": {\n    \"min\": [-0.06, -0.015, -0.03],\n    \"max\": [0.06, 0.015, 0.03],\n    \"dimensions\": [0.12, 0.03, 0.06]\n  },\n  \"volume\": 0.000216,\n  \"surface_area\": 0.0252,\n  \"center_of_mass\": [0.0, 0.0, 0.0],\n  \"faces_count\": 12,\n  \"vertices_count\": 24,\n  \"detected_planar_faces\": [\n    {\n      \"normal\": [0.0, 1.0, 0.0],\n      \"center\": [0.0, 0.015, 0.0],\n      \"area\": 0.0072,\n      \"triangles_count\": 2\n    },\n    {\n      \"normal\": [0.0, -1.0, 0.0],\n      \"center\": [0.0, -0.015, 0.0],\n      \"area\": 0.0072,\n      \"triangles_count\": 2\n    },\n    {\n      \"normal\": [1.0, 0.0, 0.0],\n      \"center\": [0.06, 0.0, 0.0],\n      \"area\": 0.0018,\n      \"triangles_count\": 2\n    },\n    {\n      \"normal\": [-1.0, 0.0, 0.0],\n      \"center\": [-0.06, 0.0, 0.0],\n      \"area\": 0.0018,\n      \"triangles_count\": 2\n    }\n  ]\n}"
      }
    ]
  }
}
```

---

### 22. `boolean_union`

Выполняет CSG булево объединение (Union) твердотельных сеток: объединяет форму детали-инструмента (`tool_part_id`) с целевой деталью (`target_part_id`). Добавляет узел `BooleanUnionMesh` в параметрическое дерево DAG целевой детали, пересчитывает геометрию, обновляет отображение во вьюпорте и синхронизирует физический коллайдер в Rapier3D.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `target_part_id` | `string` | **Да** | ID целевой детали, принимающей объединение |
| `tool_part_id` | `string` | **Да** | ID детали-инструмента, чья сетка объединяется с целевой (должен отличаться от target_part_id) |

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
        "text": "{\n  \"target_part_id\": \"part-housing-main\",\n  \"feature_id\": \"9b1deb4d-3b7d-4bad-9bdd-2b0d7b3dcb6d\",\n  \"status\": \"union_applied\",\n  \"vertices_count\": 842,\n  \"triangles_count\": 1680\n}"
      }
    ]
  }
}
```

---

### 23. `boolean_intersection`

Выполняет CSG булево пересечение (Intersection) твердотельных сеток: сохраняет только общий объем пересечения целевой детали (`target_part_id`) и инструмента (`tool_part_id`). Добавляет узел `BooleanIntersectionMesh` в параметрическое дерево DAG, перестраивает меш, обновляет вьюпорт и физический мир.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `target_part_id` | `string` | **Да** | ID целевой детали |
| `tool_part_id` | `string` | **Да** | ID детали-инструмента (должен отличаться от target_part_id) |

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
        "text": "{\n  \"target_part_id\": \"part-cylinder-base\",\n  \"feature_id\": \"1f38e4a9-82bc-4402-9988-6617a23c3a4f\",\n  \"status\": \"intersection_applied\",\n  \"vertices_count\": 512,\n  \"triangles_count\": 1020\n}"
      }
    ]
  }
}
```

---

### 24. `transform_part`

Применяет пространственные преобразования к твердотельному телу в дереве построений DAG (`FeatureOp::Transform`): линейное перемещение (`translation`), углы вращения Эйлера Roll-Pitch-Yaw в градусах (`rotation_deg`) и масштабирование (`scale`). Пересчитывает габариты AABB, объем и обновляет физические свойства.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|:---:|---|
| `part_id` | `string` | **Да** | ID трансформируемой детали |
| `translation` | `[f64; 3]` | Нет | Вектор смещения $[x, y, z]$ в метрах (по умолчанию `[0.0, 0.0, 0.0]`) |
| `rotation_deg` | `[f64; 3]` | Нет | Углы поворота $[roll_x, pitch_y, yaw_z]$ в градусах (по умолчанию `[0.0, 0.0, 0.0]`) |
| `scale` | `[f64; 3]` | Нет | Коэффициенты масштабирования $[s_x, s_y, s_z]$ (по умолчанию `[1.0, 1.0, 1.0]`, компоненты $\ne 0$) |

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
        "text": "{\n  \"part_id\": \"part-bracket-001\",\n  \"feature_id\": \"7c28a1e4-6a84-48f5-93df-f41cb87a1928\",\n  \"status\": \"transformed\",\n  \"bounding_box\": {\n    \"min\": [0.02, -0.015, 0.04],\n    \"max\": [0.08, 0.015, 0.16],\n    \"dimensions\": [0.06, 0.03, 0.12]\n  },\n  \"volume\": 0.000216\n}"
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

Последовательно выполняет массив инструментов MCP CAD в едином контексте движка за один сетевой запрос JSON-RPC. Инструмент устраняет десятки повторных сетевых вызовов (roundtrip latency) при моделировании составных деталей. Все шаги валидируются и выполняются по порядку. Если любая операция завершается с ошибкой, выполнение немедленно прерывается с указанием индекса сбойного шага и подробной диагностикой. Вложенные вызовы `batch_cad_operations` запрещены.

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
