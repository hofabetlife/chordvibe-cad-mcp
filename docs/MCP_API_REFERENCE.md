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
3. [Группа 2: Параметрическое DAG-моделирование](#группа-2-параметрическое-dag-моделирование)
   - [4. `add_feature_primitive`](#4-add_feature_primitive)
   - [5. `cut_hole`](#5-cut_hole)
   - [6. `modify_feature`](#6-modify_feature)
   - [7. `get_feature_tree`](#7-get_feature_tree)
4. [Группа 3: Кинематика и физика механизмов](#группа-3-кинематика-и-физика-механизмов)
   - [8. `create_joint`](#8-create_joint)
   - [9. `set_joint_motor`](#9-set_joint_motor)
   - [10. `run_physics_simulation`](#10-run_physics_simulation)
5. [Группа 4: Конечно-элементный анализ прочности (FEA)](#группа-4-конечно-элементный-анализ-прочности-fea)
   - [11. `run_fea_analysis`](#11-run_fea_analysis)
   - [12. `get_fea_heatmap_image`](#12-get_fea_heatmap_image)
6. [Группа 5: Визуализация, экспорт и метрики](#группа-5-визуализация-экспорт-и-метрики)
   - [13. `capture_viewport`](#13-capture_viewport)
   - [14. `export_stl`](#14-export_stl)
   - [15. `get_simulation_metrics_json`](#15-get_simulation_metrics_json)

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
| `primitive_type` | `string` | Да | Тип примитива: `"box"`, `"cylinder"`, `"sphere"` |
| `params` | `object` | Да | Геометрические размеры в метрах (см. таблицу ниже) |

**Спецификация параметров примитивов**:
- `"box"`: `{"width": number, "height": number, "depth": number}`
- `"cylinder"`: `{"radius": number, "height": number, "segments"?: integer}`
- `"sphere"`: `{"radius": number, "sectors"?: integer, "stacks"?: integer}`

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

Вырезает сквозное или глухое цилиндрическое отверстие вдоль заданного нормального вектора.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор целевой детали |
| `position` | `[number, number, number]` | Да | Координаты точки входа отверстия `[x, y, z]` (м) |
| `normal` | `[number, number, number]` | Да | Единичный вектор направления оси отверстия `[nx, ny, nz]` |
| `radius` | `number` | Да | Радиус отверстия в метрах |
| `depth` | `number` | Да | Глубина выреза в метрах |

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

## Группа 3: Кинематика и физика механизмов

### 8. `create_joint`

Устанавливает кинематическое или силовое сопряжение между двумя деталями в физическом мире Rapier3D.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `joint_type` | `string` | Да | `"revolute"`, `"prismatic"`, `"spherical"`, `"fixed"` |
| `part_a` | `string` | Да | Идентификатор первой сопрягаемой детали |
| `part_b` | `string` | Да | Идентификатор второй сопрягаемой детали |
| `anchor_a` | `[number, number, number]` | Да | Точка крепления `[x, y, z]` в локальной СК детали A (м) |
| `anchor_b` | `[number, number, number]` | Да | Точка крепления `[x, y, z]` в локальной СК детали B (м) |
| `axis` | `[number, number, number]` | Нет | Направляющая ось `[x, y, z]` (обязательна для revolute/prismatic) |
| `limits` | `[number, number]` | Нет | Пределы движения `[min, max]` (в радианах или метрах) |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 8,
  "method": "tools/call",
  "params": {
    "name": "create_joint",
    "arguments": {
      "joint_type": "revolute",
      "part_a": "part-base",
      "part_b": "part-crank",
      "anchor_a": [0.0, 0.05, 0.0],
      "anchor_b": [-0.05, 0.0, 0.0],
      "axis": [0.0, 0.0, 1.0]
    }
  }
}
```

#### Пример ответа
```json
{
  "jsonrpc": "2.0",
  "id": 8,
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

### 9. `set_joint_motor`

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
  "id": 9,
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
  "id": 9,
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

### 10. `run_physics_simulation`

Запускает динамическую симуляцию механизма вперед во времени с записью траекторий, реакций в шарнирах и детекцией соударений.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `duration_seconds` | `number` | Да | Длительность физической симуляции в секундах |
| `step_size_ms` | `number` | Нет | Шаг численного интегрирования в мс (по умолчанию `16.67` мс = 60 Гц) |

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

### 11. `run_fea_analysis`

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
  "id": 11,
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
  "id": 11,
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

### 12. `get_fea_heatmap_image`

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
  "id": 12,
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
  "id": 12,
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

## Группа 5: Визуализация, экспорт и метрики

### 13. `capture_viewport`

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
  "id": 13,
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
  "id": 13,
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

### 14. `export_stl`

Экспортирует поверхностную сетку детали в файл стандартного формата STL для 3D-печати или станков ЧПУ.

#### Входные параметры
| Параметр | Тип | Обязательный | Описание |
|---|---|---|---|
| `part_id` | `string` | Да | Идентификатор детали |
| `filepath` | `string` | Да | Путь для сохранения файла на диске (например, `"./output/bracket.stl"`) |
| `binary` | `boolean` | Нет | `true` — компактный Binary STL (по умолчанию); `false` — ASCII STL |

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 14,
  "method": "tools/call",
  "params": {
    "name": "export_stl",
    "arguments": {
      "part_id": "part-a1b2c3d4e5f64789",
      "filepath": "./output/cantilever_beam.stl",
      "binary": true
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
        "text": "{\n  \"file_path\": \"./output/cantilever_beam.stl\",\n  \"triangles_count\": 12\n}"
      }
    ]
  }
}
```

---

### 15. `get_simulation_metrics_json`

Возвращает полный агрегированный инженерный отчет по всей сцене: геометрические метаданные всех деталей, результаты последних МКЭ-расчетов и телеметрию физической симуляции.

#### Входные параметры
*Нет параметров.*

#### Пример запроса
```json
{
  "jsonrpc": "2.0",
  "id": 15,
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
  "id": 15,
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
