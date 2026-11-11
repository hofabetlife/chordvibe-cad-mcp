# MCP-CAD Engineering & 3D Modeling Skill (`SKILL.md`)

This skill defines the canonical coordinate systems, parametric FeatureTree DAG semantics, specialized manufacturing tools, and GUI/Headless execution rules for AI agents interacting with the **MCP-CAD** engine.

> **CRITICAL AGENT DIRECTIVE**:  
> **DO NOT read Rust source code files under `crates/**/*.rs`**.  
> All geometric coordinate conventions, primitive bounds, DAG behaviors, and tool contracts are completely and authoritatively specified in this document and in [`docs/AGENT_GUIDE.md`](docs/AGENT_GUIDE.md).

---

## 1. Canonical Coordinate Systems & Primitive Bounding Ranges

- **Coordinate System**: Right-handed, $Z$-up ($+Z$ is the primary longitudinal axis of revolution, extrusion, and height).
- **Default Units**: Millimeters (`"mm"`). Linear dimensions, radii, coordinates $[x, y, z]$, and bounding boxes default to mm.
- **Origin**: All primitives are centered at $[0, 0, 0]$ in their local frame (unless offset by `"position": [x, y, z]`).

### Summary Table of 3D Primitives

| Примитив (`primitive_type`) | Координатный центр (Origin) | Диапазон по высоте ($Z$) | Поперечное сечение ($XY$) | Ключевые параметры | Особенности геометрии |
|---|---|---|---|---|---|
| **`box`** | $[0, 0, 0]$ | $Z \in [-d/2, +d/2]$ | $X \in [-w/2, +w/2]$, $Y \in [-h/2, +h/2]$ | `width`, `height`, `depth` | Прямоугольный параллелепипед. $w$ вдоль $X$, $h$ вдоль $Y$, $d$ вдоль $Z$. |
| **`cylinder`** | $[0, 0, 0]$ | $Z \in [-h/2, +h/2]$ | $x^2 + y^2 \le r^2$ | `radius`, `height` | Нижний торец `cap_bottom` при $Z = -h/2$, верхний `cap_top` при $Z = +h/2$. |
| **`cone`** | $[0, 0, 0]$ | $Z \in [-h/2, +h/2]$ | $r(z) = r_b + (r_t - r_b)\frac{z + h/2}{h}$ | `radius_bottom`, `radius_top`, `height` | **Основание `radius_bottom` при $Z = -h/2$**, **вершина `radius_top` при $Z = +h/2$**. Для острой вершины `radius_top: 0.0`. |
| **`sphere`** | $[0, 0, 0]$ | $Z \in [-r, +r]$ | $x^2 + y^2 + z^2 \le r^2$ | `radius` | Сфера с центром в $[0,0,0]$. |
| **`polygon_prism`** | $[0, 0, 0]$ | $Z \in [-h/2, +h/2]$ | Правильный $N$-угольник | `sides`, `outer_radius` (или `width_across_flats`), `height`, `chamfer_radius` | Правильная призма (шестигранник DIN 934 при `sides: 6`). Фаска `chamfer_radius` на верхнем и нижнем торцах. |
| **`spur_gear`** | $[0, 0, 0]$ | $Z \in [-b/2, +b/2]$ | Эвольвентный профиль (DIN 867) | `teeth`, `module`, `face_width`, `bore_diameter`, `hub_diameter`, `keyway_width` | Прямозубая шестерня с центральным отверстием, ступицей и шпоночным пазом DIN 6885 вдоль $Z$. |
| **`revolve`** | Осесимметрично $Z$ | Ограничено точками $z$ профиля | $x^2 + y^2 = r(z)^2$ | `profile: [[r, z], ...]`, `angle_deg` | Вращение 2D-профиля вокруг оси $Z$ на угол `angle_deg` $\in (0, 360^\circ]$. Радиус $r \ge 0$. |
| **`wheel_rim`** | $[0, 0, 0]$ | $Z \in [-w/2, +w/2]$ | Колёсный обод с $N$ спицами | `rim_diameter`, `rim_width`, `spokes_count`, `hub_diameter`, `pcd_diameter` | Литой автомобильный диск со спицами, ступичным отверстием и болтовой разболтовкой PCD. |

---

## 2. Семантика дерева построения DAG (`FeatureTree`)

1. **Базовое тело**: Первый вызов `add_feature_primitive` на детали `part_id` создаёт корневое 3D-тело.
2. **Авто-объединение (`auto-csg_union`)**: Все последующие вызовы `add_feature_primitive` на том же `part_id` **автоматически объединяются с текущим телом через булево объединение `csg_union`**! Они **не перезаписывают** деталь.
3. **Авто-вычитание (`auto-csg_difference`)**: Инструменты `cut_hole` и `cut_helical_groove` автоматически вычитают инструмент из текущей геометрии через `csg_difference`.
4. **Явные булевы операции**: Для взаимодействия двух отдельных деталей используйте `boolean_difference`, `boolean_union` или `boolean_intersection`.
5. **Транзакции**: Используйте `batch_cad_operations` для атомарного выполнения цепочки операций с автоматическим откатом при любой ошибке.

---

## 3. Таблица специализированных машиностроительных инструментов

| Инструмент | Назначение | Ключевые параметры | Стандарты / Особенности |
|---|---|---|---|
| **`cut_helical_groove`** | Нарезание винтовых канавок и резьбы | `helix_radius`, `groove_radius`, `pitch`, `height`, `starts`, `profile_type: "circular" \| "metric_60"` | Стружечные канавки свёрл (`circular`) и метрическая резьба 60° (`metric_60`). **Автоматически выравнивает `start_z` по нижней грани детали `bounding_box.min[2]`**. |
| **`cut_hole`** | Сверление цилиндрических отверстий | `position`, `normal`, `radius` (или `diameter`), `depth`, `through: true`, `keyway_width`, `keyway_depth` | Сквозное сверление (`through: true`), глухие отверстия и шпоночные пазы DIN 6885. Тегирует грань как `hole_wall:feat_N`. |
| **`validate_mesh`** | Диагностика топологии и качества сетки | `part_id`, `expected_genus`, `strict_slivers: boolean` | Проверяет водонепроницаемость (`is_watertight`), манифольдность (`is_manifold`), объем (`is_positive_volume`). По умолчанию (`strict_slivers: false`) микро-треугольники канавок выводятся в `warnings`, давая статус `PASS` (`is_valid_solid: true`, `fea_ready: true`). |
| **`run_fea_analysis`** | МКЭ расчёт напряжений по Мизесу | `part_id`, `fixed_faces`, `load_cases`, `mesh_density` | Линейно-упругий расчёт методом конечных элементов (тетраэдры TET4). Задаётся по именованным граням из `query_part_geometry`. |
| **`optimize_feature_tree`** | Стоимостная оптимизация DAG (CBO) | `part_id`, `apply: true` | Алгебраические правила CSG (R1-R5), отсечение невидимых операций AABB, схлопывание коаксиальных тел в 1-проходный Revolve. |

---

## 4. Режимы отображения: GUI vs Headless

- **`capture_viewport` (Для ИИ-агента)**:
  - Выполняет быстрый оффскрин-рендеринг в Base64 PNG.
  - Работает в любом окружении (включая безголовые Linux-контейнеры, CI и облачные среды без дисплея).
  - Поддерживает ракурсы (`camera_preset: "isometric" | "front" | "top"`), режимы отладки шейдеров (`shaded`, `wireframe`, `normals`, `boundary_edges`, `fea_stress`).
- **`open_preview_window` (Для пользователя)**:
  - Открывает интерактивное нативное 3D-окно WGPU на рабочем столе пользователя с управлением камерой мышь/WASD и инспекцией деталей.
  - **Автономная работа в `stdio`-режиме**: Если сервер запущен без флага `--gui` в стандартной `stdio` сессии (Google Antigravity, Claude Desktop, Cursor), вызов `open_preview_window` **не возвращает отказ `"status": "headless"`**. Вместо этого сервер автоматически экспортирует актуальную сетку во временный бинарный STL (`./output/_mcp_preview_live.stl`) и запускает фоновый процесс `mcp-cad-server --stl <path>`!
  - Для отключения спавна в автоматических тестах используется переменная окружения `MCP_CAD_DISABLE_GUI_SPAWN=1`.
