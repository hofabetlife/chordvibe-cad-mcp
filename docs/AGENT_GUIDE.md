# Руководство для ИИ-агента по работе с MCP-CAD (`AGENT_GUIDE`)

Настоящее руководство определяет правила надёжного параметрического 3D-моделирования, топологической верификации и инженерных расчётов (МКЭ и динамики механизмов) через протокол MCP. Полный справочник схем — в [`docs/MCP_API_REFERENCE.md`](MCP_API_REFERENCE.md) и [`docs/UNITS.md`](UNITS.md).

---

> **ВАЖНЕЙШАЯ ДИРЕКТИВА АГЕНТУ**:  
> **НЕ ЧИТАЙТЕ файлы исходного кода в `crates/**/*.rs`!** Все системы координат, привязки примитивов, семантика дерева DAG и правила инструментов исчерпывающе зафиксированы в настоящем руководстве и в [`SKILL.md`](../SKILL.md).

---

## 1. Системы координат примитивов и семантика дерева DAG

### 1.1. Координатные привязки примитивов
- Все примитивы ориентированы в правосторонней декартовой системе с **вертикальной осью $Z$** ($Z$-up).
- По умолчанию начало координат $[0, 0, 0]$ находится в центре симметрии примитива:
  - **`box`**: центрирован по всем трём осям: $X \in [-w/2, +w/2]$, $Y \in [-h/2, +h/2]$, $Z \in [-d/2, +d/2]$.
  - **`cylinder`**: центрирован по высоте: $Z \in [-h/2, +h/2]$, $x^2 + y^2 \le r^2$. Нижний торец `cap_bottom` при $Z = -h/2$, верхний `cap_top` при $Z = +h/2$.
  - **`cone`**: центрирован по высоте: $Z \in [-h/2, +h/2]$. **Основание `radius_bottom` при $Z = -h/2$**, **вершина `radius_top` при $Z = +h/2$**. Для острого конуса передавайте `radius_top: 0.0`.
  - **`sphere`**: сфера с центром в $[0, 0, 0]$ и радиусом `radius`.
  - **`polygon_prism`**: правильная $N$-гранная колонка или гайка DIN 934 ($N=6$): $Z \in [-h/2, +h/2]$, с опциональной фаской `chamfer_radius` на верхнем и нижнем торцах.
  - **`spur_gear`**: эвольвентная шестерня DIN 867: центрирована по ширине венца $Z \in [-b/2, +b/2]$, со ступицей `hub_diameter`/`hub_length` и шпоночным пазом DIN 6885.
  - **`revolve`**: тело вращения 2D-профиля `[[r, z], ...]` вокруг оси $Z$ ($r \ge 0$).
  - **`wheel_rim`**: автомобильный колесный диск со ступичным отверстием, болтовой сеткой PCD и спицами.

### 1.2. Семантика FeatureTree DAG
1. Первый вызов `add_feature_primitive` создаёт корневое 3D-тело детали.
2. Все последующие вызовы `add_feature_primitive` на том же `part_id` **автоматически объединяются с телом детали через `csg_union`** (они НЕ перезаписывают деталь!).
3. Инструменты `cut_hole` и `cut_helical_groove` **автоматически вычитаются через `csg_difference`**.
4. `cut_helical_groove` автоматически выравнивает начало винтовой канавки `start_z` по нижней грани детали (`bounding_box.min[2]`).

### 1.3. GUI vs Headless (Визуализация)
- **`capture_viewport`**: используется агентом для оффскрин-рендеринга в Base64 PNG. Работает в любом безголовом Linux-окружении и CI.
- **`open_preview_window`**: используется для интерактивного показа пользователю. При вызове из стандартной `stdio`-сессии без флага `--gui` сервер **автоматически экспортирует живой STL (`output/_mcp_preview_live.stl`) и запускает фоновый процесс нативного окна WGPU**, возвращая `"status": "opened", "mode": "spawned_gui_process"`.

---

## 2. Принципы работы

1. **Единицы измерения (`[DECISION-UNITS-1]`)**:
   - По умолчанию все линейные размеры, координаты и габариты на границе MCP передаются и возвращаются в **миллиметрах (`length_unit = "mm"`)**. Переключение в метры (`"m"`) выполняется на уровне сессии через `set_units` или локально для одного вызова параметром `"length_unit": "m"`.
   - Физические величины не зависят от `length_unit`: сила — `N`, момент — `N·m`, напряжение и давление — `MPa`, модуль Юнга — `GPa`, перемещение МКЭ — `mm`, масса — `kg`, плотность — `kg/m³`, углы в CAD и вращательных шарнирах — `deg`, угловая скорость — `rad/s`, линейная скорость — `m/s`, время — `s` (`duration_seconds`) / `ms` (`step_size_ms`).
   - Внутри файлов `.stl` на диске (`export_stl`, `import_stl`) координаты вершин всегда хранятся в метрах СИ (`m`).
2. **Ознакомление со сценой (`get_scene_summary`)**:
   - При первом обращении в новом контексте или после `load_project` вызывайте `get_scene_summary`. Инструмент возвращает компактную текстовую и JSON-сводку: единицы проекта, все детали (`id`, имя, габарит, объём, масса, валидность сетки, последний расчёт FEA: `FoS` / вердикт / устарел ли `STALE`), шарниры и статус инженерных требований.
3. **Объявление требований (`set_requirements` / `check_requirements`)**:
   - До или сразу после создания детали объявите инженерные ограничения (`bbox_max`, `bbox_min`, `mass_max`, `mesh_valid`, `min_safety_factor`) через `set_requirements` с флагом `"auto_check": true`.
   - При `auto_check: true` каждый мутирующий вызов (`add_feature_primitive`, `cut_hole`, `modify_feature`, `boolean_*`, `transform_part`, `batch_cad_operations`) автоматически возвращает `requirements_status: {passed, failed, stale}` и список нарушений `failed_requirements`.
4. **Транзакции (`batch_cad_operations`)**:
   - Объединяйте серию шагов построения детали в `batch_cad_operations`. При ошибке любого шага батч прерывается и атомарно откатывает состояние сцены к исходному.
5. **Обязательный `validate_mesh` после булевых операций и вырезов**:
   - После `boolean_difference`, `boolean_union`, `boolean_intersection`, `cut_hole` и `cut_helical_groove` обязательно вызывайте `validate_mesh` (или контролируйте требование `mesh_valid`). В обычном режиме `strict_slivers: false` микро-треугольники сложных винтовых канавок выводятся в `warnings`, сохраняя статус `PASS` (`is_valid_solid: true`, `fea_ready: true`).
6. **Расчёт прочности (FEA) только после валидной сетки и по именованным граням**:
   - Запускайте `run_fea_analysis` **только после `validate_mesh == "PASS"`**.
   - Задавайте закрепления (`fixed`) и нагрузки (`loads`) по именованным граням из `query_part_geometry` (например, `"cap_bottom"`, `"cap_top"`, `"left"`, `"right"`, `"hole_wall:*"`), а не по сырым координатам.
   - Любая модификация геометрии детали увеличивает счётчик `part.revision` и переводит прежний отчёт FEA в статус `STALE`. После правок геометрии обязательно перезапустите `run_fea_analysis`.
7. **Как читать ошибки (`ErrorEnvelope`)**:
   - Если в ответе `result.isError == true`, строка `result.content[0].text` содержит JSON:
     `{"error": {"code": "...", "message": "...", "param": "...", "hint": "...", "retryable": false, "details": {...}}}`.
   - Используйте `error.param` для локализации неверного аргумента, следуйте инструкции в `error.hint` и берите допустимые идентификаторы из `error.details` (`available_parts`, `available_faces`, `supported_primitives`).

---

## 2. Типовой цикл: «Построить → Проверить → Посчитать → Итерация»

1. **Контекст и ТЗ**: `get_scene_summary` → `create_part` → `set_requirements` (`auto_check: true`).
2. **Построить**: `add_feature_primitive` / `cut_hole` / `boolean_*` (или пакетно через `batch_cad_operations`).
3. **Проверить геометрию**: `validate_mesh` (убедиться в `status == "PASS"`) → `query_part_geometry` (получить список именованных граней `faces` и проверить `warnings`).
4. **Посчитать**: `run_fea_analysis` (граничные условия `fixed` и `loads` по именованным граням) и/или `create_joint` → `set_joint_motor` → `run_physics_simulation`.
5. **Итерация**: вызвать `check_requirements`. Если есть `FAIL` (например, `min_safety_factor` или `mass_max`) либо `STALE`: изменить размеры через `modify_feature` → проверить `validate_mesh` → перезапустить `run_fea_analysis` → убедиться, что все требования в статусе `PASS` → сохранить проект через `save_project`.

---

## 3. Короткие примеры вызовов (`tools/call`)

### Пример 1: Пакетное создание кронштейна с отверстием и автопроверкой требований (в `mm`)
```json
{
  "name": "batch_cad_operations",
  "arguments": {
    "operations": [
      { "tool": "create_part", "params": { "part_id": "bracket", "name": "Support Bracket", "material": "aluminum_6061_t6" } },
      { "tool": "set_requirements", "params": {
          "part_id": "bracket",
          "auto_check": true,
          "requirements": [
            { "id": "req_box", "type": "bbox_max", "dims": [120.0, 40.0, 30.0] },
            { "id": "req_mass", "type": "mass_max", "kg": 0.35 },
            { "id": "req_mesh", "type": "mesh_valid", "expected_genus": 1 },
            { "id": "req_fos", "type": "min_safety_factor", "value": 2.0, "fea_case": "tip_load" }
          ]
      }},
      { "tool": "add_feature_primitive", "params": { "part_id": "bracket", "primitive_type": "box", "params": { "width": 100.0, "height": 20.0, "depth": 20.0 } } },
      { "tool": "cut_hole", "params": { "part_id": "bracket", "position": [0.0, 0.0, -15.0], "normal": [0.0, 0.0, 1.0], "radius": 5.0, "depth": 30.0 } }
    ]
  }
}
```

### Пример 2: Проверка сетки и опрос именованных граней перед МКЭ
```json
{ "name": "validate_mesh", "arguments": { "part_id": "bracket", "expected_genus": 1 } }
```
```json
{ "name": "query_part_geometry", "arguments": { "part_id": "bracket", "where": { "kind": "plane" } } }
```

### Пример 3: Расчёт МКЭ по именованным граням и итоговая сводка сцены
```json
{
  "name": "run_fea_analysis",
  "arguments": {
    "part_id": "bracket",
    "fea_case": "tip_load",
    "mesh_density": "medium",
    "fixed": [{ "face": "left" }],
    "loads": [{ "face": "right", "force": [0.0, -500.0, 0.0] }]
  }
}
```
```json
{ "name": "get_scene_summary", "arguments": {} }
```
