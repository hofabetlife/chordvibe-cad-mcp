# Отчёт о выполнении задачи P2-T39

## Задача
**P2-T39 [M]: Устранение геометрической неопределённости в `initialize.instructions`, `AGENT_GUIDE.md`, `SKILL.md` и `tools/mod.rs` (таблица привязок примитивов, авто-`csg_union` в DAG, спец-инструменты и `GUI vs Headless`)**

---

## 1. Контекст и проблемы

При практическом тестировании с ИИ-агентами в среде Google Antigravity выявлено:
1. Агент тратил вызовы на чтение исходного кода (`primitives.rs`, `dag.rs`), пытаясь выяснить:
   - Где находится центр координат примитивов ($Z=0$, $Z \in [-h/2, +h/2]$);
   - Куда ориентировано основание (`radius_bottom` при $Z = -h/2$) и вершина (`radius_top` при $Z = +h/2$) конуса;
   - Семантику последовательных вызовов `add_feature_primitive` на одной детали (перезапись vs автоматический `csg_union`).
2. Агент не знал о наличии готовых высокоточных машиностроительных генераторов (`cut_helical_groove`, `cut_hole` со шпоночным пазом, `spur_gear`, `polygon_prism`, `wheel_rim`).
3. Агенту не хватало чёткого разграничения `capture_viewport` (Base64 PNG для агента) vs `open_preview_window` (нативное 3D-окно для пользователя).

---

## 2. Внесённые изменения

1. **Создание `/SKILL.md`**:
   - Создан полный файл навыка по стандарту Antigravity / Claude Desktop.
   - Включает исчерпывающую таблицу систем координат всех 8 примитивов (`box`, `cylinder`, `cone`, `sphere`, `torus`, `polygon_prism`, `spur_gear`, `wheel_rim`).
   - Содержит описание семантики FeatureTree DAG: первый вызов `add_feature_primitive` создаёт базу, последующие автоматически объединяются через `csg_union`, а `cut_hole` / `cut_helical_groove` автоматически вычитаются через `csg_difference`.
   - Приведена таблица специализированных инструментов со всеми параметрами.
   - Описан раздел `GUI vs Headless` и директива не читать исходный код Rust.

2. **`docs/AGENT_GUIDE.md` и `docs/ANTIGRAVITY_SETUP.md`**:
   - Включена директива «DO NOT read Rust source files in `crates/` — all coordinate conventions and contracts are fully specified in this guide and SKILL.md».
   - Добавлены таблицы координат, DAG семантики и правила валидации сеток с `strict_slivers: false`.

3. **`crates/mcp-cad-server/src/protocol.rs` (`AGENT_INSTRUCTIONS`)**:
   - В ответ на `initialize` клиенту передаётся компактная 5-пунктовая шпаргалка с точными формулами привязки примитивов, правилом auto-`csg_union`, списком специализированных инструментов и руководством по GUI.

4. **`crates/mcp-cad-server/src/tools/mod.rs`**:
   - Дополнены описания инструментов `add_feature_primitive`, `cut_hole`, `cut_helical_groove`, `open_preview_window` и `validate_mesh` точными формулами диапазонов координат.

---

## 3. Верификация

- Проверена точность документации и соответствие реальной генерации геометрии в `crates/mcp-cad-core/src/primitives.rs` и `src/cad/primitives.ts`.
- `tsc --noEmit` и unit-тесты проходят без ошибок.
