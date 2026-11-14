# Отчёт о выполнении задачи P2-T40

## Задача
**P2-T40 [S]: Сохранение `scope: "scene"` / `part_id: null` в `check_requirements` при 1 детали в сцене и вызов `attach_auto_check_if_enabled` в `delete_part`**

---

## 1. Контекст и обнаруженные дефекты

1. **Искажение области действия требований уровня сцены при 1 детали**:
   - В `crates/mcp-cad-server/src/tools/requirements.rs` (`evaluate_single_requirement`) при проверке требований со `scope: "scene"` (`effective_part_id.is_none()`), если в сцене находилась ровно одна деталь (`state.parts.len() == 1`), вызов `resolve_target_part(None, state)` возвращал эту единственную деталь.
   - В результате требования уровня сцены (`bbox_max`, `bbox_min`, `mass_max`, `mesh_valid`) возвращали `"part_id": "<single_part_id>"` и `"scope": "<single_part_id>"` вместо `"part_id": null` и `"scope": "scene"`, что ломало контракт отчётов требований сцены в клиентах.

2. **Отсутствие автоматической валидации при удалении детали**:
   - В `crates/mcp-cad-server/src/tools/cad.rs:391-420` (`delete_part`) при удалении детали из сцены не вызывался `super::requirements::attach_auto_check_if_enabled(state, None, &mut resp)`, из-за чего статус требований сцены не обновлялся автоматически после удаления.

---

## 2. Внесённые изменения

1. **`crates/mcp-cad-server/src/tools/requirements.rs`**:
   - Для проверок `BboxMax` и `BboxMin` добавлена предварительная проверка `if effective_part_id.is_none()`, которая вычисляет общий bounding box всей сцены по всем непустым деталям до вызова `resolve_target_part`.
   - В ветках агрегации `MassMax` и `MeshValid` условие `state.parts.len() > 1` заменено на `!state.parts.is_empty()`, благодаря чему требования сцены корректно вычисляются как общесценические даже при 1 детали и сохраняют `"part_id": null, "scope": "scene"`.

2. **`crates/mcp-cad-server/src/tools/cad.rs`**:
   - В обработчик `delete_part` добавлен вызов `super::requirements::attach_auto_check_if_enabled(state, None, &mut resp)` перед формированием финального ответа.

---

## 3. Верификация

- Проверено сохранение контракта `part_id: null` и `scope: "scene"` при единичной детали в сцене.
- Проверен вызов авто-чека в `delete_part`.
- Скомпилировано и проверено линтером.
