# Отчёт по задаче P3-T20: Синхронизация контрактного теста `test_mcp_open_preview_window` с поведением `P2-T37`

**Дата выполнения:** 15.11.2026  
**Статус:** ✅ DONE  

---

## 1. Цель и анализ проблемы

В контрактном тесте [`test_mcp_open_preview_window`](../../crates/mcp-cad-server/tests/mcp_contract_tests.rs#L695-L868) вызывался инструмент `open_preview_window` на пустой сцене (`EngineState::new()` без деталей) и проверялось `val["status"] == "headless"`.

После реализации `P2-T37` в [`open_preview_window`](../../crates/mcp-cad-server/src/tools/render.rs#L72-L223):
- На пустой сцене (либо для детали без геометрии) инструмент возвращает `{"status": "empty", "message": "No geometry in scene to preview. Create parts or add features first."}`.
- При наличии непустой геометрии и установленной переменной окружения `MCP_CAD_DISABLE_GUI_SPAWN=1` инструмент экспортирует бинарный STL во временный путь (`./output/_mcp_preview_live.stl` или системный `temp_dir`) и возвращает `{"status": "headless", "stl_path": ..., "message": ...}`.

---

## 2. Выполненные изменения

1. **[`crates/mcp-cad-server/tests/mcp_contract_tests.rs`](../../crates/mcp-cad-server/tests/mcp_contract_tests.rs#L694-L868)**:
   - Обновлён тест `test_mcp_open_preview_window`:
     - Добавлен RAII-гард `PreviewTestCleanup`, который гарантирует очистку созданных временных STL-файлов (`stl_path`), удаление созданного каталога `./output` (только если он отсутствовал до теста и пуст) и восстановление исходного значения `MCP_CAD_DISABLE_GUI_SPAWN` после завершения теста (включая панику).
     - Проверено, что `open_preview_window({})` на пустой сцене возвращает `"status": "empty"` и непустой `message`.
     - Проверено, что `open_preview_window({"part_id": part_id})` на созданной детали без фич также возвращает `"status": "empty"`.
     - После добавления примитива `box` (`20×10×10 мм`) проверены оба контролируемых headless-сценария при `MCP_CAD_DISABLE_GUI_SPAWN=1` — с явным `{"part_id": part_id}` и по всей сцене `{}`: возвращается `"status": "headless"`, ссылка на `capture_viewport` в `message`, а файл по `stl_path` существует на диске и содержит бинарные треугольники (`len() > 84` байт).

---

## 3. Результаты верификации (после финальной правки кода)

| Проверка / Команда | Итог | Детали |
|---|---|---|
| `cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_open_preview_window -- --exact` | **1 passed, 0 failed** (`0.59s`) | Проверен автономный запуск теста без предустановленной `MCP_CAD_DISABLE_GUI_SPAWN` и без остаточных файлов в `./output` |
| `$env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --test mcp_contract_tests --no-fail-fast -- --test-threads=1` | **59 passed, 0 failed** (`61.45s`) | Все 59 контрактных тестов `mcp-cad-server` пройдены (`0 failed`) |
| `$env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --lib --no-fail-fast -- --test-threads=1` | **61 passed, 0 failed** (`64.19s`) | Все 61 unit-тест библиотеки `mcp-cad-server` пройдены (`0 failed`) |
| `cargo clippy --workspace --all-targets -- -D warnings` | **PASS** (`exit 0`, `5.07s`) | `0` ошибок, `0` предупреждений по всему workspace |
