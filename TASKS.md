# MCP-CAD: Список задач по устранению несоответствий и развитию (Backlog)

> **Статус аудита соответствия документации и реальных возможностей:** проведен 31.10.2026.  
> **Общий вердикт:** Базовые инженерные подсистемы (Solid DAG, Rapier3D, FEA TET4, WGPU-рендер, экспорт/импорт STL, 28 MCP-инструментов в веб-студии) функционируют и верифицированы тестами. Выявлен ряд существенных архитектурных и документационных расхождений между Rust-бэкендом и TypeScript-клиентом.

---

## Сводная таблица задач

| ID | Приоритет | Категория | Задача | Статус | Компонент |
|---|---|---|---|---|---|
| **TASK-001** | **P1 (High)** | Backend / Core | Портировать `query_optimizer` (CBO / `optimize_feature_tree`) в Rust (`crates/mcp-cad-core`) | ✅ **DONE** | `mcp-cad-core`, `mcp-cad-server` |
| **TASK-002** | **P1 (High)** | Backend / Server | Добавить `validate_mesh` (Tool #27) и `optimize_feature_tree` (Tool #28) в схему `list_tools()` Rust-сервера | ✅ **DONE** | `mcp-cad-server` |
| **TASK-003** | **P2 (Medium)** | Docs Sync | Синхронизировать счетчики инструментов (26 vs 28) во всех документах (`README.md`, `ARCHITECTURE.md`, `PROJECT_CONTEXT.md`) | ✅ **DONE** | `docs/`, `README.md`, `PROJECT_CONTEXT.md` |
| **TASK-004** | **P2 (Medium)** | Docs / PRD | Актуализировать `PRD.md`: удалить устаревшие упоминания `egui`, `apply_boolean`, уточнить статус `ExtrudeSketch` и `Fillet` | ✅ **DONE** | `PRD.md` |
| **TASK-005** | **P1 (High)** | Frontend / Types | Устранить ошибки компиляции TypeScript (`tsc --noEmit`) в `mcp-cad-studio` | ✅ **DONE** | `src/App.tsx`, `src/types/cad.ts`, `src/cad/native_bridge.ts` |
| **TASK-006** | **P3 (Low)** | DevOps / CI | Обеспечить Docker / Linux headless окружение для сборки и запуска `mcp-cad-server` под Linux | ✅ **DONE** | `Dockerfile`, `run_server.sh`, `README.md` |
| **TASK-007** | **P2 (Medium)** | Geometry | Реализовать универсальный `ExtrudeSketch` из 2D контура в Solid DAG | 📋 BACKLOG | `mcp-cad-core` |
| **TASK-008** | **P3 (Low)** | Geometry | Реализовать аналитическое скругление ребер (`Fillet`) для базовых примитивов | 📋 BACKLOG | `mcp-cad-core` |

---

## Детальное описание задач

### TASK-001 [P1]: Портировать Cost-Based CAD Query Optimizer в Rust-ядро
- **Проблема**: В `PROJECT_CONTEXT.md` (п. 4.1), `README.md` (п. 190) и `docs/MCP_API_REFERENCE.md` (инструмент №28) заявлено наличие Cost-Based Optimizer в крейте `crates/mcp-cad-core` (`query_optimizer`) и инструмента `optimize_feature_tree` в сервере. Фактически оптимизатор реализован исключительно в TypeScript (`src/cad/query_optimizer.ts`, `src/cad/mcp_dispatcher.ts`). В коде Rust (`crates/`) модуль отсутствует. При вызове инструмента через нативный `mcp-cad-server.exe` возвращается ошибка `ToolNotFound`.
- **Что необходимо сделать**:
  1. Создать модуль `crates/mcp-cad-core/src/query_optimizer.rs` с реализацией правил перезаписи графа DAG:
     - `R1_TRANSFORM_FUSION` (свертка матриц трансформаций SE(3));
     - `R2_AABB_DEAD_OP_CULLING` (отсечение операций вне ограничивающего объема);
     - `R3_DEMORGAN_CSG_BATCH` (пакетная группировка компланарных вырезов);
     - `R4_COAXIAL_REVOLVE_REDUCTION` (замена соосного сверления на 2D-вращение профиля);
     - `R5_ANALYTICAL_PRISM_THREAD_FUSION` (слияние призмы и резьбы).
  2. Добавить метод `part.optimize_dag(apply: bool)` в `mcp-cad-core`.
  3. Реализовать обработчик `cad::optimize_feature_tree` в `crates/mcp-cad-server/src/tools/cad.rs`.
  4. Подключить маршрут в `dispatch_tool` в `crates/mcp-cad-server/src/tools/mod.rs`.
- **Критерии приемки**:
  - Вызов `{"method": "tools/call", "params": {"name": "optimize_feature_tree", "arguments": {"part_id": "...", "apply": true}}}` возвращает расчетную экономию стоимости `cost_reduction_pct` и текстовый `explain_analyze_text`.
  - Юнит-тесты на проверку сохранения объема ($\Delta V < 0.5\%$) и топологических инвариантов проходят на 100%.
- **Статус**: ✅ **ИСПРАВЛЕНО И ВЕРИФИЦИРОВАНО** (`crates/mcp-cad-core/src/query_optimizer.rs`, `crates/mcp-cad-core/src/part.rs`, `crates/mcp-cad-server/src/tools/cad.rs`).

---

### TASK-002 [P1]: Экспортировать инструменты №27 (`validate_mesh`) и №28 (`optimize_feature_tree`) в схеме `list_tools()`
- **Проблема**: Функция `list_tools()` в `crates/mcp-cad-server/src/tools/mod.rs` возвращает ровно 26 инструментов. Инструмент `validate_mesh` существует в обработчике `dispatch_tool`, но отсутствует в списке `list_tools()`. Инструмент `optimize_feature_tree` отсутствует и в списке, и в диспетчере. Из-за этого LLM-клиенты (Claude Desktop, Cursor) при опросе `tools/list` не видят эти инструменты.
- **Что необходимо сделать**:
  1. Добавить определение схемы входных параметров для `validate_mesh` в `list_tools()`.
  2. Добавить определение схемы для `optimize_feature_tree` в `list_tools()`.
  3. Обновить интеграционный тест `crates/mcp-cad-server/tests/mcp_contract_tests.rs`: изменить `assert_eq!(tools.len(), 26)` на `assert_eq!(tools.len(), 28)`.
- **Критерии приемки**:
  - JSON-RPC метод `tools/list` возвращает массив из 28 инструментов с полными валидными JSON-схемами.
  - Тест `test_mcp_tools_list` успешно проходит проверку на 28 инструментов.
- **Статус**: ✅ **ИСПРАВЛЕНО И ВЕРИФИЦИРОВАНО** (`crates/mcp-cad-server/src/tools/mod.rs`, `crates/mcp-cad-server/tests/mcp_contract_tests.rs`).

---

### TASK-003 [P2]: Устранение числовых противоречий в документации (26 vs 28 инструментов)
- **Проблема**: В разных разделах документации фигурируют противоречивые цифры:
  - `README.md`: строка 16 — «26 tools», строка 158 — «доступность всех 26 инструментов», строка 197 — «28 Tools», строка 292 — «всех 26 инструментов».
  - `docs/ARCHITECTURE.md`: строка 33 — «Полный справочник 26 инструментов MCP».
  - `PROJECT_CONTEXT.md`: строка 231 — «всех 26 инструментов MCP», строка 249 — «28 инструментов».
- **Что необходимо сделать**:
  - Унифицировать во всех документах указание общего числа инженерных инструментов: **28 инструментов**.
  - Обновить ссылки в оглавлении `README.md`.
- **Критерии приемки**:
  - Во всей документации используется единое число инструментов (28 Tools).
- **Статус**: ✅ **ИСПРАВЛЕНО И ВЕРИФИЦИРОВАНО** (`README.md`, `docs/ARCHITECTURE.md`, `PROJECT_CONTEXT.md`, `qa/COVERAGE.md`).

---

### TASK-004 [P2]: Актуализация архитектурных формулировок в `PRD.md`
- **Проблема**:
  - `PRD.md` (раздел 3, 4.5) упоминает `egui` (`WGPU --> EGUI`), однако в `crates/mcp-cad-viewer` зависимость `egui` не используется (WGPU рендерер нативный, а HUD реализован в веб-студии на React/Tailwind).
  - В разделе 4.1 и 5.2 упоминается устаревший инструмент `apply_boolean`, который в актуальной реализации заменен на `boolean_difference`, `boolean_union`, `boolean_intersection`.
  - Упоминаются `ExtrudeSketch` и универсальный `Fillet`, которые не реализованы в ядре.
- **Что необходимо сделать**:
  - Внести правки в `PRD.md`, уточнив, что графический HUD реализован в веб-интерфейсе `MCP-CAD Studio`, а нативный вьюпорт работает на чистом `winit + wgpu`.
  - Заменить устаревший `apply_boolean` на актуальную спецификацию CSG-триады.
  - Пометить `ExtrudeSketch` и произвольный `Fillet` как Roadmap / Future scope.
- **Критерии приемки**:
  - `PRD.md` полностью согласуется с архитектурой `PROJECT_CONTEXT.md` и кодом.
- **Статус**: ✅ **ИСПРАВЛЕНО И ВЕРИФИЦИРОВАНО** (`PRD.md`).

---

### TASK-005 [P1]: Устранение ошибок компиляции TypeScript в `mcp-cad-studio`
- **Проблема**: `npm run lint` (`tsc --noEmit`) падал с 5 ошибками статической типизации:
  - `NativeBridgeStatus` не экспортировался из `./cad/native_bridge`;
  - Использовалось поле `titleRu` вместо `title`;
  - Несовпадение типа `QualityPreset` (`'fine'` отсутствовал в допустимых значениях);
  - Отсутствовали типы `@types/node` для тестов.
- **Статус**: ✅ **ИСПРАВЛЕНО И ВЕРИФИЦИРОВАНО**
  - Экспортирован алиас `NativeBridgeStatus`;
  - Добавлено опциональное свойство `titleRu?: string` в `GoldenArtifactMapping` и безопасный фоллбек `mapping.titleRu || mapping.title`;
  - Расширен тип `QualityPreset` до `'draft' | 'standard' | 'high' | 'fine'`;
  - Установлен dev-пакет `@types/node`.
  - `npm run lint` и `compile_applet` завершаются с кодом 0 (0 ошибок).
  - Набор из 16 тестов геометрии и оптимизатора (`src/cad/cad_engine.test.ts`) проходит на 100%.

---

### TASK-006 [P3]: Кроссплатформенная поддержка и запуск под Linux/MacOS
- **Проблема**: Проект позиционируется преимущественно как «Rust под Windows» с готовыми скриптами `.bat`/`.ps1` и путями `target\release\mcp-cad-server.exe`. Пользователи других ОС не могут запустить нативный сервер «из коробки» без предварительной настройки окружения.
- **Что необходимо сделать**:
  - Добавить `Dockerfile` и скрипт `run_server.sh` для запуска headless-сервера на Linux с эмуляцией GPU (llvmpipe / lavapipe).
  - Добавить секцию в `README.md` с инструкцией запуска под Linux/macOS.
- **Критерии приемки**:
  - Наличие рабочего `Dockerfile` и документации по запуску на Linux.
- **Статус**: ✅ **ИСПРАВЛЕНО И ВЕРИФИЦИРОВАНО** (`Dockerfile`, `run_server.sh`, `README.md`).

---

### TASK-007 [P2] & TASK-008 [P3]: Расширение параметрического геометрического ядра
- **TASK-007**: Реализовать операцию `ExtrudeSketch(profile_2d, depth)` в `crates/mcp-cad-core/src/primitives.rs` и DAG для выдавливания произвольных замкнутых 2D-полигонов.
- **TASK-008**: Реализовать алгоритм скругления ребер (`Fillet`) на триангулированных телах без нарушения условия герметичности (watertightness).
