# Отчёт о выполнении задачи P2-T37

## Задача
**P2-T37 [M]: Автономный запуск нативного 3D-окна WGPU в `open_preview_window` из `stdio`-сессии (фоновый спавн `--stl`) без отказа `"status": "headless"`**

---

## 1. Контекст и обнаруженные дефекты

1. **Отказ `{"status": "headless"}` в стандартных сессиях агента (`crates/mcp-cad-server/src/tools/render.rs:80-140`)**:
   - При подключении MCP-клиента (Google Antigravity, Claude Desktop, Cursor) сервер запускается с параметрами по умолчанию `["--mode", "stdio"]`.
   - В этом режиме `state.gui_proxy` равен `None`, и вызов инструмента `open_preview_window` возвращал отказ:
     `{"status": "headless", "message": "Server is running in headless mode without --gui. Interactive window not available, use capture_viewport instead."}`.
   - Из-за этого ИИ-агенты лезли читать исходный код `crates/mcp-cad-server/src/main.rs:122`, находили CLI-флаг `--stl <path>` и пытались через shell запускать бинарник, нарушая автономность MCP.

---

## 2. Внесённые изменения

1. **`crates/mcp-cad-server/src/tools/render.rs`**:
   - В обработчике `open_preview_window`: если `state.gui_proxy.is_none()`, движок собирает полигональную сетку (указанной детали `part_id` либо всей сцены) и экспортирует её в бинарный STL-файл (`./output/_mcp_preview_live.stl`, с автоматическим fallback во временную директорию ОС `std::env::temp_dir().join("mcp_cad_preview_live.stl")`).
   - Если переменная окружения `MCP_CAD_DISABLE_GUI_SPAWN=1` не установлена (для CI/headless контейнеров), сервер запускает независимый фоновый процесс:
     ```rust
     std::process::Command::new(std::env::current_exe()?)
         .arg("--stl")
         .arg(&preview_stl_path)
         .stdin(std::process::Stdio::null())
         .stdout(std::process::Stdio::null())
         .stderr(std::process::Stdio::null())
         .spawn()
     ```
     с полным перенаправлением `stdin/stdout/stderr` в `null`, что гарантирует чистоту потока JSON-RPC протокола `stdio`.
   - Возвращается структурированный JSON-ответ:
     `{"status": "opened", "mode": "spawned_gui_process", "pid": child.id(), "stl_path": ..., "part_id": ...}`.

2. **`src/cad/mcp_dispatcher.ts`**:
   - TypeScript-диспетчер синхронизирован с контрактом Rust: при вызове `open_preview_window` возвращает статус `"status": "opened"`, `"mode": "spawned_gui_process"`, `"stl_path": ...`, `"part_id": ...`.

3. **Документация**:
   - Обновлены `docs/ANTIGRAVITY_SETUP.md`, `docs/CLAUDE_DESKTOP_SETUP.md`, `SKILL.md` и `docs/AGENT_GUIDE.md`.

---

## 3. Верификация и тесты

- Добавлен unit-тест `verifies P2-T37..P2-T40: autonomous open_preview_window, validate_mesh strict_slivers & is_valid_solid, and piston revolve body fidelity` в `src/cad/cad_engine.test.ts`.
- Тест подтверждает: `open_preview_window` возвращает `status: 'opened'`, `mode: 'spawned_gui_process'`, `stl_path` содержит имя детали.
- `npx tsx --test --test-name-pattern="P2-T37" src/cad/cad_engine.test.ts`: **PASS**.
