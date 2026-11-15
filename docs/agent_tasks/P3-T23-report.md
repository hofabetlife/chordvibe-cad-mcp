# Отчёт по задаче P3-T23: Устранение зависания многопоточного прогона тестов `mcp-cad-server` на Windows и изоляция побочных записей `qa_wide_scan`

**Дата выполнения:** 15.11.2026  
**Статус:** ✅ DONE  

---

## 1. Диагностика и корневая причина

При воспроизведении многопоточного запуска `cargo test -p mcp-cad-server` на Windows без `-- --test-threads=1` и без ручного `MCP_CAD_DISABLE_GUI_SPAWN=1` были зафиксированы три взаимосвязанные проблемы:

1. **Конкурентная инициализация и рендеринг множества экземпляров WGPU (`OffscreenRenderer`):**
   - `EngineState::new()` (`crates/mcp-cad-server/src/state.rs`) и 4 unit-теста в `tools/metrics.rs` и `tools/project.rs` при каждом тесте создавали собственный независимый экземпляр `wgpu::Instance` / `Adapter` / `Device` / `Queue` через `OffscreenRenderer::new()`.
   - Одновременное создание, polling и уничтожение десятков WGPU-контекстов из параллельных потоков тест-раннера Rust вызывало взаимоблокировку/зависание драйвера на Windows (воспроизводилось как на `mcp-cad-server --lib`, зависавшем на ~37/61 тестах, так и в `mcp_contract_tests` при `--test-threads=4` даже с `MCP_CAD_DISABLE_GUI_SPAWN=1` — как на `WGPU_BACKEND=dx12`, так и на `WGPU_BACKEND=vulkan`).
2. **Побочный запуск GUI-процессов и гонки глобальной среды в `open_preview_window`:**
   - В `crates/mcp-cad-server/src/tools/render.rs` `open_preview_window` при отсутствии переменной окружения `MCP_CAD_DISABLE_GUI_SPAWN` по умолчанию пытался породить дочерний процесс `std::env::current_exe()` (`--gui-preview`) и перезаписывал общий файл `./output/_mcp_preview_live.stl`.
   - Тест `test_mcp_open_preview_window` в `mcp_contract_tests.rs` модифицировал процесс-глобальную переменную `MCP_CAD_DISABLE_GUI_SPAWN` через `std::env::set_var` / `std::env::remove_var` во время многопоточного прогона.
3. **Побочные записи `qa_wide_scan` в дерево репозитория:**
   - Тесты `crates/mcp-cad-server/tests/qa_wide_scan.rs` писали напрямую в отслеживаемые Git файлы `qa/results/*.json` и создавали неотслеживаемые файлы в `qa/artifacts/`.

---

## 2. Выполненные изменения

1. **[`crates/mcp-cad-viewer/src/offscreen.rs`](../../crates/mcp-cad-viewer/src/offscreen.rs)**:
   - Добавлен глобальный мьютекс `WGPU_INIT_LOCK: Mutex<()>` вокруг создания `wgpu::Instance`/`Adapter`/`Device` в `OffscreenRenderer::new()`.
   - Добавлено поле `render_lock: Mutex<()>` в `OffscreenRenderer`, сериализующее доступ к `device`/`queue`/буферам в начале `execute_render_pass()`.
   - Добавлен конструктор `pub fn shared() -> Result<Arc<Self>, ViewerError>` с кэшем `SHARED_RENDERER: Mutex<Option<Arc<OffscreenRenderer>>>`, переиспользующий один WGPU-контекст между параллельными потоками тестов и сессий без повторного создания множества `wgpu::Device`.
2. **[`crates/mcp-cad-server/src/state.rs`](../../crates/mcp-cad-server/src/state.rs)**:
   - `EngineState::new()` переключён с `Arc::new(OffscreenRenderer::new()?)` на `OffscreenRenderer::shared()?`.
3. **[`crates/mcp-cad-server/src/tools/metrics.rs`](../../crates/mcp-cad-server/src/tools/metrics.rs)** и **[`crates/mcp-cad-server/src/tools/project.rs`](../../crates/mcp-cad-server/src/tools/project.rs)**:
   - 4 тестовых вызова `OffscreenRenderer::new()` заменены на `OffscreenRenderer::shared()`.
4. **[`crates/mcp-cad-server/src/tools/render.rs`](../../crates/mcp-cad-server/src/tools/render.rs)**:
   - В `open_preview_window` добавлено автоопределение запуска внутри тестового бинарника Cargo (`cfg!(test)` или родительский каталог `current_exe()` равен `"deps"`).
   - В тестовом бинарнике, если `MCP_CAD_DISABLE_GUI_SPAWN` явно не задана, `disable_spawn` автоматически устанавливается в `true` (без влияния на боевой `mcp-cad-server.exe` и с сохранением ручного переопределения `0`/`false`/`1`/`true`), а путь превью-STL направляется в уникальный временный файл `std::env::temp_dir()/mcp_cad_preview_live_<uuid>.stl` вместо `./output/_mcp_preview_live.stl`.
5. **[`crates/mcp-cad-server/tests/mcp_contract_tests.rs`](../../crates/mcp-cad-server/tests/mcp_contract_tests.rs)**:
   - Из `test_mcp_open_preview_window` удалены гоночные вызовы `std::env::set_var` / `std::env::remove_var` для `MCP_CAD_DISABLE_GUI_SPAWN` (сохранена RAII-очистка `PreviewTestCleanup`).
6. **[`crates/mcp-cad-server/tests/qa_wide_scan.rs`](../../crates/mcp-cad-server/tests/qa_wide_scan.rs)**:
   - Добавлена RAII-обёртка `QaDirGuard` (`Drop`), по умолчанию создающая для каждого вызова уникальный каталог `std::env::temp_dir()/mcp_cad_qa_wide_scan_<pid>_<uuid>` (и удаляющая его после завершения теста) вместо записи в `qa/results/*.json` и `qa/artifacts/`.
   - Сохранена opt-in поддержка записи в репозиторий при установке `MCP_CAD_QA_WRITE_REPO=1`.
7. **[`.github/workflows/ci.yml`](../../.github/workflows/ci.yml)** и **[`README.md`](../../README.md)**:
   - В CI убран обходной костыль `-- --test-threads=1`: все 5 крейтов запускаются единым шагом `cargo test --workspace --no-fail-fast`.
   - В `README.md` актуализированы команды запуска тестов (удалены упоминания обязательного `-- --test-threads=1` и `MCP_CAD_DISABLE_GUI_SPAWN=1`).

---

## 3. Результаты верификации (после финальных правок кода)

Все проверки `mcp-cad-server` выполнены в стандартном многопоточном режиме Cargo **без `-- --test-threads=1`**, **без `MCP_CAD_DISABLE_GUI_SPAWN=1`** и **без ручного `WGPU_BACKEND`**:

| Проверка / Команда | Итог | Детали (наблюдённый вывод) |
|---|---|---|
| `cargo test -p mcp-cad-server --no-fail-fast` (прогон №1) | **135 passed, 0 failed** (`exit 0`) | `--lib`: `61 passed` (`35.86s`); `src\main.rs`: `0 passed`; `bench_suite.rs`: `1 passed` (`6.96s`); `gear_kinematic_test.rs`: `1 passed` (`0.74s`); `mcp_contract_tests.rs`: `59 passed` (`15.98s`); `qa_wide_scan.rs`: `13 passed` (`10.20s`); doc-tests: `0 passed` |
| `cargo test -p mcp-cad-server --no-fail-fast` (прогон №2) | **135 passed, 0 failed** (`exit 0`) | `--lib`: `61 passed` (`31.95s`); `src\main.rs`: `0 passed`; `bench_suite.rs`: `1 passed` (`6.41s`); `gear_kinematic_test.rs`: `1 passed` (`0.60s`); `mcp_contract_tests.rs`: `59 passed` (`14.66s`); `qa_wide_scan.rs`: `13 passed` (`9.41s`); doc-tests: `0 passed` |
| Проверка чистоты `qa/` и `output/` после `qa_wide_scan` | **CLEAN** | `Test-Path qa/artifacts` $\to$ `False`, `Test-Path output` $\to$ `False`, отслеживаемые `qa/results/*.json` в `git status` не затронуты |
| `cargo test -p mcp-cad-viewer --no-fail-fast` | **10 passed, 0 failed** (`exit 0`) | `--lib`: `0 passed`; `tests\viewer_tests.rs`: `10 passed; 0 failed` (`16.46s`) |
| `cargo clippy --workspace --all-targets -- -D warnings` | **PASS** (`exit 0`) | `0` ошибок, `0` предупреждений во всех 5 крейтах (`10.92s`) |
