# Отчёт по задаче `P5-T9`: Реализация реального CPU-fallback в `OffscreenRenderer::new()` при отсутствии GPU и Vulkan Lavapipe

- **Дата выполнения**: 21.11.2026
- **Статус**: ✅ DONE

---

## 1. Реализованные изменения

### 1.1. Pure-Rust программный CPU-растеризатор (`crates/mcp-cad-viewer/src/software.rs`)
Реализован самодостаточный программный растеризатор треугольников и отрезков без системных GPU/Vulkan зависимостей в соответствии с `ADR-003`:
- **Буфер цвета и Z-буфер (`Framebuffer`)**: фон `[38, 43, 51, 255]` (`[0.15, 0.17, 0.20, 1.0]`), глубина в диапазоне WGPU `[0.0, 1.0]`, защита от нулевых и чрезмерных размеров (`MAX_SOFTWARE_DIMENSION = 8192`).
- **Геометрический конвейер**: матричное преобразование `CameraUniform::view_proj`, клиппинг полигонов по ближней плоскости (`z_clip >= 0.0`, `w_clip >= 1e-5`), перспективно-корректная барицентрическая интерполяция мировых координат, нормалей и скаляров МКЭ, определение ориентации грани (`is_front`) по правилу `FrontFace::Ccw`.
- **Паритет шейдеров WGPU**:
  - `CAD_SHADER_WGSL`: двухстороннее затенение Блинна–Фонга с ключевым и заполняющим светом, а также все отладочные режимы `DebugRenderMode` (`None`, `Normals`, `Backfaces` с подсветкой обратных граней маджентой, `FlatShading` по геометрической нормали треугольника).
  - `WIREFRAME_SHADER_WGSL`: 3D/2D клиппинг отрезков (Liang–Barsky) и растеризация каркаса (`Wireframe`) и открытых/неманифольдных рёбер (`BoundaryEdges`) с отрицательным смещением глубины и альфа-смешиванием.
  - `FEA_HEATMAP_SHADER_WGSL`: 5-ступенчатая тепловая карта `Jet` (`blue → cyan → green → yellow → red`) с модуляцией диффузным освещением.
- **Кодирование**: вывод в стандартный формат PNG (`image::ImageFormat::Png`) и Base64.

### 1.2. Архитектура бэкендов `OffscreenRenderer` (`crates/mcp-cad-viewer/src/offscreen.rs`, `lib.rs`)
- Выделена структура `WgpuOffscreenState` и внутреннее перечисление `OffscreenBackend::Wgpu(Box<WgpuOffscreenState>) | OffscreenBackend::SoftwareCpu`.
- В `OffscreenRenderer::new()` (и `OffscreenRenderer::shared()`) при отсутствии WGPU-адаптера (`NoSuitableAdapter`) или отказе `adapter.request_device(...)` выполняется автоматическое переключение на `OffscreenBackend::SoftwareCpu` вместо возврата ошибки.
- Добавлены вспомогательные методы `OffscreenRenderer::new_software_cpu()`, `is_software_cpu()`, `backend_name()`, `device() -> Option<&wgpu::Device>`, `queue() -> Option<&wgpu::Queue>` и поддержка переменной окружения `MCP_CAD_FORCE_CPU_RENDERER=1` для детерминированного выбора CPU-режима.
- Все существующие маршруты рендеринга (`render_mesh_png`, `render_mesh_png_with_material`, `render_mesh_png_debug`, `render_fea_heatmap_png`, `render_part_*`, `*_base64`) прозрачно работают на обоих бэкендах.

### 1.3. Тесты (`crates/mcp-cad-viewer/src/offscreen.rs`, `crates/mcp-cad-viewer/tests/viewer_tests.rs`)
- Добавлен unit-тест `offscreen::tests::test_p5_t9_software_cpu_fallback_renders_valid_non_uniform_pngs_and_heatmap`, проверяющий генерацию валидных PNG/Base64 без WGPU, непустую затенённую геометрию, все 5 отладочных режимов (включая янтарные пиксели открытых рёбер в `BoundaryEdges`), сине-красный градиент тепловой карты FEA и паритет валидации ошибок (`InvalidDimensions`, `StressScalarMismatch`).
- В `tests/viewer_tests.rs` сняты молчаливые пропуски (`Skipping offscreen test...`) при отсутствии GPU — все тесты теперь строго требуют успешной инициализации `OffscreenRenderer::new()`, а также добавлен интеграционный тест `test_p5_t9_cpu_fallback_and_shared_renderer_without_gpu` для всех 4 пресетов камеры и `OffscreenRenderer::shared()`.

---

## 2. Результаты верификации (в headless Linux-контейнере без GPU и Vulkan ICD)

1. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
2. `cargo test -p mcp-cad-viewer --no-fail-fast`: **14/14 PASS** (`0 failed`):
   - `--lib`: **3/3 PASS** (`0.03s`, включая `test_p5_t9_software_cpu_fallback_renders_valid_non_uniform_pngs_and_heatmap`);
   - `tests/viewer_tests.rs`: **11/11 PASS** (`12.36s`, включая `test_debug_render_modes` и `test_p5_t9_cpu_fallback_and_shared_renderer_without_gpu`).
3. `cargo test -p mcp-cad-server --no-fail-fast` (впервые полностью разблокирован в headless-контейнере без Vulkan ICD, `EngineState::new()` проходит на `SoftwareCpu`):
   - `tests/bench_suite.rs`: **1/1 PASS** (`7.53s`);
   - `tests/gear_kinematic_test.rs`: **2/2 PASS** (`0.81s`);
   - `tests/qa_wide_scan.rs`: **13/13 PASS** (`9.39s`, включая `test_bug_010_viewport_and_preview_validation`, `test_bug_003_partial_revolve_mesh_and_renders`, `test_bug_004_wheel_rim_manifold_and_renders`);
   - `tests/mcp_contract_tests.rs`: **61/62 PASS** (включая `test_mcp_validate_mesh_and_debug_viewport_modes` и `test_mcp_e2e_engineering_workflow`; 1 несвязанное падение `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract` из `P5-T1` заведено в `P5-T17`);
   - `--lib`: **68/69 PASS** (1 несвязанное падение `tools::project::tests::test_save_and_load_roundtrip` из `P5-T1` заведено в `P5-T17`).

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)

- **`P5-T17 [S]`**: Разблокировка полного прогона `cargo test -p mcp-cad-server` в headless-контейнере выявила два устаревших после `P5-T1` утверждения в тестах:
  1. `crates/mcp-cad-server/src/tools/project.rs:682` (`test_save_and_load_roundtrip`): ожидает `materials_count == 9` (`8 builtins + 1 custom`) вместо фактических `10` (`9 builtins + 1 custom` после добавления `borosilicate_glass`).
  2. `crates/mcp-cad-server/tests/mcp_contract_tests.rs:4659` (`test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`): список `unit_keywords` не распознаёт единицу `"J/m²"` поля `define_material.fracture_energy_j_m2`.
