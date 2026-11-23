# Отчёт по задаче P5-T15: Передача полной поверхностной сетки (`vertices`/`indices`) деформированных тел и отделившихся фрагментов из `query_simulation_frame` (`P5-T6`) во фронтенд-рендерер

**Статус:** ✅ Выполнено  
**Дата:** 2026-04-12  

---

## 1. Описание проблемы
До задачи `P5-T15` эндпоинт `query_simulation_frame` (`P5-T6`) передавал только плоский список узловых координат `nodal_positions_m`, скалярные поля (`von_mises_mpa`, `plastic_strain`, `damage`) и rigid-body позы отделившихся фрагментов (`detached_fragments` без треугольной сетки). Из-за этого фронтенд-адаптер `convertRustCoupledSimToDestructionCache` в `src/cad/field_colormaps.ts`:
- обновлял `deformedPositions` только при точном совпадении количества узлов FEM-сетки с количеством вершин исходного `CadPart.mesh` (`hasExactVertexMatch`), что для произвольных тетраэдральных сеток (`TET4`/`TET10`) практически никогда не выполнялось;
- не мог отобразить изменение внешней топологии (полости от удалённых `eroded_element_indices` и отколовшихся кусков);
- заменял реальную геометрию отделившихся 3D-осколков приближёнными параллелепипедами (`buildSmallFragmentBox`).

Кроме того, при сквозной проверке Rust ↔ TypeScript протокола были обнаружены и устранены расхождения в именах полей JSON-ответов `query_simulation_frame` (`Vec<[f64; 3]>` против плоского `number[]`, `source_part_id`, `position_m`, `orientation_quat`, `elastic_j`/`plastic_j`/`fracture_j`, `kind`/`detail`, `point_m`, `frame_count`/`elapsed_ms`) и условие видимости осколков `showShard` для `solverBackend === 'rust_fem_f64'` в `CadViewport3D.tsx`.

---

## 2. Реализованное решение

### 2.1. Извлечение внешней поверхностной сетки и геометрии осколков в Rust (`crates/mcp-cad-server`)
1. **`extract_active_surface_topology(mesh, active_mask)` (`crates/mcp-cad-server/src/tools/coupled_sim.rs`)**:
   - Находит граничные грани активного подмножества тетраэдров (с учётом исключения `eroded_element_indices` и элементов, перешедших в `detached_fragments`).
   - Поддерживает как линейные `TET4` (1 треугольник на граничную грань), так и квадратичные `TET10` (4 подтреугольника с использованием 6 узлов грани: `[v0, m01, m20]`, `[m01, v1, m12]`, `[m20, m12, v2]`, `[m01, m12, m20]`).
   - Проверяет и ориентирует обход вершин каждого граничного треугольника строго по внешней нормали (против часовой стрелки, CCW) относительно четвёртой вершины тетраэдра.
   - Компактифицирует вершины поверхности в `surface_node_indices: Vec<usize>` и строит локальный индексный буфер `surface_indices: Vec<u32>`.
   - Дедуплицирует `ActiveSurfaceTopology` между кадрами в `BakedSimulationRecord::part_surface_topologies` (новая топология сохраняется только при реальном изменении маски активных элементов из-за эрозии или откола).
2. **Кэширование геометрии отделившихся фрагментов (`BakedFragmentMesh`)**:
   - При первом появлении `fragment_id` в `detached_fragments` его локальная поверхностная треугольная сетка (`vertices_m` относительно центра масс фрагмента, `normals`, `indices`, `bounding_radius_m`) извлекается один раз за симуляцию и сохраняется в `BakedSimulationRecord::fragment_meshes`.
   - Добавлен метод `CoupledSimJobManager::query_fragment_meshes` и MCP/HTTP-обёртка `query_cached_fragment_meshes_from_manager`.
3. **Расширение `query_simulation_frame` и HTTP REST API (`crates/mcp-cad-server/src/sse.rs`, `src/tools/mod.rs`)**:
   - В каждый элемент `parts[]` ответа `query_simulation_frame` добавлены `surface_vertex_count`, `surface_triangle_count`, а при `include_mesh_fields=true` — `surface_positions_m`, `surface_reference_positions_m`, `surface_indices`, `surface_node_indices`.
   - В каждый элемент `detached_fragments[]` добавлены `position_m`, `bounding_radius_m` и (при `include_fragment_meshes=true`) встроенный объект `mesh`.
   - Добавлены HTTP-маршруты `GET /api/coupled-sim/fragments` и `GET /api/coupled-sim/fragments/:sim_id` для однократной загрузки геометрии всех отделившихся осколков симуляции, а также query-параметр `include_fragment_meshes` в `GET /api/coupled-sim/frames/:sim_id/:i`.

### 2.2. Адаптер, сетевой мост и 3D-рендерер во фронтенде (`src/`)
1. **Типы (`src/types/cad.ts`)**:
   - Добавлен интерфейс `CachedFemSurfaceMesh` (`vertexCount`, `triangleCount`, `positions`, `referencePositions`, `indices`, `nodeIndices`) и поле `femSurfaceMesh?: CachedFemSurfaceMesh` в `CachedPartFrameState`.
2. **Конвертер (`src/cad/field_colormaps.ts`)**:
   - Расширены интерфейсы `RustSimulationSummary`, `RustSimulationFramePayload`, `RustSimulationFramePartState`, `RustDetachedFragmentState` и добавлен `RustFragmentMeshPayload`.
   - Реализована сборка `CachedFemSurfaceMesh` из `surface_positions_m` / `surface_reference_positions_m` / `surface_indices` / `surface_node_indices` с выборкой узловых скалярных полей (`von_mises_mpa`, `plastic_strain`, `damage`) по `surface_node_indices` — без требования совпадения числа узлов с `CadPart.mesh`.
   - Реализована функция `buildFragmentGeometryFromRustMesh`, строящая реальную `FractureShardGeometry` из `summary.fragment_meshes` или встроенного `frag.mesh` (с автоматическим расчётом нормалей при их отсутствии). Fallback `buildSmallFragmentBox` сохранён только на случай отсутствия сетки в ответе.
3. **Сетевой мост (`src/cad/native_bridge.ts`)**:
   - `bakeAndFetchCoupledSimulation` однократно запрашивает `GET /api/coupled-sim/fragments/:sim_id?length_unit=m` после завершения расчёта и запрашивает покадровые данные с `include_mesh_fields=true&include_fragment_meshes=false`.
4. **3D-вьюпорт (`src/components/CadViewport3D.tsx`)**:
   - Добавлен `femSurfaceEntriesRef`: при наличии `pState.femSurfaceMesh` создаётся и обновляется `THREE.BufferGeometry` внешней поверхности FEM-тела в мировых координатах (м) с поддержкой масштабирования деформаций (`1×`/`5×`/`20×`), динамического обновления `indices` при эрозии/отколе элементов и раскраски по всем 4 режимам (`shaded`, `von_mises`, `plastic_strain`, `damage`), а исходный CAD-меш детали скрывается.
   - Исправлено условие видимости осколков `showShard` для `destructionCache.solverBackend === 'rust_fem_f64'` (`frame.shardActive[i] === 1`).

---

## 3. Изменённые файлы
- `crates/mcp-cad-server/src/tools/coupled_sim.rs`
- `crates/mcp-cad-server/src/tools/mod.rs`
- `crates/mcp-cad-server/src/sse.rs`
- `src/types/cad.ts`
- `src/cad/field_colormaps.ts`
- `src/cad/native_bridge.ts`
- `src/components/CadViewport3D.tsx`
- `src/cad/__tests__/field_colormaps.test.ts`

---

## 4. Верификация и тесты
1. **Rust интеграционные и модульные тесты (`mcp-cad-server`)**:
   - Добавлен `test_p5_t15_active_surface_topology_outward_winding_and_erosion_cavity` (проверяет замкнутый объём `0.04 * 0.02 * 0.02` по внешним CCW-нормалям для `TET4` и `TET10` и уменьшение объёма с образованием полости при эрозии тетраэдра).
   - Добавлен `test_p5_t15_query_simulation_frame_returns_active_surface_mesh_and_fragment_geometry` (запускает HHT-α расчёт консольного изгиба балки из `borosilicate_glass` `0.10 x 0.016 x 0.016 м` с отрывом дальнего фрагмента, проверяет `surface_positions_m`, `surface_reference_positions_m`, `surface_indices`, `surface_node_indices`, изменение поверхностной топологии после эрозии, `GET /api/coupled-sim/fragments/:sim_id` и параметр `include_fragment_meshes`).
   - Команда: `cargo test -p mcp-cad-server --lib sse::tests --no-fail-fast` → **6 passed, 0 failed**.
2. **Rust Clippy**:
   - Команда: `cargo clippy --workspace --all-targets -- -D warnings` → **0 ошибок, 0 предупреждений**.
3. **TypeScript тесты (`node:test`)**:
   - Добавлен тест `P5-T15: convertRustCoupledSimToDestructionCache preserves active FEM surface mesh and real detached fragment geometry without proxy box` в `src/cad/__tests__/field_colormaps.test.ts`.
   - Команда: `npx tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts` → **74 passed, 0 failed**.
4. **Сборка, линтинг и браузерная проверка (`Playwright Chromium 1440×900`)**:
   - `compile_applet` и `lint_applet` проходят без ошибок.
   - Проверен рендеринг активной FEM-поверхности с полостью эрозии, цветовой картой `σ_vM` и отделившимся FEM-фрагментом во вьюпорте `http://localhost:3000/?bridge=live` (`0` ошибок консоли).
