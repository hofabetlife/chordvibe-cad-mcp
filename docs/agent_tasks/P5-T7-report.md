# Отчёт по задаче `P5-T7`: Визуализация реальных деформаций, полей $\sigma_{vM}$ / $\bar{\varepsilon}_p$ / $D$, локальных сколов и баланса энергий в 3D-вьюпорте и таймлайне

- **Дата выполнения**: 20.11.2026
- **Статус**: ✅ DONE

---

## 1. Реализованные изменения

### 1.1. Контракты покадровых полей и детерминированные цветовые карты (`src/types/cad.ts`, `src/cad/field_colormaps.ts`)
- В `src/types/cad.ts` добавлены типы `DestructionFieldMode` (`'shaded' | 'von_mises' | 'plastic_strain' | 'damage'`), `DeformationScalePreset` (`1 | 5 | 20`), `SimulationSolverBackend` (`'rust_fem_f64' | 'web_local_preview'`) и расширены структуры `CachedPartFrameState`, `DestructionCacheFrame`, `DestructionFrameMetrics`, `DestructionTimelineEvent` (`'yield_onset'`, `'fragment_detachment'`) и `DestructionCache`.
- Создан чистый тестируемый модуль `src/cad/field_colormaps.ts`:
  - `evalVonMisesColormap` (инженерная шкала Jet для $\sigma_{vM}$), `evalPlasticStrainColormap` (высококонтрастная шкала Plasma для необратимой $\bar{\varepsilon}_p$ с выделением упругой базы), `evalDamageColormap` (шкала повреждаемости $D \in [0, 1]$);
  - `computeYieldMarkerPct` и `buildFieldLegend` — расчёт положения маркера предела текучести $\sigma_y$ на цветовой шкале $\sigma_{vM}$;
  - `applyScaledDeformation` и `fillVertexFieldColors` — zero-GC расчёт масштабированных координат $\mathbf{x}(t) = \mathbf{X} + k \cdot \mathbf{u}(t)$ ($k \in \{1\times, 5\times, 20\times\}$) и буфера вершинных цветов;
  - `buildEnergyBalanceSeries` — генерация 4-серийного SVG-графика баланса энергий ($E_k, E_e, E_p, E_f$);
  - `convertRustCoupledSimToDestructionCache` — адаптер JSON-кадров нативного бэкенда `mcp-cad-server` (`/api/coupled-sim/frames/:sim_id/:i`) в формат `DestructionCache` с меткой `'Rust FEM f64'`.

### 1.2. Покадровые поля деформаций и энергетический баланс в клиентском движке (`src/cad/destruction_engine.ts`)
- Для каждой деформируемой детали предвычисляются локальные цилиндрические координаты вершин (`vertRadiusXY`, `vertAngle`) и ведётся необратимая история пластичности (`plasticStrainVert`), повреждаемости (`damageVert`) и остаточного пластического смещения (`plasticDispVert`).
- На каждом кадре синтезируются локализованный изгиб и замятие зубьев в зоне контакта (`deformedDisplacements`), концентрация напряжений $\sigma_{vM}$ в корневой галтели (`nodalStressMPa`), монотонно неубывающие $\bar{\varepsilon}_p$ и $D$ (`nodalPlasticStrain`, `nodalDamage`), скалярные поля на отделившихся фрагментах (`shardStressMPa`, `shardPlasticStrain`, `shardDamage`), а также физически сбалансированные каналы энергий: кинетической ($E_k$), объёмно-интегрированной упругой ($E_e$), накопительной пластической диссипации ($E_p$) и поверхностной энергии разрушения Гриффитса ($E_f$).
- Журнал событий дополнен записями `yield_onset` (`Начало текучести σ_vM > σ_y`) и `fragment_detachment` (`Отделение фрагментов`).
- Кэши клиентского фолбэка явно маркируются `solverBackend: 'web_local_preview'`, `solverLabel: 'Web Local Preview'`.

### 1.3. Интеграция с нативным Rust FEM бэкендом (`src/cad/native_bridge.ts`, `src/App.tsx`)
- В `src/cad/native_bridge.ts` добавлены `bakeNativeCoupledSimulation` (запуск `POST /api/coupled-sim/bake`, подписка на прогресс SSE `/api/coupled-sim/events` с поллингом `/api/coupled-sim/status/:job_id`, загрузка кадров `/api/coupled-sim/frames/:sim_id/:i?include_mesh_fields=true&length_unit=m`) и `cancelNativeCoupledSimulation`.
- В `src/App.tsx` запекание сначала пытается использовать активный нативный Rust-сервер (`Rust FEM f64`) и автоматически откатывается на клиентский `Web Local Preview` с явной индикацией активного решателя в UI.

### 1.4. 3D-вьюпорт, панель телеметрии и таймлайн (`src/components/{CadViewport3D,CadDestructionPanel,CadDestructionTimeline,CadRightSidebar}.tsx`)
- В `CadViewport3D.tsx`:
  - Реализовано in-place обновление деформированной геометрии (`applyScaledDeformation` + `computeVertexNormals`) и вершинных цветовых карт (`fillVertexFieldColors` с корректным преобразованием sRGB → Linear для `THREE.MeshStandardMaterial` и `THREE.SRGBColorSpace` для фрагментов);
  - Неподвижные опоры без полей деформаций (плита испытательного стенда) в режимах скалярных карт сохраняют нейтральный тёмно-стальной вид;
  - Добавлен компактный HUD `ПОЛЕ ДЕФОРМАЦИЙ` (`data-testid="destruction-field-legend"`) с бейджем решателя (`Rust FEM f64` / `Web Local Preview`), переключателями 4 режимов (`Shaded`, `σ_vM`, `ε_p`, `D`), масштаба деформаций (`1×`, `5×`, `20×`), шкалой с маркером предела текучести $\sigma_y$ (`data-testid="yield-marker-indicator"`) и живыми метриками `σ_vM`, `u_max`, `ε_p`, `D_max`.
- В `CadDestructionPanel.tsx` и `CadDestructionTimeline.tsx`:
  - Добавлен явный индикатор решателя (`solver-backend-badge`, `timeline-solver-badge`), переключатели режимов полей и масштаба деформаций;
  - Добавлен многосерийный SVG-график **Баланс энергий ($E_k, E_e, E_p, E_f$)** (`energy-balance-chart`) с числовыми значениями и переключением трека таймлайна (`Энергии` / `σ_vM`);
  - Обновлён кликабельный **Журнал событий** (`destruction-event-log`) с цветовой маркировкой типов событий (`Контакт`, `Текучесть`, `Скол`, `Фрагмент`, `Раскол`, `Стенд`).

### 1.5. Паритет в нативном Rust-вьюпорте (`crates/mcp-cad-viewer/src/{shader.rs,lib.rs}`)
- Добавлены перечисление `FieldColormap` (`VonMisesJet`, `PlasticStrain`, `Damage`), uniform-структура `FieldColormapUniform`, WGSL-шейдер `FIELD_COLORMAP_SHADER_WGSL`, функции `eval_field_colormap` и `deform_positions_with_scale` с экспортом в `lib.rs` и 2 модульными тестами.

---

## 2. Результаты верификации

1. **Unit-тесты TypeScript**:
   - `npx --yes tsx --test src/cad/cad_engine.test.ts src/cad/__tests__/analyze_mechanism.test.ts src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts src/cad/__tests__/field_colormaps.test.ts`: **72/72 PASS** (`0 failed`), включая 6 новых тестов `field_colormaps.test.ts` и новый интеграционный тест `P5-T7` в `destruction_engine.test.ts`.
   - Повторный прогон после финальной правки `fractureEnergyJM2`: **PASS** (`0 failed`).
2. **Rust `mcp-cad-viewer`**:
   - `cargo test -p mcp-cad-viewer --lib`: **2/2 PASS** (`field_colormaps_return_valid_rgb_bounds_and_distinct_endpoints`, `deform_positions_scales_linearly_for_1x_5x_20x`).
   - `cargo clippy -p mcp-cad-viewer --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений).
3. **Сборка и проверка типов Web-студии**:
   - `npm run lint` (`tsc --noEmit`): **PASS** (`0` ошибок).
   - `npm run build` (`vite build`): **PASS**.
4. **Живая проверка в браузере (Playwright Chromium, `1440×900`, 2 раунда визуальной критики)**:
   - В Раунде 1 выявлены и исправлены: (а) размытые пастельные цвета вершинных карт из-за отсутствия конвертации sRGB → Linear в Three.js, (б) дисбаланс масштаба кривой $E_e$ относительно $E_k / E_p / E_f$ на графике баланса энергий, (в) наложение toast-уведомления на верхнюю панель камер.
   - В Раунде 2 визуально подтверждены: контрастная карта $\sigma_{vM}$ с маркером `σy 310` и локальным пиком в корневой галтели, карта необратимой $\bar{\varepsilon}_p$ при масштабе `20×` с видимым изгибом/замятием зубьев, карта повреждаемости $D$ со сколом фрагментов при целой ступице, читаемый 4-серийный график $E_k, E_e, E_p, E_f$, бейдж `Web Local Preview` и **`0` ошибок / `0` предупреждений в консоли браузера**.

---

## 3. Выявленные сопутствующие проблемы (добавлены в `TASKS.md`)
- **`P5-T15`**: Передача полной поверхностной сетки (`vertices`/`indices`) деформированных тел и отделившихся фрагментов из `query_simulation_frame` (`P5-T6`) во фронтенд-рендерер, чтобы при несовпадении числа вершин поверхностной сетки `TET10` и исходного `CadPart.mesh` (а также для геометрии отломанных фрагментов) нативный режим `Rust FEM f64` рендерил точную топологию МКЭ без прокси-боксов.
