# Отчёт о выполнении задачи P2-T38

## Задача
**P2-T38 [M]: Разделение фатальных ошибок топологии (`FAIL`) и предупреждений о микро-гранях (`warnings` / `sliver_triangles`) в `validate_mesh` на производственной геометрии (`cut_helical_groove`), добавление `strict_slivers`, `is_valid_solid` и `fea_ready`**

---

## 1. Контекст и обнаруженные дефекты

1. **Ложноположительный `FAIL` на реальных производственных деталях**:
   - Ранее `validate_mesh` считал любое превышение порога sliver-треугольников (`lt_1_deg_count > 0` или `lt_5_deg_ratio > 1%`) критической ошибкой в массиве `issues`, что переводило статус сетки в `FAIL`.
   - При моделировании реальных инструментов (например, спирального сверла с винтовыми стружечными канавками `cut_helical_groove` или резьбы) на линии сбега винтовой поверхности в коническую вершину неизбежно возникают острые клиновидные треугольники, даже если сетка **на 100% замкнута, водонепроницаема и манифольдна** (`is_watertight == true`, `boundary_edges == 0`, `non_manifold_edges == 0`, `degenerate == 0`, `self_intersections == 0`).
   - Из-за этого ИИ-агенты считали модель бракованной и прерывали выполнение инженерных сценариев.

---

## 2. Внесённые изменения

1. **`crates/mcp-cad-core/src/diagnostics.rs`**:
   - В структуру `MeshValidationReport` добавлены поля:
     - `warnings: Vec<String>` — некритические замечания к качеству углов поверхностной сетки.
     - `is_valid_solid: bool` — флаг замкнутого водонепроницаемого манифольдного твердотельного тела.
     - `fea_ready: bool` — готовность сетки к тетраэдризации МКЭ.
   - Реализована функция `validate_mesh_options(mesh, expected_genus, strict_slivers)`:
     - При `strict_slivers == false` (режим по умолчанию для производственных деталей) дефекты углов переносятся в `warnings`, и при замкнутой топологии возвращается `status: "PASS"`, `is_valid_solid: true`, `fea_ready: true`.
     - При `strict_slivers == true` (сохраняется для тестов генераторов базовых примитивов) нарушение порогов приводит к `status: "FAIL"`.
   - Добавлена функция `validate_mesh_strict`.

2. **`crates/mcp-cad-server/src/tools/cad.rs` и `tools/mod.rs`**:
   - В схему инструмента `validate_mesh` добавлено опциональное булево свойство `strict_slivers` (по умолчанию `false`).
   - `validate_mesh_tool` парсит параметр и вызывает `validate_mesh_options`.

3. **`crates/mcp-cad-server/src/tools/requirements.rs`**:
   - Проверка требования `RequirementSpec::MeshValid` признаётся пройденной (`PASS`), если `report.is_valid_solid == true` и `issues.is_empty()`, а наличие `warnings` выводится в информативном поле `hint`.

4. **`src/cad/primitives.ts` и `src/cad/mcp_dispatcher.ts`**:
   - Функция `validatePartMesh` в TypeScript синхронизирована с алгоритмом Rust: принимает `strictSlivers: boolean = false`, формирует массивы `issues` и `warnings`, вычисляет `isValidSolid` и `feaReady`.
   - Диспетчер `validate_mesh` поддерживает свойство `strict_slivers`.

---

## 3. Верификация и тесты

- В `src/cad/cad_engine.test.ts` обновлён тест `BUG-018`:
  - Проверено, что при `strict_slivers: false` сплющенный цилиндр возвращает `isValidSolid: true`, `status: 'PASS'`, и предупреждения о sliver в `warnings`.
  - Проверено, что при `strict_slivers: true` тот же цилиндр возвращает `status: 'FAIL'`.
- В тесте `P2-T37..P2-T40` проверена работа `validate_mesh` со свойствами `strict_slivers`, `is_valid_solid` и `fea_ready`.
- Тесты проходят: `npx tsx --test --test-name-pattern="BUG-018|P2-T37" src/cad/cad_engine.test.ts` — **100% PASS**.
