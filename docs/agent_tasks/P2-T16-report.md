# Отчёт о выполнении задачи P2-T16

**Задача:** `P2-T16 [S]` — Строгая валидация нераспознанных `fixed_faces` и пустых `applied_forces` в `run_fea_analysis` (`tools/fea.rs`) и учёт соседних ячеек в `weld_vertices` (`import_stl.rs`)  
**Дата:** 08.11.2026  
**Статус:** ✅ DONE  
**Затронутые файлы:**
- `crates/mcp-cad-server/src/tools/fea.rs`
- `crates/mcp-cad-server/src/error.rs`
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs`
- `crates/mcp-cad-core/src/import_stl.rs`
- `src/cad/cad_engine.test.ts`
- `src/cad/primitives.ts`
- `TASKS.md`
- `docs/agent_tasks/P2-T16-report.md`

---

## 1. Описание проблемы

В рамках аудита контрактов MCP-CAD и обнаруженных замечаний при выполнении задач **P2-T5** и **P2-T8** были зафиксированы следующие дефекты:

1. **Молчаливое игнорирование нераспознанных граней и опечаток в `fixed_faces` (`crates/mcp-cad-server/src/tools/fea.rs`)**:
   - В `run_fea_analysis` при разборе массива `fixed_faces` строки, содержащие опечатки (например, `"min_z "`) или нераспознанные имена граней, не вызывали ошибку на этапе парсинга AABB, а передавались в `resolve_face_triangles_and_info`. Если строка не соответствовала ни одной грани детали, требовалось гарантировать возврат структурированной ошибки `FACE_NOT_FOUND` с привязкой параметра `param: "face_selector"`.
   - При передаче пустого массива `fixed_faces: []` модель оставалась незакреплённой, что должно строго отвергаться с кодом `FEA_UNCONSTRAINED` и `param: "fixed_faces"`.

2. **Ложноположительный расчёт МКЭ при пустом или невалидном `applied_forces`**:
   - Если клиент передавал пустой массив `applied_forces: []` или массив с объектами без обязательных ключей `force` / `force_vector` (например, `[{"wrong_key": ...}]`), ни одна сила не регистрировалась (`loads_added == false`). При наличии явного ключа `applied_forces` сервер должен возвращать ошибку `FEA_NO_LOADS` с `param: "applied_forces"`, а не рассчитывать фиктивное нулевое напряжение.
   - При передаче массива `position` некорректной размерности (например, 2 координаты вместо 3) должна возвращаться ошибка `INVALID_ARGUMENT` с `param: "position"`.

3. **Сварка вершин `weld_vertices` в `import_stl.rs`**:
   - Проверена и подтверждена реализация пространственного хеширования с поиском по всем 27 соседним ячейкам `(-1..=1)^3` по расстоянию Евклида `||p - q|| <= tolerance`, что предотвращает ложный разрыв смежных граней на границах воксельных ячеек сетки.

---

## 2. Выполненные изменения

1. **Строгая валидация и структурированные ошибки в `run_fea_analysis` (`crates/mcp-cad-server/src/tools/fea.rs`)**:
   - Обеспечен строгий возврат `ServerError::InvalidArgument` при нераспознанных гранях в `fixed_faces`, пустом массиве `fixed_faces: []` (`FEA_UNCONSTRAINED`, `param: "fixed_faces"`).
   - Внедрена проверка `explicit_loads_key && !loads_added`: при пустом массиве `applied_forces: []` или отсутствии корректного вектора силы возвращается структурированная ошибка `FEA_NO_LOADS` (`param: "applied_forces"`).
   - Внедрена проверка размерности `position`: массив должен строго содержать 3 конечных числа, иначе возвращается `INVALID_ARGUMENT` (`param: "position"`).

2. **Контракт ошибок `ErrorBody` (`crates/mcp-cad-server/src/error.rs`)**:
   - Синхронизировано сопоставление ошибок: `FEA_UNCONSTRAINED` возвращает `param: "fixed_faces"`, `FEA_NO_LOADS` возвращает `param: "applied_forces"`, а невалидные векторы `position` возвращают `param: "position"`.

3. **Тестовая верификация**:
   - Unit-тест `test_p2_t16_fea_rejects_unrecognized_fixed_faces_and_empty_or_invalid_applied_forces` в `crates/mcp-cad-server/src/tools/fea.rs`.
   - Контрактный e2e-тест `test_mcp_p2_t16_fea_strict_fixed_faces_and_applied_forces_validation` в `crates/mcp-cad-server/tests/mcp_contract_tests.rs`.
   - Исправлена рассинхронизация пресетов в TypeScript-тесте `src/cad/cad_engine.test.ts` (5 пресетов в `FEA_QUICK_LOAD_PRESETS`) и добавлен `type`-only импорт `AngleHistogramBuckets` в `src/cad/primitives.ts`.

---

## 3. Результаты проверки

- `npx tsx --test src/cad/cad_engine.test.ts`: **28 passed, 0 failed (100% PASS)**.
- `npm run lint` (`tsc --noEmit`): **0 errors**.
- `cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_p2_t16`: **PASS**.
- `cargo test --workspace --no-fail-fast`: **223+ passed, 0 failed (100% PASS)**.
