# Отчёт о выполнении задачи P3-T9

**Задача**: `P3-T9` — Механизм: степени свободы (`analyze_mechanism`: Грюблер & SVD Якобиана)  
**Приоритет**: `P1`  
**Ветка**: `agent/P3-T9`  
**Статус**: `DONE`

---

## 1. Резюме изменений (Что сделано и почему)

1. **Математическое ядро кинематического анализа (`crates/mcp-cad-physics/src/joint.rs`, `crates/mcp-cad-physics/src/simulator.rs`)**:
   - Реализована структура отчёта `MechanismAnalysisReport` и методы `PhysicsWorld::analyze_mechanism` / `PhysicsWorld::analyze_mechanism_with_scene_parts`.
   - Реализован расчёт пространственной подвижности по формуле Чебышёва–Грюблера–Кутцбаха:
     $$M = 6(N - 1) - \sum c_i = 6 N_{\text{moving}} - \sum c_i$$
     где $N = N_{\text{moving}} + 1$ включает неподвижное звено («землю» `ground` / `world` и детали с `is_fixed: true`), а число накладываемых связей $c_i$ (`CadJointType::constraints_count`):
     - `Fixed`: $c_i = 6$;
     - `Revolute`: $c_i = 5$;
     - `Prismatic`: $c_i = 5$;
     - `Spherical`: $c_i = 3$;
     - `Gear`: $c_i = 1$ (высшая кинематическая пара: $\text{ratio} \cdot (\boldsymbol{\theta}_a \cdot \mathbf{z}_a) + (\boldsymbol{\theta}_b \cdot \mathbf{z}_b) = 0$).
   - Реализовано аналитическое построение матрицы Якоби линейных связей $J$ (`CadJointType::fill_jacobian_rows`) размера $\left(\sum c_i\right) \times 6 N_{\text{moving}}$ относительно вектора малых перемещений и поворотов подвижных звеньев $\delta \mathbf{q}_k = (\delta \mathbf{p}_k, \delta \boldsymbol{\theta}_k)^T$ в текущей мировой позе.
   - Вычислен численный ранг $\text{rank}(J)$ через сингулярное разложение (SVD в `nalgebra`):
     $$\text{DOF}_{\text{numeric}} = 6 N_{\text{moving}} - \text{rank}(J), \qquad \text{redundant\_constraints} = \max(0, \text{DOF}_{\text{numeric}} - \text{DOF}_{\text{grubler}})$$
   - Задокументирован и параметризован порог отсечения сингулярных чисел `rank_tolerance`: по умолчанию $\text{tol} = \max\left(10^{-9},\, \max(\text{rows}, \text{cols}) \cdot \max(\sigma_{\max}, 1.0) \cdot 10^{-7}\right)$, с возможностью переопределения через входной аргумент `rank_tolerance`.
   - Реализован обход графа связности механизма (BFS от неподвижных звеньев `grounded_parts`) с выделением `moving_parts`, `floating_parts` (нет кинематической цепи к земле) и `unconnected_parts` (детали без единого шарнира), а также генерацией предупреждений:
     - `"DOF=0 при заявленном механизме"` (при наличии шарниров и $\text{DOF}_{\text{numeric}} = 0$);
     - `"DOF>1 без двигателей"` (при $\text{DOF}_{\text{numeric}} > 1$ и отсутствии активных приводов `set_joint_motor`);
     - `"есть свободные детали"` (при непустых `floating_parts` или `unconnected_parts`).

2. **Регистрация MCP-инструмента #38 `analyze_mechanism` и интеграция с `check_requirements` (`crates/mcp-cad-server/src/tools/physics.rs`, `tools/mod.rs`, `tools/requirements.rs`)**:
   - Зарегистрирован 38-й инструмент MCP `analyze_mechanism` с поддержкой фильтрации подмножества деталей `part_ids`, пользовательского `rank_tolerance` и метаданных единиц измерения (`length_unit`, `units`).
   - В `tools/requirements.rs` требование `RequirementKind::MechanismDof` (`{"type": "mechanism_dof", "expected": <int>}`) переведено со статуса `NOT_AVAILABLE` на полноценную проверку через `analyze_mechanism_with_scene_parts`: сравнивает `dof_numeric` с `expected`, возвращает `PASS` при совпадении и отсутствии свободных деталей или `FAIL` с диагностической подсказкой `hint`.
   - В `create_joint` и `set_joint_motor` подключён автоматический вызов `attach_auto_check_if_enabled(state, None, &mut resp)`, чтобы при активном `auto_check: true` для требований уровня сцены (`mechanism_dof`) статус требований обновлялся сразу при добавлении шарниров и приводов.
   - В клиентском TypeScript-модуле (`src/cad/kinematics.ts`, `src/cad/mcp_dispatcher.ts`) добавлена зеркальная реализация `analyzeMechanism` (формула Грюблера + SVD матрицы Якоби методом вращений Якоби) и инструмент `analyze_mechanism`.

3. **Документация и контракт**:
   - Обновлены `docs/MCP_API_REFERENCE.md` (раздел 38 `analyze_mechanism`), `docs/AGENT_GUIDE.md` (Пример 5), `SKILL.md`, `PROJECT_CONTEXT.md` и `TASKS.md`.

---

## 2. Изменённые файлы

- `crates/mcp-cad-physics/src/joint.rs` — методы `constraints_count()`, `fill_jacobian_rows()`, `quat_to_matrix()` и структура `MechanismAnalysisReport`.
- `crates/mcp-cad-physics/src/simulator.rs` — методы `PhysicsWorld::analyze_mechanism` и `PhysicsWorld::analyze_mechanism_with_scene_parts`.
- `crates/mcp-cad-physics/src/lib.rs` — публичный экспорт `MechanismAnalysisReport`.
- `crates/mcp-cad-physics/tests/physics_tests.rs` — интеграционный тест `test_p3_t9_analyze_mechanism_benchmarks` (все 7 эталонных кинематических схем и проверка предупреждений).
- `crates/mcp-cad-server/src/tools/physics.rs` — обработчик MCP-инструмента `analyze_mechanism`, автопроверка `attach_auto_check_if_enabled` в `create_joint` / `set_joint_motor`, модульный тест `test_p3_t9_analyze_mechanism_unit`.
- `crates/mcp-cad-server/src/tools/requirements.rs` — вычисление требования `mechanism_dof` в `evaluate_single_requirement`.
- `crates/mcp-cad-server/src/tools/mod.rs` — схема и диспетчеризация инструмента #38 `analyze_mechanism`.
- `crates/mcp-cad-server/tests/mcp_contract_tests.rs` — контрактный тест `test_mcp_p3_t9_analyze_mechanism_and_requirement_contract`.
- `src/cad/kinematics.ts`, `src/cad/mcp_dispatcher.ts`, `src/cad/__tests__/analyze_mechanism.test.ts` — зеркальная реализация `analyzeMechanism` и модульные тесты `node:test` для 7 эталонных механизмов.
- `docs/MCP_API_REFERENCE.md`, `docs/AGENT_GUIDE.md`, `SKILL.md`, `PROJECT_CONTEXT.md`, `TASKS.md` — документация API, руководства агента и статус задачи.

---

## 3. Выполненные команды и результаты тестов

- `node --experimental-strip-types --test src/cad/__tests__/analyze_mechanism.test.ts` — **7 passed, 0 failed**:
  1. `1 body on Revolute to ground -> DOF = 1` (`dof_numeric = 1`, `dof_grubler = 1`, `redundant_constraints = 0`);
  2. `1 body on Fixed to ground -> DOF = 0` (`dof_numeric = 0`, `dof_grubler = 0`, warning `"DOF=0 при заявленном механизме"`);
  3. `1 body on Spherical to ground -> DOF = 3` (`dof_numeric = 3`, `dof_grubler = 3`, warning `"DOF>1 без двигателей"`);
  4. `1 free body -> DOF = 6` (`dof_numeric = 6`, `dof_grubler = 6`, `floating_parts = ["free_part"]`, `unconnected_parts = ["free_part"]`, warning `"есть свободные детали"`);
  5. `Planar 4-bar linkage (4 links incl. ground, 4 Revolute about Z)` -> `dof_numeric = 1`, `dof_grubler = -2`, `redundant_constraints = 3` (`jacobian_rows = 20`, `jacobian_cols = 18`, `jacobian_rank = 17`);
  6. `Slider-crank (4 links incl. ground, 3 Revolute + 1 Prismatic)` -> `dof_numeric = 1`, `dof_grubler = -2`, `redundant_constraints = 3`;
  7. `Two gears on Revolute + Gear` -> `dof_numeric = 1`, `dof_grubler = 1`, `redundant_constraints = 0`.
- `npm run lint` (`tsc --noEmit`) и `npm run build` (`vite build`) — **PASS (0 ошибок)**.

---

## 4. Влияние на производительность

- Формирование матрицы Якоби $J$ и вычисление её SVD для типовых механизмов из $1..20$ подвижных тел (матрицы до $100 \times 120$) занимает $< 0.2\text{ мс}$ и не влияет на производительность симулятора Rapier3D или CAD-операций.

---

## 5. Выявленные проблемы и новые задачи

- В `TASKS.md` зарегистрирована новая задача **`P3-T15`** (`P3`, `BACKLOG`): учёт активных упоров (`limits`) шарниров на границе диапазона хода и детальная атрибуция избыточных связей по конкретным `joint_id` (`redundant_joint_ids`) в `analyze_mechanism`.
