# Отчёт по задаче P3-T19: Исправление проверки `min_clearance_step` в тесте `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall` и синхронизация TS-зеркала `sweepJoint`

**Дата выполнения:** 15.11.2026  
**Статус:** ✅ DONE  

---

## 1. Цель и анализ проблемы

В тесте [`test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall`](../../crates/mcp-cad-physics/tests/physics_tests.rs#L1573-L1659) проверяется пошаговая развёртка `sweep_joint` поворотного шарнира `j_hinge` рычага длиной `0.10 м` (ширина `0.01 м`, $y \in [-0.005, +0.005]$) в диапазоне $[0^\circ, 90^\circ]$ (`steps = 90`, шаг $1^\circ$) при наличии неподвижной стены ($x \in [0.02, 0.12]\text{ м}$, $y \in [0.05, 0.07]\text{ м}$).

Геометрический анализ показал корректность работы `PhysicsWorld::sweep_joint`:
- В [`JointSweepReport`](../../crates/mcp-cad-physics/src/telemetry.rs#L412-L421) поле `min_clearance` / `min_clearance_step` по контракту отслеживает минимальный зазор **по всем непересекающимся и незаклинившим шагам** траектории (`!intersects`), тогда как диапазон хода до первого столкновения отражается в `safe_travel_range` и `margin_to_collision`.
- На шаге `27` ($27^\circ$, до контакта при $\theta^* \approx 27.492^\circ$) верхний край конца рычага $(0.10, +0.005)$ достигает $y = 0.10\sin 27^\circ + 0.005\cos 27^\circ \approx 0.049854\text{ м}$, что даёт зазор до грани стены $y = 0.05\text{ м}$ около **`0.146 мм`**.
- На шагах `28..=77` рычаг пересекает конечную стену (`first_collision_step == Some(28)`).
- На шаге `78` ($78^\circ$) рычаг выходит за заднюю кромку стены с углом $(0.02, 0.07)$, где локальная координата угла стены относительно оси рычага составляет $y_{\text{loc}} = -0.02\sin 78^\circ + 0.07\cos 78^\circ \approx -0.005009\text{ м}$ против кромки рычага $y_{\text{loc}} = -0.005\text{ м}$, то есть зазор составляет около **`0.009 мм`**, что в ~16 раз меньше предстолкновительного зазора на шаге `27`.

Дополнительно при проверке TypeScript-зеркала [`src/cad/kinematics.ts`](../../src/cad/kinematics.ts#L1637) выявлено расхождение в ветке открытой кинематической цепи `sweepJoint`: обновление `minClearance` / `minClearanceStep` проверяло только `rep.min_clearance !== undefined` без условия `&& interferingPairs.length === 0` (в отличие от замкнутого контура на строке `1782` и обеих веток Rust-ядра в [`simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs#L2752)), что на сборках из $\ge 3$ деталей могло учитывать зазор на сталкивающемся шаге.

---

## 2. Выполненные изменения

1. **[`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs#L1632-L1659)**:
   - В `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall` сохранена исходная геометрия стены и добавлено геометрическое обоснование: проверяется зазор на шаге `27` (`~0.146 мм`), факт более малого глобального минимума зазора на выходе на шаге `78` (`min_clearance_step == Some(78)`, `min_clearance < samples[27].min_clearance`) и `safe_travel_range == 28°`.
   - Добавлена дополнительная проверка поддиапазона `[0°, 60°]` (`steps = 60`, $1^\circ$/шаг), где рычаг остаётся в столкновении до конца сканирования и `min_clearance_step == Some(27)`.
2. **[`src/cad/kinematics.ts`](../../src/cad/kinematics.ts#L1637)**:
   - В открытой ветке `sweepJoint` добавлена проверка `&& interferingPairs.length === 0` при обновлении `minClearance` / `minClearanceStep` / `minClearanceValue` для полного паритета с замкнутым контуром TS и Rust-реализацией.
3. **[`src/cad/__tests__/analyze_mechanism.test.ts`](../../src/cad/__tests__/analyze_mechanism.test.ts#L302-L304)**:
   - В тесте `3. sweep_joint on hinged arm hitting fixed wall` добавлены проверки `min_clearance > 0`, наличия `min_clearance_step` и отсутствия пересечений на этом шаге (`interfering_pairs.length === 0`).

---

## 3. Результаты верификации (после финальной правки кода)

| Проверка / Команда | Итог | Детали |
|---|---|---|
| `cargo test -p mcp-cad-physics --no-fail-fast` | **23 passed, 0 failed** (`9.13s`) | `tests/physics_tests.rs`: `23/23 PASS` (включая `test_p3_t10_sweep_joint_hinged_arm_hitting_fixed_wall`); `src/lib.rs`: `0 tests`; doc-tests: `0 tests` |
| `$env:MCP_CAD_DISABLE_GUI_SPAWN="1"; cargo test -p mcp-cad-server --lib --no-fail-fast -- --test-threads=1` | **61 passed, 0 failed** (`73.91s`) | Все unit-тесты сервера пройдены без ошибок |
| `npm run lint` (`tsc --noEmit`) | **PASS** (`exit 0`) | `0` ошибок TypeScript |
| `npx tsx --test src/cad/__tests__/analyze_mechanism.test.ts` | **15 passed, 0 failed** (`7846.40ms`) | Все 2 сьюта (`P3-T9`, `P3-T10/T16`) и 15 тестов пройдены |
| `cargo clippy --workspace --all-targets -- -D warnings` | **PASS** (`exit 0`, `0.83s`) | `0` ошибок, `0` предупреждений во всех 5 крейтах workspace |
