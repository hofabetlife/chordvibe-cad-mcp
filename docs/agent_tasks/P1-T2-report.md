# Отчёт о выполнении задачи P1-T2

## 1. Задача
- **ID**: `P1-T2`
- **Название**: Бенчмарк 15 задач против Rust-сервера
- **Фаза**: P1 — Архитектурное выравнивание (Two Engines Problem)
- **Ветка**: `agent/P1-T2`

## 2. Что было сделано
Проведена честная проверка эталонного набора из 15 задач (`bench/tasks.json`, `T01`…`T15`) против **настоящего нативного Rust-сервера** `mcp-cad-server` (ранее `bench/run_benchmark.ts` запускал только TypeScript-мок `src/cad/mcp_dispatcher.ts` в браузере/Node.js и заявлял `15 / 15 PASSED (100%)` в `bench/README.md`).

Созданы и проверены два независимых раннера, прогоняющих все 15 задач `bench/tasks.json` через реальный Rust-сервер `mcp-cad-server` без подгонки («озеленения») входных данных и ожиданий в `bench/tasks.json`:

1. **`crates/mcp-cad-server/tests/bench_suite.rs`** — интеграционный Rust-тест (`cargo test -p mcp-cad-server --test bench_suite -- --nocapture`), прогоняющий все 15 задач из `bench/tasks.json` через JSON-RPC 2.0 диспетчер `mcp_cad_server::stdio::handle_message` в трёх диагностических режимах:
   - **Mode 1 (Strict As-Is)**: шаги из `bench/tasks.json` отправляются в `mcp-cad-server` буквально без изменений $\to$ **`0 / 15 PASSED (0.0%)`**, так как все 15 задач начинают с `add_feature_primitive` без предварительного вызова `create_part` (TS-мок авто-создавал детали неявно в `ensurePart()`, а Rust-сервер возвращает `Part not found: <part_id>`).
   - **Mode 2 (With `create_part` Pre-Initialization, Verbatim Steps)**: перед шагами задачи вызывается `create_part` с маппингом пресетов материалов (`Al6061-T6` $\to$ `Al-6061-T6`, `Ti-6Al-4V` $\to$ `Ti6Al4V`), а сами `task.steps` отправляются без изменений $\to$ **`2 / 15 PASSED (13.3%)`** (`T01_steel_block`, `T03_titanium_sphere`).
   - **Mode 3 (Schema-Bridged Diagnostic Probe)**: поверх Mode 2 адаптируются только расхождения контрактов/схем аргументов (`through: true` $\to$ явный `depth` со смещением точки старта на верхнюю грань в `cut_hole`, `inner_radius` $\to$ 4-точечный замкнутый контур в `revolve`, `outer_radius`/`chamfer_radius` $\to$ `radius`/`chamfer` в `polygon_prism`, проброс сгенерированного сервером `joint-<uuid>` из `create_joint` в `set_joint_motor`), при сохранении всех чисел и единиц измерения из `bench/tasks.json` $\to$ **`6 / 15 PASSED (40.0%)`** (`T01`, `T03`, `T05`, `T06`, `T07`, `T10`).
2. **`bench/run_benchmark_rust.ts`** — TypeScript CLI-раннер (`npx tsx bench/run_benchmark_rust.ts`), поднимающий скомпилированный бинарник `mcp-cad-server --headless` и общающийся с ним по `stdin`/`stdout` (JSON-RPC 2.0), выводящий сводные таблицы по всем трём режимам.
3. **`docs/BENCH_BASELINE_RUST.md`** — подробный baseline-отчёт с таблицами `T01`…`T15` (статус `PASS/FAIL`, число успешных вызовов, `watertight`, погрешность габаритов, отклонение по доменным метрикам FEA/кинематики относительно аналитики) и детальным сопоставлением с заявлениями в `bench/README.md`.

### Ключевые зафиксированные расхождения (вход для `P2-T1` и `P3-*`)
- **Оси цилиндра и конуса (`T02`, `T04`)**: вопреки утверждению в `bench/README.md` (*«Both TS and Rust engines generate cylinders and cones along the Y-axis»*), Rust-ядро (`crates/mcp-cad-core/src/primitives.rs`) строит цилиндры и конусы вдоль оси **Z** (`[2R, 2R, H]`), а TS-мок — вдоль оси **Y** (`[2R, H, 2R]`). Из-за этого `T02` падает с `dim_err = 150.0%`, а `T04` — с `dim_err = 28.57%`.
- **Несоответствие единиц измерения `module` у шестерён (`T09`, `T14`)**: в `bench/tasks.json` передаётся `"module": 2.0` (в миллиметрах!), тогда как все остальные параметры и `expected_dimensions_m: [0.044, 0.044, 0.025]` заданы в метрах ($d_a = m(z+2) = 0.002 \times 22 = 0.044\text{ м}$). В TS-моке (`src/cad/primitives.ts:392`) стоял скрытый костыль `rawModule > 0.1 ? rawModule * 0.001 : rawModule`, а Rust-сервер трактует `module = 2.0` как $2.0\text{ м}$ и строит шестерню диаметром $44\text{ м}$ (`dim_err = 99900%`).
- **Неводонепроницаемая резьба в `cut_helical_groove` (`T08`)**: даже после маппинга `outer_radius` $\to$ `radius` вычитание винтовой канавки в `crates/mcp-cad-core/src/dag.rs:676` (`csg_difference`) оставляет `413` открытых граничных рёбер (`is_watertight = false`, `genus = 68` вместо `1`).
- **Игнорирование `joint_id` в `create_joint` (`T14`, `T15`)**: `crates/mcp-cad-server/src/tools/physics.rs:264` игнорирует переданный аргумент `joint_id` (`"t14_rev_1"`, `"t15_joint_crank"`) и всегда генерирует случайный `joint-<uuid>`, из-за чего `set_joint_motor` падает с `Joint with id '...' was not found in physics world`.
- **Отсутствие `TET10` и сдвиговое запирание `TET4` в FEA (`T11`, `T12`, `T13`)**:
  - `T11`: 3D-заделка `min_x` ($u_x=u_y=u_z=0$) даёт локальную концентрацию напряжений из-за коэффициента Пуассона ($\sigma = 21.30\text{ МПа}$ против 1D $20.50\text{ МПа}$, отклонение `3.93% > 1.0%`; $\delta = 0.1020\text{ мм}$ против $0.1000\text{ мм}$, отклонение `2.00% > 1.0%`), а поле вердикта называется `status: "PASS"` вместо `verdict: "SAFE"`.
  - `T12`: `run_fea_analysis` игнорирует `element_type: "TET10"` и считает консольную балку на линейных `TET4`, получая сдвиговое запирание (`sigma_err = 21.65%`, `disp_err = 19.73%` при допуске `2.0%`).
  - `T13`: воксельная сетка (`max_cells = 60` вдоль $0.60\text{ м}$, шаг $10\text{ мм}$ при радиусе отверстия $20\text{ мм}$) аппроксимирует круглое отверстие Кирша ступеньками и занижает пиковое напряжение ($\sigma = 128.06\text{ МПа}$ против аналитических $157.44\text{ МПа}$, `sigma_err = 18.66% > 10.0%`).
- **Физика и кинематика (`T14`, `T15`)**:
  - В `T14` обе шестерни создаются в `[0, 0, 0]`, имеют диаметр $40\text{ м}$ и $80\text{ м}$ из-за `module: 2.0`, а в Rapier3D нет кинематической связи передаточного отношения между шарнирами (`omega1 = 0`, `omega2 = 0`).
  - В `T15` шатун `t15_connecting_rod` в `bench/tasks.json` вообще не привязан шарнирами ни к кривошипу, ни к ползуну (в TS-моке движение ползуна вычислялось аналитической формулой по совпадению подстроки `"slider"` в `part_id`).

## 3. Какие проверки пройдены
- `cargo test -p mcp-cad-server --test bench_suite -- --nocapture` — `1 passed` (выводит полные таблицы по всем 15 задачам во всех 3 режимах).
- `npx tsx bench/run_benchmark_rust.ts` — успешно прогоняет все 15 задач через `mcp-cad-server --headless` по stdio JSON-RPC 2.0 и печатает идентичные таблицы (`0/15`, `2/15`, `6/15`).
- `cargo clippy --workspace --all-targets -- -D warnings` — `0` предупреждений и ошибок.

## 4. Какие артефакты и документы обновлены
- Создан `crates/mcp-cad-server/tests/bench_suite.rs`
- Создан `bench/run_benchmark_rust.ts`
- Создан `docs/BENCH_BASELINE_RUST.md`
- Создан `docs/agent_tasks/P1-T2-report.md`
- Обновлён `TASKS.md` (статус `P1-T2` переведён в `✅ DONE`, добавлены обнаруженные смежные баги в `BACKLOG`)

## 5. Открытые вопросы и обнаруженные смежные баги
В ходе выполнения `P1-T2` обнаружены следующие конкретные баги и расхождения контрактов, занесённые в `TASKS.md` (и задокументированные в `docs/BENCH_BASELINE_RUST.md` как вход для фаз `P2` и `P3`):
1. **`create_joint` игнорирует пользовательский `joint_id` (`crates/mcp-cad-server/src/tools/physics.rs:264`)**: всегда генерирует `joint-<uuid>`, из-за чего `set_joint_motor` в `T14` и `T15` не находит шарнир по имени.
2. **Неводонепроницаемый результат `cut_helical_groove` (`crates/mcp-cad-core/src/dag.rs:676`, `primitives.rs:1499`)**: на задаче `T08_hex_nut_m10` BSP-вычитание винтовой нарезки даёт `is_watertight = false` (`413` открытых рёбер, `genus = 68`).
3. **Несогласованность контрактов инструментов между `bench/tasks.json` (TS) и Rust `mcp-cad-server` (вход для `P2-T1`)**:
   - отсутствие `create_part` в начале задач `T01`–`T15`;
   - ориентация `cylinder` и `cone` вдоль оси Z в Rust против оси Y в TS (`T02`, `T04`);
   - единицы измерения `module: 2.0` (мм в `bench/tasks.json` против метров в Rust `spur_gear`, `T09`, `T14`);
   - отсутствие поддержки `through: true` (без `depth`) в `cut_hole` (`T05`, `T06`, `T10`, `T13`);
   - отсутствие поддержки `inner_radius` с 2-точечным профилем в `revolve` (`T07`) и алиасов `outer_radius`/`chamfer_radius` в `polygon_prism` (`T08`);
   - расхождение имён пресетов материалов (`Al6061-T6` vs `Al-6061-T6`, `Ti-6Al-4V` vs `Ti6Al4V`) и поля вердикта FEA (`status: "PASS"` vs `verdict: "SAFE"`).
