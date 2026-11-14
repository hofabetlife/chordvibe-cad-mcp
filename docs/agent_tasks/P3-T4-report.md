# Отчёт о выполнении задачи P3-T4 — Квадратичный конечный элемент `TET10` (10-узловой тетраэдр)

## 1. Summary
Реализован квадратичный 10-узловой изопараметрический тетраэдр `Tet10Element` (`crates/mcp-cad-fea/src/element.rs`), конвертер тетраэдральной сетки `TetrahedralMesh::to_tet10()` без дублирования mid-edge узлов на общих рёбрах (`crates/mcp-cad-fea/src/tetra_mesh.rs`), сборка глобальной разреженной матрицы жёсткости `30×30` и вычисление реакций опор (`crates/mcp-cad-fea/src/solver.rs`), согласованные квадратичные веса поверхностных сил Неймана (`crates/mcp-cad-fea/src/boundary.rs`), экстраполяция напряжений из 4 точек Гаусса в 10 узлов элемента (`crates/mcp-cad-fea/src/result.rs`) и параметр `element_order: 1 | 2` (по умолчанию `1`) в инструменте MCP `run_fea_analysis` (`crates/mcp-cad-server/src/tools/fea.rs`, `tools/mod.rs`).

На эталонной задаче поперечного изгиба консольной балки (`200 × 20 × 20 мм`, $F_y = -1000\text{ Н}$, сталь $E = 205\text{ ГПа}, \nu = 0.30$) переход с `TET4` (`element_order = 1`) на `TET10` (`element_order = 2`) снижает ошибку прогиба:
- на сетке **`Coarse`** (`480` элементов): с **`48.15%`** до **`0.06%`** (`0.83%` отн. балки Тимошенко), ошибка пикового напряжения снижается с **`46.18%`** до **`0.32%`**;
- на сетке **`Medium`** (`3 840` элементов): с **`20.14%`** до **`0.25%`** (`0.52%` отн. Тимошенко, при целевом пороге `<= 5.0%`), а ошибка изгибного напряжения вне зоны заделки ($\sigma(0.5h)$) снижается с **`28.21%`** до **`0.09%`** (`142.62 МПа` vs `142.50 МПа`);
- на сетке **`Fine`** (`12 960` элементов): с **`10.24%`** до **`0.32%`** (`0.46%` отн. Тимошенко), ошибка $\sigma(0.5h)$ — **`0.37%`**.

## 2. Mathematical Formulation & Architecture
1. **Нумерация узлов и функции формы `Tet10Element` (`crates/mcp-cad-fea/src/element.rs`)**:
   - Узлы `0..4` — вершины тетраэдра с барицентрическими координатами $(L_0, L_1, L_2, L_3)$, $\sum_{i=0}^3 L_i = 1$:
     $$N_i(\mathbf{L}) = L_i (2L_i - 1), \quad i \in \{0, 1, 2, 3\}.$$
   - Узлы `4..10` — середины 6 рёбер в фиксированном порядке `TET10_EDGE_PAIRS = [(0,1), (1,2), (2,0), (0,3), (1,3), (2,3)]`:
     $$N_{4+e}(\mathbf{L}) = 4 L_a L_b, \quad (a, b) = \text{TET10\_EDGE\_PAIRS}[e].$$
   - Декартовы градиенты функций формы вычисляются по правилу цепи через постоянные градиенты барицентрических координат линейного остова $\nabla_{\mathbf{x}} L_m$:
     $$\nabla_{\mathbf{x}} N_k(\mathbf{L}) = \sum_{m=0}^3 \frac{\partial N_k}{\partial L_m} \nabla_{\mathbf{x}} L_m.$$
2. **4-точечная квадратура Гаусса 2-го порядка (`degree 2`)**:
   - Для точного интегрирования квадратичных полиномов подынтегрального выражения $B(\mathbf{L})^T D B(\mathbf{L})$ при постоянном якобиане прямолинейного тетраэдра ($\det J = 6V$) используется симметричное 4-точечное правило с положительными весами $w_q = V/4$ (сумма весов на единичном тетраэдре $\sum w_q^{\text{ref}} = 4 \times \frac{1}{24} = \frac{1}{6}$) и узлами:
     $$a = \frac{5 + 3\sqrt{5}}{20} \approx 0.5854101966, \qquad b = \frac{5 - \sqrt{5}}{20} \approx 0.1381966011.$$
     $$K_e = \sum_{q=0}^3 \frac{V}{4} B(\mathbf{L}_q)^T D B(\mathbf{L}_q) \in \mathbb{R}^{30 \times 30}.$$
3. **Экстраполяция напряжений из точек Гаусса в 10 узлов**:
   - Линейное поле напряжений внутри `TET10` однозначно восстанавливается по значениям в 4 точках Гаусса $\boldsymbol{\sigma}(\mathbf{L}_q)$. Так как $L_i(\mathbf{L}_q) = b + (a-b)\delta_{iq}$ и $a - b = \frac{1}{\sqrt{5}} = \frac{\sqrt{5}}{5}$, барицентрическая экстраполяция в любую точку $\mathbf{L} = (L_0, L_1, L_2, L_3)$ имеет замкнутый вид:
     $$\boldsymbol{\sigma}(\mathbf{L}) = \sum_{q=0}^3 \left[ \frac{1 - \sqrt{5}}{4} + \sqrt{5} L_q \right] \boldsymbol{\sigma}(\mathbf{L}_q),$$
     причём сумма коэффициентов тождественно равна $1$ для любых $\mathbf{L}$ ($\sum L_q = 1$).
4. **Конвертация `TET4 -> TET10` и согласованные граничные нагрузки**:
   - `TetrahedralMesh::to_tet10()` обходит все рёбра исходной сетки `TET4`, канонизирует пару индексов `(min(a,b), max(a,b))` в `HashMap<(usize, usize), usize>` и создаёт ровно один mid-edge узел $\mathbf{x}_{\text{mid}} = \frac{1}{2}(\mathbf{x}_a + \mathbf{x}_b)$ на каждое общее ребро.
   - В `boundary.rs` для 6-узловых квадратичных граней на поверхности (`mesh.is_tet10() == true`) реализовано согласованное распределение поверхностных нагрузок ($\int_\Delta N_{\text{corner}}\,dA = 0$, $\int_\Delta N_{\text{mid}}\,dA = \frac{1}{3}A_\Delta$), благодаря чему однородное растягивающее усилие на торце воспроизводит точное постоянное поле напряжений во всех элементах (`Patch Test`).

## 3. Changes Made
- `crates/mcp-cad-fea/src/element.rs`: реализованы `Tet10Element`, `Matrix6x30`, `Matrix30`, `Vector30`, функции формы, матрица $B(\mathbf{L})$, 4-точечная квадратура $K_e$, расчёт деформаций/напряжений в точках Гаусса и экстраполяция в 10 узлов, а также 3 юнит-теста.
- `crates/mcp-cad-fea/src/tetra_mesh.rs`: добавлены `ElementOrder`, поле `tet10_elements: Vec<[usize; 10]>`, методы `is_tet10()`, `element_order()`, `to_tet10()`, `extract_surface_tet10_faces()` и 4-треугольное разбиение внешних 6-узловых граней в `extract_surface_triangles()`.
- `crates/mcp-cad-fea/src/boundary.rs`: добавлено согласованное распределение узловых весов поверхностных нагрузок для 6-узловых квадратичных граней `TET10` в `AabbFaceForce` и `match_surface_nodes_and_weights`.
- `crates/mcp-cad-fea/src/solver.rs`: добавлены `CsrMatrix::from_row_entries`, `assemble_and_solve_tet10` и `compute_nodal_reactions_tet10`.
- `crates/mcp-cad-fea/src/result.rs`: добавлены поле `element_order: u8` в `FeaReport`, функции `compute_tet10_element_quality`, `compute_fea_results_tet10` и `compute_fea_results_with_boundary_tet10`.
- `crates/mcp-cad-fea/src/convergence.rs` и `crates/mcp-cad-fea/src/lib.rs`: добавлены `run_fea_with_convergence_with_order`, `analyze_mesh_with_order`, `analyze_part_with_order` и автоматическая диспетчеризация `TET10` в `analyze_mesh`.
- `crates/mcp-cad-server/src/tools/fea.rs` и `crates/mcp-cad-server/src/tools/mod.rs`: добавлен параметр `element_order: 1 | 2` (по умолчанию `1`) и поле `"element_order"` в JSON-ответе `run_fea_analysis`.
- `crates/mcp-cad-fea/tests/fea_tests.rs` и `crates/mcp-cad-fea/tests/fea_benchmarks.rs`: добавлены интеграционный тест `test_p3_t4_tet10_mesh_conversion_eigenvalues_and_multi_element_patch_test` и бенчмарк `test_p3_t4_tet10_cantilever_benchmark`.
- Документация: обновлены `docs/FEA_ACCURACY.md`, `docs/MCP_API_REFERENCE.md`, `TASKS.md` и `PROJECT_CONTEXT.md`.

## 4. Verification
- `cargo test -p mcp-cad-fea -- --nocapture` — **21 тест, 21 проходит (0 падений)**:
  - Собственные значения матрицы жёсткости $K_e \in \mathbb{R}^{30 \times 30}$ (`nalgebra::SymmetricEigen`): ровно **6 нулевых собственных значений** ($|\lambda_i| < 10^{-4}$, 3 поступательных + 3 вращательных движения жёсткого тела) и **24 строго положительных** ($\lambda_i > 10^5$).
  - **Patch Test**: на одиночном искажённом элементе `TET10` и на 3D многоэлементной сетке `TET10` (как при кинематическом задании линейного поля перемещений на границе, так и при однородной силе Неймана на торце) деформация и напряжение фон Мизеса совпадают с аналитическими во всех точках Гаусса и во всех узлах с точностью до $10^{-9}$.
  - **Cantilever Benchmark (`test_p3_t4_tet10_cantilever_benchmark`)**: ошибка прогиба на `Medium` равна **`0.25%`** ($\le 5.0\%$), на `Coarse` — **`0.06%`**, на `Fine` — **`0.32%`**.
- `cargo test -p mcp-cad-server --lib tools::fea` — все тесты инструмента `run_fea_analysis` (включая `test_p3_t4_run_fea_analysis_tet10_element_order`) проходят.
- `cargo clippy -p mcp-cad-fea -p mcp-cad-server --all-targets -- -D warnings` — 0 ошибок, 0 предупреждений.
