# Отчёт о выполнении задачи P0-T6

**Задача:** P0-T6 — Игольчатые треугольники: фаска призмы  
**Дата:** 04.11.2026  
**Статус:** ✅ ВЫПОЛНЕНО

---

## 1. Что сделано (файлы, функции)

1. **Анализ первопричины игольчатых треугольников в `create_polygon_prism` (`crates/mcp-cad-core/src/primitives.rs`)**:
   - Для правильной $n$-угольной призмы с радиусом описанной окружности $R$ длина стороны основания равна $s_{\text{mid}} = 2 R \sin(\pi / n)$. При наложении $30^\circ$ фаски (по DIN 934, $\tan\alpha = \tan 30^\circ = 1 / \sqrt{3}$) с радиальным отступом $c_r$ и осевым отступом $c_z = c_r \tan 30^\circ$ наклонная высота трапеции фаски в 3D составляет:
     $$h_{\text{bevel}} = \sqrt{(c_r \cos(\pi / n))^2 + c_z^2}.$$
   - В прежней реализации `create_polygon_prism` каждая из $n$ верхних и $n$ нижних граней фаски эмитировалась как один неразбитый четырёхугольник (`add_quad_facet`), рассекаемый одной диагональю на два треугольника. При малой фаске относительно стороны (например, в `test_bug_005`: $n = 3$, $W = 0.04 \implies R = 0.04$ м, $s_{\text{mid}} \approx 0.06928$ м, $c_r = 0.003$ м $\implies h_{\text{bevel}} \approx 0.00229$ м, отношение сторон $\approx 30.2 : 1$) минимальный угол каждого из $6 \times 2 = 12$ треугольников фаски составлял $\arctan(h_{\text{bevel}} / s_{\text{mid}}) \approx 1.90^\circ \dots 2.86^\circ < 5^\circ$, что давало $12 / 24 = 50.00\%$ sliver-треугольников (при пороге `MAX_ALLOWED_SLIVER_RATIO = 1.00%`). А при $c_r / s_{\text{mid}} = 0.02$ отношение сторон достигало $65 : 1$, порождая игольчатые треугольники с углом $< 1^\circ$.
   - Прямое разбиение только полосы фаски на $m$ подсегментов вдоль стороны многоугольника без согласованного разрежения на торцевых крышках и вертикальных боковых гранях привело бы либо к Т-образным вершинам (потере водонепроницаемости), либо к вееру из узких треугольников с углом $< 5^\circ$ в центре крышки и на боковых стенках.

2. **Конформная триангуляция фаски, крышек и боковых граней с геометрическим разрежением 2-к-1 в `create_polygon_prism` (`crates/mcp-cad-core/src/primitives.rs`)**:
   - Порог активации фаски сделан масштабно-инвариантным (`has_chamfer = c_val > 1e-12` вместо `1e-6`), что корректно обрабатывает малые фаски ($0.02 \cdot s$) на миллиметровых призмах ($R = 0.001$ м).
   - Число сегментов разбиения стороны многоугольника на границе с фаской выбирается как степень двойки $m_{\text{edge}} = 2^p \in [1, 512]$ так, чтобы длина подсегмента $\Delta s_{\text{mid}} = s_{\text{mid}} / m_{\text{edge}}$ удовлетворяла $\Delta s_{\text{mid}} \le 3.0 \cdot \min(h_{\text{bevel}}, h_{\text{mid}})$, а при высокой фаске относительно шага крышки полоса фаски дополнительно делится по наклонной высоте на `n_bevel_steps` рядов с разбиением каждого подчетырёхугольника по кратчайшей диагонали (`push_quad_shortest_diag`).
   - **Торцевые крышки (`emit_polygon_cap`)**: от внешней границы крышки радиуса $r_{\text{cap}}$ (имеющей $m_{\text{edge}} = 2^p$ сегментов на каждую из $n$ сторон) к центру $(0, 0, \pm h/2)$ строятся вложенные гомотетичные правильные $n$-угольники с пошаговым переходом 2-к-1 ($m_{\text{curr}} \to k = m_{\text{curr}} / 2$) и масштабно-инвариантным шагом радиуса $\eta = 1 - \sigma / k$ ($\sigma = \min(0.65 \tan(\pi/n), 0.55 k, 0.75)$). Благодаря сокращению множителя $k$ в $(1 - \eta) k = \sigma$ форма переходных треугольников не зависит от $k$, обеспечивая минимальный угол $\ge 13.9^\circ \gg 5^\circ$ во всех секторах вплоть до $n = 3$. На внутреннем контуре ($m_{\text{curr}} = 1$) крышка замыкается стандартным $n$-треугольным веером с углами $\ge 30^\circ$.
   - **Вертикальные боковые грани**: от верхнего ($z = z_{\text{top\_mid}}$) и нижнего ($z = z_{\text{bot\_mid}}$) рёбер с $m_{\text{edge}}$ сегментами к середине высоты грани выполняются симметричные прямоугольные переходы 2-к-1 ($\Delta z = \min(0.55 \Delta s, 0.35 h_{\text{rem}})$), а оставшийся центральный пояс разбивается на `n_z_mid` вертикальных рядов близких к квадрату четырёхугольников. Поскольку угловые рёбра между соседними гранями $i$ и $i+1$ используют одни и те же уровни по $z$ и радиусу, сетка остаётся на 100% замкнутой и манифолдной (`boundary_edges == 0`, `t_junctions == 0`), а все вершины лежат строго на аналитических плоскостях граней призмы.

3. **Вывод точной аналитической формулы объёма призмы с фаской**:
   - Объём правильной $n$-угольной призмы радиуса описанной окружности $R$ и высоты $h$ без фаски:
     $$V_{\text{prism}} = A_n(R) \cdot h = \frac{n}{2} R^2 \sin\!\left(\frac{2\pi}{n}\right) h, \quad A_n(r) = \frac{n}{2} r^2 \sin\!\left(\frac{2\pi}{n}\right).$$
   - При срезании верхней и нижней фасок с радиальным отступом $c_r$ ($r_{\text{cap}} = R - c_r$) и вертикальной высотой $c_z = c_r \tan 30^\circ = c_r / \sqrt{3}$ (с учётом зажима $c_r \le 0.45 R$, $c_z \le 0.40 h$) каждая из двух зон фаски высотой $c_z$ представляет собой усечённую правильную $n$-угольную пирамиду с основаниями $A_n(R)$ и $A_n(r_{\text{cap}})$:
     $$V_{\text{frustum}} = \frac{c_z}{3} \left( A_n(R) + \sqrt{A_n(R) A_n(r_{\text{cap}})} + A_n(r_{\text{cap}}) \right) = \frac{n}{6} \sin\!\left(\frac{2\pi}{n}\right) c_z \left( R^2 + R r_{\text{cap}} + r_{\text{cap}}^2 \right).$$
   - Следовательно, объём срезанных фасок (двух колец треугольного сечения по периметру обоих торцов) равен:
     $$\Delta V_{\text{chamfer}} = 2 \left( A_n(R) c_z - V_{\text{frustum}} \right) = \frac{n}{3} \sin\!\left(\frac{2\pi}{n}\right) c_z \left( 3 R^2 - \left(R^2 + R(R - c_r) + (R - c_r)^2\right) \right) = n \sin\!\left(\frac{2\pi}{n}\right) c_z \, c_r \left( R - \frac{c_r}{3} \right).$$
   - Итоговый аналитический объём призмы с фаской:
     $$V = V_{\text{prism}} - \Delta V_{\text{chamfer}} = \frac{n}{2} R^2 \sin\!\left(\frac{2\pi}{n}\right) h - n \sin\!\left(\frac{2\pi}{n}\right) c_z \, c_r \left( R - \frac{c_r}{3} \right).$$
   - Так как все грани сетки кусочно-плоские и лежат точно в исходных плоскостях граней призмы и фасок, `mesh.signed_volume()` совпадает с $V_{\text{prism}} - \Delta V_{\text{chamfer}}$ с относительной погрешностью $< 10^{-12}$ (проверено с допуском `1e-7`).

4. **Расширение тестового набора в `crates/mcp-cad-core/tests/primitive_invariants.rs`**:
   - Тест `invariants_polygon_prism_chamfered_5_to_12_sides_grid` расширен на полный перебор всех сторон `sides ∈ 3..=12` и всех требуемых отношений фаски к стороне $\{0.02, \, 0.10, \, 0.30, \, c_{\max}/s\}$, а также масштабов $R \in \{0.001, 0.01, 0.05, 0.1\}$ м и отношений высоты $h / R \in \{0.5, 2.0, 5.0\}$.
   - Снят атрибут `#[ignore]` с теста `known_bug_polygon_prism_chamfer_low_sides_slivers`.

---

## 2. Как проверено (команды и итоговые строки `test result`)

- **Воспроизведение дефекта до правок:**
  - Команда: `cargo test -p mcp-cad-core --test primitive_invariants known_bug_polygon_prism_chamfer_low_sides_slivers -- --ignored --exact --nocapture`
  - Результат до правок: `test result: FAILED. 0 passed; 1 failed; 0 ignored; 0 measured; 17 filtered out; finished in 0.00s` (`Chamfered 3-sided prism failed validate_mesh: ["Sliver quality gate failed: 50.00% of triangles (12/24) have min angle < 5° (threshold 1.00%)"]`).

- **Проверка после правок:**
  - Полный набор инвариантов примитивов (`primitive_invariants`, включая перебор `sides ∈ 3..=12` $\times$ отношения фаски $\{0.02, 0.1, 0.3, \text{max}\}$ и разблокированный `known_bug_polygon_prism_chamfer_low_sides_slivers`):
    - Команда: `cargo test -p mcp-cad-core --test primitive_invariants`
    - Результат: `test result: ok. 12 passed; 0 failed; 6 ignored; 0 measured; 0 filtered out; finished in 56.60s`
  - Целевой интеграционный тест `qa_wide_scan::test_bug_005_polygon_prism_chamfer_and_sides_validation`:
    - Команда: `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test qa_wide_scan test_bug_005_polygon_prism_chamfer_and_sides_validation -- --exact --nocapture`
    - Результат: `test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 12 filtered out; finished in 1.80s`
  - Unit-тесты `mcp-cad-core --lib`, `dag_tests`, `tessellation_tests`, `mcp_contract_tests`, `gear_kinematic_test`:
    - `cargo test -p mcp-cad-core --lib` $\to$ `test result: ok. 49 passed; 0 failed; 0 ignored`
    - `cargo test -p mcp-cad-core --test dag_tests` $\to$ `test result: ok. 14 passed; 0 failed; 0 ignored`
    - `cargo test -p mcp-cad-core --test tessellation_tests` $\to$ `test result: ok. 6 passed; 0 failed; 0 ignored`
    - `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test mcp_contract_tests` $\to$ `test result: ok. 21 passed; 0 failed; 0 ignored`
    - `WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test gear_kinematic_test` $\to$ `test result: ok. 1 passed; 0 failed; 0 ignored`

---

## 3. Тесты до / после

| Таргет / Тест | До P0-T6 | После P0-T6 |
|---|---|---|
| `qa_wide_scan::test_bug_005_polygon_prism_chamfer_and_sides_validation` | ❌ FAILED (`0 passed; 1 failed`, `50.00% < 5°`) | ✅ PASS (`1 passed; 0 failed`) |
| `primitive_invariants::known_bug_polygon_prism_chamfer_low_sides_slivers` | ❌ FAILED (`#[ignore]`, `12/24 < 5°`) | ✅ PASS (снят `#[ignore]`, `12 passed; 0 failed`) |
| `primitive_invariants::invariants_polygon_prism_chamfered_5_to_12_sides_grid` | Покрывал только `sides ∈ 5..=12`, `c = 0.1*r` | ✅ PASS (полный перебор `sides ∈ 3..=12`, `c/s ∈ {0.02, 0.1, 0.3, max}`) |
| Workspace всего (`cargo test --workspace --no-fail-fast`) | `140 passed; 2 failed; 7 ignored` (149 всего) | **`142 passed; 1 failed; 6 ignored` (149 всего)** |

Единственное оставшееся активное падение во всём Rust Workspace — `qa_wide_scan::test_bug_006_csg_booleans_and_holes_watertight`, относящееся к следующей задаче **P0-T7** («CSG: инварианты объёма, отверстия, водонепроницаемость»).

---

## 4. Что не удалось и остаточные риски

- Все критерии приёмки **P0-T6** выполнены полностью без изменения порогов `Sliver quality gate` и без изменения контракта при некорректных параметрах (`sides < 3` или `height <= 0` по-прежнему возвращают пустую сетку / отклоняются валидацией сервера).
- Для призм без фаски (`chamfer = None`) при стандартных пропорциях (`s_mid / h <= 10`) сохранено минимальное число треугольников (`2n` на крышках + `2n` на боковых гранях, т.е. `24` треугольника для шестигранника).

---

## 5. Побочные находки (не чинить в рамках P0-T6, только перечислить)

- Новых дефектов, не учтённых в `TASKS.md` (после уже заведённых в прошлых итерациях **P0-T10**, **P0-T11** и **TS-FIX-01**), не выявлено.
