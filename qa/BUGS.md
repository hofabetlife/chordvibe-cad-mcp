# MCP-CAD Engine: Registry of Bugs & Discrepancies

| ID | Sev | Title | Component | Status | Discovered |
|---|---|---|---|---|---|
| BUG-001 | S4 | ARCHITECTURE.md indicates 15 tools instead of 26 | docs | OPEN | 2026-10-30 |
| BUG-002 | S2 | spur_gear: non-watertight holes and degenerate triangles with hub/bore/keyway at 60 teeth | core/primitives | OPEN | 2026-10-30 |
| BUG-003 | S2 | revolve: partial revolution (<360 deg) end-caps have inconsistent triangle winding | core/primitives | OPEN | 2026-10-30 |
| BUG-004 | S2 | wheel_rim: non-watertight composite mesh with boundary holes and self-intersections | core/primitives | OPEN | 2026-10-30 |

---

## BUG-001 [S4][OPEN] ARCHITECTURE.md: устаревшее упоминание 15 инструментов вместо 26
- Воспроизведение: Прочитать `docs/ARCHITECTURE.md` строка 33 (`MCP_API_REFERENCE.md # Полный справочник 15 инструментов MCP`).
- Ожидание: 26 инструментов MCP согласно PRD, коду и MCP_API_REFERENCE.
- Факт: Написано 15 инструментов.
- Артефакты: `docs/ARCHITECTURE.md`
- Гипотеза причины: Документ не был обновлен при добавлении инструментов групп 6-7.
- Тест-регрессия: Проверка консистентности документации.
- Статус: OPEN

---

## BUG-002 [S2][OPEN] spur_gear: негерметичность и дегенеративные треугольники при наличии ступицы/паза (60 зубьев)
- Воспроизведение: `create_spur_gear(0.002, 60, 0.015, 20.0, Some(0.006), Some(0.003), Some(0.002), Some(0.025), Some(0.010))`
- Ожидание: Watertight = true, boundary edges = 0, genus = 1.
- Факт: `is_watertight: false`, 7304 граничных ребер (дыры), 60 вырожденных треугольников, 142 самопересечения, genus = 45.0.
- Артефакты: `qa/results/primitives_mesh_scan.json`
- Гипотеза причины: Булево слияние зубьев со ступицей или наложение цилиндров ступицы и торцевых фасеток венца создает разрывы сетки и несостыковку вершин.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-003 [S2][OPEN] revolve: несогласованная ориентация треугольников на торцевых крышках частичного вращения
- Воспроизведение: `create_revolve(&[[0.01, -0.02], [0.03, -0.02], [0.03, 0.02], [0.01, 0.02]], 180.0, 32)`
- Ожидание: Watertight = true, `orientation_consistent = true`, 0 inverted edges.
- Факт: `orientation_consistent = false`, обнаружено 8 инвертированных ребер на торцевых крышках среза 180°.
- Артефакты: `qa/results/primitives_mesh_scan.json`
- Гипотеза причины: Торцевая крышка на угле $\theta_0$ или $\theta_{end}$ строится без учета нормали вращения профиля, из-за чего намотка вершин (winding order) инвертирована относительно смежных боковых граней.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN

---

## BUG-004 [S2][OPEN] wheel_rim: негерметичный составной меш обода и спиц (дыры и самопересечения)
- Воспроизведение: `create_wheel_rim(0.4572, 0.2032, Some(0.490), Some(0.160), Some(0.066), Some(0.1143), Some(5), Some(0.012), Some(0.020), Some(0.025), Some(5), Some(0.050))`
- Ожидание: Единое герметичное тело (watertight = true, genus = 5).
- Факт: `is_watertight: false`, 17956 boundary edges, 274 самопересечения спиц с цилиндром обода и центральной ступицей.
- Артефакты: `qa/artifacts/wheel_rim_wireframe.png`, `qa/artifacts/wheel_rim_normals.png`, `qa/results/primitives_mesh_scan.json`
- Гипотеза причины: Спицы, ступица и профиль обода объединяются в один буфер треугольников простым добавлением (`extend`), без проведения CSG-объединения или сшивки смежных контактных поверхностей, оставляя открытые внутренние границы.
- Тест-регрессия: `crates/mcp-cad-server/tests/qa_wide_scan.rs`
- Статус: OPEN
