# MCP-CAD Engine: Mesh Quality Report

Отчет о качестве полигональных сеток примитивов и операций по результатам Фазы 2 (Wide Scan, Раздел 7 ТЗ).

## Критерии вердикта:
- **PASS**: `watertight` == true (`boundary_edges == 0`, `non_manifold_edges == 0`) ∧ `orientation_consistent` == true (`inconsistent_edges == 0`) ∧ `signed_volume` > 0 ∧ `degenerate_triangles` == 0 ∧ `self_intersections` == 0 ∧ `euler_genus` соответствует топологии.
- **WARN**: Наличие `sliver_triangles` > порога, `T-junctions` > 0, дубликаты вершин (CAD split normals).
- **FAIL**: Нарушение любого обязательного критерия PASS.

## 1. Сводная таблица сканирования примитивов (37 конфигураций)

| Примитив | Качество / Параметры | Watertight | Orient. Cons. | Degenerate | Self-Inter. | Genus (calc / exp) | Вердикт | Примечания |
|---|---|---|---|---|---|---|---|---|
| Box | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | 12 tri, эталонный объем |
| Cylinder | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Ошибка объема < 0.06% (fine) |
| Sphere | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Ошибка объема < 0.14% (fine) |
| Cone (sharp) | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Острая вершина герметична |
| Cone (frustum) | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Ошибка объема < 0.06% (fine) |
| Polygon Prism (3,4,6,8) | chamfer = 0.0 | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Чистые плоские грани |
| Polygon Prism (3,4) | chamfer = 0.002 | PASS | PASS | 0 | 0 | 0 / 0 | **FAIL (BUG-005)** | Фаска проигнорирована (объем не изменился) |
| Polygon Prism (6,8) | chamfer = 0.002 | **FAIL (60/112)** | PASS | 0 | **7 / 21** | **10..16 / 0** | **FAIL (BUG-005)** | Дыры и T-junctions от CSG-фаски |
| Spur Gear (plain 8/20/60) | pa = 14.5/20/25 | PASS | PASS | **32..240** | **32..240** | 0 / 0 | **FAIL (BUG-002)** | Коллинеарные треугольники крышек при r_f < r_b |
| Spur Gear (hub/bore/keyway) | teeth = 8/20/60 | **FAIL (988..7304)** | PASS | **54..60** | **106..559** | **45..145 / 1** | **FAIL (BUG-002)** | Дыры от CSG объединения и вычитания |
| Revolve 360 (hollow ring) | 32 seg | PASS | PASS | 0 | 0 | 1 / 1 | **PASS** | Тороидальное кольцо, genus 1 |
| Revolve 360 (touching axis) | 32 seg | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Замкнутое тело на оси Z |
| Revolve 180 (partial) | 32 seg | PASS | **FAIL (8)** | 0 | 0 | 0 / 0 | **FAIL (BUG-003)** | Инвертированы крышки среза |
| Wheel Rim (5,6,10 spokes) | 18"x8" | **FAIL (17956..19362)** | PASS | 0 | **273..328** | **2608..2793** | **FAIL (BUG-004)** | Несшитые спицы протыкают обод |

## 2. Сводная таблица сканирования операций и цепочек (11 конфигураций)

| Операция | Watertight | Orient. Cons. | Boundary Edges | T-Junctions | Self-Inter. | Genus (calc / exp) | Вердикт | Примечания |
|---|---|---|---|---|---|---|---|---|
| `cut_hole` (through box) | **FAIL** | PASS | 170 | 68 | 60 | 19.0 / 1 | **FAIL (BUG-006)** | T-junctions и удаленные микро-треугольники |
| `cut_hole` (blind box) | **FAIL** | PASS | 130 | 58 | 30 | 17.0 / 0 | **FAIL (BUG-006)** | Несшитые ребра дна и стенок отверстия |
| `cut_hole` (with keyway) | **FAIL** | PASS | 231 | 93 | 77 | 27.5 / 1 | **FAIL (BUG-006)** | Разрывы на стыке паза и отверстия |
| `cut_hole` (angled [1,0,1]) | **FAIL** | PASS | 172 | 62 | 71 | 19.0 / 1 | **FAIL (BUG-006)** | Несшитые диагональные точки среза |
| `cut_helical_groove` (circular) | **FAIL** | **FAIL (36)** | 24777 | 0 | 2279 | 419.0 / 0 | **FAIL (BUG-006)** | Потеря мелких треугольников (`norm_sq <= 1e-16`) |
| `cut_helical_groove` (metric_60) | **FAIL** | PASS | 18446 | 0 | 428 | 153.0 / 0 | **FAIL (BUG-006)** | Потеря мелких треугольников резьбы |
| `boolean_union` (coplanar boxes) | **FAIL** | PASS | 24 | 8 | 0 | 4.0 / 0 | **FAIL (BUG-006)** | 8 T-junctions = 24 граничных ребра |
| `boolean_difference` (coplanar) | **FAIL** | PASS | 12 | 4 | 0 | 2.0 / 0 | **FAIL (BUG-006)** | 4 T-junctions = 12 граничных ребер |
| `boolean_intersection` (coplanar) | **FAIL** | PASS | 12 | 4 | 0 | 2.0 / 0 | **FAIL (BUG-006)** | 4 T-junctions = 12 граничных ребер |
| `transform_part` (neg scale) | PASS | PASS | 0 | 0 | 0 | 0 / 0 | **FAIL (BUG-007)** | `signed_volume = -0.006 < 0` (вывернут) |
| `chain_box_two_holes_genus2` | **FAIL** | PASS | 434 | 174 | 124 | 52.0 / 2 | **FAIL (BUG-006)** | Накопление T-junctions в цепочке CSG |
