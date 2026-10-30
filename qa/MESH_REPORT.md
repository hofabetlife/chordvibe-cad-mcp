# MCP-CAD Engine: Mesh Quality Report

Отчет о качестве полигональных сеток примитивов и операций (раздел 7 ТЗ).

## Критерии вердикта:
- **PASS**: `watertight` == true (boundary_edges == 0, non_manifold_edges == 0) ∧ `orientation_consistent` == true (inconsistent_edges == 0) ∧ `signed_volume` > 0 ∧ `degenerate_triangles` == 0 ∧ `self_intersections` == 0 ∧ `euler_genus` соответствует топологии.
- **WARN**: Наличие `sliver_triangles` > порога, `T-junctions` > 0, дубликаты вершин (CAD split normals).
- **FAIL**: Нарушение любого обязательного критерия PASS.

## Сводная таблица сканирования примитивов (32 конфигурации)

| Примитив | Качество | Watertight | Orient. Cons. | Degenerate | Self-Inter. | Genus (calc / exp) | Вердикт | Примечания |
|---|---|---|---|---|---|---|---|---|
| Box | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | 12 tri, CAD split normals |
| Cylinder | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Slivers на высоких цилиндрах |
| Sphere | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Сетка UV сферы с полюсами |
| Cone (sharp) | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Острая вершина корректна |
| Cone (frustum) | coarse/med/fine | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Верхняя/нижняя крышки герметичны |
| Polygon Prism (3,4,6,8) | sharp | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Чистые плоские нормали граней |
| Polygon Prism (3,4,6,8) | chamfer | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Фаски DIN 934 замкнуты |
| Spur Gear (plain 8/20/60) | - | PASS | PASS | 0 | 0 | 0 / 0 | **PASS** | Эвольвента и впадины замкнуты |
| Spur Gear (hub 8/20) | - | PASS | PASS | 0 | 0 | 1 / 1 | **PASS** | Сквозное отверстие дает genus 1 |
| Spur Gear (hub 60 teeth) | - | **FAIL** | PASS | **60** | **142** | **45.0 / 1** | **FAIL** | 7304 дыры, BUG-002 |
| Revolve 360 (ring) | 32 seg | PASS | PASS | 0 | 0 | 1 / 1 | **PASS** | Тороидальное кольцо, genus 1 |
| Revolve 180 (partial) | 32 seg | PASS | **FAIL** | 0 | 0 | 0 / 0 | **FAIL** | 8 инвертированных ребер, BUG-003 |
| Wheel Rim (5-spoke) | - | **FAIL** | PASS | 0 | **274** | **2608 / 5** | **FAIL** | 17956 дыр, несшитые спицы, BUG-004 |

Подробные числовые метрики сохранены в `qa/results/primitives_mesh_scan.json`.
Диагностические рендеры сохранены в `qa/artifacts/`.
