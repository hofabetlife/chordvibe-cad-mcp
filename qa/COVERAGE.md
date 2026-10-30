# MCP-CAD Engine: Coverage Matrix

Матрица инструментов (26 шт.) по сценариям тестирования:
- **HP**: Happy path
- **BND**: Boundary values (zero, negative, tiny 1e-6, huge 1e3, NaN/inf)
- **INV**: Invalid IDs / missing parameters / wrong types
- **IDEM**: Idempotency & order of calls
- **CONS**: Consistency between query tools

| № | Инструмент | HP | BND | INV | IDEM | CONS | Статус |
|---|---|---|---|---|---|---|---|
| 1 | `create_part` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 2 | `list_parts` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 3 | `delete_part` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 4 | `add_feature_primitive` | FAIL (BUG-002..005) | FAIL (BUG-005,008) | PASS | FAIL (BUG-008) | PASS | DEFECTS FOUND |
| 5 | `cut_hole` | FAIL (BUG-006) | FAIL (BUG-008) | PASS | FAIL (BUG-008) | PASS | DEFECTS FOUND |
| 6 | `modify_feature` | PASS | FAIL (BUG-008) | PASS | PASS | PASS | DEFECTS FOUND |
| 7 | `get_feature_tree` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 8 | `cut_helical_groove` | FAIL (BUG-006) | FAIL (BUG-008) | PASS | FAIL (BUG-008) | PASS | DEFECTS FOUND |
| 9 | `boolean_difference` | FAIL (BUG-006) | FAIL (BUG-008) | PASS | FAIL (BUG-008) | PASS | DEFECTS FOUND |
| 10 | `tessellate_part` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 11 | `create_joint` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 12 | `set_joint_motor` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 13 | `run_physics_simulation` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 14 | `run_fea_analysis` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 15 | `get_fea_heatmap_image` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 16 | `capture_viewport` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 17 | `open_preview_window` | PASS | PASS | FAIL (BUG-010) | PASS | PASS | DEFECTS FOUND |
| 18 | `import_stl` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 19 | `export_stl` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 20 | `get_simulation_metrics_json` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 21 | `query_part_geometry` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 22 | `boolean_union` | FAIL (BUG-006) | FAIL (BUG-008) | PASS | FAIL (BUG-008) | PASS | DEFECTS FOUND |
| 23 | `boolean_intersection` | FAIL (BUG-006) | FAIL (BUG-008) | PASS | FAIL (BUG-008) | PASS | DEFECTS FOUND |
| 24 | `transform_part` | PASS | FAIL (BUG-007) | PASS | FAIL (BUG-008) | PASS | DEFECTS FOUND |
| 25 | `define_material` | PASS | PASS | PASS | PASS | PASS | TESTED |
| 26 | `batch_cad_operations` | PASS | PASS | PASS | FAIL (BUG-009) | PASS | DEFECTS FOUND |
