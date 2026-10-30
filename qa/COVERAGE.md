# MCP-CAD Engine: Coverage Matrix

Матрица инструментов (26 шт.) по сценариям тестирования:
- **HP**: Happy path
- **BND**: Boundary values (zero, negative, tiny 1e-6, huge 1e3, NaN/inf)
- **INV**: Invalid IDs / missing parameters / wrong types
- **IDEM**: Idempotency & order of calls
- **CONS**: Consistency between query tools

| № | Инструмент | HP | BND | INV | IDEM | CONS | Статус |
|---|---|---|---|---|---|---|---|
| 1 | `create_part` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 2 | `list_parts` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 3 | `delete_part` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 4 | `add_feature_primitive` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 5 | `cut_hole` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 6 | `modify_feature` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 7 | `get_feature_tree` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 8 | `cut_helical_groove` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 9 | `boolean_difference` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 10 | `tessellate_part` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 11 | `create_joint` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 12 | `set_joint_motor` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 13 | `run_physics_simulation` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 14 | `run_fea_analysis` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 15 | `get_fea_heatmap_image` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 16 | `capture_viewport` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 17 | `open_preview_window` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 18 | `import_stl` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 19 | `export_stl` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 20 | `get_simulation_metrics_json` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 21 | `query_part_geometry` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 22 | `boolean_union` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 23 | `boolean_intersection` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 24 | `transform_part` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 25 | `define_material` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
| 26 | `batch_cad_operations` | UNTESTED | UNTESTED | UNTESTED | UNTESTED | UNTESTED | PENDING |
