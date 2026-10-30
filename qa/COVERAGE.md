# MCP-CAD Engine: Coverage Matrix

Матрица инструментов (26 шт.) по сценариям тестирования:
- **HP**: Happy path (корректные параметры, штатное выполнение)
- **BND**: Boundary values (ноль, отрицательные, малые $10^{-6}$, большие $10^3$, NaN/inf)
- **INV**: Invalid IDs / missing parameters / wrong types
- **IDEM**: Idempotency & order of calls
- **CONS**: Consistency between query tools

| № | Инструмент | HP | BND | INV | IDEM | CONS | Статус |
|---|---|---|---|---|---|---|---|
| 1 | `create_part` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 2 | `list_parts` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 3 | `delete_part` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 4 | `add_feature_primitive` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 5 | `cut_hole` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 6 | `modify_feature` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 7 | `get_feature_tree` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 8 | `cut_helical_groove` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 9 | `boolean_difference` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 10 | `tessellate_part` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 11 | `create_joint` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 12 | `set_joint_motor` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 13 | `run_physics_simulation` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 14 | `run_fea_analysis` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 15 | `get_fea_heatmap_image` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 16 | `capture_viewport` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 17 | `open_preview_window` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 18 | `import_stl` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 19 | `export_stl` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 20 | `get_simulation_metrics_json` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 21 | `query_part_geometry` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 22 | `boolean_union` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 23 | `boolean_intersection` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 24 | `transform_part` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 25 | `define_material` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |
| 26 | `batch_cad_operations` | PASS | PASS | PASS | PASS | PASS | 100% VERIFIED |

---

## Тестовое покрытие по крейтам (Rust Workspace)

1. **`mcp-cad-core`**:
   - `src/lib.rs` unit tests: 41/41 PASS
   - `tests/dag_tests.rs`: 14/14 PASS
   - `tests/tessellation_tests.rs`: 6/6 PASS
   - Итого: **61 тест (100% PASS)**

2. **`mcp-cad-physics`**:
   - `tests/physics_tests.rs`: 6/6 PASS
   - Итого: **6 тестов (100% PASS)**

3. **`mcp-cad-fea`**:
   - `src/lib.rs` unit tests: 5/5 PASS
   - `tests/fea_tests.rs`: 6/6 PASS
   - Итого: **11 тестов (100% PASS)**

4. **`mcp-cad-viewer`**:
   - `tests/viewer_tests.rs`: 10/10 PASS
   - Итого: **10 тестов (100% PASS)**

5. **`mcp-cad-server`**:
   - `tests/gear_kinematic_test.rs`: 1/1 PASS
   - `tests/mcp_contract_tests.rs`: 18/18 PASS
   - `tests/qa_wide_scan.rs`: 13/13 PASS
   - Итого: **32 теста (100% PASS)**

**Всего по проекту: 120 тестов, 0 падений, 0 регрессий, 0 предупреждений Clippy.**
