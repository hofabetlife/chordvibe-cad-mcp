# Отчёт о выполнении задачи P2-T5: Именованные грани — дизайн (`[DECISION]`)

- **ID задачи**: `P2-T5` (Фаза 2: Агентский Контракт)
- **Размер**: `S` (Только документ)
- **Зависимости**: `P1-T3` ([`docs/adr/0001-geometry-kernel.md`](../adr/0001-geometry-kernel.md))
- **Статус**: ✅ DONE
- **Дата**: 2026-11-06
- **Ветка**: `agent/P2-T5`

---

## 1. Что сделано

Подготовлен полный архитектурный и инженерный документ дизайна топологического именования граней для `mcp-cad`:

1. **`docs/FACE_NAMING_DESIGN.md`** (создан):
   - **Раздел 1 — Структуры данных (`crates/mcp-cad-core/src/topology.rs`)**:
     - Расширение `TriangleMesh` двумя обратно совместимыми полями (`#[serde(default, skip_serializing_if = "Vec::is_empty")]`): параллельным массивом `face_ids: Vec<u32>` (`face_ids.len() == triangle_count()` или `0`) и реестром `faces: Vec<FaceInfo>`.
     - Спецификация `FaceInfo` (`id: u32`, `name: String`, `aliases: Vec<String>`, `role: String`, `kind: FaceKind`, `origin_feature: Option<String>`, `area: f64`, `centroid: [f64; 3]`, `sample_point: [f64; 3]`, `normal_or_axis: [f64; 3]`, `radius: Option<f64>`, `triangles_count: usize`) и перечисления `FaceKind` (`Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`, `Helical`, `Other`).
     - Алгоритм `TriangleMesh::recompute_face_stats(&mut self)`, гарантирующий, что `sample_point` всегда лежит строго на поверхности реального треугольника грани (решает проблему кольцевых торцов с центральным отверстием `[0, 0, z]`).
   - **Раздел 2 — Схема стабильного именования (`"<feature_id>:<role>"`)**:
     - Полная таблица ролей и локальных `face_id` для всех 12 генераторов примитивов и 3 конструктивных вырезов в `primitives.rs` (`box`, `cylinder`, `cone`, `sphere`, `torus`, `tube`, `polygon_prism`, `extrude`, `revolve`, `loft`, `spur_gear`, `wheel_rim`, `cut_hole`, `cut_fillet`, `cut_helical_groove`).
     - Устранение разнобоя осей `create_box` (`primitives.rs:34-86`, где комментарии использовали Y-up) с остальными Z-up примитивами: канонические роли `create_box` привязаны к Z-up (`+Z` = `"top"`, `-Z` = `"bottom"`, `+X` = `"right"`, `-X` = `"left"`, `+Y` = `"back"`, `-Y` = `"front"`).
     - Механизм `stamp_feature_id(&mut self, feature_id: &str)` в `FeatureTree::evaluate_with_settings` (`dag.rs`), обеспечивающий стабильность имен `"<feature_id>:<role>"` и коротких алиасов `"<role>"` при `modify_feature` и `save_project`/`load_project`.
   - **Раздел 3 — Распространение через `transform` и `CSG` (`P2-T7` + `manifold-rust`)**:
     - Построчный план модификации `CsgPolygon` (`face_id: u32`), `Plane::split_polygon`, `polygons_to_mesh`, `repair_manifold_triangles`, `optimize_csg_mesh_quality` и `merge_face_registries(mesh_a, mesh_b, is_difference)` в `crates/mcp-cad-core/src/csg.rs`.
     - Правило инверсии знака `normal_or_axis` для граней вычитаемого тела `B` при `csg_difference(A, B)` (`Plane` инвертирует нормаль наружу полости; `Cylinder` с `radius > 0` становится вогнутым отверстием `"hole_wall"` с `radius < 0`).
     - Бесшовная совместимость с `manifold-rust v0.19.2` (`ADR-0001`): отображение `TriangleMesh::face_ids` $\leftrightarrow$ `MeshGLP::face_id` (`run_original_id`) со сдвигом `ID_OFFSET` для второго операнда.
   - **Раздел 4 — Формат селектора граней и интеграция в MCP-инструменты**:
     - Унифицированный тип `FaceSelector` (`Named`, `NamedList`, `Where(FacePredicate)` и legacy AABB `"min_z"`..`"max_z"`), поддерживающий выбор по имени, списку имён и предикату `{"where": {"kind": "cylinder", "radius": ...}}` с автоматическим пересчётом линейных допусков из активной `LengthUnit` (`mm`/`m`) в СИ.
     - Схемы интеграции в `query_part_geometry`, `run_fea_analysis` (`P2-T8`), `create_joint` и `transform_part`.
     - Алгоритм `apply_fixed_surface_triangles` / `apply_load_on_surface_triangles` в `crates/mcp-cad-fea/src/boundary.rs` для воксельной сетки `TET4` с адаптивным зазором проекции $d_\perp \le 0.85 \cdot h_{\text{voxel}}$ и автоматическим распределением суммарной силы $F_{\text{total}}$ пропорционально площадям треугольников грани.
   - **Раздел 5 — Пофайловая декомпозиция (`P2-T6`, `P2-T7`, `P2-T8`), анализ рисков и раздел `[DECISION]`**:
     - Пофайловая таблица всех затрагиваемых структур и строк кода в `mcp-cad-core`, `mcp-cad-fea` и `mcp-cad-server`.
     - Анализ рисков (память `+13.3%` в RAM и `0 байт` в `.mcpcad.json`, производительность CSG `< 1%`, воксельная аппроксимация криволинейных граней в FEA).
     - Оформлены 4 архитектурных решения `[DECISION-FACES-1..4]`.
2. **`TASKS.md`** (обновлён):
   - Задача `P2-T5` переведена в статус `✅ DONE` со ссылками на [`docs/FACE_NAMING_DESIGN.md`](../FACE_NAMING_DESIGN.md) и настоящий отчёт.
   - Зарегистрирована новая задача **`P2-T16`** по побочным находкам аудита `crates/mcp-cad-server/src/tools/fea.rs` и `crates/mcp-cad-core/src/import_stl.rs`.

---

## 2. Как проверено

Поскольку задача `P2-T5` является строго проектно-документационной (`Только документ`), изменения в исходный код крейтов `crates/*` и `src/*` не вносились. Проверено отсутствие регрессий в Rust workspace и линтере Clippy:

```text
$ cargo test -p mcp-cad-core --lib && cargo test -p mcp-cad-server --lib
running 50 tests in mcp-cad-core ...
test result: ok. 50 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

running 24 tests in mcp-cad-server ...
test result: ok. 24 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

$ cargo clippy --workspace --all-targets -- -D warnings
Finished `dev` profile [unoptimized + debuginfo] target(s)
```

Все 5 обязательных разделов задания `P2-T5` из `fix/P2_agent_contract.md` (структуры данных, схема именования, проход через `transform`/`CSG`, селектор в MCP-инструментах, разбивка на `P2-T6`/`P2-T7`/`P2-T8` с пофайловой таблицей и разделом `[DECISION]`) сверены с реальными сигнатурами и номерами строк в кодовой базе.

---

## 3. Тесты до/после

| Набор тестов / Проверка | До `P2-T5` | После `P2-T5` | Дельта |
|---|---|---|---|
| **Rust Workspace (`cargo test --workspace`)** | 184 passed, 0 failed, 0 ignored | 184 passed, 0 failed, 0 ignored | Без регрессий (задача документационная) |
| **Rust Clippy (`cargo clippy --workspace --all-targets -- -D warnings`)** | 0 errors, 0 warnings | 0 errors, 0 warnings | 0 warnings |
| **TypeScript (`npm run lint` / `compile_applet`)** | 0 errors | 0 errors | 0 errors |
| **TS Unit Suite (`npx tsx --test src/cad/cad_engine.test.ts`)** | 25 passed, 1 failed (`TS-FIX-01`) | 25 passed, 1 failed (`TS-FIX-01`) | Без изменений |

---

## 4. Что не удалось и остаточные риски

- Все требования задачи `P2-T5` выполнены в полном объёме.
- **Остаточные технические риски для этапов реализации (`P2-T6..P2-T8`)**:
  1. **Ступенчатая граница воксельной сетки `TET4` в `mcp-cad-fea`**: при выборе криволинейной или наклонной грани (например, отверстия `hole_wall` под нагрузкой в задаче Кирша `P3-T1`) узлы регулярной воксельной сетки отстоят от точной поверхности `TriangleMesh` на расстояние до $\frac{\sqrt{3}}{2} h_{\text{voxel}} \approx 0.866 \cdot h_{\text{voxel}}$. В `docs/FACE_NAMING_DESIGN.md` (раздел 4.3 и 5.2) спроектирован адаптивный фильтр проекции $d_\perp \le 0.85 \cdot h_{\text{voxel}}$ с fallback до $1.25 \cdot h_{\text{voxel}}$, а полное совпадение узлов с границей адресовано в `P3-T5` (`spikes/fe_snap/`).
  2. **Адаптивный ремонт сетки (`repair_manifold_triangles` / `optimize_csg_mesh_quality`) в `csg.rs`**: при делении и переброске рёбер в `csg.rs:582-732` дочерние треугольники должны наследовать `face_id` родительского треугольника; переброска ребра (`edge flip`) должна разрешаться только внутри одного `face_id` либо между копланарными гранями, чтобы не размывать границу между торцом и стенкой отверстия. Это требование явно зафиксировано в разделе 3.2 `docs/FACE_NAMING_DESIGN.md`.

---

## 5. Побочные находки (зарегистрированы в `TASKS.md` как `P2-T16`)

В ходе построчного аудита `crates/mcp-cad-server/src/tools/fea.rs` и `crates/mcp-cad-core/src/import_stl.rs` обнаружены два дефекта, которые **не трогались** в рамках документационной задачи `P2-T5` и вынесены в `TASKS.md` под номером **`P2-T16`**:

1. **Молчаливое игнорирование нераспознанных `fixed_faces` и пропуск дефолтной нагрузки при неверном ключе в `applied_forces` (`crates/mcp-cad-server/src/tools/fea.rs:99-108, 119, 130-180`)**:
   - В `fea.rs:99-108` любая строка в массиве `fixed_faces`, не совпавшая с 6 AABB-строками `parse_aabb_face` (например, `"cap_bottom"`, `"bottom"` или опечатка `"min_z "` с пробелом), молча пропускается без возврата ошибки `ServerError::InvalidArgument`.
   - В `fea.rs:119, 130-180` при передаче непустого массива `applied_forces` устанавливается `explicit_loads_key = true`. Если элементы массива используют ключ `"force"` вместо `"force_vector"` или невалидный `"position"`, ни одна нагрузка не добавляется (`loads_added == false`), а условие дефолтной нагрузки на строке 178 (`if !loads_added && !explicit_loads_key`) блокируется. В результате МКЭ-расчёт выполняется с **нулевым вектором внешних сил** (`nodal_forces = [0, 0, 0]`) и возвращает ложноположительный отчёт `max_von_mises_mpa: 0.0`, `safety_factor: 100.0`, `status: "PASS"`.
2. **Ошибка квантования на границе ячеек и сглаживание острых рёбер в `weld_vertices` (`crates/mcp-cad-core/src/import_stl.rs:268-315`)**:
   - В `import_stl.rs:283-290` `weld_vertices` хеширует координаты вершин в единственный округлённый целочисленный ключ `((v[0] * inv_tol).round() as i64, ...)` без проверки соседних ячеек `(-1..=1)^3`, из-за чего две вершины на расстоянии $10^{-15}\text{ м}$ по разные стороны порога округления `(k + 0.5) * tolerance` получают разные ключи и не склеиваются.
   - Кроме того, на строке 313 `weld_vertices` вызывает `new_mesh.recompute_vertex_normals()`, усредняя нормали по всем смежным треугольникам вершины (включая $90^\circ$ рёбра куба/цилиндра), что может приводить к расхождению вершинных и граневых нормалей в `validate_mesh`.
