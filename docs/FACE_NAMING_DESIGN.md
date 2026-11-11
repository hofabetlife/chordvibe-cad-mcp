# Дизайн-документ: Топологическое именование граней (`face_id` / `FaceInfo`) и семантические селекторы

- **Статус:** Proposed (`[DECISION]` — содержит варианты по умолчанию, согласованные с `ADR-0001` и `UNITS.md`)
- **Дата:** 2026-11-06
- **Задача:** `P2-T5` (вход для `P2-T6`, `P2-T7`, `P2-T8`, `P1-K4`, `P3-T7`)
- **Связанные документы:** [`docs/adr/0001-geometry-kernel.md`](adr/0001-geometry-kernel.md), [`agent_prompts/P1_kernel_migration.md`](../agent_prompts/P1_kernel_migration.md), [`docs/UNITS.md`](UNITS.md)

---

## 1. Контекст и постановка проблемы

### 1.1. Ограничения текущей реализации (проверено по кодовой базе)

На текущий момент (после завершения `P0-T1..P0-T12`, `P1-T1..P1-T4`, `P2-T1..P2-T4`) геометрическое представление детали в `mcp-cad-core` и инструменты `mcp-cad-server` имеют следующие ограничения при адресации поверхностей:

1. **Отсутствие идентификаторов граней в `TriangleMesh` (`crates/mcp-cad-core/src/topology.rs:150-155`):**
   - Структура `TriangleMesh` хранит только `pub vertices: Vec<Vertex>` и `pub indices: Vec<[u32; 3]>`.
   - После генерации примитива в `crates/mcp-cad-core/src/primitives.rs` информация о том, какие треугольники принадлежат верхнему торцу цилиндра, какие — боковой цилиндрической стенке, а какие — поверхности просверленного отверстия `cut_hole`, безвозвратно теряется.
2. **Геометрическая эвристика `inspect_geometry` (`crates/mcp-cad-core/src/topology.rs:377-515`):**
   - Метод `TriangleMesh::inspect_geometry()` возвращает только `detected_planar_faces: Vec<PlanarFaceInfo>` (до 15 крупнейших копланарных кластеров).
   - Он **не распознаёт криволинейные поверхности** (`Cylinder`, `Cone`, `Sphere`, `Torus`, `Helical`), не присваивает граням стабильных имён, использует фиксированный размерный порог плоскости `1e-4` (см. `P2-T14`) и для кольцевого торца с отверстием возвращает `center = [0, 0, z]` в пустоте отверстия.
3. **Привязка граничных условий FEA только к габаритному параллелепипеду (`crates/mcp-cad-server/src/tools/fea.rs:24-34, 74-175`, `crates/mcp-cad-fea/src/boundary.rs:28-120`):**
   - В `run_fea_analysis` закрепления задаются только через `fixed_faces: ["min_x".."max_z"]` (`parse_aabb_face`) или `fixed_plane`, а нагрузки `applied_forces` — либо точечно к ближайшему узлу (`position`), либо по умолчанию на грань `MaxX`.
   - ИИ-агент **не может** сказать: *«закрепи внутреннюю цилиндрическую поверхность отверстия под подшипник `hole_wall` и приложи распределённую силу 5000 Н к верхнему торцу `cap_top`»*.
4. **Ручное вычисление координат якорей в `create_joint` (`crates/mcp-cad-server/src/tools/physics.rs:116-290`):**
   - Для создания шарнира агенту приходится вручную рассчитывать 3D-координаты `anchor_a`, `anchor_b` и вектор `axis`, вместо того чтобы указать сопрягаемые грани деталей (`face_a: "hole_wall"`, `face_b: "side"`).

### 1.2. Связь с выбором геометрического ядра (`ADR-0001` / `P1-T3`)

Согласно [`docs/adr/0001-geometry-kernel.md`](adr/0001-geometry-kernel.md):
- В качестве целевого ядра булевых операций выбран **Вариант A (`manifold-rust v0.19.2`, Pure Rust, `Apache-2.0`)**, в котором структура `MeshGLP<f64, u32>` нативно содержит поле `face_id: Vec<u32>` и `run_original_id: Vec<u32>`, автоматически сохраняющее принадлежность каждого треугольника исходной грани при булевых операциях `union`, `difference`, `intersection` (проверено в `spikes/manifold_rust/src/main.rs:273-316` — ошибка площади верхнего торца после двух булевых операций `0.0000%`).
- При этом дизайн слоя именованных граней в `TriangleMesh` (`face_ids: Vec<u32>` + `faces: Vec<FaceInfo>`) проектируется **инвариантным к реализации CSG**:
  - **В текущем BSP-ядре (`crates/mcp-cad-core/src/csg.rs`)** `face_id` переносится полем в `CsgPolygon` через `Plane::split_polygon`, `polygons_to_mesh`, `repair_manifold_triangles` и `optimize_csg_mesh_quality` (см. раздел 3.2).
  - **При переходе на `manifold-rust` (`P1-K1..P1-K4`)** тот же вектор `TriangleMesh::face_ids` напрямую передаётся в `MeshGLP::face_id` и возвращается обратно без изменения публичного API `TriangleMesh`, `FaceInfo` и MCP-инструментов (см. раздел 3.3).
  - **При опциональном использовании B-Rep (`monstertruck`, Вариант C из `ADR-0001`)** адаптер тесселяции оболочки записывает индекс `Shell::faces` в тот же `TriangleMesh::face_ids` и заполняет `faces: Vec<FaceInfo>` аналитическими параметрами поверхности (см. раздел 3.4).

---

## 2. Структуры данных и обратная совместимость `serde` (Пункт 1)

### 2.1. Новые типы в `crates/mcp-cad-core/src/topology.rs`

В модуль `crates/mcp-cad-core/src/topology.rs` (рядом с `PlanarFaceInfo` и `GeometryInspection`, строки 116–160) вводятся следующие сериализуемые типы:

```rust
/// Геометрический класс аналитической или сеточной поверхности грани.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum FaceKind {
    /// Плоская грань (торцы, стенки бокса/призмы, фаски, дно глухого отверстия, стенки шпоночного паза).
    Plane,
    /// Цилиндрическая поверхность (боковая поверхность цилиндра, стенка круглого отверстия, шейка вала).
    Cylinder,
    /// Коническая поверхность (боковая поверхность конуса/усечённого конуса, коническая зенковка/фаска вращения).
    Cone,
    /// Сферическая поверхность.
    Sphere,
    /// Тороидальная поверхность или поверхность галтели/скругления вращения.
    Torus,
    /// Винтовая поверхность (канавка сверла, резьбовой профиль `CutHelicalGroove`).
    Helical,
    /// Произвольная/составная поверхность (эвольвентный профиль зуба, спица обода, импортированный STL без разметки).
    #[default]
    Other,
}

/// Метаданные именованной топологической грани полигональной сетки `TriangleMesh`.
///
/// Все размерные величины внутри `mcp-cad-core` хранятся строго в СИ (`m`, `m²`),
/// а на границе MCP API (`crates/mcp-cad-server/src/tools/*.rs`) конвертируются в активный
/// `length_unit` проекта (`mm` по умолчанию) согласно `docs/UNITS.md`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FaceInfo {
    /// Уникальный в пределах данной `TriangleMesh` ненулевой идентификатор грани (`>= 1`).
    /// Значение `0` в `TriangleMesh::face_ids` зарезервировано для неразмеченных треугольников.
    pub id: u32,
    /// Каноническое стабильное имя грани (например, `"cap_top"`, `"feat_0:cap_top"`, `"hole_wall:1"`).
    pub name: String,
    /// Дополнительные короткие или совместимые алиасы имени грани для удобного поиска селектором
    /// (например, `["cap_top", "top", "max_z"]` для верхнего торца базового тела).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub aliases: Vec<String>,
    /// Локальная роль грани внутри породившего примитива/операции (`"cap_top"`, `"side"`, `"hole_wall"`).
    pub role: String,
    /// Тип геометрии поверхности грани.
    pub kind: FaceKind,
    /// Идентификатор узла `FeatureNode::id` (UUID в строковом виде) или имя операции в `FeatureTree`,
    /// породившей данную грань. `None` для сеток, созданных напрямую вне DAG.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin_feature: Option<String>,
    /// Суммарная площадь треугольников грани в м² (внутри ядра).
    pub area: f64,
    /// Площадь-взвешенный центроид грани `[x, y, z]` в м:
    /// `sum(area_t * centroid_t) / sum(area_t)`.
    pub centroid: [f64; 3],
    /// Точка, гарантированно лежащая на самой поверхности грани `[x, y, z]` в м
    /// (центроид ближайшего к `centroid` треугольника данной грани).
    /// Для кольцевых торцов с центральным отверстием `centroid` попадает на ось `[0, 0, z]`,
    /// тогда как `sample_point` лежит непосредственно на металле торца.
    pub sample_point: [f64; 3],
    /// Для `FaceKind::Plane` — единичный вектор внешней нормали `[nx, ny, nz]`.
    /// Для `Cylinder`, `Cone`, `Torus`, `Helical` — единичный вектор оси вращения `[ax, ay, az]`.
    /// Для `Sphere` / `Other` — средняя внешняя нормаль или `[0.0, 0.0, 1.0]`.
    pub normal_or_axis: [f64; 3],
    /// Характерный радиус в м:
    /// - для `Cylinder`, `Sphere`, `Helical` — радиус поверхности `Some(r)`;
    /// - для `Cone` — средний радиус `Some(0.5 * (r_bottom + r_top))`;
    /// - для кольцевой или круглой плоской крышки `Plane` — внешний радиус `Some(r_outer)`;
    /// - для прямоугольных граней бокса/призмы — `None`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub radius: Option<f64>,
    /// Количество треугольников в `TriangleMesh::indices`, принадлежащих этой грани (`face_ids[t] == id`).
    pub triangles_count: usize,
}
```

### 2.2. Расширение структуры `TriangleMesh` (`crates/mcp-cad-core/src/topology.rs:150`)

```rust
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TriangleMesh {
    /// Массив вершин сетки с координатами и нормалями.
    pub vertices: Vec<Vertex>,
    /// Массив троек индексов вершин (по 1 элементу на треугольник).
    pub indices: Vec<[u32; 3]>,
    /// Параллельный массив идентификаторов граней (по 1 `u32` на каждый треугольник в `indices`).
    /// Инвариант: `face_ids.is_empty()` (разметка не задана) ИЛИ `face_ids.len() == indices.len()`.
    /// Значение `0` означает неразмеченный треугольник (`FaceKind::Other`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub face_ids: Vec<u32>,
    /// Реестр именованных граней тела, упорядоченный по `id` по возрастанию.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub faces: Vec<FaceInfo>,
}
```

### 2.3. Инварианты `TriangleMesh` и влияние на `serde` / существующий код

1. **Инвариант параллельного массива:**
   - В любой размеченной сетке выполняется `mesh.face_ids.len() == mesh.indices.len()`.
   - Сумма площадей всех зарегистрированных граней в `mesh.faces` (плюс площадь треугольников с `face_id == 0`, если такие есть) строго равна полной площади поверхности сетки:
     $$\left|\sum_{f \in \text{faces}} f.\text{area} + A(\text{id}=0) - \text{mesh.surface\_area}()\right| < 10^{-9}\cdot \text{mesh.surface\_area}()$$
   - Для базовых примитивов (`create_box`, `create_cylinder`, `create_cone`, `create_sphere`, `create_polygon_prism`) каждый треугольник получает `face_id >= 1`, поэтому `A(id=0) == 0`.
2. **Обратная совместимость `serde` (`#[serde(default, skip_serializing_if = "Vec::is_empty")]`):**
   - **Чтение старых файлов:** десериализация любых существующих JSON-файлов (`examples/bracket_design.json`, `examples/crank_mechanism.json`, `examples/gear_transmission.json`, сохранённых проектов `save_project` со схемой `1.0.0`, а также `FeatureOp::BooleanUnionMesh / BooleanDifferenceMesh / BooleanIntersectionMesh` с инлайновым `tool_mesh`) работает **без ошибок**: отсутствующие поля `face_ids` и `faces` инициализируются пустыми векторами `Vec::new()`.
   - **Компактность при сериализации:** если `face_ids` и `faces` пусты (например, для сырого импортированного STL или анимационных кадров), `skip_serializing_if = "Vec::is_empty"` вообще не пишет эти ключи в JSON.
   - **Влияние на `save_project` / `load_project` (`crates/mcp-cad-server/src/tools/project.rs:156-160`):** для процедурных деталей с непустым DAG сетка в файл проекта не сохраняется (`mesh: None`), а при `load_project` вызывается `part.rebuild()`, который детерминированно заново строит `face_ids` и `faces` из `FeatureTree`.
3. **Влияние на конструкторы и структурные литералы `TriangleMesh`:**
   - `TriangleMesh::new(vertices, indices)` и `TriangleMesh::empty()` (`topology.rs:165-175`) сохраняют прежнюю сигнатуру, инициализируя `face_ids: Vec::new()` и `faces: Vec::new()`.
   - Добавляется конструктор `TriangleMesh::with_faces(vertices: Vec<Vertex>, indices: Vec<[u32; 3]>, face_ids: Vec<u32>, faces: Vec<FaceInfo>) -> Self` и метод `mesh.recompute_face_registry()`.
   - Единственный прямой структурный литерал вне `topology.rs` в рабочем коде (`spikes/manifold_rust/src/main.rs:170`) заменяется на `TriangleMesh::new(vertices, indices)` или `TriangleMesh::with_faces(...)`.

---

## 3. Каноническая схема именования граней и её стабильность в DAG (Пункт 2)

### 3.1. Двухуровневая схема имён: локальная роль и квалифицированное имя узла DAG

ИИ-агенту одинаково важно уметь:
- на простой детали сказать коротко `{"face": "cap_top"}` или `{"face": "hole_wall"}`;
- на сложной многоступенчатой детали с несколькими цилиндрами и отверстиями однозначно сослаться на грань конкретной операции: `{"face": "hole_1:hole_wall"}` или `{"face": "<feature_uuid>:hole_wall"}`.

Поэтому вводится **двухуровневая детерминированная схема именования**:

1. **Уровень 1 — Локальная роль примитива (`role`):**
   Назначается внутри генераторов `crates/mcp-cad-core/src/primitives.rs`. Каждая грань примитива получает детерминированный `id: u32` (`1, 2, ...`), каноническую строку `role` (например, `"cap_top"`, `"cap_bottom"`, `"side"`) и начальное `name = role.clone()`.
2. **Уровень 2 — Квалификация при оценке `FeatureTree::evaluate_with_settings` (`crates/mcp-cad-core/src/dag.rs:367`):**
   Когда узел `node: &FeatureNode` (с индексом `node_idx` среди активных операций и стабильным `node.id: Uuid`) создаёт или модифицирует тело:
   - Поле `face.origin_feature` заполняется строкой `Some(node.id.to_string())`.
   - Формируется стабильный человекочитаемый префикс узла `<feature_slug>` из `node.name` (нормализованный `snake_case`, например `"base_cylinder"`, `"cut_hole_1"`) и индексный префикс `feat_{node_idx}`.
   - Для **первого (базового) тела** в дереве (`node_idx == 0`) основное имя `face.name` остаётся коротким (`"<role>"`, например `"cap_top"`), а в `face.aliases` добавляются квалифицированные формы:
     - `"<node.id>:<role>"` (абсолютно стабильно при любых переименованиях);
     - `"<feature_slug>:<role>"` (удобно, если агент передал `"name": "shaft"` при создании фичи);
     - `"feat_0:<role>"`.
   - Для **последующих добавляемых/вычитаемых операций (`node_idx >= 1`)**:
     - Если роль уникальна в текущем реестре граней тела (например, первое отверстие `hole_wall`), короткое имя `"hole_wall"` сохраняется в `aliases`, а каноническое `face.name` имеет вид:
       - для вырезов (`CutHole`, `CutHelicalGroove`, `BooleanDifferenceMesh`): `"cut:<short_id>:<role>"` (или `"hole_wall:<short_id>"`), где `<short_id>` — `feature_slug` (если имя было задано пользователем) либо `feat_{node_idx}`, а полный `"<node.id>:<role>"` и `"cut:<node.id>:<role>"` всегда записываются в `aliases`.
     - Если короткая роль уже занята другой гранью (например, второй цилиндр тоже имеет `cap_top`), короткий алиас `"cap_top"` остаётся за базовым примитивом (либо переводится в неоднозначный и требует уточнения или `where`), а у второго цилиндра грань именуется `"feat_1:cap_top"` (и `"<feature_slug>:cap_top"`, `"<node.id>:cap_top"`).

#### Гарантия стабильности при пересчёте DAG (`part.rebuild()` и `modify_feature`):
- В `FeatureTree` (`crates/mcp-cad-core/src/dag.rs:259-270`) каждый узел хранит постоянный `pub id: Uuid`, который **не меняется** при вызове `modify_feature(id, new_op)` (`dag.rs:329`), `set_feature_enabled` (`dag.rs:360`), `tessellate` (`part.rs:52`) и при сохранении/загрузке проекта (`project.rs:53`).
- Порядок обхода граней внутри каждого генератора в `primitives.rs` строго детерминирован.
- Следовательно, при любом повторном вызове `part.rebuild()` или смене плотности тесселяции (`Coarse` / `Medium` / `Fine`) имена `face.name`, `face.role`, `face.origin_feature` и `face.aliases` воспроизводятся **на 100% идентично**.

### 3.2. Таблица канонических граней для всех примитивов `primitives.rs`

| Примитив (`primitives.rs`) | Локальный `id` | Каноническая роль (`role`) | Совместимые алиасы (`aliases`) | `FaceKind` | `normal_or_axis` (до трансформации) | `radius` | Аналитическая площадь грани |
|---|---:|---|---|---|---|---|---|
| **`create_box(w, h, d)`** (`:13–89`) | `1` | `top` (`+Z`) | `["cap_top", "max_z", "+z", "front_z"]` | `Plane` | `[0.0, 0.0, 1.0]` | `None` | $w \cdot h$ |
| | `2` | `bottom` (`-Z`) | `["cap_bottom", "min_z", "-z", "back_z"]` | `Plane` | `[0.0, 0.0, -1.0]` | `None` | $w \cdot h$ |
| | `3` | `right` (`+X`) | `["max_x", "+x"]` | `Plane` | `[1.0, 0.0, 0.0]` | `None` | $h \cdot d$ |
| | `4` | `left` (`-X`) | `["min_x", "-x"]` | `Plane` | `[-1.0, 0.0, 0.0]` | `None` | $h \cdot d$ |
| | `5` | `back` (`+Y`) | `["max_y", "+y", "top_y"]` | `Plane` | `[0.0, 1.0, 0.0]` | `None` | $w \cdot d$ |
| | `6` | `front` (`-Y`) | `["min_y", "-y", "bottom_y"]` | `Plane` | `[0.0, -1.0, 0.0]` | `None` | $w \cdot d$ |
| **`create_cylinder(r, h, segs)`** (`:322–370`) | `1` | `cap_top` | `["top", "max_z", "+z"]` | `Plane` | `[0.0, 0.0, 1.0]` | `Some(r)` | $\frac{N}{2} r^2 \sin\frac{2\pi}{N} \approx \pi r^2$ |
| | `2` | `cap_bottom` | `["bottom", "min_z", "-z"]` | `Plane` | `[0.0, 0.0, -1.0]` | `Some(r)` | $\frac{N}{2} r^2 \sin\frac{2\pi}{N} \approx \pi r^2$ |
| | `3` | `side` | `["outer_wall", "cylindrical_wall"]` | `Cylinder` | `[0.0, 0.0, 1.0]` (ось $Z$) | `Some(r)` | $2 N r \sin\frac{\pi}{N} \cdot h \approx 2\pi r h$ |
| **`create_cone(r_bot, r_top, h, segs)`** (`:810–1087`) | `1` *(если $r_{\text{bot}}>0$)* | `cap_bottom` | `["bottom", "min_z", "-z"]` | `Plane` | `[0.0, 0.0, -1.0]` | `Some(r_bot)` | $\approx \pi r_{\text{bot}}^2$ |
| | `2` *(если $r_{\text{top}}>0$)* | `cap_top` | `["top", "max_z", "+z"]` | `Plane` | `[0.0, 0.0, 1.0]` | `Some(r_top)` | $\approx \pi r_{\text{top}}^2$ |
| | `3` (или `2`) | `side` | `["conical_wall"]` | `Cone` | `[0.0, 0.0, 1.0]` (ось $Z$) | `Some(0.5*(r_bot+r_top))` | $\approx \pi (r_{\text{bot}} + r_{\text{top}}) \sqrt{h^2 + (r_{\text{bot}}-r_{\text{top}})^2}$ |
| **`create_sphere(r, sec, stk)`** (`:383–497`) | `1` | `surface` | `["sphere_surface", "side"]` | `Sphere` | `[0.0, 0.0, 1.0]` | `Some(r)` | $\approx 4\pi r^2$ |
| **`create_polygon_prism(n, h, ...)`** (`:1111–1464`) | `1` | `cap_top` | `["top", "max_z", "+z"]` | `Plane` | `[0.0, 0.0, 1.0]` | `Some(r_cap)` | $\frac{n}{2} r_{\text{cap}}^2 \sin\frac{2\pi}{n}$ |
| | `2` | `cap_bottom` | `["bottom", "min_z", "-z"]` | `Plane` | `[0.0, 0.0, -1.0]` | `Some(r_cap)` | $\frac{n}{2} r_{\text{cap}}^2 \sin\frac{2\pi}{n}$ |
| | `3..3+n` | `side_0` … `side_{n-1}` | `["flat_0".."flat_{n-1}"]` | `Plane` | $[\cos\theta_i, \sin\theta_i, 0]$ | `None` | Площадь $i$-й вертикальной грани |
| *(при `chamfer > 0`)* | `3+n..3+2n` | `chamfer_top_0` … `chamfer_top_{n-1}` | — | `Plane` | Наклонная нормаль $+Z$ | `None` | Площадь верхнего скоса фаски $i$ |
| *(при `chamfer > 0`)* | `3+2n..3+3n` | `chamfer_bottom_0` … `chamfer_bottom_{n-1}` | — | `Plane` | Наклонная нормаль $-Z$ | `None` | Площадь нижнего скоса фаски $i$ |
| **`cut_hole_with_settings(...)`** (`:588–678`) | резец `1` | `hole_bottom` *(выживает у глухого отверстия)* | `["cut_bottom"]` | `Plane` | `unit_norm` (из дна наружу в полость) | `Some(r)` | $\approx \pi r^2$ (для сквозного отверстия срезается, `triangles_count == 0`) |
| | резец `2` | `hole_top` *(снаружи детали, срезается)* | — | `Plane` | `-unit_norm` | `Some(r)` | $0$ (полностью вне тела $A$) |
| | резец `3` | `hole_wall` | `["bore", "hole"]` | `Cylinder` | `unit_norm` (ось отверстия) | `Some(r)` | $\approx 2\pi r h_{\text{eff}}$ |
| *(при `keyway`)* | резец `4..=6` | `keyway_wall` | `["keyway"]` | `Plane` | Нормали стенок паза | `None` | Площадь стенок шпоночного паза |
| **`create_spur_gear` / `create_wheel_rim` / `create_revolve`** | `0` (в `P2-T6`) | `other` | — | `Other` | `[0.0, 0.0, 1.0]` | `None` | Согласно заданию `P2-T6`, остаются вне обязательного объёма (`face_id = 0` или базовые `cap_top`/`cap_bottom`/`bore_wall`). |

> **Примечание о соглашении осей в `create_box` (`primitives.rs:34-86`):**  
> В существующем коде `create_box` комментарии над гранями называли $+Z$ `Front face`, а $+Y$ — `Top face`, тогда как во всех остальных примитивах (`create_cylinder`, `create_cone`, `create_polygon_prism`, `create_spur_gear`, `create_revolve`) вертикальной осью высоты является ось **$Z$** (`+Z` = верх `top`/`cap_top`, `-Z` = низ `bottom`/`cap_bottom`). Присвоение грани $+Z$ бокса канонического имени `"top"` (с алиасами `"cap_top"`, `"max_z"`, `"+z"`) и грани $+Y$ имени `"back"` (с алиасами `"max_y"`, `"+y"`, `"top_y"`) унифицирует систему координат $Z$-up для всех примитивов и сохраняет 100% обратную совместимость с AABB-именами `min_x..max_z`.

---

## 4. Прохождение идентификаторов через `transform` и CSG (Пункт 3)

### 4.1. Прохождение через пространственные трансформации (`topology.rs:303-368`)

Методы `TriangleMesh::translate`, `rotate_rpy_deg`, `scale`, `transform` модифицируют координаты вершин на месте, **не меняя ни число треугольников, ни их порядок в `indices`**. Поэтому массив `self.face_ids: Vec<u32>` остаётся **неизменным**, а реестр `self.faces: Vec<FaceInfo>` обновляется методом `self.recompute_face_registry_after_transform(...)`:

1. **`translate([dx, dy, dz])` (`topology.rs:303`):**
   - Для каждой `face` в `self.faces`: `centroid` и `sample_point` сдвигаются на `[dx, dy, dz]`; `area`, `normal_or_axis`, `radius`, `triangles_count` не меняются.
2. **`rotate_rpy_deg(roll, pitch, yaw)` (`topology.rs:338`) и `transform(translation, rotation_deg)` (`topology.rs:354`):**
   - Матрица вращения $R \in SO(3)$ применяется к `centroid`, `sample_point` и `normal_or_axis`:
     $$\mathbf{c}' = R\mathbf{c} + \mathbf{t}, \quad \mathbf{p}_{\text{sample}}' = R\mathbf{p}_{\text{sample}} + \mathbf{t}, \quad \mathbf{n}' = R\mathbf{n}$$
   - `area` и `radius` инвариантны к жёсткому повороту и сдвигу.
   - Если в `aliases` присутствовали осевые AABB-алиасы (`"max_z"`, `"+z"` и т.д.), после нетривиального поворота или масштабирования осевые алиасы `min_x..max_z` пересчитываются по фактическому совпадению новой нормали и AABB, тогда как топологические имена (`name`, `role`, `origin_feature`) сохраняются неизменными.
3. **`scale([sx, sy, sz])` (`topology.rs:311`):**
   - При отрицательном определителе $s_x s_y s_z < 0$ (`topology.rs:330`) обход вершин каждого треугольника разворачивается (`tri.swap(1, 2)`), но индекс самого треугольника $t$ в `self.indices` не меняется, поэтому `self.face_ids[t]` остаётся валидным.
   - Для обновления `area`, `centroid`, `sample_point`, `normal_or_axis` и `radius` после произвольного (в т.ч. неравномерного или зеркального) масштабирования вызывается `self.recompute_face_registry()`:
     - `area`, `centroid`, `sample_point` пересчитываются напрямую по трансформированным вершинам треугольников;
     - для `FaceKind::Plane` вектор `normal_or_axis` пересчитывается как площадь-взвешенная внешняя нормаль треугольников грани;
     - для `Cylinder` / `Cone` / `Sphere` при равномерном масштабировании в плоскости сечения ($|s_\perp|$) радиус умножается на $|s_\perp|$, а вектор оси трансформируется и нормализуется.

### 4.2. Прохождение через CSG — Вариант с текущим BSP-деревом (`crates/mcp-cad-core/src/csg.rs`)

Если задача `P2-T7` выполняется до переключения булева ядра на `manifold-rust` (`P1-K1..K2`), прохождение `face_id` через `crates/mcp-cad-core/src/csg.rs` реализуется следующим образом:

1. **Перенумерация `face_id` операндов и объединение таблиц `FaceInfo`:**
   - Перед выполнением `csg_union(a, b)`, `csg_difference(a, b)` или `csg_intersection(a, b)` (`csg.rs:2704-2783`):
     - Операнд $A$ сохраняет свои идентификаторы `face_id` в диапазоне `1..=max_id_a`.
     - Все ненулевые `face_id` операнда $B$ сдвигаются на `offset = max_id_a`: `fid_b_mapped = fid_b + offset`.
     - В объединённый реестр кандидатов `combined_faces: HashMap<u32, FaceInfo>` копируются `a.faces` как есть, а записи из `b.faces` — с `id = f.id + offset`:
       - При **`csg_difference(a, b)`** выжившие грани тела $B$ образуют новые внутренние поверхности реза в теле $A$. Поэтому для каждой грани из `b.faces`:
         - её роль получает префикс `cut:` (если она ещё не начинается с `hole_` или `keyway_`): например, грань `side` цилиндрического инструмента превращается в `cut:<feature_key>:side` (а при вызове из `cut_hole` — `hole_wall` / `hole_wall:<feature_key>`);
         - если `f.kind == FaceKind::Plane`, её внешняя нормаль `normal_or_axis` инвертируется (`[-nx, -ny, -nz]`), так как в `csg_difference` полигоны тела $B$ инвертируются (`a_tree.invert()`, `flip_polygons(&mut b_clipped)` на строках `2749-2751`).
2. **Хранение и наследование `face_id` в `CsgPolygon` (`csg.rs:176-203`):**
   ```rust
   #[derive(Debug, Clone, PartialEq)]
   pub struct CsgPolygon {
       pub vertices: Vec<Vertex>,
       pub plane: Plane,
       pub face_id: u32,
   }
   ```
   - В `BspNode::polygons_from_mesh(mesh: &TriangleMesh)` (`csg.rs:223-302`):
     - Читается `fid_a = mesh.face_ids.get(i).copied().unwrap_or(0)`.
     - Слияние двух соседних копланарных треугольников `i` и `i + 1` в выпуклый квад (строки `234-290`) разрешается **только если `fid_a == fid_b`**! Это гарантирует, что треугольники разных логических граней никогда не сольются в один полигон.
   - В `Plane::split_polygon` (`csg.rs:44-146`):
     - Ветки без разрезания (`coplanar_front`, `coplanar_back`, `front`, `back`) клонируют `poly.clone()`, автоматически сохраняя `poly.face_id`.
     - В ветке `SPANNING` (строки `133-144`) обе отсечённые части `f_verts` и `b_verts` получают `face_id: poly.face_id`.
3. **Сборка треугольников в `polygons_to_mesh` (`csg.rs:498-708`):**
   - Массив `poly_loops` (`csg.rs:560`) и массив треугольников `tris` (`csg.rs:585`) расширяются кортежем с `face_id: u32`:
     `let mut tris: Vec<([u32; 3], Vector3<f64>, u32)> = Vec::new();`
   - В `triangulate_subdivided_convex_loop` (`csg.rs:721`) каждый порождённый треугольник получает `face_id` исходного полигона.
4. **Сохранение `face_id` в фазах ремонта и оптимизации качества (`csg.rs:675-680`):**
   - В `repair_manifold_triangles` (`csg.rs:2489-2695`):
     - При разбиении ребра T-junction (`(u, v)` точкой `w`) оба дочерних треугольника наследуют `face_id` разбиваемого треугольника.
     - При зашивании микро-щели (`tris.push(([u, m, v], cross / len, fid))`, строка `2634`) новый треугольник наследует `face_id` смежного граничного треугольника, содержащего ребро `(v, u)`.
   - В `optimize_csg_mesh_quality` (`csg.rs:808-839`):
     - **`optimize_coplanar_delaunay_edges` (`csg.rs:1017`):** переворот общей диагонали двух копланарных треугольников `(t1, t2)` разрешается **только если `t1.face_id == t2.face_id`** (оба новых треугольника сохраняют этот `face_id`).
     - **`try_remove_vertex_ear_clip` (`csg.rs:1249`) и `collapse_redundant_and_sliver_vertices` (`csg.rs:1521`):** удаление вершины через 1-ring ear-clipping выполняется внутри однородного `face_id`; при Link-Condition схлопывании ребра `u -> v` запрещается перемещать вершину `u`, если это сдвигает границу между двумя различными `face_id` вне общей прямой/кривой пересечения.
     - **`smooth_coplanar_interior_vertices` (`csg.rs:845`):** тангенциальное сглаживание применяется только к вершинам, все инцидентные треугольники которых имеют одинаковый `face_id` (границы между разными `face_id` остаются жёстко зафиксированными).
     - **`refine_coplanar_sliver_fans_and_edges` (`csg.rs:1844`):** при делении треугольника точкой Штейнера дочерние треугольники наследуют `face_id` родителя.
5. **Финальная пересборка реестра `TriangleMesh::recompute_face_registry(&mut self)`:**
   - По массиву `self.face_ids` и кандидатному словарю `combined_faces` за один линейный проход $O(T)$ по треугольникам вычисляются точные `area`, `centroid`, `sample_point` и `triangles_count` для каждого выжившего `face_id`.
   - Грани с `triangles_count == 0` (полностью удалённые булевой операцией, например торцевые крышки сквозного сверла в `cut_hole`) автоматически исключаются из `self.faces`.

### 4.3. Прохождение через CSG — Вариант с `manifold-rust v0.19.2` (`ADR-0001` / `P1-K4`)

При выполнении миграции булева ядра на `manifold-rust v0.19.2` (задачи `P1-K1..P1-K4` из [`agent_prompts/P1_kernel_migration.md`](../agent_prompts/P1_kernel_migration.md)):
1. Перед вызовом `mesh_to_manifold` идентификаторы `b.face_ids` аналогично сдвигаются на `offset = max_id_a`.
2. В `MeshGLP::<f64, u32>` передаётся `face_id: mesh.face_ids.clone()`.
3. Библиотека `manifold-rust` (`face_op_triangulate.rs`, `boolean_result_assemble.rs`) автоматически присваивает каждому результирующему треугольнику точный `face_id` родительского треугольника операнда $A$ или $B$.
4. В `manifold_to_mesh` вектор `mgl.face_id` записывается напрямую в `TriangleMesh::face_ids`, после чего выполняются face-aware `optimize_csg_mesh_quality` и `recompute_face_registry()`.

### 4.4. Адаптер для B-Rep (`monstertruck`, Вариант C из `ADR-0001`)

На случай подключения опционального B-Rep контура (`monstertruck`) во второй очереди:
- Каждая аналитическая грань `Face` в `monstertruck_topology::Shell` при триангуляции (`monstertruck_meshing`) получает последовательный `face_id = (face_idx + 1) as u32`.
- Тип `FaceKind`, `normal_or_axis` и `radius` извлекаются напрямую из аналитической поверхности `Surface` (плоскость, цилиндр, конус, сфера, тор, NURBS) без геометрического угадывания, и записываются в ту же структуру `FaceInfo` внутри `TriangleMesh::faces`. Таким образом, верхнеуровневый контракт `P2-T6..P2-T8` остаётся **полностью неизменным**.

---

## 5. Селекторы граней (`FaceSelector`) и контракты MCP-инструментов (Пункт 4)

### 5.1. Единый формат селектора `FaceSelector`

Для всех инструментов, принимающих ссылку на грань или набор граней, вводится единый JSON-формат селектора, который парсится функцией `resolve_face_selector(mesh: &TriangleMesh, selector: &Value, length_unit: LengthUnit) -> Result<Vec<&FaceInfo>, ServerError>`:

1. **Короткая строковая форма или объект с одиночным именем (`{"face": "<имя_или_алиас>"}` или просто `"<имя>"`):**
   ```json
   {"face": "cap_top"}
   ```
   Поддерживает поиск (без учёта регистра):
   - по точному совпадению с `face.name` (например, `"cap_top"`, `"cut:feat_1:hole_wall"`, `"side_0"`);
   - по совпадению с любым элементом `face.aliases` (например, `"top"`, `"max_z"`, `"+z"`, `"hole_wall"`, `"<uuid>:cap_top"`);
   - по совпадению с `face.role`, если в `mesh.faces` ровно одна грань имеет такую `role` (если граней с ролью `"hole_wall"` несколько, возвращается структурированная ошибка `AMBIGUOUS_FACE_SELECTOR` со списком конкретных имён `["hole_wall:feat_1", "hole_wall:feat_2"]` и их центроидами/радиусами в `details.candidates`);
   - по числовому `{"face_id": 3}` — прямой выбор грани с `face.id == 3`.
2. **Список имён (`{"faces": ["<имя_1>", "<имя_2>", ...]}`):**
   ```json
   {"faces": ["side_0", "side_3"]}
   ```
   Объединяет (без дубликатов по `face.id`) результаты разрешения каждого имени.
3. **Семантическо-геометрический фильтр (`{"where": { ... }}`):**
   Позволяет ИИ-агенту выбрать грани по физическому смыслу, даже не зная их точных имён:
   ```json
   {
     "where": {
       "kind": "cylinder",
       "radius": 5.0,
       "tolerance": 0.1,
       "normal": [0.0, 0.0, 1.0],
       "near": [0.0, 0.0, 25.0],
       "origin_feature": "cut_hole",
       "role": "hole_wall",
       "min_area": 10.0,
       "max_area": 5000.0
     }
   }
   ```
   **Правила фильтрации в `where` (все поля опциональны, объединяются по `AND`):**
   - `kind`: `"plane" | "cylinder" | "cone" | "sphere" | "torus" | "helical" | "other"`;
   - `radius`: ожидаемый радиус в активных `length_unit` проекта (`mm` по умолчанию), сравнивается с `face.radius` с допуском `tolerance` (по умолчанию `max(0.05 * radius, 1e-4 м)`);
   - `normal` / `axis`: вектор `[x, y, z]`; для `Plane` проверяется `dot(unit(normal), face.normal_or_axis) >= 1.0 - angle_tol` (направленное совпадение), для `Cylinder`/`Cone`/`Helical` проверяется коллинеарность осей `|dot(unit(axis), face.normal_or_axis)| >= 1.0 - angle_tol`;
   - `near`: точка `[x, y, z]` в активных `length_unit`; выбирает грани, у которых `distance(face.centroid, near) <= tolerance` или (если задан флаг `"closest": true`) единственную ближайшую грань среди прошедших остальные фильтры;
   - `origin_feature`: подстрока или UUID породившего узла DAG;
   - `role`: точное совпадение локальной роли (`"cap_top"`, `"hole_wall"`, `"side"`);
   - `min_area`, `max_area`: фильтр по площади грани в квадратных единицах проекта (`mm²` или `m²`).

### 5.2. Какие MCP-инструменты получают поддержку именованных граней и селекторов

#### 1. `query_part_geometry` (`crates/mcp-cad-server/src/tools/cad.rs:1835`)
- **Вход:** добавляется опциональный параметр `face_selector` (или `where`) для фильтрации интересующих граней.
- **Выход:** в JSON-ответ добавляется массив `"faces"` (все размерные поля масштабированы в активный `length_unit` согласно `P2-T2`), при **полном сохранении** существующего поля `"detected_planar_faces"` для обратной совместимости:
  ```json
  {
    "part_id": "part-1",
    "faces": [
      {
        "id": 1,
        "name": "cap_top",
        "aliases": ["top", "max_z", "+z", "feat_0:cap_top"],
        "role": "cap_top",
        "kind": "plane",
        "origin_feature": "550e8400-e29b-41d4-a716-446655440000",
        "area": 392.0686,
        "area_m2": 0.0003920686,
        "centroid": [0.0, 0.0, 25.0],
        "sample_point": [12.35, 1.22, 25.0],
        "normal_or_axis": [0.0, 0.0, 1.0],
        "radius": 15.0,
        "triangles_count": 192
      },
      {
        "id": 4,
        "name": "hole_wall:feat_1",
        "aliases": ["hole_wall", "bore", "cut:feat_1:hole_wall"],
        "role": "hole_wall",
        "kind": "cylinder",
        "origin_feature": "660e8400-e29b-41d4-a716-446655440111",
        "area": 3140.55,
        "area_m2": 0.00314055,
        "centroid": [0.0, 0.0, 0.0],
        "sample_point": [10.0, 0.0, 1.56],
        "normal_or_axis": [0.0, 0.0, -1.0],
        "radius": 10.0,
        "triangles_count": 512
      }
    ],
    "detected_planar_faces": [ ... ],
    "length_unit": "mm",
    "units": { ... }
  }
  ```

#### 2. `run_fea_analysis` (`crates/mcp-cad-server/src/tools/fea.rs:37` и `crates/mcp-cad-fea/src/boundary.rs:44-205`)
- **Новые входные параметры (с полным сохранением старых `fixed_faces`, `fixed_plane`, `applied_forces`):**
  - `fixed`: массив селекторов граней и/или ограничений степеней свободы:
    ```json
    "fixed": [
      {"face": "cap_bottom"},
      {"where": {"kind": "cylinder", "radius": 10.0}, "components": "all"}
    ]
    ```
    Кроме того, в существующем `fixed_faces: ["cap_bottom", "min_x"]` строки сначала проверяются через `parse_aabb_face`, а если строка не является AABB-гранью — разрешаются как имя грани из `part.mesh.faces`!
  - `loads`: массив нагрузок, привязанных к именованным граням:
    - суммарная сила `[Fx, Fy, Fz]` в Ньютонах:
      `{"face": "cap_top", "force": [0.0, 0.0, -5000.0]}`
    - нормальное давление `pressure` в МПа (`MPa`, положительное — сжатие внутрь грани вдоль `-n`, отрицательное — растяжение вдоль `+n`):
      `{"face": "hole_wall", "pressure": 15.0}`
    - крутящий/изгибающий момент `moment` в `N·m` относительно центроида грани вдоль её оси:
      `{"face": "cap_top", "moment": [0.0, 0.0, 50.0]}`
- **Алгоритм сопоставления именованной грани `TriangleMesh` с узлами воксельной сетки `TetrahedralMesh` (для `P2-T8`):**
  - Поскольку `TetrahedralMesh::from_mesh` (`crates/mcp-cad-fea/src/tetra_mesh.rs:320`) строит ступенчатую воксельную сетку с шагом ячейки $h_{\text{voxel}}$, поверхностные узлы тетра-сетки отстоят от исходных треугольников грани `TriangleMesh` на расстояние до $\frac{\sqrt{3}}{2} h_{\text{voxel}} \approx 0.866\,h_{\text{voxel}}$.
  - Для выбранной грани `face_id` собираются все её треугольники $T_f = \{ (A_k, B_k, C_k) \mid \text{mesh.face\_ids}[k] == \text{face\_id} \}$.
  - Из `tet_mesh.extract_surface_triangles()` (`tetra_mesh.rs:208`) извлекаются граничные узлы тетра-сетки $N_{\text{surf}}$ и их грузовые площади (tributary areas $w_i = \sum_{e \ni i} \frac{1}{3} S_e$).
  - Узел $i \in N_{\text{surf}}$ с координатой $\mathbf{p}_i$ включается в набор узлов грани, если минимальное евклидово расстояние от $\mathbf{p}_i$ до треугольников $T_f$ не превышает допуск:
    $$\min_{k \in T_f} \text{dist}(\mathbf{p}_i, \triangle A_k B_k C_k) \le \tau, \quad \tau = \max(0.95 \cdot h_{\text{voxel}},\ 10^{-4}\text{ м})$$
  - При распределении суммарной силы $\mathbf{F}$ по найденным узлам $i \in M_f$ веса нормируются пропорционально грузовым площадям $w_i$:
    $$\mathbf{f}_i = \mathbf{F} \cdot \frac{w_i}{\sum_{j \in M_f} w_j} \implies \left\|\sum_{i \in M_f} \mathbf{f}_i - \mathbf{F}\right\| \le 10^{-9} \|\mathbf{F}\|$$
  - Если селектор не совпал ни с одной гранью в `part.mesh.faces` — возвращается структурированная ошибка `FACE_NOT_FOUND` со списком всех доступных граней детали в `details.available_faces` и подсказкой `hint`. Если грань существует, но ни один узел `tet_mesh` не попал в допуск $\tau$ — возвращается `FEA_NO_BOUNDARY_NODES` с рекомендацией повысить `mesh_density` до `"medium"`/`"fine"`.

#### 3. `create_joint` (`crates/mcp-cad-server/src/tools/physics.rs:116`)
- **Новые опциональные параметры:** `face_a` и `face_b` (принимающие `FaceSelector`).
- **Поведение:**
  - Если передан `face_a` для детали `part_a` (не `"ground"`), а явный `anchor_a` не задан, то `anchor_a` автоматически берётся равным `face_a.centroid` (в локальных координатах детали), а если не задан `axis` — `axis` берётся равным `face_a.normal_or_axis`.
  - Аналогично `face_b` автоматически вычисляет `anchor_b = face_b.centroid` (и `axis_b = face_b.normal_or_axis` для `gear`).
  - Это позволяет агенту собирать вал с шестернёй или кривошип с шатуном без ручного расчёта координат центров отверстий.

#### 4. `transform_part` (`crates/mcp-cad-server/src/tools/cad.rs:2069`)
- **Новые опциональные параметры:**
  - `pivot_face`: `FaceSelector` — если указан, поворот `rotation_deg` и масштабирование `scale` выполняются вокруг центроида выбранной грани `face.centroid` (с компенсацией сдвига $\mathbf{t}_{\text{eff}} = \mathbf{c} - R S \mathbf{c} + \mathbf{t}$).
  - `align_face`: `{"face": "<имя>", "to_point": [x, y, z], "to_normal": [nx, ny, nz]}` — автоматически вычисляет поворот RPY и вектор переноса так, чтобы внешняя нормаль/ось выбранной грани совпала с `to_normal` (или антипараллельно при `"mate": true`), а её центроид переместился в `to_point`.

---

## 6. Разбивка реализации на задачи `P2-T6`, `P2-T7`, `P2-T8` и карта изменений кода (Пункт 5)

### 6.1. Верификация и уточнение декомпозиции задач `P2-T6..P2-T8`

Разбивка на три последовательные задачи из `fix/P2_agent_contract.md` **подтверждается** со следующими техническими уточнениями:

| Задача | Размер | Зависимости | Область работ и уточнения по итогам аудита кода |
|---|---|---|---|
| **`P2-T6`: Именованные грани: примитивы** | **M** | `P2-T5` | 1. Добавить `FaceKind`, `FaceInfo`, поля `face_ids: Vec<u32>` и `faces: Vec<FaceInfo>` (`#[serde(default, skip_serializing_if = "Vec::is_empty")]`) в `TriangleMesh` (`crates/mcp-cad-core/src/topology.rs:150`).<br>2. Реализовать `TriangleMesh::recompute_face_registry(&mut self)` и обновление `faces` в `translate`, `rotate_rpy_deg`, `scale`, `transform` (`topology.rs:303-368`).<br>3. Заполнить `face_ids` и `faces` в 5 обязательных генераторах `crates/mcp-cad-core/src/primitives.rs`: `create_box` (`:13`), `create_cylinder` (`:322`), `create_cone` (`:810`), `create_sphere` (`:383`), `create_polygon_prism` (`:1111`). Для `create_spur_gear`, `create_wheel_rim`, `create_revolve` заполнять `face_ids = vec![0; indices.len()]`.<br>4. В `FeatureTree::evaluate_with_settings` (`crates/mcp-cad-core/src/dag.rs:367`) проставлять `origin_feature = Some(node.id.to_string())` и квалифицированные алиасы.<br>5. В `query_part_geometry` (`crates/mcp-cad-server/src/tools/cad.rs:1835`) вывести массив `"faces"` с конвертацией единиц (`k_from_m`, `k_area`) и поддержкой базового `FaceSelector`. |
| **`P2-T7`: Именованные грани: проход через CSG и transform** | **L** *(на BSP)* или **S/M** *(на `manifold-rust` после `P1-K1..K2`)* | `P2-T6`, `P0-T7` | 1. В `cut_hole_with_settings` (`primitives.rs:588`) и `build_keyway_cutter_prism` (`:682`) разметить грани инструментального тела (`hole_wall`, `hole_bottom`, `keyway_wall`).<br>2. В `csg_union`, `csg_difference`, `csg_intersection` (`crates/mcp-cad-core/src/csg.rs:2704-2783`) реализовать сдвиг `face_id` операнда $B$ на `max_id_a`, префикс `cut:` (и инверсию нормали плоских граней $B$ для `csg_difference`) и пронос `face_id` сквозь `CsgPolygon`/`split_polygon`/`polygons_to_mesh` (либо через `MeshGLP::face_id` в `manifold-rust`).<br>3. В `optimize_csg_mesh_quality` (`csg.rs:808`) запретить флипы рёбер и сглаживание поперёк границ разных `face_id`.<br>4. Вызывать `recompute_face_registry()` на выходе CSG.<br>5. В `transform_part` (`tools/cad.rs:2069`) добавить опциональные селекторы `pivot_face` / `align_face`. |
| **`P2-T8`: FEA: граничные условия по именам граней** | **M** | `P2-T7` | 1. В `crates/mcp-cad-fea/src/boundary.rs` добавить варианты `DirichletConstraint::FixedSurfaceTriangles { triangles: Vec<[[f64; 3]; 3]>, tolerance: f64, components: DofComponent }` и `NeumannLoad::SurfaceTrianglesForce / Pressure / Moment` с расчётом расстояния от поверхностных узлов `TetrahedralMesh` до треугольников грани и распределением сил пропорционально грузовым площадям ($\sum \mathbf{f}_i = \mathbf{F}$ с точностью $10^{-9}$).<br>2. В `crates/mcp-cad-fea/src/tetra_mesh.rs:629` (`refine_mesh_max_edge_length`) наследовать `face_ids` при 1-к-4 подразделении.<br>3. В `run_fea_analysis` (`crates/mcp-cad-server/src/tools/fea.rs:37`) поддержать параметры `fixed` и `loads` с `FaceSelector` (`{"face": ...}`, `{"faces": ...}`, `{"where": ...}`), а также имена граней внутри `fixed_faces`, возвращая структурированные ошибки `FACE_NOT_FOUND` / `AMBIGUOUS_FACE_SELECTOR` (`crates/mcp-cad-server/src/error.rs`). |

### 6.2. Полная таблица затрагиваемых файлов и структур кода

| Крейт / Файл | Строки | Затрагиваемые структуры и функции | Характер изменений |
|---|---|---|---|
| `crates/mcp-cad-core/src/topology.rs` | `116–176`, `303–368` | `FaceKind`, `FaceInfo`, `TriangleMesh`, `GeometryInspection`, `translate`, `scale`, `rotate_rpy_deg`, `transform` | Добавление типов `FaceKind`, `FaceInfo`, полей `face_ids` и `faces` в `TriangleMesh`, методов `with_faces`, `recompute_face_registry`, обновление метаданных граней при трансформациях |
| `crates/mcp-cad-core/src/primitives.rs` | `13–89`, `229–370`, `383–497`, `588–803`, `810–1087`, `1111–1464` | `create_box`, `add_subdivided_disk_cap_2d`, `create_cylinder`, `create_sphere`, `cut_hole_with_settings`, `build_keyway_cutter_prism`, `create_cone`, `create_polygon_prism` | Заполнение `face_ids` и начального реестра `faces` при генерации каждого примитива и инструментального тела `cut_hole` |
| `crates/mcp-cad-core/src/dag.rs` | `245–255`, `367–712` | `apply_pos_rot`, `FeatureTree::evaluate_with_settings` | Привязка `origin_feature = Some(node.id.to_string())`, генерация квалифицированных имён и алиасов `<feature_key>:<role>` |
| `crates/mcp-cad-core/src/csg.rs` | `44–146`, `176–302`, `498–708`, `808–1860`, `2489–2783` | `Plane::split_polygon`, `CsgPolygon`, `BspNode::polygons_from_mesh`, `polygons_to_mesh`, `optimize_csg_mesh_quality`, `repair_manifold_triangles`, `csg_union`, `csg_difference`, `csg_intersection` | Пронос `face_id: u32` сквозь полигоны, запрет слияния и флипов между разными `face_id`, перенумерация `face_id` операнда $B$, префикс `cut:` для `csg_difference`, пересборка `faces` |
| `crates/mcp-cad-core/src/import_stl.rs` | `268–315` | `weld_vertices` | Сохранение `face_ids` при фильтрации невырожденных треугольников |
| `crates/mcp-cad-fea/src/tetra_mesh.rs` | `208–235`, `629–688` | `extract_surface_triangles`, `refine_mesh_max_edge_length` | Наследование `face_ids[t]` для 4 дочерних треугольников при 1-к-4 подразделении сетки |
| `crates/mcp-cad-fea/src/boundary.rs` | `44–120`, `205–451` | `DirichletConstraint`, `NeumannLoad`, `BoundaryConditions::resolve` | Новые варианты граничных условий по поверхностным треугольникам именованной грани с вычислением грузовых площадей узлов |
| `crates/mcp-cad-server/src/tools/cad.rs` | `1835–1888`, `2069–2149` | `query_part_geometry`, `transform_part` | Вывод `"faces"` в активных единицах, парсер `FaceSelector`, опции `pivot_face` / `align_face` |
| `crates/mcp-cad-server/src/tools/fea.rs` | `24–213` | `run_fea_analysis` | Поддержка `fixed: [FaceSelector]` и `loads: [...]` (силы, давления, моменты по именам граней) |
| `crates/mcp-cad-server/src/tools/physics.rs` | `116–290` | `create_joint` | Опциональные селекторы `face_a` / `face_b` для автоопределения `anchor_a`, `anchor_b`, `axis` |
| `crates/mcp-cad-server/src/error.rs` | `1–350` | `ServerError::to_error_body` | Новые стабильные коды ошибок `FACE_NOT_FOUND` и `AMBIGUOUS_FACE_SELECTOR` с перечислением доступных граней в `details.available_faces` |
| `crates/mcp-cad-server/src/tools/mod.rs` | `17–950` | `list_tools` | Обновление JSON Schema для `query_part_geometry`, `run_fea_analysis`, `create_joint`, `transform_part` |

---

## 7. Анализ рисков и способы их минимизации

1. **Риск №1 — Рост потребления оперативной памяти и размера JSON:**
   - *Оценка в ОЗУ:* массив `face_ids: Vec<u32>` занимает ровно **4 байта на треугольник**, а одна запись `FaceInfo` — около **128 байт**. Для типичной детали из `10 000` треугольников и `20` логических граней дополнительный расход памяти составляет $10\,000 \times 4 + 20 \times 128 \approx \mathbf{42.5\text{ КБ}}$ (менее `7%` от размера буферов `vertices` и `indices`, занимающих `~600 КБ`).
   - *Оценка в файлах проектов:* как реализовано в `P2-T4` (`crates/mcp-cad-server/src/tools/project.rs:156`), `save_project` сохраняет только параметрический DAG (`mesh: None`), поэтому размер файлов проектов `.json` на диске **увеличится на 0 байт**.
2. **Риск №2 — Влияние на производительность булевых операций CSG:**
   - *Оценка:* копирование `u32` при `split_polygon` и проверка `fid_a == fid_b` в `polygons_from_mesh` и `optimize_coplanar_delaunay_edges` являются операциями сравнения регистров `u32` ($O(1)$). Финальный пересчёт `recompute_face_registry` требует ровно одного линейного прохода по треугольникам ($< 0.05\text{ мс}$ для `10 000` треугольников).
   - Более того, запрет слияния и переворота рёбер между разными `face_id` сокращает число ложных кандидатов в `optimize_coplanar_delaunay_edges`. Ожидаемое изменение времени `csg_*` составляет **`< 2–3%`** (значительно ниже максимально допустимого порога `20%` из критериев приёмки `P2-T7`).
3. **Риск №3 — Пропуск узлов ступенчатой воксельной сетки `TetrahedralMesh` на криволинейных гранях в `P2-T8`:**
   - *Проблема:* воксельная сетка `TetrahedralMesh::from_mesh` (`tetra_mesh.rs:320`) аппроксимирует круглое отверстие ступеньками. Если использовать жёсткий допуск `1e-4 м` (как для плоских граней AABB), то на грубой сетке (`Coarse`) большинство ступенчатых узлов стенки отверстия окажутся дальше `0.1 мм` от идеального цилиндра.
   - *Минимизация:* в `P2-T8` допуск сопоставления узла тетра-сетки с треугольниками грани `face_id` вычисляется адаптивно от фактического шага воксельной ячейки $h_{\text{voxel}} = \max(\Delta x, \Delta y, \Delta z)$:
     $$\tau = \max\left(0.95 \cdot \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2} \cdot 0.5,\ \text{user\_tolerance}\right)$$
     и проверяется только по внешним граничным узлам тетра-сетки (`extract_surface_triangles()`), чтобы не захватывать внутренние объёмные узлы.
4. **Риск №4 — Размытие границ `face_id` при агрессивном схлопывании рёбер в `optimize_csg_mesh_quality`:**
   - *Проблема:* если вершина на границе между `cap_top` и `hole_wall` схлопнется внутрь плоской крышки, треугольник стенки отверстия ляжет в плоскость крышки.
   - *Минимизация:* в `collapse_redundant_and_sliver_vertices` (`csg.rs:1521`) вершина $u$ с множеством инцидентных идентификаторов граней $F(u)$ может схлопываться в вершину $v$ только при условии $F(u) \subseteq F(v)$ (внутренняя вершина грани может схлопываться на границу этой же грани, а вершина на стыке двух граней — только вдоль того же стыка $F(u) = F(v)$).

---

## 8. Архитектурные развилки (`[DECISION]`) и выбранные варианты по умолчанию

1. **`[DECISION-FACES-1]` Хранение именованных граней: меш-слой `TriangleMesh` vs ожидание полного B-Rep ядра:**
   - **Рекомендация по умолчанию (принята в данном дизайне):** Реализовать слой `face_ids: Vec<u32>` + `faces: Vec<FaceInfo>` непосредственно в `TriangleMesh` (`crates/mcp-cad-core/src/topology.rs`). Это на 100% совместимо как с текущим BSP-CSG, так и с `manifold-rust v0.19.2` (`MeshGLP::face_id`, выбранным в `ADR-0001`), не ломает существующие 184 теста и сразу разблокирует задачи `P2-T6`, `P2-T7`, `P2-T8` и `P3-T7`.
2. **`[DECISION-FACES-2]` Соглашение об именах граней `create_box` (`+Z` vs `+Y`):**
   - **Рекомендация по умолчанию:** Привести канонические роли `create_box` к единой для всего ядра системе координат **$Z$-up** (`+Z` $\to$ `"top"`, `-Z` $\to$ `"bottom"`, `+X` $\to$ `"right"`, `-X` $\to$ `"left"`, `+Y` $\to$ `"back"`, `-Y` $\to$ `"front"`), добавив в `aliases` как `"cap_top"`/`"cap_bottom"`, так и однозначные осевые имена `"max_z"`/`"min_z"`/`"max_x"`/`"min_x"`/`"max_y"`/`"min_y"`.
3. **`[DECISION-FACES-3]` Поведение при неоднозначном коротком имени роли (например, `{"face": "hole_wall"}` на детали с двумя отверстиями):**
   - **Рекомендация по умолчанию:** Если в `mesh.faces` ровно одна грань имеет `role == "hole_wall"`, короткий селектор `{"face": "hole_wall"}` выбирает её. Если таких граней $\ge 2$, возвращать структурированную ошибку `AMBIGUOUS_FACE_SELECTOR` (не угадывать молча первую попавшуюся!), перечисляя в `details.candidates` полные квалифицированные имена всех кандидатов (`"hole_wall:feat_1"`, `"hole_wall:feat_2"`) с их координатами и радиусами, либо предлагать использовать `{"faces": [...]}` / `{"where": {"role": "hole_wall", "near": [...]}}`.
4. **`[DECISION-FACES-4]` Порядок выполнения `P2-T6..P2-T7` относительно миграции ядра `P1-K1..P1-K4`:**
   - **Рекомендация по умолчанию:** В основном списке `TASKS.md` задачи `P2-T6`, `P2-T7`, `P2-T8` могут выполняться последовательно прямо по текущему дереву `crates/mcp-cad-core` (по описанному в разделе 4.2 алгоритму для `csg.rs`), а при запуске пакета `agent_prompts/P1_kernel_migration.md` структуры `FaceInfo` и `face_ids` бесшовно подключаются к `MeshGLP::face_id`.
