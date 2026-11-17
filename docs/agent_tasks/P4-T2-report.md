# Отчёт о выполнении задачи P4-T2

- **Задача**: `P4-T2` — Собственная модель твёрдого тела (`f64`, точный тензор инерции из `Part` с учётом полостей) и структуры коллайдеров в `rigid_body.rs` (замена `Rapier` `RigidBodySet`/`ColliderSet`)
- **Статус**: ✅ ВЫПОЛНЕНО (`16.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-physics/src/rigid_body.rs`](../../crates/mcp-cad-physics/src/rigid_body.rs) *(полный перевод на собственные `f64`-структуры `RigidBody`, `MassProperties`, `BodyHandle`, `BodyRegistry`, `Collider`, `ColliderShape`, `ColliderHandle`, `ColliderRegistry`; `0` импортов `rapier3d`)*
  - [`crates/mcp-cad-physics/src/lib.rs`](../../crates/mcp-cad-physics/src/lib.rs) *(экспорт `BodyHandle`, `BodyRegistry`, `Collider`, `ColliderHandle`, `ColliderRegistry`, `ColliderShape`, `ColliderType`, `ConvexHullPart`, `MassProperties`, `RigidBody`, `compute_mesh_mass_properties`)*
  - [`crates/mcp-cad-physics/src/simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs) *(перевод `PhysicsWorld::bodies` и `PhysicsWorld::colliders` на `BodyRegistry` и `ColliderRegistry` в `f64`, чтение точных масс, ЦМ и тензоров инерции `f64` в `extract_part_load_case` и `get_world_pose_vectors`, сохранение `BodyHandle` и шарниров при `update_part_collider`)*
  - [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs) *(интеграционный тест `test_p4_t2_f64_rigid_body_hollow_inertia_and_update_part_collider` и адаптация вызовов `add_part_with_options`/`update_part_collider`)*
  - [`vite.config.ts`](../../vite.config.ts) *(добавлен `server.watch.ignored` для `target/**`, `crates/**`, `spikes/**` во избежание блокировки Vite dev-сервера при сборке Rust)*
  - [`TASKS.md`](../../TASKS.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md)

---

## 1. Контекст и постановка задачи

До выполнения `P4-T2` модуль `crates/mcp-cad-physics/src/rigid_body.rs` представлял собой тонкую обёртку над типами игрового движка `rapier3d = "0.22"` в одинарной точности `f32` (`RigidBodyBuilder`, `ColliderBuilder`, `SharedShape::convex_hull`). При этом:
1. Тензор инерции и положение центра масс динамических тел рассчитывались внутри `Rapier3D` в `f32` по выпуклой оболочке (`ConvexHull`) или упрощённой аппроксимации, **игнорируя сквозные отверстия, расточки, шпоночные пазы и внутренние полости** детали `Part`.
2. Все координаты, кватернионы, скорости и массовые характеристики хранились в `RigidBodySet` / `ColliderSet` с потерей точности до `f32`.

В соответствии с архитектурным решением **ADR-004** (`docs/DECISIONS.md`), задача `P4-T2` переводит слой твёрдых тел и геометрии коллайдеров на собственные структуры двойной точности `f64` на базе `nalgebra` без единой зависимости от `rapier3d` в `rigid_body.rs`.

---

## 2. Математическая модель и структуры данных (`crates/mcp-cad-physics/src/rigid_body.rs`)

### 2.1. Точное интегрирование массы, центра масс и 3×3 тензора инерции по замкнутой сетке (`MassProperties`)
Для произвольной водонепроницаемой триангулированной границы $\partial\Omega = \bigcup_{k=1}^{N_T} \triangle(\mathbf{v}_{k,0}, \mathbf{v}_{k,1}, \mathbf{v}_{k,2})$ с плотностью материала $\rho$ реализовано двухпроходное интегрирование по знаковым тетраэдрам в `f64`:

1. **Проход 1 (объём $V$ и центр масс $\mathbf{c}_{\text{loc}}$)**:
   $$\Delta V_k = \frac{1}{6}\,\mathbf{v}_{k,0}\cdot(\mathbf{v}_{k,1} \times \mathbf{v}_{k,2}), \quad V = \sum_{k=1}^{N_T} \Delta V_k, \quad \mathbf{c}_{\text{loc}} = \frac{1}{4V}\sum_{k=1}^{N_T} \Delta V_k (\mathbf{v}_{k,0} + \mathbf{v}_{k,1} + \mathbf{v}_{k,2}), \quad m = \rho V.$$
2. **Проход 2 (точные квадратичные моменты второго порядка в системе координат с началом в $\mathbf{c}_{\text{loc}}$)**:
   Для снижения ошибок округления при больших смещениях детали относительно начала координат вершины каждого треугольника сначала сдвигаются в локальный центр масс $\tilde{\mathbf{v}}_{k,m} = \mathbf{v}_{k,m} - \mathbf{c}_{\text{loc}}$ ($m \in \{0,1,2\}$), после чего вычисляется точная каноническая ковариационная матрица симплекса:
   $$C^{(k)}_{ij} = \frac{\Delta V_k}{20}\left(\sum_{m=0}^{2} \tilde{v}_{k,m,i}\,\tilde{v}_{k,m,j} + \Big(\sum_{m=0}^{2} \tilde{v}_{k,m,i}\Big)\Big(\sum_{n=0}^{2} \tilde{v}_{k,n,j}\Big)\right), \quad C = \sum_{k=1}^{N_T} C^{(k)}.$$
   Поскольку для внутренних полостей и сквозных отверстий нормали треугольников направлены внутрь полости (внешне относительно металла детали), их знаковые объёмы $\Delta V_k < 0$ **автоматически вычитают массу и инерцию пустот** из интеграла по внешнему контуру.
   Итоговый симметричный положительно определённый тензор инерции относительно центра масс:
   $$I_{\text{loc}} = \rho\left(\text{tr}(C)\,I_3 - C\right) \in \mathbb{R}^{3\times 3}, \quad I_{\text{loc}}^{-1} = (I_{\text{loc}})^{-1}.$$

### 2.2. Твёрдое тело `RigidBody` и реестр `BodyRegistry`
- `BodyHandle(pub usize)` — стабильный generational/slot-индекс твёрдого тела.
- `RigidBody`:
  - `part_id: String`, `is_dynamic: bool`, `is_fixed: bool`;
  - `mass: f64`, `inv_mass: f64` (`0.0` для статических тел);
  - `local_com: Vector3<f64>`, `local_inertia: Matrix3<f64>`, `local_inv_inertia: Matrix3<f64>`;
  - `position: Vector3<f64>` (мировое положение начала локальной СК детали), `orientation: UnitQuaternion<f64>`;
  - `linvel: Vector3<f64>` (линейная скорость центра масс в мировой СК), `angvel: Vector3<f64>` (угловая скорость в мировой СК);
  - `force_accum: Vector3<f64>`, `torque_accum: Vector3<f64>` (аккумуляторы внешних сил и моментов относительно ЦМ);
  - `colliders: Vec<ColliderHandle>`, `sleeping: bool`;
  - методы `world_com()`, `set_world_com()`, `world_inertia()`, `world_inv_inertia()`, `velocity_at_point()`, `apply_force()`, `apply_force_at_point()`, `clear_forces()`, `update_mass_properties_from_part()`.
- `BodyRegistry` — детерминированный контейнер со слотами `Option<RigidBody>` и списком свободных слотов `free_list`, поддерживающий `insert`, `get`, `get_mut`, `remove`, `iter`, `iter_mut`.

### 2.3. Коллайдеры `ColliderShape`, `Collider` и `ColliderRegistry`
- `ColliderHandle(pub usize)` — стабильный дескриптор коллайдера.
- `ColliderShape` (`f64`) поддерживает все 6 вариантов `ColliderType`:
  1. `ConvexHull { points: Vec<[f64; 3]>, triangles: Vec<[u32; 3]>, local_aabb }` — 3D выпуклая оболочка методом Quickhull в `f64` с дедупликацией вершин и внешне ориентированными гранями;
  2. `TriMesh { vertices: Vec<[f64; 3]>, indices: Vec<[u32; 3]>, local_aabb }` — точная полигональная сетка для невыпуклых тел с отверстиями, пазами и зубьями шестерён;
  3. `Compound { parts: Vec<ConvexHullPart>, local_aabb }` — декомпозиция невыпуклой сетки по 8 октантам относительно центроида на семейство выпуклых оболочек `ConvexHullPart`;
  4. `Cube { half_extents: [f64; 3], center_offset: [f64; 3] }` — аналитический параллелепипед;
  5. `Sphere { radius: f64, center_offset: [f64; 3] }` — аналитическая сфера;
  6. `Cylinder { half_height: f64, radius: f64, center_offset: [f64; 3] }` — аналитический цилиндр вдоль локальной оси $Z$.
- Для каждого варианта `ColliderShape` реализовано аналитическое вычисление мирового габаритного бокса `compute_world_aabb(position, orientation)` через опорные функции (support mappings).
- Метод `PhysicsWorld::update_part_collider(&mut self, part: &Part)` обновляет `ColliderShape`, массу, локальный ЦМ, тензор инерции $I_{\text{loc}}$ и мировой AABB непосредственно на существующем `BodyHandle` без удаления тела и без разрыва подключённых шарниров.

---

## 3. Результаты верификации на эталонных телах

1. **Прямоугольный параллелепипед (`0.20 × 0.04 × 0.02 м`, сталь $\rho = 7850\text{ кг/м}^3$)**:
   - Масса: $m = 1.256000000000\text{ кг}$ (относительная ошибка к аналитике `< 1e-12`);
   - Тензор инерции относительно смещённого ЦМ $[0.10, -0.05, 0.03]\text{ м}$:
     - $I_{xx} = 2.093333333333 \times 10^{-4}\text{ кг}\cdot\text{м}^2$ (ошибка `< 1e-15`),
     - $I_{yy} = 4.228666666667 \times 10^{-3}\text{ кг}\cdot\text{м}^2$ (ошибка `< 1e-15`),
     - $I_{zz} = 4.354266666667 \times 10^{-3}\text{ кг}\cdot\text{м}^2$ (ошибка `< 1e-15`);
   - Внедиагональные элементы $|I_{ij}| < 10^{-16}$, невязка обращения $\|I_{\text{loc}} I_{\text{loc}}^{-1} - I_3\|_\infty < 10^{-13}$.
2. **Полая стальная втулка / маховик со сквозной расточкой ($R_{\text{out}} = 50\text{ мм}$, $R_{\text{in}} = 30\text{ мм}$, $h = 40\text{ мм}$, 128 сегментов)**:
   - Отношение масс полого и сплошного цилиндров: $m_{\text{hollow}} / m_{\text{solid}} = 0.6400$ (аналитика $1 - (30/50)^2 = 0.64$);
   - Полярный момент инерции полой втулки $I_{zz} = \frac{1}{2} m_{\text{hollow}}(R_{\text{out}}^2 + R_{\text{in}}^2)$ и экваториальные моменты $I_{xx} = I_{yy} = \frac{1}{12} m_{\text{hollow}}(3(R_{\text{out}}^2 + R_{\text{in}}^2) + h^2)$ совпадают с аналитикой в пределах погрешности полигональной аппроксимации окружности (`< 0.2%`), причём удельный радиус инерции $(I_{zz}/m)_{\text{hollow}} > (I_{zz}/m)_{\text{solid}}$, что доказывает честный учёт центральной полости (в отличие от `ConvexHull`).

---

## 4. Обнаруженные смежные проблемы (занесены в `TASKS.md`)

1. **`P4-T9 [S]` (`crates/mcp-cad-core/src/part.rs:162-167`, рассинхронизация `part.mesh` и `part.cached_mesh`)**:
   - Поле `Part::mesh` является публичным (`pub mesh: TriangleMesh`), тогда как метод `Part::mesh(&self)` возвращает `self.cached_mesh.as_ref().unwrap_or(&self.mesh)`. Если вызывающий код после `part.rebuild()` напрямую модифицирует `part.mesh.vertices` без вызова `part.invalidate_cache()` (как в тестах `test_p3_t6_static_cantilever_beam_load_case_and_newton_third_law` и `test_p3_t6_rotating_rod_centripetal_reaction_force_in_local_frame`), методы `part.volume()`, `part.center_of_mass()` и `part.bounding_box()` продолжают читать устаревший `cached_mesh`. В `rigid_body.rs` (`MassProperties::from_part`, `Collider::from_part`) добавлено прямое чтение `&part.mesh`, а задача синхронизации `Part::mesh()` занесена в `TASKS.md` как `P4-T9`.

---

## 5. Верификация

1. `cargo test -p mcp-cad-physics --no-fail-fast`: **32/32 PASS** (`7/7` unit-тестов в `lib` + `25/25` интеграционных тестов в `tests/physics_tests.rs`, включая 3 новых unit-теста `rigid_body::tests` и интеграционный тест `test_p4_t2_f64_rigid_body_hollow_inertia_and_update_part_collider`, `0 failed`).
2. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
3. `npm run lint` (`tsc --noEmit`): **PASS** (`0` ошибок).
4. `npm run build` (`vite build`): **PASS**.
5. `npm test`: **52/52 PASS**.
