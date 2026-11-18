# Отчёт о выполнении задачи P4-T4

- **Задача**: `P4-T4` — Собственный конвейер коллизий `f64` (AABB broad-phase, GJK/EPA + TriMesh BVH narrow-phase, контактный решатель с кулоновским трением и события `CollisionRecord`)
- **Статус**: ✅ ВЫПОЛНЕНО (`18.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-physics/src/collision.rs`](../../crates/mcp-cad-physics/src/collision.rs) *(новый чистый Rust-модуль `f64`-детекции столкновений: Sweep-and-Prune broad-phase с фильтрацией пар, кэшируемое двоичное дерево `TriMeshBvh`, аналитические тесты `Sphere`–`Sphere`, `Sphere`–`Cube`, `Sphere`–`Cylinder`, 15-осевой SAT + клиппинг Сазерленда–Ходжмана `Cube`–`Cube`, универсальный `f64` GJK + EPA и извлечение планарных контактных многообразий для `ConvexHull`, `Cylinder`, `Compound` и `TriMesh`; `0` импортов `rapier3d`/`parry3d`)*
  - [`crates/mcp-cad-physics/src/rigid_body.rs`](../../crates/mcp-cad-physics/src/rigid_body.rs) *(добавлено поле `Collider::tri_bvh: Option<TriMeshBvh>`, автоматически собираемое в `Collider::from_part` для `ColliderShape::TriMesh`)*
  - [`crates/mcp-cad-physics/src/solver.rs`](../../crates/mcp-cad-physics/src/solver.rs) *(замена AABB-заглушки на `collision::detect_contacts`, ввод `ContactRowRole` (`Normal` + сопряжённые касательные строки `Tangent`), внешний конусный решатель `solve_kkt_with_coulomb_contacts` с точным `2×2` trust-region/секулярным проектором `solve_2x2_coulomb_disk` для $\|\boldsymbol{\gamma}_t\|_2 \le \mu \gamma_n$, фиксация нулевых границ и anti-cycling в `solve_box_constrained_kkt`, попарная депенетрация глубочайшего контакта, исключение дублирующего торможения в проекции шага 7 и расчёт `CollisionRecord::{Started, Stopped}` с `contact_normal` и суммарным `max_impulse`)*
  - [`crates/mcp-cad-physics/src/simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs) *(добавлены `PhysicsWorld::collider_registry_mut` и `PhysicsWorld::set_part_contact_properties` с валидацией $\mu \ge 0$ и $e \in [0, 1]$)*
  - [`crates/mcp-cad-physics/src/lib.rs`](../../crates/mcp-cad-physics/src/lib.rs) *(экспорт `pub mod collision` и `ContactPoint`, `TriMeshBvh`)*
  - [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs) *(добавлены 3 интеграционных теста `P4-T4`: упругий/пластический удар и события `CollisionRecord`, статическое залипание + кинетическое скольжение по наклонной плоскости + диагональный изотропный тест Кулона под $37^\circ$, невыпуклый `Compound`/горизонтальный и вертикальный `Cylinder`/`ConvexHull` на `TriMesh` и фильтр `contacts_enabled == false`)*
  - [`TASKS.md`](../../TASKS.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md)

---

## 1. Контекст и постановка задачи

В `P4-T3` собственный многотельный решатель `MultiBodySolver` использовал временную AABB-заглушку обнаружения перекрытий без реальной геометрии примитивов, без `GJK`/`EPA`, без ускоряющего дерева `TriMesh`, без учёта выпуклой формы `Cylinder`/`ConvexHull`/`Compound`, с незаполненным полем `CollisionRecord.max_impulse: None` и с эвристическим ограничением трения без изотропного конуса Кулона $\|\boldsymbol{\gamma}_t\|_2 \le \mu\gamma_n$.

Задача `P4-T4` внедряет полноценный детерминированный `f64`-конвейер коллизий и интегрирует его в связанный Box-LCP / конусный решатель контактов без единого обращения к `rapier3d`/`parry3d`.

---

## 2. Архитектура и ключевые алгоритмы

### 2.1. Broad-Phase и ускоряющее дерево `TriMeshBvh` (`crates/mcp-cad-physics/src/collision.rs`)
1. **Sweep-and-Prune по мировым `BoundingBox` (ось X)**:
   - Сортирует коллайдеры по `world_aabb.min[0]` и отсекает непересекающиеся пары по $(x, y, z)$, пары одного тела, пары двух статических тел и пары тел, связанных шарниром с `contacts_enabled == false`.
   - Сохраняет детерминированный порядок пар и комбинирует свойства материала ($e = \frac{e_a + e_b}{2}$, $\mu = \sqrt{\mu_a \mu_b}$).
2. **`TriMeshBvh` (двоичное AABB-дерево в локальной СК, `BVH_LEAF_MAX_TRIS = 4`)**:
   - Кэшируется однократно при создании/обновлении коллайдера (`Collider::from_part`) и отвечает на запросы перекрытий в локальной СК меша за $O(\log T)$.

### 2.2. Narrow-Phase в `f64`
- **Аналитические пары**:
  - `Sphere`–`Sphere`, `Sphere`–`Cube` (OBB clamping + случай глубокого погружения центра сферы), `Sphere`–`Cylinder` (учёт произвольной оси цилиндра, торцов и боковой поверхности);
  - `Cube`–`Cube`: полный 15-осевой SAT (6 граней + 9 перекрёстных рёбер) с отсечением Сазерленда–Ходжмана (clipping инцидентной грани против 4 боковых плоскостей опорной грани) и редукцией многообразия до 4 точек опоры.
- **Универсальный `f64` GJK + EPA + планарные многообразия**:
  - Поддерживает `ConvexHull`, `Cylinder`, `TriPrism`, `Cube`, `Sphere` и `Compound` (покомпонентная проверка выпуклых суб-деталей с сохранением вогнутых карманов, например L-кронштейна).
  - При гранно-гранном контакте полиэдров извлекает до 4 опорных точек через отсечение сопряжённых граней (`build_planar_manifold`), обеспечивая устойчивую плоскостную опору без опрокидывания.
- **BVH-ускоренный `TriMesh` narrow-phase**:
  - Для сферы — точная проекция на ближайшее ребро/грань треугольника (`collide_sphere_triangle`);
  - Для произвольных выпуклых тел и другого `TriMesh` — фильтрация треугольников через `TriMeshBvh`, предварительный тест нулевой толщины треугольника в GJK против ложных back-face контактов на тонких стенках и `TriPrism` GJK/EPA + группировка копланарного многообразия.

### 2.3. Контактный решатель Кулона и `CollisionRecord` (`crates/mcp-cad-physics/src/solver.rs`)
1. **Односторонняя нормаль и упругое восстановление**:
   - $\gamma_n \ge 0$, правая часть $b_n = -e\,v_{n,\text{pre}}$ (при $v_{n,\text{pre}} < -0.05\,\text{м/с}$ и $e > 0$) без паразитного Baumgarte-отскока на покоящемся контакте.
2. **Связанный изотропный конус Кулона $\|\boldsymbol{\gamma}_t\|_2 \le \mu \gamma_n$**:
   - Для каждого контакта с $\mu > 0$ собираются две ортогональные касательные строки `t1, t2`.
   - На каждой внешней итерации конуса `solve_2x2_coulomb_disk` решает точную задачу квадратичного программирования на диске:
     $$\min_{\|\boldsymbol{\gamma}_t\|_2 \le \mu\gamma_n} \frac{1}{2}\boldsymbol{\gamma}_t^T A_{tt}\boldsymbol{\gamma}_t - \mathbf{b}_t^T \boldsymbol{\gamma}_t \iff (A_{tt} + \lambda I)\boldsymbol{\gamma}_t = \mathbf{b}_t,$$
     с использованием защищённого метода Ньютона Морэ–Соренсена. Это устраняет перекрёстное вращательно-касательное искажение ($a_{12} \ne 0$ на угловых контактах куба) и гарантирует строгую изотропность конуса под любым углом скольжения (без $\sqrt{2}$-ошибки пирамидальной аппроксимации).
   - В `solve_box_constrained_kkt` строки с нулевой шириной интервала `[l, l]` фиксируются (`state = 2`), добавлен single-pivot anti-cycling при вырожденных 4-точечных многообразиях; депенетрация выполняется однократно на уникальную пару тел по максимальной глубине.
   - События `CollisionRecord` формируются после `solve_kkt_system`, заполняя `contact_normal` глубочайшего контакта и суммарный нормальный импульс `max_impulse`.

---

## 3. Результаты верификации точности и инвариантов

1. **Наклонная плоскость ($\theta = 30^\circ$, стальной куб $0.2\text{ м}$)**:
   - **Статическое сцепление** ($\mu = 0.8 > \tan 30^\circ = 0.57735$): дрейф положения `< 5e-4 м`, скорость `< 1e-3 м/с`;
   - **Кинетическое скольжение** ($\mu = 0.2 < \tan 30^\circ$, $T = 0.5\text{ с}$): скорость $1.602929\text{ м/с}$ против точной аналитики $g(\sin\theta - \mu\cos\theta)T = 1.602929\text{ м/с}$ (погрешность **$< 10^{-4}$**), измеренное отношение импульсов $\left|\sum\gamma_{t1}\right| / \sum\gamma_n = 0.200000$.
2. **Диагональное скольжение под углом $37^\circ$ по `TriMesh`-полу ($\mu = 0.25$)**:
   - Изотропное торможение сохраняет курс ($\cos > 0.999$) и величину скорости $\approx v_0 - \mu g \Delta t_{\text{contact}}$.
3. **Упругое восстановление ($e = 1.0$) и пластическое опирание ($e = 0.0$) на `TriMesh`**:
   - Упругая сфера отскакивает на высоту $> 0.85\text{ м}$ от уровня контакта и генерирует `Started` (с `contact_normal` и положительным `max_impulse`) строго до `Stopped`; пластический куб покоится ровно на `y = 0.3 м` (допуск $< 5\times 10^{-4}\text{ м}$).
4. **Все типы коллайдеров и фильтр шарниров**:
   - Вогнутый L-кронштейн `Compound` (шарик падает в внутренний карман на `y = 0.15 м`), горизонтальный (`r = 0.15 м -> y = 0.25 м`) и вертикальный повёрнутый (`half_h = 0.2 м -> y = 0.30 м`) `Cylinder`, `ConvexHull` на `TriMesh` и подавление коллизий при `contacts_enabled == false`.

---

## 4. Обнаруженные смежные проблемы

Новых проблем, выходящих за рамки уже запланированных задач `P4-T5`–`P4-T9` (включая прямое извлечение реакций шарниров без декодера `Rapier3D` в `P4-T5` и удаление зависимости `rapier3d` из `Cargo.toml`/`Cargo.lock` в `P4-T6`), не выявлено.

---

## 5. Верификация

1. `cargo test -p mcp-cad-physics --no-fail-fast`: **37/37 PASS** (`--lib`: **8/8** в `2.23s`, включая `test_p4_t4_trimesh_bvh_query_and_analytical_primitives`; `physics_tests`: **29/29** в `10.49s`, включая 3 новых теста `P4-T4`; `0 failed`).
2. `cargo test -p mcp-cad-server --test gear_kinematic_test --no-fail-fast`: **1/1 PASS** (`test_gear_kinematic_simulation_and_preview`).
3. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
4. `npm run lint` (`tsc --noEmit`), `npm run build` (`vite build`), `npm test` (`52/52 PASS`): **PASS** (`exit 0`).
5. Проверка отсутствия `rapier3d` в `collision.rs`, `solver.rs`, `rigid_body.rs`: **0 импортов / вызовов**.
