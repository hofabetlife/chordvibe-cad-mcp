# Отчёт по задаче P5-T4: Физический контакт деформируемых тел (`Augmented Lagrangian` / `Mortar` с трением Кулона, упругим сплющиванием пятна Герца и многопарным перераспределением нагрузки)

- **Дата:** 20.11.2026
- **Задача:** `P5-T4 [L]` — *Физический контакт деформируемых тел (`Augmented Lagrangian` / `Mortar` с трением Кулона, упругим сплющиванием пятна Герца и многопарным перераспределением нагрузки)*
- **Статус:** ✅ **ВЫПОЛНЕНО**

---

## 1. Резюме изменений

1. **Аналитика Герца, проекция на деформируемую поверхность и связка `f64`-шарниров в `mcp-cad-physics` (`crates/mcp-cad-physics/src/{collision.rs, solver.rs, lib.rs}`, `#![deny(unsafe_code)]`):**
   - В `collision.rs` реализованы точные аналитические решения Герца для контакта двух параллельных упругих цилиндров (`HertzCylinderSolution`, `hertz_cylinder_contact`: $a = \sqrt{\frac{4 P' R^*}{\pi E^*}}$, $p_0 = \frac{2 P'}{\pi a}$, подповерхностный максимум $\tau_{\max} \approx 0.300 p_0$ на глубине $z \approx 0.786 a$) и двух упругих сфер (`HertzSphereSolution`, `hertz_sphere_contact`), а также индекс деформируемой поверхностной сетки `DeformableSurfaceIndex` и проекция `SurfaceProjection` с расчётом знакового зазора $g_n$ по псевдонормалям Беренцена–Аанеса (`Bærentzen–Aanæs`) и динамическим обновлением вершин `update_vertices`.
   - В `solver.rs` реализованы дескрипторы кинематической связи ступиц деформируемых тел с шарнирами и моторами `f64`-решателя (`HubJointKind`, `JointHubCouplingSpec`, `extract_joint_hub_couplings`) и функция обратной синхронизации деформированных поверхностных вершин МКЭ в `TriMesh`-коллайдер (`sync_deformed_trimesh_collider` с перестройкой локального AABB и `TriMeshBvh`).

2. **Кинематическое сопряжение ступиц/отверстий вала `HubCoupling` без искусственного закрепощения зубьев (`crates/mcp-cad-fea/src/boundary.rs`):**
   - Добавлен метод `BoundaryConditions::resolve_allowing_unconstrained`, позволяющий телам в многотельном контактном расчёте удерживаться чисто через `HubCoupling` и контакт без фиктивных граничных закреплений.
   - Реализованы селекторы узлов ступицы `HubNodeSelector` (`BoreCylinder`, `OuterCylinder`, `SurfaceTriangles`, `AabbFace`, `Nodes`), режимы привода `RevoluteHubDrive` (`PrescribedAngle`, `Torque` с опциональной крутильной жёсткостью вала $k_\theta$) и `PrismaticHubDrive` (`PrescribedDisplacement`, `Force` с осевой жёсткостью), а также спецификация `HubCoupling` (`Revolute`, `Prismatic`, `Fixed`) и её геометрический резолвер `HubCoupling::resolve`.

3. **Многотельный нелинейный контактный решатель деформируемых сеток (`crates/mcp-cad-fea/src/contact.rs`, `lib.rs`, `solver.rs`):**
   - **`DeformedSurfaceMesh` с обновлением на каждой итерации Ньютона–Рафсона:** извлечение строго внешне ориентированных граничных треугольников `TET4`/`TET10`, динамический пересчёт деформированных координат вершин, плоского дерева `SurfaceBvh`, направленных контактных площадей `contact_nodal_area` (отсекающих перпендикулярные торцевые стенки $z = \text{const}$) и сглаженных нормалей с фильтрацией острых рёбер `crease_filtered_vertex_normal` ($\mathbf{n}_{\text{tri}} \cdot \mathbf{n}_{\text{face}} > 0.80$).
   - **Две схемы дискретизации контакта (`ContactDiscretization`):**
     - `NodeToSurface` (`TET4`/`TET10`) с односторонним или симметричным двухпроходным (`symmetric: true`) контролем взаимопроникновения;
     - `MortarGauss` (квадратурный Mortar-контакт по 3 точкам Хаммера на 6-узловых квадратичных гранях `T6` для `TET10` и `T3` для `TET4`, выбирается автоматически в `ContactDiscretization::Auto` для `TET10`).
   - **Метод расширенных множителей Лагранжа (`Augmented Lagrangian` / Удзава) с изотропным сухим трением Кулона:**
     - нормальное давление $p_n = \max(0, \lambda_n - \epsilon_n g_n)$ с точным контролем непроникновения $g_n \ge -g_{\text{tol}}$ в обе стороны для всех пар (`measure_pair_penetrations`);
     - радиальный возврат касательного вектора трения Кулона на плоскости скольжения ($\|\mathbf{t}_T\| \le \mu p_n$, режимы `stick` и `slip`) с симметричной положительно полуопределённой касательной матрицей жёсткости контакта;
     - точная редукция кинематических степеней свободы `StepDofSystem` (конечные повороты Родрига $\mathbf{r}(\theta) = \mathbf{R}(\theta)\mathbf{r}_0$ для узлов отверстия вала `Revolute` со свободными зубьями, пилот-координата угла поворота/хода ступицы и предиктор жёсткого поворота на старте шага);
     - предобусловленный метод сопряжённых градиентов с симметричным проходом Гаусса–Зейделя (`solve_ssor_pcg`, SSOR $\omega = 1$).

---

## 2. Измеренные результаты эталонных тестов (`deformable_contact_tests.rs` и `mcp-cad-physics`)

Проверено командой `cargo test -p mcp-cad-fea --test deformable_contact_tests -- --nocapture`:

| № | Эталонный тест | Дискретизация | Измеренный результат | Аналитика / критерий приёмки | Погрешность / допуск | Статус |
|---|---|---|---|---|---|---|
| **1** | **Контакт Герца двух упругих стальных цилиндров** ($R_1 = R_2 = 50\text{ мм}$, $E = 205\text{ ГПа}$, $\nu = 0.29$, $P' = 400\text{ кН/м}$) | `NodeToSurface` (`TET4`, плоская деформация) | $p_{0,\text{FEM}} = \mathbf{776.36\text{ МПа}}$, $a_{\text{FEM}} = \mathbf{0.3410\text{ мм}}$, подповерхностный $\sigma_{vM,\max} = 440.47\text{ МПа}$, $g_{\text{pen},\max} = \mathbf{4.900\times 10^{-11}\text{ м}}$ | $p_{0,\text{exact}} = 754.96\text{ МПа}$, $a_{\text{exact}} = 0.3373\text{ мм}$ (погрешность $\le 5.0\%$), $\sigma_{vM,\text{ref}} \approx 422.78\text{ МПа}$ | **$p_0$: `2.84% <= 5.0%`**, **$a$: `1.11% <= 5.0%`** ($\sigma_{vM}$: `4.18%`) | ✅ PASS |
| **2** | **Сухое трение Кулона (`stick` и `slip`)** между деформируемым ползуном и упругим основанием ($\mu = 0.30$, $F_N = 10\,000\text{ Н}$) | `NodeToSurface` (`TET4`) | Шаг 2 (`stick`, $F_x = 1500\text{ Н} < \mu F_N$): `stick_pts = 6`, $F_{T,x} = \mathbf{-1500.00\text{ Н}}$; Шаг 3 (`slip`, сдвиг $50\text{ мкм}$): `slip_pts = 15/15`, $F_{T,x} = \mathbf{-2999.85\text{ Н}}$, $g_{\text{pen},\max} = 4.750\times 10^{-10}\text{ м}$ | `stick`: $F_{T,x} = -1500.00\text{ Н}$; `slip`: $F_{T,x} = -\mu F_N = -3000.00\text{ Н}$ | **`0.000%`** (`stick`), **`0.005%`** (`slip`) | ✅ PASS |
| **3** | **Контакт двух деформируемых эвольвентных шестерён при вращении под нагрузкой** ($m = 4\text{ мм}$, $z_1 = z_2 = 14$, $\alpha = 25^\circ$, $b = 6\text{ мм}$, $\mu = 0.08$, 4 шага поворота ведущего вала $\theta_1 \in [0.003, 0.018]\text{ рад}$ против моментной нагрузки ведомого вала) | Двухпроходный `NodeToSurface` (`symmetric: true`, `HubCoupling::Revolute` на отверстиях валов) | Все 4 шага: $\theta_2 = -0.0024 \to -0.0173\text{ рад}$, $T_1 = 9.49 \to 18.34\text{ Н·м}$ ($T_2 = 9.42 \to 18.38\text{ Н·м}$), $F_n = 353.3 \to 683.7\text{ Н}$, $p_{\max} = 79.22 \to 150.06\text{ МПа}$ вдоль 3D-линии контакта по ширине венца $b$, `surf_upd = 210`; максимальное проникновение по всем шагам: **`1.677e-8 м`** (`Step 1: 0.000e0`, `Step 2: 1.047e-8`, `Step 3: 3.836e-9`, `Step 4: 1.677e-8 м`) | Взаимопроникновение строго $< 10^{-5}\text{ м}$ на всех шагах, физически корректное распределение давления по линии контакта и обновление деформированной поверхности на каждой итерации Ньютона | **`1.677e-8 м < 1.0e-5 м`** (запас в **596 раз**) | ✅ PASS |
| **4** | **Упруго-пластическое вдавливание штампа и обратная связь деформированной поверхности** (алюминий $\sigma_{y0} = 150\text{ МПа}$, вдавливание $-0.45\text{ мм}$ → отвод $+1.0\text{ мм}$) | `NodeToSurface` (`TET4`, $J_2$-пластичность) | Нагружение: $\bar{\varepsilon}_{p,\max} = \mathbf{7.210\%}$, $g_{\text{pen},\max} = 7.324\times 10^{-9}\text{ м}$; после полной разгрузки на `DeformedSurfaceMesh` сохраняется пластический кратер глубиной **`-0.3948 мм`** | Формирование пластического кратера на актуальной контактной поверхности, полное разделение при отводе | `crater = -0.3948 мм`, `max_pen = 7.324e-9 м` | ✅ PASS |
| **5** | **Квадратичный `TET10` Mortar-контакт по точкам Гаусса (`T6`)** ($F_y = -8000\text{ Н}$) | `MortarGauss` (`TET10`, `Auto`) | `converged = true`, `al_iters = 10`, `newton_iters = 21`, `active = 12` точек Гаусса, $F_n = \mathbf{8000.00\text{ Н}}$, $p_{\text{peak}} = 65.15\text{ МПа}$, $g_{\text{pen},\max} = \mathbf{3.296\times 10^{-7}\text{ м}}$ | Гладкое распределение давления без осцилляций вершинных и серединных узлов `TET10`, $F_n = 8000\text{ Н}$ | **`0.000%`** по силе, `max_pen = 3.296e-7 м` | ✅ PASS |
| **6** | **Unit-тесты `mcp-cad-physics` (`collision::tests`, `solver::p5_t4_coupling_tests`)** | `DeformableSurfaceIndex`, `extract_joint_hub_couplings`, `sync_deformed_trimesh_collider` | Проверка формул Герца цилиндр/сфера, знакового зазора $g_n$ снаружи/внутри деформируемой сетки, извлечения оси/момента `Revolute` для ступицы и обновления `TriMeshBvh` коллайдера | Точное совпадение с аналитикой и инвариантами | **`< 1e-12`** | ✅ PASS |

---

## 3. Верификация тестами и линтером

Выполнено после всех финальных правок:
- `cargo test -p mcp-cad-fea --test deformable_contact_tests -- --nocapture`: **5/5 PASS** (`17.69s`).
- `cargo test -p mcp-cad-fea --no-fail-fast` (все 5 тестовых целей: `--lib`: `13/13`, `deformable_contact_tests`: `5/5`, `fea_benchmarks`: `3/3`, `fea_tests`: `14/14`, `nonlinear_elastoplastic_tests`: `4/4`): **39/39 PASS** (`0 failed`).
- `cargo test -p mcp-cad-physics --no-fail-fast` (`--lib`: `6/6`, `physics_tests`: `38/38`): **44/44 PASS** (`0 failed`).
- `cargo test -p mcp-cad-core --no-fail-fast`: **136/136 PASS** (`0 failed`).
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).

---

## 4. Затронутые файлы

- `crates/mcp-cad-physics/src/collision.rs`
- `crates/mcp-cad-physics/src/solver.rs`
- `crates/mcp-cad-physics/src/lib.rs`
- `crates/mcp-cad-fea/src/boundary.rs`
- `crates/mcp-cad-fea/src/contact.rs` *(новый)*
- `crates/mcp-cad-fea/src/solver.rs`
- `crates/mcp-cad-fea/src/lib.rs`
- `crates/mcp-cad-fea/tests/deformable_contact_tests.rs` *(новый)*
- `docs/agent_tasks/P5-T4-report.md` *(новый)*
- `TASKS.md`, `docs/PROGRESS.md`, `docs/FEA_ACCURACY.md`
