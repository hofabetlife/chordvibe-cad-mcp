# Отчёт о выполнении задачи P4-T6

- **Задача**: `P4-T6` — Удаление `rapier3d` из зависимостей, очистка `mcp-cad-server` и синхронизация веб-студии (`src/`)
- **Статус**: ✅ ВЫПОЛНЕНО (`18.11.2026`)
- **Затронутые компоненты**:
  - [`crates/mcp-cad-physics/Cargo.toml`](../../crates/mcp-cad-physics/Cargo.toml) и [`Cargo.lock`](../../Cargo.lock) *(полное удаление зависимости `rapier3d = "0.22"` и всего транзитивного дерева `parry3d` / `simba`-субкрейтов игрового движка)*
  - [`crates/mcp-cad-physics/src/simulator.rs`](../../crates/mcp-cad-physics/src/simulator.rs) *(добавлены публичные методы запроса нативных дескрипторов `PhysicsWorld::part_body_handle(&self, part_id: &str) -> Result<BodyHandle, PhysicsError>` и `PhysicsWorld::joint_handle(&self, joint_id: &str) -> Result<JointHandle, PhysicsError>`)*
  - [`crates/mcp-cad-physics/src/{rigid_body.rs, collision.rs, dae_prototype.rs}`](../../crates/mcp-cad-physics/src/lib.rs) и [`crates/mcp-cad-physics/tests/physics_tests.rs`](../../crates/mcp-cad-physics/tests/physics_tests.rs) *(очистка остаточных упоминаний `rapier3d`/`parry3d`/`RigidBodyHandle` и добавление интеграционного теста `test_p4_t6_native_handles_and_f64_motor_validation`)*
  - [`crates/mcp-cad-server/src/tools/physics.rs`](../../crates/mcp-cad-server/src/tools/physics.rs), [`crates/mcp-cad-server/src/tools/mod.rs`](../../crates/mcp-cad-server/src/tools/mod.rs), [`crates/mcp-cad-server/src/tools/recipes.rs`](../../crates/mcp-cad-server/src/tools/recipes.rs), [`crates/mcp-cad-server/src/state.rs`](../../crates/mcp-cad-server/src/state.rs), [`crates/mcp-cad-server/tests/mcp_contract_tests.rs`](../../crates/mcp-cad-server/tests/mcp_contract_tests.rs) *(удаление `Rapier`-специфичных `as f32` приведений в `set_joint_motor` с заменой на `f64`-границу `MAX_MOTOR_PARAM_F64 = 1.0e30`, актуализация описаний инструментов `create_joint`, `run_physics_simulation`, `apply_recipe`)*
  - [`src/components/CadRightSidebar.tsx`](../../src/components/CadRightSidebar.tsx), [`src/components/CadStatusBar.tsx`](../../src/components/CadStatusBar.tsx), [`src/components/CadLeftSidebar.tsx`](../../src/components/CadLeftSidebar.tsx), [`src/components/CadRibbonToolbar.tsx`](../../src/components/CadRibbonToolbar.tsx), [`src/cad/workbenches.ts`](../../src/cad/workbenches.ts), [`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts), [`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts) *(замена устаревших UI-лейблов `Состояние решателя Rapier3D` / `RAPIER3D ACTIVE` / `Rapier3D` на `Состояние решателя DAE (f64)` / `DAE f64 ACTIVE` / `f64 DAE` и добавление unit-теста `P4-T6`)*
  - [`TASKS.md`](../../TASKS.md)

---

## 1. Контекст и постановка задачи

После полной замены ядра многотельной динамики (`P4-T2`, `P4-T3`), конвейера коллизий (`P4-T4`) и извлечения реакций Лагранжа (`P4-T5`) на собственный `f64` DAE-решатель в проекте оставались:
1. Зависимость `rapier3d = "0.22"` в `crates/mcp-cad-physics/Cargo.toml` и транзитивные крейты `rapier3d`/`parry3d` в `Cargo.lock`.
2. Приведения `!(target_velocity as f32).is_finite()` и `!(max_force as f32).is_finite()` в `crates/mcp-cad-server/src/tools/physics.rs`, введённые ранее для защиты `f32`-движка `Rapier3D`, а также упоминания `Rapier3D` в описаниях инструментов `tools/mod.rs`.
3. Устаревшие ярлыки в веб-студии (`src/`): `Состояние решателя Rapier3D` в правой панели симуляции, `RAPIER3D ACTIVE` в строке состояния, бейдж `Rapier3D` в левой панели, подсказки в `CadRibbonToolbar.tsx`, подзаголовок верстака в `workbenches.ts` и описание `create_joint` в `mcp_dispatcher.ts`.

---

## 2. Выполненные изменения

### 2.1. Удаление `rapier3d` из `Cargo.toml` и `Cargo.lock`
- Из [`crates/mcp-cad-physics/Cargo.toml`](../../crates/mcp-cad-physics/Cargo.toml) удалена строка `rapier3d = "0.22"`.
- Обновлён [`Cargo.lock`](../../Cargo.lock): проверка `cargo tree | grep -i -E 'rapier|parry'` возвращает пустой результат (`0` пакетов).

### 2.2. Публичные дескрипторы `PhysicsWorld` и очистка `f32`-костылей в `mcp-cad-server`
- В [`PhysicsWorld`](../../crates/mcp-cad-physics/src/simulator.rs) добавлены публичные методы `part_body_handle(&self, part_id: &str) -> Result<BodyHandle, PhysicsError>` (включая регистронезависимую нормализацию `"ground"` / `"world"`) и `joint_handle(&self, joint_id: &str) -> Result<JointHandle, PhysicsError>`, дополняющие сигнатуры `add_part -> Result<BodyHandle, PhysicsError>`, `update_part_collider -> Result<BodyHandle, PhysicsError>` и `add_joint -> Result<JointHandle, PhysicsError>`.
- В [`crates/mcp-cad-server/src/tools/physics.rs`](../../crates/mcp-cad-server/src/tools/physics.rs) (`set_joint_motor`) удалены `Rapier`-специфичные проверки `!(target_velocity as f32).is_finite()` и `!(max_force as f32).is_finite()`. Вместо приведения к `f32` введена константа `MAX_MOTOR_PARAM_F64: f64 = 1.0e30`, защищающая квадратичные и биквадратные формы KKT/SVD в `f64`-решателе от переполнения `f64::MAX` при астрономических входах (`1.0e100`).
- Обновлены описания инструментов `create_joint`, `run_physics_simulation` и `apply_recipe` в [`crates/mcp-cad-server/src/tools/mod.rs`](../../crates/mcp-cad-server/src/tools/mod.rs), комментарии в `state.rs`, `recipes.rs`, `physics.rs`, `rigid_body.rs`, `collision.rs`, `dae_prototype.rs`, `physics_tests.rs` и `mcp_contract_tests.rs`.

### 2.3. Синхронизация веб-студии (`src/`)
- [`src/components/CadRightSidebar.tsx:755`](../../src/components/CadRightSidebar.tsx): `Состояние решателя Rapier3D` $\to$ `Состояние решателя DAE (f64)`.
- [`src/components/CadStatusBar.tsx:82`](../../src/components/CadStatusBar.tsx): `RAPIER3D ACTIVE` $\to$ `DAE f64 ACTIVE`.
- [`src/components/CadLeftSidebar.tsx:591`](../../src/components/CadLeftSidebar.tsx): бейдж `Rapier3D` $\to$ `f64 DAE`.
- [`src/components/CadRibbonToolbar.tsx:215,628`](../../src/components/CadRibbonToolbar.tsx): описание пресета `Кривошипно-ползунный механизм (f64 DAE)` и всплывающая подсказка кнопки запуска симуляции `Запуск / Пауза кинематической симуляции f64 DAE (Клавиша P)`.
- [`src/cad/workbenches.ts:36`](../../src/cad/workbenches.ts): подзаголовок верстака `3. Симуляция` обновлён на `Кинематика механизмов (f64 DAE), шарниры, приводы, зубчатые зацепления`.
- [`src/cad/mcp_dispatcher.ts:252`](../../src/cad/mcp_dispatcher.ts): описание `create_joint` обновлено на `Create a kinematic joint (revolute, prismatic, fixed, spherical, gear) in the f64 DAE solver`.

---

## 3. Обнаруженные смежные проблемы

Новых проблем, выходящих за рамки оставшихся задач `P4-T7`–`P4-T9`, не выявлено. Также в `TASKS.md` синхронизирован статус `P4-T5` (`✅ DONE`) в сводной таблице и архивном реестре.

---

## 4. Верификация

1. `cargo tree | grep -i -E 'rapier|parry'`: **пусто** (`0` совпадений).
2. `grep -rnI -i 'rapier\|parry' crates src`: **пусто** (`0` совпадений).
3. `cargo test -p mcp-cad-physics --no-fail-fast`: **39/39 PASS** (`--lib`: **8/8**, `physics_tests`: **31/31**, включая новый `test_p4_t6_native_handles_and_f64_motor_validation`; `0 failed`).
4. `cargo test -p mcp-cad-server --lib tools::physics::tests --no-fail-fast`: **7/7 PASS**.
5. `cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_p2_t11_set_joint_motor_numeric_validation` и `test_mcp_p2_t10_initialize_instructions_and_tool_descriptions_contract`: **2/2 PASS**.
6. `cargo test -p mcp-cad-server --test gear_kinematic_test --no-fail-fast`: **1/1 PASS**.
7. `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах).
8. `npm run lint` (`tsc --noEmit`), `npm run build` (`vite build`), `npm test` (`53/53 PASS`, включая новый тест `P4-T6` в `cad_engine.test.ts`): **PASS** (`exit 0`).
