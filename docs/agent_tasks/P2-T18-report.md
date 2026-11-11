# Отчёт о выполнении задачи P2-T18

## 1. Что сделано

Выполнена задача **P2-T18 [M]** («Сохранение привязанных шарниров в `physics_world` при перерегистрации детали в CSG/transform и обновление коллайдера в `add_feature_primitive`/`cut_hole`/`modify_feature`/`cut_helical_groove`»):

1. **Обновление коллайдера и инерционных характеристик без потери шарниров (`crates/mcp-cad-physics/src/simulator.rs`)**:
   - Реализован метод `PhysicsWorld::update_part_collider(&mut self, part: &Part) -> Result<RigidBodyHandle, PhysicsError>`:
     - Если деталь ещё не зарегистрирована в `part_to_body`, метод делегирует создание тела и коллайдера в `self.add_part(part, !part.is_fixed)`.
     - Если деталь уже зарегистрирована в `part_to_body`, метод сначала транзакционно строит новый `ColliderBuilder` через `create_collider_builder(part, collider_type)?` (`ColliderType::ConvexHull` для динамических тел `!part.is_fixed`, `ColliderType::TriMesh` для неподвижных `part.is_fixed`). Если сетка детали пуста или некорректна, ошибка возвращается **до** удаления старых коллайдеров, оставляя существующее состояние физического мира нетронутым.
     - После успешного построения нового коллайдера удаляются старые коллайдеры детали из `part_to_colliders`, `collider_to_part` и `colliders`, синхронизируется тип тела (`RigidBodyType::Dynamic` / `RigidBodyType::Fixed` в соответствии с `!part.is_fixed`), новый коллайдер привязывается к существующему `RigidBodyHandle` через `colliders.insert_with_parent(...)`, немедленно пересчитываются масса и тензор инерции `rb.recompute_mass_properties_from_colliders(&self.colliders)` и выполняется пробуждение (`wake_up(true)`) самого тела и всех кинематически связанных с ним через `joints` тел.
     - При этом `RigidBodyHandle`, записи `self.joints`, `self.impulse_joints` (включая настройки моторов `MotorConfig`) и `self.gear_constraints` сохраняются полностью без удаления и пересоздания.
   - Добавлены методы инспекции физического мира `PhysicsWorld::part_mass(&self, part_id: &str) -> Result<f64, PhysicsError>`, `PhysicsWorld::part_collider_aabb(&self, part_id: &str) -> Result<([f64; 3], [f64; 3]), PhysicsError>` и алиас `PhysicsWorld::part_linear_velocity(&self, part_id: &str) -> Result<[f64; 3], PhysicsError>`.

2. **Синхронизация `state.physics_world` во всех инструментах модификации геометрии (`crates/mcp-cad-server/src/tools/cad.rs`)**:
   - В `boolean_difference`, `boolean_union`, `boolean_intersection`, `transform_part` и `optimize_feature_tree` (при `apply: true`) связка `remove_part` + `add_part` заменена на вызов `state.physics_world.update_part_collider(part)`, что сохраняет все ранее созданные шарниры и моторы детали при модификации её геометрии.
   - В `add_feature_primitive`, `cut_hole`, `modify_feature`, `cut_helical_groove`, `tessellate_part` и `export_stl` добавлено обновление коллайдера и массы `state.physics_world.update_part_collider(part)` в случае, если деталь уже зарегистрирована в физическом мире (`state.physics_world.has_part(part_id)`).

3. **Модульные и интеграционные тесты**:
   - В `crates/mcp-cad-physics/tests/physics_tests.rs` добавлен тест `test_p2_t18_update_part_collider_preserves_joints_and_updates_mass_and_bounds`.
   - В `crates/mcp-cad-server/src/tools/cad.rs` добавлен unit-тест `test_p2_t18_joints_preserved_on_csg_transform_and_collider_updated_on_feature_ops`.
   - В `crates/mcp-cad-server/tests/mcp_contract_tests.rs` добавлен контрактный тест `test_mcp_p2_t18_preserve_joints_on_csg_transform_and_update_collider_on_feature_edits`.

---

## 2. Как проверено

Выполнены следующие проверочные команды:

```bash
# 1. Полный прогон тестов физического движка mcp-cad-physics
WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-physics --no-fail-fast
# test result: ok. 9 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

# 2. Прогон unit-тестов сервера mcp-cad-server
WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --lib --no-fail-fast
# test result: ok. 32 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 21.44s

# 3. Прогон контрактного теста P2-T18 в mcp_contract_tests
WGPU_BACKEND=vulkan LIBGL_ALWAYS_SOFTWARE=1 cargo test -p mcp-cad-server --test mcp_contract_tests test_mcp_p2_t18 --no-fail-fast
# test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 37 filtered out; finished in 27.99s

# 4. Прогон тестов ядра и МКЭ
cargo test -p mcp-cad-core --lib --test csg_invariants --test dag_tests --test tessellation_tests -p mcp-cad-fea --no-fail-fast
# mcp-cad-core --lib: test result: ok. 57 passed; 0 failed; 0 ignored
# csg_invariants:     test result: ok. 9 passed; 0 failed; 0 ignored
# dag_tests:          test result: ok. 14 passed; 0 failed; 0 ignored
# tessellation_tests: test result: ok. 6 passed; 0 failed; 0 ignored
# mcp-cad-fea --lib:  test result: ok. 5 passed; 0 failed; 0 ignored
# fea_tests:          test result: ok. 8 passed; 0 failed; 0 ignored

# 5. Проверка TypeScript-суита и линтера фронтенда
npx tsx --test src/cad/cad_engine.test.ts
npm run lint
npm run build
```

---

## 3. Тесты до / после

| Набор тестов | До P2-T18 (passed / failed / ignored) | После P2-T18 (passed / failed / ignored) | Дельта |
|---|---|---|---|
| `mcp-cad-physics --test physics_tests` | 8 / 0 / 0 (не собирался из-за `part_linear_velocity`) | **9 / 0 / 0** | **+1 passed** (`test_p2_t18_update_part_collider_preserves_joints_and_updates_mass_and_bounds`) |
| `mcp-cad-server --lib` | 31 / 0 / 0 | **32 / 0 / 0** | **+1 passed** (`test_p2_t18_joints_preserved_on_csg_transform_and_collider_updated_on_feature_ops`) |
| `mcp-cad-server --test mcp_contract_tests` | 37 / 0 / 0 | **38 / 0 / 0** | **+1 passed** (`test_mcp_p2_t18_preserve_joints_on_csg_transform_and_update_collider_on_feature_edits`) |
| `src/cad/cad_engine.test.ts` | 28 / 0 / 0 | **28 / 0 / 0** | Без изменений |

---

## 4. Остаточные риски

- Для динамических тел (`!part.is_fixed`) в Rapier3D по умолчанию используется выпуклая оболочка `ColliderType::ConvexHull` (так как Rapier3D поддерживает вычисление масс-инерционных характеристик и устойчивый узкофазный контакт динамических тел для выпуклых форм). Поэтому внутренние глухие/сквозные полости, не меняющие внешнюю выпуклую оболочку динамической детали, не уменьшают объём `ConvexHull`-коллайдера в `physics_world` (в отличие от неподвижных деталей с `ColliderType::TriMesh` и точного расчёта массы в `part.mass()`). При необходимости точного учёта внутренних полостей в инерции динамических тел в будущем может быть использована декомпозиция `ColliderType::Compound` или явная передача `part.mass()` через `RigidBodyBuilder::additional_mass_properties`.
- Точки привязки шарниров (`anchor_a`, `anchor_b`) задаются в локальной системе координат тела и сохраняются неизменными при `update_part_collider`; если пользователь масштабирует или сдвигает геометрию детали внутри её локальной СК через `transform_part`, локальные координаты якорей шарнира не пересчитываются автоматически.

---

## 5. Новые находки (заведены в `TASKS.md` со статусом `BACKLOG`)

1. **`P2-T23 [S]` — Защита зарезервированных идентификаторов `"ground"` / `"world"` в `create_part` / `PhysicsWorld::remove_part` и нормализация регистра в `create_joint` (`crates/mcp-cad-server/src/tools/{cad.rs, physics.rs}`, `crates/mcp-cad-physics/src/simulator.rs`)**:
   - В `create_joint` (`crates/mcp-cad-server/src/tools/physics.rs:135`) проверка `is_world_or_ground` регистронезависима (`eq_ignore_ascii_case("ground") || eq_ignore_ascii_case("world")`), однако в `PhysicsWorld::with_gravity` (`simulator.rs:142-143`) в `part_to_body` регистрируются только ключи в нижнем регистре `"ground"` и `"world"`. В результате передача `"part_a": "GROUND"` или `"World"` проходит проверку `is_world_or_ground`, но падает внутри `PhysicsWorld::add_joint` с `PartNotFound("GROUND")`. Кроме того, `create_part` (`tools/cad.rs:315`) не запрещает создание пользовательской детали с `part_id: "ground"` или `"world"`, что позволяет перезаписать или удалить через `delete_part` статический якорь `ground_handle` в `PhysicsWorld`.
   - Также при прогоне `primitive_invariants.rs` зафиксировано превышение порога Sliver Quality Gate (`1.44%` треугольников `< 5°` при лимите `1.00%`) на малом ободе колеса `d=0.14, w=0.06` в тесте `p2_t15_wheel_rim_small_diameters_no_clamp_panic_and_watertight`.
