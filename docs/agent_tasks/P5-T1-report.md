# Отчёт о выполнении задачи P5-T1

- **Задача**: `P5-T1` — Архитектурное решение `ADR-005` и полная модель материалов (упругость, $J_2$-пластичность, упрочнение, предельная деформация $\varepsilon_f$, энергия разрушения $G_f$, трение $\mu$, режим разрушения) **[DECISION]**
- **Статус**: ✅ ВЫПОЛНЕНО (`18.11.2026`)
- **Затронутые компоненты**:
  - [`docs/DECISIONS.md`](../DECISIONS.md) *(добавлено архитектурное решение `ADR-005` с обоснованием перехода от предразбиения Вороного и воксельной линейной статики к сопряжённому `f64` неявному упруго-пластическому МКЭ `Co-Rotated TET10 + fe_snap + J2 Radial Return + Augmented Lagrangian Contact + Local Damage / Element Erosion + HHT-α` с серверным кэшированием кадров)*
  - [`crates/mcp-cad-core/src/material.rs`](../../crates/mcp-cad-core/src/material.rs) и [`crates/mcp-cad-core/src/lib.rs`](../../crates/mcp-cad-core/src/lib.rs) *(добавлены `FailureMode`, псевдоним `MaterialSpec`, поля `ultimate_strength_pa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`, `failure_mode`, обратно совместимая десериализация `MaterialSerdeRaw` с физически согласованными дефолтами для старых проектов, валидация инвариантов `Material::validate()` / `MaterialRegistry::validate_all()` / `try_register()`, 9-й пресет `borosilicate_glass` + алиасы и 3 новых unit-теста `P5-T1`)*
  - [`crates/mcp-cad-server/src/tools/{cad.rs, mod.rs}`](../../crates/mcp-cad-server/src/tools/cad.rs) и [`crates/mcp-cad-server/tests/mcp_contract_tests.rs`](../../crates/mcp-cad-server/tests/mcp_contract_tests.rs) *(поддержка опциональных полей Фазы 5 и валидации `Material::validate()` в `define_material`)*
  - [`src/types/cad.ts`](../../src/types/cad.ts), [`src/cad/materials.ts`](../../src/cad/materials.ts), [`src/cad/mcp_dispatcher.ts`](../../src/cad/mcp_dispatcher.ts), [`src/components/CadLeftSidebar.tsx`](../../src/components/CadLeftSidebar.tsx), [`src/cad/cad_engine.test.ts`](../../src/cad/cad_engine.test.ts) *(синхронизация `MaterialSpec`, справочные параметры для всех 9 материалов, `deriveNonlinearMaterialDefaults`, обратно совместимый `normalizeMaterialSpec`, `validateMaterialSpec` / `assertValidMaterialSpec` и 2 новых unit-теста `P5-T1`)*
  - [`TASKS.md`](../../TASKS.md), [`docs/PROGRESS.md`](../PROGRESS.md), [`CHANGELOG.md`](../../CHANGELOG.md), [`PROJECT_CONTEXT.md`](../../PROJECT_CONTEXT.md) *(актуализация статуса `P5-T1`, фиксация 3 новых обнаруженных проблем `P5-T9`–`P5-T11` и синхронизация документации)*

---

## 1. Реализация

1. **`ADR-005` (`docs/DECISIONS.md`)**:
   - Зафиксированы причины отказа от предразбиения Вороного (`src/cad/destruction_engine.ts` сохранён исключительно как автономный fallback `Web Local Preview`), воксельной линейной статики и явной динамики (ограничение Куранта CFL $\Delta t_{\text{crit}} \approx 8.5 \times 10^{-8}\text{ с}$ для стали при $h_{\min} = 0.5\text{ мм}$).
   - Утверждена 6-слойная архитектура сопряжённого `f64` МКЭ-конвейера (`P5-T1`–`P5-T7`), таблица конститутивных параметров и известные ограничения.
2. **Расширенная модель материалов в Rust (`crates/mcp-cad-core/src/material.rs`)**:
   - Введён `FailureMode` (`Ductile` | `Brittle`, сериализация `snake_case`) и псевдоним `MaterialSpec = Material`.
   - `Material` расширен полями `ultimate_strength_pa`, `hardening_modulus_pa`, `fracture_strain`, `fracture_energy_j_m2`, `friction_coeff`, `failure_mode` с обратно совместимой десериализацией через `#[serde(from = "MaterialSerdeRaw")]`: старые 6-полевые JSON-проекты (включая высокопрочные материалы вроде титана `880 МПа`) автоматически получают физически корректные дефолты ($\sigma_u = 1.25\sigma_y$ для `ductile`, $\sigma_u = \sigma_y$ для `brittle`), проходящие `Material::validate()`.
   - Добавлен пресет `borosilicate_glass` (с алиасами `borosilicate`, `glass`, `pyrex`, `glass_borosilicate`), доведя библиотеку до **9** эталонных материалов (`steel_1045`/`steel_structural_aisi1020`, `steel_4140`/`steel_high_strength_4140`, `alu_6061`/`aluminum_6061_t6`, `ti_6al_4v`/`titanium_grade5_ti6al4v`, `brass_c360`, `pla_3d`/`pla_filament_3dprint`, `petg_3d`/`petg_filament_3dprint`, `nylon_pa12`, `borosilicate_glass`).
   - Реализована строгая проверка инвариантов `Material::validate()`: $\rho > 0$, $E > 0$, $0 < \nu < 0.5$, $\sigma_u \ge \sigma_y > 0$, $H \ge 0$, $\varepsilon_f > 0$, $G_f > 0$, $\mu \ge 0$ и конечность всех значений.
3. **Расширенная модель материалов в TypeScript (`src/types/cad.ts`, `src/cad/materials.ts`)**:
   - Расширен интерфейс `MaterialSpec` полями `ultimateStrengthPa`, `hardeningModulusPa`, `fractureStrain`, `fractureEnergyJM2`, `frictionCoeff`, `failureMode` (`MaterialFailureMode`).
   - Обновлены все 9 материалов в `MATERIALS_LIBRARY` справочными параметрами, добавлены `deriveNonlinearMaterialDefaults`, `normalizeMaterialSpec` (поддерживающий как legacy-объекты без полей Фазы 5, так и snake_case JSON из Rust) и `validateMaterialSpec` / `assertValidMaterialSpec`.
   - Обновлены `CadLeftSidebar.tsx`, `mcp_dispatcher.ts` и Rust-обработчик `define_material` (`tools/cad.rs`, `tools/mod.rs`).

---

## 2. Верификация

Проверки после финальных правок кода:
- `cargo test -p mcp-cad-core --no-fail-fast`: **136/136 PASS** (`--lib`: **79/79**, включая 3 новых unit-теста `P5-T1` в `material::tests`; `csg_invariants`: **10/10**; `dag_tests`: **16/16**; `primitive_invariants`: **25/25**; `tessellation_tests`: **6/6**; `0 failed`).
- `cargo test -p mcp-cad-fea --no-fail-fast`: **21/21 PASS** (`--lib`: **8/8**; `fea_benchmarks`: **2/2**; `fea_tests`: **11/11**; `0 failed`).
- `cargo test -p mcp-cad-physics --no-fail-fast`: **42/42 PASS** (`--lib`: **10/10**; `physics_tests`: **32/32**; `0 failed`).
- `cargo clippy --workspace --all-targets -- -D warnings`: **PASS** (`0` ошибок, `0` предупреждений во всех 5 крейтах, включая `mcp-cad-server` и `mcp-cad-viewer`).
- `npm run lint` (`tsc --noEmit`) и `npm run build` (`vite build`): **PASS** (`0` ошибок).
- `npm test` (`src/cad/cad_engine.test.ts` + `src/cad/__tests__/analyze_mechanism.test.ts`): **55/55 PASS** (`40/40`, включая 2 новых unit-теста `P5-T1` + `15/15`, `0 failed`).
- `npx --yes tsx --test src/cad/__tests__/destruction_engine.test.ts src/cad/__tests__/ortho_views.test.ts`: **10/10 PASS** (`0 failed`).
- **Ограничение окружения при проверке `mcp-cad-server`**: тесты `tools::project::tests::test_part_material_ref_deserialize` и `test_project_schema_version_constant` прошли (`PASS`), но тесты `mcp-cad-server`, инстанцирующие `EngineState::new()` (`test_mcp_define_material`, `test_save_and_load_roundtrip`), не смогли стартовать в данном headless-контейнере без `/usr/share/vulkan/icd.d` (`mesa-vulkan-drivers`) из-за отсутствия реального CPU-fallback в `OffscreenRenderer::new()` (`Viewer(NoSuitableAdapter)`).

---

## 3. Новые проблемы, выявленные и заведённые в `TASKS.md`

1. **`P5-T9 [S]`**: Отсутствие реального CPU-fallback в `OffscreenRenderer::new()` (`crates/mcp-cad-viewer/src/offscreen.rs:169-211`) при отсутствии GPU и Vulkan Lavapipe (`NoSuitableAdapter`), вопреки заявлению в `ADR-003` и `PROJECT_CONTEXT.md`.
2. **`P5-T10 [S]`**: Устаревший список `builtin_materials` в обработчике ошибки `UNKNOWN_MATERIAL` (`crates/mcp-cad-server/src/error.rs:252-266`) — содержит несуществующие пресеты (`steel_stainless_304`, `copper_c10100`, `abs_plastic`, `polycarbonate`) и не перечисляет реальные (`steel_high_strength_4140`, `petg_filament_3dprint`, `borosilicate_glass`).
3. **`P5-T11 [S]`**: Расхождение параметров стали `steel_1045` между Rust (`steel_structural_aisi1020`, $\sigma_y = 250\text{ МПа}$) и TypeScript (`steel_1045`, AISI 1045, $\sigma_y = 350\text{ МПа}$).
