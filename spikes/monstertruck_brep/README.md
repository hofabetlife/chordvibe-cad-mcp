# Spike 2: `monstertruck` (`v0.4.1`, Pure Rust B-Rep Kernel)

Прототип для архитектурного решения **P1-T3** ([`docs/adr/0001-geometry-kernel.md`](../../docs/adr/0001-geometry-kernel.md)), исследующий **Вариант C** — чистый Rust B-Rep (NURBS) геометрический движок семейства `truck` / `monstertruck` (лицензия `Apache-2.0`, без C++):
- [`monstertruck-modeling v0.4.1`](https://crates.io/crates/monstertruck-modeling/0.4.1) (примитивы, экструзия, вращение, топология `Solid` / `Shell` / `Face` / `Wire` / `Edge` / `Vertex`, `features = ["fillet"]`)
- [`monstertruck-solid v0.4.1`](https://crates.io/crates/monstertruck-solid/0.4.1) (булевы операции B-Rep `or`, `and`, `difference`)
- [`monstertruck-fillet v0.4.1`](https://crates.io/crates/monstertruck-fillet/0.4.1) (скругление рёбер `fillet_edges` методом катящегося шара — rolling-ball fillet)
- [`monstertruck-meshing v0.4.1`](https://crates.io/crates/monstertruck-meshing/0.4.1) (тесселяция B-Rep тел `robust_triangulation` / `triangulation`)
- [`monstertruck-io v0.4.1`](https://crates.io/crates/monstertruck-io/0.4.1) (чтение и запись стандарта ISO 10303-21 STEP AP214)

## Тестовая геометрия (по ТЗ P1-T3)

1. **Цилиндр**: $R_{\text{out}} = 15\text{ мм}$, $h = 50\text{ мм}$ ($z \in [-25, +25]\text{ мм}$).
2. **Минус коаксиальное сквозное отверстие**: $\varnothing 20\text{ мм}$ ($R_{\text{in}} = 10\text{ мм}$).
3. **Плюс бокс-фланец**: $40 \times 40 \times 10\text{ мм}$ ($z \in [-20, -10]\text{ мм}$).
   - Точный аналитический объём фланцевой втулки: $28\,566.3706\text{ мм}^3$.
4. **Скругление ребра $R = 2\text{ мм}$**: через нативный `monstertruck_modeling::fillet_edges`:
   - На вертикальном ребре фланца ($L = 10\text{ мм}$): аналитический объём $28\,557.7865\text{ мм}^3$.
   - На кольцевом ребре пересечения цилиндра с фланцем (`IntersectionCurve`, 4 дуги на $z = -10\text{ мм}, R = 15\text{ мм}$).
5. **Топологическое именование (`FaceID` / `EdgeID`)**: поиск верхней торцевой грани ($z = +25\text{ мм}$), проверка числа граничных контуров (`2` контура: внешний $R=15$ и внутренний $R=10$) и сравнение `FaceID` до и после булевых операций.
6. **Тесселяция и проверка `mcp_cad_core::diagnostics::validate_mesh`**: проверка замкнутости (`is_watertight`), граничных и неманифолдных рёбер, характеристики Эйлера ($\chi = 0, g = 1$), минимальных углов треугольников и объёма.
7. **Экспорт в ISO 10303-21 STEP**: генерация STEP-модели в памяти через `CompleteStepDisplay`.

## Команда запуска

Из корня репозитория:

```bash
cargo run --release --manifest-path spikes/monstertruck_brep/Cargo.toml
```

## Фактические результаты замеров (`--release`)

| Метрика | Значение (`monstertruck v0.4.1`) | Комментарий |
|---|---|---|
| **Время булевой операции Шаг 1 (`Cyl \ Hole`)** | `458.13 мс` (`10` B-Rep граней) | Marching SSI по NURBS-поверхностям |
| **Время булевой операции Шаг 2 (`(Cyl ∪ Flange) \ Hole`)** | `833.96 мс` (`20` B-Rep граней) | В ~33–160× медленнее сеточного `manifold-rust` |
| **Время тесселяции (`robust_triangulation(0.05)`)** | `36.41 мс` | Строит `PolygonMesh` из 20 NURBS/плоских граней |
| **Объём `signed_volume()` (фланцевая втулка)** | `28 560.9344 мм³` | Аналитика: `28 566.3706 мм³` (**ошибка `0.0190%`**) |
| **Число треугольников / вершин** | `21 416` тр. / `12 272` верш. | Избыточная плотность вдоль кривых подрезки |
| **Замкнутость (`is_watertight`) и род (`genus`)** | **`true`** (`open_edges = 0`, `non_manifold = 0`), $\chi = 0, g = 1.0$ | Топологически замкнутая оболочка (`ShellCondition::Closed`) |
| **Качество углов (`validate_mesh`)** | **`FAIL`** (`min_angle = 0.002°`, `1 327` тр. `< 1°`, `11.76% < 5°`) | Триангуляция `spade` в $(u,v)$-пространстве NURBS даёт игольчатые треугольники вдоль кривых пересечения |
| **Скругление ребра $R = 2\text{ мм}$ (`fillet_edges`)** | **`OK` за `7.29 мс`** (прямое ребро, `20 -> 21` грань, объём `28 551.85 мм³`, ошибка `0.0208%`, `watertight = true`); **`OK` за `3.80 мс`** на кольцевом `IntersectionCurve` шеи цилиндра | Нативный B-Rep rolling-ball fillet работает пост-CSG |
| **Идентификация грани торца (`FaceID`)** | Грань #5 (`normal = [0, 0, 1]`, `2` граничных `Wire`, `8` рёбер) найдена однозначно, но `FaceID` (`0x557706af5cf0` $\to$ `0x557706b33fe0`) меняется при CSG | `FaceID` основан на адресе `Arc`; для стабильных имён нужен внешний реестр атрибутов |
| **Экспорт ISO 10303-21 STEP (`monstertruck-io`)** | **`1.25 мс`** (`232 899 байт`, `2 723 строки`, схема `AUTOMOTIVE_DESIGN`) | Полная нативная поддержка STEP без C++ |
