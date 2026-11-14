# P3-T8 Report: Аналитическая проверка прочности зубьев шестерён по Льюису и Герцу/AGMA (`check_gear_strength`)

## 1. Цель задачи (`fix/P3_calculations.md` §P3-T8)
Реализовать 37-й инструмент MCP `check_gear_strength` для быстрой аналитической оценки изгибных напряжений у корня зуба прямозубой цилиндрической шестерни по уравнению Льюиса (с учётом динамического коэффициента скорости по Барту $K_v$) и контактных напряжений сжатия в полюсе зацепления (сопротивление питтингу) по теории Герца / AGMA / ISO 6336.

МКЭ на воксельной сетке сглаживает криволинейный эвольвентный профиль зуба, поэтому данный инструмент предоставляет ИИ-агенту мгновенную и строгую инженерную верификацию зубчатых передач без необходимости запуска дорогостоящего сеточного решателя.

---

## 2. Математический аппарат и формулы

1. **Делительный диаметр и передаваемая окружная сила**:
   - $d_1 = m \cdot z_1$ (м)
   - $W_t = \frac{2 T}{d_1}$ (Н), где $T$ — крутящий момент в Н·м.
   - Если задана мощность $P$ (Вт или кВт) и частота вращения $n$ (об/мин), момент вычисляется как:
     $$T = \frac{P}{\omega} = \frac{60 P}{2 \pi n}.$$

2. **Окружная скорость и динамический коэффициент скорости по Барту ($K_v$)**:
   - Окружная скорость на делительной окружности: $V = \frac{\pi \cdot d_1 \cdot n}{60}$ (м/с).
   - Для коммерческих фрезерованных/строганных зубьев (`commercial_cut`):
     $$K_v = \frac{6.1 + V}{6.1} \quad (\text{в единицах СИ}).$$
   - Для шлифованных/прецизионно зубофрезерованных зубьев (`precision_ground`):
     $$K_v = \frac{15.2 + V}{15.2} \quad (\text{в единицах СИ}).$$
   - Для статической нагрузки ($V = 0$ или частота не задана): $K_v = 1.0$.

3. **Изгибные напряжения у корня зуба (Уравнение Льюиса)**:
   $$\sigma_b = \frac{K_v \cdot W_t}{b \cdot m \cdot Y} \quad (\text{Па}), \qquad \sigma_{b,\text{МПа}} = \frac{\sigma_b}{10^6},$$
   где $b$ — ширина венца (м), $m$ — модуль (м), $Y$ — коэффициент формы зуба Льюиса для угла зацепления $\alpha = 20^\circ$ полной высоты (кусочно-линейная интерполяция по Shigley's Mechanical Engineering Design, Table 14-2: $z=17 \to 0.303$, $z=20 \to 0.322$, $z=400 \to 0.480$, рейка $\to 0.485$).  
   Коэффициент запаса по изгибу:
   $$\text{FoS}_b = \frac{S_y}{\sigma_{b,\text{МПа}}} \quad \left(\text{или } \frac{\sigma_{\text{allow,b}}}{\sigma_{b,\text{МПа}}}\right).$$

4. **Контактные напряжения сжатия / питтинг (Теория Герца / AGMA / ISO 6336)**:
   - Радиусы кривизны эвольвентных поверхностей зубьев в полюсе зацепления:
     $$r_1 = \frac{d_1 \sin\alpha}{2}, \qquad r_2 = \frac{d_2 \sin\alpha}{2}, \quad (d_2 = m \cdot z_2).$$
   - Коэффициент упругости материалов контактирующей пары $C_p$:
     $$C_p = \sqrt{\frac{1}{\pi \left(\frac{1 - \nu_1^2}{E_1} + \frac{1 - \nu_2^2}{E_2}\right)}} \quad (\text{Па}^{0.5}),$$
     (для пары сталь-сталь с $E = 205\text{ ГПа}, \nu = 0.29$: $C_p \approx 1.8874 \times 10^5\text{ Па}^{0.5} = 188.74\text{ МПа}^{0.5}$).
   - Контактное напряжение:
     $$\sigma_H = C_p \sqrt{\frac{K_v \cdot W_t}{b \cos\alpha} \left(\frac{1}{r_1} + \frac{1}{r_2}\right)} \quad (\text{Па}), \qquad \sigma_{H,\text{МПа}} = \frac{\sigma_H}{10^6}.$$
   - Коэффициент запаса по контактной прочности:
     $$\text{FoS}_H = \frac{\sigma_{\text{allow,H}}}{\sigma_{H,\text{МПа}}}.$$

5. **Предупреждения области инженерной применимости**:
   - `UNDERCUT_SEVERE`: $z < 12$ — сильный подрез ножки зуба, резкое снижение прочности;
   - `UNDERCUT_RISK`: $z < 17$ — теоретический риск подреза для стандартных некорригированных зубьев 20°;
   - `FACE_WIDTH_TOO_NARROW`: $b/m < 6.0$ — узкий зубчатый венец, чувствителен к перекосам валов;
   - `FACE_WIDTH_TOO_WIDE`: $b/m > 16.0$ — широкий венец, концентрация нагрузки на кромках из-за прогиба валов;
   - `NON_STANDARD_PRESSURE_ANGLE`: $|\alpha - 20^\circ| > 0.5^\circ$;
   - `BENDING_SAFETY_FACTOR_CRITICAL`: $\text{FoS}_b < 1.0$;
   - `CONTACT_SAFETY_FACTOR_CRITICAL`: $\text{FoS}_H < 1.0$.

---

## 3. Выполненные изменения

### 3.1. `crates/mcp-cad-core/src/gear_strength.rs` и `crates/mcp-cad-core/src/lib.rs`
- Создан модуль `gear_strength.rs` с функциями:
  - `lewis_form_factor(teeth, pressure_angle_deg)` — точная таблица Shigley Table 14-2 с интерполяцией;
  - `barth_dynamic_factor_si(pitch_line_velocity_m_s, method)` — динамический фактор Барта в единицах СИ;
  - `transmitted_tangential_force(torque_nm, pitch_diameter_m)` — вычисление $W_t$;
  - `lewis_bending_stress_pa(wt_n, face_width_m, module_m, lewis_y, kv)` — напряжение изгиба;
  - `hertz_elastic_coefficient_pa(e1, nu1, e2, nu2)` — упругий коэффициент $C_p$;
  - `hertz_contact_stress_pa(wt_n, face_width_m, alpha_rad, d1_m, d2_m, cp_pa, kv)` — контактное напряжение Герца;
  - `evaluate_gear_strength(&params) -> GearStrengthReport` — комплексный расчёт с формированием отчёта, предупреждений, допущений и библиографических ссылок.
- Добавлены структуры:
  - `GearManufacturingMethod` (`CommercialCut`, `PrecisionGround`, `Static`);
  - `GearStrengthCalculationParams`;
  - `GearStrengthReport`.
- Экспортированы функции и типы в `crates/mcp-cad-core/src/lib.rs`.

### 3.2. `crates/mcp-cad-server/src/tools/cad.rs`
- Реализована функция `check_gear_strength(args: &Value, state: &mut EngineState) -> Result<Value, ServerError>`:
  - Поддерживает автономный вызов по числовым параметрам (`module`, `teeth`, `face_width`, `torque`) либо вызов по `part_id` (с автоматическим извлечением параметров шестерни из узла `FeatureOp::AddSpurGear` в DAG и материала детали).
  - Поддерживает задание нагрузки как крутящим моментом (`torque` в Н·м), так и парой мощности (`power_watts` / `power_kw`) и частоты вращения (`speed_rpm`).
  - Строго соблюдает контракт единиц `length_unit` (`"mm"` по умолчанию или `"m"`).
  - Возвращает поля активного `length_unit` и суффиксные поля СИ (`_mm`, `_mpa`, `_n`).
  - Интегрирован со слоем требований `super::requirements::attach_auto_check_if_enabled`.

### 3.3. `crates/mcp-cad-server/src/tools/mod.rs`
- Зарегистрирован 37-й MCP-инструмент `check_gear_strength` в `list_tools()` с подробной JSON-схемой, диапазонами, перечислениями и примерами.
- Добавлена ветка `"check_gear_strength"` в `dispatch_tool()`.
- Счётчик инструментов обновлён до 37.

---

## 4. Результаты верификации и тестов

### 4.1. Ручной аналитический расчёт по критериям приёмки
Входные параметры:
- $m = 3\text{ мм} = 0.003\text{ м}$,
- $z_1 = 20$,
- $b = 30\text{ мм} = 0.030\text{ м}$,
- $T = 100\text{ Н}\cdot\text{м}$,
- $K_v = 1.0$ (`manufacturing_method = "static"`),
- Материал: конструкционная сталь AISI 1020 ($S_y = 250\text{ МПа}, E = 205\text{ ГПа}, \nu = 0.29$),
- Парное колесо: $z_2 = 20$.

Результаты:
1. **Делительный диаметр**: $d_1 = 3 \times 20 = 60.0\text{ мм}$ (0.060 м).
2. **Окружная сила**: $W_t = \frac{2 \times 100}{0.060} = 3333.333\text{ Н}$.
3. **Коэффициент формы зуба Льюиса**: для $z=20, \alpha=20^\circ \implies Y = 0.3220$.
4. **Изгибные напряжения по Льюису**:
   $$\sigma_b = \frac{1.0 \times 3333.333}{0.030 \times 0.003 \times 0.322} = \frac{3333.333}{2.898 \times 10^{-5}} = 115021854\text{ Па} = 115.02\text{ МПа}.$$
   *(Полное совпадение с критерием приёмки $\sigma \approx 115.0\text{ МПа}$!)*
5. **Коэффициент запаса по изгибу**:
   $$\text{FoS}_b = \frac{250}{115.02} \approx 2.173.$$
6. **Контактные напряжения Герца**:
   - $r_1 = r_2 = \frac{0.060 \sin 20^\circ}{2} = 0.0102606\text{ м}$ ($10.261\text{ мм}$).
   - $\frac{1}{r_1} + \frac{1}{r_2} \approx 194.920\text{ м}^{-1}$.
   - $C_p = \sqrt{\frac{1}{\pi \times 2 \times (1 - 0.29^2) / 2.05 \times 10^{11}}} \approx 1.8874 \times 10^5\text{ Па}^{0.5} = 188.74\text{ МПа}^{0.5}$.
   - $\sigma_H = 1.8874 \times 10^5 \times \sqrt{\frac{3333.333}{0.030 \cos 20^\circ} \times 194.920} = 1.8874 \times 10^5 \times 4800.77 = 9.061 \times 10^8\text{ Па} = 906.1\text{ МПа}$.

### 4.2. Набор unit-тестов
- `mcp-cad-core`:
  - `test_lewis_form_factor_table_points` — проверка узловых точек таблицы 14-2 ($z=12, 17, 20, 21, 400$);
  - `test_acceptance_criteria_manual_calculation` — проверка базового эталона;
  - `test_dynamic_barth_factor_and_velocity` — проверка расчёта окружной скорости ($V = 4.712\text{ м/с}$), $K_v = 1.7725$, динамического изгиба $\sigma_b = 203.88\text{ МПа}$ и контактных напряжений с парным колесом $z_2 = 40$;
  - `test_undercut_and_face_width_warnings` — проверка детекции опасного подреза ($z=10 < 12$) и узкого венца ($b/m = 4.0 < 6.0$).
- `mcp-cad-server`:
  - `test_check_gear_strength_manual_criteria` — серверный unit-тест эталонного расчёта;
  - `test_check_gear_strength_with_part_in_scene_and_power_input` — тест связки с деталью в сцене и расчёта через $P = 10\text{ кВт}, n = 1500\text{ об/мин}$;
  - `test_check_gear_strength_units_m` — проверка работы с единицами в метрах (`length_unit = "m"`).
- Контрактный тест JSON-RPC (`crates/mcp-cad-server/tests/mcp_contract_tests.rs`):
  - `test_mcp_p3_t8_check_gear_strength_contract` — проверка регистрации инструмента в `tools/list`, валидации обязательных аргументов, контракта единиц и кодов ошибок `INVALID_ARGUMENT`.

---

## 5. Обновлённая документация
- `docs/MCP_API_REFERENCE.md` — добавлен раздел 37 `check_gear_strength` с полным описанием математики, таблицы параметров, примерами JSON-запроса и ответа;
- `PROJECT_CONTEXT.md` — обновлена таблица раздела 8 (добавлен 37-й инструмент) и общее число инструментов (37);
- `docs/AGENT_GUIDE.md` — добавлен практический Пример 4 по расчёту зубчатых передач;
- `SKILL.md` — добавлена строка `check_gear_strength` в таблицу специализированных машиностроительных инструментов;
- `TASKS.md` — статус задачи `P3-T8` переведён в `✅ DONE`.
