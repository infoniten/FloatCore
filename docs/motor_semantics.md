# Семантика motor API и правила агрегации двух VESC

Для каждой функции проверено, **что именно** делает Refloat со значением (по всем местам
использования), и только из этого выведено правило агрегации. Ни одно правило не взято
«по умолчанию» как среднее или сумма — как и требует ТЗ v0.2 §2.

Базовый факт, из которого следует всё остальное:

> `mc_set_current(X)` разворачивается в «X ампер **каждому** из двух ESC».
> Значит, все величины, которые Refloat сравнивает с этим током или из которых его
> вычисляет, обязаны быть в том же пространстве — **на один мотор**.

---

## 1. Сводная таблица

| VESC API | Семантика в Refloat | Реализация на ESP32 | Правило агрегации A/B |
|---|---|---|---|
| `mc_set_current` | основная команда баланса, `motor_control.c:57,99,108` | `CAN_PACKET_SET_CURRENT` обоим ESC | **дублирование**: каждому ESC уходит одно и то же значение (с учётом `scale`/`invert`) |
| `mc_set_brake_current` | тормоз в покое, `motor_control.c:117` | `CAN_PACKET_SET_CURRENT_BRAKE` | **дублирование** |
| `mc_set_duty` | парковочный тормоз, `motor_control.c:114` | `CAN_PACKET_SET_DUTY` | **дублирование** |
| `mc_set_current_off_delay` | удержание модуляции 50 мс, `motor_control.c:98` | параметр к команде тока | **дублирование** |
| `timeout_reset` | сброс watchdog прошивки, `motor_control.c:93` | продление watchdog LogicalMotor | — |
| `mc_get_rpm` | ERPM: скорость, направление, ускорение, все пороги | STATUS_1 | **среднее** + контроль расхождения |
| `mc_get_duty_cycle_now` | duty tiltback (защита) | STATUS_1 | **max(\|A\|,\|B\|)** |
| `mc_get_tot_current` | **не используется Refloat** | — | — |
| `mc_get_tot_current_filtered` | знак → флаг `braking`; величина → только телеметрия | STATUS_1 | **среднее** |
| `mc_get_tot_current_directional_filtered` | → biquad → момент → ATR, saturation | STATUS_1 со знаком по ERPM | **среднее** |
| `mc_get_tot_current_in_filtered` | battery saturation vs `l_in_current_max` | STATUS_4 | **среднее** |
| `mc_get_tot_current_in` | только LCM-дисплей, `lcm.c:109` | STATUS_4 | **сумма** |
| `mc_get_input_voltage_filtered` | LV/HV tiltback | STATUS_5 | **среднее** + контроль расхождения |
| `mc_temp_fet_filtered` | температурный tiltback | STATUS_4 | **max(A,B)** |
| `mc_temp_motor_filtered` | температурный tiltback | STATUS_4 | **max(A,B)** |
| `mc_get_fault` | алерт (НЕ останов) | опрос обоих | **первый ненулевой**, оба сохраняются |
| `mc_get_speed` | км/ч для UI и tiltback-ов | расчёт из среднего ERPM | производная от ERPM |
| `mc_get_distance` | reverse stop, LED | STATUS_5 (тахометры) | **среднее** |
| `mc_get_amp_hours`, `..._charged`, `mc_get_watt_hours`, `..._charged` | статистика в UI | STATUS_2/3 | **сумма** |

---

## 2. Обоснование по местам использования

### 2.1. `mc_get_tot_current_filtered` → среднее

Места использования (единственное присваивание — `motor_data.c:140`):

```c
m->current = VESC_IF->mc_get_tot_current_filtered();   // motor_data.c:140
m->braking = m->current < 0;                            // motor_data.c:142
```

Дальше `m->current` уходит только в телеметрию (`main.c:1373`, `main.c:2118`,
`rt_data.h:44`), а `m->braking` — в ATR (`atr.c:85,87,127`), booster (`booster.c:43,57`)
и brake tilt (`brake_tilt.c:75`).

Знак при дублирующей команде тока у обоих ESC одинаков, поэтому для флага `braking`
среднее и сумма эквивалентны. Но значение показывается пользователю как «ток мотора»,
и в дуальной конфигурации корректный ответ — ток **одного** мотора: именно его пользователь
сравнивает с лимитом `l_current_max` в настройках ESC. **Среднее.**

### 2.2. `mc_get_tot_current_directional_filtered` → среднее (ключевой случай)

```c
m->dir_current = VESC_IF->mc_get_tot_current_directional_filtered();  // motor_data.c:141
biquad_update(&m->filt_current, m->dir_current);                       // :150
m->torque = m->filt_current.value / m->speed_constant;                 // :151
...
m->motor_current_saturation = fabsf(m->filt_current.value) / motor_current_limit;  // :163
```

где `motor_current_limit` — это `m->current_max` / `m->current_min`, полученные из
`get_cfg_float(CFG_PARAM_l_current_max / l_current_min)`, то есть **лимиты одного ESC**.

Если отдать сумму A+B, то `motor_current_saturation` окажется завышенной ровно вдвое.
Эта величина через `motor_data_get_current_saturation()` управляет ATR и haptic-обратной
связью, а `m->torque` — оценкой уклона в ATR. Результат: ATR постоянно «видит» подъём,
haptic срабатывает на половине реального тока.

**Среднее.** Формально эквивалентная альтернатива — сумма токов вместе с удвоением всех
лимитов из `get_cfg_float`, но её легко нарушить при обновлении, и она делает показания
UI непохожими на настройки ESC.

### 2.3. `mc_get_tot_current_in_filtered` → среднее

```c
ema_update(&m->batt_current, VESC_IF->mc_get_tot_current_in_filtered());  // motor_data.c:158
...
float battery_current_limit = m->batt_current.value < 0
    ? m->battery_current_min : m->battery_current_max;                     // :168
m->battery_current_saturation = fabsf(m->batt_current.value) / battery_current_limit;
```

Лимиты — снова per-ESC (`l_in_current_max/min`). Та же логика, что и в 2.2: **среднее**.

Отдельно: `lcm.c:109` использует `mc_get_tot_current_in()` для дисплея LCM. Там речь о
потреблении **пакета**, поэтому там — **сумма**. Две разные функции с разной семантикой,
и это не противоречие: одна кормит контроль насыщения, другая — индикатор.

### 2.4. `mc_get_duty_cycle_now` → max(|A|,|B|)

```c
m->duty_raw = fabsf(VESC_IF->mc_get_duty_cycle_now());   // motor_data.c:144
ema_update(&m->duty_cycle, m->duty_raw);
```

Знак отбрасывается сразу, поэтому «сохранение направления» из ТЗ v0.1 §6 не требуется.
Duty питает duty tiltback — защиту от вылета за пределы модуляции. Консервативный выбор —
**максимум по модулю**: если один мотор уже упёрся в потолок, тормозить надо всей доской.

### 2.5. `mc_get_rpm` → среднее + контроль расхождения

ERPM входит в `m->erpm`, `m->abs_erpm`, `m->erpm_sign`, ускорение (`sma_update` от
`(erpm - last_erpm)/dt`), определение направления, пороги всех tiltback-ов и условия
старта/останова. Два колеса на одной доске физически движутся с одной скоростью,
поэтому **среднее** — верная модель, а расхождение — признак неисправности
(проскальзывание, неверная инверсия, отказ ESC).

Порог расхождения задаётся `LogicalMotorConfig.erpm_mismatch_limit`; превышение →
`LM_FAULT_ERPM_MISMATCH` → снятие тяги supervisor-ом.

### 2.6. `mc_get_input_voltage_filtered` → среднее

Питает LV/HV tiltback и определение уровня заряда. Батарея общая, поэтому расхождение
показаний двух ESC — это ошибка измерения или проблема с проводкой. **Среднее**, при
расхождении > `voltage_mismatch_limit` (1 В) — диагностический флаг.

Важно: при протухании телеметрии **нельзя** подставлять 0 — это вызовет ложный LV-tiltback
на полном ходу. Правильное поведение — fault и снятие тяги.

### 2.7. Температуры → max(A,B)

Оба значения питают температурный tiltback (`l_temp_fet_start - 3`, `l_temp_motor_start - 3`).
Перегрев любого из контроллеров — повод снижать мощность. **Максимум.**

### 2.8. `mc_get_fault` → первый ненулевой, оба сохраняются

Сценарий 9 host-харнесса подтвердил: Refloat по этому коду **не останавливается** —
`motor_data_evaluate_alerts()` только добавляет алерт, `check_faults()` код фолта не
смотрит. Состояние остаётся `RUNNING`.

Следствия:

* правило «fault если fault у любого ESC» корректно, но само по себе ничего не защищает;
* в перечислении `mc_fault_code` **нет кода для потери связи с ESC**, и придумывать
  синтетический не нужно: реакция на потерю CAN — задача supervisor-а, а не Refloat;
* коды обоих ESC нужно хранить раздельно (`LogicalMotorTelemetry.esc_fault_code[2]`) —
  иначе UI покажет неполную картину.

### 2.9. Пробег и энергия

`mc_get_distance` / `mc_get_distance_abs` — это перемещение **доски**, а не сумма путей
двух колёс: **среднее**. `amp_hours` / `watt_hours` — расход из общего пакета: **сумма**.

---

## 3. Инверсия, масштаб и лимиты

Порядок преобразований обязателен и одинаков в обе стороны:

**Команда (Refloat → ESC):**

```
X = запрошенный ток (на один мотор)
  → проверка isfinite(X)                       иначе LM_FAULT_INVALID_REQUEST
  → для каждого ESC i:
        v = X * scale[i]
        v = invert[i] ? -v : v
        |v| <= current_limit[i]                иначе клип + fault
  → CAN_PACKET_SET_CURRENT(can_id[i], v)
```

**Телеметрия (ESC → Refloat):**

```
для каждого ESC i: снять инверсию у знаковых величин (ERPM, duty, направленный ток)
  → проверка возраста снимка (<= telemetry_timeout_s)
  → агрегация по правилам §1
```

Забытое снятие инверсии на приёме — самая вероятная ошибка интеграции: ERPM двух моторов
окажется противоположным по знаку, среднее даст ≈0, доска будет считать себя неподвижной
на полном ходу. Именно поэтому контроль расхождения ERPM обязателен и служит одновременно
проверкой правильности конфигурации.

---

## 4. Что проверяет mock-реализация

`compat/motor/logical_motor.h` — контракт, `tests/host/mock/logical_motor_mock.c` —
реализация без вывода на CAN. На этапе 0.2 она:

* принимает запросы `requestCurrent` / `requestBrakeCurrent` / `requestDuty` / `release`;
* отвергает NaN/Inf и значения вне `current_limit`, выставляя `LM_FAULT_INVALID_REQUEST`;
* применяет `scale` и `invert` и сохраняет значения, которые ушли бы каждому ESC;
* считает `would_send_can_frames` — сколько кадров ушло бы в боевом режиме;
* моделирует пропажу ESC и фолты прошивки для сценариев 7–9.

Проверено сценарием 4: на 0.1 с работы приходится 50 запросов тока и 100 «кадров» —
ровно 2 кадра на итерацию контура, как заложено в расчёт
[can_bandwidth.md](can_bandwidth.md).
