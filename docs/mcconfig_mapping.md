# Отображение: FloatCore Config → Virtual mcConfig → QML

## 1. Цепочка

```
        физические ESC (CAN)          собственные пределы FloatCore
                │                                  │
                ▼                                  ▼
        ┌──────────────────────────────────────────────────┐
        │   FloatCore Config   compat/config/floatcore_limits.h
        │   FcSourceLimits esc[2] · FcSourceLimits floatcore │
        │   FcBatteryConfig battery                          │
        └───────────────┬──────────────────────────────────┘
                        │ агрегация: самое консервативное
                        ▼
        ┌──────────────────────────────────────────────────┐
        │  fc_effective_current_max() и остальные getters    │
        └──────┬────────────────────────────────┬───────────┘
               │                                │
               ▼                                ▼
   VESC_IF->get_cfg_float()          VirtualMcConfValues
        (Refloat, балансировка)        (заполняется на каждый запрос)
                                              │
                                              ▼
                                   virtual_mcconf_encode()
                                              │ COMM_GET_MCCONF
                                              ▼
                                       VescIf.mcConfig()
                                              │
                                              ▼
                                        Refloat QML
```

Ключевое свойство: **обе ветки выходят из одного узла**. Значение, по которому
Refloat ограничивает ток, и значение, по которому QML рисует шкалу, — это
результат одного и того же вызова.

## 2. Таблица отображения

| FloatCore Config | Агрегация | `VirtualMcConfValues` | Параметр mcconf | Где в QML |
|---|---|---|---|---|
| `esc[i].current_max`, `floatcore.current_max` | `min` | `l_current_max` | `l_current_max` | `:6818` правая граница шкалы тока |
| `esc[i].current_min`, `floatcore.current_min` | ближайшее к нулю (`max`, значения < 0) | `l_current_min` | `l_current_min` | `:6817` левая граница шкалы тока |
| `esc[i].in_current_max`, `floatcore.in_current_max` | `min` | `l_in_current_max` | `l_in_current_max` | `:6884` шкала тока батареи |
| `esc[i].in_current_min`, `floatcore.in_current_min` | ближайшее к нулю | `l_in_current_min` | `l_in_current_min` | `:6883` шкала тока батареи |
| `esc[i].temp_fet_start`, `floatcore.temp_fet_start` | `min` | `l_temp_fet_start` | `l_temp_fet_start` | `:7465` порог предупреждения |
| `esc[i].temp_fet_end` | `min` | `l_temp_fet_end` | `l_temp_fet_end` | — (согласованность) |
| `esc[i].temp_motor_start` | `min` | `l_temp_motor_start` | `l_temp_motor_start` | `:7457` порог предупреждения |
| `esc[i].temp_motor_end` | `min` | `l_temp_motor_end` | `l_temp_motor_end` | — (согласованность) |
| `battery.cell_count` | — (общая батарея) | `si_battery_cells` | `si_battery_cells` | `:6580` В на ячейку; `:1400` миграция тюна |

Все прочие ~193 параметра схемы отдаются со значениями по умолчанию самой схемы
VESC Tool. Проекции для них нет и не должно быть.

## 3. Обоснование правил агрегации

ТЗ требует подтверждать каждое правило анализом использования, а не
предположением. Ниже — по каждому параметру.

### `l_current_max` → `min(A, B, FloatCore)`

Refloat читает его в `motor_data_refresh_motor_config()` и использует в
`pid_control()` как жёсткий потолок вычисленного тока:

```c
current_limit = d->motor.braking ? d->motor.current_min : d->motor.current_max;
if (fabsf(new_current) > current_limit) {
    new_current = sign(new_current) * current_limit;
}
```

Запрошенный ток **дублируется** каждому ESC (см. `motor_semantics.md` §1),
поэтому предел — это предел одного мотора, а не суммы. Если один ESC способен
на 40 А, а второй только на 18 А, безопасный общий потолок — 18 А. Сумма
(58 А) означала бы перегрузку слабого контроллера. **Минимум.**

### `l_current_min` → ближайшее к нулю

Значения отрицательные. «Самое консервативное» здесь — наименьшее по модулю,
то есть максимум из отрицательных: из (−20, −3, −5) выбирается −3. Тот же
аргумент: тормозной ток дублируется, слабейший ESC задаёт границу.

Дополнительно это значение задаёт левую границу шкалы: показать пользователю
−20 А, когда система не тормозит сильнее −3 А, значит нарисовать шкалу,
которая никогда не заполняется.

### `l_in_current_max` / `l_in_current_min` → так же

Refloat вычисляет `battery_current_saturation = |batt_current| / battery_current_limit`
и по нему управляет ATR и haptic. Ток батареи агрегируется как **среднее**
(`motor_semantics.md` §2.3), значит и лимит должен быть per-ESC. **Минимум**
по модулю, отдельно для положительного и отрицательного направления.

### Температурные пороги → `min`

Refloat берёт `l_temp_fet_start - 3` как порог температурного tiltback.
Температуры агрегируются как `max(A, B)` — берётся самый горячий контроллер.
Значит порог должен соответствовать самому чувствительному: если один ESC
начинает снижать мощность с 75 °C, а другой с 90 °C, общий порог — 75 °C.
Использовать максимум означало бы игнорировать ограничения слабого ESC.
**Минимум.**

Пороги `end` агрегируются так же, чтобы пара `start`/`end` осталась
согласованной.

### `si_battery_cells` → без агрегации

Батарея одна, физически общая для обоих ESC. Значение берётся из
`FcBatteryConfig`, а не из настроек ESC. Расхождение настроек ESC по числу
ячеек — это ошибка конфигурации, а не повод что-то усреднять.

### Чего здесь нет

Ни один предел не суммируется. Формулы вида `escA.limit + escB.limit`
в коде отсутствуют; вместо них `aggregate(..., PICK_MIN)` либо `PICK_MAX`
для отрицательных величин. Источник, у которого `present == false`
(ESC вне связи), в агрегации не участвует.

## 4. Мок и будущая замена реальными данными

Сейчас `esc[0]`/`esc[1]` не заполнены (`present == false`), и агрегация
вырождается в собственные пределы FloatCore, которые на host задаются флагом:

```bash
build/floatcore_host --limits cells=10,imax=25,imin=-5,inmax=15,inmin=0,fet_start=80,fet_end=100
```

Когда появится CAN, драйвер будет вызывать `floatcore_limits_set_esc()` с
данными, вычитанными из `mcconf` реальных FSESC. Ни `VirtualMcConfValues`,
ни `virtual_mcconf_encode()`, ни QML при этом не меняются — меняется только
наполнение источника:

```
Host mock ──┐
            ├──► floatcore_limits_set_esc() ──► агрегация ──► Virtual mcConfig ──► UI
Real CAN ───┘
```

## 5. Безопасность

Virtual mcConfig — только исходящий путь. Safety Supervisor его не читает и
читать не должен: он работает с `floatcore_limits()` и телеметрией напрямую.
Изменение проекции физически не может повлиять на балансировку, потому что
данные текут только в одну сторону, а обработчика `COMM_SET_MCCONF` не
существует (команда считается и игнорируется).
