# Архитектура Refloat (как есть, upstream `47a2c5ce`, v1.3.0)

Документ описывает устройство Refloat в том виде, в каком он работает на VESC. Ничего не
изменено; это база для проектирования ESP32-слоя совместимости.

## 1. Что такое пакет Refloat физически

`.vescpkg` собирается `vesc_tool --buildPkgFromDesc pkgdesc.qml` и содержит:

| Часть | Файл | Роль |
|---|---|---|
| Нативный код | `src/` → `package_lib.bin` (PIC, Cortex-M4) | **вся логика баланса** |
| Lisp-glue | `lisp/package.lisp`, 17 строк | загружает бинарник, передаёт версию FW, стартует BMS-поток |
| BMS-поллер | `lisp/bms.lisp`, 47 строк | опционально, 5 Гц |
| UI | `ui.qml.in` | конфигуратор в VESC Tool |
| Схема настроек | `src/conf/settings.xml` | из неё `vesc_tool` генерирует парсер и XML-блоб |

Бинарник линкуется по `vesc_pkg_lib/link.ld` в 96 КБ адресного пространства и загружается
LispBM-примитивом `load-native-lib`. Единственная точка контакта с прошивкой — структура
указателей на функции по фиксированному адресу `0x1000F800` (`VESC_IF`).

## 2. Модель времени и потоки

```
                       ┌────────────────────────────────────────────┐
   IMU прошивки  ──────►│ imu_ref_callback()      ~832 Гц (IMU rate) │
   (semp. rate)        │  balance_filter (Mahony) → imu_update       │
                       │  → pid_control() → motor_control_apply()    │
                       │  → mc_set_current()                        │
                       └────────────────┬───────────────────────────┘
                                        │ читает d->setpoint
                       ┌────────────────┴───────────────────────────┐
   VESC_IF->spawn ────►│ refloat_thd()            500 Гц            │
                       │  motor_data_update, footpad, state machine, │
                       │  faults, setpoint, tiltbacks, beeper, BMS   │
                       └────────────────────────────────────────────┘
                       ┌────────────────────────────────────────────┐
   VESC_IF->spawn ────►│ aux_thd()   prio −1,  ~LEDS_REFRESH_RATE   │
                       │  LEDs, одометр, refresh конфигурации мотора │
                       └────────────────────────────────────────────┘
```

Ключевые факты:

* **Команда мотору выдаётся из IMU-callback**, не из главного цикла. Частота вывода = частота
  IMU. `motor_control_apply()` гарантирует, что на каждой итерации мотору что-то задано
  (ток / duty / brake) — на этом держится watchdog прошивки (`timeout_reset()`).
* Обе частоты не захардкожены: `frequency_tracker` измеряет реальную частоту и при её
  изменении вызывает `main_freq_update_reconfigure()` / `imu_freq_update_reconfigure()`,
  которые пересчитывают коэффициенты всех фильтров (EMA/SMA/biquad), PID, booster,
  data recorder. Это очень удобно для порта: **достаточно, чтобы частоты были стабильны, а не
  равны конкретному числу.**
* Синхронизации между потоками нет — ни мьютексов, ни семафоров. Общий `Data *d` пишется и
  читается без блокировок.

## 3. Цепочка управления

```
IMU семпл (acc, gyro, dt)
   │
   ├─ balance_filter_update()   Mahony AHRS Refloat → balance_pitch
   ├─ imu_update()              pitch/roll/yaw из AHRS прошивки + pitch_rate из gyro
   │
   ▼  pid_control(d, dt)
   setpoint (из главного цикла: tiltback-и, ATR, torque tilt, turn tilt, brake tilt,
             input tilt, reverse stop, booster, smooth setpoint)
   │
   ├─ P/I/D по (setpoint − balance_pitch), rate_p по pitch_rate
   ├─ torque → current: motor_data_torque_to_current() (через flux linkage / speed constant)
   ├─ ограничение по current_max / current_min (из конфигурации VESC)
   ├─ EMA-сглаживание 25 Гц
   ▼
   motor_control_request_current()
   ▼
   motor_control_apply()  ──► mc_set_current() | mc_set_duty() | mc_set_brake_current()
```

Важно: PID работает **в моменте** и в единицах момента, а перевод момент→ток использует
`flux_linkage` и число полюсов из конфигурации VESC. При двух моторах это означает, что
константа момента должна соответствовать **одному** мотору (см. `refloat_vesc_api_dependencies.md`
§3.1).

## 4. Конечный автомат

`STATE_STARTUP → STATE_READY → STATE_RUNNING → (fault) → STATE_READY`, плюс `STATE_DISABLED`.
Дополнительно `RunMode`: normal, `MODE_FLYWHEEL`, `MODE_HANDTEST`, а также флаг `darkride`
(перевёрнутая доска, инверсия pitch_rate и тока).

Условия старта (`STATE_READY → RUNNING`): оба футпада, |balance_pitch| и |roll| < 45°,
допуск по pitch, ERPM. Условия останова (`check_faults()`): угол pitch/roll, состояние
футпадов с таймаутами, reverse stop, ERPM.

## 5. Подсистемы (все переносятся без изменений)

| Модуль | Назначение | Платформенные зависимости |
|---|---|---|
| `balance_filter.c` | Mahony AHRS Refloat | только `imu_get_quaternions` на init |
| `pid.c`, `booster.c` | контур управления | нет |
| `atr.c`, `torque_tilt.c`, `brake_tilt.c`, `turn_tilt.c` | адаптивные наклоны | нет |
| `reverse_stop.c`, `remote.c` | обратный ход, пульт | `get_ppm*`, `get_remote_state` |
| `motor_data.c` | телеметрия + фильтры + saturation | `mc_get_*`, `get_cfg_*` |
| `motor_control.c` | вывод на мотор, тон, парковочный тормоз | `mc_set_*`, `timeout_reset` |
| `footpad_sensor.c` | пороги/гистерезис/swap футпадов | `io_read_analog` |
| `state.c`, `alert_tracker.c`, `haptic_feedback.c` | состояния, алерты, haptic | `foc_play_tone` |
| `leds.c`, `led_strip.c` | анимации LED | через `led_driver.h` |
| `led_driver.c` | **STM32 TIM+DMA для WS2812** | **непортируем** |
| `data_recorder.c` | запись сессий, `plot_*` | `plot_*` (заглушки) |
| `lcm.c` | протокол Light Control Module | нет |
| `charging.c`, `bms.c` | зарядка, BMS | `system_time`, `ext_bms` |
| `conf/confparser.c`, `confxml.c` | **генерируются `vesc_tool` из `settings.xml`** | build-time |
| `filters/`, `lib/` | EMA, SMA, biquad, кольцевой буфер, утилиты | нет |

## 6. Конфигурация и UI

* Настройки описаны в `src/conf/settings.xml`; `vesc_tool --xmlConfToCode` генерирует
  `confparser.{c,h}` (сериализация), `confxml.{c,h}` (сжатый XML для VESC Tool) и
  `conf_default.h`.
* Runtime: `conf_custom_add_config(get_cfg, set_cfg, get_cfg_xml)` — VESC Tool забирает XML,
  рисует UI и шлёт обратно бинарный блоб.
* Хранение: сериализованный блоб (≤320 байт) пишется в EEPROM VESC словами по 4 байта
  через `store_eeprom_var`/`read_eeprom_var`, по явной команде `COMMAND_CFG_SAVE`.
* Собственный протокол Refloat поверх `COMM_CUSTOM_APP_DATA` — ~25 команд,
  задокументирован в `doc/commands/`.

## 7. Что из этого важно для ESP32

1. Логика уже нативная C99 — интерпретатор не нужен.
2. Вся платформа изолирована в одной структуре указателей на функции.
3. Частоты контуров самонастраиваются — жёстких требований к точным 500/832 Гц нет,
   требуется стабильность.
4. Refloat никогда не обращается к CAN — подмена «один мотор» → «два по CAN» ему не видна.
5. Единственный непортируемый файл — LED-драйвер.
