# Refloat → VESC / LispBM: полный инвентарь зависимостей

**Объект аудита:** `lukash/refloat`, ветка `main`, коммит `47a2c5ce` (версия 1.3.0, 2026-08-23).
**Правило:** исходники Refloat не изменялись. Всё ниже получено чтением кода.

---

## 0. Главный вывод аудита (влияет на выбор стратегии)

**Refloat — это не LispBM-программа.** Refloat — нативный модуль на C, компилируемый в
position-independent бинарник для Cortex-M4 и загружаемый VESC-прошивкой через LispBM-функцию
`load-native-lib`. Вся Lisp-часть пакета — 17 строк glue-кода (`lisp/package.lisp`) плюс
опциональный BMS-поллер (`lisp/bms.lisp`, ~47 строк).

Практическое следствие: балансировочная логика **не проходит через интерпретатор LispBM ни на
одном такте**. Стратегия A из ТЗ («запуск оригинального Refloat поверх LispBM на ESP32»)
не имеет предмета — переносить на LispBM нечего. Подробности и вытекающий выбор — в
[porting_strategy.md](porting_strategy.md).

**Вся связь Refloat с прошивкой VESC идёт через одну структуру указателей на функции:**

```c
#define VESC_IF ((vesc_c_if*)(0x1000F800))
```

Файл `vesc_pkg_lib/vesc_c_if.h` — часть SDK VESC, а **не** исходников Refloat. Из 200+ членов
структуры Refloat использует **74** различных члена. Ни один файл Refloat, кроме `led_driver.{c,h}`, не включает
ничего платформенного: только `<math.h>`, `<string.h>`, `<stdint.h>`, `<stdbool.h>`, `<stddef.h>`,
`<stdlib.h>` и `vesc_c_if.h`.

Проверено:

```
$ grep -rn '#include' src/ | grep -v '"local headers"' | grep -v '<std\|<math\|<string\|vesc_c_if'
src/led_driver.h:24:#include "st_types.h"     ← единственная платформенная зависимость
```

---

## 1. Realtime path: что именно вызывается в балансировочном контуре

В Refloat **два** периодических контекста плюс один фоновый.

### 1.1. IMU-callback — это и есть контур баланса

`src/main.c:734 imu_ref_callback(float *acc, float *gyro, float *mag, float dt)`,
зарегистрирован через `VESC_IF->imu_set_read_callback()` (`main.c:2710`).
Вызывается прошивкой на **частоте семплирования IMU** (на VESC обычно 832 Гц,
`CFG_PARAM_IMU_sample_rate`).

Полная цепочка внутри callback:

```
imu_ref_callback(acc, gyro, mag, dt)
  ├─ latency_tracker_update()
  ├─ vesc_system_time_ticks()          → VESC_IF->system_time_ticks()
  ├─ balance_filter_update()           Mahony AHRS, чистая математика
  ├─ frequency_tracker_update()
  ├─ imu_update()                      → imu_get_roll/pitch/yaw + imu_get_gyro
  ├─ if STATE_RUNNING: pid_control()   → расчёт тока, ограничение по current_max
  ├─ if STATE_READY:   remote_get_move_torque()
  ├─ motor_control_apply()             → timeout_reset()
  │                                    → mc_set_current_off_delay() + mc_set_current()
  │                                      либо mc_set_duty() / mc_set_brake_current()
  └─ data_recorder_sample()
```

**Критично для архитектуры ESP32:** команда тока выдаётся из IMU-callback, то есть
на частоте IMU, а не на 500 Гц главного цикла. Каждая итерация = 1 команда мотору =
2 CAN-кадра в нашей дуальной конфигурации. Это задаёт требование к пропускной способности CAN
(см. [risk_register.md](risk_register.md), R-03).

### 1.2. Главный поток — 500 Гц

`src/main.c:767 refloat_thd()`, `#define MAIN_THREAD_FREQ 500` (`main.c:61`).
Спавнится через `VESC_IF->spawn(refloat_thd, 1536, "Refloat Main", d)`, приоритет по умолчанию.
Ритм задаётся `VESC_IF->sleep_us()` с компенсацией по `timer_seconds_elapsed_since()`.

Здесь: телеметрия мотора (`motor_data_update`), опрос футпадов, конечный автомат состояний,
проверка фолтов, расчёт setpoint и всех tiltback-ов, beeper, haptic, BMS, alert tracker.
Результат — `d->setpoint`, который читает PID в IMU-callback.

### 1.3. Aux-поток — низкий приоритет

`src/main.c:1109 aux_thd()`, `VESC_IF->thread_set_priority(-1)`, период `1e6/LEDS_REFRESH_RATE`.
LED-и, сохранение одометра, перечитывание конфигурации мотора раз в 0.5 с.

### 1.4. Итог: минимальный realtime-набор API

Ровно эти вызовы обязаны работать детерминированно и без блокировок:

| API | Контекст | Частота |
|---|---|---|
| `imu_set_read_callback` (сам механизм вызова) | — | регистрация |
| `imu_get_roll` / `imu_get_pitch` / `imu_get_yaw` | IMU cb | 1× IMU rate |
| `imu_get_gyro` | IMU cb | 1× IMU rate |
| `system_time_ticks` | IMU cb + main | ~1300 Гц суммарно |
| `mc_set_current` | IMU cb | 1× IMU rate |
| `mc_set_current_off_delay` | IMU cb | 1× IMU rate |
| `mc_set_duty`, `mc_set_brake_current` | IMU cb (idle/brake) | 1× IMU rate |
| `timeout_reset` | IMU cb | 1× IMU rate |
| `mc_get_rpm`, `mc_get_speed`, `mc_get_distance` | main | 500 Гц |
| `mc_get_tot_current_filtered`, `..._directional_filtered` | main | 500 Гц |
| `mc_get_duty_cycle_now` | main | 500 Гц |
| `mc_get_tot_current_in_filtered`, `mc_get_input_voltage_filtered` | main | 500 Гц |
| `mc_temp_fet_filtered`, `mc_temp_motor_filtered` | main | 500 Гц |
| `mc_get_fault` | main | 500 Гц |
| `io_read_analog` (ADC1/ADC2) | main | 500 Гц |
| `should_terminate`, `sleep_us`, `timer_time_now`, `timer_seconds_elapsed_since` | main | 500 Гц |
| `system_time` | main (таймеры/логи) | 500 Гц |
| `app_is_output_disabled` | main (только гейт логов) | 500 Гц |

Всё остальное — не realtime.

---

## 2. Полный инвентарь по категориям

Обозначения колонки **RT**: **RT-C** — в контуре баланса (IMU callback); **RT-M** — главный
поток 500 Гц; **AUX** — фоновый; **INIT** — только при старте/останове; **CMD** — по запросу
от UI/VESC Tool.

### 2.1. Motor output (вывод на мотор)

| API | Где используется | Назначение | Частота | RT | Реализация на ESP32 |
|---|---|---|---|---|---|
| `mc_set_current(float)` | `motor_control.c:57,99,108` | основная команда тока баланса; 0 A при disable | IMU rate | **RT-C** | **Прокси на Dual FSESC.** Через supervisor → 2× CAN `CAN_PACKET_SET_CURRENT` (VESC A/B), с инверсией/scale/лимитом на мотор |
| `mc_set_current_off_delay(float)` | `motor_control.c:98` | держать модуляцию 50 мс при токе ≈0 | IMU rate | **RT-C** | Прокси: `CAN_PACKET_SET_CURRENT_REL`+off_delay не эквивалентен; использовать `CAN_PACKET_SET_CURRENT` с off_delay-вариантом (в FW 6.06 есть `set_current_off_delay` по CAN) или эмулировать удержанием ненулевого тока |
| `mc_set_brake_current(float)` | `motor_control.c:117` | тормозной ток при остановленном/медленном движении | IMU rate | **RT-C** | Прокси → `CAN_PACKET_SET_CURRENT_BRAKE` ×2 |
| `mc_set_duty(float)` | `motor_control.c:114` | парковочный тормоз (duty=0, фазное закорачивание на 6.05+) | IMU rate | **RT-C** | Прокси → `CAN_PACKET_SET_DUTY` ×2 |
| `timeout_reset()` | `motor_control.c:93` | сброс сторожевого таймера прошивки VESC | IMU rate | **RT-C** | **Локально + семантически важно.** На ESP32 — no-op для локального стека, но именно *прекращение* отправки CAN-команд обеспечивает требуемый в ТЗ §11 watchdog на стороне FSESC (VESC глушит выход через `timeout_msec`, по умолчанию 1000 мс — уменьшить до ~100 мс в конфиге обоих ESC) |
| `foc_play_tone(int,float,float)` | `haptic_feedback.c:128,132` | haptic/звук через FOC-модуляцию | по событию | RT-M | **Не проксируется в v1.** По CAN команды нет. Оставить указатель NULL — Refloat уже проверяет `if (!VESC_IF->foc_play_tone) return;` |

Примечание: `motor_control_apply()` гарантирует, что **какая-то** команда мотору выдаётся на
каждой итерации (комментарий в коде: *«BEWARE: Some sort of motor control must always be set»*).
Это ровно то поведение, на которое опирается watchdog VESC.

### 2.2. Motor telemetry (телеметрия)

| API | Где используется | Назначение | Частота | RT | Реализация на ESP32 |
|---|---|---|---|---|---|
| `mc_get_rpm()` | `motor_data.c:127`, `leds.c:547` | ERPM: скорость, направление, ускорение, все пороги | 500 Гц | **RT-M** | Агрегация A/B (см. §3) |
| `mc_get_speed()` | `motor_data.c:137` | м/с → Refloat ×3.6 = км/ч | 500 Гц | **RT-M** | Считать самим из агрегированного ERPM и локального конфига (poles/gear/wheel) |
| `mc_get_distance()` | `motor_data.c:138`, `leds.c:1123,1213` | знаковый пробег (reverse stop, LED) | 500 Гц | **RT-M** | Агрегация (среднее), из тахометров A/B |
| `mc_get_distance_abs()` | `main.c:1386,2159` | абсолютный пробег для UI | CMD | CMD | Агрегация (среднее) |
| `mc_get_duty_cycle_now()` | `motor_data.c:144` | duty → duty tiltback (ключевая защита) | 500 Гц | **RT-M** | `max(|A|,|B|)` — Refloat берёт `fabsf()`, знак не нужен |
| `mc_get_tot_current_filtered()` | `motor_data.c:140` | ток мотора; `braking = current < 0` | 500 Гц | **RT-M** | Агрегация: **среднее** (см. §3.1 — не сумма!) |
| `mc_get_tot_current_directional_filtered()` | `motor_data.c:141` | направленный ток → torque → ATR | 500 Гц | **RT-M** | Среднее |
| `mc_get_tot_current_in_filtered()` | `motor_data.c:158` | ток батареи → battery saturation | 500 Гц | **RT-M** | Среднее (при среднем сравнении с per-ESC лимитом) |
| `mc_get_tot_current_in()` | `lcm.c:109` | ток батареи для LCM-дисплея | CMD | CMD | Сумма (это отображение потребления пакета) |
| `mc_get_input_voltage_filtered()` | `motor_data.c:159`, `lcm.c:110` | напряжение → LV/HV tiltback | 500 Гц | **RT-M** | Среднее; расхождение >1 В → флаг |
| `mc_temp_fet_filtered()` | `motor_data.c:176` | температура MOSFET → tiltback | 500 Гц | **RT-M** | `max(A,B)` |
| `mc_temp_motor_filtered()` | `motor_data.c:177` | температура мотора → tiltback | 500 Гц | **RT-M** | `max(A,B)` |
| `mc_get_fault()` | `motor_data.c:183`, `main.c:1333`, `lcm.c:99` | код фолта прошивки → alert | 500 Гц | **RT-M** | «Любой A/B ≠ NONE» (см. §3.2) |
| `mc_fault_to_string()` | `main.c:2212` | текст фолта для UI | CMD | CMD | Локальная таблица строк |
| `mc_get_battery_level(float*)` | `lcm.c:186`, `leds.c:584`, `main.c:1399,2127` | уровень заряда для LED/UI | AUX/CMD | AUX | Локальный расчёт по напряжению |
| `mc_get_amp_hours(bool)` | `main.c:1395,2163` | статистика | CMD | CMD | Сумма A+B |
| `mc_get_amp_hours_charged(bool)` | `main.c:1396,2169` | статистика | CMD | CMD | Сумма A+B |
| `mc_get_watt_hours(bool)` | `main.c:1397,2173` | статистика | CMD | CMD | Сумма A+B |
| `mc_get_watt_hours_charged(bool)` | `main.c:1398,2180` | статистика | CMD | CMD | Сумма A+B |
| `mc_get_odometer()` | `main.c:1131,1133,1239,1394,2156` | одометр (метры, uint64) | AUX/CMD | AUX | Собственный счётчик в NVS |
| `foc_get_id()` / `foc_get_iq()` | `main.c:1376,1377,2183` | Id/Iq для диагностики в UI | CMD | CMD | Из CAN status msg (Iq ≈ ток мотора); Id недоступен → 0 |
| `mc_gnss()` | `main.c:2193,2385` | GNSS-данные в UI/логах | CMD | CMD | Заглушка (нулевая структура) |

### 2.3. IMU

| API | Где используется | Назначение | Частота | RT | Реализация на ESP32 |
|---|---|---|---|---|---|
| `imu_set_read_callback(cb)` | `main.c:2710`, снятие `:2658` | **Тактовый генератор контура баланса.** Прошивка вызывает cb на каждый семпл IMU с `(acc[3], gyro[3], mag[3], dt)` | регистрация | **RT-C** | **Нативно.** Задача FreeRTOS высокого приоритета, привязанная к DRDY-прерыванию ICM-20948; из ISR только `vTaskNotifyGiveFromISR`, сам callback — в задаче (на ESP32 FPU в ISR использовать нельзя) |
| `imu_get_roll/pitch/yaw()` | `imu.c:36,38,40`, `leds.c:973` | углы **встроенного AHRS прошивки** (не Refloat-фильтра) | 1× IMU rate | **RT-C** | **Нативно.** Нужен второй, независимый AHRS в compat-слое: Refloat использует и свой Mahony (`balance_filter`), и прошивочный. `imu_get_pitch` идёт в `d->imu.pitch` (стартовые условия, flywheel), `balance_pitch` — из собственного фильтра |
| `imu_get_gyro(float*)` | `imu.c:44` | сырая угловая скорость → `pitch_rate` для PID D-члена | 1× IMU rate | **RT-C** | **Нативно.** Уже деротированный по конфигу IMU массив rad/s |
| `imu_get_quaternions(float*)` | `balance_filter.c:56` | инициализация Mahony-фильтра Refloat кватернионом прошивки | INIT | INIT | Нативно, из compat-AHRS |
| `imu_startup_done()` | `main.c:844` | гейт выхода из `STATE_STARTUP` | 500 Гц до готовности | RT-M | Нативно: true после калибровки/стабилизации ICM-20948 |

**Важно про ориентацию:** Refloat полагается на то, что прошивка уже применила
`CFG_PARAM_IMU_rot_roll/pitch/yaw` и офсеты акселерометра/гироскопа. Compat-слой обязан
воспроизвести эту трансформацию, иначе знаки pitch/roll и `gyro[1]`, `gyro[2]` разъедутся.

### 2.4. ADC / footpad

| API | Где используется | Назначение | Частота | RT | Реализация на ESP32 |
|---|---|---|---|---|---|
| `io_read_analog(VESC_PIN_ADC1)` | `footpad_sensor.c:30` | напряжение левого (по умолчанию) датчика, В | 500 Гц | **RT-M** | **Нативно.** ADC1 ESP32 + калибровка + делитель, чтобы диапазон совпадал с вольтами VESC (0…3.3 В) |
| `io_read_analog(VESC_PIN_ADC2)` | `footpad_sensor.c:31` | правый датчик | 500 Гц | **RT-M** | Нативно, аналогично |

Возврат `-1.0` означает «пина нет» — Refloat это учитывает. Порог, гистерезис (`fault_adc1`,
`fault_adc2`), swap и логика single/both уже реализованы **внутри Refloat**
(`footpad_sensor.c`), дублировать их в compat-слое не нужно и не следует — ТЗ §8 выполняется
без изменений Refloat. Compat-слой отдаёт только отфильтрованные вольты.

### 2.5. IO (beeper, LED)

| API | Где используется | Назначение | Частота | RT | Реализация на ESP32 |
|---|---|---|---|---|---|
| `io_set_mode(VESC_PIN_PPM, OUTPUT)` | `main.c:107` | инициализация пина пищалки | INIT | INIT | Нативно → GPIO ESP32 |
| `io_write(VESC_PIN_PPM, 0/1)` | `main.c:103,104,108` | вкл/выкл пищалки | по событию | RT-M | Нативно → GPIO |
| `set_pad_mode(void*,uint32,uint32)` | `led_driver.c:156` | настройка пина под PWM+DMA для WS2812 | INIT | INIT | **Не переносится.** См. §4.1 |

### 2.6. CAN

**Refloat не использует CAN API напрямую — ни одного вызова `can_*`.** Вся связь с
контроллером мотора идёт через `mc_*`. Это принципиально хорошая новость: подмена одного
локального мотора на два по CAN полностью скрыта от Refloat.

(BMS-данные приходят по CAN, но через Lisp — см. §2.14.)

### 2.7. Configuration (параметры прошивки VESC)

| API | Где используется | Параметры | Частота | RT | Реализация на ESP32 |
|---|---|---|---|---|---|
| `get_cfg_float(CFG_PARAM)` | `motor_data.c:94-103` | `l_current_min/max`, `l_in_current_min/max`, `l_temp_fet_start`, `l_temp_motor_start`, `l_max_duty`, `foc_motor_flux_linkage` | 2 Гц (aux) | AUX | Зеркало конфигурации из NVS ESP32, синхронизируемое с реальными ESC по CAN (`COMM_GET_MCCONF` поверх `CAN_PACKET_PROCESS_SHORT_BUFFER`) |
| `get_cfg_float(CFG_PARAM_IMU_mahony_kp)` | `main.c:211` | обратная совместимость старых настроек | INIT | INIT | Из своей IMU-конфигурации |
| `set_cfg_float(...)` | `main.c:212,213,214` | Refloat **пишет** `IMU_mahony_kp/ki/accel_confidence_decay`, если kp>1 | INIT | INIT | Записать в свою IMU-конфигурацию |
| `get_cfg_int(CFG_PARAM)` | `motor_data.c:80,104`, `main.c:1186` | `si_battery_cells`, `si_motor_poles`, `IMU_sample_rate` | INIT/AUX | AUX | Локальная конфигурация ESP32 |
| `store_cfg()` | не используется | — | — | — | — |

`CFG_PARAM_IMU_sample_rate` = 0 трактуется Refloat как «FW 6.02» и подменяется на 620 Гц.
Compat-слой обязан вернуть **реальную** частоту семплирования ICM-20948.

### 2.8. Persistent storage

| API | Где используется | Назначение | Частота | RT | Реализация на ESP32 |
|---|---|---|---|---|---|
| `read_eeprom_var(eeprom_var*, int)` | `main.c:1158` | чтение конфигурации Refloat (80 слов по 4 байта) | INIT/CMD | INIT | **Нативно.** NVS ESP32: blob или массив 32-битных слов по индексу |
| `store_eeprom_var(eeprom_var*, int)` | `main.c:1092` | запись конфигурации (`COMMAND_CFG_SAVE`) | CMD | CMD | Нативно, NVS |
| `store_backup_data()` | `main.c:1132` | сохранение одометра | AUX (>200 м) | AUX | Нативно, NVS |

`SERIALIZED_CONFIG_LENGTH = 320` байт → 80 слов адресного пространства EEPROM. Требуется
износостойкость NVS, запись только по явной команде — уже так и есть.

### 2.9. Timing

| API | Где используется | Назначение | Частота | RT | Реализация на ESP32 |
|---|---|---|---|---|---|
| `system_time_ticks()` | `time.c:21,22`, `time.h:35` | системное время в тиках, **`SYSTEM_TICK_RATE_HZ = 10000`** (100 мкс) | ~1300 Гц | **RT-C** | `esp_timer_get_time()/100` |
| `system_time()` | `time.c:24`, `charging.c:32,54`, `leds.c`, `utils.h:48,61` | секунды с загрузки (float) | часто | RT-M | `esp_timer_get_time()*1e-6f` |
| `timer_time_now()` | `main.c:772,780`, `latency_tracker.c:36` | высокоточный таймер | 1× IMU + 500 Гц | **RT-C** | `esp_timer_get_time()` (32-битный срез) или CCOUNT |
| `timer_seconds_elapsed_since(uint32)` | `main.c:779,1058` | dt главного цикла | 500 Гц | RT-M | Разность `esp_timer` |
| `sleep_us(uint32)` | `main.c:777,1143` | ритм главного и aux потоков | 500 Гц | RT-M | `vTaskDelayUntil` (не busy-wait; 500 Гц требует tick 1000 Гц или таймер) |

**Замечание:** тик 100 мкс в `SYSTEM_TICK_RATE_HZ` определён в `vesc_c_if.h` (SDK, не Refloat) —
значение можно оставить как есть, ESP32 легко даёт такое разрешение.

### 2.10. Threads / callbacks

| API | Где используется | Назначение | RT | Реализация на ESP32 |
|---|---|---|---|---|
| `spawn(fun, 1536, name, arg)` | `main.c:2697,2703` | 2 потока: Main и Aux | INIT | `xTaskCreatePinnedToCore`. **Стек 1536 байт слишком мал для ESP32** (xtensa + printf) — compat-слой должен масштабировать запрошенный размер (×4…×8) |
| `request_terminate(lib_thread)` | `main.c:2662,2665,2706` | остановка потоков | INIT/STOP | Флаг + `vTaskDelete` после выхода из цикла |
| `should_terminate()` | `main.c:775,1118` | условие выхода из циклов | 500 Гц | Чтение per-task флага (нужен TLS: функция без аргументов) |
| `thread_set_priority(int)` | `main.c:1112,1113` | aux → приоритет −1 | INIT | Отображение диапазона −5…5 на приоритеты FreeRTOS |
| `get_arg(PROG_ADDR)` (макрос `ARG`) | по всему коду | доступ к `Data*` из callback/extension | часто | Один глобальный указатель |
| `INIT_FUN` / `HEADER` / `PROG_ADDR` | `main.c:2673`, `data.h` | секции загрузчика | INIT | Макросы переопределяются в нашем `vesc_c_if.h`; `PROG_ADDR` → 0 |

Refloat **не** использует `mutex_*` и `sem_*`. Синхронизации между IMU-callback и главным
потоком нет вообще — обмен идёт через поля `Data` без блокировок. На двухъядерном ESP32 это
требует пинить оба потока на **одно ядро**, иначе получим гонки на невыровненных многословных
записях (см. risk R-08).

### 2.11. Events / VESC Tool communication

| API | Где используется | Назначение | RT | Реализация на ESP32 |
|---|---|---|---|---|
| `set_app_data_handler(func)` | `main.c:2659(NULL),2712` | приём команд от UI (`COMM_CUSTOM_APP_DATA`) | CMD | Транспорт: BLE/Wi-Fi → диспетчер |
| `send_app_data(data, len)` | `lib/utils.h:96` (`SEND_APP_DATA`) | ответы UI, realtime-данные | CMD | То же в обратную сторону |
| `conf_custom_add_config(get,set,get_xml)` | `main.c:2711` | регистрация конфигурации Refloat в VESC Tool | INIT | Требует эмуляции `COMM_GET_CUSTOM_CONFIG*` |
| `conf_custom_clear_configs()` | `main.c:2660` | снятие при остановке | STOP | То же |
| `app_is_output_disabled()` | `utils.h:47,60`, `main.c:2554` | гейт вывода в терминал | 500 Гц | Вернуть `false` (или true при отключённом логировании) |

Набор команд протокола Refloat (первый байт payload) — из `on_command_received`:
`COMMAND_GET_INFO`, `COMMAND_CFG_RESTORE`, `COMMAND_CFG_SAVE`, `COMMAND_TUNE_DEFAULTS`,
`COMMAND_PRINT_INFO`, `COMMAND_GET_ALLDATA`, `COMMAND_EXPERIMENT`, `COMMAND_LOCK`,
`COMMAND_HANDTEST`, `COMMAND_BOOSTER`, `COMMAND_FLYWHEEL`, `COMMAND_REMOTE`,
`COMMAND_LCM_*` (5 шт.), `COMMAND_CHARGING_STATE`, `COMMAND_REALTIME_DATA`,
`COMMAND_REALTIME_DATA_INTERNAL(_IDS)`, `COMMAND_LIGHTS_CONTROL`, `COMMAND_DATA_RECORD`,
`COMMAND_ALERTS_LIST`, `COMMAND_ALERTS_CONTROL`. Документированы в `doc/commands/`.

Эти команды **уже** реализованы внутри Refloat — от ESP32 нужен только транспорт байтов.

### 2.12. Logging / Plot

| API | Где используется | Назначение | RT | Реализация на ESP32 |
|---|---|---|---|---|
| `printf(fmt, ...)` | `utils.h:50,63` (`log_msg`) | лог в терминал VESC Tool | редко | `ESP_LOGI` / UART. **Не вызывать из IMU-callback** |
| `plot_init/add_graph/set_graph/send_points` | `data_recorder.c:138,139,148,150` | графики в VESC Tool | CMD | Заглушки (no-op) в v1 |

### 2.13. Beeper / LED

| Подсистема | Зависимость | Реализация на ESP32 |
|---|---|---|
| Beeper | `io_set_mode`/`io_write` по `VESC_PIN_PPM` | Нативно, GPIO. Тривиально |
| LED (WS2812/SK6812) | `led_driver.c` напрямую пишет регистры STM32: `TIM_TypeDef`, `DMA_Stream_TypeDef`, `PAL_MODE_ALTERNATE`, `set_pad_mode` | **Единственный непортируемый файл Refloat.** См. §4.1 |

Хорошая новость: `led_driver.h` — нормальный интерфейс из 4 функций
(`init/setup/paint/destroy`), вся логика цвета/анимации в `leds.c` и `led_strip.c` — портируема
без изменений.

### 2.14. BMS

| Механизм | Детали | Реализация на ESP32 |
|---|---|---|
| `lisp/bms.lisp` | Lisp-поток, 5 Гц, читает `get-bms-val` (CAN BMS прошивки) и вызывает `ext-bms` | Нативный CAN-драйвер BMS ESP32, вызывающий ту же функцию напрямую |
| `ext_bms` (`main.c:2576`) | native extension: принимает cell_lv/hv, темп., msg_age | Вызвать напрямую с нашим кодированием `lbm_value` |

BMS по умолчанию выключен (`float_conf.bms.enabled`) → в MVP можно опустить целиком.

### 2.15. LispBM primitives

Refloat использует **9** LBM-примитивов, и только для двух собственных extension-функций:

| API | Где | Назначение |
|---|---|---|
| `lbm_add_extension` | `main.c:2713,2714` | регистрация `ext-set-fw-version`, `ext-bms` |
| `lbm_dec_as_i32` | `main.c:2565-2567,2582-2584` | распаковка аргументов |
| `lbm_dec_as_float` | `main.c:2580,2581,2585` | распаковка аргументов |
| `lbm_enc_sym_true` / `lbm_enc_sym_nil` | `main.c:2569,2589` | возврат true/nil |

**Ни одна из них не находится в realtime path.** Для ESP32 достаточно 20 строк-заглушек:
`lbm_value = uint32_t`, декодеры читают из нашего же массива. LispBM-интерпретатор **не нужен**.

### 2.16. Package runtime (механика загрузки)

| Элемент | Что это | Реализация на ESP32 |
|---|---|---|
| `load-native-lib` | LispBM загружает PIC-бинарник в RAM STM32 и вызывает `init` | Не нужен: линкуем Refloat статически, вызываем `init()` |
| `HEADER` / `.program_ptr` / `.init_fun` секции | разметка загрузчика | Макросы-пустышки |
| `PROG_ADDR` | база модуля; используется в `get_cfg_xml` для смещения адреса XML-блоба | Определить как `0` |
| `link.ld` (96 КБ) | ограничение размера модуля | Не применяется; ESP32 имеет ~520 КБ SRAM + IRAM/flash |
| `vesc_tool --xmlConfToCode` | генерация `conf/confparser.{c,h}`, `conf/confxml.{c,h}`, `conf/conf_default.h` из `settings.xml` **во время сборки** | **Build-time зависимость от бинарника `vesc_tool`.** Либо иметь его в CI, либо вендорить сгенерированные файлы (см. риск R-12) |

### 2.17. Прочее / не используется

Refloat **не** использует: `uart_*`, `packet_*`, `terminal_*`, `commands_process_packet`,
`can_*`, `mutex_*`, `sem_*`, `encoder_set_custom_callbacks`, `read_nvm/write_nvm`,
`ahrs_*` (у него собственный Mahony), `mc_release_motor`, `mc_set_handbrake`, `sys_lock/unlock`,
`shutdown_disable`, `imu_derotate`, `imu_get_accel`, `mc_set_pid_speed/pos`.

Также используются, но вне основного пути: `get_ppm`, `get_ppm_age`, `get_remote_state`
(`remote.c:72,73,76` — PPM/Wand-пульт; в MVP заглушки), `malloc`/`free`
(**только на init/config, не в realtime path** — проверено: `sma.c:42,51` вызывается из
`*_configure`, `main.c:1075,1149` — работа с конфигом, `leds.c`/`led_driver.c` — setup).

---

## 3. Семантика агрегации двух VESC (проверка правил из ТЗ §6)

ТЗ требовало проверить семантику каждого API перед реализацией. Результат проверки —
**два правила из ТЗ нужно изменить.**

### 3.1. Ток мотора и ток батареи: среднее, а не сумма

Обоснование. Refloat работает в замкнутом контуре, где выходная величина и измеряемая
величина обязаны быть в одном пространстве:

* выход: `mc_set_current(X)` — и наш motor abstraction отправляет **X ампер каждому** из двух ESC;
* вход: `mc_get_tot_current_filtered()` → `m->current`, `m->dir_current`;
* лимит: `m->current_max = get_cfg_float(CFG_PARAM_l_current_max)` — это лимит **одного** VESC;
* `pid_control()` (`main.c:707-718`) ограничивает вычисленный ток по `current_max`;
* `motor_data_get_current_saturation()` = `|filt_current| / current_limit` управляет ATR и
  haptic-обратной связью.

Если отдавать **сумму** A+B, а лимит остаётся per-ESC, то saturation будет завышена вдвое,
ATR и haptic сработают на половине реального тока, а `torque = filt_current/speed_constant`
даст удвоенный момент → ATR будет постоянно «видеть» подъём. Поэтому:

> **Правило: motor current, directional current, battery current — среднее (A+B)/2.**
> Все лимиты (`l_current_max`, `l_in_current_max`, …) берутся из конфигурации **одного** VESC.

Альтернатива (сумма токов + сумма лимитов) математически эквивалентна, но требует, чтобы
compat-слой удваивал и значения из `get_cfg_float` — легко забыть при обновлении. Среднее
проще и безопаснее.

Исключение — чисто отображаемые величины: `mc_get_tot_current_in` (LCM-дисплей),
`amp_hours`, `watt_hours` — **сумма**, это потребление пакета.

### 3.2. Fault code: «любой», но с оговоркой

`motor_data_evaluate_alerts()` (`motor_data.c:183`) передаёт код в alert tracker; `main.c:1333`
использует его для UI. Правило «fault если fault у любого ESC» корректно. Но:

* коды у A и B могут отличаться — отдавать первый ненулевой в порядке A, B, и **отдельно**
  хранить оба для UI;
* в перечислении `mc_fault_code` **нет кода для «потеря связи с ESC»**. Не надо втискивать
  туда синтетику: потеря CAN обрабатывается supervisor-ом compat-слоя, который перехватывает
  `mc_set_current` и переводит систему в безопасное состояние независимо от Refloat.
  Для UI можно отдать `FAULT_CODE_BRK` только как индикацию, но логика безопасности не должна
  зависеть от того, как Refloat отреагирует на код фолта.

### 3.3. Остальные правила ТЗ подтверждены

| Величина | Правило | Проверка |
|---|---|---|
| Input voltage | среднее | ✅ Общий пакет; используется для LV/HV tiltback. Расхождение >1 В → диагностический флаг |
| Duty | `max(|A|,|B|)` | ✅ Знак не нужен: `motor_data.c:144` берёт `fabsf()`. Максимум консервативен для duty tiltback |
| ERPM | среднее | ✅ + контроль расхождения: `|A−B| > порог` → fault (проскальзывание/рассинхрон) |
| Температуры | `max(A,B)` | ✅ Консервативно, ведёт к раннему tiltback |
| Distance / speed | среднее | ✅ Это перемещение доски, а не сумма путей |
| Amp/watt hours | сумма | ✅ Отображение потребления пакета |

---

## 4. Что придётся менять в upstream Refloat

Цель ТЗ §14 — держать изменения вне исходников Refloat. Аудит показывает, что это **почти
полностью достижимо**, потому что весь платформенный слой (`vesc_c_if.h`, `rules.mk`, `link.ld`)
лежит в `vesc_pkg_lib/`, то есть в SDK VESC, а не в `src/` Refloat.

### 4.1. `src/led_driver.c` / `led_driver.h` — единственный блокер

Пишет напрямую в регистры таймеров и DMA STM32F4. Переносу не подлежит.
**Решение без патча Refloat:** исключить оба файла из сборки и предоставить собственный
`led_driver.c` для ESP32 (RMT-периферия), реализующий тот же `led_driver.h`.
**Проверено:** `leds.c` обращается к драйверу только через 4 функции
(`led_driver_init/setup/paint/destroy`, строки 836, 905, 1053, 1254, 1269) и хранит `LedDriver`
по значению (`leds.h:90`). Полей `PinHwConfig` он не касается. Значит, замена
`led_driver.{c,h}` на ESP32-версию (RMT-периферия) с тем же прототипом **не требует патча
`leds.c`** — только исключения двух файлов из списка компилируемых.
В MVP LED-и можно вообще не собирать — ТЗ §9 это разрешает.

### 4.2. `src/time.h:20` — `typedef uint32_t time_t;`

Конфликтует с `time_t` из newlib (`<sys/types.h>`) на ESP-IDF. Ни один файл Refloat не
включает системные заголовки, объявляющие `time_t`, поэтому конфликт возникнет только если
наш shim затянет их в ту же единицу трансляции. **Решение:** в `vesc_c_if.h`-шиме не включать
ничего из ESP-IDF; всю платформенную часть держать в отдельных `.c`-файлах compat-слоя.
Патч не требуется, но требуется дисциплина сборки.

### 4.3. `get_cfg_xml` и `PROG_ADDR`

`main.c:2646-2653` возвращает `data_refloatconfig_ + PROG_ADDR`. При `#define PROG_ADDR 0`
это корректный указатель. Патч не нужен.

### 4.4. Числа с плавающей точкой

ESP32 имеет FPU **только для single precision**. В коде есть литералы двойной точности
(`balance_filter.c:44`: `0.9`, `0.1`, `1.0 -`; `motor_data.c`: `* 3.6`, `- 0.05`).
На STM32F4 их гасит флаг `-fsingle-precision-constant`. Тот же флаг обязателен для ESP32,
иначе получим программную эмуляцию double в контуре баланса. Патч не нужен, нужен флаг сборки.

### 4.5. Итоговый список потенциальных патчей

| # | Файл | Причина | Обязателен? |
|---|---|---|---|
| P-01 | `src/led_driver.{c,h}` | регистры STM32 | Только если нужны LED (не в MVP) |
| P-02 | `src/leds.c` | — | **Не требуется**, проверено: драйвер используется только через 4 функции |
| — | остальное `src/` | — | **Изменения не требуются** |

---

## 5. Ответы на Definition of Done (ТЗ §17)

1. **Какие VESC API использует Refloat?** — 74 члена `vesc_c_if`, перечислены в §2. Плюс 9
   LBM-примитивов (§2.15) и механика загрузки нативного модуля (§2.16).
2. **Какие realtime-critical?** — 22 функции, §1.4. Из них 6 — в самом контуре баланса
   (IMU-callback).
3. **Что реализуется на ESP32 нативно?** — IMU (5 API), ADC/футпады (1), время (5), потоки (5),
   NVS (3), IO/beeper (3), логирование, LBM-заглушки.
4. **Что проксируется на Dual FSESC?** — вся категория Motor output (6) и Motor telemetry (22).
5. **Можно ли запустить исходный Refloat через LispBM на ESP32?** — Вопрос не имеет смысла:
   Refloat не написан на LispBM. См. §0 и `porting_strategy.md`.
6. **Как обеспечить IMU callback с нужным таймингом?** — DRDY-прерывание ICM-20948 →
   `vTaskNotifyGiveFromISR` → задача максимального приоритета, пиннутая на ядро 1,
   вызывающая `imu_ref_callback` напрямую. §2.3, подробности в `esp32_architecture.md`.
7. **Как представить два VESC как один logical motor?** — §3 + `esp32_architecture.md`.
8. **Что нужно для совместимости с Refloat UI?** — транспорт `send_app_data`/
   `set_app_data_handler` и набор команд §2.11; сама логика команд уже в Refloat.
9. **Что нужно для VESC Tool напрямую к ESP32?** — эмуляция VESC packet protocol:
   `COMM_FW_VERSION`, `COMM_GET_VALUES`, `COMM_CUSTOM_APP_DATA`, `COMM_GET_CUSTOM_CONFIG`,
   `COMM_GET_CUSTOM_CONFIG_XML`, `COMM_SET_CUSTOM_CONFIG`, плюс LispBM-заглушки для
   определения пакета. Вне первой реализации (ТЗ §10).
10. **Какие изменения нужны в upstream?** — §4: в идеале ноль, максимум 2 файла LED-драйвера.
11. **Главные safety risks?** — `risk_register.md`.
12. **Как тестировать каждый слой?** — `esp32_architecture.md`, раздел «Тестирование».
