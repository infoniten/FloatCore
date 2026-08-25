# Контракт VESC_IF ↔ ESP32 platform layer

Точный контракт для всех 74 членов структуры `vesc_c_if`, которые использует Refloat
(`lukash/refloat @ 47a2c5ce`, v1.3.0). Сигнатуры приведены дословно из
`refloat-upstream/vesc_pkg_lib/vesc_c_if.h`.

Категории — по ТЗ v0.2 §1. Колонка **IMU-path** отвечает на вопрос «может ли вызов
произойти внутри `imu_ref_callback`», то есть в контуре баланса.

Всё в этом документе проверено host-харнессом: `tests/host` собирает неизменённые
исходники Refloat поверх mock-реализации этих 74 функций и прогоняет 10 сценариев.

---

## 1. Realtime-critical

Вызывается из `imu_ref_callback` (`main.c:734`) — на каждый семпл IMU. Бюджет всего
callback при 500 Гц — 2000 мкс; целевой бюджет реализации — не более 200 мкс,
то есть 10 % периода.

| Сигнатура | Где | Частота | IMU-path | Синхронизация | Источник на ESP32 | Latency | Поведение при отказе |
|---|---|---|---|---|---|---|---|
| `void (*imu_set_read_callback)(void (*func)(float *acc, float *gyro, float *mag, float dt))` | `main.c:2710`, снятие `:2658` | 1× при старте | задаёт сам путь | — | Задача, разбуженная DRDY ICM-20948 | — | Нет callback → нет команд мотору → тяга снимается по watchdog VESC |
| `float (*imu_get_roll)(void)` | `imu.c:36` | 1× семпл | **да** | Не нужна: пишется и читается в одной задаче | AHRS compat-слоя (Mahony), радианы | < 1 мкс (чтение поля) | Возврат последнего валидного; при протухании > 5 мс — fault |
| `float (*imu_get_pitch)(void)` | `imu.c:38`, `leds.c:973` | 1× семпл + LED | **да** | `leds.c` читает из aux-задачи — гонка на float, безвредна (атомарное 32-битное чтение) | AHRS compat-слоя | < 1 мкс | То же |
| `float (*imu_get_yaw)(void)` | `imu.c:40` | 1× семпл | **да** | — | AHRS compat-слоя | < 1 мкс | То же |
| `void (*imu_get_gyro)(float *gyro)` | `imu.c:44` | 1× семпл | **да** | Буфер обновляется до вызова callback | Деротированный гироскоп, рад/с | < 1 мкс (memcpy 12 Б) | **NaN здесь необратимо отравляет фильтр** — см. §6 |
| `systime_t (*system_time_ticks)(void)` | `time.c:21,22` | ~1300/с | **да** | Атомарно | `esp_timer_get_time()/100` | < 1 мкс | Монотонность обязательна; скачок назад ломает все таймеры Refloat |
| `uint32_t (*timer_time_now)(void)` | `main.c:772,780`, `latency_tracker.c:36` | ~1000/с | **да** | Атомарно | `esp_timer_get_time()` (младшие 32 бита) | < 1 мкс | Переполнение раз в ~71 мин обрабатывается разностной арифметикой |
| `void (*mc_set_current)(float current)` | `motor_control.c:57,99,108` | 1× семпл | **да** | Очередь TWAI потокобезопасна | → `LogicalMotor.requestCurrent` → 2 CAN-кадра | **≤ 200 мкс** до постановки в очередь TWAI | NaN/Inf/выход за лимит → кадр не отправляется, fault |
| `void (*mc_set_current_off_delay)(float delay_sec)` | `motor_control.c:98` | 1× семпл | **да** | — | Параметр к следующей команде тока | ≤ 10 мкс | Игнорируется, если ESC не поддерживает |
| `void (*mc_set_brake_current)(float current)` | `motor_control.c:117` | 1× семпл (в покое) | **да** | — | → `CAN_PACKET_SET_CURRENT_BRAKE` ×2 | ≤ 200 мкс | То же, что `mc_set_current` |
| `void (*mc_set_duty)(float dutyCycle)` | `motor_control.c:114` | 1× семпл (парк. тормоз) | **да** | — | → `CAN_PACKET_SET_DUTY` ×2 | ≤ 200 мкс | То же |
| `void (*timeout_reset)(void)` | `motor_control.c:93` | 1× семпл | **да** | — | Продление watchdog-а LogicalMotor | ≤ 10 мкс | **Прекращение вызовов = снятие тяги.** Это основной механизм безопасности |

**Проверено сценарием 4:** ровно 50 команд мотору за 0.1 с при 500 Гц — то есть одна
команда на семпл IMU, как и предсказывал аудит.

**Проверено сценарием 6:** при замолчавшем IMU главный поток продолжает работать, но
команд мотору и вызовов `timeout_reset` больше нет. Тягу снимает watchdog VESC.

---

## 2. Soft realtime

Главный поток, 500 Гц (`refloat_thd`, `main.c:767`) и aux-поток, 30 Гц. Пропуск отдельного
такта не опасен, но систематическое отставание меняет коэффициенты фильтров через
`frequency_tracker`.

| Сигнатура | Где | Частота | IMU-path | Синхронизация | Источник на ESP32 | Latency | Поведение при отказе |
|---|---|---|---|---|---|---|---|
| `void (*sleep_us)(uint32_t us)` | `main.c:777` (500 Гц), `:1143` (30 Гц) | 530/с | нет | — | `vTaskDelayUntil`, tick 1000 Гц | джиттер ≤ 5 % периода | Систематический перебор → падение частоты контура |
| `bool (*should_terminate)(void)` | `main.c:775,1118` | 530/с | нет | Флаг задачи | TLS/поле задачи | < 1 мкс | — |
| `float (*timer_seconds_elapsed_since)(uint32_t time)` | `main.c:779,1058` | 500/с | нет | — | Разность `esp_timer` | < 1 мкс | Даёт `dt` главного цикла; отрицательное значение недопустимо |
| `float (*system_time)(void)` | `time.c:24`, `charging.c:32,54`, `leds.c:926,952,1262`, `utils.h:48,61` | ~500/с | нет | Атомарно | `esp_timer_get_time()*1e-6f` | < 1 мкс | Потеря точности float после ~4.6 часа работы — известное свойство и на VESC |
| `float (*io_read_analog)(VESC_PIN pin)` | `footpad_sensor.c:30,31` | 1000/с (2 канала) | нет | Буфер обновляется ADC-задачей | ADC1 ESP32 + `esp_adc_cal` + фильтр | ≤ 50 мкс | Возврат `-1.0` = «пина нет», Refloat это учитывает. Обрыв/КЗ → значение вне физического диапазона → fault compat-слоя |
| `bool (*imu_startup_done)(void)` | `main.c:844` | 500/с до готовности | нет | — | true после калибровки ICM-20948 | < 1 мкс | Пока false, Refloat остаётся в `STATE_STARTUP` — безопасное состояние по умолчанию |
| `bool (*io_write)(VESC_PIN pin, int state)` | `main.c:103,104,108` | по событию | нет | — | GPIO (пищалка) | ≤ 10 мкс | Некритично |
| `bool (*io_set_mode)(VESC_PIN pin, VESC_PIN_MODE mode)` | `main.c:107` | 1× | нет | — | GPIO | — | Некритично |
| `bool (*app_is_output_disabled)(void)` | `utils.h:47,60`, `main.c:2554` | 500/с | нет | — | Константа/настройка | < 1 мкс | Только гейт логов, **не является предохранителем** |
| `lib_thread (*spawn)(void (*fun)(void*), size_t stack_size, const char *name, void *arg)` | `main.c:2697,2703` | 2× при старте | нет | — | `xTaskCreatePinnedToCore`, **ядро 1** | — | Запрошенные 1536 Б стека для Xtensa мало: масштабировать ×4…×8 |
| `void (*request_terminate)(lib_thread thd)` | `main.c:2662,2665,2706` | при остановке | нет | — | Флаг + join | — | — |
| `void (*thread_set_priority)(int priority)` | `main.c:1112,1113` | 1× | нет | — | −5…5 → приоритеты FreeRTOS | — | Указатель проверяется Refloat на NULL |
| `void** (*get_arg)(uint32_t prog_addr)` | макрос `ARG`, повсеместно | часто | **да** | — | Указатель на ячейку `lib_info.arg` | < 1 мкс | **Обязан возвращать адрес именно поля `lib_info.arg`** — Refloat пишет туда `info->arg = d` в начале `init()` и сразу пользуется `ARG` |

---

## 3. Telemetry

Читается главным потоком на 500 Гц. Все значения — агрегация двух VESC; правила и их
обоснование в [motor_semantics.md](motor_semantics.md).

| Сигнатура | Где | Частота | IMU-path | Синхронизация | Источник на ESP32 | Latency | Поведение при отказе |
|---|---|---|---|---|---|---|---|
| `float (*mc_get_rpm)(void)` | `motor_data.c:127`, `leds.c:547` | 500 Гц | нет | Двойная буферизация снимка телеметрии | CAN STATUS_1 обоих ESC | возраст ≤ 20 мс | Данные старше 100 мс → fault, значение не используется |
| `float (*mc_get_speed)(void)` | `motor_data.c:137` | 500 Гц | нет | то же | Расчёт из ERPM | — | то же |
| `float (*mc_get_distance)(void)` | `motor_data.c:138`, `leds.c:1123,1213` | 500 Гц | нет | то же | STATUS_5 (тахометр) | — | то же |
| `float (*mc_get_duty_cycle_now)(void)` | `motor_data.c:144` | 500 Гц | нет | то же | STATUS_1 | возраст ≤ 20 мс | Занижение опасно: duty tiltback не сработает → **при протухании подставлять последнее максимальное, а не 0** |
| `float (*mc_get_tot_current_filtered)(void)` | `motor_data.c:140` | 500 Гц | нет | то же | STATUS_1 | — | то же |
| `float (*mc_get_tot_current_directional_filtered)(void)` | `motor_data.c:141` | 500 Гц | нет | то же | STATUS_1 + знак ERPM | — | то же |
| `float (*mc_get_tot_current_in_filtered)(void)` | `motor_data.c:158` | 500 Гц | нет | то же | STATUS_4 | — | то же |
| `float (*mc_get_input_voltage_filtered)(void)` | `motor_data.c:159`, `lcm.c:110` | 500 Гц | нет | то же | STATUS_5 | — | Протухание → fault; подстановка 0 вызвала бы ложный LV-tiltback |
| `float (*mc_temp_fet_filtered)(void)` | `motor_data.c:176` | 500 Гц | нет | то же | STATUS_4 | — | Протухание → удерживать последнее (консервативно) |
| `float (*mc_temp_motor_filtered)(void)` | `motor_data.c:177` | 500 Гц | нет | то же | STATUS_4 | — | то же |
| `mc_fault_code (*mc_get_fault)(void)` | `motor_data.c:183`, `main.c:1333`, `lcm.c:99` | 500 Гц | нет | то же | Опрос обоих ESC | — | **Сценарий 9 подтвердил: Refloat по этому коду НЕ останавливается**, только поднимает алерт. Останов — обязанность supervisor-а |
| `float (*mc_get_tot_current_in)(void)` | `lcm.c:109` | по запросу | нет | — | STATUS_4, сумма | — | Только отображение |
| `float (*mc_get_distance_abs)(void)` | `main.c:1386,2159` | по запросу | нет | — | Тахометры | — | Только отображение |
| `uint64_t (*mc_get_odometer)(void)` | `main.c:1131,1133,1239,1394,2156` | 30 Гц (aux) | нет | — | Собственный счётчик в NVS | — | Только отображение |
| `float (*mc_get_battery_level)(float *wh_left)` | `lcm.c:186`, `leds.c:584`, `main.c:1399,2127` | 30 Гц | нет | — | Расчёт по напряжению | — | Только отображение |
| `float (*mc_get_amp_hours)(bool reset)` | `main.c:1395,2163` | по запросу | нет | — | STATUS_2, сумма | — | Только отображение |
| `float (*mc_get_amp_hours_charged)(bool reset)` | `main.c:1396,2169` | по запросу | нет | — | STATUS_2, сумма | — | Только отображение |
| `float (*mc_get_watt_hours)(bool reset)` | `main.c:1397,2173` | по запросу | нет | — | STATUS_3, сумма | — | Только отображение |
| `float (*mc_get_watt_hours_charged)(bool reset)` | `main.c:1398,2180` | по запросу | нет | — | STATUS_3, сумма | — | Только отображение |
| `float (*foc_get_id)(void)` | `main.c:1376,2183` | по запросу | нет | — | Недоступно по CAN → 0 | — | Только диагностика в UI |
| `const char* (*mc_fault_to_string)(mc_fault_code fault)` | `main.c:2212` | по запросу | нет | — | Локальная таблица строк | — | Только отображение |
| `volatile gnss_data* (*mc_gnss)(void)` | `main.c:2193,2385` | по запросу | нет | — | Нулевая структура | — | Только отображение |

---

## 4. Configuration

| Сигнатура | Где | Частота | IMU-path | Синхронизация | Источник на ESP32 | Latency | Поведение при отказе |
|---|---|---|---|---|---|---|---|
| `float (*get_cfg_float)(CFG_PARAM p)` | `motor_data.c:94-103`, `main.c:211` | 2 Гц (aux) | нет | Обновляется только aux-задачей | Зеркало конфигурации ESC в NVS, синхронизируемое по CAN | ≤ 1 мс | Нет данных от ESC → консервативные значения из NVS, режим ограниченного тока |
| `int (*get_cfg_int)(CFG_PARAM p)` | `motor_data.c:80,104`, `main.c:1186` | при старте / 2 Гц | нет | — | То же + реальный ODR IMU для `IMU_sample_rate` | ≤ 1 мс | `IMU_sample_rate = 0` Refloat трактует как FW 6.02 и подставляет 620 Гц — **обязаны отдать реальную частоту** |
| `bool (*set_cfg_float)(CFG_PARAM p, float value)` | `main.c:212,213,214` | 1× при старте | нет | — | Запись в свою IMU-конфигурацию | — | Затрагивает только `IMU_mahony_kp/ki/accel_confidence_decay` |
| `void (*conf_custom_add_config)(int (*get_cfg)(uint8_t*, bool), bool (*set_cfg)(uint8_t*), int (*get_cfg_xml)(uint8_t**))` | `main.c:2711` | 1× | нет | — | Регистрация для транспорта VESC Tool | — | Без реализации UI-конфигурация недоступна, баланс работает |
| `void (*conf_custom_clear_configs)(void)` | `main.c:2660` | при остановке | нет | — | — | — | — |

Список используемых `CFG_PARAM`: `l_current_max`, `l_current_min`, `l_in_current_max`,
`l_in_current_min`, `l_temp_fet_start`, `l_temp_motor_start`, `l_max_duty`,
`foc_motor_flux_linkage`, `si_motor_poles`, `si_battery_cells`, `IMU_sample_rate`,
`IMU_mahony_kp`, `IMU_mahony_ki`, `IMU_accel_confidence_decay`. Итого 14 из 50.

---

## 5. Storage

| Сигнатура | Где | Частота | IMU-path | Синхронизация | Источник на ESP32 | Latency | Поведение при отказе |
|---|---|---|---|---|---|---|---|
| `bool (*read_eeprom_var)(eeprom_var *v, int address)` | `main.c:1158` | при старте и по `COMMAND_CFG_RESTORE` | нет | — | NVS, namespace `refloat`, 80 слов по 4 байта | ≤ 10 мс | `false` → Refloat берёт значения по умолчанию (проверено: сценарии стартуют с пустым EEPROM) |
| `bool (*store_eeprom_var)(eeprom_var *v, int address)` | `main.c:1092` | по `COMMAND_CFG_SAVE` | нет | Только из потока команд | NVS | ≤ 50 мс | `false` → Refloat логирует ошибку и продолжает |
| `bool (*store_backup_data)(void)` | `main.c:1132` | при пробеге +200 м, вне движения | нет | Только aux | NVS | ≤ 50 мс | Потеря одометра некритична |

Размер сериализованной конфигурации — **282 байта** (посчитано генератором из 172
параметров `settings.xml`), при лимите Refloat `SERIALIZED_CONFIG_LENGTH = 320`.

---

## 6. UI / события

| Сигнатура | Где | Частота | IMU-path | Синхронизация | Источник на ESP32 | Latency | Поведение при отказе |
|---|---|---|---|---|---|---|---|
| `bool (*set_app_data_handler)(void (*func)(unsigned char*, unsigned int))` | `main.c:2712`, снятие `:2659` | 1× | нет | Обработчик вызывается из comms-задачи, **ядро 0** — гонка с потоками Refloat реальна | BLE/UART | — | Без него UI недоступен, баланс работает |
| `void (*send_app_data)(unsigned char *data, unsigned int len)` | `utils.h:96` (`SEND_APP_DATA`) | по запросу | нет | — | BLE/UART | не в контуре | Переполнение буфера Refloat ловит сам и вызывает `fatal_error_terminate()` |
| `int (*printf)(const char *str, ...)` | `utils.h:50,63` | редко | **не вызывать из IMU-path** | — | `ESP_LOGI`/UART | — | Форматирование float дорого; в контуре запрещено |
| `void (*plot_init)(const char*, const char*)` | `data_recorder.c:148` | по запросу | нет | — | Заглушка | — | — |
| `void (*plot_add_graph)(const char *name)` | `data_recorder.c:150` | по запросу | нет | — | Заглушка | — | — |
| `void (*plot_set_graph)(int graph)` | `data_recorder.c:138` | по запросу | нет | — | Заглушка | — | — |
| `void (*plot_send_points)(float x, float y)` | `data_recorder.c:139` | по запросу | нет | — | Заглушка | — | — |

---

## 7. Optional

Всё, что можно оставить `NULL` или заглушкой без потери балансировки. Refloat сам проверяет
часть указателей на NULL.

| Сигнатура | Где | Проверка на NULL в Refloat | Решение для v1 |
|---|---|---|---|
| `bool (*foc_play_tone)(int channel, float freq, float voltage)` | `haptic_feedback.c:128,132` | **да**, `haptic_feedback.c:129` | `NULL` — по CAN аналога нет; haptic остаётся через модуляцию тока в `motor_control.c` |
| `systime_t (*system_time_ticks)(void)` | `time.c:21` | **да**, есть fallback на `system_time()` | Реализовать: даёт разрешение 100 мкс |
| `void (*thread_set_priority)(int priority)` | `main.c:1112` | **да** | Реализовать |
| `void (*imu_get_quaternions)(float *q)` | `balance_filter.c:56` | нет | Реализовать: инициализация фильтра Refloat кватернионом AHRS |
| `remote_state (*get_remote_state)(void)` | `remote.c:76` | нет | Заглушка с `age_s = 1000` (пульт отсутствует) |
| `float (*get_ppm)(void)` | `remote.c:72` | нет | Заглушка `0.0f` |
| `float (*get_ppm_age)(void)` | `remote.c:73` | нет | Заглушка `1000.0f` |
| `void (*set_pad_mode)(void *gpio, uint32_t pin, uint32_t mode)` | `led_driver.c:156` | нет | Не нужна: `led_driver.c` заменяется реализацией на RMT |
| `bool (*lbm_add_extension)(char*, extension_fptr)` | `main.c:2713,2714` | нет | Заглушка, возвращает `true` |
| `float (*lbm_dec_as_float)(lbm_value)` | `main.c:2580,2581,2585` | нет | 10 строк: `lbm_value` — это `uint32_t` |
| `int32_t (*lbm_dec_as_i32)(lbm_value)` | `main.c:2565-2567,2582-2584` | нет | То же |
| `lbm_uint lbm_enc_sym_nil` / `lbm_enc_sym_true` | `main.c:2569,2589` | нет | Константы 0 и 1 |
| `void* (*malloc)(size_t)` / `void (*free)(void*)` | init/config | нет | `heap_caps_malloc(MALLOC_CAP_INTERNAL)`. **В контуре баланса не вызываются** (проверено: только `*_configure`, работа с конфигом и setup LED) |

---

## 8. Unused, но обязательные для ABI

Мы переиспользуем **upstream-заголовок** `vesc_c_if.h` целиком (shim только переопределяет
макросы `VESC_IF`, `HEADER`, `INIT_FUN`, `PROG_ADDR`). Поэтому структура `vesc_c_if` должна
сохранять **полный** набор полей в исходном порядке: смещения используемых указателей
зависят от всех предшествующих.

Практически это значит:

* объявлять структуру не вручную, а включением upstream-заголовка (так сделано в
  `tests/host/mock/vesc_c_if.h`) — тогда рассинхронизация ABI невозможна в принципе;
* неиспользуемые ~130 членов остаются `NULL` после `memset` — это ровно то, что делает
  прошивка VESC для функций, отсутствующих в старых версиях;
* Refloat уже написан в расчёте на NULL-указатели (см. §7).

Категории неиспользуемого: `uart_*`, `packet_*`, `terminal_*`, `commands_*`, **весь блок
`can_*`** (Refloat не обращается к CAN напрямую), `mutex_*`, `sem_*`, `ahrs_*`,
`encoder_set_custom_callbacks`, `read_nvm`/`write_nvm`/`wipe_nvm`, `mc_set_pid_*`,
`mc_set_handbrake*`, `mc_release_motor`, `mc_stat_*`, `sys_lock`/`sys_unlock`,
`shutdown_disable`, `foc_set_openloop_*`, `imu_derotate`, `imu_get_accel`, `imu_get_mag`,
`lbm_*` (кроме перечисленных в §7).

---

## 9. Выводы, полученные экспериментально

Всё ниже — не рассуждения, а наблюдаемое поведение неизменённого Refloat в host-харнессе.

1. **Команда мотору выдаётся ровно на каждый семпл IMU** — 50 команд за 0.1 с при 500 Гц
   (сценарий 4). Это подтверждает расчёт нагрузки CAN в [can_bandwidth.md](can_bandwidth.md).
2. **Refloat не детектирует пропажу IMU** (сценарий 6). Он просто перестаёт что-либо
   делать. Безопасность обеспечивается только тем, что прекращаются `timeout_reset` и
   команды тока → VESC снимает тягу по своему таймауту.
3. **Refloat не останавливается по `mc_get_fault`** (сценарий 9): фолт прошивки поднимает
   алерт и звук, состояние остаётся `RUNNING`. Снятие тяги — обязанность supervisor-а.
4. **Refloat ничего не знает об отказе одного из ESC** (сценарии 7, 8) — по конструкции:
   он видит один логический мотор. Реакция целиком на compat-слое.
5. **NaN в акселерометре безвреден**: `balance_filter.c:86` проверяет `accel_norm > 0.01`,
   а любое сравнение с NaN ложно, поэтому ветка коррекции просто пропускается.
6. **NaN в гироскопе необратимо разрушает фильтр**: он интегрируется прямо в кватернион.
   При этом `motor_control.c:112` проверяет `isnan(requested_current)`, поэтому NaN
   *не доходит* до `mc_set_current` — вместо этого выход молча вырождается в нулевой ток.
   Доска перестаёт балансировать, оставаясь в состоянии `RUNNING`, и фильтр **не
   восстанавливается** после возврата корректных данных (сценарий 10).
   → Валидация данных IMU обязана стоять **до** вызова callback, а не после.
