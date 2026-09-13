# Инвентарь протокола VESC: что нужно FloatCore

Источники: `vedderb/bldc` (`comm/packet.c`, `comm/commands.c`, `datatypes.h`,
`conf_custom.c`) и `vedderb/vesc_tool` (`commands.cpp`, `vescinterface.cpp`).
Оба репозитория изучены в состоянии на 2026-08-26.

Принцип отбора: **источник требований — сам VESC Tool**. В список реализуемого
попадает только то, что он действительно шлёт при подключении, чтении телеметрии,
загрузке Refloat UI и работе с конфигурацией. Всё остальное — в
[unsupported_commands.md](unsupported_commands.md).

---

## 1. Кадрирование

`bldc/comm/packet.c`. Реализовано в `compat/vesc_protocol/packet.c` — совпадает
с оригиналом по формату и по логике ресинхронизации.

```
[start][len...][payload][crc_hi][crc_lo][0x03]
 start = 2 → 1 байт длины   (len ≤ 255)
 start = 3 → 2 байта длины  (255 ≤ len ≤ 65535)
 start = 4 → 3 байта длины  (len > 65535)
```

* CRC — CRC-16/XMODEM (poly `0x1021`, init `0x0000`), считается **только по payload**.
* Стоп-байт всегда `0x03`.
* Более короткий пакет обязан использовать меньше байт длины: `start = 3` с `len < 255`
  отбрасывается. Это часть формата, а не наша придирка.
* `PACKET_MAX_PL_LEN` = 512, как в прошивке по умолчанию.

Особенность, унаследованная от оригинала: после мусора парсер может ждать
`bytes_left` байт, прежде чем снова пытаться декодировать. Ресинхронизация
гарантированно укладывается в размер буфера (520 байт); измерено в тестах — 63 байта.

## 2. Реализованные команды

| Command | ID | Direction | Required | Purpose | FloatCore implementation |
|---|---|---|---|---|---|
| `COMM_FW_VERSION` | 0 | Tool → FC, ответ | **да** | Идентификация. Без неё VESC Tool разрывает соединение по таймауту | `firmware_info.c`. hw_type = CUSTOM_MODULE, версия 6.06, имена «FloatCore», виртуальный UUID |
| `COMM_GET_VALUES` | 4 | Tool → FC, ответ | для телеметрии | Realtime-страница | `telemetry.c`, источник — `LogicalMotorTelemetry` |
| `COMM_GET_VALUES_SELECTIVE` | 50 | Tool → FC, ответ | опц. | То же с маской полей | `telemetry.c`, та же кодировка + маска |
| `COMM_ALIVE` | 30 | Tool → FC | **да** | Keep-alive. Прошивка на него не отвечает | Принимается, ответа нет — как в прошивке |
| `COMM_GET_DECODED_PPM` | 31 | Tool → FC, ответ | для RT App | Страница RT App опрашивает её периодически | `decoded_inputs.c`, нейтраль: приёмника нет |
| `COMM_GET_DECODED_ADC` | 32 | Tool → FC, ответ | для RT App | То же | `decoded_inputs.c`, напряжения на выводах ADC1/ADC2 педалей |
| `COMM_GET_DECODED_CHUK` | 33 | Tool → FC, ответ | для RT App | То же | `decoded_inputs.c`, нейтраль: нунчака нет |
| `COMM_CUSTOM_APP_DATA` | 36 | обе стороны | **да** | Канал QML ↔ Refloat | `custom_app.c` — чистый транспорт, содержимое не разбирается |
| `COMM_GET_CUSTOM_CONFIG_XML` | 92 | Tool → FC, ответ | **да** | Схема параметров (сжатый `settings.xml`) чанками | `config_bridge.c` + `get_cfg_xml` Refloat |
| `COMM_GET_CUSTOM_CONFIG` | 93 | Tool → FC, ответ | **да** | Текущая конфигурация | `config_bridge.c` + `get_cfg` Refloat |
| `COMM_GET_CUSTOM_CONFIG_DEFAULT` | 94 | Tool → FC, ответ | **да** | Значения по умолчанию (кнопка Restore) | То же, `is_default = true` |
| `COMM_SET_CUSTOM_CONFIG` | 95 | Tool → FC, ответ | **да** | Запись конфигурации | `set_cfg` Refloat, который сам пишет в постоянную память |
| `COMM_GET_QML_UI_APP` | 118 | Tool → FC, ответ | **да** для UI | Отдача QML интерфейса чанками | `commands.c` + `tools/gen_qml.py` |
| `COMM_PING_CAN` | 62 | Tool → FC, ответ | опц. | Сканирование устройств на CAN | Отвечаем пустым списком: проксирование CAN запрещено на этом этапе |

### Форматы, критичные для совместимости

**COMM_FW_VERSION** (порядок полей задан `vesc_tool/commands.cpp:109`):

```
[0]  COMM_FW_VERSION
[1]  fw_major            6
[2]  fw_minor            6
[3]  hw_name             "FloatCore\0"
[..] uuid                12 байт
[..] is_paired           0
[..] test_fw             0
[..] hw_type             2 = HW_TYPE_CUSTOM_MODULE
[..] custom_config_num   1
[..] has_phase_filters   0
[..] qml_hw              0
[..] qml_app             1  (2 = fullscreen)
[..] nrf_flags           0
[..] fw_name             "FloatCore\0"
[..] hw_conf_crc         uint32
```

Почему версия именно 6.06: `VescInterface::fwVersionReceived()` сверяет пару
`(major, minor)` со списком поддерживаемых. Незнакомая версия включает
**limited mode**, в котором VESC Tool не запрашивает ни custom config, ни QML —
интерфейс Refloat не откроется. 6.06 присутствует в `res/config` актуального
VESC Tool и включает всё, что нам нужно.

Почему hw_type именно CUSTOM_MODULE: при `HW_TYPE_VESC` VESC Tool считает
устройство контроллером мотора, требует `COMM_GET_MCCONF`/`COMM_GET_APPCONF` и
предлагает обновлять прошивку ESC. CUSTOM_MODULE снимает эти ожидания и при этом
**не мешает** ни custom config, ни QML — они не зависят от типа железа.

**Входы RT App** (31/32/33) — по одному формату: `int32` big-endian со шкалой
1e6, усечение как в прошивке. Полный разбор, значения и границы безопасности —
[rt_app_inputs.md](rt_app_inputs.md).

**Чанковый обмен** (XML схемы и QML) одинаков по смыслу:

```
Запрос:  [cmd][conf_ind?][int32 len][int32 offset]
Ответ:   [cmd][conf_ind?][int32 total][int32 offset][данные]
```

VESC Tool сначала просит 10 байт с нулевым смещением, чтобы узнать `total`, затем
добирает по 400 байт. Наша реализация клипует запрос по границам данных.

**Сжатие.** И XML схемы, и QML передаются в формате `qCompress`: 4 байта
big-endian с размером несжатых данных, затем поток zlib. Так их читает
`qUncompress` в `vescinterface.cpp`.

## 3. Что VESC Tool делает при подключении

Последовательность из `vescinterface.cpp`:

1. `COMM_FW_VERSION` (с повторами до таймаута) → `fwVersionReceived()`.
2. Разбор `custom_config_num` → для каждой конфигурации чанками
   `COMM_GET_CUSTOM_CONFIG_XML`, затем `loadCompressedParamsXml()`.
3. Если `qml_hw` > 0 → чанками `COMM_GET_QML_UI_HW`.
4. Если `qml_app` > 0 → чанками `COMM_GET_QML_UI_APP`, затем `qmlLoadDone()`.
5. Для каждой конфигурации `COMM_GET_CUSTOM_CONFIG` → `customConfigLoadDone()`.
6. Далее по потребности: `COMM_GET_VALUES` на realtime-странице, `COMM_ALIVE`
   как keep-alive, `COMM_CUSTOM_APP_DATA` от QML Refloat, а на открытой странице
   RT App — периодические `COMM_GET_DECODED_PPM`/`_ADC`/`_CHUK`.

Пункты 2 и 4 кэшируются VESC Tool по UUID устройства. Поэтому UUID должен быть
стабильным между запусками — иначе кэш будет расти, а при изменении QML
потребуется его сброс.

## 4. Проверено

`tests/protocol/test_protocol.c` (120 проверок) — кадрирование, CRC, кодировки
ответов, негативные случаи. Линкуется только с `compat/vesc_protocol`.

`tests/host_integration/vesc_tool_sim.py` (43 проверки) — полный сценарий
Definition of Done на настоящем сокете: подключение, идентификация, телеметрия,
загрузка QML, чтение схемы и конфигурации, запись параметра, переподключение,
перезапуск процесса, входы RT App, блокировка команд мотору.

---

# 5. Обратная сторона: FloatCore как клиент CAN (v0.7B)

Всё выше — про FloatCore в роли **устройства**, с которым говорит VESC Tool.
Этот раздел про противоположную роль: FloatCore сам обращается к настоящему
VESC по шине CAN. Роли не путать — наборы команд у них разные, и опасен здесь
именно клиентский набор: он уходит в железо, способное крутить мотор.

Источник — `vedderb/bldc`, ветка `release_6_06`: ровно та версия прошивки,
которая стоит на стенде (`FW 6.6`, прочитано и по USB, и по CAN). Ничего не
выведено из наблюдаемого трафика: номера и семантика взяты из `datatypes.h` и
обработчиков `comm/comm_can.c` и `comm/commands.c`.

Классификация продублирована в коде (`compat/can/fc_vesc_can.c`) и проверяется
тестами поимённо, чтобы расхождение между документом и сборкой не могло
остаться незамеченным.

## 5.1. Как адресуются пакеты

```
передача:  eid = controller_id | ((uint32_t) packet_id << 8)
приём:     uint8_t id = eid & 0xFF;  CAN_PACKET_ID cmd = eid >> 8;
```

Получатель обрабатывает пакет, если `id == 255 || id == id1 || id == id2`
(`comm_can.c:1585`), где `id1` и `id2` — номера двух половин Dual FSESC. Из
этого следует, что **широковещательный адрес 255 отвечает обеими половинами
сразу**, и различить ответы нечем. FloatCore адрес 255 не собирает: попытка
возвращает `FC_CAN_DIAG_BUILD_BROADCAST`.

Собственный номер FloatCore на шине — **50**. Он не совпадает ни с 118, ни со
100, и это закреплено `_Static_assert` в `fc_can_diag.h`.

## 5.2. Полный перечень типов пакетов CAN

Колонка «разрешено передавать» относится к профилю `ACTIVE_DIAG`. Пустая
клетка означает, что собрать такой пакет в этом профиле нечем — не «запрещено
правилом», а «нет кода».

| № | Тип | Класс | Разрешено передавать |
|---|---|---|---|
| 0 | `CAN_PACKET_SET_DUTY` | **команда мотору** | — |
| 1 | `CAN_PACKET_SET_CURRENT` | **команда мотору** | — |
| 2 | `CAN_PACKET_SET_CURRENT_BRAKE` | **команда мотору** | — |
| 3 | `CAN_PACKET_SET_RPM` | **команда мотору** | — |
| 4 | `CAN_PACKET_SET_POS` | **команда мотору** | — |
| 5 | `CAN_PACKET_FILL_RX_BUFFER` | буфер COMM | — |
| 6 | `CAN_PACKET_FILL_RX_BUFFER_LONG` | буфер COMM | — |
| 7 | `CAN_PACKET_PROCESS_RX_BUFFER` | буфер COMM | — |
| 8 | `CAN_PACKET_PROCESS_SHORT_BUFFER` | буфер COMM | **да** |
| 9 | `CAN_PACKET_STATUS` | статус (только приём) | — |
| 10 | `CAN_PACKET_SET_CURRENT_REL` | **команда мотору** | — |
| 11 | `CAN_PACKET_SET_CURRENT_BRAKE_REL` | **команда мотору** | — |
| 12 | `CAN_PACKET_SET_CURRENT_HANDBRAKE` | **команда мотору** | — |
| 13 | `CAN_PACKET_SET_CURRENT_HANDBRAKE_REL` | **команда мотору** | — |
| 14 | `CAN_PACKET_STATUS_2` | статус (только приём) | — |
| 15 | `CAN_PACKET_STATUS_3` | статус (только приём) | — |
| 16 | `CAN_PACKET_STATUS_4` | статус (только приём) | — |
| 17 | `CAN_PACKET_PING` | телеметрия, BMS, IO | **да** |
| 18 | `CAN_PACKET_PONG` | ответ / опрос | — |
| 19 | `CAN_PACKET_DETECT_APPLY_ALL_FOC` | меняет состояние | — |
| 20 | `CAN_PACKET_DETECT_APPLY_ALL_FOC_RES` | ответ / опрос | — |
| 21 | `CAN_PACKET_CONF_CURRENT_LIMITS` | меняет состояние | — |
| 22 | `CAN_PACKET_CONF_STORE_CURRENT_LIMITS` | меняет состояние | — |
| 23 | `CAN_PACKET_CONF_CURRENT_LIMITS_IN` | меняет состояние | — |
| 24 | `CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN` | меняет состояние | — |
| 25 | `CAN_PACKET_CONF_FOC_ERPMS` | меняет состояние | — |
| 26 | `CAN_PACKET_CONF_STORE_FOC_ERPMS` | меняет состояние | — |
| 27 | `CAN_PACKET_STATUS_5` | статус (только приём) | — |
| 28 | `CAN_PACKET_POLL_TS5700N8501_STATUS` | ответ / опрос | — |
| 29 | `CAN_PACKET_CONF_BATTERY_CUT` | меняет состояние | — |
| 30 | `CAN_PACKET_CONF_STORE_BATTERY_CUT` | меняет состояние | — |
| 31 | `CAN_PACKET_SHUTDOWN` | меняет состояние | — |
| 32 | `CAN_PACKET_IO_BOARD_ADC_1_TO_4` | телеметрия, BMS, IO | — |
| 33 | `CAN_PACKET_IO_BOARD_ADC_5_TO_8` | телеметрия, BMS, IO | — |
| 34 | `CAN_PACKET_IO_BOARD_ADC_9_TO_12` | телеметрия, BMS, IO | — |
| 35 | `CAN_PACKET_IO_BOARD_DIGITAL_IN` | телеметрия, BMS, IO | — |
| 36 | `CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL` | меняет состояние | — |
| 37 | `CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM` | меняет состояние | — |
| 38 | `CAN_PACKET_BMS_V_TOT` | телеметрия, BMS, IO | — |
| 39 | `CAN_PACKET_BMS_I` | телеметрия, BMS, IO | — |
| 40 | `CAN_PACKET_BMS_AH_WH` | телеметрия, BMS, IO | — |
| 41 | `CAN_PACKET_BMS_V_CELL` | телеметрия, BMS, IO | — |
| 42 | `CAN_PACKET_BMS_BAL` | меняет состояние | — |
| 43 | `CAN_PACKET_BMS_TEMPS` | телеметрия, BMS, IO | — |
| 44 | `CAN_PACKET_BMS_HUM` | телеметрия, BMS, IO | — |
| 45 | `CAN_PACKET_BMS_SOC_SOH_TEMP_STAT` | телеметрия, BMS, IO | — |
| 46 | `CAN_PACKET_PSW_STAT` | телеметрия, BMS, IO | — |
| 47 | `CAN_PACKET_PSW_SWITCH` | меняет состояние | — |
| 48 | `CAN_PACKET_BMS_HW_DATA_1` | телеметрия, BMS, IO | — |
| 49 | `CAN_PACKET_BMS_HW_DATA_2` | телеметрия, BMS, IO | — |
| 50 | `CAN_PACKET_BMS_HW_DATA_3` | телеметрия, BMS, IO | — |
| 51 | `CAN_PACKET_BMS_HW_DATA_4` | телеметрия, BMS, IO | — |
| 52 | `CAN_PACKET_BMS_HW_DATA_5` | телеметрия, BMS, IO | — |
| 53 | `CAN_PACKET_BMS_AH_WH_CHG_TOTAL` | телеметрия, BMS, IO | — |
| 54 | `CAN_PACKET_BMS_AH_WH_DIS_TOTAL` | телеметрия, BMS, IO | — |
| 55 | `CAN_PACKET_UPDATE_PID_POS_OFFSET` | меняет состояние | — |
| 56 | `CAN_PACKET_POLL_ROTOR_POS` | ответ / опрос | — |
| 57 | `CAN_PACKET_NOTIFY_BOOT` | ответ / опрос | — |
| 58 | `CAN_PACKET_STATUS_6` | статус (только приём) | — |
| 59 | `CAN_PACKET_GNSS_TIME` | телеметрия, BMS, IO | — |
| 60 | `CAN_PACKET_GNSS_LAT` | телеметрия, BMS, IO | — |
| 61 | `CAN_PACKET_GNSS_LON` | телеметрия, BMS, IO | — |
| 62 | `CAN_PACKET_GNSS_ALT_SPEED_HDOP` | телеметрия, BMS, IO | — |
| 63 | `CAN_PACKET_UPDATE_BAUD` | меняет состояние | — |
| 64 | `CAN_PACKET_BMS_STATUS_1` | телеметрия, BMS, IO | — |
| 65 | `CAN_PACKET_BMS_STATUS_2` | телеметрия, BMS, IO | — |
| 66 | `CAN_PACKET_BMS_STATUS_3` | телеметрия, BMS, IO | — |
| 67 | `CAN_PACKET_BMS_STATUS_4` | телеметрия, BMS, IO | — |
| 68 | `CAN_PACKET_BMS_STATUS_5` | телеметрия, BMS, IO | — |

Разрешены ровно два номера из 69.

## 5.3. Почему `PROCESS_SHORT_BUFFER` разрешён

Сам по себе тип 8 состояние не меняет: он несёт внутри обычный пакет `COMM_*`
и отдаёт его в `commands_process_packet()` (`comm_can.c:1749-1788`). То есть
безопасность решается **не номером типа CAN, а номером COMM внутри него**:
через тот же тип 8 одинаково проходит и запрос версии прошивки, и
`COMM_SET_MCCONF`.

Поэтому номер COMM в FloatCore задаётся таблицей внутри `fc_can_diag.c` и
никогда не приходит от вызывающего кода. Вызывающий называет намерение
(`FC_CAN_DIAG_FW_VERSION`), а не пакет.

Формат запроса, `comm_can.c:1749-1777`:

```
data[0] = кому отвечать (наш номер, 50)
data[1] = режим: 0 = обработать и ответить отправителю
data[2] = номер COMM
```

Режим 0 выбран сознательно: 1 пересылает пакет дальше без обработки, 2
обрабатывает молча без ответа, 3 отвечает без обёртки.

## 5.4. Белый список COMM

| № | Пакет | Почему безопасен |
|---|---|---|
| 0 | `COMM_FW_VERSION` | `commands.c:231` — читает константы сборки и UUID |
| 4 | `COMM_GET_VALUES` | `commands.c:384` — упаковывает телеметрию |
| 14 | `COMM_GET_MCCONF` | `commands.c:591` — копия `mc_interface_get_configuration()` |
| 15 | `COMM_GET_MCCONF_DEFAULT` | там же, значения по умолчанию |
| 17 | `COMM_GET_APPCONF` | `commands.c:654` — копия `app_get_configuration()` |
| 18 | `COMM_GET_APPCONF_DEFAULT` | там же, значения по умолчанию |
| 50 | `COMM_GET_VALUES_SELECTIVE` | та же ветка, что `GET_VALUES` |

Проверялось **отсутствие побочного эффекта**, а не то, что название начинается
с `GET`. Показательная проверка: `timeout_reset()` в `commands.c` встречается
только в ветках `COMM_SET_DUTY` (488), `COMM_ALIVE` (700), `COMM_SET_ODOMETER`
(893) и `COMM_SET_CURRENT_REL` (1185). Ни одной из них в списке нет.

Умолчание — **«нельзя»**: `fc_vesc_comm_is_read_only()` возвращает `false` для
любого номера вне списка, включая номера, которых в 6.6 ещё нет. Появление
новой команды в будущей прошивке не откроет её автоматически.

## 5.5. Явно запрещённое

Ни один из этих пакетов не может быть собран в профиле `ACTIVE_DIAG`, и это
проверяется негативным тестом компиляции, а не проверкой во время работы.

**Команды мотору:** `SET_DUTY`, `SET_CURRENT`, `SET_CURRENT_BRAKE`, `SET_RPM`,
`SET_POS`, `SET_CURRENT_REL`, `SET_CURRENT_BRAKE_REL`,
`SET_CURRENT_HANDBRAKE`, `SET_CURRENT_HANDBRAKE_REL` — типы 0-4 и 10-13.

**Через уровень COMM:** `COMM_SET_DUTY`, `COMM_SET_CURRENT`,
`COMM_SET_CURRENT_BRAKE`, `COMM_SET_RPM`, `COMM_SET_POS`, `COMM_SET_HANDBRAKE`,
`COMM_SET_SERVO_POS`, `COMM_SET_DETECT`.

**Изменение конфигурации:** `COMM_SET_MCCONF`, `COMM_SET_APPCONF`,
`COMM_SET_MCCONF_TEMP`, `COMM_SET_MCCONF_TEMP_SETUP`, а также
`CAN_PACKET_CONF_*` (21-26, 29, 30) — включая варианты `CONF_STORE_*`, которые
пишут во flash.

**Детекция:** `CAN_PACKET_DETECT_APPLY_ALL_FOC` (19),
`COMM_DETECT_MOTOR_PARAM`, `COMM_DETECT_MOTOR_R_L`,
`COMM_DETECT_MOTOR_FLUX_LINKAGE`, `COMM_DETECT_ENCODER`, `COMM_DETECT_HALL_FOC`.

**Прошивка и перезапуск:** `COMM_JUMP_TO_BOOTLOADER`, `COMM_ERASE_NEW_APP`,
`COMM_WRITE_NEW_APP_DATA`, `COMM_ERASE_BOOTLOADER`, `COMM_REBOOT`,
`CAN_PACKET_SHUTDOWN` (31).

**Прочее, меняющее состояние:** `CAN_PACKET_UPDATE_BAUD` (63) — смена скорости
шины, `IO_BOARD_SET_OUTPUT_DIGITAL` (36) и `_PWM` (37), `BMS_BAL` (42),
`PSW_SWITCH` (47), `UPDATE_PID_POS_OFFSET` (55), `COMM_TERMINAL_CMD` (20),
`COMM_ALIVE` (30) — продлевает моторный таймаут, `COMM_CUSTOM_APP_DATA` (36).

## 5.6. Формат ответа

Короткий ответ (полезная нагрузка ≤ 6 байт) приходит одним
`PROCESS_SHORT_BUFFER`: `[кто ответил][режим][данные…]`.

Длинный собирается из кусков (`comm_can.c:443-503`):

```
FILL_RX_BUFFER (5)        [смещение 1 байт][до 7 байт данных]
FILL_RX_BUFFER_LONG (6)   [смещение 2 байта][до 6 байт данных]
PROCESS_RX_BUFFER (7)     [кто][режим][len_hi][len_lo][crc_hi][crc_lo]
```

CRC — та же `crc16` CCITT (полином 0x1021, начальное 0), что в кадрах по USB.
FloatCore пользуется общей реализацией `vesc_crc16()` из
`compat/vesc_protocol/packet.c`, а не собственной копией: разойтись с VESC в
контрольной сумме означало бы принимать мусор за ответ. Ответ без совпадения
CRC отбрасывается и считается в `crc_errors`.
