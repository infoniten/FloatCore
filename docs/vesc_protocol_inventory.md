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
   как keep-alive, `COMM_CUSTOM_APP_DATA` от QML Refloat.

Пункты 2 и 4 кэшируются VESC Tool по UUID устройства. Поэтому UUID должен быть
стабильным между запусками — иначе кэш будет расти, а при изменении QML
потребуется его сброс.

## 4. Проверено

`tests/protocol/test_protocol.c` (71 проверка) — кадрирование, CRC, кодировки
ответов, негативные случаи. Линкуется только с `compat/vesc_protocol`.

`tests/host_integration/vesc_tool_sim.py` (25 проверок) — полный сценарий
Definition of Done на настоящем сокете: подключение, идентификация, телеметрия,
загрузка QML, чтение схемы и конфигурации, запись параметра, переподключение,
перезапуск процесса, блокировка команд мотору.
