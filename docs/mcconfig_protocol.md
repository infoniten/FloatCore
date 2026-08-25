# Протокол Virtual mcConfig

## 1. Как VESC Tool получает Motor Configuration

```
Tool → FC   [COMM_GET_MCCONF]                        запрос
FC → Tool   [COMM_GET_MCCONF][uint32 signature][~200 параметров подряд]
```

Приёмник — `Commands::processPacket()` в `vesc_tool/commands.cpp`:

```cpp
case COMM_GET_MCCONF:
case COMM_GET_MCCONF_DEFAULT:
    mTimeoutMcconf = 0;
    if (mMcConfig) {
        if (mMcConfig->deSerialize(vb)) {
            mMcConfig->updateDone();
            ...
        } else {
            emit deserializeConfigFailed(true, false);
        }
    }
```

Два факта, которые определяют всю конструкцию:

1. **Обработчик не проверяет, запрашивал ли Tool конфигурацию.** Любой пришедший
   кадр `COMM_GET_MCCONF` будет разобран и применён. Это позволяет отдать
   конфигурацию инициативно.
2. **`deSerialize()` сверяет сигнатуру** со схемой, загруженной самим Tool.
   При несовпадении — `deserializeConfigFailed` и модальный диалог.

```cpp
bool ConfigParams::deSerialize(VByteArray &vb) {
    auto signature = vb.vbPopFrontUint32();
    if (signature != getSignature()) {
        qWarning() << "Invalid signature";
        return false;
    }
    for (int i = 0; i < mSerializeOrder.size(); i++) {
        setParamSerial(vb, mSerializeOrder.at(i));
    }
    ...
}
```

## 2. Почему приходится отдавать инициативно

VESC Tool запрашивает конфигурацию мотора только у настоящих контроллеров:

```qml
// mobile/main.qml:839
if (VescIf.isPortConnected() && VescIf.getLastFwRxParams().hwTypeStr() === "VESC") {
```

FloatCore объявляет себя `HW_TYPE_CUSTOM_MODULE` и менять это не собирается —
он не контроллер мотора. Поэтому конфигурация отправляется сразу после ответа
на `COMM_FW_VERSION`, который служит надёжным признаком «Tool подключился»:

```
Tool → FC   [COMM_FW_VERSION]
FC → Tool   [COMM_FW_VERSION][...]        идентификация
FC → Tool   [COMM_GET_MCCONF][...]        инициативно, без запроса
```

Явный запрос тоже обслуживается: если Tool или скрипт пришлёт
`COMM_GET_MCCONF`, ответ придёт обычным порядком.

Поведение отключается флагом `--no-mcconf` и полем
`VescServer.mcconf_push_on_connect`.

## 3. Сигнатура

`ConfigParams::getSignature()`:

```cpp
QString sigStr;
for (QString s: mSerializeOrder) {
    sigStr.append(s);
    ConfigParam *p = getParam(s);
    if (p) {
        sigStr.append(QString("%1").arg(int(p->type)));
        sigStr.append(QString("%1").arg(int(p->vTx)));
        for (auto n: p->enumNames) sigStr.append(n);
    }
}
return Utility::crc32c((uint8_t*)bytes.data(), bytes.size());
```

CRC-32C (полином `0x82F63B78`, init `0xFFFFFFFF`, финальная инверсия).

**Алгоритм сверен с эталоном.** Для примера конфигурации из
`vesc_pkg_lib/examples/config/conf/settings.xml` он даёт `32903057` — ровно то
значение, которое сам VESC Tool записал в сгенерированный им `confparser.h`.
Это же исправило скрытую ошибку в конфигурации Refloat: до версии 0.4.1
`tools/gen_conf.py` считал сигнатуру собственным алгоритмом, и настоящий
VESC Tool отверг бы наш блоб с сообщением «Could not deserialize custom config».

Текущие значения:

| Схема | Параметров | Размер блоба | Сигнатура |
|---|---|---|---|
| VESC Tool 6.06 | 200 | 483 байта | 788332866 |
| VESC Tool 7.01 | 202 | 488 байт | 3154770096 |
| Refloat settings.xml | 172 | 282 байта | 421162011 |

Блоб вместе с байтом команды — 484/489 байт, помещается в один пакет VESC
(`PACKET_MAX_PL_LEN` = 512). Запаса немного: следующая версия схемы может
не поместиться, тогда потребуется чанковая передача.

## 4. Кодирование параметров

`ConfigParams::setParamSerial()` определяет формат по паре `(type, vTx)`:

| type | vTx | Кодирование | Байт |
|---|---|---|---|
| `CFG_T_DOUBLE` (1) | `VESC_TX_DOUBLE16` (7) | `int16` = значение × `vTxDoubleScale` | 2 |
| `CFG_T_DOUBLE` | `VESC_TX_DOUBLE32` (8) | `int32` = значение × scale | 4 |
| `CFG_T_DOUBLE` | `VESC_TX_DOUBLE32_AUTO` (9) | `buffer_append_float32_auto` | 4 |
| `CFG_T_INT` (2) | `UINT8`/`INT8` (1/2) | целое | 1 |
| `CFG_T_INT` | `UINT16`/`INT16` (3/4) | целое | 2 |
| `CFG_T_INT` | `UINT32`/`INT32` (5/6) | целое | 4 |
| `CFG_T_ENUM` (4), `CFG_T_BOOL` (5), `CFG_T_BITFIELD` (6) | — | один байт независимо от vTx | 1 |

`float32_auto` — нестандартный формат VESC (`bldc/util/buffer.c`): мантисса и
экспонента упаковываются вручную через `frexpf`. Реализован в
`virtual_mcconf_float32_auto()` побитово идентично оригиналу.

Строковых параметров в схеме mcconf нет — проверено для 6.06 и 7.01.

## 5. Генерация таблицы для новой версии VESC Tool

```bash
python3 tools/gen_mcconf.py \
    --xml /path/to/vesc_tool/res/config/7.02/parameters_mcconf.xml \
    --version 7.02 \
    --out compat/vesc_protocol/generated
```

Появятся `mcconf_schema_7_02.c` (таблица для прошивки) и
`mcconf_schema_7_02.json` (для тестов на Python). Дальше нужно добавить
`extern` в `virtual_mcconf.h` и запись в массив `SCHEMAS` в
`virtual_mcconf.c` — новая схема автоматически станет схемой по умолчанию,
если поставить её первой.

Сам XML в репозиторий не вносится: он весит ~400 КБ и принадлежит VESC Tool.
Вносится только сгенерированная таблица (~13 КБ).

## 6. Реализованные обработчики

| Команда | ID | Направление | Реализация |
|---|---|---|---|
| `COMM_GET_MCCONF` | 14 | Tool → FC, ответ | Проекция FloatCore Config |
| `COMM_GET_MCCONF_DEFAULT` | 15 | Tool → FC, ответ | Значения по умолчанию **схемы VESC Tool**, без проекции — кнопка «Restore default» не должна возвращать наши пределы как «заводские» |
| `COMM_GET_MCCONF` | 14 | FC → Tool, инициативно | Отправляется после ответа на `COMM_FW_VERSION` |
| `COMM_SET_MCCONF` | 13 | Tool → FC | **Игнорируется.** Счётчик `mcconf_writes_rejected`, ответа нет |

`COMM_GET_MCCONF_TEMP`, `COMM_SET_MCCONF_TEMP` и профили не реализованы:
Refloat QML их не использует.

## 7. Read-only гарантия

Записи не существует на уровне конструкции, а не соглашения:

1. Обработчика, применяющего `COMM_SET_MCCONF`, нет — команда попадает в
   явную ветку «посчитать и проигнорировать».
2. `virtual_mcconf_encode()` принимает `const VirtualMcConfValues *` и ничего
   никуда не пишет.
3. `FloatCoreLimits` меняется только через `floatcore_limits_set_*()`, которые
   вызываются при старте и (в будущем) драйвером CAN — но не протокольным слоем.

Проверяется тестом `test_mcconf_readonly`: полный блоб с током 200 А и 30
ячейками отправляется как `COMM_SET_MCCONF`, после чего пределы FloatCore
перечитываются и оказываются неизменными.

## 8. Трассировка

```
build/floatcore_host --trace
```

```
RX  id=0   FW_VERSION               len=1
TX  id=0   FW_VERSION               len=47
TX  id=14  GET_MCCONF               len=489
!!  id=13  SET_MCCONF               len=489  motor config write IGNORED (Virtual mcConfig is read-only)
```
