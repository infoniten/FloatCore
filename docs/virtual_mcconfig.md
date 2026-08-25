# Virtual mcConfig: аудит и назначение

## 1. Проблема

Refloat QML читает пределы из конфигурации мотора:

```qml
property ConfigParams mcConfig: VescIf.mcConfig()
```

FloatCore представляется как `HW_TYPE_CUSTOM_MODULE`, а VESC Tool запрашивает
конфигурацию мотора только у настоящих контроллеров:

```qml
// vesc_tool/mobile/main.qml:839
if (VescIf.isPortConnected() && VescIf.getLastFwRxParams().hwTypeStr() === "VESC") {
    if (!mcConfRx) { mCommands.getMcconf() }
```

и в `VescInterface::fwVersionReceived()`:

```cpp
if (params.hwType == HW_TYPE_VESC && (fwPairs.contains(fw_connected) || ...)) {
    ...
    mFwSupportsConfiguration = true;
}
```

Поэтому `VescIf.mcConfig()` оставался заполненным значениями по умолчанию из
локальной схемы VESC Tool. Шкалы и пороги в интерфейсе Refloat не соответствовали
реальным ограничениям FloatCore.

На балансировку это не влияло: Refloat берёт те же величины напрямую через
`VESC_IF->get_cfg_float()`. Расхождение было только между UI и системой.

## 2. Полный аудит обращений

Все обращения к `mcConfig` в `refloat-upstream/ui.qml.in` (v1.3.0) — строки
240-271. Больше нигде в файле `mcConfig` не встречается.

| Parameter | Used in | Purpose | FloatCore source | Required |
|---|---|---|---|---|
| `si_battery_cells` | `ui.qml.in:243` объявление; `:6580` напряжение на ячейку; **`:1400-1406` миграция тюна 1.1** | Деление напряжения пакета на число ячеек; пересчёт порогов LV/HV при миграции старого тюна | `FcBatteryConfig.cell_count` | **да** |
| `l_temp_motor_start` | `:244`; `:7457` порог предупреждения | Жёлтая зона на индикаторе температуры мотора | `fc_effective_temp_motor_start()` | **да** |
| `l_temp_fet_start` | `:245`; `:7465` порог предупреждения | Жёлтая зона на индикаторе температуры контроллера | `fc_effective_temp_fet_start()` | **да** |
| `l_current_min` | `:246`; `:6817` `minValue` шкалы тока | Левая граница шкалы тока мотора | `fc_effective_current_min()` | **да** |
| `l_current_max` | `:247`; `:6818` `maxValue` шкалы тока | Правая граница шкалы тока мотора | `fc_effective_current_max()` | **да** |
| `l_in_current_min` | `:248`; `:6883` `minValue` шкалы тока батареи | Левая граница шкалы тока батареи (в т.ч. логарифмической) | `fc_effective_in_current_min()` | **да** |
| `l_in_current_max` | `:249`; `:6884` `maxValue` шкалы тока батареи | Правая граница шкалы тока батареи | `fc_effective_in_current_max()` | **да** |
| `l_temp_motor_end` | не читается QML | Согласованность: без него в VESC Tool окажется `end` из чужой схемы при нашем `start` | `fc_effective_temp_motor_end()` | для согласованности |
| `l_temp_fet_end` | не читается QML | То же | `fc_effective_temp_fet_end()` | для согласованности |

Остальные ~193 параметра схемы отдаются со значениями по умолчанию самой схемы
VESC Tool и никак не используются Refloat. Это требование «минимальной модели»:
реализовано ровно то, что читается.

### Что из этого влияет на безопасность

Восемь из девяти параметров — чистое отображение: границы шкал и пороги
предупреждений. Ошибка в них приводит к неправильно нарисованной полоске,
не более.

Исключение — **`si_battery_cells`**. Он используется не только для отображения:

```qml
// ui.qml.in:1400
if (fwMajor > 6 || (fwMajor == 6 && fwMinor >= 5) && motorConfig.batteryCells > 0) {
    if (tune.settings["tiltback_hv"] > 10) {
        tune.settings["tiltback_hv"] /= motorConfig.batteryCells;
    }
    if (tune.settings["tiltback_lv"] > 10) {
        tune.settings["tiltback_lv"] /= motorConfig.batteryCells;
    }
}
```

Это миграция импортируемого тюна: пороги LV/HV из абсолютных вольт переводятся
в вольты на ячейку. Неверное число ячеек даёт неверные пороги низкого и
высокого напряжения — а они управляют tiltback-ом. Значение попадает в
записываемую конфигурацию Refloat, а не только на экран.

Вывод: `si_battery_cells` обязан быть настоящим. Именно поэтому источником
служит `FcBatteryConfig`, из которого его читает и сам Refloat
(`get_cfg_int(CFG_PARAM_si_battery_cells)` в `motor_data_refresh_motor_config`).

## 3. Что такое Virtual mcConfig

Read-only проекция состояния FloatCore в формат Motor Configuration VESC.

Не является конфигурацией мотора. Не имеет собственного хранилища. Значения
вычисляются на каждый запрос:

```
FloatCore Config (compat/config/floatcore_limits.h)
        │
        ├─────────────► VESC_IF->get_cfg_float()  ──► Refloat (балансировка)
        │
        └─► Virtual mcConfig ──► COMM_GET_MCCONF ──► VescIf.mcConfig() ──► Refloat QML
```

Обе стрелки выходят из одной структуры. Отдельного config store не существует —
проверяется тестом `11. единый источник пределов`, который сравнивает то, что
прочитал Refloat, с тем, что отдаёт проекция.

## 4. Ограничение: схема зависит от версии VESC Tool

`ConfigParams::deSerialize()` сверяет сигнатуру блоба с сигнатурой схемы,
загруженной самим VESC Tool. Схема выбирается по его собственной версии
(`Utility::configLatestSupported()` возвращает `VT_VERSION` сборки), а не по
версии, которую сообщили мы.

Поэтому FloatCore хранит по таблице на каждую поддерживаемую версию схемы и
выбирает нужную:

```bash
build/floatcore_host --mcconf-schema 6.06   # для VESC Tool 6.06
build/floatcore_host --mcconf-schema 7.01   # по умолчанию
build/floatcore_host --no-mcconf            # отключить целиком
```

**При несовпадении сигнатуры VESC Tool показывает диалог** «Deserializing motor
configuration failed». Это единственное неприятное последствие неверного
выбора; ни соединение, ни конфигурация Refloat при этом не страдают.
Если версия вашего VESC Tool не входит в список — либо отключите Virtual
mcConfig, либо сгенерируйте таблицу командой из
[mcconfig_protocol.md](mcconfig_protocol.md) §5.

## 5. Проверка

| Что | Где |
|---|---|
| Кодировка, сигнатура, размер, все 9 параметров фикстуры | `tests/protocol/test_protocol.c`, `test_virtual_mcconf` |
| Правила агрегации двух ESC | `test_mcconf_aggregation` |
| Невозможность записи | `test_mcconf_readonly` |
| Инициативная отправка после `COMM_FW_VERSION` | `test_mcconf_push_on_connect` |
| Refloat и проекция читают одни и те же пределы | `tests/host/scenarios.c`, сценарий 11 |
| Полный путь по сокету с фикстурой ТЗ | `tests/host_integration/vesc_tool_sim.py` |

Фикстура везде одна — из ТЗ v0.4.1 §8: 10S, мотор 25 A, тормоз −5 A, вход 15 A,
рекуперация 0 A, FET 80/100 °C.

**Не проверено:** отрисовка шкал в живом VESC Tool — GUI в среде разработки
отсутствует.
