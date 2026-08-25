# Зависимости QML-интерфейса Refloat от backend VESC Tool

Объект анализа — `refloat-upstream/ui.qml.in` (v1.3.0, ~4700 строк). Цель:
понять, что должен предоставить FloatCore, чтобы использовать этот QML
**без изменений**.

Вывод: изменения не требуются. Один параметр отображается неточно — см. §3.

## 1. Карта вызовов

Формат: `QML` → `VESC Tool Backend` → `Protocol Command` → `FloatCore Handler`.

| QML | Backend | Команда | Обработчик FloatCore | Статус |
|---|---|---|---|---|
| `VescIf.customConfig(0)` | `VescInterface::customConfig()` | `COMM_GET_CUSTOM_CONFIG_XML` + `COMM_GET_CUSTOM_CONFIG` | `config_bridge.c` → `get_cfg_xml`/`get_cfg` Refloat | ✅ |
| `VescIf.customConfigsLoaded()` | локальное состояние после загрузки | — | косвенно, через успешную загрузку схемы | ✅ |
| `vescCommands.customConfigSet(0, pkgConfig)` | `Commands::customConfigSet()` | `COMM_SET_CUSTOM_CONFIG` | `config_bridge.c` → `set_cfg` Refloat | ✅ |
| `pkgConfig.getParam*/setParam*` | `ConfigParams` из загруженного XML | — | схема = `settings.xml` Refloat | ✅ |
| `vescCommands.sendCustomAppData(buf)` (13 мест) | `Commands::sendCustomAppData()` | `COMM_CUSTOM_APP_DATA` | `custom_app.c` → `set_app_data_handler` Refloat | ✅ |
| `vescCommands.customAppDataReceived` | сигнал `Commands` | `COMM_CUSTOM_APP_DATA` (обратно) | `send_app_data` Refloat → `vesc_server_send_custom_app_data` | ✅ |
| `VescIf.getLastFwRxParams()` | сохранённые `FW_RX_PARAMS` | `COMM_FW_VERSION` | `firmware_info.c` | ✅ |
| `VescIf.mcConfig()` | `ConfigParams` конфигурации мотора | `COMM_GET_MCCONF` | **не реализуется** | ⚠ см. §3 |
| `VescIf.emitStatusMessage()` (6) | строка состояния VESC Tool | — | локально | ✅ |
| `VescIf.emitMessageDialog()` (2) | диалог VESC Tool | — | локально | ✅ |
| `VescIf.useImperialUnits()`, `storeSettings()` | настройки VESC Tool | — | локально | ✅ |
| `VescIf.getLastBleAddr()`, `getBleName()`, `storeBleName()` | адресная книга BLE | — | локально; при TCP-подключении адрес пуст | ✅ вырожденно |
| `Utility.getAppHexColor()` (86), `isDarkMode()` | тема оформления | — | локально | ✅ |
| `Utility.requestFilePermission()` (2) | разрешения Android | — | локально | ✅ |
| `confCustomLoader.item` (хук кнопки Write) | страница конфигурации мобильного UI | — | локально | ✅ вырожденно |

Ничего из перечисленного не требует ни `COMM_LISP_*`, ни механики `.vescpkg`,
ни файловой системы устройства.

## 2. Как Refloat UI попадает в VESC Tool

Штатный путь пакета — загрузка `.vescpkg` в область Lisp-кода прошивки, откуда
VESC Tool извлекает QML. FloatCore идёт коротким путём: отдаёт тот же QML
напрямую через `COMM_GET_QML_UI_APP`, объявив `qml_app = 1` в `COMM_FW_VERSION`.

```
ui.qml.in
   │ tools/gen_qml.py: подстановка {{PACKAGE_NAME}}/{{VERSION}}, qCompress
   ▼
build/gen/qml_app.c  (51 339 байт из 289 020)
   │ COMM_GET_QML_UI_APP, чанки по 400 байт
   ▼
VescInterface: qUncompress → mQmlApp → qmlLoadDone()
   ▼
вкладка Refloat в VESC Tool
```

Плюсы такого решения: не нужно эмулировать ни LispBM, ни формат пакета, ни
файловую систему. Минус: VESC Tool не покажет FloatCore на странице
VESC Packages — но там и нечего показывать, пакет не устанавливается, он
встроен в прошивку.

Проверено в `tests/host_integration/vesc_tool_sim.py`: QML забирается целиком,
распаковывается, содержит `tabTitle: "Refloat"` и корректные подстановки версии.

## 3. Единственная неточность: `VescIf.mcConfig()`

`ui.qml.in:240-249` читает из конфигурации мотора:

```qml
property int  batteryCells   = mcConfig.getParamInt("si_battery_cells")
property real tempMotorStart = mcConfig.getParamDouble("l_temp_motor_start")
property real tempFetStart   = mcConfig.getParamDouble("l_temp_fet_start")
property real currentMin     = mcConfig.getParamDouble("l_current_min")
property real currentMax     = mcConfig.getParamDouble("l_current_max")
property real inCurrentMin   = mcConfig.getParamDouble("l_in_current_min")
property real inCurrentMax   = mcConfig.getParamDouble("l_in_current_max")
```

Это границы шкал и порогов на экранах Refloat. FloatCore не отдаёт `mcconf`
(hw_type = CUSTOM_MODULE), поэтому VESC Tool подставит значения из своей
локальной схемы конфигурации, а не с устройства. Последствие: шкалы токов,
температур и уровень заряда по числу ячеек могут отображаться относительно
не тех пределов, что настроены в реальных FSESC.

На балансировку это не влияет: те же величины Refloat берёт напрямую через
`VESC_IF->get_cfg_float()`, а не из QML.

Варианты закрытия, в порядке предпочтения:

1. **Ничего не делать в v1.** Косметика, видна только в UI.
2. Реализовать `COMM_GET_MCCONF` с mcconf-совместимым блобом, оставаясь
   CUSTOM_MODULE. Дорого: формат mcconf версионный и большой.
3. Патч в `ui.qml.in`, берущий пределы из конфигурации Refloat. Нарушает
   принцип «не менять QML», зато точен. Кандидат, если пользователи будут
   путаться в шкалах.

## 4. Отличия десктопной и мобильной оболочек

`Component.onCompleted` ищет `confCustomLoader` — объект, который существует
только в мобильной QML-оболочке VESC Tool. На десктопе Refloat печатает
«There is no reliable config updated signal» и работает дальше. Это
поведение upstream, к FloatCore отношения не имеет.

## 5. Что проверено, а что нет

Проверено автоматически: QML отдаётся целиком и корректно распаковывается,
`COMM_CUSTOM_APP_DATA` ходит в обе стороны (команда `INFO` Refloat отвечает
«Refloat 1.3.0»), конфигурация читается и пишется через ту же схему, которую
получает QML.

**Не проверено:** отрисовка интерфейса в настоящем VESC Tool — GUI в этой
среде отсутствует. Проверка «открыть вкладку Refloat и покликать» остаётся
за человеком.
