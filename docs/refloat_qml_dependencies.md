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
| `VescIf.mcConfig()` | `ConfigParams` конфигурации мотора | `COMM_GET_MCCONF` | `virtual_mcconf.c` — read-only проекция FloatCore Config | ✅ с версии 0.4.1 |
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

## 3. `VescIf.mcConfig()` — закрыто в версии 0.4.1

`ui.qml.in:240-249` читает из конфигурации мотора семь параметров: пределы
токов, пороги температур и число ячеек батареи. Раньше FloatCore их не отдавал,
и VESC Tool подставлял собственные значения по умолчанию — шкалы не
соответствовали реальным ограничениям.

Теперь FloatCore отдаёт **Virtual mcConfig** — read-only проекцию своей
конфигурации в формате Motor Configuration VESC. Полный аудит и обоснование:
[virtual_mcconfig.md](virtual_mcconfig.md), отображение параметров:
[mcconfig_mapping.md](mcconfig_mapping.md), протокол:
[mcconfig_protocol.md](mcconfig_protocol.md).

Отдельно стоит отметить: `si_battery_cells` оказался не только отображаемым.
Миграция тюна 1.1 (`ui.qml.in:1400-1406`) делит на него пороги `tiltback_hv`
и `tiltback_lv`, то есть неверное число ячеек попадало бы в записываемую
конфигурацию и меняло пороги напряжения.

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
