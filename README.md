# ESP32 Refloat Compatibility Controller

Прошивка для ESP32-WROOM-32, предоставляющая **неизменённому** Refloat окружение, совместимое
с прошивкой VESC: IMU, футпады, время, потоки, хранилище и «один логический мотор», за которым
стоят два физических VESC (Flipsky Mini Dual FSESC 4.20) на шине CAN.

```
VESC Tool / приложение  ──BLE/Wi-Fi──►  ESP32
                                         ├─ VESC compatibility layer
                                         ├─ Refloat (upstream, без изменений)
                                         ├─ IMU · футпады · LED
                                         └─ Motor abstraction ──CAN──► VESC A + VESC B
```

## Статус: Этап 0.5 — первый запуск на физической ESP32

Реальный вывод на моторы не активирован. **Неизменённый Refloat 1.3.0 запущен на
живой плате ESP32-D0WD-V3**: контур управления идёт 500 Гц, команды мотору
блокируются, CAN отсутствует. Подробности — в
[docs/esp32_board_bringup.md](docs/esp32_board_bringup.md) и
[docs/esp32_safety.md](docs/esp32_safety.md).

Готово:

* **прошивка ESP32 на официальном ESP-IDF v5.5.5**: те же исходники Refloat и те же
  общие слои, что и в host-сборке; отличается только реализация платформы;
* платформенный backend `VESC_IF` поверх FreeRTOS, mock-IMU, безопасный ADC,
  NVS вместо EEPROM, read-only диагностическая консоль;

* аудит зависимостей Refloat и контракт платформенного слоя;
* host-харнесс, в котором **неизменённые исходники Refloat собираются и исполняются
  на десктопе**;
* **мост к официальному VESC Tool**: оригинальный протокол VESC поверх TCP,
  идентификация FloatCore, телеметрия, отдача интерфейса Refloat и работа с
  конфигурацией;
* **Virtual mcConfig** — read-only проекция ограничений FloatCore в формат
  Motor Configuration, благодаря которой шкалы и пороги в Refloat UI
  соответствуют реальной системе, а не значениям по умолчанию VESC Tool.

Своего приложения, GUI и конфигуратора у проекта нет — интерфейсом служит
официальный VESC Tool.

### Главные выводы

1. **Refloat — нативный код на C, а не LispBM-программа.** Балансировочная логика (~7000 строк
   C99) компилируется в бинарник для Cortex-M4; Lisp-часть — 17 строк glue-кода.
   Следовательно, Strategy A в формулировке ТЗ невыполнима: переносить на LispBM нечего.
2. **Вся связь с платформой — одна структура указателей на функции** (`VESC_IF`, 74
   используемых членов), объявленная в SDK VESC, а не в исходниках Refloat.
3. Поэтому выбрана **Strategy B**: сборка исходников Refloat под ESP32 поверх собственной
   реализации `vesc_c_if`, **с нулевыми изменениями в `refloat-upstream/src/`**
   (исключается только STM32-специфичный LED-драйвер, заменяемый файлом с тем же интерфейсом).
4. **Команда мотору выдаётся из IMU-callback**, то есть на частоте IMU, а не на 500 Гц
   главного цикла. Это определяет требования к CAN (1 Мбит/с) и к таймингу.
5. Правило агрегации токов из ТЗ скорректировано: **среднее, а не сумма** — обоснование в
   [motor_semantics.md](docs/motor_semantics.md).
6. **Тезис проверен экспериментально:** `tests/host` собирает `refloat-upstream/src`
   поверх mock-реализации 74 функций `vesc_c_if` и прогоняет 10 сценариев. Ни одной
   правки в исходниках Refloat не потребовалось.

### Что показали сценарии

Refloat не защищает себя сам — это обязанность compat-слоя:

* при пропаже IMU он просто перестаёт выдавать команды (тягу снимает watchdog VESC);
* по `mc_get_fault` он **не останавливается**, только поднимает алерт;
* об отказе одного из двух ESC он не знает по конструкции;
* NaN в гироскопе необратимо разрушает фильтр ориентации, при этом выход молча
  вырождается в нулевой ток, а состояние остаётся `RUNNING`.

## Документы

| Файл | Содержание |
|---|---|
| [docs/refloat_vesc_api_dependencies.md](docs/refloat_vesc_api_dependencies.md) | **Основной результат.** Полный инвентарь 74 зависимостей от VESC/LispBM по категориям, с выделением realtime-пути и правилами агрегации двух VESC |
| [docs/refloat_architecture.md](docs/refloat_architecture.md) | Как Refloat устроен и работает на VESC |
| [docs/porting_strategy.md](docs/porting_strategy.md) | Аргументированный выбор Strategy A/B/C |
| [docs/esp32_architecture.md](docs/esp32_architecture.md) | Устройство прошивки: задачи, тайминг, motor abstraction, supervisor, план тестирования; §0 — что уже реализовано |
| [docs/esp32_board_bringup.md](docs/esp32_board_bringup.md) | **Результаты запуска на живой плате:** железо, toolchain, тайминг контура, память, watchdog, persistence, циклы загрузки |
| [docs/esp32_safety.md](docs/esp32_safety.md) | Почему прошивка v0.5 физически не может сформировать команду мотору или кадр CAN |
| [docs/qml_parse_error_606.md](docs/qml_parse_error_606.md) | Разбор ошибки `Expected token numeric literal` в QML Scripting VESC Tool 6.06 |
| [docs/risk_register.md](docs/risk_register.md) | 20 рисков с оценкой и мерами |
| [docs/vesc_if_contract.md](docs/vesc_if_contract.md) | **Контракт платформенного слоя:** все 74 функции по 8 категориям — сигнатура, места вызова, частота, участие в контуре, синхронизация, источник данных, требование по задержке, поведение при отказе |
| [docs/motor_semantics.md](docs/motor_semantics.md) | Семантика `mc_*` и правила агрегации A/B с обоснованием по каждому месту использования |
| [docs/can_bandwidth.md](docs/can_bandwidth.md) | Расчёт загрузки шины и задержек для 500 кбит/с и 1 Мбит/с, рекомендация по битрейту |
| [docs/threading_model.md](docs/threading_model.md) | Модель потоков, разделяемое состояние, гонки на ESP32, архитектура FreeRTOS, воспроизводимость сборки |
| [docs/vesc_protocol_inventory.md](docs/vesc_protocol_inventory.md) | Инвентарь протокола VESC: кадрирование, реализованные команды, форматы, порядок подключения VESC Tool |
| [docs/vesc_tool_compat_architecture.md](docs/vesc_tool_compat_architecture.md) | Архитектура моста: слои, модель потоков, единая модель конфигурации, идентификация, барьеры безопасности |
| [docs/refloat_qml_dependencies.md](docs/refloat_qml_dependencies.md) | Карта QML → backend → команда → обработчик; как Refloat UI попадает в VESC Tool без изменений |
| [docs/unsupported_commands.md](docs/unsupported_commands.md) | Что не реализовано, почему и как это проявляется |
| [docs/virtual_mcconfig.md](docs/virtual_mcconfig.md) | Полный аудит обращений Refloat QML к `mcConfig`, назначение и границы Virtual mcConfig |
| [docs/mcconfig_mapping.md](docs/mcconfig_mapping.md) | FloatCore Config → Virtual mcConfig → QML: таблица отображения и обоснование правил агрегации двух ESC |
| [docs/mcconfig_protocol.md](docs/mcconfig_protocol.md) | Обработчики протокола, сигнатура схемы, кодирование параметров, гарантия read-only |
| [docs/rt_app_inputs.md](docs/rt_app_inputs.md) | Страница RT App: форматы `COMM_GET_DECODED_PPM/_ADC/_CHUK`, что отдаёт FloatCore и почему это безопасно |

## Сборка и тесты

```bash
git submodule update --init
make                # host-тесты, протокольные тесты, FloatCore Host
make test           # протокольные тесты (120) + сценарии Refloat (11)
make test-all       # то же + интеграционный прогон по настоящему сокету (43 проверки)
```

Либо в контейнере: `docker build -t refloat-host . && docker run --rm refloat-host`.

Харнесс детерминирован: планировщик mock-платформы воспроизводит семантику одного ядра
с вытеснением, четыре прогона подряд дают побитово одинаковый вывод.

## Подключение VESC Tool

```bash
make host                      # или: build/floatcore_host --trace
```

В VESC Tool: **Connection → TCP → 127.0.0.1 : 65102 → Connect**.

Устройство представится как `FloatCore` (`HW_TYPE_CUSTOM_MODULE`), отдаст
интерфейс Refloat, его схему конфигурации и Virtual mcConfig.

Схема Motor Configuration зависит от версии вашего VESC Tool:

```bash
build/floatcore_host --mcconf-schema 6.06   # для VESC Tool 6.06
build/floatcore_host --mcconf-schema 7.01   # по умолчанию
build/floatcore_host --no-mcconf            # отключить, если версия другая
``` Управление моторами заблокировано
в диспетчере команд, за `LogicalMotor` стоит mock, драйвера CAN не существует —
вывод на моторы физически невозможен.

## Upstream

`refloat-upstream/` — git submodule [`lukash/refloat`](https://github.com/lukash/refloat),
зафиксирован на `47a2c5ce` (v1.3.0). **Не редактируется.** Все необходимые изменения
оформляются как патчи в `/patches` (сейчас пусто — патчи не требуются).

```
git submodule update --init
```

## Безопасность

Мотор не запускается, CAN-команды тока не отправляются, самобалансировка не реализуется —
до завершения архитектурного этапа и стендовых проверок.

В прошивке ESP32 это не декларация: в `.elf` нет ни одного символа TWAI, пины CAN не
конфигурируются, трансивер не подключён, а все пять функций запроса тяги ведут в
счётчик блокировок. Проверка отсутствия CAN-передатчика автоматизирована в
`tools/esp32_smoke.sh`. Полное обоснование — [docs/esp32_safety.md](docs/esp32_safety.md).
