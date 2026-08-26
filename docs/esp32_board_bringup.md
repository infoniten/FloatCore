# Bring-up конкретной платы ESP32 (этап v0.5)

Всё в этом документе — результат работы с **физически подключённой платой**,
а не расчёт и не предположение. Логи, на которые ссылается текст, лежат в
`build/debug/` и снимались в ходе описанных прогонов.

---

## 1. Что за железо (ТЗ §0)

Ничего не принималось по названию платы: чип определён `esptool chip-id`,
flash — `esptool flash-id`, eFuse — `espefuse summary`, USB — `ioreg`.

| Параметр | Значение | Чем определено |
|---|---|---|
| Чип | **ESP32-D0WD-V3** | `esptool chip-id` → `Chip type: ESP32-D0WD-V3` |
| Ревизия | **v3.1** (WAFER_VERSION_MAJOR = 3) | `esptool`, подтверждено `espefuse summary` |
| Ядра | 2 (Xtensa LX6) + LP core | `esptool` features; runtime `esp_chip_info()` → 2 core(s) |
| Тактовая | 240 МГц max, работает на **160 МГц** (умолчание ESP-IDF) | `rtc_clk_cpu_freq_get_config()` в баннере |
| Кварц | 40 МГц | `esptool`, подтверждено runtime |
| Flash | **4 МБ**, производитель `0xa1`, device `0x4016`, 3.3 В по strapping | `esptool flash-id` |
| MAC | `58:2a:bd:61:5a:d4` | `esptool`, подтверждено `esp_read_mac()` в баннере |
| Шифрование flash | выключено (`FLASH_CRYPT_CNT = 0`) | `espefuse summary` |
| Secure boot | выключен (`ABS_DONE_0/1 = False`) | `espefuse summary` |
| USB | **мост CH340**, VID `0x1A86`, PID `0x7523`, bcdDevice `0x0264` | `ioreg -p IOUSB -l` |
| Native USB | **нет** — только UART через мост | имя устройства `usbserial-*`, не `usbmodem-*` |
| Serial device | `/dev/cu.usbserial-3120` | `ls /dev/cu.*` |
| Reset / bootloader | автосброс по DTR/RTS работает | esptool сам вошёл в загрузчик и сделал `Hard resetting via RTS pin` |

Это **обычная ESP32**, не S3 и не C3. Все решения ниже приняты уже после
этого факта.

### Что было на плате до нас

Перед первой прошивкой снят полный образ flash:
`build/debug/esp32_flash_backup_before_floatcore.bin` (4 194 304 Б).

Таблица разделов в образе:

```
phy_init      data phy   0x00f000  0x001000
otadata       data ota   0x010000  0x002000
nvs           data nvs   0x012000  0x00e000
at_customize  0x40       0x020000  0x0e0000
ota_0         app  ota_0 0x100000  0x180000
ota_1         app  ota_1 0x280000  0x180000
```

Раздел `at_customize` и пара OTA — характерная разметка прошивки **ESP-AT**.
Заводская прошивка перезаписана; если понадобится вернуть плату в исходное
состояние, образ есть.

---

## 2. Toolchain на Mac mini (ТЗ §1)

| Параметр | Значение |
|---|---|
| macOS | 26.6 (build 25G72) |
| Архитектура | **Apple Silicon**, `arm64` (Darwin 25.6.0, T6041) |
| ESP-IDF | **v5.5.5**, `~/esp/esp-idf` (git tag, `--depth 1 --recursive`) |
| Компилятор | `xtensa-esp-elf-gcc (crosstool-NG esp-14.2.0_20260121) 14.2.0` |
| Python | 3.13.3 (система) + venv IDF `idf5.5_py3.13_env` |
| cmake / ninja | 4.4.3 / 1.13.2 (Homebrew) |
| Target | `esp32` |

Замечание: `install.sh` на macOS **не ставит cmake и ninja** — их пришлось
поставить через Homebrew, иначе `idf.py` падает с «cmake must be available on
the PATH». Ни Arduino, ни PlatformIO не используются: сборка идёт официальным
ESP-IDF.

Активация окружения нужна в каждой новой сессии оболочки:

```bash
. ~/esp/esp-idf/export.sh
```

---

## 3. Как устроен таргет (ТЗ §3)

Второго FloatCore не создано. Общие слои используются те же самые:

```
platform/esp32/
├── CMakeLists.txt          проект ESP-IDF, генерация conf/ тем же gen_conf.py
├── partitions.csv          nvs 24 КБ + factory 3 МБ
├── sdkconfig.defaults      tick 1000 Гц, TWDT, panic-halt, радио выключено
├── components/refloat/     ← неизменённый refloat-upstream/src + compat/*
└── main/                   ← только платформа: fc_*.c
```

Чтобы это стало возможным, на этом этапе три вещи переехали из host-специфичных
каталогов в общие:

| Было | Стало | Почему |
|---|---|---|
| `tests/host/mock/vesc_c_if.h` | `compat/vesc_api/vesc_c_if.h` | shim один на обе платформы, иначе Refloat собирался бы по-разному |
| `tests/host/mock/st_types.h` | `compat/vesc_api/st_types.h` | то же |
| `tests/host/harness/refloat_facade.*` | `compat/refloat_glue/refloat_facade.*` | путь запуска Refloat общий: host и плата вызывают один и тот же `refloat_init()` |

Глобальный указатель переименован `mock_vesc_if` → `floatcore_vesc_if`:
и host-mock, и ESP32 обязаны предоставлять один символ.

**Разделение компонентов не косметическое.** `refloat-upstream/src/time.h`
объявляет `typedef uint32_t time_t`. Попади этот каталог в include-пути
платформенных файлов — заголовки ESP-IDF, включающие `<time.h>`, подхватили бы
его. Поэтому Refloat живёт в отдельном компоненте со своими include-путями,
ровно как в host-Makefile (`REFLOAT_INC` против `PLAIN_CFLAGS`).

---

## 4. Что реально запущено на плате

Полный лог первой загрузки: `build/debug/esp32_boot_full.log`.

```
================ FloatCore ESP32 ================
chip:        ESP32 rev v3.1, 2 core(s)
features:    WiFi BT BLE
cpu freq:    160 MHz (xtal 40 MHz)
flash:       4194304 B (4 MB)
mac:         58:2a:bd:61:5a:d4
heap:        260916 B free, 260852 B min
ESP-IDF:     v5.5.5
build:       Aug 26 2026 21:14:26 (проект floatcore_esp32, версия 446a096-dirty)
reset:       POWERON (подача питания)
boot #:      2 (счётчик в NVS — доказательство persistence)
motor:       blocked
can:         unavailable (TWAI не инициализирован, трансивер не подключён)
imu:         mock (500 Hz, покой)
adc:         mock (0.00 V, footpad disengaged)
================================================

[floatcore] Refloat initialization started
I (401) refloat: [refloat] Initializing Refloat 1.3.0 (47a2c5ce)
I (423) vesc_if: spawn: Refloat Main, стек 12288 Б (запрошено 1536), ядро 1, приоритет 12
I (432) vesc_if: spawn: Refloat Aux,  стек 12288 Б (запрошено 1536), ядро 1, приоритет 12
I (442) vesc_if: config registered: get=0x400d9bd0 set=0x400dc584 xml=0x400d96bc
[floatcore] Refloat initialized: config registered
[floatcore] footpad: NONE, adc 0.00/0.00 V, пороги 2.00/2.00 V -> DISENGAGED (ok)
[floatcore] refloat state: READY
```

Всё в баннере берётся из runtime API (`esp_chip_info`, `esp_flash_get_size`,
`rtc_clk_cpu_freq_get_config`, `esp_read_mac`, `esp_get_free_heap_size`,
`esp_get_idf_version`, `esp_app_get_description`, `esp_reset_reason`), ничего
не захардкожено.

Версия Refloat — `1.3.0 (47a2c5ce)`: это тот же самый upstream, что собирается
на host, а не упрощённая копия.

---

## 5. Контур управления (ТЗ §10)

**Частота взята из upstream, а не назначена:**
`refloat-upstream/src/main.c:61` → `#define MAIN_THREAD_FREQ 500`,
`vesc_pkg_lib/vesc_c_if.h:682` → `SYSTEM_TICK_RATE_HZ 10000`,
отсюда `main_loop_ticks = 20` тиков = **2000 мкс**. Главный поток сам
компенсирует время итерации (`main.c:1058-1059`), то есть 500 Гц — это
целевая частота, а не «спать 2 мс».

Контур баланса задаётся не главным потоком, а частотой семплов IMU
(`imu_ref_callback`). На VESC это 832 Гц аппаратно; у нас mock на 500 Гц —
значение из `docs/esp32_architecture.md` §2, отдаётся Refloat через
`get_cfg_int(CFG_PARAM_IMU_sample_rate)`.

Измерение за 60 секунд (`build/debug/esp32_timing_60s.log`, монотонный
`esp_timer`, отметка в момент пробуждения задачи):

| Канал | Итераций | Средний период | min | max | «Опозданий» (> +20 %) |
|---|---|---|---|---|---|
| `control` (imu_ref_callback) | 30 562 | **2000.0 мкс** | 1228 | 2772 | 1691 (5.5 %) |
| `refloat_thd` | 31 302 | **2000.1 мкс** | 1209 | 3443 | 1735 (5.5 %) |
| `aux_thd` | 1 744 | 35 881 мкс | 33 914 | 36 034 | 0 |

Собственные счётчики Refloat (`frequency_tracker`) показывают ровно
`imu 500.0 Hz, main 500.0 Hz`.

Читать это надо так: **дрейфа нет** (средний период совпадает с номиналом до
0.005 %), но джиттер составляет примерно ±0.8 мс, то есть ±0.8 тика FreeRTOS.
Порог «опоздания» в 20 % (2400 мкс) при таком джиттере пересекается на 5.5 %
итераций. Для v0.5 это приемлемо — контур работает, частота стабильна, но
источник джиттера ещё не локализован. Наиболее вероятные кандидаты:
квантование `vTaskDelayUntil` тиком 1 мс, латентность tick-ISR и работа
`aux_thd` на 30 Гц (совпадает по порядку с числом выбросов). Профилировать это
надо на следующем этапе — ТЗ §23 прямо запрещает оптимизировать до измерений,
измерение теперь есть.

`aux_thd` идёт на 27.9 Гц вместо 30: `LEDS_REFRESH_RATE = 30` даёт
`sleep_us(33333)`, что при тике 1 мс округляется вверх. Это поведение самого
Refloat, а не ошибка платформы.

---

## 6. Память (ТЗ §13)

Из вывода сборки (`idf.py size`):

| Область | Занято | Всего | % |
|---|---|---|---|
| Flash `.text` | 160 198 Б | — | — |
| Flash `.rodata` | 50 520 Б | — | — |
| IRAM | 56 759 Б | 131 072 Б | 43.3 |
| DRAM (`.data` + `.bss`) | 44 956 Б | 180 736 Б | 24.9 |
| Образ прошивки | 305 760 Б (`0x4ac60`) | раздел 3 МБ | 10 |

Runtime, из логов той же 70-секундной сессии:

| Замер | Свободная куча | Минимум за всё время |
|---|---|---|
| сразу после boot | 260 916 Б | 260 852 Б |
| через 10 с | 226 092 Б | 216 492 Б |
| через 60 с | **226 092 Б** | **216 492 Б** |

Между 10 и 60 секундами куча **не изменилась ни на байт** — монотонной утечки
нет. Провал до 216 КБ приходится на инициализацию Refloat (один `malloc`
структуры `Data` плюс временные буферы конфигурации) и после неё не
повторяется.

Стек (high-water mark = минимум свободного за время жизни задачи; ESP-IDF
возвращает **байты**):

| Задача | Свободно минимум | Выделено |
|---|---|---|
| `fc_imu` (контур управления) | 2 020 Б | 4 096 Б |
| `Refloat Main` | 11 652 Б | 12 288 Б |
| `Refloat Aux` | 11 660 Б | 12 288 Б |

Refloat просит `spawn(..., 1536, ...)`. Запрошенных 1536 байт хватило бы, но
масштабирование ×8 оставлено как есть: запас дешёвый, а срыв стека на Xtensa
диагностируется плохо. Самая напряжённая задача — не потоки Refloat, а
`fc_imu`: именно в ней исполняется `imu_ref_callback`, то есть весь PID и
фильтры (использовано ~2 КБ из 4 КБ).

---

## 7. Watchdog (ТЗ §11)

TWDT включён (`CONFIG_ESP_TASK_WDT_EN`, таймаут 5 с). Задачи контура подписаны
явно: `fc_imu` — в самой задаче, `Refloat Main` и `Refloat Aux` — через
`esp_task_wdt_reset()` внутри `VESC_IF->sleep_us()`, то есть в точке, которую
главный цикл проходит ровно один раз за итерацию. Watchdog **не отключён и не
ослаблен**.

Проверено на плате командой `wdtest-confirm`, которая заставляет контур не
отмечаться 8 секунд (`build/debug/esp32_wdt_test.log`):

```
E (8428) task_wdt: Task watchdog got triggered. The following tasks/users did not reset the watchdog in time:
E (8428) task_wdt:  - fc_imu (CPU 1)
E (8428) task_wdt: Print CPU 1 backtrace
Backtrace: 0x40083BB6:0x3FFB8270 0x40082ADD:0x3FFB8290 ...
```

Срыв контура именно **обнаруживается и называется по имени задачи**. В отчёте
за 10 с этот же срыв виден как `max 8002000 us`.

`CONFIG_ESP_TASK_WDT_PANIC` намеренно оставлен выключенным: на v0.5 зависание
должно быть *видно в логе вместе с backtrace*, а не спрятано за мгновенной
перезагрузкой. Когда появится реальный выход на мотор, решение надо
пересмотреть — тогда безопаснее перезагрузиться (тяга снимется по watchdog
самого VESC).

---

## 8. Диагностика падений (ТЗ §14)

`CONFIG_ESP_SYSTEM_PANIC_PRINT_HALT=y` — panic печатает всё и **останавливает
CPU**, автоматической тихой перезагрузки нет. Проверено командой
`crashtest-confirm` (`build/debug/esp32_panic_test.log`):

```
Guru Meditation Error: Core  0 panic'ed (StoreProhibited). Exception was unhandled.
Core  0 register dump:
PC      : 0x400d8f33  PS      : 0x00060130  A0 : 0x800d93c9  A1 : 0x3ffca0f0
...
EXCVADDR: 0x00000000  LBEG : 0x400014fd  LEND : 0x4000150d  LCOUNT : 0xffffffe5
Backtrace: 0x400d8f30:0x3ffca0f0 0x400d93c6:0x3ffca110 0x40085c5d:0x3ffca170
ELF file SHA256: ce2f13f46
CPU halted.
```

Есть причина, регистры, backtrace, SHA прошивки. Backtrace разворачивается
`idf.py monitor` автоматически.

---

## 9. Постоянное хранилище (ТЗ §12)

Реализация: `platform/esp32/main/fc_storage_nvs.c`. Refloat по-прежнему знает
только контракт VESC `read_eeprom_var`/`store_eeprom_var`; перевод в NVS — дело
платформы, из Refloat в NVS никто не пишет.

Модель: 128 слов в ОЗУ, на носитель уходит один blob (`nvs_set_blob` +
`nvs_commit`) через 150 мс после последней записи. 80 отдельных коммитов на
одно сохранение конфигурации износили бы flash впустую. Незаписанное слово
читается как `0xFFFFFFFF` — так же, как стёртая ячейка EEPROM на VESC, и
Refloat на первом старте корректно уходит на значения по умолчанию.

**Проверены оба уровня, и они разные:**

| Уровень | Что проверялось | Результат |
|---|---|---|
| Платформенный round-trip | счётчик загрузок в слове 100 (вне диапазона Refloat 0…79) | инкрементируется через сброс: boot #11 → #12 → #13 → #14 → #15 |
| Конфигурация Refloat | штатный путь Refloat: `set_cfg()` → `write_cfg_to_eeprom()` | значение пережило аппаратный сброс |

Второй уровень (`build/debug/esp32_persist_write.log` → `esp32_persist_verify.log`):

```
persist: leds.status.brightness_headlights_off 0.500 -> 0.600
I (4876) refloat: [refloat] Config written: 282B
persist: set_cfg вернул true
I (4884) storage: NVS: сохранено 128 слов (коммит #2)
--- аппаратный сброс ---
boot #: 7
[floatcore] Refloat initialized: config registered
[floatcore] config test value = 0.600
```

До записи каждый старт печатал `Error: Failed to deserialize config, using
defaults` — после записи это сообщение **исчезло**: Refloat читает свою
конфигурацию из NVS. Это и есть доказательство, что работает не только
платформенный контракт, но и полный путь конфигурации.

Тестовый параметр выбран нарочно косметическим — яркость фар в статус-баре
LED. Он не участвует ни в одном вычислении контура, а лента к плате не
подключена.

**Побочная находка, важная для будущих этапов.** Запись конфигурации во flash
стопорит контур: в отчёте, снятом сразу после `persist`, виден
`max 13904 us` — почти 14 мс без итерации при периоде 2 мс. Причина штатная
(операция с SPI-flash отключает кэш). Пока мотор заблокирован, это безвредно,
но запись конфигурации во время езды недопустима — на VESC она и происходит
только по явной команде сохранения.

---

## 10. Циклы загрузки (ТЗ §16, §17)

| Сценарий | Как выполнен | Результат |
|---|---|---|
| Аппаратный сброс ×3 | импульс на EN через RTS моста CH340 — электрически то же, что кнопка Reset | `build/debug/esp32_reset_cycle_{1,2,3}.log`: три подряд загрузки, каждая доходит до `Refloat initialized` / `READY` / `DISENGAGED`, boot #11 → #12 → #13 |
| Программный сброс | команда `restart` → `esp_restart()` | `build/debug/esp32_sw_restart.log`: `reset: SW (esp_restart)`, boot #15, снова `READY` |
| Отключение и подключение USB | человек выдернул и вставил кабель, затем `--send status --no-reset` | `build/debug/esp32_usb_power_cycle.log`: `reset reason POWERON`, `boot # 23` против 22 до снятия питания, `refloat state READY`, `footpad NONE`, `motor backend blocked` |

Замечание про причину сброса: аппаратный сброс по EN на ESP32 отображается как
`POWERON`, а не `EXT` — это свойство чипа (сброс по EN эквивалентен подаче
питания), а не ошибка. Программный сброс отличается корректно: `SW`.

После каждого сценария прошивка возвращается в одно и то же безопасное
состояние: footpad `NONE`, мотор `blocked`, CAN отсутствует.

---

## 11. Итоговая таблица

| Проверка | Результат | Доказательство |
|---|---|---|
| Chip detected | **PASS** | `esptool chip-id`: ESP32-D0WD-V3 rev v3.1; подтверждено runtime-баннером |
| Toolchain | **PASS** | ESP-IDF v5.5.5, gcc 14.2.0, arm64 macOS 26.6 |
| Build | **PASS** | `0x4ac60` = 305 760 Б, ноль предупреждений компилятора |
| Flash | **PASS** | `idf.py flash`, «Hash of data verified» на всех трёх образах |
| Boot | **PASS** | баннер + `READY` в 5 независимых прогонах |
| Refloat init | **PASS** | `Initializing Refloat 1.3.0 (47a2c5ce)`, `config registered`, две задачи на ядре 1 |
| Config layer | **PASS** | `conf_custom_add_config` вызван, XML/get/set зарегистрированы |
| Mock IMU contract | **PASS** | покой: acc = (0,0,1) g, gyro = 0 рад/с, quat = (1,0,0,0); Refloat показывает pitch/roll = 0.000° |
| Footpad disengaged | **PASS** | `FS_NONE` при 0.00 В против порогов 2.00/2.00 В; 29 host-тестов на границы |
| Control timing | **PASS** | 30 562 итерации за 60 с, средний период 2000.0 мкс = 500.0 Гц; джиттер ±0.8 мс задокументирован |
| Motor blocking | **PASS** | 30 587 заблокированных `set_current` за 60 с, ноль отправленных |
| CAN TX absent | **PASS** | ноль символов `twai*` в `.elf` (проверяется `tools/esp32_smoke.sh`) |
| Watchdog | **PASS** | TWDT назвал зависшую задачу `fc_imu (CPU 1)` и напечатал backtrace |
| Crash diagnostics | **PASS** | Guru Meditation + регистры + backtrace + `CPU halted` |
| Persistence | **PASS** | оба уровня: счётчик загрузок и конфигурация Refloat пережили сброс |
| Heap stability | **PASS** | 226 092 Б на 10-й и на 60-й секунде — байт в байт |
| 3 boot/reset | **PASS** | три аппаратных цикла + один программный |
| Host regression | **PASS** | protocol 120, host 11 сценариев, esp32 29, integration 43 — всё зелёное |
| USB power cycle | **PASS** | снятие питания: `boot #` 22 → 23, `POWERON`, `READY`, мотор заблокирован |

---

## 12. Отключение питания: как это проверялось

**Единственный пункт, который нельзя выполнить программно** (ТЗ §17): сброс по
EN моделирует кнопку Reset, но не снятие питания с платы, поэтому кабель
выдёргивал человек. Результат — в таблице §10, лог
`build/debug/esp32_usb_power_cycle.log`.

Повторить так:

```bash
# выдернуть USB, вставить обратно, затем
python3 tools/esp32_serial.py --seconds 12 --send status --no-reset
```

Ожидается в ответе на `status`: `reset reason POWERON`, `boot #` на единицу
больше предыдущего, `refloat state READY`, `footpad NONE`, `motor backend
blocked`. Фактически полученное:

```
uptime            15.0 s
reset reason      POWERON (подача питания)
boot #            23 (счётчик в NVS)
refloat state     READY
stop condition    NONE
footpad           NONE (adc 0.00 / 0.00 V)
imu / main freq   500.0 / 500.0 Hz (по счётчикам Refloat)
motor backend     blocked
can backend       unavailable (TWAI не инициализирован, трансивер не подключён)
```

То есть после полного снятия питания плата вернулась в то же безопасное
состояние, а счётчик загрузок в NVS пережил обесточивание.

Почему именно `status`, а не просто лог: баннер с причиной сброса печатается
один раз при старте и к моменту запуска скрипта уже вытеснен из буфера UART.
Поэтому причина сброса и счётчик загрузок продублированы в команду `status` —
проверку можно сделать в любой момент после загрузки.

`pyserial` в системном `python3` на macOS обычно отсутствует; скрипт сам
перезапускается в окружении ESP-IDF, где модуль есть, так что отдельная
установка не нужна.

---

## 13. Как повторить

```bash
. ~/esp/esp-idf/export.sh

# полный прогон: toolchain → сборка → проверка символов → flash → boot → маркеры
tools/esp32_smoke.sh /dev/cu.usbserial-3120

# отдельно
make test                 # host + протокол + тесты платформы ESP32 (без платы)
make esp32                # только сборка прошивки
python3 tools/esp32_serial.py --seconds 70 --out build/debug/timing.log
python3 tools/esp32_serial.py --seconds 15 --send status --send safety --send timing
python3 tools/esp32_serial.py --seconds 12 --send status --no-reset   # без сброса платы
```

Консоль на плате (read-only, команд мотору нет):
`status`, `tasks`, `timing`, `heap`, `config`, `safety`, `persist`, `restart`,
плюс диагностические `wdtest-confirm` и `crashtest-confirm`.
