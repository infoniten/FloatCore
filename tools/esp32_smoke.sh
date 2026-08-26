#!/usr/bin/env bash
# Воспроизводимый прогон FloatCore на живой ESP32 (ТЗ v0.5 §21).
#
#   . ~/esp/esp-idf/export.sh
#   tools/esp32_smoke.sh [PORT]
#
# Что делает: проверяет toolchain, собирает прошивку, доказывает по таблице
# символов отсутствие CAN-передатчика, прошивает, снимает ограниченный boot-лог
# и проверяет маркеры.
#
# Опасной автоматизации здесь нет: ни одной команды мотору не отправляется и
# отправить нельзя — в прошивке нет кода передачи (см. docs/esp32_safety.md).
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ESPDIR="$ROOT/platform/esp32"
PORT="${1:-${ESPPORT:-/dev/cu.usbserial-3120}}"
LOGDIR="$ROOT/build/debug"
BOOTLOG="$LOGDIR/esp32_smoke_boot.log"
SECONDS_CAPTURE="${SECONDS_CAPTURE:-14}"

pass() { printf "      \033[32mPASS\033[0m %s\n" "$1"; }
fail() { printf "      \033[31mFAIL\033[0m %s\n" "$1"; FAILED=1; }
info() { printf "      \033[90m·\033[0m %s\n" "$1"; }
FAILED=0

mkdir -p "$LOGDIR"

echo ""
echo "1. toolchain"
if ! command -v idf.py >/dev/null 2>&1; then
    fail "idf.py не найден — выполните . ~/esp/esp-idf/export.sh"
    exit 1
fi
pass "idf.py: $(idf.py --version 2>&1 | tail -1)"
command -v cmake >/dev/null 2>&1 && pass "cmake: $(cmake --version | head -1)" || fail "нет cmake"
command -v ninja >/dev/null 2>&1 && pass "ninja: $(ninja --version)" || fail "нет ninja"
pass "python: $(python3 --version)"

echo ""
echo "2. плата"
if [ ! -e "$PORT" ]; then
    fail "нет устройства $PORT"
    exit 1
fi
pass "serial: $PORT"
IDF_PY="$(ls "$HOME"/.espressif/python_env/*/bin/python 2>/dev/null | head -1)"
[ -z "$IDF_PY" ] && IDF_PY="$(command -v python3)"
# esptool 5.x: chip-id, esptool 4.x (идёт с ESP-IDF 5.5): chip_id
CHIP="$("$IDF_PY" -m esptool --port "$PORT" chip-id 2>/dev/null | grep -m1 -E 'Chip type:|Chip is' | sed 's/.*: *//;s/^Chip is //')"
if [ -z "$CHIP" ]; then
    CHIP="$("$IDF_PY" -m esptool --port "$PORT" chip_id 2>/dev/null | grep -m1 -E 'Chip type:|Chip is' | sed 's/.*: *//;s/^Chip is //')"
fi
if [ -n "$CHIP" ]; then
    pass "чип: $CHIP"
else
    info "esptool не смог определить чип (плата может быть занята) — продолжаю"
fi

echo ""
echo "3. сборка"
if (cd "$ESPDIR" && idf.py build >"$LOGDIR/esp32_smoke_build.log" 2>&1); then
    SIZE=$(grep -o 'floatcore_esp32.bin binary size 0x[0-9a-f]*' "$LOGDIR/esp32_smoke_build.log" | tail -1 | grep -o '0x[0-9a-f]*')
    pass "прошивка собрана, образ $SIZE Б"
else
    fail "сборка провалилась, см. $LOGDIR/esp32_smoke_build.log"
    exit 1
fi

echo ""
echo "4. безопасность по таблице символов (ТЗ v0.6A §15)"
ELF="$ESPDIR/build/floatcore_esp32.elf"
NM="$(ls "$HOME"/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin/xtensa-esp32-elf-nm 2>/dev/null | head -1)"
if [ -n "$NM" ] && [ -f "$ELF" ]; then
    SYMS="$("$NM" "$ELF")"

    # Ни одного символа передатчика CAN. Проверяется подстрокой, а не точным
    # именем: любой драйвер TWAI неизбежно принесёт с собой символы twai_*.
    check_absent() {
        local pattern="$1" what="$2"
        local n
        n=$(echo "$SYMS" | grep -ci "$pattern" || true)
        if [ "$n" -eq 0 ]; then
            pass "$what: символов нет"
        else
            fail "$what: найдено $n символов — stop condition"
            echo "$SYMS" | grep -i "$pattern" | sed 's/^/            /'
        fi
    }
    check_absent 'twai'                    "CAN/TWAI TX"
    check_absent 'fc_motor_gate_set_backend' "регистрация backend-а мотора"
    check_absent 'manual_set_current'      "ручная подача тока"
    check_absent 'manual_set_duty'         "ручная подача duty"
    check_absent 'motor_can_send'          "отправка команды мотору по CAN"
    check_absent 'real_motor_backend'      "реальный backend мотора"
    check_absent 'safety_bypass'           "обход политики безопасности"

    # А это, наоборот, обязано присутствовать.
    check_present() {
        local pattern="$1" what="$2"
        if echo "$SYMS" | grep -q "$pattern"; then
            pass "$what: на месте"
        else
            fail "$what: отсутствует — собран не тот код"
        fi
    }
    check_present ' [tT] refloat_init'          "настоящий Refloat"
    check_present ' [tT] fc_motor_gate_request' "единая точка выхода на мотор"
    check_present ' [tT] fc_supervisor_poll'    "Safety Supervisor"
    check_present ' [tT] icm20948_read'         "драйвер ICM-20948"

    # Профиль сборки должен быть виден в самом двоичном файле.
    if strings "$ELF" | grep -q "LAB_SAFE"; then
        pass "профиль LAB_SAFE зашит в прошивку"
    else
        fail "в прошивке нет строки профиля LAB_SAFE"
    fi
else
    info "nm недоступен, проверка символов пропущена"
fi

echo ""
echo "5. прошивка"
if (cd "$ESPDIR" && idf.py -p "$PORT" -b 460800 flash >"$LOGDIR/esp32_smoke_flash.log" 2>&1); then
    pass "записана во flash"
else
    fail "flash провалился, см. $LOGDIR/esp32_smoke_flash.log"
    exit 1
fi

echo ""
echo "6. загрузка (${SECONDS_CAPTURE} с)"
PY_SERIAL="$(command -v python3)"
if ! "$PY_SERIAL" -c "import serial" 2>/dev/null; then
    info "модуль pyserial недоступен для $PY_SERIAL"
    for CAND in "$HOME"/.espressif/python_env/*/bin/python; do
        if "$CAND" -c "import serial" 2>/dev/null; then PY_SERIAL="$CAND"; break; fi
    done
fi
"$PY_SERIAL" "$ROOT/tools/esp32_serial.py" --port "$PORT" --seconds "$SECONDS_CAPTURE" \
    --send safety --send supervisor --send imu --out "$BOOTLOG" >/dev/null 2>&1 || true

if [ ! -s "$BOOTLOG" ]; then
    fail "boot-лог пуст"
    exit 1
fi
pass "лог снят: $BOOTLOG ($(wc -l < "$BOOTLOG" | tr -d ' ') строк)"

echo ""
echo "7. маркеры в логе"
check_marker() {
    if grep -q "$1" "$BOOTLOG"; then pass "$2"; else fail "$2 (нет '$1')"; fi
}
check_marker "FloatCore ESP32"          "баннер FloatCore напечатан"
check_marker "chip:"                    "данные чипа получены из runtime API"
check_marker "ESP-IDF:"                 "версия ESP-IDF в логе"
check_marker "Initializing Refloat"     "настоящий Refloat начал инициализацию"
check_marker "config registered"        "конфигурация Refloat зарегистрирована"
check_marker "DISENGAGED"               "footpad остаётся disengaged"
check_marker "profile:     LAB_SAFE"    "прошивка собрана в лабораторном профиле"
check_marker "backend none (blocked)"   "backend мотора отсутствует"
check_marker "supervisor: DISARMED"     "supervisor поднялся в DISARMED"
check_marker "ICM-20948"                "физический IMU обнаружен"

# Счётчик физически отправленных команд обязан быть нулевым.
if grep -qE "sent=0" "$BOOTLOG"; then
    pass "physically_sent == 0 в отчёте безопасности"
else
    fail "в отчёте безопасности нет подтверждения sent=0"
fi

if grep -qE "Guru Meditation|rst:0x.*PANIC|StoreProhibited" "$BOOTLOG"; then
    fail "в логе есть panic"
else
    pass "panic при загрузке не возникал"
fi
if grep -q "Task watchdog got triggered" "$BOOTLOG"; then
    fail "TWDT сработал во время обычной загрузки"
else
    pass "watchdog не срабатывал"
fi

echo ""
if [ "$FAILED" -eq 0 ]; then
    printf "\033[32msmoke пройден\033[0m\n\n"
    exit 0
fi
printf "\033[31msmoke провален\033[0m\n\n"
exit 1
