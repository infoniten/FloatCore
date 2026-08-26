#!/usr/bin/env bash
# Интеграционный прогон: поднять FloatCore Host, прогнать симулятор VESC Tool,
# проверить, что конфигурация переживает и переподключение, и перезапуск процесса.
set -u

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN="$ROOT/build/floatcore_host"
SIM="$ROOT/tests/host_integration/vesc_tool_sim.py"
PORT="${PORT:-65199}"
EEPROM="$ROOT/build/integration_eeprom.bin"
LOG="$ROOT/build/integration_host.log"

if [ ! -x "$BIN" ]; then
    echo "Не собран $BIN — выполните 'make'" >&2
    exit 1
fi

rm -f "$EEPROM"
HOST_PID=""

cleanup() {
    [ -n "$HOST_PID" ] && kill "$HOST_PID" 2>/dev/null
    wait "$HOST_PID" 2>/dev/null
    HOST_PID=""
}
trap cleanup EXIT

start_host() {
    # Фикстура из ТЗ v0.4.1 §8
    "$BIN" --port "$PORT" --eeprom "$EEPROM" \
        --limits cells=10,imax=25,imin=-5,inmax=15,inmin=0,fet_start=80,fet_end=100 \
        --adc 1.65,1.20 \
        >>"$LOG" 2>&1 &
    HOST_PID=$!
    python3 - "$PORT" <<'PY'
import socket, sys, time
port = int(sys.argv[1])
for _ in range(100):
    try:
        socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
        sys.exit(0)
    except Exception:
        time.sleep(0.1)
sys.exit(1)
PY
}

: >"$LOG"

echo "[integration] запуск FloatCore Host на порту $PORT"
if ! start_host; then
    echo "[integration] хост не поднялся, лог:" >&2
    cat "$LOG" >&2
    exit 1
fi

python3 "$SIM" --port "$PORT"
RC=$?

if [ $RC -ne 0 ]; then
    echo "[integration] лог хоста:" >&2
    cat "$LOG" >&2
    exit $RC
fi

# Счётчики сессии: RT APP не должен оставлять неизвестных команд (ТЗ v0.4.2 DoD).
# Строка печатается при разрыве соединения, поэтому её ждём.
echo ""
echo "[integration] счётчики завершённой сессии"
for _ in $(seq 1 20); do
    grep -q "сессия завершена" "$LOG" && break
    sleep 0.1
done
SUMMARY="$(grep "сессия завершена" "$LOG" | tail -1)"
echo "      $SUMMARY"
if echo "$SUMMARY" | grep -q "неизвестных команд 0"; then
    echo -e "      \033[32mPASS\033[0m ни одной неизвестной команды за сессию"
else
    echo -e "      \033[31mFAIL\033[0m в сессии остались неизвестные команды" >&2
    exit 1
fi
if echo "$SUMMARY" | grep -qE "входов RT App ([1-9][0-9]*)"; then
    echo -e "      \033[32mPASS\033[0m входы RT App отданы"
else
    echo -e "      \033[31mFAIL\033[0m ни одного отчёта о входах RT App" >&2
    exit 1
fi

# Перезапуск процесса: конфигурация обязана сохраниться в файле EEPROM
echo ""
echo "[integration] перезапуск процесса — проверка постоянного хранилища"
cleanup
if ! start_host; then
    echo "[integration] хост не поднялся после перезапуска" >&2
    exit 1
fi

python3 - "$PORT" <<'PY'
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else ".", ""))
sys.path.insert(0, "tests/host_integration")
from vesc_tool_sim import VescLink, COMM_GET_CUSTOM_CONFIG, get_kp

port = int(sys.argv[1])
link = VescLink("127.0.0.1", port)
cfg = link.request(bytes([COMM_GET_CUSTOM_CONFIG, 0]), COMM_GET_CUSTOM_CONFIG)[2:]
kp = get_kp(cfg)
link.close()
expected = 12.5
ok = kp == expected
print(f"      {'\033[32mPASS\033[0m' if ok else '\033[31mFAIL\033[0m'} "
      f"kp = {kp} после перезапуска процесса (ожидалось {expected})")
sys.exit(0 if ok else 1)
PY
RC=$?

echo ""
if [ $RC -eq 0 ]; then
    echo -e "[integration] \033[32mинтеграционный сценарий пройден\033[0m"
else
    echo -e "[integration] \033[31mинтеграционный сценарий провален\033[0m" >&2
fi
exit $RC
