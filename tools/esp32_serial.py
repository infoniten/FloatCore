#!/usr/bin/env python3
"""Снять serial-лог с платы FloatCore: сброс, ожидание, необязательные команды.

Только чтение и read-only команды консоли: ничего, что могло бы дать выход на
мотор, здесь нет (ТЗ v0.5 §21).

    python3 tools/esp32_serial.py --seconds 70 --out build/debug/boot.log
    python3 tools/esp32_serial.py --seconds 5 --send status --send safety
"""
import argparse
import glob
import os
import subprocess
import sys
import time

try:
    import serial
except ModuleNotFoundError:
    # pyserial в системном python3 обычно отсутствует. Он гарантированно есть
    # в окружении ESP-IDF (там живёт esptool), поэтому перезапускаемся в нём,
    # а не заставляем пользователя вручную искать интерпретатор.
    # Защита от бесконечного перезапуска — переменная окружения, а не сравнение
    # путей: venv ESP-IDF ссылается на тот же бинарник python, что и системный,
    # и realpath у них совпадает.
    _candidates = sorted(glob.glob(os.path.expanduser("~/.espressif/python_env/*/bin/python")))
    if not os.environ.get("FLOATCORE_SERIAL_REEXEC"):
        for _py in _candidates:
            _probe = subprocess.run([_py, "-c", "import serial"], capture_output=True)
            if _probe.returncode == 0:
                os.environ["FLOATCORE_SERIAL_REEXEC"] = "1"
                os.execv(_py, [_py, os.path.abspath(__file__)] + sys.argv[1:])
    sys.exit(
        "Нужен модуль pyserial. Он есть в окружении ESP-IDF:\n"
        "    . ~/esp/esp-idf/export.sh && python3 tools/esp32_serial.py ...\n"
        "либо поставьте его: python3 -m pip install --user pyserial"
    )


def capture(port, baud, seconds, reset, sends, out):
    p = serial.Serial(port, baud, timeout=0.2)
    if reset:
        # Схема автосброса на CH340: RTS -> EN, DTR -> IO0. Дёргаем только EN.
        p.setDTR(False)
        p.setRTS(True)
        time.sleep(0.1)
        p.setRTS(False)
    buf = b""
    t0 = time.time()
    sent = 0
    while time.time() - t0 < seconds:
        buf += p.read(4096)
        # Команды отправляем после того, как прошивка отрисовала приглашение.
        if sent < len(sends) and (b"floatcore> " in buf or time.time() - t0 > 4):
            time.sleep(0.3)
            p.write((sends[sent] + "\r\n").encode())
            p.flush()
            sent += 1
            time.sleep(1.0)
    p.close()
    text = buf.decode("utf-8", "replace")
    if out:
        open(out, "w").write(text)
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-3120")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--no-reset", action="store_true")
    ap.add_argument("--send", action="append", default=[])
    ap.add_argument("--out")
    a = ap.parse_args()

    text = capture(a.port, a.baud, a.seconds, not a.no_reset, a.send, a.out)
    sys.stdout.write(text)


if __name__ == "__main__":
    main()
