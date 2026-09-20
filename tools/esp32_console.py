#!/usr/bin/env python3
"""Снять консоль FloatCore с окном ТИШИНЫ посередине.

Чем отличается от esp32_serial.py: тот отправляет все команды подряд в начале,
а здесь есть три фазы — команды до, окно без единого байта в UART, команды
после. Это нужно потому, что печать в консоль сама искажает то, что мы мерим:
на 115200 бод сто символов это около 9 мс, а период контура 2 мс.

Все команды read-only либо меняют только нашу конфигурацию. Ничего, что могло
бы дать выход на мотор, здесь нет и быть не может: в прошивке нет кода
передачи моторных команд.

    python3 tools/esp32_console.py --pre can-reset --window 90 --post can
    python3 tools/esp32_console.py --reset --window 310 --post timing --post sched
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
    # pyserial есть в окружении ESP-IDF; перезапускаемся в нём, а не заставляем
    # искать интерпретатор вручную. Защита от зацикливания — переменная
    # окружения, а не сравнение путей: venv IDF ссылается на тот же бинарник.
    _cand = sorted(glob.glob(os.path.expanduser("~/.espressif/python_env/*/bin/python")))
    if not os.environ.get("FLOATCORE_CONSOLE_REEXEC"):
        for _py in _cand:
            if subprocess.run([_py, "-c", "import serial"], capture_output=True).returncode == 0:
                os.environ["FLOATCORE_CONSOLE_REEXEC"] = "1"
                os.execv(_py, [_py, os.path.abspath(__file__)] + sys.argv[1:])
    sys.exit("нужен pyserial: . ~/esp/esp-idf/export.sh, либо pip install --user pyserial")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-3120")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--pre", action="append", default=[], help="команда до окна тишины")
    ap.add_argument("--post", action="append", default=[], help="команда после окна")
    ap.add_argument("--window", type=float, default=0.0, help="секунды тишины")
    ap.add_argument("--reset", action="store_true", help="сбросить плату перед началом")
    ap.add_argument("--settle", type=float, default=1.2, help="пауза после каждой команды")
    ap.add_argument("--post-settle", type=float, default=2.5)
    ap.add_argument("--out")
    a = ap.parse_args()

    p = serial.Serial(a.port, a.baud, timeout=0.2)
    buf = b""

    if a.reset:
        # Автосброс CH340: RTS -> EN. Дёргаем только EN, DTR не трогаем, иначе
        # плата уйдёт в загрузчик.
        p.setDTR(False)
        p.setRTS(True)
        time.sleep(0.1)
        p.setRTS(False)
        time.sleep(2.0)

    def send(cmd, settle):
        nonlocal buf
        p.write((cmd + "\r\n").encode())
        p.flush()
        t = time.time()
        while time.time() - t < settle:
            buf += p.read(4096)

    p.write(b"\r\n")
    p.flush()
    time.sleep(0.4)
    buf += p.read(4096)

    for c in a.pre:
        send(c, a.settle)

    buf += p.read(4096)
    mark = len(buf)
    t0 = time.time()
    while time.time() - t0 < a.window:
        buf += p.read(4096)  # только читаем, ничего не пишем
    actual = time.time() - t0
    silent = buf[mark:]

    for c in a.post:
        send(c, a.post_settle)
    p.close()

    text = buf.decode("utf-8", "replace")
    head = "### окно тишины %.2f с, байт в UART за это время: %d\n" % (actual, len(silent))
    if silent.strip():
        head += "### ВНИМАНИЕ: незапрошенный вывод внутри окна\n"
        head += silent.decode("utf-8", "replace") + "\n"
    if a.out:
        open(a.out, "w").write(head + text)
    sys.stdout.write(head + text)


if __name__ == "__main__":
    main()
