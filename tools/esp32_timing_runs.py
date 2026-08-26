#!/usr/bin/env python3
"""Серия сравнительных прогонов тайминга контура (ТЗ v0.6A §20).

Матрица построена так, чтобы вклад двух подозреваемых — обмена по I2C и
вывода в UART — можно было разделить, а не оценивать на глаз:

    A  baseline      IMU остановлен, консоль молчит
    B  real IMU      IMU читается, консоль молчит
    C  IMU + serial  IMU читается, команды идут раз в секунду
    D  serial only   IMU остановлен, команды идут раз в секунду
    E  NVS write     IMU читается, во время прогона выполняется запись

Каждый прогон начинается с аппаратного сброса и обнуления статистики, длится
одинаковое время и заканчивается снятием `timing`. Скрипт только читает
плату и пользуется командами, ни одна из которых не способна что-либо подать
на мотор.
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
    if not os.environ.get("FLOATCORE_SERIAL_REEXEC"):
        for _py in sorted(glob.glob(os.path.expanduser("~/.espressif/python_env/*/bin/python"))):
            if subprocess.run([_py, "-c", "import serial"], capture_output=True).returncode == 0:
                os.environ["FLOATCORE_SERIAL_REEXEC"] = "1"
                os.execv(_py, [_py, os.path.abspath(__file__)] + sys.argv[1:])
    sys.exit("Нужен pyserial: . ~/esp/esp-idf/export.sh")


RUNS = {
    "A": {"desc": "baseline: IMU остановлен, консоль молчит", "setup": ["imu-stop-confirm"], "chatter": False, "nvs": False},
    "B": {"desc": "real IMU активен, консоль молчит", "setup": [], "chatter": False, "nvs": False},
    "C": {"desc": "real IMU + команды раз в секунду", "setup": [], "chatter": True, "nvs": False},
    "D": {"desc": "IMU остановлен + команды раз в секунду", "setup": ["imu-stop-confirm"], "chatter": True, "nvs": False},
    "E": {"desc": "real IMU + запись конфигурации в NVS", "setup": [], "chatter": False, "nvs": True},
}


def send(port, cmd, settle=0.4):
    port.write((cmd + "\r\n").encode())
    port.flush()
    time.sleep(settle)
    return port.read(port.in_waiting or 1)


def run_one(port_name, key, seconds, out_dir):
    cfg = RUNS[key]
    print(f"\n=== прогон {key}: {cfg['desc']} ===", flush=True)
    p = serial.Serial(port_name, 115200, timeout=0.2)

    # Аппаратный сброс: каждый прогон стартует с одинакового состояния.
    p.setDTR(False)
    p.setRTS(True)
    time.sleep(0.1)
    p.setRTS(False)

    log = b""
    t0 = time.time()
    while time.time() - t0 < 6:  # дождаться загрузки и приглашения
        log += p.read(4096)

    for cmd in cfg["setup"]:
        log += send(p, cmd, 1.0)
    log += send(p, "timing-reset", 0.6)

    start = time.time()
    next_chatter = start + 1.0
    nvs_done = False
    while time.time() - start < seconds:
        log += p.read(4096)
        now = time.time()
        if cfg["chatter"] and now >= next_chatter:
            p.write(b"status\r\n")
            p.flush()
            next_chatter = now + 1.0
        if cfg["nvs"] and not nvs_done and now - start > seconds / 2:
            p.write(b"persist\r\n")
            p.flush()
            nvs_done = True
        time.sleep(0.05)

    tail = send(p, "timing", 2.5)
    tail += send(p, "safety", 1.5)
    tail += send(p, "imu", 1.5)
    log += tail
    p.close()

    path = os.path.join(out_dir, f"v0.6a_timing_{key}.log")
    open(path, "wb").write(log)
    print(f"    лог: {path}")

    text = log.decode("utf-8", "replace")
    started = False
    for line in text.splitlines():
        if "периодичность" in line:
            started = True
        if started and ("control" in line or "период " in line or "дедлайны" in line or
                        "исполнение" in line or "icm20948" in line):
            print("    " + line.strip())
        if started and "refloat_thd" in line:
            break
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-3120")
    ap.add_argument("--seconds", type=float, default=62)
    ap.add_argument("--runs", default="A,B,C,D,E")
    ap.add_argument("--out", default="build/debug")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    for key in a.runs.split(","):
        key = key.strip().upper()
        if key not in RUNS:
            print(f"неизвестный прогон {key}", file=sys.stderr)
            continue
        run_one(a.port, key, a.seconds, a.out)


if __name__ == "__main__":
    main()
