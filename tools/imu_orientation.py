#!/usr/bin/env python3
"""Снятие ориентации ICM-20948 через read-only консольную команду `imu`.

Прошивка не меняется: скрипт только периодически отправляет `imu` и
разбирает строки «последний семпл». Частота опроса самого датчика при этом
не меняется — задача fc_imu_hw как читала 500 Гц, так и читает; консоль
показывает последний валидный семпл.

    python3 tools/imu_orientation.py --seconds 6 --label baseline
"""
import argparse
import glob
import os
import re
import statistics
import subprocess
import sys
import time

try:
    import serial
except ModuleNotFoundError:
    _c = sorted(glob.glob(os.path.expanduser("~/.espressif/python_env/*/bin/python")))
    if not os.environ.get("FLOATCORE_SERIAL_REEXEC"):
        for _py in _c:
            if subprocess.run([_py, "-c", "import serial"], capture_output=True).returncode == 0:
                os.environ["FLOATCORE_SERIAL_REEXEC"] = "1"
                os.execv(_py, [_py, os.path.abspath(__file__)] + sys.argv[1:])
    sys.exit("нужен pyserial (есть в окружении ESP-IDF)")

RE_ACC = re.compile(r"последний семпл\s+acc\s+([-+][\d.]+)\s+([-+][\d.]+)\s+([-+][\d.]+)\s+g\s+\|a\|=([\d.]+)")
RE_GYR = re.compile(r"gyro\s+([-+][\d.]+)\s+([-+][\d.]+)\s+([-+][\d.]+)\s+°/с,\s+([\d.]+)")
RE_TX = re.compile(r"транзакции\s+ok=(\d+)\s+failed=(\d+)")
RE_HEALTH = re.compile(r"^health\s+(\w+)")


def collect(port, baud, seconds, hz):
    p = serial.Serial(port, baud, timeout=0.05)
    p.reset_input_buffer()
    period = 1.0 / hz
    t0 = time.time()
    rows, buf, tx = [], "", []
    while time.time() - t0 < seconds:
        p.write(b"imu\r\n")
        p.flush()
        deadline = time.time() + period
        while time.time() < deadline:
            buf += p.read(4096).decode("utf-8", "replace")
        acc = RE_ACC.findall(buf)
        gyr = RE_GYR.findall(buf)
        t = RE_TX.findall(buf)
        h = RE_HEALTH.findall(buf)
        if acc and gyr:
            n = min(len(acc), len(gyr))
            for i in range(n):
                a = acc[i]
                g = gyr[i]
                rows.append((float(a[0]), float(a[1]), float(a[2]), float(a[3]),
                             float(g[0]), float(g[1]), float(g[2]),
                             h[i] if i < len(h) else "?"))
            for i in range(min(len(t), n)):
                tx.append((int(t[i][0]), int(t[i][1])))
            buf = ""
    p.close()
    return rows, tx


def report(label, rows, tx):
    if not rows:
        print(f"[{label}] нет семплов")
        return None
    cols = list(zip(*rows))
    names = ["acc_x", "acc_y", "acc_z", "|a|", "gyro_x", "gyro_y", "gyro_z"]
    print(f"\n=== {label} === уникальных семплов: {len(rows)}")
    out = {}
    for i, nm in enumerate(names):
        v = list(cols[i])
        m = statistics.fmean(v)
        sd = statistics.pstdev(v) if len(v) > 1 else 0.0
        print(f"  {nm:7s} mean {m:+8.3f}  sd {sd:6.3f}  min {min(v):+8.3f}  max {max(v):+8.3f}")
        out[nm] = m
    states = set(cols[7])
    print(f"  health: {', '.join(sorted(states))}")
    if tx:
        d_ok = tx[-1][0] - tx[0][0]
        d_fail = tx[-1][1] - tx[0][1]
        tot = d_ok + d_fail
        print(f"  I2C за окно: ok={d_ok} failed={d_fail}"
              + (f" ({100.0*d_fail/tot:.1f} % отказов)" if tot else ""))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-3120")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--hz", type=float, default=6.0)
    ap.add_argument("--label", default="measure")
    a = ap.parse_args()
    rows, tx = collect(a.port, a.baud, a.seconds, a.hz)
    report(a.label, rows, tx)


if __name__ == "__main__":
    main()
