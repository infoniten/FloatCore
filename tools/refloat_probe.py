#!/usr/bin/env python3
"""Снять связь «физический наклон -> запрошенный Refloat ток» (ТЗ v0.6D §15-16).

Периодически отправляет read-only команду `imu` и разбирает из её вывода
состояние Refloat, углы AHRS и запрошенный балансировочный ток. Ничего не
меняет: команда `imu` относится к категории SAFE_READONLY.

    python3 tools/refloat_probe.py --seconds 20 --label "nose up"
"""
import argparse
import glob
import os
import re
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

RE_ACC = re.compile(r"raw accel\s+([-+][\d.]+)\s+([-+][\d.]+)\s+([-+][\d.]+)\s+g")
RE_GYR = re.compile(r"raw gyro\s+([-+][\d.]+)\s+([-+][\d.]+)\s+([-+][\d.]+)\s+°/с")
RE_AHRS = re.compile(r"AHRS платформы\s+pitch\s+([-+][\d.]+)\s+roll\s+([-+][\d.]+)\s+yaw\s+([-+][\d.]+)")
RE_RF = re.compile(r"Refloat\s+state (\w+), pitch\s+([-+][\d.]+), balance_pitch\s+([-+][\d.]+), roll\s+([-+][\d.]+)")
RE_CUR = re.compile(r"запрошенный ток\s+([-+][\d.]+) A")
RE_GATE = re.compile(r"Motor Gate\s+requested=(\d+) allowed=(\d+) sent=(\d+)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-3120")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    p = serial.Serial(a.port, a.baud, timeout=0.05)
    p.reset_input_buffer()
    period = 1.0 / a.hz
    t0 = time.time()
    buf = ""
    rows = []
    print(f"\n=== {a.label} ===")
    print("   t    state     acc_x   acc_y   acc_z   gyro_x  gyro_y  gyro_z  AHRSpitch  AHRSroll   AHRSyaw  bal_pitch   ток A  sent")
    while time.time() - t0 < a.seconds:
        p.write(b"imu\r\n")
        p.flush()
        deadline = time.time() + period
        while time.time() < deadline:
            buf += p.read(4096).decode("utf-8", "replace")
        acc = RE_ACC.search(buf)
        gyr = RE_GYR.search(buf)
        ah = RE_AHRS.search(buf)
        rf = RE_RF.search(buf)
        cur = RE_CUR.search(buf)
        gate = RE_GATE.search(buf)
        if acc and gyr and ah and rf and cur and gate:
            t = time.time() - t0
            row = (t, rf.group(1), float(acc.group(1)), float(acc.group(2)), float(acc.group(3)),
                   float(gyr.group(1)), float(gyr.group(2)), float(gyr.group(3)),
                   float(ah.group(1)), float(ah.group(2)), float(ah.group(3)),
                   float(rf.group(3)), float(cur.group(1)), int(gate.group(3)))
            rows.append(row)
            print("%5.1f  %-8s %+7.3f %+7.3f %+7.3f %+7.2f %+7.2f %+7.2f  %+8.3f  %+8.3f %+9.3f  %+9.3f  %+7.3f  %d" % row)
        buf = ""
    p.close()
    if rows:
        sent = max(r[13] for r in rows)
        cur_min = min(r[12] for r in rows)
        cur_max = max(r[12] for r in rows)
        bp_min = min(r[11] for r in rows)
        bp_max = max(r[11] for r in rows)
        gz_min = min(r[7] for r in rows)
        gz_max = max(r[7] for r in rows)
        yaw_min = min(r[10] for r in rows)
        yaw_max = max(r[10] for r in rows)
        print(f"      gyro_z {gz_min:+.2f}..{gz_max:+.2f} °/с, AHRS yaw {yaw_min:+.2f}..{yaw_max:+.2f} град")
        states = sorted({r[1] for r in rows})
        print(f"\nитог: state {'/'.join(states)}, balance_pitch {bp_min:+.2f}..{bp_max:+.2f} град, "
              f"ток {cur_min:+.3f}..{cur_max:+.3f} A, physically_sent max = {sent}")


if __name__ == "__main__":
    main()
