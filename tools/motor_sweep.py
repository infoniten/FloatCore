#!/usr/bin/env python3
"""Развёртка отклика мотора по току на вывешенном колесе (ТЗ v0.9B §7-§12).

ЧТО ЭТО ТАКОЕ. Инструмент измерения, ОТДЕЛЬНЫЙ от боевой логики
безопасности: он не умеет ничего, кроме как говорить с уже существующей
консолью FloatCore теми же командами, что доступны человеку. Ни одного пути
к мотору мимо Motor Gate и координатора здесь нет и быть не может.

ЗАЧЕМ. На v0.9A выяснилось, что 0.2 А не даёт ни тока, ни вращения, 0.5 А
даёт начало отклика, а 1.0 А — уверенное вращение. Делать из трёх точек
вывод «мёртвая зона 0.5 А» нельзя: за этими числами могут стоять четыре
разные величины, и различить их можно только измерением.

    A. порог приёма команды       ESC вообще принял значение
    B. первый измеримый ток       в обмотке что-то потекло
    C. первое воспроизводимое     вал сдвинулся хотя бы раз
       движение
    D. порог устойчивого          вал крутится, а не дёргается
       вращения

Отдельно снимается отклик на УЖЕ ВРАЩАЮЩЕМСЯ колесе: если там ток начинает
действовать раньше, значит существенная часть «мёртвой зоны» — трогание с
места, а не свойство регулятора. Для балансировки это принципиально разные
диагнозы.
"""
import argparse
import glob
import os
import re
import struct
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vesc_values import decode  # noqa: E402

try:
    import serial
except ModuleNotFoundError:
    _cand = sorted(glob.glob(os.path.expanduser("~/.espressif/python_env/*/bin/python")))
    if not os.environ.get("FLOATCORE_SWEEP_REEXEC"):
        for _py in _cand:
            if subprocess.run([_py, "-c", "import serial"], capture_output=True).returncode == 0:
                os.environ["FLOATCORE_SWEEP_REEXEC"] = "1"
                os.execv(_py, [_py, os.path.abspath(__file__)] + sys.argv[1:])
    sys.exit("нужен pyserial")


class Console:
    def __init__(self, port, baud=115200):
        self.p = serial.Serial(port, baud, timeout=0.2)
        self.buf = ""

    def send(self, cmd, wait=0.45):
        self.p.reset_input_buffer()
        self.p.write((cmd + "\n").encode())
        self.p.flush()
        t0 = time.time()
        out = ""
        while time.time() - t0 < wait:
            chunk = self.p.read(4096)
            if chunk:
                out += chunk.decode("utf-8", "replace")
        self.buf += out
        return out

    def values(self, node):
        """GET_VALUES одной половины, разобранный."""
        out = self.send(f"can-diag-hex values {node}", wait=0.55)
        m = re.findall(r"^  \d{3}  ([0-9a-f]+)", out, re.M)
        if not m:
            return None
        return decode(bytes.fromhex("".join(m)))


def wait_stopped(c, node, timeout_s=25.0, quiet_s=1.2):
    """Дождаться остановки вала. Возвращает True, если дождались."""
    t0 = time.time()
    still = 0.0
    while time.time() - t0 < timeout_s:
        v = c.values(node)
        if v and abs(v.get("rpm", 0)) < 30:
            still += 0.6
            if still >= quiet_s:
                return True
        else:
            still = 0.0
    return False


def run_point(c, amps, ms, node_a, node_b, samples=3, from_rest=True,
              spinup_a=0.0, spinup_ms=0):
    if from_rest and not wait_stopped(c, node_a):
        print(f"  {amps:5.2f} А: вал не остановился, точка пропущена")
        return None

    # Предварительная раскрутка (ТЗ v0.9B §9). Смысл опыта: если малый ток
    # начинает действовать на уже вращающемся колесе, значит «мёртвая зона»
    # — это трогание с места, а не свойство регулятора. Для замкнутого
    # контура это принципиально разные диагнозы.
    if spinup_a > 0.0 and spinup_ms > 0:
        c.send(f"motor-run {spinup_a:.3f} {spinup_ms}", wait=0.3)
        time.sleep(spinup_ms / 1000.0 + 0.35)

    before_a = c.values(node_a)
    before_b = c.values(node_b)
    if not before_a or not before_b:
        return None

    c.send(f"motor-run {amps:.3f} {ms}", wait=0.35)

    peak = {"current_motor": 0.0, "duty": 0.0, "rpm": 0, "current_in": 0.0}
    for _ in range(samples):
        v = c.values(node_a)
        if not v:
            continue
        for k in peak:
            if abs(v.get(k, 0)) > abs(peak[k]):
                peak[k] = v.get(k, 0)

    # Дать команде закончиться и валу докатиться.
    time.sleep(max(0.0, ms / 1000.0 - samples * 0.55) + 1.5)
    after_a = c.values(node_a)
    after_b = c.values(node_b)
    if not after_a or not after_b:
        return None

    return {
        "amps": amps,
        "i_motor": peak["current_motor"],
        "i_in": peak["current_in"],
        "duty": peak["duty"],
        "rpm": peak["rpm"],
        "dtach_a": after_a["tachometer"] - before_a["tachometer"],
        "dtach_b": after_b["tachometer"] - before_b["tachometer"],
        "v_in": after_a.get("v_in", 0.0),
        "fault_a": after_a.get("fault", "?"),
        "fault_b": after_b.get("fault", "?"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-3120")
    ap.add_argument("--levels", default="0,0.10,0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50,0.60,0.75,1.00")
    ap.add_argument("--ms", type=int, default=1500)
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--node-a", type=int, default=118)
    ap.add_argument("--node-b", type=int, default=100)
    ap.add_argument("--moving", action="store_true",
                    help="не ждать остановки: отклик на уже вращающемся колесе")
    ap.add_argument("--spinup", type=float, default=0.0,
                    help="ток предварительной раскрутки, А (0 = без раскрутки)")
    ap.add_argument("--spinup-ms", type=int, default=800)
    ap.add_argument("--out")
    a = ap.parse_args()

    c = Console(a.port)
    levels = [float(x) for x in a.levels.split(",")]

    rows = []
    print(f"{'ток':>6} {'повтор':>6} {'I мотора':>9} {'I вход':>7} {'duty':>6} "
          f"{'ERPM':>7} {'Δtach A':>8} {'Δtach B':>8} {'fault':>6}")
    for amps in levels:
        for r in range(a.repeats):
            row = run_point(c, amps, a.ms, a.node_a, a.node_b, from_rest=not a.moving,
                            spinup_a=a.spinup, spinup_ms=a.spinup_ms)
            if not row:
                continue
            row["repeat"] = r + 1
            rows.append(row)
            print(f"{amps:6.2f} {r+1:>6} {row['i_motor']:9.2f} {row['i_in']:7.2f} "
                  f"{row['duty']:6.3f} {row['rpm']:7.0f} {row['dtach_a']:8d} "
                  f"{row['dtach_b']:8d} {row['fault_a']:>6}")
            if row["fault_a"] != "NONE" or row["fault_b"] != "NONE":
                print("  ОСТАНОВКА: VESC fault")
                return 1

    if a.out:
        import json
        with open(a.out, "w") as f:
            json.dump(rows, f, ensure_ascii=False, indent=1)
        print(f"\nзаписано: {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
