#!/usr/bin/env python3
"""Разобрать ответ COMM_GET_VALUES, снятый через CAN.

Раскладка — из bldc release_6_06, comm/commands.c:384-460, при маске
0xFFFFFFFF (полный набор). Смещения считаются от начала ответа COMM, то есть
байт 0 — номер пакета.

Только чтение. Коды отказов печатаются словами: числовой код без расшифровки
одинаково похож на «всё в порядке» и на «мотор отключён».
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vesc_backup import parse_console_log  # noqa: E402

# mc_fault_code из bldc/datatypes.h
FAULTS = [
    "NONE", "OVER_VOLTAGE", "UNDER_VOLTAGE", "DRV", "ABS_OVER_CURRENT",
    "OVER_TEMP_FET", "OVER_TEMP_MOTOR", "GATE_DRIVER_OVER_VOLTAGE",
    "GATE_DRIVER_UNDER_VOLTAGE", "MCU_UNDER_VOLTAGE",
    "BOOTING_FROM_WATCHDOG_RESET", "ENCODER_SPI", "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE", "FLASH_CORRUPTION",
    "HIGH_OFFSET_CURRENT_SENSOR_1", "HIGH_OFFSET_CURRENT_SENSOR_2",
    "HIGH_OFFSET_CURRENT_SENSOR_3", "UNBALANCED_CURRENTS", "BRK", "RESOLVER_LOT",
    "RESOLVER_DOS", "RESOLVER_LOS", "FLASH_CORRUPTION_APP_CFG",
    "FLASH_CORRUPTION_MC_CFG", "ENCODER_NO_MAGNET", "ENCODER_MAGNET_TOO_STRONG",
    "PHASE_FILTER", "ENCODER_FAULT", "LV_OUTPUT_FAULT",
]

# (смещение, тип, масштаб, имя)
LAYOUT = [
    (1, "h", 10.0, "temp_fet"),
    (3, "h", 10.0, "temp_motor"),
    (5, "i", 100.0, "current_motor"),
    (9, "i", 100.0, "current_in"),
    (13, "i", 100.0, "id"),
    (17, "i", 100.0, "iq"),
    (21, "h", 1000.0, "duty"),
    (23, "i", 1.0, "rpm"),
    (27, "h", 10.0, "v_in"),
    (29, "i", 10000.0, "amp_hours"),
    (33, "i", 10000.0, "amp_hours_charged"),
    (45, "i", 1.0, "tachometer"),
    (49, "i", 1.0, "tachometer_abs"),
]


def decode(blob):
    out = {}
    for off, kind, scale, name in LAYOUT:
        size = 2 if kind == "h" else 4
        if off + size > len(blob):
            continue
        v = struct.unpack_from(">" + kind, blob, off)[0]
        out[name] = v / scale if scale != 1.0 else v
    if len(blob) > 53:
        code = blob[53]
        out["fault"] = FAULTS[code] if code < len(FAULTS) else f"код {code}"
        out["fault_code"] = code
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", action="append", required=True)
    a = ap.parse_args()

    blobs = {}
    for p in a.log:
        blobs.update(parse_console_log(p))

    rows = [(cid, decode(bytes.fromhex(h)))
            for (req, cid), h in sorted(blobs.items()) if req == "GET_VALUES"]
    if not rows:
        print("в логах нет ответов GET_VALUES")
        return 1

    cols = ["temp_fet", "temp_motor", "v_in", "current_motor", "current_in",
            "duty", "rpm", "tachometer"]
    print(f"{'':<10}" + "".join(f"{c:>15}" for c in cols) + f"{'fault':>32}")
    for cid, d in rows:
        line = f"id={cid:<7}"
        for c in cols:
            v = d.get(c)
            line += f"{v:>15.2f}" if isinstance(v, float) else f"{str(v):>15}"
        line += f"{d.get('fault', '?'):>32}"
        print(line)
    print()
    print("temp_motor около −84 °C означает, что термистор НЕ подключён:")
    print("разомкнутый датчик даёт такое значение после пересчёта NTC.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
