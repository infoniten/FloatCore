#!/usr/bin/env python3
"""Разобрать дампы конфигурации VESC, снятые через CAN, и сохранить резервные копии.

Только чтение. Ничего никуда не пишет, кроме файлов резервной копии на диске:
права записи в ESC у FloatCore нет и не появится — COMM_SET_MCCONF и
COMM_SET_APPCONF не входят в белый список передачи и не могут быть собраны.

Резервная копия хранится в двух формах сразу:

  .bin   сырые байты ответа COMM, ровно как их отдал ESC;
  .json  те же байты в hex плюс разобранные важные поля, сигнатура и отметка
         времени.

Две формы нужны по разным причинам. Сырые байты — единственное, с чем можно
сравнивать побайтово после записи. Разобранные поля — единственное, что можно
прочитать глазами и заметить, что копия вообще не та.
"""
import argparse
import binascii
import json
import os
import re
import struct
import sys
import time


def u32(b, i):
    return struct.unpack_from(">I", b, i)[0], i + 4


def f32_auto(b, i):
    """buffer_get_float32_auto из bldc/util/buffer.c."""
    v, i = u32(b, i)
    e = (v >> 23) & 0xFF
    sig_i = v & ((1 << 23) - 1)
    neg = bool(v & (1 << 31))
    sig = float(sig_i) / (1 << 23)
    if e != 0 or sig != 0:
        sig += 1.0
        e -= 127
    res = sig * (2.0 ** e)
    return (-res if neg else res), i


SIZE = {"uint8": 1, "int8": 1, "uint16": 2, "int16": 2, "uint32": 4, "int32": 4,
        "float16": 2, "float32": 4, "float32_auto": 4, "u8": 1}


def build_map(cg_path, func, struct_name):
    """Разложить сериализатор bldc на поля: имя -> (смещение, тип, масштаб).

    Разбирается ИСХОДНИК confgenerator.c, а не таблица, переписанная руками.
    Смысл в том, что раскладка обязана совпадать с прошивкой побайтово, а
    переписанная таблица расходится с ней молча.
    """
    text = open(cg_path, encoding="latin1").read()
    m = re.search(r"int32_t " + func + r"\(uint8_t \*buffer, const " + struct_name +
                  r" \*conf\) \{(.*?)\n\}", text, re.S)
    if not m:
        raise SystemExit(f"не найден сериализатор {func} в {cg_path}")
    body = re.sub(r"//[^\n]*", "", m.group(1))
    off = 0
    rows = {}
    for stmt in [x.strip().replace("\n", " ") for x in body.split(";")]:
        mm = re.search(r"buffer_append_(\w+)\(buffer,\s*(.*?),\s*&ind\)", stmt)
        if mm:
            kind, arg = mm.group(1), mm.group(2)
            name = re.search(r"conf->([\w\[\]\.]+)", arg)
            scale = re.search(r",\s*([\d.e+]+)\s*$", arg)
            rows[name.group(1) if name else "SIGNATURE"] = (
                off, kind, float(scale.group(1)) if scale else 1.0)
            off += SIZE[kind]
            continue
        mm = re.search(r"buffer\[ind\+\+\]\s*=\s*(.*)", stmt)
        if mm:
            name = re.search(r"conf->([\w\[\]\.]+)", mm.group(1))
            rows[name.group(1) if name else "?"] = (off, "u8", 1.0)
            off += 1
    rows["__total__"] = (off, "u8", 1.0)
    return rows


def get(blob, rows, name):
    """blob — полный ответ COMM, включая первый байт с номером пакета."""
    if name not in rows:
        return None
    off, kind, scale = rows[name]
    b = blob[1:]
    if off + SIZE[kind] > len(b):
        return None
    if kind == "float32_auto":
        return round(f32_auto(b, off)[0], 6)
    if kind in ("u8", "uint8"):
        return b[off]
    if kind == "int8":
        return struct.unpack_from(">b", b, off)[0]
    if kind == "uint16":
        return struct.unpack_from(">H", b, off)[0] / scale
    if kind in ("int16", "float16"):
        return struct.unpack_from(">h", b, off)[0] / scale
    if kind == "uint32":
        return struct.unpack_from(">I", b, off)[0] / scale
    if kind in ("int32", "float32"):
        return struct.unpack_from(">i", b, off)[0] / scale
    return None


def parse_console_log(path):
    """Вытащить блобы из вывода команды can-diag-hex."""
    out = {}
    cur = None
    buf = []
    for line in open(path, encoding="utf-8", errors="replace").read().splitlines():
        m = re.match(r"запрос (\S+) -> контроллер (\d+)", line)
        if m:
            if cur:
                out[cur] = "".join(buf)
            cur = (m.group(1), int(m.group(2)))
            buf = []
            continue
        m = re.match(r"\s+(\d{3})\s+([0-9a-f]+)\s*$", line)
        if m and cur:
            buf.append(m.group(2))
    if cur:
        out[cur] = "".join(buf)
    return out


MC_FIELDS = ["si_battery_cells", "si_battery_type", "si_battery_ah",
             "l_battery_cut_start", "l_battery_cut_end",
             "l_battery_regen_cut_start", "l_battery_regen_cut_end",
             "l_max_vin", "l_min_vin",
             "l_current_max", "l_current_min", "l_in_current_max",
             "l_in_current_min", "l_abs_current_max",
             "foc_motor_r", "foc_motor_l", "foc_motor_flux_linkage",
             "si_motor_poles", "si_wheel_diameter", "si_gear_ratio",
             "l_max_duty", "l_temp_fet_start", "l_temp_fet_end"]

APP_FIELDS = ["controller_id", "timeout_msec", "timeout_brake_current",
              "app_to_use", "can_baud_rate", "can_status_rate_1",
              "can_status_msgs_r1", "can_mode", "kill_sw_mode",
              "app_adc_conf.ctrl_type", "app_ppm_conf.ctrl_type"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", action="append", required=True,
                    help="вывод can-diag-hex, можно несколько раз")
    ap.add_argument("--cg", default="/tmp/bldc/cg.c",
                    help="confgenerator.c из той же версии прошивки")
    ap.add_argument("--out", default="build/debug/backup_v07d1")
    ap.add_argument("--label", default=time.strftime("%Y%m%d-%H%M%S"))
    a = ap.parse_args()

    mc = build_map(a.cg, "confgenerator_serialize_mcconf", "mc_configuration")
    app = build_map(a.cg, "confgenerator_serialize_appconf", "app_configuration")

    blobs = {}
    for path in a.log:
        blobs.update(parse_console_log(path))

    os.makedirs(a.out, exist_ok=True)
    summary = {}
    for (req, cid), hexstr in sorted(blobs.items()):
        blob = bytes.fromhex(hexstr)
        kind = {"GET_MCCONF": "mcconf", "GET_APPCONF": "appconf",
                "FW_VERSION": "fw"}.get(req, req.lower())
        base = f"{kind}_id{cid}"
        with open(os.path.join(a.out, base + ".bin"), "wb") as f:
            f.write(blob)

        rec = {"controller_id": cid, "kind": kind, "bytes": len(blob),
               "label": a.label, "raw": hexstr}
        if kind == "mcconf":
            rec["signature"] = hexstr[2:10]
            rec["fields"] = {k: get(blob, mc, k) for k in MC_FIELDS}
        elif kind == "appconf":
            rec["signature"] = hexstr[2:10]
            rec["fields"] = {k: get(blob, app, k) for k in APP_FIELDS}
        elif kind == "fw":
            z = blob.index(0, 3)
            rec["fields"] = {"version": f"{blob[1]}.{blob[2]}",
                             "hw": blob[3:z].decode(),
                             "uuid": blob[z + 1:z + 13].hex()}
        with open(os.path.join(a.out, base + ".json"), "w") as f:
            json.dump(rec, f, ensure_ascii=False, indent=1)
        summary[base] = rec

    # Проверка, что копию можно разобрать обратно: читаем с диска и сверяем.
    print(f"{'файл':<18}{'байт':>6}  сигнатура  проверка чтения")
    ok = True
    for base, rec in summary.items():
        raw = open(os.path.join(a.out, base + ".bin"), "rb").read()
        same = binascii.hexlify(raw).decode() == rec["raw"]
        ok = ok and same
        print(f"{base:<18}{len(raw):>6}  {rec.get('signature','-'):<10} "
              f"{'совпадает' if same else 'РАСХОЖДЕНИЕ'}")
    print()
    for base, rec in summary.items():
        if "fields" in rec:
            print(f"--- {base}")
            for k, v in rec["fields"].items():
                print(f"    {k:<28}{v}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
