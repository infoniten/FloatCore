#!/usr/bin/env python3
"""Read-only аудит живого VESC по его же протоколу (ТЗ v0.7A §3).

Только чтение. Список разрешённых команд задан белым списком, и отправить
что-либо вне его невозможно: request() отказывается кодировать чужой
идентификатор. Это не декларация, а свойство кода — команды вроде
COMM_SET_CURRENT в белый список не входят и попасть в него случайно не могут.

    python3 tools/vesc_audit.py --port /dev/cu.usbmodem3041 --out build/debug/vesc_audit.json
"""
import argparse
import binascii
import glob
import json
import os
import struct
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

# --- белый список. Только чтение. Ничего изменяющего состояние здесь нет и
# --- быть не может: любой другой идентификатор отвергается request().
READ_ONLY = {
    "COMM_FW_VERSION": 0,
    "COMM_GET_VALUES": 4,
    "COMM_GET_MCCONF": 14,
    "COMM_GET_MCCONF_DEFAULT": 15,
    "COMM_GET_APPCONF": 17,
    "COMM_GET_APPCONF_DEFAULT": 18,
    "COMM_GET_DECODED_PPM": 31,
    "COMM_GET_DECODED_ADC": 32,
    "COMM_GET_DECODED_CHUK": 33,
    "COMM_PING_CAN": 62,
    "COMM_GET_IMU_DATA": 65,
    # Идентификаторы сверены с bldc/datatypes.h. Здесь только GET/READ:
    # COMM_LISP_SET_RUNNING (133), COMM_LISP_ERASE_CODE (132), COMM_QMLUI_ERASE
    # (120) и любые WRITE в белый список не входят и входить не должны.
    "COMM_GET_STATS": 128,
    "COMM_GET_QML_UI_HW": 117,
    "COMM_GET_QML_UI_APP": 118,
    "COMM_LISP_GET_STATS": 134,
}
# Команда пересылки на peer по CAN. Сама по себе она ничего не меняет: меняет
# то, ЧТО в неё вложено. Вкладываем только команды из белого списка выше.
COMM_FORWARD_CAN = 34


def crc16(data: bytes) -> int:
    # CCITT/XMODEM, poly 0x1021, init 0 — как в bldc/comm/crc.c.
    tab = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
        tab.append(c)
    crc = 0
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ tab[((crc >> 8) ^ b) & 0xFF]
    return crc


def frame(payload: bytes) -> bytes:
    n = len(payload)
    if n <= 255:
        head = bytes([2, n])
    elif n <= 65535:
        head = bytes([3, n >> 8, n & 0xFF])
    else:
        head = bytes([4, n >> 16, (n >> 8) & 0xFF, n & 0xFF])
    c = crc16(payload)
    return head + payload + bytes([c >> 8, c & 0xFF, 3])


class Vesc:
    def __init__(self, port, baud=115200, timeout=2.0):
        self.p = serial.Serial(port, baud, timeout=0.2)
        self.timeout = timeout
        self.buf = b""

    def close(self):
        self.p.close()

    def _read_frame(self, deadline):
        while time.time() < deadline:
            self.buf += self.p.read(4096)
            while True:
                if not self.buf:
                    break
                start = self.buf[0]
                if start not in (2, 3, 4):
                    self.buf = self.buf[1:]
                    continue
                hdr = 1 + (start - 1)
                if len(self.buf) < hdr + 1:
                    break
                if start == 2:
                    n = self.buf[1]
                elif start == 3:
                    n = (self.buf[1] << 8) | self.buf[2]
                else:
                    n = (self.buf[1] << 16) | (self.buf[2] << 8) | self.buf[3]
                total = hdr + n + 3
                if len(self.buf) < total:
                    break
                payload = self.buf[hdr:hdr + n]
                c = (self.buf[hdr + n] << 8) | self.buf[hdr + n + 1]
                stop = self.buf[hdr + n + 2]
                self.buf = self.buf[total:]
                if stop == 3 and c == crc16(payload):
                    return payload
        return None

    def request(self, name, extra=b"", can_id=None):
        """Отправить read-only запрос и дождаться ответа."""
        if name not in READ_ONLY:
            raise ValueError(f"команда {name} не входит в белый список только-чтение")
        pid = READ_ONLY[name]
        payload = bytes([pid]) + extra
        if can_id is not None:
            payload = bytes([COMM_FORWARD_CAN, can_id]) + payload
        self.p.reset_input_buffer()
        self.buf = b""
        self.p.write(frame(payload))
        self.p.flush()
        deadline = time.time() + self.timeout
        while True:
            r = self._read_frame(deadline)
            if r is None:
                return None
            if r and r[0] == pid:
                return r
            # Ответ на другой запрос или асинхронное сообщение — пропускаем.


def u8(b, i):
    return b[i], i + 1


def u16(b, i):
    return struct.unpack_from(">H", b, i)[0], i + 2


def i16(b, i):
    return struct.unpack_from(">h", b, i)[0], i + 2


def u32(b, i):
    return struct.unpack_from(">I", b, i)[0], i + 4


def i32(b, i):
    return struct.unpack_from(">i", b, i)[0], i + 4


def f32_auto(b, i):
    # buffer_get_float32_auto: мантисса/экспонента, как в bldc/comm/buffer.c
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


def parse_fw(pl):
    i = 1
    major, i = u8(pl, i)
    minor, i = u8(pl, i)
    end = pl.find(b"\0", i)
    hw = pl[i:end].decode("ascii", "replace")
    i = end + 1
    uuid = binascii.hexlify(pl[i:i + 12]).decode()
    i += 12
    out = {"fw_major": major, "fw_minor": minor, "fw": f"{major}.{minor}",
           "hw_name": hw, "uuid": uuid}
    if i < len(pl):
        out["pairing_done"] = pl[i]
        i += 1
    if i < len(pl):
        out["fw_test_version"] = pl[i]
        i += 1
    if i < len(pl):
        out["hw_type"] = pl[i]
        i += 1
    if i < len(pl):
        out["custom_config"] = pl[i]
        i += 1
    return out


def parse_values(pl):
    i = 1
    o = {}
    o["temp_fet"], i = i16(pl, i)
    o["temp_motor"], i = i16(pl, i)
    o["current_motor"], i = i32(pl, i)
    o["current_in"], i = i32(pl, i)
    o["id"], i = i32(pl, i)
    o["iq"], i = i32(pl, i)
    o["duty"], i = i16(pl, i)
    o["rpm"], i = i32(pl, i)
    o["v_in"], i = i16(pl, i)
    o["amp_hours"], i = i32(pl, i)
    o["amp_hours_charged"], i = i32(pl, i)
    o["watt_hours"], i = i32(pl, i)
    o["watt_hours_charged"], i = i32(pl, i)
    o["tacho"], i = i32(pl, i)
    o["tacho_abs"], i = i32(pl, i)
    o["fault"], i = u8(pl, i)
    for k, s in (("temp_fet", 10), ("temp_motor", 10), ("current_motor", 100),
                 ("current_in", 100), ("id", 100), ("iq", 100), ("duty", 1000),
                 ("v_in", 10), ("amp_hours", 10000), ("amp_hours_charged", 10000),
                 ("watt_hours", 10000), ("watt_hours_charged", 10000)):
        o[k] = o[k] / s
    if i < len(pl):
        o["pid_pos"], i = i32(pl, i)
        o["pid_pos"] /= 1e6
    if i < len(pl):
        o["controller_id"], i = u8(pl, i)
    return o


def parse_ping_can(pl):
    return list(pl[1:])


def decode_mcconf(blob, schema):
    out = {}
    for p in schema["params"]:
        off, kind, sz, sc = p["offset"], p["kind"], p["size"], p["scale"]
        if off + sz > len(blob):
            continue
        b = blob
        if kind in ("MC_KIND_BYTE", "MC_KIND_U8", "MC_KIND_BOOL", "MC_KIND_ENUM"):
            v = b[off]
        elif kind == "MC_KIND_I8":
            v = struct.unpack_from(">b", b, off)[0]
        elif kind == "MC_KIND_U16":
            v = struct.unpack_from(">H", b, off)[0]
        elif kind in ("MC_KIND_I16", "MC_KIND_DOUBLE16"):
            v = struct.unpack_from(">h", b, off)[0]
        elif kind == "MC_KIND_U32":
            v = struct.unpack_from(">I", b, off)[0]
        elif kind in ("MC_KIND_I32", "MC_KIND_DOUBLE32"):
            v = struct.unpack_from(">i", b, off)[0]
        elif kind == "MC_KIND_DOUBLE32_AUTO":
            v, _ = f32_auto(b, off)
            out[p["name"]] = v
            continue
        else:
            continue
        out[p["name"]] = v / sc if sc not in (0, 1.0) else v
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="build/debug/vesc_audit.json")
    ap.add_argument("--can-id", type=int, action="append", default=[],
                    help="дополнительно прочитать конфигурацию peer по CAN")
    a = ap.parse_args()

    v = Vesc(a.port, a.baud)
    result = {"port": a.port, "raw": {}}

    def grab(name, key=None, can_id=None, extra=b""):
        pl = v.request(name, extra=extra, can_id=can_id)
        k = key or name
        if pl is None:
            result["raw"][k] = None
            print(f"  {k}: нет ответа")
            return None
        result["raw"][k] = binascii.hexlify(pl).decode()
        print(f"  {k}: {len(pl)} байт")
        return pl

    print("firmware:")
    fw = grab("COMM_FW_VERSION")
    if fw:
        result["fw"] = parse_fw(fw)
        print("   ", json.dumps(result["fw"], ensure_ascii=False))

    print("values:")
    val = grab("COMM_GET_VALUES")
    if val:
        result["values"] = parse_values(val)

    print("ping can:")
    pc = v.request("COMM_PING_CAN")
    if pc is not None:
        result["raw"]["COMM_PING_CAN"] = binascii.hexlify(pc).decode()
        result["can_peers"] = parse_ping_can(pc)
        print("   peers:", result["can_peers"])
    else:
        result["can_peers"] = None
        print("    нет ответа")

    print("mcconf / appconf (локальная половина):")
    grab("COMM_GET_MCCONF")
    grab("COMM_GET_APPCONF")
    grab("COMM_GET_MCCONF_DEFAULT")

    for cid in a.can_id:
        print(f"через CAN, id={cid}:")
        grab("COMM_FW_VERSION", key=f"COMM_FW_VERSION@{cid}", can_id=cid)
        grab("COMM_GET_VALUES", key=f"COMM_GET_VALUES@{cid}", can_id=cid)
        grab("COMM_GET_MCCONF", key=f"COMM_GET_MCCONF@{cid}", can_id=cid)
        grab("COMM_GET_APPCONF", key=f"COMM_GET_APPCONF@{cid}", can_id=cid)

    v.close()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"\nсырые ответы сохранены: {a.out}")


if __name__ == "__main__":
    main()
