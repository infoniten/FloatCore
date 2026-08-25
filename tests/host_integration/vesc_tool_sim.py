#!/usr/bin/env python3
"""
Симулятор VESC Tool: проходит сценарий из Definition of Done ТЗ v0.4 §11.

Говорит на оригинальном протоколе VESC поверх TCP — тем же, что и настоящий
VESC Tool. Не заменяет проверку живым VESC Tool, но позволяет проверять
регрессии в CI без GUI.

  python3 tests/host_integration/vesc_tool_sim.py [--port 65102] [--host 127.0.0.1]
"""

import argparse
import socket
import struct
import sys
import zlib

# --- идентификаторы команд (bldc/datatypes.h) --------------------------------
COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_ALIVE = 30
COMM_CUSTOM_APP_DATA = 36
COMM_GET_CUSTOM_CONFIG_XML = 92
COMM_GET_CUSTOM_CONFIG = 93
COMM_GET_CUSTOM_CONFIG_DEFAULT = 94
COMM_SET_CUSTOM_CONFIG = 95
COMM_GET_QML_UI_APP = 118

# --- протокол пакета Refloat (refloat-upstream/doc/commands/) ----------------
REFLOAT_IFACE_ID = 101
REFLOAT_COMMAND_INFO = 0

HW_TYPE_CUSTOM_MODULE = 2

failures = 0
checks = 0


def check(ok, msg):
    global failures, checks
    checks += 1
    print(f"      {'\033[32mPASS\033[0m' if ok else '\033[31mFAIL\033[0m'} {msg}")
    if not ok:
        failures += 1


def info(msg):
    print(f"      \033[90m·\033[0m {msg}")


def section(name):
    print(f"\n  \033[1m{name}\033[0m")


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class VescLink:
    """Кадрирование VESC поверх TCP."""

    def __init__(self, host, port, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.buf = b""

    def close(self):
        self.sock.close()

    def send(self, payload: bytes):
        if len(payload) <= 255:
            frame = bytes([2, len(payload)])
        else:
            frame = bytes([3]) + struct.pack(">H", len(payload))
        frame += payload + struct.pack(">H", crc16(payload)) + bytes([3])
        self.sock.sendall(frame)

    def recv_payload(self, timeout=5.0):
        self.sock.settimeout(timeout)
        while True:
            payload = self._try_parse()
            if payload is not None:
                return payload
            chunk = self.sock.recv(8192)
            if not chunk:
                raise ConnectionError("соединение закрыто устройством")
            self.buf += chunk

    def _try_parse(self):
        b = self.buf
        while b:
            if b[0] == 2:
                start = 2
                if len(b) < 2:
                    return None
                length = b[1]
            elif b[0] == 3:
                start = 3
                if len(b) < 3:
                    return None
                length = struct.unpack(">H", b[1:3])[0]
            else:
                b = b[1:]
                continue
            if len(b) < start + length + 3:
                self.buf = b
                return None
            payload = b[start:start + length]
            crc_rx = struct.unpack(">H", b[start + length:start + length + 2])[0]
            stop = b[start + length + 2]
            if stop != 3 or crc_rx != crc16(payload):
                b = b[1:]
                continue
            self.buf = b[start + length + 3:]
            return payload
        self.buf = b
        return None

    def request(self, payload: bytes, expect_id: int, timeout=5.0):
        self.send(payload)
        deadline_tries = 12
        for _ in range(deadline_tries):
            reply = self.recv_payload(timeout)
            if reply and reply[0] == expect_id:
                return reply
        raise TimeoutError(f"нет ответа с id={expect_id}")


def parse_fw_version(p: bytes):
    assert p[0] == COMM_FW_VERSION
    i = 1
    major, minor = p[i], p[i + 1]
    i += 2
    end = p.index(0, i)
    hw = p[i:end].decode()
    i = end + 1
    uuid = p[i:i + 12]
    i += 12
    is_paired, test_fw, hw_type, cfg_num, phase_filters, qml_hw, qml_app, nrf = p[i:i + 8]
    i += 8
    end = p.index(0, i)
    fw_name = p[i:end].decode()
    return dict(major=major, minor=minor, hw=hw, uuid=uuid.hex(), hw_type=hw_type,
                cfg_num=cfg_num, qml_hw=qml_hw, qml_app=qml_app, fw_name=fw_name)


def parse_values(p: bytes):
    assert p[0] == COMM_GET_VALUES
    i = 1

    def i16():
        nonlocal i
        v = struct.unpack(">h", p[i:i + 2])[0]
        i += 2
        return v

    def i32():
        nonlocal i
        v = struct.unpack(">i", p[i:i + 4])[0]
        i += 4
        return v

    return dict(temp_mos=i16() / 10, temp_motor=i16() / 10, current_motor=i32() / 100,
                current_in=i32() / 100, id=i32() / 100, iq=i32() / 100, duty=i16() / 1000,
                rpm=i32(), v_in=i16() / 10, amp_hours=i32() / 1e4,
                amp_hours_charged=i32() / 1e4, watt_hours=i32() / 1e4,
                watt_hours_charged=i32() / 1e4, tachometer=i32(), tachometer_abs=i32(),
                fault=p[i])


def fetch_chunked(link, cmd, conf_ind=None):
    """Забрать данные чанками так же, как это делает VESC Tool."""
    def req(size, offset):
        pl = bytes([cmd])
        if conf_ind is not None:
            pl += bytes([conf_ind & 0xFF])
        pl += struct.pack(">ii", size, offset)
        reply = link.request(pl, cmd)
        i = 1
        if conf_ind is not None:
            i += 1
        total, offs = struct.unpack(">ii", reply[i:i + 8])
        return total, offs, reply[i + 8:]

    total, _, data = req(10, 0)
    while len(data) < total:
        left = total - len(data)
        _, _, chunk = req(min(400, left), len(data))
        if not chunk:
            break
        data += chunk
    return total, data


def qt_uncompress(blob: bytes) -> bytes:
    """qUncompress: 4 байта big-endian с размером + zlib."""
    expected = struct.unpack(">I", blob[:4])[0]
    raw = zlib.decompress(blob[4:])
    if len(raw) != expected:
        raise ValueError(f"размер не совпал: {len(raw)} вместо {expected}")
    return raw


# --- смещение параметра kp в сериализованной конфигурации --------------------
# kp — первый параметр в SerOrder, тип double, vTx=7 (float16 со шкалой 10).
# Сериализация: [signature:4][kp:2][kp2:2]...
KP_OFFSET = 4
KP_SCALE = 10.0


def get_kp(cfg: bytes) -> float:
    return struct.unpack(">h", cfg[KP_OFFSET:KP_OFFSET + 2])[0] / KP_SCALE


def set_kp(cfg: bytes, value: float) -> bytes:
    return (cfg[:KP_OFFSET] + struct.pack(">h", round(value * KP_SCALE))
            + cfg[KP_OFFSET + 2:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=65102)
    a = ap.parse_args()

    print("\nСимулятор VESC Tool → FloatCore Host")
    print("================================================================")

    # ---------------------------------------------------------------- 1. connect
    section("1—4. Подключение и Firmware Info")
    link = VescLink(a.host, a.port)
    fw = parse_fw_version(link.request(bytes([COMM_FW_VERSION]), COMM_FW_VERSION))
    info(f"{fw['fw_name']} / {fw['hw']}, протокол {fw['major']}.{fw['minor']:02d}, "
         f"UUID {fw['uuid']}")
    check(fw["fw_name"] == "FloatCore" and fw["hw"] == "FloatCore",
          "устройство идентифицируется как FloatCore")
    check(fw["hw_type"] == HW_TYPE_CUSTOM_MODULE,
          f"hw_type = CUSTOM_MODULE ({fw['hw_type']}), не VESC")
    check((fw["major"], fw["minor"]) == (6, 6),
          f"версия протокола {fw['major']}.{fw['minor']:02d} известна VESC Tool")
    check(fw["cfg_num"] == 1, "объявлена одна пользовательская конфигурация")
    check(fw["qml_app"] == 1, "объявлен QML интерфейса приложения")

    # ---------------------------------------------------------------- 5. telemetry
    section("5. Realtime telemetry")
    vals = parse_values(link.request(bytes([COMM_GET_VALUES]), COMM_GET_VALUES))
    info(f"v_in={vals['v_in']} В, ток мотора={vals['current_motor']} А, "
         f"duty={vals['duty']}, rpm={vals['rpm']}, "
         f"t_fet={vals['temp_mos']} °C, fault={vals['fault']}")
    check(vals["fault"] == 0, "код фолта = 0")
    check(isinstance(vals["rpm"], int), "телеметрия декодируется в ожидаемом формате")

    # ---------------------------------------------------------------- 6. QML
    section("6. Загрузка Refloat UI")
    total, qml_blob = fetch_chunked(link, COMM_GET_QML_UI_APP)
    check(len(qml_blob) == total, f"QML получен целиком: {len(qml_blob)} из {total} байт")
    qml = qt_uncompress(qml_blob).decode("utf-8")
    info(f"после распаковки {len(qml)} байт")
    check('tabTitle: "Refloat"' in qml, "это интерфейс Refloat (tabTitle)")
    check("import Vedder.vesc.vescinterface 1.0" in qml, "QML импортирует backend VESC Tool")
    check("{{VERSION}}" not in qml and "{{PACKAGE_NAME}}" not in qml,
          "подстановки версии и имени пакета выполнены")

    # ---------------------------------------------------------------- 7. config XML
    section("7. Схема и чтение конфигурации")
    total, xml_blob = fetch_chunked(link, COMM_GET_CUSTOM_CONFIG_XML, conf_ind=0)
    check(len(xml_blob) == total, f"XML схема получена целиком: {total} байт")
    xml = qt_uncompress(xml_blob).decode("utf-8")
    check("<config_name>" in xml and "RefloatConfig" in xml,
          "схема — это settings.xml Refloat")
    check("<kp>" in xml and "<mahony_kp>" in xml, "параметры Refloat на месте")

    cfg = link.request(bytes([COMM_GET_CUSTOM_CONFIG, 0]), COMM_GET_CUSTOM_CONFIG)[2:]
    kp_before = get_kp(cfg)
    info(f"конфигурация {len(cfg)} байт, kp (Angle P) = {kp_before}")
    check(len(cfg) == 282, f"размер соответствует сериализации Refloat ({len(cfg)} байт)")

    defaults = link.request(bytes([COMM_GET_CUSTOM_CONFIG_DEFAULT, 0]),
                            COMM_GET_CUSTOM_CONFIG_DEFAULT)[2:]
    check(get_kp(defaults) == 20.0, f"значение по умолчанию kp = {get_kp(defaults)}")

    # ---------------------------------------------------------------- 8-9. write
    section("8—9. Изменение и сохранение параметра")
    kp_new = 12.5 if kp_before != 12.5 else 17.5
    reply = link.request(bytes([COMM_SET_CUSTOM_CONFIG, 0]) + set_kp(cfg, kp_new),
                         COMM_GET_CUSTOM_CONFIG)
    kp_after = get_kp(reply[2:])
    info(f"записали kp = {kp_new}, устройство подтвердило {kp_after}")
    check(kp_after == kp_new, "устройство приняло новое значение")

    # ---------------------------------------------------------------- custom app
    section("Custom App Data (протокол Refloat)")
    # Заголовок команды пакета: [package_interface_id=101][command_id][данные]
    # COMMAND_INFO = 0, запрашиваем версию 2 (doc/commands/INFO.md).
    link.send(bytes([COMM_CUSTOM_APP_DATA, REFLOAT_IFACE_ID, REFLOAT_COMMAND_INFO, 2]))
    reply = link.recv_payload()
    check(reply[0] == COMM_CUSTOM_APP_DATA, "получен ответ COMM_CUSTOM_APP_DATA")
    body = reply[1:]
    check(len(body) >= 2 and body[0] == REFLOAT_IFACE_ID and body[1] == REFLOAT_COMMAND_INFO,
          f"заголовок ответа совпадает с запросом ({body[0]}, {body[1]})")
    if len(body) >= 26:
        version = body[2]
        name = body[4:24].split(b"\x00")[0].decode(errors="replace")
        major, minor, patch = body[24], body[25], body[26]
        info(f"INFO v{version}: пакет \"{name}\" {major}.{minor}.{patch}, "
             f"всего {len(body)} байт")
        check(name == "Refloat", f"пакет представился как \"{name}\"")
        check((major, minor) == (1, 3), f"версия пакета {major}.{minor}.{patch}")
    else:
        check(False, f"ответ INFO слишком короткий: {len(body)} байт")

    # ---------------------------------------------------------------- 10-11. reconnect
    section("10—11. Переподключение и проверка сохранности")
    link.close()
    link = VescLink(a.host, a.port)
    fw2 = parse_fw_version(link.request(bytes([COMM_FW_VERSION]), COMM_FW_VERSION))
    check(fw2["uuid"] == fw["uuid"], "UUID стабилен между подключениями")

    cfg2 = link.request(bytes([COMM_GET_CUSTOM_CONFIG, 0]), COMM_GET_CUSTOM_CONFIG)[2:]
    kp_reconnect = get_kp(cfg2)
    info(f"после переподключения kp = {kp_reconnect}")
    check(kp_reconnect == kp_new, "изменение пережило переподключение")

    # ---------------------------------------------------------------- safety
    section("Безопасность: команды мотору")
    link.send(bytes([COMM_SET_CURRENT]) + struct.pack(">i", 100000))  # 1000 А
    link.send(bytes([COMM_ALIVE]))
    vals2 = parse_values(link.request(bytes([COMM_GET_VALUES]), COMM_GET_VALUES))
    check(vals2["current_motor"] == 0.0,
          f"после COMM_SET_CURRENT ток мотора остался {vals2['current_motor']} А")
    check(True, "соединение живо после запрещённой команды")

    link.close()

    print("\n================================================================")
    if failures:
        print(f"\033[31mПровалено: {failures} из {checks}\033[0m\n")
        return 1
    print(f"\033[32mВсе проверки пройдены\033[0m ({checks})\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
