#!/usr/bin/env python3
"""Побайтовое сравнение конфигурации VESC до и после записи (ТЗ v0.7D.1 §9).

Сравниваются СЫРЫЕ БАЙТЫ, а не разобранные поля. Разобранные поля показывают
то, что мы решили показать; байты показывают всё. Каждое расхождение
сопоставляется с именем поля по сериализатору bldc — чтобы «изменился байт
457» читалось как «изменился si_battery_cells».

Ожидаемые изменения задаются явно. Всё, что не заявлено ожидаемым, выводится
как НЕОБЪЯСНЁННОЕ и означает остановку.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vesc_backup import build_map, parse_console_log, get  # noqa: E402

# Поля, которые ESC перемеряет при каждом включении. Их расхождение между
# двумя чтениями ожидаемо и не означает, что кто-то что-то записал
# (см. docs/can_active_diag.md §8).
RUNTIME_FIELDS = {
    "foc_offsets_current[0]", "foc_offsets_current[1]", "foc_offsets_current[2]",
    "foc_offsets_voltage[0]", "foc_offsets_voltage[1]", "foc_offsets_voltage[2]",
    "foc_offsets_voltage_undriven[0]", "foc_offsets_voltage_undriven[1]",
    "foc_offsets_voltage_undriven[2]",
}


def field_at(rows, payload_off):
    """Имя поля, которому принадлежит байт ответа COMM."""
    off = payload_off - 1  # первый байт ответа — номер пакета COMM
    best = None
    for name, (a, kind, _scale) in rows.items():
        if name == "__total__":
            continue
        size = {"uint8": 1, "int8": 1, "uint16": 2, "int16": 2, "uint32": 4,
                "int32": 4, "float16": 2, "float32": 4, "float32_auto": 4, "u8": 1}[kind]
        if a <= off < a + size:
            best = name
            break
    return best or "?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", required=True, help=".bin из резервной копии")
    ap.add_argument("--after-log", required=True, help="вывод can-diag-hex")
    ap.add_argument("--request", required=True, choices=["GET_MCCONF", "GET_APPCONF"])
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--expect", action="append", default=[],
                    help="ожидаемое изменение: поле=старое:новое")
    ap.add_argument("--cg", default="/tmp/bldc/cg.c")
    a = ap.parse_args()

    if a.request == "GET_MCCONF":
        rows = build_map(a.cg, "confgenerator_serialize_mcconf", "mc_configuration")
    else:
        rows = build_map(a.cg, "confgenerator_serialize_appconf", "app_configuration")

    before = open(a.before, "rb").read()
    blobs = parse_console_log(a.after_log)
    key = (a.request, a.id)
    if key not in blobs:
        print(f"нет ответа {a.request} от {a.id} в {a.after_log}")
        return 2
    after = bytes.fromhex(blobs[key])

    print(f"=== {a.request} id={a.id}: {len(before)} -> {len(after)} байт")
    if len(before) != len(after):
        print("ДЛИНА ИЗМЕНИЛАСЬ — остановка")
        return 1

    expected = {}
    for e in a.expect:
        name, _, vals = e.partition("=")
        old, _, new = vals.partition(":")
        expected[name] = (old, new)

    changed = {}
    for i in range(len(before)):
        if before[i] != after[i]:
            changed.setdefault(field_at(rows, i), []).append(i)

    if not changed:
        print("изменений нет")

    verdict = 0
    for name, offs in sorted(changed.items(), key=lambda kv: kv[1][0]):
        ob = "".join(f"{before[i]:02x}" for i in offs)
        nb = "".join(f"{after[i]:02x}" for i in offs)
        old_v = get(before, rows, name)
        new_v = get(after, rows, name)
        if name in expected:
            tag = "ОЖИДАЕМОЕ"
            want_old, want_new = expected[name]
            if str(old_v) != want_old or str(new_v) != want_new:
                tag = "ОЖИДАЛОСЬ ДРУГОЕ (%s -> %s)" % (want_old, want_new)
                verdict = 1
        elif name in RUNTIME_FIELDS:
            tag = "runtime, ESC меряет при включении"
        else:
            tag = "*** НЕОБЪЯСНЁННОЕ ***"
            verdict = 1
        print(f"  {name:<34} смещения {str(offs):<22} {ob} -> {nb}   "
              f"{old_v} -> {new_v}   [{tag}]")

    for name in expected:
        if name not in changed:
            print(f"  {name:<34} НЕ ИЗМЕНИЛСЯ, хотя ожидалось   [*** запись не прошла ***]")
            verdict = 1

    print("ВЕРДИКТ:", "всё объяснено" if verdict == 0 else "ЕСТЬ НЕОБЪЯСНЁННОЕ — STOP")
    return verdict


if __name__ == "__main__":
    sys.exit(main())
