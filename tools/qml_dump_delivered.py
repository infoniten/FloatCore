#!/usr/bin/env python3
"""Снять QML ровно так, как его получает VESC Tool, и проверить на порчу.

    build/floatcore_host --port 65211 &
    python3 tools/qml_dump_delivered.py --port 65211

Забирает COMM_GET_QML_UI_APP чанками, распаковывает qUncompress, кладёт
результат в build/debug/refloat_qml_delivered.qml и сравнивает с тем, что
должен был сгенерировать tools/gen_qml.py из refloat-upstream/ui.qml.in.
Ничего не меняет — только читает.
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tests", "host_integration"))

from vesc_tool_sim import VescLink, fetch_chunked, qt_uncompress, COMM_GET_QML_UI_APP  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=65199)
    ap.add_argument("--out", default=os.path.join(ROOT, "build/debug/refloat_qml_delivered.qml"))
    a = ap.parse_args()

    link = VescLink(a.host, a.port)
    total, blob = fetch_chunked(link, COMM_GET_QML_UI_APP)
    link.close()
    raw = qt_uncompress(blob)

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    open(a.out, "wb").write(raw)
    open(a.out + ".blob", "wb").write(blob)

    version = open(os.path.join(ROOT, "refloat-upstream/version")).read().strip()
    pkg = open(os.path.join(ROOT, "refloat-upstream/package_name")).read().strip()[:20]
    expected = (open(os.path.join(ROOT, "refloat-upstream/ui.qml.in"), encoding="utf-8").read()
                .replace("{{PACKAGE_NAME}}", pkg).replace("{{VERSION}}", version).encode("utf-8"))

    checks = [
        ("объявленный размер совпадает с принятым", len(blob) == total),
        ("побайтно равен ожидаемому generated QML", raw == expected),
        ("нет BOM", not raw.startswith(b"\xef\xbb\xbf")),
        ("нет CR (только LF)", b"\r" not in raw),
        ("нет незакрытых подстановок {{...}}", b"{{" not in raw),
        ("декодируется как UTF-8", _utf8_ok(raw)),
    ]
    ok = True
    for name, res in checks:
        print(f"  [{'PASS' if res else 'FAIL'}] {name}")
        ok &= res

    print(f"\n  сжатый blob {len(blob)} B -> {len(raw)} B, {raw.count(chr(10).encode())} строк")
    print(f"  строка 1: {raw.split(chr(10).encode())[0].decode('utf-8')!r}")
    print(f"  сохранено: {a.out}")
    return 0 if ok else 1


def _utf8_ok(raw):
    try:
        raw.decode("utf-8")
        return True
    except UnicodeDecodeError:
        return False


if __name__ == "__main__":
    sys.exit(main())
