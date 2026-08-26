#!/usr/bin/env python3
"""Диагностика QML: прогоняет строку через реальный парсер Qt 5.15 (PyQt5),
воспроизводя то, что делает VESC Tool: Qt.createQmlObject(qmlToRun(), ..., "myCode").

Только диагностика, upstream не трогает.
"""
import argparse, sys
from PyQt5.QtCore import QUrl, QCoreApplication
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtQml import QQmlEngine, QQmlComponent


def parse(qml: str, url="qrc:/res/qml/myCode"):
    eng = QQmlEngine()
    comp = QQmlComponent(eng)
    comp.setData(qml.encode("utf-8"), QUrl(url))
    return [(e.line(), e.column(), e.description()) for e in comp.errors()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", nargs="?")
    ap.add_argument("--text")
    ap.add_argument("--prepend-imports", action="store_true",
                    help="как PageScripting::qmlToRun(prependImports=true)")
    ap.add_argument("--import-dir", default=None,
                    help="как qmlToRun(importDir=true): каталог для import \"file:/DIR\"")
    a = ap.parse_args()

    src = a.text if a.text is not None else open(a.file, encoding="utf-8").read()
    if a.prepend_imports:
        src = 'import "qrc:/mobile";' + src
        src = "import Vedder.vesc.vescinterface 1.0;" + src
    if a.import_dir:
        src = f'import "file:/{a.import_dir}";' + src

    app = QGuiApplication(sys.argv[:1])
    errs = parse(src)
    line1 = src.split("\n", 1)[0]
    print(f"--- строка 1 ({len(line1)} символов):")
    print(line1[:300])
    print("--- линейка колонок 41..80:")
    print("    " + "".join(str((c // 10) % 10) if c % 10 == 0 else " " for c in range(41, 81)))
    print("    " + "".join(str(c % 10) for c in range(41, 81)))
    print("    " + line1[40:80])
    for c in (50, 59, 70):
        ch = line1[c - 1] if c - 1 < len(line1) else "<конец строки>"
        print(f"    col {c}: {ch!r}")
    print(f"--- ошибок парсера: {len(errs)}")
    for ln, col, d in errs:
        print(f"    {ln}:{col}: {d}")
    return 1 if errs else 0


if __name__ == "__main__":
    sys.exit(main())
