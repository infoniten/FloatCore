#!/usr/bin/env python3
"""Вывести раскладку сериализованной конфигурации VESC в версионируемый JSON.

Зачем. Смещения полей в mcconf/appconf нельзя переписывать руками: переписанная
таблица расходится с прошивкой молча, и ошибка проявится как «необъяснённое
изменение байта» в самый неподходящий момент. Поэтому таблица ВЫВОДИТСЯ из
исходника confgenerator.c той версии прошивки, что стоит на железе.

Но и зависеть от файла в /tmp нельзя: он исчезает, а вместе с ним перестаёт
работать и сверка резервных копий. Поэтому результат вывода кладётся в
репозиторий вместе с URL источника и его sha256 — маленький, читаемый глазами
и проверяемый заново одной командой.

    python3 tools/gen_vesc_layout.py --fw release_6_06     # перегенерировать
    python3 tools/gen_vesc_layout.py --fw release_6_06 --verify   # только сверить
"""
import argparse
import hashlib
import json
import os
import re
import sys
import urllib.request

SIZE = {"uint8": 1, "int8": 1, "uint16": 2, "int16": 2, "uint32": 4, "int32": 4,
        "float16": 2, "float32": 4, "float32_auto": 4, "u8": 1}

URL_TEMPLATE = "https://raw.githubusercontent.com/vedderb/bldc/{fw}/confgenerator.c"


def parse_serializer(text, func, struct_name):
    m = re.search(r"int32_t " + func + r"\(uint8_t \*buffer, const " + struct_name +
                  r" \*conf\) \{(.*?)\n\}", text, re.S)
    if not m:
        raise SystemExit(f"не найден сериализатор {func}")
    body = re.sub(r"//[^\n]*", "", m.group(1))
    off = 0
    fields = []
    for stmt in [x.strip().replace("\n", " ") for x in body.split(";")]:
        mm = re.search(r"buffer_append_(\w+)\(buffer,\s*(.*?),\s*&ind\)", stmt)
        if mm:
            kind, arg = mm.group(1), mm.group(2)
            name = re.search(r"conf->([\w\[\]\.]+)", arg)
            scale = re.search(r",\s*([\d.e+]+)\s*$", arg)
            fields.append({"name": name.group(1) if name else "SIGNATURE",
                           "offset": off, "kind": kind,
                           "scale": float(scale.group(1)) if scale else 1.0})
            off += SIZE[kind]
            continue
        mm = re.search(r"buffer\[ind\+\+\]\s*=\s*(.*)", stmt)
        if mm:
            name = re.search(r"conf->([\w\[\]\.]+)", mm.group(1))
            fields.append({"name": name.group(1) if name else "?",
                           "offset": off, "kind": "u8", "scale": 1.0})
            off += 1
    return fields, off


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fw", default="release_6_06")
    ap.add_argument("--out", default="tools/vesc_layout")
    ap.add_argument("--verify", action="store_true",
                    help="не перезаписывать, только сверить с лежащим в репозитории")
    a = ap.parse_args()

    url = URL_TEMPLATE.format(fw=a.fw)
    raw = urllib.request.urlopen(url).read()
    sha = hashlib.sha256(raw).hexdigest()
    text = raw.decode("latin1")

    mc, mc_total = parse_serializer(text, "confgenerator_serialize_mcconf", "mc_configuration")
    app, app_total = parse_serializer(text, "confgenerator_serialize_appconf", "app_configuration")

    doc = {
        "firmware": a.fw,
        "source_url": url,
        "source_sha256": sha,
        "note": ("Смещения — внутри СЕРИАЛИЗОВАННОГО блока. В ответе COMM_GET_* "
                 "им предшествует один байт с номером пакета, поэтому смещение "
                 "в ответе на единицу больше."),
        "mcconf": {"total_bytes": mc_total, "fields": mc},
        "appconf": {"total_bytes": app_total, "fields": app},
    }

    path = os.path.join(a.out, f"{a.fw}.json")
    if a.verify:
        if not os.path.exists(path):
            print(f"нет файла {path}")
            return 1
        have = json.load(open(path))
        same = (have.get("source_sha256") == sha and
                have["mcconf"]["fields"] == mc and have["appconf"]["fields"] == app)
        print(f"{path}: {'совпадает с upstream' if same else 'РАСХОЖДЕНИЕ с upstream'}")
        print(f"  sha256 в файле:  {have.get('source_sha256')}")
        print(f"  sha256 upstream: {sha}")
        return 0 if same else 1

    os.makedirs(a.out, exist_ok=True)
    with open(path, "w") as f:
        json.dump(doc, f, indent=1)
    print(f"{path}: mcconf {mc_total} Б / {len(mc)} полей, "
          f"appconf {app_total} Б / {len(app)} полей")
    print(f"  источник {url}")
    print(f"  sha256   {sha}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
