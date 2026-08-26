# `Expected token numeric literal` в VESC Tool 6.06 — диагноз

Статус: **причина найдена и воспроизведена**. QML Refloat ни при чём —
`upstream/ui.qml.in` не изменялся и изменять его не нужно.

Короткий ответ:

> VESC Tool 6.06 выдаёт `Expected token 'numeric literal'`, потому что в
> `Qt.createQmlObject()` уходит строка, состоящая **только из двух импортов,
> которые VESC Tool приписывает сам** (`import Vedder.vesc.vescinterface 1.0;`
> + `import "qrc:/mobile";` = ровно 58 символов), **без тела QML**: страница
> QML Scripting запускает не тот редактор, в котором открыт Refloat, а всегда
> первую вкладку `main`. Парсер Qt доходит до конца строки на позиции 1:59,
> видит конец файла там, где должен начинаться корневой объект, и в процедуре
> восстановления после ошибки предлагает вставить первый подходящий токен из
> своего списка — `numeric literal`.

---

## 1. Что именно парсит Qt

### 1.1 DynamicLoader

`vesc_tool/res/qml/DynamicLoader.qml`, строки 76–82 (проверено по исходнику
и по бинарнику установленного VESC Tool):

```qml
Connections {
    target: QmlUi

    function onReloadQml(str) {
        Qt.createQmlObject(str, container, "myCode");   // ← строка 80
    }
}
```

`myCode` — это третий аргумент `Qt.createQmlObject()`, «имя файла» для
сообщений об ошибках. Он относительный, поэтому Qt резолвит его к контексту
загрузчика и печатает `qrc:/res/qml/myCode`. Никакого файла `myCode` не
существует, и **строка/колонка в ошибке относятся к самой строке `str`**, а не
к `ui.qml.in`.

DynamicLoader ничего к коду не добавляет — он лишь отдаёт `str` парсеру.

### 1.2 Кто формирует `str`

`pages/pagescripting.cpp`:

```cpp
QString PageScripting::qmlToRun(bool importDir, bool prependImports)
{
    QString res = ui->mainEdit->contentAsText();       // ← ВСЕГДА первая вкладка

    if (prependImports) {
        res.prepend("import \"qrc:/mobile\";");
        res.prepend("import Vedder.vesc.vescinterface 1.0;");
    }

    if (importDir) {
        QFileInfo f(mDirNow);
        if (f.exists() && f.isDir()) {
            res.prepend("import \"file:/" + mDirNow + "\";");
        }
    }

    return res;
}
```

`on_runButton_clicked()` → `emit reloadQml(qmlToRun())` → `onReloadQml(str)` →
`Qt.createQmlObject(str, …)`.

Два `prepend()` приписываются **без перевода строки**, поэтому всё это
оказывается в строке 1 результата.

Все три литерала найдены в бинарнике установленного VESC Tool
(`/Applications/VESC Tool.app/Contents/MacOS/VESC Tool`) — сборка совпадает с
разбираемым исходником.

### 1.3 Арифметика колонок

```
колонка:  1                                  37 38                 58 59
          |                                   | |                   | |
          import Vedder.vesc.vescinterface 1.0;import "qrc:/mobile";<здесь начинается тело>
          \_________________ 37 символов ____/\____ 21 символ _____/
```

* `import Vedder.vesc.vescinterface 1.0;` — 37 символов, колонки 1…37
* `import "qrc:/mobile";` — 21 символ, колонки 38…58
* **колонка 59 — первый символ содержимого редактора**

Отсюда сразу два вывода:

1. Ошибка указывает не внутрь Refloat, а на **самое начало того, что VESC Tool
   взял из редактора**.
2. Третий `prepend` (`import "file:/…";`) **не сработал** — иначе колонки
   сдвинулись бы на `16 + len(mDirNow)`. `mDirNow` пуст ровно тогда, когда в
   первой вкладке не открыт ни один существующий файл
   (`pagescripting.cpp:76-85`).

### 1.4 Точная строка около 1:59

Ровно та строка, на которой Qt споткнулся (вывод `tools/qml_parse_probe.py`):

```
--- строка 1 (58 символов):
import Vedder.vesc.vescinterface 1.0;import "qrc:/mobile";
--- линейка колонок 41..80:
             5         6         7         8
    1234567890123456789012345678901234567890
    ort "qrc:/mobile";
    col 50: '/'
    col 59: '<конец строки>'
    col 70: '<конец строки>'
--- ошибок парсера: 1
    1:59: Expected token `numeric literal'
```

Колонка 59 — это конец данных. Тела QML за импортами нет.

### 1.5 Почему именно «numeric literal»

Это сообщение не значит «нужна версия импорта». Оно приходит из процедуры
восстановления парсера (`qtdeclarative/src/qml/parser/qqmljs.g`, конец
`Parser::parse`): парсер по очереди пробует вставить недостающий токен из
фиксированного списка

```c
T_PLUS, T_EQ, T_COMMA, T_COLON, T_SEMICOLON,
T_RPAREN, T_RBRACKET, T_RBRACE,
T_NUMERIC_LITERAL, T_IDENTIFIER,
T_LPAREN, T_LBRACKET, T_LBRACE
```

и печатает `Expected token '<первый подошедший>'`. Там, где не хватает
операнда/корневого объекта, первым подходит `T_NUMERIC_LITERAL` → «numeric
literal». То есть сообщение означает просто **«здесь чего-то нет»**.

Отдельная проверка: настоящая нехватка версии в импорте даёт совсем другой
текст — `Library import requires a version` (проверено на Qt 5.15).

---

## 2. Минимальный воспроизводитель

```bash
python3 tools/qml_parse_probe.py --text "" --prepend-imports
# 1:59: Expected token `numeric literal'
```

То есть в терминах QML:

```qml
import Vedder.vesc.vescinterface 1.0;import "qrc:/mobile";
```

— и всё. 58 символов, никакого корневого объекта. Это полный и достаточный
reproducer; ни одна строка Refloat в нём не участвует.

Эквивалент в живом VESC Tool: открыть QML Scripting, оставить первую вкладку
`main` пустой, нажать **Run**.

---

## 3. Проверка того, что реально отдаёт FloatCore

Путь `ui.qml.in → gen_qml.py → COMM_GET_QML_UI_APP → qUncompress → QString`
проверен целиком, инструментом `tools/qml_dump_delivered.py` (только чтение,
блоб снимается с работающего `build/floatcore_host` по TCP тем же протоколом,
которым его берёт VESC Tool):

```
  [PASS] объявленный размер совпадает с принятым
  [PASS] побайтно равен ожидаемому generated QML
  [PASS] нет BOM
  [PASS] нет CR (только LF)
  [PASS] нет незакрытых подстановок {{...}}
  [PASS] декодируется как UTF-8

  сжатый blob 51339 B -> 289020 B, 7939 строк
  строка 1: '// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>'
```

Сохранённый блоб: `build/debug/refloat_qml_delivered.qml` (+ `.qml.blob` —
сжатый вид, как он идёт по проводу).

Разобрано по пунктам ТЗ:

| Проверка | Результат |
|---|---|
| незакрытые `{{PACKAGE_NAME}}` / `{{VERSION}}` | нет, подставлены `Refloat` / `1.3.0` |
| повреждённые версии в import | нет, побайтное совпадение с ожидаемым |
| лишние escape-последовательности | нет |
| BOM | нет |
| CR/LF | только LF, 7939 строк |
| случайное склеивание строк | нет (склеивание делает VESC Tool, см. §1.2) |
| подстановка чисел / десятичная запятая локали | генератор чисел не форматирует, подстановки только строковые (`"1.3.0"` внутри кавычек) |
| изменение текста при генерации | нет: `gen_qml.py` делает только две строковые замены + `zlib`; минификация (`rjsmin.py` upstream) **не применяется** |

И главное — **этот QML успешно парсится настоящим парсером Qt 5.15**:

```bash
python3 tools/qml_parse_probe.py build/debug/refloat_qml_delivered.qml --prepend-imports
# 1:38: "qrc:/mobile": no such directory
```

Единственная диагностика — артефакт стенда: каталог `qrc:/mobile` есть только
внутри ресурсов VESC Tool. Важно, что это **ошибка резолвинга импорта, а не
разбора**: Qt выдаёт её уже после того, как весь документ (289 020 байт,
7939 строк) разобран без единой синтаксической ошибки. Для контроля: если в
тот же документ подсунуть BOM после импортов, стенд честно ловит синтаксис —
`1:59: Expected token 'numeric literal'`.

---

## 4. Импорты и совместимость с Qt в VESC Tool 6.06

Установленный VESC Tool собран на **Qt 5.15.17**
(`Contents/Frameworks/QtCore.framework`). Модули из ресурсов приложения
(`Contents/Resources/qml/*/qmldir`):

| Импорт Refloat | Модуль в бандле | Потолок версии в Qt 5.15 | Вердикт |
|---|---|---|---|
| `Qt.labs.settings 1.0` | `Qt/labs/settings` | 1.1 | ✅ |
| `QtQuick 2.15` | `QtQuick.2` | 2.15 | ✅ |
| `QtQuick.Controls 2.15` | `QtQuick/Controls.2` | 2.15 | ✅ |
| `QtQuick.Layouts 1.15` | `QtQuick/Layouts` | 1.15 | ✅ |
| `QtQuick.Dialogs 1.3 as Dl` | `QtQuick/Dialogs` | 1.3 | ✅ |
| `QtQuick.Window 2.2` | `QtQuick/Window.2` | 2.15 | ✅ |
| `QtGraphicalEffects 1.15` | `QtGraphicalEffects` | 1.15 | ✅ |
| `QtQuick.Controls.Material 2.2` | `QtQuick/Controls.2/Material` | 2.15 | ✅ |
| `Vedder.vesc.vescinterface 1.0` | регистрируется в C++ | 1.0 | ✅ |
| `Vedder.vesc.codeloader 1.0` | регистрируется в C++ | 1.0 | ✅ |
| `Vedder.vesc.commands 1.0` | регистрируется в C++ | 1.0 | ✅ |
| `Vedder.vesc.configparams 1.0` | регистрируется в C++ | 1.0 | ✅ |
| `Vedder.vesc.utility 1.0` | регистрируется в C++ | 1.0 | ✅ |

Наличие регистрации специально не принималось за доказательство поддержки
версии: версии Qt-модулей взяты из потолка Qt 5.15, а не из факта наличия
каталога; каждый импорт (кроме `Vedder.*`, которых нет вне VESC Tool)
дополнительно прогнан на реальном движке Qt 5.15 по одному —
`Qt.labs.settings 1.0`, `QtQuick 2.15`, `QtQuick.Controls 2.15`,
`QtQuick.Layouts 1.15`, `QtQuick.Window 2.2`, `QtGraphicalEffects 1.15`,
`QtQuick.Controls.Material 2.2` резолвятся без ошибок.
`QtQuick.Dialogs 1.3` на стенде подвесил offscreen-процесс при загрузке
плагина (известная особенность виджетного плагина диалогов вне GUI) — версия
1.3 при этом единственная, которую предоставляет Qt 5.15, и именно её
объявляет `qmldir` в бандле VESC Tool.

Ни один импорт причиной ошибки не является — и не может ею быть: разбор
падает на колонке 59 строки 1, то есть **до** первого импорта Refloat.

---

## 5. Затронут ли обычный загрузчик приложения

Нет. Пути принципиально разные:

| | QML Scripting | Вкладка приложения |
|---|---|---|
| Где | `pages/pagescripting.cpp` + `res/qml/DynamicLoader.qml:80` | `res/qml/WelcomeQmlPanel.qml:380` |
| Вызов | `Qt.createQmlObject(str, container, "myCode")` | `Qt.createQmlObject(VescIf.qmlApp(), uiApp, "AppUi")` |
| Источник кода | текст **первой вкладки редактора** | напрямую `VescInterface::qmlApp()` |
| Приписываются импорты | **да**, 2–3 штуки, без перевода строки | **нет**, строка идёт как есть |

Обычный путь берёт QML прямо из `VescIf.qmlApp()` и ничего к нему не
дописывает, так что ни «строки 1 колонки 59», ни этой ошибки там появиться не
может. Доказательство обратного направления: имя в сообщении — `myCode`,
а его задаёт только DynamicLoader; у штатного загрузчика оно `AppUi`.

Вывод по классификации ТЗ §7: **вариант C** — особенность страницы QML
Scripting (плюс её ловушка с вкладками), не A и не B.

---

## 6. Рекомендуемое минимальное исправление

Со стороны FloatCore/Refloat исправлять нечего — менять `ui.qml.in` не нужно.

Порядок действий в VESC Tool:

1. Открыть **QML Scripting**.
2. Убедиться, что **первая вкладка (`main`) пуста** — именно её и только её
   запускает Run.
3. Нажать **Open Qmlui App**. Если `main` пуста, VESC Tool положит QML именно
   в неё и переименует вкладку в `VESC App`
   (`pagescripting.cpp:787-799`); если в `main` уже что-то есть — исходник
   уйдёт в **новую** вкладку, будет корректно отображаться, но Run по-прежнему
   будет запускать пустую `main`. Это и наблюдалось.
4. Нажать **Run**.

Дополнительно: если в настройках снят флажок «загружать содержимое
редактора» (`scripting/uploadContentEditor`), `contentAsText()` читает не
редактор, а файл с диска (`widgets/scripteditor.cpp:131`) — флажок должен быть
включён.

Отдельно стоит понимать: полноценно Refloat в QML Scripting всё равно не
заработает — DynamicLoader кладёт объект в `container`, у которого нет
`confCustomLoader` и прочего окружения мобильной оболочки. Штатное место
интерфейса — вкладка приложения (`WelcomeQmlPanel`), и её проверять надо
отдельно.

---

## 7. Можно ли починить вне upstream Refloat

Да — чинить нечего ни внутри, ни снаружи: и `ui.qml.in`, и генерация, и
доставка блоба проверены и корректны. Проблема целиком на стороне сценария
работы со страницей QML Scripting в VESC Tool.

Единственное возможное косметическое изменение вне upstream — не отдавать
комментарий-шапку в блобе (минификация, как в upstream `Makefile` через
`rjsmin.py`), но к этой ошибке оно отношения не имеет: с шапкой документ
парсится чисто (§3).

---

## 8. Инструменты диагностики

Оба добавлены этим этапом, upstream не трогают:

* `tools/qml_dump_delivered.py` — снимает блоб через `COMM_GET_QML_UI_APP` с
  работающего `build/floatcore_host`, распаковывает, сравнивает с ожидаемым
  generated QML, проверяет BOM/CR/подстановки/UTF-8.
* `tools/qml_parse_probe.py` — гоняет строку через настоящий парсер QML
  Qt 5.15 (PyQt5), повторяя `qmlToRun()`: `--prepend-imports`,
  `--import-dir DIR`, `--text`. Печатает строку 1 с линейкой колонок 41…80 и
  все диагностики парсера с line:column.

```bash
# стенд
python3 -m venv /tmp/qtvenv && /tmp/qtvenv/bin/pip install PyQt5

# снять то, что реально уходит в VESC Tool
build/floatcore_host --port 65211 &
python3 tools/qml_dump_delivered.py --port 65211

# воспроизвести ошибку
QT_QPA_PLATFORM=offscreen /tmp/qtvenv/bin/python tools/qml_parse_probe.py \
    --text "" --prepend-imports

# убедиться, что сам QML Refloat разбирается чисто
QT_QPA_PLATFORM=offscreen /tmp/qtvenv/bin/python tools/qml_parse_probe.py \
    build/debug/refloat_qml_delivered.qml --prepend-imports
```

## 9. Что осталось непроверенным

Точное состояние первой вкладки в момент нажатия Run видел только человек за
экраном; воспроизведено и доказано другое — что **любая** строка, в которой
после двух приписанных импортов нет тела QML, даёт ровно
`qrc:/res/qml/myCode:1:59: Expected token 'numeric literal'`, и что QML,
который FloatCore отдаёт, к этой ошибке отношения не имеет. Подтверждающий
шаг за человеком: положить исходник в первую вкладку и нажать Run — ошибка
разбора обязана исчезнуть.
