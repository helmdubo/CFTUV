# Установка в Blender для тестирования

Аддон состоит из **двух частей**, и это важно: их кладут в разные места.

| Часть | Что это | Куда |
|---|---|---|
| `cftuv/` | сам аддон (панели, операторы, анализ, solve, decals) | папка аддонов Blender |
| `kernel/src/cftuv_envelope/` | Blender-free envelope-ядро | пакеты Python внутри Blender |

Ядро **нельзя** класть внутрь `cftuv/` — оно намеренно ничего не знает про
Blender, и это проверяется тестом `kernel/tests/test_isolation.py`.

---

## Шаг 1 — определить пути

В Blender: `Scripting` → консоль Python:

```python
import bpy, sys, site
print("аддоны:", bpy.utils.user_resource('SCRIPTS', path="addons", create=True))
print("python :", sys.executable)
print("site   :", site.getsitepackages())
```

Типично для Windows:

```text
аддоны: C:\Users\<вы>\AppData\Roaming\Blender Foundation\Blender\4.3\scripts\addons
python : C:\Program Files\Blender Foundation\Blender 4.3\4.3\python\bin\python.exe
```

---

## Шаг 2 — зависимости

Из обычного PowerShell, подставив путь из шага 1:

```powershell
$py = "C:\Program Files\Blender Foundation\Blender 4.3\4.3\python\bin\python.exe"
& $py -m pip install "sympy==1.14.0" "pyvoronoi>=1.2.8"
```

- `sympy` — нужен envelope-ядру (и тянет за собой `mpmath`, на котором работает
  интервальный фильтр знака).
- `pyvoronoi` — нужен только легаси-бэкенду Decal Seams. Если его нет, декали
  дают именованный отказ `PYVORONOI_UNAVAILABLE`, всё остальное работает.

Если pip ругается на права — запустите PowerShell от администратора либо
добавьте `--user`.

---

## Шаг 3 — скопировать аддон

Скопировать **папку** `cftuv/` целиком в папку аддонов:

```text
<папка аддонов>\cftuv\
    __init__.py
    operators.py
    model.py
    analysis*.py
    solve*.py
    frontier_*.py
    decal*.py
    envelope_*.py
    debug.py
    ...
```

Не нужны и копировать не надо: `tests/`, `kernel/tests/`, `docs/`,
`artifacts/`, `tools/`, `research/`, `*.md`.

---

## Шаг 4 — скопировать ядро

Скопировать **папку** `kernel/src/cftuv_envelope/` в `site-packages` Python'а
Blender (путь из шага 1):

```text
<...>\4.3\python\lib\site-packages\cftuv_envelope\
```

Альтернатива, если не хочется копировать при каждом изменении — поставить
ядро как пакет, ссылающийся на рабочую копию:

```powershell
& $py -m pip install -e "<путь к репозиторию>\kernel"
```

Тогда правки в `kernel/src/` подхватываются без повторного копирования.

---

## Шаг 5 — включить и проверить

`Edit` → `Preferences` → `Add-ons` → найти **CFTUV** → включить.

Проверка в консоли Python Blender:

```python
import cftuv_envelope
from cftuv_envelope.version import __version__
print("ядро:", __version__)

import cftuv
print("аддон:", cftuv.__file__)

# то, что было починено последним: ядро принимает не идеально плоские патчи
from cftuv_envelope.contracts.metric import PlanarityAdmissionLawV1
print("политики планарности:", [item.value for item in PlanarityAdmissionLawV1])
```

Ожидаемый вывод последней строки:

```text
политики планарности: ['EXACT_SOURCE_PLANE_V1', 'NEAR_PLANAR_PROJECTION_V1']
```

Если `NEAR_PLANAR_PROJECTION_V1` отсутствует — Blender подхватил старую копию
ядра из `site-packages`. Удалите её и повторите шаг 4.

---

## Обновление после моих правок

Меняется почти всегда только эти два места:

```powershell
# 1. аддон
Copy-Item -Recurse -Force "<репозиторий>\cftuv" "<папка аддонов>\"
# 2. ядро (не нужно, если ставили через pip install -e)
Copy-Item -Recurse -Force "<репозиторий>\kernel\src\cftuv_envelope" "<site-packages>\"
```

После копирования — перезапустить Blender. Перезагрузки аддона через галочку
недостаточно: Python кэширует уже импортированные модули ядра.

Удалите `__pycache__` в обеих папках, если поведение выглядит «застрявшим».

---

## Что проверять в первую очередь

Согласно `ACCEPTANCE.md`: сцена `E:\testscene.blend`, меш `building.002`,
рёбра-цепочки 1–36.

1. **Debug-режим Envelope** — отображение элементов и их coverage.
   Кнопка `Build Envelope Debug` на панели.
2. **Drag-режим** — интерактивное перетаскивание полосы.

Известное ограничение на сегодня: envelope-ядро всё ещё медленное. На
`building.002` полные полевые ворота занимали 870 с до оптимизаций; сейчас
ядро быстрее в ~2.5 раза, но реальную цифру на вашей сцене надо мерить —
у меня её нет.
