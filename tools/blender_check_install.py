"""Самодиагностика установки CFTUV внутри Blender.

Запуск: `Scripting` → открыть этот файл → `Run Script`.
Либо вставить содержимое в консоль Python Blender.

Скрипт ничего не меняет. Он отвечает на один вопрос: почему не работает.
"""

from __future__ import annotations

import importlib
import os
import sys


def _line(status: str, text: str) -> None:
    print(f"  [{status}] {text}")


def _probe(module_name: str, why: str) -> object | None:
    try:
        module = importlib.import_module(module_name)
    except Exception as error:  # noqa: BLE001 — диагностика, показываем всё
        _line("НЕТ", f"{module_name} — {type(error).__name__}: {error}")
        _line("   ", f"      нужен для: {why}")
        return None
    where = getattr(module, "__file__", "<встроенный>")
    _line("ОК ", f"{module_name}  {where}")
    return module


def main() -> None:
    print("\n=== окружение ===")
    print(f"  python: {sys.executable}")
    print(f"  версия: {sys.version.split()[0]}")
    try:
        import bpy

        print(f"  blender: {bpy.app.version_string}")
    except Exception:
        print("  blender: скрипт запущен вне Blender")

    print("\n=== зависимости ядра ===")
    sympy = _probe("sympy", "точная арифметика envelope-ядра")
    _probe("mpmath", "интервальный фильтр знака (ставится вместе с sympy)")
    if sympy is not None and getattr(sympy, "__version__", "") != "1.14.0":
        _line(
            "!!!",
            f"sympy {sympy.__version__}, ожидается 1.14.0 — "
            "дайджесты могут разойтись",
        )

    print("\n=== Blender-free ядро ===")
    kernel = _probe("cftuv_envelope", "весь envelope-конвейер")
    if kernel is not None:
        try:
            from cftuv_envelope.version import __version__

            _line("ОК ", f"версия ядра: {__version__}")
        except Exception as error:  # noqa: BLE001
            _line("НЕТ", f"версия ядра недоступна: {error}")
        try:
            from cftuv_envelope.contracts.metric import PlanarityAdmissionLawV1

            laws = [item.value for item in PlanarityAdmissionLawV1]
            _line("ОК ", f"политики планарности: {laws}")
            if "NEAR_PLANAR_PROJECTION_V1" not in laws:
                _line(
                    "!!!",
                    "NEAR_PLANAR_PROJECTION_V1 отсутствует — Blender подхватил "
                    "старую копию ядра. Удалите её и скопируйте заново.",
                )
        except Exception as error:  # noqa: BLE001
            _line("НЕТ", f"контракты метрики не читаются: {error}")

    print("\n=== аддон ===")
    addon = _probe("cftuv", "панели и операторы")
    if addon is not None:
        try:
            from cftuv.envelope_request_export import HOST_PLANARITY_POLICY

            _line("ОК ", f"политика планарности хоста: {HOST_PLANARITY_POLICY}")
        except Exception as error:  # noqa: BLE001
            _line("НЕТ", f"хост-адаптер не читается: {error}")

    print("\n=== опциональное (только легаси-декали) ===")
    _probe("pyvoronoi", "легаси-бэкенд Decal Seams; без него — именованный отказ")

    print("\n=== устаревшие кэши ===")
    stale = []
    for module_name in ("cftuv_envelope", "cftuv"):
        module = sys.modules.get(module_name)
        root = getattr(module, "__file__", None)
        if not root:
            continue
        for current, directories, _files in os.walk(os.path.dirname(root)):
            if "__pycache__" in directories:
                stale.append(os.path.join(current, "__pycache__"))
    if stale:
        _line("... ", f"найдено {len(stale)} каталогов __pycache__")
        _line("   ", "      удалите их, если поведение выглядит «застрявшим»")
    else:
        _line("ОК ", "лишних кэшей не видно")

    print("\n=== итог ===")
    if kernel is None or sympy is None:
        print("  НЕ ГОТОВО. Смотрите строки [НЕТ] выше.")
        print("  Чаще всего: sympy не установлен в Python именно ЭТОГО Blender.")
        print(f'  Лечится: "{sys.executable}" -m pip install "sympy==1.14.0"')
    elif addon is None:
        print("  Ядро на месте, аддон не найден.")
        print("  Скопируйте папку cftuv/ в каталог аддонов и включите его")
        print("  в Edit → Preferences → Add-ons.")
    else:
        print("  Всё на месте. Можно запускать Build Envelope Debug.")


main()
