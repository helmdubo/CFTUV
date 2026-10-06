"""Общие ворота нативных сверок с эталоном: пропуск с названной причиной, пока порт сверен не с тем эталоном, что перед глазами.

Нативная операция побитово равна ОДНОЙ версии ядра на Python (`cftuv_native.pin`). Пока основная ветка двигает ядро, сверка порта с ним
не имеет смысла и не должна краснить главную: модуль пропускается, причина названа (какие файлы эталона ушли от пина). Сам механизм пина
проверяется в `test_native_pin.py` и ворот не знает.
"""

from __future__ import annotations

import pytest


def skip_unless_available(extension, operation: str) -> None:
    """Пропуск модуля (`allow_module_level`) с причиной, если нативная операция не `available`; иначе возврат."""

    status = extension.native_status()[operation]
    if status != "available":
        pytest.skip(
            f"нативная операция `{operation}` сейчас `{status}`: сверять её с этим деревом ядра нечем "
            "(перенос дельты и новый пин — отдельный шаг; `python native/cftuv-python/python/cftuv_native/pin.py kernel/src/cftuv_envelope` печатает дайджесты дерева)",
            allow_module_level=True,
        )
