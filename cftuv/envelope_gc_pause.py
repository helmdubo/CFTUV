"""Циклический сборщик мусора Python стоит на паузе на время кнопки (GC_PAUSED_DURING_BUTTON_V1).

На холодном нажатии `cover.008` (1051 домен) сборщик даёт ~1.6-1.8 с пауз в родителе: он в это время строит и принимает сотни тысяч долгоживущих
объектов (снапшоты, подготовки, результаты), и каждое поколение-2 обходит их заново. Воркеры пула дают ещё ~16 процессорных секунд пауз на восьмерых
(`envelope_domain_pool.serve_task`: пауза держится на одной задаче воркера). Значения GC не меняет вовсе: он только возвращает
память циклов. Поэтому `gc_paused` отключает его на время блока и возвращает ПРЕЖНЕЕ состояние в `finally`: сборщик, выключенный до
входа, остаётся выключенным, вложенный блок не включает его раньше внешнего, исключение состояние не теряет. Принудительного `gc.collect()`
в конце нет: после блока сборщик просто работает как раньше.

Исход назван один: `GC_PAUSED_DURING_BUTTON_V1` - ответ кнопки от паузы не зависит (проверено равенством ответа в поле), меняется только
время и пик памяти процесса, пока кнопка идёт.
"""

from __future__ import annotations

import functools
import gc
from contextlib import contextmanager

GC_PAUSE_OUTCOME = "GC_PAUSED_DURING_BUTTON_V1"


@contextmanager
def gc_paused():
    """Отключает циклический сборщик на время блока; возвращает прежнее состояние (`True` - был включён) и восстанавливает его в `finally`."""

    was_enabled = gc.isenabled()
    if was_enabled:
        gc.disable()
    try:
        yield was_enabled
    finally:
        if was_enabled:
            gc.enable()


def gc_paused_call(function):
    """Декоратор метода оператора `(self, context)`: вызов идёт под `gc_paused`; имя, `__wrapped__` и ДВА позиционных аргумента сохранены.

    Подпись `(self, context)` обязательна, а не вкус: Blender при регистрации класса проверяет число аргументов `execute` по коду функции, и обёртка
    `(*args, **kwargs)` отказала бы `ValueError: expected Operator ... "execute" function to have 2 args, found 0`.
    """

    @functools.wraps(function)
    def wrapper(self, context):
        with gc_paused():
            return function(self, context)

    return wrapper


__all__ = ("GC_PAUSE_OUTCOME", "gc_paused", "gc_paused_call")
