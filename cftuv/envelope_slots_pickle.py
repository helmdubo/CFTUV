"""Пикл записей ядра без `dataclasses.fields` на каждом объекте (SLOTS_DATACLASS_PICKLE_V1).

ЗАЧЕМ. Замороженный slots-датакласс пиклится питоновским `dataclasses._dataclass_getstate` (`[getattr(self, f.name) for f in fields(self)]`) на КАЖДОЙ записи.
На холодной кнопке `cover.008` пикл подготовок воркеров (`_packed_cold_reply`, 1053 пикла, 66 МБ) стоил 4.6 с процессора, из них ~70 % - этот `getstate`;
кадры ответов и кадры задач платят то же.

ЧТО ЗДЕСЬ. `dumps` - пикл протокола 5 со своей таблицей сводок (`Pickler.dispatch_table`): класс, который пиклится ровно парой `_dataclass_getstate`/`_dataclass_setstate`
(свои `__reduce__`/`__reduce_ex__` не заданы), сводится к `(copyreg.__newobj__, (класс,), кортеж значений полей)`: значения снимает `attrgetter` (C), имена полей берутся
из `dataclasses.fields` ОДИН раз на класс. Разбор - обычный `pickle.loads`: состояние ставит тот же `_dataclass_setstate` класса (кортеж вместо списка), поэтому кадр
не длиннее прежнего, и решения по байтам пересылки (`_worth_the_pool`) те же. Таблица выводится из самих классов по первому объекту (`__missing__`), а не из списка:
новый класс ядра получает тот же путь, забыть его некому. Прочие типы сводятся так же, как без таблицы: глобальная `copyreg.dispatch_table`, иначе `__reduce_ex__(5)`
объекта (`methodcaller`, C); классы-объекты (`type` и метаклассы) идут обычной ссылкой (`KeyError` таблицы).

ЧТО ОСТАЁТСЯ ТЕМ ЖЕ. Разобранный граф - тот же (те же классы, значения и разделение объектов: `tests/test_envelope_slots_pickle.py`); байты другие, но одни и те же
для одного графа в любом процессе: сводка класса не зависит от того, какие классы процесс уже видел. Ключ пикла подготовки (`blob_key`) поэтому детерминирован, а
все пиклы подготовок, на которых держится память воркера (`_packed_cold_reply`, `PreparationBlobsV1`), снимает этот же `dumps`. Классы не тронуты: упаковка батча
(`graph_pack.rs`, COMMIT_BATCH_PACK_V1) узнаёт их по той же паре.
"""

from __future__ import annotations

import copyreg
import dataclasses
import io
import pickle
from operator import attrgetter, methodcaller

PICKLE_PROTOCOL = 5
_GETSTATE = getattr(dataclasses, "_dataclass_getstate", None)
_SETSTATE = getattr(dataclasses, "_dataclass_setstate", None)
_NEWOBJ = copyreg.__newobj__
_DEFAULT = methodcaller("__reduce_ex__", PICKLE_PROTOCOL)


def carried(cls) -> bool:
    """Класс пиклится парой `_dataclass_getstate`/`_dataclass_setstate` и ничем своим: его сводит `dumps`."""

    return (
        _GETSTATE is not None
        and getattr(cls, "__getstate__", None) is _GETSTATE
        and getattr(cls, "__setstate__", None) is _SETSTATE
        and cls.__reduce_ex__ is object.__reduce_ex__
        and cls.__reduce__ is object.__reduce__
        and dataclasses.is_dataclass(cls)
    )


def _reducer(cls):
    """Сводка класса: значения полей в порядке `dataclasses.fields` - том же, что у `_dataclass_getstate` (его разбирает `_dataclass_setstate`)."""

    names = tuple(item.name for item in dataclasses.fields(cls))
    if len(names) > 1:
        values = attrgetter(*names)
    elif names:
        values = (lambda name: lambda obj: (getattr(obj, name),))(names[0])
    else:
        values = lambda obj: ()  # noqa: E731
    head = (cls,)

    def reduce(obj):
        return _NEWOBJ, head, values(obj)

    return reduce


class _Table(dict):
    """Сводки по классу: выводятся по первому объекту класса и запоминаются (кроме глобальной `copyreg` и классов-объектов)."""

    def __missing__(self, cls):
        registered = copyreg.dispatch_table.get(cls)
        if registered is not None:
            return registered
        if issubclass(cls, type):
            raise KeyError(cls)  # класс-объект (метакласс): пикл кладёт ссылку на класс, как без таблицы
        found = self[cls] = _reducer(cls) if carried(cls) else _DEFAULT
        return found


TABLE = _Table()


class _Pickler(pickle.Pickler):
    dispatch_table = TABLE


def dumps(value) -> bytes:
    """Пикл протокола 5 со сводкой записей ядра (`TABLE`); разбирается обычным `pickle.loads`."""

    buffer = io.BytesIO()
    _Pickler(buffer, protocol=PICKLE_PROTOCOL).dump(value)
    return buffer.getvalue()


__all__ = ("PICKLE_PROTOCOL", "TABLE", "carried", "dumps")
