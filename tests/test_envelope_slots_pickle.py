"""Пикл записей ядра сводкой класса (SLOTS_DATACLASS_PICKLE_V1, `cftuv/envelope_slots_pickle.py`).

Утверждения:

1. ТОТ ЖЕ ГРАФ: подготовка и результат настоящего холодного домена, разобранные из `dumps`, - тот же граф, что разобранный из обычного пикла (байты обычного пикла равны);
2. КАЖДЫЙ КЛАСС: каждый замороженный slots-датакласс, который поднимает воркер (`load_worker_modules`), сводится `dumps` и возвращается с теми же значениями полей;
3. БАЙТЫ ДЕТЕРМИНИРОВАНЫ: `dumps` одного графа - одни байты, и они не зависят от того, какие классы таблица уже видела (ключ пикла подготовки `blob_key` держится на них);
4. ПРОЧЕЕ КАК ПРЕЖДЕ: тип, который не сводится (свой `__getstate__`/`__reduce__`, перечисление, дробь, `copyreg`, класс-объект, не замороженный датакласс), пиклится теми же байтами, что без таблицы.
"""

from __future__ import annotations

import dataclasses
import enum
import pickle
import sys
from decimal import Decimal
from fractions import Fraction
from pathlib import Path

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_slots_pickle as slots  # noqa: E402
from cftuv.envelope_domain_pool import DomainTaskResultV1, load_worker_modules  # noqa: E402
from content_fixtures import cold_domain  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5


@dataclasses.dataclass(frozen=True, slots=True)
class _Pair:
    left: object
    right: object = None


@dataclasses.dataclass(frozen=True, slots=True)
class _One:
    only: object


@dataclasses.dataclass(frozen=True, slots=True)
class _Empty:
    pass


@dataclasses.dataclass(frozen=True, slots=True)
class _Triple(_Pair):
    third: object = None


@dataclasses.dataclass(slots=True)
class _Mutable:
    value: object


class _Kind(enum.Enum):
    A = "a"
    B = "b"


def _graph(value) -> bytes:
    """Байты обычного пикла разобранного графа: равны - графы равны (классы, значения, разделение объектов)."""

    return pickle.dumps(value, protocol=5)


def test_a_real_cold_domain_comes_back_as_the_same_graph():
    production, prepared, _revision, _request = cold_domain(quad_row_bundle(ROW), 2, frozenset(range(ROW)))
    for value in (prepared, production, DomainTaskResultV1(3, production=production)):
        assert _graph(pickle.loads(slots.dumps(value))) == _graph(pickle.loads(pickle.dumps(value, protocol=5)))
        assert slots.dumps(value) == slots.dumps(value)
        assert len(slots.dumps(value)) <= len(pickle.dumps(value, protocol=5))  # кадр не длиннее: решения по байтам пересылки (`_worth_the_pool`) те же


def test_the_bytes_do_not_depend_on_what_the_table_has_seen():
    _production, prepared, _revision, _request = cold_domain(quad_row_bundle(ROW), 2, frozenset(range(ROW)))
    saved = dict(slots.TABLE)
    try:
        slots.TABLE.clear()
        first = slots.dumps(prepared)
        slots.TABLE.clear()
        slots.dumps((_Pair(1, 2), _Kind.A, Fraction(1, 3)))
        assert slots.dumps(prepared) == first
    finally:
        slots.TABLE.clear()
        slots.TABLE.update(saved)


def _loaded_classes():
    load_worker_modules()
    seen = []
    for module in list(sys.modules.values()):
        for value in list(vars(module).values()) if module is not None else ():
            if isinstance(value, type) and value.__module__ == getattr(module, "__name__", None) and slots.carried(value) and value not in seen:
                seen.append(value)
    return seen


def test_every_frozen_slots_dataclass_the_worker_loads_is_reduced_and_comes_back_with_its_fields():
    classes = _loaded_classes()
    kernel = [cls for cls in classes if cls.__module__.startswith("cftuv_envelope")]
    assert len(kernel) > 50  # ядро целиком: таблица выводится из классов, а не из списка
    for cls in classes:
        names = [item.name for item in dataclasses.fields(cls)]
        obj = object.__new__(cls)
        for number, name in enumerate(names):
            object.__setattr__(obj, name, (cls.__qualname__, number))
        reduced = slots.TABLE[cls](obj)
        assert reduced[2] == tuple((cls.__qualname__, number) for number in range(len(names)))  # значения полей кортежем, без `fields` на объекте
        back = pickle.loads(slots.dumps(obj))
        assert type(back) is cls and [getattr(back, name) for name in names] == [getattr(obj, name) for name in names]


def test_small_and_inherited_records_round_trip_and_share_objects():
    shared = _One("x")
    value = (_Pair(shared, shared), _Triple(1, 2, 3), _One(None), _Empty(), [shared])
    back = pickle.loads(slots.dumps(value))
    assert back == value
    assert back[0].left is back[0].right is back[4][0]  # разделение объектов то же
    assert _graph(back) == _graph(pickle.loads(_graph(value)))


def test_what_is_not_reduced_pickles_to_the_same_bytes_as_without_the_table():
    plain = (
        _Kind.A,
        _Kind,
        _Pair,
        Fraction(2, 7),
        Decimal("1.25"),
        complex(1, 2),
        _Mutable(5),
        DomainTaskResultV1(7, error="boom"),
        frozenset({1, 2}),
        enum.Enum,
    )
    for value in plain:
        assert slots.dumps(value) == pickle.dumps(value, protocol=5), value
    assert not slots.carried(DomainTaskResultV1) and not slots.carried(_Mutable) and slots.carried(_Pair)
