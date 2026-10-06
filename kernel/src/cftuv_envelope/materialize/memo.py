"""Память alpha-НЕЗАВИСИМОЙ работы материализатора: живёт вместе с подготовкой и не меняет ответа.

ЗАЧЕМ. Подготовка очереди alpha-независима, и воркер пула держит её развёрнутой между шагами ширины
(`envelope_worker_store`), но материализатор на каждом шаге заново считал то, что от alpha не зависит: таблицу
станций цепей (`stations.chain_station_table`, 0.18 с из 3.2 с CPU `building`) и карту отрезков к цепям
(`source_chain_by_span`), подъём домена на треугольники источника (`lift_surface.surface_lift_of`) и таблицу
событий alpha (`interval`). Здесь они считаются один раз на подготовку и лежат в ней же.

ЭТО НЕ ЭВРИСТИКА. Запись — значение чистой функции подготовки (и названных параметров в ключе), поэтому попадание
равно промаху побитово. Цена домена остаётся ценой домена, как у памяти резки (`clip_memo`): стадия записывает разность
шести статей бюджета (`ExactWorkBudgetV1.replay`) и записи, которые положила в память канонизации
(`replay_factorization_memory`), а попадание их воспроизводит, поэтому `EXACT_WORK_*` и все стадии после неё платят
столько же, сколько заплатили бы после настоящего счёта. Потолок бюджета остаётся авторитетом отказа: записанная цена,
не влезающая в остаток, не повторяется, и стадия считается заново. Стадия, которой нечего записать при тёплой памяти
канонизации (повторный круг `_build` после снятого стыка), в память не ходит вовсе: её цена зависит от истории круга.

ГДЕ ЖИВЁТ. Поле подготовки (`ConveyorPreparationV1.materialize_memo`), вне равенства и представления, как контакты
источников. Пикл несёт её ПУСТОЙ (`__reduce__`): ключ пикла подготовки в памяти воркера (`envelope_worker_store.blob_key`)
остаётся хешем самой подготовки и не зависит от того, что процесс успел посчитать, а воркер заполняет память на первой
задаче и держит её, пока держит подготовку. Запись ограничена числом (`MEMO_ENTRY_LIMIT`) и ключуется отпечатком
кода ядра (`clip_memo.kernel_code_identity`): подготовка, пережившая смену кода в процессе, не отдаёт записи прежнего кода.
`CFTUV_MATERIALIZE_MEMO=0` в окружении процесса выключает память (замер «до», сверка «с памятью и без»).
"""

from __future__ import annotations

import os
import threading
from contextlib import contextmanager

from ..exact_sqrt_sum import (
    factorization_memory_delta,
    factorization_memory_marker,
    replay_factorization_memory,
)
from .clip_memo import kernel_code_identity

ENVIRONMENT_SWITCH = "CFTUV_MATERIALIZE_MEMO"
#: Записей на подготовку: таблица станций, подъём домена и таблицы событий по законам укладки; запас на вариации.
MEMO_ENTRY_LIMIT = 12

_ENABLED = [os.environ.get(ENVIRONMENT_SWITCH, "1").strip().lower() not in ("0", "off", "false", "no")]
_LOCK = threading.Lock()


def memo_enabled() -> bool:
    return _ENABLED[0]


@contextmanager
def memo_disabled():
    """Материализация без памяти внутри блока (сверка «с памятью и без», замер «до»)."""

    before = _ENABLED[0]
    _ENABLED[0] = False
    try:
        yield
    finally:
        _ENABLED[0] = before


class MaterializeMemoV1:
    """Записи alpha-независимой работы ОДНОЙ подготовки: `ключ -> значение`, ключ несёт отпечаток кода ядра.

    Счётчики `hits` и `misses` — свойство процесса, а не ответ: с пиклом они не едут (копия в воркере выдавала бы
    чужие числа за свои), как у памяти контактов источников.
    """

    __slots__ = ("entries", "hits", "misses")

    def __init__(self) -> None:
        self.entries: dict = {}
        self.hits = 0
        self.misses = 0

    def __reduce__(self):
        return (type(self), ())

    def __len__(self) -> int:
        return len(self.entries)

    def remembered(self, key: tuple, compute):
        """`compute()` один раз на ключ: чистое значение без цены (подъём домена, таблица событий)."""

        if not _ENABLED[0]:
            return compute()
        full = (kernel_code_identity(), *key)
        found = self.entries.get(full)
        if found is None:
            found = compute()
            self._store(full, found)
            self.misses += 1
        else:
            self.hits += 1
        return found

    def priced(self, key: tuple, budget, compute):
        """`compute()` через память С ЦЕНОЙ: попадание воспроизводит записанные статьи бюджета и записи памяти канонизации.

        Память записывает только промах, посчитанный при ХОЛОДНОЙ памяти канонизации и без траты на вызов, который
        вызывающий проверил сам (`cold`); здесь цена записывается как разность до и после и повторяется как есть.
        """

        if not _ENABLED[0]:
            return compute()
        full = (kernel_code_identity(), *key)
        found = self.entries.get(full)
        if found is not None:
            value, spent, memory = found
            if budget.replay(spent):
                replay_factorization_memory(memory)
                self.hits += 1
                return value
        marker = factorization_memory_marker()
        before = budget.spent_by_article()
        value = compute()
        spent = tuple(after - was for after, was in zip(budget.spent_by_article(), before))
        self._store(full, (value, spent, factorization_memory_delta(marker)))
        self.misses += 1
        return value

    def _store(self, key: tuple, entry) -> None:
        with _LOCK:
            if key in self.entries:
                return
            while len(self.entries) >= MEMO_ENTRY_LIMIT:
                del self.entries[next(iter(self.entries))]
            self.entries[key] = entry


def memo_of(prepared) -> MaterializeMemoV1 | None:
    """Память подготовки либо `None` (подготовка без неё: собрана мимо `prepare_conveyor` либо пикл старого кода)."""

    return getattr(prepared, "materialize_memo", None)
