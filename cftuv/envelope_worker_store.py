"""Память подготовок воркера пула: подготовка пересылается один раз, а не на каждом шаге ширины.

Подготовка домена alpha-независима и живёт в родителе, а воркеру на каждом нажатии уходил её пикл (на `building` 122 домена
8.65 МБ) и воркер его разворачивал (около 0.1 с стены на шаг ширины). Теперь воркер держит развёрнутые подготовки в
ограниченной по байтам памяти (`PreparationStoreV1`), а родитель шлёт ключ и пересылает пикл только там, где воркер подготовки не
держит.

КЛЮЧ — ЭТО СОДЕРЖИМОЕ. Ключ пикла (`blob_key`) — хеш байтов пикла вместе с отпечатком кода процесса (`code_identity`). Байты определяют
подготовку целиком, поэтому та же подготовка даёт тот же ключ, а другая (другая ревизия, другая суженная карта, другая плотность,
правка меша) — другой, и устаревшая запись не обслуживает ничего: она лишь вытесняется по давности. Отпечаток кода в ключе —
второй замок: воркер стартует из тех же файлов, что и родитель, но ключ процесса с другим кодом не совпадёт ни с чьим.
Подготовка, которой ключа не построить (ядро не импортируется), пересылается всегда, как раньше.

РОДИТЕЛЬ ЗНАЕТ, ЧТО ДЕРЖИТ ВОРКЕР (`PreparationLruV1` у каждого `_Worker`): те же вложения и тот же порядок вытеснения, что у воркера,
поэтому ключ без пикла уходит ровно тогда, когда воркер подготовку держит. Расхождение не теряет ответ: воркер отвечает
`needs_blob` (`PreparationMissing`), родитель забывает ключ и пересылает пикл. Перезапуск воркера — это новый воркер с пустой памятью:
все ключи для него промахи, и ничего больше. Размер записи — длина пикла (его знают обе стороны); память развёрнутых объектов
в несколько раз больше, поэтому предел (`WORKER_STORE_LIMIT_BYTES`) мал.

ПОДГОТОВКА ИЗ ПАМЯТИ ТОЖЕ САМАЯ, ЧТО И СВЕЖЕРАЗВЁРНУТАЯ. Развёрнутая подготовка не полностью неизменна: бюджет точной работы
транзакции домена (`work_budget`) тратится на покрытии и накапливается между задачами, а память точных предикатов и контактов
растёт (чистые кэши: значение от неё не зависит). Бюджет перед каждой задачей возвращается в состояние момента разворота
(`restore_budget`), поэтому задача на подготовке из памяти видит ровно то, что увидела бы на свежем разворачивании пикла; кэши
значение не меняют, и тест держит равенство ответа побитово на нескольких alpha подряд.
"""

from __future__ import annotations

import hashlib
from collections import OrderedDict

#: Сколько байтов пиклов подготовок держит воркер. Развёрнутые объекты больше пикла в несколько раз: 32 МБ пиклов держат всю подготовку
#: `building` (9.65 МБ) в одном воркере с трёхкратным запасом, а память воркера остаётся порядка сотен мегабайт.
WORKER_STORE_LIMIT_BYTES = 32 * 1024 * 1024


class PreparationMissing(LookupError):
    """Воркер не держит подготовку с этим ключом (вытеснена либо воркер новый): родитель пересылает пикл."""


class PreparationLruV1:
    """Ключи и размеры записей в порядке давности: ровно та арифметика вытеснения, что у воркера и у зеркала родителя.

    Запись крупнее предела не хранится вовсе (`add` её молча не принимает, `__contains__` ложь) — одинаково на обоих концах.
    """

    def __init__(self, limit: int = WORKER_STORE_LIMIT_BYTES) -> None:
        self._limit = int(limit)
        self._sizes: OrderedDict[str, int] = OrderedDict()
        self._total = 0

    def __contains__(self, key: str) -> bool:
        return key in self._sizes

    def __len__(self) -> int:
        return len(self._sizes)

    @property
    def total_bytes(self) -> int:
        return self._total

    @property
    def keys(self) -> tuple:
        return tuple(self._sizes)

    def touch(self, key: str) -> bool:
        """Обращение к записи освежает давность; `False` — записи нет."""

        if key not in self._sizes:
            return False
        self._sizes.move_to_end(key)
        return True

    def add(self, key: str, size: int) -> tuple:
        """Запись (или замена) под `key`; ключи вытесненных записей, давние первыми."""

        self.discard(key)
        if size > self._limit:
            return ()
        self._sizes[key] = size
        self._total += size
        evicted = []
        while self._total > self._limit:
            old, old_size = self._sizes.popitem(last=False)
            self._total -= old_size
            evicted.append(old)
        return tuple(evicted)

    def discard(self, key: str) -> bool:
        size = self._sizes.pop(key, None)
        if size is None:
            return False
        self._total -= size
        return True

    def clear(self) -> None:
        self._sizes.clear()
        self._total = 0


def _slot_names(budget) -> tuple:
    return tuple(name for cls in reversed(type(budget).__mro__) for name in getattr(cls, "__slots__", ()))


def capture_budget(prepared):
    """Состояние бюджета точной работы подготовки (все слоты) либо `None`, если бюджета нет."""

    budget = getattr(prepared, "work_budget", None)
    if budget is None:
        return None
    return tuple(getattr(budget, name) for name in _slot_names(budget))


def restore_budget(prepared, state) -> None:
    """Возвращает бюджет подготовки в снятое `capture_budget` состояние (счётчики и стадия)."""

    budget = getattr(prepared, "work_budget", None)
    if budget is None or state is None:
        return
    for name, value in zip(_slot_names(budget), state):
        setattr(budget, name, value)


class PreparationStoreV1(PreparationLruV1):
    """Развёрнутые подготовки воркера под ключами пиклов; `get` отдаёт подготовку с бюджетом момента разворота."""

    def __init__(self, limit: int = WORKER_STORE_LIMIT_BYTES) -> None:
        super().__init__(limit)
        self._objects: dict[str, tuple] = {}

    def put(self, key: str, prepared, size: int) -> None:
        for gone in self.add(key, size):
            self._objects.pop(gone, None)
        if key in self:
            self._objects[key] = (prepared, capture_budget(prepared))
        else:
            self._objects.pop(key, None)

    def get(self, key: str):
        """Подготовка под `key` либо `PreparationMissing`; бюджет возвращён в состояние разворота."""

        if not self.touch(key):
            raise PreparationMissing(key)
        prepared, state = self._objects[key]
        restore_budget(prepared, state)
        return prepared

    def clear(self) -> None:
        super().clear()
        self._objects.clear()


#: Память ЭТОГО процесса: воркер держит одну; в родителе (последовательный путь, тесты с пулом в процессе) ею не пользуются
#: иначе как через `solve_task`.
STORE = PreparationStoreV1()


def blob_key(blob: bytes) -> str:
    """Ключ пикла подготовки: хеш байтов вместе с отпечатком кода процесса; пусто, если кода не определить (тогда без памяти)."""

    from .envelope_content_key import ContentKeyUnsupported, code_identity

    try:
        identity = "|".join(code_identity())
    except ContentKeyUnsupported:
        return ""
    return hashlib.blake2b(identity.encode("utf-8") + b"\0" + blob, digest_size=16).hexdigest()


def prepared_of(inputs, *, retain: bool = True):
    """Подготовка задачи: из присланного пикла (и в память воркера под его ключ) либо из памяти по одному ключу.

    Присланный пикл всегда разворачивается заново: память читают только задачи без пикла, поэтому путь с пиклом остаётся
    прежним в точности. Задача без ключа память не трогает.
    """

    import pickle

    if inputs.blob is None:
        return STORE.get(inputs.key)
    prepared = pickle.loads(inputs.blob)
    if retain and inputs.key:
        STORE.put(inputs.key, prepared, len(inputs.blob))
    return prepared


__all__ = (
    "PreparationLruV1",
    "PreparationMissing",
    "PreparationStoreV1",
    "STORE",
    "WORKER_STORE_LIMIT_BYTES",
    "blob_key",
    "capture_budget",
    "prepared_of",
    "restore_budget",
)
