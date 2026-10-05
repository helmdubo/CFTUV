"""Точная память стадии резки домена: вход стадии -> её результат, адресация по СОДЕРЖИМОМУ входа.

ЗАЧЕМ. Стадия резки (`clip.cut_domain`, фазы `ClipStageV1`) — 84–91 % времени домена, а ширина декали,
переставленная в области, где покрытие уже насыщено (фронт упёрся в границы патча), меняет только UV: многоугольники,
которые получает резка, побитово те же, и резалась бы та же геометрия заново (замер `rounded_wall_noise_top` на
alpha 0.45–1.2: ход ширины 3.6–3.8 с, из них домен 2 — 3.55 с, а его геометрия не меняется; `sagging_wall` на alpha
0.85–1.3 — 2.8 с, геометрия патча 1 не меняется). Память отдаёт готовую геометрическую резку, а дешёвое (станции и `r`
новых вершин, `station_values`) считается заново: оно зависит от alpha через UV и в память не входит.

ЭТО НЕ ЭВРИСТИКА. Ключ — sha256 от КАНОНИЧЕСКОЙ записи ВСЕХ аргументов, с которыми вызвана резка (`run_clip`
принимает вход именованными аргументами и отдаёт их же стадии, поэтому аргумент, которого нет в ключе, не может дойти
до стадии), плюс всё, что стадия читает мимо аргументов: треугольники подъёма (`plane.triangles`: единственное состояние
подъёма, от которого зависят знаки, находки и подъём вершин), допуски и разрядность (`clip.clip_policy`: значения, а не
имена) и отпечаток кода ядра (`kernel_code_identity`: sha256 по всем .py пакета ядра тем же законом, что у установщика).
Запись чисел точная: целые, дроби (числитель и знаменатель), `float` через `hex`, `SqrtSumV1` — как датакласс из
канонических `terms`; множества — по отсортированным записям, словари и списки — по порядку (порядок вставки словаря
читает привязка вершин, поэтому он часть входа). Значение, которого кодировщик не знает, — `ClipKeyUnsupported`: резка
считается без памяти (`BYPASS`), а ключ с молча пропущенным значением не получается никогда.

ПОПАДАНИЕ ПОБИТОВО РАВНО ПРОМАХУ. Результат хранится пиклом (`pickle`), а не живым объектом: попавший получает свежую
копию, и никто из потребителей не может испортить запись. Побочные эффекты стадии на окружение воспроизводятся: (1) нормали
смещения новых вершин (`BoundSurfaceLiftV1.replay_lifted`: подъём пишет их по позиции, а их читают позиции хоста и
нормали батча); (2) ЦЕНА: записанная разность шести статей бюджета (`ExactWorkBudgetV1.replay`) и записи, которые стадия
положила в память канонизации (`replay_factorization_memory`), поэтому стадии после неё платят за память столько же, сколько
заплатили бы после настоящей резки. Потолок остаётся авторитетом отказа и не зависит от истории процесса: записанная цена,
которая не влезает в остаток потолка, не повторяется, и стадия считается заново (и отказывает там же и с теми же числами,
что без памяти). `EXACT_WORK_*` попавшего равны промаху, когда память канонизации до стадии та же (тот же домен, та же
alpha: `materialize_domain` каждый раз начинает с холодной памяти); при другой alpha память до резки чуть другая (UV), и
цена попавшего — цена промаха, записавшего запись: счётчики ЦЕНЫ (`EXACT_WORK_*`) ответом не являются, как и во всех
воротах. Счётчики резки (`MATERIALIZE_CLIP_*`) и всё остальное — ответ, и они равны побитово.

ГДЕ ЖИВЁТ. Один экземпляр на процесс (`MEMO`): воркер пула держит свою память, родитель — свою (последовательный путь, малая
партия). Ограничена записями и байтами (LRU) и сбрасывается целиком, когда отпечаток кода меняется. Диск не трогается: запись,
пережившая смену кода, была бы устаревшим результатом. `CFTUV_CLIP_MEMO=0` в окружении процесса выключает память (замер «до»,
сверка «с памятью и без»).
"""

from __future__ import annotations

import dataclasses
import functools
import hashlib
import os
import pickle
import threading
import time
from collections import OrderedDict
from contextlib import contextmanager
from enum import Enum
from fractions import Fraction
from pathlib import Path

from ..exact_sqrt_sum import (
    factorization_memory_delta,
    factorization_memory_marker,
    replay_factorization_memory,
)

CLIP_MEMO_SCHEMA = "cftuv.clip-memo.v1"
#: Записей в памяти процесса. Запись домена — 100–300 КБ пикла (замер на поле записан в DECISIONS); 48 записей — дюжина
#: тяжёлых доменов в нескольких ширинах.
CLIP_MEMO_ENTRY_LIMIT = 48
#: Байт пикла во всех записях процесса (запас в 80 самых больших записей): воркер пула и без того держит около 100 МБ.
CLIP_MEMO_BYTE_LIMIT = 24 << 20
ENVIRONMENT_SWITCH = "CFTUV_CLIP_MEMO"

#: Как закончился вызов: из памяти, посчитан и записан, память выключена, вход не кодируется (ключа нет).
HIT = "HIT"
MISS = "MISS"
OFF = "OFF"
BYPASS = "BYPASS"


class ClipKeyUnsupported(TypeError):
    """В входе стадии значение, которое кодировщик ключа не умеет записать ТОЧНО: стадия считается без памяти."""


@functools.lru_cache(maxsize=1)
def kernel_code_identity() -> str:
    """Отпечаток кода ядра ЭТОГО процесса: sha256 по всем .py пакета (путь и содержимое с LF), один раз за процесс.

    Тот же закон, что у установщика и у ключа содержимого хоста (`envelope_content_key.package_fingerprint`; тест хоста
    держит равенство). Пакета нет на диске (архив) — метка, и память живёт только в процессе, которому код не меняют.
    """

    root = Path(__file__).resolve().parents[1]
    if not root.is_dir():
        return "<нет каталога>"
    digest = hashlib.sha256()
    for current, directories, files in os.walk(root):
        directories[:] = sorted(item for item in directories if item != "__pycache__")
        for name in sorted(files):
            if not name.endswith(".py"):
                continue
            path = Path(current, name)
            digest.update(path.relative_to(root).as_posix().encode())
            digest.update(path.read_bytes().replace(b"\r\n", b"\n"))
    return digest.hexdigest()[:16]


_DATACLASS_FIELDS: dict[type, tuple[str, ...]] = {}


def _encode(value, out: list) -> None:
    """Дописывает в `out` каноническую запись значения: тип и значение каждого узла."""

    kind = type(value)
    if kind is int:
        out.append(f"i{value};")
    elif kind is Fraction:
        out.append(f"q{value.numerator}/{value.denominator};")
    elif kind is str:
        out.append(f"s{len(value)}:{value};")
    elif kind is bool:
        out.append("T;" if value else "F;")
    elif value is None:
        out.append("N;")
    elif kind is float:
        out.append(f"f{value.hex()};")
    elif kind is tuple or kind is list:
        out.append("(" if kind is tuple else "[")
        for item in value:
            _encode(item, out)
        out.append(")" if kind is tuple else "]")
    elif kind is frozenset or kind is set:
        parts = []
        for item in value:
            piece: list = []
            _encode(item, piece)
            parts.append("".join(piece))
        out.append("{" + "".join(sorted(parts)) + "}")
    elif kind is dict:
        out.append("<")
        for key, item in value.items():
            _encode(key, out)
            _encode(item, out)
        out.append(">")
    elif isinstance(value, Enum):
        out.append(f"e{kind.__qualname__}.{value.name};")
    elif dataclasses.is_dataclass(value) and not isinstance(value, type):
        names = _DATACLASS_FIELDS.get(kind)
        if names is None:
            names = _DATACLASS_FIELDS[kind] = tuple(item.name for item in dataclasses.fields(kind))
        out.append(f"D{kind.__qualname__}[")
        for name in names:
            out.append(f"{name}=")
            _encode(getattr(value, name), out)
        out.append("]")
    else:
        raise ClipKeyUnsupported(f"{kind.__qualname__} is not encodable in a clip memo key")


def clip_key(inputs: dict, triangles, policy) -> str:
    """Ключ памяти: sha256 от схемы, отпечатка кода, допусков, треугольников подъёма и ВСЕХ именованных аргументов стадии."""

    out: list = [CLIP_MEMO_SCHEMA, ";", kernel_code_identity(), ";"]
    _encode(policy, out)
    out.append("triangles=")
    _encode(tuple(triangles), out)
    for name, value in inputs.items():
        out.append(f"{name}=")
        _encode(value, out)
    return hashlib.sha256("".join(out).encode("utf-8")).hexdigest()


@dataclasses.dataclass(frozen=True, slots=True)
class _Entry:
    blob: bytes
    spent: tuple
    seconds: float


class ClipMemoV1:
    """Записи по ключу; вытеснение — по давности обращения; сброс целиком при смене отпечатка кода."""

    def __init__(self, entry_limit: int = CLIP_MEMO_ENTRY_LIMIT, byte_limit: int = CLIP_MEMO_BYTE_LIMIT) -> None:
        self.entry_limit = entry_limit
        self.byte_limit = byte_limit
        self.enabled = os.environ.get(ENVIRONMENT_SWITCH, "1").strip().lower() not in ("0", "off", "false", "no")
        self._entries: OrderedDict[str, _Entry] = OrderedDict()
        self._lock = threading.Lock()
        self._identity = ""
        self.bytes = 0
        self.hits = self.misses = self.bypassed = self.evictions = self.clears = 0
        #: Секунды счёта, которых попадания не стоили: сумма времени записей на момент записи (оценка, не ответ).
        self.saved_seconds = 0.0

    def __len__(self) -> int:
        return len(self._entries)

    def _bind(self) -> None:
        identity = kernel_code_identity()
        if identity != self._identity:
            if self._entries:
                self.clears += 1
            self._entries.clear()
            self.bytes = 0
            self._identity = identity

    def lookup(self, key: str) -> _Entry | None:
        with self._lock:
            self._bind()
            entry = self._entries.get(key)
            if entry is not None:
                self._entries.move_to_end(key)
            return entry

    def store(self, key: str, entry: _Entry) -> None:
        with self._lock:
            self._bind()
            if len(entry.blob) > self.byte_limit or key in self._entries:
                return
            self._entries[key] = entry
            self.bytes += len(entry.blob)
            while len(self._entries) > self.entry_limit or self.bytes > self.byte_limit:
                _key, old = self._entries.popitem(last=False)
                self.bytes -= len(old.blob)
                self.evictions += 1

    def clear(self) -> None:
        with self._lock:
            self._entries.clear()
            self.bytes = 0

    def reset_stats(self) -> None:
        self.hits = self.misses = self.bypassed = self.evictions = self.clears = 0
        self.saved_seconds = 0.0

    def snapshot(self) -> dict:
        return {
            "entries": len(self._entries),
            "bytes": self.bytes,
            "hits": self.hits,
            "misses": self.misses,
            "bypassed": self.bypassed,
            "evictions": self.evictions,
            "clears": self.clears,
            "saved_seconds": round(self.saved_seconds, 3),
        }


MEMO = ClipMemoV1()


@contextmanager
def memo_disabled():
    """Резка без памяти внутри блока (сверка «с памятью и без», замер «до»)."""

    before = MEMO.enabled
    MEMO.enabled = False
    try:
        yield
    finally:
        MEMO.enabled = before


def run_clip(compute, plane, budget, policy, **inputs):
    """`(результат compute, статус)`: резка домена через память.

    `compute(plane, budget, **inputs)` — стадия; каждый именованный аргумент входит в ключ, и стадия получает ровно их.
    """

    memo = MEMO
    if not memo.enabled:
        return compute(plane, budget, **inputs), OFF
    try:
        key = clip_key(inputs, plane.triangles, policy)
    except (ClipKeyUnsupported, AttributeError):
        memo.bypassed += 1
        return compute(plane, budget, **inputs), BYPASS
    entry = memo.lookup(key)
    if entry is not None:
        clipped, memory = pickle.loads(entry.blob)
        if budget.replay(entry.spent):
            plane.replay_lifted(clipped.lifted)
            replay_factorization_memory(memory)
            memo.hits += 1
            memo.saved_seconds += entry.seconds
            return clipped, HIT
    marker = factorization_memory_marker()
    before = budget.spent_by_article()
    started = time.perf_counter()
    clipped = compute(plane, budget, **inputs)
    seconds = time.perf_counter() - started
    spent = tuple(after - was for after, was in zip(budget.spent_by_article(), before))
    memory = factorization_memory_delta(marker)
    memo.store(key, _Entry(pickle.dumps((clipped, memory), protocol=5), spent, seconds))
    memo.misses += 1
    return clipped, MISS
