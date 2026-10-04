"""Идентичности хоста, привязанные к ревизии источника: запись, переименование, проверка остатка.

Любая правка меша меняет ревизию источника, а все идентичности хоста несут её в себе: вершины
исходника — буквально (`host-vertex:<ревизия>:<n>`), остальные — хэшем `(вид, ревизия, части)`
(`host-v0:<вид>:<24 hex>`). Результат домена, посчитанный при одной ревизии, поэтому нельзя отдать
при другой, даже если содержимое домена то же самое: писатель меша и сварка сравнивают ссылки
вершин между доменами строкой, и домен со старой ревизией не сварился бы с соседом.

Здесь живёт ровно то, что нужно, чтобы перенести результат на новую ревизию ТОЧНО:

* `stable_token` — единственная функция хэша идентичностей хоста (её зовёт `_stable_token`
  запроса). Под `record_host_tokens` она ещё и ЗАПИСЫВАЕТ каждый выданный токен вместе с его
  выводом `(вид, ревизия, части)`: запись — это то, что делает хэш обратимым без угадывания;
* `DomainLabelingV1` — записанные токены одного домена, ревизия, при которой они выданы, id
  запроса прогона (`DecalRequestId`, который воркер получает извне готовым значением) и номер
  патча домена: шов на другом конце меша сдвигает номера всех следующих патчей, а их содержимое то же;
* `relabeled` — те же токены при другой ревизии, другом id запроса и другом номере патча: вывод
  проигрывается заново той же функцией хэша с подстановкой `старая ревизия -> новая`, `старый id
  запроса -> новый`, `старый номер патча -> новый` (первая часть токена вида из
  `PATCH_SCOPED_KINDS`) и уже переименованных токенов (в порядке записи, поэтому токен, чей вывод
  ссылается на другой токен, переименуется после него);
* `LabelMapV1` — переименование одной строки. Токен, которого нет в записи, не переименовывается
  молча: `RelabelIncomplete` (результат не переносится, домен считается заново). Исключение одно —
  запрос-вид (`REQUEST_SCOPED_KINDS`): id запроса прогона приходит в выгрузку готовой строкой,
  его переименовывает подстановка, а не запись.

Модуль не знает ни Blender, ни ядра: только `hashlib`, `json`, `re`.
"""

from __future__ import annotations

import hashlib
import json
import re
import threading
from contextlib import contextmanager
from dataclasses import dataclass

#: Вид токена, чьё значение — id ЗАПРОСА прогона (`DecalRequestId`): его строит родитель до выгрузки
#: домена и отдаёт воркеру готовой строкой, поэтому в записи домена его токена нет, а переименовывает
#: его литеральная подстановка `DomainLabelingV1.request_id`. Производные (`decal-request-density`,
#: `decal-request-stretch`) выдаёт выгрузка домена, и они в записи есть.
REQUEST_SCOPED_KINDS = frozenset({"decal-request"})

#: Виды токенов, чья ПЕРВАЯ часть — номер патча-владельца домена (так их выдаёт выгрузка снапшота).
#: Остальные виды номера патча в частях не несут (цепочки и их происхождение — по рёбрам и вершинам).
#: Полноту перечня держит тест: перенос на другой номер патча равен холодному прогону на нём.
PATCH_SCOPED_KINDS = frozenset(
    {
        "angle-certificate",
        "boundary-loop",
        "chain-record",
        "chain-source",
        "chain-use",
        "corner-relation",
        "owner-sector",
        "patch-domain",
        "source-launch",
        "terminal",
        "topological-boundary",
    }
)

_TOKEN_IN_ID = re.compile(r"host-v0:([a-z][a-z0-9-]*):([0-9a-f]{24})")
#: Остальные идентичности хоста с токеном сразу за префиксом (`host-debug-diagnostic:<токен>`): любой префикс
#: `host-<имя>:` с токеном в 24 hex за ним. `host-vertex:<ревизия>:<n>` и подобные несут ревизию и индекс, а не
#: токен (за двоеточием у них `host-source:` и 64 hex, которые под `{24}` с границей не подходят).
_DIRECT_HOST_TOKEN = re.compile(r"host-(?!v0:)([a-z][a-z0-9-]*):([0-9a-f]{24})(?![0-9a-f])")
_BARE_TOKEN = re.compile(r"(?<![0-9a-f])[0-9a-f]{24}(?![0-9a-f])")
_CHAIN_SOURCE_TAIL = re.compile(r"^chain-source:.*:([0-9a-f]{24})$")

_state = threading.local()


class RelabelIncomplete(ValueError):
    """Строка несёт идентичность хоста, которой нет в записи: переименовать её точно нельзя."""


@dataclass(frozen=True, slots=True)
class HostTokenV1:
    """Один выданный токен и его вывод: `token == hash(kind, revision, parts)`."""

    kind: str
    revision: str
    parts: tuple
    token: str


@dataclass(frozen=True, slots=True)
class DomainLabelingV1:
    """Токены хоста ОДНОГО домена в порядке выдачи; ревизия, id запроса и номер патча, при которых выданы."""

    revision: str
    request_id: str
    patch_id: int
    tokens: tuple[HostTokenV1, ...]

    def has_domain_token(self) -> bool:
        """Запись полная только если в ней есть токен самого домена (снапшот строился под записью)."""

        return any(item.kind == "patch-domain" for item in self.tokens)

    def domain_id(self) -> str:
        """`PatchDomainId` домена при ревизии этой записи (подготовка несёт именно его)."""

        for item in self.tokens:
            if item.kind == "patch-domain":
                return typed_id("patch-domain", item.token)
        raise RelabelIncomplete("the record has no token of the domain")


def token_hash(kind: str, revision: str, parts) -> str:
    """Чистый хэш токена: ровно тот, что даёт `stable_token`, без записи."""

    payload = json.dumps(
        (kind, revision, tuple(parts)),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:24]


def typed_id(kind: str, token: str) -> str:
    """Типизированная идентичность хоста: единственное место, где собирается префикс `host-v0:`."""

    return f"host-v0:{kind}:{token}"


def stable_token(kind: str, revision: str, *parts: object) -> str:
    """Токен идентичности хоста; под `record_host_tokens` ещё и записывается."""

    token = token_hash(kind, revision, parts)
    sink = getattr(_state, "sink", None)
    if sink is not None:
        sink.setdefault(token, HostTokenV1(kind, revision, tuple(parts), token))
    return token


class _TokenLog:
    def __init__(self) -> None:
        self._items: dict[str, HostTokenV1] = {}

    def labeling(self, revision: str, request_id: str, patch_id: int) -> DomainLabelingV1:
        return DomainLabelingV1(
            str(revision), str(request_id), int(patch_id), tuple(self._items.values())
        )


@contextmanager
def record_host_tokens():
    """Записывает токены, выданные в ЭТОМ потоке внутри блока: `log.labeling(ревизия, id запроса, патч)`."""

    log = _TokenLog()
    previous = getattr(_state, "sink", None)
    _state.sink = log._items
    try:
        yield log
    finally:
        _state.sink = previous


class LabelMapV1:
    """Переименование строки: ревизия буквально, токены по таблице, остаток — отказ."""

    __slots__ = (
        "revision_from",
        "revision_to",
        "request_from",
        "request_to",
        "patch_literal",
        "tokens",
        "_memo",
    )

    def __init__(
        self,
        labeling: DomainLabelingV1,
        revision_to: str,
        request_to: str,
        patch_to: int,
        tokens: dict[str, str],
    ):
        self.revision_from = labeling.revision
        self.revision_to = revision_to
        self.request_from = labeling.request_id
        self.request_to = request_to
        # Номер патча в `host-patch:<ревизия>:<n>`: `(?!\d)` не даёт `:1` задеть `:12`.
        self.patch_literal = (
            None
            if labeling.patch_id == patch_to
            else (
                re.compile(re.escape(f"host-patch:{revision_to}:{labeling.patch_id}") + r"(?!\d)"),
                f"host-patch:{revision_to}:{patch_to}",
            )
        )
        self.tokens = tokens
        self._memo: dict[str, str] = {}

    def __call__(self, value: str) -> str:
        known = self._memo.get(value)
        if known is not None:
            return known
        self._reject_unrecorded(value)
        moved = self.moved_literals(value)
        tokens = self.tokens
        if tokens:
            moved = _BARE_TOKEN.sub(lambda match: tokens.get(match.group(), match.group()), moved)
        self._memo[value] = moved
        return moved

    def moved_literals(self, value: str) -> str:
        """Ревизия, id запроса и номер патча — буквальной подстановкой (токены — таблицей, отдельно)."""

        value = value.replace(self.revision_from, self.revision_to)
        if self.request_from:
            value = value.replace(self.request_from, self.request_to)
        if self.patch_literal is not None:
            value = self.patch_literal[0].sub(self.patch_literal[1], value)
        return value

    def _reject_unrecorded(self, value: str) -> None:
        """Токен хоста в исходной строке, которого нет в записи, не переименовывается молча."""

        for kind, token in _TOKEN_IN_ID.findall(value):
            if kind not in REQUEST_SCOPED_KINDS and token not in self.tokens:
                raise RelabelIncomplete(f"host token {kind}:{token} is not in the record")
        for kind, token in _DIRECT_HOST_TOKEN.findall(value):
            if token not in self.tokens:
                raise RelabelIncomplete(f"host token host-{kind}:{token} is not in the record")
        tail = _CHAIN_SOURCE_TAIL.match(value)
        if tail is not None and tail.group(1) not in self.tokens:
            raise RelabelIncomplete(f"chain-source token {tail.group(1)} is not in the record")


def _moved_part(value, move):
    if type(value) is str:
        return move(value)
    if type(value) in (tuple, list):
        return type(value)(_moved_part(item, move) for item in value)
    return value


def relabeled(
    labeling: DomainLabelingV1, revision_to: str, request_to: str, patch_to: int
) -> tuple[DomainLabelingV1, LabelMapV1]:
    """Те же токены при ревизии `revision_to`, id запроса `request_to`, патче `patch_to`, и карта для строк.

    Вывод каждого токена проигрывается заново `token_hash` с подстановкой старой ревизии на новую,
    старого id запроса на новый, старого номера патча на новый (у видов `PATCH_SCOPED_KINDS`) и уже
    переименованных токенов; токен, чей вывод ничего из этого не содержит, остаётся тем же. Карта
    содержит ВСЕ записанные токены, поэтому токен вне записи остаётся для неё неизвестным и называется
    остатком; патчевой токен, чей первый параметр — не патч домена, — тоже отказ, а не догадка.
    """

    mapping: dict[str, str] = {}
    mapper = LabelMapV1(labeling, revision_to, request_to, patch_to, mapping)

    def move(value: str) -> str:
        moved = mapper.moved_literals(value)
        if mapping:
            moved = _BARE_TOKEN.sub(lambda match: mapping.get(match.group(), match.group()), moved)
        return moved

    items = []
    for item in labeling.tokens:
        revision = move(item.revision)
        parts = _moved_part(item.parts, move)
        if item.kind in PATCH_SCOPED_KINDS:
            if not item.parts or item.parts[0] != labeling.patch_id or type(item.parts[0]) is bool:
                raise RelabelIncomplete(f"{item.kind} token does not start with the patch of the domain")
            parts = (patch_to, *parts[1:])
        token = token_hash(item.kind, revision, parts)
        mapping[item.token] = token
        items.append(HostTokenV1(item.kind, revision, parts, token))
    return DomainLabelingV1(revision_to, request_to, int(patch_to), tuple(items)), mapper


__all__ = (
    "DomainLabelingV1",
    "HostTokenV1",
    "LabelMapV1",
    "PATCH_SCOPED_KINDS",
    "REQUEST_SCOPED_KINDS",
    "RelabelIncomplete",
    "record_host_tokens",
    "relabeled",
    "stable_token",
    "token_hash",
    "typed_id",
)
