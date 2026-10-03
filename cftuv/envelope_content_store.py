"""Хранилище по СОДЕРЖИМОМУ домена: подготовки и результаты переживают смену ревизии источника.

Кэши сессии (`envelope_debug_session`) держат всё под ревизией источника и сбрасываются с её
сменой: любая правка меша делала холодными ВСЕ домены. Здесь подготовка и результаты лежат под
ключом содержимого (`envelope_content_key.domain_content_key`), поэтому правка пересчитывает
только домены, чьё содержимое изменилось, а остальные берутся отсюда.

ЗАПИСЬ ХРАНИТ ВСЁ В ТОЙ ЖЕ ИДЕНТИЧНОСТИ, В КОТОРОЙ ПОСЧИТАНО. Подготовка непрозрачна: её нельзя
переименовать, не зная, что в ней зависит от идентичностей, поэтому она остаётся такой, какой её
посчитали при ревизии `labeling.revision`, и каждый результат, посчитанный на ней, получается с теми
же идентичностями; результат несёт запись своих идентичностей (`labels`) сам. Переименование делает
только `relabel_result`, и только на выходе (воркер пула, где оно параллельно, либо родитель):
идентичности хоста (ревизия, id запроса и номер патча буквально, токены по записи вывода) переписываются
на ревизию, запрос и патч прогона, а всё, что зависит от них, пересчитывается тем, чем считало ядро
(`geometry_batch_semantic_digest`, `canonical_json_bytes`, `offset_normals_digest`): батч после переноса
самосогласован (его читает `GeometryBatchCodecV1.loads` и проверяет собственный дайджест).

ЧТО ПЕРЕНОС НЕ ОБЕЩАЕТ. Ядро нумерует грани, регионы, огибающие и узлы (`node:N`) по ОТСОРТИРОВАННЫМ
именам, а имена выводятся хэшем от идентичностей хоста, то есть от ревизии: два холодных прогона ОДНОГО
содержимого при двух ревизиях различаются порядком граней, номерами `claim:N`/`region:N`/`node:N`,
хэшами `claim:envelope-instance:*`, числом операций точной арифметики и дайджестами, и это измерено, а
не предположено (`building`, 122 домена). Перенесённый результат — это ответ при первой ревизии с
переписанными идентичностями хоста, а не побитовая копия холодного прогона при новой, которой нет и между
двумя холодными: геометрия, UV, швы, нормали и числа те же, порядок граней и номера владельцев — метки
первого вычисления. Тесты держат именно это равенство (`tests/content_equivalence.py`), а не более сильное.

Модуль не знает ни Blender, ни ядра на уровне импорта: ядро берётся лениво, как у продуктового пути.
"""

from __future__ import annotations

import io
import pickle
from collections import OrderedDict
from dataclasses import dataclass, field, replace

from .envelope_host_labels import (
    DomainLabelingV1,
    LabelMapV1,
    RelabelIncomplete,
    relabeled,
)

#: Сколько доменов по содержимому держит хранилище (вытесняется давнее по обращению). Измерено на `building`
#: (122 домена): холодное нажатие — +111 МБ, каждая правка в лёгком патче — одна запись и около 1 МБ
#: (40 правок: 122 -> 203 записи, +76 МБ); 256 записей — это здание и сто с лишним правок, порядка
#: 250 МБ в худшем случае, а не гигабайт в сессии Blender.
CONTENT_STORE_ENTRY_LIMIT = 256
#: Сколько результатов (по ключу содержимого и слоту alpha/законов) держит хранилище.
CONTENT_STORE_RESULT_LIMIT = 512
_MISSING = object()


class ContentRelabelFailed(ValueError):
    """Результат нельзя перенести на ревизию точно: домен считается заново, и отказ назван."""


@dataclass(frozen=True, slots=True)
class RelabelV1:
    """Куда перенести результат: ревизия, id запроса прогона и номер патча домена в нём.

    `base` — запись идентичностей подготовки: её получает результат, только что посчитанный на
    подготовке из хранилища (у него записи ещё нет); результату, который её уже несёт, она не нужна.
    """

    revision_to: str
    request_to: str
    patch_to: int
    base: DomainLabelingV1 | None = None


@dataclass(slots=True)
class ContentEntryV1:
    """Подготовка домена, идентичности, при которых она посчитана, и результаты на ней."""

    prepared: object
    labeling: DomainLabelingV1
    results: dict = field(default_factory=dict)


class ContentStoreV1:
    """Записи по ключу содержимого; вытеснение — по давности обращения.

    `on_forget(prepared)` вызывается, когда запись уходит (чистится пикл подготовки для воркеров).
    """

    def __init__(self, on_forget=None) -> None:
        self._entries: OrderedDict[str, ContentEntryV1] = OrderedDict()
        self._recent: OrderedDict[tuple, None] = OrderedDict()
        self._by_preparation: dict[int, str] = {}
        self._result_ids: dict[int, int] = {}
        self._on_forget = on_forget

    def __len__(self) -> int:
        return len(self._entries)

    @property
    def result_count(self) -> int:
        return len(self._recent)

    def holds(self, item) -> bool:
        """Хранилище держит именно этот объект: подготовку либо результат (по ним живут их пиклы)."""

        key = self._by_preparation.get(id(item))
        entry = None if key is None else self._entries.get(key)
        if entry is not None and entry.prepared is item:
            return True
        return self._result_ids.get(id(item), 0) > 0

    def find(self, key: str) -> ContentEntryV1 | None:
        entry = self._entries.get(key)
        if entry is not None:
            self._entries.move_to_end(key)
        return entry

    def key_of(self, prepared) -> tuple[str, ContentEntryV1] | None:
        """`(ключ, запись)` подготовки, если хранилище держит именно этот объект."""

        key = self._by_preparation.get(id(prepared))
        entry = None if key is None else self._entries.get(key)
        if entry is None or entry.prepared is not prepared:
            return None
        return key, entry

    def register_preparation(self, key: str, prepared, labeling: DomainLabelingV1) -> ContentEntryV1:
        """Запись под `key`; готовая запись с тем же ключом остаётся (подготовка та же по построению)."""

        known = self._entries.get(key)
        if known is not None:
            self._entries.move_to_end(key)
            return known
        entry = ContentEntryV1(prepared, labeling)
        self._entries[key] = entry
        self._by_preparation[id(prepared)] = key
        while len(self._entries) > CONTENT_STORE_ENTRY_LIMIT:
            self._drop(next(iter(self._entries)))
        return entry

    def register_result(self, key: str, slot: tuple, result) -> None:
        """Результат домена в хранилище; без записи идентичностей (`result.labels`) не принимается.

        Результат несёт запись сам, поэтому слот держит ПЕРВЫЙ принятый ответ в любой ревизии (перенос
        на ревизию прогона идёт от записи, что лежит в результате, и перенесённые копии слот не
        занимают: возврат к ревизии, при которой слот посчитан, остаётся попаданием без переноса).
        """

        entry = self._entries.get(key)
        if entry is None or getattr(result, "labels", None) is None or slot in entry.results:
            return
        entry.results[slot] = result
        self._result_ids[id(result)] = self._result_ids.get(id(result), 0) + 1
        pair = (key, slot)
        self._recent[pair] = None
        self._recent.move_to_end(pair)
        while len(self._recent) > CONTENT_STORE_RESULT_LIMIT:
            old_key, old_slot = next(iter(self._recent))
            del self._recent[(old_key, old_slot)]
            old = self._entries.get(old_key)
            if old is not None:
                self._release_result(old.results.pop(old_slot, None))

    def _release_result(self, result) -> None:
        if result is None:
            return
        left = self._result_ids.get(id(result), 0) - 1
        if left > 0:
            self._result_ids[id(result)] = left
        else:
            self._result_ids.pop(id(result), None)
            if self._on_forget is not None:
                self._on_forget(result)

    def result(self, key: str, slot: tuple):
        entry = self._entries.get(key)
        found = None if entry is None else entry.results.get(slot)
        if found is not None:
            self._recent.move_to_end((key, slot))
        return found

    def forget(self, key: str) -> None:
        if key in self._entries:
            self._drop(key)

    def clear(self) -> None:
        for key in list(self._entries):
            self._drop(key)

    def _drop(self, key: str) -> None:
        entry = self._entries.pop(key)
        for slot, result in entry.results.items():
            self._recent.pop((key, slot), None)
            self._release_result(result)
        self._by_preparation.pop(id(entry.prepared), None)
        if self._on_forget is not None:
            self._on_forget(entry.prepared)


def _rewritten(value, move: LabelMapV1):
    """Тот же объект, у которого каждая строка, несущая идентичность хоста, переписана `move`.

    Обход делает сам `pickle`: `persistent_id` видит каждую строку объекта и подменяет переписанную
    ссылкой на таблицу, `persistent_load` отдаёт её обратно. Тип объекта при этом не нужно знать:
    датаклассы, множества и кортежи батча собираются так же, как собирались бы для пула.
    """

    strings: list[str] = []
    index: dict[str, int | None] = {}

    class _Out(pickle.Pickler):
        def persistent_id(self, item):
            if type(item) is not str:
                return None
            found = index.get(item, _MISSING)
            if found is _MISSING:
                moved = move(item)
                found = None
                if moved != item:
                    found = len(strings)
                    strings.append(moved)
                index[item] = found
            return found

    class _In(pickle.Unpickler):
        def persistent_load(self, pid):
            return strings[pid]

    buffer = io.BytesIO()
    _Out(buffer, protocol=pickle.HIGHEST_PROTOCOL).dump(value)
    return _In(io.BytesIO(buffer.getvalue())).load()


def carried_to_run(result, relabel: RelabelV1):
    """Результат домена при ревизии и запросе прогона: с записью (`base` для свежего) и перенесённый.

    Тот же код у воркера пула (там параллельно) и у родителя (пула нет либо перенос не вышел там).
    """

    if result.labels is None:
        if relabel.base is None:
            raise ContentRelabelFailed("the result has no record of its host identities")
        result = replace(result, labels=relabel.base)
    return relabel_result(result, relabel.revision_to, relabel.request_to, relabel.patch_to)


def relabel_result(result, revision_to: str, request_to: str, patch_to: int):
    """Результат домена при ревизии `revision_to`, id запроса `request_to`, патче `patch_to`: идентичности переписаны.

    Дайджесты батча пересчитаны ядром. Перенос нужен, когда меняются ревизия или номер патча; одна смена id
    запроса его не вызывает (результат несёт id запроса своего вычисления, как и прежде), а при переносе
    id запроса становится текущим.

    Результат без записи идентичностей (`labels`) и результат, в котором осталась неучтённая
    идентичность хоста, не переносятся: `ContentRelabelFailed`, домен считается заново.
    """

    labeling = result.labels
    if labeling is None:
        raise ContentRelabelFailed("the result has no record of its host identities")
    if labeling.revision == revision_to and labeling.patch_id == patch_to:
        # Тот же меш и тот же номер патча: id запроса — метка вычисления (как у результата из кэша ревизии), и
        # смена выделения в другом месте не стоит переноса.
        return result
    try:
        new_labeling, move = relabeled(labeling, revision_to, request_to, patch_to)
        moved = _rewritten(replace(result, labels=None), move)
    except RelabelIncomplete as exc:
        raise ContentRelabelFailed(str(exc)) from exc
    batch = moved.batch
    content_digest = moved.content_digest
    offset_digest = moved.offset_normals_digest
    if batch is not None:
        from hashlib import sha256

        from cftuv_envelope.canonical import canonical_json_bytes, geometry_batch_semantic_digest
        from cftuv_envelope.ids import SemanticDigestValue

        batch = replace(
            batch,
            semantic_digest=SemanticDigestValue(geometry_batch_semantic_digest(batch).sha256_hex),
        )
        content_digest = sha256(canonical_json_bytes(batch)).hexdigest()
    if offset_digest:
        from cftuv_envelope.materialize.offset_normal import offset_normals_digest

        offset_digest = offset_normals_digest(moved.vertex_normals)
    return replace(
        moved,
        patch_id=int(patch_to),
        batch=batch,
        content_digest=content_digest,
        offset_normals_digest=offset_digest,
        labels=new_labeling,
    )


__all__ = (
    "CONTENT_STORE_ENTRY_LIMIT",
    "CONTENT_STORE_RESULT_LIMIT",
    "ContentEntryV1",
    "ContentRelabelFailed",
    "ContentStoreV1",
    "RelabelV1",
    "carried_to_run",
    "relabel_result",
)
