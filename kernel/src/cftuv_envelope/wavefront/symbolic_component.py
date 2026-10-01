"""Shared order-free topology delta for symbolic contact components."""

from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass, replace

from .event_time import EventPointV1
from .sqrt_sum import SqrtSumV1
from .symbolic_overlay import (
    JunctionRefV1,
    SymbolicVertexV1,
    refreshed_span_bindings,
)


@dataclass(frozen=True, slots=True)
class SymbolicComponentDeltaV1:
    contact_keys: tuple
    dead_refs: tuple[JunctionRefV1, ...]
    incoming: tuple | None
    outgoing: tuple | None
    birth_ref: JunctionRefV1 | None
    point_key: tuple
    leaf_resources: frozenset
    rewires: tuple[tuple[tuple, tuple, JunctionRefV1], ...] = ()


def clone_overlay(overlay):
    return type(overlay)(
        {ref: replace(vertex) for ref, vertex in overlay.vertices.items()},
        dict(overlay.spans),
        set(overlay.changed),
        overlay.time,
    )


def point_from_key(key):
    return EventPointV1(SqrtSumV1(key[0]), SqrtSumV1(key[1]))


class _SignaturePartsMemo:
    """Тексты `repr` неизменяемых частей подписи, по одному на ОБЪЕКТ.

    Подпись сортирует вершины, пролёты и листья по `repr`, а `repr` вершины
    разворачивает сотни `Fraction` в ссылках и точках: на тяжёлых доменах это
    15 % секунд. При этом подписи одной транзакции берутся с клонов ОДНОГО
    наложения (`clone_overlay` делает `replace(vertex)`, то есть вершина новая,
    а её ссылки, точка и скольжение — те же самые объекты), поэтому `repr`
    каждой неизменяемой части достаточно посчитать один раз.

    Ключ — `id(объекта)`, и запись держит сам объект: пока запись жива, адрес
    не может достаться другому. Текст части есть ровно `repr(часть)`, а текст
    кортежа — ровно `"(" + ", ".join(тексты частей) + ")"`, как у самого
    `tuple.__repr__`, поэтому ключ сортировки побитово прежний и порядок
    подписи тот же. Память не хранит ни ответа, ни цены: она живёт, пока
    открыта `signature_memo()`, и умирает вместе с ней.
    """

    __slots__ = ("texts", "births", "traces")

    def __init__(self):
        self.texts: dict = {}
        self.births: dict = {}
        self.traces: dict = {}

    def shown(self, part) -> str:
        if part is None:
            return "None"
        entry = self.texts.get(id(part))
        if entry is None:
            entry = self.texts[id(part)] = (part, repr(part))
        return entry[1]

    def birth(self, birth):
        """`(birth.canonical(), repr(...))`: каноническая форма считается раз."""

        entry = self.births.get(id(birth))
        if entry is None:
            canonical = birth.canonical()
            entry = self.births[id(birth)] = (birth, canonical, repr(canonical))
        return entry[1], entry[2]

    def trace(self, trace):
        """`(trace_authority(trace), repr(...))` для следа вершины."""

        if trace is None:
            authority = ("UNAVAILABLE",)
            return authority, repr(authority)
        crash_time = trace.crash_time
        if crash_time is None:
            authority = ("TRACE_WITHOUT_CRASH",)
            return authority, repr(authority)
        entry = self.traces.get(id(crash_time))
        if entry is None:
            authority = ("BOUNDED", crash_time.canonical())
            entry = self.traces[id(crash_time)] = (
                crash_time, authority, repr(authority)
            )
        return entry[1], entry[2]


_SIGNATURE_MEMO: ContextVar = ContextVar("overlay_signature_memo", default=None)


@contextmanager
def signature_memo():
    """Открыть память представлений подписи на время одной транзакции."""

    if _SIGNATURE_MEMO.get() is not None:
        yield
        return
    token = _SIGNATURE_MEMO.set(_SignaturePartsMemo())
    try:
        yield
    finally:
        _SIGNATURE_MEMO.reset(token)


def _joined(texts) -> str:
    return "(" + ", ".join(texts) + ")"


def overlay_signature(overlay):
    memo = _SIGNATURE_MEMO.get()
    if memo is None:
        memo = _SignaturePartsMemo()
    shown = memo.shown
    rows = []
    for v in overlay.vertices.values():
        if not v.alive:
            continue
        birth, birth_text = memo.birth(v.birth)
        authority, authority_text = memo.trace(v.trace)
        rows.append((
            _joined((
                shown(v.ref), shown(v.prev), shown(v.next),
                shown(v.prev_leaf), shown(v.next_leaf), birth_text,
                shown(v.point), shown(v.sliding), shown(v.provenance),
                authority_text,
            )),
            (
                v.ref, v.prev, v.next, v.prev_leaf, v.next_leaf, birth,
                v.point, v.sliding, v.provenance, authority,
            ),
        ))
    rows.sort(key=_row_text)
    vertices = tuple(entry for _, entry in rows)
    rows = [
        (
            _joined((
                shown(leaf), repr(item.physical_edge_id),
                shown(item.start), shown(item.end),
            )),
            (leaf, item.physical_edge_id, item.start, item.end),
        )
        for leaf, item in overlay.spans.items()
    ]
    rows.sort(key=_row_text)
    spans = tuple(entry for _, entry in rows)
    return vertices, spans, tuple(sorted(overlay.changed, key=shown))


def _row_text(row) -> str:
    return row[0]


def normalize_dead_component(
    overlay,
    *,
    contact_keys,
    dead_refs,
    point_key,
    birth_kind,
    stale_reason,
    ambiguity_reason,
):
    """Derive terminal 0/0 or the unique 1/1 boundary-arm pairing."""

    dead = set(dead_refs)
    if any(ref not in overlay.vertices or not overlay.vertices[ref].alive
           for ref in dead):
        return None, stale_reason
    incoming = {(overlay.vertices[ref].prev, overlay.vertices[ref].prev_leaf)
                for ref in dead if overlay.vertices[ref].prev not in dead}
    outgoing = {(overlay.vertices[ref].next, overlay.vertices[ref].next_leaf)
                for ref in dead if overlay.vertices[ref].next not in dead}
    if (len(incoming), len(outgoing)) not in ((0, 0), (1, 1)):
        return None, ambiguity_reason
    in_port = None if not incoming else next(iter(incoming))
    out_port = None if not outgoing else next(iter(outgoing))
    if in_port is not None:
        predecessor = overlay.vertices.get(in_port[0])
        successor = overlay.vertices.get(out_port[0])
        if (predecessor is None or successor is None or not predecessor.alive
                or not successor.alive or predecessor.next not in dead
                or successor.prev not in dead):
            return None, ambiguity_reason
    ordered_keys = tuple(sorted(contact_keys, key=repr))
    birth = None if in_port is None else JunctionRefV1(
        birth_kind, ordered_keys
    )
    leaves = frozenset(
        leaf for ref in dead
        for leaf in (overlay.vertices[ref].prev_leaf,
                     overlay.vertices[ref].next_leaf)
    )
    rewires = () if birth is None else ((in_port, out_port, birth),)
    return SymbolicComponentDeltaV1(
        ordered_keys,
        tuple(sorted(dead, key=repr)),
        in_port,
        out_port,
        birth,
        point_key,
        leaves,
        rewires,
    ), None


def delta_resources(delta):
    resources = {("DEAD", ref) for ref in delta.dead_refs}
    resources.update(("LEAF", leaf) for leaf in delta.leaf_resources)
    rewires = delta.rewires
    if not rewires and delta.incoming is not None:
        rewires = ((delta.incoming, delta.outgoing, delta.birth_ref),)
    for incoming, outgoing, _ in rewires:
        resources.update((
            ("PORT_NEXT", incoming[0]),
            ("PORT_PREV", outgoing[0]),
        ))
    return resources


def apply_component_deltas(overlay, deltas, *, collision_reason):
    """Apply disjoint normalized components as one simultaneous generation."""

    result = clone_overlay(overlay)
    def rewires(delta):
        if delta.rewires:
            return delta.rewires
        if delta.birth_ref is None:
            return ()
        return ((delta.incoming, delta.outgoing, delta.birth_ref),)

    dead = {ref for delta in deltas for ref in delta.dead_refs}
    if sum(len(delta.dead_refs) for delta in deltas) != len(dead):
        return None, collision_reason
    births = tuple(
        birth for delta in deltas for _, _, birth in rewires(delta)
    )
    if len(set(births)) != len(births) or any(
        birth in result.vertices for birth in births
    ):
        return None, collision_reason
    for delta in deltas:
        for ref in delta.dead_refs:
            result.vertices[ref].alive = False
    for delta in deltas:
        provenance = frozenset().union(
            *(result.vertices[ref].provenance for ref in delta.dead_refs)
        )
        for incoming, outgoing, birth in rewires(delta):
            _, prev_leaf = incoming
            _, next_leaf = outgoing
            result.vertices[birth] = SymbolicVertexV1(
                birth, None, None, prev_leaf, next_leaf,
                result.time, point_from_key(delta.point_key), None, provenance,
            )
            result.changed.update((prev_leaf, next_leaf))
    starts, ends = {}, {}
    for vertex in result.vertices.values():
        if not vertex.alive:
            continue
        starts.setdefault(vertex.next_leaf, []).append(vertex.ref)
        ends.setdefault(vertex.prev_leaf, []).append(vertex.ref)
    if any(len(refs) != 1 for refs in (*starts.values(), *ends.values())):
        return None, "SYMBOLIC_EDGE_SPAN_OWNER_AMBIGUOUS"
    for vertex in result.vertices.values():
        if not vertex.alive:
            continue
        previous = starts.get(vertex.prev_leaf, [])
        following = ends.get(vertex.next_leaf, [])
        if len(previous) != 1 or len(following) != 1:
            return None, "SYMBOLIC_JUNCTION_RECIPROCITY_UNRESOLVABLE"
        vertex.prev = previous[0]
        vertex.next = following[0]
    bindings, reason = refreshed_span_bindings(result)
    if reason is not None:
        return None, reason
    result.spans = bindings
    for vertex in result.vertices.values():
        if not vertex.alive:
            continue
        previous = result.vertices.get(vertex.prev)
        following = result.vertices.get(vertex.next)
        if (
            previous is None or following is None
            or not previous.alive or not following.alive
            or previous.next != vertex.ref or following.prev != vertex.ref
        ):
            return None, "SYMBOLIC_JUNCTION_RECIPROCITY_UNRESOLVABLE"
    return result, None
