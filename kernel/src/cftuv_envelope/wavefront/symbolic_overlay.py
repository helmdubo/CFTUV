"""Runtime-ID-free junction identity with regenerated symbolic leaf bindings."""

from __future__ import annotations

from dataclasses import dataclass, replace

from .event_time import compare_times
from .exact_candidate_view import (
    CandidateSpanStateV1,
    CandidateVertexStateV1,
    ExactCandidateViewV1,
    sliding_projection,
)
from . import superlevel as base
from .superlevel_closure import (
    SegmentRefV1,
    SpanFamilyRefV1,
    SymbolicMaterializationPlanV1,
)
from .symbolic_sparse_ports import with_line_ports


@dataclass(frozen=True, slots=True)
class JunctionRefV1:
    kind: str
    key: tuple


@dataclass(slots=True)
class SymbolicVertexV1:
    ref: JunctionRefV1
    prev: JunctionRefV1 | None
    next: JunctionRefV1 | None
    prev_leaf: SegmentRefV1
    next_leaf: SegmentRefV1
    birth: object
    point: object
    sliding: object | None
    provenance: frozenset[tuple]
    runtime_id: int | None = None
    trace: object | None = None
    alive: bool = True


@dataclass(frozen=True, slots=True)
class SymbolicSpanBindingV1:
    leaf: SegmentRefV1
    physical_edge_id: int
    start: JunctionRefV1 | None
    end: JunctionRefV1 | None


@dataclass(slots=True)
class SymbolicOverlayV1:
    vertices: dict[JunctionRefV1, SymbolicVertexV1]
    spans: dict[SegmentRefV1, SymbolicSpanBindingV1]
    changed: set[SegmentRefV1]
    time: object


def refreshed_span_bindings(overlay):
    """Rebind active leaf owners, refusing multiplicity instead of last-win."""

    starts = {}
    ends = {}
    for vertex in overlay.vertices.values():
        if not vertex.alive:
            continue
        starts.setdefault(vertex.next_leaf, []).append(vertex.ref)
        ends.setdefault(vertex.prev_leaf, []).append(vertex.ref)
    if any(len(refs) != 1 for refs in (*starts.values(), *ends.values())):
        return None, "SYMBOLIC_EDGE_SPAN_OWNER_AMBIGUOUS"
    bindings = {
        leaf: SymbolicSpanBindingV1(
            leaf,
            binding.physical_edge_id,
            None if leaf not in starts else starts[leaf][0],
            None if leaf not in ends else ends[leaf][0],
        )
        for leaf, binding in overlay.spans.items()
    }
    return bindings, None


def _consumed_by_locus(vertex) -> bool:
    """Порт, у которого ОБА плеча схлопнуты ровно в `now`.

    Признак точный и берётся из frozen prestate: пролёт схлопнут, когда его
    концы СОВПАЛИ. Ни рантаймовых id, ни порядков здесь нет. Такой порт не
    самостоятельный порт poststate — он внутренность локуса контакта, и его
    тождество принадлежит зародышу локуса, а не паре ключей вхождений.
    """

    return all(
        occurrence is not None
        and occurrence[1] is not None
        and occurrence[1] == occurrence[2]
        for occurrence in (vertex.prev_occurrence, vertex.next_occurrence)
    )


def _existing_ref(vertex) -> JunctionRefV1:
    """Ссылка на живой порт.

    У ВЫЖИВАЮЩЕГО порта тождество — hydration-инвариантная пара ключей
    вхождений: она обязана быть одной и той же между итерациями перестроения,
    потому что ключ контакта называет эмиттера именно ею.

    У ПОТРЕБЛЁННОГО локусом порта такой нужды нет: он не доживает до
    poststate, между итерациями его никто не называет. Поэтому он несёт полное
    тождество (точка плюс оба вхождения) — и перестаёт быть неразличимым
    алиасом выжившего порта с той же парой ключей.
    """

    if _consumed_by_locus(vertex):
        return JunctionRefV1(
            "EXISTING",
            (
                "CONSUMED_BY_LOCUS",
                vertex.point_key,
                vertex.prev_occurrence,
                vertex.next_occurrence,
            ),
        )
    return JunctionRefV1("EXISTING", base._port_identity(vertex))


def _hydrate_f0(builder, snapshot, plans, time):
    required = {
        occurrence[0]
        for vertex in snapshot.vertices
        for occurrence in (vertex.prev_occurrence, vertex.next_occurrence)
        if occurrence is not None
    }
    referenced = {
        reference.existing
        for plan in plans
        for _, predecessor, successor in plan.birth_wiring
        for reference in (predecessor, successor)
        if reference.existing is not None
    }
    referenced.update(
        ident
        for plan in plans
        for ident, _, _ in plan.existing_port_rewrites
    )
    for ident in referenced:
        vertex = builder.vertices[ident]
        required.update(
            (
                builder.edges[vertex.prev_edge].key,
                builder.edges[vertex.next_edge].key,
            )
        )
    point_keys, occurrences, duplicate = base._sparse_occurrences(
        builder, time, required
    )
    if duplicate:
        return None
    vertices = tuple(
        replace(
            vertex,
            point_key=point_keys[vertex.ident],
            prev_occurrence=occurrences.get(vertex.prev_edge),
            next_occurrence=occurrences.get(vertex.next_edge),
        )
        for vertex in snapshot.vertices
    )
    return replace(snapshot, vertices=vertices)


def _occurrences(snapshot, plans):
    result = set()
    for vertex in snapshot.vertices:
        result.update(
            item
            for item in (vertex.prev_occurrence, vertex.next_occurrence)
            if item is not None
        )
    for plan in plans:
        for cut in plan.split_cuts:
            result.update(cut.segment_occurrences)
        for birth in plan.births:
            result.update((birth.prev_occurrence, birth.next_occurrence))
        for _, prev_occurrence, next_occurrence in plan.existing_port_rewrites:
            result.update((prev_occurrence, next_occurrence))
    return result


def _leaf_bindings(snapshot, materialization):
    leaves = {
        segment.occurrence: segment
        for family in materialization.families
        for segment in family.segments
    }
    for occurrence in _occurrences(snapshot, materialization.plans):
        if occurrence not in leaves:
            family = SpanFamilyRefV1(occurrence, (occurrence[0],))
            leaves[occurrence] = SegmentRefV1(
                family, None, None, occurrence
            )
    edge_by_occurrence = base._existing_edges_by_occurrence(
        snapshot, materialization.plans
    )
    if edge_by_occurrence is None:
        return None, None
    for vertex in snapshot.vertices:
        for occurrence, edge_id in (
            (vertex.prev_occurrence, vertex.prev_edge),
            (vertex.next_occurrence, vertex.next_edge),
        ):
            if occurrence is None:
                continue
            previous = edge_by_occurrence.get(occurrence)
            if previous is not None and previous != edge_id:
                return None, None
            edge_by_occurrence[occurrence] = edge_id
    for plan in materialization.plans:
        for cut in plan.split_cuts:
            for occurrence in cut.segment_occurrences:
                edge_by_occurrence[occurrence] = cut.edge_id
    if any(item not in edge_by_occurrence for item in leaves):
        return None, None
    return leaves, edge_by_occurrence


def _birth_ref(birth, materialization, leaves):
    split_birth = next(
        (
            item
            for family in materialization.families
            for item in family.births
            if item.key == birth.key
        ),
        None,
    )
    prev_family = leaves[birth.prev_occurrence].family
    next_family = leaves[birth.next_occurrence].family
    if split_birth is not None:
        key = ("SPLIT", split_birth.contact, prev_family, next_family)
    else:
        contact = next(
            (
                contact_plan
                for plan in materialization.plans
                for contact_plan in plan.edge_contacts
                if any(
                    candidate.key == birth.key
                    for candidate in contact_plan.births
                )
            ),
            None,
        )
        key = (
            "CONTACT",
            birth.point_key,
            prev_family,
            next_family,
            () if contact is None else contact.participants,
            () if contact is None else tuple(
                kind.value for kind in contact.kinds
            ),
        )
    return JunctionRefV1("BIRTH", key)


def _initial_vertices(builder, snapshot, materialization, leaves):
    plans = materialization.plans
    dead = {ident for plan in plans for ident in plan.dead_vertex_ids}
    rewrites = {
        ident: (prev_occurrence, next_occurrence)
        for plan in plans
        for ident, prev_occurrence, next_occurrence
        in plan.existing_port_rewrites
    }
    frozen_by_id = {vertex.ident: vertex for vertex in snapshot.vertices}
    eligible = {
        ident: vertex
        for ident, vertex in frozen_by_id.items()
        if vertex.alive
        and vertex.prev_occurrence is not None
        and vertex.next_occurrence is not None
    }
    refs = {ident: _existing_ref(vertex) for ident, vertex in eligible.items()}
    # Admission уникальности — только по портам, ПЕРЕЖИВАЮЩИМ пакет. Порт,
    # оба плеча которого схлопываются в `now`, потребляется локусом контакта;
    # отказ по неразличимым алиасам остаётся для подлинно выживших.
    survivors = [
        ident for ident, vertex in eligible.items()
        if not _consumed_by_locus(vertex)
    ]
    if len({refs[ident] for ident in survivors}) != len(survivors):
        return None, None
    if len(set(refs.values())) != len(refs):
        return None, None
    vertices = {}
    for ident, frozen in eligible.items():
        if not frozen.alive or ident in dead:
            continue
        prev_occurrence, next_occurrence = rewrites.get(
            ident, (frozen.prev_occurrence, frozen.next_occurrence)
        )
        runtime = builder.vertices[ident]
        ref = refs[ident]
        vertices[ref] = SymbolicVertexV1(
            ref,
            refs.get(frozen.prev),
            refs.get(frozen.next),
            leaves[prev_occurrence],
            leaves[next_occurrence],
            runtime.birth,
            runtime.point,
            runtime.sliding,
            frozenset((ref.key,)),
            runtime_id=ident,
            trace=getattr(builder, "traces", {}).get(ident),
        )
    return vertices, refs


def build_symbolic_overlay(
    builder, snapshot, materialization, *, include_line_ports=False
):
    hydrated = _hydrate_f0(
        builder,
        snapshot,
        materialization.plans,
        materialization.plans[0].time,
    )
    if hydrated is None:
        return None
    if include_line_ports:
        hydrated = with_line_ports(
            builder, hydrated, materialization.plans[0].time
        )
        if hydrated is None:
            return None
    leaves, physical = _leaf_bindings(hydrated, materialization)
    if leaves is None:
        return None
    built = _initial_vertices(
        builder, hydrated, materialization, leaves
    )
    vertices, existing_refs = built
    if vertices is None:
        return None
    birth_refs = {}
    points = {
        base._event_point_key(event): event.point
        for plan in materialization.plans
        for event in plan.events
    }
    for plan in materialization.plans:
        for birth in plan.births:
            ref = _birth_ref(birth, materialization, leaves)
            if ref in vertices or birth.key in birth_refs:
                return None
            birth_refs[birth.key] = ref
            vertices[ref] = SymbolicVertexV1(
                ref,
                None,
                None,
                leaves[birth.prev_occurrence],
                leaves[birth.next_occurrence],
                plan.time,
                points[birth.point_key],
                None,
                frozenset((ref.key,)),
            )
    def resolve(reference):
        if reference.existing is not None:
            return existing_refs.get(reference.existing)
        return birth_refs.get(reference.birth_key)
    for plan in materialization.plans:
        for key, predecessor, successor in plan.birth_wiring:
            ref = birth_refs[key]
            prev_ref, next_ref = resolve(predecessor), resolve(successor)
            if prev_ref not in vertices or next_ref not in vertices:
                return None
            vertex = vertices[ref]
            vertex.prev, vertex.next = prev_ref, next_ref
            vertices[prev_ref].next = ref
            vertices[next_ref].prev = ref
    if any(
        (item.prev is None or item.next is None)
        and item.ref.kind == "BIRTH"
        for item in vertices.values()
    ):
        return None
    starts = {vertex.next_leaf: vertex.ref for vertex in vertices.values()}
    ends = {vertex.prev_leaf: vertex.ref for vertex in vertices.values()}
    spans = {
        leaf: SymbolicSpanBindingV1(
            leaf,
            physical[occurrence],
            starts.get(leaf),
            ends.get(leaf),
        )
        for occurrence, leaf in leaves.items()
    }
    changed = {
        segment
        for family in materialization.families
        for segment in family.segments
    }
    changed.update(
        leaf
        for vertex in vertices.values()
        if vertex.ref.kind == "BIRTH"
        for leaf in (vertex.prev_leaf, vertex.next_leaf)
    )
    return SymbolicOverlayV1(
        vertices, spans, changed, materialization.plans[0].time
    )


def _born_place(overlay, ref):
    """Место порта в `now`, если он в `now` и РОДИЛСЯ, иначе `None`.

    ПРОДОЛЖЕНИЕ ЗАКОНА ПРИМИРЕНИЯ НА КОНЦЫ ПРОЛЁТА. Закон сказал: admission
    уникальности — только про порты, ПЕРЕЖИВАЮЩИЕ пакет; порт, потреблённый
    локусом, несёт ПОЛНОЕ тождество, потому что до poststate он не доживает и
    парой ключей вхождений его никто не называет. То же самое верно про его
    МЕСТО. Порт, рождённый локусом ровно в `now`, стоит на двух СОВПАВШИХ
    антипараллельных прямых: пересечения у них нет, скольжения тоже нет
    (нормали противонаправлены), и закон движения честно отвечает «места нет».
    Но у пакета место этого порта ЕСТЬ и оно точное — это сам локус, точка его
    рождения. Спрашивать закон движения о том, что уже названо локусом, — та же
    ошибка, что называть потреблённый порт неразличимым алиасом выжившего.

    Признак точный и по содержимому пакета: время рождения совпало с `now`.
    Ни рантаймовых id, ни порядков, ни вида события здесь нет.

    ПОЧЕМУ НЕ `occurrence` ЛИСТА. `occurrence` — это ИМЯ листа, снятое в
    frozen prestate, а не его протяжённость внутри пакета: лист, который сам не
    режется, получает новый конец от контакта на СОСЕДНЕМ пролёте, и этот конец
    законно уходит за прежнее имя (измерено: `cross_full_q_*` теряет по два
    узла, если считать имя протяжённостью).
    """

    vertex = overlay.vertices.get(ref)
    if vertex is None or vertex.point is None:
        return None
    if compare_times(vertex.birth, overlay.time) != 0:
        return None
    return vertex.point


def exact_overlay_view(builder, overlay):
    def vertex_state(ref):
        vertex = overlay.vertices[ref]
        sliding = vertex.sliding
        if vertex.ref.kind != "EXISTING":
            first = span_state(vertex.prev_leaf).line
            second = span_state(vertex.next_leaf).line
            sliding = sliding_projection(first, second, vertex.point)
        return CandidateVertexStateV1(
            vertex.prev_leaf,
            vertex.next_leaf,
            vertex.birth,
            sliding,
        )

    def span_state(leaf):
        binding = overlay.spans[leaf]
        runtime = builder.edges[binding.physical_edge_id]
        return CandidateSpanStateV1(
            runtime.line,
            runtime.span,
            binding.start,
            binding.end,
            overlay.time,
            _born_place(overlay, binding.start),
            _born_place(overlay, binding.end),
        )

    def trace_bounds(ref, time):
        trace = overlay.vertices[ref].trace
        if trace is None:
            return None
        return trace.bounds_time(time)

    return ExactCandidateViewV1(
        builder._prime_universe,
        vertex_state,
        span_state,
        trace_bounds,
        builder.work_budget,
    )


def is_symbolic_split_emitter(builder, overlay, vertex) -> bool:
    """Use the runtime's exact support-line law without requiring runtime id."""

    if vertex.ref.kind == "VIRTUAL_BOUNDARY":
        return False
    if vertex.runtime_id is not None:
        runtime = builder.vertices[vertex.runtime_id]
        return runtime.reflex or runtime.sliding is not None
    if not hasattr(builder, "edges"):
        return False
    first = builder.edges[
        overlay.spans[vertex.prev_leaf].physical_edge_id
    ].line
    second = builder.edges[
        overlay.spans[vertex.next_leaf].physical_edge_id
    ].line
    cross = first.b * (-second.a) - (-first.a) * second.b
    sliding = vertex.sliding
    if sliding is None:
        sliding = sliding_projection(first, second, vertex.point)
    return cross < 0 or sliding is not None
