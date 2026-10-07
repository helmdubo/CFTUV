"""Contacts and overlays that no run of the oracle makes, for the closure seams of the native skeleton port (WP-S5).

The natural runs of the kernel reach the generations of the symbolic closure only through the corpus; the paths of a generation that cuts a leaf (`_expand_target_leaves`,
`_interior_deltas`), that pairs three dead ports by rays (`_multi_delta`), or that refuses a stale or an overlapping contact are reached by few or no polygon. The code is
SYMBOLIC: it moves junctions, leaves and keys, and whether a contact is true to the geometry is the discovery's question, not the normalisation's. So the contacts are made here
from the overlay a real run has at the call, with keys and projections of the kinds the oracle makes (ties, one point twice, a leaf that is gone, an emitter that is not in the
overlay), and given to the oracle's own `normalize_mixed_generation` and `apply_mixed_generation` and to the native seam on the same state.

`Fabricator.generation_cases(overlay, rng)` answers `[(junction contacts, interior contacts)]`; `mutants(overlay, rng, count)` answers overlays with a link, a flag, a span or a
leaf of a real one spoiled, for the discoveries and for `apply_component_deltas`.
"""

from __future__ import annotations

import dataclasses
import random
from fractions import Fraction

from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.wavefront import superlevel as base
from cftuv_envelope.wavefront.event_time import EventPointV1
from cftuv_envelope.wavefront.exact_identity import exact_point_key
from cftuv_envelope.wavefront.superlevel import VertexReferenceV1
from cftuv_envelope.wavefront.superlevel_closure import SegmentRefV1, SpanFamilyRefV1
from cftuv_envelope.wavefront.superlevel_fixed_point import SymbolicSplitContactKeyV1, SymbolicSplitContactV1
from cftuv_envelope.wavefront.symbolic_edge_closure import SymbolicEdgeContactKeyV1, SymbolicEdgeContactV1
from cftuv_envelope.wavefront.symbolic_junction_contacts import edge_contact, endpoint_contact
from cftuv_envelope.wavefront.symbolic_overlay import JunctionRefV1
from cftuv_envelope.wavefront.symbolic_split_endpoint import EndpointContactKeyV1


def participants(*leaves) -> tuple:
    return tuple(sorted({key for leaf in leaves for key in leaf.family.participant_keys}))


def random_point(rng: random.Random) -> EventPointV1:
    def coordinate():
        value = Fraction(rng.randrange(-24, 24), rng.choice((1, 1, 2, 3)))
        if rng.random() < 0.25:
            root = Fraction(rng.choice((-1, 1, 2)), rng.choice((1, 2)))
            return SqrtSumV1(((1, value), (2, root)) if value else ((2, root),))
        return SqrtSumV1.rational(value)

    return EventPointV1(coordinate(), coordinate())


class Fabricator:
    """Contacts for one overlay."""

    def __init__(self, overlay) -> None:
        self.overlay = overlay
        self.alive = [vertex for vertex in overlay.vertices.values() if vertex.alive]
        self.leaves = [leaf for leaf, binding in overlay.spans.items() if binding.start is not None and binding.end is not None]
        self.time_key = base._time_key(overlay.time)

    def usable(self) -> bool:
        return len(self.alive) >= 2 and bool(self.leaves)

    # ---- interior contacts -------------------------------------------------------------------------------------------------------

    def interior_contact(self, rng, leaf, point, projection, *, emitter=None):
        emitter = emitter or rng.choice(self.alive)
        ref = emitter.ref
        key = SymbolicSplitContactKeyV1(self.time_key, exact_point_key(point), ref, leaf.family, participants(emitter.prev_leaf, emitter.next_leaf, leaf))
        return SymbolicSplitContactV1(key, self.overlay.time, point, projection, leaf)

    def interior(self, rng: random.Random) -> tuple:
        """A set of interior contacts on one or two leaves: distinct and tied projections, one point twice, and sometimes a contact the generation must refuse."""

        count = rng.choice((1, 1, 2, 2, 3, 4))
        targets = [rng.choice(self.leaves) for _ in range(rng.choice((1, 1, 2)))]
        points = [random_point(rng) for _ in range(rng.choice((1, 2, 3, 4)))]
        projections = [SqrtSumV1.rational(Fraction(rng.randrange(-6, 7), rng.choice((1, 2)))) for _ in range(3)]
        contacts = []
        for _ in range(count):
            leaf = rng.choice(targets)
            contacts.append(self.interior_contact(rng, leaf, rng.choice(points), rng.choice(projections)))
        roll = rng.random()
        if roll < 0.08:
            contacts[0] = dataclasses.replace(contacts[0], leaf=None)
        elif roll < 0.16:
            gone = SegmentRefV1(SpanFamilyRefV1(("gone", None, None), (("gone",),)), None, None, ("gone", None, None))
            contacts[0] = dataclasses.replace(contacts[0], leaf=gone)
        elif roll < 0.24:
            absent = JunctionRefV1("EXISTING", ((9, 9, 9, 9), None))
            contacts[0] = dataclasses.replace(contacts[0], key=dataclasses.replace(contacts[0].key, emitter=absent))
        elif roll < 0.30:
            contacts.append(contacts[0])
        return tuple(contacts)

    # ---- junction contacts -------------------------------------------------------------------------------------------------------

    def edge(self, rng, point):
        vertex = rng.choice(self.alive)
        peer = self.overlay.vertices.get(vertex.next)
        if peer is None:
            return None
        key = SymbolicEdgeContactKeyV1(self.time_key, exact_point_key(point), vertex.ref, peer.ref, vertex.prev_leaf.family, vertex.next_leaf.family, peer.next_leaf.family)
        contact = SymbolicEdgeContactV1(key, vertex.prev_leaf, vertex.next_leaf, peer.next_leaf, rng.random() < 0.2, participants(vertex.prev_leaf, vertex.next_leaf, peer.next_leaf))
        return edge_contact(contact)

    def endpoint(self, rng, point):
        emitter, endpoint = rng.sample(self.alive, 2)
        leaf = rng.choice(self.leaves)
        key = EndpointContactKeyV1(self.time_key, exact_point_key(point), emitter.ref, endpoint.ref, leaf.family, participants(emitter.prev_leaf, emitter.next_leaf, leaf))
        return endpoint_contact(self.overlay, key)

    def junction(self, rng: random.Random) -> tuple:
        points = [random_point(rng) for _ in range(rng.choice((1, 1, 2)))]
        found = []
        for _ in range(rng.choice((1, 2, 2, 3, 4, 5))):
            point = rng.choice(points)
            contact = self.edge(rng, point) if rng.random() < 0.5 else self.endpoint(rng, point)
            if contact is not None:
                found.append(contact)
        if found and rng.random() < 0.08:
            found.append(found[0])
        return tuple(found)

    def generation_cases(self, rng: random.Random, count: int) -> list:
        """`[(junction contacts, interior contacts)]`: interior only, junction only, and both."""

        if not self.usable():
            return []
        cases = []
        for _ in range(count):
            roll = rng.random()
            junction = self.junction(rng) if roll < 0.7 else ()
            interior = self.interior(rng) if roll > 0.3 else ()
            cases.append((junction, interior))
        return cases


# --------------------------------------------------------------------------
# deltas applied in ways a normalisation never makes
# --------------------------------------------------------------------------


def delta_variants(rng: random.Random, overlay, deltas: tuple) -> list:
    """Sets of deltas close to the ones a generation made: one twice (the same junctions die twice), two with one birth, a birth that the overlay already holds, a delta whose arm leads
    to a leaf nobody owns (the reciprocity of what is left fails), and the same set reordered."""

    if not deltas:
        return []
    found = [(*deltas, deltas[0])]
    reborn = [position for position, delta in enumerate(deltas) if delta.rewires]
    if reborn:
        position = reborn[0]
        delta = deltas[position]
        incoming, outgoing, birth = delta.rewires[0]
        rest = delta.rewires[1:]
        existing = rng.choice(list(overlay.vertices))
        lost = SegmentRefV1(SpanFamilyRefV1(("lost", None, None), (("lost",),)), None, None, ("lost", None, None))
        for changed in (
            dataclasses.replace(delta, rewires=((incoming, outgoing, existing), *rest)),
            dataclasses.replace(delta, dead_refs=()),
            dataclasses.replace(delta, rewires=((incoming, (outgoing[0], lost), birth), *rest)),
        ):
            found.append((*deltas[:position], changed, *deltas[position + 1:]))
    shuffled = list(deltas)
    rng.shuffle(shuffled)
    found.append(tuple(shuffled))
    return found


# --------------------------------------------------------------------------
# materializations the packet never makes
# --------------------------------------------------------------------------


def spoiled_materialization(rng: random.Random, vertices: tuple, materialization):
    """`(vertices, materialization)` of a real call of `build_symbolic_overlay` with one thing spoiled: a wire dropped or crossed or pointing at a vertex that is not there, a birth
    twice, a rewrite of a vertex that dies, a plan without its births, a family without its leaves, a vertex without its occurrence."""

    plans = list(materialization.plans)
    if not plans:
        return None
    index = rng.randrange(len(plans))
    plan = plans[index]
    kind = rng.randrange(9)
    families = materialization.families
    if kind == 0 and plan.birth_wiring:
        wiring = list(plan.birth_wiring)
        del wiring[rng.randrange(len(wiring))]
        plans[index] = dataclasses.replace(plan, birth_wiring=tuple(wiring))
    elif kind == 1 and plan.birth_wiring:
        wiring = list(plan.birth_wiring)
        at = rng.randrange(len(wiring))
        key, predecessor, successor = wiring[at]
        wiring[at] = (key, successor, predecessor)
        plans[index] = dataclasses.replace(plan, birth_wiring=tuple(wiring))
    elif kind == 2 and plan.birth_wiring:
        wiring = list(plan.birth_wiring)
        at = rng.randrange(len(wiring))
        key, predecessor, successor = wiring[at]
        wiring[at] = (key, VertexReferenceV1(existing=10_000), successor) if rng.random() < 0.5 else (key, predecessor, VertexReferenceV1(birth_key=("nobody",)))
        plans[index] = dataclasses.replace(plan, birth_wiring=tuple(wiring))
    elif kind == 3 and plan.births:
        plans[index] = dataclasses.replace(plan, births=(*plan.births, plan.births[0]))
    elif kind == 4 and plan.births:
        plans[index] = dataclasses.replace(plan, births=plan.births[1:])
    elif kind == 5 and vertices:
        at = rng.randrange(len(vertices))
        spoiled = list(vertices)
        spoiled[at] = dataclasses.replace(vertices[at], prev_occurrence=None) if rng.random() < 0.5 else dataclasses.replace(vertices[at], next_occurrence=None)
        return tuple(spoiled), materialization
    elif kind == 6 and plan.existing_port_rewrites:
        plans[index] = dataclasses.replace(plan, existing_port_rewrites=plan.existing_port_rewrites[1:])
    elif kind == 7 and families:
        return vertices, dataclasses.replace(materialization, families=families[1:])
    elif kind == 8 and vertices:
        at = rng.randrange(len(vertices))
        other = vertices[rng.randrange(len(vertices))]
        spoiled = list(vertices)
        spoiled[at] = dataclasses.replace(vertices[at], next_occurrence=other.next_occurrence, prev_occurrence=other.prev_occurrence)
        return tuple(spoiled), materialization
    else:
        plans[index] = dataclasses.replace(plan, dead_vertex_ids=(*plan.dead_vertex_ids, rng.randrange(len(vertices) or 1)))
    return vertices, dataclasses.replace(materialization, plans=tuple(plans))


def spoiled_vertices(rng: random.Random, vertices: tuple) -> tuple:
    """Frozen vertices of a packet no front would freeze: the occurrences of two vertices exchanged, one occurrence missing (a line-only port), two vertices with one occurrence
    (an edge with two owners), one vertex's ends equal."""

    alive = [index for index, vertex in enumerate(vertices) if vertex.alive]
    if len(alive) < 2:
        return vertices
    first, second = rng.sample(alive, 2)
    spoiled = list(vertices)
    kind = rng.randrange(5)
    a, b = vertices[first], vertices[second]
    if kind == 0:
        spoiled[first] = dataclasses.replace(a, prev_occurrence=b.prev_occurrence, next_occurrence=b.next_occurrence)
        spoiled[second] = dataclasses.replace(b, prev_occurrence=a.prev_occurrence, next_occurrence=a.next_occurrence)
    elif kind == 1:
        spoiled[first] = dataclasses.replace(a, next_occurrence=None) if rng.random() < 0.5 else dataclasses.replace(a, prev_occurrence=None)
    elif kind == 2:
        spoiled[second] = dataclasses.replace(b, next_occurrence=a.next_occurrence)
    elif kind == 3:
        spoiled[first] = dataclasses.replace(a, prev_occurrence=a.next_occurrence)
    else:
        spoiled[second] = dataclasses.replace(b, next_edge=a.next_edge)
    return tuple(spoiled)


class SharedOwner:
    """A builder in which two live vertices start one edge (what the head of the transaction refuses before the closure): for the duration of the context, then put back."""

    def __init__(self, builder, rng: random.Random) -> None:
        alive = [vertex for vertex in builder.vertices if vertex.alive]
        self.vertex, self.other = (rng.sample(alive, 2) if len(alive) >= 2 else (None, None))
        self.saved = None if self.vertex is None else self.vertex.next_edge

    def __enter__(self):
        if self.vertex is not None:
            self.vertex.next_edge = self.other.next_edge
        return self

    def __exit__(self, *_exception) -> None:
        if self.vertex is not None:
            self.vertex.next_edge = self.saved


# --------------------------------------------------------------------------
# packets no front freezes
# --------------------------------------------------------------------------


def _spoiled_vertex(vertex, rng: random.Random):
    """The vertex with one occurrence missing, or one end of one occurrence without its point (what an antiparallel joint leaves in a real front: a `None` among the keys)."""

    choice = rng.randrange(4)
    if choice == 0:
        return dataclasses.replace(vertex, prev_occurrence=None)
    occurrence = vertex.prev_occurrence if choice % 2 else vertex.next_occurrence
    if occurrence is None:
        return vertex
    parts = list(occurrence)
    parts[1 + rng.randrange(2)] = None
    spoiled = type(occurrence)(parts)
    return dataclasses.replace(vertex, prev_occurrence=spoiled) if choice % 2 else dataclasses.replace(vertex, next_occurrence=spoiled)


def spoiled_snapshots(snapshot, rng: random.Random, count: int) -> list:
    """Snapshots no front would make, close to the real one: a repeated incident, an incident of another vertex, a swapped point, a meeting that is not one, a lost ray, a lost
    occurrence, a different projection, an order, a missing incident. Every reference stays inside the front (a reference outside it is a crash, not a contract)."""

    incidents, vertices = list(snapshot.incidents), list(snapshot.vertices)
    if not incidents:
        return []
    found = []
    # a split incident of a cut with no projection (the oracle keeps it and fails only where the cut is ordered): all of them, then the first only
    cuts = [index for index, incident in enumerate(incidents) if incident.event.kind.value == "SPLIT" and incident.target_occurrence is not None]
    for dropped in ((cuts, cuts[:1]) if cuts else ()):
        found.append(dataclasses.replace(snapshot, incidents=tuple(dataclasses.replace(incident, target_projection=None) if index in dropped else incident for index, incident in enumerate(incidents))))
    for _ in range(count):
        mine = list(incidents)
        for _step in range(rng.randrange(1, 4)):
            index = rng.randrange(len(mine))
            incident = mine[index]
            kind = rng.randrange(10)
            if kind == 0:
                mine.insert(rng.randrange(len(mine) + 1), incident)
            elif kind == 1:
                mine.append(dataclasses.replace(incident, event=dataclasses.replace(incident.event, vertex=rng.choice(vertices).ident)))
            elif kind == 2 and len(mine) > 1:
                mine[index] = dataclasses.replace(incident, point_key=mine[rng.randrange(len(mine))].point_key)
            elif kind == 3 and incident.event.kind.value == "SPLIT":
                mine[index] = dataclasses.replace(incident, met_vertex_id=rng.choice(vertices).ident, met_adjacent=rng.random() < 0.5)
            elif kind == 4:
                mine[index] = dataclasses.replace(incident, target_ray=None if rng.random() < 0.5 else (1, 0))
            elif kind == 5:
                mine[index] = dataclasses.replace(incident, target_occurrence=None if rng.random() < 0.3 else incident.target_occurrence, emitter_key=rng.choice(mine).emitter_key)
            elif kind == 6 and len(mine) > 1:
                mine[index] = dataclasses.replace(incident, target_projection=mine[rng.randrange(len(mine))].target_projection)
            elif kind == 7:
                rng.shuffle(mine)
            elif kind == 8 and len(mine) > 1:
                del mine[index]
            elif kind == 9:
                mine[index] = dataclasses.replace(incident, peer_key=rng.choice(mine).peer_key, participants=rng.choice(mine).participants)
        spoiled = vertices
        if rng.random() < 0.5:
            at = rng.randrange(len(vertices))
            spoiled = list(vertices)
            spoiled[at] = _spoiled_vertex(vertices[at], rng)
        found.append(dataclasses.replace(snapshot, incidents=tuple(mine), vertices=tuple(spoiled)))
    return found


# --------------------------------------------------------------------------
# a scripted discovery
# --------------------------------------------------------------------------

CONFLICT = "SYMBOLIC_INTERIOR_SPLIT_CONTACT_METADATA_CONFLICT"
AMBIGUOUS = "SYMBOLIC_ENDPOINT_REFERENCE_AMBIGUOUS"


class ScriptedDiscovery:
    """The two discoveries of a round of `plan_mixed_generations`, made up for the first rounds: what they answered is written down (`steps`) and goes to the native seam as the
    script of the same call. The contacts are made from the overlay the oracle has at the round (a clone of the initial one with the generations of the chain applied), so the chain
    meets contacts of every shape: a generation with an interior cut, one that the replay refuses, a chain that outgrows its budget. A round the script does not cover is answered
    by the front."""

    def __init__(self, rng: random.Random, rounds: int, junction, interior) -> None:
        self.rng, self.rounds = rng, rounds
        self.natural_junction, self.natural_interior = junction, interior
        self.steps: dict = {}
        self.junction_calls = self.interior_calls = 0

    def _scripted(self, index: int) -> bool:
        return index < self.rounds and self.rng.random() < 0.75

    def junction(self, builder, overlay, memo=None):
        index, self.junction_calls = self.junction_calls, self.junction_calls + 1
        made = Fabricator(overlay)
        if not self._scripted(index) or not made.usable():
            return self.natural_junction(builder, overlay, memo)
        found = made.junction(self.rng)
        reason = AMBIGUOUS if self.rng.random() < 0.05 else None
        self.steps.setdefault(index, {})["junction"] = (found, reason)
        return found, reason

    def interior(self, builder, overlay, memo=None):
        index, self.interior_calls = self.interior_calls, self.interior_calls + 1
        made = Fabricator(overlay)
        if not self._scripted(index) or not made.usable():
            return self.natural_interior(builder, overlay, memo)
        found = made.interior(self.rng)
        reason = CONFLICT if self.rng.random() < 0.05 else None
        self.steps.setdefault(index, {})["interior"] = (found, reason)
        return found, reason

    def wire(self, cseams, enc_str) -> list:
        """`[[junction contacts | none, reason | none, interior contacts | none, reason | none], ...]`, one entry per round up to the last one that was scripted."""

        rounds = []
        for index in range(max(self.steps, default=-1) + 1):
            step = self.steps.get(index, {})
            junction, interior = step.get("junction"), step.get("interior")
            rounds.append(
                [
                    None if junction is None else [cseams.enc_junction_contact(item) for item in junction[0]],
                    None if junction is None or junction[1] is None else enc_str(junction[1]),
                    None if interior is None else [cseams.enc_interior_contact(item) for item in interior[0]],
                    None if interior is None or interior[1] is None else enc_str(interior[1]),
                ]
            )
        return rounds


# --------------------------------------------------------------------------
# spoiled overlays
# --------------------------------------------------------------------------


def clone(overlay):
    return type(overlay)({ref: dataclasses.replace(vertex) for ref, vertex in overlay.vertices.items()}, dict(overlay.spans), set(overlay.changed), overlay.time)


def mutants(overlay, rng: random.Random, count: int) -> list:
    """Overlays no front would make, close to a real one: a link pointing elsewhere, a vertex dead, a span unbound, a leaf given to two vertices, a changed leaf that is not a span."""

    refs = list(overlay.vertices)
    if len(refs) < 2:
        return []
    found = []
    for _ in range(count):
        spoiled = clone(overlay)
        for _step in range(rng.choice((1, 1, 2, 3))):
            ref = rng.choice(refs)
            vertex = spoiled.vertices[ref]
            kind = rng.randrange(7)
            if kind == 0:
                vertex.alive = False
            elif kind == 1:
                vertex.next = rng.choice(refs)
            elif kind == 2:
                vertex.prev = rng.choice(refs)
            elif kind == 3:
                other = spoiled.vertices[rng.choice(refs)]
                vertex.next_leaf = other.next_leaf
            elif kind == 4 and spoiled.spans:
                leaf = rng.choice(list(spoiled.spans))
                binding = spoiled.spans[leaf]
                spoiled.spans[leaf] = dataclasses.replace(binding, start=None if rng.random() < 0.5 else binding.start, end=None if rng.random() < 0.5 else binding.end)
            elif kind == 5 and spoiled.spans:
                spoiled.changed.add(rng.choice(list(spoiled.spans)))
            elif kind == 6 and len(spoiled.spans) > 1:
                del spoiled.spans[rng.choice(list(spoiled.spans))]
        found.append(spoiled)
    return found
