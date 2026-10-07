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
            return SqrtSumV1(((1, value), (2, Fraction(rng.choice((-1, 1, 2)), rng.choice((1, 2))))))
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
