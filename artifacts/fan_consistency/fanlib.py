"""Общая библиотека диагностики согласованности вееров (только чтение, без правок продукта)."""
from __future__ import annotations

import math
from fractions import Fraction

import _paths

_paths.add_kernel_paths()

from cftuv_envelope.contracts.envelopes import AngularEnvelopeSpec  # noqa: E402


def frac(value) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def vertex_index(source_vertex_id) -> int:
    return int(source_vertex_id.value.rsplit(":", 1)[1])


def point_of(coordinate) -> tuple[Fraction, Fraction]:
    return frac(coordinate.x), frac(coordinate.y)


class DomainView:
    """Индексы снапшота и подготовки одного домена."""

    def __init__(self, snap, prep):
        self.snap = snap
        self.prep = prep
        self.comp = prep.compilation
        self.frame = prep.context.frame
        self.relations = {r.corner_relation_id: r for r in snap.corner_relations}
        self.certs = {c.certificate_id: c for c in snap.reflex_angle_certificates}
        self.sectors = {s.owner_sector_id: s for s in snap.angular_owner_sectors}
        self.uses = {u.chain_use_id: u for u in snap.chain_uses}
        self.chains = {c.physical_chain_id: c for c in snap.physical_chains}
        self.selections = {
            s.certificate_id: s for s in self.comp.profile_selection_certificates
        }
        self.restorations = {
            r.selection_certificate_id: r
            for r in self.comp.canonical_angle_restorations
        }
        self.source_xy = {
            c.source_vertex_id: point_of(c.domain_coordinate)
            for c in self.frame.exact_source_vertex_coordinates
        }
        binding = self.comp.evaluation_geometry_binding
        self.binding = binding
        self.eval_xy = (
            {c.source_vertex_id: point_of(c.domain_coordinate) for c in binding.source_vertex_coordinates}
            if binding is not None
            else {}
        )
        g = self.frame.exact_gram_matrix
        self.gram = (frac(g.m00), frac(g.m01), frac(g.m11))
        self.lattice = prep.lattice
        self.specs = sorted(
            (s for s in self.comp.envelope_specs if isinstance(s, AngularEnvelopeSpec)),
            key=lambda s: s.envelope_spec_id.value,
        )

    def gdot(self, a, b) -> Fraction:
        g00, g01, g11 = self.gram
        return a[0] * b[0] * g00 + (a[0] * b[1] + a[1] * b[0]) * g01 + a[1] * b[1] * g11

    def traversal(self, use_id):
        use = self.uses[use_id]
        vertices = self.chains[use.physical_chain_id].ordered_source_vertex_ids
        if use.orientation.value == "B_START_TO_END":
            vertices = tuple(reversed(vertices))
        return vertices

    def corner_neighbours(self, spec):
        """(вершина, предыдущая, следующая) по ходу владельца."""

        relation = self.relations[spec.source_relation_id]
        sector = self.sectors[relation.owner_sector_id]
        incoming, outgoing = sector.ordered_incident_chain_use_ids
        vertex = relation.source_vertex_id
        t_in = self.traversal(incoming)
        t_out = self.traversal(outgoing)
        assert t_in[-1] == vertex and t_out[0] == vertex, (t_in, t_out, vertex)
        return vertex, t_in[-2], t_out[1]

    def turn(self, coords, vertex, prev, nxt):
        """Точные dot/cross в метрике карты и угол поворота (градусы, знаковый)."""

        a = (coords[vertex][0] - coords[prev][0], coords[vertex][1] - coords[prev][1])
        b = (coords[nxt][0] - coords[vertex][0], coords[nxt][1] - coords[vertex][1])
        dot = self.gdot(a, b)
        # площадь в метрике: cross_G = det(basis coords) * sqrt(det G); берём квадрат
        g00, g01, g11 = self.gram
        det_g = g00 * g11 - g01 * g01
        cross_coord = a[0] * b[1] - a[1] * b[0]
        cross2 = cross_coord * cross_coord * det_g
        la2 = self.gdot(a, a)
        lb2 = self.gdot(b, b)
        angle = math.degrees(
            math.atan2(math.copysign(math.sqrt(float(cross2)), float(cross_coord)), float(dot))
        )
        return dot, cross2, la2, lb2, angle
