"""Сырые декларации Density: чтение носителей без вывода нормалей и веера."""

from __future__ import annotations

from decimal import Decimal
from enum import Enum
from fractions import Fraction

from sympy import Rational

from .._annulus_cut import chart_cycle
from ..contracts.analysis import (
    CertifiedPlanarSupportDirectionV1, CertifiedReflexAngleMeasureV1,
    TurnOrientation, UnavailableSourceSupportDirectionV1,
)
from ..contracts.envelopes import HuberDensitySelectionIntervalCertificateV1
from ..contracts.metric import CertifiedAffineSupportDirectionV2, ExactRationalV1
from ..ids import OpaqueId
from ..numeric import ExactRatioV1
from .common import stable_id
from .density_oracle_v1 import (
    RawDensityCanonicalFactsV1, RawDensityGeometryV1, RawDensityIncidentV1,
    RawDensityMetricV1, RawDensityPointV1, RawDensitySelectionV1, RawDensitySpecInput,
)
from .evaluation_binding_noise import NOISE_DIRECTION_SINE_BOUND, NOISE_LAW


class RawDensityExtractionErrorV1(ValueError):
    """Неполный внутренний raw carrier; не продуктовый числовой отказ."""


def _q(value) -> Fraction:
    # Только атомы уже принятого носителя; выражения не парсятся и не строятся.
    if type(value) in (int, Fraction, Decimal):
        return Fraction(value)
    if type(value) is float:
        return Fraction(str(value))
    if isinstance(value, (ExactRationalV1, ExactRatioV1)):
        return Fraction(value.numerator, value.denominator)
    if isinstance(value, Rational):
        return Fraction(int(value.p), int(value.q))
    raise RawDensityExtractionErrorV1("raw numeric fact is not a rational carrier atom")


def _pair(value) -> tuple[int, int]:
    rational = _q(value)
    return rational.numerator, rational.denominator


def _name(value) -> str:
    if isinstance(value, (OpaqueId, Enum)) and type(value.value) is str:
        return value.value
    if type(value) is str:
        return value
    raise RawDensityExtractionErrorV1("raw identity/declaration is not a text carrier")


def _coordinates(context):
    binding = context.compilation.evaluation_geometry_binding
    records = (binding.source_vertex_coordinates if binding is not None else
               getattr(context.frame, "exact_source_vertex_coordinates", None))
    if records is None:
        records = context.frame.source_vertex_coordinates
    return {item.source_vertex_id: RawDensityPointV1(_q(item.domain_coordinate.x), _q(item.domain_coordinate.y))
            for item in records}


def _metric(context) -> RawDensityMetricV1:
    metric = context.metric
    gram, inverse = metric.gram, metric.inverse_gram
    identity = getattr(context.frame, "reference_metric_id", None)
    return RawDensityMetricV1(
        (_q(gram[0][0]), _q(gram[0][1]), _q(gram[1][1])),
        (_q(inverse[0][0]), _q(inverse[0][1]), _q(inverse[1][1])),
        metric.owner_orientation_sign, None if identity is None else _name(identity))


def _direction_record(reference):
    payload = reference.direction_payload
    if isinstance(payload, CertifiedPlanarSupportDirectionV1):
        rows = (("kind", "CERTIFIED_PLANAR_V1"), ("x", _pair(payload.direction_in_domain.x)),
                ("y", _pair(payload.direction_in_domain.y)), ("authority", _name(payload.authority)))
    elif isinstance(payload, CertifiedAffineSupportDirectionV2):
        rows = (("kind", "CERTIFIED_AFFINE_V2"), ("x", _pair(payload.direction_in_domain.x)),
                ("y", _pair(payload.direction_in_domain.y)),
                ("reference_metric_id", _name(payload.reference_metric_id)))
    elif isinstance(payload, UnavailableSourceSupportDirectionV1):
        rows = (("kind", "UNAVAILABLE_V1"), ("reason", _name(payload.reason)))
    else:
        raise RawDensityExtractionErrorV1("unknown source direction carrier")
    return rows + (("source_launch_boundary_id", _name(reference.source_launch_boundary_id)),)


def _direction_declarations(context, use_id):
    # Порядок всех ссылок совпадает с _analysis_direction; никакой нормализации.
    records = tuple(_direction_record(reference)
                    for sector in context.snapshot.angular_owner_sectors
                    for reference in (sector.incoming_support_ref, sector.outgoing_support_ref)
                    if reference.chain_use_id == use_id)
    return (("records", records),)


def _anchor_segment(context, use_id, anchor):
    use = context.uses_by_id[use_id]
    chain = context.chains_by_id[use.physical_chain_id]
    vertices = context.directed_chain_vertices(use)
    pairs = tuple(zip(vertices, vertices[1:]))
    if chain.is_closed and len(vertices) > 1:
        pairs += ((vertices[-1], vertices[0]),)
    matches = tuple(pair for pair in pairs if anchor in pair)
    if len(matches) != 1:
        raise RawDensityExtractionErrorV1("angular anchor is not a unique raw chain endpoint")
    start, end = matches[0]
    allowed = set(chain.ordered_physical_edge_ids)
    edges = tuple(edge.edge_id for edge in context.snapshot.surface_ir.source_edges
                  if edge.edge_id in allowed and {edge.vertex_a_id, edge.vertex_b_id} == {start, end})
    if len(edges) != 1:
        raise RawDensityExtractionErrorV1("raw incident physical edge provenance is not total")
    return use, start, end, edges[0]


def _face_names(context, face):
    cut = getattr(context.frame.planarity_certificate, "cut", None)
    if cut is None:
        return face.vertex_cycle
    corners = frozenset((item.triangle_id, item.vertex_id) for item in cut.right_corners)
    # Чистый существующий helper: GeometryContext.chart_cycle_of менял бы memo.
    return chart_cycle(face.vertex_cycle, face.triangle_ids, corners)


def _incident(context, coordinates, use_id, anchor) -> RawDensityIncidentV1:
    use, start, end, edge_id = _anchor_segment(context, use_id, anchor)
    chart_start, chart_end = context.chart_ends_of(start, end)
    faces = tuple(face for face in context.snapshot.surface_ir.source_faces
                  if face.patch_id == use.owner_patch_id and edge_id in face.edge_cycle)
    names = tuple(_face_names(context, face) for face in faces)
    try:
        cycles = tuple(tuple(coordinates[vertex] for vertex in cycle) for cycle in names)
        first, last = coordinates[chart_start], coordinates[chart_end]
    except KeyError as error:
        raise RawDensityExtractionErrorV1("raw coordinate carrier lacks an incident/owner-face chart vertex") from error
    return RawDensityIncidentV1(
        first, last, None, _name(anchor), _name(start), _name(end),
        stable_id("source-support", use_id, edge_id), _name(use.orientation),
        _direction_declarations(context, use_id), cycles,
        tuple(tuple(_name(vertex) for vertex in cycle) for cycle in names),
        tuple(_name(face.face_id) for face in faces),
        tuple(face.edge_cycle.index(edge_id) for face in faces), _name(edge_id), _name(use_id))


def _geometry(context, sector, anchor):
    coordinates = _coordinates(context)
    uses = sector.ordered_incident_chain_use_ids
    return RawDensityGeometryV1(_metric(context), _incident(context, coordinates, uses[0], anchor),
                                _incident(context, coordinates, uses[-1], anchor))


def _interval_declaration(interval):
    return (("lower", _pair(interval.lower)), ("upper", _pair(interval.upper)),
            ("lower_kind", _name(interval.lower_kind)), ("upper_kind", _name(interval.upper_kind)),
            ("absolute_error_bound", _pair(interval.absolute_error_bound)))


def _restoration(compilation, selection):
    matches = tuple(record for record in compilation.canonical_angle_restorations
                    if record.selection_certificate_id == selection.certificate_id)
    if len(matches) > 1:
        raise RawDensityExtractionErrorV1("multiple raw restorations for one selection")
    if not matches:
        return None, ()
    record = matches[0]
    rows = tuple((key, _name(getattr(record, key))) for key in (
        "restoration_law", "selection_certificate_id", "corner_relation_id", "reflex_angle_certificate_id",
        "canonical_relation", "tolerance_policy_id"))
    rows += tuple((key, _pair(getattr(record, key))) for key in (
        "canonical_reflex_excess_over_pi", "deviation_upper_bound_radians", "tolerance_radians"))
    rows += (("source_reflex_excess_over_pi", _interval_declaration(record.source_reflex_excess_over_pi)),
             ("proven_predicates", tuple(sorted(record.proven_predicates))))
    return _q(record.canonical_reflex_excess_over_pi), rows


def _reflex_declaration(angle):
    payload = angle.measure_payload
    if not isinstance(payload, CertifiedReflexAngleMeasureV1):
        raise RawDensityExtractionErrorV1("raw selected angle lacks a certified reflex interval")
    rows = tuple((key, _name(getattr(angle, key))) for key in (
        "certificate_id", "measure_law", "measure_source", "strict_range_certificate", "reflex_excess_law"))
    rows += (("exact_two_pi", angle.exact_two_pi), ("orientation", _name(payload.orientation)),
             ("normalization_law", _name(payload.normalization_law)),
             ("phi_over_pi", _interval_declaration(payload.phi_over_pi)),
             ("reflex_excess_over_pi", _interval_declaration(payload.reflex_excess_over_pi)))
    interval = payload.reflex_excess_over_pi
    return (_q(interval.lower), _q(interval.upper)), rows


def _coordinate_rows(records, vertex_ids):
    coordinates = {item.source_vertex_id: item.domain_coordinate for item in records}
    return tuple((_name(vertex), _pair(coordinates[vertex].x), _pair(coordinates[vertex].y))
                 for vertex in sorted(vertex_ids, key=_name) if vertex in coordinates)


def _source_grid(frame):
    grid = getattr(frame, "grid_certificate", None)
    if grid is None:
        return ()
    return (("snapping_law", _name(grid.snapping_law)),
            ("window_step", None if grid.window_step is None else _pair(grid.window_step)),
            ("source_scale", grid.source_scale))


def _noise_declarations(context, sector, anchor):
    incoming = context.directed_chain_vertices(context.uses_by_id[sector.ordered_incident_chain_use_ids[0]])
    outgoing = context.directed_chain_vertices(context.uses_by_id[sector.ordered_incident_chain_use_ids[-1]])
    vertex_ids = frozenset((*incoming[-2:], anchor, *outgoing[:2]))
    source = getattr(context.frame, "exact_source_vertex_coordinates", None)
    binding = context.compilation.evaluation_geometry_binding
    rows = (("binding_present", binding is not None), ("source_coordinates_available", source is not None),
            ("incoming_directed_vertex_ids", tuple(map(_name, incoming))),
            ("outgoing_directed_vertex_ids", tuple(map(_name, outgoing))),
            ("source_coordinates", () if source is None else _coordinate_rows(source, vertex_ids)),
            ("evaluation_coordinates", () if binding is None else
             _coordinate_rows(binding.source_vertex_coordinates, vertex_ids)),
            ("source_grid_declaration", _source_grid(context.frame)),
            ("noise_direction_sine_bound", _pair(NOISE_DIRECTION_SINE_BOUND)), ("noise_law_id", _name(NOISE_LAW)))
    if binding is not None:
        rows += (("binding_schema", binding.schema_version), ("binding_law", _name(binding.binding_law)),
                 ("base_lattice_scale", getattr(binding, "base_lattice_scale", binding.lattice_scale)),
                 ("lattice_scale", binding.lattice_scale), ("refinement_power", getattr(binding, "refinement_power", None)))
    return rows


def extract_raw_density_spec_input(source_context, evaluation_context, spec, selection) -> RawDensitySpecInput:
    """Контексты уже приняты вызывающим; здесь нет GeometryContext.build/валидации геометрии."""
    snapshot = evaluation_context.snapshot
    relation = next((item for item in snapshot.corner_relations if item.corner_relation_id == spec.source_relation_id), None)
    sector = next((item for item in snapshot.angular_owner_sectors if item.owner_sector_id == spec.owner_sector_id), None)
    angle = next((item for item in snapshot.reflex_angle_certificates
                  if item.certificate_id == selection.reflex_angle_certificate_id), None)
    if relation is None or sector is None or angle is None:
        raise RawDensityExtractionErrorV1("selected Density declaration references an absent relation/sector/angle")
    interval = selection.selection_interval_certificate
    if not isinstance(interval, HuberDensitySelectionIntervalCertificateV1):
        raise RawDensityExtractionErrorV1("raw selection does not carry a Density interval")
    raw_selection = RawDensitySelectionV1(interval.q, selection.resolved_hidden_edge_count,
        _name(selection.selection_policy_id), _name(selection.max_subturn_value_id), interval.bucket_c, _name(selection.certificate_id))
    source_interval, reflex = _reflex_declaration(angle)
    canonical, restoration = _restoration(evaluation_context.compilation, selection)
    turn = sector.turn_orientation
    if turn not in (TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION):
        raise RawDensityExtractionErrorV1("raw turn orientation is not a declared owner-sector orientation")
    return RawDensitySpecInput(
        _geometry(source_context, sector, relation.source_vertex_id),
        _geometry(evaluation_context, sector, relation.source_vertex_id),
        1 if turn is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION else -1, raw_selection,
        RawDensityCanonicalFactsV1(source_interval, canonical, restoration,
            _noise_declarations(evaluation_context, sector, relation.source_vertex_id), reflex),
        _name(spec.envelope_spec_id), _name(spec.source_relation_id), _name(spec.owner_sector_id))
