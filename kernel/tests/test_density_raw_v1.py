"""Raw frontend читает реальные compile carriers до вывода Density геометрии."""

from dataclasses import FrozenInstanceError, replace
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
import ast
import copy
import pickle

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope.contracts.analysis import (
    CertifiedPlanarSupportDirectionV1, SupportDirectionAuthority,
    UnavailableSourceSupportDirectionV1, SupportDirectionUnavailableReason,
)
from cftuv_envelope.numeric import LocalVector2V1
from cftuv_envelope.reference import compile as compiler
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.density_raw_v1 import (
    RawDensityExtractionErrorV1, _direction_record, _face_names, _q,
    extract_raw_density_spec_input,
)
from cftuv_envelope.reference.metric import ExactPlanarMetric


FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
CASES = (
    ("binding_noise_canonical_v1/building_patch114", "decal_request_density4.json"),
    ("density4_exact_limit_v1/mesh2_patch2", "decal_request_density4.json"),
    ("wall_noise_top_join_density4_v1/wall_noise_top_patch1", "decal_request.json"),
)


def _case_inputs(monkeypatch, folder, request_name):
    directory = FIXTURES / folder
    snapshot = kernel.AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
        (directory / "analysis_snapshot.json").read_bytes()).record
    request = kernel.DecalRequestCodecV1.loads((directory / request_name).read_bytes())
    captured = []
    def pre_attachment(compilation, **kwargs):
        captured.append(compilation)
        return compilation, None
    kernel.compile_reference_envelopes(snapshot, request, _direction_binding_attachment=pre_attachment)
    assert len(captured) == 1
    compilation = captured[0]
    frame, diagnostics = compiler.validate_reference_geometry_payload(snapshot, compilation.plan_key.patch_domain_id,
                                                                       density_bounded=True)
    assert frame is not None, diagnostics
    flags = dict(require_evaluation_binding=False, require_certified_bound_supports=False,
                 require_canonical_fan_authority=False)
    evaluation = GeometryContext.build(compilation, frame, **flags)
    source = GeometryContext.build(replace(compilation, evaluation_geometry_binding=None), frame, **flags)
    return source, evaluation


def _selections(context):
    by_id = {item.certificate_id: item for item in context.compilation.profile_selection_certificates}
    return tuple((spec, by_id[spec.selection_certificate_id]) for spec in context.compilation.envelope_specs
                 if isinstance(spec, kernel.AngularEnvelopeSpec) and spec.resolved_hidden_edge_count > 0)


def _raw_state(context):
    names = ("angular_ideal_cache", "canonical_subturn_fan", "angular_support_cache", "support_segment_cache",
             "evaluation_noise_cache", "chart_name_cache", "strip_index_cache")
    memo = context.metric._density_exact_memo
    return (copy.deepcopy(tuple(getattr(context, name) for name in names)),
            copy.deepcopy(tuple(getattr(memo, name) for name in memo.__slots__)))


def _forbidden(*args, **kwargs):
    raise AssertionError("raw extraction attempted a geometric/numeric producer")


def _ban_geometry(monkeypatch):
    for name in ("support_segments_for_use", "_analysis_direction", "_face_side_normal", "_sign", "_dot_g",
                 "_unit_g", "_oriented_cross", "_polygon_signed_area", "chart_cycle_of", "_chart_segment_points"):
        monkeypatch.setattr(GeometryContext, name, _forbidden)
    for name in ("density_expressions", "dot_g", "unit_g", "oriented_cross"):
        monkeypatch.setattr(ExactPlanarMetric, name, _forbidden)


@pytest.mark.parametrize("folder,request_name", CASES[:2])
def test_actual_raw_input_matches_carriers_without_geometry_or_cache_writes(monkeypatch, folder, request_name):
    source, evaluation = _case_inputs(monkeypatch, folder, request_name)
    pairs = _selections(evaluation)
    assert pairs
    before = _raw_state(source), _raw_state(evaluation)
    _ban_geometry(monkeypatch)
    for spec, selection in pairs:
        raw = extract_raw_density_spec_input(source, evaluation, spec, selection)
        assert raw == extract_raw_density_spec_input(source, evaluation, spec, selection)
        assert pickle.loads(pickle.dumps(raw)) == raw
        assert raw.selection.source_hidden_count == selection.resolved_hidden_edge_count
        assert raw.selection.q == selection.selection_interval_certificate.q
        assert raw.selection.bucket_c == selection.selection_interval_certificate.bucket_c
        assert raw.spec_id == spec.envelope_spec_id.value
        assert raw.source_geometry.metric.gram == tuple(Fraction(source.metric.gram[r][c]) for r, c in ((0, 0), (0, 1), (1, 1)))
        assert raw.evaluation_geometry.metric.inverse_gram == tuple(Fraction(evaluation.metric.inverse_gram[r][c])
                                                                    for r, c in ((0, 0), (0, 1), (1, 1)))
        assert raw.evaluation_geometry.metric.owner_orientation_sign == evaluation.metric.owner_orientation_sign
        angle = next(item for item in evaluation.snapshot.reflex_angle_certificates
                     if item.certificate_id == selection.reflex_angle_certificate_id)
        reflex = dict(raw.canonical.source_reflex_declaration)
        assert set(reflex) == {"certificate_id", "measure_law", "measure_source", "strict_range_certificate",
                               "reflex_excess_law", "exact_two_pi", "orientation", "normalization_law",
                               "phi_over_pi", "reflex_excess_over_pi"}
        assert reflex["exact_two_pi"] is angle.exact_two_pi
        assert raw.canonical.source_reflex_interval == (_q(angle.measure_payload.reflex_excess_over_pi.lower),
                                                       _q(angle.measure_payload.reflex_excess_over_pi.upper))
        for context, geometry in ((source, raw.source_geometry), (evaluation, raw.evaluation_geometry)):
            carrier = context.compilation.evaluation_geometry_binding
            records = carrier.source_vertex_coordinates if carrier is not None else context.frame.exact_source_vertex_coordinates
            coordinates = {item.source_vertex_id.value: item.domain_coordinate for item in records}
            for incident in (geometry.incoming, geometry.outgoing):
                start = coordinates[incident.source_vertex_start_id]
                end = coordinates[incident.source_vertex_end_id]
                assert (incident.start.x, incident.start.y) == (_q(start.x), _q(start.y))
                assert (incident.end.x, incident.end.y) == (_q(end.x), _q(end.y))
                assert incident.owner_left is None
                assert len(incident.owner_face_edge_ordinals) == len(incident.owner_face_cycles)
                assert dict(incident.source_direction_declaration)["records"]
        noise = dict(raw.canonical.noise_policy_declaration)
        assert noise["binding_present"] == (evaluation.compilation.evaluation_geometry_binding is not None)
        assert noise["noise_direction_sine_bound"] == (1, 400)
        assert not {"noise_applicable", "evaluation_turn_cosine_squared", "ideal", "lifted"} & noise.keys()
        if not raw.canonical.restoration_declaration:
            assert raw.canonical.canonical_excess_over_pi is None
        else:
            record = next(item for item in evaluation.compilation.canonical_angle_restorations
                          if item.selection_certificate_id == selection.certificate_id)
            assert raw.canonical.canonical_excess_over_pi == _q(record.canonical_reflex_excess_over_pi)
            assert set(dict(raw.canonical.restoration_declaration)) == {
                "restoration_law", "selection_certificate_id", "corner_relation_id", "reflex_angle_certificate_id",
                "canonical_relation", "tolerance_policy_id", "canonical_reflex_excess_over_pi",
                "deviation_upper_bound_radians", "tolerance_radians", "source_reflex_excess_over_pi", "proven_predicates"}
        with pytest.raises(FrozenInstanceError):
            raw.turn_orientation_sign = 0
    assert (_raw_state(source), _raw_state(evaluation)) == before


def test_actual_early_join_fixture_has_no_density_spec_to_extract(monkeypatch):
    _, context = _case_inputs(monkeypatch, *CASES[2])
    assert _selections(context) == ()


def test_source_count_is_copied_even_when_supplied_spec_has_a_later_effective_count(monkeypatch):
    source, evaluation = _case_inputs(monkeypatch, *CASES[0])
    spec, selection = _selections(evaluation)[0]
    changed = replace(spec, resolved_hidden_edge_count=spec.resolved_hidden_edge_count + 1)
    raw = extract_raw_density_spec_input(source, evaluation, changed, selection)
    assert raw.selection.source_hidden_count == selection.resolved_hidden_edge_count
    assert raw.selection.source_hidden_count != changed.resolved_hidden_edge_count


def test_direction_declarations_preserve_unavailable_and_reference_decimal_float_facts():
    reference = SimpleNamespace(source_launch_boundary_id="boundary", direction_payload=
        CertifiedPlanarSupportDirectionV1(LocalVector2V1(0.1, -0.3), SupportDirectionAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1))
    row = dict(_direction_record(reference))
    assert row["x"] == (1, 10)
    assert row["y"] == (-3, 10)
    reference.direction_payload = UnavailableSourceSupportDirectionV1(SupportDirectionUnavailableReason.EC0_SYMBOLIC_SUPPORT_ONLY_V5)
    assert dict(_direction_record(reference))["reason"] == "EC0_SYMBOLIC_SUPPORT_ONLY_V5"
    with pytest.raises(RawDensityExtractionErrorV1):
        _q(sp.sqrt(2))
    with pytest.raises(RawDensityExtractionErrorV1):
        _q(True)


def test_cut_face_names_reuse_pure_topology_mapping_without_touching_chart_cache():
    vertex = kernel.SourceVertexId("vertex")
    other = kernel.SourceVertexId("other")
    triangle = kernel.SurfaceTriangleId("triangle")
    face = SimpleNamespace(vertex_cycle=(vertex, other, kernel.SourceVertexId("last")), triangle_ids=(triangle,))
    context = SimpleNamespace(frame=SimpleNamespace(planarity_certificate=SimpleNamespace(
        cut=SimpleNamespace(right_corners=(SimpleNamespace(triangle_id=triangle, vertex_id=vertex),)))), chart_name_cache={})
    names = _face_names(context, face)
    assert names[0].value == "vertex|cut:R" and names[1] == other
    assert context.chart_name_cache == {}


def test_missing_carrier_contract_is_internal_and_not_product_undecidable(monkeypatch):
    source, evaluation = _case_inputs(monkeypatch, *CASES[0])
    spec, selection = _selections(evaluation)[0]
    changed = replace(spec, source_relation_id=kernel.CornerRelationId("absent"))
    with pytest.raises(RawDensityExtractionErrorV1, match="absent relation/sector/angle"):
        extract_raw_density_spec_input(source, evaluation, changed, selection)


def test_raw_frontend_has_no_symbolic_constructors_or_literal_parser():
    from cftuv_envelope.reference import density_raw_v1
    tree = ast.parse(Path(density_raw_v1.__file__).read_text(encoding="utf-8"))
    banned = {"sympify", "parse_expr", "sqrt", "simplify", "expand", "expressions", "density_expressions",
              "support_segments_for_use", "_ideal_angular_support_data", "canonical_noise_fact", "build"}
    calls = {node.func.attr if isinstance(node.func, ast.Attribute) else node.func.id
             for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, (ast.Name, ast.Attribute))}
    assert not calls & banned


def test_first_density_bucket_keeps_one_hidden_ray_as_in_cover008():
    from cftuv_envelope.reference.density_oracle_v1 import RawDensitySelectionV1

    # Реальный cover.008 q4,C1,H1; source selector не использует H=C-1 при C=1.
    first = RawDensitySelectionV1(4, 1, "HUBER_EMANATED_COUNT_DENSITY_A_V1", "LINEAR_REFLEX_DENSITY_2_V1", 1)
    assert first.source_hidden_count == 1 and first.bucket_c == 1
    with pytest.raises(ValueError, match="H=max"):
        replace(first, source_hidden_count=2)
    with pytest.raises(ValueError, match="1 <= C"):
        replace(first, bucket_c=0)


@pytest.mark.parametrize("value", (-1.76, 4.68, 0.1, 1e-8, -1e20, 5e-324, -5e-324, -0.0))
def test_raw_float_atom_preserves_the_current_reference_decimal_float_law(value):
    from cftuv_envelope.reference.density_raw_v1 import _q
    from cftuv_envelope.reference.planar_types import _expr
    result = _q(value)
    assert sp.Rational(result.numerator, result.denominator) == _expr(value)
