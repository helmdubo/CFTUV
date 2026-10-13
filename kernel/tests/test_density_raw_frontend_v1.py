"""Fresh owned raw producer: actual geometry, named decisions, and warm prices."""

from dataclasses import replace
from fractions import Fraction

import pytest

from cftuv_envelope.reference import density_oracle_v1 as facts
from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1, ExpandLimitsV1
from cftuv_envelope.reference.density_owned_laws_v1 import OwnedDensityLawsV1
from cftuv_envelope.reference.density_raw_frontend_v1 import RawDensityFrontendV1
from cftuv_envelope.reference.density_raw_v1 import extract_raw_density_spec_input
from cftuv_envelope.reference.density_shell_v2 import DENSITY_SHELL_CANON_V2_CANDIDATE


def _frontend():
    ids = facts.DensityOracleCanonV1(facts.DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE,
                                    facts.DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1)
    limits = facts.DensityOracleLimitsV1(65536, 131072, 96, 1000000, ExpandLimitsV1(4096, 16, 65536))
    return RawDensityFrontendV1(ids, limits)


def _forty_five_degree_raw():
    point = facts.RawDensityPointV1
    metric = facts.RawDensityMetricV1((1, 0, 1), (1, 0, 1), 1)
    incoming = facts.RawDensityIncidentV1(point(0, 1), point(0, 0), True, "corner", "previous", "corner")
    outgoing = facts.RawDensityIncidentV1(point(0, 0), point(1, -1), True, "corner", "corner", "next")
    geometry = facts.RawDensityGeometryV1(metric, incoming, outgoing)
    selection = facts.RawDensitySelectionV1(4, 1, "HUBER_EMANATED_COUNT_DENSITY_A_V1",
        "LINEAR_REFLEX_DENSITY_2_V1", 1, "selection")
    canonical = facts.RawDensityCanonicalFactsV1((Fraction(1, 4), Fraction(1, 4)), None, (), (("binding_present", False),))
    return facts.RawDensitySpecInput(geometry, geometry, 1, selection, canonical, spec_id="spec")


@pytest.mark.parametrize("folder", ("binding_noise_canonical_v1/building_patch114", "density4_exact_limit_v1/mesh2_patch2"))
def test_actual_owned_raw_count_lift_and_canonical_facts_equal_reference_without_old_geometry(monkeypatch, folder):
    from test_density_raw_v1 import _case_inputs, _selections, _ban_geometry
    from cftuv_envelope.reference import compile as compiler, angular, density_turn, evaluation_binding_noise as noise
    from cftuv_envelope.reference.canonical_fan_rays import canonical_fan_rays_decision
    from cftuv_envelope.reference.density_result_v1 import format_count_lift, format_authority, format_noise_record

    source, evaluation = _case_inputs(monkeypatch, folder, "decal_request_density4.json")
    cases = []
    for spec, selection in _selections(evaluation):
        raw = extract_raw_density_spec_input(source, evaluation, spec, selection)
        final, lift, ideal = compiler._evaluation_density_spec(evaluation, spec, selection, raw.selection.q)
        authority = canonical_fan_rays_decision(evaluation, final, selection).authority
        record = None
        if lift is not None and lift.lift_law.value.endswith("AT_CANONICAL_EXACT_LIMIT_V1"):
            record = noise.build_noise_record(evaluation, final, selection, noise.EvaluationBindingNoiseEffectV1.CANONICAL_COUNT_LIFT, ideal)
        cases.append((raw, selection, final, lift, authority, record))
    assert cases
    _ban_geometry(monkeypatch)

    def forbidden(*args, **kwargs):
        raise AssertionError("owned raw frontend called an old symbolic geometry producer")

    for module, name in ((compiler, "_evaluation_density_spec"), (angular, "_interpolated_normals"),
                         (angular, "_incident_normal"), (density_turn, "_huber_density_interpolated_normals"),
                         (noise, "canonical_noise_applicability")):
        monkeypatch.setattr(module, name, forbidden)
    for raw, selection, final, lift, authority, record in cases:
        frontend = _frontend()
        result = frontend.evaluate(raw)
        assert result.effective_hidden_count == final.resolved_hidden_edge_count
        assert format_count_lift(result.count_lift, selection.certificate_id) == lift
        assert format_authority(result.authority, selection.certificate_id) == authority
        assert format_noise_record(result.noise_record, final.envelope_spec_id, selection.certificate_id) == record
        before = len(frontend.laws.budgets), len(frontend.laws.shell_outcomes)
        warm = frontend.evaluate(raw)
        assert warm == replace(result, ordered_work_articles=())
        assert before == (len(frontend.laws.budgets), len(frontend.laws.shell_outcomes))


def test_owned_radical_rationality_proves_proportional_components_without_inverse():
    laws = _frontend().laws
    common = 1+laws.sqrt(3)
    assert laws.rational_direction(laws.vector(common, 2*common)) == ((1, 2), True)
    assert laws.rational_direction(laws.vector(laws.sqrt(2), laws.sqrt(3))) == (None, False)
    cosine, sine = laws.principal_pair(Fraction(2, 3), 1, 3, 1)
    assert laws.rational_direction(laws.vector(cosine, sine)) == (None, None)


@pytest.mark.parametrize("gram,inverse", (((2, 0, 1), (Fraction(1, 2), 0, 1)),
                                         ((2, 1, 3), (Fraction(3, 5), Fraction(-1, 5), Fraction(2, 5)))))
def test_owned_primal_seam_roundtrip_preserves_skew_gram_covectors(gram, inverse):
    laws = _frontend().laws
    metric = laws.metric(gram, inverse, 1)
    normal = laws.vector(laws.sqrt(2), 1+laws.sqrt(3))
    covector = laws.covector(metric, normal)
    recovered = laws.primal(metric, covector)
    assert all(laws.sign(a-b) == 0 for a, b in zip(normal.expressions(), recovered.expressions()))


@pytest.mark.parametrize("q", range(2, 7))
def test_owned_count_threshold_handles_mirror_sign_and_exact_quarter_turn(q):
    laws = _frontend().laws
    for count in range(q):
        assert laws.turn_count_feasible(0, Fraction(0), count, q) == (2*(count+1) >= q)
        assert laws.turn_count_feasible(1, Fraction(1), count, q)
        assert laws.turn_count_feasible(-1, Fraction(1), count, q) == (count+1 >= q)


def test_real_preparation_failure_retains_ordered_prices_and_is_not_memoized(monkeypatch):
    from test_density_raw_v1 import _case_inputs, _selections
    from cftuv_envelope.reference.density_result_v1 import format_density_failure

    source, evaluation = _case_inputs(monkeypatch, "binding_noise_canonical_v1/building_patch114", "decal_request_density2.json")
    template = _frontend()
    frontend = RawDensityFrontendV1(template.canon_ids, replace(template.limits, preparation_cap=0))
    observed = []
    for spec, selection in _selections(evaluation):
        raw = extract_raw_density_spec_input(source, evaluation, spec, selection)
        result = frontend.evaluate(raw)
        if result.failure is None:
            continue
        assert result.failure.exception_type == "DensityRationalAuthorityExhausted"
        assert result.outcome == "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"
        assert result.ordered_work_articles and all(p.cap == 0 for p in result.ordered_work_articles)
        assert result.canon_usage
        assert format_density_failure(result.failure).diagnostics[0].message == result.failure.message
        count = len(frontend.laws.budgets)
        repeated = frontend.evaluate(raw)
        assert repeated == result
        assert len(frontend.laws.budgets) == count+len(result.ordered_work_articles)
        observed.append(result)
    assert observed


def test_successful_leaf_attempt_replays_shell_facts_without_new_work(monkeypatch):
    from cftuv_envelope.contracts.analysis import TurnOrientation
    from cftuv_envelope.reference.density_controller_v1 import bind_owned_density_fan

    frontend = _frontend()
    laws = frontend.laws
    metric = laws.metric((1, 0, 1), (1, 0, 1), 1)
    component = laws.sqrt(Fraction(1, 2))
    fan = laws.vector(1, 0), laws.vector(component, component), laws.vector(0, 1)
    kwargs = dict(lifted=False, law="VORONOI_OF_NEIGHBOUR_MIDPOINTS", spec_id="first-spec",
                  refusals=[], canon_ids=frontend.canon_ids)
    cold = bind_owned_density_fan(laws, metric, fan, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, 2, (None,), **kwargs)
    assert laws.shell_outcomes
    captured = tuple(laws.shell_outcomes)
    first = len(laws.shell_outcomes)
    budget_count = len(laws.budgets)

    def forbidden(*args, **kwargs):
        raise AssertionError("warm facts replay attempted a numerical evaluator")

    monkeypatch.setattr(laws, "envelope", forbidden)
    warm = bind_owned_density_fan(laws, metric, fan, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, 2, (None,), **{**kwargs, "spec_id": "second-spec"})
    assert warm is cold
    assert len(laws.budgets) == budget_count
    assert tuple(laws.shell_outcomes[first:]) == captured


@pytest.mark.parametrize("profile", ("inverse", "opaque", "zero"))
def test_internal_candidate_failure_never_becomes_a_geometric_refusal(monkeypatch, profile):
    from test_density_raw_v1 import _case_inputs, _selections
    from cftuv_envelope.reference.density_scalar_v2 import DensityScalarProfileUnsupported, DensityScalarProfileCode
    from cftuv_envelope.reference.density_angle_value_v1 import AngleValueError, AngleValueErrorCode

    source, evaluation = _case_inputs(monkeypatch, "density4_exact_limit_v1/mesh2_patch2", "decal_request_density4.json")
    spec, selection = _selections(evaluation)[0]
    raw = extract_raw_density_spec_input(source, evaluation, spec, selection)
    error = (DensityScalarProfileUnsupported(DensityScalarProfileCode.INVERSE_CLASS_LIMIT, "explicit candidate profile")
             if profile == "inverse" else AngleValueError(AngleValueErrorCode.UNREGISTERED_OPAQUE)
             if profile == "opaque" else ZeroDivisionError("explicit partial-domain failure"))
    frontend = _frontend()

    def unsupported(raw):
        raise error

    monkeypatch.setattr(frontend, "_compute", unsupported)
    with pytest.raises(type(error), match=str(error)):
        frontend.evaluate(raw)


def test_raw_narrow_refusal_voronoi_success_retries_only_failed_attempt_on_warm(monkeypatch):
    template = _frontend()
    frontend = RawDensityFrontendV1(template.canon_ids, replace(template.limits, search_cap=1))
    raw = _forty_five_degree_raw()
    cold = frontend.evaluate(raw)
    warm = frontend.evaluate(raw)
    assert cold.outcome == warm.outcome == "OK"
    assert cold.binding_certificates[0].bound_primitive_integer_vector == (2, 1)
    assert cold.named_refusals == warm.named_refusals
    assert len(cold.ordered_work_articles) == 3
    assert warm.ordered_work_articles == cold.ordered_work_articles[:2]
    assert warm == replace(cold, ordered_work_articles=warm.ordered_work_articles)
    # Тот же реальный leaf-controller: сравниваем также расход, а не только свой cache.
    import sympy as sp
    from cftuv_envelope import TurnOrientation, DirectionBindingReasonV1
    from cftuv_envelope.reference import adaptive_density_fan as fan
    from cftuv_envelope.reference.adaptive_density_atlas import DensityExactWorkBudget
    from cftuv_envelope.reference.density_fan_binding import bind_density_fan
    from cftuv_envelope.reference.density_vectors import _density_runtime_vector
    from cftuv_envelope.reference.metric import ExactPlanarMetric

    identity = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
    metric = ExactPlanarMetric(identity, identity, 1)
    ideal = tuple(_density_runtime_vector(*pair) for pair in (
        (1, 0), (sp.sqrt(2+sp.sqrt(2))/2, sp.sqrt(2-sp.sqrt(2))/2), (sp.sqrt(2)/2, sp.sqrt(2)/2)))
    original = DensityExactWorkBudget.__init__
    observed = []

    def remember(budget, exhausted, cap):
        original(budget, exhausted, cap)
        observed.append(budget)

    monkeypatch.setattr(DensityExactWorkBudget, "__init__", remember)
    monkeypatch.setattr(fan, "_DENSITY_EXACT_WORK_CAP", 1)
    for result in (cold, warm):
        observed.clear()
        refusals = []
        certificates, authority = bind_density_fan(metric, ideal, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, 4,
            (DirectionBindingReasonV1.SOURCE_DIRECTION_IRRATIONAL,), lifted=False, law="NARROW_ROTATION_BAND",
            spec_id="spec", refusals=refusals)
        assert authority is None and certificates[0].bound_primitive_integer_vector == (2, 1)
        assert tuple(refusals) == result.named_refusals
        assert [(b.cap, b.counters()) for b in observed] == [(p.cap, p.counters) for p in result.ordered_work_articles]


def test_pure_batch_has_one_invocation_memo_and_stops_at_first_named_failure():
    from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch

    template = _frontend()
    raw = _forty_five_degree_raw()
    second = replace(raw, spec_id="spec2")
    batch = facts.RawDensityBatchInputV1((raw, second), template.canon_ids, replace(template.limits, search_cap=1),
                                        "NARROW_ROTATION_BAND")
    results = evaluate_raw_density_batch(batch)
    assert len(results) == 2
    assert len(results[0].ordered_work_articles) == 3
    assert len(results[1].ordered_work_articles) == 2
    assert results[1].named_refusals[0][0] == "spec2"
    assert evaluate_raw_density_batch(batch) == results
    failed = evaluate_raw_density_batch(replace(batch, limits=replace(batch.limits, preparation_cap=0)))
    assert len(failed) == 1 and failed[0].failure is not None
