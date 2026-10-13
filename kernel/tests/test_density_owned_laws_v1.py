"""Explicit shared controller and separated owned admission/value gateways."""

from fractions import Fraction
from decimal import localcontext

import pytest

from cftuv_envelope.contracts.analysis import TurnOrientation
from cftuv_envelope.reference.adaptive_density_fan import certify_density_bindings_and_adaptive_fallback
from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1, ExpandLimitsV1
from cftuv_envelope.reference.density_oracle_v1 import DensityOracleLimitsV1
from cftuv_envelope.reference.density_owned_laws_v1 import OwnedDensityLawsV1
from cftuv_envelope.reference.density_scalar_v2 import DensityScalarProfileUnsupported


def _laws():
    limits = DensityOracleLimitsV1(65536, 131072, 96, 1000000, ExpandLimitsV1(4096, 16, 65536))
    return OwnedDensityLawsV1(DensityConstructArenaV1(), limits)


def test_actual_q5_rotated_boundary_both_turn_and_owner_orientations():
    for owner in (-1, 1):
        for turn in (-1, 1):
            laws = _laws()
            metric = laws.metric((1, 0, 1), (1, 0, 1), owner)
            left = laws.vector(1, 0)
            right = laws.rotate_q(metric, left, turn, 5)
            assert laws.sign(laws.dual_dot(metric, right, right)-1) == 0
            assert laws.subturn_boundary(metric, left, right, 5)
            assert laws.subturn(metric, left, right, 5)
            assert laws.sign(laws.oriented_cross(metric, left, right)) == turn


def test_certified_opaque_nonzero_division_zero_numerator_but_numeric_base_first():
    laws = _laws()
    positive = laws.wrap(laws.arena.raw_cos(laws.arena.rational(0)))
    assert laws.fraction(laws.divide_nonzero(0, positive)) == 0
    quotient = laws.divide_nonzero(1, positive)
    assert not laws.arena.has_partial_domain(quotient.value)
    assert laws.fraction(quotient-quotient) == 0
    with pytest.raises(ZeroDivisionError):
        laws.divide_nonzero(0, laws.scalar(0))
    denominator = laws.sqrt(2)+laws.sqrt(3)+laws.sqrt(5)+laws.sqrt(7)
    with pytest.raises(DensityScalarProfileUnsupported):
        laws.divide_nonzero(0, denominator)


def test_mathematical_A2_composition_reaches_shell_without_changing_literal_admission():
    laws = _laws()
    cosine, sine = laws.principal_pair(Fraction(2, 3), 1, 3, 1)
    expression = cosine+2*sine
    with localcontext() as context:
        context.prec = 3
        first = laws.envelope(expression)
    with localcontext() as context:
        context.prec = 80
        second = laws.envelope(expression)
    assert first == second
    assert first.lower.as_tuple().exponent == -27
    assert laws.sign(expression) == 1
    assert laws.arena.owned_form(cosine.value).kind == "cos"
    assert laws.values.enclosure(cosine.value).lower < laws.values.enclosure(cosine.value).upper


def test_shared_controller_closes_owned_three_ray_numeric_window():
    laws = _laws()
    metric = laws.metric((1, 0, 1), (1, 0, 1), 1)
    component = laws.sqrt(Fraction(1, 2))
    ideal = (laws.vector(1, 0), laws.vector(component, component), laws.vector(0, 1))
    certificates, authority = certify_density_bindings_and_adaptive_fallback(
        metric, ideal, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, 2,
        binding_reasons=(None,), laws=laws,
    )
    assert authority is None
    assert certificates[0].bound_primitive_integer_vector == (1, 1)
    assert laws.budgets and laws.budgets[0][0] == "PREPARATION"
    assert laws.shell_outcomes


def test_owned_operator_rejects_symbolic_or_foreign_arena_inputs():
    import sympy as sp

    laws = _laws()
    with pytest.raises(TypeError):
        laws.scalar(sp.Rational(1, 2))
    with pytest.raises(ValueError):
        laws.scalar(_laws().scalar(1))


def test_A2_point_zero_does_not_override_literal_root_admission_failure():
    from cftuv_envelope.reference.density_binding_types import DirectionBindingCertificateUnproven

    laws = _laws()
    cosine, _ = laws.principal_pair(Fraction(1, 64), 1, 2, 1)
    base = Fraction(4, 3)*cosine-1
    root = laws.sqrt(base)
    mathematical = laws.values.enclosure(root.value)
    assert mathematical.lower == mathematical.upper == 0
    with pytest.raises(DirectionBindingCertificateUnproven):
        laws.envelope(root)


@pytest.mark.parametrize("folder", ("binding_noise_canonical_v1/building_patch114", "density4_exact_limit_v1/mesh2_patch2"))
def test_actual_raw_frontend_owned_normals_equal_old_source_and_evaluation_normals(monkeypatch, folder):
    from test_density_raw_v1 import _case_inputs, _selections
    from cftuv_envelope.reference.density_raw_v1 import extract_raw_density_spec_input
    from cftuv_envelope.reference.density_shell_v2 import density_shell_v2, DensityShellV2Candidate

    source, evaluation = _case_inputs(monkeypatch, folder, "decal_request_density4.json")
    for spec, selection in _selections(evaluation):
        raw = extract_raw_density_spec_input(source, evaluation, spec, selection)
        for context, geometry, declared in ((source, raw.source_geometry, True),
                                           (evaluation, raw.evaluation_geometry, evaluation.compilation.evaluation_geometry_binding is None)):
            laws = _laws()
            metric, incoming, outgoing = laws.raw_geometry(geometry, use_direction_declarations=declared)
            for incident, normal in ((geometry.incoming, incoming), (geometry.outgoing, outgoing)):
                use = context.uses_by_id[next(key for key in context.uses_by_id if key.value == incident.chain_use_id)]
                segments = context.support_segments_for_use(use.chain_use_id, spec.envelope_spec_id.value)
                old = next(segment.owner_normal for segment in segments if segment.support_id == incident.support_id)
                for owned, expression in zip(normal.expressions(), context.metric.density_expressions(old)):
                    candidate = density_shell_v2(expression)
                    assert isinstance(candidate, DensityShellV2Candidate)
                    assert laws.arena.scalar(owned.value).value == candidate.value
                assert laws.fraction(laws.dot_g(metric, normal, normal)) == 1
