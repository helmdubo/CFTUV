"""Печать R3 сохраняет Decimal quantum, виды IR и идентичность власти."""

from dataclasses import replace
from decimal import Decimal, Inexact, ROUND_UP, localcontext
from fractions import Fraction
from types import SimpleNamespace

import pytest

from cftuv_envelope.contracts import envelopes as ir
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.ids import SelectionCertificateId
from cftuv_envelope.numeric import ExactRatioV1
from cftuv_envelope.reference import density_oracle_v1 as f
from cftuv_envelope.reference import density_result_v1 as result


SELECTION = SelectionCertificateId("selection:result-test")
PREDICATES = ("BINDING_INSIDE_OWN_ORDINAL_WINDOW", "BINDING_MONOTONE")


def interval(lower, upper=None):
    upper = lower if upper is None else upper
    return f.DecimalIntervalFactsV1(
        f.DecimalValueFactsV1(lower, -27), f.DecimalValueFactsV1(upper, -27),
        "CLOSED", "CLOSED", f.DecimalValueFactsV1(upper - lower, -27),
    )


def local_window(ordinal=1):
    return f.LocalWindowFactsV1(
        ordinal, True, 1, interval(0), interval(10**27),
        Fraction(1, 4), Fraction(3, 4), Fraction(1, 2),
        Fraction(0), Fraction(1, 4), Fraction(3, 4), Fraction(1),
    )


def atlas_window():
    return f.AtlasWindowFactsV1(
        1, (f.ChartPieceFactsV1(0, True, 1, interval(0), interval(10**27),
                               False, False, True, "NONE"),),
        0, Fraction(1, 4), Fraction(3, 4), Fraction(1, 2),
        Fraction(0), Fraction(1, 4), Fraction(3, 4), Fraction(1),
    )


def adaptive(window):
    return f.AdaptiveFanAuthorityFactsV1(
        4, 2, 1, 4, ((1, 1),), (None,), (window,),
        f.FareyHeightWitnessFactsV1(1, 1, (0,)), PREDICATES,
    )


@pytest.mark.parametrize("coefficient,exponent", [(0, -27), (10**60 + 1657, -27), (-125, -25), (120, -27)])
def test_decimal_quantum_and_interval_are_independent_of_decimal_context(coefficient, exponent):
    value = f.DecimalValueFactsV1(coefficient, exponent)
    expected = Decimal(f"{coefficient}E{exponent}")
    for precision in (1, 7, 28, 80):
        with localcontext() as context:
            context.prec = precision
            context.rounding = ROUND_UP
            context.traps[Inexact] = True
            assert result.format_decimal(value).as_tuple() == expected.as_tuple()
            shell = result.format_interval(interval(10**27 - 1, 10**27 + 1))
            assert shell.lower.as_tuple() == Decimal("0.999999999999999999999999999").as_tuple()
            assert shell.upper.as_tuple() == Decimal("1.000000000000000000000000001").as_tuple()
            assert shell.absolute_error_bound.as_tuple() == Decimal("2E-27").as_tuple()


def test_certificate_tag_preserves_reason_and_signed_rounded_width():
    certificate = f.BindingCertificateFactsV1(
        (1, 2), interval(0), interval(10**27), f.DecimalValueFactsV1(-120, -27), PREDICATES,
    )
    printed = result.format_certificate(certificate)
    assert type(printed) is ir.DirectionBindingCertificateV1
    assert printed.certified_window_width_lower_bound.as_tuple() == Decimal("-1.20E-25").as_tuple()
    reason = "EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL"
    evaluated = result.format_certificate(replace(certificate, binding_reason=reason))
    assert type(evaluated) is ir.EvaluationGeometryDirectionBindingCertificateV1
    assert evaluated.binding_reason is ir.DirectionBindingReasonV1(reason)
    assert evaluated.bound_primitive_integer_vector == printed.bound_primitive_integer_vector
    assert result.format_certificate(None) is None


@pytest.mark.parametrize("window", [local_window(), atlas_window()])
def test_adaptive_formatter_retains_source_identity_and_all_window_fields(window):
    from cftuv_envelope.reference.adaptive_density_fan import _authority_id

    source = adaptive(window)
    authority = result.format_authority(source, SELECTION)
    assert authority.authority_id == _authority_id(4, 2, ((1, 1),), authority.ordinal_windows)
    printed = authority.ordinal_windows[0]
    assert printed.ordinal == 1
    assert printed.termination_lower_slope == (1, 4)
    assert printed.termination_upper_slope == (3, 4)
    assert printed.certified_termination_width == (1, 2)
    assert printed.admissible_lower_outward == (0, 1)
    assert printed.admissible_upper_outward == (1, 1)
    assert authority.previous_height_witness == ir.AdaptiveFareyHeightRangeWitnessV2(1, 1, (0,))
    with localcontext() as context:
        context.prec = 1
        assert canonical_json_bytes(result.format_authority(source, SELECTION)) == canonical_json_bytes(authority)
    assert result.format_authority(replace(source, authority_id=authority.authority_id), SELECTION) == authority
    with pytest.raises(result.DensityResultInvalid, match="identity"):
        result.format_authority(replace(source, authority_id="wrong"), SELECTION)


def test_canonical_authority_identity_is_the_existing_producer_identity():
    from cftuv_envelope.reference.canonical_fan_rays import _authority, CANONICAL_FAN_RAYS_PREDICATES

    relation = ir.CanonicalReflexAngleRelationV1.CANONICAL_REFLEX_EXCESS_PI_OVER_2
    law = ir.CanonicalFanRaysLawV1.CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1
    rays, vectors = ((1, 1),), ((2, 3),)
    source = f.CanonicalFanAuthorityFactsV1(
        law.value, relation.value, Fraction(1, 2), 1, 4, rays, vectors,
        tuple(sorted(CANONICAL_FAN_RAYS_PREDICATES)),
    )
    expected = _authority(SimpleNamespace(certificate_id=SELECTION), relation, Fraction(1, 2), 1, 4, rays, vectors)
    printed = result.format_authority(source, SELECTION)
    assert printed == expected
    assert canonical_json_bytes(printed) == canonical_json_bytes(expected)
    other = result.format_authority(source, SelectionCertificateId("another-selection"))
    assert other.authority_id != expected.authority_id


def test_lift_only_attaches_the_declared_selection_identity():
    law = ir.EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1
    source = f.CountLiftFactsV1(law.value, 2, 3, 6, "ZERO", Fraction(0), 2, PREDICATES)
    expected = ir.EvaluationGeometrySubturnCountLiftV1(
        law, SELECTION, 2, 3, 6, ir.ExactTurnSignV1.ZERO, ExactRatioV1(0, 1), 2, frozenset(PREDICATES),
    )
    assert result.format_count_lift(source, SELECTION) == expected
    assert result.format_count_lift(None, SELECTION) is None


@pytest.mark.parametrize("mutate", [
    lambda item: replace(item, ordinal_windows=(replace(local_window(), ordinal=2),)),
    lambda item: replace(item, binding_reasons=()),
    lambda item: replace(item, bound_primitive_integer_vectors=((True, 1),)),
    lambda item: replace(item, proven_predicates=("B", "A")),
    lambda item: replace(item, proven_predicates=("A", "A")),
])
def test_bad_result_structure_is_not_silently_reordered_or_repaired(mutate):
    with pytest.raises(result.DensityResultInvalid):
        result.format_authority(mutate(adaptive(local_window())), SELECTION)


def test_invalid_interval_stays_an_invalid_result():
    with pytest.raises(ValueError, match="lower bound exceeds"):
        result.format_interval(replace(interval(0), lower=f.DecimalValueFactsV1(2, -27)))
    with pytest.raises(ValueError):
        result.format_interval(replace(interval(0), lower_kind="UNRECOGNIZED"))
    with pytest.raises(result.DensityResultInvalid, match="piece"):
        result.format_window(replace(atlas_window(), pieces=(replace(atlas_window().pieces[0], piece_index=1),)))
