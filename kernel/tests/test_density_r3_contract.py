"""R3: граница успешных исходов эталона, а не только математически верных знаков."""

from decimal import Decimal
from dataclasses import replace
from fractions import Fraction
from math import isqrt

import pytest
import sympy as sp

from cftuv_envelope.reference.angular import _density_exact_sign
from cftuv_envelope.reference.common import ReferenceGeometryError
from cftuv_envelope.reference.contracts import ReferenceOutcome
from cftuv_envelope.reference.direction_binding import _density_decimal_envelope
from cftuv_envelope.reference.planar_types import (
    ExactQuadraticFieldUnsupported,
    exact_quadratic_value,
)


def test_equal_decimal_values_with_different_scales_have_different_authority_ids():
    from cftuv_envelope.codec import to_canonical_data
    from cftuv_envelope.contracts.envelopes import AdaptiveRationalFanOrdinalWindowV2
    from cftuv_envelope.reference.adaptive_density_atlas import _exact_decimal_interval
    from cftuv_envelope.reference.adaptive_density_fan import _authority_id

    lower = _exact_decimal_interval(0)
    density_lower = _density_decimal_envelope(sp.Integer(0))
    assert lower == density_lower
    assert lower.lower.as_tuple().exponent == 0
    assert density_lower.lower.as_tuple().exponent == -27
    window = AdaptiveRationalFanOrdinalWindowV2(
        ordinal=1, use_x_denominator=True, denominator_sign=1,
        full_lower_slope_envelope=lower,
        full_upper_slope_envelope=_exact_decimal_interval(1),
        termination_lower_slope=(1, 4), termination_upper_slope=(3, 4),
        certified_termination_width=(1, 2),
        admissible_lower_outward=(0, 1), admissible_lower_inward=(1, 8),
        admissible_upper_inward=(7, 8), admissible_upper_outward=(1, 1),
    )
    scaled = replace(window, full_lower_slope_envelope=density_lower)
    assert to_canonical_data(window) == to_canonical_data(scaled)
    # ID предшествует codec и читает repr Decimal; одной рациональной величины мало.
    assert _authority_id(3, 2, ((2, 1),), (window,)) != _authority_id(
        3, 2, ((2, 1),), (scaled,)
    )


def test_outside_radical_field_does_not_imply_undecidable_density_sign():
    expression = sp.sqrt(2 + sp.sqrt(2))
    with pytest.raises(ExactQuadraticFieldUnsupported):
        exact_quadratic_value(expression)
    assert _density_exact_sign(expression) == 1
    assert _density_exact_sign(-expression) == -1


@pytest.mark.parametrize("upper", (False, True))
def test_more_precise_nested_root_sign_must_not_silently_expand_oracle_admission(upper):
    denominator = 1 << 256
    numerator = 213955882546153770510803628816683935098316969411928952863250352738335112326797
    # Целочисленный свидетель: p/D < sqrt(2+sqrt(2)) < (p+1)/D.
    # Обе стороны после первого квадрата положительны; второй квадрат сохраняет порядок.
    lower_squared = numerator * numerator - 2 * denominator * denominator
    upper_squared = (numerator + 1) ** 2 - 2 * denominator * denominator
    assert 0 < lower_squared
    assert lower_squared**2 < 2 * denominator**4 < upper_squared**2
    expression = sp.sqrt(2 + sp.sqrt(2)) - sp.Rational(numerator + int(upper), denominator)
    with pytest.raises(ReferenceGeometryError) as caught:
        _density_exact_sign(expression)
    assert caught.value.outcome is ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE
    assert str(caught.value) == "Density A exact sign is not certified without generic factorization"


@pytest.mark.parametrize("numerator,center", ((1657, "0.3236328125"), (3647, "0.7123046875")))
def test_pre_a1_rational_grid_straw_is_an_observable_two_sided_window(numerator, center):
    envelope = _density_decimal_envelope(sp.Rational(numerator, 5120))
    node, ulp = Decimal(center), Decimal("1e-27")
    assert envelope.lower == node - ulp
    assert envelope.upper == node + ulp
    assert envelope.absolute_error_bound == 2 * ulp


def test_exact_quadratic_fallback_still_resolves_a_sign_below_interval_precision():
    denominator = 1 << 256
    numerator = isqrt(2 * denominator**2)
    assert _density_exact_sign(sp.sqrt(2) - sp.Rational(numerator, denominator)) == 1
    assert _density_exact_sign(sp.sqrt(2) - sp.Rational(numerator + 1, denominator)) == -1


@pytest.mark.parametrize("inside_window", (False, True))
def test_principal_angle_exact_zero_does_not_erase_legacy_admission(inside_window):
    cosine, sine = sp.Rational(3, 4), sp.sqrt(7) / 4
    assert cosine**2 + sine**2 == 1
    assert sp.expand(4*cosine**3 - 3*cosine) == -sp.Rational(9, 16)
    assert sp.expand(3*sine - 4*sine**3) == 5*sp.sqrt(7)/16
    assert sp.Rational(1, 2) < cosine < 1
    # phi в (0,pi/3), поэтому 3phi в главной ветви (0,pi).
    # G=diag(1,7), опоры (1,0),(-9,5): внутри ordinal с кандидатом (1,1)
    # dual-dot(candidate, ideal-previous) = cos(phi)-1+sin(phi)/sqrt(7) = 0.
    phi = sp.atan2(5*sp.sqrt(7)/16, -sp.Rational(9, 16))/3
    expression = (sp.cos(phi)-1+sp.sin(phi)/sp.sqrt(7)
                  if inside_window else sp.cos(phi)-sp.Rational(3, 4))
    with pytest.raises(ReferenceGeometryError) as caught:
        _density_exact_sign(expression)
    assert caught.value.outcome is ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE
    assert str(caught.value) == "Density A exact sign is not certified without generic factorization"


def test_correctly_rounded_sine_would_silently_change_the_legacy_refusal():
    t = Fraction(365, 8192)
    a = 1041546121700086938441153703724603043422243247291
    unit = Fraction(1, 2**164)
    # 60 членов чередующегося ряда дают нижнюю границу; следующий член — остаток сверху.
    term, lower = t, t
    for k in range(1, 60):
        term *= -t*t / ((2*k)*(2*k+1))
        lower += term
    next_term = term * (-t*t) / (120*121)
    assert next_term > 0
    assert (a+1)*unit < lower < lower+next_term < (a+2)*unit
    # Legacy mpi_sin имеет нижний endpoint a*unit, на ulp шире истинного floor.
    expression = sp.sin(sp.Rational(t.numerator, t.denominator)) - sp.Rational(a, 2**164)
    with pytest.raises(ReferenceGeometryError) as caught:
        _density_exact_sign(expression)
    assert caught.value.outcome is ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE
