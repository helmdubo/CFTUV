"""Конечная Numeric таблица и opaque границы owned trig canon."""

from fractions import Fraction

import pytest
import sympy as sp

from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1, ExpandLimitsV1
from cftuv_envelope.reference.density_scalar_v2 import ScalarV2
from cftuv_envelope.reference.density_trig_v1 import (
    TrigV1Error, TrigV1ErrorCode, owned_exact_q_trig, owned_principal_ray,
    owned_rational_pi_trig, principal_pi_multiple,
)
from cftuv_envelope.reference.density_sign import _density_exact_sign
from cftuv_envelope.reference.common import ReferenceGeometryError


@pytest.mark.parametrize("ratio,sign,m", [
    (0, 0, 6), (Fraction(1, 4), 1, 4), (Fraction(1, 4), -1, 8),
    (Fraction(1, 2), 1, 3), (Fraction(1, 2), -1, 9),
    (Fraction(3, 4), 1, 2), (Fraction(3, 4), -1, 10), (1, 1, 0), (1, -1, 12),
])
def test_principal_special_angles_are_exact_and_include_zero_and_pi(ratio, sign, m):
    assert principal_pi_multiple(ratio, sign) == Fraction(m, 12)
    arena = DensityConstructArenaV1()
    for n in range(2, 7):
        for j in range(n + 1):
            pair = owned_principal_ray(arena, ratio, sign, n, j)
            coefficient = sp.Rational(m * j, 12 * n)
            for component, function in zip(pair, (sp.cos, sp.sin), strict=True):
                assert sp.simplify(arena.to_sympy_readonly(component) - function(coefficient * sp.pi)) == 0


@pytest.mark.parametrize("numerator", range(-48, 49))
def test_full_circle_twelfths_numeric_values_signs_and_unit_norm(numerator):
    arena = DensityConstructArenaV1()
    multiple = Fraction(numerator, 12)
    cosine, sine = owned_rational_pi_trig(arena, multiple)
    c, s = arena.scalar(cosine), arena.scalar(sine)
    assert c is not None and s is not None
    assert c.mul(c).add(s.mul(s)) == ScalarV2.rational(1)
    for expression, function in ((cosine, sp.cos), (sine, sp.sin)):
        scalar = arena.scalar(expression)
        expected = function(sp.Rational(numerator, 12) * sp.pi)
        assert sp.simplify(arena.to_sympy_readonly(expression) - expected) == 0
        assert scalar.value.signum()[0] == _density_exact_sign(expected)


def test_sqrt_five_island_components_and_nested_q_five_are_separate():
    arena = DensityConstructArenaV1()
    cosine, sine = owned_rational_pi_trig(arena, Fraction(2, 5))
    assert arena.scalar(cosine) == ScalarV2.sqrt_rational(5).sub(1).scaled(Fraction(1, 4))
    assert arena.scalar(sine) is None
    assert arena.literal_tree(sine)[0] == "Mul"
    cosine, sine = owned_exact_q_trig(arena, 5)
    assert arena.scalar(cosine) == ScalarV2.sqrt_rational(5).add(1).scaled(Fraction(1, 4))
    assert arena.scalar(sine) is None
    tree = arena.literal_tree(sine)
    assert tree[0] == "Mul" and tree[2][0] == "Pow"
    assert sp.simplify(arena.to_sympy_readonly(sine) - sp.sin(sp.pi / 5)) == 0
    assert _density_exact_sign(arena.to_sympy_readonly(sine)) == 1


def test_unknown_irreducible_and_nested_root_angles_stay_original_raw_opaque():
    arena = DensityConstructArenaV1()
    for multiple in (Fraction(1, 16), Fraction(3, 16), Fraction(1, 7), Fraction(-31, 16)):
        pair = owned_rational_pi_trig(arena, multiple)
        angle = arena.scaled(arena.pi(), multiple)
        assert pair == (arena.raw_cos(angle), arena.raw_sin(angle))
        assert all(arena.scalar(component) is None for component in pair)
    assert principal_pi_multiple(Fraction(2, 3), 1) is None
    assert owned_principal_ray(arena, Fraction(2, 3), 1, 6, 0) is None


@pytest.mark.parametrize("q", range(2, 7))
def test_exact_q_values_retain_nested_boundary(q):
    arena = DensityConstructArenaV1()
    pair = owned_exact_q_trig(arena, q)
    for value, function in zip(pair, (sp.cos, sp.sin), strict=True):
        assert sp.simplify(arena.to_sympy_readonly(value) - function(sp.pi / q)) == 0
    assert arena.scalar(pair[1]) is not None if q != 5 else arena.scalar(pair[1]) is None


@pytest.mark.parametrize("ratio,sign,code", [
    (Fraction(-1, 4), -1, TrigV1ErrorCode.RATIO_OUTSIDE_CLOSED_UNIT_INTERVAL),
    (2, 1, TrigV1ErrorCode.RATIO_OUTSIDE_CLOSED_UNIT_INTERVAL),
    (0, 1, TrigV1ErrorCode.INCONSISTENT_DOT_SIGN),
    (1, 0, TrigV1ErrorCode.INCONSISTENT_DOT_SIGN),
    (Fraction(1, 2), True, TrigV1ErrorCode.INCONSISTENT_DOT_SIGN),
    (0.25, 1, TrigV1ErrorCode.NON_RATIONAL_INPUT),
])
def test_invalid_input_is_explicit_internal_error(ratio, sign, code):
    with pytest.raises(TrigV1Error) as failure:
        principal_pi_multiple(ratio, sign)
    assert failure.value.code is code


def test_ray_guards_precede_angle_recognition_and_endpoints_are_numeric():
    arena = DensityConstructArenaV1()
    with pytest.raises(TrigV1Error) as failure:
        owned_principal_ray(arena, Fraction(-1), 0, 7, 8)
    assert failure.value.code is TrigV1ErrorCode.INVALID_SUBDIVISION
    with pytest.raises(TrigV1Error) as failure:
        owned_principal_ray(arena, Fraction(-1), 0, 6, 7)
    assert failure.value.code is TrigV1ErrorCode.INVALID_ORDINAL
    for sign in (-1, 1):
        pair = owned_principal_ray(arena, 1, sign, 6, 6)
        assert [arena.whole_rational(value) for value in pair] == [sign, 0]


_ACTUAL_PRINCIPAL_ANGLES = (
    "1/36", "1/30", "1/24", "1/20", "1/18", "1/16", "1/15", "1/12", "1/10",
    "1/9", "1/8", "2/15", "5/36", "3/20", "1/6", "3/16", "1/5", "5/24",
    "2/9", "1/4", "4/15", "5/18", "3/10", "1/3", "3/8", "2/5", "5/12",
    "4/9", "9/20", "1/2", "8/15", "5/9", "9/16", "3/5", "5/8", "2/3", "25/36",
)


def _old_norm_outcome(expression):
    try:
        return "sign", _density_exact_sign(expression)
    except ReferenceGeometryError as error:
        return "error", error.outcome.value


@pytest.mark.parametrize("angle", _ACTUAL_PRINCIPAL_ANGLES + ("7/30", "7/24", "7/20", "11/30", "13/30", "11/24"))
def test_actual_old_special_producer_norm_outcome_matches_owned_finite_expansion(angle):
    multiple = Fraction(angle)
    theta = sp.Rational(multiple.numerator, multiple.denominator) * sp.pi
    old_cosine, old_sine = sp.cos(theta), sp.sin(theta)
    expected = _old_norm_outcome(old_cosine**2 + old_sine**2 - 1)
    arena = DensityConstructArenaV1()
    cosine, sine = owned_rational_pi_trig(arena, multiple)
    residual = arena.sub(arena.add(arena.integer_power(cosine, 2),
                                  arena.integer_power(sine, 2)), arena.rational(1))
    expanded = arena.finite_expand(residual, ExpandLimitsV1(4096, 4, 65536))
    if expected == ("sign", 0):
        assert arena.whole_rational(expanded) == 0
    else:
        assert arena.scalar(cosine) is None and arena.literal_tree(cosine)[0] == "cos"
    assert _old_norm_outcome(arena.to_sympy_readonly(expanded)) == expected
    # Проверяем настоящие source constant values, не только совпадение двух AST.
    assert sp.simplify(arena.to_sympy_readonly(cosine) - old_cosine) == 0
    assert sp.simplify(arena.to_sympy_readonly(sine) - old_sine) == 0
