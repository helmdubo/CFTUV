"""A1 subset candidate: value identity, exact quantum, explicit applicability."""

from dataclasses import FrozenInstanceError
from decimal import Decimal, localcontext
from fractions import Fraction

import pytest
import sympy as sp

from cftuv_envelope.reference.density_shell_v2 import (
    DENSITY_SHELL_CANON_V2_CANDIDATE,
    DensityShellNotApplicableCode,
    DensityShellOutcomeCode,
    DensityShellV2Candidate,
    DensityShellV2NotApplicable,
    _shell,
    density_shell_v2,
)
from cftuv_envelope.reference.radical_v2 import RadicalSumV2


def root(number):
    return sp.Pow(sp.Integer(number), sp.Rational(1, 2), evaluate=False)


def accepted(expression):
    result = density_shell_v2(expression)
    assert isinstance(result, DensityShellV2Candidate)
    assert result.canon == DENSITY_SHELL_CANON_V2_CANDIDATE
    return result


def observation(result):
    return (result.value, result.enclosure160, result.shell,
            tuple(x.as_tuple() for x in (
                result.shell.lower, result.shell.upper, result.shell.absolute_error_bound)))


def test_huge_squarefree_equal_value_forms_and_hidden_square_have_one_shell():
    p, q, r = 2**89 - 1, 2**61 - 1, 2**31 - 1
    forms = (
        sp.Mul(root(q*r), root(p*q), evaluate=False),
        sp.Mul(sp.Integer(q), root(p*r), evaluate=False),
        root(p*q*q*r),
        sp.sqrt(q*r)*sp.sqrt(p*q),
    )
    results = [accepted(expression) for expression in forms]
    assert len({observation(result) for result in results}) == 1
    hidden = accepted(root(p*p*q))
    explicit = accepted(sp.Mul(sp.Integer(p), root(q), evaluate=False))
    assert observation(hidden) == observation(explicit)


def test_raw_add_order_distributive_forms_and_cancellation_have_one_shell():
    raw = sp.Mul(sp.Add(root(2), root(3), evaluate=False), root(2), evaluate=False)
    distributed = sp.Add(root(6), sp.Integer(2), evaluate=False)
    assert observation(accepted(raw)) == observation(accepted(distributed))
    zero = sp.Add(root(18), sp.Mul(-3, root(2), evaluate=False), evaluate=False)
    result = accepted(zero)
    assert result.value.is_zero
    assert result.enclosure160 == (0, 0)
    assert result.shell.lower.as_tuple() == Decimal("0E-27").as_tuple()
    assert result.shell.absolute_error_bound.as_tuple() == Decimal("0E-27").as_tuple()


@pytest.mark.parametrize("exponent", (-5, -1, 0, 1, 2, 7))
def test_integer_power_matches_repeated_v2_operations(exponent):
    base = accepted(sp.Add(1, root(2), evaluate=False)).value
    factor = base.inverse()[0] if exponent < 0 else base
    expected = RadicalSumV2.rational(1)
    for _ in range(abs(exponent)):
        expected = expected.mul(factor)[0]
    expression = sp.Pow(sp.Add(1, root(2), evaluate=False), exponent, evaluate=False)
    assert accepted(expression).value == expected


@pytest.mark.parametrize("exponent", (sp.Rational(3, 2), sp.Rational(-3, 2)))
def test_rational_half_integer_power(exponent):
    expression = sp.Pow(sp.Rational(2, 9), exponent, evaluate=False)
    expected = sp.Mul(sp.Rational(2, 27), root(2), evaluate=False)
    if exponent < 0:
        expected = sp.Pow(expected, -1, evaluate=False)
    assert observation(accepted(expression)) == observation(accepted(expected))


def test_three_class_inverse_uses_v2_conjugation():
    expression = sp.Add(1, root(2), root(3), evaluate=False)
    denominator = accepted(expression)
    inverse = accepted(sp.Pow(expression, -1, evaluate=False))
    assert inverse.value.mul(denominator.value)[0] == RadicalSumV2.rational(1)
    assert inverse.work.conjugation_products > 0


@pytest.mark.parametrize("numerator,center", (
    (1657, "0.323632812500000000000000000"),
    (3647, "0.712304687500000000000000000"),
))
def test_exact_rational_grid_collapses_legacy_two_sided_window(numerator, center):
    result = accepted(sp.Rational(numerator, 5120))
    assert result.enclosure160 == (Fraction(numerator, 5120),)*2
    assert result.shell.lower.as_tuple() == Decimal(center).as_tuple()
    assert result.shell.upper.as_tuple() == Decimal(center).as_tuple()
    assert result.shell.absolute_error_bound.as_tuple() == Decimal("0E-27").as_tuple()


@pytest.mark.parametrize("expression", (sp.Rational(-1, 3), -root(2), sp.Rational(1, 3)))
def test_floor_ceil_for_both_signs_are_outward(expression):
    result = accepted(expression)
    lower, upper = result.enclosure160
    lo, hi = result.shell.lower_units, result.shell.upper_units
    scale = 10**27
    assert Fraction(lo, scale) <= lower < Fraction(lo+1, scale)
    assert Fraction(hi-1, scale) < upper <= Fraction(hi, scale)
    assert Fraction(result.shell.absolute_error_bound) == Fraction(hi-lo, scale)


@pytest.mark.parametrize("precision", (1, 2, 7, 28, 90))
def test_endpoints_quantum_and_huge_width_ignore_decimal_context(precision):
    bounds = (Fraction(-10**300), Fraction(10**300) + Fraction(1, 3))
    huge_expression = sp.Mul(sp.Integer(10**300), root(2), evaluate=False)
    expected = observation(accepted(huge_expression))
    shell = _shell(bounds)
    with localcontext() as context:
        context.prec, context.Emax, context.Emin = precision, 2, -2
        context.rounding = "ROUND_FLOOR"
        for signal in context.traps:
            context.traps[signal] = True
        context.clear_flags()
        assert observation(accepted(huge_expression)) == expected
        actual = _shell(bounds)
        assert actual == shell
        assert not any(context.flags.values())
    assert Fraction(actual.absolute_error_bound) == Fraction(actual.upper_units-actual.lower_units, 10**27)
    assert all(value.as_tuple().exponent == -27 for value in (
        actual.lower, actual.upper, actual.absolute_error_bound))


@pytest.mark.parametrize("expression", (
    sp.sqrt(2 + sp.sqrt(2)), sp.sin(sp.Rational(1, 3)), sp.pi,
    sp.Symbol("x"), sp.Float("0.5"),
    sp.Pow(-2, sp.Rational(1, 2), evaluate=False),
    sp.Pow(2, sp.Rational(1, 3), evaluate=False),
    sp.Pow(sp.Add(1, 1, evaluate=False), sp.Rational(1, 2), evaluate=False),
    sp.Mul(0, sp.sin(1), evaluate=False), 2,
))
def test_unsupported_ast_is_a_typed_candidate_outcome(expression):
    result = density_shell_v2(expression)
    assert isinstance(result, DensityShellV2NotApplicable)
    assert result.code is DensityShellNotApplicableCode.UNSUPPORTED_SYNTAX
    assert result.canon == DENSITY_SHELL_CANON_V2_CANDIDATE
    assert result.detail
    assert result.work.enclosure_rounds == 0


def test_four_class_inverse_is_named_candidate_not_applicable():
    expression = sp.Pow(sp.Add(1, root(2), root(3), root(5), evaluate=False), -1, evaluate=False)
    result = density_shell_v2(expression)
    assert isinstance(result, DensityShellV2NotApplicable)
    assert result.code is DensityShellNotApplicableCode.INVERSE_CLASS_LIMIT
    assert "4 classes" in result.detail
    assert result.work.class_merge_checks > 0


@pytest.mark.parametrize("denominator", (
    sp.Integer(0),
    sp.Add(root(18), sp.Mul(-3, root(2), evaluate=False), evaluate=False),
))
def test_exact_zero_division_is_never_unsupported(denominator):
    with pytest.raises(ZeroDivisionError, match="exact zero"):
        density_shell_v2(sp.Pow(denominator, -1, evaluate=False))


def test_internal_failure_is_never_unsupported(monkeypatch):
    def broken(self, bits):
        raise RuntimeError("broken enclosure")
    monkeypatch.setattr(RadicalSumV2, "enclosure", broken)
    with pytest.raises(RuntimeError, match="broken enclosure"):
        density_shell_v2(root(2))


def test_reader_never_calls_v1_or_symbolic_normalization(monkeypatch):
    from cftuv_envelope.reference import native_exact

    def forbidden(*args, **kwargs):
        raise AssertionError("forbidden reader/normalizer")
    expression = sp.Mul(root(18), root(2), evaluate=False)
    monkeypatch.setattr(native_exact, "from_sympy", forbidden)
    for name in ("factorint", "factor", "expand", "simplify"):
        monkeypatch.setattr(sp, name, forbidden)
    assert accepted(expression).value == RadicalSumV2.rational(6)


def test_memo_is_invocation_local_and_outcome_is_immutable(monkeypatch):
    calls = []
    original = RadicalSumV2.sqrt_of_rational
    def counted(value):
        calls.append(value)
        return original(value)
    monkeypatch.setattr(RadicalSumV2, "sqrt_of_rational", staticmethod(counted))
    expression = sp.Add(root(2), root(2), evaluate=False)
    first, second = accepted(expression), accepted(expression)
    assert calls == [Fraction(2), Fraction(2)]
    assert first == second
    assert first.work.enclosure_rounds == 1
    with pytest.raises(FrozenInstanceError):
        first.canon = "other"
    with pytest.raises(FrozenInstanceError):
        first.shell.lower_units = 0


@pytest.mark.parametrize("numerator,denominator,grid,straw", (
    (0, 1, True, False), (7, 8, True, False), (-7, 8, True, False),
    (1, 2**27, True, False), (1, 5, True, True), (-1, 5, True, True),
    (1, 3, False, False), (-1, 3, False, False), (1, 2**28, False, False),
))
def test_named_rational_outcomes_are_exact_value_facts(numerator, denominator, grid, straw):
    result = accepted(sp.Rational(numerator, denominator))
    bundle = result.outcomes
    expected = (DensityShellOutcomeCode.APPLIED,)
    if grid:
        expected += (DensityShellOutcomeCode.GRID_POINT,)
    if straw:
        expected += (DensityShellOutcomeCode.NON_DYADIC_GRID_STRAW,)
    assert bundle.codes == expected
    assert (bundle.rational.numerator, bundle.rational.denominator) == (numerator, denominator)
    assert bundle.rational.grid_units == (numerator*(10**27//denominator) if grid else None)
    assert bundle.legacy_projected_min_width_units == (2 if straw else None)
    if grid:
        assert result.shell.lower_units == result.shell.upper_units == bundle.rational.grid_units


def test_irrational_outcome_has_applied_only_and_no_rational_metadata():
    bundle = accepted(root(2)).outcomes
    assert bundle.codes == (DensityShellOutcomeCode.APPLIED,)
    assert bundle.rational is None
    assert bundle.legacy_projected_min_width_units is None


def test_outcomes_use_whole_canonical_value_and_are_deeply_immutable():
    expression = sp.Add(sp.Rational(1, 10), sp.Rational(1, 10), evaluate=False)
    bundle = accepted(expression).outcomes
    assert bundle == accepted(sp.Rational(1, 5)).outcomes
    assert (bundle.rational.numerator, bundle.rational.denominator) == (1, 5)
    with pytest.raises(FrozenInstanceError):
        bundle.codes = ()
    with pytest.raises(FrozenInstanceError):
        bundle.rational.grid_units = 0
    with pytest.raises(TypeError):
        bundle.codes[0] = DensityShellOutcomeCode.GRID_POINT


@pytest.mark.parametrize("bits", (4, 64, 160))
@pytest.mark.parametrize("numerator", (1, -1, 1657, 3647))
def test_straw_proves_at_least_two_projected_units_without_legacy_evaluator(bits, numerator):
    denominator = 5 if abs(numerator) == 1 else 5120
    result = accepted(sp.Rational(numerator, denominator))
    bundle = result.outcomes
    assert DensityShellOutcomeCode.NON_DYADIC_GRID_STRAW in bundle.codes
    exact = Fraction(numerator, denominator)
    binary_scale = 2**bits
    floor = numerator*binary_scale//denominator
    lower, upper = Fraction(floor, binary_scale), Fraction(floor+1, binary_scale)
    assert lower < exact < upper
    projected = _shell((lower, upper))
    assert projected.lower_units <= bundle.rational.grid_units-1
    assert projected.upper_units >= bundle.rational.grid_units+1
    assert projected.upper_units-projected.lower_units >= bundle.legacy_projected_min_width_units
    if bits == 4:
        assert projected.upper_units-projected.lower_units > 2


@pytest.mark.parametrize("numerator", (1657, 1657, 3647, 3647))
def test_four_known_recorded_calls_receive_non_dyadic_grid_straw(numerator):
    result = accepted(sp.Rational(numerator, 5120))
    assert result.outcomes.codes == (
        DensityShellOutcomeCode.APPLIED, DensityShellOutcomeCode.GRID_POINT,
        DensityShellOutcomeCode.NON_DYADIC_GRID_STRAW,
    )
    assert result.outcomes.legacy_projected_min_width_units == 2
    assert result.shell.absolute_error_bound.as_tuple() == Decimal("0E-27").as_tuple()
