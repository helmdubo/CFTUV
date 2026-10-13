"""Python-only A1 shell CANDIDATE; no product admission or DensityWork change.

The supported whole SymPy AST is read directly into RadicalSumV2. Its value
alone determines enclosure(160), integer outward projection onto 10**-27,
and all three Decimal tuples. Unsupported syntax and the V2 inverse price
limit are named candidate-not-applicable outcomes. There is no legacy fallback.
"""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from enum import Enum
from fractions import Fraction

import sympy as sp

from .radical_v2 import (
    NO_WORK,
    NativeAlgebraWorkV1,
    RadicalInverseClassLimit,
    RadicalSumV2,
)

DENSITY_SHELL_CANON_V2_CANDIDATE = "DENSITY_SHELL_CANON_V2_CANDIDATE"
SHELL_BITS = 160
SHELL_QUANTUM = -27
SHELL_SCALE = 10**27


class DensityShellNotApplicableCode(str, Enum):
    UNSUPPORTED_SYNTAX = "DENSITY_SHELL_V2_UNSUPPORTED_SYNTAX"
    INVERSE_CLASS_LIMIT = "DENSITY_SHELL_V2_INVERSE_CLASS_LIMIT"


class DensityShellOutcomeCode(str, Enum):
    APPLIED = "DENSITY_DECIMAL_SHELL_V2_APPLIED"
    GRID_POINT = "DENSITY_DECIMAL_GRID_POINT_V2"
    NON_DYADIC_GRID_STRAW = "DENSITY_DECIMAL_NON_DYADIC_GRID_STRAW_V2"


@dataclass(frozen=True, slots=True)
class DensityShellRationalMetadataV2:
    numerator: int
    denominator: int
    grid_units: int | None


@dataclass(frozen=True, slots=True)
class DensityShellOutcomeBundleV2:
    """Replayable named value facts; no evaluator, cache or aggregate counter.

    A non-dyadic rational cannot equal a finite binary endpoint. For a grid
    point g/S enclosed by finite binary endpoints L <= g/S <= U, both
    inequalities are therefore strict: floor(S*L) <= g-1 and ceil(S*U) >= g+1.
    The legacy projected width is AT LEAST two units, not necessarily two.
    """

    codes: tuple[DensityShellOutcomeCode, ...]
    rational: DensityShellRationalMetadataV2 | None
    legacy_projected_min_width_units: int | None


@dataclass(frozen=True, slots=True)
class DensityShellV2:
    """Closed endpoints and their exact mathematical width, all at quantum -27."""

    lower: Decimal
    upper: Decimal
    absolute_error_bound: Decimal
    lower_units: int
    upper_units: int


@dataclass(frozen=True, slots=True)
class DensityShellV2Candidate:
    canon: str
    value: RadicalSumV2
    enclosure160: tuple[Fraction, Fraction]
    shell: DensityShellV2
    work: NativeAlgebraWorkV1
    outcomes: DensityShellOutcomeBundleV2


@dataclass(frozen=True, slots=True)
class DensityShellV2NotApplicable:
    canon: str
    code: DensityShellNotApplicableCode
    detail: str
    work: NativeAlgebraWorkV1


class _UnsupportedSyntax(Exception):
    pass


class _Reader:
    """AST memo and diagnostic work belong to exactly one invocation."""

    def __init__(self) -> None:
        self.memo: dict[sp.Basic, RadicalSumV2] = {}
        self.work = NO_WORK

    def priced(self, result) -> RadicalSumV2:
        value, work = result
        self.work += work
        return value

    def read(self, expression) -> RadicalSumV2:
        if not isinstance(expression, sp.Basic):
            raise _UnsupportedSyntax("expected a SymPy AST")
        if expression in self.memo:
            return self.memo[expression]
        if isinstance(expression, sp.Rational):
            value = RadicalSumV2.rational(_fraction(expression))
        elif expression.func in (sp.Add, sp.Mul):
            value = self.fold(expression)
        elif expression.func is sp.Pow:
            value = self.power(expression)
        else:
            raise _UnsupportedSyntax(f"unsupported AST node: {expression.func.__name__}")
        self.memo[expression] = value
        return value

    def fold(self, expression) -> RadicalSumV2:
        additive = expression.func is sp.Add
        result = RadicalSumV2.zero() if additive else RadicalSumV2.rational(1)
        # Каждый аргумент читается даже при нулевом произведении.
        for child in expression.args:
            value = self.read(child)
            result = self.priced(result.add(value) if additive else result.mul(value))
        return result

    def power(self, expression) -> RadicalSumV2:
        base, exponent = expression.args
        if isinstance(exponent, sp.Integer):
            return self.integer_power(self.read(base), int(exponent))
        if (isinstance(base, sp.Rational) and base >= 0
                and isinstance(exponent, sp.Rational) and exponent.q == 2):
            root = self.priced(RadicalSumV2.sqrt_of_rational(_fraction(base)))
            return self.integer_power(root, int(exponent.p))
        raise _UnsupportedSyntax("Pow requires an integer exponent or nonnegative rational base to p/2")

    def integer_power(self, base: RadicalSumV2, exponent: int) -> RadicalSumV2:
        if exponent < 0:
            base = self.priced(base.inverse())
            exponent = -exponent
        result = RadicalSumV2.rational(1)
        while exponent:
            if exponent & 1:
                result = self.priced(result.mul(base))
            exponent >>= 1
            if exponent:
                base = self.priced(base.mul(base))
        return result


def _fraction(value: sp.Rational) -> Fraction:
    return Fraction(int(value.p), int(value.q))


def _decimal_units(units: int) -> Decimal:
    # Конструкторы из int и tuple точны при любом Decimal context.
    digits = Decimal(abs(units)).as_tuple().digits
    return Decimal((int(units < 0), digits, SHELL_QUANTUM))


def _shell(bounds: tuple[Fraction, Fraction]) -> DensityShellV2:
    lower, upper = bounds
    lo = lower.numerator * SHELL_SCALE // lower.denominator
    hi = -((-upper.numerator * SHELL_SCALE) // upper.denominator)
    return DensityShellV2(
        _decimal_units(lo), _decimal_units(hi), _decimal_units(hi - lo), lo, hi,
    )


def _outcome_bundle(value: RadicalSumV2) -> DensityShellOutcomeBundleV2:
    codes = (DensityShellOutcomeCode.APPLIED,)
    rational = value.as_rational()
    if rational is None:
        return DensityShellOutcomeBundleV2(codes, None, None)
    numerator, denominator = rational.numerator, rational.denominator
    grid_units = None
    minimum_width = None
    if SHELL_SCALE % denominator == 0:
        grid_units = numerator * (SHELL_SCALE // denominator)
        codes += (DensityShellOutcomeCode.GRID_POINT,)
        if denominator & (denominator - 1):
            codes += (DensityShellOutcomeCode.NON_DYADIC_GRID_STRAW,)
            minimum_width = 2
    metadata = DensityShellRationalMetadataV2(numerator, denominator, grid_units)
    return DensityShellOutcomeBundleV2(codes, metadata, minimum_width)


def density_shell_v2(expression) -> DensityShellV2Candidate | DensityShellV2NotApplicable:
    """An explicitly separate A1 portability candidate, with diagnostic V2 work.

    Exact zero division and internal failures propagate to the caller. A named
    not-applicable outcome never constitutes a product refusal or fallback.
    """

    reader = _Reader()
    try:
        value = reader.read(expression)
    except _UnsupportedSyntax as error:
        return DensityShellV2NotApplicable(
            DENSITY_SHELL_CANON_V2_CANDIDATE,
            DensityShellNotApplicableCode.UNSUPPORTED_SYNTAX, str(error), reader.work,
        )
    except RadicalInverseClassLimit as error:
        return DensityShellV2NotApplicable(
            DENSITY_SHELL_CANON_V2_CANDIDATE,
            DensityShellNotApplicableCode.INVERSE_CLASS_LIMIT,
            error.detail, reader.work + error.work,
        )
    bounds, enclosure_work = value.enclosure(SHELL_BITS)
    return DensityShellV2Candidate(
        DENSITY_SHELL_CANON_V2_CANDIDATE, value, bounds, _shell(bounds),
        reader.work + enclosure_work, _outcome_bundle(value),
    )
