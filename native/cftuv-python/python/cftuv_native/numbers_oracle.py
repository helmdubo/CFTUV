"""The Python oracle for the native number operations, and the strict comparison against it.

Every operation of the number script (`codec.OPS`) has here the PYTHON function it ports, called on the same
arguments: the kernel's `exact_sqrt_sum`, `exact_sqrt_sum_fused` and `float_filter`, `math`, `fractions`. The
kernel is the oracle and is never edited; the caller puts `kernel/src` on `sys.path`. This module imports no
extension.

`same` is the equality the equivalence contract needs: types count (`int` is not `Fraction`, even where `==`
says they are equal), floats compare by bit pattern, sums compare term by term including the coefficient type,
a named native error equals the exception class the oracle raised.
"""

from __future__ import annotations

import math
import struct
from fractions import Fraction

from .codec import NativeError, sqrt_sum_type

#: What the oracle returns for a case it cannot afford (exact work budget exhausted); the harness drops such cases.
SKIPPED = object()

_COUNT_KEYS = ("total", "closed_rational_zero", "closed_rational_nonzero", "closed_by_enclosure", "closed_by_conjugation")
_ERROR_CODES = {OverflowError: 1, ZeroDivisionError: 2, ValueError: 3}
_SIGN_CAP = 1 << 16


def _modules():
    from cftuv_envelope import exact_sqrt_sum, exact_sqrt_sum_fused, float_filter

    return exact_sqrt_sum, exact_sqrt_sum_fused, float_filter


def _items(raw) -> list:
    return [(radicand, numerator) for radicand, numerator in raw]


def _lists(value):
    """Tuples to lists, recursively: the native side returns lists where Python returns pairs."""

    if isinstance(value, (list, tuple)):
        return [_lists(item) for item in value]
    return value


def _counts_delta(before: dict, after: dict) -> list:
    return [after[key] - before[key] for key in _COUNT_KEYS]


def _with_counts(call):
    """`(result, sign-count delta)` of `call()`; the process-wide counters are restored."""

    exact = _modules()[0]
    before = dict(exact.SIGN_COUNTS)
    try:
        result = call()
        delta = _counts_delta(before, exact.SIGN_COUNTS)
    finally:
        exact.SIGN_COUNTS.update(before)
    return result, delta


def _prefilter(value, bits: int):
    """`SqrtSumV1.sign` up to the conjugation: `[sign or None, counter delta]` (None: it went on to `_exact_sign`)."""

    exact = _modules()[0]
    budget = exact.exact_work_budget(stage="NATIVE_NUMBERS", cap=_SIGN_CAP)
    with exact.isolated_factorization_memory():
        try:
            sign, delta = _with_counts(lambda: value.sign(filter_bits=bits, budget=budget))
        except exact.ExactCanonicalizationWorkBudgetExhausted:
            return SKIPPED
    return [None if delta[4] else sign, delta]


def _affine(args):
    from cftuv_envelope import float_filter

    points, values = args
    return float_filter.affine_map_violated(dict(enumerate(points)), dict(enumerate(values)), (0, 1, 2), 3)


def _operations():
    exact, fused, filt = _modules()
    sum_type = sqrt_sum_type()
    one = Fraction(1)
    return {
        "ISQRT": lambda n: math.isqrt(n),
        "GCD": lambda a, b: math.gcd(a, b),
        "LCM": lambda a, b: math.lcm(a, b),
        "BIT_LENGTH": lambda n: n.bit_length(),
        "FLOAT_OF_INT": lambda n: float(n),
        "FLOAT_OF_FRACTION": lambda f: float(f),
        "MATH_SQRT_INT": lambda n: math.sqrt(n),
        "RAT_NEW": lambda n, d: Fraction(n, d),
        "RAT_ADD": lambda a, b: Fraction(a) + Fraction(b),
        "RAT_SUB": lambda a, b: Fraction(a) - Fraction(b),
        "RAT_MUL": lambda a, b: Fraction(a) * Fraction(b),
        "RAT_DIV": lambda a, b: Fraction(a) / Fraction(b),
        "RAT_NEG": lambda a: -Fraction(a),
        "RAT_CMP": lambda a, b: (a > b) - (a < b),
        "SUM_RATIONAL": lambda v: sum_type.rational(v),
        "SUM_ADD": lambda a, b: a + b,
        "SUM_SUB": lambda a, b: a - b,
        "SUM_NEG": lambda a: -a,
        "SUM_SCALED": lambda a, f: a.scaled(f),
        "SUM_SCALED_DIFFERENCE": lambda a, fa, b, fb: a.scaled_difference(fa, b, fb),
        "SUM_DIFFERENCE_IS_ZERO": lambda a, b: a.difference_is_zero(b),
        "SUM_MUL": lambda a, b: a * b,
        "SUM_IS_ZERO": lambda a: a.is_zero,
        "SUM_IS_RATIONAL": lambda a: a.is_rational(),
        "SUM_AS_RATIONAL": lambda a: a.as_rational(),
        "SUM_ENCLOSURE": lambda a, bits: list(a.enclosure(bits)),
        "SUM_CERTIFIED_SIGN": lambda a, bits: a.certified_sign(bits),
        "SUM_SIGN_PREFILTER": _prefilter,
        "INTEGER_FORM": lambda a: _lists(exact._integer_form(a.terms)),
        "INTEGER_ENCLOSURE": lambda items, bits: list(exact._integer_enclosure(_items(items), bits)),
        "INTEGER_CERTIFIED_SIGN": lambda items, bits: exact._integer_certified_sign(_items(items), bits),
        "SCALED_DIFFERENCE_PARTS": lambda a, fa, b, fb: _lists(exact._scaled_difference_parts(a, fa, b, fb)),
        "MULTIPLY_INTEGER_ITEMS": lambda left, right: _lists(exact._multiply_integer_items(_items(left), _items(right))),
        "REDUCED_FORM": lambda common, items: _lists(exact._reduced_form(common, _items(items))),
        "SCALED_BY_RECIPROCAL": lambda nc, ni, dc, di: exact._scaled_by_reciprocal(nc, _items(ni), dc, _items(di)),
        "FILTERED_SIGN": lambda items, bits: list(_with_counts(lambda: exact._filtered_sign(_items(items), bits))),
        "DIFFERENCE_FILTERED_SIGN": lambda a, b: list(
            _with_counts(lambda: exact._filtered_sign(exact._scaled_difference_parts(a, one, b, one)[1]))
        ),
        "ORIENTED_SUM": lambda x, y, sx, sy, off: fused.oriented_sum(x, y, sx, sy, off),
        "PRODUCT_ADDED": lambda base, left, right: fused.product_added(base, left, right),
        "SUM_OF_PRODUCTS": lambda products: fused.sum_of_products([tuple(entry) for entry in products]),
        "FF_CENTRE_AND_BOUND": lambda a: _lists(filt.centre_and_bound(a)),
        "FF_ORIENTATION_SIGN": lambda p, q, r: filt.orientation_sign(p, q, r),
        "FF_LINE_ESTIMATE": lambda point, sx, sy, dx, dy: _lists(filt.line_estimate(point, sx, sy, dx, dy)),
        "FF_POLYGON_SIGN": lambda points: filt.polygon_sign(points),
        "FF_AFFINE_MAP_VIOLATED": lambda points, values: _affine((points, values)),
    }


_TABLE: dict = {}


def run_oracle_op(operation: str, arguments):
    """The oracle's result of one operation: a value, a `NativeError` (the exception it raised) or `SKIPPED`."""

    if not _TABLE:
        _TABLE.update(_operations())
    try:
        return _lists(_TABLE[operation](*arguments))
    except (OverflowError, ZeroDivisionError, ValueError) as error:
        return NativeError(_ERROR_CODES[type(error)])


def run_oracle(ops) -> list:
    """The oracle's result for every `(operation, arguments)` of a script."""

    return [run_oracle_op(operation, arguments) for operation, arguments in ops]


def clear_oracle_state() -> None:
    """Drop what the oracle caches between operations (the id-keyed float table); answers do not depend on it."""

    _modules()[2].clear_table()


def _bits(value: float) -> bytes:
    return struct.pack("<d", value)


def same(expected, actual) -> bool:
    """Strict equality of two results (see the module docstring)."""

    kind = type(expected)
    if kind is not type(actual):
        return False
    if kind is float:
        # a NaN is a NaN whatever its sign and payload (they depend on the instruction that made it); all else by bits
        return _bits(expected) == _bits(actual) or (expected != expected and actual != actual)
    if kind is list:
        return len(expected) == len(actual) and all(same(left, right) for left, right in zip(expected, actual))
    if kind is NativeError:
        return expected.code == actual.code
    if kind is int or kind is bool or kind is Fraction or expected is None:
        return expected == actual
    if kind is sqrt_sum_type():
        left, right = expected.terms, actual.terms
        return len(left) == len(right) and all(
            a_radicand == b_radicand and type(a_coefficient) is type(b_coefficient) and a_coefficient == b_coefficient
            for (a_radicand, a_coefficient), (b_radicand, b_coefficient) in zip(left, right)
        )
    return False


def describe(value) -> str:
    """A short, exact rendering of a result for a failure message (types and bit patterns included)."""

    kind = type(value)
    if kind is float:
        return f"float({value!r} bits={_bits(value).hex()})"
    if kind is list:
        return "[" + ", ".join(describe(item) for item in value) + "]"
    if kind is sqrt_sum_type():
        body = ", ".join(f"({radicand}, {type(coefficient).__name__}({coefficient}))" for radicand, coefficient in value.terms)
        return f"SqrtSum[{body}]"
    return f"{kind.__name__}({value})" if kind in (int, Fraction, bool) else repr(value)
