"""Независимый oracle направлений: только Fraction/Decimal и целые предикаты."""

from __future__ import annotations

from decimal import Decimal, localcontext
from fractions import Fraction


def assert_k2_identity_binding(
    vectors: tuple[tuple[int, int], tuple[int, int]],
) -> None:
    """Проверить кубический корпус angular_snapshot без production binding."""

    incoming = (1, 0)
    outgoing = (-117, -44)
    sequence = (incoming, *vectors, outgoing)
    assert all(_cross(left, right) < 0 for left, right in zip(sequence, sequence[1:]))
    assert all(
        _subturn_le_pi_over_three(left, right)
        for left, right in zip(sequence, sequence[1:])
    )

    with localcontext() as context:
        context.prec = 80
        cosine_step = _bisect_cubic_cosine(Fraction(-117, 125))
        sine_step = -(Decimal(1) - cosine_step * cosine_step).sqrt()
        ideal_one = (cosine_step, sine_step)
        ideal_two = (
            2 * cosine_step * cosine_step - 1,
            2 * cosine_step * sine_step,
        )
        incoming_decimal = (Decimal(1), Decimal(0))
        outgoing_decimal = (Decimal(-117) / 125, Decimal(-44) / 125)
        lower_one = _unit_sum(incoming_decimal, ideal_one)
        upper_one = _unit_sum(ideal_one, ideal_two)
        lower_two = upper_one
        upper_two = _unit_sum(ideal_two, outgoing_decimal)
        for vector, lower, upper in (
            (vectors[0], lower_one, upper_one),
            (vectors[1], lower_two, upper_two),
        ):
            candidate = (Decimal(vector[0]), Decimal(vector[1]))
            assert _decimal_cross(lower, candidate) < 0
            assert _decimal_cross(candidate, upper) < 0


def assert_k1_dual_metric_binding(
    dual_metric: tuple[
        tuple[Fraction, Fraction], tuple[Fraction, Fraction]
    ],
    incoming: tuple[int, int],
    outgoing: tuple[int, int],
    bound: tuple[int, int],
    *,
    clockwise: bool,
) -> None:
    """Oracle реального k=1: G^-1/cos exact, окно — собственной Decimal math."""

    expected_sign = -1 if clockwise else 1
    assert _sign(_cross(incoming, bound)) == expected_sign
    assert _sign(_cross(bound, outgoing)) == expected_sign
    assert _metric_subturn(dual_metric, incoming, bound)
    assert _metric_subturn(dual_metric, bound, outgoing)
    with localcontext() as context:
        context.prec = 80
        matrix = tuple(
            tuple(Decimal(value.numerator) / value.denominator for value in row)
            for row in dual_metric
        )
        incoming_unit = _metric_unit_decimal(matrix, incoming)
        outgoing_unit = _metric_unit_decimal(matrix, outgoing)
        ideal = _metric_unit_decimal(
            matrix,
            (
                incoming_unit[0] + outgoing_unit[0],
                incoming_unit[1] + outgoing_unit[1],
            ),
        )
        lower = _metric_unit_decimal(
            matrix,
            (incoming_unit[0] + ideal[0], incoming_unit[1] + ideal[1]),
        )
        upper = _metric_unit_decimal(
            matrix,
            (ideal[0] + outgoing_unit[0], ideal[1] + outgoing_unit[1]),
        )
        candidate = (Decimal(bound[0]), Decimal(bound[1]))
        assert _sign(_decimal_cross(lower, candidate)) == expected_sign
        assert _sign(_decimal_cross(candidate, upper)) == expected_sign


def _bisect_cubic_cosine(total_cosine: Fraction) -> Decimal:
    """Единственный c in (1/2,1): 4c^3-3c=cos(total), без SymPy."""

    target = Decimal(total_cosine.numerator) / Decimal(total_cosine.denominator)
    lower = Decimal("0.5")
    upper = Decimal(1)
    for _ in range(320):
        middle = (lower + upper) / 2
        value = 4 * middle * middle * middle - 3 * middle
        if value < target:
            lower = middle
        else:
            upper = middle
    return (lower + upper) / 2


def _subturn_le_pi_over_three(left, right) -> bool:
    dot = left[0] * right[0] + left[1] * right[1]
    if dot <= 0:
        return False
    left_squared = left[0] * left[0] + left[1] * left[1]
    right_squared = right[0] * right[0] + right[1] * right[1]
    return 4 * dot * dot >= left_squared * right_squared


def _metric_subturn(matrix, left, right):
    dot = _metric_dot_fraction(matrix, left, right)
    if dot <= 0:
        return False
    return 4 * dot * dot >= (
        _metric_dot_fraction(matrix, left, left)
        * _metric_dot_fraction(matrix, right, right)
    )


def _metric_dot_fraction(matrix, left, right):
    return left[0] * (matrix[0][0] * right[0] + matrix[0][1] * right[1]) + left[
        1
    ] * (matrix[1][0] * right[0] + matrix[1][1] * right[1])


def _metric_unit_decimal(matrix, vector):
    x = Decimal(vector[0])
    y = Decimal(vector[1])
    squared = x * (matrix[0][0] * x + matrix[0][1] * y) + y * (
        matrix[1][0] * x + matrix[1][1] * y
    )
    length = squared.sqrt()
    return x / length, y / length


def _unit_sum(left, right):
    x = left[0] + right[0]
    y = left[1] + right[1]
    length = (x * x + y * y).sqrt()
    return x / length, y / length


def _cross(left, right):
    return left[0] * right[1] - left[1] * right[0]


def _decimal_cross(left, right):
    return left[0] * right[1] - left[1] * right[0]


def _sign(value):
    return (value > 0) - (value < 0)
