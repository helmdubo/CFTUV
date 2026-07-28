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


def _unit_sum(left, right):
    x = left[0] + right[0]
    y = left[1] + right[1]
    length = (x * x + y * y).sqrt()
    return x / length, y / length


def _cross(left, right):
    return left[0] * right[1] - left[1] * right[0]


def _decimal_cross(left, right):
    return left[0] * right[1] - left[1] * right[0]
