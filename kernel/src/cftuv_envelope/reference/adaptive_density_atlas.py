"""Exact coordinate-atlas helpers for adaptive Density fan authority."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from fractions import Fraction
from math import gcd

from ..contracts.envelopes import (
    AdaptiveProjectivePoleOwnershipV1,
    AdaptiveRationalFanOrdinalWindowAtlasV1,
    AdaptiveRationalFanProjectiveChartPieceV1,
)
from ..numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind


_UNREPRESENTABLE = "DENSITY_WINDOW_CHART_UNREPRESENTABLE"


@dataclass(frozen=True, slots=True)
class AtlasWindowPrepared:
    pieces: tuple[AdaptiveRationalFanProjectiveChartPieceV1, ...]
    termination_piece_index: int
    local_record: tuple[
        bool,
        int,
        CertifiedDecimalIntervalV1,
        CertifiedDecimalIntervalV1,
    ]


def _exact_decimal_interval(value: int) -> CertifiedDecimalIntervalV1:
    decimal = Decimal(value)
    return CertifiedDecimalIntervalV1(
        lower=decimal,
        upper=decimal,
        lower_kind=IntervalEndpointKind.CLOSED,
        upper_kind=IntervalEndpointKind.CLOSED,
        absolute_error_bound=Decimal(0),
    )


def _cell(metric, vector, *, sign, invalid) -> tuple[int, bool, int]:
    """Каноническая half-open ячейка: диагонали принадлежат x-chart."""

    x, y = metric.density_expressions(vector)
    if sign(x * x - y * y, metric) >= 0:
        denominator_sign = sign(x, metric)
        if denominator_sign == 0:
            raise invalid(_UNREPRESENTABLE)
        return (
            0 if denominator_sign > 0 else 2,
            True,
            denominator_sign,
        )
    denominator_sign = sign(y, metric)
    if denominator_sign == 0:
        raise invalid(_UNREPRESENTABLE)
    return (
        1 if denominator_sign > 0 else 3,
        False,
        denominator_sign,
    )


def _transition_vector(left_cell: int, right_cell: int, *, invalid):
    values = {
        frozenset((0, 1)): (1, 1),
        frozenset((1, 2)): (-1, 1),
        frozenset((2, 3)): (-1, -1),
        frozenset((0, 3)): (1, -1),
    }.get(frozenset((left_cell, right_cell)))
    if values is None:
        raise invalid(_UNREPRESENTABLE)
    return values


def _cell_path(metric, left, ideal, right, *, sign, invalid):
    values = tuple(
        metric.density_expressions(item) for item in (left, ideal, right)
    )
    turns = (
        sign(
            values[0][0] * values[1][1] - values[0][1] * values[1][0],
            metric,
        ),
        sign(
            values[1][0] * values[2][1] - values[1][1] * values[2][0],
            metric,
        ),
    )
    if 0 in turns or turns[0] != turns[1]:
        raise invalid(_UNREPRESENTABLE)
    cells = tuple(
        _cell(metric, item, sign=sign, invalid=invalid)[0]
        for item in (left, ideal, right)
    )
    step = 1 if turns[0] > 0 else -1
    path = [cells[0]]
    for _ in range(4):
        if path[-1] == cells[2]:
            break
        path.append((path[-1] + step) % 4)
    if path[-1] != cells[2] or cells[1] not in path:
        raise invalid(_UNREPRESENTABLE)
    return tuple(path)


def _piece_vectors(
    path,
    piece_index,
    left,
    right,
    metric,
    *,
    vector,
    invalid,
):
    cell = path[piece_index]
    start = (
        left
        if piece_index == 0
        else vector(
            *_transition_vector(
                path[piece_index - 1],
                cell,
                invalid=invalid,
            ),
            metric,
        )
    )
    end = (
        right
        if piece_index == len(path) - 1
        else vector(
            *_transition_vector(
                cell,
                path[piece_index + 1],
                invalid=invalid,
            ),
            metric,
        )
    )
    return start, end


def build_atlas_window_record(
    metric,
    ideal,
    left,
    right,
    *,
    sign,
    slope,
    vector,
    decimal_envelope,
    invalid,
) -> AtlasWindowPrepared:
    """Разбить oriented окно по каноническим coordinate cells без дыр."""

    path = _cell_path(
        metric,
        left,
        ideal,
        right,
        sign=sign,
        invalid=invalid,
    )
    ideal_cell = _cell(
        metric,
        ideal,
        sign=sign,
        invalid=invalid,
    )[0]
    pieces = []
    for piece_index, cell in enumerate(path):
        use_x = cell in (0, 2)
        denominator_sign = 1 if cell in (0, 1) else -1
        start_vector, end_vector = _piece_vectors(
            path,
            piece_index,
            left,
            right,
            metric,
            vector=vector,
            invalid=invalid,
        )
        start_exact = slope(start_vector, use_x, metric)
        end_exact = slope(end_vector, use_x, metric)
        order = sign(end_exact - start_exact, metric)
        if order == 0:
            raise invalid(_UNREPRESENTABLE)
        slope_increases = order > 0
        start = (
            _exact_decimal_interval(int(start_exact))
            if start_exact in (-1, 1)
            else decimal_envelope(start_exact, metric)
        )
        end = (
            _exact_decimal_interval(int(end_exact))
            if end_exact in (-1, 1)
            else decimal_envelope(end_exact, metric)
        )
        lower, upper = (start, end) if slope_increases else (end, start)
        lower_exact, upper_exact = (
            (start_exact, end_exact)
            if slope_increases
            else (end_exact, start_exact)
        )
        if lower.upper >= upper.lower:
            raise invalid(_UNREPRESENTABLE)
        pole = AdaptiveProjectivePoleOwnershipV1.NONE
        if sign(lower_exact, metric) < 0 and sign(upper_exact, metric) > 0:
            pole = (
                AdaptiveProjectivePoleOwnershipV1.Y_ZERO
                if use_x
                else AdaptiveProjectivePoleOwnershipV1.X_ZERO
            )
        start_included = piece_index != 0 and use_x
        end_included = piece_index != len(path) - 1 and use_x
        pieces.append(
            AdaptiveRationalFanProjectiveChartPieceV1(
                piece_index=piece_index,
                use_x_denominator=use_x,
                denominator_sign=denominator_sign,
                lower_slope_envelope=lower,
                upper_slope_envelope=upper,
                lower_endpoint_included=(
                    start_included if slope_increases else end_included
                ),
                upper_endpoint_included=(
                    end_included if slope_increases else start_included
                ),
                slope_increases_in_ordinal_order=slope_increases,
                pole_ownership=pole,
            )
        )
    termination_piece_index = path.index(ideal_cell)
    termination_piece = pieces[termination_piece_index]
    ideal_slope = slope(
        ideal,
        termination_piece.use_x_denominator,
        metric,
    )
    if not (
        Fraction(termination_piece.lower_slope_envelope.upper)
        < ideal_slope
        < Fraction(termination_piece.upper_slope_envelope.lower)
    ):
        raise invalid(_UNREPRESENTABLE)
    return AtlasWindowPrepared(
        pieces=tuple(pieces),
        termination_piece_index=termination_piece_index,
        local_record=(
            termination_piece.use_x_denominator,
            termination_piece.denominator_sign,
            termination_piece.lower_slope_envelope,
            termination_piece.upper_slope_envelope,
        ),
    )


def termination_piece(item, *, invalid):
    if (
        type(item.termination_piece_index) is not int
        or item.termination_piece_index < 0
        or item.termination_piece_index >= len(item.pieces)
    ):
        raise invalid("adaptive fan atlas identity invalid")
    return item.pieces[item.termination_piece_index]


def piece_ordered_endpoints(piece):
    return (
        (
            piece.lower_slope_envelope,
            piece.upper_slope_envelope,
        )
        if piece.slope_increases_in_ordinal_order
        else (
            piece.upper_slope_envelope,
            piece.lower_slope_envelope,
        )
    )


def _piece_cell_index(piece) -> int:
    return (
        0
        if piece.use_x_denominator and piece.denominator_sign == 1
        else 2
        if piece.use_x_denominator
        else 1
        if piece.denominator_sign == 1
        else 3
    )


def validate_atlas_structure(item, *, invalid) -> None:
    if (
        type(item) is not AdaptiveRationalFanOrdinalWindowAtlasV1
        or len(item.pieces) < 2
        or len(item.pieces) > 4
    ):
        raise invalid("adaptive fan atlas cardinality invalid")
    cells = []
    for index, piece in enumerate(item.pieces):
        if (
            type(piece) is not AdaptiveRationalFanProjectiveChartPieceV1
            or piece.piece_index != index
            or type(piece.use_x_denominator) is not bool
            or piece.denominator_sign not in (-1, 1)
            or type(piece.lower_endpoint_included) is not bool
            or type(piece.upper_endpoint_included) is not bool
            or type(piece.slope_increases_in_ordinal_order) is not bool
            or type(piece.pole_ownership)
            is not AdaptiveProjectivePoleOwnershipV1
            or piece.lower_slope_envelope.upper
            >= piece.upper_slope_envelope.lower
        ):
            raise invalid("adaptive fan atlas piece invalid")
        expected_pole = AdaptiveProjectivePoleOwnershipV1.NONE
        if (
            piece.lower_slope_envelope.upper < 0
            and piece.upper_slope_envelope.lower > 0
        ):
            expected_pole = (
                AdaptiveProjectivePoleOwnershipV1.Y_ZERO
                if piece.use_x_denominator
                else AdaptiveProjectivePoleOwnershipV1.X_ZERO
            )
        if piece.pole_ownership is not expected_pole:
            raise invalid("adaptive fan atlas pole ownership invalid")
        cells.append(_piece_cell_index(piece))
    steps = tuple(
        (right - left) % 4
        for left, right in zip(cells, cells[1:])
    )
    if not steps or any(
        step != steps[0] or step not in (1, 3) for step in steps
    ):
        raise invalid("adaptive fan atlas order invalid")
    first = item.pieces[0]
    last = item.pieces[-1]
    if (
        first.lower_endpoint_included
        if first.slope_increases_in_ordinal_order
        else first.upper_endpoint_included
    ) or (
        last.upper_endpoint_included
        if last.slope_increases_in_ordinal_order
        else last.lower_endpoint_included
    ):
        raise invalid("adaptive fan atlas outer boundary closed")
    for left, right in zip(item.pieces, item.pieces[1:]):
        _, left_end = piece_ordered_endpoints(left)
        right_start, _ = piece_ordered_endpoints(right)
        if (
            left_end.lower != left_end.upper
            or right_start.lower != right_start.upper
            or left_end.lower != right_start.lower
            or left_end.lower not in (Decimal(-1), Decimal(1))
        ):
            raise invalid("adaptive fan atlas gap or overlap")
        left_included = (
            left.upper_endpoint_included
            if left.slope_increases_in_ordinal_order
            else left.lower_endpoint_included
        )
        right_included = (
            right.lower_endpoint_included
            if right.slope_increases_in_ordinal_order
            else right.upper_endpoint_included
        )
        if left_included == right_included:
            raise invalid("adaptive fan atlas pole boundary ownership invalid")
    termination_piece(item, invalid=invalid)


def piece_contains_vector(piece, vector) -> bool:
    denominator = vector[0 if piece.use_x_denominator else 1]
    if (
        denominator == 0
        or (1 if denominator > 0 else -1) != piece.denominator_sign
    ):
        return False
    numerator = vector[1 if piece.use_x_denominator else 0]
    slope = Fraction(numerator, denominator)
    lower = Fraction(piece.lower_slope_envelope.upper)
    upper = Fraction(piece.upper_slope_envelope.lower)
    return (
        lower < slope < upper
        or (
            piece.lower_endpoint_included
            and piece.lower_slope_envelope.lower
            == piece.lower_slope_envelope.upper
            and slope == lower
        )
        or (
            piece.upper_endpoint_included
            and piece.upper_slope_envelope.lower
            == piece.upper_slope_envelope.upper
            and slope == upper
        )
    )


def global_termination_height(boxes, *, local_height) -> int:
    """Перенести slope-denominator bound в chart-invariant max-norm."""

    bounds = []
    for lower, upper, _, _ in boxes:
        local = local_height(((lower, upper, True, 1),))
        magnitude = max(abs(lower), abs(upper), Fraction(1))
        scaled = magnitude * local
        bounds.append(
            max(
                local,
                -((-scaled.numerator) // scaled.denominator),
            )
        )
    return max(bounds, default=0)


def primitive_global_shell(height: int):
    """Все primitive covectors ровно высоты max(|a|, |b|) == height."""

    for first in range(-height, height + 1):
        for second in (-height, height):
            if gcd(abs(first), abs(second)) == 1:
                yield first, second
    for second in range(-height + 1, height):
        for first in (-height, height):
            if gcd(abs(first), abs(second)) == 1:
                yield first, second


def search_global_height(
    metric,
    ideal,
    orientation,
    q,
    stop_height,
    *,
    local_candidate,
    best_fan,
):
    """Минимальный веер по единой, chart-invariant covector-высоте."""

    candidates = [set() for _ in range(len(ideal) - 2)]
    previous_counts = tuple(0 for _ in candidates)
    for height in range(1, stop_height + 1):
        for vector in primitive_global_shell(height):
            for ordinal in range(1, len(ideal) - 1):
                if local_candidate(
                    metric,
                    ideal,
                    ordinal,
                    vector,
                    q,
                ):
                    candidates[ordinal - 1].add(vector)
        fan = (
            best_fan(metric, ideal, orientation, q, candidates)
            if all(candidates)
            else None
        )
        if fan is not None:
            return height, fan, previous_counts, tuple(
                len(items) for items in candidates
            )
        previous_counts = tuple(len(items) for items in candidates)
    return None, None, previous_counts, tuple(
        len(items) for items in candidates
    )
