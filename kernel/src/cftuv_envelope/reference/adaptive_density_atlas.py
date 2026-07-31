"""Exact coordinate-atlas helpers for adaptive Density fan authority."""

from __future__ import annotations

from dataclasses import dataclass, replace
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
_EXHAUSTED = "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"


class DensityExactWorkBudget:
    """Детерминированный счётчик точной работы поиска минимального D*.

    Считается только точная работа. Wall-clock и любые временные лимиты
    здесь запрещены: время прогона не воспроизводимо побитово, а число
    выполненных exact-предикатов воспроизводимо и одинаково на любой машине.

    `shell_probes` — сколько примитивных ковекторов проверено предикатом
    локальной допустимости: ровно один probe на пару (перечисленный вектор
    оболочки, ordinal-окно). Это стоимость перечисления Farey/atlas-оболочки.

    `order_steps` — сколько кандидатов тронуто при Gram-упорядочивании и
    сборке веера: вход сортировки на каждый вызов плюс каждое ребро
    backtracking-обхода.

    Сумма двух счётчиков ограничивает всю точную работу поиска: размер
    накопленных множеств кандидатов сам не превышает `shell_probes`, поэтому
    ни сортировка, ни обход не могут стоить больше, чем оплачено.
    """

    __slots__ = ("cap", "exhausted", "shell_probes", "order_steps")

    def __init__(self, exhausted, cap: int) -> None:
        self.exhausted = exhausted
        self.cap = cap
        self.shell_probes = 0
        self.order_steps = 0

    @property
    def spent(self) -> int:
        return self.shell_probes + self.order_steps

    def counters(self) -> tuple[tuple[str, int], ...]:
        return (
            ("DENSITY_FAN_SHELL_PROBES", self.shell_probes),
            ("DENSITY_FAN_ORDER_STEPS", self.order_steps),
        )

    def spend_shell_probes(self, count: int) -> None:
        self.shell_probes += count
        self._enforce()

    def spend_order_steps(self, count: int) -> None:
        self.order_steps += count
        self._enforce()

    def _enforce(self) -> None:
        if self.spent > self.cap:
            raise self.exhausted(
                f"{_EXHAUSTED} cap={self.cap} "
                f"shell_probes={self.shell_probes} "
                f"order_steps={self.order_steps}"
            )


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


@dataclass(frozen=True, slots=True)
class GlobalHeightSegment:
    """Один cell-кусок ordinal-окна: карта плюс sealed рациональная оболочка.

    Внутри одной канонической ячейки выполняется |slope| <= 1, поэтому
    max-норма ковектора РАВНА знаменателю его наклона в карте этой ячейки.
    Значит перечисление числителей при фиксированном знаменателе — это и есть
    перечисление по chart-invariant высоте, и закон минимальности не меняется:
    меняется только цена его проверки.

    `outer_*` — доказанное надмножество допустимого в этом куске, `inner_*` —
    доказанное подмножество (пустое, если inner_lower >= inner_upper).
    """

    use_x_denominator: bool
    denominator_sign: int
    outer_lower: Fraction
    inner_lower: Fraction
    inner_upper: Fraction
    outer_upper: Fraction


def cell_segments(use_x: bool, denominator_sign: int, bracket):
    """Разрезать sealed-оболочку карты по границам ячеек |slope| == 1.

    Часть оболочки с |slope| >= 1 лежит в соседней ячейке: там знаменателем
    max-нормы служит другая координата. Она переводится обратной картой
    slope' = 1/slope со знаком знаменателя `denominator_sign * sign(slope)`,
    после чего снова |slope'| <= 1 и высота опять равна знаменателю.
    """

    outer_lower, inner_lower, inner_upper, outer_upper = bracket
    if outer_lower > outer_upper:
        return ()
    cuts = [outer_lower]
    cuts.extend(
        value
        for value in (Fraction(-1), Fraction(1))
        if outer_lower < value < outer_upper
    )
    cuts.append(outer_upper)
    segments = []
    for low, high in zip(cuts, cuts[1:]):
        inner = (max(inner_lower, low), min(inner_upper, high))
        if inner[0] >= inner[1]:
            inner = None
        if low >= 1 or high <= -1:
            chart = (
                not use_x,
                denominator_sign if low >= 1 else -denominator_sign,
            )
            bounds = (1 / high, 1 / low)
            inner = None if inner is None else (1 / inner[1], 1 / inner[0])
        else:
            chart = (use_x, denominator_sign)
            bounds = (low, high)
        segments.append(
            GlobalHeightSegment(
                use_x_denominator=chart[0],
                denominator_sign=chart[1],
                outer_lower=bounds[0],
                inner_lower=Fraction(0) if inner is None else inner[0],
                inner_upper=Fraction(0) if inner is None else inner[1],
                outer_upper=bounds[1],
            )
        )
    return tuple(segments)


def reachable_pieces(item, *, admissible, invalid):
    """Куски, до которых вообще дотягивается допустимый конус ordinal.

    Допустимое множество ordinal — пересечение полуплоскостей, то есть
    выпуклый конус, и он содержит окрестность ideal, лежащего в
    termination-куске. По порядку поворота он связен: если общий вектор
    перехода между соседними кусками недопустим, за ним недопустимо всё.
    Один exact-предикат на границу решает, метётся ли соседний кусок вовсе,
    и обход не может вернуться назад.
    """

    termination_piece(item, invalid=invalid)
    pieces = item.pieces
    start = item.termination_piece_index
    reached = {start}
    for step in (-1, 1):
        index = start
        while 0 <= index + step < len(pieces):
            left = min(index, index + step)
            if not admissible(
                _transition_vector(
                    _piece_cell_index(pieces[left]),
                    _piece_cell_index(pieces[left + 1]),
                    invalid=invalid,
                )
            ):
                break
            index += step
            reached.add(index)
    return tuple(sorted(reached))


@dataclass(frozen=True, slots=True)
class SegmentPlanLaws:
    """Точные предикаты Density, которыми этот модуль не владеет.

    Модуль atlas отвечает за карты, ячейки и оболочки; знаки, наклоны и
    предикат допустимости остаются в `adaptive_density_fan`. Собранные в одну
    запись, они не размазываются по семи именованным аргументам.
    """

    sign: object
    slope: object
    decimal_envelope: object
    local_candidate: object
    vector_from_slope: object
    subturn_boundary_vectors: object
    invalid: object


def piece_bracket(metric, boundary_vectors, piece, laws):
    """Sealed-оболочка одного куска по тем же двум subturn-лучам.

    Луч, чей знаменатель в карте куска имеет другой знак, здесь не
    применяется: его перенос в эту карту не был бы утверждением о самом луче.
    Пропуск ограничения только РАСШИРЯЕТ outer-оболочку, то есть она остаётся
    надмножеством; inner-часть после этого доказывается концами отдельно.
    """

    use_x = piece.use_x_denominator
    lower_bounds = [piece.lower_slope_envelope]
    upper_bounds = [piece.upper_slope_envelope]
    middle = (
        Fraction(piece.lower_slope_envelope.upper)
        + Fraction(piece.upper_slope_envelope.lower)
    ) / 2
    for boundary in boundary_vectors:
        denominator = metric.density_expressions(boundary)[0 if use_x else 1]
        if laws.sign(denominator, metric) != piece.denominator_sign:
            continue
        envelope = laws.decimal_envelope(
            laws.slope(boundary, use_x, metric),
            metric,
        )
        if Fraction(envelope.upper) < middle:
            lower_bounds.append(envelope)
        elif Fraction(envelope.lower) > middle:
            upper_bounds.append(envelope)
    lower = max(lower_bounds, key=lambda item: item.upper)
    upper = min(upper_bounds, key=lambda item: item.lower)
    return (
        Fraction(lower.lower),
        Fraction(lower.upper),
        Fraction(upper.lower),
        Fraction(upper.upper),
    )


def proven_segment(metric, ideal, ordinal, q, segment, laws):
    """Оставить inner-быстрый путь только если оба его конца допустимы.

    Допустимое множество выпукло, а наклон внутри одной карты — монотонная
    параметризация направлений, поэтому допустимость обоих концов замкнутого
    интервала доказывает допустимость всего интервала. Это ровно тот закон,
    которым запечатан одно-картный sealed-интервал.
    """

    if segment.inner_lower >= segment.inner_upper or all(
        laws.local_candidate(
            metric,
            ideal,
            ordinal,
            laws.vector_from_slope(
                value,
                segment.use_x_denominator,
                segment.denominator_sign,
            ),
            q,
        )
        for value in (segment.inner_lower, segment.inner_upper)
    ):
        return segment
    return replace(segment, inner_lower=Fraction(0), inner_upper=Fraction(0))


def ordinal_segments(
    metric,
    ideal,
    ordinal,
    orientation,
    q,
    window,
    record,
    sealed,
    laws,
):
    """План по-кускового перечисления одного ordinal под atlas-законом."""

    pieces = getattr(window, "pieces", None)
    if pieces is None:
        raw = cell_segments(record[0], record[1], sealed)
    else:
        boundary_vectors = laws.subturn_boundary_vectors(
            metric,
            ideal,
            ordinal,
            orientation,
            q,
        )
        raw = ()
        for index in reachable_pieces(
            window,
            admissible=lambda vector: laws.local_candidate(
                metric,
                ideal,
                ordinal,
                vector,
                q,
            ),
            invalid=laws.invalid,
        ):
            piece = pieces[index]
            raw += cell_segments(
                piece.use_x_denominator,
                piece.denominator_sign,
                sealed
                if index == window.termination_piece_index
                else piece_bracket(metric, boundary_vectors, piece, laws),
            )
    return tuple(
        proven_segment(metric, ideal, ordinal, q, item, laws) for item in raw
    )


def search_segments(
    metric,
    ideal,
    orientation,
    q,
    windows,
    records,
    sealed_intervals,
    laws,
):
    """По одному плану на ordinal; одно-картное окно — тоже один план."""

    return tuple(
        ordinal_segments(
            metric,
            ideal,
            ordinal,
            orientation,
            q,
            window,
            record,
            sealed,
            laws,
        )
        for ordinal, (window, record, sealed) in enumerate(
            zip(windows, records, sealed_intervals, strict=True),
            start=1,
        )
    )


def numerator_bounds(lower: Fraction, upper: Fraction, height: int):
    """Замкнутые границы числителя внутри ячейки на высоте `height`.

    Границы куска бывают ТОЧНЫМИ (переход между ячейками — это ровно
    |slope| == 1), поэтому здесь, в отличие от одно-картного закона, концы
    включаются: иначе диагональный ковектор (±1, ±1) высоты 1 не был бы
    перечислен ни одной из двух соседних ячеек. Зажим |numerator| <= height —
    это и есть определение ячейки: за ним max-норма перестаёт быть
    знаменателем.
    """

    scaled_lower = lower * height
    scaled_upper = upper * height
    first = -((-scaled_lower.numerator) // scaled_lower.denominator)
    last = scaled_upper.numerator // scaled_upper.denominator
    return max(first, -height), min(last, height)


def segment_candidates(
    metric,
    ideal,
    ordinal,
    q,
    segment,
    height,
    budget,
    *,
    local_candidate,
    vector_from_slope,
):
    """Числители одного cell-куска на одной chart-invariant высоте."""

    use_x = segment.use_x_denominator
    denominator_sign = segment.denominator_sign
    first, last = numerator_bounds(
        segment.outer_lower,
        segment.outer_upper,
        height,
    )
    # Пустая высота тоже стоит один probe: иначе последовательность пустых
    # высот прошла бы под капом без единой единицы работы.
    budget.spend_shell_probes(max(last - first + 1, 1))
    if first > last:
        return ()
    candidates = []
    for numerator in range(first, last + 1):
        if gcd(abs(numerator), height) != 1:
            continue
        slope = Fraction(numerator, height)
        vector = vector_from_slope(slope, use_x, denominator_sign)
        if (
            segment.inner_lower < slope < segment.inner_upper
            or local_candidate(metric, ideal, ordinal, vector, q)
        ):
            candidates.append(vector)
    return tuple(candidates)


def search_global_height(
    metric,
    ideal,
    orientation,
    q,
    stop_height,
    *,
    segments,
    local_candidate,
    vector_from_slope,
    best_fan,
    budget,
):
    """Минимальный веер по единой, chart-invariant covector-высоте.

    Оболочка max-нормы целиком НЕ метётся: примитивных ковекторов ровно
    высоты h не больше 8h, и проход по высотам стоил бы 4*H*(H+1)*k, то есть
    квадратично, а исчерпание бюджета было бы артефактом закона перечисления,
    а не глубины задачи. Вместо этого каждая высота стоит суммы по кускам
    ширины sealed-оболочки куска: w*h + 1 числителей, как в одно-картной
    ветви. Минимум на объединении кусков — минимум по-кусковых минимумов,
    поэтому закон минимальности прежний.
    """

    candidates = [set() for _ in segments]
    previous_counts = tuple(0 for _ in candidates)
    if not all(segments):
        return None, None, previous_counts, previous_counts
    for height in range(1, stop_height + 1):
        for ordinal, plan in enumerate(segments, start=1):
            for segment in plan:
                candidates[ordinal - 1].update(
                    segment_candidates(
                        metric,
                        ideal,
                        ordinal,
                        q,
                        segment,
                        height,
                        budget,
                        local_candidate=local_candidate,
                        vector_from_slope=vector_from_slope,
                    )
                )
        fan = (
            best_fan(metric, ideal, orientation, q, candidates, budget)
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
