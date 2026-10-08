"""Независимый oracle PLAN_COMPILE: изменённые функции из afc989c до кэшей 54bf4b9.

Копируются только заменённые ветки; этот файл не импортирует их новые реализации.
Неизменённые векторные типы и интервальный предикат остаются общими зависимостями.
"""

from __future__ import annotations

import sympy as sp
from cftuv_envelope.contracts.envelopes import StripEnvelopeSpec
from cftuv_envelope.reference.angular import _density_runtime_vector
from mpmath import iv
from cftuv_envelope._density_policy import DensityIntervalEnclosureUnsupported, density_interval_enclosure
from cftuv_envelope.reference.planar_types import ExactQuadraticFieldUnsupported, exact_quadratic_value
from cftuv_envelope.reference.common import GeometryContext, ReferenceGeometryError
from cftuv_envelope.reference.contracts import ReferenceOutcome
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import ExactPlanarVector
from cftuv_envelope.contracts.analysis import TurnOrientation
from cftuv_envelope.reference.adaptive_density_fan import _IdealTuple, _vector, _expected_orientation
from cftuv_envelope.reference import adaptive_density_band as _band


def _incident_normal(
    context: GeometryContext, chain_use_id, anchor_vertex_id
) -> tuple[ExactPlanarVector, str]:
    strip = next(
        item
        for item in context.compilation.envelope_specs
        if isinstance(item, StripEnvelopeSpec)
        and next(
            seed.chain_use_id
            for seed in context.compilation.seeds
            if getattr(seed, "seed_id", None) == item.source_seed_id
        )
        == chain_use_id
    )
    segments = context.support_segments_for_use(
        chain_use_id, strip.envelope_spec_id.value
    )
    matches = [
        item
        for item in segments
        if anchor_vertex_id
        in (item.source_vertex_start_id, item.source_vertex_end_id)
    ]
    if len(matches) != 1:
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_CHAIN_SUPPORT_NOT_LINEAR,
            f"angular anchor is not a unique physical endpoint for {chain_use_id}",
        )
    return matches[0].owner_normal, matches[0].support_id


def _density_dot_expression(
    metric: ExactPlanarMetric,
    left: ExactPlanarVector,
    right: ExactPlanarVector,
) -> sp.Expr:
    """Dual-free Gram dot без generic normalization/factor."""

    lx, ly = metric.density_expressions(left)
    rx, ry = metric.density_expressions(right)
    return (
        lx * (metric.gram[0][0] * rx + metric.gram[0][1] * ry)
        + ly * (metric.gram[1][0] * rx + metric.gram[1][1] * ry)
    )


def _density_unit_from_squared(
    vector: ExactPlanarVector,
    squared: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> ExactPlanarVector:
    if _density_exact_sign(squared, metric) <= 0:
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "Density A support direction has non-positive Gram norm",
        )
    x, y = (
        vector.expressions()
        if metric is None
        else metric.density_expressions(vector)
    )
    length = sp.sqrt(squared)
    return _density_runtime_vector(x / length, y / length)


def _density_left_unit_normal(
    metric: ExactPlanarMetric,
    tangent: ExactPlanarVector,
) -> ExactPlanarVector:
    tx, ty = metric.density_expressions(tangent)
    covector_x = metric.gram[0][0] * tx + metric.gram[0][1] * ty
    covector_y = metric.gram[1][0] * tx + metric.gram[1][1] * ty
    sign = metric.owner_orientation_sign
    raw = _density_runtime_vector(
        -sign * covector_y,
        sign * covector_x,
    )
    orthogonality = sp.expand(
        _density_dot_expression(metric, tangent, raw)
    )
    if orthogonality != 0:
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            "Density A Gram-orthogonal basis construction is not exact",
        )
    return _density_unit_from_squared(
        raw,
        sp.expand(_density_dot_expression(metric, raw, raw)),
        metric,
    )


def _huber_density_interpolated_normals(
    metric: ExactPlanarMetric,
    incoming: ExactPlanarVector,
    outgoing: ExactPlanarVector,
    count: int,
    orientation_sign: int,
    canonical_excess_over_pi=None,
    rational_rotation=None,
) -> tuple[ExactPlanarVector, ...]:
    """Равноугольный веер H=1..5 без generic root solver.

    Власть ветви выводится из production Gram-фактов: знак `dot_g` и
    несократимый рациональный `dot_g²`.  `atan2(sqrt(1-c²), c)/(H+1)` —
    точная principal-ветвь того же корня `T_n(x)=c`; интервалы ниже лишь
    сертифицируют знаки/окна и никогда не подменяют конструкцию числом.
    """

    # `H = 0` — митрованный угол JOIN (`CORNER_JOIN_SOFT_BEND_V1`): опоры есть
    # только входящая и исходящая, и ниже проверяется лишь их поворот.
    if count not in range(0, 6):
        raise ReferenceGeometryError(
            ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN,
            "Density A supports only the certified H=0..5 range",
        )
    incoming_squared = sp.expand(
        _density_dot_expression(metric, incoming, incoming)
    )
    outgoing_squared = sp.expand(
        _density_dot_expression(metric, outgoing, outgoing)
    )
    raw_dot = sp.expand(
        _density_dot_expression(metric, incoming, outgoing)
    )
    raw_cross = sp.expand(
        metric.owner_orientation_sign
        * (
            metric.density_expressions(incoming)[0]
            * metric.density_expressions(outgoing)[1]
            - metric.density_expressions(incoming)[1]
            * metric.density_expressions(outgoing)[0]
        )
    )
    if _density_exact_sign(raw_cross, metric) != orientation_sign:
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "ordered support normals do not realize the certified owner-sector turn",
        )
    if (
        incoming_squared.is_Rational is not True
        or outgoing_squared.is_Rational is not True
    ):
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            "Density A source Gram norms are not rational",
        )
    incoming = _density_unit_from_squared(
        incoming,
        incoming_squared,
        metric,
    )
    outgoing = _density_unit_from_squared(
        outgoing,
        outgoing_squared,
        metric,
    )
    if count == 0:
        return incoming, outgoing
    raw_dot_squared = sp.expand(raw_dot * raw_dot)
    if raw_dot_squared.is_Rational is not True:
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            "Density A signed-cos-squared is not rational in the declared Gram metric",
        )
    cosine_squared = raw_dot_squared / (
        incoming_squared * outgoing_squared
    )
    if (
        _density_exact_sign(cosine_squared, metric) < 0
        or _density_exact_sign(cosine_squared - 1, metric) >= 0
    ):
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "Density A requires a strict principal turn in (0, pi)",
        )
    # Угол поворота нужен только равноугольной ветке; луч по таблице
    # (`rational_rotation`) его не читает, и считать его — пустая точная работа.
    principal_turn = None
    if rational_rotation is None:
        turn_sign = _density_exact_sign(raw_dot, metric)
        cosine_total = turn_sign * sp.sqrt(cosine_squared)
        sine_squared = 1 - cosine_squared
        principal_turn = sp.atan2(sp.sqrt(sine_squared), cosine_total)
    subturn_count = count + 1
    ix, iy = metric.density_expressions(incoming)
    lx, ly = metric.density_expressions(
        _density_left_unit_normal(metric, incoming)
    )
    hidden = []
    for ordinal in range(1, subturn_count):
        if rational_rotation is not None:
            # Закон `CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1`: луч ординала
            # направлен как `a * e + b * J e` по паре `(a, b)` ряда таблицы
            # (`e` — входящая единичная опора, `J e` — её левая нормаль);
            # направление целочисленное, длина — `sqrt(a^2 + b^2)`.
            ray_a, ray_b = rational_rotation[ordinal - 1]
            length = sp.sqrt(ray_a * ray_a + ray_b * ray_b)
            hidden.append(
                _density_runtime_vector(
                    (ray_a * ix + orientation_sign * ray_b * lx) / length,
                    (ray_a * iy + orientation_sign * ray_b * ly) / length,
                )
            )
            continue
        # Канонический веер: луч ординала ставится ТОЧНЫМ поворотом входящей
        # опоры на `ordinal * u_канон * pi / (H + 1)`. Формула та же, что у
        # точного близнеца, — у него `principal_turn` и есть канонический
        # угол, поэтому и выражение луча выходит буквально то же. Остаток
        # авторского шума целиком остаётся в ПОСЛЕДНЕМ секторе.
        angle = (
            sp.Rational(ordinal, subturn_count) * principal_turn
            if canonical_excess_over_pi is None
            else sp.pi
            * sp.Rational(
                ordinal * canonical_excess_over_pi.numerator,
                subturn_count * canonical_excess_over_pi.denominator,
            )
        )
        cosine = sp.cos(angle)
        sine = orientation_sign * sp.sin(angle)
        normal = _density_runtime_vector(
            cosine * ix + sine * lx,
            cosine * iy + sine * ly,
        )
        hidden.append(normal)
    # `principal_turn in (0, pi)` доказан signed-cos² и знаком cross.
    # Поэтому каждая разность соседних ordinal углов строго одного знака.
    return incoming, *hidden, outgoing


def _covectors(
    metric,
    ideal_unit_normals,
    window_law: str = _band.WINDOW_LAW_VORONOI,
    orientation: TurnOrientation | None = None,
):
    covectors = []
    for normal in ideal_unit_normals:
        nx, ny = metric.density_expressions(normal)
        covectors.append(
            _vector(
                metric.gram[0][0] * nx + metric.gram[0][1] * ny,
                metric.gram[1][0] * nx + metric.gram[1][1] * ny,
                metric,
            )
        )
    values = _IdealTuple(covectors)
    values.metric = metric
    values.window_law = window_law
    values.band_cache = {}
    values.band_orientation = (
        None if orientation is None else _expected_orientation(orientation)
    )
    return values


def _density_exact_sign(
    expression: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> int:
    """Знак Density-факта с memo конкретной metric-транзакции."""

    expression = sp.sympify(expression)
    if metric is not None:
        cached = metric._density_exact_memo.signs.get(expression)
        if cached is not None:
            return cached
    if isinstance(expression, sp.Rational):
        return (expression.p > 0) - (expression.p < 0)
    if expression == 0:
        return 0
    saved = iv.prec
    iv.prec = 160
    try:
        enclosure = density_interval_enclosure(
            expression,
            (
                None
                if metric is None
                else metric._density_exact_memo.intervals
            ),
        )
    except (
        DensityIntervalEnclosureUnsupported,
        ArithmeticError,
        TypeError,
        ValueError,
    ):
        enclosure = None
    finally:
        iv.prec = saved
    if enclosure is not None:
        if enclosure.a > 0:
            result = 1
            if metric is not None:
                metric._density_exact_memo.signs[expression] = result
            return result
        if enclosure.b < 0:
            result = -1
            if metric is not None:
                metric._density_exact_memo.signs[expression] = result
            return result
    try:
        result = exact_quadratic_value(sp.expand(expression)).sign()
        if metric is not None:
            metric._density_exact_memo.signs[expression] = result
        return result
    except ExactQuadraticFieldUnsupported:
        pass
    raise ReferenceGeometryError(
        ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
        "Density A exact sign is not certified without generic factorization",
    )
