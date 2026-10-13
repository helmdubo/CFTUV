"""Равноугольный Density-веер и общие факты одного owner-sector поворота."""

from __future__ import annotations

import sympy as sp

from .common import ReferenceGeometryError
from .contracts import ReferenceOutcome
from .density_sign import _density_exact_sign
from .density_vectors import (
    _density_dot_expression,
    _density_left_unit_normal,
    _density_runtime_vector,
    _density_unit_from_squared,
)
from .metric import ExactPlanarMetric
from .planar_types import ExactPlanarVector


class _TurnAtom:
    """Всё, что веер Huber читает об УГЛЕ и что не зависит от числа скрытых лучей.

    Один угол (входящая и исходящая опоры) строят на нескольких счетах (`H`, `H + 1`, ...) и в двух
    геометриях транзакции; общая часть веера — квадраты норм, единичные опоры, `cos^2` и главный
    поворот — для всех них одна. Поля заполняются по ступеням, в том же порядке и с теми же отказами,
    что прежде: отказавшая ступень в память не пишется и на повторе отказывает так же.
    """

    __slots__ = (
        "incoming_squared",
        "outgoing_squared",
        "raw_dot",
        "incoming",
        "outgoing",
        "cosine_squared",
        "turn_sign",
        "principal_turn",
        "basis",
    )

    def __init__(self, incoming_squared, outgoing_squared, raw_dot, incoming, outgoing):
        self.incoming_squared = incoming_squared
        self.outgoing_squared = outgoing_squared
        self.raw_dot = raw_dot
        self.incoming = incoming
        self.outgoing = outgoing
        self.cosine_squared = None
        self.turn_sign = None
        self.principal_turn = None
        self.basis = None


def _turn_atom(metric, incoming, outgoing, orientation_sign):
    """Ступень A веера: квадраты норм, знак поворота и единичные опоры (общая часть всех счетов)."""

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
    return _TurnAtom(
        incoming_squared,
        outgoing_squared,
        raw_dot,
        _density_unit_from_squared(incoming, incoming_squared, metric),
        _density_unit_from_squared(outgoing, outgoing_squared, metric),
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
    memo = metric._density_exact_memo
    incoming_x, incoming_y = metric.density_expressions(incoming)
    outgoing_x, outgoing_y = metric.density_expressions(outgoing)
    atom_key = (incoming_x, incoming_y, outgoing_x, outgoing_y, orientation_sign)
    atom = memo.turn_atoms.get(atom_key)
    if atom is None:
        atom = memo.turn_atoms[atom_key] = _turn_atom(
            metric, incoming, outgoing, orientation_sign
        )
    if count == 0:
        return atom.incoming, atom.outgoing
    fan_key = (atom_key, count, canonical_excess_over_pi, rational_rotation)
    cached = memo.fans.get(fan_key)
    if cached is not None:
        return cached
    if atom.cosine_squared is None:
        raw_dot_squared = sp.expand(atom.raw_dot * atom.raw_dot)
        if raw_dot_squared.is_Rational is not True:
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
                "Density A signed-cos-squared is not rational in the declared Gram metric",
            )
        cosine_squared = raw_dot_squared / (
            atom.incoming_squared * atom.outgoing_squared
        )
        if (
            _density_exact_sign(cosine_squared, metric) < 0
            or _density_exact_sign(cosine_squared - 1, metric) >= 0
        ):
            raise ReferenceGeometryError(
                ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
                "Density A requires a strict principal turn in (0, pi)",
            )
        atom.cosine_squared = cosine_squared
    cosine_squared = atom.cosine_squared
    # Угол поворота нужен только равноугольной ветке; луч по таблице
    # (`rational_rotation`) его не читает, и считать его — пустая точная работа.
    principal_turn = None
    if rational_rotation is None:
        if atom.principal_turn is None:
            turn_sign = _density_exact_sign(atom.raw_dot, metric)
            cosine_total = turn_sign * sp.sqrt(cosine_squared)
            sine_squared = 1 - cosine_squared
            atom.turn_sign = turn_sign
            atom.principal_turn = sp.atan2(sp.sqrt(sine_squared), cosine_total)
        principal_turn = atom.principal_turn
    subturn_count = count + 1
    if atom.basis is None:
        ix, iy = metric.density_expressions(atom.incoming)
        lx, ly = metric.density_expressions(
            _density_left_unit_normal(metric, atom.incoming)
        )
        atom.basis = (ix, iy, lx, ly)
    ix, iy, lx, ly = atom.basis
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
    fan = (atom.incoming, *hidden, atom.outgoing)
    memo.fans[fan_key] = fan
    return fan
