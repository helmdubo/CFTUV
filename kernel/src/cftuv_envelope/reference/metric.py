"""Metric-aware exact operations for planar affine coordinates."""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import math

import sympy as sp

from ..contracts.analysis import PlanarPatchFrameV1
from ..contracts.metric import (
    AffineChartOrientationV1,
    ExactMatrix2V1,
    ExactRationalV1,
    GridSnappingLawV1,
    RationalAffinePlanarMetricV2,
)
from ..robust.grid import GridSpecV1, record_snap
from .planar_types import (
    ConstructionCertificate,
    ExactPlanarPoint,
    ExactPlanarVector,
    ExactScalar,
    IntervalEnclosureUnsupported,
    OrientedSupportLine,
    exact_normalize,
    exact_sign,
    interval_enclosure,
    point_add,
    vector_scale,
)


def _scalar(value: ExactRationalV1) -> sp.Rational:
    return sp.Rational(value.numerator, value.denominator)


def _matrix(value: ExactMatrix2V1) -> tuple[tuple[sp.Expr, sp.Expr], ...]:
    return (
        (_scalar(value.m00), _scalar(value.m01)),
        (_scalar(value.m10), _scalar(value.m11)),
    )


class SnapDisplacementExceedsBudget(ValueError):
    """Привязка сдвинула точку дальше объявленного бюджета."""


def _floor_exact(expression: sp.Expr) -> int:
    """Целая часть точного значения. Сначала фильтр, потом точный путь.

    Оболочка округляется наружу, поэтому совпадение целых частей у обеих её
    границ ДОКАЗЫВАЕТ целую часть; несовпадение уступает символьному
    `sp.floor`. Порога здесь нет — есть сертификат либо уступка.
    """

    if expression.is_Rational:
        return int(sp.floor(expression))
    try:
        enclosure = interval_enclosure(expression)
    except (IntervalEnclosureUnsupported, ArithmeticError, ValueError, TypeError):
        enclosure = None
    if enclosure is not None:
        lower, upper = math.floor(enclosure.a), math.floor(enclosure.b)
        if lower == upper:
            return int(lower)
    return int(sp.floor(expression))


def _residual_bound(expression: sp.Expr) -> Fraction:
    """Верхняя граница `|expression|` дробью, для счётчика сдвига.

    Счётчик — наблюдение, а не сертификат, и величина в нём объявлена
    границей: у алгебраической невязки точной дроби не существует, а врать про
    неё нельзя. Границу даёт та же оболочка, что и целую часть.
    """

    if expression.is_Rational:
        return Fraction(int(expression.p), int(expression.q))
    try:
        enclosure = interval_enclosure(expression)
    except (IntervalEnclosureUnsupported, ArithmeticError, ValueError, TypeError):
        return Fraction(1, 2)
    bound = max(abs(float(enclosure.a)), abs(float(enclosure.b)))
    if not math.isfinite(bound):
        return Fraction(1, 2)
    return min(Fraction(1, 2), Fraction(*float(bound).as_integer_ratio()))


def snap_exact_point(
    point: ExactPlanarPoint,
    grid: GridSpecV1,
    squared_budget: Fraction | None,
    law: str,
) -> ExactPlanarPoint:
    """Привязать точную (в общем случае алгебраическую) точку к решётке карты.

    Узел решается точно: `floor(v·scale + 1/2)` — то же правило «половина
    вверх», что и у `snap_value`, только вычисленное там, где дроби нет.
    """

    x, y = point.expressions()
    scale = grid.scale
    node_x = _floor_exact(x * scale + sp.Rational(1, 2))
    node_y = _floor_exact(y * scale + sp.Rational(1, 2))
    residual_x = _residual_bound(x * scale - node_x)
    residual_y = _residual_bound(y * scale - node_y)
    squared = (
        residual_x * residual_x + residual_y * residual_y
    ) / Fraction(scale * scale)
    record_snap(
        (node_x, node_y),
        (point.x.expression, point.y.expression),
        squared,
        bool(residual_x or residual_y),
    )
    if squared_budget is not None and squared > squared_budget:
        raise SnapDisplacementExceedsBudget(
            f"{law}: сдвиг привязки {squared} превысил бюджет {squared_budget}"
        )
    return ExactPlanarPoint.from_values(
        sp.Rational(node_x, scale), sp.Rational(node_y, scale)
    )


def _chart_grid(
    descriptor: RationalAffinePlanarMetricV2, gram
) -> tuple[GridSpecV1 | None, Fraction | None]:
    """Решётка карты и шаг решётки источника, если закон требует привязки."""

    certificate = descriptor.grid_certificate
    if certificate.snapping_law is not GridSnappingLawV1.INTEGER_GRID_SNAP_V1:
        return None, None
    from ..source_grid import chart_grid_for

    step = Fraction(
        certificate.window_step.numerator, certificate.window_step.denominator
    )
    return chart_grid_for(gram, step), step


@dataclass(frozen=True, slots=True)
class ExactPlanarMetric:
    gram: tuple[tuple[sp.Expr, sp.Expr], tuple[sp.Expr, sp.Expr]]
    inverse_gram: tuple[
        tuple[sp.Expr, sp.Expr], tuple[sp.Expr, sp.Expr]
    ]
    owner_orientation_sign: int
    # Решётка карты и шаг решётки источника в метрах. `None` — закон
    # `UNSNAPPED_EXACT_V1`: конструкции остаются точными и непривязанными.
    chart_grid: GridSpecV1 | None = None
    source_step: Fraction | None = None

    @classmethod
    def from_descriptor(
        cls,
        descriptor: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
    ) -> ExactPlanarMetric:
        if isinstance(descriptor, PlanarPatchFrameV1):
            identity = (
                (sp.Integer(1), sp.Integer(0)),
                (sp.Integer(0), sp.Integer(1)),
            )
            return cls(identity, identity, 1)
        orientation = (
            1
            if descriptor.chart_orientation
            is AffineChartOrientationV1.COORDINATE_CCW_MATCHES_OWNER_PATCH
            else -1
        )
        gram = _matrix(descriptor.exact_gram_matrix)
        chart_grid, source_step = _chart_grid(descriptor, gram)
        return cls(
            gram,
            _matrix(descriptor.exact_inverse_gram_matrix),
            orientation,
            chart_grid,
            source_step,
        )

    @property
    def squared_snap_budget(self) -> Fraction | None:
        """Объявленный бюджет сдвига привязки, в координатах карты.

        Привязка к решётке двигает точку не больше чем на половину ячейки по
        каждой оси, поэтому бюджет равен `2·(1/(2·scale))²`. На задуманном пути
        он не срабатывает никогда — и это не делает его лишним: функция
        объявляет границу, и её собственный результат обязан ей удовлетворять,
        иначе граница существует как заявление, которого никто не проверил.
        """

        if self.chart_grid is None:
            return None
        return Fraction(1, 2 * self.chart_grid.scale * self.chart_grid.scale)

    def dot_g(
        self, left: ExactPlanarVector, right: ExactPlanarVector
    ) -> sp.Expr:
        lx, ly = left.expressions()
        rx, ry = right.expressions()
        gx = self.gram[0][0] * rx + self.gram[0][1] * ry
        gy = self.gram[1][0] * rx + self.gram[1][1] * ry
        return exact_normalize(lx * gx + ly * gy)

    def length_g(self, vector: ExactPlanarVector) -> sp.Expr:
        squared = self.dot_g(vector, vector)
        if exact_sign(squared) <= 0:
            raise ValueError("zero vector has no metric length")
        return sp.sqrt(exact_normalize(squared))

    def unit_g(self, vector: ExactPlanarVector) -> ExactPlanarVector:
        return vector_scale(vector, 1 / self.length_g(vector))

    def oriented_cross(
        self, left: ExactPlanarVector, right: ExactPlanarVector
    ) -> sp.Expr:
        lx, ly = left.expressions()
        rx, ry = right.expressions()
        return exact_normalize(
            self.owner_orientation_sign * (lx * ry - ly * rx)
        )

    def owner_normal_g(
        self,
        tangent: ExactPlanarVector,
        *,
        owner_left: bool,
        normalize: bool = True,
    ) -> ExactPlanarVector:
        tx, ty = tangent.expressions()
        covector_x = self.gram[0][0] * tx + self.gram[0][1] * ty
        covector_y = self.gram[1][0] * tx + self.gram[1][1] * ty
        sign = self.owner_orientation_sign
        normal = ExactPlanarVector.from_values(
            -sign * covector_y,
            sign * covector_x,
        )
        if not owner_left:
            normal = vector_scale(normal, -1)
        return self.unit_g(normal) if normalize else normal

    def distance_to_support_g(
        self,
        point: ExactPlanarPoint,
        support_point: ExactPlanarPoint,
        unit_normal: ExactPlanarVector,
    ) -> sp.Expr:
        px, py = point.expressions()
        sx, sy = support_point.expressions()
        return self.dot_g(
            ExactPlanarVector.from_values(px - sx, py - sy),
            unit_normal,
        )

    def offset_support_g(
        self,
        point: ExactPlanarPoint,
        unit_normal: ExactPlanarVector,
        distance: object,
    ) -> ExactPlanarPoint:
        """Сместить опору на `distance` вдоль единичной нормали.

        Здесь и рождается радикал: `unit_normal` несёт `sqrt`, поэтому каждая
        построенная вершина огибающей алгебраична, и дальше это течёт во всё.
        Привязка ставится ВНУТРИ, один раз: функцию зовут из пяти мест
        (`strip.py:58,61`, `angular.py:209,210`, `cap.py:39`), и привязка в
        каждом из них была бы пятью разными законами вместо одного.
        """

        moved = point_add(point, vector_scale(unit_normal, distance))
        if self.chart_grid is None:
            return moved
        return snap_exact_point(
            moved, self.chart_grid, self.squared_snap_budget, "offset_support_g"
        )

    def support_covector_g(
        self, unit_normal: ExactPlanarVector
    ) -> ExactPlanarVector:
        nx, ny = unit_normal.expressions()
        return ExactPlanarVector.from_values(
            self.gram[0][0] * nx + self.gram[0][1] * ny,
            self.gram[1][0] * nx + self.gram[1][1] * ny,
        )

    def line_through_point_g(
        self,
        support_id: str,
        point: ExactPlanarPoint,
        unit_normal: ExactPlanarVector,
        construction: ConstructionCertificate,
        *,
        normal_speed: object = 1,
    ) -> OrientedSupportLine:
        px, py = point.expressions()
        covector = self.support_covector_g(unit_normal)
        cx, cy = covector.expressions()
        return OrientedSupportLine(
            support_id=support_id,
            normal=covector,
            constant=ExactScalar.from_value(cx * px + cy * py),
            normal_speed=ExactScalar.from_value(normal_speed),
            construction=construction,
        )

    def angle_g(
        self,
        incoming: ExactPlanarVector,
        outgoing: ExactPlanarVector,
    ) -> tuple[sp.Expr, sp.Expr]:
        left = self.unit_g(incoming)
        right = self.unit_g(outgoing)
        cosine = self.dot_g(left, right)
        determinant = exact_normalize(
            self.gram[0][0] * self.gram[1][1]
            - self.gram[0][1] * self.gram[1][0]
        )
        sine = exact_normalize(
            sp.sqrt(determinant) * self.oriented_cross(left, right)
        )
        return cosine, sine


def dot_G(
    metric: ExactPlanarMetric,
    left: ExactPlanarVector,
    right: ExactPlanarVector,
) -> sp.Expr:
    return metric.dot_g(left, right)


def length_G(
    metric: ExactPlanarMetric,
    vector: ExactPlanarVector,
) -> sp.Expr:
    return metric.length_g(vector)


def unit_G(
    metric: ExactPlanarMetric,
    vector: ExactPlanarVector,
) -> ExactPlanarVector:
    return metric.unit_g(vector)


def owner_normal_G(
    metric: ExactPlanarMetric,
    tangent: ExactPlanarVector,
    *,
    owner_left: bool,
    normalize: bool = True,
) -> ExactPlanarVector:
    return metric.owner_normal_g(
        tangent,
        owner_left=owner_left,
        normalize=normalize,
    )


def distance_to_support_G(
    metric: ExactPlanarMetric,
    point: ExactPlanarPoint,
    support_point: ExactPlanarPoint,
    unit_normal: ExactPlanarVector,
) -> sp.Expr:
    return metric.distance_to_support_g(point, support_point, unit_normal)


def offset_support_G(
    metric: ExactPlanarMetric,
    point: ExactPlanarPoint,
    unit_normal: ExactPlanarVector,
    distance: object,
) -> ExactPlanarPoint:
    return metric.offset_support_g(point, unit_normal, distance)


def angle_G(
    metric: ExactPlanarMetric,
    incoming: ExactPlanarVector,
    outgoing: ExactPlanarVector,
) -> tuple[sp.Expr, sp.Expr]:
    return metric.angle_g(incoming, outgoing)
