"""Построитель `SurfaceMetricDescriptorV2`. Вычисления живут ЗДЕСЬ, не в контракте.

Разрез взят с планарной метрики: `contracts/metric.py` описывает, что
записано, `planar_metric.py` это считает. Смешать их значило бы, что читатель
контракта обязан читать алгоритм, чтобы понять запись.

Порядок один и обратного не бывает: сначала привязка ИСТОЧНИКА к решётке
(`source_grid.resolve_source_grid` — тот же закон, что у планарной метрики,
импортом, а не копией), потом всё остальное. Квадраты длин, матрицы Грама и
углы считаются ПО ПРИВЯЗАННЫМ позициям, поэтому они рациональны точно, а не
«почти»; несортированные допроекционные координаты в метрику не попадают вовсе
и потому не могут быть перепутаны с ними.

Хост даёт топологию и позиции. Квадраты длин считает ЯДРО — это записано в
AUTH развилок S0/S2 и держится здесь: величина, посчитанная на хосте, не имела
бы ни сертификата решётки, ни воспроизводимости.
"""

from __future__ import annotations

from decimal import Decimal
from fractions import Fraction

from .contracts.metric import (
    ExactMatrix2V1,
    ExactPoint3V1,
    ExactRationalV1,
    GridSnappingLawV1,
)
from .contracts.surface_adjacency import (
    SurfaceAdjacencyIRV1,
    VertexFanUnavailableV1,
    surface_adjacency_digest,
)
from .contracts.surface_metric_v2 import (
    SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA,
    ConeAngleMeasureLawV1,
    ConeAngleVerdictV1,
    DegenerateTriangleReasonV1,
    DegenerateTriangleV1,
    IntrinsicSurfaceGeometryV2,
    NamedEpsilonV1,
    SnappedSourcePositionV1,
    SurfaceAdjacencyRefV1,
    SurfaceMetricDescriptorV2,
    SurfaceMetricEvaluationLawV1,
    SurfaceMetricPrecisionTierV1,
    SurfacePeriodicTopologyV1,
    TriangleGramV1,
    TriangleSquaredLengthsV1,
    VertexConeAngleV1,
)
from .ids import LawId
from .source_grid import resolve_source_grid
from .surface_cone_angle import (
    CONE_ANGLE_EPSILON_NAME,
    angle_bounds,
    certified_cone_angle,
    two_pi_bounds,
)


def _rational(value: Fraction) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def _point(position) -> ExactPoint3V1:
    return ExactPoint3V1(*(_rational(item) for item in position))


def _sub(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _dot(left, right) -> Fraction:
    return sum((a * b for a, b in zip(left, right, strict=True)), Fraction(0))


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def triangle_squared_sides(triangle, positions) -> tuple[Fraction, Fraction, Fraction]:
    """Квадраты длин сторон по ЯДЕРНОЙ нумерации: сторона `i` — `(v[i], v[i+1])`."""

    corners = tuple(positions[vertex_id] for vertex_id in triangle.vertex_ids)
    result = []
    for ordinal in range(3):
        edge = _sub(corners[(ordinal + 1) % 3], corners[ordinal])
        result.append(_dot(edge, edge))
    return tuple(result)


def triangle_gram(triangle, positions) -> tuple[Fraction, Fraction, Fraction]:
    """Грам пары рёбер `(v0→v1, v0→v2)`: `(g00, g01, g11)`."""

    corners = tuple(positions[vertex_id] for vertex_id in triangle.vertex_ids)
    first = _sub(corners[1], corners[0])
    second = _sub(corners[2], corners[0])
    return _dot(first, first), _dot(first, second), _dot(second, second)


def _degenerate_reason(squared, gram):
    if any(value == 0 for value in squared):
        return DegenerateTriangleReasonV1.COINCIDENT_SNAPPED_VERTICES
    g00, g01, g11 = gram
    if g00 * g11 - g01 * g01 == 0:
        return DegenerateTriangleReasonV1.COLLINEAR_SNAPPED_VERTICES
    return None


def _intrinsic(triangles, positions):
    lengths, grams, degenerate = [], [], []
    squared_by_triangle = {}
    for triangle in triangles:
        squared = triangle_squared_sides(triangle, positions)
        gram = triangle_gram(triangle, positions)
        squared_by_triangle[triangle.triangle_id] = squared
        lengths.append(
            TriangleSquaredLengthsV1(
                triangle_id=triangle.triangle_id,
                side_0=_rational(squared[0]),
                side_1=_rational(squared[1]),
                side_2=_rational(squared[2]),
            )
        )
        grams.append(
            TriangleGramV1(
                triangle_id=triangle.triangle_id,
                gram=ExactMatrix2V1(
                    m00=_rational(gram[0]),
                    m01=_rational(gram[1]),
                    m10=_rational(gram[1]),
                    m11=_rational(gram[2]),
                ),
            )
        )
        reason = _degenerate_reason(squared, gram)
        if reason is not None:
            degenerate.append(
                DegenerateTriangleV1(
                    triangle_id=triangle.triangle_id, reason=reason
                )
            )
    geometry = IntrinsicSurfaceGeometryV2(
        triangle_squared_lengths=frozenset(lengths),
        triangle_gram_matrices=frozenset(grams),
        degenerate_triangles=frozenset(degenerate),
    )
    return geometry, squared_by_triangle


def _angle_at_vertex(triangle, vertex_id, squared):
    """Границы угла треугольника при названной вершине."""

    index = triangle.vertex_ids.index(vertex_id)
    adjacent_first = squared[index]
    adjacent_second = squared[(index + 2) % 3]
    opposite = squared[(index + 1) % 3]
    return angle_bounds(adjacent_first, adjacent_second, opposite)


def _triangle_normal(triangle, positions):
    corners = tuple(positions[vertex_id] for vertex_id in triangle.vertex_ids)
    return _cross(_sub(corners[1], corners[0]), _sub(corners[2], corners[0]))


def _fan_is_exactly_planar(triangle_ids, triangles, positions) -> bool:
    """Точная компланарность веера в ПРИВЯЗАННЫХ рациональных координатах.

    Ни одного числа с плавающей точкой: коллинеарность нормалей проверяется
    векторным произведением рациональных векторов и сравнением с нулём.
    """

    reference = None
    for triangle_id in triangle_ids:
        normal = _triangle_normal(triangles[triangle_id], positions)
        if all(component == 0 for component in normal):
            return False
        if reference is None:
            reference = normal
            continue
        if any(component != 0 for component in _cross(reference, normal)):
            return False
    return reference is not None


def _verdict(enclosure, exactly_planar_closed_fan, fan_available):
    two_pi_low, two_pi_high = two_pi_bounds()
    lower, upper = Fraction(enclosure.lower), Fraction(enclosure.upper)
    if not fan_available:
        return (
            ConeAngleVerdictV1.UNDECIDED_FAIL_CLOSED,
            ConeAngleMeasureLawV1.FAN_UNAVAILABLE_V1,
        )
    if exactly_planar_closed_fan and lower <= two_pi_low and two_pi_high <= upper:
        return (
            ConeAngleVerdictV1.EXACT_TWO_PI,
            ConeAngleMeasureLawV1.EXACT_PLANAR_CLOSED_FAN_V1,
        )
    law = ConeAngleMeasureLawV1.CERTIFIED_INTERVAL_ENCLOSURE_V1
    if upper < two_pi_low:
        return ConeAngleVerdictV1.STRICTLY_LESS, law
    if lower > two_pi_high:
        return ConeAngleVerdictV1.STRICTLY_GREATER, law
    return ConeAngleVerdictV1.UNDECIDED_FAIL_CLOSED, law


def _cone_angles(adjacency, triangles, positions, squared_by_triangle, incidence):
    result = []
    for fan in adjacency.vertex_fans:
        available = not isinstance(fan, VertexFanUnavailableV1)
        members = (
            fan.ordered_triangle_ids
            if available
            else tuple(sorted(incidence[fan.vertex_id], key=lambda item: item.value))
        )
        bounds = [
            _angle_at_vertex(
                triangles[triangle_id],
                fan.vertex_id,
                squared_by_triangle[triangle_id],
            )
            for triangle_id in members
        ]
        enclosure = certified_cone_angle(bounds)
        planar_closed = (
            available
            and fan.is_closed
            and _fan_is_exactly_planar(members, triangles, positions)
        )
        verdict, law = _verdict(enclosure, planar_closed, available)
        result.append(
            VertexConeAngleV1(
                vertex_id=fan.vertex_id,
                enclosure=enclosure,
                verdict=verdict,
                measure_law=law,
            )
        )
    return frozenset(result)


def _named_epsilon(cone_angles, precision_tier):
    if precision_tier is SurfaceMetricPrecisionTierV1.REFERENCE_EXACT_SMALL:
        return None
    widest = max(
        (item.enclosure.absolute_error_bound for item in cone_angles),
        default=Decimal(0),
    )
    return NamedEpsilonV1(
        name=LawId(CONE_ANGLE_EPSILON_NAME), absolute_bound=widest
    )


def _incidence(triangles):
    incidence: dict = {}
    for triangle in triangles.values():
        for vertex_id in triangle.vertex_ids:
            incidence.setdefault(vertex_id, set()).add(triangle.triangle_id)
    return incidence


def build_surface_metric_v2(
    *,
    patch_domain_id,
    surface,
    adjacency: SurfaceAdjacencyIRV1,
    source_positions,
    surface_regime,
    snapping_law: GridSnappingLawV1 = GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    barriers=frozenset(),
    precision_tier: SurfaceMetricPrecisionTierV1 = (
        SurfaceMetricPrecisionTierV1.REFERENCE_CERTIFIED
    ),
) -> SurfaceMetricDescriptorV2:
    """Собрать метрику V2 из топологии, смежности и ДОПРОЕКЦИОННЫХ позиций.

    `source_positions` — точные рациональные тройки ДО привязки; привязку
    делает эта функция и записывает её сертификатом. Принимать уже привязанные
    позиции значило бы, что сертификат описывает не то, что произошло.
    """

    facts = resolve_source_grid(
        positions=dict(source_positions),
        faces=surface.source_faces,
        snapping_law=snapping_law,
    )
    triangles = {item.triangle_id: item for item in surface.surface_triangles}
    geometry, squared_by_triangle = _intrinsic(
        surface.surface_triangles, facts.positions
    )
    cone_angles = _cone_angles(
        adjacency,
        triangles,
        facts.positions,
        squared_by_triangle,
        _incidence(triangles),
    )
    return SurfaceMetricDescriptorV2(
        schema_version=SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA,
        patch_domain_id=patch_domain_id,
        source_revision=surface.source_revision,
        surface_regime=surface_regime,
        evaluation_law=(
            SurfaceMetricEvaluationLawV1.EVALUATED_TRIANGULATED_MESH_AFTER_SOURCE_SNAP_V1
        ),
        precision_tier=precision_tier,
        named_epsilon=_named_epsilon(cone_angles, precision_tier),
        grid_certificate=facts.certificate,
        snapped_source_positions=frozenset(
            SnappedSourcePositionV1(
                source_vertex_id=vertex_id, position=_point(position)
            )
            for vertex_id, position in facts.positions.items()
        ),
        intrinsic=geometry,
        adjacency_ref=SurfaceAdjacencyRefV1(
            source_revision=adjacency.source_revision,
            scope=adjacency.scope,
            adjacency_digest=surface_adjacency_digest(adjacency),
        ),
        barriers=frozenset(barriers),
        vertex_cone_angles=cone_angles,
        periodic_topology=SurfacePeriodicTopologyV1.NON_PERIODIC_V1,
    )


__all__ = (
    "build_surface_metric_v2",
    "triangle_gram",
    "triangle_squared_sides",
)
