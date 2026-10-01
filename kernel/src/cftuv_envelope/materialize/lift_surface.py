"""Укладка меша на ТРЕУГОЛЬНИКИ ИСТОЧНИКА: точное нахождение и барицентрический подъём.

Закон `NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1`. Домен near-planar считается на
карте — проекции его источника на сертифицированную плоскость; подъём в 3D
(`lift.PlaneLiftV1`) кладёт меш на ПЛОСКОСТЬ. Здесь меш кладётся на саму
поверхность: точка карты находится в проекции треугольника источника, а её 3D —
тот же барицентрический образ в ПРИВЯЗАННЫХ (до проекции) вершинах треугольника.

КАРТА — РЕШЁТКА ПОКРЫТИЯ. Покрытие считается в области, чьи вершины мост привязал
к узлам решётки карты (`bridge._lattice_image`, `snap_value`), поэтому точка меша
может лежать на полуячейки вне ИСТИННОЙ проекции источника (замер на `building`:
три из четырёх near-planar доменов). Триангуляция укладки строится в ТОЙ ЖЕ
решётке: вершины проекций — узлы `snap_value(u, решётка)`, и граница покрытия
совпадает с границей триангуляции. Сдвиг записан (число сдвинутых вершин и
наибольшее смещение в диагностике), а переворот знака площади какого-либо
треугольника — именованный отказ `SURFACE_LIFT_CHART_SNAP_FLIPPED_TRIANGLE`:
сохранение ориентации у всех треугольников при неизменной простой границе и есть
доказательство, что привязанная триангуляция осталась вложением.

ТОЧНО. Точка покрытия — пара `SqrtSumV1` (единицы решётки), вершины проекций —
целые точки решётки.
Принадлежность замкнутому треугольнику — три знака ориентации по рёбрам, и
каждый знак — точный знак `SqrtSumV1` под бюджетом транзакции `MATERIALIZE`
(`sign(budget=...)`): рациональная точка закрывается без радикалов, радикальная —
целочисленной оболочкой либо сопряжением, и бюджет считает вторую. Допуска нет.
Бокс-префильтр на outward-округлённых float — фильтр, который либо отбрасывает
треугольник доказуемо, либо уступает точному пути, ответ он не меняет.

ТОЖДЕСТВО НА РЕБРЕ. Точка на общем ребре двух треугольников лежит в обоих
замкнутых; выбор — ПЕРВЫЙ по имени треугольника, и он канонический, но от него
значение подъёма не зависит: барицентрические координаты вдоль ребра зависят
только от концов ребра, а концы у обоих треугольников одни, поэтому подъём с
двух сторон равен побитово (один и тот же `SqrtSumV1`, одно округление
`sqrt_sum_binary64`). Это и есть непрерывность шва между доменами: границы
домена лежат на рёбрах источника, подъём на ребре единственен.

ОТКАЗЫ ИМЕНОВАНЫ. Точка вне проекции всей триангуляции — не «ближайший
треугольник» и не допуск, а `SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION`
с числами. Исчерпание бюджета — `ExactCanonicalizationWorkBudgetExhausted`,
которое материализатор называет `EXACT_WORK_BUDGET_EXHAUSTED`.

ИНЪЕКТИВНОСТЬ карты «треугольник -> проекция» обеспечивает сертификат вложения
проекции P0-4 (`INTERIOR_OVERLAP`, `FACE_POLYGON_NOT_SIMPLE`); здесь она не
перепроверяется, а вырожденные (нулевой площади) проекции пропускаются со счётом:
у них нет внутренности, которую можно было бы накрыть.

ОГРАНИЧЕНИЕ. Лежат на поверхности ВЕРШИНЫ меша. Ребро треугольника меша, идущее
через ребро источника под изломом, остаётся хордой; вставка вершин на пересечении
с рёбрами источника — отдельный срез.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

from ..contracts.metric import NearPlanarProjectionCertificateV1
from ..exact_sqrt_sum import SqrtSumV1
from ..numeric import LocalPoint3V1
from ..robust.grid import GridSpecV1, snap_value
from .admit import MaterializationOutcome
from .frames import MaterializationRefusal
from .lift import ENCLOSURE_BITS, sqrt_sum_binary64

LOCATIONS = "MATERIALIZE_SURFACE_LIFT_LOCATIONS"
CANDIDATES = "MATERIALIZE_SURFACE_LIFT_CANDIDATE_TRIANGLES"
PREDICATES = "MATERIALIZE_SURFACE_LIFT_PREDICATES"
ON_EDGE = "MATERIALIZE_SURFACE_LIFT_ON_EDGE_POINTS"
TRIANGLES = "MATERIALIZE_SURFACE_LIFT_TRIANGLES"
DEGENERATE = "MATERIALIZE_SURFACE_LIFT_DEGENERATE_PROJECTIONS"
CHART_SNAPPED = "MATERIALIZE_SURFACE_LIFT_CHART_VERTICES_SNAPPED"


def _down(value: Fraction) -> float:
    return math.nextafter(float(value), -math.inf)


def _up(value: Fraction) -> float:
    return math.nextafter(float(value), math.inf)


@dataclass(frozen=True, slots=True)
class LiftTriangleV1:
    """Проекция одного треугольника источника и 3D его привязанных вершин.

    `chart` — вершины в единицах решётки (`scale * (u, v)`), `corners` — точные
    привязанные позиции в 3D (до проекции), `twice_area` — ориентированная
    удвоенная площадь проекции (ненулевая), `box` — outward-округлённая
    рамка `(xmin, xmax, ymin, ymax)` для префильтра.
    """

    name: str
    chart: tuple
    corners: tuple
    twice_area: Fraction
    box: tuple


def _triangle(name, chart, corners) -> LiftTriangleV1 | None:
    (ax, ay), (bx, by), (cx, cy) = chart
    area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    if not area:
        return None
    xs = [item[0] for item in chart]
    ys = [item[1] for item in chart]
    return LiftTriangleV1(
        name=name,
        chart=tuple(chart),
        corners=tuple(corners),
        twice_area=area,
        box=(_down(min(xs)), _up(max(xs)), _down(min(ys)), _up(max(ys))),
    )


@dataclass(frozen=True, slots=True)
class SurfaceLiftV1:
    """Проекции треугольников источника владельца: всё, что нужно для подъёма."""

    triangles: tuple[LiftTriangleV1, ...]
    degenerate_projections: int
    scale: int
    #: Сколько вершин карты привязка к решётке сдвинула и на сколько (наибольшее
    #: смещение по оси, единицы решётки, дробью): запись, а не молчание.
    snapped_vertices: int = 0
    snap_residual: Fraction = Fraction(0)

    @staticmethod
    def from_triangles(
        items, scale: int, snapped_vertices: int = 0, snap_residual=Fraction(0)
    ) -> "SurfaceLiftV1":
        """`items` — `(имя, три точки карты в единицах решётки, три 3D-вершины)`."""

        built = []
        degenerate = 0
        for name, chart, corners in sorted(items, key=lambda item: item[0]):
            triangle = _triangle(
                name,
                tuple((Fraction(x), Fraction(y)) for x, y in chart),
                tuple(tuple(Fraction(axis) for axis in point) for point in corners),
            )
            if triangle is None:
                degenerate += 1
            else:
                built.append(triangle)
        return SurfaceLiftV1(
            tuple(built),
            degenerate,
            int(scale),
            int(snapped_vertices),
            Fraction(snap_residual),
        )

    def bind(self, budget) -> "BoundSurfaceLiftV1":
        """Тот же подъём, привязанный к бюджету транзакции и счётчикам."""

        return BoundSurfaceLiftV1(self, budget)


def _fraction(value) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def surface_lift_of(frame, snapshot, owner_patch_id, scale: int) -> SurfaceLiftV1:
    """Подъём домена по его метрике, сертификату искажения и треугольникам снапшота.

    Позиции берутся из `snapped_source_positions` сертификата — тех самых, от
    которых измерено искажение, — а проекции вершин из координат карты метрики.
    Домен без сертификата искажения к укладке на поверхность не допущен
    (`admit_domain` отказывает до работы), поэтому здесь он обязателен.
    """

    certificate = frame.planarity_certificate
    if type(certificate) is not NearPlanarProjectionCertificateV1:
        raise ValueError("a surface lift needs a near-planar certificate")
    sigma = certificate.width_distortion
    if sigma is None:
        raise ValueError("a surface lift needs the width-distortion certificate")
    position = {
        item.source_vertex_id: tuple(
            _fraction(axis)
            for axis in (item.position.x, item.position.y, item.position.z)
        )
        for item in sigma.snapped_source_positions
    }
    grid = GridSpecV1(scale=int(scale))
    exact = {
        item.source_vertex_id: (
            _fraction(item.domain_coordinate.x) * scale,
            _fraction(item.domain_coordinate.y) * scale,
        )
        for item in frame.exact_source_vertex_coordinates
    }
    chart = {
        vertex: tuple(Fraction(snap_value(axis / scale, grid)) for axis in point)
        for vertex, point in exact.items()
    }
    moved = [vertex for vertex in exact if chart[vertex] != exact[vertex]]
    residual = max(
        (
            abs(chart[vertex][axis] - exact[vertex][axis])
            for vertex in moved
            for axis in range(2)
        ),
        default=Fraction(0),
    )
    owner_faces = {
        face.face_id
        for face in snapshot.surface_ir.source_faces
        if face.patch_id == owner_patch_id
    }
    owned = sorted(
        (
            item
            for item in snapshot.surface_ir.surface_triangles
            if item.source_face_id in owner_faces
        ),
        key=lambda item: item.triangle_id.value,
    )
    _refuse_flipped(owned, exact, chart)
    return SurfaceLiftV1.from_triangles(
        (
            (
                item.triangle_id.value,
                tuple(chart[vertex] for vertex in item.vertex_ids),
                tuple(position[vertex] for vertex in item.vertex_ids),
            )
            for item in owned
        ),
        scale,
        len(moved),
        residual,
    )


def _twice_area(points) -> Fraction:
    (ax, ay), (bx, by), (cx, cy) = points
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def _refuse_flipped(owned, exact, chart) -> None:
    """Привязка не вправе сменить знак площади ни у одной проекции."""

    flipped = []
    for item in owned:
        before = _twice_area(tuple(exact[vertex] for vertex in item.vertex_ids))
        after = _twice_area(tuple(chart[vertex] for vertex in item.vertex_ids))
        if before and after and (before > 0) != (after > 0):
            flipped.append(item.triangle_id.value)
    if flipped:
        raise MaterializationRefusal(
            MaterializationOutcome.SURFACE_LIFT_CHART_SNAP_FLIPPED_TRIANGLE,
            f"{len(flipped)} source triangle projections change orientation "
            f"when the chart is snapped to the coverage lattice: "
            f"{', '.join(flipped[:4])}",
        )


def _edge_value(start, end, point) -> SqrtSumV1:
    """Ориентация `(start, end, point)`: `(end - start) x (point - start)` точно."""

    dx, dy = end[0] - start[0], end[1] - start[1]
    return (
        point[1].scaled(dx)
        - point[0].scaled(dy)
        + SqrtSumV1.rational(dy * start[0] - dx * start[1])
    )


class BoundSurfaceLiftV1:
    """Подъём, оплачивающий свои знаки бюджетом транзакции и ведущий счётчики."""

    def __init__(self, lift: SurfaceLiftV1, budget) -> None:
        self._lift = lift
        self._budget = budget
        self._tally = {
            LOCATIONS: 0,
            CANDIDATES: 0,
            PREDICATES: 0,
            ON_EDGE: 0,
        }

    def counters(self) -> tuple[tuple[str, int], ...]:
        return (
            *self._tally.items(),
            (TRIANGLES, len(self._lift.triangles)),
            (DEGENERATE, self._lift.degenerate_projections),
            (CHART_SNAPPED, self._lift.snapped_vertices),
        )

    def note(self) -> str:
        """Числа укладки для диагностики батча: сколько карты сдвинула решётка."""

        lift = self._lift
        return (
            f"chart_vertices_snapped={lift.snapped_vertices} "
            f"snap_residual_cells={float(lift.snap_residual):.6g} "
            f"source_triangles={len(lift.triangles)}"
        )

    def _window(self, point):
        (xlow, xhigh), (ylow, yhigh) = (
            point[0].enclosure(ENCLOSURE_BITS),
            point[1].enclosure(ENCLOSURE_BITS),
        )
        return _down(xlow), _up(xhigh), _down(ylow), _up(yhigh)

    def _inside(self, triangle: LiftTriangleV1, point):
        """Три значения ориентации, если точка в замкнутом треугольнике, иначе `None`."""

        direction = 1 if triangle.twice_area > 0 else -1
        values = []
        for index in range(3):
            value = _edge_value(
                triangle.chart[index], triangle.chart[(index + 1) % 3], point
            )
            self._tally[PREDICATES] += 1
            sign = value.sign(budget=self._budget)
            if sign * direction < 0:
                return None
            values.append(value)
        return values

    def locate(self, point):
        """`(треугольник, значения ориентации)` либо именованный отказ."""

        self._tally[LOCATIONS] += 1
        xlow, xhigh, ylow, yhigh = self._window(point)
        for triangle in self._lift.triangles:
            xmin, xmax, ymin, ymax = triangle.box
            if xhigh < xmin or xlow > xmax or yhigh < ymin or ylow > ymax:
                continue
            self._tally[CANDIDATES] += 1
            values = self._inside(triangle, point)
            if values is not None:
                if any(item.is_zero for item in values):
                    self._tally[ON_EDGE] += 1
                return triangle, values
        raise MaterializationRefusal(
            MaterializationOutcome.SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION,
            f"point≈({(xlow + xhigh) / 2:.9g}, {(ylow + yhigh) / 2:.9g}) in lattice "
            f"units lies outside the projection of all "
            f"{len(self._lift.triangles)} source triangles",
            self.counters(),
        )

    @staticmethod
    def lift_in(triangle: LiftTriangleV1, values) -> tuple[SqrtSumV1, SqrtSumV1, SqrtSumV1]:
        """Подъём в ЗАДАННОМ треугольнике: `(e1·A + e2·B + e0·C) / D` по осям."""

        first, second, third = triangle.corners
        weights = (values[1], values[2], values[0])
        divisor = triangle.twice_area
        return tuple(
            weights[0].scaled(first[axis] / divisor)
            + weights[1].scaled(second[axis] / divisor)
            + weights[2].scaled(third[axis] / divisor)
            for axis in range(3)
        )

    def lift_exact(self, point):
        triangle, values = self.locate(point)
        return self.lift_in(triangle, values)

    def lift(self, point) -> LocalPoint3V1:
        x, y, z = self.lift_exact(point)
        return LocalPoint3V1(
            sqrt_sum_binary64(x), sqrt_sum_binary64(y), sqrt_sum_binary64(z)
        )

    def values_in(self, triangle: LiftTriangleV1, point):
        """Три значения ориентации точки в ЗАДАННОМ треугольнике (без проверки)."""

        return [
            _edge_value(
                triangle.chart[index], triangle.chart[(index + 1) % 3], point
            )
            for index in range(3)
        ]
