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
сохранение ориентации у всех треугольников и ПРОСТАЯ граница после привязки
(ни одного нового пересечения, касания или схлопнутого ребра границы, точно, на
тех же предикатах, что и вложение P0-4) — именованный отказ
`SURFACE_LIFT_CHART_SNAP_BOUNDARY_NOT_SIMPLE` — и есть доказательство, что
привязанная триангуляция осталась вложением. Треугольник, чья проекция СХЛОПНУЛАСЬ
в ноль именно привязкой (до неё площадь была ненулевой), отказа не даёт: у проекции
без площади нет внутренности, и соседи накрывают всё остальное. Но он назван:
счётчик `..._COLLAPSED_BY_SNAPPING` и `collapsed_by_snapping` в диагностике;
`..._DEGENERATE_PROJECTIONS` — ВСЕ треугольники с нулевой площадью в привязанной
карте, то есть и точно вырожденные (их отвергает бюджет ширины ещё до укладки,
`cos²` = 0), и схлопнутые привязкой.

ВЫХОД ЗА ТРИАНГУЛЯЦИЮ НЕ БОЛЕЕ ДВУХ ЯЧЕЕК. Полигон покрытия строится из
ГЕОМЕТРИИ цепей, а не из вершин сетки: вершина полигона на хорде между концами
цепи — не вершина источника, и после привязки обеих карт к решётке она лежит
вне привязанной триангуляции. Замер на `building.004` (приведённый репер): три
такие точки у patch 1 (до 0.979 ячейки) и четыре у patch 4 (до 1.0186). Точка не
принадлежит ни одному замкнутому треугольнику, но принадлежит поверхности «в
пределах решётки»: она лифтится ПРОДОЛЖЕНИЕМ ближайшего треугольника
(барицентрические веса с отрицательным значением), если выходит за его границу не
более чем на `EXTRAPOLATION_CELL_BOUND` ячеек решётки карты. Граница — две ячейки:
два независимых округления (карты и вершины полигона) дают не больше корня из
двух. Это допуск, и он назван: записан в реестре допусков, точка считается в
`MATERIALIZE_SURFACE_LIFT_EXTRAPOLATED_POINTS` и называется в диагностике батча
(`extrapolated_points`, `max_outside_cells`); дальше границы — прежний именованный
отказ `SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION`. Сравнение расстояния с
границей точное: `e² <= B²·|ребро|²` на `SqrtSumV1` под бюджетом.

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
`sqrt_sum_binary64`). ШОВ МЕЖДУ ДОМЕНАМИ: вершины общего ребра источника и точки,
лежащие НА этом ребре, поднимаются в соседних доменах (у каждого своя плоскость и
своя привязка карты) в побитово одну точку отрезка источника; исполняемо
(`test_the_lift_on_a_shared_source_edge_is_the_same_from_both_neighbour_domains`).
Граница прямая: точки покрытия, которые привязка к решётке УВЕЛА с ребра, кладёт
СВОЙ домен (в пределах ячейки или продолжением), и между соседями у них возможна
щель порядка ячейки решётки; для вершин и точек на ребре её нет.

ОТКАЗЫ ИМЕНОВАНЫ. Точка вне проекции всей триангуляции — не «ближайший
треугольник» и не допуск, а `SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION`
с числами. Исчерпание бюджета — `ExactCanonicalizationWorkBudgetExhausted`,
которое материализатор называет `EXACT_WORK_BUDGET_EXHAUSTED`.

ИНЪЕКТИВНОСТЬ карты «треугольник -> проекция». Вложение проекции P0-4 проверяет
ПОЛИГОНЫ граней, а не треугольники, которыми пользуется укладка: у непланарного
квада проекция полигона может быть годной при ПЕРЕВЁРНУТОМ треугольнике, и квадрат
`cos²` знак прячет. Перевёрнутый треугольник накрыл бы соседа, и `locate` выбрал бы
первого по имени — две плоскости в одной области. Поэтому переворот отказывает
ЗАРАНЕЕ, на ступени метрики: сертификат σ считает `folded_triangle_count`, судья
отказывает `NEAR_PLANAR_SOURCE_TRIANGLE_FOLDED` (`_width_distortion`). Здесь
ориентация не перепроверяется, а вырожденные (нулевой площади) проекции
пропускаются со счётом: у них нет внутренности, которую можно было бы накрыть.

ОГРАНИЧЕНИЕ. Лежат на поверхности ВЕРШИНЫ меша. Ребро треугольника меша, идущее
через ребро источника под изломом, остаётся хордой; вставка вершин на пересечении
с рёбрами источника — законы `SOURCE_TRIANGLES_CLIPPED_V1` и `SOURCE_FACES_CLIPPED_V1`
(`materialize/clip`).

ГРАНЬ ИСТОЧНИКА. Каждый треугольник несёт `face` — id грани источника, из которой его выпустила
триангуляция хоста. Диагональ четырёхгранья — общее ребро двух треугольников ОДНОЙ грани, а не
ребро меша (у неё `physical_edge_id = None`): закон `SOURCE_FACES_CLIPPED_V1` режет только по
рёбрам, общим у треугольников РАЗНЫХ граней.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

from .._annulus_cut import chart_faces, source_vertex_of, strip_of
from .._embedding import (
    _NONE,
    _boundary_occurrences,
    _nonadjacent_pairs,
    _segment_relation2,
)
from ..contracts.metric import (
    DevelopableBandChartCertificateV1,
    NearPlanarProjectionCertificateV1,
    is_unfolded_certificate,
)
from ..exact_sqrt_sum import SqrtSumV1
from ..exact_sqrt_sum_fused import oriented_sum
from ..numeric import LocalPoint3V1
from ..robust.grid import GridSpecV1, snap_value
from .admit import MaterializationOutcome
from .frames import MaterializationRefusal
from .lift import ENCLOSURE_BITS, sqrt_sum_binary64
from .offset_normal import (
    OFFSET_NORMAL_LAW,
    blend,
    min_gap_cosine,
    opposition_note,
    opposition_totals,
    source_vertex_normals,
)

LOCATIONS = "MATERIALIZE_SURFACE_LIFT_LOCATIONS"
CANDIDATES = "MATERIALIZE_SURFACE_LIFT_CANDIDATE_TRIANGLES"
PREDICATES = "MATERIALIZE_SURFACE_LIFT_PREDICATES"
ON_EDGE = "MATERIALIZE_SURFACE_LIFT_ON_EDGE_POINTS"
TRIANGLES = "MATERIALIZE_SURFACE_LIFT_TRIANGLES"
#: ВСЕ треугольники с нулевой площадью в ПРИВЯЗАННОЙ карте: точно вырожденные и
#: схлопнутые привязкой вместе. Схлопнутые привязкой — отдельный счёт ниже.
DEGENERATE = "MATERIALIZE_SURFACE_LIFT_DEGENERATE_PROJECTIONS"
#: Из них: площадь до привязки была ненулевой, привязка к решётке обнулила её.
COLLAPSED = "MATERIALIZE_SURFACE_LIFT_COLLAPSED_BY_SNAPPING"
CHART_SNAPPED = "MATERIALIZE_SURFACE_LIFT_CHART_VERTICES_SNAPPED"
EXTRAPOLATED = "MATERIALIZE_SURFACE_LIFT_EXTRAPOLATED_POINTS"
#: Точек, у которых в допуске продолжения оказалось БОЛЬШЕ ОДНОГО треугольника.
AMBIGUOUS = "MATERIALIZE_SURFACE_LIFT_CONTINUATION_AMBIGUOUS_CANDIDATES"
#: Из них — где первые два расстояния РАВНЫ точно, и выбор решило имя треугольника.
EXACT_TIES = "MATERIALIZE_SURFACE_LIFT_CONTINUATION_EXACT_TIES"

#: Допуск продолжения ближайшего треугольника: на сколько ячеек решётки карты
#: точка покрытия вправе выйти за привязанную триангуляцию. ДВЕ ячейки: вершина
#: триангуляции и вершина полигона покрытия — два независимых округления одной
#: точки до решётки (каждое не больше половины ячейки по оси), поэтому они
#: расходятся не больше чем на ячейку по оси, то есть на `√2 < 2` ячеек; больше —
#: уже не шум привязки, а другая геометрия, и это отказ. ИЗМЕРЕНО на `building`
#: и `building.004` (патчи 4, 1, 106, 109): наибольший выход 1.0186 ячейки.
#: Запись реестра допусков `SURFACE_LIFT_EXTRAPOLATION_CELLS_V1`.
EXTRAPOLATION_CELL_BOUND = Fraction(2)


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
    #: Единичные нормали смещения трёх углов (binary64), либо пусто: нормаль смещения
    #: у домена развёртки своя на вершину, у плоского и near-planar — одна на домен.
    normals: tuple = ()
    #: Идентификатор грани источника, из которой выпущен треугольник; пусто — грань неизвестна, и
    #: треугольник сам себе грань (закон `SOURCE_FACES_CLIPPED_V1` его не склеивает ни с кем).
    face: str = ""


def _upper_root(value: Fraction) -> Fraction:
    """Рациональная верхняя граница `sqrt(value)` с точностью порядка `2^-24` относительной: `ceil(sqrt(v D^2)) / D`."""

    scale = 1 << 24
    scaled = value * scale * scale
    whole = -(-scaled.numerator // scaled.denominator)
    root = math.isqrt(whole)
    return Fraction(root if root * root == whole else root + 1, scale)


def _lipschitz_square(triangle: LiftTriangleV1) -> Fraction:
    """`sigma^2` аффинного подъёма треугольника: наибольшее собственное число `J^T J`, сверху (точно по рациональным)."""

    (ax, ay), (bx, by), (cx, cy) = triangle.chart
    e1x, e1y, e2x, e2y = bx - ax, by - ay, cx - ax, cy - ay
    det = e1x * e2y - e1y * e2x
    first = [q - r for q, r in zip(triangle.corners[1], triangle.corners[0])]
    second = [q - r for q, r in zip(triangle.corners[2], triangle.corners[0])]
    cross = sum(q * r for q, r in zip(first, second))
    gram = ((sum(q * q for q in first), cross), (cross, sum(q * q for q in second)))
    # `M^-1 = [[e2y, -e2x], [-e1y, e1x]] / det`; `J^T J = M^-T G M^-1` — симметричная 2x2 `[[p, q], [q, r]]`.
    rows = ((e2y, -e2x), (-e1y, e1x))
    entries = [
        sum(rows[k][i] * gram[k][m] * rows[m][j] for k in range(2) for m in range(2)) / (det * det)
        for i, j in ((0, 0), (0, 1), (1, 1))
    ]
    p, q, r = entries
    half = (p - r) / 2
    return (p + r) / 2 + _upper_root(half * half + q * q)


def _triangle(name, chart, corners, normals=(), face="") -> LiftTriangleV1 | None:
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
        normals=tuple(normals),
        face=face,
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
    #: Треугольники, чью ненулевую проекцию привязка к решётке обнулила (подмножество
    #: `degenerate_projections`; остальные там — точно вырожденные).
    collapsed_by_snapping: int = 0
    #: Допущенные противостояния нормали вершины нормали треугольника (`offset_normal.OppositionV1`): запись.
    opposition: tuple = ()

    @staticmethod
    def from_triangles(
        items,
        scale: int,
        snapped_vertices: int = 0,
        snap_residual=Fraction(0),
        collapsed_by_snapping: int = 0,
        opposition: tuple = (),
    ) -> "SurfaceLiftV1":
        """`items` — `(имя, три точки карты в единицах решётки, три 3D-вершины[, три нормали[, грань]])`."""

        built = []
        degenerate = 0
        for name, chart, corners, *rest in sorted(items, key=lambda item: item[0]):
            triangle = _triangle(
                name,
                tuple((Fraction(x), Fraction(y)) for x, y in chart),
                tuple(tuple(Fraction(axis) for axis in point) for point in corners),
                rest[0] if rest else (),
                rest[1] if len(rest) > 1 else "",
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
            int(collapsed_by_snapping),
            tuple(opposition),
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
    unfolded = is_unfolded_certificate(certificate)
    if unfolded:
        recorded = certificate.snapped_source_positions
    elif type(certificate) is NearPlanarProjectionCertificateV1:
        sigma = certificate.width_distortion
        if sigma is None:
            raise ValueError("a surface lift needs the width-distortion certificate")
        recorded = sigma.snapped_source_positions
    else:
        raise ValueError("a surface lift needs a near-planar or developable certificate")
    position = {
        item.source_vertex_id: tuple(
            _fraction(axis)
            for axis in (item.position.x, item.position.y, item.position.z)
        )
        for item in recorded
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
    # Полоса кладётся на треугольники НОСИТЕЛЯ из сертификата (карта покрывает только их), а граница, чью простоту
    # судит привязка, - граница носителя: грани носителя целые, поэтому их циклы и есть эта граница.
    support = (
        certificate.support_triangle_ids
        if type(certificate) is DevelopableBandChartCertificateV1
        else None
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
            and (support is None or item.triangle_id in support)
        ),
        key=lambda item: item.triangle_id.value,
    )
    faces = [
        face
        for face in snapshot.surface_ir.source_faces
        if face.patch_id == owner_patch_id
        and (support is None or any(item in support for item in face.triangle_ids))
    ]
    tolerated: list = []
    # Нормали смещения - по ВЕЕРУ вершины источника (целому), а не по половине веера у копии вершины разреза: обе копии
    # - одна точка поверхности и одна нормаль.
    normals = source_vertex_normals(owned, position, tolerated) if unfolded else None
    cut = getattr(certificate, "cut", None)
    if cut is not None:
        strip = strip_of(owned, cut)
        owned = sorted(strip.triangles, key=lambda item: item.triangle_id.value)
        faces = chart_faces(faces, strip)
        position = {**position, **{vertex: position[source_vertex_of(vertex)] for vertex in strip.right_vertices()}}
    _refuse_flipped(owned, exact, chart)
    _refuse_snapped_boundary(faces, exact, chart)
    return SurfaceLiftV1.from_triangles(
        (
            (
                item.triangle_id.value,
                tuple(chart[vertex] for vertex in item.vertex_ids),
                tuple(position[vertex] for vertex in item.vertex_ids),
                ()
                if normals is None
                else tuple(normals[source_vertex_of(vertex)] for vertex in item.vertex_ids),
                item.source_face_id.value,
            )
            for item in owned
        ),
        scale,
        len(moved),
        residual,
        sum(
            1
            for item in owned
            if _twice_area(tuple(exact[vertex] for vertex in item.vertex_ids))
            and not _twice_area(tuple(chart[vertex] for vertex in item.vertex_ids))
        ),
        tuple(tolerated),
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


def _refuse_snapped_boundary(faces, exact, chart) -> None:
    """Привязка не вправе создать новое пересечение границы или схлопнуть её ребро.

    Граница патча (полурёбра, которые встречаются один раз) до привязки простая:
    это доказано вложением P0-4. После привязки к решётке проверяется ТОЧНО, на тех
    же предикатах (`_segment_relation2`): пара непримыкающих рёбер, не
    касавшаяся друг друга до привязки, не вправе пересечься, наложиться или
    коснуться после неё; ненулевое ребро не вправе схлопнуться в точку.
    """

    edges = _boundary_occurrences(faces)
    problems = [
        f"{edge.start.value}->{edge.end.value} collapses to a point"
        for edge in edges
        if chart[edge.start] == chart[edge.end] and exact[edge.start] != exact[edge.end]
    ]
    for left, right in _nonadjacent_pairs(edges):
        ends = (left.start, left.end, right.start, right.end)
        if any(chart[a] == chart[b] for a, b in ((ends[0], ends[1]), (ends[2], ends[3]))):
            continue
        if _segment_relation2(*(chart[item] for item in ends)) == _NONE:
            continue
        if _segment_relation2(*(exact[item] for item in ends)) == _NONE:
            problems.append(
                f"{left.start.value}->{left.end.value} meets "
                f"{right.start.value}->{right.end.value}"
            )
    if problems:
        raise MaterializationRefusal(
            MaterializationOutcome.SURFACE_LIFT_CHART_SNAP_BOUNDARY_NOT_SIMPLE,
            f"{len(problems)} boundary relations are created by snapping the "
            f"chart to the coverage lattice: {'; '.join(problems[:4])}",
        )


def _edge_value(start, end, point) -> SqrtSumV1:
    """Ориентация `(start, end, point)`: `(end - start) x (point - start)` точно."""

    dx, dy = end[0] - start[0], end[1] - start[1]
    if dx.denominator == dy.denominator == start[0].denominator == start[1].denominator == 1:
        # Целые концы (решётка карты): шаг и сдвиг целые, и значение собирается одной нормировкой дробей.
        return oriented_sum(
            point[0], point[1], dx.numerator, dy.numerator, dy.numerator * start[0].numerator - dx.numerator * start[1].numerator
        )
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
            EXTRAPOLATED: 0,
            AMBIGUOUS: 0,
            EXACT_TIES: 0,
        }
        self._max_outside = 0.0
        self._normal_by_position: dict = {}
        self._stretch: dict = {}

    def counters(self) -> tuple[tuple[str, int], ...]:
        return (
            *self._tally.items(),
            (TRIANGLES, len(self._lift.triangles)),
            (DEGENERATE, self._lift.degenerate_projections),
            (COLLAPSED, self._lift.collapsed_by_snapping),
            (CHART_SNAPPED, self._lift.snapped_vertices),
            *opposition_totals(self._lift.opposition),
        )

    def note(self) -> str:
        """Числа укладки для диагностики батча: сколько карты сдвинула решётка."""

        lift = self._lift
        return (
            f"chart_vertices_snapped={lift.snapped_vertices} "
            f"snap_residual_cells={float(lift.snap_residual):.6g} "
            f"source_triangles={len(lift.triangles)} "
            f"collapsed_by_snapping={lift.collapsed_by_snapping} "
            f"extrapolated_points={self._tally[EXTRAPOLATED]} "
            f"max_outside_cells={self._max_outside:.6g} "
            f"continuation_ambiguous_points={self._tally[AMBIGUOUS]} "
            f"continuation_exact_ties={self._tally[EXACT_TIES]}"
        )

    def opposition_note(self) -> str:
        """Допущенные противостояния нормали вершины нормали треугольника (глубина в допуске); пусто, если их нет."""

        return opposition_note(self._lift.opposition)

    def gap_note(self) -> str:
        """Наименьший `n_v . n_T` подъёма для диагностики; пусто, если нормалей вершин нет."""

        found = min_gap_cosine(self._lift.triangles)
        if found is None:
            return ""
        return (
            f"offset_min_gap_cosine={found[0]:.9g} (source triangle {found[1]}, corner "
            f"{found[2]}): the offset lifts the decal above a source triangle by "
            "offset * cosine; recorded, not thresholded"
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
        nearest = self._nearest(point)
        if nearest is not None:
            self._tally[EXTRAPOLATED] += 1
            return nearest
        raise MaterializationRefusal(
            MaterializationOutcome.SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION,
            f"point≈({(xlow + xhigh) / 2:.9g}, {(ylow + yhigh) / 2:.9g}) in lattice "
            f"units lies outside the projection of all "
            f"{len(self._lift.triangles)} source triangles",
            self.counters(),
        )

    def _bounded_values(self, triangle: LiftTriangleV1, point):
        """`(значения ориентации, квадрат выхода)`, если выход в допуске, иначе `None`.

        Квадрат выхода — наибольший из `e²/|ребро|²` по рёбрам, которые точка
        нарушает: ТОЧНОЕ `SqrtSumV1`, а не оценка. Им сравниваются кандидаты.
        """

        direction = 1 if triangle.twice_area > 0 else -1
        values = []
        outside = None
        for index in range(3):
            start, end = triangle.chart[index], triangle.chart[(index + 1) % 3]
            value = _edge_value(start, end, point)
            self._tally[PREDICATES] += 1
            values.append(value)
            if value.sign(budget=self._budget) * direction >= 0:
                continue
            # Расстояние до прямой ребра `|e| / |ребро|` не больше допуска B
            # тогда и только тогда, когда `e² <= B²·|ребро|²` — точно, без корня.
            length_squared = (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2
            square = value * value
            gap = SqrtSumV1.rational(EXTRAPOLATION_CELL_BOUND**2 * length_squared) - square
            self._tally[PREDICATES] += 1
            if gap.sign(budget=self._budget) < 0:
                return None
            distance = square.scaled(Fraction(1) / length_squared)
            if outside is not None:
                self._tally[PREDICATES] += 1
            if outside is None or (distance - outside).sign(budget=self._budget) > 0:
                outside = distance
        return values, outside

    def _nearest(self, point):
        """Ближайший треугольник, который точка превышает не более допуска, либо `None`.

        «Ближайший» — по ТОЧНОМУ квадрату выхода (`_bounded_values`), а не по
        приближённому расстоянию: выбор между кандидатами меняет ответ, и он не
        вправе зависеть от округления float. Равенство разрешается каноническим
        порядком имён (треугольники отсортированы по имени, побеждает меньшее);
        оба случая, «кандидатов больше одного» и «точное равенство», считаются и
        называются в диагностике батча.
        """

        xlow, xhigh, ylow, yhigh = self._window(point)
        margin = float(EXTRAPOLATION_CELL_BOUND)
        best = None
        admissible = 0
        tied = False
        for triangle in self._lift.triangles:
            xmin, xmax, ymin, ymax = triangle.box
            if (
                xhigh < xmin - margin
                or xlow > xmax + margin
                or yhigh < ymin - margin
                or ylow > ymax + margin
            ):
                continue
            self._tally[CANDIDATES] += 1
            found = self._bounded_values(triangle, point)
            if found is None:
                continue
            admissible += 1
            values, outside = found
            if best is None:
                best = (outside, triangle, values)
                continue
            self._tally[PREDICATES] += 1
            order = (outside - best[0]).sign(budget=self._budget)
            if order < 0:
                best = (outside, triangle, values)
                tied = False
            elif order == 0:
                tied = True
        if best is None:
            return None
        if admissible > 1:
            self._tally[AMBIGUOUS] += 1
            self._tally[EXACT_TIES] += int(tied)
        _, high = best[0].enclosure(ENCLOSURE_BITS)
        self._max_outside = max(self._max_outside, math.sqrt(float(high)))
        return best[1], best[2]

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
        return self.lift_named(point)[0]

    def lift_named(self, point):
        """`(позиция, (имя найденного треугольника, нормаль смещения | None))`.

        Нахождение ОДНО на точку, счётчики не растут. Нормаль смещения есть у домена
        развёртки (своя на вершину) и нужна закону топологии: смещённая по разным
        нормалям грань уже не плоская, как бы ни лежали её вершины до смещения.
        """

        triangle, values = self.locate(point)
        return self.lift_known(triangle, values)

    def lift_known(self, triangle: LiftTriangleV1, values):
        """`lift_named` для точки, чей ЗАДАННЫЙ треугольник и три значения ориентации уже известны.

        Закон `SOURCE_TRIANGLES_CLIPPED_V1` доказывает вхождение куска грани в свой треугольник
        сам (`clip`), поэтому нахождение (`locate`) заново он не просит и счётчиков подъёма не
        трогает: значения те же, что дало бы `locate` для замкнутого треугольника.
        """

        x, y, z = self.lift_in(triangle, values)
        lifted = LocalPoint3V1(
            sqrt_sum_binary64(x), sqrt_sum_binary64(y), sqrt_sum_binary64(z)
        )
        normal = None
        if triangle.normals:
            divisor = float(triangle.twice_area)
            weights = tuple(
                sqrt_sum_binary64(values[index]) / divisor for index in (1, 2, 0)
            )
            normal = blend(weights, triangle.normals)
            self._normal_by_position[(lifted.x, lifted.y, lifted.z)] = normal
        return lifted, (triangle.name, normal)

    def replay_lifted(self, lifted: dict) -> None:
        """Нормали смещения вершин, подъём которых взят из памяти резки (`clip_memo`), встают на свои позиции.

        `lift_known` пишет нормаль по позиции подъёма; вершина, поднятая не здесь, а записью памяти, оставила бы
        `offset_normals` без неё. Порядок записи — порядок рождения вершин (тот же, что в `lifted`).
        """

        for position, (_name, normal) in lifted.values():
            if normal is not None:
                self._normal_by_position[(position.x, position.y, position.z)] = normal

    @property
    def has_offset_normals(self) -> bool:
        return bool(self._normal_by_position)

    @property
    def triangles(self) -> tuple[LiftTriangleV1, ...]:
        """Треугольники подъёма по имени (проекции треугольников источника владельца)."""

        return self._lift.triangles

    def stretch_square(self, triangle: LiftTriangleV1) -> Fraction:
        """ВЕРХНЯЯ граница квадрата длины источника (метры 3D ядра) на ячейку карты В ЭТОМ треугольнике: для нанометров.

        Единица карты — не всегда `1 / scale` (приведённый репер домена несёт свой масштаб и бывает косым), поэтому
        берётся из самого треугольника: его подъём аффинен с матрицей `J` (3x2), и смещение на карте длиной `d` ячеек в
        3D не длиннее `d * sigma`, где `sigma^2` — наибольшее собственное число `J^T J` (корень — рациональной верхней
        границей, `_upper_root`). Число локально: у крутого треугольника оно велико, и общая для домена оценка
        описывала бы его, а не вершину, о которой запись. Это ОЦЕНКА для чисел записи: ответов она не решает.
        """

        found = self._stretch.get(triangle.name)
        if found is None:
            found = self._stretch[triangle.name] = _lipschitz_square(triangle)
        return found

    def window(self, point):
        """Outward-округлённая рамка `(xmin, xmax, ymin, ymax)` точки: фильтр, ответа не меняет."""

        return self._window(point)

    def line_value(self, triangle: LiftTriangleV1, index: int, point) -> SqrtSumV1:
        """Ориентация точки относительно `index`-го ребра треугольника: значение, а не знак."""

        return _edge_value(
            triangle.chart[index],
            triangle.chart[(index + 1) % len(triangle.chart)],
            point,
        )

    def rebind_position(self, old, new) -> None:
        """Нормаль смещения вершины, чья позиция стала `new`, — та же, что была у `old`.

        Закон `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` переставляет вершину `src:` в позицию
        хоста; нормали ищутся по позиции подъёма, и без этого вызова вершина осталась бы без
        нормали смещения.
        """

        normal = self._normal_by_position.get((old.x, old.y, old.z))
        if normal is not None:
            self._normal_by_position[(new.x, new.y, new.z)] = normal

    def offset_normals(self, vertices) -> tuple:
        """`((vert_key, нормаль), ...)` вершин батча по закону `OFFSET_NORMAL_LAW`."""

        return tuple(
            (
                item.vert_key.value,
                self._normal_by_position[(item.position.x, item.position.y, item.position.z)],
            )
            for item in sorted(vertices, key=lambda entry: entry.vert_key.value)
        )

    offset_normal_law = OFFSET_NORMAL_LAW

    def values_in(self, triangle: LiftTriangleV1, point):
        """Три значения ориентации точки в ЗАДАННОМ треугольнике (без проверки)."""

        return [
            _edge_value(
                triangle.chart[index], triangle.chart[(index + 1) % 3], point
            )
            for index in range(3)
        ]
