"""Полосовая карта: развёртка носителя и запас до стены досягаемости.

Модуль внутренний и ничем не владеет: носитель выбирает `_band_support`, развёртку носителя (шарнир, ARAP, решётка,
суд растяжения) делает `_developable.build_developable_chart` на ТРЕУГОЛЬНИКАХ носителя, а здесь они складываются в
`DevelopableBandChartCertificateV1`. Новое — четыре вещи.

1. ГРАНИЦА носителя по сторонам. Обход границы носителя (внутренность слева) даёт петлю; каждая сторона получает роль
   по снапшоту: сторона выбранной цепи — `RIM`, сторона прочей цепи границы патча — `ORIGINAL_BOUNDARY`, ребро без
   цепи — `REACH_WALL` (там носитель обрезан). Сторона, чью роль не назначить однозначно, — именованный отказ, а не
   догадка.
2. ЗАПАС — ВЛАСТЬ. Наименьший квадрат расстояния НА КАРТЕ между ободом и стеной досягаемости (точная дробь, метры). Он
   не меньше `cap^2`: фронт ширины `alpha <= cap` растёт внутри `alpha`-окрестности обода на карте (евклидова карта
   развёртки, стены фронт не расширяют), поэтому до стены не доходит и усечённый носитель отвечает так же, как отвечал
   бы целый патч. Меньше — именованный отказ `CHART_REACH_SHORT_OF_CAP`, а не усечённое покрытие. Носитель, равный
   всему патчу, стены досягаемости не имеет: у диска это не полоса (отказ целого патча остаётся), у кольца — разрез по
   всему патчу без запаса, и тогда `alpha` досягаемостью не ограничена.
3. ИДЕНТИЧНОСТЬ. Метрика и сертификат полосы зависят от выбора цепей и досягаемости, поэтому их имена несут и то и другое
   (иначе две полосы одного домена были бы одним объектом).
4. КОЛЬЦО. Носитель вокруг замкнутой цепи — кольцо, карта у кольца одна — развёртка диска, и кольцо режется по пути
   рёбер от вершины обода до дальней петли (`_annulus_cut`): вершины пути раздваиваются, грани по правую сторону пути
   переименовываются, края разреза становятся сторонами `CUT_LEFT` / `CUT_RIGHT` границы (стенами очереди, как и стена
   досягаемости), а в сертификат ложится `BandCutV1`: голономия склейки и два судимых числа (шов и отклонение от
   биссектрисы). Любое из них за допуском — именованный отказ, а не карта с щелью.

Цена: расстояния считаются точно, но кандидаты отбираются плавающими числами с запасом `2^-20` (классификация и записанный
запас всегда точные).
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import fields
from fractions import Fraction
from hashlib import sha256

from ._annulus_cut import (
    SEAM_RESIDUAL_BOUND,
    bisector_numbers,
    cut_strip,
    is_right_copy,
    right_copy,
    ring_cut_path,
    seam_fit,
    seam_prefix,
    source_vertex_of,
)
from ._band_support import band_support, point_segment_distance_squared
from ._developable import DevelopableChartV1, build_developable_chart, _memory_key
from ._unfold import annulus_topology, owner_topology, refusal
from .contracts.metric import (
    BandBoundaryRoleV1,
    BandBoundarySideV1,
    BandCutCornerV1,
    BandCutV1,
    BandSupportLawV1,
    BandTightenedV1,
    DevelopableBandChartCertificateV1,
    ExactRationalV1,
    MAX_CHART_REACH_CAP,
    PlanarityAdmissionLawV1,
    chart_reach_cap_is_lawful,
)
from .ids import PlanarityCertificateId
from .outcomes import NamedOutcome

#: Разряды запаса плавающего отбора пар отрезков (`2^-20`, около `1e-6`): точный минимум берётся только среди
#: отобранных, поэтому запас не допуск, а число разрядов.
_SLACK_BITS = 20


def _rational(value: Fraction | int) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def _stable(kind: str, *parts: object) -> str:
    payload = "\x1f".join((kind, *(str(item) for item in parts)))
    return f"{kind}:{sha256(payload.encode('utf-8')).hexdigest()[:24]}"


def _band_parts(selected, cap) -> tuple:
    """Часть имени, различающая полосы одного домена: досягаемость и выбранные цепи."""

    cap = Fraction(cap)
    return (f"{cap.numerator}/{cap.denominator}", *sorted(item.value for item in selected))


def band_certificate_id(source_revision, patch_domain_id, selected, cap) -> str:
    return _stable("developable-band-chart", source_revision.value, patch_domain_id.value, *_band_parts(selected, cap))


def band_metric_id(source_revision, patch_domain_id, selected, cap, required_ids) -> str:
    return _stable(
        "reference-metric-band",
        source_revision.value,
        patch_domain_id.value,
        *_band_parts(selected, cap),
        required_ids[0].value,
        *(item.value for item in required_ids),
    )


def _cut_side(start, end, cut_edges, edge, name):
    """Роль стороны разреза (`CUT_LEFT` / `CUT_RIGHT`) либо `None`, если сторона не лежит на пути разреза."""

    if frozenset((source_vertex_of(start), source_vertex_of(end))) not in cut_edges:
        return None
    if is_right_copy(start) != is_right_copy(end):
        raise refusal(
            NamedOutcome.DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED,
            f"the cut side {name} joins a left copy of a vertex to a right one",
        )
    if edge is None:
        raise refusal(
            NamedOutcome.DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED,
            f"the cut side {name} has no physical edge: the cut runs along edges of the source between faces",
        )
    return BandBoundaryRoleV1.CUT_RIGHT if is_right_copy(start) else BandBoundaryRoleV1.CUT_LEFT


def _boundary_cycle(topology, band, cutting=None) -> tuple:
    """Стороны границы носителя в порядке обхода, роли назначены; начало — сторона с наименьшим именем ребра.

    `cutting` (`RingStripV1`) — разрез кольца: имена вершин карты у копий пути не совпадают с именами источника, поэтому
    пары `ChainUse` ищутся по вершинам источника, а стороны на пути разреза получают роли края разреза.
    """

    selected = band.selected_chain_use_ids
    cut_edges = (
        frozenset()
        if cutting is None
        else frozenset(frozenset(pair) for pair in zip(cutting.path, cutting.path[1:]))
    )
    use_of: dict = {}
    ambiguous: set = set()
    for pair, use_id, edge in band.boundary_uses:
        if pair in use_of:
            ambiguous.add(pair)
        use_of[pair] = (use_id, edge)
    sides = []
    for triangle_id, ordinal in topology.boundary_sides:
        triangle = topology.by_id[triangle_id]
        start = triangle.vertex_ids[ordinal]
        end = triangle.vertex_ids[(ordinal + 1) % 3]
        edge = triangle.physical_edge_ids[ordinal]
        name = f"{start.value}->{end.value}"
        role = _cut_side(start, end, cut_edges, edge, name)
        if role is not None:
            sides.append(BandBoundarySideV1(role, edge, start, end, None))
            continue
        pair = (source_vertex_of(start), source_vertex_of(end))
        if pair in ambiguous:
            raise refusal(
                NamedOutcome.DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED,
                f"the support boundary side {name} belongs to more than one ChainUse",
            )
        known = use_of.get(pair)
        if known is not None:
            use_id, chain_edge = known
            if edge is not None and edge != chain_edge:
                raise refusal(
                    NamedOutcome.DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED,
                    f"the support boundary side {name} lies on {edge.value}, its ChainUse on {chain_edge.value}",
                )
            role = BandBoundaryRoleV1.RIM if use_id in selected else BandBoundaryRoleV1.ORIGINAL_BOUNDARY
            sides.append(BandBoundarySideV1(role, chain_edge, start, end, use_id))
            continue
        if pair[::-1] in use_of:
            raise refusal(
                NamedOutcome.DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED,
                f"the support boundary side {name} runs against its ChainUse: the owner interior is not on its left",
            )
        if edge is None:
            raise refusal(
                NamedOutcome.DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED,
                f"the support boundary side {name} has no physical edge and no ChainUse: it cannot be a reach wall",
            )
        sides.append(BandBoundarySideV1(BandBoundaryRoleV1.REACH_WALL, edge, start, end, None))
    following = {item.start_vertex_id: item for item in sides}
    first = min(sides, key=lambda item: (item.physical_edge_id.value, item.start_vertex_id.value))
    cycle = [first]
    while following[cycle[-1].end_vertex_id] is not first:
        cycle.append(following[cycle[-1].end_vertex_id])
    if len(cycle) != len(sides):
        raise refusal(
            NamedOutcome.DEVELOPABLE_SUPPORT_NOT_A_DISK,
            "the support boundary does not close into one loop of sides",
        )
    return tuple(cycle)


def _float_point(point) -> tuple:
    return (float(point[0]), float(point[1]))


def _segment_gap_squared(first, second, exact: bool):
    """Квадрат расстояния между непересекающимися отрезками: наименьший из четырёх «конец -- отрезок»."""

    if exact:
        pairs = (
            (first[0], second),
            (first[1], second),
            (second[0], first),
            (second[1], first),
        )
        return min(point_segment_distance_squared(point, *segment) for point, segment in pairs)
    pairs = (
        (first[0], second),
        (first[1], second),
        (second[0], first),
        (second[1], first),
    )
    return min(_float_point_segment(point, segment) for point, segment in pairs)


def _float_point_segment(point, segment) -> float:
    (px, py), ((sx, sy), (ex, ey)) = point, segment
    dx, dy = ex - sx, ey - sy
    length = dx * dx + dy * dy
    along = (px - sx) * dx + (py - sy) * dy
    if length == 0.0 or along <= 0.0:
        return (px - sx) ** 2 + (py - sy) ** 2
    if along >= length:
        return (px - ex) ** 2 + (py - ey) ** 2
    return (px - sx) ** 2 + (py - sy) ** 2 - along * along / length


def rim_to_wall_gap_squared(nodes: dict, sides, chart_scale: int) -> Fraction | None:
    """Наименьший квадрат расстояния на карте между ободом и стеной (метры^2), точно; `None` — нечего сравнивать."""

    def segments(role):
        return [
            (nodes[item.start_vertex_id], nodes[item.end_vertex_id])
            for item in sides
            if item.role is role
        ]

    rim = segments(BandBoundaryRoleV1.RIM)
    wall = segments(BandBoundaryRoleV1.REACH_WALL)
    if not rim or not wall:
        return None
    rim_float = [(_float_point(a), _float_point(b)) for a, b in rim]
    wall_float = [(_float_point(a), _float_point(b)) for a, b in wall]
    approximate = [
        (_segment_gap_squared(r, w, False), ri, wi)
        for ri, r in enumerate(rim_float)
        for wi, w in enumerate(wall_float)
    ]
    best = min(item[0] for item in approximate)
    limit = best * (1.0 + 2.0 ** -_SLACK_BITS) + 2.0 ** -40
    exact = min(
        _segment_gap_squared(rim[ri], wall[wi], True)
        for value, ri, wi in approximate
        if value <= limit
    )
    return Fraction(exact) / (chart_scale * chart_scale)


def _certificate(unfold, band, support, reach, sides, margin, source_revision, patch_domain_id, cut=None):
    values = {item.name: getattr(unfold, item.name) for item in fields(unfold)}
    values.update(
        certificate_id=PlanarityCertificateId(
            band_certificate_id(source_revision, patch_domain_id, band.selected_chain_use_ids, band.reach_cap)
        ),
        admission_law=PlanarityAdmissionLawV1.DEVELOPABLE_BAND_CHART_V1,
        selected_chain_use_ids=band.selected_chain_use_ids,
        reach_cap=_rational(band.reach_cap),
        support_law=BandSupportLawV1.FACES_WITHIN_EUCLIDEAN_REACH_V1,
        support_reach=_rational(reach),
        support_triangle_ids=frozenset(item.triangle_id for item in support.triangles),
        excluded_triangle_count=support.excluded_count,
        first_excluded_triangle_id=support.first_excluded,
        strip_boundary=sides,
        chart_reach_margin_squared=None if margin is None else _rational(margin),
        cut=cut,
        tightened=(
            None
            if band.requested_reach_cap is None
            else BandTightenedV1(_rational(band.requested_reach_cap), band.tightened_after)
        ),
    )
    return DevelopableBandChartCertificateV1(**values)


#: Память построителя в ПРОЦЕССЕ, как у `_developable`: полоса — чистая функция входов (носитель, развёртка, запас),
#: а собирается она несколько раз (построение метрики и проверки снапшота).
BAND_MEMORY_ENTRIES = 8
_band_memory: OrderedDict = OrderedDict()


def clear_band_chart_memory() -> None:
    _band_memory.clear()


def build_band_chart(
    *,
    source_revision,
    patch_domain_id,
    snapped,
    owner_triangles,
    required_ids,
    source_scale,
    previous_refusals,
    budget,
    declared_straight_chains,
    band,
) -> DevelopableChartV1 | None:
    """Карта и сертификат полосы вокруг выбранных цепей, `None` (носитель — весь патч) либо именованный отказ.

    `previous_refusals` кончается отказом развёртки целого патча. `snapped` и `owner_triangles` — патч ЦЕЛИКОМ: носитель
    выбирает этот модуль.
    """

    key = (
        _memory_key(
            source_revision,
            patch_domain_id,
            snapped,
            owner_triangles,
            required_ids,
            source_scale,
            previous_refusals,
            budget,
            declared_straight_chains,
        ),
        band,
    )
    if key in _band_memory:
        _band_memory.move_to_end(key)
        kept = _band_memory[key]
    else:
        kept = _band_memory[key] = _build(
            source_revision=source_revision,
            patch_domain_id=patch_domain_id,
            snapped=snapped,
            owner_triangles=owner_triangles,
            source_scale=source_scale,
            previous_refusals=previous_refusals,
            budget=budget,
            declared_straight_chains=declared_straight_chains,
            band=band,
        )
        while len(_band_memory) > BAND_MEMORY_ENTRIES:
            _band_memory.popitem(last=False)
    if kept is None:
        return None
    return DevelopableChartV1(kept.certificate, dict(kept.nodes), kept.chart_scale)


def _build(
    *,
    source_revision,
    patch_domain_id,
    snapped,
    owner_triangles,
    source_scale,
    previous_refusals,
    budget,
    declared_straight_chains,
    band,
):
    if not chart_reach_cap_is_lawful(Fraction(band.reach_cap)):
        raise ValueError(f"the chart reach cap {band.reach_cap} must lie in (0, {MAX_CHART_REACH_CAP}] m")
    reach = (1 + Fraction(budget)) * Fraction(band.reach_cap)
    support = band_support(owner_triangles, snapped, band.rim_edges, reach)
    in_support = {
        vertex: snapped[vertex]
        for vertex in {vertex for item in support.triangles for vertex in item.vertex_ids}
    }
    triangles, charted, cutting, straight = support.triangles, in_support, None, declared_straight_chains
    from .planar_metric import PlanarMetricAdmissionError

    try:
        ring = annulus_topology(support.triangles, in_support)
    except PlanarMetricAdmissionError:
        # Носитель - весь патч, и его топологию целый патч уже назвал: полоса ничего не меняет.
        if support.excluded_count:
            raise
        ring = None
    if not support.excluded_count and ring is None:
        return None
    if ring is not None:
        # Кольцо: карта одна - развёртка диска, и кольцо режется по пути от вершины обода до дальней петли.
        cutting = cut_strip(ring, ring_cut_path(ring, in_support, band.rim_edges))
        triangles = cutting.triangles
        charted = {**in_support, **{right_copy(vertex): in_support[vertex] for vertex in cutting.path}}
        # Цепь, объявленная прямой, через вершину разреза карта не проводит: её вершины стоят на обеих сторонах разреза.
        straight = tuple(chain for chain in declared_straight_chains if not set(chain) & set(cutting.path))
    vertices = sorted(charted, key=lambda item: item.value)
    chart = build_developable_chart(
        source_revision=source_revision,
        patch_domain_id=patch_domain_id,
        snapped=charted,
        owner_triangles=triangles,
        required_ids=tuple(vertices),
        source_scale=source_scale,
        previous_refusals=previous_refusals,
        budget=budget,
        declared_straight_chains=straight,
    )
    topology = owner_topology(triangles, charted)
    sides = _boundary_cycle(topology, band, cutting)
    margin = rim_to_wall_gap_squared(chart.nodes, sides, chart.chart_scale)
    cap = Fraction(band.reach_cap)
    if support.excluded_count and (margin is None or margin < cap * cap):
        shown = "none" if margin is None else f"{float(margin) ** 0.5:.6g} m"
        raise refusal(
            NamedOutcome.CHART_REACH_SHORT_OF_CAP,
            f"the chart distance from the rim to the reach wall is {shown}, shorter than the reach cap "
            f"{float(cap):.6g} m (support reach {float(reach):.6g} m, {support.excluded_count} triangles "
            "beyond the support): the strip is curled or the support is too thin",
        )
    cut = None if cutting is None else _cut_record(cutting, chart, sides, cap if support.excluded_count else None)
    certificate = _certificate(
        chart.certificate, band, support, reach, sides, margin, source_revision, patch_domain_id, cut
    )
    return DevelopableChartV1(certificate, chart.nodes, chart.chart_scale)


def cut_measurements(path, nodes: dict, chart_scale: int, sides, cap=None) -> tuple:
    """Измерения разреза по карте и границе полосы: `(cos, sin, tx, ty, остаток^2, верхняя граница sin^2 d, судья, вершин шва)`.

    Одна функция на построитель и валидатор: запись сверяется с тем же измерением, а не с его копией. `cap` - досягаемость
    у полосы со стеной (шов судится по префиксу пути в её пределах, `_annulus_cut.seam_prefix`), `None` - у целого кольца.
    Углы вершины разреза читаются по сторонам границы полосы (внутренность слева): копия `a` начинает сторону, которая
    не край разреза, копия `b` кончает такую сторону.
    """

    from .reference.evaluation_binding_noise import NOISE_DIRECTION_SINE_BOUND

    apex = path[0]
    uncut = [item for item in sides if item.role not in (BandBoundaryRoleV1.CUT_LEFT, BandBoundaryRoleV1.CUT_RIGHT)]
    out = [item for item in uncut if source_vertex_of(item.start_vertex_id) == apex]
    back = [item for item in uncut if source_vertex_of(item.end_vertex_id) == apex]
    if len(out) != 1 or len(back) != 1:
        raise refusal(
            NamedOutcome.PERIODIC_CUT_PATH_UNAVAILABLE,
            f"the cut vertex {apex.value} has {len(out)} outgoing and {len(back)} incoming sides of the strip boundary",
        )
    first = path[1]
    corner_a, corner_b = out[0].start_vertex_id, back[0].end_vertex_id
    measured = seam_prefix(nodes, path, chart_scale, cap)
    cosine, sine, shift_x, shift_y, residual = seam_fit(nodes, measured, chart_scale)
    deviation, passes = bisector_numbers(
        nodes,
        corner_a,
        out[0].end_vertex_id,
        right_copy(first) if is_right_copy(corner_a) else first,
        corner_b,
        back[0].start_vertex_id,
        right_copy(first) if is_right_copy(corner_b) else first,
        NOISE_DIRECTION_SINE_BOUND,
    )
    return cosine, sine, shift_x, shift_y, residual, deviation, passes, len(measured)


def _cut_record(cutting, chart, sides, cap) -> BandCutV1:
    """Голономия и два судимых числа разреза кольца; число за допуском - именованный отказ."""

    from .reference.evaluation_binding_noise import NOISE_DIRECTION_SINE_BOUND

    apex = cutting.path[0]
    cosine, sine, shift_x, shift_y, residual, deviation, passes, counted = cut_measurements(
        cutting.path, chart.nodes, chart.chart_scale, sides, cap
    )
    if residual > SEAM_RESIDUAL_BOUND * SEAM_RESIDUAL_BOUND:
        raise refusal(
            NamedOutcome.PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED,
            f"after the best rigid motion the two copies of the cut path differ by {float(residual) ** 0.5:.6g} m, "
            f"more than the seam bound {float(SEAM_RESIDUAL_BOUND):.6g} m (cut vertex {apex.value}, "
            f"{counted} of {len(cutting.path)} path vertices within the reach)",
        )
    if not passes:
        raise refusal(
            NamedOutcome.PERIODIC_CUT_BISECTOR_DEVIATION_EXCEEDED,
            f"the cut leaves {apex.value} at sin^2 d <= {float(deviation):.6g} from the bisector of the glued corner, "
            f"beyond the direction bound {float(NOISE_DIRECTION_SINE_BOUND) ** 2:.6g}: the wedge left uncovered at "
            "the seam is no longer noise",
        )
    return BandCutV1(
        cut_vertex_id=apex,
        path_vertex_ids=cutting.path,
        right_corners=frozenset(BandCutCornerV1(triangle, vertex) for triangle, vertex in cutting.right_corners),
        rotation_cosine=_rational(cosine),
        rotation_sine=_rational(sine),
        translation_x=_rational(shift_x),
        translation_y=_rational(shift_y),
        seam_residual_squared=_rational(residual),
        bisector_deviation_sine_squared=_rational(deviation),
        seam_vertex_count=counted,
    )
