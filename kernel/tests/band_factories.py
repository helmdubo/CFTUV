"""Входы тестов полосовой карты: поверхности-сетки в 3D и домен с названным ободом.

Поверхность задаётся сеткой `cols x rows` клеток с функцией положения `position(i, j)`; плоская МЕТКА вершины - её
индекс `(i, j)` (топология и обход петель ядру нужны от плоского вложения, а честные 3D-позиции подставляются поверх,
как в `developable_route.developable_domain`). Обод - несколько цепей по ОДНОМУ ребру (так режет цепи хост на изломах),
каждая вершина обода входит в две цепи.

Фикстуры: `arch_intrados` (полуцилиндр свода, закрытый куполом апсиды: целиком не развёртывается, а полоса у переднего
края - цилиндр, развёртка изометрична), `dome_rim` (купол над кругом, обод - часть его края или весь край: кольцо) и
`column_top` / `frustum_ring` (замкнутые колонна и усечённый конус: носитель вокруг верхнего кольца - кольцо, и оно режется
по образующей: сдвиг у цилиндра, поворот у конуса).
"""

from __future__ import annotations

import dataclasses
import math
from fractions import Fraction

import cftuv_envelope as kernel
from cftuv_envelope.chart_band import chart_band_request
from cftuv_envelope.contracts.metric import (
    CurvatureLadderPolicyV1,
    GridSnappingLawV1,
    NearPlanarFramePolicyV1,
    NearPlanarLiftLawV1,
    ExactRationalV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.declared_chains import declared_straight_chain_vertices
from cftuv_envelope.ids import ChainUseId

from materialize_factories import with_affine_metric
from reference_factories import straight_snapshot

LADDER = CurvatureLadderPolicyV1.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1


def grid_surface(cols: int, rows: int, position):
    """`(точки, циклы граней, метки)` сетки клеток; вершина `g{i}_{j}`, метка `(i, j)`."""

    points = {
        f"g{i}_{j}": tuple(float(a) for a in position(i, j))
        for i in range(cols + 1)
        for j in range(rows + 1)
    }
    labels = {f"g{i}_{j}": (float(i), float(j)) for i in range(cols + 1) for j in range(rows + 1)}
    cycles = [
        [f"g{i}_{j}", f"g{i + 1}_{j}", f"g{i + 1}_{j + 1}", f"g{i}_{j + 1}"]
        for i in range(cols)
        for j in range(rows)
    ]
    return points, cycles, labels


def arch_intrados(cols=12, barrel_rows=12, apse_rows=6, radius=1.0, step=0.1, apse_degrees=60.0):
    """Свод: полуцилиндр радиуса `radius` вдоль `y` (строки через `step`), затем апсида - сфера до `apse_degrees`.

    Передняя дуга - строка `j = 0` (`cols` рёбер). Цилиндрическая часть изометрична плоскости, апсида имеет
    гауссову кривизну `1 / radius^2`: целиком свод в бюджет растяжения не укладывается. Четвёртый элемент -
    маршрут обода, пятый - длинные цепи боковых линий пяты (вдоль образующей, затем по апсиде), с которыми полоса
    тоже обязана жить: носитель режет их на полпути.
    """

    def position(i, j):
        theta = math.pi * i / cols
        if j <= barrel_rows:
            return radius * math.cos(theta), step * j, radius * math.sin(theta)
        phi = math.radians(apse_degrees) * (j - barrel_rows) / apse_rows
        ring = radius * math.cos(phi)
        return ring * math.cos(theta), step * barrel_rows + radius * math.sin(phi), ring * math.sin(theta)

    rows = barrel_rows + apse_rows
    points, cycles, labels = grid_surface(cols, rows, position)
    route = tuple(f"g{i}_0" for i in range(cols + 1))
    right = (
        tuple(f"g{cols}_{j}" for j in range(barrel_rows + 1)),
        tuple(f"g{cols}_{j}" for j in range(barrel_rows, rows + 1)),
    )
    left = (
        tuple(f"g0_{j}" for j in range(rows, barrel_rows - 1, -1)),
        tuple(f"g0_{j}" for j in range(barrel_rows, -1, -1)),
    )
    return points, cycles, labels, route, (*right, *left)


def dome_rim(sides=16, rings=4, radius=1.0, arc=5):
    """Купол над кругом (центр и кольца), обод - `arc` рёбер внешнего кольца подряд.

    Метка вершины - её проекция на плоскость (вложение без перекрытий). Купол недевелопабелен, поэтому полоса вдоль
    дуги обода - единственная карта, которую можно построить.
    """

    points = {"c": (0.0, 0.0, radius)}
    labels = {"c": (0.0, 0.0)}
    cycles = []
    for ring in range(1, rings + 1):
        phi = (math.pi / 2) * ring / rings
        rho, height = radius * math.sin(phi), radius * math.cos(phi)
        for k in range(sides):
            theta = 2.0 * math.pi * k / sides
            points[f"p{ring}_{k}"] = (rho * math.cos(theta), rho * math.sin(theta), height)
            labels[f"p{ring}_{k}"] = (rho * math.cos(theta), rho * math.sin(theta))
    for k in range(sides):
        cycles.append(["c", f"p1_{k}", f"p1_{(k + 1) % sides}"])
    # Все циклы граней идут против часовой стрелки в плане (а метки вершин - плоское вложение без перекрытий).
    for ring in range(1, rings):
        for k in range(sides):
            cycles.append(
                [
                    f"p{ring}_{k}",
                    f"p{ring + 1}_{k}",
                    f"p{ring + 1}_{(k + 1) % sides}",
                    f"p{ring}_{(k + 1) % sides}",
                ]
            )
    route = tuple(f"p{rings}_{k}" for k in range(arc + 1))
    return points, cycles, labels, route, ()


def column_top(segments=8, rows=10, radius=1.0, step=0.1, radius_top=None):
    """Замкнутая колонна: кольцо квадов, обод - верхнее кольцо целиком.

    Метка вершины - полярная: нижнее кольцо снаружи, верхнее внутри (плоское вложение кольца). Целый патч - кольцо
    (`PERIODIC_CUT_REQUIRED`), носитель у верхнего кольца - тоже кольцо: полоса режет его по образующей от вершины обода до
    стены досягаемости (цилиндр: сдвиг). `radius_top` - радиус верхнего кольца (конус, `frustum_ring`): рёбра колец и
    высота остаются, радиус меняется линейно по строкам.
    """

    points, labels, cycles = {}, {}, []
    top = radius if radius_top is None else radius_top
    for j in range(rows + 1):
        ring_radius = radius + (top - radius) * j / rows
        for k in range(segments):
            theta = 2.0 * math.pi * k / segments
            points[f"c{j}_{k}"] = (ring_radius * math.cos(theta), ring_radius * math.sin(theta), step * j)
            outer = 2.0 + 0.5 * (rows - j) / rows
            labels[f"c{j}_{k}"] = (outer * math.cos(theta), outer * math.sin(theta))
    for j in range(rows):
        for k in range(segments):
            nxt = (k + 1) % segments
            cycles.append([f"c{j}_{k}", f"c{j}_{nxt}", f"c{j + 1}_{nxt}", f"c{j + 1}_{k}"])
    # Верхнее кольцо - внутренняя (дырная) петля плана: внутренность владельца слева, значит обход по часовой стрелке.
    route = tuple(f"c{rows}_{(-k) % segments}" for k in range(segments + 1))
    return points, cycles, labels, route, ()


def frustum_ring(segments=16, rows=10, radius=1.0, radius_top=0.6, step=0.1):
    """Усечённый конус: замкнутая колонна с радиусом, линейно меняющимся по строкам (развёртка - кольцевой сектор)."""

    return column_top(segments=segments, rows=rows, radius=radius, step=step, radius_top=radius_top)


def dome_ring(sides=16, rings=8, radius=1.0):
    """Купол над кругом, обод - ВЕСЬ внешний край (замкнутая цепь из рёбер): носитель вокруг него - кольцо у экватора."""

    points, cycles, labels, _route, extra = dome_rim(sides=sides, rings=rings, radius=radius, arc=sides)
    route = tuple(f"p{rings}_{k % sides}" for k in range(sides + 1))
    return points, cycles, labels, route, extra


def band_domain(
    parts,
    *,
    alpha="0.25",
    reach_cap=None,
    ladder=LADDER,
    developable_stretch_budget=None,
    select=None,
    requested_reach_cap=None,
    tightened_after=None,
):
    """`(снапшот, запрос, названный вход полосы)` домена; метрика строится лестницей с полосой, если вход назван.

    `parts = (точки, циклы, метки, маршрут, дополнительные цепи)`; маршрут режется на цепи по одному ребру, дополнительные
    цепи (боковые линии) остаются длинными и в выбор запроса не входят. `select` - номера цепей обода, выбранных запросом
    (`None` - все). `reach_cap=None` - полосы нет (метрика целого патча либо её отказ). `requested_reach_cap` и
    `tightened_after` - суженная карта: полоса строится под `reach_cap`, а запрос несёт запрошенную.
    """

    points, cycles, labels, route, extra = parts
    face_cycles = tuple(tuple(labels[name] for name in cycle) for cycle in cycles)
    rim = tuple(
        {"name": f"rim{index}", "points": (labels[first], labels[second])}
        for index, (first, second) in enumerate(zip(route, route[1:]))
    )
    side = tuple(
        {"name": f"side{index}", "points": tuple(labels[name] for name in chain)}
        for index, chain in enumerate(extra)
    )
    snapshot, request = straight_snapshot(faces=face_cycles, source_routes=rim + side, alpha=alpha)
    request = dataclasses.replace(
        request,
        selected_chain_use_ids=frozenset(
            ChainUseId(f"use:{item['name']}:use")
            for index, item in enumerate(rim)
            if select is None or index in select
        ),
    )
    order = []
    for cycle in face_cycles:
        for point in cycle:
            if point not in order:
                order.append(point)
    by_label = {label: name for name, label in labels.items()}
    snapshot = dataclasses.replace(
        snapshot,
        source_vertices=frozenset(
            dataclasses.replace(
                item,
                position=kernel.LocalPoint3V1(*points[by_label[order[int(item.vertex_id.value[1:])]]]),
            )
            for item in snapshot.source_vertices
        ),
    )
    if reach_cap is not None:
        cap = Fraction(reach_cap if requested_reach_cap is None else requested_reach_cap)
        request = dataclasses.replace(request, chart_reach_cap=ExactRationalV1(cap.numerator, cap.denominator))
    domain = next(iter(snapshot.patch_domains))
    band = (
        None
        if reach_cap is None
        else chart_band_request(
            snapshot.physical_chains,
            snapshot.chain_uses,
            request.selected_chain_use_ids,
            domain.patch_domain_id,
            reach_cap,
            requested_reach_cap,
            tightened_after,
        )
    )
    snapshot = with_affine_metric(
        snapshot,
        grid_policy=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
        planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1,
        near_planar_frame_policy=NearPlanarFramePolicyV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1,
        curvature_ladder=ladder,
        developable_stretch_budget=developable_stretch_budget,
        chart_band=band,
        declared_straight_chains=declared_straight_chain_vertices(
            snapshot.physical_chains, snapshot.chain_uses, domain.patch_domain_id
        ),
    )
    return snapshot, request, band
