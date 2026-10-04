"""Шарнирная развёртка источника: топология диска, дерево, предложение, решётка.

Модуль внутренний, как `_embedding` и `_width_distortion`: он не владеет допуском
(растяжение судит `_stretch` по допуску запроса `DecalRequestV1.developable_stretch_budget`),
ничего не чинит и называет каждый отказ. Он делает три вещи.

1. ТОПОЛОГИЯ ВЛАДЕЛЬЦА. Смежность треугольников выводится по парам сторон САМИХ
   треугольников владельца (сторона `i` — пара `(v[i], v[i+1])`, нумерация ядра):
   сторона с одним носителем — граница домена, с двумя — шов, с тремя —
   отказ. Таблица `SurfaceAdjacencyIRV1` построителю метрики не передаётся (хост
   вызывает одну функцию ядра и отдаёт грани и треугольники), а срез под
   `REQUEST_SCOPED_PATCH_SET` не отличает границу меша от границы запроса — но
   развёртке различать это незачем: ей нужен диск владельца, а не меш.
   Диск доказывается: связность, согласованная ориентация, веер каждой вершины —
   одна цепь либо один цикл, эйлерова характеристика `V - E + F = 1`. Кольцо
   (характеристика 0, две граничные петли) — `PERIODIC_CUT_REQUIRED`.

2. ПРЕДЛОЖЕНИЕ. Корень — треугольник с наименьшим именем; обход в ширину по
   номерам сторон; шарнир кладёт третью вершину ребёнка по другую сторону
   общей стороны. Арифметика — binary64 с ФИКСИРОВАННЫМ порядком операций над
   рациональными квадратами длин (`BINARY64_HINGE_V1`): никакого `hypot`, только
   `sqrt` (в IEEE-754 он округляется верно), поэтому предложение воспроизводимо
   побитово на любой платформе. Вершина получает ОДНУ позицию при первом
   достижении (сварка): веер недевелопабельной вершины не замыкается, и невязка
   уходит в растяжение треугольников — её судит `_stretch`, а не этот модуль.

3. РЕШЁТКА. Позиции карты привязываются к целым узлам решётки `1/S'` (половина —
   вверх, как везде в ядре); смещение каждой вершины считается ТОЧНО.

Предложение — не власть: любая карта, чьё растяжение в бюджете, а граница
проста, годна. Власть — точный сертификат растяжения.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from fractions import Fraction

from .contracts.surface_adjacency import canonical_fan_order
from .outcomes import NamedOutcome


def refusal(outcome: NamedOutcome, detail: str):
    """Именованный отказ метрики. Ленивый импорт: `planar_metric` импортирует нас."""

    from .planar_metric import PlanarMetricAdmissionError

    return PlanarMetricAdmissionError(outcome, detail)


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


def squared_distance(first, second) -> Fraction:
    """Квадрат расстояния между двумя точными 3D-позициями."""

    delta = _sub(first, second)
    return _dot(delta, delta)


@dataclass(frozen=True, slots=True)
class UnfoldTopologyV1:
    """Диск треугольников владельца: стороны, веера вершин, граница."""

    triangles: tuple
    by_id: dict
    #: `(triangle_id, ordinal) -> (triangle_id, ordinal) | None` (None — граница).
    opposite: dict
    #: `vertex_id -> (упорядоченные треугольники веера, замкнут ли веер)`.
    fans: dict
    #: Граничные стороны `(triangle_id, ordinal)`, по имени.
    boundary_sides: tuple
    boundary_loop_count: int
    euler_characteristic: int


def _first_names(items, limit: int = 3) -> str:
    return ", ".join(str(item) for item in items[:limit])


def _refuse_degenerate(triangles, positions) -> None:
    degenerate = [
        item.triangle_id.value
        for item in triangles
        if not any(
            _cross(
                _sub(positions[item.vertex_ids[1]], positions[item.vertex_ids[0]]),
                _sub(positions[item.vertex_ids[2]], positions[item.vertex_ids[0]]),
            )
        )
    ]
    if degenerate:
        raise refusal(
            NamedOutcome.DEVELOPABLE_SOURCE_TRIANGLE_DEGENERATE,
            f"{len(degenerate)} owner triangles are degenerate after the source "
            f"snap, so no hinge passes through them (first: "
            f"{_first_names(degenerate)})",
        )


def _side_records(triangles) -> dict:
    return {
        (item.triangle_id, ordinal): (
            item.vertex_ids[ordinal],
            item.vertex_ids[(ordinal + 1) % 3],
        )
        for item in triangles
        for ordinal in range(3)
    }


def _opposites(records) -> dict:
    """Пары сторон по концам; больше двух носителей и сонаправленные — отказ."""

    grouped: dict = {}
    for key, (first, second) in records.items():
        grouped.setdefault(frozenset((first, second)), []).append(key)
    result: dict = {}
    for keys in grouped.values():
        keys.sort(key=lambda item: (item[0].value, item[1]))
        if len(keys) > 2:
            raise refusal(
                NamedOutcome.DEVELOPABLE_ADJACENCY_UNAVAILABLE,
                f"a side pair is carried by {len(keys)} owner triangles "
                f"(first: {keys[0][0].value}:{keys[0][1]}), so the dual graph is "
                "not a surface",
            )
        if len(keys) == 1:
            result[keys[0]] = None
            continue
        left, right = keys
        if records[left][0] != records[right][1]:
            raise refusal(
                NamedOutcome.DEVELOPABLE_ADJACENCY_UNAVAILABLE,
                f"sides {left[0].value}:{left[1]} and {right[0].value}:{right[1]} "
                "traverse their shared ends in the same direction, so the owner "
                "triangles are not consistently oriented",
            )
        result[left] = right
        result[right] = left
    return result


def _vertex_fans(triangles, records, opposite) -> dict:
    incident: dict = {}
    for item in triangles:
        for vertex_id in item.vertex_ids:
            incident.setdefault(vertex_id, []).append(item.triangle_id)
    fans: dict = {}
    for vertex_id in sorted(incident, key=lambda item: item.value):
        links = {triangle_id: set() for triangle_id in incident[vertex_id]}
        for triangle_id in incident[vertex_id]:
            for ordinal in range(3):
                key = (triangle_id, ordinal)
                if vertex_id not in records[key]:
                    continue
                other = opposite[key]
                if other is not None:
                    links[triangle_id].add(other[0])
        ordered = canonical_fan_order(links)
        if ordered is None:
            raise refusal(
                NamedOutcome.DEVELOPABLE_ADJACENCY_UNAVAILABLE,
                f"the triangles around vertex {vertex_id.value} do not form one "
                "chain or one cycle (a pinched or non-manifold vertex)",
            )
        fans[vertex_id] = ordered
    return fans


def _boundary_loop_count(boundary_sides, records) -> int:
    """Число замкнутых граничных петель: у каждой вершины ровно одна выходящая сторона."""

    following: dict = {}
    for key in boundary_sides:
        start, end = records[key]
        if start in following:
            raise refusal(
                NamedOutcome.DEVELOPABLE_ADJACENCY_UNAVAILABLE,
                f"vertex {start.value} starts two boundary sides: the boundary "
                "of the owner triangles is not a set of simple loops",
            )
        following[start] = end
    seen: set = set()
    loops = 0
    for start in sorted(following, key=lambda item: item.value):
        if start in seen:
            continue
        loops += 1
        vertex = start
        while vertex not in seen:
            seen.add(vertex)
            vertex = following.get(vertex)
            if vertex is None:
                raise refusal(
                    NamedOutcome.DEVELOPABLE_ADJACENCY_UNAVAILABLE,
                    "a boundary side leads to a vertex with no outgoing "
                    "boundary side: the boundary is not closed",
                )
    return loops


def _connected(triangles, opposite) -> int:
    """Число достигнутых из первого треугольника по двойственному графу."""

    start = triangles[0].triangle_id
    seen = {start}
    queue = deque([start])
    while queue:
        current = queue.popleft()
        for ordinal in range(3):
            other = opposite[(current, ordinal)]
            if other is not None and other[0] not in seen:
                seen.add(other[0])
                queue.append(other[0])
    return len(seen)


def _raw_topology(owner_triangles, positions):
    """Топология треугольников владельца без суждения о диске: `(топология, числа V E F chi петли)`."""

    triangles = tuple(
        sorted(owner_triangles, key=lambda item: item.triangle_id.value)
    )
    if not triangles:
        raise refusal(
            NamedOutcome.NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE,
            "the owner Patch has no surface triangle to unfold",
        )
    _refuse_degenerate(triangles, positions)
    records = _side_records(triangles)
    opposite = _opposites(records)
    fans = _vertex_fans(triangles, records, opposite)
    reached = _connected(triangles, opposite)
    if reached != len(triangles):
        raise refusal(
            NamedOutcome.DEVELOPABLE_SUPPORT_NOT_A_DISK,
            f"the owner triangles are not connected: {reached} of "
            f"{len(triangles)} are reachable from the first by shared sides",
        )
    boundary = tuple(
        sorted(
            (key for key, other in opposite.items() if other is None),
            key=lambda item: (item[0].value, item[1]),
        )
    )
    edge_count = len({frozenset(pair) for pair in records.values()})
    euler = len(fans) - edge_count + len(triangles)
    loops = _boundary_loop_count(boundary, records)
    numbers = (
        f"V={len(fans)} E={edge_count} F={len(triangles)} "
        f"chi={euler} boundary_loops={loops}"
    )
    return (
        UnfoldTopologyV1(
            triangles=triangles,
            by_id={item.triangle_id: item for item in triangles},
            opposite=opposite,
            fans=fans,
            boundary_sides=boundary,
            boundary_loop_count=loops,
            euler_characteristic=euler,
        ),
        numbers,
    )


def owner_topology(owner_triangles, positions) -> UnfoldTopologyV1:
    """Диск треугольников владельца либо именованный отказ.

    `positions` — привязанные точные позиции вершин источника (до проекции).
    """

    topology, numbers = _raw_topology(owner_triangles, positions)
    if topology.euler_characteristic == 0 and topology.boundary_loop_count == 2:
        raise refusal(
            NamedOutcome.PERIODIC_CUT_REQUIRED,
            f"the owner triangles form a ring ({numbers}): a chart needs a cut "
            "and a holonomy, which this stage does not carry",
        )
    if topology.euler_characteristic != 1 or topology.boundary_loop_count != 1:
        raise refusal(
            NamedOutcome.DEVELOPABLE_SUPPORT_NOT_A_DISK,
            f"the owner triangles are not a disk ({numbers})",
        )
    return topology


def annulus_topology(owner_triangles, positions) -> UnfoldTopologyV1 | None:
    """Топология кольца (`chi = 0`, две граничные петли) либо `None`: диск и прочее решает `owner_topology`."""

    topology, _numbers = _raw_topology(owner_triangles, positions)
    if topology.euler_characteristic == 0 and topology.boundary_loop_count == 2:
        return topology
    return None


def _third_vertex(first, second, near_first, near_second, base, *, right: bool):
    """Третья вершина по рациональным квадратам длин: порядок операций ФИКСИРОВАН.

    `first`, `second` — уже положенные концы общей стороны (binary64), `near_*` —
    квадраты расстояний от них до новой вершины, `base` — квадрат длины стороны.
    Проекция новой вершины на сторону `along / sqrt(base)`, высота — корень из
    точного рационального остатка. `right` — по правую руку от `first -> second`.
    """

    along = (near_first - near_second + base) / 2
    height_squared = near_first - along * along / base
    shift = float(along) / math.sqrt(float(base))
    height = math.sqrt(float(height_squared))
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    length = math.sqrt(dx * dx + dy * dy)
    ex = dx / length
    ey = dy / length
    nx, ny = (ey, -ex) if right else (-ey, ex)
    return (
        first[0] + ex * shift + nx * height,
        first[1] + ey * shift + ny * height,
    )


@dataclass(frozen=True, slots=True)
class UnfoldProposalV1:
    """Предложение карты: корень, порядок обхода и позиции binary64 в метрах."""

    root_triangle_id: object
    order: tuple
    coordinates: dict
    #: Сколько раз вершина уже имела позицию, когда её достигал другой треугольник.
    welded_corner_count: int


def hinge_proposal(topology: UnfoldTopologyV1, positions) -> UnfoldProposalV1:
    """Дерево обхода и позиции карты по закону `CANONICAL_BFS_SMALLEST_TRIANGLE_ID_V1`."""

    root = topology.triangles[0]
    first, second, third = root.vertex_ids
    coordinates: dict = {}
    coordinates[first] = (0.0, 0.0)
    coordinates[second] = (
        math.sqrt(float(squared_distance(positions[first], positions[second]))),
        0.0,
    )
    coordinates[third] = _third_vertex(
        coordinates[first],
        coordinates[second],
        squared_distance(positions[first], positions[third]),
        squared_distance(positions[second], positions[third]),
        squared_distance(positions[first], positions[second]),
        right=False,
    )
    order = [root.triangle_id]
    seen = {root.triangle_id}
    queue = deque([root.triangle_id])
    welded = 0
    while queue:
        parent = topology.by_id[queue.popleft()]
        for ordinal in range(3):
            other = topology.opposite[(parent.triangle_id, ordinal)]
            if other is None or other[0] in seen:
                continue
            seen.add(other[0])
            order.append(other[0])
            queue.append(other[0])
            child = topology.by_id[other[0]]
            shared_first = parent.vertex_ids[ordinal]
            shared_second = parent.vertex_ids[(ordinal + 1) % 3]
            new = child.vertex_ids[(other[1] + 2) % 3]
            if new in coordinates:
                welded += 1
                continue
            coordinates[new] = _third_vertex(
                coordinates[shared_first],
                coordinates[shared_second],
                squared_distance(positions[shared_first], positions[new]),
                squared_distance(positions[shared_second], positions[new]),
                squared_distance(positions[shared_first], positions[shared_second]),
                right=True,
            )
    return UnfoldProposalV1(
        root_triangle_id=root.triangle_id,
        order=tuple(order),
        coordinates=coordinates,
        welded_corner_count=welded,
    )


def exact_metres(coordinates) -> dict:
    """Позиции предложения как ТОЧНЫЕ дроби (binary64 читается без потерь)."""

    return {
        vertex: (Fraction(point[0]), Fraction(point[1]))
        for vertex, point in coordinates.items()
    }


def _snap(value: Fraction, scale: int) -> int:
    """Узел `floor(value * scale + 1/2)`: то же правило, что у `snap_value`."""

    scaled = value * scale
    return (2 * scaled.numerator + scaled.denominator) // (2 * scaled.denominator)


@dataclass(frozen=True, slots=True)
class SnappedChartV1:
    """Целые узлы карты, число сдвинутых вершин и наибольшее смещение по оси."""

    nodes: dict
    snapped_vertex_count: int
    snap_residual: Fraction


def snap_to_chart_lattice(exact: dict, scale: int) -> SnappedChartV1:
    """Привязка позиций (метры, дроби) к узлам `1/scale`; смещение — точно, в узлах."""

    nodes: dict = {}
    moved = 0
    residual = Fraction(0)
    for vertex, point in exact.items():
        node = (_snap(point[0], scale), _snap(point[1], scale))
        nodes[vertex] = node
        shifts = [abs(point[axis] * scale - node[axis]) for axis in range(2)]
        if any(shifts):
            moved += 1
            residual = max(residual, *shifts)
    return SnappedChartV1(nodes=nodes, snapped_vertex_count=moved, snap_residual=residual)


def chart_metres(nodes: dict, scale: int) -> dict:
    """Узлы карты как точные позиции в метрах: `node / scale`."""

    return {
        vertex: (Fraction(node[0], scale), Fraction(node[1], scale))
        for vertex, node in nodes.items()
    }
