"""Носитель полосовой карты: грани источника в пределах досягаемости от обода.

Модуль внутренний и ничем не владеет: закон выбора назван в контракте
(`BandSupportLawV1.FACES_WITHIN_EUCLIDEAN_REACH_V1`), авторитет — не носитель, а запас в сертификате карты
(`DevelopableBandChartCertificateV1.chart_reach_margin_squared`, считает `_band_chart`). Носитель — ПРЕДЛОЖЕНИЕ, и
любой детерминированный выбор годится, пока запас на карте подтверждает его.

ЗАКОН. Грань входит, если хоть одна её вершина лежит не дальше `reach` от какого-нибудь ребра обода (евклидово
расстояние в 3D по привязанным позициям, точная дробь). Грани целиком, а не треугольники: полигон грани, примыкающей
к ободу, обязан иметь координаты всех вершин (угловые отношения хоста читают цикл грани). Носитель — связная (по общим
рёбрам) компонента принятых граней, в которой лежат все рёбра обода; обод, распавшийся на несколько компонент, —
именованный отказ. Вершина с веером, не складывающимся в одну цепь (две принятые грани касаются в вершине, а грани
между ними отброшены), лечится ДОБАВЛЕНИЕМ граней её веера (замыкание по защемлению), пока защемлений нет.

ЦЕНА. Расстояние до обода считается точно, но сперва отсекается по плавающей оболочке отрезка: `D` растёт на
относительные `2^-30`, и вершина вне расширенной оболочки точно далека. Классификация «близко/далеко» всегда точная.
"""

from __future__ import annotations

import sys
from collections import deque
from dataclasses import dataclass
from fractions import Fraction

from .contracts.surface_adjacency import canonical_fan_order
from .outcomes import NamedOutcome
from ._unfold import refusal

#: Предел раундов замыкания по защемлению: каждый раунд добавляет грани, а граней конечное число; предел нужен лишь
#: затем, чтобы входная порча (не многообразие) закончилась отказом топологии, а не бесконечным циклом.
PINCH_CLOSURE_ROUNDS = 64

#: Разряды запаса плавающей оболочки: `2^-30` относительно (около `1e-9`). Оболочка отсекает только то, что заведомо
#: дальше `reach`; запас - не допуск (классификация "близко/далеко" всегда точная), поэтому он целое число разрядов.
_SLACK_BITS = 30


@dataclass(frozen=True, slots=True)
class BandSupportV1:
    """Треугольники носителя (по имени) и сколько патча в него не вошло."""

    triangles: tuple
    excluded_count: int
    first_excluded: object | None
    pinch_closure_rounds: int


def _sub(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _dot(left, right) -> Fraction:
    return sum((a * b for a, b in zip(left, right, strict=True)), Fraction(0))


def point_segment_distance_squared(point, start, end) -> Fraction:
    """Квадрат расстояния от точки до ЗАМКНУТОГО отрезка: точная дробь, любая размерность."""

    offset = _sub(point, start)
    direction = _sub(end, start)
    length = _dot(direction, direction)
    along = _dot(offset, direction)
    if not length or along <= 0:
        return _dot(offset, offset)
    if along >= length:
        beyond = _sub(point, end)
        return _dot(beyond, beyond)
    return _dot(offset, offset) - along * along / length


def _envelopes(segments, reach: Fraction):
    """Плавающие оболочки отрезков, расширенные на `reach` с запасом: `(lo, hi)` по осям."""

    margin = float(reach) * (1.0 + 2.0 ** -_SLACK_BITS) + sys.float_info.min
    result = []
    for start, end in segments:
        low = tuple(min(float(a), float(b)) - margin for a, b in zip(start, end))
        high = tuple(max(float(a), float(b)) + margin for a, b in zip(start, end))
        result.append((low, high))
    return result


def _near_vertices(snapped, segments, reach: Fraction) -> set:
    """Вершины не дальше `reach` от какого-нибудь отрезка обода."""

    reach_squared = reach * reach
    envelopes = _envelopes(segments, reach)
    near = set()
    for vertex, position in snapped.items():
        approx = tuple(float(axis) for axis in position)
        for (low, high), (start, end) in zip(envelopes, segments):
            if any(c < lo or c > hi for c, lo, hi in zip(approx, low, high)):
                continue
            if point_segment_distance_squared(position, start, end) <= reach_squared:
                near.add(vertex)
                break
    return near


def _pair_map(triangles) -> dict:
    """`{frozenset({a, b}): [id треугольников]}` по сторонам."""

    pairs: dict = {}
    for item in triangles:
        for ordinal in range(3):
            key = frozenset((item.vertex_ids[ordinal], item.vertex_ids[(ordinal + 1) % 3]))
            pairs.setdefault(key, []).append(item.triangle_id)
    return pairs


def _components(included: set, by_id, pairs) -> list:
    """Компоненты множества треугольников по общим сторонам: списки id, по имени первого."""

    seen: set = set()
    result = []
    for start in sorted(included, key=lambda item: item.value):
        if start in seen:
            continue
        seen.add(start)
        order = [start]
        queue = deque([start])
        while queue:
            current = by_id[queue.popleft()]
            for ordinal in range(3):
                key = frozenset((current.vertex_ids[ordinal], current.vertex_ids[(ordinal + 1) % 3]))
                for other in pairs[key]:
                    if other in included and other not in seen:
                        seen.add(other)
                        order.append(other)
                        queue.append(other)
        result.append(order)
    return result


def _pinched_vertices(included: set, by_id, pairs) -> list:
    """Вершины принятых треугольников, чей веер среди принятых не одна цепь и не один цикл."""

    incident: dict = {}
    for triangle_id in included:
        for vertex in by_id[triangle_id].vertex_ids:
            incident.setdefault(vertex, []).append(triangle_id)
    pinched = []
    for vertex, triangles in incident.items():
        links = {triangle_id: set() for triangle_id in triangles}
        for triangle_id in triangles:
            item = by_id[triangle_id]
            for ordinal in range(3):
                first, second = item.vertex_ids[ordinal], item.vertex_ids[(ordinal + 1) % 3]
                if vertex not in (first, second):
                    continue
                for other in pairs[frozenset((first, second))]:
                    if other != triangle_id and other in links:
                        links[triangle_id].add(other)
        if canonical_fan_order(links) is None:
            pinched.append(vertex)
    return sorted(pinched, key=lambda item: item.value)


def band_support(owner_triangles, snapped, rim_edges, reach: Fraction) -> BandSupportV1:
    """Носитель полосы вокруг рёбер обода `rim_edges` (пары вершин) в пределах `reach`; либо именованный отказ."""

    triangles = tuple(sorted(owner_triangles, key=lambda item: item.triangle_id.value))
    by_id = {item.triangle_id: item for item in triangles}
    pairs = _pair_map(triangles)
    rim_triangles: set = set()
    for first, second in rim_edges:
        carriers = pairs.get(frozenset((first, second)))
        if not carriers:
            raise refusal(
                NamedOutcome.DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED,
                f"the rim edge {first.value}->{second.value} is carried by no owner triangle",
            )
        rim_triangles.update(carriers)
    segments = tuple((snapped[first], snapped[second]) for first, second in rim_edges)
    near = _near_vertices(snapped, segments, reach)
    faces: dict = {}
    for item in triangles:
        faces.setdefault(item.source_face_id, []).append(item.triangle_id)
    included = {
        triangle_id
        for members in faces.values()
        if any(vertex in near for triangle_id in members for vertex in by_id[triangle_id].vertex_ids)
        for triangle_id in members
    }
    if not rim_triangles <= included:
        raise refusal(
            NamedOutcome.DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED,
            "a triangle on the rim is farther from the rim than the reach: the rim is not on the owner Patch",
        )
    components = [item for item in _components(included, by_id, pairs) if rim_triangles & set(item)]
    if len(components) != 1:
        raise refusal(
            NamedOutcome.DEVELOPABLE_BAND_SUPPORT_DISCONNECTED,
            f"the selected chains of the domain lie in {len(components)} separate components of the "
            f"support within reach {float(reach):.6g} m: one chart cannot cover them",
        )
    support = set(components[0])
    rounds = 0
    while rounds < PINCH_CLOSURE_ROUNDS:
        pinched = _pinched_vertices(support, by_id, pairs)
        if not pinched:
            break
        rounds += 1
        grow = {by_id[item].source_face_id for vertex in pinched for item in _incident(triangles, vertex)}
        before = len(support)
        for face in grow:
            support.update(faces[face])
        if len(support) == before:
            break
    ordered = tuple(by_id[item] for item in sorted(support, key=lambda item: item.value))
    left = [item for item in triangles if item.triangle_id not in support]
    return BandSupportV1(
        ordered, len(left), left[0].triangle_id if left else None, rounds
    )


def _incident(triangles, vertex) -> list:
    return [item.triangle_id for item in triangles if vertex in item.vertex_ids]
