"""Сварка вершин соседних доменов и митра смещения (MESH_ASSEMBLY_NATIVE_V1, срез B2): эталон, с которым побитово сверен нативный порт (`cftuv_native.mesh_weld`).

Чистая математика без Blender. Хост (`cftuv/envelope_production_weld.py`, его докстринг описывает законы: сварка по ссылке `location:src:` при побитово
равных позициях, митра как пересечение сдвинутых плоскостей, закон конца стены `WALL_MITER_OFFSET_V1`) зовёт `weld_core` здесь либо через
`backend.mesh_assembly_compute`, а тексты находок (`.6g`, `math.dist` худшего зазора) пишет сам из чисел ответа. Семантика — CPython 3.11 на любом интерпретаторе: сумма слева
(`_cpython311.left_fold_sum` с целого нуля; `sum` 3.12+ компенсирует), `max` берёт первое из равных, равенство нормалей числовое (`-0.0 == 0.0`), позиции сравниваются побитово (`float.hex`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ._cpython311 import left_fold_sum

#: Только ссылки вершин ИСХОДНИКА сваривают: ссылка узла (`location:node:<k>`) принадлежит домену.
WELD_LOCATION_PREFIX = "location:src:"


def _dot(left, right) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _eliminate(rows, size, pivot_epsilon):
    """Метод Гаусса — Жордана с выбором ведущего: `(номера ведущих столбцов, строки)`."""

    pivots = []
    row = 0
    for column in range(size):
        if row == size:
            break
        best = max(range(row, size), key=lambda index: abs(rows[index][column]))
        if abs(rows[best][column]) <= pivot_epsilon:
            continue
        rows[row], rows[best] = rows[best], rows[row]
        scale = rows[row][column]
        rows[row] = [value / scale for value in rows[row]]
        for other in range(size):
            if other != row and rows[other][column]:
                factor = rows[other][column]
                rows[other] = [a - factor * b for a, b in zip(rows[other], rows[row])]
        pivots.append(column)
        row += 1
    return pivots, rows


def miter_offset(normals, law):
    """`(вектор смещения при d = 1, множитель митры, причина)`: `n_i . o = 1` для всех нормалей.

    `o` ищется в линейной оболочке нормалей (`o = sum a_i n_i`, `G a = 1`, `G` — матрица Грама), и
    она единственна, когда система совместна. Равные побитово нормали дают сразу `n`. Отказ —
    `(None, множитель | None, причина)`: система несовместна (почти противоположные либо
    несходящиеся плоскости) либо множитель митры выше предела. `law` — `(предел митры, допуск ведущего, допуск невязки, синус стены)` хоста (`WELD_LAW`).
    """

    first = normals[0]
    if all(item == first for item in normals):
        return first, 1.0, ""
    size = len(normals)
    rows = [[_dot(a, b) for b in normals] + [1.0] for a in normals]
    limit, pivot_epsilon, residual_epsilon, _sine = law
    pivots, rows = _eliminate(rows, size, pivot_epsilon)
    for row in range(len(pivots), size):
        if abs(rows[row][size]) > residual_epsilon:
            return None, None, "the offset planes do not meet in one point"
    weights = [0.0] * size
    for row, column in enumerate(pivots):
        weights[column] = rows[row][size]
    offset = tuple(
        left_fold_sum(weight * normal[axis] for weight, normal in zip(weights, normals))
        for axis in range(3)
    )
    if any(abs(_dot(normal, offset) - 1.0) > residual_epsilon for normal in normals):
        return None, None, "the offset planes do not meet in one point"
    factor = math.sqrt(_dot(offset, offset))
    if factor > limit:
        return None, factor, "the miter exceeds the limit"
    return offset, factor, ""


def _hex(point) -> tuple:
    return tuple(float(axis).hex() for axis in point)


def _groups_by_reference(domains):
    found: dict = {}
    for ordinal, domain in enumerate(domains):
        for local, ref in enumerate(domain.refs):
            if ref is not None and ref.startswith(WELD_LOCATION_PREFIX):
                found.setdefault(ref, []).append((ordinal, local))
    return {ref: members for ref, members in found.items() if len(members) > 1}


def _plan(domains, law):
    """`(класс сварки по (домен, вершина), смещение класса, (расхождений, точки расхождений по группе, отказов митры, худший множитель, причины))`."""

    classes: dict = {}
    miters: dict = {}
    mismatches = fallbacks = 0
    gaps: list = []
    worst_factor = 0.0
    reasons: set = set()
    for members in _groups_by_reference(domains).values():
        by_position: dict = {}
        for ordinal, local in members:
            point = domains[ordinal].positions[local]
            by_position.setdefault(_hex(point), []).append((ordinal, local))
        if len(by_position) > 1:
            mismatches += 1
            gaps.append(tuple(domains[o].positions[l] for o, l in (cls[0] for cls in by_position.values())))
        for cls in by_position.values():
            if len(cls) < 2:
                continue
            offset, factor, reason = miter_offset(
                [domains[o].normals[l] for o, l in cls], law
            )
            if offset is None:
                fallbacks += 1
                reasons.add(reason)
                worst_factor = max(worst_factor, factor or 0.0)
                continue
            for member in cls:
                classes[member] = cls[0]
            miters[cls[0]] = offset
    return classes, miters, (mismatches, gaps, fallbacks, worst_factor, reasons)




@dataclass(frozen=True, slots=True)
class WallPlanV1:
    """Итог закона конца стены: сдвиги узлов (при `d = 1`), слитые пары узлов и числа.

    `lifts[(домен, вершина)]` — вектор смещения узла; `classes[(домен, вершина)]` — голова класса слитых узлов, `miters[голова]` — общий вектор
    класса. Числа: смещённые узлы, слитые вершины (`len(classes) - len(miters)`), узлы без соседа, отказы митры с причинами и худшим множителем.
    """

    lifts: dict
    classes: dict
    miters: dict
    lifted: int
    alone: int
    fallbacks: int
    worst_factor: float
    reasons: frozenset
    #: `((домен, домен соседа), ...)`: смещение узла стены этого домена зависит от нормалей вершин ДОМЕНА-СОСЕДА (его нормаль в точке узла смешана вдоль его стены). Аффинность позиции узла от ширины
    #: поэтому держится, лишь пока нормали соседа одни (плоский сосед); модель превью (`envelope_width_preview_model`) читает эти пары.
    partners: tuple = ()

    @property
    def welded(self) -> int:
        return len(self.classes) - len(self.miters)


def _is_anchor(domain, local) -> bool:
    ref = domain.refs[local]
    return ref is not None and ref.startswith(WELD_LOCATION_PREFIX)


def _normalised(vector):
    length = math.sqrt(_dot(vector, vector))
    return None if not length else (vector[0] / length, vector[1] / length, vector[2] / length)


def _wall_heading(other, chain, at, direction, sine):
    """`(вершина за якорем, вектор от якоря, его длина)` первой стороны стены соседа, идущей тем же ребром, что `direction`, либо `None`."""

    length = math.sqrt(_dot(direction, direction))
    for step in (-1, 1):
        position = at + step
        if not 0 <= position < len(chain):
            continue
        heading = tuple(b - a for a, b in zip(other.positions[chain[at]], other.positions[chain[position]]))
        reach = math.sqrt(_dot(heading, heading))
        if not reach or not length:
            continue
        cross = _cross(direction, heading)
        if _dot(direction, heading) > 0.0 and math.sqrt(_dot(cross, cross)) <= sine * length * reach:
            return chain[position], heading, reach
    return None


def _wall_partner(domains, walls, table, ordinal, chain, at, sine):
    """`(домен соседа, его вершина за якорем, нормаль соседа в точке узла)` либо `None`: у узла нет соседа на стене.

    Якорь узла — соседняя с ним по цепи вершина `src:`; сосед — домен, у которого стена выходит из той же вершины (одна ссылка, побитово равная позиция)
    вдоль того же ребра. Нормаль соседа в точке узла — линейная по длине его стены между нормалью в якоре и нормалью за якорем (дальше конца — она сама).
    """

    domain = domains[ordinal]
    for step in (-1, 1):
        position = at + step
        if not 0 <= position < len(chain) or not _is_anchor(domain, chain[position]):
            continue
        anchor = chain[position]
        direction = tuple(b - a for a, b in zip(domain.positions[anchor], domain.positions[chain[at]]))
        for other_ordinal, number, place in table.get(domain.refs[anchor], ()):
            other = domains[other_ordinal]
            other_chain = walls[other_ordinal][number]
            if other_ordinal == ordinal or _hex(other.positions[other_chain[place]]) != _hex(domain.positions[anchor]):
                continue
            along = _wall_heading(other, other_chain, place, direction, sine)
            if along is None:
                continue
            beyond, heading, reach = along
            share = min(1.0, _dot(direction, heading) / (reach * reach))
            blended = _normalised(
                tuple((1.0 - share) * a + share * b for a, b in zip(other.normals[other_chain[place]], other.normals[beyond]))
            )
            if blended is not None:
                return other_ordinal, beyond, blended
    return None


def plan_wall_nodes(domains, law) -> WallPlanV1:
    """Закон `WALL_MITER_OFFSET_V1`: сдвиги узлов стены на линию сгиба и слияние пар узлов с побитово равными позициями.

    `domains` — `DomainVerticesV1` с цепями стен (`walls`); домен без них ничего не даёт (поведение прежнее). Предел митры и допуски — те же, что у
    `miter_offset`. Обход по номеру домена, цепям и местам в цепи: ответ от порядка множеств не зависит. Домен прежней раскладки
    (воркер пула, поднятый до правки, присылает вершины без поля `walls`) стен не имеет: закон на нём не действует, как и без цепей.
    """

    walls = [getattr(domain, "walls", ()) for domain in domains]
    table: dict = {}
    for ordinal, domain in enumerate(domains):
        for number, chain in enumerate(walls[ordinal]):
            for at, local in enumerate(chain):
                if _is_anchor(domain, local):
                    table.setdefault(domain.refs[local], []).append((ordinal, number, at))
    lifts: dict = {}
    classes: dict = {}
    miters: dict = {}
    seen: set = set()
    lifted = alone = fallbacks = 0
    worst = 0.0
    reasons: set = set()
    partners: set = set()
    for ordinal, domain in enumerate(domains):
        for chain in walls[ordinal]:
            for at, local in enumerate(chain):
                if _is_anchor(domain, local) or (ordinal, local) in seen:
                    continue
                seen.add((ordinal, local))
                partner = _wall_partner(domains, walls, table, ordinal, chain, at, law[3])
                if partner is None:
                    alone += 1
                    continue
                other_ordinal, beyond, blended = partner
                other = domains[other_ordinal]
                twin = (
                    not _is_anchor(other, beyond)
                    and (other_ordinal, beyond) not in seen
                    and _hex(other.positions[beyond]) == _hex(domain.positions[local])
                )
                offset, factor, reason = miter_offset([domain.normals[local], other.normals[beyond] if twin else blended], law)
                if offset is None:
                    fallbacks += 1
                    reasons.add(reason)
                    worst = max(worst, factor or 0.0)
                    continue
                if not twin:
                    lifts[(ordinal, local)] = offset
                    partners.add((ordinal, other_ordinal))
                    lifted += 1
                    continue
                head = (ordinal, local)
                seen.add((other_ordinal, beyond))
                classes[head] = classes[(other_ordinal, beyond)] = head
                miters[head] = offset
                lifted += 2
    return WallPlanV1(lifts, classes, miters, lifted, alone, fallbacks, worst, frozenset(reasons), tuple(sorted(partners)))


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def weld_core(domains, offset: float, law) -> tuple:
    """Числа сварки вершин доменов (`DomainVerticesV1`: `positions`, `normals`, `refs`, `walls`) со смещением `offset` под законом хоста `law` (`miter_offset`).

    `(позиции меша, номера вершин по домену, групп сварки, слитых вершин, расхождений позиций, точки расхождений по группе, отказов митры,
    худший множитель митры, причины отказов, узлов стены смещено, слито, без соседа, отказов митры стены, худший множитель стены,
    причины стены, пары доменов стены)`; причины — по возрастанию.
    """

    classes, miters, (mismatches, gaps, fallbacks, factor, reasons) = _plan(domains, law)
    merged = len(classes) - len(miters)
    groups = len(miters)
    wall = plan_wall_nodes(domains, law)
    classes = {**classes, **wall.classes}
    miters = {**miters, **wall.miters}
    positions: list = []
    placed: dict = {}
    index = []
    for ordinal, domain in enumerate(domains):
        mapping = []
        for local, point in enumerate(domain.positions):
            head = classes.get((ordinal, local))
            if head is not None and head in placed:
                mapping.append(placed[head])
                continue
            shift = miters[head] if head is not None else wall.lifts.get((ordinal, local), domain.normals[local])
            positions.append(
                (
                    point[0] + offset * shift[0],
                    point[1] + offset * shift[1],
                    point[2] + offset * shift[2],
                )
            )
            mapping.append(len(positions) - 1)
            if head is not None:
                placed[head] = len(positions) - 1
        index.append(tuple(mapping))
    return (
        tuple(positions), tuple(index), groups, merged, mismatches, tuple(gaps), fallbacks, factor, tuple(sorted(reasons)),
        wall.lifted, wall.welded, wall.alone, wall.fallbacks, wall.worst_factor, tuple(sorted(wall.reasons)), wall.partners,
    )


__all__ = (
    "WELD_LOCATION_PREFIX",
    "WallPlanV1",
    "miter_offset",
    "plan_wall_nodes",
    "weld_core",
)
