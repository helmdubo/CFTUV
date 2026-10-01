"""Объявленные прямыми цепи на карте развёртки: размещение внутренних вершин и боковой угол.

Очередь требует от объявленной прямой цепи ТОЧНОЙ коллинеарности её вершин в карте
(`reference.evaluation_geometry`). Независимая привязка каждой вершины развёртки к решётке
этого не даёт для любой прямой вне осей решётки, а карта развёртки к тому же искривляет
цепь по существу, если внутренняя геометрия патча у её вершин не плоская. Здесь два шага.

РАЗМЕЩЕНИЕ. Концы цепи — привязанные узлы развёртки. Если узлы всех её вершин уже лежат
на одной прямой и идут по ней строго по порядку, цепь остаётся как есть (байты такой карты
прежние). Иначе внутренние вершины кладутся на отрезок между концами с РАЦИОНАЛЬНЫМ
параметром: проекция положения вершины в предложении развёртки на этот отрезок,
округлённая до двоичной дроби (сдвиг вдоль хорды меньше восьмой доли узла; строгий
порядок сохранён, шаг уменьшается ровно до тех пор, пока он держится). Коллинеарность тогда по
построению; сдвиг вершины — её расстояние до хорды, а не новый допуск: его судит судья
растяжения, как и любой другой сдвиг карты. Цепь, которая у предложения идёт не строго
вперёд вдоль хорды (возвращается назад), положить нельзя — это называется, а не
округляется.

БОКОВОЙ УГОЛ. Прямая в карте — ровно `π` на стороне патча. Сумма углов веера на стороне
патча у внутренней вершины цепи — ДОКАЗАТЕЛЬСТВО: сертифицированная оболочка, отделённая от
`π`, говорит, что цепь по внутренней геометрии искривлена, и прямой в карте она стала бы
ценой растяжения. Доказательство строится, только когда веер вершины стягивается ровно к
двум рёбрам цепи (цепь — часть границы домена); иначе вердикт по этой вершине не
выносится, а растяжение судит как прежде. Вершина, у которой веер точно плоский, а два
ребра цепи точно коллинеарны, не кандидат: её угол ровно `π`.
"""

from __future__ import annotations

from fractions import Fraction

from ._fan_closure import _corner_triples, _is_exactly_coplanar
from .contracts.metric import DevelopableDeclaredChainV1
from .surface_cone_angle import angle_bounds, certified_cone_angle, two_pi_bounds


def _sub(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _dot(left, right) -> Fraction:
    return sum((a * b for a, b in zip(left, right, strict=True)), Fraction(0))


def _cross2(left, right):
    return left[0] * right[1] - left[1] * right[0]


def _round_half_up(value: Fraction) -> int:
    return (2 * value.numerator + value.denominator) // (2 * value.denominator)


def _already_straight(chain, nodes) -> bool:
    """Узлы цепи на одной прямой и идут по ней строго вперёд между концами."""

    first, last = nodes[chain[0]], nodes[chain[-1]]
    span = _sub(last, first)
    reach = _dot(span, span)
    if not reach:
        return False
    previous = Fraction(0)
    for vertex in chain[1:-1]:
        offset = _sub(nodes[vertex], first)
        along = _dot(offset, span)
        if _cross2(span, offset) or not previous < along < reach:
            return False
        previous = along
    return True


def _steps(params: list, span) -> list:
    """Двоичные дроби хорды, сохраняющие строгий порядок `0 < p1 < ... < pm < 1`.

    Шаг стартует с `2^-(b+3)` для `b` разрядов длиннейшей проекции хорды: сдвиг вершины
    вдоль хорды меньше восьмой доли узла. Не хватило для строгого порядка — шаг вдвое
    мельче, пока он не удержится (параметры строго возрастают, конец у перебора есть).
    """

    bits = max(abs(span[0]), abs(span[1])).bit_length() + 3
    while True:
        unit = 1 << bits
        steps = [Fraction(_round_half_up(item * unit), unit) for item in params]
        chain = [Fraction(0), *steps, Fraction(1)]
        if all(left < right for left, right in zip(chain, chain[1:])):
            return steps
        bits += 1


def _place(chain, exact, nodes, scale):
    """`(координаты внутренних вершин, причина | None)` одной цепи."""

    first, last = nodes[chain[0]], nodes[chain[-1]]
    span = _sub(last, first)
    reach = _dot(span, span)
    if not reach:
        return {}, "the endpoints of the chain meet in one lattice node"
    params = [
        _dot(
            _sub(tuple(axis * scale for axis in exact[vertex]), first), span
        )
        / reach
        for vertex in chain[1:-1]
    ]
    chord = [Fraction(0), *params, Fraction(1)]
    if not all(left < right for left, right in zip(chord, chord[1:])):
        return {}, "the chain does not run strictly forward along its endpoint chord"
    placed = {
        vertex: (first[0] + step * span[0], first[1] + step * span[1])
        for vertex, step in zip(chain[1:-1], _steps(params, span))
    }
    return placed, None


def chain_coordinates(chains, exact, nodes, scale):
    """`(координаты в единицах решётки, причина | None)`: узлы с внутренностями цепей на хордах.

    `exact` — предложение развёртки в метрах (дроби), `nodes` — привязанные узлы,
    `scale` — масштаб решётки карты. Цепи, чьи вершины не все на карте, пропускаются.
    """

    coordinates = dict(nodes)
    taken: set = set()
    for chain in chains:
        if any(vertex not in nodes for vertex in chain):
            continue
        if _already_straight(chain, nodes):
            continue
        placed, reason = _place(chain, exact, nodes, scale)
        if reason is not None:
            return coordinates, (
                f"chain {chain[0].value}..{chain[-1].value}: {reason}"
            )
        coordinates.update(placed)
        taken.update(chain[1:-1])
    ends = {vertex for chain in chains for vertex in (chain[0], chain[-1])}
    interior = [vertex for chain in chains for vertex in chain[1:-1]]
    clash = sorted(
        {item.value for item in taken & ends}
        | {item.value for item in interior if interior.count(item) > 1}
    )
    if clash:
        return coordinates, (
            f"vertices {clash[:3]} belong to two declared straight chains, so the "
            "chains cannot be placed one at a time"
        )
    return coordinates, None


def lattice_displacement(exact, coordinates, scale):
    """`(сдвинутых вершин, наибольшее смещение по оси в узлах)` карты относительно предложения."""

    moved, residual = 0, Fraction(0)
    for vertex, point in coordinates.items():
        shifts = [abs(exact[vertex][axis] * scale - point[axis]) for axis in range(2)]
        if any(shifts):
            moved += 1
            residual = max(residual, *shifts)
    return moved, residual


def _is_straight_boundary(chain, index, fan_ids, topology, positions) -> bool:
    """Веер точно плоский, а два ребра цепи у вершины точно коллинеарны: угол ровно `π`."""

    before = _sub(positions[chain[index]], positions[chain[index - 1]])
    after = _sub(positions[chain[index + 1]], positions[chain[index]])
    cross = (
        before[1] * after[2] - before[2] * after[1],
        before[2] * after[0] - before[0] * after[2],
        before[0] * after[1] - before[1] * after[0],
    )
    return not any(cross) and _is_exactly_coplanar(fan_ids, topology.by_id, positions)


def _side_angle(chain, index, topology, positions):
    """Оболочка суммы углов веера на стороне патча у вершины цепи либо `None`."""

    vertex = chain[index]
    fan = topology.fans.get(vertex)
    if fan is None or fan[1]:
        return None
    neighbours: set = set()
    for triangle_id in fan[0]:
        triangle = topology.by_id[triangle_id]
        for ordinal in range(3):
            if topology.opposite[(triangle_id, ordinal)] is not None:
                continue
            first = triangle.vertex_ids[ordinal]
            second = triangle.vertex_ids[(ordinal + 1) % 3]
            if vertex in (first, second):
                neighbours.add(second if vertex == first else first)
    if neighbours != {chain[index - 1], chain[index + 1]}:
        return None
    if _is_straight_boundary(chain, index, fan[0], topology, positions):
        return None
    triples = _corner_triples(vertex, fan[0], topology.by_id, positions)
    return certified_cone_angle([angle_bounds(*triple) for triple in triples])


def pi_defect_lower_bound(enclosure) -> Fraction:
    """Доказанная нижняя граница `|угол - π|` по оболочке; нуль, если оболочка `π` не отделяет."""

    low, high = two_pi_bounds()
    return max(
        Fraction(enclosure.lower) - high / 2,
        low / 2 - Fraction(enclosure.upper),
        Fraction(0),
    )


def declared_chain_records(chains, topology, positions):
    """Записи объявленных цепей: вершины и худшее доказательство боковой стороны."""

    records = []
    for chain in chains:
        if any(vertex not in positions for vertex in chain):
            continue
        best = None
        for index in range(1, len(chain) - 1):
            enclosure = _side_angle(chain, index, topology, positions)
            if enclosure is None:
                continue
            defect = pi_defect_lower_bound(enclosure)
            if best is None or defect > best[0]:
                best = (defect, chain[index], enclosure)
        records.append(
            DevelopableDeclaredChainV1(
                vertex_ids=tuple(chain),
                worst_vertex_id=None if best is None else best[1],
                side_angle_enclosure=None if best is None else best[2],
                defect_proven=best is not None and best[0] > 0,
            )
        )
    return tuple(records)


def bent_chain_text(records) -> str:
    """Доказательства искривлённых цепей: цепь, худшая вершина, оболочка угла и её отделение от `π`."""

    lines = []
    for item in records:
        if not item.defect_proven:
            continue
        defect = pi_defect_lower_bound(item.side_angle_enclosure)
        lines.append(
            f"chain {item.vertex_ids[0].value}..{item.vertex_ids[-1].value} "
            f"({len(item.vertex_ids)} vertices): worst vertex {item.worst_vertex_id.value}, "
            f"side angle in [{item.side_angle_enclosure.lower}, "
            f"{item.side_angle_enclosure.upper}] rad against pi, "
            f"proven defect >= {float(defect):.6e} rad"
        )
    if not lines:
        return "no chain carries a side-angle proof (fans do not contract to the chain)"
    extra = f"; +{len(lines) - 3} more" if len(lines) > 3 else ""
    return "; ".join(lines[:3]) + extra
