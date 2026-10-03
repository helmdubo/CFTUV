"""Запас угла у граничной вершины: цель третьего предложения развёртки (ARAP со смещённым углом).

ЗАЧЕМ. Граничная вершина с РАЗОМКНУТЫМ веером, чья сумма углов не меньше полного оборота
(`building`, патч 89: ступенька 1.6 см, вершина `building:34` несёт 360.167°), не вкладывается
в плоскость ни при какой ИЗОМЕТРИИ: последний треугольник веера накрывает первый, и граничные
рёбра карты пересекаются. Шарнир даёт изометрию и честно называет самонакрытие
(`DEVELOPABLE_CHART_SELF_OVERLAP`), а ARAP из изометрии не выходит: у неё нулевая энергия, и
накрытие энергией не штрафуется. Лечится не положением вершин, а ЦЕЛЬЮ: угол при вершине
сжимается ровно настолько, чтобы веер оставил в обороте зазор, а растяжение, которое это стоит
(доли процента при избытке в доли градуса), судит тот же точный суд, что и у любого предложения.

ЗАКОН `CONE_RELIEF_NLERP_V1`. Для каждой граничной вершины `v` с разомкнутым веером считается
сумма углов `Θ` (binary64, `atan2` над точными рациональными скалярным и векторным
произведениями). Если `Θ` больше `2π − g` (`g` — запас `CONE_RELIEF_GAP_HALF_TURNS`, доля
половины оборота), вершина получает запас: каждый её треугольник заменяется целью — треугольником
с теми же двумя сторонами при `v` и углом при ней, сдвинутым к нулю нормированной линейной
интерполяцией единичного направления (`nlerp`): `u' ∝ (1 − ρ) u + ρ e`, `e` — направление первой
стороны. Это однопараметрическая монотонная семья (угол убывает с `ρ`, первый порядок —
`Δα ≈ −ρ sin α`), поэтому `ρ` выбирается по первому порядку: `ρ = (Θ − (2π − g)) / Σ sin α_i`,
округляется ВВЕРХ до кратного `1 / CONE_RELIEF_STEPS` и ограничивается `CONE_RELIEF_MAX_STEPS`.
Применение цели — только `+ - * /` и `sqrt` (в IEEE-754 они округляются верно); `atan2` решает
лишь, кому нужен запас и сколько, а квантование `ρ` стирает разницу в последнем бите между
платформами. Цель не авторитет: карту, которую даст ARAP к этим целям, судит точный
сертификат растяжения и простота границы (`_developable`), как карту любого предложения.

Что закон НЕ делает: не трогает внутренние вершины (замкнутый веер: дефект замыкания — растяжение,
которое уже судится), не лечит самонакрытие, у которого нет вершины с избытком (спираль ленты),
и не угадывает отказ: без вершин в плане предложение не строится и отказ шарнира остаётся.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

from ._unfold import squared_distance

#: Запас, который закон оставляет вееру в обороте, в долях половины оборота (`1/90` = 2°).
#: Запись реестра допусков `DEVELOPABLE_CONE_RELIEF_GAP_V1`: избыток в доли градуса снимается
#: растяжением в доли процента, а зазор в два градуса на метровом ребре — это сантиметры, далеко
#: за шагом решётки карты.
CONE_RELIEF_GAP_HALF_TURNS = Fraction(1, 90)

#: Шаг квантования `ρ` и его потолок (`CONE_RELIEF_MAX_STEPS / CONE_RELIEF_STEPS` = половина).
CONE_RELIEF_STEPS = 1024
CONE_RELIEF_MAX_STEPS = 512

CONE_RELIEF_LAW = "CONE_RELIEF_NLERP_V1"


@dataclass(frozen=True, slots=True)
class ReliefVertexV1:
    """Вершина, которой нужен запас: сумма углов (радианы) и квантованный `ρ = steps / CONE_RELIEF_STEPS`."""

    vertex_id: object
    angle_sum: float
    steps: int


def _corner_squares(triangle, vertex_id, positions):
    """`(A, B, C)`: квадраты двух сторон при вершине и противолежащей, точно."""

    ids = triangle.vertex_ids
    corner = ids.index(vertex_id)
    here = positions[vertex_id]
    left = positions[ids[(corner + 1) % 3]]
    right = positions[ids[(corner + 2) % 3]]
    return (
        squared_distance(here, left),
        squared_distance(here, right),
        squared_distance(left, right),
    )


def _angle_and_sine(squared_left, squared_right, squared_opposite):
    """Угол между сторонами и его синус: рациональное подкоренное, один `sqrt` на величину."""

    dot = (squared_left + squared_right - squared_opposite) / 2
    root = math.sqrt(float(squared_left * squared_right))
    cross = math.sqrt(float(squared_left * squared_right - dot * dot))
    return math.atan2(cross, float(dot)), cross / root


def cone_relief_plan(topology, positions) -> tuple:
    """Вершины, которым нужен запас угла, по имени вершины; пусто, если такой вершины нет."""

    allowed = 2.0 * math.pi - float(CONE_RELIEF_GAP_HALF_TURNS) * math.pi
    plan = []
    for vertex_id in sorted(topology.fans, key=lambda item: item.value):
        order, closed = topology.fans[vertex_id]
        if closed:
            continue
        total = 0.0
        sines = 0.0
        for triangle_id in order:
            angle, sine = _angle_and_sine(
                *_corner_squares(topology.by_id[triangle_id], vertex_id, positions)
            )
            total += angle
            sines += sine
        if not total > allowed or not sines > 0.0:
            continue
        steps = math.ceil((total - allowed) / sines * CONE_RELIEF_STEPS)
        plan.append(
            ReliefVertexV1(vertex_id, total, min(max(steps, 1), CONE_RELIEF_MAX_STEPS))
        )
    return tuple(plan)


def _relieved_far_square(squared_left, squared_right, squared_opposite, steps: int) -> Fraction:
    """Квадрат противолежащей стороны, когда угол при вершине сдвинут к нулю на `steps / CONE_RELIEF_STEPS`."""

    relief = steps / CONE_RELIEF_STEPS
    dot = (squared_left + squared_right - squared_opposite) / 2
    root = math.sqrt(float(squared_left * squared_right))
    cosine = float(dot) / root
    sine = math.sqrt(float(squared_left * squared_right - dot * dot)) / root
    keep = 1.0 - relief
    along = keep * cosine + relief
    across = keep * sine
    new_cosine = along / math.sqrt(along * along + across * across)
    return Fraction(float(squared_left) + float(squared_right) - 2.0 * (root * new_cosine))


def _pair(first: int, second: int) -> tuple[int, int]:
    return (first, second) if first < second else (second, first)


def relieved_squares(topology, positions, plan) -> dict:
    """Цели ARAP: `{треугольник: (квадрат 01, квадрат 02, квадрат 12)}` для треугольников при вершинах плана.

    Угол сдвигается при каждой вершине плана по порядку обхода треугольника (0, 1, 2): треугольник с двумя
    такими вершинами получает вторую цель от уже смещённых сторон. Остальные треугольники в словаре
    отсутствуют, и ARAP берёт для них изометрию источника.
    """

    steps_of = {item.vertex_id: item.steps for item in plan}
    targets = {}
    for triangle in topology.triangles:
        ids = triangle.vertex_ids
        if not any(vertex in steps_of for vertex in ids):
            continue
        squares = {
            pair: squared_distance(positions[ids[pair[0]]], positions[ids[pair[1]]])
            for pair in ((0, 1), (0, 2), (1, 2))
        }
        for corner in range(3):
            steps = steps_of.get(ids[corner])
            if steps is None:
                continue
            left, right = (corner + 1) % 3, (corner + 2) % 3
            squares[_pair(left, right)] = _relieved_far_square(
                squares[_pair(corner, left)],
                squares[_pair(corner, right)],
                squares[_pair(left, right)],
                steps,
            )
        targets[triangle.triangle_id] = (squares[(0, 1)], squares[(0, 2)], squares[(1, 2)])
    return targets


def relief_note(plan) -> str:
    """Кому дан запас и сколько: числа для текста отказа и разбора."""

    shown = ", ".join(
        f"{item.vertex_id.value} (angle_sum={math.degrees(item.angle_sum):.6f} deg, "
        f"relief={item.steps}/{CONE_RELIEF_STEPS})"
        for item in plan
    )
    return (
        f"cone relief {CONE_RELIEF_LAW} gap={float(CONE_RELIEF_GAP_HALF_TURNS) * 180.0:g} deg for "
        f"{len(plan)} boundary vertices: {shown}"
    )
