"""Ярлык внутренней вершины развёртки: замыкается ли веер точно. Классификация, не суд.

Развёртка кладёт вершину один раз (сварка), и если веер вокруг неё не замыкается
(сумма углов не `2π`), невязка уходит в растяжение треугольников — её судит
`_stretch`. Здесь вершина только ПОЛУЧАЕТ ЯРЛЫК, который читает человек:

* `EXACT_DEVELOPABLE` — сумма углов равна `2π` ТОЧНО;
* `NEAR_DEVELOPABLE` — доказано, что сумма не `2π` (решение владельца: такая вершина
  принимается, пока растяжение в бюджете, как near-planar принимает наклон);
* `UNDECIDED_WORK_BUDGET` — не доказано ни то ни другое: веер длиннее объявленного,
  либо оболочка не разделила знак, либо кончился бюджет точной работы.

ТРИ ЗАКОНА, от дешёвого к дорогому. (1) Веер ТОЧНО компланарен в привязанных
рациональных координатах: сумма `2π` по построению. (2) Сертифицированная оболочка
суммы (`surface_cone_angle`) не содержит `2π`: доказано `≠`. (3) Оболочка содержит `2π`:
решает точный знак. Угол между сторонами с квадратами `A`, `B` при противолежащей `C`
есть `θ`, `2√(AB)·e^{iθ} = P + i√H`, `P = A + B - C`, `H = 4AB - P²`. Поэтому
`Π(P_i + i√H_i)` вещественно и положительно тогда и только тогда, когда `Σθ_i ≡ 0
(mod 2π)`. Каждое `√H_i` — `SqrtSumV1` (одно разложение на веер), произведение —
комплексное число из двух `SqrtSumV1`; мнимая часть, канонически равная нулю, даёт
замкнутость, а ненулевая доказывается оболочкой в `_ENCLOSURE_BITS` бит. Кратность
`k` в `Σθ = 2πk` отделяет оболочка (она содержит `2π`, а не `4π`).

Работа ограничена и детерминирована: веер длиннее `_EXACT_CLOSURE_MAX_FAN` не
считается, бюджет точной работы свой, а память канонизации холодная внутри вызова
(`isolated_factorization_memory`) — ярлык есть функция входа, а не истории процесса.
"""

from __future__ import annotations

from fractions import Fraction

from .contracts.metric import (
    DevelopableFanClosureLawV1,
    DevelopableVertexClassV1,
    VertexDevelopabilityClassV1,
)
from .exact_sqrt_sum import (
    ExactCanonicalizationWorkBudgetExhausted,
    SqrtSumV1,
    exact_work_budget,
    isolated_factorization_memory,
)
from .surface_cone_angle import angle_bounds, certified_cone_angle, two_pi_bounds

#: Наибольший веер, для которого считается точное замыкание: произведение `m`
#: комплексных множителей раскрывается в `2^(m-1)` членов. Больший веер получает
#: ярлык `UNDECIDED_WORK_BUDGET` по СТРУКТУРЕ, а не по времени.
_EXACT_CLOSURE_MAX_FAN = 8

#: Разрядность оболочки, которая решает знак точного замыкания.
_ENCLOSURE_BITS = 256

_STAGE = "DEVELOPABLE_FAN_CLOSURE"


def _squared(first, second) -> Fraction:
    delta = tuple(a - b for a, b in zip(first, second, strict=True))
    return sum((a * a for a in delta), Fraction(0))


def _corner_triples(vertex_id, fan_ids, by_id, positions):
    """`(A, B, C)` каждого треугольника веера при вершине: две стороны у неё и третья."""

    triples = []
    for triangle_id in fan_ids:
        triangle = by_id[triangle_id]
        index = triangle.vertex_ids.index(vertex_id)
        here = positions[vertex_id]
        before = positions[triangle.vertex_ids[(index + 1) % 3]]
        after = positions[triangle.vertex_ids[(index + 2) % 3]]
        triples.append(
            (_squared(here, before), _squared(here, after), _squared(before, after))
        )
    return tuple(triples)


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _is_exactly_coplanar(fan_ids, by_id, positions) -> bool:
    """Все треугольники веера в одной плоскости, точно: нормали параллельны."""

    reference = None
    for triangle_id in fan_ids:
        corners = tuple(positions[item] for item in by_id[triangle_id].vertex_ids)
        normal = _cross(
            tuple(a - b for a, b in zip(corners[1], corners[0], strict=True)),
            tuple(a - b for a, b in zip(corners[2], corners[0], strict=True)),
        )
        if not any(normal):
            return False
        if reference is None:
            reference = normal
        elif any(_cross(reference, normal)):
            return False
    return reference is not None


def _complex_product(triples, budget):
    """`Π(P_i + i√H_i)` как пара `SqrtSumV1` `(Re, Im)`."""

    real, imaginary = SqrtSumV1.rational(1), SqrtSumV1.zero()
    for first, second, opposite in triples:
        pivot = first + second - opposite
        height = 4 * first * second - pivot * pivot
        root = SqrtSumV1.radical(1, height, budget)
        real, imaginary = (
            real.scaled(pivot) - imaginary * root,
            real * root + imaginary.scaled(pivot),
        )
    return real, imaginary


def _exact_closure(triples, domain_id: str):
    """`True` — сумма `≡ 0 (mod 2π)`, `False` — доказано иначе, `None` — не решено."""

    with isolated_factorization_memory():
        budget = exact_work_budget(stage=_STAGE, domain_id=domain_id)
        try:
            real, imaginary = _complex_product(triples, budget)
        except ExactCanonicalizationWorkBudgetExhausted:
            return None
        if imaginary.is_zero:
            sign = real.certified_sign(_ENCLOSURE_BITS)
            return None if sign is None else sign > 0
        return False if imaginary.certified_sign(_ENCLOSURE_BITS) is not None else None


def two_pi_enclosure():
    """Сертифицированная оболочка `2π` тем же устройством, что и у оболочек суммы."""

    return certified_cone_angle([two_pi_bounds()])


def classify_vertex(
    vertex_id, fan_ids, by_id, positions, domain_id: str
) -> DevelopableVertexClassV1:
    """Ярлык одной ВНУТРЕННЕЙ вершины (замкнутый веер)."""

    triples = _corner_triples(vertex_id, fan_ids, by_id, positions)
    if _is_exactly_coplanar(fan_ids, by_id, positions):
        return DevelopableVertexClassV1(
            vertex_id=vertex_id,
            developability_class=VertexDevelopabilityClassV1.EXACT_DEVELOPABLE,
            closure_law=DevelopableFanClosureLawV1.EXACT_PLANAR_CLOSED_FAN_V1,
            angle_sum_enclosure=two_pi_enclosure(),
            fan_triangle_count=len(fan_ids),
        )
    bounds = [angle_bounds(*triple) for triple in triples]
    enclosure = certified_cone_angle(bounds)
    low, high = two_pi_bounds()
    contains = Fraction(enclosure.lower) <= low and high <= Fraction(enclosure.upper)
    if not contains:
        return DevelopableVertexClassV1(
            vertex_id=vertex_id,
            developability_class=VertexDevelopabilityClassV1.NEAR_DEVELOPABLE,
            closure_law=DevelopableFanClosureLawV1.CERTIFIED_INTERVAL_ENCLOSURE_V1,
            angle_sum_enclosure=enclosure,
            fan_triangle_count=len(fan_ids),
        )
    verdict = (
        _exact_closure(triples, domain_id)
        if len(fan_ids) <= _EXACT_CLOSURE_MAX_FAN
        else None
    )
    if verdict is None:
        label = VertexDevelopabilityClassV1.UNDECIDED_WORK_BUDGET
        law = DevelopableFanClosureLawV1.FAN_CLOSURE_UNDECIDED_V1
    else:
        label = (
            VertexDevelopabilityClassV1.EXACT_DEVELOPABLE
            if verdict
            else VertexDevelopabilityClassV1.NEAR_DEVELOPABLE
        )
        law = DevelopableFanClosureLawV1.EXACT_FAN_CLOSURE_SQRT_SUM_V1
    return DevelopableVertexClassV1(
        vertex_id=vertex_id,
        developability_class=label,
        closure_law=law,
        angle_sum_enclosure=enclosure,
        fan_triangle_count=len(fan_ids),
    )


def classify_interior_vertices(topology, positions, domain_id: str):
    """Ярлыки всех внутренних вершин диска, по имени вершины."""

    result = []
    for vertex_id in sorted(topology.fans, key=lambda item: item.value):
        order, closed = topology.fans[vertex_id]
        if closed:
            result.append(
                classify_vertex(vertex_id, order, topology.by_id, positions, domain_id)
            )
    return tuple(result)


def angle_defect_lower_bound(item: DevelopableVertexClassV1) -> Fraction:
    """Доказанная нижняя граница `|Σθ - 2π|` по оболочке; нуль, если оболочка её не отделяет."""

    low, high = two_pi_bounds()
    return max(
        Fraction(item.angle_sum_enclosure.lower) - high,
        low - Fraction(item.angle_sum_enclosure.upper),
        Fraction(0),
    )


def worst_defect_vertex(classes):
    """Вершина с наибольшей ДОКАЗАННОЙ невязкой замыкания (первая по имени при равенстве)."""

    best = None
    for item in sorted(classes, key=lambda entry: entry.vertex_id.value):
        defect = angle_defect_lower_bound(item)
        if defect > 0 and (best is None or defect > best[0]):
            best = (defect, item.vertex_id)
    return None if best is None else best[1]
