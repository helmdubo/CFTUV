"""Слитые ядра суммы корней: несколько действий над `SqrtSumV1` с ОДНОЙ нормировкой дробей на член ответа.

Цепочка `a.scaled(p) - b.scaled(q) + c`, `base + left * right`, `sum left_i * right_i` нормирует каждую дробь после каждого действия
(`gcd` на каждом члене каждого промежуточного значения). Здесь те же действия идут целыми над общим знаменателем, и `Fraction`
строится один раз на член результата. Каноническая форма единственна, поэтому значение, члены и порядок те же, что у цепочки;
типы коэффициентов тоже (`product_added` оставляет член `base`, которого нет в произведении, тем же объектом, как `__add__`).
Эталон — сама цепочка (`kernel/tests/test_clip_speed_paths.py`).
"""

from __future__ import annotations

from fractions import Fraction
from math import lcm

from ._radicand_products import accumulate_products as _accumulate_products
from .exact_sqrt_sum import SqrtSumV1, _integer_form


def oriented_sum(x: SqrtSumV1, y: SqrtSumV1, step_x: int, step_y: int, offset: int) -> SqrtSumV1:
    """`step_x * y - step_y * x + offset` при целых `step_x`, `step_y`, `offset`: одна нормировка дробей на член ответа.

    То же каноническое значение, что у цепочки `y.scaled(step_x) - x.scaled(step_y) + rational(offset)` (ориентация
    точки у прямой с целыми концами), без трёх промежуточных величин.
    """

    x_common, x_items = _integer_form(x.terms)
    y_common, y_items = _integer_form(y.terms)
    scale = lcm(x_common, y_common) if x_common != y_common else x_common
    merged: dict[int, int] = {}
    if step_x:
        factor = step_x * (scale // y_common)
        for radicand, numerator in y_items:
            merged[radicand] = numerator * factor
    if step_y:
        factor = step_y * (scale // x_common)
        for radicand, numerator in x_items:
            merged[radicand] = merged.get(radicand, 0) - numerator * factor
    if offset:
        merged[1] = merged.get(1, 0) + offset * scale
    return SqrtSumV1(
        tuple(
            (radicand, Fraction(value, scale))
            for radicand, value in sorted(merged.items())
            if value
        )
    )


def product_added(base: SqrtSumV1, left: SqrtSumV1, right: SqrtSumV1) -> SqrtSumV1:
    """`base + left * right` с одной нормировкой дробей на член ответа (то же значение, члены и типы, что у `base + left * right`).

    Член `base`, которого нет в произведении, переходит в ответ тем же объектом, как у `__add__`.
    """

    if not left.terms or not right.terms:
        return base
    base_common, base_items = _integer_form(base.terms)
    left_common, left_items = _integer_form(left.terms)
    right_common, right_items = _integer_form(right.terms)
    common = left_common * right_common
    scale = lcm(base_common, common) if base_common != common else common
    product: dict[int, int] = {}
    _accumulate_products(product, left_items, right_items, scale // common)
    factor = scale // base_common
    base_numerators = dict(base_items)
    out: dict[int, Fraction] = {}
    for radicand, value in product.items():
        if value:
            total = value + base_numerators.get(radicand, 0) * factor
            if total:
                out[radicand] = Fraction(total, scale)
    for radicand, coefficient in base.terms:
        if coefficient and not product.get(radicand):
            out[radicand] = coefficient
    return SqrtSumV1(tuple(sorted(out.items())))


def sum_of_products(
    products: "Iterable[tuple[SqrtSumV1, SqrtSumV1, int]]",
) -> SqrtSumV1:
    """`sum sign_i * left_i * right_i` (знак `+1` или `-1`) с ОДНОЙ нормировкой дробей в конце.

    То же каноническое значение, что у цепочки `total + left * right` (каноническая форма единственна), но слагаемые
    складываются целыми над общим знаменателем, и `Fraction` строится один раз на член результата, а не на каждое
    произведение и каждое сложение. Ноль у любого множителя слагаемого пропускает слагаемое, как `__mul__`.
    """

    forms = []
    scale = 1
    for left, right, sign in products:
        if not left.terms or not right.terms:
            continue
        left_common, left_items = _integer_form(left.terms)
        right_common, right_items = _integer_form(right.terms)
        common = left_common * right_common
        forms.append((left_items, right_items, sign, common))
        if common != 1:
            scale = lcm(scale, common)
    merged: dict[int, int] = {}
    for left_items, right_items, sign, common in forms:
        _accumulate_products(merged, left_items, right_items, sign * (scale // common))
    return SqrtSumV1(
        tuple(
            (radicand, Fraction(value, scale))
            for radicand, value in sorted(merged.items())
            if value
        )
    )
