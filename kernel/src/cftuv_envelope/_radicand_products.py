"""Произведение сумм корней на целых: общий цикл `_multiply_integer_items` и сумм произведений.

Только целые числа и никакой `SqrtSumV1`: модуль лежит под `exact_sqrt_sum`, и тот импортирует его, а не наоборот.
"""

from __future__ import annotations

from math import gcd

# Произведение радикандов `(a, b) -> (g, a*b/g^2)`, `g = gcd(a, b)`: чистая функция пары, поэтому память ничего не
# решает и ответа не меняет. Радиканды одного домена — произведения немногих простых, пар на порядки меньше, чем
# произведений в резке (замер `sagging_wall`: 156 тысяч попарных произведений на 305 различных радикандов), а `gcd`
# двух сотен бит и два деления стоят дороже поиска в словаре. Оба порядка пары хранятся: так ищется без сравнения.
_RADICAND_PRODUCTS: dict[tuple[int, int], tuple[int, int]] = {}
_RADICAND_PRODUCTS_LIMIT = 1 << 16


def clear_products() -> None:
    """Сбросить память пар (граница работы домена; на ответ не влияет)."""

    _RADICAND_PRODUCTS.clear()


def _radicand_product(left: int, right: int) -> tuple[int, int]:
    """`(g, a*b/g^2)` для радикандов `a`, `b` (оба больше единицы), из памяти пар."""

    common = gcd(left, right)
    found = (common, (left // common) * (right // common))
    if len(_RADICAND_PRODUCTS) >= _RADICAND_PRODUCTS_LIMIT:
        _RADICAND_PRODUCTS.clear()
    _RADICAND_PRODUCTS[(left, right)] = found
    _RADICAND_PRODUCTS[(right, left)] = found
    return found


def accumulate_products(
    merged: dict[int, int],
    left: list[tuple[int, int]],
    right: list[tuple[int, int]],
    weight: int = 1,
) -> None:
    """Дописывает в `merged` произведение `weight * (sum a_m*sqrt(m)) * (sum b_m*sqrt(m))` (целые, без нормировки).

    `sqrt(a)*sqrt(b) = g*sqrt(a*b/g^2)`, `g = gcd(a, b)`: на радикандах-единицах `g = 1`, а пары больших берутся из
    памяти (`_RADICAND_PRODUCTS`). Вес множится в левый член (один раз на член, а не на пару).
    """

    products = _RADICAND_PRODUCTS
    get = merged.get
    for left_radicand, left_numerator in left:
        if weight != 1:
            left_numerator *= weight
        if left_radicand == 1:
            for right_radicand, right_numerator in right:
                merged[right_radicand] = get(right_radicand, 0) + left_numerator * right_numerator
            continue
        for right_radicand, right_numerator in right:
            if right_radicand == 1:
                merged[left_radicand] = get(left_radicand, 0) + left_numerator * right_numerator
                continue
            found = products.get((left_radicand, right_radicand))
            if found is None:
                found = _radicand_product(left_radicand, right_radicand)
            common, radicand = found
            merged[radicand] = get(radicand, 0) + left_numerator * right_numerator * common


