"""Фильтр знака в binary64 либо доказывает знак, либо уступает точному пути: ответ `orientation` не меняется.

`faces.orientation` стоит почти 60% времени материализации (Blender 4.5, `2` домен 0), потому что считает знак
произведения двух сумм корней точно. `float_filter` даёт знак по центру и границе ошибки каждой координаты и молчит
(`None`), когда не уверен. Здесь проверяется ровно это обещание:

1. НИКОГДА неверный знак: на случайных точках из сумм корней и на подобранных вырожденных (коллинеарные, точный ноль)
   и почти вырожденных (разность двух корней порядка 1e-16 от значения) фильтр отвечает либо точным знаком, либо `None`;
2. ПРЕДИКАТ не изменился: `faces.orientation` и `shoelace_sign` равны определению через точную арифметику на том же
   корпусе, а фильтр на этом корпусе РЕШАЕТ большую часть вопросов (иначе он был бы пустышкой);
3. границы НАСТОЯЩИЕ: с отключённым запасом на ошибку (`_SLACK = 0`) тот же корпус ловит неверный знак на коллинеарных
   тройках, поэтому тест не пройдёт на фильтре без запаса;
4. краевые значения (радикант за пределами `float`) уходят в точный путь, а не в исключение.
"""

from __future__ import annotations

import random
from fractions import Fraction

import cftuv_envelope.float_filter as float_filter
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.wavefront.faces import doubled_shoelace, orientation, shoelace_sign

RADICANDS = (2, 3, 5, 6, 7, 10, 11, 13, 14, 15)


def _value(rng: random.Random, terms: int) -> SqrtSumV1:
    total = SqrtSumV1.rational(Fraction(rng.randint(-9, 9), rng.randint(1, 5)))
    for radicand in rng.sample(RADICANDS, terms):
        total = total + SqrtSumV1.radical(Fraction(rng.randint(-9, 9), rng.randint(1, 4)), radicand)
    return total


def _point(rng: random.Random):
    return (_value(rng, rng.randint(0, 4)), _value(rng, rng.randint(0, 4)))


def _exact_orientation(first, second, third) -> int:
    return (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    ).sign()


def _scaled(point, factor: Fraction):
    return (point[0].scaled(factor), point[1].scaled(factor))


def _collinear_triples(rng: random.Random, count: int):
    for _ in range(count):
        origin = _point(rng)
        direction = _point(rng)
        first = (origin[0] + direction[0], origin[1] + direction[1])
        factor = Fraction(rng.randint(2, 9), rng.randint(1, 4))
        second_shift = _scaled(direction, factor)
        second = (origin[0] + second_shift[0], origin[1] + second_shift[1])
        yield origin, first, second


def test_the_filter_never_names_a_wrong_sign_and_decides_most_random_questions():
    rng = random.Random(20261003)
    decided = unanswered = 0
    for _ in range(1500):
        triple = (_point(rng), _point(rng), _point(rng))
        exact = _exact_orientation(*triple)
        answer = float_filter.orientation_sign(*triple)
        if answer is None:
            unanswered += 1
            continue
        decided += 1
        assert answer == exact
        assert orientation(*triple) == exact
    # Фильтр, молчащий всегда, был бы верен и бесполезен: большинство вопросов обязано решаться.
    assert decided > 10 * max(1, unanswered)


def test_exact_zeros_are_never_decided_by_the_filter():
    rng = random.Random(7)
    for first, second, third in _collinear_triples(rng, 200):
        assert _exact_orientation(first, second, third) == 0
        assert float_filter.orientation_sign(first, second, third) is None
        assert orientation(first, second, third) == 0


def test_a_filter_without_error_slack_names_wrong_signs_on_exact_zeros(monkeypatch):
    """Отрицательный контроль: запас на ошибку — не украшение; без него нули объявляются знаком."""

    monkeypatch.setattr(float_filter, "_SLACK", 0.0)
    monkeypatch.setattr(float_filter, "_FLOOR", 0.0)
    monkeypatch.setattr(float_filter, "_MARGIN", 1.0)
    float_filter.clear_table()
    try:
        rng = random.Random(7)
        wrong = sum(
            1
            for first, second, third in _collinear_triples(rng, 400)
            if float_filter.orientation_sign(first, second, third) not in (None, 0)
        )
    finally:
        float_filter.clear_table()
    assert wrong > 0


def _line_through_origin(y: SqrtSumV1):
    """Тройка `(0, 0), (1, 0), (3, y)`: знак `orientation` равен знаку `y`."""

    zero = SqrtSumV1.zero()
    return (zero, zero), (SqrtSumV1.rational(1), zero), (SqrtSumV1.rational(3), y)


def test_a_cancelling_sum_of_small_roots_is_decided_correctly():
    """Пять корней, сумма ~1e-5 при слагаемых ~1e1: граница ошибки ~1e-14, знак виден и в binary64."""

    y = SqrtSumV1.zero()
    for coefficient, radicand in zip((-15, -12, 1, 5, 8), (2, 3, 5, 7, 11)):
        y = y + SqrtSumV1.radical(coefficient, radicand)
    sign = y.sign()
    assert sign != 0 and abs(float(y.enclosure(64)[0])) < 1e-4
    triple = _line_through_origin(y)
    assert float_filter.orientation_sign(*triple) == sign
    assert orientation(*triple) == sign


def test_a_difference_of_two_huge_close_roots_is_yielded_to_the_exact_path_and_answered_exactly():
    """`sqrt(2q) - sqrt(2q')` для соседних простых порядка 2^100 меньше `ulp` каждого корня: binary64 не решает."""

    import sympy

    first_prime = int(sympy.nextprime(2**100))
    second_prime = int(sympy.nextprime(first_prime))
    near = SqrtSumV1(((2 * first_prime, Fraction(1)),))
    far = SqrtSumV1(((2 * second_prime, Fraction(1)),))
    for sign, y in ((-1, near - far), (1, far - near)):
        triple = _line_through_origin(y)
        assert float_filter.orientation_sign(*triple) is None
        assert orientation(*triple) == _exact_orientation(*triple) == sign


def test_a_radicand_beyond_binary64_is_answered_by_the_exact_path():
    huge = SqrtSumV1(((((2**521) - 1) * ((2**607) - 1), Fraction(1)),))
    zero = SqrtSumV1.zero()
    triple = ((zero, zero), (huge, zero), (huge, SqrtSumV1.rational(1)))
    assert float_filter.orientation_sign(*triple) is None
    assert orientation(*triple) == _exact_orientation(*triple) == 1


def _trapezoid_area(points):
    """Определение удвоенной площади суммой трапеций: ответ веерной формы `doubled_shoelace` обязан быть той же величиной."""

    total = SqrtSumV1.zero()
    for index, (x0, y0) in enumerate(points):
        x1, y1 = points[(index + 1) % len(points)]
        total = total + (x0 * y1) - (x1 * y0)
    return total


def test_the_fan_form_of_the_shoelace_is_the_same_canonical_value_as_the_trapezoid_sum():
    rng = random.Random(13)
    for _ in range(200):
        points = tuple(_point(rng) for _ in range(rng.randint(3, 9)))
        fan, trapezoids = doubled_shoelace(points), _trapezoid_area(points)
        assert fan == trapezoids and fan.terms == trapezoids.terms
        assert [type(c) for _m, c in fan.terms] == [type(c) for _m, c in trapezoids.terms]
    # Меньше трёх точек площади нет, как и раньше.
    assert doubled_shoelace((_point(rng), _point(rng))).is_zero
    assert doubled_shoelace(()).is_zero


def test_the_shoelace_sign_equals_the_exact_area_sign():
    rng = random.Random(11)
    decided = 0
    for _ in range(300):
        size = rng.randint(3, 8)
        points = tuple(_point(rng) for _ in range(size))
        exact = doubled_shoelace(points).sign()
        assert shoelace_sign(points) == exact
        answer = float_filter.polygon_sign(points)
        assert answer in (None, exact)
        decided += int(answer is not None)
    assert decided > 250


def test_a_degenerate_polygon_is_not_decided():
    rng = random.Random(3)
    first, second, third = next(iter(_collinear_triples(rng, 1)))
    assert float_filter.polygon_sign((first, second, third)) is None
    assert shoelace_sign((first, second, third)) == 0


def test_the_table_returns_the_same_centre_for_the_same_object_and_never_confuses_two_values():
    rng = random.Random(5)
    values = [_value(rng, 3) for _ in range(50)]
    first = [float_filter.centre_and_bound(value) for value in values]
    float_filter.clear_table()
    second = [float_filter.centre_and_bound(value) for value in values]
    assert first == second
    for value, (centre, bound) in zip(values, first):
        enclosure_low, enclosure_high = value.enclosure(64)
        assert enclosure_low - Fraction(bound) <= Fraction(centre) <= enclosure_high + Fraction(bound)
