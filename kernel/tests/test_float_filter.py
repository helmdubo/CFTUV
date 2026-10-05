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
4. краевые значения (радикант за пределами `float`) уходят в точный путь, а не в исключение;
5. фильтр тождества аффинности (`affine_map_violated`, закон `SILHOUETTE_TOPOLOGY_V1`) называет только доказанное НАРУШЕНИЕ:
   на точно аффинных значениях он молчит всегда, а там, где он говорит «нарушено», точное тождество
   `tessellate.uv_vertex_on_affine_map` согласно.
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


def _subnormal_coefficient_case():
    """Аудит 0facffe: коэффициент `C` — денормал (ошибка `float(Fraction)` абсолютна до 2^-1075), `sqrt(M)` ~ 2^504
    разгоняет её далеко за наименьшее нормальное число. Точный определитель `C*C2*M - 1 = 2^-20 > 0`."""

    import sympy

    modulus = int(sympy.nextprime(2**1009))
    coefficient = Fraction(12898, 100 * 2**1075)  # (2^7 + 0.98) * 2^-1075
    assert 0 < coefficient < Fraction(1, 2**1022)
    partner = (1 + Fraction(1, 2**20)) / (coefficient * modulus)
    zero = SqrtSumV1.zero()
    one = SqrtSumV1.rational(1)
    return (
        (zero, zero),
        (SqrtSumV1(((modulus, coefficient),)), one),
        (one, SqrtSumV1(((modulus, partner),))),
    )


def test_a_subnormal_coefficient_is_never_read_in_binary64_and_the_exact_sign_is_kept():
    triple = _subnormal_coefficient_case()
    assert _exact_orientation(*triple) == 1
    assert float_filter.centre_and_bound(triple[1][0]) is None
    assert float_filter.orientation_sign(*triple) is None
    assert orientation(*triple) == 1
    assert float_filter.polygon_sign(triple) is None
    assert shoelace_sign(triple) == doubled_shoelace(triple).sign() == 1


def test_products_that_underflow_carry_the_format_floor_in_their_bound():
    """Все координаты ~1e-160, произведения ~1e-321 — денормалы, где относительная граница исчезает вместе с ними."""

    tiny = Fraction(1, 10**160)
    zero = SqrtSumV1.zero()
    first, second, third = (
        (zero, zero),
        (SqrtSumV1.radical(tiny, 2), SqrtSumV1.radical(tiny, 3)),
        (SqrtSumV1.radical(tiny, 5), SqrtSumV1.radical(tiny, 7)),
    )
    exact = _exact_orientation(first, second, third)
    assert exact == -1
    assert float_filter.orientation_sign(first, second, third) is None
    assert float_filter.polygon_sign((first, second, third)) is None
    assert orientation(first, second, third) == exact


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


def _affine_case(rng: random.Random, perturbation):
    """Четыре точки и значения `(s, r)` на них: ТОЧНО аффинная функция положения плюс `perturbation` в четвёртой вершине."""

    from cftuv_envelope.materialize.tessellate import affine_frame

    while True:
        points = {key: _point(rng) for key in range(4)}
        if _exact_orientation(points[0], points[1], points[2]) != 0:
            break
    slopes = [(Fraction(rng.randint(-5, 5), rng.randint(1, 4)), Fraction(rng.randint(-5, 5), rng.randint(1, 4))) for _ in range(2)]
    origin = [_value(rng, rng.randint(0, 2)) for _ in range(2)]
    values = {}
    for key, point in points.items():
        values[key] = tuple(
            origin[component]
            + (point[0] - points[0][0]).scaled(slopes[component][0])
            + (point[1] - points[0][1]).scaled(slopes[component][1])
            for component in range(2)
        )
    if perturbation is not None:
        values[3] = (values[3][0] + perturbation, values[3][1])
    return points, values, affine_frame(points, (0, 1, 2))


def test_the_affine_filter_never_claims_a_violation_on_an_exactly_affine_map():
    from cftuv_envelope.materialize.tessellate import uv_vertex_on_affine_map

    rng = random.Random(20261005)
    for _ in range(300):
        points, values, frame = _affine_case(rng, None)

        assert uv_vertex_on_affine_map(points, values, (0, 1, 2), frame, 3)
        assert not float_filter.affine_map_violated(points, values, (0, 1, 2), 3)


def test_the_affine_filter_names_only_violations_the_exact_identity_confirms_and_decides_the_clear_ones():
    from cftuv_envelope.materialize.tessellate import uv_vertex_on_affine_map

    rng = random.Random(20261006)
    decided = clear = 0
    for index in range(600):
        size = (Fraction(1, 2), Fraction(1, 1000), Fraction(1, 10**9), Fraction(1, 10**14))[index % 4]
        points, values, frame = _affine_case(rng, SqrtSumV1.rational(size))
        exact_on_map = uv_vertex_on_affine_map(points, values, (0, 1, 2), frame, 3)
        claimed = float_filter.affine_map_violated(points, values, (0, 1, 2), 3)

        assert not (claimed and exact_on_map)
        assert not exact_on_map  # возмущение рациональным числом всегда выводит вершину с карты
        if size >= Fraction(1, 1000):
            clear += 1
            decided += int(claimed)
    assert decided > clear // 2  # крупные нарушения фильтр обязан решать, иначе он пустышка


def test_the_affine_filter_yields_when_a_number_is_beyond_binary64():
    huge = SqrtSumV1.radical(Fraction(1), 10**400)
    points = {0: (huge, huge), 1: (huge + SqrtSumV1.rational(1), huge), 2: (huge, huge + SqrtSumV1.rational(1)), 3: (huge, huge)}
    values = {key: (SqrtSumV1.rational(Fraction(key)), SqrtSumV1.rational(Fraction(0))) for key in range(4)}

    assert not float_filter.affine_map_violated(points, values, (0, 1, 2), 3)


def test_an_affine_filter_without_error_slack_names_false_violations(monkeypatch):
    """Границы настоящие: без запаса на ошибку (`_SLACK = 0`, `_MARGIN = 1`) тот же корпус точно аффинных карт ловит ложное нарушение."""

    rng = random.Random(20261005)
    float_filter.clear_table()
    monkeypatch.setattr(float_filter, "_SLACK", 0.0)
    monkeypatch.setattr(float_filter, "_MARGIN", 1.0)
    monkeypatch.setattr(float_filter, "_FLOOR", 0.0)
    false_claims = 0
    for _ in range(300):
        points, values, _frame = _affine_case(rng, None)
        false_claims += int(float_filter.affine_map_violated(points, values, (0, 1, 2), 3))
    float_filter.clear_table()

    assert false_claims > 0
