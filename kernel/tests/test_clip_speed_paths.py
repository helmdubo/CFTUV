"""Быстрые пути резки: каждый либо доказывает, либо уступает точному пути, и ответ тот же побитово.

Резка (`materialize.clip`) считает знаки, пересечения и площади на `SqrtSumV1` под бюджетом. Дорогое в ней — не
сама геометрия, а точная арифметика, и сокращена она тремя фильтрами и несколькими ядрами, которые здесь проверяются
ПОВЕДЕНИЕМ:

* фильтры стадии (`ClipStageV1`): знак точки у ребра области — целыми у рациональной точки и float-оценкой с границей
  ошибки у прочих (`float_filter.line_estimate`), точное значение не строится, пока оно не нужно; пересечение отрезка с
  прямой не зависит от области и обхода, поэтому считается один раз на пару «отрезок, прямая» (`ClipStageV1.crossings`);
  площадь кусков, равная площади многоугольника, доказывается границей, а не суммой произведений
  (`_covers_by_construction`);
* ядра `exact_sqrt_sum`: `oriented_sum`, `product_added`, `sum_of_products`, целочисленное `divided_by` и память пар
  радикандов — то же каноническое значение, члены и типы коэффициентов, те же вызовы `prime_support` (цена).

Эталон ядер — прежние цепочки `SqrtSumV1` (копии ниже). Эталон фильтров стадии — сама стадия с выключенными фильтрами
(`reference_mode`): она исполняет прежние алгоритмы, и ответ обязан совпасть на случайных многоугольниках с радикалами,
невыпуклых, с вершинами в допуске от внутренних рёбер и вне триангуляции.
"""

from __future__ import annotations

import math
import random
from fractions import Fraction
from math import gcd

import pytest

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope import float_filter
from cftuv_envelope import _radicand_products as radicand_products
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1, exact_work_budget, reset_factorization_memory
from cftuv_envelope.exact_sqrt_sum_fused import oriented_sum, product_added, sum_of_products
from cftuv_envelope.materialize import clip, clip_memo
from cftuv_envelope.materialize.clip import ClipStageV1
from cftuv_envelope.materialize.frames import MaterializationRefusal
from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1, _edge_value
from cftuv_envelope.wavefront.faces import doubled_shoelace


@pytest.fixture(autouse=True)
def _cold_state():
    reset_factorization_memory()
    yield
    reset_factorization_memory()


def budget():
    return exact_work_budget(stage="MATERIALIZE_TEST", domain_id="clip-speed")


# --------------------------------------------------------------------------
# Эталон: прежние цепочки на `SqrtSumV1` и `Fraction`
# --------------------------------------------------------------------------


def ref_edge_value(start, end, point):
    dx, dy = end[0] - start[0], end[1] - start[1]
    return point[1].scaled(dx) - point[0].scaled(dy) + SqrtSumV1.rational(dy * start[0] - dx * start[1])


def ref_divided_by(numerator, denominator, spent):
    while True:
        rational_part = denominator.as_rational()
        if rational_part is not None:
            return numerator.scaled(Fraction(1) / rational_part)
        prime = canon._pick_prime(denominator.as_map(), spent)
        outside, inside = canon._split_by_prime(denominator.as_map(), prime)
        root = SqrtSumV1.radical(1, prime, spent)
        conjugate = SqrtSumV1._from_map(outside) - (SqrtSumV1._from_map(inside) * root)
        numerator = numerator * conjugate
        denominator = denominator * conjugate


def ref_shoelace(points):
    total = SqrtSumV1.zero()
    origin_x, origin_y = points[0]
    previous_x, previous_y = points[1][0] - origin_x, points[1][1] - origin_y
    for index in range(2, len(points)):
        next_x, next_y = points[index][0] - origin_x, points[index][1] - origin_y
        total = total + (previous_x * next_y - previous_y * next_x)
        previous_x, previous_y = next_x, next_y
    return total


def snapshot(value):
    """Члены с ТИПАМИ коэффициентов: `Fraction(3, 1)` и `3` различаются."""

    return tuple(
        (radicand, type(coefficient).__name__, coefficient.numerator, coefficient.denominator)
        for radicand, coefficient in value.terms
    )


def random_value(rng, *, nonzero=False, size=(0, 1, 2, 3, 4, 5, 8)):
    radicands = set()
    for _ in range(rng.choice(size)):
        radicand = 1
        for prime in rng.sample((2, 3, 5, 7, 11, 13), rng.randint(0, 3)):
            radicand *= prime
        radicands.add(radicand)
    terms = []
    for radicand in sorted(radicands):
        denominator = rng.choice((1, 1, 2, 3, 6, 7, 10**9 + 7))
        numerator = rng.choice((-7, -3, -1, 1, 2, 5, 11, rng.randint(-(10**12), 10**12) or 1))
        terms.append((radicand, Fraction(numerator, denominator)))
    if nonzero and not terms:
        terms.append((1, Fraction(3, 2)))
    return SqrtSumV1(tuple(terms))


# --------------------------------------------------------------------------
# Ядра `exact_sqrt_sum`
# --------------------------------------------------------------------------


def test_oriented_sum_is_the_scaled_chain_with_the_same_terms_and_coefficient_types():
    rng = random.Random(7)
    for _ in range(300):
        x, y = random_value(rng), random_value(rng)
        step_x, step_y, offset = (rng.choice((0, 1, -1, 3, -7, rng.randint(-(10**6), 10**6))) for _ in range(3))
        expected = y.scaled(step_x) - x.scaled(step_y) + SqrtSumV1.rational(offset)
        assert snapshot(oriented_sum(x, y, step_x, step_y, offset)) == snapshot(expected)


def test_the_edge_value_of_an_integer_edge_is_the_chain_and_a_fractional_edge_keeps_the_chain():
    rng = random.Random(11)
    for _ in range(200):
        point = (random_value(rng), random_value(rng))
        start = tuple(Fraction(rng.randint(-50, 50)) for _ in range(2))
        end = tuple(Fraction(rng.randint(-50, 50)) for _ in range(2))
        assert snapshot(_edge_value(start, end, point)) == snapshot(ref_edge_value(start, end, point))
        half = tuple(Fraction(rng.randint(-50, 50), rng.choice((2, 3, 7))) for _ in range(2))
        assert snapshot(_edge_value(half, end, point)) == snapshot(ref_edge_value(half, end, point))
        assert snapshot(_edge_value((1, 2), (4, 6), point)) == snapshot(ref_edge_value((1, 2), (4, 6), point))


def test_product_added_is_the_sum_with_the_same_terms_and_the_base_only_terms_stay_the_same_objects():
    rng = random.Random(3)
    for _ in range(300):
        base, left, right = random_value(rng), random_value(rng), random_value(rng)
        got, expected = product_added(base, left, right), base + left * right
        assert snapshot(got) == snapshot(expected)
        product = dict((left * right).terms)
        for radicand, coefficient in base.terms:
            if radicand not in product:
                assert dict(got.terms)[radicand] is coefficient


def test_product_added_keeps_a_non_fraction_coefficient_of_the_base_exactly_as_the_sum_does():
    base = SqrtSumV1(((1, 5), (2, Fraction(1, 3))))  # целый коэффициент: так построена величина, а не результат арифметики
    left, right = SqrtSumV1(((3, Fraction(2)),)), SqrtSumV1(((3, Fraction(1, 2)), (1, Fraction(1))))
    assert snapshot(product_added(base, left, right)) == snapshot(base + left * right)


def test_a_sum_of_products_is_the_chain_with_one_normalization():
    rng = random.Random(5)
    for _ in range(150):
        products = [(random_value(rng), random_value(rng), rng.choice((1, -1))) for _ in range(rng.randint(0, 6))]
        expected = SqrtSumV1.zero()
        for left, right, sign in products:
            expected = expected + (left * right if sign > 0 else SqrtSumV1.zero() - left * right)
        assert snapshot(sum_of_products(products)) == snapshot(expected)


def test_the_shoelace_fan_is_the_sum_of_triangles():
    rng = random.Random(9)
    for size in (3, 4, 5, 7):
        for _ in range(40):
            points = tuple((random_value(rng), random_value(rng)) for _ in range(size))
            assert snapshot(doubled_shoelace(points)) == snapshot(ref_shoelace(points))


def test_the_integer_division_is_the_conjugate_chain_with_the_same_value_types_and_work(monkeypatch):
    rng = random.Random(13)
    original = canon.prime_support
    for _ in range(120):
        numerator, denominator = random_value(rng), random_value(rng, nonzero=True)
        calls = {"new": [], "ref": []}

        def spy(log):
            def wrapped(radicand, spent=None):
                log.append(radicand)
                return original(radicand, spent)

            return wrapped

        spent_ref, spent_new = budget(), budget()
        reset_factorization_memory()
        with monkeypatch.context() as patch:
            patch.setattr(canon, "prime_support", spy(calls["ref"]))
            expected = ref_divided_by(numerator, denominator, spent_ref)
        reset_factorization_memory()
        with monkeypatch.context() as patch:
            patch.setattr(canon, "prime_support", spy(calls["new"]))
            got = numerator.divided_by(denominator, spent_new)
        assert snapshot(got) == snapshot(expected)
        assert calls["new"] == calls["ref"]
        assert spent_new.counters() == spent_ref.counters()


def test_the_radicand_product_memory_is_a_pure_function_and_stays_bounded(monkeypatch):
    rng = random.Random(17)
    left = [(m, rng.randint(-9, 9) or 1) for m in (1, 6, 35, 143, 2 * 3 * 5 * 7 * 11)]
    right = [(m, rng.randint(-9, 9) or 1) for m in (1, 10, 21, 143, 13 * 11 * 3)]
    expected: dict = {}
    for a, x in left:
        for b, y in right:
            common = gcd(a, b)
            radicand = (a // common) * (b // common)
            expected[radicand] = expected.get(radicand, 0) + x * y * common
    for warm in (False, True, True):
        if not warm:
            radicand_products.clear_products()
        merged: dict = {}
        radicand_products.accumulate_products(merged, left, right)
        assert merged == expected
    radicand_products.clear_products()
    monkeypatch.setattr(radicand_products, "_RADICAND_PRODUCTS_LIMIT", 6)
    merged = {}
    radicand_products.accumulate_products(merged, left, right, 3)
    assert merged == {radicand: value * 3 for radicand, value in expected.items()}
    assert len(radicand_products._RADICAND_PRODUCTS) <= 6 + 2
    reset_factorization_memory()
    assert not radicand_products._RADICAND_PRODUCTS


# --------------------------------------------------------------------------
# Фильтр знака у прямой: граница ошибки и приговор
# --------------------------------------------------------------------------


def exact_orientation(point, start, step):
    return (
        point[1].scaled(step[0])
        - point[0].scaled(step[1])
        + SqrtSumV1.rational(step[1] * start[0] - step[0] * start[1])
    )


def test_the_line_estimate_never_contradicts_the_exact_value():
    rng = random.Random(19)
    proven = undecided = 0
    for case in range(1500):
        scale = rng.choice((1, 10, 1000, 10**6))
        coordinates = []
        for _ in range(2):
            terms = {1: Fraction(rng.randint(-scale, scale), rng.choice((1, 3, 7)))}
            for _ in range(rng.randint(0, 3)):
                radicand = rng.choice((2, 3, 5, 6, 7, 11, 10**9 + 7))
                terms[radicand] = Fraction(rng.randint(-scale, scale), rng.choice((1, 5, 10**9)))
            coordinates.append(SqrtSumV1(tuple(sorted((m, c) for m, c in terms.items() if c))))
        start = (rng.randint(-100, 100), rng.randint(-100, 100))
        step = (rng.randint(-100, 100), rng.randint(-100, 100))
        if case % 5 == 0:
            # Точка НА прямой точно либо в долях ячейки от неё: оценка не вправе доказать знак там, где ответ нуль.
            along = Fraction(rng.choice((0, 1, 2, 7)), 3)
            offset = rng.choice((0, 0, Fraction(1, 10**12), Fraction(-1, 10**30)))
            coordinates = [
                SqrtSumV1.rational(start[0] + step[0] * along + offset * step[1]),
                SqrtSumV1.rational(start[1] + step[1] * along - offset * step[0]),
            ]
        point = tuple(coordinates)
        float_filter.clear_table()
        estimate = float_filter.line_estimate(point, float(start[0]), float(start[1]), float(step[0]), float(step[1]))
        exact = exact_orientation(point, start, step)
        if estimate is None:
            undecided += 1
            continue
        value, bound = estimate
        # Граница ошибки: точное значение лежит в `[value - bound, value + bound]` (оболочка точного шире на 2^-120).
        low, high = exact.enclosure(120)
        assert low <= Fraction(value) + Fraction(bound) and high >= Fraction(value) - Fraction(bound)
        if abs(value) > bound:
            proven += 1
            assert exact.sign(budget=budget()) == (1 if value > 0 else -1)
        else:
            undecided += 1
    assert proven > 300 and undecided > 100


def test_the_line_estimate_declines_a_coordinate_binary64_cannot_carry():
    huge = SqrtSumV1(((1, Fraction(10**400)),))
    tiny = SqrtSumV1(((1, Fraction(1, 10**400)),))
    float_filter.clear_table()
    assert float_filter.line_estimate((huge, huge), 0.0, 0.0, 1.0, 1.0) is None
    assert float_filter.line_estimate((tiny, tiny), 0.0, 0.0, 1.0, 1.0) is None


# --------------------------------------------------------------------------
# Стадия: те же ответы без фильтров и с ними
# --------------------------------------------------------------------------

CELL = 16  # единиц решётки карты на ячейку сетки
SIZE = 4  # ячеек по стороне
POOL = (2, 3, 5, 6, 7, 10, 11, 13)


class _NoMemory(dict):
    """Словарь, который ничего не помнит: пересечения считаются заново, как до памяти пар."""

    def get(self, key, default=None):
        return default

    def __setitem__(self, key, value):
        pass


def reference_mode(patch):
    """Стадия исполняет прежние алгоритмы: точный знак всегда, пересечение заново, площадь суммой произведений."""

    patch.setattr(ClipStageV1, "_edge_constants", lambda self, ti, index: None)
    patch.setattr(ClipStageV1, "_covers_by_construction", lambda self, nodes, pieces: False)
    original = ClipStageV1.__init__

    def init(self, *args, **kwargs):
        original(self, *args, **kwargs)
        self.crossings = _NoMemory()

    patch.setattr(ClipStageV1, "__init__", init)


def grid_lift(rng, rough=Fraction(1, 80), holes=()):
    """Сетка `SIZE x SIZE` ячеек: у каждой грань из двух треугольников, поверхность неровная (хорда над диагональю).

    `holes` — ячейки, которых в карте нет (проём: многоугольник над ним накрыт кусками не весь).
    """

    height = {(i, j): Fraction(rng.randint(-100, 100), 100) * rough for i in range(SIZE + 1) for j in range(SIZE + 1)}

    def corner(i, j):
        return (Fraction(i, 3) + Fraction(1, 7), Fraction(j, 3), height[(i, j)])

    items = []
    for i in range(SIZE):
        for j in range(SIZE):
            if (i, j) in holes:
                continue
            quad = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
            for index, (a, b, c) in enumerate(((0, 1, 2), (0, 2, 3))):
                chart = [(quad[k][0] * CELL, quad[k][1] * CELL) for k in (a, b, c)]
                items.append((f"t{i}_{j}_{index}", chart, [corner(*quad[k]) for k in (a, b, c)], (), f"f{i}_{j}"))
    return SurfaceLiftV1.from_triangles(items, CELL)


def rational(value):
    return SqrtSumV1.rational(Fraction(value))


def radical(coefficient, radicand):
    return SqrtSumV1.radical(Fraction(coefficient), radicand, budget())


def star_polygon(rng, center, radius, count, convex, perturb):
    """Звёздный многоугольник против часовой (углы по возрастанию): выпуклый — на окружности, иначе радиусы случайны."""

    angles = sorted(rng.uniform(0, 2 * math.pi) for _ in range(count))
    if min(b - a for a, b in zip(angles, angles[1:] + [angles[0] + 2 * math.pi])) < 0.45:
        return None
    points = []
    for angle in angles:
        rho = radius if convex else radius * rng.uniform(0.45, 1.0)
        x = Fraction(center[0] + rho * math.cos(angle)).limit_denominator(1000)
        y = Fraction(center[1] + rho * math.sin(angle)).limit_denominator(1000)
        px, py = rational(x), rational(y)
        if perturb:
            px = px + radical(Fraction(rng.randint(-2, 2), 7), rng.choice(POOL))
            py = py + radical(Fraction(rng.randint(-2, 2), 7), rng.choice(POOL))
        points.append((px, py))
    return points


def scenario_polygons(rng):
    """Многоугольники сценария: внутри сетки, у внутреннего ребра (допуск вершины `node:`), за краем сетки (свес)."""

    polygons = []
    for _ in range(rng.randint(3, 6)):
        center = (rng.uniform(8, CELL * SIZE - 8), rng.uniform(8, CELL * SIZE - 8))
        shape = star_polygon(
            rng, center, rng.uniform(2.5, 9.0), rng.randint(3, 6), convex=rng.random() < 0.6, perturb=rng.random() < 0.85
        )
        if shape is not None:
            polygons.append(shape)
    # Прямоугольник вдоль внутреннего ребра `y = 16 m` на доли ячейки от него: знак вершины `node:` обнуляется допуском.
    y = CELL * rng.randint(1, SIZE - 1) + Fraction(rng.randint(1, 40), 100)
    x0 = Fraction(rng.randint(2, 30), 1) + Fraction(1, 3)
    polygons.append(
        [
            (rational(x0), rational(y)),
            (rational(x0 + 11), rational(y) + radical(Fraction(1, 7), 2)),
            (rational(x0 + 11) + radical(Fraction(1, 9), 3), rational(y + 4)),
            (rational(x0), rational(y + 4)),
        ]
    )
    # Многоугольник за краем сетки: площадь кусков не сходится, свес остаётся ушами.
    edge = CELL * SIZE
    polygons.append(
        [
            (rational(edge - 3), rational(5)),
            (rational(edge + 3), rational(5)),
            (rational(edge + 3), rational(11) + radical(Fraction(1, 3), 3)),
            (rational(edge - 3), rational(11)),
        ]
    )
    return polygons


def input_of(polygons_xy, rng):
    """`(points, cycles, polygons)` стадии: вершины `node:<k>`, а у целых точек решётки — `src:<k>`."""

    points, cycles, polygons = {}, [], []
    for polygon in polygons_xy:
        cycle = []
        for xy in polygon:
            lattice = all(axis.as_rational() is not None and axis.as_rational().denominator == 1 for axis in xy)
            key = f"{'src' if lattice else 'node'}:{len(points)}"
            points[key] = xy
            cycle.append((key, xy))
        if rng.random() < 0.4:
            cycle.reverse()
        cycles.append(cycle)
        polygons.append((tuple(key for key, _xy in cycle),))
    return points, cycles, polygons


def cut(lift, points, cycles, polygons):
    plane = lift.bind(budget())
    reset_factorization_memory()
    return clip._cut_by_faces(
        plane,
        budget(),
        points,
        cycles,
        polygons,
        DecalTopologyLawV1.PLANAR_POLYGONS_V1,
        frozenset(),
        [False] * len(polygons),
        None,
    )


def answer_of(lift, points, cycles, polygons):
    """Ответ стадии точным текстом (типы и значения всех чисел) либо названный отказ."""

    try:
        result = cut(lift, points, cycles, polygons)
    except MaterializationRefusal as refusal:
        return ("refused", refusal.outcome.value, str(refusal))
    out: list = []
    clip_memo._encode(result, out)
    return ("".join(out),)


SEEDS = range(16)


def scenario(seed):
    rng = random.Random(1000 + seed)
    lift = grid_lift(rng)
    return (lift, *input_of(scenario_polygons(rng), rng))


@pytest.mark.parametrize("seed", SEEDS)
def test_the_stage_with_every_shortcut_gives_the_answer_of_the_stage_without_them(seed, monkeypatch):
    lift, points, cycles, polygons = scenario(seed)
    fast = answer_of(lift, points, cycles, polygons)
    with monkeypatch.context() as patch:
        reference_mode(patch)
        reference = answer_of(lift, points, cycles, polygons)
    assert fast == reference


def test_the_scenarios_exercise_the_cut_the_tolerance_the_ears_and_the_overhang():
    """Сценарии не вырождены (иначе сравнение выше сравнивало бы пустоту): куски, нули по допуску, уши, свес."""

    seen = {"pieces": 0, "zeroed": 0, "overhang": 0, "ears": 0, "inserted": 0}
    for seed in SEEDS:
        lift, points, cycles, polygons = scenario(seed)
        try:
            counters = dict(cut(lift, points, cycles, polygons).counters)
        except MaterializationRefusal:
            continue  # отказ — тоже ответ, и он сравнивается выше
        seen["pieces"] += counters[clip.PIECES_EMITTED]
        seen["zeroed"] += counters[clip.NODE_SIGNS_ZEROED]
        seen["overhang"] += counters[clip.FACES_OVERHANG]
        seen["ears"] += counters[clip.FACES_CUT_BY_EARS]
        seen["inserted"] += counters[clip.VERTICES_INSERTED]
    assert all(seen.values()), seen


def test_the_closure_proof_is_sound_covers_the_ordinary_cut_and_is_silent_on_the_overhang(monkeypatch):
    """Доказано — значит равно точно; обычные разрезы доказываются; свес не доказывается (его площадь не сходится)."""

    verdicts = []
    original = ClipStageV1._covers_by_construction

    def spy(self, nodes, pieces):
        proof = original(self, nodes, pieces)
        total = SqrtSumV1.zero()
        for _ti, piece, _part in pieces:
            total = total + self._area(piece)
        verdicts.append((proof, (total - self._area(nodes)).is_zero))
        return proof

    monkeypatch.setattr(ClipStageV1, "_covers_by_construction", spy)
    for seed in SEEDS:
        lift, points, cycles, polygons = scenario(seed)
        answer_of(lift, points, cycles, polygons)
    proved = sum(1 for proof, _exact in verdicts if proof)
    closed = sum(1 for _proof, exact in verdicts if exact)
    assert all(exact for proof, exact in verdicts if proof)
    assert proved > 0.6 * closed > 0
    assert any(not exact for _proof, exact in verdicts)


def test_a_polygon_over_a_hole_of_the_map_is_not_closed_and_the_proof_does_not_claim_it(monkeypatch):
    """Граница кусков тут — граница многоугольника ПЛЮС петля проёма: непарные рёбра не исчерпаны путями, доказательства нет."""

    lift = grid_lift(random.Random(37), holes=((1, 1),))
    square = [
        (rational(10) + radical(Fraction(1, 9), 2), rational(10) + radical(Fraction(1, 7), 3)),
        (rational(38) + radical(Fraction(1, 5), 2), rational(10) + radical(Fraction(1, 6), 5)),
        (rational(38) + radical(Fraction(1, 5), 3), rational(38) + radical(Fraction(1, 9), 2)),
        (rational(10) + radical(Fraction(1, 7), 5), rational(38) + radical(Fraction(1, 8), 3)),
    ]
    points, cycles, polygons = input_of([square], random.Random(2))
    result = cut(lift, points, cycles, polygons)
    assert dict(result.counters)[clip.FACES_OVERHANG] == 1 and dict(result.counters)[clip.PIECES_EMITTED] == 0
    fast = answer_of(lift, points, cycles, polygons)
    with monkeypatch.context() as patch:
        reference_mode(patch)
        assert answer_of(lift, points, cycles, polygons) == fast
    verdicts = []
    original = ClipStageV1._covers_by_construction
    monkeypatch.setattr(
        ClipStageV1, "_covers_by_construction", lambda self, nodes, pieces: verdicts.append(original(self, nodes, pieces)) or verdicts[-1]
    )
    cut(lift, points, cycles, polygons)
    assert verdicts == [False]


def test_a_closed_cut_does_not_compute_the_exact_area_of_its_pieces(monkeypatch):
    lift = grid_lift(random.Random(21))
    rectangle = [
        (rational(5) + radical(Fraction(1, 11), 7), rational(5) + radical(Fraction(1, 13), 2)),
        (rational(37) + radical(Fraction(1, 9), 2), rational(5) + radical(Fraction(1, 6), 3)),
        (rational(37) + radical(Fraction(1, 5), 11), rational(21)),
        (rational(5), rational(21) + radical(Fraction(1, 7), 3)),
    ]
    points, cycles, polygons = input_of([rectangle], random.Random(1))

    def refuse(self, nodes):
        raise AssertionError("the exact area was computed for a closed cut")

    monkeypatch.setattr(ClipStageV1, "_area", refuse)
    result = cut(lift, points, cycles, polygons)
    assert dict(result.counters)[clip.PIECES_EMITTED] > 1


def test_a_crossing_is_computed_once_per_segment_and_line_and_counted_every_time(monkeypatch):
    lift = grid_lift(random.Random(23))
    stage = ClipStageV1(lift.bind(budget()), budget(), {})
    first = stage._node((rational(Fraction(53, 10)), rational(Fraction(33, 10))))
    second = stage._node(
        (
            rational(Fraction(374, 10)) + radical(Fraction(1, 9), 5),
            rational(Fraction(205, 10)) + radical(Fraction(1, 8), 3),
        )
    )
    divisions = []
    original = SqrtSumV1.divided_by
    monkeypatch.setattr(
        SqrtSumV1, "divided_by", lambda self, other, spent: divisions.append(1) or original(self, other, spent)
    )
    found = {}
    for ti in range(len(stage.regions)):
        for index in range(len(stage.regions[ti].chart)):
            if stage._sign(first, ti, index) * stage._sign(second, ti, index) < 0:
                before = stage.tally[clip.DIVISIONS]
                forward = stage._crossing(first, second, ti, index)
                backward = stage._crossing(second, first, ti, index)
                assert stage.tally[clip.DIVISIONS] == before + 2  # счёт — по вызовам, как прежде
                assert forward is backward  # тот же узел: точка не зависит от обхода отрезка
                found[(ti, index)] = forward
    assert len(found) >= 4
    # Вычислений меньше вызовов: общее ребро двух областей и обратный обход не пересчитываются.
    assert len(divisions) < len(found)
    # И точка — та же, что у пересечения, посчитанного заново без памяти.
    stage.crossings = _NoMemory()
    for (ti, index), node in found.items():
        assert stage._crossing(first, second, ti, index) is node


def test_the_sign_of_a_far_irrational_point_does_not_build_the_exact_value(monkeypatch):
    lift = grid_lift(random.Random(29))
    plane = lift.bind(budget())
    stage = ClipStageV1(plane, budget(), {})
    rng = random.Random(31)
    while True:
        x, y = rng.uniform(5, 58), rng.uniform(5, 58)
        # Дальше допуска от прямой каждого ребра области (длина ребра до 23): допуск вершины `node:` её не обнуляет.
        if all(
            abs(
                (item.chart[(i + 1) % 3][0] - item.chart[i][0]) * (y - item.chart[i][1])
                - (item.chart[(i + 1) % 3][1] - item.chart[i][1]) * (x - item.chart[i][0])
            )
            > 2.5 * 23
            for item in stage.regions
            for i in range(3)
        ):
            break
    node = stage._node(
        (
            rational(Fraction(x).limit_denominator(100)) + radical(Fraction(1, 3), 2),
            rational(Fraction(y).limit_denominator(100)) + radical(Fraction(1, 5), 3),
        )
    )
    node.key = "node:0"
    calls = []
    original = plane.line_value
    monkeypatch.setattr(plane, "line_value", lambda *args: calls.append(args) or original(*args))
    signs = [
        stage._sign(node, ti, index) for ti in range(len(stage.regions)) for index in range(len(stage.regions[ti].chart))
    ]
    assert any(signs) and not calls  # знаки решены без значения: оно строится только по требованию (`_value`)
    assert stage.tally[clip.PREDICATES] == len(signs)
