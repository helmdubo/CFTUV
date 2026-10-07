"""Нативные числа (`native/cftuv-core`: `num`, `rat`, `pyfloat`, `sqrt_sum`, `fused`, `products`, `float_filter`, `codec`) равны эталону на Python ПОБИТОВО.

Эталон — ядро питона (`exact_sqrt_sum`, `exact_sqrt_sum_fused`, `float_filter`, `math`, `fractions`); его не правят. Целый сценарий операций
уходит в расширение ОДНИМ вызовом (`cftuv_native.run_number_ops`), и каждый ответ сверяется с эталоном точно (`numbers_oracle.same`):
значение, ТИП коэффициента (`int` не `Fraction`, хотя `==` их не различает), `float` по битам, поимённый отказ (`OverflowError`,
`ZeroDivisionError`, `ValueError`), приращение счётчиков знака у `_filtered_sign`.

Операнды трёх родов: 1. СЛУЧАЙНЫЕ структурные (радиканды — произведения простых из малой вселенной, в том числе простых за `i128`; коэффициенты
от 1 бита до 2000+ бит, с отрицательными, `int` и `Fraction`); 2. НАСТОЯЩИЕ — вызовы, которые ядро делает на своих домене-развёртках (запись
аргументов у подмены), и значения из корпуса `E:\\cftuv_native_corpus\\` (если он собран); 3. КРАЕВЫЕ: границы округления `float`, переполнение,
денормалы, нули, сокращение членов.

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`): чистый клон на системном Python зелёный.
"""

from __future__ import annotations

import importlib.util
import itertools
import math
import os
import random
import sys
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка нативных чисел с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import field_tier, skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "coverage")

from cftuv_native import codec  # noqa: E402
from cftuv_native import numbers_oracle as oracle  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402

PRIMES = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97)
#: Простые Мерсенна: за `i128` (127 бит), и их произведения — радиканды в сотни бит.
BIG_PRIMES = (2**61 - 1, 2**89 - 1, 2**107 - 1, 2**127 - 1)
BIT_SIZES = (1, 1, 2, 3, 5, 8, 16, 31, 32, 33, 53, 54, 63, 64, 65, 100, 127, 128, 129, 200, 256, 300, 600, 1000, 2100)
SMALL_DENOMINATORS = (1, 1, 1, 2, 3, 4, 5, 6, 8, 10, 12, 24, 60)


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон пишет в процессные счётчики и память ядра; тест их не оставляет."""

    before = dict(exact.SIGN_COUNTS)
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(before)
    oracle.clear_oracle_state()


#: Сколько операций сверено за прогон: печатается в конце модуля, чтобы «зелёный» не был пустым.
COMPARED = [0]


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(f"native numbers: {COMPARED[0]} operations compared with the Python oracle")


# --------------------------------------------------------------------------
# Сверка пакета операций с эталоном
# --------------------------------------------------------------------------


def _show(arguments) -> str:
    return "(" + ", ".join(oracle.describe(argument) for argument in arguments) + ")"


def check(ops, *, strict: bool = True, memo: bool = True) -> int:
    """Прогоняет сценарий через расширение и эталон; расхождение называет операцию и аргументы. Возвращает число сверенных."""

    got = cftuv_native.run_number_ops(ops, strict=strict, memo=memo)
    want = oracle.run_oracle(ops)
    assert len(got) == len(want) == len(ops)
    mismatches = []
    compared = 0
    for (name, arguments), expected, actual in zip(ops, want, got):
        if expected is oracle.SKIPPED:
            continue
        compared += 1
        if not oracle.same(expected, actual):
            mismatches.append(f"{name}{_show(arguments)}\n  эталон:    {oracle.describe(expected)}\n  нативное: {oracle.describe(actual)}")
    assert not mismatches, f"{len(mismatches)} из {compared} расходятся с эталоном:\n" + "\n".join(mismatches[:4])
    COMPARED[0] += compared
    return compared


# --------------------------------------------------------------------------
# Случайные структурные операнды
# --------------------------------------------------------------------------


class Gen:
    def __init__(self, seed: int) -> None:
        self.rng = random.Random(seed)

    def integer(self, bits: int | None = None, *, signed: bool = True) -> int:
        bits = bits or self.rng.choice(BIT_SIZES)
        value = self.rng.getrandbits(bits) | (1 << (bits - 1))
        return -value if signed and self.rng.random() < 0.5 else value

    def coefficient(self, int_chance: float = 0.3, max_bits: int = 2100):
        sizes = [size for size in BIT_SIZES if size <= max_bits]
        if self.rng.random() < int_chance:
            return self.integer(self.rng.choice(sizes))
        numerator = self.integer(self.rng.choice(sizes))
        if self.rng.random() < 0.4:
            denominator = self.rng.choice(SMALL_DENOMINATORS)
        else:
            denominator = self.integer(self.rng.choice(sizes), signed=False)
        return Fraction(numerator, denominator)

    def factor(self):
        """Множитель `scaled`: `int` или `Fraction`, в том числе ноль и единицы."""

        roll = self.rng.random()
        if roll < 0.08:
            return self.rng.choice((0, 1, -1, Fraction(0), Fraction(1), Fraction(-1)))
        return self.coefficient(0.5, 300)

    def universe(self, big: bool = False) -> list:
        primes = self.rng.sample(PRIMES, self.rng.randint(3, 8))
        if big:
            primes += self.rng.sample(BIG_PRIMES, self.rng.randint(1, 3))
        radicands = {1}
        reachable = 1 + sum(math.comb(len(primes), size) for size in range(1, min(4, len(primes)) + 1))
        while len(radicands) < min(12, reachable):
            chosen = self.rng.sample(primes, self.rng.randint(1, min(4, len(primes))))
            radicands.add(math.prod(chosen))
        return sorted(radicands)

    def sum(self, radicands: list, max_terms: int = 8, max_bits: int = 2100, int_chance: float = 0.3) -> SqrtSumV1:
        terms = min(self.rng.randint(0, max_terms), len(radicands))
        if max_bits > 300:
            terms = min(terms, 4)
        chosen = sorted(self.rng.sample(radicands, terms))
        return SqrtSumV1(tuple((radicand, self._nonzero(int_chance, max_bits)) for radicand in chosen))

    def _nonzero(self, int_chance: float, max_bits: int):
        while True:
            value = self.coefficient(int_chance, max_bits)
            if value:
                return value

    def related(self, source: SqrtSumV1) -> SqrtSumV1:
        """Операнд, который сокращается, делит радиканды или отличается одним членом с `source`."""

        roll = self.rng.random()
        terms = list(source.terms)
        if roll < 0.25:
            return -source
        if roll < 0.45 or not terms:
            return source
        if roll < 0.7:
            index = self.rng.randrange(len(terms))
            radicand, coefficient = terms[index]
            terms[index] = (radicand, coefficient + (1 if type(coefficient) is int else Fraction(1, 3)))
            return SqrtSumV1(tuple((m, c) for m, c in terms if c))
        return SqrtSumV1(tuple(terms[: self.rng.randint(0, len(terms))]))


def _items(gen: Gen, radicands: list) -> list:
    chosen = sorted(gen.rng.sample(radicands, gen.rng.randint(0, min(6, len(radicands)))))
    return [[radicand, gen.integer(gen.rng.choice((2, 8, 40, 130, 300)))] for radicand in chosen]


def _bits(gen: Gen) -> int:
    return gen.rng.choice((0, 1, 2, 5, 16, 63, 64, 65, 128, 200))


def _unary_and_binary_sum_ops(gen: Gen, pool: list, radicands: list) -> list:
    a, b = gen.rng.choice(pool), gen.rng.choice(pool)
    b = gen.related(a) if gen.rng.random() < 0.3 else b
    fa, fb = gen.factor(), gen.factor()
    bits = _bits(gen)
    return [
        ("SUM_ADD", (a, b)),
        ("SUM_SUB", (a, b)),
        ("SUM_NEG", (a,)),
        ("SUM_SCALED", (a, fa)),
        ("SUM_SCALED_DIFFERENCE", (a, fa, b, fb)),
        ("SUM_DIFFERENCE_IS_ZERO", (a, b)),
        ("SUM_MUL", (a, b)),
        ("SUM_IS_ZERO", (a,)),
        ("SUM_IS_RATIONAL", (a,)),
        ("SUM_AS_RATIONAL", (a,)),
        ("SUM_ENCLOSURE", (a, bits)),
        ("SUM_CERTIFIED_SIGN", (a, bits)),
        ("INTEGER_FORM", (a,)),
        ("INTEGER_ENCLOSURE", (_items(gen, radicands), bits)),
        ("INTEGER_CERTIFIED_SIGN", (_items(gen, radicands), bits)),
        ("SCALED_DIFFERENCE_PARTS", (a, fa, b, fb)),
        ("MULTIPLY_INTEGER_ITEMS", (_items(gen, radicands), _items(gen, radicands))),
        ("FILTERED_SIGN", (_items(gen, radicands), bits)),
        ("DIFFERENCE_FILTERED_SIGN", (a, b)),
    ]


def _reduction_and_division_ops(gen: Gen, pool: list, radicands: list) -> list:
    a, b = gen.rng.choice(pool), gen.rng.choice(pool)
    items = _items(gen, radicands)
    numerator_form = exact._integer_form(a.terms)
    denominator_form = exact._integer_form(b.terms)
    rational = [[1, gen.integer(gen.rng.choice((2, 40, 300)))]] if gen.rng.random() < 0.8 else []
    return [
        ("REDUCED_FORM", (gen.integer(gen.rng.choice((2, 40, 300)), signed=False), items)),
        ("REDUCED_FORM", (numerator_form[0], [list(item) for item in numerator_form[1]])),
        (
            "SCALED_BY_RECIPROCAL",
            (numerator_form[0], [list(item) for item in numerator_form[1]], gen.integer(gen.rng.choice((2, 40, 300)), signed=False), rational),
        ),
        ("SCALED_BY_RECIPROCAL", (denominator_form[0], items, denominator_form[0], [list(item) for item in denominator_form[1]][:1])),
    ]


def _fused_ops(gen: Gen, pool: list) -> list:
    x, y, base, left, right = (gen.rng.choice(pool) for _ in range(5))
    steps = [gen.integer(gen.rng.choice((1, 4, 31, 70))) for _ in range(3)]
    steps = [0 if gen.rng.random() < 0.15 else step for step in steps]
    products = [[gen.rng.choice(pool), gen.rng.choice(pool), gen.rng.choice((1, -1, 1, -1, 2, 0))] for _ in range(gen.rng.randint(0, 4))]
    return [
        ("ORIENTED_SUM", (x, y, steps[0], steps[1], steps[2])),
        ("PRODUCT_ADDED", (base, left, right)),
        ("PRODUCT_ADDED", (base, gen.related(base), gen.related(left))),
        ("SUM_OF_PRODUCTS", (products,)),
    ]


def _pool(gen: Gen, radicands: list, size: int, max_bits: int, max_terms: int, int_chance: float = 0.3) -> list:
    pool = [gen.sum(radicands, max_terms, max_bits, int_chance) for _ in range(size)]
    pool.append(SqrtSumV1(()))
    pool.append(SqrtSumV1.rational(gen.coefficient(0.5, 100)))
    return pool


@pytest.mark.parametrize("seed", range(6))
def test_random_sums_of_every_size_equal_the_oracle(seed):
    gen = Gen(1000 + seed)
    compared = 0
    for round_index in range(12):
        radicands = gen.universe(big=round_index % 3 == 0)
        max_bits = (40, 200, 600, 2100)[round_index % 4]
        pool = _pool(gen, radicands, 8, max_bits, 6)
        ops = []
        for _ in range(6):
            ops += _unary_and_binary_sum_ops(gen, pool, radicands)
            ops += _reduction_and_division_ops(gen, pool, radicands)
            ops += _fused_ops(gen, pool)
        compared += check(ops)
        pool.extend(result for result in oracle.run_oracle(ops) if type(result) is SqrtSumV1)
    assert compared > 1000


def _compact(value: SqrtSumV1, limit: int = 400) -> bool:
    """Результат, который ещё можно кормить дальше: рост разрядов от цепочки к цепочке иначе взрывается."""

    return len(value.terms) <= 8 and all(max(Fraction(c).numerator.bit_length(), Fraction(c).denominator.bit_length()) <= limit for _, c in value.terms)


def test_chained_results_keep_their_coefficient_types():
    """Результат одной операции входит в следующую: тип `int`/`Fraction` каждого члена переживает цепочку."""

    gen = Gen(77)
    radicands = gen.universe()
    pool = _pool(gen, radicands, 10, 100, 5, int_chance=0.9)
    int_terms = fraction_terms = 0
    for _ in range(40):
        ops = []
        for _ in range(8):
            a, b = gen.rng.choice(pool), gen.rng.choice(pool)
            ops += [("SUM_ADD", (a, b)), ("SUM_SUB", (a, b)), ("SUM_NEG", (a,)), ("PRODUCT_ADDED", (a, b, gen.rng.choice(pool)))]
        check(ops)
        results = [result for result in oracle.run_oracle(ops) if type(result) is SqrtSumV1]
        int_terms += sum(type(c) is int for value in results for _, c in value.terms)
        fraction_terms += sum(type(c) is Fraction for value in results for _, c in value.terms)
        pool.extend(result for result in results if _compact(result))
        pool = pool[-60:]
    assert int_terms > 50 and fraction_terms > 100, "цепочка не несла оба типа коэффициентов: сверка типов пуста"


def test_the_product_memory_never_changes_an_answer():
    gen = Gen(31)
    radicands = gen.universe(big=True)
    pool = _pool(gen, radicands, 10, 200, 7)
    ops = []
    for _ in range(40):
        a, b = gen.rng.choice(pool), gen.rng.choice(pool)
        ops += [("SUM_MUL", (a, b)), ("PRODUCT_ADDED", (a, b, a)), ("SUM_OF_PRODUCTS", ([[a, b, 1], [b, a, -1], [a, a, 1]],))]
    with_memo = cftuv_native.run_number_ops(ops, memo=True)
    without = cftuv_native.run_number_ops(ops, memo=False)
    assert all(oracle.same(left, right) for left, right in zip(with_memo, without))
    check(ops, memo=False)


def test_cancellation_and_zero_paths():
    gen = Gen(5)
    radicands = gen.universe()
    pool = _pool(gen, radicands, 12, 100, 6)
    ops = []
    for value in pool:
        zero = SqrtSumV1(())
        ops += [
            ("SUM_SUB", (value, value)),
            ("SUM_ADD", (value, -value)),
            ("SUM_ADD", (value, zero)),
            ("SUM_SUB", (zero, value)),
            ("SUM_MUL", (value, zero)),
            ("SUM_SCALED", (value, 0)),
            ("SUM_SCALED", (value, Fraction(0))),
            ("SUM_SCALED_DIFFERENCE", (value, 2, value, 2)),
            ("SUM_SCALED_DIFFERENCE", (value, 0, value, 0)),
            ("SUM_DIFFERENCE_IS_ZERO", (value, value)),
            ("DIFFERENCE_FILTERED_SIGN", (value, value)),
            ("PRODUCT_ADDED", (value, zero, value)),
            ("PRODUCT_ADDED", (-value, value, SqrtSumV1.rational(1))),
            ("SUM_OF_PRODUCTS", ([],)),
            ("SUM_OF_PRODUCTS", ([[value, zero, 1]],)),
            ("ORIENTED_SUM", (value, value, 1, 1, 0)),
            ("ORIENTED_SUM", (value, value, 0, 0, 0)),
            ("ORIENTED_SUM", (value, zero, 3, 0, -7)),
        ]
    check(ops)


def test_degenerate_integer_forms_and_float_arguments_equal_the_oracle():
    ops = [
        ("REDUCED_FORM", (0, [])),
        ("REDUCED_FORM", (0, [[1, 0]])),
        ("REDUCED_FORM", (0, [[1, 5], [2, -10]])),
        ("REDUCED_FORM", (7, [])),
        ("REDUCED_FORM", (12, [[1, 0], [3, 0]])),
        ("SCALED_BY_RECIPROCAL", (0, [[1, 1]], 1, [[1, 2]])),
        ("SCALED_BY_RECIPROCAL", (1, [[1, 1]], 0, [[1, 2]])),
        ("SCALED_BY_RECIPROCAL", (1, [], 1, [])),
        ("SCALED_BY_RECIPROCAL", (1, [], 3, [[1, 6]])),
        # `Fraction(value * ..., numerator_common * ...)` строится ПО ЧЛЕНУ: без членов нулевой знаменатель никого не отказывает
        ("SCALED_BY_RECIPROCAL", (0, [], 1, [[1, 2]])),
        ("SCALED_BY_RECIPROCAL", (0, [[1, 1]], 1, [[1, 2]])),
        ("INTEGER_ENCLOSURE", ([], 0)),
        ("INTEGER_ENCLOSURE", ([[1, 0], [2, 0]], 5)),
        ("INTEGER_CERTIFIED_SIGN", ([[2, 0]], 64)),
        ("MULTIPLY_INTEGER_ITEMS", ([], [[2, 1]])),
        ("MULTIPLY_INTEGER_ITEMS", ([[2, 0]], [[3, 5]])),
        ("FILTERED_SIGN", ([[1, 0]], 64)),
        ("FILTERED_SIGN", ([[2, 0], [3, 0]], 64)),
        # `as_rational` hands back the coefficient OBJECT: an int stays an int, the empty sum gives Fraction(0)
        ("SUM_AS_RATIONAL", (SqrtSumV1(((1, 5),)),)),
        ("SUM_AS_RATIONAL", (SqrtSumV1(((1, Fraction(5)),)),)),
        ("SUM_AS_RATIONAL", (SqrtSumV1(()),)),
        ("SUM_AS_RATIONAL", (SqrtSumV1(((2, 5),)),)),
        ("SUM_RATIONAL", (7,)),
        ("SUM_RATIONAL", (Fraction(-7, 3),)),
        ("SUM_RATIONAL", (0,)),
    ]
    point = [SqrtSumV1(((1, Fraction(3)),)), SqrtSumV1(((2, Fraction(1, 2)),))]
    infinity, nan = float("inf"), float("nan")
    for start, step in ((infinity, 1.0), (nan, 2.0), (1.0, infinity), (1.0, nan), (-0.0, 0.0), (1e308, 1e308), (5e-324, 5e-324)):
        ops.append(("FF_LINE_ESTIMATE", (point, start, start, step, step)))
        ops.append(("FF_LINE_ESTIMATE", (point, 0.0, 1.0, step, -step)))
    assert check(ops) == len(ops)
    results = oracle.run_oracle(ops)
    assert results[0] == oracle.NativeError(2) and results[1] == oracle.NativeError(2) and results[2] != oracle.NativeError(2)
    as_rational = [result for (name, _), result in zip(ops, results) if name == "SUM_AS_RATIONAL"]
    assert [type(result) for result in as_rational[:3]] == [int, Fraction, Fraction] and as_rational[3] is None


def test_cost_operations_need_their_header():
    """`EXACT_*` без заголовка стоимости (память и бюджет) не исполняются: цена без бюджета — молчаливо неверная цена."""

    for ops in (
        [("EXACT_SIGN", (SqrtSumV1(((1, 1),)), 64))],
        [("EXACT_RESET_MEMORY", ())],
        [("EXACT_DIVIDED_BY_GENERIC", (SqrtSumV1(((1, 1),)), SqrtSumV1(((1, 1),))))],
    ):
        with pytest.raises(ValueError, match="cost"):
            cftuv_native.run_number_ops(ops)
    # числовые операции того же сценария без заголовка работают: отказывает именно операция стоимости
    with pytest.raises(ValueError, match="cost header"):
        cftuv_native.run_number_ops([("SUM_IS_ZERO", (SqrtSumV1(()),)), ("EXACT_RESET_MEMORY", ())])


def test_arguments_outside_the_domain_are_refused_not_computed():
    for ops in (
        [("SUM_ENCLOSURE", (SqrtSumV1(((2, 1),)), 1 << 40))],
        [("INTEGER_ENCLOSURE", ([[0, 5]], 3))],
        [("MULTIPLY_INTEGER_ITEMS", ([[-2, 1]], [[3, 1]]))],
        [("SUM_ENCLOSURE", (SqrtSumV1(((2, 1),)), -1))],
    ):
        with pytest.raises(ValueError, match="bad arguments"):
            cftuv_native.run_number_ops(ops)


# --------------------------------------------------------------------------
# Целые, дроби, float
# --------------------------------------------------------------------------


def _boundary_integers() -> list:
    values = {0, 1, -1, 2, 10**30, -(10**30), 2**127 - 1, 2**127, -(2**127), -(2**127) - 1, 2**128, 2**200 + 12345}
    for exponent in (31, 32, 52, 53, 54, 62, 63, 64, 65, 126, 127, 128, 970, 971, 972, 1023, 1024, 1025, 2000):
        for delta in range(-3, 4):
            values.add(2**exponent + delta)
            values.add(-(2**exponent) + delta)
    values.add(2**1024 - 2**970)
    values.add(2**1024 - 2**970 - 1)
    values.add(2**1024 - 2**971)
    values.add(2**1024 - 2**970 + 1)
    return sorted(values)


def test_integer_operations_equal_python_including_the_edges():
    gen = Gen(9)
    values = _boundary_integers() + [gen.integer() for _ in range(400)]
    ops = []
    for value in values:
        ops += [("FLOAT_OF_INT", (value,)), ("BIT_LENGTH", (value,)), ("ISQRT", (value,)), ("MATH_SQRT_INT", (value,))]
    for _ in range(600):
        a, b = gen.rng.choice(values), gen.rng.choice(values)
        ops += [("GCD", (a, b)), ("LCM", (a, b))]
    ops += [("GCD", (0, 0)), ("LCM", (0, 5)), ("LCM", (0, 0)), ("GCD", (-12, 18)), ("LCM", (-4, 6)), ("ISQRT", (-4,)), ("MATH_SQRT_INT", (-1,))]
    for value in values:
        if value > 0:
            root = math.isqrt(value)
            ops += [("ISQRT", (root * root,)), ("ISQRT", (root * root - 1,))]
    assert check(ops) == len(ops)


def _tie_fractions(gen: Gen) -> list:
    """Дроби на границах округления: точные середины между двумя соседними float и числа на волосок по обе стороны."""

    fractions = []
    for exponent in list(range(-1085, -1050)) + list(range(-1050, -1000, 3)) + list(range(-60, 60, 7)) + [1000, 1020, 1022, 1023, 1024]:
        for mantissa in (2**52, 2**52 + 1, 2**53 - 1, 2**52 + 12345):
            tie = Fraction(2 * mantissa + 1, 2) * Fraction(2) ** (exponent - 52)
            scale = gen.integer(gen.rng.choice((8, 64, 200)), signed=False)
            for nudge in (-1, 0, 1):
                numerator = tie.numerator * scale + nudge
                fractions.append(Fraction(numerator, tie.denominator * scale))
    fractions += [Fraction(1, 2**1074), Fraction(1, 2**1075), Fraction(3, 2**1075), Fraction(2**53 - 1, 2**1075), Fraction(1, 2**1200)]
    fractions += [Fraction(2**1024 - 2**970, 1), Fraction(2**1030, 2**10), Fraction(2**2000 + 1, 2**1000), Fraction(10**400, 10**390)]
    return fractions + [-value for value in fractions]


def test_float_of_fraction_is_correctly_rounded_like_cpython():
    gen = Gen(11)
    values = _tie_fractions(gen)
    values += [Fraction(gen.integer(), gen.integer(signed=False)) for _ in range(600)]
    values += [Fraction(gen.integer(2000), gen.integer(2000, signed=False)) for _ in range(120)]
    ops = [("FLOAT_OF_FRACTION", (value,)) for value in values]
    assert check(ops) == len(ops)


def test_rational_arithmetic_is_fraction_arithmetic():
    gen = Gen(13)
    values = [gen.coefficient(0.3, 600) for _ in range(120)] + [0, 1, -1, Fraction(0), Fraction(1, 2)]
    ops = [("RAT_NEW", (gen.integer(gen.rng.choice(BIT_SIZES)), gen.integer(gen.rng.choice(BIT_SIZES)))) for _ in range(200)]
    ops += [("RAT_NEW", (5, 0)), ("RAT_NEW", (0, -3)), ("RAT_NEW", (-6, -4))]
    for _ in range(500):
        a, b = gen.rng.choice(values), gen.rng.choice(values)
        ops += [("RAT_ADD", (a, b)), ("RAT_SUB", (a, b)), ("RAT_MUL", (a, b)), ("RAT_DIV", (a, b)), ("RAT_CMP", (a, b))]
    ops += [("RAT_NEG", (value,)) for value in values]
    assert check(ops) == len(ops)


# --------------------------------------------------------------------------
# Фильтры binary64
# --------------------------------------------------------------------------

SMALL_RADICANDS = (1, 2, 3, 5, 6, 7, 10, 11, 13, 14, 15)


def _small_sum(rng: random.Random, terms: int | None = None) -> SqrtSumV1:
    count = rng.randint(0, 4) if terms is None else terms
    chosen = sorted(rng.sample(SMALL_RADICANDS, count))
    coefficients = [Fraction(rng.randint(-9, 9) or 1, rng.randint(1, 4)) for _ in chosen]
    return SqrtSumV1(tuple(zip(chosen, coefficients)))


def _extreme_sum(rng: random.Random, kind: int | None = None) -> SqrtSumV1:
    """Значения у границ binary64: коэффициент за `float`, меньше наименьшего нормального, радикант за `float`, переполнение произведения."""

    kind = rng.randrange(8) if kind is None else kind
    if kind == 0:
        return SqrtSumV1(((1, Fraction(2**1100)),))
    if kind == 1:
        return SqrtSumV1(((1, Fraction(1, 2**1100)),))
    if kind == 2:
        return SqrtSumV1((((2**521 - 1) * (2**607 - 1), Fraction(1)),))
    if kind == 3:
        return SqrtSumV1(((2, 3), (5, Fraction(2**1023))))
    if kind == 4:
        return SqrtSumV1(((1, Fraction(2**1023 - 1, 2**1020)), (2, Fraction(1, 3))))
    if kind == 5:  # ровно наименьшее нормальное: `abs(centre) < _FLOOR` ложно
        return SqrtSumV1(((1, Fraction(1, 2**1022)),))
    if kind == 6:  # денормал: отказ
        return SqrtSumV1(((1, Fraction(1, 2**1023)),))
    return SqrtSumV1(((1, Fraction(2**53 - 1, 2**1075)),))  # чуть меньше наименьшего нормального, но округляется ровно в него


def _point(rng: random.Random):
    return [_small_sum(rng), _small_sum(rng)]


def _scaled_point(point, factor):
    return [point[0].scaled(factor), point[1].scaled(factor)]


def _shifted(point, other):
    return [point[0] + other[0], point[1] + other[1]]


def _triples(rng: random.Random, count: int) -> list:
    triples = []
    for index in range(count):
        first, second, third = _point(rng), _point(rng), _point(rng)
        kind = index % 4
        if kind == 1:  # коллинеарные: точное нулевое значение
            direction = _point(rng)
            first = _shifted(second, direction)
            third = _shifted(second, _scaled_point(direction, Fraction(rng.randint(2, 9), rng.randint(1, 4))))
        elif kind == 2:  # повторённая точка
            third = first
        elif kind == 3 and index % 8 == 3:  # крайние значения в одной координате
            first = [_extreme_sum(rng), first[1]]
        triples.append((first, second, third))
    return triples


def _polygons(rng: random.Random, count: int) -> list:
    polygons = []
    for _ in range(count):
        corners = rng.randint(1, 8)
        polygons.append([[_small_sum(rng, rng.randint(0, 2)), _small_sum(rng, rng.randint(0, 2))] for _ in range(corners)])
    return polygons


def _exact_affine(rng: random.Random):
    """Четыре вершины и значения на точной аффинной карте, плюс вариант с нарушением у последней."""

    points = [_point(rng) for _ in range(4)]
    coefficients = [[Fraction(rng.randint(-5, 5), rng.randint(1, 3)) for _ in range(3)] for _ in range(2)]
    values = []
    for x, y in points:
        values.append([x.scaled(c[0]) + y.scaled(c[1]) + SqrtSumV1.rational(c[2]) for c in coefficients])
    violated = [list(pair) for pair in values]
    violated[3][rng.randrange(2)] = violated[3][0] + SqrtSumV1.rational(Fraction(rng.choice((1, 1000, 10**9)), rng.choice((1, 10**6, 10**12))))
    return points, values, violated


def test_the_binary64_filters_equal_the_oracle_bit_for_bit():
    rng = random.Random(21)
    ops = []
    triples = _triples(rng, 600)
    ops += [("FF_ORIENTATION_SIGN", triple) for triple in triples]
    ops += [("FF_CENTRE_AND_BOUND", (coordinate,)) for triple in triples[:120] for point in triple for coordinate in point]
    ops += [("FF_POLYGON_SIGN", (polygon,)) for polygon in _polygons(rng, 300)]
    for _ in range(300):
        start = [float(rng.randint(-20, 20)) for _ in range(2)]
        step = [float(rng.choice((0, 1, -1, 2, 3, -5, 2**40, -(2**53)))) for _ in range(2)]
        ops.append(("FF_LINE_ESTIMATE", (_point(rng), *start, *step)))
    ops += [("FF_LINE_ESTIMATE", ([_extreme_sum(rng), _small_sum(rng)], 1.0, 2.0, 3.0, -4.0)) for _ in range(40)]
    for _ in range(150):
        points, values, violated = _exact_affine(rng)
        ops += [("FF_AFFINE_MAP_VIOLATED", (points, values)), ("FF_AFFINE_MAP_VIOLATED", (points, violated))]
    outcomes = [result for result in oracle.run_oracle(ops)]
    check(ops)
    signs = {result for (name, _), result in zip(ops, outcomes) if name == "FF_ORIENTATION_SIGN"}
    assert {1, -1, None} <= signs, "фильтр должен и доказывать оба знака, и уступать: иначе сверка пуста"
    assert any(result is True for (name, _), result in zip(ops, outcomes) if name == "FF_AFFINE_MAP_VIOLATED")
    assert any(result is False for (name, _), result in zip(ops, outcomes) if name == "FF_AFFINE_MAP_VIOLATED")


def test_binary64_edge_values_leave_the_filter_instead_of_raising():
    rng = random.Random(3)
    extremes = [_extreme_sum(rng, kind) for kind in range(8)]
    ops = [("FF_CENTRE_AND_BOUND", (value,)) for value in extremes]
    ops += [("FF_ORIENTATION_SIGN", ([value, SqrtSumV1(())], [SqrtSumV1(()), SqrtSumV1(())], [SqrtSumV1(()), value])) for value in extremes]
    check(ops)
    results = oracle.run_oracle(ops[:8])
    assert [result is None for result in results] == [True, True, True, True, False, False, True, False], (
        "эталон: за float, меньше наименьшего нормального, радикант за float, переполнение произведения -> None; на самой границе и в её пределах фильтр берёт"
    )


# --------------------------------------------------------------------------
# Счётчики знака и разделение `sign`
# --------------------------------------------------------------------------


def test_the_sign_prefilter_counts_what_sqrt_sum_sign_counts():
    gen = Gen(55)
    ops = []
    for _ in range(40):
        radicands = gen.universe()
        for _ in range(20):
            value = gen.sum(radicands, 5, 40)
            ops += [("SUM_SIGN_PREFILTER", (value, bits)) for bits in (0, 2, 8, 64)]
            ops.append(("SUM_SIGN_PREFILTER", (value.scaled(Fraction(1, 7)) - value.scaled(Fraction(1, 7)), 64)))
    compared = check(ops)
    assert compared > 1500
    results = oracle.run_oracle(ops)
    conjugated = [result for result in results if result is not oracle.SKIPPED and result[0] is None]
    assert conjugated, "ни один знак не ушёл в сопряжение: ветка NeedsConjugation не проверена"
    assert all(result[1][4] == 1 and result[1][0] == 1 for result in conjugated)


def test_the_filtered_sign_counter_delta_is_reported_per_call():
    ops = [
        ("FILTERED_SIGN", ([], 64)),
        ("FILTERED_SIGN", ([[1, -9]], 64)),
        ("FILTERED_SIGN", ([[1, 1], [2, -1]], 64)),
        ("FILTERED_SIGN", ([[1, 1], [2, -1]], 0)),
        ("FILTERED_SIGN", ([[2, 5]], 64)),
        ("FILTERED_SIGN", ([[2, 0]], 64)),
    ]
    assert check(ops) == len(ops)
    results = oracle.run_oracle(ops)
    assert results[0] == [0, [1, 1, 0, 0, 0]] and results[1] == [-1, [1, 0, 1, 0, 0]] and results[3][0] is None and results[3][1] == [0, 0, 0, 0, 0]


# --------------------------------------------------------------------------
# Формат буфера
# --------------------------------------------------------------------------


def test_the_op_table_matches_the_extension():
    assert cftuv_native.number_op_table() == codec.OPS
    assert len({name for _, name in codec.OPS}) == len(codec.OPS)


def test_the_python_codec_round_trips_any_size_and_keeps_types():
    gen = Gen(2)
    values = [None, True, False, 0, -1, 2**127, -(2**127) - 1, 2**4000, -(2**4000), 1.5, -0.0, float("inf"), Fraction(3, 7), Fraction(5, 1), Fraction(-2**300, 3)]
    values += [SqrtSumV1(()), SqrtSumV1(((1, 3), (2, Fraction(5)), (3, Fraction(1, 2)))), [1, [Fraction(1, 3), None], []]]
    values += [gen.sum(gen.universe(big=True), 6, 600) for _ in range(40)]
    for value in values:
        decoded = codec.decode_value(codec.encode_value(value))
        expected = list(value) if type(value) is tuple else value
        assert oracle.same(expected, decoded), oracle.describe(value)
    nan = codec.decode_value(codec.encode_value(float("nan")))
    assert nan != nan
    with pytest.raises(codec.CodecError):
        codec.encode_value("text")
    with pytest.raises(codec.CodecError):
        codec.encode_value(SqrtSumV1(((-1, 1),)))


def test_a_value_survives_the_boundary_unchanged():
    """Через расширение и обратно (сложение с нулём возвращает тот же набор): типы и числа за `i128` целы."""

    gen = Gen(4)
    zero = SqrtSumV1(())
    values = [gen.sum(gen.universe(big=True), 6, 2100, 0.5) for _ in range(60)]
    results = cftuv_native.run_number_ops([("SUM_ADD", (value, zero)) for value in values])
    for value, result in zip(values, results):
        assert oracle.same(value, result), oracle.describe(value)


def test_the_extension_refuses_what_is_not_canonical_instead_of_guessing():
    unsorted = SqrtSumV1(((3, Fraction(1)), (2, Fraction(1))))
    duplicated = SqrtSumV1(((2, 1), (2, 2)))
    zero_term = SqrtSumV1(((2, 0),))
    for bad in (unsorted, duplicated, zero_term):
        with pytest.raises(ValueError, match="NonCanonicalSum"):
            cftuv_native.run_number_ops([("SUM_IS_ZERO", (bad,))])
    not_reduced = codec.make_fraction(2, 4)
    with pytest.raises(ValueError, match="NotInLowestTerms"):
        cftuv_native.run_number_ops([("RAT_ADD", (not_reduced, Fraction(1)))])
    with pytest.raises(ValueError, match="bad arguments"):
        cftuv_native.run_number_ops([("SUM_ADD", (Fraction(1), Fraction(1)))])
    with pytest.raises(ValueError, match="unknown opcode"):
        cftuv_native.run_number_ops([(250, ())])
    with pytest.raises(ValueError):
        codec.decode_response(b"\x01\x63")


# --------------------------------------------------------------------------
# Настоящие операнды: вызовы, которые ядро делает на своих доменах
# --------------------------------------------------------------------------

REAL_CAP = 500
#: `(имя, фабрика `developable_factories`, позиционные, именованные, маршрут, alpha)` — те же случаи, что у `test_developable_materialize`.
FLOWS = (
    ("fold-strip", "fold_strip", (), {}, ("r0a", "r0b"), "1.5"),
    ("bevel", "bevel_strip", (4,), {}, ("r0a", "r0b"), "2.5"),
    ("quarter-cylinder", "quarter_cylinder", (), {}, ("r0a", "r0b"), "0.8"),
    ("cone-sector", "cone", (8,), {"boundary_apex": True}, ("apex", "b0"), "0.5"),
)


class CallRecorder:
    """Подмена функций ядра, которая записывает аргументы настоящих вызовов (выборка без повторов по счётчику)."""

    def __init__(self, seed: int = 0) -> None:
        self.calls: dict = {}
        self.seen: dict = {}
        self.rng = random.Random(seed)

    def record(self, op: str, arguments: tuple) -> None:
        self.seen[op] = self.seen.get(op, 0) + 1
        bucket = self.calls.setdefault(op, [])
        if len(bucket) < REAL_CAP:
            bucket.append(arguments)
        else:
            slot = self.rng.randrange(self.seen[op])
            if slot < REAL_CAP:
                bucket[slot] = arguments

    def wrap(self, monkeypatch, owner, name: str, op: str, convert) -> None:
        original = getattr(owner, name)

        def recorded(*args, **kwargs):
            arguments = convert(args, kwargs)
            if arguments is not None:
                self.record(op, arguments)
            return original(*args, **kwargs)

        monkeypatch.setattr(owner, name, recorded)


def _affine_arguments(args, kwargs):
    points, values, base, index = args
    keys = (base[0], base[1], base[2], index)
    return ([list(points[key]) for key in keys], [list(values[key]) for key in keys])


def _as_is(args, kwargs):
    return tuple(args)


def _install_recorders(monkeypatch, recorder: CallRecorder) -> None:
    import cftuv_envelope.float_filter as float_filter
    import cftuv_envelope.materialize.clip as clip
    import cftuv_envelope.materialize.lift_surface as lift_surface
    import cftuv_envelope.materialize.silhouette as silhouette
    import cftuv_envelope.wavefront.faces as faces

    pair = _as_is
    wrap = recorder.wrap
    for method, op in (("__add__", "SUM_ADD"), ("__sub__", "SUM_SUB"), ("__mul__", "SUM_MUL"), ("scaled", "SUM_SCALED"), ("scaled_difference", "SUM_SCALED_DIFFERENCE")):
        wrap(monkeypatch, SqrtSumV1, method, op, pair)
    wrap(monkeypatch, SqrtSumV1, "difference_is_zero", "SUM_DIFFERENCE_IS_ZERO", pair)
    wrap(monkeypatch, clip, "product_added", "PRODUCT_ADDED", pair)
    wrap(monkeypatch, lift_surface, "oriented_sum", "ORIENTED_SUM", pair)
    wrap(monkeypatch, faces, "orientation_sign", "FF_ORIENTATION_SIGN", lambda args, kwargs: tuple([list(point) for point in args]))
    wrap(monkeypatch, faces, "polygon_sign", "FF_POLYGON_SIGN", lambda args, kwargs: ([list(point) for point in args[0]],))
    wrap(monkeypatch, float_filter, "line_estimate", "FF_LINE_ESTIMATE", lambda args, kwargs: (list(args[0]), *map(float, args[1:])))
    wrap(monkeypatch, silhouette, "affine_map_violated", "FF_AFFINE_MAP_VIOLATED", _affine_arguments)
    original = faces.sum_of_products

    def recorded_products(products):
        products = [tuple(entry) for entry in products]
        recorder.record("SUM_OF_PRODUCTS", ([list(entry) for entry in products],))
        return original(products)

    monkeypatch.setattr(faces, "sum_of_products", recorded_products)


def _run_flows(recorder: CallRecorder) -> None:
    import developable_factories as factories
    from developable_route import materialize_developable

    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1

    laws = (None, DecalTopologyLawV1.PLANAR_POLYGONS_V1, DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1)
    for law in laws:
        for _name, factory, factory_args, factory_kwargs, route, alpha in FLOWS:
            options = {} if law is None else {"decal_topology_law": law, "near_planar_lift_law": NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1}
            parts = getattr(factories, factory)(*factory_args, **factory_kwargs)
            result, _prepared = materialize_developable(parts, route, alpha=alpha, **options)
            assert result.is_materialized, (factory, law, result.detail)


@pytest.fixture(scope="module")
def real_calls():
    from cftuv_envelope.materialize import clip_memo

    recorder = CallRecorder()
    with pytest.MonkeyPatch.context() as monkeypatch:
        monkeypatch.setattr(clip_memo.MEMO, "enabled", False)
        _install_recorders(monkeypatch, recorder)
        with exact.isolated_factorization_memory():
            _run_flows(recorder)
    oracle.clear_oracle_state()
    return recorder


def _sums_of(calls: dict) -> list:
    found = {}

    def visit(value):
        if type(value) is SqrtSumV1:
            found.setdefault(value.terms, value)
        elif type(value) in (list, tuple):
            for item in value:
                visit(item)

    for bucket in calls.values():
        for arguments in bucket:
            visit(arguments)
    return list(found.values())


def _derived_ops(sums: list, rng: random.Random) -> list:
    """Операции над настоящими значениями: оболочка, знак, форма, разности, фильтры по настоящим координатам."""

    ops = []
    for value in sums:
        ops += [("SUM_ENCLOSURE", (value, 64)), ("SUM_CERTIFIED_SIGN", (value, 64)), ("INTEGER_FORM", (value,)), ("FF_CENTRE_AND_BOUND", (value,))]
        ops += [("SUM_IS_RATIONAL", (value,)), ("SUM_AS_RATIONAL", (value,)), ("SUM_SIGN_PREFILTER", (value, 64)), ("SUM_NEG", (value,))]
    for _ in range(300):
        a, b, c = (rng.choice(sums) for _ in range(3))
        ops += [("DIFFERENCE_FILTERED_SIGN", (a, b)), ("SUM_SCALED_DIFFERENCE", (a, Fraction(rng.randint(-5, 5), rng.randint(1, 7)), b, 3)), ("SUM_MUL", (a, b))]
        ops += [("PRODUCT_ADDED", (a, b, c)), ("ORIENTED_SUM", (a, b, rng.randint(-9, 9), rng.randint(-9, 9), rng.randint(-9, 9)))]
        ops += [("SUM_OF_PRODUCTS", ([[a, b, 1], [b, c, -1], [c, a, 1]],))]
        ops += [("FF_ORIENTATION_SIGN", ([a, b], [b, c], [c, a])), ("FF_POLYGON_SIGN", ([[a, b], [b, c], [c, a], [a, c]],))]
    return ops


def test_real_calls_of_the_kernel_equal_the_oracle(real_calls):
    ops = [(op, arguments) for op, bucket in real_calls.calls.items() for arguments in bucket]
    assert {"SUM_ADD", "SUM_SUB", "SUM_MUL", "SUM_SCALED", "SUM_SCALED_DIFFERENCE", "FF_ORIENTATION_SIGN", "ORIENTED_SUM", "SUM_OF_PRODUCTS", "PRODUCT_ADDED"} <= set(real_calls.calls), sorted(real_calls.calls)
    assert len(ops) > 3000
    compared = check(ops)
    assert compared > 3000
    # коэффициенты настоящих значений — `Fraction` с настоящими знаменателями (`int`-коэффициенты ядро само не создаёт: их сверяют случайные цепочки)
    sums = _sums_of(real_calls.calls)
    assert any(type(c) is Fraction and c.denominator != 1 for value in sums for _, c in value.terms)


def test_real_values_under_every_operation_equal_the_oracle(real_calls):
    sums = [value for value in _sums_of(real_calls.calls) if len(value.terms) <= 16]
    assert len(sums) > 200
    rng = random.Random(8)
    rng.shuffle(sums)
    ops = _derived_ops(sums[:600], rng)
    compared = check(ops)
    assert compared > 3000


def test_the_affine_and_line_filters_see_real_values(real_calls):
    """Тождество аффинности и оценка у прямой — на настоящих вызовах, какие эти домены порождают (на малых доменах `line_estimate` не зовётся)."""

    present = [op for op in ("FF_AFFINE_MAP_VIOLATED", "FF_LINE_ESTIMATE") if op in real_calls.calls]
    assert "FF_AFFINE_MAP_VIOLATED" in present, "закон SILHOUETTE_TOPOLOGY_V1 не позвал фильтр аффинности: записывать нечего"
    assert check([(op, arguments) for op in present for arguments in real_calls.calls[op]]) >= 3


def _corpus_directories() -> list:
    base = Path(os.environ.get("CFTUV_NATIVE_CORPUS", "E:/cftuv_native_corpus"))
    return sorted(path.parent for path in itertools.chain(base.glob("index.json"), base.glob("*/index.json")))


def _load_corpus_tool():
    cached = sys.modules.get("native_corpus")
    if cached is not None:
        return cached
    spec = importlib.util.spec_from_file_location("native_corpus", ROOT / "tools" / "native_corpus.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["native_corpus"] = module
    spec.loader.exec_module(module)
    return module


def _walk_sums(root, limit: int) -> dict:
    """Все `SqrtSumV1` в графе объектов (контейнеры, `dataclass`, слоты); ограничено числом узлов и значений."""

    found, visited, stack = {}, set(), [root]
    while stack and len(found) < limit and len(visited) < 400_000:
        value = stack.pop()
        if id(value) in visited or value is None or isinstance(value, (int, float, str, bytes, Fraction)):
            continue
        visited.add(id(value))
        if type(value) is SqrtSumV1:
            found.setdefault(value.terms, value)
        elif isinstance(value, dict):
            stack.extend(value.values())
            stack.extend(key for key in value if not isinstance(key, (int, str)))
        elif isinstance(value, (list, tuple, set, frozenset)):
            stack.extend(value)
        else:
            names = list(getattr(value, "__dict__", {}).values())
            for cls in type(value).__mro__:
                names += [getattr(value, slot, None) for slot in getattr(cls, "__slots__", ()) if isinstance(slot, str)]
            stack.extend(names)
    return found


def _corpus_rows(directory: Path) -> list:
    """По мешу — записи на медиане, трёх четвертях и в максимуме размера: мелкие записи почти пусты, нужны и крупные."""

    import json

    by_mesh: dict = {}
    for row in json.loads((directory / "index.json").read_text(encoding="utf-8")).get("records", []):
        by_mesh.setdefault(row.get("mesh"), []).append(row)
    picked = []
    for rows in by_mesh.values():
        rows.sort(key=lambda row: row.get("bytes", 0))
        picked += [directory / rows[index]["path"] for index in sorted({len(rows) // 2, (3 * len(rows)) // 4, len(rows) - 1})]
    return picked


def _corpus_sums(tool) -> list:
    import pickle

    found: dict = {}
    for directory in _corpus_directories():
        for path in _corpus_rows(directory):
            try:
                record = tool.read_record(path)
                result = record.payload["expected"]["result"]
                if result is not None:
                    found.update(_walk_sums(pickle.loads(result), 800))
                found.update(_walk_sums(tool.decode_call(record.op, record.call_blob, None, {}).args, 800))
            except Exception:  # запись, которую ещё пишут, или другой версии: пропускается, пустой корпус ниже даст skip
                continue
    return [value for value in found.values() if 0 < len(value.terms) <= 24]


@field_tier(bool(_corpus_directories()), "корпус не собран (E:/cftuv_native_corpus/*/index.json): настоящие значения корпуса не сверены")
def test_corpus_values_equal_the_oracle():
    if not _corpus_directories():
        pytest.skip("корпус не собран (E:/cftuv_native_corpus/*/index.json): настоящие значения корпуса не сверены")
    sums = _corpus_sums(_load_corpus_tool())
    if len(sums) < 100:
        pytest.skip("в корпусе не нашлось читаемых записей со значениями")
    rng = random.Random(4)
    rng.shuffle(sums)
    assert check(_derived_ops(sums[:700], rng)) > 5000
    assert any(radicand >= 2**127 or abs(Fraction(c).numerator) >= 2**127 or Fraction(c).denominator >= 2**127 for value in sums for radicand, c in value.terms), (
        "в выборке корпуса нет операндов за i128"
    )
