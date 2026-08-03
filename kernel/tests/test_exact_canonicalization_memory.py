"""Память канонизации не двигает ни одного ответа — она двигает только цену.

Мемоизация разложений (Р1) и взаимно простой базис набора радикандов (Р2)
стоят на одном тождестве: разложение целого на простые ЕДИНСТВЕННО. Поэтому
`squarefree_split` и `prime_support` обязаны возвращать ПОБИТОВО те же
значения при любом состоянии памяти, а базис обязан порождать ту же поддержку
простых, что и сам набор.

Числа ниже — измеренные, не выдуманные: это ровно те пятнадцать радикандов,
которые `walls.012` на Fan Density 0 предъявляет канонизации
(`artifacts/perf_prepare_diag/`, §3 REPORT.txt). Пять из них приходят набором
через `_prime_universe_from_q_values`, десять — поодиночке из
`squarefree_split` и `prime_support`.
"""

from __future__ import annotations

import random

import pytest

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope.exact_sqrt_sum import (
    _coprime_basis,
    _factorize,
    _prime_universe_from_q_values,
    _seed_factorization_basis,
    prime_support,
    reset_factorization_memory,
    squarefree_split,
)


# Набор, который домен предъявляет НАБОРОМ: `_prime_universe_from_q_values`.
FIELD_UNIVERSE = (
    6824216720695701361835948836879986852324977009920801928904704,
    73943945978196176332097962847740988835857293794080042659086336,
    86814285812865855333316357428078059036731529663491830546778030080,
    8815271257367225442991968805039683217668930429457914371865045504375128064,
    79337568822028543179575696065768839636502023975431072989891014190165917696,
)

# Радикалы, которые тот же домен предъявляет ПООДИНОЧКЕ — они рождаются ниже
# по конвейеру, из произведений уже канонизированных величин.
FIELD_SINGLETONS = (
    2,
    3,
    5849,
    35415780824258511981,
    10985874142994794447489454,
    2852179969732232078393414670,
    289615238697188253740039047616843361,
    651635334330847152894392169309537126,
    1139661090784992389845324353133788017459143663357645349,
    1789695942521755455579271029764900419088644768480592376617301,
)

FIELD_RADICANDS = FIELD_UNIVERSE + FIELD_SINGLETONS


@pytest.fixture(autouse=True)
def _cold_memory():
    """Каждый тест начинает с пустой памяти и оставляет её пустой."""

    reset_factorization_memory()
    yield
    reset_factorization_memory()


def _reference(radicands):
    """Ответы при ПУСТОЙ памяти на каждом шаге: базис не участвует вовсе."""

    answers = []
    for radicand in radicands:
        reset_factorization_memory()
        answers.append(
            (
                radicand,
                tuple(sorted(_factorize(radicand).items())),
                squarefree_split(radicand),
                prime_support(radicand),
            )
        )
    reset_factorization_memory()
    return tuple(answers)


def _observed(radicands, *, seed_first):
    """Ответы при ОБЩЕЙ памяти, с базисом набора или без него."""

    if seed_first:
        _seed_factorization_basis(tuple(sorted(radicands)))
    return tuple(
        (
            radicand,
            tuple(sorted(_factorize(radicand).items())),
            squarefree_split(radicand),
            prime_support(radicand),
        )
        for radicand in radicands
    )


def test_field_radicands_are_bit_for_bit_identical_with_and_without_basis():
    """Пятнадцать измеренных радикандов walls.012: три ответа, ноль расхождений."""

    reference = _reference(FIELD_RADICANDS)

    reset_factorization_memory()
    plain = _observed(FIELD_RADICANDS, seed_first=False)
    reset_factorization_memory()
    seeded = _observed(FIELD_RADICANDS, seed_first=True)

    assert plain == reference
    assert seeded == reference


def test_field_prime_universe_is_bit_for_bit_identical_with_and_without_basis():
    """Носитель простых набора не зависит от того, строился ли базис."""

    reset_factorization_memory()
    seeded = _prime_universe_from_q_values(FIELD_UNIVERSE)

    reset_factorization_memory()
    plain = tuple(
        sorted(
            {
                prime
                for radicand in FIELD_UNIVERSE
                for prime, power in _factorize(radicand).items()
                if power % 2
            }
        )
    )

    assert seeded == plain
    assert len(seeded) == 23


def test_basis_of_the_field_universe_removes_the_expensive_factorizations():
    """Измеримое утверждение Р2: ро-Поллард перестаёт быть нужен на наборе.

    Без базиса набор требует ро-Полларда на радикандах до 246 бит; базис
    сводит максимум к 117 битам, и число запусков ро-Полларда падает. Это
    ЕДИНСТВЕННОЕ, что меняет базис, — ответы проверены соседними тестами.
    """

    def rho_calls(*, seed_first):
        reset_factorization_memory()
        calls = []
        original = canon._pollard_rho

        def counting(value):
            calls.append(value)
            return original(value)

        canon._pollard_rho = counting
        try:
            if seed_first:
                _seed_factorization_basis(tuple(sorted(FIELD_UNIVERSE)))
            for radicand in FIELD_RADICANDS:
                _factorize(radicand)
        finally:
            canon._pollard_rho = original
        return calls

    without = rho_calls(seed_first=False)
    with_basis = rho_calls(seed_first=True)

    assert len(with_basis) < len(without)
    assert max(value.bit_length() for value in with_basis) < 130
    assert max(value.bit_length() for value in without) > 200


@pytest.mark.parametrize("seed", (1, 7, 12345))
def test_random_sets_with_known_factorization_survive_the_basis(seed):
    """Случайные наборы с ИЗВЕСТНЫМ разложением: базис не меняет ответ.

    Числа собираются из общего пула простых, поэтому радиканды намеренно
    делят множители — именно этот случай базис и обслуживает.
    """

    rng = random.Random(seed)
    pool = [
        2,
        3,
        5,
        7,
        1_000_003,
        1_000_033,
        2_147_483_647,
        67_280_421_310_721,
    ]
    expected = []
    values = []
    for _ in range(8):
        powers = {
            prime: rng.randrange(0, 4)
            for prime in rng.sample(pool, rng.randrange(3, len(pool)))
        }
        powers = {p: e for p, e in powers.items() if e}
        if not powers:
            powers = {pool[0]: 1}
        value = 1
        for prime, power in powers.items():
            value *= prime**power
        values.append(value)
        expected.append(tuple(sorted(powers.items())))

    reset_factorization_memory()
    _seed_factorization_basis(tuple(sorted(values)))
    for value, factors in zip(values, expected, strict=True):
        assert tuple(sorted(_factorize(value).items())) == factors
        outside, inside = squarefree_split(value)
        assert outside * outside * inside == value
        assert prime_support(inside) == tuple(
            sorted(prime for prime, power in factors if power % 2)
        )


def test_coprime_basis_generates_exactly_the_same_primes():
    """Тождество, на котором стоит Р2: базис порождает ту же поддержку."""

    basis = _coprime_basis(FIELD_UNIVERSE)

    reset_factorization_memory()
    from_basis: set[int] = set()
    for atom in basis:
        from_basis.update(_factorize(atom))

    reset_factorization_memory()
    from_set: set[int] = set()
    for radicand in FIELD_UNIVERSE:
        from_set.update(_factorize(radicand))

    assert from_basis == from_set
    product = 1
    for atom in basis:
        product *= atom
    for radicand in FIELD_UNIVERSE:
        remainder = radicand
        for atom in basis:
            while remainder % atom == 0:
                remainder //= atom
        assert remainder == 1
    assert max(atom.bit_length() for atom in basis) < 130
    assert max(value.bit_length() for value in FIELD_UNIVERSE) > 240
    assert product > 1


def test_memory_reset_returns_the_same_answers_again():
    """Сброс памяти — сброс цены, а не ответа."""

    first = _observed(FIELD_RADICANDS, seed_first=True)
    reset_factorization_memory()
    second = _observed(FIELD_RADICANDS, seed_first=True)
    reset_factorization_memory()
    third = _observed(FIELD_RADICANDS, seed_first=False)

    assert first == second == third
