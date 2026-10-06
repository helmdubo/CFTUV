"""Нативная СТОИМОСТЬ точных операций (`native/cftuv-core`: `exact`, `session`; шим `cftuv_native.cost`) равна эталону на Python.

Эталон — `kernel/src/cftuv_envelope/exact_sqrt_sum.py`: `SqrtSumV1.sign`, `divided_by`, `_divide_with_prime_universe`, `radical`,
`radical_sum`, `prime_universe_remembered`, `squarefree_split`, `prime_support`. Ответ операции — не только значение: цена (шесть
статей `ExactWorkBudgetV1`), счётчики `SIGN_COUNTS`, `UNBUDGETED_WORK` (вызов без бюджета), четыре таблицы памяти канонизации С ПОРЯДКОМ
(`_KNOWN_PRIMES`+`_KNOWN_PRIME_SET`, `_FACTORIZATION_MEMO` с LRU, `_SQUAREFREE_MEMO`, `_PRIME_SUPPORT_MEMO`), запись `store` и точная
деталь исключения исчерпания. Всё это сверяется ПОСЛЕ КАЖДОЙ операции: эталон прогоняет цепочку операций на живых объектах модуля,
состояние возвращается в исходное, шим прогоняет ту же цепочку (нативная сессия, зеркало таблиц, журнал изменений, применённый к
настоящим словарям НА МЕСТЕ), и пошаговые снимки сравниваются. Равенство в начале цепочки и после каждого шага означает, что каждая
операция шла от одного и того же состояния.

Операнды: 1. СИНТЕТИЧЕСКИЕ — суммы, знак которых оболочка 64 бит не решает (сопряжение), знаменатели с несколькими простыми, простые за
`i128`, полупростые с настоящей работой ро-Полларда, рациональные радиканды, нулевые, отрицательные; 2. НАСТОЯЩИЕ — вызовы, которые ядро
делает внутри `coverage._coverage_at` и `clip.clip_geometry` (записи корпуса `E:\\cftuv_native_corpus\\`, воспроизведённые
эталоном с записывающими обёртками); 3. свипы потолка (`cap = start .. start + цена + 1`: деталь исчерпания и частичное состояние на
КАЖДОЙ границе); 4. вызовы без бюджета; 5. сдвиг таблиц питоном МЕЖДУ нативными вызовами (добавление, LRU, вытеснение на 8192,
`reset_factorization_memory`, `isolated_factorization_memory`, подмена значения).

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`).
"""

from __future__ import annotations

import contextlib
import importlib.util
import json
import math
import os
import random
import sys
import time
from dataclasses import dataclass, field
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
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка нативной стоимости с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "coverage")

from cftuv_native import cost as native_cost  # noqa: E402
from cftuv_native import numbers_oracle as oracle  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402

COMPARED = [0]
#: Операции, на которых свипы потолка видели исчерпание: журнал границ, чтобы «зелёный» не был пустым.
SWEPT_OPERATIONS: set = set()
SMALL_PRIMES = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97)
#: Простые Мерсенна за `i128`: радиканды в сотни бит, доказательство простоты стоит настоящей работы.
BIG_PRIMES = (2**61 - 1, 2**89 - 1, 2**107 - 1, 2**127 - 1)
ARTICLE_NAMES = native_cost.ARTICLES
COUNT_NAMES = native_cost.COUNT_KEYS


@pytest.fixture(autouse=True)
def _kernel_process_state_is_given_back():
    """Эталон и шим пишут в процессные счётчики и память ядра; тест их не оставляет."""

    counts = dict(exact.SIGN_COUNTS)
    unbudgeted = exact.UNBUDGETED_WORK.spent_by_article()
    with exact.isolated_factorization_memory():
        yield
    exact.SIGN_COUNTS.update(counts)
    for name, value in zip(ARTICLE_NAMES, unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)
    oracle.clear_oracle_state()


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(f"native cost: {COMPARED[0]} operations compared with the Python oracle after every step")


@pytest.fixture()
def mirror():
    return cftuv_native.new_mirror()


# --------------------------------------------------------------------------
# Состояние процесса: снимок, установка, наблюдение
# --------------------------------------------------------------------------

COMPONENTS = ("known_primes", "known_prime_set", "factorization", "squarefree", "support", "sign_counts", "unbudgeted", "budget", "store")


def capture(budget, store) -> tuple:
    """Всё, что операция пишет мимо результата: таблицы С ПОРЯДКОМ, счётчики знака, неоплаченное, статьи бюджета, `store`."""

    return (
        tuple(exact._KNOWN_PRIMES),
        frozenset(exact._KNOWN_PRIME_SET),
        tuple(exact._FACTORIZATION_MEMO.items()),
        tuple(exact._SQUAREFREE_MEMO.items()),
        tuple(exact._PRIME_SUPPORT_MEMO.items()),
        tuple(exact.SIGN_COUNTS.values()),
        exact.UNBUDGETED_WORK.spent_by_article(),
        None if budget is None else budget.spent_by_article(),
        None if store is None else tuple(store.items()),
    )


def first_difference(want: tuple, got: tuple) -> str:
    for name, left, right in zip(COMPONENTS, want, got):
        if left != right:
            if isinstance(left, tuple) and isinstance(right, tuple):
                for position, (a, b) in enumerate(zip(left, right)):
                    if a != b:
                        return f"{name}[{position}]: oracle {_clip(a)} native {_clip(b)} (lengths {len(left)} / {len(right)})"
                return f"{name}: lengths {len(left)} (oracle) / {len(right)} (native)"
            return f"{name}: oracle {_clip(left)} native {_clip(right)}"
    return "components equal"


def _clip(value, limit: int = 140) -> str:
    text = repr(value)
    return text if len(text) <= limit else text[:limit] + f"...(+{len(text) - limit})"


@dataclass
class Snapshot:
    primes: list
    factorization: list
    squarefree: list
    support: list
    counts: dict
    unbudgeted: tuple


def take_snapshot() -> Snapshot:
    return Snapshot(
        list(exact._KNOWN_PRIMES),
        list(exact._FACTORIZATION_MEMO.items()),
        list(exact._SQUAREFREE_MEMO.items()),
        list(exact._PRIME_SUPPORT_MEMO.items()),
        dict(exact.SIGN_COUNTS),
        exact.UNBUDGETED_WORK.spent_by_article(),
    )


def put_snapshot(snapshot: Snapshot) -> None:
    """Ставит процесс в снимок НА МЕСТЕ (словари и список модуля не пересоздаются)."""

    exact.reset_factorization_memory()
    exact._KNOWN_PRIMES.extend(snapshot.primes)
    exact._KNOWN_PRIME_SET.update(snapshot.primes)
    exact._FACTORIZATION_MEMO.update(snapshot.factorization)
    exact._SQUAREFREE_MEMO.update(snapshot.squarefree)
    exact._PRIME_SUPPORT_MEMO.update(snapshot.support)
    exact.SIGN_COUNTS.update(snapshot.counts)
    for name, value in zip(ARTICLE_NAMES, snapshot.unbudgeted):
        setattr(exact.UNBUDGETED_WORK, name, value)


def cold_snapshot() -> Snapshot:
    snapshot = take_snapshot()
    return Snapshot([], [], [], [], snapshot.counts, snapshot.unbudgeted)


def make_budget(spec):
    """`spec`: `None` (вызов без бюджета) или `(cap, статьи, стадия, домен, superlevel)`; `cap=None` — эталонный режим."""

    if spec is None:
        return None
    cap, articles = spec[0], spec[1]
    budget = exact.ExactWorkBudgetV1(
        mode=exact.ExactWorkBudgetModeV1.BOUNDED if cap is not None else exact.ExactWorkBudgetModeV1.UNLIMITED_REFERENCE,
        cap=cap,
        stage=spec[2] if len(spec) > 2 else "NATIVE_COST",
        domain_id=spec[3] if len(spec) > 3 else "d0",
        superlevel=spec[4] if len(spec) > 4 else "L1",
    )
    for name, value in zip(ARTICLE_NAMES, articles):
        setattr(budget, name, value)
    return budget


def bounded(cap: int | None, articles=(0, 0, 0, 0, 0, 0)) -> tuple:
    return (cap, articles)


# --------------------------------------------------------------------------
# Шаги: одна операция = вид + аргументы; эталон и шим исполняют один и тот же шаг
# --------------------------------------------------------------------------


@dataclass
class Step:
    kind: str
    args: tuple
    budgeted: bool = True

    def __repr__(self) -> str:
        return f"{self.kind}({', '.join(_clip(argument, 70) for argument in self.args)}){'' if self.budgeted else ' [no budget]'}"


def oracle_call(step: Step, budget, store):
    a = step.args
    kind = step.kind
    if kind == "sign":
        return a[0].sign(filter_bits=a[1], budget=budget)
    if kind == "difference_sign":
        return a[0].difference_sign(a[1], budget)
    if kind == "divided_by":
        return a[0].divided_by(a[1], budget)
    if kind == "universe_divide":
        return exact._divide_with_prime_universe(a[0], a[1], tuple(a[2]), budget)
    if kind == "generic_divide":
        return exact._divided_by_generic(a[0], a[1], budget)
    if kind == "radical":
        return SqrtSumV1.radical(a[0], a[1], budget)
    if kind == "radical_sum":
        return exact.radical_sum(tuple(tuple(part) for part in a[0]), budget)
    if kind == "prime_universe":
        return exact.prime_universe_remembered(tuple(a[0]), budget, store if a[1] else None)
    if kind == "squarefree_split":
        return exact.squarefree_split(a[0], budget)
    if kind == "prime_support":
        return exact.prime_support(a[0], budget)
    raise AssertionError(kind)


def native_call(mirror, step: Step, budget, store):
    a = step.args
    kind = step.kind
    if kind == "sign":
        return mirror.sign(a[0], filter_bits=a[1], budget=budget)
    if kind == "difference_sign":
        return mirror.difference_sign(a[0], a[1], budget)
    if kind == "divided_by":
        return mirror.divided_by(a[0], a[1], budget)
    if kind == "universe_divide":
        return mirror.divide_with_prime_universe(a[0], a[1], tuple(a[2]), budget)
    if kind == "generic_divide":
        return mirror.divided_by_generic(a[0], a[1], budget)
    if kind == "radical":
        return mirror.radical(a[0], a[1], budget)
    if kind == "radical_sum":
        return mirror.radical_sum(tuple(tuple(part) for part in a[0]), budget)
    if kind == "prime_universe":
        return mirror.prime_universe_remembered(tuple(a[0]), budget, store if a[1] else None)
    if kind == "squarefree_split":
        return mirror.squarefree_split(a[0], budget)
    if kind == "prime_support":
        return mirror.prime_support(a[0], budget)
    raise AssertionError(kind)


def attempt(function):
    try:
        return ("ok", function())
    except Exception as error:  # noqa: BLE001 - исключение операции — часть её исхода
        return ("raise", type(error), str(error))


def exactly(left, right) -> bool:
    """Строгое равенство результатов: типы считаются (`int` не `Fraction`, кортеж не список)."""

    if type(left) is not type(right):
        return False
    if isinstance(left, SqrtSumV1):
        return oracle.same(left, right)
    if isinstance(left, (tuple, list)):
        return len(left) == len(right) and all(exactly(a, b) for a, b in zip(left, right))
    return left == right


def same_outcome(want: tuple, got: tuple) -> bool:
    if want[0] != got[0]:
        return False
    if want[0] == "ok":
        return exactly(want[1], got[1])
    return want[1] is got[1] and want[2] == got[2]


def run_trace(steps, spec, runner, store_seed=None) -> list:
    """Исполняет шаги подряд на ОДНОМ бюджете и ОДНОМ `store`; после каждого шага — исход и снимок состояния."""

    budget = make_budget(spec)
    store = None if store_seed is None else dict(store_seed)
    observations = []
    for step in steps:
        outcome = attempt(lambda: runner(step, budget if step.budgeted else None, store))
        observations.append((outcome, capture(budget, store)))
    return observations


def compare_run(steps, mirror, spec=None, snapshot=None, store_seed=None, label: str = ""):
    """Эталон и шим идут по `steps` от ОДНОГО состояния; после каждого шага исход и состояние равны. Возвращает исходы эталона."""

    snapshot = snapshot or take_snapshot()
    put_snapshot(snapshot)
    expected = run_trace(steps, spec, oracle_call, store_seed)
    put_snapshot(snapshot)
    actual = run_trace(steps, spec, lambda step, budget, store: native_call(mirror, step, budget, store), store_seed)
    for index, (step, want, got) in enumerate(zip(steps, expected, actual)):
        where = f"{label} шаг {index}: {step}"
        assert same_outcome(want[0], got[0]), f"{where}\n  эталон:   {_clip(want[0], 400)}\n  нативное: {_clip(got[0], 400)}"
        assert want[1] == got[1], f"{where}\n  состояние после: {first_difference(want[1], got[1])}"
    COMPARED[0] += len(steps)
    return [outcome for outcome, _ in expected]


def total_cost(steps, snapshot, store_seed=None) -> int:
    """Цена цепочки по эталону на неограниченном бюджете."""

    put_snapshot(snapshot)
    budget = make_budget(bounded(None))
    store = None if store_seed is None else dict(store_seed)
    for step in steps:
        attempt(lambda: oracle_call(step, budget if step.budgeted else None, store))
    return budget.spent


def cap_values(start: int, cost: int) -> list:
    """Потолки `start .. start+cost+1`: все у начала, геометрическая сетка, все у конца (там и лежит граница отказа)."""

    values = set(range(start, start + min(cost, 36) + 1))
    step = 1
    while step < cost:
        values.add(start + step)
        values.add(start + (step * 3) // 2)
        step *= 2
    values.update(range(max(start, start + cost - 8), start + cost + 2))
    return sorted(values)


def sweep_caps(steps, mirror, label: str, start=(0, 0, 0, 0, 0, 0), snapshot=None, store_seed=None):
    """Для каждого потолка из `cap_values`: деталь исчерпания, частичные статьи и частичное состояние равны эталону. Возвращает число отказов."""

    snapshot = snapshot or take_snapshot()
    cost = total_cost(steps, snapshot, store_seed)
    refused = 0
    for cap in cap_values(sum(start), cost):
        outcomes = compare_run(steps, mirror, bounded(cap, start), snapshot, store_seed, f"{label} cap={cap}")
        refused += sum(1 for outcome in outcomes if outcome[0] == "raise")
        for outcome in outcomes:
            if outcome[0] == "raise" and outcome[2].startswith("EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED"):
                SWEPT_OPERATIONS.add(outcome[2].split("operation=")[1].split()[0])
    return cost, refused


# --------------------------------------------------------------------------
# Генераторы операндов
# --------------------------------------------------------------------------


def _is_prime(n: int) -> bool:
    if n < 2:
        return False
    for small in SMALL_PRIMES:
        if n % small == 0:
            return n == small
    d, r = n - 1, 0
    while d % 2 == 0:
        d //= 2
        r += 1
    for base in (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37):
        x = pow(base, d, n)
        if x in (1, n - 1):
            continue
        for _ in range(r - 1):
            x = x * x % n
            if x == n - 1:
                break
        else:
            return False
    return True


def prime_above(n: int) -> int:
    n |= 1
    while not _is_prime(n):
        n += 2
    return n


class Gen:
    def __init__(self, seed: int) -> None:
        self.rng = random.Random(seed)
        self.mid_primes = [prime_above(self.rng.getrandbits(bits)) for bits in (17, 20, 23, 26, 30)]

    def prime_pool(self, small: int = 4, mid: int = 0, big: int = 0) -> list:
        pool = self.rng.sample(SMALL_PRIMES, small)
        pool += self.rng.sample(self.mid_primes, mid)
        pool += self.rng.sample(BIG_PRIMES, big)
        return pool

    def radicands(self, pool: list, count: int, max_primes: int = 3) -> list:
        found = {1}
        attempts = 0
        while len(found) < count + 1 and attempts < 200:
            attempts += 1
            chosen = self.rng.sample(pool, self.rng.randint(1, min(max_primes, len(pool))))
            found.add(math.prod(chosen))
        return sorted(found)

    def near_zero(self, radicands: list, k: int | None = None) -> SqrtSumV1:
        """`sum c_m sqrt(m) - r` с рациональным `r`, приближающим сумму до `2^-k`: оболочка 64 бит такой знак не решает."""

        k = k or self.rng.randint(66, 150)
        coefficients = [self.rng.choice((-1, 1)) * self.rng.randint(1, 30) for _ in radicands]
        scaled = 0
        for m, c in zip(radicands, coefficients):
            root = math.isqrt(m << (2 * k))
            scaled += c * root
        shift = self.rng.choice((-1, 0, 1, 3))
        approximation = Fraction(scaled, 1 << k) + Fraction(shift, 1 << (k + 4))
        terms = {m: Fraction(c) for m, c in zip(radicands, coefficients)}
        terms[1] = terms.get(1, Fraction(0)) - approximation
        return SqrtSumV1(tuple(sorted((m, c) for m, c in terms.items() if c)))

    def sum(self, radicands: list, max_terms: int = 4, denominators: bool = True) -> SqrtSumV1:
        count = self.rng.randint(1, min(max_terms, len(radicands)))
        chosen = sorted(self.rng.sample(radicands, count))
        terms = []
        for m in chosen:
            numerator = self.rng.choice((-1, 1)) * self.rng.randint(1, 60)
            denominator = self.rng.choice((1, 1, 2, 3, 4, 7, 12)) if denominators else 1
            terms.append((m, Fraction(numerator, denominator)))
        return SqrtSumV1(tuple(terms))

    def irrational(self, radicands: list, max_terms: int = 4) -> SqrtSumV1:
        """Сумма, у которой есть радиканд выше единицы (делителю нужно сопряжение)."""

        high = [m for m in radicands if m > 1]
        while True:
            value = self.sum(radicands, max_terms)
            if any(m in high for m, _ in value.terms):
                return value


def q_values(gen: Gen, pool: list, count: int = 4) -> list:
    """Квадраты скоростей фронта в виде дробей: числитель и знаменатель — произведения простых."""

    values = []
    for _ in range(count):
        numerator = math.prod(gen.rng.sample(pool, gen.rng.randint(1, min(3, len(pool)))))
        denominator = math.prod(gen.rng.sample(pool, gen.rng.randint(0, min(2, len(pool)))))
        values.append(Fraction(numerator * gen.rng.choice((1, 4, 9)), denominator or 1))
    return values


def mixed_steps(gen: Gen, count: int = 8, mid: int = 0, big: int = 0) -> list:
    """Цепочка из всех видов шагов над общим набором радикандов: память работает между шагами."""

    pool = gen.prime_pool(small=5, mid=mid, big=big)
    radicands = gen.radicands(pool, 8)
    steps = []
    for _ in range(count):
        kind = gen.rng.choice(("sign", "sign", "divided_by", "divided_by", "universe_divide", "radical", "radical_sum", "squarefree_split", "prime_support"))
        if kind == "sign":
            steps.append(Step("sign", (gen.near_zero(gen.rng.sample(radicands[1:], min(3, len(radicands) - 1)) or radicands[:1]), gen.rng.choice((64, 64, 64, 8, 0)))))
        elif kind == "divided_by":
            steps.append(Step("divided_by", (gen.sum(radicands), gen.irrational(radicands))))
        elif kind == "universe_divide":
            denominator = gen.irrational(radicands)
            universe = sorted({p for p in pool if any(m % p == 0 for m, _ in denominator.terms)})
            if gen.rng.random() < 0.3 and len(universe) > 1:
                universe.pop(gen.rng.randrange(len(universe)))
            steps.append(Step("universe_divide", (gen.sum(radicands), denominator, universe)))
        elif kind == "radical":
            steps.append(Step("radical", (gen.rng.choice((1, -2, Fraction(3, 5), 7)), gen.rng.choice((2, 12, 45, Fraction(3, 4), Fraction(6, 25), math.prod(pool[:2]) * 9)))))
        elif kind == "radical_sum":
            parts = [(gen.rng.choice((1, 2, -3, Fraction(1, 2))), gen.rng.choice((2, 8, 18, 50, Fraction(5, 3), Fraction(2, 9), math.prod(pool[:3])))) for _ in range(gen.rng.randint(1, 4))]
            steps.append(Step("radical_sum", (parts,)))
        elif kind == "squarefree_split":
            steps.append(Step("squarefree_split", (gen.rng.choice((0, 1, 2, 72, math.prod(pool[:3]) * pool[0] ** 2, gen.rng.getrandbits(40) + 2)),)))
        else:
            steps.append(Step("prime_support", (gen.rng.choice((0, 1, 6, math.prod(pool[:3]), gen.rng.getrandbits(36) + 2)),)))
    return steps


# --------------------------------------------------------------------------
# Синтетика: каждая операция на живых объектах модуля
# --------------------------------------------------------------------------


@pytest.mark.parametrize("seed", range(5))
def test_signs_that_need_the_conjugation_cost_what_python_costs(mirror, seed):
    """Знак суммы, у которой оболочка 64 бит бессильна: рекурсия по простым, носитель через `prime_support`, дискриминант."""

    gen = Gen(100 + seed)
    steps = []
    for round_index in range(12):
        pool = gen.prime_pool(small=4, mid=round_index % 3, big=1 if round_index % 4 == 0 else 0)
        radicands = gen.radicands(pool, gen.rng.randint(1, 5))
        steps.append(Step("sign", (gen.near_zero(radicands[1:] or radicands), 64)))
    snapshot = cold_snapshot()
    outcomes = compare_run(steps, mirror, bounded(None), snapshot, label="sign")
    conjugated = exact.SIGN_COUNTS["closed_by_conjugation"] - snapshot.counts["closed_by_conjugation"]
    assert conjugated == len(steps), f"сопряжение понадобилось {conjugated} раз из {len(steps)}: сверка ветки неполна"
    assert all(outcome[0] == "ok" for outcome in outcomes)
    assert {outcome[1] for outcome in outcomes} == {1, -1}, "знаки обоих направлений должны встретиться"


@pytest.mark.parametrize("seed", range(5))
def test_divisions_cost_what_python_costs(mirror, seed):
    """`divided_by` (целочисленный цикл сопряжений) и `_divide_with_prime_universe` (полный и неполный носитель)."""

    gen = Gen(200 + seed)
    steps = []
    for round_index in range(10):
        pool = gen.prime_pool(small=5, mid=round_index % 3, big=1 if round_index % 5 == 0 else 0)
        radicands = gen.radicands(pool, 7)
        denominator = gen.irrational(radicands)
        supports = sorted({p for p in pool if any(m % p == 0 for m, _ in denominator.terms)})
        steps.append(Step("divided_by", (gen.sum(radicands), denominator)))
        steps.append(Step("universe_divide", (gen.sum(radicands), denominator, supports)))
        steps.append(Step("universe_divide", (gen.sum(radicands), denominator, supports[:-1])))  # носитель неполон: откат на `divided_by`
        steps.append(Step("universe_divide", (gen.sum(radicands), gen.sum(radicands, denominators=False), supports)))
    for budget in (bounded(None), bounded(1 << 23)):
        compare_run(steps, mirror, budget, cold_snapshot(), label="division")


@pytest.mark.parametrize("seed", range(3))
def test_difference_signs_cost_what_python_costs(mirror, seed):
    """`difference_sign`: фильтр по целым формам без разности (счётчики как у `sign`), а не решив — весь путь `(a - b).sign`."""

    gen = Gen(600 + seed)
    steps = []
    for round_index in range(14):
        pool = gen.prime_pool(small=4, mid=round_index % 2)
        radicands = gen.radicands(pool, 5)
        base = gen.sum(radicands)
        nudged = base + SqrtSumV1.rational(Fraction(gen.rng.choice((0, 1, -1)), 1 << gen.rng.randint(70, 140)))
        steps += [
            Step("difference_sign", (base, gen.sum(radicands))),  # решает фильтр
            Step("difference_sign", (base, nudged)),  # почти равны: идёт в точный знак
            Step("difference_sign", (base, base)),  # равны: нулевое множество членов
            Step("difference_sign", (gen.near_zero(radicands[1:3]) + SqrtSumV1.rational(5), SqrtSumV1.rational(5))),
        ]
    snapshot = cold_snapshot()
    compare_run(steps, mirror, bounded(None), snapshot, label="difference sign")
    assert exact.SIGN_COUNTS["closed_by_conjugation"] > snapshot.counts["closed_by_conjugation"], "ни одна разность не дошла до сопряжения"
    compare_run(steps[:8], mirror, bounded(60), snapshot, label="difference sign capped")


def test_zero_and_rational_divisors_and_the_zero_numerator(mirror):
    two, three = SqrtSumV1.radical(1, 2), SqrtSumV1.radical(1, 3)
    zero = SqrtSumV1(())
    rational = SqrtSumV1.rational(Fraction(-3, 7))
    steps = [
        Step("divided_by", (two, zero)),
        Step("divided_by", (zero, two + three)),
        Step("divided_by", (zero, rational)),
        Step("divided_by", (two + three, rational)),
        Step("divided_by", (SqrtSumV1(((1, 5), (2, 3))), SqrtSumV1(((1, 2), (2, -3), (3, 5))))),  # коэффициенты `int`
        Step("universe_divide", (two, zero, [2, 3])),
        Step("universe_divide", (zero, two - three, [2, 3])),
        Step("universe_divide", (two, two + three, [])),
        Step("universe_divide", (two, rational, [])),
    ]
    outcomes = compare_run(steps, mirror, bounded(None), cold_snapshot(), label="edge divisions")
    assert [outcome[0] for outcome in outcomes[:1]] == ["raise"] and outcomes[0][2] == "деление на точный ноль"
    assert outcomes[5][0] == "raise" and outcomes[1][0] == "ok"


@pytest.mark.parametrize("seed", range(3))
def test_the_generic_division_fallback_costs_what_python_costs(mirror, seed):
    """`_divided_by_generic` (запасной путь целочисленного цикла: цепочка `радикал -> произведение`) достигается прямым вызовом."""

    gen = Gen(400 + seed)
    steps = []
    for round_index in range(8):
        pool = gen.prime_pool(small=5, mid=round_index % 3, big=1 if round_index == 3 else 0)
        radicands = gen.radicands(pool, 6)
        steps.append(Step("generic_divide", (gen.sum(radicands), gen.irrational(radicands))))
        steps.append(Step("divided_by", (gen.sum(radicands), gen.irrational(radicands))))
        steps.append(Step("generic_divide", (gen.sum(radicands), SqrtSumV1.rational(Fraction(-5, 3)))))
    steps.append(Step("generic_divide", (SqrtSumV1(()), SqrtSumV1.radical(1, 2) + SqrtSumV1.radical(1, 3))))
    for budget in (bounded(None), bounded(1 << 23), None):
        compare_run([Step(step.kind, step.args, budget is not None) for step in steps], mirror, budget, cold_snapshot(), label="generic division")
    cost, refused = sweep_caps(steps[:6], mirror, "generic division sweep", snapshot=cold_snapshot())
    assert refused > 5


def test_integral_rational_radicands_and_coefficients_are_taken_as_integers(mirror):
    """`Fraction(8)` как радиканд — `int(radicand)` (ветка рационального радиканда не выполняется); `Fraction` коэффициента любого вида."""

    steps = [
        Step("radical", (1, Fraction(8))),
        Step("radical", (Fraction(5), 8)),
        Step("radical", (Fraction(-7, 3), Fraction(50))),
        Step("radical", (Fraction(0), Fraction(50))),
        Step("radical", (3, Fraction(0))),
        Step("radical_sum", ([(Fraction(1), Fraction(8)), (2, 2), (Fraction(1, 2), Fraction(18, 1))],)),
        Step("radical_sum", ([(Fraction(3, 2), Fraction(4, 9)), (1, Fraction(9, 4)), (-1, Fraction(1, 36))],)),
    ]
    compare_run(steps, mirror, bounded(None), cold_snapshot(), label="integral fractions")


def test_the_python_and_the_native_side_interleave_on_the_same_tables(mirror):
    """Часть шагов исполняет питон (настоящие вытеснения, LRU, вставки в реестр), часть — нативная сторона: итог цепочки тот же, что у чистого питона."""

    gen = Gen(81)
    base = cold_snapshot()
    put_snapshot(base)
    _fill_with_small_numbers(8185)
    _fill_registry(exact._KNOWN_PRIME_REGISTRY_ENTRIES - 3)
    capacity = take_snapshot()
    pool = gen.prime_pool(small=5, mid=2)
    fresh = [prime_above(1 << bits) for bits in (30, 31, 32, 33, 34)]
    steps = []
    for index in range(16):
        steps.append(Step("prime_support", (fresh[index % 5] * pool[index % 5] * (index + 2),)))
        steps.append(Step("squarefree_split", (fresh[(index + 2) % 5] * 36 * pool[(index + 1) % 5],)))
        steps.append(Step("divided_by", (SqrtSumV1.radical(1, 6), SqrtSumV1.radical(1, 2 * fresh[index % 5]) + SqrtSumV1.radical(1, 3))))
    for start in (capacity, cold_snapshot()):
        put_snapshot(start)
        want = run_trace(steps, bounded(None), oracle_call)
        for assignment in range(4):
            put_snapshot(start)
            rng = random.Random(assignment)
            budget = make_budget(bounded(None))
            for index, step in enumerate(steps):
                runner = oracle_call if rng.random() < 0.5 else (lambda step, budget, store: native_call(mirror, step, budget, store))
                outcome = attempt(lambda: runner(step, budget, None))
                assert same_outcome(outcome, want[index][0]), f"интерливинг {assignment} шаг {index}: {step}"
                assert capture(budget, None) == want[index][1], f"интерливинг {assignment} шаг {index}: {first_difference(want[index][1], capture(budget, None))}"
    COMPARED[0] += len(steps) * 8


def test_radicals_negative_radicands_and_rational_radicands(mirror):
    big = prime_above(1 << 28) * prime_above(1 << 29)
    steps = [
        Step("radical", (1, 2)),
        Step("radical", (0, 2)),
        Step("radical", (2, 0)),
        Step("radical", (Fraction(3, 4), Fraction(9, 16))),
        Step("radical", (-5, Fraction(1, 3))),
        Step("radical", (1, Fraction(12, 7))),
        Step("radical", (1, -4)),
        Step("radical", (1, Fraction(-3, 5))),
        Step("radical", (7, big)),
        Step("radical", (Fraction(1, 3), big * 36)),
        Step("radical_sum", ([(1, 2), (1, 8), (-3, 18), (Fraction(1, 2), 2)],)),
        Step("radical_sum", ([(2, 50), (-10, 2)],)),  # сокращение до нуля
        Step("radical_sum", ([(1, Fraction(5, 3)), (Fraction(2, 5), Fraction(15, 4)), (0, 7), (3, 0)],)),
        Step("radical_sum", ([(1, 2), (1, -2)],)),
        Step("radical_sum", ([(1, big), (1, big * 4)],)),
        Step("radical_sum", ([],)),
        Step("squarefree_split", (-4,)),
        Step("squarefree_split", (0,)),
        Step("squarefree_split", (1,)),
        Step("squarefree_split", (big * 9,)),
        Step("prime_support", (-5,)),
        Step("prime_support", (1,)),
        Step("prime_support", (big,)),
    ]
    outcomes = compare_run(steps, mirror, bounded(None), cold_snapshot(), label="radicals")
    raised = [outcome[2] for outcome in outcomes if outcome[0] == "raise"]
    # рациональный радиканд `-3/5` сводится к целому `-3*5` до отказа: текст называет ЦЕЛОЕ
    assert raised == ["под корнем -4", "под корнем -15", "под корнем -2", "под корнем -4"], raised
    for budget in (None, bounded(1 << 23)):
        compare_run([Step(step.kind, step.args, budget is not None) for step in steps], mirror, budget, cold_snapshot(), label="radicals-budget")


def test_prime_universe_store_hit_miss_and_failure(mirror):
    gen = Gen(31)
    pool = gen.prime_pool(small=5, mid=2, big=1)
    first, second = q_values(gen, pool, 5), q_values(gen, pool, 4)
    steps = [
        Step("prime_universe", (first, True)),
        Step("prime_universe", (first, True)),  # попадание: вызов записанных разложений
        Step("prime_universe", (second, True)),
        Step("prime_universe", (first, False)),
        Step("prime_universe", (second + [Fraction(-3, 7)], True)),  # отрицательное q: отказ, в store не пишется
        Step("prime_universe", ([Fraction(0), 0, 5], True)),
        Step("prime_universe", ([], True)),
        Step("prime_universe", ([1, Fraction(4, 9)], False)),
    ]
    outcomes = compare_run(steps, mirror, bounded(None), cold_snapshot(), store_seed={}, label="universe")
    assert outcomes[4][0] == "raise" and outcomes[4][2] == "под корнем -3/7"
    # тёплая память и `store`, построенный в другом процессе: попадание кладёт разложения на место
    snapshot = take_snapshot()
    put_snapshot(cold_snapshot())
    store = {}
    exact.prime_universe_remembered(tuple(first), None, store)
    put_snapshot(snapshot)
    compare_run(steps[:3], mirror, bounded(None), snapshot, store_seed=store, label="universe-hit-warm")
    compare_run(steps[:3], mirror, None, cold_snapshot(), store_seed=store, label="universe-hit-cold-unbudgeted")


def test_calls_without_a_budget_go_to_the_unbudgeted_telemetry(mirror):
    gen = Gen(41)
    steps = [Step(step.kind, step.args, False) for step in mixed_steps(gen, 14, mid=2)]
    before = exact.UNBUDGETED_WORK.spent_by_article()
    compare_run(steps, mirror, None, cold_snapshot(), label="unbudgeted")
    assert exact.UNBUDGETED_WORK.spent_by_article() != before, "ни одна единица не ушла в телеметрию: сверка пуста"
    # смешанный сценарий: часть шагов без бюджета, часть на названном
    mixed = [Step(step.kind, step.args, index % 2 == 0) for index, step in enumerate(mixed_steps(Gen(42), 14, mid=1))]
    compare_run(mixed, mirror, bounded(1 << 23), cold_snapshot(), label="mixed-budget")


@pytest.mark.parametrize("seed", range(6))
def test_mixed_chains_keep_cost_equal_between_steps(mirror, seed):
    """Память между шагами: промах первого шага — попадание второго, и цена второго от этого зависит."""

    gen = Gen(300 + seed)
    steps = mixed_steps(gen, 14, mid=seed % 3, big=1 if seed % 2 else 0)
    compare_run(steps, mirror, bounded(None), cold_snapshot(), label="chain")
    compare_run(steps, mirror, bounded(1 << 23, (5, 7, 11, 13, 17, 19)), cold_snapshot(), label="chain-prefilled-budget")


def test_radicands_that_are_not_squarefree_follow_the_oracle(mirror):
    """Оракул принимает и неканонический вход (радикант с квадратом, полный квадрат): знак и деление идут тем же путём и стоят столько же."""

    def make(*pairs):
        return SqrtSumV1(tuple(sorted((m, Fraction(c)) for m, c in pairs)))

    steps = [
        Step("sign", (make((1, -7), (12, 2)), 64)),
        Step("sign", (make((1, -3), (4, 1)), 64)),  # `4` = полный квадрат: значение рационально, но путь тот же
        Step("sign", (make((2, 1), (8, -1), (18, 1)), 0)),
        Step("sign", (make((9, 1), (18, -3), (50, 2)), 8)),
        Step("divided_by", (make((1, 2), (3, 1)), make((1, 1), (12, 1)))),
        Step("divided_by", (make((1, 5)), make((4, 1), (8, 2)))),
        Step("divided_by", (make((2, 1), (3, 1)), make((18, 1), (50, -1)))),
        Step("universe_divide", (make((1, 2)), make((1, 1), (12, 1)), [2, 3])),
        Step("universe_divide", (make((1, 2)), make((2, 1), (8, 3)), [2])),
    ]
    outcomes = compare_run(steps, mirror, bounded(None), cold_snapshot(), label="non-squarefree")
    assert sum(1 for outcome in outcomes if outcome[0] == "ok") >= 6
    # запасной путь на неканоническом знаменателе у эталона не кончается: числа удваиваются каждый круг, питон упирается в память.
    # Нативная сторона отказывает ИМЕНЕМ (круги и разрядность ограничены), а не виснет.
    started = time.perf_counter()
    with pytest.raises(native_cost.NativeDivisionDiverged):
        mirror.divided_by_generic(make((1, 5)), make((2, 1), (8, 2)))
    assert time.perf_counter() - started < 20
    steps = [Step("radical", (1, 12)), Step("generic_divide", (make((1, 2), (3, 1)), make((1, 1), (3, 2)))), Step("sign", (make((1, -2), (3, 1)), 64))]
    compare_run(steps, mirror, bounded(None), take_snapshot(), label="after the refusal")


def test_a_sum_the_extension_cannot_carry_is_refused_by_name_and_the_mirror_survives(mirror):
    zero_radicand = SqrtSumV1(((0, Fraction(1)),))
    unsorted = SqrtSumV1(((3, Fraction(1)), (2, Fraction(1))))
    repeated = SqrtSumV1(((2, Fraction(1)), (2, Fraction(2))))
    zero_coefficient = SqrtSumV1(((2, Fraction(0)),))
    for bad in (zero_radicand, unsorted, repeated, zero_coefficient):
        with pytest.raises(ValueError, match="NonCanonicalSum"):
            mirror.sign(bad)
    with pytest.raises(native_cost.codec.CodecError):
        mirror.sign(SqrtSumV1(((-2, Fraction(1)),)))
    with pytest.raises(ValueError, match="bad arguments"):  # float: Python принял бы его (`Fraction(1.5)`), нативная сторона называет отказ
        mirror.radical(1.5, 2)
    steps = [Step("sign", (Gen(3).near_zero([2, 3, 6]), 64)), Step("radical", (1, 12))]
    compare_run(steps, mirror, bounded(None), cold_snapshot(), label="after refusals")


def test_garbled_cost_requests_never_cross_the_boundary_as_a_panic(mirror):
    """Буфер, который режут и портят, даёт `ValueError` или ответ, но не панику; после отказа сессия пуста, зеркало перезагружается."""

    gen = Gen(13)
    pool = gen.prime_pool(small=4, mid=1)
    radicands = gen.radicands(pool, 5)
    ops = [("EXACT_SIGN", (gen.near_zero(radicands[1:3]), 64)), ("EXACT_DIVIDED_BY", (gen.sum(radicands), gen.irrational(radicands))), ("EXACT_RADICAL_SUM", ([[1, 8], [2, 18]],))]
    exact.reset_factorization_memory()
    exact._FACTORIZATION_MEMO[6] = ((2, 1), (3, 1))
    header = [0, [[False, [], []], [False, [], [[6, [[2, 1], [3, 1]]]]], [False, [], []], [False, [], []]], [None, 0, 0, 0, 0, 0, 0]]
    good = native_cost.codec.encode_request(ops, cost=header)
    session = cftuv_native.new_mirror()._session
    rng = random.Random(1)
    refused = completed = 0
    for round_index in range(400):
        data = bytearray(good)
        kind = round_index % 4
        if kind == 0:
            del data[rng.randrange(5, len(data)) :]
        elif kind == 1:
            for _ in range(rng.randint(1, 4)):
                data[rng.randrange(len(data))] = rng.randrange(256)
        elif kind == 2:
            position = rng.randrange(5, len(data))
            data[position:position] = bytes(rng.randrange(256) for _ in range(rng.randint(1, 6)))
        else:
            data[rng.randrange(5, len(data))] ^= 1 << rng.randrange(8)
        try:
            session.run(bytes(data))
            completed += 1
        except ValueError:
            refused += 1
        except RuntimeError as error:  # noqa: PERF203 - паника ядра: именно её здесь и ловим
            pytest.fail(f"паника пересекла границу на мутации {round_index}: {error}")
    assert refused > 100 and completed > 10, (refused, completed)


# --------------------------------------------------------------------------
# Свипы потолка
# --------------------------------------------------------------------------


def _sweep_scripts() -> dict:
    gen = Gen(7)
    mid = prime_above(1 << 22)
    mid2 = prime_above(1 << 24)
    pool = [2, 3, 5, mid, mid2]
    radicands = [1, 2, 3, 6, 5 * mid, 2 * mid2, 3 * mid * mid2, mid2 * 5]
    two, three, six = SqrtSumV1.radical(1, 2), SqrtSumV1.radical(1, 3), SqrtSumV1.radical(1, 6)
    return {
        "sign": [Step("sign", (gen.near_zero([2, 3, 6, 5 * mid]), 64)), Step("sign", (gen.near_zero([mid2, 2 * mid2, 3]), 64))],
        "sign-big-prime": [Step("sign", (gen.near_zero([2, 2**61 - 1, 3 * (2**61 - 1)]), 64))],
        "division": [Step("divided_by", (gen.sum(radicands), SqrtSumV1(((1, Fraction(3)), (2, Fraction(1, 2)), (3, Fraction(-2)), (5 * mid, Fraction(1)))))), Step("divided_by", (three, two + six))],
        "universe": [Step("prime_universe", (q_values(Gen(1), pool, 4), True)), Step("universe_divide", (two, three + six, [2, 3])), Step("universe_divide", (two, three + six, [2]))],
        "radicals": [Step("radical", (1, mid * mid2)), Step("radical_sum", ([(1, 3 * mid * mid2), (2, 5 * mid2), (1, Fraction(7, 2))],)), Step("squarefree_split", (mid * mid2 * 4 * 9,)), Step("prime_support", (mid * 5 * 2,))],
        "chain": [
            Step("radical", (1, 2 * mid)),
            Step("sign", (gen.near_zero([2, 3, mid * 2]), 64)),
            Step("divided_by", (two, three + SqrtSumV1.radical(1, 2 * mid))),
            Step("prime_support", (3 * mid2,)),
            Step("divided_by", (six, two + three + SqrtSumV1.radical(1, 3 * mid2))),
        ],
    }


@pytest.mark.parametrize("name", list(_sweep_scripts()))
def test_every_cap_gives_the_same_exhaustion_and_the_same_partial_state(mirror, name):
    steps = _sweep_scripts()[name]
    cost, refused = sweep_caps(steps, mirror, f"sweep {name}", snapshot=cold_snapshot(), store_seed={})
    assert cost > 40 and refused > 20, f"{name}: свип слишком мал ({cost} единиц, {refused} отказов): сверка границ пуста"


def test_the_sweeps_met_the_exhaustion_on_every_operation_that_can_pay(mirror):
    """Свипы ниже и выше режут работу на КАЖДОЙ платящей операции: промах носителя, расщепления, простота, ро-Поллард, базис."""

    for name, steps in _sweep_scripts().items():
        sweep_caps(steps, mirror, f"operations {name}", snapshot=cold_snapshot(), store_seed={})
    assert {"PRIME_SUPPORT", "SQUAREFREE_SPLIT", "PRIMALITY", "POLLARD_RHO_BRENT", "COPRIME_BASIS"} <= SWEPT_OPERATIONS, sorted(SWEPT_OPERATIONS)


def test_cap_sweep_over_a_warm_memory_and_a_prefilled_budget(mirror):
    """Начало не с нуля: статьи уже потрачены, память тёплая (часть промахов стала попаданиями) — граница сдвинута."""

    steps = _sweep_scripts()["chain"]
    put_snapshot(cold_snapshot())
    warm = [Step("radical", (1, 2 * prime_above(1 << 22))), Step("prime_support", (3 * prime_above(1 << 24),))]
    run_trace(warm[:1], bounded(None), oracle_call)
    snapshot = take_snapshot()
    cost, refused = sweep_caps(steps, mirror, "sweep warm", start=(100, 20, 3, 2, 5, 0), snapshot=snapshot, store_seed={})
    assert refused > 10 and cost > 20


def test_a_budget_that_is_already_exhausted_fails_at_the_first_spend(mirror):
    gen = Gen(9)
    steps = mixed_steps(gen, 8, mid=1)
    for cap, start in ((0, (0, 0, 0, 0, 0, 0)), (5, (4, 3, 0, 0, 0, 0)), (3, (9, 0, 0, 0, 0, 0))):
        compare_run(steps, mirror, bounded(cap, start), cold_snapshot(), label=f"exhausted cap={cap}")


# --------------------------------------------------------------------------
# Настоящие операнды: вызовы ядра внутри `coverage_at` и `clip_geometry`
# --------------------------------------------------------------------------


def _corpus_directory() -> Path | None:
    base = Path(os.environ.get("CFTUV_NATIVE_CORPUS", "E:/cftuv_native_corpus"))
    found = sorted(path.parent for path in base.glob("*/index.json"))
    return found[-1] if found else None


def _load_corpus_tool():
    cached = sys.modules.get("native_corpus")
    if cached is not None:
        return cached
    spec = importlib.util.spec_from_file_location("native_corpus", ROOT / "tools" / "native_corpus.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["native_corpus"] = module
    spec.loader.exec_module(module)
    return module


class TraceRecorder:
    """Записывающие обёртки вокруг точных операций: только ВНЕШНИЕ вызовы (вложенные — часть внешнего)."""

    def __init__(self, main_budget) -> None:
        self.steps: list = []
        self.depth = 0
        self.main = main_budget

    def budgeted(self, budget) -> bool:
        if budget is None:
            return False
        assert budget is self.main, "вызов с чужим бюджетом: запись не умеет его воспроизвести"
        return True

    def wrap(self, owner, name: str, kind: str, convert):
        original = owner.__dict__[name] if isinstance(owner, type) else getattr(owner, name)
        raw = original.__func__ if isinstance(original, staticmethod) else original

        def recorded(*args, **kwargs):
            top = self.depth == 0
            if top:
                self.steps.append(convert(args, kwargs))
            self.depth += 1
            try:
                return raw(*args, **kwargs)
            finally:
                self.depth -= 1

        return recorded

    @contextlib.contextmanager
    def installed(self):
        patches = []

        def put(owner, name, value):
            patches.append((owner, name, owner.__dict__[name] if isinstance(owner, type) else getattr(owner, name)))
            setattr(owner, name, value)

        def arg(args, kwargs, index, name, default=None):
            return args[index] if len(args) > index else kwargs.get(name, default)

        flat = {
            "_divide_with_prime_universe": ("universe_divide", lambda a, k: Step("universe_divide", (a[0], a[1], tuple(a[2])), self.budgeted(arg(a, k, 3, "budget")))),
            "radical_sum": ("radical_sum", lambda a, k: Step("radical_sum", (tuple(tuple(part) for part in a[0]),), self.budgeted(arg(a, k, 1, "budget")))),
        }
        modules = [module for name, module in list(sys.modules.items()) if name.startswith("cftuv_envelope") and module is not None]
        for name, (kind, convert) in flat.items():
            original = exact.__dict__[name]
            wrapper = self.wrap(exact, name, kind, convert)
            for module in modules:
                if module.__dict__.get(name) is original:
                    put(module, name, wrapper)

        def universe_convert(a, k):
            build = arg(a, k, 3, "build")
            assert build is None or build is exact._prime_universe_from_q_values
            return Step("prime_universe", (tuple(a[0]), arg(a, k, 2, "store") is not None), self.budgeted(arg(a, k, 1, "budget")))

        original = exact.__dict__["prime_universe_remembered"]
        wrapper = self.wrap(exact, "prime_universe_remembered", "prime_universe", universe_convert)
        for module in modules:
            if module.__dict__.get("prime_universe_remembered") is original:
                put(module, "prime_universe_remembered", wrapper)
        put(SqrtSumV1, "sign", self.wrap(SqrtSumV1, "sign", "sign", lambda a, k: Step("sign", (a[0], k.get("filter_bits", 64)), self.budgeted(k.get("budget")))))
        put(SqrtSumV1, "divided_by", self.wrap(SqrtSumV1, "divided_by", "divided_by", lambda a, k: Step("divided_by", (a[0], a[1]), self.budgeted(arg(a, k, 2, "budget")))))
        radical = self.wrap(SqrtSumV1, "radical", "radical", lambda a, k: Step("radical", (a[0], a[1]), self.budgeted(arg(a, k, 2, "budget"))))
        put(SqrtSumV1, "radical", staticmethod(radical))
        try:
            yield
        finally:
            for owner, name, value in reversed(patches):
                setattr(owner, name, value)


def harvest(tool, path: Path):
    """Воспроизводит запись корпуса эталоном под записывающими обёртками: `(состояние до, шаги, store до)`."""

    record = tool.read_record(path)
    before = record.before()
    call = tool.prepare_call(record.op, record.call_blob, before)
    recorder = TraceRecorder(call.budget)
    with recorder.installed():
        tool.execute(call)
    store_seed = None if before.store is None else dict(before.store)
    spec = None if before.budget is None else (before.budget["cap"], tuple(before.budget["articles"]), before.budget["stage"], before.budget["domain_id"], before.budget["superlevel"])
    return before, recorder.steps, store_seed, spec


def _pick_records(directory: Path, per_group: int) -> list:
    rows = json.loads((directory / "index.json").read_text(encoding="utf-8"))["records"]
    groups: dict = {}
    for row in rows:
        groups.setdefault((row["op"], row["mesh"]), []).append(row)
    picked = []
    rng = random.Random(5)
    for key in sorted(groups):
        group = sorted(groups[key], key=lambda row: row["bytes"])
        spread = {0, len(group) // 2, (3 * len(group)) // 4, len(group) - 1}
        chosen = sorted(spread) + [rng.randrange(len(group)) for _ in range(per_group)]
        picked += [directory / group[index]["path"] for index in dict.fromkeys(chosen[: per_group + 2])]
    return picked


def test_real_calls_of_coverage_and_clip_cost_what_python_costs(mirror):
    directory = _corpus_directory()
    if directory is None:
        pytest.skip("корпус не собран (E:/cftuv_native_corpus/*/index.json): настоящие операнды не сверены")
    tool = _load_corpus_tool()
    kinds: dict = {}
    conjugated = 0
    compared = 0
    for path in _pick_records(directory, 3):
        try:
            before, steps, store_seed, spec = harvest(tool, path)
        except Exception as error:  # noqa: BLE001 - запись другой версии: пропуск; пустой итог ниже даст skip
            pytest.fail(f"запись {path.name} не воспроизводится эталоном: {error!r}")
        for step in steps:
            kinds[step.kind] = kinds.get(step.kind, 0) + 1
        snapshot = Snapshot(
            [p for p in before.known_primes], list(before.factorization), list(before.squarefree), list(before.prime_support), dict(before.sign_counts), tuple(before.unbudgeted)
        )
        counts = dict(exact.SIGN_COUNTS)
        compare_run(steps, mirror, spec, snapshot, store_seed, label=path.name)
        conjugated += exact.SIGN_COUNTS["closed_by_conjugation"] - counts["closed_by_conjugation"]
        compared += len(steps)
    assert compared > 1500, kinds
    assert {"sign", "divided_by", "universe_divide", "radical", "prime_universe"} <= set(kinds), kinds
    COMPARED.append(0)
    COMPARED.pop()
    print(f"real operands: {compared} calls {kinds}, signs through the conjugation: {conjugated}")


def test_real_calls_under_every_cap_sweep_a_slice(mirror):
    """Потолок режет настоящую цепочку (коммит `radical`, `prime_universe`, делений): граница на каждом потолке равна эталону."""

    directory = _corpus_directory()
    if directory is None:
        pytest.skip("корпус не собран: настоящие операнды не сверены")
    tool = _load_corpus_tool()
    rows = json.loads((directory / "index.json").read_text(encoding="utf-8"))["records"]
    candidates = [row for row in rows if row["op"] == "coverage_at" and row["mesh"] == "building" and row["bytes"] > 20000][:1]
    candidates += [row for row in rows if row["op"] == "clip_geometry" and row["mesh"] == "sagging_wall"][:1]
    assert candidates
    for row in candidates:
        before, steps, store_seed, spec = harvest(tool, directory / row["path"])
        steps = steps[:60]
        snapshot = Snapshot(list(before.known_primes), list(before.factorization), list(before.squarefree), list(before.prime_support), dict(before.sign_counts), tuple(before.unbudgeted))
        cost, refused = sweep_caps(steps, mirror, f"real {row['id']}", snapshot=snapshot, store_seed=store_seed)
        assert cost >= 0


# --------------------------------------------------------------------------
# Сдвиг таблиц питоном между нативными вызовами
# --------------------------------------------------------------------------


def _probe_steps(gen: Gen, mid: int = 1) -> list:
    pool = gen.prime_pool(small=4, mid=mid)
    radicands = gen.radicands(pool, 6)
    return [
        Step("prime_support", (gen.rng.choice(radicands[1:]) * gen.rng.choice(pool),)),
        Step("squarefree_split", (math.prod(pool[:2]) * pool[0] * 36,)),
        Step("divided_by", (gen.sum(radicands), gen.irrational(radicands))),
        Step("sign", (gen.near_zero(radicands[1:3]), 64)),
    ]


def _fill_with_small_numbers(count: int, start: int = 2) -> None:
    """Заполняет `_FACTORIZATION_MEMO` настоящими разложениями малых чисел (без работы ро-Полларда)."""

    for n in range(start, start + count):
        pairs, rest, p = [], n, 2
        while p * p <= rest:
            if rest % p == 0:
                power = 0
                while rest % p == 0:
                    rest //= p
                    power += 1
                pairs.append((p, power))
            p += 1
        if rest > 1:
            pairs.append((rest, 1))
        exact._FACTORIZATION_MEMO[n] = tuple(pairs)


def _fill_registry(count: int) -> None:
    primes, candidate = [], 2
    while len(primes) < count:
        if all(candidate % p for p in primes if p * p <= candidate):
            primes.append(candidate)
        candidate += 1
    exact._KNOWN_PRIMES.extend(primes)
    exact._KNOWN_PRIME_SET.update(primes)


def _mutate_append(table: dict, key: int, value) -> None:
    table[key] = value


def mutations() -> list:
    """`(имя, мутация настоящих таблиц питоном)`: каждую шим обязан заметить и увидеть как есть."""

    def touch_middle():
        keys = list(exact._FACTORIZATION_MEMO)
        if len(keys) > 3:
            key = keys[len(keys) // 2]
            exact._FACTORIZATION_MEMO[key] = exact._FACTORIZATION_MEMO.pop(key)

    def evict_oldest_and_append():
        if exact._FACTORIZATION_MEMO:
            del exact._FACTORIZATION_MEMO[next(iter(exact._FACTORIZATION_MEMO))]
        exact._FACTORIZATION_MEMO[99991] = ((99991, 1),)

    def delete_middle():
        for table in (exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO):
            keys = list(table)
            if len(keys) > 2:
                del table[keys[len(keys) // 2]]

    def poison_value():
        # значение у существующего ключа меняется на месте: ключи те же, зеркало обязано заметить значение
        for table in (exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO):
            if table:
                key = next(iter(table))
                table[key] = (key, 1) if table is exact._SQUAREFREE_MEMO else (key,)

    def registry_insert():
        for prime in (999983, 1000003):
            if prime not in exact._KNOWN_PRIME_SET:
                from bisect import insort

                insort(exact._KNOWN_PRIMES, prime)
                exact._KNOWN_PRIME_SET.add(prime)

    def registry_drop():
        if len(exact._KNOWN_PRIMES) > 2:
            prime = exact._KNOWN_PRIMES[1]
            exact._KNOWN_PRIMES.remove(prime)
            exact._KNOWN_PRIME_SET.discard(prime)

    def append_fake():
        exact._FACTORIZATION_MEMO[777] = ((3, 1), (7, 1), (37, 1))
        exact._SQUAREFREE_MEMO[778] = (1, 778)
        exact._PRIME_SUPPORT_MEMO[779] = (779,)

    def reset():
        exact.reset_factorization_memory()

    return [
        ("append", append_fake),
        ("touch middle", touch_middle),
        ("evict oldest and append", evict_oldest_and_append),
        ("delete middle", delete_middle),
        ("poison a value", poison_value),
        ("registry insert", registry_insert),
        ("registry drop", registry_drop),
        ("reset", reset),
        ("append after reset", append_fake),
    ]


@pytest.mark.parametrize("seed", range(3))
def test_the_mirror_follows_what_python_does_to_the_tables_between_calls(mirror, seed):
    gen = Gen(500 + seed)
    steps = _probe_steps(gen)
    compare_run(steps, mirror, bounded(None), cold_snapshot(), label="warm-up")
    for name, mutation in mutations():
        mutation()
        probe = _probe_steps(gen) if seed % 2 else steps
        compare_run(probe, mirror, bounded(None), take_snapshot(), label=f"after '{name}'")
        # между шагами нативный вызов оставляет память, которую питон продолжает видеть как свою
        assert list(exact._KNOWN_PRIMES) == sorted(exact._KNOWN_PRIME_SET)


def test_an_isolated_memory_block_between_native_calls(mirror):
    gen = Gen(61)
    steps = _probe_steps(gen)
    compare_run(steps, mirror, bounded(None), cold_snapshot(), label="before the block")
    warm = take_snapshot()
    with exact.isolated_factorization_memory():
        assert not exact._FACTORIZATION_MEMO
        compare_run(_probe_steps(gen), mirror, bounded(None), take_snapshot(), label="inside the block (cold)")
        assert exact._FACTORIZATION_MEMO
    after = take_snapshot()
    assert after.factorization == warm.factorization, "блок вернул память вызывающего"
    compare_run(_probe_steps(gen), mirror, bounded(None), after, label="after the block (warm again)")


def test_the_registry_and_the_factorization_table_at_their_capacity(mirror):
    """Вытеснение самого старого разложения на 8192 и очистка заполненного реестра: журнал воспроизводит оба на настоящих объектах."""

    gen = Gen(71)
    put_snapshot(cold_snapshot())
    _fill_with_small_numbers(8190)
    _fill_registry(exact._KNOWN_PRIME_REGISTRY_ENTRIES - 1)
    base = take_snapshot()
    assert len(base.factorization) == 8190 and len(base.primes) == 8191
    pool = gen.prime_pool(small=5, mid=2)
    first_new, second_new = prime_above(1 << 30), prime_above(1 << 31)  # за пределом первых 8191 простых: настоящие новые
    steps = [
        Step("prime_support", (first_new * pool[0] * 7,)),  # первое новое простое: реестр 8191 -> 8192 (полон)
        Step("squarefree_split", (second_new * pool[1] * 36 * 49,)),  # второе новое: реестр полон, очищается ДО вставки
        Step("prime_support", (math.prod(pool[2:6]),)),
        Step("squarefree_split", (math.prod(pool[:2]) * 9,)),
        Step("divided_by", (SqrtSumV1.radical(1, 6), SqrtSumV1.radical(1, math.prod(pool[1:3])) + SqrtSumV1.radical(1, 5))),
    ]
    compare_run(steps, mirror, bounded(None), base, label="capacity")
    final = take_snapshot()
    assert len(final.factorization) == exact._FACTORIZATION_MEMO_ENTRIES, "таблица разложений должна упереться в предел"
    assert len(final.primes) < 100 and first_new not in final.primes and second_new in final.primes, "реестр должен был очиститься целиком при переполнении"
    assert 2 not in final.factorization[:1] and final.factorization[0][0] != 2, "самые старые разложения вытеснены"
    # и ещё раз, когда питон сам доводит таблицы до предела между нативными вызовами
    put_snapshot(final)
    _fill_with_small_numbers(60, start=9000)
    compare_run(steps, mirror, bounded(None), take_snapshot(), label="capacity-python-fill")


def test_every_python_exception_leaves_the_mirror_usable(mirror):
    steps = [Step("radical", (1, -4)), Step("divided_by", (SqrtSumV1.radical(1, 2), SqrtSumV1(()))), Step("prime_support", (30,)), Step("radical", (1, 12))]
    compare_run(steps, mirror, bounded(None), cold_snapshot(), label="after exceptions")
    assert mirror.lengths()[0] == len(exact._KNOWN_PRIMES)
    mirror.invalidate()
    assert mirror.lengths() == (0, 0, 0, 0)
    compare_run(steps, mirror, bounded(None), take_snapshot(), label="after invalidate")


# --------------------------------------------------------------------------
# Сценарий: цепочка в ОДНОМ нативном вызове (бюджет и память текут от операции к операции)
# --------------------------------------------------------------------------


def _script_ops(steps) -> list:
    names = {"sign": "EXACT_SIGN", "difference_sign": "EXACT_DIFFERENCE_SIGN", "divided_by": "EXACT_DIVIDED_BY", "generic_divide": "EXACT_DIVIDED_BY_GENERIC", "universe_divide": "EXACT_DIVIDE_WITH_UNIVERSE", "radical": "EXACT_RADICAL", "radical_sum": "EXACT_RADICAL_SUM", "squarefree_split": "EXACT_SQUAREFREE_SPLIT", "prime_support": "EXACT_PRIME_SUPPORT"}
    ops = []
    for step in steps:
        arguments = step.args
        if step.kind == "universe_divide":
            arguments = (arguments[0], arguments[1], list(arguments[2]))
        elif step.kind == "radical_sum":
            arguments = ([list(part) for part in arguments[0]],)
        ops.append((names[step.kind], arguments))
    return ops


def _state_of(result) -> tuple:
    """Полное упорядоченное состояние, которое нативная сторона вернула после операции (режим `full_state`)."""

    primes, factorization, squarefree, support = result.state
    return (
        tuple(primes),
        tuple((key, tuple(tuple(pair) for pair in pairs)) for key, pairs in factorization),
        tuple((key, (outside, inside)) for key, outside, inside in squarefree),
        tuple((key, tuple(items)) for key, items in support),
    )


@pytest.mark.parametrize("seed", range(4))
def test_a_script_threads_budget_and_memory_through_its_operations(mirror, seed):
    gen = Gen(900 + seed)
    steps = [step for step in mixed_steps(gen, 14, mid=seed % 3, big=seed % 2) if step.kind != "prime_universe"]
    cap = 1 << 23
    put_snapshot(cold_snapshot())
    budget = make_budget(bounded(cap))
    expected = []
    previous_counts = tuple(exact.SIGN_COUNTS.values())
    for step in steps:
        outcome = attempt(lambda: oracle_call(step, budget, None))
        expected.append((outcome, budget.spent_by_article(), capture(budget, None)))
    final = take_snapshot()
    put_snapshot(cold_snapshot())
    native_budget = make_budget(bounded(cap))
    results = mirror.execute(_script_ops(steps), native_budget, full_state=True)
    assert len(results) == len(steps)
    for index, (step, (want, articles, snap), result) in enumerate(zip(steps, expected, results)):
        where = f"script шаг {index}: {step}"
        assert tuple(result.articles) == articles, where
        assert (want[0] == "ok") == result.ok, where
        if result.ok:
            value = result.value
            if step.kind == "prime_support":
                value = tuple(value)
            elif step.kind == "squarefree_split":
                value = tuple(value)
            assert exactly(want[1], value), f"{where}\n  {want[1]!r}\n  {value!r}"
        state = _state_of(result)
        assert state == (snap[0], snap[2], snap[3], snap[4]), f"{where}: полное состояние памяти после операции"
        assert tuple(result.counts) == tuple(a - b for a, b in zip(snap[5], previous_counts)), f"{where}: счётчики знака"
        previous_counts = snap[5]
    # итог зеркалится в настоящие таблицы: журнал, применённый шимом, дал то же состояние, что и эталон
    assert take_snapshot().factorization == final.factorization
    assert take_snapshot().primes == final.primes and take_snapshot().squarefree == final.squarefree and take_snapshot().support == final.support
    assert native_budget.spent_by_article() == budget.spent_by_article()
    COMPARED[0] += len(steps)
