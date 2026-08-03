"""Нейтральная сумма квадратных корней с рациональными коэффициентами.

Зачем это здесь. Скорость вершины волнового фронта равна `1/sin(alpha/2)` вдоль
биссектрисы, единичная нормаль ребра несёт `sqrt`, и поэтому время события в
общем случае иррационально. На целочисленной решётке иррациональность имеет
ровно одну форму: `sum c_m * sqrt(m)` по РАЗЛИЧНЫМ бесквадратным `m >= 1` с
рациональными `c_m`. Ничего другого в этой задаче не появляется.

Форма каноническая, и это главное её свойство. Квадратные корни различных
бесквадратных целых линейно независимы над Q (классическая теорема), поэтому
представление ЕДИНСТВЕННО, а значит:

- равенство двух величин — это совпадение конечного набора дробей, то есть
  ЧИСТО РАЦИОНАЛЬНАЯ проверка без единого извлечения корня;
- ноль отличается от «очень маленького» доказательством, а не допуском.

Знак — единственное место, где рациональной проверки не хватает, и он тоже
решается точно: сначала целочисленная интервальная оболочка (фильтр, не
решатель), при неудаче — рекурсивное сопряжение по простому из носителя.
Обе ветви на целых числах, порога нет ни в одной.

Чего здесь нет: float, mpmath, SymPy, epsilon. Оболочка построена на `isqrt`,
то есть на целых, и её границы точны по построению.
"""

from __future__ import annotations

from bisect import insort
from dataclasses import dataclass
from fractions import Fraction
from functools import lru_cache
from math import gcd, isqrt
import random


# Наблюдение за тем, ЧЕМ решается знак. Заведено вместе с самой величиной,
# потому что вопрос среза — «можно ли сравнивать точно и дёшево» — без этих
# четырёх чисел не имеет измеримого ответа.
#
# `closed_rational_*` — ответ дала одна дробь, корней не было вообще;
# `closed_by_enclosure` — доказала целочисленная оболочка (фильтр);
# `closed_by_conjugation` — понадобилось точное сопряжение по простому.
SIGN_COUNTS = {
    "total": 0,
    "closed_rational_zero": 0,
    "closed_rational_nonzero": 0,
    "closed_by_enclosure": 0,
    "closed_by_conjugation": 0,
}


def reset_sign_counts() -> None:
    for key in SIGN_COUNTS:
        SIGN_COUNTS[key] = 0


class NegativeRadicandError(ValueError):
    """Под корнем отрицательное. В этой задаче не бывает и молча не проходит."""


class ZeroSqrtSumDivisorError(ZeroDivisionError):
    """Деление на доказанный ноль. Именованный отказ, а не NaN."""


# --------------------------------------------------------------------------
# Целочисленная факторизация: нужна ровно для бесквадратной части радикала
# --------------------------------------------------------------------------

_MILLER_RABIN_BASES = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37)


def _is_prime(n: int) -> bool:
    """Детерминированный Миллер—Рабин для n < 3.3e24, то есть для всех наших."""

    if n < 2:
        return False
    for small in _MILLER_RABIN_BASES:
        if n % small == 0:
            return n == small
    d, r = n - 1, 0
    while d % 2 == 0:
        d //= 2
        r += 1
    for base in _MILLER_RABIN_BASES:
        x = pow(base, d, n)
        if x == 1 or x == n - 1:
            continue
        for _ in range(r - 1):
            x = x * x % n
            if x == n - 1:
                break
        else:
            return False
    return True


def _pollard_rho(n: int) -> int:
    """Нетривиальный делитель составного n. Брент, детерминированный посев."""

    if n % 2 == 0:
        return 2
    rng = random.Random(n)
    while True:
        c = rng.randrange(1, n)
        y = rng.randrange(0, n)
        divisor = _pollard_rho_brent_attempt(n, y, c, batch_size=64)
        if divisor is not None:
            assert 1 < divisor < n and n % divisor == 0
            return divisor


def _pollard_rho_brent_attempt(
    n: int,
    y: int,
    c: int,
    *,
    batch_size: int,
) -> int | None:
    """Одна попытка Брента; `None` требует следующую пару параметров.

    GCD берётся с произведения не более `batch_size` разностей. Если пакет
    дал `g == n` (включая `q == 0`), сохранённая граница пакета проигрывается
    по одному шагу. Так общий делитель не теряется внутри `gcd(0, n) == n`,
    а неудачная орбита не меняет состояние локального `Random(n)`.
    """

    if batch_size < 1:
        raise ValueError("размер пакета Брента должен быть положительным")
    power = 1
    while True:
        x = y
        for _ in range(power):
            y = (y * y + c) % n
        offset = 0
        while offset < power:
            checkpoint = y
            steps = min(batch_size, power - offset)
            product = 1
            for _ in range(steps):
                y = (y * y + c) % n
                product = product * abs(x - y) % n
            divisor = gcd(product, n)
            if divisor == 1:
                offset += steps
                continue
            if 1 < divisor < n:
                assert n % divisor == 0
                return divisor

            replay = checkpoint
            for _ in range(steps):
                replay = (replay * replay + c) % n
                divisor = gcd(abs(x - replay), n)
                if divisor == 1:
                    continue
                if 1 < divisor < n:
                    assert n % divisor == 0
                    return divisor
                # Орбита замкнулась раньше доказанного делителя: новая попытка.
                return None
            return None
        power *= 2


def _rho_factors(n: int) -> dict[int, int]:
    """Разложение с нуля: Миллер—Рабин, точный квадрат, ро-Поллард."""

    factors: dict[int, int] = {}
    stack = [n]
    while stack:
        value = stack.pop()
        if value == 1:
            continue
        if _is_prime(value):
            factors[value] = factors.get(value, 0) + 1
            continue
        root = isqrt(value)
        if root * root == value:
            stack.append(root)
            stack.append(root)
            continue
        divisor = _pollard_rho(value)
        assert 1 < divisor < value and value % divisor == 0
        stack.append(divisor)
        stack.append(value // divisor)
    return factors


# --------------------------------------------------------------------------
# Память канонизации: реестр доказанных простых и взаимно простой базис
#
# Разложение на простые ЕДИНСТВЕННО, поэтому ни память, ни базис не могут
# изменить ответ: они меняют только ПУТЬ, которым он получен. Всё, что ниже,
# — цена, а не семантика; сброс памяти обязан давать те же кортежи.
#
# Ёмкости названы, потому что владелец потребовал реестра допусков. Категория
# у всех трёх одна — NUMERIC/WORK budget: превышение не отказ и не другой
# ответ, а возврат к прежнему (дорогому) пути.
# --------------------------------------------------------------------------

# Память разложений (Р1). На walls.012 density 0 измерено 35 вызовов
# `_factorize` при 15 различных радикандах; на худшем измеренном домене
# `building` (патч 109) — 60 вызовов. 8192 записи — два порядка запаса над
# худшим измеренным доменом; запись держит целое-ключ и кортеж пар.
_FACTORIZATION_MEMO_ENTRIES = 1 << 13

# Реестр доказанных простых (Р2). На walls.012 их 28 на домен, на пятёрке
# самых тяжёлых доменов `building` — меньше трёхсот. 8192 — тот же порядок
# запаса. Переполнение очищает реестр целиком: половинчатый реестр остался бы
# корректным, но непредсказуемым по цене, а полный сброс воспроизводим.
_KNOWN_PRIME_REGISTRY_ENTRIES = 1 << 13

# Бюджет расщеплений при построении взаимно простого базиса. Каждое
# расщепление строго уменьшает сумму элементов, поэтому цикл конечен и без
# бюджета; бюджет — верхняя граница, названная явно. Измерено: базис пяти
# радикандов walls.012 строится за 0.3 мс и 10 расщеплений.
_COPRIME_BASIS_SPLIT_BUDGET = 1 << 12

_KNOWN_PRIMES: list[int] = []
_KNOWN_PRIME_SET: set[int] = set()


def reset_factorization_memory() -> None:
    """Сбросить память канонизации. Ответы не меняются — меняется только цена."""

    _KNOWN_PRIMES.clear()
    _KNOWN_PRIME_SET.clear()
    _factorization_pairs.cache_clear()
    squarefree_split.cache_clear()
    prime_support.cache_clear()


def _register_prime(prime: int) -> None:
    """Запомнить доказанное простое. Реестр возрастающий и без повторов."""

    if prime in _KNOWN_PRIME_SET:
        return
    if len(_KNOWN_PRIMES) >= _KNOWN_PRIME_REGISTRY_ENTRIES:
        _KNOWN_PRIMES.clear()
        _KNOWN_PRIME_SET.clear()
    insort(_KNOWN_PRIMES, prime)
    _KNOWN_PRIME_SET.add(prime)


def _strip_known_primes(n: int) -> tuple[dict[int, int], int]:
    """Снять уже доказанные простые ДЕЛЕНИЕМ, вернуть кофактор.

    Реестр возрастающий, поэтому обход прекращается на первом простом,
    большем остатка. Ни одного нового доказательства здесь не возникает.
    """

    factors: dict[int, int] = {}
    remainder = n
    for prime in _KNOWN_PRIMES:
        if prime > remainder:
            break
        if remainder % prime:
            continue
        power = 0
        while remainder % prime == 0:
            remainder //= prime
            power += 1
        factors[prime] = power
    return factors, remainder


@lru_cache(maxsize=_FACTORIZATION_MEMO_ENTRIES)
def _factorization_pairs(n: int) -> tuple[tuple[int, int], ...]:
    """Разложение как неизменяемый кортеж пар, по возрастанию простого.

    Кешируемый слой отделён от `_factorize` намеренно: наружу обязан уходить
    свежий `dict`, иначе вызывающий получил бы общий изменяемый объект.
    """

    if n < 2:
        return ()
    factors, remainder = _strip_known_primes(n)
    if remainder > 1:
        for prime, power in _rho_factors(remainder).items():
            factors[prime] = factors.get(prime, 0) + power
    for prime in factors:
        _register_prime(prime)
    return tuple(sorted(factors.items()))


def _factorize(n: int) -> dict[int, int]:
    """Разложение `n` на простые: `{простое: степень}`.

    Р1 — память. Функция чистая (разложение единственно), поэтому мемоизация
    не может изменить ответ. Измерено на walls.012 density 0: 35 вызовов при
    15 различных радикандах, 6.01 s против 1.93 s.

    Р2 — реестр простых. Уже доказанные простые снимаются делением, и
    ро-Поллард запускается только на кофакторе. Радикалы одного домена родом
    из одних и тех же законов прихода и делят почти все свои простые: на
    walls.012 ВСЕ 28 простых происходят из первого же набора, поэтому десять
    последующих радикандов (до 201 бита) разлагаются делением досуха.
    """

    return dict(_factorization_pairs(n))


def _coprime_basis(values: tuple[int, ...]) -> list[int]:
    """Взаимно простой базис набора. Только gcd, ни одной факторизации.

    Каждое расщепление заменяет пару `(a, b)` на `(g, a//g, b//g)`, что
    строго уменьшает сумму элементов, поэтому цикл конечен. По исчерпании
    `_COPRIME_BASIS_SPLIT_BUDGET` расщепления прекращаются, и элемент кладётся
    в базис как есть: базис перестаёт быть взаимно простым, но его назначение
    — только УЗНАТЬ простые, и ответ от этого не зависит.
    """

    basis: list[int] = []
    stack = [value for value in values if value > 1]
    splits = 0
    while stack:
        value = stack.pop()
        placed = False
        for index, atom in enumerate(basis):
            common = gcd(value, atom)
            if common == 1:
                continue
            if common == value and common == atom:
                placed = True
                break
            if splits >= _COPRIME_BASIS_SPLIT_BUDGET:
                break
            splits += 1
            basis.pop(index)
            stack.append(common)
            if atom // common > 1:
                stack.append(atom // common)
            if value // common > 1:
                stack.append(value // common)
            placed = True
            break
        if not placed:
            basis.append(value)
    return basis


def _seed_factorization_basis(radicands: tuple[int, ...]) -> None:
    """Р2: разложить НАБОР радикандов через его взаимно простой базис.

    Поэлементная факторизация платит за наибольший простой делитель КАЖДОГО
    радиканда; базис платит за него один раз. Измерено на walls.012 density 0:
    набор из пяти радикандов (203…246 бит) даёт базис из десяти элементов
    максимум в 117 бит за 0.3 мс gcd-ов, и его факторизация стоит 0.121 s
    против 1.83 s поэлементной.

    Тождество, на котором это стоит: базис порождает те же простые, что и
    сам набор, а разложение на простые единственно. Поэтому `squarefree_split`
    и `prime_support` возвращают побитово те же значения.
    """

    residues = []
    for radicand in radicands:
        _, remainder = _strip_known_primes(radicand)
        if remainder > 1:
            residues.append(remainder)
    if len(residues) < 2:
        return
    for atom in _coprime_basis(tuple(residues)):
        _factorization_pairs(atom)


@lru_cache(maxsize=None)
def squarefree_split(n: int) -> tuple[int, int]:
    """`n = g*g*m` с бесквадратным `m`. Кешируется: радикалы повторяются."""

    if n < 0:
        raise NegativeRadicandError(f"под корнем {n}")
    if n in (0, 1):
        return (0, 0) if n == 0 else (1, 1)
    outside, inside = 1, 1
    for prime, power in _factorize(n).items():
        outside *= prime ** (power // 2)
        if power % 2:
            inside *= prime
    return outside, inside


@lru_cache(maxsize=None)
def prime_support(radicand: int) -> tuple[int, ...]:
    """Простые бесквадратного радикала, по возрастанию."""

    if radicand <= 1:
        return ()
    return tuple(sorted(_factorize(radicand)))


def _prime_universe_from_q_values(
    q_values: tuple[int | Fraction, ...],
) -> tuple[int, ...]:
    """Простые нечётных степеней примитивных `q`, один раз на операцию.

    Для `q = p/r` подкоренное целое канонической формы равно `p*r`, потому
    что `sqrt(p/r) = sqrt(p*r)/r`. Факторизация здесь конечна и проверяется
    обратным произведением; дальше деление использует только делимость.

    Это ЕДИНСТВЕННОЕ место, где набор радикандов виден целиком, поэтому
    взаимно простой базис (Р2) строится именно здесь. Сам обход остаётся
    прежним: `_factorize` вызывается по одному разу на радиканд и в том же
    возрастающем порядке — базис только удешевляет эти вызовы.
    """

    transformed: set[int] = set()
    for raw_q in q_values:
        q = Fraction(raw_q)
        if q < 0:
            raise NegativeRadicandError(f"под корнем {q}")
        if q == 0:
            continue
        transformed.add(q.numerator * q.denominator)
    _seed_factorization_basis(tuple(sorted(transformed)))
    primes: set[int] = set()
    for radicand in sorted(transformed):
        factors = _factorize(radicand)
        reconstructed = 1
        for prime, power in factors.items():
            reconstructed *= prime**power
            if power % 2:
                primes.add(prime)
        if reconstructed != radicand:
            raise ArithmeticError(
                f"факторизация {radicand} не восстановила исходное число"
            )
    return tuple(sorted(primes))


def _support_from_prime_universe(
    radicand: int,
    prime_universe: tuple[int, ...],
) -> tuple[int, ...] | None:
    """Точный носитель бесквадратного радикала либо `None`.

    Остаток обязан стать единицей, произведение — исходным радикандом, а
    повторная делимость тем же простым запрещена. Любой промах означает,
    что локальное доказательство неполно и вся операция должна уйти в legacy.
    """

    if radicand <= 1:
        return () if radicand == 1 else None
    remainder = radicand
    product = 1
    support: list[int] = []
    for prime in prime_universe:
        if remainder % prime != 0:
            continue
        remainder //= prime
        product *= prime
        support.append(prime)
        if remainder % prime == 0:
            return None
        if remainder == 1:
            break
    if remainder != 1 or product != radicand:
        return None
    return tuple(support)


def _pick_prime_from_universe(
    terms: dict[int, Fraction],
    prime_universe: tuple[int, ...],
) -> int | None:
    """Минимальный простой носителя, если ВСЕ радикалы восстановлены точно."""

    smallest: int | None = None
    for radicand, coefficient in terms.items():
        if not coefficient or radicand == 1:
            continue
        support = _support_from_prime_universe(radicand, prime_universe)
        if not support:
            return None
        if smallest is None or support[0] < smallest:
            smallest = support[0]
    return smallest


# --------------------------------------------------------------------------
# Сама величина
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class SqrtSumV1:
    """`sum c_m * sqrt(m)`, m бесквадратные и различные, c_m != 0.

    Хранится отсортированным кортежем, поэтому величина хешируема и попадает в
    дайджест побитово воспроизводимо. `Fraction`, а не float — по той же
    причине, по которой невязка привязки хранится дробью.
    """

    terms: tuple[tuple[int, Fraction], ...]

    # ---- построение -----------------------------------------------------

    @staticmethod
    def zero() -> "SqrtSumV1":
        return SqrtSumV1(())

    @staticmethod
    def rational(value: Fraction | int) -> "SqrtSumV1":
        value = Fraction(value)
        return SqrtSumV1(((1, value),)) if value else SqrtSumV1(())

    @staticmethod
    def radical(
        coefficient: Fraction | int, radicand: Fraction | int
    ) -> "SqrtSumV1":
        """`coefficient * sqrt(radicand)`, приведённое к канонической форме.

        Радиканд бывает РАЦИОНАЛЬНЫМ, и это не послабление типа, а требование
        входа. Квадрат скорости фронта `q` равен `(s/|n|)^2 * |d|^2`, и у
        полевого патча `(s/|n|)^2` — дробь со знаменателем 844687660141 либо
        1439659412197 (оба БЕСКВАДРАТНЫ), поэтому целым `q` может стать лишь
        при ребре длиной в сам знаменатель узлов решётки. Доступно 942 195 при
        самом мелком допустимом шаге окна — меньше в 896 510 раз. Целочисленный
        радиканд означал бы «полевой вход не отображается», а не «типы строже».

        Приведение точное и однострочное: `sqrt(p/r) = (1/r)*sqrt(p*r)`, потому
        что `p/r = p*r/r^2` и `r > 0`. Знаменатель уезжает в коэффициент, где
        `Fraction` его держит и так, а под корнем остаётся целое, и всё, что
        ниже — бесквадратное разложение, сопряжение, оболочка — работает без
        единой правки.

        На ЦЕЛОМ радиканде ветка не выполняется вовсе, поэтому прежний путь
        остаётся побитово тем же: `Fraction(4).denominator == 1`, и `int(4)`
        возвращает то же самое `4`, которое сюда приходило раньше. Ровно этим
        сохраняется неподвижность `FROZEN_DIGESTS`.
        """

        coefficient = Fraction(coefficient)
        if coefficient == 0 or radicand == 0:
            return SqrtSumV1(())
        if isinstance(radicand, Fraction) and radicand.denominator != 1:
            coefficient /= radicand.denominator
            radicand = radicand.numerator * radicand.denominator
        else:
            radicand = int(radicand)
        outside, inside = squarefree_split(radicand)
        return SqrtSumV1(((inside, coefficient * outside),))

    @staticmethod
    def _from_map(mapping: dict[int, Fraction]) -> "SqrtSumV1":
        return SqrtSumV1(
            tuple(sorted((m, c) for m, c in mapping.items() if c))
        )

    # ---- арифметика -----------------------------------------------------

    def as_map(self) -> dict[int, Fraction]:
        return dict(self.terms)

    def __add__(self, other: "SqrtSumV1") -> "SqrtSumV1":
        merged = self.as_map()
        for radicand, coefficient in other.terms:
            merged[radicand] = merged.get(radicand, Fraction(0)) + coefficient
        return SqrtSumV1._from_map(merged)

    def __neg__(self) -> "SqrtSumV1":
        return SqrtSumV1(tuple((m, -c) for m, c in self.terms))

    def __sub__(self, other: "SqrtSumV1") -> "SqrtSumV1":
        return self + (-other)

    def scaled(self, factor: Fraction | int) -> "SqrtSumV1":
        factor = Fraction(factor)
        if factor == 0:
            return SqrtSumV1(())
        return SqrtSumV1(tuple((m, c * factor) for m, c in self.terms))

    def __mul__(self, other: "SqrtSumV1") -> "SqrtSumV1":
        """Произведение. `sqrt(a)*sqrt(b) = g*sqrt(a*b/g^2)`, g = gcd(a, b).

        Обе величины бесквадратны, поэтому `a*b/g^2` бесквадратно тоже —
        замыкание не требует ни одной новой факторизации.
        """

        merged: dict[int, Fraction] = {}
        for left_radicand, left_coefficient in self.terms:
            for right_radicand, right_coefficient in other.terms:
                common = gcd(left_radicand, right_radicand)
                radicand = (left_radicand // common) * (right_radicand // common)
                value = left_coefficient * right_coefficient * common
                merged[radicand] = merged.get(radicand, Fraction(0)) + value
        return SqrtSumV1._from_map(merged)

    def __truediv__(self, other: "SqrtSumV1") -> "SqrtSumV1":
        """Деление домножением на сопряжённые. Результат — канонический.

        Каждое сопряжение по простому p убирает p из носителя знаменателя,
        поэтому шагов ровно столько, сколько различных простых, и в конце
        знаменатель рационален. Приближения здесь нет ни на одном шаге.
        """

        if other.is_zero:
            raise ZeroSqrtSumDivisorError("деление на точный ноль")
        numerator, denominator = self, other
        while True:
            rational = denominator.as_rational()
            if rational is not None:
                return numerator.scaled(Fraction(1) / rational)
            prime = _pick_prime(denominator.as_map())
            outside, inside = _split_by_prime(denominator.as_map(), prime)
            root = SqrtSumV1.radical(1, prime)
            conjugate = SqrtSumV1._from_map(outside) - (
                SqrtSumV1._from_map(inside) * root
            )
            numerator = numerator * conjugate
            denominator = denominator * conjugate

    # ---- решения --------------------------------------------------------

    @property
    def is_zero(self) -> bool:
        """Точный ноль. Рациональная проверка: коэффициентов просто нет."""

        return not self.terms

    def is_rational(self) -> bool:
        return all(radicand == 1 for radicand, _ in self.terms)

    def as_rational(self) -> Fraction | None:
        if not self.is_rational():
            return None
        return dict(self.terms).get(1, Fraction(0))

    def enclosure(self, bits: int) -> tuple[Fraction, Fraction]:
        """Строгая оболочка на целых: `[lo, hi]`, lo <= value <= hi.

        `sqrt(m)` заключается между `isqrt(m<<2b)/2^b` и следующим узлом. Это
        фильтр, а не решатель: он либо доказывает знак, либо уступает точному
        пути ниже. Порога в нём нет — есть граница, вычисленная точно.
        """

        scale = 1 << bits
        low = high = Fraction(0)
        for radicand, coefficient in self.terms:
            if radicand == 1:
                low += coefficient
                high += coefficient
                continue
            floor_root = isqrt(radicand << (2 * bits))
            lower = Fraction(floor_root, scale)
            upper = Fraction(floor_root + 1, scale)
            if coefficient > 0:
                low += coefficient * lower
                high += coefficient * upper
            else:
                low += coefficient * upper
                high += coefficient * lower
        return low, high

    def certified_sign(self, bits: int) -> int | None:
        """Знак, доказанный оболочкой, либо None. Тождества не доказывает."""

        low, high = self.enclosure(bits)
        if low > 0:
            return 1
        if high < 0:
            return -1
        return None

    def sign(self, *, filter_bits: int = 64) -> int:
        """Точный знак. Оболочка — фильтр, сопряжение — решатель.

        Отката нет и быть не может: ветвь сопряжения конечна и целочисленна,
        поэтому «не смог» здесь не существует как исход.
        """

        SIGN_COUNTS["total"] += 1
        if not self.terms:
            SIGN_COUNTS["closed_rational_zero"] += 1
            return 0
        if len(self.terms) == 1 and self.terms[0][0] == 1:
            SIGN_COUNTS["closed_rational_nonzero"] += 1
            coefficient = self.terms[0][1]
            return (coefficient > 0) - (coefficient < 0)
        certified = self.certified_sign(filter_bits)
        if certified is not None:
            SIGN_COUNTS["closed_by_enclosure"] += 1
            return certified
        SIGN_COUNTS["closed_by_conjugation"] += 1
        return _exact_sign(self.as_map(), filter_bits)


def _divide_with_prime_universe(
    numerator: SqrtSumV1,
    denominator: SqrtSumV1,
    prime_universe: tuple[int, ...],
) -> SqrtSumV1:
    """Точное деление с локально доказанным носителем примитивных `q`.

    Сопряжения и их порядок совпадают с `__truediv__`. Отличается только
    источник минимального простого: делимость по конечному universe вместо
    факторизации каждого производного радикала. Если хотя бы один радикал
    целиком не восстановлен, прежний путь запускается заново на ИСХОДНЫХ
    операндах — частично сопряжённые величины никогда не смешиваются с legacy.
    """

    original_numerator = numerator
    original_denominator = denominator
    if denominator.is_zero:
        return numerator / denominator
    while True:
        rational = denominator.as_rational()
        if rational is not None:
            return numerator.scaled(Fraction(1) / rational)
        prime = _pick_prime_from_universe(
            denominator.as_map(),
            prime_universe,
        )
        if prime is None:
            return original_numerator / original_denominator
        outside, inside = _split_by_prime(denominator.as_map(), prime)
        root = SqrtSumV1.radical(1, prime)
        conjugate = SqrtSumV1._from_map(outside) - (
            SqrtSumV1._from_map(inside) * root
        )
        numerator = numerator * conjugate
        denominator = denominator * conjugate


def _split_by_prime(
    terms: dict[int, Fraction], prime: int
) -> tuple[dict[int, Fraction], dict[int, Fraction]]:
    """`E = A + B*sqrt(p)`. Носитель A и B уже без p."""

    outside: dict[int, Fraction] = {}
    inside: dict[int, Fraction] = {}
    for radicand, coefficient in terms.items():
        if radicand % prime == 0:
            key = radicand // prime
            inside[key] = inside.get(key, Fraction(0)) + coefficient
        else:
            outside[radicand] = outside.get(radicand, Fraction(0)) + coefficient
    return outside, inside


def _pick_prime(terms: dict[int, Fraction]) -> int | None:
    """Наименьшее простое носителя. Детерминированно, значит воспроизводимо."""

    primes: set[int] = set()
    for radicand, coefficient in terms.items():
        if coefficient and radicand > 1:
            primes.update(prime_support(radicand))
    return min(primes) if primes else None


def _exact_sign(terms: dict[int, Fraction], filter_bits: int) -> int:
    """Знак рекурсией по простым носителя. Целые числа, конечное число шагов.

    Разбиение `E = A + B*sqrt(p)` убирает p из носителя обеих частей, поэтому
    глубина рекурсии равна числу различных простых и рекурсия конечна.
    При разных знаках A и B решает `A^2 - p*B^2`: он равен нулю только если
    `sqrt(p)` лежит в меньшем поле, чего не бывает, — поэтому ноль здесь
    означает `B = 0`, и эта ветка отработана выше.
    """

    terms = {m: c for m, c in terms.items() if c}
    if not terms:
        return 0
    prime = _pick_prime(terms)
    if prime is None:
        value = terms[1]
        return (value > 0) - (value < 0)
    certified = SqrtSumV1._from_map(terms).certified_sign(filter_bits)
    if certified is not None:
        return certified

    outside, inside = _split_by_prime(terms, prime)
    outside_sign = _exact_sign(outside, filter_bits)
    inside_sign = _exact_sign(inside, filter_bits)
    if inside_sign == 0:
        return outside_sign
    if outside_sign == 0:
        return inside_sign
    if outside_sign == inside_sign:
        return outside_sign

    left = SqrtSumV1._from_map(outside)
    right = SqrtSumV1._from_map(inside)
    discriminant = left * left - (right * right).scaled(prime)
    discriminant_sign = _exact_sign(discriminant.as_map(), filter_bits)
    if discriminant_sign == 0:
        # Доказуемо недостижимо, но исход назван, а не пропущен молча.
        return 0
    return outside_sign if discriminant_sign > 0 else inside_sign
