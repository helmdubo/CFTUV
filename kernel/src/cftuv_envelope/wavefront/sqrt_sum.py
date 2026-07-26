"""Сумма квадратных корней с рациональными коэффициентами: точно, без порогов.

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
        x = y = rng.randrange(0, n)
        divisor = 1
        while divisor == 1:
            x = (x * x + c) % n
            y = (y * y + c) % n
            y = (y * y + c) % n
            divisor = gcd(abs(x - y), n)
        if divisor != n:
            return divisor


def _factorize(n: int) -> dict[int, int]:
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
        stack.append(divisor)
        stack.append(value // divisor)
    return factors


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
    def radical(coefficient: Fraction | int, radicand: int) -> "SqrtSumV1":
        """`coefficient * sqrt(radicand)`, приведённое к канонической форме."""

        coefficient = Fraction(coefficient)
        if coefficient == 0 or radicand == 0:
            return SqrtSumV1(())
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
