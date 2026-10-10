"""Родная точная арифметика эталона: суммы корней по квадратным классам, без sympy и без факторизации.

ЗАЧЕМ. Горячий путь эталона (`metric.dot_g`, `boundary._contact_candidates`, интервальный знак
`planar_types.exact_sign`) гонял выражения `sympy` там, где величина — конечная сумма рациональных
кратных корней из рациональных (класс (а) аудита `kernel/tests/test_symbolic_backend_audit.py`).
Профиль тяжёлых доменов `building`: sympy 28–31 % собственного времени, из них ~2/3 — эти три места.

ПОЧЕМУ НЕ `SqrtSumV1`. Его каноническая форма требует бесквадратного радиканда, то есть ФАКТОРИЗАЦИИ,
а радиканды поля — нормы метрики карты в десятки знаков; факторизация идёт мимо транзакционного
бюджета, и полевые ворота требуют нуля неоплаченной работы (`reference/radical_rationality.py`, тот
же довод). Здесь канон другой и дешёвый: два корня лежат в одном КЛАССЕ, когда произведение
радикандов — полный квадрат (одна `isqrt`), а корни из разных классов линейно независимы над `Q`.
Величина хранится одним коэффициентом на класс, радикант класса — любой его представитель.
Ни одной факторизации, ни одной единицы работы бюджета, ни одного `mpmath`.

ЧТО ЗДЕСЬ ТОЧНО. Сложение, умножение, обращение (до трёх классов), знак, нуль и равенство — на
целых и `Fraction`. Нуль величины — это ПУСТОТА её членов (после слияния классов), а не малость;
знак двух членов решается сравнением квадратов в рациональных; знак трёх и более — целочисленной
оболочкой с удвоением точности, и если оболочка не разделила величину с нулём до `_SIGN_BITS`, это
именованный исход `NativeSignUndecided`, а не догадка: потребитель обязан уйти на `sympy`.

ЧТО ЗДЕСЬ НЕ РЕШАЕТСЯ — и это именованный исход, а не молчание (AGENTS.md, пункт 4). Всё, что не
читается как сумма корней из рациональных (`sin`, `cos`, `atan`, `pi`, корень из суммы, вложенные
радикалы `sqrt(10 - 2*sqrt(5))`), даёт `OutsideNativeField` с кодом `EXACT_NATIVE_OUTSIDE_FIELD`.
Потребитель уступает `sympy` и записывает уступку в `symbolic_backend.BACKEND_COUNTS`.

ТЕКСТ (EXACT_SCALAR_TEXT_CANON_V2). `ExactScalar` хранит строку, и она входит в дайджесты и в ключи событий, поэтому строка
величины обязана быть функцией её ЗНАЧЕНИЯ и только. Рациональная величина — `Integer(n)` либо `Rational(p, q)`, как всегда.
Одночленная `c * sqrt(m)` — `Sqrt(Rational(P, Q))`, где `P/Q = c^2 * m` в несократимой записи, а у отрицательной впереди стоит `-`:
значение однозначно определяет пару (знак, `c^2 * m`), и пара не зависит от представителя класса, поэтому строка не требует ни
факторизации, ни `sympy`. Прежняя форма выводила радикант через `sp.sqrt(Integer(m))`, то есть через `factorint(limit=2**15)`
без ро-метода и модульный кэш разложений sympy: радикант вида `p^2 * s` с простым `p > 2^15` давал разный текст у процесса,
который уже раскладывал `p`, и у свежего. Многочленная величина канона не имеет: `canonical_text` отказывает именованно
(`EXACT_SCALAR_TEXT_CANON_UNSUPPORTED`), а не отдаёт форму, зависящую от вида исходного выражения.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from functools import lru_cache
from math import gcd, isqrt, lcm
import re

import sympy as sp

EXACT_NATIVE_OUTSIDE_FIELD = "EXACT_NATIVE_OUTSIDE_FIELD"
EXACT_NATIVE_SIGN_UNDECIDED = "EXACT_NATIVE_SIGN_UNDECIDED"

#: Предел точности оболочки знака трёх и более классов (бит). Двучленные знаки точны и без неё.
_SIGN_BITS = (64, 128, 256, 512, 1024, 2048, 4096)
#: Больше стольких классов обращение сопряжением не берётся (оценка стоимости, не математика).
_INVERSE_CLASS_LIMIT = 3


class NativeExactError(ValueError):
    """Корень именованных отказов родной арифметики; код — в `.code`."""

    code = ""

    def __init__(self, detail: str = "") -> None:
        super().__init__(f"{self.code}: {detail}" if detail else self.code)
        self.detail = detail


class OutsideNativeField(NativeExactError):
    """Величина не читается как сумма корней из рациональных: потребитель уступает sympy."""

    code = EXACT_NATIVE_OUTSIDE_FIELD


class NativeSignUndecided(NativeExactError):
    """Оболочка не отделила величину от нуля до предела точности: уступка sympy, а не догадка."""

    code = EXACT_NATIVE_SIGN_UNDECIDED


# --------------------------------------------------------------------------
# Классы: `sqrt(new) = ratio * sqrt(keep)` тогда и только тогда, когда `keep*new` — полный квадрат
# --------------------------------------------------------------------------

_CLASS_RATIO: dict[tuple[int, int], Fraction | None] = {}
_CLASS_RATIO_LIMIT = 1 << 16


def _class_ratio(keep: int, new: int) -> Fraction | None:
    key = (keep, new)
    try:
        return _CLASS_RATIO[key]
    except KeyError:
        pass
    product = keep * new
    root = isqrt(product)
    ratio = Fraction(root, keep) if root * root == product else None
    if len(_CLASS_RATIO) >= _CLASS_RATIO_LIMIT:
        _CLASS_RATIO.clear()
    _CLASS_RATIO[key] = ratio
    return ratio


def _accumulate(into: dict[int, Fraction], radicand: int, coefficient: Fraction) -> None:
    """Прибавить `coefficient * sqrt(radicand)` к сумме по классам."""

    if radicand in into:
        into[radicand] += coefficient
        return
    if radicand != 1:
        for keep in into:
            if keep == 1:
                continue
            ratio = _class_ratio(keep, radicand)
            if ratio is not None:
                into[keep] += coefficient * ratio
                return
    into[radicand] = coefficient


def _freeze(into: dict[int, Fraction]) -> "RadicalSumV1":
    return RadicalSumV1(
        tuple(sorted((radicand, value) for radicand, value in into.items() if value))
    )


def _product_term(left: int, right: int) -> tuple[int, int]:
    """`sqrt(left)*sqrt(right) = multiplier*sqrt(radicand)`; полный квадрат уходит в множитель."""

    if left == 1:
        return right, 1
    if right == 1:
        return left, 1
    common = gcd(left, right)
    radicand = (left // common) * (right // common)
    multiplier = common
    if radicand != 1:
        root = isqrt(radicand)
        if root * root == radicand:
            multiplier *= root
            radicand = 1
    return radicand, multiplier


def _coerce(other: object) -> "RadicalSumV1":
    if type(other) is RadicalSumV1:
        return other
    if isinstance(other, int):
        return RadicalSumV1.rational(other)
    if isinstance(other, Fraction):
        return RadicalSumV1.rational(other)
    if isinstance(other, sp.Basic):
        return from_sympy(other)
    raise TypeError(f"cannot read {type(other).__name__} as a native exact value")


@dataclass(frozen=True, slots=True, eq=False)
class RadicalSumV1:
    """`sum c_k * sqrt(r_k)` по различным квадратным классам; `r_k` — любой представитель класса.

    Члены отсортированы по `r_k`, коэффициенты ненулевые, ни одно произведение двух радикандов
    не является полным квадратом (кроме радиканда 1 — рациональной части).

    `==` сравнивает ЗНАЧЕНИЯ (через разность), поэтому величина не хэшируема: ключом словаря служит
    `canonical_text` для одночленной величины; `terms` требуют учёта разных представителей класса.
    """

    terms: tuple[tuple[int, Fraction], ...]

    # ---- построение ------------------------------------------------------

    @staticmethod
    def zero() -> "RadicalSumV1":
        return _ZERO

    @staticmethod
    def rational(value: Fraction | int) -> "RadicalSumV1":
        value = Fraction(value)
        return RadicalSumV1(((1, value),)) if value else _ZERO

    @staticmethod
    def sqrt_of_rational(value: Fraction | int) -> "RadicalSumV1":
        value = Fraction(value)
        if value < 0:
            raise OutsideNativeField("root of a negative number")
        if not value:
            return _ZERO
        numerator, denominator = value.numerator, value.denominator
        radicand = numerator * denominator
        root = isqrt(radicand)
        if root * root == radicand:
            return RadicalSumV1(((1, Fraction(root, denominator)),))
        return RadicalSumV1(((radicand, Fraction(1, denominator)),))

    # ---- чтение ----------------------------------------------------------

    @property
    def is_zero(self) -> bool:
        return not self.terms

    def as_rational(self) -> Fraction | None:
        """Значение, если величина рациональна; иначе `None`."""

        terms = self.terms
        if not terms:
            return Fraction(0)
        if len(terms) == 1 and terms[0][0] == 1:
            return terms[0][1]
        return None

    # ---- арифметика ------------------------------------------------------

    def __neg__(self) -> "RadicalSumV1":
        return RadicalSumV1(tuple((radicand, -value) for radicand, value in self.terms))

    def __pos__(self) -> "RadicalSumV1":
        return self

    def __add__(self, other: object) -> "RadicalSumV1":
        other = _coerce(other)
        if not other.terms:
            return self
        if not self.terms:
            return other
        total = dict(self.terms)
        for radicand, value in other.terms:
            _accumulate(total, radicand, value)
        return _freeze(total)

    __radd__ = __add__

    def __sub__(self, other: object) -> "RadicalSumV1":
        other = _coerce(other)
        if not other.terms:
            return self
        total = dict(self.terms)
        for radicand, value in other.terms:
            _accumulate(total, radicand, -value)
        return _freeze(total)

    def __rsub__(self, other: object) -> "RadicalSumV1":
        return _coerce(other).__sub__(self)

    def scaled(self, factor: Fraction | int) -> "RadicalSumV1":
        factor = Fraction(factor)
        if not factor or not self.terms:
            return _ZERO
        return RadicalSumV1(
            tuple((radicand, value * factor) for radicand, value in self.terms)
        )

    def __mul__(self, other: object) -> "RadicalSumV1":
        if isinstance(other, (int, Fraction)):
            return self.scaled(other)
        other = _coerce(other)
        if not self.terms or not other.terms:
            return _ZERO
        total: dict[int, Fraction] = {}
        for left, left_value in self.terms:
            for right, right_value in other.terms:
                radicand, multiplier = _product_term(left, right)
                _accumulate(total, radicand, left_value * right_value * multiplier)
        return _freeze(total)

    __rmul__ = __mul__

    def inverse(self) -> "RadicalSumV1":
        terms = self.terms
        if not terms:
            raise ZeroDivisionError("division by an exact zero")
        if len(terms) == 1:
            radicand, value = terms[0]
            return RadicalSumV1(((radicand, 1 / (value * radicand)),))
        if len(terms) > _INVERSE_CLASS_LIMIT:
            raise OutsideNativeField("too many square classes to invert")
        # value = rest + b*sqrt(s); (rest - b*sqrt(s))*(rest + b*sqrt(s)) = rest^2 - b^2*s
        pivot_radicand, pivot_value = terms[-1]
        rest = RadicalSumV1(terms[:-1])
        conjugate = RadicalSumV1(terms[:-1] + ((pivot_radicand, -pivot_value),))
        norm = rest * rest - RadicalSumV1(((1, pivot_value * pivot_value * pivot_radicand),))
        return conjugate * norm.inverse()

    def __truediv__(self, other: object) -> "RadicalSumV1":
        if isinstance(other, (int, Fraction)):
            return self.scaled(1 / Fraction(other))
        return self * _coerce(other).inverse()

    def __rtruediv__(self, other: object) -> "RadicalSumV1":
        return _coerce(other) * self.inverse()

    def __pow__(self, exponent: int) -> "RadicalSumV1":
        if not isinstance(exponent, int):
            return NotImplemented
        base = self if exponent >= 0 else self.inverse()
        result = _ONE
        for _ in range(abs(exponent)):
            result = result * base
        return result

    # ---- решения ---------------------------------------------------------

    def signum(self) -> int:
        """Точный знак. Нуль — пустота членов; два члена — квадраты; больше — оболочка либо отказ."""

        terms = self.terms
        count = len(terms)
        if count == 0:
            return 0
        if count == 1:
            value = terms[0][1]
            return (value > 0) - (value < 0)
        if count == 2:
            (first, first_value), (second, second_value) = terms
            first_sign = (first_value > 0) - (first_value < 0)
            second_sign = (second_value > 0) - (second_value < 0)
            if first_sign == second_sign:
                return first_sign
            # Разные знаки: больший по модулю определяется сравнением квадратов. Равенство
            # `c1^2*r1 == c2^2*r2` означало бы один класс, а классы здесь различны.
            difference = first_value * first_value * first - second_value * second_value * second
            return first_sign if difference > 0 else second_sign
        common, items = _integer_form(terms)
        for bits in _SIGN_BITS:
            low, high = _integer_enclosure(items, bits)
            if low > 0:
                return 1
            if high < 0:
                return -1
        raise NativeSignUndecided(f"{count} classes, {_SIGN_BITS[-1]} bits")

    def enclosure(self, bits: int) -> tuple[Fraction, Fraction]:
        """Строгая оболочка `[low, high]` на целых."""

        common, items = _integer_form(self.terms)
        low, high = _integer_enclosure(items, bits)
        denominator = common << bits
        return Fraction(low, denominator), Fraction(high, denominator)

    def __eq__(self, other: object) -> bool:
        try:
            return not (self - other).terms
        except (TypeError, NativeExactError):
            return NotImplemented

    __hash__ = None  # type: ignore[assignment]

    def __repr__(self) -> str:
        return "RadicalSumV1(" + " + ".join(f"({v})*sqrt({r})" for r, v in self.terms) + ")" if self.terms else "RadicalSumV1(0)"


_ZERO = RadicalSumV1(())
_ONE = RadicalSumV1(((1, Fraction(1)),))


def _integer_form(
    terms: tuple[tuple[int, Fraction], ...],
) -> tuple[int, list[tuple[int, int]]]:
    """`(L, [(r, a)])`, `c = a / L`, `L` — наименьший общий знаменатель."""

    common = 1
    for _, value in terms:
        if value.denominator != 1:
            common = lcm(common, value.denominator)
    return common, [
        (radicand, value.numerator * (common // value.denominator))
        for radicand, value in terms
    ]


def _integer_enclosure(items: list[tuple[int, int]], bits: int) -> tuple[int, int]:
    """Границы `2^bits * sum a*sqrt(r)` на целых: `isqrt` даёт нижний узел, следующий — верхний."""

    scale = 1 << bits
    shift = 2 * bits
    low = high = 0
    for radicand, numerator in items:
        if radicand == 1:
            exact = numerator * scale
            low += exact
            high += exact
            continue
        floor_root = isqrt(radicand << shift)
        if numerator > 0:
            low += numerator * floor_root
            high += numerator * (floor_root + 1)
        else:
            low += numerator * (floor_root + 1)
            high += numerator * floor_root
    return low, high


# --------------------------------------------------------------------------
# Мост с sympy: чтение и обратная сборка
# --------------------------------------------------------------------------


@lru_cache(maxsize=1 << 17)
def from_sympy(expression: sp.Basic) -> RadicalSumV1:
    """Прочитать выражение sympy как сумму корней из рациональных либо `OutsideNativeField`."""

    if expression.is_Rational:
        return RadicalSumV1.rational(Fraction(int(expression.p), int(expression.q)))
    if expression.is_Add:
        total = _ZERO
        for term in expression.args:
            total = total + from_sympy(term)
        return total
    if expression.is_Mul:
        product = _ONE
        for factor in expression.args:
            product = product * from_sympy(factor)
        return product
    if expression.is_Pow:
        base, exponent = expression.args
        if exponent.is_Integer:
            return from_sympy(base) ** int(exponent)
        if exponent.is_Rational and exponent.q == 2 and base.is_Rational:
            root = RadicalSumV1.sqrt_of_rational(
                Fraction(int(base.p), int(base.q))
            )
            return root ** int(exponent.p)
    raise OutsideNativeField(sp.srepr(expression))


def rational_ratio(numerator: object, denominator: object) -> Fraction | None:
    """`numerator / denominator` точной дробью, если оно рационально; иначе `None`.

    Частное рационально тогда и только тогда, когда в нём сокращаются ВСЕ иррациональные члены: корни из
    разных квадратных классов линейно независимы над `Q`, а класс канонизирован (`_accumulate`), поэтому
    рациональное значение есть сумма с единственным членом радиканда 1, а нуль — пустота членов. Ни
    `radsimp`, ни `simplify`, ни факторизации, ни порога: это предикат, а не оценка, и работы бюджета он не тратит.

    `None` — частное ИРРАЦИОНАЛЬНО (доказано) либо знаменатель точно нуль (частного нет). Выражение вне поля —
    `OutsideNativeField`, а не `None`: потребитель обязан отличать «доказано, что нет» от «здесь не решается» и
    уступить sympy по имени, как остальные места `symbolic_backend`.
    """

    try:
        top, bottom = _coerce(numerator), _coerce(denominator)
        if not bottom.terms:
            return None
        if not top.terms and len(bottom.terms) == 1:
            return Fraction(0)
        if len(top.terms) == len(bottom.terms) == 1:
            # Одночлены поля не требуют собирать обратную сумму и произведение.
            (top_root, top_coefficient), = top.terms
            (bottom_root, bottom_coefficient), = bottom.terms
            ratio = Fraction(1) if top_root == bottom_root else _class_ratio(bottom_root, top_root)
            return None if ratio is None else top_coefficient / bottom_coefficient * ratio
        return (top / bottom).as_rational()
    except ZeroDivisionError:
        return None


_SYMPY_TERMS: dict[tuple, sp.Expr] = {}
_SYMPY_TERMS_LIMIT = 1 << 16


def to_sympy(value: RadicalSumV1) -> sp.Expr:
    """Точное выражение sympy (без `factor`): значение то же, вид — плоская сумма."""

    key = value.terms
    cached = _SYMPY_TERMS.get(key)
    if cached is not None:
        return cached
    parts = []
    for radicand, coefficient in key:
        rational = sp.Rational(coefficient.numerator, coefficient.denominator)
        parts.append(
            rational
            if radicand == 1
            else rational * sp.sqrt(sp.Integer(radicand))
        )
    expression = sp.Add(*parts)
    if len(_SYMPY_TERMS) >= _SYMPY_TERMS_LIMIT:
        _SYMPY_TERMS.clear()
    _SYMPY_TERMS[key] = expression
    return expression


# --------------------------------------------------------------------------
# Строка `ExactScalar` (EXACT_SCALAR_TEXT_CANON_V2)
# --------------------------------------------------------------------------

EXACT_SCALAR_TEXT_CANON_V2 = "EXACT_SCALAR_TEXT_CANON_V2"
EXACT_SCALAR_TEXT_CANON_UNSUPPORTED = "EXACT_SCALAR_TEXT_CANON_UNSUPPORTED"


class ExactScalarTextCanonUnsupported(ValueError):
    """У величины нет канонической строки V2 (больше одного члена либо вне поля): именованный отказ, а не форма из `sympy`.

    Не наследует `NativeExactError`: потребители того корня уступают `sympy`, а уступка здесь вернула бы ровно ту строку, которая
    зависит от вида исходного выражения и от истории процесса.
    """

    code = EXACT_SCALAR_TEXT_CANON_UNSUPPORTED

    def __init__(self, detail: str = "") -> None:
        super().__init__(f"{self.code}: {detail}" if detail else self.code)
        self.detail = detail


def _rational_text(value: Fraction) -> str:
    if value.denominator == 1:
        return f"Integer({value.numerator})"
    return f"Rational({value.numerator}, {value.denominator})"


def _rational_root(value: Fraction) -> Fraction | None:
    """Точный корень неотрицательной дроби либо `None`."""

    numerator = isqrt(value.numerator)
    denominator = isqrt(value.denominator)
    if numerator * numerator == value.numerator and denominator * denominator == value.denominator:
        return Fraction(numerator, denominator)
    return None


def canonical_text(value: RadicalSumV1) -> str:
    """Строка `ExactScalar` величины по канону V2: функция ЗНАЧЕНИЯ, без факторизации и без `sympy`.

    Рациональная — `Integer(n)` / `Rational(p, q)`. Одночленная `c * sqrt(m)` — `Sqrt(Rational(P, Q))` с `P/Q = c^2 * m` и знаком `-`
    впереди у отрицательной: пара (знак, `c^2 * m`) определяется значением и определяет его (`c * sqrt(m) = c' * sqrt(m')` тогда и
    только тогда, когда знаки равны и `c^2 * m = c'^2 * m'`), поэтому равные величины дают равные строки, разные — разные, какой бы
    представитель класса ни стоял в `terms`. Радикант, который сам полный квадрат (вне инварианта класса, но конструктор его не
    запрещает), даёт строку рационального числа: у величины ровно одна строка.

    Больше одного члена — `ExactScalarTextCanonUnsupported`.
    """

    terms = value.terms
    if not terms:
        return _rational_text(Fraction(0))
    if len(terms) != 1:
        raise ExactScalarTextCanonUnsupported(f"{len(terms)} square classes have no canonical text")
    radicand, coefficient = terms[0]
    if radicand == 1:
        return _rational_text(coefficient)
    square = coefficient * coefficient * radicand
    root = _rational_root(square)
    if root is not None:
        return _rational_text(-root if coefficient < 0 else root)
    sign = "-" if coefficient < 0 else ""
    return f"{sign}Sqrt(Rational({square.numerator}, {square.denominator}))"


_INTEGER_TEXT = re.compile(r"\AInteger\((-?\d+)\)\Z")
_RATIONAL_TEXT = re.compile(r"\ARational\((-?\d+), (-?\d+)\)\Z")
#: Строка V2 одночленной величины. `sympify` читает `Sqrt(...)` как неопределённую функцию, не как число: читают её только `native_of_text`
#: и `planar_types._parse_expr_uncached`.
CANON_V2_TEXT = re.compile(r"\A(?P<neg>-)?Sqrt\(Rational\((?P<p>\d+), (?P<q>\d+)\)\)\Z")
#: Прежняя (до V2) форма одночленной строки: читается, но больше не пишется.
_ONE_TERM_TEXT = re.compile(
    r"\A(?:Mul\((?P<sign>Integer\(-1\), )?"
    r"(?:(?:Integer\((?P<int>\d+)\)|Rational\((?P<num>\d+), (?P<den>\d+)\)), )?"
    r"Pow\(Integer\((?P<root>\d+)\), Rational\(1, 2\)\)\)"
    r"|Pow\(Integer\((?P<bare>\d+)\), Rational\(1, 2\)\))\Z"
)


def native_of_text(text: str) -> RadicalSumV1:
    """Значение `ExactScalar.expression`: быстрый разбор рациональных и одночленных форм, иначе sympy."""

    match = _INTEGER_TEXT.match(text)
    if match is not None:
        return RadicalSumV1.rational(int(match.group(1)))
    match = _RATIONAL_TEXT.match(text)
    if match is not None:
        return RadicalSumV1.rational(Fraction(int(match.group(1)), int(match.group(2))))
    match = CANON_V2_TEXT.match(text)
    if match is not None:
        root = RadicalSumV1.sqrt_of_rational(Fraction(int(match.group("p")), int(match.group("q"))))
        return -root if match.group("neg") else root
    match = _ONE_TERM_TEXT.match(text)
    if match is not None:
        groups = match.groupdict()
        radicand = int(groups["root"] or groups["bare"])
        if groups["int"] is not None:
            magnitude = Fraction(int(groups["int"]))
        elif groups["num"] is not None:
            magnitude = Fraction(int(groups["num"]), int(groups["den"]))
        else:
            magnitude = Fraction(1)
        if groups["sign"] is not None:
            magnitude = -magnitude
        return RadicalSumV1.sqrt_of_rational(radicand).scaled(magnitude)
    return from_sympy(sp.sympify(text))


_NATIVE_OF_TEXT = lru_cache(maxsize=1 << 17)(native_of_text)
