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

ТЕКСТ. `ExactScalar` хранит `srepr`-строку, и она входит в дайджесты. Для одночленной величины
`c * sqrt(m)` строка воспроизводится БЕЗ `sympy.factor`: форма sympy для неё единственна, а
радикант приводится самим `sp.sqrt(Integer)` (один раз на радикант, с памятью). Многочленные
величины идут через `sympy` прежним путём (`factor(cancel(...))`): их строка зависит от вида
исходного выражения, а не только от значения, и родная форма её бы не повторила.
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
    `native_text` либо `terms` вместе с пониманием, что представитель класса может быть другим.
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
# Строка `srepr` (identity ExactScalar)
# --------------------------------------------------------------------------


@lru_cache(maxsize=1 << 15)
def _sympy_radical(radicand: int) -> tuple[int, int]:
    """`sqrt(radicand) = outside*sqrt(inside)` в канонической для sympy форме (один раз на радикант)."""

    root = sp.sqrt(sp.Integer(radicand))
    if root.is_Integer:
        return int(root), 1
    if root.is_Pow:
        return 1, int(root.args[0])
    if root.is_Mul:
        coefficient, power = root.args
        if coefficient.is_Integer and power.is_Pow:
            return int(coefficient), int(power.args[0])
    raise OutsideNativeField(sp.srepr(root))


def _rational_text(value: Fraction) -> str:
    if value.denominator == 1:
        return f"Integer({value.numerator})"
    return f"Rational({value.numerator}, {value.denominator})"


def one_term_text(value: RadicalSumV1) -> str | None:
    """`srepr(factor(cancel(c*sqrt(m))))` без sympy; `None` — не одночленная нерациональная величина.

    Формы проверены против sympy (`kernel/tests/test_symbolic_backend.py`): положительный
    коэффициент идёт первым множителем, отрицательный — `Integer(-1)`, затем модуль, если он не 1.
    """

    terms = value.terms
    if len(terms) != 1 or terms[0][0] == 1:
        return None
    radicand, coefficient = terms[0]
    outside, inside = _sympy_radical(radicand)
    if inside == 1:
        return None
    coefficient = coefficient * outside
    root = f"Pow(Integer({inside}), Rational(1, 2))"
    magnitude = abs(coefficient)
    if coefficient < 0:
        if magnitude == 1:
            return f"Mul(Integer(-1), {root})"
        return f"Mul(Integer(-1), {_rational_text(magnitude)}, {root})"
    if magnitude == 1:
        return root
    return f"Mul({_rational_text(magnitude)}, {root})"


_INTEGER_TEXT = re.compile(r"\AInteger\((-?\d+)\)\Z")
_RATIONAL_TEXT = re.compile(r"\ARational\((-?\d+), (-?\d+)\)\Z")
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


def native_text(value: RadicalSumV1) -> tuple[str, bool]:
    """`(srepr, emulated)`: строка `ExactScalar`; `emulated` ложно, когда пришлось звать sympy.

    Рациональная и одночленная величины собираются без sympy. Остальное — прежний путь
    `srepr(factor(cancel(выражение)))`: он единственный, который знает, как sympy записывает
    многочленную сумму, и повторять его вручную значило бы второй экземпляр правила.
    """

    rational = value.as_rational()
    if rational is not None:
        return _rational_text(rational), True
    emulated = one_term_text(value)
    if emulated is not None:
        return emulated, True
    return sp.srepr(sp.factor(sp.cancel(to_sympy(value)))), False
