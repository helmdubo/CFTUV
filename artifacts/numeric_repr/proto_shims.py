"""Прототипы замен числового представления как RUNTIME-ШИМЫ (файлы ядра не правятся).

Цель — превратить оценку «сколько секунд сэкономит замена» в ИЗМЕРЕНИЕ и одновременно
проверить «ответ не меняется» теми же воротами (`gate.compute_row`). Шим подменяет
методы/функции в памяти процесса; это не предложение кода в дерево, а доказательство
осуществимости. Каждая замена обязана быть ТОЧНО той же арифметикой (те же решения,
те же счётчики знаков), а не приближением: целочисленное представление с общим
знаменателем вместо `Fraction`.

Шимы:
  sign      — `SqrtSumV1.certified_sign(bits)` на целых: общий знаменатель L = lcm знаменателей
              коэффициентов; границы оболочки умножены на положительное L*2^bits, поэтому
              решения `low > 0` / `high < 0` те же ровно (те же `isqrt`).
  compare   — `event_time.compare_times` слитно на целых: знак `right.divisor*ld - left.divisor*rd`
              без промежуточных `SqrtSumV1`/`Fraction`; оболочка 64 бита на целых; при неудаче
              фильтра — оригинальный путь (сопряжение и его бюджет не затронуты).
  mul       — `SqrtSumV1.__mul__` с накоплением целых числителей и ОДНИМ нормированием дроби на
              результирующий член.
  addsub    — `SqrtSumV1.__add__/__sub__` без `Fraction(0)`-заглушек и промежуточного `__neg__`.
  radical   — `SqrtSumV1.radical` без лишних преобразований `Fraction` на целом радиканте.
  floorcache— кэш `isqrt(m << 128)` по радиканту (ответ тот же; кэш процессный, см. риск).
"""

from __future__ import annotations

import sys
from fractions import Fraction
from math import gcd, isqrt, lcm

from cftuv_envelope import exact_sqrt_sum as esq
from cftuv_envelope.exact_sqrt_sum import SIGN_COUNTS, SqrtSumV1

_FLOOR_CACHE: dict[int, int] = {}
_USE_FLOOR_CACHE = [False]


def _floor_root64(radicand: int) -> int:
    if _USE_FLOOR_CACHE[0]:
        value = _FLOOR_CACHE.get(radicand)
        if value is None:
            value = _FLOOR_CACHE[radicand] = isqrt(radicand << 128)
        return value
    return isqrt(radicand << 128)


# --------------------------------------------------------------------------- sign


def _certified_sign_int(self: SqrtSumV1, bits: int):
    terms = self.terms
    common = 1
    for _, coefficient in terms:
        denominator = coefficient.denominator
        if denominator != 1:
            common = lcm(common, denominator)
    scale = 1 << bits
    low = high = 0
    for radicand, coefficient in terms:
        a = coefficient.numerator * (common // coefficient.denominator)
        if radicand == 1:
            t = a * scale
            low += t
            high += t
            continue
        floor_root = (
            _floor_root64(radicand) if bits == 64 else isqrt(radicand << (2 * bits))
        )
        if a > 0:
            low += a * floor_root
            high += a * (floor_root + 1)
        else:
            low += a * (floor_root + 1)
            high += a * floor_root
    if low > 0:
        return 1
    if high < 0:
        return -1
    return None


# ------------------------------------------------------------------------ compare


def _int_form(terms):
    """(общий знаменатель, [(радикант, числитель над ним)])."""

    common = 1
    for _, coefficient in terms:
        denominator = coefficient.denominator
        if denominator != 1:
            common = lcm(common, denominator)
    return common, [
        (radicand, coefficient.numerator * (common // coefficient.denominator))
        for radicand, coefficient in terms
    ]


def _make_compare_times(original):
    def compare_times_int(left, right, budget=None):
        ld, rd = left.dividend, right.dividend
        ln, ldd = ld.numerator, ld.denominator
        rn, rdd = rd.numerator, rd.denominator
        # difference = right.divisor * ld - left.divisor * rd; коэффициент при m:
        # R_m * ln/ldd - L_m * rn/rdd. Домножаем на положительное общее кратное.
        r_common, r_items = _int_form(right.divisor.terms)
        l_common, l_items = _int_form(left.divisor.terms)
        # R_m = r_num / r_common; L_m = l_num / l_common
        # difference_m = r_num*ln/(r_common*ldd) - l_num*rn/(l_common*rdd)
        den_a = r_common * ldd
        den_b = l_common * rdd
        big = lcm(den_a, den_b)
        mul_a = big // den_a
        mul_b = big // den_b
        merged: dict[int, int] = {}
        for radicand, numerator in r_items:
            merged[radicand] = numerator * ln * mul_a
        for radicand, numerator in l_items:
            merged[radicand] = merged.get(radicand, 0) - numerator * rn * mul_b
        items = [(m, c) for m, c in merged.items() if c]
        SIGN_COUNTS["total"] += 1
        if not items:
            SIGN_COUNTS["closed_rational_zero"] += 1
            return 0
        if len(items) == 1 and items[0][0] == 1:
            SIGN_COUNTS["closed_rational_nonzero"] += 1
            return (items[0][1] > 0) - (items[0][1] < 0)
        scale = 1 << 64
        low = high = 0
        for radicand, a in items:
            if radicand == 1:
                t = a * scale
                low += t
                high += t
                continue
            fr = _floor_root64(radicand)
            if a > 0:
                low += a * fr
                high += a * (fr + 1)
            else:
                low += a * (fr + 1)
                high += a * fr
        if low > 0:
            SIGN_COUNTS["closed_by_enclosure"] += 1
            return 1
        if high < 0:
            SIGN_COUNTS["closed_by_enclosure"] += 1
            return -1
        # Фильтр не доказал знак: ОРИГИНАЛЬНЫЙ путь (сопряжение и бюджет без изменений).
        SIGN_COUNTS["total"] -= 1  # оригинал сам ведёт счёт
        return original(left, right, budget)

    return compare_times_int


# --------------------------------------------------------------------------- mul


def _mul_int(self: SqrtSumV1, other: SqrtSumV1) -> SqrtSumV1:
    if not self.terms or not other.terms:
        return SqrtSumV1(())
    l_common, l_items = _int_form(self.terms)
    r_common, r_items = _int_form(other.terms)
    merged: dict[int, int] = {}
    for left_radicand, left_num in l_items:
        for right_radicand, right_num in r_items:
            common = gcd(left_radicand, right_radicand)
            radicand = (left_radicand // common) * (right_radicand // common)
            merged[radicand] = merged.get(radicand, 0) + left_num * right_num * common
    denominator = l_common * r_common
    return SqrtSumV1(
        tuple(
            sorted(
                (m, Fraction(n, denominator)) for m, n in merged.items() if n
            )
        )
    )


# ---------------------------------------------------------------------- add / sub


def _add_fast(self: SqrtSumV1, other: SqrtSumV1) -> SqrtSumV1:
    merged = dict(self.terms)
    get = merged.get
    for radicand, coefficient in other.terms:
        old = get(radicand)
        merged[radicand] = coefficient if old is None else old + coefficient
    return SqrtSumV1(
        tuple(sorted((m, c) for m, c in merged.items() if c._numerator))
    )


def _sub_fast(self: SqrtSumV1, other: SqrtSumV1) -> SqrtSumV1:
    merged = dict(self.terms)
    get = merged.get
    for radicand, coefficient in other.terms:
        old = get(radicand)
        merged[radicand] = -coefficient if old is None else old - coefficient
    return SqrtSumV1(
        tuple(sorted((m, c) for m, c in merged.items() if c._numerator))
    )


def _radical_fast(coefficient, radicand, budget=None):
    if type(coefficient) is not Fraction:
        coefficient = Fraction(coefficient)
    if not coefficient._numerator or radicand == 0:
        return SqrtSumV1(())
    if type(radicand) is Fraction and radicand._denominator != 1:
        coefficient /= radicand._denominator
        radicand = radicand._numerator * radicand._denominator
    else:
        radicand = int(radicand)
    outside, inside = esq.squarefree_split(radicand, budget)
    return SqrtSumV1(((inside, coefficient * outside),))


# ------------------------------------------------- НАМЕРЕННО ПЛОХИЕ шимы (контроль ворот)


def _mul_bad(self: SqrtSumV1, other: SqrtSumV1) -> SqrtSumV1:
    """Верное произведение, у которого последний коэффициент сдвинут на 1/10^12."""

    result = _mul_int(self, other)
    if not result.terms:
        return result
    *head, (radicand, coefficient) = result.terms
    return SqrtSumV1((*head, (radicand, coefficient + Fraction(1, 10**12))))


def _mul_int_types(self: SqrtSumV1, other: SqrtSumV1) -> SqrtSumV1:
    """Верное по ЗНАЧЕНИЮ произведение, но целые коэффициенты хранятся как `int`."""

    result = _mul_int(self, other)
    return SqrtSumV1(
        tuple(
            (m, c.numerator if c.denominator == 1 else c) for m, c in result.terms
        )
    )


# ------------------------------------------------------------------------ install

_ORIGINALS: dict = {}


def install(names):
    names = set(names)
    # `compare` подменяет имя во всех модулях, куда оно уже импортировано: поэтому
    # сначала загружаем весь путь очереди (в воркере пула он иначе грузится лениво).
    import cftuv_envelope.wavefront.conveyor  # noqa: F401
    import cftuv_envelope.wavefront  # noqa: F401
    if "sign" in names:
        _ORIGINALS["certified_sign"] = SqrtSumV1.certified_sign
        SqrtSumV1.certified_sign = _certified_sign_int
    if "floorcache" in names:
        _USE_FLOOR_CACHE[0] = True
    if "mul" in names:
        _ORIGINALS["__mul__"] = SqrtSumV1.__mul__
        SqrtSumV1.__mul__ = _mul_int
    if "bad_area" in names:
        # Возмущение ТОЛЬКО площади грани (не ломает ход марша): ответ обязан разойтись.
        from cftuv_envelope.wavefront import faces as faces_mod

        original_shoelace = faces_mod.doubled_shoelace

        def doubled_shoelace_bad(points):
            return original_shoelace(points) + SqrtSumV1.rational(Fraction(1, 10**12))

        faces_mod.doubled_shoelace = doubled_shoelace_bad
    if "bad_mul" in names:
        _ORIGINALS["__mul__"] = SqrtSumV1.__mul__
        SqrtSumV1.__mul__ = _mul_bad
    if "int_types" in names:
        _ORIGINALS["__mul__"] = SqrtSumV1.__mul__
        SqrtSumV1.__mul__ = _mul_int_types
    if "addsub" in names:
        _ORIGINALS["__add__"] = SqrtSumV1.__add__
        _ORIGINALS["__sub__"] = SqrtSumV1.__sub__
        SqrtSumV1.__add__ = _add_fast
        SqrtSumV1.__sub__ = _sub_fast
    if "radical" in names:
        _ORIGINALS["radical"] = SqrtSumV1.__dict__["radical"]
        SqrtSumV1.radical = staticmethod(_radical_fast)
    if "compare" in names:
        for module_name, module in list(sys.modules.items()):
            if not module_name.startswith("cftuv_envelope") or module is None:
                continue
            original = getattr(module, "compare_times", None)
            if original is not None and getattr(original, "__module__", "").endswith(
                "event_time"
            ):
                _ORIGINALS.setdefault("compare_times", original)
        original = _ORIGINALS["compare_times"]
        replacement = _make_compare_times(original)
        for module_name, module in list(sys.modules.items()):
            if not module_name.startswith("cftuv_envelope") or module is None:
                continue
            if getattr(module, "compare_times", None) is original:
                setattr(module, "compare_times", replacement)
    return sorted(names)
