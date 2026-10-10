"""Дешёвый сертифицированный путь предиката «интервал рефлексного угла содержит оборот опор».

`validation._interval_contains_oriented_support_delta` решает два знака: знак ориентированного креста единичных (по метрике)
векторов и знак `cos(угол) - cos(π·r)` на каждом конце сертификата. Точный путь строит для этого единичные векторы с
вложенными радикалами, `cos(π·r)` из 28-значных десятичных и гоняет `factor/cancel/ask` SymPy: десятки миллисекунд на вызов.
Интервальный фильтр знака там работает на 80 битах, а настоящий угол отстоит от конца сертификата на ~1e-28, поэтому на
реальных кривых мешах фильтр почти не решает и знак доказывает символьный путь.

Здесь тот же предикат в ОДНОРОДНОЙ форме, без единичных векторов. Нормы метрики положительны, поэтому

    sign(cross(unit_a, unit_b)) = owner_sign * sign(cross(a, b))                 (рациональное число),
    sign(cos(a, b) - c)         = sign(dot_g(a, b) - c * sqrt(N_a * N_b)),   N = g-норма в квадрате.

Всё, кроме `c = cos(π·r)`, — рациональные числа в `Fraction`; `sqrt` берётся двусторонней целочисленной оболочкой, `c` —
строгой оболочкой `mpmath.iv` (каждая операция округляет наружу). Знак объявляется, только если вся коробка
`[c_lo, c_hi] x [s_lo, s_hi]` лежит по одну сторону от `dot_g`. Порога допуска нет: есть сертификат либо уступка.

Уступка (`None`) — не отказ и не ответ: вызывающий идёт прежним точным путём без изменений. Уступают: режим символьного
бэкенда не `NATIVE_EXACT` (`SYMPY` — прежний путь целиком, `SHADOW` — сверка обоих путей), операнды вне рационального поля,
вырожденные векторы (нулевая норма — точный путь сам бросает `ValueError`; параллельные — точный путь сам даёт нуль), конец
сертификата, не читающийся как десятичная дробь, и оболочка, накрывающая нуль: точное равенство (в том числе
закрытый/открытый конец) и всё, что ближе к нему, чем разрешает точность оболочки. Поэтому ни одна граница «закрытый
против открытого» здесь не решается: решённый знак никогда не нуль.

Точность оболочки (`_ENCLOSURE_BITS`) строго ниже предела точного пути: `is_positive` SymPy читает знак через `evalf` с
потолком `DEFAULT_MAXPREC` = 333 бита. Всё, что решает этот модуль, точный путь тоже решает; на меньших разностях
уступает этот модуль, а не точный путь (`kernel/tests/test_validator_certified_sign.py`).

Каждое решение учтено под именем в `symbolic_backend.BACKEND_COUNTS` (`angle_certificate_sign.certified`,
`angle_certificate_sign.exact_fallback.<причина>`): тихого исчезновения нет.
"""

from __future__ import annotations

from fractions import Fraction
from math import isqrt

from mpmath import iv, libmp

from ..contracts.analysis import TurnOrientation
from . import symbolic_backend as _backend
from .symbolic_backend import SymbolicBackendV1

#: Точность (бит) оболочек `sqrt` и `cos`. Сертификат хоста записан с 28 десятичными знаками (~93 бита), поэтому
#: настоящий угол отстоит от конца интервала на величину порядка 1e-28; оболочка обязана быть тоньше этого с большим
#: запасом, иначе дешёвый путь уступал бы на каждом нетривиальном угле.
_ENCLOSURE_BITS = 256

EVENT_SITE = "angle_certificate_sign"


def _cosine_pi_enclosure(numerator: int, denominator: int) -> tuple[Fraction, Fraction]:
    """Строгая оболочка `cos(π·numerator/denominator)` точными дробями."""

    saved = iv.prec
    iv.prec = _ENCLOSURE_BITS
    try:
        argument = iv.pi * (iv.mpf(numerator) / iv.mpf(denominator))
        low, high = iv.cos(argument)._mpi_
    finally:
        iv.prec = saved
    return Fraction(*libmp.to_rational(low)), Fraction(*libmp.to_rational(high))


def _sqrt_enclosure(value: Fraction) -> tuple[Fraction, Fraction]:
    """`[lo, hi]` с `lo <= sqrt(value) <= hi`, относительная ширина порядка `2^-_ENCLOSURE_BITS`. `value > 0`."""

    radicand = value.numerator * value.denominator
    shift = max(0, _ENCLOSURE_BITS - radicand.bit_length() // 2)
    root = isqrt(radicand << (2 * shift))
    scale = value.denominator << shift
    return Fraction(root, scale), Fraction(root + 1, scale)


def _proven_sign(residual: Fraction, root: tuple[Fraction, Fraction], cosine: tuple[Fraction, Fraction]) -> int | None:
    """Знак `residual - c * s` на коробке `c in cosine, s in root`; `None`, если коробка накрывает нуль."""

    products = (
        cosine[0] * root[0],
        cosine[0] * root[1],
        cosine[1] * root[0],
        cosine[1] * root[1],
    )
    if residual - max(products) > 0:
        return 1
    if residual - min(products) < 0:
        return -1
    return None


def _rational(value) -> Fraction | None:
    return Fraction(int(value.p), int(value.q)) if value.is_Rational else None


def _fallback(reason: str) -> None:
    _backend.count(EVENT_SITE, f"exact_fallback.{reason}")
    return None


def certified_oriented_support_delta(metric, incoming, outgoing, orientation, interval) -> bool | None:
    """Ответ предиката, если он СЕРТИФИЦИРОВАН дешёвым путём; иначе `None` (точный путь решает сам)."""

    if _backend.backend_mode() is not SymbolicBackendV1.NATIVE_EXACT:
        return None
    ax, ay = incoming.expressions()
    bx, by = outgoing.expressions()
    gram = metric.gram
    operands = tuple(
        _rational(item) for item in (ax, ay, bx, by, gram[0][0], gram[0][1], gram[1][0], gram[1][1])
    )
    if None in operands:
        return _fallback("outside_field")
    ax, ay, bx, by, g00, g01, g10, g11 = operands
    incoming_squared = ax * (g00 * ax + g01 * ay) + ay * (g10 * ax + g11 * ay)
    outgoing_squared = bx * (g00 * bx + g01 * by) + by * (g10 * bx + g11 * by)
    cross = ax * by - ay * bx
    if incoming_squared <= 0 or outgoing_squared <= 0 or cross == 0:
        return _fallback("degenerate")
    turn_sign = metric.owner_orientation_sign * (1 if cross > 0 else -1)
    expected_sign = 1 if orientation is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION else -1
    if turn_sign != expected_sign:
        _backend.count(EVENT_SITE, "certified")
        return False
    try:
        ends = (Fraction(str(interval.lower)), Fraction(str(interval.upper)))
    except (ValueError, ZeroDivisionError):
        return _fallback("unreadable_endpoint")
    dot = ax * (g00 * bx + g01 * by) + ay * (g10 * bx + g11 * by)
    root = _sqrt_enclosure(incoming_squared * outgoing_squared)
    lower_sign = _proven_sign(dot, root, _cosine_pi_enclosure(ends[0].numerator, ends[0].denominator))
    upper_sign = _proven_sign(dot, root, _cosine_pi_enclosure(ends[1].numerator, ends[1].denominator))
    if lower_sign is None or upper_sign is None:
        return _fallback("straddles_zero")
    _backend.count(EVENT_SITE, "certified")
    # Знак не нуль, поэтому «закрытый против открытого конец» не различается: `<= 0` и `< 0` совпадают.
    return lower_sign < 0 and upper_sign > 0
