"""Знак остатка подшага Density: интервальный фильтр до построения выражения, точный путь — как был.

Остаток `a * dot^2 - b * |l|^2 * |r|^2` (`a`, `b` — по `q`) строился выражением SymPy только ради знака: произведение
сумм заставляло `Pow(сумма, 2)` пройти `Add._eval_power` с выводом допущений по свежим тригонометрическим членам,
а это было самое дорогое место компиляции плана (около 17% на `building`).

Здесь знак берётся из ТЕХ ЖЕ оболочек `dot`, `|l|^2`, `|r|^2`, что уже лежат в памяти транзакции (`dot` спрашивали
знак секундой раньше), и доказан только тогда, когда оболочка остатка отстоит от нуля дальше `2^-100` от суммы
модулей членов. Оболочку остатка, которую прежний путь собрал бы деревом SymPy из тех же трёх оболочек, отличают от этой
несколько единиц последнего разряда 160 бит (`~2^-155` от суммы модулей), поэтому при таком запасе оба пути видят один
и тот же знак. В остальных случаях (остаток рационален, предел, оболочка пересекает нуль, неподдержанный узел)
фильтр молчит, и остаток строится и спрашивается ровно как прежде: названные исходы и тексты отказов не меняются.
"""

from __future__ import annotations

import sympy as sp
from mpmath import iv

from .._density_policy import DensityIntervalEnclosureUnsupported, density_interval_enclosure
from .adaptive_density_band import squared

#: Предельная точность оболочек Density (как в `angular._density_exact_sign`).
_PRECISION = 160
#: Запас фильтра: знак принят, если оболочка остатка дальше от нуля, чем `2^-_MARGIN_BITS` от суммы модулей членов.
_MARGIN_BITS = 100


def _filtered_sign(metric, dot, norm_left, norm_right, q: int) -> int | None:
    """`+1` / `-1`, когда знак остатка доказан с запасом; `None` — уступить точному пути."""

    if q not in (3, 4, 5, 6):
        return None
    if type(dot) is not sp.Add:
        return None  # дорогое место — только Pow(сумма, 2); прочие произведения строятся дёшево, и остаток часто рационален
    intervals = metric._density_exact_memo.intervals
    saved = iv.prec
    iv.prec = _PRECISION
    try:
        dot_box = density_interval_enclosure(dot, intervals)
        left_box = density_interval_enclosure(norm_left, intervals)
        right_box = density_interval_enclosure(norm_right, intervals)
        norms = left_box * right_box
        if q == 5:
            first = 8 * dot_box**2
            second = (3 + iv.sqrt(iv.mpf(5))) * norms
        else:
            first = {3: 4, 4: 2, 6: 4}[q] * dot_box**2
            second = {3: 1, 4: 1, 6: 3}[q] * norms
        residual = first - second
        scale = abs(first).b + abs(second).b
        limit = (scale * (iv.mpf(2) ** -_MARGIN_BITS)).b
        if residual.a > limit:
            return 1
        if -residual.b > limit:
            return -1
        return None
    except (DensityIntervalEnclosureUnsupported, ArithmeticError, TypeError, ValueError):
        return None
    finally:
        iv.prec = saved


def residual_sign(metric, dot, norm_left, norm_right, q: int) -> int:
    """Знак остатка подшага: ровно то, что вернул бы `_density_exact_sign(остаток)` на построенном выражении."""

    verdict = _filtered_sign(metric, dot, norm_left, norm_right, q)
    if verdict is not None:
        return verdict
    from .adaptive_density_fan import AdaptiveDensityFanInvalid, _sign

    norm_product = norm_left * norm_right
    dot_squared = squared(dot)
    if q == 3:
        residual = 4 * dot_squared - norm_product
    elif q == 4:
        residual = 2 * dot_squared - norm_product
    elif q == 5:
        residual = 8 * dot_squared - (3 + sp.sqrt(5)) * norm_product
    elif q == 6:
        residual = 4 * dot_squared - 3 * norm_product
    else:
        raise AdaptiveDensityFanInvalid("unsupported Density q")
    return _sign(residual, metric)
