"""UV-закон `UV_DIRECT_STRIP_V1`: прямое `(source_s, source_r) -> (u, v)`.

Решение владельца 2026-09-30: первый UV-закон продукта — без атласа, u вдоль
полосы, v поперёк. Закон записан здесь одним правилом:

    u = s / alpha        v = r / alpha

где `s` — станция вдоль `ChainUse` источника, `r` — время прихода (расстояние)
от несущей прямой источника, `alpha` — эффективная ширина полосы запроса.

Следствия, которые проверяются тестом, а не принимаются на слово:

* `v` в `[0, 1]`: на самой прямой источника `v = 0` (шов), на внешнем крае
  полосы `r = alpha` ТОЧНО, то есть `v = 1`;
* текселы изотропны: единица `u` и единица `v` — одна и та же длина
  `alpha` на поверхности, поэтому текстура не растянута;
* `u` не перезапускается на каждом ребре: `s0` копится вдоль ВСЕЙ цепи
  (`stations.chain_station_table`), и непрерывность по цепи ограничена только
  изломом (там у соседних пробегов свои системы координат, то есть шов UV);
* атласного прямоугольника нет: тайлинг — дело материала.

Считается ТОЧНО в единицах решётки: `s/alpha_lattice`, где
`alpha_lattice = alpha * scale`, — масштаб сокращается, и метры в закон не
входят вовсе; во float число превращается один раз (`sqrt_sum_binary64`).
"""

from __future__ import annotations

from fractions import Fraction

from ..exact_sqrt_sum import SqrtSumV1
from ..ids import PolicyId
from ..numeric import UvPoint2V1
from .lift import sqrt_sum_binary64

UV_DIRECT_STRIP_V1 = PolicyId("UV_DIRECT_STRIP_V1")

#: Законы, которые материализатор умеет. Всё остальное — именованный отказ
#: `UV_POLICY_UNSUPPORTED`, а не молчаливый запасной закон.
SUPPORTED_UV_POLICIES = frozenset({UV_DIRECT_STRIP_V1})


def uv_direct_strip_v1(
    s: SqrtSumV1, r: SqrtSumV1, lattice_alpha: Fraction
) -> UvPoint2V1:
    """`(u, v) = (s, r) / alpha`, точные `s`, `r` и `alpha` в единицах решётки."""

    inverse = Fraction(1) / Fraction(lattice_alpha)
    return UvPoint2V1(
        sqrt_sum_binary64(s.scaled(inverse)),
        sqrt_sum_binary64(r.scaled(inverse)),
    )
