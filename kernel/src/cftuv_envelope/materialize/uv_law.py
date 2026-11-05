"""UV-законы материализатора: политика `UV_DIRECT_STRIP_V1` (режим Straight strips) и `UV_UNIFORM_TRIM_FAN_V1`.

Одно правило перевода, общее для обоих: `u = U / alpha`, `v = r / alpha`, где `U` - числитель закона, `r` - время прихода (расстояние) от несущей прямой источника, `alpha` - эффективная ширина полосы запроса.

Числитель `U` Straight strips считает `uv_straighten` (закон `STRAIGHTEN_RUNGS_V1`, решение владельца 2026-11-05): одна pChain - прямой прямоугольный остров, как Straighten в Maya. Следствия, которые проверяются тестом, а не принимаются на слово:

* `v` в `[0, 1]`: на самой прямой источника `v = 0`, на внешнем крае полосы `r = alpha` ТОЧНО, то есть `v = 1`; где встречные фронты сомкнулись раньше (гребень, верхняя крышка), верх острова неровный при `v < 1`: высота трима постоянна, `v` к гребню не растягивается;
* основание прямое и `u` на нём - накопленная длина цепи, `s0` копится вдоль ВСЕЙ цепи (`stations.chain_station_table`);
* перекладина от вершины цепи вертикальна (одна `u` при любом `v`), торцы острова - `u = 0` и `u = L` при любом `v`; исключения названы числами `uv_straighten`;
* ЦЕНА: текселы НЕ изотропны там, где грань на поверхности - трапеция, а в UV - прямоугольник: единица `u` растянута поперёк трапеции, единица `v` нет. Это принятая цена прямых островов, замер - в реестре решений;
* атласного прямоугольника нет: тайлинг - дело материала.

Считается ТОЧНО в единицах решётки: `U / alpha_lattice`, где `alpha_lattice = alpha * scale`, - масштаб сокращается, и метры в закон не входят вовсе; во float число превращается один раз (`sqrt_sum_binary64`).
"""

from __future__ import annotations

from fractions import Fraction

from ..exact_sqrt_sum import SqrtSumV1
from ..ids import PolicyId
from ..numeric import UvPoint2V1
from .lift import sqrt_sum_binary64

UV_DIRECT_STRIP_V1 = PolicyId("UV_DIRECT_STRIP_V1")
UV_UNIFORM_TRIM_FAN_V1 = PolicyId("UV_UNIFORM_TRIM_FAN_V1")

#: Законы, которые материализатор умеет. Всё остальное — именованный отказ
#: `UV_POLICY_UNSUPPORTED`, а не молчаливый запасной закон.
SUPPORTED_UV_POLICIES = frozenset({UV_DIRECT_STRIP_V1, UV_UNIFORM_TRIM_FAN_V1})


def uv_direct_strip_v1(
    s: SqrtSumV1, r: SqrtSumV1, lattice_alpha: Fraction
) -> UvPoint2V1:
    """`(u, v) = (s, r) / alpha`: `s` - числитель `U` закона, точные числа и `alpha` в единицах решётки."""

    inverse = Fraction(1) / Fraction(lattice_alpha)
    return UvPoint2V1(
        sqrt_sum_binary64(s, factor=inverse),
        sqrt_sum_binary64(r, factor=inverse),
    )
