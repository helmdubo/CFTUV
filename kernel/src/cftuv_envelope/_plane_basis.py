"""Приведённый целочисленный базис плоскости: репер near-planar домена с малым Грамом.

Модуль внутренний, как `_embedding` и `_width_distortion`; один код у строителя и
у проверяющего. Решает одну задачу: после near-planar проекции матрица Грама
репера «от разностей спроецированных вершин» несёт знаменатели проекции.

    p' = p − ((p − o)·n / (n·n)) n          — знаменатель каждой координаты
                                              растёт на `n·n` (до 63 бит);
    G  = [A·A, A·B; A·B, B·B]               — квадрат этого знаменателя.

Квадраты длин рёбер решётки `d^T G d` — радиканды `SqrtSumV1`, и их простые
делители растут вместе с Грамом: замер `building.004` patch 4 — 110-битный
радикант, на котором Ро-Поллард не возвращается за кап 2^23 (простые до 73 бит).

Закон `REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1` строит репер ИНАЧЕ: `A = w1 / S`,
`B = w2 / S`, где `S` — масштаб решётки источника (позиции источника — целые,
делённые на `S`), а `(w1, w2)` — приведённый (Лагранж—Гаусс) базис целочисленной
решётки плоскости `{w ∈ Z³ : w·n = 0}` с примитивной нормалью `n`. Репер лежит в
той же плоскости, начало то же, координаты вершин по-прежнему точны
(`origin + u·A + v·B` восстанавливает спроецированную позицию без округления), —
меняется только базис: Грам — целые порядка `|n|`, делённые на `S²`.
"""

from __future__ import annotations

from fractions import Fraction


def _dot(left, right) -> int:
    return sum(a * b for a, b in zip(left, right, strict=True))


def _frac_dot(left, right) -> Fraction:
    return sum((a * b for a, b in zip(left, right, strict=True)), Fraction(0))


def _extended_gcd(a: int, b: int) -> tuple[int, int, int]:
    """`(g, s, t)`: `g = gcd(a, b) >= 0`, `s·a + t·b = g`."""

    old_r, r = a, b
    old_s, s = 1, 0
    old_t, t = 0, 1
    while r:
        q = old_r // r
        old_r, r = r, old_r - q * r
        old_s, s = s, old_s - q * s
        old_t, t = t, old_t - q * t
    if old_r < 0:
        old_r, old_s, old_t = -old_r, -old_s, -old_t
    return old_r, old_s, old_t


def reduced_plane_basis(normal) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    """Приведённый базис целочисленной решётки плоскости `{w : w·n = 0}`.

    `normal` — примитивный целочисленный вектор (`canonical_primitive_normal`).
    Две образующие строятся расширенным `gcd` и приводятся Лагранжем—Гауссом на
    целых: `|2 w1·w2| <= |w1|² <= |w2|²`, и они порождают ВСЮ решётку, а не
    подрешётку (`w1 × w2 = ±n`). Результат ЕДИНСТВЕННЫЙ: ничья по длине
    разрешается лексикографически, знак каждой образующей — первая ненулевая
    координата положительна. Порядок и знаки — часть закона: проверяющий
    пересчитывает тот же базис, а не сверяет произвольный.
    """

    a, b, c = (int(item) for item in normal)
    if a == 0 and b == 0:
        first, second = (1, 0, 0), (0, 1, 0)
    else:
        g, s, t = _extended_gcd(a, b)
        first = (b // g, -(a // g), 0)
        second = (-c * s, -c * t, g)
    while True:
        if _dot(second, second) < _dot(first, first) or (
            _dot(second, second) == _dot(first, first) and second < first
        ):
            first, second = second, first
        norm = _dot(first, first)
        k = (2 * _dot(first, second) + norm) // (2 * norm)
        if k == 0:
            break
        second = tuple(x - k * y for x, y in zip(second, first, strict=True))

    def positive(vector):
        lead = next((item for item in vector if item), 0)
        return tuple(-item for item in vector) if lead < 0 else vector

    return positive(first), positive(second)


def reduced_frame(*, normal, source_scale: int):
    """`(A, B)` в метрах: `w / source_scale` от приведённого базиса плоскости."""

    first, second = reduced_plane_basis(normal)
    step = Fraction(1, int(source_scale))
    return (
        tuple(step * item for item in first),
        tuple(step * item for item in second),
    )


def chart_of_positions(positions, origin, basis_a, basis_b) -> dict:
    """Координаты `(u, v)` позиций в реперe `(origin, A, B)` — точно, из Грама."""

    g00 = _frac_dot(basis_a, basis_a)
    g01 = _frac_dot(basis_a, basis_b)
    g11 = _frac_dot(basis_b, basis_b)
    determinant = g00 * g11 - g01 * g01
    result = {}
    for vertex_id, position in positions.items():
        relative = tuple(position[axis] - origin[axis] for axis in range(3))
        rhs_a, rhs_b = _frac_dot(basis_a, relative), _frac_dot(basis_b, relative)
        result[vertex_id] = (
            (g11 * rhs_a - g01 * rhs_b) / determinant,
            (g00 * rhs_b - g01 * rhs_a) / determinant,
        )
    return result

