"""Мера излома окрестности вершины угла: факт закона «МИТРА НА ИЗЛОМЕ» (`CORNER_MITER_ON_FOLD_V1`).

Чистый лист без `reference` (как `_corner_treatment`): мерой пользуются и компиляция, и проверяющий плана.

ЗАЧЕМ. Закон JOIN (`CORNER_JOIN_SAME_PCHAIN_V1`) решает тождество цепи и ведёт полосу через излом ОДНОЙ цепи. Угол между
РАЗНЫМИ цепями (либо излом, который JOIN не берёт) остаётся веером под счётом плотности. На ПЛОСКОМ патче веер
правильный (он даёт полосе обойти вогнутую вершину), но на СКЛАДКЕ патча (стена, которая гнётся у угла: кривые
`sagging_wall`, `rounded_wall`) веер лежит на разных плоскостях источника, и его края режут грани поверх складки.
Владелец (2026-10-05) хочет в такой вершине митру со швом на биссектрисе, а не веер: `k = 0`, потока нет, шов на
биссектрисе — то же состояние, что у снятого JOIN (`JOIN_WITHDRAWN_AT_STATION_CONFLICT`).

МЕРА. `sin^2` двугранного угла между двумя треугольниками источника кольца-1 вершины УГЛА ВНУТРИ патча-владельца
угла, максимум по всем парам: `|n1 x n2|^2 / (|n1|^2 |n2|^2)`. Нормаль треугольника — векторное произведение двух
его рёбер в ТОЧНЫХ рациональных позициях снапшота (`Fraction(float)` точна); ни одного числа с плавающей точкой,
ни корня, ни тригонометрии: мера рациональна. На точной плоскости `n1 x n2 = 0` ТОЖДЕСТВЕННО, и мера нуль. Бюджет
`CORNER_FOLD_SIN2_BUDGET` — запись реестра допусков (AUTHORING_INTENT, меняет топологию: складка свыше бюджета
берёт митру вместо веера).

ЧЕГО МЕРА НЕ ЗНАЕТ, И ЭТО НАЗВАНО. Нет позиций (координатно-свободные фикстуры), нет ни одного невырожденного
треугольника кольца в патче владельца — меры нет (`None`), и закон инертен: угол решается прежним законом, ответ тот
же. Вырожденный (нулевой нормали) треугольник нормали не имеет и в пары не идёт. Кольцо с одним невырожденным
треугольником даёт нуль: складки там нет, пар нет.
"""

from __future__ import annotations

from fractions import Fraction

from .numeric import LocalPoint3V1

#: БЮДЖЕТ ИЗЛОМА ОКРЕСТНОСТИ: `sin^2` двугранного угла кольца-1 вершины угла, выше которого угол берёт митру со швом
#: вместо веера. Это `sin^2(1 градус) = 3.0459e-4`, округлённый ВНИЗ до рационального `3/10000` (0.992 градуса): шум
#: представления (1e-30 и меньше) и плавная кривизна кольца (десятые доли градуса) под бюджетом, настоящий излом
#: (на здании `building` вершина 46 патча 89: 2.045 градуса) над ним. Запись реестра допусков.
CORNER_FOLD_SIN2_BUDGET = Fraction(3, 10000)


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _norm_squared(vector):
    return vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]


def triangle_normal(corners):
    """Нормаль (не единичная) треугольника по трём точным углам: `(b - a) x (c - a)`."""

    first = tuple(b - a for a, b in zip(corners[0], corners[1]))
    second = tuple(b - a for a, b in zip(corners[0], corners[2]))
    return _cross(first, second)


def max_fold_sin2(normals) -> Fraction:
    """Максимум `sin^2` угла между парами ненулевых нормалей `normals` (точно); нуль, если пар нет."""

    worst = Fraction(0)
    squared = [_norm_squared(item) for item in normals]
    for first in range(len(normals)):
        for second in range(first + 1, len(normals)):
            value = Fraction(_norm_squared(_cross(normals[first], normals[second]))) / (
                Fraction(squared[first]) * Fraction(squared[second])
            )
            if value > worst:
                worst = value
    return worst


class CornerFoldFacts:
    """Меры излома окрестностей вершин снапшота: кольцо-1 строится один раз и лениво, нормали считаются по запросу.

    Один экземпляр на снапшот и на дверь (компиляция, сверка записей, проверяющий плана): состояния, видимого
    снаружи, нет, ответ зависит только от снапшота.
    """

    def __init__(self, snapshot) -> None:
        self._surface = snapshot.surface_ir
        self._positions = {item.vertex_id: item.position for item in snapshot.source_vertices}
        self._rings = None
        self._normals = {}

    def _ring(self, vertex_id, patch_id) -> list:
        if self._rings is None:
            patch_of_face = {item.face_id: item.patch_id for item in self._surface.source_faces}
            rings: dict = {}
            for triangle in self._surface.surface_triangles:
                patch = patch_of_face.get(triangle.source_face_id)
                if patch is None:
                    continue
                for vertex in triangle.vertex_ids:
                    rings.setdefault((vertex, patch), []).append(triangle)
            self._rings = rings
        return self._rings.get((vertex_id, patch_id), [])

    def _normal(self, triangle):
        """Точная нормаль треугольника; `None` — позиции недоступны; нулевая — вырожденный."""

        if triangle.triangle_id in self._normals:
            return self._normals[triangle.triangle_id]
        corners = []
        for vertex in triangle.vertex_ids:
            position = self._positions.get(vertex)
            if not isinstance(position, LocalPoint3V1):
                corners = None
                break
            corners.append((Fraction(position.x), Fraction(position.y), Fraction(position.z)))
        normal = None if corners is None else triangle_normal(corners)
        self._normals[triangle.triangle_id] = normal
        return normal

    def sin2_at(self, vertex_id, patch_id) -> Fraction | None:
        """Мера излома вершины в патче; `None` — меры нет (позиций нет либо кольцо в патче пусто)."""

        ring = self._ring(vertex_id, patch_id)
        if not ring:
            return None
        normals = []
        for triangle in ring:
            normal = self._normal(triangle)
            if normal is None:
                return None
            if any(component != 0 for component in normal):
                normals.append(normal)
        if not normals:
            return None
        return max_fold_sin2(normals)
