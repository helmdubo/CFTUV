"""Закон `SILHOUETTE_TOPOLOGY_V1` (срез S1): рёбра и вершины декали остаются, только если они рисуют силуэт.

ЗАПРОС ВЛАДЕЛЬЦА: «я хочу сетку, где рёбра и вершины декали создают какой-то силуэт, а всё что не влияет на силуэт геометрии,
должны либо раствориться, либо не появиться». Закон `PLANAR_POLYGONS_V1` оставляет многоугольник целым, пока контур прост, а UV
аффинна, но куски резки и перекладины потока всё равно остаются отдельными гранями: ребро между двумя гранями одного региона в
одной плоскости и с одной аффинной UV ничего не рисует, а вершина на прямой между двумя рёбрами — тем более.

ГДЕ ЭТО ДЕЛАЕТСЯ. Пост-проход материализатора, ПОСЛЕ резки (`clip`), закона топологии и положения вершин `src:` и ДО цепей и
дайджестов (`assemble_batch`): цепи, станции, происхождение и дайджест строятся уже по растворённой сетке, поэтому батч и меш
хоста тождественны (хост ничего не чинит, ширина декали переписывает меш тем же путём). Позиции судятся ДО смещения хоста.
Закон — отдельный член `DecalTopologyLawV1` (надмножество `PLANAR_POLYGONS_V1`): прежние три закона побитово те же.

ЗАКОН.

1. РЁБРА. Остаются: контур (ребро одной грани), шов и граница между регионами. Растворяется внутреннее ребро одного региона,
   когда (а) объединение двух граней — простой контур (грани делят ровно это ребро и больше ни одной вершины; сетка домена без
   T-стыков, поэтому объединение двух простых граней по одному ребру проста), (б) UV в объединении ТОЧНО аффинна по положению на
   карте (`uv_is_affine_in_chart`; билинейные объединения не сливаются), (в) каждая вершина меньшей грани отстоит от плоскости
   большей не дальше `CLIP_DIAGONAL_CHORD_BUDGET` (5 мм, запись реестра `CLIP_DIAGONAL_CHORD_DEPTH_V1`) и все вершины объединения —
   не дальше от плоскости объединения (грань не дрейфует: плоскость пересчитывается по каждому слиянию). Порядок жадный
   и детерминированный: самое плоское первым (двугранный угол по возрастанию, затем ключи вершин). Порога по углу нет: суд —
   глубина хорды. Невыпуклый плоский многоугольник с аффинной UV законен (`PLANAR_AFFINE_UV_POLYGON_V1`: любая триангуляция
   показа даёт ту же поверхность и ту же UV).
2. ВЕРШИНЫ. Вершина двух рёбер (степень 2), не лежащая на цепи источника или стены (`boundary:SOURCE`, `boundary:WALL`) и не
   имеющая двойника-копии (разрез кольца), растворяется, когда ВСЕ растворённые между её соседями вершины отстоят от
   выпрямленного ребра не дальше глубины хорды, а UV, которую даёт интерполяция вдоль выпрямленного ребра, отличается от
   прежней не больше `DecalRequestV1.silhouette_uv_slide` (запись реестра `SILHOUETTE_UV_SLIDE_V1`, политика запроса, доля alpha). Грани после
   растворения — простые контуры (точно), а в треугольнике выпрямления нет ни одной чужой вершины, поэтому грани не наезжают.
   Вершины цепей источника и стены S1 не трогает никогда: они общие с соседними доменами по `location:src:`, и T-стыков шва
   (`ADAPTER_SEAM_T_JUNCTIONS`) закон не рождает; вершины `clip:` внутри домена свободны.

ЧТО ПИШЕТСЯ. Любой отказ растворить назван счётчиком (`KEPT_*`), а наибольшие глубина хорды и сдвиг UV, на которые закон пошёл,
записаны (нанометры и тысячные alpha). Нулевые числа в счётчики не пишутся: домен, где ничего не растворилось, не получает
новых строк. Исчерпание бюджета точной работы внутри прохода не отказывает домен: проход берёт свой отрезок бюджета, и если он
кончился, сетка остаётся ровно той, что была, под названным счётчиком `SKIPPED_WORK_BUDGET`.

ПРЕДЕЛ ЗАКОНА, ИЗМЕРЕННЫЙ. Точная аффинность UV объединения (пункт 1, б) отсекает почти все слияния на кривых доменах: ребро
между гранями одного региона с НЕПРЕРЫВНОЙ, но изломанной UV (смена пробега, перекладина JOIN, билинейная грань) в плоскости
лежит, но слияние отдало бы выбор UV триангуляции показа Blender'а (разница до десятков тысячных alpha и больше). `sagging_wall`
alpha 0.987: из 91 пары-кандидата патча 1 аффинны 13. Эти рёбра остаются (`KEPT_NOT_AFFINE`), каждое посчитано.

КОНТУРЫ. Цепи (`chains_of`) строятся по контурам слитых граней, а не по граням сетки, поэтому вершина, в которой сходятся
контуры нескольких слитых граней одного региона (после растворения рёбер между ними она стала вершиной двух рёбер), не
растворяется (`KEPT_JUNCTION`): растворение развело бы контуры, и граница цепей разошлась бы с границей сетки. Растворённая
вершина уходит и из контуров, и из `vertex_cycles`, и из позиций и фактов станций.

ПРОВЕРКА. `verify_silhouette` пересчитывает независимо от прохода: ни одна растворённая вершина не лежала на цепи источника или
стены (по `chains_of` исходной сетки), у уцелевших вершин факты `(s, r)` те же точно, глубина хорды и сдвиг UV каждой растворённой
вершины относительно ребра, которое её заменило в итоговой сетке (в каждом регионе граней ребра), не больше записанных
максимумов, а те — не больше допусков, растворённые вершины покрыты рёбрами итога ровно по разу, грани образуют многообразие
(полурёбра попарно разные, граница сократилась ровно растворёнными вершинами), а граница итоговых граней равна рёбрам граничных
цепей по итоговым контурам. Любое расхождение — отказ `BATCH_DID_NOT_VALIDATE` с именем `SILHOUETTE:*`.
"""

from __future__ import annotations

import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from fractions import Fraction

from ..exact_sqrt_sum import ExactCanonicalizationWorkBudgetExhausted, exact_work_budget
from ..float_filter import affine_map_violated, centre_and_bound
from ..wavefront.faces import orientation, segments_cross
from .admit import MaterializationOutcome
from .assemble import chains_of, edge_kind
from .audit import location_key
from .clip_cells import CLIP_DIAGONAL_CHORD_BUDGET
from .frames import MaterializationRefusal
from .tessellate import affine_frame, uv_vertex_on_affine_map
from .uv_law import uv_direct_strip_v1

LAW = "SILHOUETTE_TOPOLOGY_V1"

#: Имена чисел закона (они же ключи счётчиков материализатора). Нулевые числа в счётчики не пишутся.
EDGES_DISSOLVED = "MATERIALIZE_SILHOUETTE_EDGES_DISSOLVED"
VERTICES_DISSOLVED = "MATERIALIZE_SILHOUETTE_VERTICES_DISSOLVED"
KEPT_UV = "MATERIALIZE_SILHOUETTE_KEPT_UV"
KEPT_CHORD = "MATERIALIZE_SILHOUETTE_KEPT_CHORD"
KEPT_NOT_AFFINE = "MATERIALIZE_SILHOUETTE_KEPT_NOT_AFFINE"
KEPT_NOT_SIMPLE = "MATERIALIZE_SILHOUETTE_KEPT_NOT_SIMPLE"
KEPT_JUNCTION = "MATERIALIZE_SILHOUETTE_KEPT_JUNCTION"
MAX_UV_SLIDE_MILLI_ALPHA = "MATERIALIZE_SILHOUETTE_MAX_UV_SLIDE_MILLI_ALPHA"
MAX_CHORD_NM = "MATERIALIZE_SILHOUETTE_MAX_CHORD_NM"
SKIPPED_WORK_BUDGET = "MATERIALIZE_SILHOUETTE_SKIPPED_WORK_BUDGET"
SKIPPED_NOT_MANIFOLD = "MATERIALIZE_SILHOUETTE_SKIPPED_NOT_MANIFOLD"
COUNTER_NAMES = (
    EDGES_DISSOLVED,
    VERTICES_DISSOLVED,
    KEPT_UV,
    KEPT_CHORD,
    KEPT_NOT_AFFINE,
    KEPT_NOT_SIMPLE,
    KEPT_JUNCTION,
    MAX_UV_SLIDE_MILLI_ALPHA,
    MAX_CHORD_NM,
    SKIPPED_WORK_BUDGET,
    SKIPPED_NOT_MANIFOLD,
)

NANOMETRES_PER_METRE = 10**9
MILLI_ALPHA_PER_ALPHA = 1000
#: Глубина хорды закона — та же запись реестра, что у диагонали резки: одно число, одно место.
CHORD_BUDGET = CLIP_DIAGONAL_CHORD_BUDGET


@dataclass(frozen=True, slots=True)
class SilhouetteInputV1:
    """Сетка домена перед проходом: всё, что закон читает (ничего не меняет)."""

    polygons: list
    cycles: list
    vertex_cycles: object
    positions: dict
    points: dict
    facts: dict
    frame_faces: list
    layout: object
    lattice_alpha: Fraction
    uv_slide: Fraction


@dataclass(frozen=True, slots=True)
class SilhouetteV1:
    """Итог прохода: сетка после растворения и числа закона."""

    polygons: list
    cycles: list
    vertex_cycles: object
    positions: dict
    facts: dict
    #: `{(номер слитой грани, многоугольник): (номера остальных слитых граней)}`: объединённая грань наследует происхождение всех.
    merged_frames: dict
    dissolved: frozenset
    counters: tuple
    note: str
    #: Сетка изменилась (растворено хоть одно ребро или вершина).
    changed: bool = False
    #: Рёбра, заменившие цепочки растворённых вершин: `((сосед до, сосед после, (растворённые вершины)), ...)`.
    runs: tuple = ()


class _Facts:
    """Факты `(s, r)` одного региона как отображение `ключ -> (s, r)` для точных тождеств аффинности."""

    __slots__ = ("facts", "region")

    def __init__(self, facts, region) -> None:
        self.facts = facts
        self.region = region

    def __getitem__(self, key):
        return self.facts[(self.region, key)]


class _Face:
    """Грань сетки: кольцо ключей, регион, номера слитых граней (происхождение) и места."""

    __slots__ = ("ring", "region", "frames", "order", "cert", "plane", "repeats")

    def __init__(self, ring, region, frames, order) -> None:
        self.ring = ring
        self.region = region
        self.frames = frames
        self.order = order
        #: Аффинная карта UV грани: `None` — не считалась, `False` — не аффинна, иначе `(основа, кадр)`.
        self.cert = None
        self.plane = None
        self.repeats = len(set(ring)) != len(ring)


def _pairs(ring):
    return zip(ring, ring[1:] + ring[:1])


def _plane_of(points):
    """`(центр, единичная нормаль, удвоенная площадь)` кольца точек 3D (Ньюэлл от центра) либо `None`: нормали нет."""

    count = len(points)
    cx = sum(point[0] for point in points) / count
    cy = sum(point[1] for point in points) / count
    cz = sum(point[2] for point in points) / count
    nx = ny = nz = 0.0
    for first, second in _pairs(tuple(points)):
        ax, ay, az = first[0] - cx, first[1] - cy, first[2] - cz
        bx, by, bz = second[0] - cx, second[1] - cy, second[2] - cz
        nx += ay * bz - az * by
        ny += az * bx - ax * bz
        nz += ax * by - ay * bx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if not length > 0.0 or not math.isfinite(length):
        return None
    return (cx, cy, cz), (nx / length, ny / length, nz / length), length


def _distance_to_plane(plane, point) -> float:
    (cx, cy, cz), (nx, ny, nz), _area = plane
    return abs((point[0] - cx) * nx + (point[1] - cy) * ny + (point[2] - cz) * nz)


def _along(first, second, point):
    """`(параметр, расстояние)` точки `point` до отрезка `first - second`: проекция с зажимом в `[0, 1]`."""

    dx, dy, dz = second[0] - first[0], second[1] - first[1], second[2] - first[2]
    length_squared = dx * dx + dy * dy + dz * dz
    px, py, pz = point[0] - first[0], point[1] - first[1], point[2] - first[2]
    share = 0.0 if length_squared == 0.0 else max(0.0, min(1.0, (px * dx + py * dy + pz * dz) / length_squared))
    ex, ey, ez = px - dx * share, py - dy * share, pz - dz * share
    return share, math.sqrt(ex * ex + ey * ey + ez * ez)


def _on_map(points, values, base, frame, key) -> bool:
    """Вершина `key` лежит на аффинной карте UV (`base`, `frame`): точно; нарушение, доказанное в binary64, платит не точным путём."""

    return not affine_map_violated(points, values, base, key) and uv_vertex_on_affine_map(points, values, base, frame, key)


def _on_open_segment(first, second, other, budget) -> bool:
    """Точка `other` лежит на ОТКРЫТОМ отрезке `first - second` (коллинеарна и строго между концами). Точно."""

    if orientation(first, second, other, budget):
        return False
    along = ((second[0] - first[0]) * (other[0] - first[0]) + (second[1] - first[1]) * (other[1] - first[1])).sign(budget=budget)
    beyond = ((first[0] - second[0]) * (other[0] - second[0]) + (first[1] - second[1]) * (other[1] - second[1])).sign(budget=budget)
    return along > 0 and beyond > 0


def _within(value: float, budget: Fraction) -> bool:
    """Конечное число не больше точного допуска (сравнение дробей, без округления допуска)."""

    return math.isfinite(value) and Fraction(value) <= budget


def _nanometres(metres: float) -> int:
    return math.ceil(Fraction(metres) * NANOMETRES_PER_METRE)


def _milli_alpha(slide: float) -> int:
    return math.ceil(Fraction(slide) * MILLI_ALPHA_PER_ALPHA)


class _Mesh:
    """Сетка домена по граням: рёбра, плоскости, карты UV; растворяет рёбра, затем вершины."""

    def __init__(self, source: SilhouetteInputV1, budget) -> None:
        self.source = source
        self.budget = budget
        self.tally: Counter = Counter()
        self.max_chord = 0.0
        self.max_slide = 0.0
        self.faces: dict = {}
        self.half: dict = {}
        self.manifold = True
        self._xyz: dict = {}
        self._uv: dict = {}
        #: Контуры слитых граней по ключам (цепи строятся по ним): растворённая вершина уходит и из контуров.
        self.contours = [[key for key, _point in cycle] for cycle in source.cycles]
        self.in_contours: dict = defaultdict(list)
        for index, cycle in enumerate(self.contours):
            for key in cycle:
                self.in_contours[key].append(index)
        number = 0
        for index, face_polygons in enumerate(source.polygons):
            region = source.layout.region_of(source.frame_faces[index])
            for place, ring in enumerate(face_polygons):
                self.faces[number] = _Face(tuple(ring), region, (index,), (index, place))
                for edge in _pairs(self.faces[number].ring):
                    self.manifold = self.manifold and edge not in self.half
                    self.half[edge] = number
                number += 1
        self.next_id = number

    # -- величины -----------------------------------------------------------------------------

    def xyz(self, key):
        found = self._xyz.get(key)
        if found is None:
            point = self.source.positions[key]
            found = self._xyz[key] = (point.x, point.y, point.z)
        return found

    def uv(self, region, key):
        found = self._uv.get((region, key))
        if found is None:
            s, r = self.source.facts[(region, key)]
            point = uv_direct_strip_v1(s, r, self.source.lattice_alpha)
            found = self._uv[(region, key)] = (point.u, point.v)
        return found

    def box(self, keys):
        """Охватывающий прямоугольник точек карты `(x0, x1, y0, y1)` по доказанным границам binary64, либо `None`, если число фильтр не берёт."""

        found = []
        for key in keys:
            for coordinate in self.source.points[key]:
                bounded = centre_and_bound(coordinate)
                if bounded is None:
                    return None
                found.append(bounded)
        xs, ys = found[0::2], found[1::2]
        return (
            min(c - e for c, e in xs),
            max(c + e for c, e in xs),
            min(c - e for c, e in ys),
            max(c + e for c, e in ys),
        )

    def plane(self, face):
        if face.plane is None:
            face.plane = _plane_of([self.xyz(key) for key in face.ring]) or False
        return face.plane

    def dihedral(self, first, second) -> float:
        """Угол между нормалями двух граней (радианы), `atan2` от синуса и косинуса: малые углы различимы; нормали нет — бесконечность."""

        one, two = self.plane(first), self.plane(second)
        if not one or not two:
            return math.inf
        (ax, ay, az), (bx, by, bz) = one[1], two[1]
        cross = math.sqrt((ay * bz - az * by) ** 2 + (az * bx - ax * bz) ** 2 + (ax * by - ay * bx) ** 2)
        return math.atan2(cross, ax * bx + ay * by + az * bz)

    def certificate(self, face):
        """Карта UV грани: `(основа, кадр)` либо `False` (UV не аффинна по положению на карте). Точно; память на грани."""

        if face.cert is None:
            face.cert = self._solve(face)
        return face.cert

    def _solve(self, face):
        points, ring, values = self.source.points, face.ring, _Facts(self.source.facts, face.region)
        base = None
        for second in range(1, len(ring)):
            for third in range(second + 1, len(ring)):
                if orientation(points[ring[0]], points[ring[second]], points[ring[third]], self.budget):
                    base = (ring[0], ring[second], ring[third])
                    break
            if base is not None:
                break
        if base is None:
            return False
        frame = affine_frame(points, base)
        if all(_on_map(points, values, base, frame, key) for key in ring if key not in base):
            return base, frame
        return False

    # -- рёбра --------------------------------------------------------------------------------

    def dissolve_edges(self) -> None:
        candidates = []
        for (first, second), number in self.half.items():
            other = self.half.get((second, first))
            if first >= second or other is None or other == number:
                continue
            one, two = self.faces[number], self.faces[other]
            if one.region == two.region:
                candidates.append((self.dihedral(one, two), first, second))
        for _angle, first, second in sorted(candidates):
            self._try_merge(first, second)

    def _try_merge(self, first, second) -> None:
        one_id, two_id = self.half.get((first, second)), self.half.get((second, first))
        if one_id is None or two_id is None or one_id == two_id:
            return
        one, two = self.faces[one_id], self.faces[two_id]
        if one.repeats or two.repeats or set(one.ring) & set(two.ring) != {first, second}:
            self.tally[KEPT_NOT_SIMPLE] += 1
            return
        union = self._union(one, two, first, second)
        depth = self._merge_depth(one, two, union)
        if depth is None or not _within(depth, CHORD_BUDGET):
            self.tally[KEPT_CHORD] += 1
            return
        cert = self._affine_union(one, two)
        if not cert:
            self.tally[KEPT_NOT_AFFINE] += 1
            return
        merged = _Face(union, one.region, tuple(sorted({*one.frames, *two.frames})), min(one.order, two.order))
        merged.cert = cert
        for number, face in ((one_id, one), (two_id, two)):
            for edge in _pairs(face.ring):
                del self.half[edge]
            del self.faces[number]
        self.faces[self.next_id] = merged
        for edge in _pairs(union):
            self.half[edge] = self.next_id
        self.next_id += 1
        self.tally[EDGES_DISSOLVED] += 1
        self.max_chord = max(self.max_chord, depth)

    @staticmethod
    def _union(one, two, first, second):
        """Кольцо объединения двух граней по общему ребру `first -> second` (в `one`) и `second -> first` (в `two`)."""

        at = one.ring.index(second)
        start = one.ring[at:] + one.ring[:at]
        at = two.ring.index(first)
        rest = two.ring[at:] + two.ring[:at]
        return start + rest[1:-1]

    def _merge_depth(self, one, two, union):
        """Наибольшее отклонение от плоскости при слиянии (метры) либо `None`, если нормали нет."""

        big, small = (one, two) if self._twice_area(one) >= self._twice_area(two) else (two, one)
        plane = self.plane(big)
        if not plane:
            return None
        depth = max(_distance_to_plane(plane, self.xyz(key)) for key in small.ring)
        # Грань объединения лежит в допуске и от СОБСТВЕННОЙ плоскости: иначе цепочка слияний по медленно изгибающейся
        # поверхности уводила бы первые вершины грани от её плоскости без предела.
        fit = _plane_of([self.xyz(key) for key in union])
        if fit is None:
            return None
        return max(depth, max(_distance_to_plane(fit, self.xyz(key)) for key in union))

    def _twice_area(self, face) -> float:
        plane = self.plane(face)
        return plane[2] if plane else 0.0

    def _affine_union(self, one, two):
        """Карта UV объединения либо `False`: вершины второй грани вне первой лежат на карте первой (точно).

        Объединение аффинно ровно тогда, когда аффинна грань с картой и вершины второй лежат на ней: вторая тогда имеет ту же
        карту (три её неколлинеарные вершины на ней лежат), поэтому собственная карта второй не нужна. Грань с готовой картой
        берётся основой; у обеих нет — большая по числу вершин.
        """

        if one.cert is None and (two.cert or len(two.ring) > len(one.ring)):
            one, two = two, one
        cert = self.certificate(one)
        if not cert:
            return False
        base, frame = cert
        points, values, known = self.source.points, _Facts(self.source.facts, one.region), set(one.ring)
        if all(_on_map(points, values, base, frame, key) for key in two.ring if key not in known):
            return cert
        return False

    # -- вершины ------------------------------------------------------------------------------

    def dissolve_vertices(self):
        """`(растворённые вершины, {ребро: цепочка растворённых между его концами})`."""

        incident: dict = defaultdict(set)
        for number, face in self.faces.items():
            for key in face.ring:
                incident[key].add(number)
        fixed = self._fixed_vertices(incident)
        covered: dict = {}
        order = []
        for key in sorted(incident):
            found = None if key in fixed else self._line(key, incident)
            if found is not None:
                order.append((_along(self.xyz(found[0]), self.xyz(found[1]), self.xyz(key))[1], key))
        gone: set = set()
        for _sag, key in sorted(order):
            if self._try_dissolve(key, incident, covered, gone):
                gone.add(key)
        return frozenset(gone), covered

    def _fixed_vertices(self, incident) -> set:
        """Вершины, которые закон не растворяет: цепи источника и стены, вершины с двойником-копией (разрез кольца)."""

        fixed: set = set()
        for (first, second), number in self.half.items():
            if (second, first) in self.half:
                continue
            region = self.faces[number].region
            if edge_kind(self.source.facts, region, first, second, self.source.lattice_alpha) != "RIM":
                fixed.update((first, second))
        by_place: dict = defaultdict(set)
        for key in incident:
            by_place[location_key(key)].add(key)
        fixed.update(key for keys in by_place.values() if len(keys) > 1 for key in keys)
        return fixed

    def _line(self, key, incident):
        """`(сосед до, сосед после)` вершины двух рёбер либо `None`: степень не два, чужие грани, повтор вершины."""

        numbers = incident[key]
        if not 1 <= len(numbers) <= 2:
            return None
        around = []
        for number in sorted(numbers):
            face = self.faces[number]
            if face.repeats:
                return None
            at = face.ring.index(key)
            around.append((face.ring[at - 1], face.ring[(at + 1) % len(face.ring)]))
        before, after = around[0]
        if len(around) == 2 and around[1] != (after, before):
            return None
        return before, after

    def _try_dissolve(self, key, incident, covered, gone) -> bool:
        line = self._line(key, incident)
        if line is None:
            return False
        before, after = line
        if not self._contour_line(key, before, after):
            self.tally[KEPT_JUNCTION] += 1
            return False
        left, right = frozenset((before, key)), frozenset((key, after))
        run = (*covered.get(left, ()), *covered.get(right, ()), key)
        low, high = sorted((before, after))  # концы по ключам: те же числа, что у проверки (обход ребра на округление не влияет)
        start, end = self.xyz(low), self.xyz(high)
        worst = max(_along(start, end, self.xyz(item))[1] for item in run)
        if not _within(worst, CHORD_BUDGET):
            self.tally[KEPT_CHORD] += 1
            return False
        slide = self._slide(incident[key], low, high, run)
        if slide is None or not _within(slide, self.source.uv_slide):
            self.tally[KEPT_UV] += 1
            return False
        if not self._removable(key, before, after, incident, gone):
            self.tally[KEPT_NOT_SIMPLE] += 1
            return False
        for number in incident.pop(key):
            face = self.faces[number]
            for edge in _pairs(face.ring):
                del self.half[edge]
            face.ring = tuple(item for item in face.ring if item != key)
            for edge in _pairs(face.ring):
                self.half[edge] = number
        for index in set(self.in_contours.pop(key, ())):
            self.contours[index].remove(key)
        covered.pop(left, None)
        covered.pop(right, None)
        covered[frozenset((before, after))] = run
        self.tally[VERTICES_DISSOLVED] += 1
        self.max_chord = max(self.max_chord, worst)
        self.max_slide = max(self.max_slide, slide)
        return True

    def _contour_line(self, key, before, after) -> bool:
        """В каждом контуре слитой грани, где есть вершина, её соседи — те же `before` и `after`.

        Цепи строятся по контурам слитых граней, а не по граням сетки: вершина, в которой сходятся контуры трёх слитых граней
        одного региона, после слияния граней стала вершиной двух рёбер, но её растворение развело бы контуры (полурёбра
        соседних контуров перестали бы быть парой, и граница цепей разошлась бы с границей сетки).
        """

        pair = {before, after}
        for index in self.in_contours.get(key, ()):
            cycle = self.contours[index]
            at = cycle.index(key)
            if {cycle[at - 1], cycle[(at + 1) % len(cycle)]} != pair or cycle.count(key) != 1:
                return False
        return True

    def _slide(self, numbers, before, after, run):
        """Наибольший сдвиг UV (расстояние в единицах alpha) растворённых вершин `run` относительно выпрямленного ребра; `None` — фактов нет."""

        start, end, worst = self.xyz(before), self.xyz(after), 0.0
        for number in numbers:
            region = self.faces[number].region
            if any((region, item) not in self.source.facts for item in (before, after, *run)):
                return None
            (u0, v0), (u1, v1) = self.uv(region, before), self.uv(region, after)
            for item in run:
                share = _along(start, end, self.xyz(item))[0]
                u, v = self.uv(region, item)
                worst = max(worst, math.hypot(u - (u0 + (u1 - u0) * share), v - (v0 + (v1 - v0) * share)))
        return worst

    def _removable(self, key, before, after, incident, gone) -> bool:
        """Грани без вершины остаются простыми, а в треугольнике выпрямления нет чужих вершин. Точно.

        Ребро `before - after` новым быть обязано без отдельной проверки: уже стоящее ребро замкнуло бы с ломаной
        `before - key - after` треугольник, а он заполнен гранью степени два (грань-треугольник `key` схлопнулась бы: размер кольца).
        """

        points = self.source.points
        for number in incident[key]:
            if not self._shortcut_is_simple(self.faces[number].ring, key):
                return False
        corner = (points[before], points[key], points[after])
        if not orientation(corner[0], corner[1], corner[2], self.budget):
            return True
        around = self.box((before, key, after))
        for other, numbers in incident.items():
            if other in (before, key, after) or not numbers or other in gone:
                continue
            far = None if around is None else self.box((other,))
            if far is not None and (far[1] < around[0] or far[0] > around[1] or far[3] < around[2] or far[2] > around[3]):
                continue
            if self._in_corner(points, corner, points[other]):
                return False
        return True

    def _shortcut_is_simple(self, ring, key) -> bool:
        """Простое кольцо `ring` остаётся простым без вершины `key`: новое ребро не пересекает и не касается остальных. Точно.

        Кольцо без вершины простое ровно тогда, когда ребро `before - after` не пересекает трансверсально ни одно из остальных
        рёбер и не проходит через их вершины; из трёх вершин остаётся треугольник, он обязан иметь площадь.
        """

        points, at = self.source.points, ring.index(key)
        before, after = ring[at - 1], ring[(at + 1) % len(ring)]
        rest = tuple(item for item in ring if item != key)
        if len(rest) < 3:
            return False
        start, end = points[before], points[after]
        if len(rest) == 3:
            return bool(orientation(points[rest[0]], points[rest[1]], points[rest[2]], self.budget))
        for first, second in _pairs(rest):
            if before in (first, second) or after in (first, second):
                continue
            if segments_cross(start, end, points[first], points[second], self.budget):
                return False
        return not any(
            _on_open_segment(start, end, points[item], self.budget) for item in rest if item not in (before, after)
        )

    def _in_corner(self, points, corner, point) -> bool:
        """Точка в ЗАМКНУТОМ треугольнике `corner`: три знака ориентации не противоречат друг другу."""

        signs = [orientation(corner[at], corner[(at + 1) % 3], point, self.budget) for at in range(3)]
        return all(sign >= 0 for sign in signs) or all(sign <= 0 for sign in signs)

    # -- итог ---------------------------------------------------------------------------------

    def result(self, gone, covered, faces_before: int) -> SilhouetteV1:
        source = self.source
        polygons = [[] for _ in source.polygons]
        merged: dict = {}
        for number in sorted(self.faces, key=lambda item: self.faces[item].order):
            face = self.faces[number]
            index = face.order[0]
            polygons[index].append(face.ring)
            if len(face.frames) > 1:
                merged[(index, face.ring)] = tuple(item for item in face.frames if item != index)
        counters = _counters(self.tally, self.max_chord, self.max_slide)
        changed = bool(self.tally[EDGES_DISSOLVED] or self.tally[VERTICES_DISSOLVED])
        note = "" if not changed else (
            f"{LAW}: faces {faces_before} -> {len(self.faces)} edges_dissolved={self.tally[EDGES_DISSOLVED]} "
            f"vertices_dissolved={self.tally[VERTICES_DISSOLVED]} kept_uv={self.tally[KEPT_UV]} "
            f"kept_chord={self.tally[KEPT_CHORD]} kept_not_affine={self.tally[KEPT_NOT_AFFINE]} "
            f"kept_not_simple={self.tally[KEPT_NOT_SIMPLE]} kept_junction={self.tally[KEPT_JUNCTION]} max_chord_nm={_nanometres(self.max_chord)} "
            f"max_uv_slide_milli_alpha={_milli_alpha(self.max_slide)}"
        )
        return SilhouetteV1(
            polygons,
            [[item for item in cycle if item[0] not in gone] for cycle in source.cycles],
            None
            if source.vertex_cycles is None
            else [[item for item in cycle if item[0] not in gone] for cycle in source.vertex_cycles],
            {key: value for key, value in source.positions.items() if key not in gone},
            {slot: value for slot, value in source.facts.items() if slot[1] not in gone},
            merged,
            gone,
            counters,
            note,
            changed,
            tuple(sorted((*sorted(edge), run) for edge, run in covered.items())),
        )


def _counters(tally, max_chord: float, max_slide: float) -> tuple:
    """Числа закона, только ненулевые: домен, где закон ничего не сделал, не получает новых строк."""

    values = {name: tally[name] for name in COUNTER_NAMES}
    if tally[EDGES_DISSOLVED] or tally[VERTICES_DISSOLVED]:
        values[MAX_CHORD_NM] = _nanometres(max_chord)
        values[MAX_UV_SLIDE_MILLI_ALPHA] = _milli_alpha(max_slide)
    return tuple((name, values[name]) for name in COUNTER_NAMES if values[name])


def _unchanged(source: SilhouetteInputV1, counters: tuple) -> SilhouetteV1:
    return SilhouetteV1(
        source.polygons,
        source.cycles,
        source.vertex_cycles,
        source.positions,
        source.facts,
        {},
        frozenset(),
        counters,
        "",
    )


def dissolve_silhouette(source: SilhouetteInputV1, budget) -> SilhouetteV1:
    """Закон `SILHOUETTE_TOPOLOGY_V1`: сетка домена после растворения рёбер и вершин, не влияющих на силуэт.

    Проход берёт свой отрезок бюджета точной работы (остаток домена): если он кончился, сетка остаётся прежней под счётчиком
    `SKIPPED_WORK_BUDGET`, а бюджет домена не тронут; иначе потраченное списывается из бюджета домена.
    """

    mesh = _Mesh(source, budget)
    if not mesh.manifold:
        return _unchanged(source, ((SKIPPED_NOT_MANIFOLD, 1),))
    inner = budget if budget.cap is None else exact_work_budget(stage="MATERIALIZE", domain_id=budget.domain_id, cap=max(budget.remaining, 0))
    mesh.budget = inner
    faces_before = len(mesh.faces)
    try:
        mesh.dissolve_edges()
        gone, covered = mesh.dissolve_vertices()
    except ExactCanonicalizationWorkBudgetExhausted:
        return _unchanged(source, ((SKIPPED_WORK_BUDGET, 1),))
    if inner is not budget:
        budget.replay(inner.spent_by_article())
    return mesh.result(gone, covered, faces_before)


def verify_silhouette(source: SilhouetteInputV1, result: SilhouetteV1) -> tuple:
    """Независимый пересчёт: имена нарушений (пусто — закон выполнен). Читает исходную сетку и итог, проход не зовёт."""

    if not result.changed:
        return ()
    problems: list = []
    walls = {
        key
        for chain in chains_of(source.frame_faces, source.cycles, source.layout, source.facts, source.lattice_alpha)[0]
        if chain.semantic_boundary_id.value.split(":")[1] in ("SOURCE", "WALL")
        for key in (item.value for item in chain.ordered_vert_keys)
    }
    if walls & result.dissolved:
        problems.append("SOURCE_OR_WALL_VERTEX_DISSOLVED")
    kept = {slot: value for slot, value in source.facts.items() if slot[1] not in result.dissolved}
    if kept != result.facts:
        problems.append("KEPT_VERTEX_CHANGED_ITS_STATION_OR_UV")
    problems.extend(_verify_runs(source, result))
    problems.extend(_verify_topology(source, result))
    problems.extend(_verify_chains(source, result))
    return tuple(problems)


def _verify_runs(source, result) -> list:
    """Растворённые вершины покрыты рёбрами итога ровно по разу; глубина хорды и сдвиг UV каждой относительно своего ребра не больше записанных максимумов."""

    recorded = dict(result.counters)
    mesh = _Mesh(source, None)
    regions: dict = defaultdict(set)
    for index, face_polygons in enumerate(result.polygons):
        region = source.layout.region_of(source.frame_faces[index])
        for ring in face_polygons:
            for edge in _pairs(tuple(ring)):
                regions[frozenset(edge)].add(region)
    found, covered = [], []
    worst_chord = worst_slide = 0.0
    for before, after, run in result.runs:
        covered.extend(run)
        start, end = mesh.xyz(before), mesh.xyz(after)
        for region in regions.get(frozenset((before, after)), ()):
            if any((region, item) not in source.facts for item in (before, after, *run)):
                found.append("RUN_VERTEX_HAS_NO_FACT")
                continue
            (u0, v0), (u1, v1) = mesh.uv(region, before), mesh.uv(region, after)
            for item in run:
                share, _distance = _along(start, end, mesh.xyz(item))
                u, v = mesh.uv(region, item)
                worst_slide = max(worst_slide, math.hypot(u - (u0 + (u1 - u0) * share), v - (v0 + (v1 - v0) * share)))
        if frozenset((before, after)) not in regions:
            found.append("RUN_EDGE_IS_NOT_IN_THE_RESULT")
        for item in run:
            worst_chord = max(worst_chord, _along(start, end, mesh.xyz(item))[1])
    if sorted(covered) != sorted(result.dissolved):
        found.append("RUNS_DO_NOT_COVER_THE_DISSOLVED_VERTICES")
    if _nanometres(worst_chord) > recorded.get(MAX_CHORD_NM, 0) or not _within(worst_chord, CHORD_BUDGET):
        found.append("CHORD_DEPTH_BEYOND_RECORDED_MAXIMUM")
    if _milli_alpha(worst_slide) > recorded.get(MAX_UV_SLIDE_MILLI_ALPHA, 0) or not _within(worst_slide, source.uv_slide):
        found.append("UV_SLIDE_BEYOND_RECORDED_MAXIMUM")
    return found


def _directed(polygons) -> Counter:
    """Направленные рёбра граней по МЕСТАМ вершин (две копии вершины разреза кольца — одно место, как у аудита сетки)."""

    found: Counter = Counter()
    for face_polygons in polygons:
        for ring in face_polygons:
            found.update((location_key(a), location_key(b)) for a, b in _pairs(tuple(ring)))
    return found


def _outline(directed) -> set:
    return {frozenset(edge) for edge in directed if (edge[1], edge[0]) not in directed}


def _verify_chains(source, result) -> list:
    """Граница итоговых граней — ровно рёбра граничных цепей, построенных по итоговым контурам (тот же закон, что у аудита сетки)."""

    boundary = chains_of(source.frame_faces, result.cycles, source.layout, result.facts, source.lattice_alpha)[0]
    chain_edges = {
        frozenset((location_key(keys[at].value), location_key(keys[at + 1].value)))
        for keys in (chain.ordered_vert_keys for chain in boundary)
        for at in range(len(keys) - 1)
    }
    return ["OUTLINE_DOES_NOT_MATCH_THE_BOUNDARY_CHAINS"] if _outline(_directed(result.polygons)) != chain_edges else []


def _verify_topology(source, result) -> list:
    """Итоговые грани — многообразие: вершины живы, полурёбра разные, граница сократилась ровно растворёнными вершинами."""

    found = []
    live = set(result.positions)
    for face_polygons in result.polygons:
        for ring in face_polygons:
            if len(ring) < 3 or len(set(ring)) != len(ring) or any(key not in live for key in ring):
                found.append("FACE_IS_DEGENERATE_OR_USES_A_DISSOLVED_VERTEX")
                break
    after, before = _directed(result.polygons), _directed(source.polygons)
    if any(count > 1 for count in after.values()):
        found.append("HALF_EDGE_DUPLICATED")
    outline_before, outline_after = _outline(before), _outline(after)
    on_outline = {place for edge in outline_before for place in edge} & {location_key(key) for key in result.dissolved}
    if len(outline_after) != len(outline_before) - len(on_outline):
        found.append("OUTLINE_DOES_NOT_CONTRACT_BY_DISSOLVED_VERTICES")
    return found


def apply_silhouette(source: SilhouetteInputV1, budget) -> SilhouetteV1:
    """Проход и независимая проверка: любое нарушение — отказ `BATCH_DID_NOT_VALIDATE` с именем `SILHOUETTE:*` и числами."""

    result = dissolve_silhouette(source, budget)
    problems = verify_silhouette(source, result)
    if problems:
        raise MaterializationRefusal(
            MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
            "SILHOUETTE:" + ",".join(problems),
            result.counters,
        )
    return result
