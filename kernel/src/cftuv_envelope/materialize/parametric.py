"""Аффинный сертификат перетаскивания ширины: значение и ТОЧНЫЙ наклон по ширине каждой вершины и каждой петли UV домена из ОДНОГО точного прогона.

ЗАЧЕМ. Живая ширина (`cftuv/envelope_width_mesh_preview`) двигает настоящий меш декали между точными пересчётами. Прежняя модель строила многочлен
по двум-трём точным прогонам разных ширин (замер, подгонка, «досягаемость»): ей нужны были затравочные прогоны на входе в инструмент и она не знала формы
зависимости. Но форма известна: внутри заверенного интервала (`interval.AlphaIntervalV1`) с той же структурой каждая позиция вершины батча и каждое
произведение `u * alpha`, `v * alpha` - ТОЧНО аффинная функция ширины (замер `S6_PARAMETRIC_DRAG_SPIKE`: 0 неаффинных из ~28 000 вершин). Сертификат
выпускает наклон прямо из прогона, а не подгонкой.

ОТКУДА НАКЛОН (по цепочке линейных стадий, ничего не подбирается).
1. Точки покрытия: шаблон покрытия (`wavefront.coverage_template`) уже хранит их как `A + alpha * B`; `B` - наклон точки отсечения, у исходных вершин грани он нулевой.
2. Подъём: плоскость и подъём в ЗАДАННОМ треугольнике источника линейны по точке карты; наклон позиции - линейная часть подъёма от наклона точки.
3. Станции `(s, r)`: `station_of` и `transverse_of` линейны по точке, поэтому наклон факта - те же формулы от наклона точки. Факт на перекладине JOIN (`rungs`) несёт
   станцию вершины цепи (константа таблицы) и `r` собственной формулы.
4. Узлы резки (`clip:`) и факты, которые резка дописывает, берутся ПО ТРЁМ ТОЧНЫМ ЗНАЧЕНИЯМ (`alpha0 - d`, `alpha0`, `alpha0 + d`) самой стадии резки: вторая разность
   нулевая ТОЧНО, и тогда функция аффинна (дробно-линейная `p + q / (x - c)` с `q != 0` даёт ненулевую вторую разность на любом конечном шаге). Это не подгонка: ноль второй
   разности доказывает аффинность на отрезке трёх значений, а отрезок лежит внутри заверенного интервала, где событий резки нет.

ЧТО НЕ АФФИННО, и это названо (`PARAM_NON_AFFINE_*`, `PARAM_*`): домен тогда НЕ обслуживается моделью хоста и остаётся на последней точной геометрии (контур показывает линия превью, а
пересчёт ведёт lean-такт). Ничего не молчит: причина лежит в `markers` сертификата.
* узел резки либо факт на нём не аффинен по трём значениям (`PARAM_NON_AFFINE_CLIP_NODE`, `PARAM_NON_AFFINE_CLIP_FACT`), структура резки в боковых точках другая
  (`PARAM_CLIP_STRUCTURE_CHANGED`), либо резка в боковой точке отказала (`PARAM_CLIP_REFUSED`);
* вершина на ребре треугольника источника, уходящая с него (`PARAM_NON_AFFINE_LIFT_ON_EDGE`: излом подъёма);
* два ответа на вершину совпали при alpha0, но их наклоны различны (`PARAM_IDENTITY_AT_POINT`: тождество выполняется в точке, а не на окне);
* точка покрытия с двумя наклонами (`PARAM_POINT_SLOPE_AMBIGUOUS`);
* шаблон покрытия или контуры покрытия не доступны (`PARAM_TEMPLATE_UNAVAILABLE`, `PARAM_CONTOURS_UNAVAILABLE`), интервал не заверен (`PARAM_WINDOW_NOT_CERTIFIED`).

ДОМЕН-РАЗВЁРТКА (`approximate`, не `markers`). Позиции его батча ДО смещения аффинны так же точно, как у плоского домена (теорема проверена на полевых развёртках), но хост сдвигает вершины
вдоль нормалей вершин (`SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`, нормированная смесь трёх нормалей углов треугольника: не рациональная) и митрой сварки. Ядро отдаёт те же числа
и называет такой домен в `approximate` (`PARAM_NON_AFFINE_DEVELOPABLE_OFFSET`): он аффинен ТОЛЬКО по батчу, меш хоста на сдвинутой ширине аффинен лишь приближённо. Что с этим делает
хост (смещение с нормалями, замороженными на ширине сертификата, предел ошибки и имя), решает хост (`envelope_width_preview_model`), не ядро.

ЧЕГО СЕРТИФИКАТ НЕ ЗАВЕРЯЕТ: решения тесселяции, силуэта, положения вершин `src:` у хоста и диагоналей ячейки (`interval.UNCERTIFIED`) он не доказывает - они проверяются
апостериори сравнением подписи структуры (`structure.batch_structure`) и токена локальных массивов, как раньше.

ЦЕНА. Сертификат считается ПОСЛЕ всего ответа (счётчики, валидация, аудит, дайджест, интервал) и на СВОЁМ бюджете (`unlimited_reference_budget`, `PARAMETRIC`): он не двигает ни
статьи `EXACT_WORK_*`, ни память резки (боковые резки идут мимо неё), ни ответ. `CFTUV_PARAMETRIC=0` выключает его (замер «до»).

ПАМЯТЬ НАКЛОНОВ (PARAMETRIC_SLOPE_MEMO_V1). Наклоны фактов (`_fact_slopes`) и наклоны позиций (`_plane_slope`, `_triangle_slope`) - чистые функции, в которые ширина НЕ входит:
их входы - формулы граней (пробег, прямая, веер, номер региона, ключи контура), наклоны точек (константы шаблона `B`) и перекладины; плоскость, треугольник и масштаб.
Воркер держит их значения при подготовке (`memo.MaterializeMemoV1.parametric`), и ключ записи - ПОЛНЫЙ перечень этих входов (`_fact_key`; у позиции - плоскость, треугольник, масштаб, наклон точки; читаемые поля
названы `FACT_KEY_READS` и держатся тестом по исходнику), а не ширина, интервал или подпись структуры: равный ключ - тот же вызов той же функции, ответ тот же побитово.
Всё, что от ширины зависит, считается КАЖДЫЙ раз: наклоны точек покрытия (`point_slopes_of`, и его отказы), узлы резки (`_clip_slopes`), уход с ребра (`_leaves_edge`), скорость
нормали развёртки (`_normal_rate`) и ширина `alpha` сертификата. Метка `ParametricCertificateV1.memo` называет, как получены наклоны (`MEMO_LABELS`), хост считает метки
(`PRODUCTION_PARAMETRIC_MEMO_*`). `CFTUV_PARAMETRIC_MEMO=0` (и `CFTUV_MATERIALIZE_MEMO=0`) выключают память; точный режим (`exact`, тесты теоремы) её не берёт.
"""

from __future__ import annotations

import math
import os
import threading
from array import array
from collections import Counter
from dataclasses import dataclass, field, replace
from fractions import Fraction

from ..exact_sqrt_sum import SqrtSumV1, unlimited_reference_budget
from .clip import cut_domain
from .clip_memo import kernel_code_identity, memo_disabled as clip_memo_disabled
from .frames import MaterializationRefusal
from .lift import PlaneLiftV1, enclosure_midpoint, ENCLOSURE_BITS
from .memo import memo_enabled, memo_of
from .stations import transverse_root
from ..wavefront.coverage_template import KIND_CUT

PARAMETRIC_LAW = "AFFINE_DRAG_CERTIFICATE_V1"
ENVIRONMENT_SWITCH = "CFTUV_PARAMETRIC"

PARAM_DEVELOPABLE_OFFSET = "PARAM_NON_AFFINE_DEVELOPABLE_OFFSET"
PARAM_CLIP_NODE = "PARAM_NON_AFFINE_CLIP_NODE"
PARAM_CLIP_FACT = "PARAM_NON_AFFINE_CLIP_FACT"
PARAM_LIFT_ON_EDGE = "PARAM_NON_AFFINE_LIFT_ON_EDGE"
PARAM_IDENTITY_AT_POINT = "PARAM_IDENTITY_AT_POINT"
PARAM_POINT_SLOPE_AMBIGUOUS = "PARAM_POINT_SLOPE_AMBIGUOUS"
PARAM_CLIP_STRUCTURE_CHANGED = "PARAM_CLIP_STRUCTURE_CHANGED"
PARAM_CLIP_REFUSED = "PARAM_CLIP_REFUSED"
PARAM_TEMPLATE_UNAVAILABLE = "PARAM_TEMPLATE_UNAVAILABLE"
PARAM_CONTOURS_UNAVAILABLE = "PARAM_CONTOURS_UNAVAILABLE"
PARAM_WINDOW_NOT_CERTIFIED = "PARAM_WINDOW_NOT_CERTIFIED"
PARAM_LIFT_UNSUPPORTED = "PARAM_LIFT_UNSUPPORTED"
PARAM_FACT_UNRESOLVED = "PARAM_FACT_UNRESOLVED"

MARKERS = (
    PARAM_DEVELOPABLE_OFFSET,
    PARAM_CLIP_NODE,
    PARAM_CLIP_FACT,
    PARAM_LIFT_ON_EDGE,
    PARAM_IDENTITY_AT_POINT,
    PARAM_POINT_SLOPE_AMBIGUOUS,
    PARAM_CLIP_STRUCTURE_CHANGED,
    PARAM_CLIP_REFUSED,
    PARAM_TEMPLATE_UNAVAILABLE,
    PARAM_CONTOURS_UNAVAILABLE,
    PARAM_WINDOW_NOT_CERTIFIED,
    PARAM_LIFT_UNSUPPORTED,
    PARAM_FACT_UNRESOLVED,
)

_ZERO = SqrtSumV1.zero()
_ZERO_PAIR = (_ZERO, _ZERO)
_FLOAT_ZERO3 = (0.0, 0.0, 0.0)
#: Боковые точки: доля расстояния до ближайшего края интервала (метры), на которую отходят вниз и вверх от alpha0, и число вдвое меньших шагов, которые пробуются, пока структура
#: резки в боковых точках не совпала с базой. Шаг круглый (степень двойки): коэффициенты точных величин не раздуваются.
_STEP_FRACTION = 16
_STEP_ATTEMPTS = 3
#: Боковые резки идут мимо памяти резки (`clip_memo.memo_disabled` переключает флаг процесса): один сертификат в процессе за раз, чтобы чужая резка не увидела выключенную память.
_SIDE_CUT_LOCK = threading.Lock()

MEMO_SWITCH = "CFTUV_PARAMETRIC_MEMO"
#: Метки памяти наклонов у сертификата: все наклоны из памяти, все посчитаны, часть из памяти; память выключена; точный режим; у подготовки нет памяти; сертификат отказал раньше первого обращения.
MEMO_HIT, MEMO_MISS, MEMO_PARTIAL = "HIT", "MISS", "PARTIAL"
MEMO_OFF, MEMO_BYPASS_EXACT, MEMO_NO_STORE, MEMO_NOT_REACHED = "OFF", "BYPASS_EXACT", "NO_STORE", "NOT_REACHED"
MEMO_LABELS = (MEMO_HIT, MEMO_MISS, MEMO_PARTIAL, MEMO_OFF, MEMO_BYPASS_EXACT, MEMO_NO_STORE, MEMO_NOT_REACHED)
#: Записей на подготовку: таблиц фактов (одна на набор формул и наклонов: интервал и структура) и наклонов позиций. Переполнение сбрасывает записи (цена, не ответ).
MEMO_FACT_LIMIT = 8
MEMO_POSITION_LIMIT = 65536
#: Поля, которые `_fact_slopes` читает у грани кадра, пробега и прямой: ровно они (и ключи контуров, наклоны точек, перекладины, номер региона) образуют ключ `_fact_key` (тест сверяет с исходником).
FACT_KEY_READS = {"frame_face": ("line", "run", "is_fan"), "run": ("run_id", "covector", "inverse_length"), "line": ("q", "a", "b")}
_MEMO_LOCK = threading.Lock()


def parametric_enabled() -> bool:
    return os.environ.get(ENVIRONMENT_SWITCH, "1").strip().lower() not in ("0", "off", "false", "no")


def parametric_memo_enabled() -> bool:
    return os.environ.get(MEMO_SWITCH, "1").strip().lower() not in ("0", "off", "false", "no")


class ParametricRefusal(Exception):
    """Домен не получает сертификата: имя причины лежит в `name` (`PARAM_*`)."""

    def __init__(self, name: str, detail: str = "") -> None:
        super().__init__(f"{name}: {detail}" if detail else name)
        self.name = name
        self.detail = detail


@dataclass(frozen=True, slots=True)
class ParametricRequestV1:
    """Что нужно сертификату от шага ширины: шаблоны покрытия разбиений (`id(разбиение) -> CoverageTemplateV1`) и режим.

    `exact` - тестовый режим: сертификат несёт ещё и ТОЧНЫЕ величины (`ExactParametricV1`), по которым теорема «значение + наклон * сдвиг = точный прогон на сдвинутой ширине»
    проверяется равенством `SqrtSumV1`, а не допуском. Продукт его не заказывает (цена: подъём каждой вершины в точном виде).
    """

    templates: dict
    exact: bool = False


@dataclass(frozen=True, slots=True, eq=False)
class ExactParametricV1:
    """Точные величины сертификата (только режим `exact`): те же вершины и петли, что у float-массивов, в том же порядке.

    `points0`/`point_slopes` - точки карты (единицы решётки) и их наклоны по alpha решётки; `positions0`/`position_slopes` - точные подъёмы точек
    (метры, БЕЗ позиции хоста `src:` и переноса узлов - константы, не зависящие от ширины) и наклоны по alpha в метрах; `loops0` - `(s, r)` каждой петли UV
    (единицы решётки) и `loop_slopes` - их наклоны по alpha решётки; `loop_ids` - `(кольцо грани с наименьшего ключа, ключ вершины)` каждой петли: порядок граней батча от ширины к
    ширине может меняться при той же подписи структуры (она сортирует кольца), и две материализации сравниваются по этим именам, а не по номеру петли.
    """

    vertex_keys: tuple
    points0: tuple
    point_slopes: tuple
    positions0: tuple
    position_slopes: tuple
    loops0: tuple
    loop_slopes: tuple
    loop_ids: tuple
    scale: int


@dataclass(frozen=True, slots=True, eq=False)
class ParametricCertificateV1:
    """Сертификат домена при `alpha`: float-наклоны в порядке вида домена (`envelope_production_view`) и названные причины отказа.

    `position_slopes` - `d позиция / d alpha` (метры на метр), по три на вершину в порядке ключей батча (по `vert_key`); `loop_slopes` - по два на петлю UV: `d(u * alpha) / d alpha`
    и `d(v * alpha) / d alpha` (безразмерные), в порядке граней батча и их `uv_facts`. Позиция при `alpha + d`: `позиция + наклон * d`; `u` при `alpha + d`: `(u * alpha + наклон * d) / (alpha + d)`.
    `markers` пусто - батч домена аффинен (внутри заверенного интервала и той же структуры); иначе перечень `PARAM_*`, и массивы пусты. `approximate` - названные причины, по которым МЕШ хоста
    аффинен лишь приближённо при аффинном батче (развёртка: смещение по нормалям вершин, `PARAM_NON_AFFINE_DEVELOPABLE_OFFSET`); числа в этом случае есть.
    """

    law: str
    alpha: Fraction
    scale: int
    markers: tuple
    detail: str
    vertex_count: int
    loop_count: int
    position_slopes: object
    loop_slopes: object
    #: Числа сборки (для записи и замеров): узлы резки по трём значениям, вершины с ненулевым наклоном и т. п.
    counters: tuple = ()
    exact: ExactParametricV1 | None = field(default=None)
    approximate: tuple = ()
    #: Только у домена с `approximate`: наибольшая скорость поворота нормали смещения по ширине (`|d n / d alpha|`, 1/м) среди движущихся вершин, первый порядок; ноль - нормали не движутся.
    approximate_rate: float = 0.0
    #: Как получены наклоны (`MEMO_LABELS`, PARAMETRIC_SLOPE_MEMO_V1): метка запуска, как `clip_memo`, а не ответ - числа и причины от неё не зависят.
    memo: str = ""

    @property
    def is_affine(self) -> bool:
        return not self.markers and not self.approximate


def unavailable(alpha: Fraction, scale: int, name: str, detail: str = "") -> ParametricCertificateV1:
    """Сертификат без чисел: домен остаётся на последней точной геометрии, причина названа."""

    return ParametricCertificateV1(PARAMETRIC_LAW, Fraction(alpha), int(scale), (name,), detail, 0, 0, array("d"), array("d"))


# --------------------------------------------------------------------------
# Наклоны точек покрытия
# --------------------------------------------------------------------------


def _point_identity(point) -> tuple:
    """Тождество точки покрытия по канонической записи координат (как `coalesce.point_key`), но из целых: хеш `Fraction` на каждой вершине стоил бы заметной доли цены сертификата."""

    return tuple(tuple((radicand, coefficient.numerator, coefficient.denominator) for radicand, coefficient in coordinate.terms) for coordinate in point)


def point_slopes_of(prepared, coverage, templates: dict) -> dict:
    """`{тождество точки: (наклон x, наклон y)}` движущихся точек покрытия домена (наклон по alpha РЕШЁТКИ, точный); неподвижные точки в таблицу не входят.

    Точка отсечения грани - `A + alpha * B` (`CutPointV1`); её точка при alpha0 - ключ. Две разные точки отсечения с одним ключом и разными наклонами совпали бы лишь при alpha0 (событие
    покрытия), а не на окне: это `PARAM_POINT_SLOPE_AMBIGUOUS`.
    """

    partitions = {item.region_id: item.partition for item in prepared.regions}
    table: dict = {}
    for region in coverage.regions:
        contours = getattr(region, "contours", None)
        if contours is None:
            raise ParametricRefusal(PARAM_CONTOURS_UNAVAILABLE, f"region {region.region_id}")
        partition = partitions.get(region.region_id)
        template = None if partition is None else templates.get(id(partition))
        if template is None or len(template.faces) != len(contours):
            raise ParametricRefusal(PARAM_TEMPLATE_UNAVAILABLE, f"region {region.region_id}")
        for pattern, covered in zip(template.faces, contours):
            if pattern.kind != KIND_CUT:
                continue
            if len(pattern.items) != len(covered.points):
                raise ParametricRefusal(PARAM_TEMPLATE_UNAVAILABLE, f"region {region.region_id}: the template and the contour disagree")
            for item, point in zip(pattern.items, covered.points):
                if type(item) is int:
                    continue
                key = _point_identity(point)
                slope = (item.slope_x, item.slope_y)
                known = table.get(key)
                if known is not None and known != slope:
                    raise ParametricRefusal(PARAM_POINT_SLOPE_AMBIGUOUS, f"region {region.region_id}")
                table[key] = slope
    return table


# --------------------------------------------------------------------------
# Подъём: наклон позиции
# --------------------------------------------------------------------------


def _plane_slope(plane: PlaneLiftV1, slope) -> tuple:
    """Наклон позиции подъёма на плоскость (метры на метр alpha): `a * x1 + b * y1` по осям; масштаб решётки сокращается (`alpha решётки = alpha * scale`)."""

    x1, y1 = slope
    return tuple(x1.scaled(plane.basis_a[axis]) + y1.scaled(plane.basis_b[axis]) for axis in range(3))


def _edge_slope(start, end, slope) -> SqrtSumV1:
    """Наклон ориентации `(end - start) x (point - start)` по точке с наклоном `slope`: `dx * y1 - dy * x1`."""

    dx, dy = end[0] - start[0], end[1] - start[1]
    x1, y1 = slope
    return y1.scaled(dx) - x1.scaled(dy)


def _triangle_slope(triangle, slope, scale: int) -> tuple:
    """Наклон позиции подъёма в треугольнике источника (метры на метр alpha): `scale * sum(e1_k * corner_k) / twice_area` по осям."""

    weights = [_edge_slope(triangle.chart[index], triangle.chart[(index + 1) % 3], slope) for index in range(3)]
    ordered = (weights[1], weights[2], weights[0])
    divisor = triangle.twice_area
    factor = Fraction(scale)
    return tuple(
        sum(
            (item.scaled(corner[axis] * factor / divisor) for item, corner in zip(ordered, triangle.corners)),
            _ZERO,
        )
        for axis in range(3)
    )


def _normal_rate(plane, triangle, point, slope, scale: int) -> float:
    """Скорость поворота нормали смещения вершины по ширине, `|d n / d alpha|` (1/м), первый порядок: нормаль - нормированная смесь `N = sum w_k n_k` трёх нормалей углов треугольника (закон
    `SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`, веса `w_k = e_k / twice_area` те же, что в `lift_known`), веса аффинны по ширине, поэтому `N' = sum w'_k n_k`, а `n' = (N' - n (n . N')) / |N|`.

    Это float-оценка для ПРЕДЕЛА ошибки кадра хоста (`PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1`), а не часть ответа: ответов она не решает. Нулевая смесь - бесконечная скорость (ядро такую нормаль отказывает).
    """

    if not triangle.normals:
        return 0.0
    divisor = float(triangle.twice_area)
    values = plane.values_in(triangle, point)
    slopes = [_edge_slope(triangle.chart[index], triangle.chart[(index + 1) % 3], slope) for index in range(3)]
    weights = [_float(values[index]) / divisor for index in (1, 2, 0)]
    rates = [_float(slopes[index]) * scale / divisor for index in (1, 2, 0)]
    mixed = [sum(w * n[axis] for w, n in zip(weights, triangle.normals)) for axis in range(3)]
    moved = [sum(r * n[axis] for r, n in zip(rates, triangle.normals)) for axis in range(3)]
    length = math.sqrt(sum(item * item for item in mixed))
    if not length > 0.0:
        return math.inf
    unit = [item / length for item in mixed]
    along = sum(a * b for a, b in zip(unit, moved))
    return math.sqrt(sum((m - u * along) ** 2 for m, u in zip(moved, unit))) / length


def _leaves_edge(triangle, point, slope) -> bool:
    """Точка лежит на ребре треугольника и уходит с него: подъём - кусочно-аффинный с изломом, и окна у него нет."""

    for index in range(3):
        if _edge_slope(triangle.chart[index], triangle.chart[(index + 1) % 3], slope).is_zero:
            continue
        start, end = triangle.chart[index], triangle.chart[(index + 1) % 3]
        dx, dy = end[0] - start[0], end[1] - start[1]
        value = point[1].scaled(dx) - point[0].scaled(dy) + SqrtSumV1.rational(dy * start[0] - dx * start[1])
        if value.is_zero:
            return True
    return False


def _float(value: SqrtSumV1) -> float:
    if not value.terms:
        return 0.0
    numerator, denominator = enclosure_midpoint(value, ENCLOSURE_BITS)
    return numerator / denominator


# --------------------------------------------------------------------------
# Факты (s, r)
# --------------------------------------------------------------------------


def _fact_slopes(frame_faces, cycles, layout, key_slope, rungs, budget) -> dict:
    """`{(регион, ключ): (наклон s, наклон r)}` фактов до резки: собственные формулы граней от наклонов точек (`key_slope`: ключ вершины -> наклон либо `None`); перекладина JOIN - станция вершины цепи (константа) и общий `r`."""

    roots: dict = {}
    answers: dict = {}
    stations: dict = {}
    transverses: dict = {}
    for frame_face, cycle in zip(frame_faces, cycles):
        region = layout.region_of(frame_face)
        line = frame_face.line
        root = roots.get(line.q)
        if root is None:
            root = roots[line.q] = transverse_root(line, budget)
        run = frame_face.run
        # Формула грани - пробег и прямая; ключ по тождеству объектов (равные формулы разных объектов дадут равные наклоны и сойдутся при сравнении): дроби в ключе хешировались бы на каждой вершине.
        formula = (run.run_id, frame_face.is_fan, id(line))
        for key, _point in cycle:
            slot = (region, key)
            given = answers.setdefault(slot, {})
            if formula in given:
                continue
            slope = key_slope.get(key)
            if slope is None:
                given[formula] = _ZERO_PAIR
                continue
            x1, y1 = slope
            if frame_face.is_fan:
                station = _ZERO
            else:
                station = stations.get((run.run_id, key))
                if station is None:
                    station = stations[(run.run_id, key)] = (x1.scaled(run.covector[0]) + y1.scaled(run.covector[1])) * run.inverse_length
            transverse = transverses.get((id(line), key))
            if transverse is None:
                transverse = transverses[(id(line), key)] = (x1.scaled(line.a) + y1.scaled(line.b)) * root
            given[formula] = (station, transverse)
    found: dict = {}
    for slot, given in answers.items():
        values = list(given.values())
        if slot in rungs:
            # Перекладина JOIN: станция - константа таблицы; `r` общий у обоих ответов; два пробега стоят по обе стороны биссектрисы (`s_a + s_b = 2 s_v` для всех alpha).
            if any(item[1] != values[0][1] for item in values):
                raise ParametricRefusal(PARAM_IDENTITY_AT_POINT, f"rung vertex {slot[1]} in region {slot[0]}: r slopes differ")
            if len(values) == 2 and not (values[0][0] + values[1][0]).is_zero:
                raise ParametricRefusal(PARAM_IDENTITY_AT_POINT, f"rung vertex {slot[1]} in region {slot[0]}: the station sum moves")
            found[slot] = (_ZERO, values[0][1])
            continue
        first = values[0]
        if any(item != first for item in values[1:]):
            raise ParametricRefusal(PARAM_IDENTITY_AT_POINT, f"vertex {slot[1]} in region {slot[0]}: equal answers with different slopes")
        found[slot] = first
    return found


# --------------------------------------------------------------------------
# Память наклонов (PARAMETRIC_SLOPE_MEMO_V1)
# --------------------------------------------------------------------------


def _value_identity(value: SqrtSumV1) -> tuple:
    """Тождество точной величины по канонической записи (она у `SqrtSumV1` единственна): равные тождества - равные величины."""

    return tuple((radicand, coefficient.numerator, coefficient.denominator) for radicand, coefficient in value.terms)


def _fact_key(frame_faces, cycles, layout, key_slope, rungs) -> tuple:
    """`(ключ, номера регионов)`: ВСЕ входы `_fact_slopes`, кроме бюджета (неограниченный, ответа не решает), и перевод канонических номеров регионов в номера этого вызова.

    По грани - регион, пробег (имя, ковектор, `1/|d|`), веер, номер прямой в порядке первой встречи (тождество прямой решает, какие формулы `_fact_slopes` считает одной) и
    ключи контура; коэффициенты прямых; наклоны движущихся точек; перекладины. Регион входит КАНОНИЧЕСКИМ номером (порядок первой встречи в гранях): номера `region:N`
    выводятся из имён экземпляров, а те - из ширины, и тот же регион на другой ширине получает другой номер. `_fact_slopes` от номеров зависит лишь как от меток (ключ
    слота, членство в перекладинах; порядок обхода - порядок граней), поэтому её ответ при перенумерации - тот же ответ с перенумерованными слотами (кроме текста отказа:
    отказы память не держит).
    """

    lines: dict = {}
    regions: dict = {}
    coefficients = []
    faces = []
    for frame_face, cycle in zip(frame_faces, cycles):
        line, run = frame_face.line, frame_face.run
        number = lines.get(id(line))
        if number is None:
            number = lines[id(line)] = len(coefficients)
            coefficients.append((line.q, line.a, line.b))
        region = regions.setdefault(layout.region_of(frame_face), len(regions))
        faces.append((region, run.run_id, run.covector, _value_identity(run.inverse_length), frame_face.is_fan, number, tuple(key for key, _point in cycle)))
    moving = tuple(sorted((key, _point_identity(slope)) for key, slope in key_slope.items() if slope is not None))
    canonical_rungs = frozenset((regions[region], key) if region in regions else ("raw", region, key) for region, key in rungs)
    return (tuple(faces), tuple(coefficients), moving, canonical_rungs), tuple(regions)


def _fact_floats(found: dict) -> dict:
    """`{("region:N", ключ): (float s, float r)}` наклонов фактов: те же округления, что петли UV брали бы по одной."""

    return {(f"region:{slot[0]}", slot[1]): (_float(pair[0]), _float(pair[1])) for slot, pair in found.items()}


class SlopeMemoV1:
    """Наклоны ОДНОЙ подготовки по полным входам их функций; живёт в `MaterializeMemoV1.parametric` и с ней (пикл её не несёт).

    `facts`: ключ `_fact_key` -> `(наклоны, их float)` по КАНОНИЧЕСКИМ номерам регионов. `positions`: содержимое, которое читает функция наклона позиции (`("plane", a, b)` -
    `_plane_slope`; `("triangle", карта, углы, двойная площадь, масштаб)` - `_triangle_slope`) -> `{тождество наклона точки: (точный наклон позиции, его float)}`.
    """

    __slots__ = ("code", "facts", "positions", "size")

    def __init__(self, code: str) -> None:
        self.code = code
        self.facts: dict = {}
        self.positions: dict = {}
        self.size = 0


class _SlopeSource:
    """Наклоны одного сертификата: из памяти подготовки (`memo`) либо счётом (`memo` None: память выключена, точный режим); попадания и промахи дают метку."""

    __slots__ = ("memo", "label", "hits", "misses", "tables")

    def __init__(self, memo: SlopeMemoV1 | None, label: str = "") -> None:
        self.memo, self.label, self.hits, self.misses, self.tables = memo, label, 0, 0, {}

    def final_label(self) -> str:
        if self.label:
            return self.label
        if not self.hits and not self.misses:
            return MEMO_NOT_REACHED
        return MEMO_HIT if not self.misses else MEMO_MISS if not self.hits else MEMO_PARTIAL

    def facts(self, frame_faces, cycles, layout, key_slope, rungs, budget) -> tuple:
        """`(наклоны фактов, их float по "region:N")` либо `ParametricRefusal` - тот же, что дал бы `_fact_slopes` (отказ считается всегда заново: его текст несёт номер региона)."""

        memo = self.memo
        if memo is None:
            found = _fact_slopes(frame_faces, cycles, layout, key_slope, rungs, budget)
            return found, _fact_floats(found)
        key, numbers = _fact_key(frame_faces, cycles, layout, key_slope, rungs)
        entry = memo.facts.get(key)
        if entry is not None:
            self.hits += 1
            canonical, floats = entry
            return ({(numbers[slot[0]], slot[1]): pair for slot, pair in canonical.items()},
                    {(f"region:{numbers[slot[0]]}", slot[1]): pair for slot, pair in floats.items()})
        self.misses += 1
        found = _fact_slopes(frame_faces, cycles, layout, key_slope, rungs, budget)
        floats = _fact_floats(found)
        index = {number: position for position, number in enumerate(numbers)}
        canonical = {(index[slot[0]], slot[1]): pair for slot, pair in found.items()}
        canonical_floats = {(index[slot[0]], slot[1]): floats[(f"region:{slot[0]}", slot[1])] for slot in found}
        with _MEMO_LOCK:
            while len(memo.facts) >= MEMO_FACT_LIMIT:
                del memo.facts[next(iter(memo.facts))]
            memo.facts[key] = (canonical, canonical_floats)
        return found, floats

    def position(self, plane, triangle, name, slope, scale: int) -> tuple:
        """`(точный наклон позиции, его float)`: `_plane_slope` (`triangle` None) либо `_triangle_slope`; таблица по содержимому плоскости либо треугольника - одна на вызов."""

        memo = self.memo
        if memo is None:
            values = _plane_slope(plane, slope) if triangle is None else _triangle_slope(triangle, slope, scale)
            return values, tuple(_float(item) for item in values)
        table = self.tables.get(name)
        if table is None:
            content = ("plane", plane.basis_a, plane.basis_b) if triangle is None else ("triangle", triangle.chart, triangle.corners, triangle.twice_area, scale)
            with _MEMO_LOCK:
                table = memo.positions.get(content)
                if table is None:
                    if memo.size >= MEMO_POSITION_LIMIT:
                        memo.positions.clear()
                        memo.size = 0
                    table = memo.positions[content] = {}
            self.tables[name] = table
        identity = _point_identity(slope)
        found = table.get(identity)
        if found is not None:
            self.hits += 1
            return found
        self.misses += 1
        values = _plane_slope(plane, slope) if triangle is None else _triangle_slope(triangle, slope, scale)
        found = (values, tuple(_float(item) for item in values))
        with _MEMO_LOCK:
            table[identity] = found
            memo.size += 1
        return found


def _slope_source(prepared, request) -> _SlopeSource:
    """Источник наклонов сертификата: память подготовки воркера, если она есть и включена, и режим не точный."""

    if request.exact:
        return _SlopeSource(None, MEMO_BYPASS_EXACT)
    if not parametric_memo_enabled() or not memo_enabled():
        return _SlopeSource(None, MEMO_OFF)
    memo = memo_of(prepared)
    if memo is None:
        return _SlopeSource(None, MEMO_NO_STORE)
    code = kernel_code_identity()
    with _MEMO_LOCK:
        store = memo.parametric
        if store is None or store.code != code:
            store = memo.parametric = SlopeMemoV1(code)
    return _SlopeSource(store)


# --------------------------------------------------------------------------
# Боковые точки резки
# --------------------------------------------------------------------------


def _step_for(alpha: Fraction, interval) -> Fraction | None:
    """Шаг боковых точек (метры, степень двойки): не больше `1 / _STEP_FRACTION` расстояния до ближайшего края интервала; `None` - интервала нет."""

    if interval is None or interval.status != "CERTIFIED":
        return None
    reach = alpha - Fraction(interval.low)
    if interval.high is not None:
        reach = min(reach, Fraction(interval.high) - alpha)
    if reach <= 0:
        return None
    step = Fraction(1)
    limit = reach / _STEP_FRACTION
    while step > limit:
        step /= 2
    while step * 2 <= limit:
        step *= 2
    return step


def _shape(clipped) -> tuple:
    def keys(lists):
        return None if lists is None else tuple(tuple(key for key, _point in item) for item in lists)

    return (
        tuple(tuple(tuple(polygon) for polygon in face) for face in clipped.polygons),
        keys(clipped.cycles),
        keys(clipped.vertex_lists),
        keys(clipped.extra_lists),
        tuple(clipped.points),
        tuple(clipped.snapped),
    )


def _shifted(point, slope, delta: Fraction):
    return point if slope is None else (point[0] + slope[0].scaled(delta), point[1] + slope[1].scaled(delta))


def _side_cut(inputs, delta: Fraction, key_slope, fact_slope_of):
    """`(резка, факты)` при alpha решётки, сдвинутой на `delta` (метры): стадия резки по сдвинутым точкам и фактам, мимо памяти резки."""

    shift = delta * inputs.scale
    points = {key: _shifted(point, key_slope.get(key), shift) for key, point in inputs.points_before.items()}
    cycles = [[(key, points[key]) for key, _point in cycle] for cycle in inputs.cycles]
    facts = {}
    for slot, (s, r) in inputs.facts_before.items():
        slope = fact_slope_of(slot)
        facts[slot] = (s, r) if slope is None else (s + slope[0].scaled(shift), r + slope[1].scaled(shift))
    budget = unlimited_reference_budget(stage="PARAMETRIC", domain_id="")
    with _SIDE_CUT_LOCK, clip_memo_disabled():
        clipped = cut_domain(
            inputs.plane,
            budget,
            frame_faces=inputs.frame_faces,
            cycles=cycles,
            points=points,
            polygons=inputs.polygons,
            facts=facts,
            layout=inputs.layout,
            table=inputs.table,
            lattice_alpha=inputs.lattice_alpha + shift,
            law=inputs.law,
            by_faces=inputs.by_faces,
            tally=Counter(),
            inert=inputs.inert,
        )
    return clipped, facts


def _second_difference_slope(low, middle, high, delta_lattice: Fraction, name: str):
    """Наклон по трём равноотстоящим значениям `low`, `middle`, `high` (точные `SqrtSumV1`) либо отказ: вторая разность обязана быть нулём ТОЧНО."""

    up, down = high - middle, middle - low
    if up != down:
        raise ParametricRefusal(name)
    return up.scaled(1 / delta_lattice)


def _clip_slopes(inputs, interval, key_slope, fact_slope_of):
    """`(наклоны точек узлов резки, наклоны фактов, дописанных резкой)` по трём точным значениям стадии резки; отказ - `ParametricRefusal`."""

    step = _step_for(inputs.alpha, interval)
    if step is None:
        raise ParametricRefusal(PARAM_WINDOW_NOT_CERTIFIED)
    base_shape = _shape(inputs.cut)
    last = None
    for _attempt in range(_STEP_ATTEMPTS):
        try:
            up, facts_up = _side_cut(inputs, step, key_slope, fact_slope_of)
            down, facts_down = _side_cut(inputs, -step, key_slope, fact_slope_of)
        except MaterializationRefusal as refusal:
            last = ParametricRefusal(PARAM_CLIP_REFUSED, refusal.outcome.value)
            step /= 2
            continue
        if _shape(up) != base_shape or _shape(down) != base_shape:
            last = ParametricRefusal(PARAM_CLIP_STRUCTURE_CHANGED)
            step /= 2
            continue
        last = None
        break
    if last is not None:
        raise last
    delta = step * inputs.scale
    point_slopes: dict = {}
    for source, source_up, source_down in (
        (inputs.cut.points, up.points, down.points),
        (inputs.cut.snapped, up.snapped, down.snapped),
    ):
        for key, point in source.items():
            axes = tuple(
                _second_difference_slope(source_down[key][axis], point[axis], source_up[key][axis], delta, PARAM_CLIP_NODE)
                for axis in range(2)
            )
            if axes != _ZERO_PAIR:
                point_slopes[key] = axes
    fact_slopes: dict = {}
    for slot, (s, r) in inputs.facts_after.items():
        if slot in inputs.facts_before:
            continue
        up_fact, down_fact = facts_up[slot], facts_down[slot]
        fact_slopes[slot] = tuple(
            _second_difference_slope(down_fact[axis], (s, r)[axis], up_fact[axis], delta, PARAM_CLIP_FACT) for axis in range(2)
        )
    return point_slopes, fact_slopes


# --------------------------------------------------------------------------
# Сборка
# --------------------------------------------------------------------------


@dataclass(slots=True)
class ParametricInputsV1:
    """Всё, что сборка домена (`domain._assemble`) оставляет сертификату: ссылки на объекты стадий, ничего не посчитано."""

    plane: object
    frame_faces: list
    cycles: list
    points_before: dict
    points_after: dict
    polygons: list
    facts_before: dict
    facts_after: dict
    rungs: set
    layout: object
    table: object
    lattice_alpha: Fraction
    alpha: Fraction
    scale: int
    law: object
    by_faces: bool
    inert: frozenset
    cut: object
    lifted_names: dict


def capture_inputs(stage, points, facts, scalars, cut, lifted_names) -> ParametricInputsV1:
    """`ParametricInputsV1` из объектов сборки (`domain._assemble`): ссылки, ничего не считается.

    `stage` - `(подъём, слитые грани, контуры до резки, многоугольники до резки, раскладка, таблица станций, перекладины)`; `points`, `facts` - `(до резки, после)`;
    `scalars` - `(alpha метры, alpha решётки, закон топологии, резка по граням, инертные пары)`.
    """

    plane, frame_faces, cycles, polygons, layout, table, rungs = stage
    alpha, lattice_alpha, law, by_faces, inert = scalars
    return ParametricInputsV1(
        plane=plane,
        frame_faces=frame_faces,
        cycles=cycles,
        points_before=points[0],
        points_after=points[1],
        polygons=polygons,
        facts_before=facts[0],
        facts_after=facts[1],
        rungs=rungs,
        layout=layout,
        table=table,
        lattice_alpha=lattice_alpha,
        alpha=alpha,
        scale=int(table.scale),
        law=law,
        by_faces=by_faces,
        inert=inert,
        cut=cut,
        lifted_names=lifted_names,
    )


def _ring_of(face) -> tuple:
    """Кольцо ключей грани, начатое с наименьшего: имя грани, не зависящее от порядка граней батча и номеров регионов."""

    keys = tuple(item.value for item in face.ordered_vert_keys)
    start = keys.index(min(keys))
    return keys[start:] + keys[:start]


def _sorted_vertices(batch):
    return sorted(batch.vertices, key=lambda item: item.vert_key.value)


def certificate_of(inputs: ParametricInputsV1, batch, interval, request: ParametricRequestV1, developable: bool, prepared, coverage) -> ParametricCertificateV1:
    """Сертификат материализованного домена; причина отказа - в `markers`, исключения не выходят."""

    alpha, scale = inputs.alpha, inputs.scale
    source = _slope_source(prepared, request)
    try:
        if interval is None or interval.status != "CERTIFIED":
            raise ParametricRefusal(PARAM_WINDOW_NOT_CERTIFIED, "" if interval is None else f"{interval.status} {interval.reason}".strip())
        found = _certificate(inputs, batch, interval, request, prepared, coverage, developable, source)
        # У домена-развёртки позиции до смещения аффинны, но смещение идёт по нормали вершины (не рациональной): числа есть, домен назван в `approximate`.
        return replace(found, approximate=(PARAM_DEVELOPABLE_OFFSET,), memo=source.final_label()) if developable else replace(found, memo=source.final_label())
    except ParametricRefusal as refusal:
        return replace(unavailable(alpha, scale, refusal.name, refusal.detail), memo=source.final_label())


def _certificate(inputs, batch, interval, request, prepared, coverage, developable: bool = False, source: _SlopeSource | None = None) -> ParametricCertificateV1:
    scale = inputs.scale
    normal_rate = 0.0
    source = _SlopeSource(None) if source is None else source
    budget = unlimited_reference_budget(stage="PARAMETRIC", domain_id="")
    moving = point_slopes_of(prepared, coverage, request.templates)
    # Точки до резки: наклоны покрытия по ключу вершины; факты до резки: формулы граней (из памяти наклонов, PARAMETRIC_SLOPE_MEMO_V1).
    key_slope = {key: moving.get(_point_identity(point)) for key, point in inputs.points_before.items()} if moving else {}
    fact_slope, fact_floats = source.facts(inputs.frame_faces, inputs.cycles, inputs.layout, key_slope, inputs.rungs, budget)
    clip_points: dict = {}
    clip_facts: dict = {}
    if inputs.cut is not None:
        clip_points, clip_facts = _clip_slopes(inputs, interval, key_slope, lambda slot: _nonzero(fact_slope.get(slot)))

    def point_slope(key):
        found = clip_points.get(key)
        return found if found is not None else key_slope.get(key)

    plane = inputs.plane
    triangles = None if isinstance(plane, PlaneLiftV1) else {item.name: item for item in plane.triangles}
    vertices = _sorted_vertices(batch)
    position_slopes = array("d")
    exact_points, exact_point_slopes, exact_positions, exact_position_slopes = [], [], [], []
    moving_vertices = 0
    for vertex in vertices:
        key = vertex.vert_key.value
        point = inputs.points_after[key]
        slope = point_slope(key)
        if slope is None:
            position_slopes.extend(_FLOAT_ZERO3)
            positions_exact = _ZERO_TRIPLE
        else:
            moving_vertices += 1
            if triangles is None:
                values, floats = source.position(plane, None, None, slope, scale)
            else:
                name = inputs.lifted_names.get(key, (None, None))[0]
                triangle = triangles.get(name)
                if triangle is None:
                    raise ParametricRefusal(PARAM_LIFT_UNSUPPORTED, f"vertex {key}: no source triangle")
                if _leaves_edge(triangle, point, slope):
                    raise ParametricRefusal(PARAM_LIFT_ON_EDGE, f"vertex {key}")
                values, floats = source.position(plane, triangle, name, slope, scale)
                if developable:
                    normal_rate = max(normal_rate, _normal_rate(plane, triangle, point, slope, scale))
            position_slopes.extend(floats)
            positions_exact = values
        if request.exact:
            exact_points.append(point)
            exact_point_slopes.append(slope if slope is not None else _ZERO_PAIR)
            exact_position_slopes.append(positions_exact)
            exact_positions.append(_exact_lift(plane, triangles, inputs.lifted_names.get(key, (None, None))[0], point))
    # Петли UV: float наклонов фактов из памяти (те же `_float`), факты резки - поверх, как в точной таблице.
    region_floats = {**fact_floats, **_fact_floats(clip_facts)} if clip_facts else fact_floats
    region_slopes: dict = {}
    if request.exact:
        for slot, found in (*fact_slope.items(), *clip_facts.items()):
            region_slopes[(f"region:{slot[0]}", slot[1])] = found
    loops = array("d")
    exact_loops, exact_loop_slopes, exact_loop_ids = [], [], []
    count = 0
    facts = inputs.facts_after
    for face in batch.faces:
        region = face.semantic_region_id.value
        number = int(region.split(":", 1)[1])
        ring = _ring_of(face) if request.exact else None
        for fact in face.uv_facts:
            pair = region_floats.get((region, fact.vert_key.value))
            if pair is None:
                raise ParametricRefusal(PARAM_FACT_UNRESOLVED, f"{region}/{fact.vert_key.value}")
            loops.append(pair[0])
            loops.append(pair[1])
            count += 1
            if request.exact:
                exact_loops.append(facts[(number, fact.vert_key.value)])
                exact_loop_slopes.append(region_slopes[(region, fact.vert_key.value)])
                exact_loop_ids.append((ring, fact.vert_key.value))
    exact = None
    if request.exact:
        exact = ExactParametricV1(
            tuple(item.vert_key.value for item in vertices),
            tuple(exact_points),
            tuple(exact_point_slopes),
            tuple(exact_positions),
            tuple(exact_position_slopes),
            tuple(exact_loops),
            tuple(exact_loop_slopes),
            tuple(exact_loop_ids),
            scale,
        )
    return ParametricCertificateV1(
        PARAMETRIC_LAW,
        inputs.alpha,
        scale,
        (),
        "",
        len(vertices),
        count,
        position_slopes,
        loops,
        (("PARAMETRIC_MOVING_VERTICES", moving_vertices), ("PARAMETRIC_CLIP_NODE_SLOPES", len(clip_points)), ("PARAMETRIC_CLIP_FACT_SLOPES", len(clip_facts))),
        exact,
        approximate_rate=normal_rate,
    )


_ZERO_TRIPLE = (_ZERO, _ZERO, _ZERO)


def _nonzero(pair):
    return None if pair is None or pair == _ZERO_PAIR else pair


def _exact_lift(plane, triangles, name, point):
    """Точный подъём точки карты (метры, `SqrtSumV1` по осям) без позиции хоста: плоскость либо найденный треугольник."""

    if triangles is None:
        return plane.lift_exact(point)
    triangle = triangles[name]
    return plane.lift_in(triangle, plane.values_in(triangle, point))
