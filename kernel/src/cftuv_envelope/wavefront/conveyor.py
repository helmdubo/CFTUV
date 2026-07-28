"""Публичный вход очереди: снапшот и запрос — в покрытие с владельцами.

Рецепт входа очереди до этого модуля существовал только приватным хелпером
теста (`_field_bridge_input` в `test_wavefront_differential.py`) и проходил
через `evaluate_reference_raw_coverage`, то есть платил за СОЮЗ, который
очереди не нужен вовсе. Здесь тот же рецепт становится входом ядра и теряет
союз; всё остальное — те же функции, что и были.

ЦЕНА, ИЗМЕРЕННАЯ НА `building_002_point_contact_v1` (3 источника, 12 рёбер):

| ступень | было | стало |
|---|---:|---:|
| `evaluate_reference_raw_coverage` целиком | 171 мс | не вызывается |
| в т.ч. `DOMAIN_CLIP` | 69 мс | не вызывается |
| в т.ч. `RAW_UNION` | 50 мс | не вызывается |
| `compile_arrival_models` (активность юбок) | 56 мс | не вызывается |
| перевод sympy → `Fraction` (24 координаты + 12 полей закона) | 304 мс | 0.1 мс |
| `prepare` целиком | — | 59 мс |
| `coverage` на тёплой подготовке | — | 9 мс |

ГДЕ ПРОХОДИТ РАЗРЕЗ И ПОЧЕМУ ИМЕННО ТАМ. Закон прихода Strip есть ковектор
нормали опоры и его константа (`strip_front_support_line`) — обе величины
выводятся из `GeometryContext` и спеки, и ни юбки, ни её клипа о домен, ни
союза в этот вывод не входит. Это ПРОВЕРЕНО, а не предположено: множество
законов, собранных здесь, совпадает с множеством законов
`compile_arrival_models` на фикстуре поточечно (три закона, те же дроби).

Разница между путями при этом есть, и она названа, а не спрятана. Полный путь
эмитит закон только если у юбки нашёлся сегмент НА ФРОНТЕ при эффективной
alpha; здесь такого отбора нет — берутся все опорные сегменты всех Strip-спек.
Лишний закон не исчезает молча: мост отвечает `ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE`
или `MORE_ARRIVAL_LAWS_THAN_EDGES_ON_ONE_LINE`, то есть отказ приходит с
именем и числом. На фикстуре отбор не отбрасывает ничего (по одному опорному
сегменту на каждую из трёх Strip-спек), и оба пути дают три закона.

ALPHA ВХОДИТ РОВНО В ДВУХ МЕСТАХ, и оба измерены:

1. усечение граней по времени (`coverage_at`) — то, ради чего разрез и сделан;
2. ИМЯ экземпляра юбки. `strip_envelope_instance_id` стоит на ЭФФЕКТИВНОЙ
   alpha, поэтому при alpha 0.25 и 0.5 три имени различны все три. Гипотеза
   «alpha входит только в `coverage_at`» этим ОПРОВЕРГНУТА, и опровержение
   стоило одной ступени: имена экземпляров выводятся в alpha-зависимой части,
   а alpha-независимая знает владельца по `envelope_spec_id`.

Геометрия же от alpha не зависит ни в чём: петли домена, законы, решётка карты,
мост, скелет и разбиение при alpha 0.25 и 0.5 совпадают ПОБИТОВО, включая
`semantic_digest` скелета. Поэтому `prepare` кэшируем, а `coverage` — нет.

ЧЕГО ЗДЕСЬ НЕТ. `resolve_coverage_interactions` и `policy_b` не вызываются:
попарный крой на этом же домене теряет ВСЮ площадь покрытия
(`test_wavefront_differential.py`), и очередь существует именно вместо него.
`exact_union` не вызывается ни разу — ни один из четырёх бэкендов
`ExactSegmentArrangementBackend` в этот путь не входит.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from decimal import Decimal
from enum import Enum
from fractions import Fraction
from math import gcd

import sympy as sp

from ..contracts.envelopes import AngularEnvelopeSpec, StripEnvelopeSpec
from ..contracts.analysis import AnalysisSnapshotV1
from ..contracts.request import DecalRequestV1
from ..ids import PatchDomainId
from ..interactions.arrival import (
    ANGULAR_PROFILE_NORMAL_SPEED,
    STRIP_FRONT_NORMAL_SPEED,
    angular_hidden_support_lines,
    strip_front_support_line,
)
from ..numeric import LocalLengthV1
from ..reference.boundary import (
    build_domain_geometry,
    resolve_component_alphas,
)
from ..reference.common import GeometryContext, ReferenceGeometryError
from ..reference.compile import compile_reference_envelopes
from ..reference.metric import ExactPlanarMetric
from ..reference.planar_types import (
    CertifiedPredicateUndecidable,
    exact_sign,
)
from ..reference.raw_coverage import (
    normalize_requested_alpha,
    spec_effective_alpha,
)
from ..reference.strip import strip_envelope_instance_id
from ..reference.validation import validate_reference_geometry_payload
from ..robust.grid import GridSpecV1
from ..source_grid import chart_grid_for
from .bridge import (
    BridgeOutcome,
    BridgeReportV1,
    PlainArrivalLawV1,
    VertexFanLawV1,
    bridge_arrival_laws,
)
from .coverage import CoverageOutcome, coverage_at
from .faces import EdgeKey, FaceOutcome, FacePartitionV1, build_faces
from .skeleton import SkeletonOutcome, SkeletonV1, build_skeleton
from .sqrt_sum import SqrtSumV1


class ConveyorOutcome(str, Enum):
    """Чем кончился вход очереди. Тихого «не получилось» здесь нет.

    Исход называет СТУПЕНЬ, на которой путь остановился; собственный отказ
    ступени лежит рядом своим типом (`BridgeOutcome`, `SkeletonOutcome`,
    `FaceOutcome`, `CoverageOutcome`) и в строку не сплющивается.
    """

    EXACT = "EXACT"
    # План не скомпилировался — дальше идти не с чем.
    PLAN_IS_NOT_COMPILED = "PLAN_IS_NOT_COMPILED"
    # У домена нет планарного кадра: координат нет, значит нет и петель.
    DOMAIN_FRAME_IS_UNAVAILABLE = "DOMAIN_FRAME_IS_UNAVAILABLE"
    # Геометрия домена построилась, а регионов в ней ноль. Отличается от
    # «регион пуст»: пустой список регионов означает, что резать нечего.
    DOMAIN_HAS_NO_REGIONS = "DOMAIN_HAS_NO_REGIONS"
    # Координата домена не рациональна. Порога здесь быть не может: `PolygonV1`
    # живёт на решётке, а к решётке привязывают ДРОБЬ, не корень.
    DOMAIN_POINT_IS_NOT_RATIONAL = "DOMAIN_POINT_IS_NOT_RATIONAL"
    # Поле закона прихода не рационально. То же самое и по той же причине:
    # `PlainArrivalLawV1` объявлен в точных дробях.
    ARRIVAL_LAW_IS_NOT_RATIONAL = "ARRIVAL_LAW_IS_NOT_RATIONAL"
    # Направление скрытой опоры веера не рационально даже после рескейла.
    # В общем положении так и есть: при `k = 1` биссектриса есть сумма двух
    # единичных нормалей с РАЗНЫМИ радикалами, при `k = 2` направление — корень
    # кубики тройного угла. Рационален веер ровно при равных радикалах опор
    # (перпендикулярные и симметричные углы — весь полевой корпус `building.002`).
    #
    # ЭТО ОТКАЗ, А НЕ ОТКАТ. Посчитать здесь митрованный угол значило бы выдать
    # острый апекс за мягкий: очередь ответила бы числом, которое продукту не
    # соответствует, и отличить его было бы нечем. В `detail` — сколько вершин.
    ANGULAR_PROFILE_DIRECTION_IS_NOT_RATIONAL = (
        "ANGULAR_PROFILE_DIRECTION_IS_NOT_RATIONAL"
    )
    # В плане нет ни одной Strip-спеки, то есть источника фронта нет вовсе.
    NO_STRIP_SOURCE_IN_THE_PLAN = "NO_STRIP_SOURCE_IN_THE_PLAN"
    # У кадра патча нет сертификата решётки, то есть решётку карты выводить не
    # из чего. Подобрать её здесь нельзя: масштаб определяет, какой домен
    # вообще отобразится, и выбранный «на глаз» превратил бы ответ в свойство
    # выбора. Кадром без сертификата бывает `PlanarPatchFrameV1` — путь,
    # объявленный до закона решётки.
    CHART_LATTICE_IS_NOT_DECLARED = "CHART_LATTICE_IS_NOT_DECLARED"
    # Геометрия домена отказала своим исходом (`ReferenceOutcome`), и он лежит
    # в `reference_outcome`, а не пересказан словами.
    DOMAIN_GEOMETRY_REFUSED = "DOMAIN_GEOMETRY_REFUSED"
    BRIDGE_DID_NOT_MAP = "BRIDGE_DID_NOT_MAP"
    SKELETON_DID_NOT_CLOSE = "SKELETON_DID_NOT_CLOSE"
    FACES_DID_NOT_ASSEMBLE = "FACES_DID_NOT_ASSEMBLE"
    # Покрытие спрошено у подготовки, которая сама не `EXACT`.
    PREPARATION_IS_NOT_EXACT = "PREPARATION_IS_NOT_EXACT"
    COVERAGE_DID_NOT_CLOSE = "COVERAGE_DID_NOT_CLOSE"
    # Вектор эффективных alpha у одной огибающей неоднороден. Имя экземпляра
    # стоит на эффективной alpha, поэтому двух alpha ему мало не бывает.
    SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN = (
        "SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN"
    )


Counters = tuple[tuple[str, int], ...]
Timings = tuple[tuple[str, float], ...]


class _Clock:
    """Часы стадий. Ни одна ступень не проходит без своей строки времени."""

    __slots__ = ("_marks",)

    def __init__(self) -> None:
        self._marks: dict[str, float] = {}

    def add(self, stage: str, started: float) -> None:
        self._marks[stage] = self._marks.get(stage, 0.0) + (
            time.perf_counter() - started
        )

    def timings(self) -> Timings:
        return tuple(self._marks.items())


def exact_rational(expression) -> Fraction | None:
    """Точная дробь из выражения SymPy, БЕЗ `nsimplify`.

    `nsimplify` здесь стоил 304 мс на 36 значений фикстуры — по 8.5 мс на
    значение, — и всё это на числах, которые уже были `Rational`: он ИЩЕТ
    рациональное приближение, а искать нечего. Прямое чтение `p/q` даёт те же
    36 значений за 0.1 мс, и совпадение проверено поэлементно, а не по сумме.

    ЦЕНА БЫЛА НЕ ЕДИНСТВЕННОЙ БЕДОЙ, и вторая тяжелее. Прежний рецепт
    `Fraction(*map(int, sp.fraction(sp.nsimplify(x))))` на ИРРАЦИОНАЛЬНОМ
    входе не падает — он молча усекает: `sqrt(2)/2` даёт `(sqrt(2), 2)`, а
    `int(sqrt(2))` есть 1, то есть ответ `1/2` при истинном `0.7071...`,
    ошибка 29 %. Тихого исхода здесь быть не должно, поэтому

    `None` — выражение не рационально. Это ответ, а не сбой: вызывающий обязан
    назвать исход (`DOMAIN_POINT_IS_NOT_RATIONAL`, `ARRIVAL_LAW_IS_NOT_RATIONAL`)
    вместо того, чтобы округлить. Исход достижим не в теории: на треугольнике
    `straight_snapshot` нормаль гипотенузы иррациональна, и вход отвечает
    именем, а прежний хелпер ответил бы числом, которого нет.
    """

    value = sp.sympify(expression)
    if not value.is_Rational:
        return None
    return Fraction(int(value.p), int(value.q))


def chart_lattice_for_frame(frame) -> GridSpecV1 | None:
    """Решётка карты патча — ПО ОБЪЯВЛЕННОМУ ЗАКОНУ, а не подобранная.

    Шага здесь два, и оба уже объявлены ядром. Шаг источника берётся из
    сертификата самого патча (`window_step`, он же самый мелкий допустимый шаг
    окна по `FINEST_ADMISSIBLE_FIRST_V1`), а решётка карты выводится из него
    `chart_grid_for`: координаты карты — кратные базисным векторам, а не метры,
    поэтому шаг переносить напрямую нельзя.

    Свободного числа здесь нет вовсе, и это главное свойство функции.
    «Удобный» масштаб превратил бы «вход отобразился» в утверждение про
    подобранную решётку вместо утверждения про закон, который ядро объявляет
    для всех патчей одинаково.

    `None` — у кадра сертификата решётки нет. Кадр бывает двух видов
    (`validate_reference_geometry_payload` возвращает либо
    `RationalAffinePlanarMetricV2`, либо `PlanarPatchFrameV1`), и закон решётки
    объявлен только у первого. Взять «какой-нибудь» масштаб для второго нельзя:
    масштаб решает, отобразится ли домен вообще.
    """

    certificate = getattr(frame, "grid_certificate", None)
    if certificate is None:
        return None
    step = Fraction(
        certificate.window_step.numerator, certificate.window_step.denominator
    )
    return chart_grid_for(ExactPlanarMetric.from_descriptor(frame).gram, step)


@dataclass(frozen=True, slots=True)
class PreparedRegionV1:
    """Один регион домена, доведённый до разбиения. Отказ — тоже результат."""

    region_id: str
    bridge: BridgeReportV1
    skeleton: SkeletonV1 | None
    partition: FacePartitionV1 | None
    bridge_outcome: BridgeOutcome
    skeleton_outcome: SkeletonOutcome | None
    face_outcome: FaceOutcome | None
    # Владелец каждого ребра-источника — `envelope_spec_id`, имя, от alpha
    # НЕ зависящее. Имя экземпляра (`envelope_instance_id`) выводится ступенью
    # покрытия, потому что зависит.
    owner_by_edge: tuple[tuple[EdgeKey, str], ...]
    # Рёбра домена без закона прихода: у них грани нет, фронт от них не идёт.
    wall_spans: tuple[EdgeKey, ...]
    wall_edge_count: int
    # Рёбра, у которых источник есть, а КОТОРЫЙ именно — по входу не записано.
    ambiguous_owner_spans: tuple[EdgeKey, ...]

    @property
    def is_exact(self) -> bool:
        return (
            self.bridge_outcome is BridgeOutcome.EXACT
            and self.skeleton_outcome is SkeletonOutcome.EXACT
            and self.face_outcome is FaceOutcome.EXACT
        )


@dataclass(frozen=True, slots=True)
class ConveyorPreparationV1:
    """Alpha-НЕЗАВИСИМАЯ часть входа очереди. Её и следует кэшировать.

    Независимость измерена, а не объявлена: при alpha запроса 0.25 и 0.5
    `semantic_digest` скелета, состав граней и `owner_by_edge` совпадают
    побитово. Что от alpha зависит — имена экземпляров — сюда не входит.
    """

    outcome: ConveyorOutcome
    regions: tuple[PreparedRegionV1, ...]
    lattice: GridSpecV1 | None
    law_names: tuple[str, ...]
    counters: Counters
    timings: Timings
    detail: str = ""
    # Внутренние опоры alpha-зависимой ступени. Они здесь не для читателя, а
    # потому что второй раз строить их было бы вторым способом их построить.
    compilation: object | None = None
    context: GeometryContext | None = None
    domain: object | None = None
    requested_alpha: LocalLengthV1 | None = None

    def counter(self, name: str) -> int:
        return dict(self.counters).get(name, 0)

    def timing(self, stage: str) -> float:
        return dict(self.timings).get(stage, 0.0)


@dataclass(frozen=True, slots=True)
class ConveyorFaceCoverageV1:
    """Кусок покрытия и его владелец, названный обоими именами."""

    region_id: str
    owner: EdgeKey
    envelope_spec_id: str
    # Имя экземпляра юбки при ЭТОЙ alpha. `None` — вывести не удалось, и
    # причина лежит в исходе всего результата, а не подменена пустой строкой.
    envelope_instance_id: str | None
    doubled_area: SqrtSumV1


@dataclass(frozen=True, slots=True)
class ConveyorRegionCoverageV1:
    """Покрытие одного региона домена к моменту alpha."""

    region_id: str
    outcome: CoverageOutcome
    faces: tuple[ConveyorFaceCoverageV1, ...]
    doubled_area: SqrtSumV1
    polygon_doubled_area: int
    wall_spans: tuple[EdgeKey, ...]


@dataclass(frozen=True, slots=True)
class ConveyorCoverageV1:
    """Покрытие всего домена плана очередью, с владельцем у каждого куска."""

    outcome: ConveyorOutcome
    alpha: Fraction
    # Та же alpha в единицах привязанной решётки: `alpha * lattice.scale`.
    lattice_alpha: Fraction
    regions: tuple[ConveyorRegionCoverageV1, ...]
    doubled_area: SqrtSumV1
    polygon_doubled_area: int
    preparation: ConveyorPreparationV1
    counters: Counters
    timings: Timings
    detail: str = ""

    @property
    def faces(self) -> tuple[ConveyorFaceCoverageV1, ...]:
        return tuple(face for region in self.regions for face in region.faces)

    def counter(self, name: str) -> int:
        return dict(self.counters).get(name, 0)

    def timing(self, stage: str) -> float:
        return dict(self.timings).get(stage, 0.0)


def _refused(
    outcome: ConveyorOutcome,
    detail: str,
    clock: _Clock,
    counters: Counters = (),
) -> ConveyorPreparationV1:
    """Отказ подготовки, несущий то, что успел узнать.

    `counters` не для симметрии. Пере-масштабирование закона происходит ДО
    решётки и до моста, поэтому отказ на любой из поздних ступеней иначе унёс
    бы вместе с собой единственное свидетельство того, что законы вообще
    пришлось переписывать, — а «законы были рациональны» и «законы спасены
    масштабом, а споткнулись мы дальше» лечатся по-разному.
    """

    return ConveyorPreparationV1(
        outcome=outcome,
        regions=(),
        lattice=None,
        law_names=(),
        counters=counters,
        timings=clock.timings(),
        detail=detail,
    )


def _positive_common_factor(normal_x, normal_y):
    """Положительный множитель записи закона: первая ненулевая компонента нормали.

    Положительный, а не первый попавшийся, и это НЕ вкус. Деление всей записи
    на отрицательное число перевернуло бы нормаль, то есть поменяло сторону,
    в которую идёт материал, — а сторона у фронта и есть половина ответа.
    Знак снимается точным предикатом ядра (`exact_sign`), поэтому «первая
    ненулевая» здесь означает доказанно ненулевую, а не «не похожая на ноль».

    `None` — либо нормаль нулевая (прямой у закона нет вовсе, и мост скажет
    это своим именем), либо знак компоненты не доказуем. Второе отличимо от
    первого текстом, потому что лечится оно по-разному.
    """

    for value in (normal_x, normal_y):
        try:
            sign = exact_sign(value)
        except CertifiedPredicateUndecidable as exc:
            return None, f"знак компоненты нормали не доказан ({exc})"
        if sign == 0:
            continue
        return (value if sign > 0 else -value), ""
    return None, "нормаль закона нулевая, прямой у него нет"


def _rational_after_scaling(value, scale) -> Fraction | None:
    """Точная дробь `value/scale`, ДОКАЗАННАЯ обратной подстановкой.

    Одного `is_Rational` у частного было бы мало: он отвечает про то
    выражение, которое получилось, а вопрос стоит про исходное. Поэтому
    найденная дробь `r` возвращается только если `value - r*scale`
    обращается в ноль ТОЧНО (`is_zero is True`, а не «не отличается от нуля»).
    Ни порога, ни численной проверки здесь нет.

    `None` — доказательства нет. Тогда вызывающий обязан оставить прежний
    именованный отказ, а не принять запись «на глаз».
    """

    candidate = exact_rational(sp.radsimp(value / scale))
    if candidate is None:
        return None
    residue = sp.simplify(
        value - sp.Rational(candidate.numerator, candidate.denominator) * scale
    )
    return candidate if residue.is_zero is True else None


def _read_arrival_law(
    name: str, normal, constant, speed_squared
) -> tuple[PlainArrivalLawV1 | None, bool, str]:
    """Запись закона в точных дробях: как есть либо пере-масштабированная.

    ПОЧЕМУ ПЕРЕ-МАСШТАБИРОВАНИЕ ЗАКОННО. Закон прихода есть равенство
    `n*p = c + t*s`, и умножение всего равенства на положительное число даёт
    ТО ЖЕ множество точек в те же моменты времени: прямая та же, движение то
    же, сторона та же. Значит запись `(n, c, s)` — не сам закон, а одна из его
    записей, и выбор записи вызывающему ничего не стоит.

    ЗАЧЕМ ОНА НУЖНА. У полевых законов `s = 1` тождественно, и на доменах
    `building_002_weighted_normals_v1` нормаль при этом ИРРАЦИОНАЛЬНА:
    `n = (-sqrt(426753013)/8192, 0)`, `c = -sqrt(426753013)/8192`. Очередь же
    живёт в точных дробях. Деление всей записи на `|λ| = sqrt(426753013)/8192`
    даёт `n' = (-1, 0)`, `c' = -1`, `s'^2 = 67108864/426753013` — всё
    рационально, потому что `λ^2` рационально, даже когда сам `λ` нет.

    ЧТО ПРИ ЭТОМ НЕ ДВИГАЕТСЯ, и это главное. Мост берёт у закона ровно две
    вещи: класс несущей прямой (`line_class`, нормировка делением на модуль
    первой ненулевой компоненты) и отношение `(s/|n|)^2`. Обе инвариантны
    относительно умножения записи на положительное число — у отношения
    множитель сокращается в числителе и знаменателе, у класса он снимается
    самой нормировкой. Поэтому пере-масштабирование меняет ЗАПИСЬ и не может
    изменить ни сопоставление закона с ребром, ни `q` этого ребра.

    Порядок попыток тоже не косметика: сначала читается запись как есть, и
    только на её отказе пробуется масштаб. Отсюда следует, что на входах, где
    законы уже рациональны, счётчик `CONVEYOR_RESCALED_ARRIVAL_LAWS` равен
    нулю — и это проверяемое утверждение, а не намерение.
    """

    normal_x, normal_y = normal.expressions()
    law_constant = constant.as_expr()
    plain = (
        exact_rational(normal_x),
        exact_rational(normal_y),
        exact_rational(law_constant),
        exact_rational(speed_squared),
    )
    if all(item is not None for item in plain):
        return _plain_law(name, plain), False, ""

    scale, issue = _positive_common_factor(normal_x, normal_y)
    if scale is None:
        return None, False, issue
    scaled = tuple(
        _rational_after_scaling(value, divisor)
        for value, divisor in (
            (normal_x, scale),
            (normal_y, scale),
            (law_constant, scale),
            (speed_squared, scale * scale),
        )
    )
    if any(item is None for item in scaled):
        return None, False, "запись закона не рациональна и после масштаба"
    return _plain_law(name, scaled), True, ""


def _plain_law(name: str, fields) -> PlainArrivalLawV1:
    normal_x, normal_y, law_constant, speed_squared = fields
    return PlainArrivalLawV1(
        name=name,
        normal_x=normal_x,
        normal_y=normal_y,
        constant=law_constant,
        speed_squared=speed_squared,
    )


def _integer_normal(
    law: PlainArrivalLawV1,
) -> tuple[int, int, Fraction]:
    """Запись закона с ЦЕЛОЙ нормалью: `(a, b, q)` для несущей прямой очереди.

    Очередь живёт с целыми `a`, `b` (все её предикаты — целочисленные
    определители) и рациональным `q` под корнем. Закон приходит с рациональной
    нормалью, поэтому её надо умножить на общий знаменатель, а `q` — на КВАДРАТ
    того же множителя: прямая `n*p = c + t*s` при замене `n -> λn` движется на
    `s/|n|` за единицу времени по-прежнему, и чтобы `sqrt(q)/|λn|` дало то же
    число, нужно `q = (λs)^2`.

    Дальше `(a, b)` сокращается на свой НОД, и `q` — на его квадрат. Это не
    косметика: `q` входит в `line_key`, ключ прямой попадает в дайджест, и две
    записи одной прямой давали бы два разных дайджеста при одной геометрии.
    """

    scale = (
        law.normal_x.denominator * law.normal_y.denominator
        // gcd(law.normal_x.denominator, law.normal_y.denominator)
    )
    a = int(law.normal_x * scale)
    b = int(law.normal_y * scale)
    common = gcd(abs(a), abs(b)) or 1
    a, b = a // common, b // common
    return a, b, law.speed_squared * Fraction(scale, common) ** 2


@dataclass(frozen=True, slots=True)
class PlainVertexFanV1:
    """Веер вогнутой вершины домена в ТОЧНЫХ ДРОБЯХ, до всякой решётки.

    `point` — якорь угла (вершина домена), `supports` — целые нормали скрытых
    опор с их `q`, в порядке от входящей опоры к исходящей. Константа прямой
    здесь не хранится намеренно: после привязки к решётке якорь СДВИГАЕТСЯ, и
    константа, посчитанная до привязки, описывала бы прямую, не проходящую через
    вершину. Прямая веера определена тем, что проходит через СВОЮ вершину, и это
    определение переживает привязку, а константа нет.
    """

    name: str
    point: tuple[Fraction, Fraction]
    supports: tuple[tuple[int, int, Fraction], ...]


@dataclass(frozen=True, slots=True)
class _ArrivalLawsV1:
    """Законы плана, счёт пере-масштабированных и причина отказа, если он есть."""

    laws: tuple[PlainArrivalLawV1, ...]
    rescaled_count: int
    detail: str | None
    #: Вееры вогнутых вершин, чьи направления РАЦИОНАЛЬНЫ (после рескейла).
    fans: tuple[PlainVertexFanV1, ...] = ()
    #: Сколько вееров пришлось отклонить: направление алгебраично и записью в
    #: точных дробях не выражается. Тихого отката к митрованному углу здесь нет
    #: — вызывающий обязан ответить `ANGULAR_PROFILE_DIRECTION_IS_NOT_RATIONAL`.
    irrational_fan_count: int = 0
    #: Вогнутые вершины, у которых профиль выбрал `k = 0`. Веера у них нет, и это
    #: не отказ: митрованный угол есть законный член семейства.
    mitered_corner_count: int = 0


def _arrival_laws(context: GeometryContext) -> _ArrivalLawsV1:
    """Законы прихода Strip-источников плана и ВЕЕРЫ вогнутых вершин.

    Имя закона — `envelope_spec_id`, а не `envelope_instance_id`, и это не
    удобство: имя экземпляра стоит на эффективной alpha и потому alpha-зависимо
    (измерено: три различных имени при 0.25 против 0.5). Спека же — сама
    идентичность источника, и она от alpha не двигается.

    ANGULAR-СПЕКИ БОЛЬШЕ НЕ ОТБРАСЫВАЮТСЯ. Прежде здесь стояло
    `isinstance(spec, StripEnvelopeSpec)` и всё, что им не было, исчезало молча,
    — то есть очередь считала МИТРОВАННЫЙ угол там, где продукт требует мягкий,
    и разницу нечем было заметить. Теперь скрытые опоры угла идут отдельным
    каналом: они проходят не через ребро домена (их прямые пересекаются в самой
    вершине), а через пер-вершинную разметку `PlainVertexFanV1`.
    """

    compilation = context.compilation
    laws: list[PlainArrivalLawV1] = []
    rescaled_count = 0
    speed_squared = (
        STRIP_FRONT_NORMAL_SPEED.as_expr() * STRIP_FRONT_NORMAL_SPEED.as_expr()
    )
    for spec in sorted(
        compilation.envelope_specs, key=lambda item: item.envelope_spec_id.value
    ):
        if not isinstance(spec, StripEnvelopeSpec):
            continue
        seed = next(
            item
            for item in compilation.seeds
            if getattr(item, "seed_id", None) == spec.source_seed_id
        )
        for source in context.support_segments_for_use(
            seed.chain_use_id, spec.envelope_spec_id.value
        ):
            normal, constant = strip_front_support_line(context, source)
            law, rescaled, issue = _read_arrival_law(
                spec.envelope_spec_id.value, normal, constant, speed_squared
            )
            if law is None:
                return _ArrivalLawsV1(
                    (),
                    rescaled_count,
                    f"{spec.envelope_spec_id.value}/{source.support_id}: "
                    f"{issue}",
                )
            laws.append(law)
            rescaled_count += int(rescaled)
    fans = _angular_fans(context)
    return _ArrivalLawsV1(
        tuple(laws),
        rescaled_count + fans.rescaled_count,
        None,
        fans.fans,
        fans.irrational_fan_count,
        fans.mitered_corner_count,
    )


@dataclass(frozen=True, slots=True)
class _AngularFansV1:
    """Вееры плана, счёт отклонённых и счёт митрованных углов."""

    fans: tuple[PlainVertexFanV1, ...]
    rescaled_count: int
    irrational_fan_count: int
    mitered_corner_count: int


def _angular_fans(context: GeometryContext) -> _AngularFansV1:
    """Веер каждой вогнутой вершины домена — ТЕМ ЖЕ рецептом, что у эталона.

    Направления берутся у `angular_hidden_support_lines`, то есть у
    `_angular_support_data` / `_interpolated_normals` эталона. Второй реализации
    поворота здесь нет: рецепт направлений и есть то, чем `LINEAR_REFLEX_EQUAL_V1`
    отличается от любого другого профиля, и разойдись он между путями — очередь
    считала бы другой продукт, а не другую запись.

    РАЦИОНАЛЬНОСТЬ ПРОВЕРЯЕТСЯ ПОСЛЕ РЕСКЕЙЛА, и это не мелочь. Биссектриса
    перпендикулярного угла есть сумма двух единичных нормалей, каждая из которых
    иррациональна, а их сумма после деления на общий множитель — целый вектор:
    `(-1/sqrt(2), -1/sqrt(2))` даёт `(-1, -1)` при `q = 2`. Проверять до
    рескейла значило бы отклонять весь полевой корпус на записи, а не на
    геометрии.

    Отклонённый веер НЕ заменяется митром. Счёт отклонённых уходит наверх, и
    вызывающий обязан ответить именем: молча посчитать острый угол там, где
    продукт требует мягкий, — это подмена ответа, а не приближение.
    """

    speed_squared = (
        ANGULAR_PROFILE_NORMAL_SPEED.as_expr()
        * ANGULAR_PROFILE_NORMAL_SPEED.as_expr()
    )
    fans: list[PlainVertexFanV1] = []
    rescaled_count = 0
    irrational = 0
    mitered = 0
    for spec in sorted(
        context.compilation.envelope_specs,
        key=lambda item: item.envelope_spec_id.value,
    ):
        if not isinstance(spec, AngularEnvelopeSpec):
            continue
        if spec.resolved_hidden_edge_count == 0:
            # `k = 0` — тот самый митрованный угол, и он законный член
            # семейства: скрытых опор нет, вставлять в фронт нечего.
            mitered += 1
            continue
        name = spec.envelope_spec_id.value
        _, anchor, hidden = angular_hidden_support_lines(context, spec)
        point = _rational_point(anchor)
        supports: list[tuple[int, int, Fraction]] = []
        for support_id, normal, constant in hidden:
            law, rescaled, _ = _read_arrival_law(
                f"{name}/{support_id}", normal, constant, speed_squared
            )
            if law is None:
                supports = []
                break
            rescaled_count += int(rescaled)
            supports.append(_integer_normal(law))
        if point is None or not supports:
            irrational += 1
            continue
        fans.append(PlainVertexFanV1(name, point, tuple(supports)))
    return _AngularFansV1(tuple(fans), rescaled_count, irrational, mitered)


def _rational_point(point) -> tuple[Fraction, Fraction] | None:
    x = exact_rational(point.x.as_expr())
    y = exact_rational(point.y.as_expr())
    return None if x is None or y is None else (x, y)


def _region_loops(region):
    """Петли региона точными дробями: внешняя первой, дальше дыры."""

    loops: list[tuple[tuple[Fraction, Fraction], ...]] = []
    for source in (region.outer, *region.holes):
        points: list[tuple[Fraction, Fraction]] = []
        for point in source.points:
            x = exact_rational(point.x.as_expr())
            y = exact_rational(point.y.as_expr())
            if x is None or y is None:
                return None, f"{region.region_id}: координата не рациональна"
            points.append((x, y))
        loops.append(tuple(points))
    return tuple(loops), None


def _prepare_region(
    region, reading: _ArrivalLawsV1, lattice: GridSpecV1, clock: _Clock
) -> tuple[PreparedRegionV1 | None, str | None]:
    started = time.perf_counter()
    loops, issue = _region_loops(region)
    clock.add("DOMAIN_LOOPS", started)
    if loops is None:
        return None, issue

    started = time.perf_counter()
    report = bridge_arrival_laws(
        loops,
        reading.laws,
        lattice=lattice,
        weighted_fronts=True,
        vertex_fans=tuple(
            VertexFanLawV1(fan.name, fan.point, fan.supports)
            for fan in reading.fans
        ),
    )
    clock.add("BRIDGE", started)
    prepared = PreparedRegionV1(
        region_id=region.region_id,
        bridge=report,
        skeleton=None,
        partition=None,
        bridge_outcome=report.outcome,
        skeleton_outcome=None,
        face_outcome=None,
        owner_by_edge=report.owner_by_edge,
        wall_spans=report.wall_spans,
        wall_edge_count=report.wall_edge_count,
        ambiguous_owner_spans=report.ambiguous_owner_spans,
    )
    if report.polygon is None:
        return prepared, None

    started = time.perf_counter()
    skeleton = build_skeleton(report.polygon)
    clock.add("SKELETON", started)
    if skeleton.outcome is not SkeletonOutcome.EXACT:
        return (
            _with_skeleton(prepared, skeleton, None),
            None,
        )

    started = time.perf_counter()
    partition = build_faces(report.polygon, skeleton)
    clock.add("FACES", started)
    return _with_skeleton(prepared, skeleton, partition), None


def _with_skeleton(
    prepared: PreparedRegionV1,
    skeleton: SkeletonV1,
    partition: FacePartitionV1 | None,
) -> PreparedRegionV1:
    return PreparedRegionV1(
        region_id=prepared.region_id,
        bridge=prepared.bridge,
        skeleton=skeleton,
        partition=partition,
        bridge_outcome=prepared.bridge_outcome,
        skeleton_outcome=skeleton.outcome,
        face_outcome=None if partition is None else partition.outcome,
        owner_by_edge=prepared.owner_by_edge,
        wall_spans=prepared.wall_spans,
        wall_edge_count=prepared.wall_edge_count,
        ambiguous_owner_spans=prepared.ambiguous_owner_spans,
    )


def _preparation_outcome(
    regions: tuple[PreparedRegionV1, ...],
) -> tuple[ConveyorOutcome, str]:
    """Первый неудавшийся регион называет ступень; исход ступени лежит при нём.

    Порядок проверок совпадает с порядком ступеней, и другого быть не может:
    скелет без полигона не строится, грани без скелета — тоже.
    """

    for region in regions:
        if region.bridge_outcome is not BridgeOutcome.EXACT:
            return (
                ConveyorOutcome.BRIDGE_DID_NOT_MAP,
                f"{region.region_id}: {region.bridge_outcome.value}",
            )
        if region.skeleton_outcome is not SkeletonOutcome.EXACT:
            return (
                ConveyorOutcome.SKELETON_DID_NOT_CLOSE,
                f"{region.region_id}: {region.skeleton_outcome}",
            )
        if region.face_outcome is not FaceOutcome.EXACT:
            return (
                ConveyorOutcome.FACES_DID_NOT_ASSEMBLE,
                f"{region.region_id}: {region.face_outcome}",
            )
    return ConveyorOutcome.EXACT, ""


def _law_counters(reading: _ArrivalLawsV1) -> Counters:
    """Всё, что известно про законы и вееры, ДО первого региона.

    Переживает любой поздний отказ: «законы были рациональны», «законы спасены
    масштабом» и «веер отклонён» лечатся по-разному, и без счётчика поздний
    отказ унёс бы это различение с собой.
    """

    return (
        ("CONVEYOR_ARRIVAL_LAWS", len(reading.laws)),
        ("CONVEYOR_RESCALED_ARRIVAL_LAWS", reading.rescaled_count),
        ("CONVEYOR_RATIONAL_VERTEX_FANS", len(reading.fans)),
        ("CONVEYOR_IRRATIONAL_VERTEX_FANS", reading.irrational_fan_count),
        ("CONVEYOR_MITERED_CORNERS", reading.mitered_corner_count),
        (
            "CONVEYOR_FAN_SUPPORTS",
            sum(len(fan.supports) for fan in reading.fans),
        ),
    )


def _preparation_counters(
    regions: tuple[PreparedRegionV1, ...],
    reading: _ArrivalLawsV1,
    lattice: GridSpecV1,
) -> Counters:
    return _law_counters(reading) + (
        ("CONVEYOR_DOMAIN_REGIONS", len(regions)),
        ("CONVEYOR_LATTICE_SCALE", lattice.scale),
        (
            "CONVEYOR_FAN_EDGES",
            sum(item.bridge.fan_edge_count for item in regions),
        ),
        ("CONVEYOR_DOMAIN_EDGES", sum(item.bridge.edge_count for item in regions)),
        (
            "CONVEYOR_SOURCE_EDGES",
            sum(item.bridge.matched_edge_count for item in regions),
        ),
        ("CONVEYOR_WALL_EDGES", sum(item.wall_edge_count for item in regions)),
        (
            "CONVEYOR_AMBIGUOUS_OWNER_EDGES",
            sum(len(item.ambiguous_owner_spans) for item in regions),
        ),
        (
            "CONVEYOR_WEIGHTED_SOURCE_EDGES",
            sum(item.bridge.weighted_edge_count for item in regions),
        ),
        (
            "CONVEYOR_SKELETON_NODES",
            sum(
                0 if item.skeleton is None else len(item.skeleton.nodes)
                for item in regions
            ),
        ),
        (
            "CONVEYOR_FACES",
            sum(
                0 if item.partition is None else len(item.partition.faces)
                for item in regions
            ),
        ),
    )


def prepare_conveyor(
    snapshot: AnalysisSnapshotV1,
    request: DecalRequestV1,
    *,
    patch_domain_id: PatchDomainId | None = None,
) -> ConveyorPreparationV1:
    """Alpha-независимая подготовка очереди для домена плана.

    Союз не считается, попарный крой не зовётся, `exact_union` не вызывается ни
    разу.

    Проходятся ВСЕ регионы домена, а не первый: хелпер теста брал
    `domain_regions[0]`, и разница была невидима, то есть непроверяема. Сколько
    их на самом деле — ИЗМЕРЕНО, и ответ сильнее фикстуры:
    `SparsePatchDomainGeometryV1.domain_regions` возвращает РОВНО ОДИН регион
    по построению, а при нескольких внешних петлях отказывает
    `REFERENCE_INPUT_CONTRACT_INVALID`. То есть в v1 второго региона не бывает
    вовсе, и цикл здесь — не запас на будущее, а отсутствие молчаливой
    зависимости от этого контракта. Число регионов лежит счётчиком
    `CONVEYOR_DOMAIN_REGIONS`, и оно на фикстуре равно 1 не случайно.
    """

    clock = _Clock()
    inputs, refusal = _prepare_inputs(snapshot, request, patch_domain_id, clock)
    if inputs is None:
        return refusal
    compilation, context, domain, reading, lattice = inputs
    laws = reading.laws
    # Всё, что уже известно про законы и вееры, переживает любой поздний отказ.
    law_counters: Counters = _law_counters(reading)

    prepared_regions: list[PreparedRegionV1] = []
    for region in domain.domain_regions:
        prepared, issue = _prepare_region(region, reading, lattice, clock)
        if prepared is None:
            return _refused(
                ConveyorOutcome.DOMAIN_POINT_IS_NOT_RATIONAL,
                issue or "",
                clock,
                law_counters,
            )
        prepared_regions.append(prepared)

    regions = tuple(prepared_regions)
    outcome, detail = _preparation_outcome(regions)
    return ConveyorPreparationV1(
        outcome=outcome,
        regions=regions,
        lattice=lattice,
        law_names=tuple(law.name for law in laws),
        counters=_preparation_counters(regions, reading, lattice),
        timings=clock.timings(),
        detail=detail,
        compilation=compilation,
        context=context,
        domain=domain,
        requested_alpha=normalize_requested_alpha(request.requested_alpha),
    )


def _prepare_inputs(snapshot, request, patch_domain_id, clock: _Clock):
    """Всё, что нужно очереди до первого региона: план, домен, законы, решётка.

    Вынесено из `prepare_conveyor` не ради красоты, а потому что ступеней тут
    пять и каждая со своим именованным отказом; вместе с циклом по регионам
    функция перевалила объявленный предел в 120 строк. Предел — храповик, и
    поднимать его вместо разбиения значило бы менять правило под код.

    Возвращает либо `(inputs, None)`, либо `(None, отказ)`. Два значения, а не
    исключение: отказ здесь — обычный ответ подготовки, несущий тайминги и
    счётчики, а не аварийная ситуация.
    """

    started = time.perf_counter()
    compiled = compile_reference_envelopes(snapshot, request, patch_domain_id)
    clock.add("PLAN_COMPILE", started)
    compilation = compiled.compilation
    if compilation is None:
        return None, _refused(
            ConveyorOutcome.PLAN_IS_NOT_COMPILED, compiled.outcome.value, clock
        )

    started = time.perf_counter()
    frame, payload_diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    if frame is None:
        clock.add("DOMAIN_BUILD", started)
        return None, _refused(
            ConveyorOutcome.DOMAIN_FRAME_IS_UNAVAILABLE,
            payload_diagnostics[0].outcome.value,
            clock,
        )
    try:
        context = GeometryContext.build(compilation, frame)
        domain = build_domain_geometry(context)
    except ReferenceGeometryError as exc:
        clock.add("DOMAIN_BUILD", started)
        return None, _refused(
            ConveyorOutcome.DOMAIN_GEOMETRY_REFUSED, exc.outcome.value, clock
        )
    clock.add("DOMAIN_BUILD", started)
    if not domain.domain_regions:
        return None, _refused(
            ConveyorOutcome.DOMAIN_HAS_NO_REGIONS,
            str(compilation.plan_key.patch_domain_id),
            clock,
        )

    started = time.perf_counter()
    try:
        reading = _arrival_laws(context)
    except ReferenceGeometryError as exc:
        clock.add("ARRIVAL_LAWS", started)
        return None, _refused(
            ConveyorOutcome.DOMAIN_GEOMETRY_REFUSED, exc.outcome.value, clock
        )
    clock.add("ARRIVAL_LAWS", started)
    if reading.detail is not None:
        return None, _refused(
            ConveyorOutcome.ARRIVAL_LAW_IS_NOT_RATIONAL, reading.detail, clock
        )
    law_counters: Counters = _law_counters(reading)
    if reading.irrational_fan_count:
        return None, _refused(
            ConveyorOutcome.ANGULAR_PROFILE_DIRECTION_IS_NOT_RATIONAL,
            f"{reading.irrational_fan_count} вогнутых вершин из "
            f"{reading.irrational_fan_count + len(reading.fans)} с веером",
            clock,
            law_counters,
        )
    if not reading.laws:
        return None, _refused(
            ConveyorOutcome.NO_STRIP_SOURCE_IN_THE_PLAN,
            str(compilation.plan_key.patch_domain_id),
            clock,
            law_counters,
        )

    started = time.perf_counter()
    lattice = chart_lattice_for_frame(frame)
    clock.add("CHART_LATTICE", started)
    if lattice is None:
        return None, _refused(
            ConveyorOutcome.CHART_LATTICE_IS_NOT_DECLARED,
            type(frame).__name__,
            clock,
            law_counters,
        )
    return (compilation, context, domain, reading, lattice), None


def _instance_ids_by_spec(
    prepared: ConveyorPreparationV1, alpha_value: LocalLengthV1
) -> tuple[dict[str, str] | None, str]:
    """Имя экземпляра юбки на ЭТОЙ alpha, по одному на Strip-спеку.

    Alpha сюда входит дважды и оба раза по существу: резолвер границы может
    УКОРОТИТЬ её до эффективной, а имя экземпляра стоит именно на эффективной.
    Вывести имя из запрошенной было бы дешевле на 5.3 мс (медиана пяти
    прогонов на фикстуре) и неверно ровно там, где резолвер сработал, — то
    есть молча.
    """

    resolutions, _ = resolve_component_alphas(
        prepared.context, alpha_value, prepared.domain
    )
    names: dict[str, str] = {}
    for spec in prepared.compilation.envelope_specs:
        if not isinstance(spec, StripEnvelopeSpec):
            continue
        effective = spec_effective_alpha(
            prepared.compilation, spec, resolutions, alpha_value
        )
        if effective is None:
            return None, spec.envelope_spec_id.value
        names[spec.envelope_spec_id.value] = strip_envelope_instance_id(
            spec, effective
        )
    return names, ""


def _region_coverage(
    region: PreparedRegionV1,
    lattice_alpha: Fraction,
    instance_ids: dict[str, str],
) -> ConveyorRegionCoverageV1:
    owner_names = dict(region.owner_by_edge)
    covered = coverage_at(region.partition, lattice_alpha)
    faces = tuple(
        ConveyorFaceCoverageV1(
            region_id=region.region_id,
            owner=face.owner,
            envelope_spec_id=owner_names.get(face.owner, ""),
            envelope_instance_id=instance_ids.get(
                owner_names.get(face.owner, "")
            ),
            doubled_area=face.doubled_area,
        )
        for face in covered.faces
    )
    return ConveyorRegionCoverageV1(
        region_id=region.region_id,
        outcome=covered.outcome,
        faces=faces,
        doubled_area=covered.doubled_area,
        polygon_doubled_area=covered.polygon_doubled_area,
        wall_spans=region.wall_spans,
    )


# Типы alpha, объявленные входом. Список тот же, что у
# `evaluate_reference_raw_coverage`, и совпадение не случайно: два входа ядра,
# принимающие alpha по-разному, разошлись бы на первом же хосте.
DECLARED_ALPHA_TYPES = (LocalLengthV1, Decimal, int, str)

_ALPHA_CONTRACT = (
    "alpha объявлена ДЕСЯТИЧНОЙ: LocalLengthV1, Decimal, int или str "
    '(и None — взять alpha запроса). Дробь со знаменателем вида 2^a*5^b '
    'передаётся строкой без потерь: conveyor_coverage(prepared, "0.25").'
)


def _require_declared_alpha(alpha) -> None:
    """Тип alpha проверяется НА ГРАНИЦЕ, и отказ носит имя.

    Без этой проверки `Fraction(1, 4)` уходил вглубь и падал голым
    `decimal.InvalidOperation` из `Decimal(str(alpha))` — трассой про
    десятичный модуль вместо ответа про контракт. Соседний `coverage_at` в
    этом же пакете `Fraction` принимает, поэтому контраст тем более обязан
    объясняться сам, а не через чтение стека.

    `Fraction` и `float` отвергаются по ОДНОЙ причине, и она не в удобстве:
    оба переводятся в `Decimal` с молчаливой подменой значения, а дальше ядро
    считает ТОЧНО — то есть точно не то число, которое имел в виду
    вызывающий. У дроби подмена от округления, у double — от того, что его
    десятичная запись не равна ему самому.
    """

    if alpha is None or isinstance(alpha, DECLARED_ALPHA_TYPES):
        return
    if isinstance(alpha, Fraction):
        reason = (
            "перевод произвольной дроби в Decimal округляет: 1/3 стало бы "
            "двадцатью восемью девятками, и расхождение с эталонным путём "
            "было бы тихим"
        )
    elif isinstance(alpha, float):
        reason = (
            "double не равен своей десятичной записи: 0.1 есть "
            "0.1000000000000000055511151231257827, а Decimal(str(0.1)) — "
            "ровно 1/10, то есть ДРУГОЕ число, и считаться дальше оно будет "
            "точно"
        )
    else:
        reason = "перевод в Decimal для этого типа не объявлен"
    raise TypeError(
        f"{type(alpha).__name__} не принимается как alpha: {reason}. "
        f"{_ALPHA_CONTRACT}"
    )


def _declared_alpha(alpha) -> LocalLengthV1:
    """alpha объявленного типа и читаемого значения, либо именованный отказ.

    Неразбираемая строка — тот же случай, что и чужой тип: `Decimal("шире")`
    бросает `InvalidOperation`, и на границе это снова трасса про десятичный
    модуль вместо ответа про контракт.
    """

    _require_declared_alpha(alpha)
    try:
        return normalize_requested_alpha(alpha)
    except (ArithmeticError, ValueError) as exc:
        raise ValueError(
            f"alpha {alpha!r} не читается десятичной записью ({exc}). "
            f"{_ALPHA_CONTRACT}"
        ) from exc


def _empty_coverage(
    outcome: ConveyorOutcome,
    detail: str,
    prepared: ConveyorPreparationV1,
    alpha: Fraction,
    lattice_alpha: Fraction,
    clock: _Clock,
) -> ConveyorCoverageV1:
    return ConveyorCoverageV1(
        outcome=outcome,
        alpha=alpha,
        lattice_alpha=lattice_alpha,
        regions=(),
        doubled_area=SqrtSumV1.zero(),
        polygon_doubled_area=0,
        preparation=prepared,
        counters=(),
        timings=clock.timings(),
        detail=detail,
    )


def conveyor_coverage(
    prepared: ConveyorPreparationV1,
    alpha: LocalLengthV1 | Decimal | int | str | None = None,
) -> ConveyorCoverageV1:
    """Покрытие домена очередью к моменту alpha на готовом разбиении.

    `alpha=None` берёт alpha запроса, ту же, что видела подготовка. Alpha
    переводится в единицы решётки умножением на `lattice.scale`: полигон
    привязан к ней, поэтому и время фронта считается в её единицах.

    Тип alpha — тот же, что у `evaluate_reference_raw_coverage`, и ни
    `Fraction`, ни `float` в него намеренно не входят: оба переводятся в
    `Decimal` с молчаливой подменой значения. Проверка стоит на границе и
    отвечает `TypeError`, называющим контракт (`_require_declared_alpha`), —
    голой трассы из `decimal` здесь быть не должно, тем более что соседний
    `coverage_at` в этом же пакете `Fraction` принимает.
    """

    clock = _Clock()
    if alpha is None:
        alpha_value = prepared.requested_alpha
    else:
        alpha_value = _declared_alpha(alpha)
    alpha_fraction = (
        Fraction(0) if alpha_value is None else Fraction(str(alpha_value.value))
    )
    scale = 1 if prepared.lattice is None else prepared.lattice.scale
    lattice_alpha = alpha_fraction * scale

    if prepared.outcome is not ConveyorOutcome.EXACT:
        return _empty_coverage(
            ConveyorOutcome.PREPARATION_IS_NOT_EXACT,
            prepared.outcome.value,
            prepared,
            alpha_fraction,
            lattice_alpha,
            clock,
        )

    started = time.perf_counter()
    instance_ids, issue = _instance_ids_by_spec(prepared, alpha_value)
    clock.add("EFFECTIVE_ALPHA", started)
    if instance_ids is None:
        return _empty_coverage(
            ConveyorOutcome.SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN,
            issue,
            prepared,
            alpha_fraction,
            lattice_alpha,
            clock,
        )

    started = time.perf_counter()
    regions = tuple(
        _region_coverage(region, lattice_alpha, instance_ids)
        for region in prepared.regions
    )
    clock.add("COVERAGE_CLIP", started)

    total = SqrtSumV1.zero()
    for region in regions:
        total = total + region.doubled_area
    refused = tuple(
        region
        for region in regions
        if region.outcome is not CoverageOutcome.EXACT
    )
    outcome = (
        ConveyorOutcome.EXACT
        if not refused
        else ConveyorOutcome.COVERAGE_DID_NOT_CLOSE
    )
    return ConveyorCoverageV1(
        outcome=outcome,
        alpha=alpha_fraction,
        lattice_alpha=lattice_alpha,
        regions=regions,
        doubled_area=total,
        polygon_doubled_area=sum(
            region.polygon_doubled_area for region in regions
        ),
        preparation=prepared,
        counters=(
            ("CONVEYOR_COVERED_REGIONS", len(regions)),
            (
                "CONVEYOR_COVERED_FACES",
                sum(len(region.faces) for region in regions),
            ),
            ("CONVEYOR_COVERAGE_TERMS", len(total.terms)),
            (
                "CONVEYOR_NAMED_OWNERS",
                sum(
                    1
                    for region in regions
                    for face in region.faces
                    if face.envelope_instance_id
                ),
            ),
            (
                "CONVEYOR_WALL_EDGES",
                sum(len(region.wall_spans) for region in regions),
            ),
        ),
        timings=clock.timings(),
        detail="" if not refused else refused[0].outcome.value,
    )


def evaluate_conveyor_coverage(
    snapshot: AnalysisSnapshotV1,
    request: DecalRequestV1,
    *,
    alpha: LocalLengthV1 | Decimal | int | str | None = None,
    patch_domain_id: PatchDomainId | None = None,
) -> ConveyorCoverageV1:
    """Обе ступени подряд, для вызывающего, которому кэш не нужен.

    Разрез при этом не исчезает: `prepare_conveyor` остаётся отдельным входом
    именно потому, что стоит 53 мс против 7 мс у покрытия, и хост, считающий
    несколько alpha на одном домене, платит подготовку один раз.

    Тип alpha проверяется ДО подготовки, а не внутри неё. Иначе неверный тип
    стоил бы 53 мс на скелет, который заведомо будет выброшен, — и хост,
    перебирающий alpha в цикле, платил бы за каждую свою опечатку полную
    подготовку.
    """

    _require_declared_alpha(alpha)
    return conveyor_coverage(
        prepare_conveyor(snapshot, request, patch_domain_id=patch_domain_id),
        alpha,
    )
