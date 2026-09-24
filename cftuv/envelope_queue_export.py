"""Движок QUEUE в Envelope-debug: подготовка и покрытие очереди для хоста.

Зачем отдельный модуль. Эталонный путь (`evaluate_reference_raw_coverage` +
`resolve_coverage_interactions`) остаётся нетронутым: движок LEGACY обязан
вести себя ровно так же, как вёл, пока владелец сам не переключит движок.
Поэтому вход очереди живёт своим файлом, а не веткой внутри эталонного
вычислителя — иначе «не изменилось» пришлось бы доказывать чтением диффа, а не
отсутствием диффа.

Что здесь есть:

1. отображение контрактов ядра (`ConveyorPreparationV1`, `ConveyorCoverageV1`)
   в неизменяемые записи хоста, пригодные для GP-проекции и sidecar;
2. палитра владельцев: `envelope_spec_id` -> слот 0..7, детерминированно по
   сортировке имён; переполнение палитры считается, а не замалчивается;
3. постадийный вычислитель домена, зовущий `prepare_conveyor` и
   `conveyor_coverage` под именованными стадиями профиля.

Чего здесь нет: геометрии. Модуль отображает контракты и не чинит их.

ПОЧЕМУ КОНТУР БЕРЁТСЯ УСЕЧЁННЫЙ. `ConveyorFaceCoverageV1` несёт площадь куска
и имена владельца, но не его контур. Полная грань разбиения
(`partition.faces[*].points`) лежит в тех же координатах решётки, и нарисовать
можно было бы её — но тогда картинка показывала бы БОЛЬШЕ, чем измеренное
число рядом с ней, и расхождение было бы тихим. Поэтому контур берётся у
`coverage_at` — того же вызова, которым покрытие и считалось, — и нарисованная
область равна измеренной по построению. Цена названа стадией `QUEUE_CONTOUR`.

ПОЧЕМУ ВНУТРЕННИЙ РАЗДЕЛИТЕЛЬ ОДНОЙ ЦЕПИ ПОДАВЛЯЕТСЯ, и это слой ОТОБРАЖЕНИЯ,
а не разбиения. Ядро строит по грани на КАЖДОЕ ребро-источник, и два
коллинеарных ребра одной `PhysicalChain` (полевой случай: `building.004`,
рёбра 18 и 38, общая вершина 6) дают ДВЕ грани, разделённые перпендикулярной
дугой из прямой вершины скелета. Разбиение при этом верное — дуга есть
настоящая граница двух граней, — но контур ПОКРЫТИЯ рисует её как ребро
декали, то есть показывает шов там, где у продукта его нет.

Подавляется ровно внутренняя граница и ровно там, где все четыре условия
доказаны ТОЧНО и целочисленно: один домен-регион, один класс несущей прямой у
рёбер-источников (`_integer_line_class` — та же формула, что у
`bridge.line_class`), одна `PhysicalChain` (`source_chain_by_span`) и
совпавшая семантика владельца (оба имени равны, в том числе оба пустые).
Порогов нет ни одного: общий сегмент границы опознаётся ТОЧНЫМ равенством
концов через канонические `SqrtSumV1.terms`.

Тихого исчезновения при этом не появилось, и следов тому три. Дуга по-прежнему
рисуется слоем скелета (`_skeleton_segments` читает ПОЛНЫЕ грани разбиения и
слияния не видит вовсе — роли слоёв разделены). Число подавленных
разделителей лежит счётчиком `CONTOUR_MERGED_SAME_CHAIN_SEPARATORS`. Владельцы
слитых граней перечислены поимённо в `merged_owners` самой записи и в
sidecar-разметке штриха.

ЧЕМ СЛИЯНИЕ ДОКАЗЫВАЕТ СЕБЯ, и почему НЕ площадью. Площадь тут проверять
нечего: сокращение встречных полурёбер вычитает из суммы трапеций ровно ноль
(`cross(p,q) + cross(q,p) = 0`), поэтому равенство площади слитого контура
сумме площадей граней выполняется ТОЖДЕСТВЕННО при любом обходе, в том числе
неверном. Проверка такого равенства мерила бы арифметику, а не сборку, — и
выглядела бы доказательством, не будучи им.

Проверяется то, что тождеством не является: обход обязан сложиться в ОДИН
цикл, покрывший все оставшиеся полурёбра (без повторов, развилок и второго
цикла), а полученный контур обязан быть ПРОСТЫМ — та же граница 1
(`contour_crossings`), которой ядро проверяет собственные грани, и тем же
точным предикатом знака. Не доказавшая себя группа остаётся НЕслитой и
считается `CONTOUR_MERGE_BOUNDARY_UNRESOLVED`: молчаливый запасной обход
означал бы, что верным считается тот контур, который получился.
"""

from __future__ import annotations

import time
from contextlib import nullcontext
from dataclasses import dataclass, replace
from fractions import Fraction


def _measure(profile, stage: str, patch_domain_id: str | None = None):
    """Стадия профиля, если профиль есть. Свой, а не заимствованный.

    Тот же трёхстрочный помощник живёт в `envelope_request_export`, но импорт
    оттуда тянет `model.py` и через него `mathutils` — то есть Blender — в
    лёгкий путь ползунка, которому ни то, ни другое не нужно.
    """

    if profile is None:
        return nullcontext()
    return profile.measure(stage, patch_domain_id)


# Слои очереди. Номера 90..92, а НЕ 50..52: числа 50, 51, 52 уже заняты
# эталонным путём (`ENV_50_INTERACTION_COMPONENTS`, `ENV_51_FRONT_READINGS`,
# `ENV_52_EQUALITY_LOCI`), и одинаковый префикс у слоёв разных движков читался
# бы как одна группа. Порядок номера задаёт только высоту подъёма над
# поверхностью, поэтому свободный диапазон ничего не стоит.
QUEUE_SKELETON_LAYER = "ENV_90_QUEUE_SKELETON"
QUEUE_WALL_LAYER = "ENV_91_QUEUE_WALLS"
QUEUE_OWNER_LAYER_PREFIX = "ENV_92_QUEUE_OWNER_"
QUEUE_OWNER_SLOT_COUNT = 8
QUEUE_OWNER_LAYERS = tuple(
    f"{QUEUE_OWNER_LAYER_PREFIX}{slot:02d}"
    for slot in range(QUEUE_OWNER_SLOT_COUNT)
)

# Восемь цветов, различимых на тёмном фоне вьюпорта и между собой. Слот
# владельца — индекс в этом кортеже, поэтому порядок здесь и есть палитра.
QUEUE_OWNER_COLORS = (
    (1.00, 0.25, 0.25, 1.0),
    (1.00, 0.60, 0.10, 1.0),
    (1.00, 0.95, 0.15, 1.0),
    (0.30, 1.00, 0.30, 1.0),
    (0.10, 0.90, 0.95, 1.0),
    (0.30, 0.50, 1.00, 1.0),
    (0.85, 0.35, 1.00, 1.0),
    (1.00, 1.00, 1.00, 1.0),
)

QUEUE_LAYER_STYLES = {
    QUEUE_SKELETON_LAYER: ((0.55, 0.55, 0.60, 1.0), 3),
    QUEUE_WALL_LAYER: ((0.45, 0.45, 0.45, 1.0), 5),
    **{
        name: (QUEUE_OWNER_COLORS[slot], 7)
        for slot, name in enumerate(QUEUE_OWNER_LAYERS)
    },
}

QUEUE_VISIBILITY_PROPERTY = "envelope_debug_show_queue"

ENVELOPE_DEBUG_ENGINE_LEGACY = "LEGACY"
ENVELOPE_DEBUG_ENGINE_QUEUE = "QUEUE"

# Счётчик переполнения палитры. Девятый владелец получает слот первого, и это
# должно быть видно числом: молчание здесь означало бы две разные огибающие
# одного цвета без единого следа о том, что так вышло.
QUEUE_OWNER_PALETTE_WRAPPED = "QUEUE_OWNER_PALETTE_WRAPPED"

#: Сколько внутренних разделителей стадия контура подавила: каждая пара
#: встречных полурёбер, сокращённая при слиянии граней одной цепи, — единица.
CONTOUR_MERGED_SAME_CHAIN_SEPARATORS = "CONTOUR_MERGED_SAME_CHAIN_SEPARATORS"

#: Сколько ГРУПП граней слилось. Отдельно от числа разделителей, потому что три
#: коллинеарных ребра одной цепи дают одну группу и два разделителя, и по одному
#: числу эти два случая неотличимы.
CONTOUR_MERGED_SAME_CHAIN_GROUPS = "CONTOUR_MERGED_SAME_CHAIN_GROUPS"

#: Сколько групп-кандидатов НЕ слилось: обход объединения не сложился в один
#: цикл либо его площадь не совпала с суммой площадей граней. Ноль здесь —
#: измерение, а не умолчание: без счётчика «слияний не было» было бы
#: неотличимо от «слияние отказало молча».
CONTOUR_MERGE_BOUNDARY_UNRESOLVED = "CONTOUR_MERGE_BOUNDARY_UNRESOLVED"

#: Счётчики стадии контура, принадлежащие ХОСТУ, а не ядру. Пишутся ВСЕГДА и
#: перечнем: список из одного сработавшего счётчика не отличал бы ноль от
#: неизмеренного. Держатся отдельным полем `host_counters`, чтобы канонические
#: числа ядра (`counters`) остались побитово теми же.
HOST_CONTOUR_COUNTERS = (
    CONTOUR_MERGED_SAME_CHAIN_SEPARATORS,
    CONTOUR_MERGED_SAME_CHAIN_GROUPS,
    CONTOUR_MERGE_BOUNDARY_UNRESOLVED,
)

#: Разрядность целочисленной оболочки при переводе `SqrtSumV1` в число.
#: Читать величину по частям правило проекта запрещает; оболочка — объявленный
#: способ, и её середина отличается от истинного значения не больше чем на
#: половину ширины, то есть на 2^-64 в единицах решётки.
QUEUE_ENCLOSURE_BITS = 64


def sqrt_sum_float(value, *, bits: int = QUEUE_ENCLOSURE_BITS) -> float:
    """Число из `SqrtSumV1` — серединой строгой оболочки, а не по членам."""

    low, high = value.enclosure(bits)
    return float((low + high) / 2)


@dataclass(frozen=True, slots=True)
class EnvelopeQueueFaceV1:
    """Кусок покрытия: владелец обоими именами и его контур в метрах карты."""

    region_id: str
    owner: tuple[int, int, int, int]
    envelope_spec_id: str
    #: Имя экземпляра юбки при ЭТОЙ alpha. `None` — вывести не удалось, и
    #: причина лежит в исходе домена, а не подменена пустой строкой.
    envelope_instance_id: str | None
    points: tuple[tuple[float, float], ...]
    doubled_area: float
    doubled_area_text: str
    #: `PhysicalChain` ребра-источника. Аддитивное поле ХОСТА: ядро цепь в
    #: ключе владельца не несёт (`EdgeKey` — вхождение отрезка), а домен несёт
    #: её в провенансе каждого сегмента граничной петли. `None` — цепь не
    #: названа: скрытая опора веера, отказавшая выгрузка либо два разных ответа
    #: на один решёточный отрезок.
    source_chain_id: str | None = None
    #: Владельцы граней, слитых в ЭТОТ контур. Пусто — грань не сливалась.
    #: Перечень, а не число: без имён «грань слита» не говорит, с чем именно, и
    #: подавленный разделитель нечем было бы найти на меше.
    merged_owners: tuple[tuple[int, ...], ...] = ()


@dataclass(frozen=True, slots=True)
class EnvelopeQueueCoveredFaceV1:
    """Кусок покрытия ДО перевода в метры: ТОЧНЫЙ контур и оба имени владельца.

    Промежуточная запись стадии контура. Существует затем, что слияние
    внутренних разделителей обязано идти по точным координатам: во float'ах
    «тот же самый узел» превращается в «почти тот же», а порогов у этого слоя
    нет ни одного.
    """

    region_id: str
    owner: tuple[int, ...]
    envelope_spec_id: str
    envelope_instance_id: str | None
    #: Точки контура как их вернул `coverage_at`: пары `SqrtSumV1`.
    points: tuple
    #: Удвоенная площадь куска, `SqrtSumV1`.
    doubled_area: object
    source_chain_id: str | None = None
    merged_owners: tuple[tuple[int, ...], ...] = ()


@dataclass(frozen=True, slots=True)
class EnvelopeQueueMergeStatsV1:
    """Числа слияния контуров одного региона. Ноль — тоже измерение."""

    merged_separators: int = 0
    merged_groups: int = 0
    unresolved_groups: int = 0

    def __add__(self, other: "EnvelopeQueueMergeStatsV1"):
        return EnvelopeQueueMergeStatsV1(
            self.merged_separators + other.merged_separators,
            self.merged_groups + other.merged_groups,
            self.unresolved_groups + other.unresolved_groups,
        )

    def counters(self) -> tuple[tuple[str, int], ...]:
        return (
            (CONTOUR_MERGED_SAME_CHAIN_SEPARATORS, self.merged_separators),
            (CONTOUR_MERGED_SAME_CHAIN_GROUPS, self.merged_groups),
            (CONTOUR_MERGE_BOUNDARY_UNRESOLVED, self.unresolved_groups),
        )


@dataclass(frozen=True, slots=True)
class EnvelopeQueueSegmentV1:
    """Отрезок служебного слоя очереди: дуга скелета либо стена."""

    layer: str
    region_id: str
    label: str
    points: tuple[tuple[float, float], ...]


@dataclass(frozen=True, slots=True)
class EnvelopeQueueRegionV1:
    """Исходы одного региона домена, каждый своим типом и своим именем."""

    region_id: str
    bridge_outcome: str
    skeleton_outcome: str | None
    face_outcome: str | None
    coverage_outcome: str | None
    findings: tuple[str, ...]
    wall_edge_count: int
    ambiguous_owner_span_count: int


@dataclass(frozen=True, slots=True)
class EnvelopeQueueDomainV1:
    """Домен, пройденный очередью: исходы, числа, геометрия для проекции."""

    patch_id: int
    patch_domain_id: str
    preparation_outcome: str
    coverage_outcome: str
    detail: str
    lattice_scale: int
    alpha: str
    lattice_alpha: str
    law_names: tuple[str, ...]
    regions: tuple[EnvelopeQueueRegionV1, ...]
    faces: tuple[EnvelopeQueueFaceV1, ...]
    segments: tuple[EnvelopeQueueSegmentV1, ...]
    counters: tuple[tuple[str, float], ...]
    timings: tuple[tuple[str, float], ...]
    prepare_seconds: float
    coverage_seconds: float
    contour_seconds: float
    #: Счётчики стадии контура, принадлежащие ХОСТУ. Отдельным полем от
    #: `counters`: те — канонические числа ядра, и подмешивать к ним измерения
    #: слоя отображения значило бы сделать «числа ядра» свойством хоста.
    host_counters: tuple[tuple[str, float], ...] = ()
    #: Сама `ConveyorPreparationV1`. Лежит рядом с записью, а не только в кэше
    #: сессии: лёгкий путь ползунка считает покрытие именно по ней, и искать
    #: её по ключу кэша означало бы второй способ её найти.
    preparation: object | None = None

    @property
    def is_exact(self) -> bool:
        return (
            self.preparation_outcome == "EXACT"
            and self.coverage_outcome == "EXACT"
        )


@dataclass(frozen=True, slots=True)
class EnvelopeQueueSceneV1:
    """Все домены очереди одного прогона плюс общая палитра владельцев."""

    domains: tuple[EnvelopeQueueDomainV1, ...]
    #: `envelope_spec_id` -> слот палитры. Сортировка имён, индекс по модулю
    #: восьми: цвет владельца не зависит ни от alpha, ни от порядка доменов.
    palette: tuple[tuple[str, int], ...]
    palette_wrapped: int

    def slot_of(self, envelope_spec_id: str) -> int:
        return dict(self.palette).get(str(envelope_spec_id), 0)

    def layer_of(self, envelope_spec_id: str) -> str:
        return QUEUE_OWNER_LAYERS[self.slot_of(envelope_spec_id)]

    @property
    def prepare_seconds(self) -> float:
        return sum(item.prepare_seconds for item in self.domains)

    @property
    def coverage_seconds(self) -> float:
        return sum(
            item.coverage_seconds + item.contour_seconds
            for item in self.domains
        )


def build_queue_palette(
    envelope_spec_ids,
) -> tuple[tuple[tuple[str, int], ...], int]:
    """Слот владельца по имени спеки: сортировка, индекс по модулю восьми.

    Ключ — `envelope_spec_id`, а не `envelope_instance_id`: имя экземпляра
    стоит на ЭФФЕКТИВНОЙ alpha и меняется при перетаскивании ползунка, то есть
    цвет владельца прыгал бы вместе с ним. Спека же — сама идентичность
    источника, и от alpha она не двигается.
    """

    unique = tuple(sorted({str(item) for item in envelope_spec_ids if item}))
    palette = tuple(
        (name, index % QUEUE_OWNER_SLOT_COUNT)
        for index, name in enumerate(unique)
    )
    wrapped = max(0, len(unique) - QUEUE_OWNER_SLOT_COUNT)
    return palette, wrapped


def build_queue_scene(domains) -> EnvelopeQueueSceneV1:
    domains = tuple(domains)
    palette, wrapped = build_queue_palette(
        face.envelope_spec_id
        for domain in domains
        for face in domain.faces
    )
    return EnvelopeQueueSceneV1(domains, palette, wrapped)


def _lattice_scale(prepared) -> int:
    for region in prepared.regions:
        scale = region.bridge.lattice_scale
        if scale:
            return int(scale)
    lattice = prepared.lattice
    return int(lattice.scale) if lattice is not None else 1


def _region_findings(region) -> tuple[str, ...]:
    """Все именованные наблюдения моста региона, а не только его исход.

    `findings` перечисляет ВСЕ применимые исходы, `outcome` — первый по
    объявленному порядку. Печатать один исход означало бы потерять остальные.
    """

    report = region.bridge
    findings = [item.value for item in report.findings]
    if report.unmatched_laws:
        findings.append(
            f"UNMATCHED_LAWS={len(report.unmatched_laws)}"
        )
    if report.surplus_laws:
        findings.append(f"SURPLUS_LAWS={len(report.surplus_laws)}")
    if report.non_unit_speed_laws:
        findings.append(
            f"NON_UNIT_SPEED_LAWS={len(report.non_unit_speed_laws)}"
        )
    if report.off_lattice_points:
        findings.append(
            f"OFF_LATTICE_POINTS={len(report.off_lattice_points)}"
        )
    if region.ambiguous_owner_spans:
        findings.append(
            f"AMBIGUOUS_OWNER_SPANS={len(region.ambiguous_owner_spans)}"
        )
    if report.undetermined_source_count:
        findings.append(
            f"UNDETERMINED_SOURCE_EDGES={report.undetermined_source_count}"
        )
    if report.snap_residual is not None and report.snap_residual != 0:
        findings.append(f"SNAP_RESIDUAL={report.snap_residual}")
    return tuple(findings)


def _skeleton_segments(region, scale: int) -> list[EnvelopeQueueSegmentV1]:
    """Дуги скелета региона: контур каждой грани без её опорного ребра.

    Порядок точек грани объявлен ядром: `(начало ребра, конец ребра, узлы...)`.
    Значит отрезок `points[0] -> points[1]` — само ребро домена, а вся
    остальная цепочка — дуги скелета. Обход `points[1] -> ... -> points[0]`
    берёт ровно их и ничего сверх.

    Дуга принадлежит двум соседним граням, поэтому отрезки дедуплицируются по
    ТОЧНЫМ координатам (`SqrtSumV1` каноничен и хешируем): иначе каждая дуга
    рисовалась бы дважды и число штрихов перестало бы быть измерением.
    """

    partition = region.partition
    if partition is None:
        return []
    seen: set[tuple] = set()
    segments: list[EnvelopeQueueSegmentV1] = []
    for face in partition.faces:
        chain = tuple(face.points[1:]) + (face.points[0],)
        for index in range(len(chain) - 1):
            start, end = chain[index], chain[index + 1]
            key = tuple(
                sorted(
                    (
                        (start[0].terms, start[1].terms),
                        (end[0].terms, end[1].terms),
                    )
                )
            )
            if key in seen:
                continue
            seen.add(key)
            segments.append(
                EnvelopeQueueSegmentV1(
                    QUEUE_SKELETON_LAYER,
                    region.region_id,
                    "SKELETON_ARC",
                    (
                        (
                            sqrt_sum_float(start[0]) / scale,
                            sqrt_sum_float(start[1]) / scale,
                        ),
                        (
                            sqrt_sum_float(end[0]) / scale,
                            sqrt_sum_float(end[1]) / scale,
                        ),
                    ),
                )
            )
    return segments


def _wall_segments(region, scale: int) -> list[EnvelopeQueueSegmentV1]:
    """Стены региона: рёбра домена без закона прихода, нейтральным цветом."""

    return [
        EnvelopeQueueSegmentV1(
            QUEUE_WALL_LAYER,
            region.region_id,
            "WALL_SPAN",
            (
                (span[0] / scale, span[1] / scale),
                (span[2] / scale, span[3] / scale),
            ),
        )
        for span in region.wall_spans
    ]


def _face_area_text(value) -> str:
    rational = value.as_rational()
    if rational is not None:
        return str(rational)
    return " + ".join(
        f"{coefficient}*sqrt({radicand})"
        for radicand, coefficient in value.terms
    )


def _covered_contours(region, lattice_alpha: Fraction, work_budget=None):
    """Усечённые по времени контуры граней региона, в порядке разбиения.

    Тот же вызов, которым `conveyor_coverage` считал площадь, поэтому порядок
    и владельцы совпадают с `ConveyorRegionCoverageV1.faces` по построению, а
    не по совпадению чисел.

    `work_budget` — бюджет ЭКСПОРТА, а не домена. Домен к этому моменту уже
    ответил, и подмешивать перерисовку в его счёт значило бы разрешить
    картинке превратить состоявшийся ответ в отказ. Свой бюджет держит второе
    обещание: экспорт тоже конечен и кончается ИМЕНЕМ, а не счётом.
    """

    from cftuv_envelope.wavefront.coverage import coverage_at

    if region.partition is None:
        return ()
    return coverage_at(region.partition, lattice_alpha, work_budget).faces


def undirected_span(owner) -> tuple[tuple[int, int], tuple[int, int]] | None:
    """Ключ ребра-источника БЕЗ направления. `None` — отрезка у владельца нет.

    Без направления, потому что физическое ребро направления не имеет, а обход
    решёточного полигона его задаёт: `PolygonV1.build` нормирует ориентацию
    петель, и петля домена может прийти в полигон развёрнутой. Направленный
    ключ тогда молча не нашёл бы цепь — то есть слияние выключилось бы без
    единого следа, а выключаться оно обязано только по доказанному условию.

    `None` у скрытой опоры веера: её ключ пятиместный и отрезка не задаёт.
    """

    if len(owner) != 4:
        return None
    start = (int(owner[0]), int(owner[1]))
    end = (int(owner[2]), int(owner[3]))
    if start == end:
        return None
    return (start, end) if start <= end else (end, start)


def _integer_line_class(owner):
    """Класс несущей прямой ЦЕЛОЧИСЛЕННОГО ребра. `None` — прямой у него нет.

    Формула та же, что у `bridge.line_class` и `bridge._rational_edges`:
    нормаль `(-dy, dx)` смотрит влево от хода, константа `a*x0 + b*y0`, вся
    тройка делится на модуль первой ненулевой компоненты нормали. Деление на
    МОДУЛЬ, а не на саму компоненту, сохраняет знак — то есть класс различает
    сторону, в которую идёт фронт, и два встречных ребра одной прямой в один
    класс не попадут.

    Арифметика целая и дробная, ни одного float: `Fraction` от целых точна.
    """

    if len(owner) != 4:
        return None
    x0, y0, x1, y1 = (int(item) for item in owner)
    a, b = y0 - y1, x1 - x0
    if a == 0 and b == 0:
        return None
    scale = abs(a) if a != 0 else abs(b)
    return (
        Fraction(a, scale),
        Fraction(b, scale),
        Fraction(a * x0 + b * y0, scale),
    )


def source_chain_by_span(prepared) -> dict:
    """`region_id -> (ненаправленный решёточный отрезок -> PhysicalChain)`.

    АДДИТИВНАЯ выгрузка хоста поверх готовой подготовки ядра: ни одного байта
    ядра она не двигает и ни одного его числа не пересчитывает. Ядру цепь не
    нужна — владельцем грани у него служит вхождение отрезка (`EdgeKey`), — а
    домену она известна: `_loop_segment` кладёт `physical_chain_ids` в
    провенанс КАЖДОГО сегмента граничной петли.

    Решёточный образ берётся ТЕМИ ЖЕ функциями ядра (`_region_loops` и
    `_lattice_image`), которыми мост строил полигон. Второй способ его
    посчитать разошёлся бы с первым молча — а разойтись ему есть на чём:
    привязка к решётке двигает вершины, и повторять её округление на глаз
    означало бы получать другой ответ на другом масштабе.

    Сегмент, у которого цепь названа не единственным именем, и отрезок, на
    который два сегмента ответили по-разному, остаются БЕЗ цепи: два разных
    ответа — не ответ, и выбирать между ними здесь нечем.
    """

    from cftuv_envelope.wavefront.bridge import _lattice_image
    from cftuv_envelope.wavefront.conveyor import _region_loops

    domain = getattr(prepared, "domain", None)
    if domain is None:
        return {}
    lattice = getattr(prepared, "lattice", None)
    result: dict[str, dict] = {}
    for region in domain.domain_regions:
        loops, _issue = _region_loops(region)
        if loops is None:
            continue
        lattice_loops, _off_lattice, _residual = _lattice_image(loops, lattice)
        sources = (region.outer, *region.holes)
        if lattice_loops is None or len(lattice_loops) != len(sources):
            continue
        by_span: dict[tuple, str | None] = {}
        for loop_index, source in enumerate(sources):
            nodes = lattice_loops[loop_index]
            segments = source.segments
            if len(segments) != len(nodes):
                continue
            for index, segment in enumerate(segments):
                names = tuple(
                    sorted(segment.provenance.physical_chain_ids)
                )
                if len(names) != 1:
                    continue
                start = nodes[index]
                end = nodes[(index + 1) % len(nodes)]
                span = undirected_span(
                    (start[0], start[1], end[0], end[1])
                )
                if span is None:
                    continue
                if span in by_span and by_span[span] != names[0]:
                    by_span[span] = None
                    continue
                by_span[span] = names[0]
        result[region.region_id] = {
            span: name for span, name in by_span.items() if name is not None
        }
    return result


def _point_key(point):
    """Тождество точки контура: канонические наборы её координат.

    `SqrtSumV1` каноничен, поэтому равные величины дают равные `terms`, а
    разные — разные. Тот же ключ уже служит дедупликации дуг скелета
    (`_skeleton_segments`), и второго тождества точки в модуле нет.
    """

    return (point[0].terms, point[1].terms)


def _merge_key(face: EnvelopeQueueCoveredFaceV1):
    """Ключ группы слияния либо `None`, если хоть одно условие не доказано.

    Условий четыре и все они — из карточки: один регион, один класс несущей
    прямой у рёбер-источников, одна `PhysicalChain`, совпавшая семантика
    владельца. Имя экземпляра входит в ключ вместе с именем спеки: экземпляр
    стоит на ЭФФЕКТИВНОЙ alpha, и две грани одной спеки с разными эффективными
    alpha — разные огибающие, а не одна.
    """

    line = _integer_line_class(face.owner)
    if line is None or face.source_chain_id is None:
        return None
    return (
        face.region_id,
        line,
        face.source_chain_id,
        face.envelope_spec_id,
        face.envelope_instance_id,
    )


def _half_edges(face):
    """Направленные полурёбра контура. Нулевой длины — пропускаются.

    У верных граней корпуса частичного источника сегмент нулевой длины
    встречается (два узла скелета в одной точке), площадь не двигает, а
    встречной пары у него нет — он совпал бы сам с собой.
    """

    points = face.points
    size = len(points)
    edges = []
    for index in range(size):
        start_key = _point_key(points[index])
        end_key = _point_key(points[(index + 1) % size])
        if start_key != end_key:
            edges.append((start_key, end_key))
    return edges


def _adjacent_components(block):
    """Компоненты СМЕЖНОСТИ внутри группы: кто с кем делит границу.

    Нужны затем, что группа — это ещё не соседство. Два коллинеарных ребра
    одной цепи могут стоять на противоположных концах домена, и их грани не
    касаются вовсе: сливать там нечего, и объявлять это неудачей слияния
    значило бы кричать на законном входе. Отказ `CONTOUR_MERGE_BOUNDARY_UNRESOLVED`
    остаётся именем настоящей аномалии — обхода, который не сложился у граней,
    смежность которых уже доказана.
    """

    keys = [frozenset(_half_edges(face)) for face in block]
    parent = list(range(len(block)))

    def root(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for left in range(len(block)):
        for right in range(left + 1, len(block)):
            if any(
                (end, start) in keys[right] for start, end in keys[left]
            ):
                parent[root(left)] = root(right)
    components: dict[int, list[int]] = {}
    for index in range(len(block)):
        components.setdefault(root(index), []).append(index)
    return tuple(components.values())


def _union_contour(faces):
    """Контур объединения граней и число сокращённых разделителей.

    Правило одно: внутренняя граница двух граней проходится ими в РАЗНЫЕ
    стороны, поэтому пара встречных полурёбер сокращается, а оставшиеся
    складываются в один обход. Ни одного знака `SqrtSumV1` при этом не
    спрашивается — сокращение и обход суть отношения на ключах точек.

    `(None, 0)` — обход не доказан: повторившееся полуребро, развилка, ни
    одного сокращения либо цикл, покрывший не все оставшиеся полурёбра.
    Молчаливого второго порядка обхода здесь нет.

    Сегменты нулевой длины сюда не попадают вовсе (`_half_edges`): встречной
    пары у них нет — они совпали бы сами с собой.
    """

    directed: dict[tuple, object] = {}
    for face in faces:
        points = face.points
        by_start = {
            _point_key(point): point for point in points
        }
        for key in _half_edges(face):
            if key in directed:
                return None, 0
            directed[key] = by_start[key[0]]
    shared = 0
    for key in tuple(directed):
        opposite = (key[1], key[0])
        if key in directed and opposite in directed:
            del directed[key]
            del directed[opposite]
            shared += 1
    if shared == 0 or len(directed) < 3:
        return None, 0

    successor: dict[tuple, tuple] = {}
    for start_key, end_key in directed:
        if start_key in successor:
            return None, 0
        successor[start_key] = end_key
    first = next(iter(directed))[0]
    walk: list[tuple] = []
    current = first
    while True:
        following = successor.get(current)
        if following is None:
            return None, 0
        walk.append((current, following))
        current = following
        if current == first:
            break
        if len(walk) > len(directed):
            return None, 0
    if len(walk) != len(directed):
        return None, 0
    return tuple(directed[edge] for edge in walk), shared


def merge_same_chain_faces(faces, work_budget=None):
    """Грани одной цепи на одной прямой — одним контуром. Точно и с числами.

    Возвращает `(грани, EnvelopeQueueMergeStatsV1)`. Порядок сохраняется:
    слитая запись встаёт на место ПЕРВОЙ грани группы, остальные исчезают из
    списка — но не из отчёта, их владельцы перечислены в `merged_owners`.

    Слияние принимается после ДВУХ проверок, и площади среди них нет: она
    выполняется тождественно (сокращение встречных полурёбер вычитает ноль) и
    доказывала бы арифметику вместо сборки. Проверяются обход — один цикл на
    все оставшиеся полурёбра — и ПРОСТОТА полученного контура той же границей
    1 ядра (`contour_crossings`, точный знак `SqrtSumV1`, ни одного порога).
    Группа, не прошедшая их, остаётся неслитой и считается отдельным числом.
    """

    from cftuv_envelope.wavefront.faces import contour_crossings

    faces = tuple(faces)
    groups: dict[tuple, list[int]] = {}
    for index, face in enumerate(faces):
        key = _merge_key(face)
        if key is None:
            continue
        groups.setdefault(key, []).append(index)

    replacement: dict[int, EnvelopeQueueCoveredFaceV1] = {}
    dropped: set[int] = set()
    separators = 0
    merged_groups = 0
    unresolved = 0
    for members in groups.values():
        if len(members) < 2:
            continue
        group = [faces[index] for index in members]
        for component in _adjacent_components(group):
            if len(component) < 2:
                continue
            positions = [members[index] for index in component]
            block = [group[index] for index in component]
            contour, shared = _union_contour(block)
            if contour is None or contour_crossings(contour, work_budget):
                unresolved += 1
                continue
            total = block[0].doubled_area
            for face in block[1:]:
                total = total + face.doubled_area
            separators += shared
            merged_groups += 1
            primary = min(block, key=lambda item: item.owner)
            replacement[min(positions)] = EnvelopeQueueCoveredFaceV1(
                region_id=primary.region_id,
                owner=primary.owner,
                envelope_spec_id=primary.envelope_spec_id,
                envelope_instance_id=primary.envelope_instance_id,
                points=contour,
                doubled_area=total,
                source_chain_id=primary.source_chain_id,
                merged_owners=tuple(
                    sorted(
                        item.owner
                        for item in block
                        if item.owner != primary.owner
                    )
                ),
            )
            dropped.update(sorted(positions)[1:])
    merged = tuple(
        replacement.get(index, face)
        for index, face in enumerate(faces)
        if index not in dropped
    )
    return merged, EnvelopeQueueMergeStatsV1(
        separators, merged_groups, unresolved
    )


def _projected_face(
    face: EnvelopeQueueCoveredFaceV1, scale: int
) -> EnvelopeQueueFaceV1:
    """Точная запись стадии контура -> метры карты. Единственный перевод."""

    return EnvelopeQueueFaceV1(
        region_id=face.region_id,
        owner=tuple(int(item) for item in face.owner),
        envelope_spec_id=face.envelope_spec_id,
        envelope_instance_id=face.envelope_instance_id,
        points=tuple(
            (
                sqrt_sum_float(point[0]) / scale,
                sqrt_sum_float(point[1]) / scale,
            )
            for point in face.points
        ),
        doubled_area=sqrt_sum_float(face.doubled_area),
        doubled_area_text=_face_area_text(face.doubled_area),
        source_chain_id=face.source_chain_id,
        merged_owners=face.merged_owners,
    )


def build_queue_domain(
    patch_id: int,
    patch_domain_id: str,
    prepared,
    coverage,
    *,
    prepare_seconds: float,
    coverage_seconds: float,
) -> EnvelopeQueueDomainV1:
    """Домен очереди -> неизменяемая запись хоста. Ни одного тихого исхода."""

    # Точная работа ЭКСПОРТА оплачивается своим счётом. Экспорт пересчитывает
    # два точных предиката поверх уже полученного ответа домена (усечение
    # контуров и простота слитого контура), и без названного бюджета обе
    # величины уходили в счёт без исхода — та самая дыра, которую закрывает
    # BUDGET-COVERAGE-STRUCTURAL. Бюджет ОТДЕЛЬНЫЙ от доменного намеренно:
    # домен уже ответил, и его потолок не должен зависеть от того, рисуем мы
    # картинку или нет.
    from cftuv_envelope.exact_sqrt_sum import exact_work_budget

    export_budget = exact_work_budget(
        stage="EXPORT", domain_id=str(patch_domain_id)
    )
    scale = _lattice_scale(prepared)
    coverage_by_region = {
        item.region_id: item for item in coverage.regions
    }
    regions: list[EnvelopeQueueRegionV1] = []
    faces: list[EnvelopeQueueFaceV1] = []
    segments: list[EnvelopeQueueSegmentV1] = []
    contour_started = time.perf_counter()
    chain_by_region = source_chain_by_span(prepared)
    merge_stats = EnvelopeQueueMergeStatsV1()
    for region in prepared.regions:
        covered = coverage_by_region.get(region.region_id)
        regions.append(
            EnvelopeQueueRegionV1(
                region_id=region.region_id,
                bridge_outcome=region.bridge_outcome.value,
                skeleton_outcome=(
                    None
                    if region.skeleton_outcome is None
                    else region.skeleton_outcome.value
                ),
                face_outcome=(
                    None
                    if region.face_outcome is None
                    else region.face_outcome.value
                ),
                coverage_outcome=(
                    None if covered is None else covered.outcome.value
                ),
                findings=_region_findings(region),
                wall_edge_count=int(region.wall_edge_count),
                ambiguous_owner_span_count=len(region.ambiguous_owner_spans),
            )
        )
        segments.extend(_skeleton_segments(region, scale))
        segments.extend(_wall_segments(region, scale))
        if covered is None:
            continue
        contours = _covered_contours(
            region, coverage.lattice_alpha, export_budget
        )
        spans = chain_by_region.get(region.region_id, {})
        covered_faces: list[EnvelopeQueueCoveredFaceV1] = []
        for index, named in enumerate(covered.faces):
            if index >= len(contours):
                break
            contour = contours[index]
            if contour.owner != named.owner or len(contour.points) < 3:
                continue
            owner = tuple(int(item) for item in named.owner)
            span = undirected_span(owner)
            covered_faces.append(
                EnvelopeQueueCoveredFaceV1(
                    region_id=named.region_id,
                    owner=owner,
                    envelope_spec_id=str(named.envelope_spec_id),
                    envelope_instance_id=named.envelope_instance_id,
                    points=tuple(contour.points),
                    doubled_area=named.doubled_area,
                    source_chain_id=(
                        None if span is None else spans.get(span)
                    ),
                )
            )
        merged, stats = merge_same_chain_faces(covered_faces, export_budget)
        merge_stats = merge_stats + stats
        faces.extend(_projected_face(item, scale) for item in merged)
    contour_seconds = time.perf_counter() - contour_started
    return EnvelopeQueueDomainV1(
        patch_id=int(patch_id),
        patch_domain_id=str(patch_domain_id),
        preparation_outcome=prepared.outcome.value,
        coverage_outcome=coverage.outcome.value,
        detail=coverage.detail or prepared.detail,
        lattice_scale=scale,
        alpha=str(coverage.alpha),
        lattice_alpha=str(coverage.lattice_alpha),
        law_names=tuple(prepared.law_names),
        regions=tuple(regions),
        faces=tuple(faces),
        segments=tuple(segments),
        counters=tuple(prepared.counters) + tuple(coverage.counters),
        timings=tuple(prepared.timings) + tuple(coverage.timings),
        prepare_seconds=float(prepare_seconds),
        coverage_seconds=float(coverage_seconds),
        contour_seconds=contour_seconds,
        host_counters=merge_stats.counters(),
    )


def refused_queue_domain(
    patch_id: int,
    patch_domain_id: str,
    prepared,
    coverage,
    *,
    prepare_seconds: float,
    coverage_seconds: float,
) -> EnvelopeQueueDomainV1:
    """Отказ — тоже результат: числа подготовки сохраняются, геометрии нет."""

    return EnvelopeQueueDomainV1(
        patch_id=int(patch_id),
        patch_domain_id=str(patch_domain_id),
        preparation_outcome=prepared.outcome.value,
        coverage_outcome=(
            "" if coverage is None else coverage.outcome.value
        ),
        # Отказавшая ПОДГОТОВКА объясняет себя подробнее покрытия: покрытие в
        # этом случае отвечает `PREPARATION_IS_NOT_EXACT` и пересказывает лишь
        # ИМЯ её исхода, а имя спеки и опоры, на которых закон оказался
        # нерациональным, лежит только в детали подготовки. Взять деталь
        # покрытия значило бы потерять её молча.
        detail=(
            prepared.detail
            if coverage is None or prepared.outcome.value != "EXACT"
            else (coverage.detail or prepared.detail)
        ),
        lattice_scale=_lattice_scale(prepared),
        alpha="" if coverage is None else str(coverage.alpha),
        lattice_alpha=(
            "" if coverage is None else str(coverage.lattice_alpha)
        ),
        law_names=tuple(prepared.law_names),
        regions=tuple(
            EnvelopeQueueRegionV1(
                region_id=region.region_id,
                bridge_outcome=region.bridge_outcome.value,
                skeleton_outcome=(
                    None
                    if region.skeleton_outcome is None
                    else region.skeleton_outcome.value
                ),
                face_outcome=(
                    None
                    if region.face_outcome is None
                    else region.face_outcome.value
                ),
                coverage_outcome=None,
                findings=_region_findings(region),
                wall_edge_count=int(region.wall_edge_count),
                ambiguous_owner_span_count=len(region.ambiguous_owner_spans),
            )
            for region in prepared.regions
        ),
        faces=(),
        segments=(),
        counters=tuple(prepared.counters)
        + (() if coverage is None else tuple(coverage.counters)),
        timings=tuple(prepared.timings)
        + (() if coverage is None else tuple(coverage.timings)),
        prepare_seconds=float(prepare_seconds),
        coverage_seconds=float(coverage_seconds),
        contour_seconds=0.0,
        # Стадии контура на отказавшем домене не было вовсе, и три нуля здесь —
        # именно это утверждение. Пустой кортеж означал бы «не измерялось», то
        # есть неотличимое от «измерялось и промолчало».
        host_counters=EnvelopeQueueMergeStatsV1().counters(),
    )


def queue_domain_payload(domain: EnvelopeQueueDomainV1) -> dict:
    """Sidecar-представление домена очереди. Исход каждой ступени — по имени."""

    return {
        "patch_id": domain.patch_id,
        "patch_domain_id": domain.patch_domain_id,
        "preparation_outcome": domain.preparation_outcome,
        "coverage_outcome": domain.coverage_outcome,
        "detail": domain.detail,
        "lattice_scale": domain.lattice_scale,
        "alpha": domain.alpha,
        "lattice_alpha": domain.lattice_alpha,
        "law_names": list(domain.law_names),
        "prepare_seconds": domain.prepare_seconds,
        "coverage_seconds": domain.coverage_seconds,
        "contour_seconds": domain.contour_seconds,
        "counters": [
            {"name": name, "value": value}
            for name, value in domain.counters
        ],
        # Числа ХОСТА своим ключом: смешать их с числами ядра значило бы
        # потерять ответ на вопрос «чьё это измерение».
        "host_counters": [
            {"name": name, "value": value}
            for name, value in domain.host_counters
        ],
        "timings": [
            {"stage": stage, "elapsed_seconds": value}
            for stage, value in domain.timings
        ],
        "regions": [
            {
                "region_id": region.region_id,
                "bridge_outcome": region.bridge_outcome,
                "skeleton_outcome": region.skeleton_outcome,
                "face_outcome": region.face_outcome,
                "coverage_outcome": region.coverage_outcome,
                "findings": list(region.findings),
                "wall_edge_count": region.wall_edge_count,
                "ambiguous_owner_span_count": (
                    region.ambiguous_owner_span_count
                ),
            }
            for region in domain.regions
        ],
    }


def queue_scene_payload(scene: EnvelopeQueueSceneV1) -> dict:
    """Соответствие слот -> цвет -> spec_id -> instance_id и все домены."""

    instance_by_spec: dict[str, str] = {}
    for domain in scene.domains:
        for face in domain.faces:
            if face.envelope_instance_id:
                instance_by_spec.setdefault(
                    face.envelope_spec_id,
                    face.envelope_instance_id,
                )
    return {
        "engine": ENVELOPE_DEBUG_ENGINE_QUEUE,
        "palette_wrapped": scene.palette_wrapped,
        "owners": [
            {
                "envelope_spec_id": spec_id,
                "envelope_instance_id": instance_by_spec.get(spec_id),
                "palette_slot": slot,
                "layer": QUEUE_OWNER_LAYERS[slot],
                "color": list(QUEUE_OWNER_COLORS[slot]),
            }
            for spec_id, slot in scene.palette
        ],
        "domains": [
            queue_domain_payload(domain) for domain in scene.domains
        ],
    }


def _queue_diagnostics(domain: EnvelopeQueueDomainV1):
    """Каждый неудавшийся исход и каждое наблюдение моста — своей строкой.

    Тихих исходов нет: `SKELETON_DID_NOT_CLOSE` без сообщения был бы виден
    только в JSON, а исход, который надо искать в JSON, почти неотличим от
    несостоявшегося.
    """

    from .envelope_request_export import (
        EnvelopeDebugHostDiagnosticV1,
        EnvelopeDebugHostSeverity,
    )

    rows = []
    if domain.preparation_outcome != "EXACT":
        rows.append(
            EnvelopeDebugHostDiagnosticV1(
                domain.preparation_outcome,
                EnvelopeDebugHostSeverity.ERROR,
                f"QUEUE prepare: {domain.detail}",
                domain.patch_domain_id,
            )
        )
    elif domain.coverage_outcome not in ("EXACT", ""):
        rows.append(
            EnvelopeDebugHostDiagnosticV1(
                domain.coverage_outcome,
                EnvelopeDebugHostSeverity.ERROR,
                f"QUEUE coverage: {domain.detail}",
                domain.patch_domain_id,
            )
        )
    for region in domain.regions:
        for finding in region.findings:
            if finding == "EXACT":
                continue
            rows.append(
                EnvelopeDebugHostDiagnosticV1(
                    finding,
                    EnvelopeDebugHostSeverity.UNSUPPORTED,
                    f"QUEUE bridge finding on {region.region_id}",
                    domain.patch_domain_id,
                )
            )
    return tuple(rows)


def _queue_stage(domain: EnvelopeQueueDomainV1):
    from .envelope_debug_profile import EnvelopeDomainStage

    if domain.preparation_outcome != "EXACT":
        return EnvelopeDomainStage.QUEUE_PREPARE_REJECTED
    if domain.coverage_outcome != "EXACT":
        return EnvelopeDomainStage.QUEUE_COVERAGE_REJECTED
    return EnvelopeDomainStage.QUEUE_RESOLVED


def _queue_receipt(domain: EnvelopeQueueDomainV1):
    from .envelope_debug_profile import (
        EnvelopeDomainStage,
        EnvelopeDomainStageReceiptV1,
    )

    stage = _queue_stage(domain)
    if stage is EnvelopeDomainStage.QUEUE_RESOLVED:
        outcome = "EXACT"
        message = (
            f"QUEUE coverage: {len(domain.faces)} owned faces, "
            f"prepare {domain.prepare_seconds * 1000.0:.1f} ms, "
            f"coverage {domain.coverage_seconds * 1000.0:.1f} ms"
        )
    elif stage is EnvelopeDomainStage.QUEUE_PREPARE_REJECTED:
        outcome = domain.preparation_outcome
        message = domain.detail or domain.preparation_outcome
    else:
        outcome = domain.coverage_outcome
        message = domain.detail or domain.coverage_outcome
    return EnvelopeDomainStageReceiptV1(
        domain.patch_id,
        domain.patch_domain_id,
        stage,
        outcome,
        message,
    )


def run_queue_domain(
    patch_id: int,
    patch_domain_id: str,
    snapshot,
    request,
    alpha_text: str,
    *,
    selected_edges: frozenset[int] = frozenset(),
    profile=None,
    preparation_provider=None,
):
    """Две ступени очереди на одном домене, каждая под своей стадией профиля.

    Возвращает `(ConveyorPreparationV1, EnvelopeQueueDomainV1)`: подготовка
    нужна вызывающему и дальше — её кэширует сессия и по ней же считается
    покрытие при смене alpha.
    """

    from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

    started = time.perf_counter()
    with _measure(profile, "QUEUE_PREPARE", patch_domain_id):
        if preparation_provider is None:
            prepared = prepare_conveyor(snapshot, request)
        else:
            prepared = preparation_provider(
                patch_id,
                patch_domain_id,
                frozenset(selected_edges),
                snapshot,
                request,
            )
    prepare_seconds = time.perf_counter() - started

    started = time.perf_counter()
    with _measure(profile, "QUEUE_COVERAGE", patch_domain_id):
        coverage = conveyor_coverage(prepared, alpha_text)
    coverage_seconds = time.perf_counter() - started

    builder = (
        build_queue_domain
        if coverage.outcome.value == "EXACT"
        else refused_queue_domain
    )
    return prepared, replace(
        builder(
            patch_id,
            patch_domain_id,
            prepared,
            coverage,
            prepare_seconds=prepare_seconds,
            coverage_seconds=coverage_seconds,
        ),
        preparation=prepared,
    )


def recompute_queue_coverage(
    entries,
    alpha_text: str,
    *,
    profile=None,
) -> EnvelopeQueueSceneV1:
    """Лёгкий путь ползунка: только покрытие на ГОТОВЫХ подготовках.

    `entries` — `(patch_id, patch_domain_id, prepared)`. Ни `prepare_conveyor`,
    ни компиляция плана здесь не вызываются: подготовка alpha-независима, и это
    измерено ядром побитовым совпадением скелета при 0.25 и 0.5.
    """

    from cftuv_envelope.wavefront import conveyor_coverage

    domains = []
    for patch_id, patch_domain_id, prepared in entries:
        started = time.perf_counter()
        with _measure(profile, "QUEUE_COVERAGE", patch_domain_id):
            coverage = conveyor_coverage(prepared, alpha_text)
        coverage_seconds = time.perf_counter() - started
        builder = (
            build_queue_domain
            if coverage.outcome.value == "EXACT"
            else refused_queue_domain
        )
        domains.append(
            replace(
                builder(
                    patch_id,
                    patch_domain_id,
                    prepared,
                    coverage,
                    prepare_seconds=0.0,
                    coverage_seconds=coverage_seconds,
                ),
                preparation=prepared,
            )
        )
    return build_queue_scene(domains)


def _queue_snapshot_and_request(
    analysis_bundle,
    patch_id: int,
    patch_domain_id: str,
    selected_edges: frozenset[int],
    alpha,
    request_id: str,
    *,
    density,
    profile=None,
    topology_export=None,
    domain_snapshot_provider=None,
):
    from .envelope_request_export import (
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )

    with _measure(profile, "SNAPSHOT_EXPORT", patch_domain_id):
        if domain_snapshot_provider is None:
            snapshot = build_envelope_analysis_snapshot(
                analysis_bundle,
                included_patch_ids=frozenset({patch_id}),
                profile=profile,
                topology_export=topology_export,
            )
        else:
            snapshot = domain_snapshot_provider(patch_id, patch_domain_id)
        request = build_envelope_decal_request(
            snapshot,
            selected_edges,
            alpha,
            decal_request_id_value=request_id,
            density=density,
        )
    return snapshot, request


def _queue_debug_scene(
    kernel,
    profile,
    domain_id,
    snapshot,
    request,
    prepared_compilation,
    diagnostics,
):
    """Сцена ядра из ОДНОГО плана: очередь не даёт ни союза, ни разрешения.

    Нужна ради кадра патча (`patch_frames`): проекция слоёв очереди поднимает
    точки карты в 3D тем же кадром, что и эталонный путь, а не своим.
    """

    from .envelope_request_export import (
        EnvelopeDebugHostDiagnosticV1,
        EnvelopeDebugHostOutcome,
        EnvelopeDebugHostSeverity,
        _scene_diagnostics,
    )

    with _measure(profile, "DEBUG_SCENE", domain_id):
        try:
            return kernel.build_envelope_debug_scene(
                snapshot,
                request,
                (prepared_compilation,) if prepared_compilation else (),
                (),
                (),
                _scene_diagnostics(kernel, diagnostics),
            ), None
        except (KeyError, TypeError, ValueError) as exc:
            return None, EnvelopeDebugHostDiagnosticV1(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED,
                EnvelopeDebugHostSeverity.ERROR,
                f"DebugScene projection failed without fallback: {exc}",
                domain_id,
            )


def evaluate_envelope_queue_staged(
    analysis_bundle,
    selected_physical_edge_ids: frozenset[int],
    alpha: float,
    *,
    profile=None,
    topology_export=None,
    domain_snapshot_provider=None,
    preparation_provider=None,
    density,
):
    """Движок QUEUE: подготовка плюс покрытие с владельцами, без союза.

    Тот же пролог, что и у эталонного движка (`stage_domain_inputs`), поэтому
    домены нумеруются одинаково и колонки двух движков сравнимы. После пролога
    общего кода нет вовсе: `evaluate_reference_raw_coverage` и
    `resolve_coverage_interactions` здесь не вызываются ни разу.
    """

    from .envelope_debug_profile import (
        EnvelopeDebugProfileBuilderV1,
        EnvelopeDomainStage,
    )
    from .envelope_request_export import (
        EnvelopeDebugDomainEvaluationV1,
        EnvelopeDebugStagedEvaluationV1,
        EnvelopeHostAdapterError,
        _receipt_for_failure,
        _typed_value,
        _load_kernel,
    )
    from .envelope_topology_export import stage_domain_inputs

    if profile is None:
        profile = EnvelopeDebugProfileBuilderV1(
            analysis_bundle.source_revision.source_name,
            ENVELOPE_DEBUG_ENGINE_QUEUE,
        )
    (
        topology_scene,
        revision,
        patch_ids,
        request_id,
        selected_edges_by_domain,
    ) = stage_domain_inputs(
        analysis_bundle,
        selected_physical_edge_ids,
        profile=profile,
        topology_export=topology_export,
    )
    alpha_text = str(float(alpha))
    domain_evaluations = []
    kernel = None
    kernel_failure = None
    try:
        kernel, _ = _load_kernel()
    except EnvelopeHostAdapterError as exc:
        kernel_failure = exc.diagnostic()

    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        if kernel_failure is not None:
            receipt = _receipt_for_failure(
                patch_id,
                domain_id,
                EnvelopeDomainStage.QUEUE_PREPARE_REJECTED,
                kernel_failure,
            )
            profile.set_receipt(receipt)
            domain_evaluations.append(
                EnvelopeDebugDomainEvaluationV1(
                    patch_id,
                    domain_id,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    receipt,
                    (kernel_failure,),
                )
            )
            continue
        domain_evaluations.append(
            _queue_domain_evaluation(
                kernel,
                analysis_bundle,
                patch_id,
                domain_id,
                frozenset(selected_edges_by_domain[domain_id]),
                alpha,
                alpha_text,
                request_id,
                density=density,
                profile=profile,
                topology_export=topology_export,
                domain_snapshot_provider=domain_snapshot_provider,
                preparation_provider=preparation_provider,
            )
        )
    return EnvelopeDebugStagedEvaluationV1(
        topology_scene,
        tuple(domain_evaluations),
    )


def _queue_domain_evaluation(
    kernel,
    analysis_bundle,
    patch_id: int,
    domain_id: str,
    selected_edges: frozenset[int],
    alpha,
    alpha_text: str,
    request_id: str,
    *,
    density,
    profile=None,
    topology_export=None,
    domain_snapshot_provider=None,
    preparation_provider=None,
):
    from .envelope_debug_profile import EnvelopeDomainStage
    from .envelope_request_export import (
        EnvelopeDebugDomainEvaluationV1,
        EnvelopeDebugHostOutcome,
        EnvelopeHostAdapterError,
        _receipt_for_failure,
    )

    try:
        snapshot, request = _queue_snapshot_and_request(
            analysis_bundle,
            patch_id,
            domain_id,
            selected_edges,
            alpha,
            request_id,
            density=density,
            profile=profile,
            topology_export=topology_export,
            domain_snapshot_provider=domain_snapshot_provider,
        )
    except EnvelopeHostAdapterError as exc:
        diagnostic = exc.diagnostic()
        stage = (
            EnvelopeDomainStage.METRIC_REJECTED
            if exc.outcome
            in {
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                EnvelopeDebugHostOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED,
            }
            else EnvelopeDomainStage.QUEUE_PREPARE_REJECTED
        )
        receipt = _receipt_for_failure(patch_id, domain_id, stage, diagnostic)
        profile.set_receipt(receipt)
        return EnvelopeDebugDomainEvaluationV1(
            patch_id,
            domain_id,
            None,
            None,
            None,
            None,
            None,
            None,
            receipt,
            (diagnostic,),
        )

    prepared, queue_domain = run_queue_domain(
        patch_id,
        domain_id,
        snapshot,
        request,
        alpha_text,
        selected_edges=selected_edges,
        profile=profile,
        preparation_provider=preparation_provider,
    )
    diagnostics = list(_queue_diagnostics(queue_domain))
    receipt = _queue_receipt(queue_domain)
    profile.set_receipt(receipt)
    profile.add_timing("QUEUE_CONTOUR", queue_domain.contour_seconds, domain_id)
    for name, value in queue_domain.counters:
        profile.set_counter(name, value, domain_id)
    # Числа стадии контура идут в тот же профиль и тем же способом: разделены
    # они по ПРИНАДЛЕЖНОСТИ, а не по видимости, и прятать измерение хоста от
    # владельца было бы ровно тем тихим слоем, против которого счётчик заведён.
    for name, value in queue_domain.host_counters:
        profile.set_counter(name, value, domain_id)
    compilation = prepared.compilation
    debug_scene, scene_failure = _queue_debug_scene(
        kernel,
        profile,
        domain_id,
        snapshot,
        request,
        compilation,
        diagnostics,
    )
    if scene_failure is not None:
        diagnostics.append(scene_failure)
    return EnvelopeDebugDomainEvaluationV1(
        patch_id,
        domain_id,
        snapshot,
        request,
        compilation,
        None,
        None,
        debug_scene,
        receipt,
        tuple(diagnostics),
        queue_domain,
    )


def queue_timing_text(scene: EnvelopeQueueSceneV1) -> str:
    """Строка владельцу: цена очереди суммарно и у самого дорогого домена.

    Суммы мало: на полевом меше один домен стоит впятеро больше остальных, и по
    одной сумме этого не видно. Полные постадийные числа по каждому домену
    печатает консольный профиль (`QUEUE_PREPARE` / `QUEUE_COVERAGE`).
    """

    text = (
        f"QUEUE {len(scene.domains)} domains: "
        f"prepare {scene.prepare_seconds * 1000.0:.0f} ms, "
        f"coverage {scene.coverage_seconds * 1000.0:.0f} ms"
    )
    if not scene.domains:
        return text
    slowest = max(
        scene.domains,
        key=lambda item: item.prepare_seconds
        + item.coverage_seconds
        + item.contour_seconds,
    )
    total = (
        slowest.prepare_seconds
        + slowest.coverage_seconds
        + slowest.contour_seconds
    )
    return (
        f"{text} | slowest {slowest.patch_domain_id[-3:]} "
        f"{total * 1000.0:.0f} ms"
    )


__all__ = (
    "CONTOUR_MERGED_SAME_CHAIN_GROUPS",
    "CONTOUR_MERGED_SAME_CHAIN_SEPARATORS",
    "CONTOUR_MERGE_BOUNDARY_UNRESOLVED",
    "ENVELOPE_DEBUG_ENGINE_LEGACY",
    "ENVELOPE_DEBUG_ENGINE_QUEUE",
    "EnvelopeQueueCoveredFaceV1",
    "EnvelopeQueueDomainV1",
    "EnvelopeQueueFaceV1",
    "EnvelopeQueueMergeStatsV1",
    "EnvelopeQueueRegionV1",
    "EnvelopeQueueSceneV1",
    "EnvelopeQueueSegmentV1",
    "HOST_CONTOUR_COUNTERS",
    "QUEUE_ENCLOSURE_BITS",
    "QUEUE_LAYER_STYLES",
    "QUEUE_OWNER_COLORS",
    "QUEUE_OWNER_LAYERS",
    "QUEUE_OWNER_LAYER_PREFIX",
    "QUEUE_OWNER_PALETTE_WRAPPED",
    "QUEUE_OWNER_SLOT_COUNT",
    "QUEUE_SKELETON_LAYER",
    "QUEUE_VISIBILITY_PROPERTY",
    "QUEUE_WALL_LAYER",
    "build_queue_domain",
    "build_queue_palette",
    "build_queue_scene",
    "evaluate_envelope_queue_staged",
    "merge_same_chain_faces",
    "queue_domain_payload",
    "queue_scene_payload",
    "queue_timing_text",
    "recompute_queue_coverage",
    "refused_queue_domain",
    "run_queue_domain",
    "source_chain_by_span",
    "sqrt_sum_float",
    "undirected_span",
)
