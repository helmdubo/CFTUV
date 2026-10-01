"""Слияние граней одной цепи на одной прямой: ОДИН код для отладки и продукта.

Код переехал из хоста (`cftuv/envelope_queue_export.py`) сюда без единого
изменения поведения, и это не уборка: слияние решает, какие границы между
гранями считаются границами ДЕКАЛИ, а какие — внутренностью одной полосы. Если
бы отладочная картинка и продуктовый меш склеивали грани двумя разными кодами,
владелец смотрел бы на одну геометрию, а получал другую. Теперь хост зовёт тот
же `merge_same_chain_faces`, что зовёт материализатор.

ПОЧЕМУ ВНУТРЕННИЙ РАЗДЕЛИТЕЛЬ ОДНОЙ ЦЕПИ ПОДАВЛЯЕТСЯ, и это слой ОТОБРАЖЕНИЯ,
а не разбиения. Ядро строит по грани на КАЖДОЕ ребро-источник, и два
коллинеарных ребра одной `PhysicalChain` (полевой случай: `building.004`,
рёбра 18 и 38, общая вершина 6) дают ДВЕ грани, разделённые перпендикулярной
дугой из прямой вершины скелета. Разбиение при этом верное — дуга есть
настоящая граница двух граней, — но контур ПОКРЫТИЯ рисует её как ребро
декали, то есть показывает шов там, где у продукта его нет.

Подавляется ровно внутренняя граница и ровно там, где все четыре условия
доказаны ТОЧНО и целочисленно: один домен-регион, один класс несущей прямой у
рёбер-источников (`integer_line_class` — та же формула, что у
`bridge.line_class`), одна `PhysicalChain` (`stations.source_chain_by_span`) и
совпавшая семантика владельца (оба имени равны, в том числе оба пустые).
Порогов нет ни одного: общий сегмент границы опознаётся ТОЧНЫМ равенством
концов через канонические `SqrtSumV1.terms`.

Тихого исчезновения при этом не появилось, и следов тому три. Дуга по-прежнему
рисуется слоем скелета (он читает ПОЛНЫЕ грани разбиения и слияния не видит
вовсе — роли слоёв разделены). Число подавленных разделителей лежит счётчиком
`CONTOUR_MERGED_SAME_CHAIN_SEPARATORS`. Владельцы слитых граней перечислены
поимённо в `merged_owners` самой записи.

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

Для продукта следствие из последней границы прямое: материализатор
тесселирует каждый слитый контур отсечением ушей БЕЗ дыр, потому что контур
доказанно прост (а грани скелета простые по границе 1 `build_faces`).
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from ..wavefront.coverage import coverage_at
from ..wavefront.faces import contour_crossings


#: Сколько внутренних разделителей стадия контура подавила: каждая пара
#: встречных полурёбер, сокращённая при слиянии граней одной цепи, — единица.
CONTOUR_MERGED_SAME_CHAIN_SEPARATORS = "CONTOUR_MERGED_SAME_CHAIN_SEPARATORS"

#: Сколько ГРУПП граней слилось. Отдельно от числа разделителей, потому что три
#: коллинеарных ребра одной цепи дают одну группу и два разделителя, и по одному
#: числу эти два случая неотличимы.
CONTOUR_MERGED_SAME_CHAIN_GROUPS = "CONTOUR_MERGED_SAME_CHAIN_GROUPS"

#: Сколько групп-кандидатов НЕ слилось: обход объединения не сложился в один
#: цикл либо его контур не прост. Ноль здесь — измерение, а не умолчание: без
#: счётчика «слияний не было» было бы неотличимо от «слияние отказало молча».
CONTOUR_MERGE_BOUNDARY_UNRESOLVED = "CONTOUR_MERGE_BOUNDARY_UNRESOLVED"


@dataclass(frozen=True, slots=True)
class CoveredFaceV1:
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
    #: `PhysicalChain` ребра-источника. Ядру цепь в ключе владельца не нужна
    #: (`EdgeKey` — вхождение отрезка), а домен несёт её в провенансе каждого
    #: сегмента граничной петли. `None` — цепь не названа: скрытая опора веера,
    #: отказавшая выгрузка либо два разных ответа на один решёточный отрезок.
    source_chain_id: str | None = None
    #: Владельцы граней, слитых в ЭТОТ контур. Пусто — грань не сливалась.
    merged_owners: tuple[tuple[int, ...], ...] = ()


@dataclass(frozen=True, slots=True)
class MergeStatsV1:
    """Числа слияния контуров одного региона. Ноль — тоже измерение."""

    merged_separators: int = 0
    merged_groups: int = 0
    unresolved_groups: int = 0

    def __add__(self, other: "MergeStatsV1"):
        return MergeStatsV1(
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


def region_contours(region, lattice_alpha: Fraction, work_budget=None):
    """Усечённые по времени контуры граней региона, в порядке разбиения.

    Тот же вызов, которым `conveyor_coverage` считал площадь, поэтому порядок
    и владельцы совпадают с `ConveyorRegionCoverageV1.faces` по построению, а
    не по совпадению чисел.

    `work_budget` — бюджет ЭКСПОРТА (у хоста) либо материализации (у продукта),
    а не домена. Домен к этому моменту уже ответил, и подмешивать перерисовку
    в его счёт значило бы разрешить картинке превратить состоявшийся ответ в
    отказ. Свой бюджет держит второе обещание: экспорт тоже конечен и кончается
    ИМЕНЕМ, а не счётом.
    """

    if region.partition is None:
        return ()
    return coverage_at(region.partition, lattice_alpha, work_budget).faces


def covered_faces_of_region(covered, contours, spans) -> list[CoveredFaceV1]:
    """Грани региона с точными контурами и именем цепи ребра-источника.

    `covered` — `ConveyorRegionCoverageV1` (владельцы и площади), `contours` —
    `region_contours` того же региона, `spans` — `{отрезок: PhysicalChain}`
    региона из `source_chain_by_span`. Контур сопоставляется грани по ИНДЕКСУ и
    по равенству владельца: несовпавшая либо вырожденная (меньше трёх точек)
    пара пропускается, а не додумывается.
    """

    faces: list[CoveredFaceV1] = []
    for index, named in enumerate(covered.faces):
        if index >= len(contours):
            break
        contour = contours[index]
        if contour.owner != named.owner or len(contour.points) < 3:
            continue
        owner = tuple(int(item) for item in named.owner)
        span = undirected_span(owner)
        faces.append(
            CoveredFaceV1(
                region_id=named.region_id,
                owner=owner,
                envelope_spec_id=str(named.envelope_spec_id),
                envelope_instance_id=named.envelope_instance_id,
                points=tuple(contour.points),
                doubled_area=named.doubled_area,
                source_chain_id=(None if span is None else spans.get(span)),
            )
        )
    return faces


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


def integer_line_class(owner):
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


def point_key(point):
    """Тождество точки контура: канонические наборы её координат.

    `SqrtSumV1` каноничен, поэтому равные величины дают равные `terms`, а
    разные — разные. Тот же ключ служит дедупликации дуг скелета в хосте и
    вершинам материализатора: второго тождества точки в проекте нет.
    """

    return (point[0].terms, point[1].terms)


def _merge_key(face: CoveredFaceV1):
    """Ключ группы слияния либо `None`, если хоть одно условие не доказано.

    Условий четыре: один регион, один класс несущей прямой у рёбер-источников,
    одна `PhysicalChain`, совпавшая семантика владельца. Имя экземпляра входит
    в ключ вместе с именем спеки: экземпляр стоит на ЭФФЕКТИВНОЙ alpha, и две
    грани одной спеки с разными эффективными alpha — разные огибающие, а не
    одна.
    """

    line = integer_line_class(face.owner)
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
        start_key = point_key(points[index])
        end_key = point_key(points[(index + 1) % size])
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
        by_start = {point_key(point): point for point in points}
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

    Возвращает `(грани, MergeStatsV1)`. Порядок сохраняется: слитая запись
    встаёт на место ПЕРВОЙ грани группы, остальные исчезают из списка — но не
    из отчёта, их владельцы перечислены в `merged_owners`.

    Слияние принимается после ДВУХ проверок, и площади среди них нет: она
    выполняется тождественно (сокращение встречных полурёбер вычитает ноль) и
    доказывала бы арифметику вместо сборки. Проверяются обход — один цикл на
    все оставшиеся полурёбра — и ПРОСТОТА полученного контура той же границей
    1 ядра (`contour_crossings`, точный знак `SqrtSumV1`, ни одного порога).
    Группа, не прошедшая их, остаётся неслитой и считается отдельным числом.
    """

    faces = tuple(faces)
    groups: dict[tuple, list[int]] = {}
    for index, face in enumerate(faces):
        key = _merge_key(face)
        if key is None:
            continue
        groups.setdefault(key, []).append(index)

    replacement: dict[int, CoveredFaceV1] = {}
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
            replacement[min(positions)] = CoveredFaceV1(
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
    return merged, MergeStatsV1(separators, merged_groups, unresolved)
