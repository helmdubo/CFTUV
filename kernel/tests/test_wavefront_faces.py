"""Грани скелета: все объявленные границы проверяются на СОБСТВЕННОМ результате.

Правило проекта, поймавшее настоящий дефект пять раз: если функция объявляет
границу — тест обязан проверить, что её собственный результат этой границе
удовлетворяет. `faces.py` объявляет ТРИ границы, и здесь по столбцу на каждую.
Порядок — тот же, в котором их опрашивает сама `build_faces`, от причины к
следствию:

| граница | что проверяется |
|---|---|
| 1. контур каждой грани простой | `every_contour_is_simple` |
| 2. площадь каждой грани строго положительна | `every_face_is_positive` |
| 3. сумма площадей граней = площадь многоугольника | `area_reproduces_polygon` |

С этого среза границы проверяет и САМА `build_faces` перед возвратом `EXACT`, у
каждой границы свой член `FaceOutcome`, и в `detail` лежит ЧИСЛО потери. Ниже —
столбец тестов и на это: что отказ громкий, что число то самое, и что цепочка
до `coverage_at` закрыта (она не считает по неточному разбиению).

Корпус берётся ЦЕЛИКОМ, без единого отбора. Отбор «только фигуры с различными
несущими прямыми» держался на том, что ключом участника была прямая, а не
вхождение ребра; ключ сменился, и `holes_2` — фигура, ради которой отбор
существовал, — проверяется здесь наравне со всеми и отдельным тестом с числами.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.wavefront import build_skeleton
from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.faces import (
    FaceOutcome,
    build_faces,
    check_declared_boundaries,
    contour_crossings,
    doubled_shoelace,
    edge_key,
    line_key,
    orientation,
    polygon_edges,
    segments_cross,
)
from cftuv_envelope.wavefront.polygon import PolygonV1, signed_double_area
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, SplitSearch
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

import wavefront_cases
from wavefront_cases import named_corpus


# Корпус берётся ЦЕЛИКОМ. Отбор «только фигуры с различными несущими прямыми»
# существовал, пока ключом участника была прямая: `holes_2` вылетал из него,
# потому что два ребра соседних дыр коллинеарны и одной длины. Ключ стал
# вхождением ребра, и отбирать больше нечего — отбор ровно и скрывал бы
# починку.
CORPUS = named_corpus()
CORPUS_IDS = tuple(name for name, _ in CORPUS)


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_the_face_partition_reproduces_the_polygon_area_exactly(name, polygon):
    """Граница 3 на собственном результате: площадь сходится ТОЧНО."""

    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    assert partition.area_reproduces_polygon, (
        f"{name}: дефект площади {partition.area_defect.terms}"
    )
    # Не «почти ноль», а пустой канонический набор коэффициентов.
    assert partition.area_defect.terms == ()


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_every_face_of_the_partition_has_strictly_positive_area(name, polygon):
    """Граница 2 на собственном результате: щепок и вывернутых граней нет."""

    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    assert partition.every_face_is_positive
    for face in partition.faces:
        assert face.doubled_area.sign() > 0, f"{name}: грань {face.owner}"


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_every_face_contour_of_the_partition_is_simple(name, polygon):
    """Граница 1 на собственном результате: перекрученных контуров нет.

    Границы 2 и 3 обе читают ПЛОЩАДЬ, а площадь берётся формулой трапеций по
    контуру: перекрути контур — и та же формула посчитает кусок дважды либо с
    обратным знаком. Обе прежние границы такой контур пропускали молча, и
    именно так он и прошёл на полевом патче `bf6`.

    Проверяется не флагом, а ПАРАМИ сегментов: `contour_crossings` возвращает
    список, и пустота списка на каждой грани по отдельности — более сильное
    утверждение, чем «ни одна грань не пожаловалась».
    """

    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    assert partition.every_contour_is_simple
    for face in partition.faces:
        assert contour_crossings(face.points) == (), (
            f"{name}: грань {face.owner}"
        )


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_there_is_exactly_one_face_per_input_edge_and_owner_is_its_span(
    name, polygon
):
    """Owner выпадает из структуры: одна грань на ребро, ключ — его вхождение.

    Вхождение, а не несущая прямая, и разница проверяется здесь же числом: у
    `holes_2` двенадцать рёбер и только десять различных прямых, поэтому
    прежний ключ давал бы десять граней на двенадцать рёбер.
    """

    partition = build_faces(polygon, build_skeleton(polygon))
    edges = polygon_edges(polygon)
    assert len(partition.faces) == len(edges)
    assert {face.owner for face in partition.faces} == {
        edge_key(start, end) for start, end, _ in edges
    }
    assert len({face.owner for face in partition.faces}) == len(edges)


def test_two_collinear_edges_of_one_length_get_two_faces_and_the_area_closes():
    """`holes_2`: та самая фигура, на которой сборщик отказывал. Числа целиком.

    Было: ключом участника служила несущая прямая `(a, b, c, q)`, у рёбер
    соседних дыр она совпадала (обе длины 8), узлы двух РАЗНЫХ граней сливались
    в один список, и наивная сборка давала удвоенную площадь 1584 при истинной
    1344. Отказ `SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES` возвращался до счёта.

    Стало: двенадцать граней на двенадцать рёбер, сумма 1344 из 1344 ТОЧНО, все
    грани строго положительны, владельцы различны. Проверяется не только сумма —
    правило «совпадение суммы не доказывает совпадение множества» здесь и
    записано: сверяется множество владельцев и площадь КАЖДОЙ грани.
    """

    polygon = dict(named_corpus())["holes_2"]
    edges = polygon_edges(polygon)
    lines = [line_key(line) for _, _, line in edges]
    # Болезнь на месте, её просто больше не касается ключ участника.
    assert (len(edges), len(set(lines))) == (12, 10)

    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    assert partition.polygon_doubled_area == 1344
    assert partition.doubled_area.as_rational() == Fraction(1344)
    assert partition.area_defect.terms == ()
    assert partition.every_face_is_positive
    assert len(partition.faces) == 12
    assert len({face.owner for face in partition.faces}) == 12
    # Площадь каждой грани по отдельности, а не только их сумма.
    assert sorted(
        int(face.doubled_area.as_rational()) for face in partition.faces
    ) == [66, 66, 66, 66, 66, 66, 102, 102, 132, 132, 240, 240]


def test_one_edge_given_twice_is_still_a_named_outcome_not_one_shared_face():
    """Два вхождения с одинаковыми концами — исход, а не грань на двоих.

    Проверка не выдумана про запас: `LoopV1` требует различных вершин ВНУТРИ
    петли и ничего не говорит о разных петлях, поэтому вход, задавший одно
    ребро дважды, синтаксически законен. Грань у таких вхождений была бы одна
    на двоих, и молча сложить их значило бы посчитать площадь дважды.
    """

    from dataclasses import replace

    polygon = PolygonV1.build(
        ((0, 0), (12, 0), (12, 12), (0, 12)),
        (((4, 4), (4, 8), (8, 8), (8, 4)),),
    )
    skeleton = build_skeleton(polygon)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    assert build_faces(polygon, skeleton).outcome is FaceOutcome.EXACT

    # Та же дыра, записанная дважды: четыре вхождения повторяются. Скелет
    # берётся от ЗАКОННОЙ фигуры, иначе отказ пришёл бы раньше — из проверки
    # `SKELETON_IS_NOT_EXACT`, — и про вхождения ничего не сказал бы.
    twice = replace(polygon, holes=polygon.holes + (polygon.holes[0],))
    refused = build_faces(twice, skeleton)
    assert refused.outcome is FaceOutcome.TWO_EDGES_SHARE_ONE_SPAN
    assert refused.faces == ()
    assert refused.detail.startswith("4 вхождений")


def test_the_two_split_searches_give_the_same_faces():
    """Оптимизация законна ровно настолько, насколько даёт тот же ответ."""

    for name, polygon in CORPUS:
        motorcycle = build_faces(
            polygon, build_skeleton(polygon, split_search=SplitSearch.MOTORCYCLE)
        )
        exhaustive = build_faces(
            polygon, build_skeleton(polygon, split_search=SplitSearch.EXHAUSTIVE)
        )
        assert motorcycle.outcome is exhaustive.outcome, name
        assert motorcycle.doubled_area == exhaustive.doubled_area, name
        assert {
            face.owner: face.doubled_area for face in motorcycle.faces
        } == {face.owner: face.doubled_area for face in exhaustive.faces}, name


def test_the_faces_of_an_unproven_skeleton_are_refused_not_invented():
    """Скелет не доказан — граней нет. Тихого нуля здесь быть не может."""

    from dataclasses import replace

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    skeleton = build_skeleton(polygon)
    broken = replace(
        skeleton, outcome=type(skeleton.outcome).WAVEFRONT_LEFT_UNRESOLVED
    )
    partition = build_faces(polygon, broken)
    assert partition.outcome is FaceOutcome.SKELETON_IS_NOT_EXACT
    assert partition.faces == ()
    assert partition.detail == "WAVEFRONT_LEFT_UNRESOLVED"


def _skeleton_without_the_ridge_crossing(wide: int, tall: int):
    """Верный скелет креста, из которого ВЫНУТ узел пересечения гребней.

    Своей фигуры, ломающей границы сборки, в корпусе больше нет — и крест
    починен, и порядок сборки заменён на смежность. Но проверять громкость
    отказа надо на настоящей потере, а не на подделанном ярлыке, поэтому потеря
    вносится удалением ровно одного узла: того самого, которого скелету не
    хватало до починки вырожденного события.
    """

    from dataclasses import replace

    figure = wavefront_cases.cross(wide=wide, tall=tall)
    skeleton = build_skeleton(figure)
    crossing = (
        Fraction(CROSS_ARM) + Fraction(wide, 2),
        Fraction(CROSS_ARM) + Fraction(tall, 2),
    )
    kept = tuple(
        node
        for node in skeleton.nodes
        if (node.point.x.as_rational(), node.point.y.as_rational()) != crossing
    )
    assert len(kept) == len(skeleton.nodes) - 1, (wide, tall)
    return figure, replace(skeleton, nodes=kept)


def test_a_missing_skeleton_node_breaks_the_chain_and_is_named_by_its_root():
    """ПОЛОЖИТЕЛЬНЫЙ КОНТРОЛЬ нового исхода: он срабатывает на настоящей потере.

    «Отказ не сработал ни разу» без входа, на котором он срабатывает, ничего не
    значит: так себя ведёт и проверка, которая не работает вовсе. Вход —
    настоящая потеря, а не подделанный ярлык: у верного скелета креста `6 x 4`
    вынут узел пересечения гребней.

    ЧТО ЭТО ИЗМЕНИЛО ПРОТИВ ПРЕЖНЕГО ПРАВИЛА, и это находка, а не косметика.
    Монотонность про отсутствующий узел не знала вовсе: она сортировала то, что
    есть, отдавала грань с дырой и жаловалась ПОЗЖЕ — расхождением суммы на
    −28, то есть СЛЕДСТВИЕМ. Смежность видит корень: у грани ребра
    `(10, 0) -> (10, 4)` цепочка проходит 1 точку из 2, потому что звено,
    связывавшее их, и было вынутым узлом. Число в `detail` показывает, где
    именно обрыв, а не сколько площади в итоге не сошлось.
    """

    figure, wounded = _skeleton_without_the_ridge_crossing(6, 4)
    partition = build_faces(figure, wounded)
    assert partition.outcome is FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE
    assert partition.detail == (
        "ребро (10, 0) -> (10, 4): цепочка 1 точек из 2"
    )
    # Отказ приходит ДО счёта, поэтому граней нет вовсе — их и не из чего
    # собрать: обход не доказан. Это отличает его от трёх границ, где грани
    # посчитаны и без них дефект нечем измерить.
    assert partition.faces == ()
    assert partition.doubled_area.is_zero


def _partition_without_one_face(name: str, owner: tuple[int, int, int, int]):
    """Верное разбиение корпусной фигуры, из которого ВЫНУТА одна грань.

    Так же, как `_skeleton_without_the_ridge_crossing` вносит потерю в скелет,
    здесь потеря вносится в готовое разбиение. Ступень другая, и она нужна
    отдельно: смежность ловит потерю УЗЛА своим исходом ещё до счёта площадей,
    поэтому живого входа для границы 3 в скелете больше нет, а сама граница
    остаётся и обязана быть проверена настоящей потерей, а не ярлыком.

    Фигура — из корпуса, грань называется своим owner'ом, и величина потери
    берётся у самой грани, а не вписывается числом.
    """

    from dataclasses import replace

    polygon = dict(named_corpus())[name]
    intact = build_faces(polygon, build_skeleton(polygon))
    assert intact.outcome is FaceOutcome.EXACT
    lost = next(face for face in intact.faces if face.owner == owner)
    kept = tuple(face for face in intact.faces if face.owner != owner)
    assert len(kept) == len(intact.faces) - 1
    total = SqrtSumV1.zero()
    for face in kept:
        total = total + face.doubled_area
    return lost, check_declared_boundaries(
        replace(intact, faces=kept, doubled_area=total)
    )


def test_a_partition_that_loses_area_refuses_loudly_with_the_defect_as_a_number():
    """Граница 3 нарушена — исход названный, и в нём ЧИСЛО, а не слово.

    До появления этой границы потеря была ТИХОЙ: исход говорил `EXACT`,
    граница 2 держалась, а односторонняя проверка покрытия «не превысил
    многоугольник» проходила, потому что потеря делает покрытие МЕНЬШЕ.

    Потеря вносится вынутой гранью, и величина её не выдумана: дефект обязан
    совпасть с площадью ровно той грани, которой не стало. Проверяется не
    «около», а `is_zero` разности.
    """

    lost, partition = _partition_without_one_face("cross", (4, 4, 4, 0))
    assert partition.outcome is (
        FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON
    )
    assert lost.doubled_area.as_rational() == Fraction(23)
    assert partition.detail == "-23"
    assert partition.area_defect.as_rational() == Fraction(-23)
    assert (partition.area_defect + lost.doubled_area).is_zero
    # Границы 1 и 2 при этом ДЕРЖАТСЯ, и это ровно то, почему исходы разные:
    # один общий член слил бы «потерян клин», «грань вывернута» и «контур
    # перекручен» в одно слово. Потеря клина — единственная из трёх, которая
    # НЕ трогает порядок обхода внутри оставшихся граней.
    assert partition.every_face_is_positive
    assert partition.every_contour_is_simple


def _partition_with_one_twisted_face(name: str, owner: tuple[int, int, int, int]):
    """Верное разбиение корпусной фигуры с ОДНОЙ доказуемо перекрученной гранью.

    Перекрут не подделывается ярлыком и не подбирается: у грани-ЧЕТЫРЁХУГОЛЬНИКА
    переставляются местами две последние точки. Для ВЫПУКЛОГО четырёхугольника
    `A B C D` порядок `A B D C` даёт сегменты `B->D` и `C->A`, то есть ровно две
    его диагонали, а диагонали выпуклого четырёхугольника пересекаются
    трансверсально всегда. Выпуклость проверяется здесь же четырьмя знаками
    ориентации, поэтому «перекрутилось» — доказанное следствие, а не наблюдение.

    Своей фигуры, ломающей границу 1, в корпусе больше нет: остаток креста
    закрыт правилом смежности. Граница от этого не снимается, и вход ей даёт
    корпусная фигура, испорченная известным способом.
    """

    from dataclasses import replace

    polygon = dict(named_corpus())[name]
    intact = build_faces(polygon, build_skeleton(polygon))
    assert intact.outcome is FaceOutcome.EXACT
    face = next(item for item in intact.faces if item.owner == owner)
    points = face.points
    assert len(points) == 4
    # Выпуклость: все четыре тройки подряд идущих вершин крутят в одну сторону.
    assert [
        orientation(points[i], points[(i + 1) % 4], points[(i + 2) % 4])
        for i in range(4)
    ] == [1, 1, 1, 1]

    twisted_points = points[:2] + (points[3], points[2])
    twisted = replace(
        face,
        points=twisted_points,
        doubled_area=doubled_shoelace(twisted_points),
    )
    faces = tuple(
        twisted if item.owner == owner else item for item in intact.faces
    )
    total = SqrtSumV1.zero()
    for item in faces:
        total = total + item.doubled_area
    return twisted, check_declared_boundaries(
        replace(intact, faces=faces, doubled_area=total)
    )


def test_a_twisted_contour_refuses_loudly_with_the_crossing_pairs_as_numbers():
    """Граница 1 нарушена — исход названный, и в нём ПАРЫ СЕГМЕНТОВ.

    Вход берётся из корпуса и не строится по памяти: `hole_1`, грань
    ребра-дыры `(6,14) -> (14,14)` — выпуклая трапеция удвоенной площади 66.
    Переставленная в «бабочку», она даёт РОВНО одно трансверсальное
    пересечение и удвоенную площадь −18: формула трапеций на перекрученном
    контуре считает один лепесток со знаком минус.

    Пары в `detail` — не украшение. Без них «контур не простой» не отличает
    один перекрут от трёх и не показывает, какой кусок цепочки поставлен не
    туда; на `bf6` починку искали ровно по этим парам.
    """

    twisted, partition = _partition_with_one_twisted_face(
        "hole_1", (6, 14, 14, 14)
    )
    assert partition.outcome is FaceOutcome.FACE_CONTOUR_IS_NOT_SIMPLE
    # Число пересечений стоит в `detail` числом, а не словом «нарушено».
    assert partition.detail == (
        "1 пересечений на 1 гранях из 8, пары сегментов: "
        "(6, 14, 14, 14): [(1, 3)]"
    )
    guilty = [
        face for face in partition.faces if contour_crossings(face.points)
    ]
    assert len(guilty) == 1
    assert guilty[0].owner == twisted.owner
    assert contour_crossings(guilty[0].points) == ((1, 3),)
    # Грани НЕ обнуляются: дефект без них нечем измерить.
    assert len(partition.faces) == 8
    # Перекрут потянул за собой и обе остальные границы, и это не совпадение:
    # площадь считается по тому же контуру.
    assert not partition.every_face_is_positive
    assert twisted.doubled_area.as_rational() == Fraction(-18)
    assert partition.area_defect.as_rational() == Fraction(-84)


def test_the_twisted_contour_is_named_before_the_face_it_makes_negative():
    """Порядок опроса границ доказан ПАРОЙ ВХОДОВ, а не рассуждением.

    Утверждение: самопересечение — причина, неположительность — следствие, и
    поэтому первым называется самопересечение. Доказывается тем, что связь
    односторонняя:

    - есть вход, где нарушены обе границы (перекрученная грань `hole_1`), и там
      называется именно перекрут;
    - есть вход, где нарушена ТОЛЬКО граница 2 (подделанная площадь при целом
      контуре), и там называется неположительность.

    Второй вход подделывается ярлыком осознанно: грань с ЦЕЛЫМ контуром и
    неположительной площадью не рождается ни на одной фигуре корпуса, а
    проверить надо, что члены два и они не подменяют друг друга.

    Если бы порядок был обратным, первый вход отвечал бы следствием, а корень
    пришлось бы искать руками — ровно то, что и случилось на `bf6`, где
    названная сумма не давала прочесть причину.
    """

    from dataclasses import replace

    _, broken_both = _partition_with_one_twisted_face("hole_1", (6, 14, 14, 14))
    assert not broken_both.every_contour_is_simple
    assert not broken_both.every_face_is_positive
    assert broken_both.outcome is FaceOutcome.FACE_CONTOUR_IS_NOT_SIMPLE

    polygon = dict(named_corpus())["axis_square"]
    intact = build_faces(polygon, build_skeleton(polygon))
    assert intact.outcome is FaceOutcome.EXACT
    flipped = replace(intact.faces[0], doubled_area=SqrtSumV1.rational(-32))
    only_negative = check_declared_boundaries(
        replace(intact, faces=(flipped,) + intact.faces[1:])
    )
    assert only_negative.every_contour_is_simple
    assert not only_negative.every_face_is_positive
    assert only_negative.outcome is FaceOutcome.FACE_IS_NOT_POSITIVE


def test_the_simplicity_boundary_is_transversal_crossing_and_here_is_why():
    """Почему объявлена трансверсальность, а не «нет общих точек» вовсе.

    Более строгий вариант отверг бы ВЕРНЫЕ ответы, и это измерено на корпусе
    частичного источника: у пяти его фигур с `EXACT` и точно сходящейся
    площадью два узла скелета стоят в ОДНОЙ точке, отчего в контуре появляется
    сегмент нулевой длины. Он ни с чем не пересекается трансверсально и площадь
    не двигает — но «никаких общих точек» на нём срабатывает.

    Граница, отвергающая верный ответ, — не граница. Поэтому объявлено ровно
    то, что портит площадь.

    Проверяется здесь ДВУМЯ утверждениями сразу: что вырожденный сегмент
    предикат не считает пересечением, и что настоящий крест — считает.
    """

    def point(x, y):
        return (SqrtSumV1.rational(x), SqrtSumV1.rational(y))

    # Настоящий крест: диагонали единичного квадрата.
    assert segments_cross(
        point(0, 0), point(4, 4), point(0, 4), point(4, 0)
    )
    # Касание концом — не пересечение.
    assert not segments_cross(
        point(0, 0), point(4, 4), point(4, 4), point(8, 0)
    )
    # Конец одного строго ВНУТРИ другого — тоже не трансверсальность.
    assert not segments_cross(
        point(0, 0), point(4, 0), point(2, 0), point(2, 4)
    )
    # Сегмент нулевой длины — тот самый случай из корпуса частичного источника.
    assert not segments_cross(
        point(2, 2), point(2, 2), point(0, 0), point(4, 4)
    )
    # Коллинеарное наложение — не трансверсальность.
    assert not segments_cross(
        point(0, 0), point(4, 0), point(2, 0), point(6, 0)
    )
    # Предикат ориентации — на ИРРАЦИОНАЛЬНЫХ координатах тоже, иначе
    # «точно» проверялось бы только на решётке.
    root = SqrtSumV1.radical(2, Fraction(1))
    assert orientation(point(0, 0), (root, root), (root, SqrtSumV1.zero())) < 0
    assert orientation(point(0, 0), (root, root), (SqrtSumV1.zero(), root)) > 0
    assert orientation(point(0, 0), (root, root), (root, root)) == 0


def test_the_two_broken_boundaries_are_two_different_outcomes_not_one():
    """Отдельный член на каждую границу — иначе диагностика их путает.

    Вывернутая грань подделывается `replace` над готовой партицией: своей
    фигуры, ломающей ИМЕННО границу 2, в корпусе нет, а проверить надо, что
    членов два и они не подменяют друг друга.
    """

    from dataclasses import replace

    assert (
        FaceOutcome.FACE_IS_NOT_POSITIVE
        is not FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON
    )
    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    flipped = replace(
        partition.faces[0], doubled_area=SqrtSumV1.rational(-32)
    )
    broken = replace(partition, faces=(flipped,) + partition.faces[1:])
    assert not broken.every_face_is_positive
    # Свойство границы 1 у подделки тоже сломано, и это как раз показывает,
    # почему `build_faces` спрашивает про положительность ПЕРВОЙ: корень —
    # одна грань, а расхождение суммы — его следствие.
    assert broken.area_reproduces_polygon


def test_an_inexact_partition_never_reaches_the_coverage_answer():
    """Цепочка закрыта ДОКАЗАННО: `coverage_at` отказывает, а не считает.

    Проверяется на настоящей потере площади, а не на подделанном ярлыке: из
    верного разбиения корпусного креста `6 x 4` вынута грань стенки, удвоенный
    дефект −23. Раньше такая потеря протекала до самого ответа.
    """

    _, partition = _partition_without_one_face("cross", (4, 4, 4, 0))
    assert partition.outcome is (
        FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON
    )
    for alpha in (Fraction(0), Fraction(1), Fraction(3)):
        coverage = coverage_at(partition, alpha)
        assert coverage.outcome is CoverageOutcome.PARTITION_IS_NOT_EXACT
        assert coverage.faces == ()
        assert coverage.doubled_area.is_zero
        assert coverage.detail == (
            FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON.value
        )


def test_the_shoelace_over_sqrt_sums_agrees_with_the_integer_one():
    """Площадь над `SqrtSumV1` — та же формула, а не вторая арифметика."""

    for points in (
        ((0, 0), (4, 0), (4, 3), (0, 3)),
        ((0, 0), (7, 0), (3, 5)),
        ((-2, -2), (5, -1), (4, 6), (-3, 3)),
    ):
        lifted = tuple(
            (SqrtSumV1.rational(x), SqrtSumV1.rational(y)) for x, y in points
        )
        assert doubled_shoelace(lifted).as_rational() == Fraction(
            signed_double_area(points)
        )


def test_the_face_area_of_the_axis_square_is_the_quarter_it_must_be():
    """Известный ответ руками: квадрат 8x8 даёт четыре равные грани по 16."""

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    assert [
        face.doubled_area.as_rational() for face in partition.faces
    ] == [Fraction(32)] * 4
    assert partition.doubled_area.as_rational() == Fraction(128)


# --------------------------------------------------------------------------
# Крест: сетка 3..9 целиком, ПОСЛЕ починки вырожденного события
#
# Крест был отрицательным контролем среза и остаётся им: на нём сборщик
# ломался двумя разными способами. Оба корня оказались одним — вогнутая
# вершина, пришедшая не в ребро, а в ВЕРШИНУ фронта, разбиралась как разрез.
#
# | версия причины                     | измерение | вывод |
# |------------------------------------|-----------|-------|
# | «узлы с равной проекцией, порядок» | 0 пар различных точек с равной проекцией ДО починки | ОПРОВЕРГНУТА |
# | «нужной вершины НЕТ среди узлов»   | пересечения гребней не было ни на одной из 42 строк | ПОДТВЕРЖДЕНА |
# | «дефект = -(wide - tall)^2»        | вынуть один узел из ВЕРНОГО скелета — закон не воспроизводится | ОПРОВЕРГНУТА |
#
# Последняя строка важна для протокола: прежний закон описывал не «площадь
# недостающего клина», а СУММУ двух ошибок сразу — недостающего узла и двух
# лишних. Удаление одного узла из верного скелета даёт другие числа (−28 на
# кресте 6x4 против прежних −4), и это доказывает, что скелет отличался от
# верного тремя узлами, а не одним.
# --------------------------------------------------------------------------


# Толщина рукавов у `wavefront_cases.cross` по умолчанию.
CROSS_ARM = 4

# Четыре стенки центрального блока и четыре его «крышки» в порядке рёбер
# `polygon_edges`. Пересечение гребней сидит между ними.
CROSS_WALLS = (1, 5, 7, 11)
CROSS_CAPS = (2, 4, 8, 10)


def _cross_partition(wide: int, tall: int):
    figure = wavefront_cases.cross(wide=wide, tall=tall)
    skeleton = build_skeleton(figure)
    return figure, skeleton, build_faces(figure, skeleton)


def test_the_whole_cross_grid_reproduces_the_polygon_area_without_exception():
    """КРИТЕРИЙ ГОТОВНОСТИ, и он неподгоняемый: все 49 строк сетки 3..9.

    Не 42 и не «те, что попали в тест». Диагональ `wide == tall`, которая
    отказывала `WAVEFRONT_LEFT_UNRESOLVED`, сюда входит наравне со всеми: в её
    вырожденной точке встречаются сразу четыре вогнутые вершины, и правило
    сшивки по лучам разбирает эту встречу тем же способом, что и встречу двух.
    """

    measured = []
    for wide in range(3, 10):
        for tall in range(3, 10):
            _, skeleton, partition = _cross_partition(wide, tall)
            assert skeleton.outcome is SkeletonOutcome.EXACT, (wide, tall)
            measured.append(
                (wide, tall, partition.outcome, partition.area_defect.as_rational())
            )
    assert len(measured) == 49
    assert all(
        outcome is FaceOutcome.EXACT and defect == 0
        for _, _, outcome, defect in measured
    )


def test_the_square_block_of_a_cross_is_one_meeting_of_four_vertices():
    """Диагональ `wide == tall`: чем именно она отличается, числом.

    При `wide != tall` вырожденных точек ДВЕ и в каждой встречаются по две
    вогнутые вершины. При `wide == tall` она ОДНА и вершин в ней четыре, а
    узел, который она даёт, несёт ВОСЕМЬ участников — все четыре стенки блока и
    все четыре его крышки сразу. Ни один попарный разбор такого узла не даёт.
    """

    for side in range(3, 10):
        _, skeleton, partition = _cross_partition(side, side)
        assert partition.outcome is FaceOutcome.EXACT, side
        assert skeleton.counter("vertex_meeting_events") == 1, side
        centre = (
            Fraction(CROSS_ARM) + Fraction(side, 2),
            Fraction(CROSS_ARM) + Fraction(side, 2),
        )
        crossing = [
            node
            for node in skeleton.nodes
            if (node.point.x.as_rational(), node.point.y.as_rational()) == centre
        ]
        assert len(crossing) == 1, side
        assert len(crossing[0].participants) == 8, side
        assert crossing[0].converging_vertices == 4, side


def test_the_ridge_crossing_is_emitted_and_its_participants_say_which_ridge():
    """То, чего не хватало: узел пересечения гребней, и участники в нём.

    Раньше этой точки не было среди узлов ни на одной из 42 недиагональных
    строк, и сборщику было нечем собрать грань. Теперь она есть, и её участники
    называют, какой гребень пришёл последним: при `wide > tall` первой рушится
    горизонтальная полоса, и в пересечении встречаются четыре СТЕНКИ блока; при
    `wide < tall` — зеркально, четыре его КРЫШКИ. Ровно эти четыре грани и
    обрывались без узла.
    """

    for wide in range(3, 10):
        for tall in range(3, 10):
            if wide == tall:
                continue
            figure, skeleton, _ = _cross_partition(wide, tall)
            centre = (
                Fraction(CROSS_ARM) + Fraction(wide, 2),
                Fraction(CROSS_ARM) + Fraction(tall, 2),
            )
            crossing = [
                node
                for node in skeleton.nodes
                if (node.point.x.as_rational(), node.point.y.as_rational())
                == centre
            ]
            assert len(crossing) == 1, (wide, tall)
            late = CROSS_WALLS if wide > tall else CROSS_CAPS
            expected = {
                edge_key(start, end)
                for index, (start, end, _) in enumerate(polygon_edges(figure))
                if index in late
            }
            assert set(crossing[0].participants) == expected, (wide, tall)
            assert crossing[0].converging_vertices == 2, (wide, tall)


def _projection(point, dx: int, dy: int) -> SqrtSumV1:
    """Проекция точки на направление `(dx, dy)`. Без нормировки: знак важен.

    Живёт в стенде, а не в ядре, и это следствие постановки правила смежности:
    порядок сборки стал комбинаторным и проекций не спрашивает вовсе. Величина
    осталась нужна ровно здесь — чтобы измерить конфигурацию скелета, на которой
    прежнее правило и ломалось.
    """

    return point[0].scaled(dx) + point[1].scaled(dy)


def test_two_distinct_skeleton_points_of_a_cross_DO_share_a_projection_now():
    """Ничьи по проекции ПОЯВИЛИСЬ, и поставленный порядок их больше не решает.

    До починки вырожденной точки пар различных точек с равной проекцией не было
    ни одной — это и опровергло гипотезу «ломается сортировка». Теперь они есть,
    и появились они не случайно: дуга от угла блока к пересечению гребней
    ПЕРПЕНДИКУЛЯРНА стенке, поэтому оба её конца проецируются на стенку в одну
    точку.

    Разрешить такую ничью одним знаком нельзя: тот же перпендикулярный кусок
    входит в грани двух разных стенок и обходится в них в разном порядке.
    Прежнее правило разрешало её унимодальностью глубины и на части сетки всё
    равно ошибалось; поставленное правило СМЕЖНОСТИ проекций не спрашивает
    вовсе, и ничья для него не событие.

    Измерение остаётся ратчетом на форму скелета: ничьи есть, их ровно 42 * 4,
    и площадь при этом сходится точно на каждой строке.
    """

    ties = 0
    for wide in range(3, 10):
        for tall in range(3, 10):
            figure, skeleton, partition = _cross_partition(wide, tall)
            assert partition.outcome is FaceOutcome.EXACT, (wide, tall)
            for index, (start, end, _) in enumerate(polygon_edges(figure)):
                key = edge_key(start, end)
                dx, dy = end[0] - start[0], end[1] - start[1]
                buckets: dict[object, set] = {}
                for node in skeleton.nodes:
                    if key not in node.participants:
                        continue
                    point = (node.point.x, node.point.y)
                    buckets.setdefault(
                        _projection(point, dx, dy), set()
                    ).add(point)
                ties += sum(len(places) - 1 for places in buckets.values())
    # Ровно по одной ничьей на каждую из четырёх граней, которые замыкает
    # пересечение гребней, на каждой из 42 недиагональных строк. На диагонали
    # ничьих нет: там гребни приходят в точку одновременно, и перпендикулярной
    # дуги между углом блока и пересечением просто не возникает.
    assert ties == 42 * 4


def test_the_face_of_a_wall_is_the_pentagon_the_ridge_crossing_closes():
    """Известный ответ РУКАМИ на одной строке, а не только сумма по сетке.

    Крест `6 x 4`: стенка `(4,4) -> (4,0)` — нижняя половина левой стенки
    блока. Её грань замыкается пятиугольником `(4,4) (4,0) (7,3) (7,6) (6,6)`
    удвоенной площади 23. Сумма по всем двенадцати граням равна 336, то есть
    удвоенной площади фигуры `120 + 72 - 24 = 168`.
    """

    figure, _, partition = _cross_partition(6, 4)
    wall = partition.faces[11]
    assert wall.source_start == (4, 4) and wall.source_end == (4, 0)
    assert [
        (x.as_rational(), y.as_rational()) for x, y in wall.points
    ] == [(4, 4), (4, 0), (7, 3), (7, 6), (6, 6)]
    assert wall.doubled_area.as_rational() == Fraction(23)
    assert partition.doubled_area.as_rational() == Fraction(336)


@pytest.mark.parametrize("arm", (2, 4, 6))
def test_the_repair_is_not_tuned_to_one_arm_thickness(arm: int):
    """Починка проверена на ТРЁХ толщинах рукава, а не на одной.

    Толщина рукава — тот самый параметр, на котором обжёгся протокол: прежний
    закон дефекта был записан без своей области именно потому, что мерился на
    самодельном кресте с рукавом 6. Поэтому фигура берётся из корпуса, толщина
    задаётся её собственным параметром, и сетка берётся шире объявленной.

    ИСТОРИЯ ОТКАЗОВ, и она сохраняется текстом, потому что каждая её строка
    оплачена отдельной починкой:

    | рукав | до починки вырожденной точки | после неё | пропускалось | после смены ключа | остаток | сейчас |
    |---|---:|---:|---:|---:|---:|---:|
    | 2 | 0 | 51 | 0 | 51 | 30 | **81** |
    | 4 | 0 | 70 | 9 | 79 | 2 | **81** |
    | 6 | 0 | 64 | 17 | 81 | 0 | **81** |

    Пропущенные строки — те, где у фигуры два ребра делили несущую прямую;
    считаются они по-прежнему отдельно, чтобы «стало больше» не сводилось к
    «стало считаться больше строк».

    ОСТАТОК ЗАКРЫТ ПРАВИЛОМ СМЕЖНОСТИ, и это его первое число. Он начинался
    ровно там, где `|wide - tall|` достигает `2 * arm`, и объявлялся ДРУГОЙ
    вырожденной конфигурацией. Конфигурация оказалась той же самой: вся дальняя
    граница грани там перпендикулярна ребру, монотонность переворачивала обход,
    и контур перекручивался. Смежность проекций не спрашивает вовсе, поэтому
    переворачивать нечего — 32 строки остатка отвечают `EXACT`, и остатка нет.

    Чем именно отказывал остаток — тоже история, и она была находкой: сперва
    `FACE_IS_NOT_POSITIVE`, с появлением границы 1 переименован в
    `FACE_CONTOUR_IS_NOT_SIMPLE`, потому что вывернутость была СЛЕДСТВИЕМ.
    Переименование и назвало корень заменяемым.
    """

    exact = 0
    shared_line = 0
    beyond = 0
    for wide in range(3, 12):
        for tall in range(3, 12):
            figure = wavefront_cases.cross(wide=wide, tall=tall, arm=arm)
            edges = polygon_edges(figure)
            lines = [line_key(line) for _, _, line in edges]
            if len(set(lines)) != len(lines):
                shared_line += 1
            spans = [edge_key(start, end) for start, end, _ in edges]
            assert len(set(spans)) == len(spans), (arm, wide, tall)
            partition = build_faces(figure, build_skeleton(figure))
            if partition.outcome is FaceOutcome.EXACT:
                assert partition.area_defect.is_zero, (arm, wide, tall)
                # Верная строка обязана быть простой и положительной ПО КАЖДОЙ
                # грани, а не в среднем: границы проверяются здесь же на
                # собственном результате функции.
                assert partition.every_contour_is_simple, (arm, wide, tall)
                assert partition.every_face_is_positive, (arm, wide, tall)
                exact += 1
            else:
                beyond += 1
                assert False, (arm, wide, tall, partition.outcome, partition.detail)
    assert (exact, shared_line, beyond) == {
        2: (81, 0, 0),
        4: (81, 9, 0),
        6: (81, 17, 0),
    }[arm]


def test_the_adjacency_order_closes_the_whole_cross_grid_including_the_remainder():
    """КРИТЕРИЙ ГОТОВНОСТИ поставленного правила: 243 строки сетки, ни одной мимо.

    Сетка `wide, tall` из 3..11 по трём толщинам рукава берётся ЦЕЛИКОМ, без
    единого отбора, и считается один раз на все три утверждения сразу:

    | что | число |
    |---|---:|
    | `EXACT` по толщинам 2 / 4 / 6 | 81 / 81 / 81 |
    | из них строки бывшего остатка `|wide - tall| >= 2 * arm` | 30 / 2 / 0, всего **32** |
    | строки, где два ребра делят несущую прямую | 0 / 9 / 17 |

    Бывший остаток считается ОТДЕЛЬНО и по своему признаку, а не «те, что
    раньше падали»: признак `|wide - tall| >= 2 * arm` — свойство входа, и
    список строк из него следует, а не вспоминается. Раньше эти 32 строки
    отвечали `FACE_CONTOUR_IS_NOT_SIMPLE`; теперь каждая из них проверяется на
    все три границы по отдельности.

    Строки с общей несущей прямой считаются отдельно по прежней причине: они
    когда-то ПРОПУСКАЛИСЬ отбором, и без их счёта «сетка целиком» не отличалось
    бы от «сетка, из которой выкинуто семнадцать строк».
    """

    exact = {2: 0, 4: 0, 6: 0}
    remainder = {2: 0, 4: 0, 6: 0}
    shared_line = {2: 0, 4: 0, 6: 0}
    rows = 0
    for arm in (2, 4, 6):
        for wide in range(3, 12):
            for tall in range(3, 12):
                rows += 1
                figure = wavefront_cases.cross(wide=wide, tall=tall, arm=arm)
                lines = [
                    line_key(line) for _, _, line in polygon_edges(figure)
                ]
                if len(set(lines)) != len(lines):
                    shared_line[arm] += 1
                partition = build_faces(figure, build_skeleton(figure))
                assert partition.outcome is FaceOutcome.EXACT, (
                    arm,
                    wide,
                    tall,
                    partition.detail,
                )
                assert partition.every_contour_is_simple, (arm, wide, tall)
                assert partition.every_face_is_positive, (arm, wide, tall)
                assert partition.area_defect.is_zero, (arm, wide, tall)
                exact[arm] += 1
                if abs(wide - tall) >= 2 * arm:
                    remainder[arm] += 1
    assert rows == 243
    assert exact == {2: 81, 4: 81, 6: 81}
    assert remainder == {2: 30, 4: 2, 6: 0}
    assert sum(remainder.values()) == 32
    assert shared_line == {2: 0, 4: 9, 6: 17}


def test_the_chain_that_does_not_close_never_fires_on_any_corpus():
    """Новый исход не срабатывает НИ РАЗУ, и ноль этот заморожен, а не подразумеван.

    Правило смежности поставлено без тихого запасного порядка: цепочка либо
    сложилась, либо отказ `FACE_CHAIN_DOES_NOT_CLOSE`. Такое устройство имеет
    цену, и цена измеряется здесь — на каждом входе, который у проекта есть.

    | корпус | разбиений | отказов цепочки |
    |---|---:|---:|
    | `named_corpus()` | 22 | 0 |
    | сетка крестов 3..11 x рукава 2, 4, 6 | 243 | 0 |
    | корпус частичного источника (без фигур с недоказанным скелетом) | 24 | 0 |

    Ноль сам по себе ничего не доказывает — так же выглядит и проверка, которая
    не работает вовсе. Что она работает, показывает
    `test_a_missing_skeleton_node_breaks_the_chain_and_is_named_by_its_root`:
    на настоящей потере узла исход срабатывает и называет место обрыва числом.
    """

    from wavefront_cases import partial_source_corpus

    counted = {"named": 0, "grid": 0, "partial": 0}
    refusals = 0
    for _, polygon in named_corpus():
        partition = build_faces(polygon, build_skeleton(polygon))
        counted["named"] += 1
        refusals += partition.outcome is FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE
    for arm in (2, 4, 6):
        for wide in range(3, 12):
            for tall in range(3, 12):
                figure = wavefront_cases.cross(wide=wide, tall=tall, arm=arm)
                partition = build_faces(figure, build_skeleton(figure))
                counted["grid"] += 1
                refusals += partition.outcome is (
                    FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE
                )
    for _, polygon in partial_source_corpus():
        skeleton = build_skeleton(polygon)
        if skeleton.outcome is not SkeletonOutcome.EXACT:
            # Скелета нет — цепочку строить не из чего, и сборщик до неё не
            # доходит. Считать такие фигуры значило бы записать в ноль отказы,
            # которых никто не спрашивал.
            continue
        partition = build_faces(polygon, skeleton)
        counted["partial"] += 1
        refusals += partition.outcome is FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE
    assert counted == {"named": 22, "grid": 243, "partial": 24}
    assert refusals == 0
