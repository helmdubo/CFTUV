"""Грани скелета: обе объявленные границы проверяются на СОБСТВЕННОМ результате.

Правило проекта, поймавшее настоящий дефект пять раз: если функция объявляет
границу — тест обязан проверить, что её собственный результат этой границе
удовлетворяет. `faces.py` объявляет ровно две границы, и здесь по столбцу на
каждую:

| граница | что проверяется |
|---|---|
| 1. сумма площадей граней = площадь многоугольника | `area_reproduces_polygon` |
| 2. площадь каждой грани строго положительна | `every_face_is_positive` |

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
    doubled_shoelace,
    edge_key,
    face_contour,
    line_key,
    polygon_edges,
    projection,
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
    """Граница 1 на собственном результате: площадь сходится ТОЧНО."""

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

    Своей фигуры, ломающей именно границу 1, в корпусе больше нет — крест
    починен. Но проверять громкость отказа надо на настоящей потере, а не на
    подделанном ярлыке, поэтому потеря вносится удалением ровно одного узла:
    того самого, которого скелету не хватало до этого среза.
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


def test_a_partition_that_loses_area_refuses_loudly_with_the_defect_as_a_number():
    """Граница 1 нарушена — исход названный, и в нём ЧИСЛО, а не слово.

    До этого среза потеря была ТИХОЙ: исход говорил `EXACT`, граница 2
    держалась, а односторонняя проверка покрытия «не превысил многоугольник»
    проходила, потому что потеря делает покрытие МЕНЬШЕ.
    """

    figure, wounded = _skeleton_without_the_ridge_crossing(6, 4)
    partition = build_faces(figure, wounded)
    assert partition.outcome is (
        FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON
    )
    assert partition.detail == "-28"
    assert partition.area_defect.as_rational() == Fraction(-28)
    # Граница 2 при этом ДЕРЖИТСЯ, и это ровно то, почему исходы разные: один
    # общий член слил бы «потерян клин» и «грань вывернута» в одно слово.
    assert partition.every_face_is_positive


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

    Проверяется на настоящей потере площади, а не на подделанном ярлыке: у
    верного скелета креста `6 x 4` вынут узел пересечения гребней, удвоенный
    дефект −28. Раньше такая потеря протекала до самого ответа.
    """

    figure, wounded = _skeleton_without_the_ridge_crossing(6, 4)
    partition = build_faces(figure, wounded)
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


def test_two_distinct_skeleton_points_of_a_cross_DO_share_a_projection_now():
    """Ничьи по проекции ПОЯВИЛИСЬ, и порядок в них решает глубина.

    До починки пар различных точек с равной проекцией не было ни одной — это и
    опровергло гипотезу «ломается сортировка». Теперь они есть, и появились они
    не случайно: дуга от угла блока к пересечению гребней ПЕРПЕНДИКУЛЯРНА
    стенке, поэтому оба её конца проецируются на стенку в одну точку.

    Разрешить такую ничью одним знаком нельзя: тот же перпендикулярный кусок
    входит в грани двух разных стенок и обходится в них в разном порядке. Здесь
    проверяется, что ничьи есть и что при этом площадь сходится точно, — то
    есть что правило унимодальности глубины их разрешает верно.
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
                    buckets.setdefault(projection(point, dx, dy), set()).add(point)
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

    Числа ДО починки вырожденной точки: ноль верных строк на каждой из трёх
    толщин (81 из 81 сломано). Числа ПОСЛЕ неё — 51 / 70 / 64, и ещё 0 / 9 / 17
    строк ПРОПУСКАЛИСЬ, потому что у фигуры два ребра делили несущую прямую.

    Со сменой ключа участника на вхождение ребра пропускать больше нечего, и
    пропущенные строки оказались верными: 51 / 79 / 81. У остатка граница
    названа и не сдвинулась — он начинается ровно там, где `|wide - tall|`
    достигает `2 * arm`. Это ДРУГАЯ вырожденная конфигурация, а не недоделанная
    эта, и её отказ громкий (`FACE_IS_NOT_POSITIVE`).

    | рукав | было EXACT | пропускалось | стало EXACT | остаток |
    |---|---:|---:|---:|---:|
    | 2 | 51 | 0 | 51 | 30 |
    | 4 | 70 | 9 | 79 | 2 |
    | 6 | 64 | 17 | 81 | 0 |
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
                # Строка, которая раньше ПРОПУСКАЛАСЬ. Считается отдельно, чтобы
                # «стало больше» не сводилось к «стало считаться больше строк».
                shared_line += 1
            spans = [edge_key(start, end) for start, end, _ in edges]
            assert len(set(spans)) == len(spans), (arm, wide, tall)
            partition = build_faces(figure, build_skeleton(figure))
            if partition.outcome is FaceOutcome.EXACT:
                assert partition.area_defect.is_zero, (arm, wide, tall)
                exact += 1
                assert abs(wide - tall) < 2 * arm, (arm, wide, tall)
            else:
                assert partition.outcome is FaceOutcome.FACE_IS_NOT_POSITIVE
                beyond += 1
                assert abs(wide - tall) >= 2 * arm, (arm, wide, tall)
    assert (exact, shared_line, beyond) == {
        2: (51, 0, 30),
        4: (79, 9, 2),
        6: (81, 17, 0),
    }[arm]
